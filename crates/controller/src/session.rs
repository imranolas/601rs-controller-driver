//! Untyped core of a controller session: reader thread, reply matching, resync, streaming.
//! The typed API in `typed.rs` decides *which* commands may be sent; this decides *how*.

use std::fmt;
use std::io;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use link::{Link, Reader, Writer};
use protocol::{Command, Decoder, ErrorCode, Frame, Reply, Signal, reply};

use crate::event::{Bus, Event};
use crate::{Config, ControllerError, Result};

pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Inputs read as resync probes (`@I<n>`), in rotation. `@I` reads any logical input 0..255
/// [SPEC p.15]; nothing on the 601 uses these (GMC reads 221; the machine maps 229–247
/// [SPEC p.10; SYS]), so a stale reply can never look like the probe's. `gunnar probe` checks
/// that the firmware answers every one of them.
pub const RESYNC_PROBE_INPUTS: std::ops::Range<u8> = 100..120;

/// How many unmatched lines a failed resync reports.
const MAX_HEARD: usize = 8;

fn millis(d: Duration) -> u64 {
    u64::try_from(d.as_millis()).unwrap_or(u64::MAX)
}

/// State shared between the session, the reader thread and stop handles.
pub(crate) struct Shared {
    writer: Mutex<Box<dyn Writer>>,
    /// Latched by a stop; cleared only by `clear_stop`.
    pub(crate) stopped: AtomicBool,
    /// True between 0x03 (almost full) and 0x02 (room again).
    flow_paused: Mutex<bool>,
    flow_changed: Condvar,
    shutdown: AtomicBool,
    /// Bytes received so far, counted by the reader thread (lines, signals and noise alike).
    bytes_received: AtomicU64,
    pub(crate) bus: Bus,
}

impl Shared {
    /// Write one complete command, whatever the stop latch says: for the stop lane itself and
    /// for the resync in `clear_stop`, which runs while latched. The writer lock is held only
    /// for this write, so the stop lane never waits behind more than one line.
    pub(crate) fn write(&self, bytes: &[u8]) -> io::Result<()> {
        lock(&self.writer).write_all(bytes)?;
        self.bus.emit(Event::Sent(bytes.to_vec()));
        Ok(())
    }

    /// Write one complete command from the command path, unless a stop has latched.
    ///
    /// The latch is checked under the writer lock, and a stop sets it before taking that lock.
    /// So either this write goes out before the stop's `@B`/`@S`, or it sees the latch and
    /// sends nothing: no command can follow a stop onto the wire. Checking the latch any
    /// earlier leaves a window in which a stop lands between the check and the write.
    fn write_command(&self, bytes: &[u8]) -> Result<()> {
        let mut writer = lock(&self.writer);
        if self.stopped.load(Ordering::SeqCst) {
            return Err(ControllerError::Stopped);
        }
        writer.write_all(bytes)?;
        drop(writer);
        self.bus.emit(Event::Sent(bytes.to_vec()));
        Ok(())
    }

    fn set_flow_paused(&self, paused: bool) {
        *lock(&self.flow_paused) = paused;
        self.flow_changed.notify_all();
    }

    /// Wake anything blocked on flow control (used by stops).
    pub(crate) fn wake_waiters(&self) {
        let _guard = lock(&self.flow_paused);
        self.flow_changed.notify_all();
    }
}

enum Incoming {
    Line(String),
    Closed(String),
}

pub(crate) struct Session {
    pub(crate) shared: Arc<Shared>,
    rx: Receiver<Incoming>,
    reader: Option<JoinHandle<()>>,
    pub(crate) cfg: Config,
    /// Reply pairing is uncertain; resync before the next command.
    needs_resync: bool,
    /// First unsolicited `*NNN` not yet reported to the caller.
    async_error: Option<ErrorCode>,
    closed: Option<String>,
    /// Resync probes sent so far (selects the probe input).
    probes: u8,
    /// Mode-2 bytes streamed since the buffer level was last confirmed below high water.
    streamed_since_check: usize,
}

impl fmt::Debug for Session {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Session")
            .field("needs_resync", &self.needs_resync)
            .field("async_error", &self.async_error)
            .field("closed", &self.closed)
            .finish_non_exhaustive()
    }
}

impl Session {
    pub(crate) fn start(link: Link, cfg: Config) -> io::Result<Session> {
        let Link {
            reader,
            writer,
            name,
        } = link;
        let shared = Arc::new(Shared {
            writer: Mutex::new(writer),
            stopped: AtomicBool::new(false),
            flow_paused: Mutex::new(false),
            flow_changed: Condvar::new(),
            shutdown: AtomicBool::new(false),
            bytes_received: AtomicU64::new(0),
            bus: Bus::default(),
        });
        let (tx, rx) = mpsc::channel();
        let reader_shared = Arc::clone(&shared);
        let handle = thread::Builder::new()
            .name(format!("controller-reader ({name})"))
            .spawn(move || reader_loop(reader, &reader_shared, &tx))?;
        Ok(Session {
            shared,
            rx,
            reader: Some(handle),
            cfg,
            needs_resync: false,
            async_error: None,
            closed: None,
            probes: 0,
            streamed_since_check: 0,
        })
    }

    // ---------------------------------------------------------------- receive helpers

    fn close(&mut self, why: String) -> ControllerError {
        self.closed = Some(why.clone());
        ControllerError::Disconnected(why)
    }

    /// Next line within `timeout`; `Ok(None)` on timeout.
    fn recv(&mut self, timeout: Duration) -> Result<Option<String>> {
        match self.rx.recv_timeout(timeout) {
            Ok(Incoming::Line(l)) => Ok(Some(l)),
            Ok(Incoming::Closed(why)) => Err(self.close(why)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(self.close("reader stopped".to_owned())),
        }
    }

    /// Record a line nobody asked for. Error codes are kept for the next operation.
    fn unsolicited(&mut self, line: String) {
        if let Reply::Error(code) = Reply::parse(&line) {
            self.async_error.get_or_insert(code);
        }
        self.shared.bus.emit(Event::Unsolicited(line));
    }

    fn drain_unsolicited(&mut self) -> Result<()> {
        while let Some(line) = self.recv(Duration::ZERO)? {
            self.unsolicited(line);
        }
        Ok(())
    }

    /// Report (once) an error the controller sent while nobody was asking.
    pub(crate) fn take_async_error(&mut self) -> Result<()> {
        match self.async_error.take() {
            Some(code) => Err(ControllerError::AsyncError { code }),
            None => Ok(()),
        }
    }

    // ---------------------------------------------------------------- commands

    fn ensure_usable(&mut self) -> Result<()> {
        if let Some(why) = &self.closed {
            return Err(ControllerError::Disconnected(why.clone()));
        }
        if self.shared.stopped.load(Ordering::SeqCst) {
            return Err(ControllerError::Stopped);
        }
        if self.needs_resync {
            self.resync()?;
        }
        Ok(())
    }

    pub(crate) fn unexpected(&mut self, cmd: &Command, reply: &Reply) -> ControllerError {
        self.needs_resync = true;
        ControllerError::UnexpectedReply {
            command: cmd.clone(),
            reply: reply.clone(),
        }
    }

    /// Send a command that expects a reply and return that reply.
    ///
    /// `in_stream`: in mode 2, a `*NNN` may belong to an earlier streamed vector rather than
    /// to this command, so it is recorded as an async error and the wait continues. The same
    /// holds for pure queries ([`Command::is_query`]), which are never rejected: a `*NNN`
    /// during an `@X` poll is a fault that raced with it (found by the simulator).
    pub(crate) fn request(&mut self, cmd: &Command, in_stream: bool) -> Result<Reply> {
        debug_assert!(cmd.expects_reply(), "{cmd} has no reply; use send_streamed");
        self.ensure_usable()?;
        self.drain_unsolicited()?;
        self.shared.write_command(&cmd.encode())?;

        let deadline = Instant::now() + self.cfg.reply_timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let Some(line) = self.recv(remaining)? else {
                self.needs_resync = true;
                return Err(ControllerError::Timeout {
                    command: cmd.clone(),
                    timeout_ms: millis(self.cfg.reply_timeout),
                });
            };
            let reply = Reply::parse(&line);
            if (in_stream || cmd.is_query()) && matches!(reply, Reply::Error(_)) {
                self.unsolicited(line);
                continue;
            }
            if self.shared.stopped.load(Ordering::SeqCst) {
                // A stop raced with this command; this line may be the stop's own ack.
                self.needs_resync = true;
                return Err(ControllerError::Stopped);
            }
            if let Reply::Error(code) = reply {
                return Err(ControllerError::Rejected {
                    command: cmd.clone(),
                    code,
                });
            }
            return Ok(reply);
        }
    }

    pub(crate) fn expect_ack(&mut self, cmd: &Command, in_stream: bool) -> Result<()> {
        match self.request(cmd, in_stream)? {
            Reply::Ack => Ok(()),
            other => Err(self.unexpected(cmd, &other)),
        }
    }

    /// Send a mode-2 command (no reply). Blocks while the controller has signalled
    /// "buffer almost full", up to `flow_timeout`.
    pub(crate) fn send_streamed(&mut self, cmd: &Command) -> Result<()> {
        debug_assert!(!cmd.expects_reply(), "{cmd} expects a reply; use request");
        self.ensure_usable()?;
        self.drain_unsolicited()?;
        self.take_async_error()?;
        let bytes = cmd.encode();
        self.pace_by_buffer_level(bytes.len())?;
        self.wait_for_buffer_room()?;
        self.shared.write_command(&bytes)?;
        self.streamed_since_check += bytes.len();
        Ok(())
    }

    /// Host-side pacing: the 0x03 warning alone cannot stop a fast host in time (the OS serial
    /// buffer alone can hold more than its 200-byte margin). So after every
    /// `stream_window_bytes` streamed, ask `@H` (a master command, answered even when the
    /// buffer is full) and wait while the buffer is above `stream_high_water_pct`.
    fn pace_by_buffer_level(&mut self, next: usize) -> Result<()> {
        let Some(window) = self.cfg.stream_window_bytes else {
            return Ok(());
        };
        if self.streamed_since_check + next <= window {
            return Ok(());
        }
        let deadline = Instant::now() + self.cfg.flow_timeout;
        loop {
            let reply = self.request(&Command::BufferLevel, true)?;
            let level = match &reply {
                Reply::Value(v) => reply::parse_buffer_level(v),
                _ => None,
            };
            let Some(level) = level else {
                return Err(self.unexpected(&Command::BufferLevel, &reply));
            };
            if level < self.cfg.stream_high_water_pct {
                self.streamed_since_check = 0;
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(ControllerError::FlowTimeout {
                    timeout_ms: millis(self.cfg.flow_timeout),
                });
            }
            thread::sleep(self.cfg.poll_interval);
        }
    }

    fn wait_for_buffer_room(&mut self) -> Result<()> {
        let deadline = Instant::now() + self.cfg.flow_timeout;
        loop {
            {
                let paused = lock(&self.shared.flow_paused);
                if !*paused {
                    return Ok(());
                }
                if self.shared.stopped.load(Ordering::SeqCst) {
                    return Err(ControllerError::Stopped);
                }
                let now = Instant::now();
                if now >= deadline {
                    return Err(ControllerError::FlowTimeout {
                        timeout_ms: millis(self.cfg.flow_timeout),
                    });
                }
                let wait = self.cfg.poll_interval.min(deadline - now);
                let (paused, _) = self
                    .shared
                    .flow_changed
                    .wait_timeout(paused, wait)
                    .unwrap_or_else(PoisonError::into_inner);
                if !*paused {
                    return Ok(());
                }
            }
            // Still held. The #C19 host watchdog needs a character from us while the axes
            // move [SPEC1 p.26], and master commands are processed even while the stream is
            // held [SPEC1 p.5, p.12], so ask `@H`. It also surfaces faults raised meanwhile.
            self.request(&Command::BufferLevel, true)?;
            self.take_async_error()?;
        }
    }

    // ---------------------------------------------------------------- recovery

    /// Re-establish reply pairing.
    ///
    /// The controller answers strictly in order, so once the reply to a *unique* probe
    /// arrives, every reply to anything sent earlier has arrived too. The probe reads an input
    /// nothing else uses (`@I100`–`@I119`, rotating), so a stale reply can never be mistaken
    /// for it, unlike a stale `@X04` answering an `@X` probe. Everything before the probe's
    /// reply is discarded (and reported as unsolicited).
    ///
    /// If the probe's reply never comes, the error records what did arrive, so a dead link,
    /// a wrong baud rate and a controller that refuses the probe can be told apart.
    pub(crate) fn resync(&mut self) -> Result<()> {
        if let Some(why) = &self.closed {
            return Err(ControllerError::Disconnected(why.clone()));
        }
        let span = RESYNC_PROBE_INPUTS.end - RESYNC_PROBE_INPUTS.start;
        let input = RESYNC_PROBE_INPUTS.start + self.probes % span;
        self.probes = self.probes.wrapping_add(1);
        let probe = Command::ReadInput(input);
        let bytes_before = self.shared.bytes_received.load(Ordering::SeqCst);
        self.shared.write(&probe.encode())?;

        let mut lines_heard = Vec::new();
        let deadline = Instant::now() + self.cfg.reply_timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let Some(line) = self.recv(remaining)? else {
                self.needs_resync = true;
                let bytes_now = self.shared.bytes_received.load(Ordering::SeqCst);
                return Err(ControllerError::ResyncFailed {
                    probe,
                    timeout_ms: millis(self.cfg.reply_timeout),
                    bytes_heard: bytes_now.saturating_sub(bytes_before),
                    lines_heard,
                });
            };
            if reply::parse_input(&line, input).is_some() {
                self.needs_resync = false;
                self.streamed_since_check = 0;
                return Ok(());
            }
            if lines_heard.len() < MAX_HEARD {
                lines_heard.push(line.clone());
            }
            self.unsolicited(line);
        }
    }

    /// Release the stop latch after resynchronising. Errors that arrived around the stop
    /// have already been delivered as `Event::Unsolicited` and are not reported again.
    pub(crate) fn clear_stop(&mut self) -> Result<()> {
        self.resync()?;
        self.async_error = None;
        self.shared.stopped.store(false, Ordering::SeqCst);
        Ok(())
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, Ordering::SeqCst);
        if let Some(handle) = self.reader.take() {
            let _ = handle.join();
        }
    }
}

fn reader_loop(mut reader: Box<dyn Reader>, shared: &Shared, tx: &Sender<Incoming>) {
    let mut decoder = Decoder::new();
    let mut buf = [0u8; 512];
    while !shared.shutdown.load(Ordering::SeqCst) {
        let n = match reader.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(e) => {
                let why = e.to_string();
                let _ = tx.send(Incoming::Closed(why.clone()));
                shared.bus.emit(Event::Disconnected(why));
                return;
            }
        };
        shared
            .bytes_received
            .fetch_add(u64::try_from(n).unwrap_or(u64::MAX), Ordering::SeqCst);
        shared.bus.emit(Event::Received(buf[..n].to_vec()));
        for frame in decoder.push(&buf[..n]) {
            match frame {
                Frame::Line(line) => {
                    if tx.send(Incoming::Line(line)).is_err() {
                        return; // session gone
                    }
                }
                Frame::Signal(signal) => {
                    match signal {
                        Signal::BufferAlmostFull => shared.set_flow_paused(true),
                        Signal::BufferResume => shared.set_flow_paused(false),
                        _ => {}
                    }
                    shared.bus.emit(Event::Signal(signal));
                }
                Frame::Unknown(byte) => shared.bus.emit(Event::UnknownByte(byte)),
                Frame::Garbled(bytes) => shared.bus.emit(Event::Garbled(bytes)),
            }
        }
    }
}
