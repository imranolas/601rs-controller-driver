//! The session driver: runs a [`HostCore`] on a real link with threads and the clock.
//!
//! A reader thread feeds the core received bytes; the caller's thread submits commands and
//! waits for them to complete; a [`crate::StopHandle`] stops from any thread. The core decides
//! what every byte means; this file only moves bytes and keeps time.
//!
//! Two locks, always taken in this order:
//! - **The writer lock** is held across "the core decides, then the bytes go out", by commands
//!   and stops alike. A stop sets its latch in the core while holding it, so once a stop has
//!   been written, nothing else can be.
//! - **The state lock** (the core and finished requests) is only ever held briefly. The reader
//!   thread takes only this one, so a write that blocks for its time on the line never delays
//!   what arrives, such as a 0x03 "buffer almost full".

use std::fmt;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use link::{Link, Reader, Writer};
use protocol::{Command, Reply, reply};

use crate::core::{Blocked, HostCore, Output, RequestId};
use crate::error::millis;
use crate::event::{Bus, Event, StopKind};
use crate::{Config, ControllerError, Result};

pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The core and what it has finished, behind the state lock.
struct State {
    core: HostCore,
    /// A finished request, until its caller collects it.
    done: Option<(RequestId, Result<Reply>)>,
}

/// Shared by the session, the reader thread and stop handles.
pub(crate) struct Shared {
    /// Taken before `state`, never after (see the module documentation).
    writer: Mutex<Box<dyn Writer>>,
    state: Mutex<State>,
    /// Signalled whenever the core may have changed: a reply, a signal, a stop, a closed link.
    changed: Condvar,
    shutdown: AtomicBool,
    pub(crate) bus: Bus,
}

impl Shared {
    fn lock(&self) -> MutexGuard<'_, State> {
        lock(&self.state)
    }

    fn wait<'a>(&self, st: MutexGuard<'a, State>, at_most: Duration) -> MutexGuard<'a, State> {
        self.changed
            .wait_timeout(st, at_most)
            .unwrap_or_else(PoisonError::into_inner)
            .0
    }

    /// Pass the core's outputs on: completions to their waiter, events to subscribers.
    fn deliver(&self, st: &mut State, outputs: Vec<Output>) {
        for output in outputs {
            match output {
                Output::Completed { id, result } => st.done = Some((id, result)),
                Output::Event(event) => self.bus.emit(event),
            }
        }
        self.changed.notify_all();
    }

    /// Write bytes the core produced, holding the writer lock but not the state lock. A failed
    /// write means the link is gone: the core is closed, and this fails with the same
    /// `Disconnected` every later operation reports.
    fn write(&self, writer: &mut dyn Writer, bytes: &[u8]) -> Result<()> {
        if let Err(e) = writer.write_all(bytes) {
            let why = e.to_string();
            let mut st = self.lock();
            let outputs = st.core.close(why.clone());
            self.deliver(&mut st, outputs);
            return Err(ControllerError::Disconnected(why));
        }
        Ok(())
    }

    /// The stop lane: latch, then write the stop, under the writer lock.
    pub(crate) fn stop(&self, kind: StopKind) -> Result<()> {
        let mut writer = lock(&self.writer);
        let bytes = {
            let mut st = self.lock();
            let (bytes, outputs) = st.core.stop(kind);
            self.deliver(&mut st, outputs);
            bytes
        };
        self.write(&mut **writer, &bytes)?;
        self.bus.emit(Event::StopIssued(kind));
        Ok(())
    }

    pub(crate) fn is_stopped(&self) -> bool {
        self.lock().core.is_stopped()
    }
}

/// How the core registers a request: [`HostCore::request`] or [`HostCore::request_with_probe`].
type Register =
    fn(&mut HostCore, &Command, Instant) -> std::result::Result<(RequestId, Vec<u8>), Blocked>;

pub(crate) struct Session {
    shared: Arc<Shared>,
    reader: Option<JoinHandle<()>>,
    cfg: Config,
}

impl fmt::Debug for Session {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Session");
        match self.shared.state.try_lock() {
            Ok(st) => s.field("core", &st.core),
            Err(_) => s.field("core", &"<busy>"),
        };
        s.finish_non_exhaustive()
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
            state: Mutex::new(State {
                core: HostCore::new(cfg.reply_timeout),
                done: None,
            }),
            changed: Condvar::new(),
            shutdown: AtomicBool::new(false),
            bus: Bus::default(),
        });
        let reader_shared = Arc::clone(&shared);
        let handle = thread::Builder::new()
            .name(format!("controller-reader ({name})"))
            .spawn(move || reader_loop(reader, &reader_shared))?;
        Ok(Session {
            shared,
            reader: Some(handle),
            cfg,
        })
    }

    pub(crate) fn shared(&self) -> &Arc<Shared> {
        &self.shared
    }

    pub(crate) fn poll_interval(&self) -> Duration {
        self.cfg.poll_interval
    }

    /// The mode the controller last acknowledged.
    pub(crate) fn mode(&self) -> Option<protocol::Mode> {
        self.shared.lock().core.mode()
    }

    // ---------------------------------------------------------------- requests

    /// Send a command that expects a reply and return that reply, resyncing first if pairing
    /// is uncertain.
    pub(crate) fn request(&mut self, command: &Command) -> Result<Reply> {
        self.submit(command, HostCore::request)
    }

    /// Send `command` with a probe right behind it, without waiting between them (see
    /// [`HostCore::request_with_probe`]).
    pub(crate) fn request_with_probe(&mut self, command: &Command) -> Result<Reply> {
        self.submit(command, HostCore::request_with_probe)
    }

    fn submit(&mut self, command: &Command, register: Register) -> Result<Reply> {
        loop {
            let mut writer = lock(&self.shared.writer);
            // The core registers the request just before its bytes go out. A spontaneous line
            // arriving in between is judged exactly as one arriving just after: the controller
            // cannot answer a command it has not received yet.
            let decided = register(&mut self.shared.lock().core, command, Instant::now());
            match decided {
                Ok((id, bytes)) => {
                    self.shared.write(&mut **writer, &bytes)?;
                    drop(writer);
                    return self.wait_for(self.shared.lock(), id);
                }
                Err(Blocked::OutOfSync) => {
                    drop(writer);
                    self.resync()?;
                }
                Err(Blocked::Fails(e)) => return Err(e),
            }
        }
    }

    pub(crate) fn expect_ack(&mut self, command: &Command) -> Result<()> {
        match self.request(command)? {
            Reply::Ack => Ok(()),
            other => Err(self.unexpected(command, &other)),
        }
    }

    /// A reply made no sense for its command: report it, and resync before the next one.
    pub(crate) fn unexpected(&self, command: &Command, reply: &Reply) -> ControllerError {
        self.shared.lock().core.lose_sync();
        ControllerError::UnexpectedReply {
            command: command.clone(),
            reply: reply.clone(),
        }
    }

    /// Block until request `id` completes. The core finishes it with its reply, a timeout
    /// (checked here, at its deadline), a stop or a closed link. If a `*NNN` crosses the
    /// request, the probe that settles it is written from here.
    fn wait_for<'a>(&'a self, mut st: MutexGuard<'a, State>, id: RequestId) -> Result<Reply> {
        loop {
            if let Some((_, result)) = st.done.take_if(|(done, _)| *done == id) {
                return result;
            }
            if st.core.probe_due() {
                // Writes take the writer lock first, so let go of the state lock and take both
                // in order. A stop in between finishes the request, and nothing is written.
                drop(st);
                let mut writer = lock(&self.shared.writer);
                let probe = self.shared.lock().core.settling_probe(Instant::now());
                if let Some(bytes) = probe {
                    self.shared.write(&mut **writer, &bytes)?;
                }
                drop(writer);
                st = self.shared.lock();
                continue;
            }
            let now = Instant::now();
            let outputs = st.core.tick(now);
            if !outputs.is_empty() {
                self.shared.deliver(&mut st, outputs);
                continue;
            }
            let until_deadline = st
                .core
                .next_deadline()
                .map_or(self.cfg.poll_interval, |d| d.saturating_duration_since(now));
            st = self.shared.wait(st, until_deadline);
        }
    }

    /// Re-establish reply pairing (see [`HostCore::resync`]).
    pub(crate) fn resync(&mut self) -> Result<()> {
        let mut writer = lock(&self.shared.writer);
        let (id, bytes) = self.shared.lock().core.resync(Instant::now())?;
        self.shared.write(&mut **writer, &bytes)?;
        drop(writer);
        self.wait_for(self.shared.lock(), id).map(drop)
    }

    /// Release the stop latch after resynchronising.
    pub(crate) fn clear_stop(&mut self) -> Result<()> {
        self.resync()?;
        self.shared.lock().core.clear_stop();
        Ok(())
    }

    /// Report (once) a fault the controller sent while nobody was asking.
    pub(crate) fn take_async_error(&mut self) -> Result<()> {
        match self.shared.lock().core.take_async_error() {
            Some(code) => Err(ControllerError::AsyncError { code }),
            None => Ok(()),
        }
    }

    /// Sleep for `d`, but return at once with an error on a stop or a closed link.
    pub(crate) fn pause(&self, d: Duration) -> Result<()> {
        let deadline = Instant::now() + d;
        let mut st = self.shared.lock();
        loop {
            st.core.usable()?;
            let now = Instant::now();
            if now >= deadline {
                return Ok(());
            }
            st = self.shared.wait(st, deadline - now);
        }
    }

    // ---------------------------------------------------------------- mode 2

    /// Send a mode-2 command (no reply). Blocks while the controller has signalled "buffer
    /// almost full", up to `flow_timeout`.
    pub(crate) fn stream(&mut self, command: &Command) -> Result<()> {
        self.pace_by_buffer_level(command.encode().len())?;
        self.wait_for_buffer_room()?;
        loop {
            let mut writer = lock(&self.shared.writer);
            let decided = self.shared.lock().core.stream(command);
            match decided {
                Ok(bytes) => return self.shared.write(&mut **writer, &bytes),
                Err(Blocked::OutOfSync) => {
                    drop(writer);
                    self.resync()?;
                }
                Err(Blocked::Fails(e)) => return Err(e),
            }
        }
    }

    /// Host-side pacing: the 0x03 warning alone cannot stop a fast host in time (the OS serial
    /// buffer alone can hold more than its 200-byte margin). So after every
    /// `stream_window_bytes` streamed, ask `@H` (a master command, answered even when the
    /// buffer is full) and wait while the buffer is above `stream_high_water_pct`.
    fn pace_by_buffer_level(&mut self, next: usize) -> Result<()> {
        let Some(window) = self.cfg.stream_window_bytes else {
            return Ok(());
        };
        if self.shared.lock().core.streamed_since_check() + next <= window {
            return Ok(());
        }
        let deadline = Instant::now() + self.cfg.flow_timeout;
        loop {
            let reply = self.request(&Command::BufferLevel)?;
            let level = match &reply {
                Reply::Value(v) => reply::parse_buffer_level(v),
                _ => None,
            };
            let Some(level) = level else {
                return Err(self.unexpected(&Command::BufferLevel, &reply));
            };
            if level < self.cfg.stream_high_water_pct {
                self.shared.lock().core.buffer_confirmed();
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(ControllerError::FlowTimeout {
                    timeout_ms: millis(self.cfg.flow_timeout),
                });
            }
            self.pause(self.cfg.poll_interval)?;
        }
    }

    fn wait_for_buffer_room(&mut self) -> Result<()> {
        let deadline = Instant::now() + self.cfg.flow_timeout;
        loop {
            {
                let hold_until = (Instant::now() + self.cfg.poll_interval).min(deadline);
                let mut st = self.shared.lock();
                loop {
                    st.core.usable()?;
                    if !st.core.flow_paused() {
                        return Ok(());
                    }
                    let now = Instant::now();
                    if now >= deadline {
                        return Err(ControllerError::FlowTimeout {
                            timeout_ms: millis(self.cfg.flow_timeout),
                        });
                    }
                    if now >= hold_until {
                        break;
                    }
                    st = self.shared.wait(st, hold_until - now);
                }
            }
            // Still held. The #C19 host watchdog needs a character from us while the axes
            // move [SPEC1 p.26], and master commands are processed even while the stream is
            // held [SPEC1 p.5, p.12], so ask `@H`. It also surfaces faults raised meanwhile.
            self.request(&Command::BufferLevel)?;
            self.take_async_error()?;
        }
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

fn reader_loop(mut reader: Box<dyn Reader>, shared: &Shared) {
    let mut buf = [0u8; 512];
    while !shared.shutdown.load(Ordering::SeqCst) {
        match reader.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                let mut st = shared.lock();
                let outputs = st.core.receive(&buf[..n]);
                shared.deliver(&mut st, outputs);
            }
            Err(e) => {
                let mut st = shared.lock();
                let outputs = st.core.close(e.to_string());
                shared.deliver(&mut st, outputs);
                return;
            }
        }
    }
}
