//! The host side of the protocol, pure: bytes and time in, bytes and outcomes out.
//!
//! [`HostCore`] mirrors the simulator's `SimCore`. It decides what every byte *means* (which
//! reply belongs to which command, when pairing is lost and how it is regained, the stop latch,
//! flow control, faults nobody asked about) and touches nothing: no threads, no clock, no I/O.
//! A driver feeds it received bytes and the current time and writes the bytes it returns. The
//! session in `session.rs` is one such driver; a test can be another, wiring it straight to the
//! simulator on a virtual clock.
//!
//! The rules it keeps:
//! - **One outstanding request.** The controller answers strictly in order, so a reply pairs
//!   with the one command waiting for it.
//! - **Resync when pairing is uncertain** (a fresh core, a timeout, a reply that made no sense,
//!   a stop): requests are refused with [`Blocked::OutOfSync`] until a unique probe has made a
//!   round trip, and everything that arrives before its reply is discarded.
//! - **A stop latches.** It finishes any outstanding request as `Stopped` and refuses every
//!   send until [`HostCore::clear_stop`]. A driver writes the stop's bytes under the same lock as
//!   every other write, so nothing can follow a stop onto the wire.
//! - **Faults are kept.** A `*NNN` that cannot be a reply is reported once, by the next
//!   operation that asks, and blocks streaming until it has been.

use std::time::{Duration, Instant};

use protocol::{Command, Decoder, ErrorCode, Frame, Mode, Reply, Signal, reply};

use crate::error::millis;
use crate::{ControllerError, Event, Result, StopKind};

/// Inputs read as resync probes (`@I<n>`), in rotation. `@I` reads any logical input 0..255
/// [SPEC p.15]; nothing on the 601 uses these (GMC reads 221; the machine maps 229–247
/// [SPEC p.10; SYS]), so a stale reply can never look like the probe's. `gunnar probe` checks
/// that the firmware answers every one of them.
pub const RESYNC_PROBE_INPUTS: std::ops::Range<u8> = 100..120;

/// How many unmatched lines a failed resync reports.
const MAX_HEARD: usize = 8;

/// Names a request until it completes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RequestId(u64);

/// What the core makes of received bytes or passing time.
#[derive(Debug)]
pub enum Output {
    /// The outstanding request (or resync probe) has finished.
    Completed {
        id: RequestId,
        result: Result<Reply>,
    },
    /// Something for subscribers.
    Event(Event),
}

/// Why the core will not send a command now.
#[derive(Debug)]
pub enum Blocked {
    /// Pairing is uncertain: run [`HostCore::resync`] to completion, then submit again.
    OutOfSync,
    /// The operation fails with this error.
    Fails(ControllerError),
}

#[derive(Debug)]
enum Pending {
    Request {
        id: RequestId,
        command: Command,
        deadline: Instant,
    },
    Probe {
        id: RequestId,
        input: u8,
        deadline: Instant,
        /// [`HostCore::bytes_received`] when the probe went out.
        bytes_at_start: u64,
        lines_heard: Vec<String>,
    },
}

impl Pending {
    fn id(&self) -> RequestId {
        match self {
            Pending::Request { id, .. } | Pending::Probe { id, .. } => *id,
        }
    }

    fn deadline(&self) -> Instant {
        match self {
            Pending::Request { deadline, .. } | Pending::Probe { deadline, .. } => *deadline,
        }
    }
}

/// Can a `*NNN` that arrives while `command` is outstanding be its reply?
///
/// Not for a pure query: those are never rejected, so the `*NNN` is a fault that raced the poll.
/// Not in mode 2 either, where it may belong to a command streamed earlier, except for the mode
/// switch, which is answered directly. For every other command the `*NNN` is taken as its
/// rejection.
fn error_may_answer(command: &Command, mode: Option<Mode>) -> bool {
    if command.is_query() {
        return false;
    }
    mode != Some(Mode::Cutting) || matches!(command, Command::SetMode(_))
}

/// The host side of a session with the controller. See the module documentation.
#[derive(Debug)]
pub struct HostCore {
    reply_timeout: Duration,
    decoder: Decoder,
    pending: Option<Pending>,
    next_id: u64,
    /// Every earlier reply is known to have arrived.
    in_sync: bool,
    stopped: bool,
    closed: Option<String>,
    /// The mode the controller last acknowledged switching to.
    mode: Option<Mode>,
    /// Between 0x03 (buffer almost full) and 0x02 (room again).
    flow_paused: bool,
    /// The first `*NNN` that could not be a reply, until it is reported.
    async_error: Option<ErrorCode>,
    /// Resync probes sent so far (selects the probe input).
    probes: u8,
    bytes_received: u64,
    /// Mode-2 bytes streamed since the buffer level was last confirmed below high water.
    streamed_since_check: usize,
}

impl HostCore {
    /// A core that waits `reply_timeout` for each reply. It starts out of sync: the first
    /// thing on the wire is a resync probe.
    pub fn new(reply_timeout: Duration) -> Self {
        Self {
            reply_timeout,
            decoder: Decoder::new(),
            pending: None,
            next_id: 0,
            in_sync: false,
            stopped: false,
            closed: None,
            mode: None,
            flow_paused: false,
            async_error: None,
            probes: 0,
            bytes_received: 0,
            streamed_since_check: 0,
        }
    }

    fn next_id(&mut self) -> RequestId {
        self.next_id += 1;
        RequestId(self.next_id)
    }

    /// `Ok` unless the link is closed or a stop has latched.
    pub fn usable(&self) -> Result<()> {
        if let Some(why) = &self.closed {
            return Err(ControllerError::Disconnected(why.clone()));
        }
        if self.stopped {
            return Err(ControllerError::Stopped);
        }
        Ok(())
    }

    fn ready(&mut self) -> std::result::Result<(), Blocked> {
        self.usable().map_err(Blocked::Fails)?;
        if self.pending.take().is_some() {
            // A reply is still due for an abandoned request: it would pair with this one.
            self.in_sync = false;
        }
        if self.in_sync {
            Ok(())
        } else {
            Err(Blocked::OutOfSync)
        }
    }

    // ------------------------------------------------------------------ sending

    /// Submit a command that expects a reply. On `Ok`, write the bytes; the reply arrives as
    /// [`Output::Completed`] with the returned id, from [`Self::receive`] or [`Self::tick`].
    pub fn request(
        &mut self,
        command: &Command,
        now: Instant,
    ) -> std::result::Result<(RequestId, Vec<u8>), Blocked> {
        debug_assert!(
            command.expects_reply(),
            "{command} has no reply; use stream"
        );
        self.ready()?;
        let id = self.next_id();
        self.pending = Some(Pending::Request {
            id,
            command: command.clone(),
            deadline: now + self.reply_timeout,
        });
        Ok((id, command.encode()))
    }

    /// Submit a mode-2 command, which gets no reply. Refused while a fault is unreported: the
    /// command would only queue up behind an error the caller has not seen.
    pub fn stream(&mut self, command: &Command) -> std::result::Result<Vec<u8>, Blocked> {
        debug_assert!(
            !command.expects_reply(),
            "{command} expects a reply; use request"
        );
        self.ready()?;
        if let Some(code) = self.async_error.take() {
            return Err(Blocked::Fails(ControllerError::AsyncError { code }));
        }
        let bytes = command.encode();
        self.streamed_since_check += bytes.len();
        Ok(bytes)
    }

    /// Start re-establishing reply pairing: write the returned probe and wait for its
    /// completion. Allowed while a stop is latched (that is how a stop is cleared).
    ///
    /// The probe reads an input nothing else uses, rotating through
    /// [`RESYNC_PROBE_INPUTS`], so a stale reply can never pass for its answer. Everything that
    /// arrives before that answer is discarded as unsolicited. If it never comes, the error
    /// records what did arrive, so a dead link, a wrong baud rate and a controller that refuses
    /// the probe can be told apart.
    pub fn resync(&mut self, now: Instant) -> Result<(RequestId, Vec<u8>)> {
        if let Some(why) = &self.closed {
            return Err(ControllerError::Disconnected(why.clone()));
        }
        let span = RESYNC_PROBE_INPUTS.end - RESYNC_PROBE_INPUTS.start;
        let input = RESYNC_PROBE_INPUTS.start + self.probes % span;
        self.probes = self.probes.wrapping_add(1);
        let id = self.next_id();
        self.in_sync = false;
        self.pending = Some(Pending::Probe {
            id,
            input,
            deadline: now + self.reply_timeout,
            bytes_at_start: self.bytes_received,
            lines_heard: Vec::new(),
        });
        Ok((id, Command::ReadInput(input).encode()))
    }

    /// The stop lane: latch, and finish any outstanding request as `Stopped`. Its reply and the
    /// stop's own acknowledgement are both still on their way, so pairing is re-established
    /// before anything else is sent. Returns the stop command, to be written at once.
    pub fn stop(&mut self, kind: StopKind) -> (Vec<u8>, Vec<Output>) {
        self.stopped = true;
        self.in_sync = false;
        let finished = self.pending.take().map(|p| Output::Completed {
            id: p.id(),
            result: Err(ControllerError::Stopped),
        });
        let command = match kind {
            StopKind::Brake => Command::Brake,
            StopKind::Emergency => Command::EmergencyStop,
        };
        (command.encode(), finished.into_iter().collect())
    }

    /// Release the stop latch, once [`Self::resync`] has completed. Faults that arrived around
    /// the stop have already gone out as events and are not reported again.
    pub fn clear_stop(&mut self) {
        self.stopped = false;
        self.async_error = None;
    }

    // ------------------------------------------------------------------ receiving

    /// Bytes from the controller, and what they mean.
    pub fn receive(&mut self, bytes: &[u8]) -> Vec<Output> {
        self.bytes_received += u64::try_from(bytes.len()).unwrap_or(u64::MAX);
        let mut out = Vec::new();
        for frame in self.decoder.push(bytes) {
            match frame {
                Frame::Line(line) => self.line(line, &mut out),
                Frame::Signal(signal) => {
                    match signal {
                        Signal::BufferAlmostFull => self.flow_paused = true,
                        Signal::BufferResume => self.flow_paused = false,
                        _ => {}
                    }
                    out.push(Output::Event(Event::Signal(signal)));
                }
                Frame::Unknown(byte) => out.push(Output::Event(Event::UnknownByte(byte))),
                Frame::Garbled(bytes) => out.push(Output::Event(Event::Garbled(bytes))),
            }
        }
        out
    }

    fn line(&mut self, line: String, out: &mut Vec<Output>) {
        let reply = Reply::parse(&line);
        let answers = match &mut self.pending {
            None => false,
            Some(Pending::Probe {
                input, lines_heard, ..
            }) => {
                let answers = reply::parse_input(&line, *input).is_some();
                if !answers && lines_heard.len() < MAX_HEARD {
                    lines_heard.push(line.clone());
                }
                answers
            }
            Some(Pending::Request { command, .. }) => {
                !matches!(reply, Reply::Error(_)) || error_may_answer(command, self.mode)
            }
        };
        if !answers {
            if let Reply::Error(code) = reply {
                self.async_error.get_or_insert(code);
            }
            out.push(Output::Event(Event::Unsolicited(line)));
            return;
        }
        match self.pending.take() {
            Some(Pending::Probe { id, .. }) => {
                self.in_sync = true;
                self.streamed_since_check = 0;
                out.push(Output::Completed {
                    id,
                    result: Ok(reply),
                });
            }
            Some(Pending::Request { id, command, .. }) => {
                let result = match reply {
                    Reply::Error(code) => Err(ControllerError::Rejected { command, code }),
                    reply => {
                        if let (Command::SetMode(mode), Reply::Ack) = (&command, &reply) {
                            self.mode = Some(*mode);
                        }
                        Ok(reply)
                    }
                };
                out.push(Output::Completed { id, result });
            }
            None => {}
        }
    }

    /// Time has passed: a request whose deadline has gone finishes with an error.
    pub fn tick(&mut self, now: Instant) -> Vec<Output> {
        if !self.pending.as_ref().is_some_and(|p| now >= p.deadline()) {
            return Vec::new();
        }
        let timeout_ms = millis(self.reply_timeout);
        let finished = match self.pending.take() {
            Some(Pending::Request { id, command, .. }) => {
                self.in_sync = false;
                Output::Completed {
                    id,
                    result: Err(ControllerError::Timeout {
                        command,
                        timeout_ms,
                    }),
                }
            }
            Some(Pending::Probe {
                id,
                input,
                bytes_at_start,
                lines_heard,
                ..
            }) => Output::Completed {
                id,
                result: Err(ControllerError::ResyncFailed {
                    probe: Command::ReadInput(input),
                    timeout_ms,
                    bytes_heard: self.bytes_received - bytes_at_start,
                    lines_heard,
                }),
            },
            None => return Vec::new(),
        };
        vec![finished]
    }

    /// The link is gone; an outstanding request finishes as `Disconnected`.
    pub fn close(&mut self, why: String) -> Vec<Output> {
        self.closed.get_or_insert_with(|| why.clone());
        self.pending
            .take()
            .map(|p| Output::Completed {
                id: p.id(),
                result: Err(ControllerError::Disconnected(why)),
            })
            .into_iter()
            .collect()
    }

    // ------------------------------------------------------------------ state

    /// A reply arrived that made no sense for its command: pairing is no longer certain.
    pub fn lose_sync(&mut self) {
        self.in_sync = false;
    }

    /// Report (once) a fault the controller sent while nobody was asking.
    pub fn take_async_error(&mut self) -> Option<ErrorCode> {
        self.async_error.take()
    }

    /// When the outstanding request runs out of time, if there is one.
    pub fn next_deadline(&self) -> Option<Instant> {
        self.pending.as_ref().map(Pending::deadline)
    }

    pub fn is_stopped(&self) -> bool {
        self.stopped
    }

    pub fn is_in_sync(&self) -> bool {
        self.in_sync
    }

    /// The mode the controller last acknowledged switching to.
    pub fn mode(&self) -> Option<Mode> {
        self.mode
    }

    /// The controller has signalled "buffer almost full" (0x03) and not yet "room" (0x02).
    pub fn flow_paused(&self) -> bool {
        self.flow_paused
    }

    /// Mode-2 bytes streamed since the buffer level was last confirmed below high water.
    pub fn streamed_since_check(&self) -> usize {
        self.streamed_since_check
    }

    /// The buffer level was just confirmed below high water.
    pub fn buffer_confirmed(&mut self) {
        self.streamed_since_check = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn completed(out: &[Output]) -> Option<(RequestId, &Result<Reply>)> {
        out.iter().find_map(|o| match o {
            Output::Completed { id, result } => Some((*id, result)),
            Output::Event(_) => None,
        })
    }

    /// A core that has made its first round trip.
    fn synced() -> (HostCore, Instant) {
        let now = Instant::now();
        let mut core = HostCore::new(Duration::from_millis(200));
        let (_, probe) = core.resync(now).expect("resync");
        assert_eq!(probe, b"@I100\r");
        core.receive(b"@I100,0\r");
        assert!(core.is_in_sync());
        (core, now)
    }

    #[test]
    fn a_new_core_resyncs_first() {
        let mut core = HostCore::new(Duration::from_millis(200));
        assert!(matches!(
            core.request(&Command::Status, Instant::now()),
            Err(Blocked::OutOfSync)
        ));
    }

    #[test]
    fn replies_pair_in_order_and_the_mode_follows_acknowledged_switches() {
        let (mut core, now) = synced();
        let (id, bytes) = core
            .request(&Command::SetMode(Mode::Cutting), now)
            .expect("send");
        assert_eq!(bytes, b"@M2\r");
        let out = core.receive(b"\r");
        assert_eq!(completed(&out).map(|(i, _)| i), Some(id));
        assert_eq!(core.mode(), Some(Mode::Cutting));
    }

    #[test]
    fn a_rejection_answers_a_command_but_not_a_query() {
        let (mut core, now) = synced();
        let (_, _) = core.request(&Command::ClearError, now).expect("send");
        let out = core.receive(b"*021\r");
        assert!(matches!(
            completed(&out),
            Some((_, Err(ControllerError::Rejected { .. })))
        ));

        let (id, _) = core.request(&Command::Status, now).expect("send");
        let out = core.receive(b"*033\r");
        assert!(completed(&out).is_none(), "a query is never rejected");
        let out = core.receive(b"@X06\r");
        assert_eq!(completed(&out).map(|(i, _)| i), Some(id));
        assert_eq!(core.take_async_error(), Some(ErrorCode(33)));
    }

    #[test]
    fn a_timeout_forces_a_resync() {
        let (mut core, now) = synced();
        let (id, _) = core.request(&Command::Status, now).expect("send");
        assert!(core.tick(now + Duration::from_millis(199)).is_empty());
        let out = core.tick(now + Duration::from_millis(200));
        assert!(matches!(
            completed(&out),
            Some((i, Err(ControllerError::Timeout { .. }))) if i == id
        ));
        assert!(matches!(
            core.request(&Command::Status, now),
            Err(Blocked::OutOfSync)
        ));
    }

    #[test]
    fn a_stop_finishes_the_request_and_blocks_until_cleared() {
        let (mut core, now) = synced();
        let (id, _) = core.request(&Command::Status, now).expect("send");
        let (bytes, out) = core.stop(StopKind::Brake);
        assert_eq!(bytes, b"@B\r");
        assert!(matches!(
            completed(&out),
            Some((i, Err(ControllerError::Stopped))) if i == id
        ));
        assert!(matches!(
            core.request(&Command::Status, now),
            Err(Blocked::Fails(ControllerError::Stopped))
        ));
        let (_, probe) = core.resync(now).expect("resync while stopped");
        assert_eq!(probe, b"@I101\r");
        // The late @X reply and the @B acknowledgement come first, then the probe's answer.
        core.receive(b"@X14\r\r\x01@I101,0\r");
        core.clear_stop();
        assert!(core.request(&Command::Status, now).is_ok());
    }

    #[test]
    fn streaming_waits_for_an_unreported_fault() {
        let (mut core, _) = synced();
        core.receive(b"*022\r");
        let vector = Command::Vector {
            speed_entry: 20,
            x: 0,
            y: 0,
            z: 0,
        };
        assert!(matches!(
            core.stream(&vector),
            Err(Blocked::Fails(ControllerError::AsyncError {
                code: ErrorCode(22)
            }))
        ));
        assert!(core.stream(&vector).is_ok(), "reported once");
    }
}
