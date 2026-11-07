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
//! - **A `*NNN` is settled, not guessed.** It may be the waiting command's rejection or a fault
//!   that crossed it on the wire, and nothing in the line says which. So a probe goes out right
//!   behind it: if the command's own answer arrives before the probe's, the `*NNN` was a fault;
//!   if not, it was the rejection.
//! - **Resync when pairing is uncertain** (a fresh core, a timeout, a reply that made no sense,
//!   a stop): requests are refused with [`Blocked::OutOfSync`] until a unique probe has made a
//!   round trip, and everything that arrives before its reply is discarded.
//! - **A stop latches.** It finishes any outstanding request as `Stopped` and refuses every
//!   send until [`HostCore::clear_stop`]. A driver writes the stop's bytes under the same lock as
//!   every other write, so nothing can follow a stop onto the wire.
//! - **Faults are kept.** A `*NNN` that is not a reply is reported once, by the next operation
//!   that asks. Until then nothing that moves the machine or changes its outputs, settings or
//!   mode is sent: such a command fails with the fault instead.

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
    /// A `*NNN` arrived while `command` was waiting for its reply: its rejection, or a fault
    /// that crossed it. The probe written behind it (see [`HostCore::settling_probe`]) is
    /// answered after the command, so once the probe's answer is in, so is the command's.
    /// Also the state of [`HostCore::request_with_probe`], with the probe written at once.
    Settling {
        id: RequestId,
        command: Command,
        /// The first `*NNN`, as received.
        first: Option<(ErrorCode, String)>,
        /// Any more `*NNN` before the probe's answer: faults.
        more: Vec<(ErrorCode, String)>,
        /// The first line that can only be a reply (CR or a value): the command's own answer.
        answer: Option<Reply>,
        /// The probe's input, once the probe has been written.
        probe: Option<u8>,
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
            Pending::Request { id, .. }
            | Pending::Settling { id, .. }
            | Pending::Probe { id, .. } => *id,
        }
    }

    fn deadline(&self) -> Instant {
        match self {
            Pending::Request { deadline, .. }
            | Pending::Settling { deadline, .. }
            | Pending::Probe { deadline, .. } => *deadline,
        }
    }
}

/// Does `command` wait for an unreported fault to be seen? Everything that moves the machine or
/// changes its outputs, settings or mode does. Reading state, braking and clearing the error go
/// through, so a caller can always look and make safe.
fn held_by_fault(command: &Command) -> bool {
    use Command::*;
    !(command.is_query()
        || matches!(
            command,
            ReadInput(_) | ReadOutput(_) | QueryParam(_) | Brake | EmergencyStop | ClearError
        ))
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
    /// The first `*NNN` that was not a reply, until it is reported.
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
    /// While [`Self::probe_due`], write [`Self::settling_probe`] too.
    ///
    /// Refused with the fault while one is unreported, if the command would move the machine
    /// or change its outputs, settings or mode.
    pub fn request(
        &mut self,
        command: &Command,
        now: Instant,
    ) -> std::result::Result<(RequestId, Vec<u8>), Blocked> {
        let id = self.admit(command)?;
        self.pending = Some(Pending::Request {
            id,
            command: command.clone(),
            deadline: now + self.reply_timeout,
        });
        Ok((id, command.encode()))
    }

    /// Submit `command` with a probe right behind it, in one write, without waiting for its
    /// reply: the reply-order check (`gunnar check`). Settling a `*NNN` and every resync rely
    /// on the controller answering in the order it is sent. It completes like
    /// [`Self::request`], unless the probe's answer comes first: then with
    /// [`ControllerError::Overtaken`], and pairing is re-established before the next request.
    pub fn request_with_probe(
        &mut self,
        command: &Command,
        now: Instant,
    ) -> std::result::Result<(RequestId, Vec<u8>), Blocked> {
        let id = self.admit(command)?;
        let input = self.next_probe_input();
        self.pending = Some(Pending::Settling {
            id,
            command: command.clone(),
            first: None,
            more: Vec::new(),
            answer: None,
            probe: Some(input),
            deadline: now + self.reply_timeout,
        });
        let mut bytes = command.encode();
        bytes.extend(Command::ReadInput(input).encode());
        Ok((id, bytes))
    }

    /// May `command` be sent now? Its id if so.
    fn admit(&mut self, command: &Command) -> std::result::Result<RequestId, Blocked> {
        debug_assert!(
            command.expects_reply(),
            "{command} has no reply; use stream"
        );
        self.ready()?;
        if held_by_fault(command) {
            if let Some(code) = self.async_error.take() {
                return Err(Blocked::Fails(ControllerError::AsyncError { code }));
            }
        }
        Ok(self.next_id())
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
        let input = self.next_probe_input();
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
        let mut out = Vec::new();
        if let Some(pending) = self.pending.take() {
            self.abandon(pending, ControllerError::Stopped, &mut out);
        }
        let command = match kind {
            StopKind::Brake => Command::Brake,
            StopKind::Emergency => Command::EmergencyStop,
        };
        (command.encode(), out)
    }

    /// A `*NNN` is waiting to be settled and its probe has not been written yet.
    pub fn probe_due(&self) -> bool {
        self.usable().is_ok() && matches!(self.pending, Some(Pending::Settling { probe: None, .. }))
    }

    /// The probe that settles a `*NNN` which arrived while a command was waiting for its reply
    /// (see the module documentation), or `None` if none is due. Write it at once, under the
    /// same lock as every other write. It reads an input nothing else uses, like a resync
    /// probe, and the request's deadline restarts from `now`.
    pub fn settling_probe(&mut self, now: Instant) -> Option<Vec<u8>> {
        if !self.probe_due() {
            return None;
        }
        let input = self.next_probe_input();
        let timeout = self.reply_timeout;
        let Some(Pending::Settling {
            probe, deadline, ..
        }) = &mut self.pending
        else {
            return None;
        };
        *probe = Some(input);
        *deadline = now + timeout;
        Some(Command::ReadInput(input).encode())
    }

    /// The next input in the probe rotation ([`RESYNC_PROBE_INPUTS`]).
    fn next_probe_input(&mut self) -> u8 {
        let span = RESYNC_PROBE_INPUTS.end - RESYNC_PROBE_INPUTS.start;
        let input = RESYNC_PROBE_INPUTS.start + self.probes % span;
        self.probes = self.probes.wrapping_add(1);
        input
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
        let code = match reply {
            Reply::Error(code) => Some(code),
            _ => None,
        };
        match self.pending.take() {
            None => self.unsolicited(line, code, out),
            Some(Pending::Probe {
                id,
                input,
                deadline,
                bytes_at_start,
                mut lines_heard,
            }) => {
                if reply::parse_input(&line, input).is_some() {
                    self.in_sync = true;
                    self.streamed_since_check = 0;
                    out.push(Output::Completed {
                        id,
                        result: Ok(reply),
                    });
                } else {
                    if lines_heard.len() < MAX_HEARD {
                        lines_heard.push(line.clone());
                    }
                    self.pending = Some(Pending::Probe {
                        id,
                        input,
                        deadline,
                        bytes_at_start,
                        lines_heard,
                    });
                    self.unsolicited(line, code, out);
                }
            }
            Some(Pending::Request {
                id,
                command,
                deadline,
            }) => match code {
                Some(code) => {
                    self.pending = Some(Pending::Settling {
                        id,
                        command,
                        first: Some((code, line)),
                        more: Vec::new(),
                        answer: None,
                        probe: None,
                        deadline,
                    });
                }
                None => self.complete(id, command, Ok(reply), out),
            },
            Some(Pending::Settling {
                id,
                command,
                mut first,
                mut more,
                mut answer,
                probe,
                deadline,
            }) => {
                if probe.is_some_and(|input| reply::parse_input(&line, input).is_some()) {
                    self.settle(id, command, first, more, answer, out);
                    return;
                }
                match code {
                    Some(code) if first.is_none() => first = Some((code, line)),
                    Some(code) => more.push((code, line)),
                    None if answer.is_none() => answer = Some(reply),
                    None => self.unsolicited(line, None, out),
                }
                self.pending = Some(Pending::Settling {
                    id,
                    command,
                    first,
                    more,
                    answer,
                    probe,
                    deadline,
                });
            }
        }
    }

    /// A command's reply has come.
    fn complete(
        &mut self,
        id: RequestId,
        command: Command,
        result: Result<Reply>,
        out: &mut Vec<Output>,
    ) {
        if let (Command::SetMode(mode), Ok(Reply::Ack)) = (&command, &result) {
            self.mode = Some(*mode);
        }
        out.push(Output::Completed { id, result });
    }

    /// The settling probe has its answer, so the command's has come too, if it was going to.
    /// A CR or value among the lines is the command's answer and every `*NNN` was a fault.
    /// Without one, the first `*NNN` was the rejection. (If two cross a command that is
    /// rejected, which is which can swap; that it was rejected cannot.)
    ///
    /// With neither, the probe was answered before the command: replies are not in order, and
    /// the command's reply is still to come, so pairing must be re-established.
    fn settle(
        &mut self,
        id: RequestId,
        command: Command,
        first: Option<(ErrorCode, String)>,
        more: Vec<(ErrorCode, String)>,
        answer: Option<Reply>,
        out: &mut Vec<Output>,
    ) {
        let result = match (answer, first) {
            (Some(reply), first) => {
                if let Some((code, line)) = first {
                    self.unsolicited(line, Some(code), out);
                }
                Ok(reply)
            }
            (None, Some((code, _))) => Err(ControllerError::Rejected {
                command: command.clone(),
                code,
            }),
            (None, None) => {
                self.in_sync = false;
                Err(ControllerError::Overtaken {
                    command: command.clone(),
                })
            }
        };
        for (code, line) in more {
            self.unsolicited(line, Some(code), out);
        }
        self.complete(id, command, result, out);
    }

    /// A line no command was waiting for. A `*NNN` is a fault, kept until it is reported.
    fn unsolicited(&mut self, line: String, code: Option<ErrorCode>, out: &mut Vec<Output>) {
        if let Some(code) = code {
            self.async_error.get_or_insert(code);
        }
        out.push(Output::Event(Event::Unsolicited(line)));
    }

    /// Finish `pending` with `error`. The `*NNN` a settling request held were never settled,
    /// so they are reported as faults: reported twice is better than lost.
    fn abandon(&mut self, pending: Pending, error: ControllerError, out: &mut Vec<Output>) {
        let id = pending.id();
        if let Pending::Settling { first, more, .. } = pending {
            for (code, line) in first.into_iter().chain(more) {
                self.unsolicited(line, Some(code), out);
            }
        }
        out.push(Output::Completed {
            id,
            result: Err(error),
        });
    }

    /// Time has passed: a request whose deadline has gone finishes with an error.
    pub fn tick(&mut self, now: Instant) -> Vec<Output> {
        if !self.pending.as_ref().is_some_and(|p| now >= p.deadline()) {
            return Vec::new();
        }
        let Some(pending) = self.pending.take() else {
            return Vec::new();
        };
        let timeout_ms = millis(self.reply_timeout);
        let error = match &pending {
            Pending::Request { command, .. } | Pending::Settling { command, .. } => {
                self.in_sync = false;
                ControllerError::Timeout {
                    command: command.clone(),
                    timeout_ms,
                }
            }
            Pending::Probe {
                input,
                bytes_at_start,
                lines_heard,
                ..
            } => ControllerError::ResyncFailed {
                probe: Command::ReadInput(*input),
                timeout_ms,
                bytes_heard: self.bytes_received - bytes_at_start,
                lines_heard: lines_heard.clone(),
            },
        };
        let mut out = Vec::new();
        self.abandon(pending, error, &mut out);
        out
    }

    /// The link is gone; an outstanding request finishes as `Disconnected`.
    pub fn close(&mut self, why: String) -> Vec<Output> {
        self.closed.get_or_insert_with(|| why.clone());
        let mut out = Vec::new();
        if let Some(pending) = self.pending.take() {
            self.abandon(pending, ControllerError::Disconnected(why), &mut out);
        }
        out
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

    /// A `*NNN` with a command waiting: nothing completes until the probe written behind it
    /// has its answer.
    fn crossed(core: &mut HostCore, command: &Command, error: &[u8], now: Instant) -> RequestId {
        let (id, _) = core.request(command, now).expect("send");
        let out = core.receive(error);
        assert!(completed(&out).is_none(), "not settled yet");
        assert!(core.probe_due());
        assert_eq!(core.settling_probe(now).as_deref(), Some(&b"@I101\r"[..]));
        assert!(!core.probe_due(), "one probe");
        id
    }

    #[test]
    fn a_rejection_is_settled_by_the_probe() {
        let (mut core, now) = synced();
        let id = crossed(&mut core, &Command::ClearError, b"*021\r", now);
        let out = core.receive(b"@I101,0\r");
        assert!(matches!(
            completed(&out),
            Some((i, Err(ControllerError::Rejected { code: ErrorCode(21), .. }))) if i == id
        ));
        assert_eq!(core.take_async_error(), None, "a rejection is not a fault");
    }

    /// What the runner met: a limit fault arrives just before a mode switch's acknowledgement.
    /// Taken as the switch's rejection, it left the acknowledgement to pair with the next
    /// command and the host believing the old mode.
    #[test]
    fn a_fault_crossing_a_mode_switch_is_a_fault_and_the_switch_stands() {
        let (mut core, now) = synced();
        let id = crossed(
            &mut core,
            &Command::SetMode(Mode::Parameters),
            b"*033\r",
            now,
        );
        let out = core.receive(b"\r@I101,0\r");
        assert!(matches!(completed(&out), Some((i, Ok(Reply::Ack))) if i == id));
        assert_eq!(core.mode(), Some(Mode::Parameters));
        assert!(
            out.iter()
                .any(|o| matches!(o, Output::Event(Event::Unsolicited(l)) if l == "*033"))
        );
        assert_eq!(core.take_async_error(), Some(ErrorCode(33)));
    }

    #[test]
    fn a_fault_racing_a_query_leaves_it_its_answer() {
        let (mut core, now) = synced();
        let id = crossed(&mut core, &Command::Status, b"*033\r", now);
        assert!(
            completed(&core.receive(b"@X06\r")).is_none(),
            "the probe is due first"
        );
        let out = core.receive(b"@I101,0\r");
        assert!(matches!(completed(&out), Some((i, Ok(Reply::Status(_)))) if i == id));
        assert_eq!(core.take_async_error(), Some(ErrorCode(33)));
    }

    /// A query can be rejected too (`&C52` → `*021`): nothing assumes it cannot.
    #[test]
    fn a_rejected_query_is_rejected() {
        let (mut core, now) = synced();
        let id = crossed(&mut core, &Command::ErrorCode, b"*021\r", now);
        let out = core.receive(b"@I101,0\r");
        assert!(matches!(
            completed(&out),
            Some((i, Err(ControllerError::Rejected { .. }))) if i == id
        ));
    }

    #[test]
    fn a_stop_while_settling_reports_the_fault() {
        let (mut core, now) = synced();
        let id = crossed(&mut core, &Command::SetMode(Mode::Jog), b"*033\r", now);
        let (_, out) = core.stop(StopKind::Brake);
        assert!(matches!(
            completed(&out),
            Some((i, Err(ControllerError::Stopped))) if i == id
        ));
        assert!(
            out.iter()
                .any(|o| matches!(o, Output::Event(Event::Unsolicited(l)) if l == "*033"))
        );
        assert!(!core.probe_due());
    }

    /// The reply-order check: a command and a probe in one write.
    #[test]
    fn a_command_and_its_probe_in_one_write() {
        let (mut core, now) = synced();
        let (id, bytes) = core
            .request_with_probe(&Command::SetMode(Mode::Parameters), now)
            .expect("send");
        assert_eq!(bytes, b"@M0\r@I101\r");
        assert!(!core.probe_due(), "already written");
        let out = core.receive(b"\r@I101,0\r");
        assert!(matches!(completed(&out), Some((i, Ok(Reply::Ack))) if i == id));

        let (id, _) = core
            .request_with_probe(&Command::QueryParam(protocol::Param::Config(52)), now)
            .expect("send");
        let out = core.receive(b"*021\r@I102,0\r");
        assert!(matches!(
            completed(&out),
            Some((i, Err(ControllerError::Rejected { code: ErrorCode(21), .. }))) if i == id
        ));
        assert!(core.request(&Command::Status, now).is_ok(), "still in sync");
    }

    /// If the probe is answered first, the command's reply is still to come: it must not pair
    /// with the next command.
    #[test]
    fn a_probe_answered_first_is_out_of_order() {
        let (mut core, now) = synced();
        let (id, _) = core
            .request_with_probe(&Command::SetMode(Mode::Parameters), now)
            .expect("send");
        let out = core.receive(b"@I101,0\r");
        assert!(matches!(
            completed(&out),
            Some((i, Err(ControllerError::Overtaken { .. }))) if i == id
        ));
        assert!(matches!(
            core.request(&Command::Status, now),
            Err(Blocked::OutOfSync)
        ));
    }

    /// Until a fault has been reported, nothing that acts is sent; looking, braking and
    /// clearing the error are.
    #[test]
    fn an_unreported_fault_holds_back_what_acts() {
        let (mut core, now) = synced();
        core.receive(b"*033\r");
        for command in [Command::Status, Command::Brake, Command::ClearError] {
            core.request(&command, now).expect("goes through");
            core.receive(b"\r");
        }
        assert!(matches!(
            core.request(&Command::SetMode(Mode::Jog), now),
            Err(Blocked::Fails(ControllerError::AsyncError {
                code: ErrorCode(33)
            }))
        ));
        assert!(
            core.request(&Command::SetMode(Mode::Jog), now).is_ok(),
            "reported once"
        );
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
