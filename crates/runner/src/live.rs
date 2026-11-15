//! The machine's state as it changes, pushed to whoever listens: which head is down, the clamps,
//! where the blade is over the board, moving or still, homed, the op running and how many of its
//! steps are done.
//!
//! It is read from the controller, not predicted. Position is `@LX` and `@LY` (open-loop
//! steppers, so the commanded position). Heads and clamps follow the outputs in effect, which
//! inside a stream run later than the runner sent them: so the runner numbers each stream's
//! commands from 0 (`@N0`) as it starts it and notes where its outputs commands fall, and `@N`,
//! the number of the command running, says which of them have run (every vector, `A`, `W`, `I`
//! and `C` counts, as stage 5 measured on the machine). It names the command running, not how
//! far through it, so progress is coarse: the op's steps done of its steps (each move, change of
//! the heads or clamps, and dwell), not how far along a move.
//!
//! It is read only while the runner waits for the machine, with the stream already written, so
//! path data always has the line: `@N` at every poll, the position every
//! [`Config::state_every`](crate::Config::state_every). A state is pushed when anything but the
//! position and the steps done changes, and otherwise when they do, no more often than that.

use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use controller::Ask;
use machine::{Heads, Profile, frame};
use program::{ClampSide, Head};
use protocol::{Axis, Command, Status};

use crate::Event;

/// The machine as last read.
#[derive(Debug, Clone, PartialEq)]
pub struct MachineState {
    /// Which head is down; none until the runner has set the outputs.
    pub heads: Option<Heads>,
    /// Which clamps are closed; none until the runner has set the outputs.
    pub clamps: Option<ClampSide>,
    /// Whose blade [`MachineState::at_mm`] is: the head down, else the last one down.
    pub head: Head,
    /// That blade over the board, in the board frame, mm; none until read.
    pub at_mm: Option<[f64; 2]>,
    pub moving: bool,
    /// Homed, and not stopped since: positions can be trusted.
    pub homed: bool,
    /// The program's op running, during a run.
    pub op: Option<usize>,
    /// During an op, its steps: how many are done, and which is running.
    pub steps: Option<OpSteps>,
}

/// An op's steps, as the machine works through them: each move, change of the heads or
/// clamps, and dwell (and a stream's flags), as the machine finishes them. Coarse: an edge of a
/// rectangle is a handful, an oval hundreds of moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OpSteps {
    /// How many are done.
    pub done: u32,
    /// How many the op has.
    pub of: u32,
    /// What the step under way does, while the machine works on one.
    pub running: Option<StepKind>,
}

/// What a step does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepKind {
    /// A stream's flags, and the dead time before its first move.
    Start,
    /// A move with the heads up.
    Travel,
    /// A move with a head down.
    Cut,
    /// A head going down or up, or the blade coming out of the board.
    Heads,
    Clamps,
    /// A dwell: a head settling.
    Settle,
    /// Waiting for an input.
    Wait,
}

/// A stream going out: how many of its commands `@N` counts, and the outputs as each of its
/// outputs commands runs.
#[derive(Debug, Clone, Default)]
struct Stream {
    counted: u32,
    outputs: Vec<(u32, u8)>,
    /// The outputs before it.
    before: Option<u8>,
    /// `@N`, as last read.
    at: u32,
}

/// An op's steps in order, from its lowered `steps`, the outputs being `outputs` before it:
/// what `@N` counts in a stream, and the heads and clamps set outside one ([`OpSteps`]).
pub(crate) fn steps_in(
    steps: &[machine::Step],
    mut outputs: Option<u8>,
    profile: &Profile,
) -> Vec<StepKind> {
    let mut kinds = Vec::new();
    for step in steps {
        let machine::Step::Send(c) = step else {
            continue;
        };
        let kind = match c {
            Command::StreamFlags(_) => StepKind::Start,
            Command::Vector { .. } | Command::Step { .. } => {
                let down = outputs.is_some_and(|b| profile.heads(b) != Heads::Up);
                if down {
                    StepKind::Cut
                } else {
                    StepKind::Travel
                }
            }
            Command::StreamOutputs(bits) | Command::SetOutputs(bits) => {
                let clamps_only = outputs.is_some_and(|b| {
                    profile.heads(b) == profile.heads(*bits)
                        && profile.clamped(b) != profile.clamped(*bits)
                });
                outputs = Some(*bits);
                if clamps_only {
                    StepKind::Clamps
                } else {
                    StepKind::Heads
                }
            }
            Command::StreamDwellMs(_) => StepKind::Settle,
            Command::StreamWaitInput { .. } => StepKind::Wait,
            _ => continue,
        };
        kinds.push(kind);
    }
    kinds
}

/// Whether `@N` counts the mode-2 command `c` (stage 5).
fn counted(c: &Command) -> bool {
    matches!(
        c,
        Command::Vector { .. }
            | Command::Step { .. }
            | Command::StreamOutputs(_)
            | Command::StreamDwellMs(_)
            | Command::StreamWaitInput { .. }
            | Command::StreamFlags(_)
    )
}

/// What the runner knows and reads of the machine's state.
#[derive(Debug)]
pub(crate) struct Live {
    every: Duration,
    stream: Option<Stream>,
    /// The outputs in effect, outside a stream.
    outputs: Option<u8>,
    head: Head,
    at_mm: Option<[f64; 2]>,
    at_read: Option<Instant>,
    moving: bool,
    homed: bool,
    /// No stop or failure since the last homing (the runner's word): only then is `homed`
    /// believed.
    trusted: bool,
    op: Option<usize>,
    /// The op's steps, and how many were done before the stream now running.
    op_steps: Vec<StepKind>,
    op_done: u32,
    /// The state last pushed, and when.
    last: Option<(MachineState, Instant)>,
}

impl Live {
    pub(crate) fn new(every: Duration) -> Self {
        Self {
            every,
            stream: None,
            outputs: None,
            head: Head::Bevel,
            at_mm: None,
            at_read: None,
            moving: false,
            homed: false,
            trusted: true,
            op: None,
            op_steps: Vec::new(),
            op_done: 0,
            last: None,
        }
    }

    /// Whether state is read and pushed at all.
    pub(crate) fn on(&self) -> bool {
        !self.every.is_zero()
    }

    /// The runner is about to start a stream (after setting `@N0`), the outputs being
    /// `before`.
    pub(crate) fn stream_starts(&mut self, before: Option<u8>) {
        self.stream = Some(Stream {
            before,
            ..Stream::default()
        });
    }

    /// The runner sent `c` into the stream.
    pub(crate) fn streamed(&mut self, c: &Command) {
        if let Some(s) = &mut self.stream
            && counted(c)
        {
            s.counted += 1;
            if let Command::StreamOutputs(bits) = c {
                s.outputs.push((s.counted, *bits));
            }
        }
    }

    /// The stream has run (the machine is idle, or the stream was abandoned): its outputs are
    /// the runner's own from here.
    pub(crate) fn stream_ends(&mut self, outputs: Option<u8>) {
        if let Some(s) = self.stream.take() {
            self.op_done += s.counted;
        }
        self.outputs = outputs;
    }

    /// Outputs set outside a stream (`$A`).
    pub(crate) fn outputs_set(&mut self, bits: u8) {
        self.outputs = Some(bits);
        if self.op.is_some() {
            self.op_done += 1;
        }
    }

    /// Op `op` starts, with its steps ([`steps_in`]); or none runs.
    pub(crate) fn op(&mut self, op: Option<usize>, steps: Vec<StepKind>) {
        self.op = op;
        self.op_steps = steps;
        self.op_done = 0;
    }

    fn of(&self) -> u32 {
        u32::try_from(self.op_steps.len()).unwrap_or(u32::MAX)
    }

    /// How many of the op's steps are done: those before the stream, and in it all but the
    /// command `@N` names while it moves, all of them once it is still.
    fn steps_done(&self) -> u32 {
        let streamed = self.stream.as_ref().map_or(0, |s| {
            if self.moving {
                s.at.saturating_sub(1)
            } else {
                s.at
            }
        });
        (self.op_done + streamed).min(self.of())
    }

    /// The op's steps: done, and the one running while the machine works.
    fn op_steps(&self) -> OpSteps {
        let done = self.steps_done();
        let running = if self.moving {
            usize::try_from(done)
                .ok()
                .and_then(|k| self.op_steps.get(k))
                .copied()
        } else {
            None
        };
        OpSteps {
            done,
            of: self.of(),
            running,
        }
    }

    /// Whether the runner trusts the controller's homing (false after a stop or a failure).
    pub(crate) fn trust(&mut self, trusted: bool) {
        self.trusted = trusted;
        self.homed &= trusted;
    }

    pub(crate) fn streaming(&self) -> bool {
        self.stream.is_some()
    }

    /// What a status says: moving, and homed (if trusted).
    pub(crate) fn observe(&mut self, status: &Status) {
        self.moving = status.moving;
        self.homed = self.trusted && status.homed;
    }

    /// The outputs in effect: inside a stream, those of its last outputs command `@N` has
    /// reached.
    fn outputs_now(&self) -> Option<u8> {
        match &self.stream {
            Some(s) => s
                .outputs
                .iter()
                .rev()
                .find(|(k, _)| *k <= s.at)
                .map(|(_, bits)| *bits)
                .or(s.before),
            None => self.outputs,
        }
    }

    /// Read what's due while the runner waits: `@N` inside a stream, the position every
    /// [`Live::every`].
    pub(crate) fn read(
        &mut self,
        ask: &mut dyn Ask,
        status: &Status,
        profile: &Profile,
    ) -> controller::Result<()> {
        self.observe(status);
        if let Some(s) = &mut self.stream {
            s.at = ask.command_counter()?;
        }
        let due = self.at_read.is_none_or(|t| t.elapsed() >= self.every);
        if due {
            let x = ask.position_steps(Axis::X)?;
            let y = ask.position_steps(Axis::Y)?;
            self.at_steps([x, y], profile);
        }
        Ok(())
    }

    /// The carriage at `steps`: where the blade of the head in use is over the board.
    pub(crate) fn at_steps(&mut self, steps: [i32; 2], profile: &Profile) {
        if let Some(bits) = self.outputs_now() {
            match profile.heads(bits) {
                Heads::BevelDown => self.head = Head::Bevel,
                Heads::StraightDown => self.head = Head::Straight,
                Heads::Up => {}
            }
        }
        self.at_mm = Some(frame::board_point(profile, profile.head(self.head), steps));
        self.at_read = Some(Instant::now());
    }

    pub(crate) fn state(&self, profile: &Profile) -> MachineState {
        let outputs = self.outputs_now();
        MachineState {
            heads: outputs.map(|b| profile.heads(b)),
            clamps: outputs.map(|b| profile.clamped(b)),
            head: self.head,
            at_mm: self.at_mm,
            moving: self.moving,
            homed: self.homed,
            op: self.op,
            steps: self.op.map(|_| self.op_steps()),
        }
    }

    /// The state to push now if anything at all has changed: at the end of an op, whose steps
    /// done should read all of them before it is reported done.
    pub(crate) fn changed(&mut self, profile: &Profile) -> Option<MachineState> {
        let now = self.state(profile);
        let push = self.last.as_ref().is_none_or(|(last, _)| *last != now);
        push.then(|| {
            self.last = Some((now.clone(), Instant::now()));
            now
        })
    }

    /// The state to push now, if any: on any change but where the blade is and how many steps
    /// are done (the step running included), at once; on those, no more often than
    /// [`Live::every`] (an oval's steps come by the hundred).
    pub(crate) fn due(&mut self, profile: &Profile) -> Option<MachineState> {
        let now = self.state(profile);
        let push = match &self.last {
            None => true,
            Some((last, at)) => {
                let quiet = |s: &MachineState| MachineState {
                    at_mm: None,
                    steps: s.steps.map(|o| OpSteps { done: 0, ..o }),
                    ..s.clone()
                };
                quiet(last) != quiet(&now) || (*last != now && at.elapsed() >= self.every)
            }
        };
        push.then(|| {
            self.last = Some((now.clone(), Instant::now()));
            now
        })
    }
}

/// Every runner event, to each subscriber still listening.
#[derive(Debug, Clone, Default)]
pub(crate) struct Bus(Arc<Mutex<Vec<Sender<Event>>>>);

impl Bus {
    pub(crate) fn subscribe(&self) -> Receiver<Event> {
        let (tx, rx) = mpsc::channel();
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(tx);
        rx
    }

    /// To every subscriber; one that has gone is dropped.
    pub(crate) fn publish(&self, e: &Event) {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .retain(|tx| tx.send(e.clone()).is_ok());
    }
}
