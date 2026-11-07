//! The runner: programs and manual operations on one controller session.

use std::fmt;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, TryRecvError};
use std::time::Duration;

use controller::{
    Controller, ControllerError, Event as Signalled, Mode, Setting, Signal, StopHandle, Units, mode,
};
use machine::{Host, Profile, Step};
use program::{ClampSide, Op, Program, Resume};
use protocol::{Axis, Command, Status};

use crate::session::{Session, SessionError};

/// How the runner waits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// The longest a motion may take before the runner gives up on it.
    pub idle_timeout: Duration,
    /// The longest homing an axis may take.
    pub homing_timeout: Duration,
    /// The green start button's input (I221 on the 601).
    pub start_button: u8,
    /// How often requests and the start button are looked at while paused or holding.
    pub poll: Duration,
    /// Host-side waits (head settles) are divided by this: 1 on the machine, and the
    /// simulator's speed-up when it runs faster than life.
    pub speedup: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            idle_timeout: Duration::from_secs(180),
            homing_timeout: Duration::from_secs(180),
            start_button: 221,
            poll: Duration::from_millis(50),
            speedup: 1.0,
        }
    }
}

/// What the station asks during a run. Pause and cancel take effect at the end of the current
/// cut; to stop at once, use the [`Runner::stop_handle`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// Stop at the end of the current cut, heads up, and hold.
    Pause,
    /// Carry on after a pause, or after a `pause` op that waits for the operator.
    Resume,
    /// End the run at the end of the current cut: heads up, park, clamps released.
    Cancel,
}

/// What happens during a run, as it happens.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// The run starts: its ops, and the estimated run time.
    Started {
        ops: usize,
        estimate_s: f64,
    },
    /// Op `index` starts, with the program's `id` and `label` for it.
    Op {
        index: usize,
        id: Option<String>,
        label: Option<String>,
    },
    OpDone {
        index: usize,
    },
    /// Paused before op `next`, heads up.
    Paused {
        next: usize,
    },
    /// A `pause` op: waiting for the operator or the green button.
    Holding {
        index: usize,
        message: String,
        resume: Resume,
    },
    Resumed,
    Finished,
    /// Cancelled before op `next`; parked, clamps released.
    Cancelled {
        next: usize,
    },
    /// The run failed in op `op` (`None`: before the first op or after the last).
    Failed {
        op: Option<usize>,
        error: String,
        safe: bool,
    },
}

/// How a run ended, when it ended as asked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    Finished,
    Cancelled { next: usize },
}

/// Why something failed on the controller's side.
#[derive(Debug)]
pub enum Cause {
    Controller(ControllerError),
    /// The session is gone.
    Lost,
    /// The lowered job asked for a command the runner doesn't send: a bug in the lowering.
    Unsupported(Command),
}

impl fmt::Display for Cause {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Cause::Controller(e) => e.fmt(f),
            Cause::Lost => write!(f, "the controller session is gone"),
            Cause::Unsupported(c) => write!(f, "`{c}` is not a command the runner sends (a bug)"),
        }
    }
}

impl From<SessionError> for Cause {
    fn from(e: SessionError) -> Self {
        match e {
            SessionError::Controller(e) => Cause::Controller(e),
            SessionError::Lost => Cause::Lost,
        }
    }
}

#[derive(Debug)]
pub enum Error {
    /// The program can't be cut on this machine (machine::lower refused it): nothing was sent.
    Invalid(machine::Error),
    /// The machine isn't homed, or hasn't been since a stop or a failure: home it first.
    NotHomed,
    /// Homing ended without the controller knowing its position.
    HomingIncomplete(Status),
    /// The controller's configuration differs from the profile's and couldn't be set.
    Misconfigured(Vec<String>),
    /// Talking to the controller failed while nothing was moving: a read, or a setting
    /// written while idle. Nothing needed making safe, and homing is not lost.
    Controller(Cause),
    /// Something failed in op `op` (`None`: outside the program's ops). The runner then
    /// stopped and made the machine safe: heads up, clamps released (`safe` says whether that
    /// worked). It must be homed again before it moves.
    Failed {
        op: Option<usize>,
        cause: Cause,
        safe: bool,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Invalid(e) => write!(f, "the program can't be cut: {e}"),
            Error::NotHomed => write!(f, "the machine must be homed first"),
            Error::HomingIncomplete(s) => {
                write!(f, "homing ended without a home position (status {s})")
            }
            Error::Misconfigured(what) => write!(
                f,
                "the controller's configuration won't take: {}",
                what.join("; ")
            ),
            Error::Controller(cause) => write!(f, "the controller: {cause}"),
            Error::Failed { op, cause, safe } => {
                let at = op.map_or_else(
                    || "outside the program's ops".to_owned(),
                    |o| format!("in op {o}"),
                );
                let then = if *safe {
                    "heads up, clamps released"
                } else {
                    "the safe state could NOT be confirmed"
                };
                write!(f, "failed {at}: {cause}; {then}; home before moving again")
            }
        }
    }
}

impl std::error::Error for Error {}

impl From<SessionError> for Error {
    fn from(e: SessionError) -> Self {
        Error::Controller(e.into())
    }
}

/// Whether to carry on after a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flow {
    Continue,
    Cancel,
}

/// Runs programs and manual operations, one at a time, on a controller session.
pub struct Runner {
    profile: Profile,
    config: Config,
    session: Session,
    stop: StopHandle,
    signals: Receiver<Signalled>,
    /// The outputs last set, if any: the clamps a manual park keeps.
    outputs: Option<u8>,
    /// A stop or a failure since the last homing: positions aren't trusted until homed again.
    needs_homing: bool,
    /// Inside a run or a manual operation: if dropped then, make the machine safe.
    busy: bool,
}

impl fmt::Debug for Runner {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Runner")
            .field("mode", &self.session.mode())
            .field("outputs", &self.outputs)
            .field("needs_homing", &self.needs_homing)
            .finish_non_exhaustive()
    }
}

impl Runner {
    pub fn new(ctl: Controller<mode::Parameters>, profile: Profile, config: Config) -> Self {
        let stop = ctl.stop_handle();
        let signals = ctl.subscribe();
        Self {
            profile,
            config,
            session: Session::new(ctl),
            stop,
            signals,
            outputs: None,
            needs_homing: false,
            busy: false,
        }
    }

    /// A handle that stops the machine at once, from any thread (`@B`). The run in progress
    /// then fails, the runner makes the machine safe, and it must be homed again.
    pub fn stop_handle(&self) -> StopHandle {
        self.stop.clone()
    }

    pub fn profile(&self) -> &Profile {
        &self.profile
    }

    /// Whether the machine must be homed before it moves: after a stop or a failure, or if the
    /// controller doesn't know its position.
    pub fn needs_homing(&mut self) -> Result<bool, Error> {
        let s = self.session.status()?;
        Ok(self.needs_homing || !s.homed)
    }

    /// Where the carriage is, steps (X, Y, Z).
    pub fn position(&mut self) -> Result<[i32; 3], Error> {
        Ok(self.session.position()?)
    }

    /// Bring the controller's configuration in line with the profile's
    /// ([`Profile::controller_settings`]): read each setting back, write the ones that differ,
    /// and read them again. Returns what had to be written; fails if any won't take.
    pub fn configure(&mut self) -> Result<Vec<Setting>, Error> {
        let mut written = Vec::new();
        let mut wrong = Vec::new();
        for s in self.profile.controller_settings() {
            let key = s.key();
            if self.session.read_setting(key)? == s {
                continue;
            }
            self.session.set(s)?;
            match self.session.read_setting(key)? {
                now if now == s => written.push(s),
                now => wrong.push(format!("{key} reads `{now}` after `{s}`")),
            }
        }
        if wrong.is_empty() {
            Ok(written)
        } else {
            Err(Error::Misconfigured(wrong))
        }
    }

    /// Home the machine as GMC does: heads up, Z first, then X and Y (protocol reference §4.1).
    pub fn home(&mut self) -> Result<(), Error> {
        self.busy = true;
        let r = self.homing();
        self.busy = false;
        match r {
            Ok(s) if s.homed => {
                self.needs_homing = false;
                Ok(())
            }
            Ok(s) => Err(Error::HomingIncomplete(s)),
            Err(f) => Err(self.fail(None, f)),
        }
    }

    fn homing(&mut self) -> Result<Status, Cause> {
        if self.stop.is_stopped() {
            self.session.clear_stop()?;
        }
        let p = &self.profile;
        let (h, neutral) = (p.homing, p.neutral_outputs);
        let clamps = self.outputs.unwrap_or(0) & (p.clamps.left | p.clamps.right);
        self.set_outputs(clamps | neutral)?;
        self.session.clear_error()?;
        for s in [
            Setting::Units(Units::Steps),
            Setting::HomingDoneSignal(true),
            Setting::Speed {
                entry: 10,
                value: h.search_speed,
            },
            Setting::HomeOffset {
                axis: Axis::Z,
                value: h.offset_steps[2],
            },
        ] {
            self.session.set(s)?;
        }
        self.session.home(&[Axis::Z])?;
        let homed = |s: &Status| !s.moving && !s.homing;
        self.session.wait_until(self.config.homing_timeout, homed)?;
        for s in [
            Setting::Speed {
                entry: 11,
                value: h.back_off_speed,
            },
            Setting::RampMs(h.ramp_ms),
            Setting::HomeOffset {
                axis: Axis::X,
                value: h.offset_steps[0],
            },
            Setting::HomeOffset {
                axis: Axis::Y,
                value: h.offset_steps[1],
            },
        ] {
            self.session.set(s)?;
        }
        self.session.home(&[Axis::X, Axis::Y])?;
        Ok(self.session.wait_until(self.config.homing_timeout, homed)?)
    }

    /// Both heads up and travel to the park, the clamps as they are.
    pub fn park(&mut self) -> Result<(), Error> {
        self.require_homed()?;
        let p = &self.profile;
        let clamps = self.outputs.unwrap_or(0) & (p.clamps.left | p.clamps.right);
        let steps = machine::park(p, clamps);
        self.manual(&steps)
    }

    /// Close or open the clamps, heads up.
    pub fn clamp(&mut self, side: ClampSide) -> Result<(), Error> {
        let c = self.profile.clamps;
        let bits = match side {
            ClampSide::Left => c.left,
            ClampSide::Right => c.right,
            ClampSide::Both => c.left | c.right,
            ClampSide::None => 0,
        };
        let steps = [Step::Send(Command::SetOutputs(
            bits | self.profile.neutral_outputs,
        ))];
        self.manual(&steps)
    }

    fn manual(&mut self, steps: &[Step]) -> Result<(), Error> {
        let (_tx, rx) = mpsc::channel();
        self.busy = true;
        let r = self.exec(steps, None, &rx, &mut |_| {});
        self.busy = false;
        r.map(|_| ()).map_err(|f| self.fail(None, f))
    }

    fn require_homed(&mut self) -> Result<(), Error> {
        if self.needs_homing()? {
            Err(Error::NotHomed)
        } else {
            Ok(())
        }
    }

    /// Run a program: check it can be cut, require a homed machine, then its ops in order,
    /// reporting each; honour `pause` ops, and [`Request`]s between cuts; park and release the
    /// clamps at the end. On any failure, stop and make the machine safe.
    pub fn run(
        &mut self,
        program: &Program,
        requests: &Receiver<Request>,
        events: &mut dyn FnMut(Event),
    ) -> Result<Outcome, Error> {
        let job = machine::lower(program, &self.profile).map_err(Error::Invalid)?;
        self.require_homed()?;
        let estimate_s = machine::estimate(&job, &self.profile, &Host::default()).total_s;
        events(Event::Started {
            ops: job.ops.len(),
            estimate_s,
        });
        self.busy = true;
        let r = self.job(program, &job, requests, events);
        self.busy = false;
        match r {
            Ok(outcome) => {
                events(match outcome {
                    Outcome::Finished => Event::Finished,
                    Outcome::Cancelled { next } => Event::Cancelled { next },
                });
                Ok(outcome)
            }
            Err((op, f)) => {
                let e = self.fail(op, f);
                if let Error::Failed { op, cause, safe } = &e {
                    events(Event::Failed {
                        op: *op,
                        error: cause.to_string(),
                        safe: *safe,
                    });
                }
                Err(e)
            }
        }
    }

    fn job(
        &mut self,
        program: &Program,
        job: &machine::Job,
        requests: &Receiver<Request>,
        events: &mut dyn FnMut(Event),
    ) -> Result<Outcome, (Option<usize>, Cause)> {
        self.exec(&job.setup, None, requests, events)
            .map_err(|f| (None, f))?;
        let mut outcome = Outcome::Finished;
        for (index, steps) in job.ops.iter().enumerate() {
            if self
                .between(index, requests, events)
                .map_err(|f| (Some(index), f))?
                == Flow::Cancel
            {
                outcome = Outcome::Cancelled { next: index };
                break;
            }
            let (id, label) = names(&program.ops[index]);
            events(Event::Op { index, id, label });
            if self
                .exec(steps, Some(index), requests, events)
                .map_err(|f| (Some(index), f))?
                == Flow::Cancel
            {
                outcome = Outcome::Cancelled { next: index + 1 };
                break;
            }
            events(Event::OpDone { index });
        }
        if outcome != Outcome::Finished {
            // The last cut may still be running in a stream: let it end first.
            self.session
                .wait_idle(self.config.idle_timeout)
                .map_err(|e| (None, e.into()))?;
        }
        self.exec(&job.finish, None, requests, events)
            .map_err(|f| (None, f))?;
        Ok(outcome)
    }

    /// Between cuts: act on the requests waiting.
    fn between(
        &mut self,
        next: usize,
        requests: &Receiver<Request>,
        events: &mut dyn FnMut(Event),
    ) -> Result<Flow, Cause> {
        loop {
            match requests.try_recv() {
                Ok(Request::Pause) => {
                    self.session.wait_idle(self.config.idle_timeout)?;
                    events(Event::Paused { next });
                    let flow = self.wait_for_go(true, requests)?;
                    if flow == Flow::Continue {
                        events(Event::Resumed);
                    }
                    return Ok(flow);
                }
                Ok(Request::Cancel) => return Ok(Flow::Cancel),
                Ok(Request::Resume) => {}
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => return Ok(Flow::Continue),
            }
        }
    }

    /// Hold until the operator resumes (or cancels), or, if `button`, the green button is
    /// pressed. A dropped request channel cancels: nobody is there to resume.
    fn wait_for_go(&mut self, button: bool, requests: &Receiver<Request>) -> Result<Flow, Cause> {
        while self.signals.try_recv().is_ok() {} // presses from before the hold don't count
        loop {
            match requests.recv_timeout(self.config.poll) {
                Ok(Request::Resume) => return Ok(Flow::Continue),
                Ok(Request::Cancel) | Err(RecvTimeoutError::Disconnected) => {
                    return Ok(Flow::Cancel);
                }
                Ok(Request::Pause) | Err(RecvTimeoutError::Timeout) => {}
            }
            if button {
                let pressed = std::iter::from_fn(|| self.signals.try_recv().ok()).any(|e| {
                    matches!(e, Signalled::Signal(Signal::Input1High | Signal::Input1Low))
                });
                if pressed || self.session.read_input(self.config.start_button)? {
                    return Ok(Flow::Continue);
                }
            }
        }
    }

    fn exec(
        &mut self,
        steps: &[Step],
        index: Option<usize>,
        requests: &Receiver<Request>,
        events: &mut dyn FnMut(Event),
    ) -> Result<Flow, Cause> {
        for step in steps {
            match step {
                Step::Send(c) => self.send(c)?,
                Step::WaitIdle => {
                    self.session.wait_idle(self.config.idle_timeout)?;
                }
                Step::Settle(ms) => {
                    std::thread::sleep(Duration::from_secs_f64(
                        f64::from(*ms) / 1000.0 / self.config.speedup,
                    ));
                }
                Step::Hold { message, resume } => {
                    events(Event::Holding {
                        index: index.unwrap_or(0),
                        message: message.clone(),
                        resume: *resume,
                    });
                    if self.wait_for_go(*resume == Resume::StartButton, requests)? == Flow::Cancel {
                        return Ok(Flow::Cancel);
                    }
                    events(Event::Resumed);
                }
            }
        }
        Ok(Flow::Continue)
    }

    fn send(&mut self, c: &Command) -> Result<(), Cause> {
        match (c.required_mode(), c) {
            (Some(Mode::Parameters), Command::Set(s)) => self.session.set(*s)?,
            (Some(Mode::Jog), Command::SetOutputs(bits)) => self.set_outputs(*bits)?,
            (Some(Mode::Cutting), _) => {
                self.session.stream(c)?;
                if let Command::StreamOutputs(bits) = c {
                    self.outputs = Some(*bits);
                }
            }
            _ => return Err(Cause::Unsupported(c.clone())),
        }
        Ok(())
    }

    fn set_outputs(&mut self, bits: u8) -> Result<(), SessionError> {
        self.session.set_outputs(bits)?;
        self.outputs = Some(bits);
        Ok(())
    }

    /// Report a failure, after making the machine safe; it must be homed again.
    fn fail(&mut self, op: Option<usize>, cause: Cause) -> Error {
        let safe = self.make_safe();
        self.needs_homing = true;
        Error::Failed { op, cause, safe }
    }

    /// After a stop or a failure (PROGRAM_FORMAT §5): brake (the stop handle may already have),
    /// re-establish reply pairing and release the stop latch (a reply abandoned by the stop is
    /// discarded, not paired), clear any error (`@@`, as GMC does after errors: a controller in
    /// an error state refuses mode switches), wait for standstill, then heads up and clamps
    /// released. Whether that last step worked.
    fn make_safe(&mut self) -> bool {
        if !self.stop.is_stopped() {
            let _ = self.session.brake();
        }
        let _ = self.session.clear_stop();
        let _ = self.session.clear_error();
        let _ = self.session.wait_idle(Duration::from_secs(30));
        self.set_outputs(self.profile.neutral_outputs).is_ok()
    }
}

impl Drop for Runner {
    /// Dropped mid-run (a panic on the runner's thread): make the machine safe.
    fn drop(&mut self) {
        if self.busy {
            let _ = self.make_safe();
        }
    }
}

fn names(op: &Op) -> (Option<String>, Option<String>) {
    match op {
        Op::Clamp(c) => (c.id.clone(), c.label.clone()),
        Op::Cut(c) => (c.id.clone(), c.label.clone()),
        Op::Move(m) => (m.id.clone(), m.label.clone()),
        Op::Pause(p) => (p.id.clone(), p.label.clone()),
    }
}
