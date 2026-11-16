//! The runner: programs and manual operations on one controller session.

use std::fmt;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, TryRecvError};
use std::time::Duration;

use controller::{Controller, Event as Signalled, Mode, Setting, Signal, StopHandle, Units, mode};
use machine::{Host, Profile, Step};
use program::{ClampSide, Program, Resume};
use protocol::{Axis, Command, Status};

use crate::error::{Cause, Error, Mismatch};
use crate::live::{Bus, Live, MachineState, steps_in};
use crate::session::Session;

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
    /// How often the position is read and a [`Event::State`] pushed while the machine works
    /// (`@LX` and `@LY`, while the runner waits; heads and clamps at every poll). Zero: no
    /// state is read or pushed.
    pub state_every: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            idle_timeout: Duration::from_secs(180),
            homing_timeout: Duration::from_secs(180),
            start_button: 221,
            poll: Duration::from_millis(50),
            speedup: 1.0,
            state_every: Duration::from_millis(100),
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
    /// The machine's state, when it changes ([`MachineState`]): during runs and manual
    /// operations, and from [`Runner::refresh`].
    State(MachineState),
}

/// How a run ended, when it ended as asked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    Finished,
    Cancelled { next: usize },
}

/// A failure during a run, and the op it happened in (`None`: outside the program's ops).
#[derive(Debug)]
struct Failure {
    op: Option<usize>,
    cause: Cause,
}

impl From<Cause> for Failure {
    fn from(cause: Cause) -> Self {
        Failure { op: None, cause }
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
    live: Live,
    bus: Bus,
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
            live: Live::new(config.state_every),
            bus: Bus::default(),
        }
    }

    /// Every [`Event`] from now on, runs' and manual operations' alike, for as long as the
    /// receiver is kept: for a UI that shows the machine live, or a log.
    pub fn subscribe(&self) -> Receiver<Event> {
        self.bus.subscribe()
    }

    /// A handle that stops the machine at once, from any thread (`@B`). The run in progress
    /// then fails, the runner makes the machine safe, and it must be homed again.
    pub fn stop_handle(&self) -> StopHandle {
        self.stop.clone()
    }

    pub fn profile(&self) -> &Profile {
        &self.profile
    }

    /// Take a new machine profile between runs, without reconnecting: a calibration revision
    /// applied while the machine is on (CALIBRATION.md §9). Homing stays: a profile changes the
    /// arithmetic that places a cut, not where the carriage is. If the controller settings it
    /// implies differ ([`Profile::controller_settings`]; a calibration never changes them), they
    /// are written as [`configure`](Runner::configure) does, and what was written is returned.
    /// Nothing can run meanwhile: a run holds the runner.
    ///
    /// # Errors
    /// As [`configure`](Runner::configure); the new profile is kept, as it is after a failed
    /// configuration at the start.
    pub fn set_profile(&mut self, profile: Profile) -> Result<Vec<Setting>, Error> {
        let settings_differ = profile.controller_settings() != self.profile.controller_settings();
        self.profile = profile;
        if settings_differ {
            self.configure()
        } else {
            Ok(Vec::new())
        }
    }

    /// Whether the machine must be homed before it moves: after a stop (even one made while
    /// nothing ran) or a failure, or if the controller doesn't know its position.
    pub fn needs_homing(&mut self) -> Result<bool, Error> {
        // A stop latches the session, so asking the controller would only report the stop.
        if self.needs_homing || self.stop.is_stopped() {
            return Ok(true);
        }
        let s = self.session.status()?;
        Ok(!s.homed)
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
                read if read == s => written.push(s),
                read => wrong.push(Mismatch { wanted: s, read }),
            }
        }
        if wrong.is_empty() {
            Ok(written)
        } else {
            Err(Error::Misconfigured(wrong))
        }
    }

    /// Home the machine as GMC does: heads up, Z first, then X and Y; then set the bevel head's
    /// V-groove switch to normal cutting, and park (protocol reference §4.1).
    pub fn home(&mut self) -> Result<(), Error> {
        let bus = self.bus.clone();
        let out = &mut |e: Event| bus.publish(&e);
        self.busy = true;
        let r = self.homing(out);
        self.busy = false;
        let result = match r {
            Ok(s) if s.homed => {
                self.needs_homing = false;
                self.live.trust(true);
                Ok(())
            }
            Ok(s) => Err(Error::HomingIncomplete(s)),
            Err(f) => Err(self.fail(None, f)),
        };
        let _ = self.report(out);
        result
    }

    /// Read the machine's state and push it to subscribers if it has changed: for a UI between
    /// runs, when nothing else is reading it. Asks nothing while a stop is latched.
    pub fn refresh(&mut self) -> Result<(), Error> {
        let bus = self.bus.clone();
        Ok(self.report(&mut |e| bus.publish(&e))?)
    }

    /// Read the state now, and push it to `out` if it has changed.
    fn report(&mut self, out: &mut dyn FnMut(Event)) -> Result<(), Cause> {
        if !self.live.on() || self.stop.is_stopped() {
            return Ok(());
        }
        let status = self.session.status()?;
        let [x, y, _] = self.session.position()?;
        self.live.observe(&status);
        self.live.at_steps([x, y], &self.profile);
        if let Some(s) = self.live.due(&self.profile) {
            out(Event::State(s));
        }
        Ok(())
    }

    /// Wait until `done`, reading the machine's state as it goes and pushing it to `out` when
    /// it changes.
    fn wait(
        &mut self,
        timeout: Duration,
        done: impl FnMut(&Status) -> bool,
        out: &mut dyn FnMut(Event),
    ) -> Result<Status, Cause> {
        if !self.live.on() {
            return self.session.wait_until(timeout, done);
        }
        let Self {
            session,
            live,
            profile,
            ..
        } = self;
        session.wait_until_with(timeout, done, |ask, status| {
            live.read(ask, status, profile)?;
            if let Some(s) = live.due(profile) {
                out(Event::State(s));
            }
            Ok(())
        })
    }

    fn homing(&mut self, out: &mut dyn FnMut(Event)) -> Result<Status, Cause> {
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
        self.wait(self.config.homing_timeout, homed, out)?;
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
        let s = self.wait(self.config.homing_timeout, homed, out)?;
        if !s.homed {
            return Ok(s);
        }
        let x = self.session.position()?[0];
        let p = &self.profile;
        let mut steps = machine::vgroove_reset(p, clamps, x);
        steps.extend(machine::park(p, clamps));
        let (_tx, rx) = mpsc::channel();
        self.exec(&steps, None, &rx, out)?;
        self.session.status()
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
        let bus = self.bus.clone();
        let out = &mut |e: Event| bus.publish(&e);
        self.busy = true;
        let r = self.exec(steps, None, &rx, out);
        self.busy = false;
        let result = r.map(|_| ()).map_err(|f| self.fail(None, f));
        let _ = self.report(out);
        result
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
        // Every event to the subscribers too.
        let bus = self.bus.clone();
        let events = &mut |e: Event| {
            bus.publish(&e);
            events(e);
        };
        events(Event::Started {
            ops: job.ops.len(),
            estimate_s,
        });
        self.busy = true;
        let r = self.job(program, &job, requests, events);
        self.busy = false;
        self.live.op(None, Vec::new());
        // The machine's state as the run leaves it, then how the run ended: always the last.
        match r {
            Ok(outcome) => {
                let _ = self.report(events);
                events(match outcome {
                    Outcome::Finished => Event::Finished,
                    Outcome::Cancelled { next } => Event::Cancelled { next },
                });
                Ok(outcome)
            }
            Err(Failure { op, cause }) => {
                let safe = self.recover();
                let _ = self.report(events);
                events(Event::Failed {
                    op,
                    error: cause.to_string(),
                    safe,
                });
                Err(Error::Failed { op, cause, safe })
            }
        }
    }

    fn job(
        &mut self,
        program: &Program,
        job: &machine::Job,
        requests: &Receiver<Request>,
        events: &mut dyn FnMut(Event),
    ) -> Result<Outcome, Failure> {
        self.exec(&job.setup, None, requests, events)?;
        let outcome = self.ops(program, job, requests, events)?;
        self.exec(&job.finish, None, requests, events)?;
        Ok(outcome)
    }

    /// The program's ops in order, with the requests between them. Each ends with the machine
    /// still: a pause, a cancel or the end then finds nothing running.
    fn ops(
        &mut self,
        program: &Program,
        job: &machine::Job,
        requests: &Receiver<Request>,
        events: &mut dyn FnMut(Event),
    ) -> Result<Outcome, Failure> {
        for (index, (op, steps)) in program.ops.iter().zip(&job.ops).enumerate() {
            let at = |cause| Failure {
                op: Some(index),
                cause,
            };
            if self.between(index, requests, events).map_err(at)? == Flow::Cancel {
                return Ok(Outcome::Cancelled { next: index });
            }
            events(Event::Op {
                index,
                id: op.id().map(str::to_owned),
                label: op.label().map(str::to_owned),
            });
            self.live
                .op(Some(index), steps_in(steps, self.outputs, &self.profile));
            if self
                .exec(steps, Some(index), requests, events)
                .map_err(at)?
                == Flow::Cancel
            {
                return Ok(Outcome::Cancelled { next: index + 1 });
            }
            // GMC's form ends each op still. A stream runs on after its commands are sent:
            // let the cut end, so that it is done when reported done, and a fault in it is
            // laid on it rather than on the next.
            if self.profile.capabilities.streams() {
                self.wait(self.config.idle_timeout, Status::is_idle, events)
                    .map_err(at)?;
                self.stream_done(events);
            }
            // All its steps done, before it is reported done.
            if self.live.on()
                && let Some(s) = self.live.changed(&self.profile)
            {
                events(Event::State(s));
            }
            events(Event::OpDone { index });
        }
        Ok(Outcome::Finished)
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
                    self.wait(self.config.idle_timeout, Status::is_idle, events)?;
                    self.stream_done(events);
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
                Step::Send(c) => {
                    self.send(c)?;
                    // Heads or clamps set outside a stream: in effect now.
                    if self.live.on()
                        && matches!(c, Command::SetOutputs(_))
                        && let Some(s) = self.live.due(&self.profile)
                    {
                        events(Event::State(s));
                    }
                }
                Step::WaitIdle => {
                    self.wait(self.config.idle_timeout, Status::is_idle, events)?;
                    self.stream_done(events);
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
                // A stream starts: number its commands from 0, so `@N` says which have run.
                if self.live.on() && !self.live.streaming() {
                    self.session.switch(Mode::Cutting)?;
                    self.session.set_command_counter(0)?;
                    self.live.stream_starts(self.outputs);
                }
                self.session.stream(c)?;
                self.live.streamed(c);
                if let Command::StreamOutputs(bits) = c {
                    self.outputs = Some(*bits);
                }
            }
            _ => return Err(Cause::Unsupported(c.clone())),
        }
        Ok(())
    }

    /// The machine is still after a stream: what it set is in effect. Push the state.
    fn stream_done(&mut self, out: &mut dyn FnMut(Event)) {
        if self.live.streaming() {
            self.live.stream_ends(self.outputs);
            if let Some(s) = self.live.due(&self.profile) {
                out(Event::State(s));
            }
        }
    }

    fn set_outputs(&mut self, bits: u8) -> Result<(), Cause> {
        self.session.set_outputs(bits)?;
        self.outputs = Some(bits);
        self.live.outputs_set(bits);
        Ok(())
    }

    /// Report a failure, after making the machine safe; it must be homed again.
    fn fail(&mut self, op: Option<usize>, cause: Cause) -> Error {
        let safe = self.recover();
        Error::Failed { op, cause, safe }
    }

    /// After a failure: make the machine safe, and want it homed again. Whether it is safe.
    fn recover(&mut self) -> bool {
        self.needs_homing = true;
        self.live.trust(false);
        let safe = self.make_safe();
        // Whatever the stream left set, the outputs are now what was last set.
        self.live.stream_ends(self.outputs);
        safe
    }

    /// After a stop or a failure (PROGRAM_FORMAT §5): brake through the stop lane, which
    /// waits for no reply and so goes out even when the controller has stopped answering (a
    /// second `@B` after a stop is harmless), re-establish reply pairing and release the stop
    /// latch (a reply abandoned by the stop is discarded, not paired), clear any error (`@@`,
    /// as GMC does after errors: a controller in an error state refuses mode switches), wait
    /// for standstill, then heads up and clamps released. Whether that last step worked.
    ///
    /// Another stop while this runs (an operator pressing again because nothing seemed to
    /// happen) latches again, and everything after it would fail on the latch: so it starts
    /// over, a few times at most. A failure that isn't a stop is reported at once.
    fn make_safe(&mut self) -> bool {
        for _ in 0..SAFE_ATTEMPTS {
            let _ = self.stop.brake();
            let _ = self.session.clear_stop();
            let _ = self.session.clear_error();
            let _ = self.session.wait_idle(Duration::from_secs(30));
            if self.set_outputs(self.profile.neutral_outputs).is_ok() {
                return true;
            }
            if !self.stop.is_stopped() {
                return false;
            }
        }
        false
    }
}

/// How many times making the machine safe starts over when stops keep coming.
const SAFE_ATTEMPTS: usize = 5;

impl Drop for Runner {
    /// Dropped mid-run (a panic on the runner's thread): make the machine safe.
    fn drop(&mut self) {
        if self.busy {
            let _ = self.make_safe();
        }
    }
}
