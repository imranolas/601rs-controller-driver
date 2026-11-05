//! The typed API: [`Controller<M>`] exposes only the commands its mode accepts.

use std::fmt;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use link::Link;
use protocol::{Axis, Command, Mode, Reply, Status, StreamFlags, Switches, reply};

use crate::session::Session;
use crate::{Config, ControllerError, Event, Result, StopHandle};

/// Typestate markers for the controller mode.
pub mod mode {
    /// Mode 0: parameters.
    #[derive(Debug)]
    pub struct Parameters;
    /// Mode 1: jog, homing, outputs.
    #[derive(Debug)]
    pub struct Jog;
    /// Mode 2: cutting (streamed vectors).
    #[derive(Debug)]
    pub struct Cutting;
}

mod sealed {
    pub trait Sealed {}
    impl Sealed for super::mode::Parameters {}
    impl Sealed for super::mode::Jog {}
    impl Sealed for super::mode::Cutting {}
}

/// Implemented by the [`mode`] markers only.
pub trait ModeMarker: sealed::Sealed {
    const MODE: Mode;
}
impl ModeMarker for mode::Parameters {
    const MODE: Mode = Mode::Parameters;
}
impl ModeMarker for mode::Jog {
    const MODE: Mode = Mode::Jog;
}
impl ModeMarker for mode::Cutting {
    const MODE: Mode = Mode::Cutting;
}

/// A connected controller in mode `M`.
pub struct Controller<M> {
    session: Session,
    _mode: PhantomData<M>,
}

impl<M: ModeMarker> fmt::Debug for Controller<M> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Controller")
            .field("mode", &M::MODE)
            .field("session", &self.session)
            .finish()
    }
}

/// A failed mode switch. The controller is handed back, still in its old mode.
pub struct ModeSwitchError<M> {
    pub controller: Box<Controller<M>>,
    pub error: ControllerError,
}

impl<M: ModeMarker> fmt::Debug for ModeSwitchError<M> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ModeSwitchError")
            .field("from", &M::MODE)
            .field("error", &self.error)
            .finish()
    }
}

impl<M: ModeMarker> fmt::Display for ModeSwitchError<M> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "switching out of {:?}: {}", M::MODE, self.error)
    }
}

impl<M: ModeMarker> std::error::Error for ModeSwitchError<M> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.error)
    }
}

/// Open a session: resync (probe with `@X`), read the version (`@V`), enter mode 0.
/// Returns the controller and its version string (e.g. `@V2.00/55W 19.10.10 2807001`).
pub fn connect(link: Link, cfg: Config) -> Result<(Controller<mode::Parameters>, String)> {
    let mut session = Session::start(link, cfg)?;
    session.resync()?;
    let version = match session.request(&Command::Version, false)? {
        Reply::Value(v) if v.starts_with("@V") => v,
        other => return Err(session.unexpected(&Command::Version, &other)),
    };
    session.expect_ack(&Command::SetMode(Mode::Parameters), false)?;
    Ok((
        Controller {
            session,
            _mode: PhantomData,
        },
        version,
    ))
}

// ------------------------------------------------------------------ every mode (master commands)

impl<M: ModeMarker> Controller<M> {
    fn in_stream() -> bool {
        M::MODE == Mode::Cutting
    }

    fn query(&mut self, cmd: &Command) -> Result<Reply> {
        self.session.request(cmd, Self::in_stream())
    }

    fn ack(&mut self, cmd: &Command) -> Result<()> {
        self.session.expect_ack(cmd, Self::in_stream())
    }

    fn value<T>(&mut self, cmd: &Command, parse: impl FnOnce(&str) -> Option<T>) -> Result<T> {
        let reply = self.query(cmd)?;
        if let Reply::Value(v) = &reply {
            if let Some(t) = parse(v) {
                return Ok(t);
            }
        }
        Err(self.session.unexpected(cmd, &reply))
    }

    pub fn mode(&self) -> Mode {
        M::MODE
    }

    /// `@X`
    pub fn status(&mut self) -> Result<Status> {
        let cmd = Command::Status;
        match self.query(&cmd)? {
            Reply::Status(s) => Ok(s),
            other => Err(self.session.unexpected(&cmd, &other)),
        }
    }

    /// `@E`: current error code (0 = none).
    pub fn error_code(&mut self) -> Result<u16> {
        self.value(&Command::ErrorCode, reply::parse_error_code)
    }

    /// `@L<a>`: axis position in steps.
    pub fn position_steps(&mut self, axis: Axis) -> Result<i32> {
        self.value(&Command::PositionSteps(axis), |l| {
            reply::parse_position(l, axis)
        })
    }

    /// `@I<n>`: logical input state (e.g. 221 = start button).
    pub fn read_input(&mut self, input: u8) -> Result<bool> {
        self.value(&Command::ReadInput(input), |l| reply::parse_input(l, input))
    }

    /// `@H`: interface buffer fill level in percent.
    pub fn buffer_level_pct(&mut self) -> Result<u8> {
        self.value(&Command::BufferLevel, reply::parse_buffer_level)
    }

    /// `@Y`: stop, ready, reference and limit switches.
    pub fn switches(&mut self) -> Result<Switches> {
        self.value(&Command::Switches, Switches::parse)
    }

    /// `@Q<n>`: state of logical output `n`. Q242–Q249 are the auxiliary signals 1–8, i.e. the
    /// bits of `$A`/`A` (head and clamp valves on the 601).
    pub fn read_output(&mut self, output: u8) -> Result<bool> {
        self.value(&Command::ReadOutput(output), |l| {
            reply::parse_output(l, output)
        })
    }

    /// `@N`: number of the mode-2 command currently executing.
    pub fn command_counter(&mut self) -> Result<u32> {
        self.value(&Command::CommandCounter, reply::parse_command_counter)
    }

    /// `@N<n>`: set the mode-2 command counter (e.g. 0 at the start of a job).
    pub fn set_command_counter(&mut self, value: u32) -> Result<()> {
        self.ack(&Command::SetCommandCounter(value))
    }

    /// `@O`: speed override in percent.
    pub fn override_pct(&mut self) -> Result<u16> {
        self.value(&Command::Override, reply::parse_override)
    }

    /// `@O<pct>`: set the speed override (10–200 %); takes effect immediately in all modes.
    pub fn set_override(&mut self, pct: u16) -> Result<()> {
        if !(10..=200).contains(&pct) {
            return Err(ControllerError::OverrideOutOfRange(pct));
        }
        self.ack(&Command::SetOverride(pct))
    }

    /// `@B` as a normal command, e.g. to end a run-until-stopped jog. Unlike
    /// [`StopHandle::brake`] this does not latch.
    pub fn brake(&mut self) -> Result<()> {
        self.ack(&Command::Brake)
    }

    /// Poll `@X` every `poll_interval` until `done` holds.
    ///
    /// Fails fast on a fault (error flag set: [`ControllerError::Fault`] with the `@E` code),
    /// on an unsolicited `*NNN` ([`ControllerError::AsyncError`]), on a stop, or when
    /// `timeout` passes ([`ControllerError::WaitTimeout`]). Never "carries on".
    pub fn wait_until(
        &mut self,
        timeout: Duration,
        mut done: impl FnMut(&Status) -> bool,
    ) -> Result<Status> {
        let deadline = Instant::now() + timeout;
        loop {
            let status = self.status()?;
            self.session.take_async_error()?;
            if status.error {
                let code = self.error_code().ok();
                return Err(ControllerError::Fault { status, code });
            }
            if done(&status) {
                return Ok(status);
            }
            if Instant::now() >= deadline {
                let timeout_ms = u64::try_from(timeout.as_millis()).unwrap_or(u64::MAX);
                return Err(ControllerError::WaitTimeout {
                    timeout_ms,
                    last: status,
                });
            }
            std::thread::sleep(self.session.cfg.poll_interval);
        }
    }

    /// Wait until no axis is moving.
    pub fn wait_idle(&mut self, timeout: Duration) -> Result<Status> {
        self.wait_until(timeout, Status::is_idle)
    }

    /// A handle that can brake or stop from any thread.
    pub fn stop_handle(&self) -> StopHandle {
        StopHandle {
            shared: Arc::clone(&self.session.shared),
        }
    }

    /// Receive every [`Event`] from now on.
    pub fn subscribe(&self) -> Receiver<Event> {
        self.session.shared.bus.subscribe()
    }

    /// After a stop: resynchronise with the controller, release the stop latch and return
    /// the current status.
    pub fn clear_stop(&mut self) -> Result<Status> {
        self.session.clear_stop()?;
        self.status()
    }

    /// `@M<n>`: switch mode (the controller only allows this at standstill, else `*028`).
    pub fn into_mode<N: ModeMarker>(
        mut self,
    ) -> std::result::Result<Controller<N>, ModeSwitchError<M>> {
        // A `*NNN` here is the reply to this command, even when leaving mode 2.
        match self.session.expect_ack(&Command::SetMode(N::MODE), false) {
            Ok(()) => Ok(Controller {
                session: self.session,
                _mode: PhantomData,
            }),
            Err(error) => Err(ModeSwitchError {
                controller: Box::new(self),
                error,
            }),
        }
    }

    pub fn into_parameters(
        self,
    ) -> std::result::Result<Controller<mode::Parameters>, ModeSwitchError<M>> {
        self.into_mode()
    }

    pub fn into_jog(self) -> std::result::Result<Controller<mode::Jog>, ModeSwitchError<M>> {
        self.into_mode()
    }

    pub fn into_cutting(
        self,
    ) -> std::result::Result<Controller<mode::Cutting>, ModeSwitchError<M>> {
        self.into_mode()
    }
}

// ------------------------------------------------------------------ mode 0

impl Controller<mode::Parameters> {
    /// `#G<entry>,<value>`: speed of a speed-table entry.
    pub fn set_speed(&mut self, entry: u8, value: u32) -> Result<()> {
        self.ack(&Command::SetSpeed { entry, value })
    }

    /// `#R<ms>`: maximum ramp time.
    pub fn set_ramp_ms(&mut self, ms: u16) -> Result<()> {
        self.ack(&Command::SetRampMs(ms))
    }

    /// `#C1,<ms>`: mode-2 dead time before a path starts.
    pub fn set_dead_time_ms(&mut self, ms: u16) -> Result<()> {
        self.ack(&Command::SetDeadTimeMs(ms))
    }

    /// `#C5`: 0x01 "positioned" signal on/off.
    pub fn set_positioned_signal(&mut self, on: bool) -> Result<()> {
        self.ack(&Command::SetPositionedSignal(on))
    }

    /// `#C7`: 0x03/0x02 buffer flow control on/off. Turn it off again before handing the
    /// machine back to GMC, which does not expect these bytes.
    pub fn set_flow_control(&mut self, on: bool) -> Result<()> {
        self.ack(&Command::SetFlowControl(on))
    }

    /// `#C19`: host watchdog in ms (0 = off, max 10000). While axes move the controller aborts
    /// with `*068` if no character arrives from the host for this long [SPEC1 p.26]. The session
    /// keeps talking during waits (`@X` polls, `@H` while the stream is held), each at
    /// `Config::poll_interval`, so choose a value well above that (e.g. ≥ 4×).
    pub fn set_host_watchdog_ms(&mut self, ms: u16) -> Result<()> {
        self.ack(&Command::SetHostWatchdogMs(ms))
    }

    /// `#C6`: 0x04 after homing. Keep it on: when off, the controller holds back the reply
    /// to `$H` until homing has finished, which exceeds any sensible reply deadline.
    pub fn set_homing_done_signal(&mut self, on: bool) -> Result<()> {
        self.ack(&Command::SetHomingDoneSignal(on))
    }

    /// `#C8`: 0x05 heartbeat after `ms` without traffic from the controller (0 = off).
    pub fn set_heartbeat_ms(&mut self, ms: u16) -> Result<()> {
        self.ack(&Command::SetHeartbeatMs(ms))
    }

    /// `#C35`: level-change bytes for signal input 1 (I221, start button: 0x06/0x07) and
    /// signal input 2 (I222: 0x08/0x09).
    pub fn set_signal_acks(&mut self, input1: bool, input2: bool) -> Result<()> {
        self.ack(&Command::SetSignalAcks { input1, input2 })
    }
}

// ------------------------------------------------------------------ mode 1

impl Controller<mode::Jog> {
    /// `$H<axes>`: start homing the axes in order. Returns when accepted; wait for completion
    /// with [`Controller::wait_until`] (`homed && !moving`) or the `HomingDone` signal.
    pub fn home(&mut self, axes: &[Axis]) -> Result<()> {
        self.ack(&Command::Home(axes.to_vec()))
    }

    /// `$A<hh>`: set the auxiliary outputs (head/clamp bitmask).
    pub fn set_outputs(&mut self, bits: u8) -> Result<()> {
        self.ack(&Command::SetOutputs(bits))
    }

    /// `$L<entry>,<axis><steps>`: start jogging one axis. Returns when accepted; the motion
    /// runs on. `steps = ±999_999` (relative) runs until [`Controller::brake`].
    pub fn jog_steps(
        &mut self,
        speed_entry: u8,
        axis: Axis,
        steps: i32,
        relative: bool,
    ) -> Result<()> {
        self.ack(&Command::JogSteps {
            speed_entry,
            axis,
            steps,
            relative,
        })
    }
}

// ------------------------------------------------------------------ mode 2

impl Controller<mode::Cutting> {
    /// `L<entry>,X<x>,Y<y>,Z<z>`: stream one absolute vector (steps). Returns as soon as it is
    /// written; the controller does not acknowledge mode-2 commands. Blocks while the
    /// controller signals "buffer almost full" (requires `#C7,1`).
    pub fn vector(&mut self, speed_entry: u8, x: i32, y: i32, z: i32) -> Result<()> {
        self.session.send_streamed(&Command::Vector {
            speed_entry,
            x,
            y,
            z,
        })
    }

    /// `A<hh>`: switch the auxiliary outputs at this point in the path (head/clamp valves).
    pub fn outputs(&mut self, bits: u8) -> Result<()> {
        self.session.send_streamed(&Command::StreamOutputs(bits))
    }

    /// `W<ms>`: dwell at this point in the path, timed by the controller.
    pub fn dwell_ms(&mut self, ms: u32) -> Result<()> {
        self.session.send_streamed(&Command::StreamDwellMs(ms))
    }

    /// `G<entry>,<v>`: change a speed-table entry at this point in the path.
    pub fn stream_speed(&mut self, entry: u8, value: u32) -> Result<()> {
        self.session
            .send_streamed(&Command::StreamSetSpeed { entry, value })
    }

    /// `I<n>` / `I-<n>`: wait here until input `n` is high / low.
    pub fn wait_input(&mut self, input: u8, high: bool) -> Result<()> {
        self.session
            .send_streamed(&Command::StreamWaitInput { input, high })
    }

    /// `C<bits>`: one-shot special functions for the next motion.
    pub fn flags(&mut self, flags: StreamFlags) -> Result<()> {
        self.session.send_streamed(&Command::StreamFlags(flags))
    }
}
