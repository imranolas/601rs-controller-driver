//! The typed API: [`Controller<M>`] exposes only the commands its mode accepts.

use std::fmt;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use link::Link;
use protocol::{
    Axis, Command, ErrorCode, InputDef, JogTarget, Level, Mode, OverridePct, Param, Reply, Setting,
    SettingKey, Side, Status, StreamFlags, Switches, reply,
};

use crate::error::millis;
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

/// Open a session: resync (unique `@I1xx` probe, see [`crate::RESYNC_PROBE_INPUTS`]), read the
/// version (`@V`), enter mode 0. Returns the controller and its version string
/// (e.g. `@V2.00/55W 19.10.10 2807001`). If the controller never answers the probe, the
/// [`ControllerError::ResyncFailed`] says what came back instead.
pub fn connect(link: Link, cfg: Config) -> Result<(Controller<mode::Parameters>, String)> {
    let mut session = Session::start(link, cfg)?;
    session.resync()?;
    let version = match session.request(&Command::Version)? {
        Reply::Value(v) if v.starts_with("@V") => v,
        other => return Err(session.unexpected(&Command::Version, &other)),
    };
    session.expect_ack(&Command::SetMode(Mode::Parameters))?;
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
    fn query(&mut self, cmd: &Command) -> Result<Reply> {
        self.session.request(cmd)
    }

    fn ack(&mut self, cmd: &Command) -> Result<()> {
        self.session.expect_ack(cmd)
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

    /// Send `command` and a probe in one write, without waiting between them, and return the
    /// command's reply: the reply-order check `gunnar check` runs. Reply pairing relies on the
    /// controller answering in the order it is sent; if the probe is answered first, this
    /// fails with [`ControllerError::Overtaken`]. A rejection is returned as
    /// [`ControllerError::Rejected`], as by any other call. `command` must be one this mode
    /// takes, and not a switch to another mode, which only [`Self::into_mode`] makes
    /// ([`ControllerError::WrongMode`] if not; nothing is sent).
    pub fn request_with_probe(&mut self, command: &Command) -> Result<Reply> {
        let takes = command.expects_reply()
            && command.required_mode().is_none_or(|mode| mode == M::MODE)
            && !matches!(command, Command::SetMode(to) if *to != M::MODE);
        if !takes {
            return Err(ControllerError::WrongMode {
                command: command.clone(),
                mode: M::MODE,
            });
        }
        self.session.request_with_probe(command)
    }

    /// `@E`: current error code (0 = none).
    pub fn error_code(&mut self) -> Result<ErrorCode> {
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

    /// `@O`: the speed override.
    pub fn override_pct(&mut self) -> Result<OverridePct> {
        self.value(&Command::Override, reply::parse_override)
    }

    /// `@@`: clear the controller's error state. Not in the spec; GMC sends it before homing
    /// and after faults [CAP g_init].
    pub fn clear_error(&mut self) -> Result<()> {
        self.ack(&Command::ClearError)
    }

    /// `@O<pct>`: set the speed override; takes effect immediately in all modes.
    pub fn set_override(&mut self, pct: OverridePct) -> Result<()> {
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
                let timeout_ms = millis(timeout);
                return Err(ControllerError::WaitTimeout {
                    timeout_ms,
                    last: status,
                });
            }
            self.session.pause(self.session.poll_interval())?;
        }
    }

    /// Wait until no axis is moving.
    pub fn wait_idle(&mut self, timeout: Duration) -> Result<Status> {
        self.wait_until(timeout, Status::is_idle)
    }

    /// A handle that can brake or stop from any thread.
    pub fn stop_handle(&self) -> StopHandle {
        StopHandle {
            shared: Arc::clone(self.session.shared()),
        }
    }

    /// Receive every [`Event`] from now on.
    pub fn subscribe(&self) -> Receiver<Event> {
        self.session.shared().bus.subscribe()
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
        match self.session.expect_ack(&Command::SetMode(N::MODE)) {
            Ok(()) => {
                // The type and the controller's acknowledged mode change together.
                debug_assert_eq!(self.session.mode(), Some(N::MODE));
                Ok(Controller {
                    session: self.session,
                    _mode: PhantomData,
                })
            }
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
    /// `#…`: change a setting. See [`Setting`] for what each one does.
    pub fn set(&mut self, setting: Setting) -> Result<()> {
        self.ack(&Command::Set(setting))
    }

    /// Read a setting back (`&…`), typed, ready to be restored later with [`Self::set`].
    pub fn read_setting(&mut self, key: SettingKey) -> Result<Setting> {
        self.value(&Command::QueryParam(key.param()), |v| {
            Setting::from_reply(key, v)
        })
    }

    /// `&I<n>`: how logical input `n` is wired, or whether it is wired at all.
    pub fn input_def(&mut self, input: u8) -> Result<InputDef> {
        self.value(&Command::QueryParam(Param::InputDef(input)), |v| {
            reply::parse_input_def(v, input)
        })
    }

    /// `&D<a>`: the end `axis` homes towards. Any reply other than direction 0 or 1 is an
    /// [`ControllerError::UnexpectedReply`], never a guess.
    pub fn homing_side(&mut self, axis: Axis) -> Result<Side> {
        self.value(&Command::QueryParam(Param::Homing(axis)), |v| {
            reply::parse_homing_side(v, axis)
        })
    }

    /// Read any parameter back as the raw fields of its reply (`&C13` → `["1"]`,
    /// `&DX` → `["0","10","11","0"]`), for diagnostics. Prefer the typed reads above.
    pub fn query_param(&mut self, param: Param) -> Result<Vec<String>> {
        self.value(&Command::QueryParam(param), |v| {
            reply::parse_param(v, param)
        })
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

    /// `$L<entry>,<axis><steps>`: start jogging one axis to or by a number of steps. Returns
    /// when accepted; the motion runs on. A relative jog of
    /// [`JogTarget::UNTIL_BRAKED`] runs until [`Controller::brake`].
    pub fn jog_steps(&mut self, speed_entry: u8, axis: Axis, target: JogTarget) -> Result<()> {
        self.ack(&Command::JogSteps {
            speed_entry,
            axis,
            target,
        })
    }
}

// ------------------------------------------------------------------ mode 2

impl Controller<mode::Cutting> {
    /// `L<entry>,X<x>,Y<y>,Z<z>`: stream one absolute vector (steps). Returns as soon as it is
    /// written; the controller does not acknowledge mode-2 commands. Blocks while the
    /// controller signals "buffer almost full" (requires `#C7,1`).
    pub fn vector(&mut self, speed_entry: u8, x: i32, y: i32, z: i32) -> Result<()> {
        self.session.stream(&Command::Vector {
            speed_entry,
            x,
            y,
            z,
        })
    }

    /// Stream a path through `points` (absolute, in steps) at speed entry `speed_entry`, in the
    /// compact form of [`protocol::Command::path`]: about half the bytes per point of
    /// [`Self::vector`], which is what lets a dense curve keep up with its feed over the serial
    /// line. Each call starts with an absolute vector, so nothing depends on what was streamed
    /// before it (a stop, a mode switch).
    pub fn path(&mut self, speed_entry: u8, points: &[[i32; 3]]) -> Result<()> {
        for command in Command::path(speed_entry, points) {
            self.session.stream(&command)?;
        }
        Ok(())
    }

    /// `A<hh>`: switch the auxiliary outputs at this point in the path (head/clamp valves).
    pub fn outputs(&mut self, bits: u8) -> Result<()> {
        self.session.stream(&Command::StreamOutputs(bits))
    }

    /// `W<ms>`: dwell at this point in the path, timed by the controller.
    pub fn dwell_ms(&mut self, ms: u32) -> Result<()> {
        self.session.stream(&Command::StreamDwellMs(ms))
    }

    /// `G<entry>,<v>`: change a speed-table entry at this point in the path.
    pub fn stream_speed(&mut self, entry: u8, value: u32) -> Result<()> {
        self.session
            .stream(&Command::StreamSetSpeed { entry, value })
    }

    /// `I<n>` / `I-<n>`: wait here until input `n` is at `level`.
    pub fn wait_input(&mut self, input: u8, level: Level) -> Result<()> {
        self.session
            .stream(&Command::StreamWaitInput { input, level })
    }

    /// `C<bits>`: one-shot special functions for the next motion.
    pub fn flags(&mut self, flags: StreamFlags) -> Result<()> {
        self.session.stream(&Command::StreamFlags(flags))
    }

    /// Stream any mode-2 command (a vector, a follow-on step, `A`, `W`, `C`, …), as a lowered
    /// job lists them. Anything that isn't a mode-2 command is refused without being sent.
    pub fn stream(&mut self, command: &Command) -> Result<()> {
        if command.required_mode() != Some(Mode::Cutting) {
            return Err(ControllerError::WrongMode {
                command: command.clone(),
                mode: Mode::Cutting,
            });
        }
        self.session.stream(command)
    }
}
