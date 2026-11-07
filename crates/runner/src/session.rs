//! The controller session across its modes. `controller::Controller<M>` only offers what mode
//! `M` accepts, and a mode switch consumes it; the runner needs one value it can keep, so this
//! holds whichever mode the controller is in and switches when a command needs another.

use std::time::Duration;

use controller::{Controller, ControllerError, Mode, Setting, SettingKey, Status, mode};
use protocol::{Axis, Command};

enum Ctl {
    Parameters(Controller<mode::Parameters>),
    Jog(Controller<mode::Jog>),
    Cutting(Controller<mode::Cutting>),
}

/// The same call on whichever mode the controller is in (master commands).
macro_rules! any {
    ($ctl:expr, $c:ident => $body:expr) => {
        match $ctl {
            Ctl::Parameters($c) => $body,
            Ctl::Jog($c) => $body,
            Ctl::Cutting($c) => $body,
        }
    };
}

pub(crate) struct Session {
    /// `None` only if a mode switch failed and lost the controller, which `into_mode` never
    /// does (it hands the controller back), so in practice always `Some`.
    ctl: Option<Ctl>,
}

/// Why a session call failed: the controller said so, or the session is gone.
#[derive(Debug)]
pub(crate) enum SessionError {
    Controller(ControllerError),
    Lost,
}

impl From<ControllerError> for SessionError {
    fn from(e: ControllerError) -> Self {
        SessionError::Controller(e)
    }
}

type R<T> = std::result::Result<T, SessionError>;

impl Session {
    pub(crate) fn new(ctl: Controller<mode::Parameters>) -> Self {
        Self {
            ctl: Some(Ctl::Parameters(ctl)),
        }
    }

    fn ctl(&mut self) -> R<&mut Ctl> {
        self.ctl.as_mut().ok_or(SessionError::Lost)
    }

    pub(crate) fn mode(&self) -> Option<Mode> {
        self.ctl.as_ref().map(|c| match c {
            Ctl::Parameters(_) => Mode::Parameters,
            Ctl::Jog(_) => Mode::Jog,
            Ctl::Cutting(_) => Mode::Cutting,
        })
    }

    /// Switch to `to` (`@M<n>`), unless already there. The controller refuses while anything
    /// moves (`*028`); a refused switch leaves the session in its old mode.
    pub(crate) fn switch(&mut self, to: Mode) -> R<()> {
        if self.mode() == Some(to) {
            return Ok(());
        }
        let ctl = self.ctl.take().ok_or(SessionError::Lost)?;
        macro_rules! go {
            ($c:expr, $back:path) => {
                match to {
                    Mode::Parameters => $c.into_parameters().map(Ctl::Parameters),
                    Mode::Jog => $c.into_jog().map(Ctl::Jog),
                    Mode::Cutting => $c.into_cutting().map(Ctl::Cutting),
                }
                .map_err(|e| ($back(*e.controller), e.error))
            };
        }
        let switched = match ctl {
            Ctl::Parameters(c) => go!(c, Ctl::Parameters),
            Ctl::Jog(c) => go!(c, Ctl::Jog),
            Ctl::Cutting(c) => go!(c, Ctl::Cutting),
        };
        match switched {
            Ok(ctl) => {
                self.ctl = Some(ctl);
                Ok(())
            }
            Err((ctl, e)) => {
                self.ctl = Some(ctl);
                Err(e.into())
            }
        }
    }

    pub(crate) fn status(&mut self) -> R<Status> {
        Ok(any!(self.ctl()?, c => c.status())?)
    }

    pub(crate) fn wait_until(
        &mut self,
        timeout: Duration,
        done: impl FnMut(&Status) -> bool,
    ) -> R<Status> {
        Ok(any!(self.ctl()?, c => c.wait_until(timeout, done))?)
    }

    pub(crate) fn wait_idle(&mut self, timeout: Duration) -> R<Status> {
        self.wait_until(timeout, Status::is_idle)
    }

    pub(crate) fn read_input(&mut self, input: u8) -> R<bool> {
        Ok(any!(self.ctl()?, c => c.read_input(input))?)
    }

    pub(crate) fn brake(&mut self) -> R<()> {
        Ok(any!(self.ctl()?, c => c.brake())?)
    }

    pub(crate) fn clear_stop(&mut self) -> R<Status> {
        Ok(any!(self.ctl()?, c => c.clear_stop())?)
    }

    pub(crate) fn clear_error(&mut self) -> R<()> {
        Ok(any!(self.ctl()?, c => c.clear_error())?)
    }

    pub(crate) fn position(&mut self) -> R<[i32; 3]> {
        let mut at = [0; 3];
        for (k, axis) in Axis::ALL.into_iter().enumerate() {
            at[k] = any!(self.ctl()?, c => c.position_steps(axis))?;
        }
        Ok(at)
    }

    pub(crate) fn set(&mut self, s: Setting) -> R<()> {
        self.switch(Mode::Parameters)?;
        match self.ctl()? {
            Ctl::Parameters(c) => Ok(c.set(s)?),
            _ => Err(SessionError::Lost),
        }
    }

    pub(crate) fn read_setting(&mut self, key: SettingKey) -> R<Setting> {
        self.switch(Mode::Parameters)?;
        match self.ctl()? {
            Ctl::Parameters(c) => Ok(c.read_setting(key)?),
            _ => Err(SessionError::Lost),
        }
    }

    pub(crate) fn set_outputs(&mut self, bits: u8) -> R<()> {
        self.switch(Mode::Jog)?;
        match self.ctl()? {
            Ctl::Jog(c) => Ok(c.set_outputs(bits)?),
            _ => Err(SessionError::Lost),
        }
    }

    pub(crate) fn home(&mut self, axes: &[Axis]) -> R<()> {
        self.switch(Mode::Jog)?;
        match self.ctl()? {
            Ctl::Jog(c) => Ok(c.home(axes)?),
            _ => Err(SessionError::Lost),
        }
    }

    pub(crate) fn stream(&mut self, cmd: &Command) -> R<()> {
        self.switch(Mode::Cutting)?;
        match self.ctl()? {
            Ctl::Cutting(c) => Ok(c.stream(cmd)?),
            _ => Err(SessionError::Lost),
        }
    }
}
