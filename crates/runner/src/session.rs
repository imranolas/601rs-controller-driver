//! The controller session across its modes. `controller::Controller<M>` only offers what mode
//! `M` accepts, and a mode switch consumes it; the runner needs one value it can keep, so this
//! holds whichever mode the controller is in and switches when a command needs another.

use std::time::Duration;

use controller::{Controller, Mode, ModeMarker, Setting, SettingKey, Status, mode};
use protocol::{Axis, Command};

use crate::error::Cause;

enum Ctl {
    Parameters(Controller<mode::Parameters>),
    Jog(Controller<mode::Jog>),
    Cutting(Controller<mode::Cutting>),
}

impl From<Controller<mode::Parameters>> for Ctl {
    fn from(c: Controller<mode::Parameters>) -> Self {
        Ctl::Parameters(c)
    }
}

impl From<Controller<mode::Jog>> for Ctl {
    fn from(c: Controller<mode::Jog>) -> Self {
        Ctl::Jog(c)
    }
}

impl From<Controller<mode::Cutting>> for Ctl {
    fn from(c: Controller<mode::Cutting>) -> Self {
        Ctl::Cutting(c)
    }
}

/// The same call on whichever mode the controller is in (master commands). A macro, because
/// the call is generic over the mode and a closure can't be.
macro_rules! any {
    ($ctl:expr, $c:ident => $body:expr) => {
        match $ctl {
            Ctl::Parameters($c) => $body,
            Ctl::Jog($c) => $body,
            Ctl::Cutting($c) => $body,
        }
    };
}

/// Switch `c` to `to`. A refused switch hands the controller back, still in its mode.
fn switch<M: ModeMarker>(c: Controller<M>, to: Mode) -> (Ctl, Result<(), Cause>)
where
    Ctl: From<Controller<M>>,
{
    let switched = match to {
        Mode::Parameters => c.into_parameters().map(Ctl::Parameters),
        Mode::Jog => c.into_jog().map(Ctl::Jog),
        Mode::Cutting => c.into_cutting().map(Ctl::Cutting),
    };
    match switched {
        Ok(ctl) => (ctl, Ok(())),
        Err(e) => (Ctl::from(*e.controller), Err(e.error.into())),
    }
}

type R<T> = Result<T, Cause>;

pub(crate) struct Session {
    /// `None` only while a mode switch is under way, or if one panicked part-way.
    ctl: Option<Ctl>,
}

impl Session {
    pub(crate) fn new(ctl: Controller<mode::Parameters>) -> Self {
        Self {
            ctl: Some(Ctl::Parameters(ctl)),
        }
    }

    fn ctl(&mut self) -> R<&mut Ctl> {
        self.ctl.as_mut().ok_or(Cause::Lost)
    }

    pub(crate) fn mode(&self) -> Option<Mode> {
        self.ctl.as_ref().map(|c| any!(c, c => c.mode()))
    }

    /// Switch to `to` (`@M<n>`), unless already there. The controller refuses while anything
    /// moves (`*028`); a refused switch leaves the session in its old mode.
    pub(crate) fn switch(&mut self, to: Mode) -> R<()> {
        if self.mode() == Some(to) {
            return Ok(());
        }
        let ctl = self.ctl.take().ok_or(Cause::Lost)?;
        let (ctl, result) = any!(ctl, c => switch(c, to));
        self.ctl = Some(ctl);
        result
    }

    /// The controller in mode 0, switching to it if need be.
    fn params(&mut self) -> R<&mut Controller<mode::Parameters>> {
        self.switch(Mode::Parameters)?;
        match self.ctl()? {
            Ctl::Parameters(c) => Ok(c),
            _ => unreachable!("a switch to mode 0 that succeeded leaves the session in it"),
        }
    }

    /// The controller in mode 1, switching to it if need be.
    fn jog(&mut self) -> R<&mut Controller<mode::Jog>> {
        self.switch(Mode::Jog)?;
        match self.ctl()? {
            Ctl::Jog(c) => Ok(c),
            _ => unreachable!("a switch to mode 1 that succeeded leaves the session in it"),
        }
    }

    /// The controller in mode 2, switching to it if need be.
    fn cutting(&mut self) -> R<&mut Controller<mode::Cutting>> {
        self.switch(Mode::Cutting)?;
        match self.ctl()? {
            Ctl::Cutting(c) => Ok(c),
            _ => unreachable!("a switch to mode 2 that succeeded leaves the session in it"),
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
        Ok(self.params()?.set(s)?)
    }

    pub(crate) fn read_setting(&mut self, key: SettingKey) -> R<Setting> {
        Ok(self.params()?.read_setting(key)?)
    }

    pub(crate) fn set_outputs(&mut self, bits: u8) -> R<()> {
        Ok(self.jog()?.set_outputs(bits)?)
    }

    pub(crate) fn home(&mut self, axes: &[Axis]) -> R<()> {
        Ok(self.jog()?.home(axes)?)
    }

    pub(crate) fn stream(&mut self, cmd: &Command) -> R<()> {
        Ok(self.cutting()?.stream(cmd)?)
    }
}
