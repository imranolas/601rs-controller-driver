use std::fmt;
use std::io;
use std::sync::Arc;
use std::sync::atomic::Ordering;

use protocol::Command;

use crate::event::{Event, StopKind};
use crate::session::Shared;

/// Brakes or stops the machine from any thread, bypassing whatever the session is doing.
///
/// A stop **latches** the session: every operation returns
/// [`ControllerError::Stopped`](crate::ControllerError::Stopped) until
/// [`Controller::clear_stop`](crate::Controller::clear_stop) is called. The latch is set
/// *before* the stop takes the writer lock, and the command path checks it *under* that lock,
/// so nothing the session sends can follow the stop onto the wire.
#[derive(Clone)]
pub struct StopHandle {
    pub(crate) shared: Arc<Shared>,
}

impl fmt::Debug for StopHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("StopHandle")
            .field("stopped", &self.shared.stopped.load(Ordering::SeqCst))
            .finish()
    }
}

impl StopHandle {
    /// `@B`: stop all axes with a ramp. Positions are kept [SPEC p.13].
    pub fn brake(&self) -> io::Result<()> {
        self.issue(&Command::Brake, StopKind::Brake)
    }

    /// `@S`: stop immediately, without a ramp. Positions are lost, so the machine must be
    /// homed again [SPEC p.13]. For emergencies only.
    pub fn emergency_stop(&self) -> io::Result<()> {
        self.issue(&Command::EmergencyStop, StopKind::Emergency)
    }

    pub fn is_stopped(&self) -> bool {
        self.shared.stopped.load(Ordering::SeqCst)
    }

    fn issue(&self, cmd: &Command, kind: StopKind) -> io::Result<()> {
        self.shared.stopped.store(true, Ordering::SeqCst);
        self.shared.wake_waiters();
        self.shared.write(&cmd.encode())?;
        self.shared.bus.emit(Event::StopIssued(kind));
        Ok(())
    }
}
