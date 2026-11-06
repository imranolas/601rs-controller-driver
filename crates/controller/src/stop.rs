use std::fmt;
use std::sync::Arc;

use crate::Result;
use crate::event::StopKind;
use crate::session::Shared;

/// Brakes or stops the machine from any thread, bypassing whatever the session is doing.
///
/// A stop **latches** the session: the request in progress finishes as
/// [`ControllerError::Stopped`](crate::ControllerError::Stopped), and so does every operation
/// after it, until [`Controller::clear_stop`](crate::Controller::clear_stop) is called. The latch
/// is set and the stop written in one step under the lock every write takes, so nothing the
/// session sends can follow the stop onto the wire.
#[derive(Clone)]
pub struct StopHandle {
    pub(crate) shared: Arc<Shared>,
}

impl fmt::Debug for StopHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("StopHandle")
            .field("stopped", &self.is_stopped())
            .finish_non_exhaustive()
    }
}

impl StopHandle {
    /// `@B`: stop all axes with a ramp. Positions are kept [SPEC p.13].
    pub fn brake(&self) -> Result<()> {
        self.shared.stop(StopKind::Brake)
    }

    /// `@S`: stop immediately, without a ramp. Positions are lost, so the machine must be
    /// homed again [SPEC p.13]. For emergencies only.
    pub fn emergency_stop(&self) -> Result<()> {
        self.shared.stop(StopKind::Emergency)
    }

    pub fn is_stopped(&self) -> bool {
        self.shared.is_stopped()
    }
}
