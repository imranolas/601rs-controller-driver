//! What can go wrong, and how far it got.

use controller::{ControllerError, Setting};
use protocol::{Command, Status};

/// Why something failed on the controller's side.
#[derive(Debug, thiserror::Error)]
pub enum Cause {
    #[error(transparent)]
    Controller(#[from] ControllerError),
    /// The session is gone: a mode switch panicked part-way.
    #[error("the controller session is gone")]
    Lost,
    /// The lowered job asked for a command the runner doesn't send: a bug in the lowering.
    #[error("`{0}` is not a command the runner sends (a bug)")]
    Unsupported(Command),
}

/// A controller setting that reads back other than it was written.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("{} reads `{read}` after `{wanted}`", .wanted.key())]
pub struct Mismatch {
    pub wanted: Setting,
    pub read: Setting,
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The program can't be cut on this machine (machine::lower refused it): nothing was sent.
    #[error("the program can't be cut: {0}")]
    Invalid(machine::Error),
    /// The machine isn't homed, or hasn't been since a stop or a failure: home it first.
    #[error("the machine must be homed first")]
    NotHomed,
    /// Homing ended without the controller knowing its position.
    #[error("homing ended without a home position (status {0})")]
    HomingIncomplete(Status),
    /// Settings of the profile's that the controller won't keep.
    #[error("the controller's configuration won't take: {}", list(.0))]
    Misconfigured(Vec<Mismatch>),
    /// Talking to the controller failed while nothing was moving: a read, or a setting
    /// written while idle. Nothing needed making safe, and homing is not lost.
    #[error("the controller: {0}")]
    Controller(Cause),
    /// Something failed in op `op` (`None`: outside the program's ops). The runner then
    /// stopped and made the machine safe: heads up, clamps released (`safe` says whether that
    /// worked). It must be homed again before it moves.
    #[error("failed {}: {cause}; {}; home before moving again", at(*.op), then(*.safe))]
    Failed {
        op: Option<usize>,
        cause: Cause,
        safe: bool,
    },
}

impl From<Cause> for Error {
    fn from(cause: Cause) -> Self {
        Error::Controller(cause)
    }
}

fn list(mismatches: &[Mismatch]) -> String {
    let all: Vec<String> = mismatches.iter().map(ToString::to_string).collect();
    all.join("; ")
}

fn at(op: Option<usize>) -> String {
    op.map_or_else(
        || "outside the program's ops".to_owned(),
        |o| format!("in op {o}"),
    )
}

fn then(safe: bool) -> &'static str {
    if safe {
        "heads up, clamps released"
    } else {
        "the safe state could NOT be confirmed"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_failure_says_where_and_how_it_was_left() {
        let e = Error::Failed {
            op: Some(3),
            cause: Cause::Lost,
            safe: false,
        };
        assert_eq!(
            e.to_string(),
            "failed in op 3: the controller session is gone; the safe state could NOT be \
             confirmed; home before moving again"
        );
    }
}
