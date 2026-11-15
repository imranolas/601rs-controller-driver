//! Calibrating the 601 (CALIBRATION.md): the machine's calibration file, the calibration sheet,
//! where each of its cuts went, and the fit of the blade model to the operator's readings. Pure
//! but for [`store`]: no clock, so `mc601` and an app (Bevel) use it alike.
//!
//! - [`calibration`]: the file. The calibrated values over the driver's defaults (the shop's
//!   601 as fitted), the board profiles, and the history; what everything that cuts asks for
//!   the machine profile, the blade and a profile's material.
//! - [`key`]: what can be calibrated, each value's key (`machine.bevel.offset.x`), unit, words
//!   and bounds.
//! - [`store`]: where the file lives, shared by `mc601` and Bevel, and reading and writing it
//!   without one overwriting the other's change.
//! - [`sheet`]: calibration sheet 2, in carriage coordinates, and the readings to take off it.
//! - [`sent`]: the cuts a program sent, in carriage millimetres, from the bytes on the line or
//!   from lowering the program again with the profile it was cut with. Lowering is pure, so an
//!   app that keeps no log still knows where every cut went.
//! - [`fit`]: the readings as linear rows in each head's unknowns (where its line lands on the
//!   registered corner, where each side of its slit lands, its drag), solved by least squares
//!   with each unknown's standard error; the combs read by eye.
//!
//! Steps 1 and 2 of CALIBRATION.md §10. The routines as plans, and the fit's proposals applied
//! to the file, come next.
//!
//! Depends on `compiler`, `machine` and `program`.

pub mod calibration;
pub mod fit;
pub mod key;
pub mod sent;
pub mod sheet;
pub mod store;

pub use calibration::{Calibration, Change, Edit, Entry, Last, ValueState, Why};
pub use fit::{CombReading, Face, Fit, Sheet, Unknown, fit_sheets};
pub use key::{Key, Val};
pub use sent::{SentCut, cuts_from_bytes, cuts_from_lowering};

/// What went wrong.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The calibration file, or a value for it, isn't right; the message says how.
    #[error("{0}")]
    File(String),
    /// The calibration file changed since it was read: read it again, and redo the change.
    #[error(
        "the calibration has changed since it was read (revision {read}, now {now}): read it \
         again"
    )]
    Moved { read: u32, now: u32 },
    #[error("{path}: {message}")]
    Io { path: String, message: String },
    /// The sheet, its cuts and its readings don't agree; the message says how.
    #[error("{0}")]
    Sheet(String),
    /// The sheet can't be lowered with the profile it was cut with.
    #[error("the sheet can't be lowered: {0}")]
    Lower(#[from] machine::Error),
}

impl From<String> for Error {
    fn from(message: String) -> Self {
        Error::Sheet(message)
    }
}

impl From<&str> for Error {
    fn from(message: &str) -> Self {
        Error::Sheet(message.to_owned())
    }
}
