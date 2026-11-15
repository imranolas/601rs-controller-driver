//! Calibrating the 601 (CALIBRATION.md): the calibration sheet, where each of its cuts went,
//! and the fit of the blade model to the operator's readings. Pure: no I/O, no clock, so
//! `mc601` and an app (Bevel) use it alike, and each keeps its own files.
//!
//! - [`sheet`]: calibration sheet 2, in carriage coordinates, and the readings to take off it.
//! - [`sent`]: the cuts a program sent, in carriage millimetres, from the bytes on the line or
//!   from lowering the program again with the profile it was cut with. Lowering is pure, so an
//!   app that keeps no log still knows where every cut went.
//! - [`fit`]: the readings as linear rows in each head's unknowns (where its line lands on the
//!   registered corner, where each side of its slit lands, its drag), solved by least squares
//!   with each unknown's standard error; the combs read by eye.
//!
//! This is step 1 of CALIBRATION.md §10: the fit as `mc601 calibrate` had it, moved out of the
//! binary unchanged. The calibration file, the routines as plans and the proposals come next.
//!
//! Depends on `machine` and `program`.

pub mod fit;
pub mod sent;
pub mod sheet;

pub use fit::{CombReading, Face, Fit, Sheet, Unknown, fit_sheets};
pub use sent::{SentCut, cuts_from_bytes, cuts_from_lowering};

/// Why a sheet can't be fitted.
#[derive(Debug, thiserror::Error)]
pub enum Error {
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
