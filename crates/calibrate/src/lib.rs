//! Calibrating the 601 (CALIBRATION.md): the machine's calibration file, the two routines that
//! calibrate it, as plans an app can run, their sessions, the fit to the operator's readings,
//! and what it proposes. Pure but for [`store`]: no clock, so `mc601` and an app use it
//! alike.
//!
//! - [`calibration`]: the file. The calibrated values over the driver's defaults (the shop's
//!   601 as fitted), the board profiles, and the history; what everything that cuts asks for
//!   the machine profile, how each head cuts (the tool model, `compiler::Blade`), and a
//!   profile's material and thickness.
//! - [`key`]: what can be calibrated, each value's key (`machine.bevel.offset.x`), unit, words
//!   and bounds.
//! - [`store`]: where the file lives, shared by `mc601` and apps, and reading and writing it
//!   without one overwriting the other's change.
//! - [`reading`]: one reading, described once: what it measures, what it should read, where it
//!   is taken, and whether a value can be it.
//! - [`plan`]: a routine as data: the program to cut, and the readings to take off it in steps;
//!   [`heads`] (stage 1) and [`runons`] (stage 2) make them.
//! - [`session`]: one run of a routine, kept as a file: how far it was cut, the readings so far.
//! - [`fit`]: the caliper readings as linear rows in the tool model's values (where each head's
//!   line lands on the registered corner, its slit's offset, width and lean), solved by least
//!   squares with each unknown's standard error; and what a calibration predicts.
//! - [`proposal`]: what a routine's readings propose for the file, each change with its verdict
//!   and its effect in words, applied as one revision.
//! - [`sent`]: the cuts a program sent, in carriage millimetres, from lowering it again with the
//!   profile it was cut with. Lowering is pure, so no run log is needed.
//!
//! Depends on `compiler`, `machine` and `program`.

pub mod calibration;
pub mod fit;
pub mod heads;
pub mod key;
pub mod plan;
pub mod proposal;
pub mod reading;
pub mod runons;
pub mod sent;
pub mod session;
pub mod store;

pub use calibration::{Calibration, Change, Edit, Entry, Last, ValueState, Why};
pub use fit::{
    CALIPER_SD_MM, Fit, Leftover, Precision, Prior, Sheet, Unknown, fit_sheets, fit_sheets_with,
    known, precision, predict,
};
pub use key::{Key, Val};
pub use plan::{Plan, Routine, Step, Stock};
pub use proposal::{Proposal, Verdict};
pub use reading::{Check, Comb, CombReading, Edge, End, Kind, Reading, Side, Span, Tooth, combs};
pub use sent::SentCut;
pub use session::Session;

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
    /// A routine can't be planned on this board, or with this calibration.
    #[error("{0}")]
    Plan(String),
    /// A reading isn't one of the plan's, can't be taken yet, or a value can't be it.
    #[error("{0}")]
    Reading(String),
    /// The sheet, its cuts and its readings don't agree; the message says how.
    #[error("{0}")]
    Sheet(String),
    /// The sheet can't be lowered with the profile it was cut with.
    #[error("the sheet can't be lowered: {0}")]
    Lower(#[from] machine::Error),
}
