//! A routine as data (CALIBRATION.md §6): what to cut, and what to read off it once cut. The
//! program is in the board frame and is cut as any sheet is; where its cuts went comes from
//! lowering it again with the machine profile it was placed with, so no run log is needed.
//!
//! The geometry is the driver's: each reading carries where it is taken on the board, and what
//! it should read by the calibration the plan was made with, so a front end (Bevel) only draws
//! and asks.

use std::collections::HashMap;

use machine::Profile;
use program::Program;
use serde::{Deserialize, Serialize};

use crate::key::Surface;
use crate::{Error, Sheet};

/// A calibration routine (CALIBRATION.md §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Routine {
    /// Stage 1: each head's position and slit, and the bevel's lean.
    Heads,
}

impl Routine {
    /// How the history and the program name it.
    pub fn name(self) -> &'static str {
        match self {
            Routine::Heads => "heads",
        }
    }
}

/// What to cut and what to read.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Plan {
    pub routine: Routine,
    /// The board it needs, in words.
    pub board: String,
    /// What to cut, in the board frame; its readings are listed in its
    /// `meta.calibration.readings`, which the fit reads.
    pub program: Program,
    /// The machine profile it was placed with, the calibration's: lowering the program with it
    /// gives where the carriage went.
    pub cut_with: Profile,
    /// The readings, grouped by how the pieces are held, in the order to take them.
    pub steps: Vec<Step>,
    pub readings: Vec<Reading>,
}

/// Readings taken with the pieces held one way.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Step {
    /// "Lift the mount out of the frame, push out the window, and lay the mount back up."
    pub instruction: String,
    /// Reading ids, in order.
    pub readings: Vec<String>,
}

/// One reading, by calipers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Reading {
    /// "W.width".
    pub id: String,
    /// "The window, left to right (inside jaws)".
    pub prompt: String,
    /// The side of the board up as it is read.
    pub surface: Surface,
    /// What it should read by the calibration the plan was made with, mm.
    pub expected_mm: f64,
    /// Further than this from what's expected, ask before keeping it, mm.
    pub unusual_mm: f64,
    /// Where it is taken, in the board frame: from one edge to the other, mm.
    pub dimension: [[f64; 2]; 2],
}

/// A value as typed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Check {
    Fine,
    /// Further from what's expected than a misread usually is: ask before keeping it.
    Unusual {
        off_mm: f64,
    },
}

impl Plan {
    pub fn reading(&self, id: &str) -> Option<&Reading> {
        self.readings.iter().find(|r| r.id == id)
    }

    /// Whether `value` can be reading `id`, and whether it is unusual.
    ///
    /// # Errors
    /// When the plan has no such reading, or the value isn't a length.
    pub fn check(&self, id: &str, value: f64) -> Result<Check, Error> {
        let r = self
            .reading(id)
            .ok_or_else(|| format!("{id} isn't a reading of this plan"))?;
        if !(value.is_finite() && value > 0.0) {
            return Err(format!("{id}: {value} isn't a length").into());
        }
        let off_mm = (value - r.expected_mm).abs();
        Ok(if off_mm > r.unusual_mm {
            Check::Unusual { off_mm }
        } else {
            Check::Fine
        })
    }

    /// The cut sheet with `readings`: where its cuts went, by lowering the program with the
    /// profile it was placed with.
    ///
    /// # Errors
    /// When a reading isn't the plan's, or the program no longer lowers.
    pub fn sheet(
        &self,
        label: impl Into<String>,
        readings: HashMap<String, f64>,
    ) -> Result<Sheet, Error> {
        if let Some(id) = readings.keys().find(|id| self.reading(id).is_none()) {
            return Err(format!("{id} isn't a reading of this plan").into());
        }
        Sheet::lowered(label, self.program.clone(), &self.cut_with, readings)
    }
}
