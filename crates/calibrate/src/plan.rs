//! A routine as data (CALIBRATION.md §6): what to cut, and what to read off it once cut. The
//! program is in the board frame and is cut as any sheet is; where its cuts went comes from
//! lowering it again with the machine profile it was placed with, so no run log is needed.
//!
//! The geometry is the driver's: each reading carries where it is taken on the board, and what
//! it should read by the calibration the plan was made with, so a front end only draws
//! and asks.

use std::collections::HashMap;

use machine::Profile;
use program::{Board, Clamp, ClampSide, Cut, Head, Material, Op, Program};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::key::Surface;
use crate::{Calibration, Error, Sheet};

/// A calibration routine (CALIBRATION.md §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Routine {
    /// Stage 1: each head's position and slit, and the bevel's lean.
    Heads,
    /// Stage 2: each head's reach along the cut, on each face, by its combs.
    RunOns,
}

impl Routine {
    /// How the history and the program name it.
    pub fn name(self) -> &'static str {
        match self {
            Routine::Heads => "heads",
            Routine::RunOns => "runons",
        }
    }
}

/// The offcut both stages are cut on, square, mm.
pub const OFFCUT_MM: f64 = 150.0;

/// The board a routine is cut on.
#[derive(Debug, Clone, PartialEq)]
pub struct Stock {
    /// As measured with calipers, mm: the bevel's lean is its run across this, and the reach
    /// on the face is where the face lies.
    pub thickness_mm: f64,
    pub material: Material,
}

impl Stock {
    /// Profile `id`'s board, as the calibration file has it.
    ///
    /// # Errors
    /// When there's no such profile.
    pub fn profile(cal: &Calibration, id: &str) -> Result<Stock, Error> {
        Ok(Stock {
            thickness_mm: cal.thickness(id)?,
            material: cal.material(id)?,
        })
    }

    /// An offcut `thickness_mm` thick, cut as calibration sheet 2 was.
    pub fn offcut(thickness_mm: f64) -> Stock {
        Stock {
            thickness_mm,
            material: Material {
                name: "Calibration offcut".to_owned(),
                speed_pct: 95.0,
                head_settle_ms: 250,
                blade_angle_offset_deg: program::BladeOffsets::default(),
            },
        }
    }

    /// The board, in words, for the plan.
    pub(crate) fn need(&self) -> String {
        format!(
            "An offcut at least {OFFCUT_MM:.0} × {OFFCUT_MM:.0} mm, {} mm thick as measured with \
             calipers, face down into the front-left stops",
            self.thickness_mm
        )
    }

    pub(crate) fn check(&self) -> Result<(), Error> {
        let t = self.thickness_mm;
        if (0.5..=6.0).contains(&t) {
            Ok(())
        } else {
            Err(format!("calibration needs a board 0.5 to 6 mm thick, not {t} mm").into())
        }
    }
}

/// A cut a routine makes: its line from `a` to `b` on the board, mm.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Line {
    pub id: String,
    pub label: String,
    pub head: Head,
    pub a: [f64; 2],
    pub b: [f64; 2],
}

/// A routine's program on `stock`: the left clamp, then `lines` in order, its readings listed
/// as the fit reads them.
pub(crate) fn program(
    id: &str,
    routine: Routine,
    stock: &Stock,
    lines: &[Line],
    readings: Vec<Value>,
) -> Program {
    let mut ops = vec![Op::Clamp(Clamp {
        id: None,
        label: None,
        side: ClampSide::Left,
        turned: Vec::new(),
    })];
    ops.extend(lines.iter().map(|l| {
        Op::Cut(Cut {
            id: Some(l.id.clone()),
            label: Some(l.label.clone()),
            head: l.head,
            path_mm: vec![l.a, l.b],
            speed_pct: None,
        })
    }));
    Program {
        format: program::FORMAT.to_owned(),
        version: program::VERSION,
        id: id.to_owned(),
        meta: Some(json!({
            "source": format!("calibrate: {} of the 601's calibration (CALIBRATION.md §5)", routine.name()),
            "calibration": {
                "version": 2, "frame": "board", "routine": routine.name(),
                "readings": readings,
            },
        })),
        board: Board {
            width_mm: OFFCUT_MM,
            height_mm: OFFCUT_MM,
            thickness_mm: stock.thickness_mm,
        },
        material: stock.material.clone(),
        ops,
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

/// One reading.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Reading {
    /// "W.width".
    pub id: String,
    /// "The window, left to right (inside jaws)".
    pub prompt: String,
    /// The side of the board up as it is read.
    pub surface: Surface,
    pub kind: Kind,
    /// Where it is taken, in the board frame: from one edge to the other, mm; for a comb, its
    /// reference.
    pub dimension: [[f64; 2]; 2],
    /// The program's cuts it reads: if one of them didn't finish, it can't be taken.
    pub needs: Vec<String>,
}

/// What a reading is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// A length, by calipers, mm.
    Length {
        /// What it should read by the calibration the plan was made with.
        expected_mm: f64,
        /// Further than this from what's expected, ask before keeping it.
        unusual_mm: f64,
    },
    /// A comb's first tooth to come free at its reference, by flexing the board: 1 to `teeth`
    /// from the shortest, or 0 if none does. Its teeth are the program's cuts `cuts`, tooth 1
    /// first.
    Tooth { teeth: usize, cuts: Vec<String> },
}

impl Reading {
    /// What a length should read; none for a comb.
    pub fn expected_mm(&self) -> Option<f64> {
        match self.kind {
            Kind::Length { expected_mm, .. } => Some(expected_mm),
            Kind::Tooth { .. } => None,
        }
    }
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
    /// When the plan has no such reading, or the value isn't a length, or a tooth of the comb.
    pub fn check(&self, id: &str, value: f64) -> Result<Check, Error> {
        let r = self
            .reading(id)
            .ok_or_else(|| format!("{id} isn't a reading of this plan"))?;
        match r.kind {
            Kind::Length {
                expected_mm,
                unusual_mm,
            } => {
                if !(value.is_finite() && value > 0.0) {
                    return Err(format!("{id}: {value} isn't a length").into());
                }
                let off_mm = (value - expected_mm).abs();
                Ok(if off_mm > unusual_mm {
                    Check::Unusual { off_mm }
                } else {
                    Check::Fine
                })
            }
            Kind::Tooth { teeth, .. } => {
                if value.fract() == 0.0 && (0.0..=teeth as f64).contains(&value) {
                    Ok(Check::Fine)
                } else {
                    Err(
                        format!("{id}: give the tooth, 1 to {teeth}, or 0 if none came free")
                            .into(),
                    )
                }
            }
        }
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
