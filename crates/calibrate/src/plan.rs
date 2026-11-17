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
use serde_json::json;

use crate::fit::predict;
use crate::key::Surface;
use crate::reading::{Check, Kind, Reading};
use crate::{Calibration, Error, Sheet};

/// A calibration routine (CALIBRATION.md §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
            Err(Error::Plan(format!(
                "calibration needs a board 0.5 to 6 mm thick, not {t} mm"
            )))
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

/// A routine's program on `stock`: the left clamp, then `lines` in order. Its meta names the
/// routine, which marks it a calibration sheet; its readings are the plan's.
pub(crate) fn program(id: &str, routine: Routine, stock: &Stock, lines: &[Line]) -> Program {
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
            "calibration": { "frame": "board", "routine": routine.name() },
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct Plan {
    pub routine: Routine,
    /// The board it needs, in words.
    pub board: String,
    /// What to cut, in the board frame.
    pub program: Program,
    /// The machine profile it was placed with, the calibration's: lowering the program with it
    /// gives where the carriage went. Its schema is any object: an app keeps it, never reads it.
    #[cfg_attr(
        feature = "schemars",
        schemars(with = "serde_json::Map<String, serde_json::Value>")
    )]
    pub cut_with: Profile,
    /// The readings, grouped by how the pieces are held, in the order to take them.
    pub steps: Vec<Step>,
    pub readings: Vec<Reading>,
}

/// Readings taken with the pieces held one way.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct Step {
    /// "Lift the mount out of the frame, push out the window, and lay the mount back up."
    pub instruction: String,
    /// The pieces as they're held for it, so a front end draws exactly those: the mount alone
    /// with its window open, the frame alone, the drop-out face up.
    #[serde(default)]
    pub pieces: Vec<Piece>,
    /// Reading ids, in order.
    pub readings: Vec<String>,
}

/// A piece of the cut board as it's held, in the board frame, mm: its outline, the holes in
/// it, and the side up. Its edges are where the readings' dimensions end.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct Piece {
    /// "the mount".
    pub name: String,
    pub surface: Surface,
    /// Its outline, a closed polygon (the first point not repeated).
    pub outline: Vec<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub holes: Vec<Vec<[f64; 2]>>,
}

impl Piece {
    /// The rectangle `[x0, y0, x1, y1]`, anticlockwise from its front-left corner.
    pub(crate) fn rectangle(name: &str, surface: Surface, [x0, y0, x1, y1]: [f64; 4]) -> Piece {
        Piece {
            name: name.to_owned(),
            surface,
            outline: vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
            holes: Vec::new(),
        }
    }

    /// The piece with the rectangle `[x0, y0, x1, y1]` open in it.
    pub(crate) fn with_hole(mut self, [x0, y0, x1, y1]: [f64; 4]) -> Piece {
        self.holes
            .push(vec![[x0, y0], [x0, y1], [x1, y1], [x1, y0]]);
        self
    }
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
        self.reading(id)
            .ok_or_else(|| Error::Reading(format!("{id} isn't a reading of this plan")))?
            .check(value)
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
            return Err(Error::Reading(format!("{id} isn't a reading of this plan")));
        }
        Ok(Sheet {
            label: label.into(),
            cuts: crate::sent::cuts(&self.program, &self.cut_with)?,
            plan: self.clone(),
            readings,
        })
    }

    /// The plan with each length's expected value, by calibration `cal`, where its cuts go.
    ///
    /// # Errors
    /// When the program doesn't lower.
    pub(crate) fn expecting(mut self, cal: &Calibration) -> Result<Plan, Error> {
        let expected = predict(&self.sheet("plan", HashMap::new())?, cal)?;
        for r in &mut self.readings {
            if let Kind::Length { expected_mm, .. } = &mut r.kind {
                *expected_mm = expected.get(&r.id).copied();
            }
        }
        Ok(self)
    }
}
