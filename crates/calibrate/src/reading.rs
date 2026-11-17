//! A reading (CALIBRATION.md §6): one value the operator takes off a cut sheet, and what it
//! measures. One description serves everything: the plan shows it (its prompt, the side up,
//! where it is taken, what it should read), the session checks it as typed, the fit makes it a
//! row of the model, and stage 2 reads its combs.

use program::Head;
use serde::{Deserialize, Serialize};

use crate::Error;
use crate::key::{Axis, Surface};

/// A side of a cut, as it travels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    Left,
    Right,
}

/// An end of a cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum End {
    Start,
    End,
}

impl End {
    pub const BOTH: [End; 2] = [End::Start, End::End];

    pub fn word(self) -> &'static str {
        match self {
            End::Start => "start",
            End::End => "end",
        }
    }

    /// Its place in a pair: 0 for the start, 1 for the end (`compiler::HeadCut::run_on`).
    pub fn index(self) -> usize {
        match self {
            End::Start => 0,
            End::End => 1,
        }
    }
}

/// An edge: one side of a cut, read on one side of the board.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Edge {
    /// The cut, by its id in the program.
    pub cut: String,
    /// The side of the cut the piece read lies on.
    pub side: Side,
    /// The side of the board up as it is read: a bevel cut's edges differ, a straight cut's
    /// don't.
    pub surface: Surface,
}

/// What a length by calipers spans.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Span {
    /// From the board's edge against the stops to `edge`, along `axis`.
    FromStock { axis: Axis, edge: Edge },
    /// From one edge to the other, along `axis`.
    Between { axis: Axis, edges: [Edge; 2] },
}

impl Span {
    /// The program's cuts it reads.
    pub fn cuts(&self) -> Vec<&str> {
        match self {
            Span::FromStock { edge, .. } => vec![edge.cut.as_str()],
            Span::Between { edges, .. } => edges.iter().map(|e| e.cut.as_str()).collect(),
        }
    }
}

/// A comb (stage 2): a reference cut and teeth whose run-ons are graded, the shortest first.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Comb {
    pub head: Head,
    /// Which end of a cut it shows: at an end the teeth run on past the reference, at a start
    /// they begin past it.
    pub end: End,
    /// The reference cut, by id.
    pub reference: String,
    /// Tooth 1 first.
    pub teeth: Vec<Tooth>,
}

/// One tooth of a comb: its cut, and how far it runs on past the reference (at a start, how
/// far before it it begins), mm.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Tooth {
    pub cut: String,
    pub run_on_mm: f64,
}

/// What a reading is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// A length, by calipers, mm.
    Length {
        span: Span,
        /// What it should read by the calibration the plan was made with; none when not known.
        expected_mm: Option<f64>,
        /// Further than this from what's expected, ask before keeping it.
        unusual_mm: f64,
    },
    /// A comb's first tooth to come free at its reference, by flexing the board: 1 to its
    /// teeth from the shortest, or 0 if none does.
    Tooth(Comb),
}

/// One reading.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Reading {
    /// "W.width".
    pub id: String,
    /// "The window, left to right (inside jaws)".
    pub prompt: String,
    /// The side of the board up as it is read.
    pub surface: Surface,
    pub kind: Kind,
    /// Where it is taken, in the board frame, for a drawing: from one edge to the other, mm;
    /// for a comb, its reference.
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

impl Reading {
    /// What a length should read, if known; none for a comb.
    pub fn expected_mm(&self) -> Option<f64> {
        match self.kind {
            Kind::Length { expected_mm, .. } => expected_mm,
            Kind::Tooth(_) => None,
        }
    }

    /// The comb it reads, if it reads one.
    pub fn comb(&self) -> Option<&Comb> {
        match &self.kind {
            Kind::Tooth(c) => Some(c),
            Kind::Length { .. } => None,
        }
    }

    /// The program's cuts it reads: if one of them didn't finish, it can't be taken.
    pub fn needs(&self) -> Vec<&str> {
        match &self.kind {
            Kind::Length { span, .. } => span.cuts(),
            Kind::Tooth(c) => c
                .teeth
                .iter()
                .map(|t| t.cut.as_str())
                .chain([c.reference.as_str()])
                .collect(),
        }
    }

    /// Whether `value` can be this reading, and whether it is unusual.
    ///
    /// # Errors
    /// When it isn't a length, or a tooth of the comb.
    pub fn check(&self, value: f64) -> Result<Check, Error> {
        let id = &self.id;
        match &self.kind {
            Kind::Length {
                expected_mm,
                unusual_mm,
                ..
            } => {
                if !(value.is_finite() && value > 0.0) {
                    return Err(Error::Reading(format!("{id}: {value} isn't a length")));
                }
                Ok(match expected_mm.map(|e| (value - e).abs()) {
                    Some(off_mm) if off_mm > *unusual_mm => Check::Unusual { off_mm },
                    _ => Check::Fine,
                })
            }
            Kind::Tooth(c) => {
                let teeth = c.teeth.len();
                if value.fract() == 0.0 && (0.0..=teeth as f64).contains(&value) {
                    Ok(Check::Fine)
                } else {
                    Err(Error::Reading(format!(
                        "{id}: give the tooth, 1 to {teeth}, or 0 if none came free"
                    )))
                }
            }
        }
    }
}

/// A comb as read: the first tooth to come free (0: none did).
#[derive(Debug, Clone, PartialEq)]
pub struct CombReading {
    pub id: String,
    pub head: Head,
    pub end: End,
    pub tooth: usize,
    /// Each tooth's run-on, tooth 1 first, mm.
    pub run_ons: Vec<f64>,
}

/// The combs among the readings `taken`, each as read.
///
/// # Errors
/// When a comb's value isn't one of its teeth.
pub fn combs<'a>(
    taken: impl IntoIterator<Item = (&'a Reading, f64)>,
) -> Result<Vec<CombReading>, Error> {
    taken
        .into_iter()
        .filter_map(|(r, v)| r.comb().map(|c| (r, c, v)))
        .map(|(r, c, v)| {
            r.check(v)?;
            Ok(CombReading {
                id: r.id.clone(),
                head: c.head,
                end: c.end,
                tooth: v as usize,
                run_ons: c.teeth.iter().map(|t| t.run_on_mm).collect(),
            })
        })
        .collect()
}
