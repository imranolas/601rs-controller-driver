//! The record's calibration sheets (`testdata/calibration`, and [`crate::sheet`]): calibrations
//! 1 and 2, cut raw in carriage coordinates before the routines, whose programs list their
//! readings in `meta.calibration`. Read here into [`Reading`]s, once, so nothing else knows the
//! old forms: version 1 calls them `measurements`, names cuts by op index and gives an edge's
//! fields inline; version 2 names cuts by id.

use program::{Head, Op, Program};
use serde::Deserialize;
use serde_json::Value;

use crate::Error;
use crate::key::{Axis, Surface};
use crate::plan::Step;
use crate::reading::{Comb, Edge, End, Kind, Reading, Side, Span, Tooth};

/// Further than this from the value a sheet gives, a reading is asked about, mm, unless the
/// sheet says otherwise.
const UNUSUAL_MM: f64 = 1.5;

/// A cut as a sheet names it: by id (version 2), or by op index (version 1).
#[derive(Deserialize)]
#[serde(untagged)]
enum CutRef {
    Id(String),
    Op(usize),
}

/// A side of the board as a sheet names it: a straight cut's edges are on `any`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum OldFace {
    #[default]
    Any,
    Back,
    Face,
}

impl OldFace {
    /// A straight cut's edge is the same on either side: read as it lay, back up.
    fn surface(self) -> Surface {
        match self {
            OldFace::Face => Surface::Face,
            OldFace::Any | OldFace::Back => Surface::Back,
        }
    }
}

#[derive(Deserialize)]
struct OldEdge {
    cut: CutRef,
    kept: Side,
    #[serde(default)]
    face: OldFace,
}

/// A reading as a sheet lists it, by its `kind`.
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Listed {
    Position {
        axis: Axis,
        edge: OldEdge,
    },
    /// Version 1's position, its edge's fields inline.
    Edge {
        axis: Axis,
        #[serde(flatten)]
        edge: OldEdge,
    },
    Width {
        axis: Axis,
        edges: [OldEdge; 2],
    },
    Overrun {
        cut: CutRef,
        end: End,
        crossing: CutRef,
        kept: Side,
        #[serde(default)]
        face: OldFace,
    },
    Comb {
        head: Head,
        at: End,
        values: Vec<f64>,
    },
}

/// What a sheet says of any reading, for the person taking it.
#[derive(Deserialize)]
struct Said {
    name: String,
    #[serde(default)]
    what: String,
    #[serde(default)]
    step: Option<String>,
    #[serde(default)]
    about: Option<f64>,
    #[serde(default)]
    unusual: Option<f64>,
}

/// The readings a record sheet lists, as [`Reading`]s, and the steps they are taken in. The
/// record's sheets are never drawn: their dimensions are left at the origin. A comb's teeth are
/// its cuts by the sheet's names (`SE.1` to `SE.7`, its reference `SE.ref`).
///
/// # Errors
/// When the program isn't a calibration sheet, or a reading isn't one.
pub fn readings(program: &Program) -> Result<(Vec<Step>, Vec<Reading>), Error> {
    let id = &program.id;
    let list = program
        .meta
        .as_ref()
        .and_then(|m| m.get("calibration"))
        .and_then(|c| c.get("readings").or_else(|| c.get("measurements")))
        .and_then(Value::as_array)
        .ok_or_else(|| {
            Error::Sheet(format!(
                "{id}: not a calibration sheet (no readings listed)"
            ))
        })?;
    let cut = |r: &CutRef| match r {
        CutRef::Id(s) => Ok(s.clone()),
        CutRef::Op(i) => program
            .ops
            .get(*i)
            .and_then(Op::id)
            .map(str::to_owned)
            .ok_or_else(|| Error::Sheet(format!("{id}: op {i} isn't a named cut"))),
    };
    let edge = |e: &OldEdge| -> Result<Edge, Error> {
        Ok(Edge {
            cut: cut(&e.cut)?,
            side: e.kept,
            surface: e.face.surface(),
        })
    };
    let mut steps: Vec<Step> = Vec::new();
    let mut readings = Vec::new();
    for v in list {
        let bad = |e: serde_json::Error| Error::Sheet(format!("{id}: {e}, in {v}"));
        let said = Said::deserialize(v).map_err(bad)?;
        let length = |span: Span| {
            let face_up = match &span {
                Span::FromStock { edge, .. } => edge.surface == Surface::Face,
                Span::Between { edges, .. } => edges.iter().any(|e| e.surface == Surface::Face),
                Span::Overrun { crossing, .. } => crossing.surface == Surface::Face,
            };
            let kind = Kind::Length {
                span,
                expected_mm: said.about,
                unusual_mm: said.unusual.unwrap_or(UNUSUAL_MM),
            };
            (
                kind,
                if face_up {
                    Surface::Face
                } else {
                    Surface::Back
                },
            )
        };
        let (kind, surface) = match Listed::deserialize(v).map_err(bad)? {
            Listed::Position { axis, edge: e } | Listed::Edge { axis, edge: e } => {
                length(Span::FromStock {
                    axis,
                    edge: edge(&e)?,
                })
            }
            Listed::Width {
                axis,
                edges: [a, b],
            } => length(Span::Between {
                axis,
                edges: [edge(&a)?, edge(&b)?],
            }),
            Listed::Overrun {
                cut: c,
                end,
                crossing,
                kept,
                face,
            } => length(Span::Overrun {
                cut: cut(&c)?,
                end,
                crossing: Edge {
                    cut: cut(&crossing)?,
                    side: kept,
                    surface: face.surface(),
                },
            }),
            Listed::Comb { head, at, values } => {
                let tag = format!(
                    "{}{}",
                    if head == Head::Bevel { "B" } else { "S" },
                    if at == End::Start { "S" } else { "E" }
                );
                let comb = Comb {
                    head,
                    end: at,
                    reference: format!("{tag}.ref"),
                    teeth: values
                        .iter()
                        .enumerate()
                        .map(|(i, &run_on_mm)| Tooth {
                            cut: format!("{tag}.{}", i + 1),
                            run_on_mm,
                        })
                        .collect(),
                };
                (Kind::Tooth(comb), Surface::Face)
            }
        };
        let instruction = said.step.unwrap_or_default();
        match steps.last_mut() {
            Some(s) if s.instruction == instruction => s.readings.push(said.name.clone()),
            _ => steps.push(Step {
                instruction,
                readings: vec![said.name.clone()],
            }),
        }
        readings.push(Reading {
            id: said.name,
            prompt: said.what,
            surface,
            kind,
            dimension: [[0.0; 2]; 2],
        });
    }
    Ok((steps, readings))
}

#[cfg(test)]
mod tests {
    use super::*;

    const RECORD: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/calibration");

    fn record(name: &str) -> Program {
        let path = std::path::Path::new(RECORD).join(name);
        let text =
            std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        Program::from_json(&text).expect("a program")
    }

    /// Calibration 1's twenty readings: its edges by op index, its overruns across a crossing.
    #[test]
    fn version_1_names_its_cuts_by_op() {
        let (_, rs) = readings(&record("calibration_1.json")).expect("readings");
        assert_eq!(rs.len(), 20);
        let r = rs.iter().find(|r| r.id == "B1.left").expect("listed");
        assert_eq!(
            r.kind,
            Kind::Length {
                span: Span::FromStock {
                    axis: Axis::X,
                    edge: Edge {
                        cut: "B1.left".into(),
                        side: Side::Left,
                        surface: Surface::Back
                    }
                },
                expected_mm: None,
                unusual_mm: UNUSUAL_MM,
            }
        );
        assert!(rs.iter().any(|r| matches!(
            &r.kind,
            Kind::Length {
                span: Span::Overrun { .. },
                ..
            }
        )));
    }

    /// Calibration sheet 2, as recorded and as built: the same readings, its combs' teeth the
    /// program's cuts.
    #[test]
    fn version_2_reads_as_the_sheet_is_built() {
        let (_, recorded) = readings(&record("calibration_2.json")).expect("readings");
        let built = crate::sheet::program(&machine::Profile::gunnar_601());
        let (_, rs) = readings(&built).expect("readings");
        assert_eq!(rs, recorded);
        let ids: Vec<&str> = built.ops.iter().filter_map(Op::id).collect();
        for r in &rs {
            assert!(r.needs().iter().all(|c| ids.contains(c)), "{}", r.id);
        }
        let combs = rs.iter().filter(|r| r.comb().is_some()).count();
        assert_eq!((rs.len(), combs), (15, 4));
    }
}
