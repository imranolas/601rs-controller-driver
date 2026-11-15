//! The fit (CALIBRATION.md §7). The model, per head, in carriage millimetres:
//!
//! - `P`: where the carriage puts the head's line on the corner where boards register, so a cut
//!   commanded at carriage `c` runs along board `c − P`. One pair per head: only the sum of a
//!   board origin and a head offset can be seen in a cut, so it is fitted as one;
//! - `e`: where each side of the slit lands from the line, to the left and to the right of
//!   travel, positive towards that side; for the bevel, separately on the back (as cut) and on
//!   the face, which differ by the bevel's run;
//! - the drag at the start and at the end of a cut (calibration 1's overrun readings).
//!
//! An edge reading is `c − P + σe` (σ = ±1, the kept side along the axis), a width is two edges'
//! difference (`P` cancels), an overrun is `overcut ± drag − e`. The combs are read by eye: the
//! first tooth that reaches the reference gives an interval for the overcut a corner needs.

use std::collections::HashMap;

use machine::{MachineMm, Profile};
use program::{Cut, Head, Op, Program};
use serde_json::Value;

use crate::Error;
use crate::sent::{SentCut, cuts_from_lowering};

/// Which face of the board a slit edge is read on: the bevel's differ, by the bevel's run.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Face {
    Any,
    Back,
    Front,
}

/// One unknown of the fit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unknown {
    /// Head, axis: where the carriage puts the head's line on the registered corner.
    P(Head, usize),
    /// Head, kept side right?, face: where that side of the slit lands from the line.
    E(Head, bool, Face),
    /// Head, at the start?: the drag.
    Drag(Head, bool),
}

impl Unknown {
    pub fn head(&self) -> Head {
        match *self {
            Unknown::P(h, _) | Unknown::E(h, _, _) | Unknown::Drag(h, _) => h,
        }
    }
    fn order(&self) -> (u8, u8, u8, u8) {
        let h = u8::from(self.head() == Head::Bevel);
        match *self {
            Unknown::P(_, a) => (h, 0, a as u8, 0),
            Unknown::E(_, r, f) => (h, 1, u8::from(r), f as u8),
            Unknown::Drag(_, s) => (h, 2, u8::from(!s), 0),
        }
    }
    /// How the report names it: `Px`, `e left, back`, `drag start`.
    pub fn label(&self) -> String {
        match *self {
            Unknown::P(_, a) => format!("P{}", ["x", "y"][a]),
            Unknown::E(_, r, f) => {
                let side = if r { "right" } else { "left" };
                match f {
                    Face::Any => format!("e {side}"),
                    Face::Back => format!("e {side}, back"),
                    Face::Front => format!("e {side}, face"),
                }
            }
            Unknown::Drag(_, s) => format!("drag {}", if s { "start" } else { "end" }),
        }
    }
}

struct Row {
    name: String,
    terms: Vec<(Unknown, f64)>,
    rhs: f64,
}

/// A comb read by eye: the first tooth that reached (0 for none).
#[derive(Debug, Clone, PartialEq)]
pub struct CombReading {
    pub name: String,
    pub head: Head,
    pub at_start: bool,
    pub tooth: usize,
    /// Each tooth's overcut, least first, mm.
    pub values: Vec<f64>,
}

fn head_of(s: &str) -> Result<Head, Error> {
    match s {
        "bevel" => Ok(Head::Bevel),
        "straight" => Ok(Head::Straight),
        _ => Err(format!("unknown head {s:?}").into()),
    }
}

/// A cut as the fit sees it: head, start, end, unit direction (along X or Y).
struct Line {
    head: Head,
    a: [f64; 2],
    b: [f64; 2],
    d: [f64; 2],
}

impl Line {
    fn new(c: &SentCut, what: &str) -> Result<Line, Error> {
        let (a, b) = (c.points[0], c.points[c.points.len() - 1]);
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        let d = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
        if d[0].abs() > 1e-6 && d[1].abs() > 1e-6 {
            return Err(format!("{what}: calibration cuts run along X or Y").into());
        }
        Ok(Line {
            head: c.head,
            a,
            b,
            d,
        })
    }
    /// The kept side's direction along `axis`: ±1.
    fn sigma(&self, kept_right: bool, axis: usize) -> f64 {
        let dir = if kept_right {
            [self.d[1], -self.d[0]]
        } else {
            [-self.d[1], self.d[0]]
        };
        dir[axis].round()
    }
    fn e(&self, kept_right: bool, face: Face) -> Unknown {
        let face = if self.head == Head::Straight {
            Face::Any
        } else {
            face
        };
        Unknown::E(self.head, kept_right, face)
    }
}

fn face_of(v: Option<&str>) -> Face {
    match v {
        Some("face") => Face::Front,
        Some("back") => Face::Back,
        _ => Face::Any,
    }
}

fn text_of(v: &Value, k: &str) -> String {
    v.get(k)
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_owned()
}

/// One sheet's rows and comb readings.
fn sheet_rows(sheet: &Sheet) -> Result<(Vec<Row>, Vec<CombReading>), Error> {
    let (program, cuts, label) = (&sheet.program, &sheet.cuts, &sheet.label);
    let cal = program
        .meta
        .as_ref()
        .and_then(|m| m.get("calibration"))
        .ok_or_else(|| format!("{label}: not a calibration sheet (no meta.calibration)"))?;
    // The program's cuts in order, with their op indices, matched to the log's.
    let ops: Vec<(usize, &Cut)> = program
        .ops
        .iter()
        .enumerate()
        .filter_map(|(i, o)| match o {
            Op::Cut(c) => Some((i, c)),
            _ => None,
        })
        .collect();
    if ops.len() != cuts.len() {
        return Err(format!(
            "{label}: the log has {} cuts, the program {}: not this program's run, or it stopped",
            cuts.len(),
            ops.len()
        )
        .into());
    }
    for (k, ((_, c), l)) in ops.iter().zip(cuts).enumerate() {
        if c.head != l.head {
            return Err(format!("{label}: cut {} is on another head in the log", k + 1).into());
        }
    }
    // v2 names cuts by id; v1 (calibration 1) by op index.
    let line = |v: &Value, key: &str| -> Result<Line, Error> {
        let k = match v.get(key) {
            Some(Value::String(id)) => ops.iter().position(|(_, c)| c.id.as_deref() == Some(id)),
            Some(Value::Number(n)) => ops.iter().position(|(i, _)| Some(*i as u64) == n.as_u64()),
            _ => None,
        }
        .ok_or_else(|| format!("{label}: no cut {:?}", v.get(key)))?;
        Line::new(&cuts[k], &format!("cut {k}"))
    };
    let (list, v2) = match (cal.get("readings"), cal.get("measurements")) {
        (Some(Value::Array(a)), _) => (a, true),
        (_, Some(Value::Array(a))) => (a, false),
        _ => return Err(format!("{label}: no readings listed").into()),
    };
    let names: Vec<String> = list.iter().map(|m| text_of(m, "name")).collect();
    if let Some(unknown) = sheet.readings.keys().find(|k| !names.contains(k)) {
        return Err(format!("{label}: {unknown} is not a reading of this sheet").into());
    }
    // An edge: its line, and the kept side's sigma and unknown.
    let edge = |e: &Value, axis: usize| -> Result<(Line, f64, Unknown), Error> {
        let l = line(e, "cut")?;
        let right = text_of(e, "kept") == "right";
        let sigma = l.sigma(right, axis);
        let u = l.e(right, face_of(e.get("face").and_then(Value::as_str)));
        Ok((l, sigma, u))
    };
    let mut rows = Vec::new();
    let mut combs = Vec::new();
    for m in list {
        let name = text_of(m, "name");
        let Some(&value) = sheet.readings.get(&name) else {
            continue;
        };
        let axis = usize::from(text_of(m, "axis") == "y");
        match text_of(m, "kind").as_str() {
            // v1 calls a position reading an edge, with the edge's fields inline.
            "position" | "edge" => {
                let spec = if v2 {
                    m.get("edge").unwrap_or(&Value::Null)
                } else {
                    m
                };
                let (l, sigma, u) = edge(spec, axis)?;
                rows.push(Row {
                    name,
                    terms: vec![(Unknown::P(l.head, axis), -1.0), (u, sigma)],
                    rhs: value - l.a[axis],
                });
            }
            "width" => {
                let es = m
                    .get("edges")
                    .and_then(Value::as_array)
                    .filter(|a| a.len() == 2)
                    .ok_or_else(|| format!("{name}: a width needs two edges"))?;
                let mut one = edge(&es[0], axis)?;
                let mut two = edge(&es[1], axis)?;
                if two.0.a[axis] < one.0.a[axis] {
                    std::mem::swap(&mut one, &mut two);
                }
                let mut terms = vec![(two.2, two.1), (one.2, -one.1)];
                if one.0.head != two.0.head {
                    terms.push((Unknown::P(two.0.head, axis), -1.0));
                    terms.push((Unknown::P(one.0.head, axis), 1.0));
                }
                rows.push(Row {
                    name,
                    terms,
                    rhs: value - (two.0.a[axis] - one.0.a[axis]),
                });
            }
            "overrun" => {
                let (l, c) = (line(m, "cut")?, line(m, "crossing")?);
                let along = usize::from(l.d[1].abs() > 0.5);
                let at = c.a[along];
                let start = text_of(m, "end") == "start";
                let overcut = if start {
                    (at - l.a[along]) * l.d[along]
                } else {
                    (l.b[along] - at) * l.d[along]
                };
                let right = text_of(m, "kept") == "right";
                let face = face_of(m.get("face").and_then(Value::as_str));
                rows.push(Row {
                    name,
                    terms: vec![
                        (Unknown::Drag(l.head, start), if start { 1.0 } else { -1.0 }),
                        (c.e(right, face), -1.0),
                    ],
                    rhs: value - overcut,
                });
            }
            "comb" => {
                let values: Vec<f64> = m
                    .get("values")
                    .and_then(Value::as_array)
                    .map(|a| a.iter().filter_map(Value::as_f64).collect())
                    .unwrap_or_default();
                let tooth = value.round() as usize;
                if value < 0.0 || tooth > values.len() || (value - value.round()).abs() > 1e-9 {
                    return Err(format!(
                        "{name}: give the tooth number, 1 to {} (0: none reached)",
                        values.len()
                    )
                    .into());
                }
                combs.push(CombReading {
                    name,
                    head: head_of(&text_of(m, "head"))?,
                    at_start: text_of(m, "at") == "start",
                    tooth,
                    values,
                });
            }
            k => return Err(format!("{name}: unknown kind {k:?}").into()),
        }
    }
    Ok((rows, combs))
}

/// Solve the normal equations by Gauss–Jordan with partial pivoting; also returns the inverse
/// (for the uncertainties). `None` for a column the readings don't determine.
fn solve(ata: &[Vec<f64>], atb: &[f64]) -> (Vec<Option<f64>>, Vec<Vec<f64>>) {
    let n = atb.len();
    let mut m: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = ata[i].clone();
            row.extend((0..n).map(|j| f64::from(u8::from(i == j))));
            row.push(atb[i]);
            row
        })
        .collect();
    let scale = (0..n).map(|i| ata[i][i].abs()).fold(0.0, f64::max).max(1.0);
    let mut solved = vec![true; n];
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))
            .unwrap_or(col);
        if m[pivot][col].abs() < 1e-9 * scale {
            solved[col] = false;
            continue;
        }
        m.swap(col, pivot);
        let p = m[col][col];
        for v in &mut m[col] {
            *v /= p;
        }
        for row in 0..n {
            if row != col {
                let f = m[row][col];
                if f != 0.0 {
                    for k in 0..m[row].len() {
                        m[row][k] -= f * m[col][k];
                    }
                }
            }
        }
    }
    let x = (0..n).map(|i| solved[i].then(|| m[i][2 * n])).collect();
    let inv = (0..n).map(|i| m[i][n..2 * n].to_vec()).collect();
    (x, inv)
}

/// The fitted values: each unknown with its standard error (`None`: not determined).
#[derive(Debug, Clone, PartialEq)]
pub struct Fit {
    /// Each unknown, its value (`None`: the readings don't settle it) and its standard error.
    pub values: Vec<(Unknown, Option<f64>, f64)>,
    /// Each reading less the model, mm.
    pub leftovers: Vec<(String, f64)>,
    /// How far the readings scatter about the model, mm (NaN: no readings to spare).
    pub sigma: f64,
    /// How many readings were fitted.
    pub rows: usize,
}

impl Fit {
    /// A head's offset as fitted, and its standard errors, mm: the board origin held where
    /// `profile` has it, the offset is what puts the head's fitted line there (`origin − P`).
    pub fn head_offset(&self, head: Head, profile: &Profile) -> Option<([f64; 2], [f64; 2])> {
        let (px, sx) = self.get(Unknown::P(head, 0))?;
        let (py, sy) = self.get(Unknown::P(head, 1))?;
        let MachineMm([ox, oy]) = profile.board_origin_mm;
        Some(([ox - px, oy - py], [sx, sy]))
    }

    /// `profile` with each head's offset as fitted, to the micrometre, where the readings
    /// settle it.
    pub fn profile(&self, profile: &Profile) -> Profile {
        let mut fitted = profile.clone();
        for h in [Head::Straight, Head::Bevel] {
            if let Some((new, _)) = self.head_offset(h, profile) {
                let head = match h {
                    Head::Bevel => &mut fitted.bevel,
                    Head::Straight => &mut fitted.straight,
                };
                head.offset_mm = new.map(|v| (v * 1000.0).round() / 1000.0);
            }
        }
        fitted
    }

    /// An unknown's value and standard error, if the readings settle it.
    pub fn get(&self, u: Unknown) -> Option<(f64, f64)> {
        self.values
            .iter()
            .find(|(k, _, _)| *k == u)
            .and_then(|(_, v, se)| v.map(|v| (v, *se)))
    }
}

fn fit_rows(rows: &[Row]) -> Fit {
    let mut cols: Vec<Unknown> = rows
        .iter()
        .flat_map(|r| r.terms.iter().map(|t| t.0))
        .collect();
    cols.sort_by_key(Unknown::order);
    cols.dedup();
    let (m, n) = (rows.len(), cols.len());
    let a: Vec<Vec<f64>> = rows
        .iter()
        .map(|r| {
            let mut v = vec![0.0; n];
            for &(k, c) in &r.terms {
                v[cols.iter().position(|x| *x == k).expect("a column")] += c;
            }
            v
        })
        .collect();
    let y: Vec<f64> = rows.iter().map(|r| r.rhs).collect();
    let ata: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| (0..m).map(|k| a[k][i] * a[k][j]).sum())
                .collect()
        })
        .collect();
    let atb: Vec<f64> = (0..n)
        .map(|i| (0..m).map(|k| a[k][i] * y[k]).sum())
        .collect();
    let (x, inv) = solve(&ata, &atb);
    let p: Vec<f64> = x.iter().map(|v| v.unwrap_or(0.0)).collect();
    let leftovers: Vec<(String, f64)> = (0..m)
        .map(|k| {
            let model: f64 = (0..n).map(|j| a[k][j] * p[j]).sum();
            (rows[k].name.clone(), y[k] - model)
        })
        .collect();
    let dof = m.saturating_sub(x.iter().filter(|v| v.is_some()).count());
    let sigma = if dof > 0 {
        (leftovers.iter().map(|(_, r)| r * r).sum::<f64>() / dof as f64).sqrt()
    } else {
        f64::NAN
    };
    let values = cols
        .iter()
        .enumerate()
        .map(|(j, &u)| (u, x[j], sigma * inv[j][j].max(0.0).sqrt()))
        .collect();
    Fit {
        values,
        leftovers,
        sigma,
        rows: m,
    }
}

/// One sheet's data: the program, the cuts it sent, its readings.
#[derive(Debug, Clone, PartialEq)]
pub struct Sheet {
    pub label: String,
    pub program: Program,
    pub cuts: Vec<SentCut>,
    pub readings: HashMap<String, f64>,
}

impl Sheet {
    /// A sheet whose cuts are worked out by lowering `program` with `cut_with`, the profile it
    /// was cut with: no run log needed.
    ///
    /// # Errors
    /// When the program can't be lowered with that profile.
    pub fn lowered(
        label: impl Into<String>,
        program: Program,
        cut_with: &Profile,
        readings: HashMap<String, f64>,
    ) -> Result<Sheet, Error> {
        let cuts = cuts_from_lowering(&program, cut_with)?;
        Ok(Sheet {
            label: label.into(),
            program,
            cuts,
            readings,
        })
    }
}

/// Fit every sheet's readings together.
///
/// # Errors
/// When there's no sheet, the sheets are on boards of different thickness, or a sheet's cuts
/// and readings don't agree with its program.
pub fn fit_sheets(sheets: &[Sheet]) -> Result<(Fit, Vec<CombReading>), Error> {
    let Some(first) = sheets.first() else {
        return Err("no sheets to fit".into());
    };
    let thick = first.program.board.thickness_mm;
    if sheets
        .iter()
        .any(|s| (s.program.board.thickness_mm - thick).abs() > 1e-9)
    {
        return Err(
            "the sheets are on boards of different thickness, where the bevel's face \
                    edge differs: fit them separately"
                .into(),
        );
    }
    let mut rows = Vec::new();
    let mut combs = Vec::new();
    for s in sheets {
        let (r, c) = sheet_rows(s)?;
        rows.extend(r);
        combs.extend(c);
    }
    Ok((fit_rows(&rows), combs))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Readings made up from known heads, worked out edge by edge from the sheet's own
    /// geometry, come back out of the fit.
    #[test]
    fn the_fit_recovers_known_heads() {
        let profile = Profile::gunnar_601();
        let p = crate::sheet::program(&profile);
        let sheet = Sheet::lowered("test", p, &profile.raw(), HashMap::new()).expect("lowers");
        let start = |id: &str| -> [f64; 2] {
            let k = sheet
                .program
                .ops
                .iter()
                .filter_map(|o| match o {
                    Op::Cut(c) => Some(c),
                    _ => None,
                })
                .position(|c| c.id.as_deref() == Some(id))
                .expect("cut");
            sheet.cuts[k].points[0]
        };
        let x = |id: &str| start(id)[0];
        let y = |id: &str| start(id)[1];
        // Straight: P (10, 20), slit sides 0.05 left and 0.15 right. Bevel: P (90, 18), the
        // window's back edge 0.4 outside, its face edge 2.1 outside.
        let (sp, sl, sr) = ([10.0, 20.0], 0.05, 0.15);
        let (bp, back, face) = ([90.0, 18.0], 0.4, 2.1);
        let readings: HashMap<String, f64> = [
            ("SL.width", (x("S.B") - x("S.A")) - 2.0 * sl),
            ("SR.width", (x("S.C") - x("S.B")) - 2.0 * sr),
            ("SL.length", (y("S.K") - y("S.F")) - 2.0 * sl),
            ("S.left", x("S.A") - sp[0] - sr),
            ("S.front", y("S.F") - sp[1] - sr),
            ("W.left", x("W.left") - bp[0] - back),
            ("W.width", (x("W.right") - x("W.left")) + 2.0 * back),
            ("W.front", y("W.front") - bp[1] - back),
            ("W.height", (y("W.back") - y("W.front")) + 2.0 * back),
            ("D.width", (x("W.right") - x("W.left")) + 2.0 * face),
            ("D.height", (y("W.back") - y("W.front")) + 2.0 * face),
            ("S.end", 5.0),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_owned(), v))
        .collect();
        let sheet = Sheet { readings, ..sheet };
        let (fit, combs) = fit_sheets(&[sheet]).expect("fits");
        let near = |u: Unknown, want: f64| {
            let (v, _) = fit.get(u).expect("determined");
            assert!((v - want).abs() < 1e-6, "{u:?}: {v} against {want}");
        };
        near(Unknown::P(Head::Straight, 0), sp[0]);
        near(Unknown::P(Head::Straight, 1), sp[1]);
        near(Unknown::E(Head::Straight, false, Face::Any), sl);
        near(Unknown::E(Head::Straight, true, Face::Any), sr);
        near(Unknown::P(Head::Bevel, 0), bp[0]);
        near(Unknown::P(Head::Bevel, 1), bp[1]);
        near(Unknown::E(Head::Bevel, false, Face::Back), back);
        near(Unknown::E(Head::Bevel, true, Face::Front), -face);
        assert!(fit.sigma < 1e-6);
        assert_eq!(combs.len(), 1);
        assert_eq!((combs[0].tooth, combs[0].at_start), (5, false));
        // The board origin held, each head's offset is what puts its line at P.
        let MachineMm([ox, oy]) = profile.board_origin_mm;
        let (off, _) = fit.head_offset(Head::Bevel, &profile).expect("settled");
        assert!((off[0] - (ox - bp[0])).abs() < 1e-6 && (off[1] - (oy - bp[1])).abs() < 1e-6);
        let fitted = fit.profile(&profile);
        assert_eq!(
            fitted.straight.offset_mm,
            [ox - sp[0], oy - sp[1]].map(|v| (v * 1000.0).round() / 1000.0)
        );
    }

    #[test]
    fn no_sheets_is_an_error_not_a_panic() {
        assert!(fit_sheets(&[]).is_err());
    }
}
