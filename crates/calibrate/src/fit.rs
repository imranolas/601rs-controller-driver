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
use nalgebra::{DMatrix, DVector};
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
    /// The reading's uncertainty relative to a caliper reading's: 1 for calipers.
    sd: f64,
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
                    sd: 1.0,
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
                    sd: 1.0,
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
                    sd: 1.0,
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

/// The least-squares solution of a set of rows, by singular value decomposition: each reading
/// divided by its uncertainty (weighted least squares), so a reading counts by how well it is
/// known. Unlike the normal equations, it says which unknowns the readings fix on their own and
/// which only in combination ([`Fit::tied`]).
struct Solved {
    cols: Vec<Unknown>,
    /// Each unknown's value, if the readings fix it on its own.
    x: Vec<Option<f64>>,
    /// Each unknown's variance, in units of a caliper reading's variance.
    var: Vec<f64>,
    /// Each reading less the model, in units of the reading's own uncertainty.
    resid: Vec<f64>,
    /// Each reading's leverage: how much of the model it alone decides, 0 to 1.
    leverage: Vec<f64>,
    rank: usize,
    tied: Vec<Vec<Unknown>>,
    model: Model,
}

/// What the readings reach, for any combination of the unknowns: the least-squares solution
/// (the shortest, where unknowns are tied), its covariance per unit of a caliper reading's
/// variance, and the projector onto the combinations the readings fix.
#[derive(Debug, Clone, PartialEq)]
struct Model {
    cols: Vec<Unknown>,
    x: DVector<f64>,
    cov: DMatrix<f64>,
    reach: DMatrix<f64>,
}

impl Model {
    /// The combination `Σ c·u` as a vector over the columns; none if it names an unknown the
    /// readings never see.
    fn vector(&self, c: &[(Unknown, f64)]) -> Option<DVector<f64>> {
        let mut v = DVector::<f64>::zeros(self.cols.len());
        for &(u, k) in c {
            match self.cols.iter().position(|x| *x == u) {
                Some(j) => v[j] += k,
                None if k != 0.0 => return None,
                None => {}
            }
        }
        // Fixed when it lies wholly in what the readings reach.
        let off = (&self.reach * &v - &v).norm();
        (off <= 1e-9 * v.norm().max(1.0)).then_some(v)
    }

    /// The combination's value and variance per unit of a caliper reading's variance.
    fn combination(&self, c: &[(Unknown, f64)]) -> Option<(f64, f64)> {
        let v = self.vector(c)?;
        Some((v.dot(&self.x), (v.transpose() * &self.cov * &v)[(0, 0)]))
    }
}

/// Below this, relative to the largest, a singular value is none: the readings don't reach that
/// combination of unknowns.
const RANK_TOL: f64 = 1e-9;

fn solve(rows: &[Row]) -> Solved {
    let mut cols: Vec<Unknown> = rows
        .iter()
        .flat_map(|r| r.terms.iter().map(|t| t.0))
        .collect();
    cols.sort_by_key(Unknown::order);
    cols.dedup();
    let (m, n) = (rows.len(), cols.len());
    // At least as many rows as unknowns (the extra ones zero), so the SVD gives the whole of V,
    // the combinations the readings don't reach included.
    let k = m.max(n);
    let mut a = DMatrix::<f64>::zeros(k, n);
    let mut b = DVector::<f64>::zeros(k);
    for (i, r) in rows.iter().enumerate() {
        for &(u, c) in &r.terms {
            let j = cols.iter().position(|x| *x == u).unwrap_or_default();
            a[(i, j)] += c / r.sd;
        }
        b[i] = r.rhs / r.sd;
    }
    let svd = a.clone().svd(true, true);
    let s = &svd.singular_values;
    let (Some(u), Some(v_t)) = (svd.u.as_ref(), svd.v_t.as_ref()) else {
        unreachable!("U and Vᵀ were asked for")
    };
    let tol = s.max() * RANK_TOL;
    let kept: Vec<usize> = (0..s.len()).filter(|&i| s[i] > tol).collect();
    let mut x = DVector::<f64>::zeros(n);
    let mut cov = DMatrix::<f64>::zeros(n, n);
    let mut reach = DMatrix::<f64>::zeros(n, n);
    for &i in &kept {
        let v = v_t.row(i).transpose();
        x += &v * (u.column(i).dot(&b) / s[i]);
        let outer = &v * v.transpose();
        cov += &outer / s[i].powi(2);
        reach += outer;
    }
    // An unknown is fixed on its own when its axis lies wholly in the space the readings reach.
    let fixed = |j: usize| reach[(j, j)] > 1.0 - 1e-9;
    let var: Vec<f64> = (0..n).map(|j| cov[(j, j)]).collect();
    let model = &a * &x;
    let resid = (0..m).map(|i| b[i] - model[i]).collect();
    let leverage = (0..m)
        .map(|r| kept.iter().map(|&i| u[(r, i)].powi(2)).sum())
        .collect();
    // The combinations the readings don't reach: unknowns that share one are tied.
    let mut tied: Vec<Vec<Unknown>> = Vec::new();
    for i in (0..n).filter(|i| !kept.contains(i)) {
        let mut group: Vec<Unknown> = (0..n)
            .filter(|&j| v_t[(i, j)].abs() > 1e-6)
            .map(|j| cols[j])
            .collect();
        if let Some(t) = tied
            .iter_mut()
            .find(|t| t.iter().any(|u| group.contains(u)))
        {
            t.append(&mut group);
            t.sort_by_key(Unknown::order);
            t.dedup();
        } else if !group.is_empty() {
            tied.push(group);
        }
    }
    Solved {
        x: (0..n).map(|j| fixed(j).then(|| x[j])).collect(),
        var,
        resid,
        leverage,
        rank: kept.len(),
        tied,
        model: Model {
            cols: cols.clone(),
            x,
            cov,
            reach,
        },
        cols,
    }
}

/// One reading less the model.
#[derive(Debug, Clone, PartialEq)]
pub struct Leftover {
    pub name: String,
    /// mm.
    pub mm: f64,
    /// In units of what this reading should scatter, judged by the other readings and allowing
    /// for how much it decides on its own (externally studentised): beyond about 3.5, more
    /// likely a misread than scatter. None when nothing else checks it: it alone decides what
    /// it reads, or no other reading is spare.
    pub t: Option<f64>,
}

/// The fitted values: each unknown with its standard error (`None`: not determined).
#[derive(Debug, Clone, PartialEq)]
pub struct Fit {
    /// Each unknown, its value (`None`: the readings don't fix it on its own) and its standard
    /// error (NaN when not fixed).
    pub values: Vec<(Unknown, Option<f64>, f64)>,
    /// Each reading less the model.
    pub leftovers: Vec<Leftover>,
    /// How far a caliper reading scatters about the model, mm (NaN: no readings to spare).
    pub sigma: f64,
    /// How many readings were fitted.
    pub rows: usize,
    /// Unknowns the readings fix only together (a sum, a difference), each group once.
    pub tied: Vec<Vec<Unknown>>,
    model: Model,
}

impl Fit {
    /// A combination of unknowns, `Σ c·u`, and its standard error, if the readings fix it:
    /// fixed even when its parts are tied (the bevel head's line less the straight head's, say,
    /// when only the borders of a mount were read).
    pub fn combination(&self, c: &[(Unknown, f64)]) -> Option<(f64, f64)> {
        let (v, var) = self.model.combination(c)?;
        Some((v, self.sigma * var.sqrt()))
    }

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

    /// The readings worth taking again: studentised beyond `t`, worst first.
    pub fn suspects(&self, t: f64) -> Vec<&Leftover> {
        let mut out: Vec<&Leftover> = self
            .leftovers
            .iter()
            .filter(|l| l.t.is_some_and(|v| v.abs() > t))
            .collect();
        out.sort_by(|a, b| {
            let key = |l: &Leftover| l.t.map_or(0.0, f64::abs);
            key(b).total_cmp(&key(a))
        });
        out
    }
}

fn fit_rows(rows: &[Row]) -> Fit {
    let s = solve(rows);
    let m = rows.len();
    let dof = m.saturating_sub(s.rank);
    let sigma = if dof > 0 {
        (s.resid.iter().map(|r| r * r).sum::<f64>() / dof as f64).sqrt()
    } else {
        f64::NAN
    };
    // Each reading judged by the scatter of the others: the scatter without it is the whole
    // sum of squares less its share. Judged by a scatter it is part of, a misread would hide
    // itself (that measure can't pass √dof, under 3 on most sheets).
    let ss: f64 = s.resid.iter().map(|r| r * r).sum();
    let leftovers = rows
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let (e, h) = (s.resid[i], s.leverage[i]);
            let t = (dof > 1 && h < 1.0 - 1e-9).then(|| {
                let rest = (ss - e * e / (1.0 - h)).max(0.0) / (dof - 1) as f64;
                e / (rest * (1.0 - h)).sqrt()
            });
            Leftover {
                name: r.name.clone(),
                mm: e * r.sd,
                t: t.map(|t| if t.is_nan() { 0.0 } else { t }),
            }
        })
        .collect();
    let values = s
        .cols
        .iter()
        .enumerate()
        .map(|(j, &u)| {
            let se = if s.x[j].is_some() {
                sigma * s.var[j].sqrt()
            } else {
                f64::NAN
            };
            (u, s.x[j], se)
        })
        .collect();
    Fit {
        values,
        leftovers,
        sigma,
        rows: m,
        tied: s.tied,
        model: s.model,
    }
}

/// How far a caliper reading scatters, mm: calibration 1's readings about the model (0.112
/// mm, 20 readings, 10 Nov). It weighs a prior against the readings, and turns a plan's
/// precision into millimetres before anything is read.
pub const CALIPER_SD_MM: f64 = 0.11;

/// What is already known of an unknown, from the calibration: its value and standard error.
/// In a fit it is one more reading, of that unknown alone, weighed by its standard error
/// against [`CALIPER_SD_MM`]. A routine can then read only what it means to check and take
/// the rest as known.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prior {
    pub unknown: Unknown,
    pub value: f64,
    pub se_mm: f64,
}

impl Prior {
    fn row(&self) -> Row {
        Row {
            name: format!("known {}", self.unknown.label()),
            terms: vec![(self.unknown, 1.0)],
            rhs: self.value,
            sd: (self.se_mm / CALIPER_SD_MM).max(1e-6),
        }
    }
}

/// How precisely a set of readings would fix each unknown, before any is taken.
#[derive(Debug, Clone, PartialEq)]
pub struct Precision {
    /// Each unknown's standard error, mm, for caliper readings scattering [`CALIPER_SD_MM`]
    /// (`None`: these readings don't fix it on its own).
    pub values: Vec<(Unknown, Option<f64>)>,
    pub tied: Vec<Vec<Unknown>>,
    /// The readings, priors not counted.
    pub rows: usize,
    /// The readings that are an unknown's only witness: nothing would catch a misread.
    pub unchecked: Vec<String>,
    model: Model,
}

impl Precision {
    pub fn get(&self, u: Unknown) -> Option<f64> {
        self.values
            .iter()
            .find(|(k, _)| *k == u)
            .and_then(|(_, se)| *se)
    }

    /// A combination's standard error, mm, if the readings would fix it.
    pub fn combination(&self, c: &[(Unknown, f64)]) -> Option<f64> {
        self.model
            .combination(c)
            .map(|(_, var)| CALIPER_SD_MM * var.sqrt())
    }
}

/// How precisely `sheet`'s readings, with `priors`, would fix each unknown, from which readings
/// are taken (their values are ignored): what a plan can promise before it is cut, and what a
/// reading left out would cost.
///
/// # Errors
/// When the sheet's cuts and readings don't agree with its program.
pub fn precision(sheet: &Sheet, priors: &[Prior]) -> Result<Precision, Error> {
    let zeroed = Sheet {
        readings: sheet.readings.keys().map(|k| (k.clone(), 0.0)).collect(),
        ..sheet.clone()
    };
    let (mut rows, _) = sheet_rows(&zeroed)?;
    let read = rows.len();
    rows.extend(priors.iter().map(Prior::row));
    let s = solve(&rows);
    Ok(Precision {
        values: s
            .cols
            .iter()
            .enumerate()
            .map(|(j, &u)| (u, s.x[j].map(|_| CALIPER_SD_MM * s.var[j].sqrt())))
            .collect(),
        tied: s.tied,
        rows: read,
        unchecked: rows[..read]
            .iter()
            .zip(&s.leverage)
            .filter(|(_, h)| **h > 1.0 - 1e-9)
            .map(|(r, _)| r.name.clone())
            .collect(),
        model: s.model,
    })
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
    fit_sheets_with(sheets, &[])
}

/// Fit every sheet's readings together, with what is already known.
///
/// # Errors
/// As [`fit_sheets`].
pub fn fit_sheets_with(
    sheets: &[Sheet],
    priors: &[Prior],
) -> Result<(Fit, Vec<CombReading>), Error> {
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
    rows.extend(priors.iter().map(Prior::row));
    Ok((fit_rows(&rows), combs))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Calibration;

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

    fn row(name: &str, terms: &[(Unknown, f64)], rhs: f64, sd: f64) -> Row {
        Row {
            name: name.into(),
            terms: terms.to_vec(),
            rhs,
            sd,
        }
    }

    /// Readings that only ever see two unknowns as a sum fix neither: the normal equations
    /// reported the first as fixed (and the sum as its value).
    #[test]
    fn a_sum_alone_fixes_neither_of_its_parts() {
        let (p, e) = (
            Unknown::P(Head::Straight, 0),
            Unknown::E(Head::Straight, true, Face::Any),
        );
        let q = Unknown::P(Head::Bevel, 0);
        let fit = fit_rows(&[
            row("a", &[(p, 1.0), (e, 1.0)], 1.0, 1.0),
            row("b", &[(p, 1.0), (e, 1.0)], 1.2, 1.0),
            row("c", &[(q, 1.0)], 5.0, 1.0),
            row("d", &[(q, 1.0)], 5.2, 1.0),
        ]);
        assert_eq!(fit.get(p), None);
        assert_eq!(fit.get(e), None);
        assert_eq!(fit.tied, vec![vec![p, e]]);
        let (v, _) = fit.get(q).expect("fixed");
        assert!((v - 5.1).abs() < 1e-12);
        // The sum itself is fixed; a difference isn't.
        let (sum, _) = fit.combination(&[(p, 1.0), (e, 1.0)]).expect("fixed");
        assert!((sum - 1.1).abs() < 1e-12);
        assert_eq!(fit.combination(&[(p, 1.0), (e, -1.0)]), None);
    }

    /// A reading ten times less certain counts a hundred times less.
    #[test]
    fn a_reading_counts_by_how_well_it_is_known() {
        let u = Unknown::P(Head::Bevel, 1);
        let fit = fit_rows(&[
            row("calipers", &[(u, 1.0)], 1.0, 1.0),
            row("rule", &[(u, 1.0)], 2.0, 10.0),
        ]);
        let (v, _) = fit.get(u).expect("fixed");
        assert!((v - (1.0 + 2.0 / 100.0) / (1.0 + 1.0 / 100.0)).abs() < 1e-12);
    }

    /// Readings with a caliper's scatter, one misread by half a millimetre. On calibration sheet
    /// 2 a width has one twin (the window across and along both read the bevel's back edge), so
    /// a misread width is caught but not told from its twin: both stand out, and nothing else.
    /// The readings from the board's edges are each a head's only witness: nothing checks them.
    #[test]
    fn a_misread_stands_out_where_another_reading_checks_it() {
        let profile = Profile::gunnar_601();
        let p = crate::sheet::program(&profile);
        let names = [
            "SL.width",
            "SR.width",
            "SL.length",
            "S.left",
            "S.front",
            "W.left",
            "W.width",
            "W.front",
            "W.height",
            "D.width",
            "D.height",
        ];
        let sheet = Sheet::lowered("test", p, &profile.raw(), HashMap::new()).expect("lowers");
        let clean = expected(&sheet, &names, &Calibration::new());
        // A fixed scatter of a few hundredths, as calipers give.
        let noise = [
            0.03, -0.02, 0.01, -0.04, 0.02, 0.0, -0.01, 0.03, -0.03, 0.02, -0.01,
        ];
        let mut readings: HashMap<String, f64> = names
            .iter()
            .zip(noise)
            .map(|(n, d)| ((*n).to_owned(), clean[*n] + d))
            .collect();
        *readings.get_mut("W.width").expect("listed") += 0.5;
        let (fit, _) = fit_sheets(&[Sheet { readings, ..sheet }]).expect("fits");
        let mut suspects: Vec<&str> = fit.suspects(3.5).iter().map(|l| l.name.as_str()).collect();
        suspects.sort_unstable();
        assert_eq!(suspects, ["W.height", "W.width"], "{:?}", fit.leftovers);
        let unchecked: Vec<&str> = fit
            .leftovers
            .iter()
            .filter(|l| l.t.is_none())
            .map(|l| l.name.as_str())
            .collect();
        assert_eq!(
            unchecked,
            ["SR.width", "S.left", "S.front", "W.left", "W.front"]
        );
    }

    /// What each reading of calibration sheet 2's strips and window would show, by a known
    /// calibration: the slit edges and head offsets put the cuts' edges where they land.
    fn expected(sheet: &Sheet, names: &[&str], truth: &Calibration) -> HashMap<String, f64> {
        let zeroed = Sheet {
            readings: names.iter().map(|n| ((*n).to_owned(), 0.0)).collect(),
            ..sheet.clone()
        };
        let (rows, _) = sheet_rows(&zeroed).expect("rows");
        let machine = truth.machine();
        let MachineMm([ox, oy]) = machine.board_origin_mm;
        // Each side of the slit on the back, outward from the line: the fit's edges.
        let edge = |h: Head, right: bool| {
            let [left, r] = compiler::Blade::fitted().head(h).sides(0.0);
            if right { -r } else { left }
        };
        let value = |u: Unknown| match u {
            Unknown::P(h, 0) => ox - machine.head(h).offset_mm[0],
            Unknown::P(h, _) => oy - machine.head(h).offset_mm[1],
            Unknown::E(h, right, Face::Front) => -edge(h, right) - 1.7,
            Unknown::E(h, right, _) => edge(h, right),
            Unknown::Drag(..) => 0.0,
        };
        rows.iter()
            .map(|r| {
                let model: f64 = r.terms.iter().map(|&(u, c)| c * value(u)).sum();
                // A row reads `value − geometry = model`, so the reading is model + geometry.
                (r.name.clone(), model - r.rhs)
            })
            .collect()
    }

    /// Calibration sheet 2's strips and window fix both heads' lines and slit edges; leave out
    /// the one reading from the board's left edge and the straight head's X is no longer fixed.
    #[test]
    fn a_plan_says_what_its_readings_will_fix() {
        let profile = Profile::gunnar_601();
        let p = crate::sheet::program(&profile);
        let all = [
            "SL.width",
            "SR.width",
            "SL.length",
            "S.left",
            "S.front",
            "W.left",
            "W.width",
            "W.front",
            "W.height",
            "D.width",
            "D.height",
        ];
        let with = |names: &[&str]| {
            let readings = names.iter().map(|n| ((*n).to_owned(), 0.0)).collect();
            Sheet::lowered("plan", p.clone(), &profile.raw(), readings).expect("lowers")
        };
        let full = precision(&with(&all), &[]).expect("a plan");
        assert_eq!(full.values.len(), 8);
        assert!(full.values.iter().all(|(_, se)| se.is_some()), "{full:?}");
        let px = Unknown::P(Head::Straight, 0);
        let without: Vec<&str> = all.iter().copied().filter(|n| *n != "S.left").collect();
        assert_eq!(
            precision(&with(&without), &[]).expect("a plan").get(px),
            None
        );
        // Known from the calibration to 0.05 mm, the straight X needs no reading of its own.
        let known = Prior {
            unknown: px,
            value: -6.19,
            se_mm: 0.05,
        };
        let se = precision(&with(&without), &[known])
            .expect("a plan")
            .get(px);
        assert!(se.is_some_and(|s| s <= 0.05), "{se:?}");
    }

    #[test]
    fn no_sheets_is_an_error_not_a_panic() {
        assert!(fit_sheets(&[]).is_err());
    }
}
