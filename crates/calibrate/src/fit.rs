//! The fit (CALIBRATION.md §7), in the tool model's own terms (`compiler::HeadCut`), per head,
//! in carriage millimetres:
//!
//! - `P`: where the carriage puts the head's line on the corner where boards register, so a cut
//!   commanded at carriage `c` runs along board `c − P`. One pair per head: only the sum of a
//!   board origin and a head offset can be seen in a cut, so it is fitted as one;
//! - the slit: its middle `s` left of the line on the back, its width `w`, and how far it moves
//!   left per mm deeper, `r` (the tangent of the lean; none on the straight head). A depth `z`
//!   below the back, its left side lies `s + z·r + w/2` left of the line, its right side
//!   `w/2 − s − z·r` right of it.
//!
//! An edge reading is `c − P + σe` (σ = ±1, the side read along the axis, `e` where that side
//! lands), a width is two edges' difference (`P` cancels unless the heads differ). Each is one
//! linear row; the combs, read by eye, aren't fitted ([`crate::runons`]). Every unknown is the
//! same on any board, so sheets cut on different boards fit together.

use std::collections::HashMap;

use compiler::Blade;
use machine::{MachineMm, Profile};
use nalgebra::{DMatrix, DVector};
use program::{Head, Op};

use crate::key::{Axis, Surface};
use crate::plan::Plan;
use crate::reading::{Edge, Kind, Reading, Side, Span};
use crate::sent::SentCut;
use crate::{Calibration, Error};

/// One unknown of the fit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unknown {
    /// Where the carriage puts the head's line on the registered corner, mm: the board origin
    /// less the head's offset.
    P(Head, Axis),
    /// Where the slit's middle lies on the back, mm left of the line (`HeadCut::sideways_mm`).
    Sideways(Head),
    /// How far apart the slit's sides are, mm (`HeadCut::width_mm`).
    Width(Head),
    /// How far the slit moves left per mm deeper into the board: the tangent of the lean
    /// (`HeadCut::run_per_mm`). The bevel's only.
    Run(Head),
}

impl Unknown {
    pub fn head(&self) -> Head {
        match *self {
            Unknown::P(h, _) | Unknown::Sideways(h) | Unknown::Width(h) | Unknown::Run(h) => h,
        }
    }

    fn order(&self) -> (u8, u8) {
        let h = u8::from(self.head() == Head::Bevel);
        match *self {
            Unknown::P(_, a) => (h, a.index() as u8),
            Unknown::Sideways(_) => (h, 2),
            Unknown::Width(_) => (h, 3),
            Unknown::Run(_) => (h, 4),
        }
    }

    /// How the report names it: `Px`, `sideways`, `run per mm`.
    pub fn label(&self) -> &'static str {
        match *self {
            Unknown::P(_, Axis::X) => "Px",
            Unknown::P(_, Axis::Y) => "Py",
            Unknown::Sideways(_) => "sideways",
            Unknown::Width(_) => "slit width",
            Unknown::Run(_) => "run per mm",
        }
    }

    /// Its unit: mm, or mm per mm of board for the run.
    pub fn unit(&self) -> &'static str {
        match self {
            Unknown::Run(_) => "mm/mm",
            _ => "mm",
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

/// Where `side` of `head`'s slit lands from its line, `depth_mm` below the back, mm toward
/// that side: as terms in the unknowns. The straight head has no lean, so its sides are the
/// same on either face.
fn side_terms(head: Head, side: Side, depth_mm: f64) -> Vec<(Unknown, f64)> {
    let k = match side {
        Side::Left => 1.0,
        Side::Right => -1.0,
    };
    let mut terms = vec![(Unknown::Sideways(head), k), (Unknown::Width(head), 0.5)];
    if head == Head::Bevel {
        terms.push((Unknown::Run(head), k * depth_mm));
    }
    terms
}

/// A cut as the fit sees it: its head, where it starts in carriage mm, the side of it an edge
/// lies on along each axis (±1, for the side left of travel), and where it starts in the
/// program's own frame, which orders cuts as they lie on the board (in carriage mm, the two
/// heads' cuts are their offset apart).
struct Line {
    head: Head,
    a: [f64; 2],
    left: [f64; 2],
    planned: [f64; 2],
}

impl Line {
    fn new(c: &SentCut, planned: [f64; 2], what: &str) -> Result<Line, Error> {
        let (a, b) = (c.points[0], c.points[c.points.len() - 1]);
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        let d = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
        if d[0].abs() > 1e-6 && d[1].abs() > 1e-6 {
            return Err(Error::Sheet(format!(
                "{what}: calibration cuts run along X or Y"
            )));
        }
        Ok(Line {
            head: c.head,
            a,
            left: [-d[1], d[0]].map(f64::round),
            planned,
        })
    }

    /// The side's direction along `axis`: ±1.
    fn sigma(&self, side: Side, axis: Axis) -> f64 {
        let left = self.left[axis.index()];
        match side {
            Side::Left => left,
            Side::Right => -left,
        }
    }
}

/// What a caliper reading should show by the model: `Σ c·u + at`.
struct Lin {
    terms: Vec<(Unknown, f64)>,
    at: f64,
}

/// Every caliper reading of `sheet`'s plan, as what the model says it reads, in the plan's
/// order.
fn lengths(sheet: &Sheet) -> Result<Vec<(&Reading, Lin)>, Error> {
    let label = &sheet.label;
    let bad = |m: String| Error::Sheet(format!("{label}: {m}"));
    // The program's cuts in order, matched to the cuts sent.
    let planned: Vec<&program::Cut> = sheet
        .plan
        .program
        .ops
        .iter()
        .filter_map(|o| match o {
            Op::Cut(c) => Some(c),
            _ => None,
        })
        .collect();
    if planned.len() != sheet.cuts.len() {
        return Err(bad(format!(
            "{} cuts were sent, the program has {}",
            sheet.cuts.len(),
            planned.len()
        )));
    }
    let line = |id: &str| -> Result<Line, Error> {
        let k = planned
            .iter()
            .position(|c| c.id.as_deref() == Some(id))
            .ok_or_else(|| bad(format!("there's no cut {id}")))?;
        let start = planned[k].path_mm.first().copied().unwrap_or_default();
        Line::new(&sheet.cuts[k], start, id)
    };
    let thickness = sheet.plan.program.board.thickness_mm;
    // Where an edge lands from its line, along `axis`.
    let edge = |l: &Line, e: &Edge, axis: Axis| {
        let depth = match e.surface {
            Surface::Back => 0.0,
            Surface::Face => thickness,
        };
        let sigma = l.sigma(e.side, axis);
        side_terms(l.head, e.side, depth)
            .into_iter()
            .map(move |(u, c)| (u, c * sigma))
    };
    let mut out = Vec::new();
    for r in &sheet.plan.readings {
        let Kind::Length { span, .. } = &r.kind else {
            continue;
        };
        let lin = match span {
            Span::FromStock { axis, edge: e } => {
                let l = line(&e.cut)?;
                let mut terms = vec![(Unknown::P(l.head, *axis), -1.0)];
                terms.extend(edge(&l, e, *axis));
                Lin {
                    terms,
                    at: l.a[axis.index()],
                }
            }
            Span::Between { axis, edges } => {
                let i = axis.index();
                let (mut one, mut two) = (&edges[0], &edges[1]);
                let (mut l1, mut l2) = (line(&one.cut)?, line(&two.cut)?);
                // As they lie on the board: in carriage mm the heads are their offset apart.
                if l2.planned[i] < l1.planned[i] {
                    (one, two, l1, l2) = (two, one, l2, l1);
                }
                let mut terms: Vec<(Unknown, f64)> = edge(&l2, two, *axis).collect();
                terms.extend(edge(&l1, one, *axis).map(|(u, c)| (u, -c)));
                if l1.head != l2.head {
                    terms.push((Unknown::P(l2.head, *axis), -1.0));
                    terms.push((Unknown::P(l1.head, *axis), 1.0));
                }
                Lin {
                    terms,
                    at: l2.a[i] - l1.a[i],
                }
            }
        };
        out.push((r, lin));
    }
    Ok(out)
}

/// The rows of the caliper readings taken on `sheet`.
fn rows(sheet: &Sheet) -> Result<Vec<Row>, Error> {
    if let Some(id) = sheet
        .readings
        .keys()
        .find(|id| sheet.plan.reading(id).is_none())
    {
        return Err(Error::Reading(format!(
            "{}: {id} is not a reading of this plan",
            sheet.label
        )));
    }
    Ok(lengths(sheet)?
        .into_iter()
        .filter_map(|(r, lin)| {
            sheet.readings.get(&r.id).map(|value| Row {
                name: r.id.clone(),
                terms: lin.terms,
                rhs: value - lin.at,
                sd: 1.0,
            })
        })
        .collect())
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
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
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

/// Off by less than this, mm, a reading is no misread however closely the others agree: about
/// what calipers resolve on a cut edge. Without it, readings that fit almost exactly (made-up
/// ones, rounded) make a rounding error look like many scatters.
pub const MISREAD_MM: f64 = 0.02;

impl Leftover {
    /// Worth taking again: studentised beyond `t`, and off by at least [`MISREAD_MM`].
    pub fn suspect(&self, t: f64) -> bool {
        self.t.is_some_and(|v| v.abs() > t) && self.mm.abs() >= MISREAD_MM
    }
}

/// The fitted values: each unknown with its standard error (`None`: not determined).
#[derive(Debug, Clone, PartialEq)]
pub struct Fit {
    /// Each unknown, and its value and standard error if the readings fix it on its own (the
    /// error NaN when no reading is spare).
    pub values: Vec<(Unknown, Option<(f64, f64)>)>,
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
        let (px, sx) = self.get(Unknown::P(head, Axis::X))?;
        let (py, sy) = self.get(Unknown::P(head, Axis::Y))?;
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
            .find(|(k, _)| *k == u)
            .and_then(|(_, fixed)| *fixed)
    }

    /// The readings worth taking again ([`Leftover::suspect`]), worst first.
    pub fn suspects(&self, t: f64) -> Vec<&Leftover> {
        let mut out: Vec<&Leftover> = self.leftovers.iter().filter(|l| l.suspect(t)).collect();
        out.sort_by(|a, b| {
            let key = |l: &Leftover| l.t.map_or(0.0, f64::abs);
            key(b).total_cmp(&key(a))
        });
        out
    }
}

/// The fit of `rows`, the first `read` of them readings and the rest priors: only the readings
/// count as readings and have leftovers.
fn fit_rows(rows: &[Row], read: usize) -> Fit {
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
    let leftovers = rows[..read]
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
        .map(|(j, &u)| (u, s.x[j].map(|x| (x, sigma * s.var[j].sqrt()))))
        .collect();
    Fit {
        values,
        leftovers,
        sigma,
        rows: read,
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
    /// `unknown` held at `value`: known to a micrometre.
    pub fn held(unknown: Unknown, value: f64) -> Prior {
        Prior {
            unknown,
            value,
            se_mm: 0.001,
        }
    }

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
    let mut rows = rows(sheet)?;
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

/// A plan as cut: where its cuts went (by lowering its program with the profile it was placed
/// with, [`Plan::sheet`]), and the readings taken off it.
#[derive(Debug, Clone, PartialEq)]
pub struct Sheet {
    pub label: String,
    pub plan: Plan,
    pub cuts: Vec<SentCut>,
    /// The readings taken, by id: mm, or a comb's tooth.
    pub readings: HashMap<String, f64>,
}

impl Sheet {
    /// The readings taken, each with its value, in the plan's order.
    pub fn taken(&self) -> impl Iterator<Item = (&Reading, f64)> {
        self.plan
            .readings
            .iter()
            .filter_map(|r| self.readings.get(&r.id).map(|v| (r, *v)))
    }
}

/// What calibration `cal` says an unknown is: where the carriage puts each head's line on the
/// registered corner, and the head's tool model (`compiler::HeadCut`).
pub fn known(u: Unknown, cal: &Calibration) -> f64 {
    known_in(u, &cal.blade(), &cal.machine())
}

fn known_in(u: Unknown, blade: &Blade, machine: &Profile) -> f64 {
    let c = blade.head(u.head());
    match u {
        Unknown::P(h, axis) => {
            let MachineMm(origin) = machine.board_origin_mm;
            origin[axis.index()] - machine.head(h).offset_mm[axis.index()]
        }
        Unknown::Sideways(_) => c.sideways_mm,
        Unknown::Width(_) => c.width_mm,
        Unknown::Run(_) => c.run_per_mm(),
    }
}

/// What each caliper reading `sheet` lists should show by calibration `cal`, its cuts where
/// they went: a plan's expected values, and the readings the tests fit. The sheet's own
/// readings are ignored.
///
/// # Errors
/// When the sheet's cuts and readings don't agree with its program.
pub fn predict(sheet: &Sheet, cal: &Calibration) -> Result<HashMap<String, f64>, Error> {
    let (blade, machine) = (cal.blade(), cal.machine());
    Ok(lengths(sheet)?
        .into_iter()
        .map(|(r, lin)| {
            let model: f64 = lin
                .terms
                .iter()
                .map(|&(u, c)| c * known_in(u, &blade, &machine))
                .sum();
            (r.id.clone(), model + lin.at)
        })
        .collect())
}

/// Fit every sheet's caliper readings together.
///
/// # Errors
/// When there's no sheet, or a sheet's cuts and readings don't agree with its program.
pub fn fit_sheets(sheets: &[Sheet]) -> Result<Fit, Error> {
    fit_sheets_with(sheets, &[])
}

/// Fit every sheet's caliper readings together, with what is already known. Sheets on boards
/// of any thickness fit together: each reading's depth is its own board's.
///
/// # Errors
/// As [`fit_sheets`].
pub fn fit_sheets_with(sheets: &[Sheet], priors: &[Prior]) -> Result<Fit, Error> {
    if sheets.is_empty() {
        return Err(Error::Sheet("no sheets to fit".to_owned()));
    }
    let mut all = Vec::new();
    for s in sheets {
        all.extend(rows(s)?);
    }
    let read = all.len();
    all.extend(priors.iter().map(Prior::row));
    Ok(fit_rows(&all, read))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::Stock;

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
            Unknown::P(Head::Straight, Axis::X),
            Unknown::Sideways(Head::Straight),
        );
        let q = Unknown::P(Head::Bevel, Axis::X);
        let rows = [
            row("a", &[(p, 1.0), (e, 1.0)], 1.0, 1.0),
            row("b", &[(p, 1.0), (e, 1.0)], 1.2, 1.0),
            row("c", &[(q, 1.0)], 5.0, 1.0),
            row("d", &[(q, 1.0)], 5.2, 1.0),
        ];
        let fit = fit_rows(&rows, rows.len());
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
        let u = Unknown::P(Head::Bevel, Axis::Y);
        let rows = [
            row("calipers", &[(u, 1.0)], 1.0, 1.0),
            row("rule", &[(u, 1.0)], 2.0, 10.0),
        ];
        let fit = fit_rows(&rows, rows.len());
        let (v, _) = fit.get(u).expect("fixed");
        assert!((v - (1.0 + 2.0 / 100.0) / (1.0 + 1.0 / 100.0)).abs() < 1e-12);
    }

    /// Stage 1 fixes every unknown with the bevel's width held; leave out both readings from the
    /// board's left edge and nothing places either head along X, until the calibration says
    /// where the straight head's line is.
    #[test]
    fn a_plan_says_what_its_readings_will_fix() {
        let cal = Calibration::new();
        let plan = crate::heads::plan(&cal, &Stock::offcut(1.5)).expect("a plan");
        let held = crate::heads::priors(&cal);
        let with = |left_out: &[&str]| {
            let taken = plan
                .readings
                .iter()
                .filter(|r| !left_out.contains(&r.id.as_str()))
                .map(|r| (r.id.clone(), 0.0))
                .collect();
            plan.sheet("plan", taken).expect("a sheet")
        };
        let full = precision(&with(&[]), &held).expect("a plan");
        assert_eq!(full.values.len(), 9);
        assert!(full.values.iter().all(|(_, se)| se.is_some()), "{full:?}");
        let (px, bx) = (
            Unknown::P(Head::Straight, Axis::X),
            Unknown::P(Head::Bevel, Axis::X),
        );
        let without = precision(&with(&["S.left", "S.left2"]), &held).expect("a plan");
        assert_eq!((without.get(px), without.get(bx)), (None, None));
        assert_eq!(without.tied, vec![vec![px, bx]]);
        // Known from the calibration to 0.05 mm, the straight X needs no reading of its own,
        // and the borders carry it to the bevel's.
        let known = Prior {
            unknown: px,
            value: known(px, &cal),
            se_mm: 0.05,
        };
        let with_known =
            precision(&with(&["S.left", "S.left2"]), &[held[0], known]).expect("a plan");
        assert!(
            with_known.get(px).is_some_and(|s| s <= 0.05 + 1e-9),
            "{with_known:?}"
        );
        assert!(with_known.get(bx).is_some());
    }

    #[test]
    fn no_sheets_is_an_error_not_a_panic() {
        assert!(fit_sheets(&[]).is_err());
    }
}
