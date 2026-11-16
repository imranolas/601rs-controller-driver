//! The fit (CALIBRATION.md §7), in the tool model's own terms (`compiler::HeadCut`), per head,
//! in carriage millimetres:
//!
//! - `P`: where the carriage puts the head's line on the corner where boards register, so a cut
//!   commanded at carriage `c` runs along board `c − P`. One pair per head: only the sum of a
//!   board origin and a head offset can be seen in a cut, so it is fitted as one;
//! - the slit: its middle `s` left of the line on the back, its width `w`, and how far it moves
//!   left per mm deeper, `r` (the tangent of the lean; none on the straight head). A depth `z`
//!   below the back, its left side lies `s + z·r + w/2` left of the line, its right side
//!   `w/2 − s − z·r` right of it;
//! - how far a cut starts before its commanded start, and stops short of its commanded end, on
//!   the back (calibration 1's overrun readings).
//!
//! An edge reading is `c − P + σe` (σ = ±1, the side read along the axis, `e` where that side
//! lands), a width is two edges' difference (`P` cancels), an overrun is `overcut ± lead or
//! shortfall − e`. Each is one linear row; the combs, read by eye, aren't fitted
//! ([`crate::runons`]). Every unknown is the same on any board, so sheets cut on different
//! boards fit together.

use std::collections::HashMap;

use compiler::Blade;
use machine::{MachineMm, Profile};
use nalgebra::{DMatrix, DVector};
use program::{Head, Op, Program};

use crate::key::{Axis, Surface};
use crate::reading::{Edge, End, Kind, Reading, Side, Span};
use crate::sent::{SentCut, cuts_from_lowering};
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
    /// How far a cut starts before its commanded start, on the back, mm.
    Lead(Head),
    /// How far a cut stops short of its commanded end, on the back, mm.
    Shortfall(Head),
}

impl Unknown {
    pub fn head(&self) -> Head {
        match *self {
            Unknown::P(h, _)
            | Unknown::Sideways(h)
            | Unknown::Width(h)
            | Unknown::Run(h)
            | Unknown::Lead(h)
            | Unknown::Shortfall(h) => h,
        }
    }

    fn order(&self) -> (u8, u8) {
        let h = u8::from(self.head() == Head::Bevel);
        match *self {
            Unknown::P(_, a) => (h, a.index() as u8),
            Unknown::Sideways(_) => (h, 2),
            Unknown::Width(_) => (h, 3),
            Unknown::Run(_) => (h, 4),
            Unknown::Lead(_) => (h, 5),
            Unknown::Shortfall(_) => (h, 6),
        }
    }

    /// How the report names it: `Px`, `sideways`, `run per mm`.
    pub fn label(&self) -> String {
        match *self {
            Unknown::P(_, Axis::X) => "Px",
            Unknown::P(_, Axis::Y) => "Py",
            Unknown::Sideways(_) => "sideways",
            Unknown::Width(_) => "slit width",
            Unknown::Run(_) => "run per mm",
            Unknown::Lead(_) => "lead, back",
            Unknown::Shortfall(_) => "shortfall, back",
        }
        .to_owned()
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

/// A cut as the fit sees it: head, start, end, unit direction (along X or Y), all in carriage
/// mm; and where it starts in the program's own frame, which orders cuts as they lie on the
/// board (in carriage mm, the two heads' cuts are their offset apart).
struct Line {
    head: Head,
    a: [f64; 2],
    b: [f64; 2],
    d: [f64; 2],
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
            b,
            d,
            planned,
        })
    }

    /// The side's direction along `axis`: ±1.
    fn sigma(&self, side: Side, axis: Axis) -> f64 {
        let toward = match side {
            Side::Right => [self.d[1], -self.d[0]],
            Side::Left => [-self.d[1], self.d[0]],
        };
        toward[axis.index()].round()
    }
}

/// What a caliper reading should show by the model: `Σ c·u + at`.
struct Lin {
    terms: Vec<(Unknown, f64)>,
    at: f64,
}

/// Every caliper reading `sheet` lists, as what the model says it reads, in the order listed.
fn lengths(sheet: &Sheet) -> Result<Vec<(&Reading, Lin)>, Error> {
    let label = &sheet.label;
    let bad = |m: String| Error::Sheet(format!("{label}: {m}"));
    // The program's cuts in order, matched to the cuts sent.
    let planned: Vec<&program::Cut> = sheet
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
            "{} cuts were sent, the program has {}: not this program's run, or it stopped",
            sheet.cuts.len(),
            planned.len()
        )));
    }
    for (k, (p, s)) in planned.iter().zip(&sheet.cuts).enumerate() {
        if p.head != s.head {
            return Err(bad(format!("cut {} was sent on the other head", k + 1)));
        }
    }
    let line = |id: &str| -> Result<Line, Error> {
        let k = planned
            .iter()
            .position(|c| c.id.as_deref() == Some(id))
            .ok_or_else(|| bad(format!("there's no cut {id}")))?;
        let start = planned[k].path_mm.first().copied().unwrap_or_default();
        Line::new(&sheet.cuts[k], start, id)
    };
    let thickness = sheet.program.board.thickness_mm;
    // Where an edge lands from its line, toward its side.
    let e = |head: Head, e: &Edge| {
        let depth = match e.surface {
            Surface::Back => 0.0,
            Surface::Face => thickness,
        };
        side_terms(head, e.side, depth)
    };
    let scaled =
        |terms: Vec<(Unknown, f64)>, k: f64| terms.into_iter().map(move |(u, c)| (u, c * k));
    let mut out = Vec::new();
    for r in &sheet.listed {
        let Kind::Length { span, .. } = &r.kind else {
            continue;
        };
        let lin = match span {
            Span::FromStock { axis, edge } => {
                let l = line(&edge.cut)?;
                let mut terms = vec![(Unknown::P(l.head, *axis), -1.0)];
                terms.extend(scaled(e(l.head, edge), l.sigma(edge.side, *axis)));
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
                let mut terms: Vec<(Unknown, f64)> =
                    scaled(e(l2.head, two), l2.sigma(two.side, *axis)).collect();
                terms.extend(scaled(e(l1.head, one), -l1.sigma(one.side, *axis)));
                if l1.head != l2.head {
                    terms.push((Unknown::P(l2.head, *axis), -1.0));
                    terms.push((Unknown::P(l1.head, *axis), 1.0));
                }
                Lin {
                    terms,
                    at: l2.a[i] - l1.a[i],
                }
            }
            Span::Overrun { cut, end, crossing } => {
                let (l, c) = (line(cut)?, line(&crossing.cut)?);
                let along = usize::from(l.d[1].abs() > 0.5);
                let at = c.a[along];
                let (overcut, mut terms) = match end {
                    End::Start => (
                        (at - l.a[along]) * l.d[along],
                        vec![(Unknown::Lead(l.head), 1.0)],
                    ),
                    End::End => (
                        (l.b[along] - at) * l.d[along],
                        vec![(Unknown::Shortfall(l.head), -1.0)],
                    ),
                };
                terms.extend(scaled(e(c.head, crossing), -1.0));
                Lin { terms, at: overcut }
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
        .find(|id| !sheet.listed.iter().any(|r| r.id == **id))
    {
        return Err(Error::Reading(format!(
            "{}: {id} is not a reading of this sheet",
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

/// One cut sheet: its program, the cuts it sent, the readings its program lists, and those
/// taken.
#[derive(Debug, Clone, PartialEq)]
pub struct Sheet {
    pub label: String,
    pub program: Program,
    pub cuts: Vec<SentCut>,
    /// Every reading the sheet has, in order: what each measures.
    pub listed: Vec<Reading>,
    /// The readings taken, by id: mm, or a comb's tooth.
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
        listed: Vec<Reading>,
        readings: HashMap<String, f64>,
    ) -> Result<Sheet, Error> {
        let cuts = cuts_from_lowering(&program, cut_with)?;
        Ok(Sheet {
            label: label.into(),
            program,
            cuts,
            listed,
            readings,
        })
    }

    /// The readings taken, each with its value, in the order listed.
    pub fn taken(&self) -> impl Iterator<Item = (&Reading, f64)> {
        self.listed
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
        Unknown::Lead(_) => c.lead_mm[0],
        Unknown::Shortfall(_) => c.shortfall_mm[0],
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
    use crate::Calibration;

    /// Calibration sheet 2, cut raw, with `readings`.
    fn sheet_2(readings: HashMap<String, f64>) -> Sheet {
        let profile = Profile::gunnar_601();
        let p = crate::sheet::program(&profile);
        let (_, listed) = crate::legacy::readings(&p).expect("listed");
        Sheet::lowered("sheet 2", p, &profile.raw(), listed, readings).expect("lowers")
    }

    /// Readings made up from known heads, worked out edge by edge from the sheet's own
    /// geometry, come back out of the fit.
    #[test]
    fn the_fit_recovers_known_heads() {
        let profile = Profile::gunnar_601();
        let sheet = sheet_2(HashMap::new());
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
        // Alone, the bevel's width, offset and lean are seen only together: held at none.
        let fit = fit_sheets(std::slice::from_ref(&sheet)).expect("fits");
        let (sb, wb, rb) = (
            Unknown::Sideways(Head::Bevel),
            Unknown::Width(Head::Bevel),
            Unknown::Run(Head::Bevel),
        );
        assert_eq!(fit.tied, vec![vec![sb, wb, rb]]);
        let fit =
            fit_sheets_with(std::slice::from_ref(&sheet), &[Prior::held(wb, 0.0)]).expect("fits");
        let combs = crate::combs(sheet.taken()).expect("combs");
        let near = |u: Unknown, want: f64| {
            let (v, _) = fit.get(u).expect("determined");
            assert!((v - want).abs() < 1e-6, "{u:?}: {v} against {want}");
        };
        let (s, b) = (Head::Straight, Head::Bevel);
        near(Unknown::P(s, Axis::X), sp[0]);
        near(Unknown::P(s, Axis::Y), sp[1]);
        // The slit's middle half the difference of its sides left of the line, its width their sum.
        near(Unknown::Sideways(s), (sl - sr) / 2.0);
        near(Unknown::Width(s), sl + sr);
        near(Unknown::P(b, Axis::X), bp[0]);
        near(Unknown::P(b, Axis::Y), bp[1]);
        // The back edge 0.4 out, the face edge 2.1: a run of 1.7 mm across the 1.5 mm board.
        near(sb, back);
        near(rb, (face - back) / 1.5);
        assert!(fit.sigma < 1e-6);
        assert_eq!(combs.len(), 1);
        assert_eq!((combs[0].tooth, combs[0].end), (5, End::End));
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

    /// Readings with a caliper's scatter, one misread by half a millimetre. On calibration sheet
    /// 2 a width has one twin (the window across and along both read the bevel's back edge), so
    /// a misread width is caught but not told from its twin: both stand out, and nothing else.
    /// The readings from the board's edges are each a head's only witness: nothing checks them.
    #[test]
    fn a_misread_stands_out_where_another_reading_checks_it() {
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
        let sheet = sheet_2(HashMap::new());
        let clean = predict(&sheet, &Calibration::new()).expect("predicted");
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
        let fit = fit_sheets(&[Sheet { readings, ..sheet }]).expect("fits");
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

    /// Calibration sheet 2's strips and window fix both heads' lines and slit edges; leave out
    /// the one reading from the board's left edge and the straight head's X is no longer fixed.
    #[test]
    fn a_plan_says_what_its_readings_will_fix() {
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
        let with = |names: &[&str]| sheet_2(names.iter().map(|n| ((*n).to_owned(), 0.0)).collect());
        // The bevel's width held, as stage 1 holds it: its sides are read on different faces.
        let held = Prior::held(Unknown::Width(Head::Bevel), 0.0);
        let full = precision(&with(&all), &[held]).expect("a plan");
        assert_eq!(full.values.len(), 9);
        assert!(full.values.iter().all(|(_, se)| se.is_some()), "{full:?}");
        let px = Unknown::P(Head::Straight, Axis::X);
        let without: Vec<&str> = all.iter().copied().filter(|n| *n != "S.left").collect();
        assert_eq!(
            precision(&with(&without), &[held]).expect("a plan").get(px),
            None
        );
        // Known from the calibration to 0.05 mm, the straight X needs no reading of its own.
        let known = Prior {
            unknown: px,
            value: -6.19,
            se_mm: 0.05,
        };
        let se = precision(&with(&without), &[held, known])
            .expect("a plan")
            .get(px);
        assert!(se.is_some_and(|s| s <= 0.05), "{se:?}");
    }

    #[test]
    fn no_sheets_is_an_error_not_a_panic() {
        assert!(fit_sheets(&[]).is_err());
    }
}
