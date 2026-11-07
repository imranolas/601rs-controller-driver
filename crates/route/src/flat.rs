//! A flat encoding for the WebAssembly boundary: one array of numbers in, one out, so the
//! browser needs no glue beyond copying arrays.
//!
//! In:
//! ```text
//! version (2), start x, start y, end (0 or 1), end x, end y, speed, accel,
//! seed, kicks (0: automatic), jobs (n),
//! for each job: ways (m), afters (a), flip (0 or 1),
//!               m × [entry x, entry y, exit x, exit y], m way indices if flip is 1,
//!               a job indices,
//! given route (0 or 1), and if 1: n job indices (the order), n way indices (by job)
//! ```
//! Out: `cost, n job indices (the order), n way indices (by job)`.
//!
//! Version 1 is the same without the three end numbers (no end).

use crate::{Error, Job, Options, Problem, Pt, Route, Travel, Way, optimise};

pub const FLAT_VERSION: f64 = 2.0;

/// Kicks beyond this are a mistake, not a request.
const MAX_KICKS: u64 = 1_000_000;

struct Reader<'a> {
    data: &'a [f64],
    at: usize,
}

impl Reader<'_> {
    fn num(&mut self) -> Result<f64, Error> {
        let v = *self.data.get(self.at).ok_or(Error::BadLength)?;
        self.at += 1;
        if v.is_finite() { Ok(v) } else { Err(Error::BadNumber) }
    }

    fn whole(&mut self) -> Result<u64, Error> {
        let v = self.num()?;
        if v < 0.0 || v.fract() != 0.0 || v > 9.0e15 { Err(Error::BadNumber) } else { Ok(v as u64) }
    }

    /// A whole number below `bound`.
    fn index(&mut self, bound: usize) -> Result<usize, Error> {
        let v = self.whole()?;
        usize::try_from(v).ok().filter(|&i| i < bound).ok_or(Error::BadNumber)
    }

    /// A count of items still to come, each at least `width` numbers long, so a bad count
    /// can't ask for a huge allocation.
    fn count(&mut self, width: usize) -> Result<usize, Error> {
        let v = self.whole()?;
        let left = (self.data.len() - self.at) as u64;
        if v.saturating_mul(width.max(1) as u64) > left {
            return Err(Error::BadLength);
        }
        usize::try_from(v).map_err(|_| Error::BadLength)
    }

    fn point(&mut self) -> Result<Pt, Error> {
        Ok([self.num()?, self.num()?])
    }

    fn indices(&mut self, count: usize, bound: usize) -> Result<Vec<usize>, Error> {
        (0..count).map(|_| self.index(bound)).collect()
    }
}

/// Decode a problem, optimise it, encode the route.
pub fn solve_flat(input: &[f64]) -> Result<Vec<f64>, Error> {
    let mut r = Reader { data: input, at: 0 };
    let version = r.num()?;
    if version != FLAT_VERSION && version != 1.0 {
        return Err(Error::BadVersion);
    }
    let start = r.point()?;
    let end = if version == 1.0 {
        None
    } else {
        let has = r.index(2)? == 1;
        let e = r.point()?;
        has.then_some(e)
    };
    let travel = Travel { speed: r.num()?, accel: r.num()? };
    let seed = r.whole()?;
    let kicks = usize::try_from(r.whole()?.min(MAX_KICKS)).map_err(|_| Error::BadNumber)?;
    let n = r.count(7)?;
    let mut jobs = Vec::with_capacity(n);
    for _ in 0..n {
        let m = r.count(4)?;
        let a = r.count(1)?;
        let flip = r.index(2)? == 1;
        let mut ways = Vec::with_capacity(m);
        for _ in 0..m {
            ways.push(Way { entry: r.point()?, exit: r.point()? });
        }
        let flip = if flip { Some(r.indices(m, m)?) } else { None };
        let after = r.indices(a, n)?;
        jobs.push(Job { ways, flip, after });
    }
    let given = if r.index(2)? == 1 {
        let order = r.indices(n, n)?;
        let ways = r.indices(n, usize::MAX)?;
        Some(Route { order, ways })
    } else {
        None
    };
    if r.at != input.len() {
        return Err(Error::BadLength);
    }
    let p = Problem::new(jobs, start, travel)?.with_end(end)?;
    let route = optimise(&p, given.as_ref(), &Options { seed, kicks, ..Options::default() });
    let mut out = Vec::with_capacity(1 + 2 * n);
    out.push(p.cost(&route));
    out.extend(route.order.iter().map(|&j| j as f64));
    out.extend(route.ways.iter().map(|&w| w as f64));
    Ok(out)
}
