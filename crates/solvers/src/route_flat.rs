//! The route optimiser's format ([`route`]): a sheet's cuts in, their order and ways out.
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

use route::{Job, Options, Problem, Pt, Route, Travel, Way, optimise};

use crate::format::{Error, Reader};

pub const VERSION: f64 = 2.0;

/// Kicks beyond this are a mistake, not a request.
const MAX_KICKS: u64 = 1_000_000;

/// A decoded input.
#[derive(Debug, Clone)]
pub struct Input {
    pub problem: Problem,
    /// The route to start from (and never do worse than).
    pub given: Option<Route>,
    /// The seed and the kicks; the rest are the defaults.
    pub options: Options,
}

fn point(r: &mut Reader<'_>) -> Result<Pt, Error> {
    Ok([r.num()?, r.num()?])
}

pub fn decode(input: &[f64]) -> Result<Input, Error> {
    let mut r = Reader::new(input);
    let version = r.num()?;
    if version != VERSION && version != 1.0 {
        return Err(Error::BadVersion);
    }
    let start = point(&mut r)?;
    let end = if version == 1.0 {
        None
    } else {
        let has = r.flag()?;
        let e = point(&mut r)?;
        has.then_some(e)
    };
    let travel = Travel {
        speed: r.num()?,
        accel: r.num()?,
    };
    let seed = r.whole()?;
    let kicks = usize::try_from(r.whole()?.min(MAX_KICKS)).map_err(|_| Error::BadNumber)?;
    let n = r.count(7)?;
    let mut jobs = Vec::with_capacity(n);
    for _ in 0..n {
        let m = r.count(4)?;
        let a = r.count(1)?;
        let flip = r.flag()?;
        let mut ways = Vec::with_capacity(m);
        for _ in 0..m {
            ways.push(Way {
                entry: point(&mut r)?,
                exit: point(&mut r)?,
            });
        }
        let flip = if flip { Some(r.indices(m, m)?) } else { None };
        let after = r.indices(a, n)?;
        jobs.push(Job { ways, flip, after });
    }
    let given = if r.flag()? {
        let order = r.indices(n, n)?;
        let ways = r.indices(n, usize::MAX)?;
        Some(Route { order, ways })
    } else {
        None
    };
    r.end()?;
    let problem = Problem::new(jobs, start, travel)?.with_end(end)?;
    Ok(Input {
        problem,
        given,
        options: Options {
            seed,
            kicks,
            ..Options::default()
        },
    })
}

/// Decode a problem, optimise it, encode the route.
pub fn solve(input: &[f64]) -> Result<Vec<f64>, Error> {
    let Input {
        problem: p,
        given,
        options,
    } = decode(input)?;
    let route = optimise(&p, given.as_ref(), &options);
    let mut out = Vec::with_capacity(1 + 2 * route.order.len());
    out.push(p.cost(&route));
    out.extend(route.order.iter().map(|&j| j as f64));
    out.extend(route.ways.iter().map(|&w| w as f64));
    Ok(out)
}
