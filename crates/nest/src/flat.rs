//! A flat encoding for the WebAssembly boundary: one array of numbers in, one out, as for the
//! route optimiser, so the browser needs no glue beyond copying arrays. Lengths are whole
//! numbers (the UI sends tenths of a millimetre).
//!
//! In:
//! ```text
//! version (1), objective (0 bin packing, 1 knapsack), max width (0: default), growth (0: default),
//! bins (n), n × [w, h, trim left, trim right, trim top, trim bottom, copies],
//! items (m), m × [w, h, copies, profit (-1: the area), rotate (0 or 1)]
//! ```
//! Out: `bins used, profit, placements (k), k × [item, bin, x, y, w, h, rotated (0 or 1)]`, so
//! at most 3 + 7 × (all the copies) numbers.

use std::fmt;

use crate::{Bin, Error, Instance, Item, Len, Objective, Options, Trims, solve};

pub const FLAT_VERSION: f64 = 1.0;

/// Why a flat problem couldn't be solved.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlatError {
    /// A number isn't usable: not finite, or not a whole number where one is needed.
    BadNumber,
    /// The input ended early, or carried more than it said.
    BadLength,
    BadVersion,
    Solve(Error),
}

impl FlatError {
    /// A code for the WebAssembly boundary (always 2 or more; the shim negates it).
    pub fn code(&self) -> i32 {
        match self {
            FlatError::BadNumber => 2,
            FlatError::BadLength => 3,
            FlatError::BadVersion => 4,
            FlatError::Solve(Error::BadSize) => 5,
            FlatError::Solve(Error::NoBins) => 6,
            FlatError::Solve(Error::TooBig(_)) => 7,
            FlatError::Solve(Error::TooMany) => 8,
            FlatError::Solve(Error::Bug(_)) => 9,
        }
    }
}

impl fmt::Display for FlatError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FlatError::BadNumber => write!(f, "a number in the input isn't usable"),
            FlatError::BadLength => write!(f, "the input is the wrong length"),
            FlatError::BadVersion => write!(f, "the input is for another version of the format"),
            FlatError::Solve(e) => e.fmt(f),
        }
    }
}

impl std::error::Error for FlatError {}

struct Reader<'a> {
    data: &'a [f64],
    at: usize,
}

impl Reader<'_> {
    fn num(&mut self) -> Result<f64, FlatError> {
        let v = *self.data.get(self.at).ok_or(FlatError::BadLength)?;
        self.at += 1;
        if v.is_finite() { Ok(v) } else { Err(FlatError::BadNumber) }
    }

    /// A whole number in -2³¹..2³¹.
    fn int(&mut self) -> Result<i64, FlatError> {
        let v = self.num()?;
        if v.fract() != 0.0 || v.abs() > i32::MAX as f64 { Err(FlatError::BadNumber) } else { Ok(v as i64) }
    }

    fn len(&mut self) -> Result<Len, FlatError> {
        Ok(self.int()? as Len)
    }

    fn count(&mut self, width: usize) -> Result<usize, FlatError> {
        let v = self.int()?;
        let v = usize::try_from(v).map_err(|_| FlatError::BadNumber)?;
        if v.saturating_mul(width) > self.data.len() - self.at {
            return Err(FlatError::BadLength);
        }
        Ok(v)
    }

    fn whole(&mut self) -> Result<u32, FlatError> {
        u32::try_from(self.int()?).map_err(|_| FlatError::BadNumber)
    }
}

/// Decode a problem, lay it out, encode the layout.
pub fn solve_flat(input: &[f64]) -> Result<Vec<f64>, FlatError> {
    let mut r = Reader { data: input, at: 0 };
    if r.num()? != FLAT_VERSION {
        return Err(FlatError::BadVersion);
    }
    let objective = match r.int()? {
        0 => Objective::BinPacking,
        1 => Objective::Knapsack,
        _ => return Err(FlatError::BadNumber),
    };
    let defaults = Options::default();
    let max_width = match r.whole()? {
        0 => defaults.max_width,
        w => w as usize,
    };
    let g = r.num()?;
    let growth = if g == 0.0 {
        defaults.growth
    } else if g > 1.0 {
        g
    } else {
        return Err(FlatError::BadNumber);
    };
    let n = r.count(7)?;
    let mut bins = Vec::with_capacity(n);
    for _ in 0..n {
        let (w, h) = (r.len()?, r.len()?);
        let trims = Trims { left: r.len()?, right: r.len()?, top: r.len()?, bottom: r.len()? };
        bins.push(Bin { w, h, trims, copies: r.whole()? });
    }
    let m = r.count(5)?;
    let mut items = Vec::with_capacity(m);
    for _ in 0..m {
        let (w, h, copies) = (r.len()?, r.len()?, r.whole()?);
        let profit = match r.int()? {
            -1 => None,
            p if p >= 0 => Some(p),
            _ => return Err(FlatError::BadNumber),
        };
        let rotate = match r.int()? {
            0 => false,
            1 => true,
            _ => return Err(FlatError::BadNumber),
        };
        items.push(Item { w, h, copies, profit, rotate });
    }
    if r.at != input.len() {
        return Err(FlatError::BadLength);
    }
    let instance = Instance { objective, bins, items };
    let s = solve(&instance, &Options { max_width, growth }).map_err(FlatError::Solve)?;
    let mut out = Vec::with_capacity(3 + 7 * s.placements.len());
    out.extend([s.bins_used as f64, s.profit as f64, s.placements.len() as f64]);
    for p in &s.placements {
        out.extend([p.item as f64, p.bin as f64, p.x as f64, p.y as f64, p.w as f64, p.h as f64, if p.rotated { 1.0 } else { 0.0 }]);
    }
    Ok(out)
}
