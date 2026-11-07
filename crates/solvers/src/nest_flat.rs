//! The nester's format ([`nest`]): bins and mats in, placements out. Lengths are whole numbers
//! (the UI sends tenths of a millimetre).
//!
//! In:
//! ```text
//! version (1), objective (0 bin packing, 1 knapsack), max width (0: default), growth (0: default),
//! bins (n), n × [w, h, trim left, trim right, trim top, trim bottom, copies],
//! items (m), m × [w, h, copies, profit (-1: the area), rotate (0 or 1)]
//! ```
//! Out: `bins used, profit, placements (k), k × [item, bin, x, y, w, h, rotated (0 or 1)]`, so
//! at most 3 + 7 × (all the copies) numbers.

use nest::{Bin, Instance, Item, Len, Objective, Options, Trims};

use crate::format::{Error, Reader};

pub const VERSION: f64 = 1.0;

/// A decoded input.
#[derive(Debug, Clone)]
pub struct Input {
    pub instance: Instance,
    pub options: Options,
}

fn len(r: &mut Reader<'_>) -> Result<Len, Error> {
    Len::try_from(r.int()?).map_err(|_| Error::BadNumber)
}

fn copies(r: &mut Reader<'_>) -> Result<u32, Error> {
    u32::try_from(r.whole()?).map_err(|_| Error::BadNumber)
}

pub fn decode(input: &[f64]) -> Result<Input, Error> {
    let mut r = Reader::new(input);
    if r.num()? != VERSION {
        return Err(Error::BadVersion);
    }
    let objective = if r.flag()? {
        Objective::Knapsack
    } else {
        Objective::BinPacking
    };
    let defaults = Options::default();
    let max_width = match r.whole()? {
        0 => defaults.max_width,
        w => usize::try_from(w).map_err(|_| Error::BadNumber)?,
    };
    let growth = match r.num()? {
        0.0 => defaults.growth,
        g if g > 1.0 => g,
        _ => return Err(Error::BadNumber),
    };
    let n = r.count(7)?;
    let mut bins = Vec::with_capacity(n);
    for _ in 0..n {
        let (w, h) = (len(&mut r)?, len(&mut r)?);
        let trims = Trims {
            left: len(&mut r)?,
            right: len(&mut r)?,
            top: len(&mut r)?,
            bottom: len(&mut r)?,
        };
        bins.push(Bin {
            w,
            h,
            trims,
            copies: copies(&mut r)?,
        });
    }
    let m = r.count(5)?;
    let mut items = Vec::with_capacity(m);
    for _ in 0..m {
        let (w, h, copies) = (len(&mut r)?, len(&mut r)?, copies(&mut r)?);
        let profit = match r.int()? {
            -1 => None,
            p if p >= 0 => Some(p),
            _ => return Err(Error::BadNumber),
        };
        items.push(Item {
            w,
            h,
            copies,
            profit,
            rotate: r.flag()?,
        });
    }
    r.end()?;
    Ok(Input {
        instance: Instance {
            objective,
            bins,
            items,
        },
        options: Options { max_width, growth },
    })
}

/// Decode an instance, lay it out, encode the layout.
pub fn solve(input: &[f64]) -> Result<Vec<f64>, Error> {
    let Input { instance, options } = decode(input)?;
    let s = nest::solve(&instance, &options)?;
    let mut out = Vec::with_capacity(3 + 7 * s.placements.len());
    out.extend([
        s.bins_used as f64,
        s.profit as f64,
        s.placements.len() as f64,
    ]);
    for p in &s.placements {
        let rotated = if p.rotated { 1.0 } else { 0.0 };
        out.extend([
            p.item as f64,
            p.bin as f64,
            p.x as f64,
            p.y as f64,
            p.w as f64,
            p.h as f64,
            rotated,
        ]);
    }
    Ok(out)
}
