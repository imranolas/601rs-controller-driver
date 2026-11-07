//! An independent check of a layout: counts, sizes and turns, trims, overlaps, and the cut
//! structure (three stages, the first horizontal, each last-stage piece one item: an item may
//! be narrower than its column, trimmed by a fourth cut).

use std::fmt;

use crate::{Instance, Len, Objective, Placement, Solution};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Violation {
    /// Bin packing places every copy; knapsack no more than the copies.
    Count {
        item: usize,
        placed: u32,
        copies: u32,
    },
    /// A placement names an item or bin that doesn't exist.
    Unknown(usize),
    /// A placement's footprint isn't its item's, turned only if the item may turn.
    Size(usize),
    /// A placement reaches into the trims or past the bin.
    Outside(usize),
    /// Two placements overlap.
    Overlap(usize, usize),
    /// A bin's layout can't be cut with three guillotine stages, the first horizontal.
    NotThreeStage { bin: usize },
    /// `bins_used` or `profit` doesn't match the placements.
    Totals,
}

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Violation::Count {
                item,
                placed,
                copies,
            } => write!(f, "item {item}: {placed} placed, {copies} copies"),
            Violation::Unknown(p) => {
                write!(f, "placement {p} names an item or bin that doesn't exist")
            }
            Violation::Size(p) => write!(f, "placement {p} isn't its item's size"),
            Violation::Outside(p) => write!(f, "placement {p} reaches into the trims"),
            Violation::Overlap(a, b) => write!(f, "placements {a} and {b} overlap"),
            Violation::NotThreeStage { bin } => {
                write!(f, "bin {bin} isn't a three-stage guillotine layout")
            }
            Violation::Totals => write!(f, "the totals don't match the placements"),
        }
    }
}

impl std::error::Error for Violation {}

/// Check a layout against its instance.
pub fn check(instance: &Instance, solution: &Solution) -> Result<(), Violation> {
    let bins = instance.bin_sequence();
    let mut placed = vec![0u32; instance.items.len()];
    for (k, p) in solution.placements.iter().enumerate() {
        let (Some(item), Some(bin)) = (instance.items.get(p.item), bins.get(p.bin)) else {
            return Err(Violation::Unknown(k));
        };
        let size_ok = (p.w, p.h, p.rotated) == (item.w, item.h, false)
            || (item.rotate && (p.w, p.h, p.rotated) == (item.h, item.w, item.w != item.h));
        if !size_ok {
            return Err(Violation::Size(k));
        }
        let t = bin.trims;
        if p.x < t.left
            || p.y < t.top
            || p.x + p.w > bin.w - t.right
            || p.y + p.h > bin.h - t.bottom
        {
            return Err(Violation::Outside(k));
        }
        placed[p.item] += 1;
    }
    for (item, (&n, it)) in placed.iter().zip(&instance.items).enumerate() {
        let wrong = match instance.objective {
            Objective::BinPacking => n != it.copies,
            Objective::Knapsack => n > it.copies,
        };
        if wrong {
            return Err(Violation::Count {
                item,
                placed: n,
                copies: it.copies,
            });
        }
    }
    let ps = &solution.placements;
    for a in 0..ps.len() {
        for b in a + 1..ps.len() {
            let (p, q) = (&ps[a], &ps[b]);
            if p.bin == q.bin
                && p.x < q.x + q.w
                && q.x < p.x + p.w
                && p.y < q.y + q.h
                && q.y < p.y + p.h
            {
                return Err(Violation::Overlap(a, b));
            }
        }
    }
    let used = ps.iter().map(|p| p.bin + 1).max().unwrap_or(0);
    let profit: i64 = ps.iter().map(|p| instance.profit(p.item)).sum();
    if used != solution.bins_used || profit != solution.profit {
        return Err(Violation::Totals);
    }
    for bin in 0..used {
        let rects: Vec<&Placement> = ps.iter().filter(|p| p.bin == bin).collect();
        if !three_stage(&rects) {
            return Err(Violation::NotThreeStage { bin });
        }
    }
    Ok(())
}

/// Three guillotine stages, the first horizontal: the bin splits into full-width bands
/// wherever no item crosses, each band into columns, each column into pieces of one item.
fn three_stage(rects: &[&Placement]) -> bool {
    let rows = |r: &&Placement| (r.y, r.y + r.h);
    let cols = |r: &&Placement| (r.x, r.x + r.w);
    groups(rects, rows).iter().all(|band| {
        groups(band, cols)
            .iter()
            .all(|column| groups(column, rows).iter().all(|piece| piece.len() == 1))
    })
}

/// Split rectangles into groups whose intervals (as `span` gives them) overlap, chained;
/// touching intervals don't overlap, so a cut can run between them.
fn groups<'a>(
    rects: &[&'a Placement],
    span: impl Fn(&&Placement) -> (Len, Len),
) -> Vec<Vec<&'a Placement>> {
    let mut sorted: Vec<&Placement> = rects.to_vec();
    sorted.sort_by_key(|r| span(r).0);
    let mut out: Vec<Vec<&Placement>> = Vec::new();
    let mut end = Len::MIN;
    for r in sorted {
        let (a, b) = span(&r);
        match out.last_mut() {
            Some(g) if a < end => {
                g.push(r);
                end = end.max(b);
            }
            _ => {
                out.push(vec![r]);
                end = b;
            }
        }
    }
    out
}
