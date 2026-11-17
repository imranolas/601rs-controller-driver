//! Nesting mounts into shaped bins: the drop-out of an oval window, an offcut with a corner cut
//! away, a board with another piece still in it. The mounts are rectangles kept square to the
//! board (a quarter turn at most, so the straight head cuts them true); the bins are polygons
//! with holes.
//!
//! [`solve`] fills each bin's straight parts with guillotine layouts, whose mounts share cuts,
//! and the rest with the irregular nester (`irregular/mixed.rs`); [`solve_free`] is the irregular nester
//! alone.
//!
//! The irregular nester is PackingSolver's irregular tree search (Florian Fontan, MIT licence,
//! <https://github.com/fontanf/packingsolver>, `src/irregular/tree_search.cpp`), for
//! rectangles: one mount per step, resting on the bin's floor or on the top of the mount
//! placed before it, slid right along that support until it overlaps nothing; mounts that could
//! go somewhere before keep their place unless the new mount is in the way, and then slide on
//! from there. The search is the guillotine layouts' iterative beam search, with PackingSolver's
//! guides, in two directions (left to right, and bottom to top).
//!
//! Left out, against PackingSolver: shapes other than rectangles, spacing, simplifying the
//! bins (the caller flattens curves), the skyline (here a mount may slide into a gap behind the
//! others), dominance between partial layouts, and its other algorithms (the maximal spaces
//! search, periodic packings, local search).
//!
//! Lengths are integers, as for [`crate::solve`]. Mounts are placed exactly inside the
//! polygons and know nothing of the curves they stand for, so flatten a bin's outline inside
//! the real edge (vertices on a convex curve, rounded in) and a hole outside it (vertices on
//! the polygon round the curve, rounded out), never across.

mod geometry;
mod mixed;
mod scheme;

use crate::{Area, Error, Item, Len, Objective, Options, Placement, Solution, Violation};

/// A vertex.
pub type Point = [Len; 2];

/// A bin's shape: its outline and the holes in it, each a ring of vertices, either way round.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Region {
    pub outline: Vec<Point>,
    pub holes: Vec<Vec<Point>>,
}

impl Region {
    /// A `w` × `h` rectangle with its bottom-left corner at the origin.
    pub fn rect(w: Len, h: Len) -> Region {
        Region {
            outline: vec![[0, 0], [w, 0], [w, h], [0, h]],
            holes: Vec::new(),
        }
    }

    /// The rings with the outline anticlockwise and the holes clockwise: the free side of every
    /// edge is on its left.
    pub(crate) fn rings(&self) -> Vec<Vec<Point>> {
        let turn = |ring: &Vec<Point>, anticlockwise: bool| {
            let mut r = ring.clone();
            if (geometry::area2(&r) > 0) != anticlockwise {
                r.reverse();
            }
            r
        };
        std::iter::once(turn(&self.outline, true))
            .chain(self.holes.iter().map(|h| turn(h, false)))
            .collect()
    }

    /// The outline's bounding box: x0, y0, x1, y1.
    pub(crate) fn bbox(&self) -> [Len; 4] {
        let xs = self.outline.iter().map(|p| p[0]);
        let ys = self.outline.iter().map(|p| p[1]);
        [
            xs.clone().min().unwrap_or(0),
            ys.clone().min().unwrap_or(0),
            xs.max().unwrap_or(0),
            ys.max().unwrap_or(0),
        ]
    }

    /// The free area: the outline's less its holes'.
    pub fn area(&self) -> Area {
        let holes: i64 = self.holes.iter().map(|h| geometry::area2(h).abs()).sum();
        (geometry::area2(&self.outline).abs() - holes) / 2
    }

    /// The rectangle (x, y, w, h) lies inside, touching the edges at most.
    pub fn contains(&self, r: [Len; 4]) -> bool {
        geometry::contains(&self.rings(), r)
    }

    /// The same region with x and y swapped.
    pub(crate) fn transposed(&self) -> Region {
        let swap = |ring: &Vec<Point>| ring.iter().map(|p| [p[1], p[0]]).collect();
        Region {
            outline: swap(&self.outline),
            holes: self.holes.iter().map(swap).collect(),
        }
    }
}

/// A bin type: a drop-out, an offcut, a sheet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bin {
    pub region: Region,
    pub copies: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Instance {
    pub objective: Objective,
    pub bins: Vec<Bin>,
    pub items: Vec<Item>,
}

impl Instance {
    /// The bins with their copies laid out in order: bin indices of a [`Solution`] count these.
    pub fn bin_sequence(&self) -> Vec<&Bin> {
        self.bins
            .iter()
            .flat_map(|b| std::iter::repeat_n(b, b.copies as usize))
            .collect()
    }

    pub fn profit(&self, item: usize) -> Area {
        let it = &self.items[item];
        it.profit.unwrap_or(it.w as Area * it.h as Area)
    }

    fn validate(&self) -> Result<(), Error> {
        if self.bins.is_empty() {
            return Err(Error::NoBins);
        }
        for b in &self.bins {
            let r = &b.region;
            let ring_ok = |ring: &Vec<Point>| ring.len() >= 3 && geometry::area2(ring) != 0;
            if !ring_ok(&r.outline) || !r.holes.iter().all(ring_ok) || r.area() <= 0 {
                return Err(Error::BadSize);
            }
        }
        if self.items.iter().any(|i| i.w <= 0 || i.h <= 0) {
            return Err(Error::BadSize);
        }
        let copies: u64 = self.bins.iter().map(|b| b.copies as u64).sum();
        if copies > u16::MAX as u64
            || self.items.len() > usize::from(u16::MAX)
            || self.items.iter().any(|i| i.copies > u16::MAX as u32)
        {
            return Err(Error::TooMany);
        }
        Ok(())
    }
}

/// Lay out the items, one bin at a time (for knapsack the smallest first, as the irregular
/// search does; for bin packing in order): guillotine layouts in each bin's straight parts and
/// the irregular nester round the rest, unless the irregular nester alone places more
/// (`irregular/mixed.rs`). Groups ([`Item::group`]) then choose which copies of
/// interchangeable mounts go where, as for [`crate::solve`]; the irregular search itself knows
/// nothing of them. Every layout is checked ([`check`]) before it is returned. Bin packing that
/// can't place every mount is refused: with the first item that fits in no bin on its own,
/// or if each fits but the bins run out, the first item left out.
pub fn solve(instance: &Instance, options: &Options) -> Result<Solution, Error> {
    instance.validate()?;
    let bins = instance.bin_sequence();
    let mut order: Vec<usize> = (0..bins.len()).collect();
    if instance.objective == Objective::Knapsack {
        order.sort_by_key(|&b| bins[b].region.area());
    }
    let mut left: Vec<u32> = instance.items.iter().map(|i| i.copies).collect();
    let mut solution = Solution::default();
    for b in order {
        if left.iter().all(|&n| n == 0) {
            break;
        }
        let items: Vec<Item> = instance
            .items
            .iter()
            .zip(&left)
            .map(|(it, &copies)| Item {
                copies,
                ..it.clone()
            })
            .collect();
        for p in mixed::fill(&bins[b].region, &items, options) {
            left[p.item] -= 1;
            solution.placements.push(Placement { bin: b, ..p });
        }
    }
    finish(instance, solution)
}

/// The irregular nester alone, on all the bins at once: no guillotine layouts.
pub fn solve_free(instance: &Instance, options: &Options) -> Result<Solution, Error> {
    instance.validate()?;
    finish(instance, scheme::solve(instance, options))
}

/// Totals, the refusal of an incomplete bin packing, and the check.
fn finish(instance: &Instance, mut solution: Solution) -> Result<Solution, Error> {
    solution.profit = solution
        .placements
        .iter()
        .map(|p| instance.profit(p.item))
        .sum();
    solution.bins_used = solution
        .placements
        .iter()
        .map(|p| p.bin + 1)
        .max()
        .unwrap_or(0);
    if instance.objective == Objective::BinPacking {
        let mut placed = vec![0u32; instance.items.len()];
        for p in &solution.placements {
            placed[p.item] += 1;
        }
        let items = &instance.items;
        if let Some(short) = (0..items.len()).find(|&k| placed[k] < items[k].copies) {
            let fits_alone = |k: usize| {
                let one = Instance {
                    objective: Objective::Knapsack,
                    bins: instance.bins.clone(),
                    items: vec![Item {
                        copies: 1,
                        ..items[k].clone()
                    }],
                };
                let quick = Options {
                    max_width: 1,
                    growth: 2.0,
                };
                !scheme::solve(&one, &quick).placements.is_empty()
            };
            let k = (0..items.len())
                .find(|&k| items[k].copies > 0 && !fits_alone(k))
                .unwrap_or(short);
            return Err(Error::TooBig(k));
        }
    }
    crate::groups::relabel(
        instance.objective,
        &instance.items,
        |k| instance.profit(k),
        &mut solution.placements,
    );
    check(instance, &solution).map_err(Error::Bug)?;
    Ok(solution)
}

/// An independent check of a layout: counts, sizes and turns, every mount inside its bin's
/// region (exactly, on integers), no overlaps, and the totals.
pub fn check(instance: &Instance, solution: &Solution) -> Result<(), Violation> {
    let bins = instance.bin_sequence();
    let rings: Vec<Vec<Vec<Point>>> = bins.iter().map(|b| b.region.rings()).collect();
    let mut placed = vec![0u32; instance.items.len()];
    for (k, p) in solution.placements.iter().enumerate() {
        let (Some(item), Some(bin_rings)) = (instance.items.get(p.item), rings.get(p.bin)) else {
            return Err(Violation::Unknown(k));
        };
        let size_ok = (p.w, p.h, p.rotated) == (item.w, item.h, false)
            || (item.rotate && (p.w, p.h, p.rotated) == (item.h, item.w, item.w != item.h));
        if !size_ok {
            return Err(Violation::Size(k));
        }
        if !geometry::contains(bin_rings, [p.x, p.y, p.w, p.h]) {
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
            if p.bin == q.bin && geometry::overlap([p.x, p.y, p.w, p.h], [q.x, q.y, q.w, q.h]) {
                return Err(Violation::Overlap(a, b));
            }
        }
    }
    let used = ps.iter().map(|p| p.bin + 1).max().unwrap_or(0);
    let profit: Area = ps.iter().map(|p| instance.profit(p.item)).sum();
    if used != solution.bins_used || profit != solution.profit {
        return Err(Violation::Totals);
    }
    Ok(())
}

/// The straight-head strokes a layout's mount edges need: edges on one line that overlap or
/// touch are one stroke, as for the guillotine layouts.
pub fn strokes(solution: &Solution) -> usize {
    crate::strokes::strokes(
        solution
            .placements
            .iter()
            .map(|p| (p.bin, p.x, p.y, p.w, p.h)),
    )
    .0
}

#[cfg(test)]
mod tests;
