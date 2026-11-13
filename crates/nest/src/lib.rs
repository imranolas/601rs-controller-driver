//! Nesting mats onto sheets of mount board: three-stage guillotine layouts, the first stage
//! horizontal (full-width strips, so every strip reaches the clamped side), mats edge to edge
//! (zero kerf: neighbours share their cuts), hard trims at the sheet's edges, and mats turned a
//! quarter turn where the board allows it.
//!
//! Two problems:
//! - [`Objective::BinPacking`]: every mat, on as few bins as possible, used in the order
//!   given (offcuts first, then full sheets), and then as little of the last bin as possible,
//!   so the offcuts are used up and the last sheet keeps the biggest piece (a full-width band
//!   along its top);
//!   and then the fewest strokes (the straight-head cuts of the mats' edges, `strokes`);
//! - [`Objective::Knapsack`]: as much of the mats as fits (by profit, area by default) on the
//!   bins given, e.g. the drop-outs of other mats' openings.
//!
//! The method is PackingSolver's (Florian Fontan, MIT licence,
//! <https://github.com/fontanf/packingsolver>): its tree search for guillotine cutting, an
//! iterative beam search over partial layouts built one mat at a time. This is a reimplementation
//! of that design for the cases above only (no defects, minimum waste, cut limits or kerf).
//! Every layout is then rearranged within its strips and columns for fewer strokes.
//!
//! Lengths are integers in any unit (the UI uses tenths of a millimetre), so positions add up
//! exactly. Coordinates are the board frame's (PROGRAM_FORMAT §2): the origin at a bin's
//! bottom-left corner, where the board registers by the left clamp, x to the right and y up,
//! away from the operator. Layouts start from that corner: the first strip lies along the
//! bottom edge.

mod beam;
mod check;
mod guillotine;
pub mod irregular;
mod strokes;

pub use check::{Violation, check};

/// A length, in the instance's unit.
pub type Len = i32;
/// An area, or a profit.
pub type Area = i64;

/// Trims kept clear at a bin's edges.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Trims {
    pub left: Len,
    pub right: Len,
    pub top: Len,
    pub bottom: Len,
}

/// A bin type: a sheet, or a drop-out.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bin {
    pub w: Len,
    pub h: Len,
    pub trims: Trims,
    pub copies: u32,
}

/// An item type: a mat's footprint.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Item {
    pub w: Len,
    pub h: Len,
    pub copies: u32,
    /// For [`Objective::Knapsack`]; the area if `None`.
    pub profit: Option<Area>,
    /// May be turned a quarter turn.
    pub rotate: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Objective {
    /// Every item, on as few bins as possible, used in order (offcuts first, then full
    /// sheets); then as little of the last bin as possible.
    BinPacking,
    /// As much profit as fits on the bins.
    Knapsack,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Instance {
    pub objective: Objective,
    pub bins: Vec<Bin>,
    pub items: Vec<Item>,
}

/// One placed item: its type, its bin (counting copies: the bins of the instance expanded in
/// order), and its footprint there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Placement {
    pub item: usize,
    pub bin: usize,
    pub x: Len,
    pub y: Len,
    pub w: Len,
    pub h: Len,
    /// Turned a quarter turn from the item type's w × h.
    pub rotated: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Solution {
    pub placements: Vec<Placement>,
    /// Bins used: the highest bin index used, plus one (a knapsack may skip bins nothing
    /// fits in).
    pub bins_used: usize,
    pub profit: Area,
}

/// How hard to search.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    /// The widest beam: each pass keeps at most this many partial layouts per depth.
    pub max_width: usize,
    /// Each pass's beam is this much wider than the last (from 1).
    pub growth: f64,
}

impl Default for Options {
    /// Widths 1, 4, 16, 64, 256, 1024. On the benchmark queues (195, from 4 to ~160 mats,
    /// against PackingSolver's results) every sheet count matched by 256; at 1024 the last
    /// sheet's leftover matched PackingSolver's leftovers objective on 163 of 165 (6 and 11 mm
    /// short on the other two). A gentler growth repeats more passes for nothing.
    fn default() -> Self {
        Self {
            max_width: 1024,
            growth: 4.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    /// A bin or an item has no area, or trims leave a bin no room.
    #[error("a bin or an item has no area, or its trims leave no room")]
    BadSize,
    /// No bins.
    #[error("no bins")]
    NoBins,
    /// Item `0` fits no bin (bin packing needs every item).
    #[error("item {0} fits no bin")]
    TooBig(usize),
    /// More than 65,535 copies of an item, item types or bins.
    #[error("too many copies, item types or bins")]
    TooMany,
    /// The layout failed its own check: a bug, reported rather than cut. The message carries
    /// the violation, so it is not also given as the source.
    #[error("the layout failed its own check ({0}): a bug")]
    Bug(Violation),
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
            let t = b.trims;
            if b.w <= 0 || b.h <= 0 || t.left < 0 || t.right < 0 || t.top < 0 || t.bottom < 0 {
                return Err(Error::BadSize);
            }
            if t.left + t.right >= b.w || t.top + t.bottom >= b.h {
                return Err(Error::BadSize);
            }
        }
        if self.items.iter().any(|i| i.w <= 0 || i.h <= 0) {
            return Err(Error::BadSize);
        }
        let copies: u64 = self.bins.iter().map(|b| b.copies as u64).sum();
        // The search keeps item types and counts as u16.
        if copies > u16::MAX as u64
            || self.items.len() > usize::from(u16::MAX)
            || self.items.iter().any(|i| i.copies > u16::MAX as u32)
        {
            return Err(Error::TooMany);
        }
        if self.objective == Objective::BinPacking {
            for (k, it) in self.items.iter().enumerate() {
                let fits = |w: Len, h: Len| {
                    self.bins.iter().any(|b| {
                        w <= b.w - b.trims.left - b.trims.right
                            && h <= b.h - b.trims.top - b.trims.bottom
                    })
                };
                if it.copies > 0 && !fits(it.w, it.h) && !(it.rotate && fits(it.h, it.w)) {
                    return Err(Error::TooBig(k));
                }
            }
        }
        Ok(())
    }
}

/// Lay out the items. Identical item types are searched as one (no symmetric layouts), and
/// every layout is checked ([`check`]) before it is returned.
pub fn solve(instance: &Instance, options: &Options) -> Result<Solution, Error> {
    instance.validate()?;
    // Merge identical item types: same size, turn and profit.
    let mut groups: Vec<(Item, Vec<usize>)> = Vec::new();
    for (k, it) in instance.items.iter().enumerate() {
        if it.copies == 0 {
            continue;
        }
        let profit = Some(instance.profit(k));
        match groups.iter_mut().find(|(g, _)| {
            g.rotate == it.rotate
                && g.profit == profit
                && ((g.w, g.h) == (it.w, it.h) || (it.rotate && (g.w, g.h) == (it.h, it.w)))
        }) {
            Some((g, members)) => {
                g.copies += it.copies;
                members.push(k);
            }
            None => groups.push((
                Item {
                    profit,
                    ..it.clone()
                },
                vec![k],
            )),
        }
    }
    if groups.iter().any(|(g, _)| g.copies > u16::MAX as u32) {
        return Err(Error::TooMany);
    }
    let merged = Instance {
        items: groups.iter().map(|(g, _)| g.clone()).collect(),
        ..instance.clone()
    };
    let mut solution = guillotine::solve(&merged, options);
    // Back to the original item types, copies in order.
    let mut next: Vec<(usize, u32)> = vec![(0, 0); groups.len()];
    for p in &mut solution.placements {
        let g = p.item;
        let (members, slot) = (&groups[g].1, &mut next[g]);
        while instance.items[members[slot.0]].copies == slot.1 {
            *slot = (slot.0 + 1, 0);
        }
        let original = members[slot.0];
        slot.1 += 1;
        let it = &instance.items[original];
        p.rotated = (p.w, p.h) != (it.w, it.h);
        p.item = original;
    }
    check(instance, &solution).map_err(Error::Bug)?;
    strokes::rearrange(instance, &mut solution);
    Ok(solution)
}

#[cfg(test)]
mod tests;
