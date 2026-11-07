//! The guillotine branching scheme, after PackingSolver's `rectangleguillotine::BranchingScheme`
//! (Florian Fontan, MIT), for three stages, non-exact (an item may be shorter than its row),
//! zero kerf and hard trims.
//!
//! A node is a partial layout, built one item at a time. In the scheme's own frame the first
//! stage is vertical: a bin is cut into strips (1-cuts, along x), a strip into rows (2-cuts,
//! along y), a row into items side by side (3-cuts, along x). The next item goes in the
//! current row, in a new row of the current strip, in a new strip, or in a new bin. [`solve`]
//! turns the instance (x ↔ y) so that the real first stage is horizontal, and turns the layout
//! back.

use std::cell::Cell;
use std::cmp::Ordering;
use std::rc::Rc;

use crate::beam::{self, Widths};
use crate::{Area, Instance, Len, Objective, Options, Placement, Solution};

/// A bin in the scheme's frame: its usable area, the usable area of the bins before it, and
/// its index in the instance's bin sequence.
#[derive(Debug, Clone, Copy)]
struct Bin {
    index: usize,
    x0: Len,
    x_end: Len,
    y0: Len,
    y_end: Len,
    area_before: Area,
}

impl Bin {
    fn area(&self) -> Area {
        (self.x_end - self.x0) as Area * (self.y_end - self.y0) as Area
    }
}

/// An item type in the scheme's frame.
#[derive(Debug, Clone, Copy)]
struct Item {
    w: Len,
    h: Len,
    copies: u16,
    area: Area,
    profit: Area,
    rotate: bool,
}

impl Item {
    fn size(&self, rotated: bool) -> (Len, Len) {
        if rotated { (self.h, self.w) } else { (self.w, self.h) }
    }
}

/// Where the next item goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    SameRow,
    NewRow,
    NewStrip,
    NewBin,
}

/// Which partial layouts the beam keeps (PackingSolver's guides 0, 1, 4 and 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Guide {
    /// Area used per item area.
    AreaRatio,
    /// Waste rate per item area, per item.
    WasteRate,
    /// Area used per profit.
    AreaPerProfit,
    /// Area used per profit, per item area, per item.
    AreaPerProfitPerItem,
}

/// The cuts still open in the last bin: its current strip spans x1_prev..x1_curr, the
/// strip's current row y2_prev..y2_curr, and the row is filled up to x3_curr.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Front {
    bin: u16,
    x1_prev: Len,
    x1_curr: Len,
    y2_prev: Len,
    y2_curr: Len,
    x3_curr: Len,
}

/// `f1` leaves at least as much room as `f2` (PackingSolver's front dominance, for hard trims).
fn dominates(f1: &Front, f2: &Front) -> bool {
    if f1.bin != f2.bin {
        return f1.bin < f2.bin;
    }
    f1.x1_curr <= f2.x1_prev
        || (f1.x1_prev <= f2.x1_prev && f1.x1_curr <= f2.x1_curr && f1.y2_curr <= f2.y2_prev)
        || (f1.x1_prev <= f2.x1_prev
            && f1.x3_curr <= f2.x3_curr
            && f1.x1_curr <= f2.x1_curr
            && f1.y2_prev <= f2.y2_prev
            && f1.y2_curr <= f2.y2_curr)
}

/// A partial layout.
#[derive(Debug)]
pub(crate) struct Node {
    /// Creation order: breaks ties between equal guides, so the search is deterministic.
    id: u64,
    parent: Option<Rc<Node>>,
    /// The item placed last: type, turned, and its corner (left, bottom); none at the root.
    item: Option<Placed>,
    /// Bins used.
    bins: u16,
    front: Front,
    /// Copies placed, per item type.
    placed: Box<[u16]>,
    items: u32,
    item_area: Area,
    profit: Area,
    /// The area used up so far: everything left of the current strip, and the parts of the
    /// strip below the current row and left of the row's end.
    current_area: Area,
}

#[derive(Debug, Clone, Copy)]
struct Placed {
    item: u16,
    rotated: bool,
    x: Len,
    y: Len,
}

type N = Rc<Node>;

struct Scheme {
    bins: Vec<Bin>,
    items: Vec<Item>,
    objective: Objective,
    guide: Guide,
    total_items: u32,
    total_item_area: Area,
    total_profit: Area,
    total_bin_area: Area,
    /// The highest profit per area of any item type (for the knapsack bound).
    efficiency: f64,
    /// The fewest bins all the items could fit in, by area, and the least area they could
    /// use there: the bins before the last, and the last up to the first full-width line
    /// past the items' area.
    min_bins: usize,
    min_area: Area,
    next_id: Cell<u64>,
}

impl Scheme {
    fn guide(&self, n: &Node) -> f64 {
        let (used, area, items, profit) = (n.current_area as f64, n.item_area as f64, n.items as f64, n.profit as f64);
        match self.guide {
            Guide::AreaRatio => used / area,
            Guide::WasteRate => {
                let mut rate = (used - area) / used;
                if rate < 0.02 {
                    rate = 0.01 + rate / 2.0;
                }
                rate / area * items
            }
            Guide::AreaPerProfit => used / profit,
            Guide::AreaPerProfitPerItem => used / profit / area * items,
        }
    }

    /// The most profit any completion of `n` could reach.
    fn profit_bound(&self, n: &Node) -> Area {
        let items_left = self.total_item_area - n.item_area;
        let room = self.total_bin_area - n.current_area;
        if room >= items_left {
            self.total_profit
        } else {
            n.profit + (room as f64 * self.efficiency) as Area
        }
    }

    /// Put item `t` (turned or not) at `level` in bin `b`, unless a sibling with the same
    /// item leaves more room.
    fn insert(&self, out: &mut Vec<N>, parent: &N, t: usize, rotated: bool, level: Level, b: usize) {
        let (w, h) = self.items[t].size(rotated);
        let p = &parent.front;
        let bin = &self.bins[b];
        let (x1_prev, x, y) = match level {
            Level::SameRow => (p.x1_prev, p.x3_curr, p.y2_prev),
            Level::NewRow => (p.x1_prev, p.x1_prev, p.y2_curr),
            Level::NewStrip => (p.x1_curr, p.x1_curr, bin.y0),
            Level::NewBin => (bin.x0, bin.x0, bin.y0),
        };
        let (x3, top) = (x + w, y + h);
        if x3 > bin.x_end || top > bin.y_end {
            return;
        }
        let in_strip = matches!(level, Level::SameRow | Level::NewRow);
        let front = Front {
            bin: b as u16,
            x1_prev,
            x1_curr: if in_strip { x3.max(p.x1_curr) } else { x3 },
            y2_prev: y,
            y2_curr: if level == Level::SameRow { top.max(p.y2_curr) } else { top },
            x3_curr: x3,
        };
        let same_item = |c: &N| c.item.is_some_and(|i| i.item as usize == t);
        if out.iter().any(|c| same_item(c) && dominates(&c.front, &front)) {
            return;
        }
        out.retain(|c| !(same_item(c) && dominates(&front, &c.front)));
        out.push(self.child(parent, Placed { item: t as u16, rotated, x, y }, front));
    }

    /// Every item type with copies left, each way round, at `level` in bin `b`.
    fn insert_all(&self, out: &mut Vec<N>, parent: &N, level: Level, b: usize) {
        for (t, it) in self.items.iter().enumerate() {
            if parent.placed[t] >= it.copies {
                continue;
            }
            if it.rotate && it.w != it.h {
                let wide = it.w > it.h;
                self.insert(out, parent, t, !wide, level, b);
                self.insert(out, parent, t, wide, level, b);
            } else {
                self.insert(out, parent, t, false, level, b);
            }
        }
    }

    fn child(&self, parent: &N, placed: Placed, front: Front) -> N {
        let it = &self.items[placed.item as usize];
        let mut counts = parent.placed.clone();
        counts[placed.item as usize] += 1;
        let items = parent.items + 1;
        let bin = &self.bins[front.bin as usize];
        let height = (bin.y_end - bin.y0) as Area;
        let current_area = bin.area_before
            + if items == self.total_items {
                (front.x1_curr - bin.x0) as Area * height
            } else {
                (front.x1_prev - bin.x0) as Area * height
                    + (front.x1_curr - front.x1_prev) as Area * (front.y2_prev - bin.y0) as Area
                    + (front.x3_curr - front.x1_prev) as Area * (front.y2_curr - front.y2_prev) as Area
            };
        let id = self.next_id.get();
        self.next_id.set(id + 1);
        Rc::new(Node {
            id,
            parent: Some(Rc::clone(parent)),
            item: Some(placed),
            bins: front.bin + 1,
            front,
            placed: counts,
            items,
            item_area: parent.item_area + it.area,
            profit: parent.profit + it.profit,
            current_area,
        })
    }
}

impl beam::Scheme for Scheme {
    type Node = N;
    type Key = Box<[u16]>;

    fn root(&self) -> N {
        Rc::new(Node {
            id: 0,
            parent: None,
            item: None,
            bins: 0,
            front: Front::default(),
            placed: vec![0; self.items.len()].into_boxed_slice(),
            items: 0,
            item_area: 0,
            profit: 0,
            current_area: 0,
        })
    }

    fn children(&self, parent: &N, out: &mut Vec<N>) {
        let levels: &[Level] = if parent.item.is_none() {
            &[Level::NewBin]
        } else {
            &[Level::SameRow, Level::NewRow, Level::NewStrip, Level::NewBin]
        };
        let p = &parent.front;
        for &level in levels {
            // Open a new row, strip or bin only if nothing fits without growing the current one.
            let grows = out.iter().any(|c| match level {
                Level::SameRow => false,
                Level::NewRow => c.front.x1_curr == p.x1_curr && c.front.y2_curr == p.y2_curr,
                Level::NewStrip => c.front.x1_curr == p.x1_curr,
                Level::NewBin => true,
            });
            if grows {
                break;
            }
            if level == Level::NewBin {
                // The next bin something fits in (a small drop-out may fit nothing).
                for b in parent.bins as usize..self.bins.len() {
                    self.insert_all(out, parent, level, b);
                    if !out.is_empty() {
                        break;
                    }
                }
            } else {
                self.insert_all(out, parent, level, parent.bins as usize - 1);
            }
        }
        if parent.item.is_some() {
            let same_bin = |c: &N| c.bins == parent.bins;
            let has_bin = out.iter().any(same_bin);
            let has_strip = out.iter().any(|c| same_bin(c) && c.front.x1_curr == p.x1_curr);
            let has_row = out.iter().any(|c| same_bin(c) && c.front.x1_curr == p.x1_curr && c.front.y2_curr == p.y2_curr);
            out.retain(|c| {
                let new_bin = c.bins > parent.bins;
                let new_strip = new_bin || c.front.x1_prev >= p.x1_curr;
                let new_row = new_strip || c.front.y2_prev >= p.y2_curr;
                !((has_row && new_row) || (has_strip && new_strip) || (has_bin && new_bin))
            });
        }
    }

    fn is_leaf(&self, n: &N) -> bool {
        n.items == self.total_items
    }

    fn better(&self, n: &N, best: Option<&N>) -> bool {
        match self.objective {
            // Fewer bins; then, as many, less of the last one used: bins are used in order, so
            // with offcuts first this uses them up and leaves the biggest piece of the last
            // sheet (a full-width band at its end).
            Objective::BinPacking => {
                self.is_leaf(n)
                    && best.is_none_or(|b| !self.is_leaf(b) || (n.bins, n.current_area) < (b.bins, b.current_area))
            }
            Objective::Knapsack => best.is_none_or(|b| n.profit > b.profit),
        }
    }

    fn bound(&self, n: &N, best: &N) -> bool {
        match self.objective {
            Objective::BinPacking => {
                if !self.is_leaf(best) {
                    return false;
                }
                // Any completion uses at least all the item area plus the waste so far (waste
                // never shrinks as items are added), so at least this many bins; and with as many
                // as `best`, no less area.
                let used = self.total_item_area + (n.current_area - n.item_area);
                let mut need = used;
                let mut bins = 0;
                while need > 0 {
                    let Some(bin) = self.bins.get(bins) else { return true };
                    need -= bin.area();
                    bins += 1;
                }
                bins > best.bins as usize || (bins == best.bins as usize && used >= best.current_area)
            }
            Objective::Knapsack => self.profit_bound(n) <= best.profit,
        }
    }

    fn order(&self, a: &N, b: &N) -> Ordering {
        self.guide(a).total_cmp(&self.guide(b)).then(a.id.cmp(&b.id))
    }

    fn key(&self, n: &N) -> Box<[u16]> {
        n.placed.clone()
    }

    fn dominates(&self, a: &N, b: &N) -> bool {
        dominates(&a.front, &b.front)
    }

    fn is_optimal(&self, best: &N) -> bool {
        match self.objective {
            Objective::BinPacking => {
                self.is_leaf(best) && (best.bins as usize, best.current_area) <= (self.min_bins, self.min_area)
            }
            Objective::Knapsack => best.profit >= self.total_profit,
        }
    }
}

/// Lay out the items (the instance is valid, and its item types distinct).
pub(crate) fn solve(instance: &Instance, options: &Options) -> Solution {
    // The scheme's frame is the real one turned: x ↔ y, so its vertical first stage is our
    // horizontal one. A real bin's top trim is the scheme's left, its left trim the scheme's
    // bottom.
    let mut bins: Vec<Bin> = instance
        .bin_sequence()
        .iter()
        .enumerate()
        .map(|(index, b)| Bin { index, x0: b.trims.top, x_end: b.h - b.trims.bottom, y0: b.trims.left, y_end: b.w - b.trims.right, area_before: 0 })
        .collect();
    // Knapsack: the bins by increasing area, whatever their order (as PackingSolver does), so
    // the small drop-outs are filled first.
    if instance.objective == Objective::Knapsack {
        bins.sort_by_key(Bin::area);
    }
    let mut area_before = 0;
    for bin in &mut bins {
        bin.area_before = area_before;
        area_before += bin.area();
    }
    let items: Vec<Item> = instance
        .items
        .iter()
        .enumerate()
        .map(|(k, it)| Item {
            w: it.h,
            h: it.w,
            copies: it.copies as u16,
            area: it.w as Area * it.h as Area,
            profit: instance.profit(k),
            rotate: it.rotate,
        })
        .collect();
    let total_item_area: Area = items.iter().map(|i| i.area * i.copies as Area).sum();
    let mut min_bins = 0;
    let mut covered = 0;
    while covered < total_item_area && min_bins < bins.len() {
        covered += bins[min_bins].area();
        min_bins += 1;
    }
    // In the last bin, the items left over need whole lines across it.
    let min_area = match min_bins.checked_sub(1).map(|k| bins[k]) {
        Some(last) => {
            let height = (last.y_end - last.y0) as Area;
            let left = total_item_area - last.area_before;
            last.area_before + (left + height - 1) / height * height
        }
        None => 0,
    };
    let guides = match instance.objective {
        Objective::BinPacking => [Guide::AreaRatio, Guide::WasteRate],
        Objective::Knapsack => [Guide::AreaPerProfit, Guide::AreaPerProfitPerItem],
    };
    let widths = Widths { max_width: options.max_width.max(1), growth: options.growth.max(1.01) };
    let mut best: Option<N> = None;
    for guide in guides {
        let scheme = Scheme {
            total_items: items.iter().map(|i| i.copies as u32).sum(),
            total_item_area,
            total_profit: items.iter().map(|i| i.profit * i.copies as Area).sum(),
            total_bin_area: area_before,
            efficiency: items.iter().map(|i| i.profit as f64 / i.area as f64).fold(0.0, f64::max),
            min_bins,
            min_area,
            next_id: Cell::new(1),
            bins: bins.clone(),
            items: items.clone(),
            objective: instance.objective,
            guide,
        };
        if beam::search(&scheme, widths, &mut best) {
            break;
        }
    }
    // Back to the real frame: the placed items along the chain from the best node.
    let mut solution = Solution::default();
    let mut node = best.as_ref();
    while let Some(n) = node {
        if let Some(p) = n.item {
            let it = &items[p.item as usize];
            let (w, h) = it.size(p.rotated);
            solution.placements.push(Placement {
                item: p.item as usize,
                bin: bins[n.front.bin as usize].index,
                x: p.y,
                y: p.x,
                w: h,
                h: w,
                rotated: p.rotated,
            });
            solution.profit += it.profit;
        }
        node = n.parent.as_ref();
    }
    solution.placements.reverse();
    solution.bins_used = solution.placements.iter().map(|p| p.bin + 1).max().unwrap_or(0);
    solution
}
