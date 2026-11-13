//! The irregular branching scheme, after PackingSolver's `irregular::BranchingScheme` (Florian
//! Fontan, MIT), for rectangles.
//!
//! A node is a partial layout, one mount added per level. Its children are its insertions: each
//! mount type with copies left, each way round, resting on a support and slid right until it
//! overlaps nothing (PackingSolver's `TrapezoidSetSlider`). The supports are the top of the mount
//! placed last; and, in a bin just opened, the bin's own floor (the outline's upward-facing
//! edges and the holes' tops). The insertions of a node's parent are kept, and slid on only if
//! the new mount is in their way, so a mount can rest against any mount placed before, not only
//! the last. Only when nothing fits in the current bin do the insertions open the next bin.

use std::cell::Cell;
use std::cmp::Ordering;
use std::collections::HashSet;
use std::rc::Rc;

use super::geometry::{self, Path, Planes, Pos};
use super::{Instance, Point};
use crate::beam::{self, Widths};
use crate::{Area, Len, Objective, Options, Placement, Solution};

/// A bin in the scheme's frame.
struct Bin {
    /// Its index in the instance's bin sequence.
    index: usize,
    rings: Vec<Vec<Point>>,
    bbox: [Len; 4],
    area: Area,
    /// The area of the bins before it.
    area_before: Area,
    /// For each orientation ([`orientation`]): where its corner can't go, one convex polygon per
    /// obstacle round the bin's region.
    blocked: Vec<Vec<Planes>>,
    /// What the mounts can rest on in an empty bin.
    supports: Vec<Vec<[f64; 2]>>,
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
        if rotated {
            (self.h, self.w)
        } else {
            (self.w, self.h)
        }
    }

    /// The ways round to try: a square, or a mount that may not turn, only one.
    fn turns(&self) -> &'static [bool] {
        if self.rotate && self.w != self.h {
            &[false, true]
        } else {
            &[false]
        }
    }
}

/// An orientation's index, for [`Bin::blocked`].
fn orientation(item: usize, rotated: bool) -> usize {
    2 * item + usize::from(rotated)
}

/// Which partial layouts the beam keeps (PackingSolver's irregular guides 0, 1, 4 and 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Guide {
    /// Area used per item area.
    AreaRatio,
    /// Area used per item area, per item area again: favours the big mounts.
    BigItems,
    /// Area used per profit.
    AreaPerProfit,
    /// Area used per profit, per item area, per item.
    AreaPerProfitPerItem,
}

/// A place a mount can go: where it rests, how far along it has slid, and its corner on the
/// grid.
#[derive(Debug, Clone)]
struct Insertion {
    item: u16,
    rotated: bool,
    /// The scheme's bin.
    bin: u16,
    path: Rc<Path>,
    pos: Pos,
    x: Len,
    y: Len,
}

#[derive(Debug, Clone, Copy)]
struct Placed {
    item: u16,
    rotated: bool,
    bin: u16,
    x: Len,
    y: Len,
    w: Len,
    h: Len,
}

impl Placed {
    fn rect(&self) -> [Len; 4] {
        [self.x, self.y, self.w, self.h]
    }
}

/// A partial layout.
#[derive(Debug)]
pub(crate) struct Node {
    /// Creation order: breaks ties between equal guides, so the search is deterministic.
    id: u64,
    parent: Option<Rc<Node>>,
    /// The mount placed last; none at the root.
    item: Option<Placed>,
    /// The layout is in the transposed frame (x ↔ y).
    transposed: bool,
    /// Bins used: the current bin's index, plus one.
    bins: u16,
    /// The insertions found for this node's parent, its own among them.
    siblings: Rc<Vec<Insertion>>,
    /// Copies placed, per item type.
    placed: Box<[u16]>,
    items: u32,
    item_area: Area,
    profit: Area,
    /// The item area in the current bin.
    bin_item_area: Area,
    /// The area used up: the bins before the current one, and the current one's
    /// ([`geometry::used_area`]).
    used_area: f64,
}

type N = Rc<Node>;

struct Scheme<'a> {
    bins: &'a [Bin],
    items: &'a [Item],
    transposed: bool,
    objective: Objective,
    guide: Guide,
    total_items: u32,
    total_profit: Area,
    total_bin_area: Area,
    /// The highest profit per area of any item type (for the knapsack bound).
    efficiency: f64,
    /// The fewest bins the items' area needs (for bin packing).
    min_bins: u16,
    next_id: Cell<u64>,
}

impl Scheme<'_> {
    /// The mounts in the current bin.
    fn mounts(&self, n: &Node) -> Vec<[Len; 4]> {
        let mut out = Vec::new();
        let mut node = Some(n);
        while let Some(m) = node {
            match m.item {
                Some(p) if p.bin + 1 == n.bins => out.push(p.rect()),
                _ => break,
            }
            node = m.parent.as_deref();
        }
        out
    }

    fn guide(&self, n: &Node) -> f64 {
        let (used, area, items, profit) = (
            n.used_area,
            n.item_area as f64,
            n.items as f64,
            n.profit as f64,
        );
        match self.guide {
            Guide::AreaRatio => used / area,
            Guide::BigItems => used / area / area,
            Guide::AreaPerProfit => used / profit,
            Guide::AreaPerProfitPerItem => used / profit / area * items,
        }
    }

    /// The most profit any completion of `n` could reach: no more than the room left in the
    /// current bin and the bins after it, at the best profit per area.
    fn profit_bound(&self, n: &Node) -> Area {
        let room = match n.item {
            Some(p) => {
                self.total_bin_area - self.bins[p.bin as usize].area_before - n.bin_item_area
            }
            None => self.total_bin_area,
        };
        self.total_profit
            .min(n.profit + (room as f64 * self.efficiency) as Area)
    }

    /// Slide a mount of orientation `o` from `pos` on, then put its corner on the grid: the
    /// nearest whole position where it fits exactly.
    fn settle(
        &self,
        bin: &Bin,
        (item, rotated): (usize, bool),
        path: &Path,
        pos: &mut Pos,
        mounts: &[[Len; 4]],
    ) -> Option<(Len, Len)> {
        let (w, h) = self.items[item].size(rotated);
        let blocked = &bin.blocked[orientation(item, rotated)];
        let x_limit = (bin.bbox[2] - w) as f64;
        if !geometry::slide(path, pos, blocked, mounts, (w, h), x_limit) {
            return None;
        }
        let [x, y] = pos.at(path);
        let grid = |v: f64| [v.round(), (v - 1e-6).ceil(), (v + 1e-6).floor()];
        let mut tried = Vec::with_capacity(9);
        for gx in grid(x) {
            for gy in grid(y) {
                let (gx, gy) = (gx as Len, gy as Len);
                if tried.contains(&(gx, gy)) {
                    continue;
                }
                tried.push((gx, gy));
                let r = [gx, gy, w, h];
                if geometry::contains(&bin.rings, r)
                    && mounts.iter().all(|&m| !geometry::overlap(m, r))
                {
                    return Some((gx, gy));
                }
            }
        }
        None
    }

    /// Every item type with copies left, each way round, resting on `chain` in bin `b`.
    fn insert_all(
        &self,
        out: &mut Vec<Insertion>,
        b: usize,
        chain: &[[f64; 2]],
        mounts: &[[Len; 4]],
        placed: &[u16],
    ) {
        let bin = &self.bins[b];
        for (t, it) in self.items.iter().enumerate() {
            if placed[t] >= it.copies {
                continue;
            }
            for &rotated in it.turns() {
                let (w, h) = it.size(rotated);
                if w > bin.bbox[2] - bin.bbox[0] || h > bin.bbox[3] - bin.bbox[1] {
                    continue;
                }
                let path = Rc::new(Path::new(chain, w as f64));
                let mut pos = Pos::start(&path);
                if let Some((x, y)) = self.settle(bin, (t, rotated), &path, &mut pos, mounts) {
                    out.push(Insertion {
                        item: t as u16,
                        rotated,
                        bin: b as u16,
                        path,
                        pos,
                        x,
                        y,
                    });
                }
            }
        }
    }

    fn child(&self, parent: &N, s: &Insertion, siblings: &Rc<Vec<Insertion>>) -> N {
        let it = &self.items[s.item as usize];
        let (w, h) = it.size(s.rotated);
        let placed_item = Placed {
            item: s.item,
            rotated: s.rotated,
            bin: s.bin,
            x: s.x,
            y: s.y,
            w,
            h,
        };
        let same_bin = parent.item.is_some() && parent.bins == s.bin + 1;
        let mut mounts = if same_bin {
            self.mounts(parent)
        } else {
            Vec::new()
        };
        mounts.push(placed_item.rect());
        let bin = &self.bins[s.bin as usize];
        let mut placed = parent.placed.clone();
        placed[s.item as usize] += 1;
        let id = self.next_id.get();
        self.next_id.set(id + 1);
        Rc::new(Node {
            id,
            parent: Some(Rc::clone(parent)),
            item: Some(placed_item),
            transposed: self.transposed,
            bins: s.bin + 1,
            siblings: Rc::clone(siblings),
            placed,
            items: parent.items + 1,
            item_area: parent.item_area + it.area,
            profit: parent.profit + it.profit,
            bin_item_area: if same_bin { parent.bin_item_area } else { 0 } + it.area,
            used_area: bin.area_before as f64 + geometry::used_area(bin.bbox, &mounts),
        })
    }
}

impl beam::Scheme for Scheme<'_> {
    type Node = N;
    type Key = u64;

    fn root(&self) -> N {
        Rc::new(Node {
            id: 0,
            parent: None,
            item: None,
            transposed: self.transposed,
            bins: 0,
            siblings: Rc::new(Vec::new()),
            placed: vec![0; self.items.len()].into_boxed_slice(),
            items: 0,
            item_area: 0,
            profit: 0,
            bin_item_area: 0,
            used_area: 0.0,
        })
    }

    fn children(&self, parent: &N, out: &mut Vec<N>) {
        let mut found: Vec<Insertion> = Vec::new();
        if let Some(last) = parent.item {
            let b = last.bin as usize;
            let bin = &self.bins[b];
            let mounts = self.mounts(parent);
            // The parent's insertions: still where they were unless the new mount is in the way.
            for s in parent.siblings.iter() {
                let it = &self.items[s.item as usize];
                if parent.placed[s.item as usize] >= it.copies {
                    continue;
                }
                let (w, h) = it.size(s.rotated);
                if !geometry::overlap(last.rect(), [s.x, s.y, w, h]) {
                    found.push(s.clone());
                    continue;
                }
                let mut pos = s.pos;
                let turn = (s.item as usize, s.rotated);
                if let Some((x, y)) = self.settle(bin, turn, &s.path, &mut pos, &mounts) {
                    found.push(Insertion {
                        pos,
                        x,
                        y,
                        ..s.clone()
                    });
                }
            }
            // New ones on top of it.
            let top = (last.y + last.h) as f64;
            let chain = [[last.x as f64, top], [(last.x + last.w) as f64, top]];
            self.insert_all(&mut found, b, &chain, &mounts, &parent.placed);
        }
        if found.is_empty() {
            // The next bin something fits in (a small drop-out may fit nothing).
            for b in parent.bins as usize..self.bins.len() {
                for chain in &self.bins[b].supports {
                    self.insert_all(&mut found, b, chain, &[], &parent.placed);
                }
                if !found.is_empty() {
                    break;
                }
            }
        }
        // Different supports often end in the same place.
        let mut seen = HashSet::new();
        found.retain(|s| seen.insert((s.item, s.rotated, s.bin, s.x, s.y)));
        let siblings = Rc::new(found);
        for s in siblings.iter() {
            out.push(self.child(parent, s, &siblings));
        }
    }

    fn is_leaf(&self, n: &N) -> bool {
        n.items == self.total_items
    }

    fn better(&self, n: &N, best: Option<&N>) -> bool {
        match self.objective {
            // Fewer bins; then, as many, less of the last one used up.
            Objective::BinPacking => {
                self.is_leaf(n)
                    && best.is_none_or(|b| {
                        !self.is_leaf(b)
                            || n.bins < b.bins
                            || (n.bins == b.bins && n.used_area < b.used_area)
                    })
            }
            Objective::Knapsack => best.is_none_or(|b| n.profit > b.profit),
        }
    }

    fn bound(&self, n: &N, best: &N) -> bool {
        match self.objective {
            Objective::BinPacking => self.is_leaf(best) && n.bins > best.bins,
            Objective::Knapsack => self.profit_bound(n) <= best.profit,
        }
    }

    fn order(&self, a: &N, b: &N) -> Ordering {
        self.guide(a)
            .total_cmp(&self.guide(b))
            .then(a.id.cmp(&b.id))
    }

    /// No dominance: without PackingSolver's skyline there's no cheap test that one partial
    /// layout leaves at least as much room as another.
    fn key(&self, n: &N) -> u64 {
        n.id
    }

    fn dominates(&self, _: &N, _: &N) -> bool {
        false
    }

    fn is_optimal(&self, best: &N) -> bool {
        match self.objective {
            Objective::BinPacking => self.is_leaf(best) && best.bins <= self.min_bins,
            Objective::Knapsack => best.profit >= self.total_profit,
        }
    }
}

/// The bins and item types in a frame: as given, or transposed (x ↔ y).
fn frame(instance: &Instance, transposed: bool) -> (Vec<Bin>, Vec<Item>) {
    let items: Vec<Item> = instance
        .items
        .iter()
        .enumerate()
        .map(|(k, it)| {
            let (w, h) = if transposed {
                (it.h, it.w)
            } else {
                (it.w, it.h)
            };
            Item {
                w,
                h,
                copies: it.copies as u16,
                area: it.w as Area * it.h as Area,
                profit: instance.profit(k),
                rotate: it.rotate,
            }
        })
        .collect();
    let mut bins: Vec<Bin> = instance
        .bin_sequence()
        .iter()
        .enumerate()
        .map(|(index, b)| {
            let region = if transposed {
                b.region.transposed()
            } else {
                b.region.clone()
            };
            let rings = region.rings();
            let bbox = region.bbox();
            let margin = 2.0 * ((bbox[2] - bbox[0]) + (bbox[3] - bbox[1])) as f64 + 10.0;
            let obstacles = geometry::obstacles(&rings, bbox, margin);
            let mut blocked = Vec::with_capacity(2 * items.len());
            for it in &items {
                for rotated in [false, true] {
                    let (w, h) = it.size(rotated);
                    blocked.push(
                        obstacles
                            .iter()
                            .map(|o| geometry::blocked(o, w as f64, h as f64))
                            .collect(),
                    );
                }
            }
            Bin {
                index,
                supports: geometry::supports(&rings),
                rings,
                bbox,
                area: region.area(),
                area_before: 0,
                blocked,
            }
        })
        .collect();
    // Knapsack: the bins by increasing area, whatever their order (as PackingSolver does), so
    // the small drop-outs are filled first.
    if instance.objective == Objective::Knapsack {
        bins.sort_by_key(|b| b.area);
    }
    let mut area_before = 0;
    for bin in &mut bins {
        bin.area_before = area_before;
        area_before += bin.area;
    }
    (bins, items)
}

/// Lay out the items (the instance is valid).
pub(crate) fn solve(instance: &Instance, options: &Options) -> Solution {
    let guides = match instance.objective {
        Objective::BinPacking => [Guide::AreaRatio, Guide::BigItems],
        Objective::Knapsack => [Guide::AreaPerProfit, Guide::AreaPerProfitPerItem],
    };
    let widths = Widths {
        max_width: options.max_width.max(1),
        growth: options.growth.max(1.01),
    };
    let mut best: Option<N> = None;
    let mut frames = Vec::new();
    for transposed in [false, true] {
        let (bins, items) = frame(instance, transposed);
        let total_item_area: Area = items.iter().map(|i| i.area * i.copies as Area).sum();
        let mut min_bins = 0u16;
        let mut room = 0;
        for b in &bins {
            if room >= total_item_area {
                break;
            }
            room += b.area;
            min_bins += 1;
        }
        let mut optimal = false;
        for guide in guides {
            let scheme = Scheme {
                bins: &bins,
                items: &items,
                transposed,
                objective: instance.objective,
                guide,
                total_items: items.iter().map(|i| i.copies as u32).sum(),
                total_profit: items.iter().map(|i| i.profit * i.copies as Area).sum(),
                total_bin_area: bins.iter().map(|b| b.area).sum(),
                efficiency: items
                    .iter()
                    .map(|i| i.profit as f64 / i.area as f64)
                    .fold(0.0, f64::max),
                min_bins,
                next_id: Cell::new(1),
            };
            if beam::search(&scheme, widths, &mut best) {
                // The whole tree was searched (whatever the guide, it's the same tree), or the
                // best can't be beaten.
                optimal = best
                    .as_ref()
                    .is_some_and(|b| beam::Scheme::is_optimal(&scheme, b));
                break;
            }
        }
        frames.push(bins);
        if optimal {
            break;
        }
    }
    // Back to the real frame: the placed mounts along the chain from the best node.
    let mut solution = Solution::default();
    let mut node = best.as_ref();
    while let Some(n) = node {
        if let Some(p) = n.item {
            let bins = &frames[usize::from(n.transposed)];
            let (x, y, w, h) = if n.transposed {
                (p.y, p.x, p.h, p.w)
            } else {
                (p.x, p.y, p.w, p.h)
            };
            solution.placements.push(Placement {
                item: p.item as usize,
                bin: bins[p.bin as usize].index,
                x,
                y,
                w,
                h,
                rotated: p.rotated,
            });
            solution.profit += instance.profit(p.item as usize);
        }
        node = n.parent.as_ref();
    }
    solution.placements.reverse();
    solution.bins_used = solution
        .placements
        .iter()
        .map(|p| p.bin + 1)
        .max()
        .unwrap_or(0);
    solution
}
