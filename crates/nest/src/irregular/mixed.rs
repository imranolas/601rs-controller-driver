//! Guillotine layouts where the cuts can run straight, the irregular nester round the rest.
//!
//! A shaped bin's straight parts (the rectangle under an arch, the board either side of a kept
//! piece) are filled by the guillotine nester ([`crate::solve`]), so their mounts share cuts in
//! three stages and have their strokes rearranged. The irregular nester then fills what's left,
//! round the curves and in the gaps, with the guillotine mounts as holes.
//!
//! The straight parts are chosen greedily, a block at a time: of the largest rectangles inside
//! what's left ([`geometry::rectangles`]), the one whose guillotine layout leads to the best
//! whole layout, with the irregular nester filling round it (both searched quickly); most
//! profit, then fewest strokes. A block must hold two mounts or more (one alone shares no
//! cuts), and is added only if it does at least as well as stopping. The result is kept only if
//! it places as much as the irregular nester alone; as much, with fewer strokes.

use super::{Bin, Instance, Region, geometry, scheme};
use crate::{Area, Item, Len, Objective, Options, Placement, Trims, strokes};

/// How many of the largest rectangles to try at each step.
const CANDIDATES: usize = 12;

/// How hard to search when only comparing rectangles.
const QUICK: Options = Options {
    max_width: 16,
    growth: 4.0,
};

fn profit(items: &[Item], placements: &[Placement]) -> Area {
    placements
        .iter()
        .map(|p| {
            let it = &items[p.item];
            it.profit.unwrap_or(it.w as Area * it.h as Area)
        })
        .sum()
}

fn stroke_count(placements: &[Placement]) -> usize {
    strokes::strokes(placements.iter().map(|p| (0, p.x, p.y, p.w, p.h))).0
}

/// The items with `left` copies each.
fn remaining(items: &[Item], left: &[u32]) -> Vec<Item> {
    items
        .iter()
        .zip(left)
        .map(|(it, &copies)| Item {
            copies,
            ..it.clone()
        })
        .collect()
}

/// A guillotine layout (knapsack) of what's left in the rectangle `r`, placed there.
fn guillotine(r: [Len; 4], items: &[Item], options: &Options) -> Vec<Placement> {
    let instance = crate::Instance {
        objective: Objective::Knapsack,
        bins: vec![crate::Bin {
            w: r[2],
            h: r[3],
            trims: Trims::default(),
            copies: 1,
        }],
        items: items.to_vec(),
    };
    let Ok(s) = crate::solve(&instance, options) else {
        return Vec::new();
    };
    s.placements
        .into_iter()
        .map(|p| Placement {
            x: p.x + r[0],
            y: p.y + r[1],
            ..p
        })
        .collect()
}

/// The irregular nester alone in one region (knapsack).
fn free(region: &Region, items: &[Item], options: &Options) -> Vec<Placement> {
    let instance = Instance {
        objective: Objective::Knapsack,
        bins: vec![Bin {
            region: region.clone(),
            copies: 1,
        }],
        items: items.to_vec(),
    };
    scheme::solve(&instance, options).placements
}

/// What's left once `block` is placed: fewer copies, and its mounts as holes in the region.
fn without(rest: &Region, left: &[u32], block: &[Placement]) -> (Region, Vec<u32>) {
    let (mut rest, mut left) = (rest.clone(), left.to_vec());
    for p in block {
        left[p.item] -= 1;
        let (x, y) = (p.x + p.w, p.y + p.h);
        rest.holes
            .push(vec![[p.x, p.y], [x, p.y], [x, y], [p.x, y]]);
    }
    (rest, left)
}

/// Fill one region with as much of the items as fits (knapsack): placements in bin 0.
pub(crate) fn fill(region: &Region, items: &[Item], options: &Options) -> Vec<Placement> {
    // Most profit, then fewest strokes.
    let score = |p: &[Placement]| (profit(items, p), std::cmp::Reverse(stroke_count(p)));
    let alone = free(region, items, options);
    let mut left: Vec<u32> = items.iter().map(|i| i.copies).collect();
    let mut rest = region.clone();
    let mut blocks: Vec<Placement> = Vec::new();
    // Each step is judged by the whole layout it leads to, the irregular nester filling round
    // the blocks, searched quickly: stopping here, or another block in one of the rectangles.
    let mut stop = score(&free(region, items, &QUICK));
    loop {
        let mut rects = geometry::rectangles(&rest.rings());
        rects.sort_by_key(|r| std::cmp::Reverse(r[2] as Area * r[3] as Area));
        rects.truncate(CANDIDATES);
        let what = remaining(items, &left);
        let mut best = None;
        for r in rects {
            let block = guillotine(r, &what, &QUICK);
            if block.len() < 2 {
                continue;
            }
            let (after, after_left) = without(&rest, &left, &block);
            let mut layout = blocks.clone();
            layout.extend(block);
            layout.extend(free(&after, &remaining(items, &after_left), &QUICK));
            let s = score(&layout);
            if best.as_ref().is_none_or(|(b, _)| s > *b) {
                best = Some((s, r));
            }
        }
        let Some((s, r)) = best.filter(|(s, _)| *s >= stop) else {
            break;
        };
        let block = guillotine(r, &what, options);
        if block.len() < 2 {
            break;
        }
        (rest, left) = without(&rest, &left, &block);
        blocks.extend(block);
        stop = s;
    }
    if blocks.is_empty() {
        return alone;
    }
    blocks.extend(free(&rest, &remaining(items, &left), options));
    if score(&blocks) >= score(&alone) {
        blocks
    } else {
        alone
    }
}
