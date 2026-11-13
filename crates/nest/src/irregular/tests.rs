use super::*;
use crate::Placement;

fn item(w: Len, h: Len, copies: u32, rotate: bool) -> Item {
    Item {
        w,
        h,
        copies,
        profit: None,
        rotate,
    }
}

/// An oval `w` × `h` with its bottom-left corner at the origin, as `n` vertices on or just
/// inside it (rounded towards the centre), so the polygon lies inside the oval.
fn oval(w: Len, h: Len, n: usize) -> Region {
    let (a, b) = (w as f64 / 2.0, h as f64 / 2.0);
    let outline = (0..n)
        .map(|k| {
            let t = std::f64::consts::TAU * k as f64 / n as f64;
            [
                w / 2 + (a * t.cos()).trunc() as Len,
                h / 2 + (b * t.sin()).trunc() as Len,
            ]
        })
        .collect();
    Region {
        outline,
        holes: Vec::new(),
    }
}

fn bin(region: Region, copies: u32) -> Bin {
    Bin { region, copies }
}

fn options(max_width: usize) -> Options {
    Options {
        max_width,
        growth: 4.0,
    }
}

fn knapsack(bins: Vec<Bin>, items: Vec<Item>) -> Instance {
    Instance {
        objective: Objective::Knapsack,
        bins,
        items,
    }
}

fn count(s: &Solution, item: usize) -> usize {
    s.placements.iter().filter(|p| p.item == item).count()
}

#[test]
fn a_rectangle_fits_against_and_along_the_edges_but_not_across_them() {
    let r = Region::rect(200, 100);
    assert!(r.contains([0, 0, 200, 100]));
    assert!(r.contains([50, 0, 100, 100]));
    assert!(!r.contains([150, 0, 100, 100]));
    assert!(!r.contains([-1, 0, 10, 10]));
    let holed = Region {
        holes: vec![vec![[80, 30], [120, 30], [120, 70], [80, 70]]],
        ..r
    };
    assert!(holed.contains([0, 0, 80, 100]));
    assert!(holed.contains([80, 70, 40, 30]));
    assert!(!holed.contains([70, 20, 20, 20]));
    // A rectangle round the whole hole: no hole edge crosses it, but its centre is in the hole.
    assert!(!holed.contains([60, 10, 80, 80]));
}

#[test]
fn a_rectangular_bin_packs_like_a_sheet() {
    let instance = knapsack(
        vec![bin(Region::rect(200, 200), 1)],
        vec![item(100, 100, 5, false)],
    );
    let s = solve(&instance, &options(16)).expect("a layout");
    assert_eq!(count(&s, 0), 4);
    assert_eq!(s.profit, 40_000);
}

#[test]
fn the_largest_rectangle_in_an_oval_fits_and_a_larger_one_doesnt() {
    // A 4000 × 3000 oval holds a rectangle when (w / 4000)² + (h / 3000)² ≤ 1.
    let o = || vec![bin(oval(4000, 3000, 64), 1)];
    let fits = knapsack(o(), vec![item(2800, 2100, 1, false)]); // 0.98
    assert_eq!(count(&solve(&fits, &options(16)).expect("a layout"), 0), 1);
    let too_big = knapsack(o(), vec![item(2900, 2200, 1, false)]); // 1.06
    assert_eq!(
        count(&solve(&too_big, &options(16)).expect("a layout"), 0),
        0
    );
}

#[test]
fn an_oval_holds_more_than_its_largest_rectangle() {
    // The largest rectangle in a 4000 × 2000 oval is 2828 × 1414: two 1000 squares. The oval
    // itself holds three, one low in the middle and one either side.
    let instance = knapsack(
        vec![bin(oval(4000, 2000, 64), 1)],
        vec![item(1000, 1000, 4, false)],
    );
    let s = solve(&instance, &options(64)).expect("a layout");
    assert_eq!(count(&s, 0), 3, "{s:?}");
}

#[test]
fn mounts_keep_out_of_holes() {
    let region = Region {
        outline: vec![[0, 0], [3000, 0], [3000, 1000], [0, 1000]],
        // Another piece kept in the middle, nearly the bin's height.
        holes: vec![vec![[1000, 1], [2000, 1], [2000, 999], [1000, 999]]],
    };
    let instance = knapsack(vec![bin(region, 1)], vec![item(900, 900, 3, false)]);
    let s = solve(&instance, &options(16)).expect("a layout");
    assert_eq!(count(&s, 0), 2);
}

#[test]
fn a_mount_turns_to_fit() {
    let instance = knapsack(
        vec![bin(Region::rect(3000, 1000), 1)],
        vec![item(800, 2500, 1, true)],
    );
    let s = solve(&instance, &options(16)).expect("a layout");
    assert_eq!(s.placements.len(), 1);
    let p = s.placements[0];
    assert!(p.rotated);
    assert_eq!((p.w, p.h), (2500, 800));
}

#[test]
fn bin_packing_fills_bins_in_order() {
    let instance = Instance {
        objective: Objective::BinPacking,
        bins: vec![bin(oval(4000, 2000, 64), 3)],
        items: vec![item(1000, 1000, 5, false)],
    };
    let s = solve(&instance, &options(16)).expect("a layout");
    assert_eq!(s.bins_used, 2);
    assert_eq!(s.placements.len(), 5);
}

#[test]
fn bin_packing_refuses_a_mount_that_fits_nowhere() {
    let instance = Instance {
        objective: Objective::BinPacking,
        bins: vec![bin(oval(4000, 2000, 64), 1)],
        items: vec![item(1000, 1000, 1, false), item(3000, 1500, 1, false)],
    };
    assert_eq!(solve(&instance, &options(16)), Err(Error::TooBig(1)));
}

#[test]
fn the_check_catches_a_mount_across_the_outline() {
    let instance = knapsack(
        vec![bin(oval(4000, 2000, 64), 1)],
        vec![item(100, 100, 1, false)],
    );
    let corner = Solution {
        placements: vec![Placement {
            item: 0,
            bin: 0,
            x: 0,
            y: 0,
            w: 100,
            h: 100,
            rotated: false,
        }],
        bins_used: 1,
        profit: 10_000,
    };
    assert_eq!(check(&instance, &corner), Err(Violation::Outside(0)));
}

/// A `w` wide window with straight sides `h` high and a round top, the curve's vertices on it.
fn arch(w: Len, h: Len, n: usize) -> Region {
    let r = w as f64 / 2.0;
    let mut outline = vec![[0, 0], [w, 0]];
    for k in 0..=n {
        let t = std::f64::consts::PI * k as f64 / n as f64;
        outline.push([
            w / 2 + (r * t.cos()).trunc() as Len,
            h + (r * t.sin()).trunc() as Len,
        ]);
    }
    Region {
        outline,
        holes: Vec::new(),
    }
}

#[test]
fn the_straight_parts_are_found() {
    let rings = arch(3000, 2500, 32).rings();
    let rects = geometry::rectangles(&rings);
    assert!(rects.contains(&[0, 0, 3000, 2500]), "{rects:?}");
    assert!(rects.iter().all(|&r| geometry::contains(&rings, r)));
    let holed = Region {
        holes: vec![vec![[1000, 200], [2000, 200], [2000, 800], [1000, 800]]],
        ..Region::rect(3000, 1000)
    };
    let rects = geometry::rectangles(&holed.rings());
    for r in [
        [0, 0, 1000, 1000],
        [2000, 0, 1000, 1000],
        [0, 0, 3000, 200],
        [0, 800, 3000, 200],
    ] {
        assert!(rects.contains(&r), "{r:?} in {rects:?}");
    }
    assert_eq!(rects.len(), 4);
}

#[test]
fn straight_parts_get_guillotine_layouts() {
    // Three squares fit the oval either way; the guillotine nester puts them in one row, which
    // shares its cuts.
    let instance = knapsack(
        vec![bin(oval(4000, 2000, 64), 1)],
        vec![item(1000, 1000, 4, false)],
    );
    let mixed = solve(&instance, &options(64)).expect("a layout");
    let free = solve_free(&instance, &options(64)).expect("a layout");
    assert_eq!(count(&mixed, 0), 3);
    assert!(
        mixed
            .placements
            .iter()
            .all(|p| p.y == mixed.placements[0].y)
    );
    assert_eq!(strokes(&mixed), 6);
    assert!(strokes(&mixed) < strokes(&free));
}

#[test]
fn guillotine_blocks_never_cost_area() {
    // On the arch the irregular nester alone places more: that layout is kept.
    let instance = knapsack(
        vec![bin(arch(3000, 2500, 32), 1)],
        vec![
            item(1016, 1524, 6, true),
            item(1270, 1778, 6, true),
            item(2032, 2540, 3, true),
            item(1480, 2100, 4, true),
            item(1000, 1000, 6, true),
        ],
    );
    let mixed = solve(&instance, &options(64)).expect("a layout");
    let free = solve_free(&instance, &options(64)).expect("a layout");
    assert!(mixed.profit >= free.profit);
}
