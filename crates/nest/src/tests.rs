use super::*;

fn sheet(w: Len, h: Len, trim: Len, copies: u32) -> Bin {
    Bin {
        w,
        h,
        trims: Trims {
            left: trim,
            right: trim,
            top: trim,
            bottom: trim,
        },
        copies,
    }
}

fn item(w: Len, h: Len, copies: u32, rotate: bool) -> Item {
    Item {
        w,
        h,
        copies,
        profit: None,
        rotate,
    }
}

fn place(item: usize, bin: usize, x: Len, y: Len, w: Len, h: Len) -> Placement {
    Placement {
        item,
        bin,
        x,
        y,
        w,
        h,
        rotated: false,
    }
}

fn layout(instance: &Instance, placements: Vec<Placement>) -> Solution {
    let bins_used = placements.iter().map(|p| p.bin + 1).max().unwrap_or(0);
    let profit = placements.iter().map(|p| instance.profit(p.item)).sum();
    Solution {
        placements,
        bins_used,
        profit,
    }
}

/// The fewest sheets the items could fit on, by area.
fn area_bound(instance: &Instance) -> usize {
    let b = &instance.bins[0];
    let usable =
        (b.w - b.trims.left - b.trims.right) as Area * (b.h - b.trims.top - b.trims.bottom) as Area;
    let items: Area = instance
        .items
        .iter()
        .map(|i| i.w as Area * i.h as Area * i.copies as Area)
        .sum();
    ((items + usable - 1) / usable) as usize
}

#[test]
fn the_checker_accepts_a_three_stage_layout() {
    // Two bands: the first has a column of two items and a column of one; the second one item.
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(100, 100, 0, 1)],
        items: vec![item(40, 30, 3, false), item(30, 50, 1, false)],
    };
    let s = layout(
        &inst,
        vec![
            place(0, 0, 0, 0, 40, 30),
            place(0, 0, 0, 30, 40, 30),
            place(1, 0, 40, 0, 30, 50),
            place(0, 0, 0, 60, 40, 30),
        ],
    );
    assert_eq!(check(&inst, &s), Ok(()));
}

#[test]
fn the_checker_rejects_bad_layouts() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(100, 100, 5, 2)],
        items: vec![item(40, 40, 2, false)],
    };
    let ok = layout(
        &inst,
        vec![place(0, 0, 5, 5, 40, 40), place(0, 0, 45, 5, 40, 40)],
    );
    assert_eq!(check(&inst, &ok), Ok(()));
    let overlap = layout(
        &inst,
        vec![place(0, 0, 5, 5, 40, 40), place(0, 0, 30, 5, 40, 40)],
    );
    assert_eq!(check(&inst, &overlap), Err(Violation::Overlap(0, 1)));
    let in_trim = layout(
        &inst,
        vec![place(0, 0, 0, 5, 40, 40), place(0, 0, 45, 5, 40, 40)],
    );
    assert_eq!(check(&inst, &in_trim), Err(Violation::Outside(0)));
    let missing = layout(&inst, vec![place(0, 0, 5, 5, 40, 40)]);
    assert!(matches!(
        check(&inst, &missing),
        Err(Violation::Count { .. })
    ));
    let turned = layout(
        &inst,
        vec![
            Placement {
                rotated: true,
                ..place(0, 0, 5, 5, 40, 40)
            },
            place(0, 0, 45, 5, 40, 40),
        ],
    );
    assert_eq!(check(&inst, &turned), Err(Violation::Size(0)));
}

#[test]
fn the_checker_rejects_layouts_that_need_more_stages() {
    // A pinwheel: no guillotine cut at all.
    let inst = Instance {
        objective: Objective::Knapsack,
        bins: vec![sheet(30, 30, 0, 1)],
        items: vec![item(20, 10, 4, true), item(10, 10, 1, false)],
    };
    let pinwheel = layout(
        &inst,
        vec![
            place(0, 0, 0, 0, 20, 10),
            Placement {
                rotated: true,
                ..place(0, 0, 20, 0, 10, 20)
            },
            place(0, 0, 10, 20, 20, 10),
            Placement {
                rotated: true,
                ..place(0, 0, 0, 10, 10, 20)
            },
            place(1, 0, 10, 10, 10, 10),
        ],
    );
    assert_eq!(
        check(&inst, &pinwheel),
        Err(Violation::NotThreeStage { bin: 0 })
    );
    // Four stages: a tall item keeps the band whole, so in the other column one piece holds
    // two items side by side.
    let inst = Instance {
        objective: Objective::Knapsack,
        bins: vec![sheet(100, 100, 0, 1)],
        items: vec![
            item(20, 20, 2, false),
            item(40, 20, 1, false),
            item(20, 40, 1, false),
        ],
    };
    let four = layout(
        &inst,
        vec![
            place(1, 0, 0, 0, 40, 20),
            place(0, 0, 0, 20, 20, 20),
            place(0, 0, 20, 20, 20, 20),
            place(2, 0, 40, 0, 20, 40),
        ],
    );
    assert_eq!(
        check(&inst, &four),
        Err(Violation::NotThreeStage { bin: 0 })
    );
}

#[test]
fn four_mats_fill_one_sheet() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(8150, 10150, 50, 4)],
        items: vec![item(4000, 5000, 4, true)],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    assert_eq!(s.bins_used, 1);
}

#[test]
fn a_mat_turns_to_fit() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(510, 1010, 5, 1)],
        items: vec![item(600, 400, 1, true)],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    assert!(s.placements[0].rotated);
    let fixed = Instance {
        items: vec![item(600, 400, 1, false)],
        ..inst
    };
    assert_eq!(solve(&fixed, &Options::default()), Err(Error::TooBig(0)));
}

#[test]
fn identical_mats_from_different_orders_come_back_as_their_own_items() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(8150, 10150, 50, 10)],
        items: vec![
            item(4000, 5000, 2, true),
            item(3000, 2000, 3, true),
            item(5000, 4000, 1, true),
        ],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    for k in 0..3 {
        assert_eq!(
            s.placements.iter().filter(|p| p.item == k).count() as u32,
            inst.items[k].copies
        );
    }
}

#[test]
fn knapsack_fills_what_fits() {
    let inst = Instance {
        objective: Objective::Knapsack,
        bins: vec![sheet(1000, 1000, 0, 1)],
        items: vec![item(500, 500, 5, false)],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    assert_eq!(s.placements.len(), 4);
}

/// Small, seeded.
fn rng(seed: &mut u64) -> u64 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    *seed >> 33
}

#[test]
fn random_queues_give_legal_layouts_within_the_bounds() {
    let sizes = [
        (2030, 2540),
        (2790, 3560),
        (4060, 5080),
        (2100, 2970),
        (2970, 4200),
        (3000, 4000),
        (4000, 5000),
        (5000, 7000),
        (6000, 8000),
        (2500, 2000),
    ];
    let mut seed = 11;
    for _ in 0..40 {
        let kinds = 1 + rng(&mut seed) as usize % 6;
        let items: Vec<Item> = (0..kinds)
            .map(|_| {
                let (w, h) = sizes[rng(&mut seed) as usize % sizes.len()];
                item(w, h, 1 + rng(&mut seed) as u32 % 6, rng(&mut seed) % 4 != 0)
            })
            .collect();
        let copies = items.iter().map(|i| i.copies).sum();
        let inst = Instance {
            objective: Objective::BinPacking,
            bins: vec![sheet(8150, 10150, 50, copies)],
            items,
        };
        let options = Options {
            max_width: 64,
            ..Options::default()
        };
        let s = solve(&inst, &options).expect("solves");
        assert_eq!(check(&inst, &s), Ok(()));
        assert!(s.bins_used >= area_bound(&inst));
        assert_eq!(solve(&inst, &options), Ok(s));
    }
}

#[test]
fn a_drop_out_too_small_for_anything_is_skipped() {
    let inst = Instance {
        objective: Objective::Knapsack,
        bins: vec![
            sheet(100, 100, 0, 1),
            sheet(1000, 1000, 0, 1),
            sheet(600, 600, 0, 1),
        ],
        items: vec![item(500, 500, 3, false)],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    assert_eq!(s.placements.len(), 3);
    // Smallest bin first: the 600 one takes a mat, the 1000 one the other two.
    assert!(s.placements.iter().all(|p| p.bin == 1 || p.bin == 2));
}

/// How far down its bin the layout reaches.
fn extent(s: &Solution, bin: usize, top_trim: Len) -> Len {
    s.placements
        .iter()
        .filter(|p| p.bin == bin)
        .map(|p| p.y + p.h)
        .max()
        .unwrap_or(top_trim)
        - top_trim
}

#[test]
fn an_offcut_takes_the_mats_that_leave_most_of_the_sheet() {
    // The offcut takes either the tall mat or the two short ones. The tall one there leaves
    // the short ones side by side on the sheet: one band of 250, not 500.
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(400, 500, 0, 1), sheet(800, 1000, 0, 3)],
        items: vec![item(400, 500, 1, false), item(400, 250, 2, false)],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    assert_eq!(s.bins_used, 2);
    assert!(s.placements.iter().any(|p| p.item == 0 && p.bin == 0));
    assert_eq!(extent(&s, 1, 0), 250);
}

#[test]
fn the_last_sheet_keeps_its_biggest_band() {
    // Side by side, not stacked: the layout reaches 500 down the sheet, not 1000.
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(800, 1000, 0, 2)],
        items: vec![item(400, 500, 2, false)],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(extent(&s, 0, 0), 500);
}

/// The search counts item types in u16: one more is refused, not truncated into another type.
#[test]
fn too_many_item_types_are_refused() {
    let inst = Instance {
        objective: Objective::Knapsack,
        bins: vec![sheet(1000, 1000, 0, 1)],
        items: vec![item(1, 1, 1, false); usize::from(u16::MAX) + 1],
    };
    assert_eq!(solve(&inst, &Options::default()), Err(Error::TooMany));
}

fn strokes_of(s: &Solution) -> usize {
    crate::strokes::strokes(s.placements.iter().map(|p| (p.bin, p.x, p.y, p.w, p.h))).0
}

#[test]
fn edges_on_one_line_that_touch_are_one_stroke() {
    // Two mats side by side: top and bottom run across both, and they share the middle edge.
    let side_by_side = [(0, 0, 0, 100, 200), (0, 100, 0, 100, 200)];
    assert_eq!(crate::strokes::strokes(side_by_side), (5, 1000));
    // On different sheets nothing is shared.
    let apart = [(0, 0, 0, 100, 200), (1, 100, 0, 100, 200)];
    assert_eq!(crate::strokes::strokes(apart).0, 8);
}

/// Two columns whose stacks are the other way round: turning one over lines up the edge
/// between the mats, one stroke instead of two, and the sheet's leftover stays.
#[test]
fn rearranging_lines_up_the_stacks() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(1000, 1000, 0, 1)],
        items: vec![item(100, 200, 2, false), item(100, 100, 2, false)],
    };
    let mut s = layout(
        &inst,
        vec![
            place(0, 0, 0, 0, 100, 200),
            place(1, 0, 0, 200, 100, 100),
            place(1, 0, 100, 0, 100, 100),
            place(0, 0, 100, 100, 100, 200),
        ],
    );
    assert_eq!(check(&inst, &s), Ok(()));
    assert_eq!(strokes_of(&s), 7);
    crate::strokes::rearrange(&inst, &mut s);
    assert_eq!(check(&inst, &s), Ok(()));
    assert_eq!(strokes_of(&s), 6, "{:?}", s.placements);
    let bottom = s.placements.iter().map(|p| p.y + p.h).max();
    assert_eq!(bottom, Some(300), "the same depth of sheet used");
}

/// The search prefers, of equally good layouts, the one with fewer strokes, and the
/// rearranging never loses one.
#[test]
fn fewer_strokes_without_more_sheets() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(8150, 10150, 50, 4)],
        items: vec![
            item(3000, 4000, 3, true),
            item(3000, 2000, 4, true),
            item(2500, 2000, 3, true),
        ],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    let mut again = s.clone();
    crate::strokes::rearrange(&inst, &mut again);
    assert!(strokes_of(&again) <= strokes_of(&s));
    assert_eq!(s.bins_used, area_bound(&inst));
}

/// Rearranging starts from the layout as it is, so a second time changes nothing.
#[test]
fn rearranging_twice_changes_nothing() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(8150, 10150, 50, 6)],
        items: vec![
            item(3050, 4060, 4, true),
            item(5000, 4000, 5, true),
            item(2970, 4200, 6, true),
            item(2500, 2000, 1, true),
        ],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    let mut again = s.clone();
    crate::strokes::rearrange(&inst, &mut again);
    assert_eq!(strokes_of(&again), strokes_of(&s));
}

/// Layouts start at the bottom-left corner (the left clamp's, where boards register) and the
/// leftover is a band along the top: trims of different sizes show which edge is which.
#[test]
fn layouts_start_at_the_bottom_left() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![Bin {
            w: 1000,
            h: 1000,
            trims: Trims {
                left: 20,
                right: 5,
                top: 50,
                bottom: 10,
            },
            copies: 1,
        }],
        items: vec![item(300, 200, 5, false)],
    };
    let s = solve(&inst, &Options::default()).expect("solves");
    assert_eq!(check(&inst, &s), Ok(()));
    let min = |f: fn(&Placement) -> Len| s.placements.iter().map(f).min();
    assert_eq!(min(|p| p.x), Some(20));
    assert_eq!(min(|p| p.y), Some(10));
    let top = s.placements.iter().map(|p| p.y + p.h).max();
    assert_eq!(top, Some(410), "two strips of 200 from the bottom trim");
}
