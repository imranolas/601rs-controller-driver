use machine::Profile;
use program::{BladeOffsets, ClampSide, Head, Op};

use super::*;

fn material() -> Material {
    Material {
        name: "MSK_1.5".to_owned(),
        speed_pct: 95.0,
        head_settle_ms: 250,
        blade_angle_offset_deg: BladeOffsets {
            straight: 0.0,
            bevel: 3.0,
            bevel_round: None,
        },
    }
}

fn sheet(w: f64, h: f64, contours: Vec<Contour>) -> Sheet {
    Sheet {
        id: "test".to_owned(),
        meta: None,
        board: Board {
            width_mm: w,
            height_mm: h,
            thickness_mm: 1.5,
        },
        material: material(),
        contours,
        cleanup: Vec::new(),
    }
}

fn rect(box_mm: [f64; 4], tool: Tool, keep: Keep) -> Contour {
    Contour {
        id: None,
        label: None,
        shape: Shape::Rect,
        box_mm,
        tool,
        keep,
    }
}

fn mount(box_mm: [f64; 4]) -> Contour {
    rect(box_mm, Tool::Straight, Keep::In)
}

fn window(box_mm: [f64; 4]) -> Contour {
    rect(box_mm, Tool::Bevel, Keep::Out)
}

fn run(s: &Sheet) -> Compiled {
    run_with(s, &Options::default())
}

fn run_with(s: &Sheet, o: &Options) -> Compiled {
    compile(s, &Blade::fitted(), &Profile::gunnar_601(), o).expect("compiles")
}

/// The cuts: id, head, path.
fn cuts(c: &Compiled) -> Vec<(String, Head, Vec<[f64; 2]>)> {
    c.program
        .ops
        .iter()
        .filter_map(|op| match op {
            Op::Cut(c) => Some((c.id.clone().unwrap_or_default(), c.head, c.path_mm.clone())),
            _ => None,
        })
        .collect()
}

/// Where in the program the cut with an id containing `part` is.
fn at(c: &Compiled, part: &str) -> usize {
    cuts(c)
        .iter()
        .position(|(id, _, _)| id.split('+').any(|p| p == part))
        .unwrap_or_else(|| panic!("no cut of {part}"))
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

#[test]
fn a_window_is_cut_clockwise_inside_its_edge() {
    let c = run(&sheet(
        300.0,
        400.0,
        vec![window([100.0, 100.0, 200.0, 300.0])],
    ));
    let cs = cuts(&c);
    assert_eq!(cs.len(), 4);
    let e = 0.4525;
    let (_, head, right) = &cs[at(&c, "c1.right")];
    assert_eq!(*head, Head::Bevel);
    // Down the right edge, its line 0.4525 mm inside, from 0.4 mm above the top line to
    // 3.8 mm below the bottom one.
    assert!(close(right[0][0], 200.0 - e) && close(right[1][0], 200.0 - e));
    assert!(close(right[0][1], 300.0 - e + 0.4) && close(right[1][1], 100.0 + e - 3.8));
    // Clockwise: right, bottom, left, top, each starting where the last ended.
    let (_, _, bottom) = &cs[at(&c, "c1.bottom")];
    assert!(bottom[0][0] > bottom[1][0], "the bottom edge runs −X");
}

#[test]
fn a_kept_bevel_piece_is_cut_anticlockwise_outside_its_edge() {
    let c = run(&sheet(
        300.0,
        400.0,
        vec![rect([100.0, 100.0, 200.0, 300.0], Tool::Bevel, Keep::In)],
    ));
    let cs = cuts(&c);
    let (_, _, right) = &cs[at(&c, "c1.right")];
    assert!(close(right[0][0], 200.4525), "outside the piece: {right:?}");
    assert!(right[1][1] > right[0][1], "up the right edge");
}

#[test]
fn neighbours_share_their_cuts() {
    // Two mounts side by side: one top, one bottom and three uprights.
    let c = run(&sheet(
        400.0,
        300.0,
        vec![
            mount([50.0, 50.0, 150.0, 200.0]),
            mount([150.0, 50.0, 250.0, 200.0]),
        ],
    ));
    let cs = cuts(&c);
    assert_eq!(cs.len(), 5, "{cs:?}");
    let (id, _, middle) = &cs[at(&c, "c1.right")];
    assert_eq!(id, "c1.right+c2.left");
    // Kept both sides: the slit's middle on the edge, 0.095 mm right of the line going +Y.
    assert!(close(middle[0][0], 150.0 - 0.095), "{middle:?}");
    // Every straight cut runs +X or +Y.
    for (_, _, p) in &cs {
        assert!(p[1][0] >= p[0][0] && p[1][1] >= p[0][1], "{p:?}");
    }
}

#[test]
fn each_mount_is_cut_free_on_its_left_edge() {
    // A 2 × 2 grid of mounts, sharing edges: each line between them is one stroke.
    let m = |x: f64, y: f64| mount([x, y, x + 100.0, y + 100.0]);
    let c = run(&sheet(
        400.0,
        400.0,
        vec![
            m(50.0, 50.0),
            m(150.0, 50.0),
            m(50.0, 150.0),
            m(150.0, 150.0),
        ],
    ));
    assert_eq!(cuts(&c).len(), 6);
    for k in 1..=4 {
        let others = ["top", "right", "bottom"].map(|s| at(&c, &format!("c{k}.{s}")));
        let left = at(&c, &format!("c{k}.left"));
        assert!(
            others.iter().all(|&o| o < left),
            "mount {k}: its left edge last"
        );
    }
}

#[test]
fn a_piece_in_a_drop_out_comes_before_the_window_round_it() {
    let c = run(&sheet(
        400.0,
        400.0,
        vec![
            mount([50.0, 50.0, 350.0, 350.0]),
            window([100.0, 100.0, 300.0, 300.0]),
            mount([150.0, 150.0, 250.0, 200.0]),
        ],
    ));
    let piece = ["top", "right", "bottom", "left"].map(|s| at(&c, &format!("c3.{s}")));
    let opening = ["top", "right", "bottom", "left"].map(|s| at(&c, &format!("c2.{s}")));
    let outline = ["top", "right", "bottom", "left"].map(|s| at(&c, &format!("c1.{s}")));
    assert!(piece.iter().max() < opening.iter().min());
    assert!(opening.iter().max() < outline.iter().min());
}

#[test]
fn a_cut_that_cannot_keep_off_a_pad_turns_it() {
    // The mount's bottom edge runs 5 mm from the front edge, over pad F1 (75.5–118.5 mm).
    let s = sheet(300.0, 300.0, vec![mount([50.0, 5.0, 150.0, 100.0])]);
    let c = run(&s);
    assert_eq!(c.turned, ["F1"]);
    match &c.program.ops[0] {
        Op::Clamp(clamp) => assert_eq!(clamp.turned, ["F1"]),
        op => panic!("{op:?}"),
    }
}

#[test]
fn a_way_that_keeps_off_the_pads_is_taken() {
    // The bottom edge stops 1.5 mm short of F1: going +X it runs on over it, going −X not.
    let s = sheet(300.0, 300.0, vec![mount([30.0, 5.0, 74.0, 60.0])]);
    assert_eq!(run(&s).turned, ["F1"]);
    let either = Options {
        straight_away_from_clamps: false,
        ..Options::default()
    };
    assert!(run_with(&s, &either).turned.is_empty());
}

#[test]
fn a_wide_board_is_held_by_both_clamps() {
    let side = |w: f64| match &run(&sheet(w, 300.0, vec![mount([100.0, 100.0, 200.0, 200.0])]))
        .program
        .ops[0]
    {
        Op::Clamp(c) => c.side,
        op => panic!("{op:?}"),
    };
    assert_eq!(side(700.0), ClampSide::Left);
    assert_eq!(side(900.0), ClampSide::Both);
}

#[test]
fn an_oval_piece_is_cut_clockwise_with_its_run_out_outside() {
    let c = run(&sheet(
        400.0,
        400.0,
        vec![Contour {
            shape: Shape::Oval,
            ..mount([100.0, 100.0, 300.0, 250.0])
        }],
    ));
    let cs = cuts(&c);
    assert_eq!(cs.len(), 1);
    let (_, head, path) = &cs[0];
    assert_eq!(*head, Head::Straight);
    let centre = [200.0, 175.0];
    // Each point on the ellipse grown by the straight head's right edge (0.0875 mm): the
    // normalised radius is 1 to within that over the semi-axes.
    let r = |p: [f64; 2]| ((p[0] - centre[0]) / 100.0).hypot((p[1] - centre[1]) / 75.0);
    assert!(
        path.iter()
            .all(|&p| r(p) > 1.0 && r(p) < 1.0 + 0.3 / 75.0 + 1e-3)
    );
    assert!(
        r(path[path.len() - 1]) > r(path[0]) + 0.2 / 100.0,
        "runs out outside"
    );
}

#[test]
fn a_v_groove_waits_for_a_later_format() {
    let s = sheet(
        300.0,
        300.0,
        vec![rect([50.0, 50.0, 150.0, 150.0], Tool::Vgroove, Keep::Out)],
    );
    assert_eq!(
        compile(
            &s,
            &Blade::fitted(),
            &Profile::gunnar_601(),
            &Options::default()
        ),
        Err(Error::Vgroove { contour: 0 })
    );
}

#[test]
fn a_contour_off_the_board_is_refused() {
    let s = sheet(300.0, 300.0, vec![mount([250.0, 50.0, 350.0, 150.0])]);
    assert_eq!(
        compile(
            &s,
            &Blade::fitted(),
            &Profile::gunnar_601(),
            &Options::default()
        ),
        Err(Error::BadContour { contour: 0 })
    );
}

#[test]
fn the_same_sheet_gives_the_same_program() {
    let s = sheet(
        500.0,
        400.0,
        vec![
            mount([50.0, 50.0, 250.0, 350.0]),
            window([100.0, 100.0, 200.0, 300.0]),
            mount([250.0, 50.0, 450.0, 350.0]),
            Contour {
                shape: Shape::Oval,
                ..window([300.0, 100.0, 400.0, 300.0])
            },
        ],
    );
    assert_eq!(run(&s).program, run(&s).program);
}

#[test]
fn an_edge_out_of_the_head_s_reach_is_refused() {
    // The bevel head sits 71.5 mm left of the carriage: it can't reach past about 1,488 mm.
    let s = sheet(1550.0, 300.0, vec![window([1450.0, 100.0, 1540.0, 200.0])]);
    assert_eq!(
        compile(
            &s,
            &Blade::fitted(),
            &Profile::gunnar_601(),
            &Options::default()
        ),
        Err(Error::OutOfReach {
            contour: 0,
            head: Head::Bevel
        })
    );
    // The straight head reaches the board's left edge.
    run(&sheet(300.0, 300.0, vec![mount([0.0, 50.0, 100.0, 150.0])]));
}

#[test]
fn a_board_with_every_pad_turned_is_refused() {
    // On a 300 mm board only F1 and L1 hold it, and this mount's edges run over both.
    let s = sheet(300.0, 300.0, vec![mount([10.0, 5.0, 290.0, 295.0])]);
    assert_eq!(
        compile(
            &s,
            &Blade::fitted(),
            &Profile::gunnar_601(),
            &Options::default()
        ),
        Err(Error::Unclamped {
            turned: vec!["F1".to_owned(), "L1".to_owned()]
        })
    );
    // One pad turned, the other still holding.
    let c = run(&sheet(300.0, 300.0, vec![mount([30.0, 5.0, 290.0, 295.0])]));
    assert_eq!(
        (c.turned, c.held_by),
        (vec!["F1".to_owned()], vec!["L1".to_owned()])
    );
}

/// An app's example: a 600 × 500 piece in the trims' corner of a
/// 1015 × 815 sheet frees the rest of its strip and the band above it.
fn bevels_corner() -> Sheet {
    Sheet {
        cleanup: vec![
            [621.0, 21.0, 1015.0, 21.0],
            [0.0, 521.0, 1015.0, 521.0],
            [621.0, 21.0, 621.0, 521.0],
        ],
        ..sheet(
            1015.0,
            815.0,
            vec![
                mount([21.0, 21.0, 621.0, 521.0]),
                window([80.0, 80.0, 560.0, 460.0]),
            ],
        )
    }
}

#[test]
fn clean_up_lines_share_cuts_with_the_pieces_as_the_nester_counts() {
    // Each of the three clean-up lines is on one of the piece's edges, overlapping or touching it:
    // the strip's front line runs on from the piece's bottom edge, the band's line through its
    // top edge, the line up its side is its right edge. Four straight cuts, with or without.
    let with = run(&bevels_corner());
    let without = run(&Sheet {
        cleanup: Vec::new(),
        ..bevels_corner()
    });
    let straight = |c: &Compiled| {
        cuts(c)
            .into_iter()
            .filter(|(_, h, _)| *h == Head::Straight)
            .collect::<Vec<_>>()
    };
    assert_eq!(straight(&with).len(), 4, "{:?}", straight(&with));
    assert_eq!(straight(&without).len(), 4);
    let line = |id: &str| {
        straight(&with)
            .into_iter()
            .find(|(i, _, _)| i.split('+').any(|p| p == id))
            .unwrap_or_else(|| panic!("no cut of {id}"))
    };
    // The bottom edge runs on to the board's right edge (1015, and its run-on past it).
    let (id, _, bottom) = line("c1.bottom");
    assert!(id.contains("cleanup.1"), "{id}");
    assert!(bottom[1][0] > 1015.0, "{bottom:?}");
    // The top edge from the board's left edge (starting 0.5 mm in, as every straight cut does)
    // to its right: the straight head reaches past the left edge, as GMC's does when it cuts to
    // the edge.
    let (id, _, top) = line("c1.top");
    assert!(id.contains("cleanup.2"), "{id}");
    assert!((top[0][0] - 0.5).abs() < 1e-9, "{top:?}");
    assert!(line("c1.right").0.contains("cleanup.3"));
    assert!(with.uncut.is_empty(), "{:?}", with.uncut);
    assert!(with.turned.is_empty());
}

#[test]
fn a_clean_up_line_on_no_piece_s_edge_comes_after_every_piece() {
    // A line up through the band, dividing it in two: no piece's edge is on it.
    let mut s = bevels_corner();
    s.cleanup.push([800.0, 521.0, 800.0, 815.0]);
    let c = run(&s);
    let cs = cuts(&c);
    let alone = cs
        .iter()
        .position(|(id, _, _)| id == "cleanup.4")
        .expect("its own cut");
    assert_eq!(alone, cs.len() - 1, "after every piece's cut: {cs:?}");
    assert!(
        cs.iter()
            .filter(|(id, _, _)| id.split('+').all(|p| p.starts_with("cleanup.")))
            .count()
            == 1,
        "the others share the pieces' cuts"
    );
}

#[test]
fn a_clean_up_line_over_a_pad_turns_it_before_the_run() {
    // Up from the front edge through pad F1, beside a mount.
    let s = Sheet {
        cleanup: vec![[97.0, 0.0, 97.0, 200.0]],
        ..sheet(400.0, 400.0, vec![mount([150.0, 50.0, 350.0, 350.0])])
    };
    let c = run(&s);
    assert_eq!(c.turned, ["F1"]);
    assert!(!c.held_by.contains(&"F1".to_owned()));
    match &c.program.ops[0] {
        Op::Clamp(clamp) => assert_eq!(clamp.turned, ["F1"]),
        op => panic!("{op:?}"),
    }
    // All of it, from the front edge.
    assert!(c.uncut.is_empty(), "{:?}", c.uncut);
}

#[test]
fn clean_up_lines_along_the_pieces_alone_add_no_cut() {
    let s = Sheet {
        cleanup: vec![[621.0, 21.0, 621.0, 521.0]],
        ..bevels_corner()
    };
    let c = run(&s);
    assert!(
        cuts(&c)
            .iter()
            .all(|(id, _, _)| !id.starts_with("cleanup."))
    );
}

#[test]
fn a_clean_up_line_must_run_along_x_or_y() {
    let s = Sheet {
        cleanup: vec![[100.0, 100.0, 200.0, 150.0]],
        ..bevels_corner()
    };
    assert_eq!(
        compile(
            &s,
            &Blade::fitted(),
            &Profile::gunnar_601(),
            &Options::default()
        ),
        Err(Error::BadCleanup { line: 0 })
    );
}
