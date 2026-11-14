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
    let e = 0.453;
    let (_, head, right) = &cs[at(&c, "c1.right")];
    assert_eq!(*head, Head::Bevel);
    // Down the right edge, its line 0.453 mm inside, from 0.4 mm above the top line to
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
    assert!(close(right[0][0], 200.453), "outside the piece: {right:?}");
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
    assert!(close(middle[0][0], 150.0 - 0.0945), "{middle:?}");
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
    // Each point on the ellipse grown by the straight head's right edge (0.087 mm): the
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
fn an_edge_the_straight_head_cannot_reach_is_refused() {
    // The straight head sits 6.4 mm right of the carriage: it can't reach x 3.
    let s = sheet(300.0, 300.0, vec![mount([3.0, 50.0, 100.0, 150.0])]);
    assert_eq!(
        compile(
            &s,
            &Blade::fitted(),
            &Profile::gunnar_601(),
            &Options::default()
        ),
        Err(Error::OutOfReach {
            contour: 0,
            head: Head::Straight
        })
    );
    // At 10 mm it can.
    run(&sheet(
        300.0,
        300.0,
        vec![mount([10.0, 50.0, 100.0, 150.0])],
    ));
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
