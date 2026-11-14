//! GMC's own jobs (`testdata/programs`), rebuilt from their designs with GMC's blade numbers
//! ([`Blade::gmc`]): the compiler must make GMC's cuts.
//!
//! The designs are read back from GMC's lines by the same numbers: a bevel window's line lies
//! 0.1 mm inside its back edge, a straight line on its edge. The windows are the 100 × 200 mm
//! the captures were made with (protocol reference §1).

use std::path::Path;

use compiler::{Blade, Contour, Keep, Options, Shape, Sheet, Tool, compile};
use machine::Profile;
use program::{Head, Op, Program};

fn fixture(name: &str) -> Program {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/programs")
        .join(format!("{name}.json"));
    Program::from_json(&std::fs::read_to_string(path).expect("fixture reads")).expect("a program")
}

fn contour(shape: Shape, box_mm: [f64; 4], tool: Tool, keep: Keep) -> Contour {
    Contour {
        id: None,
        label: None,
        shape,
        box_mm,
        tool,
        keep,
    }
}

/// GMC's job's board and material, with `contours`.
fn sheet(gmc: &Program, contours: Vec<Contour>) -> Sheet {
    Sheet {
        id: gmc.id.clone(),
        meta: None,
        board: gmc.board.clone(),
        material: gmc.material.clone(),
        contours,
        cleanup: Vec::new(),
    }
}

fn options() -> Options {
    Options {
        // GMC starts a closed curve at its top.
        curve_starts: 1,
        ..Options::default()
    }
}

fn cuts(p: &Program) -> Vec<(Head, Vec<[f64; 2]>)> {
    p.ops
        .iter()
        .filter_map(|op| match op {
            Op::Cut(c) => Some((c.head, c.path_mm.clone())),
            _ => None,
        })
        .collect()
}

fn near(a: [f64; 2], b: [f64; 2], tol: f64) -> bool {
    (a[0] - b[0]).abs() <= tol && (a[1] - b[1]).abs() <= tol
}

/// Every one of GMC's rectangle cuts is one of ours, the same way round: bevel edges to
/// 0.03 mm (GMC's steps are 0.025 mm). Straight lines lie on the same line to 0.03 mm and run
/// over the same span to 0.3 mm: read back in this profile's frame, GMC's straight lines sit
/// about 0.26 mm along from where its own trims put them, the other way on each axis (its
/// straight head's offset differs from the fitted one).
fn same_rectangle_cuts(gmc: &Program, ours: &Program) {
    let (theirs, mine) = (cuts(gmc), cuts(ours));
    assert_eq!(theirs.len(), mine.len(), "as many cuts");
    for (head, path) in &theirs {
        let (a, b) = (path[0], path[path.len() - 1]);
        let found = mine.iter().any(|(h, p)| {
            let (c, d) = (p[0], p[p.len() - 1]);
            h == head
                && match head {
                    Head::Bevel => near(a, c, 0.03) && near(b, d, 0.03),
                    Head::Straight => {
                        let across = if a[1] == b[1] { 1 } else { 0 };
                        (a[across] - c[across]).abs() <= 0.03
                            && (b[across] - d[across]).abs() <= 0.03
                            && near(a, c, 0.3)
                            && near(b, d, 0.3)
                    }
                }
        });
        assert!(
            found,
            "GMC's {head:?} cut {a:?} → {b:?} isn't made: {mine:?}"
        );
    }
}

/// The windows before the border, and the border ending on its left edge, as GMC's does.
fn in_order(ours: &Program) {
    let heads: Vec<Head> = cuts(ours).into_iter().map(|(h, _)| h).collect();
    let first_straight = heads.iter().position(|&h| h == Head::Straight);
    assert!(
        first_straight.is_some_and(|k| heads[k..].iter().all(|&h| h == Head::Straight)),
        "windows first: {heads:?}"
    );
    let labels: Vec<&str> = ours.ops.iter().filter_map(|op| op.label()).collect();
    assert!(
        labels[labels.len() - 1].ends_with("left edge"),
        "cut free on the left, as GMC: {labels:?}"
    );
}

#[test]
fn gmcs_regular_mount() {
    let gmc = fixture("g_regular_mount");
    let s = sheet(
        &gmc,
        vec![
            contour(
                Shape::Rect,
                [70.8, 71.38, 170.8, 271.38],
                Tool::Bevel,
                Keep::Out,
            ),
            contour(
                Shape::Rect,
                [21.21, 21.18, 221.21, 331.18],
                Tool::Straight,
                Keep::In,
            ),
        ],
    );
    let c = compile(&s, &Blade::gmc(), &Profile::gunnar_601(), &options()).expect("compiles");
    same_rectangle_cuts(&gmc, &c.program);
    in_order(&c.program);
    assert!(c.turned.is_empty(), "keeps off the pads: {:?}", c.turned);
}

#[test]
fn gmcs_twin_windows_on_three_thicknesses() {
    // The bevel cuts run on further on thicker board, by its thickness at each end; the lines
    // don't move.
    for name in ["g_multi_C10", "g_multi_C11", "g_multi_C12"] {
        let gmc = fixture(name);
        let s = sheet(
            &gmc,
            vec![
                contour(
                    Shape::Rect,
                    [70.825, 71.405, 170.825, 271.405],
                    Tool::Bevel,
                    Keep::Out,
                ),
                contour(
                    Shape::Rect,
                    [220.825, 71.405, 320.825, 271.405],
                    Tool::Bevel,
                    Keep::Out,
                ),
                contour(
                    Shape::Rect,
                    [21.235, 21.205, 371.235, 321.205],
                    Tool::Straight,
                    Keep::In,
                ),
            ],
        );
        let c = compile(&s, &Blade::gmc(), &Profile::gunnar_601(), &options()).expect("compiles");
        same_rectangle_cuts(&gmc, &c.program);
        in_order(&c.program);
    }
}

/// The distance from `p` to the polyline `line`.
fn off(p: [f64; 2], line: &[[f64; 2]]) -> f64 {
    line.windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let t =
                (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
        })
        .fold(f64::INFINITY, f64::min)
}

fn length(path: &[[f64; 2]]) -> f64 {
    path.windows(2)
        .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
        .sum()
}

#[test]
fn gmcs_oval() {
    let gmc = fixture("g_oval_mount");
    let s = sheet(
        &gmc,
        vec![
            contour(
                Shape::Oval,
                [60.813, 56.394, 160.811, 256.392],
                Tool::Bevel,
                Keep::Out,
            ),
            contour(
                Shape::Rect,
                [21.235, 21.205, 201.235, 291.205],
                Tool::Straight,
                Keep::In,
            ),
        ],
    );
    let c = compile(&s, &Blade::gmc(), &Profile::gunnar_601(), &options()).expect("compiles");
    let (theirs, mine) = (cuts(&gmc), cuts(&c.program));
    let gmc_oval = &theirs
        .iter()
        .find(|(h, _)| *h == Head::Bevel)
        .expect("GMC's oval")
        .1;
    let oval = &mine
        .iter()
        .find(|(h, _)| *h == Head::Bevel)
        .expect("our oval")
        .1;
    // GMC's first lap: its first 604 points.
    let gmc_lap = &gmc_oval[..604];
    let lap_len = length(gmc_lap);
    // Ours starts at the top too, and goes round clockwise on GMC's line.
    assert!(
        near(oval[0], gmc_oval[0], 0.05),
        "{:?} vs {:?}",
        oval[0],
        gmc_oval[0]
    );
    let area: f64 = oval
        .windows(2)
        .take_while(|w| length(&[oval[0], w[1]]) > 0.0)
        .map(|w| w[0][0] * w[1][1] - w[1][0] * w[0][1])
        .sum();
    assert!(area < 0.0, "clockwise");
    let mut along = 0.0;
    let mut lap_points = 0;
    for w in oval.windows(2) {
        if along >= lap_len - 0.5 {
            break;
        }
        along += length(w);
        lap_points += 1;
        assert!(
            off(w[1], gmc_lap) < 0.05,
            "{:?} is {} mm off GMC's lap",
            w[1],
            off(w[1], gmc_lap)
        );
    }
    // Far fewer points than GMC's 1,108 for the whole cut.
    assert!(
        oval.len() * 2 < gmc_oval.len(),
        "{} points against GMC's {}",
        oval.len(),
        gmc_oval.len()
    );
    // The overlap: GMC's 1.9 laps, to within a few millimetres (its last point is the first past
    // its time; ours the same, on longer chords).
    let (ours_len, gmc_len) = (length(oval), length(gmc_oval));
    assert!(
        (ours_len - gmc_len).abs() < 6.0,
        "{ours_len:.1} mm against GMC's {gmc_len:.1}"
    );
    // And it ends 0.3 mm inside the line, in the waste.
    let end = oval[oval.len() - 1];
    let inside = off(end, gmc_lap);
    assert!(
        (inside - 0.3).abs() < 0.05,
        "ends {inside:.3} mm off the line"
    );
    assert!(within(end, gmc_lap), "inside the window, not outside");
    assert!(lap_points > 100, "{lap_points} lap points");
}

/// `p` lies inside the closed polyline `ring` (even-odd).
fn within(p: [f64; 2], ring: &[[f64; 2]]) -> bool {
    let mut inside = false;
    for w in ring.windows(2) {
        let (a, b) = (w[0], w[1]);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
            if p[0] < x {
                inside = !inside;
            }
        }
    }
    inside
}
