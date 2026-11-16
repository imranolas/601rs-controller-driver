//! Calibration plans compared before anything is cut: how precisely each fixes what a mount
//! depends on, from how many readings, and how many of those nothing would check.
//!
//! - **Sheet 2**: calibration sheet 2's strips and window, read from the board's edges.
//! - **Test mount**: a 120 × 100 mm mount with a 60 × 40 mm window (borders of 30 mm), cut as
//!   any mount is: the outline with the straight head, the window with the bevel. Its four
//!   borders are read across, outside jaws on two cut edges.
//!
//! ```sh
//! cargo run -p calibrate --example designs
//! ```

use std::collections::HashMap;

use calibrate::{Precision, Prior, Sheet, Unknown, precision};
use machine::Profile;
use program::{BladeOffsets, Board, Clamp, ClampSide, Cut, Head, Material, Op, Program};
use serde_json::{Value, json};

const SHEET_2: [&str; 11] = [
    "SL.width",
    "SR.width",
    "SL.length",
    "S.left",
    "S.front",
    "W.left",
    "W.width",
    "W.front",
    "W.height",
    "D.width",
    "D.height",
];

fn edge(cut: &str, kept: &str, face: &str) -> Value {
    json!({ "cut": cut, "kept": kept, "face": face })
}

fn width(name: &str, axis: &str, a: Value, b: Value) -> Value {
    json!({ "name": name, "kind": "width", "axis": axis, "edges": [a, b] })
}

/// The test mount, in carriage coordinates, cut raw: the window first, clockwise, then the
/// outline, each straight stroke +X or +Y as the compiler cuts them. `mixed`: the window's
/// right and back edges cut with the straight head (+Y and +X), its left and front with the
/// bevel.
fn test_mount(mixed: bool) -> Program {
    let (s, b) = (Head::Straight, Head::Bevel);
    let ([x0, x1, y0, y1], o) = ([15.0, 135.0, 15.0, 115.0], 3.0);
    let ([wx0, wx1, wy0, wy1], w) = ([45.0, 105.0, 45.0, 85.0], 4.5);
    let cut = |id: &str, head, a: [f64; 2], z: [f64; 2]| {
        Op::Cut(Cut {
            id: Some(id.into()),
            label: None,
            head,
            path_mm: vec![a, z],
            speed_pct: None,
        })
    };
    let ops = vec![
        Op::Clamp(Clamp {
            id: None,
            label: None,
            side: ClampSide::Left,
            turned: Vec::new(),
        }),
        if mixed {
            cut("W.right", s, [wx1, wy0 - o], [wx1, wy1 + o])
        } else {
            cut("W.right", b, [wx1, wy1 + w], [wx1, wy0 - w])
        },
        cut("W.front", b, [wx1 + w, wy0], [wx0 - w, wy0]),
        cut("W.left", b, [wx0, wy0 - w], [wx0, wy1 + w]),
        if mixed {
            cut("W.back", s, [wx0 - o, wy1], [wx1 + o, wy1])
        } else {
            cut("W.back", b, [wx0 - w, wy1], [wx1 + w, wy1])
        },
        cut("O.front", s, [x0 - o, y0], [x1 + o, y0]),
        cut("O.back", s, [x0 - o, y1], [x1 + o, y1]),
        cut("O.left", s, [x0, y0 - o], [x0, y1 + o]),
        cut("O.right", s, [x1, y0 - o], [x1, y1 + o]),
    ];
    // The mount is right of O.left and O.back as they travel, left of O.right and O.front;
    // the window keeps the board on its left, clockwise.
    let (ol, or, of, ob) = (
        edge("O.left", "right", "any"),
        edge("O.right", "left", "any"),
        edge("O.front", "left", "any"),
        edge("O.back", "right", "any"),
    );
    // The window's edges keep the board: on the bevel's left as it goes clockwise; mixed, the
    // straight right edge (+Y) keeps it on its right and the back (+X) on its left.
    let w = |c: &str| match (mixed, c) {
        (true, "W.right") => edge(c, "right", "any"),
        (true, "W.back") => edge(c, "left", "any"),
        _ => edge(c, "left", "back"),
    };
    let d = |c: &str| edge(c, "right", "face");
    let readings = vec![
        width("O.width", "x", ol.clone(), or.clone()),
        width("O.height", "y", of.clone(), ob.clone()),
        width("B.left", "x", ol.clone(), w("W.left")),
        width("B.right", "x", w("W.right"), or),
        width("B.front", "y", of.clone(), w("W.front")),
        width("B.back", "y", w("W.back"), ob),
        width("W.width", "x", w("W.left"), w("W.right")),
        width("W.height", "y", w("W.front"), w("W.back")),
        width("D.width", "x", d("W.left"), d("W.right")),
        width("D.height", "y", d("W.front"), d("W.back")),
        json!({ "name": "S.left", "kind": "position", "axis": "x", "edge": ol }),
        json!({ "name": "S.front", "kind": "position", "axis": "y", "edge": of }),
    ];
    Program {
        format: program::FORMAT.to_owned(),
        version: program::VERSION,
        id: "test_mount".to_owned(),
        meta: Some(
            json!({ "calibration": { "version": 2, "frame": "carriage", "readings": readings } }),
        ),
        board: Board {
            width_mm: 150.0,
            height_mm: 130.0,
            thickness_mm: 1.5,
        },
        material: Material {
            name: "MSK_1.5".to_owned(),
            speed_pct: 95.0,
            head_settle_ms: 250,
            blade_angle_offset_deg: BladeOffsets::default(),
        },
        ops,
    }
}

/// Stage 1 on a 150 × 150 offcut: a 100 × 100 mount with a 50 × 50 bevel window, its outline
/// cut anticlockwise (the mount on the blade's left all round, as a kept piece is cut), 25 mm in
/// from the front and left edges, clear of the clamp pads. The mount's size reads the straight
/// slit's left side, the frame's opening (inside jaws, once the mount is out) its right side, and
/// the borders the bevel head against the straight with the slit's lean cancelled.
fn stage_1() -> Program {
    let (s, b) = (Head::Straight, Head::Bevel);
    let ([x0, x1, y0, y1], o) = ([25.0, 125.0, 25.0, 125.0], 3.0);
    let ([wx0, wx1, wy0, wy1], w) = ([50.0, 100.0, 50.0, 100.0], 4.5);
    let cut = |id: &str, head, a: [f64; 2], z: [f64; 2]| {
        Op::Cut(Cut {
            id: Some(id.into()),
            label: None,
            head,
            path_mm: vec![a, z],
            speed_pct: None,
        })
    };
    let ops = vec![
        Op::Clamp(Clamp {
            id: None,
            label: None,
            side: ClampSide::Left,
            turned: Vec::new(),
        }),
        cut("W.right", b, [wx1, wy1 + w], [wx1, wy0 - w]),
        cut("W.front", b, [wx1 + w, wy0], [wx0 - w, wy0]),
        cut("W.left", b, [wx0, wy0 - w], [wx0, wy1 + w]),
        cut("W.back", b, [wx0 - w, wy1], [wx1 + w, wy1]),
        // Anticlockwise: front +X, right +Y, back −X, left −Y; the mount always on the left.
        cut("O.front", s, [x0 - o, y0], [x1 + o, y0]),
        cut("O.right", s, [x1, y0 - o], [x1, y1 + o]),
        cut("O.back", s, [x1 + o, y1], [x0 - o, y1]),
        cut("O.left", s, [x0, y1 + o], [x0, y0 - o]),
    ];
    let m = |c: &str| edge(c, "left", "any");
    let f = |c: &str| edge(c, "right", "any");
    let w = |c: &str| edge(c, "left", "back");
    let d = |c: &str| edge(c, "right", "face");
    let readings = vec![
        width("B.left", "x", m("O.left"), w("W.left")),
        width("B.right", "x", w("W.right"), m("O.right")),
        width("B.front", "y", m("O.front"), w("W.front")),
        width("B.back", "y", w("W.back"), m("O.back")),
        width("M.width", "x", m("O.left"), m("O.right")),
        width("M.height", "y", m("O.front"), m("O.back")),
        width("W.width", "x", w("W.left"), w("W.right")),
        width("W.height", "y", w("W.front"), w("W.back")),
        width("F.width", "x", f("O.left"), f("O.right")),
        width("F.height", "y", f("O.front"), f("O.back")),
        width("D.width", "x", d("W.left"), d("W.right")),
        width("D.height", "y", d("W.front"), d("W.back")),
        json!({ "name": "S.left", "kind": "position", "axis": "x", "edge": f("O.left") }),
        json!({ "name": "S.front", "kind": "position", "axis": "y", "edge": f("O.front") }),
        // The same faces read again, near the frame's other end: a misread is caught, and an
        // offcut whose factory edge isn't straight shows.
        json!({ "name": "S.left2", "kind": "position", "axis": "x", "edge": f("O.left") }),
        json!({ "name": "S.front2", "kind": "position", "axis": "y", "edge": f("O.front") }),
    ];
    Program {
        format: program::FORMAT.to_owned(),
        version: program::VERSION,
        id: "stage_1".to_owned(),
        meta: Some(
            json!({ "calibration": { "version": 2, "frame": "carriage", "readings": readings } }),
        ),
        board: Board {
            width_mm: 150.0,
            height_mm: 150.0,
            thickness_mm: 1.5,
        },
        material: Material {
            name: "MSK_1.5".to_owned(),
            speed_pct: 95.0,
            head_settle_ms: 250,
            blade_angle_offset_deg: BladeOffsets::default(),
        },
        ops,
    }
}

fn plan(program: &Program, names: &[&str], priors: &[Prior]) -> Precision {
    let profile = Profile::gunnar_601();
    let readings: HashMap<String, f64> = names.iter().map(|n| ((*n).to_owned(), 0.0)).collect();
    let sheet = Sheet::lowered("plan", program.clone(), &profile.raw(), readings).expect("lowers");
    precision(&sheet, priors).expect("a plan")
}

fn main() {
    let (s, b) = (Head::Straight, Head::Bevel);
    let p = |h, a| Unknown::P(h, a);
    let es = |right| Unknown::E(s, right, calibrate::Face::Any);
    let eb = Unknown::E(b, false, calibrate::Face::Back);
    // What a mount depends on: the window's place on it (across and along), the window's size (the bevel's back edge), the outline's size (the
    // straight slit's two sides together); and, only for a piece cut to the stock edge, where
    // the straight head is on the table.
    //
    // A border reads the bevel head against the straight together with the straight slit's
    // lean (its two sides' difference), the outline's strokes running +X and +Y as every
    // mount's do: across, (left − right) / 2 = P_s − P_b + (e_l − e_r) / 2; along, the lean's
    // sign turns. That is what places a window on a mount, so it is what is scored.
    let ef = Unknown::E(b, true, calibrate::Face::Front);
    let wanted: [(&str, Vec<(Unknown, f64)>); 7] = [
        (
            "window across",
            vec![
                (p(b, 0), 1.0),
                (p(s, 0), -1.0),
                (es(true), 0.5),
                (es(false), -0.5),
            ],
        ),
        (
            "window along",
            vec![
                (p(b, 1), 1.0),
                (p(s, 1), -1.0),
                (es(true), -0.5),
                (es(false), 0.5),
            ],
        ),
        ("bevel back", vec![(eb, 1.0)]),
        ("bevel face", vec![(ef, 1.0)]),
        ("straight slit", vec![(es(false), 1.0), (es(true), 1.0)]),
        // A piece's edge cut along the stock edge, +Y with the piece on its right, or +X with it
        // on its left: where the straight head is on the table, as it cuts.
        ("stock edge X", vec![(p(s, 0), 1.0), (es(true), -1.0)]),
        ("stock edge Y", vec![(p(s, 1), 1.0), (es(false), -1.0)]),
    ];
    let sheet2 = calibrate::sheet::program(&Profile::gunnar_601());
    let mount = test_mount(false);
    let mixed = test_mount(true);
    let known = |se| {
        vec![
            Prior {
                unknown: eb,
                value: 0.45,
                se_mm: se,
            },
            Prior {
                unknown: es(false),
                value: -0.10,
                se_mm: se,
            },
            Prior {
                unknown: es(true),
                value: 0.09,
                se_mm: se,
            },
        ]
    };
    let borders = ["B.left", "B.right", "B.front", "B.back"];
    let with = |more: &[&'static str]| -> Vec<&'static str> {
        borders.iter().chain(more).copied().collect()
    };
    let designs: Vec<(&str, &Program, Vec<&str>, Vec<Prior>)> = vec![
        ("sheet 2, as now", &sheet2, SHEET_2.to_vec(), vec![]),
        (
            "mount: borders, outline",
            &mount,
            with(&["O.width", "O.height"]),
            vec![],
        ),
        (
            "mount: + window",
            &mount,
            with(&["O.width", "O.height", "W.width", "W.height"]),
            vec![],
        ),
        (
            "mount: new machine",
            &mount,
            with(&[
                "O.width", "O.height", "W.width", "W.height", "D.width", "D.height", "S.left",
                "S.front",
            ]),
            vec![],
        ),
        (
            "mount: new profile",
            &mount,
            vec!["W.width", "W.height", "D.width", "D.height"],
            vec![],
        ),
        (
            "check: borders, edges known",
            &mount,
            borders.to_vec(),
            known(0.03),
        ),
        (
            "mixed window: + window",
            &mixed,
            with(&["O.width", "O.height", "W.width", "W.height"]),
            vec![],
        ),
        (
            "mixed: borders, edges known",
            &mixed,
            borders.to_vec(),
            known(0.03),
        ),
    ];
    println!("standard errors, mm, for caliper readings scattering 0.11 mm (calibration 1's)\n");
    print!("{:28} {:>3} {:>3}", "plan", "n", "unc");
    for (w, _) in &wanted {
        print!(" {w:>13}");
    }
    println!();
    for (label, program, names, priors) in designs {
        let pr = plan(program, &names, &priors);
        print!("{label:28} {:>3} {:>3}", pr.rows, pr.unchecked.len());
        for (_, c) in &wanted {
            let se = pr
                .combination(c)
                .map_or_else(|| "–".to_owned(), |v| format!("{v:.3}"));
            print!(" {se:>13}");
        }
        println!();
        if !pr.unchecked.is_empty() {
            println!("{:36}nothing checks: {}", "", pr.unchecked.join(", "));
        }
    }

    // Each value on its own, as calibration sheet 2 fixes them: on 150 × 150 offcuts.
    let single: [(&str, Unknown); 8] = [
        ("straight X", p(s, 0)),
        ("straight Y", p(s, 1)),
        ("bevel X", p(b, 0)),
        ("bevel Y", p(b, 1)),
        ("slit left", es(false)),
        ("slit right", es(true)),
        ("bevel back", eb),
        ("bevel face", ef),
    ];
    let stage = stage_1();
    let all14 = [
        "B.left", "B.right", "B.front", "B.back", "M.width", "M.height", "W.width", "W.height",
        "F.width", "F.height", "D.width", "D.height", "S.left", "S.front",
    ];
    let rows: Vec<(&str, &Program, Vec<&str>)> = vec![
        ("sheet 2, as now", &sheet2, SHEET_2.to_vec()),
        ("stage 1, all 14", &stage, all14.to_vec()),
        (
            "stage 1, no face (12)",
            &stage,
            all14
                .iter()
                .copied()
                .filter(|n| !n.starts_with('D'))
                .collect(),
        ),
        (
            "stage 1, stock read twice",
            &stage,
            all14
                .iter()
                .copied()
                .chain(["S.left2", "S.front2"])
                .collect(),
        ),
    ];
    println!("\neach value on its own, mm\n");
    print!("{:28} {:>3} {:>3}", "plan", "n", "unc");
    for (w, _) in &single {
        print!(" {w:>11}");
    }
    print!(" {:>13} {:>13}", "window across", "window along");
    println!();
    for (label, program, names) in rows {
        let pr = plan(program, &names, &[]);
        print!("{label:28} {:>3} {:>3}", pr.rows, pr.unchecked.len());
        for (_, u) in &single {
            let se = pr
                .get(*u)
                .map_or_else(|| "–".to_owned(), |v| format!("{v:.3}"));
            print!(" {se:>11}");
        }
        for (_, c) in &wanted[..2] {
            let se = pr
                .combination(c)
                .map_or_else(|| "–".to_owned(), |v| format!("{v:.3}"));
            print!(" {se:>13}");
        }
        println!();
        if !pr.unchecked.is_empty() {
            println!("{:36}nothing checks: {}", "", pr.unchecked.join(", "));
        }
    }
}
