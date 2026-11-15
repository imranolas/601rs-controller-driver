//! Calibration sheet 2 (`testdata/calibration/calibration_2.html`): a straight strip block, a
//! bevel window and four overcut combs on a small offcut, in carriage coordinates, cut raw
//! ([`machine::Profile::raw`]). The profile only places the features roughly on the board; the
//! fit never uses it. Its readings are listed in the program's `meta.calibration.readings`.

use machine::{MachineMm, Profile};
use program::{BladeOffsets, Board, Clamp, ClampSide, Cut, Head, Material, Op, Program};
use serde_json::{Value, json};

/// The straight strip block: cuts along Y at these X (A down, B up, C down), across at Y.
const STRIP_X: [f64; 3] = [28.0, 38.0, 48.0];
const STRIP_Y: [f64; 2] = [28.0, 53.0];
/// The bevel window, clockwise as windows are cut: X then Y extents.
const WINDOW: [f64; 4] = [68.0, 93.0, 28.0, 53.0];
const STRAIGHT_OVERCUT: f64 = 3.0;
const BEVEL_OVERCUT: f64 = 4.5;
/// The combs' teeth, least overcut first: how far each runs past (end) or starts before
/// (start, negative: after) its reference line, mm. Centred on calibration 1's predictions.
const S_END: [f64; 7] = [0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4];
const S_START: [f64; 7] = [-2.7, -2.4, -2.1, -1.8, -1.5, -1.2, -0.9];
const B_END: [f64; 7] = [1.6, 1.9, 2.2, 2.5, 2.8, 3.1, 3.4];
const B_START: [f64; 7] = [-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9];
/// Calibration 1's bevel edges, for the sheet's "about" values: the back edge outside the
/// line, and the face edge.
const BEVEL_BACK: f64 = 0.45;
const BEVEL_FACE: f64 = 2.16;

/// A cut on the board, from `a` to `b`, mm from the stops.
struct Planned {
    id: String,
    head: Head,
    a: [f64; 2],
    b: [f64; 2],
}

fn planned(id: &str, head: Head, a: [f64; 2], b: [f64; 2]) -> Planned {
    Planned {
        id: id.to_owned(),
        head,
        a,
        b,
    }
}

/// The sheet's cuts on the board: bevel first, then straight, so the stream breaks once.
fn layout() -> Vec<Planned> {
    let (s, b) = (Head::Straight, Head::Bevel);
    let mut out = Vec::new();
    // The bevel window, clockwise: right down, front left, left up, back right.
    let [x0, x1, y0, y1] = WINDOW;
    let o = BEVEL_OVERCUT;
    out.push(planned("W.right", b, [x1, y1 + o], [x1, y0 - o]));
    out.push(planned("W.front", b, [x1 + o, y0], [x0 - o, y0]));
    out.push(planned("W.left", b, [x0, y0 - o], [x0, y1 + o]));
    out.push(planned("W.back", b, [x0 - o, y1], [x1 + o, y1]));
    combs(&mut out, b, 70.0, 128.0, 96.0, &B_END, &B_START, "B");
    // The straight strips: A down, B up, C down, F right across the front, K left across the
    // back. SL lies left of A and B, SR right of B and C.
    let o = STRAIGHT_OVERCUT;
    let [ya, yb] = STRIP_Y;
    for (x, id, down) in [
        (STRIP_X[0], "S.A", true),
        (STRIP_X[1], "S.B", false),
        (STRIP_X[2], "S.C", true),
    ] {
        let (from, to) = if down {
            (yb + o, ya - o)
        } else {
            (ya - o, yb + o)
        };
        out.push(planned(id, s, [x, from], [x, to]));
    }
    out.push(planned(
        "S.F",
        s,
        [STRIP_X[0] - o, ya],
        [STRIP_X[2] + o, ya],
    ));
    out.push(planned(
        "S.K",
        s,
        [STRIP_X[2] + o, yb],
        [STRIP_X[0] - o, yb],
    ));
    combs(&mut out, s, 26.0, 128.0, 56.0, &S_END, &S_START, "S");
    out
}

/// An end comb (reference along X at Y 66, travelling left, teeth coming down past it) from
/// `end_x`, and a start comb (reference along Y at `ref_x`, travelling down from `top`, teeth
/// starting at or after it and running left).
#[allow(clippy::too_many_arguments)]
fn combs(
    out: &mut Vec<Planned>,
    head: Head,
    end_x: f64,
    ref_x: f64,
    top: f64,
    ends: &[f64; 7],
    starts: &[f64; 7],
    tag: &str,
) {
    let r = 66.0;
    out.push(planned(
        &format!("{tag}E.ref"),
        head,
        [end_x + 36.0, r],
        [end_x, r],
    ));
    for (i, e) in ends.iter().enumerate() {
        let x = end_x + 4.0 + 5.0 * i as f64;
        out.push(planned(
            &format!("{tag}E.{}", i + 1),
            head,
            [x, r + 10.0],
            [x, r - e],
        ));
    }
    out.push(planned(
        &format!("{tag}S.ref"),
        head,
        [ref_x, top],
        [ref_x, top - 30.0],
    ));
    for (i, s) in starts.iter().enumerate() {
        let y = top - 26.0 + 4.0 * i as f64;
        out.push(planned(
            &format!("{tag}S.{}", i + 1),
            head,
            [ref_x + s, y],
            [ref_x + s - 8.0, y],
        ));
    }
}

fn edge(cut: &str, kept: &str, face: &str) -> Value {
    json!({ "cut": cut, "kept": kept, "face": face })
}

/// What to read off the sheet, and roughly what each should be.
fn readings() -> Vec<Value> {
    let (wx0, wx1, wy0, wy1) = (WINDOW[0], WINDOW[1], WINDOW[2], WINDOW[3]);
    let r = |v: f64| (v * 100.0).round() / 100.0;
    vec![
        json!({"name": "SL.width", "kind": "width", "axis": "x", "about": 10.0,
               "what": "straight strip SL, across",
               "edges": [edge("S.A", "left", "any"), edge("S.B", "left", "any")]}),
        json!({"name": "SR.width", "kind": "width", "axis": "x", "about": 10.0,
               "what": "straight strip SR, across",
               "edges": [edge("S.B", "right", "any"), edge("S.C", "right", "any")]}),
        json!({"name": "SL.length", "kind": "width", "axis": "y", "about": 25.0,
               "what": "straight strip SL, end to end",
               "edges": [edge("S.F", "left", "any"), edge("S.K", "left", "any")]}),
        json!({"name": "S.left", "kind": "position", "axis": "x", "about": STRIP_X[0],
               "what": "left edge to the straight hole's left edge", "edge": edge("S.A", "right", "any")}),
        json!({"name": "S.front", "kind": "position", "axis": "y", "about": STRIP_Y[0],
               "what": "front edge to the straight hole's front edge", "edge": edge("S.F", "right", "any")}),
        json!({"name": "W.left", "kind": "position", "axis": "x", "about": r(wx0 - BEVEL_BACK),
               "what": "left edge to the window's left edge, on the back", "edge": edge("W.left", "left", "back")}),
        json!({"name": "W.width", "kind": "width", "axis": "x", "about": r(wx1 - wx0 + 2.0 * BEVEL_BACK),
               "what": "the window, left to right, on the back",
               "edges": [edge("W.left", "left", "back"), edge("W.right", "left", "back")]}),
        json!({"name": "W.front", "kind": "position", "axis": "y", "about": r(wy0 - BEVEL_BACK),
               "what": "front edge to the window's front edge, on the back", "edge": edge("W.front", "left", "back")}),
        json!({"name": "W.height", "kind": "width", "axis": "y", "about": r(wy1 - wy0 + 2.0 * BEVEL_BACK),
               "what": "the window, front to back, on the back",
               "edges": [edge("W.front", "left", "back"), edge("W.back", "left", "back")]}),
        json!({"name": "D.width", "kind": "width", "axis": "x", "about": r(wx1 - wx0 + 2.0 * BEVEL_FACE),
               "what": "the loose drop-out across, face up",
               "edges": [edge("W.left", "right", "face"), edge("W.right", "right", "face")]}),
        json!({"name": "D.height", "kind": "width", "axis": "y", "about": r(wy1 - wy0 + 2.0 * BEVEL_FACE),
               "what": "the loose drop-out the other way, face up",
               "edges": [edge("W.front", "right", "face"), edge("W.back", "right", "face")]}),
        json!({"name": "S.end", "kind": "comb", "head": "straight", "at": "end", "values": S_END,
               "what": "straight end comb: the first tooth whose slit reaches the reference, on the face"}),
        json!({"name": "S.start", "kind": "comb", "head": "straight", "at": "start", "values": S_START,
               "what": "straight start comb: the first tooth whose slit reaches the reference, on the face"}),
        json!({"name": "B.end", "kind": "comb", "head": "bevel", "at": "end", "values": B_END,
               "what": "bevel end comb: the first tooth whose slit reaches the reference, on the face"}),
        json!({"name": "B.start", "kind": "comb", "head": "bevel", "at": "start", "values": B_START,
               "what": "bevel start comb: the first tooth whose slit reaches the reference, on the face"}),
    ]
}

/// The sheet as a program in carriage coordinates, placed with `profile`'s idea of where the
/// heads are.
pub fn program(profile: &Profile) -> Program {
    let MachineMm([ox, oy]) = profile.board_origin_mm;
    let round = |v: f64| (v * 1000.0).round() / 1000.0;
    let mut ops = vec![Op::Clamp(Clamp {
        id: None,
        label: None,
        side: ClampSide::Left,
        turned: Vec::new(),
    })];
    for p in layout() {
        let [dx, dy] = profile.head(p.head).offset_mm;
        let at = |q: [f64; 2]| [round(q[0] + ox - dx), round(q[1] + oy - dy)];
        ops.push(Op::Cut(Cut {
            id: Some(p.id),
            label: None,
            head: p.head,
            path_mm: vec![at(p.a), at(p.b)],
            speed_pct: None,
        }));
    }
    Program {
        format: program::FORMAT.to_owned(),
        version: program::VERSION,
        id: "calibration_2".to_owned(),
        meta: Some(json!({
            "source": "mc601 calibrate sheet: the machine-and-head calibration sheet, in carriage \
                       millimetres (cut raw). Features sit roughly where drawn in \
                       testdata/calibration/calibration_2.html, placed with the profile's head \
                       offsets; the fit reads the exact steps from the run log.",
            "calibration": {"version": 2, "frame": "carriage", "readings": readings()},
        })),
        board: Board {
            width_mm: 140.0,
            height_mm: 105.0,
            thickness_mm: 1.5,
        },
        material: Material {
            name: "MSK_1.5".to_owned(),
            speed_pct: 95.0,
            head_settle_ms: 250,
            blade_angle_offset_deg: BladeOffsets {
                straight: 0.0,
                bevel: 0.0,
                bevel_round: None,
            },
        },
        ops,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Placed with the profile's offsets, every cut clears the clamp border, and the sheet
    /// lowers raw.
    #[test]
    fn the_sheet_lowers_raw_and_clears_the_clamps() {
        let profile = Profile::gunnar_601();
        let p = program(&profile);
        machine::lower(&p, &profile.raw()).expect("lowers raw");
        let MachineMm([ox, oy]) = profile.board_origin_mm;
        for o in &p.ops {
            if let Op::Cut(c) = o {
                let [dx, dy] = profile.head(c.head).offset_mm;
                for q in &c.path_mm {
                    let (x, y) = (q[0] + dx - ox, q[1] + dy - oy);
                    assert!(x >= 21.0 && y >= 21.0, "{:?} at {x}, {y}", c.id);
                }
            }
        }
    }
}
