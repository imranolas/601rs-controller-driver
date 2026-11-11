//! `mc601 calibrate`: the machine-and-head calibration, from raw cuts.
//!
//! - `sheet FILE` writes the calibration sheet (`testdata/calibration/calibration_2.json`): a
//!   straight strip block, a bevel window and four overcut combs on a small offcut, in carriage
//!   coordinates, cut raw by `mc601 run` ([`machine::Profile::raw`]). The profile only places
//!   the features roughly on the board; the fit never uses it.
//! - `fit PROGRAM LOG READINGS [PROGRAM LOG READINGS ...]` reads each cut's carriage steps from
//!   the run log (what was sent, whatever profile it was cut with) and the caliper readings, and
//!   fits by least squares, pooling every sheet given. `--profile FILE` writes the profile with
//!   the fitted head offsets.
//!
//! The model, per head, in carriage millimetres:
//!
//! - `P`: where the carriage puts the head's line on the corner where boards register, so a cut
//!   commanded at carriage `c` runs along board `c − P`. One pair per head: only the sum of a
//!   board origin and a head offset can be seen in a cut, so it is fitted as one;
//! - `e`: where each side of the slit lands from the line, to the left and to the right of
//!   travel, positive towards that side; for the bevel, separately on the back (as cut) and on
//!   the face, which differ by the bevel's run;
//! - the drag at the start and at the end of a cut (calibration 1's overrun readings).
//!
//! An edge reading is `c − P + σe` (σ = ±1, the kept side along the axis), a width is two edges'
//! difference (`P` cancels), an overrun is `overcut ± drag − e`. The combs are read by eye: the
//! first tooth that reaches the reference gives an interval for the overcut a corner needs.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use link::Direction;
use machine::{MachineMm, Profile};
use program::{BladeOffsets, Board, Clamp, ClampSide, Cut, Head, Material, Op, Program};
use serde_json::{Value, json};

use crate::config::tidy;
use crate::operator::{NO_ANSWER, Operator};
use crate::session::Session;
use crate::style::{DIM, GOOD, WARN, paint};
use crate::target::Res;

// ------------------------------------------------------------------ the sheet

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
pub fn sheet_program(profile: &Profile) -> Program {
    let MachineMm([ox, oy]) = profile.board_origin_mm;
    let round = |v: f64| (v * 1000.0).round() / 1000.0;
    let mut ops = vec![Op::Clamp(Clamp {
        id: None,
        label: None,
        side: ClampSide::Left,
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

pub fn sheet(out: &Path) -> Res<bool> {
    let program = sheet_program(&Profile::gunnar_601());
    let text = serde_json::to_string_pretty(&program)? + "\n";
    std::fs::write(out, text).map_err(|e| format!("{}: {e}", out.display()))?;
    let cuts = program
        .ops
        .iter()
        .filter(|o| matches!(o, Op::Cut(_)))
        .count();
    println!(
        "{}: {cuts} cuts in carriage coordinates, cut raw by `mc601 run`",
        out.display()
    );
    Ok(true)
}

/// The sheet's file name in a session.
const SHEET_FILE: &str = "calibration_2.json";
const READINGS_FILE: &str = "readings.txt";

/// `mc601 calibrate cut`: write the sheet into the session, cut it raw, take its readings.
pub fn cut(port: &str, session: &Session) -> Res<bool> {
    let file = session.dir.join(SHEET_FILE);
    let program = sheet_program(&Profile::gunnar_601());
    std::fs::write(&file, serde_json::to_string_pretty(&program)? + "\n")
        .map_err(|e| format!("{}: {e}", file.display()))?;
    anstream::println!(
        "{}",
        paint(
            DIM,
            "the machine-and-head sheet: a 1.5 mm offcut of at least 140 × 105 mm; what to \
             measure: testdata/calibration/calibration_2.html"
        )
    );
    crate::cut::run(port, std::slice::from_ref(&file), session, false)
}

// ------------------------------------------------------------------ the run log

/// A cut as the log shows it was sent: its head, and its points in carriage mm.
#[derive(Debug, Clone, PartialEq)]
pub struct Logged {
    pub head: Head,
    pub points: Vec<[f64; 2]>,
}

/// The head whose blade outputs `bits` lower, if any: the straight head's `i_sh` (`08`), or the
/// bevel's `i_t`/`i_b` (`01`, `04`).
fn head_down(bits: u8) -> Option<Head> {
    if bits & 0x08 != 0 {
        Some(Head::Straight)
    } else if bits & 0x05 != 0 {
        Some(Head::Bevel)
    } else {
        None
    }
}

/// Apply a vector's axes (`X…,Y…` absolute, or `x…,y…` relative) to `pos`; whether X or Y was
/// among them.
fn apply(axes: &str, relative: bool, pos: &mut [i64; 2]) -> bool {
    let mut any = false;
    for tok in axes.split(',') {
        let (axis, v) = tok.split_at(tok.len().min(1));
        let Ok(v) = v.parse::<i64>() else { continue };
        let i = match axis {
            "X" | "x" => 0,
            "Y" | "y" => 1,
            _ => continue,
        };
        pos[i] = if relative { pos[i] + v } else { v };
        any = true;
    }
    any
}

/// The cuts a run log sent, in order: from each head-down output to the next output without
/// it, with the vectors between (absolute `L…`, or the compact steps `x…,y…`).
pub fn cuts_from_log(text: &str, profile: &Profile) -> Res<Vec<Logged>> {
    let mut sent = Vec::new();
    for e in text.lines().filter_map(link::tap::parse_line) {
        if e.direction == Direction::HostToController {
            sent.extend(e.bytes);
        }
    }
    let s = profile.scale;
    let mm = |p: [i64; 2]| {
        [
            p[0] as f64 / s.x_steps_per_mm,
            p[1] as f64 / s.y_steps_per_mm,
        ]
    };
    let mut pos = [0i64, 0];
    let mut cuts: Vec<Logged> = Vec::new();
    let mut open: Option<Logged> = None;
    for raw in sent.split(|&b| b == b'\r') {
        let cmd = String::from_utf8_lossy(raw).trim().to_owned();
        let outputs = cmd
            .strip_prefix("$A")
            .or_else(|| cmd.strip_prefix('A'))
            .and_then(|h| u8::from_str_radix(h, 16).ok());
        if let Some(bits) = outputs {
            let head = head_down(bits);
            if open.as_ref().map(|c| c.head) != head {
                if let Some(c) = open.take() {
                    if c.points.len() >= 2 {
                        cuts.push(c);
                    }
                }
                open = head.map(|head| Logged {
                    head,
                    points: vec![mm(pos)],
                });
            }
            continue;
        }
        let moved = if let Some(rest) = cmd.strip_prefix('L') {
            // `L20,X…,Y…,Z…`: skip the speed entry.
            rest.split_once(',')
                .is_some_and(|(_, axes)| apply(axes, false, &mut pos))
        } else if cmd.starts_with(['x', 'y', 'z']) {
            apply(&cmd, true, &mut pos)
        } else {
            false
        };
        if moved {
            if let Some(c) = open.as_mut() {
                c.points.push(mm(pos));
            }
        }
    }
    if let Some(c) = open.take() {
        if c.points.len() >= 2 {
            cuts.push(c);
        }
    }
    Ok(cuts)
}

// ------------------------------------------------------------------ the fit

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Face {
    Any,
    Back,
    Front,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Unknown {
    /// Head, axis: where the carriage puts the head's line on the registered corner.
    P(Head, usize),
    /// Head, kept side right?, face: where that side of the slit lands from the line.
    E(Head, bool, Face),
    /// Head, at the start?: the drag.
    Drag(Head, bool),
}

impl Unknown {
    fn head(&self) -> Head {
        match *self {
            Unknown::P(h, _) | Unknown::E(h, _, _) | Unknown::Drag(h, _) => h,
        }
    }
    fn order(&self) -> (u8, u8, u8, u8) {
        let h = u8::from(self.head() == Head::Bevel);
        match *self {
            Unknown::P(_, a) => (h, 0, a as u8, 0),
            Unknown::E(_, r, f) => (h, 1, u8::from(r), f as u8),
            Unknown::Drag(_, s) => (h, 2, u8::from(!s), 0),
        }
    }
    fn label(&self) -> String {
        match *self {
            Unknown::P(_, a) => format!("P{}", ["x", "y"][a]),
            Unknown::E(_, r, f) => {
                let side = if r { "right" } else { "left" };
                match f {
                    Face::Any => format!("e {side}"),
                    Face::Back => format!("e {side}, back"),
                    Face::Front => format!("e {side}, face"),
                }
            }
            Unknown::Drag(_, s) => format!("drag {}", if s { "start" } else { "end" }),
        }
    }
}

struct Row {
    name: String,
    terms: Vec<(Unknown, f64)>,
    rhs: f64,
}

/// A comb read by eye: the first tooth that reached (0 for none).
pub struct CombReading {
    name: String,
    head: Head,
    at_start: bool,
    tooth: usize,
    values: Vec<f64>,
}

fn head_of(s: &str) -> Res<Head> {
    match s {
        "bevel" => Ok(Head::Bevel),
        "straight" => Ok(Head::Straight),
        _ => Err(format!("unknown head {s:?}").into()),
    }
}

/// A cut as the fit sees it: head, start, end, unit direction (along X or Y).
struct Line {
    head: Head,
    a: [f64; 2],
    b: [f64; 2],
    d: [f64; 2],
}

impl Line {
    fn new(c: &Logged, what: &str) -> Res<Line> {
        let (a, b) = (c.points[0], c.points[c.points.len() - 1]);
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        let d = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
        if d[0].abs() > 1e-6 && d[1].abs() > 1e-6 {
            return Err(format!("{what}: calibration cuts run along X or Y").into());
        }
        Ok(Line {
            head: c.head,
            a,
            b,
            d,
        })
    }
    /// The kept side's direction along `axis`: ±1.
    fn sigma(&self, kept_right: bool, axis: usize) -> f64 {
        let dir = if kept_right {
            [self.d[1], -self.d[0]]
        } else {
            [-self.d[1], self.d[0]]
        };
        dir[axis].round()
    }
    fn e(&self, kept_right: bool, face: Face) -> Unknown {
        let face = if self.head == Head::Straight {
            Face::Any
        } else {
            face
        };
        Unknown::E(self.head, kept_right, face)
    }
}

fn face_of(v: Option<&str>) -> Face {
    match v {
        Some("face") => Face::Front,
        Some("back") => Face::Back,
        _ => Face::Any,
    }
}

fn text_of(v: &Value, k: &str) -> String {
    v.get(k)
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_owned()
}

/// One sheet's rows and comb readings.
fn sheet_rows(sheet: &Sheet) -> Res<(Vec<Row>, Vec<CombReading>)> {
    let (program, cuts, label) = (&sheet.program, &sheet.cuts, &sheet.label);
    let cal = program
        .meta
        .as_ref()
        .and_then(|m| m.get("calibration"))
        .ok_or_else(|| format!("{label}: not a calibration sheet (no meta.calibration)"))?;
    // The program's cuts in order, with their op indices, matched to the log's.
    let ops: Vec<(usize, &Cut)> = program
        .ops
        .iter()
        .enumerate()
        .filter_map(|(i, o)| match o {
            Op::Cut(c) => Some((i, c)),
            _ => None,
        })
        .collect();
    if ops.len() != cuts.len() {
        return Err(format!(
            "{label}: the log has {} cuts, the program {}: not this program's run, or it stopped",
            cuts.len(),
            ops.len()
        )
        .into());
    }
    for (k, ((_, c), l)) in ops.iter().zip(cuts).enumerate() {
        if c.head != l.head {
            return Err(format!("{label}: cut {} is on another head in the log", k + 1).into());
        }
    }
    // v2 names cuts by id; v1 (calibration 1) by op index.
    let line = |v: &Value, key: &str| -> Res<Line> {
        let k = match v.get(key) {
            Some(Value::String(id)) => ops.iter().position(|(_, c)| c.id.as_deref() == Some(id)),
            Some(Value::Number(n)) => ops.iter().position(|(i, _)| Some(*i as u64) == n.as_u64()),
            _ => None,
        }
        .ok_or_else(|| format!("{label}: no cut {:?}", v.get(key)))?;
        Line::new(&cuts[k], &format!("cut {k}"))
    };
    let (list, v2) = match (cal.get("readings"), cal.get("measurements")) {
        (Some(Value::Array(a)), _) => (a, true),
        (_, Some(Value::Array(a))) => (a, false),
        _ => return Err(format!("{label}: no readings listed").into()),
    };
    let names: Vec<String> = list.iter().map(|m| text_of(m, "name")).collect();
    if let Some(unknown) = sheet.readings.keys().find(|k| !names.contains(k)) {
        return Err(format!("{label}: {unknown} is not a reading of this sheet").into());
    }
    // An edge: its line, and the kept side's sigma and unknown.
    let edge = |e: &Value, axis: usize| -> Res<(Line, f64, Unknown)> {
        let l = line(e, "cut")?;
        let right = text_of(e, "kept") == "right";
        let sigma = l.sigma(right, axis);
        let u = l.e(right, face_of(e.get("face").and_then(Value::as_str)));
        Ok((l, sigma, u))
    };
    let mut rows = Vec::new();
    let mut combs = Vec::new();
    for m in list {
        let name = text_of(m, "name");
        let Some(&value) = sheet.readings.get(&name) else {
            continue;
        };
        let axis = usize::from(text_of(m, "axis") == "y");
        match text_of(m, "kind").as_str() {
            // v1 calls a position reading an edge, with the edge's fields inline.
            "position" | "edge" => {
                let spec = if v2 {
                    m.get("edge").unwrap_or(&Value::Null)
                } else {
                    m
                };
                let (l, sigma, u) = edge(spec, axis)?;
                rows.push(Row {
                    name,
                    terms: vec![(Unknown::P(l.head, axis), -1.0), (u, sigma)],
                    rhs: value - l.a[axis],
                });
            }
            "width" => {
                let es = m
                    .get("edges")
                    .and_then(Value::as_array)
                    .filter(|a| a.len() == 2)
                    .ok_or_else(|| format!("{name}: a width needs two edges"))?;
                let mut one = edge(&es[0], axis)?;
                let mut two = edge(&es[1], axis)?;
                if two.0.a[axis] < one.0.a[axis] {
                    std::mem::swap(&mut one, &mut two);
                }
                let mut terms = vec![(two.2, two.1), (one.2, -one.1)];
                if one.0.head != two.0.head {
                    terms.push((Unknown::P(two.0.head, axis), -1.0));
                    terms.push((Unknown::P(one.0.head, axis), 1.0));
                }
                rows.push(Row {
                    name,
                    terms,
                    rhs: value - (two.0.a[axis] - one.0.a[axis]),
                });
            }
            "overrun" => {
                let (l, c) = (line(m, "cut")?, line(m, "crossing")?);
                let along = usize::from(l.d[1].abs() > 0.5);
                let at = c.a[along];
                let start = text_of(m, "end") == "start";
                let overcut = if start {
                    (at - l.a[along]) * l.d[along]
                } else {
                    (l.b[along] - at) * l.d[along]
                };
                let right = text_of(m, "kept") == "right";
                let face = face_of(m.get("face").and_then(Value::as_str));
                rows.push(Row {
                    name,
                    terms: vec![
                        (Unknown::Drag(l.head, start), if start { 1.0 } else { -1.0 }),
                        (c.e(right, face), -1.0),
                    ],
                    rhs: value - overcut,
                });
            }
            "comb" => {
                let values: Vec<f64> = m
                    .get("values")
                    .and_then(Value::as_array)
                    .map(|a| a.iter().filter_map(Value::as_f64).collect())
                    .unwrap_or_default();
                let tooth = value.round() as usize;
                if value < 0.0 || tooth > values.len() || (value - value.round()).abs() > 1e-9 {
                    return Err(format!(
                        "{name}: give the tooth number, 1 to {} (0: none reached)",
                        values.len()
                    )
                    .into());
                }
                combs.push(CombReading {
                    name,
                    head: head_of(&text_of(m, "head"))?,
                    at_start: text_of(m, "at") == "start",
                    tooth,
                    values,
                });
            }
            k => return Err(format!("{name}: unknown kind {k:?}").into()),
        }
    }
    Ok((rows, combs))
}

/// Solve the normal equations by Gauss–Jordan with partial pivoting; also returns the inverse
/// (for the uncertainties). `None` for a column the readings don't determine.
fn solve(ata: &[Vec<f64>], atb: &[f64]) -> (Vec<Option<f64>>, Vec<Vec<f64>>) {
    let n = atb.len();
    let mut m: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = ata[i].clone();
            row.extend((0..n).map(|j| f64::from(u8::from(i == j))));
            row.push(atb[i]);
            row
        })
        .collect();
    let scale = (0..n).map(|i| ata[i][i].abs()).fold(0.0, f64::max).max(1.0);
    let mut solved = vec![true; n];
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))
            .unwrap_or(col);
        if m[pivot][col].abs() < 1e-9 * scale {
            solved[col] = false;
            continue;
        }
        m.swap(col, pivot);
        let p = m[col][col];
        for v in &mut m[col] {
            *v /= p;
        }
        for row in 0..n {
            if row != col {
                let f = m[row][col];
                if f != 0.0 {
                    for k in 0..m[row].len() {
                        m[row][k] -= f * m[col][k];
                    }
                }
            }
        }
    }
    let x = (0..n).map(|i| solved[i].then(|| m[i][2 * n])).collect();
    let inv = (0..n).map(|i| m[i][n..2 * n].to_vec()).collect();
    (x, inv)
}

/// The fitted values: each unknown with its standard error (`None`: not determined).
pub struct Fit {
    values: Vec<(Unknown, Option<f64>, f64)>,
    leftovers: Vec<(String, f64)>,
    sigma: f64,
    rows: usize,
}

impl Fit {
    fn get(&self, u: Unknown) -> Option<(f64, f64)> {
        self.values
            .iter()
            .find(|(k, _, _)| *k == u)
            .and_then(|(_, v, se)| v.map(|v| (v, *se)))
    }
}

fn fit_rows(rows: &[Row]) -> Fit {
    let mut cols: Vec<Unknown> = rows
        .iter()
        .flat_map(|r| r.terms.iter().map(|t| t.0))
        .collect();
    cols.sort_by_key(Unknown::order);
    cols.dedup();
    let (m, n) = (rows.len(), cols.len());
    let a: Vec<Vec<f64>> = rows
        .iter()
        .map(|r| {
            let mut v = vec![0.0; n];
            for &(k, c) in &r.terms {
                v[cols.iter().position(|x| *x == k).expect("a column")] += c;
            }
            v
        })
        .collect();
    let y: Vec<f64> = rows.iter().map(|r| r.rhs).collect();
    let ata: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| (0..m).map(|k| a[k][i] * a[k][j]).sum())
                .collect()
        })
        .collect();
    let atb: Vec<f64> = (0..n)
        .map(|i| (0..m).map(|k| a[k][i] * y[k]).sum())
        .collect();
    let (x, inv) = solve(&ata, &atb);
    let p: Vec<f64> = x.iter().map(|v| v.unwrap_or(0.0)).collect();
    let leftovers: Vec<(String, f64)> = (0..m)
        .map(|k| {
            let model: f64 = (0..n).map(|j| a[k][j] * p[j]).sum();
            (rows[k].name.clone(), y[k] - model)
        })
        .collect();
    let dof = m.saturating_sub(x.iter().filter(|v| v.is_some()).count());
    let sigma = if dof > 0 {
        (leftovers.iter().map(|(_, r)| r * r).sum::<f64>() / dof as f64).sqrt()
    } else {
        f64::NAN
    };
    let values = cols
        .iter()
        .enumerate()
        .map(|(j, &u)| (u, x[j], sigma * inv[j][j].max(0.0).sqrt()))
        .collect();
    Fit {
        values,
        leftovers,
        sigma,
        rows: m,
    }
}

fn read_readings(path: &Path) -> Res<HashMap<String, f64>> {
    let file = tidy(path);
    let text = std::fs::read_to_string(path).map_err(|e| format!("{file}: {e}"))?;
    let mut out = HashMap::new();
    for (i, l) in text.lines().enumerate() {
        let l = l.split('#').next().unwrap_or_default().trim();
        if l.is_empty() {
            continue;
        }
        let mut it = l.split_whitespace();
        let (Some(name), Some(v), None) = (it.next(), it.next(), it.next()) else {
            return Err(format!("{file}:{}: want `name value`", i + 1).into());
        };
        let v: f64 = v
            .parse()
            .map_err(|_| format!("{file}:{}: {v} is not a number", i + 1))?;
        if out.insert(name.to_owned(), v).is_some() {
            return Err(format!("{file}:{}: {name} given twice", i + 1).into());
        }
    }
    Ok(out)
}

/// One sheet's data: the program, the cuts its log shows were sent, its readings.
pub struct Sheet {
    pub label: String,
    pub program: Program,
    pub cuts: Vec<Logged>,
    pub readings: HashMap<String, f64>,
}

fn load_sheet(program: &Path, log: &Path, readings: &Path, profile: &Profile) -> Res<Sheet> {
    let text =
        std::fs::read_to_string(program).map_err(|e| format!("{}: {e}", program.display()))?;
    let parsed = Program::from_json(&text).map_err(|e| format!("{}: {e}", program.display()))?;
    let log_text = std::fs::read_to_string(log).map_err(|e| format!("{}: {e}", log.display()))?;
    Ok(Sheet {
        label: tidy(program),
        program: parsed,
        cuts: cuts_from_log(&log_text, profile)?,
        readings: read_readings(readings)?,
    })
}

/// A calibration session's sheet and log: the program in the folder that lists readings, and
/// the log `session.json` names (or the folder's own `.log`).
fn session_files(dir: &Path) -> Res<(PathBuf, Program, PathBuf)> {
    let entries = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut program = None;
    let mut logs = Vec::new();
    for e in entries.filter_map(Result::ok) {
        let path = e.path();
        match path.extension().and_then(|x| x.to_str()) {
            Some("json")
                if path
                    .file_name()
                    .is_some_and(|n| n != "session.json" && n != "profile.json") =>
            {
                let Ok(text) = std::fs::read_to_string(&path) else {
                    continue;
                };
                if let Ok(p) = Program::from_json(&text) {
                    if p.meta.as_ref().and_then(|m| m.get("calibration")).is_some() {
                        program = Some((path, p));
                    }
                }
            }
            Some("log") => logs.push(path),
            _ => {}
        }
    }
    let (path, program) =
        program.ok_or_else(|| format!("{}: no calibration sheet in this session", tidy(dir)))?;
    let named = std::fs::read_to_string(dir.join("session.json"))
        .ok()
        .and_then(|t| serde_json::from_str::<Value>(&t).ok())
        .and_then(|v| v.get("log").and_then(Value::as_str).map(PathBuf::from))
        .filter(|p| p.exists());
    let log = named
        .or_else(|| logs.into_iter().next())
        .ok_or_else(|| format!("{}: no run log in this session", tidy(dir)))?;
    Ok((path, program, log))
}

/// Session folders under `data`, newest first, that hold a calibration sheet.
fn calibration_sessions(data: &Path) -> Vec<PathBuf> {
    crate::session::list(data)
        .into_iter()
        .map(|s| s.dir)
        .filter(|d| session_files(d).is_ok())
        .collect()
}

/// The newest calibration session.
pub fn latest_session(data: &Path) -> Res<PathBuf> {
    calibration_sessions(data)
        .into_iter()
        .next()
        .ok_or_else(|| {
            format!(
                "no calibration session in {}: cut one with `mc601 calibrate cut`",
                tidy(data)
            )
            .into()
        })
}

/// Calibration sessions that have readings, for `fit --all`.
pub fn sessions_with_readings(data: &Path) -> Vec<PathBuf> {
    calibration_sessions(data)
        .into_iter()
        .filter(|d| d.join(READINGS_FILE).exists())
        .collect()
}

/// Which face a reading is taken on, from its edges.
fn face_note(m: &Value) -> &'static str {
    let faces: Vec<String> = m
        .get("edges")
        .and_then(Value::as_array)
        .cloned()
        .or_else(|| m.get("edge").cloned().map(|e| vec![e]))
        .unwrap_or_default()
        .iter()
        .map(|e| text_of(e, "face"))
        .collect();
    if text_of(m, "kind") == "comb" {
        "by eye, face up"
    } else if faces.iter().all(|f| f == "back") && !faces.is_empty() {
        "on the back"
    } else if faces.iter().all(|f| f == "face") && !faces.is_empty() {
        "face up"
    } else {
        "either face"
    }
}

/// How far a reading may stray from its expected value before it is questioned, mm.
const NEAR_MM: f64 = 1.5;

/// `mc601 calibrate read`: take a session's readings one by one (each saved as it is given),
/// then fit them and keep the fit and a fitted profile in the session.
pub fn read(dir: &Path, op: &Operator) -> Res<bool> {
    let (_, program, log) = session_files(dir)?;
    let list: Vec<Value> = program
        .meta
        .as_ref()
        .and_then(|m| {
            m.pointer("/calibration/readings")
                .or_else(|| m.pointer("/calibration/measurements"))
        })
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    let file = dir.join(READINGS_FILE);
    let mut readings = if file.exists() {
        read_readings(&file)?
    } else {
        HashMap::new()
    };
    anstream::println!(
        "\n{} {} readings. Type each in mm (a comb: the first tooth that reached, 0 for none). \
         Enter alone keeps the value shown or skips it; q stops, keeping what is typed.",
        paint(crate::style::BOLD, format!("Readings for {}:", program.id)),
        list.len()
    );
    let save = |readings: &HashMap<String, f64>| -> Res<()> {
        let mut text = format!(
            "# {} readings, {} (mc601 calibrate read)\n",
            program.id,
            chrono::Local::now().format("%Y-%m-%d %H:%M")
        );
        for m in &list {
            let name = text_of(m, "name");
            if let Some(v) = readings.get(&name) {
                let _ = writeln!(text, "{name} {v}");
            }
        }
        std::fs::write(&file, text).map_err(|e| format!("{}: {e}", file.display()))?;
        Ok(())
    };
    'readings: for (i, m) in list.iter().enumerate() {
        let name = text_of(m, "name");
        let comb = text_of(m, "kind") == "comb";
        let about = m.get("about").and_then(Value::as_f64);
        let teeth = m
            .get("values")
            .and_then(Value::as_array)
            .map_or(0, Vec::len);
        loop {
            let shown = readings
                .get(&name)
                .map_or_else(String::new, |v| format!(" [{v}]"));
            let hint = if comb {
                format!("tooth 1 to {teeth}, 0: none")
            } else {
                about.map_or_else(String::new, |a| format!("about {a}"))
            };
            let question = format!(
                "[{:>2}/{}] {name:10} {}  {}{shown} >",
                i + 1,
                list.len(),
                text_of(m, "what"),
                paint(DIM, format!("{} · {hint}", face_note(m)))
            );
            let Some(answer) = op.ask(&question) else {
                break 'readings;
            };
            let answer = answer.trim();
            if answer == "q" || answer == NO_ANSWER {
                break 'readings;
            }
            if answer.is_empty() {
                break;
            }
            let Ok(v) = answer.parse::<f64>() else {
                anstream::println!("  {}", paint(WARN, format!("{answer:?} is not a number")));
                continue;
            };
            if comb && (v < 0.0 || v > teeth as f64 || v.fract() != 0.0) {
                anstream::println!(
                    "  {}",
                    paint(WARN, format!("give a tooth number, 0 to {teeth}"))
                );
                continue;
            }
            if let (false, Some(a)) = (comb, about) {
                if (v - a).abs() > NEAR_MM {
                    let keep = op.ask(&format!(
                        "{} {v} is {:.1} mm from the {a} expected. Keep it? (y/n)",
                        paint(WARN, "check:"),
                        (v - a).abs()
                    ));
                    if keep.as_deref() != Some("y") {
                        continue;
                    }
                }
            }
            readings.insert(name.clone(), v);
            save(&readings)?;
            break;
        }
    }
    save(&readings)?;
    anstream::println!("{}", paint(DIM, format!("readings: {}", tidy(&file))));
    if readings.is_empty() {
        return Ok(true);
    }
    let profile = Profile::gunnar_601();
    let text = std::fs::read_to_string(&log).map_err(|e| format!("{}: {e}", log.display()))?;
    let thickness = program.board.thickness_mm;
    let sheet = Sheet {
        label: tidy(dir),
        program,
        cuts: cuts_from_log(&text, &profile)?,
        readings,
    };
    let (fit, combs) = fit_sheets(&[sheet])?;
    let (out, fitted) = report(&fit, &combs, thickness, &profile);
    anstream::print!("\n{out}");
    keep(dir, &out, &fitted)?;
    Ok(true)
}

/// Keep a fit and its profile in `dir`.
fn keep(dir: &Path, report: &str, fitted: &Profile) -> Res<()> {
    let plain = anstream::adapter::strip_str(report).to_string();
    std::fs::write(dir.join("fit.txt"), plain).map_err(|e| format!("{}: {e}", dir.display()))?;
    std::fs::write(
        dir.join("profile.json"),
        serde_json::to_string_pretty(fitted)? + "\n",
    )
    .map_err(|e| format!("{}: {e}", dir.display()))?;
    anstream::println!(
        "{}",
        paint(
            DIM,
            format!("fit and fitted profile: {}", tidy(&dir.join("fit.txt")))
        )
    );
    Ok(())
}

/// Fit every sheet's readings together.
pub fn fit_sheets(sheets: &[Sheet]) -> Res<(Fit, Vec<CombReading>)> {
    let thick = sheets[0].program.board.thickness_mm;
    if sheets
        .iter()
        .any(|s| (s.program.board.thickness_mm - thick).abs() > 1e-9)
    {
        return Err(
            "the sheets are on boards of different thickness, where the bevel's face \
                    edge differs: fit them separately"
                .into(),
        );
    }
    let mut rows = Vec::new();
    let mut combs = Vec::new();
    for s in sheets {
        let (r, c) = sheet_rows(s)?;
        rows.extend(r);
        combs.extend(c);
    }
    Ok((fit_rows(&rows), combs))
}

/// `mc601 calibrate fit`: fit session folders and loose sheets together.
pub fn fit(dirs: &[PathBuf], sheets: &[[PathBuf; 3]], profile_out: Option<&Path>) -> Res<bool> {
    let profile = Profile::gunnar_601();
    let mut all = Vec::new();
    for d in dirs {
        let (path, _, log) = session_files(d)?;
        let readings = d.join(READINGS_FILE);
        if !readings.exists() {
            return Err(format!(
                "{}: no readings yet: take them with `mc601 calibrate read`",
                tidy(d)
            )
            .into());
        }
        let mut s = load_sheet(&path, &log, &readings, &profile)?;
        s.label = tidy(d);
        all.push(s);
    }
    for [p, l, r] in sheets {
        all.push(load_sheet(p, l, r, &profile)?);
    }
    let (fit, combs) = fit_sheets(&all)?;
    let thickness = all[0].program.board.thickness_mm;
    let (out, fitted) = report(&fit, &combs, thickness, &profile);
    anstream::println!(
        "{}",
        paint(
            DIM,
            format!(
                "{} sheet(s): {}",
                all.len(),
                all.iter()
                    .map(|s| s.label.as_str())
                    .collect::<Vec<_>>()
                    .join("; ")
            )
        )
    );
    anstream::print!("{out}");
    if let Some(file) = profile_out {
        std::fs::write(file, serde_json::to_string_pretty(&fitted)? + "\n")
            .map_err(|e| format!("{}: {e}", file.display()))?;
        println!(
            "\nthe profile with the fitted head offsets: {}",
            file.display()
        );
    }
    Ok(true)
}

fn head_name(h: Head) -> &'static str {
    match h {
        Head::Bevel => "bevel",
        Head::Straight => "straight",
    }
}

/// The fit, as text for the terminal (styled; strip it for a file), and the profile with the
/// fitted head offsets.
fn report(
    fit: &Fit,
    combs: &[CombReading],
    thickness: f64,
    profile: &Profile,
) -> (String, Profile) {
    let mut o = String::new();
    let n = fit.values.iter().filter(|v| v.1.is_some()).count();
    let scatter = if fit.sigma.is_nan() {
        "–".to_owned()
    } else {
        format!("{:.3} mm", fit.sigma)
    };
    let _ = writeln!(
        o,
        "{} readings, {n} unknowns; the readings scatter {scatter} about the model",
        fit.rows
    );
    let MachineMm([ox, oy]) = profile.board_origin_mm;
    let mut fitted = profile.clone();
    for h in [Head::Straight, Head::Bevel] {
        if !fit.values.iter().any(|(u, _, _)| u.head() == h) {
            continue;
        }
        let _ = writeln!(
            o,
            "\n{}",
            paint(crate::style::BOLD, format!("{} head", head_name(h)))
        );
        for (u, v, se) in fit.values.iter().filter(|(u, _, _)| u.head() == h) {
            let _ = match v {
                Some(v) => writeln!(
                    o,
                    "  {:16} {v:+9.3} mm  {}",
                    u.label(),
                    paint(DIM, format!("± {se:.3}"))
                ),
                None => writeln!(
                    o,
                    "  {:16} {}",
                    u.label(),
                    paint(WARN, "not determined by these readings")
                ),
            };
        }
        let g = |u| fit.get(u);
        if h == Head::Straight {
            if let (Some((l, _)), Some((r, _))) = (
                g(Unknown::E(h, false, Face::Any)),
                g(Unknown::E(h, true, Face::Any)),
            ) {
                let _ = writeln!(
                    o,
                    "  slit {:.3} mm wide, its middle {:+.3} mm right of the line",
                    l + r,
                    (r - l) / 2.0
                );
            }
        } else if let (Some((back, _)), Some((face, _))) = (
            g(Unknown::E(h, false, Face::Back)),
            g(Unknown::E(h, true, Face::Front)),
        ) {
            // A window's mat edge lies `back` outside its line on the back; on the face, where
            // the drop-out's face edge is, `-face` outside it.
            let run = -face - back;
            let _ = writeln!(
                o,
                "  a window's edge {back:+.3} mm outside its line on the back, {:+.3} on the face: \
                 a bevel of {run:.2} mm across {thickness} mm board, {:.1}° from vertical",
                -face,
                (run / thickness).atan().to_degrees()
            );
        }
        if let (Some((px, sx)), Some((py, sy))) = (g(Unknown::P(h, 0)), g(Unknown::P(h, 1))) {
            let now = profile.head(h).offset_mm;
            let new = [ox - px, oy - py];
            let note = |a: f64, b: f64, se: f64| {
                if (a - b).abs() > 2.0 * se {
                    paint(WARN, "moved")
                } else {
                    paint(GOOD, "within the noise")
                }
            };
            let _ = writeln!(
                o,
                "  head offset ({:.3}, {:.3}) mm in the profile, fitted ({:.3}, {:.3}): X {}, Y {}",
                now[0],
                now[1],
                new[0],
                new[1],
                note(now[0], new[0], sx),
                note(now[1], new[1], sy)
            );
            let head = match h {
                Head::Bevel => &mut fitted.bevel,
                Head::Straight => &mut fitted.straight,
            };
            head.offset_mm = new.map(|v| (v * 1000.0).round() / 1000.0);
        }
    }
    if !combs.is_empty() {
        let _ = writeln!(
            o,
            "\n{}",
            paint(
                crate::style::BOLD,
                "overcut that frees a corner on the face (by eye)"
            )
        );
        for c in combs {
            let v = &c.values;
            let range = match c.tooth {
                0 => paint(
                    WARN,
                    format!(
                        "more than {:+.1} mm: no tooth reached, widen the comb",
                        v[v.len() - 1]
                    ),
                ),
                1 => paint(
                    WARN,
                    format!("{:+.1} mm or less: tooth 1 reached, widen the comb", v[0]),
                ),
                k => format!(
                    "between {:+.1} and {:+.1} mm (tooth {k})",
                    v[k - 2],
                    v[k - 1]
                ),
            };
            let _ = writeln!(
                o,
                "  {:8} {:5} {range}  {}",
                head_name(c.head),
                if c.at_start { "start" } else { "end" },
                paint(DIM, &c.name)
            );
        }
    }
    let _ = writeln!(
        o,
        "\n{}",
        paint(crate::style::BOLD, "leftovers (reading less model)")
    );
    for (name, r) in &fit.leftovers {
        let flag = if r.abs() > (2.0 * fit.sigma).max(0.15) {
            paint(WARN, "  look again")
        } else {
            String::new()
        };
        let _ = writeln!(o, "  {name:14} {r:+.3}{flag}");
    }
    (o, fitted)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The sheet, cut raw, as its log shows it: each cut's points in carriage mm.
    fn logged(program: &Program) -> Vec<Logged> {
        program
            .ops
            .iter()
            .filter_map(|o| match o {
                Op::Cut(c) => Some(Logged {
                    head: c.head,
                    points: c.path_mm.clone(),
                }),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn the_sheet_is_raw_and_clear_of_the_clamps() {
        let profile = Profile::gunnar_601();
        let p = sheet_program(&profile);
        assert!(crate::cut::is_raw(&p));
        machine::lower(&p, &profile.raw()).expect("lowers raw");
        // Placed with the profile's offsets, every cut clears the clamp border.
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

    #[test]
    fn a_log_gives_back_the_cuts_it_sent() {
        let text = "    0.001 > C1⏎\n    0.002 > L21,X400,Y800,Z0⏎\n    0.003 > A4A⏎\n    \
                    0.004 > W250⏎\n    0.005 > L20,X1400,Y800,Z0⏎\n    0.006 > x40,y-80⏎\n    \
                    0.007 > A42⏎\n    0.008 > L21,X0,Y0,Z0⏎\n";
        let cuts = cuts_from_log(text, &Profile::gunnar_601()).expect("parses");
        assert_eq!(
            cuts,
            vec![Logged {
                head: Head::Straight,
                points: vec![[10.0, 20.0], [35.0, 20.0], [36.0, 18.0]],
            }]
        );
    }

    /// Readings made up from known heads, worked out edge by edge from the sheet's own
    /// geometry, come back out of the fit.
    #[test]
    fn the_fit_recovers_known_heads() {
        let p = sheet_program(&Profile::gunnar_601());
        let cuts = logged(&p);
        let start = |id: &str| -> [f64; 2] {
            let k = p
                .ops
                .iter()
                .filter_map(|o| match o {
                    Op::Cut(c) => Some(c),
                    _ => None,
                })
                .position(|c| c.id.as_deref() == Some(id))
                .expect("cut");
            cuts[k].points[0]
        };
        let x = |id: &str| start(id)[0];
        let y = |id: &str| start(id)[1];
        // Straight: P (10, 20), slit sides 0.05 left and 0.15 right. Bevel: P (90, 18), the
        // window's back edge 0.4 outside, its face edge 2.1 outside.
        let (sp, sl, sr) = ([10.0, 20.0], 0.05, 0.15);
        let (bp, back, face) = ([90.0, 18.0], 0.4, 2.1);
        let readings: HashMap<String, f64> = [
            ("SL.width", (x("S.B") - x("S.A")) - 2.0 * sl),
            ("SR.width", (x("S.C") - x("S.B")) - 2.0 * sr),
            ("SL.length", (y("S.K") - y("S.F")) - 2.0 * sl),
            ("S.left", x("S.A") - sp[0] - sr),
            ("S.front", y("S.F") - sp[1] - sr),
            ("W.left", x("W.left") - bp[0] - back),
            ("W.width", (x("W.right") - x("W.left")) + 2.0 * back),
            ("W.front", y("W.front") - bp[1] - back),
            ("W.height", (y("W.back") - y("W.front")) + 2.0 * back),
            ("D.width", (x("W.right") - x("W.left")) + 2.0 * face),
            ("D.height", (y("W.back") - y("W.front")) + 2.0 * face),
            ("S.end", 5.0),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_owned(), v))
        .collect();
        let sheet = Sheet {
            label: "test".into(),
            program: p,
            cuts,
            readings,
        };
        let (fit, combs) = fit_sheets(&[sheet]).expect("fits");
        let near = |u: Unknown, want: f64| {
            let (v, _) = fit.get(u).expect("determined");
            assert!((v - want).abs() < 1e-6, "{u:?}: {v} against {want}");
        };
        near(Unknown::P(Head::Straight, 0), sp[0]);
        near(Unknown::P(Head::Straight, 1), sp[1]);
        near(Unknown::E(Head::Straight, false, Face::Any), sl);
        near(Unknown::E(Head::Straight, true, Face::Any), sr);
        near(Unknown::P(Head::Bevel, 0), bp[0]);
        near(Unknown::P(Head::Bevel, 1), bp[1]);
        near(Unknown::E(Head::Bevel, false, Face::Back), back);
        near(Unknown::E(Head::Bevel, true, Face::Front), -face);
        assert!(fit.sigma < 1e-6);
        assert_eq!(combs.len(), 1);
        assert_eq!((combs[0].tooth, combs[0].at_start), (5, false));
    }
}
