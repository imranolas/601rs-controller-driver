//! Stage 1, the heads (CALIBRATION.md §5): a 100 mm mount with a 50 mm bevel window on a
//! 150 × 150 mm offcut, 25 mm in from the front and left edges, clear of the clamp pads.
//!
//! The window is cut clockwise with the bevel and the outline anticlockwise with the straight
//! head, so the mount stays on the blade's left all round, as a kept piece is cut; the outline's
//! strokes run both ways. Sixteen caliper readings, none that nothing checks: the mount's
//! borders read the bevel head against the straight; the mount and the frame's opening read the
//! straight slit's two sides; the window on the back and the drop-out on its face, the bevel's
//! lean; the frame's sides from the stock edges, read twice each, where the straight head is.
//! They fix each head's position, the straight slit's sideways offset and width, and the bevel's
//! sideways offset on the back and its lean.

use std::collections::HashMap;

use program::Head;
use serde_json::{Value, json};

use crate::fit::{Face, Fit, Unknown, known, predict};
use crate::key::{Axis, BladeValue, Key, Surface};
use crate::plan::{Kind, Line, Plan, Reading, Routine, Step, Stock, program};
use crate::proposal::{Change, Proposal, SUSPECT_T};
use crate::{Calibration, Error, Sheet};

/// The mount and the window, `[x0, y0, x1, y1]` on the board, mm.
const MOUNT: [f64; 4] = [25.0, 25.0, 125.0, 125.0];
const WINDOW: [f64; 4] = [50.0, 50.0, 100.0, 100.0];
/// How much further than the calibration's run-ons each cut runs, mm, so every piece comes free
/// even if they are short.
const MARGIN_MM: f64 = 1.0;
/// A reading this far from what's expected is asked about, mm: past a misread's scatter, short
/// of a board not against the stops.
const UNUSUAL_MM: f64 = 1.0;

/// A side of a cut a reading is taken from: the cut, whether the piece is on its right as it
/// travels, and the board's side up.
#[derive(Debug, Clone, Copy)]
struct Edge {
    cut: &'static str,
    right: bool,
    surface: Surface,
}

/// The mount's side of an outline cut, the frame's, the mount's side of a window cut on the
/// back, the drop-out's on its face.
fn mount(cut: &'static str) -> Edge {
    Edge {
        cut,
        right: false,
        surface: Surface::Back,
    }
}

fn frame(cut: &'static str) -> Edge {
    Edge {
        right: true,
        ..mount(cut)
    }
}

fn window(cut: &'static str) -> Edge {
    mount(cut)
}

fn drop_out(cut: &'static str) -> Edge {
    Edge {
        cut,
        right: true,
        surface: Surface::Face,
    }
}

/// What a reading spans: two edges, or the stock edge (against the stops) to one.
#[derive(Debug, Clone, Copy)]
enum Span {
    Width(Edge, Edge),
    FromStock(Edge),
}

struct Spec {
    id: &'static str,
    prompt: &'static str,
    step: usize,
    axis: Axis,
    /// Where along the other axis it is taken, mm on the board.
    at: f64,
    span: Span,
}

const STEPS: [&str; 3] = [
    "Lift the mount out of the frame, push the window's drop-out out of it, and lay the mount \
     back up, as it lay on the machine.",
    "Lay the frame back up, as it lay on the machine.",
    "Turn the drop-out face up.",
];

fn specs() -> Vec<Spec> {
    use Axis::{X, Y};
    use Span::{FromStock, Width};
    let spec = |id, prompt, step, axis, at, span| Spec {
        id,
        prompt,
        step,
        axis,
        at,
        span,
    };
    vec![
        spec(
            "B.left",
            "The mount's left border, its edge to the window's (outside jaws)",
            0,
            X,
            75.0,
            Width(mount("O.left"), window("W.left")),
        ),
        spec(
            "B.right",
            "The mount's right border",
            0,
            X,
            75.0,
            Width(window("W.right"), mount("O.right")),
        ),
        spec(
            "B.front",
            "The mount's front border",
            0,
            Y,
            75.0,
            Width(mount("O.front"), window("W.front")),
        ),
        spec(
            "B.back",
            "The mount's back border",
            0,
            Y,
            75.0,
            Width(window("W.back"), mount("O.back")),
        ),
        spec(
            "W.width",
            "The window, left to right (inside jaws)",
            0,
            X,
            62.0,
            Width(window("W.left"), window("W.right")),
        ),
        spec(
            "W.height",
            "The window, front to back (inside jaws)",
            0,
            Y,
            62.0,
            Width(window("W.front"), window("W.back")),
        ),
        spec(
            "M.width",
            "The mount, left to right (outside jaws)",
            0,
            X,
            112.0,
            Width(mount("O.left"), mount("O.right")),
        ),
        spec(
            "M.height",
            "The mount, front to back (outside jaws)",
            0,
            Y,
            112.0,
            Width(mount("O.front"), mount("O.back")),
        ),
        spec(
            "F.width",
            "The frame's opening, left to right (inside jaws)",
            1,
            X,
            88.0,
            Width(frame("O.left"), frame("O.right")),
        ),
        spec(
            "F.height",
            "The frame's opening, front to back (inside jaws)",
            1,
            Y,
            88.0,
            Width(frame("O.front"), frame("O.back")),
        ),
        spec(
            "S.left",
            "The frame's left side near the front: the board's edge to the opening",
            1,
            X,
            35.0,
            FromStock(frame("O.left")),
        ),
        spec(
            "S.left2",
            "The frame's left side near the back",
            1,
            X,
            115.0,
            FromStock(frame("O.left")),
        ),
        spec(
            "S.front",
            "The frame's front side near the left: the board's edge to the opening",
            1,
            Y,
            35.0,
            FromStock(frame("O.front")),
        ),
        spec(
            "S.front2",
            "The frame's front side near the right",
            1,
            Y,
            115.0,
            FromStock(frame("O.front")),
        ),
        spec(
            "D.width",
            "The drop-out, left to right, face up (outside jaws)",
            2,
            X,
            85.0,
            Width(drop_out("W.left"), drop_out("W.right")),
        ),
        spec(
            "D.height",
            "The drop-out, front to back, face up (outside jaws)",
            2,
            Y,
            85.0,
            Width(drop_out("W.front"), drop_out("W.back")),
        ),
    ]
}

/// The window clockwise, then the outline anticlockwise, each line on its design and run on
/// past its corners by the calibration's run-ons on this board, and [`MARGIN_MM`] more.
fn lines(cal: &Calibration, thickness_mm: f64) -> Vec<Line> {
    let blade = cal.blade();
    let ends = |h: Head| blade.head(h).run_on(thickness_mm).map(|r| r + MARGIN_MM);
    let ([s, e], b) = (ends(Head::Bevel), Head::Bevel);
    let [x0, y0, x1, y1] = WINDOW;
    let line = |id: &str, label: &str, head, a, b| Line {
        id: id.to_owned(),
        label: label.to_owned(),
        head,
        a,
        b,
    };
    let mut out = vec![
        line(
            "W.right",
            "window, right edge",
            b,
            [x1, y1 + s],
            [x1, y0 - e],
        ),
        line(
            "W.front",
            "window, front edge",
            b,
            [x1 + s, y0],
            [x0 - e, y0],
        ),
        line("W.left", "window, left edge", b, [x0, y0 - s], [x0, y1 + e]),
        line("W.back", "window, back edge", b, [x0 - s, y1], [x1 + e, y1]),
    ];
    let ([s, e], h) = (ends(Head::Straight), Head::Straight);
    let [x0, y0, x1, y1] = MOUNT;
    out.extend([
        line(
            "O.front",
            "mount, front edge",
            h,
            [x0 - s, y0],
            [x1 + e, y0],
        ),
        line(
            "O.right",
            "mount, right edge",
            h,
            [x1, y0 - s],
            [x1, y1 + e],
        ),
        line("O.back", "mount, back edge", h, [x1 + s, y1], [x0 - e, y1]),
        line("O.left", "mount, left edge", h, [x0, y1 + s], [x0, y0 - e]),
    ]);
    out
}

fn edge_json(lines: &[Line], e: Edge) -> Value {
    let face = match (head_of(lines, e.cut), e.surface) {
        (Head::Straight, _) => "any",
        (Head::Bevel, Surface::Back) => "back",
        (Head::Bevel, Surface::Face) => "face",
    };
    json!({ "cut": e.cut, "kept": if e.right { "right" } else { "left" }, "face": face })
}

fn head_of(lines: &[Line], cut: &str) -> Head {
    lines
        .iter()
        .find(|l| l.id == cut)
        .map_or(Head::Straight, |l| l.head)
}

/// The readings as the fit reads them (`meta.calibration.readings`), each with what it should
/// read if that is known.
fn readings_json(lines: &[Line], expected: &HashMap<String, f64>) -> Vec<Value> {
    specs()
        .iter()
        .map(|s| {
            let axis = if s.axis == Axis::X { "x" } else { "y" };
            let mut v = match s.span {
                Span::Width(a, b) => json!({
                    "name": s.id, "kind": "width", "axis": axis,
                    "edges": [edge_json(lines, a), edge_json(lines, b)],
                }),
                Span::FromStock(e) => json!({
                    "name": s.id, "kind": "position", "axis": axis,
                    "edge": edge_json(lines, e),
                }),
            };
            v["what"] = json!(s.prompt);
            v["step"] = json!(STEPS[s.step]);
            v["unusual"] = json!(UNUSUAL_MM);
            if let Some(x) = expected.get(s.id) {
                v["about"] = json!((x * 100.0).round() / 100.0);
            }
            v
        })
        .collect()
}

/// Where an edge lies on the board, by the calibration, mm along `axis`.
fn edge_at(lines: &[Line], e: Edge, axis: Axis, cal: &Calibration, thickness_mm: f64) -> f64 {
    let Some(l) = lines.iter().find(|l| l.id == e.cut) else {
        return f64::NAN;
    };
    let i = usize::from(axis == Axis::Y);
    let len = (l.b[0] - l.a[0]).hypot(l.b[1] - l.a[1]);
    let d = [(l.b[0] - l.a[0]) / len, (l.b[1] - l.a[1]) / len];
    // Toward the kept side: right of travel, or left.
    let toward = if e.right {
        [d[1], -d[0]]
    } else {
        [-d[1], d[0]]
    };
    let face = match e.surface {
        Surface::Face if l.head == Head::Bevel => Face::Front,
        Surface::Back if l.head == Head::Bevel => Face::Back,
        _ => Face::Any,
    };
    l.a[i] + toward[i].round() * known(Unknown::E(l.head, e.right, face), cal, thickness_mm)
}

/// Stage 1's plan on `stock`, placed and predicted with calibration `cal`.
///
/// # Errors
/// When the board isn't one stage 1 can be cut on, or the program doesn't fit the machine.
pub fn plan(cal: &Calibration, stock: &Stock) -> Result<Plan, Error> {
    stock.check()?;
    let t = stock.thickness_mm;
    let cut_with = cal.machine();
    let lines = lines(cal, t);
    let program = |readings| program("stage_1", Routine::Heads, stock, &lines, readings);
    let unplaced = program(readings_json(&lines, &HashMap::new()));
    let sheet = Sheet::lowered("stage 1", unplaced, &cut_with, HashMap::new())?;
    let expected = predict(&sheet, cal)?;
    let readings: Vec<Reading> = specs()
        .iter()
        .map(|s| {
            let at = |e| edge_at(&lines, e, s.axis, cal, t);
            let (from, to) = match s.span {
                Span::Width(a, b) => (at(a), at(b)),
                Span::FromStock(e) => (0.0, at(e)),
            };
            let point = |v| match s.axis {
                Axis::X => [v, s.at],
                Axis::Y => [s.at, v],
            };
            let surface = match s.span {
                Span::Width(a, b) if a.surface == Surface::Face || b.surface == Surface::Face => {
                    Surface::Face
                }
                _ => Surface::Back,
            };
            Reading {
                id: s.id.to_owned(),
                prompt: s.prompt.to_owned(),
                surface,
                kind: Kind::Length {
                    expected_mm: expected.get(s.id).copied().unwrap_or(f64::NAN),
                    unusual_mm: UNUSUAL_MM,
                },
                dimension: [point(from), point(to)],
                needs: match s.span {
                    Span::Width(a, b) => vec![a.cut.to_owned(), b.cut.to_owned()],
                    Span::FromStock(e) => vec![e.cut.to_owned()],
                },
            }
        })
        .collect();
    let steps = STEPS
        .iter()
        .enumerate()
        .map(|(k, instruction)| Step {
            instruction: (*instruction).to_owned(),
            readings: specs()
                .iter()
                .filter(|s| s.step == k)
                .map(|s| s.id.to_owned())
                .collect(),
        })
        .collect();
    Ok(Plan {
        routine: Routine::Heads,
        board: stock.need(),
        program: program(readings_json(&lines, &expected)),
        cut_with,
        steps,
        readings,
    })
}

/// What stage 1's fit proposes for `cal`: each head's offset, the straight slit's sideways
/// offset and width, the bevel's sideways offset on the back and its lean across board
/// `thickness_mm` thick. The bevel's slit width is held: its two sides are read on different
/// faces, so only their sum with the lean shows.
pub fn propose(fit: &Fit, cal: &Calibration, thickness_mm: f64) -> Proposal {
    let (s, b) = (Head::Straight, Head::Bevel);
    let machine = cal.machine();
    let mut changes = Vec::new();
    for h in [s, b] {
        let offset = fit.head_offset(h, &machine);
        for (i, axis) in [Axis::X, Axis::Y].into_iter().enumerate() {
            let fitted = offset.map(|(v, se)| (v[i], se[i]));
            changes.push(Change::new(cal, Key::HeadOffset(h, axis), fitted));
        }
    }
    let blade = |head, value| Key::Blade { head, value };
    let (left, right) = (
        Unknown::E(s, false, Face::Any),
        Unknown::E(s, true, Face::Any),
    );
    changes.push(Change::new(
        cal,
        blade(s, BladeValue::Sideways),
        fit.combination(&[(left, 0.5), (right, -0.5)]),
    ));
    changes.push(Change::new(
        cal,
        blade(s, BladeValue::Width),
        fit.combination(&[(left, 1.0), (right, 1.0)]),
    ));
    let width = cal.blade().bevel.width_mm;
    let (back, face) = (
        Unknown::E(b, false, Face::Back),
        Unknown::E(b, true, Face::Front),
    );
    changes.push(Change::new(
        cal,
        blade(b, BladeValue::Sideways),
        fit.get(back).map(|(v, se)| (v - width / 2.0, se)),
    ));
    let run = fit
        .combination(&[(face, -1.0), (back, -1.0)])
        .map(|(v, se)| (v + width, se));
    let t = thickness_mm;
    let lean = run.map(|(run, se)| {
        let r = (run / t).atan();
        (r.to_degrees(), (se / t * r.cos().powi(2)).to_degrees())
    });
    changes.push(Change::new(cal, blade(b, BladeValue::Lean), lean));
    let mut notes = Vec::new();
    if let (Some((run, _)), Some((deg, se))) = (run, lean) {
        notes.push(format!(
            "The bevel runs {run:.2} mm across the {t} mm board: {deg:.1}° ± {se:.1}° from \
             vertical."
        ));
    }
    Proposal {
        routine: Routine::Heads,
        readings: fit.rows,
        scatter_mm: fit.sigma,
        changes,
        suspects: fit.suspects(SUSPECT_T).into_iter().cloned().collect(),
        notes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::{fit_sheets, precision};
    use crate::{Edit, Why};

    fn key(s: &str) -> Key {
        s.parse().expect(s)
    }

    /// With the default calibration, on 1.5 mm board.
    fn default_plan() -> Plan {
        plan(&Calibration::new(), &Stock::offcut(1.5)).expect("a plan")
    }

    #[test]
    fn the_plan_cuts_clear_of_the_pads_and_lowers() {
        let p = default_plan();
        machine::lower(&p.program, &p.cut_with).expect("lowers");
        let pads = p
            .cut_with
            .pads_on(crate::plan::OFFCUT_MM, crate::plan::OFFCUT_MM);
        for op in &p.program.ops {
            if let program::Op::Cut(c) = op {
                let hit: Vec<&str> = pads
                    .iter()
                    .filter(|pad| pad.hit(c.path_mm[0], c.path_mm[1]))
                    .map(|pad| pad.name.as_str())
                    .collect();
                assert!(hit.is_empty(), "{:?} runs over {hit:?}", c.id);
                let inside = |q: &[f64; 2]| q.iter().all(|v| (21.0..=129.0).contains(v));
                assert!(c.path_mm.iter().all(inside), "{:?}", c.path_mm);
            }
        }
    }

    #[test]
    fn sixteen_readings_in_three_steps_none_unchecked() {
        let p = default_plan();
        assert_eq!(p.readings.len(), 16);
        let stepped: Vec<&String> = p.steps.iter().flat_map(|s| &s.readings).collect();
        assert_eq!(stepped.len(), 16);
        let all: HashMap<String, f64> = p.readings.iter().map(|r| (r.id.clone(), 0.0)).collect();
        let sheet = p.sheet("plan", all).expect("a sheet");
        let pr = precision(&sheet, &[]).expect("a precision");
        assert!(pr.unchecked.is_empty(), "{:?}", pr.unchecked);
        assert!(pr.tied.is_empty(), "{:?}", pr.tied);
        // For caliper readings scattering 0.11 mm: the straight head's position to 0.087 mm,
        // the bevel's to 0.117 (the borders' scatter added to the straight's), each slit side to
        // under 0.04, and the window's place on the mount across to under 0.09.
        for (u, se) in &pr.values {
            let most = match u {
                Unknown::P(Head::Straight, _) => 0.09,
                Unknown::P(Head::Bevel, _) => 0.12,
                _ => 0.04,
            };
            assert!(se.is_some_and(|se| se < most), "{u:?}: {se:?}");
        }
        let (s, b) = (Head::Straight, Head::Bevel);
        let across = pr
            .combination(&[
                (Unknown::P(b, 0), 1.0),
                (Unknown::P(s, 0), -1.0),
                (Unknown::E(s, true, Face::Any), 0.5),
                (Unknown::E(s, false, Face::Any), -0.5),
            ])
            .expect("fixed");
        assert!(across < 0.09, "{across}");
    }

    /// What the default calibration expects, as the design reads: borders of 25 mm less the
    /// bevel's back edge plus the straight slit's side, the drop-out 2.16 mm out on its face.
    #[test]
    fn the_expected_readings_are_the_designs_with_the_calibrations_edges() {
        let p = default_plan();
        let e = |id: &str| p.reading(id).and_then(Reading::expected_mm).expect(id);
        let (back, face) = (0.4525, 2.1625);
        // To the motor step the cuts are rounded to, 0.025 mm, as lowering sends them.
        let close = |a: f64, b: f64| (a - b).abs() < 0.026;
        // Cut anticlockwise, the straight slit lies outside its line: its left side, the
        // mount's edge, 0.1025 mm out, its right side, the frame's, 0.0875.
        assert!(
            close(e("M.width"), 100.0 + 2.0 * 0.1025),
            "{}",
            e("M.width")
        );
        assert!(
            close(e("F.width"), 100.0 + 2.0 * 0.0875),
            "{}",
            e("F.width")
        );
        assert!(close(e("W.width"), 50.0 + 2.0 * back), "{}", e("W.width"));
        assert!(close(e("D.width"), 50.0 + 2.0 * face), "{}", e("D.width"));
        assert!(close(e("B.left"), 25.0 - back + 0.1025), "{}", e("B.left"));
        // The stock edge to the opening: 25 mm, less the slit's right side, where the default
        // offsets put the head.
        assert!(close(e("S.left"), 25.0 - 0.0875), "{}", e("S.left"));
        assert_eq!(e("S.left"), e("S.left2"));
        // Each dimension spans what it reads.
        for r in &p.readings {
            let [a, b] = r.dimension;
            let span = (b[0] - a[0]).abs() + (b[1] - a[1]).abs();
            let want = r.expected_mm().expect("a length");
            assert!(close(span, want), "{}: {span} against {want}", r.id);
        }
        assert_eq!(
            p.check("W.width", 50.9).expect("a length"),
            crate::Check::Fine
        );
        assert!(matches!(
            p.check("W.width", 52.5),
            Ok(crate::Check::Unusual { .. })
        ));
        assert!(p.check("W.depth", 1.0).is_err() && p.check("W.width", -1.0).is_err());
    }

    /// A machine whose heads and blades differ from the file's: stage 1, read on it with a
    /// caliper's scatter, proposes what it is, and applying the proposal makes the file say so.
    #[test]
    fn the_fit_recovers_a_machine_unlike_the_file() {
        let file = Calibration::new();
        let why = Why::typed("2026-11-16T12:00:00Z");
        let truth = file
            .set(
                &[
                    Edit::set(key("machine.straight.offset.x"), 6.613),
                    Edit::set(key("machine.straight.offset.y"), 1.668),
                    Edit::set(key("machine.bevel.offset.x"), -71.152),
                    Edit::set(key("machine.bevel.offset.y"), 2.282),
                    Edit::set(key("blade.straight.sideways"), -0.03),
                    Edit::set(key("blade.straight.width"), 0.05),
                    Edit::set(key("blade.bevel.sideways"), 0.6),
                    Edit::set(key("blade.bevel.lean"), 45.0),
                ],
                &why,
            )
            .expect("a truth");
        let p = plan(&file, &Stock::offcut(1.4)).expect("a plan");
        let all: HashMap<String, f64> = p.readings.iter().map(|r| (r.id.clone(), 0.0)).collect();
        let cut = p.sheet("stage 1", all).expect("a sheet");
        let clean = predict(&cut, &truth).expect("predicted");
        let noise = [
            0.03, -0.02, 0.01, -0.04, 0.02, 0.0, -0.01, 0.03, -0.03, 0.02, -0.01, 0.01, 0.0, -0.02,
            0.01, 0.02,
        ];
        let readings: HashMap<String, f64> = p
            .readings
            .iter()
            .zip(noise)
            .map(|(r, n)| (r.id.clone(), clean[&r.id] + n))
            .collect();
        let (fit, _) = fit_sheets(&[p.sheet("stage 1", readings).expect("a sheet")]).expect("fits");
        let proposal = propose(&fit, &file, 1.4);
        assert!(proposal.suspects.is_empty(), "{:?}", proposal.suspects);
        for c in &proposal.changes {
            let want = truth.num(&c.key).expect("a value");
            let (after, se) = (c.after.expect("fixed"), c.se.expect("an error"));
            assert!(
                (after - want).abs() < 3.0 * se + 0.002,
                "{}: {after} ± {se} against {want}",
                c.key
            );
        }
        let applied = proposal
            .apply(&file, &proposal.chosen(), &why)
            .expect("applied");
        assert_eq!(applied.revision(), 1);
        let entry = &applied.history()[0];
        assert_eq!(entry.routine.as_deref(), Some("heads"));
        assert!(entry.changes.iter().all(|c| c.se.is_some()));
        // Every value moved by more than its noise is proposed; the lean, 3.7° off on a 1.4 mm
        // board, among them.
        let moved = proposal.chosen();
        assert!(moved.contains(&key("blade.bevel.lean")), "{moved:?}");
        assert!(moved.contains(&key("machine.bevel.offset.x")), "{moved:?}");
    }

    /// A reading misread by half a millimetre is named, not fitted into the heads.
    #[test]
    fn a_misread_is_named() {
        let file = Calibration::new();
        let p = default_plan();
        let all: HashMap<String, f64> = p.readings.iter().map(|r| (r.id.clone(), 0.0)).collect();
        let clean = predict(&p.sheet("plan", all).expect("a sheet"), &file).expect("predicted");
        let noise = [0.02, -0.01, 0.0, 0.01, -0.02, 0.01, 0.0, -0.01];
        let mut readings: HashMap<String, f64> = p
            .readings
            .iter()
            .zip(noise.iter().cycle())
            .map(|(r, n)| (r.id.clone(), clean[&r.id] + n))
            .collect();
        *readings.get_mut("S.left").expect("listed") += 0.5;
        let (fit, _) = fit_sheets(&[p.sheet("stage 1", readings).expect("a sheet")]).expect("fits");
        let proposal = propose(&fit, &file, 1.5);
        let named: Vec<&str> = proposal.suspects.iter().map(|l| l.name.as_str()).collect();
        assert!(named.contains(&"S.left"), "{named:?}");
    }
}
