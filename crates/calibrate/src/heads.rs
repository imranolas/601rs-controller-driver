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

use compiler::Blade;
use program::Head;

use crate::fit::{Fit, Prior, Unknown, fit_sheets_with};
use crate::key::{Axis, BladeValue, Key, Surface};
use crate::plan::{Line, Plan, Routine, Step, Stock, program};
use crate::proposal::{Change, Proposal, SUSPECT_T};
use crate::reading::{Edge, Kind, Reading, Side, Span};
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

/// An [`Edge`], as the table below writes it: the cut, the side of it the piece read lies on,
/// and the board's side up.
#[derive(Debug, Clone, Copy)]
struct E(&'static str, Side, Surface);

impl E {
    fn edge(self) -> Edge {
        Edge {
            cut: self.0.to_owned(),
            side: self.1,
            surface: self.2,
        }
    }
}

/// The mount's side of an outline cut, the frame's, the mount's side of a window cut on the
/// back, the drop-out's on its face.
const fn mount(cut: &'static str) -> E {
    E(cut, Side::Left, Surface::Back)
}

const fn frame(cut: &'static str) -> E {
    E(cut, Side::Right, Surface::Back)
}

const fn window(cut: &'static str) -> E {
    mount(cut)
}

const fn drop_out(cut: &'static str) -> E {
    E(cut, Side::Right, Surface::Face)
}

/// What a reading spans: two edges, or the stock edge (against the stops) to one.
#[derive(Debug, Clone, Copy)]
enum S {
    Between(E, E),
    FromStock(E),
}

/// A reading: its id, the step it is taken in, its axis, where along the other axis it is
/// taken (mm on the board), what it spans, and its prompt.
struct Spec(&'static str, usize, Axis, f64, S, &'static str);

const STEPS: [&str; 3] = [
    "Lift the mount out of the frame, push the window's drop-out out of it, and lay the mount \
     back up, as it lay on the machine.",
    "Lay the frame back up, as it lay on the machine.",
    "Turn the drop-out face up.",
];

#[rustfmt::skip]
const SPECS: [Spec; 16] = {
    use Axis::{X, Y};
    use S::{Between as W, FromStock as F};
    [
        Spec("B.left",   0, X,  75.0, W(mount("O.left"), window("W.left")),
             "The mount's left border, its edge to the window's (outside jaws)"),
        Spec("B.right",  0, X,  75.0, W(window("W.right"), mount("O.right")), "The mount's right border"),
        Spec("B.front",  0, Y,  75.0, W(mount("O.front"), window("W.front")), "The mount's front border"),
        Spec("B.back",   0, Y,  75.0, W(window("W.back"), mount("O.back")),   "The mount's back border"),
        Spec("W.width",  0, X,  62.0, W(window("W.left"), window("W.right")),
             "The window, left to right (inside jaws)"),
        Spec("W.height", 0, Y,  62.0, W(window("W.front"), window("W.back")),
             "The window, front to back (inside jaws)"),
        Spec("M.width",  0, X, 112.0, W(mount("O.left"), mount("O.right")),
             "The mount, left to right (outside jaws)"),
        Spec("M.height", 0, Y, 112.0, W(mount("O.front"), mount("O.back")),
             "The mount, front to back (outside jaws)"),
        Spec("F.width",  1, X,  88.0, W(frame("O.left"), frame("O.right")),
             "The frame's opening, left to right (inside jaws)"),
        Spec("F.height", 1, Y,  88.0, W(frame("O.front"), frame("O.back")),
             "The frame's opening, front to back (inside jaws)"),
        Spec("S.left",   1, X,  35.0, F(frame("O.left")),
             "The frame's left side near the front: the board's edge to the opening"),
        Spec("S.left2",  1, X, 115.0, F(frame("O.left")),  "The frame's left side near the back"),
        Spec("S.front",  1, Y,  35.0, F(frame("O.front")),
             "The frame's front side near the left: the board's edge to the opening"),
        Spec("S.front2", 1, Y, 115.0, F(frame("O.front")), "The frame's front side near the right"),
        Spec("D.width",  2, X,  85.0, W(drop_out("W.left"), drop_out("W.right")),
             "The drop-out, left to right, face up (outside jaws)"),
        Spec("D.height", 2, Y,  85.0, W(drop_out("W.front"), drop_out("W.back")),
             "The drop-out, front to back, face up (outside jaws)"),
    ]
};

/// The window clockwise, then the outline anticlockwise, each line on its design and run on
/// past its corners by the calibration's run-ons on this board, and [`MARGIN_MM`] more.
fn lines(cal: &Calibration, thickness_mm: f64) -> Vec<Line> {
    let blade = cal.blade();
    let ends = |h: Head| blade.head(h).run_on(thickness_mm).map(|r| r + MARGIN_MM);
    let line = |id: &str, label: &str, head, a, b| Line {
        id: id.to_owned(),
        label: label.to_owned(),
        head,
        a,
        b,
    };
    let (b, [bs, be]) = (Head::Bevel, ends(Head::Bevel));
    let (h, [hs, he]) = (Head::Straight, ends(Head::Straight));
    let [wx0, wy0, wx1, wy1] = WINDOW;
    let [x0, y0, x1, y1] = MOUNT;
    #[rustfmt::skip]
    let out = vec![
        line("W.right", "window, right edge", b, [wx1, wy1 + bs], [wx1, wy0 - be]),
        line("W.front", "window, front edge", b, [wx1 + bs, wy0], [wx0 - be, wy0]),
        line("W.left",  "window, left edge",  b, [wx0, wy0 - bs], [wx0, wy1 + be]),
        line("W.back",  "window, back edge",  b, [wx0 - bs, wy1], [wx1 + be, wy1]),
        line("O.front", "mount, front edge",  h, [x0 - hs, y0],   [x1 + he, y0]),
        line("O.right", "mount, right edge",  h, [x1, y0 - hs],   [x1, y1 + he]),
        line("O.back",  "mount, back edge",   h, [x1 + hs, y1],   [x0 - he, y1]),
        line("O.left",  "mount, left edge",   h, [x0, y1 + hs],   [x0, y0 - he]),
    ];
    out
}

/// Where an edge lies on the board by the tool model, mm along `axis`.
fn edge_at(lines: &[Line], e: E, axis: Axis, blade: &Blade, thickness_mm: f64) -> f64 {
    let Some(l) = lines.iter().find(|l| l.id == e.0) else {
        return f64::NAN;
    };
    let len = (l.b[0] - l.a[0]).hypot(l.b[1] - l.a[1]);
    let d = [(l.b[0] - l.a[0]) / len, (l.b[1] - l.a[1]) / len];
    // Toward the side read, and how far that side of the slit lies from the line that way.
    let depth = match e.2 {
        Surface::Back => 0.0,
        Surface::Face => thickness_mm,
    };
    let [left, right] = blade.head(l.head).sides(depth);
    let (toward, out) = match e.1 {
        Side::Right => ([d[1], -d[0]], -right),
        Side::Left => ([-d[1], d[0]], left),
    };
    let i = axis.index();
    l.a[i] + toward[i].round() * out
}

impl Spec {
    /// The reading, with where it is taken on the board by the tool model; what it should read
    /// is the plan's to work out ([`Plan::expecting`]).
    fn reading(&self, lines: &[Line], blade: &Blade, thickness_mm: f64) -> Reading {
        let Spec(id, _, axis, along, span, prompt) = *self;
        let at = |e| edge_at(lines, e, axis, blade, thickness_mm);
        let (from, to, span) = match span {
            S::Between(a, b) => (
                at(a),
                at(b),
                Span::Between {
                    axis,
                    edges: [a.edge(), b.edge()],
                },
            ),
            S::FromStock(e) => (
                0.0,
                at(e),
                Span::FromStock {
                    axis,
                    edge: e.edge(),
                },
            ),
        };
        let point = |v| match axis {
            Axis::X => [v, along],
            Axis::Y => [along, v],
        };
        let face_up = matches!(span, Span::Between { ref edges, .. }
            if edges.iter().any(|e| e.surface == Surface::Face));
        Reading {
            id: id.to_owned(),
            prompt: prompt.to_owned(),
            surface: if face_up {
                Surface::Face
            } else {
                Surface::Back
            },
            kind: Kind::Length {
                span,
                expected_mm: None,
                unusual_mm: UNUSUAL_MM,
            },
            dimension: [point(from), point(to)],
        }
    }
}

/// Stage 1's plan on `stock`, placed and predicted with calibration `cal`.
///
/// # Errors
/// When the board isn't one stage 1 can be cut on, or the program doesn't fit the machine.
pub fn plan(cal: &Calibration, stock: &Stock) -> Result<Plan, Error> {
    stock.check()?;
    let t = stock.thickness_mm;
    let lines = lines(cal, t);
    let blade = cal.blade();
    Plan {
        routine: Routine::Heads,
        board: stock.need(),
        program: program("stage_1", Routine::Heads, stock, &lines),
        cut_with: cal.machine(),
        steps: STEPS
            .iter()
            .enumerate()
            .map(|(k, instruction)| Step {
                instruction: (*instruction).to_owned(),
                readings: SPECS
                    .iter()
                    .filter(|s| s.1 == k)
                    .map(|s| s.0.to_owned())
                    .collect(),
            })
            .collect(),
        readings: SPECS.iter().map(|s| s.reading(&lines, &blade, t)).collect(),
    }
    .expecting(cal)
}

/// What stage 1 takes as known: the bevel's slit width, where `cal` has it. Its two sides are
/// read on different faces, so the readings see the width only together with its sideways
/// offset and lean.
pub fn priors(cal: &Calibration) -> Vec<Prior> {
    vec![Prior::held(
        Unknown::Width(Head::Bevel),
        cal.blade().bevel.width_mm,
    )]
}

/// Stage 1's fit of `sheets` (or the record's), with what it takes as known ([`priors`]).
///
/// # Errors
/// As [`fit_sheets_with`].
pub fn fit(sheets: &[Sheet], cal: &Calibration) -> Result<Fit, Error> {
    fit_sheets_with(sheets, &priors(cal))
}

/// What stage 1's fit proposes for `cal`: each head's offset and slit's sideways offset, the
/// straight slit's width, and the bevel's lean, read on board `thickness_mm` thick.
pub fn propose(fit: &Fit, cal: &Calibration, thickness_mm: f64) -> Proposal {
    let (s, b) = (Head::Straight, Head::Bevel);
    let machine = cal.machine();
    let mut changes = Vec::new();
    for h in [s, b] {
        let offset = fit.head_offset(h, &machine);
        for axis in Axis::BOTH {
            let fitted = offset.map(|(v, se)| (v[axis.index()], se[axis.index()]));
            changes.push(Change::new(cal, Key::HeadOffset(h, axis), fitted));
        }
    }
    let mut set = |head, value, fitted| {
        changes.push(Change::new(cal, Key::Blade { head, value }, fitted));
    };
    set(s, BladeValue::Sideways, fit.get(Unknown::Sideways(s)));
    set(s, BladeValue::Width, fit.get(Unknown::Width(s)));
    set(b, BladeValue::Sideways, fit.get(Unknown::Sideways(b)));
    let run = fit.get(Unknown::Run(b));
    let lean = run.map(|(r, se)| {
        let a = r.atan();
        (a.to_degrees(), (se * a.cos().powi(2)).to_degrees())
    });
    set(b, BladeValue::Lean, lean);
    let t = thickness_mm;
    let mut notes = Vec::new();
    if let (Some((r, _)), Some((deg, se))) = (run, lean) {
        notes.push(format!(
            "The bevel runs {:.2} mm across the {t} mm board: {deg:.1}° ± {se:.1}° from \
             vertical.",
            r * t
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
    .with_effects(cal, thickness_mm)
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::*;
    use crate::fit::{precision, predict};
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
        let pr = precision(&sheet, &priors(&Calibration::new())).expect("a precision");
        assert!(pr.unchecked.is_empty(), "{:?}", pr.unchecked);
        assert!(pr.tied.is_empty(), "{:?}", pr.tied);
        // For caliper readings scattering 0.11 mm: the straight head's position to 0.087 mm,
        // the bevel's to 0.117 (the borders' scatter added to the straight's); each slit's
        // middle to under 0.04, the straight slit's width to 0.052 (each side to 0.036), the
        // bevel's run to 0.035 per mm of board, its width held; and the window's place on the
        // mount across to under 0.09.
        for (u, se) in &pr.values {
            let most = match u {
                Unknown::P(Head::Straight, _) => 0.09,
                Unknown::P(Head::Bevel, _) => 0.12,
                Unknown::Width(Head::Straight) => 0.06,
                Unknown::Width(Head::Bevel) => 0.002,
                _ => 0.04,
            };
            assert!(se.is_some_and(|se| se < most), "{u:?}: {se:?}");
        }
        let (s, b) = (Head::Straight, Head::Bevel);
        let across = pr
            .combination(&[
                (Unknown::P(b, Axis::X), 1.0),
                (Unknown::P(s, Axis::X), -1.0),
                (Unknown::Sideways(s), -1.0),
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
        let fit = fit(&[p.sheet("stage 1", readings).expect("a sheet")], &file).expect("fits");
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
        // In words: the straight head's cuts land 0.2 mm right of where the file puts them.
        let x = proposal
            .changes
            .iter()
            .find(|c| c.key == key("machine.straight.offset.x"))
            .expect("proposed");
        assert!(
            x.effect.contains("the straight head's cuts land 0.2") && x.effect.contains("right"),
            "{}",
            x.effect
        );
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
        let fit = fit(&[p.sheet("stage 1", readings).expect("a sheet")], &file).expect("fits");
        let proposal = propose(&fit, &file, 1.5);
        let named: Vec<&str> = proposal.suspects.iter().map(|l| l.name.as_str()).collect();
        assert!(named.contains(&"S.left"), "{named:?}");
    }
}
