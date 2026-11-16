//! Stage 2, the run-ons (CALIBRATION.md §5): calibration sheet 2's combs on a 150 × 150 mm
//! offcut, read by flexing the board.
//!
//! A comb is a reference cut and seven teeth whose run-ons are graded round what the calibration
//! predicts on this board: −1.2, −0.6, −0.3, 0, +0.3, +0.6 and +1.2 mm, fine where the answer
//! most likely falls. An end comb's teeth run on past the reference, a start comb's begin past it.
//! The reference is cut by the same head and keeps the teeth's side on its right, as a window's
//! next edge keeps its drop-out, in production's order: after the teeth at an end, before them at
//! a start. Each head has an end comb and a start comb, each cut twice, its teeth running toward
//! the back once and toward the front once, so cuts both ways are checked: eight combs, 64 cuts.
//!
//! Flexed along its reference, a tooth whose cut has gone through to it on both faces comes
//! free there. The first tooth to do so gives the run-on that frees a corner on this board: the
//! head's reach on the face that frees last. [`propose`] writes it as that face's lead or
//! shortfall, so the compiler's run-on on this board is the one that freed it in both directions.

use std::collections::HashMap;

use program::Head;
use serde_json::{Value, json};

use crate::fit::CombReading;
use crate::key::{BladeValue, Key, Surface};
use crate::plan::{Kind, Line, Plan, Reading, Routine, Step, Stock, program};
use crate::proposal::{Change, Proposal};
use crate::{Calibration, Error};

/// Each tooth's run-on less the prediction, mm, tooth 1 first: fine in the middle, coarse at the
/// ends. Finer than 0.3 isn't worth cutting: whether a corner comes free varies with the board's
/// fibres and the blade's edge by about 0.1–0.2 mm.
pub const GRADE: [f64; 7] = [-1.2, -0.6, -0.3, 0.0, 0.3, 0.6, 1.2];
/// How far a tooth reaches on its own side of the reference, mm, and the teeth's spacing.
const TOOTH_MM: f64 = 10.0;
const PITCH_MM: f64 = 5.0;
/// How far the reference runs past its outer teeth, mm.
const REFERENCE_PAST_MM: f64 = 3.0;
/// The furthest a tooth may run on, mm, so neighbouring combs keep clear of each other.
const MOST_MM: f64 = 7.0;
/// Each head's end comb and start comb, and where their references lie on the board (Y, mm).
const ROWS: [(Head, bool, f64); 4] = [
    (Head::Bevel, false, 38.0),
    (Head::Bevel, true, 66.0),
    (Head::Straight, false, 94.0),
    (Head::Straight, true, 122.0),
];
/// Where each row's two combs start (X, mm): the first with its teeth in front of the
/// reference, the second with them behind it.
const SIDES: [(bool, f64); 2] = [(false, 30.0), (true, 90.0)];

const STEP: &str = "Leave the board back up, as it lay on the machine. Flex it along each comb's \
                    reference: a tooth whose cut has gone through to the reference comes free \
                    there. Count the teeth from the left: tooth 1 is the shortest.";

fn head_word(h: Head) -> &'static str {
    match h {
        Head::Bevel => "bevel",
        Head::Straight => "straight",
    }
}

fn end_word(at_start: bool) -> &'static str {
    if at_start { "start" } else { "end" }
}

/// One comb: its id, what it is, its teeth's run-ons and its cuts in the order they are made.
struct Comb {
    id: String,
    head: Head,
    at_start: bool,
    prompt: String,
    run_ons: [f64; 7],
    reference: Line,
    teeth: Vec<Line>,
}

fn combs(cal: &Calibration, thickness_mm: f64) -> Result<Vec<Comb>, Error> {
    let blade = cal.blade();
    let mut out = Vec::new();
    for (head, at_start, y) in ROWS {
        let nominal = blade.head(head).run_on(thickness_mm)[usize::from(!at_start)];
        let run_ons = GRADE.map(|g| ((nominal + g) * 1000.0).round() / 1000.0);
        if run_ons.iter().any(|r| r.abs() > MOST_MM) {
            return Err(format!(
                "the {} head's run-on at the {}, {nominal:.2} mm, is beyond what stage 2's combs \
                 can show",
                head_word(head),
                end_word(at_start)
            )
            .into());
        }
        for (behind, x0) in SIDES {
            // A point `v` from the reference toward the teeth, at `x`.
            let at = |x: f64, v: f64| if behind { [x, y + v] } else { [x, y - v] };
            // The teeth run toward the back (+Y) at an end with them in front, or at a start
            // with them behind.
            let to_back = behind == at_start;
            let id = format!(
                "{}.{}.{}",
                if head == Head::Bevel { "B" } else { "S" },
                end_word(at_start),
                if to_back { "back" } else { "front" }
            );
            let what = format!(
                "{} {} comb, teeth toward the {}",
                head_word(head),
                end_word(at_start),
                if to_back { "back" } else { "front" }
            );
            let teeth = run_ons
                .iter()
                .enumerate()
                .map(|(j, &r)| {
                    let x = x0 + PITCH_MM * j as f64;
                    // An end's tooth comes to the reference and runs on past it; a start's
                    // begins past it and runs away.
                    let (a, b) = if at_start {
                        (at(x, -r), at(x, TOOTH_MM))
                    } else {
                        (at(x, TOOTH_MM), at(x, -r))
                    };
                    Line {
                        id: format!("{id}.{}", j + 1),
                        label: format!("{what}, tooth {}", j + 1),
                        head,
                        a,
                        b,
                    }
                })
                .collect();
            // The teeth on the reference's right: it runs +X with them in front, −X behind.
            let (left, right) = (
                [x0 - REFERENCE_PAST_MM, y],
                [x0 + PITCH_MM * 6.0 + REFERENCE_PAST_MM, y],
            );
            let (a, b) = if behind { (right, left) } else { (left, right) };
            out.push(Comb {
                prompt: format!(
                    "The {what}: the first tooth to come free at the reference (1 to 7; 0 if \
                     none)"
                ),
                reference: Line {
                    id: format!("{id}.ref"),
                    label: format!("{what}, reference"),
                    head,
                    a,
                    b,
                },
                id,
                head,
                at_start,
                run_ons,
                teeth,
            });
        }
    }
    Ok(out)
}

/// Stage 2's plan on `stock`, its teeth graded round calibration `cal`'s run-ons on this board.
///
/// # Errors
/// When the board isn't one the combs can be cut on, a run-on is beyond what they can show, or
/// the program doesn't fit the machine.
pub fn plan(cal: &Calibration, stock: &Stock) -> Result<Plan, Error> {
    stock.check()?;
    let combs = combs(cal, stock.thickness_mm)?;
    let mut lines = Vec::new();
    for c in &combs {
        // Production's order: a start's reference before its teeth, an end's after.
        if c.at_start {
            lines.push(c.reference.clone());
        }
        lines.extend(c.teeth.iter().cloned());
        if !c.at_start {
            lines.push(c.reference.clone());
        }
    }
    let json: Vec<Value> = combs
        .iter()
        .map(|c| {
            json!({
                "name": c.id, "kind": "comb", "head": head_word(c.head),
                "at": end_word(c.at_start), "values": c.run_ons,
                "what": c.prompt, "step": STEP, "how": "by flexing the board",
            })
        })
        .collect();
    let program = program("stage_2", Routine::RunOns, stock, &lines, json);
    let cut_with = cal.machine();
    machine::lower(&program, &cut_with)?;
    Ok(Plan {
        routine: Routine::RunOns,
        board: stock.need(),
        program,
        cut_with,
        steps: vec![Step {
            instruction: STEP.to_owned(),
            readings: combs.iter().map(|c| c.id.clone()).collect(),
        }],
        readings: combs
            .iter()
            .map(|c| Reading {
                id: c.id.clone(),
                prompt: c.prompt.clone(),
                surface: Surface::Back,
                kind: Kind::Tooth {
                    teeth: c.teeth.len(),
                    cuts: c.teeth.iter().map(|t| t.id.clone()).collect(),
                },
                dimension: [c.reference.a, c.reference.b],
                needs: c
                    .teeth
                    .iter()
                    .chain([&c.reference])
                    .map(|l| l.id.clone())
                    .collect(),
            })
            .collect(),
    })
}

/// The run-on that freed the corner in every direction read, and its standard error (NaN when
/// a comb missed and it is only a bound or a guess). The comb puts it somewhere in the step
/// below the tooth that freed it, as likely anywhere there: a standard error of step ÷ √12, so
/// a value a tooth away has moved and one on the same tooth is within the noise.
fn freed(
    read: &[&CombReading],
    at_start: bool,
    what: &str,
    notes: &mut Vec<String>,
) -> Option<(f64, f64)> {
    let first = read.first()?;
    let n = first.values.len();
    let told: Vec<String> = read
        .iter()
        .map(|c| match c.tooth {
            0 => format!("none on {}", c.name),
            k => format!("tooth {k} on {}", c.name),
        })
        .collect();
    let told = told.join(", ");
    // A run-on in words: how far a cut starts before the corner, or runs past it.
    let words = |r: f64| match (at_start, r >= 0.0) {
        (true, true) => format!("a cut starting {r:.2} mm before it"),
        (true, false) => format!("a cut starting {:.2} mm after it", -r),
        (false, true) => format!("a cut running {r:.2} mm past it"),
        (false, false) => format!("a cut stopping {:.2} mm short of it", -r),
    };
    if let Some(none) = read.iter().find(|c| c.tooth == 0) {
        // Every tooth too short: the next comb is centred past the last.
        let last = none.values.last().copied()?;
        let next = last + (last - none.values[n / 2]);
        notes.push(format!(
            "{what}: {told}. No tooth came free: the corner needs more than the longest, {}; \
             proposed {}, the middle of a comb past this one: cut stage 2 again.",
            words(last),
            words(next)
        ));
        return Some((next, f64::NAN));
    }
    // The run-on that freed the corner in every direction: the longest freeing tooth's.
    let frees = read
        .iter()
        .map(|c| c.values[c.tooth - 1])
        .fold(f64::NEG_INFINITY, f64::max);
    if read.iter().all(|c| c.tooth == 1) {
        notes.push(format!(
            "{what}: {told}. Every tooth came free: the shortest, {}, is enough; proposed that: \
             cut stage 2 again to pin it.",
            words(frees)
        ));
        return Some((frees, f64::NAN));
    }
    // Adjacent teeth agree to within the comb's step; further apart, a direction differs.
    let (lo, hi) = read.iter().fold((usize::MAX, 0), |(lo, hi), c| {
        (lo.min(c.tooth), hi.max(c.tooth))
    });
    let apart = if hi - lo >= 2 {
        " The two directions are more than a tooth apart: the longer is kept; look at the comb \
         that freed later."
    } else {
        ""
    };
    notes.push(format!(
        "{what}: {told}: the corner frees with {}.{apart}",
        words(frees)
    ));
    // The step below the tooth that set it.
    let setter = read
        .iter()
        .find(|c| c.values[c.tooth - 1] == frees)
        .unwrap_or(first);
    let k = setter.tooth.max(2) - 1;
    Some((
        frees,
        (setter.values[k] - setter.values[k - 1]) / 12f64.sqrt(),
    ))
}

/// What stage 2's combs, read on board `thickness_mm` thick, propose for `cal`: for each head,
/// the face's start lead and end shortfall that make the compiler's run-ons on this board the
/// ones that freed the corners; the back's too, where calibration 1 had it freeing later than
/// the combs show.
pub fn propose(combs: &[CombReading], cal: &Calibration, thickness_mm: f64) -> Proposal {
    let blade = cal.blade();
    let mut changes = Vec::new();
    let mut notes = Vec::new();
    for (head, at_start, _) in ROWS {
        let read: Vec<&CombReading> = combs
            .iter()
            .filter(|c| c.head == head && c.at_start == at_start)
            .collect();
        let what = format!("The {} head's {}", head_word(head), end_word(at_start));
        let Some((run_on, se)) = freed(&read, at_start, &what, &mut notes) else {
            continue;
        };
        let c = blade.head(head);
        let [back, face] = [c.farthest(0.0), c.farthest(thickness_mm)];
        let key = |value| Key::Blade { head, value };
        let mut set = |value, v: f64| changes.push(Change::new(cal, key(value), Some((v, se))));
        // The compiler runs a cut on past a square corner until it meets the neighbouring
        // slit's far side on each face, less how early it starts or short it stops there.
        if at_start {
            set(BladeValue::Lead(Surface::Face), face - run_on);
            if back - c.lead_mm[0] > run_on + 1e-9 {
                set(BladeValue::Lead(Surface::Back), back - run_on);
            }
        } else {
            set(BladeValue::Shortfall(Surface::Face), run_on - face);
            if back + c.shortfall_mm[0] > run_on + 1e-9 {
                set(BladeValue::Shortfall(Surface::Back), run_on - back);
            }
        }
    }
    Proposal {
        routine: Routine::RunOns,
        readings: combs.len(),
        scatter_mm: f64::NAN,
        changes,
        suspects: Vec::new(),
        notes,
    }
    .with_effects(cal, thickness_mm)
}

/// Each comb's first tooth to come free, if the corner frees at `need(head, at_start)` mm: the
/// readings a machine would show. For the tests, and for trying a plan.
pub fn simulate(plan: &Plan, need: impl Fn(Head, bool) -> f64) -> HashMap<String, f64> {
    let listed = plan
        .program
        .meta
        .as_ref()
        .and_then(|m| m.pointer("/calibration/readings"))
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    listed
        .iter()
        .filter_map(|m| {
            let name = m.get("name")?.as_str()?.to_owned();
            let head = if m.get("head")?.as_str()? == "bevel" {
                Head::Bevel
            } else {
                Head::Straight
            };
            let at_start = m.get("at")?.as_str()? == "start";
            let values: Vec<f64> = m
                .get("values")?
                .as_array()?
                .iter()
                .filter_map(Value::as_f64)
                .collect();
            let n = need(head, at_start);
            let tooth = values
                .iter()
                .position(|&r| r >= n - 1e-9)
                .map_or(0, |j| j + 1);
            Some((name, tooth as f64))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::comb_readings;
    use crate::proposal::Verdict;
    use crate::{Edit, Why};

    fn key(s: &str) -> Key {
        s.parse().expect(s)
    }

    /// What `cal`'s compiler runs on by at a square corner on this board: the run-on that frees
    /// it, in the model.
    fn need(cal: &Calibration, t: f64) -> impl Fn(Head, bool) -> f64 {
        let blade = cal.blade();
        move |h, at_start| blade.head(h).run_on(t)[usize::from(!at_start)]
    }

    #[test]
    fn eight_combs_of_seven_teeth_graded_round_the_files_run_ons() {
        let p = plan(&Calibration::new(), &Stock::offcut(1.5)).expect("a plan");
        let cuts: Vec<&program::Cut> = p
            .program
            .ops
            .iter()
            .filter_map(|o| match o {
                program::Op::Cut(c) => Some(c),
                _ => None,
            })
            .collect();
        assert_eq!((cuts.len(), p.readings.len()), (64, 8));
        let pads = p
            .cut_with
            .pads_on(crate::plan::OFFCUT_MM, crate::plan::OFFCUT_MM);
        for c in &cuts {
            assert!(
                !pads.iter().any(|pad| pad.hit(c.path_mm[0], c.path_mm[1])),
                "{:?}",
                c.id
            );
            let inside = |q: &[f64; 2]| q.iter().all(|v| (21.0..=135.0).contains(v));
            assert!(c.path_mm.iter().all(inside), "{:?}", c.path_mm);
        }
        // The bevel's end comb centred on its run-on on 1.5 mm board, 3.8 mm past the corner.
        let sheet = p.sheet("plan", simulate(&p, |_, _| 0.0)).expect("a sheet");
        let read = comb_readings(&sheet).expect("combs");
        let b_end = read
            .iter()
            .find(|c| c.name == "B.end.back")
            .expect("listed");
        assert!((b_end.values[3] - 3.8).abs() < 1e-9, "{:?}", b_end.values);
        assert!((b_end.values[0] - 2.6).abs() < 1e-9 && (b_end.values[6] - 5.0).abs() < 1e-9);
        // An end comb's reference comes after its teeth, a start comb's before.
        let order: Vec<&str> = cuts.iter().filter_map(|c| c.id.as_deref()).collect();
        let at = |id: &str| order.iter().position(|o| *o == id).expect(id);
        assert!(at("B.end.back.ref") > at("B.end.back.7"));
        assert!(at("S.start.front.ref") < at("S.start.front.1"));
        assert!(p.check("B.end.back", 0.0).is_ok() && p.check("B.end.back", 7.0).is_ok());
        assert!(p.check("B.end.back", 8.0).is_err() && p.check("B.end.back", 2.5).is_err());
    }

    /// A machine whose reach differs from the file's: the combs it would free, read back, give
    /// run-ons within a tooth of its own, never short of them.
    #[test]
    fn the_combs_recover_a_machines_reach() {
        let file = Calibration::new();
        let why = Why::typed("2026-11-16T16:00:00Z");
        let truth = file
            .set(
                &[
                    Edit::set(key("blade.bevel.shortfall.face"), 1.9),
                    Edit::set(key("blade.bevel.lead.face"), 1.5),
                    Edit::set(key("blade.straight.shortfall.face"), 1.8),
                    Edit::set(key("blade.straight.lead.face"), 0.9),
                ],
                &why,
            )
            .expect("a machine");
        let t = 1.5;
        let p = plan(&file, &Stock::offcut(t)).expect("a plan");
        let sheet = p
            .sheet("stage 2", simulate(&p, need(&truth, t)))
            .expect("a sheet");
        let proposal = propose(&comb_readings(&sheet).expect("combs"), &file, t);
        assert!(proposal.changes.iter().all(|c| c.verdict != Verdict::Large));
        let applied = proposal
            .apply(&file, &proposal.chosen(), &why)
            .expect("applied");
        let (got, want) = (need(&applied, t), need(&truth, t));
        for (h, at_start) in [
            (Head::Bevel, false),
            (Head::Bevel, true),
            (Head::Straight, false),
            (Head::Straight, true),
        ] {
            let (g, w) = (got(h, at_start), want(h, at_start));
            assert!(
                g >= w - 1e-9 && g - w <= 0.3 + 1e-9,
                "{h:?} start {at_start}: {g} against {w}"
            );
        }
        assert_eq!(applied.history()[0].routine.as_deref(), Some("runons"));
        // In words: the bevel's cuts run on one tooth further at each corner.
        let end = proposal
            .changes
            .iter()
            .find(|c| c.key == key("blade.bevel.shortfall.face"))
            .expect("proposed");
        assert_eq!(
            end.effect,
            "On 1.5 mm board, the bevel head's cuts run on 0.30 mm further at each corner."
        );
    }

    /// A comb whose teeth all fall short says so, and moves the next one past it.
    #[test]
    fn a_comb_that_misses_moves_the_next_one() {
        let file = Calibration::new();
        let p = plan(&file, &Stock::offcut(1.5)).expect("a plan");
        let base = need(&file, 1.5);
        let short = p
            .sheet(
                "stage 2",
                simulate(&p, |h, s| {
                    base(h, s) + if h == Head::Bevel && !s { 2.0 } else { 0.0 }
                }),
            )
            .expect("a sheet");
        let proposal = propose(&comb_readings(&short).expect("combs"), &file, 1.5);
        let c = proposal
            .changes
            .iter()
            .find(|c| c.key == key("blade.bevel.shortfall.face"))
            .expect("proposed");
        assert_eq!(c.verdict, Verdict::Large);
        assert!(
            (c.after.expect("a value") - (c.before + 2.4)).abs() < 1e-3,
            "{c:?}"
        );
        assert!(
            proposal
                .notes
                .iter()
                .any(|n| n.contains("cut stage 2 again"))
        );
    }
}
