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

use crate::key::{BladeValue, Key, Surface};
use crate::plan::{Line, Plan, Routine, Step, Stock, program};
use crate::proposal::{Change, Proposal};
use crate::reading::{Comb, CombReading, End, Kind, Reading, Tooth};
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
const ROWS: [(Head, End, f64); 4] = [
    (Head::Bevel, End::End, 38.0),
    (Head::Bevel, End::Start, 66.0),
    (Head::Straight, End::End, 94.0),
    (Head::Straight, End::Start, 122.0),
];
/// Where each row's two combs start (X, mm): the first with its teeth in front of the
/// reference, the second with them behind it.
const SIDES: [(bool, f64); 2] = [(false, 30.0), (true, 90.0)];

const STEP: &str = "Leave the board back up, as it lay on the machine. Flex it along each comb's \
                    reference: a tooth whose cut has gone through to the reference comes free \
                    there. Count the teeth from the left: tooth 1 is the shortest.";

/// One comb's reading, and its cuts: the reference, and the teeth, tooth 1 first.
struct Cut {
    reading: Reading,
    reference: Line,
    teeth: Vec<Line>,
}

fn combs(cal: &Calibration, thickness_mm: f64) -> Result<Vec<Cut>, Error> {
    let blade = cal.blade();
    let mut out = Vec::new();
    for (head, end, y) in ROWS {
        let nominal = blade.head(head).run_on(thickness_mm)[end.index()];
        let run_ons = GRADE.map(|g| ((nominal + g) * 1000.0).round() / 1000.0);
        if run_ons.iter().any(|r| r.abs() > MOST_MM) {
            return Err(Error::Plan(format!(
                "the {head} head's run-on at the {}, {nominal:.2} mm, is beyond what stage 2's \
                 combs can show",
                end.word()
            )));
        }
        for (behind, x0) in SIDES {
            // A point `v` from the reference toward the teeth, at `x`.
            let at = |x: f64, v: f64| if behind { [x, y + v] } else { [x, y - v] };
            // The teeth run toward the back (+Y) at an end with them in front, or at a start
            // with them behind.
            let toward = if behind == (end == End::Start) {
                "back"
            } else {
                "front"
            };
            let letter = if head == Head::Bevel { "B" } else { "S" };
            let id = format!("{letter}.{}.{toward}", end.word());
            let what = format!("{head} {} comb, teeth toward the {toward}", end.word());
            let line = |suffix: &str, label: String, a, b| Line {
                id: format!("{id}.{suffix}"),
                label: format!("{what}, {label}"),
                head,
                a,
                b,
            };
            let teeth: Vec<Line> = run_ons
                .iter()
                .enumerate()
                .map(|(j, &r)| {
                    let x = x0 + PITCH_MM * j as f64;
                    // An end's tooth comes to the reference and runs on past it; a start's
                    // begins past it and runs away.
                    let (a, b) = match end {
                        End::Start => (at(x, -r), at(x, TOOTH_MM)),
                        End::End => (at(x, TOOTH_MM), at(x, -r)),
                    };
                    line(&(j + 1).to_string(), format!("tooth {}", j + 1), a, b)
                })
                .collect();
            // The teeth on the reference's right: it runs +X with them in front, −X behind.
            let (left, right) = (
                [x0 - REFERENCE_PAST_MM, y],
                [x0 + PITCH_MM * 6.0 + REFERENCE_PAST_MM, y],
            );
            let (a, b) = if behind { (right, left) } else { (left, right) };
            let reference = line("ref", "reference".to_owned(), a, b);
            let reading = Reading {
                id: id.clone(),
                prompt: format!(
                    "The {what}: the first tooth to come free at the reference (1 to 7; 0 if \
                     none)"
                ),
                surface: Surface::Back,
                kind: Kind::Tooth(Comb {
                    head,
                    end,
                    reference: reference.id.clone(),
                    teeth: teeth
                        .iter()
                        .zip(run_ons)
                        .map(|(t, run_on_mm)| Tooth {
                            cut: t.id.clone(),
                            run_on_mm,
                        })
                        .collect(),
                }),
                dimension: [reference.a, reference.b],
            };
            out.push(Cut {
                reading,
                reference,
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
        let start = c.reading.comb().is_some_and(|c| c.end == End::Start);
        if start {
            lines.push(c.reference.clone());
        }
        lines.extend(c.teeth.iter().cloned());
        if !start {
            lines.push(c.reference.clone());
        }
    }
    let program = program("stage_2", Routine::RunOns, stock, &lines);
    let cut_with = cal.machine();
    machine::lower(&program, &cut_with)?;
    let readings: Vec<Reading> = combs.into_iter().map(|c| c.reading).collect();
    Ok(Plan {
        routine: Routine::RunOns,
        board: stock.need(),
        program,
        cut_with,
        steps: vec![Step {
            instruction: STEP.to_owned(),
            readings: readings.iter().map(|r| r.id.clone()).collect(),
        }],
        readings,
    })
}

/// The run-on that freed the corner in every direction read, and its standard error (NaN when
/// a comb missed and it is only a bound or a guess). The comb puts it somewhere in the step
/// below the tooth that freed it, as likely anywhere there: a standard error of step ÷ √12, so
/// a value a tooth away has moved and one on the same tooth is within the noise.
fn freed(
    read: &[&CombReading],
    end: End,
    what: &str,
    notes: &mut Vec<String>,
) -> Option<(f64, f64)> {
    let first = read.first()?;
    let n = first.run_ons.len();
    let told: Vec<String> = read
        .iter()
        .map(|c| match c.tooth {
            0 => format!("none on {}", c.id),
            k => format!("tooth {k} on {}", c.id),
        })
        .collect();
    let told = told.join(", ");
    // A run-on in words: how far a cut starts before the corner, or runs past it.
    let words = |r: f64| match (end, r >= 0.0) {
        (End::Start, true) => format!("a cut starting {r:.2} mm before it"),
        (End::Start, false) => format!("a cut starting {:.2} mm after it", -r),
        (End::End, true) => format!("a cut running {r:.2} mm past it"),
        (End::End, false) => format!("a cut stopping {:.2} mm short of it", -r),
    };
    if let Some(none) = read.iter().find(|c| c.tooth == 0) {
        // Every tooth too short: the next comb is centred past the last.
        let last = none.run_ons.last().copied()?;
        let next = last + (last - none.run_ons[n / 2]);
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
        .map(|c| c.run_ons[c.tooth - 1])
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
        .find(|c| c.run_ons[c.tooth - 1] == frees)
        .unwrap_or(first);
    let k = setter.tooth.max(2) - 1;
    Some((
        frees,
        (setter.run_ons[k] - setter.run_ons[k - 1]) / 12f64.sqrt(),
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
    for (head, end, _) in ROWS {
        let read: Vec<&CombReading> = combs
            .iter()
            .filter(|c| c.head == head && c.end == end)
            .collect();
        let what = format!("The {head} head's {}", end.word());
        let Some((run_on, se)) = freed(&read, end, &what, &mut notes) else {
            continue;
        };
        let c = blade.head(head);
        let [back, face] = [c.farthest(0.0), c.farthest(thickness_mm)];
        let key = |value| Key::Blade { head, value };
        let mut set = |value, v: f64| changes.push(Change::new(cal, key(value), Some((v, se))));
        // The compiler runs a cut on past a square corner until it meets the neighbouring
        // slit's far side on each face, less how early it starts or short it stops there.
        match end {
            End::Start => {
                set(BladeValue::Lead(Surface::Face), face - run_on);
                if back - c.lead_mm[0] > run_on + 1e-9 {
                    set(BladeValue::Lead(Surface::Back), back - run_on);
                }
            }
            End::End => {
                set(BladeValue::Shortfall(Surface::Face), run_on - face);
                if back + c.shortfall_mm[0] > run_on + 1e-9 {
                    set(BladeValue::Shortfall(Surface::Back), run_on - back);
                }
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

/// Each comb's first tooth to come free, if the corner frees at `need(head, end)` mm: the
/// readings a machine would show. For the tests, and for trying a plan.
pub fn simulate(plan: &Plan, need: impl Fn(Head, End) -> f64) -> HashMap<String, f64> {
    plan.readings
        .iter()
        .filter_map(|r| {
            let c = r.comb()?;
            let n = need(c.head, c.end);
            let tooth = c
                .teeth
                .iter()
                .position(|t| t.run_on_mm >= n - 1e-9)
                .map_or(0, |j| j + 1);
            Some((r.id.clone(), tooth as f64))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proposal::Verdict;
    use crate::reading::combs;
    use crate::{Edit, Why};

    fn key(s: &str) -> Key {
        s.parse().expect(s)
    }

    /// What `cal`'s compiler runs on by at a square corner on this board: the run-on that frees
    /// it, in the model.
    fn need(cal: &Calibration, t: f64) -> impl Fn(Head, End) -> f64 {
        let blade = cal.blade();
        move |h, end: End| blade.head(h).run_on(t)[end.index()]
    }

    /// The combs as read with `values`.
    fn read(p: &Plan, values: &HashMap<String, f64>) -> Vec<CombReading> {
        combs(
            p.readings
                .iter()
                .filter_map(|r| values.get(&r.id).map(|v| (r, *v))),
        )
        .expect("combs")
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
        let combs = read(&p, &simulate(&p, |_, _| 0.0));
        let b_end = combs.iter().find(|c| c.id == "B.end.back").expect("listed");
        let r = &b_end.run_ons;
        assert!((r[3] - 3.8).abs() < 1e-9, "{r:?}");
        assert!((r[0] - 2.6).abs() < 1e-9 && (r[6] - 5.0).abs() < 1e-9);
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
        let proposal = propose(&read(&p, &simulate(&p, need(&truth, t))), &file, t);
        assert!(proposal.changes.iter().all(|c| c.verdict != Verdict::Large));
        let applied = proposal
            .apply(&file, &proposal.chosen(), &why)
            .expect("applied");
        let (got, want) = (need(&applied, t), need(&truth, t));
        for (h, end, _) in ROWS {
            let (g, w) = (got(h, end), want(h, end));
            assert!(
                g >= w - 1e-9 && g - w <= 0.3 + 1e-9,
                "{h:?} {end:?}: {g} against {w}"
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
        let short = simulate(&p, |h, end| {
            base(h, end)
                + if (h, end) == (Head::Bevel, End::End) {
                    2.0
                } else {
                    0.0
                }
        });
        let proposal = propose(&read(&p, &short), &file, 1.5);
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
