//! `mc601 calibrate`: calibrating the heads at the terminal (CALIBRATION.md §5). The plans, where
//! their cuts went, the fit and the proposals are the `calibrate` crate's; this keeps the session
//! folders, takes the readings, shows what they propose, and writes what the operator accepts to
//! the calibration file.
//!
//! - `heads` and `runons` are stages 1 and 2: the plan into a session folder as `routine.json`,
//!   cut as any sheet, then its readings.
//! - `read` takes a session's readings (again, or the rest of them) and offers what they propose.
//! - `fit` fits stage 1 sessions together: a report, and what they would propose.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use ::calibrate::key::Surface;
use ::calibrate::proposal::{Proposal, SUSPECT_T, Verdict};
use ::calibrate::session::Session as RoutineSession;
use ::calibrate::{Check, Fit, Reading, Routine, Stock, Unknown, Why};
use program::Head;

use crate::config::tidy;
use crate::operator::{NO_ANSWER, Operator};
use crate::session::Session;
use crate::style::{BOLD, DIM, GOOD, WARN, paint};
use crate::target::Res;

/// A stage's session (`calibrate::Session`: its plan, how far it was cut, its readings); its
/// program sits beside it under the program's id.
const ROUTINE_FILE: &str = "routine.json";

fn write_json(path: &Path, value: &impl serde::Serialize) -> Res<()> {
    std::fs::write(path, serde_json::to_string_pretty(value)? + "\n")
        .map_err(|e| format!("{}: {e}", path.display()).into())
}

/// The board a stage is cut on: a profile's, or an offcut of the thickness given.
pub fn stock(profile: Option<&str>, thickness: Option<f64>) -> Res<Stock> {
    let cal = crate::calibration::current();
    Ok(match (profile, thickness) {
        (Some(id), t) => {
            let mut s = Stock::profile(&cal, id)?;
            if let Some(t) = t {
                s.thickness_mm = t;
            }
            s
        }
        (None, Some(t)) => Stock::offcut(t),
        (None, None) => {
            return Err("give the offcut's thickness as measured with calipers \
                        (--thickness MM), or its profile (--profile ID)"
                .into());
        }
    })
}

/// `mc601 calibrate heads | runons`: the stage's plan into the session and cut; once it is
/// cut, the run offers to take its readings (`cut::run`).
pub fn stage(port: &str, session: &Session, routine: Routine, stock: &Stock) -> Res<bool> {
    let cal = crate::calibration::current();
    let (plan, what) = match routine {
        Routine::Heads => (::calibrate::heads::plan(&cal, stock)?, "stage 1, the heads"),
        Routine::RunOns => (
            ::calibrate::runons::plan(&cal, stock)?,
            "stage 2, the run-ons",
        ),
    };
    let file = session.dir.join(format!("{}.json", plan.program.id));
    write_json(&file, &plan.program)?;
    let id = session
        .dir
        .file_name()
        .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
    save_routine(&session.dir, &RoutineSession::new(id, &cal, plan.clone()))?;
    anstream::println!(
        "{}",
        paint(
            DIM,
            format!(
                "{what}: {}. {} readings once it is cut.",
                plan.board,
                plan.readings.len()
            )
        )
    );
    crate::cut::run(
        port,
        std::slice::from_ref(&file),
        session,
        crate::cut::Form::default(),
    )
}

/// The stage's session in `dir`, if it is one.
fn load_routine(dir: &Path) -> Res<Option<RoutineSession>> {
    let path = dir.join(ROUTINE_FILE);
    if !path.exists() {
        return Ok(None);
    }
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", tidy(&path)))?;
    Ok(Some(
        RoutineSession::from_json(&text).map_err(|e| format!("{}: {e}", tidy(&path)))?,
    ))
}

/// The stage's session in `dir`.
fn routine(dir: &Path) -> Res<RoutineSession> {
    load_routine(dir)?.ok_or_else(|| {
        format!(
            "{}: not a calibration session (no {ROUTINE_FILE})",
            tidy(dir)
        )
        .into()
    })
}

fn save_routine(dir: &Path, routine: &RoutineSession) -> Res<()> {
    let path = dir.join(ROUTINE_FILE);
    std::fs::write(&path, routine.to_json()).map_err(|e| format!("{}: {e}", tidy(&path)).into())
}

/// After a run in `dir`: how far a stage's program was cut, so its readings can be taken (those
/// whose cuts all finished).
pub fn cut_ended(dir: &Path, cut: ::calibrate::session::Cut) -> Res<()> {
    if let Some(mut r) = load_routine(dir)? {
        r.cut = cut;
        save_routine(dir, &r)?;
    }
    Ok(())
}

/// Calibration session folders under `data`, newest first, each with its session.
fn calibration_sessions(data: &Path) -> Vec<(PathBuf, RoutineSession)> {
    crate::session::list(data)
        .into_iter()
        .filter_map(|s| Some((s.dir.clone(), load_routine(&s.dir).ok()??)))
        .collect()
}

/// The newest calibration session.
pub fn latest_session(data: &Path) -> Res<PathBuf> {
    calibration_sessions(data)
        .into_iter()
        .next()
        .map(|(dir, _)| dir)
        .ok_or_else(|| {
            format!(
                "no calibration session in {}: cut one with `mc601 calibrate heads` or `runons`",
                tidy(data)
            )
            .into()
        })
}

/// Stage 1 sessions with readings, for `fit --all`.
pub fn stage_1_sessions(data: &Path) -> Vec<PathBuf> {
    calibration_sessions(data)
        .into_iter()
        .filter(|(_, r)| r.plan.routine == Routine::Heads && !r.readings.is_empty())
        .map(|(dir, _)| dir)
        .collect()
}

/// `mc601 calibrate read`: take a session's readings one by one, each saved as it is given, then
/// show what they propose for the calibration file, keep it with the fit in the session, and
/// apply it if the operator says so.
pub fn read(dir: &Path, op: &Operator) -> Res<bool> {
    let mut s = routine(dir)?;
    let plan = s.plan.clone();
    // Those whose cuts didn't all finish can't be taken.
    let list: Vec<(&str, &Reading)> = plan
        .steps
        .iter()
        .flat_map(|step| {
            step.readings
                .iter()
                .filter_map(|id| plan.reading(id))
                .map(|r| (step.instruction.as_str(), r))
        })
        .filter(|(_, r)| s.is_available(&r.id))
        .collect();
    let all = plan.readings.len();
    if list.len() < all {
        anstream::println!(
            "{}",
            paint(
                WARN,
                format!(
                    "{} of {all} readings can't be taken: the cut stopped before their cuts were \
                     made",
                    all - list.len()
                )
            )
        );
    }
    let combs = if plan.routine == Routine::RunOns {
        " (a comb: the first tooth to come free, 0 for none)"
    } else {
        ""
    };
    anstream::println!(
        "\n{} {} readings. Type each in mm{combs}. Enter alone keeps the value shown or skips \
         it; q stops, keeping what is typed.",
        paint(BOLD, format!("Readings for {}:", plan.program.id)),
        list.len()
    );
    let mut shown_step = "";
    'readings: for (i, &(step, r)) in list.iter().enumerate() {
        if !step.is_empty() && step != shown_step {
            shown_step = step;
            anstream::println!("\n{}", paint(BOLD, step));
        }
        let how = match r.surface {
            Surface::Back => "back up",
            Surface::Face => "face up",
        };
        let hint = match (r.comb(), r.expected_mm()) {
            (Some(c), _) => format!("tooth 1 to {}, 0: none", c.teeth.len()),
            (None, Some(e)) => format!("about {e:.2}"),
            (None, None) => String::new(),
        };
        loop {
            let shown = s
                .readings
                .get(&r.id)
                .map_or_else(String::new, |v| format!(" [{v}]"));
            let question = format!(
                "[{:>2}/{}] {:10} {}  {}{shown} >",
                i + 1,
                list.len(),
                r.id,
                r.prompt,
                paint(DIM, format!("{how} · {hint}"))
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
            match r.check(v) {
                Err(e) => {
                    anstream::println!("  {}", paint(WARN, e.to_string()));
                    continue;
                }
                Ok(Check::Unusual { off_mm }) => {
                    let keep = op.ask(&format!(
                        "{} {v} is {off_mm:.1} mm from the {:.2} expected. Keep it? (y/n)",
                        paint(WARN, "check:"),
                        r.expected_mm().unwrap_or(f64::NAN)
                    ));
                    if keep.as_deref() != Some("y") {
                        continue;
                    }
                }
                Ok(Check::Fine) => {}
            }
            s.readings.insert(r.id.clone(), v);
            save_routine(dir, &s)?;
            break;
        }
    }
    save_routine(dir, &s)?;
    anstream::println!(
        "{}",
        paint(DIM, format!("readings: {}", tidy(&dir.join(ROUTINE_FILE))))
    );
    if s.readings.is_empty() {
        return Ok(true);
    }
    let cal = crate::calibration::current();
    let p = s.propose(&cal)?;
    // Stage 1's fit in its own terms goes in the session's record; the proposal is shown.
    let fitted = match plan.routine {
        Routine::Heads => report(&::calibrate::heads::fit(&[s.sheet()?], &cal)?),
        Routine::RunOns => String::new(),
    };
    let proposed = proposal_text(&p);
    anstream::print!("{proposed}");
    keep(dir, &(fitted + &proposed))?;
    offer(&p, dir, op)?;
    Ok(true)
}

fn verdict_word(v: Verdict) -> String {
    match v {
        Verdict::WithinNoise => paint(DIM, "within its noise"),
        Verdict::Moved => paint(GOOD, "moved"),
        Verdict::Large => paint(WARN, "large: check the loading and readings"),
        Verdict::Undetermined => paint(DIM, "not fixed by these readings"),
    }
}

/// The proposal, as text for the terminal (styled).
fn proposal_text(p: &Proposal) -> String {
    let mut o = String::new();
    let scatter = if p.scatter_mm.is_finite() {
        format!(", scatter {:.3} mm", p.scatter_mm)
    } else {
        String::new()
    };
    let _ = writeln!(
        o,
        "\n{}  {}",
        paint(BOLD, "proposed for the calibration file"),
        paint(DIM, format!("{} readings{scatter}", p.readings))
    );
    for c in &p.changes {
        let places = if c.unit == "°" { 2 } else { 3 };
        let after = c
            .after
            .map_or_else(|| "–".to_owned(), |a| format!("{a:.places$}"));
        let se =
            c.se.map_or_else(String::new, |se| format!(" ± {se:.places$}"));
        let _ = writeln!(
            o,
            "  {:30} {:>9.places$} → {after:>9} {}{se:10}  {}",
            c.key.to_string(),
            c.before,
            c.unit,
            verdict_word(c.verdict)
        );
        if !c.effect.is_empty() && matches!(c.verdict, Verdict::Moved | Verdict::Large) {
            let _ = writeln!(o, "      {}", paint(DIM, &c.effect));
        }
    }
    for n in &p.notes {
        let _ = writeln!(o, "  {}", paint(DIM, n));
    }
    o
}

/// Offer to write the proposal's moved values (and its large ones, if confirmed) to the
/// calibration file, unless a reading is suspect.
fn offer(p: &Proposal, dir: &Path, op: &Operator) -> Res<()> {
    if !p.suspects.is_empty() {
        let names: Vec<&str> = p.suspects.iter().map(|l| l.name.as_str()).collect();
        anstream::println!(
            "\n{} measure {} again first (`mc601 calibrate read` keeps the other readings); \
             nothing is written while a reading is suspect.",
            paint(WARN, "check:"),
            names.join(", ")
        );
        return Ok(());
    }
    let mut chosen = p.chosen();
    let large: Vec<_> = p
        .changes
        .iter()
        .filter(|c| c.verdict == Verdict::Large)
        .collect();
    if !large.is_empty() {
        let keys: Vec<String> = large.iter().map(|c| c.key.to_string()).collect();
        let answer = op.ask(&format!(
            "{} {} moved further than a machine usually does. Include them? (y/n)",
            paint(WARN, "check:"),
            keys.join(", ")
        ));
        if answer.as_deref() == Some("y") {
            chosen.extend(large.iter().map(|c| c.key.clone()));
        }
    }
    if chosen.is_empty() {
        println!("\nnothing to write: every value fixed is within its noise of the file's");
        return Ok(());
    }
    let cal = crate::calibration::current();
    let answer = op.ask(&format!(
        "Write {} value(s) to the calibration as revision {}? (y/n)",
        chosen.len(),
        cal.revision() + 1
    ));
    if answer.as_deref() != Some("y") {
        println!("nothing written; `mc601 calibrate read` offers it again");
        return Ok(());
    }
    let why = Why {
        session: dir.file_name().map(|n| n.to_string_lossy().into_owned()),
        ..Why::typed(crate::calibration::now())
    };
    let next = p.apply(&cal, &chosen, &why)?;
    crate::calibration::commit(&cal, &next)?;
    Ok(())
}

/// Keep what the readings gave in `dir`, as `fit.txt`.
fn keep(dir: &Path, text: &str) -> Res<()> {
    let path = dir.join("fit.txt");
    let plain = anstream::adapter::strip_str(text).to_string();
    std::fs::write(&path, plain).map_err(|e| format!("{}: {e}", tidy(&path)))?;
    anstream::println!("{}", paint(DIM, format!("fit: {}", tidy(&path))));
    Ok(())
}

/// `mc601 calibrate fit`: stage 1 sessions fitted together, and what they would propose. A
/// report: applying is one session's, by `mc601 calibrate read`, since a blade changed between
/// sessions would make the pool wrong.
pub fn fit(dirs: &[PathBuf]) -> Res<bool> {
    let cal = crate::calibration::current();
    let mut sheets = Vec::new();
    for d in dirs {
        let r = routine(d)?;
        if r.plan.routine != Routine::Heads {
            return Err(format!(
                "{}: stage 2's combs are read, not fitted: `mc601 calibrate read`",
                tidy(d)
            )
            .into());
        }
        let mut sheet = r.sheet()?;
        sheet.label = tidy(d);
        sheets.push(sheet);
    }
    let Some(first) = sheets.first() else {
        return Err("nothing to fit: give stage 1 session folders, or --all".into());
    };
    let thickness = first.plan.program.board.thickness_mm;
    let fit = ::calibrate::heads::fit(&sheets, &cal)?;
    let labels: Vec<&str> = sheets.iter().map(|s| s.label.as_str()).collect();
    anstream::println!(
        "{}",
        paint(
            DIM,
            format!("{} session(s): {}", sheets.len(), labels.join("; "))
        )
    );
    anstream::print!("{}", report(&fit));
    let p = ::calibrate::heads::propose(&fit, &cal, thickness);
    anstream::print!("{}", proposal_text(&p));
    anstream::println!(
        "{}",
        paint(
            DIM,
            "a report: `mc601 calibrate read SESSION` applies one session's proposal"
        )
    );
    Ok(true)
}

/// The fit in its own terms, for the terminal (styled; strip it for a file): each head's values
/// and their standard errors, what the readings fix only together, and each reading less the
/// model.
fn report(fit: &Fit) -> String {
    let mut o = String::new();
    let n = fit.values.iter().filter(|(_, v)| v.is_some()).count();
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
    for h in [Head::Straight, Head::Bevel] {
        if !fit.values.iter().any(|(u, _)| u.head() == h) {
            continue;
        }
        let _ = writeln!(o, "\n{}", paint(BOLD, format!("{h} head")));
        for (u, fixed) in fit.values.iter().filter(|(u, _)| u.head() == h) {
            let _ = match fixed {
                Some((v, se)) => writeln!(
                    o,
                    "  {:16} {v:+9.3} {:5} {}",
                    u.label(),
                    u.unit(),
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
        if let Some((run, se)) = fit.get(Unknown::Run(h)) {
            let lean = run.atan();
            let _ = writeln!(
                o,
                "  a lean of {:.1}° ± {:.1}° from vertical",
                lean.to_degrees(),
                (se * lean.cos().powi(2)).to_degrees()
            );
        }
    }
    let _ = writeln!(o, "\n{}", paint(BOLD, "leftovers (reading less model)"));
    for l in &fit.leftovers {
        // Studentised: in units of what the reading should scatter, given the rest.
        let t =
            l.t.map_or_else(|| "    –".to_owned(), |t| format!("{t:+5.1}"));
        let flag = if l.suspect(SUSPECT_T) {
            paint(WARN, "  look again")
        } else {
            String::new()
        };
        let _ = writeln!(o, "  {:14} {:+.3}  {}{flag}", l.name, l.mm, paint(DIM, t));
    }
    for group in &fit.tied {
        let names: Vec<String> = group
            .iter()
            .map(|u| format!("{} {}", u.head(), u.label()))
            .collect();
        let _ = writeln!(
            o,
            "\n{} {}",
            paint(WARN, "fixed only together, not each on its own:"),
            names.join(", ")
        );
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;
    use ::calibrate::Plan;

    /// Cut `plan` on the simulator to the park, and hold what went down the line to what
    /// lowering the plan gives, command for command, so its fit needs no log.
    fn cut_on_the_simulator(plan: &Plan) {
        const SPEEDUP: f64 = 20.0;
        let (link, sim) = sim::spawn(
            sim::SimConfig {
                homed_at_start: false,
                ..sim::SimConfig::default()
            },
            SPEEDUP,
        );
        let (ctl, _) = controller::connect(link, controller::Config::default()).expect("connects");
        let mut r = runner::Runner::new(
            ctl,
            plan.cut_with.clone(),
            runner::Config {
                speedup: SPEEDUP,
                ..runner::Config::default()
            },
        );
        r.configure().expect("configures");
        r.home().expect("homes");
        let (_requests, rx) = std::sync::mpsc::channel();
        let outcome = r.run(&plan.program, &rx, &mut |_| {}).expect("runs");
        assert_eq!(outcome, runner::Outcome::Finished);
        let received = sim.received();
        let mut sent = received.iter();
        let job = machine::lower(&plan.program, &plan.cut_with).expect("lowers");
        for op in &job.ops {
            for step in op {
                if let machine::Step::Send(c) = step {
                    let line = String::from_utf8_lossy(&c.encode()).trim_end().to_owned();
                    assert!(
                        sent.any(|l| *l == line),
                        "{line}: not sent, or not in order"
                    );
                }
            }
        }
    }

    /// Stage 1 on the simulator, end to end: the plan runs to the park as lowering it says;
    /// readings a machine unlike the file would show, read to a hundredth, fit back to that
    /// machine; and the proposal, applied, makes the file say so.
    #[test]
    fn stage_1_runs_on_the_simulator_and_fits_back_to_the_machine() {
        use ::calibrate::{Calibration, Edit};
        let file = Calibration::new();
        let plan = ::calibrate::heads::plan(&file, &Stock::offcut(1.5)).expect("a plan");
        cut_on_the_simulator(&plan);

        let key = |s: &str| -> ::calibrate::Key { s.parse().expect(s) };
        let why = Why::typed("2026-11-16T15:00:00Z");
        let truth = file
            .set(
                &[
                    Edit::set(key("machine.straight.offset.x"), 6.613),
                    Edit::set(key("machine.straight.offset.y"), 1.668),
                    Edit::set(key("machine.bevel.offset.x"), -71.252),
                    Edit::set(key("machine.bevel.offset.y"), 2.132),
                    Edit::set(key("blade.straight.sideways"), -0.03),
                    Edit::set(key("blade.straight.width"), 0.05),
                    Edit::set(key("blade.bevel.sideways"), 0.6),
                    Edit::set(key("blade.bevel.lean"), 46.0),
                ],
                &why,
            )
            .expect("a machine");
        let all = plan.readings.iter().map(|r| (r.id.clone(), 0.0)).collect();
        let clean = ::calibrate::predict(&plan.sheet("sim", all).expect("a sheet"), &truth)
            .expect("predicted");
        let readings = clean
            .iter()
            .map(|(k, v)| (k.clone(), (v * 100.0).round() / 100.0))
            .collect();
        let sheet = plan.sheet("sim", readings).expect("a sheet");
        let fit = ::calibrate::heads::fit(&[sheet], &file).expect("fits");
        let p = ::calibrate::heads::propose(&fit, &file, 1.5);
        assert!(p.suspects.is_empty(), "{:?}", p.suspects);
        assert_eq!(p.chosen().len(), 8, "{:?}", p.changes);
        let applied = p.apply(&file, &p.chosen(), &why).expect("applied");
        for c in &p.changes {
            let (got, want) = (
                applied.num(&c.key).expect("set"),
                truth.num(&c.key).expect("set"),
            );
            let within = if c.unit == "°" { 0.5 } else { 0.02 };
            assert!(
                (got - want).abs() < within,
                "{}: {got} against {want}",
                c.key
            );
        }
        assert!(proposal_text(&p).contains("blade.bevel.lean"));
    }

    /// Stage 2 on the simulator, end to end: its 64 cuts run to the park as lowering them says;
    /// the teeth a machine with a longer reach would free give its run-ons back, within a tooth
    /// and never short; and the proposal, applied, makes the file say so.
    #[test]
    fn stage_2_runs_on_the_simulator_and_finds_the_machines_reach() {
        use ::calibrate::{Calibration, Edit, End, runons};
        let file = Calibration::new();
        let t = 1.5;
        let plan = runons::plan(&file, &Stock::offcut(t)).expect("a plan");
        cut_on_the_simulator(&plan);
        let key = |s: &str| -> ::calibrate::Key { s.parse().expect(s) };
        let why = Why::typed("2026-11-16T17:00:00Z");
        let truth = file
            .set(
                &[
                    Edit::set(key("blade.bevel.shortfall.face"), 2.0),
                    Edit::set(key("blade.straight.lead.face"), 0.3),
                ],
                &why,
            )
            .expect("a machine");
        let need = |cal: &Calibration| {
            let b = cal.blade();
            move |h, end: End| b.head(h).run_on(t)[end.index()]
        };
        let readings = runons::simulate(&plan, need(&truth));
        let taken = plan
            .readings
            .iter()
            .filter_map(|r| readings.get(&r.id).map(|v| (r, *v)));
        let combs = ::calibrate::combs(taken).expect("combs");
        let p = runons::propose(&combs, &file, t);
        assert!(proposal_text(&p).contains("blade.bevel.shortfall.face"));
        let applied = p.apply(&file, &p.chosen(), &why).expect("applied");
        let (got, want) = (need(&applied), need(&truth));
        for h in [Head::Bevel, Head::Straight] {
            for end in End::BOTH {
                let (g, w) = (got(h, end), want(h, end));
                assert!(
                    g >= w - 1e-9 && g - w <= 0.3 + 1e-9,
                    "{h:?} {end:?}: {g} against {w}"
                );
            }
        }
    }
}
