//! `mc601 calibrate`: calibrating the heads at the terminal (CALIBRATION.md §5). The plan,
//! where its cuts went, the fit and the proposal are the `calibrate` crate's; this keeps the
//! session folders, takes the readings, prints the fit and writes what the operator accepts to
//! the calibration file.
//!
//! - `heads` is stage 1: its plan into a session folder, cut as any sheet, then its readings,
//!   the fit, and the changes it proposes, applied if the operator says so.
//! - `read` takes a session's readings (again, or the rest of them) and fits them.
//! - `fit` pools sessions, and the record's sheets (calibrations 1 and 2, cut raw), reading each
//!   cut's carriage steps from its run log.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use ::calibrate::proposal::{Proposal, SUSPECT_T, Verdict};
use ::calibrate::session::Session as RoutineSession;
use ::calibrate::{
    CombReading, Face, Fit, Routine, SentCut, Sheet, Stock, Unknown, Why, fit_sheets,
};
use link::Direction;
use machine::Profile;
use program::{Head, Program};
use serde_json::Value;

use crate::config::tidy;
use crate::operator::{NO_ANSWER, Operator};
use crate::session::Session;
use crate::style::{BOLD, DIM, GOOD, WARN, paint};
use crate::target::Res;

// ------------------------------------------------------------------ stages 1 and 2

/// A stage's session (`calibrate::Session`: its plan, how far it was cut, its readings); its
/// program sits beside it under the program's id. The record's raw sheets keep their readings
/// in `readings.txt`.
const ROUTINE_FILE: &str = "routine.json";
const READINGS_FILE: &str = "readings.txt";

fn write_json(path: &Path, value: &impl serde::Serialize) -> Res<()> {
    std::fs::write(path, serde_json::to_string_pretty(value)? + "\n")
        .map_err(|e| format!("{}: {e}", path.display()).into())
}

/// The board stage 1 is cut on: a profile's, or an offcut of the thickness given.
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

// ------------------------------------------------------------------ the run log

/// The cuts a run log sent, in order, in carriage mm: the bytes the host wrote, read by
/// [`::calibrate::cuts_from_bytes`].
pub fn cuts_from_log(text: &str, profile: &Profile) -> Vec<SentCut> {
    let mut sent = Vec::new();
    for e in text.lines().filter_map(link::tap::parse_line) {
        if e.direction == Direction::HostToController {
            sent.extend(e.bytes);
        }
    }
    ::calibrate::cuts_from_bytes(&sent, profile)
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

fn load_sheet(program: &Path, log: &Path, readings: &Path, profile: &Profile) -> Res<Sheet> {
    let text =
        std::fs::read_to_string(program).map_err(|e| format!("{}: {e}", program.display()))?;
    let parsed = Program::from_json(&text).map_err(|e| format!("{}: {e}", program.display()))?;
    let log_text = std::fs::read_to_string(log).map_err(|e| format!("{}: {e}", log.display()))?;
    Ok(Sheet {
        label: tidy(program),
        program: parsed,
        cuts: cuts_from_log(&log_text, profile),
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

fn text_of(v: &Value, k: &str) -> String {
    v.get(k)
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_owned()
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
    // A straight cut's edge is the same on either face, so it doesn't decide.
    let (back, face) = (
        faces.iter().any(|f| f == "back"),
        faces.iter().any(|f| f == "face"),
    );
    if text_of(m, "kind") == "comb" {
        "by eye, face up"
    } else if back && !face {
        "on the back"
    } else if face && !back {
        "face up"
    } else {
        "either face"
    }
}

/// How far a reading may stray from its expected value before it is questioned, mm, unless the
/// sheet says (`unusual`).
const NEAR_MM: f64 = 1.5;

/// `mc601 calibrate read`: take a session's readings one by one (each saved as it is given),
/// then fit them and keep the fit and a fitted profile in the session. For a routine with a
/// plan (stage 1), the fit's proposal for the calibration file, applied if the operator says so.
pub fn read(dir: &Path, op: &Operator) -> Res<bool> {
    let (_, program, log) = session_files(dir)?;
    let mut routine = load_routine(dir)?;
    let mut list: Vec<Value> = program
        .meta
        .as_ref()
        .and_then(|m| {
            m.pointer("/calibration/readings")
                .or_else(|| m.pointer("/calibration/measurements"))
        })
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    // A stage's readings whose cuts didn't all finish can't be taken.
    if let Some(r) = &routine {
        let all = list.len();
        list.retain(|m| r.is_available(&text_of(m, "name")));
        if list.len() < all {
            anstream::println!(
                "{}",
                paint(
                    WARN,
                    format!(
                        "{} of {all} readings can't be taken: the cut stopped before their cuts \
                         were made",
                        all - list.len()
                    )
                )
            );
        }
    }
    let file = match &routine {
        Some(_) => dir.join(ROUTINE_FILE),
        None => dir.join(READINGS_FILE),
    };
    let mut readings: HashMap<String, f64> = match &routine {
        Some(r) => r.readings.iter().map(|(k, v)| (k.clone(), *v)).collect(),
        None if file.exists() => read_readings(&file)?,
        None => HashMap::new(),
    };
    let combs = if list.iter().any(|m| text_of(m, "kind") == "comb") {
        " (a comb: the first tooth that reached, 0 for none)"
    } else {
        ""
    };
    anstream::println!(
        "\n{} {} readings. Type each in mm{combs}. Enter alone keeps the value shown or skips \
         it; q stops, keeping what is typed.",
        paint(BOLD, format!("Readings for {}:", program.id)),
        list.len()
    );
    let save = |readings: &HashMap<String, f64>, routine: &mut Option<RoutineSession>| -> Res<()> {
        if let Some(r) = routine {
            r.readings = readings.iter().map(|(k, v)| (k.clone(), *v)).collect();
            return save_routine(dir, r);
        }
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
    let mut step = String::new();
    'readings: for (i, m) in list.iter().enumerate() {
        let name = text_of(m, "name");
        if !text_of(m, "step").is_empty() && text_of(m, "step") != step {
            step = text_of(m, "step");
            anstream::println!("\n{}", paint(BOLD, &step));
        }
        let comb = text_of(m, "kind") == "comb";
        let about = m.get("about").and_then(Value::as_f64);
        let near = m.get("unusual").and_then(Value::as_f64).unwrap_or(NEAR_MM);
        let how = m
            .get("how")
            .and_then(Value::as_str)
            .unwrap_or_else(|| face_note(m));
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
            if comb && (v < 0.0 || v > teeth as f64 || v.fract() != 0.0) {
                anstream::println!(
                    "  {}",
                    paint(WARN, format!("give a tooth number, 0 to {teeth}"))
                );
                continue;
            }
            if let (false, Some(a)) = (comb, about) {
                if (v - a).abs() > near {
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
            save(&readings, &mut routine)?;
            break;
        }
    }
    save(&readings, &mut routine)?;
    anstream::println!("{}", paint(DIM, format!("readings: {}", tidy(&file))));
    if readings.is_empty() {
        return Ok(true);
    }
    let profile = crate::calibration::machine();
    let cal = crate::calibration::current();
    let thickness = program.board.thickness_mm;
    // A stage's cuts are where lowering its plan with the profile it was placed with puts them;
    // the record's raw sheets', where their run logs show. Stage 2's combs give their proposal
    // directly; anything with lengths is fitted, and reported, first.
    let (out, fitted, proposal) = match &routine {
        Some(r) if r.plan.routine == Routine::RunOns => {
            let p = r.propose(&cal)?;
            (proposal_text(&p), None, Some(p))
        }
        Some(r) => {
            let (fit, combs) = fit_sheets(&[r.sheet()?])?;
            let (mut out, fitted) = report(&fit, &combs, thickness, &profile);
            let p = r.propose(&cal)?;
            out.push_str(&proposal_text(&p));
            (out, Some(fitted), Some(p))
        }
        None => {
            let text =
                std::fs::read_to_string(&log).map_err(|e| format!("{}: {e}", log.display()))?;
            let sheet = Sheet {
                label: tidy(dir),
                program,
                cuts: cuts_from_log(&text, &profile),
                readings,
            };
            let (fit, combs) = fit_sheets(&[sheet])?;
            let (out, fitted) = report(&fit, &combs, thickness, &profile);
            (out, Some(fitted), None)
        }
    };
    anstream::print!("\n{out}");
    keep(dir, &out, fitted.as_ref())?;
    if let Some(p) = &proposal {
        offer(p, dir, op)?;
    }
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

/// Keep a fit, and the profile with its fitted head offsets if it has one, in `dir`.
fn keep(dir: &Path, report: &str, fitted: Option<&Profile>) -> Res<()> {
    let plain = anstream::adapter::strip_str(report).to_string();
    std::fs::write(dir.join("fit.txt"), plain).map_err(|e| format!("{}: {e}", dir.display()))?;
    if let Some(fitted) = fitted {
        write_json(&dir.join("profile.json"), fitted)?;
    }
    let what = if fitted.is_some() {
        "fit and fitted profile"
    } else {
        "fit"
    };
    anstream::println!(
        "{}",
        paint(DIM, format!("{what}: {}", tidy(&dir.join("fit.txt"))))
    );
    Ok(())
}

/// `mc601 calibrate fit`: fit session folders and loose sheets together.
pub fn fit(dirs: &[PathBuf], sheets: &[[PathBuf; 3]], profile_out: Option<&Path>) -> Res<bool> {
    let profile = crate::calibration::machine();
    let mut all = Vec::new();
    for d in dirs {
        // A stage's session fits from its plan and readings; stage 2's combs aren't fitted.
        if let Some(r) = load_routine(d)? {
            if r.plan.routine == Routine::RunOns {
                return Err(format!(
                    "{}: stage 2's combs are read, not fitted: `mc601 calibrate read`",
                    tidy(d)
                )
                .into());
            }
            let mut s = r.sheet()?;
            s.label = tidy(d);
            all.push(s);
            continue;
        }
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
    let fitted = fit.profile(profile);
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
        if let Some((new, [sx, sy])) = fit.head_offset(h, profile) {
            let now = profile.head(h).offset_mm;
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
            .map(|u| format!("{} {}", head_name(u.head()), u.label()))
            .collect();
        let _ = writeln!(
            o,
            "\n{} {}",
            paint(WARN, "fixed only together, not each on its own:"),
            names.join(", ")
        );
    }
    (o, fitted)
}

#[cfg(test)]
mod tests {
    use super::*;

    const RECORD: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/calibration");

    fn record(name: &str) -> String {
        let path = Path::new(RECORD).join(name);
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
    }

    fn program(name: &str) -> Program {
        Program::from_json(&record(name)).expect("a program")
    }

    /// The profile calibration 1 was cut with, 10 Nov: its readings file says where the heads
    /// were, and the log agrees to the step.
    fn cut_with_on_10_nov() -> Profile {
        let mut p = Profile::gunnar_601();
        p.straight.offset_mm = [6.57, 1.86];
        p.bevel.offset_mm = [-71.50, 1.90];
        p
    }

    use ::calibrate::Plan;

    /// Cut `plan` on the simulator to the park, and hold what went down the line to what
    /// lowering the plan gives, cut for cut, so its fit needs no log.
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
        let sent: Vec<u8> = sim
            .received()
            .iter()
            .flat_map(|l| l.bytes().chain(*b"\r"))
            .collect();
        let lowered =
            ::calibrate::cuts_from_lowering(&plan.program, &plan.cut_with).expect("lowers");
        assert_eq!(::calibrate::cuts_from_bytes(&sent, &plan.cut_with), lowered);
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
        let (fit, _) = fit_sheets(&[plan.sheet("sim", readings).expect("a sheet")]).expect("fits");
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
        use ::calibrate::{Calibration, Edit, runons};
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
            move |h, at_start: bool| b.head(h).run_on(t)[usize::from(!at_start)]
        };
        let readings = runons::simulate(&plan, need(&truth));
        let combs = ::calibrate::comb_readings(&plan.sheet("sim", readings).expect("a sheet"))
            .expect("combs");
        let p = runons::propose(&combs, &file, t);
        assert!(proposal_text(&p).contains("blade.bevel.shortfall.face"));
        let applied = p.apply(&file, &p.chosen(), &why).expect("applied");
        let (got, want) = (need(&applied), need(&truth));
        for h in [Head::Bevel, Head::Straight] {
            for at_start in [false, true] {
                let (g, w) = (got(h, at_start), want(h, at_start));
                assert!(
                    g >= w - 1e-9 && g - w <= 0.3 + 1e-9,
                    "{h:?} {at_start}: {g} against {w}"
                );
            }
        }
    }

    #[test]
    fn the_sheet_is_cut_raw() {
        assert!(crate::cut::is_raw(&::calibrate::sheet::program(
            &Profile::gunnar_601()
        )));
    }

    #[test]
    fn a_log_gives_back_the_cuts_it_sent() {
        let text = "    0.001 > C1⏎\n    0.002 > L21,X400,Y800,Z0⏎\n    0.003 > A4A⏎\n    \
                    0.004 > W250⏎\n    0.005 > L20,X1400,Y800,Z0⏎\n    0.006 > x40,y-80⏎\n    \
                    0.007 > A42⏎\n    0.008 > L21,X0,Y0,Z0⏎\n";
        let cuts = cuts_from_log(text, &Profile::gunnar_601());
        assert_eq!(
            cuts,
            vec![SentCut {
                head: Head::Straight,
                points: vec![[10.0, 20.0], [35.0, 20.0], [36.0, 18.0]],
            }]
        );
    }

    /// Lowering a program again with the profile it was cut with gives exactly what its run log
    /// shows was sent: so an app that keeps no log still knows where every cut went. Two runs of
    /// calibration 1, with two profiles.
    #[test]
    fn the_record_lowers_as_it_was_sent() {
        let p = program("calibration_1.json");
        for (log, cut_with) in [
            ("calibration_1.run.log", cut_with_on_10_nov()),
            ("calibration_1b.run.log", Profile::gunnar_601()),
        ] {
            let logged = cuts_from_log(&record(log), &cut_with);
            let lowered = ::calibrate::cuts_from_lowering(&p, &cut_with).expect("lowers");
            assert_eq!(logged.len(), 16, "{log}");
            assert_eq!(lowered, logged, "{log}");
        }
    }

    /// The record's fit, from its log and from its lowering alike, is what `mc601 calibrate
    /// fit` printed before the fit moved into the `calibrate` crate (15 Nov). The readings are
    /// in hundredths, so every value is an exact half-thousandth: printed to three places, a
    /// value such as -0.1025 shows as -0.102 or -0.103 by the last bit, so they're held exactly.
    #[test]
    fn the_record_fits_as_it_did() {
        let readings =
            read_readings(&Path::new(RECORD).join("calibration_1.readings.txt")).expect("readings");
        let cut_with = cut_with_on_10_nov();
        let p = program("calibration_1.json");
        let from_log = Sheet {
            label: "calibration_1".into(),
            program: p.clone(),
            cuts: cuts_from_log(&record("calibration_1.run.log"), &cut_with),
            readings: readings.clone(),
        };
        let lowered = Sheet::lowered("calibration_1", p, &cut_with, readings).expect("lowers");
        let (fit, _) = fit_sheets(&[from_log]).expect("fits");
        let (again, _) = fit_sheets(&[lowered]).expect("fits");
        assert_eq!(fit, again);

        assert_eq!((fit.rows, fit.values.len()), (20, 12));
        assert!((fit.sigma - 0.112361).abs() < 1e-6, "scatter {}", fit.sigma);
        assert!(fit.tied.is_empty());
        let (s, b) = (Head::Straight, Head::Bevel);
        for (u, want, se) in [
            (Unknown::P(s, 0), -6.1875, 0.05618),
            (Unknown::P(s, 1), -2.1425, 0.05618),
            (Unknown::E(s, false, Face::Any), -0.1025, 0.05618),
            (Unknown::E(s, true, Face::Any), 0.0875, 0.05618),
            (Unknown::Drag(s, true), 1.9675, 0.12562),
            (Unknown::Drag(s, false), 1.2025, 0.12562),
            (Unknown::P(b, 0), 71.6775, 0.05618),
            (Unknown::P(b, 1), -2.3075, 0.05618),
            (Unknown::E(b, false, Face::Back), 0.4525, 0.05618),
            (Unknown::E(b, true, Face::Front), -2.1625, 0.05618),
            (Unknown::Drag(b, true), 2.2225, 0.12562),
            (Unknown::Drag(b, false), 0.6475, 0.12562),
        ] {
            let (v, e) = fit.get(u).expect("settled");
            assert!((v - want).abs() < 1e-6, "{u:?}: {v} against {want}");
            assert!((e - se).abs() < 1e-5, "{u:?}: ± {e} against {se}");
        }
        // The tool's values the record fixes (CALIBRATION.md §3): the straight slit's width and
        // sideways offset, the bevel's run across the 1.5 mm board, and each head's plunge (how
        // much further back a cut starts than it falls short at its end, on the back).
        let el = Unknown::E(s, false, Face::Any);
        let er = Unknown::E(s, true, Face::Any);
        let (ef, eb) = (
            Unknown::E(b, true, Face::Front),
            Unknown::E(b, false, Face::Back),
        );
        for (what, c, want) in [
            ("slit width", vec![(el, 1.0), (er, 1.0)], -0.015),
            ("sideways", vec![(er, 0.5), (el, -0.5)], 0.095),
            ("bevel run", vec![(ef, -1.0), (eb, -1.0)], 1.71),
            (
                "straight plunge",
                vec![
                    (Unknown::Drag(s, true), 1.0),
                    (Unknown::Drag(s, false), -1.0),
                ],
                0.765,
            ),
            (
                "bevel plunge",
                vec![
                    (Unknown::Drag(b, true), 1.0),
                    (Unknown::Drag(b, false), -1.0),
                ],
                1.575,
            ),
        ] {
            let (v, _) = fit.combination(&c).expect("fixed");
            assert!((v - want).abs() < 1e-6, "{what}: {v} against {want}");
        }
        // The profile written with `--profile`, to the micrometre: 6.4125 rounds up there, though
        // the report prints it as 6.412.
        let fitted = fit.profile(&Profile::gunnar_601());
        assert_eq!(fitted.straight.offset_mm, [6.413, 1.868]);
        assert_eq!(fitted.bevel.offset_mm, [-71.452, 2.032]);
    }

    /// The record's fitted head offsets, written into the calibration file, are the machine
    /// everything then drives: the file resolves to the profile the fit writes.
    #[test]
    fn the_records_fit_in_the_file_is_the_machine() {
        use ::calibrate::key::Axis;
        use ::calibrate::{Calibration, Edit, Key, Why};
        let readings =
            read_readings(&Path::new(RECORD).join("calibration_1.readings.txt")).expect("readings");
        let sheet = Sheet::lowered(
            "calibration_1",
            program("calibration_1.json"),
            &cut_with_on_10_nov(),
            readings,
        )
        .expect("lowers");
        let (fit, _) = fit_sheets(&[sheet]).expect("fits");
        let cal = Calibration::new();
        let mut edits = Vec::new();
        for h in [Head::Straight, Head::Bevel] {
            let (mm, se) = fit.head_offset(h, &cal.machine()).expect("settled");
            for (i, axis) in [Axis::X, Axis::Y].into_iter().enumerate() {
                let v = (mm[i] * 1000.0).round() / 1000.0;
                edits.push(Edit::set(Key::HeadOffset(h, axis), v).with_se(se[i]));
            }
        }
        let why = Why {
            routine: Some("calibration_1".into()),
            ..Why::typed("2026-11-10T16:00:00Z")
        };
        let cal = cal.set(&edits, &why).expect("set");
        assert_eq!(cal.machine(), fit.profile(&Profile::gunnar_601()));
        assert_eq!(cal.revision(), 1);
    }
}
