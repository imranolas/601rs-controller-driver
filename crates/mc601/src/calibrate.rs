//! `mc601 calibrate`: the machine-and-head calibration, from raw cuts, at the terminal. The
//! sheet, where its cuts went and the fit are the `calibrate` crate's; this keeps the session
//! folders, takes the readings and prints the fit.
//!
//! - `sheet FILE` writes the calibration sheet (`testdata/calibration/calibration_2.json`): a
//!   straight strip block, a bevel window and four overcut combs on a small offcut, in carriage
//!   coordinates, cut raw by `mc601 run` ([`machine::Profile::raw`]).
//! - `fit PROGRAM LOG READINGS [PROGRAM LOG READINGS ...]` reads each cut's carriage steps from
//!   the run log (what was sent, whatever profile it was cut with) and the caliper readings, and
//!   fits by least squares, pooling every sheet given. `--profile FILE` writes the profile with
//!   the fitted head offsets.
//!
//! The model is in `calibrate::fit`.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use ::calibrate::{CombReading, Face, Fit, SentCut, Sheet, Unknown, fit_sheets};
use link::Direction;
use machine::Profile;
use program::{Head, Op, Program};
use serde_json::Value;

use crate::config::tidy;
use crate::operator::{NO_ANSWER, Operator};
use crate::session::Session;
use crate::style::{DIM, GOOD, WARN, paint};
use crate::target::Res;

// ------------------------------------------------------------------ the sheet

/// `mc601 calibrate sheet`: the sheet's program, written to `out`.
pub fn sheet(out: &Path) -> Res<bool> {
    let program = ::calibrate::sheet::program(&crate::calibration::machine());
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
    let program = ::calibrate::sheet::program(&crate::calibration::machine());
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

/// A reading this many of its own scatters off the model, judged by the rest, is likely a
/// misread (`calibrate::Leftover::t`).
const SUSPECT_T: f64 = 3.5;

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
    let profile = crate::calibration::machine();
    let text = std::fs::read_to_string(&log).map_err(|e| format!("{}: {e}", log.display()))?;
    let thickness = program.board.thickness_mm;
    let sheet = Sheet {
        label: tidy(dir),
        program,
        cuts: cuts_from_log(&text, &profile),
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

/// `mc601 calibrate fit`: fit session folders and loose sheets together.
pub fn fit(dirs: &[PathBuf], sheets: &[[PathBuf; 3]], profile_out: Option<&Path>) -> Res<bool> {
    let profile = crate::calibration::machine();
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
        let flag = if l.t.is_some_and(|t| t.abs() > SUSPECT_T) {
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
