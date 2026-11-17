//! `mc601 run`: cut programs (PROGRAM_FORMAT.md) on the machine through the runner, one after
//! another, asking before each: GMC's own jobs (`testdata/programs`), an app's, and the calibration
//! stages' sheets (`mc601 calibrate`), which are ordinary programs in the board frame.
//!
//! Programs run streamed (the capabilities stages 5, 6 and 8 confirmed on the 601: compact
//! paths, the head in the stream, a travel entry, flow control): each cut's travel, head down,
//! settle, path and head up in mode 2, cut after cut, the stream breaking only where a setting
//! must change in mode 0 (the start speed `#S` between the heads). `--gmc` runs GMC's form.
//!
//! Every program is lowered and checked before anything moves. The machine is configured as the
//! profile says and homed as GMC homes it (offered each time; required if it doesn't know its
//! position). Each program then waits for Enter, with the board it expects. Ctrl-C stops at once
//! (`@B`): the runner raises the heads and releases the clamps, and the machine must be homed
//! again; a second Ctrl-C quits on the spot.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;

use calibrate::session::Cut;
use machine::{Capabilities, Host, Profile};
use program::{Head, Op, Program, Resume};
use runner::{Event, Outcome, Request, Runner};

use crate::config::tidy;
use crate::operator::{Confirm, Operator};
use crate::session::Session;
use crate::style::{BAD, DIM, GOOD, paint};
use crate::target::{self, Res, SIM_SPEEDUP, Target};

/// A program read and checked: its file, and its estimated run time.
struct Loaded {
    file: String,
    program: Program,
    estimate_s: f64,
}

pub(crate) fn read(file: &Path) -> Res<Program> {
    let text = std::fs::read_to_string(file).map_err(|e| format!("{}: {e}", file.display()))?;
    Ok(Program::from_json(&text).map_err(|e| format!("{}: {e}", file.display()))?)
}

/// Whether `program` is a calibration stage's sheet, with readings to take (its plan's).
fn has_readings(program: &Program) -> bool {
    program
        .meta
        .as_ref()
        .and_then(|m| m.get("calibration"))
        .is_some()
}

/// Lower `program` with `profile`.
fn load(file: &str, program: Program, profile: &Profile) -> Res<Loaded> {
    let job = machine::lower(&program, profile).map_err(|e| format!("{file}: {e}"))?;
    let estimate_s = machine::estimate(&job, profile, &Host::default()).total_s;
    Ok(Loaded {
        file: file.to_owned(),
        program,
        estimate_s,
    })
}

/// One line about the program, one about its material, one about its ops.
fn describe(l: &Loaded) -> String {
    let p = &l.program;
    let (b, m) = (&p.board, &p.material);
    let cuts = |head: Head| {
        p.ops
            .iter()
            .filter(|op| matches!(op, Op::Cut(c) if c.head == head))
            .count()
    };
    format!(
        "{} ({}): board {} × {} mm, {} mm thick\n     material {}: speed {} %, head settle {} \
         ms, blade offset straight {}°, bevel {}°\n     {} ops: {} bevel cuts, {} straight \
         cuts; about {:.0} s",
        p.id,
        l.file,
        b.width_mm,
        b.height_mm,
        b.thickness_mm,
        m.name,
        m.speed_pct,
        m.head_settle_ms,
        m.blade_angle_offset_deg.straight,
        m.blade_angle_offset_deg.bevel,
        p.ops.len(),
        cuts(Head::Bevel),
        cuts(Head::Straight),
        l.estimate_s
    )
}

fn op_name(op: &Op) -> String {
    let name = match op {
        Op::Clamp(c) if c.turned.is_empty() => format!("clamp {:?}", c.side).to_lowercase(),
        Op::Clamp(c) => format!(
            "{}, {} turned out of the way",
            format!("clamp {:?}", c.side).to_lowercase(),
            c.turned.join(", ")
        ),
        Op::Cut(c) => {
            let len: f64 = c
                .path_mm
                .windows(2)
                .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                .sum();
            format!(
                "{} cut, {:.0} mm",
                format!("{:?}", c.head).to_lowercase(),
                len
            )
        }
        Op::Move(m) => format!("move to {:.1}, {:.1} mm", m.to_mm[0], m.to_mm[1]),
        Op::Pause(p) => format!("pause: {}", p.message),
    };
    match op.label().or(op.id()) {
        Some(l) => format!("{name} ({l})"),
        None => name,
    }
}

/// How a run sends its cuts, where it differs from the machine profile: to compare with GMC's.
#[derive(Debug, Clone, Copy, Default)]
pub struct Form {
    /// Each cut as GMC sends it: settings, head and path as separate steps.
    pub gmc: bool,
    /// Bevel cuts started as GMC starts them, at full start speed on thick board, not softly.
    pub gmc_bevel_start: bool,
    /// An oval's blade let settle fully before it moves, not GMC's 100 ms.
    pub oval_settle: bool,
}

/// Cut `files` in `session`: each program is copied into the session folder first, and a
/// calibration sheet that finishes offers to take its readings there and then.
pub fn run(port: &str, files: &[PathBuf], session: &Session, form: Form) -> Res<bool> {
    let mut profile = Profile {
        capabilities: if form.gmc {
            Capabilities::default()
        } else {
            Capabilities::all()
        },
        ..crate::calibration::machine()
    };
    if form.gmc_bevel_start {
        profile.soft_start.bevel = false;
        println!("bevel cuts start as GMC's: at full start speed, whatever the board");
    }
    if form.oval_settle {
        profile.bevel.closed_settle_ms = None;
        println!("an oval's blade settles fully before it moves");
    }
    let programs: Vec<Program> = files.iter().map(|f| read(f)).collect::<Res<_>>()?;
    // The session keeps what was cut, under the file's own name (unless it is there already).
    for (k, f) in files.iter().enumerate() {
        if f.parent() == Some(session.dir.as_path()) {
            continue;
        }
        let name = f.file_name().map_or_else(
            || format!("program-{k}.json"),
            |n| n.to_string_lossy().into_owned(),
        );
        let to = session.dir.join(&name);
        let to = if to.exists() {
            session.dir.join(format!("{k}-{name}"))
        } else {
            to
        };
        std::fs::copy(f, &to).map_err(|e| format!("{}: {e}", f.display()))?;
    }
    println!("{}", if form.gmc { "GMC's form" } else { "streamed" });
    let loaded: Vec<Loaded> = files
        .iter()
        .zip(programs)
        .map(|(f, p)| {
            let name = f
                .file_name()
                .map_or_else(|| tidy(f), |n| n.to_string_lossy().into_owned());
            load(&name, p, &profile)
        })
        .collect::<Res<_>>()?;
    for l in &loaded {
        println!("  - {}", describe(l));
    }

    let (ctl, version, Target { sim, log, .. }) = target::open(
        port,
        &session.log_path,
        "run",
        controller::Config::default(),
    )?;
    println!("connected: {version}");
    let op = Operator::new(sim.clone());
    if !op.is_sim() {
        println!(
            "\nThis CUTS. Before you start:\n\
             - GMC is closed; nobody else is using the machine\n\
             - blades in, set for the board; scrap board only; the table otherwise clear\n\
             - someone is at the e-stop\n\
             Each program waits for Enter. Ctrl-C stops at once (@B): heads up, clamps released."
        );
        if op.confirm("Ready?") != Confirm::Go {
            return Ok(true);
        }
    }

    let mut runner = Runner::new(
        ctl,
        profile,
        runner::Config {
            speedup: if op.is_sim() { SIM_SPEEDUP } else { 1.0 },
            ..runner::Config::default()
        },
    );
    let stop = runner.stop_handle();
    let stopping = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&stopping);
    ctrlc::set_handler(move || {
        if flag.swap(true, Ordering::SeqCst) {
            eprintln!("\n  Ctrl-C again: quitting now. Home before the next run.");
            std::process::exit(130);
        }
        let _ = stop.brake();
        eprintln!(
            "\n  Ctrl-C: stopped (@B). The runner raises the heads and releases the clamps; \
             Ctrl-C again quits on the spot."
        );
    })?;

    let written = runner.configure()?;
    if written.is_empty() {
        anstream::println!("{}", paint(DIM, "  configuration: as the profile wants"));
    } else {
        let names: Vec<String> = written.iter().map(|s| format!("{s:?}")).collect();
        println!("  configuration written: {}", names.join(", "));
    }

    let must_home = runner.needs_homing()?;
    let prompt = if must_home {
        "Home (required: the machine doesn't know its position): heads up, Z, then X and Y, \
         the V-groove switch reset up the left side at full speed, then the park. Clear the \
         table."
    } else {
        "Home first? Heads up, Z, then X and Y, the V-groove switch reset up the left side at \
         full speed, then the park. Clear the table. (s = skip: it is homed already)"
    };
    match op.confirm(prompt) {
        Confirm::Go => {
            runner.home()?;
            println!("  homed and parked");
        }
        Confirm::Skip if !must_home => {}
        _ => return Ok(true),
    }

    let mut all_ok = true;
    for l in &loaded {
        let p = &l.program;
        let b = &p.board;
        let turned = p.turned_at_start();
        let turn = if turned.is_empty() {
            String::new()
        } else {
            format!(
                " Turn clamp pads {} out of the way first.",
                turned.join(", ")
            )
        };
        let ask = format!(
            "Cut {}: load a {} × {} mm board, {} mm thick, into the front-left corner against \
             the left clamp.{turn} The heads come down. (about {:.0} s)",
            p.id, b.width_mm, b.height_mm, b.thickness_mm, l.estimate_s
        );
        match op.confirm(&ask) {
            Confirm::Go => {}
            Confirm::Skip => continue,
            Confirm::Quit => break,
        }
        if runner.needs_homing()? {
            if op.confirm("The machine must be homed again first. Home now?") != Confirm::Go {
                break;
            }
            runner.home()?;
        }
        stopping.store(false, Ordering::SeqCst);
        let (tx, rx) = mpsc::channel();
        let ops = p.ops.len();
        let mut done = 0;
        let result = runner.run(p, &rx, &mut |e| match e {
            Event::Started { ops, estimate_s } => {
                println!("  started: {ops} ops, about {estimate_s:.0} s");
            }
            Event::Op { index, .. } => {
                println!("  op {}/{ops}: {}", index + 1, op_name(&p.ops[index]));
            }
            Event::OpDone { index } => done = index + 1,
            // The command line shows ops as they come, not the machine live.
            Event::Resumed | Event::State(_) => {}
            Event::Paused { next } => println!("  paused before op {}", next + 1),
            Event::Holding {
                message, resume, ..
            } => {
                println!("  holding: {message}");
                match resume {
                    Resume::StartButton => println!("  press the green button to go on"),
                    Resume::Operator => {
                        let go = op.confirm("Go on?");
                        let _ = tx.send(if go == Confirm::Go {
                            Request::Resume
                        } else {
                            Request::Cancel
                        });
                    }
                }
            }
            Event::Finished => {
                anstream::println!("  {}", paint(GOOD, "finished, parked, clamps released"));
            }
            Event::Cancelled { next } => {
                println!(
                    "  cancelled before op {}: parked, clamps released",
                    next + 1
                );
            }
            Event::Failed {
                op: failed,
                error,
                safe,
            } => anstream::println!(
                "  {}: {error}; {}",
                paint(
                    BAD,
                    format!(
                        "FAILED{}",
                        failed.map_or(String::new(), |i| format!(" in op {}", i + 1))
                    )
                ),
                if safe {
                    "heads up, clamps released; home before the next run"
                } else {
                    "the machine may NOT be safe: check it before anything else"
                }
            ),
        });
        // A calibration stage's session keeps how far it was cut: what can be read.
        let cut = match &result {
            Ok(Outcome::Finished) => Cut::Done,
            Ok(Outcome::Cancelled { next }) => Cut::Stopped { ops_done: *next },
            Err(_) => Cut::Stopped { ops_done: done },
        };
        crate::calibrate::cut_ended(&session.dir, cut)?;
        if has_readings(p) && cut != Cut::Done {
            anstream::println!(
                "{}",
                paint(
                    DIM,
                    "  the readings whose cuts were made can still be taken: `mc601 calibrate read`"
                )
            );
        }
        match result {
            Ok(Outcome::Finished) => {
                log.note(&format!("finished {}", p.id));
                if has_readings(p) && !op.is_sim() {
                    if op.confirm(
                        "Take this sheet's readings now? (s = later, with `mc601 calibrate read`)",
                    ) == Confirm::Go
                    {
                        crate::calibrate::read(&session.dir, &op)?;
                    }
                } else if has_readings(p) {
                    anstream::println!(
                        "{}",
                        paint(
                            DIM,
                            "  simulator: nothing was cut to measure; `mc601 calibrate read` \
                             takes readings typed in, to try the fit"
                        )
                    );
                }
            }
            Ok(Outcome::Cancelled { .. }) => all_ok = false,
            Err(e) => {
                log.note(&format!("failed {}: {e}", p.id));
                all_ok = false;
                if op.confirm("Go on with the next program?") != Confirm::Go {
                    break;
                }
            }
        }
    }
    anstream::println!("{}", paint(DIM, format!("session: {}", tidy(&session.dir))));
    if let Some(e) = log.write_error() {
        anstream::eprintln!(
            "{} writing the log failed ({e}); the transcript is incomplete",
            paint(BAD, "error:")
        );
        return Ok(false);
    }
    Ok(all_ok)
}
