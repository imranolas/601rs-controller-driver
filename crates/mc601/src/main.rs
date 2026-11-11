//! `mc601`: drive, check and calibrate the Gunnar 601 from the terminal. `mc601 --help` and
//! `mc601 <command> --help` say how.
//!
//! Every command that talks to the machine (or to the simulator, with `--sim`) is a session: a
//! dated folder under the data folder with its log of every byte, copies of the programs, and
//! any readings and fits (`session`). The port comes from `--port`, else the remembered one
//! (`mc601 config`), else the only USB-serial port plugged in (`config`).

mod calibrate;
mod check;
mod config;
mod cut;
mod operator;
mod probe;
mod report;
mod session;
mod style;
mod target;

use std::io::{self, IsTerminal};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{ArgAction, Args, Parser, Subcommand};
use link::Direction;
use link::capture;
use link::tap::printable;

use crate::config::{Config, tidy};
use crate::operator::{Confirm, Operator};
use crate::report::Report;
use crate::session::Session;
use crate::style::{BAD, DIM, GOOD, WARN, paint};
use crate::target::Res;

#[derive(Parser)]
#[command(
    name = "mc601",
    version,
    about = "Drive, check and calibrate the Gunnar 601 mat cutter",
    after_help = "Each session that talks to the machine gets its own folder (log, programs, readings) \
                  under the data folder: `mc601 logs` lists them. The port comes from --port, else \
                  `mc601 config`, else the only USB-serial port plugged in."
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Cut programs, asking before each
    #[command(
        after_help = "Examples:\n  mc601 run job.json\n  mc601 run job.json --sim          \
                            a dry run on the simulator\n  mc601 run calibration_2.json      a \
                            calibration sheet: cut raw, then its readings"
    )]
    Run(RunArgs),
    /// Calibrate the heads: cut the sheet, take its readings, fit them
    #[command(subcommand)]
    Calibrate(CalibrateCommand),
    /// List recent sessions
    Logs(LogsArgs),
    /// Ask the controller read-only questions; nothing moves (stage 1)
    Probe(MachineArgs),
    /// Staged hardware checks, asking before every step (stages 2 to 8)
    #[command(after_help = "Examples:\n  mc601 check --stage 3\n  mc601 check --stage 2-8 --sim")]
    Check(CheckArgs),
    /// List the serial ports
    Ports,
    /// Show or change the remembered port and data folder
    #[command(
        after_help = "Examples:\n  mc601 config\n  mc601 config --port /dev/cu.usbserial-2110\n  \
                            mc601 config --data-dir ~/mc601"
    )]
    Config(ConfigArgs),
    /// Print a GMC serial capture as host and controller lines
    Capture {
        /// The capture, a port-monitor export (`g_*.txt`)
        file: PathBuf,
    },
}

#[derive(Args, Clone)]
struct MachineArgs {
    /// Serial port [default: the remembered one, else the only USB-serial port]
    #[arg(long, short, value_name = "PORT")]
    port: Option<String>,
    /// Use the simulator instead of the machine
    #[arg(long, conflicts_with = "port")]
    sim: bool,
    /// Write the log to FILE instead of the session folder
    #[arg(long, value_name = "FILE")]
    log: Option<PathBuf>,
}

#[derive(Args)]
struct RunArgs {
    /// The programs, cut one after another
    #[arg(required = true, value_name = "PROGRAM")]
    programs: Vec<PathBuf>,
    #[command(flatten)]
    machine: MachineArgs,
    /// Send each cut as GMC does (settings, head and path as separate steps)
    #[arg(long)]
    gmc: bool,
}

#[derive(Args)]
struct CheckArgs {
    #[command(flatten)]
    machine: MachineArgs,
    /// Stages: one (3), a range (3-5) or all
    #[arg(long, default_value = "all", value_parser = parse_stages)]
    stage: Stages,
    /// Test move speed, steps/s (100 to 4000: 2.5 to 100 mm/s)
    #[arg(long, default_value_t = 1200, value_parser = clap::value_parser!(u32).range(100..=4000))]
    speed: u32,
    /// The simulator with its limit switches mapped (stage 7 drives into one)
    #[arg(long, conflicts_with = "port")]
    sim_limits: bool,
}

#[derive(Subcommand)]
enum CalibrateCommand {
    /// Cut the machine-and-head sheet raw, take its readings and fit them
    #[command(
        after_help = "The sheet needs a 1.5 mm offcut of at least 140 × 105 mm in the front-left \
                            corner. What to measure: testdata/calibration/calibration_2.html"
    )]
    Cut(MachineArgs),
    /// Take a calibration session's readings, then fit them [default: the latest session]
    Read {
        /// The session folder
        session: Option<PathBuf>,
    },
    /// Fit calibration sessions together
    #[command(
        after_help = "Examples:\n  mc601 calibrate fit --all\n  mc601 calibrate fit \
                            \"~/Documents/mc601/sessions/2026-11-10 14.32.05 calibrate calibration_2\"\n  \
                            mc601 calibrate fit --sheet calibration_1.json calibration_1.run.log \
                            calibration_1.readings.txt"
    )]
    Fit(FitArgs),
    /// Write the sheet's program to a file, to look at or cut with `mc601 run`
    Sheet {
        /// Where to write it
        out: PathBuf,
    },
}

#[derive(Args)]
struct FitArgs {
    /// Calibration session folders
    sessions: Vec<PathBuf>,
    /// Every calibration session in the data folder that has readings
    #[arg(long)]
    all: bool,
    /// A sheet kept elsewhere: its program, run log and readings
    #[arg(long, num_args = 3, value_names = ["PROGRAM", "LOG", "READINGS"], action = ArgAction::Append)]
    sheet: Vec<PathBuf>,
    /// Also write a profile with the fitted head offsets to FILE
    #[arg(long, value_name = "FILE")]
    profile: Option<PathBuf>,
}

#[derive(Args)]
struct LogsArgs {
    /// How many to list
    #[arg(short = 'n', default_value_t = 20)]
    count: usize,
    /// Print only the latest session's folder (`open "$(mc601 logs --last)"`)
    #[arg(long)]
    last: bool,
}

#[derive(Args)]
struct ConfigArgs {
    /// Remember this port
    #[arg(long, value_name = "PORT")]
    port: Option<String>,
    /// Forget the remembered port (use the only USB-serial port again)
    #[arg(long, conflicts_with = "port")]
    forget_port: bool,
    /// Keep sessions in DIR
    #[arg(long, value_name = "DIR")]
    data_dir: Option<PathBuf>,
}

#[derive(Debug, Clone)]
struct Stages(Vec<u8>);

/// `all`, one stage (`3`) or a rising range (`3-5`), within 2..8.
fn parse_stages(v: &str) -> Result<Stages, String> {
    const STAGES: std::ops::RangeInclusive<u8> = 2..=8;
    let bad =
        || format!("give 2..8, a range such as 3-5, or all (stage 1 is `mc601 probe`), not {v:?}");
    let stage = |s: &str| {
        s.parse()
            .ok()
            .filter(|n| STAGES.contains(n))
            .ok_or_else(bad)
    };
    let stages: Vec<u8> = match v {
        "all" => STAGES.collect(),
        v => match v.split_once('-') {
            Some((a, b)) => (stage(a)?..=stage(b)?).collect(),
            None => vec![stage(v)?],
        },
    };
    if stages.is_empty() {
        return Err(bad());
    }
    Ok(Stages(stages))
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Run(a) => run(a),
        Command::Calibrate(c) => calibrate_command(c),
        Command::Logs(a) => logs(a),
        Command::Probe(m) => machine_check("probe", &m, false, &[], 0),
        Command::Check(a) => machine_check("check", &a.machine, a.sim_limits, &a.stage.0, a.speed),
        Command::Ports => ports(),
        Command::Config(a) => config(a),
        Command::Capture { file } => decode_capture(&file),
    };
    match result {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            anstream::eprintln!("{} {e}", paint(BAD, "error:"));
            ExitCode::FAILURE
        }
    }
}

/// The port to use and a new session for `cmd`, announced in one line.
fn open_session(
    cmd: &str,
    what: &str,
    m: &MachineArgs,
    sim_limits: bool,
) -> Res<(String, Session)> {
    let cfg = Config::load()?;
    let ports = link::serial::available().unwrap_or_default();
    let (port, why) = config::resolve_port(m.port.as_deref(), m.sim, sim_limits, &cfg, &ports)?;
    let session = Session::create(&cfg.data_dir(), cmd, what, &port, m.log.as_deref())?;
    let on = if target::is_sim_port(&port) {
        why
    } else {
        format!("{port} ({why})")
    };
    anstream::println!(
        "{} on {on}  {}",
        paint(style::BOLD, cmd),
        paint(DIM, format!("· session {}", tidy(&session.dir)))
    );
    Ok((port, session))
}

/// Someone must be there to answer before anything moves: piped input would say "go" for them.
fn require_terminal(port: &str, cmd: &str) -> Res<()> {
    if !target::is_sim_port(port) && !io::stdin().is_terminal() {
        return Err(format!(
            "`mc601 {cmd}` drives the machine and asks before it moves: run it from a terminal \
             (stdin is not one)"
        )
        .into());
    }
    Ok(())
}

fn run(a: RunArgs) -> Res<bool> {
    let what: Vec<String> = a
        .programs
        .iter()
        .map(|p| {
            p.file_stem()
                .map_or_else(String::new, |s| s.to_string_lossy().into_owned())
        })
        .collect();
    let (port, session) = open_session("run", &what.join(" "), &a.machine, false)?;
    require_terminal(&port, "run")?;
    finish(&session, cut::run(&port, &a.programs, &session, a.gmc))
}

/// Record how a session ended, and pass the result on.
fn finish(session: &Session, r: Res<bool>) -> Res<bool> {
    session.finish(match &r {
        Ok(true) => "finished",
        Ok(false) => "stopped or failed",
        Err(_) => "failed",
    });
    r
}

fn calibrate_command(c: CalibrateCommand) -> Res<bool> {
    match c {
        CalibrateCommand::Cut(m) => {
            let (port, session) = open_session("calibrate", "calibration_2", &m, false)?;
            require_terminal(&port, "calibrate cut")?;
            finish(&session, calibrate::cut(&port, &session))
        }
        CalibrateCommand::Read { session } => {
            let data = Config::load()?.data_dir();
            let dir = match session {
                Some(d) => d,
                None => calibrate::latest_session(&data)?,
            };
            calibrate::read(&dir, &Operator::new(None))
        }
        CalibrateCommand::Fit(f) => {
            let data = Config::load()?.data_dir();
            let mut dirs = f.sessions;
            if f.all {
                dirs.extend(calibrate::sessions_with_readings(&data));
            }
            let triples: Vec<[PathBuf; 3]> = f
                .sheet
                .chunks(3)
                .map(|c| [c[0].clone(), c[1].clone(), c[2].clone()])
                .collect();
            if dirs.is_empty() && triples.is_empty() {
                return Err(
                    "nothing to fit: give session folders, --all, or --sheet PROGRAM LOG READINGS"
                        .into(),
                );
            }
            calibrate::fit(&dirs, &triples, f.profile.as_deref())
        }
        CalibrateCommand::Sheet { out } => calibrate::sheet(&out),
    }
}

fn logs(a: LogsArgs) -> Res<bool> {
    let data = Config::load()?.data_dir();
    let all = session::list(&data);
    if a.last {
        let last = all.first().ok_or("no sessions yet")?;
        println!("{}", last.dir.display());
        return Ok(true);
    }
    if all.is_empty() {
        println!("no sessions yet in {}", tidy(&data.join("sessions")));
        return Ok(true);
    }
    for s in all.iter().take(a.count) {
        let name = s
            .dir
            .file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        let when: String = name.chars().take(19).collect();
        let outcome = match s.outcome.as_str() {
            "finished" => paint(GOOD, &s.outcome),
            "failed" | "stopped or failed" => paint(BAD, &s.outcome),
            o if o.contains(" fail") && !o.contains(" 0 fail") => paint(BAD, o),
            "ended early" => paint(WARN, &s.outcome),
            o => o.to_owned(),
        };
        anstream::println!(
            "  {}  {:10} {:28} {outcome}  {}",
            paint(DIM, when),
            s.cmd,
            s.what,
            paint(DIM, &s.port)
        );
    }
    anstream::println!(
        "{}",
        paint(DIM, format!("folders: {}", tidy(&data.join("sessions"))))
    );
    Ok(true)
}

fn config(a: ConfigArgs) -> Res<bool> {
    let mut cfg = Config::load()?;
    let changed = a.port.is_some() || a.forget_port || a.data_dir.is_some();
    if let Some(p) = a.port {
        cfg.port = Some(p);
    }
    if a.forget_port {
        cfg.port = None;
    }
    if let Some(d) = a.data_dir {
        cfg.data_dir = Some(d);
    }
    if changed {
        cfg.save()?;
    }
    anstream::println!(
        "{}",
        paint(DIM, format!("config: {}", tidy(&config::path())))
    );
    println!(
        "port: {}",
        cfg.port
            .as_deref()
            .unwrap_or("not set: the only USB-serial port plugged in")
    );
    let env = if std::env::var_os("MC601_DATA").is_some() {
        "  (from $MC601_DATA)"
    } else {
        ""
    };
    println!("data folder: {}{env}", tidy(&cfg.data_dir()));
    Ok(true)
}

fn ports() -> Res<bool> {
    let all = link::serial::available().map_err(|e| format!("cannot list serial ports: {e}"))?;
    let shown = config::visible_ports(&all);
    if shown.is_empty() {
        println!("no serial ports found");
    }
    let remembered = Config::load()?.port;
    for p in shown {
        let mut notes = Vec::new();
        if p.usb {
            notes.push("USB-serial");
        }
        if remembered.as_deref() == Some(p.name.as_str()) {
            notes.push("remembered");
        }
        let note = if notes.is_empty() {
            String::new()
        } else {
            paint(DIM, format!("  ← {}", notes.join(", ")))
        };
        anstream::println!("{}{note}", p.name);
    }
    Ok(true)
}

/// `mc601 probe` and `mc601 check`.
fn machine_check(
    cmd: &str,
    m: &MachineArgs,
    sim_limits: bool,
    stages: &[u8],
    speed: u32,
) -> Res<bool> {
    let what = if cmd == "check" {
        match stages {
            [one] => format!("stage {one}"),
            [a, .., b] => format!("stages {a}-{b}"),
            [] => String::new(),
        }
    } else {
        String::new()
    };
    let (port, session) = open_session(cmd, &what, m, sim_limits)?;
    if cmd == "check" {
        require_terminal(&port, "check")?;
    }
    let r = probe_or_check(cmd, &port, &session, stages, speed);
    session.finish(&match &r {
        Ok((_, tally)) => tally.clone(),
        Err(_) => "failed".to_owned(),
    });
    r.map(|(ok, _)| ok)
}

fn probe_or_check(
    cmd: &str,
    port: &str,
    session: &Session,
    stages: &[u8],
    speed: u32,
) -> Res<(bool, String)> {
    let (ctl, version, target) = target::open(port, &session.log_path, cmd, target::tool_config())?;
    println!("connected: {version}");
    let mut report = Report::new(target.log.clone());
    let completed = if cmd == "probe" {
        probe::run(ctl, &version, &mut report)?;
        true
    } else {
        let op = Operator::new(target.sim.clone());
        if !target.is_sim() {
            println!(
                "\nThis drives the machine. Before you start:\n\
                 - GMC is closed; nobody else is using the machine\n\
                 - scrap board only, the table is clear, blades are up\n\
                 - someone is at the e-stop for every step that moves\n\
                 Each step says what it will do and waits for Enter. Ctrl-C brakes (@B) and exits."
            );
            if op.confirm("Ready?") != Confirm::Go {
                return Ok((true, "not started".to_owned()));
            }
        }
        let stop = ctl.stop_handle();
        ctrlc::set_handler(move || {
            let _ = stop.brake();
            eprintln!("\n  Ctrl-C: braked (@B). {}", check::UNRESTORED);
            std::process::exit(130);
        })?;
        let mut ctx = check::Ctx {
            events: ctl.subscribe(),
            stop: ctl.stop_handle(),
            r: report,
            op,
            speed,
            scale: if target.is_sim() { 0.1 } else { 1.0 },
            quit: false,
            park_confirmed: false,
        };
        let completed = check::run(ctl, stages, &mut ctx)?;
        report = ctx.r;
        completed
    };
    let ok = report.summary();
    anstream::println!("{}", paint(DIM, format!("log: {}", tidy(&target.log_path))));
    // The transcript is what comes back from the machine; one with gaps must not pass.
    if let Some(e) = target.log.write_error() {
        anstream::eprintln!(
            "{} writing the log failed ({e}); the transcript is incomplete",
            paint(BAD, "error:")
        );
        return Ok((false, "log incomplete".to_owned()));
    }
    Ok((ok && completed, report.tally()))
}

fn decode_capture(file: &Path) -> Res<bool> {
    let transfers = capture::read_file(file).map_err(|e| format!("{}: {e}", file.display()))?;
    for t in transfers {
        let arrow = match t.direction {
            Direction::HostToController => '>',
            Direction::ControllerToHost => '<',
        };
        let (h, m, s) = (t.at_secs / 3600, t.at_secs / 60 % 60, t.at_secs % 60);
        println!("{h:02}:{m:02}:{s:02} {arrow} {}", printable(&t.bytes));
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn the_command_line_is_well_formed() {
        Cli::command().debug_assert();
    }

    #[test]
    fn stages_are_one_a_rising_range_or_all() {
        assert_eq!(parse_stages("4").map(|s| s.0), Ok(vec![4]));
        assert_eq!(parse_stages("3-5").map(|s| s.0), Ok(vec![3, 4, 5]));
        assert_eq!(parse_stages("all").map(|s| s.0), Ok((2..=8).collect()));
        for bad in ["5-3", "1", "9", "2-9", "x", "3-", ""] {
            assert!(parse_stages(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn a_run_takes_programs_and_the_simulator() {
        let cli =
            Cli::try_parse_from(["mc601", "run", "a.json", "b.json", "--sim"]).expect("parses");
        let Command::Run(a) = cli.command else {
            panic!("not run")
        };
        assert_eq!(a.programs.len(), 2);
        assert!(a.machine.sim);
        assert!(
            Cli::try_parse_from(["mc601", "run", "--sim"]).is_err(),
            "a program is required"
        );
        assert!(Cli::try_parse_from(["mc601", "run", "a.json", "--sim", "--port", "x"]).is_err());
        let cli = Cli::try_parse_from([
            "mc601",
            "calibrate",
            "fit",
            "--sheet",
            "p",
            "l",
            "r",
            "--all",
        ])
        .expect("parses");
        let Command::Calibrate(CalibrateCommand::Fit(f)) = cli.command else {
            panic!("not fit")
        };
        assert_eq!((f.sheet.len(), f.all), (3, true));
    }
}
