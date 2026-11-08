//! `gunnar`: command-line tool.
//!
//! ```text
//! gunnar ports                          list serial ports
//! gunnar capture <g_*.txt>              decode a GMC serial capture
//! gunnar probe <port|sim> [--log F]     read-only questions to the controller
//! gunnar check <port|sim> [--stage S] [--speed N] [--log F]
//!                                       staged hardware checks (S = 2..7, a range 3-5, or all)
//! ```
//!
//! `sim` instead of a port runs against the simulator. Every byte is logged with timestamps
//! (default `gunnar-<cmd>-<time>.log`).

mod check;
mod operator;
mod probe;
mod report;
mod target;

use std::io::{self, IsTerminal};
use std::process::ExitCode;

use link::Direction;
use link::capture;
use link::serial::PortInfo;
use link::tap::printable;

use crate::operator::{Confirm, Operator};
use crate::report::Report;
use crate::target::Res;

const USAGE: &str = "usage:
  gunnar ports
  gunnar capture <g_*.txt>
  gunnar probe <port|sim> [--log FILE]
  gunnar check <port|sim> [--stage 2..7|A-B|all] [--speed STEPS_PER_S] [--log FILE]";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("ports") => ports(),
        Some("capture") if args.len() == 2 => decode_capture(&args[1]),
        Some(cmd @ ("probe" | "check")) if args.len() >= 2 => run(cmd, &args[1], &args[2..]),
        _ => {
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn ports() -> Res<bool> {
    let all = link::serial::available().map_err(|e| format!("cannot list serial ports: {e}"))?;
    let shown = visible_ports(&all);
    if shown.is_empty() {
        println!("no serial ports found");
    }
    for p in shown {
        let hint = if p.usb { "  ← USB-serial" } else { "" };
        println!("{}{hint}", p.name);
    }
    Ok(true)
}

/// The ports worth offering. macOS lists every serial device twice: `/dev/cu.X` (call-out,
/// the one a host opens) and `/dev/tty.X` (dial-in, which waits for carrier detect). Only that
/// `tty.` twin is hidden; every other name (`/dev/ttyUSB0`, `/dev/ttyACM0`, `COM4`, …) is shown.
fn visible_ports(all: &[PortInfo]) -> Vec<&PortInfo> {
    let has_callout_twin = |name: &str| {
        name.strip_prefix("/dev/tty.").is_some_and(|dev| {
            all.iter()
                .any(|p| p.name.strip_prefix("/dev/cu.") == Some(dev))
        })
    };
    all.iter().filter(|p| !has_callout_twin(&p.name)).collect()
}

struct Options {
    log: Option<String>,
    stages: Vec<u8>,
    speed: u32,
}

fn parse_options(rest: &[String]) -> Res<Options> {
    let mut o = Options {
        log: None,
        stages: (2..=7).collect(),
        speed: 1200,
    };
    let mut it = rest.iter();
    while let Some(flag) = it.next() {
        let value = it
            .next()
            .ok_or_else(|| format!("{flag} needs a value\n{USAGE}"))?;
        match flag.as_str() {
            "--log" => o.log = Some(value.clone()),
            "--speed" => {
                o.speed = value
                    .parse()
                    .ok()
                    .filter(|s| (100..=4000).contains(s))
                    .ok_or_else(|| {
                        format!("--speed {value}: give 100..4000 steps/s (2.5–100 mm/s)")
                    })?;
            }
            "--stage" => o.stages = parse_stages(value)?,
            _ => return Err(format!("unknown option {flag}\n{USAGE}").into()),
        }
    }
    Ok(o)
}

/// `all`, one stage (`3`) or a rising range (`3-5`), within 2..7.
fn parse_stages(v: &str) -> Res<Vec<u8>> {
    const STAGES: std::ops::RangeInclusive<u8> = 2..=7;
    let bad = || {
        format!("--stage {v}: give 2..7, a range such as 3-5, or all (stage 1 is `gunnar probe`)")
    };
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
        return Err(bad().into());
    }
    Ok(stages)
}

fn run(cmd: &str, port: &str, rest: &[String]) -> Res<bool> {
    let opts = parse_options(rest)?;
    // Every step that moves waits for the operator. Without a terminal there is nobody to
    // answer, and piped input (`yes '' | gunnar check …`) would answer "go" for them.
    if cmd == "check" && !target::is_sim_port(port) && !io::stdin().is_terminal() {
        return Err(
            "`gunnar check` drives the machine and asks the operator before every \
                    step; run it from a terminal (stdin is not a TTY)"
                .into(),
        );
    }
    let (ctl, version, target) =
        target::open(port, opts.log.as_deref(), cmd, target::tool_config())?;
    println!("connected to {port}: {version}");
    println!("log: {}", target.log_path.display());
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
                return Ok(true);
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
            speed: opts.speed,
            scale: if target.is_sim() { 0.1 } else { 1.0 },
            quit: false,
            park_confirmed: false,
        };
        let completed = check::run(ctl, &opts.stages, &mut ctx)?;
        report = ctx.r;
        completed
    };
    let ok = report.summary();
    println!("log: {}", target.log_path.display());
    // The transcript is what comes back from the machine; one with gaps must not pass.
    if let Some(e) = target.log.write_error() {
        eprintln!("error: writing the log failed ({e}); the transcript is incomplete");
        return Ok(false);
    }
    Ok(ok && completed)
}

fn decode_capture(file: &str) -> Res<bool> {
    let transfers = capture::read_file(file).map_err(|e| format!("{file}: {e}"))?;
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

    #[test]
    fn stages_are_one_a_rising_range_or_all() {
        assert_eq!(parse_stages("4").ok(), Some(vec![4]));
        assert_eq!(parse_stages("3-5").ok(), Some(vec![3, 4, 5]));
        assert_eq!(parse_stages("all").ok(), Some((2..=7).collect()));
        for bad in ["5-3", "1", "8", "2-9", "x", "3-", ""] {
            let e = parse_stages(bad).expect_err(bad).to_string();
            assert!(e.starts_with("--stage"), "{bad}: {e}");
        }
    }

    #[test]
    fn port_list_hides_only_the_macos_dialin_twin() {
        let all: Vec<PortInfo> = [
            "/dev/cu.usbserial-1420",
            "/dev/tty.usbserial-1420", // twin of the cu. device: hidden
            "/dev/ttyUSB0",            // Linux USB-serial
            "/dev/ttyACM0",
            "/dev/ttyS0",
            "COM4",
            "/dev/tty.debug-console", // no cu. twin: shown
        ]
        .map(|name| PortInfo {
            name: name.to_owned(),
            usb: false,
        })
        .into();
        let shown: Vec<&str> = visible_ports(&all)
            .iter()
            .map(|p| p.name.as_str())
            .collect();
        assert_eq!(
            shown,
            [
                "/dev/cu.usbserial-1420",
                "/dev/ttyUSB0",
                "/dev/ttyACM0",
                "/dev/ttyS0",
                "COM4",
                "/dev/tty.debug-console",
            ]
        );
    }
}
