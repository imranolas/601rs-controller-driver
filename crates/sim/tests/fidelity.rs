//! Fidelity: replay what a host sent into the simulator and require the replies the real
//! controller gave. Two sources:
//! - every GMC capture in `testdata/captures/` (`g_*.txt`, port-monitor exports), and
//! - every `gunnar` session log in `testdata/sessions/` (`*.log`, the `link::tap` transcripts
//!   that `gunnar probe` and `gunnar check` write at the machine; see the README there).
//!
//! Compared: every reply line except
//! - `@X` status (timing-dependent; GMC's piled-up polls make pairing meaningless anyway),
//! - the licence exchange in mode 9 (file transfer is not emulated),
//! - live values, which depend on timing or the physical machine rather than on the commands:
//!   positions (`@L`, `@P`), inputs (`@I`), switches (`@Y`), outputs read back (`@Q`), buffer
//!   level (`@H`), progress (`@N`) and the clock (`&C97`). For these only the prefix, i.e. which
//!   query the line answers, is compared.
//!
//! A difference in a session log is a fact about the box the simulator does not know yet: fix
//! the simulator, or `SimConfig::default()` (which describes the 601), until the log replays.

use std::path::{Path, PathBuf};
use std::time::Duration;

use link::{Direction, capture, tap};
use protocol::{Decoder, Frame, Signal, Status};
use sim::{SimConfig, SimCore};

/// One transfer of a recording, whatever its source.
struct Transfer {
    at: Duration,
    direction: Direction,
    bytes: Vec<u8>,
}

/// The files with `extension` in `testdata/<dir>`, sorted; none if the directory is missing.
fn recordings(dir: &str, extension: &str) -> Vec<PathBuf> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(dir);
    let Ok(entries) = std::fs::read_dir(&dir) else {
        return Vec::new();
    };
    let mut files: Vec<PathBuf> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == extension))
        .collect();
    files.sort();
    files
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default()
}

fn gmc_capture(path: &Path) -> Vec<Transfer> {
    capture::read_file(path)
        .expect("capture")
        .into_iter()
        .map(|t| Transfer {
            at: Duration::from_secs(u64::from(t.at_secs)),
            direction: t.direction,
            bytes: t.bytes,
        })
        .collect()
}

fn session_log(text: &str) -> Vec<Transfer> {
    text.lines()
        .filter_map(tap::parse_line)
        .map(|e| Transfer {
            at: e.at,
            direction: e.direction,
            bytes: e.bytes,
        })
        .collect()
}

/// Was the machine homed when the recording started? Its first status reply says.
fn homed_at_start(transfers: &[Transfer]) -> bool {
    let mut decoder = Decoder::new();
    transfers
        .iter()
        .filter(|t| t.direction == Direction::ControllerToHost)
        .flat_map(|t| decoder.push(&t.bytes))
        .find_map(|frame| match frame {
            Frame::Line(line) => Status::parse(&line),
            _ => None,
        })
        .is_some_and(|s| s.homed)
}

fn significant(line: &str) -> Option<String> {
    if line.starts_with("@X") {
        return None;
    }
    // Live values: compare which query the line answers, not the value.
    for prefix in ["@L", "@P"] {
        if let Some(rest) = line.strip_prefix(prefix) {
            return Some(format!("{prefix}{}…", rest.chars().next().unwrap_or('?')));
        }
    }
    for prefix in ["@I", "@Q"] {
        if line.starts_with(prefix) {
            if let Some((number, _)) = line.split_once(',') {
                return Some(format!("{number},…"));
            }
        }
    }
    for prefix in ["@Y", "@H", "@N", "&C97,"] {
        if line.starts_with(prefix) {
            return Some(format!("{prefix}…"));
        }
    }
    Some(line.to_owned())
}

struct Replay {
    /// (host command that preceded it, reply)
    real: Vec<(String, String)>,
    sim: Vec<(String, String)>,
}

fn replay(transfers: &[Transfer]) -> Replay {
    let mut core = SimCore::new(SimConfig {
        instant_motion: true,
        homed_at_start: homed_at_start(transfers),
        ..SimConfig::default()
    });

    let (mut host, mut real_dec, mut sim_dec) = (Decoder::new(), Decoder::new(), Decoder::new());
    let mut out = Replay {
        real: Vec::new(),
        sim: Vec::new(),
    };
    let mut last_cmd = String::from("<start>");
    let mut mode9 = false;
    let mut keep_next_real = false; // the ack of `@M9` itself

    for t in transfers {
        match t.direction {
            Direction::HostToController => {
                for frame in host.push(&t.bytes) {
                    let Frame::Line(cmd) = frame else { continue }; // 0x06 ACKs of the licence exchange
                    if mode9 && !cmd.starts_with("@M") {
                        continue;
                    }
                    if cmd != "@X" {
                        last_cmd.clone_from(&cmd);
                    }
                    for f in sim_dec.push(&core.handle_line(&cmd, t.at)) {
                        if let Frame::Line(r) = f {
                            if let Some(s) = significant(&r) {
                                out.sim.push((cmd.clone(), s));
                            }
                        }
                    }
                    if cmd == "@M9" {
                        mode9 = true;
                        keep_next_real = true;
                    } else if cmd.starts_with("@M") {
                        mode9 = false;
                    }
                }
            }
            Direction::ControllerToHost => {
                for frame in real_dec.push(&t.bytes) {
                    let r = match frame {
                        // A signal input changed (the operator pressed the green button) and
                        // the box said so (`#C35`): the simulator's input changes too.
                        Frame::Signal(signal) => {
                            let input = match signal {
                                Signal::Input1High => Some((221, true)),
                                Signal::Input1Low => Some((221, false)),
                                Signal::Input2High => Some((222, true)),
                                Signal::Input2Low => Some((222, false)),
                                _ => None,
                            };
                            if let Some((n, high)) = input {
                                core.set_input(n, high);
                            }
                            continue;
                        }
                        Frame::Line(r) => r,
                        Frame::Unknown(_) | Frame::Garbled(_) => continue,
                    };
                    if mode9 && !std::mem::take(&mut keep_next_real) {
                        continue;
                    }
                    if let Some(s) = significant(&r) {
                        out.real.push((last_cmd.clone(), s));
                    }
                }
            }
        }
    }
    out
}

/// `None` if the simulator gave the replies the controller gave; otherwise where they first
/// part ways.
fn compare(name: &str, r: &Replay) -> Option<String> {
    // Recording boundaries: recording can start while one poll is outstanding (its reply has
    // no command in the file) and can stop before the last reply arrives. So allow skipping
    // one leading real reply and one missing reply at the end; nothing else.
    let matches = |skip: usize| {
        let real = &r.real[skip.min(r.real.len())..];
        let n = real.len().min(r.sim.len());
        real.len().abs_diff(r.sim.len()) <= 1
            && real[..n]
                .iter()
                .zip(&r.sim[..n])
                .all(|((_, a), (_, b))| a == b)
    };
    if matches(0) || matches(1) {
        return None;
    }
    let first_diff = r
        .real
        .iter()
        .zip(&r.sim)
        .position(|((_, a), (_, b))| a != b);
    let i = first_diff.unwrap_or(r.real.len().min(r.sim.len()));
    let mut report = format!(
        "\n{name}: {} real vs {} simulated replies; first difference at #{i}\n",
        r.real.len(),
        r.sim.len()
    );
    for j in i.saturating_sub(3)..(i + 3) {
        let real = r
            .real
            .get(j)
            .map_or("—".into(), |(c, l)| format!("{c:>16} → {l:?}"));
        let sim = r
            .sim
            .get(j)
            .map_or("—".into(), |(c, l)| format!("{c:>16} → {l:?}"));
        report += &format!("  #{j:<5} real {real:<44} sim {sim}\n");
    }
    Some(report)
}

#[test]
fn simulator_answers_like_the_real_controller() {
    let mut report = String::new();
    let mut compared = 0;
    for path in recordings("captures", "txt") {
        let r = replay(&gmc_capture(&path));
        compared += r.real.len();
        report += &compare(&file_name(&path), &r).unwrap_or_default();
    }
    assert!(
        report.is_empty(),
        "simulator diverges from the real controller:{report}"
    );
    assert!(compared > 1000, "too few replies compared ({compared})");
}

#[test]
fn simulator_answers_like_the_box_in_every_session_log() {
    let mut report = String::new();
    for path in recordings("sessions", "log") {
        let text = std::fs::read_to_string(&path).expect("session log");
        let r = replay(&session_log(&text));
        report += &compare(&file_name(&path), &r).unwrap_or_default();
    }
    assert!(
        report.is_empty(),
        "the simulator does not answer like the box yet:{report}"
    );
}

/// A session log as `gunnar` writes it: connect, a couple of queries, a rejected parameter.
const SESSION: &str = "\
# gunnar probe /dev/cu.usbserial-1420
    0.001 > @I100⏎
    0.004 < @I100,0⏎
    0.005 > @V⏎
    0.009 < @V2.00/55W 19.10.10 2807001⏎
    0.010 > @M0⏎
    0.012 < ⏎
    0.013 > @X⏎
    0.015 < @X04⏎
    0.016 > &I240⏎
    0.019 < &I240,000h,0,0⏎
    0.020 > #C52,800⏎
    0.022 < *021⏎
# note: anything the tool wants to record
";

#[test]
fn a_session_log_replays_against_the_simulator() {
    let transfers = session_log(SESSION);
    assert!(homed_at_start(&transfers), "@X04: homed");
    let r = replay(&transfers);
    assert_eq!(r.real.len(), 5, "@X is not compared: {:?}", r.real);
    assert_eq!(compare("session", &r), None);
}

#[test]
fn a_reply_the_simulator_does_not_give_is_reported() {
    // As if the box had accepted `#C52`, which the simulator (like the real 601) rejects.
    let log = SESSION.replace("*021⏎", "⏎");
    let report = compare("session", &replay(&session_log(&log))).expect("a difference");
    assert!(report.contains("#C52,800"), "{report}");
}
