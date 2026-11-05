//! Fidelity: replay every command GMC sent in the captures into the simulator and require the
//! same replies the real controller gave.
//!
//! Compared: every reply line except
//! - `@X` status (timing-dependent; GMC's piled-up polls make pairing meaningless anyway),
//! - the licence exchange in mode 9 (file transfer is not emulated),
//! - live values that cannot match: positions (`@L<a>…` compares prefix only) and the
//!   controller clock (`&C97,…` compares prefix only).

use std::path::{Path, PathBuf};
use std::time::Duration;

use link::capture::{self, Direction};
use protocol::{Decoder, Frame};
use sim::{SimConfig, SimCore};

fn captures() -> Vec<PathBuf> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/captures");
    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .expect("testdata/captures")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "txt"))
        .collect();
    files.sort();
    files
}

fn significant(line: &str) -> Option<String> {
    if line.starts_with("@X") {
        return None;
    }
    if let Some(rest) = line.strip_prefix("@L") {
        return Some(format!("@L{}…", rest.chars().next().unwrap_or('?')));
    }
    if line.starts_with("&C97,") {
        return Some("&C97,…".to_owned());
    }
    Some(line.to_owned())
}

struct Replay {
    /// (host command that preceded it, reply)
    real: Vec<(String, String)>,
    sim: Vec<(String, String)>,
}

fn replay(path: &Path) -> Replay {
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    let transfers = capture::read_file(path).expect("capture");
    let mut core = SimCore::new(SimConfig {
        instant_motion: true,
        // Only g_init starts from power-on; every other capture starts with a homed machine.
        homed_at_start: name != "g_init.txt",
        ..SimConfig::default()
    });

    let (mut host, mut real_dec, mut sim_dec) = (Decoder::new(), Decoder::new(), Decoder::new());
    let mut out = Replay { real: Vec::new(), sim: Vec::new() };
    let mut last_cmd = String::from("<start>");
    let mut mode9 = false;
    let mut keep_next_real = false; // the ack of `@M9` itself

    for t in &transfers {
        let at = Duration::from_secs(u64::from(t.at_secs));
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
                    for f in sim_dec.push(&core.handle_line(&cmd, at)) {
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
                    let Frame::Line(r) = frame else { continue };
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

#[test]
fn simulator_answers_like_the_real_controller() {
    let mut report = String::new();
    let mut compared = 0;
    for path in captures() {
        let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        let r = replay(&path);
        compared += r.real.len();
        // Capture boundaries: recording can start while one GMC poll is outstanding (its reply
        // has no command in the file) and can stop before the last reply arrives. So allow
        // skipping one leading real reply and one missing reply at the end; nothing else.
        let matches = |skip: usize| {
            let real = &r.real[skip.min(r.real.len())..];
            let n = real.len().min(r.sim.len());
            real.len().abs_diff(r.sim.len()) <= 1 && real[..n].iter().zip(&r.sim[..n]).all(|((_, a), (_, b))| a == b)
        };
        if matches(0) || matches(1) {
            continue;
        }
        let first_diff = r.real.iter().zip(&r.sim).position(|((_, a), (_, b))| a != b);
        let i = first_diff.unwrap_or(r.real.len().min(r.sim.len()));
        report += &format!(
            "\n{name}: {} real vs {} simulated replies; first difference at #{i}\n",
            r.real.len(),
            r.sim.len()
        );
        for j in i.saturating_sub(3)..(i + 3) {
            let real = r.real.get(j).map_or("—".into(), |(c, l)| format!("{c:>16} → {l:?}"));
            let sim = r.sim.get(j).map_or("—".into(), |(c, l)| format!("{c:>16} → {l:?}"));
            report += &format!("  #{j:<5} real {real:<44} sim {sim}\n");
        }
    }
    assert!(report.is_empty(), "simulator diverges from the real controller:{report}");
    assert!(compared > 1000, "too few replies compared ({compared})");
}
