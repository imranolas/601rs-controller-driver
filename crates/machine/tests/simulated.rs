//! Lowered jobs run on the simulator, in both forms: GMC's, and one stream per cut with every
//! capability on (the simulator follows the spec, so it has them all).
//!
//! The test drives the simulator as the runner will, but no more: it switches to the mode each
//! command needs, waits until the controller is idle where the job says so, and lets head
//! settles pass. Bytes take the time the serial line takes to carry them, and it honours the
//! controller's flow control (`#C7,1`: 0x03 when its buffer is nearly full, 0x02 when there is
//! room), as the runner must for a cut that streams its path during a long travel. Any error the simulator raises fails the test, so a mode switch while the
//! carriage still runs (`*028`), a command in the wrong mode, or a target outside the envelope
//! would. After every wait, the carriage must be where the last vector sent it; at the end,
//! parked with the clamps open.

use std::path::Path;
use std::time::Duration;

use machine::{Capabilities, Job, Profile, Step, lower};
use program::Program;
use protocol::{Command, Mode};
use sim::{LINE_BYTES_PER_SEC, SimConfig, SimCore};

const TICK: Duration = Duration::from_millis(5);

fn fixture(name: &str) -> Program {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/programs")
        .join(format!("{name}.json"));
    Program::from_json(&std::fs::read_to_string(path).expect("fixture reads")).expect("a program")
}

/// Errors in the simulator's output (`*NNN`), or nothing.
fn errors(bytes: &[u8]) -> Vec<String> {
    String::from_utf8_lossy(bytes)
        .split('\r')
        .filter(|l| l.starts_with('*'))
        .map(str::to_owned)
        .collect()
}

struct Run {
    sim: SimCore,
    now: Duration,
    mode: u8,
    /// Where the last vector sent the carriage, steps.
    target: [i64; 3],
    /// The controller asked us to hold the stream (0x03), until it says 0x02.
    held: bool,
    problems: Vec<String>,
}

impl Run {
    fn note(&mut self, what: &str, bytes: &[u8]) {
        for &b in bytes {
            match b {
                0x03 => self.held = true,
                0x02 => self.held = false,
                _ => {}
            }
        }
        for e in errors(bytes) {
            self.problems.push(format!("{what}: {e}"));
        }
    }

    fn line(&mut self, line: &str) {
        let bytes = line.len() + 1;
        self.pass(Duration::from_secs_f64(bytes as f64 / LINE_BYTES_PER_SEC));
        let out = self.sim.handle_line(line, self.now);
        self.note(line, &out);
    }

    fn send(&mut self, c: &Command) {
        if c.required_mode() == Some(Mode::Cutting) {
            let mut waited = Duration::ZERO;
            while self.held {
                self.pass(TICK);
                waited += TICK;
                assert!(waited < Duration::from_secs(60), "held for good");
            }
        }
        if let Some(m) = c.required_mode().map(Mode::code) {
            if m != self.mode {
                self.line(&format!("@M{m}"));
                self.mode = m;
            }
        }
        match *c {
            Command::Vector { x, y, z, .. } => self.target = [x.into(), y.into(), z.into()],
            Command::Step { x, y, z } => {
                self.target = [
                    self.target[0] + i64::from(x),
                    self.target[1] + i64::from(y),
                    self.target[2] + i64::from(z),
                ];
            }
            _ => {}
        }
        self.line(&c.to_string());
    }

    fn pass(&mut self, d: Duration) {
        let end = self.now + d;
        while self.now < end {
            self.now = (self.now + TICK).min(end);
            let out = self.sim.tick(self.now);
            self.note("tick", &out);
        }
    }

    fn wait_idle(&mut self) {
        let mut waited = Duration::ZERO;
        while self.sim.is_moving() {
            self.pass(TICK);
            waited += TICK;
            assert!(waited < Duration::from_secs(120), "never idle");
        }
        let at = self.sim.snapshot(self.now).position;
        if at != self.target {
            self.problems
                .push(format!("idle at {at:?}, sent to {:?}", self.target));
        }
    }
}

/// Run a job; the problems found, and where it ended (position, outputs).
fn run(job: &Job, profile: &Profile) -> (Vec<String>, [i64; 3], u8) {
    let park = [
        i64::from(profile.park_steps[0]),
        i64::from(profile.park_steps[1]),
        0,
    ];
    let sim = SimCore::new(SimConfig {
        homed_at_start: true,
        start_position: park,
        ..SimConfig::default()
    });
    let mut r = Run {
        sim,
        now: Duration::ZERO,
        mode: 0,
        target: park,
        held: false,
        problems: Vec::new(),
    };
    r.line("#C7,1");
    for step in job.steps() {
        match step {
            Step::Send(c) => r.send(c),
            Step::WaitIdle => r.wait_idle(),
            Step::Settle(ms) => r.pass(Duration::from_millis(u64::from(*ms))),
            Step::Hold { .. } => {}
        }
    }
    r.wait_idle();
    let s = r.sim.snapshot(r.now);
    (r.problems, s.position, s.outputs)
}

fn check(name: &str, caps: Capabilities) {
    let profile = Profile {
        capabilities: caps,
        ..Profile::gunnar_601()
    };
    let job = lower(&fixture(name), &profile).expect("lowers");
    let (problems, at, outputs) = run(&job, &profile);
    assert!(
        problems.is_empty(),
        "{name} {caps:?}:\n  {}",
        problems.join("\n  ")
    );
    let park = profile.park_steps;
    assert_eq!(
        at,
        [i64::from(park[0]), i64::from(park[1]), 0],
        "{name}: not parked"
    );
    assert_eq!(
        outputs, profile.neutral_outputs,
        "{name}: clamps or heads left on"
    );
}

#[test]
fn gmcs_form_runs_on_the_simulator() {
    for name in ["g_multi_C10", "g_multi_C12", "g_oval_mount"] {
        check(name, Capabilities::default());
    }
}

#[test]
fn one_stream_per_cut_runs_on_the_simulator() {
    for name in ["g_multi_C10", "g_multi_C12", "g_oval_mount"] {
        check(name, Capabilities::all());
        check(
            name,
            Capabilities {
                compact_paths: true,
                ..Capabilities::default()
            },
        );
        check(
            name,
            Capabilities {
                stream_head: true,
                ..Capabilities::default()
            },
        );
    }
}

/// With every capability on, a run of cuts with the same settings is one stream: the job
/// has no wait until idle between them. (The runner still lets each cut end before sending
/// the next, so that it reports cuts as they end.)
#[test]
fn cuts_with_the_same_settings_share_a_stream() {
    let profile = Profile {
        capabilities: Capabilities::all(),
        ..Profile::gunnar_601()
    };
    let job = lower(&fixture("g_multi_C10"), &profile).expect("lowers");
    let waits = |op: &Vec<Step>| op.iter().filter(|s| **s == Step::WaitIdle).count();
    let bevel_ops: Vec<&Vec<Step>> = job.ops[1..9].iter().collect();
    // The first bevel cut waits for the clamp; the others stream on.
    assert!(
        bevel_ops[1..].iter().all(|op| waits(op) == 0),
        "{:?}",
        bevel_ops.iter().map(|o| waits(o)).collect::<Vec<_>>()
    );
    let gmc = lower(&fixture("g_multi_C10"), &Profile::gunnar_601()).expect("lowers");
    let all_waits = |j: &Job| j.steps().filter(|s| **s == Step::WaitIdle).count();
    assert!(
        all_waits(&job) * 4 < all_waits(&gmc),
        "{} waits, GMC's form {}",
        all_waits(&job),
        all_waits(&gmc)
    );
}
