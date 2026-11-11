//! The runner against the simulator, over a real controller session: configuration, homing,
//! GMC's own jobs (`testdata/programs`), pause, cancel, holds, stop-now and faults.

use std::path::Path;
use std::sync::mpsc;
use std::time::Duration;

use machine::Profile;
use program::{Clamp, ClampSide, Cut, Head, Op, Pause, Program, Resume};
use runner::{Config, Error, Event, Outcome, Request, Runner};
use sim::{Sim, SimConfig};

/// The simulator runs this much faster than life; the runner's host waits keep up.
const SPEEDUP: f64 = 20.0;

fn start() -> (Runner, Sim) {
    start_with(Profile::gunnar_601())
}

fn start_with(profile: Profile) -> (Runner, Sim) {
    let (link, sim) = sim::spawn(
        SimConfig {
            homed_at_start: false,
            ..SimConfig::default()
        },
        SPEEDUP,
    );
    let (ctl, _version) =
        controller::connect(link, controller::Config::default()).expect("connects");
    let runner = Runner::new(
        ctl,
        profile,
        Config {
            speedup: SPEEDUP,
            ..Config::default()
        },
    );
    (runner, sim)
}

/// Configured and homed.
fn ready() -> (Runner, Sim) {
    ready_with(Profile::gunnar_601())
}

fn ready_with(profile: Profile) -> (Runner, Sim) {
    let (mut r, sim) = start_with(profile);
    r.configure().expect("configures");
    r.home().expect("homes");
    (r, sim)
}

fn fixture(name: &str) -> Program {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/programs")
        .join(format!("{name}.json"));
    Program::from_json(&std::fs::read_to_string(path).expect("fixture reads")).expect("a program")
}

fn vectors(sim: &Sim) -> usize {
    sim.received().iter().filter(|l| l.starts_with('L')).count()
}

fn parked(sim: &Sim) -> bool {
    let p = Profile::gunnar_601();
    let s = sim.snapshot();
    s.position == [i64::from(p.park_steps[0]), i64::from(p.park_steps[1]), 0]
        && s.outputs == p.neutral_outputs
}

/// Run with requests sent from the event callback by `respond`.
fn run_with(
    r: &mut Runner,
    program: &Program,
    mut respond: impl FnMut(&Event, &mpsc::Sender<Request>),
) -> (Result<Outcome, Error>, Vec<Event>) {
    let (tx, rx) = mpsc::channel();
    let mut seen = Vec::new();
    let result = r.run(program, &rx, &mut |e| {
        respond(&e, &tx);
        seen.push(e);
    });
    (result, seen)
}

#[test]
fn configure_writes_what_differs_and_then_nothing() {
    let (mut r, _sim) = start();
    let written = r.configure().expect("configures");
    assert!(
        !written.is_empty(),
        "the 601 boots without some of the profile's configuration"
    );
    assert_eq!(r.configure().expect("configures again"), Vec::new());
}

#[test]
fn nothing_runs_on_an_unhomed_machine() {
    let (mut r, sim) = start();
    r.configure().expect("configures");
    let (result, events) = run_with(&mut r, &fixture("g_regular_mount"), |_, _| {});
    assert!(matches!(result, Err(Error::NotHomed)), "{result:?}");
    assert!(events.is_empty());
    assert_eq!(vectors(&sim), 0);
}

/// Homing ends as GMC's does: the V-groove switch pushed to normal cutting, then the park.
#[test]
fn homing_resets_the_vgroove_switch_and_parks() {
    let (mut r, sim) = ready();
    let after_homing: Vec<String> = sim
        .received()
        .into_iter()
        .skip_while(|l| !l.starts_with("$H"))
        .filter(|l| l.starts_with('L'))
        .collect();
    assert_eq!(
        after_homing,
        [
            "L20,X0,Y40600,Z749",
            "L20,X0,Y41800,Z749",
            "L20,X0,Y40600,Z749",
            "L20,X58860,Y7923,Z0",
        ]
    );
    assert!(parked(&sim));
    assert!(!r.needs_homing().expect("reads"));
}

#[test]
fn an_impossible_program_is_refused_before_anything_moves() {
    let (mut r, sim) = ready();
    let mut p = fixture("g_regular_mount");
    if let Op::Cut(c) = &mut p.ops[1] {
        c.path_mm = vec![[50.0, 50.0], [100.0, 50.0], [100.0, 100.0]]; // a 90° corner in one cut
    }
    let before = vectors(&sim);
    let (result, _) = run_with(&mut r, &p, |_, _| {});
    assert!(
        matches!(
            result,
            Err(Error::Invalid(machine::Error::SharpCorner { .. }))
        ),
        "{result:?}"
    );
    assert_eq!(vectors(&sim), before);
}

#[test]
fn gmcs_job_runs_to_the_park() {
    let (mut r, sim) = ready();
    let program = fixture("g_multi_C10");
    let homing = vectors(&sim);
    let (result, events) = run_with(&mut r, &program, |_, _| {});
    assert_eq!(result.expect("runs"), Outcome::Finished);
    let ops = program.ops.len();
    assert!(
        matches!(events.first(), Some(Event::Started { ops: n, estimate_s }) if *n == ops && *estimate_s > 10.0)
    );
    assert_eq!(
        events
            .iter()
            .filter(|e| matches!(e, Event::Op { .. }))
            .count(),
        ops
    );
    assert_eq!(
        events
            .iter()
            .filter(|e| matches!(e, Event::OpDone { .. }))
            .count(),
        ops
    );
    assert_eq!(events.last(), Some(&Event::Finished));
    assert!(parked(&sim), "{:?}", sim.snapshot());
    // Every vector the lowering produced reached the controller.
    let job = machine::lower(&program, r.profile()).expect("lowers");
    let sent = job
        .commands()
        .filter(|c| matches!(c, protocol::Command::Vector { .. }))
        .count();
    assert_eq!(vectors(&sim) - homing, sent);
}

#[test]
fn pause_resume_and_cancel_between_cuts() {
    let (mut r, sim) = ready();
    let (tx, rx) = mpsc::channel();
    tx.send(Request::Pause).expect("sends");
    let mut events = Vec::new();
    let result = r.run(&fixture("g_regular_mount"), &rx, &mut |e| {
        match &e {
            Event::Paused { .. } => tx.send(Request::Resume).expect("sends"),
            Event::OpDone { index: 2 } => tx.send(Request::Cancel).expect("sends"),
            _ => {}
        }
        events.push(e);
    });
    assert_eq!(result.expect("runs"), Outcome::Cancelled { next: 3 });
    assert!(events.contains(&Event::Paused { next: 0 }));
    assert!(events.contains(&Event::Resumed));
    assert_eq!(events.last(), Some(&Event::Cancelled { next: 3 }));
    assert!(parked(&sim), "{:?}", sim.snapshot());
}

/// A cut, a hold for the green button (or the operator), and another cut.
fn with_a_hold(resume: Resume) -> Program {
    let mut p = fixture("g_regular_mount");
    p.ops = vec![
        Op::Clamp(Clamp {
            id: None,
            label: None,
            side: ClampSide::Left,
        }),
        p.ops[1].clone(),
        Op::Pause(Pause {
            id: Some("flip".into()),
            label: None,
            message: "Flip the board".into(),
            resume,
        }),
        Op::Cut(Cut {
            id: None,
            label: None,
            head: Head::Straight,
            path_mm: vec![[21.0, 21.0], [221.0, 21.0]], // GMC's front edge, 21 mm in
            speed_pct: None,
        }),
    ];
    p
}

#[test]
fn a_pause_op_holds_until_the_green_button() {
    let (mut r, sim) = ready();
    let (result, events) = run_with(&mut r, &with_a_hold(Resume::StartButton), |e, _| {
        if matches!(e, Event::Holding { .. }) {
            std::thread::sleep(Duration::from_millis(30));
            sim.press_start_button();
        }
    });
    assert_eq!(result.expect("runs"), Outcome::Finished);
    assert!(events.iter().any(
        |e| matches!(e, Event::Holding { index: 2, message, .. } if message == "Flip the board")
    ));
    assert!(parked(&sim));
}

#[test]
fn a_pause_op_holds_until_the_operator_resumes() {
    let (mut r, sim) = ready();
    let (result, _) = run_with(&mut r, &with_a_hold(Resume::Operator), |e, tx| {
        if matches!(e, Event::Holding { .. }) {
            sim.press_start_button(); // not enough for an operator hold
            std::thread::sleep(Duration::from_millis(100));
            tx.send(Request::Resume).expect("sends");
        }
    });
    assert_eq!(result.expect("runs"), Outcome::Finished);
}

#[test]
fn stop_now_makes_the_machine_safe_and_wants_homing() {
    let (mut r, sim) = ready();
    let stop = r.stop_handle();
    let program = fixture("g_multi_C10");
    let (result, events) = run_with(&mut r, &program, |e, _| {
        if matches!(e, Event::Op { index: 3, .. }) {
            let stop = stop.clone();
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(15));
                stop.brake().expect("brakes");
            });
        }
    });
    match result {
        Err(Error::Failed {
            op: Some(3),
            safe: true,
            ..
        }) => {}
        other => panic!("{other:?}"),
    }
    assert!(matches!(
        events.last(),
        Some(Event::Failed {
            op: Some(3),
            safe: true,
            ..
        })
    ));
    let s = sim.snapshot();
    assert_eq!(
        s.outputs,
        r.profile().neutral_outputs,
        "heads up, clamps released"
    );
    assert!(!s.moving);
    // Not again until homed.
    assert!(r.needs_homing().expect("reads"));
    let (again, _) = run_with(&mut r, &program, |_, _| {});
    assert!(matches!(again, Err(Error::NotHomed)), "{again:?}");
    r.home().expect("homes");
    let (after, _) = run_with(&mut r, &fixture("g_regular_mount"), |_, _| {});
    assert_eq!(after.expect("runs"), Outcome::Finished);
    assert!(parked(&sim));
}

#[test]
fn a_fault_during_a_cut_makes_the_machine_safe() {
    let (mut r, sim) = ready();
    let (result, _) = run_with(&mut r, &fixture("g_regular_mount"), |e, _| {
        if matches!(e, Event::Op { index: 2, .. }) {
            sim.inject_error(33); // the X+ limit switch
        }
    });
    assert!(
        matches!(
            result,
            Err(Error::Failed {
                op: Some(2),
                safe: true,
                ..
            })
        ),
        "{result:?}"
    );
    assert_eq!(sim.snapshot().outputs, r.profile().neutral_outputs);
    assert!(r.needs_homing().expect("reads"));
}

/// A read that fails while nothing moves is the controller's failure, not a failed run: the
/// runner made nothing safe, and the machine stays homed.
#[test]
fn a_failed_read_is_not_a_failed_run() {
    let (mut r, sim) = ready();
    sim.disconnect();
    match r.position() {
        Err(Error::Controller(_)) => {}
        other => panic!("{other:?}"),
    }
    assert!(matches!(r.needs_homing(), Err(Error::Controller(_))));
}

#[test]
fn manual_park_and_clamps() {
    let (mut r, sim) = start();
    assert!(matches!(r.park(), Err(Error::NotHomed)));
    r.configure().expect("configures");
    r.home().expect("homes");
    r.clamp(ClampSide::Left).expect("clamps");
    assert_eq!(sim.snapshot().outputs, 0x42);
    r.park().expect("parks");
    let p = r.profile().park_steps;
    assert_eq!(
        sim.snapshot().position,
        [i64::from(p[0]), i64::from(p[1]), 0]
    );
    assert_eq!(sim.snapshot().outputs, 0x42, "the clamps stay as they were");
    r.clamp(ClampSide::None).expect("releases");
    assert_eq!(sim.snapshot().outputs, 0x02);
}

#[test]
fn gmcs_oval_runs_as_one_motion() {
    let (mut r, sim) = ready();
    let (result, _) = run_with(&mut r, &fixture("g_oval_mount"), |_, _| {});
    assert_eq!(result.expect("runs"), Outcome::Finished);
    assert!(parked(&sim));
    let received = sim.received();
    assert!(received.iter().any(|l| l == "$A41"), "the slow head-down");
    assert!(vectors(&sim) > 1100);
}

/// The one-stream form, with every capability the simulator has: the controller's flow
/// control on, cuts with the same settings streamed one after another, and a pause still
/// waiting for the cut in hand to end.
#[test]
fn one_stream_per_cut_runs_and_pauses() {
    let profile = Profile {
        capabilities: machine::Capabilities::all(),
        ..Profile::gunnar_601()
    };
    let (mut r, sim) = ready_with(profile);
    let (result, events) = run_with(&mut r, &fixture("g_multi_C10"), |e, tx| match e {
        Event::OpDone { index: 4 } => tx.send(Request::Pause).expect("sends"),
        Event::Paused { .. } => tx.send(Request::Resume).expect("sends"),
        _ => {}
    });
    assert_eq!(result.expect("runs"), Outcome::Finished);
    assert!(events.contains(&Event::Paused { next: 5 }));
    assert!(parked(&sim), "{:?}", sim.snapshot());
    let received = sim.received();
    assert!(
        !received.iter().any(|l| l == "#C7,0"),
        "flow control left on (the 601 boots with it on)"
    );
    assert!(
        received.iter().any(|l| l.starts_with("L21,")),
        "travel on its own entry"
    );
}
