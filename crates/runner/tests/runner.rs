//! The runner against the simulator, over a real controller session: configuration, homing,
//! GMC's own jobs (`testdata/programs`), pause, cancel, holds, stop-now and faults.

use std::path::Path;
use std::sync::mpsc;
use std::time::Duration;

use machine::Profile;
use program::{Clamp, ClampSide, Cut, Head, Op, Pause, Program, Resume};
use runner::{
    Config, Error, Event, Heads, MachineState, OpSteps, Outcome, Request, Runner, StepKind,
};
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

/// A straight cut run on past the board's left edge, as a clean-up line to the edge is: the
/// straight head is 6.4 mm right of the carriage, so the carriage goes about 7.2 mm below zero,
/// into the room homing leaves before the switch (`#OX,300`). It runs, and meets no switch.
#[test]
fn a_cut_past_the_boards_left_edge_meets_no_switch() {
    let (mut r, sim) = ready();
    let mut program = fixture("g_regular_mount");
    program.ops = vec![
        Op::Clamp(Clamp {
            id: None,
            label: None,
            side: ClampSide::Left,
            turned: Vec::new(),
        }),
        Op::Cut(Cut {
            id: Some("to the edge".into()),
            label: None,
            head: Head::Straight,
            path_mm: vec![[200.0, 300.0], [-1.0, 300.0]],
            speed_pct: None,
        }),
    ];
    let job = machine::lower(&program, r.profile()).expect("inside the envelope");
    let leftmost = job
        .commands()
        .filter_map(|c| match c {
            protocol::Command::Vector { x, .. } => Some(*x),
            _ => None,
        })
        .min()
        .expect("vectors");
    assert!(
        leftmost < -250,
        "the carriage goes below zero: {leftmost} steps"
    );
    let (result, events) = run_with(&mut r, &program, |_, _| {});
    assert_eq!(result.expect("runs"), Outcome::Finished, "{events:?}");
    assert!(parked(&sim), "{:?}", sim.snapshot());
}

/// A calibration applied between runs: the next run is placed by the new profile, without
/// reconnecting or homing again, and nothing is written to the controller.
#[test]
fn a_new_profile_between_runs_places_the_next_run_and_keeps_homing() {
    let (mut r, sim) = ready();
    let mut calibrated = Profile::gunnar_601();
    calibrated.straight.offset_mm[0] += 0.2;
    calibrated.bevel.offset_mm[1] -= 0.1;
    assert_eq!(
        r.set_profile(calibrated.clone()).expect("taken"),
        Vec::new()
    );
    assert!(!r.needs_homing().expect("reads"), "homing kept");
    assert_eq!(r.profile(), &calibrated);
    let program = fixture("g_multi_C10");
    let before = sim.received().len();
    let (result, _) = run_with(&mut r, &program, |_, _| {});
    assert_eq!(result.expect("runs"), Outcome::Finished);
    let sent: Vec<String> = sim.received()[before..]
        .iter()
        .filter(|l| l.starts_with('L'))
        .cloned()
        .collect();
    let lowered = |p: &Profile| -> Vec<String> {
        machine::lower(&program, p)
            .expect("lowers")
            .commands()
            .filter(|c| matches!(c, protocol::Command::Vector { .. }))
            .map(ToString::to_string)
            .collect()
    };
    assert_eq!(sent, lowered(&calibrated));
    assert_ne!(sent, lowered(&Profile::gunnar_601()));
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
            turned: Vec::new(),
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

/// A stop from another thread the moment a cut starts lands as the runner switches the
/// controller to mode 2 for the cut, and cuts the switch short (an app, 14 Nov: four times in
/// four). The controller had switched; the runner, believing mode 1, sent the heads up and
/// clamps released (`$A02`) unswitched, and the controller refused it (`*001`), leaving the
/// clamps closed and the stop reported unsafe.
#[test]
fn a_stop_as_a_cut_starts_is_made_safe() {
    for op in [1, 2, 3] {
        let (mut r, sim) = ready();
        let stop = r.stop_handle();
        let (result, _) = run_with(&mut r, &fixture("g_multi_C10"), |e, _| {
            if matches!(e, Event::Op { index, .. } if *index == op) {
                let stop = stop.clone();
                std::thread::spawn(move || stop.brake().expect("brakes"));
            }
        });
        assert!(
            matches!(result, Err(Error::Failed { safe: true, .. })),
            "op {op}: {result:?}"
        );
        let s = sim.snapshot();
        assert_eq!(
            s.outputs,
            r.profile().neutral_outputs,
            "op {op}: heads up, clamps released"
        );
        assert!(!s.moving);
    }
}

/// A second stop while the first is being made safe (an operator pressing again) latched
/// again, and the heads-up-clamps-released after it failed on the latch: still made safe.
#[test]
fn a_second_stop_while_making_safe_is_made_safe_too() {
    for second_ms in [2, 20] {
        let (mut r, sim) = ready();
        let stop = r.stop_handle();
        let (result, _) = run_with(&mut r, &fixture("g_multi_C10"), |e, _| {
            if matches!(e, Event::Op { index: 2, .. }) {
                let stop = stop.clone();
                std::thread::spawn(move || {
                    stop.brake().expect("brakes");
                    std::thread::sleep(Duration::from_millis(second_ms));
                    stop.brake().expect("brakes again");
                });
            }
        });
        assert!(
            matches!(result, Err(Error::Failed { safe: true, .. })),
            "{second_ms} ms: {result:?}"
        );
        // The second stop may land after the run has ended: let it, then look.
        std::thread::sleep(Duration::from_millis(50));
        let s = sim.snapshot();
        assert_eq!(s.outputs, r.profile().neutral_outputs, "{second_ms} ms");
        assert!(!s.moving);
    }
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

/// A failure that is not a stop still brakes through the stop lane, which waits for no reply:
/// here the controller stops answering mid-cut, so a brake sent as a request never went out.
#[test]
fn a_failure_brakes_without_waiting_for_replies() {
    let (mut r, sim) = ready();
    let mut sent_before = 0;
    let (result, _) = run_with(&mut r, &fixture("g_multi_C10"), |e, _| {
        if matches!(e, Event::Op { index: 3, .. }) {
            sent_before = sim.received().len();
            sim.set_reply_delay(Duration::from_secs(5));
        }
    });
    assert!(
        matches!(result, Err(Error::Failed { op: Some(3), .. })),
        "{result:?}"
    );
    assert!(
        sim.received()[sent_before..].iter().any(|l| l == "@B"),
        "braked"
    );
}

/// A stop made while nothing runs (Ctrl-C at a prompt) latches the session: the machine
/// wants homing, which clears it, rather than every request failing as stopped.
#[test]
fn a_stop_while_idle_wants_homing() {
    let (mut r, sim) = ready();
    r.stop_handle().brake().expect("brakes");
    assert!(r.needs_homing().expect("reads"));
    let (refused, _) = run_with(&mut r, &fixture("g_regular_mount"), |_, _| {});
    assert!(matches!(refused, Err(Error::NotHomed)), "{refused:?}");
    r.home().expect("homes");
    assert!(!r.needs_homing().expect("reads"));
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
/// control on, and cuts with the same settings streamed one after another. A cut is reported
/// done once it has ended, and a pause after it sends nothing of the next.
#[test]
fn one_stream_per_cut_runs_and_pauses() {
    let profile = Profile {
        capabilities: machine::Capabilities::all(),
        ..Profile::gunnar_601()
    };
    let (mut r, sim) = ready_with(profile);
    let mut sent_at_done = None;
    let (result, events) = run_with(&mut r, &fixture("g_multi_C10"), |e, tx| match e {
        Event::OpDone { index: 4 } => {
            assert!(!sim.snapshot().moving, "cut 4 is over when reported done");
            sent_at_done = Some(vectors(&sim));
            tx.send(Request::Pause).expect("sends");
        }
        Event::Paused { .. } => {
            assert_eq!(Some(vectors(&sim)), sent_at_done, "nothing of cut 5 yet");
            tx.send(Request::Resume).expect("sends");
        }
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

/// A runner on the simulator at `speedup`, reading the machine's state every `state_every`,
/// configured and homed.
fn live(speedup: f64, state_every: Duration) -> (Runner, Sim) {
    live_with(Profile::gunnar_601(), speedup, state_every)
}

fn live_with(profile: Profile, speedup: f64, state_every: Duration) -> (Runner, Sim) {
    let (link, sim) = sim::spawn(
        SimConfig {
            homed_at_start: false,
            ..SimConfig::default()
        },
        speedup,
    );
    let (ctl, _version) =
        controller::connect(link, controller::Config::default()).expect("connects");
    let mut r = Runner::new(
        ctl,
        profile,
        Config {
            speedup,
            state_every,
            ..Config::default()
        },
    );
    r.configure().expect("configures");
    r.home().expect("homes");
    (r, sim)
}

fn states(events: &[Event]) -> Vec<&MachineState> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::State(s) => Some(s),
            _ => None,
        })
        .collect()
}

/// In GMC's form (the head lowered by `$A` between the travel and the cut) and in the one-stream
/// form (by an `A` inside the cut's stream, which only `@N` says has run).
#[test]
fn the_machine_state_is_pushed_live_during_a_run() {
    let streamed = Profile {
        capabilities: machine::Capabilities::all(),
        ..Profile::gunnar_601()
    };
    for profile in [Profile::gunnar_601(), streamed] {
        pushed_live(profile);
    }
}

fn pushed_live(profile: Profile) {
    let (mut r, _sim) = live_with(profile, 5.0, Duration::from_millis(20));
    let subscribed = r.subscribe();
    let program = fixture("g_regular_mount");
    let (result, events) = run_with(&mut r, &program, |_, _| {});
    assert_eq!(result.expect("runs"), Outcome::Finished);
    // The subscriber hears what the run's callback does, in the same order.
    let heard: Vec<Event> = subscribed.try_iter().collect();
    assert_eq!(heard, events);

    let all = states(&events);
    assert!(all.len() > program.ops.len(), "{} states", all.len());
    // During op 1, the window's right edge (bevel, down x = 170.7 from y 271.7 to 67.7): the
    // bevel head down, the left clamp closed, the blade on the line.
    let op1: Vec<&&MachineState> = all.iter().filter(|s| s.op == Some(1)).collect();
    let cutting: Vec<&&MachineState> = op1
        .iter()
        .filter(|s| s.heads == Some(Heads::BevelDown))
        .copied()
        .collect();
    assert!(!cutting.is_empty(), "the head seen down: {op1:?}");
    for s in &cutting {
        assert_eq!(s.clamps, Some(ClampSide::Left));
        assert_eq!(s.head, Head::Bevel);
        let [x, y] = s.at_mm.expect("a position");
        assert!((x - 170.7).abs() < 0.1, "on the line: {x}");
        assert!((67.0..=272.5).contains(&y), "along it: {y}");
    }
    // Coarse progress: op 1's steps done rise, one by one or by the stream, to all of them.
    let steps: Vec<OpSteps> = op1.iter().filter_map(|s| s.steps).collect();
    assert_eq!(steps.len(), op1.len(), "every state in an op has its steps");
    let of = steps[0].of;
    assert!(
        of >= 5,
        "travel, head down, cut, blade out, head up at least: {of}"
    );
    assert!(
        steps.iter().all(|s| s.of == of && s.done <= of),
        "{steps:?}"
    );
    assert!(
        steps.windows(2).all(|w| w[1].done >= w[0].done),
        "{steps:?}"
    );
    assert!(steps.iter().any(|s| s.done > 0 && s.done < of), "{steps:?}");
    // The step running, as it happens: the travel to the edge, then the cut along it.
    let running: Vec<StepKind> = steps.iter().filter_map(|s| s.running).collect();
    let first = |k: StepKind| running.iter().position(|&r| r == k);
    assert!(
        matches!((first(StepKind::Travel), first(StepKind::Cut)), (Some(t), Some(c)) if t < c),
        "{running:?}"
    );
    assert!(
        steps
            .iter()
            .all(|s| s.running != Some(StepKind::Cut) || s.done >= 2),
        "the head is down before the cut: {steps:?}"
    );
    // And all of them by the time it is reported done.
    let done_at = events
        .iter()
        .position(|e| *e == Event::OpDone { index: 1 })
        .expect("op 1 done");
    match &events[done_at - 1] {
        Event::State(s) => assert!(
            matches!(s.steps, Some(OpSteps { done, of: n, .. }) if done == of && n == of),
            "{s:?}"
        ),
        e => panic!("{e:?}"),
    }
    assert!(
        all.iter()
            .filter(|s| s.op.is_none())
            .all(|s| s.steps.is_none())
    );
    // The straight head's turn comes: the border.
    assert!(all.iter().any(|s| s.heads == Some(Heads::StraightDown)));
    // Last before Finished: heads up, clamps open, still, homed, no op.
    let last = &events[events.len() - 2];
    match last {
        Event::State(s) => {
            assert_eq!(s.heads, Some(Heads::Up));
            assert_eq!(s.clamps, Some(ClampSide::None));
            assert!(!s.moving && s.homed);
            assert_eq!(s.op, None);
        }
        e => panic!("{e:?}"),
    }
    assert_eq!(events.last(), Some(&Event::Finished));
}

#[test]
fn between_runs_a_refresh_pushes_only_what_changed() {
    let (mut r, _sim) = live(20.0, Duration::from_millis(20));
    let subscribed = r.subscribe();
    r.clamp(ClampSide::Left).expect("clamps");
    let after_clamp: Vec<Event> = subscribed.try_iter().collect();
    match after_clamp.last() {
        Some(Event::State(s)) => {
            assert_eq!(s.clamps, Some(ClampSide::Left));
            assert_eq!(s.heads, Some(Heads::Up));
        }
        other => panic!("{other:?}"),
    }
    r.refresh().expect("reads");
    r.refresh().expect("reads");
    assert_eq!(subscribed.try_iter().count(), 0, "nothing changed");
}

#[test]
fn with_no_state_reads_nothing_is_pushed() {
    let (mut r, _sim) = live(20.0, Duration::ZERO);
    let (result, events) = run_with(&mut r, &fixture("g_regular_mount"), |_, _| {});
    assert_eq!(result.expect("runs"), Outcome::Finished);
    assert!(states(&events).is_empty());
}
