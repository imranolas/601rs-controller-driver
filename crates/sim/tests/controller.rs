//! End-to-end: the real controller layer driving the simulator.

use std::thread;
use std::time::Duration;

use controller::{Axis, Config, Controller, ControllerError, Event, Signal, connect, mode};
use sim::{Sim, SimConfig, spawn};

fn cfg() -> Config {
    Config {
        reply_timeout: Duration::from_millis(200),
        poll_interval: Duration::from_millis(5),
        flow_timeout: Duration::from_secs(5),
        ..Config::default()
    }
}

/// Start a simulator (`speedup` 20 → homing one axis: 3 s → 150 ms) and connect to it.
fn start(config: SimConfig, speedup: f64) -> (Controller<mode::Parameters>, Sim) {
    start_with(config, speedup, cfg())
}

fn start_with(config: SimConfig, speedup: f64, ccfg: Config) -> (Controller<mode::Parameters>, Sim) {
    let (link, sim) = spawn(config, speedup);
    let (ctl, version) = connect(link, ccfg).expect("connect");
    assert!(version.starts_with("@V2.00/55W"));
    (ctl, sim)
}

fn homed() -> SimConfig {
    SimConfig { homed_at_start: true, ..SimConfig::default() }
}

const LONG: Duration = Duration::from_secs(10);

#[test]
fn home_then_cut_a_rectangle() {
    let (mut p, sim) = start(SimConfig::default(), 20.0);
    p.set_speed(20, 36_000).expect("#G20");
    p.set_dead_time_ms(100).expect("#C1");

    let mut jog = p.into_jog().expect("@M1");
    jog.home(&[Axis::Z]).expect("$Hz");
    jog.wait_until(LONG, |s| s.homed && !s.moving).expect("Z homed");
    jog.home(&[Axis::X, Axis::Y]).expect("$Hxy");
    let s = jog.wait_until(LONG, |s| s.homed && !s.moving).expect("XY homed");
    assert!(s.homed);
    jog.set_outputs(0x45).expect("clamp + bevel head");

    let mut cut = jog.into_cutting().expect("@M2");
    let rect = [(9695, 10775), (9695, 2615), (5551, 2615), (5551, 10775), (9695, 10775)];
    for (x, y) in rect {
        cut.vector(20, x, y, 774).expect("vector");
    }
    cut.wait_idle(LONG).expect("cut finished");

    let snap = sim.snapshot();
    assert_eq!(snap.position, [9695, 10775, 774]);
    assert_eq!((snap.outputs, snap.error, snap.moving), (0x45, 0, false));
}

#[test]
fn status_shows_the_dead_time_then_motion() {
    let (mut p, _sim) = start(homed(), 1.0);
    p.set_dead_time_ms(300).expect("#C1");
    p.set_speed(20, 4000).expect("#G20");
    let mut cut = p.into_cutting().expect("@M2");
    cut.vector(20, 400, 0, 0).expect("vector");
    let s = cut.status().expect("@X");
    assert!(s.moving && s.waiting, "@X15 while #C1 runs: {s:?}");
    thread::sleep(Duration::from_millis(350));
    let s = cut.status().expect("@X");
    assert!(s.moving && !s.waiting, "@X14 once moving: {s:?}");
    cut.wait_idle(LONG).expect("idle");
}

#[test]
fn stop_handle_brakes_a_running_jog() {
    let (p, sim) = start(homed(), 1.0);
    let mut jog = p.into_jog().expect("@M1");
    let stop = jog.stop_handle();
    jog.jog_steps(20, Axis::X, 999_999, true).expect("endless jog");
    let stopper = thread::spawn(move || {
        thread::sleep(Duration::from_millis(150));
        stop.brake().expect("@B");
    });
    assert!(matches!(jog.wait_idle(LONG), Err(ControllerError::Stopped)));
    stopper.join().expect("stopper");

    let s = jog.clear_stop().expect("clear_stop");
    assert!(!s.moving && s.homed, "braked, reference kept: {s:?}");
    let x = jog.position_steps(Axis::X).expect("@LX");
    assert!(x > 0 && x < 999_999, "stopped part-way, at {x}");
    assert_eq!(sim.snapshot().position[0], i64::from(x));
}

#[test]
fn emergency_stop_requires_rehoming() {
    let (p, _sim) = start(homed(), 20.0);
    let stop = p.stop_handle();
    stop.emergency_stop().expect("@S");
    let mut p = p;
    let s = p.clear_stop().expect("clear_stop");
    assert!(!s.homed, "@S loses the reference");
    let err = p.into_cutting().expect_err("mode 2 needs homing");
    assert!(matches!(err.error, ControllerError::Rejected { code: 41, .. }));
}

/// An oval-like curve: 400 short vectors (≈ 9 KB of text).
fn stream_curve(cut: &mut Controller<mode::Cutting>) -> controller::Result<()> {
    for i in 0..400 {
        let a = f64::from(i) * std::f64::consts::TAU / 400.0;
        let (x, y) = (7000.0 + 2000.0 * a.cos(), 6000.0 + 4000.0 * a.sin());
        cut.vector(20, x as i32, y as i32, i * 7)?;
    }
    Ok(())
}

#[test]
fn pacing_lets_a_long_curve_through_a_small_buffer() {
    // 1000-byte buffer: window 256 ≤ 1000 × (100 − 50)% keeps it from overrunning.
    let paced = Config { stream_window_bytes: Some(256), stream_high_water_pct: 50, ..cfg() };
    let (mut p, sim) = start_with(SimConfig { buffer_capacity: 1000, ..homed() }, 20.0, paced);
    p.set_flow_control(true).expect("#C7,1");
    p.set_speed(20, 40_000).expect("#G20");
    let events = p.subscribe();
    let mut cut = p.into_cutting().expect("@M2");
    stream_curve(&mut cut).expect("every vector accepted");
    cut.wait_idle(LONG).expect("curve finished");
    assert_eq!(sim.snapshot().error, 0);
    let waited = sim.received().iter().filter(|l| l.as_str() == "@H").count();
    assert!(waited > 0, "buffer level was checked");
    drop(events);
}

#[test]
fn without_pacing_the_same_curve_overruns_even_with_0x03_signals() {
    // The 200-byte early warning cannot stop a host that writes faster than it can react.
    let (mut p, _sim) = start(SimConfig { buffer_capacity: 1000, ..homed() }, 20.0);
    p.set_flow_control(true).expect("#C7,1");
    p.set_speed(20, 40_000).expect("#G20");
    let mut cut = p.into_cutting().expect("@M2");
    let outcome = stream_curve(&mut cut).and_then(|()| cut.wait_idle(LONG).map(|_| ()));
    assert!(
        matches!(outcome, Err(ControllerError::AsyncError { code: 22 })),
        "buffer overrun (*022) must surface, got {outcome:?}"
    );
}

#[test]
fn limit_switch_during_a_cut_surfaces() {
    let (p, _sim) = start(homed(), 20.0);
    let mut cut = p.into_cutting().expect("@M2");
    cut.vector(20, 70_000, 100, 0).expect("streamed"); // beyond the X envelope
    let outcome = cut.wait_idle(LONG);
    assert!(
        matches!(outcome, Err(ControllerError::AsyncError { code: 33 } | ControllerError::Fault { code: Some(33), .. })),
        "limit switch must surface, got {outcome:?}"
    );
}

#[test]
fn host_watchdog_stops_the_machine_when_the_host_goes_quiet() {
    let (mut p, sim) = start(homed(), 1.0);
    p.set_host_watchdog_ms(200).expect("#C19");
    let mut jog = p.into_jog().expect("@M1");
    jog.jog_steps(20, Axis::X, 999_999, true).expect("endless jog");
    thread::sleep(Duration::from_millis(400)); // host silent
    assert!(!sim.snapshot().moving, "controller stopped on its own");
    let outcome = jog.wait_idle(LONG);
    assert!(
        matches!(outcome, Err(ControllerError::AsyncError { code: 68 } | ControllerError::Fault { code: Some(68), .. })),
        "watchdog abort must surface, got {outcome:?}"
    );
}

#[test]
fn slow_and_lost_replies_recover() {
    let (mut p, sim) = start(homed(), 20.0);
    sim.set_reply_delay(Duration::from_millis(300)); // longer than the 200 ms deadline
    assert!(matches!(p.status(), Err(ControllerError::Timeout { .. })));
    sim.set_reply_delay(Duration::ZERO);
    assert!(p.status().expect("resynced past the late reply").homed);

    sim.drop_next_replies(1);
    assert!(matches!(p.position_steps(Axis::X), Err(ControllerError::Timeout { .. })));
    assert_eq!(p.position_steps(Axis::X).expect("recovered"), 0);
}

#[test]
fn start_button_during_a_cut_is_an_event() {
    let (p, sim) = start(homed(), 1.0);
    let events = p.subscribe();
    let mut cut = p.into_cutting().expect("@M2");
    cut.vector(20, 50_000, 0, 0).expect("long vector");
    thread::sleep(Duration::from_millis(50));
    sim.press_start_button();
    let got = events.iter().take(200).any(|e| e == Event::Signal(Signal::StartButton));
    assert!(got, "0x07 delivered as an event");
}

#[test]
fn pulled_cable_is_a_disconnect() {
    let (mut p, sim) = start(homed(), 20.0);
    sim.disconnect();
    thread::sleep(Duration::from_millis(20));
    assert!(matches!(p.status(), Err(ControllerError::Disconnected(_))));
}
