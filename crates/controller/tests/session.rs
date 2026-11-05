//! Controller-layer tests against a scripted fake controller on an in-memory link.
//! Each test reproduces one situation from the protocol reference or one of the GMC
//! failure modes (timeouts, late replies, async errors, stops during waits).

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::Receiver;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use controller::{Axis, Config, Controller, ControllerError, Event, Signal, connect, mode};
use link::memory::{self, Remote};

// ----------------------------------------------------------------------------- fake controller

enum Act {
    Reply(Vec<u8>),
    Silent,
    Hangup,
}

fn reply(s: &str) -> Act {
    Act::Reply(s.as_bytes().to_vec())
}

/// Replies like the real controller for everything the tests don't script.
fn standard(line: &str) -> Act {
    match line {
        "@X" => reply("@X04\r"),
        "@V" => reply("@V2.00/55W 19.10.10 2807001\r"),
        "@B" => Act::Reply(b"\r\x01".to_vec()),
        "@E" => reply("@E0\r"),
        "@H" => reply("@H12\r"),
        "@LX" => reply("@LX44149\r"),
        l if l.starts_with("@I") => reply(&format!("{l},0\r")),
        l if l.starts_with('L') => Act::Silent, // mode 2: no reply
        _ => reply("\r"),
    }
}

/// Run a fake controller; returns the host link and a handle yielding every line the host sent.
fn fake<F>(mut respond: F) -> (link::Link, JoinHandle<Vec<String>>)
where
    F: FnMut(&str, &Remote) -> Act + Send + 'static,
{
    let (link, mut remote) = memory::pair(Duration::from_millis(2));
    let handle = thread::spawn(move || {
        let mut seen = Vec::new();
        while let Some(line) = remote.recv_line(Duration::from_secs(5)) {
            seen.push(line.clone());
            match respond(&line, &remote) {
                Act::Reply(bytes) => {
                    remote.send(&bytes);
                }
                Act::Silent => {}
                Act::Hangup => break,
            }
        }
        seen
    });
    (link, handle)
}

fn cfg() -> Config {
    Config {
        reply_timeout: Duration::from_millis(200),
        poll_interval: Duration::from_millis(5),
        flow_timeout: Duration::from_millis(200),
        ..Config::default()
    }
}

fn open(link: link::Link) -> Controller<mode::Parameters> {
    connect(link, cfg()).expect("connect").0
}

fn finish<M>(ctl: Controller<M>, fake: JoinHandle<Vec<String>>) -> Vec<String> {
    drop(ctl);
    fake.join().expect("fake controller thread")
}

fn wait_for(events: &Receiver<Event>, want: &Event) -> bool {
    let deadline = Instant::now() + Duration::from_secs(1);
    while let Ok(e) = events.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
        if &e == want {
            return true;
        }
    }
    false
}

// ----------------------------------------------------------------------------- tests

#[test]
fn connect_probes_reads_version_and_enters_mode_0() {
    let (link, fake) = fake(|l, _| standard(l));
    let (ctl, version) = connect(link, cfg()).expect("connect");
    assert_eq!(version, "@V2.00/55W 19.10.10 2807001");
    assert_eq!(finish(ctl, fake), ["@I100", "@V", "@M0"]);
}

#[test]
fn typed_queries() {
    let (link, fake) = fake(|l, _| standard(l));
    let mut ctl = open(link);
    assert_eq!(ctl.position_steps(Axis::X).expect("@LX"), 44149);
    assert!(!ctl.read_input(221).expect("@I221"));
    assert_eq!(ctl.buffer_level_pct().expect("@H"), 12);
    assert_eq!(ctl.error_code().expect("@E"), 0);
    assert!(ctl.status().expect("@X").homed);
    finish(ctl, fake);
}

#[test]
fn full_journey_through_the_modes() {
    let polls = AtomicUsize::new(0);
    let (link, fake) = fake(move |l, _| match l {
        // 2× "homing in progress", then homed and idle
        "@X" => match polls.fetch_add(1, Ordering::SeqCst) {
            0 | 1 => reply("@X18\r"),
            _ => reply("@X04\r"),
        },
        other => standard(other),
    });
    let ctl = open(link);

    let mut jog = ctl.into_jog().expect("@M1");
    jog.home(&[Axis::X, Axis::Y]).expect("$Hxy");
    let s = jog
        .wait_until(Duration::from_secs(1), |s| s.homed && !s.moving)
        .expect("homed");
    assert!(s.homed);
    jog.set_outputs(0x42).expect("$A42");

    let mut cut = jog.into_cutting().expect("@M2");
    cut.vector(20, 9695, 10775, 774).expect("vector 1");
    cut.vector(20, 9695, 2615, 774).expect("vector 2");
    cut.wait_idle(Duration::from_secs(1)).expect("idle");

    let mut params = cut.into_parameters().expect("@M0");
    params.set_speed(20, 36000).expect("#G20");

    assert_eq!(
        finish(params, fake),
        [
            "@I100",
            "@V",
            "@M0", // connect: unique resync probe, version, mode 0
            "@M1",
            "$Hxy",
            "@X",
            "@X",
            "@X",
            "$A42", // home + clamp
            "@M2",
            "L20,X9695,Y10775,Z774",
            "L20,X9695,Y2615,Z774",
            "@X", // stream, then wait
            "@M0",
            "#G20,36000",
        ]
    );
}

#[test]
fn rejected_command_keeps_pairing() {
    let (link, fake) = fake(|l, _| match l {
        "#C19,500" => reply("*021\r"),
        other => standard(other),
    });
    let mut ctl = open(link);
    match ctl.set_host_watchdog_ms(500) {
        Err(ControllerError::Rejected { command, code }) => {
            assert_eq!((command.as_str(), code), ("#C19,500", 21));
        }
        other => panic!("expected Rejected, got {other:?}"),
    }
    ctl.status().expect("still in sync");
    // No resync was needed: the *021 *was* the reply.
    assert_eq!(finish(ctl, fake), ["@I100", "@V", "@M0", "#C19,500", "@X"]);
}

#[test]
fn timeout_then_late_reply_is_resynced() {
    let late = AtomicUsize::new(0);
    let (link, fake) = fake(move |l, remote| {
        if l == "@LX" {
            late.store(1, Ordering::SeqCst);
            return Act::Silent; // answer too late
        }
        if late.swap(0, Ordering::SeqCst) == 1 {
            remote.send(b"@LX44149\r"); // the late reply lands in front of the next answer
        }
        standard(l)
    });
    let mut ctl = open(link);
    assert!(matches!(
        ctl.position_steps(Axis::X),
        Err(ControllerError::Timeout { .. })
    ));
    // GMC would now read "@LX44149" as the answer to whatever it sent next.
    let s = ctl.status().expect("status after resync");
    assert!(s.homed);
    assert_eq!(
        finish(ctl, fake),
        ["@I100", "@V", "@M0", "@LX", "@I101", "@X"],
        "unique resync probe, then the real @X"
    );
}

#[test]
fn fault_flag_reports_error_code() {
    let (link, fake) = fake(|l, _| match l {
        "@X" => reply("@X06\r"), // stopped, error, homed
        "@E" => reply("@E41\r"),
        other => standard(other),
    });
    let mut ctl = open(link);
    match ctl.wait_idle(Duration::from_secs(1)) {
        Err(ControllerError::Fault { status, code }) => {
            assert!(status.error);
            assert_eq!(code, Some(41));
        }
        other => panic!("expected Fault, got {other:?}"),
    }
    finish(ctl, fake);
}

#[test]
fn wait_times_out_explicitly() {
    let (link, fake) = fake(|l, _| match l {
        "@X" => reply("@X14\r"), // moving forever
        other => standard(other),
    });
    let mut ctl = open(link);
    assert!(matches!(
        ctl.wait_idle(Duration::from_millis(50)),
        Err(ControllerError::WaitTimeout { last, .. }) if last.moving
    ));
    finish(ctl, fake);
}

#[test]
fn stop_from_another_thread_interrupts_a_wait_and_latches() {
    let (link, fake) = fake(|l, _| match l {
        "@X" => reply("@X14\r"),
        other => standard(other),
    });
    let mut ctl = open(link);
    let events = ctl.subscribe();
    let stop = ctl.stop_handle();
    let stopper = thread::spawn(move || {
        thread::sleep(Duration::from_millis(40));
        stop.brake().expect("brake");
    });

    let started = Instant::now();
    assert!(matches!(
        ctl.wait_idle(Duration::from_secs(5)),
        Err(ControllerError::Stopped)
    ));
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "stop must interrupt promptly"
    );
    stopper.join().expect("stopper");

    assert!(wait_for(
        &events,
        &Event::StopIssued(controller::StopKind::Brake)
    ));
    assert!(
        matches!(ctl.status(), Err(ControllerError::Stopped)),
        "latched until cleared"
    );
    ctl.clear_stop().expect("clear_stop");
    ctl.status().expect("usable again");
    assert!(finish(ctl, fake).iter().any(|l| l == "@B"));
}

#[test]
fn mode_two_respects_buffer_flow_control() {
    let vectors = AtomicUsize::new(0);
    let (link, fake) = fake(move |l, remote| {
        if l.starts_with('L') && vectors.fetch_add(1, Ordering::SeqCst) == 0 {
            remote.send(&[0x03]); // almost full
            thread::sleep(Duration::from_millis(100));
            remote.send(&[0x02]); // room again
            return Act::Silent;
        }
        standard(l)
    });
    let mut cut = open(link).into_cutting().expect("@M2");
    cut.vector(20, 1, 2, 3).expect("first");
    thread::sleep(Duration::from_millis(30)); // let 0x03 arrive
    let t = Instant::now();
    cut.vector(20, 4, 5, 6).expect("second, after 0x02");
    assert!(
        t.elapsed() >= Duration::from_millis(50),
        "second vector must wait for 0x02"
    );
    finish(cut, fake);
}

#[test]
fn mode_two_flow_timeout() {
    let (link, fake) = fake(|l, remote| {
        if l.starts_with('L') {
            remote.send(&[0x03]); // full, never resumes
            return Act::Silent;
        }
        standard(l)
    });
    let mut cut = open(link).into_cutting().expect("@M2");
    cut.vector(20, 1, 2, 3).expect("first");
    thread::sleep(Duration::from_millis(30));
    assert!(matches!(
        cut.vector(20, 4, 5, 6),
        Err(ControllerError::FlowTimeout { .. })
    ));
    finish(cut, fake);
}

#[test]
fn async_error_during_a_cut_is_reported() {
    let (link, fake) = fake(|l, _| {
        if l.starts_with('L') {
            return reply("*033\r"); // limit switch while executing the vector
        }
        standard(l)
    });
    let mut cut = open(link).into_cutting().expect("@M2");
    cut.vector(20, 999_999, 0, 0).expect("streamed");
    assert!(matches!(
        cut.wait_idle(Duration::from_secs(1)),
        Err(ControllerError::AsyncError { code: 33 })
    ));
    finish(cut, fake);
}

#[test]
fn rejected_mode_switch_hands_the_controller_back() {
    let (link, fake) = fake(|l, _| match l {
        "@M2" => reply("*028\r"), // mode change only at standstill
        other => standard(other),
    });
    let ctl = open(link);
    let err = ctl.into_cutting().expect_err("must be rejected");
    assert!(matches!(
        err.error,
        ControllerError::Rejected { code: 28, .. }
    ));
    let mut ctl = *err.controller; // still Controller<Parameters>
    ctl.status().expect("still usable");
    finish(ctl, fake);
}

#[test]
fn positioned_signal_arrives_as_event() {
    let (link, fake) = fake(|l, _| standard(l));
    let mut jog = open(link).into_jog().expect("@M1");
    let events = jog.subscribe();
    jog.brake().expect("@B"); // replies CR then 0x01, as in g_motions
    assert!(wait_for(&events, &Event::Signal(Signal::Positioned)));
    finish(jog, fake);
}

#[test]
fn disconnect_is_reported_and_sticky() {
    let (link, fake) = fake(|l, _| match l {
        "@LX" => Act::Hangup,
        other => standard(other),
    });
    let mut ctl = open(link);
    assert!(matches!(
        ctl.position_steps(Axis::X),
        Err(ControllerError::Disconnected(_))
    ));
    assert!(matches!(
        ctl.status(),
        Err(ControllerError::Disconnected(_))
    ));
    finish(ctl, fake);
}

/// Found by the simulator: a limit switch fires while an `@X` poll is in flight, so `*033`
/// arrives before `@X04`. Queries are never rejected, so the `*033` is an async fault and the
/// `@X04` still pairs with the poll; nothing gets out of step.
#[test]
fn fault_racing_a_status_poll_is_async_not_a_rejection() {
    let (link, fake) = fake(|line, _| match line {
        "@X" => reply("*033\r@X04\r"),
        l => standard(l),
    });
    let mut ctl = open(link);
    let err = ctl
        .wait_idle(Duration::from_secs(1))
        .expect_err("fault reported");
    assert!(
        matches!(err, ControllerError::AsyncError { code: 33 }),
        "{err}"
    );
    let s = ctl.status().expect("still in step");
    assert!(s.is_idle());
    finish(ctl, fake);
}
