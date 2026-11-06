//! The host's protocol core wired straight to the simulator's controller core. Both are pure,
//! so there are no threads, no link and no sleeping: time is a value both sides are given. A
//! six-second homing takes no time, and every interleaving of bytes is exactly reproducible,
//! including the awkward ones: a reply that misses its deadline, a fault that overtakes a reply,
//! a stop while a reply is still on its way.

use std::time::{Duration, Instant};

use controller::core::{Blocked, HostCore, Output, RequestId};
use controller::{ControllerError, ErrorCode, Event, Signal, StopKind};
use protocol::{Axis, Command, Mode, Reply};
use sim::{SimConfig, SimCore};

const REPLY_TIMEOUT: Duration = Duration::from_millis(200);

/// Both cores on one virtual clock. Controller → host bytes stay in flight until delivered, so
/// a test decides exactly when each reply arrives.
struct Bench {
    host: HostCore,
    sim: SimCore,
    /// The host's time base; time moves only by `advance`.
    t0: Instant,
    now: Duration,
    in_flight: Vec<u8>,
    outputs: Vec<Output>,
}

impl Bench {
    fn new(config: SimConfig) -> Self {
        Self {
            host: HostCore::new(REPLY_TIMEOUT),
            sim: SimCore::new(config),
            t0: Instant::now(),
            now: Duration::ZERO,
            in_flight: Vec::new(),
            outputs: Vec::new(),
        }
    }

    fn homed() -> Self {
        Self::new(SimConfig {
            homed_at_start: true,
            ..SimConfig::default()
        })
    }

    fn host_now(&self) -> Instant {
        self.t0 + self.now
    }

    /// Host → controller: each line reaches the simulator at once, and its answer queues up
    /// behind whatever is already in flight.
    fn transmit(&mut self, bytes: &[u8]) {
        for line in bytes.split(|&b| b == b'\r').filter(|l| !l.is_empty()) {
            let line = std::str::from_utf8(line).expect("commands are ASCII");
            let answer = self.sim.handle_line(line, self.now);
            self.in_flight.extend(answer);
        }
    }

    /// Controller → host: everything in flight arrives now.
    fn deliver(&mut self) {
        let bytes = std::mem::take(&mut self.in_flight);
        let outputs = self.host.receive(&bytes);
        self.outputs.extend(outputs);
    }

    /// Let `d` pass on both sides.
    fn advance(&mut self, d: Duration) {
        self.now += d;
        let spontaneous = self.sim.tick(self.now);
        self.in_flight.extend(spontaneous);
        let outputs = self.host.tick(self.host_now());
        self.outputs.extend(outputs);
    }

    /// Submit a request and transmit it. Its answer stays in flight until delivered.
    fn send(&mut self, command: &Command) -> Result<RequestId, Blocked> {
        let (id, bytes) = self.host.request(command, self.host_now())?;
        self.transmit(&bytes);
        Ok(id)
    }

    /// The result of request `id`: delivering what is in flight and letting time pass until
    /// the core finishes it.
    fn completion(&mut self, id: RequestId) -> Result<Reply, ControllerError> {
        loop {
            let found = self
                .outputs
                .iter()
                .position(|o| matches!(o, Output::Completed { id: done, .. } if *done == id));
            if let Some(i) = found {
                if let Output::Completed { result, .. } = self.outputs.remove(i) {
                    return result;
                }
            }
            self.deliver();
            self.advance(Duration::from_millis(10));
        }
    }

    /// A request run to completion, resyncing first whenever the core asks: what the session
    /// driver does, without the threads.
    fn request(&mut self, command: &Command) -> Result<Reply, ControllerError> {
        loop {
            match self.send(command) {
                Ok(id) => return self.completion(id),
                Err(Blocked::OutOfSync) => {
                    let (id, bytes) = self.host.resync(self.host_now())?;
                    self.transmit(&bytes);
                    self.completion(id)?;
                }
                Err(Blocked::Fails(e)) => return Err(e),
            }
        }
    }

    fn saw(&self, event: &Event) -> bool {
        self.outputs
            .iter()
            .any(|o| matches!(o, Output::Event(e) if e == event))
    }
}

#[test]
fn homing_takes_no_time_on_a_virtual_clock() {
    let mut b = Bench::new(SimConfig::default());
    assert!(matches!(
        b.request(&Command::SetMode(Mode::Jog)),
        Ok(Reply::Ack)
    ));
    assert_eq!(b.sim.received()[0], "@I100", "a new core resyncs first");
    assert_eq!(b.host.mode(), Some(Mode::Jog));

    b.request(&Command::Home(vec![Axis::X, Axis::Y]))
        .expect("$Hxy");
    b.advance(Duration::from_secs(6)); // two axes, 3 s each on the model
    b.deliver();
    assert!(b.saw(&Event::Signal(Signal::HomingDone)));
    let Ok(Reply::Status(s)) = b.request(&Command::Status) else {
        panic!("@X");
    };
    assert!(s.homed && !s.moving, "{s}");
}

/// A reply that misses its deadline is never paired with the next command, however late it
/// arrives: here it lands after the resync probe has gone out, in front of the probe's answer.
#[test]
fn a_late_reply_is_never_paired_with_the_next_command() {
    let mut b = Bench::homed();
    b.request(&Command::Status).expect("in sync");
    let id = b.send(&Command::PositionSteps(Axis::X)).expect("@LX");
    b.advance(REPLY_TIMEOUT); // the answer is still in flight
    assert!(matches!(
        b.completion(id),
        Err(ControllerError::Timeout { .. })
    ));

    let Ok(Reply::Status(s)) = b.request(&Command::Status) else {
        panic!("@X after the resync");
    };
    assert!(s.homed);
    assert!(
        b.saw(&Event::Unsolicited("@LX0".into())),
        "discarded, not paired"
    );
    assert_eq!(b.sim.received(), ["@I100", "@X", "@LX", "@I101", "@X"]);
}

/// A fault that overtakes a status reply is reported as a fault; the poll still gets its status.
#[test]
fn a_fault_during_a_status_poll_is_a_fault_not_the_reply() {
    let mut b = Bench::homed();
    b.request(&Command::Status).expect("in sync");
    let fault = b.sim.inject_error(33, b.now);
    b.in_flight.extend(fault);
    let id = b.send(&Command::Status).expect("@X");
    let Ok(Reply::Status(s)) = b.completion(id) else {
        panic!("the status is the reply");
    };
    assert!(s.error && s.homed, "{s}");
    assert_eq!(b.host.take_async_error(), Some(ErrorCode(33)));
}

/// A stop finishes the outstanding request at once and refuses everything until cleared.
/// Clearing resyncs past both the abandoned reply and the stop's own acknowledgement.
#[test]
fn a_stop_finishes_the_request_and_blocks_until_cleared() {
    let mut b = Bench::homed();
    b.request(&Command::Status).expect("in sync");
    let id = b.send(&Command::PositionSteps(Axis::X)).expect("@LX"); // answer in flight
    let (stop, outputs) = b.host.stop(StopKind::Brake);
    b.outputs.extend(outputs);
    b.transmit(&stop);
    assert!(matches!(b.completion(id), Err(ControllerError::Stopped)));
    assert!(matches!(
        b.send(&Command::Status),
        Err(Blocked::Fails(ControllerError::Stopped))
    ));

    let (probe, bytes) = b.host.resync(b.host_now()).expect("resync while stopped");
    b.transmit(&bytes);
    b.completion(probe)
        .expect("the probe is answered after the @LX reply and the @B acknowledgement");
    b.host.clear_stop();
    assert!(matches!(b.request(&Command::Status), Ok(Reply::Status(_))));
}
