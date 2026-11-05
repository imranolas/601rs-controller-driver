//! Runs a [`SimCore`] on a thread behind an in-memory link.

use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use link::Link;
use link::memory;

use crate::core::{SimConfig, SimCore, SimTime, Snapshot};

struct State {
    core: SimCore,
    t0: Instant,
    speedup: f64,
    reply_delay: Duration,
    drop_replies: u32,
    /// Bytes waiting to be sent, with the real time they are due.
    outbox: Vec<(Instant, Vec<u8>)>,
}

impl State {
    fn now(&self) -> SimTime {
        self.t0.elapsed().mul_f64(self.speedup)
    }

    /// Queue bytes for sending. The controller answers strictly in order, so nothing may
    /// overtake bytes already queued, however their delays differ.
    fn enqueue(&mut self, bytes: Vec<u8>, delay: Duration) {
        if !bytes.is_empty() {
            let mut due = Instant::now() + delay;
            if let Some((last, _)) = self.outbox.last() {
                due = due.max(*last);
            }
            self.outbox.push((due, bytes));
        }
    }
}

struct Shared {
    state: Mutex<State>,
    shutdown: AtomicBool,
    hang_up: AtomicBool,
}

fn lock(m: &Mutex<State>) -> MutexGuard<'_, State> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Handle to a running simulator. Dropping it stops the simulator (and closes the link).
pub struct Sim {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl fmt::Debug for Sim {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Sim").field("snapshot", &self.snapshot()).finish()
    }
}

/// Start a simulator. `speedup` scales simulated time (1.0 = real time; 20.0 = homing that
/// takes 3 s simulated finishes in 150 ms). Returns the host side of the link and a handle.
pub fn spawn(config: SimConfig, speedup: f64) -> (Link, Sim) {
    let (link, mut remote) = memory::pair(Duration::from_millis(2));
    let shared = Arc::new(Shared {
        state: Mutex::new(State {
            core: SimCore::new(config),
            t0: Instant::now(),
            speedup,
            reply_delay: Duration::ZERO,
            drop_replies: 0,
            outbox: Vec::new(),
        }),
        shutdown: AtomicBool::new(false),
        hang_up: AtomicBool::new(false),
    });
    let thread_shared = Arc::clone(&shared);
    let thread = thread::Builder::new()
        .name("sim".to_owned())
        .spawn(move || {
            while !thread_shared.shutdown.load(Ordering::SeqCst) && !thread_shared.hang_up.load(Ordering::SeqCst) {
                let line = remote.recv_line(Duration::from_millis(1));
                let mut st = lock(&thread_shared.state);
                let now = st.now();
                if let Some(line) = line {
                    let reply = st.core.handle_line(&line, now);
                    if st.drop_replies > 0 && !reply.is_empty() {
                        st.drop_replies -= 1;
                    } else {
                        let delay = st.reply_delay;
                        st.enqueue(reply, delay);
                    }
                }
                let spontaneous = st.core.tick(now);
                st.enqueue(spontaneous, Duration::ZERO);

                let real_now = Instant::now();
                let ready = st.outbox.iter().take_while(|(at, _)| *at <= real_now).count();
                let due: Vec<_> = st.outbox.drain(..ready).collect();
                drop(st);
                for (_, bytes) in due {
                    if !remote.send(&bytes) {
                        return; // host gone
                    }
                }
            }
        })
        .expect("spawn simulator thread");
    (link, Sim { shared, thread: Some(thread) })
}

impl Sim {
    fn with<T>(&self, f: impl FnOnce(&mut State) -> T) -> T {
        f(&mut lock(&self.shared.state))
    }

    pub fn snapshot(&self) -> Snapshot {
        self.with(|st| {
            let now = st.now();
            st.core.snapshot(now)
        })
    }

    /// Every line the host has sent so far.
    pub fn received(&self) -> Vec<String> {
        self.with(|st| st.core.received().to_vec())
    }

    /// Raise a machine fault now (e.g. 33 = limit switch): motion halts, `*NNN` is sent.
    pub fn inject_error(&self, code: u16) {
        self.with(|st| {
            let now = st.now();
            let out = st.core.inject_error(code, now);
            st.enqueue(out, Duration::ZERO);
        });
    }

    pub fn press_start_button(&self) {
        self.with(|st| {
            let out = st.core.press_start_button();
            st.enqueue(out, Duration::ZERO);
        });
    }

    pub fn set_input(&self, input: u8, value: bool) {
        self.with(|st| st.core.set_input(input, value));
    }

    /// Delay every subsequent reply (not spontaneous signals) by `delay`.
    pub fn set_reply_delay(&self, delay: Duration) {
        self.with(|st| st.reply_delay = delay);
    }

    /// Swallow the next `n` replies entirely.
    pub fn drop_next_replies(&self, n: u32) {
        self.with(|st| st.drop_replies = n);
    }

    /// Close the link from the controller side, as if the cable were pulled.
    pub fn disconnect(&self) {
        self.shared.hang_up.store(true, Ordering::SeqCst);
    }
}

impl Drop for Sim {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
