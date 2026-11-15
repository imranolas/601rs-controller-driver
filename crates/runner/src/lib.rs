//! The **runner** drives the cutter: it holds the controller session and every safety rule that
//! depends on sequence and state, and runs programs and the station's manual operations with
//! it. The machine model (`machine`) decides what each cut sends; the runner decides when, and
//! stops.
//!
//! [`Runner`] holds a controller session ([`Runner::new`]) and does one thing at a time on it:
//! [`configure`](Runner::configure) the controller as the machine profile needs,
//! [`home`](Runner::home), [`park`](Runner::park), [`clamp`](Runner::clamp), or
//! [`run`](Runner::run) a program, reporting [`Event`]s and taking [`Request`]s between cuts.
//! Its [`stop_handle`](Runner::stop_handle) stops the machine at once from any thread. The
//! station runs it on a thread of its own. `tests/runner.rs` drives it against the simulator.
//!
//! **Live state.** While it works, the runner reads the machine's state as it waits and pushes
//! [`Event::State`] ([`MachineState`]: heads, clamps, where the blade is over the board, moving,
//! homed, the op, and its [steps](OpSteps): how many done, and which is running) when it
//! changes, the position and the count no more often than [`Config::state_every`]. [`subscribe`](Runner::subscribe) hears every event, runs' and
//! manual operations' alike, without a callback; between runs, the station calls
//! [`refresh`](Runner::refresh) now and then and hears only what changed. The state is read
//! from the controller, never predicted, and only once a stream is written, so path data keeps
//! the line (the `live` module says how).
//!
//! What it owns:
//! - **State:** homed (from `@X` flag 4), head, clamps, the run. No motion before homing.
//! - **Only whole cuts.** It sends what the machine model lowered, never a bare head down, and
//!   nothing outside the envelope but GMC's V-groove switch reset after homing (protocol
//!   reference §4.1).
//! - **Running a program:** validate (V1–V9 with `program`, V10 with the machine model) before
//!   opening the port; require homing (offer to home) and the operator's word that the board is
//!   loaded; push the controller configuration; run the ops in order; honour `pause` ops (the
//!   green button or the operator); report progress per op `id`/`label`; after the last op, the
//!   safe end state and park.
//! - **Pause** at the end of the current cut (the UI, Space, the green button): finish the cut,
//!   head up, hold.
//! - **Stop now:** brake through the controller's stop lane, restore the safe state itself
//!   (below), and report which op failed and why. A run is never resumed silently mid-op.
//! - **The safe end state** on every exit path, errors and drops included: head up, clamps
//!   released.
//! - **Manual operations** for the station, outside any program: home, park, clamp and release
//!   a board, move head-up. The same state and rules, without a program.
//!
//! The UI only renders runner events and sends requests; it never touches the wire.
//!
//! **Until `mc601 check` has run on the box,** the runner uses GMC's form throughout: a cut at
//! a time, `@X` polled until idle, `$A` in mode 1, progress by finished cuts. Everything below
//! that hands work to the controller waits for its check (protocol reference §10 lists what
//! each assumes until then). After any stop, until stages 4 and 6 confirm that `@B` discards
//! the queue, the runner asks for homing before moving again.
//!
//! Depends on `machine`, `controller` and `program`.
//!
//! # Who does what: the host or the controller
//!
//! Give each job to whoever owns its timing or its information.
//!
//! **The controller** owns timing relative to motion, and facts only it has. Handing it these
//! jobs replaces GMC's choreography of mode switches and round trips (1,694 mode switches for
//! 1,334 vectors in its captures) with one stream per cut, timed exactly, with the look-ahead
//! intact. Each needs firmware 2.00 to honour what the v1.00 spec says; `mc601 check` stages
//! 2, 5 and 6 test them, except `G` in the stream, which no stage covers yet.
//!
//! | Job | On the controller | What GMC does instead |
//! |---|---|---|
//! | Let the head settle before cutting | `W<ms>` in the stream (`head_settle_ms`) | the `#C1` dead time (100/200 ms), set in mode 0 before each cut |
//! | Head down and up, clamps | `A<hh>` in the stream, in order with the vectors | `$A` in mode 1 between moves |
//! | Wait for the green button (a `Pause` op) | `I221` in the stream | polls `@I221` |
//! | Start a positioning move at once | `C1` before the vector | `@M0`, `#C1,0`, `@M2` |
//! | Feed per segment | `G<entry>,<v>` in the stream | `@M0`, `#G20,…`, `@M2` per feature |
//! | Progress through a cut | the `@N` command counter | nothing |
//! | Not overrunning its buffer | `#C7,1`: it sends 0x03 when almost full, 0x02 when there is room | sends whole contours and relies on them fitting |
//! | Stopping when the host dies | `#C19` host watchdog | nothing |
//! | Reporting the green button | `#C35,1`: 0x06 pressed, 0x07 released | enables it, but also polls `@I221` |
//!
//! **The host** owns the decisions: what to cut and in what order, soft limits, units and
//! board geometry, pairing replies with commands and recovering from doubt, and stopping.
//!
//! **Not earned yet:**
//! - Switching the speed override on and off in the stream (`C8`/`C16`): only if the UI gets
//!   a speed knob.
//! - The `#C8` heartbeat: idle `@X` polling already shows the controller is alive.
//! - Pacing the stream by asking `@H` (`controller::Config::stream_window_bytes`): the host
//!   doing what `#C7` lets the controller do. Worth it only if the controller's 0x03 warning
//!   reaches the host too late over the real link, which `mc601 check` stage 6 shows. Keep one
//!   of the two, not both.
//!
//! # Consequences of handing work to the controller
//!
//! - **A brake discards what is queued, safe state included.** Head-up queued as `A02` behind
//!   a cut is thrown away with the cut, so the stop path restores the safe state itself:
//!   brake, `clear_stop`, head up, clamps released. (That `@B` discards the queue is what
//!   `mc601 check` stages 4 and 6 confirm on the machine.)
//! - **Stream one cut at a time.** Whatever has been streamed runs unless braked, so the unit
//!   of streaming is the unit of safety: travel head-up, orient the blade, head down, settle,
//!   stroke, head up. The look-ahead works within a cut and the host decides between cuts,
//!   which is what makes "pause at the end of the current cut" possible.
//!
//! # Streaming just in time
//!
//! Send cut N+1 when `@N` reaches cut N's last command. The queue never empties, so cuts follow
//! each other without a gap, and a pause requested before that point still stops after cut N.
//! Keep a running count instead of resetting `@N` per cut, since the next cut is queued while
//! the previous one runs. Stage 5 settles what the counter means on firmware 2.00: which
//! commands it counts, whether it numbers the command being executed or those finished, and
//! whether it wraps after 9999.
//!
//! # Faults during a cut
//!
//! An unprompted fault during a cut (`*022` buffer overrun, a limit switch, `*068` from the host
//! watchdog) stops the cut at once: brake, `clear_stop`, head up, clamps released, and report
//! which cut failed and why. Never retry. Mode 2 has no per-command replies, so the host cannot
//! tell which streamed command was refused or lost, and the controller may already have cut
//! past the gap. With the byte budget and flow control, `*022` cannot happen; if it does, the
//! pacing has a bug.

mod error;
mod live;
mod run;
mod session;

pub use error::{Cause, Error, Mismatch};
pub use live::{MachineState, OpSteps, StepKind};
pub use machine::Heads;
pub use run::{Config, Event, Outcome, Request, Runner};
