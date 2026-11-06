//! The **safe** layer: machine operations in millimetres and degrees.
//!
//! Safety depends on sequence and state, so this layer is a state machine, not a filter.
//! Invariants it owns (to implement):
//! - No motion before homing; homed state tracked from `@X` flag 4.
//! - **No travel with a head down.** A cut is a single unit: travel head-up → orient blade →
//!   head down → stroke → head up. There is no public "head down" operation.
//! - Soft limits: every target is checked against the machine envelope before sending.
//! - Units: board frame mm/deg → steps via the machine profile (scale, head offsets,
//!   board origin); blade angle unwrapping for the rotary Z axis.
//! - Ramp/speed laws: `#R = min(700, 900000 / accel)`, `#G20 = 360 × speed%`.
//! - Pause at the end of the current cut; stop-now via the controller's stop lane;
//!   defined safe end state (head up, clamps released) on every exit path.
//!
//! Depends on `controller`; knows nothing about the program file format.
//!
//! # Who does what: the host or the controller
//!
//! Give each job to whoever owns its timing or its information.
//!
//! **The controller** owns timing relative to motion, and facts only it has. Handing it these
//! jobs replaces GMC's choreography of mode switches and round trips (1,694 mode switches for
//! 1,334 vectors in its captures) with one stream per cut, timed exactly, with the look-ahead
//! intact. Each needs firmware 2.00 to honour what the v1.00 spec says; `gunnar check` stages
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
//!   reaches the host too late over the real link, which `gunnar check` stage 6 shows. Keep one
//!   of the two, not both.
//!
//! # Consequences of handing work to the controller
//!
//! - **A brake discards what is queued, safe state included.** Head-up queued as `A02` behind
//!   a cut is thrown away with the cut, so the stop path restores the safe state itself:
//!   brake, `clear_stop`, head up, clamps released. (That `@B` discards the queue is what
//!   `gunnar check` stages 4 and 6 confirm on the machine.)
//! - **Stream one cut at a time.** Whatever has been streamed runs unless braked, so the unit
//!   of streaming is the unit of safety: travel head-up, orient the blade, head down, settle,
//!   stroke, head up. The look-ahead works within a cut and the host decides between cuts,
//!   which is what makes "pause at the end of the current cut" possible.
//!
//! # Keeping up with the cut
//!
//! The serial line carries about 11,500 bytes/s (115200 baud, the controller's maximum). A path
//! that needs more starves: the controller runs its buffer dry and slows or stalls mid-contour.
//! That is GMC's pause in curves. Its oval (`g_oval_mount`) has 1,107 vectors with a median
//! segment of 0.77 mm, each a full `L20,X…,Y…,Z…`; at its feed of 855 mm/s it needs 22,300
//! bytes/s, almost twice what the line carries. Three rules:
//! - **Budget the bytes.** A path needs feed ÷ segment length × bytes per vector. Keep that
//!   under about 70% of the line, checked when the program is converted, not found mid-cut.
//! - **Break curves by chord tolerance,** not into fixed small steps: keep every point within a
//!   tolerance of the true curve, e.g. 0.02 mm, under one 0.025 mm motor step. GMC's oval drops
//!   from 1,107 to 316 vectors.
//! - **Stream paths compactly** with `Controller::path`: one absolute vector, then relative
//!   steps without the `L20,` prefix, unchanged axes left out. That halves the bytes per vector
//!   and stays exact. Together with the chord tolerance, GMC's oval needs a third of the line.
//!
//! # Feeds, ramps and dead time in one stream
//!
//! - **Two speed entries,** not one rewritten: `L21` for travel and `L20` for cutting, both set
//!   in mode 0 before the job. Each vector names its entry, so nothing changes mid-stream.
//!   In-stream `G` is ambiguous in the spec ("for all subsequent moves"); `gunnar check` stage 5
//!   measures whether it applies in sequence or on receipt, and nothing should rely on it
//!   before then.
//! - **One ramp** for the stream: `#R` can only be set in mode 0. Use the cutting ramp; GMC's
//!   shorter travel ramp (112 ms against 250 ms) only matters on long travels.
//! - **The head settles with an explicit `W`,** not with the `#C1` dead time as GMC does. The
//!   dead time only applies when a stream starts from an empty queue (its purpose is to let
//!   several vectors arrive before motion starts); with just-in-time streaming that happens
//!   once per job, so keep it short.
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
