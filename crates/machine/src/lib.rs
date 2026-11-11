//! The **machine model**: how a program's cuts become controller commands on this machine, and
//! whether they can. Pure: no session, no clock, no I/O, so the compiler, the browser's cut
//! preview and the golden tests use it just as the runner does.
//!
//! - [`Profile`] ([`Profile::gunnar_601`]): the machine's facts, readable as JSON.
//! - [`frame`]: board millimetres to steps for each head, and the blade's direction.
//! - [`lower`]: a program as the steps that run it, in GMC's form or one stream per cut.
//! - [`estimate`]: its run time, and how much of the serial line each path needs.
//!
//! The tests hold it to GMC's own jobs (`tests/golden.rs`: every motion of ten captured jobs,
//! straight edges and the oval), run its jobs on the simulator in both forms
//! (`tests/simulated.rs`), and check the estimate against GMC's run times (`tests/estimate.rs`).
//!
//! What it owns:
//! - **The machine profile** (PROGRAM_FORMAT §8): scales (40 steps/mm, 8.333 steps/°), the board
//!   origin, each head's offset (bevel −71.5, straight +6.2 mm in X), the soft-limit envelope,
//!   the park position, speeds and accelerations, and the head and clamp output bits. The one
//!   source of machine facts for every crate: `gunnar`'s checks, the simulator's settings, the
//!   compiler's travel model and park.
//! - **Units:** board frame mm/deg → steps for the selected head; blade angles by the rules in
//!   PROGRAM_FORMAT §4.2 and protocol reference §3.2, unwrapped so the rotary axis keeps counting
//!   past 360° along a cut.
//! - **A cut as a unit.** A program cut lowers to one sequence: travel head-up, orient the blade,
//!   head down, settle, stroke, head up. Nothing else lowers to head down, so nothing this crate
//!   produces can travel with a blade in the board. That is safety by construction; the runner
//!   adds safety by sequence.
//! - **The job's settings:** the ramp and speed laws (`#R = min(700, 900000 / accel)`,
//!   `#G20 = 360 × speed%`), the speed entries, the start speed by thickness (below).
//! - **Checks before a run**, none of which needs the machine: every coordinate inside the
//!   envelope after the head offsets (PROGRAM_FORMAT V10), every cut within the serial line's
//!   byte budget, and the estimated run time. The cut preview shows the same.
//!
//! A cut lowers to one of two forms:
//! - **GMC's,** proven by the captures on this firmware: settings in mode 0, head down and up
//!   with `$A` in mode 1, the path in mode 2, `@X` polled until idle ([`lower`]). The golden
//!   tests (`tests/golden.rs`) hold it to GMC's own jobs: every motion of the `g_multi`
//!   captures (three boards, five speeds) and `g_regular_mount` runs with the same vectors and
//!   the same controller settings and outputs, with two deliberate differences, both described
//!   there. That also fixes where GMC lays its layouts, 21 mm in from the corner where boards
//!   register ([`Profile::board_origin_mm`]). The oval comes with curves.
//! - **One stream per cut,** the controller timing the head, the settle and the start (the
//!   runner's "Who does what"). It needs firmware 2.00 to honour the v1.00 spec, so each part
//!   is a capability in the profile, switched on once `gunnar check` has shown it on the box.
//!
//! Depends on `protocol` (the commands) and `program` (the cuts), not on `controller`.
//!
//! # Keeping up with the cut
//!
//! The serial line carries about 11,500 bytes/s (115200 baud, the controller's maximum). A path
//! that needs more starves: the controller runs its buffer dry and slows or stalls mid-contour.
//! That is GMC's pause in curves. Its oval (`g_oval_mount`, 1.9 laps with GMC's overlap) has
//! 1,107 vectors with a median segment of 0.77 mm, each a full `L20,X…,Y…,Z…`; at its feed of
//! 855 mm/s it needs 22,300 bytes/s, almost twice what the line carries. Three rules:
//! - **Budget the bytes.** A path needs feed ÷ segment length × bytes per vector. Keep that
//!   under about 70% of the line, checked when the program is converted, not found mid-cut.
//! - **Break curves by chord tolerance and turn,** not into fixed small steps: keep every point
//!   within 0.02 mm of the true curve (under one 0.025 mm motor step), and turn at most 2° per
//!   point, because the blade turns during each vector (PROGRAM_FORMAT §4.2). GMC's oval drops
//!   from 1,107 to 425 vectors.
//! - **Stream paths compactly** with `protocol::Command::path`: one absolute vector, then relative
//!   steps without the `L20,` prefix, unchanged axes left out. That halves the bytes per vector
//!   and stays exact. With the flattening above, GMC's oval needs 43% of the line.
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
//! # Starting a cut
//!
//! `#S` is the speed the motors start at, with no ramp. GMC leaves it at 100 mm/s for the bevel
//! head at every thickness, so each bevel edge jumps from rest to 100 mm/s (or straight to the
//! cutting speed, if lower) with the blade in the board. Operators see a wobble at the start on
//! thick board. GMC softens the start only for the straight head (protocol reference §3.3).
//! The machine layer softens it for both:
//! - **A start speed per job, by thickness:** `#S` from the head's 4000 steps/s down to 400
//!   (10 mm/s), linearly from 2 mm to 3.5 mm of board (GMC's rule, without its jump at 3.5 mm).
//!   A job is one board, so it is set once with the other parameters. Travel loses a few
//!   milliseconds; the tightest curves, whose speed is `#S`, slow down too, which suits thick
//!   board.
//! - **The ramp stays one per job** (above), not GMC's 700 ms on thick board, which would slow
//!   every travel move. If the start still wobbles, begin the cut with a few millimetres at a
//!   third, slow speed entry: the controller starts at that speed and should accelerate into the
//!   cut without a mode switch (to confirm with the test cuts).
//! - Which of these the wobble needs, and whether the blade trim plays a part, is for test cuts
//!   on thick board (protocol reference §10).

mod estimate;
pub mod frame;
mod lower;
mod profile;

pub use estimate::{Estimate, Host, OpEstimate, estimate};
pub use frame::{CarriageMm, MachineMm};
pub use lower::{
    At, ENTRY, Error, Job, MIN_SEGMENT_MM, SMOOTH_TURN_DEG, Step, TRAVEL_ENTRY, lower, park,
    vgroove_reset,
};
pub use profile::{
    Accel, Capabilities, Clamps, ControllerConfig, DeadTime, Envelope, Head, Homing, Line, Profile,
    RadiusSlowdown, Scale, SoftStart, VGrooveSwitch,
};

#[cfg(test)]
mod tests;
