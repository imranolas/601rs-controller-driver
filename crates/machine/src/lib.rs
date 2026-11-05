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
