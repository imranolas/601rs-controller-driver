//! Simulator of the Gunnar 601rs "CNC-Control" controller.
//!
//! - [`SimCore`]: the controller model. Pure: text lines in, bytes out, explicit clock. It
//!   speaks the **whole** text protocol GMC uses (not only the subset this project sends), so
//!   recorded GMC sessions can be replayed against it.
//! - [`spawn`]: runs a core on a thread behind an in-memory [`link::Link`], with a speed-up
//!   factor and fault injection through the returned [`Sim`] handle.
//!
//! Behaviour follows the controller's manual (cited as [SPEC p.N]) and the captures
//! [CAP]. Where the 601's firmware is known to differ from the 2000 spec, the captures win.
//! Physical details not yet measured (travel envelope, homing time, real speeds, buffer size)
//! are configurable placeholders in [`SimConfig`].

mod core;
mod runner;

pub use crate::core::{SimConfig, SimCore, SimTime, Snapshot};
pub use crate::runner::{LINE_BYTES_PER_SEC, Sim, spawn};
