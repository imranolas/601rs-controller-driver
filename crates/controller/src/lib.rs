//! The **valid** layer: a session with the CNC-Control controller that sends only
//! well-formed commands, only in the mode that accepts them.
//!
//! What this layer guarantees:
//! - **One outstanding command.** A command expecting a reply is matched with *its* reply
//!   before anything else is sent. Every wait has a deadline that ends in an explicit
//!   [`ControllerError`], never in "carry on".
//! - **Rejection or fault, settled.** A `*NNN` while a command waits may be its rejection or a
//!   fault that crossed it; a probe written right behind it tells which (see [`core`]).
//! - **Resync.** After a timeout or an unexpected reply, the next operation first
//!   re-establishes reply pairing: it reads an input nothing else uses ([`RESYNC_PROBE_INPUTS`])
//!   and discards everything that arrives before that reply. GMC never did this.
//! - **Mode as a type.** [`Controller<M>`] only has the methods its mode accepts; switching
//!   mode consumes the controller and returns one in the new mode.
//! - **Stop lane.** A [`StopHandle`] brakes (`@B`) or emergency-stops (`@S`) from any thread,
//!   bypassing anything in progress. A stop **latches**: every operation fails with
//!   [`ControllerError::Stopped`] until [`Controller::clear_stop`] is called.
//! - **Mode-2 streaming** without per-command replies, honouring the controller's
//!   0x03/0x02 buffer signals when enabled (`#C7,1`).
//! - **Errors are never swallowed.** `*NNN` arriving while nobody asked (e.g. a limit switch
//!   during a cut) is reported as [`ControllerError::AsyncError`] by the next wait, stream, or
//!   command that would move the machine or change its outputs, settings or mode. Until then
//!   none of those is sent; reading state, braking and `@@` still are.
//!
//! How it is built: [`core::HostCore`] holds every one of those rules and is pure (bytes and
//! time in, bytes and outcomes out), like the simulator's `SimCore`. The session runs it on a
//! real link with a reader thread and the clock, and [`Controller<M>`] is the typed API on top.
//!
//! Nothing here knows about millimetres, heads or programs; that is the `machine` crate.
//!
//! ```no_run
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! use std::time::Duration;
//! use controller::{Axis, Config, connect};
//!
//! let link = link::serial::open(&link::serial::SerialConfig::new("/dev/cu.usbserial-1"))?;
//! let (ctl, version) = connect(link, Config::default())?;     // Controller<Parameters>
//! println!("controller {version}");
//! let mut jog = ctl.into_jog().map_err(|e| e.error)?;        // Controller<Jog>
//! jog.home(&[Axis::Z])?;
//! jog.wait_idle(Duration::from_secs(60))?;
//! # Ok(()) }
//! ```
//!
//! Commands are only available in the right mode. This does not compile:
//!
//! ```compile_fail
//! fn cut_in_parameter_mode(c: &mut controller::Controller<controller::mode::Parameters>) {
//!     let _ = c.vector(20, 0, 0, 0); // vector() exists only on Controller<mode::Cutting>
//! }
//! ```

pub mod core;
mod error;
mod event;
mod session;
mod stop;
mod typed;

use std::time::Duration;

pub use crate::core::RESYNC_PROBE_INPUTS;
pub use error::{ControllerError, Result};
pub use event::{Event, StopKind};
pub use protocol::{
    Axis, ErrorCode, InputDef, JogTarget, Level, LimitSwitch, Mode, OverridePct, Param, Setting,
    SettingKey, Side, Signal, SignalAcks, SpeedOverride, Status, StreamFlags, Switches, Units,
    WatchdogMs,
};
pub use stop::StopHandle;
pub use typed::{Controller, ModeMarker, ModeSwitchError, connect, mode};

/// Timing parameters of a session.
#[derive(Debug, Clone)]
pub struct Config {
    /// Deadline for a reply to a single command.
    pub reply_timeout: Duration,
    /// Interval between `@X` polls while waiting for a condition.
    pub poll_interval: Duration,
    /// How long a mode-2 send may wait for the controller to signal buffer room (0x02).
    pub flow_timeout: Duration,
    /// Mode-2 pacing: after this many bytes streamed, check the controller's buffer level
    /// (`@H`) and wait while it is at or above `stream_high_water_pct`. `None` streams freely
    /// and relies on the 0x03/0x02 signals (`#C7,1`) and on the buffer being large, as GMC does
    /// (it sends a 24 KB oval in one write [CAP g_oval_mount]). Enable once `@H` is verified on
    /// the real firmware; choose `window ≤ capacity × (100 − high_water)%`.
    pub stream_window_bytes: Option<usize>,
    pub stream_high_water_pct: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            reply_timeout: Duration::from_millis(1000),
            poll_interval: Duration::from_millis(50),
            flow_timeout: Duration::from_secs(10),
            stream_window_bytes: None,
            stream_high_water_pct: 50,
        }
    }
}
