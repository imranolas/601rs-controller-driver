//! Wire protocol of the CNC-Control ("CNCCON") controller in the Gunnar 601rs.
//!
//! Pure: no I/O, no clock, no threads. Everything here can be tested with byte slices.
//! Built to the protocol description v1.00/00b (B. Lewetz, 20 Jul 2000), cited as
//! [SPEC1 p.N]; behaviour proven by the GMC captures is cited as [CAP]. Semantics and
//! citations: `GMC_Protocol_Reference.md` at the workspace root.

pub mod command;
pub mod decode;
pub mod errors;
pub mod reply;
pub mod setting;
pub mod status;
pub mod switches;

pub use command::{
    Axis, Command, JogTarget, Level, Mode, OverridePct, Param, SpeedOverride, StreamFlags,
};
pub use decode::{Decoder, Frame, Signal};
pub use errors::ErrorCode;
pub use reply::{InputDef, Reply};
pub use setting::{Radius, Setting, SettingKey, SignalAcks, Units, WatchdogMs};
pub use status::Status;
pub use switches::{LimitSwitch, Side, Switches};
