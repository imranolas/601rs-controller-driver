//! Wire protocol of the CNC-Control ("CNCCON") controller in the Gunnar 601rs.
//!
//! Pure: no I/O, no clock, no threads. Everything here can be tested with byte slices.
//! Semantics and citations: `GMC_Protocol_Reference.md` at the workspace root.

pub mod command;
pub mod decode;
pub mod reply;
pub mod status;

pub use command::{Axis, Command, Mode};
pub use decode::{Decoder, Frame, Signal};
pub use reply::Reply;
pub use status::Status;
