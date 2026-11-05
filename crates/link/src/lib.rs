//! Byte transport to the controller.
//!
//! Everything above this crate talks to a [`Port`] and cannot tell whether it is a real serial
//! port, the simulator (todo) or a replayed capture.

use std::io;

pub mod capture;
#[cfg(feature = "serial")]
pub mod serial;

/// A bidirectional byte link.
pub trait Port: Send {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()>;

    /// Read available bytes into `buf`. Returns `Ok(0)` when nothing arrived within the
    /// port's read timeout. That is not an error and not end-of-stream.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
}
