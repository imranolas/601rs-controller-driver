//! Byte transport to the controller.
//!
//! A [`Link`] is a pair of independent halves: a [`Reader`] (owned by the controller's reader
//! thread) and a [`Writer`] (shared by the command path and the stop lane). Everything above
//! this crate cannot tell whether the link is a real serial port ([`serial`]), an in-memory
//! pair driven by a test or simulator ([`memory`]), or something else.

use std::fmt;
use std::io;

pub mod capture;
pub mod memory;
#[cfg(feature = "serial")]
pub mod serial;
pub mod tap;

/// Which way bytes travelled on a link.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Host → controller ("Written data" in a GMC capture, `>` in a transcript).
    HostToController,
    /// Controller → host ("Read data", `<`).
    ControllerToHost,
}

/// The receiving half of a link.
pub trait Reader: Send {
    /// Read available bytes into `buf`. Returns `Ok(0)` when nothing arrived within the
    /// link's read timeout. That is not an error and not end-of-stream. An `Err` means the
    /// link is gone.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
}

/// The sending half of a link.
pub trait Writer: Send {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()>;
}

/// An open, bidirectional link to the controller.
pub struct Link {
    /// Human-readable name, e.g. `/dev/cu.usbserial-1420` or `memory`.
    pub name: String,
    pub reader: Box<dyn Reader>,
    pub writer: Box<dyn Writer>,
}

impl fmt::Debug for Link {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Link")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}
