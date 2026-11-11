//! Byte transport to the controller.
//!
//! A [`Link`] is a pair of independent halves: a [`Reader`] (owned by the controller's reader
//! thread) and a [`Writer`] (shared by the command path and the stop lane). Everything above
//! this crate cannot tell whether the link is a real serial port ([`serial`]), an in-memory
//! pair driven by a test or simulator ([`memory`]), or something else.

use std::fmt;
use std::io;
use std::thread;
use std::time::{Duration, Instant};

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
///
/// `write_all` may block, but should return while no more than a few dozen bytes still wait
/// to be sent: a stop written next goes out behind those and nothing else. A serial port's
/// own write returns long before that: at the 601 from macOS, ~11 KB (a second of the line)
/// went out ahead of a stop written after them. So the serial link is [`Paced`].
pub trait Writer: Send {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()>;
}

/// A line that sends one byte after another at a fixed rate.
pub(crate) struct Line {
    bytes_per_sec: f64,
    /// When the line will have sent everything queued so far.
    free_at: Instant,
}

impl Line {
    pub(crate) fn new(bytes_per_sec: f64) -> Self {
        Self {
            bytes_per_sec,
            free_at: Instant::now(),
        }
    }

    fn wire_time(&self, bytes: usize) -> Duration {
        Duration::from_secs_f64(bytes as f64 / self.bytes_per_sec)
    }

    /// Queue `bytes` more; returns when the line will have sent all it holds.
    pub(crate) fn queue(&mut self, bytes: usize) -> Instant {
        self.free_at = self.free_at.max(Instant::now()) + self.wire_time(bytes);
        self.free_at
    }
}

/// A [`Writer`] that keeps pace with the line under it: each write returns once all but
/// `lead` bytes of what has been written have had time to cross, so the OS and the adapter
/// never hold more than that.
pub struct Paced<W> {
    inner: W,
    line: Line,
    lead: Duration,
}

impl<W> fmt::Debug for Paced<W> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Paced")
            .field("bytes_per_sec", &self.line.bytes_per_sec)
            .field("lead", &self.lead)
            .finish_non_exhaustive()
    }
}

impl<W: Writer> Paced<W> {
    /// `bytes_per_sec` should be no faster than the line really sends: if the model runs
    /// ahead, the bytes it thinks are gone pile up unseen.
    pub fn new(inner: W, bytes_per_sec: f64, lead_bytes: usize) -> Self {
        let line = Line::new(bytes_per_sec);
        let lead = line.wire_time(lead_bytes);
        Self { inner, line, lead }
    }
}

impl<W: Writer> Writer for Paced<W> {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        let sent = self.line.queue(bytes.len());
        self.inner.write_all(bytes)?;
        if let Some(until) = sent.checked_sub(self.lead) {
            thread::sleep(until.saturating_duration_since(Instant::now()));
        }
        Ok(())
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Writes as fast as the caller does, as a serial port's OS buffer would.
    struct NoWait;

    impl Writer for NoWait {
        fn write_all(&mut self, _: &[u8]) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn a_paced_writer_keeps_at_most_its_lead_unsent() {
        // 1000 bytes/s, a 100-byte lead: ten 50-byte writes take 500 ms on the line, and the
        // last returns with 100 bytes still to go.
        let mut w = Paced::new(NoWait, 1000.0, 100);
        let start = Instant::now();
        for _ in 0..10 {
            w.write_all(&[0; 50]).expect("written");
        }
        let took = start.elapsed();
        assert!(
            took >= Duration::from_millis(390) && took < Duration::from_millis(480),
            "{took:?}"
        );
        // Once the line has gone quiet, a write within the lead returns at once.
        thread::sleep(Duration::from_millis(150));
        let start = Instant::now();
        w.write_all(&[0; 50]).expect("written");
        assert!(start.elapsed() < Duration::from_millis(20));
    }
}
