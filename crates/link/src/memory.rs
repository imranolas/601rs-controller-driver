//! In-memory link: the host side is a normal [`Link`]; the other side is a [`Remote`] that a
//! test (or the simulator) uses to play the controller.

use std::collections::VecDeque;
use std::fmt;
use std::io;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::time::{Duration, Instant};

use crate::{Link, Reader, Writer};

/// Create a connected pair. `read_timeout` bounds each host-side `read`, like a serial port's.
pub fn pair(read_timeout: Duration) -> (Link, Remote) {
    let (host_tx, remote_rx) = mpsc::channel::<Vec<u8>>();
    let (remote_tx, host_rx) = mpsc::channel::<Vec<u8>>();
    let link = Link {
        name: "memory".to_owned(),
        reader: Box::new(MemReader { rx: host_rx, pending: VecDeque::new(), timeout: read_timeout }),
        writer: Box::new(MemWriter { tx: host_tx }),
    };
    (link, Remote { rx: remote_rx, tx: remote_tx, buf: Vec::new() })
}

struct MemReader {
    rx: Receiver<Vec<u8>>,
    pending: VecDeque<u8>,
    timeout: Duration,
}

impl Reader for MemReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.pending.is_empty() {
            match self.rx.recv_timeout(self.timeout) {
                Ok(bytes) => self.pending.extend(bytes),
                Err(RecvTimeoutError::Timeout) => return Ok(0),
                Err(RecvTimeoutError::Disconnected) => {
                    return Err(io::Error::new(io::ErrorKind::BrokenPipe, "remote end closed"));
                }
            }
        }
        let n = buf.len().min(self.pending.len());
        for (dst, b) in buf.iter_mut().zip(self.pending.drain(..n)) {
            *dst = b;
        }
        Ok(n)
    }
}

struct MemWriter {
    tx: Sender<Vec<u8>>,
}

impl Writer for MemWriter {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.tx
            .send(bytes.to_vec())
            .map_err(|_| io::Error::new(io::ErrorKind::BrokenPipe, "remote end closed"))
    }
}

/// The controller's side of an in-memory link. Dropping it closes the link.
pub struct Remote {
    rx: Receiver<Vec<u8>>,
    tx: Sender<Vec<u8>>,
    buf: Vec<u8>,
}

impl fmt::Debug for Remote {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Remote").field("buffered", &self.buf.len()).finish_non_exhaustive()
    }
}

impl Remote {
    /// Send bytes to the host. Returns false if the host side is gone.
    pub fn send(&self, bytes: &[u8]) -> bool {
        self.tx.send(bytes.to_vec()).is_ok()
    }

    /// Next CR-terminated line written by the host, without the CR. `None` on timeout or if
    /// the host side is gone.
    pub fn recv_line(&mut self, timeout: Duration) -> Option<String> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(pos) = self.buf.iter().position(|&b| b == b'\r') {
                let line: Vec<u8> = self.buf.drain(..=pos).collect();
                return Some(String::from_utf8_lossy(&line[..line.len() - 1]).into_owned());
            }
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            match self.rx.recv_timeout(deadline - now) {
                Ok(bytes) => self.buf.extend(bytes),
                Err(_) => return None,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let (mut link, mut remote) = pair(Duration::from_millis(5));
        link.writer.write_all(b"@V\r@X").expect("write");
        link.writer.write_all(b"\r").expect("write");
        assert_eq!(remote.recv_line(Duration::from_millis(50)).as_deref(), Some("@V"));
        assert_eq!(remote.recv_line(Duration::from_millis(50)).as_deref(), Some("@X"));

        assert!(remote.send(b"@X04\r"));
        let mut buf = [0u8; 3];
        assert_eq!(link.reader.read(&mut buf).expect("read"), 3);
        assert_eq!(&buf, b"@X0");
        assert_eq!(link.reader.read(&mut buf).expect("read"), 2);
        assert_eq!(link.reader.read(&mut buf).expect("read"), 0, "timeout is Ok(0)");

        drop(remote);
        assert!(link.reader.read(&mut buf).is_err(), "closed remote is an error");
    }
}
