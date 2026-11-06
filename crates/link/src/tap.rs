//! A transcript of everything on a link, in both directions, with timestamps.
//!
//! [`tap`] wraps a [`Link`] so every byte written or read is also appended to a [`Log`]. The
//! log is plain text, one transfer per line, readable by people and by [`parse_line`]:
//!
//! ```text
//! # gunnar probe /dev/cu.usbserial-1420
//!    0.003 > @V⏎
//!    0.011 < @V2.00/55W 19.10.10 2807001⏎
//!    1.204 < <01>
//! # note: anything the tool wants to record
//! ```
//!
//! `>` is host → controller, `<` controller → host. CR is shown as `⏎`, other control bytes
//! as `<xx>`. Lines starting with `#` are notes.

use std::fmt;
use std::io::{self, Write};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use crate::{Direction, Link, Reader, Writer};

/// Shared, timestamped transcript. Cloning shares the same file and clock.
///
/// Writing never interrupts the link: a failed write is remembered instead, and
/// [`Log::write_error`] reports it, because a transcript with gaps must not pass for a whole one.
#[derive(Clone)]
pub struct Log {
    sink: Arc<Mutex<Sink>>,
    start: Instant,
}

struct Sink {
    out: Box<dyn Write + Send>,
    /// The first write that failed.
    error: Option<io::Error>,
}

impl Sink {
    fn line(&mut self, line: fmt::Arguments<'_>) {
        let result = writeln!(self.out, "{line}").and_then(|()| self.out.flush());
        if let Err(e) = result {
            self.error.get_or_insert(e);
        }
    }
}

impl fmt::Debug for Log {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Log").finish_non_exhaustive()
    }
}

impl Log {
    pub fn new(out: impl Write + Send + 'static) -> Self {
        Self {
            sink: Arc::new(Mutex::new(Sink {
                out: Box::new(out),
                error: None,
            })),
            start: Instant::now(),
        }
    }

    fn sink(&self) -> MutexGuard<'_, Sink> {
        self.sink.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Append a `# …` note (one line per line of `text`).
    pub fn note(&self, text: &str) {
        let mut sink = self.sink();
        for line in text.lines() {
            sink.line(format_args!("# {line}"));
        }
    }

    /// The first write to the transcript that failed, if any: then the transcript has gaps.
    pub fn write_error(&self) -> Option<String> {
        self.sink().error.as_ref().map(ToString::to_string)
    }

    fn transfer(&self, direction: Direction, bytes: &[u8]) {
        let t = self.start.elapsed().as_secs_f64();
        let arrow = match direction {
            Direction::HostToController => '>',
            Direction::ControllerToHost => '<',
        };
        self.sink()
            .line(format_args!("{t:9.3} {arrow} {}", printable(bytes)));
    }
}

/// Render bytes as one line: printable ASCII as is, CR as `⏎`, anything else as `<xx>`.
pub fn printable(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|&b| match b {
            b'\r' => "⏎".to_owned(),
            0x20..=0x7E if b != b'<' => (b as char).to_string(),
            _ => format!("<{b:02x}>"),
        })
        .collect()
}

/// One transfer read back from a transcript.
#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    /// Time since the transcript started.
    pub at: Duration,
    pub direction: Direction,
    pub bytes: Vec<u8>,
}

/// Parse a transcript line back into an [`Entry`]. Notes and blank lines give `None`.
pub fn parse_line(line: &str) -> Option<Entry> {
    let line = line.trim_start();
    if line.starts_with('#') || line.is_empty() {
        return None;
    }
    let (t, rest) = line.split_once(' ')?;
    let (arrow, text) = rest.split_once(' ').unwrap_or((rest, ""));
    let direction = match arrow {
        ">" => Direction::HostToController,
        "<" => Direction::ControllerToHost,
        _ => return None,
    };
    let mut bytes = Vec::new();
    let mut chars = text.chars();
    while let Some(c) = chars.next() {
        match c {
            '⏎' => bytes.push(b'\r'),
            '<' => {
                let hex: String = chars.by_ref().take(3).collect();
                bytes.push(u8::from_str_radix(hex.strip_suffix('>')?, 16).ok()?);
            }
            c if c.is_ascii() => bytes.push(c as u8),
            _ => return None,
        }
    }
    Some(Entry {
        at: Duration::try_from_secs_f64(t.parse().ok()?).ok()?,
        direction,
        bytes,
    })
}

/// Wrap `link` so all traffic is also written to `log`.
pub fn tap(link: Link, log: Log) -> Link {
    Link {
        name: link.name,
        reader: Box::new(TapReader {
            inner: link.reader,
            log: log.clone(),
        }),
        writer: Box::new(TapWriter {
            inner: link.writer,
            log,
        }),
    }
}

struct TapReader {
    inner: Box<dyn Reader>,
    log: Log,
}

impl Reader for TapReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = self.inner.read(buf)?;
        if n > 0 {
            self.log.transfer(Direction::ControllerToHost, &buf[..n]);
        }
        Ok(n)
    }
}

struct TapWriter {
    inner: Box<dyn Writer>,
    log: Log,
}

impl Writer for TapWriter {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        // Log first: if the write fails, the transcript still shows what was attempted.
        self.log.transfer(Direction::HostToController, bytes);
        self.inner.write_all(bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn printable_round_trips() {
        let bytes = b"@X18\r\x01\x01\x04<";
        let line = format!("   1.250 < {}", printable(bytes));
        assert_eq!(line, "   1.250 < @X18⏎<01><01><04><3c>");
        assert_eq!(
            parse_line(&line),
            Some(Entry {
                at: Duration::from_millis(1250),
                direction: Direction::ControllerToHost,
                bytes: bytes.to_vec(),
            })
        );
        assert_eq!(parse_line("# a note"), None);
    }

    /// A writer that fails after `ok` lines, like a full disk.
    struct FailingAfter {
        ok: usize,
    }

    impl Write for FailingAfter {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            if buf.ends_with(b"\n") {
                if self.ok == 0 {
                    return Err(io::Error::other("disk full"));
                }
                self.ok -= 1;
            }
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn a_failed_write_is_reported_not_lost() {
        let log = Log::new(FailingAfter { ok: 1 });
        log.note("first");
        assert_eq!(log.write_error(), None);
        log.note("second");
        log.note("third");
        assert_eq!(log.write_error().as_deref(), Some("disk full"));
    }
}
