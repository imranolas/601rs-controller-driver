//! Reader for GMC serial captures (the `g_*.txt` software port-monitor exports).
//!
//! Format: UTF-16LE text. Each transfer starts with a header line
//! `[dd/mm/yyyy hh:mm:ss] Written data (COM4)` or `... Read data (COM4)`, followed by
//! hex-dump rows of up to 16 bytes (`40 56 0d   ...   @V.`).
//! The files record no line settings (baud etc.).

use std::path::Path;

use crate::Direction;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Transfer {
    /// Seconds since midnight (captures have 1 s resolution).
    pub at_secs: u32,
    pub direction: Direction,
    pub bytes: Vec<u8>,
}

#[derive(Debug, thiserror::Error)]
pub enum CaptureError {
    #[error("reading capture: {0}")]
    Io(#[from] std::io::Error),
    #[error("capture is not UTF-16LE text")]
    NotUtf16,
    #[error("hex row before any transfer header (line {0})")]
    OrphanRow(usize),
}

pub fn read_file(path: impl AsRef<Path>) -> Result<Vec<Transfer>, CaptureError> {
    let raw = std::fs::read(path)?;
    parse_utf16le(&raw)
}

pub fn parse_utf16le(raw: &[u8]) -> Result<Vec<Transfer>, CaptureError> {
    if raw.len() % 2 != 0 {
        return Err(CaptureError::NotUtf16);
    }
    let units: Vec<u16> = raw
        .chunks_exact(2)
        .map(|c| u16::from_le_bytes([c[0], c[1]]))
        .collect();
    let text = String::from_utf16_lossy(&units);
    parse_text(text.trim_start_matches('\u{feff}'))
}

pub fn parse_text(text: &str) -> Result<Vec<Transfer>, CaptureError> {
    let mut out: Vec<Transfer> = Vec::new();
    for (n, line) in text.lines().enumerate() {
        if let Some((at_secs, direction)) = parse_header(line) {
            out.push(Transfer {
                at_secs,
                direction,
                bytes: Vec::new(),
            });
        } else if let Some(row) = parse_hex_row(line) {
            out.last_mut()
                .ok_or(CaptureError::OrphanRow(n + 1))?
                .bytes
                .extend(row);
        }
    }
    Ok(out)
}

fn parse_header(line: &str) -> Option<(u32, Direction)> {
    let line = line.trim_start_matches('\u{feff}');
    let rest = line.strip_prefix('[')?;
    let (stamp, tail) = rest.split_once(']')?;
    let direction = if tail.trim_start().starts_with("Written data") {
        Direction::HostToController
    } else if tail.trim_start().starts_with("Read data") {
        Direction::ControllerToHost
    } else {
        return None;
    };
    let time = stamp.split_whitespace().nth(1)?;
    let mut hms = time.split(':').map(|p| p.parse::<u32>().ok());
    let (h, m, s) = (hms.next()??, hms.next()??, hms.next()??);
    Some((h * 3600 + m * 60 + s, direction))
}

/// A hex row is leading whitespace then 1–16 groups of "hh " (two lowercase hex digits and
/// one space); the ASCII column follows after extra padding.
fn parse_hex_row(line: &str) -> Option<Vec<u8>> {
    let body = line.trim_start();
    if body.len() == line.len() {
        return None; // rows are always indented; headers are not
    }
    let b = body.as_bytes();
    let mut bytes = Vec::new();
    let mut i = 0;
    while bytes.len() < 16 && i + 3 <= b.len() && b[i + 2] == b' ' {
        let hex = std::str::from_utf8(&b[i..i + 2]).ok()?;
        if !hex.bytes().all(|c| c.is_ascii_hexdigit()) {
            break;
        }
        bytes.push(u8::from_str_radix(hex, 16).ok()?);
        i += 3;
    }
    (!bytes.is_empty()).then_some(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_header_and_rows() {
        let text = "[28/07/2026 15:48:22] Written data (COM4)\t\r\n    40 56 0d                                          @V.\r\n\
                    [28/07/2026 15:48:23] Read data (COM4)\t\r\n    0d                                                .\r\n";
        let t = parse_text(text).expect("parse");
        assert_eq!(t.len(), 2);
        assert_eq!(t[0].direction, Direction::HostToController);
        assert_eq!(t[0].bytes, b"@V\r");
        assert_eq!(t[0].at_secs, 15 * 3600 + 48 * 60 + 22);
        assert_eq!(t[1].direction, Direction::ControllerToHost);
        assert_eq!(t[1].bytes, b"\r");
    }

    #[test]
    fn full_row_of_sixteen() {
        let row = "    30 31 32 33 34 35 36 37 38 39 61 62 63 64 65 66  0123456789abcdef";
        assert_eq!(parse_hex_row(row).expect("row").len(), 16);
    }
}
