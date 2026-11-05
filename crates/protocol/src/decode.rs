//! Streaming decoder for controller → host bytes.
//!
//! The stream mixes two things [SPEC p.6–7]:
//! - reply lines terminated by CR (an empty line is a bare acknowledgement), and
//! - single out-of-band signal bytes (0x01–0x07) that may arrive between or inside lines,
//!   e.g. `@X18\r\x01\x01\x04` after homing [CAP g_init].
//!
//! Signals are emitted immediately and never disturb a partially received line.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Signal {
    /// 0x01: a motion finished ("positioned"); enabled by `#C5,1`.
    Positioned,
    /// 0x02: buffer has room again after `BufferAlmostFull` (resume, like XON); `#C7,1`.
    BufferResume,
    /// 0x03: mode-2 buffer almost full (pause, like XOFF); `#C7,1`.
    BufferAlmostFull,
    /// 0x04: homing finished; `#C6,1`.
    HomingDone,
    /// 0x05: keep-alive ("node guarding"); `#C8,<ms>`.
    KeepAlive,
    /// 0x07: start button pressed during a cut (host code, not in the spec).
    StartButton,
}

impl Signal {
    pub const fn from_byte(b: u8) -> Option<Signal> {
        match b {
            0x01 => Some(Signal::Positioned),
            0x02 => Some(Signal::BufferResume),
            0x03 => Some(Signal::BufferAlmostFull),
            0x04 => Some(Signal::HomingDone),
            0x05 => Some(Signal::KeepAlive),
            0x07 => Some(Signal::StartButton),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    /// A CR-terminated line, without the CR. Empty = bare acknowledgement.
    Line(String),
    Signal(Signal),
    /// A control byte this decoder does not know. Surfaced, never silently dropped.
    Unknown(u8),
}

#[derive(Debug, Default)]
pub struct Decoder {
    line: Vec<u8>,
}

impl Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed received bytes; returns every complete frame, in arrival order.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut out = Vec::new();
        for &b in bytes {
            match b {
                b'\r' => {
                    let text = String::from_utf8_lossy(&self.line).into_owned();
                    self.line.clear();
                    out.push(Frame::Line(text));
                }
                0x20..=0x7E => self.line.push(b),
                _ => out.push(match Signal::from_byte(b) {
                    Some(s) => Frame::Signal(s),
                    None => Frame::Unknown(b),
                }),
            }
        }
        out
    }

    /// Bytes of an incomplete line still waiting for its CR.
    pub fn pending(&self) -> &[u8] {
        &self.line
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(s: &str) -> Frame {
        Frame::Line(s.to_owned())
    }

    #[test]
    fn homing_reply_with_trailing_signals() {
        // Exact bytes from g_init after `$Hz`.
        let mut d = Decoder::new();
        let frames = d.push(b"\r@X18\r\x01\x01\x04");
        assert_eq!(
            frames,
            vec![
                line(""),
                line("@X18"),
                Frame::Signal(Signal::Positioned),
                Frame::Signal(Signal::Positioned),
                Frame::Signal(Signal::HomingDone),
            ]
        );
    }

    #[test]
    fn lines_split_across_reads() {
        let mut d = Decoder::new();
        assert!(d.push(b"@X1").is_empty());
        assert_eq!(d.pending(), b"@X1");
        assert_eq!(d.push(b"4\r@X0"), vec![line("@X14")]);
        assert_eq!(d.push(b"4\r"), vec![line("@X04")]);
    }

    #[test]
    fn signal_inside_a_line_does_not_corrupt_it() {
        let mut d = Decoder::new();
        assert_eq!(
            d.push(b"@X\x011\r"),
            vec![Frame::Signal(Signal::Positioned), line("@X1")]
        );
    }

    #[test]
    fn unknown_control_bytes_are_surfaced() {
        let mut d = Decoder::new();
        assert_eq!(d.push(&[0x06]), vec![Frame::Unknown(0x06)]);
    }
}
