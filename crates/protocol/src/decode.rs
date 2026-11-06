//! Streaming decoder for controller → host bytes.
//!
//! The stream mixes two things [SPEC1 p.6–7]:
//! - reply lines terminated by CR (an empty line is a bare acknowledgement), and
//! - single out-of-band signal bytes (0x01–0x09) that may arrive between or inside lines,
//!   e.g. `@X18\r\x01\x01\x04` after homing [CAP g_init].
//!
//! Signals are emitted immediately and never disturb a partially received line.

/// Out-of-band signal bytes [SPEC1 p.7]. Each is switched on by a parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Signal {
    /// 0x01: a motion finished ("positioned"); `#C5,1`.
    Positioned,
    /// 0x02: buffer has room again after `BufferAlmostFull` (like XON); `#C7,1`.
    BufferResume,
    /// 0x03: mode-2 buffer almost full, ~200 characters left (like XOFF); `#C7,1`.
    BufferAlmostFull,
    /// 0x04: homing finished; `#C6,1`.
    HomingDone,
    /// 0x05: heartbeat ("node guarding"); `#C8,<ms>`.
    KeepAlive,
    /// 0x06: signal input 1 (I221, the green start button on the 601) went HIGH; `#C35` bit 1.
    Input1High,
    /// 0x07: signal input 1 (I221) went LOW; `#C35` bit 1. GMC treats this byte as "start
    /// button" [GMC], i.e. it reacts to the release.
    Input1Low,
    /// 0x08: signal input 2 (I222) went HIGH; `#C35` bit 2.
    Input2High,
    /// 0x09: signal input 2 (I222) went LOW; `#C35` bit 2.
    Input2Low,
}

impl Signal {
    pub const fn from_byte(b: u8) -> Option<Signal> {
        match b {
            0x01 => Some(Signal::Positioned),
            0x02 => Some(Signal::BufferResume),
            0x03 => Some(Signal::BufferAlmostFull),
            0x04 => Some(Signal::HomingDone),
            0x05 => Some(Signal::KeepAlive),
            0x06 => Some(Signal::Input1High),
            0x07 => Some(Signal::Input1Low),
            0x08 => Some(Signal::Input2High),
            0x09 => Some(Signal::Input2Low),
            _ => None,
        }
    }

    pub const fn byte(self) -> u8 {
        match self {
            Signal::Positioned => 0x01,
            Signal::BufferResume => 0x02,
            Signal::BufferAlmostFull => 0x03,
            Signal::HomingDone => 0x04,
            Signal::KeepAlive => 0x05,
            Signal::Input1High => 0x06,
            Signal::Input1Low => 0x07,
            Signal::Input2High => 0x08,
            Signal::Input2Low => 0x09,
        }
    }
}

/// The longest line the decoder collects; a longer run of printable bytes without a CR is
/// noise, e.g. from a wrong baud rate. Replies are short, but the mode-9 licence exchange
/// echoes hex records of up to 2368 bytes [CAP g_init], so the bound sits well above that.
pub const MAX_LINE: usize = 4096;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    /// A CR-terminated line, without the CR. Empty = bare acknowledgement.
    Line(String),
    Signal(Signal),
    /// A control byte this decoder does not know. Surfaced, never silently dropped.
    Unknown(u8),
    /// The first [`MAX_LINE`] bytes of a printable run that never ended in a CR: noise. The
    /// rest of the run, up to and including the next CR, is dropped, so it can never be
    /// mistaken for a reply line.
    Garbled(Vec<u8>),
}

#[derive(Debug, Default)]
pub struct Decoder {
    line: Vec<u8>,
    /// Inside a garbled run: drop printable bytes until the next CR.
    garbled: bool,
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
                b'\r' if std::mem::take(&mut self.garbled) => {}
                b'\r' => {
                    let text = String::from_utf8_lossy(&self.line).into_owned();
                    self.line.clear();
                    out.push(Frame::Line(text));
                }
                0x20..=0x7E if self.garbled => {}
                0x20..=0x7E if self.line.len() == MAX_LINE => {
                    out.push(Frame::Garbled(std::mem::take(&mut self.line)));
                    self.garbled = true;
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
    fn all_nine_signal_bytes() {
        let mut d = Decoder::new();
        let frames = d.push(&[1, 2, 3, 4, 5, 6, 7, 8, 9]);
        let bytes: Vec<u8> = frames
            .iter()
            .map(|f| match f {
                Frame::Signal(s) => s.byte(),
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(bytes, [1, 2, 3, 4, 5, 6, 7, 8, 9]);
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
            d.push(b"@X\x061\r"),
            vec![Frame::Signal(Signal::Input1High), line("@X1")]
        );
    }

    #[test]
    fn an_endless_line_is_surfaced_once_then_dropped_to_the_next_cr() {
        let mut d = Decoder::new();
        let noise = vec![b'~'; MAX_LINE + 40];
        assert_eq!(d.push(&noise), vec![Frame::Garbled(vec![b'~'; MAX_LINE])]);
        assert!(d.pending().is_empty(), "the buffer stays bounded");
        // The tail of the run ends at the CR without producing a line; the next one is clean.
        assert_eq!(d.push(b"~~~\r@X04\r"), vec![line("@X04")]);
    }

    #[test]
    fn unknown_control_bytes_are_surfaced() {
        let mut d = Decoder::new();
        assert_eq!(d.push(&[0x0B]), vec![Frame::Unknown(0x0B)]);
    }
}
