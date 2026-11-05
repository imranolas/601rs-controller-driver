//! Commands the host may send.
//!
//! Only commands this software is allowed to send are representable. Deliberately absent:
//! mode 3/4/9 (`@M9` is file transfer and firmware update), `#W` (persist parameters on the
//! controller), `@R` (controller restart), raw text. A diagnostic raw console, if needed,
//! lives outside this type.

use std::fmt;

/// Controller operating mode [SPEC p.6, p.12].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Mode 0: parameters (`#` set, `&` query).
    Parameters,
    /// Mode 1: jog, homing, outputs (`$` commands).
    Jog,
    /// Mode 2: cutting; buffered vectors, no per-command reply.
    Cutting,
}

impl Mode {
    pub const fn code(self) -> u8 {
        match self {
            Mode::Parameters => 0,
            Mode::Jog => 1,
            Mode::Cutting => 2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Axis {
    X,
    Y,
    /// Tangential blade rotation (rotary axis).
    Z,
}

impl Axis {
    const fn upper(self) -> char {
        match self {
            Axis::X => 'X',
            Axis::Y => 'Y',
            Axis::Z => 'Z',
        }
    }
    const fn lower(self) -> char {
        match self {
            Axis::X => 'x',
            Axis::Y => 'y',
            Axis::Z => 'z',
        }
    }
}

/// A single controller command. `encode` produces the exact bytes, including the trailing CR.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    // ---- master commands: accepted in any mode [SPEC p.11] ----
    /// `@V`: version string.
    Version,
    /// `@X`: status (see [`crate::Status`]).
    Status,
    /// `@M<n>`: switch mode (only while stopped).
    SetMode(Mode),
    /// `@B`: brake all axes with ramp; positions kept.
    Brake,
    /// `@S`: stop immediately; positions lost (re-home required).
    EmergencyStop,
    /// `@L<a>`: axis position in steps.
    PositionSteps(Axis),
    /// `@I<n>`: read logical input (e.g. 221 = start button).
    ReadInput(u8),

    // ---- mode 1 (`$`) [SPEC p.29–31] ----
    /// `$H<axes>`: home the axes in the given order.
    Home(Vec<Axis>),
    /// `$A<hh>`: set auxiliary outputs (head/clamp bitmask).
    SetOutputs(u8),

    // ---- mode 0 (`#`) [SPEC p.17–27] ----
    /// `#G<entry>,<v>`: speed of speed-table entry.
    SetSpeed { entry: u8, value: u32 },
    /// `#R<ms>`: maximum ramp time.
    SetRampMs(u16),
    /// `#C1,<ms>`: mode-2 dead time before a path starts.
    SetDeadTimeMs(u16),

    // ---- mode 2 [SPEC p.31–34] ----
    /// `L<entry>,X<x>,Y<y>,Z<z>`: absolute vector in steps at speed-table entry.
    Vector { speed_entry: u8, x: i32, y: i32, z: i32 },
}

impl Command {
    /// The mode this command requires, or `None` for master commands (any mode).
    pub fn required_mode(&self) -> Option<Mode> {
        use Command::*;
        match self {
            Version | Status | SetMode(_) | Brake | EmergencyStop | PositionSteps(_)
            | ReadInput(_) => None,
            Home(_) | SetOutputs(_) => Some(Mode::Jog),
            SetSpeed { .. } | SetRampMs(_) | SetDeadTimeMs(_) => Some(Mode::Parameters),
            Vector { .. } => Some(Mode::Cutting),
        }
    }

    /// True if the controller sends a reply line (CR ack, value, or `*NNN`).
    /// Mode-2 commands are fire-and-forget; only errors come back [SPEC p.6–7].
    pub fn expects_reply(&self) -> bool {
        self.required_mode() != Some(Mode::Cutting)
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut s = self.to_string();
        s.push('\r');
        s.into_bytes()
    }
}

impl fmt::Display for Command {
    /// The command text without the trailing CR.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Command::*;
        match self {
            Version => write!(f, "@V"),
            Status => write!(f, "@X"),
            SetMode(m) => write!(f, "@M{}", m.code()),
            Brake => write!(f, "@B"),
            EmergencyStop => write!(f, "@S"),
            PositionSteps(a) => write!(f, "@L{}", a.upper()),
            ReadInput(n) => write!(f, "@I{n}"),
            Home(axes) => {
                write!(f, "$H")?;
                axes.iter().try_for_each(|a| write!(f, "{}", a.lower()))
            }
            SetOutputs(bits) => write!(f, "$A{bits:02X}"),
            SetSpeed { entry, value } => write!(f, "#G{entry},{value}"),
            SetRampMs(ms) => write!(f, "#R{ms}"),
            SetDeadTimeMs(ms) => write!(f, "#C1,{ms}"),
            Vector { speed_entry, x, y, z } => write!(f, "L{speed_entry},X{x},Y{y},Z{z}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encodings must match what GMC sent in the captures byte for byte.
    #[test]
    fn encodes_like_gmc() {
        let cases: &[(Command, &str)] = &[
            (Command::Version, "@V\r"),
            (Command::Status, "@X\r"),
            (Command::SetMode(Mode::Cutting), "@M2\r"),
            (Command::Brake, "@B\r"),
            (Command::PositionSteps(Axis::X), "@LX\r"),
            (Command::ReadInput(221), "@I221\r"),
            (Command::Home(vec![Axis::X, Axis::Y]), "$Hxy\r"),
            (Command::Home(vec![Axis::Z]), "$Hz\r"),
            (Command::SetOutputs(0x4A), "$A4A\r"),
            (Command::SetOutputs(0x02), "$A02\r"),
            (Command::SetSpeed { entry: 20, value: 36000 }, "#G20,36000\r"),
            (Command::SetRampMs(225), "#R225\r"),
            (Command::SetDeadTimeMs(100), "#C1,100\r"),
            (
                Command::Vector { speed_entry: 20, x: 9695, y: 10775, z: 774 },
                "L20,X9695,Y10775,Z774\r",
            ),
        ];
        for (cmd, wire) in cases {
            assert_eq!(cmd.encode(), wire.as_bytes(), "{cmd:?}");
        }
    }

    #[test]
    fn mode_two_commands_expect_no_reply() {
        let v = Command::Vector { speed_entry: 20, x: 0, y: 0, z: 0 };
        assert!(!v.expects_reply());
        assert!(Command::Status.expects_reply());
        assert_eq!(Command::Brake.required_mode(), None);
    }
}
