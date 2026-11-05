//! Commands the host may send, per the CNC-Control protocol v1.00/00b (20 Jul 2000), cited as
//! [SPEC1 p.N]. Commands proven by the GMC captures are marked [CAP].
//!
//! Only commands this software is allowed to send are representable. Deliberately absent:
//! mode 3/4/9 (`@M9` is file transfer and firmware update), `#W` (persist parameters on the
//! controller), `@R` (controller restart), `@Q<n>,b` (setting arbitrary logical outputs),
//! `@D` (forcing internal flags), raw text, and everything the spec marks "noch nicht
//! realisiert" (curves `S`, arcs `K`, mode-2 flags `C2`/`C4`).

use std::fmt;

/// Controller operating mode [SPEC1 p.6, p.12].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Mode 0: parameters (`#` set, `&` query).
    Parameters,
    /// Mode 1: jog, homing, outputs (`$` commands).
    Jog,
    /// Mode 2: cutting; buffered path commands, no per-command reply.
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
    pub const fn upper(self) -> char {
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

/// Mode-2 one-shot special functions `C<bits>` [SPEC1 p.35]. Only the implemented bits exist
/// here; bits 2 (diagonal) and 4 (axes one at a time) are "noch nicht realisiert".
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct StreamFlags {
    /// Bit 1: start the next vector without the `#C1` dead time.
    pub no_dead_time: bool,
    /// Bit 8: from here, the speed override applies (tool in the material).
    pub override_on: bool,
    /// Bit 16: no speed override (tool not engaged, rapid move).
    pub override_off: bool,
}

impl StreamFlags {
    pub const fn bits(self) -> u8 {
        (self.no_dead_time as u8)
            | ((self.override_on as u8) << 3)
            | ((self.override_off as u8) << 4)
    }
}

/// A single controller command. `encode` produces the exact bytes, including the trailing CR.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    // ---- master commands: accepted in any mode, processed immediately [SPEC1 p.12–17] ----
    /// `@V`: version string [CAP].
    Version,
    /// `@X`: status (see [`crate::Status`]) [CAP].
    Status,
    /// `@M`: query the current mode; reply `@M0n`.
    QueryMode,
    /// `@M<n>`: switch mode (only while stopped) [CAP].
    SetMode(Mode),
    /// `@B`: brake all axes with ramp; positions kept [CAP].
    Brake,
    /// `@S`: stop immediately; positions lost (re-home required).
    EmergencyStop,
    /// `@L<a>`: axis position in steps [CAP].
    PositionSteps(Axis),
    /// `@I<n>`: read logical input (e.g. 221 = start button) [CAP].
    ReadInput(u8),
    /// `@Q<n>`: read logical output (Q242–Q249 = auxiliary signals 1–8, i.e. the `$A` bits).
    ReadOutput(u8),
    /// `@Y`: switch inputs (stop, ready, reference, limits) as 16 bits; see [`crate::Switches`].
    Switches,
    /// `@E`: current error code (0 = none), same numbering as `*NNN`.
    ErrorCode,
    /// `@H`: interface buffer fill level in percent.
    BufferLevel,
    /// `@N`: number of the mode-2 command currently executing (progress).
    CommandCounter,
    /// `@N<n>`: set the mode-2 command counter.
    SetCommandCounter(u32),
    /// `@O`: query the speed override in percent.
    Override,
    /// `@O<pct>`: set the speed override, 10–200 %; applies immediately in all modes.
    SetOverride(u16),

    // ---- mode 1 (`$`) [SPEC1 p.30–32] ----
    /// `$H<axes>`: home the axes in the given order [CAP].
    Home(Vec<Axis>),
    /// `$A<hh>`: set auxiliary outputs (head/clamp bitmask) [CAP].
    SetOutputs(u8),
    /// `$L<entry>,<axis><steps>`: jog one axis in steps at speed-table entry; `relative`
    /// selects the lowercase (relative) axis letter. ±999999 means "run until braked" [CAP].
    JogSteps {
        speed_entry: u8,
        axis: Axis,
        steps: i32,
        relative: bool,
    },

    // ---- mode 0 (`#`) [SPEC1 p.17–29] ----
    /// `#G<entry>,<v>`: speed of speed-table entry [CAP].
    SetSpeed { entry: u8, value: u32 },
    /// `#R<ms>`: maximum ramp time [CAP].
    SetRampMs(u16),
    /// `#C1,<ms>`: mode-2 dead time before a path starts [CAP].
    SetDeadTimeMs(u16),
    /// `#C5,<0|1>`: send 0x01 when a motion finishes [CAP].
    SetPositionedSignal(bool),
    /// `#C6,<0|1>`: send 0x04 after homing. When off, the CR reply to `$H` is held back until
    /// homing has finished [SPEC1 p.23].
    SetHomingDoneSignal(bool),
    /// `#C7,<0|1>`: 0x03/0x02 buffer flow-control bytes. GMC never enables this, so whoever
    /// enables it must disable it again before handing the machine back to GMC.
    SetFlowControl(bool),
    /// `#C8,<ms>`: heartbeat; the controller sends 0x05 after this long without sending
    /// anything; 0 disables it [SPEC1 p.23].
    SetHeartbeatMs(u16),
    /// `#C19,<ms>`: host watchdog. The controller aborts motion (`*068`) if nothing arrives from
    /// the host for this long while axes move; 0 disables it [SPEC1 p.26].
    SetHostWatchdogMs(u16),
    /// `#C35,<0..3>`: level-change bytes for signal inputs I221 (0x06 high / 0x07 low) and
    /// I222 (0x08 / 0x09) [SPEC1 p.28]. GMC sends `#C35,1` [CAP].
    SetSignalAcks { input1: bool, input2: bool },

    // ---- mode 2: buffered, no reply [SPEC1 p.32–35] ----
    /// `L<entry>,X<x>,Y<y>,Z<z>`: absolute vector in steps at speed-table entry [CAP].
    Vector {
        speed_entry: u8,
        x: i32,
        y: i32,
        z: i32,
    },
    /// `A<hh>`: set auxiliary outputs, in sequence with the path.
    StreamOutputs(u8),
    /// `W<ms>`: dwell, in sequence with the path (e.g. let a valve switch).
    StreamDwellMs(u32),
    /// `G<entry>,<v>`: change a speed-table entry, in sequence with the path.
    StreamSetSpeed { entry: u8, value: u32 },
    /// `I<n>` / `I-<n>`: wait until input `n` is high / low. Undefined inputs don't wait.
    StreamWaitInput { input: u8, high: bool },
    /// `C<bits>`: one-shot special functions for the next motion.
    StreamFlags(StreamFlags),
}

impl Command {
    /// A pure master query: answered with a value in any state, never with `*NNN`. A `*NNN`
    /// that arrives while one is outstanding is therefore an asynchronous fault (e.g. a limit
    /// switch reached during the poll), not the reply.
    pub fn is_query(&self) -> bool {
        use Command::*;
        matches!(
            self,
            Version
                | Status
                | QueryMode
                | PositionSteps(_)
                | Switches
                | ErrorCode
                | BufferLevel
                | CommandCounter
                | Override
        )
    }

    /// The mode this command requires, or `None` for master commands (any mode).
    pub fn required_mode(&self) -> Option<Mode> {
        use Command::*;
        match self {
            Version | Status | QueryMode | SetMode(_) | Brake | EmergencyStop
            | PositionSteps(_) | ReadInput(_) | ReadOutput(_) | Switches | ErrorCode
            | BufferLevel | CommandCounter | SetCommandCounter(_) | Override | SetOverride(_) => {
                None
            }
            Home(_) | SetOutputs(_) | JogSteps { .. } => Some(Mode::Jog),
            SetSpeed { .. }
            | SetRampMs(_)
            | SetDeadTimeMs(_)
            | SetPositionedSignal(_)
            | SetHomingDoneSignal(_)
            | SetFlowControl(_)
            | SetHeartbeatMs(_)
            | SetHostWatchdogMs(_)
            | SetSignalAcks { .. } => Some(Mode::Parameters),
            Vector { .. }
            | StreamOutputs(_)
            | StreamDwellMs(_)
            | StreamSetSpeed { .. }
            | StreamWaitInput { .. }
            | StreamFlags(_) => Some(Mode::Cutting),
        }
    }

    /// True if the controller sends a reply line (CR ack, value, or `*NNN`).
    /// Mode-2 commands are fire-and-forget; only errors come back [SPEC1 p.6–7].
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
            QueryMode => write!(f, "@M"),
            SetMode(m) => write!(f, "@M{}", m.code()),
            Brake => write!(f, "@B"),
            EmergencyStop => write!(f, "@S"),
            PositionSteps(a) => write!(f, "@L{}", a.upper()),
            ReadInput(n) => write!(f, "@I{n}"),
            ReadOutput(n) => write!(f, "@Q{n}"),
            Switches => write!(f, "@Y"),
            ErrorCode => write!(f, "@E"),
            BufferLevel => write!(f, "@H"),
            CommandCounter => write!(f, "@N"),
            SetCommandCounter(n) => write!(f, "@N{n}"),
            Override => write!(f, "@O"),
            SetOverride(pct) => write!(f, "@O{pct}"),
            Home(axes) => {
                write!(f, "$H")?;
                axes.iter().try_for_each(|a| write!(f, "{}", a.lower()))
            }
            SetOutputs(bits) => write!(f, "$A{bits:02X}"),
            JogSteps {
                speed_entry,
                axis,
                steps,
                relative,
            } => {
                let a = if *relative {
                    axis.lower()
                } else {
                    axis.upper()
                };
                write!(f, "$L{speed_entry},{a}{steps}")
            }
            SetSpeed { entry, value } => write!(f, "#G{entry},{value}"),
            SetRampMs(ms) => write!(f, "#R{ms}"),
            SetDeadTimeMs(ms) => write!(f, "#C1,{ms}"),
            SetPositionedSignal(on) => write!(f, "#C5,{}", u8::from(*on)),
            SetHomingDoneSignal(on) => write!(f, "#C6,{}", u8::from(*on)),
            SetFlowControl(on) => write!(f, "#C7,{}", u8::from(*on)),
            SetHeartbeatMs(ms) => write!(f, "#C8,{ms}"),
            SetHostWatchdogMs(ms) => write!(f, "#C19,{ms}"),
            SetSignalAcks { input1, input2 } => {
                write!(f, "#C35,{}", u8::from(*input1) | (u8::from(*input2) << 1))
            }
            Vector {
                speed_entry,
                x,
                y,
                z,
            } => write!(f, "L{speed_entry},X{x},Y{y},Z{z}"),
            StreamOutputs(bits) => write!(f, "A{bits:02X}"),
            StreamDwellMs(ms) => write!(f, "W{ms}"),
            StreamSetSpeed { entry, value } => write!(f, "G{entry},{value}"),
            StreamWaitInput { input, high } => {
                write!(f, "I{}{input}", if *high { "" } else { "-" })
            }
            StreamFlags(flags) => write!(f, "C{}", flags.bits()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encodings must match what GMC sent in the captures byte for byte, and the spec's
    /// examples for commands GMC never sends.
    #[test]
    fn encodes_like_gmc_and_the_spec() {
        let cases: &[(Command, &str)] = &[
            // [CAP]
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
            (
                Command::JogSteps {
                    speed_entry: 20,
                    axis: Axis::X,
                    steps: -999_999,
                    relative: true,
                },
                "$L20,x-999999\r",
            ),
            (
                Command::JogSteps {
                    speed_entry: 10,
                    axis: Axis::X,
                    steps: 3,
                    relative: false,
                },
                "$L10,X3\r",
            ),
            (
                Command::SetSpeed {
                    entry: 20,
                    value: 36000,
                },
                "#G20,36000\r",
            ),
            (Command::SetRampMs(225), "#R225\r"),
            (Command::SetDeadTimeMs(100), "#C1,100\r"),
            (Command::SetPositionedSignal(true), "#C5,1\r"),
            (
                Command::SetSignalAcks {
                    input1: true,
                    input2: false,
                },
                "#C35,1\r",
            ),
            (
                Command::Vector {
                    speed_entry: 20,
                    x: 9695,
                    y: 10775,
                    z: 774,
                },
                "L20,X9695,Y10775,Z774\r",
            ),
            // [SPEC1]
            (Command::QueryMode, "@M\r"),
            (Command::ReadOutput(250), "@Q250\r"),
            (Command::Switches, "@Y\r"),
            (Command::ErrorCode, "@E\r"),
            (Command::BufferLevel, "@H\r"),
            (Command::CommandCounter, "@N\r"),
            (Command::SetCommandCounter(0), "@N0\r"),
            (Command::Override, "@O\r"),
            (Command::SetOverride(120), "@O120\r"),
            (Command::SetHomingDoneSignal(true), "#C6,1\r"),
            (Command::SetFlowControl(false), "#C7,0\r"),
            (Command::SetHeartbeatMs(400), "#C8,400\r"),
            (Command::SetHostWatchdogMs(500), "#C19,500\r"),
            (
                Command::SetSignalAcks {
                    input1: true,
                    input2: true,
                },
                "#C35,3\r",
            ),
            (Command::StreamOutputs(0x1B), "A1B\r"),
            (Command::StreamDwellMs(250), "W250\r"),
            (
                Command::StreamSetSpeed {
                    entry: 23,
                    value: 33000,
                },
                "G23,33000\r",
            ),
            (
                Command::StreamWaitInput {
                    input: 233,
                    high: true,
                },
                "I233\r",
            ),
            (
                Command::StreamWaitInput {
                    input: 255,
                    high: false,
                },
                "I-255\r",
            ),
            (
                Command::StreamFlags(StreamFlags {
                    no_dead_time: true,
                    ..Default::default()
                }),
                "C1\r",
            ),
            (
                Command::StreamFlags(StreamFlags {
                    override_on: true,
                    override_off: false,
                    no_dead_time: true,
                }),
                "C9\r",
            ),
            (
                Command::StreamFlags(StreamFlags {
                    override_off: true,
                    ..Default::default()
                }),
                "C16\r",
            ),
        ];
        for (cmd, wire) in cases {
            assert_eq!(cmd.encode(), wire.as_bytes(), "{cmd:?}");
        }
    }

    #[test]
    fn mode_two_commands_expect_no_reply() {
        for cmd in [
            Command::Vector {
                speed_entry: 20,
                x: 0,
                y: 0,
                z: 0,
            },
            Command::StreamOutputs(0),
            Command::StreamDwellMs(1),
            Command::StreamFlags(StreamFlags::default()),
        ] {
            assert!(!cmd.expects_reply(), "{cmd}");
        }
        assert!(Command::Status.expects_reply());
        assert_eq!(Command::Brake.required_mode(), None);
        assert_eq!(Command::CommandCounter.required_mode(), None);
    }
}
