//! Commands the host may send, per the CNC-Control protocol v1.00/00b (20 Jul 2000), cited as
//! [SPEC1 p.N]. Commands proven by the GMC captures are marked [CAP].
//!
//! Only commands this software is allowed to send are representable. Deliberately absent:
//! mode 3/4/9 (`@M9` is file transfer and firmware update), `#W` (persist parameters on the
//! controller), `@R` (controller restart), `@Q<n>,b` (setting arbitrary logical outputs),
//! `@D` (forcing internal flags), raw text, and everything the spec marks "noch nicht
//! realisiert" (curves `S`, arcs `K`, mode-2 flags `C2`/`C4`).

use std::fmt;

use crate::Setting;

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
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    /// Position in `[x, y, z]` arrays.
    pub const fn index(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }

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

/// Where a `$L` jog goes, in steps [SPEC1 p.30–31; CAP].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JogTarget {
    /// To an absolute position (`$L<entry>,X<steps>`).
    To(i32),
    /// By a distance from where the axis is (`$L<entry>,x<steps>`).
    By(i32),
}

impl JogTarget {
    /// A relative jog this long (either sign) runs until braked [SPEC p.30; CAP g_motions
    /// `$L20,x-999999`].
    pub const UNTIL_BRAKED: i32 = 999_999;
}

/// An input level, for mode-2 `I` waits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Level {
    High,
    Low,
}

/// From the next motion on, does the `@O` speed override apply (`C8`) or not (`C16`)
/// [SPEC1 p.35]?
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpeedOverride {
    /// Bit 8: the override applies (tool in the material).
    On,
    /// Bit 16: no override (tool not engaged, rapid move).
    Off,
}

/// Mode-2 one-shot special functions `C<bits>` [SPEC1 p.35]. Only the implemented bits exist
/// here; bits 2 (diagonal) and 4 (axes one at a time) are "noch nicht realisiert".
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct StreamFlags {
    /// Bit 1: start the next vector without the `#C1` dead time.
    pub no_dead_time: bool,
    /// Bit 8 or 16: switch the speed override on or off; `None` leaves it as it is.
    pub speed_override: Option<SpeedOverride>,
}

impl StreamFlags {
    pub fn bits(self) -> u8 {
        let speed_override = match self.speed_override {
            None => 0,
            Some(SpeedOverride::On) => 8,
            Some(SpeedOverride::Off) => 16,
        };
        u8::from(self.no_dead_time) | speed_override
    }
}

/// A speed override in percent, 10–200 [SPEC1 p.16]. Values outside that range cannot be
/// built, so `@O` can only carry a valid one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct OverridePct(u16);

impl OverridePct {
    pub const MIN: u16 = 10;
    pub const MAX: u16 = 200;
    /// 100 %: speeds as programmed.
    pub const NORMAL: Self = Self(100);

    /// `None` outside 10–200.
    pub const fn new(pct: u16) -> Option<Self> {
        if pct >= Self::MIN && pct <= Self::MAX {
            Some(Self(pct))
        } else {
            None
        }
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

impl fmt::Display for OverridePct {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} %", self.0)
    }
}

/// A parameter that can be read back with `&…` in mode 0 [SPEC1 p.29–30]. Reading is
/// harmless; nothing here writes a parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Param {
    /// `&C<n>`: configuration value `#C<n>`.
    Config(u8),
    /// `&G<entry>`: speed of a table entry.
    Speed(u8),
    /// `&R`: maximum ramp time (ms).
    Ramp,
    /// `&M`: maximum speed.
    MaxSpeed,
    /// `&S`: start/stop speed.
    StartSpeed,
    /// `&B<entry>`: braking angle of a table entry, 0–90° [SPEC1 p.18]. The query table on
    /// p.29 shows `&B` without an entry, but `#B` takes one (`#By,xx`) and GMC sets it per
    /// entry (`#B20,90` [CAP]), so this asks per entry, like `&G`.
    BrakeAngle(u8),
    /// `&Z<a>`: axis resolution numerator, steps per revolution (GMC: `#ZZ,3000` [CAP g_init]).
    StepsPerRev(Axis),
    /// `&N<a>`: axis resolution denominator, µm per revolution (GMC: `#NZ,10000` [CAP g_init]).
    MicrometresPerRev(Axis),
    /// `&O<a>`: offset driven after homing.
    HomeOffset(Axis),
    /// `&D<a>`: homing direction and speed entries (`dir,g1,g2,g3`; dir 0 = negative).
    Homing(Axis),
    /// `&I<n>`: input definition (`port,bit,invert`; port `000h` = not mapped).
    InputDef(u8),
}

impl fmt::Display for Param {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Param::Config(n) => write!(f, "&C{n}"),
            Param::Speed(g) => write!(f, "&G{g}"),
            Param::Ramp => write!(f, "&R"),
            Param::MaxSpeed => write!(f, "&M"),
            Param::StartSpeed => write!(f, "&S"),
            Param::BrakeAngle(g) => write!(f, "&B{g}"),
            Param::StepsPerRev(a) => write!(f, "&Z{}", a.upper()),
            Param::MicrometresPerRev(a) => write!(f, "&N{}", a.upper()),
            Param::HomeOffset(a) => write!(f, "&O{}", a.upper()),
            Param::Homing(a) => write!(f, "&D{}", a.upper()),
            Param::InputDef(n) => write!(f, "&I{n}"),
        }
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
    /// `@O<pct>`: set the speed override; applies immediately in all modes [SPEC1 p.16].
    SetOverride(OverridePct),
    /// `@@`: not in either spec. GMC sends it before homing and after errors; the controller
    /// answers `\r` and the error state clears [CAP g_init; GMC].
    ClearError,

    // ---- mode 1 (`$`) [SPEC1 p.30–32] ----
    /// `$H<axes>`: home the axes in the given order [CAP].
    Home(Vec<Axis>),
    /// `$A<hh>`: set auxiliary outputs (head/clamp bitmask) [CAP].
    SetOutputs(u8),
    /// `$L<entry>,<axis><steps>`: jog one axis at a speed-table entry [CAP].
    JogSteps {
        speed_entry: u8,
        axis: Axis,
        target: JogTarget,
    },

    // ---- mode 0 (`#` set, `&` query) [SPEC1 p.17–30] ----
    /// `#…`: change a setting.
    Set(Setting),
    /// `&…`: read a parameter back; reply `&C13,1`, `&R300`, `&DX,0,10,11,0`, …
    QueryParam(Param),

    // ---- mode 2: buffered, no reply [SPEC1 p.32–35] ----
    /// `L<entry>,X<x>,Y<y>,Z<z>`: absolute vector in steps at speed-table entry [CAP].
    Vector {
        speed_entry: u8,
        x: i32,
        y: i32,
        z: i32,
    },
    /// `x<dx>,y<dy>,z<dz>`: a follow-on vector, relative to the previous vector's target, at
    /// that vector's speed entry [SPEC1 p.33: lowercase = relative; after the first vector "the
    /// L and the speed entry can be omitted"]. Axes that do not move are left out. Only valid
    /// after a [`Command::Vector`] in the same stream; [`Command::path`] builds them.
    Step { x: i32, y: i32, z: i32 },
    /// `A<hh>`: set auxiliary outputs, in sequence with the path.
    StreamOutputs(u8),
    /// `W<ms>`: dwell, in sequence with the path (e.g. let a valve switch).
    StreamDwellMs(u32),
    /// `G<entry>,<v>`: set a speed-table entry "for all subsequent moves with this entry"
    /// [SPEC1 p.34]. Whether that means in sequence with the stream or from the moment it
    /// arrives is not settled; `gunnar check` stage 5 measures it.
    StreamSetSpeed { entry: u8, value: u32 },
    /// `I<n>` / `I-<n>`: wait until input `n` is high / low. Undefined inputs don't wait.
    StreamWaitInput { input: u8, level: Level },
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
            | BufferLevel | CommandCounter | SetCommandCounter(_) | Override | SetOverride(_)
            | ClearError => None,
            Home(_) | SetOutputs(_) | JogSteps { .. } => Some(Mode::Jog),
            Set(_) | QueryParam(_) => Some(Mode::Parameters),
            Vector { .. }
            | Step { .. }
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

    /// A path through `points` (absolute, in steps) at speed entry `speed_entry`, encoded
    /// compactly: the first point as an absolute `L<entry>,X…,Y…,Z…`, every later one as a
    /// relative [`Command::Step`] (`x31,y-2,z1`). That is about half the bytes per point, and
    /// exact: each step is the difference of two integer positions, so the steps add up to the
    /// last point. Repeated points are skipped.
    pub fn path(speed_entry: u8, points: &[[i32; 3]]) -> Vec<Command> {
        let Some((&first, rest)) = points.split_first() else {
            return Vec::new();
        };
        let mut out = vec![Command::Vector {
            speed_entry,
            x: first[0],
            y: first[1],
            z: first[2],
        }];
        let mut prev = first;
        for &p in rest {
            if p != prev {
                out.push(Command::Step {
                    x: p[0] - prev[0],
                    y: p[1] - prev[1],
                    z: p[2] - prev[2],
                });
                prev = p;
            }
        }
        out
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
            SetOverride(pct) => write!(f, "@O{}", pct.get()),
            ClearError => write!(f, "@@"),
            Home(axes) => {
                write!(f, "$H")?;
                axes.iter().try_for_each(|a| write!(f, "{}", a.lower()))
            }
            SetOutputs(bits) => write!(f, "$A{bits:02X}"),
            JogSteps {
                speed_entry,
                axis,
                target,
            } => match target {
                JogTarget::To(steps) => write!(f, "$L{speed_entry},{}{steps}", axis.upper()),
                JogTarget::By(steps) => write!(f, "$L{speed_entry},{}{steps}", axis.lower()),
            },
            Set(setting) => write!(f, "{setting}"),
            QueryParam(p) => write!(f, "{p}"),
            Vector {
                speed_entry,
                x,
                y,
                z,
            } => write!(f, "L{speed_entry},X{x},Y{y},Z{z}"),
            Step { x, y, z } => {
                let moved: Vec<String> = [('x', x), ('y', y), ('z', z)]
                    .into_iter()
                    .filter(|(_, d)| **d != 0)
                    .map(|(axis, d)| format!("{axis}{d}"))
                    .collect();
                if moved.is_empty() {
                    write!(f, "x0") // no motion, but still a valid vector
                } else {
                    write!(f, "{}", moved.join(","))
                }
            }
            StreamOutputs(bits) => write!(f, "A{bits:02X}"),
            StreamDwellMs(ms) => write!(f, "W{ms}"),
            StreamSetSpeed { entry, value } => write!(f, "G{entry},{value}"),
            StreamWaitInput { input, level } => match level {
                Level::High => write!(f, "I{input}"),
                Level::Low => write!(f, "I-{input}"),
            },
            StreamFlags(flags) => write!(f, "C{}", flags.bits()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encodings must match what GMC sent in the captures byte for byte, and the spec's
    /// examples for commands GMC never sends. (Settings: see `setting.rs`.)
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
                    target: JogTarget::By(-JogTarget::UNTIL_BRAKED),
                },
                "$L20,x-999999\r",
            ),
            (
                Command::JogSteps {
                    speed_entry: 10,
                    axis: Axis::X,
                    target: JogTarget::To(3),
                },
                "$L10,X3\r",
            ),
            (
                Command::Set(Setting::Speed {
                    entry: 20,
                    value: 36000,
                }),
                "#G20,36000\r",
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
            (
                Command::SetOverride(OverridePct::new(120).expect("in range")),
                "@O120\r",
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
                    level: Level::High,
                },
                "I233\r",
            ),
            (
                Command::StreamWaitInput {
                    input: 255,
                    level: Level::Low,
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
                    no_dead_time: true,
                    speed_override: Some(SpeedOverride::On),
                }),
                "C9\r",
            ),
            (
                Command::StreamFlags(StreamFlags {
                    speed_override: Some(SpeedOverride::Off),
                    ..Default::default()
                }),
                "C16\r",
            ),
        ];
        for (cmd, wire) in cases {
            assert_eq!(cmd.encode(), wire.as_bytes(), "{cmd:?}");
        }
    }

    /// The first points of GMC's oval [CAP g_oval_mount], encoded as a compact path.
    #[test]
    fn a_path_is_one_absolute_vector_then_relative_steps() {
        let oval = [
            [7323, 10159, 5],
            [7346, 10158, 16],
            [7370, 10157, 28],
            [7370, 10157, 28], // repeated: skipped
            [7393, 10157, 39], // y unchanged: left out
        ];
        let wire: Vec<String> = Command::path(20, &oval)
            .iter()
            .map(ToString::to_string)
            .collect();
        assert_eq!(
            wire,
            [
                "L20,X7323,Y10159,Z5",
                "x23,y-1,z11",
                "x24,y-1,z12",
                "x23,z11"
            ]
        );
        // Exact: the steps add up to the last point.
        let end = Command::path(20, &oval)
            .iter()
            .fold([0; 3], |at, cmd| match *cmd {
                Command::Vector { x, y, z, .. } => [x, y, z],
                Command::Step { x, y, z } => [at[0] + x, at[1] + y, at[2] + z],
                _ => at,
            });
        assert_eq!(end, oval[4]);
        assert!(Command::path(20, &[]).is_empty());
        assert_eq!(Command::Step { x: 0, y: 0, z: 0 }.to_string(), "x0");
        assert!(!Command::Step { x: 1, y: 0, z: 0 }.expects_reply());
    }

    #[test]
    fn override_range() {
        assert_eq!(OverridePct::new(9), None);
        assert_eq!(OverridePct::new(10).map(OverridePct::get), Some(10));
        assert_eq!(OverridePct::new(200).map(OverridePct::get), Some(200));
        assert_eq!(OverridePct::new(201), None);
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
        assert_eq!(
            Command::Set(Setting::RampMs(300)).required_mode(),
            Some(Mode::Parameters)
        );
    }
}
