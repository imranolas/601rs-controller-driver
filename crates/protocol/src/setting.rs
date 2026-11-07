//! Controller settings this software may change in mode 0 (`#…`), and how to read them back.
//!
//! A [`Setting`] is a parameter with its value; a [`SettingKey`] names one without a value. Every
//! key has a query ([`SettingKey::param`]) whose reply [`Setting::from_reply`] parses back into
//! a `Setting`, so a value can be saved before a change and restored exactly. Nothing here
//! persists a value on the controller (`#W` is not representable).

use std::fmt;

use crate::reply::{parse_flag, parse_param};
use crate::{Axis, Param};

/// `#C11`: the units of speeds and offsets [SPEC1 p.24].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Units {
    /// `#C11,0`: steps and steps/s (Hz). GMC uses this [CAP g_init].
    Steps,
    /// `#C11,1`: µm and µm/s.
    Micrometres,
}

/// `#C35`: which signal inputs report their level changes as bytes [SPEC1 p.7, p.28].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct SignalAcks {
    /// Bit 1: signal input 1 (I221, the green start button): 0x06 high, 0x07 low.
    pub input1: bool,
    /// Bit 2: signal input 2 (I222): 0x08 high, 0x09 low.
    pub input2: bool,
}

impl SignalAcks {
    pub fn bits(self) -> u8 {
        u8::from(self.input1) | (u8::from(self.input2) << 1)
    }

    /// `None` for values with bits beyond the two inputs.
    pub fn from_bits(bits: u8) -> Option<Self> {
        (bits <= 3).then_some(Self {
            input1: bits & 1 != 0,
            input2: bits & 2 != 0,
        })
    }
}

/// `#C19` host watchdog timeout: 0 (off) to 10 000 ms [SPEC1 p.26]. Values outside that range
/// cannot be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct WatchdogMs(u16);

impl WatchdogMs {
    /// The longest timeout the controller accepts; 0 switches the watchdog off.
    pub const MAX: u16 = 10_000;

    /// `None` above [`Self::MAX`].
    pub const fn new(ms: u16) -> Option<Self> {
        if ms <= Self::MAX {
            Some(Self(ms))
        } else {
            None
        }
    }

    /// `ms`, or the longest timeout if `ms` is longer.
    pub fn saturating(ms: u64) -> Self {
        Self(u16::try_from(ms.min(u64::from(Self::MAX))).unwrap_or(Self::MAX))
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// The radius slowdown registers `#C20`–`#C28` [SPEC1 p.9, p.26–27; protocol reference
/// §2.4.1]: how the controller slows down on tight curves in mode 2.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Radius {
    /// `#C20`: RADMIN, the radius (steps) at which the carriage runs at the start speed `#S`.
    Min,
    /// `#C21`: RADMAX, the radius (steps) from which it runs at full speed.
    Max,
    /// `#C24`: a new speed differing by less than this (%) is ignored.
    LowerPct,
    /// `#C25`: a new speed differing by more than this (%) applies at once.
    UpperPct,
    /// `#C26`: how often a faster speed must recur before it applies.
    FasterCount,
    /// `#C27`: how often a slower speed must recur before it applies.
    SlowerCount,
    /// `#C28`: steepness of the slowdown below RADMIN.
    Steepness,
}

impl Radius {
    /// The register number.
    pub const fn register(self) -> u8 {
        match self {
            Self::Min => 20,
            Self::Max => 21,
            Self::LowerPct => 24,
            Self::UpperPct => 25,
            Self::FasterCount => 26,
            Self::SlowerCount => 27,
            Self::Steepness => 28,
        }
    }
}

/// A setting with its value: one mode-0 `#…` command. Read one back with
/// [`Setting::from_reply`]; [`Setting::key`] says which it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Setting {
    /// `#C1,<ms>`: mode-2 dead time before a path starts [CAP].
    DeadTimeMs(u16),
    /// `#C5`: send 0x01 when a motion finishes [CAP].
    PositionedSignal(bool),
    /// `#C6`: send 0x04 after homing. Keep it on: when off, the controller holds the CR reply
    /// to `$H` back until homing has finished, which outlasts any sensible reply deadline
    /// [SPEC1 p.23].
    HomingDoneSignal(bool),
    /// `#C7`: the 0x03/0x02 buffer flow-control bytes. GMC never enables them and does not
    /// expect them, so whoever turns this on must turn it off before handing the machine back.
    FlowControl(bool),
    /// `#C8,<ms>`: heartbeat: the controller sends 0x05 after this long without sending
    /// anything; 0 = off [SPEC1 p.23].
    HeartbeatMs(u16),
    /// `#C11`: units of speeds and offsets [SPEC1 p.24; CAP g_init].
    Units(Units),
    /// `#C19,<ms>`: host watchdog. While axes move, the controller aborts with `*068` if
    /// nothing arrives from the host for this long [SPEC1 p.26]. The session keeps talking
    /// during waits (`@X` polls, `@H` while the stream is held), each at
    /// `Config::poll_interval`, so choose a value well above that (≥ 4×).
    HostWatchdog(WatchdogMs),
    /// `#C35`: level-change bytes for the signal inputs [SPEC1 p.28]. GMC sends `#C35,1` [CAP].
    SignalAcks(SignalAcks),
    /// `#G<entry>,<value>`: speed of a speed-table entry [CAP].
    Speed { entry: u8, value: u32 },
    /// `#R<ms>`: maximum ramp time [CAP].
    RampMs(u16),
    /// `#S<v>`: start/stop speed (steps/s with `#C11,0`): motors start at it without a ramp
    /// [SPEC1 p.8, p.19; CAP].
    StartSpeed(u32),
    /// `#B<entry>,<degrees>`: braking angle of a speed-table entry, 0–90°: a sharper direction
    /// change brakes to a stop [SPEC1 p.19; CAP `#B20,90`].
    BrakeAngle { entry: u8, degrees: u8 },
    /// `#C20`–`#C28`: the radius slowdown [CAP].
    Radius(Radius, u32),
    /// `#M<v>`: the maximum speed (steps/s with `#C11,0`). Ramps (`#R`) are the time to reach
    /// it [SPEC1 p.8, p.19; CAP g_init `#M36000`].
    MaxSpeed(u32),
    /// `#Z<a>,<n>`: steps per revolution of an axis; with `#C13` the controller turns a rotary
    /// axis the short way round [SPEC1 p.20; CAP g_init `#ZZ,3000`].
    StepsPerRev { axis: Axis, value: u32 },
    /// `#N<a>,<n>`: µm per revolution of an axis [SPEC1 p.20; CAP g_init `#NZ,10000`].
    MicrometresPerRev { axis: Axis, value: u32 },
    /// `#C2`: extra braking criteria, bits: 1 an XY-to-Z direction change beyond the braking
    /// angle, 2 a change of speed entry, 4 the start and end of a Z move, 8 U moves [SPEC1
    /// p.22]. GMC sends `#C2,0` [CAP g_init].
    BrakeCriteria(u8),
    /// `#C12`: ramp when homing reaches the switch [SPEC1 p.24; CAP g_init `#C12,1`].
    HomingRamp(bool),
    /// `#C13`: Z is a rotary axis, always turned the short way [SPEC1 p.24; CAP `#C13,1`].
    RotaryZ(bool),
    /// `#C14`: dynamic path speed, bits: 1 diagonal, 2 radius, 4 tangential (the radius from
    /// the Z axis), 8 buffer fill [SPEC1 p.24–25]. GMC sends `#C14,13` [CAP g_init].
    DynamicSpeed(u8),
    /// `#O<a>,<value>`: offset driven after homing (steps with `#C11,0`) [SPEC1 p.20; CAP g_init].
    HomeOffset { axis: Axis, value: i32 },
}

/// Which setting, without its value: what to read back before changing it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SettingKey {
    DeadTime,
    PositionedSignal,
    HomingDoneSignal,
    FlowControl,
    Heartbeat,
    Units,
    HostWatchdog,
    SignalAcks,
    Speed(u8),
    Ramp,
    StartSpeed,
    BrakeAngle(u8),
    Radius(Radius),
    MaxSpeed,
    StepsPerRev(Axis),
    MicrometresPerRev(Axis),
    BrakeCriteria,
    HomingRamp,
    RotaryZ,
    DynamicSpeed,
    HomeOffset(Axis),
}

impl SettingKey {
    /// The query that reads this setting back.
    pub const fn param(self) -> Param {
        match self {
            Self::DeadTime => Param::Config(1),
            Self::PositionedSignal => Param::Config(5),
            Self::HomingDoneSignal => Param::Config(6),
            Self::FlowControl => Param::Config(7),
            Self::Heartbeat => Param::Config(8),
            Self::Units => Param::Config(11),
            Self::HostWatchdog => Param::Config(19),
            Self::SignalAcks => Param::Config(35),
            Self::Speed(entry) => Param::Speed(entry),
            Self::Ramp => Param::Ramp,
            Self::StartSpeed => Param::StartSpeed,
            Self::BrakeAngle(entry) => Param::BrakeAngle(entry),
            Self::Radius(r) => Param::Config(r.register()),
            Self::MaxSpeed => Param::MaxSpeed,
            Self::StepsPerRev(axis) => Param::StepsPerRev(axis),
            Self::MicrometresPerRev(axis) => Param::MicrometresPerRev(axis),
            Self::BrakeCriteria => Param::Config(2),
            Self::HomingRamp => Param::Config(12),
            Self::RotaryZ => Param::Config(13),
            Self::DynamicSpeed => Param::Config(14),
            Self::HomeOffset(axis) => Param::HomeOffset(axis),
        }
    }
}

impl fmt::Display for SettingKey {
    /// The query that reads it, e.g. `&C19`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.param().fmt(f)
    }
}

impl Setting {
    pub const fn key(&self) -> SettingKey {
        match *self {
            Self::DeadTimeMs(_) => SettingKey::DeadTime,
            Self::PositionedSignal(_) => SettingKey::PositionedSignal,
            Self::HomingDoneSignal(_) => SettingKey::HomingDoneSignal,
            Self::FlowControl(_) => SettingKey::FlowControl,
            Self::HeartbeatMs(_) => SettingKey::Heartbeat,
            Self::Units(_) => SettingKey::Units,
            Self::HostWatchdog(_) => SettingKey::HostWatchdog,
            Self::SignalAcks(_) => SettingKey::SignalAcks,
            Self::Speed { entry, .. } => SettingKey::Speed(entry),
            Self::RampMs(_) => SettingKey::Ramp,
            Self::StartSpeed(_) => SettingKey::StartSpeed,
            Self::BrakeAngle { entry, .. } => SettingKey::BrakeAngle(entry),
            Self::Radius(r, _) => SettingKey::Radius(r),
            Self::MaxSpeed(_) => SettingKey::MaxSpeed,
            Self::StepsPerRev { axis, .. } => SettingKey::StepsPerRev(axis),
            Self::MicrometresPerRev { axis, .. } => SettingKey::MicrometresPerRev(axis),
            Self::BrakeCriteria(_) => SettingKey::BrakeCriteria,
            Self::HomingRamp(_) => SettingKey::HomingRamp,
            Self::RotaryZ(_) => SettingKey::RotaryZ,
            Self::DynamicSpeed(_) => SettingKey::DynamicSpeed,
            Self::HomeOffset { axis, .. } => SettingKey::HomeOffset(axis),
        }
    }

    /// Parse the reply to `key`'s query (`&C19,400`, `&G20,36000`, `&OX, 150`, …) into the
    /// setting it reports. `None` if the line does not answer that query or its value is
    /// malformed or out of range: never a guess.
    pub fn from_reply(key: SettingKey, line: &str) -> Option<Self> {
        let fields = parse_param(line, key.param())?;
        let [value] = fields.as_slice() else {
            return None;
        };
        Some(match key {
            SettingKey::DeadTime => Self::DeadTimeMs(value.parse().ok()?),
            SettingKey::PositionedSignal => Self::PositionedSignal(parse_flag(value)?),
            SettingKey::HomingDoneSignal => Self::HomingDoneSignal(parse_flag(value)?),
            SettingKey::FlowControl => Self::FlowControl(parse_flag(value)?),
            SettingKey::Heartbeat => Self::HeartbeatMs(value.parse().ok()?),
            SettingKey::Units => Self::Units(match value.as_str() {
                "0" => Units::Steps,
                "1" => Units::Micrometres,
                _ => return None,
            }),
            SettingKey::HostWatchdog => Self::HostWatchdog(WatchdogMs::new(value.parse().ok()?)?),
            SettingKey::SignalAcks => Self::SignalAcks(SignalAcks::from_bits(value.parse().ok()?)?),
            SettingKey::Speed(entry) => Self::Speed {
                entry,
                value: value.parse().ok()?,
            },
            SettingKey::Ramp => Self::RampMs(value.parse().ok()?),
            SettingKey::StartSpeed => Self::StartSpeed(value.parse().ok()?),
            SettingKey::BrakeAngle(entry) => Self::BrakeAngle {
                entry,
                degrees: value.parse().ok().filter(|&d| d <= 90)?,
            },
            SettingKey::Radius(r) => Self::Radius(r, value.parse().ok()?),
            SettingKey::MaxSpeed => Self::MaxSpeed(value.parse().ok()?),
            SettingKey::StepsPerRev(axis) => Self::StepsPerRev {
                axis,
                value: value.parse().ok()?,
            },
            SettingKey::MicrometresPerRev(axis) => Self::MicrometresPerRev {
                axis,
                value: value.parse().ok()?,
            },
            SettingKey::BrakeCriteria => {
                Self::BrakeCriteria(value.parse().ok().filter(|&b| b < 16)?)
            }
            SettingKey::HomingRamp => Self::HomingRamp(parse_flag(value)?),
            SettingKey::RotaryZ => Self::RotaryZ(parse_flag(value)?),
            SettingKey::DynamicSpeed => Self::DynamicSpeed(value.parse().ok().filter(|&b| b < 16)?),
            SettingKey::HomeOffset(axis) => Self::HomeOffset {
                axis,
                value: value.parse().ok()?,
            },
        })
    }
}

impl fmt::Display for Setting {
    /// The command text without the trailing CR.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flag = |on: bool| u8::from(on);
        match *self {
            Self::DeadTimeMs(ms) => write!(f, "#C1,{ms}"),
            Self::PositionedSignal(on) => write!(f, "#C5,{}", flag(on)),
            Self::HomingDoneSignal(on) => write!(f, "#C6,{}", flag(on)),
            Self::FlowControl(on) => write!(f, "#C7,{}", flag(on)),
            Self::HeartbeatMs(ms) => write!(f, "#C8,{ms}"),
            Self::Units(units) => write!(f, "#C11,{}", flag(units == Units::Micrometres)),
            Self::HostWatchdog(ms) => write!(f, "#C19,{}", ms.get()),
            Self::SignalAcks(acks) => write!(f, "#C35,{}", acks.bits()),
            Self::Speed { entry, value } => write!(f, "#G{entry},{value}"),
            Self::RampMs(ms) => write!(f, "#R{ms}"),
            Self::StartSpeed(v) => write!(f, "#S{v}"),
            Self::BrakeAngle { entry, degrees } => write!(f, "#B{entry},{degrees}"),
            Self::Radius(r, v) => write!(f, "#C{},{v}", r.register()),
            Self::MaxSpeed(v) => write!(f, "#M{v}"),
            Self::StepsPerRev { axis, value } => write!(f, "#Z{},{value}", axis.upper()),
            Self::MicrometresPerRev { axis, value } => write!(f, "#N{},{value}", axis.upper()),
            Self::BrakeCriteria(bits) => write!(f, "#C2,{bits}"),
            Self::HomingRamp(on) => write!(f, "#C12,{}", flag(on)),
            Self::RotaryZ(on) => write!(f, "#C13,{}", flag(on)),
            Self::DynamicSpeed(bits) => write!(f, "#C14,{bits}"),
            Self::HomeOffset { axis, value } => write!(f, "#O{},{value}", axis.upper()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What each setting looks like on the wire, and the reply that reads it back.
    const CASES: &[(Setting, &str, &str)] = &[
        (Setting::DeadTimeMs(100), "#C1,100", "&C1,100"),
        (Setting::PositionedSignal(true), "#C5,1", "&C5,1"),
        (Setting::HomingDoneSignal(false), "#C6,0", "&C6,0"),
        (Setting::FlowControl(true), "#C7,1", "&C7,1"),
        (Setting::HeartbeatMs(400), "#C8,400", "&C8,400"),
        (Setting::Units(Units::Steps), "#C11,0", "&C11,0"),
        (
            Setting::HostWatchdog(WatchdogMs(500)),
            "#C19,500",
            "&C19,500",
        ),
        (
            Setting::SignalAcks(SignalAcks {
                input1: true,
                input2: false,
            }),
            "#C35,1",
            "&C35,1",
        ),
        (
            Setting::Speed {
                entry: 20,
                value: 36000,
            },
            "#G20,36000",
            "&G20,36000",
        ),
        (Setting::RampMs(225), "#R225", "&R225"),
        (Setting::StartSpeed(4000), "#S4000", "&S4000"),
        (
            Setting::BrakeAngle {
                entry: 20,
                degrees: 90,
            },
            "#B20,90",
            "&B20,90",
        ),
        (
            Setting::Radius(Radius::Max, 12000),
            "#C21,12000",
            "&C21,12000",
        ),
        (Setting::Radius(Radius::Steepness, 1), "#C28,1", "&C28,1"),
        (Setting::MaxSpeed(36000), "#M36000", "&M36000"),
        (
            Setting::StepsPerRev {
                axis: Axis::Z,
                value: 3000,
            },
            "#ZZ,3000",
            "&ZZ,3000",
        ),
        (
            Setting::MicrometresPerRev {
                axis: Axis::Z,
                value: 10000,
            },
            "#NZ,10000",
            "&NZ,10000",
        ),
        (Setting::BrakeCriteria(0), "#C2,0", "&C2,0"),
        (Setting::HomingRamp(true), "#C12,1", "&C12,1"),
        (Setting::RotaryZ(true), "#C13,1", "&C13,1"),
        (Setting::DynamicSpeed(13), "#C14,13", "&C14,13"),
        (
            Setting::HomeOffset {
                axis: Axis::X,
                value: 150,
            },
            "#OX,150",
            "&OX, 150",
        ),
    ];

    #[test]
    fn encodes_and_reads_back_every_setting() {
        for &(setting, wire, reply) in CASES {
            assert_eq!(setting.to_string(), wire);
            assert_eq!(
                Setting::from_reply(setting.key(), reply),
                Some(setting),
                "{reply}"
            );
        }
    }

    #[test]
    fn rejects_values_it_cannot_restore_exactly() {
        let read = Setting::from_reply;
        assert_eq!(
            read(SettingKey::HostWatchdog, "&C19,10001"),
            None,
            "above max"
        );
        assert_eq!(
            read(SettingKey::PositionedSignal, "&C5,2"),
            None,
            "not a flag"
        );
        assert_eq!(read(SettingKey::SignalAcks, "&C35,4"), None, "unknown bit");
        assert_eq!(read(SettingKey::DeadTime, "&C1,-1"), None);
        assert_eq!(read(SettingKey::DeadTime, "&C13,1"), None, "other query");
        assert_eq!(read(SettingKey::Speed(20), "&G20,1,2"), None, "extra field");
    }

    #[test]
    fn watchdog_range() {
        assert_eq!(WatchdogMs::new(10_000).map(WatchdogMs::get), Some(10_000));
        assert_eq!(WatchdogMs::new(10_001), None);
        assert_eq!(WatchdogMs::saturating(60_000).get(), 10_000);
    }
}
