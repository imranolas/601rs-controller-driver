//! What can be calibrated: each value's key, in the file and in the history as a dotted path
//! (`machine.bevel.offset.x`, `blade.bevel.lean`), its unit, its words for a person, and the
//! bounds a value must keep to be one at all. The bounds catch a typing slip or a
//! corrupt file; whether a change is plausible is the fit's question, not this.

use std::fmt;
use std::str::FromStr;

use program::Head;
use serde::{Deserialize, Serialize};

use crate::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Axis {
    X,
    Y,
}

/// A side of the board: the back, up, where the blade enters; the face, the customer's, down.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Surface {
    Back,
    Face,
}

/// One of a profile's blade trims (PROGRAM_FORMAT §3): the bevel head's on straight edges and
/// on closed curves, and the straight head's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Trim {
    Straight,
    Bevel,
    BevelRound,
}

/// How a head cuts: the tool model (`compiler::HeadCut`, CALIBRATION.md §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum BladeValue {
    /// Where the slit's middle lies on the back, mm left of the line as the cut travels.
    Sideways,
    /// How far apart the slit's sides are, mm.
    Width,
    /// The blade's lean from vertical, degrees, leftward as it goes down.
    Lean,
    /// How far a cut starts before its commanded start, on that side of the board, mm.
    Lead(Surface),
    /// How far a cut stops short of its commanded end, on that side of the board, mm.
    Shortfall(Surface),
    /// A closed curve's overlap, ms at its speed.
    OverlapMs,
    /// How far the overlap drifts toward the waste by its end, mm.
    RunOutMm,
}

/// A board profile's own settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ProfileValue {
    Name,
    ThicknessMm,
    SpeedPct,
    HeadSettleMs,
    Trim(Trim),
}

/// One value that can be calibrated.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub enum Key {
    /// Where a head's blade is from the carriage (`machine::Head::offset_mm`).
    HeadOffset(Head, Axis),
    /// Steps per mm (`machine::Scale`).
    Scale(Axis),
    /// How a head cuts, on any board.
    Blade { head: Head, value: BladeValue },
    /// A board profile's setting.
    Profile {
        profile: String,
        value: ProfileValue,
    },
}

/// A value: a number, or (a profile's name) text.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Val {
    Num(f64),
    Text(String),
}

impl Val {
    pub fn num(&self) -> Option<f64> {
        match self {
            Val::Num(v) => Some(*v),
            Val::Text(_) => None,
        }
    }

    pub fn text(&self) -> Option<&str> {
        match self {
            Val::Text(t) => Some(t),
            Val::Num(_) => None,
        }
    }
}

impl fmt::Display for Val {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Val::Num(v) => write!(f, "{v}"),
            Val::Text(t) => f.write_str(t),
        }
    }
}

pub(crate) const HEADS: [Head; 2] = [Head::Bevel, Head::Straight];

fn head_word(h: Head) -> &'static str {
    match h {
        Head::Bevel => "bevel",
        Head::Straight => "straight",
    }
}

fn head_of(s: &str) -> Option<Head> {
    HEADS.into_iter().find(|h| head_word(*h) == s)
}

/// A profile id: letters, digits, `_` and `-`, so it can sit in a dotted key.
pub fn valid_profile_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 32
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

impl BladeValue {
    fn path(self) -> &'static str {
        match self {
            BladeValue::Sideways => "sideways",
            BladeValue::Width => "width",
            BladeValue::Lean => "lean",
            BladeValue::Lead(Surface::Back) => "lead.back",
            BladeValue::Lead(Surface::Face) => "lead.face",
            BladeValue::Shortfall(Surface::Back) => "shortfall.back",
            BladeValue::Shortfall(Surface::Face) => "shortfall.face",
            BladeValue::OverlapMs => "overlap_ms",
            BladeValue::RunOutMm => "run_out_mm",
        }
    }

    pub const ALL: [BladeValue; 9] = [
        BladeValue::Sideways,
        BladeValue::Width,
        BladeValue::Lean,
        BladeValue::Lead(Surface::Back),
        BladeValue::Lead(Surface::Face),
        BladeValue::Shortfall(Surface::Back),
        BladeValue::Shortfall(Surface::Face),
        BladeValue::OverlapMs,
        BladeValue::RunOutMm,
    ];
}

impl ProfileValue {
    fn path(self) -> &'static str {
        match self {
            ProfileValue::Name => "name",
            ProfileValue::ThicknessMm => "thickness_mm",
            ProfileValue::SpeedPct => "speed_pct",
            ProfileValue::HeadSettleMs => "head_settle_ms",
            ProfileValue::Trim(Trim::Straight) => "trim.straight",
            ProfileValue::Trim(Trim::Bevel) => "trim.bevel",
            ProfileValue::Trim(Trim::BevelRound) => "trim.bevel_round",
        }
    }

    const ALL: [ProfileValue; 7] = [
        ProfileValue::Name,
        ProfileValue::ThicknessMm,
        ProfileValue::SpeedPct,
        ProfileValue::HeadSettleMs,
        ProfileValue::Trim(Trim::Straight),
        ProfileValue::Trim(Trim::Bevel),
        ProfileValue::Trim(Trim::BevelRound),
    ];

    /// What every profile must have.
    pub const REQUIRED: [ProfileValue; 4] = [
        ProfileValue::Name,
        ProfileValue::ThicknessMm,
        ProfileValue::SpeedPct,
        ProfileValue::HeadSettleMs,
    ];
}

impl Key {
    /// The machine's keys, in the order the overview lists them.
    pub fn machine() -> Vec<Key> {
        let mut out = Vec::new();
        for h in HEADS {
            out.push(Key::HeadOffset(h, Axis::X));
            out.push(Key::HeadOffset(h, Axis::Y));
        }
        out.push(Key::Scale(Axis::X));
        out.push(Key::Scale(Axis::Y));
        for head in HEADS {
            for value in BladeValue::ALL {
                out.push(Key::Blade { head, value });
            }
        }
        out
    }

    /// A profile's keys, in the order the overview lists them.
    pub fn profile(id: &str) -> Vec<Key> {
        ProfileValue::ALL
            .into_iter()
            .map(|value| Key::Profile {
                profile: id.to_owned(),
                value,
            })
            .collect()
    }

    /// The profile it belongs to, if any.
    pub fn profile_id(&self) -> Option<&str> {
        match self {
            Key::Profile { profile, .. } => Some(profile),
            _ => None,
        }
    }

    /// Whether its value is text (a profile's name) rather than a number.
    pub fn is_text(&self) -> bool {
        matches!(
            self,
            Key::Profile {
                value: ProfileValue::Name,
                ..
            }
        )
    }

    /// Its unit, as shown beside a value.
    pub fn unit(&self) -> &'static str {
        match self {
            Key::HeadOffset(..) => "mm",
            Key::Scale(_) => "steps/mm",
            Key::Blade { value, .. } => match value {
                BladeValue::Lean => "°",
                BladeValue::OverlapMs => "ms",
                _ => "mm",
            },
            Key::Profile { value, .. } => match value {
                ProfileValue::Name => "",
                ProfileValue::ThicknessMm => "mm",
                ProfileValue::SpeedPct => "%",
                ProfileValue::HeadSettleMs => "ms",
                ProfileValue::Trim(_) => "°",
            },
        }
    }

    /// In words, for a person: "Bevel head, X offset".
    pub fn label(&self) -> String {
        let head = |h: Head| match h {
            Head::Bevel => "Bevel head",
            Head::Straight => "Straight head",
        };
        match self {
            Key::HeadOffset(h, a) => format!(
                "{}, {} offset",
                head(*h),
                if *a == Axis::X { "X" } else { "Y" }
            ),
            Key::Scale(a) => format!("{} scale", if *a == Axis::X { "X" } else { "Y" }),
            Key::Blade { head: h, value, .. } => {
                let what = match value {
                    BladeValue::Sideways => "slit's middle left of the line, on the back",
                    BladeValue::Width => "slit width",
                    BladeValue::Lean => "lean from vertical",
                    BladeValue::Lead(Surface::Back) => "start lead, on the back",
                    BladeValue::Lead(Surface::Face) => "start lead, on the face",
                    BladeValue::Shortfall(Surface::Back) => "end shortfall, on the back",
                    BladeValue::Shortfall(Surface::Face) => "end shortfall, on the face",
                    BladeValue::OverlapMs => "overlap on closed curves",
                    BladeValue::RunOutMm => "run-out on closed curves",
                };
                format!("{}, {what}", head(*h))
            }
            Key::Profile { value, .. } => match value {
                ProfileValue::Name => "Name".into(),
                ProfileValue::ThicknessMm => "Thickness".into(),
                ProfileValue::SpeedPct => "Cutting speed".into(),
                ProfileValue::HeadSettleMs => "Head settle".into(),
                ProfileValue::Trim(Trim::Straight) => "Straight head trim".into(),
                ProfileValue::Trim(Trim::Bevel) => "Bevel trim, straight edges".into(),
                ProfileValue::Trim(Trim::BevelRound) => "Bevel trim, closed curves".into(),
            },
        }
    }

    /// Whether `v` can be this key's value at all: the right kind, finite, within bounds.
    pub fn check(&self, v: &Val) -> Result<(), Error> {
        let bad = |why: &str| Err(Error::File(format!("{self}: {v} {why}")));
        if self.is_text() {
            return match v.text() {
                Some(t) if !t.trim().is_empty() => Ok(()),
                Some(_) => bad("is empty"),
                None => bad("isn't text"),
            };
        }
        let Some(x) = v.num() else {
            return bad("isn't a number");
        };
        if !x.is_finite() {
            return bad("isn't a number");
        }
        let (lo, hi) = match self {
            Key::HeadOffset(..) => (-2000.0, 2000.0),
            Key::Scale(_) => (1.0, 1000.0),
            Key::Blade { value, .. } => match value {
                BladeValue::Sideways => (-5.0, 5.0),
                BladeValue::Width => (-1.0, 5.0),
                BladeValue::Lean => (-30.0, 75.0),
                BladeValue::Lead(_) | BladeValue::Shortfall(_) => (-30.0, 30.0),
                BladeValue::OverlapMs => (0.0, 10_000.0),
                BladeValue::RunOutMm => (0.0, 5.0),
            },
            Key::Profile { value, .. } => match value {
                ProfileValue::Name => unreachable!("text, above"),
                ProfileValue::ThicknessMm => (0.01, 20.0),
                ProfileValue::SpeedPct => (1.0, 100.0),
                ProfileValue::HeadSettleMs => (0.0, 10_000.0),
                ProfileValue::Trim(_) => (-45.0, 45.0),
            },
        };
        if !(lo..=hi).contains(&x) {
            return bad(&format!("is outside {lo} to {hi} {}", self.unit()));
        }
        if matches!(
            self,
            Key::Profile {
                value: ProfileValue::HeadSettleMs,
                ..
            }
        ) && x.fract() != 0.0
        {
            return bad("isn't a whole number of ms");
        }
        Ok(())
    }
}

impl fmt::Display for Key {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let axis = |a: &Axis| if *a == Axis::X { "x" } else { "y" };
        match self {
            Key::HeadOffset(h, a) => write!(f, "machine.{}.offset.{}", head_word(*h), axis(a)),
            Key::Scale(a) => write!(f, "machine.scale.{}", axis(a)),
            Key::Blade { head, value } => {
                write!(f, "blade.{}.{}", head_word(*head), value.path())
            }
            Key::Profile { profile, value } => write!(f, "profile.{profile}.{}", value.path()),
        }
    }
}

impl FromStr for Key {
    type Err = Error;

    fn from_str(s: &str) -> Result<Key, Error> {
        let bad = || Error::File(format!("{s:?} isn't a calibration value"));
        let parts: Vec<&str> = s.split('.').collect();
        let axis = |a: &str| match a {
            "x" => Some(Axis::X),
            "y" => Some(Axis::Y),
            _ => None,
        };
        let blade = |rest: &[&str]| {
            let path = rest.join(".");
            BladeValue::ALL.into_iter().find(|v| v.path() == path)
        };
        let key = match parts.as_slice() {
            ["machine", "scale", a] => Key::Scale(axis(a).ok_or_else(bad)?),
            ["machine", h, "offset", a] => {
                Key::HeadOffset(head_of(h).ok_or_else(bad)?, axis(a).ok_or_else(bad)?)
            }
            ["blade", h, rest @ ..] => Key::Blade {
                head: head_of(h).ok_or_else(bad)?,
                value: blade(rest).ok_or_else(bad)?,
            },
            ["profile", id, rest @ ..] if valid_profile_id(id) => {
                let path = rest.join(".");
                let value = ProfileValue::ALL
                    .into_iter()
                    .find(|v| v.path() == path)
                    .ok_or_else(bad)?;
                Key::Profile {
                    profile: (*id).to_owned(),
                    value,
                }
            }
            _ => return Err(bad()),
        };
        Ok(key)
    }
}

impl TryFrom<String> for Key {
    type Error = Error;
    fn try_from(s: String) -> Result<Key, Error> {
        s.parse()
    }
}

impl From<Key> for String {
    fn from(k: Key) -> String {
        k.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_key_reads_back_as_written() {
        let mut keys = Key::machine();
        keys.extend(Key::profile("C2"));
        assert_eq!(keys.len(), 6 + 18 + 7);
        for k in keys {
            let text = k.to_string();
            assert_eq!(text.parse::<Key>().expect(&text), k, "{text}");
        }
    }

    #[test]
    fn keys_read_as_documented() {
        let k: Key = "machine.bevel.offset.x".parse().expect("a key");
        assert_eq!(k, Key::HeadOffset(Head::Bevel, Axis::X));
        let k: Key = "blade.bevel.shortfall.face".parse().expect("a key");
        assert_eq!(
            k,
            Key::Blade {
                head: Head::Bevel,
                value: BladeValue::Shortfall(Surface::Face)
            }
        );
        assert_eq!(
            (k.unit(), k.label().as_str()),
            ("mm", "Bevel head, end shortfall, on the face")
        );
        let k: Key = "profile.C16.trim.bevel_round".parse().expect("a key");
        assert_eq!(k.label(), "Bevel trim, closed curves");
    }

    #[test]
    fn what_isnt_a_key_is_refused() {
        for s in [
            "",
            "machine",
            "machine.bevel.offset.z",
            "machine.vgroove.offset.x",
            "blade.bevel.edge.left",
            "blade.bevel.lead",
            "profile.C 2.name",
            "profile.C2.colour",
            "profile.C2.bevel.run_on.end",
            "profile..name",
        ] {
            assert!(s.parse::<Key>().is_err(), "{s}");
        }
    }

    #[test]
    fn values_keep_to_their_kind_and_bounds() {
        let speed: Key = "profile.C2.speed_pct".parse().expect("a key");
        assert!(speed.check(&Val::Num(95.0)).is_ok());
        assert!(speed.check(&Val::Num(0.0)).is_err());
        assert!(speed.check(&Val::Text("fast".into())).is_err());
        let settle: Key = "profile.C2.head_settle_ms".parse().expect("a key");
        assert!(settle.check(&Val::Num(250.5)).is_err());
        let name: Key = "profile.C2.name".parse().expect("a key");
        assert!(name.check(&Val::Text("Mount board 1.4".into())).is_ok());
        assert!(name.check(&Val::Text("  ".into())).is_err());
        assert!(name.check(&Val::Num(1.0)).is_err());
        let x = Key::HeadOffset(Head::Bevel, Axis::X);
        assert!(x.check(&Val::Num(f64::NAN)).is_err());
    }
}
