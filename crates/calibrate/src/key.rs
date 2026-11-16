//! What can be calibrated: each value's key, in the file and in the history as a dotted path
//! (`machine.bevel.offset.x`, `blade.bevel.lean`), and all there is to say of it in one table:
//! its unit, its words for a person, the bounds a value must keep to be one at all, and how far
//! it plausibly moves between calibrations. The bounds catch a typing slip or a corrupt file;
//! a move past the plausible is a fit's to question ([`crate::Verdict::Large`]).

use std::fmt;
use std::str::FromStr;

use program::Head;
use serde::{Deserialize, Serialize};

use crate::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    X,
    Y,
}

impl Axis {
    pub const BOTH: [Axis; 2] = [Axis::X, Axis::Y];

    /// Its place in a point: 0 for X, 1 for Y.
    pub fn index(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
        }
    }

    /// As a key writes it: `x`, `y`.
    fn word(self) -> &'static str {
        match self {
            Axis::X => "x",
            Axis::Y => "y",
        }
    }
}

/// A side of the board: the back, up, where the blade enters; the face, the customer's, down.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
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

/// One value that can be calibrated. Ordered as the file and the overview list them: the
/// heads' offsets, the blade, then each profile. The scale isn't one: X and Y each move alone
/// on their own rails, at GMC's 40 steps/mm, which its captures match (CALIBRATION.md §5).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub enum Key {
    /// Where a head's blade is from the carriage (`machine::Head::offset_mm`).
    HeadOffset(Head, Axis),
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

/// A profile id: letters, digits, `_` and `-`, so it can sit in a dotted key.
pub fn valid_profile_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 32
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

/// All there is to say of a value: its path in its key, its unit, its words, the bounds a value
/// must keep to be one at all, and how far it plausibly moves between calibrations.
#[derive(Debug, Clone, Copy)]
struct About {
    path: &'static str,
    unit: &'static str,
    words: &'static str,
    bounds: [f64; 2],
    plausible: f64,
}

const fn about(
    path: &'static str,
    unit: &'static str,
    words: &'static str,
    bounds: [f64; 2],
    plausible: f64,
) -> About {
    About {
        path,
        unit,
        words,
        bounds,
        plausible,
    }
}

/// A profile's settings aren't fitted, so nothing they move to is implausible.
const TYPED: f64 = f64::INFINITY;

impl BladeValue {
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

    #[rustfmt::skip]
    fn about(self) -> About {
        use BladeValue::*;
        use Surface::{Back, Face};
        match self {
            Sideways        => about("sideways",       "mm", "slit's middle left of the line, on the back", [-5.0, 5.0], 0.5),
            Width           => about("width",          "mm", "slit width",                  [-1.0, 5.0], 0.5),
            Lean            => about("lean",           "°",  "lean from vertical",          [-30.0, 75.0], 5.0),
            Lead(Back)      => about("lead.back",      "mm", "start lead, on the back",     [-30.0, 30.0], 1.5),
            Lead(Face)      => about("lead.face",      "mm", "start lead, on the face",     [-30.0, 30.0], 1.5),
            Shortfall(Back) => about("shortfall.back", "mm", "end shortfall, on the back",  [-30.0, 30.0], 1.5),
            Shortfall(Face) => about("shortfall.face", "mm", "end shortfall, on the face",  [-30.0, 30.0], 1.5),
            OverlapMs       => about("overlap_ms",     "ms", "overlap on closed curves",    [0.0, 10_000.0], 500.0),
            RunOutMm        => about("run_out_mm",     "mm", "run-out on closed curves",    [0.0, 5.0], 0.5),
        }
    }
}

impl ProfileValue {
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

    #[rustfmt::skip]
    fn about(self) -> About {
        use ProfileValue::{HeadSettleMs, Name, SpeedPct, ThicknessMm};
        match self {
            Name                   => about("name",             "",   "Name",                       [0.0, 0.0], TYPED),
            ThicknessMm            => about("thickness_mm",     "mm", "Thickness",                  [0.01, 20.0], TYPED),
            SpeedPct               => about("speed_pct",        "%",  "Cutting speed",              [1.0, 100.0], TYPED),
            HeadSettleMs           => about("head_settle_ms",   "ms", "Head settle",                [0.0, 10_000.0], TYPED),
            Self::Trim(Trim::Straight)   => about("trim.straight",    "°",  "Straight head trim",         [-45.0, 45.0], TYPED),
            Self::Trim(Trim::Bevel)      => about("trim.bevel",       "°",  "Bevel trim, straight edges", [-45.0, 45.0], TYPED),
            Self::Trim(Trim::BevelRound) => about("trim.bevel_round", "°",  "Bevel trim, closed curves",  [-45.0, 45.0], TYPED),
        }
    }
}

impl Key {
    /// The machine's keys, in the order the overview lists them.
    pub fn machine() -> Vec<Key> {
        let mut out = Vec::new();
        for h in Head::ALL {
            out.extend(Axis::BOTH.map(|a| Key::HeadOffset(h, a)));
        }
        for head in Head::ALL {
            out.extend(BladeValue::ALL.map(|value| Key::Blade { head, value }));
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

    fn about(&self) -> About {
        match self {
            Key::HeadOffset(..) => about("offset", "mm", "offset", [-2000.0, 2000.0], 1.0),
            Key::Blade { value, .. } => value.about(),
            Key::Profile { value, .. } => value.about(),
        }
    }

    /// Its unit, as shown beside a value.
    pub fn unit(&self) -> &'static str {
        self.about().unit
    }

    /// How far it plausibly moves between calibrations: beyond it, a change is more likely a
    /// mistake in loading or reading than a fact about the machine.
    pub fn plausible(&self) -> f64 {
        self.about().plausible
    }

    /// What a fitted value is written to: a hundredth of a degree for the lean, else a
    /// micrometre.
    pub fn step(&self) -> f64 {
        match self {
            Key::Blade {
                value: BladeValue::Lean,
                ..
            } => 0.01,
            _ => 0.001,
        }
    }

    /// In words, for a person: "Bevel head, X offset".
    pub fn label(&self) -> String {
        let head = |h: &Head| match h {
            Head::Bevel => "Bevel head",
            Head::Straight => "Straight head",
        };
        let words = self.about().words;
        match self {
            Key::HeadOffset(h, a) => format!("{}, {} {words}", head(h), a.word().to_uppercase()),
            Key::Blade { head: h, .. } => format!("{}, {words}", head(h)),
            Key::Profile { .. } => words.to_owned(),
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
        let Some(x) = v.num().filter(|x| x.is_finite()) else {
            return bad("isn't a number");
        };
        let [lo, hi] = self.about().bounds;
        if !(lo..=hi).contains(&x) {
            return bad(&format!("is outside {lo} to {hi} {}", self.unit()));
        }
        let whole_ms = matches!(
            self,
            Key::Profile {
                value: ProfileValue::HeadSettleMs,
                ..
            }
        );
        if whole_ms && x.fract() != 0.0 {
            return bad("isn't a whole number of ms");
        }
        Ok(())
    }
}

impl fmt::Display for Key {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Key::HeadOffset(h, a) => write!(f, "machine.{h}.offset.{}", a.word()),
            Key::Blade { head, value } => write!(f, "blade.{head}.{}", value.about().path),
            Key::Profile { profile, value } => {
                write!(f, "profile.{profile}.{}", value.about().path)
            }
        }
    }
}

impl FromStr for Key {
    type Err = Error;

    fn from_str(s: &str) -> Result<Key, Error> {
        let bad = || Error::File(format!("{s:?} isn't a calibration value"));
        let parts: Vec<&str> = s.split('.').collect();
        let axis = |a: &str| Axis::BOTH.into_iter().find(|x| x.word() == a);
        let head = |h: &str| h.parse::<Head>().ok();
        let key = match parts.as_slice() {
            ["machine", h, "offset", a] => {
                Key::HeadOffset(head(h).ok_or_else(bad)?, axis(a).ok_or_else(bad)?)
            }
            ["blade", h, rest @ ..] => {
                let path = rest.join(".");
                Key::Blade {
                    head: head(h).ok_or_else(bad)?,
                    value: BladeValue::ALL
                        .into_iter()
                        .find(|v| v.about().path == path)
                        .ok_or_else(bad)?,
                }
            }
            ["profile", id, rest @ ..] if valid_profile_id(id) => {
                let path = rest.join(".");
                Key::Profile {
                    profile: (*id).to_owned(),
                    value: ProfileValue::ALL
                        .into_iter()
                        .find(|v| v.about().path == path)
                        .ok_or_else(bad)?,
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
        assert_eq!(keys.len(), 4 + 18 + 7);
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
