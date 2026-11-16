//! The calibration file (CALIBRATION.md §4): one per machine, shared by `mc601` and apps. It
//! holds the calibrated values only, each under its key ([`Key`]), over the driver's defaults:
//! the shop's 601 as fitted (`machine::Profile::gunnar_601`, `compiler::Blade::fitted`). It
//! also holds the board profiles, and the history of every change, append-only, so any
//! revision can be gone back to.
//!
//! Everything that cuts or predicts a cut asks the file, never the defaults:
//! [`Calibration::machine`] for the machine profile, [`Calibration::blade`] for how each head
//! cuts (on any board: the compiler works out the rest from the board's thickness), and
//! [`Calibration::material`] and [`Calibration::thickness`] for a profile's board.

use std::collections::BTreeMap;

use compiler::{Blade, HeadCut};
use machine::Profile;
use program::{BladeOffsets, Head, Material};
use serde::{Deserialize, Serialize};

use crate::Error;
use crate::key::{Axis, BladeValue, Key, ProfileValue, Surface, Trim, Val};

pub const FORMAT: &str = "601rs-calibration";
pub const VERSION: u32 = 1;

/// The machine's calibration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    format: String,
    version: u32,
    /// One more with every change; 0 is the defaults, never changed.
    revision: u32,
    /// The calibrated values, by key. Absent: the default.
    #[serde(default)]
    values: BTreeMap<String, Val>,
    #[serde(default)]
    history: Vec<Entry>,
}

/// One revision: what changed, when, and why.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Entry {
    pub revision: u32,
    /// When, as the caller gave it (RFC 3339).
    pub at: String,
    /// The routine whose fit it was; none for a value typed in.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub routine: Option<String>,
    /// The session it came from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub session: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    pub changes: Vec<Change>,
}

/// One value's change, as the file held it: none is the default.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Change {
    pub key: Key,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub before: Option<Val>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub after: Option<Val>,
    /// The fitted value's standard error.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub se: Option<f64>,
}

/// A change asked for: a value for `key`, or none to go back to the default.
#[derive(Debug, Clone, PartialEq)]
pub struct Edit {
    pub key: Key,
    pub value: Option<Val>,
    pub se: Option<f64>,
}

impl Edit {
    pub fn set(key: Key, value: impl Into<Val>) -> Edit {
        Edit {
            key,
            value: Some(value.into()),
            se: None,
        }
    }

    pub fn unset(key: Key) -> Edit {
        Edit {
            key,
            value: None,
            se: None,
        }
    }

    pub fn with_se(self, se: f64) -> Edit {
        Edit {
            se: Some(se),
            ..self
        }
    }
}

impl From<f64> for Val {
    fn from(v: f64) -> Val {
        Val::Num(v)
    }
}

impl From<&str> for Val {
    fn from(v: &str) -> Val {
        Val::Text(v.to_owned())
    }
}

impl From<String> for Val {
    fn from(v: String) -> Val {
        Val::Text(v)
    }
}

/// Why a change is made: the history entry's when, routine, session and note.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Why {
    pub at: String,
    pub routine: Option<String>,
    pub session: Option<String>,
    pub note: Option<String>,
}

impl Why {
    /// A value typed in, at `at`.
    pub fn typed(at: impl Into<String>) -> Why {
        Why {
            at: at.into(),
            ..Why::default()
        }
    }
}

/// A value as the overview shows it.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ValueState {
    pub key: Key,
    pub label: String,
    pub unit: &'static str,
    /// The value in effect.
    pub value: Val,
    /// Whether the file sets it; if not, it is the default.
    pub set: bool,
    /// Its last change, if it has one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last: Option<Last>,
}

/// When a value last changed, and how.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Last {
    pub revision: u32,
    pub at: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub routine: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub se: Option<f64>,
}

/// Where a blade value lives in a head's tool model.
pub(crate) fn blade_value(c: &mut HeadCut, value: BladeValue) -> &mut f64 {
    let side = |s: Surface| usize::from(s == Surface::Face);
    match value {
        BladeValue::Sideways => &mut c.sideways_mm,
        BladeValue::Width => &mut c.width_mm,
        BladeValue::Lean => &mut c.lean_deg,
        BladeValue::Lead(s) => &mut c.lead_mm[side(s)],
        BladeValue::Shortfall(s) => &mut c.shortfall_mm[side(s)],
        BladeValue::OverlapMs => &mut c.overlap_ms,
        BladeValue::RunOutMm => &mut c.run_out_mm,
    }
}

/// The driver's default for a blade value: the shop's 601 as fitted.
fn blade_default(head: Head, value: BladeValue) -> f64 {
    *blade_value(Blade::fitted().head_mut(head), value)
}

impl Default for Calibration {
    fn default() -> Self {
        Calibration::new()
    }
}

impl Calibration {
    /// A machine never calibrated: every value the driver's default.
    pub fn new() -> Self {
        Calibration {
            format: FORMAT.to_owned(),
            version: VERSION,
            revision: 0,
            values: BTreeMap::new(),
            history: Vec::new(),
        }
    }

    /// # Errors
    /// When it isn't a calibration file of this version, or a value isn't one.
    pub fn from_json(text: &str) -> Result<Self, Error> {
        let c: Calibration = serde_json::from_str(text).map_err(|e| Error::File(e.to_string()))?;
        c.validate()?;
        Ok(c)
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_default() + "\n"
    }

    pub fn revision(&self) -> u32 {
        self.revision
    }

    /// Every change, oldest first.
    pub fn history(&self) -> &[Entry] {
        &self.history
    }

    fn validate(&self) -> Result<(), Error> {
        if self.format != FORMAT {
            return Err(Error::File(format!(
                "not a calibration file: its format is {:?}",
                self.format
            )));
        }
        if self.version != VERSION {
            return Err(Error::File(format!(
                "calibration version {} isn't one this driver reads (it reads {VERSION})",
                self.version
            )));
        }
        for (k, v) in &self.values {
            let key: Key = k.parse()?;
            if key.to_string() != *k {
                return Err(Error::File(format!("{k}: write it as {key}")));
            }
            key.check(v)?;
        }
        self.check_profiles()?;
        let mut last = 0;
        for e in &self.history {
            if e.revision <= last {
                return Err(Error::File(format!(
                    "history: revision {} comes after {last}",
                    e.revision
                )));
            }
            last = e.revision;
        }
        if last > self.revision || (!self.history.is_empty() && last != self.revision) {
            return Err(Error::File(format!(
                "the file is at revision {}, its history at {last}",
                self.revision
            )));
        }
        Ok(())
    }

    /// Every profile has a name, thickness, speed and settle.
    fn check_profiles(&self) -> Result<(), Error> {
        for id in self.profile_ids() {
            let missing: Vec<String> = ProfileValue::REQUIRED
                .into_iter()
                .map(|value| Key::Profile {
                    profile: id.clone(),
                    value,
                })
                .filter(|k| !self.values.contains_key(&k.to_string()))
                .map(|k| k.to_string())
                .collect();
            if !missing.is_empty() {
                return Err(Error::File(format!(
                    "profile {id} needs a name, thickness, speed and settle: missing {}",
                    missing.join(", ")
                )));
            }
        }
        Ok(())
    }

    /// The ids of every profile the file mentions.
    fn profile_ids(&self) -> Vec<String> {
        let mut ids: Vec<String> = self
            .values
            .keys()
            .filter_map(|k| k.parse::<Key>().ok())
            .filter_map(|k| k.profile_id().map(str::to_owned))
            .collect();
        ids.sort();
        ids.dedup();
        ids
    }

    /// The board profiles: id and name, by id.
    pub fn profiles(&self) -> Vec<(String, String)> {
        self.profile_ids()
            .into_iter()
            .map(|id| {
                let name = self
                    .raw(&Key::Profile {
                        profile: id.clone(),
                        value: ProfileValue::Name,
                    })
                    .and_then(Val::text)
                    .unwrap_or_default()
                    .to_owned();
                (id, name)
            })
            .collect()
    }

    fn has_profile(&self, id: &str) -> bool {
        self.values.contains_key(
            &Key::Profile {
                profile: id.to_owned(),
                value: ProfileValue::Name,
            }
            .to_string(),
        )
    }

    fn need_profile(&self, id: &str) -> Result<(), Error> {
        if self.has_profile(id) {
            Ok(())
        } else {
            Err(Error::File(format!("there's no profile {id}")))
        }
    }

    /// The value as the file holds it; none: the default.
    pub fn raw(&self, key: &Key) -> Option<&Val> {
        self.values.get(&key.to_string())
    }

    /// Whether the file sets `key`.
    pub fn is_set(&self, key: &Key) -> bool {
        self.raw(key).is_some()
    }

    /// The value in effect: the file's, else the driver's default.
    ///
    /// # Errors
    /// For a profile that doesn't exist.
    pub fn get(&self, key: &Key) -> Result<Val, Error> {
        if let Some(v) = self.raw(key) {
            return Ok(v.clone());
        }
        let n = |v: f64| Ok(Val::Num(v));
        match key {
            Key::HeadOffset(h, a) => n(Profile::gunnar_601().head(*h).offset_mm[a.index()]),
            Key::Scale(Axis::X) => n(Profile::gunnar_601().scale.x_steps_per_mm),
            Key::Scale(Axis::Y) => n(Profile::gunnar_601().scale.y_steps_per_mm),
            Key::Blade { head, value } => n(blade_default(*head, *value)),
            Key::Profile { profile, value } => {
                self.need_profile(profile)?;
                match value {
                    ProfileValue::Trim(Trim::Straight | Trim::Bevel) => n(0.0),
                    ProfileValue::Trim(Trim::BevelRound) => self.get(&Key::Profile {
                        profile: profile.clone(),
                        value: ProfileValue::Trim(Trim::Bevel),
                    }),
                    // Required, so set if the profile exists.
                    _ => Err(Error::File(format!("{key} isn't set"))),
                }
            }
        }
    }

    /// A number in effect.
    ///
    /// # Errors
    /// For a profile that doesn't exist, or a key whose value is text.
    pub fn num(&self, key: &Key) -> Result<f64, Error> {
        self.get(key)?
            .num()
            .ok_or_else(|| Error::File(format!("{key} isn't a number")))
    }

    /// The machine profile: `Profile::gunnar_601()` with the calibrated head offsets and scale.
    pub fn machine(&self) -> Profile {
        let mut p = Profile::gunnar_601();
        let num = |k: Key| self.num(&k).unwrap_or_default();
        for h in Head::ALL {
            let offset = [
                num(Key::HeadOffset(h, Axis::X)),
                num(Key::HeadOffset(h, Axis::Y)),
            ];
            match h {
                Head::Bevel => p.bevel.offset_mm = offset,
                Head::Straight => p.straight.offset_mm = offset,
            }
        }
        p.scale.x_steps_per_mm = num(Key::Scale(Axis::X));
        p.scale.y_steps_per_mm = num(Key::Scale(Axis::Y));
        p
    }

    /// A profile's board thickness, mm.
    ///
    /// # Errors
    /// For a profile that doesn't exist.
    pub fn thickness(&self, id: &str) -> Result<f64, Error> {
        self.num(&Key::Profile {
            profile: id.to_owned(),
            value: ProfileValue::ThicknessMm,
        })
    }

    /// How each head cuts, the tool model: the compiler places lines and run-ons from it and
    /// the board's thickness.
    pub fn blade(&self) -> Blade {
        let mut b = Blade::fitted();
        for head in Head::ALL {
            for value in BladeValue::ALL {
                if let Some(v) = self.raw(&Key::Blade { head, value }).and_then(Val::num) {
                    *blade_value(b.head_mut(head), value) = v;
                }
            }
        }
        b
    }

    /// The program `material` for profile `id`: its name, speed, settle and trims.
    ///
    /// # Errors
    /// For a profile that doesn't exist.
    pub fn material(&self, id: &str) -> Result<Material, Error> {
        let k = |value| Key::Profile {
            profile: id.to_owned(),
            value,
        };
        self.need_profile(id)?;
        Ok(Material {
            name: self
                .get(&k(ProfileValue::Name))?
                .text()
                .unwrap_or_default()
                .to_owned(),
            speed_pct: self.num(&k(ProfileValue::SpeedPct))?,
            head_settle_ms: self.num(&k(ProfileValue::HeadSettleMs))? as u32,
            blade_angle_offset_deg: BladeOffsets {
                straight: self.num(&k(ProfileValue::Trim(Trim::Straight)))?,
                bevel: self.num(&k(ProfileValue::Trim(Trim::Bevel)))?,
                bevel_round: self
                    .raw(&k(ProfileValue::Trim(Trim::BevelRound)))
                    .and_then(Val::num),
            },
        })
    }

    /// `key`'s last change.
    pub fn last(&self, key: &Key) -> Option<Last> {
        self.history.iter().rev().find_map(|e| {
            e.changes
                .iter()
                .rev()
                .find(|c| c.key == *key)
                .map(|c| Last {
                    revision: e.revision,
                    at: e.at.clone(),
                    routine: e.routine.clone(),
                    se: c.se,
                })
        })
    }

    /// Every value, the machine's and then each profile's, as the overview shows them.
    pub fn values(&self) -> Vec<ValueState> {
        let mut keys = Key::machine();
        for (id, _) in self.profiles() {
            keys.extend(Key::profile(&id));
        }
        keys.into_iter()
            .filter_map(|key| {
                let value = self.get(&key).ok()?;
                Some(ValueState {
                    label: key.label(),
                    unit: key.unit(),
                    set: self.is_set(&key),
                    last: self.last(&key),
                    value,
                    key,
                })
            })
            .collect()
    }

    /// The edits made, as a new revision. Edits that change nothing are dropped; if none is
    /// left, the calibration comes back as it was, at the same revision.
    ///
    /// # Errors
    /// When a value isn't one for its key, or a profile would lack its name, thickness, speed
    /// or settle.
    pub fn set(&self, edits: &[Edit], why: &Why) -> Result<Calibration, Error> {
        let mut next = self.clone();
        let mut changes: Vec<Change> = Vec::new();
        for e in edits {
            if let Some(v) = &e.value {
                e.key.check(v)?;
            }
            let k = e.key.to_string();
            let before = next.values.get(&k).cloned();
            match &e.value {
                Some(v) => next.values.insert(k, v.clone()),
                None => next.values.remove(&k),
            };
            match changes.iter_mut().find(|c| c.key == e.key) {
                Some(c) => {
                    c.after = e.value.clone();
                    c.se = e.se;
                }
                None => changes.push(Change {
                    key: e.key.clone(),
                    before,
                    after: e.value.clone(),
                    se: e.se,
                }),
            }
        }
        changes.retain(|c| c.before != c.after);
        if changes.is_empty() {
            return Ok(self.clone());
        }
        next.check_profiles()?;
        next.revision += 1;
        next.history.push(Entry {
            revision: next.revision,
            at: why.at.clone(),
            routine: why.routine.clone(),
            session: why.session.clone(),
            note: why.note.clone(),
            changes,
        });
        Ok(next)
    }

    /// Profile `id` gone, all its values with it, as a new revision.
    ///
    /// # Errors
    /// When there's no such profile.
    pub fn remove_profile(&self, id: &str, why: &Why) -> Result<Calibration, Error> {
        self.need_profile(id)?;
        let edits: Vec<Edit> = Key::profile(id)
            .into_iter()
            .filter(|k| self.is_set(k))
            .map(Edit::unset)
            .collect();
        self.set(&edits, why)
    }

    /// Every value as it was at revision `to`, as a new revision (so a revert can be reverted).
    ///
    /// # Errors
    /// When `to` is later than this revision, or the history doesn't reach back to it.
    pub fn revert(&self, to: u32, why: &Why) -> Result<Calibration, Error> {
        if to > self.revision {
            return Err(Error::File(format!(
                "there's no revision {to}: the calibration is at {}",
                self.revision
            )));
        }
        let later: Vec<&Entry> = self.history.iter().filter(|e| e.revision > to).collect();
        if later.len() as u32 != self.revision - to {
            return Err(Error::File(format!(
                "the history doesn't reach back to revision {to}"
            )));
        }
        let mut then = self.values.clone();
        for e in later.iter().rev() {
            for c in e.changes.iter().rev() {
                let k = c.key.to_string();
                match &c.before {
                    Some(v) => then.insert(k, v.clone()),
                    None => then.remove(&k),
                };
            }
        }
        let mut edits = Vec::new();
        for k in self.values.keys().chain(then.keys()) {
            let (now, was) = (self.values.get(k), then.get(k));
            if now != was && !edits.iter().any(|e: &Edit| e.key.to_string() == *k) {
                edits.push(Edit {
                    key: k.parse()?,
                    value: was.cloned(),
                    se: None,
                });
            }
        }
        let why = Why {
            note: why
                .note
                .clone()
                .or_else(|| Some(format!("back to revision {to}"))),
            ..why.clone()
        };
        self.set(&edits, &why)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(n: u32) -> Why {
        Why::typed(format!("2026-11-15T10:{n:02}:00Z"))
    }

    fn key(s: &str) -> Key {
        s.parse().expect(s)
    }

    /// C2 as a board list might have it, with GMC's C2 settings.
    fn with_c2() -> Calibration {
        Calibration::new()
            .set(
                &[
                    Edit::set(key("profile.C2.name"), "Mount board 1.4"),
                    Edit::set(key("profile.C2.thickness_mm"), 1.4),
                    Edit::set(key("profile.C2.speed_pct"), 95.0),
                    Edit::set(key("profile.C2.head_settle_ms"), 200.0),
                    Edit::set(key("profile.C2.trim.bevel"), 1.0),
                ],
                &at(1),
            )
            .expect("a profile")
    }

    #[test]
    fn a_machine_never_calibrated_is_the_shops_601_as_fitted() {
        let c = Calibration::new();
        assert_eq!(c.machine(), Profile::gunnar_601());
        assert_eq!(c.revision(), 0);
        assert!(c.profiles().is_empty());
        assert_eq!(c.blade(), Blade::fitted());
        let back = Calibration::from_json(&c.to_json()).expect("reads back");
        assert_eq!(back, c);
    }

    #[test]
    fn a_head_offset_set_is_the_machines() {
        let c = Calibration::new()
            .set(
                &[Edit::set(key("machine.bevel.offset.x"), -71.462).with_se(0.03)],
                &Why {
                    routine: Some("heads".into()),
                    ..at(5)
                },
            )
            .expect("set");
        let p = c.machine();
        assert_eq!(
            p.bevel.offset_mm,
            [-71.462, Profile::gunnar_601().bevel.offset_mm[1]]
        );
        assert_eq!(p.straight, Profile::gunnar_601().straight);
        assert_eq!(c.revision(), 1);
        let last = c.last(&key("machine.bevel.offset.x")).expect("changed");
        assert_eq!(
            (last.revision, last.routine.as_deref(), last.se),
            (1, Some("heads"), Some(0.03))
        );
        assert_eq!(c.history()[0].changes[0].before, None);
    }

    #[test]
    fn a_profile_gives_its_material_and_thickness() {
        let c = with_c2();
        assert_eq!(
            c.profiles(),
            vec![("C2".to_owned(), "Mount board 1.4".to_owned())]
        );
        let m = c.material("C2").expect("a material");
        assert_eq!(
            (m.name.as_str(), m.speed_pct, m.head_settle_ms),
            ("Mount board 1.4", 95.0, 200)
        );
        assert_eq!(
            m.blade_angle_offset_deg,
            BladeOffsets {
                straight: 0.0,
                bevel: 1.0,
                bevel_round: None
            }
        );
        assert_eq!(c.thickness("C2").expect("a thickness"), 1.4);
        assert_eq!(
            c.num(&key("profile.C2.trim.bevel_round")).expect("a value"),
            1.0
        );
    }

    #[test]
    fn a_blade_value_set_is_the_heads_on_every_board() {
        let c = with_c2()
            .set(
                &[
                    Edit::set(key("blade.bevel.lean"), 45.0),
                    Edit::set(key("blade.bevel.shortfall.face"), 2.0),
                ],
                &at(2),
            )
            .expect("set");
        let b = c.blade();
        assert_eq!((b.bevel.lean_deg, b.bevel.shortfall_mm[1]), (45.0, 2.0));
        assert_eq!(b.bevel.sideways_mm, Blade::fitted().bevel.sideways_mm);
        assert_eq!(b.straight, Blade::fitted().straight);
        assert_eq!(c.num(&key("blade.bevel.lean")).expect("a value"), 45.0);
        assert_eq!(
            c.num(&key("blade.straight.sideways")).expect("a value"),
            -0.095
        );
    }

    #[test]
    fn a_profile_must_be_whole() {
        let half = Calibration::new().set(&[Edit::set(key("profile.C3.name"), "Std back")], &at(1));
        assert!(half.is_err());
        let gone = with_c2().set(&[Edit::unset(key("profile.C2.speed_pct"))], &at(2));
        assert!(gone.is_err());
        let removed = with_c2().remove_profile("C2", &at(2)).expect("removed");
        assert!(removed.profiles().is_empty());
        assert_eq!(removed.revision(), 2);
    }

    #[test]
    fn edits_that_change_nothing_make_no_revision() {
        let c = with_c2();
        let same = c
            .set(&[Edit::set(key("profile.C2.speed_pct"), 95.0)], &at(2))
            .expect("set");
        assert_eq!(same, c);
    }

    #[test]
    fn revert_goes_back_and_can_itself_be_reverted() {
        let c1 = with_c2();
        let c2 = c1
            .set(&[Edit::set(key("machine.straight.offset.x"), 6.5)], &at(2))
            .expect("set");
        let c3 = c2
            .set(
                &[
                    Edit::set(key("machine.straight.offset.x"), 6.6),
                    Edit::set(key("profile.C2.speed_pct"), 80.0),
                ],
                &at(3),
            )
            .expect("set");
        let back = c3.revert(1, &at(4)).expect("reverted");
        assert_eq!(back.revision(), 4);
        assert_eq!(back.values, c1.values);
        assert_eq!(
            back.history()[3].note.as_deref(),
            Some("back to revision 1")
        );
        let again = back.revert(3, &at(5)).expect("reverted");
        assert_eq!(again.values, c3.values);
        let none = again.revert(0, &at(6)).expect("reverted");
        assert_eq!(none.machine(), Profile::gunnar_601());
        assert!(none.profiles().is_empty());
        assert!(none.revert(9, &at(7)).is_err());
    }

    #[test]
    fn the_file_reads_back_as_written() {
        let c = with_c2()
            .set(
                &[Edit::set(key("machine.scale.x"), 40.02).with_se(0.004)],
                &at(2),
            )
            .expect("set");
        let text = c.to_json();
        assert!(text.contains("\"machine.scale.x\": 40.02"), "{text}");
        assert_eq!(Calibration::from_json(&text).expect("reads"), c);
    }

    #[test]
    fn a_file_that_isnt_right_is_refused() {
        let ok = with_c2().to_json();
        for (from, to) in [
            ("\"version\": 1", "\"version\": 2"),
            ("601rs-calibration", "something-else"),
            (
                "\"profile.C2.speed_pct\": 95.0",
                "\"profile.C2.speed_pct\": 195.0",
            ),
            (
                "\"profile.C2.speed_pct\": 95.0",
                "\"profile.C2.colour\": 95.0",
            ),
            ("\"profile.C2.speed_pct\": 95.0,", ""),
            ("\"revision\": 1", "\"revision\": 3"),
        ] {
            assert!(ok.contains(from), "{from}");
            let bad = ok.replacen(from, to, 1);
            assert!(Calibration::from_json(&bad).is_err(), "{to}");
        }
    }

    #[test]
    fn the_overview_lists_every_value() {
        let c = with_c2();
        let v = c.values();
        assert_eq!(v.len(), Key::machine().len() + Key::profile("C2").len());
        let speed = v
            .iter()
            .find(|s| s.key == key("profile.C2.speed_pct"))
            .expect("listed");
        assert!(speed.set);
        assert_eq!(speed.last.as_ref().map(|l| l.revision), Some(1));
        let x = v
            .iter()
            .find(|s| s.key == key("machine.bevel.offset.x"))
            .expect("listed");
        assert!(!x.set && x.last.is_none());
        assert_eq!(x.value, Val::Num(Profile::gunnar_601().bevel.offset_mm[0]));
    }
}
