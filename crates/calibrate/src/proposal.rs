//! What a fit proposes for the calibration file (CALIBRATION.md §7): each value before and
//! after, ± its standard error, and a verdict; and applying the chosen changes as one revision.

use serde::Serialize;

use crate::Error;
use crate::calibration::{Calibration, Edit, Why};
use crate::fit::Leftover;
use crate::key::{BladeValue, Key};
use crate::plan::Routine;

/// A reading this many of its own scatters off the model, judged by the rest, is more likely a
/// misread than scatter ([`Leftover::t`]).
pub const SUSPECT_T: f64 = 3.5;

/// What a fit proposes.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Proposal {
    pub routine: Routine,
    /// How many readings were fitted.
    pub readings: usize,
    /// How far a reading scatters about the model, mm (NaN: none to spare).
    pub scatter_mm: f64,
    pub changes: Vec<Change>,
    /// Readings worth taking again, worst first.
    pub suspects: Vec<Leftover>,
    /// What else the fit says, in words.
    pub notes: Vec<String>,
}

/// One value the fit proposes.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Change {
    pub key: Key,
    pub label: String,
    pub unit: &'static str,
    /// As the calibration has it.
    pub before: f64,
    /// As fitted; none if the readings don't fix it.
    pub after: Option<f64>,
    pub se: Option<f64>,
    pub verdict: Verdict,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    /// Within twice its standard error of the calibration's: kept unless chosen.
    WithinNoise,
    /// Moved: proposed.
    Moved,
    /// Further than the machine plausibly moves: more likely a loading or reading mistake, to
    /// confirm before applying.
    Large,
    /// The readings don't fix it.
    Undetermined,
}

/// How far a value plausibly moves between calibrations: beyond it, a change is
/// [`Verdict::Large`].
fn plausible(key: &Key) -> f64 {
    match key {
        Key::HeadOffset(..) => 1.0,
        Key::Scale(_) => 0.2,
        Key::Blade { value, .. } => match value {
            BladeValue::Sideways | BladeValue::Width => 0.5,
            BladeValue::Lean => 5.0,
            BladeValue::Lead(_) | BladeValue::Shortfall(_) => 1.5,
            BladeValue::OverlapMs => 500.0,
            BladeValue::RunOutMm => 0.5,
        },
        Key::Profile { .. } => f64::INFINITY,
    }
}

impl Change {
    /// The change of `key` from what `cal` has to `fitted` (value, standard error), rounded to
    /// what is worth writing: a micrometre, a hundredth of a degree.
    pub fn new(cal: &Calibration, key: Key, fitted: Option<(f64, f64)>) -> Change {
        let before = cal.num(&key).unwrap_or(f64::NAN);
        let step = if key.unit() == "°" { 0.01 } else { 0.001 };
        let after = fitted.map(|(v, _)| (v / step).round() * step);
        let se = fitted.map(|(_, se)| se).filter(|se| se.is_finite());
        let verdict = match after {
            None => Verdict::Undetermined,
            Some(a) if (a - before).abs() > plausible(&key) => Verdict::Large,
            Some(a) if se.is_some_and(|se| (a - before).abs() <= 2.0 * se) => Verdict::WithinNoise,
            Some(_) => Verdict::Moved,
        };
        Change {
            label: key.label(),
            unit: key.unit(),
            key,
            before,
            after,
            se,
            verdict,
        }
    }
}

impl Proposal {
    /// What to apply unless the operator says otherwise: the moved values.
    pub fn chosen(&self) -> Vec<Key> {
        self.changes
            .iter()
            .filter(|c| c.verdict == Verdict::Moved)
            .map(|c| c.key.clone())
            .collect()
    }

    /// The `chosen` changes as a new revision of `cal`, each with its standard error, the
    /// history naming the routine.
    ///
    /// # Errors
    /// When a chosen key isn't one this proposal fixes, or a value isn't one for its key.
    pub fn apply(
        &self,
        cal: &Calibration,
        chosen: &[Key],
        why: &Why,
    ) -> Result<Calibration, Error> {
        let mut edits = Vec::new();
        for key in chosen {
            let c = self
                .changes
                .iter()
                .find(|c| c.key == *key)
                .ok_or_else(|| Error::File(format!("{key} isn't in this proposal")))?;
            let after = c
                .after
                .ok_or_else(|| Error::File(format!("{key}: the readings don't fix it")))?;
            let edit = Edit::set(key.clone(), after);
            edits.push(match c.se {
                Some(se) => edit.with_se(se),
                None => edit,
            });
        }
        let why = Why {
            routine: Some(self.routine.name().to_owned()),
            ..why.clone()
        };
        cal.set(&edits, &why)
    }
}
