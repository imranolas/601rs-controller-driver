//! What a fit proposes for the calibration file (CALIBRATION.md §7): each value before and
//! after, ± its standard error, and a verdict; and applying the chosen changes as one revision.

use program::Head;
use serde::Serialize;

use crate::Error;
use crate::calibration::{Calibration, Edit, Why, blade_value};
use crate::fit::Leftover;
use crate::key::{Axis, BladeValue, Key};
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
    /// What applying it means for the cuts, on the board the routine was cut on, in words;
    /// empty when it means nothing worth saying.
    pub effect: String,
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
    /// what is worth writing: a micrometre, a hundredth of a degree. Rounded to that, a value
    /// within half a step of the file's is within the noise whatever its standard error, so
    /// readings that fit exactly (with no scatter, and so no error) don't call a value the file
    /// holds more finely moved.
    pub fn new(cal: &Calibration, key: Key, fitted: Option<(f64, f64)>) -> Change {
        let before = cal.num(&key).unwrap_or(f64::NAN);
        let step = if key.unit() == "°" { 0.01 } else { 0.001 };
        let after = fitted.map(|(v, _)| (v / step).round() * step);
        let se = fitted.map(|(_, se)| se).filter(|se| se.is_finite());
        let verdict = match after {
            None => Verdict::Undetermined,
            Some(a) if (a - before).abs() > plausible(&key) => Verdict::Large,
            Some(a) if (a - before).abs() <= step / 2.0 + 1e-9 => Verdict::WithinNoise,
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
            effect: String::new(),
        }
    }
}

/// What changing `key` from `before` to `after` means for the cuts on board `thickness_mm`
/// thick, in words: where they land now, by the file, and what applying it changes.
fn effect(cal: &Calibration, key: &Key, before: f64, after: f64, thickness_mm: f64) -> String {
    let d = after - before;
    if !d.is_finite() || d.abs() < 0.0005 {
        return String::new();
    }
    let t = thickness_mm;
    match *key {
        Key::HeadOffset(h, axis) => {
            let dir = match (axis, d > 0.0) {
                (Axis::X, true) => "right",
                (Axis::X, false) => "left",
                (Axis::Y, true) => "toward the back",
                (Axis::Y, false) => "toward the front",
            };
            format!(
                "By the file, the {h} head's cuts land {:.2} mm {dir} of where they should: this \
                 puts them back.",
                d.abs()
            )
        }
        Key::Blade {
            head: h,
            value: BladeValue::Sideways,
        } => {
            let side = if d > 0.0 { "left" } else { "right" };
            let windows = if h == Head::Bevel {
                format!(
                    ", so windows come out {:.2} mm {} on the back",
                    2.0 * d.abs(),
                    if d > 0.0 { "larger" } else { "smaller" }
                )
            } else {
                String::new()
            };
            format!(
                "By the file, every edge the {h} head cuts lands {:.2} mm {side} of where it should, \
                 as the cut travels{windows}: this puts them back.",
                d.abs()
            )
        }
        Key::Blade {
            head: h,
            value: BladeValue::Width,
        } => format!(
            "The {h} head's slit is {:.2} mm {} than the file has it, so its pieces come out that \
             much {} each way: this puts them back.",
            d.abs(),
            if d > 0.0 { "wider" } else { "narrower" },
            if d > 0.0 { "smaller" } else { "larger" }
        ),
        Key::Blade { head: h, value } => {
            // What the compiler would run on by at a square corner, before and after.
            let was = cal.blade();
            let mut now = was;
            *blade_value(now.head_mut(h), value) = after;
            let ([s0, e0], [s1, e1]) = (was.head(h).run_on(t), now.head(h).run_on(t));
            let corners = |what: &str, by: f64, more: &str, less: &str| {
                (by.abs() >= 0.0005).then(|| {
                    format!(
                        "{what} {:.2} mm {}",
                        by.abs(),
                        if by > 0.0 { more } else { less }
                    )
                })
            };
            let both: Vec<String> = [
                corners("start", s1 - s0, "sooner", "later"),
                corners("run on", e1 - e0, "further", "less far"),
            ]
            .into_iter()
            .flatten()
            .collect();
            match value {
                BladeValue::Lean => {
                    let run = |deg: f64| t * deg.to_radians().tan();
                    let mut s = format!(
                        "On {t} mm board the bevel runs {:.2} mm across, not {:.2}",
                        run(after),
                        run(before)
                    );
                    if !both.is_empty() {
                        s += &format!(": its cuts {} at each corner", both.join(" and "));
                    }
                    s + "."
                }
                BladeValue::Lead(_) | BladeValue::Shortfall(_) if both.is_empty() => format!(
                    "No change on {t} mm board: the other face decides how far the {h} head's cuts \
                     run."
                ),
                BladeValue::Lead(_) | BladeValue::Shortfall(_) => format!(
                    "On {t} mm board, the {h} head's cuts {} at each corner.",
                    both.join(" and ")
                ),
                _ => String::new(),
            }
        }
        _ => String::new(),
    }
}

impl Proposal {
    /// Each change's effect in words, on board `thickness_mm` thick, against `cal`.
    pub(crate) fn with_effects(mut self, cal: &Calibration, thickness_mm: f64) -> Proposal {
        for c in &mut self.changes {
            if let Some(after) = c.after {
                c.effect = effect(cal, &c.key, c.before, after, thickness_mm);
            }
        }
        self
    }

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
                .ok_or_else(|| Error::Reading(format!("{key} isn't in this proposal")))?;
            let after = c
                .after
                .ok_or_else(|| Error::Reading(format!("{key}: the readings don't fix it")))?;
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The bevel's sideways default is 0.4525: fitted exactly, with no scatter and so no
    /// standard error, it rounds to 0.453 and is within the noise, not moved (found by an app).
    #[test]
    fn a_value_held_finer_than_written_isnt_moved_by_rounding() {
        let cal = Calibration::new();
        let key: Key = "blade.bevel.sideways".parse().expect("a key");
        let exact = Change::new(&cal, key.clone(), Some((0.4525, 0.0)));
        assert_eq!(exact.verdict, Verdict::WithinNoise, "{exact:?}");
        let moved = Change::new(&cal, key.clone(), Some((0.4555, 0.0)));
        assert_eq!(moved.verdict, Verdict::Moved, "{moved:?}");
        let large = Change::new(&cal, key, Some((1.2, 0.01)));
        assert_eq!(large.verdict, Verdict::Large);
    }
}
