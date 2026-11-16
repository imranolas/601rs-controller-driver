//! A session (CALIBRATION.md §8): one run of a routine, kept so it can be resumed, read later,
//! and fitted again. It holds the plan as cut (with the machine profile it was placed with), the
//! calibration's revision then, how far the cut got, and the readings taken so far. It is plain
//! data, serialisable, so `mc601` keeps it in its session folder and an app as a file of its own.
//!
//! A cut that stopped keeps its session: a reading whose cuts all finished can still be taken
//! and fitted; one that needs a cut that never finished can't.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::plan::{Plan, Routine};
use crate::proposal::Proposal;
use crate::reading::{Check, Reading, combs};
use crate::{Calibration, Error, Sheet, heads, runons};

pub const FORMAT: &str = "601rs-calibration-session";
/// 2: each reading says what it measures (`Kind::Length::span`, `Kind::Tooth` a comb), and no
/// longer lists its cuts.
pub const VERSION: u32 = 2;

/// One run of a routine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Session {
    format: String,
    version: u32,
    /// Given by the caller: "2026-11-16 10.31.02 calibrate stage_1".
    pub id: String,
    /// The calibration's revision the plan was made with.
    pub revision: u32,
    pub plan: Plan,
    pub cut: Cut,
    /// The readings taken so far, by id: mm, or a comb's tooth.
    #[serde(default)]
    pub readings: BTreeMap<String, f64>,
}

/// How far the plan's program was cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Cut {
    NotYet,
    Done,
    /// Stopped, or failed, with the program's first `ops_done` ops finished.
    Stopped {
        ops_done: usize,
    },
}

impl Session {
    /// A session of `plan`, made with calibration `cal`, not yet cut.
    pub fn new(id: impl Into<String>, cal: &Calibration, plan: Plan) -> Session {
        Session {
            format: FORMAT.to_owned(),
            version: VERSION,
            id: id.into(),
            revision: cal.revision(),
            plan,
            cut: Cut::NotYet,
            readings: BTreeMap::new(),
        }
    }

    /// # Errors
    /// When it isn't a session of this version.
    pub fn from_json(text: &str) -> Result<Session, Error> {
        let s: Session = serde_json::from_str(text).map_err(|e| Error::File(e.to_string()))?;
        if s.format != FORMAT || s.version != VERSION {
            return Err(Error::File(format!(
                "not a calibration session this driver reads ({} version {})",
                s.format, s.version
            )));
        }
        Ok(s)
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_default() + "\n"
    }

    /// Whether reading `id` can be taken: every cut it reads finished.
    pub fn is_available(&self, id: &str) -> bool {
        let Some(r) = self.plan.reading(id) else {
            return false;
        };
        match self.cut {
            Cut::NotYet => false,
            Cut::Done => true,
            Cut::Stopped { ops_done } => r.needs().into_iter().all(|cut| {
                self.plan
                    .program
                    .ops
                    .iter()
                    .position(|op| op.id() == Some(cut))
                    .is_some_and(|k| k < ops_done)
            }),
        }
    }

    /// The readings that can be taken, in the plan's order.
    pub fn available(&self) -> Vec<&Reading> {
        self.plan
            .readings
            .iter()
            .filter(|r| self.is_available(&r.id))
            .collect()
    }

    /// Whether `value` can be reading `id`, and whether it is unusual.
    ///
    /// # Errors
    /// When the plan has no such reading, its cuts didn't all finish, or the value isn't one.
    pub fn check(&self, id: &str, value: f64) -> Result<Check, Error> {
        let check = self.plan.check(id, value)?;
        if !self.is_available(id) {
            return Err(Error::Reading(format!(
                "{id} can't be taken: not all its cuts were made"
            )));
        }
        Ok(check)
    }

    /// Keep `value` as reading `id` (an unusual one too: whether to keep it is asked first).
    ///
    /// # Errors
    /// As [`check`](Session::check).
    pub fn record(&mut self, id: &str, value: f64) -> Result<Check, Error> {
        let check = self.check(id, value)?;
        self.readings.insert(id.to_owned(), value);
        Ok(check)
    }

    /// Drop reading `id`, to take it again.
    pub fn forget(&mut self, id: &str) {
        self.readings.remove(id);
    }

    /// The readings taken that can be, each with its value, in the plan's order.
    pub fn taken(&self) -> impl Iterator<Item = (&Reading, f64)> {
        self.plan.readings.iter().filter_map(|r| {
            let v = self.readings.get(&r.id)?;
            self.is_available(&r.id).then_some((r, *v))
        })
    }

    /// The cut sheet with the readings taken: where its cuts went, by lowering the plan's
    /// program with the profile it was placed with.
    ///
    /// # Errors
    /// When the program no longer lowers.
    pub fn sheet(&self) -> Result<Sheet, Error> {
        let readings = self
            .readings
            .iter()
            .filter(|(id, _)| self.is_available(id))
            .map(|(id, v)| (id.clone(), *v))
            .collect();
        self.plan.sheet(self.id.clone(), readings)
    }

    /// What the readings taken propose for `cal`: stage 1 fitted, stage 2's combs read.
    ///
    /// # Errors
    /// When there are no readings yet, or they can't be fitted.
    pub fn propose(&self, cal: &Calibration) -> Result<Proposal, Error> {
        if self.readings.is_empty() {
            return Err(Error::Reading(format!("{}: no readings yet", self.id)));
        }
        let t = self.plan.program.board.thickness_mm;
        let mut p = match self.plan.routine {
            Routine::Heads => heads::propose(&heads::fit(&[self.sheet()?], cal)?, cal, t),
            Routine::RunOns => runons::propose(&combs(self.taken())?, cal, t),
        };
        if cal.revision() != self.revision {
            p.notes.push(format!(
                "Cut with the calibration at revision {}; it is at revision {} now, and each \
                 change is against now's value.",
                self.revision,
                cal.revision()
            ));
        }
        Ok(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::Stock;
    use crate::{Edit, Key, Why};

    fn stage_1() -> Session {
        let cal = Calibration::new();
        Session::new(
            "2026-11-16 18.00.00 calibrate stage_1",
            &cal,
            heads::plan(&cal, &Stock::offcut(1.5)).expect("a plan"),
        )
    }

    /// The readings the default calibration expects, slightly off: what a session would take.
    fn expected(s: &Session) -> Vec<(String, f64)> {
        s.plan
            .readings
            .iter()
            .zip([0.01, -0.02, 0.0, 0.02].iter().cycle())
            .map(|(r, n)| (r.id.clone(), r.expected_mm().expect("a length") + n))
            .collect()
    }

    #[test]
    fn nothing_is_read_before_the_cut_and_everything_after() {
        let mut s = stage_1();
        assert!(s.available().is_empty());
        assert!(s.record("W.width", 50.9).is_err());
        s.cut = Cut::Done;
        assert_eq!(s.available().len(), 16);
        for (id, v) in expected(&s) {
            assert_eq!(s.record(&id, v).expect("kept"), Check::Fine, "{id}");
        }
        assert!(matches!(
            s.record("W.width", 53.0),
            Ok(Check::Unusual { .. })
        ));
        s.forget("W.width");
        assert_eq!(s.readings.len(), 15);
        let p = s.propose(&Calibration::new()).expect("a proposal");
        assert_eq!(p.routine, Routine::Heads);
        assert!(p.suspects.is_empty(), "{:?}", p.suspects);
    }

    /// Stopped after the window: the window and drop-out can be read, nothing on the mount's
    /// outline can.
    #[test]
    fn a_stopped_cut_keeps_what_it_finished() {
        let mut s = stage_1();
        // The clamp and the window's four cuts.
        s.cut = Cut::Stopped { ops_done: 5 };
        let ids: Vec<&str> = s.available().iter().map(|r| r.id.as_str()).collect();
        assert_eq!(ids, ["W.width", "W.height", "D.width", "D.height"]);
        assert!(s.check("B.left", 24.6).is_err());
        // Without the window's back edge, its width (left and right) can be read, its height
        // can't.
        s.cut = Cut::Stopped { ops_done: 4 };
        assert!(s.is_available("W.width") && !s.is_available("W.height"));
    }

    #[test]
    fn a_session_reads_back_as_written_and_says_when_the_file_moved_on() {
        let mut s = stage_1();
        s.cut = Cut::Done;
        for (id, v) in expected(&s) {
            s.record(&id, v).expect("kept");
        }
        let back = Session::from_json(&s.to_json()).expect("reads");
        assert_eq!(back, s);
        assert!(Session::from_json(&s.to_json().replace(FORMAT, "something-else")).is_err());
        let later = Calibration::new()
            .set(
                &[Edit::set(
                    "blade.bevel.lean".parse::<Key>().expect("a key"),
                    48.0,
                )],
                &Why::typed("2026-11-16T18:31:00Z"),
            )
            .expect("set");
        let p = s.propose(&later).expect("a proposal");
        assert!(
            p.notes.iter().any(|n| n.contains("revision 0")),
            "{:?}",
            p.notes
        );
    }
}
