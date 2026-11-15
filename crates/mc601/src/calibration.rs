//! The machine's calibration (CALIBRATION.md §4), for every command that drives or calibrates:
//! loaded once, from `--calibration FILE`, else the file `mc601` shares with Bevel
//! (`calibrate::store::default_path`), and `mc601 calibration`, to see it and change it.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::sync::OnceLock;

use calibrate::{Calibration, Edit, Key, Val, ValueState, Why, store};
use machine::Profile;

use crate::config::tidy;
use crate::style::{BOLD, DIM, GOOD, paint};
use crate::target::Res;

static LOADED: OnceLock<(PathBuf, Calibration)> = OnceLock::new();

/// Load the calibration for this run, from `file` or the shared one.
pub fn init(file: Option<PathBuf>) -> Res<()> {
    let path = file.unwrap_or_else(store::default_path);
    let cal = store::load(&path)?;
    let _ = LOADED.set((path, cal));
    Ok(())
}

/// The calibration file this run uses.
pub fn path() -> PathBuf {
    LOADED
        .get()
        .map_or_else(store::default_path, |(p, _)| p.clone())
}

/// The calibration as loaded; the defaults before [`init`] (in tests).
pub fn current() -> Calibration {
    LOADED.get().map(|(_, c)| c.clone()).unwrap_or_default()
}

/// The machine profile, calibrated.
pub fn machine() -> Profile {
    current().machine()
}

/// For a session's first line: which calibration drives it.
pub fn describe() -> String {
    let c = current();
    match c.revision() {
        0 => "calibration: the defaults".to_owned(),
        r => format!("calibration r{r}"),
    }
}

fn now() -> String {
    chrono::Utc::now().format("%Y-%m-%dT%H:%M:%SZ").to_string()
}

/// A value as a person reads it: numbers to 0.1 µm, without trailing zeros.
fn shown(v: &Val) -> String {
    match v {
        Val::Num(x) => {
            let s = format!("{:.4}", (x * 1e4).round() / 1e4);
            let s = s.trim_end_matches('0').trim_end_matches('.');
            if s == "-0" {
                "0".to_owned()
            } else {
                s.to_owned()
            }
        }
        Val::Text(t) => format!("{t:?}"),
    }
}

fn shown_opt(v: Option<&Val>) -> String {
    v.map_or_else(|| "default".to_owned(), shown)
}

/// The day of an RFC 3339 time.
fn day(at: &str) -> &str {
    at.get(..10).unwrap_or(at)
}

/// Which group of the overview a value sits in.
fn group(key: &Key, cal: &Calibration) -> String {
    match key {
        Key::HeadOffset(..) | Key::Scale(_) => "machine".to_owned(),
        Key::Blade { profile: None, .. } => "blade, on the reference board".to_owned(),
        _ => {
            let id = key.profile_id().unwrap_or_default();
            let name = cal
                .profiles()
                .into_iter()
                .find(|(p, _)| p == id)
                .map(|(_, n)| n)
                .unwrap_or_default();
            format!("profile {id}: {name}")
        }
    }
}

/// `mc601 calibration show`: every value, the file's or the default; `set_only`: the file's.
pub fn show(set_only: bool) -> Res<bool> {
    let cal = current();
    let at = cal
        .history()
        .last()
        .map_or_else(String::new, |e| format!(", {}", day(&e.at)));
    anstream::println!(
        "{} {}  {}",
        paint(BOLD, "calibration"),
        tidy(&path()),
        paint(DIM, format!("· revision {}{at}", cal.revision()))
    );
    let values: Vec<ValueState> = cal
        .values()
        .into_iter()
        .filter(|v| v.set || !set_only)
        .collect();
    if values.is_empty() {
        println!("nothing calibrated: every value is the driver's default");
        return Ok(true);
    }
    let mut last_group = String::new();
    for v in values {
        let g = group(&v.key, &cal);
        if g != last_group {
            anstream::println!("\n{}", paint(BOLD, &g));
            last_group = g;
        }
        let value = format!("{} {}", shown(&v.value), v.key.unit());
        let whence = match (&v.last, v.set) {
            (Some(l), true) => {
                let mut s = format!("r{} {}", l.revision, day(&l.at));
                if let Some(r) = &l.routine {
                    let _ = write!(s, " {r}");
                }
                if let Some(se) = l.se {
                    let _ = write!(s, " ±{se:.3}");
                }
                paint(GOOD, s)
            }
            (_, true) => paint(GOOD, "set"),
            (_, false) => paint(DIM, "default"),
        };
        anstream::println!(
            "  {:34} {value:>18}  {whence}  {}",
            v.key.to_string(),
            paint(DIM, v.label)
        );
    }
    Ok(true)
}

/// `mc601 calibration history`: every change, oldest first.
pub fn history() -> Res<bool> {
    let cal = current();
    if cal.history().is_empty() {
        println!("{}: no changes yet", tidy(&path()));
        return Ok(true);
    }
    for e in cal.history() {
        let mut head = format!("r{}  {}", e.revision, e.at);
        for part in [&e.routine, &e.session, &e.note].into_iter().flatten() {
            let _ = write!(head, "  {part}");
        }
        anstream::println!("{}", paint(BOLD, head));
        for c in &e.changes {
            let se = c.se.map_or_else(String::new, |s| format!("  ±{s:.3}"));
            anstream::println!(
                "  {:34} {} → {}{}",
                c.key.to_string(),
                shown_opt(c.before.as_ref()),
                shown_opt(c.after.as_ref()),
                paint(DIM, se)
            );
        }
    }
    Ok(true)
}

/// Write `next` over the file read as `was`, and say what changed.
fn commit(was: &Calibration, next: &Calibration) -> Res<bool> {
    if next.revision() == was.revision() {
        println!("nothing changes");
        return Ok(true);
    }
    let path = path();
    store::save(&path, next, was.revision())?;
    if let Some(e) = next.history().last() {
        anstream::println!(
            "{} {}",
            paint(BOLD, format!("revision {}", e.revision)),
            paint(DIM, tidy(&path))
        );
        for c in &e.changes {
            println!(
                "  {:34} {} → {}",
                c.key.to_string(),
                shown_opt(c.before.as_ref()),
                shown_opt(c.after.as_ref())
            );
        }
    }
    Ok(true)
}

fn why(note: Option<String>) -> Why {
    Why {
        note,
        ..Why::typed(now())
    }
}

/// `mc601 calibration set KEY VALUE [KEY VALUE ...]`: one revision.
pub fn set(pairs: &[String], note: Option<String>) -> Res<bool> {
    if pairs.is_empty() || pairs.len() % 2 != 0 {
        return Err("give each KEY with its VALUE".into());
    }
    let mut edits = Vec::new();
    for kv in pairs.chunks(2) {
        let key: Key = kv[0].parse()?;
        let value = if key.is_text() {
            Val::Text(kv[1].clone())
        } else {
            Val::Num(
                kv[1]
                    .parse()
                    .map_err(|_| format!("{}: {:?} isn't a number", kv[0], kv[1]))?,
            )
        };
        edits.push(Edit::set(key, value));
    }
    let was = current();
    commit(&was, &was.set(&edits, &why(note))?)
}

/// `mc601 calibration unset KEY ...`: back to the default, as one revision.
pub fn unset(keys: &[String], note: Option<String>) -> Res<bool> {
    let edits = keys
        .iter()
        .map(|k| Ok(Edit::unset(k.parse()?)))
        .collect::<Res<Vec<_>>>()?;
    let was = current();
    commit(&was, &was.set(&edits, &why(note))?)
}

/// `mc601 calibration remove-profile ID`.
pub fn remove_profile(id: &str) -> Res<bool> {
    let was = current();
    commit(&was, &was.remove_profile(id, &why(None))?)
}

/// `mc601 calibration revert N`: every value as at revision N, as a new revision.
pub fn revert(to: u32) -> Res<bool> {
    let was = current();
    commit(&was, &was.revert(to, &why(None))?)
}

/// `mc601 calibration path`.
pub fn print_path() -> Res<bool> {
    println!("{}", path().display());
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_are_shown_as_a_person_reads_them() {
        assert_eq!(shown(&Val::Num(2.3 + 1.4)), "3.7");
        assert_eq!(shown(&Val::Num(-71.45)), "-71.45");
        assert_eq!(shown(&Val::Num(200.0)), "200");
        assert_eq!(shown(&Val::Num(-0.00001)), "0");
        assert_eq!(
            shown(&Val::Text("Mount board 1.4".into())),
            "\"Mount board 1.4\""
        );
    }
}
