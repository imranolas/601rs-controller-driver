//! Where the calibration file lives, and reading and writing it: the crate's only I/O, here so
//! that `mc601` and apps, which share the file (CALIBRATION.md §4), can't disagree about it.
//!
//! - The file is `$MC601_CALIBRATION`, else `calibration.json` in [`config_dir`], beside
//!   `mc601`'s `config.json`.
//! - A missing file is a machine never calibrated: the defaults.
//! - A write names the revision it read, and is refused if the file has moved on since, so
//!   neither writer overwrites the other's change. It goes to a temporary file first and is
//!   renamed into place, so a reader never sees half a file.

use std::path::{Path, PathBuf};

use crate::Error;
use crate::calibration::Calibration;

/// `mc601`'s folder: `$XDG_CONFIG_HOME/mc601`, else `%APPDATA%\mc601` on Windows, else
/// `~/.config/mc601`.
pub fn config_dir() -> PathBuf {
    let var = |name: &str| std::env::var_os(name).filter(|v| !v.is_empty());
    if let Some(x) = var("XDG_CONFIG_HOME") {
        return PathBuf::from(x).join("mc601");
    }
    if cfg!(windows) {
        if let Some(a) = var("APPDATA") {
            return PathBuf::from(a).join("mc601");
        }
    }
    var("HOME")
        .or_else(|| var("USERPROFILE"))
        .map_or_else(|| PathBuf::from("."), PathBuf::from)
        .join(".config")
        .join("mc601")
}

/// The machine's calibration file.
pub fn default_path() -> PathBuf {
    std::env::var_os("MC601_CALIBRATION")
        .filter(|v| !v.is_empty())
        .map_or_else(|| config_dir().join("calibration.json"), PathBuf::from)
}

fn io(path: &Path, e: &std::io::Error) -> Error {
    Error::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    }
}

/// The calibration at `path`; the defaults if there's no file.
///
/// # Errors
/// When the file can't be read, or isn't a calibration file this driver reads.
pub fn load(path: &Path) -> Result<Calibration, Error> {
    match std::fs::read_to_string(path) {
        Ok(text) => Calibration::from_json(&text).map_err(|e| match e {
            Error::File(m) => Error::File(format!("{}: {m}", path.display())),
            e => e,
        }),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Calibration::new()),
        Err(e) => Err(io(path, &e)),
    }
}

/// Write `cal` to `path`, which must still be at `read`, the revision it was read at.
///
/// # Errors
/// [`Error::Moved`] when the file has changed since; or it can't be read or written.
pub fn save(path: &Path, cal: &Calibration, read: u32) -> Result<(), Error> {
    let now = load(path)?.revision();
    if now != read {
        return Err(Error::Moved { read, now });
    }
    if cal.revision() == read {
        return Ok(());
    }
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|e| io(dir, &e))?;
    }
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, cal.to_json()).map_err(|e| io(&tmp, &e))?;
    std::fs::rename(&tmp, path).map_err(|e| io(path, &e))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::{Edit, Why};

    fn scratch(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("calibrate-store-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn saved_and_loaded_and_a_stale_write_refused() {
        let dir = scratch("save");
        let path = dir.join("mc601").join("calibration.json");
        let read = load(&path).expect("defaults");
        assert_eq!(read, Calibration::new());
        let why = Why::typed("2026-11-15T10:00:00Z");
        let key = |s: &str| s.parse().expect(s);
        let mine = read
            .set(&[Edit::set(key("machine.scale.x"), 40.01)], &why)
            .expect("set");
        save(&path, &mine, read.revision()).expect("saved");
        assert_eq!(load(&path).expect("loads"), mine);
        // Another writer that read before this save is refused, not let overwrite it.
        let theirs = read
            .set(&[Edit::set(key("machine.scale.y"), 39.99)], &why)
            .expect("set");
        match save(&path, &theirs, read.revision()) {
            Err(Error::Moved { read: 0, now: 1 }) => {}
            other => panic!("{other:?}"),
        }
        assert_eq!(load(&path).expect("loads"), mine);
        assert!(!path.with_extension("json.tmp").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_broken_file_says_where_it_is() {
        let dir = scratch("broken");
        std::fs::create_dir_all(&dir).expect("a folder");
        let path = dir.join("calibration.json");
        std::fs::write(&path, "{ \"format\": \"601rs-calibration\" ").expect("written");
        let e = load(&path).expect_err("refused").to_string();
        assert!(e.contains("calibration.json"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
