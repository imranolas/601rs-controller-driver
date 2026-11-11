//! A session: one probe, check, run or calibration, with everything it writes in one dated
//! folder under the data folder's `sessions/`: the log, copies of the programs, the readings
//! and the fit, and a `session.json` summary that `mc601 logs` lists.

use std::path::{Path, PathBuf};

use chrono::{DateTime, Local};
use serde_json::{Value, json};

use crate::target::Res;

pub struct Session {
    pub dir: PathBuf,
    pub log_path: PathBuf,
    started: DateTime<Local>,
    cmd: String,
    what: String,
    port: String,
}

/// A folder name from free text: no path separators or characters shells and Finder trip on.
fn safe(text: &str) -> String {
    let cleaned: String = text
        .chars()
        .map(|c| match c {
            '/' | '\\' | ':' | '*' | '?' | '"' | '<' | '>' | '|' => '-',
            c if c.is_control() => '-',
            c => c,
        })
        .collect();
    cleaned.trim().chars().take(60).collect()
}

/// The folder name for a session started at `at`: `2026-11-10 14.32.05 run calibration_2`.
pub fn folder_name(at: DateTime<Local>, cmd: &str, what: &str) -> String {
    let stamp = at.format("%Y-%m-%d %H.%M.%S");
    let what = safe(what);
    if what.is_empty() {
        format!("{stamp} {cmd}")
    } else {
        format!("{stamp} {cmd} {what}")
    }
}

impl Session {
    /// A new session folder under `data`; its log goes to `log` if given, else into the folder.
    pub fn create(
        data: &Path,
        cmd: &str,
        what: &str,
        port: &str,
        log: Option<&Path>,
    ) -> Res<Session> {
        let started = Local::now();
        let base = data.join("sessions");
        let name = folder_name(started, cmd, what);
        let mut dir = base.join(&name);
        let mut n = 2;
        while dir.exists() {
            dir = base.join(format!("{name} {n}"));
            n += 1;
        }
        std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let log_path = log.map_or_else(|| dir.join(format!("{cmd}.log")), Path::to_path_buf);
        let s = Session {
            dir,
            log_path,
            started,
            cmd: cmd.to_owned(),
            what: what.to_owned(),
            port: port.to_owned(),
        };
        s.write_summary("running")?;
        Ok(s)
    }

    /// Record how the session ended: `finished`, `failed`, `stopped`, or a check's tally.
    pub fn finish(&self, outcome: &str) {
        let _ = self.write_summary(outcome);
    }

    fn write_summary(&self, outcome: &str) -> Res<()> {
        let v = json!({
            "cmd": self.cmd,
            "what": self.what,
            "port": self.port,
            "started": self.started.to_rfc3339(),
            "ended": if outcome == "running" { Value::Null } else { Value::String(Local::now().to_rfc3339()) },
            "outcome": outcome,
            "log": self.log_path,
        });
        let file = self.dir.join("session.json");
        std::fs::write(&file, serde_json::to_string_pretty(&v)? + "\n")
            .map_err(|e| format!("{}: {e}", file.display()))?;
        Ok(())
    }
}

/// A past session, as `mc601 logs` lists it.
pub struct Summary {
    pub dir: PathBuf,
    pub cmd: String,
    pub what: String,
    pub port: String,
    pub outcome: String,
}

/// The sessions under `data`, newest first.
pub fn list(data: &Path) -> Vec<Summary> {
    let Ok(entries) = std::fs::read_dir(data.join("sessions")) else {
        return Vec::new();
    };
    let mut out: Vec<Summary> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_dir())
        .map(|dir| {
            let v: Value = std::fs::read_to_string(dir.join("session.json"))
                .ok()
                .and_then(|t| serde_json::from_str(&t).ok())
                .unwrap_or(Value::Null);
            let s = |k: &str| {
                v.get(k)
                    .and_then(Value::as_str)
                    .unwrap_or_default()
                    .to_owned()
            };
            let outcome = match s("outcome").as_str() {
                // Never finished: stopped with Ctrl-C, or it crashed.
                "running" | "" => "ended early".to_owned(),
                o => o.to_owned(),
            };
            Summary {
                cmd: s("cmd"),
                what: s("what"),
                port: s("port"),
                outcome,
                dir,
            }
        })
        .collect();
    out.sort_by(|a, b| b.dir.file_name().cmp(&a.dir.file_name()));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;

    #[test]
    fn folders_are_named_by_time_command_and_what() {
        let at = Local
            .with_ymd_and_hms(2026, 11, 10, 14, 32, 5)
            .single()
            .expect("a time");
        assert_eq!(
            folder_name(at, "run", "calibration_2"),
            "2026-11-10 14.32.05 run calibration_2"
        );
        assert_eq!(folder_name(at, "probe", ""), "2026-11-10 14.32.05 probe");
        assert_eq!(
            folder_name(at, "run", "a/b: c"),
            "2026-11-10 14.32.05 run a-b- c",
            "no separators"
        );
    }

    #[test]
    fn a_session_lists_with_its_outcome() {
        let data = std::env::temp_dir().join(format!("mc601-session-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&data);
        let s = Session::create(&data, "run", "job", "sim", None).expect("created");
        assert!(s.log_path.starts_with(&s.dir));
        assert_eq!(list(&data)[0].outcome, "ended early");
        s.finish("finished");
        let l = list(&data);
        assert_eq!(
            (l[0].cmd.as_str(), l[0].what.as_str(), l[0].outcome.as_str()),
            ("run", "job", "finished")
        );
        let _ = std::fs::remove_dir_all(&data);
    }
}
