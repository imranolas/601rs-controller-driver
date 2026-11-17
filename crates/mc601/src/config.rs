//! What `mc601` remembers between runs: the machine's serial port and the data folder, in
//! `config.json` in `mc601`'s folder (`~/.config/mc601/`, `$XDG_CONFIG_HOME/mc601/`, or
//! `%APPDATA%\mc601\` on Windows: `calibrate::store::config_dir`, beside the calibration
//! file), or the file `$MC601_CONFIG` names. The data folder holds the sessions (`mc601 logs`); it defaults to
//! `~/Documents/mc601`, and `$MC601_DATA` overrides it.

use std::path::{Path, PathBuf};

use link::serial::PortInfo;
use serde::{Deserialize, Serialize};

use crate::target::{Res, SIM, SIM_LIMITS};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    /// The machine's serial port, e.g. `/dev/cu.usbserial-2110`.
    pub port: Option<String>,
    /// Where sessions are kept.
    pub data_dir: Option<PathBuf>,
}

fn home() -> PathBuf {
    std::env::var_os("HOME").map_or_else(|| PathBuf::from("."), PathBuf::from)
}

/// The config file.
pub fn path() -> PathBuf {
    if let Some(p) = std::env::var_os("MC601_CONFIG") {
        return p.into();
    }
    calibrate::store::config_dir().join("config.json")
}

impl Config {
    /// The saved config, or the defaults if there is none yet.
    pub fn load() -> Res<Config> {
        let file = path();
        match std::fs::read_to_string(&file) {
            Ok(text) => {
                Ok(serde_json::from_str(&text).map_err(|e| format!("{}: {e}", file.display()))?)
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Config::default()),
            Err(e) => Err(format!("{}: {e}", file.display()).into()),
        }
    }

    pub fn save(&self) -> Res<()> {
        let file = path();
        if let Some(dir) = file.parent() {
            std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        }
        std::fs::write(&file, serde_json::to_string_pretty(self)? + "\n")
            .map_err(|e| format!("{}: {e}", file.display()))?;
        Ok(())
    }

    /// Where sessions go: `$MC601_DATA`, else the configured folder, else `~/Documents/mc601`.
    pub fn data_dir(&self) -> PathBuf {
        std::env::var_os("MC601_DATA")
            .map(PathBuf::from)
            .or_else(|| self.data_dir.clone())
            .unwrap_or_else(|| home().join("Documents").join("mc601"))
    }
}

/// The ports worth offering. macOS lists every serial device twice: `/dev/cu.X` (call-out,
/// the one a host opens) and `/dev/tty.X` (dial-in, which waits for carrier detect). Only that
/// `tty.` twin is hidden; every other name (`/dev/ttyUSB0`, `/dev/ttyACM0`, `COM4`, …) is shown.
pub fn visible_ports(all: &[PortInfo]) -> Vec<&PortInfo> {
    let has_callout_twin = |name: &str| {
        name.strip_prefix("/dev/tty.").is_some_and(|dev| {
            all.iter()
                .any(|p| p.name.strip_prefix("/dev/cu.") == Some(dev))
        })
    };
    all.iter().filter(|p| !has_callout_twin(&p.name)).collect()
}

/// Which port to open, and why it was chosen: the simulator if asked, else the port given,
/// else the configured one (if it is plugged in), else the only USB-serial port there is.
pub fn resolve_port(
    given: Option<&str>,
    sim: bool,
    sim_limits: bool,
    config: &Config,
    ports: &[PortInfo],
) -> Result<(String, String), String> {
    if sim_limits {
        return Ok((
            SIM_LIMITS.to_owned(),
            "the simulator, limit switches mapped".to_owned(),
        ));
    }
    if sim {
        return Ok((SIM.to_owned(), "the simulator".to_owned()));
    }
    if let Some(p) = given {
        return Ok((p.to_owned(), "given".to_owned()));
    }
    let shown = visible_ports(ports);
    let names = |v: &[&PortInfo]| {
        v.iter()
            .map(|p| p.name.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    };
    if let Some(p) = &config.port {
        if shown.iter().any(|q| &q.name == p) {
            return Ok((p.clone(), "configured".to_owned()));
        }
        return Err(format!(
            "the configured port {p} isn't connected (ports: {}). Plug the adapter in, give \
             --port, or change it with `mc601 config --port PORT`",
            if shown.is_empty() {
                "none".to_owned()
            } else {
                names(&shown)
            }
        ));
    }
    let usb: Vec<&PortInfo> = shown.into_iter().filter(|p| p.usb).collect();
    match usb.as_slice() {
        [one] => Ok((one.name.clone(), "the only USB-serial port".to_owned())),
        [] => Err(
            "no USB-serial port found: plug the adapter in, or give --port. --sim runs \
                   against the simulator"
                .to_owned(),
        ),
        many => Err(format!(
            "several USB-serial ports ({}): give --port, or remember one with `mc601 config \
             --port PORT`",
            names(many)
        )),
    }
}

/// `path`, shortened with `~` for the home folder.
pub fn tidy(path: &Path) -> String {
    let home = home();
    match path.strip_prefix(&home) {
        Ok(rest) => format!("~/{}", rest.display()),
        Err(_) => path.display().to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ports(names: &[(&str, bool)]) -> Vec<PortInfo> {
        names
            .iter()
            .map(|&(name, usb)| PortInfo {
                name: name.to_owned(),
                usb,
            })
            .collect()
    }

    #[test]
    fn port_list_hides_only_the_macos_dialin_twin() {
        let all = ports(&[
            ("/dev/cu.usbserial-1420", false),
            ("/dev/tty.usbserial-1420", false), // twin of the cu. device: hidden
            ("/dev/ttyUSB0", false),            // Linux USB-serial
            ("/dev/ttyACM0", false),
            ("/dev/ttyS0", false),
            ("COM4", false),
            ("/dev/tty.debug-console", false), // no cu. twin: shown
        ]);
        let shown: Vec<&str> = visible_ports(&all)
            .iter()
            .map(|p| p.name.as_str())
            .collect();
        assert_eq!(
            shown,
            [
                "/dev/cu.usbserial-1420",
                "/dev/ttyUSB0",
                "/dev/ttyACM0",
                "/dev/ttyS0",
                "COM4",
                "/dev/tty.debug-console",
            ]
        );
    }

    #[test]
    fn the_port_is_given_configured_or_the_only_usb_one() {
        let one = ports(&[
            ("/dev/cu.Bluetooth", false),
            ("/dev/cu.usbserial-2110", true),
            ("/dev/tty.usbserial-2110", true),
        ]);
        let none = Config::default();
        let pick =
            |given, cfg: &Config, ps: &[PortInfo]| resolve_port(given, false, false, cfg, ps);
        assert_eq!(
            pick(None, &none, &one).map(|p| p.0),
            Ok("/dev/cu.usbserial-2110".into())
        );
        assert_eq!(
            pick(Some("/dev/x"), &none, &one).map(|p| p.0),
            Ok("/dev/x".into())
        );
        let cfg = Config {
            port: Some("/dev/cu.usbserial-2110".into()),
            ..Config::default()
        };
        assert_eq!(pick(None, &cfg, &one).map(|p| p.1), Ok("configured".into()));
        assert!(
            pick(None, &cfg, &[]).is_err(),
            "configured but not plugged in"
        );
        assert!(pick(None, &none, &[]).is_err(), "nothing to use");
        let two = ports(&[("/dev/cu.usbserial-1", true), ("/dev/cu.usbserial-2", true)]);
        assert!(
            pick(None, &none, &two)
                .expect_err("ambiguous")
                .contains("several")
        );
        assert_eq!(
            resolve_port(Some("/dev/x"), true, false, &cfg, &one).map(|p| p.0),
            Ok(SIM.into())
        );
    }
}
