//! Open the machine (a serial port) or the simulator, with every byte logged.

use std::fs::File;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use controller::{Config, Controller, ControllerError, connect, mode};
use link::tap::{self, Log};
use sim::{Sim, SimConfig};

pub type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// Port name that selects the simulator instead of a serial port.
pub const SIM: &str = "sim";
/// The simulator with the X/Y limit-switch inputs mapped, so stage 7 drives into one.
pub const SIM_LIMITS: &str = "sim-limits";

/// Simulator speed-up: homing takes 3 s per axis on the model; at 10× the whole check
/// runs in about a minute.
const SIM_SPEEDUP: f64 = 10.0;

/// Does `port` name the simulator rather than a serial port?
pub fn is_sim_port(port: &str) -> bool {
    port == SIM || port == SIM_LIMITS
}

pub struct Target {
    pub sim: Option<Arc<Sim>>,
    pub log: Log,
    pub log_path: PathBuf,
}

impl Target {
    pub fn is_sim(&self) -> bool {
        self.sim.is_some()
    }
}

/// What to check when `connect` fails, judged by what (if anything) came back.
fn connect_hint(e: &ControllerError) -> &'static str {
    match e {
        ControllerError::ResyncFailed { bytes_heard: 0, .. } => {
            "Nothing came back: check the cable (null-modem), that GMC is closed, and that the \
             box is powered."
        }
        ControllerError::ResyncFailed { lines_heard, .. } if lines_heard.is_empty() => {
            "Bytes came back but no reply line: the line settings are probably wrong (the \
             driver uses 115200 8-N-1)."
        }
        ControllerError::ResyncFailed { .. } => {
            "The controller answers, but not to the driver's resync probe (@I100-119). Keep the \
             log: this firmware needs a different probe input."
        }
        _ => "Check the cable (null-modem), that GMC is closed, and that the box is powered.",
    }
}

/// Default log file: `gunnar-<cmd>-<unix seconds>.log` in the current directory.
fn default_log(cmd: &str) -> PathBuf {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    PathBuf::from(format!("gunnar-{cmd}-{secs}.log"))
}

/// Session timing for the hardware tools: faster polling than the default so timings
/// (dead time, override) can be measured to ~20 ms.
pub fn tool_config() -> Config {
    Config {
        reply_timeout: Duration::from_millis(1000),
        poll_interval: Duration::from_millis(20),
        flow_timeout: Duration::from_secs(20),
        ..Config::default()
    }
}

/// Open `port` (or the simulator for `sim`), log to `log_path`, connect.
pub fn open(
    port: &str,
    log_path: Option<&str>,
    cmd: &str,
    cfg: Config,
) -> Res<(Controller<mode::Parameters>, String, Target)> {
    let log_path = log_path.map_or_else(|| default_log(cmd), PathBuf::from);
    let log = Log::new(File::create(&log_path)?);
    log.note(&format!("gunnar {cmd} {port}"));

    let (link, sim) = if is_sim_port(port) {
        // The machine as its profile describes it (SimConfig::default() is the placeholder
        // the capture replays need).
        let p = machine::Profile::gunnar_601();
        let (e, sc) = (p.envelope_mm, p.scale);
        let steps = |mm: f64, scale: f64| (mm * scale).round() as i64;
        let mut config = SimConfig {
            homed_at_start: false,
            steps_per_mm: sc.x_steps_per_mm,
            envelope: [
                (
                    steps(e.x[0], sc.x_steps_per_mm),
                    steps(e.x[1], sc.x_steps_per_mm),
                ),
                // Y travels at least as far as GMC pushes the V-groove switch, past the
                // soft limit (protocol reference §4.1).
                (
                    steps(e.y[0], sc.y_steps_per_mm),
                    steps(e.y[1], sc.y_steps_per_mm)
                        .max(p.vgroove_switch.map_or(0, |v| i64::from(v.y_steps))),
                ),
            ],
            ..SimConfig::default()
        };
        if port == SIM_LIMITS {
            config.input_defs = [239, 240, 243, 244]
                .map(|n| (n, "001h,0,0".to_owned()))
                .to_vec();
        }
        let (link, sim) = sim::spawn(config, SIM_SPEEDUP);
        (link, Some(Arc::new(sim)))
    } else {
        let link = link::serial::open(&link::serial::SerialConfig::new(port))
            .map_err(|e| format!("cannot open {port}: {e}"))?;
        (link, None)
    };
    let link = tap::tap(link, log.clone());
    let (ctl, version) = connect(link, cfg).map_err(|e| {
        format!(
            "cannot start a session with the controller on {port}: {e}\n{}",
            connect_hint(&e)
        )
    })?;
    log.note(&format!("connected: {version}"));
    Ok((ctl, version, Target { sim, log, log_path }))
}
