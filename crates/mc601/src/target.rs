//! Open the machine (a serial port) or the simulator, with every byte logged.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use controller::{Config, Controller, ControllerError, connect, mode};
use link::tap::{self, Log};
use machine::Profile;
use sim::{Sim, SimConfig};

pub type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// Port name that selects the simulator instead of a serial port.
pub const SIM: &str = "sim";
/// The simulator with the X/Y limit-switch inputs mapped, so stage 7 drives into one.
pub const SIM_LIMITS: &str = "sim-limits";

/// Simulator speed-up: homing takes 3 s per axis on the model; at 10× the whole check
/// runs in about a minute.
pub(crate) const SIM_SPEEDUP: f64 = 10.0;

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

/// The simulator with the 601's travel as the machine profile gives it (`SimConfig::default()`'s
/// is a placeholder). The carriage's travel is counted from the zero homing leaves
/// `homing.offset_steps` off each minus switch; the simulator's, from that switch.
fn sim_config(p: &Profile) -> SimConfig {
    let mut plus = p.carriage_steps(p.travel_mm().hi());
    // Y travels at least as far as GMC pushes the V-groove switch, past the soft limit
    // (protocol reference §4.1).
    plus[1] = plus[1].max(p.vgroove_switch.map_or(0, |v| v.y_steps));
    SimConfig {
        homed_at_start: false,
        steps_per_mm: p.scale.x_steps_per_mm,
        travel: std::array::from_fn(|a| i64::from(plus[a] + p.homing.offset_steps[a])),
        ..SimConfig::default()
    }
}

/// Home and run `program` to its end on the simulator `mc601` builds for `profile`; the
/// simulator, for what went down the line.
#[cfg(test)]
pub(crate) fn run_on_the_simulator(profile: Profile, program: &program::Program) -> Sim {
    const SPEEDUP: f64 = 20.0;
    let (link, sim) = sim::spawn(sim_config(&profile), SPEEDUP);
    let (ctl, _) = connect(link, Config::default()).expect("connects");
    let config = runner::Config {
        speedup: SPEEDUP,
        ..runner::Config::default()
    };
    let mut r = runner::Runner::new(ctl, profile, config);
    r.configure().expect("configures");
    r.home().expect("homes");
    let (_requests, rx) = std::sync::mpsc::channel();
    let outcome = r.run(program, &rx, &mut |_| {}).expect("runs");
    assert_eq!(outcome, runner::Outcome::Finished);
    sim
}

/// Open `port` (or the simulator for `sim`), log every byte to `log_path`, connect.
pub fn open(
    port: &str,
    log_path: &Path,
    cmd: &str,
    cfg: Config,
) -> Res<(Controller<mode::Parameters>, String, Target)> {
    let log = Log::new(File::create(log_path).map_err(|e| format!("{}: {e}", log_path.display()))?);
    let log_path = log_path.to_path_buf();
    log.note(&format!("mc601 {cmd} {port}"));

    let (link, sim) = if is_sim_port(port) {
        let mut config = sim_config(&crate::calibration::machine());
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The simulator's travel is the carriage's from the minus switch: homing's 300 steps (7.5 mm)
    /// below zero, then X to the bevel head at GMC's limit (1641.45 mm) and Y on past the soft
    /// limits to the V-groove stub at 1045 mm.
    #[test]
    fn the_simulators_travel_is_the_profiles_from_the_switch() {
        let travel = sim_config(&Profile::gunnar_601()).travel;
        assert_eq!(travel, [300 + 65_658, 300 + 41_800]);
    }

    /// The default travel, which the runner's tests and other apps' simulators use, reaches as
    /// far as the 601's.
    #[test]
    fn the_default_travel_covers_the_601s() {
        let (default, the_601) = (
            SimConfig::default().travel,
            sim_config(&Profile::gunnar_601()).travel,
        );
        assert!(
            (0..2).all(|a| default[a] >= the_601[a]),
            "{default:?} < {the_601:?}"
        );
    }

    /// A program runs to its end on the simulator as `mc601 run --sim` builds it: homing,
    /// GMC's V-groove switch reset against the stub, then the cuts.
    #[test]
    fn a_program_runs_on_the_simulator_mc601_builds() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/programs/g_regular_mount.json");
        let program = crate::cut::read(&path).expect("a program");
        run_on_the_simulator(Profile::gunnar_601(), &program);
    }

    /// A jumbo board's far edges, cut with the bevel head on the simulator mc601 builds: the
    /// carriage near the end of its travel meets no limit switch.
    #[test]
    fn a_jumbo_boards_far_edges_cut_on_the_simulator() {
        let json = serde_json::json!({
            "format": program::FORMAT, "version": program::VERSION, "id": "jumbo",
            "board": { "width_mm": 1525.0, "height_mm": 1020.0, "thickness_mm": 1.5 },
            "material": { "name": "MSK_1.5", "speed_pct": 95.0, "head_settle_ms": 250 },
            "ops": [
                { "op": "clamp", "side": "both" },
                { "op": "cut", "head": "bevel", "path_mm": [[1525.0, 40.0], [1525.0, 1000.0]] },
                { "op": "cut", "head": "bevel", "path_mm": [[1500.0, 1020.0], [40.0, 1020.0]] }
            ]
        });
        let program = program::Program::from_json(&json.to_string()).expect("a program");
        run_on_the_simulator(Profile::gunnar_601(), &program);
    }
}
