//! `gunnar check`: staged hardware checks, each one confirmed by the operator first.
//!
//! | Stage | Moves | Answers |
//! |-------|-------|---------|
//! | 2 | no  | replies come back in the order sent (a command and a probe in one write), `0x05` heartbeat (`#C8`), green button edges `0x06`/`0x07` (`#C35`) |
//! | 3 | yes | homing like GMC, the V-groove switch reset, `0x04`, jogs and `0x01`, `@B`, `@Q` vs `$A`, outputs across a mode switch, clamp sides |
//! | 4 | yes | a mode-2 square the way GMC does it: `@X` 15 → 14 → 04, exact return; `@B` mid-path discards the rest |
//! | 5 | yes | v1.00 in-stream `A`/`W`/`I`/`C`, `@N` progress, `C1` vs dead time, `@O` with `C8`/`C16`; compact paths, a second speed entry, in-stream `G`, what `@N` counts and how wide it is |
//! | 6 | yes | buffer size via `@H`; which of `0x02`/`0x03` means "full"; `#C19` watchdog during a long stream; no overrun at full speed; `@B` on a held stream discards it |
//! | 7 | yes | `@S` loses the position; a limit switch arrives as an async `*03x` and shows in `@Y` (the drive towards it ends 20 mm past GMC's drivable range) |
//!
//! Safety rules:
//! - Homing raises both heads first (`$A02`). Every other moving stage starts with the heads
//!   up and the carriage at the park, GMC's end-of-job position at the far end of X, clear of
//!   the board's corner and the home switches; it asks before travelling there.
//! - Every move stays within [`REACH_MM`] of the park, into the table ([`inward`]), so it stays
//!   inside the soft limits even when a stop under test fails. Two moves leave them: stage 3's
//!   V-groove switch reset, GMC's own after homing (3 cm on against a stub at the back), and
//!   stage 7's drive towards a limit switch the controller can see.
//! - The heads stay up: only the spare bit `0x80` and, after asking, the clamps are switched.
//! - Nothing is saved (`#W` is never sent); the parameters a stage changes are read first and
//!   written back at the end. Ctrl-C brakes (`@B`) and exits.

use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

use controller::{
    Axis, Controller, ControllerError, Event, JogTarget, Level, LimitSwitch, Mode, ModeMarker,
    OverridePct, Param, Setting, SettingKey, Side, Signal, SignalAcks, SpeedOverride, StopHandle,
    StreamFlags, Units, WatchdogMs, mode,
};
use machine::{Profile, Step};
use protocol::Command;

use crate::operator::{Confirm, Operator};
use crate::report::Report;
use crate::target::Res;

/// Speed-table entry used for all test moves (GMC's choice too).
const ENTRY: u8 = 20;
/// A second entry, at half the speed: stage 5 checks that each vector's entry picks its feed,
/// which is how one stream can travel and cut at different speeds.
const TRAVEL_ENTRY: u8 = 21;
/// The machine's facts, from its profile (`machine::Profile`).
fn profile() -> Profile {
    Profile::gunnar_601()
}

/// X/Y scale [GCF `MachineScale` 400 /cm].
fn steps_per_mm() -> i32 {
    profile().scale.x_steps_per_mm.round() as i32
}

fn step_20mm() -> i32 {
    20 * steps_per_mm()
}

/// End of the X drivable range in steps: GMC's `DRX` 156 cm [GCF], its own soft-limit basis and
/// the profile's envelope. A fitted X+ limit switch sits at or just beyond it.
fn x_drivable() -> i32 {
    let p = profile();
    (p.envelope_mm.x[1] * p.scale.x_steps_per_mm).round() as i32
}

/// How far past the drivable range stage 7 may drive while looking for the limit switch.
fn limit_overtravel() -> i32 {
    step_20mm()
}
/// Stage 7's speed towards the limit switch, steps/s (10 mm/s).
const LIMIT_SPEED: u32 = 400;
/// The override stage 5 runs at.
const HALF_SPEED: OverridePct = OverridePct::new(50).expect("50 % is a valid override");

type Params = Controller<mode::Parameters>;

/// Settings a stage may change; read before, written back after.
const TOUCHED: &[SettingKey] = &[
    SettingKey::DeadTime,
    SettingKey::PositionedSignal,
    SettingKey::HomingDoneSignal,
    SettingKey::FlowControl,
    SettingKey::Heartbeat,
    SettingKey::Units,
    SettingKey::HostWatchdog,
    SettingKey::SignalAcks,
    SettingKey::Speed(10),
    SettingKey::Speed(11),
    SettingKey::Speed(ENTRY),
    SettingKey::Speed(TRAVEL_ENTRY),
    SettingKey::Ramp,
    SettingKey::HomeOffset(Axis::X),
    SettingKey::HomeOffset(Axis::Y),
    SettingKey::HomeOffset(Axis::Z),
];

pub struct Ctx {
    pub r: Report,
    pub op: Operator,
    pub stop: StopHandle,
    pub events: Receiver<Event>,
    /// Test move speed in steps/s (`#G20`).
    pub speed: u32,
    /// Simulator time runs faster; scale waits and timing thresholds by this.
    pub scale: f64,
    /// The operator chose to stop: every prompt after that is answered "no", and the run ends
    /// once the stage in hand has wound down (parameters are still restored).
    pub quit: bool,
    /// The operator has confirmed the park is where GMC parks (asked once per run).
    pub park_confirmed: bool,
}

impl Ctx {
    /// Ask before a step that moves or changes something. A step not taken is reported as
    /// skipped, by the operator's choice or because they stopped the run.
    fn go(&mut self, prompt: &str, step: &str) -> bool {
        if self.quit {
            return false;
        }
        match self.op.confirm(prompt) {
            Confirm::Go => return true,
            Confirm::Skip => self.r.skip(step, "skipped by the operator"),
            Confirm::Quit => {
                self.quit = true;
                self.r.skip(step, "the operator stopped the run");
            }
        }
        false
    }

    /// `ms` of machine time as a wall-clock duration.
    fn t(&self, ms: u64) -> Duration {
        Duration::from_secs_f64(ms as f64 / 1000.0 * self.scale)
    }

    /// A host-watchdog timeout that must stay `ms` long relative to our wall-clock polling.
    /// On the simulator one wall-clock ms is 1/scale machine ms, so scale it up to keep the
    /// same margin against the poll interval as on the machine.
    fn watchdog(&self, ms: u64) -> WatchdogMs {
        WatchdogMs::saturating((ms as f64 / self.scale) as u64)
    }

    fn drain(&self) {
        while self.events.try_recv().is_ok() {}
    }

    fn signals(&self) -> Vec<Signal> {
        self.events
            .try_iter()
            .filter_map(|e| match e {
                Event::Signal(s) => Some(s),
                _ => None,
            })
            .collect()
    }

    /// The flow-control bytes (0x03, 0x02) among the signals since the last look.
    fn flow_signals(&self) -> Vec<Signal> {
        self.signals()
            .into_iter()
            .filter(|s| matches!(s, Signal::BufferAlmostFull | Signal::BufferResume))
            .collect()
    }

    /// Wait for `wanted` in order (other signals ignored); returns what was seen.
    fn wait_signals(&self, wanted: &[Signal], timeout: Duration) -> Vec<Signal> {
        let deadline = Instant::now() + timeout;
        let mut seen = Vec::new();
        while seen.len() < wanted.len() {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.events.recv_timeout(left) {
                Ok(Event::Signal(s)) if s == wanted[seen.len()] => seen.push(s),
                Ok(_) => {}
                Err(_) => break,
            }
        }
        seen
    }
}

fn pos<M: ModeMarker>(c: &mut Controller<M>) -> Res<[i32; 3]> {
    Ok([
        c.position_steps(Axis::X)?,
        c.position_steps(Axis::Y)?,
        c.position_steps(Axis::Z)?,
    ])
}

/// How far any move strays from the park, mm: the longest is a 100 mm jog that a stop under
/// test should cut short.
pub const REACH_MM: i32 = 100;

fn reach() -> i32 {
    REACH_MM * steps_per_mm()
}

/// The way into the table from the park, X and Y: towards the middle of the envelope. On the
/// 601 that is towards home in X (the park is at the far end) and away from it in Y.
fn inward() -> [i32; 2] {
    let p = profile();
    let towards_middle = |park: i32, [lo, hi]: [f64; 2], scale: f64| {
        if f64::from(park) > (lo + hi) / 2.0 * scale {
            -1
        } else {
            1
        }
    };
    [
        towards_middle(p.park_steps[0], p.envelope_mm.x, p.scale.x_steps_per_mm),
        towards_middle(p.park_steps[1], p.envelope_mm.y, p.scale.y_steps_per_mm),
    ]
}

/// The profile, its park and these moves take home to be at the low end of X and Y, as GMC's
/// coordinates show (after homing it travels to `X0`, `Y40600` [CAP g_init]). Whether the
/// controller homes that way (`&D`); if not, nothing should move.
fn home_at_low_ends(p: &mut Params, ctx: &mut Ctx) -> Res<bool> {
    for axis in [Axis::X, Axis::Y] {
        if p.homing_side(axis)? != Side::Minus {
            ctx.r.fail(
                "home at the low ends",
                format!(
                    "{axis:?} homes towards plus (&D), but the profile and its park assume                      minus: no stage will move"
                ),
            );
            return Ok(false);
        }
    }
    Ok(true)
}

/// Where every moving stage starts: both heads up, and the carriage at the park, asking
/// before travelling there. Whether it got there; a stage that didn't must not move.
fn to_park(p: Params, ctx: &mut Ctx) -> Res<(Params, bool)> {
    let mut p = p;
    if !home_at_low_ends(&mut p, ctx)? {
        return Ok((p, false));
    }
    // Positions mean nothing until homed (after `@S`, say): never travel on them.
    if !p.status()?.homed {
        ctx.r.skip("travel to the park", "the machine is not homed");
        return Ok((p, false));
    }
    let profile = profile();
    let mut j = p.into_jog()?;
    j.set_outputs(profile.neutral_outputs)?;
    let [x, y, z] = pos(&mut j)?;
    let [px, py] = profile.park_steps;
    let p = if [x, y] == [px, py] {
        j.into_parameters()?
    } else {
        let mm = |s: i32| s / steps_per_mm();
        let prompt = format!(
            "Heads up, and travel to the park at X {} mm, Y {} mm (from X {} mm, Y {} mm).",
            mm(px),
            mm(py),
            mm(x),
            mm(y)
        );
        if !ctx.go(&prompt, "travel to the park") {
            return Ok((j.into_parameters()?, false));
        }
        let mut c = j.into_cutting()?;
        c.vector(ENTRY, px, py, z)?;
        idle(&mut c)?;
        c.into_parameters()?
    };
    let here = park_is_gmcs(ctx);
    Ok((p, here))
}

/// Once per run: is this where GMC parks? It is the profile's park, so anything but a "yes"
/// means the profile doesn't describe this machine, and nothing more moves.
fn park_is_gmcs(ctx: &mut Ctx) -> bool {
    if ctx.park_confirmed {
        return true;
    }
    let answer = ctx.op.ask(
        "Is the carriage where GMC parks it: at the right, the head close to the front rail? \
         (y/n)",
    );
    match answer.as_deref() {
        None | Some("y") => {
            ctx.park_confirmed = true;
            ctx.r.pass(
                "the park is GMC's",
                answer.map_or("(simulator)".to_owned(), |a| format!("operator: {a}")),
            );
            true
        }
        Some(a) => {
            ctx.r.fail(
                "the park is GMC's",
                format!(
                    "operator: {a}. The profile does not describe this machine; the run stops \
                     here"
                ),
            );
            ctx.quit = true;
            false
        }
    }
}

fn idle<M: ModeMarker>(c: &mut Controller<M>) -> Res<()> {
    c.wait_idle(Duration::from_secs(180))?;
    Ok(())
}

// ------------------------------------------------------------------ run

/// Run the selected stages. Returns false if a stage aborted (parameters not restored).
pub fn run(mut p: Params, stages: &[u8], ctx: &mut Ctx) -> Res<bool> {
    let snapshot: Vec<(SettingKey, Result<Setting, String>)> = TOUCHED
        .iter()
        .map(|&key| (key, p.read_setting(key).map_err(|e| e.to_string())))
        .collect();
    let override_pct = p.override_pct().ok();

    for &stage in stages {
        let title = match stage {
            2 => "Stage 2: reply order and signals (no motion)",
            3 => "Stage 3: homing, jogs, brake, outputs",
            4 => "Stage 4: mode-2 square, GMC style",
            5 => "Stage 5: v1.00 in-stream commands",
            6 => "Stage 6: buffer, flow control, watchdog",
            7 => "Stage 7: faults (@S, limit switch)",
            _ => continue,
        };
        ctx.r.heading(title);
        if !ctx.go(&format!("{title}. Start?"), title) {
            if ctx.quit {
                break;
            }
            continue;
        }
        let result = match stage {
            2 => stage2(p, ctx),
            3 => stage3(p, ctx),
            4 => stage4(p, ctx),
            5 => stage5(p, ctx),
            6 => stage6(p, ctx),
            _ => stage7(p, ctx),
        };
        match result {
            Ok(back) if ctx.quit => {
                p = back;
                break;
            }
            Ok(back) => p = back,
            Err(e) => {
                let _ = ctx.stop.brake();
                ctx.r.fail(title, format!("aborted: {e}"));
                println!("\n  Braked (@B). {UNRESTORED}");
                return Ok(false);
            }
        }
    }

    ctx.r.heading("Restoring parameters");
    restore(&mut p, &snapshot, &mut ctx.r);
    if let Some(pct) = override_pct {
        let _ = p.set_override(pct);
    }
    Ok(true)
}

/// What to do about the settings a run changed and could not put back. Nothing is saved
/// (`#W`), so a power cycle restores them all. Starting GMC does not: it never writes `#C6`,
/// `#C7`, `#C8`, `#C19` or `#G21` (`probe.rs`'s tables of what it writes).
pub const UNRESTORED: &str = "Parameters this run changed were NOT restored. Nothing was saved: \
    power-cycle the controller before using it again, with GMC too (GMC does not rewrite #C6, \
    #C7, #C8, #C19 or #G21).";

/// Write every snapshotted setting back exactly as it was read. One that could not be read
/// before the run (or read back as a valid value) is reported, never written with a substitute.
fn restore(p: &mut Params, snapshot: &[(SettingKey, Result<Setting, String>)], r: &mut Report) {
    let mut failed = Vec::new();
    for (key, saved) in snapshot {
        let result = match saved {
            Ok(setting) => p.set(*setting).map_err(|e| e.to_string()),
            Err(e) => Err(format!("not read before the run ({e})")),
        };
        if let Err(e) = result {
            failed.push(format!("{key}: {e}"));
        }
    }
    if failed.is_empty() {
        r.pass(
            "restore",
            "all changed parameters written back (nothing saved with #W)",
        );
    } else {
        r.fail(
            "restore",
            format!(
                "NOT restored, power-cycle the controller to reset them: {}",
                failed.join("; ")
            ),
        );
    }
}

// ------------------------------------------------------------------ shared steps

/// Home Z, then X and Y, with GMC's settings [CAP g_init]: both heads up (`$A02`), then
/// `@@ #G10,1750 #OZ,0 $Hz` then `#G10,1750 #G11,750 #R100 #OX,300 #OY,300 $Hxy`. Not if the
/// carriage is on a home switch, as GMC won't [GMC].
fn home_all(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    let mut pressed = Vec::new();
    for (input, axis) in [(235, "X"), (236, "Y")] {
        if p.read_input(input)? {
            pressed.push(format!("{axis} (I{input})"));
        }
    }
    if !pressed.is_empty() {
        ctx.r.fail(
            "homing",
            format!(
                "not started: the carriage is on the {} home switch. Move it off (GMC's manual \
                 mover) and run stage 3 again",
                pressed.join(" and ")
            ),
        );
        return Ok(p);
    }
    // GMC has the heads up from its connect on [GMC]; with the outputs at
    // `00`, as they may be after a power-on, the bevel head is down.
    let mut j = p.into_jog()?;
    j.set_outputs(profile().neutral_outputs)?;
    let mut p = j.into_parameters()?;
    p.set(Setting::Units(Units::Steps))?;
    p.set(Setting::HomingDoneSignal(true))?;
    p.set(Setting::PositionedSignal(true))?;
    p.clear_error()?;
    p.set(Setting::Speed {
        entry: 10,
        value: 1750,
    })?;
    p.set(Setting::HomeOffset {
        axis: Axis::Z,
        value: 0,
    })?;
    let mut j = p.into_jog()?;
    ctx.drain();
    j.home(&[Axis::Z])?;
    let got = ctx.wait_signals(&[Signal::HomingDone], Duration::from_secs(120));
    idle(&mut j)?;
    ctx.r.check(
        !got.is_empty(),
        "0x04 after $Hz",
        "homing-done signal (#C6,1)",
    );

    let mut p = j.into_parameters()?;
    p.set(Setting::Speed {
        entry: 10,
        value: 1750,
    })?;
    p.set(Setting::Speed {
        entry: 11,
        value: 750,
    })?;
    p.set(Setting::HomeOffset {
        axis: Axis::X,
        value: 300,
    })?;
    p.set(Setting::HomeOffset {
        axis: Axis::Y,
        value: 300,
    })?;
    p.set(Setting::RampMs(100))?; // GMC's `#R,100` for homing
    let mut j = p.into_jog()?;
    j.home(&[Axis::X, Axis::Y])?;
    idle(&mut j)?;
    let s = j.status()?;
    let at = pos(&mut j)?;
    ctx.r
        .check(s.homed, "homed", format!("{s} after $Hxy, at {at:?} steps"));
    Ok(j.into_parameters()?)
}

/// Set the bevel head's V-groove switch to normal cutting, as GMC does after homing and the
/// runner does too: `machine::vgroove_reset`, at GMC's speeds. Only if Y homed towards the
/// operator (`homed_towards`, "f"), so that +Y runs to the stub at the back.
fn vgroove_reset(mut p: Params, ctx: &mut Ctx, homed_towards: Option<&str>) -> Res<Params> {
    let step = "V-groove switch to normal";
    let profile = profile();
    let Some(switch) = profile.vgroove_switch else {
        return Ok(p);
    };
    if homed_towards.is_some_and(|a| !a.starts_with('f')) {
        ctx.r.skip(
            step,
            "Y did not home towards the operator, so +Y may not run to the stub",
        );
        return Ok(p);
    }
    let prompt = format!(
        "Set the bevel head's V-groove switch to normal cutting, as GMC does after homing: at \
         full speed ({} mm/s) up the left side to Y {} mm, 3 cm on against the stub at the back \
         (past the soft limit), and back.",
        profile.speed(profile.travel_speed_pct) / profile.scale.y_steps_per_mm as u32,
        switch.y_steps / steps_per_mm() - 30,
    );
    if !ctx.go(&prompt, step) {
        return Ok(p);
    }
    let [x, ..] = pos(&mut p)?;
    let mut p = exec(p, &machine::vgroove_reset(&profile, 0, x))?;
    setup_moves(&mut p, ctx)?; // the test speed and ramp again, for the moves after
    match ctx
        .op
        .ask("Did the head meet the stub, and is its V-groove switch at normal cutting? (y/n)")
    {
        Some(a) => ctx.r.check(a == "y", step, format!("operator: {a}")),
        None => ctx.r.pass(step, "sent (simulator)"),
    }
    Ok(p)
}

/// Run steps from `machine` as the runner does: settings in mode 0, outputs in mode 1, each
/// vector in mode 2 until the controller is idle.
fn exec(mut p: Params, steps: &[Step]) -> Res<Params> {
    for step in steps {
        match step {
            Step::Send(Command::Set(s)) => p.set(*s)?,
            Step::Send(Command::SetOutputs(bits)) => {
                let mut j = p.into_jog()?;
                j.set_outputs(*bits)?;
                p = j.into_parameters()?;
            }
            &Step::Send(Command::Vector {
                speed_entry,
                x,
                y,
                z,
            }) => {
                let mut c = p.into_cutting()?;
                c.vector(speed_entry, x, y, z)?;
                idle(&mut c)?;
                p = c.into_parameters()?;
            }
            Step::WaitIdle => {} // each vector waits
            other => return Err(format!("not a step stage 3 runs: {other:?}").into()),
        }
    }
    Ok(p)
}

fn setup_moves(p: &mut Params, ctx: &Ctx) -> Res<()> {
    p.set(Setting::Units(Units::Steps))?;
    p.set(Setting::PositionedSignal(true))?;
    p.set(Setting::DeadTimeMs(100))?;
    p.set(Setting::RampMs(250))?;
    p.set(Setting::Speed {
        entry: ENTRY,
        value: ctx.speed,
    })?;
    Ok(())
}

fn require_homed(p: &mut Params, ctx: &mut Ctx, what: &str) -> Res<bool> {
    let s = p.status()?;
    if !s.homed {
        ctx.r.skip(what, "machine not homed: run stage 3 first");
    }
    Ok(s.homed)
}

// ------------------------------------------------------------------ stage 2

fn stage2(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    reply_order(&mut p, ctx)?;

    p.set(Setting::HeartbeatMs(500))?;
    ctx.drain();
    thread::sleep(ctx.t(2000));
    let n = ctx
        .signals()
        .iter()
        .filter(|s| **s == Signal::KeepAlive)
        .count();
    ctx.r.check(
        n >= 2,
        "#C8 heartbeat",
        format!("{n} × 0x05 in 2 s of silence at #C8,500"),
    );

    p.set(Setting::HeartbeatMs(0))?;
    thread::sleep(ctx.t(300));
    ctx.drain();
    thread::sleep(ctx.t(1500));
    let n = ctx
        .signals()
        .iter()
        .filter(|s| **s == Signal::KeepAlive)
        .count();
    ctx.r
        .check(n == 0, "#C8,0 stops it", format!("{n} × 0x05 in 1.5 s"));

    p.set(Setting::SignalAcks(SignalAcks {
        input1: true,
        input2: false,
    }))?;
    ctx.drain();
    ctx.op.press_green(ctx.t(300));
    let want = [Signal::Input1High, Signal::Input1Low];
    let got = ctx.wait_signals(&want, Duration::from_secs(30));
    ctx.r.check(
        got == want,
        "#C35 green button",
        format!("expected 0x06 then 0x07, saw {got:?}"),
    );
    Ok(p)
}

/// How often each command goes out with a probe behind it.
const ORDER_ROUNDS: usize = 20;

/// Reply pairing, settling a `*NNN` (a rejection, or a fault that crossed the command?) and
/// every resync rely on the controller answering in the order it is sent. Here a command and a
/// probe go out in one write, without waiting, many times over, and the command's reply must
/// come first every time. Three kinds of reply: a CR to a mode-0 write (the dead time, written
/// back as it is), a CR to a master command (`@M0`, the switch a crossing fault was once taken
/// to reject), and a rejection (`&C52`: GMC's `#C52,800` is refused with `*021` [CAP g_init],
/// so C52 is no parameter).
fn reply_order(p: &mut Params, ctx: &mut Ctx) -> Res<()> {
    let dead_time = p.read_setting(SettingKey::DeadTime)?;
    let commands = [
        Command::Set(dead_time),
        Command::SetMode(Mode::Parameters),
        Command::QueryParam(Param::Config(52)),
    ];
    let mut refused = false;
    for command in commands {
        let mut overtaken = 0;
        let mut replies: Vec<String> = Vec::new();
        for _ in 0..ORDER_ROUNDS {
            let reply = match p.request_with_probe(&command) {
                Ok(reply) => reply.to_string(),
                Err(ControllerError::Rejected { code, .. }) => {
                    refused = true;
                    code.to_string()
                }
                Err(ControllerError::Overtaken { .. }) => {
                    overtaken += 1;
                    // Its own reply is still to come, and a `*021` arriving unasked would pass
                    // for a fault and hold back the rest of the stage. Let it arrive; drop it.
                    thread::sleep(ctx.t(100));
                    let _ = p.wait_idle(Duration::from_secs(1));
                    continue;
                }
                Err(e) => return Err(e.into()),
            };
            if !replies.contains(&reply) {
                replies.push(reply);
            }
        }
        ctx.r.check(
            overtaken == 0,
            &format!("reply order: {command}"),
            format!(
                "{} of {ORDER_ROUNDS} answered before the probe sent right behind; reply: {}",
                ORDER_ROUNDS - overtaken,
                replies.join(", ")
            ),
        );
    }
    if !refused {
        ctx.r.info(
            "reply order: a rejection",
            "&C52 was answered, not refused: the order of a rejection is still unseen",
        );
    }
    Ok(())
}

// ------------------------------------------------------------------ stage 3

fn stage3(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    setup_moves(&mut p, ctx)?;
    let mut p = if ctx.go(
        "Raise the heads, then home Z, then X and Y, exactly as GMC does (about a minute). \
         Clear the table.",
        "homing",
    ) {
        let mut p = home_all(p, ctx)?;
        let question = "Which way did Y move while homing: f = towards you (the front rail), \
                        b = away? About how far, in cm?";
        if p.status()?.homed {
            let seen = ctx.op.ask(question);
            if let Some(seen) = &seen {
                ctx.r.info(
                    "Y while homing (operator)",
                    format!("{seen} (from GMC's park: about 20 cm to the front, then 7.5 mm back)"),
                );
            }
            p = vgroove_reset(p, ctx, seen.as_deref())?;
        }
        p
    } else {
        p
    };
    if !require_homed(&mut p, ctx, "jogs")? {
        return Ok(p);
    }
    let (p, at_park) = to_park(p, ctx)?;
    if !at_park {
        return Ok(p);
    }
    let [ix, iy] = inward();
    let dirs = [(Axis::X, ix), (Axis::Y, iy)];
    let mut j = p.into_jog()?;

    for (axis, dir) in dirs {
        if !ctx.go(
            &format!("Jog {axis:?} 20 mm into the table and back."),
            &format!("jog {axis:?}"),
        ) {
            continue;
        }
        let i = axis.index();
        let start = pos(&mut j)?;
        ctx.drain();
        j.jog_steps(ENTRY, axis, JogTarget::By(step_20mm() * dir))?;
        idle(&mut j)?;
        let end = pos(&mut j)?;
        let positioned = ctx.signals().contains(&Signal::Positioned);
        ctx.r.check(
            end[i] - start[i] == step_20mm() * dir,
            &format!("jog {axis:?}"),
            format!(
                "{} → {} steps (asked {:+})",
                start[i],
                end[i],
                step_20mm() * dir
            ),
        );
        ctx.r.check(
            positioned,
            &format!("0x01 after jog {axis:?}"),
            "positioned signal (#C5,1)",
        );
        // The board frame's Y (PROGRAM_FORMAT §2), which every program relies on.
        if axis == Axis::Y && dir > 0 {
            if let Some(a) = ctx
                .op
                .ask("Did the head move away from you, towards the back rail? (y/n)")
            {
                ctx.r.check(
                    a == "y",
                    "+Y is away from the operator",
                    format!("operator: {a}. Programs assume it (PROGRAM_FORMAT §2)"),
                );
            }
        }
        j.jog_steps(ENTRY, axis, JogTarget::To(start[i]))?;
        idle(&mut j)?;
    }

    let x_dir = dirs[0].1;
    if ctx.go(
        "Jog X 100 mm into the table and brake (@B) after a moment.",
        "@B during a jog",
    ) {
        let start = pos(&mut j)?;
        j.jog_steps(ENTRY, Axis::X, JogTarget::By(reach() * x_dir))?;
        thread::sleep(ctx.t(800));
        j.brake()?;
        idle(&mut j)?;
        let end = pos(&mut j)?;
        let moved = (end[0] - start[0]) * x_dir;
        ctx.r.check(
            moved > 0 && moved < reach(),
            "@B brakes a jog",
            format!("stopped after {moved} of {} steps", reach()),
        );
        j.jog_steps(ENTRY, Axis::X, JogTarget::To(start[0]))?;
        idle(&mut j)?;
    }

    // Spare output bit 7 (i_free, Q249): switches nothing on the 601.
    j.set_outputs(0x82)?;
    let q = j.read_output(249)?;
    ctx.r.check(
        q,
        "@Q249 follows $A bit 7",
        format!("$A82 → @Q249,{}", u8::from(q)),
    );
    let p = j.into_parameters()?;
    let mut j = p.into_jog()?;
    let kept = j.read_output(249)?;
    ctx.r.info(
        "outputs across a mode switch",
        if kept {
            "kept (as the captures show)"
        } else {
            "cleared (as the spec says)"
        },
    );
    j.set_outputs(0x02)?;

    if ctx.go(
        "Close each clamp in turn (outputs 0x20, then 0x40). Hands clear.",
        "clamps",
    ) {
        for (bits, label) in [
            (0x22u8, "bit 0x20 (.gcf i_pl)"),
            (0x42, "bit 0x40 (.gcf i_pr)"),
        ] {
            j.set_outputs(bits)?;
            thread::sleep(ctx.t(1000));
            let answer = ctx
                .op
                .ask("Which clamp closed? l = left, r = right, b = both, n = none:");
            ctx.r.info(
                &format!("clamp {label}"),
                answer.unwrap_or_else(|| "(simulator)".into()),
            );
            j.set_outputs(0x02)?;
            thread::sleep(ctx.t(1000));
        }
    }
    Ok(j.into_parameters()?)
}

// ------------------------------------------------------------------ stage 4

fn observe_status<M: ModeMarker>(c: &mut Controller<M>, ctx: &Ctx) -> Res<Vec<String>> {
    let deadline = Instant::now() + Duration::from_secs(120);
    let mut codes: Vec<String> = Vec::new();
    loop {
        let s = c.status()?;
        let code = s.to_string();
        if codes.last() != Some(&code) {
            codes.push(code);
        }
        if !s.moving || Instant::now() > deadline {
            break;
        }
        thread::sleep(ctx.t(10).max(Duration::from_millis(5)));
    }
    Ok(codes)
}

fn stage4(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    if !require_homed(&mut p, ctx, "stage 4")? {
        return Ok(p);
    }
    if !ctx.go(
        "Drive a 20 mm square in mode 2, head up, and return.",
        "square",
    ) {
        return Ok(p);
    }
    setup_moves(&mut p, ctx)?;
    let (mut p, at_park) = to_park(p, ctx)?;
    if !at_park {
        return Ok(p);
    }
    let [ix, iy] = inward();
    let (dx, dy) = (step_20mm() * ix, step_20mm() * iy);
    let [x0, y0, z0] = pos(&mut p)?;
    let mut c = p.into_cutting()?;
    c.outputs(0x02)?;
    for (x, y) in square(x0, y0, dx, dy) {
        c.vector(ENTRY, x, y, z0)?;
    }
    let codes = observe_status(&mut c, ctx)?;
    ctx.r.info("@X during the square", codes.join(" → "));
    ctx.r.check(
        codes.iter().any(|c| c == "@X14") && codes.last().is_some_and(|c| c == "@X04"),
        "@X sequence",
        "moving (@X14) then idle (@X04); @X15 = dead time, only seen if the poll catches it",
    );
    let end = pos(&mut c)?;
    ctx.r.check(
        end == [x0, y0, z0],
        "square returns exactly",
        format!("{end:?} vs start"),
    );
    if ctx.go(
        "Drive the square again and brake it (@B) during the first leg: the rest of the path \
         must be discarded, not resumed. Head up.",
        "@B mid-path",
    ) {
        brake_mid_path(&mut c, ctx, [x0, y0, z0], (dx, dy))?;
    }
    Ok(c.into_parameters()?)
}

/// The stage-4 square from `(x0, y0)`: away along X, away along Y, back along X, back along Y.
fn square(x0: i32, y0: i32, dx: i32, dy: i32) -> [(i32, i32); 4] {
    [(x0 + dx, y0), (x0 + dx, y0 + dy), (x0, y0 + dy), (x0, y0)]
}

/// Brake a mode-2 path in its first leg, through the stop lane as the runner's stop-now will,
/// and check that the rest of the path is discarded rather than resumed. The path is the
/// square just driven, so even a controller that resumes stays inside it.
fn brake_mid_path(
    c: &mut Controller<mode::Cutting>,
    ctx: &mut Ctx,
    start: [i32; 3],
    (dx, dy): (i32, i32),
) -> Res<()> {
    let [x0, y0, z0] = start;
    for (x, y) in square(x0, y0, dx, dy) {
        c.vector(ENTRY, x, y, z0)?;
    }
    time_to_motion(c)?;
    thread::sleep(ctx.t(200));
    ctx.stop.brake()?;
    c.clear_stop()?; // resync past the stop and release the latch
    idle(c)?;
    let stopped_at = pos(c)?;
    let level = c.buffer_level_pct()?;
    thread::sleep(ctx.t(1500)); // past the #C1,100 dead time and longer than a whole leg
    let s = c.status()?;
    let later = pos(c)?;
    if stopped_at == start && later == start {
        ctx.r.info(
            "@B mid-path",
            "the square had finished before the brake landed: inconclusive, try a lower --speed",
        );
    } else {
        ctx.r.check(
            later == stopped_at && !s.moving,
            "@B discards the rest of the path",
            format!("braked at {stopped_at:?} (@H{level}); 1.5 s later at {later:?}, {s}"),
        );
    }
    idle(c)?; // if it did resume, let it finish before driving back
    c.vector(ENTRY, x0, y0, z0)?;
    idle(c)?;
    let end = pos(c)?;
    ctx.r.check(
        end == start,
        "stream accepts vectors after @B",
        format!("back at {end:?}"),
    );
    Ok(())
}

// ------------------------------------------------------------------ stage 5

fn time_to_motion(c: &mut Controller<mode::Cutting>) -> Res<Duration> {
    let start = Instant::now();
    loop {
        let s = c.status()?;
        if s.moving && !s.waiting {
            return Ok(start.elapsed());
        }
        if !s.moving && start.elapsed() > Duration::from_millis(200) {
            // Already finished between polls.
            return Ok(start.elapsed());
        }
        if start.elapsed() > Duration::from_secs(10) {
            return Err("no motion within 10 s".into());
        }
    }
}

fn timed_move(c: &mut Controller<mode::Cutting>, entry: u8, [x, y, z]: [i32; 3]) -> Res<Duration> {
    let start = Instant::now();
    c.vector(entry, x, y, z)?;
    idle(c)?;
    Ok(start.elapsed())
}

fn stage5(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    if !require_homed(&mut p, ctx, "stage 5")? {
        return Ok(p);
    }
    if !ctx.go(
        "Run v1.00 mode-2 features: short moves into the table from the park, heads up.",
        "v1.00",
    ) {
        return Ok(p);
    }
    setup_moves(&mut p, ctx)?;
    let (mut p, at_park) = to_park(p, ctx)?;
    if !at_park {
        return Ok(p);
    }
    let d = step_20mm() * inward()[0];
    let [x0, y0, z0] = pos(&mut p)?;
    let mut c = p.into_cutting()?;

    // (a) A and W in the stream, @N progress, @Q readback.
    c.set_command_counter(0)?;
    c.outputs(0x82)?;
    c.dwell_ms(1000)?;
    c.vector(ENTRY, x0 + d, y0, z0)?;
    c.outputs(0x02)?;
    thread::sleep(ctx.t(400));
    let s = c.status()?;
    let n_dwell = c.command_counter()?;
    let q = c.read_output(249)?;
    let at = pos(&mut c)?;
    ctx.r.check(
        s.waiting,
        "W dwell reports waiting",
        format!("{s} during W1000"),
    );
    // "The command currently being processed" [SPEC1 p.15]: the dwell is the second command.
    ctx.r.info(
        "@N during the dwell",
        match n_dwell {
            2 => "2: it numbers the command being executed".to_owned(),
            1 => "1: it counts the commands finished".to_owned(),
            n => format!("{n}: see the @N count after A, W, L, A below"),
        },
    );
    ctx.r.check(
        q,
        "A82 applied in sequence",
        format!("@Q249,{} during the dwell", u8::from(q)),
    );
    ctx.r.check(
        at[0] == x0,
        "vector waits for the dwell",
        format!("X {} during the dwell", at[0]),
    );
    idle(&mut c)?;
    let n = c.command_counter()?;
    ctx.r.info(
        "@N after A, W, L, A",
        format!("{n} (the simulator assumes 4)"),
    );
    let q = c.read_output(249)?;
    ctx.r
        .check(!q, "A02 after the vector", format!("@Q249,{}", u8::from(q)));
    c.vector(ENTRY, x0, y0, z0)?;
    idle(&mut c)?;

    // (b) C1 skips the dead time.
    let mut p = c.into_parameters()?;
    p.set(Setting::DeadTimeMs(1000))?;
    let mut c = p.into_cutting()?;
    c.vector(ENTRY, x0 + d, y0, z0)?;
    let t_dead = time_to_motion(&mut c)?;
    idle(&mut c)?;
    c.flags(StreamFlags {
        no_dead_time: true,
        ..StreamFlags::default()
    })?;
    c.vector(ENTRY, x0, y0, z0)?;
    let t_c1 = time_to_motion(&mut c)?;
    idle(&mut c)?;
    ctx.r.check(
        t_c1 * 2 < t_dead,
        "C1 skips the #C1 dead time",
        format!(
            "start after {} ms with #C1,1000; {} ms with C1",
            t_dead.as_millis(),
            t_c1.as_millis()
        ),
    );
    let mut p = c.into_parameters()?;
    p.set(Setting::DeadTimeMs(100))?;
    let mut c = p.into_cutting()?;

    // (c) Does a C flag advance @N?
    c.set_command_counter(0)?;
    c.flags(StreamFlags {
        no_dead_time: true,
        ..StreamFlags::default()
    })?;
    c.vector(ENTRY, x0 + d, y0, z0)?;
    idle(&mut c)?;
    let n = c.command_counter()?;
    ctx.r.info(
        "@N after C1, L",
        format!("{n}: C {} counted", if n >= 2 { "is" } else { "is not" }),
    );
    c.vector(ENTRY, x0, y0, z0)?;
    idle(&mut c)?;

    // (d) Override applies after C8, not after C16.
    let d2 = 2 * d;
    c.set_override(HALF_SPEED)?;
    c.flags(StreamFlags {
        speed_override: Some(SpeedOverride::Off),
        ..StreamFlags::default()
    })?;
    let t_off = timed_move(&mut c, ENTRY, [x0 + d2, y0, z0])?;
    c.flags(StreamFlags {
        speed_override: Some(SpeedOverride::On),
        ..StreamFlags::default()
    })?;
    let t_on = timed_move(&mut c, ENTRY, [x0, y0, z0])?;
    c.set_override(OverridePct::NORMAL)?;
    ctx.r.check(
        t_on.as_secs_f64() > 1.4 * t_off.as_secs_f64(),
        "@O50 honours C16/C8",
        format!(
            "40 mm: {} ms after C16 (no override), {} ms after C8",
            t_off.as_millis(),
            t_on.as_millis()
        ),
    );

    // (e) I221: hold the stream until the green button.
    ctx.drain();
    c.wait_input(221, Level::High)?;
    c.vector(ENTRY, x0 + d, y0, z0)?;
    thread::sleep(ctx.t(1500));
    let s = c.status()?;
    let at = pos(&mut c)?;
    ctx.r.check(
        at[0] == x0 && s.waiting,
        "I221 holds the stream",
        format!("X {} and {s} before the button", at[0]),
    );
    ctx.op.press_green(ctx.t(300));
    c.wait_idle(Duration::from_secs(120))?;
    let at = pos(&mut c)?;
    ctx.r.check(
        at[0] == x0 + d,
        "stream resumes on the button",
        format!("X {}", at[0]),
    );
    c.vector(ENTRY, x0, y0, z0)?;
    idle(&mut c)?;

    // (f) A compact path [SPEC1 p.33]: one absolute vector, then relative steps without the
    //     `L20,` prefix, unchanged axes left out. Every step is positive, so a firmware that
    //     read them as absolute would head for a point near home: a long way, but inside the
    //     soft limits. From the park there is room for 20 mm of +X and +Y.
    let up = step_20mm();
    let path = [
        [x0, y0, z0],
        [x0 + up / 2, y0, z0],
        [x0 + up, y0 + up / 2, z0],
        [x0 + up, y0 + up, z0 + 150], // and turn the blade
    ];
    c.path(ENTRY, &path)?;
    idle(&mut c)?;
    let end = pos(&mut c)?;
    let compact: usize = Command::path(ENTRY, &path)
        .iter()
        .map(|cmd| cmd.encode().len())
        .sum();
    let absolute: usize = path.iter().map(|&[x, y, z]| vector_bytes(x, y, z)).sum();
    ctx.r.check(
        end == path[3],
        "compact path lands exactly",
        format!("{end:?}; {compact} bytes against {absolute} as absolute vectors"),
    );
    c.vector(ENTRY, x0, y0, z0)?;
    idle(&mut c)?;

    // ...and a second speed entry: each vector names its entry, so one stream can travel and
    // cut at different feeds without changing anything mid-stream.
    let mut p = c.into_parameters()?;
    p.set(Setting::Speed {
        entry: TRAVEL_ENTRY,
        value: (ctx.speed / 2).max(1),
    })?;
    let mut c = p.into_cutting()?;
    let t20 = timed_move(&mut c, ENTRY, [x0 + d2, y0, z0])?;
    let t21 = timed_move(&mut c, TRAVEL_ENTRY, [x0, y0, z0])?;
    ctx.r.check(
        t21.as_secs_f64() > 1.4 * t20.as_secs_f64(),
        "each vector's speed entry picks its feed",
        format!(
            "40 mm: {} ms with L{ENTRY}, {} ms with L{TRAVEL_ENTRY} at half the speed",
            t20.as_millis(),
            t21.as_millis()
        ),
    );

    // (g) In-stream G sets an entry "for all subsequent moves with this entry" [SPEC1 p.34]:
    //     in sequence with the stream, or from the moment it arrives? Out at full speed, G
    //     halves the entry, back: in sequence, the way back takes about twice as long.
    let started = Instant::now();
    c.vector(ENTRY, x0 + d2, y0, z0)?;
    c.stream_speed(ENTRY, (ctx.speed / 2).max(1))?;
    c.vector(ENTRY, x0, y0, z0)?;
    // The carriage turns back the moment it arrives, so a poll seldom sees the far point
    // itself: the outward time is the last poll before it started back.
    let mut far = (Duration::ZERO, 0);
    loop {
        let x = (c.position_steps(Axis::X)? - x0).abs();
        if x > far.1 {
            far = (started.elapsed(), x);
        }
        if far.1 >= d2.abs() * 9 / 10 && x < far.1 {
            break;
        }
        if started.elapsed() > Duration::from_secs(120) {
            return Err("the outward move never arrived".into());
        }
    }
    let t_out = far.0;
    idle(&mut c)?;
    let t_back = started.elapsed().saturating_sub(t_out);
    let ratio = t_back.as_secs_f64() / t_out.as_secs_f64().max(1e-3);
    ctx.r.info(
        "in-stream G",
        format!(
            "40 mm out in {} ms, back in {} ms after G halved the entry: {}",
            t_out.as_millis(),
            t_back.as_millis(),
            if ratio > 1.5 {
                "applied in sequence with the stream"
            } else if ratio < 1.25 {
                "applied on receipt (the move ahead slowed too)"
            } else {
                "unclear"
            }
        ),
    );
    c.stream_speed(ENTRY, ctx.speed)?;

    // (h) How wide is @N (shown as `@Nxxxx` [SPEC1 p.15])? Set it near 10 000 and run three
    //     short vectors, the kind of command it certainly counts.
    let step = steps_per_mm() * d.signum(); // 1 mm into the table
    c.set_command_counter(9998)?;
    for x in [x0 + step, x0, x0 + step] {
        c.vector(ENTRY, x, y0, z0)?;
    }
    idle(&mut c)?;
    let n = c.command_counter()?;
    ctx.r.info(
        "@N width",
        format!(
            "@N9998, then three vectors: {n} ({})",
            if n >= 10_000 {
                "more than four digits"
            } else if n < 9998 {
                "it wraps after 9999"
            } else {
                "the vectors were not counted"
            }
        ),
    );
    c.vector(ENTRY, x0, y0, z0)?;
    idle(&mut c)?;
    Ok(c.into_parameters()?)
}

// ------------------------------------------------------------------ stage 6

/// Five laps of a 15 mm-radius circle in the square into the table from `(x0, y0)`, as 1250
/// short vectors (~22 KB): ends where it starts.
fn circle(x0: i32, y0: i32, sx: i32, sy: i32) -> Vec<(i32, i32)> {
    let r = 600.0;
    let (cx, cy) = (
        f64::from(x0) + r * f64::from(sx),
        f64::from(y0) + r * f64::from(sy),
    );
    let mut pts = vec![(x0, cy.round() as i32)];
    let segments = 250;
    for k in 1..=5 * segments {
        let a = std::f64::consts::TAU * f64::from(k) / f64::from(segments);
        let x = cx - r * a.cos() * f64::from(sx);
        let y = cy - r * a.sin() * f64::from(sy);
        pts.push((x.round() as i32, y.round() as i32));
    }
    pts.push((x0, y0));
    pts
}

fn vector_bytes(x: i32, y: i32, z: i32) -> usize {
    Command::Vector {
        speed_entry: ENTRY,
        x,
        y,
        z,
    }
    .encode()
    .len()
}

fn stage6(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    if !require_homed(&mut p, ctx, "stage 6")? {
        return Ok(p);
    }
    setup_moves(&mut p, ctx)?;
    let (mut p, at_park) = to_park(p, ctx)?;
    if !at_park {
        return Ok(p);
    }
    let [sx, sy] = inward();
    let [x0, y0, z0] = pos(&mut p)?;
    let pts = circle(x0, y0, sx, sy);

    // (a) Buffer size: fill it during a long dead time (nothing is consumed), then ask @H.
    if ctx.go(
        "Measure the buffer: 100 short moves on a 30 mm circle, then return.",
        "buffer size",
    ) {
        p.set(Setting::FlowControl(false))?;
        p.set(Setting::DeadTimeMs(3000))?;
        let mut c = p.into_cutting()?;
        let mut bytes = 0;
        for &(x, y) in pts.iter().take(100) {
            c.vector(ENTRY, x, y, z0)?;
            bytes += vector_bytes(x, y, z0);
        }
        thread::sleep(ctx.t(200)); // let the last bytes cross the wire (~150 ms at 115200)
        let pct = c.buffer_level_pct()?;
        if pct > 0 {
            ctx.r.info(
                "buffer size",
                format!(
                    "{bytes} bytes = {pct}% → about {} bytes",
                    bytes * 100 / usize::from(pct)
                ),
            );
        } else {
            ctx.r.info(
                "buffer size",
                format!("{bytes} bytes read as 0%: more than {} KB", bytes / 10),
            );
        }
        c.vector(ENTRY, x0, y0, z0)?;
        idle(&mut c)?;
        p = c.into_parameters()?;
        p.set(Setting::DeadTimeMs(100))?;
    }

    // (b) and (c) stream with flow control on.
    p.set(Setting::FlowControl(true))?;
    let mut c = p.into_cutting()?;
    let start = [x0, y0, z0];
    let mut reversed = false;
    if ctx.go(
        "Stream 5 laps of the circle (~22 KB) with flow control (#C7,1) and the host watchdog \
         (#C19,400). Head up.",
        "flow control",
    ) {
        let mut p = c.into_parameters()?;
        p.set(Setting::HostWatchdog(ctx.watchdog(400)))?;
        c = p.into_cutting()?;
        reversed = stream_with_flow_control(&mut c, ctx, &pts, start)?;
    }
    if reversed {
        ctx.r.skip(
            "@B on a held stream",
            "0x02/0x03 are the other way round: the driver's flow control must be fixed first",
        );
    } else if ctx.go(
        "Stream the circle again and brake (@B) the moment the controller holds the stream \
         (0x03), or once everything is sent: nothing may move afterwards. Head up.",
        "@B on a held stream",
    ) {
        brake_held_stream(&mut c, ctx, &pts, start)?;
    }
    Ok(c.into_parameters()?)
}

/// (b) The full stream with `#C7,1`: which flow byte comes first, and does the host watchdog
/// stay fed? Returns true if `0x02`/`0x03` turned out to be the other way round.
fn stream_with_flow_control(
    c: &mut Controller<mode::Cutting>,
    ctx: &mut Ctx,
    pts: &[(i32, i32)],
    start: [i32; 3],
) -> Res<bool> {
    let [x0, y0, z0] = start;
    ctx.drain();
    let mut flow = Vec::new();
    let mut reversed = false;
    let started = Instant::now();
    // Send in small chunks with pauses longer than the wire time, so a flow byte is seen
    // before more than its ~200-byte margin has been sent, whichever way round it is.
    for chunk in pts.chunks(8) {
        for &(x, y) in chunk {
            c.vector(ENTRY, x, y, z0)?;
        }
        thread::sleep(ctx.t(30).max(Duration::from_millis(3)));
        flow.extend(ctx.flow_signals());
        if flow.first() == Some(&Signal::BufferResume) {
            reversed = true;
            break; // 0x02 first: the prose on p.5 is right. Stop before overrunning.
        }
    }
    idle(c)?;
    flow.extend(ctx.flow_signals());
    let bytes: usize = pts.iter().map(|&(x, y)| vector_bytes(x, y, z0)).sum();
    ctx.r.info(
        "stream",
        format!(
            "{} vectors, {bytes} bytes in {:.1} s",
            pts.len(),
            started.elapsed().as_secs_f64()
        ),
    );
    if flow.is_empty() {
        ctx.r
            .info("flow bytes", "none: the buffer never got close to full");
    } else {
        ctx.r.check(
            !reversed,
            "0x03 = almost full, 0x02 = room (table p.7)",
            format!("seen in order: {}", flow_names(&flow)),
        );
    }
    if reversed {
        c.vector(ENTRY, x0, y0, z0)?;
        idle(c)?;
    }
    let end = pos(c)?;
    ctx.r.check(
        end == [x0, y0, z0],
        "long stream completes (no *068, no *022)",
        format!("ended at {end:?}"),
    );
    Ok(reversed)
}

/// (c) Brake a long stream while the controller is holding it, the way the runner's stop-now
/// will: through the stop lane, from another thread, the moment `0x03` arrives (or once
/// everything is sent, if the buffer never fills). Afterwards nothing may move.
///
/// The spec says `@B` stops the axes [SPEC1 p.14], not what becomes of path data already
/// received; and master commands are handled even while the stream is held [SPEC1 p.5, p.12],
/// so a brake could overtake queued vectors that then start afterwards. Stage 4 brakes a
/// four-vector path, which cannot show that. The path stays on the circle either way.
fn brake_held_stream(
    c: &mut Controller<mode::Cutting>,
    ctx: &mut Ctx,
    pts: &[(i32, i32)],
    start: [i32; 3],
) -> Res<()> {
    let [x0, y0, z0] = start;
    ctx.drain();
    let events = c.subscribe();
    let stop = c.stop_handle();
    let (sent_all, all_sent) = mpsc::channel::<()>();
    let braker = thread::spawn(move || {
        loop {
            // Dropped sender: everything went out without the stream being held.
            if !matches!(all_sent.try_recv(), Err(TryRecvError::Empty)) {
                return (false, stop.brake());
            }
            if let Ok(Event::Signal(Signal::BufferAlmostFull)) =
                events.recv_timeout(Duration::from_millis(2))
            {
                return (true, stop.brake());
            }
        }
    });
    let mut sent = 0;
    let mut failed = None;
    for &(x, y) in pts {
        match c.vector(ENTRY, x, y, z0) {
            Ok(()) => sent += 1,
            Err(ControllerError::Stopped) => break, // the brake landed
            Err(e) => {
                failed = Some(e);
                break;
            }
        }
    }
    drop(sent_all);
    let (held, braked) = braker.join().map_err(|_| "the brake thread panicked")?;
    braked?;
    if let Some(e) = failed {
        return Err(e.into());
    }
    c.clear_stop()?;
    idle(c)?;
    let stopped_at = pos(c)?;
    let level = c.buffer_level_pct()?;
    thread::sleep(ctx.t(3000)); // well past the dead time and many legs of the circle
    let s = c.status()?;
    let later = pos(c)?;
    // Everything since the start of the step. `clear_stop` has dropped faults that arrived
    // around the stop, so they are read from the events here.
    let seen: Vec<Event> = ctx.events.try_iter().collect();
    let flow: Vec<Signal> = seen
        .iter()
        .filter_map(|e| match e {
            Event::Signal(s @ (Signal::BufferAlmostFull | Signal::BufferResume)) => Some(*s),
            _ => None,
        })
        .collect();
    let faults: Vec<&str> = seen
        .iter()
        .filter_map(|e| match e {
            Event::Unsolicited(line) if line.starts_with('*') => Some(line.as_str()),
            _ => None,
        })
        .collect();
    ctx.r.check(
        faults.is_empty(),
        "no overrun streaming at full speed with #C7,1",
        if faults.is_empty() {
            "the controller's 0x03 came in time over this link".to_owned()
        } else {
            format!(
                "{}: 0x03 came too late over this link (adapter buffering?), so pace by @H as \
                 well (Config::stream_window_bytes)",
                faults.join(" ")
            )
        },
    );
    let when = if held {
        format!(
            "while the stream was held (0x03), after {sent} of {} vectors",
            pts.len()
        )
    } else {
        format!(
            "with all {} vectors sent; the stream was never held",
            pts.len()
        )
    };
    ctx.r.check(
        later == stopped_at && !s.moving,
        "@B discards queued path data",
        format!(
            "braked {when}; @H{level} after; {later:?} 3 s later, {s}; flow bytes: {}",
            flow_names(&flow)
        ),
    );
    if held && flow.last() == Some(&Signal::BufferAlmostFull) {
        ctx.r.fail(
            "0x02 after the brake",
            "none came: the stream stays held after @B, so the driver would wait for room \
             that never comes (reset the flow state on a stop)",
        );
        return Ok(()); // a return move would only wait out the flow timeout
    }
    idle(c)?; // if it did resume, let it finish before driving back
    c.vector(ENTRY, x0, y0, z0)?;
    idle(c)?;
    let end = pos(c)?;
    ctx.r.check(
        end == start,
        "stream usable after the brake",
        format!("back at {end:?}"),
    );
    Ok(())
}

/// `0x03 0x02 …`, or `none`.
fn flow_names(flow: &[Signal]) -> String {
    if flow.is_empty() {
        return "none".to_owned();
    }
    let names: Vec<&str> = flow
        .iter()
        .map(|s| match s {
            Signal::BufferAlmostFull => "0x03",
            _ => "0x02",
        })
        .collect();
    names.join(" ")
}

// ------------------------------------------------------------------ stage 7

fn stage7(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    if !require_homed(&mut p, ctx, "stage 7")? {
        return Ok(p);
    }
    setup_moves(&mut p, ctx)?;
    let (mut p, at_park) = to_park(p, ctx)?;
    if !at_park {
        return Ok(p);
    }
    let ix = inward()[0];

    if ctx.go(
        "Jog X 100 mm into the table and stop it with @S (no ramp: abrupt). The position is \
         lost, so the machine re-homes and returns to the park afterwards.",
        "@S",
    ) {
        let mut j = p.into_jog()?;
        j.jog_steps(ENTRY, Axis::X, JogTarget::By(reach() * ix))?;
        thread::sleep(ctx.t(800));
        ctx.stop.emergency_stop()?;
        thread::sleep(ctx.t(200));
        let s = j.clear_stop()?;
        ctx.r.check(
            !s.homed,
            "@S loses the position",
            format!("{s} after @S (flag 4 = home known)"),
        );
        p = home_all(j.into_parameters()?, ctx)?;
        setup_moves(&mut p, ctx)?;
        let (back, at_park) = to_park(p, ctx)?;
        p = back;
        if !at_park {
            return Ok(p);
        }
    }

    // The limit at the park's end of X: the near one.
    let switch = LimitSwitch {
        axis: Axis::X,
        side: if ix < 0 { Side::Plus } else { Side::Minus },
    };
    let (input, code) = (switch.input(), switch.error_code());
    let def = p.input_def(input)?;
    if !def.is_mapped() {
        ctx.r.skip(
            "limit switch",
            format!(
                "{switch} limit (I{input}) is not mapped [{def}]: the controller cannot see it, \
                 so the check will not drive towards it"
            ),
        );
        return Ok(p);
    }
    // Drive no further than just past the drivable range: a mapped input is not proof of a
    // working switch, and if it never trips the move must end on its own, not at the frame.
    let target = if switch.side == Side::Plus {
        x_drivable() + limit_overtravel()
    } else {
        -limit_overtravel()
    };
    let x = p.position_steps(Axis::X)?;
    let distance = u64::from((target - x).unsigned_abs());
    let prompt = format!(
        "Drive X slowly (10 mm/s) towards the {switch} limit switch until it trips: at most {} mm, \
         to X {} mm (20 mm past GMC's drivable range). Hand on the e-stop; Ctrl-C brakes.",
        (target - x).abs() / steps_per_mm(),
        target / steps_per_mm()
    );
    if !ctx.go(&prompt, "limit switch") {
        return Ok(p);
    }
    p.set(Setting::Speed {
        entry: ENTRY,
        value: LIMIT_SPEED,
    })?;
    let mut j = p.into_jog()?;
    j.jog_steps(ENTRY, Axis::X, JogTarget::To(target))?;
    // The bounded move's own duration, a quarter more for ramps and speed error, plus slack.
    let timeout = ctx.t(distance * 1000 / u64::from(LIMIT_SPEED) * 5 / 4) + Duration::from_secs(5);
    let result = j.wait_idle(timeout);
    if matches!(result, Err(ControllerError::WaitTimeout { .. })) {
        j.brake()?; // slower than it should be: do not leave it running
        idle(&mut j)?;
    }
    let hit = matches!(
        &result,
        Err(ControllerError::AsyncError { code: c }) | Err(ControllerError::Fault { code: Some(c), .. })
            if *c == code
    );
    ctx.r.check(
        hit,
        &format!("*{:03} from the {switch} limit", code.0),
        match &result {
            Ok(s) => format!("no fault: ended {s}"),
            Err(e) => e.to_string(),
        },
    );
    let sw = j.switches()?;
    ctx.r.check(
        sw.limit(switch),
        &format!("@Y shows the {switch} limit"),
        format!("{sw:?}"),
    );
    j.clear_error()?;
    j.jog_steps(ENTRY, Axis::X, JogTarget::By(step_20mm() * ix))?; // back into the table
    idle(&mut j)?;
    let sw = j.switches()?;
    ctx.r.check(
        !sw.any_limit(),
        "limit clears after moving off",
        format!("{sw:?}"),
    );
    Ok(j.into_parameters()?)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every stage's moves stay within the reach of the park, into the table, and stage 5's
    /// compact path goes 20 mm out in +X and +Y: all of it inside the soft limits.
    #[test]
    fn every_move_from_the_park_stays_inside_the_soft_limits() {
        let p = profile();
        let [px, py] = p.park_steps;
        let [ix, iy] = inward();
        let corners = [
            [px, py],
            [px + reach() * ix, py + reach() * iy],
            [px + step_20mm(), py + step_20mm()],
        ];
        for steps in corners {
            assert!(p.envelope_mm.contains(p.carriage_mm(steps)), "{steps:?}");
        }
        assert_eq!(
            inward(),
            [-1, 1],
            "the 601's park: far end of X, near home in Y"
        );
    }
}
