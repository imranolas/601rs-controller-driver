//! `gunnar check`: staged hardware checks, each one confirmed by the operator first.
//!
//! | Stage | Moves | Answers |
//! |-------|-------|---------|
//! | 2 | no  | `0x05` heartbeat (`#C8`), green button edges `0x06`/`0x07` (`#C35`) |
//! | 3 | yes | homing like GMC, `0x04`, jogs and `0x01`, `@B`, `@Q` vs `$A`, outputs across a mode switch, clamp sides |
//! | 4 | yes | a mode-2 square the way GMC does it: `@X` 15 → 14 → 04, exact return; `@B` mid-path discards the rest |
//! | 5 | yes | v1.00 in-stream `A`/`W`/`I`/`C`, `@N` progress, `C1` vs dead time, `@O` with `C8`/`C16` |
//! | 6 | yes | buffer size via `@H`; which of `0x02`/`0x03` means "full"; `#C19` watchdog during a long stream |
//! | 7 | yes | `@S` loses the position; a limit switch arrives as an async `*03x` and shows in `@Y` (the drive towards it ends 20 mm past GMC's drivable range) |
//!
//! Safety rules: the head stays up (outputs `0x02`; only the spare bit `0x80` and, after
//! asking, the clamps are switched). Moves are small and go *away* from the home switches
//! (direction from `&D`). Nothing is saved (`#W` is never sent); the parameters a stage
//! changes are read first and written back at the end. Ctrl-C brakes (`@B`) and exits.

use std::sync::mpsc::Receiver;
use std::thread;
use std::time::{Duration, Instant};

use controller::{
    Axis, Controller, ControllerError, Event, JogTarget, Level, LimitSwitch, ModeMarker,
    OverridePct, Setting, SettingKey, Side, Signal, SignalAcks, SpeedOverride, StopHandle,
    StreamFlags, Units, WatchdogMs, mode,
};
use protocol::Command;

use crate::operator::{Confirm, Operator};
use crate::report::Report;
use crate::target::Res;

/// Speed-table entry used for all test moves (GMC's choice too).
const ENTRY: u8 = 20;
/// X/Y scale [GCF `MachineScale` 400 /cm].
const STEPS_PER_MM: i32 = 40;
const STEP_20MM: i32 = 20 * STEPS_PER_MM;
/// End of the X drivable range in steps: GMC's `DRX` 156 cm × 400 steps/cm [GCF], its own
/// soft-limit basis. A fitted X+ limit switch sits at or just beyond it.
const X_DRIVABLE: i32 = 62_400;
/// How far past the drivable range stage 7 may drive while looking for the limit switch.
const LIMIT_OVERTRAVEL: i32 = STEP_20MM;
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
}

impl Ctx {
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

/// +1 if the axis homes towards minus (so plus moves away from the switch), −1 if it homes
/// towards plus. A `&D` reply that says neither is an error, never a guess: a wrong direction
/// could drive the axis into its home switch.
fn away(p: &mut Params, axis: Axis) -> Res<i32> {
    Ok(match p.homing_side(axis)? {
        Side::Minus => 1,
        Side::Plus => -1,
    })
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
            2 => "Stage 2: signals (no motion)",
            3 => "Stage 3: homing, jogs, brake, outputs",
            4 => "Stage 4: mode-2 square, GMC style",
            5 => "Stage 5: v1.00 in-stream commands",
            6 => "Stage 6: buffer, flow control, watchdog",
            7 => "Stage 7: faults (@S, limit switch)",
            _ => continue,
        };
        ctx.r.heading(title);
        match ctx.op.confirm(&format!("{title}. Start?")) {
            Confirm::Go => {}
            Confirm::Skip => {
                ctx.r.skip(title, "skipped by the operator");
                continue;
            }
            Confirm::Quit => break,
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
            Ok(back) => p = back,
            Err(e) => {
                let _ = ctx.stop.brake();
                ctx.r.fail(title, format!("aborted: {e}"));
                println!(
                    "\n  Braked (@B). Parameters changed by this run were NOT restored. Nothing \
                     was saved, so power-cycling the controller (or starting GMC) restores them."
                );
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
                "NOT restored, power-cycle the controller (or start GMC) to reset them: {}",
                failed.join("; ")
            ),
        );
    }
}

// ------------------------------------------------------------------ shared steps

/// Home Z, then X and Y, with GMC's settings [CAP g_init]:
/// `@@ #G10,1750 #OZ,0 $Hz` then `#G10,1750 #G11,750 #OX,300 #OY,300 $Hxy`.
fn home_all(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
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
    let mut j = p.into_jog()?;
    j.home(&[Axis::X, Axis::Y])?;
    idle(&mut j)?;
    let s = j.status()?;
    let at = pos(&mut j)?;
    ctx.r
        .check(s.homed, "homed", format!("{s} after $Hxy, at {at:?} steps"));
    Ok(j.into_parameters()?)
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

// ------------------------------------------------------------------ stage 3

fn stage3(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    setup_moves(&mut p, ctx)?;
    let mut p = match ctx
        .op
        .confirm("Home Z, then X and Y, exactly as GMC does (about a minute). Clear the table.")
    {
        Confirm::Go => home_all(p, ctx)?,
        _ => {
            ctx.r.skip("homing", "skipped by the operator");
            p
        }
    };
    if !require_homed(&mut p, ctx, "jogs")? {
        return Ok(p);
    }
    let dirs = [
        (Axis::X, away(&mut p, Axis::X)?),
        (Axis::Y, away(&mut p, Axis::Y)?),
    ];
    let mut j = p.into_jog()?;

    for (axis, dir) in dirs {
        if ctx
            .op
            .confirm(&format!("Jog {axis:?} 20 mm away from home and back."))
            != Confirm::Go
        {
            ctx.r.skip(&format!("jog {axis:?}"), "skipped");
            continue;
        }
        let i = axis.index();
        let start = pos(&mut j)?;
        ctx.drain();
        j.jog_steps(ENTRY, axis, JogTarget::By(STEP_20MM * dir))?;
        idle(&mut j)?;
        let end = pos(&mut j)?;
        let positioned = ctx.signals().contains(&Signal::Positioned);
        ctx.r.check(
            end[i] - start[i] == STEP_20MM * dir,
            &format!("jog {axis:?}"),
            format!(
                "{} → {} steps (asked {:+})",
                start[i],
                end[i],
                STEP_20MM * dir
            ),
        );
        ctx.r.check(
            positioned,
            &format!("0x01 after jog {axis:?}"),
            "positioned signal (#C5,1)",
        );
        j.jog_steps(ENTRY, axis, JogTarget::To(start[i]))?;
        idle(&mut j)?;
    }

    let x_dir = dirs[0].1;
    if ctx
        .op
        .confirm("Jog X 100 mm away from home and brake (@B) after a moment.")
        == Confirm::Go
    {
        let start = pos(&mut j)?;
        j.jog_steps(ENTRY, Axis::X, JogTarget::By(5 * STEP_20MM * x_dir))?;
        thread::sleep(ctx.t(800));
        j.brake()?;
        idle(&mut j)?;
        let end = pos(&mut j)?;
        let moved = (end[0] - start[0]) * x_dir;
        ctx.r.check(
            moved > 0 && moved < 5 * STEP_20MM,
            "@B brakes a jog",
            format!("stopped after {moved} of {} steps", 5 * STEP_20MM),
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

    if ctx
        .op
        .confirm("Close each clamp in turn (outputs 0x20, then 0x40). Hands clear.")
        == Confirm::Go
    {
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
    if ctx
        .op
        .confirm("Drive a 20 mm square in mode 2, head up, and return.")
        != Confirm::Go
    {
        ctx.r.skip("square", "skipped");
        return Ok(p);
    }
    setup_moves(&mut p, ctx)?;
    let (dx, dy) = (
        STEP_20MM * away(&mut p, Axis::X)?,
        STEP_20MM * away(&mut p, Axis::Y)?,
    );
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
    if ctx.op.confirm(
        "Drive the square again and brake it (@B) during the first leg: the rest of the path \
         must be discarded, not resumed. Head up.",
    ) == Confirm::Go
    {
        brake_mid_path(&mut c, ctx, [x0, y0, z0], (dx, dy))?;
    } else {
        ctx.r.skip("@B mid-path", "skipped");
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

fn timed_move(c: &mut Controller<mode::Cutting>, x: i32, y: i32, z: i32) -> Res<Duration> {
    let start = Instant::now();
    c.vector(ENTRY, x, y, z)?;
    idle(c)?;
    Ok(start.elapsed())
}

fn stage5(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    if !require_homed(&mut p, ctx, "stage 5")? {
        return Ok(p);
    }
    if ctx
        .op
        .confirm("Run v1.00 mode-2 features: short moves away from home, head up.")
        != Confirm::Go
    {
        ctx.r.skip("v1.00", "skipped");
        return Ok(p);
    }
    setup_moves(&mut p, ctx)?;
    let d = STEP_20MM * away(&mut p, Axis::X)?;
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
    let q = c.read_output(249)?;
    let at = pos(&mut c)?;
    ctx.r.check(
        s.waiting,
        "W dwell reports waiting",
        format!("{s} during W1000"),
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
    let t_off = timed_move(&mut c, x0 + d2, y0, z0)?;
    c.flags(StreamFlags {
        speed_override: Some(SpeedOverride::On),
        ..StreamFlags::default()
    })?;
    let t_on = timed_move(&mut c, x0, y0, z0)?;
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
    Ok(c.into_parameters()?)
}

// ------------------------------------------------------------------ stage 6

/// Five laps of a 15 mm-radius circle in the square away from home, as 1250 short vectors
/// (~22 KB): ends where it starts.
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
    let (sx, sy) = (away(&mut p, Axis::X)?, away(&mut p, Axis::Y)?);
    let [x0, y0, z0] = pos(&mut p)?;
    let pts = circle(x0, y0, sx, sy);
    p.set(Setting::HostWatchdog(ctx.watchdog(400)))?;

    // (a) Buffer size: fill it during a long dead time (nothing is consumed), then ask @H.
    if ctx
        .op
        .confirm("Measure the buffer: 100 short moves on a 30 mm circle, then return.")
        == Confirm::Go
    {
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

    // (b) Full stream with #C7,1: which byte comes first, and does the watchdog stay fed?
    if ctx.op.confirm(
        "Stream 5 laps of the circle (~22 KB) with flow control (#C7,1) and the host watchdog \
         (#C19,400). Head up.",
    ) != Confirm::Go
    {
        ctx.r.skip("flow control", "skipped");
        return Ok(p);
    }
    p.set(Setting::FlowControl(true))?;
    let mut c = p.into_cutting()?;
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
        for s in ctx.signals() {
            if matches!(s, Signal::BufferAlmostFull | Signal::BufferResume) {
                flow.push(s);
            }
        }
        if flow.first() == Some(&Signal::BufferResume) {
            reversed = true;
            break; // 0x02 first: the prose on p.5 is right. Stop before overrunning.
        }
    }
    idle(&mut c)?;
    for s in ctx.signals() {
        if matches!(s, Signal::BufferAlmostFull | Signal::BufferResume) {
            flow.push(s);
        }
    }
    let bytes: usize = pts.iter().map(|&(x, y)| vector_bytes(x, y, z0)).sum();
    ctx.r.info(
        "stream",
        format!(
            "{} vectors, {bytes} bytes in {:.1} s",
            pts.len(),
            started.elapsed().as_secs_f64()
        ),
    );
    let names: Vec<&str> = flow
        .iter()
        .map(|s| {
            if *s == Signal::BufferAlmostFull {
                "0x03"
            } else {
                "0x02"
            }
        })
        .collect();
    if flow.is_empty() {
        ctx.r
            .info("flow bytes", "none: the buffer never got close to full");
    } else {
        ctx.r.check(
            !reversed,
            "0x03 = almost full, 0x02 = room (table p.7)",
            format!("seen in order: {}", names.join(" ")),
        );
    }
    if reversed {
        c.vector(ENTRY, x0, y0, z0)?;
        idle(&mut c)?;
    }
    let end = pos(&mut c)?;
    ctx.r.check(
        end == [x0, y0, z0],
        "long stream completes (no *068, no *022)",
        format!("ended at {end:?}"),
    );
    Ok(c.into_parameters()?)
}

// ------------------------------------------------------------------ stage 7

fn stage7(mut p: Params, ctx: &mut Ctx) -> Res<Params> {
    if !require_homed(&mut p, ctx, "stage 7")? {
        return Ok(p);
    }
    setup_moves(&mut p, ctx)?;
    let dir = away(&mut p, Axis::X)?;

    if ctx.op.confirm(
        "Jog X away from home and stop it with @S (no ramp: abrupt). The position is lost, so \
         the machine re-homes afterwards.",
    ) == Confirm::Go
    {
        let mut j = p.into_jog()?;
        j.jog_steps(ENTRY, Axis::X, JogTarget::By(5 * STEP_20MM * dir))?;
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
    }

    let switch = LimitSwitch {
        axis: Axis::X,
        side: if dir > 0 { Side::Plus } else { Side::Minus },
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
    let target = if dir > 0 {
        X_DRIVABLE + LIMIT_OVERTRAVEL
    } else {
        -LIMIT_OVERTRAVEL
    };
    let x = p.position_steps(Axis::X)?;
    let distance = u64::from((target - x).unsigned_abs());
    if ctx.op.confirm(&format!(
        "Drive X slowly (10 mm/s) towards the {switch} limit switch until it trips: at most {} mm, \
         to X {} mm (20 mm past GMC's drivable range). Hand on the e-stop; Ctrl-C brakes.",
        (target - x).abs() / STEPS_PER_MM,
        target / STEPS_PER_MM
    )) != Confirm::Go
    {
        ctx.r.skip("limit switch", "skipped");
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
    j.jog_steps(ENTRY, Axis::X, JogTarget::By(-STEP_20MM * dir))?;
    idle(&mut j)?;
    let sw = j.switches()?;
    ctx.r.check(
        !sw.any_limit(),
        "limit clears after moving off",
        format!("{sw:?}"),
    );
    Ok(j.into_parameters()?)
}
