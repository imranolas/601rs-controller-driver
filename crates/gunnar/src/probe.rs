//! `gunnar probe`: read-only questions to the controller. Nothing moves and no parameter is
//! written. The one state change is a mode switch to 1 and straight back to 0, which answers
//! whether the controller accepts mode changes without GMC's licence exchange.

use std::fmt::Debug;

use controller::{
    Axis, Controller, ControllerError, ErrorCode, OverridePct, Param, RESYNC_PROBE_INPUTS, mode,
};

use crate::report::Report;
use crate::target::Res;

/// Every parameter GMC writes once at connect and then relies on [CAP g_init]. GMC never saves
/// them (`#W`), so after a power cycle the box runs on its own values; where those differ, a
/// driver that does not write these too runs the machine differently from GMC.
const GMC_AT_CONNECT: &[(Param, &str)] = &[
    (Param::Config(2), "0"),
    (Param::Config(5), "1"),
    (Param::Config(11), "0"),
    (Param::Config(12), "1"),
    (Param::Config(13), "1"),
    (Param::Config(14), "13"),
    (Param::Config(16), "4"),
    (Param::Config(31), "105"),
    (Param::Config(32), "109"),
    (Param::Config(33), "113"),
    (Param::Config(34), "123"),
    (Param::Config(35), "1"),
    (Param::Config(37), "1"), // accepted [CAP g_init], but not in the spec's query table
    (Param::StepsPerRev(Axis::Z), "3000"),
    (Param::MicrometresPerRev(Axis::Z), "10000"),
    (Param::MaxSpeed, "36000"),
];

/// Every parameter GMC writes before each use (a move, a cut, homing), with the values seen in
/// the captures [CAP g_*]. A driver has to write these itself whatever the box boots with, so
/// the probe only records them.
const GMC_PER_USE: &[(Param, &str)] = &[
    (Param::Config(1), "0–200, the dead time per move"),
    (Param::Config(20), "400"),
    (Param::Config(21), "5000 / 12000"),
    (Param::Config(24), "30"),
    (Param::Config(25), "100"),
    (Param::Config(26), "5"),
    (Param::Config(27), "5"),
    (Param::Config(28), "1"),
    (Param::Speed(20), "3600–36000 as the feed"),
    (Param::Ramp, "100–700"),
    (Param::StartSpeed, "400–4000"),
    (Param::BrakeAngle(20), "90"),
    (Param::Speed(10), "1750 / 3000 for homing"),
    (Param::Speed(11), "750 for homing"),
    (Param::HomeOffset(Axis::X), "300"),
    (Param::HomeOffset(Axis::Y), "300"),
    (Param::HomeOffset(Axis::Z), "0"),
];

/// Parameters GMC never writes but the driver or the checks depend on.
const DRIVER_READS: &[Param] = &[
    Param::Config(4),
    Param::Config(6),
    Param::Config(7),
    Param::Config(8),
    Param::Config(19),
    Param::Homing(Axis::X),
    Param::Homing(Axis::Y),
    Param::Homing(Axis::Z),
];

/// Logical inputs: name, number [SPEC1 p.11].
const INPUTS: &[(&str, u8)] = &[
    ("ISIGNAL1 (green button)", 221),
    ("ISIGNAL2", 222),
    ("IREF1 (X home)", 235),
    ("IREF2 (Y home)", 236),
    ("IREF3 (Z home)", 237),
    ("IENDM1 (X- limit)", 239),
    ("IENDM2 (Y- limit)", 240),
    ("IENDM3 (Z- limit)", 241),
    ("IENDP1 (X+ limit)", 243),
    ("IENDP2 (Y+ limit)", 244),
    ("IENDP3 (Z+ limit)", 245),
    ("IBEREIT (machine ready)", 247),
    ("ISTOP (stop)", 254),
    ("ISTART (start)", 255),
];

/// A v1.00 feature: pass if the firmware answers it, fail with the error if not.
fn feature<T: Debug>(r: &mut Report, name: &str, result: controller::Result<T>) -> Option<T> {
    match result {
        Ok(v) => {
            r.pass(name, format!("{v:?}"));
            Some(v)
        }
        Err(e) => {
            r.fail(name, format!("not supported or failed: {e}"));
            None
        }
    }
}

pub fn run(mut p: Controller<mode::Parameters>, version: &str, r: &mut Report) -> Res<()> {
    r.heading("Identity and state");
    r.info("version (@V)", version);
    if !version.contains("2.00/55W") {
        r.fail(
            "firmware",
            "expected 2.00/55W (the captures were made with it)",
        );
    }
    let status = p.status()?;
    r.info("status (@X)", format!("{status} = {status:?}"));
    match p.error_code() {
        Ok(code) => r.info("error code (@E)", code.to_string()),
        Err(e) => r.fail("error code (@E)", e.to_string()),
    }
    for axis in Axis::ALL {
        match p.position_steps(axis) {
            Ok(v) => r.info(&format!("position {axis:?} (@L)"), format!("{v} steps")),
            Err(e) => r.fail(&format!("position {axis:?} (@L)"), e.to_string()),
        }
    }

    // `connect` has just proved `@I100`; every later resync takes the next input in turn, and
    // one the firmware refuses would make a resync fail in the middle of a job.
    r.heading("Resync probe inputs");
    let refused: Vec<String> = RESYNC_PROBE_INPUTS
        .filter_map(|n| p.read_input(n).err().map(|e| format!("@I{n}: {e}")))
        .collect();
    let first = RESYNC_PROBE_INPUTS.start;
    let last = RESYNC_PROBE_INPUTS.end - 1;
    if refused.is_empty() {
        r.pass(
            &format!("@I{first}–@I{last}"),
            "all answered: every resync can pair replies",
        );
    } else {
        r.fail(&format!("@I{first}–@I{last}"), refused.join("; "));
    }

    r.heading("v1.00 master commands");
    feature(r, "switches (@Y)", p.switches());
    feature(r, "buffer level (@H)", p.buffer_level_pct());
    feature(r, "command counter (@N)", p.command_counter());
    feature(r, "override (@O)", p.override_pct().map(OverridePct::get));
    let outputs: Vec<(u8, controller::Result<bool>)> =
        (242..=249).map(|q| (q, p.read_output(q))).collect();
    let detail: Vec<String> = outputs
        .iter()
        .map(|(q, level)| match level {
            Ok(on) => format!("Q{q}={}", u8::from(*on)),
            Err(e) => format!("Q{q}: {e}"),
        })
        .collect();
    r.check(
        outputs.iter().all(|(_, level)| level.is_ok()),
        "outputs (@Q242–249)",
        detail.join(" "),
    );

    r.heading("Inputs: level (@I) and mapping (&I)");
    let mut limits_mapped = Vec::new();
    for &(name, n) in INPUTS {
        let level = p
            .read_input(n)
            .map_or_else(|e| format!("({e})"), |on| u8::from(on).to_string());
        let def = p.input_def(n);
        let mapped = def.as_ref().is_ok_and(|d| d.is_mapped());
        if (239..=246).contains(&n) && mapped {
            limits_mapped.push(name);
        }
        let state = if mapped { "mapped" } else { "NOT mapped" };
        let def = def.map_or_else(|e| format!("({e})"), |d| d.to_string());
        r.info(
            &format!("I{n} {name}"),
            format!("level {level}, {state} [{def}]"),
        );
    }
    if limits_mapped.is_empty() {
        r.info(
            "limit switches",
            "none mapped: the controller cannot see any limit; stage 7 will not drive into one",
        );
    } else {
        r.info(
            "limit switches",
            format!("mapped: {}", limits_mapped.join(", ")),
        );
    }

    r.heading("Parameters GMC writes at connect: does the box boot with the same?");
    let mut differ = Vec::new();
    let mut unreadable = Vec::new();
    for &(param, gmc) in GMC_AT_CONNECT {
        let name = param.to_string();
        match p.query_param(param) {
            Ok(fields) if fields.join(",") == gmc => r.info(&name, format!("{gmc} (same as GMC)")),
            Ok(fields) => {
                r.info(&name, format!("{} (GMC sets {gmc})", fields.join(",")));
                differ.push(name);
            }
            Err(e) => {
                r.info(&name, format!("cannot be read back: {e}"));
                unreadable.push(name);
            }
        }
    }
    let total = GMC_AT_CONNECT.len();
    let mut verdict = if differ.is_empty() {
        format!(
            "all {} readable values match GMC's",
            total - unreadable.len()
        )
    } else {
        format!(
            "{} of {total} differ ({}): a driver must write them at connect, as GMC does",
            differ.len(),
            differ.join(" ")
        )
    };
    if !unreadable.is_empty() {
        verdict += &format!(
            "; {} could not be read back ({})",
            unreadable.len(),
            unreadable.join(" ")
        );
    }
    r.info("box vs GMC at connect", verdict);

    r.heading("Parameters GMC writes before each use (a driver must write them too)");
    for &(param, gmc) in GMC_PER_USE {
        let value = p
            .query_param(param)
            .map_or_else(|e| format!("cannot be read back: {e}"), |f| f.join(","));
        r.info(&param.to_string(), format!("{value} (GMC writes {gmc})"));
    }

    r.heading("Parameters the driver relies on");
    for &param in DRIVER_READS {
        let value = p
            .query_param(param)
            .map_or_else(|e| format!("cannot be read back: {e}"), |f| f.join(","));
        r.info(&param.to_string(), value);
    }

    r.heading("Mode switch without GMC's licence exchange");
    match p.into_jog() {
        Ok(jog) => {
            r.pass(
                "@M1 accepted",
                "the controller does not need GMC's mode-9 exchange",
            );
            jog.into_parameters()?;
        }
        Err(e) => match e.error {
            ControllerError::Rejected {
                code: code @ (ErrorCode::SERVICE_CODE_REQUIRED | ErrorCode::SERVICE_CODE_INVALID),
                ..
            } => r.fail(
                "@M1 refused",
                format!(
                    "{code}: service code / start counter. Workaround: start GMC once after \
                     power-on, close it, then connect"
                ),
            ),
            ControllerError::Rejected {
                code: code @ ErrorCode::MODE_CHANGE_WHILE_MOVING,
                ..
            } => {
                r.fail(
                    "@M1 refused",
                    format!("{code}: the machine is moving; stop it and probe again"),
                );
            }
            other => r.fail("@M1 refused", other.to_string()),
        },
    }
    Ok(())
}
