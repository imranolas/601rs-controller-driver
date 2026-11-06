//! `gunnar probe`: read-only questions to the controller. Nothing moves and no parameter is
//! written. The one state change is a mode switch to 1 and straight back to 0, which answers
//! whether the controller accepts mode changes without GMC's licence exchange.

use std::fmt::Debug;

use controller::{
    Axis, Controller, ControllerError, ErrorCode, OverridePct, Param, RESYNC_PROBE_INPUTS, mode,
};

use crate::report::Report;
use crate::target::Res;

/// What GMC writes at init [CAP g_init]. The probe reports where the box differs, i.e. what
/// it boots with versus what GMC assumes.
const GMC_INIT: &[(Param, &str)] = &[
    (Param::Config(2), "0"),
    (Param::Config(5), "1"),
    (Param::Config(11), "0"),
    (Param::Config(12), "1"),
    (Param::Config(13), "1"),
    (Param::Config(14), "13"),
    (Param::Config(35), "1"),
    (Param::Config(20), "400"),
    (Param::Config(21), "12000"),
    (Param::Speed(20), "36000"),
    (Param::Ramp, "300"),
    (Param::MaxSpeed, "36000"),
    (Param::StartSpeed, "4000"),
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

    r.heading("Parameters (&…) against GMC's init values");
    for &(param, gmc) in GMC_INIT {
        match p.query_param(param) {
            Ok(v) => {
                let v = v.join(",");
                let note = if v == gmc {
                    "same as GMC".to_owned()
                } else {
                    format!("GMC sets {gmc}")
                };
                r.info(&param.to_string(), format!("{v} ({note})"));
            }
            Err(e) => r.fail(&param.to_string(), e.to_string()),
        }
    }
    for param in [
        Param::Config(1),
        Param::Config(4),
        Param::Config(6),
        Param::Config(7),
        Param::Config(8),
        Param::Config(19),
        Param::Speed(10),
        Param::Speed(11),
        Param::HomeOffset(Axis::X),
        Param::HomeOffset(Axis::Y),
        Param::HomeOffset(Axis::Z),
        Param::Homing(Axis::X),
        Param::Homing(Axis::Y),
        Param::Homing(Axis::Z),
    ] {
        match p.query_param(param) {
            Ok(v) => r.info(&param.to_string(), v.join(",")),
            Err(e) => r.fail(&param.to_string(), e.to_string()),
        }
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
