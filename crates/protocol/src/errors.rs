//! Controller error codes `*NNN` / `@E` [SPEC1 p.38–40], translated.
//!
//! `*069` is not in the spec; GMC treats it as a controller error [GMC].

use std::fmt;

use crate::Axis;
use crate::switches::{LimitSwitch, Side};

/// A controller error code, as sent in `*NNN` replies and by `@E` [SPEC1 p.38–40].
///
/// Any number can arrive (the firmware sends codes the spec does not list), so this wraps the
/// number as is. `Display` gives the wire form and its meaning: `*033 (limit switch X plus
/// reached)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ErrorCode(pub u16);

impl ErrorCode {
    /// `*007`: service code required. Also what a used-up start counter is expected to give
    /// (the spec does not name that error) [SPEC1 p.10].
    pub const SERVICE_CODE_REQUIRED: Self = Self(7);
    /// `*008`: service code invalid.
    pub const SERVICE_CODE_INVALID: Self = Self(8);
    /// `*028`: a mode change is only possible at standstill.
    pub const MODE_CHANGE_WHILE_MOVING: Self = Self(28);

    /// Meaning of the code, or `None` if the spec does not define it.
    pub fn describe(self) -> Option<&'static str> {
        Some(match self.0 {
            0 => "no error",
            1 => "command not valid in this mode",
            2 => "speed-table entry invalid",
            3 => "command format invalid",
            4 => "speed invalid",
            6 => "ramp invalid",
            7 => "service code required",
            8 => "service code invalid",
            10 => "braking angle invalid",
            12 => "axis invalid",
            13 => "resolution numerator invalid",
            15 => "resolution denominator invalid",
            17 => "backlash invalid",
            19 => "reference offset invalid",
            21 => "parameter id invalid",
            22 => "memory too small (buffer overrun)",
            23 => "parameter error in file",
            24 => "parameter exceeds limits",
            25 => "parameter file not found",
            26 => "parameter file not written",
            27 => "invalid mode",
            28 => "mode change only possible at standstill",
            29 => "master command unknown",
            30 => "machine is already moving",
            31 => "command invalid",
            32 => "limit switch X minus reached",
            33 => "limit switch X plus reached",
            34 => "limit switch Y minus reached",
            35 => "limit switch Y plus reached",
            36 => "limit switch Z minus reached",
            37 => "limit switch Z plus reached",
            38 => "limit switch U minus reached",
            39 => "limit switch U plus reached",
            40 => "stop signal detected",
            41 => "homing not performed",
            42 => "spindle-speed table entry invalid",
            43 => "spindle speed invalid",
            44 => "baud rate invalid",
            45 => "wait time invalid",
            46 => "signal states invalid",
            47 => "checksum error in file transfer",
            48 => "timeout receiving file",
            49 => "error creating file",
            50 => "error in download command",
            51 => "error renaming new file",
            52 => "error setting date/time",
            53 => "error deleting file",
            54 => "file not found",
            55 => "parameter value invalid",
            56 => "machine not ready",
            57 => "error in batch program",
            58 => "input NOBATCH active, mode 3 not possible",
            59 => "no batch program CNCCON.CMD found",
            60 => "parameter value too small",
            61 => "parameter value too large",
            62 => "machine is standing on a limit switch",
            63 => "homing not possible, machine is standing on a switch",
            64 => "no acknowledgement from host during file upload",
            65 => "error setting file attribute",
            66 => "input number of mode-2 I command invalid",
            67 => "value of mode-2 special function invalid",
            68 => "connection lost, aborted by node guarding (host watchdog)",
            _ => return None,
        })
    }

    /// The limit switch a `*032`–`*037` names (EENDM1 = X minus, EENDP1 = X plus, …).
    /// `None` for every other code, including `*038`/`*039` of a fourth axis the 601 lacks.
    pub fn limit_switch(self) -> Option<LimitSwitch> {
        let n = self.0.checked_sub(32)?;
        let axis = match n / 2 {
            0 => Axis::X,
            1 => Axis::Y,
            2 => Axis::Z,
            _ => return None,
        };
        let side = if n % 2 == 1 { Side::Plus } else { Side::Minus };
        Some(LimitSwitch { axis, side })
    }
}

impl fmt::Display for ErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let meaning = self.describe().unwrap_or("undocumented error");
        write!(f, "*{:03} ({meaning})", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_and_unknown() {
        assert_eq!(ErrorCode(21).describe(), Some("parameter id invalid"));
        assert_eq!(ErrorCode(7), ErrorCode::SERVICE_CODE_REQUIRED);
        assert!(
            ErrorCode(68)
                .describe()
                .is_some_and(|s| s.contains("watchdog"))
        );
        assert_eq!(ErrorCode(5).describe(), None, "*005 is blank in the spec");
        assert_eq!(ErrorCode(69).to_string(), "*069 (undocumented error)");
        assert_eq!(
            ErrorCode(33).to_string(),
            "*033 (limit switch X plus reached)"
        );
    }

    #[test]
    fn limit_switch_codes_round_trip() {
        for code in 32..=37 {
            let switch = ErrorCode(code).limit_switch().expect("a limit switch");
            assert_eq!(switch.error_code(), ErrorCode(code));
        }
        assert_eq!(
            ErrorCode(33).limit_switch(),
            Some(LimitSwitch {
                axis: Axis::X,
                side: Side::Plus
            })
        );
        assert_eq!(ErrorCode(38).limit_switch(), None, "U axis");
        assert_eq!(ErrorCode(40).limit_switch(), None);
    }
}
