//! Controller error codes `*NNN` / `@E` [SPEC1 p.38–40], translated.
//!
//! `*069` is not in the spec; GMC treats it as a controller error [GMC].

/// Human-readable meaning of an error code, or `None` if the spec does not define it.
pub fn describe(code: u16) -> Option<&'static str> {
    Some(match code {
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

/// [`describe`], or a placeholder for codes the spec does not define.
pub fn describe_or_unknown(code: u16) -> &'static str {
    describe(code).unwrap_or("undocumented error")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_and_unknown() {
        assert_eq!(describe(21), Some("parameter id invalid"));
        assert_eq!(describe(7), Some("service code required"));
        assert_eq!(describe(68).map(|s| s.contains("watchdog")), Some(true));
        assert_eq!(describe(5), None, "*005 is blank in the spec");
        assert_eq!(describe_or_unknown(69), "undocumented error");
    }
}
