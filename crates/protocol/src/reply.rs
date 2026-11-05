//! Classification of a received reply line, and parsers for typed value replies.

use crate::{Axis, Status};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reply {
    /// Bare CR: command accepted (modes 0, 1, 4).
    Ack,
    /// `*NNN`: command rejected or machine fault [SPEC p.37–38].
    Error(u16),
    /// `@X…` status.
    Status(Status),
    /// Any other value reply (`@V…`, `@LX…`, `@I221,0`, `&C13,1`, …), verbatim.
    Value(String),
}

impl Reply {
    pub fn parse(line: &str) -> Reply {
        if line.is_empty() {
            return Reply::Ack;
        }
        if let Some(code) = line.strip_prefix('*').and_then(|n| n.parse().ok()) {
            return Reply::Error(code);
        }
        if let Some(s) = Status::parse(line) {
            return Reply::Status(s);
        }
        Reply::Value(line.to_owned())
    }
}

/// `@LX-23450` → position of `axis` in steps [SPEC p.13].
pub fn parse_position(line: &str, axis: Axis) -> Option<i32> {
    let rest = line.strip_prefix("@L")?;
    let rest = rest.strip_prefix(axis.upper())?;
    rest.parse().ok()
}

/// `@I235,1` → state of input `n` [SPEC p.15].
pub fn parse_input(line: &str, n: u8) -> Option<bool> {
    parse_io(line, "@I", n)
}

/// `@Q250,0` → state of output `n` [SPEC1 p.16].
pub fn parse_output(line: &str, n: u8) -> Option<bool> {
    parse_io(line, "@Q", n)
}

fn parse_io(line: &str, prefix: &str, n: u8) -> Option<bool> {
    let rest = line.strip_prefix(prefix)?;
    let (num, val) = rest.split_once(',')?;
    if num.parse::<u8>().ok()? != n {
        return None;
    }
    match val {
        "0" => Some(false),
        "1" => Some(true),
        _ => None,
    }
}

/// `@M01` → mode number (two digits with leading zero) [SPEC1 p.13].
pub fn parse_mode(line: &str) -> Option<u8> {
    let digits = line.strip_prefix("@M")?;
    if digits.len() != 2 {
        return None;
    }
    digits.parse().ok()
}

/// `@N412` → mode-2 command counter [SPEC1 p.15].
pub fn parse_command_counter(line: &str) -> Option<u32> {
    line.strip_prefix("@N")?.parse().ok()
}

/// `@O120` → speed override in percent [SPEC1 p.16].
pub fn parse_override(line: &str) -> Option<u16> {
    line.strip_prefix("@O")?.parse().ok()
}

/// `@E23` → error code [SPEC p.13].
pub fn parse_error_code(line: &str) -> Option<u16> {
    line.strip_prefix("@E")?.parse().ok()
}

/// `@H86` → buffer fill level in percent [SPEC p.15].
pub fn parse_buffer_level(line: &str) -> Option<u8> {
    line.strip_prefix("@H")?.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifies() {
        assert_eq!(Reply::parse(""), Reply::Ack);
        assert_eq!(Reply::parse("*021"), Reply::Error(21));
        assert!(matches!(Reply::parse("@X04"), Reply::Status(_)));
        assert_eq!(
            Reply::parse("@V2.00/55W 19.10.10 2807001"),
            Reply::Value("@V2.00/55W 19.10.10 2807001".into())
        );
    }

    #[test]
    fn typed_values() {
        // Values from the captures (g_motions) and the spec examples.
        assert_eq!(parse_position("@LX44149", Axis::X), Some(44149));
        assert_eq!(parse_position("@LY-23450", Axis::Y), Some(-23450));
        assert_eq!(parse_position("@LY7958", Axis::X), None, "wrong axis");
        assert_eq!(parse_input("@I221,0", 221), Some(false));
        assert_eq!(parse_input("@I235,1", 235), Some(true));
        assert_eq!(parse_input("@I235,1", 236), None, "wrong input");
        assert_eq!(parse_error_code("@E0"), Some(0));
        assert_eq!(parse_error_code("@E33"), Some(33));
        assert_eq!(parse_buffer_level("@H86"), Some(86));
        assert_eq!(parse_output("@Q250,0", 250), Some(false));
        assert_eq!(parse_mode("@M01"), Some(1));
        assert_eq!(parse_mode("@M1"), None, "two digits with leading zero");
        assert_eq!(parse_command_counter("@N412"), Some(412));
        assert_eq!(parse_override("@O120"), Some(120));
    }
}
