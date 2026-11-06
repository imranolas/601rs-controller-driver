//! Classification of a received reply line, and parsers for typed value replies.

use std::fmt;

use crate::{Axis, ErrorCode, OverridePct, Param, Side, Status};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reply {
    /// Bare CR: command accepted (modes 0, 1, 4).
    Ack,
    /// `*NNN`: command rejected or machine fault [SPEC p.37–38].
    Error(ErrorCode),
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
            return Reply::Error(ErrorCode(code));
        }
        if let Some(s) = Status::parse(line) {
            return Reply::Status(s);
        }
        Reply::Value(line.to_owned())
    }
}

impl fmt::Display for Reply {
    /// The reply as it was on the wire (an error with its meaning).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reply::Ack => f.write_str("bare CR (acknowledgement)"),
            Reply::Error(code) => code.fmt(f),
            Reply::Status(s) => s.fmt(f),
            Reply::Value(v) => f.write_str(v),
        }
    }
}

/// The definition of a logical input: which hardware port bit drives it [SPEC1 p.28].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InputDef {
    /// Port `000h`: nothing drives the input, so the controller cannot see it.
    Unmapped,
    Mapped {
        port: u16,
        bit: u8,
        inverted: bool,
    },
}

impl InputDef {
    pub const fn is_mapped(self) -> bool {
        matches!(self, InputDef::Mapped { .. })
    }
}

impl fmt::Display for InputDef {
    /// The wire notation: `279h,7,1`, or `000h (not mapped)`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            InputDef::Unmapped => f.write_str("000h (not mapped)"),
            InputDef::Mapped {
                port,
                bit,
                inverted,
            } => write!(f, "{port:03X}h,{bit},{}", u8::from(inverted)),
        }
    }
}

/// `0` / `1` → bool; anything else is not a flag.
pub(crate) fn parse_flag(s: &str) -> Option<bool> {
    match s {
        "0" => Some(false),
        "1" => Some(true),
        _ => None,
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
    parse_flag(val)
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

/// `@O120` → speed override [SPEC1 p.16]. `None` outside 10–200 %, too.
pub fn parse_override(line: &str) -> Option<OverridePct> {
    OverridePct::new(line.strip_prefix("@O")?.parse().ok()?)
}

/// `@E23` → error code [SPEC p.13].
pub fn parse_error_code(line: &str) -> Option<ErrorCode> {
    line.strip_prefix("@E")?.parse().ok().map(ErrorCode)
}

/// `@H86` → buffer fill level in percent [SPEC p.15].
pub fn parse_buffer_level(line: &str) -> Option<u8> {
    line.strip_prefix("@H")?.parse().ok()
}

/// `&C13,1` / `&R300` / `&DX,0,10,11,0` / `&OX, 150` → the fields after the query prefix,
/// trimmed. `None` if the line does not answer `param`.
pub fn parse_param(line: &str, param: Param) -> Option<Vec<String>> {
    let prefix = param.to_string();
    let rest = line.strip_prefix(prefix.as_str())?;
    // `&C1` must not accept the reply to `&C13`: a bare value must not start with a digit
    // that continues the number, so numbered queries need a comma.
    let numbered = matches!(
        param,
        Param::Config(_) | Param::Speed(_) | Param::InputDef(_)
    );
    let rest = match rest.strip_prefix(',') {
        Some(fields) => fields,
        None if numbered => return None,
        None => rest,
    };
    Some(rest.split(',').map(|f| f.trim().to_owned()).collect())
}

/// `&I240,000h,0,0` → the definition of input `n` [SPEC1 p.28; CAP g_init].
pub fn parse_input_def(line: &str, n: u8) -> Option<InputDef> {
    let fields = parse_param(line, Param::InputDef(n))?;
    let [port, bit, inverted] = fields.as_slice() else {
        return None;
    };
    let port = u16::from_str_radix(port.strip_suffix(['h', 'H'])?, 16).ok()?;
    if port == 0 {
        return Some(InputDef::Unmapped);
    }
    Some(InputDef::Mapped {
        port,
        bit: bit.parse().ok()?,
        inverted: parse_flag(inverted)?,
    })
}

/// `&DX,0,10,11,0` → the end `axis` homes towards: direction 0 = minus, 1 = plus
/// [SPEC1 p.29–30]. Only the direction is read; anything else there is `None`, not a guess.
pub fn parse_homing_side(line: &str, axis: Axis) -> Option<Side> {
    let fields = parse_param(line, Param::Homing(axis))?;
    match fields.first().map(String::as_str) {
        Some("0") => Some(Side::Minus),
        Some("1") => Some(Side::Plus),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifies() {
        assert_eq!(Reply::parse(""), Reply::Ack);
        assert_eq!(Reply::parse("*021"), Reply::Error(ErrorCode(21)));
        assert!(matches!(Reply::parse("@X04"), Reply::Status(_)));
        assert_eq!(
            Reply::parse("@V2.00/55W 19.10.10 2807001"),
            Reply::Value("@V2.00/55W 19.10.10 2807001".into())
        );
        assert_eq!(
            Reply::parse("*021").to_string(),
            "*021 (parameter id invalid)"
        );
        assert_eq!(Reply::parse("@X14").to_string(), "@X14");
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
        assert_eq!(parse_error_code("@E0"), Some(ErrorCode(0)));
        assert_eq!(parse_error_code("@E33"), Some(ErrorCode(33)));
        assert_eq!(parse_buffer_level("@H86"), Some(86));
        assert_eq!(parse_output("@Q250,0", 250), Some(false));
        assert_eq!(parse_mode("@M01"), Some(1));
        assert_eq!(parse_mode("@M1"), None, "two digits with leading zero");
        assert_eq!(parse_command_counter("@N412"), Some(412));
        assert_eq!(parse_override("@O120"), OverridePct::new(120));
        assert_eq!(parse_override("@O5"), None, "out of range");
    }

    #[test]
    fn parameter_replies() {
        let f = |l: &str, p| parse_param(l, p);
        assert_eq!(f("&C13,1", Param::Config(13)), Some(vec!["1".into()]));
        assert_eq!(f("&C13,1", Param::Config(1)), None, "&C1 vs &C13");
        assert_eq!(f("&R300", Param::Ramp), Some(vec!["300".into()]));
        assert_eq!(
            f("&OX, 150", Param::HomeOffset(Axis::X)),
            Some(vec!["150".into()])
        );
        assert_eq!(
            f("&DX,0,34,35,36", Param::Homing(Axis::X)),
            Some(vec!["0".into(), "34".into(), "35".into(), "36".into()])
        );
        assert_eq!(
            f("&I240,000h,0,0", Param::InputDef(240)),
            Some(vec!["000h".into(), "0".into(), "0".into()])
        );
    }

    #[test]
    fn input_definitions() {
        // [CAP g_init] and the 601's mapping [SYS].
        assert_eq!(
            parse_input_def("&I240,000h,0,0", 240),
            Some(InputDef::Unmapped)
        );
        let start = parse_input_def("&I221,279h,7,1", 221).expect("parse");
        assert_eq!(
            start,
            InputDef::Mapped {
                port: 0x279,
                bit: 7,
                inverted: true
            }
        );
        assert!(start.is_mapped());
        assert_eq!(start.to_string(), "279h,7,1");
        assert_eq!(parse_input_def("&I221,279h,7,1", 222), None, "other input");
        assert_eq!(parse_input_def("&I221,279,7,1", 221), None, "no h suffix");
    }

    #[test]
    fn homing_direction() {
        assert_eq!(
            parse_homing_side("&DX,0,10,11,0", Axis::X),
            Some(Side::Minus)
        );
        assert_eq!(
            parse_homing_side("&DY,1,10,11,0", Axis::Y),
            Some(Side::Plus)
        );
        assert_eq!(parse_homing_side("&DX,2,10,11,0", Axis::X), None);
        assert_eq!(parse_homing_side("&DX,", Axis::X), None);
    }
}
