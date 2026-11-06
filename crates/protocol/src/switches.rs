//! `@Y` switch inputs: `@Yhhhh`, 16 bits `sbff rrrr uuzz yyxx` [SPEC1 p.13].
//!
//! s = stop input, b = machine ready, ff = undefined, rrrr = reference switches (X lowest),
//! then per axis a pair of limit switches. The spec example `@Y4001` = "limit + of axis X,
//! machine ready" fixes the order within a pair: the lower bit is PLUS, the upper is MINUS.

use std::fmt;

use crate::Axis;
use crate::errors::ErrorCode;

/// One end of an axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Side {
    Minus,
    Plus,
}

/// One limit switch: an axis and the end it guards.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LimitSwitch {
    pub axis: Axis,
    pub side: Side,
}

impl LimitSwitch {
    const fn pair(self) -> u8 {
        match self.axis {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }

    /// The error the controller sends on reaching this switch: `*032` X minus (EENDM1),
    /// `*033` X plus (EENDP1), … `*037` Z plus [SPEC1 p.38].
    pub fn error_code(self) -> ErrorCode {
        let plus = u16::from(self.side == Side::Plus);
        ErrorCode(32 + 2 * u16::from(self.pair()) + plus)
    }

    /// Its logical input: IENDM1–3 = I239–I241, IENDP1–3 = I243–I245 [SPEC1 p.10].
    pub const fn input(self) -> u8 {
        match self.side {
            Side::Minus => 239 + self.pair(),
            Side::Plus => 243 + self.pair(),
        }
    }

    /// Its bit in `@Y`: the lower bit of the axis pair is PLUS.
    const fn bit(self) -> u16 {
        let minus = match self.side {
            Side::Minus => 1,
            Side::Plus => 0,
        };
        1 << (2 * self.pair() + minus)
    }
}

impl fmt::Display for LimitSwitch {
    /// `X+`, `Y-`, …
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sign = match self.side {
            Side::Minus => '-',
            Side::Plus => '+',
        };
        write!(f, "{}{sign}", self.axis.upper())
    }
}

/// The `@Y` switch inputs, kept as the raw bits; the methods decode them.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Switches(u16);

impl Switches {
    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u16 {
        self.0
    }

    /// Parse a reply line such as `@Y4001`.
    pub fn parse(line: &str) -> Option<Self> {
        let hex = line.strip_prefix("@Y")?;
        if hex.len() != 4 {
            return None;
        }
        u16::from_str_radix(hex, 16).ok().map(Self)
    }

    /// The stop input (ISTOP).
    pub const fn stop(self) -> bool {
        self.0 & (1 << 15) != 0
    }

    /// Machine ready (IBEREIT).
    pub const fn ready(self) -> bool {
        self.0 & (1 << 14) != 0
    }

    /// The reference (home) switch of `axis`.
    pub const fn reference(self, axis: Axis) -> bool {
        let n = match axis {
            Axis::X => 8,
            Axis::Y => 9,
            Axis::Z => 10,
        };
        self.0 & (1 << n) != 0
    }

    pub const fn limit(self, switch: LimitSwitch) -> bool {
        self.0 & switch.bit() != 0
    }

    /// Any limit switch active, including the fourth axis's.
    pub const fn any_limit(self) -> bool {
        self.0 & 0x00FF != 0
    }
}

impl fmt::Display for Switches {
    /// The wire form, `@Y4001`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "@Y{:04X}", self.0)
    }
}

impl fmt::Debug for Switches {
    /// `Switches(@Y4001: ready, limit X+)`
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut active = Vec::new();
        if self.stop() {
            active.push("stop".to_owned());
        }
        if self.ready() {
            active.push("ready".to_owned());
        }
        for axis in Axis::ALL {
            if self.reference(axis) {
                active.push(format!("reference {}", axis.upper()));
            }
            for side in [Side::Minus, Side::Plus] {
                let switch = LimitSwitch { axis, side };
                if self.limit(switch) {
                    active.push(format!("limit {switch}"));
                }
            }
        }
        write!(f, "Switches({self}: {})", active.join(", "))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const X_PLUS: LimitSwitch = LimitSwitch {
        axis: Axis::X,
        side: Side::Plus,
    };
    const X_MINUS: LimitSwitch = LimitSwitch {
        axis: Axis::X,
        side: Side::Minus,
    };

    #[test]
    fn spec_example() {
        let s = Switches::parse("@Y4001").expect("parse");
        assert!(s.ready() && !s.stop());
        assert!(s.limit(X_PLUS) && !s.limit(X_MINUS));
        assert!(s.any_limit());
        assert_eq!(s.to_string(), "@Y4001");
        assert_eq!(format!("{s:?}"), "Switches(@Y4001: ready, limit X+)");
    }

    #[test]
    fn reference_and_stop_bits() {
        let s = Switches::parse("@YC300").expect("parse");
        assert!(s.stop() && s.ready());
        assert!(s.reference(Axis::X) && s.reference(Axis::Y) && !s.reference(Axis::Z));
        assert!(!s.any_limit());
    }

    #[test]
    fn limit_switches_match_the_error_codes_and_inputs() {
        // *032 EENDM1 = X minus, *033 EENDP1 = X plus, *034/*035 = Y minus/plus
        assert_eq!(X_MINUS.error_code(), ErrorCode(32));
        assert_eq!(X_PLUS.error_code(), ErrorCode(33));
        let y_plus = LimitSwitch {
            axis: Axis::Y,
            side: Side::Plus,
        };
        assert_eq!(y_plus.error_code(), ErrorCode(35));
        assert_eq!(
            (X_MINUS.input(), X_PLUS.input(), y_plus.input()),
            (239, 243, 244)
        );
        assert!(Switches::from_bits(0b0100).limit(LimitSwitch {
            axis: Axis::Y,
            side: Side::Plus
        }));
        assert_eq!(X_PLUS.to_string(), "X+");
    }

    #[test]
    fn rejects_malformed() {
        for bad in ["@Y", "@Y400", "@Y40011", "@YZZZZ", "@X4001"] {
            assert_eq!(Switches::parse(bad), None, "{bad}");
        }
    }
}
