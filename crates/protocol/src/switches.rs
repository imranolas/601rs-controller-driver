//! `@Y` switch inputs: `@Yhhhh`, 16 bits `sbff rrrr uuzz yyxx` [SPEC1 p.13].
//!
//! s = stop input, b = machine ready, ff = undefined, rrrr = reference switches (X lowest),
//! then per axis a pair of limit switches. The spec example `@Y4001` = "limit + of axis X,
//! machine ready" fixes the order within a pair: the lower bit is PLUS, the upper is MINUS.

/// Axis index: 0 = X, 1 = Y, 2 = Z, 3 = U (4th axis, unused on the 601).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Switches {
    pub raw: u16,
    pub stop: bool,
    pub ready: bool,
    pub reference: [bool; 4],
    pub limit_plus: [bool; 4],
    pub limit_minus: [bool; 4],
}

impl Switches {
    pub fn from_bits(raw: u16) -> Switches {
        let bit = |n: u16| raw & (1 << n) != 0;
        let pairs = [0, 2, 4, 6];
        Switches {
            raw,
            stop: bit(15),
            ready: bit(14),
            reference: [bit(8), bit(9), bit(10), bit(11)],
            limit_plus: pairs.map(bit),
            limit_minus: pairs.map(|p| bit(p + 1)),
        }
    }

    /// Parse a reply line such as `@Y4001`.
    pub fn parse(line: &str) -> Option<Switches> {
        let hex = line.strip_prefix("@Y")?;
        if hex.len() != 4 {
            return None;
        }
        u16::from_str_radix(hex, 16).ok().map(Switches::from_bits)
    }

    /// Any limit switch active.
    pub fn any_limit(&self) -> bool {
        self.limit_plus.iter().chain(&self.limit_minus).any(|&b| b)
    }

    /// Bit for a limit-switch error code `*032`–`*039` (EENDM1, EENDP1, … EENDP4).
    pub fn bit_for_limit_error(code: u16) -> Option<u16> {
        let n = code.checked_sub(32).filter(|n| *n < 8)?;
        let (axis, plus) = (n / 2, n % 2 == 1);
        Some(axis * 2 + u16::from(!plus))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spec_example() {
        let s = Switches::parse("@Y4001").expect("parse");
        assert!(s.ready && !s.stop);
        assert!(s.limit_plus[0], "X plus");
        assert!(!s.limit_minus[0]);
        assert!(s.any_limit());
    }

    #[test]
    fn reference_and_stop_bits() {
        let s = Switches::parse("@YC300").expect("parse");
        assert!(s.stop && s.ready && s.reference[0] && s.reference[1] && !s.reference[2]);
        assert!(!s.any_limit());
    }

    #[test]
    fn limit_error_codes_map_to_bits() {
        // *032 EENDM1 = X minus, *033 EENDP1 = X plus, *034/*035 = Y minus/plus
        assert_eq!(Switches::bit_for_limit_error(33), Some(0));
        assert_eq!(Switches::bit_for_limit_error(32), Some(1));
        assert_eq!(Switches::bit_for_limit_error(35), Some(2));
        assert_eq!(Switches::bit_for_limit_error(34), Some(3));
        assert_eq!(Switches::bit_for_limit_error(40), None);
    }

    #[test]
    fn rejects_malformed() {
        for bad in ["@Y", "@Y400", "@Y40011", "@YZZZZ", "@X4001"] {
            assert_eq!(Switches::parse(bad), None, "{bad}");
        }
    }
}
