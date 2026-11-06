//! `@X` status reply: `@X<r><f>` [SPEC p.12].
//!
//! `r` = 0 stopped / 1 moving; `f` = hex flags 1 waiting (dead time / input), 2 error,
//! 4 home position known, 8 homing in progress. Flag 4 = "homed" is settled by the captures:
//! absent during homing (`@X18`), present in every state afterwards (`04`/`14`/`15`).

use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Status {
    pub moving: bool,
    pub waiting: bool,
    pub error: bool,
    pub homed: bool,
    pub homing: bool,
}

impl Status {
    /// Parse a reply line such as `@X14`. Returns `None` if it is not a status reply.
    pub fn parse(line: &str) -> Option<Status> {
        let rest = line.strip_prefix("@X")?;
        let mut chars = rest.chars();
        let run = match chars.next()? {
            '0' => false,
            '1' => true,
            _ => return None,
        };
        let flags = chars.next()?.to_digit(16)?;
        if chars.next().is_some() {
            return None;
        }
        Some(Status {
            moving: run,
            waiting: flags & 1 != 0,
            error: flags & 2 != 0,
            homed: flags & 4 != 0,
            homing: flags & 8 != 0,
        })
    }

    /// Idle: not moving. GMC treats any `@X0…` as "move finished" [GMC].
    pub fn is_idle(&self) -> bool {
        !self.moving
    }
}

impl fmt::Display for Status {
    /// The wire form, e.g. `@X14`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flags = u8::from(self.waiting)
            | (u8::from(self.error) << 1)
            | (u8::from(self.homed) << 2)
            | (u8::from(self.homing) << 3);
        write!(f, "@X{}{flags:X}", u8::from(self.moving))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn captured_codes() {
        let s = Status::parse("@X04").expect("parse");
        assert!(s.is_idle() && s.homed && !s.error && !s.waiting && !s.homing);

        let s = Status::parse("@X14").expect("parse");
        assert!(s.moving && s.homed && !s.waiting);

        let s = Status::parse("@X15").expect("parse");
        assert!(s.moving && s.homed && s.waiting);

        let s = Status::parse("@X18").expect("parse");
        assert!(s.moving && s.homing && !s.homed);
    }

    #[test]
    fn displays_as_the_wire_code() {
        for code in ["@X04", "@X14", "@X15", "@X18", "@X06", "@X00"] {
            let s = Status::parse(code).expect("parse");
            assert_eq!(s.to_string(), code);
        }
    }

    #[test]
    fn rejects_non_status() {
        for bad in [
            "",
            "@X",
            "@X1",
            "@X2",
            "@X1G",
            "@X140",
            "@V2.00/55W",
            "*021",
        ] {
            assert_eq!(Status::parse(bad), None, "{bad}");
        }
    }
}
