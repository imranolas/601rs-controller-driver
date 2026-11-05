//! Classification of a received reply line.

use crate::Status;

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
}
