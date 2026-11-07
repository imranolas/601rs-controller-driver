use protocol::{Command, ErrorCode, Mode, Reply, Status};

/// What came back while a resync probe went unanswered.
fn heard(bytes: &u64, lines: &[String]) -> String {
    match (*bytes, lines) {
        (0, _) => "nothing came back at all".to_owned(),
        (n, []) => format!("{n} byte(s) came back, but no complete reply line"),
        (n, lines) => format!("{n} byte(s) came back, but only other replies: {lines:?}"),
    }
}

fn fault_code(code: &Option<ErrorCode>) -> String {
    code.map_or_else(|| "error code unreadable".to_owned(), |c| c.to_string())
}

#[derive(Debug, thiserror::Error)]
pub enum ControllerError {
    /// The link is gone: the reader saw it close, or a write to it failed.
    #[error("link closed: {0}")]
    Disconnected(String),

    /// The session could not start (its reader thread could not be spawned).
    #[error("cannot start the session: {0}")]
    Io(#[from] std::io::Error),

    #[error("no reply to `{command}` within {timeout_ms} ms")]
    Timeout { command: Command, timeout_ms: u64 },

    /// The resync probe (`@I1xx`) got no reply of its own, so replies cannot be paired with
    /// commands. What did arrive tells why: nothing (link, cable, power), bytes but no complete
    /// line (baud rate, framing), or other replies (the controller answers, but not the probe).
    #[error(
        "no reply to the resync probe `{probe}` within {timeout_ms} ms: {}",
        heard(.bytes_heard, .lines_heard)
    )]
    ResyncFailed {
        probe: Command,
        timeout_ms: u64,
        /// Bytes received after the probe was sent.
        bytes_heard: u64,
        /// The first lines received after the probe that were not its reply.
        lines_heard: Vec<String>,
    },

    #[error("controller rejected `{command}` with {code}")]
    Rejected { command: Command, code: ErrorCode },

    /// A probe sent right behind `command` was answered before `command` was: the controller
    /// does not answer in the order it is sent, which reply pairing relies on.
    #[error("the probe sent after `{command}` was answered first: replies came out of order")]
    Overtaken { command: Command },

    /// A `*NNN` arrived that was not the reply to a command we sent: a fault during motion
    /// (e.g. `*033` limit switch) or an error caused by a streamed mode-2 command.
    #[error("controller reported {code}")]
    AsyncError { code: ErrorCode },

    #[error("unexpected reply to `{command}`: {reply}")]
    UnexpectedReply { command: Command, reply: Reply },

    /// `@X` has its error flag set. `code` is the `@E` error code, if it could be read.
    #[error("machine reports a fault (status {status}, {})", fault_code(.code))]
    Fault {
        status: Status,
        code: Option<ErrorCode>,
    },

    #[error("condition not reached within {timeout_ms} ms (last status {last})")]
    WaitTimeout { timeout_ms: u64, last: Status },

    #[error("controller buffer stayed full for {timeout_ms} ms")]
    FlowTimeout { timeout_ms: u64 },

    #[error("a stop was issued; call clear_stop() before continuing")]
    Stopped,

    /// A command offered in a mode that doesn't take it; it was not sent.
    #[error("`{command}` is not a {mode:?} command")]
    WrongMode { command: Command, mode: Mode },
}

pub type Result<T> = std::result::Result<T, ControllerError>;

/// A duration as whole milliseconds, for the `timeout_ms` fields above.
pub(crate) fn millis(d: std::time::Duration) -> u64 {
    u64::try_from(d.as_millis()).unwrap_or(u64::MAX)
}
