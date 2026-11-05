use protocol::Status;
use protocol::errors::describe_or_unknown;

fn text(code: &u16) -> &'static str {
    describe_or_unknown(*code)
}

#[derive(Debug, thiserror::Error)]
pub enum ControllerError {
    #[error("link closed: {0}")]
    Disconnected(String),

    #[error("I/O error on the link: {0}")]
    Io(#[from] std::io::Error),

    #[error("no reply to `{command}` within {timeout_ms} ms")]
    Timeout { command: String, timeout_ms: u64 },

    #[error("controller rejected `{command}` with *{code:03}: {}", text(.code))]
    Rejected { command: String, code: u16 },

    /// A `*NNN` arrived that was not the reply to a command we sent: a fault during motion
    /// (e.g. `*033` limit switch) or an error caused by a streamed mode-2 command.
    #[error("controller reported *{code:03}: {}", text(.code))]
    AsyncError { code: u16 },

    #[error("unexpected reply to `{command}`: {reply}")]
    UnexpectedReply { command: String, reply: String },

    /// `@X` has its error flag set. `code` is the `@E` error code, if it could be read.
    #[error("machine reports a fault (status {status:?}, error code {code:?})")]
    Fault { status: Status, code: Option<u16> },

    #[error("condition not reached within {timeout_ms} ms (last status {last:?})")]
    WaitTimeout { timeout_ms: u64, last: Status },

    #[error("controller buffer stayed full for {timeout_ms} ms")]
    FlowTimeout { timeout_ms: u64 },

    #[error("a stop was issued; call clear_stop() before continuing")]
    Stopped,

    #[error("override {0}% out of range (10–200)")]
    OverrideOutOfRange(u16),
}

pub type Result<T> = std::result::Result<T, ControllerError>;
