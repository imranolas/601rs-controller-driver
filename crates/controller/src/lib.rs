//! The **valid** layer: a session with the controller that sends only well-formed commands
//! that are allowed in the current mode.
//!
//! Responsibilities (to implement):
//! - Own the [`link::Port`]: one reader thread that decodes frames ([`protocol::Decoder`])
//!   and one writer.
//! - **One outstanding command** in modes 0/1, its reply matched before the next is sent,
//!   every wait bounded by a deadline that returns an explicit error (never "carry on").
//! - **Mode as a type** (typestate): `Controller<Parameters>`, `Controller<Jog>`,
//!   `Controller<Cutting>`; methods exist only in the mode that accepts them.
//! - Mode-2 streaming without per-command replies; flow control via
//!   `Signal::BufferAlmostFull`/`BufferResume` or `@H`.
//! - **Stop lane**: `@B`/`@S` bypass the queue and can be triggered from any thread
//!   ([`StopHandle`]).
//! - Surface async signals (positioned, homing done, start button) as events.
//!
//! Nothing here knows about millimetres, heads or programs; that is `machine`.

use std::marker::PhantomData;

/// Typestate markers for the controller mode.
pub mod mode {
    #[derive(Debug)]
    pub struct Parameters;
    #[derive(Debug)]
    pub struct Jog;
    #[derive(Debug)]
    pub struct Cutting;
}

/// A connected controller in mode `M`. Construction and methods to follow.
#[derive(Debug)]
pub struct Controller<M> {
    _mode: PhantomData<M>,
}

/// Cloneable handle that can brake or stop the machine from any thread, bypassing the
/// command queue. To implement.
#[derive(Debug, Clone)]
pub struct StopHandle {
    _private: (),
}

#[derive(Debug, thiserror::Error)]
pub enum ControllerError {
    #[error("I/O on the link: {0}")]
    Io(#[from] std::io::Error),
    #[error("no reply to `{command}` within {timeout_ms} ms")]
    Timeout { command: String, timeout_ms: u64 },
    #[error("controller rejected `{command}` with *{code:03}")]
    Rejected { command: String, code: u16 },
    #[error("unexpected reply to `{command}`: {reply:?}")]
    UnexpectedReply { command: String, reply: String },
}
