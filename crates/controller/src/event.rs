use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Mutex;

use protocol::Signal;

use crate::session::lock;

/// Something observed on the link, delivered to every subscriber.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// Bytes written to the controller (for tracing / GMC-format logs).
    Sent(Vec<u8>),
    /// Bytes received from the controller, as read (for tracing).
    Received(Vec<u8>),
    /// An out-of-band signal byte (positioned, homing done, start button, flow control …).
    Signal(Signal),
    /// A control byte the decoder does not know.
    UnknownByte(u8),
    /// A reply line that no command was waiting for (late reply, async `*NNN`, …).
    Unsolicited(String),
    /// A stop was issued through a [`crate::StopHandle`].
    StopIssued(StopKind),
    /// The link closed.
    Disconnected(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopKind {
    /// `@B`: ramped stop, positions kept.
    Brake,
    /// `@S`: immediate stop, positions lost.
    Emergency,
}

#[derive(Default)]
pub(crate) struct Bus {
    subscribers: Mutex<Vec<Sender<Event>>>,
}

impl Bus {
    pub(crate) fn subscribe(&self) -> Receiver<Event> {
        let (tx, rx) = mpsc::channel();
        lock(&self.subscribers).push(tx);
        rx
    }

    /// Deliver to every live subscriber; dropped receivers are forgotten.
    pub(crate) fn emit(&self, event: Event) {
        lock(&self.subscribers).retain(|s| s.send(event.clone()).is_ok());
    }
}
