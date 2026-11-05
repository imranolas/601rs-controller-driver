//! Executes a validated [`program::Program`] on the machine (to implement).
//!
//! - Before: validate (including V10 machine envelope), require homed, confirm board loaded.
//! - During: run ops in order on a dedicated thread; emit progress events per op `id`/`label`.
//! - Pause (UI, Space, green start button): finish the current cut, raise the head, hold.
//! - Stop now: brake via the controller stop lane, raise the head, release, report the op.
//! - After / on any exit path: safe end state (head up, clamps released, park).
//!
//! The UI only renders runner events and sends requests; it never touches the wire.
