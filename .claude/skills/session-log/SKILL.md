---
name: session-log
description: Take a session log written by mc601 at the machine into testdata/sessions so the simulator must replay it, and fix the simulator where it differs from the box. Use when the operator brings back a log from probe, check, run or calibrate, or asks to add one.
---

# A session log from the machine

A log in `testdata/sessions/` is a regression test: `cargo test` replays it against the
simulator (`crates/sim/tests/fidelity.rs`) and requires the replies the box gave
(`testdata/sessions/README.md`). A difference is a fact about the 601 the simulator doesn't know
yet.

1. **Find the log.** The operator gives a path, or `mc601 logs` lists the sessions in their data
   folder (reading it is fine). The log is the `<cmd>.log` in the session folder. Read its
   `session.json` for what was run.
2. **Copy it in, unchanged.** `cp` it to `testdata/sessions/<what-it-was>.log`, named for what it
   was (`calibration_1-run.log`), with the extension `.log`. Never edit it, trim it or write one
   by hand. The guard hook refuses edits there.
3. **Replay it.** `cargo test -p sim --test fidelity`. The report names the first difference:
   `#1356  real &I243 → "&I243,001h,0,0"  sim &I243 → "&I243,000h,0,0"`.
4. **Fix the simulator, not the log.** Find what the box did around that reply, then change
   `crates/sim` or `SimConfig::default()` (which describes the 601) until the log replays.
   Repeat from step 3 until it passes, and run the whole suite: another log must not break.
5. **Write the fact down.** In `PROTOCOL_REFERENCE.md`: the fact with `[SESSION <name>]`, a row
   in §13 (was, now, source), and remove any *(sim assumption)* or §10 open item it settles.
6. **What a replay can't check.** Timing and the physical machine (`@X` status, positions,
   inputs, switches, outputs read back, buffer level, progress) are compared only by which query
   they answer. For a `check` session, read its PASS, FAIL and INFO lines: each finding the
   simulator doesn't yet show needs its own fix and an end-to-end test in
   `crates/sim/tests/controller.rs`.
7. **Commit.** `scripts/check.sh`, then one commit: `sim: <the fact>`, the body naming the log
   and what the box does. Add the log in the same commit.
