# Working on 601rs

601rs is an open driver for the Gunnar 601 mat cutter, in Rust. It speaks the RS-232 protocol of
the CNC-Control controller inside the machine, and runs, checks and calibrates the cutter.
`mc601` is its command line. Apps use its crates as libraries, so a change to a public API
reaches them too.

**It moves a machine with a blade in it.** An agent never drives the machine. Read *The machine*
below before anything else.

Read these first, in this order:

1. [`README.md`](README.md): what each crate does, and how they fit together.
2. [`PROTOCOL_REFERENCE.md`](PROTOCOL_REFERENCE.md): the wire protocol and the cut sequence, with
   the source of every fact. §11 is what the session layer guarantees. The code cites this
   document rather than repeating it.
3. [`PROGRAM_FORMAT.md`](PROGRAM_FORMAT.md) and [`program.schema.json`](program.schema.json): the
   program, the contract between the compiler and the runner.
4. [`CALIBRATION.md`](CALIBRATION.md): the calibration file and the two routines. §1 says where
   the work stands.
5. [`docs/hardware-testing.md`](docs/hardware-testing.md): what the operator does at the machine.

## Layout

```
crates/protocol      pure codec: commands to bytes, bytes to replies; no I/O, no clock
crates/link          the serial port paced to the line, an in-memory pair, a byte tap
crates/controller    valid commands only: one outstanding, deadlines, modes, the stop lane
crates/sim           the controller simulator behind an in-memory link
crates/program       the program format: types, parsing, validation
crates/machine       the machine model, pure: profile, lowering, soft limits, run time
crates/runner        drives the cutter: homing, programs, pause, cancel, stop, the safe state
crates/route         the cut order
crates/nest          mats onto sheets
crates/compiler      a laid-out sheet to a program, by the calibrated tool model
crates/calibrate     the calibration file and the routines: plans, sessions, fit, proposals
crates/mc601         the command line
testdata/captures    GMC serial captures: recordings, never edited
testdata/programs    GMC's jobs as programs, written from the captures by a golden test
testdata/sessions    mc601 logs from the machine, replayed against the simulator
scripts/check.sh     everything a commit must pass
```

## Commands

```sh
scripts/check.sh                            # fmt, clippy (host and wasm32), every test: before each commit
cargo test --workspace                      # about 2 min from cold; replays every capture and session log
cargo test -p sim --test fidelity           # only the session-log replays
cargo fmt --all
cargo run -p mc601 -- capture testdata/captures/g_init.txt   # decode a GMC capture
BLESS=1 cargo test -p machine --test golden # rewrite testdata/programs (see Recordings are evidence)
```

Once per machine: `rustup target add wasm32-unknown-unknown`.

`mc601` itself, on the simulator only, with its files in a scratch folder:

```sh
export MC601_DATA=$TMPDIR/mc601 MC601_CONFIG=$TMPDIR/mc601/config.json \
       MC601_CALIBRATION=$TMPDIR/mc601/calibration.json
cargo run -p mc601 -- run --sim testdata/programs/g_regular_mount.json
yes '' | cargo run -p mc601 -- check --sim --stage 2   # check waits for Enter at every step
```

## The machine

- **Never talk to the real controller.** `run`, `probe`, `check` and `calibrate heads | runons`
  open the serial port unless given `--sim` (or `--sim-limits` for `check`). Only the operator
  runs those, at the machine, with someone at the stop button. You may ask the operator to run
  one: give the exact command and what it will do, and read the log they bring back.
- **Even on the simulator, mc601 writes the operator's files.** Each session goes into the data
  folder (`~/Documents/mc601` by default) and becomes `mc601 logs --last`, which
  `calibrate read` picks up. The routines' apply step and `calibration set | unset | revert`
  write the machine's calibration file, which everything that cuts reads. Set `MC601_DATA`,
  `MC601_CONFIG` and `MC601_CALIBRATION` (or pass `--calibration`) to a scratch folder first.
- **The guarantees of PROTOCOL_REFERENCE §11 are safety, not style.** One outstanding command
  with a deadline. A stop (`@B`, `@S`) goes past anything in progress, behind at most 64 queued
  bytes, and latches. After any failure the runner brakes, resynchronises and leaves the heads up
  and the clamps released, then asks for homing. A head is down only inside a cut. Every move is
  checked against the soft limits before anything is sent. A change to `protocol`, `link`,
  `controller`, `runner`, `machine` or `sim` that touches any of these gets a test against the
  simulator that fails without it, and a review focused on them before it is committed.
- **What the driver never sends** (`@M9`, `#W`, `@R`, `@D`, `#C18`) cannot be represented in
  `protocol::Command`. Keep it so: no new variant, no raw-bytes escape, no command text formatted
  outside `protocol`.
- **The simulator is not the machine.** Behaviour the simulator assumes and no session has
  shown is marked *(sim assumption)* in PROTOCOL_REFERENCE.md, with the `mc601 check` stage that
  would settle it (§10). Never report the simulator's behaviour as the 601's.

## Rules

- **Layering.** As it is now: `protocol`, `program`, `route`, `nest` and `link` depend on none of
  ours; `controller` on `protocol` and `link`; `sim` on `link`; `machine` on `protocol` and
  `program`; `runner` on `controller`, `machine`, `program` and `protocol`; `compiler` on
  `machine`, `program` and `route`; `calibrate` on `compiler`, `machine`, `program` and
  `protocol`; `mc601` on what it needs. Tests may reach further as dev-dependencies (most use
  `sim`). A new edge is a design change: say why in the commit.
- **Pure crates stay pure.** `protocol` and `machine` have no I/O and no clock. `calibrate` has no
  clock (callers pass the time) and does I/O only in `calibrate::store`. Apps run `protocol`,
  `program`, `machine`, `route`, `nest`, `compiler` and `calibrate` in the browser, so they must
  build for `wasm32-unknown-unknown` (check.sh builds them), with no threads, serial ports or
  system time. A build can't catch a call that panics there at run time, such as
  `Instant::now`: don't add one.
- **Every fact has a source.** A fact about the controller or GMC goes into PROTOCOL_REFERENCE.md
  with its citation (`[SPEC1 p.N]`, `[CAP name]`, `[SESSION name]`, `[GMC]`, `[GCF]`), and a
  changed fact gets a row in §13 (was, now, source). Captures and sessions beat the manual
  (§13.1). A claim with nothing behind it is marked *(unverified)*. Code cites the section
  (`§5`, `[SPEC1 p.30]`) instead of restating it.
- **Recordings are evidence.** Never edit `testdata/captures/` or a log in `testdata/sessions/`.
  A replay that differs is a fact about the box the simulator doesn't know yet: fix the
  simulator or `SimConfig::default()`, never the log (`testdata/sessions/README.md`).
  `testdata/programs/` is written from the captures by `BLESS=1`: bless only when lowering
  changed on purpose, and say so in the commit.
- **Tests first, and never weakened.** A fix comes with a test that fails without it.
  Behaviour against the controller is tested end to end against the simulator
  (`crates/sim/tests/controller.rs`, `crates/runner/tests/`). Never change an expectation, a
  tolerance or a golden file to make a test pass. If you think one is wrong, stop and say why.
- **Keep the docs true, in the same commit.** The program format: PROGRAM_FORMAT.md,
  program.schema.json, and its `version` when §10 says so. The protocol: PROTOCOL_REFERENCE.md.
  Calibration: CALIBRATION.md, with §1 kept current. What the operator does:
  `docs/hardware-testing.md` and `mc601 --help`. The README's crate table and numbers when they
  change.
- **No vendor material.** The repository holds no code, documents, parameter files or
  screenshots from GMC or the controller's maker. Say what GMC does in our own words, cited
  `[GMC]`, and copy no code and no text. Captures have the controller's licence file blanked;
  blank it in any new one.
- **Everything committed is public**, commit messages included. A change to a public API says
  in its commit what callers must change.
- **Words.** British spelling (licence, centre, behaviour, optimise). Plain, short sentences in
  docs, comments, help text and commit messages. A comment says why, not what.
- **Dependencies.** Few and well known: `serde`, `serde_json`, `thiserror`, `serialport`,
  `ctrlc`, `nalgebra` (in `calibrate` only), and `clap`, `anstream`, `anstyle`, `chrono` in
  `mc601`. Ask before adding one.
- **Commits.** Small, each passing `scripts/check.sh`. The summary names the crate or document,
  a colon, then what changed in a phrase ("sim: the homing offset leaves room below zero, as on
  the 601"). The body says why, then what. One branch per piece of work (`calibrate/m3-plans`).
  Don't push or open a pull request unless asked.

## Code

Idiomatic, expressive and concise. Every change looks at the code it touches for what can be
merged or removed, and leaves it simpler than it was.

- **Say it once.** One type, table or function per idea, read by everything that needs it. A
  reading is one `calibrate::Reading`; each calibration key's path, unit, bounds and words are
  one table. Where the same knowledge sits in two places, or a value is translated into another
  form and back, merge them.
- **Delete what nothing needs.** Code, options and formats that served only something retired
  go in the change that retires it, as calibrations 1 and 2 went with everything that served
  them (4742c4f). Keep reading what is already on disk (sessions, calibration files, programs)
  by its version, and nothing more.
- **Let the types hold the rules.** Parse at the edge into types that can't hold a wrong value:
  `OverridePct` is 10 to 200, and `Controller<M>` offers only mode `M`'s commands. No silent
  default for a missing field, no string where an enum would do, no `Option` for a value that is
  always there.
- **Plain Rust.** `std`, iterators, `?`, `let … else`, `match` on enums. Code stays concrete
  until a second caller needs it generic: no trait with one implementation, no builder for a
  few fields, no option nobody passes, nothing written for later.
- **Short and flat.** Small functions named for what they return, early returns over nesting,
  and a better name over a comment.
- **A simplification gets a commit of its own**, before or after the change it was found in,
  so each one reads on its own and passes `scripts/check.sh`.
- Edition 2024, `rust-version` 1.85 (clippy flags newer std APIs). `unsafe` is forbidden. No
  `unwrap()`: `?` and `thiserror` errors in the libraries, `expect("why it holds")` where a value
  can't be missing. Public types derive `Debug`. Values stay exact through JSON
  (`float_roundtrip`): a program, plan or session read back lowers to the same steps.

## When in doubt

Prefer what the captures show GMC doing, and the simplest change that keeps the guarantees.
If the documents and the code disagree, or a fact has no source, say so rather than guess, and
write the question in the document's open section (PROTOCOL_REFERENCE §10, PROGRAM_FORMAT §9,
CALIBRATION §11).
