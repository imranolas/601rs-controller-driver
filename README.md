# 601rs

An open-source driver for the Gunnar 601 mat cutter, in Rust. It talks to the CNC-Control
controller in the machine over RS-232, and runs, checks and calibrates the machine from macOS,
Linux or Windows. `mc601` is its command line.

> **Not affiliated with Gunnar.** 601rs is an independent project, not made, endorsed or
> supported by Gunnar, the maker of the 601. Gunnar and GMC are trademarks of their owners and
> appear here only to say which machine this drives and which software it works alongside.

> **It moves a machine with a blade in it.** It has run on one 601 so far. Read
> [`docs/hardware-testing.md`](docs/hardware-testing.md) before you point it at a machine, stay
> within reach of the machine's stop button, and expect bugs. It comes with no warranty (see
> the licences).

It takes **vector geometry plus a profile** (a board type) and cuts it: nested onto sheets by
board type, compiled into programs, checked and run. Mat design (an editor, templates, layout)
belongs to a separate app.

## How it was built

From the controller's programming manual (CNC-Control v1.00, SIG Positec), serial captures of
GMC (Gunnar's own software) driving a 601, sessions on that machine, and study of how GMC drives
it. [`PROTOCOL_REFERENCE.md`](PROTOCOL_REFERENCE.md) gives the source of every fact. The
repository holds no code or documents from GMC or from the controller's maker; the captures in
`testdata/captures/` are recordings of the serial line, with the controller's licence file
blanked.

- [`PROTOCOL_REFERENCE.md`](PROTOCOL_REFERENCE.md): wire protocol, cut sequence, calibration (with citations)
- [`PROGRAM_FORMAT.md`](PROGRAM_FORMAT.md): the program format (the contract between the compiler and the runner)
- [`DIAGNOSTIC_TOOL_SPEC.md`](DIAGNOSTIC_TOOL_SPEC.md): calibration/diagnostic routines
- [`docs/hardware-testing.md`](docs/hardware-testing.md): running `mc601 probe` and `mc601 check` at the machine

## Use

```sh
cargo install --path crates/mc601    # puts `mc601` on the PATH
mc601 check --sim                    # every hardware check, against the simulator
mc601 probe                          # at the machine: read-only, nothing moves
mc601 run job.json                   # cut a program
mc601 --help
```

## Layers

Each layer is a crate. A crate may only depend on crates **below** it. *(planned)* marks crates
not written yet.

```
station     (planned) service on the machine PC: queue, HTTP API and events for apps; starts
            cuts only from the machine's own screen
nest        places mats on sheets of their board type, and in other mats' drop-outs:
            three-stage guillotine layouts by iterative beam search (after PackingSolver);
            offcuts before full sheets; every layout checked before it is returned
compiler    (planned) geometry + profile → Program: flattening, overcut, overlap, cut order
profile     (planned) board types: speed, trims, overcut, overlap, start speed
route       the cut order: which cut next and which way round, so the head travels least,
            within the cutting rules

runner      drives the cutter: configures the controller, homes, runs programs and manual
            operations; pause, cancel, holds, stop now, safe state after any failure, events
machine     the machine model, pure: profile, mm/deg → steps, each cut lowered as one unit
            (GMC's form, held to its captures, or one stream per cut), soft limits, run time,
            byte budget
controller  VALID commands: one outstanding at a time, matched replies + deadlines,
            mode tracking (typestate), stop lane, async signals
protocol    pure codec: Command → bytes, bytes → Frame/Reply. No I/O, no clock.
link        Reader/Writer halves: serial port (paced to the line) | in-memory pair | byte tap
sim         controller simulator behind an in-memory link (tests, dry runs); checked
            against every GMC capture and session log by the fidelity test

program     the Program format: types, parsing, validation (used by runner and apps)
mc601       CLI: run (cut programs), calibrate, check (staged hardware checks), probe, logs,
            config, ports, capture (decode GMC captures)
```

**Invariant:** commands that must never be sent from this software (mode 9 file transfer,
firmware upload, parameter save `#W`) are not representable in `protocol::Command`.

## Build and test

```sh
cargo test                       # unit tests + replay tests against testdata/captures/g_*.txt
cargo run -p mc601 -- capture testdata/captures/g_init.txt    # decode a GMC serial capture
```

## Licence

Licensed under either of the [Apache License, Version 2.0](LICENSE-APACHE) or the
[MIT licence](LICENSE-MIT), at your option. Unless you explicitly state otherwise, any
contribution intentionally submitted for inclusion in the work by you, as defined in the
Apache-2.0 licence, shall be dual licensed as above, without any additional terms or
conditions.

Third-party notices: [`THIRD-PARTY-NOTICES.md`](THIRD-PARTY-NOTICES.md).
