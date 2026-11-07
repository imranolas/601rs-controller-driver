# 601rs: drive the Gunnar 601rs mat cutter

Takes **vector geometry plus a profile** (a board type) and cuts it: nested onto sheets by board
type, compiled into programs, checked and run. Cross-platform; replaces GMC's machine control.
Mat design (the editor, templates, auto layout) is moving to a separate app. Background:

- [`GMC_Protocol_Reference.md`](GMC_Protocol_Reference.md): wire protocol, cut sequence, calibration (with citations)
- [`PROGRAM_FORMAT.md`](PROGRAM_FORMAT.md): the program format (the contract between the compiler and the runner)
- [`GMC_Diagnostic_Tool_Spec.md`](GMC_Diagnostic_Tool_Spec.md): calibration/diagnostic routines
- [`docs/hardware-testing.md`](docs/hardware-testing.md): running `gunnar probe` and `gunnar check` at the machine

## Layers

Each layer is a crate. A crate may only depend on crates **below** it. *(planned)* marks crates
not written yet.

```
station     (planned) service on the machine PC: queue, design library, HTTP API + events,
            serves the web app; starts cuts only from the machine's own screen
nest        places mats on sheets of their board type, and in other mats' drop-outs:
            three-stage guillotine layouts by iterative beam search (after PackingSolver);
            offcuts before full sheets; every layout checked before it is returned
compiler    (planned) geometry + profile → Program: flattening, overcut, overlap, cut order
profile     (planned) board types: speed, trims, overcut, overlap, start speed
route       the cut order: which cut next and which way round, so the head travels least,
            within the cutting rules
solvers     route and nest for the browser: one WebAssembly module, their array formats and
            error codes (route and nest themselves stay plain typed libraries)

runner      drives the cutter: session, homed?, head and clamps, programs and manual
            operations; pause, stop now, safe end state, progress events (to implement)
machine     the machine model, pure: profile, mm/deg → steps, each cut lowered as one unit
            (GMC's form, held to its captures, or one stream per cut), soft limits, run time,
            byte budget
controller  VALID commands: one outstanding at a time, matched replies + deadlines,
            mode tracking (typestate), stop lane, async signals
protocol    pure codec: Command → bytes, bytes → Frame/Reply. No I/O, no clock.
link        Reader/Writer halves: serial port | in-memory pair (paced) | byte tap
sim         controller simulator behind an in-memory link (tests, dry runs); checked
            against every GMC capture and session log by the fidelity test

program     the Program format: types, parsing, validation (used by runner and apps)
gunnar      CLI: ports, capture (decode GMC captures), probe, check (staged hardware checks)
```

**Invariant:** commands that must never be sent from this software (mode 9 file transfer,
firmware upload, parameter save `#W`) are not representable in `protocol::Command`.

## Build and test

```sh
cargo test                       # unit tests + replay tests against testdata/captures/g_*.txt
cargo run -p gunnar -- capture testdata/captures/g_init.txt    # decode a GMC serial capture
```
