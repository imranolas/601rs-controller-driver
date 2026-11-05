# 601rs: driver and tooling for the Gunnar 601rs mat cutter

Cross-platform replacement for the GMC host software's machine control. Background:

- [`GMC_Protocol_Reference.md`](GMC_Protocol_Reference.md): wire protocol, cut sequence, calibration (with citations)
- [`PROGRAM_FORMAT.md`](PROGRAM_FORMAT.md): the program format (the contract between designers/queue and the runner)
- [`GMC_Diagnostic_Tool_Spec.md`](GMC_Diagnostic_Tool_Spec.md): calibration/diagnostic routines

## Layers

Each layer is a crate. A crate may only depend on crates **below** it.

```
runner      executes a validated Program; pause / stop-now / progress events
machine     SAFE operations: mm/deg, homed?, head state, soft limits, cut-as-a-unit
controller  VALID commands: one outstanding at a time, matched replies + deadlines,
            mode tracking (typestate), stop lane, async signals
protocol    pure codec: Command → bytes, bytes → Frame/Reply. No I/O, no clock.
link        Port trait: serial | simulator (todo) | capture replay

program     the Program format: types, parsing, validation (used by runner and apps)
gunnar      CLI (currently: decode GMC serial captures)
```

**Invariant:** commands that must never be sent from this software (mode 9 file transfer,
firmware upload, parameter save `#W`) are not representable in `protocol::Command`.

## Build and test

```sh
cargo test                       # unit tests + replay tests against testdata/captures/g_*.txt
cargo run -p gunnar -- capture testdata/captures/g_init.txt    # decode a GMC serial capture
```
