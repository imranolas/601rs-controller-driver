# Session logs from the machine

Put the logs that `mc601` writes at the machine here: the `<cmd>.log` in a session folder,
renamed to say what it was (the extension must be `.log`). The `gunnar-<cmd>-<time>.log` files
are from before the command was renamed.

`cargo test` then replays every one of them into the simulator
(`crates/sim/tests/fidelity.rs`) and requires the replies the box gave. A difference is a fact
about the box the simulator does not know yet, and the report points at the reply:

```
gunnar-check-…log: 1383 real vs 1383 simulated replies; first difference at #1356
  #1356  real            &I243 → "&I243,001h,0,0"          sim            &I243 → "&I243,000h,0,0"
```

Fix the simulator, or `SimConfig::default()` (which describes the 601), until the log replays.
From then on the log is a regression test: the simulator cannot drift from the box again.

What a replay cannot check, because it depends on timing or the physical machine rather than on
the commands: `@X` status, positions, inputs, switches, outputs read back, buffer level and
progress. Those values are compared only by which query they answer. The behaviour behind them
is what `mc601 check` itself reports as PASS/FAIL, and each finding there needs its own fix in
the simulator and an end-to-end test (`crates/sim/tests/controller.rs`).

Button presses replay if the box reported them (`#C35`, bytes 0x06–0x09). Anything else the
operator did by hand does not.
