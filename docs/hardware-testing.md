# Testing against the machine

Two commands, both from the workspace root. Every byte in both directions is written to a
timestamped log (`gunnar-<cmd>-<time>.log`, or `--log FILE`). Bring the logs back: each one
becomes a capture we can replay against the simulator and cite in the docs.

## Before you go

```sh
cargo build --release -p gunnar
./target/release/gunnar probe sim            # dry run, read-only
./target/release/gunnar check sim            # dry run of every stage (~30 s)
./target/release/gunnar check sim-limits     # same, with limit inputs mapped (stage 7 path)
```

## At the machine

1. **Close GMC.** Only one program can hold the port.
2. Connect a USB-serial adapter and the null-modem cable to the controller box.
3. Find the port: `./target/release/gunnar ports` (look for `/dev/cu.usbserial-…`).
4. **Stage 1, read-only:** `./target/release/gunnar probe /dev/cu.usbserial-XXXX`
   - No motion, no parameter writes; one `@M1`/`@M0` mode switch.
   - If it cannot connect, the error says whether nothing came back (cable, power), bytes
     but no reply (line settings), or replies that were not the driver's `@I100` probe.
   - Answers: baud/framing, whether the firmware answers the resync probe inputs
     `@I100`–`@I119`, v1.00 commands on firmware 2.00, which inputs are mapped,
     what the box boots with versus what GMC sets, and whether it needs GMC's licence
     exchange (`@M1 accepted` vs `*007`).
5. **Stages 2–7:** `./target/release/gunnar check /dev/cu.usbserial-XXXX --stage 2`, then
   `--stage 3`, and so on (or `--stage 2-7`). Each step says what it will do and waits for
   Enter (`s` skips the step, `q` at a stage prompt ends the run).

| Stage | Moves | What it checks |
|-------|-------|----------------|
| 2 | no | `#C8` heartbeat `0x05`; `#C35` green-button bytes `0x06`/`0x07` (you press the button) |
| 3 | yes | homing exactly as GMC does it; 20 mm jogs away from home and back; `@B` brakes a jog; `@Q249` follows `$A` bit 7 (spare output); outputs across a mode switch; which clamp each bit closes (you answer) |
| 4 | yes | 20 mm square in mode 2, GMC style; `@X` 15 → 14 → 04; exact return; `@B` during the square discards the rest of the path instead of resuming it (stop-now depends on this) |
| 5 | yes | in-stream `A`/`W`/`I221`/`C`; `@N` progress (does `C` count?); `C1` skips the dead time; `@O50` with `C16`/`C8`; the stream waits for the green button (you press it) |
| 6 | yes | buffer size via `@H`; which of `0x02`/`0x03` means "full" (the spec contradicts itself); a 22 KB stream with `#C7,1` and `#C19,400` completes |
| 7 | yes | `@S` loses the position (then re-homes); if the X limit input is mapped, drives slowly into it, but no further than 20 mm past GMC's drivable range (X 1560 mm): async `*033`, `@Y` limit bit, recovery |

## Safety

- Someone at the e-stop for every stage that moves. Scrap board, clear table, head up.
- Moves are 20–100 mm, at 30 mm/s by default (`--speed` in steps/s, 100–4000), always
  *away* from the home switches (direction read from `&D`).
- The head stays up: outputs are `0x02`; only the spare bit `0x80` and, after asking, the
  clamps are switched.
- **Ctrl-C brakes (`@B`) and exits.** A stage that fails also brakes.
- `check` only runs on the machine from a terminal, and a closed stdin ends the run: no step
  ever goes ahead without someone pressing Enter.
- Nothing is saved (`#W`, `@R` and `@M9` are never sent). Parameters a stage changes are
  read first and written back at the end; if a run aborts, power-cycling the box or starting
  GMC restores them.
- Stage 7 only drives into a limit switch if the controller can see it (`&I243`/`&I239`
  mapped). The captures show `&I240,000h` (Y− not mapped), so expect a SKIP.
