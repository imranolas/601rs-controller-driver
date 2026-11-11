# Testing against the machine

Every command that talks to the machine is a session: a dated folder under the data folder
(`~/Documents/Gunnar/sessions/` unless `gunnar config --data-dir` says otherwise) with the log of
every byte in both directions, copies of the programs, and any readings and fits.
`gunnar logs` lists them; `open "$(gunnar logs --last)"` opens the latest. Copy the logs worth
keeping into `testdata/sessions/`: `cargo test` replays each one against the simulator and
points at every reply where the simulator differs from the box (see the README there).

`gunnar --help` and `gunnar <command> --help` list every option.

## Before you go

```sh
cargo install --path crates/gunnar      # puts `gunnar` on the PATH (~/.cargo/bin)
gunnar probe --sim                      # dry run, read-only
gunnar check --sim                      # dry run of every stage (~45 s)
gunnar check --sim-limits               # same, with limit inputs mapped (stage 7 path)
```

## At the machine

1. **Close GMC.** Only one program can hold the port.
2. Connect a USB-serial adapter and the null-modem cable to the controller box.
3. The port: with one USB-serial adapter plugged in, `gunnar` uses it. Otherwise `gunnar ports`
   lists them; give one with `--port`, or remember it with `gunnar config --port PORT`.
4. **Stage 1, read-only:** `gunnar probe`
   - No motion, no parameter writes; one `@M1`/`@M0` mode switch.
   - If it cannot connect, the error says whether nothing came back (cable, power), bytes
     but no reply (line settings), or replies that were not the driver's `@I100` probe.
   - Answers: baud/framing, whether the firmware answers the resync probe inputs
     `@I100`–`@I119`, v1.00 commands on firmware 2.00, which inputs are mapped,
     and whether it needs GMC's licence exchange (`@M1 accepted` vs `*007`).
   - Reads back every parameter GMC writes. The line **box vs GMC at connect** says whether
     the box boots with GMC's configuration or a driver must write it at connect, as GMC does.
5. **Stages 2–8:** `gunnar check --stage 2`, then
   `--stage 3`, and so on (or `--stage 2-8`). Each step says what it will do and waits for
   Enter. `s` skips the step. `q` at any prompt ends the run: the stage in hand winds down
   without asking anything more, and the parameters are restored.

| Stage | Moves | What it checks |
|-------|-------|----------------|
| 2 | no | replies come back in the order sent (a command and a probe in one write, 20 times each for a write, `@M0` and a refusal); `#C8` heartbeat `0x05`; `#C35` green-button bytes `0x06`/`0x07` (you press the button) |
| 3 | yes | homing exactly as GMC does it, heads up first and never on a home switch (you say which way Y moved); GMC's V-groove switch reset at its speeds, up the left side at 900 mm/s and 3 cm on against the stub at the back (only if Y homed towards you; you say whether the switch is at normal cutting); then the park (you confirm it is GMC's, or the run stops), and 20 mm jogs into the table and back (you say whether +Y went away from you); `@B` brakes a jog; `@Q249` follows `$A` bit 7 (spare output); outputs across a mode switch; which clamp each bit closes (you answer) |
| 4 | yes | 20 mm square in mode 2, GMC style; `@X` 15 → 14 → 04; exact return; `@B` during the square discards the rest of the path instead of resuming it (stop-now depends on this) |
| 5 | yes | in-stream `A`/`W`/`I221`/`C`; `@N` progress (does `C` count? the command running or those finished? does it wrap after 9999?); `C1` skips the dead time; `@O50` with `C16`/`C8`; the stream waits for the green button (you press it); a compact path (relative steps without the `L20,` prefix) lands exactly; a second speed entry gives its own feed; whether in-stream `G` applies in sequence or on receipt |
| 6 | yes | buffer size via `@H`; which of `0x02`/`0x03` means "full" (the spec contradicts itself); a 22 KB stream with `#C7,1` and `#C19,400` completes; streaming at full speed does not overrun (else 0x03 comes too late over this link); `@B` while the stream is held discards the queued path, nothing moves afterwards, and `0x02` follows |
| 7 | yes | `@S` loses the position (then re-homes); if the X limit input is mapped, drives slowly into it, but no further than 20 mm past GMC's drivable range (X 1560 mm): async `*033`, `@Y` limit bit, recovery |
| 8 | yes | **blades out**: each head down as GMC lowers it, nothing moving (does `i_b` bypass the throttle, so `05` is fast and `01` slow? the lift `00`; the straight head `0A`: you answer); a quarter turn of Z alone, timed, and inside a stream as a corner (does the carriage stand still while the blade turns?); how long after `@X04` a trailing `A` switches; a stream held at the green-button wait filled with 1 mm moves until a flow byte (up to 512 KB): buffer size, which byte means full, `#C19` fed by `@H`, `@B` empties it (don't press the button) |

## Safety

- Someone at the e-stop for every stage that moves. Scrap board, clear table, head up.
- Homing raises both heads first (`$A02`). Every other stage that moves starts with the heads
  up and the carriage at the park (it asks before travelling there): GMC's end-of-job
  position, clear of the board's corner and the home switches.
- Moves are 20–100 mm from the park, into the table, at 30 mm/s by default (`--speed` in
  steps/s, 100–4000). They stay inside the soft limits even if a stop under test fails. Two
  moves leave them: stage 3's V-groove switch reset, GMC's own after homing and at its speeds
  (3 cm past the soft limit, against the stub at the back), and stage 7's limit-switch drive,
  only if the controller can see that switch.
  A controller whose homing direction (`&D`) isn't the profile's gets no moves at all.
- The heads stay up: only the spare bit `0x80` and, after asking, the clamps are switched.
  Stage 8 lowers each head, but only after you say both blades are out, and nothing moves.
- Only Enter starts a step; anything but `s` or `q` asks again.
- **Ctrl-C brakes (`@B`) and exits.** A stage that fails also brakes. Either way the settings
  the run changed are not restored: power-cycle the controller before using it again, with GMC
  too (GMC doesn't rewrite all of them).
- `check` only runs on the machine from a terminal, and a closed stdin ends the run: no step
  ever goes ahead without someone pressing Enter.
- Nothing is saved (`#W`, `@R` and `@M9` are never sent). Parameters a stage changes are
  read first and written back at the end; if a run aborts, power-cycling the box or starting
  GMC restores them.
- Stage 7 only drives into a limit switch if the controller can see it (`&I243`/`&I239`
  mapped). The captures show `&I240,000h` (Y− not mapped), so expect a SKIP.

## Calibrating the heads

Once per blade or holder change, cut the machine-and-head sheet raw: no profile, offset or trim
is applied, so nothing it measures inherits an earlier estimate. The layout and what to measure
are in `testdata/calibration/calibration_2.html`.

```sh
gunnar calibrate cut          # write the sheet, cut it raw, then take its readings and fit them
gunnar calibrate read         # take (or correct) the latest session's readings later
gunnar calibrate fit --all    # fit every calibration session that has readings, together
```

- Offcut 140 × 105 mm or larger, 1.5 mm, in the front-left corner against the left clamp.
- After the cut, `gunnar` asks for each reading in turn: what to measure, on which face, and
  roughly what to expect. A value far from that is questioned before it is kept. Comb answers
  are tooth numbers (0 if none reached). Each value is saved as it is typed, so stopping early
  loses nothing; `calibrate read` shows the saved values and Enter keeps them.
- The fit reads each cut's exact carriage steps from the run log, so the profile current at the
  time doesn't matter. Its report and a profile with the fitted head offsets
  (`fit.txt`, `profile.json`) go into the session. Change the profile only where the fit says
  a value has moved (more than twice its uncertainty).
- A sheet kept outside the data folder fits too: `gunnar calibrate fit --sheet PROGRAM LOG
  READINGS` (calibration 1: `testdata/calibration/calibration_1.json`, `.run.log`,
  `.readings.txt`), alone or with sessions.
