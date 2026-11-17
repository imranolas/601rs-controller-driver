# Testing against the machine

Every command that talks to the machine is a session: a dated folder under the data folder
(`~/Documents/mc601/sessions/` unless `mc601 config --data-dir` says otherwise) with the log of
every byte in both directions, copies of the programs, and any readings and fits.
`mc601 logs` lists them; `open "$(mc601 logs --last)"` opens the latest. Copy the logs worth
keeping into `testdata/sessions/`: `cargo test` replays each one against the simulator and
points at every reply where the simulator differs from the box (see the README there).

`mc601 --help` and `mc601 <command> --help` list every option.

## Before you go

```sh
cargo install --path crates/mc601      # puts `mc601` on the PATH (~/.cargo/bin)
mc601 probe --sim                      # dry run, read-only
mc601 check --sim                      # dry run of every stage (~45 s)
mc601 check --sim-limits               # same, with limit inputs mapped (stage 7 path)
```

## At the machine

1. **Close GMC.** Only one program can hold the port.
2. Connect a USB-serial adapter and the null-modem cable to the controller box.
3. The port: with one USB-serial adapter plugged in, `mc601` uses it. Otherwise `mc601 ports`
   lists them; give one with `--port`, or remember it with `mc601 config --port PORT`.
4. **Stage 1, read-only:** `mc601 probe`
   - No motion, no parameter writes; one `@M1`/`@M0` mode switch.
   - If it cannot connect, the error says whether nothing came back (cable, power), bytes
     but no reply (line settings), or replies that were not the driver's `@I100` probe.
   - Answers: baud/framing, whether the firmware answers the resync probe inputs
     `@I100`–`@I119`, v1.00 commands on firmware 2.00, which inputs are mapped,
     and whether it needs GMC's licence exchange (`@M1 accepted` vs `*007`).
   - Reads back every parameter GMC writes. The line **box vs GMC at connect** says whether
     the box boots with GMC's configuration or a driver must write it at connect, as GMC does.
5. **Stages 2–8:** `mc601 check --stage 2`, then
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
| 7 | yes | `@S` loses the position (then re-homes); if the X limit input is mapped, drives slowly into it, but no further than 20 mm past where the carriage goes (X 1641.5 mm, the bevel head at GMC's limit): async `*033`, `@Y` limit bit, recovery |
| 8 | yes | **blades out**: each head down as GMC lowers it, nothing moving (does `i_b` bypass the throttle, so `05` is fast and `01` slow? the lift `00`; the straight head `0A`: you answer); a quarter turn of Z alone, timed, and inside a stream as a corner (does the carriage stand still while the blade turns?); how long after `@X04` a trailing `A` switches; a stream held at the green-button wait filled with 1 mm moves until a flow byte (up to 512 KB): buffer size, which byte means full, `#C19` fed by `@H`, `@B` empties it (don't press the button) |

**The room below zero.** The driver takes the carriage to have 7.5 mm below zero after homing:
GMC's `#OX,300` and `#OY,300` leave each axis that far off its switch before zeroing it
(SPEC1 p.30), and cuts to the board's left and front edges use it (the straight head reaches
1.3 mm past the left edge). The simulator models it the same way. Before cutting a real sheet that
runs to the edge, once stage 3 has homed: cut a scrap board with a straight cut run on past its
left edge (a sheet with a clean-up line to the edge does). It must finish without `*032` (X
minus) or `*034` (Y minus). If either comes, the room is smaller than assumed: say so, and
the driver's reach below zero (`machine::Profile::reach_mm`) changes to match.

**The far end.** The driver lets the carriage go to X 1641.5 mm, where the bevel head's blade
meets GMC's limit (1 cm past `DRX`), as Cut Other End and a jumbo board need. No session yet has
the carriage past the park, X 1471.5 mm. Before a job relies on it: run a scrap jumbo-sized
sheet, or a straight cut along a board's right edge with the bevel head, near the right-hand
clamp, with a hand on the stop. It must finish without `*033` (X plus) or a knock at the end of
the rail. If either comes, say so: the driver's reach (`machine::Profile::envelope_mm`) changes
to match.

## Safety

- Someone at the e-stop for every stage that moves. Scrap board, clear table, head up.
- Homing raises both heads first (`$A02`). Every other stage that moves starts with the heads
  up and the carriage at the park (it asks before travelling there): GMC's end-of-job
  position, clear of the board's corner and the home switches.
- Moves are 20–100 mm from the park, into the table, at 30 mm/s by default (`--speed` in
  steps/s, 100–4000). They stay inside the soft limits even if a stop under test fails. Two
  moves leave them: stage 3's V-groove switch reset, GMC's own after homing and at its speeds
  (past the carriage's Y travel, against the stub at the back), and stage 7's limit-switch drive,
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

Stage 1 (CALIBRATION.md §5), on a new machine or after a knock or head service: a 100 mm mount
with a 50 mm bevel window on a 150 × 150 mm offcut, and 16 caliper readings. It sets each head's
position, the straight slit's offset and width, and the bevel's offset and lean, in the
calibration file `mc601` shares with apps.

```sh
mc601 calibrate heads --thickness 1.42   # stage 1: plan, cut, then take its readings, fit, offer the changes
mc601 calibrate runons --thickness 1.42  # stage 2: the combs, after stage 1, once per blade
mc601 calibrate read                     # take (or correct) the latest session's readings later
mc601 calibration history                # what was written, when, by which session
```

- An offcut at least 150 × 150 mm, 1.2 to 2 mm board best, face down into the front-left stops.
  Measure its thickness with calipers first: the bevel's lean is its run across it.
  `--profile ID` cuts at a profile's speed and settle, and takes its thickness unless given.
- After the cut, `mc601` asks for each reading in turn, grouped by how the pieces are held: what
  to measure, on which face, and what the calibration expects. A value more than 1 mm from that
  is questioned before it is kept. Each value is saved as it is typed, so stopping early loses
  nothing; `calibrate read` shows the saved values and Enter keeps them.
- Where the cuts went comes from lowering the session's plan again with the profile it was cut
  with, so no log is needed. The fit, and what it proposes for each value (before → after ± its
  uncertainty: moved, within its noise, or large), go into the session's `fit.txt`. Moved values
  are written as one revision if you say so; large ones only if you confirm them; nothing while
  a reading is suspect (measure it again). `mc601 calibration revert N` undoes it.
- A run that stops keeps its session: `mc601 calibrate read` then asks only for the readings
  whose cuts were made. Two stage 1 offcuts fit together with `mc601 calibrate fit DIR DIR`
  (a report, about 1.4 times as precise), if one alone scatters more than you'd like.
- Stage 2 cuts eight combs on another 150 × 150 offcut of a common board. Leave the board back
  up and flex it along each comb's reference: enter the first tooth that comes free there,
  counting from the left (tooth 1 is the shortest), or 0 if none does. A comb where none, or
  every one, comes free moves the file's value so the next stage 2 is centred past it: cut it
  again. Once on 2.5 to 2.7 mm board too, to see whether the reach changes with thickness.
- `mc601 calibrate fit --all` pools every stage 1 session that has readings.

## Starting cuts on thick board

Before stage 2's combs (CALIBRATION.md §5), since how a cut starts changes how far it reaches,
compare the driver's start with GMC's. GMC's own jobs are ready as programs:

```sh
mc601 run testdata/programs/g_multi_C12.json                     # twin windows, MSK_4.1: the bevel starts at 10 mm/s
mc601 run testdata/programs/g_multi_C12.json --gmc-bevel-start   # the same, as GMC: the bevel jumps to 100 mm/s
mc601 run testdata/programs/g_oval_mount.json                    # the oval moves off 100 ms after its blade starts down, as GMC
mc601 run testdata/programs/g_oval_mount.json --oval-settle      # the same, the blade fully down first
```

- The windows on MSK_4.1 (at least 371 × 321 mm), into the front-left stops as GMC lays them.
  Look at the start of each bevel edge on the face: the wobble, if any, and whether the corners
  come free. Note which run was which.
- The oval on any 1.5 mm board (at least 201 × 291 mm): look at the join where the cut started,
  on the face.
- Keep the better of each pair by leaving its switch off (the default) or saying so: the
  defaults are the driver's soft start and GMC's oval entry.
