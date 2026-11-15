# 601rs — Calibration (design, draft)

**Status:** a design, 15 Nov 2026; step 1 of §10 is built. Decided with Imran the same day: one file for `mc601` and
Bevel, the profile list in it, the shop's fitted numbers as the defaults, and hooks left for
later (§11). Nothing here is built yet beyond what §1 says exists. It replaces the calibration parts of [`DIAGNOSTIC_TOOL_SPEC.md`](DIAGNOSTIC_TOOL_SPEC.md)
(4.4, 4.5, 4.9). That spec is older than the driver.

**Companions:** [`PROTOCOL_REFERENCE.md`](PROTOCOL_REFERENCE.md) §3.2 (the blade trim), §7 (the
overcut) and §8 (the profiles); [`PROGRAM_FORMAT.md`](PROGRAM_FORMAT.md) §3 (the material) and §8
(the machine profile).

The aim: a framer calibrates the machine and each board profile from **Bevel**. They pick a
routine, cut its sheet at the machine, type in what the calipers say, see what will change, and
accept it. The result is one file that every part reads: the compiler, the runner, `mc601`, and
Bevel's Arrange and Cut. The driver provides everything except the screens. Bevel's screens stay
generic: a new routine needs no new UI.

---

## 1. Where things stand

What exists works. It just can't be reached from an app, and its results don't flow anywhere.

- **The fit** is in the `mc601` binary (`crates/mc601/src/calibrate.rs`). It is a least-squares
  fit of the blade model (each head's line `P`, the slit edges `e`, the drag), with standard
  errors and the overcut combs read by eye. It is good and it stays (§7). But it is driven from
  the terminal, and it gets each cut's carriage position by parsing the serial tap log. Bevel's
  station keeps no such log.
- **The sheet** (`calibration_2`) is cut **raw**: carriage coordinates, run with
  `Profile::raw()`, chosen by `meta.calibration.frame = "carriage"` in `mc601 run`. The runner
  takes its profile once, at `Runner::new`, so Bevel's station can't cut raw without a new mode.
- **Calibrated numbers live in source.** The board origin `(0.225, −0.275)` and the head offsets
  `(−71.45, 2.03)` and `(6.41, 1.88)` are in `Profile::gunnar_601()`. The slit edges are in
  `Blade::fitted()`. Today a calibration means editing Rust and rebuilding.
- **Bevel hardcodes both.** `machine::Profile::gunnar_601()` appears in five places in Bevel
  (`station`, `arrange::time`, `arrange::load`, `arrange::arrange`), and
  `compiler::Blade::fitted()` in one (`arrange::time`). Bevel's profiles in
  `data/materials.json` (C2, C3, C16) have a name and a thickness. Their cutting settings
  (speed, settle, trims) are placeholders.
- **Profile values have no home.** Speed, head settle and blade trims travel in each program's
  `material`. Run-ons, overlaps and run-outs are GMC's numbers in `Blade::gmc()`, the same for
  every board.

## 2. Principles

1. **One file per machine.** It holds the calibrated values only. Everything else is the
   driver's defaults. It is read by everything that cuts or predicts a cut.
2. **A routine is data.** Its plan is a program to cut, the readings to take, a drawing to
   show them on, and the instructions. Bevel renders plans. It never knows what a comb is.
3. **Calibration sheets are ordinary programs** in the board frame. They are cut through the
   normal runner with the current calibration: no raw mode, and every safety check applies. The
   fit learns where each cut went by lowering the program again with the profile it was cut
   with. Lowering is pure, so no log is needed.
4. **Pure.** No I/O and no clock in the library. Bevel and `mc601` store the files and pass in
   the time.
5. **Nothing changes without the operator.** The fit proposes. Apply writes a new revision,
   which records what changed, from what, and why. Every revision is kept, and revert is one
   step.
6. **One forward model.** The value each reading should show is predicted by the current
   calibration. The same prediction gives the "expected about" hint, flags a misread, and
   generates the synthetic readings that test the fit.

## 3. What is calibrated

Two scopes. **Machine** values belong to the machine and its heads. They change after a knock,
a head service or a new machine. **Profile** values belong to a board profile (C2, C3, …) on
this machine's blades. They change with a new board type, a new blade or a different blade
depth.

| Value | Scope | Today | Set by | Used by |
|-------|-------|-------|--------|---------|
| Head offset X, Y (each head) | machine | `Profile::gunnar_601()` | `heads` | lowering: runner, estimate, compiler's travel |
| Scale X, Y (steps/mm) | machine | `Profile::gunnar_601()` (40) | `scale` | lowering |
| Straight slit edges (left, right of travel) | machine | `Blade::fitted()` | `heads` | compiler: where a line goes |
| Bevel edge on the back, reference board | machine | `Blade::fitted()` | `heads` | compiler (unless the profile has its own) |
| Thickness | profile | Bevel's `materials.json` | typed (calipers) | compiler (run-on model), validation |
| Speed %, head settle | profile | Bevel stand-ins | typed | program `material` |
| Bevel trim: straight edges, closed curves | profile | Bevel stand-ins | typed (hooks: later) | program `material` |
| Run-on, start and end (each head) | profile | `Blade::gmc()` | `overcut` | compiler |
| Bevel edge, back and face, on this board | profile | none | `overcut` | compiler; Bevel's face opening |
| Overlap, run-out (closed curves) | profile | `Blade::gmc()` | `ovals` (later) | compiler |

**Held, not fitted:** the board origin. A cut shows only the sum of the board origin and a
head offset (§7), so the origin stays where it is and the head offsets carry the fit. This is
what `mc601 calibrate` does now.

**Not calibration:** the envelope, the park, speeds and ramps, the clamp pads, the controller
settings. They stay in `machine::Profile` as defaults. `mc601 check` covers the hardware.

**Why the scope matters downstream.** Machine values are applied when a program is lowered, so
programs stay valid across a machine calibration (PROGRAM_FORMAT §8). Profile values that place
lines (edges, run-ons) are baked into a program's `path_mm` by the compiler. A program compiled
before a profile changed is stale. So the compiler's caller stamps `meta.calibration_revision`
on every program, and Bevel recompiles a sheet when the stamp is older than the profile it
uses (§9).

## 4. The calibration file

```jsonc
{
  "format": "601rs-calibration",
  "version": 1,
  "revision": 7,                                  // +1 on every change
  "machine": {                                    // over Profile::gunnar_601(); absent: default
    "bevel":    { "offset_mm": [-71.462, 2.041] },
    "straight": { "offset_mm": [6.405, 1.874] },
    "scale":    { "x_steps_per_mm": 40.0, "y_steps_per_mm": 40.0 }
  },
  "blade": {                                      // over Blade::fitted(): the reference board
    "bevel":    { "edge_mm": [0.453, 0.453] },
    "straight": { "edge_mm": [-0.102, 0.087] }
  },
  "profiles": {
    "C2": {
      "name": "Mount board 1.4",
      "thickness_mm": 1.4,
      "speed_pct": 95,
      "head_settle_ms": 200,
      "blade_angle_offset_deg": { "straight": 0, "bevel": 3, "bevel_round": 0 },
      "bevel":    { "run_on_mm": [0.3, 3.6], "edge_mm": [0.47, 0.47], "face_mm": 2.07 },
      "straight": { "run_on_mm": [-0.5, 2.3] }
    }
  },
  "history": [
    { "revision": 7, "at": "2026-11-15T10:42:00Z", "routine": "overcut", "profile": "C2",
      "session": "2026-11-15 10.31 overcut C2",
      "changes": [ { "key": "profile.C2.bevel.run_on.end", "before": 3.7, "after": 3.6, "se": 0.15 } ] }
  ]
}
```

**Rules**

- **Overrides only.** A value that's absent is the driver's default. The file stays short, and
  better defaults in a new driver still apply.
- **The history is append-only.** The current value's provenance (when, which routine, its
  uncertainty) is the last entry that touched it. Revert to revision *n* writes a new revision
  equal to *n*, so revert can be reverted too. A typed value (thickness, speed) is a history entry
  with no routine.
- **Profile ids are the shop's.** The file owns the profile list and each profile's cutting
  values, since they depend on this machine's blades. Bevel's board list refers to a profile by
  id (decided, §11).
- `deny_unknown_fields`, as the other formats. Version 1 refuses other versions.
- **One file for the machine**, shared by `mc601` and Bevel (decided, §11): a calibration taken
  from either is the machine's. The path rule is the driver's, so the two can't drift:
  `calibrate::default_path()`, the crate's one function that isn't pure (it reads environment
  variables only). It gives `$MC601_CALIBRATION` if set, else `calibration.json` beside
  `mc601`'s `config.json`: `~/.config/mc601/` (or `$XDG_CONFIG_HOME/mc601/`), and
  `%APPDATA%\mc601\` on Windows, the shop PC. `mc601` takes `--calibration FILE` too.
- **Two writers, one file.** A write names the revision it read. If the file has moved on since,
  the write is refused and the caller reads again, so neither overwrites the other's change.
- **A missing file is the defaults**, which are the shop's 601 as fitted (decided, §11). Another
  shop's 601 starts there and runs `heads` first. The overview says where each default came from
  ("fitted on the shop's 601, 10 Nov").

**Resolving it.** Everything else asks the file, never the defaults directly:

```rust
impl Calibration {
    /// Every value the driver's default: a machine never calibrated.
    pub fn new() -> Self;
    pub fn from_json(text: &str) -> Result<Self, Error>;
    pub fn to_json(&self) -> String;
    pub fn revision(&self) -> u32;

    /// `Profile::gunnar_601()` with the machine's overrides.
    pub fn machine(&self) -> machine::Profile;
    /// The blade model for cutting this profile's board. For each value: the profile's own,
    /// then the machine's (reference board), then the driver's model. The run-on model is
    /// `run_on_mm + run_on_per_mm × thickness`.
    pub fn blade(&self, profile: &str) -> Result<compiler::Blade, Error>;
    /// The program `material` for this profile: speed, settle, trims.
    pub fn material(&self, profile: &str) -> Result<program::Material, Error>;
    pub fn thickness(&self, profile: &str) -> Result<f64, Error>;

    /// Every value that can be calibrated, with its current value, where it came from
    /// (default, typed, or fitted by a routine on a date, ± its uncertainty): the overview.
    pub fn values(&self) -> Vec<ValueState>;
    /// A typed value: thickness, speed, settle, a profile's name; a new profile.
    pub fn set(&self, key: &Key, value: Typed, stamp: &Stamp) -> Result<Calibration, Error>;
    pub fn revert(&self, to: u32, stamp: &Stamp) -> Result<Calibration, Error>;
}
```

`Key` is a typed path (`HeadOffset { head, axis }`, `RunOn { profile, head, end }`, …). Its JSON
form is the dotted string in the history (`machine.bevel.offset.x`,
`profile.C2.bevel.run_on.end`), so Bevel's TypeScript can key on it. `Stamp` is `{ at, session }`,
given by the caller.

## 5. Routines

| Routine | Scope | Board | What it cuts | Readings | Sets | When |
|---------|-------|-------|--------------|----------|------|------|
| `check` | both | an offcut ≥ 130 × 130 of the profile | a 100 mm square mount with an off-centre 40 mm window, compiled as any job | 6 by calipers | nothing by default: pass, or which routine to run | first thing on a doubtful day; after any change |
| `heads` | machine | an offcut ≥ 140 × 105, 1.2–2 mm | calibration 2's straight strips and bevel window | 11 by calipers | head offsets, straight slit edges, bevel edge (reference board) | new machine; after a knock or head service |
| `scale` | machine | any board ≥ 960 × 760 | four 16 mm slits in the front and left trim bands, 900 and 700 mm apart | 2 by steel rule | scale X, Y | new machine; once a year |
| `overcut` | profile | an offcut ≥ 140 × 105 of the profile | a bevel window and four combs (start and end, each head) | 4 by calipers, 4 by eye | run-ons; bevel edge on the back and face | new profile; new blade; corners not freed or overcut showing |
| `ovals` | profile | later | ovals at overlaps and run-outs | 1 by eye | overlap, run-out | later: GMC's values serve until then |

**A new machine:** `heads`, `scale`, then each profile's `overcut`, then `check`.
**A new profile:** type its thickness, speed, settle and trims, then `overcut`, `check`. **A new blade:**
`overcut` for the profiles it cuts, then `check`.

**Each in brief:**

- **`check`** cuts what the shop cuts: a mount compiled from a sheet, with the current
  calibration, through the compiler. It measures the outside (straight head), the window on the
  back (bevel) and two borders. The borders show the bevel head against the straight head. Each
  reading passes within a tolerance (0.2 mm to start with). The verdict names the routine that
  owns a failure: borders → `heads`, window size → `overcut`. The same fit runs (§7), so its
  numbers are there to see. This is the routine the shop runs often, so it must be short.
- **`heads`** is calibration 2 (`testdata/calibration/calibration_2.html`) without the combs,
  which move to `overcut`. It keeps its strips, its window and its position readings from the
  board's edges, which only it has. It is cut on a profile's board between 1.2 and 2 mm. The plan
  takes that profile for its thickness and speed, and writes machine values.
- **`scale`** costs no board. The slits lie in the front and left 20 mm, which Bevel's trims
  throw away, clear of the clamp pads. The X slits at 40 and 940 fall before F1 (from 75.5) and
  between F3 and F4 (598.5 to 963.5). The Y slits at 40 and 740 fall before L1 (from 80.5) and
  past L3 (to 688.5). The two readings are slit to slit with a steel rule. At 900 mm, a reading to
  0.25 mm pins the scale to 0.03 %. A change is proposed only beyond 0.1 %, which is 1 mm on a
  1 m mount. Squareness is open (question 5).
- **`overcut`** reads the run-on that frees a corner on this board. The combs are calibration 2's,
  each tooth a step further. The setting is the first tooth that reached (§7). The window gives
  the bevel edge on this board on the back and, with the drop-out face up, on the face. The two
  together are the bevel's run across this thickness. Bevel can then show a window's face
  opening exactly.
- **Hooks: later.** A hook is a curve at the start of a cut, more common on the bevel head.
  GMC's answer is a fixed yaw per profile, its blade trim (3° on C10), found with a test
  rectangle (protocol reference §3.2). There is probably a better fix than that. It will be
  looked at on its own, and a routine follows from whatever it finds. Until then each profile's
  trims are typed, the shop's current values carried over, and nothing here changes how a cut
  starts.

**Every plan is tested** to validate (V1–V10), to keep clear of the clamp pads on its smallest
board, and to fit the envelope with the default calibration and with plausible errors in it.

## 6. A routine as data: the plan

```rust
pub enum Routine { Check, Heads, Scale, Overcut }

/// What the overview lists: title, scope, why and when, minutes, the board it needs.
pub fn routines() -> Vec<RoutineInfo>;

/// The plan for a routine, on this profile's board, with this calibration.
pub fn plan(routine: Routine, cal: &Calibration, profile: &str) -> Result<Plan, Error>;

pub struct Plan {
    pub routine: Routine,
    pub profile: String,
    pub board: BoardNeed,          // least size, thickness range, which profile; in words too
    pub program: Program,          // board frame; cut as any program (station.run)
    pub drawing: Drawing,          // the sheet as it will lie: pieces, cuts, labels
    pub steps: Vec<MeasureStep>,   // the readings, grouped by how the board is held
    pub readings: Vec<Reading>,
}

pub struct MeasureStep {
    pub instruction: String,       // "Leave the sheet face down." "Push out the drop-out, face up."
    pub readings: Vec<String>,     // reading ids, in order
}

pub struct Reading {
    pub id: String,                // "W.width"
    pub prompt: String,            // "The window, left to right, on the back"
    pub kind: ReadingKind,
    pub tool: Tool,                // Calipers | Rule | Eye
    pub face: Face,                // Back (as cut, face down) | Face (turned over)
    pub piece: String,             // which piece to hold: "the sheet", "the drop-out", "strip SL"
    pub needs_ops: Vec<usize>,     // the program's ops it depends on (for a stopped run)
    pub dimension: Option<Dimension>, // where to draw it: two points and an axis, board frame
}

pub enum ReadingKind {
    /// mm. Expected: the current calibration's prediction. Beyond `unusual_mm` from it, ask.
    Length { expected_mm: f64, unusual_mm: f64 },
    /// One of several, by eye: comb teeth. `none` allowed where it means something
    /// ("no tooth reached").
    Pick { options: Vec<PickOption>, none: Option<String> },
}
```

The drawing and each reading's dimension are geometry from the driver, so Bevel only draws them
(Bevel's rule: the front end does no geometry). A pick's options carry their places in the
drawing too: tooth 3 is the third line of that comb.

**JSON, one reading:**

```jsonc
{ "id": "W.width", "prompt": "The window, left to right, on the back",
  "kind": { "length": { "expected_mm": 25.91, "unusual_mm": 1.5 } },
  "tool": "calipers", "face": "back", "piece": "the sheet", "needs_ops": [1, 2, 3, 4],
  "dimension": { "from": [67.55, 40.5], "to": [93.45, 40.5], "axis": "x" } }
```

## 7. The fit

The model is `mc601 calibrate`'s, unchanged. Per head, in carriage mm:

- `P`: where the carriage puts the head's line on the registered corner. A cut commanded at
  carriage `c` runs along board `c − P`.
- `e`: where each side of the slit lands from the line. It is positive towards that side. For
  the bevel it is separate on the back and on the face.

A position reading is `c − P + σe`, where σ = ±1 is the kept side along the axis. A width is
two edges' difference, so `P` cancels unless the heads differ. Each is one linear row. The rows
are solved by least squares, with each unknown's standard error and the scatter of the readings
about the model. An unknown the readings don't settle is reported as such and left alone.

**What changes from today:**

- **Carriage positions come from lowering**, not from the log: `machine::lower(program,
  profile_cut_with)`, which is pure. A session keeps the profile it was cut with. `mc601` checks
  the result against its log, so a disagreement between the two is a test failure, not a
  surprise.
- **Picks become values with an uncertainty.** A comb's first tooth that reached gives an
  interval: past the previous tooth, up to this one. The setting is this tooth's value: the
  least run-on known to free the corner. The uncertainty is half the step. Tooth 1, or none,
  means the comb missed: the proposal says so and plans a wider comb next time.
- **Suspects.** After the fit, a reading more than 3 scatters off the model is named: "measure
  W.front again". One bad reading among 13 shouldn't move a head.
- **Verdicts.** Each change is *within the noise* (|Δ| ≤ 2 se: the value is kept unless chosen),
  *moved* (proposed), or *large* (beyond a plausible bound, which needs confirming). The bounds
  start at: head offset 1 mm, scale 0.5 %, edge 0.5 mm, run-on 1.5 mm. A
  large change is more likely a mistake in loading or reading than a fact about the machine.
- **Pooling.** `fit` takes several sessions of one routine and profile, as `fit --all` does now.
  The fit is in absolute terms, so earlier sheets still count. Pooling is always on offer. It is
  never automatic, because a blade change in between makes it wrong.

```rust
/// The value each reading should show, by this calibration: the expected values, and the
/// synthetic readings the tests fit.
pub fn predict(plan: &Plan, cal: &Calibration) -> BTreeMap<String, f64>;

/// Fit one or more sessions of the same routine and profile.
pub fn fit(sessions: &[Session], cal: &Calibration) -> Result<Proposal, Error>;

pub struct Proposal {
    pub readings: usize,
    pub unknowns: usize,
    pub scatter_mm: Option<f64>,
    pub changes: Vec<Change>,
    pub suspects: Vec<Suspect>,       // reading id, how far off the model
    pub notes: Vec<String>,           // "A bevel of 1.71 mm across 1.5 mm board, 48.7° from vertical."
}

pub struct Change {
    pub key: Key,
    pub label: String,                // "Bevel head, X"
    pub before: f64,
    pub after: f64,
    pub se: Option<f64>,
    pub verdict: Verdict,             // WithinNoise | Moved | Large | Undetermined
    pub effect: String,               // "Windows move 0.21 mm to the right."
}

/// The chosen changes as a new revision, with its history entry.
pub fn apply(cal: &Calibration, p: &Proposal, chosen: &[Key], stamp: &Stamp)
    -> Result<Calibration, Error>;
```

**Tests.** The fits of `testdata/calibration` (calibration 1 and 2, cut raw, from their logs)
must give today's numbers. For each routine, a known "true" calibration with errors goes through
`predict`, plus noise, and `fit` must recover it within its standard errors. A plan must fail
loudly with a reading it doesn't list, a pick out of range, or a board of the wrong thickness.

## 8. Sessions: cut, read, fit, apply

A session is one run of a routine. It is serialisable, so Bevel can keep it as a file, resume
it tomorrow, and fit it again later.

```rust
pub struct Session {
    pub id: String,                       // given by the caller: "2026-11-15 10.31 overcut C2"
    pub plan: Plan,                       // as cut
    pub revision: u32,                    // the calibration it was cut with
    pub cut_with: machine::Profile,       // the profile the runner lowered it with
    pub cut: Cut,                         // NotYet | Done | Stopped { ops_done }
    pub readings: BTreeMap<String, Entry>,// value, and "unusual, kept anyway"
}

/// The readings this session can take: those whose cuts all finished.
pub fn available(s: &Session) -> Vec<&Reading>;
/// One value, as it is typed: fine, unusual (ask), or not acceptable (why).
pub fn check(s: &Session, reading: &str, answer: Answer) -> Check;   // a length, or a pick
```

**The flow, as Bevel shows it.**

1. **Prepare.** The plan's board need in words ("An offcut of C2, at least 140 × 105 mm, into
   the front-left stops"), and the drawing of what will be cut.
2. **Cut.** The plan's program goes to the station like any sheet: `station.run(program)`.
   Load, Run, pause, stop, the safe state: Cut's own screens and the runner's rules, unchanged.
   A stop keeps the session: readings that need an op that never finished drop out
   (`needs_ops`). The rest can still be read and fitted.
3. **Measure.** One step at a time (`steps`), one reading at a time. The drawing shows which
   piece to hold, which face up, and the dimension to take. The expected value is faint beside
   the field. Each value is checked as it is typed and saved at once. They can be taken in any
   order, skipped, or finished later. Picks are a row of choices, not a number to type.
4. **Result.** The proposal: each change before → after, ± its uncertainty, its verdict and its
   effect. *Moved* changes are ticked, *within the noise* ones aren't, and *large* ones ask.
   Suspects come first: measure again or carry on. The fit is cheap, so the result can update
   live as readings come in.
5. **Apply.** A new revision is written. Programs compiled before it are recompiled before they
   are cut (§3). **Revert** is on the history.

## 9. What changes

**In the driver**

| Where | Change |
|-------|--------|
| new crate `calibrate` | The file (§4), the routines and plans (§5, §6), the fit, sessions and apply (§7, §8). Pure. Depends on `compiler`, `machine` and `program`, so it sits beside `nest` in the README's table. It replaces the planned `profile` crate: a profile's values are its file's. |
| `compiler` | No new signature: `compile` already takes a `Blade` and a `Profile`, from `Calibration::blade` and `machine`. `Blade::fitted()` stays as the default model. |
| `machine` | `Profile::gunnar_601()` keeps the shop's fitted numbers as the defaults (decided, §11). Each fitted value names the record it came from, as it does now. |
| `runner` | `Runner::set_profile(profile)` between jobs (busy otherwise), so a new revision applies without reopening the session. Homing stays valid: the profile changes the arithmetic, not where the carriage is. It re-runs `configure` if the controller settings differ. |
| `mc601` | `calibrate` becomes a terminal front end over the crate: `mc601 calibrate <routine> [--profile C2]` (cut, read, fit, apply); `mc601 calibration show \| history \| revert N \| set KEY VALUE`. `run` and `check` read the file. `calibrate fit --sheet` keeps fitting the old raw sheets from their logs, for the record and the golden tests. |

**In Bevel** (for Bevel's own spec; the driver is designed to make it possible):

- **The Tauri shell** loads the file once and passes it on. It gives `cal.machine()` to
  `Station::open`, and `cal.blade(p)` and `cal.material(p)` to Arrange's times and Cut's
  compile. That replaces the five hardcoded defaults. After an apply, it calls the runner's
  `set_profile` and marks Arrange's cached times stale.
- **The pure parts** (`routines`, `plan`, `predict`, `check`, `fit`, `apply`) can go in
  `design-wasm`, as Arrange's do. The browser build then runs the whole flow against the
  simulator. Storage and the station stay in the shell, as Cut's do.
- **A Machine page**, beside the three steps or under settings, holds:
  - the overview (`values()`: each value, where it came from, when, ±), with `check` at the top;
  - each profile's state;
  - the history with revert.
- **A routine** runs as Prepare → Cut (Cut's own Load and Run) → Measure → Result.
- **Profiles move** from `data/materials.json` into the calibration file (decided). Board codes
  keep their profile id. Thickness, speed, settle and trims are typed on the Machine page.
  Run-ons come from `overcut`, or are typed with a note in the history.

## 10. Order of work

Each step passes `cargo test` and stops for review.

1. **`calibrate`: the fit, moved.** *Built, 15 Nov.* The sheet (`calibrate::sheet`), where its
   cuts went (`calibrate::sent`) and the fit (`calibrate::fit`) are out of `mc601`, unchanged:
   `mc601 calibrate fit` prints the same report and writes the same profile, and `calibrate
   sheet` the same program. A sheet's cuts come from lowering (`Sheet::lowered`) as well as
   from a log, by one reading of the commands. Golden, in `mc601`: both runs of calibration 1
   lower exactly as their logs show they were sent, each with the profile it was cut with; and
   its fit, from the log or the lowering alike, is the one printed before the move. Calibration
   2 has no readings in `testdata` yet, so it is held by the synthetic fit only.
2. **The file.** `Calibration`: overrides, resolution (`machine`, `blade`, `material`),
   `values`, `set`, history, revert. `mc601 run` and `check` read it. Calibration 2's fit applied
   to `Calibration::new()` gives today's `gunnar_601` offsets.
3. **Plans, sessions, proposals.** `heads` and `overcut` as plans in the board frame, with
   steps, drawing and dimensions; `predict`, `check`, `fit` over sessions, verdicts, suspects,
   `apply`. Synthetic recovery tests. `mc601 calibrate heads|overcut` on the sim, end to end.
4. **The runner.** `set_profile`.
5. **`check` and `scale`.**
6. **A request to Bevel**, written there by Bevel's own process: what §9 lists.

`ovals` waits for a shop complaint about joins: GMC's overlaps serve until then. Hooks wait
for their own look (§5).

## 11. Decided, and still open

**Decided with Imran, 15 Nov:**

1. **One file for `mc601` and Bevel.** Both drive the same machine from the one shop PC, so a
   calibration from either is the machine's (§4).
2. **The profile list is in the calibration file.** A profile's numbers depend on this
   machine's blades. Bevel's board list refers to a profile by id (§4, §9).
3. **The shop's fitted numbers are the defaults** in `Profile::gunnar_601()` and
   `Blade::fitted()`. A missing file is the shop's 601 as fitted. GMC's numbers stay as
   `Blade::gmc()` for the golden tests.
4. **Hooks are left for later.** A hook is a curve at the start of a cut, more common on the
   bevel head. GMC's per-profile yaw is one fix, probably not the best. It will be looked at on
   its own, and no `trim` routine is planned until then (§5).

**Still open:**

5. **Squareness.** A third slit would give the diagonal, but 1.1 m is past a steel rule.
   Measure and report only, or leave it out?
6. **Blade changes.** Should Bevel record "blade changed" (Cut's Load already asks for one), so
   the Machine page can suggest `overcut` for the profiles that blade cuts?
