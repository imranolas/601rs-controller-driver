# 601rs — Calibration (design, draft)

**Status:** a design, 15–16 Nov 2026. Steps 1 and 2 of §10 are built, and step 3's fit. On
16 Nov, after tracing GMC's cut geometry in its code (protocol reference §3.3, §7, §7.1):
the tool model (§3) and the routines rebuilt around it (§5). Decisions are in §11. It replaces the calibration parts of [`DIAGNOSTIC_TOOL_SPEC.md`](DIAGNOSTIC_TOOL_SPEC.md)
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
4. **Pure, but for the store.** No clock in the library: Bevel and `mc601` pass in the time.
   Its only I/O is `calibrate::store`, the calibration file's path, load and save, kept there so
   the two front ends that share the file can't disagree about it.
5. **Nothing changes without the operator.** The fit proposes. Apply writes a new revision,
   which records what changed, from what, and why. Every revision is kept, and revert is one
   step.
6. **One forward model.** The value each reading should show is predicted by the current
   calibration. The same prediction gives the "expected about" hint, flags a misread, and
   generates the synthetic readings that test the fit.

## 3. What is calibrated: the tool, not the board

*Adopted 16 Nov (§11). It replaces the per-profile run-ons and edges of the first draft.*

**Words.** The **face** is the customer's side of the board; the 601 cuts it face down, so the
face lies on the sheets under the board. The **back** is up, where the blade enters, and where
Bevel's designs give a window's edge. "On the face" always means the customer's side, never
whichever side is up.

GMC calibrates numbers per head and patches each board in its profile (protocol reference §7):
its thickness rule takes every bevel as 45°, its straight run-on ignores thickness, and the
shop's profiles raise the thickness of the thicker boards (MSK_2.5 set to 2.7 mm, MSK_4.1 to
4.6, the 4.6 mm board to 5.0 and a 0.5 mm `OCAddon`) to make up for it. This design describes
each head physically instead, and works everything else out from the board's thickness: edges
on both faces, the bevel's run, the run-ons, corners at any angle, for any board.

**Per head** (both heads alike; the straight head is a lean of 0°):

| Value | What it is | From |
|-------|------------|------|
| Position X, Y | where the carriage puts the head's line on the registered corner (`P`, §7) | stage 1 |
| Sideways offset | where the cut sits across the line, in the blade's frame: it turns with the direction of travel, which is how cutting both ways tells it from the position | stage 1 |
| Slit width | how far the cut's two sides are apart; about none (the board closes behind the blade) | stage 1 |
| Lean | the blade's angle from vertical across the cut: 0° straight, about 48° bevel (GMC's `Inclination` 42° is to the board). The bevel runs `tan(lean)` mm across per mm of board | stage 1 (back and face) |
| Start lead, end shortfall | how far the cut starts before the commanded start and stops short of the commanded end, **on the back and on the face**: measured, not derived (below) | stage 2, calibration 1 on the back |
| Blade edge angle | 37.5° to the spine, one blade in both heads (measured 13 Nov from a photo) | known |

**Per board:** the thickness, **measured with calipers**, never GMC's profile value, which was
raised to lengthen the run-ons this model works out itself. Speed, head settle and trims stay
per profile; they aren't geometry. The tip depth needs no value of its own: stage 1 reads
each face's edge and stage 2 each face's reach directly, and the shop keeps the depth to no
more than four printer sheets under a board.

**What follows** (a face at height `h` above the tip: the face at `d`, the back at `d + t`):

- **Across:** the cut on a face sits `sideways + h × tan(lean)` from the line, its sides half
  the slit width either side. The bevel's run across the board is `t × tan(lean)`.
- **Along:** measured, on each face, with combs (§5). The head plunges straight down and lifts
  straight up (Imran, 16 Nov), so where a cut starts on a face is the blade's cross-section at
  that height as it plunges, and where it ends is the cutting edge at that height as it lifts:
  fixed by the blade and the head, which is why it can be measured once and carried to other boards. The blade's shape would
  predict it given which way its edge faces and where the tip sits, but calibration 1 can't
  tell those apart: worked through with its back readings, a face that simply trails or leads
  the back by the edge angle would leave a GMC C10 window joined at its corners on the face
  (about 0.7 mm at each cut's end, or 2.8 mm at each start), and those windows come out. So
  each head's start lead and end shortfall are measured on both faces, once per blade; a second
  board says whether they change with thickness.
- **Corners:** at a corner turning by `φ`, each face's corner sits where that face's edges meet,
  so the run-on needed on a face is the neighbouring edge's offset on that face × `tan(φ ÷ 2)`
  plus that face's end shortfall (or, at a start, less its start lead). A run-on is the least
  that frees the corner on both faces, plus a chosen margin.

**What calibration 1 already fixes** (its fit, 10 Nov, 1.5 mm board as named, not measured):

| Value | Fitted |
|-------|--------|
| Straight slit width | −0.015 ± 0.080 mm: none |
| Straight sideways offset | 0.095 ± 0.040 mm right of travel |
| Bevel back edge, face edge | 0.45 and 2.16 ± 0.06 mm outside the line |
| Bevel run across the board | 1.71 ± 0.08 mm: a lean of 48.7° ± 1.3° |
| On the back: start lead, end shortfall | straight 1.97, 1.20; bevel 2.22, 0.65 ± 0.13 mm |
| The plunge (lead less shortfall) | straight 0.77, bevel 1.58 ± 0.19 mm |

**What it can't fix:**

1. **The start lead and end shortfall on the face.** Calibration 1 read its overruns on the
   back only. Its four overrun readings taken again on the face, on the same sheet, give them
   for a program we know exactly; failing that, the face corners of a window GMC cut on C10
   (does a cut run on past each corner, along which edge, how far); stage 2 measures them.
2. **Whether the reach changes with thickness.** The combs (§5) cut on a 2.5–2.7 mm board as
   well as a common one settle it without thick board. GMC's straight run-on doesn't change
   with thickness at all, and the shop finds thick boards unreliable.

**What isn't geometry** (blade bending in dense board, fibres tearing on the face, the cut
springing closed, drag with speed, the wobble at a cut's start) becomes a small correction per
head that starts at zero, allowed per board only if a comb or a check shows one board behaving
differently. Hooks (§5) are steering, not geometry, and stay separate.

**Held, not fitted:** the board origin. A cut shows only its sum with a head's offset (§7);
its (0.225, −0.275) mm is GMC's straight head's sideways shift (protocol reference §7).

**Not calibration:** the envelope, the park, speeds and ramps, the clamp pads, the controller
settings. They stay in `machine::Profile`. `mc601 check` covers the hardware.

**Downstream.** The head's position is applied when a program is lowered, so programs stay
valid across it (PROGRAM_FORMAT §8). The rest of the tool places lines and run-ons, which the
compiler bakes into `path_mm`, so a program compiled before the tool changed is stale: the
compiler's caller stamps `meta.calibration_revision`, and Bevel recompiles a sheet when the
stamp is older than the calibration (§9). With per-profile values gone, only a tool change or a
board's thickness makes a program stale.

## 4. The calibration file

*Built (§10, step 2): `calibrate::calibration`, `calibrate::key`, `calibrate::store`.*

```jsonc
{
  "format": "601rs-calibration",
  "version": 1,
  "revision": 7,                                  // +1 on every change; 0: never changed
  "values": {                                     // each over its default; absent: the default
    "machine.bevel.offset.x": -71.462,
    "machine.bevel.offset.y": 2.041,
    "blade.straight.edge.left": -0.102,
    "profile.C2.name": "Mount board 1.4",
    "profile.C2.thickness_mm": 1.4,
    "profile.C2.speed_pct": 95.0,
    "profile.C2.head_settle_ms": 200.0,
    "profile.C2.trim.bevel": 3.0,
    "profile.C2.bevel.run_on.end": 3.6
  },
  "history": [
    { "revision": 7, "at": "2026-11-15T10:42:00Z", "routine": "runons",
      "session": "2026-11-15 10.31 runons",
      "changes": [ { "key": "profile.C2.bevel.run_on.end", "before": 3.7, "after": 3.6, "se": 0.15 } ] }
  ]
}
```

**Keys.** Each value is one number (or, for a profile's name, text) under a dotted key. One
value is one history line, so setting an X offset never carries its Y along, and revert is
exact. `calibrate::key::Key` is the typed form; its text is what the file, the history,
`mc601` and Bevel's TypeScript use.

| Key | Unit | Default |
|-----|------|---------|
| `machine.<head>.offset.<x\|y>` | mm | `Profile::gunnar_601()` |
| `machine.scale.<x\|y>` | steps/mm | `Profile::gunnar_601()` (40) |
| `blade.<head>.edge.<left\|right>` | mm, on the back | `Blade::fitted()` |
| `blade.<head>.run_on.<start\|end>` | mm, on board of no thickness | `Blade::fitted()` |
| `blade.<head>.run_on_per_mm` | mm per mm of board | `Blade::fitted()` |
| `blade.<head>.overlap_ms`, `blade.<head>.run_out_mm` | ms, mm | `Blade::fitted()` |
| `profile.<id>.name`, `.thickness_mm`, `.speed_pct`, `.head_settle_ms` | text, mm, %, ms | required |
| `profile.<id>.trim.<straight\|bevel\|bevel_round>` | ° | 0; `bevel_round` the bevel's |
| `profile.<id>.<head>.edge.<left\|right>`, `.run_on.<start\|end>`, `.overlap_ms`, `.run_out_mm` | as the blade's; run-ons as measured on the profile's board | the machine's, at the profile's thickness |

`<head>` is `bevel` or `straight`; a profile id is letters, digits, `_` and `-`. Each key has
bounds that catch a typing slip or a corrupt file (a speed of 1 to 100 %, a scale of 1 to 1000
steps/mm, …). Whether a change is plausible is the fit's question (§7), not the file's.

**Rules**

- **Overrides only.** A value that's absent is the driver's default. The file stays short, and
  better defaults in a new driver still apply.
- **A profile exists while it has a name**, and must then have a thickness, speed and settle
  too. Removing a profile unsets all its values, as one revision.
- **The history is append-only.** Each change keeps the value the file held before and after
  (absent: the default), and a fitted value its standard error. The current value's
  provenance (when, which routine, its uncertainty) is the last entry that touched it. Revert to
  revision *n* writes a new revision equal to *n*, so revert can be reverted too. A typed value
  (thickness, speed) is an entry with no routine. Edits that change nothing make no revision.
- **Profile ids are the shop's.** The file owns the profile list and each profile's cutting
  values, since they depend on this machine's blades. Bevel's board list refers to a profile by
  id (decided, §11).
- `deny_unknown_fields`, an unknown key refused, and version 1 refuses other versions.
- **One file for the machine**, shared by `mc601` and Bevel (decided, §11): a calibration taken
  from either is the machine's. The path rule is the driver's, so the two can't drift:
  `calibrate::store`, the crate's only I/O. The file is `$MC601_CALIBRATION` if set, else
  `calibration.json` beside `mc601`'s `config.json`: `~/.config/mc601/` (or
  `$XDG_CONFIG_HOME/mc601/`), and `%APPDATA%\mc601\` on Windows, the shop PC. `mc601` takes
  `--calibration FILE` too.
- **Two writers, one file.** `store::save` names the revision it read. If the file has moved
  on since, the write is refused (`Error::Moved`) and the caller reads again, so neither
  overwrites the other's change. It writes a temporary file and renames it into place.
- **A missing file is the defaults**, which are the shop's 601 as fitted (decided, §11). Another
  shop's 601 starts there and runs stage 1 first (§5).

**Resolving it.** Everything else asks the file, never the defaults directly:

```rust
impl Calibration {
    /// Every value the driver's default: a machine never calibrated.
    pub fn new() -> Self;
    pub fn from_json(text: &str) -> Result<Self, Error>;
    pub fn to_json(&self) -> String;
    pub fn revision(&self) -> u32;
    pub fn history(&self) -> &[Entry];

    /// `Profile::gunnar_601()` with the calibrated head offsets and scale.
    pub fn machine(&self) -> machine::Profile;
    /// The blade model for cutting this profile's board. For each value: the profile's own,
    /// then the machine's (reference board), then the driver's model. A profile's run-ons are as
    /// measured on its board; the machine's are `run_on_mm + run_on_per_mm × thickness`.
    pub fn blade(&self, profile: &str) -> Result<compiler::Blade, Error>;
    /// The program `material` for this profile: name, speed, settle, trims.
    pub fn material(&self, profile: &str) -> Result<program::Material, Error>;
    pub fn thickness(&self, profile: &str) -> Result<f64, Error>;
    pub fn profiles(&self) -> Vec<(String, String)>;      // id, name

    /// The value in effect, and as the file holds it.
    pub fn get(&self, key: &Key) -> Result<Val, Error>;
    pub fn raw(&self, key: &Key) -> Option<&Val>;
    /// Every value, the machine's and each profile's, with its label, unit, whether the file
    /// sets it, and its last change (when, which routine, ± its standard error): the overview.
    pub fn values(&self) -> Vec<ValueState>;

    /// Edits (a value, or back to the default; a fitted one with its standard error) as one
    /// new revision, with why: when, routine, session, note.
    pub fn set(&self, edits: &[Edit], why: &Why) -> Result<Calibration, Error>;
    pub fn remove_profile(&self, id: &str, why: &Why) -> Result<Calibration, Error>;
    pub fn revert(&self, to: u32, why: &Why) -> Result<Calibration, Error>;
}

// calibrate::store
pub fn default_path() -> PathBuf;
pub fn load(path: &Path) -> Result<Calibration, Error>;          // missing: the defaults
pub fn save(path: &Path, cal: &Calibration, read: u32) -> Result<(), Error>;
```

## 5. Routines

*Redesigned 16 Nov (§11): calibration sheet 2's approach, cut raw and fixing each head's values
on their own, on 150 × 150 mm offcuts, with no slits (a slit has no edge to measure from) and
every reading checked by another. The drawings, to scale:
https://claude.ai/artifact/JATu4mFX289ZErPrNZRALL (private).*

| Routine | When | Board | What it cuts | Readings | Sets |
|---------|------|-------|--------------|----------|------|
| **Stage 1: the heads** | new machine; after a knock or head service | an offcut 150 × 150, 1.2–2 mm | a 100 mm mount with a 50 mm bevel window, its outline cut anticlockwise | 16 by calipers, none unchecked | each head's position, sideways offset, slit width; the bevel's lean |
| **Stage 2: the run-ons** | new blade | an offcut 150 × 150 of a common board | calibration sheet 2's combs: per head, an end and a start comb, each cut twice with its teeth coming from opposite sides | 8: each comb's first tooth to come free, by flexing the board | each head's start lead and end shortfall, on each face |

**A new machine:** stage 1, then stage 2. **A new blade:** stage 2. **A new board:** its
thickness, by calipers; stage 2 on it only if it misbehaves.

**Taken as given, not calibrated** (decided 16 Nov, §11): **square**, since X and Y each move
alone along their own rails, so the angle between them is fixed by how the rails are mounted
and doesn't drift; and the **scale**, GMC's 40 steps/mm, which its captures match.

- **Stage 1** (`designs` example, "stage 1"): the window clockwise with the bevel, the outline
  anticlockwise with the straight head, so the mount stays on the blade's left all round. The
  mount's size reads one side of the straight cut, the frame's opening (inside jaws, the mount
  out) the other, the borders the bevel head against the straight with the cut's off-centre
  cancelled; the drop-out face up gives the bevel's lean; each stock edge is read twice. Window
  placement ±0.08 mm against calibration sheet 2's ±0.17, for caliper readings scattering
  0.11 mm. It cuts straight strokes both ways, as the compiler then may.
- **Stage 2** (decided 16 Nov, §11): **calibration sheet 2's combs**, each a reference cut and
  seven teeth whose run-ons are **graded** around the best prediction (calibration 1 and GMC's
  values at first, the file's after): −1.2, −0.6, −0.3, 0, +0.3, +0.6 and +1.2 mm, fine where
  the answer most likely falls and coarse at the ends; an end comb's teeth run on past the
  reference, a start comb's begin past it. **Read by flexing
  the board** along the reference: a tooth whose cut has gone through to the reference, on both
  faces, comes free there (Imran, 16 Nov: the board flexes enough that this is plain). Enter the
  first tooth that does, counting from the shortest: one number per comb, no hairlines to
  judge and no ruler on a slit. Each comb is cut twice, its teeth coming to the reference from
  opposite sides, so cuts in both directions are checked (and the tooth's other side). The
  reference is cut by the same head, keeping the teeth's side on its right as a window's next
  or last edge keeps its drop-out, in production's order (after the teeth at an end, before
  them at a start). Eight combs, eight numbers, 64 cuts. With each face's corner where stage 1
  puts it, a comb's number is the head's reach: on the face that frees last exactly, the other
  as a bound. The run-on is set there plus a small chosen margin; the tool model estimates any
  overcut showing on the face. Cut once per blade on a common board; a second on a 2.5–2.7 mm
  board says whether the reach changes with thickness. **One pass:** a comb that misses (every
  tooth free, or none) says which way, and only it is cut again, shifted; centred on the file's
  values after the first calibration, a miss is rare. Steps finer than 0.3 mm aren't worth
  cutting: whether a corner comes free varies with the board's fibres and the blade's edge by
  about 0.1–0.2 mm, so finer teeth read erratically, and the margin above the threshold covers
  it. **Built after the bevel's soft start and the oval entry wait** (§9): how a cut starts
  changes its start.
- **Calibration sheet 2** stays in the golden tests, as the record.
- **Ovals: later.** GMC's overlaps and run-outs serve until a shop complaint about joins.
- **Hooks: later.** A hook is a curve at the start of a cut, more common on the bevel head.
  The bevel has no soft start at any thickness in GMC (protocol reference §3.3), the likeliest
  cause of the wobble on thick board; hooks are looked at again once the driver softens the
  bevel's start. Until then each profile's trims are typed, the shop's values carried over.

**Every plan is tested** to validate (V1–V10), to keep clear of the clamp pads on its smallest
board, and to fit the envelope with the default calibration and with plausible errors in it.

## 6. A routine as data: the plan

```rust
pub enum Routine { Heads, RunOns }   // stage 1, stage 2

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
    /// One of several, by eye: a comb's first tooth to come free, or sheets cut through.
    /// `none` allowed where it means something ("none came free").
    Pick { options: Vec<PickOption>, none: Option<String> },
}
```

The drawing and each reading's dimension are geometry from the driver, so Bevel only draws them
(Bevel's rule: the front end does no geometry). A pick's options carry their places in the
drawing too: tooth 3 is the third of its comb.

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
- **Picks become values with an uncertainty.** In a comb (§5), the first tooth to come free
  gives an interval for the run-on that frees that corner on both faces: past the tooth
  before, up to this one. With each face's corner where stage 1 puts it, that is the head's
  reach on each face within half a step: the limiting face exactly, the other as a bound. A
  comb whose first tooth already comes free, or whose last doesn't, missed: the proposal says
  so and plans one shifted next time.
- **The solve is an SVD** (`nalgebra`, built at step 3), not the normal equations. It says
  which unknowns the readings fix each on its own; unknowns seen only as a sum or a difference
  are reported tied (`Fit::tied`), never one of them fixed at the sum's value, as the normal
  equations did. Each reading is weighed by its uncertainty (a steel rule's against
  calipers'). On a sheet that fixes everything, the numbers are the normal equations'.
- **Priors.** What the calibration already knows of an unknown, its value and standard error,
  is one more reading of it (`Prior`, weighed against a caliper reading's 0.11 mm scatter,
  calibration 1's). A routine reads only what it means to check and takes the rest as known.
- **Precision before cutting.** `precision(sheet, priors)` gives each unknown's standard
  error from which readings a plan takes, before any is read: what a plan can promise, and
  what leaving a reading out costs (`cargo run -p calibrate --example readings`).
- **Suspects.** After the fit, each reading's residual in units of what it should scatter,
  judged by the other readings and allowing for how much it decides alone (externally
  studentised, `Leftover::t`). Beyond 3.5 it is named: "measure W.width again". A reading
  that is an unknown's only witness has no `t`: nothing can check it, and the plan should say
  so. (Judged by a scatter it is part of, a misread would hide: that measure can't pass √dof,
  under 3 on most sheets.)
- **What calibration sheet 2 shows** (15 Nov, from `precision`): its 11 strip and window
  readings fix 8 unknowns. The four readings from the board's edges (`S.left`, `S.front`,
  `W.left`, `W.front`) and `SR.width` are each an unknown's only witness, so a misread there
  moves a head unseen; the spare readings come in twins (`W.width` and `W.height`, `D.width`
  and `D.height`, `SL.width` and `SL.length`), so a misread is caught but not told from its
  twin. Each head offset is good to about ±0.12 mm, set by its one position reading. With the
  slit edges known from the file (±0.03 mm), the four position readings alone fix the head
  offsets to ±0.114 mm: four readings, not eleven, would serve a quick check.
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
    pub id: String,                       // given by the caller: "2026-11-15 10.31 runons"
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
| `mc601` | `calibrate` becomes a terminal front end over the crate: `mc601 calibrate <routine> [--profile C2]` (cut, read, fit, apply); `mc601 calibration show \| history \| set KEY VALUE … \| unset KEY … \| remove-profile ID \| revert N \| path` (built, step 2). `run`, `check`, `probe` and `calibrate` read the file, from `--calibration FILE` or the shared path. `calibrate fit --sheet` keeps fitting the old raw sheets from their logs, for the record and the golden tests. |

**In Bevel** (for Bevel's own spec; the driver is designed to make it possible):

- **The Tauri shell** loads the file once and passes it on. It gives `cal.machine()` to
  `Station::open`, and `cal.blade(p)` and `cal.material(p)` to Arrange's times and Cut's
  compile. That replaces the five hardcoded defaults. After an apply, it calls the runner's
  `set_profile` and marks Arrange's cached times stale.
- **The pure parts** (`routines`, `plan`, `predict`, `check`, `fit`, `apply`) can go in
  `design-wasm`, as Arrange's do. The browser build then runs the whole flow against the
  simulator. Storage and the station stay in the shell, as Cut's do.
- **A Machine page**, beside the three steps or under settings, holds:
  - the overview (`values()`: each value, where it came from, when, ±);
  - each profile's state;
  - the history with revert.
- **A routine** runs as Prepare → Cut (Cut's own Load and Run) → Measure → Result.
- **Profiles move** from `data/materials.json` into the calibration file (decided). Board codes
  keep their profile id. Thickness, speed, settle and trims are typed on the Machine page.
  Run-ons come from the tool (§3), worked out for each board's thickness.

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
2. **The file.** *Built, 15 Nov.* `Calibration` (§4): the values over the defaults by key,
   the profiles, resolution (`machine`, `blade`, `material`), `values` for the overview, `set`,
   `remove_profile` and `revert` with the history; `store` for the shared path, load and a
   save that refuses a stale write. `mc601 run`, `check`, `probe` and `calibrate` drive with the
   file's machine (each session's first line names the revision), and `mc601 calibration`
   shows it, sets and unsets values, removes a profile, lists the history and reverts. Golden,
   in `mc601`: calibration 1's fitted head offsets, set in the file, resolve to the profile its
   fit writes. Writing a fit into the file from `mc601 calibrate` is step 3's proposal.
3. **The fit.** *Built, 16 Nov:* the SVD solve, what is tied, weights, priors, `precision`,
   combinations, studentised suspects; the record's fit unchanged.
4. **Settle the tool model** (§3) before building on it: calibration 1's overrun readings
   taken again on the face, if its sheet is still there; then stage 2's combs on a common board
   and on a 2.5–2.7 mm one.
5. **The tool in the compiler and the file.** A `Tool` per head (§3) from which the compiler's
   per-head cut numbers (`HeadCut`) are worked out for a board's thickness; the calibration
   file's keys become the tool's (per-profile edges and run-ons go); the fit's unknowns named
   for them. Golden: GMC's jobs still rebuilt with GMC's numbers.
6. **The start of a cut.** The bevel's soft start by thickness (the machine crate's plan) and
   the oval entry's short wait, each switchable, tried against GMC's on thick board.
7. **Plans, sessions, proposals:** stage 1 and stage 2 (combs) as plans, with steps,
   drawing and dimensions; `predict`, `check`, `fit` over sessions, verdicts, `apply`;
   synthetic recovery tests; `mc601 calibrate` on the sim, end to end.
8. **The runner:** `set_profile`.
9. **A request to Bevel**, written there by Bevel's own process: what §9 lists.

Ovals and hooks wait (§5).

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

**Decided with Imran, 16 Nov:**

5. **Calibrate the tool, not the board** (§3): each head described physically, the board by
   its measured thickness; GMC's per-profile patches not copied.
6. **Straight strokes may run both ways,** once stage 1 has measured each side of the cut.
7. **Calibration on 150 × 150 offcuts, in stages, with no slits** (§5): calibration sheet 2's
   approach, rebuilt. Sheet 2 stays as the record.
8. **Only stage 1 and stage 2 are routines.** No check, scale or sheet count: the tip depth
   needs no value of its own (§3; the shop cuts through no more than four printer sheets),
   and square and scale are taken as given (§5).
9. **Run-ons by calibration sheet 2's combs** (§5), read by flexing the board: the first tooth
   whose cut comes free through the reference is the measurement. Each comb cut from both
   sides, so both directions are checked, and separately for the start and the end (a comb
   for both would loosen its pieces). One number per comb to enter. Teeth graded ±1.2 mm round
   the prediction, 0.3 mm apart in the middle; one pass, a comb that misses cut again.

**Still open:**

10. **Blade changes.** Should Bevel record "blade changed" (Cut's Load already asks for one), so
    the Machine page can suggest stage 2?
11. **Each head's start lead and end shortfall on the face** (§3): calibration 1's overrun
    readings taken again on the face, if its sheet is still there.
