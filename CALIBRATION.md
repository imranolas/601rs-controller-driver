# 601rs — Calibration (design, draft)

**Status:** a design, 15–16 Nov 2026. Steps 1 and 2 of §10 are built, and step 3's fit. On
16 Nov, after tracing GMC's cut geometry in its code (protocol reference §3.3, §7, §7.1):
the tool model (§3) and the routines rebuilt around it (§5). Decisions are in §11. It replaces the calibration parts of [`DIAGNOSTIC_TOOL_SPEC.md`](DIAGNOSTIC_TOOL_SPEC.md)
(4.4, 4.5, 4.9). That spec is older than the driver.

**Companions:** [`PROTOCOL_REFERENCE.md`](PROTOCOL_REFERENCE.md) §3.2 (the blade trim), §7 (the
overcut) and §8 (the profiles); [`PROGRAM_FORMAT.md`](PROGRAM_FORMAT.md) §3 (the material) and §8
(the machine profile).

The aim: a framer calibrates the machine and each board profile from **an app**. They pick a
routine, cut its sheet at the machine, type in what the calipers say, see what will change, and
accept it. The result is one file that every part reads: the compiler, the runner, `mc601`, and
the apps that lay out and cut sheets. The driver provides everything except the screens. An
app's screens stay generic: a new routine needs no new UI.

---

## 1. Where things stand

*16 Nov, end of day.* Built on `calibrate/m3-plans` (§10 has each step):

- **The file** (§4): the machine's calibration, one per machine, shared by `mc601` and apps,
  with its history and revert. Everything that cuts asks it: `machine()`, `blade()`,
  `material(profile)`, `thickness(profile)`.
- **The tool model** (§3) in the compiler: each head described physically, lines and run-ons
  worked out from it and the board's thickness.
- **The two routines** (§5): stage 1, the heads (16 caliper readings), and stage 2, the
  run-ons (8 combs read by flexing), each a plan (§6), a session (§8) and a proposal with
  verdicts and effects in words (§7); `mc601 calibrate heads | runons | read | fit`.
- **How a cut starts:** the bevel's soft start on thick board, and an oval's entry as GMC's,
  each switchable.
- **The runner** takes a new calibration between runs (`set_profile`).
- All of it runs end to end on the simulator, and GMC's own jobs are still rebuilt to the step.
- **Refactored, 16 Nov** (`calibrate/refactor`): a reading is described once (`calibrate::reading`),
  and the fit is in the tool model's own terms. The record (calibrations 1 and 2, cut raw before
  the routines) is retired with everything that served only it (§5), and the scale is no longer a
  key.

**Waiting on the machine** (`docs/hardware-testing.md`), in order:

1. The comparison cuts on thick board: the bevel's soft start against GMC's start, and an
   oval's entry against a full settle.
2. Stage 1 on a measured offcut: the first real scatter, and straight strokes cut both ways.
3. Stage 2 on a common board, then on a 2.5–2.7 mm board: whether the reach changes with
   thickness.

**Waiting on the apps** (step 9, §10): calibrating from an app, on the driver's sessions
and stage 2; cutting with the file's blade and machine, not the hardcoded defaults; programs
compiled again when the calibration is newer; the board profiles in the file.

**Not yet:** pooling sessions in a proposal (a report only, `mc601 calibrate fit DIR DIR`);
ovals and hooks (§5); an app recording a blade change (§11).

## 2. Principles

1. **One file per machine.** It holds the calibrated values only. Everything else is the
   driver's defaults. It is read by everything that cuts or predicts a cut.
2. **A routine is data.** Its plan is a program to cut, the readings to take, a drawing to
   show them on, and the instructions. An app renders plans. It never knows what a comb is.
3. **Calibration sheets are ordinary programs** in the board frame. They are cut through the
   normal runner with the current calibration: no raw mode, and every safety check applies. The
   fit learns where each cut went by lowering the program again with the profile it was cut
   with. Lowering is pure, so no log is needed.
4. **Pure, but for the store.** No clock in the library: apps and `mc601` pass in the time.
   Its only I/O is `calibrate::store`, the calibration file's path, load and save, kept there so
   the front ends that share the file can't disagree about it.
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
a design gives a window's edge. "On the face" always means the customer's side, never
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

*Built 16 Nov (§10, step 5):* `compiler::HeadCut` is a head's tool model (the position stays the
machine profile's head offset), and the file's keys are its values: `blade.<head>.sideways`,
`.width`, `.lean`, `.lead.<back|face>`, `.shortfall.<back|face>` (§4).

**Per board:** the thickness, **measured with calipers**, never GMC's profile value, which was
raised to lengthen the run-ons this model works out itself. Speed, head settle and trims stay
per profile; they aren't geometry. The tip depth needs no value of its own: stage 1 reads
each face's edge and stage 2 each face's reach directly, and the shop keeps the depth to no
more than four printer sheets under a board.

**What follows** (a depth `z` below the back: the back at 0, the face at `t`):

- **Across:** the slit's middle lies `sideways + z × tan(lean)` left of the line, its sides half
  the slit width either side; the bevel's run across the board is `t × tan(lean)`. **The blade
  enters the back at the same place on any board,** so the back edge stays where the line puts
  it and the face edge moves out with the board's thickness. That is GMC's rule face down (it
  never moves a line for thickness: protocol reference §7.1), and the shop bears it out: had the
  face stayed put instead, GMC's back openings on 4.6 mm board would be 7 mm smaller than on
  1.5 mm board, and its run-ons, which grow 1 mm per mm, would need no help; instead the shop
  raised its thicker profiles' thickness by 0.2–0.9 mm, roughly what a 48.7° blade needs beyond
  GMC's 45° (0.14 mm per mm of board: 0.35–0.64 mm on those boards).
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
  that frees the corner on both faces; stage 2 adds a chosen margin when it sets the face's
  values. The compiler cuts square corners only, and takes the neighbour's slit at its
  farthest from its line.

**The defaults** (`Blade::fitted`, built 16 Nov): calibration 1's values below, and on the face,
where it read nothing along the cut, whatever gives GMC's run-ons on its 1.5 mm board, until
stage 2 measures them (decided, §11). So on 1.5 mm board every cut is as before. On other boards
the lines don't move, the straight run-ons don't change, and the bevel's grow by its run,
1.14 mm per mm of board, where GMC's grow by 1: 0.43 mm more at each end on 4.6 mm board, 0.07
less on 1.0 mm.

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
   back only, and its sheet is gone: stage 2 measures them.
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
compiler's caller stamps `meta.calibration_revision`, and an app recompiles a sheet when the
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
    "blade.straight.sideways": -0.095,
    "blade.bevel.shortfall.face": 1.55,
    "profile.C2.name": "Mount board 1.4",
    "profile.C2.thickness_mm": 1.4,
    "profile.C2.speed_pct": 95.0,
    "profile.C2.head_settle_ms": 200.0,
    "profile.C2.trim.bevel": 3.0
  },
  "history": [
    { "revision": 7, "at": "2026-11-15T10:42:00Z", "routine": "runons",
      "session": "2026-11-15 10.31 runons",
      "changes": [ { "key": "blade.bevel.shortfall.face", "before": 1.7, "after": 1.55, "se": 0.15 } ] }
  ]
}
```

**Keys.** Each value is one number (or, for a profile's name, text) under a dotted key. One
value is one history line, so setting an X offset never carries its Y along, and revert is
exact. `calibrate::key::Key` is the typed form; its text is what the file, the history,
`mc601` and apps use.

| Key | Unit | Default |
|-----|------|---------|
| `machine.<head>.offset.<x\|y>` | mm | `Profile::gunnar_601()` |
| `blade.<head>.sideways` | mm left of the line, on the back | `Blade::fitted()` |
| `blade.<head>.width` | mm | `Blade::fitted()` |
| `blade.<head>.lean` | ° from vertical | `Blade::fitted()` |
| `blade.<head>.lead.<back\|face>`, `blade.<head>.shortfall.<back\|face>` | mm | `Blade::fitted()` |
| `blade.<head>.overlap_ms`, `blade.<head>.run_out_mm` | ms, mm | `Blade::fitted()` |
| `profile.<id>.name`, `.thickness_mm`, `.speed_pct`, `.head_settle_ms` | text, mm, %, ms | required |
| `profile.<id>.trim.<straight\|bevel\|bevel_round>` | ° | 0; `bevel_round` the bevel's |

`<head>` is `bevel` or `straight`; a profile id is letters, digits, `_` and `-`. Each key has
bounds that catch a typing slip or a corrupt file (a speed of 1 to 100 %, a head offset within 2 m
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
  values, since they depend on this machine's blades. An app's board list refers to a profile by
  id (decided, §11).
- `deny_unknown_fields`, an unknown key refused, and version 1 refuses other versions.
- **One file for the machine**, shared by `mc601` and apps (decided, §11): a calibration taken
  from any of them is the machine's. The path rule is the driver's, so they can't drift:
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

    /// `Profile::gunnar_601()` with the calibrated head offsets.
    pub fn machine(&self) -> machine::Profile;
    /// How each head cuts, on any board (§3): the compiler works out each line's place and
    /// each cut's run-ons from it and the board's thickness.
    pub fn blade(&self) -> compiler::Blade;
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

- **Stage 1** (`calibrate::heads`): the window clockwise with the bevel, the outline
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
  changes its start. *Built, 16 Nov* (`calibrate::runons`, `mc601 calibrate runons`): the
  proposal takes, for each head and end, the run-on of the longest first-freeing tooth over
  the two directions (the run-on that freed the corner both ways), and writes the face's lead
  or shortfall so that the compiler's run-on on this board is exactly that; the back's too,
  only where calibration 1 had it freeing later. No margin is added: the freeing tooth is up
  to a step past the threshold already. Teeth two or more apart between the directions are
  noted.
- **The record is retired** (16 Nov): calibrations 1 and 2, cut raw from carriage coordinates
  before the routines. Their values live on as the defaults (`Blade::fitted`,
  `Profile::gunnar_601`); their sheets, readings and the code that read them are in git history
  (before `calibrate/refactor`); calibration 1's two run logs are simulator regression tests
  (`testdata/sessions/calibration_1*-run.log`). Nothing is cut raw any more.
- **Ovals: later.** GMC's overlaps and run-outs serve until a shop complaint about joins.
- **Hooks: later.** A hook is a curve at the start of a cut, more common on the bevel head.
  The bevel has no soft start at any thickness in GMC (protocol reference §3.3), the likeliest
  cause of the wobble on thick board; hooks are looked at again once the driver softens the
  bevel's start. Until then each profile's trims are typed, the shop's values carried over.

**Every plan is tested** to validate (V1–V10), to keep clear of the clamp pads on its smallest
board, and to fit the envelope with the default calibration and with plausible errors in it.

## 6. A routine as data: the plan

*Stage 1 built, 16 Nov (§10, step 7): `calibrate::plan`, `calibrate::heads`.*

```rust
pub enum Routine { Heads, RunOns }  // stage 1, stage 2

pub struct Stock { pub thickness_mm: f64, pub material: Material }   // Stock::profile, ::offcut
pub fn heads::plan(cal: &Calibration, stock: &Stock) -> Result<Plan, Error>;
pub fn runons::plan(cal: &Calibration, stock: &Stock) -> Result<Plan, Error>;

pub struct Plan {
    pub routine: Routine,
    pub board: String,             // the board it needs, in words
    pub program: Program,          // board frame, cut as any program
    pub cut_with: machine::Profile,// the calibration's machine it was placed with: lowering
                                   // the program with it gives where the carriage went
    pub steps: Vec<Step>,          // the readings, grouped by how the pieces are held
    pub readings: Vec<Reading>,
}

pub struct Step {
    pub instruction: String,       // "Lift the mount out of the frame, push the window's drop-out
                                   //  out of it, and lay the mount back up, as it lay on the machine."
    pub readings: Vec<String>,     // reading ids, in order
}

// calibrate::reading: one reading, described once. The plan shows it, the session checks it,
// the fit makes it a row, stage 2 reads its comb, `mc601 calibrate read` asks for it.
pub struct Reading {
    pub id: String,                // "W.width"
    pub prompt: String,            // "The window, left to right (inside jaws)"
    pub surface: Surface,          // the side up as it is read: Back | Face
    pub kind: Kind,
    pub dimension: [[f64; 2]; 2],  // where it is taken, board frame: one edge to the other;
                                   // for a comb, its reference
}

pub enum Kind {
    /// By calipers. Expected: by the calibration the plan was made with; further than
    /// `unusual_mm` from it, ask before keeping it.
    Length { span: Span, expected_mm: Option<f64>, unusual_mm: f64 },
    /// A comb's first tooth to come free, by flexing the board: 1 to its teeth, 0 for none.
    Tooth(Comb),
}

pub enum Span {                    // what the calipers span
    FromStock { axis: Axis, edge: Edge },          // the board's edge at the stops to an edge
    Between { axis: Axis, edges: [Edge; 2] },
    Overrun { cut: String, end: End, crossing: Edge },  // calibration 1's
}
pub struct Edge { pub cut: String, pub side: Side, pub surface: Surface }  // a side of a cut
pub struct Comb { pub head: Head, pub end: End, pub reference: String, pub teeth: Vec<Tooth> }
pub struct Tooth { pub cut: String, pub run_on_mm: f64 }  // tooth 1, the shortest, first

impl Reading {
    pub fn check(&self, value: f64) -> Result<Check, Error>;  // Fine | Unusual { off_mm }
    pub fn needs(&self) -> Vec<&str>;                        // the cuts it reads
}

impl Plan {
    pub fn check(&self, id: &str, value: f64) -> Result<Check, Error>;
    pub fn sheet(&self, label, readings) -> Result<Sheet, Error>;      // lowered with cut_with
}
```

The drawing is the program's cuts and each reading's dimension, all geometry from the driver, so
a front end only draws them. A comb's teeth are named
cuts of the program, so the drawing can mark each one.

**JSON, one reading:**

```jsonc
{ "id": "W.width", "prompt": "The window, left to right (inside jaws)", "surface": "back",
  "kind": { "length": {
    "span": { "between": { "axis": "x", "edges": [
      { "cut": "W.left", "side": "left", "surface": "back" },
      { "cut": "W.right", "side": "left", "surface": "back" } ] } },
    "expected_mm": 50.905, "unusual_mm": 1.0 } },
  "dimension": [[49.5475, 62.0], [100.4525, 62.0]] }
```

## 7. The fit

*Rewritten 16 Nov in the tool model's own terms (§3), the readings' fit unchanged:* per head, in
carriage mm,

- `P`: where the carriage puts the head's line on the registered corner. A cut commanded at
  carriage `c` runs along board `c − P`.
- The slit: its middle `s` left of the line on the back, its width `w`, and `r`, how far it
  moves left per mm deeper (tan of the lean; none on the straight head). At a depth `z` below
  the back, its left side lies `e = s + z·r + w/2` left of the line, its right side
  `e = w/2 − s − z·r` right of it.

How far a cut starts early or stops short isn't fitted: stage 2's combs read it (§5).

A position reading is `c − P + σe`, where σ = ±1 is the side read along the axis. A width is
two edges' difference, so `P` cancels unless the heads differ. Each is one linear row. The rows
are solved by least squares, with each unknown's standard error and the scatter of the readings
about the model. An unknown the readings don't settle is reported as such and left alone.

**What changes from today:**

- **Carriage positions come from lowering**, not from a log: `machine::lower(program,
  profile_cut_with)`, which is pure. A session keeps the profile it was cut with. `mc601`'s tests
  hold it to what the simulator received, command for command.
- **Picks become values with an uncertainty.** In a comb (§5), the first tooth to come free
  gives an interval for the run-on that frees that corner on both faces: past the tooth
  before, up to this one. With each face's corner where stage 1 puts it, that is the head's
  reach on each face within a step: the limiting face exactly, the other as a bound. *Built:*
  the value is the freeing tooth's run-on, and its standard error the step below it ÷ √12 (as
  likely anywhere in it), so a value a tooth away has moved and one on the same tooth is
  within the noise. A comb whose first tooth already comes free, or whose last doesn't,
  missed: the proposal says so, with no standard error, and proposes the shortest tooth (it
  freed) or the middle of a comb past the longest (2.4 mm on), so the next stage 2, centred
  on the file, is shifted.
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
  what leaving a reading out costs.
- **Suspects.** After the fit, each reading's residual in units of what it should scatter,
  judged by the other readings and allowing for how much it decides alone (externally
  studentised, `Leftover::t`). Beyond 3.5, and off by at least 0.02 mm (`MISREAD_MM`, about
  what calipers resolve on a cut edge, so readings that fit almost exactly don't make a
  rounding error a misread), it is named: "measure W.width again". A reading
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
  start at: head offset 1 mm, a slit's offset or width 0.5 mm, the lean 5°, a lead or shortfall
  1.5 mm. A large change is more likely a mistake in loading or reading than a fact about the
  machine.
- **Pooling.** `fit` takes several sessions of one routine and profile, as `fit --all` does now.
  The fit is in absolute terms, so earlier sheets still count. Pooling is always on offer. It is
  never automatic, because a blade change in between makes it wrong.

*Built for stage 1, 16 Nov:*

```rust
/// What a calibration says each unknown is, and so what each reading should show: a plan's
/// expected values, and the synthetic readings the tests fit.
pub fn known(u: Unknown, cal: &Calibration) -> f64;
pub fn predict(sheet: &Sheet, cal: &Calibration) -> Result<HashMap<String, f64>, Error>;

/// Stage 1's fit, holding the bevel's width (`heads::priors`): its two sides are read on
/// different faces, so the readings see it only with its offset and lean.
pub fn heads::fit(sheets: &[Sheet], cal: &Calibration) -> Result<Fit, Error>;
/// Stage 1's fit as changes to the file: each head's offset and slit's sideways offset, the
/// straight slit's width, the bevel's lean: each unknown is a key's value, the lean its run's
/// arctangent.
pub fn heads::propose(fit: &Fit, cal: &Calibration, thickness_mm: f64) -> Proposal;

pub struct Proposal {
    pub routine: Routine,
    pub readings: usize,
    pub scatter_mm: f64,              // NaN: none to spare
    pub changes: Vec<Change>,
    pub suspects: Vec<Leftover>,      // worst first
    pub notes: Vec<String>,           // "The bevel runs 1.71 mm across the 1.5 mm board: 48.7° ± 1.3°"
}

pub struct Change {
    pub key: Key,
    pub label: String,                // "Bevel head, X offset"
    pub unit: &'static str,
    pub before: f64,
    pub after: Option<f64>,           // rounded to a micrometre, a hundredth of a degree
    pub se: Option<f64>,
    pub verdict: Verdict,             // WithinNoise | Moved | Large | Undetermined
}

impl Proposal {
    pub fn chosen(&self) -> Vec<Key>;                                  // the moved values
    /// The chosen changes as one revision, each with its standard error, the history naming
    /// the routine.
    pub fn apply(&self, cal: &Calibration, chosen: &[Key], why: &Why) -> Result<Calibration, Error>;
}
```

*Built since:* each change's effect in words (`Change::effect`, worked out on the board the
routine was cut on, the run-ons by the compiler's own `run_on`); and, found by an app, a value
that rounds to within half a step of the file's (a micrometre, a hundredth of a degree) is
within the noise whatever its standard error, so readings that fit exactly don't call the
bevel's 0.4525 moved to 0.453. Pooling sessions in a proposal is still to come.

**Tests.** For each routine, a known "true" calibration with errors goes through
`predict`, plus noise, and `fit` must recover it within its standard errors. A plan must fail
loudly with a reading it doesn't list, a pick out of range, or a board of the wrong thickness.
*Stage 1, 16 Nov:* a machine unlike the file is recovered within three standard errors, and the
proposal applied makes the file say so; a reading misread by 0.5 mm is named; every expected
reading is the design's with the calibration's edges, to the motor step; and in `mc601`, the
plan runs on the simulator to the park, what went down the line is what lowering the plan
gives, command for command, and readings read to a hundredth fit back to the machine they came from.
Writing it found a fault older than stage 1: a width read across the two heads' cuts (a
mount's border) was ordered by carriage position, where the heads are 78 mm apart, so the fit
expected a border of −24.7 mm. Edges are now ordered as they lie on the board.

## 8. Sessions: cut, read, fit, apply

*Built, 16 Nov:* `calibrate::Session`. A session is one run of a routine. It is serialisable,
so an app can keep it as a file, resume it tomorrow, and fit it again later; `mc601` keeps it in
the session folder as `routine.json`, beside the program (`stage_1.json`, `stage_2.json`), the
run's log and the fit with its proposal (`fit.txt`).

```rust
pub struct Session {                      // format "601rs-calibration-session", version 2
    pub id: String,                       // given by the caller: "2026-11-16 10.31.02 calibrate stage_1"
    pub revision: u32,                    // the calibration the plan was made with
    pub plan: Plan,                       // as cut, with the profile it was placed with
    pub cut: Cut,                         // NotYet | Done | Stopped { ops_done }
    pub readings: BTreeMap<String, f64>,  // as kept: mm, or a comb's tooth
}

impl Session {
    pub fn new(id, cal: &Calibration, plan: Plan) -> Session;           // not yet cut
    pub fn available(&self) -> Vec<&Reading>;   // whose cuts all finished (`Reading::needs`)
    pub fn check(&self, id, value) -> Result<Check, Error>;            // also: its cuts made
    pub fn record(&mut self, id, value) -> Result<Check, Error>;       // kept, unusual or not
    pub fn forget(&mut self, id);
    pub fn sheet(&self) -> Result<Sheet, Error>;                       // lowered, no log
    pub fn propose(&self, cal: &Calibration) -> Result<Proposal, Error>;  // either stage
    pub fn from_json(text) -> Result<Session, Error>;  pub fn to_json(&self) -> String;
}
```

An unusual value kept anyway isn't marked: the operator was asked, and a misread still shows
as a suspect in the fit. If the file has moved on since the plan was made, the proposal says so
(each change is against now's value). Floats read back exactly (`serde_json`'s
`float_roundtrip`, on for the workspace), so a session reloaded lowers to the very same steps.
In `mc601`, a run that stops records how far it got, and `calibrate read` asks only for the
readings whose cuts were made; `calibrate fit DIR DIR …` fits stage 1 sessions together.

**The flow, as an app shows it.**

1. **Prepare.** The plan's board need in words ("An offcut of C2, at least 140 × 105 mm, into
   the front-left stops"), and the drawing of what will be cut.
2. **Cut.** The plan's program goes to the station like any sheet: `station.run(program)`.
   Load, Run, pause, stop, the safe state: the app's own screens and the runner's rules, unchanged.
   A stop keeps the session: readings that need a cut that never finished drop out
   (`Reading::needs`). The rest can still be read and fitted.
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
| `compiler` | *Built, 16 Nov:* `HeadCut` is a head's tool model (§3); `compile` takes a `Blade` (both heads) from `Calibration::blade` and a `Profile` from `machine`, and works out lines and run-ons for the board's thickness. `Blade::fitted()` stays as the default model, `Blade::gmc()` as GMC's for the golden tests. |
| `machine` | `Profile::gunnar_601()` keeps the shop's fitted numbers as the defaults (decided, §11). Each fitted value names the record it came from, as it does now. |
| `runner` | *Built, 16 Nov:* `Runner::set_profile(profile)` between jobs (a run holds the runner, so nothing runs meanwhile), so a new revision applies without reopening the session. Homing stays valid: the profile changes the arithmetic, not where the carriage is. It re-runs `configure` if the controller settings differ; a calibration never changes them. |
| `mc601` | `calibrate` becomes a terminal front end over the crate: `mc601 calibrate <routine> [--profile C2]` (cut, read, fit, apply; `heads` built, 16 Nov); `mc601 calibration show \| history \| set KEY VALUE … \| unset KEY … \| remove-profile ID \| revert N \| path` (built, step 2). `run`, `check`, `probe` and `calibrate` read the file, from `--calibration FILE` or the shared path. `calibrate fit --sheet` keeps fitting the old raw sheets from their logs, for the record and the golden tests. |

**In an app** (the driver is designed to make it possible):

- **The app** loads the file once and passes it on. It gives `cal.machine()` to the runner's
  session, and `cal.blade()`, `cal.material(p)` and `cal.thickness(p)` to its time estimates
  and to `compile`. After an apply, it calls the runner's `set_profile` and marks its cached
  estimates stale.
- **The pure parts** (`routines`, `plan`, `predict`, `check`, `fit`, `apply`) build for
  WebAssembly, so a browser build can run the whole flow against the simulator. Storage and
  the machine stay native.
- **A machine page** holds:
  - the overview (`values()`: each value, where it came from, when, ±);
  - each profile's state;
  - the history with revert.
- **A routine** runs as Prepare → Cut (the app's own Load and Run) → Measure → Result.
- **Profiles move** from an app's own board list into the calibration file (decided). Board codes
  keep their profile id. Thickness, speed, settle and trims are typed on the machine page.
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
4. **Settle the tool model** (§3) before building on it: stage 2's combs on a common board and
   on a 2.5–2.7 mm one (calibration 1's sheet, which might have been read again on the face, is
   gone).
5. **The tool in the compiler and the file.** *Built, 16 Nov:* `compiler::HeadCut` is a head's
   tool model (§3; the name `Tool` is a contour's), from which the compiler works out each
   line's place and each cut's run-ons for a board's thickness. The file's blade keys are the
   tool's; the per-profile edges and run-ons are gone, so `Calibration::blade()` takes no
   profile. Defaults as §3. The fit keeps its linear unknowns (each side of the slit on each
   face, and the drag along the cut); its proposal maps them onto the tool's keys (step 7),
   through the combinations calibration 1's golden test already checks. Golden: GMC's jobs
   still rebuilt with GMC's numbers, now as a tool (`Blade::gmc`: the bevel at 45°, its trims
   as the reach along the cut).
6. **The start of a cut.** The bevel's soft start by thickness (the machine crate's plan) and
   the oval entry's short wait, each switchable, tried against GMC's on thick board. *Built,
   16 Nov:* `machine::SoftStart::bevel` (`#S` 4000 → 400 from 2 to 3.5 mm of board, the ramp
   unchanged) and `Head::closed_settle_ms` (the bevel moves off 100 ms after its slow
   head-down, as GMC does, not after the profile's settle), both on by default; `mc601 run
   --gmc-bevel-start` and `--oval-settle` switch them off for comparison cuts. GMC's golden
   jobs are lowered with GMC's start. The test cuts on thick board are the shop's
   (`docs/hardware-testing.md`).
7. **Plans, sessions, proposals:** stage 1 and stage 2 (combs) as plans, with steps,
   drawing and dimensions; `predict`, `check`, `fit` over sessions, verdicts, `apply`;
   synthetic recovery tests; `mc601 calibrate` on the sim, end to end. *Stage 1 built, 16 Nov*
   (§6, §7): `calibrate::heads` (plan, propose), `calibrate::plan`, `calibrate::proposal`;
   `mc601 calibrate heads --thickness MM` cuts it and offers its readings, and `mc601
   calibrate read` fits them and offers the proposal, written as one revision if the operator
   says so. `mc601 calibrate cut` and `calibrate sheet`, which cut calibration sheet 2, are
   gone, and since 16 Nov the record too (§5). *Stage 2 built, 16 Nov:*
   `calibrate::runons` (plan, propose, `simulate`), `mc601 calibrate runons`; its 64 cuts run
   on the simulator as lowering says, and the combs a machine would free give its run-ons back
   within a tooth, never short. *The `Session` type built, 16 Nov* (§8): `mc601` keeps it as
   `routine.json`; a stopped run records how far it got and only its finished readings are
   asked for; `calibrate fit` fits stage 1 sessions together (pooling, as a report). Still to
   come: each change's effect in words, and pooling in a proposal.
8. **The runner:** `set_profile`. *Built, 16 Nov:* on the simulator, the run after it sends
   exactly what the new profile lowers to, homing kept, nothing written to the controller.
9. **The apps' part:** what §9 lists. *Handed over, 16 Nov:* `set_profile`, a finely held
   value no longer called moved by rounding, and what §9 asks of an app.

Ovals and hooks wait (§5).

## 11. Decided, and still open

**Decided with Imran, 15 Nov:**

1. **One file for `mc601` and apps.** They drive the same machine from the one shop PC, so a
   calibration from any of them is the machine's (§4).
2. **The profile list is in the calibration file.** A profile's numbers depend on this
   machine's blades. An app's board list refers to a profile by id (§4, §9).
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
   approach, rebuilt. Sheet 2 was the record until 16 Nov (§5).
8. **Only stage 1 and stage 2 are routines.** No check, scale or sheet count: the tip depth
   needs no value of its own (§3; the shop cuts through no more than four printer sheets),
   and square and scale are taken as given (§5).
9. **Run-ons by calibration sheet 2's combs** (§5), read by flexing the board: the first tooth
   whose cut comes free through the reference is the measurement. Each comb cut from both
   sides, so both directions are checked, and separately for the start and the end (a comb
   for both would loosen its pieces). One number per comb to enter. Teeth graded ±1.2 mm round
   the prediction, 0.3 mm apart in the middle; one pass, a comb that misses cut again.

10. **Until stage 2, the tool model as designed** (decided 16 Nov): the bevel's run-ons grow by
    its measured run, 1.14 mm per mm of board, not GMC's 1.0; the face's reach is set to give
    GMC's run-ons on 1.5 mm board (§3).

**Still open:**

11. **Blade changes.** Should an app record "blade changed" (it asks when a board is loaded), so
    its machine page can suggest stage 2?
12. **Each head's start lead and end shortfall on the face** (§3): stage 2's combs (calibration
    1's sheet is gone).
