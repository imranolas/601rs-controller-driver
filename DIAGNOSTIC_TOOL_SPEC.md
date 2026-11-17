# 601rs — Diagnostic & Calibration Tool Spec

> **Superseded for calibration** by [`CALIBRATION.md`](CALIBRATION.md) (4.4, 4.5 and 4.9 here).
> This is the first plan, written before the driver; it is kept as the record.

**Companion to:** `PROTOCOL_REFERENCE.md` (the reference says *what the machine
does*; this says *how to methodically measure the rest and keep it calibrated*).

**Purpose:** A single tool that (1) fills the remaining protocol/calibration unknowns
through repeatable routines, (2) proves the driver's transport layer in a low-stakes
context, and (3) becomes the permanent calibration/trim tool (blade wear, new
materials, re-tuning).

**Build-order role:** FIRST build, before driver or CAM. It de-risks everything after
it — the transport machinery it needs (open port, send, poll `@X`, read `@LX/@LY`) is
exactly the driver's core, validated here while only sending safe commands.

---

## 1. Language & architecture

- **Go** — same choice as the driver, so this tool's transport layer *becomes* the
  driver's transport layer. Single static binary, cross-compiles to the Windows
  machine now running GMC. `go.bug.st/serial` for the port.
- **Structure:** a `transport` package (port open, framed send/receive, `@X` poll,
  timeouts) + a `routines` package (each calibration/diagnostic procedure) + a CLI
  (menu or subcommands) + a `logger` (replayable capture, GMC-compatible format).
- The `transport` package is the shared foundation with the eventual driver — write
  it once, cleanly, here.

---

## 2. Safety defaults (non-negotiable for this tool)

A diagnostic tool pokes at unknowns — exactly when the machine misbehaves. Therefore:

- **Blade UP by default.** No routine lowers a head unless the operator explicitly
  opts in per-run (`--allow-cut`). Default routines are move/read/probe only.
- **Soft-limit every move.** Reject any commanded coordinate outside the known
  envelope *before* sending. Envelope comes from the homing reference + measured
  travel (one of the first things the tool establishes).
- **Home first, or refuse.** Motion routines require a completed home (state check)
  unless explicitly overridden for a specific low-level probe.
- **Long timeout on homing/`@X` waits** (seconds), short elsewhere; every wait has a
  max-timeout backstop that drives to safe state (blade up, motion stop) on expiry.
- **Dry-run mode** (`--simulate`): print the exact bytes each routine *would* send,
  with no port open. Use to review a routine before running it on the machine.
- **Big obvious STOP.** Any long routine is abortable from the keyboard; abort drives
  to safe state.

---

## 3. Log format (replayable, GMC-compatible)

Log every byte both directions, timestamped, in a format **diffable against the
existing GMC sniffs**. Minimum per event: ISO-8601 timestamp (ms), direction
(TX/RX), raw bytes (hex), decoded ASCII. Also log a structured per-routine result
(JSON): routine name, inputs, measured values, computed constants, pass/fail.

Rationale: the tool's output and GMC captures become interchangeable — you can diff
"what GMC does" vs "what my tool does" directly when validating the driver later.

---

## 4. Routines (the menu)

Each routine: states its purpose, what it sends, what it measures, what it outputs.
Ordered by value / dependency.

### 4.1 Raw command console
- **Purpose:** exploration; answer "what does `#OX` do" style questions.
- Type a command → see timestamped reply. Envelope/blade guards still apply to
  recognized motion commands; a `--raw-unsafe` flag bypasses for expert use.
- **Output:** live transcript to the log.

### 4.2 Identify & handshake check
- **Purpose:** confirm connection + whether the licence exchange is needed to drive.
- Send `@V` → expect version. Then attempt `@X`, a safe `@I` poll, and (opt-in) a
  tiny jog **without** performing the `:ULICENSE` exchange.
- **Measures:** does the controller respond to motion/status commands without the
  licence handshake? (Resolves the open "is licence needed" question.)
- **Output:** version string, licence-required yes/no.

### 4.3 Homing + envelope establishment
- **Purpose:** home, then learn the travel envelope for soft limits.
- Run the init/arm sequence → `$Hxy` → poll `@X` until `@X04` (long timeout).
  Confirm `@LX/@LY` at reference (≈3,2). Then **carefully** jog to each extreme
  (blade up, slow, envelope-guarded incrementally) reading `@LX/@LY` at the limits.
- **Measures:** X/Y travel range → the envelope all other routines soft-limit against.
- **Also captures:** the `@X` status during homing (confirm `@X18`→`@X04`), and —
  if a fault is *accidentally* induced — whatever code appears (feeds 4.7).
- **Output:** envelope min/max per axis; homing timing.

### 4.4 Scale calibration
- **Purpose:** pin XY units/mm precisely (remove the ±2% from cut-derived estimate).
- Command known-distance single-axis moves (e.g. 50, 100, 150 mm nominal via the
  current scale), read `@LX/@LY` before/after, compute actual units/mm per axis.
  Repeat and average; check X == Y.
- **Measures:** precise units/mm (currently ~41.5).
- **Output:** scale constant per axis + variance.

### 4.5 Head-offset calibration ★ (highest-value missing constant)
- **Purpose:** measure the side-by-side head X spacing.
- **Method A (read-based):** command the carriage to a fixed reference; select bevel
  (`$A45`)… actually both heads share one carriage position, so read-based alone
  can't separate them — use witness cuts:
- **Method B (witness, opt-in `--allow-cut`, scrap loaded):** with the bevel head,
  cut a tiny witness mark at commanded (X,Y). Raise. With the straight head, cut a
  witness mark at the *same commanded* (X,Y). Measure the physical distance between
  the two marks (or read the commanded coords GMC/your tool used for a shared
  physical target). The X delta = head spacing.
- **Measures:** the fixed X offset between bevel and straight heads (<1 mm accuracy
  required — a wrong value shifts windows vs mat edge).
- **Output:** head-offset constant (X), to be added when switching heads in CAM.

### 4.6 Actuator mapper
- **Purpose:** complete the `$A` bitmask table.
- Cycle each candidate `$A` code (opt-in for ones that lower heads), pause for the
  operator to observe/confirm which clamp/head actuates, log the mapping.
- **Measures:** confirm `42`=L clamp, `22`=R clamp, `05`=bevel, `0A`=straight, and
  decode combinations (`45`,`4A`,`62`).
- **Output:** actuator bit map.

### 4.7 `@X` status prober / fault vocabulary ★ (safety-critical)
- **Purpose:** decode `@X` codes beyond `04`(ready)/`18`(busy), and other error
  replies.
- Safely provoke edge conditions: send a move to the envelope boundary; send a
  malformed/unknown command; (opt-in) request a slightly-out-of-range move and see
  if it's rejected vs clamped. Log every distinct `@X` code and reply.
- **Measures:** the fault/status vocabulary the driver must handle.
- **Output:** table of `@X` codes and their meaning.
- ⚠️ Some fault modes (jam) can't be software-induced — see 4.8.

### 4.8 Abort/pause capture harness ★ (safety-critical, passive)
- **Purpose:** learn GMC's abort/pause command (which we do NOT yet have).
- **Passive-sniff mode:** the tool records the serial line while *GMC* runs and the
  operator hits GMC's STOP/pause mid-cut. (Same setup as the original captures.)
  The tool timestamps and flags the region around the abort for easy extraction.
- **Measures:** GMC's abort/pause/resume command bytes and the machine's response.
- **Output:** the abort command sequence → feeds the driver's e-stop path.
- Note: requires a machine session with a sacrificial board; the tool makes the
  capture systematic but can't manufacture the abort itself.

### 4.9 Overcut trim measurement ★ (the ongoing calibration tool)
- **Purpose:** confirm/fit the overcut model and re-tune per material / blade wear.
- With a known material profile + thickness (opt-in cut, scrap): cut a known-size
  reference. Measure actual cut length (via `@LX/@LY` at endpoints or physical
  measurement of the witness). Compare to nominal → applied overcut.
- Cross-check against the model: `geometric = thickness × tan(42°)`; residual should
  match the `.gcf` Front/Back Begin/End trims (fractions of mm).
- **Measures:** applied overcut per material; validates geometric+trim model; lets
  the operator adjust the stored trim methodically.
- **Output:** measured overcut vs predicted; suggested trim adjustments.
- **Re-runnable:** this is the permanent trim tool — run when blades wear or new
  stock is added.

### 4.10 Feed-rate verification
- **Purpose:** confirm `#G = 360 × Speed%` and characterize real motion speed.
- Command moves at several `#G` values; time them (segment send → `@X04`); confirm
  the linear speed law and measure actual mm/s for the streaming-budget model.
- **Output:** feed law confirmation + real throughput (informs the driver's pacing).

---

## 5. Suggested run order (methodical calibration pass)

A full "calibrate the machine" session, safe→cutting:

1. **Identify & handshake** (4.2) — confirm comms, licence question.
2. **Homing + envelope** (4.3) — establish soft limits.
3. **Scale** (4.4) — pin units/mm.
4. **Actuator mapper** (4.6) — confirm `$A` codes (blade-up parts first).
5. **`@X` prober** (4.7) — decode status/fault codes (non-cutting probes).
6. **Feed verification** (4.10) — confirm speed law.
7. *[load scrap, `--allow-cut`]*
8. **Head-offset** (4.5) — the key missing constant.
9. **Overcut trim** (4.9) — validate model, tune trims.
10. *[separate GMC session]* **Abort capture** (4.8) — passive sniff of GMC stop.

Output: a populated calibration file (envelope, scale, head offset, actuator map,
`@X` codes, overcut trims per material) + the abort command sequence. That file is
the driver's + CAM's calibration input.

---

## 6. Relationship to the driver & CAM

- The **transport package** built here is the driver's transport layer — reused, not
  rewritten.
- The **calibration file** this produces is consumed by the driver (envelope, scale,
  `@X` codes, actuator map, abort) and CAM (head offset, overcut model).
- The **log format** matches GMC captures, so driver validation = diff tool-output
  against GMC doing the same operation.
- Routines 4.5 / 4.9 remain useful **permanently** as the calibration/trim tool.

---

## 7. What this does NOT remove

- **Being at the machine for the cutting routines** (head offset, overcut) and for
  the **abort capture** (4.8) — these need physical measurement / a real GMC abort.
  The tool makes them systematic and their output reusable; it can't manufacture a
  jam or a GMC abort from software.
- **Operator judgement on safety.** Blade-up defaults and soft limits reduce risk;
  they don't eliminate the need for a careful operator during opt-in cutting routines.
