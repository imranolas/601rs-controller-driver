# Gunnar 601rs — Program Format v1 (DRAFT)

A **program** is one complete, compiled machine run for one board: everything the machine does
between "board loaded" and "board released". Designers and the order queue **produce** programs.
The runner **executes** them. A program is the only contract between the two.

```
Order ──▶ Design ──(compiler)──▶ PROGRAM ──(runner + machine profile)──▶ wire bytes ──▶ controller
                                    ▲
                     this document defines this boundary
```

Schema: [`program.schema.json`](program.schema.json). Protocol background: [`GMC_Protocol_Reference.md`](GMC_Protocol_Reference.md).

---

## 1. Design principles

1. **Geometry is final; the machine is abstract.** A program contains finished toolpaths (overcut
   applied, cut order decided, curves flattened) in **millimetres and degrees**. It contains nothing
   that depends on the specific machine: no steps, no head offsets, no ramps, no controller commands.
   Recalibrating the machine never invalidates a stored program.
2. **Unsafe states can't be expressed.** There is no "head down" instruction. A `cut` lowers the
   head, cuts one continuous stroke, and raises it again, as a single unit. A program can't leave a
   head down or travel with the blade in the material.
3. **Validate before running.** Every rule in §6 can be checked without a machine. The runner
   refuses an invalid program before opening the port.
4. **Data, not code.** Plain JSON: no loops, no variables, no expressions. Any language can read
   and write it, and a program can be diffed, stored and previewed.
5. **The runner owns everything around the program.** Homing, controller configuration, the safe
   end state and abort handling all belong to the runner. The program only says what to cut.

---

## 2. Coordinate system and units

- **Board frame.** Origin (0,0) is the **bottom-left corner of the board** as it lies registered
  on the machine. +X is to the right and +Y is away from the operator (up in a plan view).
  All coordinates are in **mm**, as JSON numbers, using at most 0.01 mm of precision.
- The runner maps board frame → machine steps using the machine profile (§8): board origin
  offset, the selected head's offset (bevel −71.5 / straight +6.2 mm in X), and the scales
  (40 steps/mm, 8.333 steps/°).
- **Angles** are in degrees. Field names carry their unit as a suffix: `_mm`, `_deg`, `_pct`, `_ms`.

> **Open:** the exact registration (which machine stop the board's bottom-left corner sits
> against) and the direction of Y in machine steps. The first hardware session will settle this
> (see diagnostic spec §4.3). The format doesn't change either way; only the profile does.

---

## 3. Document structure

```jsonc
{
  "format": "gunnar-program",
  "version": 1,
  "id": "4c1e…",                       // unique; used by the queue and run logs
  "meta": { … },                       // free-form, ignored by the runner (order id, design, author)
  "board":    { "width_mm": 200, "height_mm": 310, "thickness_mm": 1.5 },
  "material": { "name": "MSK_1.5", "speed_pct": 95, "head_settle_ms": 250,
                "blade_angle_offset_deg": { "straight": 3, "bevel": 3 } },
  "ops": [ … ]                         // executed strictly in order
}
```

| Field | Required | Meaning |
|-------|----------|---------|
| `format`, `version` | yes | Format identity. The runner rejects versions it doesn't know. |
| `id` | yes | Unique id (UUID recommended). |
| `meta` | no | Anything: `order_id`, `customer`, `design_ref`, `created_by`, `created_at`, … |
| `board` | yes | Board size and thickness. Used for validation and preview, not for motion. |
| `material` | yes | Defaults for every cut. Values come from the material library (today: the `.gcf` profiles C1–C16). |
| `material.speed_pct` | yes | Cutting speed as % of the machine maximum (1–100). |
| `material.head_settle_ms` | yes | Wait after a head goes down or up (`.gcf` `HeadUpDown`). |
| `material.blade_angle_offset_deg` | no | Per-head blade angle correction (`.gcf` `ZCorrection_Straight`), default 0. |
| `ops` | yes | The operations. |

---

## 4. Operations

Each op is an object with an `op` field. Every op may also carry a `label` (shown to staff:
"Opening 1", "Outer edge") and an `id`, which run logs use to report progress.

### 4.1 `clamp`
```json
{ "op": "clamp", "side": "left" }          // "left" | "right" | "both" | "none"
```
Sets the clamp state for the following cuts. `"none"` releases the clamps. Clamps stay as set until
the next `clamp` op. The runner always releases everything at the end.

### 4.2 `cut`
```json
{
  "op": "cut",
  "label": "Opening 1 — left edge",
  "head": "bevel",                          // "straight" | "bevel"
  "path_mm": [[48.65, 58.65], [48.65, 261.35]],
  "speed_pct": 95                           // optional override of material.speed_pct
}
```
One continuous stroke **with the blade in the material**. The runner executes it as one unit:

1. Travel with the head up, at positioning speed, to `path_mm[0]`, turning the blade to the
   first segment's direction.
2. Lower `head`, then wait `head_settle_ms`.
3. Follow `path_mm` at the cutting speed. The blade angle follows each segment's direction, plus
   `blade_angle_offset_deg[head]`.
4. Raise the head, then wait `head_settle_ms`.

- `path_mm` is a polyline of ≥ 2 points. **Overcut is already included.** The compiler extends the
  ends; the runner doesn't.
- **Direction matters.** For the bevel head, the travel direction decides which side the bevel
  faces. Convention: **the bevel faces the left of the direction of travel** (§9 Open).
- **Corners:** the blade can turn while in the material only by up to the machine's break angle
  (15°). Any sharper corner must be split into separate `cut` ops (validation rule V7). So a
  rectangle is four cuts, as GMC does it, while an oval is one cut: about 300 points when
  flattened as below (GMC sends about 1,100).
- **Curves** are pre-flattened by the compiler, by **chord tolerance**: the most a chord may
  stray from the true curve. At radius of curvature r, a chord of length L strays by about
  L²/8r, and the blade turns by about L/r at each vertex. Each chord's length is
  **L = min(√(8·r·e), r·θ, L_max)**:
  - **e = 0.02 mm** (no more than 0.05 mm): under one 0.025 mm motor step, so finer points
    can't change the cut.
  - **θ = the break angle (15°)**, so a smooth curve never breaks V7. With e = 0.02 mm this
    limit only applies below about 2.3 mm radius.
  - **L_max = 10 mm**, so nearly straight curves aren't reduced to a few long lines.

  The chords come out long where the curve is gentle and short where it's tight. GMC instead
  picks one spacing per curve from a table of radius bands, set by the curve's tightest point
  (protocol reference §3). On a constant radius that gives 1–6× the points this tolerance needs,
  and more where the radius varies. On GMC's oval it is 1,107 points where about 300 do. Sent the
  way GMC sends them, those points need nearly twice what the serial line carries; flattened by
  tolerance and sent compactly, about a third (protocol reference §5). The runner streams long
  curve cuts as a single buffered path.

### 4.3 `move`
```json
{ "op": "move", "to_mm": [0, 330] }
```
Head-up travel at positioning speed. Use it to present the board or clear the cutting area.
It's rarely needed: `cut` already travels to its start point.

### 4.4 `pause`
```json
{ "op": "pause", "message": "Flip the board face down", "resume": "start_button" }
```
Stops with the head up (clamps unchanged) and shows `message` to staff. `resume` is either
`"start_button"` (the green button on the machine; input I221) or `"operator"` (confirmed in the UI).
Use it for multi-step jobs, e.g. back-bevel cuts that need the board flipped.

### Not in v1, on purpose
`home`, `park`, `head_down`/`head_up`, controller parameters, raw commands, dwell. Each of these is
either runner-owned (§5) or would let a program express an unsafe state. Tools other than
straight and bevel (V-groove, score, pens) will come in a later version as new `head` values.

---

## 5. What the runner adds (not in the program)

| Phase | Runner behaviour |
|-------|------------------|
| Before | Check validation (§6) and soft limits against the profile's envelope. Require the machine to be homed (offer to home if not). Push controller configuration (`#C…`, I/O mapping). Require the operator to confirm that the board is loaded. |
| During | Lower the program to wire commands: head offsets, step scaling, `#R`/`#G20` from the laws, `#C1` dead time, blade-angle unwrapping (the rotary axis keeps counting past 360°, as in the captures), streaming and flow control, `@X` waits. Report progress per op `id`/`label`. |
| After (normal) | Raise the head, release the clamps (`$A02`), park, and report the run as complete. |
| Abort / error | `@B` brake, head up, release the clamps, then report *which op failed and why* in plain language. A run is never resumed silently in the middle of an op. |

---

## 6. Validation rules

| # | Rule |
|---|------|
| V1 | `format` = `"gunnar-program"` and `version` is supported. |
| V2 | All required fields are present; units are in range (`speed_pct` 1–100, `head_settle_ms` 0–5000, thickness > 0). |
| V3 | Every `path_mm` has ≥ 2 points, all finite numbers. |
| V4 | No zero-length segments (consecutive points ≥ 0.01 mm apart). |
| V5 | Cuts stay within the board, allowing for overcut: every point lies within the board rectangle grown by a margin (default 5 mm) on all sides. |
| V6 | At least one `cut`. |
| V7 | Within a `cut`, the direction change between consecutive segments is ≤ the break angle (15°, from the profile). |
| V8 | `ids`, when present, are unique within the program. |
| V9 | The `pause` `resume` value is known. |
| V10 *(runner)* | Every machine coordinate, after the profile mapping, is inside the machine's soft-limit envelope. |

V1–V9 need no machine profile, so the designer can check them while editing. V10 needs the profile.

---

## 7. Example: `g_regular_mount`

Board 200 × 310 mm, 1.5 mm, profile C10. Bevel opening 100 × 200 mm with borders
50 left, 50 right, 50 top, 60 bottom → opening from (50, 60) to (150, 260). Interior first, outer edge last.
Bevel overcut ≈ 1.5 × tan 42° ≈ 1.35 mm at each end. The `.gcf` trims are left out here for readability.

```json
{
  "format": "gunnar-program",
  "version": 1,
  "id": "example-regular-mount",
  "meta": { "source": "reconstruction of g_regular_mount capture" },
  "board": { "width_mm": 200, "height_mm": 310, "thickness_mm": 1.5 },
  "material": { "name": "MSK_1.5", "speed_pct": 95, "head_settle_ms": 250,
                "blade_angle_offset_deg": { "straight": 3, "bevel": 3 } },
  "ops": [
    { "op": "clamp", "side": "left" },

    { "op": "cut", "id": "o1-l", "label": "Opening — left",   "head": "bevel",
      "path_mm": [[50, 261.35], [50, 58.65]] },
    { "op": "cut", "id": "o1-b", "label": "Opening — bottom", "head": "bevel",
      "path_mm": [[48.65, 60], [151.35, 60]] },
    { "op": "cut", "id": "o1-r", "label": "Opening — right",  "head": "bevel",
      "path_mm": [[150, 58.65], [150, 261.35]] },
    { "op": "cut", "id": "o1-t", "label": "Opening — top",    "head": "bevel",
      "path_mm": [[151.35, 260], [48.65, 260]] },

    { "op": "cut", "id": "e-l", "label": "Outer — left",   "head": "straight",
      "path_mm": [[0, 310], [0, 0]] },
    { "op": "cut", "id": "e-b", "label": "Outer — bottom", "head": "straight",
      "path_mm": [[0, 0], [200, 0]] },
    { "op": "cut", "id": "e-r", "label": "Outer — right",  "head": "straight",
      "path_mm": [[200, 0], [200, 310]] },
    { "op": "cut", "id": "e-t", "label": "Outer — top",    "head": "straight",
      "path_mm": [[200, 310], [0, 310]] }
  ]
}
```

The opening is cut anticlockwise (left → bottom → right → top), which under the §4.2 convention puts
every bevel facing outward, away from the opening. Whether that is correct, and whether GMC goes the
same way, is to be checked against the capture (§9).

**Golden test:** compile this program with the 601 profile and diff the resulting command stream
against `g_regular_mount`. Matching it is the acceptance test for both the lowering code and the
coordinate mapping.

---

## 8. Machine profile (companion, sketch)

Separate file, owned by the machine, produced by calibration (diagnostic tool) or imported from `.gcf`:

```jsonc
{
  "machine": "gunnar-601rs",
  "serial": { "baud": 115200 },
  "scale": { "x_steps_per_mm": 40, "y_steps_per_mm": 40, "z_steps_per_deg": 8.3333 },
  "board_origin_steps": [?, ?],                         // open: registration
  "heads": { "bevel":    { "offset_mm": [-71.5, 1.9], "outputs": "0x05" },
             "straight": { "offset_mm": [6.2, 1.7],   "outputs": "0x0A" } },
  "clamps": { "left": "0x40", "right": "0x20" },        // open: side naming
  "neutral_outputs": "0x02",
  "max_speed": 36000, "pos_speed_pct": 100,
  "accel": { "position": 4000, "cut": 3600, "fast_z": 8000 },
  "break_angle_deg": 15,
  "envelope_mm": { "x": [?, ?], "y": [?, ?] },          // from the diagnostic tool
  "park_steps": [58860, 7923]
}
```

Keeping this separate is what makes programs portable across recalibrations and blade changes.

---

## 9. Open questions

| # | Question | Settled by |
|---|----------|-----------|
| Q1 | Board registration and Y direction (board frame → machine steps) | Golden test against `g_regular_mount`, then the first hardware session |
| Q2 | Bevel-side convention: left or right of travel? Does GMC cut openings clockwise? | Path direction in `g_regular_mount` / `g_multi_*` |
| Q3 | Where exactly GMC applies the Front/Back Begin/End overcut trims | Test cuts, one trim at a time |
| Q4 | Blade-angle formula: `atan2(dy,dx)` + offset? (captures show Z = 774 ≈ 92.9° on a vertical edge) | Golden test |
| Q5 | Does a run wait for the start button before cutting (GMC polls I221)? Per program or per queue item? | Product decision |
| Q6 | Does `pause` with clamps engaged work for flipping, or do flips need `clamp none` first? | Product + hardware |

---

## 10. Versioning

- `version` is an integer. Additive, optional fields don't change it. Anything that changes the
  meaning of an existing field, or adds a required field or a new `op`, bumps it.
- Runners reject unknown versions and unknown `op` values. They never skip an op they don't understand.
