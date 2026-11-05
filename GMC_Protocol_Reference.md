# 601rs — Protocol & Calibration Reference

**Status:** Command semantics come from the **controller's programming manual** and from **how
GMC does things**. Numeric constants, sequences and calibration come from **serial captures** and the
**parameter file**. Where an earlier capture-only inference conflicted with spec/code, the
spec/code reading is used and the old reading is recorded in the changelog (§13).
Items marked *(unverified)* are not backed by spec, code or capture.

**Purpose:** Specification for building an independent driver + CAM pipeline. Covers the on-wire
command grammar, the cut-sequence state machine, universal constants, the feed-rate and ramp
laws, the overcut/thickness model, and per-profile calibration.

### Citation key

| Tag | Source |
|-----|--------|
| **[SPEC p.N]** | The controller's programming manual, "CNC-Control — Bahnsteuerung für 4 Schrittmotorachsen", v0.99/69, 18 Apr 2000, SIG Positec / B. Lewetz (German). A draft. The controller runs firmware 2.00/55W, so newer behaviour can differ. Not included here. |
| **[SPEC1 p.N]** | The same manual at **v1.00/00b, 20 Jul 2000**, which comes with a radius-slowdown flowchart. The newest version found. Where it differs from v0.99, **SPEC1 wins**. The driver is built to SPEC1 with no fallbacks; captures override it where they disagree (§13.1). Not included here. |
| **[GMC]** | How GMC, Gunnar's own software for the 601, does it: learned from studying GMC 4.0 and 5.0, and not reproduced here. The captures were made with GMC 4.0.502.683. |
| **[CAP name]** | Serial captures in `testdata/captures/` (`g_*.txt`, UTF-16 software port-monitor export; no line settings recorded). Decoded to W/R lines for analysis. |
| **[GCF]** | `GMCParameters.gcf` (this folder) |
| **[SYS]** | The controller parameter file for the 601 that GMC installs (`CNCCON_2.00_601.SYS`; not included) |

**Source captures:**
- `g_init`: power-on / homing / identify / licence handshake
- `g_motions`: jog on all axes + actuators (L/R/U/D, rotate, clamps, blades)
- `g_regular_mount`: single rectangular window (100×200mm), 1.5mm board
- `g_oval_mount`: single oval window, used for curve flattening
- `g_multi_C10`, `g_multi_C11`, `g_multi_C12`: twin windows at three profiles
- `g_multi_Cx_20…100`: speed sweep, 1.5mm board, 20–100%

---

## 1. Physical / transport layer

- **Port:** RS-232, null-modem cable to the controller (a DOS compact PC running "CNC-Control") [SPEC p.4–5]. COM4 in captures [GCF `LastSuccessfullyUsedComPort`].
- **Encoding:** ASCII. **Every command and reply ends with `\r` (0x0D)** [SPEC p.6].
- **Baud: 115200**, **8-N-1** [SPEC p.6].
  - **GMC sets 115200 in both v4 and v5.** `new ComPort(port, 115200, …)` appears in [GMC] and [GMC]. There is no other baud setting in either version.
  - **The captures confirm it from timing.** In [CAP g_oval_mount], the host wrote 24,253 bytes in one burst ending with `@X`, and the controller's `@X14` reply is stamped ≤ 3 s later (the timestamps have 1-second resolution). That needs at least ~6,000 bytes/s on the wire. At 9600 baud (~960 B/s) the burst alone takes ~25 s. At 115200 baud it takes ~2.1 s.
  - **Where the earlier "9600" came from:** the capture files hold only timestamped Written/Read events, with no baud value. The figure came from the monitor's own settings, not from the line. A software port monitor records the application's reads and writes regardless of baud rate, so the decoded data looks correct either way.
  - The controller's baud rate is configurable with `#C18,<code>` (5 = 9600, 8 = 57600, 9 = 115200), taking effect after a reset [SPEC p.25]. The shipped [SYS] has `CB=9` (probably = 115200 *(unverified key)*).
- **Host line settings:** no handshake, DTR and RTS asserted, read/write timeouts 128 ms [GMC].
- **Acknowledgements** [SPEC p.6–7]:
  - In modes 0, 1 and 4, every accepted command is answered with `\r`. A rejected command gets `*NNN\r` (§2.6).
  - In **mode 2 (cutting), accepted commands get no reply**; only errors are sent [SPEC p.6–7]. Observed: `L20,…` gets no `\r` [CAP g_regular_mount].
- **Async control bytes** from the controller, each switched on or off by a parameter [SPEC p.7, p.22]:

| Byte | Meaning | Enable | Observed |
|------|---------|--------|----------|
| `0x01` | motion finished ("positioned") | `#C5,1` | yes. GMC sends `#C5,1` at init. Seen after `@B` and homing [CAP g_init, g_motions] |
| `0x02` | buffer has room again, resume sending (like XON) | `#C7,1` | no (`#C7` never sent) |
| `0x03` | mode-2 buffer almost full (~200 chars left), pause sending (like XOFF) | `#C7,1` | no |
| `0x04` | homing finished | `#C6,1` | yes, after `$Hz` [CAP g_init] |
| `0x05` | keep-alive ("node guarding"): sent after `<ms>` with nothing else sent | `#C8,<ms>` | no |
| `0x06` / `0x07` | signal input 1 (`I221`, the **green start button**) went high / low | `#C35` bit 1 | GMC sends `#C35,1` [CAP g_init] and reacts to `0x07` [GMC] [SPEC1 p.7, p.28] |
| `0x08` / `0x09` | signal input 2 (`I222`) went high / low | `#C35` bit 2 | no [SPEC1 p.7, p.28] |

- **`0x02`/`0x03` contradiction in SPEC1.** The prose on p.5 says `0x02` = "stop, ~200 chars before XOFF" and `0x03` = "room again". The byte table on p.7, the v0.99 spec [SPEC p.7] and GMC (it stops sending on `0x03`) all say **`0x03` = almost full, `0x02` = room again**. The driver follows the table. Verify with `#C7,1` on hardware.
- GMC's "`0x07` = start button" is therefore **I221 low = button released** [SPEC1]. Enable `#C35,1` to get both edges.
- In the other direction, **`0x06`** is also the *host's* ACK during **mode-9 file transfer** (the licence exchange inside `@M9`) [CAP g_init; SPEC p.37]. The two never meet: the driver never enters mode 9.

---

## 2. Command grammar

The controller works in **modes**, switched with `@M<n>`. Each mode accepts its own command
family. **Master commands (`@`) work in every mode** and are handled immediately, even when the
buffer is full [SPEC p.6, p.11].

| Mode | Purpose | Command prefix | Reply |
|------|---------|----------------|-------|
| any | master control / query | `@` | reply or `\r` |
| 0 | parameters: set with `#`, query with `&` | `#`, `&` | `\r` / value |
| 1 | jog, homing, outputs (one motion at a time, each with its own ramp) | `$` | `\r` |
| 2 | **cutting**: buffered path vectors with look-ahead speed control | none (`L`, `V`, `A`, `W`, `I`, `C`, `G`) | **none** |
| 3 | batch program (`CNCCON.CMD` on the controller) | — | — |
| 4 | continuous velocity mode | `!` | `\r` |
| 9 | file transfer / firmware update | — | — |

[SPEC p.6, p.12]. A mode can only be changed while the machine is stopped, otherwise `*028` [SPEC p.12, p.37].

**Axes:** X, Y, Z (the tangential blade rotation), U. In motion commands, an **uppercase axis
letter means absolute, lowercase means relative** [SPEC p.29, p.32].

### 2.1 `@`: master commands (any mode)

| Command | Reply | Meaning |
|---------|-------|---------|
| `@V` | `@V2.00/55W 19.10.10 2807001` | Version string: version/revision, date, serial [SPEC p.14; CAP g_init]. `@V1..@V4` return the parts separately [SPEC p.14]. GMC splits it into a high and a low version [GMC]. `@V7`/`@V8` are also sent [CAP g_init] *(not in the spec)*. |
| `@M<n>` / `@M` | `\r` / `@M0n` | Set / query the mode [SPEC p.12]. |
| `@X` | `@X<r><f>` | **Status** [SPEC p.12]. `r` = 0 stopped / 1 moving. `f` = hex flags: **1** waiting (dead time or input), **2** error, **4** home position known (see note), **8** homing in progress. Values seen: `04`, `14`, `15`, `18` (§5). |
| `@Y` | `@Yhhhh` | Switch bits `sbff rrrr uuzz yyxx`: `s` stop input, `b` machine ready, `rrrr` reference switches, then one pair per axis (U Z Y X) with the **lower bit = PLUS limit**, higher = MINUS. Spec example `@Y4001` = ready, X+ limit active [SPEC1 p.13]. Limit errors map as `*033`→X+ bit 0, `*032`→X− bit 1, `*035`→Y+ bit 2, … |
| `@E` | `@E<n>` | Current error code, same numbering as `*NNN` [SPEC p.13]. |
| `@P<a>` | `@PX-23450` | Axis position in µm [SPEC p.13]. |
| `@L<a>` | `@LX44149` | Axis position **in steps** [SPEC p.13; CAP g_motions]. |
| `@B` | `\r` (then `0x01` when stopped) | **Brake**: stop all axes with a ramp. Positions are kept [SPEC p.13; CAP g_motions]. |
| `@S` | `\r` | **Emergency stop** with no ramp. Positions are lost, so the machine must be homed again [SPEC p.13]. |
| `@N[n]` | `@N<n>` / `\r` | Set / query the mode-2 command number: the buffered command **currently being processed**. Set it to 0 before a job and poll it to drive a progress or graphics display [SPEC p.14; SPEC1 p.14–15]. |
| `@R` | — | Restart the controller [SPEC p.14]. |
| `@D01` | `\r` | Force the "home position known" flag [SPEC p.14–15]. |
| `@H` | `@H86` | Buffer fill level in % [SPEC p.15]. |
| `@I<n>` | `@I<n>,b` | Read logical input `n` (0..255) [SPEC p.15]. |
| `@Q<n>[,b]` | `@Q<n>,b` / `\r` | Read / set logical output [SPEC p.15]. `Q242`–`Q249` are auxiliary signals 1–8, i.e. the `$A`/`A` bits 0–7, so `@Q` reads back what a streamed `A` did [SPEC1 p.11–12]. GMC uses `@Q241,1/0` for the vacuum valve [GMC]. |
| `@O[pct]` | `@O<pct>` / `\r` | Speed override 10–200 %, takes effect immediately on all motion in all modes [SPEC1 p.16]; in mode 2 a stream can exempt rapid moves with `C16` and re-enable it with `C8` (§2.4) [SPEC1 p.35]. The driver rejects values outside 10–200 before sending. |
| `@@` | `\r` | *(Not in spec.)* Sent at init and on mode-2 errors; probably clears the error/reset state [GMC]. |

> **Note on `@X` flag 4.** The spec contradicts itself. Its flag table says 4 = "homing needed",
> but its own example says `@X04` = "stopped, home position known" [SPEC p.12]. The captures
> settle it: flag 4 is **absent** while homing (`@X18`) and **present** in every state after
> homing (`04`, `14`, `15`) [CAP all]. So **4 = home position known**.

### 2.2 `$`: mode 1 (jog, homing, outputs)

| Command | Meaning |
|---------|---------|
| `$H<axes>` | **Home** the listed axes in that order, e.g. `$Hz`, `$Hxy` [SPEC p.29]. Replies `\r` at once; completion is the `0x04` byte (if `#C6,1`) or `@X` returning to `0x` [SPEC p.22, p.29; CAP g_init]. |
| `$L<g>,<a><steps>[,…]` | Jog in **steps** using speed-table entry `g` [SPEC p.30]. Uppercase axis = absolute, lowercase = relative. GMC jogs with `$L20,x±999999` (run until stopped) and then `@B` [CAP g_motions]. `$L10,X3` = absolute move to X = 3 steps [CAP g_init]. |
| `$E<g>,<a><µm>[,…]` | Same as `$L` but in µm [SPEC p.29–30]. |
| `$C<a>` / `$C` | **Zero the step and position counter** for one axis / all axes [SPEC p.30]. |
| `$B` / `$S` | Stop the jog with a ramp / immediately [SPEC p.30]. |
| `$A<hex>` | Set the 8 auxiliary outputs as a hex bitmask (§2.5) [SPEC p.31; GMC]. |
| `$D<n>` / `$W<ms>` | Spindle speed / wait [SPEC p.31]. |

### 2.3 `#` / `&`: mode 0 parameters (set / query)

The query form is the same letter with `&` and no value, and the reply repeats the set format:
`&C13` → `&C13,1` [SPEC p.28].

| Command | Meaning | Seen |
|---------|---------|------|
| `#G<g>,<v>` | **Speed for table entry `g`** (1..100). Units depend on `#C11` (µm/s or Hz) [SPEC p.18]. | `#G20,<feed>` per segment, `#G10,1750/3000` at homing |
| `#B<g>,<deg>` | Braking angle for table entry `g` (0..90°): direction changes larger than this force a full stop [SPEC p.18]. | `#B20,90` (`#B20,20` for fast-pen tools) [GMC] |
| `#M<v>` | Maximum speed [SPEC p.18]. | `#M36000` [CAP g_init] = `AbsMaxSpeed` [GCF] |
| `#R<ms>` | **Maximum ramp (acceleration) time in ms** (spec range 10..400 [SPEC p.18; SPEC1 p.19]; firmware 2.00 accepts `#R470` and `#R700` with `\r` [CAP g_multi_C11, g_multi_C12]). GMC computes it from an acceleration value (§3.1) [GMC]. | `#R225`, `#R250`, `#R112`, `#R700`, `#R470` |
| `#S<v>` | Start/stop speed: the speed below which no ramp is needed, also used on tight radii [SPEC p.18]. GMC sends the tool's `StartStop` [GMC]. | `#S4000`, `#S1000` |
| `#Z<a>,<n>` / `#N<a>,<n>` | Axis resolution: steps per revolution / µm per revolution [SPEC p.19]. | `#ZZ,3000`, `#NZ,10000` [GMC] |
| `#O<a>,<v>` | Offset moved after homing, before the counters are zeroed [SPEC p.19]. | `#OX,300`, `#OY,300`, `#OZ,0` |
| `#D<a>,dir,g1,g2,g3` | Homing direction and the speed-table entries to use [SPEC p.19]. | — |
| `#I<n>,port,bit,inv` / `#Q…` | Map a logical input/output to a hardware port, bit and inversion [SPEC p.19–20]. | `&I…` queries only |
| `#H<n>,<v>` / `&H<n>` | Usage counters (MF1, MF2, hours) [SPEC p.20]. | [GMC] |
| `#P<a>,<µm>` / `#L<a>,<steps>` | Set the current position [SPEC p.27]. | — |
| `#W` / `#X` | Save parameters to disk / reload them [SPEC p.27]. | — |
| `#C<n>,<v>` | Configuration registers, see below. | |

**`#C` registers:**

| Reg | Meaning [SPEC p.17, p.20–27] | Value seen | Code |
|-----|------------------------------|------------|------|
| C1 | **Mode-2 dead time (ms)**: after the first vector the controller waits this long so the look-ahead can fill before moving | `100` before straight segments, `200` before curves, `0` for positioning | [GMC] |
| C2 | Braking criteria bitmask (1 XY/Z angle, 2 speed change, 4 Z move, 8 U move) | `0` | [GMC] |
| C5 | Send `0x01` when a move finishes | `1` | |
| C6 | Send `0x04` when homing finishes | — | |
| C7 | Enable `0x02`/`0x03` flow control | — | |
| C8 | Keep-alive interval (ms) | — | |
| C11 | Parameters in steps/Hz vs µm/µm·s⁻¹ | `0` | [GMC] |
| C12 | Ramp during homing | `1` | |
| C13 | Z is a rotary axis (always turns the shorter way) | `1` | |
| C14 | Dynamic speed control bitmask (1 diagonal, 2 radius, 4 tangential, 8 buffer) | `13` = diagonal + tangential + buffer | [GMC] |
| C16 | Usage-counter enable bits | `4` | |
| C18 | Baud rate code | — | |
| C19 | Host watchdog: abort if nothing is received from the host for this many ms | — | |
| C20 / C21 | Radius slowdown: min / max radius | `400` / `12000` | [GMC] |
| C24 / C25 | Radius slowdown: lower / upper % threshold | `30` / `100` | [GMC] |
| C26 / C27 | Radius slowdown: rising / falling threshold counts | `5` / `5` | [GMC] |
| C28 | Radius slowdown steepness factor | `1` | [GMC] |
| C29 | Auto-home at controller startup | — | |
| C31–C34 | CMOS addresses for the usage counters | — | |
| C35, C37 | *(not in spec)* | `1`, `1` | |
| C52 | *(not in spec)*: **rejected with `*021`** ("invalid parameter id") by this firmware | `800` | [CAP g_init] |
| C97 | *(not in spec)*: **controller real-time clock** `day,month,year,hour,min,sec`. `&C97` reads it, `#C97,…` sets it | reply `&C97,165,165,2065,165,22,39` (date fields look unset, 0xA5) | [GMC] |

### 2.4 Mode 2: path commands (no reply)

| Command | Meaning |
|---------|---------|
| `L<g>,X<steps>,Y<steps>,Z<steps>` | **Linear vector in steps**, using speed-table entry `g` [SPEC p.31–32]. GMC always uses `g = 20`, so **`L20` means "vector at speed entry 20"**, and the speed itself is set by `#G20,<v>` [GMC]. Uppercase = absolute. |
| `X…,Y…,Z…` | Follow-on vector reusing the previous `g` [SPEC p.32]. GMC repeats the full `L20,` every time [CAP g_oval_mount]. |
| `V<g>,…` | Same as `L` but in µm [SPEC p.32]. |
| `G<g>,<v>` | Change a table speed while in mode 2 [SPEC p.32]. |
| `A<hex>` | Set the auxiliary outputs (same bitmask as `$A`) [SPEC p.33]. |
| `W<ms>` | Dwell [SPEC p.33]. |
| `I<±n>` | Wait until input `n` is high (+) or low (−) [SPEC p.33]. |
| `C<bits>` | Special functions [SPEC1 p.35]: **1** = run the next vector without the `#C1` dead time (one-shot; "for single moves with no path after them"); **8** = from here the speed override applies (tool in the material); **16** = from here no override (tool not engaged, rapid). **2** (diagonal-first) and **4** (one axis at a time) are listed but "noch nicht realisiert" (not yet realised). `C0` cancels a pending one-shot before the move. An invalid value is `*067`. The driver encodes only 1, 8 and 16 (or 0). |

Because `A`, `W` and `I` sit **in the buffer**, a whole job (outputs, dwells, waits for the operator, vectors)
can be streamed once and runs in order without host round-trips; `@N` reports progress. Each executed item
(vector, `A`, `W`, `I`) advances `@N`; `C` is a modifier and does not *(sim assumption, verify on hardware)*.
While an `I` wait or a `W` dwell holds the stream, `@X` reads `15` (running, waiting), the same as during the dead time.

### 2.5 `$A` / `A` output bitmask

Bit `i` of the hex value is output `i`. The bit roles come from the machine parameters
[GCF `i_*`; GMC]:

| Bit | Hex | `.gcf` name | Role |
|-----|-----|-------------|------|
| 0 | 01 | `i_t` | bevel tool *(name only)* |
| 1 | 02 | `i_bh` | head-up / hold (set at init and on head-up) [GMC] |
| 2 | 04 | `i_b` | blade / bevel down [GMC] |
| 3 | 08 | `i_sh` | straight head |
| 4 | 10 | `i_dd` | head switch pulse, 250 ms [GMC] |
| 5 | 20 | `i_pl` | clamp ("p… left") |
| 6 | 40 | `i_pr` | clamp ("p… right") |
| 7 | 80 | `i_free` | spare |

Observed codes [CAP g_motions, mounts]: `02` neutral (only `i_bh`) · `05` bevel head down (`i_t` + `i_b`)
· `0A` straight head down (`i_bh` + `i_sh`) · `22` (`i_bh` + `i_pl`) · `42` (`i_bh` + `i_pr`) ·
`45` = `40` + `05` · `4A` = `40` + `0A` · `62` = both clamps.

> ⚠️ **Clamp side naming conflict.** The capture notes label `42` as the *left* clamp and `22`
> as the *right*, but the `.gcf` names put bit 6 (`40`) on `i_pr` (right) and bit 5 (`20`) on
> `i_pl` (left). Either the `p` in the names doesn't mean the side, or the wiring is mirrored.
> **Confirm physically before relying on it.**

### 2.6 Inputs and errors

Logical inputs [SPEC p.10]: `I235`/`I236`/`I237` = **home (reference) switches for axes 1/2/3 (X/Y/Z)**;
I239–I246 = limit switches; I247 = machine ready; I254/I255 = stop/start.
[SYS] maps them on the 601: `I235=001h,6,1`, `I236=001h,7,0`, `I237=001h,4,0`, start button `I221=279h,7,1`.
GMC polls `@I221` for the **start button** (`,1` then `,0`) [GMC]. With `#C35,1` the
controller instead *pushes* `0x06`/`0x07` on each edge [SPEC1 p.7, p.28], so no polling is needed.
`&I240` is an **input-definition query**. Its reply `&I240,000h,0,0` means input 240 (axis-2 limit) is not mapped [SPEC p.28].

Errors are `*NNN\r` [SPEC p.37–38]. The ones that matter most:
`*001` command not valid in this mode · `*003` bad format · `*021` invalid parameter id · `*028` mode change only while stopped ·
`*030` already moving · `*032–039` limit switch hit · `*040` stop input · `*041` not homed · `*056` machine not ready ·
`*062` sitting on a limit switch · `*063` can't home, sitting on the switch · `*066` bad input number in a mode-2 `I` · `*067` invalid value for a mode-2 `C` ·
`*068` host watchdog timeout · `*007`/`*008` service code needed / invalid: once the start counter reaches 0, mode switches are refused [SPEC1 p.10, p.38].
The driver prints the SPEC1 text for every code it knows (`protocol::errors::describe`).
`*069` is not in the spec; GMC treats it as a controller error [GMC].

### 2.7 Licence (mode 9, do not reproduce)

`@M9` → `:ULICENSE.LIC` → hex blocks, each followed by a `0x06` ACK → `@M0` [CAP g_init]. This is mode-9 file transfer:
GMC reads the licence file stored on the controller [GMC].
It is **not needed for motion**. A clean-room driver should not send `@M9` at all, and must also never trigger the GMC
firmware-update path (an `.rtb` upload when the `@V` version doesn't match) [GMC].

---

## 3. Universal constants

| Constant | Value | Source |
|----------|-------|--------|
| **XY scale** | **40 steps/mm** (0.025 mm/step) | [GCF] `MachineScale.ScaleX/Y = 400 /cm`; GMC converts with it [GMC]. Matches captures. |
| **Z (blade angle) scale** | **≈ 8.333 steps/deg** (750/90) | [GCF] `ScaleZ = 8.3333`. Z is a rotary axis (`#C13,1`). |
| **Feed-rate law** | **`#G20 = AbsMaxSpeed × Speed% / 100` = `360 × Speed%`** | [GMC] with `AbsMaxSpeed = 36000` [GCF]. Exact match to the speed sweep [CAP g_multi_Cx_*]. GMC's speed reduction factor is 1 here. |
| **Rapid / positioning feed** | `#G20,36000` (`PosSpeed = 100 %`) | [GCF] `PosSpeed`; [GMC] |
| **Ramp law** | see §3.1 | |
| **Bevel blade inclination** | **42°** | [GCF] `Heads[Bevelled].Inclination` |
| **Curve chord tolerance** | ≈ 0.5 mm (≈ 24 steps per segment) | [CAP g_oval_mount] segment spacing |
| **BreakAngle** | 15° | [GCF] |

### 3.1 Ramp law (`#R`), confirmed

```
v_ref   = (AbsMaxSpeed / ScaleY) × 10        = (36000 / 400) × 10 = 900
#R (ms) = min(700, v_ref / (accel / 1000))   = min(700, 900000 / accel)
```
[GMC]

| `.gcf` acceleration | Value | `#R` | Used for |
|---------------------|-------|------|----------|
| `_PosRampe` | 4000 | **225** | positioning moves |
| `_RampeCut` | 3600 | **250** | cutting (most tools) |
| `_FastZRampe` | 8000 | **112** | short moves with a large blade rotation [GMC] |
| `_RampeCut` | 420 | **700** (clamped) | tools with a slow ramp |
| (dynamic) | — | 470 | thickness-dependent ramp between `DynamicMatSetting.MatLow/High` [GMC] |

These values account for every `#R` value in the captures. **`#R` is not a blade angle**; the blade
angle travels in the Z field of each `L20` vector.

---

## 4. Cut-sequence state machine

The order is **identical across all profiles** [CAP g_multi_*]. Annotations follow [GMC].

```
1. INIT
   @V → (licence, mode 9: skip) → @M0 → config push (#C…, #I/&I…) → homing (§4.1)
2. JOB SETUP (mode 0)
   $A42                    clamp (mode 1)
   #C20 #C21 #C24–#C28     radius-slowdown parameters for the active tool
   #S<StartStop>           start/stop speed
   #B20,90                 braking angle for speed entry 20
3. FOR EACH FEATURE (interiors first, boundary last):
     -- positioning (blade up) --
     @M0, #C1,0            no dead time
     #R225                 positioning ramp (_PosRampe)
     #G20,36000            positioning speed (100 %)
     @M2, L20,X,Y,Z        move to the start point; Z = blade angle
     poll @X until 0x      (§5)
     -- head down --
     $A45 / $A4A           bevel / straight head down (+ clamp bit)
     -- cut --
     @M0, #C1,100|200      dead time: 100 ms for a straight edge, 200 ms for a curve
     #R<cut ramp>          e.g. 250
     #G20,<360×Speed%>     cutting speed
     @M2, L20,… [, L20,…]  one vector (edge) or the whole curve streamed in one go (§5)
     poll @X until 0x
     -- head up --
     $A42
4. TEARDOWN
   $A42                    head up, clamp held
   #C1,0, #R225, #G20,36000
   L20,X58860,Y7923,Z0     park; blade angle back to 0
   $A02                    release everything (neutral)
   @I221 …                 idle: poll the start button
```

**Safe state / abort:** `@B` (brake, positions kept) → `$A02`/head up → park. Use `@S` only in an
emergency, because it loses the position and the machine must be homed again [SPEC p.13].

**Feature-ordering rule (verified from captures):** all interior windows are cut, each as a
complete closed loop, **before** the outer boundary. The boundary is what releases the piece, so it
must be last [CAP g_multi_*].

### 4.1 Homing sequence (from the `g_init` timeline, 66 s total)

| Time | Phase |
|------|-------|
| t+0–7 s | `@V`, licence (mode 9), `&C97` clock read, `@V7`/`@V8` |
| t+8–9 s | Mode-0 config: `$CX/$CY/$CZ` (**zero the counters**), `#C…`, `#ZZ,3000`/`#NZ,10000` (Z resolution), `#M36000`, `#S4000` |
| t+~9 s | **Z home:** `#G10,1750` (`ZInitSpeed` [GCF]), `#OZ,0`, `@M1`, `$Hz` → `\r`, `@X18`, then bytes `0x01 0x01 0x04` (homing finished) [CAP g_init; GMC] |
| t+10 s | **XY home:** `#G10,3000`, `@@`, `#R,100`, `#OX,300`, `#OY,300` (offsets moved after homing), `@M1`, **`$Hxy`** |
| t+10–49 s | Physical homing. `@X` returns `@X18` (moving + homing in progress, home not yet known) for about 38 s |
| t+49 s | `@X04`: stopped and homed |
| t+49–54 s | Post-home positioning (`$L10,X3`, `L20` + `#R`) |
| t+57 s → | Idle (`@I221` polling) |

**Driver:** `$H<axes>` → wait for the `0x04` byte **or** poll `@X` until the first digit is `0` and flag 4 is set → read `@LX`/`@LY` to confirm.
Homing uses the reference switches I235–I237 [SPEC p.10; SYS], which are real inputs even though no switch has been spotted on the machine yet.

---

## 4.2 Head selection and the side-by-side X offset

Two heads (bevel + straight) sit side by side on the carriage and are selected with the output
bitmask (§2.5): `05`/`45` bevel down, `0A`/`4A` straight down.

**Fixed X offset between heads:** [GCF] `Heads[*].ToolHeadCorrection.StartPX/StartPY` (cm):

| Head | `StartPX` | `StartPY` |
|------|-----------|-----------|
| Bevelled | **−7.15 cm** | 0.19 cm |
| Straight | **+0.62 cm** | 0.17 cm |

→ **X head spacing = 7.77 cm ≈ 77.7 mm**, with Y ≈ shared. GMC subtracts `StartPX/StartPY/StartPZ`
from every target before converting to steps [GMC].
Cross-check: bevel window X-centre 7699 − straight border X-centre 4642 = 3057 steps = **76.4 mm** [CAP g_regular_mount]. Confirmed within overcut noise.

---

## 5. Pacing: the `@X` loop and buffering

The controller is **not** a one-segment executor. Mode 2 has an input buffer with look-ahead
speed control [SPEC p.6, p.8, p.31]. GMC (serial path) uses it in two ways
[GMC]:

| Path | What GMC sends | Captured |
|------|----------------|----------|
| straight edge / positioning | one `L20,…`, then polls `@X` | [CAP g_regular_mount] |
| curve (oval) | **all vectors back-to-back, then one `@X` poll loop**. The oval was 1,107 `L20` lines in a single write with no polls in between | [CAP g_oval_mount] |

**`@X` sequence after a vector** [CAP g_regular_mount]:
`@X15` (moving, homed, **waiting out the `#C1` dead time**) → `@X14` (moving) → `@X04` (stopped).

```
send L20,…            (one, or a whole contour)
loop:
    send "@X\r"; read reply
    if reply[2] == '0': break          # stopped → next step (GMC tests "@X0")
    if flag & 2: error → read @E, handle
    # 15/14/18 = still running
```
[GMC]

**Flow control:** GMC never enables `#C7` and relies on contours fitting in the buffer. A new
driver streaming long contours should **send `#C7,1` and honour `0x03`/`0x02`**, or check `@H` (fill %)
before sending more [SPEC p.5, p.7, p.15, p.22]. The buffer size is unknown *(unverified)*.
At 115200 baud a 1,107-vector oval (~24 KB) takes about 2 s to transmit [CAP g_oval_mount].

**Host watchdog:** `#C19,<ms>` makes the controller stop if the host goes quiet. That gives
"connection lost → machine stops" safety for free [SPEC p.25].

---

## 5.1 What the spec recommends, and what the driver does

Every piece of advice in SPEC1 ("sollte", "muß", "Empfehlung", …) and where the driver stands.
"Machine layer" means policy that belongs above the controller (not written yet).

| SPEC1 says | Page | Driver |
|------------|------|--------|
| Use a FIFO UART on the PC for high baud rates | 5 | USB-serial adapters buffer; nothing to do |
| Text protocol uses the `0x02`/`0x03` handshake, not RTS/CTS (that is for a future binary protocol) | 5 | Follows: no hardware flow control; `0x03`/`0x02` honoured when `#C7,1` |
| Master commands are always processed, even when the stream is held | 5, 12 | Relied on: `@H`, `@X`, `@B` sent during a hold |
| Mode 0/1: one command, wait for its `\r` before the next | 6, 32 | Follows: one outstanding command, every wait has a deadline |
| Mode 2: always send several commands ahead, or the look-ahead cannot work | 6–7, 22 | Controller streams without waiting; machine layer must stream whole contours, not vector-then-poll as GMC does |
| `#C1` dead time exists so several vectors arrive before motion starts; `C1` skips it "for single moves with no path after them" | 22, 35 | Controller offers `flags(no_dead_time)`; machine layer should use `C1` for positioning moves instead of GMC's `@M0 #C1,0 @M2` round trip |
| Buffer under 10 % for long → jerky motion; `#C14` bit 8 slows down to compensate (on in the 601's config, `#C14=13`) | 8, 25 | Keep the buffer fed: stream ahead; `@H` pacing bounds it from above |
| Use `@H` to decide whether to send more vectors or wait | 16 | Supported (`Config::stream_window_bytes`), **off by default** until checked on hardware |
| `#C19`: the host must send a character within the time while axes move | 26 | **Fixed:** the session sends `@H` while the stream is held (test `flow_pause_keeps_the_host_watchdog_fed`). Choose `#C19` ≥ 4 × `poll_interval` |
| `#C8` `0x05` can be used as a liveness signal; it only comes in send pauses | 23 | Decoded and published as an event; no host-side "controller silent" alarm yet (machine layer) |
| `@B`: wait for the end of motion (status or `0x01`) before reading the position | 14 | Caller must `wait_idle` after `brake()`; machine layer should do this always |
| `@S` / `$S` / `!S` only in an emergency (positions are lost, re-home) | 14, 32, 38 | `StopHandle::emergency_stop` exists for the e-stop path only; normal stop = `@B` |
| In mode 2 the run flag is set from the first path data, even while waiting; use the wait flag | 13 | Follows: `@X15` read as running + waiting; `wait_idle` uses the run digit |
| `#C4,2` (no screen output) if no monitor is attached: display costs controller time | 22–23 | Not sent. The 601's box has no monitor. Candidate for the session setup (machine layer); check the current value with `&C4` first |
| `#C23` pulse stretch as low as possible | 26 | Controller config, not touched (never send `#W`) |
| Baud change needs a reset (`@R`) | 15, 26 | Never done (`#C18`, `@R` not sent) |
| Mode 9 is for external programs only | 38 | Never entered |

## 6. Feed rate (solved)

**`#G20,<v>` sets the speed of table entry 20**, which every `L20` vector uses. GMC computes it as
`AbsMaxSpeed × Speed% / 100` [GMC] = **`360 × Speed%`**:

| Speed % | `#G20` | verified |
|---------|--------|----------|
| 20 / 40 / 60 / 80 / 100 | 7200 / 14400 / 21600 / 28800 / 36000 | ✓ sweep |
| 95 (C10) / 35 (C11) / 10 (C12) | 34200 / 12600 / 3600 | ✓ profile captures |

`Speed%` comes straight from the `.gcf` profile. `#S` is the **start/stop speed**, not the feed [SPEC p.18].
`#G10` is a different table entry, used for homing speed. There is **no depth axis**: blade
protrusion is set by hand on the holder.

---

## 7. Overcut / thickness compensation (in the toolpath)

*(Capture- and `.gcf`-derived, unchanged.)*

Thickness compensation shows up as **toolpath extension (overcut)** in X/Y, not as any `#` parameter.
A bevel blade at 42° has its bottom edge lagging its top edge, so thicker board needs the cut to run
further past each corner.

| Profile | Thickness | Y start → end | Cut length (steps) |
|---------|-----------|---------------|--------------------|
| C10 | 1.5 mm | 10776 → 2616 | 8160 |
| C11 | 2.7 mm | 10824 → 2568 | 8256 |
| C12 | 4.1 mm | 10900 → 2492 | 8408 |

The cut grows symmetrically at both ends. The blade angle Z also shifts slightly (774 → 783). This
matches `Heads[Bevelled].MatThicknessDependent = true` [GCF].

**Model:** `overcut ≈ thickness × tan(42°)` (geometric baseline) **+** per-head sub-mm trims [GCF]:

| Head | FrontOvercut (Begin/End) mm | BackOvercut (Begin/End) mm | trash_overcut | NC |
|------|------------------------------|-----------------------------|---------------|-----|
| Bevelled | −0.18 / 0.082 | −0.12 / 0.22 | 0.2 | −0.02 |
| Straight | −0.05 / 0.23 | 0 / 0 | 0 | −0.05 |

⚠️ Open: exactly how Begin/End × Front/Back map onto each cut. Test cuts, one trim at a time, would settle it.

---

## 8. Per-profile calibration table

*(From [GCF], unchanged.)* Wire values are derived: feed §6, ramp §3.1, overcut §7.

| Profile | Label | Thickness* | Speed% | ZCorr_Str | ZCorr_Rnd | HeadUpDown | OCAddon |
|---------|-------|-----------|--------|-----------|-----------|------------|---------|
| C1 | Barrier | 0.11 | 95 | 4 | 0 | 200 | 0 |
| C2 | Std_Front | 0.14 | 95 | 1 | 0 | 200 | 0 |
| C3 | Std_Back | 0.14 | 95 | 3 | 1 | 200 | 0 |
| C4 | Arqadi_Back | 0.16 | 55 | 7 | 0 | 200 | 0 |
| C5 | Arqadia_Xtra_Thick | 0.24 | 45 | 4 | 0 | 200 | 0 |
| C6 | BB_Ultra_thick | 0.25 | 45 | 3 | 0 | 200 | 0 |
| C7 | Aqua_Bak | 0.27 | 60 | 6 | 0 | 200 | 0 |
| C8 | Brittania | 0.35 | 20 | 6 | 0 | 300 | 0 |
| C9 | V-Groove | 0.07 | 95 | 0 | 0 | 200 | 0 |
| **C10** | **MSK_1.5** | **0.15** | **95** | 3 | 0 | 250 | 0 |
| **C11** | **MSK_2.5** | **0.27** | **35** | 4 | 0 | 200 | 0 |
| **C12** | **MSK_4.1** | **0.46** | **10** | 4 | 0 | 400 | 0 |
| C13 | D33** | 0.22 | 45 | 5 | 0 | 200 | 0 |
| C14 | Britania_3mm | 0.30 | 45 | 4 | 0 | 300 | 0 |
| C15 | Crescent_Rag | 0.15 | 70 | 3 | 0 | 200 | 0 |
| C16 | 4.6 | 0.50 | 8 | 4 | 0 | 500 | 0.05 |

*Thickness in cm (0.15 → 1.5 mm). **Bold** = captured and verified. `HeadUpDown` = head settle delay in ms
(GMC sleeps `HDSleep`/`HUSleep` after switching the head outputs [GMC]).

**Tool priority:** Straight=1, Bevel=2, V-Groove=3, Score/Deboss=4, Pens=5 [GCF `Heads[*].Priority`].

---

## 9. Licensing

See §2.7. Do not reproduce. It isn't needed to move the machine.

---

## 10. Open items

| Item | Status | How to close |
|------|--------|--------------|
| ~~Head-spacing X offset~~ | **Resolved: 77.7 mm** [GCF] | — |
| ~~`@X` codes~~ | **Resolved:** `r` + flags (1 wait, 2 error, 4 homed, 8 homing) [SPEC p.12 + CAP] | Capture an error state to see flag 2 |
| ~~`#R` meaning~~ | **Resolved: ramp ms** (§3.1) | — |
| ~~`#C1` meaning~~ | **Resolved: mode-2 dead time** | — |
| ~~`$CX/$CY/$CZ`, `#ZZ/#NZ`, `#OX/#OY`, `#C20–28`~~ | **Resolved** (§2.3) | — |
| **Abort mid-cut** | Spec gives `@B` (ramped) / `@S` (immediate) [SPEC p.13]; GMC's own stop path not captured | Capture a GMC stop mid-cut |
| **Fault responses** | `*NNN` table known [SPEC p.37–38]; not yet seen live | Provoke a safe fault (e.g. `$L` into a limit at low speed) |
| Clamp side naming (`20` vs `40`) | Conflict between capture labels and `.gcf` names | Physical check |
| ~~`#C35`~~ | **Resolved:** signal-input acks `0x06–0x09` [SPEC1 p.28] | — |
| `@@`, `@V7/@V8`, `#C37/#C52`, `*069` | Not in either spec | Not needed to drive the machine |
| Buffer size / flow control | `#C7` never enabled by GMC | Test `#C7,1` + `@H` on hardware |
| **v1.00 features on 2.00 firmware** | Built to SPEC1 without fallbacks; modelled in the simulator only: in-stream `A`/`W`/`I`/`C`, `@N` progress, `@Q242–249` readback, `@Y` limit bits, `#C6`/`#C8`/`#C35`, `@O` with `C8`/`C16` | First hardware session: run each once (checklist), compare with the sim |
| `@N`: does `C` count? | Sim assumes not | Stream `C1` + `L`, read `@N` |
| Outputs on mode switch | SPEC says reset; captures show they persist | Sim follows captures; confirm with `@Q242` across `@M` |
| `@LX`/`@LY`: commanded vs actual position | Open-loop steppers → commanded | — |
| ~~Baud / parity / stop bits~~ | **Resolved: 115200 8-N-1** [SPEC p.6; GMC; CAP g_oval_mount timing] | Confirm on first connect: if `@V` gets no reply at 115200, try 9600 |

---

## 11. Driver implementation notes

- **Language:** compiled, tight timing (Go recommended; goroutine + channel fits the read loop / feed loop / API split).
- **Architecture:** a standalone driver process with a local HTTP+WebSocket API, single-lease control, bound to localhost.
- **Reader:** a single reader that splits incoming bytes into `\r`-terminated replies **plus** the out-of-band bytes `0x01–0x09`, which can arrive inside or between replies (e.g. `\r\x01`, `@X18\r\x01\x01\x04`) [CAP g_init, g_motions; SPEC1 p.7].
- **Faults racing a poll:** a `*033` raised by motion can arrive while an `@X` (or other pure query) is outstanding, *before* its reply. Pure queries are never rejected, so a `*NNN` there is an async fault: record it and keep waiting for the real reply. Treating it as the reply puts every later reply out of step. Found by the simulator; handled in `controller::session` (`Command::is_query`).
- **Core loop:** mode 0 setup → mode 2 → stream vectors (honour `0x03`/`0x02` or `@H`) → poll `@X` until `0x`. In mode 2, don't wait for a `\r` after each vector, because none is sent.
- **Safety:** soft limits on every `L20`; `#C19` host watchdog; `@B` for stop; safe state = head up (`$A02`), motion stopped. Never send `@M9`.
- **Simulator mode:** validate jobs, model timing, emit the intended bytes with no machine attached.

---

## 12. CAM pipeline summary

```
SVG/DXF in  →  interpret layers (bevel/straight/score per cut-type convention)
            →  offset for bevel + kerf; compute overcut per §7
            →  nest (PackingSolver, rectangle guillotine — validated winner)
            →  sequence (topo-sort: interiors before boundary; tool-batch by priority)
            →  flatten curves to ~0.5mm chords; compute per-segment blade tangent (Z)
            →  emit (#C1, #R, #G20, $A, L20 …) with #G20 = 360 × Speed%, #R per §3.1
            →  driver streams per §5
```

The machine controls **XY position + blade rotation** only. Depth is set in hardware.

---

## 13. Changelog: corrections from spec/code (2026-11-05)

| Was (capture inference) | Now | Source |
|-------------------------|-----|--------|
| `#R<angle>` rotates the blade | `#R<ms>` = max ramp time; GMC computes `min(700, 900000/accel)` | SPEC p.18; GMC |
| `L20` = a cut-move command; `#` family holds cut moves | `L<g>` = mode-2 vector in steps at speed entry `g` (GMC uses 20); it has no prefix | SPEC p.31–32; GMC |
| `#G20` = global feed rate | speed of table entry 20 (`#G<g>,v`) | SPEC p.18 |
| `#C1,100/0` = blade engage/disengage | mode-2 dead time (ms) before the path starts; 100 for edges, 200 for curves, 0 for positioning | SPEC p.20; GMC |
| `@B` = "begin" | brake (stop with ramp); the controller then sends `0x01` | SPEC p.13; CAP g_motions |
| `$CX/$CY/$CZ` = axis config | zero the counters | SPEC p.30 |
| `$L10/$L20` = "modes" | mode-1 jog in steps at speed entry 10 / 20 | SPEC p.30 |
| `#S` = global max/approach speed | start/stop speed | SPEC p.18; GMC |
| `#B20,90` = bevel parameter | braking angle for speed entry 20 | SPEC p.18 |
| `@X` codes `04` ready / `18` busy only | `r`+flags; `14`/`15` dominate during cuts (9,719 / 344 replies) | SPEC p.12; CAP |
| Wait for exactly `@X04` | wait for a first digit of `0` (GMC tests `@X0`) | GMC |
| Controller has no buffer; each oval segment is polled | Mode 2 is buffered with look-ahead; the oval was streamed as 1,107 vectors with one poll at the end | SPEC p.6, p.8; CAP g_oval_mount; GMC |
| `@I235/236` = unknown | home switches X/Y | SPEC p.10; SYS |
| `&I240` = status flag word | input-definition query (input 240 not mapped) | SPEC p.28 |
| `&C97` = geometry readback | controller real-time clock (d,m,y,h,m,s) | GMC |
| `06` ACK in a general handshake | ACK inside mode-9 file transfer | CAP g_init (`@M9`) |
| Baud 9600 | 115200: set in GMC v4 and v5 code; 9600 is ruled out by the oval-burst timing | GMC; CAP g_oval_mount |
| Flow control = `@X` poll only | `0x02`/`0x03` bytes (`#C7`) and `@H` also exist | SPEC p.5, p.7, p.15 |
| `0x07` = start button "pressed" (host code) | `0x06`/`0x07` = I221 high/low, `0x08`/`0x09` = I222, enabled by `#C35` | SPEC1 p.7, p.28 |
| `C` flags 1/2/4 | 1 no dead time, 8 override on, 16 override off; 2/4 "not yet realised"; `C0` cancels | SPEC1 p.35 |
| `@Y` pair order unspecified | lower bit of each pair = PLUS (`@Y4001` = X+) | SPEC1 p.13 |
| `#C35` unknown | signal-input acks | SPEC1 p.28 |

### 13.1 Where the captures override SPEC1

| SPEC1 says | Captures show | Driver/sim follows |
|------------|---------------|--------------------|
| A mode switch resets the outputs [SPEC1 p.32, p.34] | `$A` outputs persist across `@M` changes [CAP g_motions, mounts] | captures (outputs persist) |
| `#R` range 10..400 ms [SPEC1 p.19] | `#R470`, `#R700` acknowledged [CAP g_multi_C11, g_multi_C12] | captures (no 400 cap) |
