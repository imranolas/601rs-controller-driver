# 601rs — Protocol & Calibration Reference

**What this is.** Everything known about driving the CNC-Control controller in the Gunnar
601rs from a host, with the source of every fact. The Rust driver in `crates/` is built against
this document and cites it rather than repeating it.

**How far to trust each fact.**
- Command semantics come from the controller's programming manual, v1.00/00b [SPEC1].
- Where the captures or sessions show the 601 behaving differently, they win (§13.1).
- Constants, sequences and calibration come from serial captures of GMC driving the 601 [CAP],
  from how GMC does things [GMC] and from its parameter file [GCF].
- *(unverified)* marks a claim nothing backs yet. *(sim assumption)* marks behaviour the
  simulator assumes until a session on the machine settles it.
- §10 lists each open question with the `mc601 probe` or `mc601 check` stage that answers it.
  Session logs from the machine go in `testdata/sessions/` [SESSION], and this document is
  updated from them.

### Citation key

| Tag | Source |
|-----|--------|
| **[SPEC1 p.N]** | The controller's programming manual, "CNC-Control — Bahnsteuerung für 4 Schrittmotorachsen", **v1.00/00b, 20 Jul 2000**, SIG Positec / B. Lewetz (German). The newest version found, and the one the driver is built to, with no fallbacks. Not included here (`docs/spec/README.md`). |
| **[SPEC p.N]** | The same manual at v0.99/69, 18 Apr 2000, a draft. Cited only where it differs from SPEC1. |
| **[RADIUS]** | A flowchart of the controller's radius slowdown ("Tangentialverzögerung, Steuerung Weissenberger") that comes with the manual. |
| **[GMC]** | How GMC, Gunnar's own software for the 601, does it: learned from studying GMC 4.0 and 5.0, and not reproduced here. The captures were made with GMC 4.0.502.683. |
| **[CAP name]** | GMC serial captures in `testdata/captures/` (`g_*.txt`, UTF-16 export of a software port monitor, 1-second timestamps, no line settings). `mc601 capture <file>` decodes them. |
| **[SESSION name]** | Logs written by `mc601` at the machine, in `testdata/sessions/`. |
| **[GCF]** | GMC's machine and tool parameters for the 601 (its `.gcf` file; not included). |
| **[SYS]** | The controller parameter file for the 601 that GMC installs (`CNCCON_2.00_601.SYS`; not included). |
| **[HELP …]** | GMC's user documentation: [HELP RN] its release notes, and **[HELP CA n]** page n of its Configuration Assistant (calibration of borders, overcuts, trims, the throttle, COE). |

**Captures** [CAP]:
- `g_init`: power-on, licence exchange, configuration, homing (66 s)
- `g_motions`: jogs on every axis, outputs (clamps, heads), `@B`
- `g_regular_mount`: one rectangular window (100 × 200 mm), 1.5 mm board
- `g_oval_mount`: one oval window, cut as one lap plus GMC's overlap (1.9 laps, §7.1) in 1,107
  vectors
- `g_multi_C10`, `g_multi_C11`, `g_multi_C12`: twin windows with three profiles
- `g_multi_Cx_20` … `g_multi_Cx_100`: speed sweep, 20–100 %

---

## 1. Physical / transport layer

- **Link:** RS-232 null-modem cable to the controller, a compact DOS PC running "CNC-Control"
  [SPEC1 p.5]. GMC used COM4 [GCF `LastSuccessfullyUsedComPort`]; on macOS or Linux, a USB-serial
  adapter (`mc601 ports` lists them).
- **Encoding:** ASCII. Every command and reply ends with CR (0x0D); fields are separated by
  commas [SPEC1 p.6]. Distances and positions are signed 32-bit integers in µm or steps
  [SPEC1 p.7].
- **Line: 115200 baud, 8-N-1, no hardware handshake.** The spec: "19200/57600/115200 baud
  selectable (default 115200), 8 data bits, 1 stop bit, no parity" [SPEC1 p.6].
  - GMC uses 115200 in v4 and v5, and has no other baud setting [GMC].
  - The captures agree. In `g_oval_mount` the host wrote 24,253 bytes in one burst ending with
    `@X`, and the `@X14` reply is stamped at most 3 s later. At 9600 baud the burst alone takes
    25 s; at 115200, 2.1 s.
  - An earlier "9600" came from the port monitor's own settings, not from the line: a software
    monitor records the application's reads and writes whatever the baud rate.
  - `#C18,<code>` sets the baud (5 = 9600, 6 = 19200, 7 = 38400, 8 = 57600, 9 = 115200),
    effective after a restart [SPEC1 p.26]. [SYS] has `CB=9`. The driver never changes it.
  - RTS/CTS is reserved for a binary protocol that was planned but never described; the text
    protocol uses the 0x02/0x03 bytes below instead [SPEC1 p.5].
- **GMC's port settings:** no handshake, DTR and RTS asserted, read and write timeouts 128 ms
  [GMC].
- **macOS with a Prolific PL2303 adapter** (the one at the 601): the `serialport` crate writes
  the termios with 9600 and sets 115200 with `IOSSIOSPEED`, which reads back as set but leaves
  the line unreadable to the box: nothing comes back. A termios write at 115200 works, so the
  driver writes the speed that way as well (`link::serial`) [probe 2026-11-10].
- **The line is the bottleneck.** Ten bits per byte gives **11,520 bytes/s**, 87 µs per byte.
  GMC's vector `L20,X15696,Y10776,Z774⏎` is 23 bytes, so the line carries at most about 500 of
  them per second. Polls, mode switches, parameters and path data all share it (§5).

**Replies** [SPEC1 p.6–7, p.12]:
- In modes 0, 1 and 4, every command gets either CR or `*NNN` (§2.6).
- In **mode 2, accepted commands get no reply**, only errors. The look-ahead needs several
  commands queued ahead of the one running, so the host must not wait for each. Observed:
  `L20,…` gets nothing back [CAP g_regular_mount].
- **Master commands (`@`) are answered in every mode** and are processed at once, even when the
  buffer is full and the host has been told to pause.
- `*NNN` also reports **machine faults** (a limit switch, the stop input), which arrive when
  they happen and answer nothing [SPEC1 p.6]. The host must tell the two kinds apart (§2.6).

**Signal bytes** from the controller. Each is switched on by a parameter, and they can arrive
between replies or inside one: `⏎@X18⏎<01><01><04>` [SPEC1 p.7, p.23, p.28; CAP g_init].

| Byte | Meaning | Enable | Seen in the captures | GMC |
|------|---------|--------|----------------------|-----|
| `0x01` | an axis came to rest after a move ("positioned") | `#C5,1` | after mode-1 moves, homing and `@B` [CAP g_init, g_motions]. **Never after a mode-2 path**: none in any cut capture, although `#C5,1` is set | sets `#C5,1` at connect; its serial path polls `@X` instead |
| `0x02` | buffer has room again (like XON) | `#C7,1` | never | never enables `#C7` |
| `0x03` | mode-2 buffer almost full (like XOFF) | `#C7,1` | never | its USB path holds data while the buffer is almost full [GMC] |
| `0x04` | homing finished | `#C6,1` | after `$Hz` and `$Hxy` [CAP g_init] | never sets `#C6`: the 601 boots with it on |
| `0x05` | keep-alive ("node guarding"), after `<ms>` with nothing else sent | `#C8,<ms>` | never | never enables |
| `0x06` / `0x07` | signal input 1 (I221, the **green start button**) went high / low | `#C35` bit 1 | never (the button was not pressed while connected) | sets `#C35,1`; a `0x07` during a poll means "start button while cutting" and stops the job [GMC] |
| `0x08` / `0x09` | signal input 2 (I222) went high / low | `#C35` bit 2 | never | — |

- **0x02/0x03 contradiction.** SPEC1's prose says 0x02 comes "about 200 characters before the
  usual XOFF" and 0x03 when there is room again [SPEC1 p.5]. Its own byte table [SPEC1 p.7], the
  v0.99 table [SPEC p.7] and GMC's USB code all say the opposite: **0x03 = almost full, 0x02 =
  room again**. The driver follows the table; `mc601 check` stage 6 settles it.
- **Why the controller has its own XON/XOFF.** With a real XOFF, a Windows serial driver can send
  nothing, not even an abort, until XON comes. With these bytes the host itself holds back path
  data and can still send master commands such as `@B` [SPEC1 p.5, p.7].
- GMC's "`0x07` = start button" is I221 going **low**, i.e. the button being released.
- **`0x06` in the other direction** is the host's ACK during mode-9 file transfer (GMC's licence
  read, §2.7) [CAP g_init]. The driver never enters mode 9.
- **Host watchdog `#C19`** ("node guarding master"): while axes move, if nothing arrives from the
  host for this many ms (1–10000; 0 = off), the controller aborts the motion and sends `*068`
  [SPEC1 p.26, p.40]. GMC never sets it.

---

## 2. Command grammar

The controller works in **modes**, switched with `@M<n>`. Each mode accepts its own family of
commands; **master commands (`@`) work in every mode** [SPEC1 p.6, p.12].

| Mode | Purpose | Commands | Reply |
|------|---------|----------|-------|
| any | master control and queries | `@` | reply or CR |
| 0 | parameters | `#` set, `&` query | CR or value |
| 1 | setup moves: jog, homing, outputs. Every move ramps up and down on its own; no path optimisation | `$` | CR |
| 2 | **machining**: a buffered path with look-ahead speed control | `L`, `V`, follow-on vectors, `G`, `A`, `D`, `W`, `I`, `C` | **none**, only errors |
| 3 | batch: runs the text program `CNCCON.CMD` stored on the controller | (file) | — |
| 4 | velocity: axes run endlessly at speeds that can change on the fly | `!` | CR |
| 9 | file transfer and firmware update, "used only by external programs" | — | — |

[SPEC1 p.6, p.12–13, p.30, p.32, p.36–38]. A mode can only be changed at standstill, otherwise
`*028`. `@M` answers with two digits: `@M01` [SPEC1 p.12–13].

- **Mode 3** starts by itself at power-on if `CNCCON.CMD` exists and input INOBATCH (I234) is not
  active. Its commands are `REF`, `GOTO`, `MOVE`, `WAIT`, `INL`/`INH`/`INR`/`INF` (wait for low,
  high, rising, falling), `SPEED`, `SET`, `RESET` [SPEC1 p.36]. GMC never uses it.
- **Mode 4:** `!G<g>` picks the speed entry (default 1); `!X<v>[,Y<v>…]` sets signed per-axis
  speeds, effective at once; `!A`, `!B` (brake), `!S` (stop) [SPEC1 p.37–38]. GMC jogs in
  mode 1 instead.
- The driver uses modes 0, 1 and 2 only.

**Axes:** X, Y, Z (the blade's rotation) and U (not fitted). In mode-1 and mode-2 moves an
**uppercase axis letter means absolute and lowercase relative**, chosen per coordinate
[SPEC1 p.30, p.33].

**Speed table:** 100 entries, each holding a speed and a braking angle. Every move names the
entry it runs at [SPEC1 p.7, p.17]. Speeds above the 40 kHz step rate are clamped [SPEC1 p.17].
GMC runs every vector and jog at entry 20 and homes with entries 10 and 11.

### 2.1 `@`: master commands (any mode)

| Command | Reply | Meaning |
|---------|-------|---------|
| `@V` | `@V2.00/55W 19.10.10 2807001` | Version/revision, date, serial number [SPEC1 p.15; CAP g_init]. `@V1`–`@V3` return the parts, `@V4` the controller number. The 601 omits the space after `@V` that the spec's example shows. GMC also sends `@V7` and `@V8` (not in the spec). |
| `@M<n>` / `@M` | CR / `@M0n` | Set / query the mode [SPEC1 p.12–13]. |
| `@X` | `@X<r><f>` | **Status** [SPEC1 p.13]. `r` = 0 stopped, 1 moving. `f` = hex flags: **1** waiting for an input or a dwell (mode 2), **2** error, **4** home position known (see note), **8** homing. Seen: `18` homing, `04` idle, `15` moving and waiting (the dead time), `14` moving. Flag 2 has never been seen. In mode 2 the run flag is set from the first path data, even while the controller waits; use flag 1 to see the wait [SPEC1 p.13]. |
| `@Y` | `@Yhhhh` | Switch states, bits `sbff rrrr uuzz yyxx`: `s` stop input, `b` machine ready, `ff` undefined, `rrrr` home switches (X lowest), then a pair per axis (U Z Y X) with the **lower bit = plus limit**. `@Y4001` = machine ready, X+ limit hit [SPEC1 p.13]. |
| `@E` | `@E<n>` | Current error, the same numbers as `*NNN`; 0 = none [SPEC1 p.14]. |
| `@P<a>` / `@L<a>` | `@PX-23450` / `@LX44149` | Axis position in µm / in steps [SPEC1 p.14; CAP g_motions]. Open-loop steppers, so this is the commanded position. |
| `@B` | CR, then `0x01` | **Brake**: all axes stop with a ramp; positions are kept. To read the position, first wait for standstill (`@X` or `0x01`) [SPEC1 p.14; CAP g_motions]. Whether a brake in mode 2 discards the rest of the buffered path or only pauses it, the spec does not say (stages 4 and 6). |
| `@S` | CR | **Emergency stop** without a ramp. Positions are lost; home again. "Only in emergencies" [SPEC1 p.14]. |
| `@N<n>` / `@N` | CR / `@N<n>` | Set / read the mode-2 **command number**: "the command currently being processed", for progress displays [SPEC1 p.14–15]. The overview calls it the interface's "handshake with consecutive command numbers" [SPEC1 p.6]. What counts, and its width, are open (stage 5). |
| `@H` | `@H86` | Interface buffer fill in % [SPEC1 p.16]: "in mode 2, decide with it whether to send more vectors or wait". |
| `@I<n>` | `@I<n>,b` | Read logical input 0..255; with no number, all 256 [SPEC1 p.16]. |
| `@Q<n>` / `@Q<n>,b` | `@Q<n>,b` / CR | Read / set a logical output [SPEC1 p.16]. Q242–Q249 are aux signals 1–8, the `$A`/`A` bits 0–7, so `@Q` reads back what a streamed `A` did [SPEC1 p.11]. GMC switches the vacuum with `@Q241,1/0` [GMC] (Q241 is nominally spindle-speed output 8); not in the captures. |
| `@O<pct>` / `@O` | CR / `@O<pct>` | Speed override 10–200 %, effective at once on all motion in all modes [SPEC1 p.16]. In mode 2 a stream can exempt rapids with `C16` and include moves again with `C8` [SPEC1 p.35]. The driver's `OverridePct` cannot hold a value outside 10–200. The 601 answers `@O0` until one is set (at power-on; GMC never sends `@O`) and moves at the programmed speeds: the driver reads it as 100 % [probe 2026-11-10]. **`@O50` has no effect on the 601**: accepted and read back, but mode-2 moves keep their speed after `C8` or `C16` alike, at 30 mm/s (40 mm in 1465 and 1470 ms) and at 100 mm/s, above `#S2501` (551 and 538 ms) [stage 5, 2026-11-10]. A speed knob would have to change the speed entries instead. |
| `@D<hh>` | CR | Set internal flags; `@D01` marks the machine homed without homing [SPEC1 p.15]. Never sent: the machine must really home. |
| `@R` | none | Restart the control program; rereads the parameters [SPEC1 p.15]. Never sent. |
| `@@` | CR | *Not in the spec.* GMC sends it before homing and after mode-2 errors [GMC; CAP g_init]; it probably clears the error state. The driver's `clear_error` sends it. |
| `@+`, `@$`, `@=` | | Diagnostics: internal states, the step buffer's pointers, the last step timings [SPEC1 p.17]. Not used. |

> **`@X` flag 4.** The spec contradicts itself: its flag table says the bit is 0 when the home
> position is known and 1 when homing is needed, but its example reads `@X04` as "stopped, home
> position known" [SPEC1 p.13]. The captures settle it: flag 4 is **absent** while homing
> (`@X18`) and **present** in every state after (`04`, `14`, `15`) [CAP all]. **4 = home
> position known.**

### 2.2 `$`: mode 1 (jog, homing, outputs)

Every mode-1 move ramps up and down on its own; nothing is optimised across moves [SPEC1 p.30].

| Command | Meaning |
|---------|---------|
| `$H<axes>` | **Home** the axes in the given order, e.g. `$Hz`, `$Hxy`. Homing searches the switch, backs off it, moves the optional offset (`#O`), then zeroes the counters [SPEC1 p.30]. With `#C6,1` the CR comes at once and `0x04` at the end; with `#C6` off the CR is held until homing finishes [SPEC1 p.23]. The 601 answers at once and sends `0x04` [CAP g_init]. Otherwise poll `@X` until it reads `0x` with flag 4. |
| `$L<g>,<a><steps>[,…]` | Move in **steps** at speed entry `g` [SPEC1 p.31]. For an endless jog, give a target beyond the machine; `@B`/`$B` then stops it without losing steps. `0x01` at the end if `#C5,1`. GMC jogs `$L20,x±999999` and then sends `@B` [CAP g_motions]; `$L10,X3` is an absolute move to X = 3 steps [CAP g_init]. |
| `$E<g>,<a><µm>[,…]` | The same in µm [SPEC1 p.31]. |
| `$C<a>` / `$C` | **Zero the step and position counters** of one axis / all axes [SPEC1 p.31]. |
| `$B` / `$S` | Stop the move with a ramp / at once (positions lost) [SPEC1 p.31–32]. |
| `$A<hex>` | Set the 8 aux outputs as a hex bitmask (§2.5) [SPEC1 p.32; GMC]. The spec says a mode switch clears them; on the 601 they persist (§13.1). |
| `$D<n>` / `$W<ms>` | Spindle speed 0–255 / wait. `$W` "is not needed with a host", since every mode-1 command is acknowledged before the next [SPEC1 p.32]. |

### 2.3 `#` / `&`: mode 0 parameters (set / query)

Parameters can only be set and read in mode 0 [SPEC1 p.17]. A query is the same letter with `&`
and no value, and the reply repeats the set format: `&C13` → `&C13,1` [SPEC1 p.29]. Values last
until the controller restarts unless saved with `#W` [SPEC1 p.17, p.28]. Neither GMC nor this
driver ever saves.

| Command | Meaning [SPEC1 page] | GMC sends |
|---------|----------------------|-----------|
| `#G<g>,<v>` | **Speed of table entry `g`** (1..100): steps/s or µm/s by `#C11` [p.19] | `#G20,<feed>` before every move; `#G10,1750`, `#G10,3000` and `#G11,750` for homing |
| `#B<g>,<deg>` | **Braking angle** of entry `g`, 0..90°: a direction change larger than this brakes to a stop [p.19] | `#B20,90` per job (`#B20,20` for fast pens [GMC]) |
| `#E<g>,<v>,<deg>` | Speed and braking angle together [p.19] | — |
| `#M<v>` | Maximum speed [p.19] | `#M36000` at connect (`AbsMaxSpeed` [GCF]) |
| `#R<ms>` | **Ramp time**: the shortest time to reach `#M` from rest [p.8, p.19]. Spec range 10..400; the 601 accepts 470 and 700 (§13.1). GMC derives it from the tool's acceleration (§3.1) [GMC] | `#R112`, `#R225`, `#R250`, `#R300`, `#R470`, `#R700` |
| `#S<v>` | **Start/stop speed**: below it no ramp is needed; also the speed for tight radii and large direction changes [p.8, p.19] | the tool's `StartStop`, 400–4000 [GMC] |
| `#Z<a>,<n>` / `#N<a>,<n>` | Resolution: steps per revolution / µm per revolution [p.20] | `#ZZ,3000`, `#NZ,10000` at connect [GMC] |
| `#U<a>,<steps>` | Backlash compensation [p.20] | — |
| `#Y<a>,<0 or 1>` | Invert the axis's direction signal [p.20] | — |
| `#O<a>,<v>` | Offset moved after homing, before the counters are zeroed [p.20] | `#OX,300`, `#OY,300`, `#OZ,0` before homing |
| `#D<a>,<dir>,<g1>,<g2>,<g3>` | Homing direction (0 negative, 1 positive) and the speed entries for the search, the back-off and the offset [p.20] | — (the probe and the checks read `&D`) |
| `#I<n>,<port>,<bit>,<inv>` / `#Q…` | Map a logical input / output to hardware: port 0 = unmapped, 1–4 = LPT1–4, anything else a port address; bit 0..7; invert flag [p.21] | only on other firmware versions [GMC]; on the 601 only `&I240` |
| `#H<n>,<v>` | Usage counters (pages 10 and 21 number them differently) [p.10, p.21] | `#H1,` / `&H1` [GMC] |
| `#P<a>,<µm>` / `#L<a>,<steps>` | Declare the current position [p.28] | — |
| `#W` / `#X` | Save the parameters to disk (CR once written) / reread them (no reply) [p.28] | never |
| `#C<n>,<v>` | Configuration registers, below | |

**`#C` registers** [SPEC1 p.18 overview, p.22–28 detail]. "When" says when GMC writes the
register: once at **connect** [CAP g_init], before each **use** (a move, a cut, homing), or
**never**. For the "never" ones the 601 runs on its own boot values, which `mc601 probe` reads.

| Reg | Meaning | Page | GMC | When |
|-----|---------|------|-----|------|
| C1 | **Mode-2 dead time (ms)**: after the first vector the controller waits this long before moving, so the look-ahead has several vectors. `C1` in the stream skips it for one vector | 22, 35 | `0` travel, `100` straight edge, `200` curve, `50` at connect [GMC] | use |
| C2 | Extra braking criteria, bits: 1 XY-to-Z direction change beyond the braking angle; 2 change of speed entry; 4 start and end of a Z move; 8 U moves. An XY direction change beyond the braking angle always brakes | 22 | `0` [GMC] | connect |
| C3 | LPT port for the step/direction output | 22 | — | never |
| C4 | Controller screen output: 0 everything, 2 nothing ("recommended" with no monitor), 3 states only. Display costs controller time | 22–23 | — | never |
| C5 | Send `0x01` when a move ends | 23 | `1` | connect |
| C6 | Send `0x04` after homing (and the CR at once) | 23 | — | never; on at boot |
| C7 | `0x02`/`0x03` flow control | 23 | — | never; on at boot (`&C7,1` [probe 2026-11-10]) |
| C8 | `0x05` keep-alive interval (ms) | 23 | — | never |
| C9 | Ramp profile: 0 linear computed, 1 linear table, 2 sinusoidal, 3 exponential, 4 user table `RAMPE.DAT` | 23–24 | — | never |
| C11 | Units: 0 steps and steps/s, 1 µm and µm/s | 24 | `0` | connect |
| C12 | Ramp when homing reaches the switch | 24 | `1` | connect |
| C13 | Z is a rotary axis: it always turns the shorter way (212° becomes −148°) | 24 | `1` | connect |
| C14 | Dynamic path speed, bits: 1 diagonal, 2 radius, 4 tangential, 8 buffer fill (§2.4.1) | 24–25 | `13` = 1 + 4 + 8 [GMC] | connect |
| C15 | Hardware watchdog: 0 none, 1 Rocky418 board | 25 | — | never |
| C16 | Usage counters, bits: 1 operating seconds, 2 aux signal 1 count, 4 aux signal 2 count | 25 | `4` | connect |
| C18 | Baud code (§1); effective after a restart | 26 | — | never |
| C19 | Host watchdog (ms) (§1) | 26 | — | never |
| C20 / C21 | Radius slowdown: smallest / largest radius (§2.4.1). Spec minimums 1000 / 100000 | 26 | `400` / `12000` or `5000` (`CurveMin/MaxRadius × ScaleY` [GMC]) | use |
| C23 | Step pulse stretch; "as low as possible" | 26 | — | never |
| C24 / C25 | Radius slowdown: lower / upper threshold, % | 26–27 | `30` / `100` [GMC] | use |
| C26 / C27 | Radius slowdown: how often a change up / down must recur | 27 | `5` / `5` [GMC] | use |
| C28 | Steepness of the slowdown below the smallest radius | 27 | `1` [GMC] | use |
| C29 | Auto-home at controller start-up: axis digits in order (`21` = Y then X; 0 = off) | 27 | — | never |
| C30 | Spindle speed port | 27 | — | never |
| C31–C34 | CMOS addresses of the usage counters (Rocky418: 105, 109, 113, 123) | 27–28 | `105`, `109`, `113`, `123` | connect |
| C35 | Signal-input bytes: 1 = I221 as `0x06`/`0x07`, 2 = I222 as `0x08`/`0x09`, 3 = both | 28 | `1`, for the 601, 3001, 4001 and 801 [GMC] | connect |
| C37 | *Not in the spec.* Accepted with CR | — | `1`, for the 601, 3001, F1 and 801 [GMC] | connect |
| C52 | *Not in the spec.* **Rejected with `*021`** (parameter id invalid) by the 601 [CAP g_init] | — | `800`, sent to every model [GMC] | connect |
| C97 | *Not in the spec.* The controller's clock, `day,month,year,hour,min,sec`: `&C97` → `&C97,165,165,2065,165,22,39` (unset fields read 0xA5) | — | read at connect [GMC] | connect |

### 2.4 Mode 2: path commands (no reply)

"In mode 2 the controller uses look-ahead speed optimisation and holds the requested path speed
exactly. The first command must give the kind of data and the speed; following commands may
leave them out." All braking and dynamic settings act **only in mode 2** [SPEC1 p.32].

| Command | Meaning | GMC |
|---------|---------|-----|
| `L<g>,<a><steps>[,…]` | **Vector in steps** at speed entry `g`. Axes not given stay put; each coordinate's case picks absolute or relative [SPEC1 p.33] | `L20,X…,Y…,Z…`, absolute, for every vector [GMC]; `L20,x…,y…,z…` for its relative moves [GMC] |
| `V<g>,<a><µm>[,…]` | The same in µm [SPEC1 p.33] | — |
| `<a><v>[,…]` | **Follow-on vector**: no letter, no entry. Reuses the kind and the entry of the previous vector: `X2300,Z0` [SPEC1 p.33] | only on its USB path, after a first `L20` [GMC] |
| `G<g>,<v>` | New speed for entry `g`, "used by all following moves with this entry" [SPEC1 p.34] | — |
| `A<hex>` | Aux outputs, the same bitmask as `$A` [SPEC1 p.34] | — |
| `D<n>` | Spindle speed 0–255 [SPEC1 p.34] | — |
| `W<ms>` | Dwell [SPEC1 p.34] | — |
| `I<±n>` | Wait until input `n` is high (`+`) or low (`−`). An unmapped input does not wait; an invalid number is `*066` [SPEC1 p.35, p.40] | — |
| `C<bits>` | Special functions for the next move (below) [SPEC1 p.35] | — |
| `S…` / `K…` | Bézier curve / arc: "not yet realised" [SPEC1 p.33–34] | — |

**`C` bits** [SPEC1 p.35]: **1** = run the next vector without the `#C1` dead time ("for single
moves with no path after them"); **8** = from here on the speed override applies (tool in the
material); **16** = from here on no override (tool clear, rapid). **2** (diagonal first) and
**4** (one axis at a time) are "not yet realised". `C0` cancels a pending function before the
move; an invalid value is `*067`. The spec says "values 0..7" yet defines 8 and 16. The driver
encodes only 1, 8 and 16, or 0 (`StreamFlags`).

**Compact paths.** The follow-on form drops `L20,`, and relative coordinates are short, so after
one absolute vector a path can be sent as relative steps with unchanged axes left out:

```
L20,X15696,Y10776,Z774     GMC: every vector absolute and in full (23 bytes)
L20,X15696,Y10776,Z774     compact: the first vector absolute,
x-21,y14,z3                then relative steps (12 bytes)
y30                        an axis that does not move is left out
```

`Command::path` emits this form and skips repeated points. The spec defines the follow-on
vector; whether firmware 2.00 accepts it, and whether it still follows a `G`, `A` or `W`, is
open (stage 5).

**A whole cut can be one stream.** `A`, `W` and `I` sit in the buffer and run in order with the
vectors, so a cut (head down, settle, path, head up) needs no mode switch and no round trip, and
`@N` reports progress through it. While a `W` or an `I` holds the stream, `@X` reads `15`
(running and waiting), as during the dead time [SPEC1 p.13]. Each executed vector, `A`, `W`,
`I` and `C` advances `@N`, which numbers the command running (2 during the `W` of `A`, `W`),
and counts past 9999 without wrapping (`@N9998`, three vectors, `@N10001`). `@X04` can come a
few ms before a trailing `A` runs: in 3 of 6 runs of `L`, `A82` (30 and 100 mm/s) `@Q249`
still read 0 after `@X04`, and switched 3–9 ms later (stage 8, 2026-11-10). So after a stream
whose last command is an `A`, `@X04` does not yet mean the outputs are set: read them back
(`@Q`) before relying on them. The driver never ends on one: a head moved in the stream is
`A` then its settling `W` (`machine::lower`), and `@X04` comes after the dwell. An in-stream `G`
takes effect in sequence, when reached [stage 5, 2026-11-10]. The simulator still counts
without `C` and applies `G` on receipt.

### 2.4.1 Path speed control (what the controller does with the buffer)

The controller plans the speed along the buffered path itself [SPEC1 p.8–9, p.22, p.24–27].

It **brakes to a stop** when [SPEC1 p.8]:
1. the path ends and no more data is buffered. This also happens mid-path when the host falls
   behind: "with mostly small vectors and too high a speed, the step buffer runs empty now and
   then and the controller must brake to avoid losing steps";
2. a wait command (`W`, `I`) comes;
3. the direction changes by more than the entry's braking angle (`#B`);
4. the speed entry changes (only with `#C2` bit 2);
5. the Z axis moves (only with `#C2` bit 4).

GMC sets `#C2,0` and `#B20,90`. So within one speed entry only a turn of more than 90° and the
end of the data stop the carriage; turning the blade alone does not.

It **slows down without stopping** according to `#C14` [SPEC1 p.24–25]:
- **1, diagonal:** the XY speed is scaled by the direction of travel (71 % = 1/√2 at 45°), so
  the path speed, not each axis's speed, matches the set speed.
- **2, radius:** the radius is estimated from the vector length and the angle between vectors,
  averaged over the last three short vectors. XY only.
- **4, tangential:** as 2, but the radius comes from the rotation of axis 3 (Z, the blade).
  "Generally more accurate than 2."
- **8, buffer fill:** if the interface buffer stays under 10 %, the controller slows down so the
  buffer can refill.

GMC sets 13 = diagonal + tangential + buffer fill. With bit 4 the controller reads curvature
from Z, so a curve must carry the blade's tangent in Z for the controller to slow down on it
*(inference from the spec)*.

**Radius slowdown parameters** [SPEC1 p.9, p.26–27]. Only vectors shorter than RADMAX are
considered. At a radius ≥ RADMAX (`C21`) the carriage runs at full speed; at RADMIN (`C20`) at
the start/stop speed (`#S`); in between, linearly. Below RADMIN the speed keeps falling linearly
below `#S`, scaled down further by `C28`. A new speed that differs from the current one by more
than `C25` % applies at once; by less than `C24` %, it is ignored; in between it must recur
`C26` times (faster) or `C27` times (slower) first. GMC's values in steps (`#C11,0`): `C20` 400
(10 mm), `C21` 12000 (300 mm) or 5000 (125 mm), `C24` 30, `C25` 100, `C26`/`C27` 5, `C28` 1.

The 2003 flowchart [RADIUS] describes a later variant: vectors longer than RADMAX/2 always run
at full speed; fixed bands at 25 % and 50 % with counters take the place of `C24`–`C27`; a
change must hold for two vectors in a row before it applies. Which variant firmware 2.00 runs is
unknown. Nothing in the driver depends on it.

### 2.5 `$A` / `A` output bitmask

Bit `i` of the hex value is aux signal `i + 1`, logical output `Q242 + i` [SPEC1 p.11, p.32]. The
roles come from the machine parameters [GCF `i_*`; GMC]:

| Bit | Hex | `.gcf` name | Role, from how GMC uses it on the 601 |
|-----|-----|-------------|------|
| 0 | 01 | `i_t` | the bevel blade's plunge: on to cut, off to take the blade out. Alone, GMC's slow head-down for closed curves [GMC] |
| 1 | 02 | `i_bh` | holds the **bevel** head up. On at connect [GMC] and for the bevel's full head-up [GMC]; off whenever the bevel goes down; and on, untouched, while the straight head cuts (`4A`), so it keeps the bevel head clear |
| 2 | 04 | `i_b` | joins `i_t` for the bevel's normal head-down; never set alone. GMC calls the head-down without it "slow" [GMC] |
| 3 | 08 | `i_sh` | the straight head: on down, off up; nothing else [GMC] |
| 4 | 10 | `i_dd` | a 250 ms pulse [GMC], only in the V-groove depth adjustment: not used in mat cutting |
| 5 | 20 | `i_pl` | the clamp for a board on the **right** half of the table (below) |
| 6 | 40 | `i_pr` | the clamp for a board on the **left** half, where boards are registered: every job in the captures clamps with it |
| 7 | 80 | `i_free` | spare on the 601: GMC sets it only with a third tool position's head down on its native-USB machines [GMC] |

So the bevel head has three states: up (`i_bh`, `02`), down with the blade out (all three
off, `00`: GMC treats it as the head staying down [GMC]), and cutting (`i_t` + `i_b`,
`05`, or `i_t` alone, `01`). The straight head has two, and every straight-head output keeps
`i_bh` on: `08` alone would drop the bevel head too.

**The clamps** [GMC]: for a board up to half
the table's width (780 mm on the 601), GMC closes
`i_pr` (`40`) alone; a wider board, and the connect, get both (`60`). **Cut Other End (COE)**
switches the clamping side and moves the registration to the mirrored end of the table, so the
operator can load one end while the machine cuts the other: GMC shifts the job by
the table's width less the board's and clamps with `20`, where the usual end
clamps with `40`. With the park at the right (high X), `40` is the left clamp and `20` the
right, as the capture notes say: the `.gcf` names have them the other way round. Stage 3 still
asks the operator. GMC calibrates COE as the size of the table: a test cut at each end, the far
border checked to ±0.2 mm [HELP CA 4000]. COE needs a second registration in the machine profile
and a choice of end per job; neither exists yet.

What GMC writes [CAP g_init, g_motions, g_regular_mount]:
- `02` neutral (`i_bh` only), at the end of every job.
- `62` both clamps, at connect.
- Bevel edge: `45` head down (`40` + `05`: `i_t` + `i_b`), then `40`, the blade out with the
  head still down, then `42`, the head up. On board up to `SafeTurn` (0.2 cm [GCF]) GMC hops between an opening's edges
  after `40` alone and raises the head only at the end of the opening [GMC]. `05` is GMC's
  default bevel head-down [GMC].
- Bevel oval: `41` head down (`40` + `01`: `i_t` without `i_b`) [CAP g_oval_mount]. This is
  GMC's **slow head-down** [GMC], which it uses instead of the usual one for a shape that
  gets its overlap (a closed smooth curve, §7.1) unless it is an outer contour [GMC]; its other path tests that the cut
  closes on itself, the first segment within 22.5° of the way back from the last point to the
  first [GMC]. On the F1 it sets `i_t` and `i_b`; on the 601,
  `i_t` alone. The name says why: on a rectangle the blade goes in at the overcut, outside the
  edge that shows, but on a closed curve it goes in on that edge. GMC's Configuration Assistant
  says how [HELP CA 4100 "Setting the Throttle"]: a throttle screw on the Z head "creates a
  delayed blade entry into the matboard" for circles and small curves, so the blade "does not
  leave any marks". It is set so that, on a circle cut without overcut, the blade's entry starts
  between two marks along the curve: the blade comes down slowly while the carriage already
  moves, and the circle overcut ("the amount of time that the blade head retraces the curved
  opening", `CurveAddon`, §7.1) then cuts over the entry. So `i_t` alone lowers the blade
  through the throttle, and `i_b` most likely bypasses it for the normal plunge.
- Straight edge: `4A` head down (`40` + `0A`: `i_bh` + `i_sh`), then `42` head up.
- Between jobs and tools: `42` (`i_bh` + `i_pr`); `22` (`i_bh` + `i_pl`) in the jog capture.

GMC sleeps `HeadUpDown` ms (200–500 by profile, §8) after switching a head [GMC]. Separately, it switches the vacuum through `@Q241` (§2.1).

**Clamp sides, confirmed at the machine** (stage 3, 2026-11-10): `40` closes the **left** clamp
and `20` the **right**, as the capture notes label `42` and `22`. The `.gcf` names (`i_pr` on
`40`, `i_pl` on `20`) do not name the side.

**Where the clamps press** [GCF] [GMC]. The 601 is set up with clamp pads (`Clamps` true), not
a vacuum table, so GMC checks a layout against each pad, not against the whole-edge borders
`SysBx`/`SysBy` (those apply only on a machine with neither) [GMC]. Positions are pad centres
in cm from the table's front-left corner, where boards register:

| Pads | Along | Centres (cm) |
|------|-------|--------------|
| `ClampsX` | the front edge, reaching `ClampHeight` 2 cm in | 9.7, 32.7, 57.7, 98.5, 123.2, 146.2 |
| `ClampsY` | the left edge, reaching 2 cm in | 10.2, 32.7, 66.7 |

Each pad is `ClampWidth` 4.3 cm along its edge [GCF]; GMC's default parameters have the same.
For Cut Other End GMC mirrors the left pads at the table's right end, `PageWidth` 156 cm
[GMC]. The pads on the table's left half are the ones a board up to 78 cm reaches, so they are
presumably the left clamp's (`40`), and the rest the right clamp's.

A cut that runs over a pad isn't refused: GMC lists the pads with their positions and numbers,
"Please deactivate the indicated clamps", highlights them, and lets the operator go on once
they are turned out of the way [GMC]. The driver does the same through the `clamp` op's
`turned` (PROGRAM_FORMAT §4.1).

### 2.6 Inputs, outputs and errors

**Logical inputs** [SPEC1 p.11]. Inputs are debounced for 2 ms and read every program cycle
(0.5–1 ms) [SPEC1 p.10].

| Input | Name | Meaning |
|-------|------|---------|
| I221 / I222 | ISIGNAL1 / 2 | signal inputs behind the `0x06`–`0x09` bytes. **I221 is the green start button** on the 601 [SYS `I221=279h,7,1`] |
| I223–I226 | ILST1–4 | power stage 1–4 ready |
| I227 / I228 | IFEST / ISPINDEL | clamping closed / spindle turning |
| I229–I232 | IEND1–4 | both limit switches of an axis on one line |
| I233 / I234 | IZUNTEN / INOBATCH | Z down / block batch mode |
| **I235–I238** | IREF1–4 | **home switches** of X, Y, Z, U. [SYS]: `I235=001h,6,1`, `I236=001h,7,0`, `I237=001h,4,0` (LPT1) |
| **I239–I242** | IENDM1–4 | **minus limit switches** of X, Y, Z, U |
| **I243–I246** | IENDP1–4 | **plus limit switches** of X, Y, Z, U |
| I247 | IBEREIT | machine ready |
| I254 / I255 | ISTOP / ISTART | stop / start |

`&I240` is an input-definition query. The 601's reply `&I240,000h,0,0` means the Y− limit is not
mapped [CAP g_init]; stage 7 drives into a limit only if `&I243` or `&I239` is mapped.

**Logical outputs** [SPEC1 p.11–12]: Q255 ready (no error), Q251 motors running, Q250 motors in a
ramp, **Q242–Q249 aux signals 1–8** (the `$A` bits), Q234–Q241 spindle-speed outputs 1–8, Q233
batch running.

GMC polls `@I221` for the start button while idle [GMC]. With
`#C35,1` the controller pushes `0x06`/`0x07` on each edge instead [SPEC1 p.28]. Before homing,
GMC checks that X and Y are not sitting on their home switches (`@I235`, `@I236`, then `@I240`
if mapped) and retries twice, 2 s apart [GMC; CAP g_init].

**Errors** are `*NNN⏎` [SPEC1 p.38–40]. The driver prints the SPEC1 text for every code it knows
(`ErrorCode`'s `Display`: `*033 (limit switch X+ …)`, from `protocol::errors::describe`).

| Kind | Codes |
|------|-------|
| **Rejections** (the answer to a command) | `*001` not valid in this mode · `*002` speed entry invalid · `*003` format invalid · `*004` speed invalid · `*006` ramp invalid · `*010` braking angle invalid · `*012` axis invalid · `*013`/`*015` resolution invalid · `*017` backlash invalid · `*019` home offset invalid · `*021` parameter id invalid · `*024` parameter exceeds limits · `*027` invalid mode · `*028` mode change only at standstill · `*029` unknown master command · `*030` machine already moving · `*031` command invalid · `*041` not homed · `*042`/`*043` spindle entry / speed invalid · `*044` baud invalid · `*045` wait time invalid · `*046` signal states invalid · `*055` parameter value invalid · `*060`/`*061` value too small / too large · `*066` bad input number in a mode-2 `I` · `*067` bad value in a mode-2 `C` |
| **Machine events** (arrive whenever they happen) | `*032`–`*039` **limit switch hit**: even = minus, odd = plus; `*032`/`*033` X, `*034`/`*035` Y, `*036`/`*037` Z, `*038`/`*039` U · `*040` stop input · `*068` host watchdog expired (§1) |
| **Either** | `*022` memory too small (the simulator raises it on a buffer overrun *(sim assumption)*) · `*056` machine not ready · `*062` standing on a limit switch · `*063` cannot home, standing on the switch · `*007`/`*008` service code required / invalid: once the start counter reaches 0, mode switches are refused [SPEC1 p.10] |
| Files, batch, mode 9 | `*023`, `*025`, `*026`, `*047`–`*054`, `*057`–`*059`, `*064`, `*065` |
| Not in the spec | `*069`: GMC treats it as a controller error [GMC] |

The spec's prose uses `*023` as its limit-switch example [SPEC1 p.6], but its table makes 023 a
file error and the limits 032–039. `@E` calls every code but 0 "still to be defined" [SPEC1
p.14]; the table came later.

**Answer or event?** A `*NNN` does not say whether it answers the waiting command or reports a
fault that crossed it, and the table cannot always tell: the "either" codes, codes the spec
lacks (`*069`), and in mode 2 a streamed command's error arriving while `@M` waits. So the
driver (`controller::core`) settles it instead of guessing. When a `*NNN` arrives while a
command waits for its reply, a probe goes out right behind the command (`@I1xx`, an input
nothing uses, as for a resync). Replies come in order, so once the probe's answer is in, so is
the command's: a CR or value before it is the command's answer and the `*NNN` was a fault;
nothing before it means the `*NNN` was the rejection. It costs one round trip per `*NNN`.

A fault is published as `Event::Unsolicited` and reported as `ControllerError::AsyncError` by
the next wait or stream, or by the next command that would move the machine or change its
outputs, settings or mode; until then none of those is sent. Reading state, `@B`, `@S` and `@@`
still go through.

Taking a `*NNN` as the answer to any command but a query, as the driver first did, goes wrong in
a way the runner's fault test found: a limit fault arriving just before a mode switch's
acknowledgement is taken for its rejection, the acknowledgement then pairs with the next
command, and the host's idea of the mode is wrong (replayed by
`a_fault_ahead_of_a_mode_switch…` in `crates/sim/tests/host_core.rs`).

### 2.7 Licence (mode 9, do not reproduce)

`@M9` → `:ULICENSE.LIC` → hex blocks, each followed by a `0x06` ACK → `@M0` [CAP g_init]. This is
mode-9 file transfer: GMC reads the licence file stored on the controller [GMC]. It is **not needed for motion**. A clean-room driver must
not send `@M9`, and must never trigger GMC's firmware-update path (an `.rtb` upload when the `@V`
version does not match) [GMC]. Mode 9 is "used only by
external programs" [SPEC1 p.38]. Whether the 601 accepts mode switches without the licence
exchange is the probe's first question.

---

## 3. Universal constants

| Constant | Value | Source |
|----------|-------|--------|
| **XY scale** | **40 steps/mm** (0.025 mm/step) | [GCF] `MachineScale.ScaleX/Y = 400 /cm`; GMC converts with it [GMC]. Matches the captures. |
| **Z (blade angle) scale** | **3000 steps per revolution**, 8.333 steps/° | `#ZZ,3000` = `ScaleZ × 360` [GCF `ScaleZ = 8.3333`; GMC]. Z is rotary (`#C13,1`). |
| **Z wrap** | Between cuts GMC brings Z back into 0..2999: a window's edges go 774 → 1524 → 2274 → **24**. Along one cut it keeps counting: the oval's Z runs 5 → 5521 (§3.2) | [CAP g_regular_mount, g_oval_mount]. The controller turns the shorter way. |
| **Step rate limit** | 40 kHz per axis (1000 mm/s on XY); higher speeds are clamped | [SPEC1 p.17] |
| **Feed law** | **`#G20 = 360 × Speed%`** steps/s = **9 × Speed% mm/s**; 100 % = 900 mm/s | [GMC]: `AbsMaxSpeed × Speed% / 100` with `AbsMaxSpeed = 36000` [GCF], `#C11,0`. Matches the speed sweep exactly [CAP g_multi_Cx_*]. GMC's speed reduction factor is 1 on the 601. |
| **Rapid (travel)** | `#G20,36000` = 900 mm/s (`PosSpeed = 100 %`) | [GCF] `PosSpeed`; [GMC] |
| **Ramp law** | §3.1 | |
| **Bevel blade inclination** | **42° to the board, so 48° from vertical** (§7) | [GCF] `Heads[Bevelled].Inclination`; how GMC reads it [GMC] |
| **Curve segment length** | One spacing per curve, picked by the curve's smallest radius from the `Distances` table: radius < 5 cm → 20/cm (0.5 mm); < 10 cm → 10/cm (1 mm); < 40 cm → 5/cm (2 mm); < 200 cm → 1/cm (10 mm). This is a chord **length**, not a deviation tolerance. On a constant radius the chords stray 0.001–0.006 mm in most bands (up to 0.03 mm for 10 mm chords just over 40 cm radius), and use 1–6× the points a 0.02 mm tolerance needs; more where the radius varies. The oval: 1,107 vectors over 928 mm (1.9 laps, §7.1), 0.28–1.27 mm apart (median 0.77); within 0.02 mm the same path needs 316 (§5). See the note below. | [GCF] `Distances`; [GMC]; [CAP g_oval_mount] |
| **BreakAngle** | 15°. Over USB, GMC ends a smooth run of curve points at any joint sharper than this. It is not the controller's braking angle, which GMC sets to 90° (`#B20,90`) | [GCF]; [GMC] |

**How GMC flattens curves.** For each curve, GMC samples about three points per cm,
estimates the radius at each sample, and keeps the smallest [GMC].
That radius picks a divisions-per-cm value from `Distances`, and the whole curve uses it, so its
tightest point sets the spacing everywhere. Only a curve gentler than every band (radius
≥ 200 cm) falls back to a radius from crossing the normals at the two ends, or to 10/cm.
The number of points is the arc length (cm) × divisions, doubled once if the end tangents
differ by more than 5° per point (not on Novex) [GMC]. Over USB, GMC groups curves into smooth
runs that end at any joint sharper than the break angle,
and lowers the divisions until each run fits in `PolyBufferCount` (3000 [GCF]) points. Nothing
in GMC limits the byte rate, which is why it hesitates mid-curve (§5). The driver's compiler
flattens to a chord tolerance instead (PROGRAM_FORMAT §4.2).

### 3.1 Ramp law (`#R`), confirmed

`#R` is the time to reach `#M` (36000 steps/s = 900 mm/s) from rest [SPEC1 p.8, p.19], so GMC
divides that speed by the tool's acceleration in mm/s²:

```
v_ref   = (AbsMaxSpeed / ScaleY) × 10        = (36000 / 400) × 10 = 900   mm/s
#R (ms) = min(700, v_ref / (accel / 1000))   = min(700, 900000 / accel)
```
[GMC]

| `.gcf` acceleration | Value (mm/s²) | `#R` | Used for |
|---------------------|---------------|------|----------|
| `_PosRampe` | 4000 | **225** | travel |
| `_RampeCut` | 3600 | **250** | cutting (most tools) |
| `_FastZRampe` | 8000 | **112** | short travel with a large blade rotation [GMC] |
| `_RampeCut` | 420 | **700** (clamped) | deboss and pens (not in the captures) |
| (thickness rule) | — | **470**, **700** | straight-head cuts on 2.7 and 4.6 mm board (§3.3) |
| (diagonal rule) | — | **253** | a cut whose first segment is off the axis: the oval (§3.3) |

These account for every `#R` in the captures except `#R300` at connect and `#R,100` before
homing [CAP g_init]. **`#R` is not a blade angle**; the blade angle travels in each vector's Z
(§3.2).

### 3.2 Blade angle (Z) along a path

**The formula.** In machine steps, GMC's Z for a vector is

```
Z = trunc(8.3333 × (heading − StartPZ + trim))
heading = direction of travel, clockwise from +X (travel towards −Y = 90°)
```

- `StartPZ` is the head's blade mounting angle: bevel 0°, straight 180° [GCF].
- `trim`: the bevel head takes the material's `ZCorrection_Straight` (3° on C10) on ordinary
  cuts and its `ZCorrection_Round` (0° on C10) on closed smooth shapes (§7.1). The straight head
  uses its own setting (`UseMatProfileSetting` false, `StraightCorrection` 0) [GMC; GCF].
- Checks [CAP g_regular_mount]: a bevel edge travelling −Y gives trunc(8.3333 × 93) = **774**; a
  straight edge travelling +X gives trunc(8.3333 × −180) = **−1499**. Both are in the capture.
  With C11 and C12 (trim 4°) the same bevel edge is trunc(8.3333 × 94) = **783** [CAP
  g_multi_C11, g_multi_C12].

**The trim is deliberate yaw.** Every profile carries a straight-edge trim of 0–7° and a curve
trim, 0° except C3's 1° (§8). So on every straight edge GMC runs the bevel blade 1–7° off its
direction of travel, on purpose. It corrects **hooked corners** [HELP CA 3800 "Hooked
Corners"]: "All cuts should run absolutely straight. If at the beginning of a cut a hooked
corner occurs, this can be corrected by using the straight degree correction." The angled blade
steers off line as it enters, and the trim points it back. It is calibrated per mat profile with
a test rectangle; on 1.3 mm board it should fall between 2° and 6°, or the Z axis needs aligning
again; and "the Round Degree Correction should normally be set to 0°". The values follow the
material, not the thickness (Arqadia back 7°, the 4.6 mm board 4°).

**Which heading.** Each vector carries the heading of **the chord it cuts**, the line from the
previous point to its own [GMC]. Along a
path Z stays continuous: an angle that would jump by more than 178° gets ±360° instead, so Z
keeps counting past one revolution (the oval runs from 5 to 5521). On the oval, Z matches the
heading of the chord it ends with to −0.5 ± 1.2 steps [CAP g_oval_mount].

**What the controller does with it.** An `L` vector moves every axis it names together, in
proportion [SPEC1 p.33], Z included. So the blade turns *during* each chord: it starts at the
previous chord's heading and reaches this chord's heading only at the end. Until then it is
off its direction of travel by the part of the turn still to go. On the oval the turn per chord
is 1.3° at most and 0.6° on average: that is how far GMC's blade is off on curves.

GMC also sends Z-only vectors (`L20,z<steps>`) [GMC], but on their own,
never inside a path.

**The driver's rule** (PROGRAM_FORMAT §4.2): at a smooth point, the direction halfway between
its two chords, which is the curve's tangent; curves flattened to at most 2° of turn per point;
at a corner, stop and turn in place. The blade is then never more than 1° off its travel, on the
true tangent at every point. Copying GMC's rule with tolerance-based chords would leave it 3–15°
off at the start of each chord on tight curves.

### 3.3 Starting a cut: GMC's soft start

`#S` is the speed the motors **start** at, with no ramp; above it they accelerate at the `#R`
rate [SPEC1 p.8, p.19]. Every head's `StartStop` is 4000 steps/s, 100 mm/s [GCF]. So a cut
starts by jumping from standstill to 100 mm/s, or straight to the cutting speed if that is
lower, with the blade already in the board. GMC softens that in three ways
[GMC]:

1. **Thickness soft start, straight head only.** From
   `DynamicMatSetting.MatLow` (0.2 cm) up, GMC lengthens the ramp toward `_MaxRampe` (1285 mm/s²,
   so 700 ms) and lowers `#S` toward `StartSpeed` (400 steps/s, 10 mm/s):

   ```
   f  = 1                          if thickness > MatHigh (0.35 cm)
        thickness / (MatLow + MatHigh)   otherwise
   #R = cut ramp + (700 − cut ramp) × f
   #S = StartStop − (StartStop − 400) × f
   ```

   `f` jumps from 0.64 to 1 at 3.5 mm. After the cut, GMC restores `#R250` and `#S4000`.
2. **Diagonal smoothing, every cut** (`DiagonalRampeSmoother` true, factor 1 [GCF]). The ramp
   is multiplied by 1 + d, where d is how far the first segment is off the nearest axis, divided
   by 45° (0 along an axis, 1 at 45°), capped at 1500 ms [GMC]. The oval's
   first segment is 1 % off the axis: `#R253`.
3. **Thin-board straight-head start.** Below `MatLow`, drawn shapes (not templates) take
   `ImportParameters.CBStartStop` as `#S` (1000, 25 mm/s) [GMC].

The captures [CAP g_multi_C10, g_multi_C11, g_multi_C12]:

| Board | Straight-head edges | Bevel edges |
|-------|---------------------|-------------|
| 1.5 mm (C10) | `#R250 #S1000` | `#R250 #S4000` |
| 2.7 mm (C11) | `#R470 #S2232` | `#R250 #S4000` |
| 4.6 mm (C12) | `#R700 #S400` | `#R250 #S4000` |

**Bevel edges get no soft start at any thickness.** Each one jumps to 100 mm/s (or to the
cutting speed) with the blade in the board, then accelerates at 3,600 mm/s²:

| Bevel edge | Starts at | Then accelerates over |
|------------|-----------|-----------------------|
| 1.5 mm at 95 % (855 mm/s) | 100 mm/s | ~100 mm (0.2 s) |
| 2.7 mm at 35 % (315 mm/s) | 100 mm/s | ~12 mm |
| 4.6 mm at 10 % (90 mm/s) | 90 mm/s | no ramp |

**How GMC does it** [GMC; 16 Nov]: the thickness soft start is for the
straight head only, so a bevel cut never gets it; its ramp only stretches for a first segment off
the axes, and its `#S` stays at the head's `StartStop`. The drawn-shape start (`CBStartStop`,
1000 [GCF]) could apply to bevel cuts, but the captures show bevel edges at
`#S4000` and straight edges at `#S1000` in the same job: GMC seems not to take a
window's outline for a drawn shape (not confirmed). The
cutting speed is the profile's speed %, the same on curves and straight edges (the radius
slowdown is the controller's, §2.4.1). After a head-down GMC waits the profile's `HeadUpDown`
(250 ms on C10, 400 ms on C12), else the head's `HDSleep`. Above `SafeTurn` (0.2 cm [GCF])
GMC lifts the blade and then the head after every edge; on thinner board the head stays down
between a window's edges with only the blade out (`$A40`), as the C10 capture shows.

Operators see a wobble at the start of cuts on thick board that changes with the cutting speed.
The profile's speed % is the only start setting that varies with thickness for the bevel, so
that fits. Whether the cause is the jump, the acceleration or the blade trim (§3.2) is open
(§10). The driver softens the bevel's start too (16 Nov): `#S` from 4000 down to 400 as the
board goes from 2 to 3.5 mm, and 400 on thicker board, the ramp unchanged
(`machine::SoftStart::bevel`; `crates/machine/src/lib.rs`, "Starting a cut"). `mc601 run
--gmc-bevel-start` cuts as GMC does, to compare.

---

## 4. Cut-sequence state machine

The order is **the same for every profile** [CAP g_multi_*]. Annotations follow [GMC]. Each window edge is a cut of its own:
the head goes up at every corner, the blade turns, the head goes down again. GMC sends every
`@M` twice (§5).

```
1. CONNECT and HOME (§4.1)
2. JOB SETUP
   @M1  $A42                    clamp
   @M0  #C20 #C21 #C24…#C28     radius slowdown for the tool
        #S<StartStop>  #B20,90
3. FOR EACH CUT (interiors first, boundary last):
     -- travel, head up --
     @M0  #C1,0                 no dead time
          #R225 or #R112        #R112 when the blade turns a long way
          #G20,36000            travel at 100 %
     @M2  L20,X,Y,Z             absolute; Z = blade angle for the coming cut
     poll @X until "@X0"        (§5)
     -- head down --
     @M0  #R250                 cutting ramp
     @M1  $A45 (bevel) or $A4A (straight)
     -- cut --
     @M0  #C1,100 or #C1,200    dead time: 100 ms straight edge, 200 ms curve
          #R… #S…               straight head on thick board: softer start (§3.3)
          #G20,<360 × Speed%>
     @M2  L20,…                 one vector per straight edge; a curve as all its vectors in one write
     poll @X until "@X0"
     -- head up --
     @M1  $A40 (bevel) or $A42 (straight)
4. END OF JOB
   $A42                         head up, clamps held
   @M0 #C1,0 #R225 #G20,36000  @M2 L20,X58860,Y7923,Z0     park (AutoPark 140 × 20 cm for the
                                                             bevel blade [GCF]); blade back to 0
   @M1  $A02                    release
   idle: poll @I221 (start button)
```

**Curves** differ from straight edges [CAP g_oval_mount]: head down with `$A41` (§2.5),
`#C1,200`, `#R253`, then every vector of the curve in one write. A closed smooth shape keeps
cutting past its start before the head comes up (§7.1).

**Abort:** `@B` (brake, positions kept) → head up (`$A02`) → park. `@S` only in an emergency: it
loses the position and the machine must home again [SPEC1 p.14].

**Feature order (from the captures):** every interior window is cut, each as a complete closed
loop, **before** the outer boundary. The boundary frees the piece, so it must come last
[CAP g_multi_*].

**Travel.** GMC's cut order comes from fixed sort keys, not from travel [GMC]:
- **Heads one after another,** by their configured priority, every cut of one head before
  the next; V-groove first. The priorities are configuration; in the captures the bevel head
  goes first.
- **Openings (contours):** farthest first, by the distance from the origin to each contour's
  bounding-box corner. A head with `ChildrenPreCut` cuts a contour lying inside another
  before it. A closed curve is cut clockwise, or by face side on a `MatThicknessDependent`
  head. Its start is the corner farthest from the origin, but only on the last contour of a
  head; the others start at their first segment. The manual order only chooses which
  entities are cut; it doesn't reorder them.
- **Straight lines (the border):** collinear lines that overlap or touch are merged, so a shared
  edge is cut once. Then they are sorted by their distance from the axes, descending, with
  the topmost horizontal first, and every line is turned to run +X/+Y (direction setting 3,
  which GMC sets for the straight head on machines without a vacuum table).
- **No travel minimisation,** except in vacuum-table mode, which picks lines and contours
  nearest-first. Nothing in the ordering refers to the clamp.

So within a window the edges follow round (travel of a few millimetres), but the second window
starts at a fixed corner, and after each border edge the head travels back across the board.
On `g_multi_C10` that is 1.53 m of head-up travel between cuts for 2.54 m of cutting, plus
1.46 m back to the park at (1471.5, 198.1) [CAP g_multi_C10]. The same cuts in the order the
route optimiser (`crates/route`) chooses travel 0.32 m between cuts: each window entered at its
nearest corner and the border cut as one loop, still finishing on the clamp-side edge
GMC's own optimiser lays out sheets (nesting, with "filler" pieces and a rotation switch); it
doesn't order cuts.

### 4.1 Connect and homing (the `g_init` timeline, 66 s)

| Time | Phase |
|------|-------|
| t+0–8 s | `@V`, licence read (mode 9, twice), `&C97` clock, `@V7`, `@V8` |
| t+8 s | `#C35,1`, `#C37,1`, `#C52,800` (→ `*021`); home-switch check (`@I235`, `@I236`, `&I240`); **zero the counters** `$CX $CY $CZ`; `$L10,X3`, `$L10,Y2` (each answered by `0x01`); `$A02`, `$A62` (clamps) |
| t+9 s | Mode-0 configuration: `#C12,1 #C13,1 #C20,400 #C21,5000 #C31…34 #C16,4 #C11,0 #ZZ,3000 #NZ,10000 #G20,36000 #R300 #M36000 #C5,1 #C14,13 #C1,50 #C2,0`, then the radius parameters and `#S4000` |
| t+9 s | **Z home:** `@@`, `#G10,1750` (`ZInitSpeed` [GCF]), `#OZ,0`, `@M1`, `$Hz` → CR, `@X18`, then `0x01 0x01 0x04`; `@X04` a second later [GMC] |
| t+10 s | **XY home:** `#G10,3000`, `@@`, `#G10,1750`, `#G11,750`, `#R,100`, `#OX,300`, `#OY,300` (offsets moved after homing), `@M1`, **`$Hxy`** |
| t+10–49 s | Homing. `@X18` (moving, homing, home not yet known) about 4 times a second for 39 s; `0x01` at t+44 and t+49, `0x04` at t+49 |
| t+49 s | `@X04`: stopped and homed |
| t+50–54 s | `@LX @LY @LZ`; the V-groove adjustment in mode 2 (`L20,X0,Y40600,Z749`, `Y41800`, `Y40600`: §4.1); park `L20,X58860,Y7923,Z0`; `$A02` |
| t+57 s → | Idle: `@I221` polling |

**Driver:** `$H<axes>` → wait for `0x04` **or** poll `@X` until the first digit is `0` with flag
4 set → read `@LX`/`@LY`. Homing uses the home switches I235–I237 [SPEC1 p.11; SYS], which are
real inputs even though no switch has been spotted on the machine yet.

**The controller homes; GMC only asks.** GMC sends speeds, the ramp and the offsets, then `$Hz`
and `$Hxy` (`$Hy`, `$Hx` on an F1) [GMC]. It never sends a
homing direction (`#D`), and on the 601 it does not rewrite the home-switch inputs either:
GMC does that only for other controller versions [GMC], and `g_init` holds no
`#I` or `#D` write. Direction and switches are the controller's own configuration; `mc601 probe`
reads them (`&D`, `&I235`–`&I237`).

**Home is the front-left corner, by the red button; the park is front-right.** GMC's machine
frame (cm) becomes steps by an offset and a scale, `trunc((y − StartPY) × ScaleY)`, with no
mirror anywhere, and a job reaches that frame by translations only (the one mirror is Cut
Other End's) [GMC]. So home, the board's registration
corner and the park share a Y edge: the board's corner is about 2 cm from home in X and Y, and
AutoPark (140, 20) cm is (58 860, 7 923) steps, 20 cm from Y's home switch. The operator
registers boards in the front-left corner, by the red button, and sees the head park close to
the front rail: home is front-left, +X to the right, +Y away from the operator, as
PROGRAM_FORMAT §2 has it. The timing of `g_init` agrees: from the park, X's homing ran 34 s
(about 148 cm at `#G10,1750`), Y's 5 s (about 20 cm to the front, then 7.5 mm back); then GMC
ran the head up the left side to the back rail at 900 mm/s, and parked. That run is what an
operator remembers as homing "up". `mc601 check` stage 3 asks which way Y moved, and confirms
the park and +Y before anything else moves.

**The run up the left side resets the head's V-groove switch.** The bevel head has a mechanical
switch for V-groove depth (GMC's "DBD"); a small stub at the back of the table toggles it when
the head is pushed against it (operator). After every homing GMC sets it to normal cutting with
a short routine [GMC]: on a machine whose head changes this way
(`DBDHeadChangeKind` ≠ 0, the 601) it drives to 3 cm short of `VGChangeY` (at the current X, 0,
the blade at 90°: `L20,X0,Y40600,Z749`), pushes on to `VGChangeY` (`Y41800`, 104.5 cm) and
comes back, with its own soft-limit check off, then reads the Y home switch and ignores the
result [GMC]. Its speeds are those of any of its positioning moves: `#C1,0 #R225 #G20,36000`
for the run there, and `#R112` for the push and back,
the fast ramp it gives a move under `FastZMinLenXY` [GMC] [CAP g_init]. The blade's angle picks the way the switch goes: 90° for normal cutting, 270° for
V-groove (`VGChangeZ` + 180) [GMC]. Heads switched electrically get a 250 ms pulse of `i_dd`
instead (§2.5) [GMC]. The push goes 30 mm past the 101.5 cm the
profile allows, so Y's travel reaches at least 104.5 cm. `mc601 --sim` puts the simulator's plus
switches no further: X at the profile's 156 cm, Y at the stub, each counted from its minus switch
7.5 mm below zero. Neither plus switch is measured yet (`mc601 check` stage 7) *(sim assumption)*.

A driver must do the same after homing, before the bevel head cuts: a head left in V-groove
mode by an earlier job would cut at V-groove depth. Here `machine::vgroove_reset` holds GMC's
steps (the profile's `vgroove_switch`), the runner sends them after every homing and then
parks, and `mc601 check` stage 3 sends them with the operator watching.

### 4.2 Head selection and the side-by-side X offset

The two heads (bevel and straight) sit side by side on the carriage and are selected with the
output bitmask (§2.5): `45` bevel down, `4A` straight down.

**Fixed X offset between the heads:** [GCF] `Heads[*].ToolHeadCorrection.StartPX/StartPY` (cm):

| Head | `StartPX` | `StartPY` |
|------|-----------|-----------|
| Bevelled | **−7.15 cm** | 0.19 cm |
| Straight | **+0.62 cm** | 0.17 cm |

→ **X head spacing = 7.77 cm ≈ 77.7 mm**, Y about the same for both. GMC subtracts
`StartPX/StartPY/StartPZ` from every target before converting to steps [GMC]. Cross-check: bevel window X-centre 7699 − straight border X-centre 4642 = 3057 steps =
**76.4 mm** [CAP g_regular_mount], which agrees within the overcut.

---

## 5. Pacing: the `@X` loop and buffering

The controller is **not** a one-segment executor. Mode 2 has an input buffer with look-ahead
speed control [SPEC1 p.6–8, p.32]. GMC's serial path uses it in two ways [GMC]:

| Move | What GMC sends | Captured |
|------|----------------|----------|
| straight edge, travel | one `L20,…`, then polls `@X` | [CAP g_regular_mount] |
| curve (the oval) | **all its vectors in one write**, then one poll loop: 1,107 `L20` lines, 24,253 bytes, no polls in between | [CAP g_oval_mount] |

**`@X` after a vector** [CAP g_regular_mount]: `@X15` (moving, homed, **waiting out the `#C1`
dead time**) → `@X14` (moving) → `@X04` (stopped).

```
send L20,…            (one vector, or a whole contour)
sleep 50 ms
loop:
    send "@X⏎"; sleep 10 ms (50 ms after a multi-vector write); read
    if the reply contains "@X0": done            # stopped (GMC tests "@X0")
    if it contains 0x07: the start button was released: stop the job
```
[GMC]. GMC gives up after 30,000 polls and
declares the link lost after 100 empty reads. Measured: up to **62 `@X` per second**
[CAP g_regular_mount].

**GMC's command round trip.** For every mode-0 or mode-1 command GMC sends, sleeps 5 ms, then
reads in a loop with 10 ms sleeps until a CR arrives (giving up after 200 empty reads)
[GMC]. Each command therefore costs at least
about 15 ms, however fast the controller answers. On the serial path GMC sends every `@M`
twice [GMC]. That is why every `@M` in the captures is doubled.

**Where GMC's pauses come from.** Two causes, both visible in the captures.

1. **Between cuts, choreography.** Besides the polls, each window edge costs **24 commands**,
   14 of them `@M` (7 mode changes, each sent twice), and two poll loops
   [CAP g_regular_mount]. All of it happens with the carriage stopped: GMC waits for `@X0`
   before it sends anything. At 15 ms or more per command, that is 0.3–0.5 s of standing still
   per cut, on top of the dead time and the head's settle time.
2. **On curves, the line cannot keep up.** At 855 mm/s (95 %) the controller consumes path data
   faster than 115200 baud delivers it [CAP g_oval_mount: 928 mm of path, 1.9 laps]:

   | Encoding of the oval | Vectors | Bytes | Bytes/s needed | Share of the line |
   |----------------------|---------|-------|----------------|-------------------|
   | GMC: `L20,X,Y,Z` for every vector | 1,107 | 24,250 | 22,341 | **194 %**: it starves |
   | Same points, compact relative (§2.4) | 1,107 | 12,647 | 11,651 | 101 %: borderline |
   | Within 0.02 mm of GMC's points, compact | 316 | 4,175 | 3,846 | **33 %** |
   | Within 0.05 mm of GMC's points, compact | 174 | 2,457 | 2,264 | 20 % |
   | Within 0.02 mm and ≤ 2° of turn per point, compact (the driver's rule, §3.2) | 425 | 5,419 | 4,992 | **43 %** |

   When the buffer stays under 10 %, `#C14` bit 8 slows the carriage, and when it runs empty the
   controller brakes to a stop (§2.4.1). That is GMC's hesitation on contours. It is also why
   GMC never overflowed the buffer without flow control: on its curves the controller drains the
   buffer faster than the line fills it. At a low feed the reverse holds, and a long curve could
   overflow a buffer of unknown size.

**What the driver does instead** (the machine layer's design: `crates/machine/src/lib.rs`):
- A cut is **one mode-2 stream**: travel with `C1` (no dead time), `A` head down, `W` settle,
  the path, `A` head up. No mode switches, no polls in between.
- Curves are flattened to a **chord tolerance** and a 2° turn limit, not a fixed spacing
  (PROGRAM_FORMAT §4.2), and sent in the **compact** form, so the bytes per second stay well
  inside the line (the table above).
- Travel and cutting use **two speed entries** set once per job (20 and 21), so a stream needs
  no `G`.
- The next cut is streamed **just in time**, when `@N` shows the current one under way, so the
  machine stops only where it physically must.
- A faulted cut is never retried; `*022` fails the cut.

**Flow control.** GMC never enables `#C7` and relies on its contours fitting. The driver sends
`#C7,1` and honours `0x03`/`0x02`: while held, it stops sending path data but still sends master
commands. `@H` pacing is a fallback (`Config::stream_window_bytes`, off by default until stage 6
measures the buffer). The buffer's size is unknown *(unverified)*. A USB-serial adapter holds
some bytes of its own, which still go out after `0x03`; stage 6 checks that nothing overruns at
full speed.

**Host watchdog.** `#C19,<ms>` makes the controller stop if the host goes quiet while axes move
[SPEC1 p.26]: "connection lost → machine stops" for free. The driver keeps it fed with `@H`
while a stream is held.

**The simulator runs at the line's speed.** `sim::spawn` uses a link paced at 11,520 bytes/s
(`LINE_BYTES_PER_SEC`), so starvation and flow control behave as on the machine. The test
`on_a_real_line_0x03_alone_keeps_the_buffer_from_overflowing` shows `0x03` suffices when the link
adds no buffering of its own.

### 5.1 What the spec recommends, and what the driver does

Every piece of advice in SPEC1 ("sollte", "muß", "Empfehlung", …) and where the driver stands.
"Machine layer" means policy above the controller crate: designed in `crates/machine`, not yet
written.

| SPEC1 says | Page | Driver |
|------------|------|--------|
| Use a FIFO UART on the PC for high baud rates | 5 | USB-serial adapters buffer. That delays `0x03`'s effect; stage 6 checks for overruns at full speed |
| The text protocol uses the `0x02`/`0x03` handshake, not RTS/CTS (that is for a future binary protocol) | 5 | Follows: no hardware flow control; `0x03`/`0x02` honoured with `#C7,1` |
| Master commands are always processed, even when the stream is held | 5, 12 | Relied on: `@H`, `@X`, `@B` are sent during a hold |
| Modes 0 and 1: one command, wait for its CR before the next | 6, 32 | Follows: one outstanding command; every wait has a deadline. After a timeout the session resyncs with a unique `@I100`–`@I119` probe before the next command |
| Mode 2: always send several commands ahead, or the look-ahead cannot work | 6–7 | `stream()` writes without waiting. The machine layer streams a whole cut, not vector-then-poll as GMC does |
| `#C1` dead time lets several vectors arrive before motion starts; `C1` skips it "for single moves with no path after them" | 22, 35 | `StreamFlags { no_dead_time }`. The machine layer uses it for travel instead of GMC's `@M0 #C1,0 @M2` |
| Buffer under 10 % for long → jerky motion; `#C14` bit 8 slows down to compensate (on in the 601's config) | 8, 25 | `Command::path` sends compact vectors; the machine layer flattens to a chord tolerance. GMC's fastest curve then needs under half the line (§5) |
| Use `@H` to decide whether to send more vectors or wait | 16 | `Config::stream_window_bytes`, **off by default** until stage 6 measures the buffer |
| `#C19`: the host must send a character within the time while axes move | 26 | The session sends `@H` while a stream is held (test `flow_pause_keeps_the_host_watchdog_fed`). Choose `#C19` ≥ 4 × `poll_interval` |
| `#C8`: `0x05` can serve as a sign of life; it only comes in send pauses | 23 | Decoded and published as `Event::Signal`. No "controller silent" alarm yet (machine layer) |
| `@B`: wait for the end of motion (status or `0x01`) before reading the position | 14 | `brake()` then `wait_idle()`. A stop through `StopHandle` latches until `clear_stop()` |
| `@S`, `$S`, `!S` only in an emergency (positions lost, re-home) | 14, 32, 38 | `StopHandle::emergency_stop` is for the e-stop path only; the normal stop is `@B` |
| In mode 2 the run flag is set from the first path data, even while waiting; use the wait flag | 13 | Follows: `@X15` is read as running and waiting; `wait_idle` uses the run digit |
| `#C4,2` (no screen output) when no monitor is attached: the display costs controller time | 22–23 | Not sent; the probe reads `&C4`. If the 601 boots with output on, the session setup should send `#C4,2` |
| `#C23` pulse stretch as low as possible | 26 | Not touched (and never saved: no `#W`) |
| A baud change needs a restart (`@R`) | 15, 26 | Never done: `#C18` and `@R` are never sent |
| Mode 9 is for external programs only | 38 | Never entered |

---

## 6. Feed rate (solved)

**`#G20,<v>` sets the speed of table entry 20**, which every `L20` vector uses. GMC computes it as
`AbsMaxSpeed × Speed% / 100` [GMC] = **`360 × Speed%`** steps/s
= 9 × Speed% mm/s:

| Speed % | `#G20` | mm/s | Verified |
|---------|--------|------|----------|
| 20 / 40 / 60 / 80 / 100 | 7200 / 14400 / 21600 / 28800 / 36000 | 180 / 360 / 540 / 720 / 900 | ✓ sweep |
| 95 (C10) / 35 (C11) / 10 (C12) | 34200 / 12600 / 3600 | 855 / 315 / 90 | ✓ profile captures |

`Speed%` comes straight from the `.gcf` profile. `#S` is the **start/stop speed**, not the feed
[SPEC1 p.19]. `#G10` and `#G11` are other table entries, used for homing. There is **no depth
axis**: the blade's protrusion is set by hand on the holder.

---

## 7. Overcut / thickness compensation (in the toolpath)

*(From the captures and the `.gcf`.)*

Thickness compensation shows up as **toolpath extension (overcut)** in X/Y, not as any `#`
parameter. The bevel blade leans, so its edge on the far side of the board lags its edge on
the near side, and thicker board needs the cut to run further past each corner.

| Profile | Thickness | Y start → end | Cut length (steps) |
|---------|-----------|---------------|--------------------|
| C10 | 1.5 mm | 10776 → 2616 | 8160 |
| C11 | 2.7 mm | 10824 → 2568 | 8256 |
| C12 | 4.6 mm | 10900 → 2492 | 8408 |

The cut grows by the same amount at both ends, which matches
`Heads[Bevelled].MatThicknessDependent = true` [GCF]. The blade angle Z changes too (774 → 783),
but that is the profile's blade trim (3° on C10, 4° on C11 and C12, §3.2), not the thickness:
C11 and C12 differ in thickness and both give 783.

**GMC's rule** [GMC; 16 Nov]. GMC splits each contour at corners sharper
than the break angle and, for each cut, as the 601 cuts (board face down: `CutFaceUp` false in
the shop's parameters [GCF]):

1. **The line** moves sideways by the head's `NC` ÷ 2 (the `.gcf`'s lengths are in cm): the
   bevel's line 0.1 mm into the window, the straight head's 0.25 mm to the right of travel.
   The straight head's shift is what lands GMC's layout edges at X 601–602 and Y 761–762 steps
   (§4.2): the driver's board origin, (0.225, −0.275) mm, is this shift less a step's rounding,
   not a property of the corner. The fitted head offsets absorb it.
2. **The start and the end.** Face down, a thickness-dependent head (the bevel) takes its
   **back** trims and the others (the straight) their **front** trims; the bevel's front trims
   and the straight head's back trims apply only face up.
   - Bevel: start `BackBegin + StartV`, end `BackEnd + StartV`, and the profile's `OCAddon` at
     both ends. `StartV` is where this edge and its neighbour, each moved out by the board's
     thickness, meet: `thickness × tan(turn ÷ 2)` at a corner turning by `turn`, so the
     thickness at a square corner and none at a joint under about 1.7°. From the design's
     corners that is −1.2 + t and 2.2 + t mm; from the line's own corners, 0.1 mm inside,
     −1.1 + t and 2.3 + t, the compiler's `Blade::gmc`. The table above: 48 steps (1.2 mm) more
     at each end for 1.2 mm more board, 76 (1.9 mm) for 1.9 mm.
   - Straight: start `FrontBegin` (−0.5 mm), end `FrontEnd` (2.3 mm), whatever the board.
   - At a corner turning the other way, as an L-shaped window's inner corner does, the bevel
     adds `trash_overcut` (2 mm) and twice `StartV` more, unless that corner is where the
     contour starts.
3. **No thickness offset of the line.** GMC's design is the opening on the back. Only when
   cutting face up does it move a bevel line, by `thickness × tan(90° − Inclination)`, so that
   the back opening stays the design; face down the blade already enters on the back.
4. **Closed smooth curves** (ovals) take no start or end trims and no thickness term (they have
   no corner): their overlap and run-out (§7.1) do that work.

The trims [GCF], in cm:

| Head | FrontOvercut (Begin/End) | BackOvercut (Begin/End) | trash_overcut | NC |
|------|--------------------------|-------------------------|---------------|-----|
| Bevelled | −0.18 / 0.082 (face up only) | **−0.12 / 0.22** | 0.2 | −0.02 |
| Straight | **−0.05 / 0.23** | 0 / 0 (face up only) | 0 | −0.05 |

Of the profiles, only C16 (5 mm) sets an `OCAddon`: 0.05 cm.

**The inclination is to the board, not to vertical** [GMC; GCF]. `Inclination` 42° is the
blade's angle to the board: cutting face up, GMC takes the bevel's run across the board as
`thickness × tan(90° − 42°)`, 1.11 mm per mm. Face down, as the 601 cuts, GMC never uses it.
It reads it the other way, `tan(Inclination)`, only in two modes the 601 has off (bevelled
outside cuts, an F1 default, and always cutting outside contours from the back), and computes
that twice more without using it; its description says only "the inclination of the blade in
the orthogonal direction". Calibration 1 agrees with 42° to the board: the bevel's run measured
1.71 ± 0.08 mm on 1.5 mm board, 48.7° ± 1.3° from vertical; 48° predicts 1.67 mm, 42° from
vertical 1.35 mm, 4.5 standard errors off.

**Where GMC falls short**, each to be confirmed at the machine before the driver departs from
it:

- **The thickness term takes the bevel as 45°.** At every corner GMC moves the edges out by the
  board's thickness, where its own inclination says 1.11 × the thickness. With the blade's
  edge at 37.5° to its spine (measured 13 Nov) leaning 48°, a cut's ends lag its tip by about
  `tan 37.5° ÷ cos 48° = 1.15` mm per mm of board, against GMC's 1.0. A base trim set on thin
  board then leaves thick board short, by about 0.5 mm at 5 mm. The shop's only `OCAddon` is
  0.5 mm, on C16 (5 mm), which is what that predicts; C12 (4.6 mm) has none, so this suggests,
  not proves.
- **One set of trims per head.** A profile can only add the same `OCAddon` at both ends; the
  start and end can't differ by board.
- **One sideways shift per head** (`NC`). The slit's two sides aren't modelled. With strokes in
  one direction only (the straight head's +X and +Y), the shift is indistinguishable from the
  head offset: calibration must cut both ways to tell them apart.
- **The inclination read two ways**, as above, under an ambiguous description.
- **The trash overcut depends on where the contour starts**, so the same corner can get it or
  not.
- **No soft start for the bevel at any thickness** (§3.3): every bevel cut jumps to 100 mm/s,
  or to its cutting speed, with the blade in the board, and only the straight head's start
  softens with thickness. The likeliest cause of the wobble operators see at the start of
  thick-board bevel cuts.
- **The drawn-shape gentle start reaches straight edges but not bevel windows** (§3.3),
  apparently because GMC's lookup misses a window's outline rather than by design.
- **Curves** (§7.1): the start is wherever the shape's outline begins, never chosen; the entry
  waits the head's 100 ms whatever the profile; the run-out is dropped without a word when a
  curve's points are unevenly spaced.

### 7.1 Closed smooth shapes: the overlap lap

GMC does not stop a closed curve where it started. It keeps cutting round, over the start again.

**Which shapes.** A shape gets the overlap [GMC] if it is closed and every joint, including
where it closes, turns by less than the break angle (15°). Ovals and
circles qualify; rectangles don't. A tool with a negative `CurveAddon` never gets the overlap.

**The overlap** [GMC]. After the loop, GMC
appends the loop's first points again, a retrace of the start. How far it goes is a **time**,
the head's `CurveAddon` in ms, at the cutting speed. Below `CurveMaxRadius` (30 cm) each segment
counts as longer, in proportion to how much tighter it is, modelling the controller's radius
slowdown. That 30 cm is the radius GMC gives the controller as `#C21,12000` (§2.4.1).

**The run-out.** Toward the end, the appended points are pushed sideways, linearly from 0 up to
`CurveAddonEndOffset` at the last point. The ramp covers the whole overlap, or its last half when
the overlap is more than a third of the path. Which side depends on the cut's orientation
settings. `CurveAddonStartOffset` would do the same at the start (0 on every head).

**On the captured oval** [CAP g_oval_mount], bevel head at 95 %:
- One lap is 483 mm (604 points). The overlap adds 443 mm, 0.92 of a lap, so the cut is 1.9 laps
  and 928 mm.
- By GMC's radius model those 443 mm count as 1,560 mm; 1,850 ms at 855 mm/s is 1,582 mm. The
  overlap stops at the first point past that.
- The second lap follows the first to 0.075 mm on average and ends 0.31 mm off it, inside the
  oval, in the waste: the 0.3 mm run-out.

**Why** (an operator's account, consistent with the code): the overlap re-cuts the place where
the blade went in and the carriage was still speeding up, and moves the stop far away onto line
already cut. The run-out takes the blade off the line gradually, on the waste side, so no exit
mark lands on the visible edge.

| Head | `CurveAddon` (overlap) | `CurveAddonEndOffset` (run-out) |
|------|------------------------|---------------------------------|
| Bevelled | 1,850 ms | 0.03 cm (0.3 mm) |
| InvBev | 950 ms | 0 |
| Straight | 500 ms | 0.03 cm (0.3 mm) |
| Score, Deboss, Pen1–3 | 0 | 0 |

[GCF]. The driver's compiler does the same for closed smooth cuts, starting from these values
(PROGRAM_FORMAT §4.2).

**How GMC does it** [GMC; 16 Nov], beyond the above:

- **The overlap is a time, so it shrinks with the speed**: the profile's speed % of 900 mm/s
  for `CurveAddon` ms, the radius model capping `CurveMaxRadius` at 50 cm. About 1,580 mm on
  C10 at 95 %; about 170 mm on C12 at 10 %.
- **The run-out** ramps over the overlap's last points (its last half when the overlap is
  more than a third of the path), on the side set by the head's orientation and whether the
  board is face up, and then GMC drops the last point, so the full run-out is never quite
  reached (0.31 mm off on the captured oval, for 0.3). GMC skips the run-out, logging it only
  to its debug file, when a curve's point spacing varies more than tenfold.
- **The start** is wherever the shape's outline begins: GMC sets only the direction (a bevel
  contour clockwise when cutting face down), never the start point. Its ovals start at the top
  (§4), so the entry always falls at the same place on the curve.
- **The entry**: an oval window (not an outer contour) goes down with `i_t` alone, through the
  Z head's throttle, and GMC waits only the head's `HDSleep` (100 ms on the bevel), not the
  profile's `HeadUpDown`, before moving off while the blade is still coming down. How far it
  travels during the entry depends on the throttle screw and the cutting speed; the overlap,
  being a time, cuts back over it whatever the speed.
- **No thickness term**: a smooth closed curve has no corner, so no `StartV` (§7), and face
  down no line offset; only the bevel's 0.1 mm `NC` shift.

---

## 8. Per-profile calibration table

*(From [GCF].)* Wire values are derived: feed §6, ramp §3.1, overcut §7.

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

*Thickness in cm (0.15 → 1.5 mm). **Bold** = captured and verified. `ZCorr_Str` / `ZCorr_Rnd`
= the bevel blade's trim in degrees on straight edges / closed curves (`ZCorrection_Straight` /
`ZCorrection_Round`, §3.2). `HeadUpDown` = head settle time in ms: GMC sleeps
`HDSleep`/`HUSleep` after switching the head outputs [GMC].

**Tool priority:** Straight = 1, Bevel = 2, V-Groove = 3, Score/Deboss = 4, Pens = 5
[GCF `Heads[*].Priority`].

---

## 9. Licensing

See §2.7. Do not reproduce it; the machine moves without it.

---

## 10. Open items

**Settled:** head spacing (77.7 mm, §4.2) · `@X` codes (§2.1) · `#R` = ramp time (§3.1) · `#C1`
= dead time · `$C`, `#Z`/`#N`, `#O`, `#C20`–`#C28` (§2.3) · `#C35` (§1) · baud 115200 8-N-1 (§1) ·
GMC's curve flattening (§3) · GMC's blade angle per vector and the profile trims (§3.2) · GMC's
soft start (§3.3) · GMC's overlap lap on closed curves (§7.1) · GMC's pauses (§5).

**Open**, with what answers each. The probe is read-only; check stages 3–7 move the machine (see
`docs/hardware-testing.md`).

| Question | Assumed until then | Answered by |
|----------|--------------------|-------------|
| Does the 601 accept mode switches without GMC's licence exchange (`@M1` → CR, or `*007`)? | yes | probe |
| Does it answer the resync probes `@I100`–`@I119`? | yes, `@I1xx,0` | probe |
| Does it boot with GMC's connect configuration, or must the driver write it (`C2`, `C5`, `C11`–`C14`, `C16`, `C35`, `#ZZ`, `#NZ`, `#M`)? | must write | probe ("box vs GMC at connect") |
| Boot values of the registers GMC never writes: `C4`, `C6`, `C7`, `C8`, `C19`, `&D` homing directions | — | probe |
| Do the v1.00 queries exist on firmware 2.00 (`@N`, `@O`, `@Y`, `@Q242`–`@Q249`)? | yes | probe |
| Which clamp each of bits `20` and `40` closes | `40` left, `20` right, from GMC's clamp logic (§2.5) | stage 3 (operator answers) |
| Do outputs survive a mode switch? | yes (captures) | stage 3 (`@Q249` across `@M`) |
| Does `@B` in mode 2 discard the rest of the path, or resume it? | discard | stages 4 and 6 |
| Compact follow-on vectors on 2.00; after `G`, `A`, `W`? | accepted | stage 5 |
| Does a second speed entry give its own feed? | yes | stage 5 |
| In-stream `G`: in sequence or on receipt? | in sequence (stage 5, 2026-11-10); the sim still applies it on receipt | stage 5 |
| `@N`: does `C` count? the running command or the finished ones? width; wrap after 9999? | `C` counts; the running command; no wrap (10001 after 9998) (stage 5, 2026-11-10), and the sim counts it too | stage 5 |
| Which of `0x02`/`0x03` means "full"? | **`0x03`**, as the byte table says: filling a held stream, `0x03` came at 72,340 bytes (`@H98`, about 1 KB before full, not the prose's ~200 bytes) and nothing before it (stage 8, 2026-11-10). The simulator still warns 200 bytes before full | stage 8 |
| Buffer size | **about 73 KB**: behind a held `I221`, `@H` reads 0 until about 8 KB are in, then rises 1 % per ~650 bytes (1 % at 8.6 KB, 50 % at 40.6 KB, 95 % at 70 KB), so it counts the last ~65 KB (stage 8, 2026-11-10; stage 6's 2200 bytes read `@H0` for this reason). The simulator models both parts | stage 8 |
| Does `0x03` come in time through a USB adapter (no overrun at full speed)? | yes | stage 6 |
| After `@B` on a held stream: is the queue gone, and does `0x02` follow? | **yes, both**: 72 KB held at `I221` after `0x03`, `@B`, `0x02` 36 ms later, then `@H0`, `@X04`, nothing moved (stage 8, 2026-11-10) | stage 8 |
| What `*022` means | buffer overrun | stage 6 |
| `#C19` with a held stream | fed by `@H`: no fault in 2 s of `@H` every 100 ms with `#C19,400` (stage 8, 2026-11-10) | stage 8 |
| `@S` loses the position | yes (spec) | stage 7 |
| Limit fault: async `*03x`, `@Y` bit, recovery | spec | stage 7 (only if a limit input is mapped) |
| Does a master command ever overtake the reply to the command before it? Settling a `*NNN` (§2.6) and every resync rely on it not doing so | no: replies come in order | stage 2: a command and a probe in one write, 20 times each for a mode-0 write, `@M0` and `&C52` (refused like GMC's `#C52,800` [CAP g_init]); `ControllerError::Overtaken` if the probe is answered first |
| No `0x01` after mode-2 paths? | none: not one in 18 mode-2 runs with `#C5,1`; all 19 came after mode-1 moves (session logs, 2026-11-10) | any session log |
| A Z-only vector inside a stream (the driver's corner pivot): does the carriage stop, and how fast does the blade turn? | **taken, on the spot**: X arrived, the blade turned a quarter (750 steps) with X and Y still, then Y moved off. Z runs at the entry's speed in steps/s: a quarter turn in 651 ms at 1200 steps/s, 216 ms at 4000 (from the send, ramp included) (stage 8, 2026-11-10, heads up). Whether a pivot in the board cuts clean is for test cuts; the machine model still splits a cut at its corners | stage 8 |
| Do the 2° turn limit, the tangent rule and GMC's overlap and run-out values cut clean? | GMC's values | test cuts |
| Does `i_b` bypass the Z head's throttle (§2.5: `01` is the throttled, delayed blade entry for curves)? | **yes**: blades out, `$01` brought the bevel head down slowly and `$05` fast; the lift `$00` after `$05` raised the blade with the head still down; `$0A` lowered the straight head alone (stage 8, 2026-11-10, operator watching) | stage 8 |
| The right trim per material on this machine and blade (it corrects hooked corners, §3.2) | GMC's per-profile trims (§8) | GMC's procedure: a test rectangle per profile, the corners checked [HELP CA 3800] |
| The wobble at the start of thick-board bevel cuts: the jump to `#S`, the acceleration, or the trim? | — | thick-board test cuts: the driver's soft start (`#S400` on 4.6 mm) against GMC's (`mc601 run --gmc-bevel-start`, `#S4000`), then if needed a slow lead-in, trim 0 against the profile's |
| An oval's entry: does moving off 100 ms after the slow head-down, as GMC does, leave a cleaner join than letting the blade settle? | — | an oval window cut both ways, `mc601 run` and `mc601 run --oval-settle` |
| Overcut mapping (§7) | — | test cuts, one trim at a time |
| Radius slowdown variant (p.9 or [RADIUS]) | — | not needed |
| `@@`, `@V7`/`@V8`, `#C37`, `#C52`, `*069` | — | not needed to drive the machine |

---

## 11. Driver architecture

A Rust workspace in `crates/`, layered so each layer can be tested without the ones above it.

| Crate | Role |
|-------|------|
| `protocol` | Pure, no I/O: encodes commands, decodes replies and signal bytes, error texts, typed settings (`Setting`, `OverridePct`, `StreamFlags`, `Command::path`). |
| `link` | Bytes to the controller: a serial port, an in-memory pair (optionally paced at a line rate), and a tap that logs every byte with a timestamp. Reader and writer are separate halves. |
| `controller` | A session with the controller. `HostCore` is a pure state machine (bytes in, outputs and deadlines out) that decides what is a reply (a `*NNN` while a command waits is settled by a probe, §2.6), tracks sync, mode, flow control, unreported faults and the stop latch. A driver runs it with a reader thread and two locks, writer before state. `Controller<M>` only offers the commands mode `M` accepts; `StopHandle` brakes from any thread. |
| `sim` | `SimCore` speaks the whole protocol GMC uses; `spawn` runs it behind a link paced at the line rate, with fault injection. Every GMC capture and every session log in `testdata/sessions/` is replayed against it (`crates/sim/tests/fidelity.rs`). |
| `program` | The program format (`PROGRAM_FORMAT.md`): types, parsing, validation. |
| `machine` | The machine model, pure: the machine profile, board frame mm/deg → steps, each cut lowered to controller commands as one unit (head down only inside a cut), soft limits, the byte budget, the run time. Written: the profile, the mapping, the lowering in GMC's form (held to the captures by golden tests, `crates/machine/tests/golden.rs`) and in one stream per cut (each part a capability, off until `mc601 check` shows it), and the run-time and line-budget estimate. |
| `runner` | Drives the cutter on a controller session: checks and writes the controller configuration the machine profile assumes, homes as GMC does, runs programs (progress per op; `pause` ops on the green button or the operator; pause and cancel between cuts; stop now from any thread) and manual operations (park, clamps), and after any failure brakes, resynchronises, clears the error and restores the safe state (heads up, clamps released), then requires homing. Tested against the simulator over a real session (`crates/runner/tests/runner.rs`). |
| `route`, `nest` | The cut order, and the nesting of mats onto sheets. |
| `mc601` | The command line: `ports`, `capture`, `probe`, `check`. |

What the session layer guarantees, and why:
- **One reader** splits the incoming bytes into CR-terminated replies and the signal bytes
  `0x01`–`0x09`, which can arrive inside a reply [CAP g_init, g_motions; SPEC1 p.7]. Over 4096
  bytes without a CR is reported as `Event::Garbled` (a wrong baud rate, for example).
- **One outstanding command**, matched with its own reply, with a deadline. A `*NNN` that cannot
  be the answer is an event (§2.6).
- **Resync.** After a timeout, the next operation first reads an input nothing else uses
  (`@I100`–`@I119`, a new one each time) and discards everything before its answer. GMC has no
  such step, so one late reply leaves every later reply paired with the wrong command.
- **Mode 2 has no acknowledgements**: `stream()` writes without waiting. While flow control
  holds path data, master commands still go.
- **Stop lane.** `StopHandle` writes `@B` or `@S` past anything in progress. A stop latches:
  every operation fails until `clear_stop()`.
- **Never sent:** `@M9`, `#W`, `@R`, `@D`, `#C18`.

---

## 12. CAM pipeline summary

```
SVG/DXF in  →  layers → tools (bevel / straight / score, by the cut-type convention)
            →  offset for bevel and kerf; overcut per §7
            →  nest (crates/nest: three-stage guillotine, after PackingSolver's tree search)
            →  sequence: interiors before the boundary, then the order and each cut's way round
               that travel least (crates/route)
            →  flatten curves by chord tolerance (0.02 mm, turn ≤ 2° per point; PROGRAM_FORMAT §4.2)
            →  closed smooth cuts: overlap past the start, run-out toward the waste (§7.1)
            →  blade angle: the curve's tangent at smooth points; stop and turn at corners (§3.2)
            →  check the byte budget: bytes/s at the feed must stay well inside 11,520 (§5)
            →  per cut, one mode-2 stream: C1 + travel at entry 21, A head down, W settle,
               the path at entry 20 (#G20 = 360 × Speed%), A head up; #R per §3.1
            →  the driver streams with #C7 flow control, and lets each cut end before it
               sends the next (so a cut is reported done when it is; following @N could
               send the next while this one runs)
```

The machine controls **XY position and blade rotation** only. Depth is set by hand.

---

## 13. Changelog

### 2026-11-06: start of a cut and the blade trim

| Was | Now | Source |
|-----|-----|--------|
| Z shifts 774 → 783 with thickness | the profile's blade trim: 3° on C10, 4° on C11 and C12 | GCF `ZCorrection_Straight`; CAP g_multi_C10–C12 |
| `#R470` "thickness-dependent"; `#R700` from slow-ramp tools | GMC's thickness soft start, straight head only: `#R` and `#S` scaled with thickness; bevel edges get none (§3.3) | GMC; GCF `DynamicMatSetting`; CAP |
| `#R253` unexplained | diagonal ramp smoothing (§3.3) | GMC |

### 2026-11-06: blade angle and closed curves

| Was | Now | Source |
|-----|-----|--------|
| Blade angle "follows each segment's direction"; formula open | Z = trunc(8.3333 × (heading − StartPZ + trim)); each vector carries the heading of the chord it cuts, and the blade turns during the chord (§3.2) | GMC; SPEC1 p.33; CAP |
| GMC keeps Z in 0..2999 | only between cuts; along a cut Z keeps counting (oval 5 → 5521) | CAP g_oval_mount |
| The oval: one curve of 1,107 vectors | one lap plus GMC's overlap: 1.9 laps, the last 0.3 mm drifting into the waste (§7.1) | GMC; GCF; CAP g_oval_mount |
| Bevel head down = `45` | `45` on straight edges, `41` on the oval | CAP g_oval_mount |

### 2026-11-06: rewrite

| Was | Now | Source |
|-----|-----|--------|
| Citations to the v0.99 draft's pages | SPEC1 pages throughout; v0.99 only where it differs | SPEC1 |
| Error codes at "p.37–38"; limits as one block | p.38–40; 032–039 alternate minus/plus per axis; full list grouped into rejections, events and either | SPEC1 p.38–40 |
| `#C` table without C3, C4, C9, C15, C23, C29, C30, and without when GMC writes each | complete, with "connect / use / never" | SPEC1 p.18–28; CAP g_init |
| `#C35`/`#C37` "not in spec", sent to every model | `#C35` is in SPEC1 p.28; both are model-gated in GMC; `#C52` is sent to every model | GMC |
| `0x01` after every move | after mode-1 moves, homing and `@B` only; never after a mode-2 path | CAP all |
| `$H` replies CR at once | only with `#C6,1`; otherwise the CR comes at the end | SPEC1 p.23 |
| `#R` = "maximum ramp time" | the time to reach `#M` from rest; GMC's accelerations are in mm/s² | SPEC1 p.8, p.19 |
| Z scale as 8.333 steps/° only | 3000 steps per revolution; GMC keeps Z in 0..2999 | GMC; CAP g_regular_mount |
| "Curve chord tolerance ≈ 0.5 mm (≈ 24 steps)" | a chord length chosen by radius band (`Distances`); the oval's vectors are 0.28–1.27 mm apart; the driver flattens to a chord tolerance instead | GCF; GMC; CAP g_oval_mount |
| Head up = `$A42` | `$A40` after a bevel edge, `$A42` after a straight edge | CAP g_regular_mount |
| `$L10,X3` after homing | before homing; after homing comes travel in mode 2 and the park | CAP g_init |
| Doubled `@M` unexplained | GMC sends each `@M` twice on the serial path | GMC |
| GMC's pauses unexplained | choreography between cuts; line starvation on curves | §5 |
| Mode-2 follow-on vector noted, not used | compact paths are the driver's encoding; GMC's USB path uses the follow-on form | SPEC1 p.33; GMC |
| Path speed control mentioned in passing | braking rules, `#C14` bits and the radius slowdown (§2.4.1) | SPEC1 p.8–9, p.22–27; RADIUS |
| Modes 3, 4, 9 as one line each | what each does and why the driver does not use it | SPEC1 p.36–38 |
| Driver notes: Go, HTTP + WebSocket | the Rust workspace as built (§11) | — |
| Open items as a list | each mapped to the probe or a check stage (§10) | — |

### 2026-11-05: corrections from spec and code

| Was (capture inference) | Now | Source |
|-------------------------|-----|--------|
| `#R<angle>` rotates the blade | `#R<ms>` = ramp time; GMC computes `min(700, 900000/accel)` | SPEC p.18; GMC |
| `L20` = a cut-move command; `#` family holds cut moves | `L<g>` = mode-2 vector in steps at speed entry `g` (GMC uses 20); no prefix | SPEC p.31–32; GMC |
| `#G20` = global feed rate | speed of table entry 20 (`#G<g>,v`) | SPEC p.18 |
| `#C1,100/0` = blade engage/disengage | mode-2 dead time (ms) before the path starts; 100 for edges, 200 for curves, 0 for travel | SPEC p.20; GMC |
| `@B` = "begin" | brake (stop with ramp); the controller then sends `0x01` | SPEC p.13; CAP g_motions |
| `$CX/$CY/$CZ` = axis config | zero the counters | SPEC p.30 |
| `$L10/$L20` = "modes" | mode-1 jog in steps at speed entry 10 / 20 | SPEC p.30 |
| `#S` = global max/approach speed | start/stop speed | SPEC p.18; GMC |
| `#B20,90` = bevel parameter | braking angle for speed entry 20 | SPEC p.18 |
| `@X` codes `04` ready / `18` busy only | `r` + flags; `14`/`15` dominate during cuts (9,719 / 344 replies) | SPEC p.12; CAP |
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

| SPEC1 says | Captures show | Driver and sim follow |
|------------|---------------|-----------------------|
| A mode switch clears the outputs [SPEC1 p.32, p.34, p.38] | `$A` outputs persist across `@M` changes [CAP g_motions, mounts] | the captures (outputs persist) |
| `#R` range 10..400 ms [SPEC1 p.19] | `#R470`, `#R700` acknowledged [CAP g_multi_C11, g_multi_C12] | the captures (no 400 cap) |
| `#C20` ≥ 1000, `#C21` ≥ 100000 [SPEC1 p.26] | `#C20,400`, `#C21,12000` and `#C21,5000` acknowledged [CAP g_init, g_regular_mount]. The spec's own query examples show `&C20,400` and `&C21,6000` [SPEC1 p.29] | the captures (no minimum) |
| `0x01` whenever an axis comes to rest after a move [SPEC1 p.23] | none after mode-2 paths, with `#C5,1` set [CAP all cut captures] | the captures: after a stream the driver polls `@X` instead of waiting for `0x01` |
| `@X` flag table: bit 4 set = homing needed [SPEC1 p.13] | set in every state after homing, absent while homing [CAP all]; the spec's own example agrees | the captures (4 = home position known) |
