//! The controller model: pure, explicit time, text in / bytes out.

use std::collections::{BTreeMap, VecDeque};
use std::time::Duration;

/// Simulated time since the core was created.
pub type SimTime = Duration;

/// How long a simulated press of the green button holds I221 high.
pub const BUTTON_HOLD: Duration = Duration::from_millis(200);

const ENDLESS_JOG: i64 = 999_999;

const X: usize = 0;
const Y: usize = 1;
const Z: usize = 2;

/// `#C` registers the 601 firmware accepts: the spec's list [SPEC p.17] plus 35/37 seen
/// accepted in [CAP g_init]. Anything else is `*021`, as the real controller answers `#C52`.
const KNOWN_C: &[u16] = &[
    1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 18, 19, 20, 21, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 37,
];

#[derive(Debug, Clone)]
pub struct SimConfig {
    /// Reply to `@V` [CAP g_init].
    pub version: String,
    pub homed_at_start: bool,
    pub start_position: [i64; 3],
    /// X and Y travel limits in steps (Z is rotary and unlimited). Placeholder until the
    /// diagnostic tool measures the envelope; covers every coordinate in the captures
    /// (X up to 58860 = park, Y up to 41800 in g_init).
    pub envelope: [(i64, i64); 2],
    /// Simulated duration of homing one axis.
    pub homing_time_per_axis: Duration,
    /// Mode-2 input buffer in bytes (unknown on the real controller).
    pub buffer_capacity: usize,
    /// Speed (steps/s) for speed-table entries never set with `#G`.
    pub default_speed: u32,
    /// Motions and dead times complete instantly (used for protocol replay).
    pub instant_motion: bool,
    /// `#C6` default: 0x04 after homing. The real controller sent it without GMC enabling it.
    pub homing_done_signal: bool,
    /// The start counter has reached 0: mode changes are refused [SPEC1 p.10]. The spec does
    /// not name the error; `*007` ("service code required") is inferred.
    pub start_counter_exhausted: bool,
    /// Steps per mm on X/Y, for the µm-based commands (`@P`, `$E`, `V`). [GCF ScaleX/Y]
    pub steps_per_mm: f64,
    /// Input definitions present at start (`&I<n>` → `port,bit,invert`). Empty by default,
    /// like the 601 as far as the captures show (`&I240,000h,0,0` [CAP g_init]). Limit errors
    /// come from the envelope either way.
    pub input_defs: Vec<(u8, String)>,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            version: "@V2.00/55W 19.10.10 2807001".to_owned(),
            homed_at_start: false,
            start_position: [0, 0, 0],
            envelope: [(0, 60_000), (0, 43_000)],
            homing_time_per_axis: Duration::from_secs(3),
            buffer_capacity: 4096,
            default_speed: 10_000,
            instant_motion: false,
            homing_done_signal: true,
            steps_per_mm: 40.0,
            start_counter_exhausted: false,
            input_defs: Vec::new(),
        }
    }
}

/// Observable state, for tests and dry-run tooling.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Snapshot {
    pub mode: u8,
    pub homed: bool,
    pub moving: bool,
    pub position: [i64; 3],
    pub outputs: u8,
    pub error: u16,
    pub queued_bytes: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Jog,
    Path,
    Dwell,
}

#[derive(Debug, Clone)]
struct Motion {
    from: [i64; 3],
    to: [i64; 3],
    start: SimTime,
    end: SimTime,
    kind: Kind,
    /// Set when the target was beyond the envelope: raised when the limit is reached.
    limit_error: Option<u16>,
}

#[derive(Debug, Clone)]
enum Item {
    Vector {
        target: [i64; 3],
        speed: u8,
    },
    Outputs(u8),
    Dwell(Duration),
    /// `I<n>` / `I-<n>`: wait for an input level [SPEC1 p.35].
    WaitInput {
        input: u8,
        high: bool,
    },
    /// `C8` / `C16`: speed override on / off from here [SPEC1 p.35].
    Override(bool),
}

#[derive(Debug, Clone)]
struct Queued {
    item: Item,
    bytes: usize,
}

#[derive(Debug, Clone)]
struct Homing {
    axes: Vec<usize>,
    index: usize,
    axis_end: SimTime,
}

#[derive(Debug)]
pub struct SimCore {
    cfg: SimConfig,
    mode: u8,
    homed: bool,
    homing: Option<Homing>,
    pos: [i64; 3],
    motion: Option<Motion>,
    queue: VecDeque<Queued>,
    queued_bytes: usize,
    path_target: [i64; 3],
    path_speed: Option<u8>,
    dead_until: Option<SimTime>,
    outputs: u8,
    inputs: BTreeMap<u8, bool>,
    input_defs: BTreeMap<u8, String>,
    speeds: BTreeMap<u8, u32>,
    brake_angles: BTreeMap<u8, u32>,
    ramp_ms: u32,
    start_speed: u32,
    max_speed: u32,
    c: BTreeMap<u16, i64>,
    clock: String,
    offsets: [i64; 3],
    axis_params: BTreeMap<String, String>,
    counters: BTreeMap<u8, i64>,
    error: u16,
    override_pct: u32,
    command_counter: i64,
    last_host: SimTime,
    /// When the machine last became free (end of the previous motion), so queued motions
    /// chain back-to-back however coarse the ticks are.
    free_at: SimTime,
    /// The last path vector: when it ends, its XY direction, its speed entry. The next one
    /// carries on without a new ramp if it starts then, turning no more than the entry's
    /// braking angle: the controller's look-ahead [SPEC1 p.8].
    last_path: Option<(SimTime, [f64; 2], u8)>,
    flow_paused: bool,
    /// The green button is held down until this time (a press lasts [`BUTTON_HOLD`]).
    button_release_at: Option<SimTime>,
    /// Limit-switch error code while the machine stands on that switch (for `@Y`).
    limit_hit: Option<u16>,
    /// `C1`: start the next path without the `#C1` dead time.
    next_no_dead_time: bool,
    /// `C8`/`C16`: whether the `@O` override applies to motions started from here.
    override_active: bool,
    /// When the controller last sent anything (for the `#C8` heartbeat).
    last_tx: SimTime,
    /// `#C6,0`: the CR reply to `$H` is held back until homing has finished.
    pending_home_ack: bool,
    received: Vec<String>,
}

// ------------------------------------------------------------------ small helpers

fn push_line(out: &mut Vec<u8>, s: &str) {
    out.extend_from_slice(s.as_bytes());
    out.push(b'\r');
}

fn push_ack(out: &mut Vec<u8>) {
    out.push(b'\r');
}

fn push_err(out: &mut Vec<u8>, code: u16) {
    push_line(out, &format!("*{code:03}"));
}

fn axis_index(c: char) -> Option<(usize, bool)> {
    match c {
        'X' => Some((X, true)),
        'Y' => Some((Y, true)),
        'Z' => Some((Z, true)),
        'x' => Some((X, false)),
        'y' => Some((Y, false)),
        'z' => Some((Z, false)),
        _ => None,
    }
}

/// `x2300,Y0` → [(axis, absolute, value)].
fn parse_axis_values(s: &str) -> Option<Vec<(usize, bool, i64)>> {
    s.split(',')
        .filter(|p| !p.is_empty())
        .map(|p| {
            let mut chars = p.chars();
            let (axis, abs) = axis_index(chars.next()?)?;
            Some((axis, abs, chars.as_str().parse().ok()?))
        })
        .collect()
}

/// `20,X1,Y2` → (20, "X1,Y2").
fn split_entry(s: &str) -> Option<(u8, &str)> {
    let (g, rest) = s.split_once(',')?;
    Some((g.parse().ok()?, rest))
}

fn parse_hex_u8(s: &str) -> Option<u8> {
    u8::from_str_radix(s, 16).ok()
}

impl SimCore {
    pub fn new(cfg: SimConfig) -> Self {
        let mut c = BTreeMap::new();
        c.insert(6, i64::from(cfg.homing_done_signal));
        let input_defs = cfg.input_defs.iter().cloned().collect();
        Self {
            homed: cfg.homed_at_start,
            pos: cfg.start_position,
            path_target: cfg.start_position,
            cfg,
            mode: 0,
            homing: None,
            motion: None,
            queue: VecDeque::new(),
            queued_bytes: 0,
            path_speed: None,
            dead_until: None,
            outputs: 0,
            inputs: BTreeMap::new(),
            input_defs,
            speeds: BTreeMap::new(),
            brake_angles: BTreeMap::new(),
            ramp_ms: 0,
            start_speed: 0,
            max_speed: 0,
            c,
            clock: "1,1,2026,0,0,0".to_owned(),
            offsets: [0; 3],
            axis_params: BTreeMap::new(),
            counters: BTreeMap::new(),
            error: 0,
            override_pct: 100,
            command_counter: 0,
            last_host: Duration::ZERO,
            free_at: Duration::ZERO,
            last_path: None,
            flow_paused: false,
            button_release_at: None,
            limit_hit: None,
            next_no_dead_time: false,
            override_active: true,
            last_tx: Duration::ZERO,
            pending_home_ack: false,
            received: Vec::new(),
        }
    }

    fn c_on(&self, reg: u16) -> bool {
        self.c.get(&reg).copied().unwrap_or(0) != 0
    }

    fn c_val(&self, reg: u16) -> i64 {
        self.c.get(&reg).copied().unwrap_or(0)
    }

    // ------------------------------------------------------------------ observation

    /// Every line the host has sent, in order.
    pub fn received(&self) -> &[String] {
        &self.received
    }

    pub fn is_moving(&self) -> bool {
        self.homing.is_some() || self.motion.is_some() || !self.queue.is_empty()
    }

    pub fn position_at(&self, now: SimTime) -> [i64; 3] {
        let Some(m) = &self.motion else {
            return self.pos;
        };
        let span = m.end.saturating_sub(m.start).as_secs_f64();
        let frac = if span <= 0.0 {
            1.0
        } else {
            (now.saturating_sub(m.start).as_secs_f64() / span).min(1.0)
        };
        let mut p = [0; 3];
        for (a, slot) in p.iter_mut().enumerate() {
            let delta = (m.to[a] - m.from[a]) as f64;
            *slot = m.from[a] + (delta * frac).round() as i64;
        }
        p
    }

    pub fn snapshot(&self, now: SimTime) -> Snapshot {
        Snapshot {
            mode: self.mode,
            homed: self.homed,
            moving: self.is_moving(),
            position: self.position_at(now),
            outputs: self.outputs,
            error: self.error,
            queued_bytes: self.queued_bytes,
        }
    }

    /// Is the head of the queue an `I` command whose input is not yet at the wanted level?
    fn blocked_on_input(&self) -> bool {
        matches!(self.queue.front(), Some(Queued { item: Item::WaitInput { input, high }, .. })
            if self.input_defined(*input) && self.input(*input) != *high)
    }

    /// Spec: "if the input is not defined at all, execution continues immediately".
    fn input_defined(&self, input: u8) -> bool {
        // I221 and up are fixed hardware inputs (signal, reference, stop) [SPEC1 p.7].
        input >= 221 || self.inputs.contains_key(&input) || self.input_defs.contains_key(&input)
    }

    fn input(&self, input: u8) -> bool {
        self.inputs.get(&input).copied().unwrap_or(false)
    }

    fn status_line(&self, now: SimTime) -> String {
        let moving = self.is_moving();
        // Flag 1: "waiting for an input or a wait time is running" [SPEC1 p.13]: the #C1
        // dead time, a W dwell, or an I wait.
        let in_dead_time = self.motion.is_none()
            && self.homing.is_none()
            && !self.queue.is_empty()
            && self.dead_until.is_none_or(|t| now < t);
        let dwelling = self.motion.as_ref().is_some_and(|m| m.kind == Kind::Dwell);
        let waiting =
            in_dead_time || dwelling || (self.motion.is_none() && self.blocked_on_input());
        let flags = u8::from(waiting)
            | (u8::from(self.error != 0) << 1)
            | (u8::from(self.homed) << 2)
            | (u8::from(self.homing.is_some()) << 3);
        format!("@X{}{:X}", u8::from(moving), flags)
    }

    // ------------------------------------------------------------------ fault injection

    /// Raise a machine error now (e.g. 33 = limit switch, 40 = stop input): motion halts and
    /// `*NNN` is sent, as the controller does on a fault during motion [SPEC p.6].
    pub fn inject_error(&mut self, code: u16, now: SimTime) -> Vec<u8> {
        let mut out = Vec::new();
        self.halt(now);
        self.error = code;
        push_err(&mut out, code);
        out
    }

    /// Press the green start button (signal input 1, I221) at `now`; it is released
    /// [`BUTTON_HOLD`] later by [`tick`](Self::tick). With `#C35` bit 1 the controller reports
    /// the level changes as 0x06 (high) and then 0x07 (low) [SPEC1 p.7, p.28]; GMC enables this
    /// with `#C35,1` and reacts to the 0x07. While held, `@I221` reads 1 and `I221` waits pass.
    pub fn press_start_button(&mut self, now: SimTime) -> Vec<u8> {
        self.button_release_at = Some(now + BUTTON_HOLD);
        self.set_input(221, true)
    }

    /// Set an input level. Level changes on the two signal inputs I221/I222 produce the
    /// `#C35` bytes when enabled.
    pub fn set_input(&mut self, input: u8, value: bool) -> Vec<u8> {
        let changed = self.input(input) != value;
        self.inputs.insert(input, value);
        let acks = self.c_val(35);
        match (changed, input) {
            (true, 221) if acks & 1 != 0 => vec![if value { 0x06 } else { 0x07 }],
            (true, 222) if acks & 2 != 0 => vec![if value { 0x08 } else { 0x09 }],
            _ => Vec::new(),
        }
    }

    // ------------------------------------------------------------------ host input

    /// Process one line from the host (without CR). Returns bytes to send back.
    pub fn handle_line(&mut self, line: &str, now: SimTime) -> Vec<u8> {
        self.received.push(line.to_owned());
        self.last_host = now;
        let mut out = self.tick(now);

        if self.mode == 9 {
            // File transfer (licence, firmware) is not emulated: only leaving mode 9 is.
            if let Some(n) = line.strip_prefix("@M") {
                self.set_mode(n, now, &mut out);
            }
            return out;
        }
        if line.starts_with('@') {
            self.master(line, now, &mut out);
        } else {
            match self.mode {
                0 => self.mode0(line, &mut out),
                1 => self.mode1(line, now, &mut out),
                2 => self.mode2(line, &mut out),
                _ => push_err(&mut out, 1),
            }
        }
        if !out.is_empty() {
            self.last_tx = now;
        }
        out
    }

    /// `@Y` bits `sbff rrrr uuzz yyxx` [SPEC1 p.13].
    fn switch_bits(&self) -> u16 {
        let mut bits: u16 = 1 << 14; // machine ready
        if self.input(254) {
            bits |= 1 << 15; // stop input (ISTOP)
        }
        for (i, input) in (235..=238).enumerate() {
            if self.input(input) {
                bits |= 1 << (8 + i); // reference switches IREF1..4
            }
        }
        if let Some(bit) = self.limit_hit.and_then(protocol_limit_bit) {
            bits |= 1 << bit;
        }
        bits
    }

    // ------------------------------------------------------------------ master commands

    fn master(&mut self, line: &str, now: SimTime, out: &mut Vec<u8>) {
        let body = &line[1..];
        let mut chars = body.chars();
        let Some(key) = chars.next() else {
            return push_err(out, 29);
        };
        let arg = chars.as_str();
        match (key, arg) {
            ('V', "") => {
                let v = self.cfg.version.clone();
                push_line(out, &v);
            }
            ('V', "7") => push_line(out, "@V000000000/"), // [CAP g_init]
            ('V', "8") => push_line(out, "@V0"),          // [CAP g_init]
            ('M', "") => push_line(out, &format!("@M0{}", self.mode)),
            ('M', n) => self.set_mode(n, now, out),
            ('X', "") => {
                let s = self.status_line(now);
                push_line(out, &s);
            }
            ('Y', "") => push_line(out, &format!("@Y{:04X}", self.switch_bits())),
            ('E', "") => push_line(out, &format!("@E{}", self.error)),
            ('L', a) | ('P', a) if a.len() == 1 => {
                match axis_index(a.chars().next().unwrap_or('?')) {
                    Some((axis, true)) => {
                        let steps = self.position_at(now)[axis];
                        let v = if key == 'L' {
                            steps
                        } else {
                            (steps as f64 * 1000.0 / self.cfg.steps_per_mm) as i64
                        };
                        push_line(out, &format!("@{key}{a}{v}"));
                    }
                    _ => push_err(out, 12),
                }
            }
            ('B', "") => self.brake(now, out),
            ('S', "") => {
                self.halt(now);
                self.homed = false; // positions lost [SPEC p.13]
                push_ack(out);
            }
            ('N', "") => push_line(out, &format!("@N{}", self.command_counter)),
            ('N', n) => match n.parse() {
                Ok(v) => {
                    self.command_counter = v;
                    push_ack(out);
                }
                Err(_) => push_err(out, 3),
            },
            ('H', "") => {
                let pct = self.queued_bytes * 100 / self.cfg.buffer_capacity.max(1);
                push_line(out, &format!("@H{pct}"));
            }
            ('I', n) => match n.parse::<u8>() {
                Ok(n) => {
                    let v = self.inputs.get(&n).copied().unwrap_or(false);
                    push_line(out, &format!("@I{n},{}", u8::from(v)));
                }
                Err(_) => push_err(out, 3),
            },
            // Q242–Q249 are the auxiliary signals 1–8 = the `$A`/`A` bits [SPEC1 p.11–12].
            ('Q', q) => {
                let (n, set) = match q.split_once(',') {
                    Some((n, v)) => (n, Some(v)),
                    None => (q, None),
                };
                let Ok(n) = n.parse::<u8>() else {
                    return push_err(out, 3);
                };
                let aux_bit = (242..=249).contains(&n).then(|| n - 242);
                match set {
                    None => {
                        let v = aux_bit.is_some_and(|b| self.outputs & (1 << b) != 0);
                        push_line(out, &format!("@Q{n},{}", u8::from(v)));
                    }
                    Some(v) => {
                        let on = match v {
                            "0" => false,
                            "1" => true,
                            _ => return push_err(out, 46),
                        };
                        if let Some(b) = aux_bit {
                            if on {
                                self.outputs |= 1 << b
                            } else {
                                self.outputs &= !(1 << b)
                            }
                        }
                        push_ack(out);
                    }
                }
            }
            ('D', flags) => match parse_hex_u8(flags) {
                Some(f) => {
                    if f & 1 != 0 {
                        self.homed = true;
                    }
                    push_ack(out);
                }
                None => push_err(out, 3),
            },
            ('O', "") => push_line(out, &format!("@O{}", self.override_pct)),
            ('O', pct) => match pct.parse::<u32>() {
                Ok(p) if (10..=200).contains(&p) => {
                    self.override_pct = p;
                    push_ack(out);
                }
                _ => push_err(out, 55),
            },
            ('R', "") => {
                // Restart: no reply [SPEC p.14].
                let cfg = self.cfg.clone();
                let received = std::mem::take(&mut self.received);
                *self = SimCore::new(cfg);
                self.received = received;
            }
            // `@@`: not in the spec; the 601 acks it [CAP g_init]. Modelled as "clear error".
            ('@', "") => {
                self.error = 0;
                self.limit_hit = None;
                push_ack(out);
            }
            _ => push_err(out, 29),
        }
    }

    fn set_mode(&mut self, n: &str, now: SimTime, out: &mut Vec<u8>) {
        let mode = match n {
            "0" => 0,
            "1" => 1,
            "2" => 2,
            "9" => 9,
            _ => return push_err(out, 27),
        };
        if self.is_moving() {
            return push_err(out, 28);
        }
        // A used-up start counter refuses mode switches [SPEC1 p.10, *007 p.38].
        if self.cfg.start_counter_exhausted && mode != self.mode {
            return push_err(out, 7);
        }
        // Outputs are deliberately NOT cleared here: the spec says a mode switch resets
        // them, but the captures show they persist [CAP].
        if mode == 2 && !self.homed {
            return push_err(out, 41);
        }
        if mode == 2 && self.mode != 2 {
            self.path_target = self.position_at(now);
            self.path_speed = None;
        }
        self.mode = mode;
        push_ack(out);
    }

    fn brake(&mut self, now: SimTime, out: &mut Vec<u8>) {
        let was_moving = self.is_moving();
        let homing = self.homing.is_some();
        self.halt(now);
        if homing {
            self.homed = false;
        }
        push_ack(out);
        if was_moving && self.c_on(5) {
            out.push(0x01);
        }
    }

    /// Stop everything at the current position.
    fn halt(&mut self, now: SimTime) {
        self.pos = self.position_at(now);
        self.motion = None;
        self.homing = None;
        self.queue.clear();
        self.queued_bytes = 0;
        self.dead_until = None;
        self.path_target = self.pos;
        self.free_at = now;
    }

    // ------------------------------------------------------------------ mode 0

    fn mode0(&mut self, line: &str, out: &mut Vec<u8>) {
        let Some(rest) = line.strip_prefix('#') else {
            return match line.strip_prefix('&') {
                Some(q) => self.query(q, out),
                None => push_err(out, 1),
            };
        };
        let mut chars = rest.chars();
        let Some(key) = chars.next() else {
            return push_err(out, 21);
        };
        let arg = chars.as_str();
        let num = |s: &str| s.trim_start_matches(',').parse::<i64>().ok();
        let ok = match key {
            'G' => split_entry(arg)
                .and_then(|(g, v)| Some(self.speeds.insert(g, v.parse().ok()?)))
                .is_some(),
            'B' => split_entry(arg)
                .and_then(|(g, v)| Some(self.brake_angles.insert(g, v.parse().ok()?)))
                .is_some(),
            'E' => split_entry(arg)
                .and_then(|(g, v)| {
                    let (speed, angle) = v.split_once(',')?;
                    self.speeds.insert(g, speed.parse().ok()?);
                    self.brake_angles.insert(g, angle.parse().ok()?);
                    Some(())
                })
                .is_some(),
            'M' => num(arg).map(|v| self.max_speed = v as u32).is_some(),
            // `#R225` and the `#R,100` form both occur [CAP g_init].
            'R' => num(arg).map(|v| self.ramp_ms = v as u32).is_some(),
            'S' => num(arg).map(|v| self.start_speed = v as u32).is_some(),
            'O' => match arg.split_once(',') {
                Some((a, v)) => match (a.chars().next().and_then(axis_index), v.parse::<i64>()) {
                    (Some((axis, _)), Ok(v)) => {
                        self.offsets[axis] = v;
                        true
                    }
                    _ => false,
                },
                None => false,
            },
            // Per-axis parameters, stored by key and axis letter (`DX` → `0,10,11,0`).
            'Z' | 'N' | 'U' | 'Y' | 'D' => match arg.split_once(',') {
                Some((a, v))
                    if a.len() == 1 && axis_index(a.chars().next().unwrap_or(' ')).is_some() =>
                {
                    self.axis_params
                        .insert(format!("{key}{}", a.to_ascii_uppercase()), v.to_owned());
                    true
                }
                _ => false,
            },
            'I' | 'Q' => match arg.split_once(',') {
                Some((n, def)) => match n.parse::<u8>() {
                    Ok(n) => {
                        self.input_defs.insert(n, def.to_owned());
                        true
                    }
                    Err(_) => false,
                },
                None => false,
            },
            'H' => split_entry(arg)
                .and_then(|(n, v)| Some(self.counters.insert(n, v.parse().ok()?)))
                .is_some(),
            'C' => {
                let Some((n, v)) = arg.split_once(',') else {
                    return push_err(out, 3);
                };
                let Ok(n) = n.parse::<u16>() else {
                    return push_err(out, 3);
                };
                if n == 97 {
                    self.clock = v.to_owned();
                    true
                } else if KNOWN_C.contains(&n) {
                    match v.split(',').next().and_then(|v| v.parse::<i64>().ok()) {
                        Some(v) => {
                            self.c.insert(n, v);
                            true
                        }
                        None => false,
                    }
                } else {
                    return push_err(out, 21);
                }
            }
            'P' | 'L' => match arg.split_once(',') {
                Some((a, v)) => match (a.chars().next().and_then(axis_index), v.parse::<i64>()) {
                    (Some((axis, _)), Ok(v)) => {
                        let steps = if key == 'L' {
                            v
                        } else {
                            (v as f64 * self.cfg.steps_per_mm / 1000.0) as i64
                        };
                        self.pos[axis] = steps;
                        self.path_target[axis] = steps;
                        true
                    }
                    _ => false,
                },
                None => false,
            },
            'W' => true,
            'X' => return, // reload parameters: no reply [SPEC p.27]
            _ => return push_err(out, 21),
        };
        if ok { push_ack(out) } else { push_err(out, 3) }
    }

    fn query(&mut self, q: &str, out: &mut Vec<u8>) {
        let mut chars = q.chars();
        let key = chars.next();
        let arg = chars.as_str();
        let reply = match key {
            Some('C') => match arg.parse::<u16>() {
                Ok(97) => format!("&C97,{}", self.clock),
                Ok(n) if KNOWN_C.contains(&n) => format!("&C{n},{}", self.c_val(n)),
                _ => return push_err(out, 21),
            },
            Some('G') => match arg.parse::<u8>() {
                Ok(g) => format!(
                    "&G{g},{}",
                    self.speeds
                        .get(&g)
                        .copied()
                        .unwrap_or(self.cfg.default_speed)
                ),
                Err(_) => return push_err(out, 3),
            },
            // Braking angle per table entry, like `&G` (`#By,xx` [SPEC1 p.18]).
            Some('B') => match arg.parse::<u8>() {
                Ok(g) => format!("&B{g},{}", self.brake_angles.get(&g).copied().unwrap_or(0)),
                Err(_) => return push_err(out, 3),
            },
            Some('I') => match arg.parse::<u8>() {
                Ok(n) => format!(
                    "&I{n},{}",
                    self.input_defs.get(&n).map_or("000h,0,0", String::as_str)
                ),
                Err(_) => return push_err(out, 3),
            },
            Some('H') => match arg.parse::<u8>() {
                Ok(n) => format!("&H{n},{}", self.counters.get(&n).copied().unwrap_or(0)),
                Err(_) => return push_err(out, 3),
            },
            Some('O') => match arg.chars().next().and_then(axis_index) {
                Some((axis, _)) => format!("&O{},{}", arg.to_ascii_uppercase(), self.offsets[axis]),
                None => return push_err(out, 12),
            },
            Some(k @ ('D' | 'Z' | 'N' | 'U' | 'Y')) => {
                let a = arg.to_ascii_uppercase();
                if a.len() != 1 || axis_index(a.chars().next().unwrap_or(' ')).is_none() {
                    return push_err(out, 12);
                }
                // Default homing: search negative with entries 10/11 (the 601's home is at
                // the minus end; GMC homes with #G10/#G11) [CAP g_init].
                let default = if k == 'D' { "0,10,11,0" } else { "0" };
                let v = self
                    .axis_params
                    .get(&format!("{k}{a}"))
                    .map_or(default, String::as_str);
                format!("&{k}{a},{v}")
            }
            Some('M') => format!("&M{}", self.max_speed),
            Some('R') => format!("&R{}", self.ramp_ms),
            Some('S') => format!("&S{}", self.start_speed),
            _ => return push_err(out, 21),
        };
        push_line(out, &reply);
    }

    // ------------------------------------------------------------------ mode 1

    fn mode1(&mut self, line: &str, now: SimTime, out: &mut Vec<u8>) {
        let Some(rest) = line.strip_prefix('$') else {
            return push_err(out, 1);
        };
        let mut chars = rest.chars();
        let Some(key) = chars.next() else {
            return push_err(out, 31);
        };
        let arg = chars.as_str();
        match key {
            'H' => {
                if self.is_moving() {
                    return push_err(out, 30);
                }
                let axes: Option<Vec<usize>> =
                    arg.chars().map(|c| axis_index(c).map(|(a, _)| a)).collect();
                match axes {
                    Some(axes) if !axes.is_empty() => {
                        self.error = 0;
                        self.homed = false;
                        self.homing = Some(Homing {
                            axes,
                            index: 0,
                            axis_end: now + self.homing_duration(),
                        });
                        // With #C6 off the ack is withheld until homing completes [SPEC1 p.23, p.30].
                        if self.c_val(6) == 0 {
                            self.pending_home_ack = true;
                        } else {
                            push_ack(out);
                        }
                    }
                    _ => push_err(out, 12),
                }
            }
            'L' | 'E' => {
                if self.is_moving() {
                    return push_err(out, 30);
                }
                let Some((g, vals)) = split_entry(arg) else {
                    return push_err(out, 3);
                };
                let Some(vals) = parse_axis_values(vals) else {
                    return push_err(out, 3);
                };
                let from = self.pos;
                let mut to = from;
                for (axis, abs, v) in vals {
                    let steps = if key == 'L' || axis == Z {
                        v
                    } else {
                        (v as f64 * self.cfg.steps_per_mm / 1000.0) as i64
                    };
                    to[axis] = if abs { steps } else { from[axis] + steps };
                }
                self.start_motion(from, to, g, Kind::Jog, now);
                push_ack(out);
            }
            'C' => {
                if arg.is_empty() {
                    self.pos = [0; 3];
                } else if let Some((axis, _)) = arg.chars().next().and_then(axis_index) {
                    self.pos[axis] = 0;
                } else {
                    return push_err(out, 12);
                }
                self.path_target = self.pos;
                push_ack(out);
            }
            'B' => self.brake(now, out),
            'S' => {
                self.halt(now);
                self.homed = false;
                push_ack(out);
            }
            'A' => match parse_hex_u8(arg) {
                Some(bits) => {
                    self.outputs = bits;
                    push_ack(out);
                }
                None => push_err(out, 46),
            },
            'D' | 'W' => push_ack(out),
            _ => push_err(out, 31),
        }
    }

    fn homing_duration(&self) -> Duration {
        if self.cfg.instant_motion {
            Duration::ZERO
        } else {
            self.cfg.homing_time_per_axis
        }
    }

    // ------------------------------------------------------------------ mode 2

    fn mode2(&mut self, line: &str, out: &mut Vec<u8>) {
        let Some(first) = line.chars().next() else {
            return;
        };
        let item = match first {
            'L' | 'V' => {
                let Some((g, vals)) = split_entry(&line[1..]) else {
                    return push_err(out, 3);
                };
                self.path_speed = Some(g);
                match self.vector_target(vals, first == 'V') {
                    Some(target) => Item::Vector { target, speed: g },
                    None => return push_err(out, 3),
                }
            }
            'X' | 'Y' | 'Z' | 'x' | 'y' | 'z' => {
                let Some(g) = self.path_speed else {
                    return push_err(out, 2);
                };
                match self.vector_target(line, false) {
                    Some(target) => Item::Vector { target, speed: g },
                    None => return push_err(out, 3),
                }
            }
            'A' => match parse_hex_u8(&line[1..]) {
                Some(bits) => Item::Outputs(bits),
                None => return push_err(out, 46),
            },
            'W' => match line[1..].parse::<u64>() {
                Ok(ms) => Item::Dwell(Duration::from_millis(ms)),
                Err(_) => return push_err(out, 45),
            },
            // "From now on, for all subsequent moves with this entry" [SPEC1 p.34]. Modelled
            // as applied on receipt (not queued, not counted by @N); whether the firmware
            // applies it in sequence with the stream instead is what check stage 5 measures.
            'G' => {
                match split_entry(&line[1..]).and_then(|(g, v)| Some((g, v.parse::<u32>().ok()?))) {
                    Some((g, v)) => {
                        self.speeds.insert(g, v);
                    }
                    None => push_err(out, 4),
                }
                return;
            }
            // Special functions [SPEC1 p.35]: 1 = no dead time before the next vector
            // (one-shot), 8/16 = override on/off from here on. 2 and 4 are listed but
            // "noch nicht realisiert": accepted without effect. `C0` cancels a pending
            // one-shot. Anything outside the defined bits → *067 ("Wert ungültig").
            'C' => {
                let Ok(bits) = line[1..].parse::<u32>() else {
                    return push_err(out, 67);
                };
                if bits & !31 != 0 {
                    return push_err(out, 67);
                }
                self.next_no_dead_time = bits & 1 != 0;
                if bits & 8 != 0 {
                    Item::Override(true)
                } else if bits & 16 != 0 {
                    Item::Override(false)
                } else {
                    return;
                }
            }
            // Wait until input n is high (`In`) or low (`I-n`); an undefined input does not
            // wait; a bad number is *066 [SPEC1 p.35, p.40].
            'I' => {
                let (high, n) = match line[1..].strip_prefix('-') {
                    Some(n) => (false, n),
                    None => (true, &line[1..]),
                };
                match n.parse::<u8>() {
                    Ok(input) => Item::WaitInput { input, high },
                    Err(_) => return push_err(out, 66),
                }
            }
            'D' => return, // spindle: accepted, not modelled
            _ => return push_err(out, 31),
        };
        let bytes = line.len() + 1;
        if self.queued_bytes + bytes > self.cfg.buffer_capacity {
            return push_err(out, 22); // buffer overrun: "Speicher zu klein"
        }
        if let Item::Vector { target, .. } = &item {
            self.path_target = *target;
        }
        self.queue.push_back(Queued { item, bytes });
        self.queued_bytes += bytes;
        if self.c_on(7) && !self.flow_paused && self.cfg.buffer_capacity - self.queued_bytes < 200 {
            self.flow_paused = true;
            out.push(0x03); // ~200 chars before full [SPEC p.5]
        }
    }

    fn vector_target(&self, vals: &str, micrometres: bool) -> Option<[i64; 3]> {
        let mut target = self.path_target;
        for (axis, abs, v) in parse_axis_values(vals)? {
            let steps = if micrometres && axis != Z {
                (v as f64 * self.cfg.steps_per_mm / 1000.0) as i64
            } else {
                v
            };
            target[axis] = if abs { steps } else { target[axis] + steps };
        }
        Some(target)
    }

    // ------------------------------------------------------------------ time

    fn start_motion(&mut self, from: [i64; 3], to: [i64; 3], speed: u8, kind: Kind, now: SimTime) {
        self.last_path = None;
        self.start_motion_from(from, to, speed, kind, now, true);
    }

    /// Start a motion; `from_rest`: it ramps up first (it doesn't carry on from the last one).
    fn start_motion_from(
        &mut self,
        from: [i64; 3],
        to: [i64; 3],
        speed: u8,
        kind: Kind,
        now: SimTime,
        from_rest: bool,
    ) {
        // A jog of ≥ 999999 is "run until stopped" [SPEC p.30; CAP g_motions `$L20,x-999999`].
        // Even with instant motion it must keep running until braked.
        let endless = kind == Kind::Jog && (0..3).any(|a| (to[a] - from[a]).abs() >= ENDLESS_JOG);
        let mut to = to;
        let mut limit_error = None;
        for (axis, (lo, hi)) in self.cfg.envelope.iter().enumerate() {
            // Error numbers per axis: minus = 32 + 2·axis, plus = 33 + 2·axis [SPEC p.37–38]
            let base = 32 + 2 * axis as u16;
            if to[axis] < *lo {
                to[axis] = *lo;
                limit_error = Some(base);
            } else if to[axis] > *hi {
                to[axis] = *hi;
                limit_error = Some(base + 1);
            }
        }
        let duration = if endless && self.cfg.instant_motion {
            Duration::from_secs(3600)
        } else {
            self.move_duration(from, to, speed, from_rest)
        };
        self.motion = Some(Motion {
            from,
            to,
            start: now,
            end: now + duration,
            kind,
            limit_error,
        });
    }

    fn move_duration(&self, from: [i64; 3], to: [i64; 3], speed: u8, from_rest: bool) -> Duration {
        if self.cfg.instant_motion {
            return Duration::ZERO;
        }
        let d = |a: usize| (to[a] - from[a]) as f64;
        let xy = d(X).hypot(d(Y));
        let distance = if xy > 0.0 { xy } else { d(Z).abs() };
        if distance == 0.0 {
            return Duration::ZERO;
        }
        let speed = f64::from(
            self.speeds
                .get(&speed)
                .copied()
                .unwrap_or(self.cfg.default_speed)
                .max(1),
        ) * f64::from(if self.override_active {
            self.override_pct
        } else {
            100
        }) / 100.0;
        if !from_rest || self.ramp_ms == 0 {
            return Duration::from_secs_f64(distance / speed);
        }
        // From rest to rest: accelerate at #M per #R ms [SPEC1 p.8, p.19], cruise, brake. With
        // no #M yet, the ramp reaches this move's own speed.
        let top = if self.max_speed > 0 {
            f64::from(self.max_speed)
        } else {
            speed
        };
        let accel = top / (f64::from(self.ramp_ms) / 1000.0);
        let secs = if distance < speed * speed / accel {
            2.0 * (distance / accel).sqrt()
        } else {
            distance / speed + speed / accel
        };
        Duration::from_secs_f64(secs)
    }

    /// Advance the model to `now`. Returns bytes the controller sends on its own (signals,
    /// fault messages).
    pub fn tick(&mut self, now: SimTime) -> Vec<u8> {
        let mut out = Vec::new();

        // Host watchdog [SPEC p.25]: abort motion if the host is silent too long.
        let watchdog = self.c_val(19);
        if watchdog > 0
            && self.is_moving()
            && now.saturating_sub(self.last_host) > Duration::from_millis(watchdog as u64)
        {
            self.halt(now);
            self.error = 68;
            push_err(&mut out, 68);
        }

        let positioned_signal = self.c_on(5);
        let homing_done_signal = self.c_on(6);
        let homing_step = self.homing_duration();
        let dead_time = if self.cfg.instant_motion {
            Duration::ZERO
        } else {
            Duration::from_millis(self.c_val(1).max(0) as u64)
        };
        let heartbeat = self.c_val(8);
        if self.button_release_at.is_some_and(|t| now >= t) {
            self.button_release_at = None;
            out.extend(self.set_input(221, false));
        }

        loop {
            let mut progressed = false;

            if let Some(h) = &mut self.homing {
                if now >= h.axis_end {
                    let axis = h.axes[h.index];
                    // Off the switch, then the offset, then the counter zeroed [SPEC1 p.30]:
                    // GMC reads `@LX0` after homing with `#OX,300` [CAP g_init].
                    self.pos[axis] = 0;
                    self.path_target[axis] = 0;
                    if positioned_signal {
                        out.extend_from_slice(&[0x01, 0x01]); // seek + release [CAP g_init]
                    }
                    h.index += 1;
                    if h.index < h.axes.len() {
                        h.axis_end += homing_step;
                    } else {
                        self.free_at = h.axis_end;
                        self.homing = None;
                        self.homed = true;
                        if std::mem::take(&mut self.pending_home_ack) {
                            out.push(b'\r'); // the withheld `$H` ack (#C6 off)
                        }
                        if homing_done_signal {
                            out.push(0x04);
                        }
                    }
                    progressed = true;
                }
            }

            if let Some(m) = &self.motion {
                if now >= m.end {
                    let m = m.clone();
                    self.pos = m.to;
                    self.free_at = m.end;
                    self.motion = None;
                    if let Some(code) = m.limit_error {
                        self.halt(now);
                        self.error = code;
                        self.limit_hit = Some(code);
                        push_err(&mut out, code);
                    } else if m.kind == Kind::Jog && positioned_signal {
                        out.push(0x01);
                    }
                    progressed = true;
                }
            }

            if self.motion.is_none() && self.homing.is_none() && !self.queue.is_empty() {
                // Non-motion items run as soon as the machine is free: no dead time.
                let front = self.queue.front().map(|q| match q.item {
                    Item::WaitInput { .. } => Some(None),
                    Item::Override(on) => Some(Some(on)),
                    _ => None,
                });
                match front.flatten() {
                    Some(None) if self.blocked_on_input() => break,
                    Some(None) => {
                        self.pop_front_item();
                        continue;
                    }
                    Some(Some(on)) => {
                        self.override_active = on;
                        self.pop_front_item();
                        continue;
                    }
                    None => {}
                }
                let free_at = self.free_at;
                let dead = if self.next_no_dead_time {
                    Duration::ZERO
                } else {
                    dead_time
                };
                let start_at =
                    (*self.dead_until.get_or_insert(free_at.max(now) + dead)).max(free_at);
                if now >= start_at {
                    if let Some(q) = self.queue.pop_front() {
                        self.queued_bytes -= q.bytes;
                        self.command_counter += 1;
                        if matches!(q.item, Item::Vector { .. }) {
                            self.next_no_dead_time = false;
                            self.limit_hit = None;
                        }
                        match q.item {
                            Item::WaitInput { .. } | Item::Override(_) => {}
                            Item::Vector { target, speed } => {
                                let from = self.pos;
                                let dir = direction(from, target);
                                let carries_on = match (self.last_path, dir) {
                                    (Some((end, last, entry)), Some(dir)) => {
                                        end == start_at && entry == speed && {
                                            let cos = (last[0] * dir[0] + last[1] * dir[1])
                                                .clamp(-1.0, 1.0);
                                            let limit =
                                                self.brake_angles.get(&speed).copied().unwrap_or(0);
                                            cos.acos().to_degrees() <= f64::from(limit)
                                        }
                                    }
                                    _ => false,
                                };
                                self.start_motion_from(
                                    from,
                                    target,
                                    speed,
                                    Kind::Path,
                                    start_at,
                                    !carries_on,
                                );
                                self.last_path = match (&self.motion, dir) {
                                    (Some(m), Some(dir)) => Some((m.end, dir, speed)),
                                    _ => None,
                                };
                            }
                            Item::Outputs(bits) => self.outputs = bits,
                            Item::Dwell(d) => {
                                let d = if self.cfg.instant_motion {
                                    Duration::ZERO
                                } else {
                                    d
                                };
                                self.motion = Some(Motion {
                                    from: self.pos,
                                    to: self.pos,
                                    start: start_at,
                                    end: start_at + d,
                                    kind: Kind::Dwell,
                                    limit_error: None,
                                });
                            }
                        }
                        progressed = true;
                    }
                }
            }

            if !progressed {
                break;
            }
        }

        if self.motion.is_none() && self.queue.is_empty() {
            self.dead_until = None; // the path has ended; the next one waits #C1 again
        }
        if self.flow_paused && self.queued_bytes <= self.cfg.buffer_capacity / 2 {
            self.flow_paused = false;
            out.push(0x02);
        }
        // #C8 heartbeat: 0x05 after this many ms without sending anything [SPEC1 p.23].
        if heartbeat > 0
            && out.is_empty()
            && now.saturating_sub(self.last_tx) >= Duration::from_millis(heartbeat as u64)
        {
            out.push(0x05);
        }
        if !out.is_empty() {
            self.last_tx = now;
        }
        out
    }

    fn pop_front_item(&mut self) {
        if let Some(q) = self.queue.pop_front() {
            self.queued_bytes -= q.bytes;
            self.command_counter += 1;
        }
    }
}

/// Bit in `@Y` for the limit switch named by error `*032`–`*037`: axis pairs `yyxx`…,
/// the lower bit of each pair is the PLUS switch [SPEC1 p.13].
fn protocol_limit_bit(code: u16) -> Option<u32> {
    let k = code.checked_sub(32)?;
    if k > 5 {
        return None;
    }
    let axis = u32::from(k / 2);
    let plus = k % 2 == 1;
    Some(axis * 2 + if plus { 0 } else { 1 })
}

/// The XY direction of a move, as a unit vector; `None` if it doesn't move in XY.
fn direction(from: [i64; 3], to: [i64; 3]) -> Option<[f64; 2]> {
    let (dx, dy) = ((to[0] - from[0]) as f64, (to[1] - from[1]) as f64);
    let len = dx.hypot(dy);
    (len > 0.0).then(|| [dx / len, dy / len])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    fn text(bytes: &[u8]) -> String {
        bytes
            .iter()
            .map(|&b| {
                if (0x20..0x7f).contains(&b) {
                    (b as char).to_string()
                } else {
                    format!("<{b:02x}>")
                }
            })
            .collect()
    }

    /// Send `line` at time `t`, return the reply as text (CR shown as <0d>).
    fn send(core: &mut SimCore, line: &str, t: u64) -> String {
        text(&core.handle_line(line, ms(t)))
    }

    fn homed() -> SimCore {
        SimCore::new(SimConfig {
            homed_at_start: true,
            ..SimConfig::default()
        })
    }

    #[test]
    fn status_sequence_after_a_vector_matches_captures() {
        // [CAP g_regular_mount]: after L20 the controller reports 15 (dead time), 14, then 04.
        let mut c = homed();
        for (cmd, t) in [
            ("#C1,100", 0),
            ("#G20,4000", 0),
            ("@M2", 0),
            ("L20,X400,Y0,Z0", 0),
        ] {
            send(&mut c, cmd, t);
        }
        assert_eq!(
            send(&mut c, "@X", 10),
            "@X15<0d>",
            "waiting out #C1 dead time"
        );
        assert_eq!(send(&mut c, "@X", 150), "@X14<0d>", "moving");
        assert_eq!(
            send(&mut c, "@X", 300),
            "@X04<0d>",
            "400 steps @ 4000/s = 100 ms after 100 ms dead time"
        );
        assert_eq!(c.snapshot(ms(300)).position, [400, 0, 0]);
    }

    #[test]
    fn homing_reports_18_then_signals() {
        // [CAP g_init]: $Hz → CR; @X18 while homing; then 0x01 0x01 0x04.
        let mut c = SimCore::new(SimConfig::default());
        send(&mut c, "#C5,1", 0);
        send(&mut c, "@M1", 0);
        assert_eq!(send(&mut c, "$Hz", 0), "<0d>");
        assert_eq!(send(&mut c, "@X", 100), "@X18<0d>");
        assert_eq!(text(&c.tick(ms(3000))), "<01><01><04>");
        assert_eq!(send(&mut c, "@X", 3001), "@X04<0d>");
    }

    #[test]
    fn rules_from_spec_and_captures() {
        let mut c = SimCore::new(SimConfig::default());
        assert_eq!(send(&mut c, "#C52,800", 0), "*021<0d>", "[CAP g_init]");
        assert_eq!(send(&mut c, "#C35,1", 0), "<0d>", "[CAP g_init]");
        assert_eq!(send(&mut c, "#R,100", 0), "<0d>", "[CAP g_init]");
        assert_eq!(send(&mut c, "@M2", 0), "*041<0d>", "mode 2 requires homing");
        assert_eq!(
            send(&mut c, "$Hz", 0),
            "*001<0d>",
            "$ commands only in mode 1"
        );
        assert_eq!(
            send(&mut c, "@Q", 0),
            "*003<0d>",
            "@Q needs an output number"
        );
        assert_eq!(send(&mut c, "@K", 0), "*029<0d>", "unknown master command");
    }

    #[test]
    fn mode_change_only_at_standstill() {
        let mut c = homed();
        send(&mut c, "@M1", 0);
        send(&mut c, "$L20,x4000", 0);
        assert_eq!(send(&mut c, "@M0", 10), "*028<0d>");
        assert_eq!(send(&mut c, "@M0", 2000), "<0d>");
    }

    #[test]
    fn brake_mid_jog_keeps_position_and_signals() {
        // [CAP g_motions]: $L20,x-999999 … @B → CR 0x01, then @LX reads where it stopped.
        let mut c = homed();
        send(&mut c, "#C5,1", 0);
        send(&mut c, "@M1", 0);
        send(&mut c, "$L20,x999999", 0);
        assert_eq!(send(&mut c, "@B", 500), "<0d><01>");
        let x = c.snapshot(ms(500)).position[X];
        assert!(
            (4000..=6000).contains(&x),
            "≈ 0.5 s at 10000 steps/s, got {x}"
        );
        assert_eq!(send(&mut c, "@LX", 600), format!("@LX{x}<0d>"));
        assert!(c.snapshot(ms(600)).homed, "@B keeps the reference");
    }

    #[test]
    fn emergency_stop_loses_reference() {
        let mut c = homed();
        send(&mut c, "@M1", 0);
        send(&mut c, "$L20,x999", 0);
        send(&mut c, "@S", 10);
        assert!(!c.snapshot(ms(10)).homed);
        assert_eq!(send(&mut c, "@X", 11), "@X00<0d>");
    }

    #[test]
    fn limit_switch_stops_motion_with_error() {
        let mut c = homed();
        send(&mut c, "@M1", 0);
        send(&mut c, "$L20,X70000", 0); // beyond the 60000 envelope
        let out = text(&c.tick(ms(60_000)));
        assert_eq!(out, "*033<0d>", "EENDP1");
        let s = c.snapshot(ms(60_000));
        assert_eq!((s.position[X], s.error, s.moving), (60_000, 33, false));
        assert_eq!(send(&mut c, "@X", 60_001), "@X06<0d>", "error flag set");
        assert_eq!(send(&mut c, "@@", 60_002), "<0d>");
        assert_eq!(send(&mut c, "@X", 60_003), "@X04<0d>");
    }

    #[test]
    fn flow_control_signals_and_overrun() {
        let cfg = SimConfig {
            homed_at_start: true,
            buffer_capacity: 400,
            ..SimConfig::default()
        };
        // Without #C7: no warning, overrun is *022.
        let mut c = SimCore::new(cfg.clone());
        send(&mut c, "@M2", 0);
        let mut replies = String::new();
        for i in 0..40 {
            replies += &send(&mut c, &format!("L20,X{},Y0,Z0", i * 10), 0);
        }
        assert!(replies.contains("*022"), "overrun reported: {replies}");

        // With #C7,1: 0x03 before full, 0x02 once drained to half.
        let mut c = SimCore::new(cfg);
        send(&mut c, "#C7,1", 0);
        send(&mut c, "@M2", 0);
        let mut replies = String::new();
        for i in 0..16 {
            // ~240 bytes queued: inside the 200-byte warning zone, below 400
            replies += &send(&mut c, &format!("L20,X{},Y0,Z0", i * 10), 0);
        }
        assert!(
            replies.contains("<03>") && !replies.contains("*022"),
            "{replies}"
        );
        assert_eq!(text(&c.tick(ms(5_000))), "<02>");
    }

    #[test]
    fn host_watchdog_aborts_motion() {
        let mut c = homed();
        send(&mut c, "#C19,200", 0);
        send(&mut c, "@M1", 0);
        send(&mut c, "$L20,x999999", 0);
        assert_eq!(text(&c.tick(ms(150))), "", "host still within 200 ms");
        assert_eq!(
            text(&c.tick(ms(250))),
            "*068<0d>",
            "no host byte for > 200 ms"
        );
        assert!(!c.snapshot(ms(250)).moving);
    }

    #[test]
    fn start_button() {
        // A press holds I221 high for BUTTON_HOLD, then tick releases it.
        let mut c = homed();
        assert_eq!(send(&mut c, "@I221", 0), "@I221,0<0d>");
        assert_eq!(
            c.press_start_button(ms(0)),
            Vec::<u8>::new(),
            "no bytes without #C35"
        );
        assert_eq!(send(&mut c, "@I221", 100), "@I221,1<0d>", "held");
        assert_eq!(c.tick(ms(200)), Vec::<u8>::new());
        assert_eq!(send(&mut c, "@I221", 201), "@I221,0<0d>", "released");

        send(&mut c, "#C35,1", 300);
        assert_eq!(
            c.press_start_button(ms(300)),
            vec![0x06],
            "#C35 bit 1: high"
        );
        assert_eq!(c.tick(ms(500)), vec![0x07], "then low on release");

        send(&mut c, "#C35,2", 600);
        assert_eq!(c.press_start_button(ms(600)), Vec::<u8>::new(), "bit 1 off");
        c.tick(ms(800));
        assert_eq!(c.set_input(222, true), vec![0x08]);
        assert_eq!(
            c.set_input(222, true),
            Vec::<u8>::new(),
            "only level changes"
        );
        assert_eq!(c.set_input(222, false), vec![0x09]);
    }

    #[test]
    fn button_pressed_before_a_wait_does_not_satisfy_it() {
        // A press is a level, not a latch: an earlier press must not release a later I221.
        let mut c = homed();
        for cmd in ["#G20,4000", "@M2"] {
            send(&mut c, cmd, 0);
        }
        c.press_start_button(ms(0));
        c.tick(ms(300));
        send(&mut c, "I221", 400);
        send(&mut c, "L20,X400,Y0,Z0", 400);
        c.tick(ms(1000));
        assert_eq!(c.snapshot(ms(1000)).position[0], 0, "still held");
        c.press_start_button(ms(1000));
        c.tick(ms(1001));
        c.tick(ms(1500));
        assert_eq!(c.snapshot(ms(1500)).position[0], 400);
    }

    #[test]
    fn c1_skips_dead_time_for_the_next_vector_only() {
        let mut c = homed();
        for cmd in ["#C1,100", "#G20,4000", "@M2", "C1", "L20,X400,Y0,Z0"] {
            send(&mut c, cmd, 0);
        }
        assert_eq!(
            send(&mut c, "@X", 10),
            "@X14<0d>",
            "moving at once: no dead time"
        );
        c.tick(ms(200));
        send(&mut c, "L20,X0,Y0,Z0", 200);
        assert_eq!(
            send(&mut c, "@X", 210),
            "@X15<0d>",
            "dead time again for a new path"
        );
    }

    #[test]
    fn c_flag_values() {
        // [SPEC1 p.35]: 2 and 4 are "not yet realised" (accepted, no effect); C0 cancels a
        // pending one-shot; undefined values are *067.
        let mut c = homed();
        for cmd in ["#C1,100", "#G20,4000", "@M2"] {
            send(&mut c, cmd, 0);
        }
        assert_eq!(send(&mut c, "C2", 0), "");
        assert_eq!(send(&mut c, "C4", 0), "");
        assert_eq!(
            send(&mut c, "C9", 0),
            "",
            "1 | 8 is fine, no reply in mode 2"
        );
        assert_eq!(send(&mut c, "C32", 0), "*067<0d>");
        assert_eq!(send(&mut c, "Cx", 0), "*067<0d>");
        assert_eq!(send(&mut c, "C0", 0), "", "cancels the C1 inside C9");
        send(&mut c, "L20,X400,Y0,Z0", 0);
        assert_eq!(
            send(&mut c, "@X", 10),
            "@X15<0d>",
            "dead time applies again"
        );
        assert_eq!(send(&mut c, "I-x", 10), "*066<0d>");
    }

    #[test]
    fn override_off_in_stream_ignores_at_o() {
        let mut c = homed();
        for cmd in [
            "#G20,4000",
            "@O50",
            "@M2",
            "C16",
            "L20,X400,Y0,Z0",
            "C8",
            "L20,X0,Y0,Z0",
        ] {
            send(&mut c, cmd, 0);
        }
        assert_eq!(send(&mut c, "@X", 99), "@X14<0d>");
        assert_eq!(
            c.snapshot(ms(100)).position[0],
            400,
            "override off: full speed, 100 ms"
        );
        c.tick(ms(100));
        c.tick(ms(250));
        assert_ne!(
            c.snapshot(ms(250)).position[0],
            0,
            "override back on: 50 %, 200 ms"
        );
        c.tick(ms(300));
        assert_eq!(c.snapshot(ms(300)).position[0], 0);
    }

    #[test]
    fn dwell_reports_waiting_and_outputs_apply_in_order() {
        let mut c = homed();
        for cmd in [
            "#G20,4000",
            "@M2",
            "A01",
            "W100",
            "A03",
            "L20,X400,Y0,Z0",
            "A00",
        ] {
            send(&mut c, cmd, 0);
        }
        c.tick(ms(1));
        assert_eq!(send(&mut c, "@Q242", 1), "@Q242,1<0d>", "A01 applied");
        assert_eq!(
            send(&mut c, "@Q243", 1),
            "@Q243,0<0d>",
            "A03 not yet: dwell first"
        );
        assert_eq!(send(&mut c, "@X", 50), "@X15<0d>", "dwell reports waiting");
        c.tick(ms(110));
        assert_eq!(send(&mut c, "@Q243", 110), "@Q243,1<0d>");
        c.tick(ms(300));
        assert_eq!(
            send(&mut c, "@Q242", 300),
            "@Q242,0<0d>",
            "A00 after the vector"
        );
        assert_eq!(send(&mut c, "@N", 300), "@N5<0d>", "five items executed");
    }

    #[test]
    fn wait_input_blocks_until_level() {
        let mut c = homed();
        for cmd in ["#G20,4000", "@M2", "I221", "L20,X400,Y0,Z0"] {
            send(&mut c, cmd, 0);
        }
        c.tick(ms(500));
        assert_eq!(send(&mut c, "@X", 500), "@X15<0d>", "waiting on I221");
        assert_eq!(c.snapshot(ms(500)).position[0], 0);
        c.set_input(221, true);
        c.tick(ms(501));
        c.tick(ms(1000));
        assert_eq!(c.snapshot(ms(1000)).position[0], 400);
    }

    #[test]
    fn q_outputs_are_the_a_bits() {
        let mut c = homed();
        send(&mut c, "@M1", 0);
        send(&mut c, "$A05", 0);
        assert_eq!(send(&mut c, "@Q242", 0), "@Q242,1<0d>");
        assert_eq!(send(&mut c, "@Q243", 0), "@Q243,0<0d>");
        assert_eq!(send(&mut c, "@Q244", 0), "@Q244,1<0d>");
        assert_eq!(send(&mut c, "@Q243,1", 0), "<0d>");
        assert_eq!(send(&mut c, "@Q243", 0), "@Q243,1<0d>", "set via @Q");
    }

    #[test]
    fn y_reports_the_limit_switch_until_cleared() {
        let mut c = homed();
        assert_eq!(send(&mut c, "@Y", 0), "@Y4000<0d>");
        send(&mut c, "@M1", 0);
        send(&mut c, "$L20,X999999", 0);
        let out = text(&c.tick(ms(3_700_000)));
        assert_eq!(out, "*033<0d>", "X plus limit");
        assert_eq!(
            send(&mut c, "@Y", 3_700_000),
            "@Y4001<0d>",
            "spec example: X+ limit, ready"
        );
        send(&mut c, "@@", 3_700_000);
        assert_eq!(send(&mut c, "@Y", 3_700_000), "@Y4000<0d>");
    }

    #[test]
    fn c8_heartbeat_when_quiet() {
        let mut c = homed();
        send(&mut c, "#C8,100", 0);
        assert_eq!(c.tick(ms(50)), Vec::<u8>::new());
        assert_eq!(c.tick(ms(100)), vec![0x05]);
        assert_eq!(c.tick(ms(150)), Vec::<u8>::new());
        send(&mut c, "@X", 190);
        assert_eq!(
            c.tick(ms(250)),
            Vec::<u8>::new(),
            "the reply at 190 restarted the timer"
        );
        assert_eq!(c.tick(ms(290)), vec![0x05]);
    }

    #[test]
    fn c6_off_withholds_the_home_ack() {
        let mut c = SimCore::new(SimConfig {
            homing_done_signal: false,
            ..SimConfig::default()
        });
        send(&mut c, "@M1", 0);
        assert_eq!(send(&mut c, "$Hz", 0), "", "no ack while homing");
        assert_eq!(text(&c.tick(ms(3000))), "<0d>", "ack on completion");
    }

    #[test]
    fn exhausted_start_counter_refuses_mode_switch() {
        let mut c = SimCore::new(SimConfig {
            start_counter_exhausted: true,
            ..SimConfig::default()
        });
        assert_eq!(send(&mut c, "@M1", 0), "*007<0d>");
        assert_eq!(send(&mut c, "@M0", 0), "<0d>", "staying in mode 0 is fine");
    }
}
