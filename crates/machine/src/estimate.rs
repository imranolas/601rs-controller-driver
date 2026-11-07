//! What a lowered job will take, before it runs: the run time, and how much of the serial line
//! each path needs (the byte budget, crate docs "Keeping up with the cut").
//!
//! The run time follows the steps as the controller would: the speeds, ramps, start speed and
//! dead times set, a look-ahead along each path that keeps the speed through turns up to the
//! braking angle, the radius slowdown on curves (protocol reference §2.4.1), head settles, and
//! the host's side: the line time of every command, a reply for each one outside a stream, a
//! mode switch where the mode changes, and noticing the controller is idle. The host's numbers
//! are assumptions ([`Host`]); `tests/estimate.rs` holds the whole against GMC's captured jobs.

use protocol::{Command, Mode, Radius, Setting};

use crate::lower::{Job, Step};
use crate::profile::Profile;

/// The host's side of the timing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Host {
    /// From sending a command outside a stream to having its reply, beyond the line time, ms.
    pub reply_ms: f64,
    /// How often the controller is asked whether it is idle, ms: on average half of this
    /// passes between the end of a motion and noticing it.
    pub poll_ms: f64,
}

impl Default for Host {
    /// The controller session's poll interval (`controller::Config`), and a reply time that
    /// brings the estimate of GMC's captured jobs within a few per cent of their run times.
    fn default() -> Self {
        Self {
            reply_ms: 12.0,
            poll_ms: 50.0,
        }
    }
}

/// A job's run time, by what it goes on, in seconds.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Estimate {
    /// Head-up moves.
    pub travel_s: f64,
    /// Moves with a head down.
    pub cutting_s: f64,
    /// Dead times before paths, dwells, and heads settling.
    pub waiting_s: f64,
    /// Sending commands, replies, mode switches, and noticing the controller is idle.
    pub talking_s: f64,
    pub total_s: f64,
    /// `pause` ops, waiting for someone: not counted.
    pub holds: usize,
    /// Per op of the program.
    pub ops: Vec<OpEstimate>,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct OpEstimate {
    pub seconds: f64,
    /// The most of the serial line any of its paths needs: its busiest quarter second's bytes
    /// against what the line carries, at the full feed (so curves count at more than the
    /// radius slowdown lets them run). Over [`crate::Line::budget_share`], the carriage may wait
    /// for data mid-path.
    pub line_share: f64,
}

/// The window over which a path's bytes are weighed against the line.
const WINDOW_S: f64 = 0.25;

/// Estimate a lowered job's run time and line load.
pub fn estimate(job: &Job, profile: &Profile, host: &Host) -> Estimate {
    let mut e = Estimate::default();
    let mut c = Controller::new(profile);
    let sections = std::iter::once(&job.setup)
        .chain(&job.ops)
        .chain(std::iter::once(&job.finish));
    let last = job.ops.len() + 1;
    for (k, steps) in sections.enumerate() {
        let before = e.total_s;
        let mut share: f64 = 0.0;
        let mut run: Vec<&Command> = Vec::new();
        for step in steps {
            match step {
                Step::Send(cmd) if cmd.required_mode() == Some(Mode::Cutting) => run.push(cmd),
                Step::Send(cmd) => {
                    share = share.max(c.run(&mut run, &mut e, host));
                    c.command(cmd, &mut e, host);
                }
                Step::WaitIdle => {
                    share = share.max(c.run(&mut run, &mut e, host));
                    c.queue_empty = true;
                    e.talking_s += host.poll_ms / 2000.0;
                }
                Step::Settle(ms) => {
                    share = share.max(c.run(&mut run, &mut e, host));
                    e.waiting_s += f64::from(*ms) / 1000.0;
                }
                Step::Hold { .. } => {
                    share = share.max(c.run(&mut run, &mut e, host));
                    c.queue_empty = true;
                    e.holds += 1;
                }
            }
            e.total_s = e.travel_s + e.cutting_s + e.waiting_s + e.talking_s;
        }
        share = share.max(c.run(&mut run, &mut e, host));
        e.total_s = e.travel_s + e.cutting_s + e.waiting_s + e.talking_s;
        if (1..last).contains(&k) {
            e.ops.push(OpEstimate {
                seconds: e.total_s - before,
                line_share: share,
            });
        }
    }
    e
}

/// The controller's state, as far as timing needs it.
struct Controller<'a> {
    profile: &'a Profile,
    mode: Option<Mode>,
    speeds: [u32; 256],
    ramp_ms: u16,
    start_speed: u32,
    dead_time_ms: u16,
    brake_deg: [u8; 256],
    radius_min: f64,
    radius_max: f64,
    outputs: u8,
    pos: [i32; 3],
    /// Nothing queued: the next path waits the dead time first.
    queue_empty: bool,
}

/// One vector of a path.
struct Segment {
    len: f64,
    dir: Option<[f64; 2]>,
    turn_deg: f64,
    /// How far the blade turns along it, radians: the controller's radius comes from this.
    blade_rad: f64,
    entry: u8,
    bytes: usize,
    cutting: bool,
}

impl<'a> Controller<'a> {
    fn new(profile: &'a Profile) -> Self {
        let r = profile.radius_slowdown;
        Self {
            profile,
            mode: None,
            speeds: [0; 256],
            ramp_ms: profile.ramp_ms(profile.accel.travel),
            start_speed: profile.start_speed,
            dead_time_ms: 0,
            brake_deg: [0; 256],
            radius_min: f64::from(r.min_radius_steps),
            radius_max: f64::from(r.max_radius_steps),
            outputs: profile.neutral_outputs,
            pos: [profile.park_steps[0], profile.park_steps[1], 0],
            queue_empty: true,
        }
    }

    fn line_s(&self, bytes: usize) -> f64 {
        bytes as f64 / self.profile.line.bytes_per_sec
    }

    /// A command outside a stream: line time, a reply, and a mode switch first if needed.
    fn command(&mut self, cmd: &Command, e: &mut Estimate, host: &Host) {
        if let Some(m) = cmd.required_mode() {
            if self.mode != Some(m) {
                e.talking_s += self.line_s(4) + host.reply_ms / 1000.0;
                self.mode = Some(m);
            }
        }
        e.talking_s += self.line_s(cmd.encode().len()) + host.reply_ms / 1000.0;
        match *cmd {
            Command::Set(s) => match s {
                Setting::Speed { entry, value } => self.speeds[usize::from(entry)] = value,
                Setting::RampMs(ms) => self.ramp_ms = ms,
                Setting::StartSpeed(v) => self.start_speed = v,
                Setting::DeadTimeMs(ms) => self.dead_time_ms = ms,
                Setting::BrakeAngle { entry, degrees } => {
                    self.brake_deg[usize::from(entry)] = degrees
                }
                Setting::Radius(Radius::Min, v) => self.radius_min = f64::from(v),
                Setting::Radius(Radius::Max, v) => self.radius_max = f64::from(v),
                _ => {}
            },
            Command::SetOutputs(bits) => self.outputs = bits,
            _ => {}
        }
    }

    fn head_down(&self) -> bool {
        let p = self.profile;
        let low = self.outputs & 0x0F;
        [p.bevel, p.straight]
            .iter()
            .any(|h| low == h.down || h.down_closed.is_some_and(|d| low == d))
    }

    /// Run the stream commands gathered so far; returns the most of the line its paths need.
    fn run(&mut self, run: &mut Vec<&Command>, e: &mut Estimate, host: &Host) -> f64 {
        if run.is_empty() {
            return 0.0;
        }
        if self.mode != Some(Mode::Cutting) {
            e.talking_s += self.line_s(4) + host.reply_ms / 1000.0;
            self.mode = Some(Mode::Cutting);
        }
        // A path from an empty queue waits the dead time first, unless a `C1` skips it.
        let mut dead = std::mem::take(&mut self.queue_empty);
        let mut share: f64 = 0.0;
        let mut piece: Vec<Segment> = Vec::new();
        let mut bytes = 0;
        let mut entry = 20u8;
        let mut moving_s = 0.0;
        let flush = |c: &mut Self,
                     piece: &mut Vec<Segment>,
                     e: &mut Estimate,
                     moving_s: &mut f64,
                     share: &mut f64| {
            if piece.is_empty() {
                return;
            }
            *share = share.max(c.line_share(piece));
            for (s, t) in piece.iter().zip(c.plan(piece)) {
                if s.cutting {
                    e.cutting_s += t;
                } else {
                    e.travel_s += t;
                }
                *moving_s += t;
            }
            piece.clear();
        };
        for &cmd in run.iter() {
            bytes += cmd.encode().len();
            match *cmd {
                Command::Vector {
                    speed_entry,
                    x,
                    y,
                    z,
                } => {
                    entry = speed_entry;
                    self.push(&mut piece, [x, y, z], entry, cmd.encode().len());
                }
                Command::Step { x, y, z } => {
                    let to = [self.pos[0] + x, self.pos[1] + y, self.pos[2] + z];
                    self.push(&mut piece, to, entry, cmd.encode().len());
                }
                Command::StreamOutputs(bits) => {
                    flush(self, &mut piece, e, &mut moving_s, &mut share);
                    self.outputs = bits;
                }
                Command::StreamDwellMs(ms) => {
                    flush(self, &mut piece, e, &mut moving_s, &mut share);
                    e.waiting_s += f64::from(ms) / 1000.0;
                    moving_s += f64::from(ms) / 1000.0;
                }
                Command::StreamFlags(f) if f.no_dead_time => dead = false,
                _ => {}
            }
            if dead && !piece.is_empty() {
                e.waiting_s += f64::from(self.dead_time_ms) / 1000.0;
                moving_s += f64::from(self.dead_time_ms) / 1000.0;
                dead = false;
            }
        }
        flush(self, &mut piece, e, &mut moving_s, &mut share);
        // The line may be slower than the motion: then the motion waits for it.
        let line = self.line_s(bytes);
        if line > moving_s {
            e.talking_s += line - moving_s;
        }
        run.clear();
        share
    }

    fn push(&mut self, piece: &mut Vec<Segment>, to: [i32; 3], entry: u8, bytes: usize) {
        let d = |a: usize| f64::from(to[a] - self.pos[a]);
        let xy = d(0).hypot(d(1));
        let dz = d(2).abs();
        let dir = (xy > 0.0).then(|| [d(0) / xy, d(1) / xy]);
        let turn_deg = match (piece.last().and_then(|s| s.dir), dir) {
            (Some(a), Some(b)) => (a[0] * b[0] + a[1] * b[1])
                .clamp(-1.0, 1.0)
                .acos()
                .to_degrees(),
            _ => 0.0,
        };
        // A turn beyond the braking angle stops the carriage: a new piece (the caller's
        // segments are planned piece by piece, so mark it with a stop).
        let stop = turn_deg > f64::from(self.brake_deg[usize::from(entry)]);
        let len = if xy > 0.0 { xy } else { dz };
        let turn_deg = if stop { f64::INFINITY } else { turn_deg };
        let blade_rad = (dz / self.profile.scale.z_steps_per_deg).to_radians();
        piece.push(Segment {
            len,
            dir,
            turn_deg,
            blade_rad,
            entry,
            bytes,
            cutting: self.head_down(),
        });
        self.pos = to;
    }

    /// The radius slowdown's speed where the blade turns by `turn_rad` over `len` steps.
    fn radius_limit(&self, len: f64, turn_rad: f64, cruise: f64) -> f64 {
        if turn_rad <= 0.0 || len >= self.radius_max {
            return cruise;
        }
        let r = len / turn_rad;
        let s = f64::from(self.start_speed).min(cruise);
        if r >= self.radius_max {
            cruise
        } else if r >= self.radius_min {
            s + (cruise - s) * (r - self.radius_min) / (self.radius_max - self.radius_min)
        } else {
            s * r / self.radius_min
        }
        .max(1.0)
    }

    /// Seconds per segment of a piece, by a look-ahead planner: accelerate at `#M` per `#R`,
    /// start and stop at the start speed, and keep the speed through turns up to the braking
    /// angle (a sharper one is a stop).
    fn plan(&self, piece: &[Segment]) -> Vec<f64> {
        let a = if self.ramp_ms == 0 {
            f64::INFINITY
        } else {
            f64::from(self.profile.max_speed) / (f64::from(self.ramp_ms) / 1000.0)
        };
        let n = piece.len();
        let cruise: Vec<f64> = piece
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let c = f64::from(self.speeds[usize::from(s.entry)].max(1));
                // `#C14` tangential: the radius from the blade's turn, averaged over the last
                // three vectors [SPEC1 p.24–25].
                let w = &piece[i.saturating_sub(2)..=i];
                let (len, turn) = w
                    .iter()
                    .fold((0.0, 0.0), |(l, t), s| (l + s.len, t + s.blade_rad));
                if s.cutting {
                    self.radius_limit(len, turn, c)
                } else {
                    c
                }
            })
            .collect();
        let start = |c: f64| f64::from(self.start_speed).min(c);
        // Junction speeds: at the ends and at stops, the start speed; elsewhere the slower side.
        let mut j = vec![0.0; n + 1];
        j[0] = start(cruise[0]);
        j[n] = start(cruise[n - 1]);
        for i in 1..n {
            j[i] = if piece[i].turn_deg.is_finite() {
                cruise[i - 1].min(cruise[i])
            } else {
                start(cruise[i - 1].min(cruise[i]))
            };
        }
        if a.is_finite() {
            for i in 0..n {
                j[i + 1] = j[i + 1].min((j[i] * j[i] + 2.0 * a * piece[i].len).sqrt());
            }
            for i in (0..n).rev() {
                j[i] = j[i].min((j[i + 1] * j[i + 1] + 2.0 * a * piece[i].len).sqrt());
            }
        }
        (0..n)
            .map(|i| {
                let (v0, v1, l) = (j[i], j[i + 1], piece[i].len);
                let c = cruise[i].max(v0).max(v1);
                if !a.is_finite() {
                    return l / c;
                }
                let (d0, d1) = ((c * c - v0 * v0) / (2.0 * a), (c * c - v1 * v1) / (2.0 * a));
                if d0 + d1 <= l {
                    (c - v0) / a + (c - v1) / a + (l - d0 - d1) / c
                } else {
                    let vp = ((2.0 * a * l + v0 * v0 + v1 * v1) / 2.0).sqrt();
                    (vp - v0) / a + (vp - v1) / a
                }
            })
            .collect()
    }

    /// The busiest quarter second of a piece's bytes against the line, at the full feed.
    fn line_share(&self, piece: &[Segment]) -> f64 {
        if !piece.iter().any(|s| s.cutting) {
            return 0.0;
        }
        let times: Vec<f64> = piece
            .iter()
            .map(|s| s.len / f64::from(self.speeds[usize::from(s.entry)].max(1)))
            .collect();
        let total: f64 = times.iter().sum();
        let window = WINDOW_S.min(total).max(1e-6);
        let (mut lo, mut t, mut b, mut best) = (0, 0.0, 0usize, 0.0_f64);
        for hi in 0..piece.len() {
            t += times[hi];
            b += piece[hi].bytes;
            while t - times[lo] >= window && lo < hi {
                t -= times[lo];
                b -= piece[lo].bytes;
                lo += 1;
            }
            if t >= window - 1e-9 {
                best = best.max(b as f64 / t);
            }
        }
        best / self.profile.line.bytes_per_sec
    }
}
