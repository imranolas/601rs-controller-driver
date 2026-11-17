//! A program lowered to the steps that run it: commands to send, waits until the controller is
//! idle, head settles and holds. Two forms, by the profile's [`Capabilities`]:
//!
//! - **GMC's** (no capabilities): per cut, the settings in mode 0, the head down and up with
//!   `$A` in mode 1 and a host wait while it settles, the path in mode 2, and a wait until the
//!   controller is idle before anything else (protocol reference §4). The captures show it
//!   working on this firmware; the golden tests (`tests/golden.rs`) hold it to them.
//! - **One stream per cut** (the capabilities `mc601 check` stage 5 shows): the travel, the
//!   head down, the settle, the path and the head up as one mode-2 stream, the path sent
//!   compactly. Settings change in mode 0 only when they must, so cuts with the same settings
//!   follow one another in one stream; the runner decides how far ahead to send them (just in
//!   time, so that a pause still stops at the end of a cut).
//!
//! In both, the head is up whenever the carriage travels, and a wait until idle comes before
//! anything that isn't part of a stream (a setting, an output in mode 1, a host wait), so
//! nothing changes while the carriage runs. A setting is only sent when its value changes.
//!
//! After a bevel edge GMC takes the blade out (`$A40`) and then raises the head (`$A42`), as
//! here, but on thin board it skips the second step between the edges of one opening and hops
//! a few millimetres with the head down ([GMC]; on
//! board thicker than its `SafeTurn`, 2 mm [GCF], it doesn't). This doesn't copy that.
//!
//! A closed curve (a cut with no corners that turns through a full loop) goes down with the
//! head's `down_closed` outputs and takes the material's `bevel_round` trim, as GMC's oval does.
//! The overlap that closes it is the compiler's.

use std::fmt;

use std::collections::HashMap;

use program::{ClampSide, Cut, Head as HeadKind, Op, Program, Resume};
use protocol::{Command, Mode, Radius, Setting, SettingKey, StreamFlags};

use crate::frame::{self, CarriageMm, MachineMm, Pt};
use crate::profile::{Capabilities, Pad, Profile};

/// The speed-table entry cutting uses, and in GMC's form travel too.
pub const ENTRY: u8 = 20;

/// The speed-table entry travel uses in one stream with the cut (`Capabilities::travel_entry`).
pub const TRAVEL_ENTRY: u8 = 21;

/// A turn up to this at a point (degrees) is part of a curve: the blade turns while the
/// carriage moves. A sharper one is a corner (PROGRAM_FORMAT §4.2).
pub const SMOOTH_TURN_DEG: f64 = 3.0;

/// Consecutive points closer than this are an error (PROGRAM_FORMAT V4).
pub const MIN_SEGMENT_MM: f64 = 0.01;

/// One thing the runner does, in order.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// Send a command, in the mode it needs ([`Command::required_mode`]).
    Send(Command),
    /// Wait until the controller has finished everything sent (`@X` idle). Comes before any
    /// step that isn't part of a stream, so nothing changes while the carriage runs.
    WaitIdle,
    /// Wait on the host, ms, for a head to settle after `$A` (the material's
    /// `head_settle_ms`; GMC sleeps the same).
    Settle(u32),
    /// Hold with the head up until the operator or the green button says go (a `pause` op).
    Hold { message: String, resume: Resume },
}

/// A program as steps: the job's settings, then each op, then the end of the job.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Job {
    pub setup: Vec<Step>,
    /// One list per program op, in order, so the runner can report progress and failures by
    /// op and pause between them.
    pub ops: Vec<Vec<Step>>,
    /// Heads up, park, clamps released.
    pub finish: Vec<Step>,
}

impl Job {
    pub fn steps(&self) -> impl Iterator<Item = &Step> {
        self.setup
            .iter()
            .chain(self.ops.iter().flatten())
            .chain(&self.finish)
    }

    pub fn commands(&self) -> impl Iterator<Item = &Command> {
        self.steps().filter_map(|s| match s {
            Step::Send(c) => Some(c),
            _ => None,
        })
    }
}

/// Where in a job something is wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum At {
    /// The job as a whole (its material).
    Job,
    /// An op, by its index in the program.
    Op(usize),
    /// The park after the last op.
    Park,
}

impl fmt::Display for At {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            At::Job => f.write_str("the job"),
            At::Op(op) => write!(f, "op {op}"),
            At::Park => f.write_str("the park"),
        }
    }
}

/// Why a program can't be lowered. `op` is the index into the program's ops; `point` into a
/// cut's `path_mm`.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error {
    /// A number isn't finite, or is out of range: a speed outside 1–100 %, a thickness that
    /// isn't positive (PROGRAM_FORMAT V2, V3).
    #[error("{at}: a number is not finite or out of range")]
    BadNumber { at: At },
    /// A cut has fewer than two points (V3).
    #[error("op {op}: a cut needs at least two points")]
    TooShort { op: usize },
    /// Two consecutive points are closer than [`MIN_SEGMENT_MM`] (V4).
    #[error("op {op}: points {} and {point} are closer than {} mm", .point - 1, MIN_SEGMENT_MM)]
    ZeroSegment { op: usize, point: usize },
    /// A cut turns more than the break angle at a point (V7): it must be two cuts.
    #[error("op {op}: the cut turns {turn_deg:.1}° at point {point}; split it into two cuts")]
    SharpCorner {
        op: usize,
        point: usize,
        turn_deg: f64,
    },
    /// A blade would leave the machine's soft limits (V10, [`Profile::reach_mm`]).
    #[error("{at}: the {head} blade would reach {blade_mm}, outside the machine's limits")]
    OutsideEnvelope {
        at: At,
        head: HeadKind,
        blade_mm: MachineMm,
    },
    /// The carriage would leave its travel ([`Profile::travel_mm`]).
    #[error("{at}: the carriage would reach {carriage_mm}, outside its travel")]
    OutsideTravel { at: At, carriage_mm: CarriageMm },
    /// A cut runs over a clamp pad ([`Profile::pads`]) the program hasn't turned out of the way
    /// (the `clamp` op's `turned`): the head would hit it.
    #[error(
        "op {op}: the segment to point {point} runs over clamp {pad}; turn it out of the way \
         and name it in the clamp op's `turned`"
    )]
    OnClamp {
        op: usize,
        point: usize,
        pad: String,
    },
    /// A `clamp` op changes which pads are turned while the board is clamped: a pad is turned
    /// by hand with the clamps open, so the clamps must be released (`side: none`) and a
    /// `pause` given to the operator first.
    #[error(
        "op {op}: the clamp pads can only be turned with the clamps open: release them and \
         pause first"
    )]
    TurnedWhileClamped { op: usize },
    /// A `clamp` op turns a pad the profile doesn't have.
    #[error("op {op}: there is no clamp {pad} to turn")]
    UnknownClamp { op: usize, pad: String },
    /// On a machine without clamp pads, a cut comes nearer the board's left or front edge than
    /// [`Profile::clamp_border_mm`]: the head would hit the clamps.
    #[error(
        "op {op}: point {point} at {x:.2}, {y:.2} mm is within {border_x} mm of the left edge or \
         {border_y} mm of the front edge, where the head would hit the clamps"
    )]
    NearClamps {
        op: usize,
        point: usize,
        x: f64,
        y: f64,
        border_x: f64,
        border_y: f64,
    },
}

/// Lower a program for a machine. The carriage is taken to start at the park.
pub fn lower(program: &Program, profile: &Profile) -> Result<Job, Error> {
    let m = &program.material;
    let thickness = program.board.thickness_mm;
    if !is_pct(m.speed_pct) || !(thickness.is_finite() && thickness > 0.0) {
        return Err(Error::BadNumber { at: At::Job });
    }
    let mut l = Lowering {
        profile,
        program,
        caps: profile.capabilities,
        clamps: 0,
        outputs: profile.neutral_outputs,
        head: None,
        carriage: profile.park_steps,
        blade_deg: 0.0,
        settings: HashMap::new(),
        streaming: false,
        out: Vec::new(),
        pads: profile.pads(),
        turned: Vec::new(),
        released: true,
        may_turn: true,
    };
    let r = profile.radius_slowdown;
    for (reg, v) in [
        (Radius::Min, r.min_radius_steps),
        (Radius::Max, r.max_radius_steps),
        (Radius::LowerPct, r.lower_pct),
        (Radius::UpperPct, r.upper_pct),
        (Radius::FasterCount, r.faster_count),
        (Radius::SlowerCount, r.slower_count),
        (Radius::Steepness, r.steepness),
    ] {
        l.set(Setting::Radius(reg, v));
    }
    l.set(Setting::StartSpeed(profile.start_speed));
    l.set(Setting::BrakeAngle {
        entry: ENTRY,
        degrees: profile.brake_angle_deg,
    });
    if l.caps.travel_entry {
        l.set(Setting::Speed {
            entry: TRAVEL_ENTRY,
            value: profile.speed(profile.travel_speed_pct),
        });
    }
    let setup = std::mem::take(&mut l.out);

    let mut ops = Vec::with_capacity(program.ops.len());
    for (i, op) in program.ops.iter().enumerate() {
        match op {
            Op::Clamp(c) => {
                if let Some(pad) = c
                    .turned
                    .iter()
                    .find(|t| !l.pads.iter().any(|p| &p.name == *t))
                {
                    return Err(Error::UnknownClamp {
                        op: i,
                        pad: pad.clone(),
                    });
                }
                // Pads are turned by hand with the clamps open: before the program, or in a pause
                // after a release.
                let same = c.turned.len() == l.turned.len()
                    && c.turned.iter().all(|t| l.turned.contains(t));
                if !same && !l.may_turn {
                    return Err(Error::TurnedWhileClamped { op: i });
                }
                l.turned.clone_from(&c.turned);
                l.released = c.side == ClampSide::None;
                l.may_turn &= l.released;
                l.clamp(c.side);
            }
            Op::Cut(c) => l.cut(i, c)?,
            Op::Move(mv) => {
                let kind = l.head.unwrap_or(HeadKind::Straight);
                let head = profile.head(kind);
                let p = mv.to_mm;
                if !p.iter().all(|v| v.is_finite()) {
                    return Err(Error::BadNumber { at: At::Op(i) });
                }
                let z = (profile.scale.z_steps_per_deg * l.blade_deg) as i32;
                let deg = l.blade_deg;
                let steps = frame::steps(profile, head, p);
                l.check(At::Op(i), kind, steps)?;
                l.travel(steps, z, deg);
            }
            Op::Pause(p) => {
                l.may_turn |= l.released;
                l.idle();
                l.out.push(Step::Hold {
                    message: p.message.clone(),
                    resume: p.resume,
                });
            }
        }
        if !l.caps.streams() {
            l.idle();
        }
        ops.push(std::mem::take(&mut l.out));
    }

    l.outputs(l.clamps | profile.neutral_outputs);
    let park = profile.park_steps;
    let carriage_mm = profile.carriage_mm(park);
    if !profile.travel_mm().contains(carriage_mm) {
        return Err(Error::OutsideTravel {
            at: At::Park,
            carriage_mm,
        });
    }
    l.travel(park, 0, 0.0);
    l.outputs(profile.neutral_outputs);
    l.idle();
    Ok(Job {
        setup,
        ops,
        finish: l.out,
    })
}

/// A manual park: both heads up, the clamps as `clamps` (output bits), and head-up travel to
/// the park. (A job parks at its end by itself.)
pub fn park(profile: &Profile, clamps: u8) -> Vec<Step> {
    let p = profile;
    let [x, y] = p.park_steps;
    vec![
        Step::Send(Command::SetOutputs(clamps | p.neutral_outputs)),
        Step::Send(Command::Set(Setting::DeadTimeMs(0))),
        Step::Send(Command::Set(Setting::RampMs(p.ramp_ms(p.accel.travel)))),
        Step::Send(Command::Set(Setting::Speed {
            entry: ENTRY,
            value: p.speed(p.travel_speed_pct),
        })),
        Step::Send(Command::Vector {
            speed_entry: ENTRY,
            x,
            y,
            z: 0,
        }),
        Step::WaitIdle,
    ]
}

/// How far short of the V-groove stub the push starts and ends, mm (GMC's 3 cm).
const VGROOVE_RUN_UP_MM: f64 = 30.0;

/// Set the bevel head's V-groove switch to normal cutting, as GMC does after homing
/// ([GMC]; protocol reference §4.1): heads up, the
/// carriage at X `x` and the blade at the switch's normal angle, travel to 3 cm short of the
/// stub, push on to it and come back. At GMC's speeds: the travel speed throughout, the travel
/// ramp there, and for the push and back the fast ramp its rule gives a 3 cm move (`travel`).
/// The push goes past the envelope; nothing else here does. Empty if there is no such switch.
pub fn vgroove_reset(profile: &Profile, clamps: u8, x: i32) -> Vec<Step> {
    let p = profile;
    let Some(switch) = p.vgroove_switch else {
        return Vec::new();
    };
    let z = frame::blade_steps(p, &p.bevel, 0.0, switch.normal_deg);
    let short = switch.y_steps - (VGROOVE_RUN_UP_MM * p.scale.y_steps_per_mm) as i32;
    let ramp = |accel| Step::Send(Command::Set(Setting::RampMs(p.ramp_ms(accel))));
    let to = |y| {
        Step::Send(Command::Vector {
            speed_entry: ENTRY,
            x,
            y,
            z,
        })
    };
    vec![
        Step::Send(Command::SetOutputs(clamps | p.neutral_outputs)),
        Step::Send(Command::Set(Setting::DeadTimeMs(0))),
        ramp(p.accel.travel),
        Step::Send(Command::Set(Setting::Speed {
            entry: ENTRY,
            value: p.speed(p.travel_speed_pct),
        })),
        to(short),
        Step::WaitIdle,
        ramp(p.accel.fast_turn),
        to(switch.y_steps),
        Step::WaitIdle,
        to(short),
        Step::WaitIdle,
    ]
}

/// A speed in % that the machine can run: over 0, at most 100.
fn is_pct(v: f64) -> bool {
    v > 0.0 && v <= 100.0
}

/// How far a direction is off the nearest axis, as a share of 45°: 0 along an axis, 1 on the
/// diagonal (GMC's `DiagonalRampeSmoother` [GMC]).
fn off_axis(deg: f64) -> f64 {
    let m = deg.rem_euclid(90.0);
    let d = if m > 45.0 { 90.0 - m } else { m } / 45.0;
    if d < 1e-5 { 0.0 } else { d }
}

struct Lowering<'a> {
    profile: &'a Profile,
    program: &'a Program,
    caps: Capabilities,
    clamps: u8,
    outputs: u8,
    /// The head the last cut used.
    head: Option<HeadKind>,
    /// Where the carriage last was, steps, and the blade's angle, degrees: GMC chooses each
    /// travel's ramp from how far both have to go.
    carriage: [i32; 2],
    blade_deg: f64,
    /// Each setting as last sent, so an unchanged one isn't sent again.
    settings: HashMap<SettingKey, Setting>,
    /// Stream commands sent since the last wait until idle.
    streaming: bool,
    out: Vec<Step>,
    /// The clamp pads, and those the last `clamp` op turned out of the way.
    pads: Vec<Pad>,
    turned: Vec<String>,
    /// The clamps are open (the last `clamp` op released them, or there was none yet).
    released: bool,
    /// The operator can turn pads now: before any clamping, or in a pause since a release.
    may_turn: bool,
}

impl Lowering<'_> {
    /// Send a command; anything outside a stream first waits for the stream to end.
    fn send(&mut self, c: Command) {
        let stream = c.required_mode() == Some(Mode::Cutting);
        if !stream {
            self.idle();
        }
        self.streaming |= stream;
        self.out.push(Step::Send(c));
    }

    fn idle(&mut self) {
        if self.streaming {
            self.out.push(Step::WaitIdle);
            self.streaming = false;
        }
    }

    fn set(&mut self, s: Setting) {
        if self.settings.get(&s.key()) != Some(&s) {
            self.settings.insert(s.key(), s);
            self.send(Command::Set(s));
        }
    }

    /// The outputs, in mode 1.
    fn outputs(&mut self, bits: u8) {
        if bits != self.outputs {
            self.send(Command::SetOutputs(bits));
            self.outputs = bits;
        }
    }

    /// A head moving, then settling: in the stream (`A`, `W`) or in mode 1 with a host wait.
    fn head_to(&mut self, bits: u8, settle: u32) {
        if bits == self.outputs {
            return;
        }
        if self.caps.stream_head {
            self.send(Command::StreamOutputs(bits));
            self.send(Command::StreamDwellMs(settle));
            self.outputs = bits;
        } else {
            self.outputs(bits);
            self.out.push(Step::Settle(settle));
        }
    }

    fn clamp(&mut self, side: ClampSide) {
        let c = self.profile.clamps;
        self.clamps = match side {
            ClampSide::Left => c.left,
            ClampSide::Right => c.right,
            ClampSide::Both => c.left | c.right,
            ClampSide::None => 0,
        };
        self.outputs(self.clamps | self.profile.neutral_outputs);
    }

    /// Head-up travel to `steps`, the blade turning to `z` on the way. In GMC's form a short
    /// travel with a large turn of the blade takes the faster ramp, by its rule for a serial
    /// port [GMC]; in one stream it runs at the travel entry,
    /// with the stream's ramp.
    fn travel(&mut self, steps: [i32; 2], z: i32, deg: f64) {
        let p = self.profile;
        if self.caps.travel_entry {
            self.send(Command::StreamFlags(StreamFlags {
                no_dead_time: true,
                speed_override: None,
            }));
            self.send(Command::Vector {
                speed_entry: TRAVEL_ENTRY,
                x: steps[0],
                y: steps[1],
                z,
            });
        } else {
            let len_cm = p.carriage_mm(self.carriage).distance(p.carriage_mm(steps)) / 10.0;
            let turn = (deg - self.blade_deg).abs();
            let fast = (len_cm < 1.0 && turn > 22.5)
                || ((1.0..3.3).contains(&len_cm) && turn < 10.0)
                || len_cm <= 0.01;
            self.set(Setting::DeadTimeMs(0));
            self.set(Setting::RampMs(p.ramp_ms(if fast {
                p.accel.fast_turn
            } else {
                p.accel.travel
            })));
            self.set(Setting::Speed {
                entry: ENTRY,
                value: p.speed(p.travel_speed_pct),
            });
            self.send(Command::Vector {
                speed_entry: ENTRY,
                x: steps[0],
                y: steps[1],
                z,
            });
        }
        self.carriage = steps;
        self.blade_deg = deg;
    }

    /// `kind`'s blade, with the carriage at `steps`, is within its reach (V10).
    fn check(&self, at: At, kind: HeadKind, steps: [i32; 2]) -> Result<(), Error> {
        let p = self.profile;
        let blade_mm = p.head(kind).blade(p.carriage_mm(steps));
        if p.reach_mm(kind).contains(blade_mm) {
            Ok(())
        } else {
            Err(Error::OutsideEnvelope {
                at,
                head: kind,
                blade_mm,
            })
        }
    }

    fn cut(&mut self, op: usize, cut: &Cut) -> Result<(), Error> {
        let p = self.profile;
        let m = &self.program.material;
        let kind = cut.head;
        let head = *p.head(kind);
        let speed = cut.speed_pct.unwrap_or(m.speed_pct);
        if !is_pct(speed) {
            return Err(Error::BadNumber { at: At::Op(op) });
        }
        if self.pads.is_empty() {
            let [bx, by] = p.clamp_border_mm;
            // A hair of slack: GMC's own layouts cut exactly on the border.
            if let Some((point, &[x, y])) = cut
                .path_mm
                .iter()
                .enumerate()
                .find(|&(_, &[x, y])| x < bx - 1e-6 || y < by - 1e-6)
            {
                return Err(Error::NearClamps {
                    op,
                    point,
                    x,
                    y,
                    border_x: bx,
                    border_y: by,
                });
            }
        } else if let Some((point, pad)) = pads_hit(&self.pads, &cut.path_mm)
            .into_iter()
            .find(|(_, pad)| !self.turned.contains(&pad.name))
        {
            return Err(Error::OnClamp {
                op,
                point,
                pad: pad.name.clone(),
            });
        }
        let strokes = strokes(op, &cut.path_mm, p.break_angle_deg)?;
        let closed = strokes.len() == 1 && {
            let h = frame::headings(&cut.path_mm);
            (h[h.len() - 1] - h[0]).abs() >= 360.0 - SMOOTH_TURN_DEG
        };
        let offsets = &m.blade_angle_offset_deg;
        let trim = match kind {
            HeadKind::Bevel if closed => offsets.bevel_round.unwrap_or(offsets.bevel),
            HeadKind::Bevel => offsets.bevel,
            HeadKind::Straight => offsets.straight,
        };
        if !trim.is_finite() {
            return Err(Error::BadNumber { at: At::Op(op) });
        }
        let down = if closed {
            head.down_closed.unwrap_or(head.down)
        } else {
            head.down
        };

        self.head = Some(kind);
        let settle = m.head_settle_ms;
        // On a closed curve the blade may still be coming down as the cut starts.
        let settle_down = if closed {
            head.closed_settle_ms.unwrap_or(settle)
        } else {
            settle
        };
        let cut_ramp = p.ramp_ms(p.accel.cut);
        let feed = p.speed(speed);
        for stroke in strokes {
            let dirs = frame::directions(stroke);
            let steps: Vec<[i32; 2]> = stroke.iter().map(|&q| frame::steps(p, &head, q)).collect();
            let z: Vec<i32> = dirs
                .iter()
                .map(|&d| frame::blade_steps(p, &head, trim, d))
                .collect();
            let deg = |k: usize| frame::blade_deg(&head, trim, dirs[k]);
            let last = stroke.len() - 1;
            let (ramp, start) = self.start(kind, dirs[0], cut_ramp);
            let dead = if stroke.len() > 2 {
                p.dead_time_ms.curve
            } else {
                p.dead_time_ms.line
            };
            let settings = |l: &mut Self| {
                l.set(Setting::DeadTimeMs(dead));
                l.set(Setting::RampMs(ramp.unwrap_or(cut_ramp)));
                l.set(Setting::StartSpeed(start.unwrap_or(p.start_speed)));
                l.set(Setting::Speed {
                    entry: ENTRY,
                    value: feed,
                });
            };

            for q in &steps {
                self.check(At::Op(op), kind, *q)?;
            }
            // Settings can't change inside a stream: in one stream they come first; in GMC's
            // form, the cutting ramp before the head goes down and the rest after.
            if self.caps.travel_entry || self.caps.stream_head {
                if self.caps.travel_entry {
                    settings(self);
                }
                self.travel(steps[0], z[0], deg(0));
                if !self.caps.travel_entry {
                    settings(self);
                }
                self.head_to(self.clamps | down, settle_down);
            } else {
                self.travel(steps[0], z[0], deg(0));
                self.set(Setting::RampMs(cut_ramp));
                self.head_to(self.clamps | down, settle_down);
                settings(self);
            }
            let points: Vec<[i32; 3]> = (1..=last)
                .map(|k| [steps[k][0], steps[k][1], z[k]])
                .collect();
            if self.caps.compact_paths {
                for c in Command::path(ENTRY, &points) {
                    self.send(c);
                }
            } else {
                for q in points {
                    self.send(Command::Vector {
                        speed_entry: ENTRY,
                        x: q[0],
                        y: q[1],
                        z: q[2],
                    });
                }
            }
            if let Some(lift) = head.lift {
                self.head_to(self.clamps | lift, settle);
            }
            self.head_to(self.clamps | head.up, settle);
            // GMC's form puts the usual ramp and start speed back for the travel that follows.
            if !self.caps.travel_entry {
                self.set(Setting::RampMs(cut_ramp));
                self.set(Setting::StartSpeed(p.start_speed));
            }
            self.carriage = steps[last];
            self.blade_deg = deg(last);
        }
        Ok(())
    }

    /// The cut's ramp and start speed where they differ from the usual (protocol reference
    /// §3.3): the soft start by the board's thickness ([`SoftStart`]: GMC's for the straight
    /// head, a gentler start speed for the bevel), and GMC's longer ramp for a cut whose first
    /// segment runs off the axes [GMC].
    fn start(&self, kind: HeadKind, first_deg: f64, cut_ramp: u16) -> (Option<u16>, Option<u32>) {
        let p = self.profile;
        let cut = f64::from(cut_ramp);
        let d = p.diagonal_ramp_factor.map_or(0.0, |_| off_axis(first_deg));
        let diagonal = || {
            let factor = p.diagonal_ramp_factor.filter(|&f| f > 0.001).unwrap_or(1.0);
            (cut * (1.0 + d) / factor).min(f64::from(p.max_diagonal_ramp_ms))
        };
        let diagonal_ramp = (d != 0.0).then(diagonal);
        let s = p.soft_start;
        let t = self.program.board.thickness_mm;
        match kind {
            HeadKind::Bevel => {
                let f = ((t - s.thin_mm) / (s.thick_mm - s.thin_mm)).clamp(0.0, 1.0);
                let start = f64::from(p.start_speed);
                let speed = (s.bevel && f > 0.0)
                    .then(|| (start - (start - f64::from(s.slow_speed)) * f) as u32);
                (diagonal_ramp.map(|r| r as u16), speed)
            }
            HeadKind::Straight => {
                if t < s.thin_mm {
                    return (diagonal_ramp.map(|r| r as u16), Some(s.thin_speed));
                }
                let f = if t > s.thick_mm {
                    1.0
                } else {
                    t / (s.thin_mm + s.thick_mm)
                };
                let max = f64::from(p.accel.max_ramp_ms);
                let mut ramp = cut + (max - cut) * f;
                if d != 0.0 && ramp < cut * (1.0 + d) {
                    ramp = diagonal();
                }
                let start = f64::from(p.start_speed);
                let speed = start - (start - f64::from(s.slow_speed)) * f;
                (Some(ramp as u16), Some(speed as u32))
            }
        }
    }
}

/// A cut's path split at its corners into strokes with no corner inside: in GMC's form the
/// head goes up at a corner, the blade turns, and the head goes down again (PROGRAM_FORMAT Q7).
fn strokes(op: usize, path: &[Pt], break_deg: f64) -> Result<Vec<&[Pt]>, Error> {
    if path.len() < 2 {
        return Err(Error::TooShort { op });
    }
    if !path.iter().flatten().all(|v| v.is_finite()) {
        return Err(Error::BadNumber { at: At::Op(op) });
    }
    for (k, w) in path.windows(2).enumerate() {
        if (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]) < MIN_SEGMENT_MM {
            return Err(Error::ZeroSegment { op, point: k + 1 });
        }
    }
    let h = frame::headings(path);
    let mut out = Vec::new();
    let mut from = 0;
    for k in 1..h.len() {
        let turn = (h[k] - h[k - 1]).abs();
        if turn > break_deg {
            return Err(Error::SharpCorner {
                op,
                point: k,
                turn_deg: turn,
            });
        }
        if turn > SMOOTH_TURN_DEG {
            out.push(&path[from..=k]);
            from = k;
        }
    }
    out.push(&path[from..]);
    Ok(out)
}

/// The pads a cut along `path` passes over, each with the first point it reaches past one, in
/// path order. A path of one point is a point.
pub fn pads_hit<'p>(pads: &'p [Pad], path: &[Pt]) -> Vec<(usize, &'p Pad)> {
    let mut out: Vec<(usize, &Pad)> = Vec::new();
    let segments = path
        .windows(2)
        .enumerate()
        .map(|(k, w)| (k + 1, w[0], w[1]))
        .chain(path.first().filter(|_| path.len() == 1).map(|&a| (0, a, a)));
    for (point, a, b) in segments {
        for pad in pads {
            if pad.hit(a, b) && !out.iter().any(|(_, p)| p.name == pad.name) {
                out.push((point, pad));
            }
        }
    }
    out
}
