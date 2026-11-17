//! Board millimetres to motor steps, and the blade's direction along a path.
//!
//! Three frames, all in millimetres: the **board**'s ([`Pt`], the program's); the
//! **machine**'s ([`MachineMm`]: where a blade is, from home); and the **carriage**'s
//! ([`CarriageMm`]: where the carriage is, the frame of the steps). The soft limits are the
//! blade's ([`Profile::reach_mm`]). A blade
//! is at the carriage plus its head's offset ([`Head::carriage`], [`Head::blade`]).

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::profile::{Head, Profile};

/// A point in board millimetres (PROGRAM_FORMAT §2: origin at the board's bottom-left corner,
/// +Y away from the operator).
pub type Pt = [f64; 2];

/// Where a blade is, mm from home. In JSON, `[x, y]`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct MachineMm(pub [f64; 2]);

/// Where the carriage is, mm from home.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarriageMm(pub [f64; 2]);

impl CarriageMm {
    pub fn distance(self, other: Self) -> f64 {
        let ([x0, y0], [x1, y1]) = (self.0, other.0);
        (x1 - x0).hypot(y1 - y0)
    }
}

fn write_mm(f: &mut fmt::Formatter<'_>, [x, y]: [f64; 2]) -> fmt::Result {
    write!(f, "({x:.1}, {y:.1}) mm")
}

impl fmt::Display for CarriageMm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write_mm(f, self.0)
    }
}

impl fmt::Display for MachineMm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write_mm(f, self.0)
    }
}

/// The carriage steps that put `head`'s blade over board point `p`.
pub fn steps(profile: &Profile, head: &Head, p: Pt) -> [i32; 2] {
    profile.carriage_steps(head.carriage(machine_mm(profile, p)))
}

/// The board point under `head`'s blade with the carriage at `steps`.
pub fn board_point(profile: &Profile, head: &Head, steps: [i32; 2]) -> Pt {
    let MachineMm([x, y]) = head.blade(profile.carriage_mm(steps));
    let MachineMm([ox, oy]) = profile.board_origin_mm;
    [x - ox, y - oy]
}

/// Board point `p` on the machine.
pub fn machine_mm(profile: &Profile, [x, y]: Pt) -> MachineMm {
    let MachineMm([ox, oy]) = profile.board_origin_mm;
    MachineMm([ox + x, oy + y])
}

/// The direction of travel from `a` to `b`: degrees clockwise from +X, so travelling towards −Y
/// is 90° (protocol reference §3.2). In [0, 360).
pub fn heading(a: Pt, b: Pt) -> f64 {
    let h = (a[1] - b[1]).atan2(b[0] - a[0]).to_degrees();
    if h < 0.0 { h + 360.0 } else { h }
}

/// The segments' headings, each within 180° of the one before, so the blade turns the short
/// way and keeps counting past a revolution along a curve. The first is in [0, 360).
pub fn headings(path: &[Pt]) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::with_capacity(path.len().saturating_sub(1));
    for w in path.windows(2) {
        let mut h = heading(w[0], w[1]);
        if let Some(&prev) = out.last() {
            h += 360.0 * ((prev - h) / 360.0).round();
        }
        out.push(h);
    }
    out
}

/// The blade's direction at each point of a path with no corners (PROGRAM_FORMAT §4.2): the
/// segment's heading at either end, and halfway between the two segments' headings at each
/// point between, which is the true curve's tangent there.
pub fn directions(path: &[Pt]) -> Vec<f64> {
    let h = headings(path);
    let (Some(&first), Some(&last)) = (h.first(), h.last()) else {
        return Vec::new();
    };
    let mut out = Vec::with_capacity(path.len());
    out.push(first);
    out.extend(h.windows(2).map(|w| (w[0] + w[1]) / 2.0));
    out.push(last);
    out
}

/// The Z steps for a blade direction: GMC's formula, `trunc(scale × (direction − mount +
/// trim))` (protocol reference §3.2, PROGRAM_FORMAT Q4).
pub fn blade_steps(profile: &Profile, head: &Head, trim_deg: f64, direction_deg: f64) -> i32 {
    (profile.scale.z_steps_per_deg * blade_deg(head, trim_deg, direction_deg)) as i32
}

/// The blade angle in degrees on the Z axis for a direction of travel.
pub fn blade_deg(head: &Head, trim_deg: f64, direction_deg: f64) -> f64 {
    direction_deg - head.blade_mount_deg + trim_deg
}
