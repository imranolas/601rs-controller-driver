//! Board millimetres to motor steps, and the blade's direction along a path.

use crate::profile::{Head, Profile};

/// A point in board millimetres (PROGRAM_FORMAT §2: origin at the board's bottom-left corner,
/// +Y away from the operator).
pub type Pt = [f64; 2];

/// The carriage steps that put `head`'s blade over board point `p`.
pub fn steps(profile: &Profile, head: &Head, p: Pt) -> [i32; 2] {
    let m = machine_mm(profile, p);
    let s = profile.scale;
    [
        ((m[0] - head.offset_mm[0]) * s.x_steps_per_mm).round() as i32,
        ((m[1] - head.offset_mm[1]) * s.y_steps_per_mm).round() as i32,
    ]
}

/// The board point under `head`'s blade with the carriage at `steps`.
pub fn board_point(profile: &Profile, head: &Head, steps: [i32; 2]) -> Pt {
    let c = profile.carriage_mm(steps);
    let o = profile.board_origin_mm;
    [
        c[0] + head.offset_mm[0] - o[0],
        c[1] + head.offset_mm[1] - o[1],
    ]
}

/// Board point `p` in machine millimetres.
pub fn machine_mm(profile: &Profile, p: Pt) -> [f64; 2] {
    let o = profile.board_origin_mm;
    [o[0] + p[0], o[1] + p[1]]
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
