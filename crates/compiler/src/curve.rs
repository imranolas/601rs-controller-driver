//! Ovals: one closed cut round the ellipse in a box.
//!
//! The line is the ellipse's true offset, moved by the head's edge so the kept edge lands on
//! the ellipse. It is flattened by chord tolerance (PROGRAM_FORMAT §4.2): each chord no longer
//! than √(8·r·e), r·θ or 10 mm at the curve's radius r there, so the chords are long where it is
//! gentle and short where it is tight. After the lap it cuts on over its start (the overlap),
//! for the head's overlap time at the cut's feed, a segment counting as longer in proportion to
//! how much tighter it is than the radius below which the controller slows; toward the end it
//! drifts off the line into the waste (the run-out). Both as GMC does them (protocol reference
//! §7.1).

use std::f64::consts::{FRAC_PI_2, TAU};

use program::Head;

use crate::blade::{HeadCut, Kept};
use crate::{Contour, Keep, Part, Planned, Side};

/// The most a chord may stray from the curve, mm.
const CHORD_TOLERANCE_MM: f64 = 0.02;
/// The most the path may turn at one point, degrees.
const MAX_TURN_DEG: f64 = 2.0;
const MAX_CHORD_MM: f64 = 10.0;
/// How finely a lap is measured, to place its points.
const SAMPLES: usize = 4096;

/// How to cut a closed curve with one head.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Curve<'a> {
    pub head: Head,
    pub cut: &'a HeadCut,
    /// The cut's feed, mm/s.
    pub feed_mm_s: f64,
    /// The radius below which the controller slows on a curve, mm.
    pub slowdown_below_mm: f64,
    /// How many places it may start, the first at the top.
    pub starts: usize,
}

/// The line: an ellipse's offset, `offset` mm outward, as `t` goes round anticlockwise.
#[derive(Debug, Clone, Copy)]
struct Line {
    centre: [f64; 2],
    a: f64,
    b: f64,
    offset: f64,
}

impl Line {
    /// How fast the ellipse's point moves with `t`, squared.
    fn speed2(&self, t: f64) -> f64 {
        let (s, c) = t.sin_cos();
        (self.a * s).powi(2) + (self.b * c).powi(2)
    }

    /// The ellipse's radius of curvature at `t`.
    fn ellipse_radius(&self, t: f64) -> f64 {
        self.speed2(t).powf(1.5) / (self.a * self.b)
    }

    /// The outward unit normal.
    fn normal(&self, t: f64) -> [f64; 2] {
        let (s, c) = t.sin_cos();
        let (nx, ny) = (self.b * c, self.a * s);
        let len = nx.hypot(ny);
        [nx / len, ny / len]
    }

    fn point(&self, t: f64, out: f64) -> [f64; 2] {
        let (s, c) = t.sin_cos();
        let n = self.normal(t);
        let o = self.offset + out;
        [
            self.centre[0] + self.a * c + o * n[0],
            self.centre[1] + self.b * s + o * n[1],
        ]
    }

    /// The line's radius of curvature at `t`: the ellipse's, offset.
    fn radius(&self, t: f64) -> f64 {
        self.ellipse_radius(t) + self.offset
    }

    /// How fast the line's point moves with `t`: the ellipse's, scaled as the offset scales
    /// the radius.
    fn speed(&self, t: f64) -> f64 {
        self.speed2(t).sqrt() * self.radius(t) / self.ellipse_radius(t)
    }
}

/// The longest chord at radius `r`.
fn chord(r: f64) -> f64 {
    (8.0 * r * CHORD_TOLERANCE_MM)
        .sqrt()
        .min(r * MAX_TURN_DEG.to_radians())
        .min(MAX_CHORD_MM)
}

fn dist(p: [f64; 2], q: [f64; 2]) -> f64 {
    (q[0] - p[0]).hypot(q[1] - p[1])
}

impl Curve<'_> {
    /// The cut round contour `k`'s oval, one way per start; none if the line's offset would
    /// turn it inside out.
    pub(crate) fn oval(&self, k: usize, c: &Contour) -> Option<Planned> {
        let [x0, y0, x1, y1] = c.box_mm;
        let window = c.keep == Keep::Out;
        // A bevel keeps the board on its left: windows clockwise, pieces anticlockwise. The
        // straight head cuts closed curves clockwise, as GMC's does.
        let clockwise = self.head == Head::Straight || window;
        let kept_left = clockwise == window;
        let kept = if kept_left { Kept::Left } else { Kept::Right };
        // The normal to the left of travel is outward going clockwise.
        let toward_left = if clockwise { 1.0 } else { -1.0 };
        let line = Line {
            centre: [(x0 + x1) / 2.0, (y0 + y1) / 2.0],
            a: (x1 - x0) / 2.0,
            b: (y1 - y0) / 2.0,
            offset: toward_left * self.cut.shift(kept),
        };
        let tightest = line.a.min(line.b).powi(2) / line.a.max(line.b);
        if !(line.a > 0.0 && line.b > 0.0 && tightest + line.offset > 0.0) {
            return None;
        }
        // The waste is right of travel when the board is kept on the left.
        let waste = if kept_left { -toward_left } else { toward_left };
        let ways = (0..self.starts)
            .map(|s| {
                let dir = if clockwise { -1.0 } else { 1.0 };
                let start = FRAC_PI_2 + dir * TAU * s as f64 / self.starts as f64;
                self.path(&line, start, dir, waste)
            })
            .collect();
        Some(Planned {
            head: self.head,
            parts: vec![Part::Edge {
                contour: k,
                side: Side::Round,
            }],
            ways,
            flip: None,
        })
    }

    /// The lap from `start`, `dir` round (−1 clockwise), and the overlap with its run-out
    /// `waste` outward.
    fn path(&self, line: &Line, start: f64, dir: f64, waste: f64) -> Vec<[f64; 2]> {
        // How many chords the lap needs, accumulated along it.
        let dt = dir * TAU / SAMPLES as f64;
        let density = |t: f64| line.speed(t) / chord(line.radius(t));
        let mut total = vec![0.0; SAMPLES + 1];
        for j in 1..=SAMPLES {
            let (ta, tb) = (start + dt * (j - 1) as f64, start + dt * j as f64);
            total[j] = total[j - 1] + 0.5 * (density(ta) + density(tb)) * dt.abs();
        }
        let n = (total[SAMPLES].ceil() as usize).max(8);
        let step = total[SAMPLES] / n as f64;
        let mut ts = Vec::with_capacity(n + 1);
        let mut j = 0;
        for k in 0..n {
            let want = step * k as f64;
            while total[j + 1] < want {
                j += 1;
            }
            let f = (want - total[j]) / (total[j + 1] - total[j]);
            ts.push(start + dt * (j as f64 + f));
        }
        ts.push(start + dir * TAU);
        let mut path: Vec<[f64; 2]> = ts.iter().map(|&t| line.point(t, 0.0)).collect();
        // The lap closes exactly.
        path[n] = path[0];
        let lap: f64 = path.windows(2).map(|w| dist(w[0], w[1])).sum();

        // The overlap: the lap again from its start, until its time at the feed is spent.
        let want = self.feed_mm_s * self.cut.overlap_ms / 1000.0;
        let mut over: Vec<(f64, f64)> = Vec::new();
        let (mut counted, mut along) = (0.0, 0.0);
        let mut i = 0;
        while counted < want {
            let (ta, tb) = (ts[i], ts[i + 1]);
            let len = dist(path[i], path[i + 1]);
            let r = line.radius((ta + tb) / 2.0);
            counted += len * (self.slowdown_below_mm / r).max(1.0);
            along += len;
            over.push((tb, along));
            i = (i + 1) % n;
        }
        // The run-out ramps up over the whole overlap, or its last half when the overlap is
        // more than a third of the lap.
        let from = if along > lap / 3.0 { along / 2.0 } else { 0.0 };
        let run_out = self.cut.run_out_mm * waste;
        path.extend(over.into_iter().map(|(t, at)| {
            let f = ((at - from) / (along - from)).clamp(0.0, 1.0);
            line.point(t, run_out * f)
        }));
        path
    }
}
