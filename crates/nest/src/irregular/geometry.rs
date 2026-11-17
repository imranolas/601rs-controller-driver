//! The geometry of the irregular scheme, in two kinds.
//!
//! Exact, on integers: whether a mount fits in a bin ([`contains`]). Every placement is tested
//! this way before it is kept, so a layout never depends on rounding.
//!
//! Floating point, for the slide: the bin's obstacles as convex polygons ([`obstacles`]), the
//! places a mount's corner can't go because of one ([`blocked`]), the path the corner follows
//! as the mount rests on a support ([`Path`]), and the slide along it ([`slide`]).

use crate::Len;

use super::Point;

/// Positions closer than this to an obstacle's edge count as touching it, not overlapping.
const EPS: f64 = 1e-6;

/// Twice the signed area of a ring: positive anticlockwise.
pub(crate) fn area2(ring: &[Point]) -> i64 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            a[0] as i64 * b[1] as i64 - b[0] as i64 * a[1] as i64
        })
        .sum()
}

/// The open rectangle `r` (x, y, w, h) and the segment `ab` share a point. They do when their
/// bounding boxes overlap and the segment's line has corners of the rectangle strictly on both
/// sides: past the rectangle, along the line, the segment is outside its box.
fn crosses(a: Point, b: Point, r: [Len; 4]) -> bool {
    let [x0, y0, w, h] = r;
    let (x1, y1) = (x0 + w, y0 + h);
    if a[0].max(b[0]) <= x0 || a[0].min(b[0]) >= x1 || a[1].max(b[1]) <= y0 || a[1].min(b[1]) >= y1
    {
        return false;
    }
    let (dx, dy) = ((b[0] - a[0]) as i64, (b[1] - a[1]) as i64);
    let side = |x: Len, y: Len| dx * (y - a[1]) as i64 - dy * (x - a[0]) as i64;
    let s = [side(x0, y0), side(x1, y0), side(x1, y1), side(x0, y1)];
    s.iter().any(|&v| v > 0) && s.iter().any(|&v| v < 0)
}

/// The point (`px2`, `py2`) / 2 is inside `ring` (even-odd; never on it, here).
fn inside_ring(ring: &[Point], px2: i64, py2: i64) -> bool {
    let n = ring.len();
    let mut inside = false;
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        let (ax, ay) = (2 * a[0] as i64, 2 * a[1] as i64);
        let (bx, by) = (2 * b[0] as i64, 2 * b[1] as i64);
        if (ay > py2) != (by > py2) {
            // Is the point left of the edge at its height? Cross-multiplied, by the edge's sign.
            let (lhs, rhs, den) = ((px2 - ax) * (by - ay), (py2 - ay) * (bx - ax), by - ay);
            if (den > 0 && lhs < rhs) || (den < 0 && lhs > rhs) {
                inside = !inside;
            }
        }
    }
    inside
}

/// The rectangle `r` (x, y, w, h) lies in the region bounded by `rings` (an outline and its
/// holes), its edges allowed to touch theirs. No edge of the region passes through the
/// rectangle's inside, so its centre settles it.
pub(crate) fn contains(rings: &[Vec<Point>], r: [Len; 4]) -> bool {
    for ring in rings {
        let n = ring.len();
        if (0..n).any(|i| crosses(ring[i], ring[(i + 1) % n], r)) {
            return false;
        }
    }
    let (cx2, cy2) = (2 * r[0] as i64 + r[2] as i64, 2 * r[1] as i64 + r[3] as i64);
    rings
        .iter()
        .filter(|ring| inside_ring(ring, cx2, cy2))
        .count()
        % 2
        == 1
}

/// Rectangles (x, y, w, h) overlap: more than touching.
pub(crate) fn overlap(a: [Len; 4], b: [Len; 4]) -> bool {
    a[0] < b[0] + b[2] && b[0] < a[0] + a[2] && a[1] < b[1] + b[3] && b[1] < a[1] + a[3]
}

/// Everything around a region that a mount mustn't overlap, as convex polygons (anticlockwise):
/// in each band between two vertices' heights, the gaps between the free intervals and out to
/// `margin` past the bounding box; and the strips below and above it. PackingSolver splits
/// the same into trapezoids with horizontal tops and bottoms.
pub(crate) fn obstacles(rings: &[Vec<Point>], bbox: [Len; 4], margin: f64) -> Vec<Vec<[f64; 2]>> {
    let [x0, y0, x1, y1] = bbox.map(|v| v as f64);
    let (xl, xr) = (x0 - margin, x1 + margin);
    let rect = |a: f64, b: f64, c: f64, d: f64| vec![[a, b], [c, b], [c, d], [a, d]];
    let mut out = vec![rect(xl, y0 - margin, xr, y0), rect(xl, y1, xr, y1 + margin)];
    let mut ys: Vec<Len> = rings.iter().flatten().map(|p| p[1]).collect();
    ys.sort_unstable();
    ys.dedup();
    for band in ys.windows(2) {
        let (ya, yb) = (band[0], band[1]);
        let xs = crossings(rings, ya, yb);
        // The free intervals are between edges 0 and 1, 2 and 3, …; the obstacles around them.
        let mut left = (xl, xl);
        let (ya, yb) = (ya as f64, yb as f64);
        let gap = |l: (f64, f64), r: (f64, f64), out: &mut Vec<Vec<[f64; 2]>>| {
            if r.0 - l.0 > EPS || r.1 - l.1 > EPS {
                out.push(vec![[l.0, ya], [r.0, ya], [r.1, yb], [l.1, yb]]);
            }
        };
        for pair in xs.chunks_exact(2) {
            gap(left, pair[0], &mut out);
            left = pair[1];
        }
        gap(left, (xr, xr), &mut out);
    }
    out
}

/// The edges across the band `ya`..`yb` (between two vertices' heights, so each edge crosses
/// all of it or none), as their x at its bottom and top, left to right.
fn crossings(rings: &[Vec<Point>], ya: Len, yb: Len) -> Vec<(f64, f64)> {
    let mut xs: Vec<(f64, f64)> = Vec::new();
    for ring in rings {
        let n = ring.len();
        for i in 0..n {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            if a[1].min(b[1]) <= ya && a[1].max(b[1]) >= yb && a[1] != b[1] {
                let at = |y: Len| {
                    a[0] as f64 + (y - a[1]) as f64 * (b[0] - a[0]) as f64 / (b[1] - a[1]) as f64
                };
                xs.push((at(ya), at(yb)));
            }
        }
    }
    xs.sort_by(|p, q| (p.0 + p.1).total_cmp(&(q.0 + q.1)));
    xs
}

/// The largest axis-aligned rectangles (x, y, w, h) inside the region, with their tops and
/// bottoms at vertices' heights: where a guillotine layout can go. Each is checked exactly, and
/// none lies inside another.
pub(crate) fn rectangles(rings: &[Vec<Point>]) -> Vec<[Len; 4]> {
    let mut ys: Vec<Len> = rings.iter().flatten().map(|p| p[1]).collect();
    ys.sort_unstable();
    ys.dedup();
    // Per band: the whole x-intervals free over all its height (an edge's furthest point in).
    let bands: Vec<Vec<(Len, Len)>> = ys
        .windows(2)
        .map(|band| {
            crossings(rings, band[0], band[1])
                .chunks_exact(2)
                .filter_map(|pair| {
                    let lo = (pair[0].0.max(pair[0].1) - 1e-9).ceil() as Len;
                    let hi = (pair[1].0.min(pair[1].1) + 1e-9).floor() as Len;
                    (hi > lo).then_some((lo, hi))
                })
                .collect()
        })
        .collect();
    let mut out = Vec::new();
    for i in 0..bands.len() {
        let mut free = bands[i].clone();
        for j in i..bands.len() {
            if j > i {
                free = free
                    .iter()
                    .flat_map(|&(a, b)| {
                        bands[j]
                            .iter()
                            .map(move |&(c, d)| (a.max(c), b.min(d)))
                            .filter(|(lo, hi)| hi > lo)
                    })
                    .collect();
            }
            if free.is_empty() {
                break;
            }
            out.extend(
                free.iter()
                    .map(|&(lo, hi)| [lo, ys[i], hi - lo, ys[j + 1] - ys[i]]),
            );
        }
    }
    out.retain(|&r| contains(rings, r));
    out.sort_unstable();
    out.dedup();
    let within = |r: &[Len; 4], s: &[Len; 4]| {
        r != s
            && r[0] >= s[0]
            && r[1] >= s[1]
            && r[0] + r[2] <= s[0] + s[2]
            && r[1] + r[3] <= s[1] + s[3]
    };
    let all = out.clone();
    out.retain(|r| !all.iter().any(|s| within(r, s)));
    out
}

/// The chains of edges a mount can rest on: edges going right with the free side above them
/// (`rings` turned so that the free side is on every edge's left), joined while their slopes
/// fall, so that each chain is convex and a flat bottom resting on it follows a simple path.
pub(crate) fn supports(rings: &[Vec<Point>]) -> Vec<Vec<[f64; 2]>> {
    let mut out: Vec<Vec<[f64; 2]>> = Vec::new();
    for ring in rings {
        let n = ring.len();
        let edge = |i: usize| (ring[i % n], ring[(i + 1) % n]);
        let goes_right = |i: usize| {
            let (a, b) = edge(i);
            b[0] > a[0]
        };
        // Start after an edge that isn't a support, so no chain wraps round.
        let Some(first) = (0..n).find(|&i| !goes_right(i)) else {
            continue;
        };
        let mut chain: Vec<Point> = Vec::new();
        let mut flush = |chain: &mut Vec<Point>| {
            if chain.len() >= 2 {
                out.push(chain.iter().map(|p| [p[0] as f64, p[1] as f64]).collect());
            }
            chain.clear();
        };
        for i in first + 1..=first + n {
            if !goes_right(i) {
                flush(&mut chain);
                continue;
            }
            let (a, b) = edge(i);
            let falls = chain.len() >= 2 && {
                let p = chain[chain.len() - 2];
                let (dx1, dy1) = ((a[0] - p[0]) as i64, (a[1] - p[1]) as i64);
                let (dx2, dy2) = ((b[0] - a[0]) as i64, (b[1] - a[1]) as i64);
                dy2 * dx1 <= dy1 * dx2
            };
            if !falls {
                flush(&mut chain);
                chain.push(a);
            }
            chain.push(b);
        }
        flush(&mut chain);
    }
    out
}

/// An open convex polygon as half-planes: a point `p` is inside when `n · p < c` for every
/// (`n`, `c`), `n` a unit vector.
pub(crate) type Planes = Vec<([f64; 2], f64)>;

/// The convex hull, anticlockwise (Andrew's monotone chain).
fn hull(mut pts: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    pts.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let half = |pts: &mut dyn Iterator<Item = [f64; 2]>| {
        let mut h: Vec<[f64; 2]> = Vec::new();
        for p in pts {
            while h.len() >= 2 && cross(h[h.len() - 2], h[h.len() - 1], p) <= 0.0 {
                h.pop();
            }
            h.push(p);
        }
        h.pop();
        h
    };
    let mut lower = half(&mut pts.iter().copied());
    lower.extend(half(&mut pts.iter().rev().copied()));
    lower
}

/// Where a `w` × `h` mount's bottom-left corner can't go without overlapping `obstacle`: the
/// obstacle grown left by `w` and down by `h` (its Minkowski sum with the mount turned
/// round), which is convex too.
pub(crate) fn blocked(obstacle: &[[f64; 2]], w: f64, h: f64) -> Planes {
    let pts = obstacle
        .iter()
        .flat_map(|v| [[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]].map(|r| [v[0] - r[0], v[1] - r[1]]))
        .collect();
    let hull = hull(pts);
    let n = hull.len();
    (0..n)
        .map(|i| {
            let (p, q) = (hull[i], hull[(i + 1) % n]);
            let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
            let len = dx.hypot(dy);
            let normal = [dy / len, -dx / len];
            (normal, normal[0] * p[0] + normal[1] * p[1])
        })
        .collect()
}

/// Where a `w` × `h` mount's corner can't go because of a placed mount `r` (x, y, w, h).
fn blocked_by_mount(r: [Len; 4], w: Len, h: Len) -> [([f64; 2], f64); 4] {
    let [x, y, rw, rh] = r.map(|v| v as f64);
    let (w, h) = (w as f64, h as f64);
    [
        ([1.0, 0.0], x + rw),
        ([-1.0, 0.0], -(x - w)),
        ([0.0, 1.0], y + rh),
        ([0.0, -1.0], -(y - h)),
    ]
}

fn inside(planes: &[([f64; 2], f64)], p: [f64; 2]) -> bool {
    planes
        .iter()
        .all(|(n, c)| n[0] * p[0] + n[1] * p[1] < c - EPS)
}

/// How far along `d` (in multiples of it) a point `p` inside `planes` gets out.
fn exit(planes: &[([f64; 2], f64)], p: [f64; 2], d: [f64; 2]) -> f64 {
    let len = d[0].hypot(d[1]);
    planes
        .iter()
        .filter_map(|(n, c)| {
            let along = n[0] * d[0] + n[1] * d[1];
            (along > 1e-12 * len).then(|| (c - (n[0] * p[0] + n[1] * p[1])) / along)
        })
        .fold(f64::INFINITY, f64::min)
}

/// The path of a mount's bottom-left corner as its flat bottom, `w` wide, rests on a chain
/// (left to right, slopes falling) and slides along it: PackingSolver's slide, for a
/// supported part that is one edge. It starts with the mount's right corner on the chain's
/// first point, then follows the rising edges (the right corner on them), the mount's own
/// width, and the falling edges (the left corner on them).
#[derive(Debug, Clone)]
pub(crate) struct Path {
    start: [f64; 2],
    steps: Vec<[f64; 2]>,
}

impl Path {
    pub(crate) fn new(chain: &[[f64; 2]], w: f64) -> Path {
        let edges: Vec<[f64; 2]> = chain
            .windows(2)
            .map(|p| [p[1][0] - p[0][0], p[1][1] - p[0][1]])
            .collect();
        let mut steps: Vec<[f64; 2]> = edges.iter().copied().filter(|e| e[1] > 0.0).collect();
        steps.push([w, 0.0]);
        steps.extend(edges.iter().copied().filter(|e| e[1] <= 0.0));
        steps.retain(|s| s[0] != 0.0 || s[1] != 0.0);
        Path {
            start: [chain[0][0] - w, chain[0][1]],
            steps,
        }
    }
}

/// How far along its path a mount is: a step, and a fraction of it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Pos {
    step: usize,
    base: [f64; 2],
    u: f64,
}

impl Pos {
    pub(crate) fn start(path: &Path) -> Pos {
        Pos {
            step: 0,
            base: path.start,
            u: 0.0,
        }
    }

    /// The mount's bottom-left corner.
    pub(crate) fn at(&self, path: &Path) -> [f64; 2] {
        match path.steps.get(self.step) {
            Some(d) => [self.base[0] + self.u * d[0], self.base[1] + self.u * d[1]],
            None => self.base,
        }
    }

    /// Move on until the corner is out of `planes`. False if the path ends first.
    fn advance(&mut self, path: &Path, planes: &[([f64; 2], f64)]) -> bool {
        loop {
            let d = path.steps[self.step];
            let t = exit(planes, self.at(path), d);
            if self.u + t <= 1.0 {
                self.u += t;
                return true;
            }
            self.base = [self.base[0] + d[0], self.base[1] + d[1]];
            self.step += 1;
            self.u = 0.0;
            if self.step == path.steps.len() {
                return false;
            }
            if !inside(planes, self.at(path)) {
                return true;
            }
        }
    }
}

/// Slide a `w` × `h` mount along `path` from `pos` to the first place where it overlaps
/// neither the bin's obstacles (`blocked`, for this size) nor the mounts placed (`mounts`),
/// with its corner no further right than `x_limit`. Each move is the least that gets it out
/// of one obstacle, so it stops at the first such place whatever the obstacles' order. False
/// if there is none.
pub(crate) fn slide(
    path: &Path,
    pos: &mut Pos,
    blocked: &[Planes],
    mounts: &[[Len; 4]],
    (w, h): (Len, Len),
    x_limit: f64,
) -> bool {
    if path.steps.is_empty() {
        return false;
    }
    let out_of = |planes: &[([f64; 2], f64)], pos: &mut Pos, moved: &mut bool| {
        if inside(planes, pos.at(path)) {
            if !pos.advance(path, planes) {
                return false;
            }
            *moved = true;
        }
        pos.at(path)[0] <= x_limit + EPS
    };
    loop {
        let mut moved = false;
        for planes in blocked {
            if !out_of(planes, pos, &mut moved) {
                return false;
            }
        }
        for &m in mounts {
            if !out_of(&blocked_by_mount(m, w, h), pos, &mut moved) {
                return false;
            }
        }
        if !moved {
            return pos.at(path)[0] <= x_limit + EPS;
        }
    }
}

/// The area a partial layout of one bin has used up, as PackingSolver's guides count it: the
/// bin's height times the distance from its left side to the furthest left edge of any mount,
/// and the mounts' skyline to the right of that.
pub(crate) fn used_area(bbox: [Len; 4], mounts: &[[Len; 4]]) -> f64 {
    let Some(xs) = mounts.iter().map(|m| m[0]).max() else {
        return 0.0;
    };
    let mut area = (xs - bbox[0]) as f64 * (bbox[3] - bbox[1]) as f64;
    let mut ys: Vec<Len> = mounts.iter().flat_map(|m| [m[1], m[1] + m[3]]).collect();
    ys.sort_unstable();
    ys.dedup();
    for band in ys.windows(2) {
        let right = mounts
            .iter()
            .filter(|m| m[1] <= band[0] && m[1] + m[3] >= band[1])
            .map(|m| m[0] + m[2])
            .max();
        if let Some(r) = right.filter(|&r| r > xs) {
            area += (r - xs) as f64 * (band[1] - band[0]) as f64;
        }
    }
    area
}
