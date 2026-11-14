//! Rectangles: a bevel rectangle's four cuts, and straight edges merged into strokes; and the
//! stretches of the clean-up lines that no piece's edge cuts.

use std::cmp::Ordering;

use program::Head;

use crate::blade::{HeadCut, Kept};
use crate::{Contour, Keep, Part, Planned, Side};

/// Edges closer than this are on one line, and intervals closer than this touch.
const SAME_MM: f64 = 1e-6;

/// The path from `a` to `b`, run on by `[before, after]` along its direction.
fn run_on(a: [f64; 2], b: [f64; 2], [before, after]: [f64; 2]) -> Vec<[f64; 2]> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = dx.hypot(dy);
    let u = [dx / len, dy / len];
    vec![
        [a[0] - before * u[0], a[1] - before * u[1]],
        [b[0] + after * u[0], b[1] + after * u[1]],
    ]
}

/// A bevel rectangle's four cuts, each one way. The board is kept on the blade's left: a
/// window is cut clockwise (from its right edge down, as GMC does), a kept piece
/// anticlockwise. Each line moves toward the waste by the head's left edge, so the back edge
/// lands on the box, and runs on past the corners where the lines meet.
pub(crate) fn bevel(contour: usize, c: &Contour, cut: &HeadCut, thickness_mm: f64) -> Vec<Planned> {
    let [x0, y0, x1, y1] = c.box_mm;
    // Along the normal to the left, the line moves by `shift`: toward the waste, inside a
    // window, outside a piece.
    let d = -cut.shift(Kept::Left);
    let into = if c.keep == Keep::Out { d } else { -d };
    let (x0, y0, x1, y1) = (x0 + into, y0 + into, x1 - into, y1 - into);
    let edges: [(Side, [f64; 2], [f64; 2]); 4] = match c.keep {
        Keep::Out => [
            (Side::Right, [x1, y1], [x1, y0]),
            (Side::Bottom, [x1, y0], [x0, y0]),
            (Side::Left, [x0, y0], [x0, y1]),
            (Side::Top, [x0, y1], [x1, y1]),
        ],
        Keep::In => [
            (Side::Right, [x1, y0], [x1, y1]),
            (Side::Top, [x1, y1], [x0, y1]),
            (Side::Left, [x0, y1], [x0, y0]),
            (Side::Bottom, [x0, y0], [x1, y0]),
        ],
    };
    let ends = cut.run_on(thickness_mm);
    edges
        .into_iter()
        .map(|(side, a, b)| Planned {
            head: Head::Bevel,
            parts: vec![Part::Edge { contour, side }],
            ways: vec![run_on(a, b, ends)],
            flip: None,
        })
        .collect()
}

/// One straight edge of a rectangle.
#[derive(Debug, Clone, Copy)]
struct Edge {
    /// Along x (at a y), or along y (at an x).
    horizontal: bool,
    at: f64,
    from: f64,
    to: f64,
    /// The board is kept on its + side (above, or to the right), and on its − side. A clean-up
    /// line keeps both: an offcut on one side, and the waste or another offcut on the other.
    plus: bool,
    minus: bool,
    part: Part,
}

/// Edges on one line that overlap or touch, cut as one.
#[derive(Debug, Clone)]
struct Stroke {
    horizontal: bool,
    at: f64,
    from: f64,
    to: f64,
    plus: bool,
    minus: bool,
    parts: Vec<Part>,
}

/// Shorter than this, a stretch of a clean-up line is rounding, not a cut (V4's shortest step).
const SHORTEST_MM: f64 = 0.01;

/// The straight rectangles' strokes, and the clean-up's: the stretches of the clean-up lines
/// that no piece's stroke runs along. Each runs +X or +Y if `away_from_clamps`, else either
/// way. Where a clean-up stretch meets a piece's stroke end to end, one runs on over the other.
///
/// The clean-up's strokes are cut as far as the head reaches, `reach` (`[x0, y0, x1, y1]` on the
/// board); the stretches beyond it are returned too, to be broken by hand.
pub(crate) fn straight(
    contours: &[Contour],
    straight: &[usize],
    cleanup: &[[f64; 4]],
    cut: &HeadCut,
    thickness_mm: f64,
    away_from_clamps: bool,
    reach: [f64; 4],
) -> (Vec<Planned>, Vec<Planned>, Vec<[f64; 4]>) {
    let mut edges = Vec::with_capacity(4 * straight.len());
    for &k in straight {
        let c = &contours[k];
        let [x0, y0, x1, y1] = c.box_mm;
        let piece = c.keep == Keep::In;
        let part = |side| Part::Edge { contour: k, side };
        for (horizontal, at, from, to, plus, side) in [
            (true, y0, x0, x1, piece, Side::Bottom),
            (true, y1, x0, x1, !piece, Side::Top),
            (false, x0, y0, y1, piece, Side::Left),
            (false, x1, y0, y1, !piece, Side::Right),
        ] {
            edges.push(Edge {
                horizontal,
                at,
                from,
                to,
                plus,
                minus: !plus,
                part: part(side),
            });
        }
    }
    let pieces = merge(edges);
    let mut stretches = Vec::new();
    for (line, &[x0, y0, x1, y1]) in cleanup.iter().enumerate() {
        let horizontal = (y0 - y1).abs() <= SAME_MM;
        let (at, a, b) = if horizontal {
            (y0, x0, x1)
        } else {
            (x0, y0, y1)
        };
        // What's left of it once the pieces' strokes along it are taken away.
        let mut left = vec![(a.min(b), a.max(b))];
        for s in pieces
            .iter()
            .filter(|s| s.horizontal == horizontal && (s.at - at).abs() <= SAME_MM)
        {
            left = left
                .into_iter()
                .flat_map(|(f, t)| [(f, t.min(s.from)), (f.max(s.to), t)])
                .filter(|(f, t)| t - f >= SHORTEST_MM)
                .collect();
        }
        stretches.extend(left.into_iter().map(|(from, to)| Edge {
            horizontal,
            at,
            from,
            to,
            plus: true,
            minus: true,
            part: Part::Cleanup { line },
        }));
    }
    let ends = cut.run_on(thickness_mm);
    let planned = |s: Stroke| -> Planned {
        let forward = way(&s, cut, ends, true);
        if away_from_clamps {
            Planned {
                head: Head::Straight,
                parts: s.parts,
                ways: vec![forward],
                flip: None,
            }
        } else {
            let back = way(&s, cut, ends, false);
            Planned {
                head: Head::Straight,
                parts: s.parts,
                ways: vec![forward, back],
                flip: Some(vec![1, 0]),
            }
        }
    };
    // The clean-up, within reach.
    let mut uncut = Vec::new();
    let mut cleanup_cuts = Vec::new();
    for s in merge(stretches) {
        let (lo, hi) = if s.horizontal {
            (reach[0], reach[2])
        } else {
            (reach[1], reach[3])
        };
        let (across_lo, across_hi) = if s.horizontal {
            (reach[1], reach[3])
        } else {
            (reach[0], reach[2])
        };
        let line = |from: f64, to: f64| {
            if s.horizontal {
                [from, s.at, to, s.at]
            } else {
                [s.at, from, s.at, to]
            }
        };
        let within = s.at >= across_lo && s.at <= across_hi;
        let (from, to) = (s.from.max(lo), s.to.min(hi));
        if !within || to - from < SHORTEST_MM {
            uncut.push(line(s.from, s.to));
            continue;
        }
        if s.from < lo {
            uncut.push(line(s.from, lo));
        }
        if s.to > hi {
            uncut.push(line(hi, s.to));
        }
        // Along the line: x for a horizontal one. Each way's run-on stops at the reach too.
        let k = usize::from(!s.horizontal);
        let mut p = planned(s);
        for w in &mut p.ways {
            for pt in w.iter_mut() {
                pt[k] = pt[k].clamp(lo, hi);
            }
        }
        cleanup_cuts.push(p);
    }
    let pieces = pieces.into_iter().map(planned).collect();
    (pieces, cleanup_cuts, uncut)
}

/// Edges on one line that overlap or touch, as one stroke each.
fn merge(mut edges: Vec<Edge>) -> Vec<Stroke> {
    // The lines: edges sorted by where they lie, those within SAME_MM of a line's first on it.
    edges.sort_by(|a, b| {
        (a.horizontal, a.at)
            .partial_cmp(&(b.horizontal, b.at))
            .unwrap_or(Ordering::Equal)
    });
    let mut lines: Vec<Vec<Edge>> = Vec::new();
    for e in edges {
        match lines.last_mut() {
            Some(line) if line[0].horizontal == e.horizontal && e.at - line[0].at <= SAME_MM => {
                line.push(e);
            }
            _ => lines.push(vec![e]),
        }
    }
    // Along each line, edges that overlap or touch are one stroke.
    let mut merged: Vec<Stroke> = Vec::new();
    for mut line in lines {
        line.sort_by(|a, b| a.from.partial_cmp(&b.from).unwrap_or(Ordering::Equal));
        let start = merged.len();
        for e in line {
            if let Some(s) = merged[start..].last_mut()
                && e.from <= s.to + SAME_MM
            {
                s.to = s.to.max(e.to);
                s.plus |= e.plus;
                s.minus |= e.minus;
                if !s.parts.contains(&e.part) {
                    s.parts.push(e.part);
                }
                continue;
            }
            merged.push(Stroke {
                horizontal: e.horizontal,
                at: e.at,
                from: e.from,
                to: e.to,
                plus: e.plus,
                minus: e.minus,
                parts: vec![e.part],
            });
        }
    }
    merged
}

/// A stroke's path, `forward` (+X or +Y) or back.
fn way(s: &Stroke, cut: &HeadCut, ends: [f64; 2], forward: bool) -> Vec<[f64; 2]> {
    // The + side is left of travel going +X or −Y, right of it going −X or +Y; the normal to
    // the left points + going +X or −Y.
    let plus_is_left = forward == s.horizontal;
    let kept = match (s.plus, s.minus) {
        (true, true) => Kept::Both,
        (true, false) if plus_is_left => Kept::Left,
        (true, false) => Kept::Right,
        (false, _) if plus_is_left => Kept::Right,
        (false, _) => Kept::Left,
    };
    let normal = if plus_is_left { 1.0 } else { -1.0 };
    let at = s.at + normal * cut.shift(kept);
    let point = |along: f64| {
        if s.horizontal {
            [along, at]
        } else {
            [at, along]
        }
    };
    let (a, b) = if forward {
        (point(s.from), point(s.to))
    } else {
        (point(s.to), point(s.from))
    };
    run_on(a, b, ends)
}
