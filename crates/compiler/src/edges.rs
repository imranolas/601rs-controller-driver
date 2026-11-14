//! Rectangles: a bevel rectangle's four cuts, and straight edges merged into strokes.

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
            parts: vec![Part { contour, side }],
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
    /// The board is kept on its + side (above, or to the right).
    plus: bool,
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

/// The straight rectangles' strokes. Each runs +X or +Y if `away_from_clamps`, else either way.
pub(crate) fn strokes(
    contours: &[Contour],
    straight: &[usize],
    cut: &HeadCut,
    thickness_mm: f64,
    away_from_clamps: bool,
) -> Vec<Planned> {
    let mut edges = Vec::with_capacity(4 * straight.len());
    for &k in straight {
        let c = &contours[k];
        let [x0, y0, x1, y1] = c.box_mm;
        let piece = c.keep == Keep::In;
        let part = |side| Part { contour: k, side };
        edges.extend([
            Edge {
                horizontal: true,
                at: y0,
                from: x0,
                to: x1,
                plus: piece,
                part: part(Side::Bottom),
            },
            Edge {
                horizontal: true,
                at: y1,
                from: x0,
                to: x1,
                plus: !piece,
                part: part(Side::Top),
            },
            Edge {
                horizontal: false,
                at: x0,
                from: y0,
                to: y1,
                plus: piece,
                part: part(Side::Left),
            },
            Edge {
                horizontal: false,
                at: x1,
                from: y0,
                to: y1,
                plus: !piece,
                part: part(Side::Right),
            },
        ]);
    }
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
                s.minus |= !e.plus;
                s.parts.push(e.part);
                continue;
            }
            merged.push(Stroke {
                horizontal: e.horizontal,
                at: e.at,
                from: e.from,
                to: e.to,
                plus: e.plus,
                minus: !e.plus,
                parts: vec![e.part],
            });
        }
    }
    let ends = cut.run_on(thickness_mm);
    merged
        .into_iter()
        .map(|s| {
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
        })
        .collect()
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
