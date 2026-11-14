//! The cuts' order, and the program's ops.

use machine::{CarriageMm, Profile, frame, pads_hit};
use program::{Clamp, Cut, Head, Op};
use route::{Job, Problem, Travel, Way};

use crate::{Error, Keep, Options, Part, Planned, Shape, Sheet, Side};

/// Boxes this close count as touching.
const SAME_MM: f64 = 1e-6;

/// Box `a` lies inside box `b`, and isn't the same box.
fn inside(a: [f64; 4], b: [f64; 4]) -> bool {
    let within = |a: [f64; 4], b: [f64; 4]| {
        a[0] >= b[0] - SAME_MM
            && a[1] >= b[1] - SAME_MM
            && a[2] <= b[2] + SAME_MM
            && a[3] <= b[3] + SAME_MM
    };
    within(a, b) && !within(b, a)
}

/// Which cuts must come before each: whatever lies inside a contour before its cuts, and each
/// kept rectangle's left edge after its other three, so it is cut free on the clamped side, as
/// GMC ends a mount. Through shared edges this works leftward, toward the clamped side: a
/// mount's left edge is its left neighbour's right, which comes before that one's left. Each
/// rule runs from a cut to one further left or inside, so the rules can't go round in a circle.
/// (Top and right before bottom and left can: on a grid, the line between two rows is the top
/// of one row and the bottom of the other, and crosses the line between two columns.)
/// The clean-up comes after every other cut.
pub(crate) fn precedence(sheet: &Sheet, cuts: &[Planned]) -> Vec<Vec<usize>> {
    let boxes: Vec<[f64; 4]> = sheet.contours.iter().map(|c| c.box_mm).collect();
    let n = cuts.len();
    let mut after = vec![Vec::new(); n];
    for (j, outer) in cuts.iter().enumerate() {
        for (i, inner) in cuts.iter().enumerate() {
            let within = inner.parts.iter().filter_map(|p| p.contour()).any(|p| {
                outer
                    .parts
                    .iter()
                    .filter_map(|q| q.contour())
                    .any(|q| inside(boxes[p], boxes[q]))
            });
            if i != j && within {
                after[j].push(i);
            }
        }
    }
    for (k, c) in sheet.contours.iter().enumerate() {
        if c.keep != Keep::In || c.shape != Shape::Rect {
            continue;
        }
        let edge = |cut: &Planned, sides: &[Side]| cut.parts.iter().any(|p| p.is_edge(k, sides));
        let others: Vec<usize> = (0..n)
            .filter(|&i| edge(&cuts[i], &[Side::Top, Side::Right, Side::Bottom]))
            .collect();
        for j in (0..n).filter(|&j| edge(&cuts[j], &[Side::Left])) {
            after[j].extend(others.iter().copied().filter(|&i| i != j));
        }
    }
    // The clean-up after every piece's cut, so the board is whole while they are cut.
    let cleanup = |c: &Planned| c.parts.iter().all(|p| matches!(p, Part::Cleanup { .. }));
    for j in (0..n).filter(|&j| cleanup(&cuts[j])) {
        after[j].extend((0..n).filter(|&i| !cleanup(&cuts[i])));
    }
    after
}

/// The ops: the clamps (with any pads the cuts can't keep off turned away), then the cuts in
/// the order that travels least within the rules. Also the pads turned.
pub(crate) fn ops(
    sheet: &Sheet,
    cuts: &[Planned],
    profile: &Profile,
    options: &Options,
) -> Result<Pads, Error> {
    let pads = profile.pads();
    // Each way's pads; of the ways the head can reach, those that keep off the pads, if any
    // do.
    let hits: Vec<Vec<Vec<usize>>> = cuts
        .iter()
        .map(|c| {
            c.ways
                .iter()
                .map(|path| {
                    pads_hit(&pads, path)
                        .into_iter()
                        .filter_map(|(_, pad)| pads.iter().position(|p| p.name == pad.name))
                        .collect()
                })
                .collect()
        })
        .collect();
    let mut usable: Vec<Vec<usize>> = Vec::with_capacity(cuts.len());
    for (c, ways) in cuts.iter().zip(&hits) {
        let reach: Vec<usize> = (0..ways.len())
            .filter(|&w| reachable(profile, c.head, &c.ways[w]))
            .collect();
        if reach.is_empty() {
            return Err(match c.parts[0] {
                Part::Edge { contour, .. } => Error::OutOfReach {
                    contour,
                    head: c.head,
                },
                Part::Cleanup { line } => Error::CleanupOutOfReach { line },
            });
        }
        let clear: Vec<usize> = reach
            .iter()
            .copied()
            .filter(|&w| ways[w].is_empty())
            .collect();
        usable.push(if clear.is_empty() { reach } else { clear });
    }
    let after = precedence(sheet, cuts);
    let jobs: Vec<Job> = cuts
        .iter()
        .zip(&usable)
        .zip(after)
        .map(|((c, use_ways), after)| Job {
            ways: use_ways
                .iter()
                .map(|&w| {
                    let path = &c.ways[w];
                    Way {
                        entry: path[0],
                        exit: path[path.len() - 1],
                    }
                })
                .collect(),
            flip: c.flip.clone().filter(|_| use_ways.len() == c.ways.len()),
            after,
        })
        .collect();
    let park = frame::board_point(profile, &profile.straight, profile.park_steps);
    let travel = Travel {
        speed: profile.speed(profile.travel_speed_pct) as f64 / profile.scale.x_steps_per_mm,
        accel: profile.accel.travel,
    };
    let problem = Problem::new(jobs, park, travel)?.with_end(Some(park))?;
    let route = route::optimise(&problem, None, &options.route);

    let mut turned: Vec<usize> = Vec::new();
    let mut cut_ops = Vec::with_capacity(cuts.len());
    for &j in &route.order {
        let w = usable[j][route.ways[j]];
        for &pad in &hits[j][w] {
            if !turned.contains(&pad) {
                turned.push(pad);
            }
        }
        let c = &cuts[j];
        cut_ops.push(Op::Cut(Cut {
            id: Some(id(sheet, &c.parts)),
            label: Some(label(sheet, &c.parts)),
            head: c.head,
            path_mm: c.ways[w].clone(),
            speed_pct: None,
        }));
    }
    turned.sort_unstable();
    let turned: Vec<String> = turned.into_iter().map(|p| pads[p].name.clone()).collect();
    // Something must still hold the board.
    let b = &sheet.board;
    let held_by: Vec<String> = profile
        .pads_on(b.width_mm, b.height_mm)
        .into_iter()
        .map(|p| p.name)
        .filter(|name| !turned.contains(name))
        .collect();
    if held_by.is_empty() && !profile.pads().is_empty() {
        return Err(Error::Unclamped { turned });
    }
    let mut ops = vec![Op::Clamp(Clamp {
        id: None,
        label: None,
        side: profile.clamp_side(sheet.board.width_mm),
        turned: turned.clone(),
    })];
    ops.extend(cut_ops);
    Ok(Pads {
        ops,
        turned,
        held_by,
    })
}

/// The ops, and the clamp pads turned away and still holding the board.
pub(crate) struct Pads {
    pub ops: Vec<Op>,
    pub turned: Vec<String>,
    pub held_by: Vec<String>,
}

/// How far inside its limits the carriage stays, mm, so rounding to motor steps can't take it
/// out.
const REACH_MARGIN_MM: f64 = 0.05;

/// `head` can follow `path`: the carriage stays inside the machine's limits (V10). The straight
/// head sits 6.4 mm right of the carriage on the 601, so it can't reach the board's left
/// 6.2 mm.
fn reachable(profile: &Profile, head: Head, path: &[[f64; 2]]) -> bool {
    let h = profile.head(head);
    let e = &profile.envelope_mm;
    let m = REACH_MARGIN_MM;
    path.iter().all(|&p| {
        let CarriageMm([x, y]) = h.carriage(frame::machine_mm(profile, p));
        x >= e.x[0] + m && x <= e.x[1] - m && y >= e.y[0] + m && y <= e.y[1] - m
    })
}

/// What a cut cuts, for the run log: each contour's id (or `c` and its number) and the side,
/// or `cleanup` and the line's number; `+` between the parts sharing it.
fn id(sheet: &Sheet, parts: &[Part]) -> String {
    let names: Vec<String> = parts
        .iter()
        .map(|&p| match p {
            Part::Edge { contour, side } => {
                let name = sheet.contours[contour]
                    .id
                    .clone()
                    .unwrap_or_else(|| format!("c{}", contour + 1));
                format!("{name}.{}", side.name())
            }
            Part::Cleanup { line } => format!("cleanup.{}", line + 1),
        })
        .collect();
    names.join("+")
}

/// What a cut cuts, for the operator: "Opening, left edge / Mount 2, right edge".
fn label(sheet: &Sheet, parts: &[Part]) -> String {
    let names: Vec<String> = parts
        .iter()
        .map(|&p| match p {
            Part::Edge { contour, side } => {
                let name = sheet.contours[contour]
                    .label
                    .clone()
                    .unwrap_or_else(|| format!("contour {}", contour + 1));
                match side {
                    Side::Round => name,
                    side => format!("{name}, {} edge", side.name()),
                }
            }
            Part::Cleanup { line } => format!("clean-up line {}", line + 1),
        })
        .collect();
    names.join(" / ")
}
