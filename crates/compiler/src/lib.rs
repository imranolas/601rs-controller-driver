//! A laid-out sheet to a program (PROGRAM_FORMAT v1): the step between the nester and the
//! runner.
//!
//! A [`Sheet`] is the board as it lies on the machine, face down, in the board frame
//! (PROGRAM_FORMAT §2), and the contours to cut on it: each a rectangle or an ellipse in an
//! axis-aligned box (its back edge, as Bevel's design crate gives it), with a tool and the side
//! the board is kept on. [`compile`] turns them into cuts and puts them in order:
//!
//! - **Bevel rectangles**: four cuts, a window's clockwise and a kept piece's anticlockwise, so
//!   the board is always kept on the blade's left; each line placed by the tool model
//!   ([`Blade`]) so the back edge lands on the design's, and run on past its corners until it
//!   meets the next cut on both faces (as fitted: 0.4 mm before and 3.8 mm past on 1.5 mm
//!   board, as GMC's, each growing by the bevel's run across the board, 1.14 mm per mm).
//! - **Straight rectangles**: their edges on one line that overlap or touch are one stroke, so
//!   mats laid edge to edge share their cuts (as the nester counts them). Each stroke runs +X or
//!   +Y, away from the clamped corner, as GMC sets the straight head on a machine without a
//!   vacuum table ([`Options::straight_away_from_clamps`]).
//! - **Ovals**: one closed cut, flattened by chord tolerance (PROGRAM_FORMAT §4.2), on past its
//!   start by the head's overlap and drifting off the line toward the waste at the end.
//! - **Order** ([`route`]): whatever lies inside a contour is cut before it (a drop-out's pieces
//!   before the window round them, windows before their mount's outline); each kept piece's
//!   left edge after its other three, so it is cut free on the clamped side, as GMC ends a
//!   mount, and a strip of mounts stays attached at the left until each is free; and, within
//!   those rules, the least head-up travel, from the park back to it.
//! - **Clamps and reach**: of the ways to cut a contour, those the head can reach (on the 601
//!   both reach the board's left and front edges; the bevel head not past about 1,488 mm), and
//!   of those the ones that keep off the
//!   clamp pads, if any do; the pads the rest run over are turned out of the way
//!   ([`Compiled::turned`], the `clamp` op's `turned`), as long as one still holds the board.
//!   Both clamps close on a board wider than half the table.
//! - **Clean-up** ([`Sheet::cleanup`], if the caller wants the waste worth keeping freed): the
//!   lines that free it share cuts with the pieces' straight edges as the nester counts them
//!   (on one line, overlapping or touching: one cut, with the piece); a line on no piece's edge
//!   is cut after every piece. Their pads are turned with the rest, before the run. Each line is cut as far as the
//!   straight head reaches; the rest is left to break by hand ([`Compiled::uncut`]).
//!
//! The program is then lowered by the machine model, which checks it against the machine, and
//! its run time estimated.

mod blade;
mod curve;
mod edges;
mod order;

pub use blade::{Blade, HeadCut};
use machine::{Estimate, Host, Profile};
use program::{Board, Head, Material, Program};
use serde::{Deserialize, Serialize};

/// A sheet to cut, as it lies on the machine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sheet {
    /// The program's id.
    pub id: String,
    /// Passed through to the program.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub meta: Option<serde_json::Value>,
    pub board: Board,
    pub material: Material,
    pub contours: Vec<Contour>,
    /// The clean-up, if the waste worth keeping is to be cut free: lines along the layout's
    /// guillotine cuts, `[x0, y0, x1, y1]` each, along x or y, cut with the straight head. A line
    /// on one line with a piece's straight edge, overlapping or touching it, is one cut with it, as
    /// the nester counts strokes; a line on no piece's edge is cut after every piece. Each cut's
    /// id names what it cuts, `cleanup.` and a line's number among them. Empty: no clean-up.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cleanup: Vec<[f64; 4]>,
}

/// One contour to cut, in the board frame, face down.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Contour {
    /// Named in its cuts' ids, so a run's progress can be traced back to it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    /// Named in its cuts' labels, for the operator.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    pub shape: Shape,
    /// Its back edge's box, `[x0, y0, x1, y1]`, mm.
    pub box_mm: [f64; 4],
    pub tool: Tool,
    pub keep: Keep,
}

/// What a contour is, in its box.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Shape {
    Rect,
    /// The ellipse inside the box.
    Oval,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Tool {
    Bevel,
    Straight,
    Vgroove,
}

/// Which side of a contour the board keeps.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Keep {
    /// The inside: a piece.
    In,
    /// The outside: a window.
    Out,
}

/// How to compile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    /// Straight cuts run only +X or +Y, away from the clamped corner, as GMC sets the straight
    /// head on a machine without a vacuum table (protocol reference §4): a board held only at its
    /// front and left edges is pulled against the clamps, not pushed. `false` lets each stroke
    /// run either way, for less travel.
    pub straight_away_from_clamps: bool,
    /// How many places round a closed curve it may start (the first at its top, as GMC's), for
    /// the route to choose from.
    pub curve_starts: usize,
    pub route: route::Options,
    pub host: Host,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            straight_away_from_clamps: true,
            curve_starts: 8,
            route: route::Options::default(),
            host: Host::default(),
        }
    }
}

/// A compiled sheet.
#[derive(Debug, Clone, PartialEq)]
pub struct Compiled {
    pub program: Program,
    pub estimate: Estimate,
    /// The clamp pads its cuts can't keep off, which the operator turns out of the way before
    /// it runs (also in the program's `clamp` op).
    pub turned: Vec<String>,
    /// The pads still holding the board once those are turned: at least one.
    pub held_by: Vec<String>,
    /// The stretches of the clean-up lines the straight head can't reach, `[x0, y0, x1, y1]`
    /// each, left for the operator to break by hand: beyond where the carriage may go (on the 601
    /// it reaches past the board's left and front edges, so none there).
    pub uncut: Vec<[f64; 4]>,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error {
    #[error("the board needs a positive width, height and thickness")]
    BadBoard,
    #[error("there is nothing to cut")]
    Empty,
    #[error("contour {contour}: its box isn't a box on the board")]
    BadContour { contour: usize },
    #[error("clean-up line {line}: it isn't a line along x or y on the board")]
    BadCleanup { line: usize },
    #[error("contour {contour}: an oval this small can't take the line's offset")]
    TooSmall { contour: usize },
    #[error(
        "the cuts run over every clamp pad holding the board ({}): it would be cut unclamped",
        turned.join(", ")
    )]
    Unclamped { turned: Vec<String> },
    #[error("contour {contour}: V-grooves need a later version of the program format")]
    Vgroove { contour: usize },
    #[error("contour {contour}: the {} head can't reach it on this machine: it is too near the board's edge", head_name(*head))]
    OutOfReach { contour: usize, head: Head },
    #[error("clean-up line {line}: the straight head can't reach it on this machine")]
    CleanupOutOfReach { line: usize },
    #[error("the cuts can't be put in order: {0}")]
    Route(#[from] route::Error),
    #[error("the program doesn't fit the machine: {0}")]
    Machine(#[from] machine::Error),
}

/// Compile a sheet for a machine.
pub fn compile(
    sheet: &Sheet,
    blade: &Blade,
    profile: &Profile,
    options: &Options,
) -> Result<Compiled, Error> {
    check(sheet)?;
    let thickness = sheet.board.thickness_mm;
    let feed = profile.speed(sheet.material.speed_pct) as f64 / profile.scale.x_steps_per_mm;
    let slowdown_mm =
        profile.radius_slowdown.max_radius_steps as f64 / profile.scale.x_steps_per_mm;
    let mut cuts = Vec::new();
    let mut straight = Vec::new();
    for (k, c) in sheet.contours.iter().enumerate() {
        match (c.tool, c.shape) {
            (Tool::Vgroove, _) => return Err(Error::Vgroove { contour: k }),
            (Tool::Bevel, Shape::Rect) => {
                cuts.extend(edges::bevel(k, c, &blade.bevel, thickness));
            }
            (Tool::Straight, Shape::Rect) => straight.push(k),
            (tool, Shape::Oval) => {
                let head = if tool == Tool::Bevel {
                    Head::Bevel
                } else {
                    Head::Straight
                };
                let curve = curve::Curve {
                    head,
                    cut: blade.head(head),
                    feed_mm_s: feed,
                    slowdown_below_mm: slowdown_mm,
                    starts: options.curve_starts.max(1),
                };
                cuts.push(curve.oval(k, c).ok_or(Error::TooSmall { contour: k })?);
            }
        }
    }
    let (strokes, cleanup_cuts, uncut) = edges::straight(
        &sheet.contours,
        &straight,
        &sheet.cleanup,
        &blade.straight,
        thickness,
        options.straight_away_from_clamps,
        reach(profile, Head::Straight),
    );
    cuts.extend(strokes);
    cuts.extend(cleanup_cuts);
    let order::Pads {
        ops,
        turned,
        held_by,
    } = order::ops(sheet, &cuts, profile, options)?;
    let program = Program {
        format: program::FORMAT.to_owned(),
        version: program::VERSION,
        id: sheet.id.clone(),
        meta: sheet.meta.clone(),
        board: sheet.board.clone(),
        material: sheet.material.clone(),
        ops,
    };
    let job = machine::lower(&program, profile)?;
    let estimate = machine::estimate(&job, profile, &options.host);
    Ok(Compiled {
        program,
        estimate,
        turned,
        held_by,
        uncut,
    })
}

/// How far inside its limits the carriage stays, mm, so rounding to motor steps can't take it out
/// (the route's check allows half as much).
const REACH_MARGIN_MM: f64 = 0.1;

/// Where on the board `head` can cut, `[x0, y0, x1, y1]`: where the carriage stays inside the
/// machine's limits (V10).
fn reach(profile: &Profile, head: Head) -> [f64; 4] {
    let [dx, dy] = profile.head(head).offset_mm;
    let machine::MachineMm([ox, oy]) = profile.board_origin_mm;
    let e = &profile.envelope_mm;
    let m = REACH_MARGIN_MM;
    [
        e.x[0] + dx - ox + m,
        e.y[0] + dy - oy + m,
        e.x[1] + dx - ox - m,
        e.y[1] + dy - oy - m,
    ]
}

/// How far a contour may reach past the board's edge: GMC's own layouts sit a fraction of a
/// millimetre over it.
const OVER_EDGE_MM: f64 = 1.0;

fn check(sheet: &Sheet) -> Result<(), Error> {
    let b = &sheet.board;
    let positive = |v: f64| v.is_finite() && v > 0.0;
    if !(positive(b.width_mm) && positive(b.height_mm) && positive(b.thickness_mm)) {
        return Err(Error::BadBoard);
    }
    if sheet.contours.is_empty() {
        return Err(Error::Empty);
    }
    for (k, c) in sheet.contours.iter().enumerate() {
        let [x0, y0, x1, y1] = c.box_mm;
        let ok = c.box_mm.iter().all(|v| v.is_finite())
            && x0 < x1
            && y0 < y1
            && x0 >= -OVER_EDGE_MM
            && y0 >= -OVER_EDGE_MM
            && x1 <= b.width_mm + OVER_EDGE_MM
            && y1 <= b.height_mm + OVER_EDGE_MM;
        if !ok {
            return Err(Error::BadContour { contour: k });
        }
    }
    for (k, &[x0, y0, x1, y1]) in sheet.cleanup.iter().enumerate() {
        let on =
            |v: f64, size: f64| v.is_finite() && v >= -OVER_EDGE_MM && v <= size + OVER_EDGE_MM;
        let (along_x, along_y) = ((y0 - y1).abs() <= 1e-6, (x0 - x1).abs() <= 1e-6);
        let ok = on(x0, b.width_mm)
            && on(x1, b.width_mm)
            && on(y0, b.height_mm)
            && on(y1, b.height_mm)
            && along_x != along_y;
        if !ok {
            return Err(Error::BadCleanup { line: k });
        }
    }
    Ok(())
}

/// What one cut cuts: an edge of a contour, or a stretch of a clean-up line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Part {
    Edge { contour: usize, side: Side },
    Cleanup { line: usize },
}

impl Part {
    pub(crate) fn contour(self) -> Option<usize> {
        match self {
            Part::Edge { contour, .. } => Some(contour),
            Part::Cleanup { .. } => None,
        }
    }

    /// An edge of contour `k` on one of `sides`.
    pub(crate) fn is_edge(self, k: usize, sides: &[Side]) -> bool {
        matches!(self, Part::Edge { contour, side } if contour == k && sides.contains(&side))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Side {
    Top,
    Right,
    Bottom,
    Left,
    /// All the way round: an oval.
    Round,
}

impl Side {
    pub(crate) fn name(self) -> &'static str {
        match self {
            Side::Top => "top",
            Side::Right => "right",
            Side::Bottom => "bottom",
            Side::Left => "left",
            Side::Round => "round",
        }
    }
}

/// A cut to make, and the ways it can be made.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Planned {
    pub head: Head,
    /// What it cuts: one contour's edge, or several where edges share a stroke.
    pub parts: Vec<Part>,
    /// Each way to make it, as its path.
    pub ways: Vec<Vec<[f64; 2]>>,
    /// For each way, the same stroke the other way round (straight lines).
    pub flip: Option<Vec<usize>>,
}

#[cfg(test)]
mod tests;

fn head_name(head: Head) -> &'static str {
    match head {
        Head::Bevel => "bevel",
        Head::Straight => "straight",
    }
}
