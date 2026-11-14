//! Where each head's cuts land and how far they run on: the numbers that turn a design's edge
//! into a cut's path.
//!
//! A cut leaves an edge on each side of its slit. Where that edge lands, from the line the
//! blade was sent along, is the head's `edge_mm`, measured on the back of the board (the side
//! the blade enters, face down, and the side Bevel's contours give). A cut also runs on past
//! the ends of its edge so that the corner is cut through on both faces: `run_on_mm`, which
//! grows with the board's thickness on the bevel head, whose blade leans into the cut.

use program::Head;

/// How one head's cuts land.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeadCut {
    /// Where the edge a cut leaves lands from its line, mm, positive toward that edge's side,
    /// on the back: for the edge left of the direction of travel, and for the edge right of it.
    pub edge_mm: [f64; 2],
    /// How far a cut runs past the ends of its edge on board of no thickness, mm: before its
    /// start, and past its end. Negative starts short.
    pub run_on_mm: [f64; 2],
    /// How much each of those grows per mm of board.
    pub run_on_per_mm: f64,
    /// A closed curve's overlap: how long it keeps cutting past its start, ms at its speed
    /// (PROGRAM_FORMAT §4.2; protocol reference §7.1).
    pub overlap_ms: f64,
    /// How far the overlap drifts toward the waste by its end, mm: the run-out.
    pub run_out_mm: f64,
}

impl HeadCut {
    /// How far a cut runs past its edge's start and end on board `thickness_mm` thick.
    pub fn run_on(&self, thickness_mm: f64) -> [f64; 2] {
        self.run_on_mm
            .map(|r| r + self.run_on_per_mm * thickness_mm)
    }
}

/// Both heads' cuts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blade {
    pub bevel: HeadCut,
    pub straight: HeadCut,
}

impl Blade {
    /// The calibration fit's edges (`mc601 calibrate fit` on `testdata/calibration`,
    /// calibration 1, ±0.06 mm), GMC's run-ons, overlaps and run-outs until the overcut combs are
    /// read.
    ///
    /// - The bevel window's back edge lands 0.453 mm outside its line. A bevel cut always keeps
    ///   the board on its left (windows clockwise, kept pieces anticlockwise), so only the left
    ///   number is used.
    /// - The straight head's slit is next to nothing wide, its middle 0.095 mm right of the line:
    ///   the left edge lands 0.102 mm right of it, the right edge 0.087 mm right.
    /// - GMC's bevel cuts start 0.4 mm before an edge's corner and run 3.8 mm past the far one on
    ///   1.5 mm board, both growing 1 mm per mm of board (its C10, C11 and C12 jobs); its
    ///   straight cuts start 0.5 mm short and run 2.3 mm past, whatever the board (its trims
    ///   in its parameters; the captures agree to 0.3 mm).
    pub fn fitted() -> Blade {
        Blade {
            bevel: HeadCut {
                edge_mm: [0.453, 0.453],
                ..Blade::gmc().bevel
            },
            straight: HeadCut {
                edge_mm: [-0.102, 0.087],
                ..Blade::gmc().straight
            },
        }
    }

    /// GMC's numbers, as its programs show them (`testdata/programs`): a bevel window's line
    /// 0.1 mm inside its edge, a straight cut on its edge, and the run-ons, overlaps and
    /// run-outs of [`Blade::fitted`]. The golden tests rebuild GMC's jobs with it.
    pub fn gmc() -> Blade {
        Blade {
            bevel: HeadCut {
                edge_mm: [0.1, 0.1],
                run_on_mm: [-1.1, 2.3],
                run_on_per_mm: 1.0,
                overlap_ms: 1850.0,
                run_out_mm: 0.3,
            },
            straight: HeadCut {
                edge_mm: [0.0, 0.0],
                run_on_mm: [-0.5, 2.3],
                run_on_per_mm: 0.0,
                overlap_ms: 500.0,
                run_out_mm: 0.3,
            },
        }
    }

    pub fn head(&self, head: Head) -> &HeadCut {
        match head {
            Head::Bevel => &self.bevel,
            Head::Straight => &self.straight,
        }
    }
}

impl Default for Blade {
    fn default() -> Self {
        Blade::fitted()
    }
}

/// Which side of a cut the board is kept on, as it travels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kept {
    Left,
    Right,
    Both,
}

impl HeadCut {
    /// How far to move a line from the edge it cuts, mm along the normal to the left of
    /// travel, so the kept edge (or the middle of the slit, when both sides are kept) lands on
    /// it.
    pub(crate) fn shift(&self, kept: Kept) -> f64 {
        let [left, right] = self.edge_mm;
        match kept {
            Kept::Left => -left,
            Kept::Right => right,
            Kept::Both => -(left - right) / 2.0,
        }
    }
}
