//! How each head cuts: the tool model (CALIBRATION.md §3). Each head is described as it is,
//! physically, in the blade's frame, which turns with the direction of travel; where a line goes
//! and how far its cut runs on are worked out from that and the board's thickness.
//!
//! - **Across the cut:** where the slit lies on the back (its sideways offset and width), and
//!   the blade's lean, which moves the slit left of travel as it goes down. The blade enters the
//!   back where it would on any board: GMC's rule face down (protocol reference §7.1, it never
//!   moves a line for thickness), which the shop's thick-board profiles bear out. So the back
//!   edge stays where the line puts it, and the face edge lies `thickness × tan(lean)` further
//!   left.
//! - **Along the cut:** how far a cut starts before its commanded start (its lead) and stops
//!   short of its commanded end (its shortfall), on the back and on the face. Measured, not
//!   derived: they come from the blade's shape and the head's plunge.
//! - **Corners:** a cut runs on past a square corner until it meets its neighbour's slit, at
//!   its farthest from that cut's line, on both faces, less how far it starts early or stops
//!   short there.
//!
//! The **back** is the side up, where the blade enters and where Bevel's designs give an edge;
//! the **face** is the customer's side, down on the sheets.

use program::Head;

/// How one head cuts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeadCut {
    /// Where the middle of the slit lies on the back, mm left of the line as the cut travels
    /// (negative: right of it).
    pub sideways_mm: f64,
    /// How far apart the slit's sides are, mm: about none, as the board closes behind the blade.
    pub width_mm: f64,
    /// The blade's lean from vertical, degrees, toward the left of travel as it goes down: none
    /// on the straight head.
    pub lean_deg: f64,
    /// How far a cut starts before its commanded start, mm: on the back, and on the face.
    pub lead_mm: [f64; 2],
    /// How far a cut stops short of its commanded end, mm: on the back, and on the face.
    pub shortfall_mm: [f64; 2],
    /// A closed curve's overlap: how long it keeps cutting past its start, ms at its speed
    /// (PROGRAM_FORMAT §4.2; protocol reference §7.1).
    pub overlap_ms: f64,
    /// How far the overlap drifts toward the waste by its end, mm: the run-out.
    pub run_out_mm: f64,
}

impl HeadCut {
    /// How far the slit moves left per mm deeper into the board: `tan(lean)`.
    pub fn run_per_mm(&self) -> f64 {
        self.lean_deg.to_radians().tan()
    }

    /// Where the slit's sides lie `depth_mm` below the back, mm left of the line: its left side,
    /// then its right.
    pub fn sides(&self, depth_mm: f64) -> [f64; 2] {
        let middle = self.sideways_mm + depth_mm * self.run_per_mm();
        [middle + self.width_mm / 2.0, middle - self.width_mm / 2.0]
    }

    /// How far the slit's farther side lies from the line, `depth_mm` below the back, mm.
    fn farthest(&self, depth_mm: f64) -> f64 {
        let [left, right] = self.sides(depth_mm);
        left.abs().max(right.abs())
    }

    /// How far a cut on board `thickness_mm` thick runs past a square corner of its edge: before
    /// its start, and past its end. Negative starts or stops short of the corner.
    pub fn run_on(&self, thickness_mm: f64) -> [f64; 2] {
        // Where the neighbour's slit lies, on the back and on the face.
        let [back, face] = [self.farthest(0.0), self.farthest(thickness_mm)];
        [
            (back - self.lead_mm[0]).max(face - self.lead_mm[1]),
            (back + self.shortfall_mm[0]).max(face + self.shortfall_mm[1]),
        ]
    }
}

/// Both heads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blade {
    pub bevel: HeadCut,
    pub straight: HeadCut,
}

/// The board calibration 1 was cut on, mm, as named.
const CALIBRATION_1_MM: f64 = 1.5;

impl Blade {
    /// The shop's 601 as fitted: calibration 1's values (`mc601 calibrate fit` on
    /// `testdata/calibration`, ± 0.06 mm across the cut, ± 0.13 along it), and on the face,
    /// where it read nothing along the cut, whatever gives GMC's run-ons on its 1.5 mm board,
    /// until stage 2's combs measure it (CALIBRATION.md §5).
    ///
    /// - The straight head's slit is next to nothing wide, its middle 0.095 mm right of the
    ///   line. Its run-ons don't change with the board: 0.5 mm short of the start, 2.3 mm past
    ///   the end, as GMC's.
    /// - The bevel's back edge lands 0.4525 mm left of its line, and it runs 1.71 mm further
    ///   left across the 1.5 mm board: a lean of 48.7°. Its run-ons are GMC's on that board,
    ///   0.4 mm before the start and 3.8 mm past the end, and grow with the board by that run,
    ///   1.14 mm per mm, where GMC's grow by 1 (it takes the bevel as 45°).
    pub fn fitted() -> Blade {
        let gmc = Blade::gmc();
        // The face's lead and shortfall: GMC's run-ons on calibration 1's board, less where the
        // neighbouring slit lies on the face.
        let face_to_gmc = |c: HeadCut, gmc: &HeadCut| {
            let far = c.farthest(CALIBRATION_1_MM);
            let [start, end] = gmc.run_on(CALIBRATION_1_MM);
            HeadCut {
                lead_mm: [c.lead_mm[0], far - start],
                shortfall_mm: [c.shortfall_mm[0], end - far],
                ..c
            }
        };
        Blade {
            bevel: face_to_gmc(
                HeadCut {
                    sideways_mm: 0.4525,
                    width_mm: 0.0,
                    lean_deg: (1.71 / CALIBRATION_1_MM).atan().to_degrees(),
                    lead_mm: [2.2225, 0.0],
                    shortfall_mm: [0.6475, 0.0],
                    ..gmc.bevel
                },
                &gmc.bevel,
            ),
            straight: face_to_gmc(
                HeadCut {
                    sideways_mm: -0.095,
                    width_mm: -0.015,
                    lean_deg: 0.0,
                    lead_mm: [1.9675, 0.0],
                    shortfall_mm: [1.2025, 0.0],
                    ..gmc.straight
                },
                &gmc.straight,
            ),
        }
    }

    /// GMC's numbers, as its programs show them (`testdata/programs`; protocol reference §7.1):
    /// a bevel window's line 0.1 mm inside its edge, the bevel taken as 45°, its trims as the
    /// reach along the cut; a straight cut on its edge. The golden tests rebuild GMC's jobs with
    /// it.
    pub fn gmc() -> Blade {
        Blade {
            bevel: HeadCut {
                sideways_mm: 0.1,
                width_mm: 0.0,
                lean_deg: 45.0,
                lead_mm: [1.2, 1.2],
                shortfall_mm: [2.2, 2.2],
                overlap_ms: 1850.0,
                run_out_mm: 0.3,
            },
            straight: HeadCut {
                sideways_mm: 0.0,
                width_mm: 0.0,
                lean_deg: 0.0,
                lead_mm: [0.5, 0.5],
                shortfall_mm: [2.3, 2.3],
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

    pub fn head_mut(&mut self, head: Head) -> &mut HeadCut {
        match head {
            Head::Bevel => &mut self.bevel,
            Head::Straight => &mut self.straight,
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
    /// travel, so the kept edge on the back (or the middle of the slit, when both sides are
    /// kept) lands on it.
    pub(crate) fn shift(&self, kept: Kept) -> f64 {
        let [left, right] = self.sides(0.0);
        match kept {
            Kept::Left => -left,
            Kept::Right => -right,
            Kept::Both => -self.sideways_mm,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f64; 2], b: [f64; 2]) -> bool {
        (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9
    }

    #[test]
    fn gmcs_run_ons_grow_by_the_boards_thickness_on_the_bevel_only() {
        let g = Blade::gmc();
        for t in [1.0, 1.5, 4.6] {
            assert!(close(g.bevel.run_on(t), [t - 1.1, 2.3 + t]), "{t}");
            assert!(close(g.straight.run_on(t), [-0.5, 2.3]), "{t}");
        }
    }

    #[test]
    fn fitted_run_ons_are_gmcs_on_calibration_1s_board_and_grow_by_the_bevels_run() {
        let (f, g) = (Blade::fitted(), Blade::gmc());
        assert!(close(f.bevel.run_on(1.5), g.bevel.run_on(1.5)));
        assert!(close(f.straight.run_on(1.5), g.straight.run_on(1.5)));
        assert!((f.bevel.run_per_mm() - 1.14).abs() < 1e-12);
        let [start, end] = f.bevel.run_on(4.6);
        let more = 0.14 * 3.1;
        assert!((start - (3.5 + more)).abs() < 1e-9 && (end - (6.9 + more)).abs() < 1e-9);
        assert!(close(f.straight.run_on(4.6), [-0.5, 2.3]));
        // On the back the bevel edge is calibration 1's, on the face 1.71 mm further out.
        assert!(close(f.bevel.sides(0.0), [0.4525, 0.4525]));
        assert!(close(f.bevel.sides(1.5), [2.1625, 2.1625]));
        assert!(close(f.straight.sides(0.0), [-0.1025, -0.0875]));
    }

    #[test]
    fn a_line_moves_so_the_kept_edge_lands_on_it() {
        let s = Blade::fitted().straight;
        assert!((s.shift(Kept::Left) - 0.1025).abs() < 1e-12);
        assert!((s.shift(Kept::Right) - 0.0875).abs() < 1e-12);
        assert!((s.shift(Kept::Both) - 0.095).abs() < 1e-12);
    }
}
