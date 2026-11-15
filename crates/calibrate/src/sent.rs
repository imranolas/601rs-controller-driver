//! Where each cut went: the cuts a stream of commands sends, in carriage millimetres. The fit
//! needs the carriage positions a sheet was cut at, whatever profile placed it.
//!
//! The stream is either the bytes a run log shows went down the line ([`cuts_from_bytes`]), or
//! the program lowered again with the profile it was cut with ([`cuts_from_lowering`]). One
//! reading of the commands serves both, so they can't disagree about what a command means; the
//! record's log and its lowering are held to each other in `mc601`'s tests.

use machine::Profile;
use program::{Head, Program};

use crate::Error;

/// A cut as it was sent: its head, and its points in carriage mm.
#[derive(Debug, Clone, PartialEq)]
pub struct SentCut {
    pub head: Head,
    pub points: Vec<[f64; 2]>,
}

/// The head whose blade outputs `bits` lower, if any: the straight head's `i_sh` (`08`), or the
/// bevel's `i_t`/`i_b` (`01`, `04`).
fn head_down(bits: u8) -> Option<Head> {
    if bits & 0x08 != 0 {
        Some(Head::Straight)
    } else if bits & 0x05 != 0 {
        Some(Head::Bevel)
    } else {
        None
    }
}

/// Apply a vector's axes (`X…,Y…` absolute, or `x…,y…` relative) to `pos`; whether X or Y was
/// among them.
fn apply(axes: &str, relative: bool, pos: &mut [i64; 2]) -> bool {
    let mut any = false;
    for tok in axes.split(',') {
        let (axis, v) = tok.split_at(tok.len().min(1));
        let Ok(v) = v.parse::<i64>() else { continue };
        let i = match axis {
            "X" | "x" => 0,
            "Y" | "y" => 1,
            _ => continue,
        };
        pos[i] = if relative { pos[i] + v } else { v };
        any = true;
    }
    any
}

/// The cuts the host sent, in order, from the bytes it wrote (each command ends in `\r`): from
/// each head-down output to the next output without it, with the vectors between (absolute
/// `L…`, or the compact steps `x…,y…`). Steps become mm by `profile`'s scale.
pub fn cuts_from_bytes(sent: &[u8], profile: &Profile) -> Vec<SentCut> {
    let s = profile.scale;
    let mm = |p: [i64; 2]| {
        [
            p[0] as f64 / s.x_steps_per_mm,
            p[1] as f64 / s.y_steps_per_mm,
        ]
    };
    let mut pos = [0i64, 0];
    let mut cuts: Vec<SentCut> = Vec::new();
    let mut open: Option<SentCut> = None;
    for raw in sent.split(|&b| b == b'\r') {
        let cmd = String::from_utf8_lossy(raw).trim().to_owned();
        let outputs = cmd
            .strip_prefix("$A")
            .or_else(|| cmd.strip_prefix('A'))
            .and_then(|h| u8::from_str_radix(h, 16).ok());
        if let Some(bits) = outputs {
            let head = head_down(bits);
            if open.as_ref().map(|c| c.head) != head {
                if let Some(c) = open.take() {
                    if c.points.len() >= 2 {
                        cuts.push(c);
                    }
                }
                open = head.map(|head| SentCut {
                    head,
                    points: vec![mm(pos)],
                });
            }
            continue;
        }
        let moved = if let Some(rest) = cmd.strip_prefix('L') {
            // `L20,X…,Y…,Z…`: skip the speed entry.
            rest.split_once(',')
                .is_some_and(|(_, axes)| apply(axes, false, &mut pos))
        } else if cmd.starts_with(['x', 'y', 'z']) {
            apply(&cmd, true, &mut pos)
        } else {
            false
        };
        if moved {
            if let Some(c) = open.as_mut() {
                c.points.push(mm(pos));
            }
        }
    }
    if let Some(c) = open.take() {
        if c.points.len() >= 2 {
            cuts.push(c);
        }
    }
    cuts
}

/// The cuts `program` sends when lowered with `cut_with`, the profile it was (or will be) cut
/// with: what a run log of it would show, without the log.
///
/// # Errors
/// When the program can't be lowered with that profile.
pub fn cuts_from_lowering(program: &Program, cut_with: &Profile) -> Result<Vec<SentCut>, Error> {
    let job = machine::lower(program, cut_with)?;
    let bytes: Vec<u8> = job.commands().flat_map(|c| c.encode()).collect();
    Ok(cuts_from_bytes(&bytes, cut_with))
}

#[cfg(test)]
mod tests {
    use super::*;
    use program::Op;

    /// Calibration sheet 2 is cut raw, so the carriage runs along each cut's own points: the
    /// lowering gives them back, to the step.
    #[test]
    fn a_raw_sheet_lowers_to_its_own_points() {
        let profile = Profile::gunnar_601();
        let p = crate::sheet::program(&profile);
        let cuts = cuts_from_lowering(&p, &profile.raw()).expect("lowers raw");
        let want: Vec<&program::Cut> = p
            .ops
            .iter()
            .filter_map(|o| match o {
                Op::Cut(c) => Some(c),
                _ => None,
            })
            .collect();
        assert_eq!(cuts.len(), want.len());
        let step = 1.0 / profile.scale.x_steps_per_mm;
        for (got, want) in cuts.iter().zip(want) {
            assert_eq!(got.head, want.head, "{:?}", want.id);
            for end in [0, 1] {
                let (g, w) = if end == 0 {
                    (got.points[0], want.path_mm[0])
                } else {
                    (
                        got.points[got.points.len() - 1],
                        want.path_mm[want.path_mm.len() - 1],
                    )
                };
                for axis in 0..2 {
                    assert!(
                        (g[axis] - w[axis]).abs() <= step / 2.0 + 1e-9,
                        "{:?}: {g:?} against {w:?}",
                        want.id
                    );
                }
            }
        }
    }
}
