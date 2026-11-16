//! Where each cut went: the cuts a program sends, in carriage millimetres, from lowering it again
//! with the profile it was cut with. Lowering is pure, so an app that keeps no log still knows
//! where every cut went; `mc601`'s tests hold it to what the simulator received.

use machine::Profile;
use program::{Head, Program};
use protocol::Command;

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

/// The cuts `program` sends when lowered with `cut_with`, the profile it was (or will be) cut
/// with, in order: from each head-down output to the next output without it, through the
/// vectors between. Steps become mm by the profile's scale.
///
/// # Errors
/// When the program can't be lowered with that profile.
pub fn cuts(program: &Program, cut_with: &Profile) -> Result<Vec<SentCut>, Error> {
    let job = machine::lower(program, cut_with)?;
    let s = cut_with.scale;
    let mm = |[x, y]: [i64; 2]| [x as f64 / s.x_steps_per_mm, y as f64 / s.y_steps_per_mm];
    let mut at = [0i64, 0];
    let mut out: Vec<SentCut> = Vec::new();
    let mut open: Option<SentCut> = None;
    let mut close = |open: &mut Option<SentCut>| {
        if let Some(c) = open.take().filter(|c| c.points.len() >= 2) {
            out.push(c);
        }
    };
    for c in job.commands() {
        match *c {
            Command::SetOutputs(bits) | Command::StreamOutputs(bits) => {
                let head = head_down(bits);
                if open.as_ref().map(|c| c.head) != head {
                    close(&mut open);
                    open = head.map(|head| SentCut {
                        head,
                        points: vec![mm(at)],
                    });
                }
            }
            Command::Vector { x, y, .. } => at = [x.into(), y.into()],
            Command::Step { x, y, .. } => at = [at[0] + i64::from(x), at[1] + i64::from(y)],
            _ => continue,
        }
        if matches!(c, Command::Vector { .. } | Command::Step { .. }) {
            if let Some(cut) = open.as_mut() {
                cut.points.push(mm(at));
            }
        }
    }
    close(&mut open);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Calibration;
    use crate::fit::{Unknown, known};
    use crate::key::Axis;
    use crate::plan::Stock;
    use program::Op;

    /// Stage 1 lowered: each cut runs along its own points moved by `P`, where the profile puts
    /// its head's line on the registered corner, to the motor step.
    #[test]
    fn each_cut_runs_where_its_heads_line_is_put() {
        let cal = Calibration::new();
        let plan = crate::heads::plan(&cal, &Stock::offcut(1.5)).expect("a plan");
        let sent = cuts(&plan.program, &plan.cut_with).expect("lowers");
        let planned: Vec<&program::Cut> = plan
            .program
            .ops
            .iter()
            .filter_map(|o| match o {
                Op::Cut(c) => Some(c),
                _ => None,
            })
            .collect();
        assert_eq!(sent.len(), planned.len());
        let step = 1.0 / plan.cut_with.scale.x_steps_per_mm;
        for (got, want) in sent.iter().zip(planned) {
            assert_eq!(got.head, want.head, "{:?}", want.id);
            let p = Axis::BOTH.map(|a| known(Unknown::P(want.head, a), &cal));
            let ends = |v: &[[f64; 2]]| [v[0], v[v.len() - 1]];
            for (g, w) in ends(&got.points).into_iter().zip(ends(&want.path_mm)) {
                for i in 0..2 {
                    assert!(
                        (g[i] - (w[i] + p[i])).abs() <= step / 2.0 + 1e-9,
                        "{:?}: {g:?} against {w:?} + {p:?}",
                        want.id
                    );
                }
            }
        }
    }
}
