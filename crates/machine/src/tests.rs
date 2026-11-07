use program::{
    BladeOffsets, Board, Clamp, ClampSide, Cut, Head as HeadKind, Material, Op, Program,
};
use protocol::{Command, Setting};

use crate::profile::Capabilities;

use super::*;
use crate::frame::{blade_steps, board_point, directions, steps};

fn p601() -> Profile {
    Profile::gunnar_601()
}

#[test]
fn blade_angles_match_the_captures() {
    let p = p601();
    // [CAP g_regular_mount]: a bevel edge travelling −Y with C10's 3° trim, a straight edge
    // travelling +X; [CAP g_multi_C11]: the bevel edge with a 4° trim.
    assert_eq!(blade_steps(&p, &p.bevel, 3.0, 90.0), 774);
    assert_eq!(blade_steps(&p, &p.bevel, 3.0, 180.0), 1524);
    assert_eq!(blade_steps(&p, &p.bevel, 3.0, 270.0), 2274);
    assert_eq!(blade_steps(&p, &p.bevel, 3.0, 0.0), 24);
    assert_eq!(blade_steps(&p, &p.straight, 0.0, 0.0), -1499);
    assert_eq!(blade_steps(&p, &p.straight, 0.0, 270.0), 749);
    assert_eq!(blade_steps(&p, &p.bevel, 4.0, 90.0), 783);
}

#[test]
fn headings_are_clockwise_from_plus_x() {
    use crate::frame::heading;
    assert_eq!(heading([0.0, 0.0], [1.0, 0.0]), 0.0);
    assert_eq!(heading([0.0, 0.0], [0.0, -1.0]), 90.0);
    assert_eq!(heading([0.0, 0.0], [-1.0, 0.0]), 180.0);
    assert_eq!(heading([0.0, 0.0], [0.0, 1.0]), 270.0);
}

#[test]
fn board_points_and_steps_agree_for_both_heads() {
    let p = p601();
    // GMC's straight edges on g_regular_mount: the board's left edge and its bottom edge.
    assert_eq!(steps(&p, &p.straight, [0.0, 0.0]), [601, 761]);
    assert_eq!(steps(&p, &p.straight, [200.0, 310.0]), [8601, 13161]);
    for head in [p.bevel, p.straight] {
        for s in [[601, 761], [9695, 2615], [58_860, 7923], [-3, 0]] {
            assert_eq!(steps(&p, &head, board_point(&p, &head, s)), s);
        }
    }
}

#[test]
fn ramps_follow_gmcs_law() {
    let p = p601();
    assert_eq!(p.ramp_ms(p.accel.travel), 225);
    assert_eq!(p.ramp_ms(p.accel.cut), 250);
    assert_eq!(p.ramp_ms(p.accel.fast_turn), 112);
    assert_eq!(p.ramp_ms(420.0), 700, "capped");
    assert_eq!(p.speed(95.0), 34_200);
    assert_eq!(p.speed(100.0), 36_000);
}

#[test]
fn a_curve_carries_the_tangent_at_each_point() {
    // Turning 2° at each point: the blade is halfway between the segments there.
    let pts: Vec<[f64; 2]> = (0..4)
        .map(|k| {
            let a = (k as f64 * 2.0_f64).to_radians();
            [10.0 * a.cos(), -10.0 * a.sin()]
        })
        .collect();
    let d = directions(&pts);
    assert_eq!(d.len(), 4);
    for (k, &dir) in d.iter().enumerate() {
        let segs = [1.0, 3.0, 5.0];
        let want = match k {
            0 => segs[0],
            3 => segs[2],
            _ => (segs[k - 1] + segs[k]) / 2.0,
        };
        assert!((dir - (90.0 + want)).abs() < 1e-9, "point {k}: {dir}");
    }
}

#[test]
fn the_blade_keeps_counting_round_a_loop() {
    // A square spiral of small turns past 360°: no jump back to 0.
    let mut pts = vec![[0.0, 0.0]];
    let mut a = 0.0_f64;
    for _ in 0..200 {
        a += 2.0;
        let last = pts[pts.len() - 1];
        pts.push([
            last[0] + a.to_radians().cos(),
            last[1] - a.to_radians().sin(),
        ]);
    }
    let d = directions(&pts);
    assert!(d.windows(2).all(|w| (w[1] - w[0]).abs() <= 2.0 + 1e-9));
    assert!(d[d.len() - 1] > 360.0);
}

fn program(thickness_mm: f64, ops: Vec<Op>) -> Program {
    Program {
        format: program::FORMAT.to_owned(),
        version: program::VERSION,
        id: "test".to_owned(),
        meta: None,
        board: Board {
            width_mm: 200.0,
            height_mm: 310.0,
            thickness_mm,
        },
        material: Material {
            name: "MSK_1.5".to_owned(),
            speed_pct: 95.0,
            head_settle_ms: 250,
            blade_angle_offset_deg: BladeOffsets {
                straight: 0.0,
                bevel: 3.0,
                bevel_round: None,
            },
        },
        ops,
    }
}

fn cut(head: HeadKind, path: &[[f64; 2]]) -> Op {
    Op::Cut(Cut {
        id: None,
        label: None,
        head,
        path_mm: path.to_vec(),
        speed_pct: None,
    })
}

fn clamp() -> Op {
    Op::Clamp(Clamp {
        id: None,
        label: None,
        side: ClampSide::Left,
    })
}

#[test]
fn corners_split_a_cut_and_sharp_ones_are_refused() {
    let p = p601();
    // 10° at the middle point: two strokes, each its own head down and up.
    let a = 10.0_f64.to_radians();
    let bent = [
        [50.0, 50.0],
        [100.0, 50.0],
        [100.0 + 50.0 * a.cos(), 50.0 - 50.0 * a.sin()],
    ];
    let job = lower(
        &program(1.5, vec![clamp(), cut(HeadKind::Bevel, &bent)]),
        &p,
    )
    .expect("lowers");
    let downs = job
        .commands()
        .filter(|c| **c == Command::SetOutputs(0x45))
        .count();
    assert_eq!(downs, 2);
    let b = 20.0_f64.to_radians();
    let sharp = [
        [50.0, 50.0],
        [100.0, 50.0],
        [100.0 + 50.0 * b.cos(), 50.0 - 50.0 * b.sin()],
    ];
    let err = lower(&program(1.5, vec![cut(HeadKind::Bevel, &sharp)]), &p).expect_err("too sharp");
    assert!(
        matches!(
            err,
            Error::SharpCorner {
                op: 0,
                point: 1,
                ..
            }
        ),
        "{err:?}"
    );
    let doubled = [[50.0, 50.0], [50.0, 50.005], [80.0, 50.0]];
    let err =
        lower(&program(1.5, vec![cut(HeadKind::Bevel, &doubled)]), &p).expect_err("zero segment");
    assert_eq!(err, Error::ZeroSegment { op: 0, point: 1 });
}

#[test]
fn the_straight_head_starts_softly_on_thick_board() {
    let p = p601();
    let edge = [[0.0, 0.0], [200.0, 0.0]];
    let settings = |t: f64| -> Vec<Setting> {
        let job = lower(
            &program(t, vec![clamp(), cut(HeadKind::Straight, &edge)]),
            &p,
        )
        .expect("lowers");
        job.commands()
            .filter_map(|c| match c {
                Command::Set(s @ (Setting::RampMs(_) | Setting::StartSpeed(_))) => Some(*s),
                _ => None,
            })
            .collect()
    };
    // [CAP g_multi_C10, C11, C12]: #S1000; #R470 #S2232; #R700 #S400. Each restored after.
    let has = |t, s: &[Setting]| s.iter().all(|x| settings(t).contains(x));
    assert!(has(
        1.5,
        &[Setting::StartSpeed(1000), Setting::StartSpeed(4000)]
    ));
    assert!(has(
        2.7,
        &[
            Setting::RampMs(470),
            Setting::StartSpeed(2232),
            Setting::RampMs(250),
            Setting::StartSpeed(4000)
        ]
    ));
    assert!(has(4.6, &[Setting::RampMs(700), Setting::StartSpeed(400)]));
}

/// A bevel opening's edges, a 10° corner (two strokes), and a straight edge.
fn mixed_job() -> Program {
    let a = 10.0_f64.to_radians();
    program(
        1.5,
        vec![
            clamp(),
            cut(HeadKind::Bevel, &[[50.0, 260.0], [50.0, 60.0]]),
            cut(HeadKind::Bevel, &[[50.0, 60.0], [150.0, 60.0]]),
            cut(
                HeadKind::Bevel,
                &[
                    [60.0, 80.0],
                    [110.0, 80.0],
                    [110.0 + 30.0 * a.cos(), 80.0 + 30.0 * a.sin()],
                ],
            ),
            cut(HeadKind::Straight, &[[0.0, 0.0], [0.0, 310.0]]),
        ],
    )
}

fn forms() -> [Capabilities; 4] {
    [
        Capabilities::default(),
        Capabilities::all(),
        Capabilities {
            compact_paths: true,
            ..Capabilities::default()
        },
        Capabilities {
            stream_head: true,
            ..Capabilities::default()
        },
    ]
}

#[test]
fn nothing_changes_while_the_carriage_runs() {
    // After a stream command, only more of the stream until a wait for idle: no setting, no
    // output in mode 1, no host wait, no hold, no mode switch while the carriage may run.
    for caps in forms() {
        let p = Profile {
            capabilities: caps,
            ..p601()
        };
        let job = lower(&mixed_job(), &p).expect("lowers");
        let mut streaming = false;
        for s in job.steps() {
            match s {
                Step::Send(c) if c.required_mode() == Some(protocol::Mode::Cutting) => {
                    streaming = true
                }
                Step::WaitIdle => streaming = false,
                other => assert!(!streaming, "{caps:?}: {other:?} while the stream may run"),
            }
        }
        assert!(!streaming, "{caps:?}: the job ends running");
    }
}

#[test]
fn the_heads_are_up_whenever_the_carriage_travels() {
    // Every vector sent with the heads up is a travel, one per stroke and one to the park; all
    // the others are path, sent with a head down.
    for caps in forms() {
        let p = Profile {
            capabilities: caps,
            ..p601()
        };
        let job = lower(&mixed_job(), &p).expect("lowers");
        let (mut outputs, mut up, mut down) = (p.neutral_outputs, 0, 0);
        let mut at = [0; 3];
        for c in job.commands() {
            match *c {
                Command::SetOutputs(o) | Command::StreamOutputs(o) => outputs = o,
                Command::Vector { x, y, z, .. } => {
                    at = [x, y, z];
                    if outputs & 0x0F == p.neutral_outputs {
                        up += 1
                    } else {
                        down += 1
                    }
                }
                Command::Step { x, y, z } => {
                    at = [at[0] + x, at[1] + y, at[2] + z];
                    assert_ne!(
                        outputs & 0x0F,
                        p.neutral_outputs,
                        "{caps:?}: a step with the heads up"
                    );
                    down += 1;
                }
                _ => {}
            }
        }
        // Strokes: 1 + 1 + 2 + 1; then the park. Path vectors: 1 + 1 + (1 + 1) + 1.
        assert_eq!((up, down), (6, 5), "{caps:?}");
    }
}

#[test]
fn a_cut_off_the_axes_gets_a_longer_ramp() {
    // GMC's diagonal smoothing: 250 ms × (1 + 30° ÷ 45°).
    let p = p601();
    let a = 30.0_f64.to_radians();
    let job = lower(
        &program(
            1.5,
            vec![cut(
                HeadKind::Bevel,
                &[[50.0, 50.0], [50.0 + 80.0 * a.cos(), 50.0 - 80.0 * a.sin()]],
            )],
        ),
        &p,
    )
    .expect("lowers");
    assert!(
        job.commands()
            .any(|c| *c == Command::Set(Setting::RampMs(416)))
    );
    let along = lower(
        &program(
            1.5,
            vec![cut(HeadKind::Bevel, &[[50.0, 50.0], [130.0, 50.0]])],
        ),
        &p,
    )
    .expect("lowers");
    assert!(
        !along
            .commands()
            .any(|c| matches!(c, Command::Set(Setting::RampMs(r)) if *r > 250))
    );
}

#[test]
fn a_closed_curve_goes_down_as_gmcs_oval_and_takes_the_round_trim() {
    // A circle of radius 40 with a quarter lap of overlap, 2° a point.
    let pts: Vec<[f64; 2]> = (0..=225)
        .map(|k| {
            let t = (k as f64 * 2.0).to_radians();
            [100.0 + 40.0 * t.sin(), 150.0 + 40.0 * t.cos()]
        })
        .collect();
    let mut prog = program(1.5, vec![clamp(), cut(HeadKind::Bevel, &pts)]);
    prog.material.blade_angle_offset_deg.bevel_round = Some(0.0);
    let p = p601();
    let job = lower(&prog, &p).expect("lowers");
    assert!(
        job.commands().any(|c| *c == Command::SetOutputs(0x41)),
        "closed: $A41"
    );
    assert!(!job.commands().any(|c| *c == Command::SetOutputs(0x45)));
    // The first chord's direction (1° past +X here) with the round trim (0), not the bevel's 3°.
    let first = job.commands().find_map(|c| match *c {
        Command::Vector { z, .. } => Some(z),
        _ => None,
    });
    let chord = crate::frame::heading(pts[0], pts[1]);
    assert_eq!(first, Some(blade_steps(&p, &p.bevel, 0.0, chord)));
    assert_ne!(first, Some(blade_steps(&p, &p.bevel, 3.0, chord)));
    // An arc of the same circle isn't closed.
    let arc = program(1.5, vec![clamp(), cut(HeadKind::Bevel, &pts[..60])]);
    let job = lower(&arc, &p).expect("lowers");
    assert!(job.commands().any(|c| *c == Command::SetOutputs(0x45)));
}

#[test]
fn the_job_ends_parked_with_the_clamps_open() {
    let p = p601();
    let job = lower(
        &program(
            1.5,
            vec![
                clamp(),
                cut(HeadKind::Straight, &[[0.0, 0.0], [200.0, 0.0]]),
            ],
        ),
        &p,
    )
    .expect("lowers");
    let tail: Vec<&Command> = job
        .finish
        .iter()
        .filter_map(|s| if let Step::Send(c) = s { Some(c) } else { None })
        .collect();
    assert!(tail.contains(&&Command::Vector {
        speed_entry: ENTRY,
        x: 58_860,
        y: 7923,
        z: 0
    }));
    assert_eq!(tail.last(), Some(&&Command::SetOutputs(0x02)));
}

#[test]
fn targets_outside_the_machine_are_refused() {
    let p = p601();
    let far = [[2000.0, 0.0], [2100.0, 0.0]];
    let err = lower(&program(1.5, vec![cut(HeadKind::Straight, &far)]), &p).expect_err("outside");
    assert!(
        matches!(err, Error::OutsideEnvelope { op: Some(0), .. }),
        "{err:?}"
    );
}

#[test]
fn the_profile_reads_back_from_json() {
    let p = p601();
    let text = serde_json::to_string_pretty(&p).expect("serialises");
    assert_eq!(serde_json::from_str::<Profile>(&text).expect("parses"), p);
}
