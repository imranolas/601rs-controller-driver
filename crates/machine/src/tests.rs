use program::{
    BladeOffsets, Board, Clamp, ClampSide, Cut, Head as HeadKind, Material, Op, Program,
};
use protocol::{Command, Setting};

use crate::profile::Capabilities;

use super::*;
use crate::frame::{CarriageMm, blade_steps, board_point, directions, steps};

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
    // GMC's straight edges on g_regular_mount, its layout 21 mm in from the registered
    // corner, where they physically land: GMC's own offset put them 0.21 and 0.18 mm short.
    assert_eq!(steps(&p, &p.straight, [21.21, 21.18]), [601, 761]);
    assert_eq!(steps(&p, &p.straight, [221.21, 331.18]), [8601, 13161]);
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

/// Where GMC lays a layout, board mm: 21 mm in from the corner where boards register, clear of
/// the clamps ([GCF] `MatLayX`/`MatLayY`). These tests describe their cuts from that corner.
const LAYOUT: [f64; 2] = [21.0, 21.0];

/// A cut along `path`, given from GMC's layout corner ([`LAYOUT`]).
fn cut(head: HeadKind, path: &[[f64; 2]]) -> Op {
    Op::Cut(Cut {
        id: None,
        label: None,
        head,
        path_mm: path
            .iter()
            .map(|&[x, y]| [x + LAYOUT[0], y + LAYOUT[1]])
            .collect(),
        speed_pct: None,
    })
}

/// A machine without clamp pads keeps GMC's whole-edge border.
#[test]
fn cuts_keep_clear_of_the_border_on_a_machine_without_pads() {
    let mut p = p601();
    p.clamp_pads.front_mm.clear();
    p.clamp_pads.left_mm.clear();
    // GMC's own layouts cut exactly on its 21 mm border.
    let edge = [[0.0, 0.0], [200.0, 0.0]];
    lower(
        &program(1.5, vec![clamp(), cut(HeadKind::Straight, &edge)]),
        &p,
    )
    .expect("on the border");
    for near in [[[-0.5, 50.0], [-0.5, 150.0]], [[50.0, -1.0], [150.0, -1.0]]] {
        let err = lower(&program(1.5, vec![cut(HeadKind::Bevel, &near)]), &p).expect_err("near");
        assert!(
            matches!(
                err,
                Error::NearClamps {
                    op: 0,
                    point: 0,
                    ..
                }
            ),
            "{err:?}"
        );
    }
}

#[test]
fn a_board_over_half_the_table_is_held_by_both_clamps() {
    let p = p601();
    assert_eq!(p.clamp_side(780.0), ClampSide::Left);
    assert_eq!(p.clamp_side(781.0), ClampSide::Both);
    assert!(!p.can_cut_at_other_end(700.0, 500.0));
    // A 300 × 400 mm board reaches one pad at the front and two at the left.
    let on: Vec<String> = p
        .pads_on(300.0, 400.0)
        .into_iter()
        .map(|p| p.name)
        .collect();
    assert_eq!(on, ["F1", "L1", "L2"]);
    assert_eq!(p.pads_on(300.0, 400.0)[0].rect_mm, [75.5, 0.0, 118.5, 20.0]);
}

/// Board-frame cuts (no layout offset) with the clamp op turning `turned`.
fn cuts_turning(turned: &[&str], paths: &[&[[f64; 2]]]) -> Program {
    let mut ops = vec![Op::Clamp(Clamp {
        id: None,
        label: None,
        side: ClampSide::Left,
        turned: turned.iter().map(|s| (*s).to_owned()).collect(),
    })];
    ops.extend(paths.iter().map(|path| {
        Op::Cut(Cut {
            id: None,
            label: None,
            head: HeadKind::Bevel,
            path_mm: path.to_vec(),
            speed_pct: None,
        })
    }));
    program(1.5, ops)
}

#[test]
fn cuts_run_to_the_edge_but_not_over_a_pad_unless_it_is_turned() {
    let p = p601();
    // 5 mm from the front edge, between pads F1 (to 118.5) and F2: clear.
    let between = [[130.0, 5.0], [190.0, 5.0]];
    lower(&cuts_turning(&[], &[&between]), &p).expect("runs to the edge between pads");
    // Across F1, with no point on it: the segment counts.
    let across = [[50.0, 5.0], [150.0, 5.0]];
    let err = lower(&cuts_turning(&[], &[&across]), &p).expect_err("over F1");
    assert!(
        matches!(&err, Error::OnClamp { op: 1, point: 1, pad } if pad == "F1"),
        "{err:?}"
    );
    lower(&cuts_turning(&["F1"], &[&across]), &p).expect("F1 turned out of the way");
    // Up the left edge, onto L2 (from 305.5).
    let up = [[5.0, 130.0], [5.0, 309.0]];
    let err = lower(&cuts_turning(&["F1"], &[&up]), &p).expect_err("over L2");
    assert!(
        matches!(&err, Error::OnClamp { pad, .. } if pad == "L2"),
        "{err:?}"
    );
    let err = lower(&cuts_turning(&["F9"], &[&between]), &p).expect_err("no F9");
    assert!(
        matches!(&err, Error::UnknownClamp { op: 0, pad } if pad == "F9"),
        "{err:?}"
    );
    assert_eq!(
        cuts_turning(&["L2", "F1"], &[&up]).turned_at_start(),
        ["L2", "F1"]
    );
}

fn clamp() -> Op {
    Op::Clamp(Clamp {
        id: None,
        label: None,
        side: ClampSide::Left,
        turned: Vec::new(),
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

#[test]
fn the_bevel_starts_softly_on_thick_board_unless_switched_off() {
    let edge = [[0.0, 0.0], [200.0, 0.0]];
    let job = |p: &Profile, t: f64| {
        lower(&program(t, vec![clamp(), cut(HeadKind::Bevel, &edge)]), p).expect("lowers")
    };
    let starts = |p: &Profile, t: f64| -> Vec<u32> {
        job(p, t)
            .commands()
            .filter_map(|c| match c {
                Command::Set(Setting::StartSpeed(s)) => Some(*s),
                _ => None,
            })
            .collect()
    };
    let ramps = |p: &Profile, t: f64| -> Vec<u16> {
        job(p, t)
            .commands()
            .filter_map(|c| match c {
                Command::Set(Setting::RampMs(r)) => Some(*r),
                _ => None,
            })
            .collect()
    };
    let p = p601();
    // Below 2 mm, as GMC: the head's start speed. Halfway from 2 to 3.5 mm, halfway from 100 to
    // 10 mm/s; from 3.5 mm, 10 mm/s. The ramp stays the cut's.
    assert!(starts(&p, 1.5).iter().all(|&s| s == 4000));
    assert!(starts(&p, 2.75).contains(&2200));
    assert!(starts(&p, 4.6).contains(&400));
    assert_eq!(ramps(&p, 4.6), ramps(&p, 1.5));
    // Switched off, GMC's: every bevel cut starts at 100 mm/s.
    let mut gmc = p601();
    gmc.soft_start.bevel = false;
    assert!(starts(&gmc, 4.6).iter().all(|&s| s == 4000));
}

/// How long the host waits after each change of the outputs to `outputs`, ms.
fn settles_after(job: &Job, outputs: u8) -> Vec<u32> {
    let steps: Vec<&Step> = job.steps().collect();
    steps
        .windows(2)
        .filter_map(|w| match (w[0], w[1]) {
            (Step::Send(Command::SetOutputs(o)), Step::Settle(ms)) if *o == outputs => Some(*ms),
            (Step::Send(Command::StreamOutputs(o)), Step::Send(Command::StreamDwellMs(ms)))
                if *o == outputs =>
            {
                Some(*ms)
            }
            _ => None,
        })
        .collect()
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
    // It moves off after GMC's 100 ms, the blade still coming down through the throttle, where
    // an edge waits the material's settle, 250 ms; or, switched off, the oval does too.
    assert_eq!(settles_after(&job, 0x41), [100]);
    let mut settled = p601();
    settled.bevel.closed_settle_ms = None;
    assert_eq!(
        settles_after(&lower(&prog, &settled).expect("lowers"), 0x41),
        [250]
    );
    // An arc of the same circle isn't closed.
    let arc = program(1.5, vec![clamp(), cut(HeadKind::Bevel, &pts[..60])]);
    let job = lower(&arc, &p).expect("lowers");
    assert!(job.commands().any(|c| *c == Command::SetOutputs(0x45)));
    assert_eq!(settles_after(&job, 0x45), [250]);
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

/// GMC's V-groove reset after homing, as it sent it [CAP g_init]; the mode switches and the
/// `#R250` it sends between moves (and replaces before the next) aside.
#[test]
fn the_vgroove_reset_is_gmcs() {
    let p = p601();
    let wire: Vec<String> = vgroove_reset(&p, 0, 0)
        .iter()
        .filter_map(|s| {
            if let Step::Send(c) = s {
                Some(c.to_string())
            } else {
                None
            }
        })
        .collect();
    assert_eq!(
        wire,
        [
            "$A02",
            "#C1,0",
            "#R225",
            "#G20,36000",
            "L20,X0,Y40600,Z749",
            "#R112",
            "L20,X0,Y41800,Z749",
            "L20,X0,Y40600,Z749",
        ]
    );
    assert!(
        !p.envelope_mm.contains(p.carriage_mm([0, 41_800])),
        "past the soft limit"
    );
}

#[test]
fn targets_outside_the_machine_are_refused() {
    let p = p601();
    let far = [[2000.0, 0.0], [2100.0, 0.0]];
    let err = lower(&program(1.5, vec![cut(HeadKind::Straight, &far)]), &p).expect_err("outside");
    assert!(
        matches!(err, Error::OutsideEnvelope { at: At::Op(0), .. }),
        "{err:?}"
    );
}

#[test]
fn the_profile_reads_back_from_json() {
    let p = p601();
    let text = serde_json::to_string_pretty(&p).expect("serialises");
    assert_eq!(serde_json::from_str::<Profile>(&text).expect("parses"), p);
    assert!(
        text.contains("\"board_origin_mm\": [\n"),
        "a point is [x, y], whatever its frame"
    );
}

#[test]
fn points_carry_their_frame() {
    let p = p601();
    let carriage = p.carriage_mm([400, -80]);
    assert_eq!(carriage, CarriageMm([10.0, -2.0]));
    assert_eq!(p.carriage_steps(carriage), [400, -80]);
    assert_eq!(carriage.to_string(), "(10.0, -2.0) mm");
    assert_eq!(carriage.distance(CarriageMm([13.0, 2.0])), 5.0);
    // The bevel head's blade is 71.45 mm to the carriage's left.
    assert_eq!(p.bevel.blade(carriage).0[0], 10.0 + p.bevel.offset_mm[0]);
    assert!((p.bevel.blade(carriage).0[0] + 61.45).abs() < 1e-9);
}

#[test]
fn errors_say_where() {
    assert_eq!(
        Error::ZeroSegment { op: 2, point: 5 }.to_string(),
        format!("op 2: points 4 and 5 are closer than {MIN_SEGMENT_MM} mm")
    );
    assert_eq!(
        Error::BadNumber { at: At::Job }.to_string(),
        "the job: a number is not finite or out of range"
    );
    assert_eq!(
        Error::OutsideEnvelope {
            at: At::Park,
            carriage_mm: CarriageMm([1.0, -2.0])
        }
        .to_string(),
        "the park: the carriage would reach (1.0, -2.0) mm, outside the machine's limits"
    );
}

#[test]
fn pads_are_turned_only_with_the_clamps_open() {
    let p = p601();
    let clamp_turning = |side: ClampSide, turned: &[&str]| {
        Op::Clamp(Clamp {
            id: None,
            label: None,
            side,
            turned: turned.iter().map(|s| (*s).to_owned()).collect(),
        })
    };
    let across = vec![[50.0, 5.0], [150.0, 5.0]];
    let cut_across = || {
        Op::Cut(Cut {
            id: None,
            label: None,
            head: HeadKind::Bevel,
            path_mm: across.clone(),
            speed_pct: None,
        })
    };
    let pause = || {
        Op::Pause(program::Pause {
            id: None,
            label: None,
            message: "Turn F1 out of the way".to_owned(),
            resume: program::Resume::StartButton,
        })
    };
    let clear = cut(HeadKind::Bevel, &[[50.0, 50.0], [150.0, 50.0]]);
    // Turning F1 while the board is clamped: refused.
    let err = lower(
        &program(
            1.5,
            vec![
                clamp(),
                clear.clone(),
                clamp_turning(ClampSide::Left, &["F1"]),
                cut_across(),
            ],
        ),
        &p,
    )
    .expect_err("turned while clamped");
    assert!(
        matches!(err, Error::TurnedWhileClamped { op: 2 }),
        "{err:?}"
    );
    // Released, but no pause for the operator: refused too.
    let err = lower(
        &program(
            1.5,
            vec![
                clamp(),
                clear.clone(),
                clamp_turning(ClampSide::None, &[]),
                clamp_turning(ClampSide::Left, &["F1"]),
                cut_across(),
            ],
        ),
        &p,
    )
    .expect_err("no pause");
    assert!(
        matches!(err, Error::TurnedWhileClamped { op: 3 }),
        "{err:?}"
    );
    // Released, a pause, then clamped again with F1 turned.
    lower(
        &program(
            1.5,
            vec![
                clamp(),
                clear,
                clamp_turning(ClampSide::None, &[]),
                pause(),
                clamp_turning(ClampSide::Left, &["F1"]),
                cut_across(),
            ],
        ),
        &p,
    )
    .expect("turned with the clamps open");
}

#[test]
fn the_outputs_say_which_head_is_down_and_which_clamps_close() {
    let p = p601();
    assert_eq!(p.heads(0x02), Heads::Up);
    assert_eq!(p.heads(0x42), Heads::Up);
    assert_eq!(p.heads(0x45), Heads::BevelDown);
    assert_eq!(p.heads(0x41), Heads::BevelDown, "the slow head-down");
    assert_eq!(
        p.heads(0x40),
        Heads::BevelDown,
        "the blade out, the head still down"
    );
    assert_eq!(p.heads(0x4A), Heads::StraightDown);
    assert_eq!(p.clamped(0x42), ClampSide::Left);
    assert_eq!(p.clamped(0x62), ClampSide::Both);
    assert_eq!(p.clamped(0x02), ClampSide::None);
}
