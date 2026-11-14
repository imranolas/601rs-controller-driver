//! Golden tests: GMC's own jobs, as programs, lowered by this crate, against what GMC sent.
//!
//! For each capture, the cuts GMC made are a program in board millimetres
//! (`testdata/programs/<capture>.json`, written from the capture by this test with
//! `BLESS=1`, or when missing). Lowering that program must reproduce every motion GMC sent:
//! the same vectors, run with the same controller state, i.e. every setting and output GMC had
//! set by then has the same value. How GMC got there (sending each mode switch twice,
//! re-sending settings that didn't change) doesn't matter to the controller, so it doesn't
//! matter here.
//!
//! The vectors' X and Y come back exactly because the program was made from them; what the
//! test holds is everything else: the head offsets (both heads' cuts in one board frame), the
//! blade angles from the path's direction and the material's trim, the ramps, speeds, dead
//! times and soft start, the head and clamp outputs, and the park.
//!
//! Two differences are deliberate:
//! - On thin board GMC hops between the edges of an opening with the bevel head down and only
//!   the blade out (`$A40`). This crate raises the head for every travel (`$A42`), as GMC does
//!   on thicker board.
//! - After a straight-head cut on thin board, GMC sometimes leaves the thin-board start speed
//!   (`#S1000`) set for the travel that follows: always for the park, and on `g_multi_Cx_100`
//!   for every travel after the first straight edge. This crate puts `#S` back after every cut,
//!   so travel starts at the usual speed.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use link::Direction;
use machine::{Profile, lower};
use program::{BladeOffsets, Board, Clamp, ClampSide, Cut, Head, Material, Op, Program};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// The commands GMC sent, without the `@` master commands (mode switches and polls).
fn sent(capture: &str) -> Vec<String> {
    let path = repo()
        .join("testdata/captures")
        .join(format!("{capture}.txt"));
    let transfers = link::capture::read_file(&path).expect("capture reads");
    let bytes: Vec<u8> = transfers
        .into_iter()
        .filter(|t| t.direction == Direction::HostToController)
        .flat_map(|t| t.bytes)
        .collect();
    String::from_utf8_lossy(&bytes)
        .split('\r')
        .map(str::trim)
        .filter(|c| !c.is_empty() && !c.starts_with('@'))
        .map(str::to_owned)
        .collect()
}

/// One motion: the vectors sent together, and the controller state they ran with.
#[derive(Debug, PartialEq)]
struct Motion {
    state: BTreeMap<String, String>,
    vectors: Vec<String>,
}

/// The key a command sets (`#G20`, `#C1`, `#R`, `$A`, …) and its value, or `None` for a vector.
fn setting(c: &str) -> Option<(String, String)> {
    if c.starts_with('L') {
        return None;
    }
    Some(match c.rsplit_once(',') {
        Some((k, v)) if c.starts_with('#') => (k.to_owned(), v.to_owned()),
        _ => (c[..2].to_owned(), c[2..].to_owned()),
    })
}

/// The motions, and the state at the end.
fn motions(commands: &[String]) -> (Vec<Motion>, BTreeMap<String, String>) {
    let mut state = BTreeMap::new();
    let mut out: Vec<Motion> = Vec::new();
    let mut in_motion = false;
    for c in commands {
        match setting(c) {
            None => {
                if !in_motion {
                    out.push(Motion {
                        state: state.clone(),
                        vectors: Vec::new(),
                    });
                    in_motion = true;
                }
                out.last_mut().expect("a motion").vectors.push(c.clone());
            }
            Some((k, v)) => {
                in_motion = false;
                state.insert(k, v);
            }
        }
    }
    (out, state)
}

fn vector(v: &str) -> [i32; 3] {
    let mut out = [0; 3];
    for (k, part) in v.split(',').skip(1).enumerate() {
        out[k] = part[1..].parse().expect("a coordinate");
    }
    out
}

/// What the captures don't say: the material (GMC's profile) and the board's thickness.
struct Job {
    capture: &'static str,
    material: &'static str,
    thickness_mm: f64,
    bevel_trim_deg: f64,
    /// The bevel's trim on closed curves (GMC's `ZCorrection_Round`), where the job has one.
    bevel_round_deg: Option<f64>,
    head_settle_ms: u32,
    /// One of GMC's templates rather than a drawn shape (below).
    template: bool,
}

const JOBS: &[Job] = &[
    Job {
        capture: "g_multi_C10",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: None,
        head_settle_ms: 250,
        template: false,
    },
    Job {
        capture: "g_multi_C11",
        material: "MSK_2.5",
        thickness_mm: 2.7,
        bevel_trim_deg: 4.0,
        bevel_round_deg: None,
        head_settle_ms: 200,
        template: false,
    },
    Job {
        capture: "g_multi_C12",
        material: "MSK_4.1",
        thickness_mm: 4.6,
        bevel_trim_deg: 4.0,
        bevel_round_deg: None,
        head_settle_ms: 400,
        template: false,
    },
    Job {
        capture: "g_multi_Cx_20",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: None,
        head_settle_ms: 250,
        template: false,
    },
    Job {
        capture: "g_multi_Cx_40",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: None,
        head_settle_ms: 250,
        template: false,
    },
    Job {
        capture: "g_multi_Cx_60",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: None,
        head_settle_ms: 250,
        template: false,
    },
    Job {
        capture: "g_multi_Cx_80",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: None,
        head_settle_ms: 250,
        template: false,
    },
    Job {
        capture: "g_multi_Cx_100",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: None,
        head_settle_ms: 250,
        template: false,
    },
    Job {
        capture: "g_regular_mount",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: None,
        head_settle_ms: 250,
        template: true,
    },
    Job {
        capture: "g_oval_mount",
        material: "MSK_1.5",
        thickness_mm: 1.5,
        bevel_trim_deg: 3.0,
        bevel_round_deg: Some(0.0),
        head_settle_ms: 250,
        template: true,
    },
];

/// GMC's curve as smooth points, in steps. Rebuilt from 0.025 mm steps, the chords of GMC's
/// oval (0.3–1.3 mm) turn by up to 7.6° where it turned by 1.3° at most, so their directions
/// can't carry a blade angle. The oval is an ellipse: fit it to the points' extent, put each
/// point on it at the same angle, and add back the point's offset from it, smoothed over 31
/// points. That keeps the run-out, which takes the blade off the line at the end (protocol
/// reference §7.1), and drops the rounding.
fn smooth_ellipse(points: &[[i32; 3]]) -> Vec<[f64; 2]> {
    let xs = points.iter().map(|p| f64::from(p[0]));
    let ys = points.iter().map(|p| f64::from(p[1]));
    let (x0, x1) = xs.fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
    let (y0, y1) = ys.fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
    let (cx, cy, a, b) = (
        (x0 + x1) / 2.0,
        (y0 + y1) / 2.0,
        (x1 - x0) / 2.0,
        (y1 - y0) / 2.0,
    );
    let on: Vec<([f64; 2], [f64; 2])> = points
        .iter()
        .map(|p| {
            let (x, y) = (f64::from(p[0]), f64::from(p[1]));
            let t = ((y - cy) / b).atan2((x - cx) / a);
            let e = [cx + a * t.cos(), cy + b * t.sin()];
            (e, [x - e[0], y - e[1]])
        })
        .collect();
    (0..on.len())
        .map(|k| {
            let w = &on[k.saturating_sub(15)..(k + 16).min(on.len())];
            let n = w.len() as f64;
            let d = [
                w.iter().map(|o| o.1[0]).sum::<f64>() / n,
                w.iter().map(|o| o.1[1]).sum::<f64>() / n,
            ];
            [on[k].0[0] + d[0], on[k].0[1] + d[1]]
        })
        .collect()
}

/// GMC's cuts as a program: each motion with a head down is a cut from where the travel
/// before it ended.
fn program_from(job: &Job, profile: &Profile) -> Program {
    let (motions, _) = motions(&sent(job.capture));
    let mut ops = Vec::new();
    let mut clamped = false;
    let mut speed = None;
    let mut size = [0.0_f64; 2];
    let mut from = [0, 0, 0];
    let clean = |v: f64| (v * 1e6).round() / 1e6 + 0.0;
    for m in &motions {
        let outputs = u8::from_str_radix(&m.state["$A"], 16).expect("hex outputs");
        if !clamped {
            let side = match outputs & 0x60 {
                0x40 => ClampSide::Left,
                0x20 => ClampSide::Right,
                0x60 => ClampSide::Both,
                _ => ClampSide::None,
            };
            ops.push(Op::Clamp(Clamp {
                id: None,
                label: None,
                side,
                turned: Vec::new(),
            }));
            clamped = true;
        }
        let head = match outputs & 0x0F {
            0x05 | 0x01 => Some(Head::Bevel),
            0x0A => Some(Head::Straight),
            _ => None,
        };
        let last = vector(m.vectors.last().expect("vectors"));
        if let Some(head) = head {
            let h = profile.head(head);
            let steps: Vec<[i32; 3]> = std::iter::once(from)
                .chain(m.vectors.iter().map(|v| vector(v)))
                .collect();
            let smooth: Vec<[f64; 2]> = if steps.len() > 10 {
                smooth_ellipse(&steps)
            } else {
                steps
                    .iter()
                    .map(|s| [f64::from(s[0]), f64::from(s[1])])
                    .collect()
            };
            let (scale, o) = (profile.scale, profile.board_origin_mm);
            let path: Vec<[f64; 2]> = smooth
                .iter()
                .map(|s| {
                    [
                        s[0] / scale.x_steps_per_mm + h.offset_mm[0] - o.0[0],
                        s[1] / scale.y_steps_per_mm + h.offset_mm[1] - o.0[1],
                    ]
                    .map(clean)
                })
                .collect();
            if head == Head::Straight {
                let [a, b] = [path[0], path[path.len() - 1]];
                if a[0] == b[0] {
                    size[0] = size[0].max(a[0].round());
                } else if a[1] == b[1] {
                    size[1] = size[1].max(a[1].round());
                }
            }
            speed = Some(m.state["#G20"].parse::<f64>().expect("a speed") / 360.0);
            ops.push(Op::Cut(Cut {
                id: None,
                label: None,
                head,
                path_mm: path,
                speed_pct: None,
            }));
        }
        from = last;
    }
    Program {
        format: program::FORMAT.to_owned(),
        version: program::VERSION,
        id: job.capture.to_owned(),
        meta: Some(serde_json::json!({
            "source": format!("the cuts GMC made in testdata/captures/{}.txt, in board mm (crates/machine/tests/golden.rs)", job.capture)
        })),
        board: Board {
            width_mm: size[0],
            height_mm: size[1],
            thickness_mm: job.thickness_mm,
        },
        material: Material {
            name: job.material.to_owned(),
            speed_pct: speed.expect("at least one cut"),
            head_settle_ms: job.head_settle_ms,
            blade_angle_offset_deg: BladeOffsets {
                straight: 0.0,
                bevel: job.bevel_trim_deg,
                bevel_round: job.bevel_round_deg,
            },
        },
        ops,
    }
}

/// The program for a capture: read from its fixture, written first if asked or missing.
fn fixture(job: &Job, profile: &Profile) -> Program {
    let path = repo()
        .join("testdata/programs")
        .join(format!("{}.json", job.capture));
    if std::env::var_os("BLESS").is_some() || !path.exists() {
        let text =
            serde_json::to_string_pretty(&program_from(job, profile)).expect("serialises") + "\n";
        std::fs::create_dir_all(path.parent().expect("a directory")).expect("directory");
        std::fs::write(&path, text).expect("fixture written");
    }
    Program::from_json(&std::fs::read_to_string(&path).expect("fixture reads")).expect("a program")
}

/// Where our motions differ from GMC's: every key GMC had set must have the same value, and
/// every vector the same steps, within `tolerance` (X and Y, Z).
fn differences(
    job: &Job,
    tolerance: [i32; 2],
    ignore: impl Fn(&Motion, &str) -> bool,
) -> Vec<String> {
    let profile = Profile::gunnar_601();
    let program = fixture(job, &profile);
    let ours: Vec<String> = lower(&program, &profile)
        .expect("lowers")
        .commands()
        .map(|c| c.to_string())
        .collect();
    let (gmc, gmc_end) = motions(&sent(job.capture));
    let (mine, mine_end) = motions(&ours);
    let mut out = Vec::new();
    if gmc.len() != mine.len() {
        out.push(format!("{} motions, GMC {}", mine.len(), gmc.len()));
    }
    for (k, (g, m)) in gmc.iter().zip(&mine).enumerate() {
        let close = |a: &String, b: &String| {
            let (a, b) = (vector(a), vector(b));
            (a[0] - b[0]).abs() <= tolerance[0]
                && (a[1] - b[1]).abs() <= tolerance[0]
                && (a[2] - b[2]).abs() <= tolerance[1]
        };
        if g.vectors.len() != m.vectors.len()
            || !g.vectors.iter().zip(&m.vectors).all(|(a, b)| close(a, b))
        {
            let first = g
                .vectors
                .iter()
                .zip(&m.vectors)
                .position(|(a, b)| !close(a, b));
            out.push(format!(
                "motion {k}: {} vectors, GMC {}; first apart: {:?}",
                m.vectors.len(),
                g.vectors.len(),
                first.map(|i| (&m.vectors[i], &g.vectors[i]))
            ));
        }
        let outputs = g.state.get("$A").map(String::as_str);
        let hop = outputs == Some("40");
        let head_up = outputs.is_some_and(|a| a.ends_with('2') || a.ends_with('0'));
        let thin_start = g.state.get("#S").is_some_and(|s| s == "1000");
        for (key, want) in &g.state {
            let deliberate = (hop && key == "$A") || (head_up && thin_start && key == "#S");
            if ignore(g, key) || deliberate {
                continue;
            }
            let got = m.state.get(key);
            if got != Some(want) {
                out.push(format!(
                    "motion {k} ({}): {key} = {got:?}, GMC {want}",
                    g.vectors[0]
                ));
            }
        }
    }
    if mine_end.get("$A") != gmc_end.get("$A") {
        out.push(format!(
            "outputs at the end {:?}, GMC {:?}",
            mine_end.get("$A"),
            gmc_end.get("$A")
        ));
    }
    out
}

#[test]
fn gmcs_drawn_jobs_lower_to_what_gmc_sent() {
    for job in JOBS.iter().filter(|j| !j.template) {
        let diff = differences(job, [0, 0], |_, _| false);
        assert!(diff.is_empty(), "{}:\n  {}", job.capture, diff.join("\n  "));
    }
}

fn straight_down(m: &Motion) -> bool {
    m.state.get("$A").is_some_and(|a| a.ends_with('A'))
}

/// `g_regular_mount` is one of GMC's templates, and GMC starts the straight head on thin board
/// at `#S1000` only for drawn shapes (protocol reference §3.3). Programs don't say which a cut
/// came from, so this crate always uses the drawn shapes' start; everything else matches.
#[test]
fn gmcs_template_differs_only_in_the_thin_board_start() {
    let job = JOBS
        .iter()
        .find(|j| j.capture == "g_regular_mount")
        .expect("listed");
    assert!(
        !differences(job, [0, 0], |_, _| false).is_empty(),
        "the template should differ in #S"
    );
    let diff = differences(job, [0, 0], |m, key| key == "#S" && straight_down(m));
    assert!(diff.is_empty(), "{}", diff.join("\n"));
}

/// GMC's oval (a template too): a closed curve, 1,107 vectors over 1.9 laps, lowered with the
/// closed-curve head-down (`$A41`), the curve's dead time (`#C1,200`), the longer ramp for a
/// first segment just off the axis (`#R253`), the closed-curve trim, and the blade keeping
/// count past a revolution (Z from 5 to 5521). X and Y are within a step of GMC's (the fixture
/// is a fitted ellipse, above). Z differs by the blade rule: GMC sends each chord's direction at
/// its end, this crate the curve's tangent, halfway between the chords (PROGRAM_FORMAT §4.2);
/// on this oval ours is 0.3° ahead on average and never more than 0.72° apart.
#[test]
fn gmcs_oval_lowers_to_what_gmc_sent_within_the_blade_rule() {
    let job = JOBS
        .iter()
        .find(|j| j.capture == "g_oval_mount")
        .expect("listed");
    let under_a_degree = 8;
    let diff = differences(job, [1, under_a_degree], |m, key| {
        key == "#S" && straight_down(m)
    });
    assert!(diff.is_empty(), "{}", diff.join("\n"));
}
