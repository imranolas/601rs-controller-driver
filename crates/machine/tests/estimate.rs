//! The run-time estimate against GMC's captured jobs, and the line budget.

use std::path::Path;

use machine::{Capabilities, Estimate, Host, Profile, estimate, lower};
use program::Program;

fn fixture(name: &str) -> Program {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/programs")
        .join(format!("{name}.json"));
    Program::from_json(&std::fs::read_to_string(path).expect("fixture reads")).expect("a program")
}

fn run(name: &str, caps: Capabilities) -> Estimate {
    let profile = Profile {
        capabilities: caps,
        ..Profile::gunnar_601()
    };
    estimate(
        &lower(&fixture(name), &profile).expect("lowers"),
        &profile,
        &Host::default(),
    )
}

/// GMC's run times, from its first `$A42` to its last `$A02` in each capture (to the second),
/// and how much longer this crate's form of the job is: on thin board GMC hops between an
/// opening's edges with the bevel head down (3 hops a window), where this crate raises it and
/// lets it settle once more (250 ms).
const GMC: &[(&str, f64, f64)] = &[
    ("g_regular_mount", 17.0, 0.75),
    ("g_multi_C10", 24.0, 1.5),
    ("g_multi_C11", 29.0, 0.0),
    ("g_multi_C12", 52.0, 0.0),
    ("g_multi_Cx_20", 31.0, 1.5),
    ("g_multi_Cx_40", 24.0, 1.5),
    ("g_multi_Cx_60", 22.0, 1.5),
    ("g_multi_Cx_80", 21.0, 1.5),
    ("g_multi_Cx_100", 21.0, 1.5),
    ("g_oval_mount", 15.0, 0.0),
];

#[test]
fn the_estimate_is_within_an_eighth_of_gmcs_run_times() {
    let mut worst: (f64, &str) = (0.0, "");
    for &(name, gmc, extra) in GMC {
        let e = run(name, Capabilities::default());
        let off = (e.total_s - extra - gmc) / gmc;
        if off.abs() > worst.0.abs() {
            worst = (off, name);
        }
        assert!(
            off.abs() < 0.125,
            "{name}: {:.1} s, GMC {gmc} s (+{extra} s)",
            e.total_s
        );
        let parts = e.travel_s + e.cutting_s + e.waiting_s + e.talking_s;
        assert!((parts - e.total_s).abs() < 1e-9);
        let ops: f64 = e.ops.iter().map(|o| o.seconds).sum();
        assert!(
            ops < e.total_s && ops > 0.5 * e.total_s,
            "{name}: ops {ops:.1} of {:.1}",
            e.total_s
        );
    }
    eprintln!("worst: {} {:+.0}%", worst.1, worst.0 * 100.0);
}

#[test]
fn one_stream_per_cut_saves_the_round_trips() {
    for &(name, ..) in GMC {
        let (gmc, stream) = (
            run(name, Capabilities::default()),
            run(name, Capabilities::all()),
        );
        assert!(
            stream.total_s < gmc.total_s - 1.0,
            "{name}: {:.1} s against {:.1} s",
            stream.total_s,
            gmc.total_s
        );
        assert!(stream.talking_s < gmc.talking_s / 2.0, "{name}");
    }
}

/// GMC's oval, sent as GMC sends it (1,107 full vectors), needs well over what the line
/// carries: why GMC hesitates in curves. Compact paths halve it. (Flattened by chord
/// tolerance, the compiler's way, the same oval needs 425 vectors and fits.)
#[test]
fn the_line_budget_flags_gmcs_oval() {
    let profile = Profile::gunnar_601();
    let line = |caps| {
        run("g_oval_mount", caps)
            .ops
            .iter()
            .map(|o| o.line_share)
            .fold(0.0, f64::max)
    };
    let (full, compact) = (
        line(Capabilities::default()),
        line(Capabilities {
            compact_paths: true,
            ..Capabilities::default()
        }),
    );
    assert!(full > 2.0, "{full}");
    assert!(
        compact < 0.55 * full && compact > profile.line.budget_share,
        "{compact} against {full}"
    );
    for &(name, ..) in GMC.iter().filter(|(n, ..)| n.starts_with("g_multi")) {
        let straight = run(name, Capabilities::default())
            .ops
            .iter()
            .map(|o| o.line_share)
            .fold(0.0, f64::max);
        assert!(straight < 0.05, "{name}: {straight}");
    }
}
