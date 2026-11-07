//! Benchmark the route optimiser on dumped problems, over several seeds.
//!
//! Each line of the input file: `id reference-cost value value ...`, where the values are a
//! problem as the browser sends it ([`solvers::route_flat`]). The reference is the best route
//! known (exact where the sheet is small enough).
//!
//! cargo run -p solvers --release --example route_bench -- problems.txt [seeds] [kicks,kicks,...] [all|near|watchers,...]

use std::time::Instant;

use route::{Look, Options, Problem, Route, Tuning};
use solvers::route_flat;

struct Case {
    id: String,
    reference: f64,
    problem: Problem,
    given: Option<Route>,
}

fn parse(line: &str) -> Option<Case> {
    let mut it = line.split_whitespace();
    let id = it.next()?.to_string();
    let reference: f64 = it.next()?.parse().ok()?;
    let values: Vec<f64> = it.map(|t| t.parse().ok()).collect::<Option<_>>()?;
    let input = route_flat::decode(&values).ok()?;
    Some(Case {
        id,
        reference,
        problem: input.problem,
        given: input.given,
    })
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let Some(path) = args.get(1) else {
        eprintln!("usage: bench problems.txt [seeds] [kicks,...] [all|near|watchers,...]");
        return;
    };
    let text = std::fs::read_to_string(path).expect("readable problems file");
    let cases: Vec<Case> = text.lines().filter_map(parse).collect();
    let seeds: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(8);
    let kicks: Vec<usize> = args.get(3).map_or(vec![0], |s| {
        s.split(',').filter_map(|k| k.parse().ok()).collect()
    });
    let looks: Vec<Look> = args.get(4).map_or(vec![Options::default().look], |s| {
        s.split(',')
            .filter_map(|l| match l {
                "all" => Some(Look::All),
                "near" => Some(Look::Near),
                "watchers" => Some(Look::Watchers),
                _ => None,
            })
            .collect()
    });
    // TUNE="heat=0.3,blink=0.05,ruin=10,run=3,string=0.5"
    let mut tuning = Tuning::default();
    for kv in std::env::var("TUNE")
        .unwrap_or_default()
        .split(',')
        .filter(|s| !s.is_empty())
    {
        let (k, v) = kv.split_once('=').expect("key=value");
        let v: f64 = v.parse().expect("a number");
        match k {
            "heat" => tuning.heat = v,
            "blink" => tuning.blink = v,
            "ruin" => tuning.max_ruin = v as usize,
            "run" => tuning.max_run = v as usize,
            "string" => tuning.string = v,
            "runs" => tuning.runs = v as usize,
            "bridge" => tuning.bridge = v,
            _ => panic!("unknown setting {k}"),
        }
    }
    println!("{} problems, {seeds} seeds each, {tuning:?}", cases.len());
    for &look in &looks {
        for &k in &kicks {
            let (mut sum_gap, mut worst, mut ms, mut misses, mut runs) = (0.0, 0.0f64, 0.0, 0, 0);
            let mut per: Vec<String> = Vec::new();
            for c in &cases {
                let (mut case_worst, mut case_ms) = (0.0f64, 0.0);
                for seed in 1..=seeds {
                    let t = Instant::now();
                    let r = route::optimise(
                        &c.problem,
                        c.given.as_ref(),
                        &Options {
                            seed,
                            kicks: k,
                            look,
                            tuning,
                        },
                    );
                    let dt = t.elapsed().as_secs_f64() * 1000.0;
                    assert!(c.problem.is_legal(&r), "{}: illegal route", c.id);
                    let gap = (c.problem.cost(&r) / c.reference - 1.0) * 100.0;
                    sum_gap += gap;
                    worst = worst.max(gap);
                    case_worst = case_worst.max(gap);
                    misses += usize::from(gap > 0.005);
                    ms += dt;
                    case_ms += dt;
                    runs += 1;
                }
                per.push(format!(
                    "{} {:+.2}% {:.0}ms",
                    c.id,
                    case_worst,
                    case_ms / seeds as f64
                ));
            }
            println!(
                "{look:?} kicks {k}: mean {:+.3}%, worst {:+.2}%, {misses}/{runs} runs off the reference, {:.0} ms per sheet set\n   worst per sheet: {}",
                sum_gap / runs as f64,
                worst,
                ms / seeds as f64,
                per.join(" | ")
            );
        }
    }
}
