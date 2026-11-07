//! Benchmark the optimiser on dumped problems, over several seeds.
//!
//! Each line of the input file: `id reference-cost value value ...`, where the values are the
//! flat encoding of a problem (see `route::solve_flat`). The reference is the best route known
//! (exact where the sheet is small enough).
//!
//! cargo run -p route --release --example bench -- problems.txt [seeds] [kicks,kicks,...] [all|near|watchers,...]

use std::time::Instant;

use route::{Job, Look, Options, Problem, Route, Travel, Tuning, Way};

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
    let v: Vec<f64> = it.map(|t| t.parse().ok()).collect::<Option<_>>()?;
    let mut at = 0;
    let mut num = || {
        at += 1;
        v.get(at - 1).copied()
    };
    let version = num()?;
    let start = [num()?, num()?];
    let end = if version >= 2.0 {
        let has = num()? == 1.0;
        let e = [num()?, num()?];
        has.then_some(e)
    } else {
        None
    };
    let travel = Travel { speed: num()?, accel: num()? };
    let (_seed, _kicks) = (num()?, num()?);
    let n = num()? as usize;
    let mut jobs = Vec::with_capacity(n);
    for _ in 0..n {
        let (m, a, flip) = (num()? as usize, num()? as usize, num()? == 1.0);
        let mut ways = Vec::with_capacity(m);
        for _ in 0..m {
            ways.push(Way { entry: [num()?, num()?], exit: [num()?, num()?] });
        }
        let flip = if flip { Some((0..m).map(|_| num().map(|x| x as usize)).collect::<Option<Vec<_>>>()?) } else { None };
        let after = (0..a).map(|_| num().map(|x| x as usize)).collect::<Option<Vec<_>>>()?;
        jobs.push(Job { ways, flip, after });
    }
    let given = if num()? == 1.0 {
        let order = (0..n).map(|_| num().map(|x| x as usize)).collect::<Option<Vec<_>>>()?;
        let ways = (0..n).map(|_| num().map(|x| x as usize)).collect::<Option<Vec<_>>>()?;
        Some(Route { order, ways })
    } else {
        None
    };
    let problem = Problem::new(jobs, start, travel).ok()?.with_end(end).ok()?;
    Some(Case { id, reference, problem, given })
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
    let kicks: Vec<usize> = args.get(3).map_or(vec![0], |s| s.split(',').filter_map(|k| k.parse().ok()).collect());
    let looks: Vec<Look> = args.get(4).map_or(vec![Look::Watchers], |s| {
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
    for kv in std::env::var("TUNE").unwrap_or_default().split(',').filter(|s| !s.is_empty()) {
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
                    let r = route::optimise(&c.problem, c.given.as_ref(), &Options { seed, kicks: k, look, tuning });
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
                per.push(format!("{} {:+.2}% {:.0}ms", c.id, case_worst, case_ms / seeds as f64));
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
