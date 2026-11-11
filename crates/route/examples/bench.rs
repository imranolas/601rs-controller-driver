//! How much head-up travel the route optimiser saves on a sheet, and how long it takes.
//!
//! Each sheet is mats laid edge to edge on a 32 × 40 in board, each with one to three
//! openings. An opening's four bevel edges are cut clockwise, so each has one way; a mat's four
//! border edges are straight-head lines, cut either way, and only after the mat's openings.
//! The head starts at the park and goes back to it. Travel is the 601's: 900 mm/s, 4000 mm/s².
//!
//! cargo run --release -p route --example bench [seeds]

use std::time::Instant;

use route::{Job, Options, Problem, Route, Travel, Way};

const PARK: [f64; 2] = [1471.5, 198.1];
const TRAVEL: Travel = Travel {
    speed: 900.0,
    accel: 4000.0,
};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 33
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * (self.next() % 10_000) as f64 / 10_000.0
    }
}

/// Clockwise around a rectangle, from the bottom-left: one cut per edge.
fn edges(x0: f64, y0: f64, x1: f64, y1: f64) -> [([f64; 2], [f64; 2]); 4] {
    let (a, b, c, d) = ([x0, y0], [x0, y1], [x1, y1], [x1, y0]);
    [(a, b), (b, c), (c, d), (d, a)]
}

/// `mats` mats in rows across the sheet, and the order a fixed rule would cut them in: mat by
/// mat, each opening's edges and then the border, every cut its first way.
fn sheet(rng: &mut Rng, mats: usize) -> (Problem, Route) {
    let cols = (mats as f64).sqrt().ceil() as usize;
    let rows = mats.div_ceil(cols);
    let (w, h) = (1015.0 / cols as f64, 815.0 / rows as f64);
    let mut jobs: Vec<Job> = Vec::new();
    for m in 0..mats {
        let (x0, y0) = ((m % cols) as f64 * w, (m / cols) as f64 * h);
        let border = rng.range(0.12, 0.2) * w.min(h);
        let openings = 1 + rng.next() as usize % 3;
        let gap = rng.range(15.0, 30.0);
        let inner = w - 2.0 * border;
        let each = (inner - gap * (openings - 1) as f64) / openings as f64;
        let mut windows = Vec::new();
        for k in 0..openings {
            let ox = x0 + border + k as f64 * (each + gap);
            for (entry, exit) in edges(ox, y0 + border, ox + each, y0 + h - border) {
                windows.push(jobs.len());
                jobs.push(Job {
                    ways: vec![Way { entry, exit }],
                    flip: None,
                    after: Vec::new(),
                });
            }
        }
        for (entry, exit) in edges(x0, y0, x0 + w, y0 + h) {
            jobs.push(Job {
                ways: vec![
                    Way { entry, exit },
                    Way {
                        entry: exit,
                        exit: entry,
                    },
                ],
                flip: Some(vec![1, 0]),
                after: windows.clone(),
            });
        }
    }
    let n = jobs.len();
    let p = Problem::new(jobs, PARK, TRAVEL)
        .and_then(|p| p.with_end(Some(PARK)))
        .expect("a valid problem");
    let fixed = Route {
        order: (0..n).collect(),
        ways: vec![0; n],
    };
    assert!(p.is_legal(&fixed));
    (p, fixed)
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() {
    let seeds: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);
    println!("one sheet per size, {seeds} seeds each; travel is head-up seconds, park to park\n");
    println!(
        "| Mats | Cuts | Fixed order | Nearest first | Optimised | Saved vs nearest first | Median time | Slowest |"
    );
    println!(
        "|-----:|-----:|------------:|--------------:|----------:|-----------------------:|------------:|--------:|"
    );
    let mut rng = Rng(601);
    for mats in [1, 2, 4, 6, 9, 12] {
        let (p, fixed) = sheet(&mut rng, mats);
        let nearest = p.cost(&p.nearest_first());
        let (mut costs, mut times) = (Vec::new(), Vec::new());
        for seed in 1..=seeds {
            let start = Instant::now();
            let r = route::optimise(
                &p,
                Some(&fixed),
                &Options {
                    seed,
                    ..Options::default()
                },
            );
            times.push(start.elapsed().as_secs_f64() * 1000.0);
            assert!(p.is_legal(&r), "every route keeps the cutting rules");
            costs.push(p.cost(&r));
        }
        let slowest = times.iter().copied().fold(0.0, f64::max);
        let optimised = median(&mut costs);
        println!(
            "| {mats} | {} | {:.1} s | {:.1} s | {:.1} s | {:.0} % | {:.0} ms | {:.0} ms |",
            p.jobs().len(),
            p.cost(&fixed),
            nearest,
            optimised,
            100.0 * (1.0 - optimised / nearest),
            median(&mut times),
            slowest,
        );
    }
}
