//! How fast the nester lays out a queue, and how close it gets to the fewest sheets possible.
//!
//! Each queue is mats in common frame sizes, mixed at random, on 32 × 40 in (815 × 1015 mm)
//! sheets with a 5 mm trim at every edge, most mats free to turn. No layout can use fewer sheets
//! than the lower bound (`lower_bound`); it isn't always reachable, so over it isn't always worse
//! than best.
//!
//! cargo run --release -p nest --example queues [queues per size]

use std::time::Instant;

use nest::{Bin, Instance, Item, Len, Objective, Options, Trims, check, solve};

/// Common mat sizes, tenths of a millimetre: 8 × 10, 11 × 14 and 16 × 20 in, A4, A3, 30 × 40,
/// 40 × 50, 50 × 70 and 60 × 80 cm, and a landscape 25 × 20 cm.
const SIZES: [(Len, Len); 10] = [
    (2030, 2540),
    (2790, 3560),
    (4060, 5080),
    (2100, 2970),
    (2970, 4200),
    (3000, 4000),
    (4000, 5000),
    (5000, 7000),
    (6000, 8000),
    (2500, 2000),
];
const SHEET: (Len, Len, Len) = (8150, 10150, 50);

/// A small deterministic generator, so every run benchmarks the same queues.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 33
    }
}

fn queue(rng: &mut Rng, mats: u32) -> Instance {
    let mut items: Vec<Item> = Vec::new();
    let mut left = mats;
    while left > 0 {
        let (w, h) = SIZES[rng.next() as usize % SIZES.len()];
        let copies = (1 + rng.next() as u32 % 6).min(left);
        left -= copies;
        items.push(Item {
            w,
            h,
            copies,
            profit: None,
            rotate: rng.next() % 4 != 0,
        });
    }
    let (w, h, trim) = SHEET;
    Instance {
        objective: Objective::BinPacking,
        bins: vec![Bin {
            w,
            h,
            trims: Trims {
                left: trim,
                right: trim,
                top: trim,
                bottom: trim,
            },
            copies: mats,
        }],
        items,
    }
}

/// Fewer sheets than this can't hold the queue: the sheets the mats' area needs, and one each for
/// the mats too big to share a sheet with each other (over half the sheet each way, either way
/// round), whichever is more.
fn lower_bound(inst: &Instance) -> usize {
    let (w, h, trim) = SHEET;
    let (uw, uh) = (w - 2 * trim, h - 2 * trim);
    let area: f64 = inst
        .items
        .iter()
        .map(|i| (i.w * i.h) as f64 * f64::from(i.copies))
        .sum();
    let by_area = (area / (uw * uh) as f64).ceil() as usize;
    let big = |a: Len, b: Len| 2 * a > uw && 2 * b > uh;
    let alone: u32 = inst
        .items
        .iter()
        .filter(|i| big(i.w, i.h) && (!i.rotate || big(i.h, i.w) || i.h > uw || i.w > uh))
        .map(|i| i.copies)
        .sum();
    by_area.max(alone as usize)
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() {
    let queues: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(10);
    println!(
        "{queues} random queues per size, 32 × 40 in sheets, default options (beam up to 1024)\n"
    );
    println!(
        "| Mats | Sheets used | Lower bound | At the bound | Sheet use | Median time | Slowest |"
    );
    println!(
        "|-----:|------------:|-----------:|---------------:|----------:|------------:|--------:|"
    );
    let mut rng = Rng(2026);
    for mats in [8, 20, 40, 80, 160] {
        let (mut used, mut bound, mut at, mut area, mut sheet_area) = (0, 0, 0, 0.0, 0.0);
        let mut times = Vec::new();
        for _ in 0..queues {
            let inst = queue(&mut rng, mats);
            let start = Instant::now();
            let s = solve(&inst, &Options::default()).expect("solves");
            times.push(start.elapsed().as_secs_f64() * 1000.0);
            assert_eq!(check(&inst, &s), Ok(()), "every layout is legal");
            let b = lower_bound(&inst);
            used += s.bins_used;
            bound += b;
            at += usize::from(s.bins_used == b);
            let (w, h, trim) = SHEET;
            area += inst
                .items
                .iter()
                .map(|i| (i.w * i.h) as f64 * f64::from(i.copies))
                .sum::<f64>();
            sheet_area += ((w - 2 * trim) * (h - 2 * trim)) as f64 * s.bins_used as f64;
        }
        let slowest = times.iter().copied().fold(0.0, f64::max);
        println!(
            "| {mats} | {:.1} | {:.1} | {at} of {queues} | {:.0} % | {:.0} ms | {:.0} ms |",
            used as f64 / queues as f64,
            bound as f64 / queues as f64,
            100.0 * area / sheet_area,
            median(&mut times),
            slowest,
        );
    }
}
