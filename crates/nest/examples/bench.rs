//! Benchmark against reference results (PackingSolver's): one instance per line,
//!   kind name | bin w,h,trim,copies ... | item w,h,copies,rotate ... | reference ms
//! where kind is `sheets` (bin packing; the reference is a number of sheets) or `knapsack`
//! (the reference is the profit, area by default).
//!
//! cargo run -p nest --release --example bench -- instances.txt [width,width,...]

use std::time::Instant;

use nest::{Bin, Instance, Item, Objective, Options, Trims, check, solve};

struct Case {
    name: String,
    instance: Instance,
    /// Sheets (bin packing), profit (knapsack), or with offcuts first: full sheets used, and
    /// a second number, the bins used in all.
    reference: i64,
    reference_bins: Option<i64>,
    reference_ms: f64,
    /// The reference is the last bin used and how far its strips reach (PackingSolver's
    /// bin packing with leftovers).
    leftovers: bool,
}

fn numbers(s: &str) -> Vec<i64> {
    s.split(',').filter_map(|v| v.parse().ok()).collect()
}

fn parse(line: &str) -> Option<Case> {
    let mut parts = line.split('|').map(str::trim);
    let mut head = parts.next()?.split_whitespace();
    let kind = head.next()?;
    let objective = match kind {
        "sheets" | "offcuts" | "leftovers" => Objective::BinPacking,
        "knapsack" => Objective::Knapsack,
        _ => return None,
    };
    let name = head.next()?.to_string();
    let bins = parts
        .next()?
        .split_whitespace()
        .map(|b| {
            let v = numbers(b);
            let t = v[2] as i32;
            Bin {
                w: v[0] as i32,
                h: v[1] as i32,
                trims: Trims {
                    left: t,
                    right: t,
                    top: t,
                    bottom: t,
                },
                copies: v[3] as u32,
            }
        })
        .collect();
    let items = parts
        .next()?
        .split_whitespace()
        .map(|i| {
            let v = numbers(i);
            Item {
                w: v[0] as i32,
                h: v[1] as i32,
                copies: v[2] as u32,
                profit: None,
                rotate: v[3] == 1,
            }
        })
        .collect();
    let tail: Vec<f64> = parts
        .next()?
        .split_whitespace()
        .filter_map(|v| v.parse().ok())
        .collect();
    let (reference, reference_bins, reference_ms) = if kind == "offcuts" || kind == "leftovers" {
        (tail[0] as i64, Some(tail[1] as i64), tail[2])
    } else {
        (tail[0] as i64, None, tail[1])
    };
    let leftovers = kind == "leftovers";
    Some(Case {
        name,
        instance: Instance {
            objective,
            bins,
            items,
        },
        reference,
        reference_bins,
        reference_ms,
        leftovers,
    })
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let Some(path) = args.get(1) else {
        eprintln!("usage: bench instances.txt [width,width,...]");
        return;
    };
    let cases: Vec<Case> = std::fs::read_to_string(path)
        .expect("readable")
        .lines()
        .filter_map(parse)
        .collect();
    let widths: Vec<usize> = args.get(2).map_or(vec![2048], |s| {
        s.split(',').filter_map(|w| w.parse().ok()).collect()
    });
    let reference_ms: f64 = cases.iter().map(|c| c.reference_ms).sum();
    println!(
        "{} instances; PackingSolver took {:.0} ms in all",
        cases.len(),
        reference_ms
    );
    for &max_width in &widths {
        // GROWTH=2048 with a width of 2048: PackingSolver's not-anytime schedule (1, then 2048).
        let growth = std::env::var("GROWTH")
            .ok()
            .and_then(|g| g.parse().ok())
            .unwrap_or(Options::default().growth);
        let options = Options { max_width, growth };
        let (mut worse, mut better, mut ms_total, mut ms_max) =
            (Vec::new(), Vec::new(), 0.0, (0.0, String::new()));
        for c in &cases {
            let t = Instant::now();
            let s = solve(&c.instance, &options).expect("solves");
            let ms = t.elapsed().as_secs_f64() * 1000.0;
            if let Err(v) = check(&c.instance, &s) {
                panic!("{}: illegal layout: {v}", c.name);
            }
            let (ours, gain) = match (c.instance.objective, c.reference_bins) {
                (Objective::BinPacking, Some(ref_extent)) if c.leftovers => {
                    let last = s.placements.iter().map(|p| p.bin).max().unwrap_or(0);
                    let bottom = c.instance.bin_sequence()[last].trims.bottom;
                    let extent = s
                        .placements
                        .iter()
                        .filter(|p| p.bin == last)
                        .map(|p| p.y + p.h)
                        .max()
                        .unwrap_or(bottom)
                        - bottom;
                    let ours = last as i64 * 100_000 + extent as i64;
                    (ours, (c.reference * 100_000 + ref_extent) - ours)
                }
                (Objective::BinPacking, Some(ref_bins)) => {
                    // Offcuts first: full sheets used (the last bin type), then bins used in all.
                    let sheet_from = c.instance.bins[..c.instance.bins.len() - 1]
                        .iter()
                        .map(|b| b.copies as usize)
                        .sum::<usize>();
                    let mut used: Vec<usize> = s.placements.iter().map(|p| p.bin).collect();
                    used.sort_unstable();
                    used.dedup();
                    let sheets = used.iter().filter(|&&b| b >= sheet_from).count() as i64;
                    let bins = used.len() as i64;
                    let gain = (c.reference - sheets) * 1000 + (ref_bins - bins);
                    (sheets * 1000 + bins, gain)
                }
                (Objective::BinPacking, None) => {
                    (s.bins_used as i64, c.reference - s.bins_used as i64)
                }
                (Objective::Knapsack, _) => (s.profit, s.profit - c.reference),
            };
            if std::env::var("VERBOSE").is_ok() {
                println!(
                    "  {}: ours {} reference {} {:?}",
                    c.name, ours, c.reference, c.reference_bins
                );
            }
            if gain < 0 {
                worse.push(format!("{} ({} vs {})", c.name, ours, c.reference));
            } else if gain > 0 {
                better.push(c.name.clone());
            }
            ms_total += ms;
            if ms > ms_max.0 {
                ms_max = (ms, c.name.clone());
            }
        }
        println!(
            "max width {max_width}: worse on {}, better on {}, of {}; {:.0} ms in all, slowest {:.0} ms ({})",
            worse.len(),
            better.len(),
            cases.len(),
            ms_total,
            ms_max.0,
            ms_max.1
        );
        if !worse.is_empty() {
            println!("   worse: {}", worse.join(", "));
        }
    }
}
