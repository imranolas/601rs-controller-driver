use nest::{Bin, Instance, Item, Len, Objective, Trims};
use route::{Job, Options, Problem, Route, Travel, Way, optimise};

use super::*;

const TRAVEL: Travel = Travel {
    speed: 900.0,
    accel: 3600.0,
};

/// Cuts of every kind across a sheet: bevel edges (one way), straight lines (either way
/// round) and closed curves (four start points), some after others.
fn jobs(n: usize) -> Vec<Job> {
    (0..n)
        .map(|i| {
            let (x, y) = ((i * 137 % 800) as f64, (i * 251 % 1000) as f64);
            let (a, b) = ([x, y], [x + 120.0, y + 40.0]);
            let ways = match i % 3 {
                0 => vec![Way { entry: a, exit: b }],
                1 => vec![Way { entry: a, exit: b }, Way { entry: b, exit: a }],
                _ => (0..4)
                    .map(|k| {
                        let p = [x + 30.0 * k as f64, y];
                        Way { entry: p, exit: p }
                    })
                    .collect(),
            };
            let flip = (i % 3 == 1).then(|| vec![1, 0]);
            let after = if i >= 3 && i % 4 == 0 {
                vec![i - 3]
            } else {
                vec![]
            };
            Job { ways, flip, after }
        })
        .collect()
}

/// A problem as the browser encodes it.
fn encode_route(jobs: &[Job], given: Option<&Route>, seed: u64, kicks: usize) -> Vec<f64> {
    let mut v = vec![
        route_flat::VERSION,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        TRAVEL.speed,
        TRAVEL.accel,
    ];
    v.extend([seed as f64, kicks as f64, jobs.len() as f64]);
    for j in jobs {
        v.extend([
            j.ways.len() as f64,
            j.after.len() as f64,
            if j.flip.is_some() { 1.0 } else { 0.0 },
        ]);
        for w in &j.ways {
            v.extend([w.entry[0], w.entry[1], w.exit[0], w.exit[1]]);
        }
        if let Some(f) = &j.flip {
            v.extend(f.iter().map(|&w| w as f64));
        }
        v.extend(j.after.iter().map(|&p| p as f64));
    }
    match given {
        Some(r) => {
            v.push(1.0);
            v.extend(r.order.iter().map(|&j| j as f64));
            v.extend(r.ways.iter().map(|&w| w as f64));
        }
        None => v.push(0.0),
    }
    v
}

#[test]
fn the_route_format_gives_the_same_route() {
    let jobs = jobs(30);
    let p = Problem::new(jobs.clone(), [0.0, 0.0], TRAVEL).expect("valid");
    let given = p.nearest_first();
    let out = route_flat::solve(&encode_route(&jobs, Some(&given), 5, 250)).expect("solves");
    let direct = optimise(
        &p,
        Some(&given),
        &Options {
            seed: 5,
            kicks: 250,
            ..Options::default()
        },
    );
    assert_eq!(out.len(), 61);
    assert!((out[0] - p.cost(&direct)).abs() < 1e-12);
    let order: Vec<usize> = out[1..31].iter().map(|&v| v as usize).collect();
    let ways: Vec<usize> = out[31..].iter().map(|&v| v as usize).collect();
    assert_eq!(Route { order, ways }, direct);
}

#[test]
fn the_route_format_rejects_bad_input() {
    let good = encode_route(&jobs(5), None, 1, 10);
    assert!(route_flat::solve(&good).is_ok());
    let mut v = good.clone();
    v[0] = 3.0;
    assert_eq!(route_flat::solve(&v).err(), Some(Error::BadVersion));
    assert_eq!(
        route_flat::solve(&good[..good.len() - 1]).err(),
        Some(Error::BadLength)
    );
    let mut v = good.clone();
    v.push(0.0);
    assert_eq!(route_flat::solve(&v).err(), Some(Error::BadLength));
    let mut v = good.clone();
    v[1] = f64::NAN;
    assert_eq!(route_flat::solve(&v).err(), Some(Error::BadNumber));
    let mut v = good;
    v[10] = 1e9; // more jobs than the input could hold
    assert_eq!(route_flat::solve(&v).err(), Some(Error::BadLength));
    let way = Way {
        entry: [0.0, 0.0],
        exit: [1.0, 0.0],
    };
    let cycle = [
        Job {
            ways: vec![way],
            flip: None,
            after: vec![1],
        },
        Job {
            ways: vec![way],
            flip: None,
            after: vec![0],
        },
    ];
    assert_eq!(
        route_flat::solve(&encode_route(&cycle, None, 1, 10)).err(),
        Some(Error::Route(route::Error::Cycle))
    );
}

#[test]
fn route_format_version_1_still_reads() {
    let mut v = encode_route(&jobs(5), None, 1, 10);
    v[0] = 1.0;
    v.drain(3..6);
    assert!(route_flat::solve(&v).is_ok());
}

fn sheet(w: Len, h: Len, trim: Len, copies: u32) -> Bin {
    Bin {
        w,
        h,
        trims: Trims {
            left: trim,
            right: trim,
            top: trim,
            bottom: trim,
        },
        copies,
    }
}

fn item(w: Len, h: Len, copies: u32, rotate: bool) -> Item {
    Item {
        w,
        h,
        copies,
        profit: None,
        rotate,
    }
}

/// An instance as the browser encodes it.
fn encode_nest(inst: &Instance) -> Vec<f64> {
    let knapsack = if inst.objective == Objective::Knapsack {
        1.0
    } else {
        0.0
    };
    let mut v = vec![
        nest_flat::VERSION,
        knapsack,
        0.0,
        0.0,
        inst.bins.len() as f64,
    ];
    for b in &inst.bins {
        let t = b.trims;
        v.extend([b.w, b.h, t.left, t.right, t.top, t.bottom].map(f64::from));
        v.push(b.copies as f64);
    }
    v.push(inst.items.len() as f64);
    for i in &inst.items {
        let profit = i.profit.map_or(-1.0, |p| p as f64);
        v.extend([
            i.w as f64,
            i.h as f64,
            i.copies as f64,
            profit,
            if i.rotate { 1.0 } else { 0.0 },
        ]);
    }
    v
}

#[test]
fn the_nest_format_gives_the_same_layout() {
    let inst = Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(4000, 5000, 50, 1), sheet(8150, 10150, 50, 6)],
        items: vec![
            item(4000, 5000, 2, true),
            item(3000, 2000, 3, true),
            item(2030, 2540, 4, false),
        ],
    };
    let direct = nest::solve(&inst, &nest::Options::default()).expect("solves");
    let out = nest_flat::solve(&encode_nest(&inst)).expect("solves");
    assert_eq!(out[0] as usize, direct.bins_used);
    assert_eq!(out[1] as i64, direct.profit);
    assert_eq!(out[2] as usize, direct.placements.len());
    for (k, p) in direct.placements.iter().enumerate() {
        let rotated = if p.rotated { 1.0 } else { 0.0 };
        let want = [
            p.item as f64,
            p.bin as f64,
            p.x as f64,
            p.y as f64,
            p.w as f64,
            p.h as f64,
            rotated,
        ];
        assert_eq!(&out[3 + 7 * k..3 + 7 * k + 7], want);
    }
}

fn small_nest() -> Instance {
    Instance {
        objective: Objective::BinPacking,
        bins: vec![sheet(1000, 1000, 0, 2)],
        items: vec![item(400, 500, 2, true)],
    }
}

#[test]
fn the_nest_format_rejects_bad_input() {
    let good = encode_nest(&small_nest());
    assert!(nest_flat::solve(&good).is_ok());
    let mut v = good.clone();
    v[0] = 2.0;
    assert_eq!(nest_flat::solve(&v).err(), Some(Error::BadVersion));
    assert_eq!(
        nest_flat::solve(&good[..good.len() - 1]).err(),
        Some(Error::BadLength)
    );
    let mut v = good.clone();
    v.push(0.0);
    assert_eq!(nest_flat::solve(&v).err(), Some(Error::BadLength));
    let mut v = good.clone();
    v[5] = 10.5; // a bin width
    assert_eq!(nest_flat::solve(&v).err(), Some(Error::BadNumber));
    let mut v = good;
    v[13] = 5000.0; // an item too big for the bins
    assert_eq!(
        nest_flat::solve(&v).err(),
        Some(Error::Nest(nest::Error::TooBig(0)))
    );
}

#[test]
fn every_refusal_has_its_own_code() {
    let all: Vec<Error> = vec![
        Error::BadNumber,
        Error::BadLength,
        Error::BadVersion,
        route::Error::NoWays(0).into(),
        route::Error::BadAfter(0).into(),
        route::Error::BadFlip(0).into(),
        route::Error::Cycle.into(),
        route::Error::BadTravel.into(),
        route::Error::NotFinite.into(),
        nest::Error::BadSize.into(),
        nest::Error::NoBins.into(),
        nest::Error::TooBig(0).into(),
        nest::Error::TooMany.into(),
        nest::Error::Bug(nest::Violation::Totals).into(),
    ];
    let mut codes: Vec<i32> = all.iter().map(Error::code).collect();
    assert!(codes.iter().all(|&c| c >= 2), "-1 is the export's own");
    codes.sort_unstable();
    codes.dedup();
    assert_eq!(codes.len(), all.len());
}

#[test]
#[allow(unsafe_code)]
fn the_exports_copy_the_answer_out_or_refuse() {
    let input = encode_nest(&small_nest());
    let want = nest_flat::solve(&input).expect("solves");
    let (inp, out, room) = (solvers_alloc(input.len()), solvers_alloc(64), 64);
    // SAFETY: both buffers come from solvers_alloc with these lengths and are distinct.
    unsafe {
        std::slice::from_raw_parts_mut(inp, input.len()).copy_from_slice(&input);
        let written = nest_solve(inp, input.len(), out, room);
        assert_eq!(written as usize, want.len());
        assert_eq!(std::slice::from_raw_parts(out, want.len()), want);
        assert_eq!(
            nest_solve(inp, input.len(), out, 2),
            -1,
            "too small an output"
        );
        *inp = 9.0;
        assert_eq!(
            nest_solve(inp, input.len(), out, room),
            -Error::BadVersion.code()
        );
        solvers_free(inp, input.len());
        solvers_free(out, room);
    }
}
