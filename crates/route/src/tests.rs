use super::*;

const TRAVEL: Travel = Travel { speed: 900.0, accel: 3600.0 };

fn uniform(rng: &mut Rng, scale: f64) -> f64 {
    (rng.next() % 1_000_000) as f64 / 1_000_000.0 * scale
}

/// Jobs scattered on a sheet: bevel edges (one way), straight lines (either way round) and
/// closed curves (several start points), with a random precedence from lower to higher index.
fn random_jobs(seed: u64, n: usize) -> Vec<Job> {
    let mut rng = Rng(seed);
    let mut jobs = Vec::new();
    for i in 0..n {
        let c = [uniform(&mut rng, 800.0), uniform(&mut rng, 1000.0)];
        let ways = match rng.below(3) {
            0 => {
                let d = [uniform(&mut rng, 400.0) - 200.0, uniform(&mut rng, 400.0) - 200.0];
                vec![Way { entry: c, exit: [c[0] + d[0], c[1] + d[1]] }]
            }
            1 => {
                let d = [uniform(&mut rng, 400.0) - 200.0, uniform(&mut rng, 400.0) - 200.0];
                let e = [c[0] + d[0], c[1] + d[1]];
                vec![Way { entry: c, exit: e }, Way { entry: e, exit: c }]
            }
            _ => {
                let r = 30.0 + uniform(&mut rng, 120.0);
                let at = |k: usize| {
                    let a = k as f64 * std::f64::consts::TAU / 6.0;
                    [c[0] + r * a.cos(), c[1] + r * a.sin()]
                };
                (0..6).map(|k| Way { entry: at(k), exit: at(k + 1) }).collect()
            }
        };
        let flip = (ways.len() == 2).then(|| vec![1, 0]);
        let after = (0..i).filter(|_| rng.below(100) < 15).collect();
        jobs.push(Job { ways, flip, after });
    }
    jobs
}

fn random_problem(seed: u64, n: usize) -> Problem {
    Problem::new(random_jobs(seed, n), [0.0, 0.0], TRAVEL).expect("random problems are valid")
}

/// Every legal order, each with its exact best ways.
fn brute_force(p: &Problem) -> f64 {
    fn go(p: &Problem, order: &mut Vec<usize>, used: &mut Vec<bool>, best: &mut f64) {
        let n = p.jobs.len();
        if order.len() == n {
            *best = best.min(p.cost(&p.best_ways(order)));
            return;
        }
        for j in 0..n {
            if !used[j] && p.jobs[j].after.iter().all(|&q| used[q]) {
                used[j] = true;
                order.push(j);
                go(p, order, used, best);
                order.pop();
                used[j] = false;
            }
        }
    }
    let mut best = f64::INFINITY;
    go(p, &mut Vec::new(), &mut vec![false; p.jobs.len()], &mut best);
    best
}

#[test]
fn travel_time_follows_the_acceleration_model() {
    let t = |d: f64| TRAVEL.time([0.0, 0.0], [d, 0.0]);
    assert_eq!(t(0.0), 0.0);
    assert!((t(100.0) - 2.0 * (100.0f64 / 3600.0).sqrt()).abs() < 1e-12);
    assert!((t(900.0) - 1.25).abs() < 1e-12);
}

#[test]
fn finds_the_optimum_of_small_problems() {
    let mut misses = Vec::new();
    for seed in 0..200 {
        let p = random_problem(seed, 7);
        let best = brute_force(&p);
        let r = optimise(&p, None, &Options::default());
        assert!(p.is_legal(&r), "seed {seed}: illegal route");
        if p.cost(&r) > best + 1e-9 {
            misses.push(format!("seed {seed}: +{:.2}%", (p.cost(&r) / best - 1.0) * 100.0));
        }
    }
    assert!(misses.is_empty(), "{} of 200 missed the optimum: {}", misses.len(), misses.join(", "));
}

#[test]
fn best_ways_is_exact() {
    for seed in 0..20 {
        let p = random_problem(seed, 6);
        let order = p.topological();
        let dp = p.cost(&p.best_ways(&order));
        // Every combination of ways for this order.
        let mut best = f64::INFINITY;
        let mut ways = vec![0; 6];
        loop {
            best = best.min(p.cost(&Route { order: order.clone(), ways: ways.clone() }));
            let mut k = 0;
            while k < 6 {
                ways[k] += 1;
                if ways[k] < p.jobs[k].ways.len() {
                    break;
                }
                ways[k] = 0;
                k += 1;
            }
            if k == 6 {
                break;
            }
        }
        assert!((dp - best).abs() < 1e-9, "seed {seed}: DP {dp} against {best}");
    }
}

#[test]
fn larger_problems_stay_legal_and_never_get_worse() {
    for seed in 0..10 {
        let p = random_problem(100 + seed, 60);
        let given = p.nearest_first();
        let r = optimise(&p, Some(&given), &Options { seed, kicks: 200, ..Options::default() });
        assert!(p.is_legal(&r));
        assert!(p.cost(&r) <= p.cost(&given) + 1e-9);
    }
}

#[test]
fn the_same_seed_gives_the_same_route() {
    let p = random_problem(7, 40);
    let o = Options { seed: 3, kicks: 300, ..Options::default() };
    assert_eq!(optimise(&p, None, &o), optimise(&p, None, &o));
}

#[test]
fn an_illegal_given_route_is_ignored() {
    let p = random_problem(9, 12);
    let backwards = Route { order: (0..12).rev().collect(), ways: vec![0; 12] };
    let r = optimise(&p, Some(&backwards), &Options::default());
    assert!(p.is_legal(&r));
}

#[test]
fn rejects_bad_problems() {
    let way = Way { entry: [0.0, 0.0], exit: [1.0, 0.0] };
    let job = |after: Vec<usize>| Job { ways: vec![way], flip: None, after };
    assert_eq!(Problem::new(vec![job(vec![1]), job(vec![0])], [0.0, 0.0], TRAVEL).err(), Some(Error::Cycle));
    assert_eq!(Problem::new(vec![job(vec![5])], [0.0, 0.0], TRAVEL).err(), Some(Error::BadAfter(0)));
    assert_eq!(Problem::new(vec![Job::default()], [0.0, 0.0], TRAVEL).err(), Some(Error::NoWays(0)));
    let bad_flip = Job { ways: vec![way], flip: Some(vec![1]), after: vec![] };
    assert_eq!(Problem::new(vec![bad_flip], [0.0, 0.0], TRAVEL).err(), Some(Error::BadFlip(0)));
    let stuck = Travel { speed: 0.0, accel: 1.0 };
    assert_eq!(Problem::new(vec![job(vec![])], [0.0, 0.0], stuck).err(), Some(Error::BadTravel));
}

/// The flat encoding of a problem, as the browser builds it.
fn encode(jobs: &[Job], given: Option<&Route>, seed: u64, kicks: usize) -> Vec<f64> {
    let mut v = vec![FLAT_VERSION, 0.0, 0.0, 0.0, 0.0, 0.0, TRAVEL.speed, TRAVEL.accel, seed as f64, kicks as f64, jobs.len() as f64];
    for j in jobs {
        v.extend([j.ways.len() as f64, j.after.len() as f64, if j.flip.is_some() { 1.0 } else { 0.0 }]);
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
fn the_flat_encoding_gives_the_same_route() {
    let jobs = random_jobs(11, 30);
    let p = Problem::new(jobs.clone(), [0.0, 0.0], TRAVEL).expect("valid");
    let given = p.nearest_first();
    let out = solve_flat(&encode(&jobs, Some(&given), 5, 250)).expect("solves");
    let direct = optimise(&p, Some(&given), &Options { seed: 5, kicks: 250, ..Options::default() });
    assert_eq!(out.len(), 61);
    assert!((out[0] - p.cost(&direct)).abs() < 1e-12);
    let order: Vec<usize> = out[1..31].iter().map(|&v| v as usize).collect();
    let ways: Vec<usize> = out[31..].iter().map(|&v| v as usize).collect();
    assert_eq!(Route { order, ways }, direct);
}

#[test]
fn the_flat_encoding_rejects_bad_input() {
    let good = encode(&random_jobs(12, 5), None, 1, 10);
    assert!(solve_flat(&good).is_ok());
    let mut v = good.clone();
    v[0] = 3.0;
    assert_eq!(solve_flat(&v).err(), Some(Error::BadVersion));
    assert_eq!(solve_flat(&good[..good.len() - 1]).err(), Some(Error::BadLength));
    let mut v = good.clone();
    v.push(0.0);
    assert_eq!(solve_flat(&v).err(), Some(Error::BadLength));
    let mut v = good.clone();
    v[1] = f64::NAN;
    assert_eq!(solve_flat(&v).err(), Some(Error::BadNumber));
    let mut v = good;
    v[10] = 1e9; // more jobs than the input could hold
    assert_eq!(solve_flat(&v).err(), Some(Error::BadLength));
}

/// Every legal order with its best ways, counting the move to `end`.
fn brute_force_to(p: &Problem) -> f64 {
    brute_force(p)
}

#[test]
fn an_end_point_is_counted_and_chosen_for() {
    let mut misses = 0;
    for seed in 0..60 {
        let jobs = random_jobs(500 + seed, 7);
        let p = Problem::new(jobs, [0.0, 0.0], TRAVEL).expect("valid").with_end(Some([1400.0, 200.0])).expect("finite");
        let best = brute_force_to(&p);
        let r = optimise(&p, None, &Options::default());
        assert!(p.is_legal(&r));
        // The cost includes the final move.
        let mut at = [0.0, 0.0];
        let mut c = 0.0;
        for &j in &r.order {
            let w = p.jobs[j].ways[r.ways[j]];
            c += TRAVEL.time(at, w.entry);
            at = w.exit;
        }
        assert!((p.cost(&r) - c - TRAVEL.time(at, [1400.0, 200.0])).abs() < 1e-9);
        misses += usize::from(p.cost(&r) > best + 1e-9);
    }
    assert_eq!(misses, 0, "missed the optimum with an end point");
}

#[test]
fn version_1_input_still_reads() {
    let mut v = encode(&random_jobs(13, 5), None, 1, 10);
    v[0] = 1.0;
    v.drain(3..6);
    assert!(solve_flat(&v).is_ok());
}

