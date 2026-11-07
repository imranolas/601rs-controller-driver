//! The route of a sheet's cuts: which cut comes next and which way round it is cut, so the head
//! spends least time travelling between cuts, within the rules on what must be cut first.
//!
//! Every cut is a [`Job`] with one or more [`Way`]s to cut it: a bevel edge one way only
//! (clockwise, so the bevel faces the waste), a straight-head line from either end, a closed
//! curve from any of several start points. Jobs carry precedence: everything inside an opening
//! before the opening, openings before outer edges, the clamp rule. So this is a sequential
//! ordering problem with a choice of entry per job.
//!
//! The search is iterated local search, built from the parts of LKH that matter at this size
//! (tens of cuts, not thousands):
//! - candidate lists: for each job, the jobs whose way in lies nearest its way out, and the
//!   other way round;
//! - moves that keep every cut's direction: carry a short run of cuts next to a near neighbour,
//!   let two adjacent runs of any length swap places (or-3opt), reverse a run of straight lines
//!   (each cut the other way round), change one cut's way; first improvement;
//! - after each descent, the exact best ways for the order, by dynamic programming;
//! - kicks: ruin and recreate. Take out a cut and some of its neighbours and put each back
//!   where it costs least, at any legal place and way. A kicked route is kept if it is better,
//!   or (early on) only slightly worse.
//!
//! Every move keeps the precedence, so a route is always legal. It is seeded, so the same input
//! always gives the same route, and it starts from the given route, so it is never worse.

use std::fmt;

/// A point on the sheet, in millimetres.
pub type Pt = [f64; 2];

/// One way to cut a job: where the head goes down, and where it comes up.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Way {
    pub entry: Pt,
    pub exit: Pt,
}

/// One cut.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Job {
    pub ways: Vec<Way>,
    /// For each way, the way that cuts the same path the other way round (straight lines).
    pub flip: Option<Vec<usize>>,
    /// Jobs that must be cut before this one.
    pub after: Vec<usize>,
}

/// An order of the jobs, and the way each job is cut (indexed by job).
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    pub order: Vec<usize>,
    pub ways: Vec<usize>,
}

/// Head-up travel time: a move of `d` mm takes 2√(d/a) while the head never reaches full speed,
/// else d/v + v/a.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Travel {
    /// mm/s
    pub speed: f64,
    /// mm/s²
    pub accel: f64,
}

impl Travel {
    pub fn time(&self, a: Pt, b: Pt) -> f64 {
        let d = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        if d < self.speed * self.speed / self.accel {
            2.0 * (d / self.accel).sqrt()
        } else {
            d / self.speed + self.speed / self.accel
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// A job has no way to cut it.
    NoWays(usize),
    /// A job must come after a job that doesn't exist, or after itself.
    BadAfter(usize),
    /// A job's flip list is the wrong length or names a way it doesn't have.
    BadFlip(usize),
    /// The precedence has a cycle, so no order can keep it.
    Cycle,
    /// The travel model needs a positive speed and acceleration.
    BadTravel,
    /// A point isn't finite.
    NotFinite,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::NoWays(j) => write!(f, "job {j} has no way to cut it"),
            Error::BadAfter(j) => write!(
                f,
                "job {j} must come after a job that doesn't exist, or itself"
            ),
            Error::BadFlip(j) => write!(f, "job {j} has a bad flip list"),
            Error::Cycle => write!(f, "the precedence has a cycle"),
            Error::BadTravel => write!(
                f,
                "the travel model needs a positive speed and acceleration"
            ),
            Error::NotFinite => write!(f, "a point isn't finite"),
        }
    }
}

impl std::error::Error for Error {}

/// Improvements smaller than this (seconds) don't count, so the search can't cycle on rounding.
const EPS: f64 = 1e-9;

/// A checked ordering problem.
#[derive(Debug, Clone)]
pub struct Problem {
    jobs: Vec<Job>,
    start: Pt,
    /// Where the head goes after the last cut (the park position), if the route should count
    /// that move; without it the route ends at the last cut.
    end: Option<Pt>,
    travel: Travel,
    /// For each job, the jobs that must come after it.
    succs: Vec<Vec<usize>>,
}

impl Problem {
    pub fn new(mut jobs: Vec<Job>, start: Pt, travel: Travel) -> Result<Self, Error> {
        let n = jobs.len();
        if !(travel.speed > 0.0
            && travel.accel > 0.0
            && travel.speed.is_finite()
            && travel.accel.is_finite())
        {
            return Err(Error::BadTravel);
        }
        if !(start[0].is_finite() && start[1].is_finite()) {
            return Err(Error::NotFinite);
        }
        for (i, j) in jobs.iter_mut().enumerate() {
            if j.ways.is_empty() {
                return Err(Error::NoWays(i));
            }
            if j.ways
                .iter()
                .any(|w| !w.entry.iter().chain(&w.exit).all(|v| v.is_finite()))
            {
                return Err(Error::NotFinite);
            }
            if j.after.iter().any(|&p| p >= n || p == i) {
                return Err(Error::BadAfter(i));
            }
            if let Some(f) = &j.flip {
                if f.len() != j.ways.len() || f.iter().any(|&w| w >= j.ways.len()) {
                    return Err(Error::BadFlip(i));
                }
            }
            j.after.sort_unstable();
            j.after.dedup();
        }
        let mut succs = vec![Vec::new(); n];
        for (i, j) in jobs.iter().enumerate() {
            for &p in &j.after {
                succs[p].push(i);
            }
        }
        let p = Self {
            jobs,
            start,
            end: None,
            travel,
            succs,
        };
        if p.topological().len() < n {
            return Err(Error::Cycle);
        }
        Ok(p)
    }

    /// Count the move from the last cut to `end` (the park position) too, so the route also
    /// chooses how it finishes.
    pub fn with_end(mut self, end: Option<Pt>) -> Result<Self, Error> {
        if end.is_some_and(|e| !(e[0].is_finite() && e[1].is_finite())) {
            return Err(Error::NotFinite);
        }
        self.end = end;
        Ok(self)
    }

    pub fn jobs(&self) -> &[Job] {
        &self.jobs
    }

    /// Head-up travel time from the start through the route (and on to the end, if set).
    pub fn cost(&self, r: &Route) -> f64 {
        self.cost_of(&r.order, &r.ways)
    }

    fn cost_of(&self, order: &[usize], ways: &[usize]) -> f64 {
        let mut at = self.start;
        let mut c = 0.0;
        for &j in order {
            let w = self.jobs[j].ways[ways[j]];
            c += self.travel.time(at, w.entry);
            at = w.exit;
        }
        c + self.finish(at)
    }

    /// The move from where the head comes up last to the end, if there is one.
    fn finish(&self, at: Pt) -> f64 {
        self.end.map_or(0.0, |e| self.travel.time(at, e))
    }

    /// Every job once, each way one it has, and the precedence kept.
    pub fn is_legal(&self, r: &Route) -> bool {
        let n = self.jobs.len();
        if r.order.len() != n || r.ways.len() != n {
            return false;
        }
        let mut pos = vec![usize::MAX; n];
        for (i, &j) in r.order.iter().enumerate() {
            if j >= n || pos[j] != usize::MAX {
                return false;
            }
            pos[j] = i;
        }
        (0..n).all(|j| {
            r.ways[j] < self.jobs[j].ways.len()
                && self.jobs[j].after.iter().all(|&p| pos[p] < pos[j])
        })
    }

    /// The best way to cut each job for a fixed order: exact, by dynamic programming.
    pub fn best_ways(&self, order: &[usize]) -> Route {
        let mut ways = vec![0; self.jobs.len()];
        let Some(&first) = order.first() else {
            return Route {
                order: Vec::new(),
                ways,
            };
        };
        let mut cost: Vec<f64> = self.jobs[first]
            .ways
            .iter()
            .map(|w| self.travel.time(self.start, w.entry))
            .collect();
        let mut back: Vec<Vec<usize>> = vec![vec![0; cost.len()]];
        for k in 1..order.len() {
            let prev = &self.jobs[order[k - 1]].ways;
            let cur = &self.jobs[order[k]].ways;
            let mut next = vec![f64::INFINITY; cur.len()];
            let mut b = vec![0; cur.len()];
            for (wi, w) in cur.iter().enumerate() {
                for (pi, p) in prev.iter().enumerate() {
                    let c = cost[pi] + self.travel.time(p.exit, w.entry);
                    if c < next[wi] {
                        next[wi] = c;
                        b[wi] = pi;
                    }
                }
            }
            cost = next;
            back.push(b);
        }
        let last = &self.jobs[order[order.len() - 1]].ways;
        let total = |i: usize| cost[i] + self.finish(last[i].exit);
        let mut wi = 0;
        for i in 1..cost.len() {
            if total(i) < total(wi) {
                wi = i;
            }
        }
        for k in (0..order.len()).rev() {
            ways[order[k]] = wi;
            wi = back[k][wi];
        }
        Route {
            order: order.to_vec(),
            ways,
        }
    }

    /// Build a route job by job: `pick` chooses one of the ready jobs (by its place in
    /// `ready`) and a way for it, given where the head is. A cycle in the precedence ends the
    /// route early.
    fn build(&self, mut pick: impl FnMut(&[usize], Pt) -> (usize, usize)) -> Route {
        let n = self.jobs.len();
        let mut waiting: Vec<usize> = self.jobs.iter().map(|j| j.after.len()).collect();
        let mut ready: Vec<usize> = (0..n).filter(|&j| waiting[j] == 0).collect();
        let mut order = Vec::with_capacity(n);
        let mut ways = vec![0; n];
        let mut at = self.start;
        while !ready.is_empty() {
            let (r, w) = pick(&ready, at);
            let j = ready.swap_remove(r);
            order.push(j);
            ways[j] = w;
            at = self.jobs[j].ways[w].exit;
            for &s in &self.succs[j] {
                waiting[s] -= 1;
                if waiting[s] == 0 {
                    ready.push(s);
                }
            }
        }
        Route { order, ways }
    }

    /// An order that keeps the precedence: lowest-numbered ready job first.
    fn topological(&self) -> Vec<usize> {
        let lowest = |ready: &[usize], _: Pt| {
            (
                ready
                    .iter()
                    .enumerate()
                    .min_by_key(|e| e.1)
                    .map_or(0, |e| e.0),
                0,
            )
        };
        self.build(lowest).order
    }

    /// Nearest ready job next, by its nearest way in.
    pub fn nearest_first(&self) -> Route {
        self.build(|ready, at| {
            let (mut best, mut cost) = ((0, 0), f64::INFINITY);
            for (r, &j) in ready.iter().enumerate() {
                for (w, way) in self.jobs[j].ways.iter().enumerate() {
                    let c = self.travel.time(at, way.entry);
                    if c < cost {
                        (best, cost) = ((r, w), c);
                    }
                }
            }
            best
        })
    }
}

/// How hard, and how, to search. The defaults are tuned on the benchmark sheets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    pub seed: u64,
    /// Kicks in all (shared between the starts); 0 picks a number from the size of the sheet.
    pub kicks: usize,
    /// Which jobs the descent looks at again after a change.
    pub look: Look,
    pub tuning: Tuning,
}

/// Search settings, for benchmarking; the defaults are the tuned values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    /// Acceptance temperature of kicked routes at the start, as a share of the route's mean
    /// hop; it cools to a hundredth of that.
    pub heat: f64,
    /// Chance of skipping each place when a job is put back ("blinks"), so the same ruin
    /// doesn't always rebuild the same way.
    pub blink: f64,
    /// Most jobs taken out by one kick.
    pub max_ruin: usize,
    /// Longest run carried by the or-opt move.
    pub max_run: usize,
    /// Chance that a kick takes out a stretch of the route rather than a patch of the sheet.
    pub string: f64,
    /// Independent runs sharing the kicks: they start in turn from the given route and from
    /// a nearest-first route, each with its own seed.
    pub runs: usize,
    /// Chance that a kick is a double bridge (two random adjacent stretches swap places)
    /// rather than ruin and recreate.
    pub bridge: f64,
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            heat: 1.0,
            blink: 0.05,
            max_ruin: 30,
            max_run: 3,
            string: 0.8,
            runs: 4,
            bridge: 0.2,
        }
    }
}

/// Which jobs the descent looks at again after a change ("don't-look bits").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Look {
    /// Every job, every pass, until a pass finds nothing.
    All,
    /// The jobs either side of each change.
    Near,
    /// Those, and the jobs that have them in their candidate lists.
    Watchers,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            seed: 1,
            kicks: 0,
            look: Look::Near,
            tuning: Tuning::default(),
        }
    }
}

/// The automatic number of kicks for `n` jobs: 60 per job, 800 to 2,500. On the benchmark
/// sheets that reaches the reference in every run (24 seeds each); fewer kicks leave the odd
/// run stuck on the mid-size sheets.
pub fn auto_kicks(n: usize) -> usize {
    (60 * n).clamp(800, 2500)
}

/// The order and ways that keep head-up travel time least. Starts from `from` (if it is legal)
/// and from a nearest-first order, and returns the best route found, never worse than `from`.
pub fn optimise(p: &Problem, from: Option<&Route>, o: &Options) -> Route {
    let n = p.jobs.len();
    let given = match from {
        Some(r) if p.is_legal(r) => r.clone(),
        _ => Route {
            order: p.topological(),
            ways: vec![0; n],
        },
    };
    let mut best = given.clone();
    let mut best_cost = p.cost(&best);
    let settled = p.best_ways(&given.order);
    if p.cost(&settled) < best_cost - EPS {
        best_cost = p.cost(&settled);
        best = settled.clone();
    }
    if n < 2 {
        return best;
    }
    let cands = Cands::new(p, 10);
    let starts = [settled, p.nearest_first()];
    let kicks = if o.kicks == 0 { auto_kicks(n) } else { o.kicks };
    let runs = o.tuning.runs.max(1);
    for run in 0..runs {
        let init = &starts[run % starts.len()];
        let mut s = Search::new(
            p,
            &cands,
            init,
            o.seed.wrapping_add(run as u64),
            o.look,
            o.tuning,
        );
        let r = s.iterate(kicks / runs);
        let c = p.cost(&r);
        if c < best_cost - EPS {
            best_cost = c;
            best = r;
        }
    }
    best
}

/// Candidate lists, worked out once per problem.
#[derive(Debug)]
struct Cands {
    /// For each job: the jobs whose way in lies nearest one of its ways out.
    next: Vec<Vec<usize>>,
    /// For each job: the jobs whose way out lies nearest one of its ways in.
    prev: Vec<Vec<usize>>,
    /// The jobs whose way in lies nearest the start.
    first: Vec<usize>,
    is_first: Vec<bool>,
    /// The jobs whose way out lies nearest the end (none without an end).
    is_last: Vec<bool>,
    /// For each job: the jobs nearest it either way round, for ruin.
    near: Vec<Vec<usize>>,
    /// For each job: the jobs that have it in their `next` or `prev` lists (so a change near it
    /// can open a move for them).
    watchers: Vec<Vec<usize>>,
}

impl Cands {
    fn new(p: &Problem, k: usize) -> Self {
        let n = p.jobs.len();
        let k = k.min(n.saturating_sub(1));
        // A few ways per job are enough to judge nearness.
        let sample = |j: &Job, f: fn(&Way) -> Pt| -> Vec<Pt> {
            let step = j.ways.len().div_ceil(8);
            j.ways.iter().step_by(step.max(1)).map(f).collect()
        };
        let ins: Vec<Vec<Pt>> = p.jobs.iter().map(|j| sample(j, |w| w.entry)).collect();
        let outs: Vec<Vec<Pt>> = p.jobs.iter().map(|j| sample(j, |w| w.exit)).collect();
        let gap = |a: &[Pt], b: &[Pt]| {
            let mut m = f64::INFINITY;
            for x in a {
                for y in b {
                    m = m.min((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2));
                }
            }
            m
        };
        let nearest = |d: &dyn Fn(usize) -> f64, skip: usize| -> Vec<usize> {
            let mut all: Vec<(f64, usize)> =
                (0..n).filter(|&o| o != skip).map(|o| (d(o), o)).collect();
            all.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            all.into_iter().take(k).map(|e| e.1).collect()
        };
        let next: Vec<Vec<usize>> = (0..n)
            .map(|j| nearest(&|o| gap(&outs[j], &ins[o]), j))
            .collect();
        let prev: Vec<Vec<usize>> = (0..n)
            .map(|j| nearest(&|o| gap(&ins[j], &outs[o]), j))
            .collect();
        let near: Vec<Vec<usize>> = (0..n)
            .map(|j| nearest(&|o| gap(&outs[j], &ins[o]).min(gap(&ins[j], &outs[o])), j))
            .collect();
        let first = nearest(&|o| gap(&[p.start], &ins[o]), usize::MAX);
        let mut is_first = vec![false; n];
        for &j in &first {
            is_first[j] = true;
        }
        let mut is_last = vec![false; n];
        if let Some(e) = p.end {
            for j in nearest(&|o| gap(&outs[o], &[e]), usize::MAX) {
                is_last[j] = true;
            }
        }
        let mut watchers = vec![Vec::new(); n];
        for j in 0..n {
            for &o in next[j].iter().chain(&prev[j]) {
                if !watchers[o].contains(&j) {
                    watchers[o].push(j);
                }
            }
        }
        Self {
            next,
            prev,
            first,
            is_first,
            is_last,
            near,
            watchers,
        }
    }
}

/// Small, fast and seeded (SplitMix64).
#[derive(Debug)]
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// 0..n (n ≥ 1).
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }

    /// (0, 1].
    fn unit(&mut self) -> f64 {
        ((self.next() >> 11) + 1) as f64 / (1u64 << 53) as f64
    }
}

#[derive(Debug)]
struct Search<'a> {
    p: &'a Problem,
    c: &'a Cands,
    order: Vec<usize>,
    pos: Vec<usize>,
    way: Vec<usize>,
    rng: Rng,
    /// Don't-look bits: only jobs near a recent change are examined again. `queue` holds the
    /// awake jobs, `awake` marks them.
    awake: Vec<bool>,
    queue: Vec<usize>,
    look: Look,
    tune: Tuning,
}

impl<'a> Search<'a> {
    fn new(
        p: &'a Problem,
        c: &'a Cands,
        init: &Route,
        seed: u64,
        look: Look,
        tune: Tuning,
    ) -> Self {
        let n = init.order.len();
        let mut s = Self {
            p,
            c,
            order: init.order.clone(),
            pos: vec![0; n],
            way: init.ways.clone(),
            rng: Rng(seed),
            awake: vec![false; n],
            queue: Vec::with_capacity(n),
            look,
            tune,
        };
        s.reindex(0, n.saturating_sub(1));
        s
    }

    fn wake(&mut self, j: usize) {
        if !self.awake[j] {
            self.awake[j] = true;
            self.queue.push(j);
        }
    }

    /// Wake the jobs at positions from..=to and either side, and the jobs watching them.
    fn wake_around(&mut self, from: usize, to: usize) {
        let Some(last) = self.order.len().checked_sub(1) else {
            return;
        };
        let c = self.c;
        for i in from.saturating_sub(1)..=(to + 1).min(last) {
            let j = self.order[i];
            self.wake(j);
            if self.look == Look::Watchers {
                for &w in &c.watchers[j] {
                    self.wake(w);
                }
            }
        }
    }

    fn t(&self, a: Pt, b: Pt) -> f64 {
        self.p.travel.time(a, b)
    }

    fn w(&self, j: usize, w: usize) -> Way {
        self.p.jobs[j].ways[w]
    }

    fn cur(&self, j: usize) -> Way {
        self.w(j, self.way[j])
    }

    fn exit_before(&self, i: usize) -> Pt {
        if i == 0 {
            self.p.start
        } else {
            self.cur(self.order[i - 1]).exit
        }
    }

    /// Where the head goes down next after position i - 1: the job at i, or after the last job
    /// the end (if any). Every move's cost uses this, so all of them count the end.
    fn entry_at(&self, i: usize) -> Option<Pt> {
        match self.order.get(i) {
            Some(&j) => Some(self.cur(j).entry),
            None => self.p.end,
        }
    }

    fn reindex(&mut self, from: usize, to: usize) {
        for i in from..=to.min(self.order.len().saturating_sub(1)) {
            self.pos[self.order[i]] = i;
        }
    }

    fn route(&self) -> Route {
        Route {
            order: self.order.clone(),
            ways: self.way.clone(),
        }
    }

    fn restore(&mut self, r: &Route) {
        self.order.clone_from(&r.order);
        self.way.clone_from(&r.ways);
        self.reindex(0, self.order.len().saturating_sub(1));
        self.queue.clear();
        self.awake.fill(false);
    }

    fn cost(&self) -> f64 {
        self.p.cost_of(&self.order, &self.way)
    }

    /// The best way for job `j` between a way out and (maybe) a way in, and what it costs.
    fn best_way(&self, j: usize, before: Pt, after: Option<Pt>) -> (usize, f64) {
        let mut best = (self.way[j], f64::INFINITY);
        for (wi, w) in self.p.jobs[j].ways.iter().enumerate() {
            let c = self.t(before, w.entry) + after.map_or(0.0, |e| self.t(w.exit, e));
            if c < best.1 {
                best = (wi, c);
            }
        }
        best
    }

    /// No job in a..=b has to come before a job in b+1..=k.
    fn can_swap(&self, a: usize, b: usize, k: usize) -> bool {
        if b + 1 - a <= k - b {
            self.order[a..=b].iter().all(|&x| {
                self.p.succs[x]
                    .iter()
                    .all(|&s| !(b < self.pos[s] && self.pos[s] <= k))
            })
        } else {
            self.order[b + 1..=k].iter().all(|&y| {
                self.p.jobs[y]
                    .after
                    .iter()
                    .all(|&q| !(a <= self.pos[q] && self.pos[q] <= b))
            })
        }
    }

    /// Runs A = a..=b and B = b+1..=k swap places, if that shortens the route and keeps the
    /// precedence. A run of one cut may change its way.
    fn try_exchange(&mut self, a: usize, b: usize, k: usize) -> bool {
        if !(a <= b && b < k && k < self.order.len()) {
            return false;
        }
        let (a0, al, b0, bl) = (
            self.order[a],
            self.order[b],
            self.order[b + 1],
            self.order[k],
        );
        let pe = self.exit_before(a);
        let nx = self.entry_at(k + 1);
        let old = self.t(pe, self.cur(a0).entry)
            + self.t(self.cur(al).exit, self.cur(b0).entry)
            + nx.map_or(0.0, |e| self.t(self.cur(bl).exit, e));
        let wb = if b0 == bl {
            self.best_way(b0, pe, Some(self.cur(a0).entry)).0
        } else {
            self.way[b0]
        };
        let b_first = self.w(b0, wb);
        let b_exit = if b0 == bl {
            b_first.exit
        } else {
            self.cur(bl).exit
        };
        let wa = if a0 == al {
            self.best_way(a0, b_exit, nx).0
        } else {
            self.way[a0]
        };
        let a_first = self.w(a0, wa);
        let a_exit = if a0 == al {
            a_first.exit
        } else {
            self.cur(al).exit
        };
        let new = self.t(pe, b_first.entry)
            + self.t(b_exit, a_first.entry)
            + nx.map_or(0.0, |e| self.t(a_exit, e));
        if new >= old - EPS || !self.can_swap(a, b, k) {
            return false;
        }
        self.way[b0] = wb;
        self.way[a0] = wa;
        self.swap_runs(a, b, k);
        true
    }

    /// Runs a..=b and b+1..=k swap places; wakes the jobs at the joins.
    fn swap_runs(&mut self, a: usize, b: usize, k: usize) {
        self.order[a..=k].rotate_left(b + 1 - a);
        self.reindex(a, k);
        let lb = k - b;
        self.wake_around(a, a);
        self.wake_around(a + lb - 1, a + lb);
        self.wake_around(k, k);
    }

    fn flipped(&self, x: usize) -> usize {
        self.p.jobs[x]
            .flip
            .as_ref()
            .map_or(self.way[x], |f| f[self.way[x]])
    }

    /// Cut the run i..=j (straight lines only, no precedence among them) in reverse order, each
    /// line the other way round, if that shortens the route.
    fn try_reverse(&mut self, i: usize, j: usize) -> bool {
        if i >= j
            || j >= self.order.len()
            || self.order[i..=j]
                .iter()
                .any(|&x| self.p.jobs[x].flip.is_none())
        {
            return false;
        }
        let pe = self.exit_before(i);
        let nx = self.entry_at(j + 1);
        let mut old = self.t(pe, self.cur(self.order[i]).entry)
            + nx.map_or(0.0, |e| self.t(self.cur(self.order[j]).exit, e));
        for m in i..j {
            old += self.t(
                self.cur(self.order[m]).exit,
                self.cur(self.order[m + 1]).entry,
            );
        }
        let mut new = 0.0;
        let mut at = pe;
        for m in (i..=j).rev() {
            let x = self.order[m];
            let w = self.w(x, self.flipped(x));
            new += self.t(at, w.entry);
            at = w.exit;
        }
        if let Some(e) = nx {
            new += self.t(at, e);
        }
        if new >= old - EPS {
            return false;
        }
        if self.order[i..=j].iter().any(|&x| {
            self.p.jobs[x]
                .after
                .iter()
                .any(|&q| (i..=j).contains(&self.pos[q]))
        }) {
            return false;
        }
        for m in i..=j {
            let x = self.order[m];
            self.way[x] = self.flipped(x);
        }
        self.order[i..=j].reverse();
        self.reindex(i, j);
        self.wake_around(i, i);
        self.wake_around(j, j);
        true
    }

    /// Cut the job at position i another way, if that shortens the route.
    fn try_way(&mut self, i: usize) -> bool {
        let x = self.order[i];
        if self.p.jobs[x].ways.len() < 2 {
            return false;
        }
        let pe = self.exit_before(i);
        let nx = self.entry_at(i + 1);
        let now = self.cur(x);
        let old = self.t(pe, now.entry) + nx.map_or(0.0, |e| self.t(now.exit, e));
        let (w, c) = self.best_way(x, pe, nx);
        if c < old - EPS {
            self.way[x] = w;
            self.wake_around(i, i);
            true
        } else {
            false
        }
    }

    /// Try every move anchored at position i; apply the first that shortens the route.
    fn improve_at(&mut self, i: usize) -> bool {
        let c = self.c;
        let n = self.order.len();
        if self.try_way(i) {
            return true;
        }
        // A short run starting at i: just after a job whose way out is near its way in, or
        // just before a job whose way in is near its way out (or first of all).
        for len in 1..=self.tune.max_run.min(n - i) {
            let j = i + len - 1;
            let (s0, sl) = (self.order[i], self.order[j]);
            if i > 0 && c.is_first[s0] && self.try_exchange(0, i - 1, j) {
                return true;
            }
            if j + 1 < n && c.is_last[sl] && self.try_exchange(i, j, n - 1) {
                return true;
            }
            for &d in &c.prev[s0] {
                let g = self.pos[d];
                if (g + 1 < i && self.try_exchange(g + 1, i - 1, j))
                    || (g > j && self.try_exchange(i, j, g))
                {
                    return true;
                }
            }
            for &e in &c.next[sl] {
                let h = self.pos[e];
                if (h < i && self.try_exchange(h, i - 1, j))
                    || (h > j + 1 && self.try_exchange(i, j, h - 1))
                {
                    return true;
                }
            }
        }
        // Two adjacent runs of any length swap: A from i, B after it, so that the job before
        // A is followed by a near job (B's first), and B's last by a job near A's first.
        let firsts: &[usize] = if i == 0 {
            &c.first
        } else {
            &c.next[self.order[i - 1]]
        };
        for &b0 in firsts {
            let bp = self.pos[b0];
            if bp <= i {
                continue;
            }
            for &bl in &c.prev[self.order[i]] {
                let k = self.pos[bl];
                if k >= bp && self.try_exchange(i, bp - 1, k) {
                    return true;
                }
            }
        }
        // A run of straight lines from i, reversed so that the job before it is followed by a
        // near line.
        if self.p.jobs[self.order[i]].flip.is_some() {
            for &x in firsts {
                let j = self.pos[x];
                if j > i && self.try_reverse(i, j) {
                    return true;
                }
            }
        }
        false
    }

    /// First-improvement descent over the awake jobs, until none can improve.
    fn descend(&mut self) {
        if self.look == Look::All {
            self.queue.clear();
            self.awake.fill(false);
            loop {
                let mut improved = false;
                for i in 0..self.order.len() {
                    improved |= self.improve_at(i);
                }
                self.queue.clear();
                self.awake.fill(false);
                if !improved {
                    return;
                }
            }
        }
        while let Some(j) = self.queue.pop() {
            self.awake[j] = false;
            if self.improve_at(self.pos[j]) {
                self.wake(j);
            }
        }
    }

    /// The exact best ways for the order, if they help; wakes the jobs whose way changed.
    fn settle(&mut self) -> bool {
        let r = self.p.best_ways(&self.order);
        if self.p.cost(&r) < self.cost() - EPS {
            for j in 0..self.way.len() {
                if self.way[j] != r.ways[j] {
                    self.way[j] = r.ways[j];
                    self.wake_around(self.pos[j], self.pos[j]);
                }
            }
            true
        } else {
            false
        }
    }

    fn polish(&mut self) {
        loop {
            self.descend();
            if !self.settle() {
                break;
            }
        }
        self.queue.clear();
        self.awake.fill(false);
    }

    /// Ruin and recreate: take out a job and some of its neighbours, then put each back (in an
    /// order the precedence allows) at its cheapest legal place and way.
    fn kick(&mut self) {
        let n = self.order.len();
        if n >= 4 && self.rng.unit() <= self.tune.bridge && self.bridge() {
            return;
        }
        let c = self.c;
        let seed = self.rng.below(n);
        let size = 2 + self
            .rng
            .below(self.tune.max_ruin.saturating_sub(1).min(n / 2).max(1));
        let mut out = vec![false; n];
        if self.rng.unit() <= self.tune.string {
            // A stretch of the route around the seed.
            let from = self.pos[seed]
                .saturating_sub(self.rng.below(size))
                .min(n - size.min(n));
            for &x in &self.order[from..(from + size).min(n)] {
                out[x] = true;
            }
        } else {
            // A patch of the sheet: the seed and most of its nearest jobs.
            out[seed] = true;
            let mut taken = 1;
            for &x in &c.near[seed] {
                if taken >= size {
                    break;
                }
                if self.rng.below(4) != 0 {
                    out[x] = true;
                    taken += 1;
                }
            }
        }
        let mut left: Vec<usize> = (0..n).filter(|&x| out[x]).collect();
        // The jobs either side of each gap meet new neighbours: look at them again.
        let mut touched: Vec<usize> = Vec::new();
        for &x in &left {
            let i = self.pos[x];
            for m in [i.wrapping_sub(1), i + 1] {
                if let Some(&y) = self.order.get(m) {
                    if !out[y] {
                        touched.push(y);
                    }
                }
            }
        }
        let mut placed = Vec::with_capacity(left.len());
        self.order.retain(|&x| !out[x]);
        self.reindex(0, self.order.len().saturating_sub(1));
        while !left.is_empty() {
            let ready: Vec<usize> = (0..left.len())
                .filter(|&r| self.p.jobs[left[r]].after.iter().all(|&q| !out[q]))
                .collect();
            let x = left.swap_remove(ready[self.rng.below(ready.len())]);
            self.insert_cheapest(x, &out);
            out[x] = false;
            placed.push(x);
        }
        for j in touched.into_iter().chain(placed) {
            self.wake_around(self.pos[j], self.pos[j]);
        }
    }

    /// Double bridge: two random adjacent stretches swap places, if the precedence allows
    /// (a few tries). Wakes the jobs at the joins.
    fn bridge(&mut self) -> bool {
        let n = self.order.len();
        for _ in 0..8 {
            let a = self.rng.below(n - 1);
            let b = a + self.rng.below((n - 1 - a).min(12));
            let k = b + 1 + self.rng.below((n - 1 - b).min(12));
            if k >= n || !self.can_swap(a, b, k) {
                continue;
            }
            self.swap_runs(a, b, k);
            return true;
        }
        false
    }

    /// The last place job `x` may go: before every placed job that must follow it, directly or
    /// through jobs still out (x → y → z with y out still puts x before z).
    fn latest(&self, x: usize, out: &[bool]) -> usize {
        let mut hi = self.order.len();
        let mut seen = vec![false; out.len()];
        let mut stack = vec![x];
        while let Some(y) = stack.pop() {
            for &s in &self.p.succs[y] {
                if !seen[s] {
                    seen[s] = true;
                    if out[s] {
                        stack.push(s);
                    } else {
                        hi = hi.min(self.pos[s]);
                    }
                }
            }
        }
        hi
    }

    /// Put job `x` back where it costs least. Its predecessors are all placed (the kick puts
    /// jobs back in an order the precedence allows).
    fn insert_cheapest(&mut self, x: usize, out: &[bool]) {
        let p = self.p;
        let job = &p.jobs[x];
        let lo = job
            .after
            .iter()
            .map(|&q| self.pos[q] + 1)
            .max()
            .unwrap_or(0);
        let hi = self.latest(x, out);
        let mut best = (f64::INFINITY, lo, 0);
        for at in lo..=hi {
            if at < hi && self.rng.unit() < self.tune.blink {
                continue;
            }
            let pe = self.exit_before(at);
            let nx = self.entry_at(at);
            let base = nx.map_or(0.0, |e| self.t(pe, e));
            for (wi, w) in job.ways.iter().enumerate() {
                let d = self.t(pe, w.entry) + nx.map_or(0.0, |e| self.t(w.exit, e)) - base;
                if d < best.0 {
                    best = (d, at, wi);
                }
            }
        }
        self.order.insert(best.1, x);
        self.way[x] = best.2;
        self.reindex(best.1, self.order.len() - 1);
    }

    /// Iterated local search: polish, then kick and polish again, keeping the best route.
    fn iterate(&mut self, kicks: usize) -> Route {
        for j in 0..self.order.len() {
            self.wake(j);
        }
        self.polish();
        let mut cur = self.route();
        let mut cur_cost = self.cost();
        let mut best = cur.clone();
        let mut best_cost = cur_cost;
        let t0 = self.tune.heat * cur_cost / self.order.len() as f64;
        let cool = 0.01f64.powf(1.0 / kicks.max(1) as f64);
        let mut temp = t0;
        for _ in 0..kicks {
            self.kick();
            self.polish();
            let c = self.cost();
            temp *= cool;
            if c < cur_cost - temp * self.rng.unit().ln() + EPS {
                cur = self.route();
                cur_cost = c;
                if c < best_cost - EPS {
                    best = cur.clone();
                    best_cost = c;
                }
            } else {
                self.restore(&cur);
            }
        }
        best
    }
}

#[cfg(test)]
mod tests;
