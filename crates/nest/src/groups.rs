//! Groups ([`Item::group`]): a job's items kept on one bin where that costs nothing else, and the
//! earlier groups on the earlier bins.
//!
//! The copies of interchangeable item types ([`kinds`]: the same size, turn and profit) are one
//! item type to the search, so which copy goes in which of their places is free. [`relabel`]
//! chooses, once the layout is found: the fewest groups split over bins, then the earlier
//! groups first ([`Score`]). Each kind's copies start in group order in its places in bin order,
//! then pairs of copies trade places while that does better. No rectangle moves, so the bins,
//! the leftover and the strokes stay as they are.
//!
//! Tried and left out, on 100 random queues with 1,265 split groups after this:
//! - ranking finished layouts by their groups inside the search: after strokes, 2 fewer splits,
//!   and on one queue its pick, equal before `strokes::rearrange`, came out a stroke worse after
//!   it; before strokes, 7 fewer for 70 more strokes;
//! - swapping mats between sheets afterwards and laying the two out again, kept only if the
//!   strokes and the leftover hold: 13 fewer, at about a quarter of the nester's time. Mounts
//!   with fillets did no better.
//!
//! At the same sheet count most splits are forced: the jobs averaged half a sheet, on sheets
//! about 80 % full. Keeping every job that fits a sheet whole would cost about 3 % more sheets
//! (by area), which is no tie-break.

use std::collections::BTreeMap;

use crate::{Area, Item, Objective, Placement};

/// How well a layout keeps its groups together and in order: the less, the better.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct Score {
    /// Groups with copies on more than one bin.
    splits: usize,
    /// Bin packing: each bin's lowest group, in bin order (`i64::MAX` for a bin with none), so
    /// the earlier groups are on the earlier bins. Knapsack: each group's profit placed,
    /// negated, in group order, so the earlier groups are placed first.
    order: Vec<i64>,
}

/// One kind's copies, and the places they fill.
struct Class {
    /// Each copy's group (renumbered by [`dense`]), the item types' copies in order.
    groups: Vec<Option<usize>>,
    /// As many places as copies: the bins of the kind's placements, in increasing order, then
    /// `None` for each copy left out of a knapsack.
    places: Vec<Option<usize>>,
    /// One copy's profit.
    profit: Area,
}

/// The item types whose copies can take each other's places: the same size (either way round
/// if they may turn), turn and profit. Each kind lists its item types in order; types with no
/// copies are left out.
pub(crate) fn kinds(items: &[Item], profit: impl Fn(usize) -> Area) -> Vec<Vec<usize>> {
    let mut kinds: Vec<Vec<usize>> = Vec::new();
    for (k, it) in items.iter().enumerate() {
        if it.copies == 0 {
            continue;
        }
        let same = |&j: &usize| {
            let jt = &items[j];
            jt.rotate == it.rotate
                && profit(j) == profit(k)
                && ((jt.w, jt.h) == (it.w, it.h) || (it.rotate && (jt.w, jt.h) == (it.h, it.w)))
        };
        match kinds.iter_mut().find(|kind| same(&kind[0])) {
            Some(kind) => kind.push(k),
            None => kinds.push(vec![k]),
        }
    }
    kinds
}

/// Each item type's group, renumbered 0, 1, … in order; none if no item type has a group.
fn dense(items: &[Item]) -> Option<Vec<Option<usize>>> {
    let mut groups: Vec<u32> = items.iter().filter_map(|it| it.group).collect();
    if groups.is_empty() {
        return None;
    }
    groups.sort_unstable();
    groups.dedup();
    Some(
        items
            .iter()
            .map(|it| it.group.map(|g| groups.partition_point(|&x| x < g)))
            .collect(),
    )
}

/// Settle which copy of interchangeable item types each placement is, for the groups (the
/// module's doc), the rectangles left where they are. Nothing changes if no item type has a
/// group, or if the placements don't match the items (the check reports that).
pub(crate) fn relabel(
    objective: Objective,
    items: &[Item],
    profit: impl Fn(usize) -> Area,
    placements: &mut [Placement],
) {
    let Some(group) = dense(items) else {
        return;
    };
    let kinds = kinds(items, &profit);
    let mut kind_of = vec![None; items.len()];
    for (c, kind) in kinds.iter().enumerate() {
        for &k in kind {
            kind_of[k] = Some(c);
        }
    }
    // Each kind's placements, by bin.
    let mut on: Vec<Vec<usize>> = vec![Vec::new(); kinds.len()];
    for (i, p) in placements.iter().enumerate() {
        let Some(&Some(c)) = kind_of.get(p.item) else {
            return;
        };
        on[c].push(i);
    }
    let mut classes = Vec::with_capacity(kinds.len());
    let mut owners: Vec<Vec<usize>> = Vec::with_capacity(kinds.len());
    for (kind, here) in kinds.iter().zip(&mut on) {
        here.sort_by_key(|&i| placements[i].bin);
        let owner: Vec<usize> = kind
            .iter()
            .flat_map(|&k| std::iter::repeat_n(k, items[k].copies as usize))
            .collect();
        if here.len() > owner.len() {
            return;
        }
        let mut places: Vec<Option<usize>> =
            here.iter().map(|&i| Some(placements[i].bin)).collect();
        places.resize(owner.len(), None);
        classes.push(Class {
            groups: owner.iter().map(|&k| group[k]).collect(),
            places,
            profit: profit(kind[0]),
        });
        owners.push(owner);
    }
    let mut state = State::new(objective, &classes);
    state.improve();
    for (c, here) in on.iter().enumerate() {
        for (place, &i) in here.iter().enumerate() {
            let k = owners[c][state.at[c][place]];
            let (p, it) = (&mut placements[i], &items[k]);
            p.item = k;
            p.rotated = (p.w, p.h) != (it.w, it.h);
        }
    }
}

/// An assignment of copies to places, with the counts its score needs, kept up to date as
/// copies trade places.
struct State<'a> {
    objective: Objective,
    classes: &'a [Class],
    /// The copy in each place, per class.
    at: Vec<Vec<usize>>,
    /// Per group: the bins it's on, and how many of its copies are on each.
    on: Vec<Vec<(usize, u32)>>,
    /// Per bin: how many copies of each group it has.
    per_bin: Vec<BTreeMap<usize, u32>>,
    /// Per group: the profit placed.
    placed: Vec<Area>,
    splits: usize,
}

impl<'a> State<'a> {
    /// Each class's copies in group order (ungrouped last) in its places in order.
    fn new(objective: Objective, classes: &'a [Class]) -> State<'a> {
        let groups = classes
            .iter()
            .flat_map(|c| c.groups.iter().flatten())
            .max()
            .map_or(0, |g| g + 1);
        let bins = classes
            .iter()
            .flat_map(|c| c.places.iter().flatten())
            .max()
            .map_or(0, |b| b + 1);
        let mut state = State {
            objective,
            classes,
            at: Vec::with_capacity(classes.len()),
            on: vec![Vec::new(); groups],
            per_bin: vec![BTreeMap::new(); bins],
            placed: vec![0; groups],
            splits: 0,
        };
        for class in classes {
            let mut copies: Vec<usize> = (0..class.groups.len()).collect();
            copies.sort_by_key(|&k| (class.groups[k].is_none(), class.groups[k]));
            for (&copy, &bin) in copies.iter().zip(&class.places) {
                state.add(class.groups[copy], bin, class.profit);
            }
            state.at.push(copies);
        }
        state
    }

    fn add(&mut self, group: Option<usize>, bin: Option<usize>, profit: Area) {
        let (Some(g), Some(b)) = (group, bin) else {
            return;
        };
        match self.on[g].iter_mut().find(|(x, _)| *x == b) {
            Some((_, n)) => *n += 1,
            None => {
                self.on[g].push((b, 1));
                if self.on[g].len() == 2 {
                    self.splits += 1;
                }
            }
        }
        *self.per_bin[b].entry(g).or_default() += 1;
        self.placed[g] += profit;
    }

    fn remove(&mut self, group: Option<usize>, bin: Option<usize>, profit: Area) {
        let (Some(g), Some(b)) = (group, bin) else {
            return;
        };
        let on = &mut self.on[g];
        if let Some(k) = on.iter().position(|(x, _)| *x == b) {
            on[k].1 -= 1;
            if on[k].1 == 0 {
                on.swap_remove(k);
                if on.len() == 1 {
                    self.splits -= 1;
                }
            }
        }
        if let Some(n) = self.per_bin[b].get_mut(&g) {
            *n -= 1;
            if *n == 0 {
                self.per_bin[b].remove(&g);
            }
        }
        self.placed[g] -= profit;
    }

    /// The copies in places `i` and `j` of class `c` trade places.
    fn swap(&mut self, c: usize, i: usize, j: usize) {
        let classes = self.classes;
        let class = &classes[c];
        let (a, b) = (self.at[c][i], self.at[c][j]);
        let (ga, gb) = (class.groups[a], class.groups[b]);
        let (pi, pj) = (class.places[i], class.places[j]);
        self.remove(ga, pi, class.profit);
        self.remove(gb, pj, class.profit);
        self.add(ga, pj, class.profit);
        self.add(gb, pi, class.profit);
        self.at[c].swap(i, j);
    }

    fn score(&self) -> Score {
        let order = match self.objective {
            Objective::BinPacking => self
                .per_bin
                .iter()
                .map(|groups| groups.keys().next().map_or(i64::MAX, |&g| g as i64))
                .collect(),
            Objective::Knapsack => self.placed.iter().map(|&p| -p).collect(),
        };
        Score {
            splits: self.splits,
            order,
        }
    }

    /// Pairs of copies trade places while that does better. Each trade lowers the score, so
    /// this ends.
    fn improve(&mut self) {
        let mut best = self.score();
        // Only classes with copies of more than one group in more than one place have a choice.
        let classes = self.classes;
        let mixed = |v: &[Option<usize>]| v.iter().any(|x| *x != v[0]);
        let open: Vec<usize> = (0..classes.len())
            .filter(|&c| mixed(&classes[c].groups) && mixed(&classes[c].places))
            .collect();
        loop {
            let mut better = false;
            for &c in &open {
                let class = &classes[c];
                let n = class.places.len();
                for i in 0..n {
                    for j in i + 1..n {
                        let (gi, gj) = (class.groups[self.at[c][i]], class.groups[self.at[c][j]]);
                        if class.places[i] == class.places[j] || gi == gj {
                            continue;
                        }
                        self.swap(c, i, j);
                        if self.splits <= best.splits {
                            let score = self.score();
                            if score < best {
                                best = score;
                                better = true;
                                continue;
                            }
                        }
                        self.swap(c, i, j);
                    }
                }
            }
            if !better {
                return;
            }
        }
    }
}
