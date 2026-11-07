//! Iterative beam search, after treesearchsolver's `iterative_beam_search_2` (Florian Fontan,
//! MIT): search the tree level by level, keeping the `width` best nodes of each level by a
//! guide; drop nodes that can't beat the best solution, and nodes dominated by another with
//! the same key; then search again with a wider beam. A pass that never had to drop a node for
//! want of width searched everything, and ends the search.

use std::cmp::Ordering;
use std::collections::HashMap;
use std::hash::Hash;

/// A tree of partial solutions to search.
pub(crate) trait Scheme {
    type Node: Clone;
    /// Nodes are only compared for dominance when their keys are equal.
    type Key: Hash + Eq;

    fn root(&self) -> Self::Node;
    fn children(&self, node: &Self::Node, out: &mut Vec<Self::Node>);
    /// A complete solution: it has no children worth searching.
    fn is_leaf(&self, node: &Self::Node) -> bool;
    /// `node` is a better solution than `best` (or there is no best yet).
    fn better(&self, node: &Self::Node, best: Option<&Self::Node>) -> bool;
    /// Nothing below `node` can be better than `best`.
    fn bound(&self, node: &Self::Node, best: &Self::Node) -> bool;
    /// Which nodes the beam keeps: the least first.
    fn order(&self, a: &Self::Node, b: &Self::Node) -> Ordering;
    fn key(&self, node: &Self::Node) -> Self::Key;
    /// `a` is at least as good a start as `b` for everything below it.
    fn dominates(&self, a: &Self::Node, b: &Self::Node) -> bool;
    /// `best` can't be improved on (it meets a lower bound): the search can stop.
    fn is_optimal(&self, best: &Self::Node) -> bool;
}

/// The beam widths to try: from 1, each `growth` times the last, up to `max_width`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Widths {
    pub max_width: usize,
    pub growth: f64,
}

/// Search, improving on `best`. Returns whether the search finished: it searched everything,
/// or `best` is optimal.
pub(crate) fn search<S: Scheme>(scheme: &S, widths: Widths, best: &mut Option<S::Node>) -> bool {
    let mut width = 1usize;
    let mut children = Vec::new();
    loop {
        let mut complete = true;
        let mut level = vec![scheme.root()];
        while !level.is_empty() {
            // The next level: candidates, with dropped ones as None; for each key, its live ones.
            let mut next: Vec<Option<S::Node>> = Vec::new();
            let mut by_key: HashMap<S::Key, Vec<usize>> = HashMap::new();
            for node in &level {
                if best.as_ref().is_some_and(|b| scheme.bound(node, b)) {
                    continue;
                }
                scheme.children(node, &mut children);
                for child in children.drain(..) {
                    if scheme.better(&child, best.as_ref()) {
                        *best = Some(child.clone());
                        if best.as_ref().is_some_and(|b| scheme.is_optimal(b)) {
                            return true;
                        }
                    }
                    if scheme.is_leaf(&child)
                        || best.as_ref().is_some_and(|b| scheme.bound(&child, b))
                    {
                        continue;
                    }
                    let same = by_key.entry(scheme.key(&child)).or_default();
                    if same.iter().any(|&i| {
                        next[i]
                            .as_ref()
                            .is_some_and(|n| scheme.dominates(n, &child))
                    }) {
                        continue;
                    }
                    same.retain(|&i| {
                        let dominated = next[i]
                            .as_ref()
                            .is_some_and(|n| scheme.dominates(&child, n));
                        if dominated {
                            next[i] = None;
                        }
                        !dominated
                    });
                    same.push(next.len());
                    next.push(Some(child));
                }
            }
            let mut kept: Vec<S::Node> = next.into_iter().flatten().collect();
            if kept.len() > width {
                complete = false;
                kept.select_nth_unstable_by(width, |a, b| scheme.order(a, b));
                kept.truncate(width);
            }
            kept.sort_by(|a, b| scheme.order(a, b));
            level = kept;
        }
        if complete {
            return true;
        }
        let wider = (width + 1).max((width as f64 * widths.growth) as usize);
        if wider > widths.max_width {
            return false;
        }
        width = wider;
    }
}
