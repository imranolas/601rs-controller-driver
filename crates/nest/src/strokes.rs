//! Strokes: the straight-head cuts a layout's outer edges need. Mats lie edge to edge, so edges
//! on one line that overlap or touch are one stroke, as the compiler cuts them. On the 601 a
//! stroke costs about a second beyond its length (the head down and settling, the dead time,
//! the head up), and more with the travel to it; the length hardly counts (855 mm/s).
//!
//! The search breaks ties between equal layouts by their strokes; [`rearrange`] then lowers
//! them further without changing what goes in which strip, column or bin. On 215 benchmark
//! queues (5,751 mats on 1,304 sheets) the two together cut 3.3 % of the strokes, from 7.9 to
//! 7.6 a sheet, with the same sheets and leftovers.

use std::collections::{BTreeSet, HashMap};

use crate::{Area, Instance, Len, Solution, check};

/// A rectangle: bin, x, y, w, h.
pub(crate) type Rect = (usize, Len, Len, Len, Len);

/// The strokes the rectangles' edges need, and their total length.
pub(crate) fn strokes(rects: impl IntoIterator<Item = Rect>) -> (usize, Area) {
    let mut lines: HashMap<(usize, bool, Len), Vec<(Len, Len)>> = HashMap::new();
    for (b, x, y, w, h) in rects {
        for (across, at, a, z) in [
            (true, y, x, x + w),
            (true, y + h, x, x + w),
            (false, x, y, y + h),
            (false, x + w, y, y + h),
        ] {
            lines.entry((b, across, at)).or_default().push((a, z));
        }
    }
    let (mut n, mut len) = (0, 0);
    for segs in lines.values_mut() {
        segs.sort_unstable();
        let (mut a, mut z) = segs[0];
        for &(c, d) in &segs[1..] {
            if c > z {
                n += 1;
                len += Area::from(z - a);
                a = c;
            }
            z = z.max(d);
        }
        n += 1;
        len += Area::from(z - a);
    }
    (n, len)
}

/// A mat in a column: its placement, size, and whether it lies against the column's right side.
#[derive(Debug, Clone)]
struct Cell {
    k: usize,
    w: Len,
    h: Len,
    right: bool,
}

/// A column of a band: mats stacked from its bottom, or from its top.
#[derive(Debug, Clone)]
struct Column {
    w: Len,
    cells: Vec<Cell>,
    from_top: bool,
}

/// A full-width band of a bin (the first stage).
#[derive(Debug, Clone)]
struct Band {
    h: Len,
    columns: Vec<Column>,
}

/// One bin's layout as bands, columns and stacks, from its bottom-left inside the trims.
struct Sheet {
    bin: usize,
    left: Len,
    bottom: Len,
}

impl Sheet {
    fn place(&self, bands: &[Band], out: &mut Vec<(usize, Rect)>) {
        out.clear();
        let mut y = self.bottom;
        for band in bands {
            let mut x = self.left;
            for c in &band.columns {
                let stack: Len = c.cells.iter().map(|m| m.h).sum();
                let mut my = if c.from_top { y + band.h - stack } else { y };
                for m in &c.cells {
                    let mx = if m.right { x + c.w - m.w } else { x };
                    out.push((m.k, (self.bin, mx, my, m.w, m.h)));
                    my += m.h;
                }
                x += c.w;
            }
            y += band.h;
        }
    }

    fn cost(&self, bands: &[Band], buf: &mut Vec<(usize, Rect)>) -> (usize, Area) {
        self.place(bands, buf);
        strokes(buf.iter().map(|&(_, r)| r))
    }

    /// Take the first change that lowers the strokes (then the length), until none does:
    /// swap two bands, two columns of a band or two mats of a stack, stack a column from the
    /// other end, or move a mat to the other side of its column.
    fn climb(&self, bands: &mut Vec<Band>) -> (usize, Area) {
        let mut buf = Vec::new();
        let mut best = self.cost(bands, &mut buf);
        let mut improved = true;
        while improved {
            improved = false;
            let mut attempt = |bands: &mut Vec<Band>, change: &dyn Fn(&mut Vec<Band>)| {
                let before = bands.clone();
                change(bands);
                let cost = self.cost(bands, &mut buf);
                if cost < best {
                    best = cost;
                    improved = true;
                } else {
                    *bands = before;
                }
            };
            for i in 0..bands.len() {
                for j in i + 1..bands.len() {
                    attempt(bands, &|b| b.swap(i, j));
                }
                for c in 0..bands[i].columns.len() {
                    for d in c + 1..bands[i].columns.len() {
                        attempt(bands, &|b| b[i].columns.swap(c, d));
                    }
                    attempt(bands, &|b| b[i].columns[c].from_top ^= true);
                    for m in 0..bands[i].columns[c].cells.len() {
                        for n in m + 1..bands[i].columns[c].cells.len() {
                            attempt(bands, &|b| b[i].columns[c].cells.swap(m, n));
                        }
                        let column = &bands[i].columns[c];
                        if column.cells[m].w < column.w {
                            attempt(bands, &|b| b[i].columns[c].cells[m].right ^= true);
                        }
                    }
                }
            }
        }
        best
    }
}

/// Lines across `spans` that cut none of them, from `start` on.
fn clear_lines(start: Len, spans: impl Iterator<Item = (Len, Len)> + Clone) -> Vec<Len> {
    let mut lines: Vec<Len> = spans.clone().flat_map(|(a, z)| [a, z]).collect();
    lines.push(start);
    lines.sort_unstable();
    lines.dedup();
    lines.retain(|&v| v >= start && !spans.clone().any(|(a, z)| a < v && v < z));
    lines
}

/// Restarts of the climb from shuffled orders, after the one from the layout as found.
const RESTARTS: usize = 20;

/// Lower the strokes of each bin's layout without changing which mats share a band or a
/// column: reorder its bands, each band's columns and each column's stack, stack a column from
/// the top, and move a narrower mat to its column's right. Bands stay full width and together
/// from the bottom, so the leftover is the same. Deterministic. A layout that doesn't
/// check afterwards (a bug) is left as it was.
pub(crate) fn rearrange(instance: &Instance, solution: &mut Solution) {
    let before = solution.clone();
    let seq = instance.bin_sequence();
    let used: BTreeSet<usize> = solution.placements.iter().map(|p| p.bin).collect();
    let mut buf = Vec::new();
    for bin in used {
        let sheet = Sheet {
            bin,
            left: seq[bin].trims.left,
            bottom: seq[bin].trims.bottom,
        };
        let p = &solution.placements;
        let mats: Vec<usize> = (0..p.len()).filter(|&k| p[k].bin == bin).collect();
        let ys = clear_lines(
            sheet.bottom,
            mats.iter().map(|&k| (p[k].y, p[k].y + p[k].h)),
        );
        let bands: Vec<Band> = ys
            .windows(2)
            .map(|v| {
                let in_band: Vec<usize> = mats
                    .iter()
                    .copied()
                    .filter(|&k| p[k].y >= v[0] && p[k].y + p[k].h <= v[1])
                    .collect();
                let xs = clear_lines(
                    sheet.left,
                    in_band.iter().map(|&k| (p[k].x, p[k].x + p[k].w)),
                );
                let columns = xs
                    .windows(2)
                    .map(|u| {
                        let mut ks: Vec<usize> = in_band
                            .iter()
                            .copied()
                            .filter(|&k| p[k].x >= u[0] && p[k].x + p[k].w <= u[1])
                            .collect();
                        ks.sort_by_key(|&k| p[k].y);
                        Column {
                            w: u[1] - u[0],
                            cells: ks
                                .iter()
                                .map(|&k| Cell {
                                    k,
                                    w: p[k].w,
                                    h: p[k].h,
                                    right: p[k].x > u[0],
                                })
                                .collect(),
                            from_top: ks.first().is_some_and(|&k| p[k].y > v[0]),
                        }
                    })
                    .collect();
                Band {
                    h: v[1] - v[0],
                    columns,
                }
            })
            .collect();
        let mut best_bands = bands.clone();
        let mut best = sheet.climb(&mut best_bands);
        let mut seed = 7 + bin as u64;
        for _ in 0..RESTARTS {
            let mut b = bands.clone();
            shuffle(&mut b, &mut seed);
            for band in &mut b {
                shuffle(&mut band.columns, &mut seed);
                for c in &mut band.columns {
                    shuffle(&mut c.cells, &mut seed);
                    c.from_top = next(&mut seed) % 2 == 1;
                }
            }
            let cost = sheet.climb(&mut b);
            if cost < best {
                (best, best_bands) = (cost, b);
            }
        }
        sheet.place(&best_bands, &mut buf);
        for &(k, (_, x, y, _, _)) in &buf {
            solution.placements[k].x = x;
            solution.placements[k].y = y;
        }
    }
    if check(instance, solution).is_err() {
        *solution = before;
    }
}

/// Small, seeded.
fn next(seed: &mut u64) -> usize {
    *seed = seed
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    (*seed >> 33) as usize
}

fn shuffle<T>(v: &mut [T], seed: &mut u64) {
    for i in (1..v.len()).rev() {
        v.swap(i, next(seed) % (i + 1));
    }
}
