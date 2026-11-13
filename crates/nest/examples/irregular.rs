//! Mounts nested into shaped bins, drawn as SVG: an oval drop-out, an arched one, and an
//! offcut with another piece still in it; each by the irregular nester alone, and with
//! guillotine layouts in its straight parts. Prints how long each took.
//!
//! cargo run --release -p nest --example irregular > layouts.svg

use std::fmt::Write;
use std::time::Instant;

use nest::irregular::{Bin, Instance, Point, Region, solve, solve_free, strokes};
use nest::{Item, Len, Objective, Options, Solution};

/// Which side of a curve its polygon lies: a bin's outline must lie inside the real edge, a
/// kept piece (a hole) outside it, so no mount crosses either.
#[derive(Clone, Copy, PartialEq)]
enum Side {
    Inside,
    Outside,
}

/// An oval `w` × `h` at `(x, y)` as `n` vertices: on the curve and rounded towards its centre
/// (the chords lie inside), or on the polygon round it and rounded away (outside).
fn oval(x: Len, y: Len, w: Len, h: Len, n: usize, side: Side) -> Vec<Point> {
    let grow = match side {
        Side::Inside => 1.0,
        Side::Outside => 1.0 / (std::f64::consts::PI / n as f64).cos(),
    };
    let (a, b) = (grow * w as f64 / 2.0, grow * h as f64 / 2.0);
    let round = |v: f64| match side {
        Side::Inside => v.trunc(),
        Side::Outside => v.signum() * v.abs().ceil(),
    } as Len;
    (0..n)
        .map(|k| {
            let t = std::f64::consts::TAU * k as f64 / n as f64;
            [
                x + w / 2 + round(a * t.cos()),
                y + h / 2 + round(b * t.sin()),
            ]
        })
        .collect()
}

/// A `w` wide window with straight sides `h` high and a round top.
fn arch(w: Len, h: Len, n: usize) -> Vec<Point> {
    let r = w as f64 / 2.0;
    let mut ring = vec![[0, 0], [w, 0]];
    for k in 0..=n {
        let t = std::f64::consts::PI * k as f64 / n as f64;
        ring.push([
            w / 2 + (r * t.cos()).trunc() as Len,
            h + (r * t.sin()).trunc() as Len,
        ]);
    }
    ring
}

fn mat(w: Len, h: Len, copies: u32) -> Item {
    Item {
        w,
        h,
        copies,
        profit: None,
        rotate: true,
        group: None,
    }
}

fn main() {
    // Common small mounts, tenths of a millimetre: 4 × 6, 5 × 7 and 8 × 10 in, A5, 10 × 10 cm.
    let small = || {
        vec![
            mat(1016, 1524, 6),
            mat(1270, 1778, 6),
            mat(2032, 2540, 3),
            mat(1480, 2100, 4),
            mat(1000, 1000, 6),
        ]
    };
    let scenes: Vec<(&str, Region, Vec<Item>)> = vec![
        (
            "400 × 200 oval, 100 mm squares",
            Region {
                outline: oval(0, 0, 4000, 2000, 64, Side::Inside),
                holes: vec![],
            },
            vec![mat(1000, 1000, 6)],
        ),
        (
            "400 × 300 oval, small mounts",
            Region {
                outline: oval(0, 0, 4000, 3000, 64, Side::Inside),
                holes: vec![],
            },
            small(),
        ),
        (
            "300 wide arch, small mounts",
            Region {
                outline: arch(3000, 2500, 32),
                holes: vec![],
            },
            small(),
        ),
        (
            "600 × 450 offcut round a kept 250 × 200 oval",
            Region {
                outline: vec![[0, 0], [6000, 0], [6000, 4500], [0, 4500]],
                holes: vec![oval(2200, 1600, 2500, 2000, 48, Side::Outside)],
            },
            small(),
        ),
    ];
    let options = Options::default();
    let methods: [(&str, Solver); 2] = [
        ("irregular alone", solve_free),
        ("guillotine where straight", solve),
    ];
    // A row per scene, a column per method, each in a 640 × 540 mm cell.
    let (cell_w, cell_h) = (640.0, 540.0);
    let mut body = String::new();
    let rows = scenes.len();
    for (row, (name, region, items)) in scenes.into_iter().enumerate() {
        for (col, (method, solver)) in methods.iter().enumerate() {
            let (x_at, y_at) = (20.0 + cell_w * col as f64, cell_h * row as f64);
            let instance = Instance {
                objective: Objective::Knapsack,
                bins: vec![Bin {
                    region: region.clone(),
                    copies: 1,
                }],
                items: items.clone(),
            };
            let start = Instant::now();
            let s = solver(&instance, &options).expect("a layout");
            let ms = start.elapsed().as_secs_f64() * 1e3;
            let (placed, n) = (s.placements.len(), strokes(&s));
            let fill = 100.0 * s.profit as f64 / region.area() as f64;
            eprintln!(
                "{name}, {method}: {placed} mounts, {fill:.0}% filled, {n} strokes, {ms:.0} ms"
            );
            draw(&mut body, (x_at, y_at), &region, &s);
            let h = region
                .outline
                .iter()
                .map(|p| p[1] as f64 / 10.0)
                .fold(0.0, f64::max);
            let _ = write!(
                body,
                r##"<text x="{x_at}" y="{}" font-family="sans-serif" font-size="11">{name}: {method}</text><text x="{x_at}" y="{}" font-family="sans-serif" font-size="10" fill="#555">{placed} mounts · {fill:.0}% filled · {n} strokes · {ms:.0} ms</text>"##,
                y_at + 22.0,
                y_at + h + 56.0,
            );
        }
    }
    println!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{height}" viewBox="0 0 {w} {height}"><rect width="100%" height="100%" fill="white"/>{body}</svg>"#,
        w = 2.0 * cell_w,
        height = cell_h * rows as f64,
    );
}

type Solver = fn(&Instance, &Options) -> Result<Solution, nest::Error>;

/// The region and its mounts, in millimetres with y up, from `(x_at, y_at)`.
fn draw(body: &mut String, (x_at, y_at): (f64, f64), region: &Region, s: &Solution) {
    let h = region
        .outline
        .iter()
        .map(|p| p[1] as f64 / 10.0)
        .fold(0.0, f64::max);
    let ring = |r: &[Point]| {
        let mut d = String::new();
        for (i, p) in r.iter().enumerate() {
            let c = if i == 0 { 'M' } else { 'L' };
            let _ = write!(d, "{c}{} {} ", p[0] as f64 / 10.0, p[1] as f64 / 10.0);
        }
        d + "Z"
    };
    let mut d = ring(&region.outline);
    for hole in &region.holes {
        d += " ";
        d += &ring(hole);
    }
    let _ = write!(
        body,
        r##"<g transform="translate({x_at} {top}) scale(1 -1)"><path d="{d}" fill="#f3efe6" stroke="#333" stroke-width="0.6" fill-rule="evenodd"/>"##,
        top = y_at + h + 40.0,
    );
    for p in &s.placements {
        let hue = (p.item * 67) % 360;
        let _ = write!(
            body,
            r##"<rect x="{}" y="{}" width="{}" height="{}" fill="hsl({hue} 55% 70%)" stroke="#222" stroke-width="0.4"/>"##,
            p.x as f64 / 10.0,
            p.y as f64 / 10.0,
            p.w as f64 / 10.0,
            p.h as f64 / 10.0,
        );
    }
    body.push_str("</g>");
}
