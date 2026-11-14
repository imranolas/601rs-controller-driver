//! Sheets laid out by the nester, compiled: every one compiles and lowers, cuts each contour's
//! every edge once, shares each edge between neighbours (as many straight cuts as the nester
//! counts strokes), and keeps the order's rules. And one runs on the simulator.

use std::sync::mpsc;

use compiler::{Blade, Compiled, Contour, Keep, Options, Shape, Sheet, Tool, compile};
use machine::Profile;
use nest::{Bin, Instance, Item, Objective, Placement, Solution, Trims};
use program::{BladeOffsets, Board, Head, Material, Op};

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

const SHEET_MM: (f64, f64) = (815.0, 1015.0);

fn material() -> Material {
    Material {
        name: "MSK_1.5".to_owned(),
        speed_pct: 95.0,
        head_settle_ms: 250,
        blade_angle_offset_deg: BladeOffsets {
            straight: 0.0,
            bevel: 3.0,
            bevel_round: None,
        },
    }
}

/// A queue of mats in tenths of a millimetre, laid out on 32 × 40 in sheets 21 mm in from the clamped edges.
fn layout(rng: &mut Rng) -> (Instance, Solution) {
    let sizes = [
        (2030, 2540),
        (2790, 3560),
        (2100, 2970),
        (3000, 4000),
        (2500, 2000),
    ];
    let items: Vec<Item> = (0..2 + rng.next() % 4)
        .map(|_| {
            let (w, h) = sizes[rng.next() as usize % sizes.len()];
            Item {
                w,
                h,
                copies: 1 + rng.next() as u32 % 3,
                rotate: true,
                ..Item::default()
            }
        })
        .collect();
    let instance = Instance {
        objective: Objective::BinPacking,
        bins: vec![Bin {
            w: 8150,
            h: 10150,
            // GMC's 21 mm in from the clamped edges, so no cut runs over a pad; 5 mm elsewhere.
            trims: Trims {
                left: 210,
                right: 50,
                top: 50,
                bottom: 210,
            },
            copies: 20,
        }],
        items,
    };
    let s = nest::solve(&instance, &nest::Options::default()).expect("lays out");
    (instance, s)
}

/// Bin `bin`'s mats as mounts, each with one to three windows (one of them now and then an
/// oval) inside a 40 mm border.
fn sheet(rng: &mut Rng, s: &Solution, bin: usize) -> Sheet {
    let mut contours = Vec::new();
    for p in s.placements.iter().filter(|p| p.bin == bin) {
        let [x0, y0, w, h] = [p.x, p.y, p.w, p.h].map(|v| f64::from(v) / 10.0);
        let (x1, y1) = (x0 + w, y0 + h);
        let windows = 1 + rng.next() as usize % 3;
        let border = 40.0;
        let gap = 20.0;
        let each = (w - 2.0 * border - gap * (windows - 1) as f64) / windows as f64;
        for k in 0..windows {
            let wx = x0 + border + k as f64 * (each + gap);
            let oval = rng.next() % 5 == 0;
            contours.push(Contour {
                id: None,
                label: None,
                shape: if oval { Shape::Oval } else { Shape::Rect },
                box_mm: [wx, y0 + border, wx + each, y1 - border],
                tool: Tool::Bevel,
                keep: Keep::Out,
            });
        }
        contours.push(Contour {
            id: None,
            label: None,
            shape: Shape::Rect,
            box_mm: [x0, y0, x1, y1],
            tool: Tool::Straight,
            keep: Keep::In,
        });
    }
    Sheet {
        id: format!("sheet {bin}"),
        meta: None,
        board: Board {
            width_mm: SHEET_MM.0,
            height_mm: SHEET_MM.1,
            thickness_mm: 1.5,
        },
        material: material(),
        contours,
    }
}

/// Each cut's parts ("c3.left"), in program order.
fn parts(c: &Compiled) -> Vec<(Head, Vec<String>)> {
    c.program
        .ops
        .iter()
        .filter_map(|op| match op {
            Op::Cut(cut) => Some((
                cut.head,
                cut.id
                    .as_deref()
                    .unwrap_or_default()
                    .split('+')
                    .map(str::to_owned)
                    .collect(),
            )),
            _ => None,
        })
        .collect()
}

#[test]
fn nested_sheets_compile_with_each_edge_cut_once_in_order() {
    let mut rng = Rng(601);
    let profile = Profile::gunnar_601();
    let mut sheets = 0;
    for _ in 0..6 {
        let (_, s) = layout(&mut rng);
        for bin in 0..s.bins_used {
            let sh = sheet(&mut rng, &s, bin);
            let c =
                compile(&sh, &Blade::fitted(), &profile, &Options::default()).expect("compiles");
            sheets += 1;
            let cuts = parts(&c);
            // Every edge of every contour once.
            let mut all: Vec<&String> = cuts.iter().flat_map(|(_, p)| p).collect();
            let n = all.len();
            all.sort();
            all.dedup();
            assert_eq!(all.len(), n, "an edge cut twice");
            let edges: usize = sh
                .contours
                .iter()
                .map(|c| if c.shape == Shape::Oval { 1 } else { 4 })
                .sum();
            assert_eq!(n, edges, "every edge cut");
            // As many straight cuts as the nester counts strokes for the bin.
            let mine: Vec<Placement> = s
                .placements
                .iter()
                .filter(|p| p.bin == bin)
                .copied()
                .collect();
            let strokes = nest::irregular::strokes(&Solution {
                placements: mine,
                bins_used: 1,
                profit: 0,
            });
            let straight = cuts.iter().filter(|(h, _)| *h == Head::Straight).count();
            assert_eq!(straight, strokes, "shared edges cut once");
            // The rules: windows before their mount's outline, each mount's left edge last.
            let place = |part: &str| cuts.iter().position(|(_, p)| p.iter().any(|q| q == part));
            for (k, contour) in sh.contours.iter().enumerate() {
                if contour.keep != Keep::In {
                    continue;
                }
                let id = |s: &str| format!("c{}.{s}", k + 1);
                let left = place(&id("left")).expect("cut");
                for s in ["top", "right", "bottom"] {
                    assert!(place(&id(s)).expect("cut") < left, "mount {k}: left last");
                }
                for (w, inner) in sh.contours.iter().enumerate() {
                    let [a, b] = [inner.box_mm, contour.box_mm];
                    if inner.keep == Keep::Out
                        && a[0] > b[0]
                        && a[2] < b[2]
                        && a[1] > b[1]
                        && a[3] < b[3]
                    {
                        let last_window = cuts
                            .iter()
                            .rposition(|(_, p)| {
                                p.iter().any(|q| q.starts_with(&format!("c{}.", w + 1)))
                            })
                            .expect("cut");
                        let first_outline = cuts
                            .iter()
                            .position(|(_, p)| {
                                p.iter().any(|q| q.starts_with(&format!("c{}.", k + 1)))
                            })
                            .expect("cut");
                        assert!(last_window < first_outline, "window {w} before mount {k}");
                    }
                }
            }
            assert_eq!(
                compile(&sh, &Blade::fitted(), &profile, &Options::default())
                    .expect("compiles")
                    .program,
                c.program,
                "same sheet, same program"
            );
        }
    }
    assert!(sheets >= 6, "{sheets} sheets");
}

/// The simulator runs this much faster than life.
const SPEEDUP: f64 = 20.0;

#[test]
fn a_compiled_sheet_runs_on_the_simulator() {
    let mut rng = Rng(7);
    let (_, s) = layout(&mut rng);
    let sh = sheet(&mut rng, &s, 0);
    let profile = Profile::gunnar_601();
    let c = compile(&sh, &Blade::fitted(), &profile, &Options::default()).expect("compiles");

    let (link, _sim) = sim::spawn(
        sim::SimConfig {
            homed_at_start: false,
            ..sim::SimConfig::default()
        },
        SPEEDUP,
    );
    let (ctl, _version) =
        controller::connect(link, controller::Config::default()).expect("connects");
    let mut runner = runner::Runner::new(
        ctl,
        profile,
        runner::Config {
            speedup: SPEEDUP,
            ..runner::Config::default()
        },
    );
    runner.configure().expect("configures");
    runner.home().expect("homes");
    let (_tx, rx) = mpsc::channel();
    let outcome = runner.run(&c.program, &rx, &mut |_| {}).expect("runs");
    assert_eq!(outcome, runner::Outcome::Finished);
}
