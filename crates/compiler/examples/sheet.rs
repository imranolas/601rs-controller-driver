//! Nested sheets compiled: what each costs on the machine, with straight cuts run away from the
//! clamps (the default, as GMC on the 601) and either way. Writes the first sheet's program.
//!
//! cargo run --release -p compiler --example sheet [program.json]
//! mc601 run --sim program.json

use compiler::{Blade, Contour, Keep, Options, Shape, Sheet, Tool, compile};
use machine::Profile;
use nest::{Bin, Instance, Item, Objective, Trims};
use program::{BladeOffsets, Board, Material};

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

/// A queue of common mat sizes on 32 × 40 in sheets, each mat with one to three windows, one
/// in five an oval; the sheets as they come off the nester.
fn sheets(rng: &mut Rng, mats: u32) -> Vec<Sheet> {
    // Tenths of a millimetre: 8 × 10 and 11 × 14 in, A4, 30 × 40 cm, a landscape 25 × 20 cm.
    let sizes = [
        (2030, 2540),
        (2790, 3560),
        (2100, 2970),
        (3000, 4000),
        (2500, 2000),
    ];
    let mut items = Vec::new();
    let mut left = mats;
    while left > 0 {
        let (w, h) = sizes[rng.next() as usize % sizes.len()];
        let copies = (1 + rng.next() as u32 % 3).min(left);
        left -= copies;
        items.push(Item {
            w,
            h,
            copies,
            rotate: true,
            ..Item::default()
        });
    }
    let instance = Instance {
        objective: Objective::BinPacking,
        bins: vec![Bin {
            w: 8150,
            h: 10150,
            // GMC's 21 mm in from the clamped edges (its `MatLayX`/`MatLayY` when the captures were
            // made), so no cut runs over a clamp pad; 5 mm elsewhere.
            trims: Trims {
                left: 210,
                right: 50,
                top: 50,
                bottom: 210,
            },
            copies: mats,
        }],
        items,
    };
    let s = nest::solve(&instance, &nest::Options::default()).expect("lays out");
    (0..s.bins_used)
        .map(|bin| {
            let mut contours = Vec::new();
            for p in s.placements.iter().filter(|p| p.bin == bin) {
                let [x0, y0, w, h] = [p.x, p.y, p.w, p.h].map(|v| f64::from(v) / 10.0);
                let windows = 1 + rng.next() as usize % 3;
                let (border, gap) = (40.0, 20.0);
                let each = (w - 2.0 * border - gap * (windows - 1) as f64) / windows as f64;
                for k in 0..windows {
                    let wx = x0 + border + k as f64 * (each + gap);
                    contours.push(Contour {
                        id: None,
                        label: Some(format!("Opening {}", contours.len() + 1)),
                        shape: if rng.next() % 5 == 0 {
                            Shape::Oval
                        } else {
                            Shape::Rect
                        },
                        box_mm: [wx, y0 + border, wx + each, y0 + h - border],
                        tool: Tool::Bevel,
                        keep: Keep::Out,
                    });
                }
                contours.push(Contour {
                    id: None,
                    label: Some(format!("Mount {}", contours.len() + 1)),
                    shape: Shape::Rect,
                    box_mm: [x0, y0, x0 + w, y0 + h],
                    tool: Tool::Straight,
                    keep: Keep::In,
                });
            }
            Sheet {
                id: format!("sheet-{}", bin + 1),
                meta: None,
                board: Board {
                    width_mm: 815.0,
                    height_mm: 1015.0,
                    thickness_mm: 1.5,
                },
                material: Material {
                    name: "MSK_1.5".to_owned(),
                    speed_pct: 95.0,
                    head_settle_ms: 250,
                    blade_angle_offset_deg: BladeOffsets {
                        straight: 0.0,
                        bevel: 3.0,
                        bevel_round: None,
                    },
                },
                contours,
                cleanup: Vec::new(),
            }
        })
        .collect()
}

fn main() {
    let profile = Profile::gunnar_601();
    let blade = Blade::fitted();
    let away = Options::default();
    let either = Options {
        straight_away_from_clamps: false,
        ..Options::default()
    };
    let mut rng = Rng(2026);
    let all = sheets(&mut rng, 24);
    println!(
        "| Sheet | Mounts | Cuts | Pads turned | Run time | Travel | Travel, straight cuts either way |"
    );
    println!(
        "|------:|-------:|-----:|:------------|---------:|-------:|---------------------------------:|"
    );
    let (mut t_away, mut t_either) = (0.0, 0.0);
    for s in &all {
        let a = compile(s, &blade, &profile, &away).expect("compiles");
        let e = compile(s, &blade, &profile, &either).expect("compiles");
        let mounts = s.contours.iter().filter(|c| c.keep == Keep::In).count();
        let cuts = a.program.ops.len() - 1;
        t_away += a.estimate.travel_s;
        t_either += e.estimate.travel_s;
        println!(
            "| {} | {mounts} | {cuts} | {} | {:.0} s | {:.1} s | {:.1} s |",
            s.id,
            if a.turned.is_empty() {
                "none".to_owned()
            } else {
                a.turned.join(", ")
            },
            a.estimate.total_s,
            a.estimate.travel_s,
            e.estimate.travel_s,
        );
    }
    println!(
        "\nHead-up travel in all: {t_away:.1} s with straight cuts away from the clamps, \
         {t_either:.1} s either way."
    );
    if let Some(path) = std::env::args().nth(1) {
        let first = compile(&all[0], &blade, &profile, &away).expect("compiles");
        let json = serde_json::to_string_pretty(&first.program).expect("serialises");
        std::fs::write(&path, json).expect("writes");
        println!("Wrote {} to {path}.", all[0].id);
    }
}
