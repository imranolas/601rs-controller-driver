//! What each reading of calibration sheet 2's strips and window buys, before anything is cut:
//! each unknown's standard error with every reading, then with each one left out, then with the
//! calibration's values known and only the head positions read.
//!
//! ```sh
//! cargo run -p calibrate --example readings
//! ```

use std::collections::HashMap;

use calibrate::{CALIPER_SD_MM, Precision, Prior, Sheet, Unknown, precision};
use machine::Profile;

const READINGS: [&str; 11] = [
    "SL.width",
    "SR.width",
    "SL.length",
    "S.left",
    "S.front",
    "W.left",
    "W.width",
    "W.front",
    "W.height",
    "D.width",
    "D.height",
];

fn sheet(names: &[&str]) -> Sheet {
    let profile = Profile::gunnar_601();
    let readings: HashMap<String, f64> = names.iter().map(|n| ((*n).to_owned(), 0.0)).collect();
    let program = calibrate::sheet::program(&profile);
    let (_, listed) = calibrate::legacy::readings(&program).expect("listed");
    Sheet::lowered("sheet 2", program, &profile.raw(), listed, readings).expect("lowers")
}

fn name(u: Unknown) -> String {
    format!("{:?} {}", u.head(), u.label()).to_lowercase()
}

fn row(p: &Precision, order: &[Unknown]) -> String {
    order
        .iter()
        .map(|u| {
            p.get(*u)
                .map_or_else(|| "    –".to_owned(), |se| format!("{se:5.3}"))
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn main() {
    // The bevel's width held, as stage 1 holds it: the sheet reads its sides on different faces.
    let held = calibrate::heads::priors(&calibrate::Calibration::new());
    let all = precision(&sheet(&READINGS), &held).expect("a plan");
    let order: Vec<Unknown> = all.values.iter().map(|(u, _)| *u).collect();
    println!("caliper readings scattering {CALIPER_SD_MM} mm; standard errors, mm\n");
    for (i, u) in order.iter().enumerate() {
        println!("  u{i}  {}", name(*u));
    }
    let head: String = (0..order.len())
        .map(|i| format!("   u{i}"))
        .collect::<Vec<_>>()
        .join(" ");
    println!("\n{:22} {head}", "readings");
    println!("{:22} {}", "all 11", row(&all, &order));
    for left_out in READINGS {
        let rest: Vec<&str> = READINGS
            .iter()
            .copied()
            .filter(|n| *n != left_out)
            .collect();
        let p = precision(&sheet(&rest), &held).expect("a plan");
        println!("{:22} {}", format!("without {left_out}"), row(&p, &order));
    }
    // A check: the slits known from the calibration to ±0.03 mm (the bevel's run to 0.02 per
    // mm), only the four head positions read.
    let known: Vec<Prior> = order
        .iter()
        .filter(|u| !matches!(u, Unknown::P(..)))
        .map(|&u| Prior {
            unknown: u,
            value: 0.0,
            se_mm: if matches!(u, Unknown::Run(_)) {
                0.02
            } else {
                0.03
            },
        })
        .collect();
    let four = ["S.left", "S.front", "W.left", "W.front"];
    let p = precision(&sheet(&four), &known).expect("a plan");
    println!("{:22} {}", "4 positions, edges known", row(&p, &order));
}
