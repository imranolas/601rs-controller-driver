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
    Sheet::lowered(
        "sheet 2",
        calibrate::sheet::program(&profile),
        &profile.raw(),
        readings,
    )
    .expect("lowers")
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
    let all = precision(&sheet(&READINGS), &[]).expect("a plan");
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
        let p = precision(&sheet(&rest), &[]).expect("a plan");
        println!("{:22} {}", format!("without {left_out}"), row(&p, &order));
    }
    // A check: the slit edges known from the calibration to ±0.03 mm, only the four head
    // positions read.
    let known: Vec<Prior> = order
        .iter()
        .filter(|u| matches!(u, Unknown::E(..)))
        .map(|&u| Prior {
            unknown: u,
            value: 0.0,
            se_mm: 0.03,
        })
        .collect();
    let four = ["S.left", "S.front", "W.left", "W.front"];
    let p = precision(&sheet(&four), &known).expect("a plan");
    println!("{:22} {}", "4 positions, edges known", row(&p, &order));
}
