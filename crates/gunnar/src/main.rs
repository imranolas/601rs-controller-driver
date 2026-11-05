//! `gunnar`: command-line tool.
//!
//! Subcommands:
//!   capture <file>   decode a GMC serial capture into one line per transfer
//!                    (`>` host→controller, `<` controller→host; control bytes as <xx>)

use std::process::ExitCode;

use link::capture::{self, Direction};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [cmd, file] if cmd == "capture" => decode_capture(file),
        _ => {
            eprintln!("usage: gunnar capture <g_*.txt>");
            ExitCode::from(2)
        }
    }
}

fn decode_capture(file: &str) -> ExitCode {
    let transfers = match capture::read_file(file) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("{file}: {e}");
            return ExitCode::FAILURE;
        }
    };
    for t in transfers {
        let arrow = match t.direction {
            Direction::HostToController => '>',
            Direction::ControllerToHost => '<',
        };
        let (h, m, s) = (t.at_secs / 3600, t.at_secs / 60 % 60, t.at_secs % 60);
        println!("{h:02}:{m:02}:{s:02} {arrow} {}", printable(&t.bytes));
    }
    ExitCode::SUCCESS
}

fn printable(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|&b| match b {
            b'\r' => "⏎".to_owned(),
            0x20..=0x7E => (b as char).to_string(),
            _ => format!("<{b:02x}>"),
        })
        .collect()
}
