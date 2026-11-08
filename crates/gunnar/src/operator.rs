//! The person at the machine. On the simulator, actions are performed automatically and
//! every confirmation is answered "yes".

use std::io::{self, BufRead, Write};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use sim::Sim;

pub struct Operator {
    sim: Option<Arc<Sim>>,
}

impl Operator {
    pub fn new(sim: Option<Arc<Sim>>) -> Self {
        Self { sim }
    }

    pub fn is_sim(&self) -> bool {
        self.sim.is_some()
    }

    /// One line from the operator, trimmed and lower-cased. `None` if stdin is closed or
    /// unreadable.
    fn read_line() -> Option<String> {
        let mut s = String::new();
        match io::stdin().lock().read_line(&mut s) {
            Ok(0) | Err(_) => None,
            Ok(_) => Some(s.trim().to_lowercase()),
        }
    }

    /// Describe what is about to happen and wait for Enter. `s` skips; `q` quits the run.
    /// Only Enter goes: anything else asks again, so a "no" never starts a move. End of input
    /// quits: it is never consent, because nobody is there to watch the machine.
    pub fn confirm(&self, what: &str) -> Confirm {
        if self.is_sim() {
            return Confirm::Go;
        }
        println!("\n  >> {what}");
        loop {
            print!("     Enter = go, s = skip this step, q = stop the check: ");
            let _ = io::stdout().flush();
            match Self::read_line().as_deref() {
                None => {
                    println!("\n     (no operator input: stdin is closed; stopping)");
                    return Confirm::Quit;
                }
                Some("") => return Confirm::Go,
                Some("s") => return Confirm::Skip,
                Some("q") => return Confirm::Quit,
                Some(_) => {}
            }
        }
    }

    /// Ask the operator what they saw. `None` on the simulator.
    pub fn ask(&self, question: &str) -> Option<String> {
        if self.is_sim() {
            return None;
        }
        print!("  ?? {question} ");
        let _ = io::stdout().flush();
        Some(Self::read_line().unwrap_or_else(|| "(no answer: stdin is closed)".to_owned()))
    }

    /// Ask the operator to press and release the green start button. On the simulator the
    /// press happens `after` from now, so the caller can be waiting for it.
    pub fn press_green(&self, after: Duration) {
        match &self.sim {
            Some(sim) => {
                let sim = Arc::clone(sim);
                thread::spawn(move || {
                    thread::sleep(after);
                    sim.press_start_button();
                });
            }
            None => println!("  >> Press and release the GREEN start button now."),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Confirm {
    Go,
    Skip,
    Quit,
}
