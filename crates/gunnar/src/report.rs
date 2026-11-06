//! Findings of a probe or check run: printed as they happen, noted in the log, summarised.

use std::fmt;

use link::tap::Log;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// Behaves as the driver expects.
    Pass,
    /// Differs from what the driver expects: needs a fix or a doc correction.
    Fail,
    /// A fact we did not know (a value, an answer to an open question).
    Info,
    /// Not run, with the reason.
    Skip,
}

impl fmt::Display for Outcome {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Outcome::Pass => "PASS",
            Outcome::Fail => "FAIL",
            Outcome::Info => "INFO",
            Outcome::Skip => "SKIP",
        })
    }
}

#[derive(Debug, Clone)]
pub struct Finding {
    pub outcome: Outcome,
    pub name: String,
    pub detail: String,
}

pub struct Report {
    log: Log,
    pub findings: Vec<Finding>,
}

impl Report {
    pub fn new(log: Log) -> Self {
        Self {
            log,
            findings: Vec::new(),
        }
    }

    pub fn add(&mut self, outcome: Outcome, name: &str, detail: impl Into<String>) {
        let detail = detail.into();
        let line = format!("{outcome}  {name}: {detail}");
        println!("  {line}");
        self.log.note(&line);
        self.findings.push(Finding {
            outcome,
            name: name.to_owned(),
            detail,
        });
    }

    pub fn pass(&mut self, name: &str, detail: impl Into<String>) {
        self.add(Outcome::Pass, name, detail);
    }
    pub fn fail(&mut self, name: &str, detail: impl Into<String>) {
        self.add(Outcome::Fail, name, detail);
    }
    pub fn info(&mut self, name: &str, detail: impl Into<String>) {
        self.add(Outcome::Info, name, detail);
    }
    pub fn skip(&mut self, name: &str, detail: impl Into<String>) {
        self.add(Outcome::Skip, name, detail);
    }
    /// Pass if `ok`, otherwise fail, with the same detail.
    pub fn check(&mut self, ok: bool, name: &str, detail: impl Into<String>) {
        self.add(if ok { Outcome::Pass } else { Outcome::Fail }, name, detail);
    }

    pub fn heading(&self, text: &str) {
        println!("\n== {text}");
        self.log.note(&format!("== {text}"));
    }

    /// Print the summary; true if nothing failed.
    pub fn summary(&self) -> bool {
        let count = |o| self.findings.iter().filter(|f| f.outcome == o).count();
        let (p, f, i, s) = (
            count(Outcome::Pass),
            count(Outcome::Fail),
            count(Outcome::Info),
            count(Outcome::Skip),
        );
        println!("\n== Summary: {p} pass, {f} fail, {i} info, {s} skipped");
        for x in self.findings.iter().filter(|x| x.outcome == Outcome::Fail) {
            println!("  FAIL  {}: {}", x.name, x.detail);
        }
        self.log.note(&format!(
            "summary: {p} pass, {f} fail, {i} info, {s} skipped"
        ));
        f == 0
    }
}
