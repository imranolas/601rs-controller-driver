//! Terminal styles: colour that carries meaning and nothing else. Print styled text with
//! `anstream::println!`, which drops the colour when the output isn't a terminal or `NO_COLOR`
//! is set; the logs never get it.

use anstyle::{AnsiColor, Color, Style};

const fn fg(c: AnsiColor) -> Style {
    Style::new().fg_color(Some(Color::Ansi(c)))
}

/// Passed, applied, done.
pub const GOOD: Style = fg(AnsiColor::Green).bold();
/// Failed, refused, an error.
pub const BAD: Style = fg(AnsiColor::Red).bold();
/// Worth a second look: a value that moved, a reading far from expected.
pub const WARN: Style = fg(AnsiColor::Yellow).bold();
/// Secondary: hints, key help, units, context.
pub const DIM: Style = Style::new().dimmed();
/// What the operator is being asked to do.
pub const BOLD: Style = Style::new().bold();

/// `text` in `style`, for `anstream::println!`.
pub fn paint(style: Style, text: impl std::fmt::Display) -> String {
    format!("{style}{text}{style:#}")
}
