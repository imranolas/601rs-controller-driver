//! The program format v1 (see `PROGRAM_FORMAT.md`).
//!
//! Parsing rejects structurally invalid programs (unknown fields, unknown ops, bad enums).
//! The semantic rules (V4 segment length, V5 within board, V7 break angle, V8 unique ids)
//! will live in `validate`, which is still to come; V10 (machine envelope) needs a
//! machine profile and belongs to the runner.

use serde::{Deserialize, Serialize};

pub const FORMAT: &str = "gunnar-program";
pub const VERSION: u32 = 1;

pub type PointMm = [f64; 2];

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Program {
    pub format: String,
    pub version: u32,
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub meta: Option<serde_json::Value>,
    pub board: Board,
    pub material: Material,
    pub ops: Vec<Op>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Board {
    pub width_mm: f64,
    pub height_mm: f64,
    pub thickness_mm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Material {
    pub name: String,
    pub speed_pct: f64,
    pub head_settle_ms: u32,
    #[serde(default)]
    pub blade_angle_offset_deg: BladeOffsets,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BladeOffsets {
    #[serde(default)]
    pub straight: f64,
    #[serde(default)]
    pub bevel: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Head {
    Straight,
    Bevel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClampSide {
    Left,
    Right,
    Both,
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Resume {
    StartButton,
    Operator,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Op {
    Clamp(Clamp),
    Cut(Cut),
    Move(Move),
    Pause(Pause),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Clamp {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    pub side: ClampSide,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Cut {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    pub head: Head,
    pub path_mm: Vec<PointMm>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_pct: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Move {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    pub to_mm: PointMm,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pause {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    pub message: String,
    pub resume: Resume,
}

#[derive(Debug, thiserror::Error)]
pub enum ProgramError {
    #[error("not a program: {0}")]
    Parse(#[from] serde_json::Error),
    #[error("V1: format must be \"{FORMAT}\", found \"{0}\"")]
    WrongFormat(String),
    #[error("V1: unsupported version {0} (this build supports {VERSION})")]
    UnsupportedVersion(u32),
}

impl Program {
    /// Parse and check identity (rule V1). Structural rules are enforced by the types.
    pub fn from_json(text: &str) -> Result<Program, ProgramError> {
        let p: Program = serde_json::from_str(text)?;
        if p.format != FORMAT {
            return Err(ProgramError::WrongFormat(p.format));
        }
        if p.version != VERSION {
            return Err(ProgramError::UnsupportedVersion(p.version));
        }
        Ok(p)
    }
}
