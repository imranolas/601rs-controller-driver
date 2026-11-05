use program::{Head, Op, Program, ProgramError};

const REGULAR_MOUNT: &str = include_str!("fixtures/regular_mount.json");

#[test]
fn parses_spec_example() {
    let p = Program::from_json(REGULAR_MOUNT).expect("valid program");
    assert_eq!(p.ops.len(), 9);
    let cuts: Vec<_> = p
        .ops
        .iter()
        .filter_map(|op| match op {
            Op::Cut(c) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(cuts.len(), 8);
    assert!(cuts[..4].iter().all(|c| c.head == Head::Bevel), "interior (bevel) first");
    assert!(cuts[4..].iter().all(|c| c.head == Head::Straight), "outer edge last");
}

#[test]
fn round_trips() {
    let p = Program::from_json(REGULAR_MOUNT).expect("valid program");
    let again = Program::from_json(&serde_json::to_string(&p).expect("serialize")).expect("reparse");
    assert_eq!(p, again);
}

#[test]
fn rejects_unknown_op() {
    let bad = REGULAR_MOUNT.replacen(r#""op": "clamp""#, r#""op": "head_down""#, 1);
    assert!(matches!(Program::from_json(&bad), Err(ProgramError::Parse(_))));
}

#[test]
fn rejects_unknown_field_in_op() {
    let bad = REGULAR_MOUNT.replacen(r#""side": "left""#, r#""side": "left", "force": 9"#, 1);
    assert!(matches!(Program::from_json(&bad), Err(ProgramError::Parse(_))));
}

#[test]
fn rejects_wrong_version() {
    let bad = REGULAR_MOUNT.replacen(r#""version": 1"#, r#""version": 2"#, 1);
    assert!(matches!(Program::from_json(&bad), Err(ProgramError::UnsupportedVersion(2))));
}
