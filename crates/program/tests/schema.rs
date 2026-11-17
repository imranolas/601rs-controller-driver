//! `program.schema.json`, at the workspace root, is the program types' own schema: generated
//! from them, so it can't drift from what the driver reads and writes. `BLESS=1` writes it again.
#![cfg(feature = "schemars")]

use std::path::Path;

#[test]
fn the_schema_file_is_the_types_own() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../program.schema.json");
    let schema = schemars::schema_for!(program::Program);
    let text = serde_json::to_string_pretty(&schema).expect("serialises") + "\n";
    if std::env::var_os("BLESS").is_some() {
        std::fs::write(&path, &text).expect("written");
    }
    assert!(
        std::fs::read_to_string(&path).expect("reads") == text,
        "program.schema.json is not the types' schema: BLESS=1 cargo test -p program --features schemars"
    );
}
