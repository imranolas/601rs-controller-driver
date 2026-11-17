//! With the `schemars` feature, each type that serialises has a schema of its own name. Two types
//! that shared one would come out numbered (`Change2`), by whichever a generator met first.
#![cfg(feature = "schemars")]

#[test]
fn every_schema_has_a_name_of_its_own() {
    let mut g = schemars::SchemaGenerator::default();
    g.subschema_for::<calibrate::Calibration>();
    g.subschema_for::<calibrate::Proposal>();
    g.subschema_for::<calibrate::Session>();
    g.subschema_for::<calibrate::ValueState>();
    g.subschema_for::<calibrate::fit::Leftover>();
    g.subschema_for::<program::Cut>();
    let numbered: Vec<_> = g
        .definitions()
        .keys()
        .filter(|name| name.ends_with(char::is_numeric))
        .collect();
    assert!(numbered.is_empty(), "schemas sharing a name: {numbered:?}");
}
