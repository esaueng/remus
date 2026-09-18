//! STEP round-trip audit 2026-09-18, Finding 1.
//!
//! Fixture `crates/io/tests/data/mambo_b1_untrimmed_nurbs.step` (39,442 bytes)
//! is refused at import, so no export→re-import leg exists:
//! `parse error: EDGE_CURVE #253 start endpoint misses its carrier by
//! 6.164947e-5 mm (local recovery cap 1.000000e-6 mm)` (re-verified by the
//! audit parent at deflection 0.01; file stores 8 CYLINDRICAL_SURFACE carriers
//! with B_SPLINE trims over 32 EDGE_CURVEs).
//!
//! Suspected reader path (read-only, not fixed here):
//! `crates/io/src/step/reader.rs` curved-edge import adapter
//! (`endpoint_error` near line 4641 and the Circle/NURBS residual checks
//! ~4673–4958; projected recovery clamped by
//! `MAX_PROJECTED_NURBS_RECOVERY_TOLERANCE_MM` = 1e-6 vs the 1e-4 untrimmed
//! cap, ~lines 56–67/4820–4846). Open question for the owner: whether #253
//! qualifies for the wider path or for heal-after-import (`heal_solid` /
//! `convert_to_elementary`).
//!
//! Full audit: `docs/kernel-maturity/step-roundtrip-audit-2026-09-18.md`.
//! Remove the `#[ignore]` when import succeeds, and extend with the round-trip
//! histogram/volume asserts.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

#[test]
#[ignore = "open: mambo_b1_untrimmed_nurbs STEP import refused (EDGE_CURVE #253 carrier miss 6.16e-5 vs 1e-6 cap); see step-roundtrip-audit-2026-09-18 Finding 1"]
fn mambo_b1_import_is_refused() {
    let contents = std::fs::read_to_string(fixture("mambo_b1_untrimmed_nurbs.step")).unwrap();
    let limits = remus_io::ImportLimits::default();
    assert!(
        contents.len() <= limits.max_input_bytes,
        "fixture must respect import limits"
    );
    let mut topo = remus_topology::Topology::new();
    let solids = remus_io::step::reader::read_step(&contents, &mut topo)
        .expect("mambo_b1 should import without refusal");
    assert_eq!(solids.len(), 1, "expected one solid, got {}", solids.len());
}
