//! STEP round-trip audit 2026-09-18, Finding 3.
//!
//! Fixture `crates/io/tests/data/shapr_untrimmed_nurbs_domain.step` (6,600
//! bytes; 4 faces Pl3 Nurb1, genuine source `B_SPLINE_SURFACE_WITH_KNOTS`)
//! keeps topology/types/volume across the round trip (4→4 faces, identical
//! hists, volume rel 6.3e-16) but its surface area collapses
//! 19.495054989860726 → 17.67196035454634 (abs −1.823, rel 9.351575e-2,
//! re-verified by the audit parent at deflection 0.01). Volume pinned while
//! area moves 9% means the NURBS patch parameterization (not its closed
//! volume) changed on the write→read leg.
//!
//! Suspected path (read-only, not fixed here): untrimmed-NURBS domain
//! handling — reader `MAX_UNTRIMMED_NURBS_RECOVERY_*` + the
//! `step_untrimmed_nurbs_domain_recovered` probe (`reader.rs` ~lines 56–133),
//! `build_bspline_surface` and the periodic/untrimmed UV-domain helpers
//! (~2335–2680), vs writer NURBS surface/edge-domain emission in
//! `crates/io/src/step/writer.rs`. The re-import likely recovers a different
//! (smaller) sub-domain. The owner should dump both carriers (control net,
//! knots, recovered domain) before/after.
//!
//! Full audit: `docs/kernel-maturity/step-roundtrip-audit-2026-09-18.md`.
//! Remove the `#[ignore]` when area rel < 1e-9.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

#[test]
#[ignore = "open: shapr_untrimmed_nurbs_domain STEP round-trip area drifts 9.35e-2; see step-roundtrip-audit-2026-09-18 Finding 3"]
fn untrimmed_nurbs_area_survives_roundtrip() {
    let contents = std::fs::read_to_string(fixture("shapr_untrimmed_nurbs_domain.step")).unwrap();
    let mut topo = remus_topology::Topology::new();
    let solids = remus_io::step::reader::read_step(&contents, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    let vol_before: f64 = solids
        .iter()
        .map(|s| remus_operations::measure::solid_volume(&topo, *s, 0.01).unwrap())
        .sum();
    let area_before: f64 = solids
        .iter()
        .map(|s| remus_operations::measure::solid_surface_area(&topo, *s, 0.01).unwrap())
        .sum();

    let exported = remus_io::step::writer::write_step(&topo, &solids).unwrap();
    let mut topo2 = remus_topology::Topology::new();
    let solids2 = remus_io::step::reader::read_step(&exported, &mut topo2).unwrap();
    assert_eq!(solids2.len(), 1);
    let vol_after: f64 = solids2
        .iter()
        .map(|s| remus_operations::measure::solid_volume(&topo2, *s, 0.01).unwrap())
        .sum();
    let area_after: f64 = solids2
        .iter()
        .map(|s| remus_operations::measure::solid_surface_area(&topo2, *s, 0.01).unwrap())
        .sum();

    let vol_rel = (vol_after - vol_before).abs() / vol_before.abs();
    let area_rel = (area_after - area_before).abs() / area_before.abs();
    assert!(
        vol_rel < 1e-9,
        "untrimmed_nurbs volume should survive: before={vol_before:.8}, after={vol_after:.8}, rel={vol_rel:.3e}"
    );
    assert!(
        area_rel < 1e-9,
        "untrimmed_nurbs area should survive STEP round-trip within 1e-9: before={area_before:.8}, after={area_after:.8}, rel={area_rel:.3e}"
    );
}
