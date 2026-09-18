//! STEP round-trip audit 2026-09-18, Finding 2.
//!
//! Fixture `crates/io/tests/data/shapr3d_hammer_holder.step` (931,835 bytes,
//! 160 faces: Pl52 Cyl42 Cone2 Sph8 Tor14 Nurb42) round-trips its topology and
//! types bit-identically (160→160 faces, identical FaceSurface/EdgeCurve
//! histograms, area rel 0.0) but its volume drifts 5.02406431268449305e4 →
//! 5.02406460872056996e4 (abs +0.00296, rel 5.892362e-8 > 1e-9 threshold,
//! re-verified by the audit parent at deflection 0.01).
//!
//! Suspected path (read-only, not fixed here): the NURBS write→read leg —
//! writer emits full precision (`fmt_f64`/`fmt_weight` use `{:.17E}` in
//! `crates/io/src/step/writer.rs` ~line 1660), so the drift likely comes from
//! NURBS re-parameterization / edge-domain re-derivation
//! (`reader.rs::build_bspline_surface` + periodic/untrimmed domain helpers
//! ~2335–2680) re-tessellating the 42 NURBS faces differently at 0.01, not
//! from decimal rounding. The owner should re-measure both sides at finer
//! deflection and compare `oriented_solid_volume` before calling it a writer
//! bug.
//!
//! Full audit: `docs/kernel-maturity/step-roundtrip-audit-2026-09-18.md`.
//! Remove the `#[ignore]` when rel drift < 1e-9.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

#[test]
#[ignore = "open: shapr3d_hammer_holder STEP round-trip volume drifts 5.89e-8 (> 1e-9); see step-roundtrip-audit-2026-09-18 Finding 2"]
fn hammer_holder_volume_survives_roundtrip() {
    let contents = std::fs::read_to_string(fixture("shapr3d_hammer_holder.step")).unwrap();
    let mut topo = remus_topology::Topology::new();
    let solids = remus_io::step::reader::read_step(&contents, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    let before: f64 = solids
        .iter()
        .map(|s| remus_operations::measure::solid_volume(&topo, *s, 0.01).unwrap())
        .sum();

    let exported = remus_io::step::writer::write_step(&topo, &solids).unwrap();
    let mut topo2 = remus_topology::Topology::new();
    let solids2 = remus_io::step::reader::read_step(&exported, &mut topo2).unwrap();
    assert_eq!(solids2.len(), 1);
    let after: f64 = solids2
        .iter()
        .map(|s| remus_operations::measure::solid_volume(&topo2, *s, 0.01).unwrap())
        .sum();

    let rel = (after - before).abs() / before.abs();
    assert!(
        rel < 1e-9,
        "hammer_holder volume should survive STEP round-trip within 1e-9: before={before:.8}, after={after:.8}, rel={rel:.3e}"
    );
}
