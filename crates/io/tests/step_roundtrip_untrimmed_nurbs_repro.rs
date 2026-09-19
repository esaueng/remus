//! STEP round-trip audit 2026-09-18, Finding 3.
//!
//! Fixture `crates/io/tests/data/shapr_untrimmed_nurbs_domain.step` (6,600
//! bytes; 4 faces Pl3 Nurb1, genuine source `B_SPLINE_SURFACE_WITH_KNOTS`)
//! keeps topology/types/volume across the round trip. The historical reader
//! physically split parameter-trimmed NURBS edge carriers while also storing
//! the declared edge interval, replacing their exact control polygons and
//! collapsing surface area by 9.35%. The reader now retains each complete
//! carrier and stores the trim only as edge parameter authority.
//!
//! This regression pins volume, area, and exact NURBS edge carrier identity.
//!
//! Full audit: `docs/kernel-maturity/step-roundtrip-audit-2026-09-18.md`.
//! Fixed by retaining the basis carrier for parameter-trimmed NURBS curves.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

#[test]
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

    let carrier_signatures = |topology: &remus_topology::Topology,
                              solid: remus_topology::solid::SolidId| {
        let mut signatures = remus_topology::explorer::solid_edges(topology, solid)
            .unwrap()
            .into_iter()
            .filter_map(|edge_id| {
                let edge = topology.edge(edge_id).unwrap();
                matches!(edge.curve(), remus_topology::edge::EdgeCurve::NurbsCurve(_))
                    .then(|| format!("{:?}|{:?}", edge.curve(), edge.trim()))
            })
            .collect::<Vec<_>>();
        signatures.sort();
        signatures
    };
    assert_eq!(
        carrier_signatures(&topo, solids[0]),
        carrier_signatures(&topo2, solids2[0]),
        "NURBS edge carriers and their authoritative trims must remain exact"
    );
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
