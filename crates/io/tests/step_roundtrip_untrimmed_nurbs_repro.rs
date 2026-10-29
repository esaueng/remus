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
//! This regression pins volume, area, and exact NURBS edge parameter
//! authority. Remus's own export cannot use `TRIMMED_CURVE` edge geometry
//! (ISO 10303-42 forbids it), so it writes each partial edge's carrier
//! restricted to the edge's span by knot insertion: the round trip keeps
//! every trim bit for bit and traces the same points, on a restricted
//! carrier whose knot domain is that trim.
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

    let exported = remus_io::step::writer::write_step(&topo, &solids).unwrap();
    let mut topo2 = remus_topology::Topology::new();
    let solids2 = remus_io::step::reader::read_step(&exported, &mut topo2).unwrap();
    assert_eq!(solids2.len(), 1);

    // Each NURBS edge as (exact trim bits, points at ninths of its span,
    // knot domain), sorted by trim.
    type Signature = ((u64, u64), Vec<remus_math::vec::Point3>, (f64, f64));
    let edge_signatures = |topology: &remus_topology::Topology,
                           solid: remus_topology::solid::SolidId| {
        let mut signatures: Vec<Signature> = remus_topology::explorer::solid_edges(topology, solid)
            .unwrap()
            .into_iter()
            .filter_map(|edge_id| {
                let edge = topology.edge(edge_id).unwrap();
                let remus_topology::edge::EdgeCurve::NurbsCurve(nurbs) = edge.curve() else {
                    return None;
                };
                let (t0, t1) = edge.trim().unwrap();
                let points = (0..=9)
                    .map(|k| nurbs.evaluate(t0 + (t1 - t0) * f64::from(k) / 9.0))
                    .collect();
                Some(((t0.to_bits(), t1.to_bits()), points, nurbs.domain()))
            })
            .collect();
        signatures.sort_by_key(|signature| signature.0);
        signatures
    };
    let source = edge_signatures(&topo, solids[0]);
    let round = edge_signatures(&topo2, solids2[0]);
    assert!(!source.is_empty());
    assert_eq!(source.len(), round.len());
    for ((source_trim, source_points, _), (round_trim, round_points, round_domain)) in
        source.iter().zip(&round)
    {
        assert_eq!(
            source_trim, round_trim,
            "NURBS edge trims must remain exact"
        );
        let (lo, hi) = {
            let (t0, t1) = (f64::from_bits(round_trim.0), f64::from_bits(round_trim.1));
            if t0 < t1 { (t0, t1) } else { (t1, t0) }
        };
        assert_eq!(
            (round_domain.0.to_bits(), round_domain.1.to_bits()),
            (lo.to_bits(), hi.to_bits()),
            "the exported carrier is restricted to the edge's span"
        );
        for (a, b) in source_points.iter().zip(round_points) {
            assert!(
                (*a - *b).length() <= 1e-12,
                "NURBS edge moved by {}",
                (*a - *b).length()
            );
        }
    }
    let vol_after: f64 = solids2
        .iter()
        .map(|s| remus_operations::measure::solid_volume(&topo2, *s, 0.01).unwrap())
        .sum();

    let vol_rel = (vol_after - vol_before).abs() / vol_before.abs();
    assert!(
        vol_rel < 1e-9,
        "untrimmed_nurbs volume should survive: before={vol_before:.8}, after={vol_after:.8}, rel={vol_rel:.3e}"
    );

    // The prism is 2 mm tall, so each planar cap (bounded by a NURBS edge
    // and its chord) covers volume / 2. The source measures its caps from
    // NURBS edges using part of a larger carrier, which the planar area path
    // over-counts (1.443 mm² per cap, not 0.5315); the round trip writes
    // carriers restricted to each edge, whose caps agree with the volume to
    // the accuracy of the two numerical paths (1e-5). Every
    // other face keeps its area.
    let face_areas = |topology: &remus_topology::Topology,
                      solid: remus_topology::solid::SolidId| {
        let mut caps = Vec::new();
        let mut others = Vec::new();
        for face in remus_topology::explorer::solid_faces(topology, solid).unwrap() {
            let area = remus_operations::measure::face_area(topology, face, 0.01).unwrap();
            let planar = topology.face(face).unwrap().surface().is_planar();
            let nurbs_bounded = remus_topology::explorer::face_edges(topology, face)
                .unwrap()
                .into_iter()
                .any(|edge| {
                    matches!(
                        topology.edge(edge).unwrap().curve(),
                        remus_topology::edge::EdgeCurve::NurbsCurve(_)
                    )
                });
            if planar && nurbs_bounded {
                caps.push(area);
            } else {
                others.push(area);
            }
        }
        others.sort_by(f64::total_cmp);
        (caps, others)
    };
    let (_, source_others) = face_areas(&topo, solids[0]);
    let (round_caps, round_others) = face_areas(&topo2, solids2[0]);
    assert_eq!(round_caps.len(), 2);
    for cap in &round_caps {
        let expected = vol_before / 2.0;
        assert!(
            (cap - expected).abs() / expected < 1e-5,
            "round-tripped cap area {cap:.10} != volume / height {expected:.10}"
        );
    }
    assert_eq!(source_others.len(), round_others.len());
    for (before, after) in source_others.iter().zip(&round_others) {
        assert!(
            (after - before).abs() / before.abs() < 1e-9,
            "untrimmed_nurbs face area should survive STEP round-trip: before={before:.8}, after={after:.8}"
        );
    }
}
