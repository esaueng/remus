//! Pin the README's first code example end-to-end.
//!
//! The example cuts a cylinder (radius 5, axis = z through the origin) from a
//! 30x20x10 box whose corner sits at the origin — the cylinder axis is exactly
//! coincident with the box's vertical corner edge, so this is the recurring
//! tangential-contact class. A flagship example that errors is worse than a
//! modest one; this test keeps it green.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_io::step::reader::read_step;
use remus_io::step::write_step;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use std::collections::BTreeMap;

/// Face census by surface type tag.
fn face_census(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(fid).unwrap().surface().type_tag())
            .or_default() += 1;
    }
    census
}

#[test]
fn readme_first_example_runs_end_to_end() {
    let mut topo = Topology::new();

    // A block with a cylindrical hole — verbatim from the README.
    let block = make_box(&mut topo, 30.0, 20.0, 10.0).unwrap();
    let hole = make_cylinder(&mut topo, 5.0, 15.0).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, block, hole)
        .expect("README example cut must succeed (axis-on-corner-edge tangential contact)");

    // Analytic result, not a mesh fallback: exactly the quarter-groove faces.
    let census = face_census(&topo, drilled);
    assert_eq!(
        census.get("cylinder"),
        Some(&1),
        "groove face must stay an analytic cylinder: {census:?}"
    );
    assert_eq!(
        census.values().sum::<usize>(),
        7,
        "expected 6 planes + 1 cylinder: {census:?}"
    );

    // Volume: box minus the quarter cylinder inside it.
    let vol = solid_volume(&topo, drilled, 0.1).unwrap();
    let expected = 30.0 * 20.0 * 10.0 - std::f64::consts::PI * 25.0 * 10.0 / 4.0;
    let rel = (vol - expected).abs() / expected;
    assert!(
        rel < 0.01,
        "volume {vol:.3} vs expected {expected:.3} (rel {rel:.4})"
    );

    // Ray-cast ground truth (volume alone can mask an un-carved cut): points
    // inside the drilled quarter-cylinder must be Outside, neighbors Inside.
    let probes = [
        (Point3::new(1.0, 1.0, 5.0), PointClassification::Outside),
        (Point3::new(2.0, 2.0, 9.5), PointClassification::Outside),
        (Point3::new(3.0, 3.0, 0.5), PointClassification::Outside),
        (Point3::new(4.0, 4.0, 5.0), PointClassification::Inside),
        (Point3::new(15.0, 10.0, 5.0), PointClassification::Inside),
        (Point3::new(0.5, 6.0, 5.0), PointClassification::Inside),
        (Point3::new(6.0, 0.5, 5.0), PointClassification::Inside),
    ];
    for (p, want) in probes {
        let got = classify_point(&topo, drilled, p, 0.05, 1e-6).unwrap();
        assert_eq!(got, want, "classify ({}, {}, {})", p.x(), p.y(), p.z());
    }

    // Export and analytic re-import round-trip.
    let step = write_step(&topo, &[drilled]).unwrap();
    assert!(!step.is_empty());
    let mut topo2 = Topology::new();
    let solids = read_step(&step, &mut topo2).unwrap();
    assert_eq!(solids.len(), 1, "STEP round-trip must yield one solid");
    let census2 = face_census(&topo2, solids[0]);
    assert_eq!(
        census2.get("cylinder"),
        Some(&1),
        "STEP round-trip must keep the cylinder analytic: {census2:?}"
    );
}
