//! The point classifier must read nested inner wires as alternating void and
//! material.
//!
//! The z = 21 wall top of `oring_nested_holes.step` has three nested inner
//! wires: the cavity opening (half-width 61.55) contains the island band
//! (outer half-width 22.45), which contains the central hole (21.25). Both
//! planar trim tests rejected a point in any hole, so a ray crossing the
//! island band missed it, and points under the band classified `Outside`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid};
use remus_math::vec::Point3;
use remus_topology::Topology;

#[test]
fn points_under_a_nested_island_band_are_inside() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/oring_nested_holes.step"
    );
    let mut topo = Topology::new();
    let text = std::fs::read_to_string(path).unwrap();
    let solid = remus_io::step::reader::read_step(&text, &mut topo).unwrap()[0];
    let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
    let options = ClassifyOptions::default();
    let inside = [
        // Mid-band on each side, just under the top.
        Point3::new(21.85, 0.0, 20.9),
        Point3::new(-21.85, 3.0, 20.9),
        Point3::new(5.0, 21.85, 20.9),
        Point3::new(-7.0, -21.85, 20.9),
        // Fine-mesh oracle probes the classifier had wrong (RFC 0008 harness).
        Point3::new(
            -20.390_852_500_699_527,
            -20.698_994_584_280_626,
            20.821_278_149_069_567,
        ),
        Point3::new(
            21.690_032_264_508_748,
            19.880_798_006_332_94,
            20.964_255_629_813_913,
        ),
    ];
    for point in inside {
        assert_eq!(
            prepared.classify_point(point, &options).unwrap(),
            PointClassification::Inside,
            "{point:?}"
        );
    }
    // Just above the band.
    for point in [
        Point3::new(21.85, 0.0, 21.1),
        Point3::new(0.0, -21.85, 21.1),
    ] {
        assert_eq!(
            prepared.classify_point(point, &options).unwrap(),
            PointClassification::Outside,
            "{point:?}"
        );
    }
}
