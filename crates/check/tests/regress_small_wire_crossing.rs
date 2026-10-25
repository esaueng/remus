//! The wire validator must detect interior crossings at small model scales.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_check::validate::check_wire_self_intersection;
use remus_check::validate::checks::CheckId;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;

#[test]
fn small_nonorthogonal_bow_tie_is_rejected() {
    for scale in [1.0, 1e-8] {
        let mut topo = Topology::new();
        let tolerance = scale * 1e-10;
        let wire = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(scale, 0.5 * scale, 0.0),
                Point3::new(0.1 * scale, -0.4 * scale, 0.0),
                Point3::new(0.6 * scale, 0.6 * scale, 0.0),
            ],
            tolerance,
        )
        .unwrap();
        let issues = check_wire_self_intersection(&topo, wire, tolerance).unwrap();
        assert!(
            issues
                .iter()
                .any(|issue| issue.check == CheckId::WireSelfIntersection),
            "scale={scale}: {issues:?}"
        );
    }
}
