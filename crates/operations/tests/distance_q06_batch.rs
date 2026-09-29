//! Q06 operations-layer batch: `point_to_solid_batch` matches a loop over the
//! one-shot path in deterministic input order, with reusable candidate buffers.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::vec::Point3;
use remus_operations::distance::{point_to_solid_batch, point_to_solid_distance};
use remus_topology::Topology;
use remus_topology::test_utils::make_unit_cube_manifold_at;

#[test]
fn operations_batch_matches_one_shot_in_order() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
    let pts = vec![
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(0.5, 0.5, 3.0),
        Point3::new(0.5, 0.5, 3.0),
        Point3::new(3.0, 0.5, 0.5),
    ];
    let batched = point_to_solid_batch(&topo, &pts, cube).unwrap();
    assert_eq!(batched.len(), pts.len());
    for (i, p) in pts.iter().enumerate() {
        let single = point_to_solid_distance(&topo, *p, cube).unwrap();
        assert!(
            (single.distance - batched[i].distance).abs() < 1e-12,
            "index {i}"
        );
        assert_eq!(single.point_b, batched[i].point_b);
    }
}

#[test]
fn operations_empty_batch_validates_and_returns_empty() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
    let out = point_to_solid_batch(&topo, &[], cube).unwrap();
    assert!(out.is_empty());
    let empty = Topology::new();
    assert!(point_to_solid_batch(&empty, &[], cube).is_err());
}
