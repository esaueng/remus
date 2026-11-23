//! The lower hemisphere and upper cone share an equator at z=0.
#![allow(clippy::unwrap_used)]
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::Point3;

#[test]
fn hemisphere_attached_to_cone_is_not_a_pure_sphere() {
    let mut topo = remus_topology::Topology::new();
    let solids =
        remus_io::step::read_step(include_str!("data/hemisphere_cone.step"), &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    for (point, expected) in [
        (Point3::new(0., 0., 5.), PointClassification::Inside),
        (Point3::new(1., 0., 3.), PointClassification::OnBoundary),
        (Point3::new(1., 0., 5.), PointClassification::Outside),
    ] {
        assert!(remus_algo::classifier::classify_analytic(&topo, solids[0], point).is_none());
        assert_eq!(
            classify_point(&topo, solids[0], point, &ClassifyOptions::default()).unwrap(),
            expected
        );
    }
}
