//! The exact boolean fallback contract (Issue 11).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use brepkit_math::context::{FallbackPolicy, OperationContext};
use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, BooleanQuality, boolean, boolean_with_context};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::make_box;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;

fn overlapping_boxes(
    topo: &mut Topology,
) -> (brepkit_topology::SolidId, brepkit_topology::SolidId) {
    let a = make_box(topo, 2.0, 2.0, 2.0).unwrap();
    let b = make_box(topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(topo, b, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
    (a, b)
}

#[test]
fn default_context_matches_legacy_and_reports_exact_on_clean_input() {
    let mut topo_a = Topology::new();
    let (a1, a2) = overlapping_boxes(&mut topo_a);
    let legacy = boolean(&mut topo_a, BooleanOp::Fuse, a1, a2).unwrap();
    let v_legacy = solid_volume(&topo_a, legacy, 0.05).unwrap();

    let mut topo_b = Topology::new();
    let (b1, b2) = overlapping_boxes(&mut topo_b);
    let out = boolean_with_context(
        &mut topo_b,
        BooleanOp::Fuse,
        b1,
        b2,
        &OperationContext::new(),
    )
    .unwrap();
    assert_eq!(out.quality, BooleanQuality::Exact);
    let v_ctx = solid_volume(&topo_b, out.solid, 0.05).unwrap();
    assert!((v_legacy - v_ctx).abs() < 1e-9);
    assert!((v_ctx - 15.0).abs() < 1e-6);
}

#[test]
fn exact_only_succeeds_where_the_exact_pipeline_does() {
    let mut topo = Topology::new();
    let (a, b) = overlapping_boxes(&mut topo);
    let ctx = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let out = boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &ctx).unwrap();
    assert_eq!(out.quality, BooleanQuality::Exact);
}

#[test]
fn approximate_only_skips_the_exact_pipeline_and_discloses_quality() {
    let mut topo = Topology::new();
    let (a, b) = overlapping_boxes(&mut topo);
    let ctx =
        OperationContext::new().with_fallback(FallbackPolicy::ApproximateOnly { budget: 0.05 });
    let out = boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &ctx).unwrap();
    assert_eq!(
        out.quality,
        BooleanQuality::Approximate { deflection: 0.05 },
        "the approximate path must disclose itself"
    );
    // Planar co-refinement is geometrically faithful on boxes.
    let v = solid_volume(&topo, out.solid, 0.05).unwrap();
    assert!((v - 15.0).abs() < 1e-3, "mesh fuse volume {v}");
}
