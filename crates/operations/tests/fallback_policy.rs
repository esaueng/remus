//! The exact boolean fallback contract (Issue 11).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn overlapping_boxes(topo: &mut Topology) -> (remus_topology::SolidId, remus_topology::SolidId) {
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

/// A boss whose wall is a hair off tangent to the block's side (B21 fixture,
/// shared with the WASM batch contract): the exact pipeline cannot assemble
/// it, so it is the one configuration that exercises the mesh fallback
/// without corrupting an operand.
fn tangent_boss(topo: &mut Topology) -> (remus_topology::SolidId, remus_topology::SolidId) {
    use remus_operations::primitives::make_cylinder;
    let block = make_box(topo, 60.0, 40.0, 8.0).unwrap();
    let boss = make_cylinder(topo, 10.0, 16.0).unwrap();
    transform_solid(topo, boss, &Mat4::translation(9.999, 20.0, 0.0)).unwrap();
    (block, boss)
}

fn live_counts(topo: &Topology) -> (usize, usize, usize, usize, usize, usize) {
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    )
}

#[test]
fn tangent_boss_fixture_needs_the_fallback() {
    // Guard the fixture itself: if the exact pipeline learns this case, the
    // refusal tests below stop proving anything and must move fixtures.
    let mut topo = Topology::new();
    let (a, b) = tangent_boss(&mut topo);
    let out =
        boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &OperationContext::new()).unwrap();
    assert_eq!(
        out.quality,
        BooleanQuality::Approximate { deflection: 0.1 },
        "fixture must still require the mesh fallback"
    );
}

#[test]
fn plain_boolean_refuses_instead_of_silently_approximating() {
    use remus_operations::OperationsError;
    let mut topo = Topology::new();
    let (a, b) = tangent_boss(&mut topo);
    let before = live_counts(&topo);
    let err = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap_err();
    assert!(
        matches!(err, OperationsError::ExactOnlyUnattainable),
        "a bare handle cannot disclose a mesh result; expected the typed refusal, got {err}"
    );
    assert_eq!(
        live_counts(&topo),
        before,
        "a refusal must not mutate the arena"
    );
    let v = solid_volume(&topo, a, 0.05).unwrap();
    assert!(
        (v - 19_200.0).abs() < 1e-6,
        "operand must be untouched: {v}"
    );
}

#[test]
fn plain_options_boolean_refuses_like_the_handle_entry_point() {
    use remus_operations::OperationsError;
    use remus_operations::boolean::{BooleanOptions, boolean_with_options};
    let mut topo = Topology::new();
    let (a, b) = tangent_boss(&mut topo);
    let before = live_counts(&topo);
    let err = boolean_with_options(
        &mut topo,
        BooleanOp::Fuse,
        a,
        b,
        BooleanOptions {
            deflection: 0.02,
            ..BooleanOptions::default()
        },
    )
    .unwrap_err();
    assert!(
        matches!(err, OperationsError::ExactOnlyUnattainable),
        "deflection is not consent to approximate; got {err}"
    );
    assert_eq!(live_counts(&topo), before);
}

#[test]
fn plain_evolution_boolean_refuses_like_the_handle_entry_point() {
    use remus_operations::OperationsError;
    use remus_operations::boolean::boolean_with_evolution;
    let mut topo = Topology::new();
    let (a, b) = tangent_boss(&mut topo);
    let before = live_counts(&topo);
    let err = boolean_with_evolution(&mut topo, BooleanOp::Fuse, a, b).unwrap_err();
    assert!(
        matches!(err, OperationsError::ExactOnlyUnattainable),
        "got {err}"
    );
    assert_eq!(live_counts(&topo), before);
}

#[test]
fn optioned_outcome_discloses_the_fallback_it_was_allowed_to_take() {
    use remus_operations::boolean::{BooleanOptions, boolean_outcome_with_options};
    let mut topo = Topology::new();
    let (a, b) = tangent_boss(&mut topo);
    let opts = BooleanOptions {
        unify_faces: true,
        ..BooleanOptions::default()
    };
    let ctx =
        OperationContext::new().with_fallback(FallbackPolicy::AllowApproximate { budget: 0.05 });
    let out = boolean_outcome_with_options(&mut topo, BooleanOp::Fuse, a, b, &opts, &ctx).unwrap();
    assert_eq!(
        out.quality,
        BooleanQuality::Approximate { deflection: 0.05 },
        "the permissive path must disclose the fallback and its budget"
    );
    // Block plus the boss volume outside it: the boss stands on the block's
    // top face and its far half overhangs nothing, so it adds its full height
    // above z = 8 (16 - 8 = 8 mm) times the disc area, within mesh accuracy.
    let v = solid_volume(&topo, out.solid, 0.05).unwrap();
    let expected = 19_200.0 + std::f64::consts::PI * 100.0 * 8.0;
    assert!(
        (v - expected).abs() / expected < 0.02,
        "fused volume {v} vs {expected}"
    );

    // The same options under exact-only are refused, with nothing retained.
    let mut topo = Topology::new();
    let (a, b) = tangent_boss(&mut topo);
    let before = live_counts(&topo);
    let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let err =
        boolean_outcome_with_options(&mut topo, BooleanOp::Fuse, a, b, &opts, &exact).unwrap_err();
    assert!(
        matches!(
            err,
            remus_operations::OperationsError::ExactOnlyUnattainable
        ),
        "got {err}"
    );
    assert_eq!(live_counts(&topo), before);
}

#[test]
fn fuse_solids_refuses_like_the_handle_entry_point() {
    use remus_operations::OperationsError;
    use remus_operations::compound_ops::fuse_solids;
    let mut topo = Topology::new();
    let (a, b) = tangent_boss(&mut topo);
    let before = live_counts(&topo);
    let err = fuse_solids(&mut topo, &[a, b]).unwrap_err();
    assert!(
        matches!(err, OperationsError::ExactOnlyUnattainable),
        "fuseAll returns a bare handle too; got {err}"
    );
    assert_eq!(live_counts(&topo), before);
}
