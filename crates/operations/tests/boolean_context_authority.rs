//! Public boolean policy must reach every execution path and fail atomically.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_operations::OperationsError;
use remus_operations::boolean::{
    BooleanOp, BooleanOptions, BooleanQuality, boolean, boolean_with_context, boolean_with_options,
};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

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
fn custom_tolerance_reaches_the_operations_fast_path() {
    let gap = 5e-5;

    let mut default_topo = Topology::new();
    let default_a = make_box(&mut default_topo, 1.0, 1.0, 1.0).unwrap();
    let default_b = make_box(&mut default_topo, 1.0, 1.0, 1.0).unwrap();
    transform_solid(
        &mut default_topo,
        default_b,
        &Mat4::translation(1.0 + gap, 0.0, 0.0),
    )
    .unwrap();
    let exact_default = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let default_result = boolean_with_context(
        &mut default_topo,
        BooleanOp::Fuse,
        default_a,
        default_b,
        &exact_default,
    )
    .unwrap();
    let default_faces = remus_topology::explorer::solid_faces(&default_topo, default_result.solid)
        .unwrap()
        .len();

    let mut loose_topo = Topology::new();
    let loose_a = make_box(&mut loose_topo, 1.0, 1.0, 1.0).unwrap();
    let loose_b = make_box(&mut loose_topo, 1.0, 1.0, 1.0).unwrap();
    transform_solid(
        &mut loose_topo,
        loose_b,
        &Mat4::translation(1.0 + gap, 0.0, 0.0),
    )
    .unwrap();
    let exact_loose = OperationContext::new()
        .with_tolerance(Tolerance::loose())
        .with_fallback(FallbackPolicy::ExactOnly);
    let loose_result = boolean_with_context(
        &mut loose_topo,
        BooleanOp::Fuse,
        loose_a,
        loose_b,
        &exact_loose,
    )
    .unwrap();
    let loose_faces = remus_topology::explorer::solid_faces(&loose_topo, loose_result.solid)
        .unwrap()
        .len();

    assert_eq!(default_result.quality, BooleanQuality::Exact);
    assert_eq!(loose_result.quality, BooleanQuality::Exact);
    assert_eq!(
        default_faces, 12,
        "default tolerance keeps two disjoint boxes"
    );
    assert_eq!(
        loose_faces, 6,
        "loose tolerance joins the sub-tolerance gap"
    );
}

fn approximate_cylinder_fuse(deflection: f64) -> usize {
    let mut topo = Topology::new();
    let a = make_cylinder(&mut topo, 2.0, 3.0).unwrap();
    let b = make_cylinder(&mut topo, 2.0, 3.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(2.0, 0.0, 0.0)).unwrap();

    let options = BooleanOptions {
        deflection,
        unify_faces: false,
        ..BooleanOptions::default()
    };
    let context = options
        .operation_context()
        .with_fallback(FallbackPolicy::ApproximateOnly {
            budget: options.deflection,
        });
    let outcome = boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &context).unwrap();
    assert_eq!(
        outcome.quality,
        BooleanQuality::Approximate { deflection },
        "the option-derived fallback budget must be disclosed"
    );
    remus_topology::explorer::solid_faces(&topo, outcome.solid)
        .unwrap()
        .len()
}

#[test]
fn option_deflection_controls_the_actual_fallback_mesh() {
    let coarse_faces = approximate_cylinder_fuse(0.5);
    let fine_faces = approximate_cylinder_fuse(0.02);
    assert!(
        fine_faces > coarse_faces,
        "finer fallback deflection must produce more faces ({fine_faces} vs {coarse_faces})"
    );
}

fn l_fuse(unify_faces: bool) -> usize {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 3.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut topo, 1.0, 3.0, 1.0).unwrap();
    let result = boolean_with_options(
        &mut topo,
        BooleanOp::Fuse,
        a,
        b,
        BooleanOptions {
            unify_faces,
            ..BooleanOptions::default()
        },
    )
    .unwrap();
    remus_topology::explorer::solid_faces(&topo, result)
        .unwrap()
        .len()
}

#[test]
fn unification_option_changes_result_simplification() {
    let raw_faces = l_fuse(false);
    let unified_faces = l_fuse(true);
    assert!(
        unified_faces < raw_faces,
        "unification must reduce the L-fuse face count ({unified_faces} vs {raw_faces})"
    );
}

#[test]
fn failed_healing_rolls_back_the_whole_optioned_boolean() {
    let mut topo = Topology::new();
    let box_id = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let mut unhealed_topo = topo.clone();
    let before = live_counts(&topo);
    let before_slots = topo.allocated_slot_count();
    let opts = BooleanOptions {
        tolerance: Tolerance {
            linear: 3.0,
            angular: Tolerance::new().angular,
            relative: Tolerance::new().relative,
        },
        unify_faces: false,
        heal_after_boolean: true,
        ..BooleanOptions::default()
    };

    let unhealed = boolean_with_options(
        &mut unhealed_topo,
        BooleanOp::Fuse,
        box_id,
        box_id,
        BooleanOptions {
            heal_after_boolean: false,
            ..opts
        },
    );
    assert!(
        unhealed.is_ok(),
        "the control operation must succeed when healing is disabled: {unhealed:?}"
    );

    let err = boolean_with_options(&mut topo, BooleanOp::Fuse, box_id, box_id, opts).unwrap_err();
    assert!(
        matches!(err, OperationsError::HealingValidationFailed { .. }),
        "healing must run and reject its invalid result, got {err}"
    );
    assert_eq!(
        live_counts(&topo),
        before,
        "failed post-processing must roll back"
    );
    assert!(
        topo.solid(box_id).is_ok(),
        "pre-existing handles must survive rollback"
    );
    assert!(
        topo.allocated_slot_count() > before_slots,
        "the test must exercise mutations before rollback"
    );
}

fn overlapping_box_fuse(
    run: impl FnOnce(
        &mut Topology,
        remus_topology::solid::SolidId,
        remus_topology::solid::SolidId,
    ) -> remus_topology::solid::SolidId,
) -> (usize, f64) {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 2.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut topo, 2.0, 1.0, 1.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(1.0, 0.0, 0.0)).unwrap();
    let result = run(&mut topo, a, b);
    let faces = remus_topology::explorer::solid_faces(&topo, result)
        .unwrap()
        .len();
    (faces, solid_volume(&topo, result, 0.01).unwrap())
}

#[test]
fn default_context_and_options_preserve_legacy_boolean_behavior() {
    let legacy = overlapping_box_fuse(|topo, a, b| boolean(topo, BooleanOp::Fuse, a, b).unwrap());
    let contextual = overlapping_box_fuse(|topo, a, b| {
        let outcome =
            boolean_with_context(topo, BooleanOp::Fuse, a, b, &OperationContext::new()).unwrap();
        assert_eq!(outcome.quality, BooleanQuality::Exact);
        outcome.solid
    });
    let optioned = overlapping_box_fuse(|topo, a, b| {
        boolean_with_options(topo, BooleanOp::Fuse, a, b, BooleanOptions::default()).unwrap()
    });

    assert_eq!(legacy.0, contextual.0);
    assert_eq!(legacy.0, optioned.0);
    assert!((legacy.1 - contextual.1).abs() <= 1e-9);
    assert!((legacy.1 - optioned.1).abs() <= 1e-9);
}

#[test]
fn invalid_public_options_are_rejected_without_mutation() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let before = live_counts(&topo);
    let err = boolean_with_options(
        &mut topo,
        BooleanOp::Fuse,
        a,
        b,
        BooleanOptions {
            deflection: 0.0,
            ..BooleanOptions::default()
        },
    )
    .unwrap_err();
    assert!(matches!(err, OperationsError::InvalidInput { .. }));
    assert_eq!(live_counts(&topo), before);
}

#[test]
fn large_scale_exact_pipeline_preserves_material_and_operands() {
    // Exact straight-edge projection replaces the fixed-iteration search
    // that displaced large-model junctions and orphaned the tool walls.
    let scale = 1e6;
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, scale, scale, scale).unwrap();
    let tool = make_box(&mut topo, 0.4 * scale, 0.4 * scale, 2.0 * scale).unwrap();
    transform_solid(
        &mut topo,
        tool,
        &Mat4::translation(0.3 * scale, 0.3 * scale, -0.5 * scale),
    )
    .unwrap();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let result = boolean_with_context(&mut topo, BooleanOp::Cut, blank, tool, &context).unwrap();
    assert!(matches!(result.quality, BooleanQuality::Exact));
    for (solid, expected) in [(result.solid, 0.84), (blank, 1.0), (tool, 0.32)] {
        let volume = solid_volume(&topo, solid, 0.01 * scale).unwrap() / scale.powi(3);
        assert!((volume - expected).abs() < 1e-9);
        assert!(
            remus_operations::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
    }
}
