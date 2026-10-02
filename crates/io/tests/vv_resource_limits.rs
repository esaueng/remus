//! Imported topology must not amplify a boolean's vertex-pair work unchecked.

#![allow(clippy::unwrap_used)]

use remus_algo::bop::BooleanOp;
use remus_math::context::{FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::mat::Mat4;
use remus_topology::Topology;

#[test]
fn imported_disjoint_spheres_with_overlapping_carrier_boxes() {
    let mut source = Topology::new();
    let a = remus_operations::primitives::make_sphere(&mut source, 1.0, 16).unwrap();
    let b = remus_operations::primitives::make_sphere(&mut source, 1.0, 16).unwrap();
    remus_operations::transform::transform_solid(&mut source, b, &Mat4::translation(1.5, 0.0, 1.5))
        .unwrap();
    let bytes = remus_io::arena_io::serialize_solids(&source, &[a, b]).unwrap();
    let mut topo = Topology::new();
    let roots = remus_io::arena_io::deserialize_solids(&bytes, &mut topo).unwrap();
    assert_eq!(roots.len(), 2);
    let before = remus_io::arena_io::serialize_solids(&topo, &roots).unwrap();
    let context = OperationContext::new().with_budgets(WorkBudgets::new().with_vertex_pairs(255));
    let result = remus_algo::gfa::boolean_with_context(
        &mut topo,
        BooleanOp::Intersect,
        roots[0],
        roots[1],
        &context,
    );
    assert!(matches!(
        result,
        Err(remus_algo::error::AlgoError::ResourceLimitExceeded {
            resource: "GFA vertex pairs",
            limit: 255,
            actual: 256
        })
    ));
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &roots).unwrap(),
        before
    );

    // Even the permissive legacy-quality context must preserve a resource
    // refusal rather than start another unbudgeted fallback computation.
    let context = context.with_fallback(FallbackPolicy::AllowApproximate { budget: 1e-3 });
    let error = remus_operations::boolean::boolean_with_context(
        &mut topo,
        remus_operations::boolean::BooleanOp::Intersect,
        roots[0],
        roots[1],
        &context,
    )
    .unwrap_err();
    assert!(matches!(
        error,
        remus_operations::OperationsError::Algo(
            remus_algo::error::AlgoError::ResourceLimitExceeded {
                resource: "GFA vertex pairs",
                limit: 255,
                actual: 256
            }
        )
    ));
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &roots).unwrap(),
        before
    );
}

#[test]
fn imported_exact_overlap_preserves_geometry_at_the_budget_boundary() {
    use remus_topology::test_utils::make_unit_cube_manifold_at;

    let mut source = Topology::new();
    let a = make_unit_cube_manifold_at(&mut source, 0.0, 0.0, 0.0);
    let b = make_unit_cube_manifold_at(&mut source, 0.5, 0.5, 0.5);
    let bytes = remus_io::arena_io::serialize_solids(&source, &[a, b]).unwrap();
    let mut topo = Topology::new();
    let roots = remus_io::arena_io::deserialize_solids(&bytes, &mut topo).unwrap();
    let context = OperationContext::new()
        .with_budgets(WorkBudgets::new().with_vertex_pairs(64))
        .with_fallback(FallbackPolicy::ExactOnly);
    let result = remus_operations::boolean::boolean_with_context(
        &mut topo,
        remus_operations::boolean::BooleanOp::Intersect,
        roots[0],
        roots[1],
        &context,
    )
    .unwrap();
    assert!(matches!(
        result.quality,
        remus_operations::boolean::BooleanQuality::Exact
    ));
    let result = result.solid;
    assert!(
        (remus_check::properties::solid_volume(
            &topo,
            result,
            &remus_check::properties::PropertiesOptions::default()
        )
        .unwrap()
            - 0.125)
            .abs()
            < 1e-10
    );
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(&topo, result).unwrap(),
        (6, 12, 8)
    );
}
