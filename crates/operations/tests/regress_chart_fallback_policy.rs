//! An uncertified exact candidate still obeys the caller's fallback policy.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn fixture() -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let a = make_cylinder(&mut topo, 3.0, 20.0).unwrap();
    transform_solid(&mut topo, a, &Mat4::translation(0.0, 0.0, -10.0)).unwrap();
    let b = make_cylinder(&mut topo, 3.0, 2.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::rotation_y(std::f64::consts::FRAC_PI_2)).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(-1.0, 0.0, 0.0)).unwrap();
    (topo, a, b)
}

fn counts(t: &Topology) -> [usize; 6] {
    [
        t.num_vertices(),
        t.num_edges(),
        t.num_wires(),
        t.num_faces(),
        t.num_shells(),
        t.num_solids(),
    ]
}

#[test]
fn failed_chart_refuses_exact_and_preserves_explicit_mesh_policy() {
    let (input, a, b) = fixture();
    let mut exact = input.clone();
    let refused = boolean_with_context(
        &mut exact,
        BooleanOp::Fuse,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    );
    assert!(matches!(
        refused,
        Err(OperationsError::ExactOnlyUnattainable)
    ));
    // Rollback preserves all live entities and operand contents. High-water
    // slots allocated by the rejected attempt deliberately remain retired.
    assert_eq!(counts(&exact), counts(&input));
    for solid in [a, b] {
        assert_eq!(
            format!("{:?}", exact.solid(solid).unwrap()),
            format!("{:?}", input.solid(solid).unwrap())
        );
    }
    let budget = 0.05;
    let mut permitted = input.clone();
    let outcome = boolean_with_context(
        &mut permitted,
        BooleanOp::Fuse,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::AllowApproximate { budget }),
    )
    .expect("the declared mesh fallback must remain available");
    assert_eq!(
        outcome.quality,
        BooleanQuality::Approximate { deflection: budget }
    );
    let mut mesh_only = input;
    let reference = boolean_with_context(
        &mut mesh_only,
        BooleanOp::Fuse,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ApproximateOnly { budget }),
    )
    .unwrap();
    assert_eq!(reference.quality, outcome.quality);
    assert_eq!(
        counts(&permitted),
        counts(&mesh_only),
        "no rejected exact candidate remains live"
    );
    let got = solid_volume(&permitted, outcome.solid, budget).unwrap();
    let expected = solid_volume(&mesh_only, reference.solid, budget).unwrap();
    assert!(
        (got - expected).abs() < 1e-8,
        "same declared mesh route: {got} vs {expected}"
    );
}
