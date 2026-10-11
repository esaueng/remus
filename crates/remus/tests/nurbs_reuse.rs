//! Native facade policy propagation for certified NURBS carrier operations.
#![allow(clippy::unwrap_used)]

use remus::prelude::*;

#[test]
fn facade_uses_session_approximation_and_cancellation_policy() {
    let curve = NurbsCurve::new(
        1,
        vec![2., 2., 5., 5.],
        vec![Point3::new(0., 0., 0.), Point3::new(3., 0., 0.)],
        vec![1., 1.],
    )
    .unwrap();
    let options = CubicFitOptions::new(1e-7, 1e-7);
    let exact =
        Model::with_context(OperationContext::new().with_fallback(FallbackPolicy::ExactOnly));
    assert!(matches!(
        exact.fit_cubic_curve(&curve, &options),
        Err(ReuseError::Unsupported { .. })
    ));
    let model = Model::with_context(
        OperationContext::new().with_fallback(FallbackPolicy::AllowApproximate { budget: 1e-7 }),
    );
    let fitted = model.fit_cubic_curve(&curve, &options).unwrap();
    assert!(fitted.approximate);
    assert_eq!(fitted.curve.domain(), curve.domain());
    assert!(fitted.position_bound <= 1e-7);
    let token = CancellationToken::new();
    token.cancel();
    let cancelled = Model::with_context(OperationContext::new().with_cancellation(token));
    assert!(matches!(
        cancelled.simplify_nurbs_curve(&curve, &ReductionOptions::default()),
        Err(ReuseError::Math(MathError::Cancelled))
    ));
    assert_eq!(model.topology().num_edges(), 0);
}
