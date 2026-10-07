//! Checkpoint sharing must preserve evaluation and the existing wire format.
#![allow(clippy::unwrap_used)]
use remus_math::nurbs::{curve::NurbsCurve, surface::NurbsSurface};
use remus_math::vec::Point3;

#[test]
fn immutable_clones_share_storage_and_evaluate_identically() {
    let curve = NurbsCurve::new(
        1,
        vec![0., 0., 1., 1.],
        vec![Point3::new(0., 0., 0.), Point3::new(3., 2., 1.)],
        vec![1., 2.],
    )
    .unwrap();
    let copy = curve.clone();
    assert!(std::ptr::eq(
        curve.control_points().as_ptr(),
        copy.control_points().as_ptr()
    ));
    assert!(std::ptr::eq(
        curve.weights().as_ptr(),
        copy.weights().as_ptr()
    ));
    for i in 0..100 {
        assert_eq!(
            curve.evaluate(f64::from(i) / 99.),
            copy.evaluate(f64::from(i) / 99.)
        );
    }
    let surface = surface();
    let copy = surface.clone();
    assert!(std::ptr::eq(
        surface.control_points().as_ptr(),
        copy.control_points().as_ptr()
    ));
    assert!(std::ptr::eq(
        surface.weights().as_ptr(),
        copy.weights().as_ptr()
    ));
    for i in 0..100 {
        assert_eq!(
            surface.evaluate(f64::from(i) / 99., 0.3),
            copy.evaluate(f64::from(i) / 99., 0.3)
        );
    }
}

fn surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0., 0., 1., 1.],
        vec![0., 0., 1., 1.],
        vec![
            vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 0.)],
            vec![Point3::new(1., 0., 0.), Point3::new(1., 1., 1.)],
        ],
        vec![vec![1., 2.], vec![1., 1.]],
    )
    .unwrap()
}

#[cfg(feature = "serde")]
#[test]
fn sharing_preserves_plain_array_serialization() {
    let source = surface();
    let serialized = serde_json::to_value(&source).unwrap();
    assert_eq!(
        serialized,
        serde_json::json!({
            "degree_u": 1, "degree_v": 1, "knots_u": source.knots_u(), "knots_v": source.knots_v(),
            "control_points": source.control_points(), "weights": source.weights()
        })
    );
    let restored: NurbsSurface = serde_json::from_value(serialized).unwrap();
    assert_eq!(source, restored);
    assert_eq!(source.evaluate(0.7, 0.3), restored.evaluate(0.7, 0.3));
}
