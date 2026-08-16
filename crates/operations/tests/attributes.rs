//! Attribute store, propagation rules, and lifecycle (Issue 14).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean_with_evolution};
use brepkit_operations::copy::copy_solid_with_face_map;
use brepkit_operations::evolution::propagate_face_attributes;
use brepkit_operations::primitives::make_box;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::attributes::{ColorRgb, EntityAttributes};
use brepkit_topology::explorer::solid_faces;

fn named(name: &str, color: Option<ColorRgb>) -> EntityAttributes {
    EntityAttributes {
        name: Some(name.to_string()),
        color,
        ..Default::default()
    }
}

#[test]
fn copy_carries_solid_and_face_attributes() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    let red = ColorRgb::new(1.0, 0.0, 0.0).unwrap();
    topo.attributes_mut()
        .set_solid(solid, named("plate", Some(red)));
    topo.attributes_mut()
        .set_face(face, named("top", Some(red)));

    let (copied, face_map) = copy_solid_with_face_map(&mut topo, solid).unwrap();
    let copied_face = topo.face_id_from_index(face_map[&face.index()]).unwrap();

    assert_eq!(
        topo.attributes().solid(copied).unwrap().name.as_deref(),
        Some("plate")
    );
    assert_eq!(
        topo.attributes().face(copied_face).unwrap().name.as_deref(),
        Some("top")
    );
    // Source attributes are untouched.
    assert!(topo.attributes().solid(solid).is_some());
    assert!(topo.attributes().face(face).is_some());
}

#[test]
fn boolean_evolution_propagates_face_attributes_by_rule() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let b = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();

    // Attribute every input face of A.
    let blue = ColorRgb::new(0.0, 0.0, 1.0).unwrap();
    for (i, face) in solid_faces(&topo, a).unwrap().into_iter().enumerate() {
        topo.attributes_mut()
            .set_face(face, named(&format!("a{i}"), Some(blue)));
    }

    let (result, map) = boolean_with_evolution(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let report = propagate_face_attributes(&mut topo, &map);
    assert!(report.carried > 0, "modified faces must carry attributes");

    // Every attributed result face got its attributes from a modified
    // input; generated/unresolved result faces carry none.
    let result_faces = solid_faces(&topo, result).unwrap();
    let attributed = result_faces
        .iter()
        .filter(|f| topo.attributes().face(**f).is_some())
        .count();
    assert_eq!(attributed, report.carried);
    assert!(
        attributed < result_faces.len(),
        "faces from operand B and generated faces stay unattributed"
    );

    // Wherever an input face maps to multiple pieces, the name copies
    // unchanged to every piece: same semantic surface, no synthesized
    // suffixes. (Whether this fuse's map records 1→N or per-piece 1→1
    // entries is engine granularity; the rule holds either way.)
    for outputs in map.modified.values() {
        let names: Vec<_> = outputs
            .iter()
            .filter_map(|idx| topo.face_id_from_index(*idx))
            .filter_map(|f| topo.attributes().face(f))
            .map(|attrs| attrs.name.clone())
            .collect();
        assert!(
            names.windows(2).all(|w| w[0] == w[1]),
            "all pieces of one input share the unchanged name"
        );
    }
}

#[test]
fn delete_solid_removes_attribute_entries() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    topo.attributes_mut().set_solid(solid, named("gone", None));
    topo.attributes_mut()
        .set_face(face, named("gone-face", None));

    topo.delete_solid(solid).unwrap();
    assert_eq!(
        topo.attributes().len(),
        0,
        "no attribute may outlive its entity"
    );
}

#[test]
fn restore_rolls_the_store_back_with_the_topology() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let snapshot = topo.clone();

    topo.attributes_mut().set_solid(solid, named("late", None));
    topo.restore_preserving_handle_slots(&snapshot);
    assert!(topo.attributes().solid(solid).is_none());
}

#[test]
fn color_channels_are_validated_with_a_typed_error() {
    use brepkit_math::diagnostic::{FailureCategory, ToDiagnostic};
    let err = ColorRgb::new(1.5, 0.0, 0.0).unwrap_err();
    let d = err.diagnostic();
    assert_eq!(d.category(), FailureCategory::InvalidInput);
    assert_eq!(d.code(), "invalid_color_channel");
    assert!(ColorRgb::new(0.0, f64::NAN, 0.0).is_err());
}
