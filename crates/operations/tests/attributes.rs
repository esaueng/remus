//! Attribute store, propagation rules, and lifecycle (Issue 14).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean_with_evolution};
use brepkit_operations::copy::{copy_and_transform_solid, copy_solid_with_face_map};
use brepkit_operations::evolution::{EvolutionMap, propagate_face_attributes};
use brepkit_operations::primitives::make_box;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::attributes::{ColorRgb, EntityAttributes};
use brepkit_topology::explorer::solid_faces;

fn named(name: &str, color: Option<ColorRgb>) -> EntityAttributes {
    EntityAttributes {
        name: Some(name.to_string()),
        color,
    }
}

#[test]
fn copy_carries_solid_and_face_attributes() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    let red = ColorRgb::new(1.0, 0.0, 0.0).unwrap();
    topo.set_solid_attributes(solid, named("plate", Some(red)))
        .unwrap();
    topo.set_face_attributes(face, named("top", Some(red)))
        .unwrap();

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
fn copy_and_transform_carries_solid_and_face_attributes() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    topo.set_solid_attributes(solid, named("translated", None))
        .unwrap();
    topo.set_face_attributes(face, named("datum", None))
        .unwrap();

    let copied =
        copy_and_transform_solid(&mut topo, solid, &Mat4::translation(10.0, 20.0, 30.0)).unwrap();

    assert_eq!(
        topo.attributes().solid(copied).unwrap().name.as_deref(),
        Some("translated")
    );
    let copied_names: Vec<_> = solid_faces(&topo, copied)
        .unwrap()
        .into_iter()
        .filter_map(|copied_face| topo.attributes().face(copied_face))
        .filter_map(|attributes| attributes.name.as_deref())
        .collect();
    assert_eq!(copied_names, vec!["datum"]);
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
        topo.set_face_attributes(face, named(&format!("a{i}"), Some(blue)))
            .unwrap();
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
fn propagation_merges_only_fields_on_which_every_source_agrees() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let result = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let a_face = solid_faces(&topo, a).unwrap()[0];
    let b_face = solid_faces(&topo, b).unwrap()[0];
    let result_face = solid_faces(&topo, result).unwrap()[0];
    let blue = ColorRgb::new(0.0, 0.0, 1.0).unwrap();
    topo.set_face_attributes(a_face, named("left", Some(blue)))
        .unwrap();
    topo.set_face_attributes(b_face, named("right", Some(blue)))
        .unwrap();

    let mut map = EvolutionMap::exact();
    map.add_modified(a_face.index(), result_face.index());
    map.add_modified(b_face.index(), result_face.index());
    let report = propagate_face_attributes(&mut topo, &map);

    assert_eq!(report.carried, 1);
    let attributes = topo.attributes().face(result_face).unwrap();
    assert_eq!(attributes.name, None);
    assert_eq!(attributes.color, Some(blue));
}

#[test]
fn generated_and_unresolved_outputs_are_explicitly_unattributed() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let generated_result = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let unresolved_result = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let source_face = solid_faces(&topo, source).unwrap()[0];
    let generated_face = solid_faces(&topo, generated_result).unwrap()[0];
    let unresolved_face = solid_faces(&topo, unresolved_result).unwrap()[0];
    topo.set_face_attributes(generated_face, named("must clear", None))
        .unwrap();
    topo.set_face_attributes(unresolved_face, named("must also clear", None))
        .unwrap();

    let mut map = EvolutionMap::exact();
    map.add_generated(source_face.index(), generated_face.index());
    map.add_unresolved(unresolved_face.index(), vec![source_face.index()]);
    let report = propagate_face_attributes(&mut topo, &map);

    assert_eq!(report.carried, 0);
    assert_eq!(report.unresolved_outputs, 1);
    assert!(topo.attributes().face(generated_face).is_none());
    assert!(topo.attributes().face(unresolved_face).is_none());
}

#[test]
fn delete_solid_removes_attribute_entries() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    topo.set_solid_attributes(solid, named("gone", None))
        .unwrap();
    topo.set_face_attributes(face, named("gone-face", None))
        .unwrap();

    topo.delete_solid(solid).unwrap();
    assert_eq!(
        topo.attributes().len(),
        0,
        "no attribute may outlive its entity"
    );
    assert!(
        topo.set_face_attributes(face, named("stale", None))
            .is_err()
    );
    assert!(
        topo.set_solid_attributes(solid, named("stale", None))
            .is_err()
    );
    assert!(topo.attributes().is_empty());
}

#[test]
fn restore_rolls_the_store_back_with_the_topology() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let snapshot = topo.clone();

    topo.set_solid_attributes(solid, named("late", None))
        .unwrap();
    topo.restore_preserving_handle_slots(&snapshot);
    assert!(topo.attributes().solid(solid).is_none());
}

#[test]
fn restore_drops_attributes_for_entities_retired_after_the_snapshot() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    topo.set_solid_attributes(solid, named("retired", None))
        .unwrap();
    topo.set_face_attributes(face, named("retired-face", None))
        .unwrap();
    let snapshot = topo.clone();

    topo.delete_solid(solid).unwrap();
    topo.restore_preserving_handle_slots(&snapshot);

    assert!(topo.solid(solid).is_err());
    assert!(topo.face(face).is_err());
    assert!(topo.attributes().solid(solid).is_none());
    assert!(topo.attributes().face(face).is_none());
}

#[test]
fn color_channels_are_validated_with_a_typed_error() {
    use brepkit_math::diagnostic::{FailureCategory, ToDiagnostic};
    let err = ColorRgb::new(1.5, 0.0, 0.0).unwrap_err();
    let d = err.diagnostic();
    assert_eq!(d.category(), FailureCategory::InvalidInput);
    assert_eq!(d.code(), "invalid_color_channel");
    assert!(ColorRgb::new(0.0, f64::NAN, 0.0).is_err());
    let valid = ColorRgb::new(0.25, 0.5, 0.75).unwrap();
    assert_eq!((valid.r(), valid.g(), valid.b()), (0.25, 0.5, 0.75));
}
