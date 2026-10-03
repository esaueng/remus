//! Coordinated boolean batch commits and retained session-state tests.

#![allow(clippy::unwrap_used)]
use crate::kernel::BrepKernel;

#[test]
fn coordinated_boolean_items_commit_independently_and_keep_session_state() {
    let mut k = BrepKernel::new();
    let a = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    let b = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    k.transform_solid_binding(
        b,
        vec![
            1.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
    )
    .unwrap();
    k.assembly_new("retained");
    let sketch = k.sketch_new();
    k.sketch_add_point(sketch, 1.0, 2.0, false).unwrap();
    k.gcs_new();
    let checkpoint = k.checkpoint();
    let input = k.serialize_solids(&[a, b]).unwrap();
    let session = format!(
        "{:?}{:?}{:?}{:?}{}",
        k.assemblies, k.sketches, k.gcs_sketches, k.checkpoints, k.poisoned
    );
    let mut retired = Vec::new();
    for _ in 0..3 {
        let rows: serde_json::Value = serde_json::from_str(&k.execute_batch(&format!(
            r#"[
            {{"op":"fuse","args":{{"solidA":{a},"solidB":{b}}}}},
            {{"op":"cut","args":{{"solidA":{a},"solidB":{a}}}}},
            {{"op":"fuse","args":{{"solidA":{a},"solidB":{b}}}}}
        ]"#
        )))
        .unwrap();
        assert!(rows[1]["error"].is_string());
        for row in [&rows[0], &rows[2]] {
            let id = u32::try_from(row["ok"].as_u64().unwrap()).unwrap();
            assert!((k.volume(id, 0.01).unwrap() - 1.5).abs() < 1e-8);
            retired.push(id);
        }
        assert_eq!(k.serialize_solids(&[a, b]).unwrap(), input);
        assert_eq!(
            format!(
                "{:?}{:?}{:?}{:?}{}",
                k.assemblies, k.sketches, k.gcs_sketches, k.checkpoints, k.poisoned
            ),
            session
        );
        k.restore(checkpoint).unwrap();
        k.make_box_solid(2.0, 2.0, 2.0).unwrap();
        for &id in &retired {
            assert!(k.resolve_solid(id).is_err());
        }
        k.restore(checkpoint).unwrap();
    }
    assert_eq!(k.serialize_solids(&[a, b]).unwrap(), input);
}

fn journaled_box(width: f64) -> (BrepKernel, u32, String) {
    let mut kernel = BrepKernel::new();
    let outer = kernel.make_box_solid(width, 3.0, 4.0).unwrap();
    let inner = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
    let result: serde_json::Value =
        serde_json::from_str(&kernel.fuse_journaled(outer, inner).unwrap()).unwrap();
    let solid = u32::try_from(result["solid"].as_u64().unwrap()).unwrap();
    let operation = u32::try_from(result["op"].as_u64().unwrap()).unwrap();
    let reference = kernel
        .make_operation_output_ref(operation, "face", 0)
        .unwrap();
    (kernel, solid, reference)
}

#[test]
fn arena_append_preserves_existing_operation_output_reference() {
    let (source, source_solid, _) = journaled_box(7.0);
    let (mut destination, original_solid, reference) = journaled_box(2.0);
    let original_faces = destination.get_solid_faces(original_solid).unwrap();
    let before = destination.resolve_ref(&reference).unwrap();
    let resolution: serde_json::Value = serde_json::from_str(&before).unwrap();
    assert_eq!(resolution["status"], "bound");

    let bytes = source.serialize_solids(&[source_solid]).unwrap();
    let imported = destination.deserialize_solids(&bytes).unwrap();
    assert_eq!(imported.len(), 1);
    assert_eq!(destination.resolve_ref(&reference).unwrap(), before);
    assert_eq!(
        destination.get_solid_faces(original_solid).unwrap(),
        original_faces
    );
    let imported_faces = destination.get_solid_faces(imported[0]).unwrap();
    assert!(
        original_faces
            .iter()
            .all(|face| !imported_faces.contains(face))
    );
    assert!((destination.volume(original_solid, 0.01).unwrap() - 24.0).abs() < 1e-8);
    assert!((destination.volume(imported[0], 0.01).unwrap() - 84.0).abs() < 1e-8);
}

#[test]
fn deleting_solid_keeps_shared_sheet_measurable_and_serializable() {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(2.0, 3.0, 4.0).unwrap();
    let face = kernel.get_solid_faces(solid).unwrap()[0];
    let sheet = kernel.make_sheet_body(vec![face]).unwrap();
    let before = kernel.serialize_sheets(&[sheet]).unwrap();
    let area = kernel.sheet_area(sheet, 0.01).unwrap();
    assert!((area - 6.0).abs() < 1e-10);

    kernel.delete_solid(solid).unwrap();
    assert!(kernel.resolve_solid(solid).is_err());
    assert_eq!(
        kernel.sheet_area(sheet, 0.01).unwrap().to_bits(),
        area.to_bits()
    );
    assert_eq!(kernel.serialize_sheets(&[sheet]).unwrap(), before);
    assert!(
        kernel
            .tessellate_sheet(sheet, 0.05, None)
            .unwrap()
            .triangle_count()
            > 0
    );
}
