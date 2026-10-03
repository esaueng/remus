//! Public binding regressions for arena history and shared sheet retirement.

#![allow(clippy::unwrap_used)]

use crate::kernel::BrepKernel;

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
