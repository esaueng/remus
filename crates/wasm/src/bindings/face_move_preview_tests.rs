//! Source-relative live face moves preserve accepted geometry on refusal.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::vec::Vec3;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_topology::explorer::solid_faces;
use serde_json::{Value, json};

use crate::handles::face_id_to_u32;
use crate::kernel::RemusKernel;

fn fixture(kernel: &mut RemusKernel) -> (u32, u32) {
    let solid = kernel.make_box_solid(10.0, 6.0, 4.0).unwrap();
    let id = kernel.resolve_solid(solid).unwrap();
    let face = solid_faces(kernel.topo(), id)
        .unwrap()
        .into_iter()
        .find(|&face| {
            kernel
                .topo()
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 0.999)
        })
        .unwrap();
    (solid, face_id_to_u32(face))
}

fn direct(kernel: &mut RemusKernel, solid: u32, face: u32, distance: f64) -> Value {
    serde_json::to_value(kernel.move_faces_detailed_impl(solid, &[face], distance)).unwrap()
}

fn batch(kernel: &mut RemusKernel, solid: u32, face: u32, distance: f64) -> Value {
    let input = json!([{"op": "moveFacesDetailed", "args": {
        "solid": solid, "faces": [face], "distance": distance
    }}]);
    let output: Value = serde_json::from_str(&kernel.execute_batch(&input.to_string())).unwrap();
    output[0]["ok"].clone()
}

fn result_solid(result: &Value) -> u32 {
    assert_eq!(result["status"], "ok", "{result}");
    assert_eq!(result["details"]["quality"], "exact");
    u32::try_from(result["value"].as_u64().unwrap()).unwrap()
}

#[test]
fn refused_drag_preserves_source_and_last_valid_mesh_then_recovers() {
    for preview in [direct, batch] {
        let mut kernel = RemusKernel::new();
        let (source, face) = fixture(&mut kernel);
        let accepted = result_solid(&preview(&mut kernel, source, face, 2.0));
        assert!((kernel.volume(accepted, 0.05).unwrap() - 360.0).abs() < 1e-7);
        let before = kernel.serialize_solids(&[source, accepted]).unwrap();
        let id = kernel.resolve_solid(accepted).unwrap();
        let mesh = tessellate_solid_with_tolerance(kernel.topo(), id, 0.05, 0.1).unwrap();

        let refused = preview(&mut kernel, source, face, -20.0);
        assert_eq!(refused["status"], "error", "{refused}");
        assert!(refused["value"].is_null());
        assert!(refused["code"].is_string());
        assert!(refused["category"].is_string());
        assert!(refused["details"]["message"].is_string());
        assert_eq!(
            kernel.serialize_solids(&[source, accepted]).unwrap(),
            before
        );
        let retained = tessellate_solid_with_tolerance(kernel.topo(), id, 0.05, 0.1).unwrap();
        assert_eq!(retained.positions, mesh.positions);
        assert_eq!(retained.normals, mesh.normals);
        assert_eq!(retained.indices, mesh.indices);

        let recovered = result_solid(&preview(&mut kernel, source, face, 1.0));
        // Each preview starts from the original 4 mm height, not the 6 mm
        // accepted preview. Returning to a valid distance needs no new source.
        assert!((kernel.volume(recovered, 0.05).unwrap() - 300.0).abs() < 1e-7);
        assert!((kernel.volume(source, 0.05).unwrap() - 240.0).abs() < 1e-7);
        assert!((kernel.volume(accepted, 0.05).unwrap() - 360.0).abs() < 1e-7);
    }
}

#[test]
fn direct_and_batch_face_move_envelopes_match_on_success_and_refusal() {
    let mut direct_kernel = RemusKernel::new();
    let (source, face) = fixture(&mut direct_kernel);
    let mut batch_kernel = RemusKernel::new();
    let (batch_source, batch_face) = fixture(&mut batch_kernel);
    for distance in [2.0, -20.0, -1.0, 0.0] {
        assert_eq!(
            direct(&mut direct_kernel, source, face, distance),
            batch(&mut batch_kernel, batch_source, batch_face, distance)
        );
    }
}

#[test]
fn invalid_face_move_inputs_are_data_and_do_not_change_the_source() {
    let mut kernel = RemusKernel::new();
    let (source, face) = fixture(&mut kernel);
    let before = kernel.serialize_solids(&[source]).unwrap();
    for (solid, faces, distance) in [
        (u32::MAX, vec![face], 1.0),
        (source, vec![u32::MAX], 1.0),
        (source, vec![], 1.0),
        (source, vec![face], f64::NAN),
        (source, vec![face], f64::INFINITY),
    ] {
        let result =
            serde_json::to_value(kernel.move_faces_detailed_impl(solid, &faces, distance)).unwrap();
        assert_eq!(result["status"], "error", "{result}");
        assert!(result["value"].is_null());
        assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);
    }
    let input = json!([{"op": "moveFacesDetailed", "args": {
        "solid": source, "faces": "invalid", "distance": 1.0
    }}]);
    let output: Value = serde_json::from_str(&kernel.execute_batch(&input.to_string())).unwrap();
    assert_eq!(output[0]["ok"]["status"], "error");
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);
}
