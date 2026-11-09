//! OpenZCAD #1253 through the direct and both batch loft bindings.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use crate::kernel::RemusKernel;

const EXACT_VOLUME: f64 = 61_261.056_745_000_96;

fn sections(kernel: &mut RemusKernel) -> Vec<u32> {
    [(0.0, 0.0, 25.0), (12.0, 60.0, 10.0)]
        .into_iter()
        .map(|(x, z, radius)| {
            let edge = kernel
                .make_circle_edge(x, 0.0, z, 0.0, 0.0, 1.0, radius)
                .unwrap();
            let wire = kernel.make_wire(vec![edge], true).unwrap();
            kernel.make_planar_face_from_wire(wire).unwrap()
        })
        .collect()
}

fn assert_exact_result(kernel: &RemusKernel, solid: u32) {
    assert_eq!(kernel.validate_solid(solid).unwrap(), 0);
    let faces = kernel.get_solid_faces(solid).unwrap();
    assert_eq!(
        faces.len(),
        3,
        "two caps and one curved side, not 32 facets"
    );
    let surfaces: Vec<_> = faces
        .iter()
        .map(|&face| kernel.get_surface_type(face).unwrap())
        .collect();
    assert_eq!(surfaces.iter().filter(|&kind| kind == "plane").count(), 2);
    assert_eq!(surfaces.iter().filter(|&kind| kind == "bspline").count(), 1);
    let id = kernel.resolve_solid(solid).unwrap();
    let mass = remus_operations::measure::mass_properties(kernel.topo(), id).unwrap();
    assert!((mass.mass - EXACT_VOLUME).abs() < EXACT_VOLUME * 1e-9);
    // This API intentionally measures the display tessellation for NURBS
    // solids. Its chord error is distinct from faceting the exact geometry.
    let mesh_volume = kernel.volume(solid, 0.005).unwrap();
    assert!((mesh_volume - EXACT_VOLUME).abs() < EXACT_VOLUME * 1e-3);
}

#[test]
fn direct_and_both_batches_preserve_offset_circular_sections() {
    for smooth in [false, true] {
        for route in 0..3 {
            let mut kernel = RemusKernel::new();
            let faces = sections(&mut kernel);
            let solid = if route == 0 {
                if smooth {
                    kernel.loft_smooth_faces(faces).unwrap()
                } else {
                    kernel.loft_faces(faces).unwrap()
                }
            } else {
                let request = json!([{
                    "op": if smooth { "loftSmooth" } else { "loft" },
                    "args": { "faces": faces }
                }])
                .to_string();
                let response = if route == 1 {
                    kernel.execute_batch(&request)
                } else {
                    kernel.execute_batch_v2(&request)
                };
                let cells: Vec<Value> = serde_json::from_str(&response).unwrap();
                u32::try_from(cells[0]["ok"].as_u64().expect(&response)).unwrap()
            };
            assert_exact_result(&kernel, solid);
        }
    }
}
