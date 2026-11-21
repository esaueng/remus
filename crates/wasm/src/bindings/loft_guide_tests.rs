//! `loftWithOptions` `"guide"`: a loft that follows a NURBS guide exactly,
//! through both batch routes, and its JSON refusals.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use crate::kernel::RemusKernel;

/// Two 2 × 2 squares at z = 0 and z = 10, corner (−1, −1) first.
fn squares(kernel: &mut RemusKernel) -> Vec<u32> {
    [0.0, 10.0]
        .into_iter()
        .map(|z| {
            let wire = kernel
                .make_polygon_wire(vec![-1.0, -1.0, z, 1.0, -1.0, z, 1.0, 1.0, z, -1.0, 1.0, z])
                .unwrap();
            kernel.make_planar_face_from_wire(wire).unwrap()
        })
        .collect()
}

/// The parabola `x = −1 + 6u(1 − u)`, `z = 10u` through both (−1, −1)
/// corners; every slice is the square translated, so the volume is 40.
fn parabola() -> Value {
    json!({
        "degree": 2,
        "knots": [0, 0, 0, 1, 1, 1],
        "controlPoints": [[-1, -1, 0], [2, -1, 5], [-1, -1, 10]],
    })
}

#[test]
fn guide_option_builds_the_exact_loft_through_both_batches() {
    for v2 in [false, true] {
        for ruled in [true, false] {
            let mut kernel = RemusKernel::new();
            let faces = squares(&mut kernel);
            let request = json!([{
                "op": "loftWithOptions",
                "args": { "faces": faces, "options": { "ruled": ruled, "guide": parabola() } }
            }])
            .to_string();
            let response = if v2 {
                kernel.execute_batch_v2(&request)
            } else {
                kernel.execute_batch(&request)
            };
            let cells: Vec<Value> = serde_json::from_str(&response).unwrap();
            let solid = u32::try_from(cells[0]["ok"].as_u64().expect(&response)).unwrap();
            assert_eq!(kernel.validate_solid(solid).unwrap(), 0);
            let id = kernel.resolve_solid(solid).unwrap();
            let mass = remus_operations::measure::mass_properties(kernel.topo(), id).unwrap();
            assert!((mass.mass - 40.0).abs() < 1e-6, "{}", mass.mass);
        }
    }
}

#[test]
fn guide_option_refusals_are_typed() {
    for (label, options, phrase) in [
        (
            "with connections",
            json!({ "guide": parabola(), "connections": [[-1, -1, 0], [-1, -1, 10]] }),
            "cannot be combined with loft connections",
        ),
        (
            "with an apex",
            json!({ "guide": parabola(), "endPoint": [0, 0, 20] }),
            "apex point",
        ),
        (
            "bad point",
            json!({ "guide": {
                "degree": 1, "knots": [0, 0, 1, 1],
                "controlPoints": [[-1, -1, 0], [-1, -1]],
            } }),
            "finite [x, y, z] point",
        ),
        (
            "bad knots",
            json!({ "guide": {
                "degree": 1, "knots": [0, 0, 1],
                "controlPoints": [[-1, -1, 0], [-1, -1, 10]],
            } }),
            "not a valid NURBS curve",
        ),
        (
            "misses a section",
            json!({ "guide": {
                "degree": 1, "knots": [0, 0, 1, 1],
                "controlPoints": [[-1, -1, 0], [0, 0, 10]],
            } }),
            "does not pass through a corner of section 1",
        ),
    ] {
        let mut kernel = RemusKernel::new();
        let faces = squares(&mut kernel);
        let face_ids = faces
            .iter()
            .map(|&face| kernel.resolve_face(face).unwrap())
            .collect();
        let error =
            super::operations::loft_with_options_impl(kernel.topo_mut(), face_ids, &options)
                .expect_err(label);
        let crate::error::WasmError::Operations(remus_operations::OperationsError::InvalidInput {
            ..
        }) = &error
        else {
            panic!("{label}: untyped {error:?}");
        };
        assert!(error.to_string().contains(phrase), "{label}: {error}");
    }
}
