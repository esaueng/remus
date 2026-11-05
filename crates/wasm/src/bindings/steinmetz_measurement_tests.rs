//! Direct (`intersectDetailed` / `cutDetailed` / `fuseDetailed`) and both
//! batch WASM qualification of the measured volume of equal-radius
//! perpendicular cylinders. Tier: core — the booleans write the live session.
//!
//! The Steinmetz bicylinder r = 10 read 5333.2664 against 16r³/3 (1.25e-5
//! low) although its B-rep is exact: the cylinder faces' ellipse trims were
//! measured through a sampled polygon. The native root cause and oracle are
//! in `crates/operations/tests/regress_steinmetz_exact_measurement.rs`.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use serde_json::{Value, json};

use crate::kernel::RemusKernel;
use remus_operations::boolean::BooleanOp;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const TOL: f64 = 1e-10;

/// Row-major placements of two cylinders of radius `r`, length `6r`, along
/// Z and X through the origin.
fn along_z(r: f64) -> Vec<f64> {
    vec![
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        -3.0 * r,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn along_x(r: f64) -> Vec<f64> {
    // rotation_y(90°) then half the length back along X.
    vec![
        0.0,
        0.0,
        1.0,
        -3.0 * r,
        0.0,
        1.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn expected(op: &str, r: f64) -> f64 {
    let steinmetz = 16.0 * r.powi(3) / 3.0;
    let cylinder = PI * r * r * 6.0 * r;
    match op {
        "intersect" => steinmetz,
        "cut" => cylinder - steinmetz,
        _ => 2.0 * cylinder - steinmetz,
    }
}

#[test]
fn detailed_booleans_measure_the_closed_form() {
    for scale in SCALES {
        let r = 10.0 * scale;
        for (op, operation) in [
            (BooleanOp::Intersect, "intersect"),
            (BooleanOp::Cut, "cut"),
            (BooleanOp::Fuse, "fuse"),
        ] {
            for swap in [false, true] {
                let what = format!("{operation} scale {scale} swap {swap}");
                let mut kernel = RemusKernel::new();
                let a = kernel.make_cylinder_solid(r, 6.0 * r).unwrap();
                kernel.transform_solid_binding(a, along_z(r)).unwrap();
                let b = kernel.make_cylinder_solid(r, 6.0 * r).unwrap();
                kernel.transform_solid_binding(b, along_x(r)).unwrap();
                let (x, y) = if swap { (b, a) } else { (a, b) };
                let result =
                    serde_json::to_value(kernel.binary_boolean_detailed_impl(op, operation, x, y))
                        .unwrap();
                assert_eq!(result["status"], "ok", "{what}: {result}");
                let solid = u32::try_from(result["value"].as_u64().unwrap()).unwrap();
                assert_eq!(kernel.validate_solid(solid).unwrap(), 0, "{what}");
                let volume = kernel.volume(solid, 0.01 * r).unwrap();
                let target = expected(operation, r);
                assert!(
                    (volume - target).abs() <= TOL * target,
                    "{what}: volume {volume} vs closed form {target}"
                );
            }
        }
    }
}

#[test]
fn both_batches_measure_the_closed_form() {
    for v2 in [false, true] {
        for scale in SCALES {
            let r = 10.0 * scale;
            for op in ["intersect", "cut", "fuse"] {
                let what = format!("v2={v2} {op} scale {scale}");
                let request = json!([
                    {"op": "makeCylinder", "args": {"radius": r, "height": 6.0 * r}},
                    {"op": "transform", "args": {"solid": 0, "matrix": along_z(r)}},
                    {"op": "makeCylinder", "args": {"radius": r, "height": 6.0 * r}},
                    {"op": "transform", "args": {"solid": 1, "matrix": along_x(r)}},
                    {"op": op, "args": {"solidA": 0, "solidB": 1}},
                ])
                .to_string();
                let mut kernel = RemusKernel::new();
                let mut run = |request: &str| -> Vec<Value> {
                    let response = if v2 {
                        kernel.execute_batch_v2(request)
                    } else {
                        kernel.execute_batch(request)
                    };
                    serde_json::from_str(&response).unwrap()
                };
                let cells = run(&request);
                assert!(
                    cells.iter().all(|cell| cell.get("ok").is_some()),
                    "{what}: {cells:?}"
                );
                let solid = cells[4]["ok"].as_u64().unwrap();
                let probes = json!([
                    {"op": "validateSolid", "args": {"solid": solid}},
                    {"op": "volume", "args": {"solid": solid, "deflection": 0.01 * r}},
                ])
                .to_string();
                let cells = run(&probes);
                assert_eq!(cells[0]["ok"], 0, "{what}: {cells:?}");
                let volume = cells[1]["ok"].as_f64().unwrap();
                let target = expected(op, r);
                assert!(
                    (volume - target).abs() <= TOL * target,
                    "{what}: volume {volume} vs closed form {target}"
                );
            }
        }
    }
}
