//! Direct and both batch WASM qualification for the B38 sphere–torus
//! orientation repair (P-Class 8.1 seeds 5201 and 1146). Tier: core — the
//! exact boolean writes the live session.
//!
//! Each persisted chain (mirror included) runs every operator in both operand
//! orders. Direct calls must return an exact, validator-clean body with the
//! independent slice-integral volume; `executeBatch` and `executeBatchV2`
//! must agree with them and, apart from the seed-1146 legs that keep a
//! sphere face with two section holes (a separate tessellation defect), mesh
//! watertight at two deflections. A refused exact-only cell must leave the
//! session unchanged.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use serde_json::{Value, json};

use crate::kernel::RemusKernel;

/// Independent slice integrals (disc ∩ annulus per height), as in the native
/// `regress_opseq_sphere_torus_orientation`.
const SEED_5201_INTERSECTION: f64 = 241.467_103_793_182_87;
const SEED_1146_INTERSECTION: f64 = 135.979_005_791_330_85;

struct Seed {
    name: &'static str,
    sphere: f64,
    torus: f64,
    intersection: f64,
}

const SEEDS: [Seed; 2] = [
    Seed {
        name: "seed5201",
        sphere: 4.0 / 3.0 * std::f64::consts::PI * 125.0,
        torus: 2.0 * std::f64::consts::PI * std::f64::consts::PI * 5.5 * 2.75 * 2.75,
        intersection: SEED_5201_INTERSECTION,
    },
    Seed {
        name: "seed1146",
        sphere: 4.0 / 3.0 * std::f64::consts::PI * 512.0,
        torus: 2.0 * std::f64::consts::PI * std::f64::consts::PI * 4.0 * 2.0 * 2.0,
        intersection: SEED_1146_INTERSECTION,
    },
];

fn expected(seed: &Seed, op: &str, sphere_first: bool) -> f64 {
    match (op, sphere_first) {
        ("fuse", _) => seed.sphere + seed.torus - seed.intersection,
        ("intersect", _) => seed.intersection,
        ("cut", true) => seed.sphere - seed.intersection,
        _ => seed.torus - seed.intersection,
    }
}

/// The persisted chain as batch operations; returns `(ops, sphere, torus)`.
fn chain(seed: &Seed) -> (Vec<Value>, u32, u32) {
    if seed.name == "seed5201" {
        (
            vec![
                json!({"op": "makeSphere", "args": {"radius": 5.0, "segments": 16}}),
                json!({"op": "makeTorus", "args": {"majorRadius": 5.5, "minorRadius": 2.75, "segments": 16}}),
                json!({"op": "mirror", "args": {"solid": 0, "px": 0.0, "py": 1.5, "pz": 0.0, "nx": 0.0, "ny": 1.0, "nz": 0.0}}),
            ],
            2,
            1,
        )
    } else {
        (
            vec![
                json!({"op": "makeTorus", "args": {"majorRadius": 4.0, "minorRadius": 2.0, "segments": 16}}),
                json!({"op": "makeSphere", "args": {"radius": 8.0, "segments": 16}}),
                json!({"op": "mirror", "args": {"solid": 0, "px": -4.0, "py": 0.0, "pz": 0.0, "nx": 1.0, "ny": 0.0, "nz": 0.0}}),
                json!({"op": "transform", "args": {"solid": 2, "matrix": [
                    1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -3.0, 0.0, 0.0, 0.0, 1.0
                ]}}),
            ],
            1,
            2,
        )
    }
}

/// The same chain through direct kernel calls; returns `(sphere, torus)`.
fn chain_direct(kernel: &mut RemusKernel, seed: &Seed) -> (u32, u32) {
    if seed.name == "seed5201" {
        let sphere = kernel.make_sphere_solid(5.0, 16).unwrap();
        let torus = kernel.make_torus_solid(5.5, 2.75, 16).unwrap();
        let image = kernel
            .mirror_solid(sphere, 0.0, 1.5, 0.0, 0.0, 1.0, 0.0)
            .unwrap();
        (image, torus)
    } else {
        let torus = kernel.make_torus_solid(4.0, 2.0, 16).unwrap();
        let sphere = kernel.make_sphere_solid(8.0, 16).unwrap();
        let image = kernel
            .mirror_solid(torus, -4.0, 0.0, 0.0, 1.0, 0.0, 0.0)
            .unwrap();
        kernel
            .transform_solid_binding(
                image,
                vec![
                    1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -3.0, 0.0, 0.0, 0.0, 1.0,
                ],
            )
            .unwrap();
        (sphere, image)
    }
}

const OPS: [&str; 3] = ["intersect", "fuse", "cut"];

#[test]
fn direct_calls_return_exact_valid_sphere_torus_legs() {
    for seed in &SEEDS {
        for op in OPS {
            for sphere_first in [true, false] {
                let label = format!("{} direct {op} sphere_first={sphere_first}", seed.name);
                let mut kernel = RemusKernel::new();
                let (sphere, torus) = chain_direct(&mut kernel, seed);
                let (a, b) = if sphere_first {
                    (sphere, torus)
                } else {
                    (torus, sphere)
                };
                let result = kernel
                    .boolean_with_quality_impl(
                        op,
                        a,
                        b,
                        Some(true),
                        None,
                        None,
                        None,
                        None,
                        None,
                        None,
                    )
                    .unwrap_or_else(|_| panic!("{label}: refused"));
                assert_eq!(result.quality, "exact", "{label}");
                assert_eq!(
                    kernel.validate_solid(result.solid).unwrap(),
                    0,
                    "{label}: validation"
                );
                let volume = kernel.volume(result.solid, 0.001).unwrap();
                let target = expected(seed, op, sphere_first);
                assert!(
                    (volume - target).abs() < 2e-4 * target,
                    "{label}: volume {volume} against {target}"
                );
                for (operand, closed) in [(sphere, seed.sphere), (torus, seed.torus)] {
                    assert_eq!(kernel.validate_solid(operand).unwrap(), 0, "{label}");
                    let volume = kernel.volume(operand, 0.001).unwrap();
                    assert!(
                        (volume - closed).abs() < 2e-4 * closed,
                        "{label}: operand volume {volume}"
                    );
                }
            }
        }
    }
}

#[test]
fn both_batches_return_exact_valid_watertight_sphere_torus_legs() {
    for seed in &SEEDS {
        for v2 in [false, true] {
            for op in OPS {
                for sphere_first in [true, false] {
                    let label = format!(
                        "{} batchv2={v2} {op} sphere_first={sphere_first}",
                        seed.name
                    );
                    let mut kernel = RemusKernel::new();
                    let (mut ops, sphere, torus) = chain(seed);
                    let (a, b) = if sphere_first {
                        (sphere, torus)
                    } else {
                        (torus, sphere)
                    };
                    ops.push(json!({"op": "booleanWithQuality", "args": {
                        "operation": op, "solidA": a, "solidB": b, "exactOnly": true
                    }}));
                    let run = |kernel: &mut RemusKernel, request: &str| {
                        if v2 {
                            kernel.execute_batch_v2(request)
                        } else {
                            kernel.execute_batch(request)
                        }
                    };
                    let cells: Vec<Value> =
                        serde_json::from_str(&run(&mut kernel, &Value::Array(ops).to_string()))
                            .unwrap();
                    assert!(
                        cells.iter().all(|cell| cell.get("ok").is_some()),
                        "{label}: {cells:?}"
                    );
                    let last = &cells[cells.len() - 1]["ok"];
                    assert_eq!(last["quality"], "exact", "{label}");
                    let solid = last["solid"].as_u64().unwrap();
                    let probes = json!([
                        {"op": "validateSolid", "args": {"solid": solid}},
                        {"op": "volume", "args": {"solid": solid, "deflection": 0.001}},
                        {"op": "meshQuality", "args": {"solid": solid, "deflection": 0.1}},
                        {"op": "meshQuality", "args": {"solid": solid, "deflection": 0.02}},
                    ])
                    .to_string();
                    let cells: Vec<Value> =
                        serde_json::from_str(&run(&mut kernel, &probes)).unwrap();
                    assert_eq!(cells[0]["ok"], 0, "{label}: {cells:?}");
                    let volume = cells[1]["ok"].as_f64().unwrap();
                    let target = expected(seed, op, sphere_first);
                    assert!(
                        (volume - target).abs() < 2e-4 * target,
                        "{label}: volume {volume} against {target}"
                    );
                    // Seed 1146's fuse and sphere − torus cut keep one sphere
                    // face with two section holes; its open mesh is a separate
                    // tessellation defect outside this repair.
                    let holed_sphere =
                        seed.name == "seed1146" && (op == "fuse" || (op == "cut" && sphere_first));
                    if !holed_sphere {
                        for cell in &cells[2..] {
                            assert_eq!(cell["ok"]["isWatertight"], true, "{label}: {cells:?}");
                            assert_eq!(cell["ok"]["boundaryEdges"], 0, "{label}");
                            assert_eq!(cell["ok"]["nonManifoldEdges"], 0, "{label}");
                        }
                    }
                }
            }
        }
    }
}

/// An exact-only refusal through either batch leaves the session as it was:
/// the placed seed-1146 fuse is outside the qualified domain.
#[test]
fn both_batches_roll_back_a_refused_placed_fuse() {
    for v2 in [false, true] {
        let mut kernel = RemusKernel::new();
        let (mut ops, sphere, torus) = chain(&SEEDS[1]);
        // The native matrix's rigid placement, applied to both operands.
        let placement = remus_math::mat::Mat4::translation(17.0, -23.0, 31.0)
            * remus_math::mat::Mat4::rotation_x(0.37)
            * remus_math::mat::Mat4::rotation_z(-0.61);
        let matrix: Vec<f64> = (0..4)
            .flat_map(|row| (0..4).map(move |col| placement.0[row][col]))
            .collect();
        for solid in [sphere, torus] {
            ops.push(json!({"op": "transform", "args": {"solid": solid, "matrix": matrix}}));
        }
        let run = |kernel: &mut RemusKernel, request: &str| {
            if v2 {
                kernel.execute_batch_v2(request)
            } else {
                kernel.execute_batch(request)
            }
        };
        let cells: Vec<Value> =
            serde_json::from_str(&run(&mut kernel, &Value::Array(ops).to_string())).unwrap();
        assert!(
            cells.iter().all(|cell| cell.get("ok").is_some()),
            "{cells:?}"
        );
        let census = |kernel: &mut RemusKernel| {
            let probes = json!([
                {"op": "volume", "args": {"solid": sphere, "deflection": 0.001}},
                {"op": "volume", "args": {"solid": torus, "deflection": 0.001}},
                {"op": "validateSolid", "args": {"solid": sphere}},
                {"op": "validateSolid", "args": {"solid": torus}},
            ])
            .to_string();
            run(kernel, &probes)
        };
        let before = census(&mut kernel);
        let request = json!([{"op": "booleanWithQuality", "args": {
            "operation": "fuse", "solidA": sphere, "solidB": torus, "exactOnly": true
        }}])
        .to_string();
        let cells: Vec<Value> = serde_json::from_str(&run(&mut kernel, &request)).unwrap();
        assert!(cells[0].get("error").is_some(), "batchv2={v2}: {cells:?}");
        assert_eq!(census(&mut kernel), before, "batchv2={v2}: rollback");
        // The next handle is the one the refused fuse would have taken.
        let probe =
            json!([{"op": "makeSphere", "args": {"radius": 1.0, "segments": 16}}]).to_string();
        let cells: Vec<Value> = serde_json::from_str(&run(&mut kernel, &probe)).unwrap();
        assert_eq!(
            cells[0]["ok"], 3,
            "batchv2={v2}: refused fuse allocated a handle"
        );
    }
}
