//! WASM contract tests for the stabilization-plan Phase A operations:
//! draft and defeature through `executeBatch`, with the same closed-form
//! volume oracles as the native qualification suites
//! (`crates/operations/tests/qualify_draft.rs`, `qualify_defeature.rs`),
//! plus native/WASM determinism pins.
//!
//! Every kernel call goes through `execute_batch`: `JsError` cannot be
//! constructed on non-wasm targets, so the `#[wasm_bindgen]` methods are
//! not directly testable on their error paths.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;

use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{face_vertices, solid_edges, solid_faces};

fn run(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&k.execute_batch(&json)).unwrap()
}

fn run_all_ok(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let results = run(k, ops);
    for (i, r) in results.iter().enumerate() {
        assert!(
            r.get("ok").is_some(),
            "op {i} ({}) failed: {r}",
            ops[i]["op"]
        );
    }
    results
        .into_iter()
        .map(|r| r.get("ok").cloned().unwrap())
        .collect()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn as_u32(v: &serde_json::Value) -> u32 {
    u32::try_from(v.as_u64().unwrap()).unwrap()
}

fn generalized_move_fixture(kernel: &mut BrepKernel) -> (u32, u32) {
    let topo = kernel.topo_mut();
    let plate = make_box(topo, 40.0, 40.0, 5.0).expect("plate");
    let boss = make_box(topo, 16.0, 16.0, 10.0).expect("boss");
    transform_solid(topo, boss, &Mat4::translation(12.0, 12.0, 5.0)).expect("place boss");
    let sharp = boolean(topo, BooleanOp::Fuse, plate, boss).expect("fuse boss");
    let drill = make_cylinder(topo, 3.0, 17.0).expect("drill");
    transform_solid(topo, drill, &Mat4::translation(20.0, 20.0, -1.0)).expect("place drill");
    let bored = boolean(topo, BooleanOp::Cut, sharp, drill).expect("drill boss");
    let edge = solid_edges(topo, bored)
        .expect("edges")
        .into_iter()
        .find(|edge| {
            let edge = topo.edge(*edge).expect("edge");
            let start = topo.vertex(edge.start()).expect("start").point();
            let end = topo.vertex(edge.end()).expect("end").point();
            matches!(edge.curve(), EdgeCurve::Line)
                && Tolerance::new().approx_eq(start.y(), 12.0)
                && Tolerance::new().approx_eq(end.y(), 12.0)
                && Tolerance::new().approx_eq(start.z(), 15.0)
                && Tolerance::new().approx_eq(end.z(), 15.0)
        })
        .expect("boss top edge");
    let filleted = fillet_v2(topo, bored, &[edge], 1.0)
        .expect("fillet boss")
        .solid;
    let cap = solid_faces(topo, filleted)
        .expect("faces")
        .into_iter()
        .filter(|face| {
            let face = topo.face(*face).expect("face");
            face.inner_wires().len() == 1
                && face.effective_plane_normal().is_some_and(|normal| {
                    normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - Tolerance::new().angular
                })
        })
        .max_by(|first, second| {
            let max_z = |face| {
                face_vertices(topo, face)
                    .expect("face vertices")
                    .into_iter()
                    .map(|vertex| topo.vertex(vertex).expect("vertex").point().z())
                    .fold(f64::NEG_INFINITY, f64::max)
            };
            max_z(*first).total_cmp(&max_z(*second))
        })
        .expect("holed boss cap");
    (
        crate::handles::solid_id_to_u32(filleted),
        crate::handles::face_id_to_u32(cap),
    )
}

fn kernel_volume(kernel: &BrepKernel, solid: u32) -> f64 {
    let solid = kernel.resolve_solid(solid).expect("solid handle");
    remus_operations::measure::solid_volume(kernel.topo(), solid, 0.001).expect("volume")
}

/// The wall of `solid` whose stored plane normal is closest to `target`.
fn wall_by_normal(k: &mut BrepKernel, solid: u32, target: [f64; 3]) -> u32 {
    let faces = run_all_ok(
        k,
        &[op("getSolidFaces", serde_json::json!({"solid": solid}))],
    );
    let handles: Vec<u32> = faces[0].as_array().unwrap().iter().map(as_u32).collect();
    let mut best = (f64::MIN, 0u32);
    for h in handles {
        let n = run_all_ok(k, &[op("getFaceNormal", serde_json::json!({"face": h}))]);
        let n = n[0].as_array().unwrap();
        let dot = n[0].as_f64().unwrap() * target[0]
            + n[1].as_f64().unwrap() * target[1]
            + n[2].as_f64().unwrap() * target[2];
        if dot > best.0 {
            best = (dot, h);
        }
    }
    assert!(best.0 > 0.99, "no wall matching normal {target:?}");
    best.1
}

/// Draft through the batch API matches the native closed form
/// (1 + tan(a)/2 for a unit cube's +X wall about the base plane).
#[test]
fn batch_draft_volume_matches_closed_form() {
    let mut k = BrepKernel::new();
    let out = run_all_ok(
        &mut k,
        &[op(
            "makeBox",
            serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
        )],
    );
    let cube = as_u32(&out[0]);
    let wall = wall_by_normal(&mut k, cube, [1.0, 0.0, 0.0]);

    let angle = 5.0_f64.to_radians();
    let out = run_all_ok(
        &mut k,
        &[op(
            "draft",
            serde_json::json!({
                "solid": cube, "faces": [wall], "angle": angle,
                "dirX": 0.0, "dirY": 0.0, "dirZ": 1.0,
                "neutralX": 0.0, "neutralY": 0.0, "neutralZ": 0.0,
            }),
        )],
    );
    let drafted = as_u32(&out[0]);

    let out = run_all_ok(
        &mut k,
        &[op(
            "volume",
            serde_json::json!({"solid": drafted, "deflection": 0.01}),
        )],
    );
    let vol = out[0].as_f64().unwrap();
    let expected = 1.0 + angle.tan() / 2.0;
    assert!(
        ((vol - expected) / expected).abs() < 1e-9,
        "expected {expected}, got {vol}"
    );
}

/// Defeature through the batch API: removing a through-hole's walls
/// restores the plain box volume exactly.
#[test]
fn batch_defeature_restores_plain_box() {
    let mut k = BrepKernel::new();
    let out = run_all_ok(
        &mut k,
        &[
            op(
                "makeBox",
                serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
            ),
            op(
                "makeBox",
                serde_json::json!({"width": 0.4, "height": 0.4, "depth": 2.0}),
            ),
        ],
    );
    let cube = as_u32(&out[0]);
    let cutter = as_u32(&out[1]);
    run_all_ok(
        &mut k,
        &[op(
            "transform",
            serde_json::json!({"solid": cutter, "matrix": [
                1.0, 0.0, 0.0, 0.3,
                0.0, 1.0, 0.0, 0.3,
                0.0, 0.0, 1.0, -0.5,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
    let out = run_all_ok(
        &mut k,
        &[op(
            "cut",
            serde_json::json!({"solidA": cube, "solidB": cutter}),
        )],
    );
    let holed = as_u32(&out[0]);

    // The four hole walls: faces whose plane offset |d| is strictly inside
    // the unit box (0.3 or 0.7 on x/y).
    let faces = run_all_ok(
        &mut k,
        &[op("getSolidFaces", serde_json::json!({"solid": holed}))],
    );
    let handles: Vec<u32> = faces[0].as_array().unwrap().iter().map(as_u32).collect();
    let mut walls = Vec::new();
    for h in handles {
        let vs = run_all_ok(
            &mut k,
            &[op("getFaceVertexPositions", serde_json::json!({"face": h}))],
        );
        // Fallback: use normals + a vertex probe is not available as a batch
        // op; classify by every vertex lying strictly inside the hole prism.
        let coords = vs[0].as_array().unwrap();
        let mut inside = !coords.is_empty();
        for chunk in coords.chunks(3) {
            let (x, y) = (chunk[0].as_f64().unwrap(), chunk[1].as_f64().unwrap());
            inside &= (0.29..=0.71).contains(&x) && (0.29..=0.71).contains(&y);
        }
        if inside {
            walls.push(h);
        }
    }
    assert_eq!(walls.len(), 4, "expected the 4 hole walls");

    let out = run_all_ok(
        &mut k,
        &[op(
            "defeature",
            serde_json::json!({"solid": holed, "faces": walls}),
        )],
    );
    let healed = as_u32(&out[0]);
    let out = run_all_ok(
        &mut k,
        &[op(
            "volume",
            serde_json::json!({"solid": healed, "deflection": 0.01}),
        )],
    );
    let vol = out[0].as_f64().unwrap();
    assert!(
        (vol - 1.0).abs() < 1e-9,
        "defeatured box should be exactly 1.0, got {vol}"
    );
}

/// The batch draft is deterministic across fresh kernels: identical result
/// volumes bit-for-bit.
#[test]
fn batch_draft_is_deterministic() {
    let run_once = || {
        let mut k = BrepKernel::new();
        let out = run_all_ok(
            &mut k,
            &[op(
                "makeBox",
                serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
            )],
        );
        let cube = as_u32(&out[0]);
        let wall = wall_by_normal(&mut k, cube, [1.0, 0.0, 0.0]);
        let out = run_all_ok(
            &mut k,
            &[op(
                "draft",
                serde_json::json!({
                    "solid": cube, "faces": [wall], "angle": 0.1,
                    "dirZ": 1.0,
                }),
            )],
        );
        let drafted = as_u32(&out[0]);
        let out = run_all_ok(
            &mut k,
            &[op(
                "volume",
                serde_json::json!({"solid": drafted, "deflection": 0.01}),
            )],
        );
        out[0].as_f64().unwrap().to_bits()
    };
    assert_eq!(run_once(), run_once());
}

/// The shipped direct and batch entry points run the same exact
/// boss/plate/fillet/hole re-limitation cell.
#[test]
fn direct_and_batch_move_faces_match_generalized_witness() {
    let mut direct = BrepKernel::new();
    let (solid, cap) = generalized_move_fixture(&mut direct);
    let direct_result = direct
        .move_faces_binding(solid, vec![cap], 2.0)
        .expect("direct moveFaces");
    let direct_volume = kernel_volume(&direct, direct_result);

    let mut batch = BrepKernel::new();
    let (solid, cap) = generalized_move_fixture(&mut batch);
    let output = run_all_ok(
        &mut batch,
        &[op(
            "moveFaces",
            serde_json::json!({"solid": solid, "faces": [cap], "distance": 2.0}),
        )],
    );
    let batch_result = as_u32(&output[0]);
    let batch_volume = kernel_volume(&batch, batch_result);

    assert_eq!(direct_volume.to_bits(), batch_volume.to_bits());
}

#[test]
fn direct_and_batch_defeature_restore_curved_rim() {
    let fixture = |kernel: &mut BrepKernel| {
        let topo = kernel.topo_mut();
        let sharp = make_cylinder(topo, 10.0, 20.0).unwrap();
        let rim = solid_edges(topo, sharp)
            .unwrap()
            .into_iter()
            .find(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)))
            .unwrap();
        let input = fillet_v2(topo, sharp, &[rim], 1.0).unwrap().solid;
        let band = solid_faces(topo, input)
            .unwrap()
            .into_iter()
            .find(|&face| {
                matches!(
                    topo.face(face).unwrap().surface(),
                    remus_topology::face::FaceSurface::Torus(_)
                )
            })
            .unwrap();
        (
            crate::handles::solid_id_to_u32(input),
            crate::handles::face_id_to_u32(band),
        )
    };
    let mut direct = BrepKernel::new();
    let (input, band) = fixture(&mut direct);
    let result = direct.defeature(input, vec![band]).unwrap();
    let direct_volume = kernel_volume(&direct, result);
    let mut batch = BrepKernel::new();
    let (input, band) = fixture(&mut batch);
    let result = run_all_ok(
        &mut batch,
        &[op(
            "defeature",
            serde_json::json!({"solid": input, "faces": [band]}),
        )],
    );
    let batch_volume = kernel_volume(&batch, as_u32(&result[0]));
    let expected = std::f64::consts::PI * 2000.0;
    assert!((direct_volume - expected).abs() < expected * 1e-4);
    assert_eq!(direct_volume.to_bits(), batch_volume.to_bits());
}

#[test]
fn direct_and_batch_defeature_restore_curved_wall_boss() {
    let fixture = |kernel: &mut BrepKernel| {
        let topo = kernel.topo_mut();
        let base = make_cylinder(topo, 10.0, 20.0).unwrap();
        let boss = make_box(topo, 5.0, 4.0, 4.0).unwrap();
        transform_solid(topo, boss, &Mat4::translation(8.0, -2.0, 8.0)).unwrap();
        let outcome = remus_operations::boolean::boolean_with_context(
            topo,
            BooleanOp::Fuse,
            base,
            boss,
            &remus_math::context::OperationContext::new()
                .with_fallback(remus_math::context::FallbackPolicy::ExactOnly),
        )
        .unwrap();
        assert_eq!(
            outcome.quality,
            remus_operations::boolean::BooleanQuality::Exact
        );
        let input = outcome.solid;
        let selected: Vec<_> = solid_faces(topo, input)
            .unwrap()
            .into_iter()
            .filter(|&face| {
                matches!(
                    topo.face(face).unwrap().surface(),
                    remus_topology::face::FaceSurface::Plane { .. }
                ) && face_vertices(topo, face).unwrap().iter().all(|&vertex| {
                    let z = topo.vertex(vertex).unwrap().point().z();
                    z > 7.0 && z < 13.0
                })
            })
            .map(crate::handles::face_id_to_u32)
            .collect();
        assert_eq!(selected.len(), 5);
        (crate::handles::solid_id_to_u32(input), selected)
    };
    let mut direct = BrepKernel::new();
    let (input, faces) = fixture(&mut direct);
    let result = direct.defeature(input, faces).unwrap();
    let direct_volume = kernel_volume(&direct, result);
    let mut batch = BrepKernel::new();
    let (input, faces) = fixture(&mut batch);
    let result = run_all_ok(
        &mut batch,
        &[op(
            "defeature",
            serde_json::json!({"solid": input, "faces": faces}),
        )],
    );
    let batch_volume = kernel_volume(&batch, as_u32(&result[0]));
    let expected = std::f64::consts::PI * 2000.0;
    assert!((direct_volume - expected).abs() < expected * 1e-4);
    assert_eq!(direct_volume.to_bits(), batch_volume.to_bits());
}

#[test]
fn batch_offset_cone_sphere_boolean_retains_exact_carriers() {
    let overlap = 197.106_403_011_067_53;
    for (operation, expected) in [
        (
            "fuse",
            (208.0 + 256.0 / 3.0) * std::f64::consts::PI - overlap,
        ),
        ("cut", 208.0 * std::f64::consts::PI - overlap),
        ("intersect", overlap),
    ] {
        let mut kernel = BrepKernel::new();
        let inputs = run_all_ok(
            &mut kernel,
            &[
                op(
                    "makeCone",
                    serde_json::json!({"bottomRadius":6,"topRadius":2,"height":12}),
                ),
                op("makeSphere", serde_json::json!({"radius":4,"segments":24})),
            ],
        );
        let a = as_u32(&inputs[0]);
        let b = as_u32(&inputs[1]);
        run_all_ok(
            &mut kernel,
            &[op(
                "transform",
                serde_json::json!({"solid":b,"matrix":[1,0,0,2,0,1,0,0,0,0,1,6,0,0,0,1]}),
            )],
        );
        let result = run_all_ok(
            &mut kernel,
            &[op(
                "booleanWithQuality",
                serde_json::json!({"operation":operation,"solidA":a,"solidB":b,"exactOnly":true}),
            )],
        );
        assert_eq!(result[0]["quality"], "exact");
        let solid = as_u32(&result[0]["solid"]);
        assert!((kernel_volume(&kernel, solid) - expected).abs() < expected * 0.001);
        let result = run_all_ok(
            &mut kernel,
            &[op("validateSolid", serde_json::json!({"solid":solid}))],
        );
        assert_eq!(result[0], 0);
    }
}

#[test]
fn batch_offset_sphere_cylinder_boolean_retains_exact_carriers() {
    let overlap = 294.188_425_924_194_9;
    for (operation, expected) in [
        ("fuse", (288.0 + 180.0) * std::f64::consts::PI - overlap),
        ("cut", 288.0 * std::f64::consts::PI - overlap),
        ("intersect", overlap),
    ] {
        let mut kernel = BrepKernel::new();
        let inputs = run_all_ok(
            &mut kernel,
            &[
                op("makeSphere", serde_json::json!({"radius":6,"segments":24})),
                op("makeCylinder", serde_json::json!({"radius":3,"height":20})),
            ],
        );
        let a = as_u32(&inputs[0]);
        let b = as_u32(&inputs[1]);
        run_all_ok(
            &mut kernel,
            &[op(
                "transform",
                serde_json::json!({"solid":b,"matrix":[1,0,0,2,0,1,0,0,0,0,1,-10,0,0,0,1]}),
            )],
        );
        let result = run_all_ok(
            &mut kernel,
            &[op(
                "booleanWithQuality",
                serde_json::json!({"operation":operation,"solidA":a,"solidB":b,"exactOnly":true}),
            )],
        );
        assert_eq!(result[0]["quality"], "exact");
        let solid = as_u32(&result[0]["solid"]);
        assert!((kernel_volume(&kernel, solid) - expected).abs() < expected * 0.001);
        let result = run_all_ok(
            &mut kernel,
            &[op("validateSolid", serde_json::json!({"solid":solid}))],
        );
        assert_eq!(result[0], 0);
    }
}

#[test]
fn batch_offset_torus_sphere_boolean_retains_exact_carriers() {
    let overlap = 56.270_214_829_835_2;
    for (operation, expected) in [
        (
            "fuse",
            48.0 * std::f64::consts::PI.powi(2) + 36.0 * std::f64::consts::PI - overlap,
        ),
        ("cut", 48.0 * std::f64::consts::PI.powi(2) - overlap),
        ("intersect", overlap),
    ] {
        let mut kernel = BrepKernel::new();
        let inputs = run_all_ok(
            &mut kernel,
            &[
                op(
                    "makeTorus",
                    serde_json::json!({"majorRadius":6,"minorRadius":2,"segments":32}),
                ),
                op("makeSphere", serde_json::json!({"radius":3,"segments":24})),
            ],
        );
        let a = as_u32(&inputs[0]);
        let b = as_u32(&inputs[1]);
        run_all_ok(
            &mut kernel,
            &[op(
                "transform",
                serde_json::json!({"solid":b,"matrix":[1,0,0,5,0,1,0,0,0,0,1,1,0,0,0,1]}),
            )],
        );
        let result = run_all_ok(
            &mut kernel,
            &[op(
                "booleanWithQuality",
                serde_json::json!({"operation":operation,"solidA":a,"solidB":b,"exactOnly":true}),
            )],
        );
        assert_eq!(result[0]["quality"], "exact");
        let solid = as_u32(&result[0]["solid"]);
        assert!((kernel_volume(&kernel, solid) - expected).abs() < expected * 0.001);
        let result = run_all_ok(
            &mut kernel,
            &[op("validateSolid", serde_json::json!({"solid":solid}))],
        );
        assert_eq!(result[0], 0);
    }
}

#[test]
fn batch_through_tool_booleans_preserve_material_across_scales() {
    for exponent in -5..=6 {
        let scale = 10.0_f64.powi(exponent);
        for placed in [false, true] {
            for (operation, expected) in [("fuse", 1.16), ("cut", 0.84), ("intersect", 0.16)] {
                let mut kernel = BrepKernel::new();
                let inputs = run_all_ok(
                    &mut kernel,
                    &[
                        op(
                            "makeBox",
                            serde_json::json!({"width":scale,"height":scale,"depth":scale}),
                        ),
                        op(
                            "makeBox",
                            serde_json::json!({"width":0.4*scale,"height":0.4*scale,"depth":2.0*scale}),
                        ),
                    ],
                );
                let blank = as_u32(&inputs[0]);
                let tool = as_u32(&inputs[1]);
                run_all_ok(
                    &mut kernel,
                    &[op(
                        "transform",
                        serde_json::json!({"solid":tool,"matrix":[1,0,0,0.3*scale,0,1,0,0.3*scale,0,0,1,-0.5*scale,0,0,0,1]}),
                    )],
                );
                if placed {
                    let c = 0.37_f64.cos();
                    let s = 0.37_f64.sin();
                    for solid in [blank, tool] {
                        run_all_ok(
                            &mut kernel,
                            &[op(
                                "transform",
                                serde_json::json!({"solid":solid,"matrix":[c,0,s,17.0*scale,0,1,0,-23.0*scale,-s,0,c,31.0*scale,0,0,0,1]}),
                            )],
                        );
                    }
                }
                let result = run_all_ok(
                    &mut kernel,
                    &[op(
                        "booleanWithQuality",
                        serde_json::json!({"operation":operation,"solidA":blank,"solidB":tool,"exactOnly":true}),
                    )],
                );
                assert_eq!(result[0]["quality"], "exact");
                let solid = as_u32(&result[0]["solid"]);
                let volume = kernel_volume(&kernel, solid) / scale.powi(3);
                assert!(
                    (volume - expected).abs() / expected < 1e-6,
                    "{operation} scale={scale:e} placed={placed}: {volume} vs {expected}"
                );
                let validation = run_all_ok(
                    &mut kernel,
                    &[op("validateSolid", serde_json::json!({"solid":solid}))],
                );
                assert_eq!(validation[0], 0);
            }
        }
    }
}
