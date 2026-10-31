//! Batch `getFaceNormal` contract: a planar face reports its outward normal,
//! oriented by the face's `reversed` flag, the same vector the direct
//! `getFaceNormal` returns (RFC 0010 drift D1), and every other face is
//! refused as the direct method refuses it (D1b).
//!
//! Boolean assembly emits reversed faces whose stored plane normal points
//! into the material; the batch arm used to report that stored normal. It
//! also used to answer a curved face with its surface normal at `(0, 0)`.
//!
//! Every query goes through `execute_batch`: `JsError` cannot be constructed
//! on non-wasm targets. The direct method is called only on planar faces,
//! where it succeeds; its refusal is checked through the shared body.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::RemusKernel;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;

const TOL: f64 = 1e-9;

fn run_ok(k: &mut RemusKernel, op: &str, args: serde_json::Value) -> serde_json::Value {
    let json = serde_json::json!([{"op": op, "args": args}]).to_string();
    let out: Vec<serde_json::Value> = serde_json::from_str(&k.execute_batch(&json)).unwrap();
    out[0]
        .get("ok")
        .cloned()
        .unwrap_or_else(|| panic!("{op} failed: {}", out[0]))
}

fn as_u32(v: &serde_json::Value) -> u32 {
    u32::try_from(v.as_u64().unwrap()).unwrap()
}

fn as_vec3(v: &serde_json::Value) -> [f64; 3] {
    let a = v.as_array().unwrap();
    assert_eq!(a.len(), 3, "normal {v}");
    [
        a[0].as_f64().unwrap(),
        a[1].as_f64().unwrap(),
        a[2].as_f64().unwrap(),
    ]
}

fn solid_faces(k: &mut RemusKernel, solid: u32) -> Vec<u32> {
    run_ok(k, "getSolidFaces", serde_json::json!({"solid": solid}))
        .as_array()
        .unwrap()
        .iter()
        .map(as_u32)
        .collect()
}

fn batch_normal(k: &mut RemusKernel, face: u32) -> [f64; 3] {
    as_vec3(&run_ok(
        k,
        "getFaceNormal",
        serde_json::json!({"face": face}),
    ))
}

fn is_reversed(k: &RemusKernel, face: u32) -> bool {
    let id = k.resolve_face(face).unwrap();
    k.topo().face(id).unwrap().is_reversed()
}

fn is_planar(k: &RemusKernel, face: u32) -> bool {
    let id = k.resolve_face(face).unwrap();
    k.topo().face(id).unwrap().surface().is_planar()
}

fn approx_eq3(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < TOL)
}

/// Box 2×2×2 cut by box 1×1×4, both at the origin: an L-prism with the
/// corner `[0,1]×[0,1]` removed through the full height. Every face is
/// axis-aligned, so its outward normal follows from the plane it lies on:
/// `x = 0` and the inner wall `x = 1` face −X (material lies at larger x),
/// `x = 2` faces +X, and likewise in y and z. The two inner walls come out
/// of the boolean reversed, so the stored plane normal points inward there.
#[test]
fn batch_face_normal_is_outward_on_every_face_of_a_box_cut() {
    let mut k = RemusKernel::new();
    let blank = as_u32(&run_ok(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 2.0, "height": 2.0, "depth": 2.0}),
    ));
    let tool = as_u32(&run_ok(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 1.0, "height": 1.0, "depth": 4.0}),
    ));
    let cut = as_u32(&run_ok(
        &mut k,
        "cut",
        serde_json::json!({"solidA": blank, "solidB": tool}),
    ));

    let faces = solid_faces(&mut k, cut);
    assert_eq!(faces.len(), 8, "L-prism face count");
    let mut reversed = 0;
    let mut seen_inner_walls = 0;
    for face in faces {
        let positions: Vec<f64> = run_ok(
            &mut k,
            "getFaceVertexPositions",
            serde_json::json!({"face": face}),
        )
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();
        // The axis the face is perpendicular to is the one every vertex
        // shares; its level gives the side the material is on.
        let (axis, level) = (0..3)
            .find_map(|axis| {
                let first = positions[axis];
                positions
                    .chunks_exact(3)
                    .all(|p| (p[axis] - first).abs() < TOL)
                    .then_some((axis, first))
            })
            .expect("axis-aligned planar face");
        let sign = if (level - 2.0).abs() < TOL { 1.0 } else { -1.0 };
        let mut expected = [0.0; 3];
        expected[axis] = sign;
        if axis < 2 && (level - 1.0).abs() < TOL {
            seen_inner_walls += 1;
        }
        if is_reversed(&k, face) {
            reversed += 1;
        }

        let normal = batch_normal(&mut k, face);
        assert!(
            approx_eq3(normal, expected),
            "face {face} on plane {}={level}: batch normal {normal:?}, outward {expected:?}",
            ["x", "y", "z"][axis]
        );
    }
    assert_eq!(seen_inner_walls, 2, "both notch walls present");
    assert!(
        reversed >= 2,
        "the fixture must exercise reversed faces (got {reversed})"
    );
}

/// Each fixture is built natively; every planar face of the result is then
/// queried through the batch and directly. The two must agree exactly.
#[test]
fn batch_and_direct_face_normal_agree_on_planar_faces_of_booleans() {
    type Fixture = fn(&mut RemusKernel) -> u32;
    let fixtures: [(&str, Fixture); 4] = [
        ("box cut by offset box", |k| {
            let topo = k.topo_mut();
            let a = make_box(topo, 4.0, 4.0, 4.0).unwrap();
            let b = make_box(topo, 2.0, 2.0, 2.0).unwrap();
            transform_solid(topo, b, &Mat4::translation(3.0, 1.0, 1.0)).unwrap();
            crate::handles::solid_id_to_u32(boolean(topo, BooleanOp::Cut, a, b).unwrap())
        }),
        ("box with an internal pocket cut from the top", |k| {
            let topo = k.topo_mut();
            let a = make_box(topo, 6.0, 6.0, 3.0).unwrap();
            let b = make_box(topo, 2.0, 2.0, 2.0).unwrap();
            transform_solid(topo, b, &Mat4::translation(2.0, 2.0, 2.0)).unwrap();
            crate::handles::solid_id_to_u32(boolean(topo, BooleanOp::Cut, a, b).unwrap())
        }),
        ("fused stepped boxes", |k| {
            let topo = k.topo_mut();
            let a = make_box(topo, 4.0, 4.0, 2.0).unwrap();
            let b = make_box(topo, 2.0, 2.0, 2.0).unwrap();
            transform_solid(topo, b, &Mat4::translation(1.0, 1.0, 2.0)).unwrap();
            crate::handles::solid_id_to_u32(boolean(topo, BooleanOp::Fuse, a, b).unwrap())
        }),
        ("plate drilled through by a cylinder", |k| {
            let topo = k.topo_mut();
            let a = make_box(topo, 10.0, 10.0, 2.0).unwrap();
            let b = make_cylinder(topo, 2.0, 4.0).unwrap();
            transform_solid(topo, b, &Mat4::translation(5.0, 5.0, -1.0)).unwrap();
            crate::handles::solid_id_to_u32(boolean(topo, BooleanOp::Cut, a, b).unwrap())
        }),
    ];

    let mut reversed_planar = 0;
    for (name, build) in fixtures {
        let mut k = RemusKernel::new();
        let solid = build(&mut k);
        let faces = solid_faces(&mut k, solid);
        let mut planar = 0;
        for face in faces {
            if !is_planar(&k, face) {
                continue;
            }
            planar += 1;
            if is_reversed(&k, face) {
                reversed_planar += 1;
            }
            let batch = batch_normal(&mut k, face);
            let direct: [f64; 3] = k
                .get_face_normal(face)
                .unwrap_or_else(|_| panic!("{name}: direct refused planar face {face}"))
                .try_into()
                .unwrap();
            assert!(
                approx_eq3(batch, direct),
                "{name}: face {face} batch {batch:?} vs direct {direct:?}"
            );
        }
        assert!(planar > 0, "{name}: no planar faces");
    }
    assert!(
        reversed_planar > 0,
        "the fixtures must exercise reversed planar faces"
    );
}

/// Curved faces of every analytic primitive refuse in both batch contracts
/// with the direct method's message; their planar caps still answer.
#[test]
fn batch_face_normal_refuses_curved_faces_like_the_direct_method() {
    const MESSAGE: &str = "invalid input: getFaceNormal only works on planar faces";
    let primitives = [
        (
            "makeCylinder",
            serde_json::json!({"radius": 1.0, "height": 2.0}),
        ),
        (
            "makeCone",
            serde_json::json!({"bottomRadius": 1.0, "topRadius": 0.5, "height": 2.0}),
        ),
        ("makeSphere", serde_json::json!({"radius": 1.0})),
        (
            "makeTorus",
            serde_json::json!({"majorRadius": 3.0, "minorRadius": 1.0}),
        ),
    ];
    for (primitive, args) in primitives {
        let mut k = RemusKernel::new();
        let solid = as_u32(&run_ok(&mut k, primitive, args));
        let mut curved = 0;
        for face in solid_faces(&mut k, solid) {
            let direct = k.face_normal_impl(face);
            if is_planar(&k, face) {
                let direct = direct.unwrap_or_else(|e| panic!("{primitive}: {e}"));
                assert!(
                    approx_eq3(batch_normal(&mut k, face), direct),
                    "{primitive}"
                );
                continue;
            }
            curved += 1;
            assert_eq!(
                direct
                    .expect_err("direct refuses a curved face")
                    .to_string(),
                MESSAGE
            );
            let request =
                serde_json::json!([{"op": "getFaceNormal", "args": {"face": face}}]).to_string();
            let legacy: serde_json::Value =
                serde_json::from_str(&k.execute_batch(&request)).unwrap();
            assert_eq!(legacy[0]["error"], MESSAGE, "{primitive}: {legacy}");
            assert!(legacy[0].get("ok").is_none(), "{primitive}: {legacy}");
            let v2: serde_json::Value =
                serde_json::from_str(&k.execute_batch_v2(&request)).unwrap();
            assert_eq!(v2[0]["error"]["message"], MESSAGE, "{primitive}: {v2}");
            assert!(v2[0].get("ok").is_none(), "{primitive}: {v2}");
        }
        assert!(curved > 0, "{primitive}: no curved face");
    }
}
