//! Optional `executeBatch` arguments: absent means default, present but
//! malformed is an error (kernel review WASM-5).
//!
//! The arms below read their optional arguments with `get_f64(..).unwrap_or(d)`
//! (and `get_u32` / `as_bool` equivalents), so a present value of the wrong
//! type silently became the default: `mirror` with `nx: "0"` mirrored about
//! the YZ plane and reported success.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use crate::handles::face_id_to_u32;
use crate::kernel::BrepKernel;

fn run_v2(kernel: &mut BrepKernel, op: &str, args: &serde_json::Value) -> serde_json::Value {
    let out = kernel.execute_batch_v2(&serde_json::json!([{"op": op, "args": args}]).to_string());
    let mut results: Vec<serde_json::Value> = serde_json::from_str(&out).unwrap();
    results.remove(0)
}

/// A box solid, one of its faces, and a straight NURBS spine edge.
fn fixture() -> (BrepKernel, u32, u32, u32) {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let faces = remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap();
    let face = face_id_to_u32(faces[0]);
    let spine = run_v2(
        &mut kernel,
        "makeNurbsEdge",
        &serde_json::json!({
            "startX": 0.0, "startY": 0.0, "startZ": 0.0,
            "endX": 0.0, "endY": 0.0, "endZ": 5.0,
            "degree": 1,
            "knots": [0.0, 0.0, 1.0, 1.0],
            "controlPoints": [0.0, 0.0, 0.0, 0.0, 0.0, 5.0],
            "weights": [1.0, 1.0]
        }),
    );
    let spine = u32::try_from(spine["ok"].as_u64().expect("spine edge")).unwrap();
    (kernel, solid, face, spine)
}

/// Every arm that had a defaulted optional argument, with valid required
/// arguments, and the optional keys it reads.
fn cases(
    solid: u32,
    face: u32,
    spine: u32,
) -> Vec<(&'static str, serde_json::Value, &'static [&'static str])> {
    vec![
        (
            "makeSphere",
            serde_json::json!({"radius": 1.0}),
            &["segments"],
        ),
        (
            "makeTorus",
            serde_json::json!({"majorRadius": 3.0, "minorRadius": 1.0}),
            &["segments"],
        ),
        (
            "extrude",
            serde_json::json!({"face": face}),
            &["dx", "dy", "dz", "distance"],
        ),
        (
            "revolve",
            serde_json::json!({"face": face, "angle": 90.0}),
            &["originX", "originY", "originZ", "axisX", "axisY", "axisZ"],
        ),
        (
            "multiSectionSweep",
            serde_json::json!({"faces": [], "params": [], "spineEdge": spine}),
            &["ruled"],
        ),
        (
            "projectEdges",
            serde_json::json!({
                "solid": solid,
                "originX": 0.0, "originY": 0.0, "originZ": 10.0,
                "dirX": 0.0, "dirY": 0.0, "dirZ": -1.0,
                "xAxisX": 1.0, "xAxisY": 0.0, "xAxisZ": 0.0
            }),
            &["hiddenLines"],
        ),
        (
            "mirror",
            serde_json::json!({"solid": solid}),
            &["px", "py", "pz", "nx", "ny", "nz"],
        ),
        (
            "convertToElementary",
            serde_json::json!({"solid": solid}),
            &["tolerance"],
        ),
        (
            "healSolid",
            serde_json::json!({"solid": solid}),
            &["tolerance"],
        ),
        (
            "repairSolid",
            serde_json::json!({"solid": solid}),
            &["tolerance"],
        ),
        (
            "classifyPoint",
            serde_json::json!({"solid": solid, "x": 0.5, "y": 0.5, "z": 0.5}),
            &["tolerance"],
        ),
        (
            "circularPattern",
            serde_json::json!({"solid": solid, "count": 2}),
            &["ax", "ay", "az"],
        ),
        (
            "gridPattern",
            serde_json::json!({
                "solid": solid, "spacingX": 5.0, "spacingY": 5.0, "countX": 1, "countY": 1
            }),
            &["dirXx", "dirXy", "dirXz", "dirYx", "dirYy", "dirYz"],
        ),
        (
            "offsetFace",
            serde_json::json!({"face": face, "distance": 0.5}),
            &["samples"],
        ),
        (
            "section",
            serde_json::json!({"solid": solid}),
            &["px", "py", "pz", "nx", "ny", "nz"],
        ),
        (
            "split",
            serde_json::json!({"solid": solid}),
            &["px", "py", "pz", "nx", "ny", "nz"],
        ),
        (
            "sewFaces",
            serde_json::json!({"faces": [face]}),
            &["tolerance"],
        ),
        (
            "linearPattern",
            serde_json::json!({"solid": solid, "spacing": 5.0, "count": 2}),
            &["dx", "dy", "dz"],
        ),
        (
            "fuseWithOptions",
            serde_json::json!({"solidA": solid, "solidB": solid}),
            &["unifyFaces"],
        ),
        (
            "cutWithOptions",
            serde_json::json!({"solidA": solid, "solidB": solid}),
            &["unifyFaces"],
        ),
        (
            "intersectWithOptions",
            serde_json::json!({"solidA": solid, "solidB": solid}),
            &["unifyFaces"],
        ),
        (
            "draft",
            serde_json::json!({"solid": solid, "angle": 5.0, "faces": [face]}),
            &["dirX", "dirY", "dirZ", "neutralX", "neutralY", "neutralZ"],
        ),
    ]
}

/// Every optional key, set to a value of the wrong type, is refused as an
/// `invalid_argument` naming that key instead of reading as its default.
#[test]
fn malformed_optional_arguments_are_refused_not_defaulted() {
    let (mut kernel, solid, face, spine) = fixture();
    let mut checked = 0;
    for (op, base, keys) in cases(solid, face, spine) {
        for &key in keys {
            for bad in [serde_json::json!("0"), serde_json::json!([1.0])] {
                let mut args = base.clone();
                args[key] = bad.clone();
                let result = run_v2(&mut kernel, op, &args);
                assert_eq!(
                    (
                        result["error"]["code"].as_str(),
                        result["error"]["details"]["argument"].as_str()
                    ),
                    (Some("invalid_argument"), Some(key)),
                    "{op} with {key} = {bad} must refuse that argument, got {result}"
                );
                checked += 1;
            }
        }
    }
    // 54 numeric defaults plus the five boolean ones.
    assert_eq!(checked, 2 * 59);
}

/// Naming and journaled dispatch have the same strict optional-argument
/// contract as the main batch arms, including on the legacy route.
#[test]
fn naming_and_journaled_options_refuse_before_mutating_state() {
    for v2 in [false, true] {
        let (mut kernel, solid, face, _) = fixture();
        kernel
            .set_face_name(face, Some("retained name".to_owned()))
            .unwrap();
        let before = (
            kernel.serialize_solids(&[solid]).unwrap(),
            kernel.journal_summary(),
            kernel.topo().num_solids(),
            kernel.topo().num_vertices(),
        );
        for (op, args, key) in [
            (
                "propagateAttributesForOp",
                serde_json::json!({"op": 0, "allowInferred": "false"}),
                "allowInferred",
            ),
            (
                "captureSignatureRef",
                serde_json::json!({"kind": "face", "handle": face, "quantum": "bad"}),
                "quantum",
            ),
            (
                "setFaceName",
                serde_json::json!({"face": face, "name": 123}),
                "name",
            ),
            (
                "setFaceName",
                serde_json::json!({"face": face, "name": ["bad"]}),
                "name",
            ),
            (
                "extrudeJournaled",
                serde_json::json!({"face": face, "dx": "bad"}),
                "dx",
            ),
            (
                "extrudeJournaled",
                serde_json::json!({"face": face, "dy": false}),
                "dy",
            ),
            (
                "extrudeJournaled",
                serde_json::json!({"face": face, "dz": {}}),
                "dz",
            ),
            (
                "extrudeJournaled",
                serde_json::json!({"face": face, "distance": [1]}),
                "distance",
            ),
        ] {
            let request = serde_json::json!([{"op": op, "args": args}]).to_string();
            let response = if v2 {
                kernel.execute_batch_v2(&request)
            } else {
                kernel.execute_batch(&request)
            };
            let response: serde_json::Value = serde_json::from_str(&response).unwrap();
            if v2 {
                assert_eq!(
                    response[0]["error"]["code"], "invalid_argument",
                    "{op}: {response}"
                );
                assert_eq!(
                    response[0]["error"]["details"]["argument"], key,
                    "{op}: {response}"
                );
            } else {
                assert!(
                    response[0]["error"]
                        .as_str()
                        .is_some_and(|error| error.contains(key)),
                    "{op}: {response}"
                );
            }
            assert_eq!(
                (
                    kernel.serialize_solids(&[solid]).unwrap(),
                    kernel.journal_summary(),
                    kernel.topo().num_solids(),
                    kernel.topo().num_vertices(),
                ),
                before,
                "{op}.{key} must preserve geometry, attributes and journal"
            );
        }
    }
}

#[test]
fn naming_and_journaled_options_keep_absent_and_null_defaults() {
    for v2 in [false, true] {
        let mut kernel = BrepKernel::new();
        let face = kernel.make_rectangle(4.0, 2.0).unwrap();
        let mut run = |op: &str, args: serde_json::Value| {
            let request = serde_json::json!([{"op": op, "args": args}]).to_string();
            let result = if v2 {
                kernel.execute_batch_v2(&request)
            } else {
                kernel.execute_batch(&request)
            };
            let result: serde_json::Value = serde_json::from_str(&result).unwrap();
            assert!(result[0].get("ok").is_some(), "{op}: {result}");
            result[0]["ok"].clone()
        };
        let signature = run(
            "captureSignatureRef",
            serde_json::json!({"kind": "face", "handle": face}),
        );
        assert_eq!(
            signature,
            run(
                "captureSignatureRef",
                serde_json::json!({"kind": "face", "handle": face, "quantum": null})
            )
        );
        for name in [serde_json::json!({}), serde_json::json!({"name": null})] {
            run(
                "setFaceName",
                serde_json::json!({"face": face, "name": "datum"}),
            );
            let mut clear = name;
            clear["face"] = serde_json::json!(face);
            run("setFaceName", clear);
            assert!(run("getFaceName", serde_json::json!({"face": face})).is_null());
        }
        let first = run("extrudeJournaled", serde_json::json!({"face": face}));
        let second = run(
            "extrudeJournaled",
            serde_json::json!({"face": face, "dx": null, "dy": null, "dz": null, "distance": null}),
        );
        assert_eq!(
            run(
                "propagateAttributesForOp",
                serde_json::json!({"op": first["op"]})
            ),
            run(
                "propagateAttributesForOp",
                serde_json::json!({"op": first["op"], "allowInferred": null})
            )
        );
        for solid in [
            first["solid"].as_u64().unwrap(),
            second["solid"].as_u64().unwrap(),
        ] {
            let volume = run("volume", serde_json::json!({"solid": solid}));
            assert!((volume.as_f64().unwrap() - 8.0).abs() < 1e-9);
        }
    }
}

/// Integer options also refuse values outside the `u32` range, which used to
/// read as the default too.
#[test]
fn out_of_range_integer_options_are_refused() {
    let (mut kernel, _solid, face, _spine) = fixture();
    for (op, args, key) in [
        (
            "makeSphere",
            serde_json::json!({"radius": 1.0, "segments": -1}),
            "segments",
        ),
        (
            "makeTorus",
            serde_json::json!({"majorRadius": 3.0, "minorRadius": 1.0, "segments": 4_294_967_312_u64}),
            "segments",
        ),
        (
            "offsetFace",
            serde_json::json!({"face": face, "distance": 0.5, "samples": 2.5}),
            "samples",
        ),
    ] {
        let result = run_v2(&mut kernel, op, &args);
        assert_eq!(
            result["error"]["details"]["argument"].as_str(),
            Some(key),
            "{op}: {result}"
        );
    }
}

/// The reported case: a malformed normal component no longer mirrors about
/// the default plane, while absent and `null` components keep the defaults.
#[test]
fn mirror_refuses_a_string_normal_and_keeps_defaults_for_absent_ones() {
    let (mut kernel, solid, _face, _spine) = fixture();
    let solids_before = kernel.topo().num_solids();
    let refused = run_v2(
        &mut kernel,
        "mirror",
        &serde_json::json!({"solid": solid, "nx": "0", "ny": 0.0, "nz": 1.0}),
    );
    assert_eq!(refused["error"]["details"]["argument"], "nx", "{refused}");
    assert_eq!(kernel.topo().num_solids(), solids_before);

    // Default plane: through the origin with normal (1, 0, 0), so x flips.
    let bbox = |kernel: &mut BrepKernel, handle: u64| -> Vec<f64> {
        let result = run_v2(kernel, "boundingBox", &serde_json::json!({"solid": handle}));
        serde_json::from_value(result["ok"].clone()).expect("bounding box")
    };
    let original = bbox(&mut kernel, u64::from(solid));
    for args in [
        serde_json::json!({"solid": solid}),
        serde_json::json!({"solid": solid, "nx": null, "px": null}),
    ] {
        let mirrored = run_v2(&mut kernel, "mirror", &args);
        let image = bbox(&mut kernel, mirrored["ok"].as_u64().expect("mirror result"));
        let expected = [
            -original[3],
            original[1],
            original[2],
            -original[0],
            original[4],
            original[5],
        ];
        for (got, want) in image.iter().zip(expected) {
            assert!((got - want).abs() < 1e-9, "{image:?} vs {expected:?}");
        }
    }
}

/// `multiSectionSweep` dropped unreadable `params` entries, which could pair
/// the remaining parameters with the wrong sections.
#[test]
fn multi_section_sweep_refuses_unreadable_params() {
    let (mut kernel, _solid, face, spine) = fixture();
    let result = run_v2(
        &mut kernel,
        "multiSectionSweep",
        &serde_json::json!({"faces": [face, face], "params": [0.0, "x", 1.0], "spineEdge": spine}),
    );
    assert_eq!(
        result["error"]["details"]["argument"], "params[1]",
        "{result}"
    );
}
