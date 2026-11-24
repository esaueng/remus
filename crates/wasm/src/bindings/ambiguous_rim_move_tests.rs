//! Face moves over a history-free chamfer or draft band (#1957): the default
//! detailed and prepared moves refuse with `ambiguous_rim_chamfer`, naming
//! the band; `moveFacesDetailedWithOptions` and `prepareMoveFacesWithOptions`
//! build the shape the caller reads it as.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::vec::Vec3;
use remus_operations::blend_ops::chamfer_v2;
use remus_operations::measure::solid_volume;
use remus_topology::explorer::{solid_edges, solid_faces};

use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;

const X: f64 = 60.0;
const Y: f64 = 40.0;
const C: f64 = 0.35;

/// A 60 × 40 × 20 box with its bottom rim chamfered 0.35 without the
/// journal, and its bottom face.
fn fixture(kernel: &mut RemusKernel) -> (u32, u32) {
    let blank = kernel.make_box_solid(X, Y, 20.0).unwrap();
    let blank = kernel.resolve_solid(blank).unwrap();
    let rim: Vec<_> = solid_edges(kernel.topo(), blank)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let edge = kernel.topo().edge(edge).unwrap();
            [edge.start(), edge.end()]
                .into_iter()
                .all(|v| kernel.topo().vertex(v).unwrap().point().z().abs() < 1e-9)
        })
        .collect();
    assert_eq!(rim.len(), 4);
    let solid = chamfer_v2(kernel.topo_mut(), blank, &rim, C, C)
        .unwrap()
        .solid;
    let bottom = solid_faces(kernel.topo(), solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            kernel
                .topo()
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(Vec3::new(0.0, 0.0, -1.0)) > 1.0 - 1e-9)
        })
        .unwrap();
    (solid_id_to_u32(solid), face_id_to_u32(bottom))
}

fn volume(kernel: &RemusKernel, solid: u32) -> f64 {
    solid_volume(kernel.topo(), kernel.resolve_solid(solid).unwrap(), 1e-3).unwrap()
}

/// The volume added by moving the bottom out by `distance` with the band
/// kept on its planes (read as a draft).
fn drafted_change(distance: f64) -> f64 {
    let section =
        |z: f64| 2.0f64.mul_add(-C * (1.0 - z / C), X) * 2.0f64.mul_add(-C * (1.0 - z / C), Y);
    distance / 6.0 * (4.0f64.mul_add(section(-0.5 * distance), section(-distance)) + section(0.0))
}

#[test]
fn the_default_move_refuses_naming_the_band() {
    let mut kernel = RemusKernel::new();
    let (solid, bottom) = fixture(&mut kernel);
    let before = kernel.serialize_solids(&[solid]).unwrap();
    for result in [
        kernel.move_faces_detailed_impl(solid, &[bottom], 2.0),
        kernel.move_faces_detailed_with_options_impl(solid, &[bottom], 2.0, "{}"),
        kernel.move_faces_detailed_with_options_impl(
            solid,
            &[bottom],
            2.0,
            r#"{"ambiguousRim":"refuse"}"#,
        ),
    ] {
        let result = serde_json::to_value(result).unwrap();
        assert_eq!(result["status"], "error", "{result}");
        assert_eq!(result["category"], "unsupported", "{result}");
        assert_eq!(result["code"], "ambiguous_rim_chamfer", "{result}");
        let details = &result["details"];
        assert_eq!(details["kernelCode"], "ambiguous_rim_chamfer", "{result}");
        assert_eq!(details["cap"], bottom, "{result}");
        for key in ["face", "wall", "edge"] {
            assert!(details[key].is_u64(), "{key}: {result}");
        }
        assert!((details["capSetback"].as_f64().unwrap() - C).abs() < 1e-9);
        assert!((details["wallSetback"].as_f64().unwrap() - C).abs() < 1e-9);
    }
    let prepared = kernel.prepare_move_faces_impl(solid, &[bottom]).unwrap();
    let error = kernel.move_faces_prepared_impl(prepared, 2.0).unwrap_err();
    assert!(error.to_string().contains("ambiguous rim band"), "{error}");
    assert_eq!(kernel.serialize_solids(&[solid]).unwrap(), before);
}

#[test]
fn the_caller_reads_the_band_as_a_chamfer_or_a_draft() {
    for (reading, distance, change) in [
        ("chamfer", 2.0, 2.0 * X * Y),
        ("chamfer", -2.0, -2.0 * X * Y),
        ("draft", 2.0, drafted_change(2.0)),
        ("draft", -0.1, drafted_change(-0.1)),
    ] {
        let mut kernel = RemusKernel::new();
        let (solid, bottom) = fixture(&mut kernel);
        let source = volume(&kernel, solid);
        let options = format!(r#"{{"ambiguousRim":"{reading}"}}"#);
        let result = serde_json::to_value(kernel.move_faces_detailed_with_options_impl(
            solid,
            &[bottom],
            distance,
            &options,
        ))
        .unwrap();
        assert_eq!(result["status"], "ok", "{reading} {distance}: {result}");
        assert_eq!(result["details"]["ambiguousRim"], reading);
        let moved = u32::try_from(result["value"].as_u64().unwrap()).unwrap();
        let got = volume(&kernel, moved) - source;
        assert!(
            (got - change).abs() <= 1e-9 * change.abs(),
            "{reading} {distance}: volume changed by {got}, expected {change}"
        );

        let prepared = kernel
            .prepare_move_faces_with_options_impl(
                solid,
                &[bottom],
                super::modifier_detailed::parse_move_faces_options(&options).unwrap(),
            )
            .unwrap();
        let dragged = kernel.move_faces_prepared_impl(prepared, distance).unwrap();
        let got = volume(&kernel, dragged) - source;
        assert!(
            (got - change).abs() <= 1e-9 * change.abs(),
            "prepared {reading} {distance}: volume changed by {got}, expected {change}"
        );
    }
}

#[test]
fn malformed_options_are_refused() {
    let mut kernel = RemusKernel::new();
    let (solid, bottom) = fixture(&mut kernel);
    for options in [
        "",
        "[]",
        r#"{"ambiguousRim":"maybe"}"#,
        r#"{"ambiguousRim":1}"#,
        r#"{"rim":"chamfer"}"#,
    ] {
        let result = serde_json::to_value(kernel.move_faces_detailed_with_options_impl(
            solid,
            &[bottom],
            2.0,
            options,
        ))
        .unwrap();
        assert_eq!(result["status"], "error", "{options}: {result}");
        assert_eq!(result["code"], "invalid_argument", "{options}: {result}");
        assert_eq!(
            result["details"]["argument"], "options",
            "{options}: {result}"
        );
    }
}

/// `executeBatch` reaches the same options: `moveFacesDetailedWithOptions`
/// with `options` resolves the band, without it refuses, and malformed
/// options are refused; each matches the direct call.
#[test]
fn the_batch_route_takes_the_same_options() {
    let cases = [
        (Some(serde_json::json!({"ambiguousRim": "chamfer"})), "ok"),
        (Some(serde_json::json!({"ambiguousRim": "draft"})), "ok"),
        (
            Some(serde_json::json!({"ambiguousRim": "legacyReplay"})),
            "ok",
        ),
        (None, "error"),
        (Some(serde_json::json!({"ambiguousRim": "maybe"})), "error"),
    ];
    for (options, status) in cases {
        let mut kernel = RemusKernel::new();
        let (solid, bottom) = fixture(&mut kernel);
        let mut args = serde_json::json!({"solid": solid, "faces": [bottom], "distance": 2.0});
        if let Some(options) = &options {
            args["options"] = options.clone();
        }
        let input = serde_json::json!([{"op": "moveFacesDetailedWithOptions", "args": args}]);
        let output: serde_json::Value =
            serde_json::from_str(&kernel.execute_batch(&input.to_string())).unwrap();
        let batch = output[0]["ok"].clone();
        assert_eq!(batch["status"], status, "{options:?}: {output}");

        let mut kernel = RemusKernel::new();
        let (solid, bottom) = fixture(&mut kernel);
        let text = options.map_or_else(|| "{}".to_owned(), |options| options.to_string());
        let direct = serde_json::to_value(kernel.move_faces_detailed_with_options_impl(
            solid,
            &[bottom],
            2.0,
            &text,
        ))
        .unwrap();
        assert_eq!(batch, direct, "{text}");
    }
}

/// The multi-face twin takes the same options: over the history-free bottom
/// rim, moving the bottom and the top apart refuses by default, proceeds
/// when the band is read as a draft (the draft volume plus the top's slab),
/// refuses as unsupported when it is read as a chamfer; the batch op
/// matches the direct call.
#[test]
fn the_multi_face_twin_takes_the_same_options() {
    let top_of = |kernel: &RemusKernel, solid: u32| {
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
                    .is_some_and(|n| n.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
            })
            .unwrap();
        face_id_to_u32(face)
    };
    for (options, status, code) in [
        ("{}", "error", Some("ambiguous_rim_chamfer")),
        (r#"{"ambiguousRim":"draft"}"#, "ok", None),
        (
            r#"{"ambiguousRim":"chamfer"}"#,
            "error",
            Some("rim_chamfer_not_carried"),
        ),
    ] {
        let mut kernel = RemusKernel::new();
        let (solid, bottom) = fixture(&mut kernel);
        let top = top_of(&kernel, solid);
        let source = volume(&kernel, solid);
        let direct = serde_json::to_value(kernel.move_faces_multi_detailed_with_options_impl(
            solid,
            &[bottom, top],
            &[2.0, 1.0],
            options,
        ))
        .unwrap();
        assert_eq!(direct["status"], status, "{options}: {direct}");
        if let Some(code) = code {
            assert_eq!(direct["code"], code, "{options}: {direct}");
            assert_eq!(direct["category"], "unsupported", "{options}: {direct}");
        } else {
            let moved = u32::try_from(direct["value"].as_u64().unwrap()).unwrap();
            let expected = drafted_change(2.0) + X * Y;
            let got = volume(&kernel, moved) - source;
            assert!(
                (got - expected).abs() <= 1e-9 * expected,
                "{got} vs {expected}"
            );
        }

        let mut kernel = RemusKernel::new();
        let (solid, bottom) = fixture(&mut kernel);
        let top = top_of(&kernel, solid);
        let input = serde_json::json!([{"op": "moveFacesMultiDetailedWithOptions", "args": {
            "solid": solid, "faces": [bottom, top], "distances": [2.0, 1.0],
            "options": serde_json::from_str::<serde_json::Value>(options).unwrap(),
        }}]);
        let output: serde_json::Value =
            serde_json::from_str(&kernel.execute_batch(&input.to_string())).unwrap();
        assert_eq!(output[0]["ok"], direct, "{options}");
    }
}
