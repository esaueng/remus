//! Contract tests for `cutThreadDetailed`, direct and batch.
//!
//! The direct side runs `cut_thread_detailed_impl` (a `JsError` cannot be
//! built off-wasm); the batch side goes through `execute_batch_v2`, whose
//! `ok` value is the same envelope.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use crate::kernel::RemusKernel;

fn live_counts(kernel: &RemusKernel) -> (usize, usize, usize, usize) {
    let topo = kernel.topo();
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    )
}

fn faces_of(kernel: &RemusKernel, solid: u32) -> Vec<u32> {
    let id = kernel.resolve_solid(solid).unwrap();
    remus_topology::explorer::solid_faces(kernel.topo(), id)
        .unwrap()
        .into_iter()
        .map(crate::handles::face_id_to_u32)
        .collect()
}

fn side_face(kernel: &RemusKernel, solid: u32) -> u32 {
    faces_of(kernel, solid)
        .into_iter()
        .find(|&face| {
            let id = kernel.resolve_face(face).unwrap();
            matches!(
                kernel.topo().face(id).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .unwrap()
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

fn cut_batch(kernel: &mut RemusKernel, solid: u32, face: u32, spec: &Value) -> Value {
    let ops = json!([{ "op": "cutThreadDetailed", "args": { "solid": solid, "face": face, "spec": spec } }]);
    let results: Value = serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap();
    results[0].clone()
}

/// ISO M6×1 on a 6 mm boss.
fn m6() -> Value {
    json!({
        "pitch": 1.0,
        "depth": 0.541_265_877_365_273_1,
        "crestWidth": 0.125,
        "upperFlankAngle": 30.0,
        "lowerFlankAngle": 30.0
    })
}

#[test]
fn thread_on_a_boss_matches_between_direct_and_batch() {
    let spec = m6();
    let mut direct_kernel = RemusKernel::new();
    let blank = direct_kernel.make_cylinder_solid(3.0, 6.0).unwrap();
    let face = side_face(&direct_kernel, blank);
    let direct =
        serde_json::to_value(direct_kernel.cut_thread_detailed_impl(blank, face, &spec)).unwrap();
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    assert_eq!(details["quality"], "approximate");
    assert!(details["deviationBound"].as_f64().unwrap() < 1e-4);
    let geometry = &details["geometry"];
    assert_eq!(geometry["kind"], "external");
    assert!((geometry["majorDiameter"].as_f64().unwrap() - 6.0).abs() < 1e-12);
    assert!((geometry["rootWidth"].as_f64().unwrap() - 0.25).abs() < 1e-12);
    assert!((geometry["turns"].as_f64().unwrap() - 6.0).abs() < 1e-12);
    let solid = handle(&direct["value"]);
    assert_ne!(solid, blank, "the input solid is kept");
    let id = direct_kernel.resolve_solid(solid).unwrap();
    let report = remus_operations::validate::validate_solid(direct_kernel.topo(), id).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);

    let mut batch_kernel = RemusKernel::new();
    let batch_blank = batch_kernel.make_cylinder_solid(3.0, 6.0).unwrap();
    let batch_face = side_face(&batch_kernel, batch_blank);
    let batch = cut_batch(&mut batch_kernel, batch_blank, batch_face, &spec);
    assert_eq!(batch["ok"], direct);
}

#[test]
fn refusals_are_typed_and_atomic() {
    let mut kernel = RemusKernel::new();
    let blank = kernel.make_cylinder_solid(3.0, 6.0).unwrap();
    let face = side_face(&kernel, blank);
    let cap = faces_of(&kernel, blank)
        .into_iter()
        .find(|&f| f != face)
        .unwrap();
    let before = live_counts(&kernel);
    let with = |patch: Value| {
        let mut spec = m6();
        for (key, value) in patch.as_object().unwrap() {
            spec[key] = value.clone();
        }
        spec
    };
    let cases = [
        (
            face,
            with(json!({ "length": 7.0 })),
            "thread:span-outside-face",
        ),
        (cap, m6(), "thread:not-a-cylinder"),
        (
            face,
            with(json!({ "majorDiameter": 6.4 })),
            "thread:cylinder-too-small",
        ),
        (
            face,
            with(json!({ "crestWidth": 0.9 })),
            "thread:root-closed",
        ),
        (
            face,
            with(json!({ "starts": 0 })),
            "thread:invalid-parameter",
        ),
        (
            face,
            with(json!({ "lowerFlankAngle": 80.0 })),
            "thread:invalid-parameter",
        ),
        (face, with(json!({ "pitch": "fine" })), "invalid_argument"),
        (face, with(json!({ "flank": 30.0 })), "invalid_argument"),
    ];
    for (target, spec, code) in cases {
        let refused =
            serde_json::to_value(kernel.cut_thread_detailed_impl(blank, target, &spec)).unwrap();
        assert_eq!(refused["status"], "error", "{spec}: {refused}");
        assert_eq!(refused["code"], code, "{spec}: {refused}");
        assert_eq!(live_counts(&kernel), before, "{spec}");
        let batch = cut_batch(&mut kernel, blank, target, &spec);
        assert_eq!(batch["ok"], refused, "{spec}: {batch}");
        assert_eq!(live_counts(&kernel), before, "{spec}");
    }
}
