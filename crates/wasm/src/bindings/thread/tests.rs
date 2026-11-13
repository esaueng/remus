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

/// ISO M6×1 on a 6 mm boss, chamfered at the free end.
fn m6() -> Value {
    json!({
        "pitch": 1.0,
        "depth": 0.541_265_877_365_273_1,
        "crestWidth": 0.125,
        "upperFlankAngle": 30.0,
        "lowerFlankAngle": 30.0,
        "end": { "kind": "chamfer", "angle": 45.0 }
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
    assert_eq!(geometry["start"], json!({ "kind": "plain" }));
    assert_eq!(geometry["end"]["kind"], "chamfer");
    // A cylindrical thread: no taper, the same diameters at both ends.
    assert_eq!(geometry["taperAngle"], json!(0.0));
    for key in ["majorDiameterAtStart", "majorDiameterAtEnd"] {
        assert_eq!(geometry[key], geometry["majorDiameter"], "{key}");
    }
    for key in ["minorDiameterAtStart", "minorDiameterAtEnd"] {
        assert_eq!(geometry[key], geometry["minorDiameter"], "{key}");
    }
    let solid = handle(&direct["value"]);
    assert_ne!(solid, blank, "the input solid is kept");
    let id = direct_kernel.resolve_solid(solid).unwrap();
    let report = remus_operations::validate::validate_solid(direct_kernel.topo(), id).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);

    // Every role names a face of the result; one cone closes the chamfer.
    let faces = faces_of(&direct_kernel, solid);
    let roles = details["faceRoles"].as_array().unwrap();
    let count = |name: &str| roles.iter().filter(|entry| entry["role"] == name).count();
    for entry in roles {
        assert!(faces.contains(&handle(&entry["face"])), "{entry}");
    }
    assert_eq!(count("chamfer.end"), 1);
    assert_eq!(count("runout.end"), 0);
    for name in ["crest", "root", "flank.upper", "flank.lower"] {
        assert!(count(name) >= 11, "{name}: {}", count(name));
    }

    let mut batch_kernel = RemusKernel::new();
    let batch_blank = batch_kernel.make_cylinder_solid(3.0, 6.0).unwrap();
    let batch_face = side_face(&batch_kernel, batch_blank);
    let batch = cut_batch(&mut batch_kernel, batch_blank, batch_face, &spec);
    assert_eq!(batch["ok"], direct);
}

/// NPT 1/4-18 on a 0.540 in pipe, from the pipe's end to the hand-tight
/// plane, where ASME B1.20.1 gives the pitch diameter `E1`.
fn npt_quarter() -> Value {
    let pitch = 25.4 / 18.0;
    let depth = 0.8 * pitch;
    let length = 0.2278 * 25.4;
    json!({
        "pitch": pitch,
        "depth": depth,
        "crestWidth": 0.5 * (pitch - 2.0 * depth * 30.0_f64.to_radians().tan()),
        "upperFlankAngle": 30.0,
        "lowerFlankAngle": 30.0,
        "majorDiameter": 0.49163 * 25.4 + depth,
        "length": length,
        "taper": { "ratio": 0.0625, "smallEnd": "start", "gaugeOffset": length }
    })
}

#[test]
fn tapered_pipe_thread_matches_between_direct_and_batch() {
    let spec = npt_quarter();
    let radius = 0.5 * 0.540 * 25.4;
    let mut direct_kernel = RemusKernel::new();
    let blank = direct_kernel.make_cylinder_solid(radius, 12.0).unwrap();
    let face = side_face(&direct_kernel, blank);
    let direct =
        serde_json::to_value(direct_kernel.cut_thread_detailed_impl(blank, face, &spec)).unwrap();
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    let geometry = &details["geometry"];
    let number = |key: &str| geometry[key].as_f64().unwrap();
    // 1 in 16 on the diameter: a half-angle of 1.7899°.
    assert!((number("taperAngle") - (1.0_f64 / 32.0).atan().to_degrees()).abs() < 1e-12);
    let depth = 0.8 * 25.4 / 18.0;
    assert!((number("majorDiameter") - (0.49163 * 25.4 + depth)).abs() < 1e-12);
    assert!((number("majorDiameterAtEnd") - number("majorDiameter")).abs() < 1e-12);
    // The pitch diameter at the pipe's end is the standard's E0, to the
    // table's 1e-5 in.
    let at_start = 0.5 * (number("majorDiameterAtStart") + number("minorDiameterAtStart"));
    assert!(
        (at_start - 0.47739 * 25.4).abs() < 0.6e-5 * 25.4,
        "{at_start}"
    );
    let solid = handle(&direct["value"]);
    let id = direct_kernel.resolve_solid(solid).unwrap();
    let report = remus_operations::validate::validate_solid(direct_kernel.topo(), id).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    // Crest and root lands are cones.
    let roles = details["faceRoles"].as_array().unwrap();
    let mut lands = 0;
    for entry in roles {
        if entry["role"] == "crest" || entry["role"] == "root" {
            lands += 1;
            let id = direct_kernel.resolve_face(handle(&entry["face"])).unwrap();
            assert!(matches!(
                direct_kernel.topo().face(id).unwrap().surface(),
                remus_topology::face::FaceSurface::Cone(_)
            ));
        }
    }
    assert!(lands >= 8, "{lands}");

    let mut batch_kernel = RemusKernel::new();
    let batch_blank = batch_kernel.make_cylinder_solid(radius, 12.0).unwrap();
    let batch_face = side_face(&batch_kernel, batch_blank);
    let batch = cut_batch(&mut batch_kernel, batch_blank, batch_face, &spec);
    assert_eq!(batch["ok"], direct);
}

/// The same `chamfer` end on a thread that stops short of the face's end
/// builds a runout, and `runout` asks for one by name.
#[test]
fn chamfer_short_of_the_face_end_is_a_runout_direct_and_batch() {
    for end in ["chamfer", "runout"] {
        let mut spec = m6();
        spec["length"] = json!(4.0);
        spec["end"] = json!({ "kind": end, "angle": 45.0 });
        let mut direct_kernel = RemusKernel::new();
        let blank = direct_kernel.make_cylinder_solid(3.0, 6.0).unwrap();
        let face = side_face(&direct_kernel, blank);
        let direct =
            serde_json::to_value(direct_kernel.cut_thread_detailed_impl(blank, face, &spec))
                .unwrap();
        assert_eq!(direct["status"], "ok", "{direct}");
        let details = &direct["details"];
        let finish = &details["geometry"]["end"];
        assert_eq!(finish["kind"], "runout", "{end}");
        // 45°: the runout reaches one thread depth into the thread.
        assert!((finish["length"].as_f64().unwrap() - 0.541_265_877_365_273_1).abs() < 1e-12);
        let solid = handle(&direct["value"]);
        let id = direct_kernel.resolve_solid(solid).unwrap();
        let report = remus_operations::validate::validate_solid(direct_kernel.topo(), id).unwrap();
        assert!(report.is_valid(), "{:?}", report.issues);
        let faces = faces_of(&direct_kernel, solid);
        let roles = details["faceRoles"].as_array().unwrap();
        let pieces: Vec<_> = roles
            .iter()
            .filter(|entry| entry["role"] == "runout.end")
            .collect();
        assert!(pieces.len() >= 3, "{}", pieces.len());
        for entry in &pieces {
            let face = handle(&entry["face"]);
            assert!(faces.contains(&face));
            let id = direct_kernel.resolve_face(face).unwrap();
            assert!(matches!(
                direct_kernel.topo().face(id).unwrap().surface(),
                remus_topology::face::FaceSurface::Cone(_)
            ));
        }
        assert!(roles.iter().all(|entry| entry["role"] != "chamfer.end"));

        let mut batch_kernel = RemusKernel::new();
        let batch_blank = batch_kernel.make_cylinder_solid(3.0, 6.0).unwrap();
        let batch_face = side_face(&batch_kernel, batch_blank);
        let batch = cut_batch(&mut batch_kernel, batch_blank, batch_face, &spec);
        assert_eq!(batch["ok"], direct, "{end}");
    }
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
            with(json!({
                "offset": 1.0, "length": 2.0,
                "start": { "kind": "chamfer", "angle": 15.0 },
                "end": { "kind": "chamfer", "angle": 15.0 }
            })),
            "thread:ends-overlap",
        ),
        (
            face,
            with(json!({
                "offset": 1.0, "length": 2.0,
                "start": { "kind": "runout", "angle": 15.0 },
                "end": { "kind": "plain" }
            })),
            "thread:finish-too-long",
        ),
        (
            face,
            with(json!({ "end": { "kind": "runout", "angle": 90.0 } })),
            "thread:invalid-parameter",
        ),
        (
            face,
            with(json!({ "end": { "kind": "relief", "angle": 45.0 } })),
            "invalid_argument",
        ),
        (
            face,
            with(json!({ "taper": { "ratio": 0.5, "smallEnd": "start" } })),
            "thread:taper-out-of-range",
        ),
        (
            face,
            with(json!({ "taper": { "ratio": 0.0, "smallEnd": "end" } })),
            "thread:taper-out-of-range",
        ),
        // The major diameter on a gauge plane below the top passes the boss
        // at the large end.
        (
            face,
            with(json!({
                "majorDiameter": 6.0,
                "taper": { "ratio": 0.0625, "smallEnd": "start" }
            })),
            "thread:cylinder-too-small",
        ),
        (
            face,
            with(json!({ "taper": { "ratio": 0.0625, "smallEnd": "top" } })),
            "invalid_argument",
        ),
        (
            face,
            with(json!({ "taper": { "ratio": 0.0625 } })),
            "invalid_argument",
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
