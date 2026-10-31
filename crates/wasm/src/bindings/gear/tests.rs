//! Contract tests for `makeGearDetailed`, direct and batch.
//!
//! The direct side runs `make_gear_detailed_impl` (a `JsError` cannot be
//! built off-wasm); the batch side goes through `execute_batch_v2`, whose
//! `ok` value is the same envelope. Volumes are checked against a native
//! `make_gear` on its own topology, and refusals against unchanged session
//! counts.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use remus_operations::gear::{GearKind, GearParams, GearPlacement, make_gear};
use remus_topology::Topology;

use crate::kernel::RemusKernel;

fn envelope(kernel: &mut RemusKernel, spec: &Value) -> Value {
    serde_json::to_value(kernel.make_gear_detailed_impl(spec)).unwrap()
}

fn batch_v2(kernel: &mut RemusKernel, args: &Value) -> Value {
    let ops = json!([{ "op": "makeGearDetailed", "args": args }]);
    let results: Value = serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap();
    results[0].clone()
}

fn live_counts(kernel: &RemusKernel) -> (usize, usize, usize, usize) {
    let topo = kernel.topo();
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    )
}

fn native_volume(params: &GearParams) -> f64 {
    let mut topo = Topology::new();
    let outcome = make_gear(&mut topo, params, &GearPlacement::default()).unwrap();
    remus_operations::measure::solid_volume(&topo, outcome.solid, 1e-2).unwrap()
}

fn assert_valid(kernel: &RemusKernel, solid: u32) {
    let id = kernel.resolve_solid(solid).unwrap();
    let report = remus_operations::validate::validate_solid(kernel.topo(), id).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

#[test]
fn external_gear_reports_approximate_flanks_with_their_bound_and_dimensions() {
    let mut kernel = RemusKernel::new();
    let spec = json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0 });
    let result = envelope(&mut kernel, &spec);
    assert_eq!(result["status"], "ok", "{result}");
    let details = &result["details"];
    assert_eq!(details["quality"], "approximate");
    assert_eq!(details["approximated"], "involute flanks");
    let bound = details["flankDeviationBound"].as_f64().unwrap();
    assert!(bound > 0.0 && bound <= 2e-5, "{bound}");
    assert!(details["spansPerFlank"].as_u64().unwrap() >= 1);
    assert!((details["geometry"]["pitchDiameter"].as_f64().unwrap() - 40.0).abs() < 1e-12);
    assert!((details["geometry"]["tipDiameter"].as_f64().unwrap() - 44.0).abs() < 1e-12);
    assert_eq!(details["warnings"], json!([]));

    let solid = handle(&result["value"]);
    let expected = native_volume(&GearParams::standard(GearKind::External, 2.0, 20, 5.0));
    let volume = kernel.volume(solid, 1e-2).unwrap();
    assert!(
        ((volume - expected) / expected).abs() < 1e-9,
        "{volume} vs {expected}"
    );
    assert_valid(&kernel, solid);
}

#[test]
fn batch_and_direct_results_are_identical() {
    let spec = json!({
        "kind": "external", "module": 1.5, "teeth": 13, "faceWidth": 4.0,
        "pressureAngle": 25.0, "profileShift": 0.2, "backlash": 0.05,
        "boreDiameter": 6.0, "origin": [1.0, 2.0, 3.0], "axis": [0.0, 1.0, 0.0],
        "xDirection": [1.0, 0.0, 0.0], "phase": 7.5
    });
    let mut direct_kernel = RemusKernel::new();
    let direct = envelope(&mut direct_kernel, &spec);
    let mut batch_kernel = RemusKernel::new();
    let batch = batch_v2(&mut batch_kernel, &spec);
    assert_eq!(direct["status"], "ok", "{direct}");
    assert_eq!(batch["ok"], direct);
    assert_eq!(live_counts(&direct_kernel), live_counts(&batch_kernel));
}

#[test]
fn rack_is_exact_and_internal_gear_needs_its_rim() {
    let mut kernel = RemusKernel::new();
    let rack = envelope(
        &mut kernel,
        &json!({ "kind": "rack", "module": 1.0, "teeth": 9, "faceWidth": 3.0 }),
    );
    assert_eq!(rack["status"], "ok", "{rack}");
    assert_eq!(rack["details"]["quality"], "exact");
    assert!(rack["details"].get("approximated").is_none());
    assert_eq!(rack["details"]["flankDeviationBound"], 0.0);
    assert!(rack["details"]["geometry"]["pitchDiameter"].is_null());

    let before = live_counts(&kernel);
    let missing_rim = envelope(
        &mut kernel,
        &json!({ "kind": "internal", "module": 1.0, "teeth": 40, "faceWidth": 3.0 }),
    );
    assert_eq!(missing_rim["status"], "error");
    assert_eq!(missing_rim["code"], "gear:invalid-parameter");
    assert_eq!(missing_rim["category"], "invalid_input");
    assert_eq!(live_counts(&kernel), before);

    let ring = envelope(
        &mut kernel,
        &json!({ "kind": "internal", "module": 1.0, "teeth": 40, "faceWidth": 3.0, "rimDiameter": 52.0 }),
    );
    assert_eq!(ring["status"], "ok", "{ring}");
    assert_valid(&kernel, handle(&ring["value"]));
}

#[test]
fn profile_refusals_carry_dimensions_and_leave_the_session_unchanged() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);
    let pointed = json!({
        "kind": "external", "module": 2.0, "teeth": 8, "faceWidth": 5.0, "profileShift": 1.2
    });
    let result = envelope(&mut kernel, &pointed);
    assert_eq!(result["status"], "error", "{result}");
    assert_eq!(result["code"], "gear:pointed-tip");
    assert!(result["value"].is_null());
    assert!(
        result["details"]["geometry"]["tipThickness"]
            .as_f64()
            .unwrap()
            <= 0.0
    );
    assert_eq!(result["details"]["operation"], "makeGear");
    assert_eq!(live_counts(&kernel), before);

    let batch = batch_v2(&mut kernel, &pointed);
    assert_eq!(batch["ok"], result);

    let bore = envelope(
        &mut kernel,
        &json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "boreDiameter": 36.0 }),
    );
    assert_eq!(bore["code"], "gear:bore-too-large");
    assert_eq!(bore["details"]["argument"], "boreDiameter");
    assert_eq!(live_counts(&kernel), before);
}

#[test]
fn undercut_warning_names_the_avoiding_shift() {
    let mut kernel = RemusKernel::new();
    let result = envelope(
        &mut kernel,
        &json!({ "kind": "external", "module": 1.0, "teeth": 10, "faceWidth": 2.0 }),
    );
    assert_eq!(result["status"], "ok", "{result}");
    let warnings = result["details"]["warnings"].as_array().unwrap();
    let undercut = warnings.iter().find(|w| w["code"] == "undercut").unwrap();
    let expected = 1.0 - 5.0 * 20.0_f64.to_radians().sin().powi(2);
    assert!((undercut["minProfileShift"].as_f64().unwrap() - expected).abs() < 1e-12);
}

#[test]
fn malformed_specs_refuse_as_invalid_arguments() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);
    for spec in [
        json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "boreDiamter": 4.0 }),
        json!({ "kind": "external", "module": 2.0, "faceWidth": 5.0 }),
        json!({ "kind": "external", "module": 2.0, "teeth": 20.5, "faceWidth": 5.0 }),
        json!({ "kind": "helical", "module": 2.0, "teeth": 20, "faceWidth": 5.0 }),
        json!({ "kind": "external", "module": "2", "teeth": 20, "faceWidth": 5.0 }),
        json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "axis": [0.0, 1.0] }),
        json!([1, 2, 3]),
    ] {
        let result = envelope(&mut kernel, &spec);
        assert_eq!(result["status"], "error", "{spec}: {result}");
        assert_eq!(result["code"], "invalid_argument", "{spec}: {result}");
        assert_eq!(result["category"], "invalid_input");
        let batch = batch_v2(&mut kernel, &spec);
        assert_eq!(batch["ok"], result, "{spec}");
    }
    let zero_module = envelope(
        &mut kernel,
        &json!({ "kind": "external", "module": 0.0, "teeth": 20, "faceWidth": 5.0 }),
    );
    assert_eq!(zero_module["code"], "gear:invalid-parameter");
    assert!(zero_module["details"].get("geometry").is_none());
    assert_eq!(live_counts(&kernel), before);
}

fn faces_of(kernel: &RemusKernel, solid: u32) -> Vec<u32> {
    let id = kernel.resolve_solid(solid).unwrap();
    remus_topology::explorer::solid_faces(kernel.topo(), id)
        .unwrap()
        .into_iter()
        .map(crate::handles::face_id_to_u32)
        .collect()
}

/// The boss cylinder face of a `makeCylinder` solid.
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

#[test]
fn new_gears_name_every_face_once() {
    let mut kernel = RemusKernel::new();
    let result = envelope(
        &mut kernel,
        &json!({ "kind": "external", "module": 1.0, "teeth": 12, "faceWidth": 2.0, "boreDiameter": 3.0 }),
    );
    assert_eq!(result["status"], "ok", "{result}");
    let roles = result["details"]["faceRoles"].as_array().unwrap();
    let faces = faces_of(&kernel, handle(&result["value"]));
    assert_eq!(roles.len(), faces.len());
    let named: std::collections::BTreeSet<_> = roles.iter().map(|r| handle(&r["face"])).collect();
    assert_eq!(named, faces.iter().copied().collect());
    let names: std::collections::BTreeSet<_> =
        roles.iter().map(|r| r["role"].as_str().unwrap()).collect();
    assert_eq!(names.len(), roles.len());
    assert!(names.contains("tooth[11].flank.plus") && names.contains("bore"));
    assert!((result["details"]["geometry"]["module"].as_f64().unwrap() - 1.0).abs() < 1e-15);
}

fn cut_batch(kernel: &mut RemusKernel, solid: u32, face: u32, spec: &Value) -> Value {
    let ops = json!([{ "op": "cutGearTeethDetailed", "args": { "solid": solid, "face": face, "spec": spec } }]);
    let results: Value = serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap();
    results[0].clone()
}

#[test]
fn teeth_on_a_boss_match_between_direct_and_batch_and_solve_the_module() {
    let spec = json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0, "phase": 4.5 });
    let mut direct_kernel = RemusKernel::new();
    let blank = direct_kernel.make_cylinder_solid(11.0, 6.0).unwrap();
    let face = side_face(&direct_kernel, blank);
    let direct =
        serde_json::to_value(direct_kernel.cut_gear_teeth_detailed_impl(blank, face, &spec))
            .unwrap();
    assert_eq!(direct["status"], "ok", "{direct}");
    assert!((direct["details"]["geometry"]["module"].as_f64().unwrap() - 1.0).abs() < 1e-12);
    assert!(
        (direct["details"]["geometry"]["tipDiameter"]
            .as_f64()
            .unwrap()
            - 22.0)
            .abs()
            < 1e-9
    );
    let roles = direct["details"]["faceRoles"].as_array().unwrap();
    assert_eq!(
        roles
            .iter()
            .filter(|r| r["role"].as_str().unwrap().contains(".flank."))
            .count(),
        40
    );
    let solid = handle(&direct["value"]);
    assert_ne!(solid, blank, "the input solid is kept");
    assert_valid(&direct_kernel, solid);

    let mut batch_kernel = RemusKernel::new();
    let batch_blank = batch_kernel.make_cylinder_solid(11.0, 6.0).unwrap();
    let batch_face = side_face(&batch_kernel, batch_blank);
    let batch = cut_batch(&mut batch_kernel, batch_blank, batch_face, &spec);
    assert_eq!(batch["ok"], direct);
}

/// Teeth over part of the face: `offset` places them, the collars stay, and
/// both entry points agree.
#[test]
fn teeth_over_the_middle_of_a_boss_keep_its_collars() {
    let spec = json!({ "kind": "external", "teeth": 22, "faceWidth": 3.0, "offset": 1.5 });
    let mut direct_kernel = RemusKernel::new();
    let blank = direct_kernel.make_cylinder_solid(12.0, 6.0).unwrap();
    let face = side_face(&direct_kernel, blank);
    let direct =
        serde_json::to_value(direct_kernel.cut_gear_teeth_detailed_impl(blank, face, &spec))
            .unwrap();
    assert_eq!(direct["status"], "ok", "{direct}");
    let solid = handle(&direct["value"]);
    assert_valid(&direct_kernel, solid);
    // Only the middle three of the six units lose their tooth spaces.
    let whole = std::f64::consts::PI * 144.0 * 6.0;
    let id = direct_kernel.resolve_solid(solid).unwrap();
    let volume = remus_operations::measure::solid_volume(direct_kernel.topo(), id, 1e-3).unwrap();
    assert!(
        volume < whole && volume > whole * 0.9,
        "{volume} of {whole}"
    );

    let mut batch_kernel = RemusKernel::new();
    let batch_blank = batch_kernel.make_cylinder_solid(12.0, 6.0).unwrap();
    let batch_face = side_face(&batch_kernel, batch_blank);
    let batch = cut_batch(&mut batch_kernel, batch_blank, batch_face, &spec);
    assert_eq!(batch["ok"], direct);
}

#[test]
fn cylinder_refusals_are_typed_and_atomic() {
    let mut kernel = RemusKernel::new();
    let blank = kernel.make_cylinder_solid(11.0, 6.0).unwrap();
    let face = side_face(&kernel, blank);
    let cap = faces_of(&kernel, blank)
        .into_iter()
        .find(|&f| f != face)
        .unwrap();
    let before = live_counts(&kernel);
    let cases = [
        (
            face,
            json!({ "kind": "external", "teeth": 20, "faceWidth": 4.0, "offset": 3.0 }),
            "gear:span-outside-face",
        ),
        (
            cap,
            json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0 }),
            "gear:not-a-cylinder",
        ),
        (
            face,
            json!({ "kind": "internal", "teeth": 40, "faceWidth": 6.0 }),
            "gear:invalid-parameter",
        ),
        (
            face,
            json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0, "fit": "module", "module": 1.2 }),
            "gear:cylinder-too-small",
        ),
        (
            face,
            json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0, "module": 1.0 }),
            "invalid_argument",
        ),
        (
            face,
            json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0, "fit": "module" }),
            "invalid_argument",
        ),
        (
            face,
            json!({ "kind": "rack", "teeth": 20, "faceWidth": 6.0 }),
            "invalid_argument",
        ),
        (
            face,
            json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0, "startOffset": 1.0 }),
            "invalid_argument",
        ),
    ];
    for (target, spec, code) in cases {
        let direct =
            serde_json::to_value(kernel.cut_gear_teeth_detailed_impl(blank, target, &spec))
                .unwrap();
        assert_eq!(direct["status"], "error", "{spec}: {direct}");
        assert_eq!(direct["code"], code, "{spec}: {direct}");
        assert_eq!(direct["details"]["operation"], "cutGearTeeth");
        assert_eq!(live_counts(&kernel), before, "{spec}");
        assert_eq!(
            cut_batch(&mut kernel, blank, target, &spec)["ok"],
            direct,
            "{spec}"
        );
    }
    let past = serde_json::to_value(kernel.cut_gear_teeth_detailed_impl(
        blank,
        face,
        &json!({ "kind": "external", "teeth": 20, "faceWidth": 4.0, "offset": 3.0 }),
    ))
    .unwrap();
    assert_eq!(past["details"]["argument"], "faceWidth");
    let missing = cut_batch(
        &mut kernel,
        blank,
        9999,
        &json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0 }),
    );
    assert_eq!(missing["ok"]["status"], "error");
}

#[test]
fn helical_gears_take_a_helix_angle_and_hand_direct_and_batch() {
    let spec = json!({
        "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 8.0,
        "helixAngle": 20.0, "hand": "left", "boreDiameter": 10.0
    });
    let mut direct_kernel = RemusKernel::new();
    let direct = envelope(&mut direct_kernel, &spec);
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    assert_eq!(details["quality"], "approximate");
    assert!(details["spansAlongAxis"].as_u64().unwrap() >= 1);
    let bound = details["flankDeviationBound"].as_f64().unwrap();
    assert!(bound > 0.0 && bound <= 2.02e-5, "{bound}");
    let beta = 20.0_f64.to_radians();
    let geometry = &details["geometry"];
    assert!((geometry["module"].as_f64().unwrap() - 2.0).abs() < 1e-12);
    assert!((geometry["transverseModule"].as_f64().unwrap() - 2.0 / beta.cos()).abs() < 1e-12);
    assert!((geometry["helixAngle"].as_f64().unwrap() - 20.0).abs() < 1e-12);
    let d = 40.0 / beta.cos();
    assert!((geometry["pitchDiameter"].as_f64().unwrap() - d).abs() < 1e-12);
    assert!(
        (geometry["lead"].as_f64().unwrap() - std::f64::consts::PI * d / beta.tan()).abs() < 1e-9
    );
    assert_valid(&direct_kernel, handle(&direct["value"]));

    let mut batch_kernel = RemusKernel::new();
    let batch = batch_v2(&mut batch_kernel, &spec);
    assert_eq!(batch["ok"], direct);

    // A spur gear reports no lead; bad helix specs refuse.
    let spur = envelope(
        &mut direct_kernel,
        &json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0 }),
    );
    assert_eq!(spur["details"]["geometry"]["lead"], Value::Null);
    assert_eq!(spur["details"]["spansAlongAxis"], 0);
    let before = live_counts(&direct_kernel);
    let steep = envelope(
        &mut direct_kernel,
        &json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "helixAngle": 50.0 }),
    );
    assert_eq!(steep["code"], "gear:invalid-parameter", "{steep}");
    let bad_hand = envelope(
        &mut direct_kernel,
        &json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "hand": "up" }),
    );
    assert_eq!(bad_hand["code"], "invalid_argument", "{bad_hand}");
    assert_eq!(live_counts(&direct_kernel), before);
}

#[test]
fn helical_teeth_on_a_boss_solve_the_normal_module() {
    let spec = json!({ "kind": "external", "teeth": 20, "faceWidth": 6.0, "helixAngle": 15.0 });
    let mut kernel = RemusKernel::new();
    let blank = kernel.make_cylinder_solid(11.0, 6.0).unwrap();
    let face = side_face(&kernel, blank);
    let direct =
        serde_json::to_value(kernel.cut_gear_teeth_detailed_impl(blank, face, &spec)).unwrap();
    assert_eq!(direct["status"], "ok", "{direct}");
    let expected = 22.0 / (20.0 / 15.0_f64.to_radians().cos() + 2.0);
    assert!((direct["details"]["geometry"]["module"].as_f64().unwrap() - expected).abs() < 1e-12);
    assert!(
        (direct["details"]["geometry"]["tipDiameter"]
            .as_f64()
            .unwrap()
            - 22.0)
            .abs()
            < 1e-9
    );
    assert_valid(&kernel, handle(&direct["value"]));

    let mut batch_kernel = RemusKernel::new();
    let batch_blank = batch_kernel.make_cylinder_solid(11.0, 6.0).unwrap();
    let batch_face = side_face(&batch_kernel, batch_blank);
    let batch = cut_batch(&mut batch_kernel, batch_blank, batch_face, &spec);
    assert_eq!(batch["ok"], direct);
}

#[test]
fn herringbone_gears_name_both_halves_direct_and_batch() {
    let spec = json!({
        "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 8.0,
        "helixAngle": 20.0, "herringbone": true
    });
    let mut direct_kernel = RemusKernel::new();
    let direct = envelope(&mut direct_kernel, &spec);
    assert_eq!(direct["status"], "ok", "{direct}");
    let roles: Vec<&str> = direct["details"]["faceRoles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["role"].as_str().unwrap())
        .collect();
    for name in [
        "cap.start",
        "cap.end",
        "tooth[0].flank.plus.start-half",
        "tooth[0].flank.plus.end-half",
        "root[19].end-half",
    ] {
        assert!(roles.contains(&name), "{name}: {roles:?}");
    }
    assert_valid(&direct_kernel, handle(&direct["value"]));

    let mut batch_kernel = RemusKernel::new();
    let batch = batch_v2(&mut batch_kernel, &spec);
    assert_eq!(batch["ok"], direct);

    // Without a helix angle there is no herringbone, and the session is kept.
    let before = live_counts(&direct_kernel);
    let flat = envelope(
        &mut direct_kernel,
        &json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "herringbone": true }),
    );
    assert_eq!(flat["code"], "gear:invalid-parameter", "{flat}");
    assert_eq!(live_counts(&direct_kernel), before);
}

fn bevel_envelope(kernel: &mut RemusKernel, spec: &Value) -> Value {
    serde_json::to_value(kernel.make_bevel_gear_detailed_impl(spec)).unwrap()
}

fn bevel_batch(kernel: &mut RemusKernel, args: &Value) -> Value {
    let ops = json!([{ "op": "makeBevelGearDetailed", "args": args }]);
    let results: Value = serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap();
    results[0].clone()
}

#[test]
fn bevel_gears_report_their_cones_and_roles_direct_and_batch() {
    let spec = json!({
        "module": 2.0, "teeth": 20, "mateTeeth": 40, "faceWidth": 8.0, "boreDiameter": 10.0
    });
    let mut direct_kernel = RemusKernel::new();
    let direct = bevel_envelope(&mut direct_kernel, &spec);
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    assert_eq!(details["quality"], "approximate");
    assert_eq!(details["approximated"], "spherical involute flanks");
    let bound = details["flankDeviationBound"].as_f64().unwrap();
    assert!(bound > 0.0 && bound <= 1e-5 * 2.0 * 1.01, "{bound}");
    assert!(details["spansPerFlank"].as_u64().unwrap() >= 1);
    // Right-angle pair at 20:40: tan δ = 1/2, degrees at this boundary.
    let geometry = &details["geometry"];
    let delta = 0.5_f64.atan().to_degrees();
    assert!((geometry["pitchAngle"].as_f64().unwrap() - delta).abs() < 1e-12);
    assert!((geometry["matePitchAngle"].as_f64().unwrap() - (90.0 - delta)).abs() < 1e-12);
    assert!((geometry["pitchDiameter"].as_f64().unwrap() - 40.0).abs() < 1e-12);
    let a = 20.0 / 0.5_f64.atan().sin();
    assert!((geometry["outerConeDistance"].as_f64().unwrap() - a).abs() < 1e-12);

    let roles: Vec<&str> = details["faceRoles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["role"].as_str().unwrap())
        .collect();
    for name in [
        "cap.start",
        "cap.end",
        "bore",
        "tooth[0].flank.plus",
        "tooth[19].flank.minus",
        "tooth[0].tip",
        "root[19]",
    ] {
        assert!(roles.contains(&name), "{name}: {roles:?}");
    }
    let solid = handle(&direct["value"]);
    assert_eq!(roles.len(), faces_of(&direct_kernel, solid).len());
    assert_valid(&direct_kernel, solid);

    let mut batch_kernel = RemusKernel::new();
    let batch = bevel_batch(&mut batch_kernel, &spec);
    assert_eq!(batch["ok"], direct);
}

#[test]
fn bevel_refusals_carry_cone_dimensions_and_keep_the_session() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);

    let bore = json!({
        "module": 2.0, "teeth": 20, "mateTeeth": 40, "faceWidth": 8.0, "boreDiameter": 40.0
    });
    let result = bevel_envelope(&mut kernel, &bore);
    assert_eq!(result["status"], "error", "{result}");
    assert_eq!(result["code"], "gear:bore-too-large");
    assert_eq!(result["details"]["argument"], "boreDiameter");
    assert_eq!(result["details"]["operation"], "makeBevelGear");
    assert!(
        result["details"]["geometry"]["rootDiameter"]
            .as_f64()
            .unwrap()
            > 0.0
    );
    assert!(result["value"].is_null());
    assert_eq!(live_counts(&kernel), before);
    assert_eq!(bevel_batch(&mut kernel, &bore)["ok"], result);

    // A face width reaching the apex has no cone dimensions to show.
    let long = bevel_envelope(
        &mut kernel,
        &json!({ "module": 2.0, "teeth": 20, "mateTeeth": 40, "faceWidth": 60.0 }),
    );
    assert_eq!(long["code"], "gear:invalid-parameter", "{long}");
    assert!(long["details"].get("geometry").is_none());

    for spec in [
        json!({ "module": 2.0, "teeth": 20, "faceWidth": 8.0 }),
        json!({ "module": 2.0, "teeth": 20, "mateTeeth": 40, "faceWidth": 8.0, "helixAngle": 10.0 }),
        json!({ "module": 2.0, "teeth": 20, "mateTeeth": 40.5, "faceWidth": 8.0 }),
    ] {
        let malformed = bevel_envelope(&mut kernel, &spec);
        assert_eq!(malformed["code"], "invalid_argument", "{spec}: {malformed}");
        assert_eq!(bevel_batch(&mut kernel, &spec)["ok"], malformed, "{spec}");
    }
    assert_eq!(live_counts(&kernel), before);
}

fn worm_envelope(kernel: &mut RemusKernel, op: &str, spec: &Value) -> Value {
    let result = match op {
        "makeWormDetailed" => kernel.make_worm_detailed_impl(spec),
        _ => kernel.make_worm_wheel_detailed_impl(spec),
    };
    serde_json::to_value(result).unwrap()
}

fn worm_batch(kernel: &mut RemusKernel, op: &str, args: &Value) -> Value {
    let ops = json!([{ "op": op, "args": args }]);
    let results: Value = serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap();
    results[0].clone()
}

#[test]
fn worms_and_their_wheels_report_dimensions_and_roles_direct_and_batch() {
    let worm = json!({
        "module": 2.0, "starts": 2, "diameterFactor": 10.0, "length": 12.0,
        "boreDiameter": 6.0, "hand": "left"
    });
    let mut direct_kernel = RemusKernel::new();
    let direct = worm_envelope(&mut direct_kernel, "makeWormDetailed", &worm);
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    assert_eq!(details["quality"], "approximate");
    assert_eq!(details["approximated"], "involute helicoid flanks");
    let bound = details["flankDeviationBound"].as_f64().unwrap();
    assert!(bound > 0.0 && bound <= 1e-5 * 2.0 * 1.01, "{bound}");
    assert!(details["spansAlongAxis"].as_u64().unwrap() >= 1);
    // tan γ = z₁/q = 0.2, in degrees at this boundary.
    let geometry = &details["geometry"];
    let gamma = 0.2_f64.atan();
    assert!((geometry["leadAngle"].as_f64().unwrap() - gamma.to_degrees()).abs() < 1e-12);
    assert!((geometry["pitchDiameter"].as_f64().unwrap() - 20.0).abs() < 1e-12);
    assert!((geometry["lead"].as_f64().unwrap() - 4.0 * std::f64::consts::PI).abs() < 1e-12);
    let roles: Vec<&str> = details["faceRoles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["role"].as_str().unwrap())
        .collect();
    for name in [
        "cap.start",
        "cap.end",
        "bore",
        "tooth[1].flank.plus",
        "root[0]",
    ] {
        assert!(roles.contains(&name), "{name}: {roles:?}");
    }
    let solid = handle(&direct["value"]);
    assert_eq!(roles.len(), faces_of(&direct_kernel, solid).len());
    assert_valid(&direct_kernel, solid);
    let mut batch_kernel = RemusKernel::new();
    assert_eq!(
        worm_batch(&mut batch_kernel, "makeWormDetailed", &worm)["ok"],
        direct
    );

    let wheel = json!({
        "module": 2.0, "starts": 2, "diameterFactor": 10.0, "hand": "left",
        "teeth": 30, "faceWidth": 10.0, "boreDiameter": 12.0
    });
    let direct = worm_envelope(&mut direct_kernel, "makeWormWheelDetailed", &wheel);
    assert_eq!(direct["status"], "ok", "{direct}");
    let geometry = &direct["details"]["geometry"];
    assert!((geometry["helixAngle"].as_f64().unwrap() - gamma.to_degrees()).abs() < 1e-12);
    assert!((geometry["module"].as_f64().unwrap() - 2.0 * gamma.cos()).abs() < 1e-12);
    assert!((geometry["transverseModule"].as_f64().unwrap() - 2.0).abs() < 1e-12);
    assert!((geometry["pitchDiameter"].as_f64().unwrap() - 60.0).abs() < 1e-12);
    assert!((geometry["centerDistance"].as_f64().unwrap() - 40.0).abs() < 1e-12);
    assert_valid(&direct_kernel, handle(&direct["value"]));
    assert_eq!(
        worm_batch(&mut batch_kernel, "makeWormWheelDetailed", &wheel)["ok"],
        direct
    );
}

#[test]
fn worm_refusals_carry_dimensions_and_keep_the_session() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);
    let bored = json!({
        "module": 2.0, "starts": 1, "diameterFactor": 10.0, "length": 10.0, "boreDiameter": 16.0
    });
    let result = worm_envelope(&mut kernel, "makeWormDetailed", &bored);
    assert_eq!(result["code"], "gear:bore-too-large", "{result}");
    assert_eq!(result["details"]["argument"], "boreDiameter");
    assert_eq!(result["details"]["operation"], "makeWorm");
    assert!(
        (result["details"]["geometry"]["rootDiameter"]
            .as_f64()
            .unwrap()
            - 15.0)
            .abs()
            < 1e-12
    );
    assert!(result["value"].is_null());
    assert_eq!(
        worm_batch(&mut kernel, "makeWormDetailed", &bored)["ok"],
        result
    );

    // A lead angle past 45° has no worm to show.
    let steep = json!({ "module": 2.0, "starts": 4, "diameterFactor": 3.0, "length": 10.0 });
    let result = worm_envelope(&mut kernel, "makeWormDetailed", &steep);
    assert_eq!(result["code"], "gear:invalid-parameter", "{result}");
    assert!(result["details"].get("geometry").is_none());

    let few = json!({
        "module": 2.0, "starts": 1, "diameterFactor": 10.0, "teeth": 2, "faceWidth": 8.0
    });
    let result = worm_envelope(&mut kernel, "makeWormWheelDetailed", &few);
    assert_eq!(result["code"], "gear:invalid-parameter", "{result}");
    assert_eq!(result["details"]["operation"], "makeWormWheel");

    for (op, spec) in [
        (
            "makeWormDetailed",
            json!({ "module": 2.0, "starts": 1, "diameterFactor": 10.0, "length": 10.0, "teeth": 3 }),
        ),
        (
            "makeWormWheelDetailed",
            json!({ "module": 2.0, "starts": 1, "diameterFactor": 10.0, "teeth": 30, "faceWidth": 8.0, "length": 4.0 }),
        ),
        (
            "makeWormDetailed",
            json!({ "module": 2.0, "starts": 1.5, "diameterFactor": 10.0, "length": 10.0 }),
        ),
    ] {
        let malformed = worm_envelope(&mut kernel, op, &spec);
        assert_eq!(malformed["code"], "invalid_argument", "{spec}: {malformed}");
        assert_eq!(
            worm_batch(&mut kernel, op, &spec)["ok"],
            malformed,
            "{spec}"
        );
    }
    assert_eq!(live_counts(&kernel), before);
}

#[test]
fn throated_worm_wheels_report_their_throat_direct_and_batch() {
    let spec = json!({
        "module": 2.0, "starts": 1, "diameterFactor": 10.0,
        "teeth": 30, "faceWidth": 14.0, "throated": true, "boreDiameter": 12.0
    });
    let mut direct_kernel = RemusKernel::new();
    let direct = worm_envelope(&mut direct_kernel, "makeWormWheelDetailed", &spec);
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    let curve = details["throatCurveBound"].as_f64().unwrap();
    assert!(curve > 0.0 && curve <= 1e-5 * 2.0, "{curve}");
    // a = 40, throat diameter 64, outside diameter 66 by default, ρ = 8.
    let geometry = &details["geometry"];
    for (key, value) in [
        ("centerDistance", 40.0),
        ("throatDiameter", 64.0),
        ("outsideDiameter", 66.0),
        ("throatRadius", 8.0),
        ("tipDiameter", 66.0),
    ] {
        assert!(
            (geometry[key].as_f64().unwrap() - value).abs() < 1e-12,
            "{key}"
        );
    }
    let width = 2.0 * (64.0_f64 - 49.0).sqrt();
    assert!((geometry["throatWidth"].as_f64().unwrap() - width).abs() < 1e-12);
    let roles: Vec<&str> = details["faceRoles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["role"].as_str().unwrap())
        .collect();
    for name in [
        "tooth[0].throat",
        "tooth[29].tip.start-half",
        "tooth[7].tip.end-half",
        "bore",
    ] {
        assert!(roles.contains(&name), "{name}");
    }
    let solid = handle(&direct["value"]);
    assert_eq!(roles.len(), faces_of(&direct_kernel, solid).len());
    assert_valid(&direct_kernel, solid);
    let mut batch_kernel = RemusKernel::new();
    assert_eq!(
        worm_batch(&mut batch_kernel, "makeWormWheelDetailed", &spec)["ok"],
        direct
    );

    let before = live_counts(&batch_kernel);
    let narrow = json!({
        "module": 2.0, "starts": 1, "diameterFactor": 10.0,
        "teeth": 30, "faceWidth": 7.0, "throated": true
    });
    let refused = worm_envelope(&mut batch_kernel, "makeWormWheelDetailed", &narrow);
    assert_eq!(refused["code"], "gear:invalid-parameter", "{refused}");
    let plain = json!({
        "module": 2.0, "starts": 1, "diameterFactor": 10.0,
        "teeth": 30, "faceWidth": 14.0, "outsideDiameter": 66.0
    });
    let refused = worm_envelope(&mut batch_kernel, "makeWormWheelDetailed", &plain);
    assert_eq!(refused["code"], "invalid_argument", "{refused}");
    assert_eq!(refused["details"]["argument"], "outsideDiameter");
    assert_eq!(live_counts(&batch_kernel), before);
}

fn enveloping_batch(kernel: &mut RemusKernel, args: &Value) -> Value {
    worm_batch(kernel, "makeEnvelopingWormDetailed", args)
}

#[test]
fn enveloping_worms_report_dimensions_and_roles_direct_and_batch() {
    let spec = json!({
        "module": 2.0, "starts": 1, "wheelTeeth": 30, "diameterFactor": 10.0,
        "length": 16.0, "boreDiameter": 6.0, "hand": "left"
    });
    let mut direct_kernel = RemusKernel::new();
    let direct =
        serde_json::to_value(direct_kernel.make_enveloping_worm_detailed_impl(&spec)).unwrap();
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    assert_eq!(details["quality"], "approximate");
    assert_eq!(details["approximated"], "Hindley thread flanks");
    let bound = details["flankDeviationBound"].as_f64().unwrap();
    assert!(bound > 0.0 && bound <= 1e-5 * 2.0 * 1.01, "{bound}");
    assert!(details["spansAlongFlank"].as_u64().unwrap() >= 1);
    let geometry = &details["geometry"];
    assert!((geometry["centerDistance"].as_f64().unwrap() - 40.0).abs() < 1e-12);
    assert!((geometry["ratio"].as_f64().unwrap() - 30.0).abs() < 1e-12);
    assert!((geometry["leadAngle"].as_f64().unwrap() - 0.1_f64.atan().to_degrees()).abs() < 1e-12);
    assert!((geometry["throatTipDiameter"].as_f64().unwrap() - 24.0).abs() < 1e-12);
    // The wheel sits on the x direction at the center distance from the
    // throat, half way along the worm.
    let center = &details["wheelCenter"];
    assert!((center[0].as_f64().unwrap() - 40.0).abs() < 1e-12);
    assert!((center[2].as_f64().unwrap() - 8.0).abs() < 1e-12);
    let roles: Vec<&str> = details["faceRoles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["role"].as_str().unwrap())
        .collect();
    for name in [
        "cap.start",
        "cap.end",
        "bore",
        "tooth[0].flank.plus[0]",
        "tooth[0].flank.minus[0]",
        "tooth[0].tip[0]",
        "root[0][0]",
    ] {
        assert!(roles.contains(&name), "{name}: {roles:?}");
    }
    let solid = handle(&direct["value"]);
    assert_eq!(roles.len(), faces_of(&direct_kernel, solid).len());
    assert_valid(&direct_kernel, solid);
    let mut batch_kernel = RemusKernel::new();
    assert_eq!(enveloping_batch(&mut batch_kernel, &spec)["ok"], direct);
}

#[test]
fn enveloping_worm_refusals_carry_dimensions_and_keep_the_session() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);
    let bored = json!({
        "module": 2.0, "starts": 1, "wheelTeeth": 30, "diameterFactor": 10.0,
        "length": 16.0, "boreDiameter": 15.0
    });
    let result = serde_json::to_value(kernel.make_enveloping_worm_detailed_impl(&bored)).unwrap();
    assert_eq!(result["code"], "gear:bore-too-large", "{result}");
    assert_eq!(result["details"]["argument"], "boreDiameter");
    assert_eq!(result["details"]["operation"], "makeEnvelopingWorm");
    assert!(
        (result["details"]["geometry"]["throatRootDiameter"]
            .as_f64()
            .unwrap()
            - 15.0)
            .abs()
            < 1e-12
    );
    assert_eq!(enveloping_batch(&mut kernel, &bored)["ok"], result);
    // Longer than its tip hourglass: no worm to show.
    let long = json!({
        "module": 2.0, "starts": 1, "wheelTeeth": 30, "diameterFactor": 10.0, "length": 60.0
    });
    let result = serde_json::to_value(kernel.make_enveloping_worm_detailed_impl(&long)).unwrap();
    assert_eq!(result["code"], "gear:invalid-parameter", "{result}");
    assert!(result["details"].get("geometry").is_none());
    // A misspelled key refuses rather than falling back to a default.
    let typo = json!({
        "module": 2.0, "starts": 1, "wheelTeeth": 30, "diameterFactor": 10.0,
        "length": 16.0, "wheelTooth": 30
    });
    let result = serde_json::to_value(kernel.make_enveloping_worm_detailed_impl(&typo)).unwrap();
    assert_eq!(result["code"], "invalid_argument", "{result}");
    assert_eq!(enveloping_batch(&mut kernel, &typo)["ok"], result);
    assert_eq!(live_counts(&kernel), before);
}

fn enveloping_wheel_batch(kernel: &mut RemusKernel, args: &Value) -> Value {
    worm_batch(kernel, "makeEnvelopingWormWheelDetailed", args)
}

#[test]
fn enveloping_wheels_report_dimensions_and_roles_direct_and_batch() {
    let spec = json!({
        "module": 2.0, "starts": 1, "wheelTeeth": 30, "diameterFactor": 10.0,
        "length": 24.0, "faceWidth": 8.0, "boreDiameter": 12.0
    });
    let mut direct_kernel = RemusKernel::new();
    let direct =
        serde_json::to_value(direct_kernel.make_enveloping_worm_wheel_detailed_impl(&spec))
            .unwrap();
    assert_eq!(direct["status"], "ok", "{direct}");
    let details = &direct["details"];
    assert_eq!(details["quality"], "approximate");
    assert_eq!(
        details["approximated"],
        "Hindley wheel flanks off the mid-plane"
    );
    let bound = details["flankDeviationBound"].as_f64().unwrap();
    assert!(bound > 0.0 && bound <= 2.0 * 1e-5 * 2.0 * 1.01, "{bound}");
    let relief = details["relief"].as_f64().unwrap();
    assert!((0.0..0.1).contains(&relief), "{relief}");
    let geometry = &details["geometry"];
    for (key, value) in [
        ("centerDistance", 40.0),
        ("pitchDiameter", 60.0),
        ("throatDiameter", 64.0),
        ("rootDiameter", 55.0),
        ("outsideDiameter", 66.0),
        ("throatRadius", 8.0),
    ] {
        assert!(
            (geometry[key].as_f64().unwrap() - value).abs() < 1e-12,
            "{key}"
        );
    }
    assert_eq!(geometry["flankTurn"].as_array().unwrap().len(), 2);
    // The worm sits on the wheel's x direction at the center distance,
    // half way across the face, its axis across the wheel's.
    let worm = &details["wormPlacement"];
    assert!((worm["origin"][0].as_f64().unwrap() - 40.0).abs() < 1e-12);
    assert!((worm["origin"][2].as_f64().unwrap() - 4.0).abs() < 1e-12);
    assert!(worm["axis"][2].as_f64().unwrap().abs() < 1e-12);
    let roles: Vec<&str> = details["faceRoles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["role"].as_str().unwrap())
        .collect();
    for name in [
        "cap.start",
        "cap.end",
        "bore",
        "tooth[0].throat",
        "tooth[0].tip.start-half",
        "tooth[0].flank.plus.start-half",
        "tooth[29].flank.minus.end-half",
        "root[0]",
    ] {
        assert!(roles.contains(&name), "{name}: {roles:?}");
    }
    let solid = handle(&direct["value"]);
    assert_eq!(roles.len(), faces_of(&direct_kernel, solid).len());
    assert_valid(&direct_kernel, solid);
    let mut batch_kernel = RemusKernel::new();
    assert_eq!(
        enveloping_wheel_batch(&mut batch_kernel, &spec)["ok"],
        direct
    );
}

#[test]
fn enveloping_wheel_refusals_carry_the_worm_and_keep_the_session() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);
    // The throat (7.75 wide) does not fit a 6 wide face.
    let narrow = json!({
        "module": 2.0, "starts": 1, "wheelTeeth": 30, "diameterFactor": 10.0,
        "length": 24.0, "faceWidth": 6.0
    });
    let result =
        serde_json::to_value(kernel.make_enveloping_worm_wheel_detailed_impl(&narrow)).unwrap();
    assert_eq!(result["code"], "gear:invalid-parameter", "{result}");
    assert_eq!(result["details"]["operation"], "makeEnvelopingWormWheel");
    assert!(
        (result["details"]["wormGeometry"]["centerDistance"]
            .as_f64()
            .unwrap()
            - 40.0)
            .abs()
            < 1e-12
    );
    assert_eq!(enveloping_wheel_batch(&mut kernel, &narrow)["ok"], result);
    // The worm's own backlash is not a wheel option.
    let typo = json!({
        "module": 2.0, "starts": 1, "wheelTeeth": 30, "diameterFactor": 10.0,
        "length": 24.0, "faceWidth": 8.0, "wormBacklash": 0.1
    });
    let result =
        serde_json::to_value(kernel.make_enveloping_worm_wheel_detailed_impl(&typo)).unwrap();
    assert_eq!(result["code"], "invalid_argument", "{result}");
    assert_eq!(enveloping_wheel_batch(&mut kernel, &typo)["ok"], result);
    assert_eq!(live_counts(&kernel), before);
}
