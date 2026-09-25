//! O4.7 fillet/chamfer Detailed twins: success parity, typed refusals,
//! batch-V2 code parity, precedence, and rollback.
//!
//! The twins reuse the exact production dispatch (`fillet_whole_selection`
//! for fillet, `try_chamfer` for chamfer) with the batch panic guard, so
//! they cannot drift into a v2-only path. Engine failures map through
//! `blend_failure` (native `kernelCode`); handle failures fall back to the
//! stable batch-v2 wire code. Scalar validation reaches the engines, matching
//! the batch path; the legacy direct methods validate scalars first via
//! `validate_positive`, so a combined invalid-handle + invalid-scalar fault
//! reports the handle here and the scalar there. That precedence difference
//! is pinned below, not hidden.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;

fn run(kernel: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&kernel.execute_batch(&json)).unwrap()
}

fn run_v2(kernel: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&kernel.execute_batch_v2(&json)).unwrap()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn make_box(kernel: &mut BrepKernel, w: f64, h: f64, d: f64) -> u32 {
    let out = run(
        kernel,
        &[op(
            "makeBox",
            serde_json::json!({"width": w, "height": h, "depth": d}),
        )],
    );
    u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap()
}

fn edge_handles(kernel: &mut BrepKernel, solid: u32) -> Vec<u32> {
    let out = run(
        kernel,
        &[op("solidEdges", serde_json::json!({"solid": solid}))],
    );
    out[0]["ok"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| u32::try_from(v.as_u64().unwrap()).unwrap())
        .collect()
}

fn volume(kernel: &mut BrepKernel, solid: u32) -> f64 {
    let out = run(
        kernel,
        &[op(
            "volume",
            serde_json::json!({"solid": solid, "deflection": 0.01}),
        )],
    );
    out[0]["ok"].as_f64().unwrap()
}

fn counts(kernel: &BrepKernel) -> (usize, usize, usize, usize) {
    (
        kernel.topo().num_vertices(),
        kernel.topo().num_edges(),
        kernel.topo().num_faces(),
        kernel.topo().num_solids(),
    )
}

fn slots(kernel: &BrepKernel) -> usize {
    kernel.topo().allocated_slot_count()
}

fn batch_v2_kernel_code(error: &serde_json::Value) -> &str {
    error["details"]["kernelCode"]
        .as_str()
        .or_else(|| error["code"].as_str())
        .unwrap()
}

fn assert_error_envelope(value: &serde_json::Value, operation: &str) {
    assert_eq!(value["status"], "error", "expected error envelope: {value}");
    assert!(value["code"].is_string(), "code must be a string: {value}");
    assert!(
        value["category"].is_string(),
        "category must be a string: {value}"
    );
    assert!(value["value"].is_null(), "value must be null: {value}");
    assert!(
        value["details"].is_object(),
        "details must be an object: {value}"
    );
    assert!(
        value["details"]["message"].is_string(),
        "details.message must be a string: {value}"
    );
    assert_eq!(
        value["details"]["operation"], operation,
        "details.operation must name the method: {value}"
    );
}

/// Build the OpenZCAD plate fixture from the fail-closed suite: a straight
/// top-perimeter edge plus a bore rim whose joint selection is refused whole
/// as `edges-not-blended`.
fn plate_with_bore(kernel: &mut BrepKernel) -> (u32, u32, u32) {
    let out = run(
        kernel,
        &[
            op(
                "makeBox",
                serde_json::json!({"width": 80.0, "height": 60.0, "depth": 6.0}),
            ),
            op(
                "makeCylinder",
                serde_json::json!({"radius": 2.25, "height": 20.0}),
            ),
            op(
                "transform",
                serde_json::json!({"solid": 1, "matrix": [1.0,0.0,0.0,40.0, 0.0,1.0,0.0,30.0, 0.0,0.0,1.0,-4.0, 0.0,0.0,0.0,1.0]}),
            ),
            op("cut", serde_json::json!({"solidA": 0, "solidB": 1})),
        ],
    );
    let plate = u32::try_from(out[3]["ok"].as_u64().unwrap()).unwrap();
    let edges = edge_handles(kernel, plate);
    let mut perimeter = None;
    let mut rim = None;
    for &handle in &edges {
        let eid = kernel.resolve_edge(handle).unwrap();
        let edge = kernel.topo.edge(eid).unwrap();
        let a = kernel.topo.vertex(edge.start()).unwrap().point();
        let b = kernel.topo.vertex(edge.end()).unwrap().point();
        let is_circle = matches!(edge.curve(), remus_topology::edge::EdgeCurve::Circle(_));
        if is_circle && rim.is_none() {
            rim = Some(handle);
            continue;
        }
        let on_top = (a.z() - 6.0).abs() < 1e-9 && (b.z() - 6.0).abs() < 1e-9;
        let spans_x = (a.x() - b.x()).abs() > 1.0 && (a.y() - b.y()).abs() < 1e-9;
        if on_top && spans_x && perimeter.is_none() {
            perimeter = Some(handle);
        }
    }
    (
        plate,
        perimeter.expect("plate must have a straight top-perimeter edge"),
        rim.expect("plate must have a circular bore rim"),
    )
}

#[test]
fn fillet_detailed_success_matches_legacy_and_batch() {
    let mut direct = BrepKernel::new();
    let solid = make_box(&mut direct, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut direct, solid);
    let before_vol = volume(&mut direct, solid);

    let result = direct.fillet_detailed_impl(solid, vec![edges[0]], 1.0);
    let value = serde_json::to_value(&result).unwrap();
    assert_eq!(value["status"], "ok", "{value}");
    assert!(value["code"].is_null());
    assert!(value["category"].is_null());
    assert_eq!(value["details"], serde_json::json!({}));
    let handle = u32::try_from(value["value"].as_u64().unwrap()).unwrap();
    assert_ne!(handle, solid, "a real fillet must mint a new solid");
    assert!(direct.resolve_solid(handle).is_ok());
    let direct_vol = direct.volume(handle, 0.01).unwrap();
    // Closed-form: (1-pi/4)*r^2 per unit length on a convex 10 mm edge.
    let expected_removed = (1.0 - std::f64::consts::FRAC_PI_4) * 1.0 * 10.0;
    assert!(
        (before_vol - direct_vol - expected_removed).abs() < 0.5,
        "fillet must remove ~{expected_removed:.2} mm^3: {before_vol} -> {direct_vol}"
    );
    // Representation: a straight-edge fillet wall is an analytic cylinder,
    // never a NURBS fallback.
    let faces =
        remus_topology::explorer::solid_faces(&direct.topo, direct.resolve_solid(handle).unwrap())
            .unwrap();
    let has_cylinder = faces.iter().any(|&f| {
        matches!(
            direct.topo.face(f).unwrap().surface(),
            remus_topology::face::FaceSurface::Cylinder(_)
        )
    });
    assert!(
        has_cylinder,
        "straight-edge fillet must emit a cylindrical wall"
    );

    // Legacy helper and batch agree on material.
    let mut legacy = BrepKernel::new();
    let lsolid = make_box(&mut legacy, 10.0, 10.0, 10.0);
    let ledges = edge_handles(&mut legacy, lsolid);
    let leid = legacy.resolve_edge(ledges[0]).unwrap();
    let lsid = legacy.resolve_solid(lsolid).unwrap();
    let lresult =
        crate::helpers::fillet_whole_selection(legacy.topo_mut(), lsid, &[leid], 1.0).unwrap();
    let lvol = remus_operations::measure::solid_volume(legacy.topo(), lresult, 0.01).unwrap();
    assert!(
        (direct_vol - lvol).abs() < 1e-9,
        "detailed and legacy must agree: {direct_vol} vs {lvol}"
    );

    let mut batch = BrepKernel::new();
    let bsetup = run(
        &mut batch,
        &[
            op(
                "makeBox",
                serde_json::json!({"width": 10.0, "height": 10.0, "depth": 10.0}),
            ),
            op("solidEdges", serde_json::json!({"solid": 0})),
        ],
    );
    let bedge = bsetup[1]["ok"].as_array().unwrap()[0].as_u64().unwrap();
    let bout = run(
        &mut batch,
        &[op(
            "fillet",
            serde_json::json!({"solid": 0, "radius": 1.0, "edges": [bedge]}),
        )],
    );
    assert!(bout[0]["ok"].is_number(), "batch fillet failed: {bout:?}");
    let bhandle = u32::try_from(bout[0]["ok"].as_u64().unwrap()).unwrap();
    let bvol = volume(&mut batch, bhandle);
    assert!(
        (direct_vol - bvol).abs() < 1e-9,
        "detailed and batch must agree: {direct_vol} vs {bvol}"
    );
}

#[test]
fn chamfer_detailed_success_matches_legacy_and_batch() {
    let mut direct = BrepKernel::new();
    let solid = make_box(&mut direct, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut direct, solid);
    let before_vol = volume(&mut direct, solid);

    let result = direct.chamfer_detailed_impl(solid, vec![edges[0]], 1.0);
    let value = serde_json::to_value(&result).unwrap();
    assert_eq!(value["status"], "ok", "{value}");
    assert!(value["code"].is_null());
    let handle = u32::try_from(value["value"].as_u64().unwrap()).unwrap();
    assert_ne!(handle, solid);
    let direct_vol = direct.volume(handle, 0.01).unwrap();
    assert!(
        direct_vol < before_vol,
        "chamfering a convex edge removes material: {before_vol} -> {direct_vol}"
    );

    let mut legacy = BrepKernel::new();
    let lsolid = make_box(&mut legacy, 10.0, 10.0, 10.0);
    let ledges = edge_handles(&mut legacy, lsolid);
    let leid = legacy.resolve_edge(ledges[0]).unwrap();
    let lsid = legacy.resolve_solid(lsolid).unwrap();
    let lresult = crate::helpers::try_chamfer(legacy.topo_mut(), lsid, &[leid], 1.0).unwrap();
    let lvol = remus_operations::measure::solid_volume(legacy.topo(), lresult, 0.01).unwrap();
    assert!((direct_vol - lvol).abs() < 1e-9);

    let mut batch = BrepKernel::new();
    let bsetup = run(
        &mut batch,
        &[
            op(
                "makeBox",
                serde_json::json!({"width": 10.0, "height": 10.0, "depth": 10.0}),
            ),
            op("solidEdges", serde_json::json!({"solid": 0})),
        ],
    );
    let bedge = bsetup[1]["ok"].as_array().unwrap()[0].as_u64().unwrap();
    let bout = run(
        &mut batch,
        &[op(
            "chamfer",
            serde_json::json!({"solid": 0, "distance": 1.0, "edges": [bedge]}),
        )],
    );
    assert!(bout[0]["ok"].is_number(), "batch chamfer failed: {bout:?}");
    let bvol = volume(
        &mut batch,
        u32::try_from(bout[0]["ok"].as_u64().unwrap()).unwrap(),
    );
    assert!((direct_vol - bvol).abs() < 1e-9);
}

#[test]
fn detailed_duplicate_edges_dedup_to_success() {
    for (operation, run_detailed) in [("fillet", true), ("chamfer", false)] {
        let mut kernel = BrepKernel::new();
        let solid = make_box(&mut kernel, 10.0, 10.0, 10.0);
        let edges = edge_handles(&mut kernel, solid);
        let value = if run_detailed {
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0], edges[0]], 1.0))
                .unwrap()
        } else {
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![edges[0], edges[0]], 1.0))
                .unwrap()
        };
        assert_eq!(value["status"], "ok", "{operation} duplicate: {value}");
        let handle = u32::try_from(value["value"].as_u64().unwrap()).unwrap();
        assert!(kernel.resolve_solid(handle).is_ok());
    }
}

#[test]
fn detailed_refusals_are_data_with_stable_types() {
    let mut kernel = BrepKernel::new();
    let solid = make_box(&mut kernel, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut kernel, solid);
    // Foreign edge from a second solid.
    let other = make_box(&mut kernel, 5.0, 5.0, 5.0);
    let other_edges = edge_handles(&mut kernel, other);
    let foreign = other_edges[0];

    let cases: Vec<(&str, serde_json::Value)> = vec![
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![], 1.0)).unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(9999, vec![edges[0]], 1.0)).unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![9999], 1.0)).unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![foreign], 1.0)).unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0]], 0.0)).unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0]], -1.0)).unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0]], f64::NAN))
                .unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0]], f64::INFINITY))
                .unwrap(),
        ),
        (
            "fillet",
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0]], 50.0)).unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![], 1.0)).unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(9999, vec![edges[0]], 1.0)).unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![9999], 1.0)).unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![foreign], 1.0)).unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![edges[0]], 0.0)).unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![edges[0]], -2.0))
                .unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![edges[0]], f64::NAN))
                .unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(
                solid,
                vec![edges[0]],
                f64::INFINITY,
            ))
            .unwrap(),
        ),
        (
            "chamfer",
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![edges[0]], 50.0))
                .unwrap(),
        ),
    ];
    for (operation, value) in cases {
        assert_error_envelope(&value, operation);
    }

    // Existing typed unsupported-support witness: the plate mixed selection
    // is refused whole as `edges-not-blended`.
    let mut plate_kernel = BrepKernel::new();
    let (plate, perimeter, rim) = plate_with_bore(&mut plate_kernel);
    let mixed =
        serde_json::to_value(plate_kernel.fillet_detailed_impl(plate, vec![perimeter, rim], 4.0))
            .unwrap();
    assert_error_envelope(&mixed, "fillet");
    assert_eq!(mixed["code"], "edges-not-blended", "{mixed}");
}

#[test]
fn detailed_refusal_codes_match_batch_v2() {
    // Every refusal the batch JSON surface can carry must report the same
    // native code from the direct twin. Non-finite scalars have no JSON
    // spelling, so they are covered by the envelope test above, not here.
    let mut kernel = BrepKernel::new();
    let solid = make_box(&mut kernel, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut kernel, solid);
    let other = make_box(&mut kernel, 5.0, 5.0, 5.0);
    let foreign = edge_handles(&mut kernel, other)[0];

    let fillet_cases: Vec<(Vec<u32>, f64)> = vec![
        (vec![], 1.0),
        (vec![u32::MAX], 1.0),
        (vec![foreign], 1.0),
        (vec![edges[0]], 0.0),
        (vec![edges[0]], -1.0),
        (vec![edges[0]], 50.0),
    ];
    for (edge_list, radius) in fillet_cases {
        let direct =
            serde_json::to_value(kernel.fillet_detailed_impl(solid, edge_list.clone(), radius))
                .unwrap();
        assert_eq!(direct["status"], "error");
        let batch = run_v2(
            &mut kernel,
            &[op(
                "fillet",
                serde_json::json!({"solid": solid, "radius": radius, "edges": edge_list}),
            )],
        );
        let batch_error = &batch[0]["error"];
        assert_eq!(
            direct["code"].as_str().unwrap(),
            batch_v2_kernel_code(batch_error),
            "fillet edges={edge_list:?} radius={radius}: direct={direct} batch={batch_error}"
        );
        assert_eq!(direct["category"], batch_error["category"]);
    }

    // Invalid solid handle (valid scalar): both surfaces report the wire code.
    let direct =
        serde_json::to_value(kernel.fillet_detailed_impl(u32::MAX, vec![edges[0]], 1.0)).unwrap();
    let batch = run_v2(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": u32::MAX, "radius": 1.0, "edges": [edges[0]]}),
        )],
    );
    assert_eq!(direct["code"], "invalid_handle");
    assert_eq!(
        direct["code"].as_str().unwrap(),
        batch[0]["error"]["code"].as_str().unwrap()
    );
    assert_eq!(direct["category"], batch[0]["error"]["category"]);

    let chamfer_cases: Vec<(Vec<u32>, f64)> = vec![
        (vec![u32::MAX], 1.0),
        (vec![foreign], 1.0),
        (vec![edges[0]], 0.0),
        (vec![edges[0]], -1.0),
        (vec![edges[0]], 50.0),
    ];
    for (edge_list, distance) in chamfer_cases {
        let direct =
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, edge_list.clone(), distance))
                .unwrap();
        let batch = run_v2(
            &mut kernel,
            &[op(
                "chamfer",
                serde_json::json!({"solid": solid, "distance": distance, "edges": edge_list}),
            )],
        );
        assert_eq!(
            direct["code"].as_str().unwrap(),
            batch_v2_kernel_code(&batch[0]["error"]),
            "chamfer edges={edge_list:?} distance={distance}: direct={direct} batch={batch:?}"
        );
        assert_eq!(direct["category"], batch[0]["error"]["category"]);
    }

    let direct =
        serde_json::to_value(kernel.chamfer_detailed_impl(u32::MAX, vec![edges[0]], 1.0)).unwrap();
    let batch = run_v2(
        &mut kernel,
        &[op(
            "chamfer",
            serde_json::json!({"solid": u32::MAX, "distance": 1.0, "edges": [edges[0]]}),
        )],
    );
    assert_eq!(direct["code"], "invalid_handle");
    assert_eq!(direct["code"], batch[0]["error"]["code"]);

    // The mixed-selection witness agrees across surfaces.
    let mut plate_kernel = BrepKernel::new();
    let (plate, perimeter, rim) = plate_with_bore(&mut plate_kernel);
    let direct =
        serde_json::to_value(plate_kernel.fillet_detailed_impl(plate, vec![perimeter, rim], 4.0))
            .unwrap();
    let batch = run_v2(
        &mut plate_kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": plate, "radius": 4.0, "edges": [perimeter, rim]}),
        )],
    );
    assert_eq!(direct["code"], "edges-not-blended");
    assert_eq!(
        direct["code"].as_str().unwrap(),
        batch_v2_kernel_code(&batch[0]["error"])
    );
}

#[test]
fn detailed_validation_precedence_is_handles_first_like_batch() {
    // Combined fault: an invalid solid handle plus an invalid scalar. The
    // detailed twins resolve handles before reaching the engines, so the
    // handle wins — matching `executeBatchV2`, which reports `invalid_handle`
    // for the same document. The legacy direct `fillet`/`chamfer` validate
    // the scalar first and would report the scalar instead; that legacy
    // precedence is unchanged and is not what the twins follow.
    let mut kernel = BrepKernel::new();
    let solid = make_box(&mut kernel, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut kernel, solid);

    let fillet =
        serde_json::to_value(kernel.fillet_detailed_impl(u32::MAX, vec![edges[0]], 0.0)).unwrap();
    assert_eq!(fillet["code"], "invalid_handle", "{fillet}");
    let batch = run_v2(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": u32::MAX, "radius": 0.0, "edges": [edges[0]]}),
        )],
    );
    assert_eq!(batch[0]["error"]["code"], "invalid_handle");

    let chamfer =
        serde_json::to_value(kernel.chamfer_detailed_impl(u32::MAX, vec![edges[0]], 0.0)).unwrap();
    assert_eq!(chamfer["code"], "invalid_handle", "{chamfer}");
    let batch = run_v2(
        &mut kernel,
        &[op(
            "chamfer",
            serde_json::json!({"solid": u32::MAX, "distance": 0.0, "edges": [edges[0]]}),
        )],
    );
    assert_eq!(batch[0]["error"]["code"], "invalid_handle");
}

#[test]
fn detailed_refusals_leave_no_trace_including_after_allocation() {
    // Oversize blends allocate a candidate before the volume/cliff guards
    // refuse it; the twin must roll back exactly like the batch path.
    for (operation, oversize) in [("fillet", true), ("chamfer", false)] {
        let mut kernel = BrepKernel::new();
        let solid = make_box(&mut kernel, 10.0, 10.0, 10.0);
        let edges = edge_handles(&mut kernel, solid);
        let before_counts = counts(&kernel);
        let before_slots = slots(&kernel);
        let before_vol = volume(&mut kernel, solid);
        let journal_before = run(&mut kernel, &[op("journalSummary", serde_json::json!({}))]);

        let value = if oversize {
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0]], 50.0)).unwrap()
        } else {
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![edges[0]], 50.0)).unwrap()
        };
        assert_eq!(value["status"], "error", "{operation}: {value}");
        // Entity counts roll back exactly; arena slots are high-water
        // preserved by contract (retired handles never alias), so slots may
        // grow while V/E/F/solids match.
        assert_eq!(
            counts(&kernel),
            before_counts,
            "{operation}: topology leaked"
        );
        assert!(
            slots(&kernel) >= before_slots,
            "{operation}: slot high-water must not rewind"
        );
        assert!(
            (volume(&mut kernel, solid) - before_vol).abs() < 1e-9,
            "{operation}: volume changed"
        );
        assert!(kernel.resolve_solid(solid).is_ok());
        assert!(kernel.resolve_edge(edges[0]).is_ok());
        let journal_after = run(&mut kernel, &[op("journalSummary", serde_json::json!({}))]);
        assert_eq!(journal_before, journal_after, "{operation}: journal leaked");

        // A fresh valid blend still works on the untouched input.
        let retry = if oversize {
            serde_json::to_value(kernel.fillet_detailed_impl(solid, vec![edges[0]], 1.0)).unwrap()
        } else {
            serde_json::to_value(kernel.chamfer_detailed_impl(solid, vec![edges[0]], 1.0)).unwrap()
        };
        assert_eq!(retry["status"], "ok", "{operation} retry: {retry}");
    }

    // The mixed-selection witness also rolls back whole.
    let mut kernel = BrepKernel::new();
    let (plate, perimeter, rim) = plate_with_bore(&mut kernel);
    let before_counts = counts(&kernel);
    let before_vol = volume(&mut kernel, plate);
    let refused =
        serde_json::to_value(kernel.fillet_detailed_impl(plate, vec![perimeter, rim], 4.0))
            .unwrap();
    assert_eq!(refused["code"], "edges-not-blended");
    assert_eq!(counts(&kernel), before_counts);
    assert!((volume(&mut kernel, plate) - before_vol).abs() < 1e-9);
}
