//! Contract tests for the O4.7 construction twins.
//!
//! The direct side runs the natively-testable `*_detailed_impl` bodies (a
//! `JsError` cannot be built off-wasm); the batch side goes through
//! `execute_batch` / `execute_batch_v2`. Every success asserts the
//! disclosed `details.quality == "exact"`, the independently derived volume,
//! and the result carriers/topology — not only the returned handle. Every
//! refusal is checked against the legacy op's `executeBatchV2` code and
//! category and for unchanged topology.
//!
//! Both engines are exact-only by construction (translated profile curves
//! for extrude, exact surfaces of revolution for revolve), so there is no
//! `approximate` success and no `exactOnly` flag: the approximation cells
//! are unreachable by design and every test below pins `quality: "exact"`.
//!
//! Batch parity replays the detailed op on an identically built twin kernel:
//! batch has no `makeRectangle`/`makePolygon`/`makeCircleEdge` ops, so the
//! fixtures cannot be rebuilt via batch builders. Building the same face
//! twice through the same direct calls is deterministic, so a face handle
//! means the same entity on both twins.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use crate::handles::face_id_to_u32;
use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

type Counts = (usize, usize, usize, usize, usize);

fn counts(kernel: &BrepKernel) -> Counts {
    let topo = kernel.topo();
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
        topo.allocated_slot_count(),
    )
}

/// Live entity counts, without the handle high-water mark: a rollback after
/// an attempt that allocated keeps those slots retired so no stale handle
/// can alias a later entity.
fn live_counts(kernel: &BrepKernel) -> (usize, usize, usize, usize) {
    let (vertices, edges, faces, solids, _) = counts(kernel);
    (vertices, edges, faces, solids)
}

fn envelope(result: SolidOperationDetailedResult) -> Value {
    serde_json::to_value(result).unwrap()
}

fn batch_v2(kernel: &mut BrepKernel, ops: &Value) -> Value {
    serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap()
}

fn batch_legacy(kernel: &mut BrepKernel, ops: &Value) -> Value {
    serde_json::from_str(&kernel.execute_batch(&ops.to_string())).unwrap()
}

/// Native code a legacy `executeBatchV2` error projects: `kernelCode` when
/// present, the wire code otherwise (the O4.7 direct-result rule).
fn v2_code(error: &Value) -> &str {
    error["details"]["kernelCode"]
        .as_str()
        .or_else(|| error["code"].as_str())
        .unwrap()
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

fn volume(kernel: &BrepKernel, solid: u32, deflection: f64) -> f64 {
    kernel.volume(solid, deflection).unwrap()
}

/// Scale-relative tessellation deflection: analytic faces integrate exactly
/// regardless, but the knob must stay scale-appropriate for any NURBS band.
fn deflection_for(scale: f64) -> f64 {
    (scale * 1e-2).max(1e-9)
}

// ── Fixture builders ────────────────────────────────────────────────

/// Axis-aligned rectangle face in the XY plane, centered at the origin.
fn rect_face(kernel: &mut BrepKernel, w: f64, h: f64) -> u32 {
    kernel.make_rectangle(w, h).unwrap()
}

/// Exact-circle disc face in the XY plane (a true `Circle` edge, not the
/// NURBS-arc `makeCircleFace`), centered at `(cx, cy, 0)`.
fn disc_face(kernel: &mut BrepKernel, cx: f64, cy: f64, r: f64) -> u32 {
    let edge = kernel
        .make_circle_edge(cx, cy, 0.0, 0.0, 0.0, 1.0, r)
        .unwrap();
    let wire = kernel.make_wire(vec![edge], true).unwrap();
    kernel.make_planar_face_from_wire(wire).unwrap()
}

/// Rectangle face with corners `(x0, y0)`, `(x1, y0)`, `(x1, y1)`,
/// `(x0, y1)` in the XY plane (`z = 0`). Used for revolve profiles whose
/// radial extent must be placed relative to the axis.
fn offset_rect_face(kernel: &mut BrepKernel, x0: f64, x1: f64, y0: f64, y1: f64) -> u32 {
    kernel
        .make_polygon(vec![x0, y0, 0.0, x1, y0, 0.0, x1, y1, 0.0, x0, y1, 0.0])
        .unwrap()
}

/// Outer rectangle with a single exact-circle hole, in the XY plane.
fn holed_rect_face(kernel: &mut BrepKernel, ow: f64, oh: f64, hole_r: f64) -> u32 {
    let hw = ow / 2.0;
    let hh = oh / 2.0;
    let outer = kernel
        .make_polygon_wire(vec![-hw, -hh, 0.0, hw, -hh, 0.0, hw, hh, 0.0, -hw, hh, 0.0])
        .unwrap();
    let hole_edge = kernel
        .make_circle_edge(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, hole_r)
        .unwrap();
    let hole = kernel.make_wire(vec![hole_edge], true).unwrap();
    kernel.make_face_from_wires(outer, vec![hole]).unwrap()
}

/// The analytic cylinder side face of a fresh cylinder solid.
fn cylinder_side_face(kernel: &mut BrepKernel, r: f64, h: f64) -> u32 {
    use remus_topology::face::FaceSurface;
    let solid = kernel.make_cylinder_solid(r, h).unwrap();
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let faces = remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap();
    let side = faces
        .into_iter()
        .find(|face| {
            matches!(
                kernel.topo().face(*face).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .unwrap();
    face_id_to_u32(side)
}

struct Carriers {
    planes: usize,
    cylinders: usize,
    tori: usize,
    nurbs: usize,
    total: usize,
}

/// Result faces by carrier family.
fn count_carriers(kernel: &BrepKernel, solid: u32) -> Carriers {
    use remus_topology::face::FaceSurface;
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let faces = remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap();
    let mut out = Carriers {
        planes: 0,
        cylinders: 0,
        tori: 0,
        nurbs: 0,
        total: faces.len(),
    };
    for face in faces {
        match kernel.topo().face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => out.planes += 1,
            FaceSurface::Cylinder(_) => out.cylinders += 1,
            FaceSurface::Torus(_) => out.tori += 1,
            FaceSurface::Nurbs(_) => out.nurbs += 1,
            FaceSurface::Cone(_) | FaceSurface::Sphere(_) => {
                panic!("unexpected cone/sphere carrier in construction fixture")
            }
        }
    }
    out
}

fn assert_exact_success(direct: &Value, operation: &str) -> u32 {
    assert_eq!(direct["status"], "ok", "{operation}: {direct}");
    assert!(direct["code"].is_null(), "{operation}: {direct}");
    assert!(direct["category"].is_null(), "{operation}: {direct}");
    assert_eq!(
        direct["details"]["quality"], "exact",
        "{operation}: {direct}"
    );
    assert!(
        direct["details"].get("engine").is_none(),
        "{operation}: {direct}"
    );
    handle(&direct["value"])
}

/// Direct/batch parity for one extrude call: `build` runs the same direct
/// face construction on three fresh kernels so the face handle matches.
fn check_extrude_parity(
    build: impl Fn(&mut BrepKernel) -> u32,
    dx: f64,
    dy: f64,
    dz: f64,
    dist: f64,
) -> (BrepKernel, u32, Value) {
    let mut direct_kernel = BrepKernel::new();
    let face = build(&mut direct_kernel);
    let direct = envelope(direct_kernel.extrude_detailed_impl(face, dx, dy, dz, dist));

    let mut v2_kernel = BrepKernel::new();
    let v2_face = build(&mut v2_kernel);
    assert_eq!(v2_face, face);
    let v2 = batch_v2(
        &mut v2_kernel,
        &json!([{"op": "extrudeDetailed", "args": {"face": face, "dx": dx, "dy": dy, "dz": dz, "distance": dist}}]),
    );
    assert_eq!(v2[0]["ok"], direct, "direct/executeBatchV2 parity");

    let mut legacy_kernel = BrepKernel::new();
    let legacy_face = build(&mut legacy_kernel);
    assert_eq!(legacy_face, face);
    let legacy = batch_legacy(
        &mut legacy_kernel,
        &json!([{"op": "extrudeDetailed", "args": {"face": face, "dx": dx, "dy": dy, "dz": dz, "distance": dist}}]),
    );
    assert_eq!(legacy[0]["ok"], direct, "direct/executeBatch parity");

    (direct_kernel, handle(&direct["value"]), direct)
}

/// Direct/batch parity for one revolve call; same twin-kernel discipline as
/// [`check_extrude_parity`].
#[allow(clippy::too_many_arguments)]
fn check_revolve_parity(
    build: impl Fn(&mut BrepKernel) -> u32,
    ox: f64,
    oy: f64,
    oz: f64,
    ax: f64,
    ay: f64,
    az: f64,
    angle: f64,
) -> (BrepKernel, u32, Value) {
    let mut direct_kernel = BrepKernel::new();
    let face = build(&mut direct_kernel);
    let direct = envelope(direct_kernel.revolve_detailed_impl(face, ox, oy, oz, ax, ay, az, angle));

    let mut v2_kernel = BrepKernel::new();
    let v2_face = build(&mut v2_kernel);
    assert_eq!(v2_face, face);
    let v2 = batch_v2(
        &mut v2_kernel,
        &json!([{"op": "revolveDetailed", "args": {"face": face, "originX": ox, "originY": oy, "originZ": oz, "axisX": ax, "axisY": ay, "axisZ": az, "angle": angle}}]),
    );
    assert_eq!(v2[0]["ok"], direct, "direct/executeBatchV2 parity");

    let mut legacy_kernel = BrepKernel::new();
    let legacy_face = build(&mut legacy_kernel);
    assert_eq!(legacy_face, face);
    let legacy = batch_legacy(
        &mut legacy_kernel,
        &json!([{"op": "revolveDetailed", "args": {"face": face, "originX": ox, "originY": oy, "originZ": oz, "axisX": ax, "axisY": ay, "axisZ": az, "angle": angle}}]),
    );
    assert_eq!(legacy[0]["ok"], direct, "direct/executeBatch parity");

    (direct_kernel, handle(&direct["value"]), direct)
}

// ── Extrude: rectangle, circle, holed profiles ───────────────────────

#[test]
fn extrude_rectangle_matches_area_times_distance_at_all_scales() {
    for scale in [1e-3, 1.0, 1e3] {
        let w = 2.0 * scale;
        let h = 3.0 * scale;
        let dist = 5.0 * scale;
        let deflection = deflection_for(scale);
        let expected = w * h * dist;

        let (kernel, result, direct) =
            check_extrude_parity(|k| rect_face(k, w, h), 0.0, 0.0, 1.0, dist);
        assert_exact_success(&direct, "extrude");
        let measured = volume(&kernel, result, deflection);
        assert!(
            ((measured - expected) / expected).abs() < 1e-6,
            "scale {scale}: {measured} vs {expected}"
        );

        // Six planar faces: two caps plus four walls.
        let carriers = count_carriers(&kernel, result);
        assert_eq!(carriers.total, 6, "scale {scale}");
        assert_eq!(carriers.planes, 6, "scale {scale}");
        assert_eq!(carriers.nurbs, 0, "scale {scale}");

        // Same engine path as the legacy op: identical committed geometry.
        let mut legacy_kernel = BrepKernel::new();
        let legacy_face = rect_face(&mut legacy_kernel, w, h);
        let legacy = batch_v2(
            &mut legacy_kernel,
            &json!([{"op": "extrude", "args": {"face": legacy_face, "dx": 0.0, "dy": 0.0, "dz": 1.0, "distance": dist}}]),
        );
        let legacy_solid = handle(&legacy[0]["ok"]);
        assert_eq!(legacy_solid, result, "scale {scale}: same handle");
        assert!(
            (volume(&legacy_kernel, legacy_solid, deflection) - measured).abs() / expected < 1e-9,
            "scale {scale}: legacy geometry diverges"
        );
    }
}

#[test]
fn extrude_circle_matches_pi_r_squared_h() {
    let r = 2.0;
    let dist = 5.0;
    let expected = std::f64::consts::PI * r * r * dist;

    let (kernel, result, direct) =
        check_extrude_parity(|k| disc_face(k, 0.0, 0.0, r), 0.0, 0.0, 1.0, dist);
    assert_exact_success(&direct, "extrude");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);

    // Two planar caps plus one exact cylinder wall.
    let carriers = count_carriers(&kernel, result);
    assert_eq!(carriers.total, 3);
    assert_eq!(carriers.planes, 2);
    assert_eq!(carriers.cylinders, 1);
    assert_eq!(carriers.nurbs, 0);
}

#[test]
fn extrude_holed_rectangle_subtracts_hole_exactly() {
    let (ow, oh, hole_r, dist) = (4.0, 3.0, 0.5, 5.0);
    let expected = (ow * oh - std::f64::consts::PI * hole_r * hole_r) * dist;

    let (kernel, result, direct) =
        check_extrude_parity(|k| holed_rect_face(k, ow, oh, hole_r), 0.0, 0.0, 1.0, dist);
    assert_exact_success(&direct, "extrude");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);

    // Two holed planar caps, four outer planar walls, one cylinder bore.
    let carriers = count_carriers(&kernel, result);
    assert_eq!(carriers.total, 7);
    assert_eq!(carriers.planes, 6);
    assert_eq!(carriers.cylinders, 1);
    assert_eq!(carriers.nurbs, 0);
}

#[test]
fn extrude_negative_distance_extrudes_opposite_with_same_volume() {
    let (w, h, dist) = (2.0, 3.0, 5.0);
    let expected = w * h * dist;

    let mut kernel = BrepKernel::new();
    let face = rect_face(&mut kernel, w, h);
    let direct = envelope(kernel.extrude_detailed_impl(face, 0.0, 0.0, 1.0, -dist));
    let result = assert_exact_success(&direct, "extrude");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);
}

#[test]
fn extrude_rigid_placement_preserves_volume() {
    let expected = 2.0 * 3.0 * 5.0;

    // Translated placement: rectangle corners offset by (100, -50, 25).
    let mut kernel = BrepKernel::new();
    let face = kernel
        .make_polygon(vec![
            100.0, -50.0, 25.0, 102.0, -50.0, 25.0, 102.0, -47.0, 25.0, 100.0, -47.0, 25.0,
        ])
        .unwrap();
    let direct = envelope(kernel.extrude_detailed_impl(face, 0.0, 0.0, 1.0, 5.0));
    let result = assert_exact_success(&direct, "extrude");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);

    // Rotated placement: rectangle in the XZ plane extruded along +Y.
    let mut rotated = BrepKernel::new();
    let rface = rotated
        .make_polygon(vec![
            0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.0, 3.0,
        ])
        .unwrap();
    let rdirect = envelope(rotated.extrude_detailed_impl(rface, 0.0, 1.0, 0.0, 5.0));
    let rresult = assert_exact_success(&rdirect, "extrude");
    assert!((volume(&rotated, rresult, 0.01) - expected).abs() / expected < 1e-6);
}

// ── Revolve: cylinder, annulus, sector, torus ────────────────────────

/// Axis used by every revolve fixture below: the Y axis through the origin.
fn y_axis() -> (f64, f64, f64, f64, f64, f64) {
    (0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
}

#[test]
fn revolve_rectangle_full_gives_cylinder() {
    let (w, h) = (2.0, 5.0);
    let expected = std::f64::consts::PI * w * w * h;
    let (ox, oy, oz, dx, dy, dz) = y_axis();

    let (kernel, result, direct) = check_revolve_parity(
        |k| offset_rect_face(k, 0.0, w, 0.0, h),
        ox,
        oy,
        oz,
        dx,
        dy,
        dz,
        360.0,
    );
    assert_exact_success(&direct, "revolve");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);

    // One cylinder wall plus two planar discs.
    let carriers = count_carriers(&kernel, result);
    assert_eq!(carriers.total, 3);
    assert_eq!(carriers.planes, 2);
    assert_eq!(carriers.cylinders, 1);
    assert_eq!(carriers.nurbs, 0);

    // Same handle and geometry as the legacy op on an identical kernel.
    let mut legacy_kernel = BrepKernel::new();
    let legacy_face = offset_rect_face(&mut legacy_kernel, 0.0, w, 0.0, h);
    let legacy = batch_v2(
        &mut legacy_kernel,
        &json!([{"op": "revolve", "args": {"face": legacy_face, "originX": ox, "originY": oy, "originZ": oz, "axisX": dx, "axisY": dy, "axisZ": dz, "angle": 360.0}}]),
    );
    let legacy_solid = handle(&legacy[0]["ok"]);
    assert_eq!(legacy_solid, result);
    assert!((volume(&legacy_kernel, legacy_solid, 0.01) - expected).abs() / expected < 1e-6);
}

#[test]
fn revolve_offset_rectangle_gives_annulus() {
    let (ri, ro, h) = (1.0, 3.0, 4.0);
    let expected = std::f64::consts::PI * (ro * ro - ri * ri) * h;
    let (ox, oy, oz, dx, dy, dz) = y_axis();

    let (kernel, result, direct) = check_revolve_parity(
        |k| offset_rect_face(k, ri, ro, 0.0, h),
        ox,
        oy,
        oz,
        dx,
        dy,
        dz,
        360.0,
    );
    assert_exact_success(&direct, "revolve");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);

    // Inner and outer cylinders plus two planar annuli.
    let carriers = count_carriers(&kernel, result);
    assert_eq!(carriers.total, 4);
    assert_eq!(carriers.planes, 2);
    assert_eq!(carriers.cylinders, 2);
    assert_eq!(carriers.nurbs, 0);
}

#[test]
fn revolve_partial_gives_sector_fraction() {
    let (w, h) = (2.0, 5.0);
    let full = std::f64::consts::PI * w * w * h;
    let (ox, oy, oz, dx, dy, dz) = y_axis();

    for (angle, fraction) in [(90.0, 0.25), (180.0, 0.5)] {
        let mut kernel = BrepKernel::new();
        let face = offset_rect_face(&mut kernel, 0.0, w, 0.0, h);
        let direct = envelope(kernel.revolve_detailed_impl(face, ox, oy, oz, dx, dy, dz, angle));
        let result = assert_exact_success(&direct, "revolve");
        let expected = full * fraction;
        // Partial bands are exact NURBS measured through tessellation, so
        // allow tessellation convergence (full analytic cases pin 1e-6 via
        // exact integration).
        let measured = volume(&kernel, result, 0.0001);
        assert!(
            (measured - expected).abs() / expected < 1e-3,
            "angle {angle}: {measured} vs {expected}"
        );

        // Exact construction, not a mesh fallback: a handful of faces.
        let solid_id = kernel.resolve_solid(result).unwrap();
        let faces = remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap();
        assert!(faces.len() < 20, "angle {angle}: {} faces", faces.len());
    }
}

#[test]
fn revolve_disc_offset_gives_torus() {
    let (major, minor) = (5.0, 1.0);
    let expected = 2.0 * std::f64::consts::PI * std::f64::consts::PI * major * minor * minor;
    let (ox, oy, oz, dx, dy, dz) = y_axis();

    let (kernel, result, direct) = check_revolve_parity(
        |k| disc_face(k, major, 0.0, minor),
        ox,
        oy,
        oz,
        dx,
        dy,
        dz,
        360.0,
    );
    assert_exact_success(&direct, "revolve");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);

    // One analytic torus face.
    let carriers = count_carriers(&kernel, result);
    assert_eq!(carriers.total, 1);
    assert_eq!(carriers.tori, 1);
    assert_eq!(carriers.nurbs, 0);
}

#[test]
fn revolve_scales_and_translated_placement() {
    for scale in [1e-3, 1.0, 1e3] {
        let w = 2.0 * scale;
        let h = 5.0 * scale;
        let expected = std::f64::consts::PI * w * w * h;
        let deflection = deflection_for(scale);

        let mut kernel = BrepKernel::new();
        let face = offset_rect_face(&mut kernel, 0.0, w, 0.0, h);
        let direct =
            envelope(kernel.revolve_detailed_impl(face, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 360.0));
        let result = assert_exact_success(&direct, "revolve");
        let rel = ((volume(&kernel, result, deflection) - expected) / expected).abs();
        assert!(rel < 1e-6, "scale {scale}: rel {rel}");
    }

    // Translated placement: axis and profile shifted together by (10, 20, 30).
    let (w, h) = (2.0, 5.0);
    let expected = std::f64::consts::PI * w * w * h;
    let mut kernel = BrepKernel::new();
    let face = kernel
        .make_polygon(vec![
            10.0, 20.0, 30.0, 12.0, 20.0, 30.0, 12.0, 25.0, 30.0, 10.0, 25.0, 30.0,
        ])
        .unwrap();
    let direct =
        envelope(kernel.revolve_detailed_impl(face, 10.0, 20.0, 30.0, 0.0, 1.0, 0.0, 360.0));
    let result = assert_exact_success(&direct, "revolve");
    assert!((volume(&kernel, result, 0.01) - expected).abs() / expected < 1e-6);
}

#[test]
fn degrees_to_radians_conversion_is_exact() {
    let (w, h) = (2.0, 5.0);
    let full = std::f64::consts::PI * w * w * h;
    let (ox, oy, oz, dx, dy, dz) = y_axis();

    // 180 degrees is exactly half the full volume (tessellation-measured
    // partial band, so the sector tolerance applies).
    let (kernel, half_solid, half) = check_revolve_parity(
        |k| offset_rect_face(k, 0.0, w, 0.0, h),
        ox,
        oy,
        oz,
        dx,
        dy,
        dz,
        180.0,
    );
    assert_exact_success(&half, "revolve");
    assert!(
        (volume(&kernel, half_solid, 0.0001) - full * 0.5).abs() / full < 1e-3,
        "180-degree sector"
    );

    // 360 succeeds (inclusive upper bound).
    let mut full_kernel = BrepKernel::new();
    let full_face = offset_rect_face(&mut full_kernel, 0.0, w, 0.0, h);
    let full_result =
        envelope(full_kernel.revolve_detailed_impl(full_face, ox, oy, oz, dx, dy, dz, 360.0));
    assert_exact_success(&full_result, "revolve");

    // The native operation receives radians: 180 degrees is exactly pi.
    assert!((180.0_f64.to_radians() - std::f64::consts::PI).abs() < 1e-15);
}

// ── Argument refusals: typed data, legacy parity, rollback ────────────

#[test]
fn argument_refusals_are_typed_data() {
    let mut kernel = BrepKernel::new();
    let face = rect_face(&mut kernel, 2.0, 3.0);
    let before = counts(&kernel);

    for (label, direct) in [
        (
            "extrude dir_x NaN",
            envelope(kernel.extrude_detailed_impl(face, f64::NAN, 0.0, 1.0, 1.0)),
        ),
        (
            "extrude distance infinite",
            envelope(kernel.extrude_detailed_impl(face, 0.0, 0.0, 1.0, f64::INFINITY)),
        ),
        (
            "extrude zero direction",
            envelope(kernel.extrude_detailed_impl(face, 0.0, 0.0, 0.0, 1.0)),
        ),
        (
            "extrude zero distance",
            envelope(kernel.extrude_detailed_impl(face, 0.0, 0.0, 1.0, 0.0)),
        ),
        (
            "revolve origin NaN",
            envelope(kernel.revolve_detailed_impl(face, f64::NAN, 0.0, 0.0, 0.0, 1.0, 0.0, 90.0)),
        ),
        (
            "revolve axis infinite",
            envelope(kernel.revolve_detailed_impl(
                face,
                0.0,
                0.0,
                0.0,
                0.0,
                f64::INFINITY,
                0.0,
                90.0,
            )),
        ),
        (
            "revolve zero axis",
            envelope(kernel.revolve_detailed_impl(face, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0)),
        ),
        (
            "revolve zero angle",
            envelope(kernel.revolve_detailed_impl(face, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)),
        ),
        (
            "revolve negative angle",
            envelope(kernel.revolve_detailed_impl(face, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -90.0)),
        ),
        (
            "revolve angle above 360",
            envelope(kernel.revolve_detailed_impl(face, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 360.0001)),
        ),
    ] {
        assert_eq!(direct["status"], "error", "{label}: {direct}");
        assert_eq!(direct["code"], "invalid_argument", "{label}: {direct}");
        assert_eq!(direct["category"], "invalid_input", "{label}: {direct}");
        assert!(direct["value"].is_null(), "{label}: {direct}");
        assert!(
            direct["details"]["message"].is_string(),
            "{label}: {direct}"
        );
    }
    assert_eq!(counts(&kernel), before, "argument refusal mutated topology");
}

#[test]
fn handle_refusals_match_legacy_batch_v2_and_preserve_handles() {
    let mut kernel = BrepKernel::new();
    let face = rect_face(&mut kernel, 2.0, 3.0);
    let before = counts(&kernel);
    let invalid = u32::MAX;

    // A handle from a larger foreign kernel that is out of bounds here.
    // Handles are per-type arena indices, so a foreign handle with an
    // overlapping index would alias the local entity by design; only a
    // non-overlapping index is a refusal.
    let mut foreign = BrepKernel::new();
    let _ = rect_face(&mut foreign, 2.0, 3.0);
    let foreign_face = rect_face(&mut foreign, 4.0, 5.0);
    assert!(
        foreign_face != face,
        "foreign fixture must not overlap the local handle"
    );

    // Each case runs sequentially: every refusal must leave the counts
    // exactly as they were, so an eager case vector would hide the mutator.
    let cases: Vec<(&str, &str, Value)> = vec![
        (
            "stale extrude face",
            "extrude",
            envelope(kernel.extrude_detailed_impl(invalid, 0.0, 0.0, 1.0, 1.0)),
        ),
        (
            "foreign extrude face",
            "extrude",
            envelope(kernel.extrude_detailed_impl(foreign_face, 0.0, 0.0, 1.0, 1.0)),
        ),
        (
            "stale revolve face",
            "revolve",
            envelope(kernel.revolve_detailed_impl(invalid, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 90.0)),
        ),
        (
            "foreign revolve face",
            "revolve",
            envelope(kernel.revolve_detailed_impl(
                foreign_face,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                90.0,
            )),
        ),
    ];

    for (label, operation, direct) in &cases {
        assert_eq!(direct["status"], "error", "{label}: {direct}");
        assert_eq!(direct["code"], "invalid_handle", "{label}: {direct}");
        assert_eq!(direct["category"], "invalid_input", "{label}: {direct}");
        assert!(direct["value"].is_null(), "{label}: {direct}");
        assert_eq!(direct["details"]["operation"], *operation, "{label}");
        assert_eq!(direct["details"]["entity"], "face", "{label}");
        assert!(direct["details"]["message"].is_string(), "{label}");
        assert_eq!(counts(&kernel), before, "{label}: mutated");
    }

    // Refusal codes match the legacy ops' `executeBatchV2` codes.
    for (label, operation, args) in [
        (
            "stale extrude face",
            "extrude",
            json!({"face": invalid, "dx": 0.0, "dy": 0.0, "dz": 1.0, "distance": 1.0}),
        ),
        (
            "stale revolve face",
            "revolve",
            json!({"face": invalid, "originX": 0.0, "originY": 0.0, "originZ": 0.0, "axisX": 0.0, "axisY": 1.0, "axisZ": 0.0, "angle": 90.0}),
        ),
    ] {
        let legacy = batch_v2(&mut kernel, &json!([{"op": operation, "args": args}]));
        let legacy_error = &legacy[0]["error"];
        let direct = cases
            .iter()
            .find(|(l, _, _)| *l == label)
            .map(|(_, _, d)| d)
            .unwrap();
        assert_eq!(
            direct["code"].as_str().unwrap(),
            v2_code(legacy_error),
            "{label}: direct={direct} legacy={legacy_error}"
        );
        assert_eq!(direct["category"], legacy_error["category"], "{label}");
        assert_eq!(counts(&kernel), before, "{label}: legacy mutated");
    }

    // Batch twins return the identical envelope under both contracts.
    let batch = batch_v2(
        &mut kernel,
        &json!([{"op": "extrudeDetailed", "args": {"face": invalid, "dx": 0.0, "dy": 0.0, "dz": 1.0, "distance": 1.0}}]),
    );
    assert_eq!(batch[0]["ok"]["code"], "invalid_handle", "{batch}");
    assert_eq!(counts(&kernel), before);

    // Pre-existing handles stay usable: a fresh detailed call on the live
    // face commits with the expected volume.
    let retry = envelope(kernel.extrude_detailed_impl(face, 0.0, 0.0, 1.0, 1.0));
    let retry_solid = assert_exact_success(&retry, "extrude");
    assert!((volume(&kernel, retry_solid, 0.01) - 6.0).abs() < 1e-6);
}

/// A partial revolution of a curved profile that has no exact cap fill
/// stays a typed refusal (the supported full-revolution path needs no
/// caps). The exact refusal depends on the profile (degenerate boundary
/// normal or curved cap boundary); the twin mirrors whatever the engine
/// reports, with the legacy code and rollback.
#[test]
fn unsupported_partial_revolve_refuses_with_rollback() {
    let mut kernel = BrepKernel::new();
    let face = cylinder_side_face(&mut kernel, 2.0, 5.0);
    let before = live_counts(&kernel);
    let before_counts = counts(&kernel);

    let direct = envelope(kernel.revolve_detailed_impl(face, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 90.0));
    assert_eq!(direct["status"], "error", "{direct}");
    assert!(direct["value"].is_null());
    assert_eq!(direct["details"]["operation"], "revolve");
    assert_eq!(live_counts(&kernel), before, "refusal must roll back");
    assert_eq!(counts(&kernel), before_counts);

    let legacy = batch_v2(
        &mut kernel,
        &json!([{"op": "revolve", "args": {"face": face, "originX": 0.0, "originY": 0.0, "originZ": 0.0, "axisX": 0.0, "axisY": 1.0, "axisZ": 0.0, "angle": 90.0}}]),
    );
    let legacy_error = &legacy[0]["error"];
    assert_eq!(
        direct["code"].as_str().unwrap(),
        v2_code(legacy_error),
        "direct={direct} legacy={legacy_error}"
    );
    assert_eq!(direct["category"], legacy_error["category"]);

    let batch = batch_v2(
        &mut kernel,
        &json!([{"op": "revolveDetailed", "args": {"face": face, "originX": 0.0, "originY": 0.0, "originZ": 0.0, "axisX": 0.0, "axisY": 1.0, "axisZ": 0.0, "angle": 90.0}}]),
    );
    assert_eq!(batch[0]["ok"], direct, "batch twin parity");
    assert_eq!(live_counts(&kernel), before);
}

// ── Approximation cells are unreachable ─────────────────────────────

#[test]
fn every_success_reports_exact_with_no_approximate_branch() {
    // The matrix above covers rectangle, circle, holed, partial, full,
    // scaled and placed profiles. Each pins quality exact at the call site;
    // this test states the envelope rule once: no construction twin takes
    // an exactOnly/approximation knob, so no call can commit an approximate
    // body. A future sampled-refit path must add disclosure first.
    let mut kernel = BrepKernel::new();
    let face = rect_face(&mut kernel, 2.0, 3.0);
    let extruded = envelope(kernel.extrude_detailed_impl(face, 0.0, 0.0, 1.0, 2.0));
    assert_eq!(extruded["details"]["quality"], "exact");
    assert!(extruded["details"].get("approximateFaces").is_none());
    assert!(extruded["details"].get("sampledFaces").is_none());
    assert!(extruded["details"].get("deflection").is_none());

    let mut revolved_kernel = BrepKernel::new();
    let rface = offset_rect_face(&mut revolved_kernel, 0.0, 2.0, 0.0, 5.0);
    let revolved =
        envelope(revolved_kernel.revolve_detailed_impl(rface, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 360.0));
    assert_eq!(revolved["details"]["quality"], "exact");
    assert!(revolved["details"].get("approximateFaces").is_none());
}
