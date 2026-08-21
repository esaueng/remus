//! Regression tests for faces built with inner (hole) wires and then
//! extruded — the `addHolesToFace` / `makeFaceFromWires` → `extrude` path.
//!
//! `docs/production-readiness/stability-matrix.md` lists Extrude as
//! "Blocked: full degenerate/cavity matrix incomplete", and until these
//! tests existed the hole-attaching APIs had no coverage of any kind. Two
//! profiles are exercised end to end:
//!
//! 1. a polygon annulus — all-line loops, exactly known area;
//! 2. an 'O'-like contour whose outer and inner loops both MIX line edges
//!    with cubic-bezier (NURBS) edges, which is the shape a glyph outline
//!    produces.
//!
//! Each asserts the extruded solid is watertight, has the expected face
//! count, has volume ≈ (outer − hole) area × depth against an oracle
//! computed here independently of the kernel, and classifies points in the
//! hole as outside the material — volume alone cannot distinguish a real
//! through-hole from one merely subtracted from the integral.
//!
//! `validate_solid` is asserted against an explicit allow-list, at every
//! severity rather than errors only: see [`assert_solid`]. The two extrude
//! orientation defects this file originally carried as `#[ignore]`
//! ready-repros are fixed and their tests now run as regression pins:
//! `extruded_annulus_shell_orientation_is_consistent` and
//! `o_glyph_bezier_cap_band_classifies_correctly`.
//!
//! The classification probes are placed where the answer is derivable by
//! hand from the profile definition.
//!
//! Every kernel call goes through `execute_batch`: `JsError` cannot be
//! constructed on non-wasm targets, so the `#[wasm_bindgen]` methods are
//! not directly testable on their error paths.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;

// ── batch plumbing ────────────────────────────────────────────────

/// Run a batch and return every result entry.
fn run(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&k.execute_batch(&json)).unwrap()
}

/// Run a batch, require every op to succeed, and return the `ok` payloads.
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

/// Run a batch whose LAST op must fail, and return that failure message.
fn run_expect_last_error(k: &mut BrepKernel, ops: &[serde_json::Value]) -> String {
    let results = run(k, ops);
    for (i, r) in results.iter().enumerate().take(ops.len() - 1) {
        assert!(
            r.get("ok").is_some(),
            "setup op {i} ({}) failed: {r}",
            ops[i]["op"]
        );
    }
    let last = results.last().unwrap();
    match last.get("error").and_then(serde_json::Value::as_str) {
        Some(s) => s.to_string(),
        None => panic!("expected the last op to fail, got {last}"),
    }
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn as_u32(v: &serde_json::Value) -> u32 {
    u32::try_from(v.as_u64().unwrap()).unwrap()
}

// ── loop description ──────────────────────────────────────────────

/// One segment of a 2D loop laid on the plane `z = z`.
#[derive(Clone, Copy)]
enum Seg {
    /// Straight segment to `(x, y)`.
    Line(f64, f64),
    /// Cubic bezier to `(x3, y3)` with controls `(x1, y1)`, `(x2, y2)`.
    Cubic(f64, f64, f64, f64, f64, f64),
}

/// A closed loop: a start point plus segments returning to it.
struct Loop {
    start: (f64, f64),
    segs: Vec<Seg>,
    z: f64,
}

impl Loop {
    /// Emit the batch ops that build this loop's edges, then a `makeWire`.
    ///
    /// Endpoint doubles are shared bit-for-bit between adjacent segments
    /// (each segment's start is the literal previous end), which is what
    /// `makeWire`'s 1e-7 weld needs to close the loop.
    fn build_ops(&self, ops: &mut Vec<serde_json::Value>) -> usize {
        let first_edge_index = ops.len();
        let mut cur = self.start;
        for seg in &self.segs {
            match *seg {
                Seg::Line(x, y) => {
                    ops.push(op(
                        "makeLineEdge",
                        serde_json::json!({
                            "x1": cur.0, "y1": cur.1, "z1": self.z,
                            "x2": x, "y2": y, "z2": self.z,
                        }),
                    ));
                    cur = (x, y);
                }
                Seg::Cubic(x1, y1, x2, y2, x3, y3) => {
                    ops.push(op(
                        "makeNurbsEdge",
                        serde_json::json!({
                            "startX": cur.0, "startY": cur.1, "startZ": self.z,
                            "endX": x3, "endY": y3, "endZ": self.z,
                            "degree": 3,
                            "knots": [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                            "controlPoints": [
                                cur.0, cur.1, self.z,
                                x1, y1, self.z,
                                x2, y2, self.z,
                                x3, y3, self.z,
                            ],
                            "weights": [1.0, 1.0, 1.0, 1.0],
                        }),
                    ));
                    cur = (x3, y3);
                }
            }
        }
        assert!(
            (cur.0 - self.start.0).abs() < 1e-12 && (cur.1 - self.start.1).abs() < 1e-12,
            "loop does not return to its start point"
        );
        first_edge_index
    }

    /// Number of edges this loop contributes.
    fn edge_count(&self) -> usize {
        self.segs.len()
    }

    /// Signed area of the loop, computed here by densely sampling the same
    /// segment definitions the kernel was handed. This is the oracle the
    /// extruded volume is checked against — it never consults the kernel.
    fn signed_area(&self) -> f64 {
        const BEZIER_SAMPLES: usize = 4096;
        let mut pts: Vec<(f64, f64)> = vec![self.start];
        let mut cur = self.start;
        for seg in &self.segs {
            match *seg {
                Seg::Line(x, y) => {
                    pts.push((x, y));
                    cur = (x, y);
                }
                Seg::Cubic(x1, y1, x2, y2, x3, y3) => {
                    for i in 1..=BEZIER_SAMPLES {
                        #[allow(clippy::cast_precision_loss)]
                        let t = i as f64 / BEZIER_SAMPLES as f64;
                        let mt = 1.0 - t;
                        let b0 = mt * mt * mt;
                        let b1 = 3.0 * mt * mt * t;
                        let b2 = 3.0 * mt * t * t;
                        let b3 = t * t * t;
                        pts.push((
                            b0 * cur.0 + b1 * x1 + b2 * x2 + b3 * x3,
                            b0 * cur.1 + b1 * y1 + b2 * y2 + b3 * y3,
                        ));
                    }
                    cur = (x3, y3);
                }
            }
        }
        // The final point repeats `start`; shoelace wraps anyway.
        let n = pts.len() - 1;
        let mut acc = 0.0;
        for i in 0..n {
            let (xi, yi) = pts[i];
            let (xj, yj) = pts[i + 1];
            acc += xi.mul_add(yj, -(xj * yi));
        }
        acc / 2.0
    }
}

/// A square loop, CCW when `ccw`, laid at `z = 0`.
fn square(half: f64, ccw: bool) -> Loop {
    let corners = if ccw {
        [(-half, -half), (half, -half), (half, half), (-half, half)]
    } else {
        [(-half, -half), (-half, half), (half, half), (half, -half)]
    };
    Loop {
        start: corners[0],
        segs: vec![
            Seg::Line(corners[1].0, corners[1].1),
            Seg::Line(corners[2].0, corners[2].1),
            Seg::Line(corners[3].0, corners[3].1),
            Seg::Line(corners[0].0, corners[0].1),
        ],
        z: 0.0,
    }
}

/// A "capsule": two straight sides joined by two cubic-bezier caps.
///
/// `w` is the half-width of the straight sides, `h` their half-height, and
/// `bulge` how far past `w` the caps reach at their control points. CCW when
/// `ccw`. This is the mixed line/bezier contour an 'O' glyph produces.
fn capsule(w: f64, h: f64, bulge: f64, ccw: bool) -> Loop {
    let b = w + bulge;
    if ccw {
        Loop {
            start: (-w, -h),
            segs: vec![
                // bottom, left → right
                Seg::Line(w, -h),
                // right cap, bottom → top, bulging +x
                Seg::Cubic(b, -h, b, h, w, h),
                // top, right → left
                Seg::Line(-w, h),
                // left cap, top → bottom, bulging −x
                Seg::Cubic(-b, h, -b, -h, -w, -h),
            ],
            z: 0.0,
        }
    } else {
        Loop {
            start: (-w, -h),
            segs: vec![
                // left cap, bottom → top, bulging −x
                Seg::Cubic(-b, -h, -b, h, -w, h),
                // top, left → right
                Seg::Line(w, h),
                // right cap, top → bottom, bulging +x
                Seg::Cubic(b, h, b, -h, w, -h),
                // bottom, right → left
                Seg::Line(-w, -h),
            ],
            z: 0.0,
        }
    }
}

// ── assembly + assertions ─────────────────────────────────────────

/// How the holed face is assembled — the two APIs must agree.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum FaceApi {
    /// `makeFaceFromWires(outer, [holes])`
    FromWires,
    /// `makePlanarFaceFromWire(outer)` then `addHolesToFace(face, [holes])`
    AddHoles,
}

/// Build `outer` + `holes`, make a face by `api`, extrude by `depth`, and
/// return the kernel plus the solid handle.
fn extrude_holed_face(outer: &Loop, holes: &[Loop], api: FaceApi, depth: f64) -> (BrepKernel, u32) {
    let mut k = BrepKernel::new();
    let mut ops: Vec<serde_json::Value> = Vec::new();

    let outer_first = outer.build_ops(&mut ops);
    let outer_edges: Vec<usize> = (outer_first..outer_first + outer.edge_count()).collect();
    let mut hole_edge_ranges = Vec::new();
    for h in holes {
        let first = h.build_ops(&mut ops);
        hole_edge_ranges.push((first, h.edge_count()));
    }

    // Resolve the edge handles produced above, then continue in a second
    // batch: makeWire needs the handles as literal arguments.
    let edge_results = run_all_ok(&mut k, &ops);
    let edge_handle = |i: usize| as_u32(&edge_results[i]);

    let mut ops2: Vec<serde_json::Value> = vec![op(
        "makeWire",
        serde_json::json!({
            "edges": outer_edges.iter().map(|&i| edge_handle(i)).collect::<Vec<_>>(),
            "closed": true,
        }),
    )];
    for &(first, count) in &hole_edge_ranges {
        ops2.push(op(
            "makeWire",
            serde_json::json!({
                "edges": (first..first + count).map(edge_handle).collect::<Vec<_>>(),
                "closed": true,
            }),
        ));
    }
    let wire_results = run_all_ok(&mut k, &ops2);
    let outer_wire = as_u32(&wire_results[0]);
    let hole_wires: Vec<u32> = wire_results[1..].iter().map(as_u32).collect();

    let face_ops = match api {
        FaceApi::FromWires => vec![op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": hole_wires}),
        )],
        FaceApi::AddHoles => vec![op(
            "makePlanarFaceFromWire",
            serde_json::json!({"wire": outer_wire}),
        )],
    };
    let face_results = run_all_ok(&mut k, &face_ops);
    let face = match api {
        FaceApi::FromWires => as_u32(&face_results[0]),
        FaceApi::AddHoles => {
            let base = as_u32(&face_results[0]);
            let r = run_all_ok(
                &mut k,
                &[op(
                    "addHolesToFace",
                    serde_json::json!({"face": base, "holeWires": hole_wires}),
                )],
            );
            as_u32(&r[0])
        }
    };

    let solid_results = run_all_ok(
        &mut k,
        &[op(
            "extrude",
            serde_json::json!({
                "face": face, "dirX": 0.0, "dirY": 0.0, "dirZ": 1.0, "distance": depth,
            }),
        )],
    );
    let solid = as_u32(&solid_results[0]);
    (k, solid)
}

/// Assert the solid is watertight, has `expected_faces` faces, has volume
/// within `rel_tol` of `expected_volume`, and classifies `inside_probes` as
/// material and `outside_probes` (points in the holes, and outside the
/// body) as void.
///
/// The coarser of the two mesh densities [`assert_solid`] measures volume at.
/// Must stay below `bbox_diag × 5e-5` for every solid tested here, or
/// `volume_tessellation_deflection` clamps both densities to the same mesh
/// and the convergence check becomes vacuous.
const VOLUME_DEFLECTION_COARSE: f64 = 5e-4;

/// The finer of the two — see [`VOLUME_DEFLECTION_COARSE`].
const VOLUME_DEFLECTION_FINE: f64 = 1e-4;

/// `validate_solid` is run too, and asserted against an explicit allow-list
/// covering EVERY severity, not just `Error`. The shell-level orientation
/// defect is fixed, and `check_face_orientation` now compares the stored
/// wire winding against the STORED surface normal (the reversal flag
/// mirrors normal and traversal together), so correctly wound reversed
/// ruled-NURBS hole walls no longer warn. `expected_flipped_faces` pins the
/// count at zero, so a regression that flips any face inside out still
/// fails the test.
#[allow(clippy::too_many_arguments)]
fn assert_solid(
    k: &mut BrepKernel,
    solid: u32,
    expected_faces: usize,
    expected_flipped_faces: usize,
    expected_volume: f64,
    rel_tol: f64,
    deflection: f64,
    inside_probes: &[(f64, f64, f64)],
    outside_probes: &[(f64, f64, f64)],
) {
    use remus_check::validate::CheckId;

    let quality = run_all_ok(
        k,
        &[op(
            "meshQuality",
            serde_json::json!({"solid": solid, "deflection": deflection}),
        )],
    );
    let q = &quality[0];
    assert_eq!(
        q["boundaryEdges"].as_u64(),
        Some(0),
        "mesh has boundary edges — not watertight: {q}"
    );
    assert_eq!(
        q["nonManifoldEdges"].as_u64(),
        Some(0),
        "mesh has non-manifold edges: {q}"
    );
    assert_eq!(
        q["isWatertight"].as_bool(),
        Some(true),
        "not watertight: {q}"
    );

    let solid_id = k.resolve_solid(solid).unwrap();
    let report = remus_check::validate::validate_solid(
        &k.topo,
        solid_id,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    let unexpected: Vec<_> = report
        .issues
        .iter()
        .filter(|i| !matches!(i.check, CheckId::FaceOrientationConsistency))
        .collect();
    assert!(
        unexpected.is_empty(),
        "validate_solid reported issues beyond the known NURBS wall orientation \
         warnings: {unexpected:?}"
    );
    let shell_errors = report
        .issues
        .iter()
        .filter(|i| i.check == CheckId::ShellOrientationConsistent)
        .count();
    assert_eq!(
        shell_errors, 0,
        "shell orientation must validate cleanly, got {shell_errors} error(s): {:?}",
        report.issues
    );
    let flipped_faces = report
        .issues
        .iter()
        .filter(|i| i.check == CheckId::FaceOrientationConsistency)
        .count();
    assert_eq!(
        flipped_faces, expected_flipped_faces,
        "expected {expected_flipped_faces} FaceOrientationConsistency warning(s), \
         got {flipped_faces} — a face outside the hole walls has been flipped: {:?}",
        report.issues
    );

    let faces = remus_topology::explorer::solid_faces(&k.topo, solid_id).unwrap();
    assert_eq!(
        faces.len(),
        expected_faces,
        "face count — a count far above this means the profile was faceted \
         rather than swept exactly"
    );

    // Volume is measured at TWO mesh densities. A single measurement cannot
    // distinguish "correct" from "wrong but inside the band at this one
    // density"; geometry that is actually broken drifts as the mesh refines
    // instead of converging on the oracle. Measured here, the glyph goes
    // from 3.3e-4 relative error to 6.8e-5 — convergence, not drift.
    //
    // Both densities must be below the clamp `solid_volume` applies:
    // `volume_tessellation_deflection` caps the requested deflection at
    // `bbox_diag × 5e-5`, which is ~1.4e-3 for the annulus and ~1.0e-3 for
    // the glyph. A pair derived from the caller's (preview-tuned)
    // `deflection` would land above the cap and collapse to the SAME mesh,
    // making the comparison vacuous, so the pair is fixed rather than
    // derived (see the two constants above).
    let measure_volume = |k: &mut BrepKernel, defl: f64| -> f64 {
        run_all_ok(
            k,
            &[op(
                "volume",
                serde_json::json!({"solid": solid, "deflection": defl}),
            )],
        )[0]
        .as_f64()
        .unwrap()
    };
    let v = measure_volume(k, VOLUME_DEFLECTION_COARSE);
    let v_fine = measure_volume(k, VOLUME_DEFLECTION_FINE);
    for (label, measured) in [("coarse", v), ("fine", v_fine)] {
        let err = (measured - expected_volume).abs() / expected_volume.abs();
        assert!(
            err < rel_tol,
            "{label} volume {measured} vs expected {expected_volume} \
             (relative error {err:.3e} > {rel_tol:.3e})"
        );
    }
    // Refinement must not move the answer away from the oracle.
    let coarse_err = (v - expected_volume).abs();
    let fine_err = (v_fine - expected_volume).abs();
    assert!(
        fine_err <= coarse_err.mul_add(1.0, 1e-9),
        "volume diverges under refinement: {v} at deflection \
         {VOLUME_DEFLECTION_COARSE} but {v_fine} at {VOLUME_DEFLECTION_FINE} \
         (oracle {expected_volume}) — the signature of bad geometry"
    );

    // Volume alone cannot tell a solid with a real through-hole from one
    // whose hole was merely subtracted from the integral — probe the hole.
    let options = remus_check::classify::ClassifyOptions::default();
    for &(x, y, z) in inside_probes {
        let c = remus_check::classify::classify_point(
            &k.topo,
            solid_id,
            remus_math::vec::Point3::new(x, y, z),
            &options,
        )
        .unwrap();
        assert_eq!(
            c,
            remus_check::classify::PointClassification::Inside,
            "({x}, {y}, {z}) should be inside the material"
        );
    }
    for &(x, y, z) in outside_probes {
        let c = remus_check::classify::classify_point(
            &k.topo,
            solid_id,
            remus_math::vec::Point3::new(x, y, z),
            &options,
        )
        .unwrap();
        assert_eq!(
            c,
            remus_check::classify::PointClassification::Outside,
            "({x}, {y}, {z}) should be outside the material — the hole is \
             not actually open"
        );
    }
}

// ── (a) polygon annulus ───────────────────────────────────────────

#[test]
fn annulus_from_wires_extrudes_to_a_watertight_tube() {
    let outer = square(10.0, true);
    let hole = square(5.0, false);
    let depth = 5.0;
    // 20×20 outer minus 10×10 hole.
    let expected_volume = (400.0 - 100.0) * depth;

    // In the ring, in the hole, and clear of the body.
    let inside = [(7.5, 0.0, 2.5), (0.0, -7.5, 2.5)];
    let outside = [(0.0, 0.0, 2.5), (0.0, 0.0, 10.0), (30.0, 0.0, 2.5)];

    let (mut k, solid) = extrude_holed_face(&outer, &[hole], FaceApi::FromWires, depth);
    // 4 outer walls + 4 hole walls + 2 caps.
    assert_solid(
        &mut k,
        solid,
        10,
        0,
        expected_volume,
        1e-9,
        0.05,
        &inside,
        &outside,
    );
}

#[test]
fn annulus_via_add_holes_to_face_matches_make_face_from_wires() {
    let depth = 5.0;
    let expected_volume = (400.0 - 100.0) * depth;
    let (mut k, solid) = extrude_holed_face(
        &square(10.0, true),
        &[square(5.0, false)],
        FaceApi::AddHoles,
        depth,
    );
    assert_solid(
        &mut k,
        solid,
        10,
        0,
        expected_volume,
        1e-9,
        0.05,
        &[(7.5, 0.0, 2.5)],
        &[(0.0, 0.0, 2.5)],
    );
}

#[test]
fn annulus_with_two_disjoint_holes_extrudes_cleanly() {
    let outer = square(10.0, true);
    let hole_a = Loop {
        start: (-7.0, -3.0),
        segs: vec![
            Seg::Line(-7.0, 3.0),
            Seg::Line(-3.0, 3.0),
            Seg::Line(-3.0, -3.0),
            Seg::Line(-7.0, -3.0),
        ],
        z: 0.0,
    };
    let hole_b = Loop {
        start: (3.0, -3.0),
        segs: vec![
            Seg::Line(3.0, 3.0),
            Seg::Line(7.0, 3.0),
            Seg::Line(7.0, -3.0),
            Seg::Line(3.0, -3.0),
        ],
        z: 0.0,
    };
    assert!(hole_a.signed_area() < 0.0, "hole A should be CW");
    assert!(hole_b.signed_area() < 0.0, "hole B should be CW");

    let depth = 2.0;
    let expected_volume = (400.0 - 24.0 - 24.0) * depth;
    let (mut k, solid) = extrude_holed_face(&outer, &[hole_a, hole_b], FaceApi::FromWires, depth);
    // 4 outer walls + 4 + 4 hole walls + 2 caps.
    assert_solid(
        &mut k,
        solid,
        14,
        0,
        expected_volume,
        1e-9,
        0.05,
        // The bridge between the two holes, and the margin around them.
        &[(0.0, 0.0, 1.0), (0.0, 8.0, 1.0)],
        // Inside each hole.
        &[(-5.0, 0.0, 1.0), (5.0, 0.0, 1.0)],
    );
}

// ── (b) 'O'-like contour, lines mixed with beziers ────────────────

#[test]
fn o_glyph_contour_mixing_lines_and_beziers_extrudes_to_a_valid_solid() {
    let outer = capsule(4.0, 6.0, 5.0, true);
    let hole = capsule(2.0, 3.0, 3.0, false);
    assert!(outer.signed_area() > 0.0, "outer should be CCW");
    assert!(hole.signed_area() < 0.0, "hole should be CW");

    let depth = 3.0;
    let expected_volume = (outer.signed_area() + hole.signed_area()) * depth;

    let (mut k, solid) = extrude_holed_face(&outer, &[hole], FaceApi::FromWires, depth);
    // 4 outer walls (2 planar, 2 ruled NURBS) + 4 hole walls + 2 caps.
    // The caps are chorded by tessellation, so volume is checked at a
    // relative tolerance rather than exactly.
    //
    // Every probe below is on the x = 0 axis, where both contours are bounded
    // by their straight sides: material for 3 < |y| < 6, void for |y| < 3.
    // The bezier-cap band (|x| > 2) is probed separately by
    // `o_glyph_bezier_cap_band_classifies_correctly`.
    assert_solid(
        &mut k,
        solid,
        10,
        0,
        expected_volume,
        2e-3,
        0.005,
        // In the wall of the 'O', above and below the counter.
        &[(0.0, 4.5, 1.5), (0.0, -4.5, 1.5)],
        // In the counter, and clear of the glyph.
        &[(0.0, 0.0, 1.5), (0.0, 20.0, 1.5)],
    );
}

/// Regression pin for a classification defect in the bezier-cap band,
/// originally an `#[ignore]` ready-repro; the defect is fixed and this
/// test now passes unmodified (its original acceptance target).
///
/// Along `y = 0` the 'O' is bounded by the two bezier caps, and both
/// contours' caps are computable by hand: the hole's right cap is the cubic
/// `P0=(2,3) P1=(5,3) P2=(5,−3) P3=(2,−3)`, which at `t = 0.5` reaches
/// `x = (2 + 3·5 + 3·5 + 2)/8 = 4.25`; the outer's reaches `7.75` the same
/// way. So at `y = 0` the void is `|x| < 4.25` and the material ring is
/// `4.25 < |x| < 7.75`.
///
/// When broken, the solid classified the opposite way round and then
/// scrambled further out (Inside at 2.0–4.0, Outside at 4.5–6.0, then
/// alternating) while STILL being watertight, 10-faced, and matching the
/// independent shoelace volume oracle to 7e-5 — no other rung of the
/// ladder catches this class, which is why the probes are pinned here.
#[test]
fn o_glyph_bezier_cap_band_classifies_correctly() {
    let (k, solid) = extrude_holed_face(
        &capsule(4.0, 6.0, 5.0, true),
        &[capsule(2.0, 3.0, 3.0, false)],
        FaceApi::FromWires,
        3.0,
    );
    let solid_id = k.resolve_solid(solid).unwrap();
    let options = remus_check::classify::ClassifyOptions::default();
    let classify = |x: f64| {
        remus_check::classify::classify_point(
            &k.topo,
            solid_id,
            remus_math::vec::Point3::new(x, 0.0, 1.5),
            &options,
        )
        .unwrap()
    };
    // Inside the counter — the hole reaches x = 4.25 at y = 0.
    for x in [2.5, 3.0, 3.5, 4.0] {
        assert_eq!(
            classify(x),
            remus_check::classify::PointClassification::Outside,
            "({x}, 0, 1.5) is inside the counter and must classify Outside"
        );
    }
    // In the material ring, 4.25 < x < 7.75.
    for x in [4.5, 5.0, 6.0, 7.0] {
        assert_eq!(
            classify(x),
            remus_check::classify::PointClassification::Inside,
            "({x}, 0, 1.5) is in the material ring and must classify Inside"
        );
    }
}

#[test]
fn o_glyph_contour_via_add_holes_to_face_matches_make_face_from_wires() {
    let outer = capsule(4.0, 6.0, 5.0, true);
    let hole = capsule(2.0, 3.0, 3.0, false);
    let depth = 3.0;
    let expected_volume = (outer.signed_area() + hole.signed_area()) * depth;

    let (mut k, solid) = extrude_holed_face(&outer, &[hole], FaceApi::AddHoles, depth);
    assert_solid(
        &mut k,
        solid,
        10,
        0,
        expected_volume,
        2e-3,
        0.005,
        &[(0.0, 4.5, 1.5)],
        &[(0.0, 0.0, 1.5)],
    );
}

#[test]
fn glyph_side_walls_are_exact_nurbs_not_faceted() {
    // The bezier caps must become ruled NURBS side faces. If extrude ever
    // falls back to chording the profile, the face count explodes and the
    // face-count assertion above is the signal — this test states the
    // stronger property directly.
    let (k, solid) = extrude_holed_face(
        &capsule(4.0, 6.0, 5.0, true),
        &[capsule(2.0, 3.0, 3.0, false)],
        FaceApi::FromWires,
        3.0,
    );
    let solid_id = k.resolve_solid(solid).unwrap();
    let faces = remus_topology::explorer::solid_faces(&k.topo, solid_id).unwrap();
    let nurbs = faces
        .iter()
        .filter(|&&f| {
            matches!(
                k.topo.face(f).unwrap().surface(),
                remus_topology::face::FaceSurface::Nurbs(_)
            )
        })
        .count();
    assert_eq!(
        nurbs,
        4,
        "expected one NURBS wall per bezier segment (2 outer + 2 hole), got {nurbs} of {} faces",
        faces.len()
    );
}

// ── regression pins: extrude's shell orientation on holed profiles ─

/// Regression pin for extrude's shell orientation on holed profiles,
/// originally an `#[ignore]` ready-repro; the defect is fixed and this
/// test now passes unmodified (its original acceptance target).
///
/// When broken, extruding a face with an inner wire produced a shell in
/// which edges shared between two adjacent faces were traversed in the
/// SAME direction by both, where a closed oriented shell requires opposite
/// directions (`validate_solid` reported `ShellOrientationConsistent`
/// errors — 8 on this annulus, 16 on the 'O' glyph — plus one
/// `FaceOrientationConsistency` warning per hole wall at `dot = −1.000`).
/// The result was nonetheless watertight and of the right volume, so this
/// full `validate_solid` is_valid pin is the only rung that catches the
/// class. It matters for consumers that read orientation rather than
/// re-derive it (STEP export, GFA).
#[test]
fn extruded_annulus_shell_orientation_is_consistent() {
    let (k, solid) = extrude_holed_face(
        &square(10.0, true),
        &[square(5.0, false)],
        FaceApi::FromWires,
        5.0,
    );
    let solid_id = k.resolve_solid(solid).unwrap();
    let report = remus_check::validate::validate_solid(
        &k.topo,
        solid_id,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        report.is_valid(),
        "validate_solid found {} error(s): {:?}",
        report.error_count(),
        report.issues
    );
}

// ── validation: what addHolesToFace used to accept silently ───────

/// Build an outer square wire and return `(kernel, outer_wire_handle)`.
fn kernel_with_outer_square(half: f64) -> (BrepKernel, u32) {
    let mut k = BrepKernel::new();
    let outer = square(half, true);
    let mut ops = Vec::new();
    outer.build_ops(&mut ops);
    let edges = run_all_ok(&mut k, &ops);
    let handles: Vec<u32> = edges.iter().map(as_u32).collect();
    let w = run_all_ok(
        &mut k,
        &[op(
            "makeWire",
            serde_json::json!({"edges": handles, "closed": true}),
        )],
    );
    let wire = as_u32(&w[0]);
    (k, wire)
}

/// Build a loop's wire in an existing kernel and return its handle.
fn build_wire(k: &mut BrepKernel, l: &Loop, closed: bool) -> u32 {
    let mut ops = Vec::new();
    l.build_ops(&mut ops);
    let edges = run_all_ok(k, &ops);
    let handles: Vec<u32> = edges.iter().map(as_u32).collect();
    let w = run_all_ok(
        k,
        &[op(
            "makeWire",
            serde_json::json!({"edges": handles, "closed": closed}),
        )],
    );
    as_u32(&w[0])
}

#[test]
fn make_wire_rejects_a_non_boolean_closed_argument() {
    // `as_bool().unwrap_or(true)` used to turn each of these into a wire
    // flagged CLOSED — the opposite of what the caller asked for — and
    // `Wire::new` performs no closure validation, so nothing downstream
    // would have noticed except, by luck, `validate_hole_wires`.
    let mut k = BrepKernel::new();
    let mut ops = Vec::new();
    square(10.0, true).build_ops(&mut ops);
    let edges = run_all_ok(&mut k, &ops);
    let handles: Vec<u32> = edges.iter().map(as_u32).collect();
    for bad in [
        serde_json::json!(0),
        serde_json::json!("false"),
        serde_json::json!([]),
    ] {
        let msg = run_expect_last_error(
            &mut k,
            &[op(
                "makeWire",
                serde_json::json!({"edges": handles, "closed": bad}),
            )],
        );
        assert!(msg.contains("'closed'"), "message for {bad} was: {msg}");
    }
    // An absent `closed` still means closed.
    let r = run_all_ok(
        &mut k,
        &[op("makeWire", serde_json::json!({"edges": handles}))],
    );
    let wid = k.resolve_wire(as_u32(&r[0])).unwrap();
    assert!(k.topo.wire(wid).unwrap().is_closed());
}

#[test]
fn open_hole_wire_is_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    // Three sides of a square: a path, not a loop.
    let ops = [
        op(
            "makeLineEdge",
            serde_json::json!({"x1": -5.0, "y1": -5.0, "z1": 0.0,
                               "x2": -5.0, "y2":  5.0, "z2": 0.0}),
        ),
        op(
            "makeLineEdge",
            serde_json::json!({"x1": -5.0, "y1":  5.0, "z1": 0.0,
                               "x2":  5.0, "y2":  5.0, "z2": 0.0}),
        ),
        op(
            "makeLineEdge",
            serde_json::json!({"x1":  5.0, "y1":  5.0, "z1": 0.0,
                               "x2":  5.0, "y2": -5.0, "z2": 0.0}),
        ),
    ];
    let edges = run_all_ok(&mut k, &ops);
    let handles: Vec<u32> = edges.iter().map(as_u32).collect();
    let w = run_all_ok(
        &mut k,
        &[op(
            "makeWire",
            serde_json::json!({"edges": handles, "closed": false}),
        )],
    );
    let open_wire = as_u32(&w[0]);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [open_wire]}),
        )],
    );
    assert!(msg.contains("not a closed loop"), "message was: {msg}");
}

#[test]
fn hole_wire_off_the_face_plane_is_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let lifted = Loop {
        start: (-5.0, -5.0),
        segs: vec![
            Seg::Line(-5.0, 5.0),
            Seg::Line(5.0, 5.0),
            Seg::Line(5.0, -5.0),
            Seg::Line(-5.0, -5.0),
        ],
        z: 1.0,
    };
    let hole_wire = build_wire(&mut k, &lifted, true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(
        msg.contains("does not lie on the face's surface"),
        "message was: {msg}"
    );
}

#[test]
fn hole_wire_outside_the_outer_wire_is_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let outside = Loop {
        start: (20.0, 20.0),
        segs: vec![
            Seg::Line(20.0, 25.0),
            Seg::Line(25.0, 25.0),
            Seg::Line(25.0, 20.0),
            Seg::Line(20.0, 20.0),
        ],
        z: 0.0,
    };
    let hole_wire = build_wire(&mut k, &outside, true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(msg.contains("not contained"), "message was: {msg}");
}

#[test]
fn hole_wire_straddling_the_outer_boundary_is_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    // Half in, half out.
    let straddle = Loop {
        start: (5.0, -2.0),
        segs: vec![
            Seg::Line(5.0, 2.0),
            Seg::Line(15.0, 2.0),
            Seg::Line(15.0, -2.0),
            Seg::Line(5.0, -2.0),
        ],
        z: 0.0,
    };
    let hole_wire = build_wire(&mut k, &straddle, true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(msg.contains("not contained"), "message was: {msg}");
}

/// A concave, non-self-intersecting 'U' outer contour opening upward, CCW.
/// Area 400 − 12 × 16 = 208. The notch is x ∈ (−6, 6), y > −6.
fn u_shaped_outer() -> Loop {
    Loop {
        start: (-10.0, -10.0),
        segs: vec![
            Seg::Line(10.0, -10.0),
            Seg::Line(10.0, 10.0),
            Seg::Line(6.0, 10.0),
            Seg::Line(6.0, -6.0),
            Seg::Line(-6.0, -6.0),
            Seg::Line(-6.0, 10.0),
            Seg::Line(-10.0, 10.0),
            Seg::Line(-10.0, -10.0),
        ],
        z: 0.0,
    }
}

#[test]
fn a_hole_crossing_a_concave_outer_boundary_is_rejected() {
    // Point containment alone accepts this: all four corners of the bar sit
    // inside the two arms of the 'U', and only its middle is out in the
    // notch. Before the edge-crossing test was added, `makeFaceFromWires`
    // returned ok and the extruded result was not watertight (24 boundary
    // edges) with volume 352 where the material is 600. Concave outers are
    // the norm for the glyph case this API exists to serve, and every other
    // containment test here uses a convex square — where a straddling hole
    // necessarily puts a corner outside, so the point test alone suffices.
    let mut k = BrepKernel::new();
    let outer_wire = build_wire(&mut k, &u_shaped_outer(), true);
    let bar = Loop {
        start: (-8.0, 0.0),
        segs: vec![
            Seg::Line(-8.0, 2.0),
            Seg::Line(8.0, 2.0),
            Seg::Line(8.0, 0.0),
            Seg::Line(-8.0, 0.0),
        ],
        z: 0.0,
    };
    let hole_wire = build_wire(&mut k, &bar, true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(
        msg.contains("crosses the outer boundary"),
        "message was: {msg}"
    );
}

#[test]
fn a_hole_wholly_inside_a_concave_outer_wire_is_still_accepted() {
    // Guard against the crossing test over-rejecting: a hole inside one arm
    // of the 'U' is legitimate and must pass.
    let mut k = BrepKernel::new();
    let outer_wire = build_wire(&mut k, &u_shaped_outer(), true);
    let in_arm = Loop {
        start: (-9.0, 0.0),
        segs: vec![
            Seg::Line(-9.0, 4.0),
            Seg::Line(-7.0, 4.0),
            Seg::Line(-7.0, 0.0),
            Seg::Line(-9.0, 0.0),
        ],
        z: 0.0,
    };
    let hole_wire = build_wire(&mut k, &in_arm, true);
    let r = run_all_ok(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(r[0].as_u64().is_some());
}

#[test]
fn a_self_intersecting_hole_wire_is_rejected() {
    // A bowtie: topologically closed, coplanar, distinct from the outer wire
    // and with all four vertices strictly inside it, so every other check
    // passes it. With one hole the hole-vs-hole overlap pass never runs —
    // that check is loop-vs-loop, never loop-vs-itself. Before the
    // self-crossing test, this built a face and extruded to a shell with 16
    // boundary edges.
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let bowtie = Loop {
        start: (-5.0, -5.0),
        segs: vec![
            Seg::Line(5.0, 5.0),
            Seg::Line(5.0, -5.0),
            Seg::Line(-5.0, 5.0),
            Seg::Line(-5.0, -5.0),
        ],
        z: 0.0,
    };
    let hole_wire = build_wire(&mut k, &bowtie, true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(
        msg.contains("hole wire 0") && msg.contains("crosses itself"),
        "message was: {msg}"
    );
}

#[test]
fn a_self_intersecting_outer_wire_is_rejected() {
    let mut k = BrepKernel::new();
    let bowtie = Loop {
        start: (-10.0, -10.0),
        segs: vec![
            Seg::Line(10.0, 10.0),
            Seg::Line(10.0, -10.0),
            Seg::Line(-10.0, 10.0),
            Seg::Line(-10.0, -10.0),
        ],
        z: 0.0,
    };
    let outer_wire = build_wire(&mut k, &bowtie, true);
    let hole_wire = build_wire(&mut k, &square(2.0, false), true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(
        msg.contains("outer wire") && msg.contains("crosses itself"),
        "message was: {msg}"
    );
}

#[test]
fn a_curved_hole_edge_bulging_out_of_the_outer_wire_is_rejected() {
    // This is what pins `OPEN_CURVE_SAMPLES` above 1. The bezier runs from
    // (5, −5) to (5, 5) — both endpoints comfortably inside the half-10
    // outer square — with controls at x = 25, so it reaches x = 20 at
    // t = 0.5. Sampled only at its endpoints, the hole outlines as a plain
    // 10×10 square and is accepted; sampled at interior parameters, the
    // excursion is seen. Neither the point test nor the crossing test can
    // find an escape the outline never represents.
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let bulging = Loop {
        start: (5.0, -5.0),
        segs: vec![
            Seg::Cubic(25.0, -5.0, 25.0, 5.0, 5.0, 5.0),
            Seg::Line(-5.0, 5.0),
            Seg::Line(-5.0, -5.0),
            Seg::Line(5.0, -5.0),
        ],
        z: 0.0,
    };
    let hole_wire = build_wire(&mut k, &bulging, true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    assert!(msg.contains("not contained"), "message was: {msg}");
}

#[test]
fn the_outer_wire_cannot_be_its_own_hole() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [outer_wire]}),
        )],
    );
    // The distinguishing phrase, not the bare token "outer wire", which
    // "the outer wire is not planar" would also satisfy.
    assert!(
        msg.contains("is the face's own outer wire"),
        "message was: {msg}"
    );
}

#[test]
fn the_same_hole_listed_twice_is_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let hole_wire = build_wire(&mut k, &square(5.0, false), true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire, hole_wire]}),
        )],
    );
    assert!(msg.contains("listed twice"), "message was: {msg}");
}

#[test]
fn nested_holes_are_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let big = build_wire(&mut k, &square(6.0, false), true);
    let small = build_wire(&mut k, &square(3.0, false), true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [big, small]}),
        )],
    );
    assert!(msg.contains("overlaps"), "message was: {msg}");
}

#[test]
fn partially_overlapping_holes_are_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    // Two rectangles that cross, neither containing the other.
    let a = Loop {
        start: (-6.0, -1.0),
        segs: vec![
            Seg::Line(-6.0, 1.0),
            Seg::Line(2.0, 1.0),
            Seg::Line(2.0, -1.0),
            Seg::Line(-6.0, -1.0),
        ],
        z: 0.0,
    };
    let b = Loop {
        start: (-1.0, -6.0),
        segs: vec![
            Seg::Line(-1.0, 2.0),
            Seg::Line(1.0, 2.0),
            Seg::Line(1.0, -6.0),
            Seg::Line(-1.0, -6.0),
        ],
        z: 0.0,
    };
    let wa = build_wire(&mut k, &a, true);
    let wb = build_wire(&mut k, &b, true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [wa, wb]}),
        )],
    );
    assert!(msg.contains("overlaps"), "message was: {msg}");
}

#[test]
fn adding_a_hole_that_duplicates_an_existing_one_is_rejected() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let hole_wire = build_wire(&mut k, &square(5.0, false), true);
    let faces = run_all_ok(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    let face = as_u32(&faces[0]);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "addHolesToFace",
            serde_json::json!({"face": face, "holeWires": [hole_wire]}),
        )],
    );
    assert!(msg.contains("already an inner wire"), "message was: {msg}");
}

#[test]
fn adding_a_hole_that_geometrically_overlaps_an_existing_one_is_rejected() {
    // The DISTINCT-wire case. `adding_a_hole_that_duplicates_an_existing_one`
    // above passes the same handle back, which the cheap identity check
    // catches before any geometry runs — so without this test the whole
    // new-hole-vs-existing-hole comparison could be deleted and the suite
    // would stay green. Here the second wire is a different wire nested
    // inside the first, which only the geometric pass can see.
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let existing = build_wire(&mut k, &square(6.0, false), true);
    let faces = run_all_ok(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [existing]}),
        )],
    );
    let face = as_u32(&faces[0]);
    let nested = build_wire(&mut k, &square(3.0, false), true);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "addHolesToFace",
            serde_json::json!({"face": face, "holeWires": [nested]}),
        )],
    );
    assert!(
        msg.contains("overlaps an existing inner wire"),
        "message was: {msg}"
    );
}

#[test]
fn add_holes_to_face_rejects_an_invalid_wire_handle() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let faces = run_all_ok(
        &mut k,
        &[op(
            "makePlanarFaceFromWire",
            serde_json::json!({"wire": outer_wire}),
        )],
    );
    let face = as_u32(&faces[0]);
    let msg = run_expect_last_error(
        &mut k,
        &[op(
            "addHolesToFace",
            serde_json::json!({"face": face, "holeWires": [9999]}),
        )],
    );
    // The distinguishing phrase: a bare "wire" is shared by nearly every
    // error this API can emit, so it would not tell a bad handle from a
    // handle that resolved to some unrelated wire and failed a later check.
    assert!(msg.contains("invalid wire handle"), "message was: {msg}");
}

#[test]
fn a_valid_hole_is_still_accepted_after_hardening() {
    // Guard against the checks over-rejecting: the ordinary case must pass.
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let hole_wire = build_wire(&mut k, &square(5.0, false), true);
    let r = run_all_ok(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [hole_wire]}),
        )],
    );
    let face_id = k.resolve_face(as_u32(&r[0])).unwrap();
    assert_eq!(k.topo.face(face_id).unwrap().inner_wires().len(), 1);
}

#[test]
fn a_ccw_hole_is_accepted_because_extrude_handles_either_winding() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let ccw_hole = build_wire(&mut k, &square(5.0, true), true);
    let r = run_all_ok(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire, "innerWires": [ccw_hole]}),
        )],
    );
    assert!(r[0].as_u64().is_some());
}

#[test]
fn make_face_from_wires_with_no_holes_is_a_plain_planar_face() {
    let (mut k, outer_wire) = kernel_with_outer_square(10.0);
    let r = run_all_ok(
        &mut k,
        &[op(
            "makeFaceFromWires",
            serde_json::json!({"outerWire": outer_wire}),
        )],
    );
    let face_id = k.resolve_face(as_u32(&r[0])).unwrap();
    let face = k.topo.face(face_id).unwrap();
    assert!(face.inner_wires().is_empty());
    assert!(matches!(
        face.surface(),
        remus_topology::face::FaceSurface::Plane { .. }
    ));
}

#[test]
fn make_face_from_wires_rejects_a_non_planar_outer_wire() {
    let mut k = BrepKernel::new();
    let skew = Loop {
        start: (0.0, 0.0),
        segs: vec![
            Seg::Line(10.0, 0.0),
            Seg::Line(10.0, 10.0),
            Seg::Line(0.0, 10.0),
            Seg::Line(0.0, 0.0),
        ],
        z: 0.0,
    };
    let mut ops = Vec::new();
    skew.build_ops(&mut ops);
    // Replace one edge with a lifted one so the loop is not coplanar.
    ops[1] = op(
        "makeLineEdge",
        serde_json::json!({"x1": 10.0, "y1": 0.0, "z1": 0.0, "x2": 10.0, "y2": 10.0, "z2": 6.0}),
    );
    ops[2] = op(
        "makeLineEdge",
        serde_json::json!({"x1": 10.0, "y1": 10.0, "z1": 6.0, "x2": 0.0, "y2": 10.0, "z2": 0.0}),
    );
    let edges = run_all_ok(&mut k, &ops);
    let handles: Vec<u32> = edges.iter().map(as_u32).collect();
    let msg = run_expect_last_error(
        &mut k,
        &[
            op(
                "makeWire",
                serde_json::json!({"edges": handles, "closed": true}),
            ),
            op("makeFaceFromWires", serde_json::json!({"outerWire": 0})),
        ],
    );
    assert!(msg.to_lowercase().contains("planar"), "message was: {msg}");
}
