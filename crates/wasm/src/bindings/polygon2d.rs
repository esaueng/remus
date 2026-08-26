//! 2D polygon operation bindings.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use crate::error::{
    WasmError, validate_all_finite, validate_finite, validate_positive, validate_work_product,
};
use crate::helpers::{parse_polygon_2d, parse_polygon_2d_checked, polygons_overlap_2d};
use crate::kernel::BrepKernel;
use crate::types::PolygonBoolean2dResult;
use remus_math::polygon_boolean::{BooleanOp as PolyBooleanOp, polygon_boolean};
use remus_math::polygon2d::{
    chamfer_polygon_2d, fillet_polygon_2d, find_common_segments, sutherland_hodgman_clip,
};

/// Parse the `operation` string accepted by `polygonBoolean2d`.
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] for any name outside the three
/// supported operations.
pub fn parse_polygon_boolean_op(name: &str) -> Result<PolyBooleanOp, WasmError> {
    match name {
        "union" => Ok(PolyBooleanOp::Union),
        "intersection" => Ok(PolyBooleanOp::Intersection),
        "difference" => Ok(PolyBooleanOp::Difference),
        other => Err(WasmError::InvalidInput {
            reason: format!(
                "unknown polygon boolean operation '{other}' \
                 (expected 'union', 'intersection', or 'difference')"
            ),
        }),
    }
}

/// Resolve the optional caller tolerance to a usable absolute linear value.
///
/// `None` (and only `None`) falls back to the workspace linear tolerance;
/// an explicitly supplied non-positive or non-finite value is an error
/// rather than a silent substitution, because
/// [`polygon_boolean`] answers an empty result for such a tolerance and the
/// caller would read that as "the polygons do not overlap".
fn resolve_polygon_tolerance(tolerance: Option<f64>) -> Result<f64, WasmError> {
    match tolerance {
        None => Ok(remus_math::tolerance::Tolerance::new().linear),
        Some(t) => {
            validate_positive(t, "tolerance")?;
            Ok(t)
        }
    }
}

/// Shared implementation behind `polygonUnion2d` / `polygonBoolean2d`.
///
/// Returns a [`WasmError`] (not a `JsError`) so it stays callable from
/// native tests and from `executeBatch` dispatch.
///
/// Bound a polygon-pair query whose cost is the product of the two vertex
/// counts, before any parsing or geometry work starts.
///
/// `polygonBoolean2d` grew this bound when its own report landed; the three
/// siblings below share its O(n·m) shape and were left without one. A JS caller
/// can hand any of them two large arrays and stall the tab with no error, so
/// they take the same budget rather than each waiting for its own report.
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] if either polygon has more vertices than
/// a `u32` can hold, or if the product exceeds the public WASM work budget.
fn bound_polygon_pair(coords_a: &[f64], coords_b: &[f64]) -> Result<(), WasmError> {
    let vertices_a = u32::try_from(coords_a.len() / 2).map_err(|_| WasmError::InvalidInput {
        reason: "polygon A has too many vertices".to_string(),
    })?;
    let vertices_b = u32::try_from(coords_b.len() / 2).map_err(|_| WasmError::InvalidInput {
        reason: "polygon B has too many vertices".to_string(),
    })?;
    let _ = validate_work_product(vertices_a, vertices_b, "polygon edge comparisons")?;
    Ok(())
}

/// # Errors
///
/// Returns [`WasmError::InvalidInput`] if either coordinate array is
/// malformed (odd length, fewer than 3 points, non-finite values) or the
/// supplied tolerance is not positive and finite, or if the pair of polygons
/// exceeds the public WASM work budget.
pub fn polygon_boolean_2d_impl(
    coords_a: &[f64],
    coords_b: &[f64],
    op: PolyBooleanOp,
    tolerance: Option<f64>,
) -> Result<PolygonBoolean2dResult, WasmError> {
    let vertices_a = u32::try_from(coords_a.len() / 2).map_err(|_| WasmError::InvalidInput {
        reason: "polygon A has too many vertices".to_string(),
    })?;
    let vertices_b = u32::try_from(coords_b.len() / 2).map_err(|_| WasmError::InvalidInput {
        reason: "polygon B has too many vertices".to_string(),
    })?;
    let _ = validate_work_product(vertices_a, vertices_b, "polygon edge comparisons")?;

    let poly_a = parse_polygon_2d_checked(coords_a, "polygon A")?;
    let poly_b = parse_polygon_2d_checked(coords_b, "polygon B")?;
    let tol = resolve_polygon_tolerance(tolerance)?;

    let result = polygon_boolean(&poly_a, &poly_b, op, tol);
    let flatten = |loops: &[Vec<remus_math::vec::Point2>]| -> Vec<Vec<f64>> {
        loops
            .iter()
            .map(|l| l.iter().flat_map(|p| [p.x(), p.y()]).collect())
            .collect()
    };
    Ok(PolygonBoolean2dResult {
        outer: flatten(&result.outer),
        holes: flatten(&result.holes),
    })
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Batch 6: Polygon offset ──────────────────────────────────────

    /// Offset a 2D polygon by a signed distance.
    ///
    /// `coords` is a flat array `[x,y, x,y, ...]` of 2D points.
    /// Returns a flat array of offset polygon coordinates.
    #[wasm_bindgen(js_name = "offsetPolygon2d")]
    #[allow(clippy::needless_pass_by_value, clippy::unused_self)]
    pub fn offset_polygon_2d(
        &self,
        coords: Vec<f64>,
        distance: f64,
        tolerance: f64,
    ) -> Result<Vec<f64>, JsError> {
        if !coords.len().is_multiple_of(2) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "2D coordinate array length must be even, got {}",
                    coords.len()
                ),
            }
            .into());
        }
        validate_all_finite(&coords, "coords")?;
        validate_finite(distance, "distance")?;
        validate_positive(tolerance, "tolerance")?;
        let points: Vec<remus_math::vec::Point2> = coords
            .chunks_exact(2)
            .map(|c| remus_math::vec::Point2::new(c[0], c[1]))
            .collect();
        let result = remus_math::polygon_offset::offset_polygon_2d(&points, distance, tolerance)?;
        Ok(result.iter().flat_map(|p| [p.x(), p.y()]).collect())
    }

    // ── 2D Blueprint Operations ────────────────────────────────────

    /// Test if a 2D point is inside a closed polygon.
    ///
    /// `polygon_coords` is a flat array `[x,y, x,y, ...]`.
    /// Returns `true` if the point is inside the polygon (winding number test).
    #[wasm_bindgen(js_name = "pointInPolygon2d")]
    #[allow(clippy::unused_self)]
    pub fn point_in_polygon_2d(
        &self,
        polygon_coords: Vec<f64>,
        px: f64,
        py: f64,
    ) -> Result<bool, JsError> {
        if !polygon_coords.len().is_multiple_of(2) || polygon_coords.len() < 6 {
            return Err(WasmError::InvalidInput {
                reason: "polygon needs at least 3 points (6 coordinates)".into(),
            }
            .into());
        }
        validate_all_finite(&polygon_coords, "polygon_coords")?;
        validate_finite(px, "px")?;
        validate_finite(py, "py")?;
        let polygon: Vec<remus_math::vec::Point2> = polygon_coords
            .chunks_exact(2)
            .map(|c| remus_math::vec::Point2::new(c[0], c[1]))
            .collect();
        let point = remus_math::vec::Point2::new(px, py);
        Ok(remus_math::predicates::point_in_polygon(point, &polygon))
    }

    /// Test if two 2D polygons intersect (overlap).
    ///
    /// Both polygons are flat arrays `[x,y, x,y, ...]`.
    /// Returns `true` if any vertex of one polygon is inside the other
    /// or if any edges cross.
    #[wasm_bindgen(js_name = "polygonsIntersect2d")]
    #[allow(clippy::unused_self)]
    pub fn polygons_intersect_2d(
        &self,
        coords_a: Vec<f64>,
        coords_b: Vec<f64>,
    ) -> Result<bool, JsError> {
        bound_polygon_pair(&coords_a, &coords_b)?;
        let poly_a = parse_polygon_2d(&coords_a)?;
        let poly_b = parse_polygon_2d(&coords_b)?;
        Ok(polygons_overlap_2d(&poly_a, &poly_b))
    }

    /// Compute the boolean intersection of two 2D polygons.
    ///
    /// Both polygons are flat arrays `[x,y, x,y, ...]`.
    /// Returns a flat array of the intersection polygon coordinates,
    /// or an empty array if they don't intersect.
    ///
    /// Uses the Sutherland-Hodgman algorithm (convex clipper).
    #[wasm_bindgen(js_name = "intersectPolygons2d")]
    #[allow(clippy::unused_self)]
    pub fn intersect_polygons_2d(
        &self,
        coords_a: Vec<f64>,
        coords_b: Vec<f64>,
    ) -> Result<Vec<f64>, JsError> {
        bound_polygon_pair(&coords_a, &coords_b)?;
        let subject = parse_polygon_2d(&coords_a)?;
        let clip = parse_polygon_2d(&coords_b)?;
        let result = sutherland_hodgman_clip(&subject, &clip);
        Ok(result.iter().flat_map(|p| [p.x(), p.y()]).collect())
    }

    /// Find common (shared) edges between two adjacent 2D polygons.
    ///
    /// Both polygons are flat arrays `[x,y, x,y, ...]`.
    /// Returns a flat array of common segment endpoints `[x1,y1, x2,y2, ...]`,
    /// or an empty array if no common segments exist.
    #[wasm_bindgen(js_name = "commonSegment2d")]
    #[allow(clippy::unused_self)]
    pub fn common_segment_2d(
        &self,
        coords_a: Vec<f64>,
        coords_b: Vec<f64>,
    ) -> Result<Vec<f64>, JsError> {
        bound_polygon_pair(&coords_a, &coords_b)?;
        let poly_a = parse_polygon_2d(&coords_a)?;
        let poly_b = parse_polygon_2d(&coords_b)?;
        let tolerance = 1e-7;
        let result = find_common_segments(&poly_a, &poly_b, tolerance);
        Ok(result
            .iter()
            .flat_map(|(a, b)| [a.x(), a.y(), b.x(), b.y()])
            .collect())
    }

    /// Union two 2D polygons with the robust arrangement-based engine.
    ///
    /// Both polygons are flat arrays `[x,y, x,y, ...]`; either winding is
    /// accepted (orientation is normalized internally). `tolerance` is an
    /// absolute linear tolerance in the polygons' own units; pass `null`
    /// or `undefined` for the kernel default (1e-7).
    ///
    /// Returns a JSON string
    /// `{"outer": [[x,y,...], ...], "holes": [[x,y,...], ...]}`
    /// (the `PolygonBoolean2dResult` TypeScript type). Outer loops are
    /// counter-clockwise, hole loops clockwise, and each loop is implicitly
    /// closed. A disjoint union yields several `outer` loops; a union that
    /// encloses a void yields a `holes` entry — unlike
    /// [`intersectPolygons2d`](Self::intersect_polygons_2d), which is a
    /// convex-only Sutherland–Hodgman clipper returning a single loop.
    ///
    /// Both result lists are empty when the operation produces no geometry.
    #[wasm_bindgen(js_name = "polygonUnion2d")]
    #[allow(clippy::needless_pass_by_value, clippy::unused_self)]
    pub fn polygon_union_2d(
        &self,
        coords_a: Vec<f64>,
        coords_b: Vec<f64>,
        tolerance: Option<f64>,
    ) -> Result<JsValue, JsError> {
        let result =
            polygon_boolean_2d_impl(&coords_a, &coords_b, PolyBooleanOp::Union, tolerance)?;
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }

    /// Boolean of two 2D polygons: `"union"`, `"intersection"`, or
    /// `"difference"` (`A \ B`).
    ///
    /// Encoding, winding, and tolerance semantics are identical to
    /// [`polygonUnion2d`](Self::polygon_union_2d).
    #[wasm_bindgen(js_name = "polygonBoolean2d")]
    #[allow(clippy::needless_pass_by_value, clippy::unused_self)]
    pub fn polygon_boolean_2d(
        &self,
        coords_a: Vec<f64>,
        coords_b: Vec<f64>,
        operation: &str,
        tolerance: Option<f64>,
    ) -> Result<JsValue, JsError> {
        let op = parse_polygon_boolean_op(operation)?;
        let result = polygon_boolean_2d_impl(&coords_a, &coords_b, op, tolerance)?;
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }

    /// Round corners of a 2D polygon by inserting arc-approximation vertices.
    ///
    /// `coords` is a flat array `[x,y, x,y, ...]`.
    /// `radius` is the fillet radius.
    /// Returns a flat array of the filleted polygon coordinates.
    #[wasm_bindgen(js_name = "fillet2d")]
    #[allow(clippy::unused_self)]
    pub fn fillet_2d(&self, coords: Vec<f64>, radius: f64) -> Result<Vec<f64>, JsError> {
        validate_positive(radius, "radius")?;
        let polygon = parse_polygon_2d(&coords)?;
        let result = fillet_polygon_2d(&polygon, radius);
        Ok(result.iter().flat_map(|p| [p.x(), p.y()]).collect())
    }

    /// Cut corners of a 2D polygon with flat bevels.
    ///
    /// `coords` is a flat array `[x,y, x,y, ...]`.
    /// `distance` is the chamfer distance from each corner.
    /// Returns a flat array of the chamfered polygon coordinates.
    #[wasm_bindgen(js_name = "chamfer2d")]
    #[allow(clippy::unused_self)]
    pub fn chamfer_2d(&self, coords: Vec<f64>, distance: f64) -> Result<Vec<f64>, JsError> {
        validate_positive(distance, "distance")?;
        let polygon = parse_polygon_2d(&coords)?;
        let result = chamfer_polygon_2d(&polygon, distance);
        Ok(result.iter().flat_map(|p| [p.x(), p.y()]).collect())
    }
}

#[cfg(test)]
mod polygon_boolean_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;

    /// Axis-aligned square with lower-left corner `(x, y)` and side `s`,
    /// as a flat coordinate array.
    fn sq(x: f64, y: f64, s: f64) -> Vec<f64> {
        vec![x, y, x + s, y, x + s, y + s, x, y + s]
    }

    /// Shoelace signed area of a flat `[x,y,...]` loop.
    fn signed_area(loop_coords: &[f64]) -> f64 {
        let n = loop_coords.len() / 2;
        let mut acc = 0.0;
        for i in 0..n {
            let j = (i + 1) % n;
            acc += loop_coords[2 * i].mul_add(
                loop_coords[2 * j + 1],
                -(loop_coords[2 * j] * loop_coords[2 * i + 1]),
            );
        }
        acc / 2.0
    }

    /// Run one batch op and return the `ok` payload, panicking with the
    /// kernel's own message if the op errored.
    fn run_ok(op: &str, args: serde_json::Value) -> serde_json::Value {
        let mut k = BrepKernel::new();
        let json = serde_json::json!([{"op": op, "args": args}]).to_string();
        let raw = k.execute_batch(&json);
        let parsed: Vec<serde_json::Value> = serde_json::from_str(&raw).unwrap();
        assert_eq!(parsed.len(), 1, "one op in, one result out");
        match parsed[0].get("ok") {
            Some(v) => v.clone(),
            None => panic!("expected ok, got {}", parsed[0]),
        }
    }

    /// Run one batch op and return the `error` message.
    fn run_err(op: &str, args: serde_json::Value) -> String {
        let mut k = BrepKernel::new();
        let json = serde_json::json!([{"op": op, "args": args}]).to_string();
        let raw = k.execute_batch(&json);
        let parsed: Vec<serde_json::Value> = serde_json::from_str(&raw).unwrap();
        match parsed[0].get("error").and_then(serde_json::Value::as_str) {
            Some(s) => s.to_string(),
            None => panic!("expected error, got {}", parsed[0]),
        }
    }

    /// Even-odd point-in-polygon on a flat `[x,y,...]` loop, so a test can
    /// assert the SHAPE of a result loop and not only its area.
    fn point_in_loop(loop_coords: &[f64], x: f64, y: f64) -> bool {
        let n = loop_coords.len() / 2;
        let mut inside = false;
        for i in 0..n {
            let j = (i + n - 1) % n;
            let (xi, yi) = (loop_coords[2 * i], loop_coords[2 * i + 1]);
            let (xj, yj) = (loop_coords[2 * j], loop_coords[2 * j + 1]);
            if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
                inside = !inside;
            }
        }
        inside
    }

    /// Pull `outer` / `holes` out of the payload as flat coordinate loops.
    fn loops(payload: &serde_json::Value) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let take = |key: &str| -> Vec<Vec<f64>> {
            payload[key]
                .as_array()
                .unwrap_or_else(|| panic!("missing '{key}' in {payload}"))
                .iter()
                .map(|l| {
                    l.as_array()
                        .unwrap()
                        .iter()
                        .map(|v| v.as_f64().unwrap())
                        .collect()
                })
                .collect()
        };
        (take("outer"), take("holes"))
    }

    // ── polygonUnion2d ────────────────────────────────────────────

    #[test]
    fn union_of_overlapping_squares_is_one_outer_loop() {
        let payload = run_ok(
            "polygonUnion2d",
            serde_json::json!({"coordsA": sq(0.0, 0.0, 10.0), "coordsB": sq(5.0, 5.0, 10.0)}),
        );
        let (outer, holes) = loops(&payload);
        assert_eq!(outer.len(), 1, "overlapping squares merge into one loop");
        assert!(holes.is_empty(), "no void enclosed");
        // 100 + 100 − 25 overlap.
        assert!(
            (signed_area(&outer[0]) - 175.0).abs() < 1e-6,
            "union area was {}",
            signed_area(&outer[0])
        );
        // Area alone does not pin the shape: an axis-aligned rectangle of
        // 175 units, or a wrongly ordered ring, would satisfy it. The true
        // union is an 8-vertex L, and the notch at (12, 2) is outside it.
        assert_eq!(
            outer[0].len(),
            16,
            "the union of two offset squares is an 8-vertex L-shaped ring, got {:?}",
            outer[0]
        );
        assert!(
            !point_in_loop(&outer[0], 12.0, 2.0),
            "(12, 2) is in the notch of the L and must be outside the union"
        );
        assert!(
            point_in_loop(&outer[0], 2.0, 2.0) && point_in_loop(&outer[0], 12.0, 12.0),
            "both arms of the L must be inside the union"
        );
    }

    #[test]
    fn union_of_disjoint_squares_keeps_both_outer_loops() {
        let payload = run_ok(
            "polygonUnion2d",
            serde_json::json!({"coordsA": sq(0.0, 0.0, 1.0), "coordsB": sq(5.0, 5.0, 1.0)}),
        );
        let (outer, holes) = loops(&payload);
        assert_eq!(
            outer.len(),
            2,
            "a flat-array encoding would have fused these two loops"
        );
        assert!(holes.is_empty());
    }

    #[test]
    fn union_outer_loops_are_counter_clockwise_whatever_the_input_winding() {
        // `sq` emits CCW, so all four permutations have to be spelled out
        // rather than assumed.
        let cw = |x: f64, y: f64, s: f64| vec![x, y, x, y + s, x + s, y + s, x + s, y];
        let a_cw = cw(0.0, 0.0, 10.0);
        let a_ccw = sq(0.0, 0.0, 10.0);
        let b_cw = cw(5.0, 5.0, 10.0);
        let b_ccw = sq(5.0, 5.0, 10.0);
        assert!(signed_area(&a_cw) < 0.0 && signed_area(&a_ccw) > 0.0);
        for (label, a, b) in [
            ("CW + CW", &a_cw, &b_cw),
            ("CW + CCW", &a_cw, &b_ccw),
            ("CCW + CW", &a_ccw, &b_cw),
            ("CCW + CCW", &a_ccw, &b_ccw),
        ] {
            let payload = run_ok(
                "polygonUnion2d",
                serde_json::json!({"coordsA": a, "coordsB": b}),
            );
            let (outer, _) = loops(&payload);
            assert_eq!(outer.len(), 1, "{label}");
            assert!(
                signed_area(&outer[0]) > 0.0,
                "{label}: outer loop must be CCW (positive signed area)"
            );
            assert!(
                (signed_area(&outer[0]) - 175.0).abs() < 1e-6,
                "{label}: union area was {}",
                signed_area(&outer[0])
            );
        }
    }

    // ── polygonBoolean2d ──────────────────────────────────────────

    #[test]
    fn difference_reports_the_punched_void_as_a_hole() {
        let payload = run_ok(
            "polygonBoolean2d",
            serde_json::json!({
                "coordsA": sq(0.0, 0.0, 10.0),
                "coordsB": sq(3.0, 3.0, 2.0),
                "operation": "difference",
            }),
        );
        let (outer, holes) = loops(&payload);
        assert_eq!(outer.len(), 1, "outer boundary preserved");
        assert_eq!(holes.len(), 1, "punched void is a hole, not a second outer");
        assert!(
            (signed_area(&outer[0]) - 100.0).abs() < 1e-6,
            "outer area was {}",
            signed_area(&outer[0])
        );
        assert!(
            signed_area(&holes[0]) < 0.0,
            "hole loop must be CW (negative signed area)"
        );
        assert!(
            (signed_area(&holes[0]) + 4.0).abs() < 1e-6,
            "hole area was {}",
            signed_area(&holes[0])
        );
        assert_eq!(outer[0].len(), 8, "outer is the original 4-vertex square");
        assert_eq!(holes[0].len(), 8, "hole is the 4-vertex punched square");
        // The hole sits where B was, not somewhere else of the same area.
        assert!(point_in_loop(&holes[0], 4.0, 4.0));
        assert!(!point_in_loop(&holes[0], 8.0, 8.0));
    }

    #[test]
    fn intersection_returns_the_overlap() {
        let payload = run_ok(
            "polygonBoolean2d",
            serde_json::json!({
                "coordsA": sq(0.0, 0.0, 10.0),
                "coordsB": sq(3.0, 3.0, 2.0),
                "operation": "intersection",
            }),
        );
        let (outer, holes) = loops(&payload);
        assert_eq!(outer.len(), 1);
        assert!(holes.is_empty());
        assert!((signed_area(&outer[0]) - 4.0).abs() < 1e-6);
        assert_eq!(outer[0].len(), 8, "the overlap is B itself, 4 vertices");
        assert!(point_in_loop(&outer[0], 4.0, 4.0));
        assert!(!point_in_loop(&outer[0], 1.0, 1.0));
    }

    #[test]
    fn union_via_polygon_boolean_matches_polygon_union() {
        let args = serde_json::json!({
            "coordsA": sq(0.0, 0.0, 10.0),
            "coordsB": sq(5.0, 5.0, 10.0),
        });
        let via_union = run_ok("polygonUnion2d", args.clone());
        let mut boolean_args = args;
        boolean_args["operation"] = serde_json::json!("union");
        let via_boolean = run_ok("polygonBoolean2d", boolean_args);
        assert_eq!(via_union, via_boolean);
    }

    #[test]
    fn disjoint_intersection_is_empty_not_an_error() {
        let payload = run_ok(
            "polygonBoolean2d",
            serde_json::json!({
                "coordsA": sq(0.0, 0.0, 1.0),
                "coordsB": sq(5.0, 5.0, 1.0),
                "operation": "intersection",
            }),
        );
        let (outer, holes) = loops(&payload);
        assert!(outer.is_empty());
        assert!(holes.is_empty());
    }

    // ── error paths ───────────────────────────────────────────────

    #[test]
    fn unknown_operation_is_rejected() {
        let msg = run_err(
            "polygonBoolean2d",
            serde_json::json!({
                "coordsA": sq(0.0, 0.0, 1.0),
                "coordsB": sq(5.0, 5.0, 1.0),
                "operation": "xor",
            }),
        );
        assert!(msg.contains("xor"), "message was: {msg}");
    }

    #[test]
    fn odd_coordinate_count_is_rejected() {
        let msg = run_err(
            "polygonUnion2d",
            serde_json::json!({
                "coordsA": vec![0.0, 0.0, 1.0, 0.0, 1.0],
                "coordsB": sq(0.0, 0.0, 1.0),
            }),
        );
        assert!(msg.contains("polygon A"), "message was: {msg}");
    }

    #[test]
    fn non_finite_coordinate_is_rejected() {
        let mut coords = sq(0.0, 0.0, 1.0);
        coords[3] = f64::NAN;
        // serde_json cannot encode NaN, so go through the impl directly —
        // this is the same guard the batch path hits for an `Infinity` literal.
        let err = polygon_boolean_2d_impl(&coords, &sq(0.0, 0.0, 1.0), PolyBooleanOp::Union, None)
            .unwrap_err();
        assert!(err.to_string().contains("not finite"), "was: {err}");
    }

    #[test]
    fn non_positive_tolerance_is_rejected_rather_than_silently_defaulted() {
        let msg = run_err(
            "polygonUnion2d",
            serde_json::json!({
                "coordsA": sq(0.0, 0.0, 10.0),
                "coordsB": sq(5.0, 5.0, 10.0),
                "tolerance": 0.0,
            }),
        );
        assert!(msg.contains("tolerance"), "message was: {msg}");
    }

    #[test]
    fn polygon_boolean_work_above_the_public_budget_is_rejected() {
        let polygon = vec![0.0; 202];
        let err = polygon_boolean_2d_impl(&polygon, &polygon, PolyBooleanOp::Intersection, None)
            .unwrap_err();
        assert!(
            err.to_string().contains("polygon edge comparisons"),
            "was: {err}"
        );
    }

    /// `polygonBoolean2d` grew a work bound when its own report landed; the
    /// three siblings that share its O(n·m) shape were left unbounded, so a JS
    /// caller could stall the tab with no error. They take the same budget now.
    #[test]
    fn polygon_pair_siblings_share_the_public_work_budget() {
        let polygon = vec![0.0; 202];
        let err = bound_polygon_pair(&polygon, &polygon).unwrap_err();
        assert!(
            err.to_string().contains("polygon edge comparisons"),
            "was: {err}"
        );
        // A pair inside the budget is untouched.
        let small = vec![0.0; 20];
        assert!(bound_polygon_pair(&small, &small).is_ok());
    }

    #[test]
    fn absent_tolerance_uses_the_kernel_default() {
        let with_default = run_ok(
            "polygonUnion2d",
            serde_json::json!({"coordsA": sq(0.0, 0.0, 10.0), "coordsB": sq(5.0, 5.0, 10.0)}),
        );
        let explicit = run_ok(
            "polygonUnion2d",
            serde_json::json!({
                "coordsA": sq(0.0, 0.0, 10.0),
                "coordsB": sq(5.0, 5.0, 10.0),
                "tolerance": remus_math::tolerance::Tolerance::new().linear,
            }),
        );
        assert_eq!(with_default, explicit);
    }

    #[test]
    fn the_caller_tolerance_actually_reaches_the_boolean_engine() {
        // The test above cannot fail on an implementation that parses the
        // caller's tolerance and then throws it away, because it compares
        // the default against an explicit copy of the default. This one
        // demands a result that CHANGES with the tolerance.
        //
        // Two 10×10 squares separated by a 1e-3 gap. Below the gap the union
        // is two disjoint loops; above it the near-coincident edges snap
        // together and the union is a single 6-vertex loop.
        let a = sq(0.0, 0.0, 10.0);
        let b = sq(10.001, 0.0, 10.0);
        let union_at = |tol: Option<f64>| -> Vec<Vec<f64>> {
            let mut args = serde_json::json!({"coordsA": a, "coordsB": b});
            if let Some(t) = tol {
                args["tolerance"] = serde_json::json!(t);
            }
            loops(&run_ok("polygonUnion2d", args)).0
        };

        let fine = union_at(Some(1e-4));
        assert_eq!(
            fine.len(),
            2,
            "at tolerance 1e-4 the 1e-3 gap is real and the squares stay apart"
        );

        let coarse = union_at(Some(1e-2));
        assert_eq!(
            coarse.len(),
            1,
            "at tolerance 1e-2 the gap is below tolerance and the squares merge"
        );
        assert_eq!(coarse[0].len(), 12, "the merged loop has 6 vertices");

        // And the default behaves like the fine tolerance, not the coarse
        // one — so a hard-coded coarse constant would fail here too.
        assert_eq!(union_at(None).len(), 2);
    }

    #[test]
    fn a_failed_polygon_op_does_not_stop_the_rest_of_the_batch() {
        let mut k = BrepKernel::new();
        let json = serde_json::json!([
            {"op": "polygonBoolean2d", "args": {
                "coordsA": sq(0.0, 0.0, 1.0), "coordsB": sq(0.0, 0.0, 1.0), "operation": "xor"}},
            {"op": "polygonUnion2d", "args": {
                "coordsA": sq(0.0, 0.0, 10.0), "coordsB": sq(5.0, 5.0, 10.0)}},
        ])
        .to_string();
        let raw = k.execute_batch(&json);
        let parsed: Vec<serde_json::Value> = serde_json::from_str(&raw).unwrap();
        assert_eq!(parsed.len(), 2);
        assert!(parsed[0].get("error").is_some());
        assert!(parsed[1].get("ok").is_some());
    }
}
