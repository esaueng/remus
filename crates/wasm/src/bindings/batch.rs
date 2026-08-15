//! Batch execution and dispatch bindings.

#![allow(clippy::missing_errors_doc, clippy::too_many_lines)]

use std::rc::Rc;

use wasm_bindgen::prelude::*;

use brepkit_math::mat::Mat4;
use brepkit_math::nurbs::curve::NurbsCurve;
use brepkit_math::nurbs::surface::NurbsSurface;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::boolean::{self, BooleanOp, boolean};
use brepkit_operations::extrude::extrude;
use brepkit_operations::measure;
use brepkit_operations::push_pull::{push_pull_face, resize_cylindrical_face};
use brepkit_operations::revolve::revolve;
use brepkit_operations::sweep::sweep;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::edge::EdgeCurve;

use crate::error::{StructuredWasmError, WasmError};
use crate::handles::{
    compound_id_to_u32, edge_id_to_u32, face_id_to_u32, solid_id_to_u32, wire_id_to_u32,
};
use crate::helpers::{
    TOL, classify_to_string, get_f64, get_f64_array, get_u32, get_u32_array, panic_message,
    try_chamfer, try_fillet,
};
use crate::kernel::BrepKernel;

/// Maximum encoded JSON accepted by one `executeBatch` call (16 MiB).
const MAX_BATCH_JSON_BYTES: usize = 16 * 1024 * 1024;
/// Maximum operations executed by one `executeBatch` call.
const MAX_BATCH_OPERATIONS: usize = 10_000;
/// Default tessellation deflection for batch operations when omitted.
const DEFAULT_DEFLECTION: f64 = 0.1;

fn validate_deflection(deflection: f64) -> Result<f64, StructuredWasmError> {
    crate::error::validate_positive(deflection, "deflection").map_err(StructuredWasmError::from)?;
    Ok(deflection)
}

fn get_deflection(args: &serde_json::Value) -> Result<f64, StructuredWasmError> {
    let deflection = match args.get("deflection") {
        None => DEFAULT_DEFLECTION,
        Some(_) => get_f64(args, "deflection")?,
    };
    validate_deflection(deflection)
}

#[wasm_bindgen(typescript_custom_section)]
const BATCH_V2_TYPES: &str = r#"
/** Stable error codes returned by `executeBatchV2`. */
export type BatchErrorCodeV2 =
  | "invalid_json"
  | "batch_limit_exceeded"
  | "missing_operation"
  | "unknown_operation"
  | "invalid_argument"
  | "invalid_handle"
  | "topology_error"
  | "operation_failed"
  | "resource_limit_exceeded"
  | "internal_error";

/** Machine-readable error returned by `executeBatchV2`. */
export interface BatchErrorV2 {
  code: BatchErrorCodeV2;
  message: string;
  details: Record<string, string | number | boolean | null>;
}

/** One parsed item in the JSON string returned by `executeBatchV2`. */
export type BatchResultV2 = { ok: unknown } | { error: BatchErrorV2 };
"#;

#[derive(Clone, Copy)]
enum BatchContract {
    Legacy,
    V2,
}

type BatchItemResult = Result<serde_json::Value, StructuredWasmError>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BatchOpKind {
    ReadOnly,
    Mutating,
}

/// Classify operations before doing any work that scales with topology size.
///
/// Keeping unknown operations out of the mutating fallback is important: an
/// attacker must not be able to trigger a full topology snapshot with an
/// arbitrary operation name.
///
/// `ReadOnly` is a performance hint, never a correctness claim. Those ops take
/// an `Rc` share of the topology instead of a deep copy; if one does mutate,
/// `Rc::make_mut` still materialises the pre-op state, so rollback stays exact.
fn batch_op_kind(op: &str) -> Option<BatchOpKind> {
    match op {
        "boundingBox"
        | "centerOfMass"
        | "chamfer2d"
        | "classifyPoint"
        | "detectCoincidentFaces"
        | "fillet2d"
        | "getNurbsCurveData"
        | "getNurbsSurfaceData"
        | "getNurbsSurfaceDataParity"
        | "massProperties"
        | "meshQuality"
        | "polygonBoolean2d"
        | "polygonUnion2d"
        | "projectEdges"
        | "solidEdges"
        | "solidToSolidDistance"
        | "surfaceArea"
        | "validateSolid"
        | "volume" => Some(BatchOpKind::ReadOnly),
        "makeBox"
        | "makeCylinder"
        | "makeSphere"
        | "makeCone"
        | "makeTorus"
        | "makeEllipsoid"
        | "fuse"
        | "cut"
        | "intersect"
        | "fuseWithOptions"
        | "cutWithOptions"
        | "intersectWithOptions"
        | "fuseWithEvolution"
        | "cutWithEvolution"
        | "intersectWithEvolution"
        | "compoundCut"
        | "fuseAll"
        | "transform"
        | "copySolid"
        | "copyAndTransformSolid"
        | "pushPullFace"
        | "resizeCylindricalFace"
        | "extrude"
        | "revolve"
        | "sweep"
        | "sweepWithOptions"
        | "helicalSweep"
        | "multiSectionSweep"
        | "guidedSweep"
        | "minkowskiSum"
        | "chamfer"
        | "fillet"
        | "filletVariable"
        | "filletV2"
        | "chamferV2"
        | "chamferDistanceAngle"
        | "shell"
        | "mirror"
        | "unifyFaces"
        | "convertToBspline"
        | "convertToElementary"
        | "healSolid"
        | "repairSolid"
        | "loft"
        | "loftWithOptions"
        | "loftSmooth"
        | "circularPattern"
        | "gridPattern"
        | "defeature"
        | "copyWire"
        | "copyFace"
        | "transformWire"
        | "transformFace"
        | "offsetFace"
        | "offsetSolid"
        | "offsetSolidV2"
        | "section"
        | "split"
        | "sewFaces"
        | "thicken"
        | "pipe"
        | "linearPattern"
        | "draft"
        | "makeTangentArc3d"
        | "liftCurve2dToPlane"
        | "offsetWire"
        | "offsetWireWithJoinType"
        | "offsetWire2DWithJoin"
        | "makeLineEdge"
        | "makeNurbsEdge"
        | "makeWire"
        | "makePlanarFaceFromWire"
        | "makeFaceFromWires"
        | "addHolesToFace" => Some(BatchOpKind::Mutating),
        _ => None,
    }
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Batch execution ──────────────────────────────────────────

    /// Execute a batch of operations, crossing the JS/WASM boundary once.
    ///
    /// Accepts a JSON string containing an array of operation objects:
    /// ```json
    /// [
    ///   {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 2.0}},
    ///   {"op": "fuse", "args": {"solidA": 0, "solidB": 1}},
    ///   {"op": "volume", "args": {"solid": 2, "deflection": 0.1}}
    /// ]
    /// ```
    ///
    /// Returns a JSON string with an array of results:
    /// ```json
    /// [
    ///   {"ok": 0},
    ///   {"ok": 2},
    ///   {"error": "invalid solid id"}
    /// ]
    /// ```
    ///
    /// Operations are executed sequentially; an error in one does not
    /// prevent execution of subsequent operations.
    #[wasm_bindgen(js_name = "executeBatch")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn execute_batch(&mut self, json: &str) -> String {
        self.execute_batch_contract(json, BatchContract::Legacy)
    }

    /// Execute a batch and return stable machine-readable error codes.
    ///
    /// The returned JSON string contains the same bare result array and
    /// success envelopes as [`executeBatch`](Self::execute_batch). Error
    /// envelopes are additive structured objects with `code`, the unchanged
    /// human-readable `message`, and an always-present `details` object.
    /// Existing `executeBatch` behavior is unchanged.
    #[wasm_bindgen(js_name = "executeBatchV2")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn execute_batch_v2(&mut self, json: &str) -> String {
        self.execute_batch_contract(json, BatchContract::V2)
    }
}

impl BrepKernel {
    fn execute_batch_contract(&mut self, json: &str, contract: BatchContract) -> String {
        let results = self.execute_batch_results(json);
        let serialized = results
            .into_iter()
            .map(|result| match (contract, result) {
                (_, Ok(value)) => serde_json::json!({"ok": value}),
                (BatchContract::Legacy, Err(error)) => {
                    serde_json::json!({"error": error.message()})
                }
                (BatchContract::V2, Err(error)) => serde_json::json!({"error": error}),
            })
            .collect();
        serde_json::Value::Array(serialized).to_string()
    }

    fn execute_batch_results(&mut self, json: &str) -> Vec<BatchItemResult> {
        if json.len() > MAX_BATCH_JSON_BYTES {
            let message = format!(
                "batch JSON exceeds {MAX_BATCH_JSON_BYTES} byte limit (got {})",
                json.len()
            );
            return vec![Err(StructuredWasmError::batch_limit(
                message,
                "json_bytes",
                MAX_BATCH_JSON_BYTES,
                json.len(),
            ))];
        }
        let ops: Vec<serde_json::Value> = match serde_json::from_str(json) {
            Ok(v) => v,
            Err(e) => {
                return vec![Err(StructuredWasmError::invalid_json(&e))];
            }
        };
        if ops.len() > MAX_BATCH_OPERATIONS {
            let message = format!(
                "batch exceeds {MAX_BATCH_OPERATIONS} operation limit (got {})",
                ops.len()
            );
            return vec![Err(StructuredWasmError::batch_limit(
                message,
                "operations",
                MAX_BATCH_OPERATIONS,
                ops.len(),
            ))];
        }

        ops.iter()
            .enumerate()
            .map(|(operation_index, entry)| {
                let op = match entry["op"].as_str() {
                    Some(s) => s,
                    None => {
                        return Err(StructuredWasmError::missing_operation(operation_index));
                    }
                };
                let args = &entry["args"];
                let kind = match batch_op_kind(op) {
                    Some(kind) => kind,
                    None => {
                        return Err(StructuredWasmError::unknown_operation(op)
                            .with_operation_context(operation_index, op));
                    }
                };
                self.dispatch_with_rollback(kind, op, args)
                    .map_err(|error| error.with_operation_context(operation_index, op))
            })
            .collect()
    }

    /// Runs one batch operation, undoing its topology changes if it fails.
    ///
    /// The two arms differ only in what taking the rollback snapshot costs;
    /// both restore the exact pre-operation state. See [`batch_op_kind`].
    fn dispatch_with_rollback(
        &mut self,
        kind: BatchOpKind,
        op: &str,
        args: &serde_json::Value,
    ) -> BatchItemResult {
        if kind == BatchOpKind::ReadOnly {
            // O(1). Still correct if the op does mutate: `Rc::make_mut` in
            // `topo_mut` sees the extra reference and copies the pre-op arenas
            // aside before the first mutation lands.
            let snapshot = Rc::clone(&self.topo);
            let result = self.dispatch_op(op, args);
            if result.is_err() {
                self.topo_mut().restore_preserving_handle_slots(&snapshot);
            }
            result
        } else {
            // Copy aside so `self.topo` stays unshared and the operation
            // mutates it in place, keeping its arena capacity.
            let snapshot = self.topo().clone();
            let result = self.dispatch_op(op, args);
            if result.is_err() {
                self.topo_mut().restore_preserving_handle_slots(&snapshot);
            }
            result
        }
    }
}

/// A `(u_range, v_range)` pair, each `(min, max)`.
type UvRanges = ((f64, f64), (f64, f64));

/// Build the in-plane axes used by `plane_to_nurbs`.
///
/// Must match `brepkit_heal::construct::convert_surface`'s private frame
/// so projected face corners reconstruct the plane rectangle consistently.
fn plane_frame_axes(normal: Vec3) -> (Vec3, Vec3) {
    let seed = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u_axis = normal
        .cross(seed)
        .normalize()
        .unwrap_or_else(|_| Vec3::new(1.0, 0.0, 0.0));
    let v_axis = normal.cross(u_axis);
    (u_axis, v_axis)
}

impl BrepKernel {
    /// Extract a `NurbsCurve` from an edge.
    ///
    /// NURBS edges are returned directly. Line, Circle, and Ellipse edges
    /// are converted to their exact rational NURBS equivalent using the
    /// edge's bounding vertices (and the curve's analytic params for
    /// circles/ellipses).
    pub(crate) fn extract_nurbs_curve(&self, edge: u32) -> Result<NurbsCurve, WasmError> {
        use brepkit_geometry::convert::{circle_to_nurbs, ellipse_to_nurbs, line_to_nurbs};
        use std::f64::consts::TAU;

        let edge_id = self.resolve_edge(edge)?;
        let edge_data = self.topo.edge(edge_id)?;
        let start_v = edge_data.start();
        let end_v = edge_data.end();
        let start_pt = self.topo.vertex(start_v)?.point();
        let end_pt = self.topo.vertex(end_v)?.point();

        match edge_data.curve() {
            EdgeCurve::NurbsCurve(c) => Ok(c.clone()),
            EdgeCurve::Line => {
                Ok(
                    line_to_nurbs(start_pt, end_pt).map_err(|e| WasmError::InvalidInput {
                        reason: format!("line_to_nurbs failed: {e}"),
                    })?,
                )
            }
            EdgeCurve::Circle(c) => {
                let (t_start, t_end) = if start_v == end_v {
                    (0.0, TAU)
                } else {
                    let ts = c.project(start_pt);
                    let mut te = c.project(end_pt);
                    if te <= ts {
                        te += TAU;
                    }
                    (ts, te)
                };
                Ok(
                    circle_to_nurbs(c, t_start, t_end).map_err(|e| WasmError::InvalidInput {
                        reason: format!("circle_to_nurbs failed: {e}"),
                    })?,
                )
            }
            EdgeCurve::Ellipse(e) => {
                let (t_start, t_end) = if start_v == end_v {
                    (0.0, TAU)
                } else {
                    let ts = e.project(start_pt);
                    let mut te = e.project(end_pt);
                    if te <= ts {
                        te += TAU;
                    }
                    (ts, te)
                };
                Ok(
                    ellipse_to_nurbs(e, t_start, t_end).map_err(|err| WasmError::InvalidInput {
                        reason: format!("ellipse_to_nurbs failed: {err}"),
                    })?,
                )
            }
            // Exact 3-control-point conic Beziers over the arc bounded by the
            // edge's two vertices. `project` inverts the parameterization
            // exactly, so no sampling or fitting is involved.
            EdgeCurve::Hyperbola(h) => {
                let (t0, t1) = (h.project(start_pt), h.project(end_pt));
                let (lo, hi) = (t0.min(t1), t0.max(t1));
                Ok(
                    brepkit_heal::construct::convert_curve::hyperbola_to_nurbs(h, lo, hi).map_err(
                        |err| WasmError::InvalidInput {
                            reason: format!("hyperbola_to_nurbs failed: {err}"),
                        },
                    )?,
                )
            }
            EdgeCurve::Parabola(pb) => {
                let (t0, t1) = (pb.project(start_pt), pb.project(end_pt));
                let (lo, hi) = (t0.min(t1), t0.max(t1));
                Ok(
                    brepkit_heal::construct::convert_curve::parabola_to_nurbs(pb, lo, hi).map_err(
                        |err| WasmError::InvalidInput {
                            reason: format!("parabola_to_nurbs failed: {err}"),
                        },
                    )?,
                )
            }
        }
    }

    /// Extract a `NurbsSurface` from a face.
    ///
    /// NURBS faces are returned directly. Analytic surfaces are converted to
    /// their NURBS equivalent: planes and cylinders are geometrically exact;
    /// cones, spheres, and tori use the exact rational forms from
    /// `brepkit_heal::construct::convert_surface`. Plane and cone parameter
    /// ranges are derived from the face's boundary vertices.
    pub(crate) fn extract_nurbs_surface(&self, face: u32) -> Result<NurbsSurface, WasmError> {
        use brepkit_heal::construct::convert_surface;
        use brepkit_topology::face::FaceSurface;

        let face_id = self.resolve_face(face)?;
        let face_data = self.topo.face(face_id)?;

        let map_err = |context: &str, e: brepkit_heal::HealError| WasmError::InvalidInput {
            reason: format!("{context}: {e}"),
        };

        match face_data.surface() {
            FaceSurface::Nurbs(s) => Ok(s.clone()),
            FaceSurface::Plane { normal, d } => {
                let (u_range, v_range) = self.plane_face_uv_bounds(face_id, *normal, *d)?;
                convert_surface::plane_to_nurbs(*normal, *d, u_range, v_range)
                    .map_err(|e| map_err("plane_to_nurbs failed", e))
            }
            FaceSurface::Cylinder(c) => {
                let v_range = self.analytic_face_v_bounds(face_id, face_data.surface())?;
                convert_surface::cylinder_to_nurbs(c, v_range)
                    .map_err(|e| map_err("cylinder_to_nurbs failed", e))
            }
            FaceSurface::Cone(c) => {
                let v_range = self.analytic_face_v_bounds(face_id, face_data.surface())?;
                convert_surface::cone_to_nurbs(c, v_range)
                    .map_err(|e| map_err("cone_to_nurbs failed", e))
            }
            FaceSurface::Sphere(s) => convert_surface::sphere_to_nurbs(s)
                .map_err(|e| map_err("sphere_to_nurbs failed", e)),
            FaceSurface::Torus(t) => {
                convert_surface::torus_to_nurbs(t).map_err(|e| map_err("torus_to_nurbs failed", e))
            }
        }
    }

    /// Derive the parametric rectangle of a planar face by sampling its outer
    /// boundary edges and projecting the samples onto the same local frame
    /// `plane_to_nurbs` uses.
    ///
    /// Sampling the edge curves (not just the bounding vertices) is required for
    /// circle- and ellipse-bounded faces such as cylinder/cone caps, whose
    /// outer wire may carry a single seam vertex while the disk spans a finite
    /// rectangle in the plane frame.
    #[allow(clippy::cast_precision_loss)]
    fn plane_face_uv_bounds(
        &self,
        face_id: brepkit_topology::face::FaceId,
        normal: Vec3,
        d: f64,
    ) -> Result<UvRanges, WasmError> {
        const EDGE_SAMPLES: usize = 16;

        let face_data = self.topo.face(face_id)?;
        let wire = self.topo.wire(face_data.outer_wire())?;
        let origin = Point3::new(0.0, 0.0, 0.0) + normal * d;
        let (u_axis, v_axis) = plane_frame_axes(normal);

        let mut u_min = f64::INFINITY;
        let mut u_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;
        let mut v_max = f64::NEG_INFINITY;
        for oe in wire.edges() {
            let edge = self.topo.edge(oe.edge())?;
            let start = self.topo.vertex(edge.start())?.point();
            let end = self.topo.vertex(edge.end())?.point();
            let curve = edge.curve();
            let (t0, t1) = curve.domain_with_endpoints(start, end);
            for i in 0..=EDGE_SAMPLES {
                let t = t0 + (t1 - t0) * (i as f64 / EDGE_SAMPLES as f64);
                let p = curve.evaluate_with_endpoints(t, start, end);
                let rel = p - origin;
                let u = rel.dot(u_axis);
                let v = rel.dot(v_axis);
                u_min = u_min.min(u);
                u_max = u_max.max(u);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
        if u_max <= u_min || v_max <= v_min {
            return Err(WasmError::InvalidInput {
                reason: "planar face has degenerate parametric extent".to_string(),
            });
        }
        Ok(((u_min, u_max), (v_min, v_max)))
    }

    /// Derive the axial/generator parameter range of an analytic face by
    /// projecting its boundary vertices onto the surface.
    fn analytic_face_v_bounds(
        &self,
        face_id: brepkit_topology::face::FaceId,
        surface: &brepkit_topology::face::FaceSurface,
    ) -> Result<(f64, f64), WasmError> {
        let verts = brepkit_topology::explorer::face_vertices(&self.topo, face_id)?;
        let mut v_min = f64::INFINITY;
        let mut v_max = f64::NEG_INFINITY;
        for vid in verts {
            let p = self.topo.vertex(vid)?.point();
            if let Some((_, v)) = surface.project_point(p) {
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
        if v_max <= v_min {
            return Err(WasmError::InvalidInput {
                reason: "analytic face has degenerate axial extent".to_string(),
            });
        }
        Ok((v_min, v_max))
    }

    /// Create an edge from a `NurbsCurve`, using its endpoints.
    pub(crate) fn nurbs_curve_to_edge(
        &mut self,
        points: &[Point3],
        curve: NurbsCurve,
    ) -> brepkit_topology::edge::EdgeId {
        let start = points[0];
        let end = points[points.len() - 1];
        brepkit_topology::builder::make_nurbs_edge(self.topo_mut(), start, end, curve, TOL)
    }

    /// Create an edge from a `NurbsCurve`, evaluating its endpoints.
    pub(crate) fn nurbs_curve_to_edge_from_curve(
        &mut self,
        curve: &NurbsCurve,
    ) -> brepkit_topology::edge::EdgeId {
        brepkit_topology::builder::make_nurbs_edge_from_curve(self.topo_mut(), curve, TOL)
    }

    /// Create a face from a `NurbsSurface` with a rectangular domain wire.
    pub(crate) fn nurbs_surface_to_face(
        &mut self,
        surface: NurbsSurface,
    ) -> Result<brepkit_topology::face::FaceId, JsError> {
        Ok(brepkit_topology::builder::make_nurbs_face(
            self.topo_mut(),
            surface,
            TOL,
        )?)
    }

    /// Dispatch a single batch operation by name.
    #[allow(clippy::too_many_lines)]
    fn dispatch_op(
        &mut self,
        op: &str,
        args: &serde_json::Value,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        match op {
            "makeBox" => {
                let w = get_f64(args, "width")?;
                let h = get_f64(args, "height")?;
                let d = get_f64(args, "depth")?;
                let solid = brepkit_operations::primitives::make_box(self.topo_mut(), w, h, d)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeCylinder" => {
                let r = get_f64(args, "radius")?;
                let h = get_f64(args, "height")?;
                let solid = brepkit_operations::primitives::make_cylinder(self.topo_mut(), r, h)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeSphere" => {
                let r = get_f64(args, "radius")?;
                let segments = get_u32(args, "segments").unwrap_or(16);
                let solid = brepkit_operations::primitives::make_sphere(
                    self.topo_mut(),
                    r,
                    segments as usize,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeCone" => {
                let br = get_f64(args, "bottomRadius")?;
                let tr = get_f64(args, "topRadius")?;
                let h = get_f64(args, "height")?;
                let solid = brepkit_operations::primitives::make_cone(self.topo_mut(), br, tr, h)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeTorus" => {
                let major = get_f64(args, "majorRadius")?;
                let minor = get_f64(args, "minorRadius")?;
                let segments = get_u32(args, "segments").unwrap_or(16);
                let solid = brepkit_operations::primitives::make_torus(
                    self.topo_mut(),
                    major,
                    minor,
                    segments as usize,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeEllipsoid" => {
                let rx = get_f64(args, "rx")?;
                let ry = get_f64(args, "ry")?;
                let rz = get_f64(args, "rz")?;
                if rx <= 0.0 || ry <= 0.0 || rz <= 0.0 {
                    return Err(StructuredWasmError::invalid_argument(
                        "rx, ry, rz must be positive",
                        None,
                    ));
                }
                let solid = brepkit_operations::primitives::make_sphere(self.topo_mut(), 1.0, 16)
                    .map_err(StructuredWasmError::from)?;
                let mat = brepkit_math::mat::Mat4::scale(rx, ry, rz);
                transform_solid(self.topo_mut(), solid, &mat).map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "fuse" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let result = boolean(self.topo_mut(), BooleanOp::Fuse, a_id, b_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "cut" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let result = boolean(self.topo_mut(), BooleanOp::Cut, a_id, b_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "intersect" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let result = boolean(self.topo_mut(), BooleanOp::Intersect, a_id, b_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "fuseWithOptions" | "cutWithOptions" | "intersectWithOptions" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let unify_faces = args["unifyFaces"].as_bool();
                let bool_op = match op {
                    "fuseWithOptions" => BooleanOp::Fuse,
                    "cutWithOptions" => BooleanOp::Cut,
                    _ => BooleanOp::Intersect,
                };
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let opts = brepkit_operations::boolean::BooleanOptions {
                    unify_faces: unify_faces.unwrap_or(true),
                    ..Default::default()
                };
                let result = brepkit_operations::boolean::boolean_with_options(
                    self.topo_mut(),
                    bool_op,
                    a_id,
                    b_id,
                    opts,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "fuseWithEvolution" | "cutWithEvolution" | "intersectWithEvolution" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let bool_op = match op {
                    "fuseWithEvolution" => BooleanOp::Fuse,
                    "cutWithEvolution" => BooleanOp::Cut,
                    _ => BooleanOp::Intersect,
                };
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let (result, evolution) = brepkit_operations::boolean::boolean_with_evolution(
                    self.topo_mut(),
                    bool_op,
                    a_id,
                    b_id,
                )
                .map_err(StructuredWasmError::from)?;
                let evolution: serde_json::Value = serde_json::from_str(&evolution.to_json())
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "solid": solid_id_to_u32(result),
                    "evolution": evolution,
                }))
            }
            "detectCoincidentFaces" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let pairs = brepkit_algo::diagnostic::detect_coincident_faces(
                    self.topo(),
                    a_id,
                    b_id,
                    brepkit_math::tolerance::Tolerance::default(),
                )
                .map_err(StructuredWasmError::from)?;
                Ok(crate::bindings::booleans::coincident_face_pairs_to_json(
                    &pairs,
                ))
            }
            "compoundCut" => {
                let target = get_u32(args, "target")?;
                let target_id = self
                    .resolve_solid(target)
                    .map_err(StructuredWasmError::from)?;
                let tool_arr = args["tools"]
                    .as_array()
                    .ok_or("missing or invalid 'tools' array")?;
                let tools: Vec<brepkit_topology::solid::SolidId> = tool_arr
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        let h = v
                            .as_u64()
                            .ok_or_else(|| format!("tools[{i}] is not a number"))
                            .map(|n| n as u32)?;
                        self.resolve_solid(h).map_err(StructuredWasmError::from)
                    })
                    .collect::<Result<Vec<_>, StructuredWasmError>>()?;
                let result = boolean::compound_cut(
                    self.topo_mut(),
                    target_id,
                    &tools,
                    boolean::BooleanOptions::default(),
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "fuseAll" => {
                let solid_arr = args["solids"]
                    .as_array()
                    .ok_or("missing or invalid 'solids' array")?;
                let solids: Vec<brepkit_topology::solid::SolidId> = solid_arr
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        let h = v
                            .as_u64()
                            .ok_or_else(|| format!("solids[{i}] is not a number"))
                            .map(|n| n as u32)?;
                        self.resolve_solid(h).map_err(StructuredWasmError::from)
                    })
                    .collect::<Result<Vec<_>, StructuredWasmError>>()?;
                let compound = self
                    .topo_mut()
                    .add_compound(brepkit_topology::compound::Compound::new(solids));
                let result = brepkit_operations::compound_ops::fuse_all(self.topo_mut(), compound)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "transform" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let matrix = args["matrix"]
                    .as_array()
                    .ok_or("missing or invalid 'matrix'")?;
                if matrix.len() != 16 {
                    return Err(StructuredWasmError::invalid_argument(
                        format!("matrix must have 16 elements, got {}", matrix.len()),
                        Some("matrix"),
                    ));
                }
                let elems: Vec<f64> = matrix
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        v.as_f64()
                            .ok_or_else(|| format!("matrix[{i}] is not a number"))
                    })
                    .collect::<Result<_, _>>()?;
                let rows = std::array::from_fn(|i| std::array::from_fn(|j| elems[i * 4 + j]));
                let mat = Mat4(rows);
                transform_solid(self.topo_mut(), solid_id, &mat)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid_id)))
            }
            "volume" => {
                let s = get_u32(args, "solid")?;
                let deflection = get_deflection(args)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let v = measure::solid_volume(&self.topo, solid_id, deflection)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(v))
            }
            "validateSolid" => {
                let solid = get_u32(args, "solid")?;
                let solid_id = self
                    .resolve_solid(solid)
                    .map_err(StructuredWasmError::from)?;
                let report = brepkit_operations::validate::validate_solid(&self.topo, solid_id)
                    .map_err(StructuredWasmError::from)?;
                let error_count = u32::try_from(report.error_count())
                    .map_err(|_| "validation error count exceeds u32".to_string())?;
                Ok(serde_json::json!(error_count))
            }
            "surfaceArea" => {
                let s = get_u32(args, "solid")?;
                let deflection = get_deflection(args)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let a = measure::solid_surface_area(&self.topo, solid_id, deflection)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(a))
            }
            "boundingBox" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let aabb = measure::solid_bounding_box(&self.topo, solid_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!([
                    aabb.min.x(),
                    aabb.min.y(),
                    aabb.min.z(),
                    aabb.max.x(),
                    aabb.max.y(),
                    aabb.max.z()
                ]))
            }
            "centerOfMass" => {
                let s = get_u32(args, "solid")?;
                let deflection = get_deflection(args)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let com = measure::solid_center_of_mass(&self.topo, solid_id, deflection)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!([com.x(), com.y(), com.z()]))
            }
            "massProperties" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let props = measure::mass_properties(&self.topo, solid_id)
                    .map_err(StructuredWasmError::from)?;
                let (moments, axes) = props.principal_inertia();
                Ok(serde_json::json!({
                    "volume": props.mass,
                    "centerOfMass": [props.center.x(), props.center.y(), props.center.z()],
                    "inertia": props.inertia,
                    "principalMoments": moments,
                    "principalAxes": axes.iter().flatten().copied().collect::<Vec<f64>>(),
                }))
            }
            "meshQuality" => {
                let s = get_u32(args, "solid")?;
                let deflection = get_deflection(args)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let mesh = brepkit_operations::tessellate::tessellate_solid(
                    &self.topo, solid_id, deflection,
                )
                .map_err(StructuredWasmError::from)?;
                let quality = brepkit_operations::tessellate::welded_mesh_quality(&mesh);
                Ok(serde_json::json!({
                    "boundaryEdges": quality.boundary_edges,
                    "nonManifoldEdges": quality.non_manifold_edges,
                    "eulerCharacteristic": quality.euler_characteristic,
                    "isWatertight": quality.is_watertight(),
                }))
            }
            "solidEdges" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edges = brepkit_topology::explorer::solid_edges(&self.topo, solid_id)
                    .map_err(StructuredWasmError::from)?;
                let handles: Vec<u32> = edges.iter().map(|&e| edge_id_to_u32(e)).collect();
                Ok(serde_json::json!(handles))
            }
            "solidToSolidDistance" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let result =
                    brepkit_operations::distance::solid_to_solid_distance(&self.topo, a_id, b_id)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!([
                    result.distance,
                    result.point_a.x(),
                    result.point_a.y(),
                    result.point_a.z(),
                    result.point_b.x(),
                    result.point_b.y(),
                    result.point_b.z(),
                ]))
            }
            "copySolid" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let copy = brepkit_operations::copy::copy_solid(self.topo_mut(), solid_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(copy)))
            }
            "copyAndTransformSolid" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let matrix = args["matrix"]
                    .as_array()
                    .ok_or("missing or invalid 'matrix'")?;
                if matrix.len() != 16 {
                    return Err(StructuredWasmError::invalid_argument(
                        format!("matrix must have 16 elements, got {}", matrix.len()),
                        Some("matrix"),
                    ));
                }
                let elems: Vec<f64> = matrix
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        v.as_f64()
                            .ok_or_else(|| format!("matrix[{i}] is not a number"))
                    })
                    .collect::<Result<_, _>>()?;
                let rows = std::array::from_fn(|i| std::array::from_fn(|j| elems[i * 4 + j]));
                let mat = Mat4(rows);
                let copy = brepkit_operations::copy::copy_and_transform_solid(
                    self.topo_mut(),
                    solid_id,
                    &mat,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(copy)))
            }
            // ── Batch 8: new batch-dispatched operations ──────────────
            "pushPullFace" => {
                let s = get_u32(args, "solid")?;
                let f = get_u32(args, "face")?;
                let distance = get_f64(args, "distance")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let result = push_pull_face(self.topo_mut(), solid_id, face_id, distance)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "resizeCylindricalFace" => {
                let s = get_u32(args, "solid")?;
                let f = get_u32(args, "face")?;
                let radius = get_f64(args, "radius")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let result = resize_cylindrical_face(self.topo_mut(), solid_id, face_id, radius)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "extrude" => {
                let f = get_u32(args, "face")?;
                let dx = get_f64(args, "dx").unwrap_or(0.0);
                let dy = get_f64(args, "dy").unwrap_or(0.0);
                let dz = get_f64(args, "dz").unwrap_or(1.0);
                let dist = get_f64(args, "distance").unwrap_or(1.0);
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let dir = Vec3::new(dx, dy, dz);
                let solid = extrude(self.topo_mut(), face_id, dir, dist)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "revolve" => {
                let f = get_u32(args, "face")?;
                let angle_degrees = get_f64(args, "angle")?;
                let ox = get_f64(args, "originX").unwrap_or(0.0);
                let oy = get_f64(args, "originY").unwrap_or(0.0);
                let oz = get_f64(args, "originZ").unwrap_or(0.0);
                let ax = get_f64(args, "axisX").unwrap_or(0.0);
                let ay = get_f64(args, "axisY").unwrap_or(0.0);
                let az = get_f64(args, "axisZ").unwrap_or(1.0);
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                // Convert degrees to radians to match the direct WASM binding.
                let solid = revolve(
                    self.topo_mut(),
                    face_id,
                    Point3::new(ox, oy, oz),
                    Vec3::new(ax, ay, az),
                    angle_degrees.to_radians(),
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "sweep" => {
                let f = get_u32(args, "face")?;
                let e = get_u32(args, "pathEdge")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let curve = self
                    .extract_nurbs_curve(e)
                    .map_err(StructuredWasmError::from)?;
                let solid =
                    sweep(self.topo_mut(), face_id, &curve).map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "sweepWithOptions" => {
                let profile = get_u32(args, "profile")?;
                let path_edge = get_u32(args, "pathEdge")?;
                let contact_mode = match args.get("contactMode") {
                    None | Some(serde_json::Value::Null) => "rmf",
                    Some(value) => value.as_str().ok_or("missing or invalid 'contactMode'")?,
                };
                let scale_values = match args.get("scaleValues") {
                    None | Some(serde_json::Value::Null) => Vec::new(),
                    Some(_) => get_f64_array(args, "scaleValues")?,
                };
                let segments = match args.get("segments") {
                    None | Some(serde_json::Value::Null) => 0,
                    Some(_) => get_u32(args, "segments")?,
                };
                let corner_mode = match args.get("cornerMode") {
                    None | Some(serde_json::Value::Null) => "smooth",
                    Some(value) => value.as_str().ok_or("missing or invalid 'cornerMode'")?,
                };
                let face_id = self
                    .resolve_face(profile)
                    .map_err(StructuredWasmError::from)?;
                let path_curve = self
                    .extract_nurbs_curve(path_edge)
                    .map_err(StructuredWasmError::from)?;
                let options = super::operations::parse_sweep_options(
                    contact_mode,
                    scale_values,
                    segments,
                    corner_mode,
                )?;
                let result = brepkit_operations::sweep::sweep_with_options(
                    self.topo_mut(),
                    face_id,
                    &path_curve,
                    &options,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "helicalSweep" => {
                let profile = get_u32(args, "profile")?;
                let origin = Point3::new(
                    get_f64(args, "axisOriginX")?,
                    get_f64(args, "axisOriginY")?,
                    get_f64(args, "axisOriginZ")?,
                );
                let axis_dir = Vec3::new(
                    get_f64(args, "axisDirX")?,
                    get_f64(args, "axisDirY")?,
                    get_f64(args, "axisDirZ")?,
                );
                let radius = get_f64(args, "radius")?;
                let pitch = get_f64(args, "pitch")?;
                let turns = get_f64(args, "turns")?;
                crate::error::validate_positive(radius, "radius")
                    .map_err(StructuredWasmError::from)?;
                crate::error::validate_positive(pitch, "pitch")
                    .map_err(StructuredWasmError::from)?;
                let face_id = self
                    .resolve_face(profile)
                    .map_err(StructuredWasmError::from)?;
                let result = brepkit_operations::helix::helical_sweep(
                    self.topo_mut(),
                    face_id,
                    origin,
                    axis_dir,
                    radius,
                    pitch,
                    turns,
                    8,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "multiSectionSweep" => {
                let faces: Vec<u32> = args["faces"]
                    .as_array()
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let params: Vec<f64> = args["params"]
                    .as_array()
                    .map(|a| a.iter().filter_map(serde_json::Value::as_f64).collect())
                    .unwrap_or_default();
                if faces.len() != params.len() {
                    return Err("multiSectionSweep: faces and params length mismatch".into());
                }
                let spine_edge = get_u32(args, "spineEdge")?;
                let spine = self
                    .extract_nurbs_curve(spine_edge)
                    .map_err(StructuredWasmError::from)?;
                let ruled = args["ruled"].as_bool().unwrap_or(true);
                let sections: Vec<(brepkit_topology::face::FaceId, f64)> = faces
                    .iter()
                    .zip(params.iter())
                    .map(|(&h, &p)| {
                        self.resolve_face(h)
                            .map(|f| (f, p))
                            .map_err(StructuredWasmError::from)
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                let solid = brepkit_operations::sweep::multi_section_sweep(
                    self.topo_mut(),
                    &spine,
                    &sections,
                    ruled,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "guidedSweep" => {
                let face_id = self
                    .resolve_face(get_u32(args, "face")?)
                    .map_err(StructuredWasmError::from)?;
                let spine = self
                    .extract_nurbs_curve(get_u32(args, "spineEdge")?)
                    .map_err(StructuredWasmError::from)?;
                let aux = self
                    .extract_nurbs_curve(get_u32(args, "auxEdge")?)
                    .map_err(StructuredWasmError::from)?;
                let solid =
                    brepkit_operations::sweep::sweep_guided(self.topo_mut(), face_id, &spine, aux)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "minkowskiSum" => {
                let a = self
                    .resolve_solid(get_u32(args, "solidA")?)
                    .map_err(StructuredWasmError::from)?;
                let b = self
                    .resolve_solid(get_u32(args, "solidB")?)
                    .map_err(StructuredWasmError::from)?;
                let solid =
                    brepkit_operations::primitives::make_minkowski_sum(self.topo_mut(), a, b)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "projectEdges" => {
                let solid = self
                    .resolve_solid(get_u32(args, "solid")?)
                    .map_err(StructuredWasmError::from)?;
                let origin = Point3::new(
                    get_f64(args, "originX")?,
                    get_f64(args, "originY")?,
                    get_f64(args, "originZ")?,
                );
                let dir = Vec3::new(
                    get_f64(args, "dirX")?,
                    get_f64(args, "dirY")?,
                    get_f64(args, "dirZ")?,
                );
                let x_axis = Vec3::new(
                    get_f64(args, "xAxisX")?,
                    get_f64(args, "xAxisY")?,
                    get_f64(args, "xAxisZ")?,
                );
                let hidden_lines = args["hiddenLines"].as_bool().unwrap_or(true);
                let deflection = get_deflection(args)?;
                let result = brepkit_operations::projection::project_edges(
                    &self.topo,
                    solid,
                    origin,
                    dir,
                    x_axis,
                    hidden_lines,
                    deflection,
                )
                .map_err(StructuredWasmError::from)?;
                let flatten = |polys: &[Vec<brepkit_math::vec::Point2>]| -> Vec<Vec<f64>> {
                    polys
                        .iter()
                        .map(|poly| poly.iter().flat_map(|p| [p.x(), p.y()]).collect())
                        .collect()
                };
                Ok(serde_json::json!({
                    "visible": flatten(&result.visible),
                    "hidden": flatten(&result.hidden),
                }))
            }
            "chamfer" => {
                let s = get_u32(args, "solid")?;
                let dist = get_f64(args, "distance")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edge_handles: Vec<u32> = args["edges"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                // Same engine chain and panic guard as the `chamfer` binding,
                // and as the sibling `fillet` arm below. Calling the v1
                // flat-bevel engine directly here meant a batch chamfer never
                // reached the v2 fallback, so a closed cylinder rim failed
                // through `executeBatch` while succeeding through `chamfer`.
                let chamfer_result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    try_chamfer(self.topo_mut(), solid_id, &edge_ids, dist)
                }));
                let result = match chamfer_result {
                    Ok(inner) => inner.map_err(StructuredWasmError::from)?,
                    Err(panic_info) => {
                        return Err(StructuredWasmError::operation_failed(panic_message(
                            &panic_info,
                            "Chamfer",
                        )));
                    }
                };
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "fillet" => {
                let s = get_u32(args, "solid")?;
                let radius = get_f64(args, "radius")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edge_handles: Vec<u32> = args["edges"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let fillet_result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    try_fillet(self.topo_mut(), solid_id, &edge_ids, radius)
                }));
                let result = match fillet_result {
                    Ok(inner) => inner.map_err(StructuredWasmError::from)?,
                    Err(panic_info) => {
                        return Err(StructuredWasmError::operation_failed(panic_message(
                            &panic_info,
                            "Fillet",
                        )));
                    }
                };
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "filletVariable" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let specs = args["specs"]
                    .as_array()
                    .ok_or_else(|| "missing 'specs' array".to_string())?;
                let mut edge_laws = Vec::with_capacity(specs.len());
                for spec in specs {
                    let edge_handle = spec["edge"]
                        .as_u64()
                        .ok_or_else(|| "missing 'edge' in fillet spec".to_string())?
                        as u32;
                    let edge_id = self
                        .resolve_edge(edge_handle)
                        .map_err(StructuredWasmError::from)?;
                    let start_val = spec["start"]
                        .as_f64()
                        .or_else(|| spec["startRadius"].as_f64());
                    let end_val = spec["end"].as_f64().or_else(|| spec["endRadius"].as_f64());
                    let law_str =
                        spec["law"]
                            .as_str()
                            .unwrap_or_else(|| match (start_val, end_val) {
                                (Some(sv), Some(ev)) if (sv - ev).abs() > f64::EPSILON => "linear",
                                _ => "constant",
                            });
                    let law = match law_str {
                        "linear" => brepkit_operations::fillet::FilletRadiusLaw::Linear {
                            start: start_val.unwrap_or(1.0),
                            end: end_val.unwrap_or(1.0),
                        },
                        "scurve" => brepkit_operations::fillet::FilletRadiusLaw::SCurve {
                            start: start_val.unwrap_or(1.0),
                            end: end_val.unwrap_or(1.0),
                        },
                        _ => {
                            let r = spec["radius"].as_f64().or(start_val).unwrap_or(1.0);
                            brepkit_operations::fillet::FilletRadiusLaw::Constant(r)
                        }
                    };
                    edge_laws.push((edge_id, law));
                }
                let result = brepkit_operations::fillet::fillet_variable(
                    self.topo_mut(),
                    solid_id,
                    &edge_laws,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "filletV2" => {
                let s = get_u32(args, "solid")?;
                let radius = get_f64(args, "radius")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edge_handles: Vec<u32> = args["edges"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = brepkit_operations::blend_ops::fillet_v2(
                    self.topo_mut(),
                    solid_id,
                    &edge_ids,
                    radius,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result.solid)))
            }
            "chamferV2" => {
                let s = get_u32(args, "solid")?;
                let d1 = get_f64(args, "d1")?;
                let d2 = get_f64(args, "d2")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edge_handles: Vec<u32> = args["edges"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = brepkit_operations::blend_ops::chamfer_v2(
                    self.topo_mut(),
                    solid_id,
                    &edge_ids,
                    d1,
                    d2,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result.solid)))
            }
            "chamferDistanceAngle" => {
                let s = get_u32(args, "solid")?;
                let distance = get_f64(args, "distance")?;
                let angle = get_f64(args, "angle")?;
                if angle >= std::f64::consts::FRAC_PI_2 {
                    return Err("angle must be less than π/2".into());
                }
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edge_handles: Vec<u32> = args["edges"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = brepkit_operations::blend_ops::chamfer_distance_angle(
                    self.topo_mut(),
                    solid_id,
                    &edge_ids,
                    distance,
                    angle,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result.solid)))
            }
            "shell" => {
                let s = get_u32(args, "solid")?;
                let thickness = get_f64(args, "thickness")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_handles: Vec<u32> = args["faces"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = brepkit_operations::shell_op::shell(
                    self.topo_mut(),
                    solid_id,
                    thickness,
                    &face_ids,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "mirror" => {
                let s = get_u32(args, "solid")?;
                let px = get_f64(args, "px").unwrap_or(0.0);
                let py = get_f64(args, "py").unwrap_or(0.0);
                let pz = get_f64(args, "pz").unwrap_or(0.0);
                let nx = get_f64(args, "nx").unwrap_or(1.0);
                let ny = get_f64(args, "ny").unwrap_or(0.0);
                let nz = get_f64(args, "nz").unwrap_or(0.0);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let result = brepkit_operations::mirror::mirror(
                    self.topo_mut(),
                    solid_id,
                    Point3::new(px, py, pz),
                    Vec3::new(nx, ny, nz),
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "unifyFaces" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                brepkit_operations::heal::unify_faces(self.topo_mut(), solid_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid_id)))
            }
            "convertToBspline" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let count = brepkit_operations::heal::convert_to_bspline(self.topo_mut(), solid_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "solid": solid_id_to_u32(solid_id),
                    "converted": count,
                }))
            }
            "convertToElementary" => {
                let s = get_u32(args, "solid")?;
                let tol = get_f64(args, "tolerance").unwrap_or(crate::helpers::TOL);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let count =
                    brepkit_operations::heal::convert_to_elementary(self.topo_mut(), solid_id, tol)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "solid": solid_id_to_u32(solid_id),
                    "converted": count,
                }))
            }
            "healSolid" => {
                let s = get_u32(args, "solid")?;
                let tol = get_f64(args, "tolerance").unwrap_or(1e-7);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                brepkit_operations::heal::heal_solid(self.topo_mut(), solid_id, tol)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid_id)))
            }
            "repairSolid" => {
                let s = get_u32(args, "solid")?;
                let tol = get_f64(args, "tolerance").unwrap_or(1e-7);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let report = brepkit_operations::heal::repair_solid(self.topo_mut(), solid_id, tol)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "solid": solid_id_to_u32(solid_id),
                    "errorsBefore": report.before.error_count(),
                    "errorsAfter": report.after.error_count(),
                    "totalRepairs": report.total_repairs(),
                }))
            }
            "classifyPoint" => {
                let s = get_u32(args, "solid")?;
                let x = get_f64(args, "x")?;
                let y = get_f64(args, "y")?;
                let z = get_f64(args, "z")?;
                let tol = get_f64(args, "tolerance").unwrap_or(1e-7);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let pt = Point3::new(x, y, z);
                let result = brepkit_operations::classify::classify_point(
                    &self.topo, solid_id, pt, 0.1, tol,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(classify_to_string(result)))
            }
            "loft" => {
                let face_handles: Vec<u32> = args["faces"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = brepkit_operations::loft::loft(self.topo_mut(), &face_ids)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "loftWithOptions" => {
                let face_handles = get_u32_array(args, "faces")?;
                let options = match args.get("options") {
                    None | Some(serde_json::Value::Null) => serde_json::Value::Null,
                    Some(value) if value.is_object() => value.clone(),
                    Some(_) => return Err("missing or invalid 'options' object".into()),
                };
                let face_ids = face_handles
                    .into_iter()
                    .map(|handle| self.resolve_face(handle).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result =
                    super::operations::loft_with_options_impl(self.topo_mut(), face_ids, &options)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "loftSmooth" => {
                let face_handles: Vec<u32> = args["faces"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = brepkit_operations::loft::loft_smooth(self.topo_mut(), &face_ids)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "circularPattern" => {
                let s = get_u32(args, "solid")?;
                let ax = get_f64(args, "ax").unwrap_or(0.0);
                let ay = get_f64(args, "ay").unwrap_or(0.0);
                let az = get_f64(args, "az").unwrap_or(1.0);
                let count = get_u32(args, "count")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let axis = Vec3::new(ax, ay, az);
                let compound = brepkit_operations::pattern::circular_pattern(
                    self.topo_mut(),
                    solid_id,
                    axis,
                    count as usize,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(compound_id_to_u32(compound)))
            }
            "gridPattern" => {
                let s = get_u32(args, "solid")?;
                let dxx = get_f64(args, "dirXx").unwrap_or(1.0);
                let dxy = get_f64(args, "dirXy").unwrap_or(0.0);
                let dxz = get_f64(args, "dirXz").unwrap_or(0.0);
                let dyx = get_f64(args, "dirYx").unwrap_or(0.0);
                let dyy = get_f64(args, "dirYy").unwrap_or(1.0);
                let dyz = get_f64(args, "dirYz").unwrap_or(0.0);
                let sx = get_f64(args, "spacingX")?;
                let sy = get_f64(args, "spacingY")?;
                let cx = get_u32(args, "countX")?;
                let cy = get_u32(args, "countY")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let compound = brepkit_operations::pattern::grid_pattern(
                    self.topo_mut(),
                    solid_id,
                    Vec3::new(dxx, dxy, dxz),
                    Vec3::new(dyx, dyy, dyz),
                    sx,
                    sy,
                    cx as usize,
                    cy as usize,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(compound_id_to_u32(compound)))
            }
            "defeature" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_handles: Vec<u32> = args["faces"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result =
                    brepkit_operations::defeature::defeature(self.topo_mut(), solid_id, &face_ids)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "copyWire" => {
                let w = get_u32(args, "wire")?;
                let wire_id = self.resolve_wire(w).map_err(StructuredWasmError::from)?;
                let copy = brepkit_operations::copy::copy_wire(self.topo_mut(), wire_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(wire_id_to_u32(copy)))
            }
            "copyFace" => {
                let f = get_u32(args, "face")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let copy = brepkit_operations::copy::copy_face(self.topo_mut(), face_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(face_id_to_u32(copy)))
            }
            "transformWire" => {
                let w = get_u32(args, "wire")?;
                let wire_id = self.resolve_wire(w).map_err(StructuredWasmError::from)?;
                let matrix = args["matrix"]
                    .as_array()
                    .ok_or("missing or invalid 'matrix'")?;
                if matrix.len() != 16 {
                    return Err(StructuredWasmError::invalid_argument(
                        format!("matrix must have 16 elements, got {}", matrix.len()),
                        Some("matrix"),
                    ));
                }
                let elems: Vec<f64> = matrix
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        v.as_f64()
                            .ok_or_else(|| format!("matrix[{i}] is not a number"))
                    })
                    .collect::<Result<_, _>>()?;
                if let Some(pos) = elems.iter().position(|v| !v.is_finite()) {
                    return Err(StructuredWasmError::invalid_argument(
                        format!("matrix element at index {pos} is not finite"),
                        Some("matrix"),
                    ));
                }
                let rows = std::array::from_fn(|i| std::array::from_fn(|j| elems[i * 4 + j]));
                let mat = Mat4(rows);
                brepkit_operations::transform::transform_wire(self.topo_mut(), wire_id, &mat)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(null))
            }
            "transformFace" => {
                let f = get_u32(args, "face")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let matrix = args["matrix"]
                    .as_array()
                    .ok_or("missing or invalid 'matrix'")?;
                if matrix.len() != 16 {
                    return Err(StructuredWasmError::invalid_argument(
                        format!("matrix must have 16 elements, got {}", matrix.len()),
                        Some("matrix"),
                    ));
                }
                let elems: Vec<f64> = matrix
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        v.as_f64()
                            .ok_or_else(|| format!("matrix element {i} is not a number"))
                    })
                    .collect::<Result<_, _>>()?;
                if let Some(pos) = elems.iter().position(|v| !v.is_finite()) {
                    return Err(StructuredWasmError::invalid_argument(
                        format!("matrix element at index {pos} is not finite"),
                        Some("matrix"),
                    ));
                }
                let rows = std::array::from_fn(|i| std::array::from_fn(|j| elems[i * 4 + j]));
                let mat = Mat4(rows);
                brepkit_operations::transform::transform_face(self.topo_mut(), face_id, &mat)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(null))
            }
            "offsetFace" => {
                let f = get_u32(args, "face")?;
                let dist = get_f64(args, "distance")?;
                let samples = get_u32(args, "samples").unwrap_or(16);
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let result = brepkit_operations::offset_face::offset_face(
                    self.topo_mut(),
                    face_id,
                    dist,
                    samples as usize,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(face_id_to_u32(result)))
            }
            "offsetSolid" => {
                let s = get_u32(args, "solid")?;
                let dist = get_f64(args, "distance")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let result =
                    brepkit_operations::offset_v2::offset_solid_v2(self.topo_mut(), solid_id, dist)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "offsetSolidV2" => {
                let s = get_u32(args, "solid")?;
                let dist = get_f64(args, "distance")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let result =
                    brepkit_operations::offset_v2::offset_solid_v2(self.topo_mut(), solid_id, dist)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "section" => {
                let s = get_u32(args, "solid")?;
                let px = get_f64(args, "px").unwrap_or(0.0);
                let py = get_f64(args, "py").unwrap_or(0.0);
                let pz = get_f64(args, "pz").unwrap_or(0.0);
                let nx = get_f64(args, "nx").unwrap_or(0.0);
                let ny = get_f64(args, "ny").unwrap_or(0.0);
                let nz = get_f64(args, "nz").unwrap_or(1.0);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let result = brepkit_operations::section::section(
                    self.topo_mut(),
                    solid_id,
                    Point3::new(px, py, pz),
                    Vec3::new(nx, ny, nz),
                )
                .map_err(StructuredWasmError::from)?;
                let face_ids: Vec<u32> = result.faces.iter().map(|&f| face_id_to_u32(f)).collect();
                Ok(serde_json::json!(face_ids))
            }
            "split" => {
                let s = get_u32(args, "solid")?;
                let px = get_f64(args, "px").unwrap_or(0.0);
                let py = get_f64(args, "py").unwrap_or(0.0);
                let pz = get_f64(args, "pz").unwrap_or(0.0);
                let nx = get_f64(args, "nx").unwrap_or(0.0);
                let ny = get_f64(args, "ny").unwrap_or(0.0);
                let nz = get_f64(args, "nz").unwrap_or(1.0);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let result = brepkit_operations::split::split(
                    self.topo_mut(),
                    solid_id,
                    Point3::new(px, py, pz),
                    Vec3::new(nx, ny, nz),
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "positive": solid_id_to_u32(result.positive),
                    "negative": solid_id_to_u32(result.negative),
                }))
            }
            "sewFaces" => {
                let face_handles: Vec<u32> = args["faces"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let tol = get_f64(args, "tolerance").unwrap_or(1e-6);
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let solid = brepkit_operations::sew::sew_faces(self.topo_mut(), &face_ids, tol)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "thicken" => {
                let f = get_u32(args, "face")?;
                let thickness = get_f64(args, "thickness")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let result =
                    brepkit_operations::thicken::thicken(self.topo_mut(), face_id, thickness)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "pipe" => {
                let f = get_u32(args, "face")?;
                let e = get_u32(args, "pathEdge")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let curve = self
                    .extract_nurbs_curve(e)
                    .map_err(StructuredWasmError::from)?;
                let solid = brepkit_operations::pipe::pipe(self.topo_mut(), face_id, &curve, None)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "linearPattern" => {
                let s = get_u32(args, "solid")?;
                let dx = get_f64(args, "dx").unwrap_or(1.0);
                let dy = get_f64(args, "dy").unwrap_or(0.0);
                let dz = get_f64(args, "dz").unwrap_or(0.0);
                let spacing = get_f64(args, "spacing")?;
                let count = get_u32(args, "count")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let compound = brepkit_operations::pattern::linear_pattern(
                    self.topo_mut(),
                    solid_id,
                    Vec3::new(dx, dy, dz),
                    spacing,
                    count as usize,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(compound_id_to_u32(compound)))
            }
            "draft" => {
                let s = get_u32(args, "solid")?;
                let angle = get_f64(args, "angle")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_handles: Vec<u32> = args["faces"]
                    .as_array()
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_u64().map(|n| n as u32))
                            .collect()
                    })
                    .unwrap_or_default();
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let dx = get_f64(args, "dirX").unwrap_or(0.0);
                let dy = get_f64(args, "dirY").unwrap_or(0.0);
                let dz = get_f64(args, "dirZ").unwrap_or(1.0);
                let npx = get_f64(args, "neutralX").unwrap_or(0.0);
                let npy = get_f64(args, "neutralY").unwrap_or(0.0);
                let npz = get_f64(args, "neutralZ").unwrap_or(0.0);
                let dir = Vec3::new(dx, dy, dz);
                let neutral = Point3::new(npx, npy, npz);
                let result = brepkit_operations::draft::draft(
                    self.topo_mut(),
                    solid_id,
                    &face_ids,
                    dir,
                    neutral,
                    angle,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "makeTangentArc3d" => {
                let sx = get_f64(args, "startX")?;
                let sy = get_f64(args, "startY")?;
                let sz = get_f64(args, "startZ")?;
                let tx = get_f64(args, "tangentX")?;
                let ty = get_f64(args, "tangentY")?;
                let tz = get_f64(args, "tangentZ")?;
                let ex = get_f64(args, "endX")?;
                let ey = get_f64(args, "endY")?;
                let ez = get_f64(args, "endZ")?;
                let eid = self
                    .make_tangent_arc_3d_impl(sx, sy, sz, tx, ty, tz, ex, ey, ez)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(eid))
            }
            "liftCurve2dToPlane" => {
                let ct = get_u32(args, "curveType")?;
                let params_arr = args["curveParams"]
                    .as_array()
                    .ok_or("missing or invalid 'curveParams'")?;
                let cp: Vec<f64> = params_arr
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        v.as_f64()
                            .ok_or_else(|| format!("curveParams[{i}] is not a number"))
                    })
                    .collect::<Result<_, _>>()?;
                let ox = get_f64(args, "originX")?;
                let oy = get_f64(args, "originY")?;
                let oz = get_f64(args, "originZ")?;
                let xx = get_f64(args, "xAxisX")?;
                let xy = get_f64(args, "xAxisY")?;
                let xz = get_f64(args, "xAxisZ")?;
                let nx = get_f64(args, "normalX")?;
                let ny = get_f64(args, "normalY")?;
                let nz = get_f64(args, "normalZ")?;
                let t0 = get_f64(args, "tStart")?;
                let t1 = get_f64(args, "tEnd")?;
                let eid = self
                    .lift_curve2d_to_plane_impl(ct, cp, ox, oy, oz, xx, xy, xz, nx, ny, nz, t0, t1)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(eid))
            }
            "offsetWire" => {
                let f = get_u32(args, "face")?;
                let dist = get_f64(args, "distance")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let wire_id =
                    brepkit_operations::offset_wire::offset_wire(self.topo_mut(), face_id, dist)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(wire_id_to_u32(wire_id)))
            }
            "offsetWireWithJoinType" => {
                let f = get_u32(args, "face")?;
                let dist = get_f64(args, "distance")?;
                let jt_str = args["joinType"]
                    .as_str()
                    .ok_or("missing or invalid 'joinType' string")?;
                let jt = super::operations::parse_join_type_str(jt_str)
                    .map_err(StructuredWasmError::from)?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let wire_id = brepkit_operations::offset_wire::offset_wire_with_join(
                    self.topo_mut(),
                    face_id,
                    dist,
                    jt,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(wire_id_to_u32(wire_id)))
            }
            "offsetWire2DWithJoin" => {
                let w = get_u32(args, "wire")?;
                let dist = get_f64(args, "distance")?;
                let jt_str = args["joinType"]
                    .as_str()
                    .ok_or("missing or invalid 'joinType' string")?;
                let jt = super::operations::parse_join_type_str(jt_str)
                    .map_err(StructuredWasmError::from)?;
                let wire_id = self.resolve_wire(w).map_err(StructuredWasmError::from)?;
                let face_id =
                    brepkit_topology::builder::make_planar_face_from_wire(self.topo_mut(), wire_id)
                        .map_err(StructuredWasmError::from)?;
                let result = brepkit_operations::offset_wire::offset_wire_with_join(
                    self.topo_mut(),
                    face_id,
                    dist,
                    jt,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(wire_id_to_u32(result)))
            }
            // ── Shape construction ──────────────────────────────
            // Building one glyph outline costs dozens of these calls; a
            // 20-character word costs roughly a thousand. Batching them
            // collapses that into a single boundary crossing.
            "makeLineEdge" => {
                let eid = brepkit_topology::builder::make_line_edge(
                    self.topo_mut(),
                    Point3::new(
                        get_f64(args, "x1")?,
                        get_f64(args, "y1")?,
                        get_f64(args, "z1")?,
                    ),
                    Point3::new(
                        get_f64(args, "x2")?,
                        get_f64(args, "y2")?,
                        get_f64(args, "z2")?,
                    ),
                    TOL,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(edge_id_to_u32(eid)))
            }
            "makeNurbsEdge" => {
                let eid = self
                    .make_nurbs_edge_impl(
                        get_f64(args, "startX")?,
                        get_f64(args, "startY")?,
                        get_f64(args, "startZ")?,
                        get_f64(args, "endX")?,
                        get_f64(args, "endY")?,
                        get_f64(args, "endZ")?,
                        get_u32(args, "degree")?,
                        get_f64_array(args, "knots")?,
                        get_f64_array(args, "controlPoints")?,
                        get_f64_array(args, "weights")?,
                    )
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(eid))
            }
            "makeWire" => {
                let edges = get_u32_array(args, "edges")?;
                // Absent `closed` means "closed"; a present but non-boolean
                // one is a caller error. `as_bool().unwrap_or(true)` would
                // turn `"closed": 0` / `"false"` into a CLOSED wire — the
                // opposite of the intent — and `Wire::new` does not validate
                // closure, so nothing downstream would catch it.
                let closed = match args.get("closed") {
                    None | Some(serde_json::Value::Null) => true,
                    Some(v) => v.as_bool().ok_or("invalid 'closed' boolean")?,
                };
                let wid = self
                    .make_wire_impl(&edges, closed)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(wid))
            }
            "makePlanarFaceFromWire" => {
                let w = get_u32(args, "wire")?;
                let wid = self.resolve_wire(w).map_err(StructuredWasmError::from)?;
                let fid =
                    brepkit_topology::builder::make_planar_face_from_wire(self.topo_mut(), wid)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(face_id_to_u32(fid)))
            }
            "makeFaceFromWires" => {
                let outer = get_u32(args, "outerWire")?;
                let inner = match args.get("innerWires") {
                    None | Some(serde_json::Value::Null) => Vec::new(),
                    Some(_) => get_u32_array(args, "innerWires")?,
                };
                let fid = self
                    .make_face_from_wires_impl(outer, &inner)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(fid))
            }
            "addHolesToFace" => {
                let face = get_u32(args, "face")?;
                let holes = get_u32_array(args, "holeWires")?;
                let fid = self
                    .add_holes_to_face_impl(face, &holes)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(fid))
            }
            "polygonUnion2d" | "polygonBoolean2d" => {
                let coords_a = get_f64_array(args, "coordsA")?;
                let coords_b = get_f64_array(args, "coordsB")?;
                let op = if op == "polygonUnion2d" {
                    brepkit_math::polygon_boolean::BooleanOp::Union
                } else {
                    let name = args["operation"]
                        .as_str()
                        .ok_or("missing or invalid 'operation' string")?;
                    super::polygon2d::parse_polygon_boolean_op(name)
                        .map_err(StructuredWasmError::from)?
                };
                // Absent `tolerance` means "kernel default"; a present but
                // non-numeric one is a caller error, not a default request.
                let tolerance = match args.get("tolerance") {
                    None | Some(serde_json::Value::Null) => None,
                    Some(v) => Some(v.as_f64().ok_or("invalid 'tolerance'")?),
                };
                let result =
                    super::polygon2d::polygon_boolean_2d_impl(&coords_a, &coords_b, op, tolerance)
                        .map_err(StructuredWasmError::from)?;
                serde_json::to_value(result).map_err(StructuredWasmError::from)
            }
            "fillet2d" => {
                let coords = get_f64_array(args, "coords")?;
                let radius = get_f64(args, "radius")?;
                crate::error::validate_positive(radius, "radius")
                    .map_err(StructuredWasmError::from)?;
                let polygon = crate::helpers::parse_polygon_2d_checked(&coords, "coords")
                    .map_err(StructuredWasmError::from)?;
                let result = brepkit_math::polygon2d::fillet_polygon_2d(&polygon, radius);
                let coords: Vec<f64> = result
                    .iter()
                    .flat_map(|point| [point.x(), point.y()])
                    .collect();
                Ok(serde_json::json!(coords))
            }
            "chamfer2d" => {
                let coords = get_f64_array(args, "coords")?;
                let distance = get_f64(args, "distance")?;
                crate::error::validate_positive(distance, "distance")
                    .map_err(StructuredWasmError::from)?;
                let polygon = crate::helpers::parse_polygon_2d_checked(&coords, "coords")
                    .map_err(StructuredWasmError::from)?;
                let result = brepkit_math::polygon2d::chamfer_polygon_2d(&polygon, distance);
                let coords: Vec<f64> = result
                    .iter()
                    .flat_map(|point| [point.x(), point.y()])
                    .collect();
                Ok(serde_json::json!(coords))
            }
            "getNurbsCurveData" => {
                let edge = get_u32(args, "edge")?;
                let curve = self
                    .extract_nurbs_curve(edge)
                    .map_err(StructuredWasmError::from)?;
                Ok(super::nurbs::curve_data_json(&curve))
            }
            "getNurbsSurfaceData" => {
                let face = get_u32(args, "face")?;
                let surface = self
                    .extract_nurbs_surface(face)
                    .map_err(StructuredWasmError::from)?;
                Ok(super::nurbs::surface_data_json(&surface))
            }
            "getNurbsSurfaceDataParity" => {
                let face = get_u32(args, "face")?;
                self.free_form_surface_data_parity(face)
                    .map_err(StructuredWasmError::from)
            }
            _ => Err(StructuredWasmError::unknown_operation(op)),
        }
    }
}

#[cfg(test)]
mod batch_contract_tests {
    #![allow(clippy::expect_used)]

    use super::*;

    fn parse(response: &str) -> serde_json::Value {
        serde_json::from_str(response).expect("batch response must be valid JSON")
    }

    #[test]
    fn batch_deflection_validation_matches_direct_bindings() {
        for deflection in [f64::NAN, f64::INFINITY, 0.0, -0.1] {
            let direct_error = crate::error::validate_positive(deflection, "deflection")
                .expect_err("direct binding validator must reject invalid deflection");
            let batch_error = validate_deflection(deflection)
                .expect_err("batch validator must reject invalid deflection");
            assert_eq!(batch_error.message(), direct_error.to_string());
        }
    }

    #[test]
    fn batch_deflection_defaults_only_when_omitted() {
        let default = get_deflection(&serde_json::json!({}))
            .expect("omitted deflection must use the batch default");
        assert_eq!(
            default.to_bits(),
            DEFAULT_DEFLECTION.to_bits(),
            "omitted deflection must use the exact batch default"
        );
        assert!(get_deflection(&serde_json::json!({"deflection": null})).is_err());
    }

    fn v2_error(kernel: &mut BrepKernel, input: &str) -> serde_json::Value {
        parse(&kernel.execute_batch_v2(input))[0]["error"].clone()
    }

    #[test]
    fn classifies_operations_before_topology_snapshotting() {
        assert_eq!(batch_op_kind("volume"), Some(BatchOpKind::ReadOnly));
        assert_eq!(batch_op_kind("projectEdges"), Some(BatchOpKind::ReadOnly));
        assert_eq!(batch_op_kind("makeBox"), Some(BatchOpKind::Mutating));
        assert_eq!(batch_op_kind("notAnOperation"), None);
    }

    #[test]
    fn legacy_errors_remain_strings_with_unchanged_messages() {
        let cases = [
            (r"[{}]", "missing or invalid 'op' field"),
            (
                r#"[{"op":"notAnOperation","args":{}}]"#,
                "unknown operation: notAnOperation",
            ),
            (
                r#"[{"op":"makeBox","args":{"height":2,"depth":3}}]"#,
                "missing or invalid 'width'",
            ),
            (
                r#"[{"op":"volume","args":{"solid":42,"deflection":0.1}}]"#,
                "invalid solid handle: index 42 is out of bounds",
            ),
        ];

        for (input, expected_message) in cases {
            let mut kernel = BrepKernel::new();
            let response = parse(&kernel.execute_batch(input));
            assert_eq!(response[0]["error"], expected_message);
            assert!(response[0]["error"].is_string());
        }
    }

    #[test]
    fn v2_public_contract_has_stable_codes_messages_and_details() {
        let mut kernel = BrepKernel::new();
        let invalid_json = v2_error(&mut kernel, "[");
        assert_eq!(invalid_json["code"], "invalid_json");
        assert_eq!(invalid_json["details"]["line"], 1);
        assert_eq!(invalid_json["details"]["column"], 1);

        let mut kernel = BrepKernel::new();
        let missing = v2_error(&mut kernel, r"[{}]");
        assert_eq!(missing["code"], "missing_operation");
        assert_eq!(missing["message"], "missing or invalid 'op' field");
        assert_eq!(missing["details"]["operationIndex"], 0);

        let mut kernel = BrepKernel::new();
        let unknown = v2_error(&mut kernel, r#"[{"op":"notAnOperation","args":{}}]"#);
        assert_eq!(unknown["code"], "unknown_operation");
        assert_eq!(unknown["details"]["operation"], "notAnOperation");
        assert_eq!(unknown["details"]["operationIndex"], 0);

        let mut kernel = BrepKernel::new();
        let argument = v2_error(
            &mut kernel,
            r#"[{"op":"makeBox","args":{"height":2,"depth":3}}]"#,
        );
        assert_eq!(argument["code"], "invalid_argument");
        assert_eq!(argument["details"]["argument"], "width");
        assert_eq!(argument["details"]["operation"], "makeBox");

        let mut kernel = BrepKernel::new();
        let handle = v2_error(
            &mut kernel,
            r#"[{"op":"volume","args":{"solid":42,"deflection":0.1}}]"#,
        );
        assert_eq!(handle["code"], "invalid_handle");
        assert_eq!(handle["details"]["entity"], "solid");
        assert_eq!(handle["details"]["index"], 42);

        let mut kernel = BrepKernel::new();
        let response = parse(&kernel.execute_batch_v2(
            r#"[
                {"op":"makeBox","args":{"width":1,"height":1,"depth":1}},
                {"op":"cut","args":{"solidA":0,"solidB":0}}
            ]"#,
        ));
        assert_eq!(response[1]["error"]["code"], "operation_failed");
        assert_eq!(response[1]["error"]["details"]["operation"], "cut");
    }

    #[test]
    fn every_initial_error_code_and_required_detail_shape_is_pinned() {
        let parse_error = serde_json::from_str::<serde_json::Value>("[")
            .expect_err("fixture must be invalid JSON");
        let errors = [
            StructuredWasmError::invalid_json(&parse_error),
            StructuredWasmError::batch_limit("limit", "operations", 10, 11),
            StructuredWasmError::missing_operation(2),
            StructuredWasmError::unknown_operation("futureOp")
                .with_operation_context(3, "futureOp"),
            StructuredWasmError::invalid_argument("bad radius", Some("radius"))
                .with_operation_context(4, "makeCylinder"),
            StructuredWasmError::from(WasmError::InvalidHandle {
                entity: "face",
                index: 9,
            })
            .with_operation_context(5, "extrude"),
            StructuredWasmError::from(brepkit_topology::TopologyError::WireNotClosed)
                .with_operation_context(6, "makePlanarFaceFromWire"),
            StructuredWasmError::operation_failed("refused").with_operation_context(7, "fillet"),
            StructuredWasmError::resource_limit("budget", "mesh_entities", 100, 101),
            StructuredWasmError::internal("serialization failed")
                .with_operation_context(8, "polygonBoolean2d"),
        ];
        let expected_codes = [
            "invalid_json",
            "batch_limit_exceeded",
            "missing_operation",
            "unknown_operation",
            "invalid_argument",
            "invalid_handle",
            "topology_error",
            "operation_failed",
            "resource_limit_exceeded",
            "internal_error",
        ];

        for (error, expected_code) in errors.into_iter().zip(expected_codes) {
            let value = serde_json::to_value(error).expect("structured error must serialize");
            assert_eq!(value["code"], expected_code);
            assert!(value["message"].is_string());
            assert!(value["details"].is_object());
        }

        let resource = serde_json::to_value(StructuredWasmError::resource_limit(
            "budget",
            "mesh_entities",
            100,
            101,
        ))
        .expect("resource error must serialize");
        assert_eq!(resource["details"]["resource"], "mesh_entities");
        assert_eq!(resource["details"]["limit"], 100);
        assert_eq!(resource["details"]["actual"], 101);
    }

    #[test]
    fn typed_mapping_never_changes_the_existing_display_message() {
        let errors = [
            brepkit_operations::OperationsError::InvalidInput {
                reason: "bad argument".to_string(),
            },
            brepkit_operations::OperationsError::Check(
                brepkit_check::CheckError::ClassificationFailed("ambiguous".to_string()),
            ),
            brepkit_operations::OperationsError::NonManifoldResult,
        ];

        for error in errors {
            let expected = error.to_string();
            assert_eq!(StructuredWasmError::from(error).message(), expected);
        }
    }

    #[test]
    fn both_contracts_share_successes_and_error_messages() {
        let input = r#"[
            {"op":"makeBox","args":{"width":2,"height":3,"depth":4}},
            {"op":"volume","args":{"solid":0,"deflection":0.1}},
            {"op":"volume","args":{"solid":99,"deflection":0.1}},
            {"op":"notAnOperation","args":{}}
        ]"#;
        let legacy = parse(&BrepKernel::new().execute_batch(input));
        let v2 = parse(&BrepKernel::new().execute_batch_v2(input));

        assert_eq!(legacy[0], v2[0]);
        assert_eq!(legacy[1], v2[1]);
        assert_eq!(legacy[2]["error"], v2[2]["error"]["message"]);
        assert_eq!(legacy[3]["error"], v2[3]["error"]["message"]);
    }

    #[test]
    fn malformed_batch_corpus_returns_json_and_never_stops_later_items() {
        let malformed_documents = [
            "",
            "[",
            "null",
            "{}",
            r"[1,2",
            r#"[{"op":null}]"#,
            r#"[{"op":false}]"#,
            "\0",
        ];
        for input in malformed_documents {
            let legacy = BrepKernel::new().execute_batch(input);
            let v2 = BrepKernel::new().execute_batch_v2(input);
            assert!(parse(&legacy).is_array());
            assert!(parse(&v2).is_array());
        }

        let mut kernel = BrepKernel::new();
        let response =
            parse(&kernel.execute_batch_v2(
                r#"[{}, {"op":"makeBox","args":{"width":1,"height":1,"depth":1}}]"#,
            ));
        assert_eq!(response[0]["error"]["code"], "missing_operation");
        assert_eq!(response[1]["ok"], 0);
    }

    #[test]
    fn rejects_too_many_operations_before_dispatch() {
        let mut kernel = BrepKernel::new();
        let operation = serde_json::json!({"op": "volume", "args": {}});
        let json = serde_json::Value::Array(vec![operation; MAX_BATCH_OPERATIONS + 1]).to_string();
        let response = kernel.execute_batch(&json);
        assert!(response.contains("operation limit"));

        let v2 = parse(&kernel.execute_batch_v2(&json));
        assert_eq!(v2[0]["error"]["code"], "batch_limit_exceeded");
        assert_eq!(v2[0]["error"]["details"]["resource"], "operations");
        assert_eq!(v2[0]["error"]["details"]["limit"], MAX_BATCH_OPERATIONS);
        assert_eq!(
            v2[0]["error"]["details"]["actual"],
            MAX_BATCH_OPERATIONS + 1
        );
    }

    #[test]
    fn rejects_oversized_json_before_parsing() {
        let mut kernel = BrepKernel::new();
        let json = " ".repeat(MAX_BATCH_JSON_BYTES + 1);
        let response = kernel.execute_batch(&json);
        assert!(response.contains("byte limit"));

        let v2 = parse(&kernel.execute_batch_v2(&json));
        assert_eq!(v2[0]["error"]["code"], "batch_limit_exceeded");
        assert_eq!(v2[0]["error"]["details"]["resource"], "json_bytes");
    }
}

#[cfg(test)]
mod rollback_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use crate::kernel::BrepKernel;

    /// Live entity counts across every arena a batch op can touch.
    type Counts = (usize, usize, usize, usize, usize, usize);

    fn counts(k: &BrepKernel) -> Counts {
        let t = k.topo();
        (
            t.num_vertices(),
            t.num_edges(),
            t.num_wires(),
            t.num_faces(),
            t.num_shells(),
            t.num_solids(),
        )
    }

    fn kernel_with_box() -> BrepKernel {
        let mut k = BrepKernel::new();
        let out = k.execute_batch(r#"[{"op":"makeBox","args":{"width":2,"height":2,"depth":2}}]"#);
        assert!(!out.contains("error"), "seed failed: {out}");
        k
    }

    /// A `pushPullFace` far enough to consume the solid fails *after* building
    /// geometry. Without rollback it leaves the arenas dirty.
    const DIRTY_FAILING_OP: &str = r#"{"solid":0,"face":0,"distance":-50.0}"#;

    /// Pins the premise of [`failed_batch_op_leaves_topology_untouched`]: the
    /// operation it uses really does mutate before failing. If this ever stops
    /// holding, that test is passing vacuously and needs a new operation.
    #[test]
    fn chosen_failing_op_really_dirties_topology_without_rollback() {
        let mut k = kernel_with_box();
        let before = counts(&k);
        let args: serde_json::Value = serde_json::from_str(DIRTY_FAILING_OP).unwrap();

        // `dispatch_op` is the raw path with no snapshot around it.
        let result = k.dispatch_op("pushPullFace", &args);

        assert!(result.is_err(), "expected the op to fail, got {result:?}");
        assert_ne!(
            before,
            counts(&k),
            "op no longer leaves partial mutation; pick another for the rollback test"
        );
    }

    /// A mid-batch failure must leave topology exactly as it was, and the
    /// operations after it must still see the correct pre-failure state.
    #[test]
    fn failed_batch_op_leaves_topology_untouched() {
        let mut k = kernel_with_box();
        let before = counts(&k);

        let batch = format!(
            r#"[
                {{"op":"pushPullFace","args":{DIRTY_FAILING_OP}}},
                {{"op":"volume","args":{{"solid":0,"deflection":0.05}}}}
            ]"#
        );
        let parsed: serde_json::Value = serde_json::from_str(&k.execute_batch(&batch)).unwrap();

        // The failing op reports an error...
        assert!(
            parsed[0]["error"].is_string(),
            "expected op 0 to fail, got {parsed}"
        );
        // ...and left no live entity behind.
        assert_eq!(
            before,
            counts(&k),
            "failed op mutated topology; rollback regressed"
        );

        // The untouched solid still resolves and measures correctly.
        let vol = parsed[1]["ok"].as_f64().expect("volume should succeed");
        assert!((vol - 8.0).abs() < 0.05, "expected ~8.0, got {vol}");

        // A later mutating op still builds on the correct state.
        let parsed: serde_json::Value = serde_json::from_str(
            &k.execute_batch(r#"[{"op":"makeBox","args":{"width":1,"height":1,"depth":1}}]"#),
        )
        .unwrap();
        let handle = parsed[0]["ok"].as_u64().expect("makeBox should succeed");
        let parsed: serde_json::Value = serde_json::from_str(&k.execute_batch(&format!(
            r#"[{{"op":"volume","args":{{"solid":{handle},"deflection":0.05}}}}]"#
        )))
        .unwrap();
        let vol = parsed[0]["ok"].as_f64().expect("volume should succeed");
        assert!((vol - 1.0).abs() < 0.05, "expected ~1.0, got {vol}");
    }

    /// Rollback must survive a *second* failure later in the same batch, since
    /// the snapshot is taken per item rather than once per batch.
    #[test]
    fn repeated_failures_each_roll_back_independently() {
        let mut k = kernel_with_box();

        // A solid created between two failures keeps its handle; rollback
        // retires the failed op's slots rather than reusing them, so the
        // handle is read back from the result instead of assumed.
        let batch = format!(
            r#"[
                {{"op":"pushPullFace","args":{DIRTY_FAILING_OP}}},
                {{"op":"makeBox","args":{{"width":3,"height":3,"depth":3}}}},
                {{"op":"pushPullFace","args":{DIRTY_FAILING_OP}}}
            ]"#
        );
        let parsed: serde_json::Value = serde_json::from_str(&k.execute_batch(&batch)).unwrap();

        assert!(parsed[0]["error"].is_string(), "op 0 should fail: {parsed}");
        assert!(parsed[2]["error"].is_string(), "op 2 should fail: {parsed}");
        let handle = parsed[1]["ok"].as_u64().expect("makeBox should succeed");

        // The box created between the two failures survived the second one.
        let parsed: serde_json::Value = serde_json::from_str(&k.execute_batch(&format!(
            r#"[{{"op":"volume","args":{{"solid":{handle},"deflection":0.05}}}}]"#
        )))
        .unwrap();
        let vol = parsed[0]["ok"].as_f64().expect("volume should succeed");
        assert!((vol - 27.0).abs() < 0.2, "expected ~27.0, got {vol}");
    }

    /// Guards the `is_read_only_op` allowlist: every listed op that is
    /// exercisable here must leave topology byte-identical. A drifting entry is
    /// only a lost optimisation, never a wrong rollback — but it should still
    /// be caught.
    #[test]
    fn read_only_ops_do_not_mutate_topology() {
        let ops = [
            r#"{"op":"volume","args":{"solid":0,"deflection":0.1}}"#,
            r#"{"op":"surfaceArea","args":{"solid":0,"deflection":0.1}}"#,
            r#"{"op":"boundingBox","args":{"solid":0}}"#,
            r#"{"op":"centerOfMass","args":{"solid":0,"deflection":0.1}}"#,
            r#"{"op":"massProperties","args":{"solid":0,"deflection":0.1}}"#,
            r#"{"op":"validateSolid","args":{"solid":0}}"#,
            r#"{"op":"solidEdges","args":{"solid":0}}"#,
            r#"{"op":"meshQuality","args":{"solid":0,"deflection":0.1}}"#,
            r#"{"op":"classifyPoint","args":{"solid":0,"x":0.5,"y":0.5,"z":0.5}}"#,
        ];

        for op in ops {
            let mut k = kernel_with_box();
            let before = counts(&k);
            let out = k.execute_batch(&format!("[{op}]"));
            assert!(!out.contains("\"error\""), "{op} failed: {out}");
            assert_eq!(before, counts(&k), "{op} mutated topology");
        }
    }
}
