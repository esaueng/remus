//! Batch execution and dispatch bindings.

#![allow(clippy::missing_errors_doc, clippy::too_many_lines)]

use std::rc::Rc;

use wasm_bindgen::prelude::*;

use remus_math::mat::Mat4;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{self, BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure;
use remus_operations::push_pull::{move_faces, push_pull_face, resize_cylindrical_face};
use remus_operations::query::opposing_planar_face_pairs;
use remus_operations::revolve::revolve;
use remus_operations::sweep::sweep;
use remus_operations::transform::transform_solid;
use remus_topology::edge::EdgeCurve;

use super::operations::{parse_variable_fillet_specs, validate_move_faces_topology_work};
use crate::error::{
    StructuredWasmError, WasmError, validate_face_pair_count, validate_work_count,
    validate_work_product,
};
use crate::handles::{
    compound_id_to_u32, edge_id_to_u32, face_id_to_u32, shell_id_to_u32, solid_id_to_u32,
    wire_id_to_u32,
};
use crate::helpers::{
    TOL, classify_to_string, get_bool, get_f64, get_f64_array, get_u32, get_u32_array,
    get_u32_array_optional, panic_message, try_chamfer,
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

fn get_angular_tolerance(args: &serde_json::Value) -> Result<f64, StructuredWasmError> {
    let angular_tolerance = match args.get("angularTolerance") {
        None => remus_math::chord::DEFAULT_ANGULAR_TOL,
        Some(_) => get_f64(args, "angularTolerance")?,
    };
    crate::error::validate_positive(angular_tolerance, "angularTolerance")
        .map_err(StructuredWasmError::from)?;
    Ok(angular_tolerance)
}

fn get_optional_work_budget(
    args: &serde_json::Value,
    name: &str,
) -> Result<Option<usize>, StructuredWasmError> {
    match args.get(name) {
        None | Some(serde_json::Value::Null) => Ok(None),
        Some(value) => {
            let raw = value.as_f64().ok_or_else(|| {
                StructuredWasmError::invalid_argument(
                    format!("invalid '{name}': expected number"),
                    Some(name),
                )
            })?;
            crate::error::validate_iteration_budget(raw, name)
                .map(Some)
                .map_err(|error| {
                    StructuredWasmError::invalid_argument(error.to_string(), Some(name))
                })
        }
    }
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
  | "cancelled"
  | "resource_limit_exceeded"
  | "internal_error";

/** Kernel-wide failure categories carried by `executeBatchV2` errors. */
export type BatchFailureCategoryV2 =
  | "invalid_input"
  | "invalid_topology"
  | "unsupported"
  | "nonconvergence"
  | "resource_limit"
  | "tolerance_violation"
  | "quality_refused"
  | "cancelled"
  | "internal";

/** Machine-readable error returned by `executeBatchV2`. */
export interface BatchErrorV2 {
  code: BatchErrorCodeV2;
  category: BatchFailureCategoryV2;
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
        #[cfg(feature = "io")]
        "exportStep" | "exportStepSheet" => Some(BatchOpKind::ReadOnly),
        #[cfg(feature = "io")]
        "importStepBodies" | "importStepWithValidation" => Some(BatchOpKind::Mutating),
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
        | "sheetArea"
        | "sheetBoundingBox"
        | "sheetCenterOfArea"
        | "sheetVolume"
        | "surfaceArea"
        | "tessellateSheet"
        | "validateSheetBody"
        | "validateSolid"
        | "resolveOperationOutput"
        | "journalSummary"
        | "getFaceName"
        | "getBlendRegion"
        | "getSolidFaces"
        | "getFaceNormal"
        | "getFaceCurvature"
        | "getFaceMinRadius"
        | "getFaceVertexPositions"
        | "getOpposingPlanarFacePairs"
        | "wireLength"
        | "volume" => Some(BatchOpKind::ReadOnly),
        // Serialized-reference ops: their dispatch arms in `naming.rs` are
        // `io`-gated because the reference codec is. Classifying them without
        // the feature would admit an op that can never dispatch, so the batch
        // would take a rollback share of the topology only to fail afterwards.
        #[cfg(feature = "io")]
        "makeOperationOutputRef"
        | "captureSignatureRef"
        | "addRefDiscriminator"
        | "resolveRef"
        | "resolveRefFaceAttributes" => Some(BatchOpKind::ReadOnly),
        "makeBox"
        | "makeCompound"
        | "fuseJournaled"
        | "cutJournaled"
        | "intersectJournaled"
        | "journalBarrier"
        | "propagateAttributesForOp"
        | "setFaceName"
        | "fuseWithEntityEvolution"
        | "cutWithEntityEvolution"
        | "intersectWithEntityEvolution"
        | "filletJournaled"
        | "chamferJournaled"
        | "linearPatternJournaled"
        | "imprint"
        | "offsetJournaled"
        | "makeCylinder"
        | "makeSphere"
        | "makeCone"
        | "makeTorus"
        | "makeEllipsoid"
        | "fuse"
        | "cut"
        | "intersect"
        | "booleanRegions"
        | "booleanCompoundRegions"
        | "booleanWithQuality"
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
        | "moveFaces"
        | "resizeCylindricalFace"
        | "extrude"
        | "revolve"
        | "sweep"
        | "sweepWire"
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
        | "healSolidDetailed"
        | "repairSolid"
        | "repairSolidDetailed"
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
        | "splitBySheet"
        | "trimSheetBySolid"
        | "trimSheetBySheet"
        | "mutualTrimSheets"
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
        | "makeSheetBody"
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
/// Must match `remus_heal::construct::convert_surface`'s private frame
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
        use remus_geometry::convert::{circle_to_nurbs, ellipse_to_nurbs, line_to_nurbs};
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
                    remus_heal::construct::convert_curve::hyperbola_to_nurbs(h, lo, hi).map_err(
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
                    remus_heal::construct::convert_curve::parabola_to_nurbs(pb, lo, hi).map_err(
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
    /// `remus_heal::construct::convert_surface`. Plane and cone parameter
    /// ranges are derived from the face's boundary vertices.
    pub(crate) fn extract_nurbs_surface(&self, face: u32) -> Result<NurbsSurface, WasmError> {
        use remus_heal::construct::convert_surface;
        use remus_topology::face::FaceSurface;

        let face_id = self.resolve_face(face)?;
        let face_data = self.topo.face(face_id)?;

        let map_err = |context: &str, e: remus_heal::HealError| WasmError::InvalidInput {
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
        face_id: remus_topology::face::FaceId,
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
            let (t0, t1) = edge
                .strict_domain()
                .map_err(|error| WasmError::InvalidInput {
                    reason: format!("batch plane-face UV bounds require edge authority: {error}"),
                })?;
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
        face_id: remus_topology::face::FaceId,
        surface: &remus_topology::face::FaceSurface,
    ) -> Result<(f64, f64), WasmError> {
        let verts = remus_topology::explorer::face_vertices(&self.topo, face_id)?;
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

    /// Create an edge over the `NurbsCurve`'s authoritative native domain.
    pub(crate) fn nurbs_curve_to_edge(
        &mut self,
        curve: NurbsCurve,
    ) -> Result<remus_topology::edge::EdgeId, WasmError> {
        let (t0, t1) = curve.domain();
        let start = curve.evaluate(t0);
        let end = curve.evaluate(t1);
        self.add_certified_curve_edge(
            EdgeCurve::NurbsCurve(curve),
            (t0, t1),
            start,
            end,
            (start - end).length() <= TOL,
            TOL,
        )
    }

    /// Create an edge from a `NurbsCurve`, evaluating its endpoints.
    pub(crate) fn nurbs_curve_to_edge_from_curve(
        &mut self,
        curve: &NurbsCurve,
    ) -> Result<remus_topology::edge::EdgeId, WasmError> {
        let (t0, t1) = curve.domain();
        let start = curve.evaluate(t0);
        let end = curve.evaluate(t1);
        self.add_certified_curve_edge(
            EdgeCurve::NurbsCurve(curve.clone()),
            (t0, t1),
            start,
            end,
            (start - end).length() <= TOL,
            TOL,
        )
    }

    /// Create a face from a `NurbsSurface` with a rectangular domain wire.
    pub(crate) fn nurbs_surface_to_face(
        &mut self,
        surface: NurbsSurface,
    ) -> Result<remus_topology::face::FaceId, JsError> {
        Ok(remus_topology::builder::make_nurbs_face(
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
            #[cfg(feature = "io")]
            "exportStep" => {
                let solid = get_u32(args, "solid")?;
                let solid_id = self
                    .resolve_solid(solid)
                    .map_err(StructuredWasmError::from)?;
                let options = match args.get("options") {
                    None | Some(serde_json::Value::Null) => {
                        remus_io::step::StepWriteOptions::default()
                    }
                    Some(value) => serde_json::from_value(value.clone()).map_err(|error| {
                        StructuredWasmError::invalid_argument(
                            format!("invalid STEP write options: {error}"),
                            Some("options"),
                        )
                    })?,
                };
                let step =
                    remus_io::step::write_step_with_options(&self.topo, &[solid_id], &options)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::Value::String(step))
            }
            #[cfg(feature = "io")]
            "exportStepSheet" => {
                let sheet = get_u32(args, "sheet")?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let options = match args.get("options") {
                    None | Some(serde_json::Value::Null) => {
                        remus_io::step::StepWriteOptions::default()
                    }
                    Some(value) => serde_json::from_value(value.clone()).map_err(|error| {
                        StructuredWasmError::invalid_argument(
                            format!("invalid STEP write options: {error}"),
                            Some("options"),
                        )
                    })?,
                };
                let step = remus_io::step::write_step_bodies_with_options(
                    &self.topo,
                    &[],
                    &[sheet_id],
                    &options,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::Value::String(step))
            }
            #[cfg(feature = "io")]
            "importStepBodies" => {
                let data = args["data"].as_str().ok_or_else(|| {
                    StructuredWasmError::invalid_argument(
                        "missing or invalid 'data' STEP string",
                        Some("data"),
                    )
                })?;
                let optional_f64 =
                    |name: &'static str| -> Result<Option<f64>, StructuredWasmError> {
                        match args.get(name) {
                            None | Some(serde_json::Value::Null) => Ok(None),
                            Some(value) => value.as_f64().map(Some).ok_or_else(|| {
                                StructuredWasmError::invalid_argument(
                                    format!("'{name}' must be a number"),
                                    Some(name),
                                )
                            }),
                        }
                    };
                let limits = super::io::import_limits_from(
                    optional_f64("maxInputBytes")?,
                    optional_f64("maxEntities")?,
                )
                .map_err(StructuredWasmError::from)?;
                let result =
                    remus_io::step::read_step_bodies_with_limits(data, self.topo_mut(), limits)
                        .map_err(StructuredWasmError::from)?;
                Ok(super::io::step_body_result_json(&result))
            }
            #[cfg(feature = "io")]
            "importStepWithValidation" => {
                let data = args["data"].as_str().ok_or_else(|| {
                    StructuredWasmError::invalid_argument(
                        "missing or invalid 'data' STEP string",
                        Some("data"),
                    )
                })?;
                let options = match args.get("options") {
                    None | Some(serde_json::Value::Null) => {
                        remus_io::step::StepValidationOptions::default()
                    }
                    Some(value) => serde_json::from_value(value.clone()).map_err(|error| {
                        StructuredWasmError::invalid_argument(
                            format!("invalid STEP validation options: {error}"),
                            Some("options"),
                        )
                    })?,
                };
                let optional_f64 =
                    |name: &'static str| -> Result<Option<f64>, StructuredWasmError> {
                        match args.get(name) {
                            None | Some(serde_json::Value::Null) => Ok(None),
                            Some(value) => value.as_f64().map(Some).ok_or_else(|| {
                                StructuredWasmError::invalid_argument(
                                    format!("'{name}' must be a number"),
                                    Some(name),
                                )
                            }),
                        }
                    };
                let limits = super::io::import_limits_from(
                    optional_f64("maxInputBytes")?,
                    optional_f64("maxEntities")?,
                )
                .map_err(StructuredWasmError::from)?;
                let result = remus_io::step::read_step_with_validation(
                    data,
                    self.topo_mut(),
                    limits,
                    options,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(super::io::step_validation_result_json(result))
            }
            "makeBox" => {
                let w = get_f64(args, "width")?;
                let h = get_f64(args, "height")?;
                let d = get_f64(args, "depth")?;
                let solid = remus_operations::primitives::make_box(self.topo_mut(), w, h, d)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeCompound" => {
                let handles = get_u32_array(args, "solids")?;
                let count = u32::try_from(handles.len()).unwrap_or(u32::MAX);
                validate_work_count(count, "solids").map_err(StructuredWasmError::from)?;
                let solids = handles
                    .into_iter()
                    .map(|handle| {
                        self.resolve_solid(handle)
                            .map_err(StructuredWasmError::from)
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                let compound = self
                    .topo_mut()
                    .add_compound(remus_topology::compound::Compound::new(solids));
                Ok(serde_json::json!(compound_id_to_u32(compound)))
            }
            "makeCylinder" => {
                let r = get_f64(args, "radius")?;
                let h = get_f64(args, "height")?;
                let solid = remus_operations::primitives::make_cylinder(self.topo_mut(), r, h)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeSphere" => {
                let r = get_f64(args, "radius")?;
                let segments = get_u32(args, "segments").unwrap_or(16);
                let segments =
                    validate_work_count(segments, "segments").map_err(StructuredWasmError::from)?;
                let solid = remus_operations::primitives::make_sphere(self.topo_mut(), r, segments)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeCone" => {
                let br = get_f64(args, "bottomRadius")?;
                let tr = get_f64(args, "topRadius")?;
                let h = get_f64(args, "height")?;
                let solid = remus_operations::primitives::make_cone(self.topo_mut(), br, tr, h)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "makeTorus" => {
                let major = get_f64(args, "majorRadius")?;
                let minor = get_f64(args, "minorRadius")?;
                let segments = get_u32(args, "segments").unwrap_or(16);
                let segments =
                    validate_work_count(segments, "segments").map_err(StructuredWasmError::from)?;
                let solid = remus_operations::primitives::make_torus(
                    self.topo_mut(),
                    major,
                    minor,
                    segments,
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
                let solid = remus_operations::primitives::make_sphere(self.topo_mut(), 1.0, 16)
                    .map_err(StructuredWasmError::from)?;
                let mat = remus_math::mat::Mat4::scale(rx, ry, rz);
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
            "booleanRegions" => {
                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let operation = args["operation"].as_str().ok_or_else(|| {
                    StructuredWasmError::invalid_argument(
                        "missing or invalid 'operation' string",
                        Some("operation"),
                    )
                })?;
                let bool_op = match operation {
                    "fuse" | "union" => BooleanOp::Fuse,
                    "cut" | "difference" => BooleanOp::Cut,
                    "intersect" | "intersection" => BooleanOp::Intersect,
                    _ => {
                        return Err(StructuredWasmError::invalid_argument(
                            format!("unknown boolean op: {operation}"),
                            Some("operation"),
                        ));
                    }
                };
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let result = remus_operations::boolean::boolean_regions(
                    self.topo_mut(),
                    bool_op,
                    a_id,
                    b_id,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(crate::handles::compound_id_to_u32(
                    result.compound
                )))
            }
            "booleanCompoundRegions" => {
                let a = get_u32(args, "compoundA")?;
                let b = get_u32(args, "compoundB")?;
                let operation = args["operation"].as_str().ok_or_else(|| {
                    StructuredWasmError::invalid_argument(
                        "missing or invalid 'operation' string",
                        Some("operation"),
                    )
                })?;
                let bool_op = match operation {
                    "fuse" | "union" => BooleanOp::Fuse,
                    "cut" | "difference" => BooleanOp::Cut,
                    "intersect" | "intersection" => BooleanOp::Intersect,
                    _ => {
                        return Err(StructuredWasmError::invalid_argument(
                            format!("unknown boolean op: {operation}"),
                            Some("operation"),
                        ));
                    }
                };
                let a_id = self
                    .resolve_compound(a)
                    .map_err(StructuredWasmError::from)?;
                let b_id = self
                    .resolve_compound(b)
                    .map_err(StructuredWasmError::from)?;
                let result = remus_operations::boolean::boolean_compound_regions(
                    self.topo_mut(),
                    bool_op,
                    a_id,
                    b_id,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(compound_id_to_u32(result.compound)))
            }
            "booleanWithQuality" => {
                use remus_operations::boolean::{BooleanQuality, boolean_with_context};

                let a = get_u32(args, "solidA")?;
                let b = get_u32(args, "solidB")?;
                let operation = args["operation"].as_str().ok_or_else(|| {
                    StructuredWasmError::invalid_argument(
                        "missing or invalid 'operation' string",
                        Some("operation"),
                    )
                })?;
                let bool_op = match operation {
                    "fuse" | "union" => BooleanOp::Fuse,
                    "cut" | "difference" => BooleanOp::Cut,
                    "intersect" | "intersection" => BooleanOp::Intersect,
                    _ => {
                        return Err(StructuredWasmError::invalid_argument(
                            format!("unknown boolean op: {operation}"),
                            Some("operation"),
                        ));
                    }
                };
                let exact_only = match args.get("exactOnly") {
                    None | Some(serde_json::Value::Null) => false,
                    Some(value) => value.as_bool().ok_or_else(|| {
                        StructuredWasmError::invalid_argument(
                            "invalid 'exactOnly': expected boolean",
                            Some("exactOnly"),
                        )
                    })?,
                };
                let newton_budget = get_optional_work_budget(args, "newtonIterations")?;
                let subdivision_budget = get_optional_work_budget(args, "subdivisionDepth")?;
                let march_budget = get_optional_work_budget(args, "marchSteps")?;
                let queue_budget = get_optional_work_budget(args, "queueSize")?;
                let segment_budget = get_optional_work_budget(args, "segments")?;
                let branch_budget = get_optional_work_budget(args, "branchesPerDirection")?;
                let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
                let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
                let context = super::booleans::quality_context(
                    exact_only,
                    newton_budget,
                    subdivision_budget,
                    march_budget,
                    queue_budget,
                    segment_budget,
                    branch_budget,
                );
                let outcome = boolean_with_context(self.topo_mut(), bool_op, a_id, b_id, &context)
                    .map_err(StructuredWasmError::from)?;
                match outcome.quality {
                    BooleanQuality::Exact => Ok(serde_json::json!({
                        "solid": solid_id_to_u32(outcome.solid),
                        "quality": "exact"
                    })),
                    BooleanQuality::Approximate { deflection } => Ok(serde_json::json!({
                        "solid": solid_id_to_u32(outcome.solid),
                        "quality": "approximate",
                        "deflection": deflection
                    })),
                }
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
                let opts = remus_operations::boolean::BooleanOptions {
                    unify_faces: unify_faces.unwrap_or(true),
                    ..Default::default()
                };
                let result = remus_operations::boolean::boolean_with_options(
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
                let (result, evolution) = remus_operations::boolean::boolean_with_evolution(
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
                let pairs = remus_algo::diagnostic::detect_coincident_faces(
                    self.topo(),
                    a_id,
                    b_id,
                    remus_math::tolerance::Tolerance::default(),
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
                let tools: Vec<remus_topology::solid::SolidId> = tool_arr
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
                let solids: Vec<remus_topology::solid::SolidId> = solid_arr
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
                    .add_compound(remus_topology::compound::Compound::new(solids));
                let result = remus_operations::compound_ops::fuse_all(self.topo_mut(), compound)
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
            "wireLength" => {
                let wire = get_u32(args, "wire")?;
                let wire_id = self.resolve_wire(wire).map_err(StructuredWasmError::from)?;
                let length =
                    measure::body_length(&self.topo, remus_topology::BodyId::Wire(wire_id))
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(length))
            }
            "validateSolid" => {
                let solid = get_u32(args, "solid")?;
                let solid_id = self
                    .resolve_solid(solid)
                    .map_err(StructuredWasmError::from)?;
                let report = remus_operations::validate::validate_solid(&self.topo, solid_id)
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
            "sheetArea" => {
                let sheet = get_u32(args, "sheet")?;
                let deflection = get_deflection(args)?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let area = measure::body_surface_area(
                    &self.topo,
                    remus_topology::BodyId::Shell(sheet_id),
                    deflection,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(area))
            }
            "sheetBoundingBox" => {
                let sheet = get_u32(args, "sheet")?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let aabb = measure::sheet_bounding_box(&self.topo, sheet_id)
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
            "sheetCenterOfArea" => {
                let sheet = get_u32(args, "sheet")?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let center = measure::sheet_center_of_area(&self.topo, sheet_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!([center.x(), center.y(), center.z()]))
            }
            "sheetVolume" => {
                let sheet = get_u32(args, "sheet")?;
                let deflection = get_deflection(args)?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let volume = measure::body_volume(
                    &self.topo,
                    remus_topology::BodyId::Shell(sheet_id),
                    deflection,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(volume))
            }
            "validateSheetBody" => {
                let sheet = get_u32(args, "sheet")?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let report = remus_check::validate::validate_sheet_body(
                    &self.topo,
                    sheet_id,
                    &remus_check::validate::ValidateOptions::default(),
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "errorCount": report.error_count(),
                    "warningCount": report.warning_count(),
                    "issues": report.issues.into_iter().map(|issue| serde_json::json!({
                        "severity": match issue.severity {
                            remus_check::validate::Severity::Info => "info",
                            remus_check::validate::Severity::Error => "error",
                            remus_check::validate::Severity::Warning => "warning",
                        },
                        "description": issue.description,
                    })).collect::<Vec<_>>(),
                }))
            }
            "tessellateSheet" => {
                let sheet = get_u32(args, "sheet")?;
                let deflection = get_deflection(args)?;
                let angular_tolerance = get_angular_tolerance(args)?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let mesh = remus_operations::tessellate::tessellate_body_with_tolerance(
                    &self.topo,
                    remus_topology::BodyId::Shell(sheet_id),
                    deflection,
                    angular_tolerance,
                )
                .map_err(StructuredWasmError::from)?;
                let positions = mesh
                    .positions
                    .iter()
                    .flat_map(|point| [point.x(), point.y(), point.z()])
                    .collect::<Vec<_>>();
                let normals = mesh
                    .normals
                    .iter()
                    .flat_map(|normal| [normal.x(), normal.y(), normal.z()])
                    .collect::<Vec<_>>();
                Ok(serde_json::json!({
                    "positions": positions,
                    "normals": normals,
                    "indices": mesh.indices,
                }))
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
                let angular_tolerance = get_angular_tolerance(args)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let mesh = remus_operations::tessellate::tessellate_solid_with_tolerance(
                    &self.topo,
                    solid_id,
                    deflection,
                    angular_tolerance,
                )
                .map_err(StructuredWasmError::from)?;
                let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
                Ok(serde_json::json!({
                    "triangleCount": quality.triangle_count,
                    "boundaryEdges": quality.boundary_edges,
                    "nonManifoldEdges": quality.non_manifold_edges,
                    "eulerCharacteristic": quality.euler_characteristic,
                    "isWatertight": quality.is_watertight(),
                }))
            }
            "solidEdges" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edges = remus_topology::explorer::solid_edges(&self.topo, solid_id)
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
                    remus_operations::distance::solid_to_solid_distance(&self.topo, a_id, b_id)
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
                let copy = remus_operations::copy::copy_solid(self.topo_mut(), solid_id)
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
                let copy = remus_operations::copy::copy_and_transform_solid(
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
            "moveFaces" => {
                let s = get_u32(args, "solid")?;
                let faces = get_u32_array(args, "faces")?;
                let distance = get_f64(args, "distance")?;
                let face_count = u32::try_from(faces.len()).unwrap_or(u32::MAX);
                validate_work_count(face_count, "faces").map_err(StructuredWasmError::from)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_ids = faces
                    .into_iter()
                    .map(|face| self.resolve_face(face).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                validate_move_faces_topology_work(self.topo(), solid_id, &face_ids)
                    .map_err(StructuredWasmError::from)?;
                let result = move_faces(self.topo_mut(), solid_id, &face_ids, distance)
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
            "sweepWire" => {
                let profile = get_u32(args, "profile")?;
                let path_edge = get_u32(args, "pathEdge")?;
                let wire_id = self
                    .resolve_wire(profile)
                    .map_err(StructuredWasmError::from)?;
                let path_curve = self
                    .extract_nurbs_curve(path_edge)
                    .map_err(StructuredWasmError::from)?;
                let solid =
                    remus_operations::sweep::sweep_wire(self.topo_mut(), wire_id, &path_curve)
                        .map_err(StructuredWasmError::from)?;
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
                let result = remus_operations::sweep::sweep_with_options(
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
                let result = remus_operations::helix::helical_sweep(
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
                let faces: Vec<u32> = get_u32_array_optional(args, "faces")?;
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
                let sections: Vec<(remus_topology::face::FaceId, f64)> = faces
                    .iter()
                    .zip(params.iter())
                    .map(|(&h, &p)| {
                        self.resolve_face(h)
                            .map(|f| (f, p))
                            .map_err(StructuredWasmError::from)
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                let solid = remus_operations::sweep::multi_section_sweep(
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
                    remus_operations::sweep::sweep_guided(self.topo_mut(), face_id, &spine, aux)
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
                let solid = remus_operations::primitives::make_minkowski_sum(self.topo_mut(), a, b)
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
                let result = remus_operations::projection::project_edges(
                    &self.topo,
                    solid,
                    origin,
                    dir,
                    x_axis,
                    hidden_lines,
                    deflection,
                )
                .map_err(StructuredWasmError::from)?;
                let flatten = |polys: &[Vec<remus_math::vec::Point2>]| -> Vec<Vec<f64>> {
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
                let edge_handles: Vec<u32> = get_u32_array_optional(args, "edges")?;
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
                    Ok(inner) => inner.map_err(StructuredWasmError::blend_failure)?,
                    Err(panic_info) => {
                        self.poisoned = true;
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
                let edge_handles: Vec<u32> = get_u32_array_optional(args, "edges")?;
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                // Same engine chain, whole-selection rule, and panic guard as
                // the `fillet` binding: a selection whose only succeeding
                // subset is the planar one is an `edges-not-blended` refusal
                // here too, never a quietly reduced answer.
                let fillet_result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    crate::helpers::fillet_whole_selection(
                        self.topo_mut(),
                        solid_id,
                        &edge_ids,
                        radius,
                    )
                }));
                let result = match fillet_result {
                    Ok(inner) => inner.map_err(StructuredWasmError::blend_failure)?,
                    Err(panic_info) => {
                        self.poisoned = true;
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
                let edge_specs =
                    parse_variable_fillet_specs(self, specs).map_err(StructuredWasmError::from)?;
                let result = remus_operations::fillet::fillet_variable_with_setbacks(
                    self.topo_mut(),
                    solid_id,
                    &edge_specs,
                )
                .map_err(StructuredWasmError::blend_failure)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "filletV2" => {
                let s = get_u32(args, "solid")?;
                let radius = get_f64(args, "radius")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edge_handles: Vec<u32> = get_u32_array_optional(args, "edges")?;
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = remus_operations::blend_ops::fillet_v2(
                    self.topo_mut(),
                    solid_id,
                    &edge_ids,
                    radius,
                )
                .map_err(StructuredWasmError::blend_failure)?;
                Ok(serde_json::json!(solid_id_to_u32(result.solid)))
            }
            "chamferV2" => {
                let s = get_u32(args, "solid")?;
                let d1 = get_f64(args, "d1")?;
                let d2 = get_f64(args, "d2")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let edge_handles: Vec<u32> = get_u32_array_optional(args, "edges")?;
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = remus_operations::blend_ops::chamfer_v2(
                    self.topo_mut(),
                    solid_id,
                    &edge_ids,
                    d1,
                    d2,
                )
                .map_err(StructuredWasmError::blend_failure)?;
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
                let edge_handles: Vec<u32> = get_u32_array_optional(args, "edges")?;
                let edge_ids: Vec<_> = edge_handles
                    .iter()
                    .map(|&h| self.resolve_edge(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = remus_operations::blend_ops::chamfer_distance_angle(
                    self.topo_mut(),
                    solid_id,
                    &edge_ids,
                    distance,
                    angle,
                )
                .map_err(StructuredWasmError::blend_failure)?;
                Ok(serde_json::json!(solid_id_to_u32(result.solid)))
            }
            "shell" => {
                let s = get_u32(args, "solid")?;
                let thickness = get_f64(args, "thickness")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_handles: Vec<u32> = get_u32_array_optional(args, "faces")?;
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = remus_operations::shell_op::shell(
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
                let result = remus_operations::mirror::mirror(
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
                remus_operations::heal::unify_faces(self.topo_mut(), solid_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid_id)))
            }
            "convertToBspline" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let count = remus_operations::heal::convert_to_bspline(self.topo_mut(), solid_id)
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
                    remus_operations::heal::convert_to_elementary(self.topo_mut(), solid_id, tol)
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
                remus_operations::heal::heal_solid(self.topo_mut(), solid_id, tol)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid_id)))
            }
            "healSolidDetailed" => {
                let solid = get_u32(args, "solid")?;
                let result = self
                    .heal_solid_detailed_impl(solid)
                    .map_err(StructuredWasmError::from)?;
                serde_json::to_value(result).map_err(StructuredWasmError::from)
            }
            "repairSolid" => {
                let s = get_u32(args, "solid")?;
                let tol = get_f64(args, "tolerance").unwrap_or(1e-7);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let report = remus_operations::heal::repair_solid(self.topo_mut(), solid_id, tol)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "solid": solid_id_to_u32(solid_id),
                    "errorsBefore": report.before.error_count(),
                    "errorsAfter": report.after.error_count(),
                    "totalRepairs": report.total_repairs(),
                }))
            }
            "repairSolidDetailed" => {
                let solid = get_u32(args, "solid")?;
                let result = self
                    .repair_solid_detailed_impl(solid)
                    .map_err(StructuredWasmError::from)?;
                serde_json::to_value(result).map_err(StructuredWasmError::from)
            }
            "classifyPoint" => {
                let s = get_u32(args, "solid")?;
                let x = get_f64(args, "x")?;
                let y = get_f64(args, "y")?;
                let z = get_f64(args, "z")?;
                let tol = get_f64(args, "tolerance").unwrap_or(1e-7);
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let pt = Point3::new(x, y, z);
                let result =
                    remus_operations::classify::classify_point(&self.topo, solid_id, pt, 0.1, tol)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(classify_to_string(result)))
            }
            "loft" => {
                let face_handles: Vec<u32> = get_u32_array_optional(args, "faces")?;
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = remus_operations::loft::loft(self.topo_mut(), &face_ids)
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
                let face_handles: Vec<u32> = get_u32_array_optional(args, "faces")?;
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result = remus_operations::loft::loft_smooth(self.topo_mut(), &face_ids)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "circularPattern" => {
                let s = get_u32(args, "solid")?;
                let ax = get_f64(args, "ax").unwrap_or(0.0);
                let ay = get_f64(args, "ay").unwrap_or(0.0);
                let az = get_f64(args, "az").unwrap_or(1.0);
                let count = get_u32(args, "count")?;
                let count =
                    validate_work_count(count, "count").map_err(StructuredWasmError::from)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let axis = Vec3::new(ax, ay, az);
                let compound = remus_operations::pattern::circular_pattern(
                    self.topo_mut(),
                    solid_id,
                    axis,
                    count,
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
                validate_work_count(cx, "countX").map_err(StructuredWasmError::from)?;
                validate_work_count(cy, "countY").map_err(StructuredWasmError::from)?;
                validate_work_product(cx, cy, "grid copies").map_err(StructuredWasmError::from)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let compound = remus_operations::pattern::grid_pattern(
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
            "getSolidFaces" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let faces = remus_topology::explorer::solid_faces(self.topo(), solid_id)
                    .map_err(crate::error::WasmError::from)
                    .map_err(StructuredWasmError::from)?;
                let handles: Vec<u32> = faces.iter().map(|f| face_id_to_u32(*f)).collect();
                Ok(serde_json::json!(handles))
            }
            "getOpposingPlanarFacePairs" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_count = u32::try_from(
                    remus_topology::explorer::solid_faces(self.topo(), solid_id)
                        .map_err(crate::error::WasmError::from)
                        .map_err(StructuredWasmError::from)?
                        .len(),
                )
                .unwrap_or(u32::MAX);
                validate_face_pair_count(face_count).map_err(StructuredWasmError::from)?;
                let pairs = opposing_planar_face_pairs(self.topo(), solid_id, Tolerance::default())
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::Value::Array(
                    pairs
                        .iter()
                        .map(|pair| {
                            serde_json::json!({
                                "faceA": face_id_to_u32(pair.face_a),
                                "faceB": face_id_to_u32(pair.face_b),
                                "distance": pair.distance,
                                "overlapArea": pair.overlap_area,
                                "faceAreaA": pair.face_area_a,
                                "faceAreaB": pair.face_area_b,
                                "normal": [
                                    pair.normal.x(),
                                    pair.normal.y(),
                                    pair.normal.z()
                                ],
                                "faceABordersBlend": pair.face_a_borders_blend,
                                "faceBBordersBlend": pair.face_b_borders_blend,
                            })
                        })
                        .collect(),
                ))
            }
            "getBlendRegion" => {
                let s = get_u32(args, "solid")?;
                let f = get_u32(args, "face")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let region =
                    remus_operations::resize_blend::blend_region(self.topo(), solid_id, face_id)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "faces": region
                        .faces
                        .into_iter()
                        .map(face_id_to_u32)
                        .collect::<Vec<_>>(),
                    "radius": region.radius,
                }))
            }
            "getFaceNormal" => {
                // The face's stored surface normal at its outer-wire start (a
                // plane's constant normal; curved surfaces report the normal
                // at parameter-space origin). Enough to pick a wall by
                // direction from a batch script.
                let f = get_u32(args, "face")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let face = self
                    .topo()
                    .face(face_id)
                    .map_err(crate::error::WasmError::from)
                    .map_err(StructuredWasmError::from)?;
                let normal = match face.surface() {
                    remus_topology::face::FaceSurface::Plane { normal, .. } => *normal,
                    other => other.normal(0.0, 0.0),
                };
                Ok(serde_json::json!([normal.x(), normal.y(), normal.z()]))
            }
            "getFaceCurvature" => {
                // Principal curvatures at (u, v) on a face's surface, sorted
                // k1 >= k2, signed positive for convex-outward relative to
                // the face's effective outward normal. d1/d2 are null at
                // umbilic points (sphere, plane) rather than fabricated.
                let f = get_u32(args, "face")?;
                let u = get_f64(args, "u")?;
                let v = get_f64(args, "v")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let report =
                    remus_check::analyze::curvature::surface_curvature(self.topo(), face_id, u, v)
                        .map_err(StructuredWasmError::from)?;
                let (d1, d2) = match report.directions {
                    Some((d1, d2)) => (
                        serde_json::json!([d1.x(), d1.y(), d1.z()]),
                        serde_json::json!([d2.x(), d2.y(), d2.z()]),
                    ),
                    None => (serde_json::Value::Null, serde_json::Value::Null),
                };
                Ok(serde_json::json!({
                    "k1": report.k1,
                    "k2": report.k2,
                    "gaussian": report.gaussian,
                    "mean": report.mean,
                    "d1": d1,
                    "d2": d2,
                }))
            }
            "getFaceMinRadius" => {
                // 1 / max(|k1|, |k2|) over the face's trimmed domain.
                // Orientation-independent; exact on analytic surfaces,
                // approximate on NURBS. JSON has no Infinity, so a plane's
                // infinite radius is reported as `minRadius: null` with the
                // explicit `isInfinite` flag.
                let f = get_u32(args, "face")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let min_radius =
                    remus_check::analyze::curvature::min_radius_of_curvature(self.topo(), face_id)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!({
                    "minRadius": min_radius,
                    "isInfinite": !min_radius.is_finite(),
                }))
            }
            "getFaceVertexPositions" => {
                // Flat [x, y, z, ...] positions of every vertex the face's
                // wires reference (outer first, then holes), in wire order.
                // Enough to select faces by region from a batch script.
                let f = get_u32(args, "face")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let mut coords: Vec<f64> = Vec::new();
                let (outer, inners) = {
                    let face = self
                        .topo()
                        .face(face_id)
                        .map_err(crate::error::WasmError::from)
                        .map_err(StructuredWasmError::from)?;
                    (face.outer_wire(), face.inner_wires().to_vec())
                };
                for wid in std::iter::once(outer).chain(inners) {
                    let wire = self
                        .topo()
                        .wire(wid)
                        .map_err(crate::error::WasmError::from)
                        .map_err(StructuredWasmError::from)?;
                    for oe in wire.edges() {
                        let edge = self
                            .topo()
                            .edge(oe.edge())
                            .map_err(crate::error::WasmError::from)
                            .map_err(StructuredWasmError::from)?;
                        let vid = oe.oriented_start(edge);
                        let p = self
                            .topo()
                            .vertex(vid)
                            .map_err(crate::error::WasmError::from)
                            .map_err(StructuredWasmError::from)?
                            .point();
                        coords.extend([p.x(), p.y(), p.z()]);
                    }
                }
                Ok(serde_json::json!(coords))
            }
            "defeature" => {
                let s = get_u32(args, "solid")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_handles: Vec<u32> = get_u32_array_optional(args, "faces")?;
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let result =
                    remus_operations::defeature::defeature(self.topo_mut(), solid_id, &face_ids)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "copyWire" => {
                let w = get_u32(args, "wire")?;
                let wire_id = self.resolve_wire(w).map_err(StructuredWasmError::from)?;
                let copy = remus_operations::copy::copy_wire(self.topo_mut(), wire_id)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(wire_id_to_u32(copy)))
            }
            "copyFace" => {
                let f = get_u32(args, "face")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let copy = remus_operations::copy::copy_face(self.topo_mut(), face_id)
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
                remus_operations::transform::transform_wire(self.topo_mut(), wire_id, &mat)
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
                remus_operations::transform::transform_face(self.topo_mut(), face_id, &mat)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(null))
            }
            "offsetFace" => {
                let f = get_u32(args, "face")?;
                let dist = get_f64(args, "distance")?;
                let samples = get_u32(args, "samples").unwrap_or(16);
                validate_work_product(samples, samples, "sample grid")
                    .map_err(StructuredWasmError::from)?;
                let samples =
                    validate_work_count(samples, "samples").map_err(StructuredWasmError::from)?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let result = remus_operations::offset_face::offset_face(
                    self.topo_mut(),
                    face_id,
                    dist,
                    samples,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(face_id_to_u32(result)))
            }
            "offsetSolid" => {
                let s = get_u32(args, "solid")?;
                let dist = get_f64(args, "distance")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let result =
                    remus_operations::offset_v2::offset_solid_v2(self.topo_mut(), solid_id, dist)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(result)))
            }
            "offsetSolidV2" => {
                let s = get_u32(args, "solid")?;
                let dist = get_f64(args, "distance")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let result =
                    remus_operations::offset_v2::offset_solid_v2(self.topo_mut(), solid_id, dist)
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
                let result = remus_operations::section::section(
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
                let result = remus_operations::split::split(
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
            "splitBySheet" => {
                let solid = get_u32(args, "solid")?;
                let sheet = get_u32(args, "sheet")?;
                let solid_id = self
                    .resolve_solid(solid)
                    .map_err(StructuredWasmError::from)?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let result =
                    remus_operations::split::split_by_sheet(self.topo_mut(), solid_id, sheet_id)
                        .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(compound_id_to_u32(result)))
            }
            "trimSheetBySolid" => {
                let sheet = get_u32(args, "sheet")?;
                let solid = get_u32(args, "solid")?;
                let keep_inside = get_bool(args, "keepInside")?;
                let sheet_id = self
                    .resolve_shell(sheet)
                    .map_err(StructuredWasmError::from)?;
                let solid_id = self
                    .resolve_solid(solid)
                    .map_err(StructuredWasmError::from)?;
                let mode = if keep_inside {
                    remus_operations::boolean::SheetTrimMode::KeepInside
                } else {
                    remus_operations::boolean::SheetTrimMode::KeepOutside
                };
                let result = remus_operations::boolean::trim_sheet_by_solid(
                    self.topo_mut(),
                    sheet_id,
                    solid_id,
                    mode,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(shell_id_to_u32(result)))
            }
            "trimSheetBySheet" => {
                let target = get_u32(args, "target")?;
                let tool = get_u32(args, "tool")?;
                let keep_positive = get_bool(args, "keepPositive")?;
                let target_id = self
                    .resolve_shell(target)
                    .map_err(StructuredWasmError::from)?;
                let tool_id = self
                    .resolve_shell(tool)
                    .map_err(StructuredWasmError::from)?;
                let side = if keep_positive {
                    remus_operations::boolean::SheetSide::Positive
                } else {
                    remus_operations::boolean::SheetSide::Negative
                };
                let result = remus_operations::boolean::trim_sheet_by_sheet(
                    self.topo_mut(),
                    target_id,
                    tool_id,
                    side,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(shell_id_to_u32(result)))
            }
            "mutualTrimSheets" => {
                let sheet_a = get_u32(args, "sheetA")?;
                let sheet_b = get_u32(args, "sheetB")?;
                let keep_a_positive = get_bool(args, "keepAPositive")?;
                let keep_b_positive = get_bool(args, "keepBPositive")?;
                let sheet_a_id = self
                    .resolve_shell(sheet_a)
                    .map_err(StructuredWasmError::from)?;
                let sheet_b_id = self
                    .resolve_shell(sheet_b)
                    .map_err(StructuredWasmError::from)?;
                let side_a = if keep_a_positive {
                    remus_operations::boolean::SheetSide::Positive
                } else {
                    remus_operations::boolean::SheetSide::Negative
                };
                let side_b = if keep_b_positive {
                    remus_operations::boolean::SheetSide::Positive
                } else {
                    remus_operations::boolean::SheetSide::Negative
                };
                let result = remus_operations::boolean::mutual_trim_sheets(
                    self.topo_mut(),
                    sheet_a_id,
                    sheet_b_id,
                    side_a,
                    side_b,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!([
                    shell_id_to_u32(result.sheet_a),
                    shell_id_to_u32(result.sheet_b),
                ]))
            }
            "sewFaces" => {
                let face_handles: Vec<u32> = get_u32_array_optional(args, "faces")?;
                let tol = get_f64(args, "tolerance").unwrap_or(1e-6);
                let face_ids: Vec<_> = face_handles
                    .iter()
                    .map(|&h| self.resolve_face(h).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let solid = remus_operations::sew::sew_faces(self.topo_mut(), &face_ids, tol)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(solid_id_to_u32(solid)))
            }
            "thicken" => {
                let f = get_u32(args, "face")?;
                let thickness = get_f64(args, "thickness")?;
                let face_id = self.resolve_face(f).map_err(StructuredWasmError::from)?;
                let result =
                    remus_operations::thicken::thicken(self.topo_mut(), face_id, thickness)
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
                let solid = remus_operations::pipe::pipe(self.topo_mut(), face_id, &curve, None)
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
                let count =
                    validate_work_count(count, "count").map_err(StructuredWasmError::from)?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let compound = remus_operations::pattern::linear_pattern(
                    self.topo_mut(),
                    solid_id,
                    Vec3::new(dx, dy, dz),
                    spacing,
                    count,
                )
                .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(compound_id_to_u32(compound)))
            }
            "draft" => {
                let s = get_u32(args, "solid")?;
                let angle = get_f64(args, "angle")?;
                let solid_id = self.resolve_solid(s).map_err(StructuredWasmError::from)?;
                let face_handles: Vec<u32> = get_u32_array_optional(args, "faces")?;
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
                let result = remus_operations::draft::draft(
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
                    remus_operations::offset_wire::offset_wire(self.topo_mut(), face_id, dist)
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
                let wire_id = remus_operations::offset_wire::offset_wire_with_join(
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
                    remus_topology::builder::make_planar_face_from_wire(self.topo_mut(), wire_id)
                        .map_err(StructuredWasmError::from)?;
                let result = remus_operations::offset_wire::offset_wire_with_join(
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
                let eid = remus_topology::builder::make_line_edge(
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
                let fid = remus_topology::builder::make_planar_face_from_wire(self.topo_mut(), wid)
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
            "makeSheetBody" => {
                let handles = get_u32_array(args, "faces")?;
                let faces = handles
                    .iter()
                    .map(|&handle| self.resolve_face(handle).map_err(StructuredWasmError::from))
                    .collect::<Result<Vec<_>, _>>()?;
                let sheet = remus_operations::sew::make_sheet_body(self.topo_mut(), &faces)
                    .map_err(StructuredWasmError::from)?;
                Ok(serde_json::json!(shell_id_to_u32(sheet)))
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
                    remus_math::polygon_boolean::BooleanOp::Union
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
                let result = remus_math::polygon2d::fillet_polygon_2d(&polygon, radius);
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
                let result = remus_math::polygon2d::chamfer_polygon_2d(&polygon, distance);
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
            other => self
                .dispatch_naming_op(other, args)
                .or_else(|| self.dispatch_evolution_op(other, args))
                .unwrap_or_else(|| Err(StructuredWasmError::unknown_operation(other))),
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
    fn detailed_healing_batch_discloses_success_and_typed_refusal() {
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(1.0, 1.0, 1.0).expect("box fixture");
        let success = parse(
            &kernel.execute_batch_v2(
                &serde_json::json!([{
                    "op": "repairSolidDetailed",
                    "args": { "solid": solid }
                }])
                .to_string(),
            ),
        );
        assert_eq!(success[0]["ok"]["verified"], true);
        assert_eq!(success[0]["ok"]["errorsAfter"], 0);
        assert_eq!(success[0]["ok"]["repairs"], serde_json::json!([]));

        let invalid = kernel.topo_mut().add_empty_solid();
        let invalid_handle = crate::handles::solid_id_to_u32(invalid);
        let counts_before = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_wires(),
            kernel.topo().num_faces(),
            kernel.topo().num_shells(),
            kernel.topo().num_solids(),
        );
        let refused = parse(
            &kernel.execute_batch_v2(
                &serde_json::json!([{
                    "op": "healSolidDetailed",
                    "args": { "solid": invalid_handle }
                }])
                .to_string(),
            ),
        );
        assert_eq!(
            refused[0]["error"]["details"]["kernelCode"],
            "healing_validation_failed"
        );
        assert_eq!(refused[0]["error"]["category"], "invalid_topology");
        assert_eq!(
            counts_before,
            (
                kernel.topo().num_vertices(),
                kernel.topo().num_edges(),
                kernel.topo().num_wires(),
                kernel.topo().num_faces(),
                kernel.topo().num_shells(),
                kernel.topo().num_solids(),
            )
        );
        assert!(kernel.topo().solid(invalid).is_ok());
    }

    #[cfg(feature = "io")]
    #[test]
    fn step_validation_capability_has_export_and_import_batch_companions() {
        let mut exporting = BrepKernel::new();
        let export = parse(&exporting.execute_batch_v2(
            r#"[
                {"op":"makeBox","args":{"width":2,"height":3,"depth":4}},
                {"op":"exportStep","args":{"solid":0,"options":{"validationProperties":true}}}
            ]"#,
        ));
        let step = export[1]["ok"].as_str().expect("STEP string");
        assert!(step.contains("geometric validation property"));

        let input = serde_json::to_string(&serde_json::json!([{
            "op": "importStepWithValidation",
            "args": { "data": step }
        }]))
        .expect("batch JSON");
        let mut importing = BrepKernel::new();
        let imported = parse(&importing.execute_batch_v2(&input));
        assert_eq!(
            imported[0]["ok"]["solids"]
                .as_array()
                .expect("solid handles")
                .len(),
            1
        );
        assert!(
            imported[0]["ok"]["diagnostics"]
                .as_array()
                .expect("import diagnostics")
                .is_empty()
        );
        assert!(
            (imported[0]["ok"]["validation"][0]["declared"]["volume"]
                .as_f64()
                .expect("declared volume")
                - 24.0)
                .abs()
                < 1e-9
        );
        assert!(
            imported[0]["ok"]["validation"][0]["diagnostics"]
                .as_array()
                .expect("diagnostics array")
                .is_empty()
        );
    }

    #[cfg(feature = "io")]
    #[test]
    fn sheet_step_batch_companions_preserve_handle_class_and_refuse_solid_shells() {
        let mut exporting = BrepKernel::new();
        let face =
            remus_topology::builder::make_rectangle_face(exporting.topo_mut(), 2.0, 1.0, TOL)
                .expect("trimmed face");
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(-1.0, -0.5, 0.0), Point3::new(1.0, -0.5, 0.0)],
                vec![Point3::new(-1.0, 0.5, 0.0), Point3::new(1.0, 0.5, 0.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .expect("bilinear NURBS");
        exporting
            .topo_mut()
            .face_mut(face)
            .expect("face")
            .set_surface(remus_topology::face::FaceSurface::Nurbs(surface));
        let sheet = remus_operations::sew::make_sheet_body(exporting.topo_mut(), &[face])
            .expect("sheet body");
        let exported = parse(&exporting.execute_batch_v2(&format!(
            r#"[{{"op":"exportStepSheet","args":{{"sheet":{}}}}}]"#,
            shell_id_to_u32(sheet)
        )));
        let step = exported[0]["ok"].as_str().expect("sheet STEP string");
        assert!(step.contains("SHELL_BASED_SURFACE_MODEL("));
        assert!(step.contains("OPEN_SHELL("));
        assert!(step.contains("B_SPLINE_SURFACE_WITH_KNOTS("));

        let mut wrong_class = BrepKernel::new();
        wrong_class
            .make_box_solid(2.0, 1.0, 1.0)
            .expect("solid fixture");
        let rejected = parse(
            &wrong_class.execute_batch_v2(r#"[{"op":"exportStepSheet","args":{"sheet":0}}]"#),
        );
        assert_eq!(
            rejected[0]["error"]["details"]["kernelCode"],
            "body_class_unresolved"
        );
        let message = rejected[0]["error"]["message"]
            .as_str()
            .expect("typed error message");
        assert!(
            message.contains("body class solid, expected sheet"),
            "{message}"
        );

        let input = serde_json::to_string(&serde_json::json!([{
            "op": "importStepBodies",
            "args": { "data": step }
        }]))
        .expect("batch JSON");
        let mut importing = BrepKernel::new();
        let imported = parse(&importing.execute_batch_v2(&input));
        assert!(
            imported[0]["ok"]["solids"]
                .as_array()
                .expect("solid handle array")
                .is_empty()
        );
        assert_eq!(
            imported[0]["ok"]["sheets"]
                .as_array()
                .expect("sheet handle array")
                .len(),
            1
        );
        assert!(
            imported[0]["ok"]["diagnostics"]
                .as_array()
                .expect("diagnostic array")
                .is_empty()
        );
    }

    #[test]
    fn plane_face_bounds_refuse_missing_curved_authority_without_mutation() {
        use remus_topology::edge::EdgeCurve;
        use remus_topology::explorer::solid_faces;
        use remus_topology::face::FaceSurface;

        let mut kernel = BrepKernel::new();
        let solid = remus_operations::primitives::make_cylinder(kernel.topo_mut(), 2.0, 3.0)
            .expect("cylinder fixture");
        let cap = solid_faces(kernel.topo(), solid)
            .expect("cylinder faces")
            .into_iter()
            .find(|&face_id| {
                kernel
                    .topo()
                    .face(face_id)
                    .is_ok_and(|face| matches!(face.surface(), FaceSurface::Plane { .. }))
            })
            .expect("planar cylinder cap");
        let rim = {
            let face = kernel.topo().face(cap).expect("cap");
            let wire = kernel.topo().wire(face.outer_wire()).expect("cap wire");
            wire.edges()
                .iter()
                .map(remus_topology::wire::OrientedEdge::edge)
                .find(|&edge_id| {
                    kernel
                        .topo()
                        .edge(edge_id)
                        .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::Circle(_)))
                })
                .expect("cap rim")
        };
        kernel.topo_mut().edge_mut(rim).expect("rim").set_trim(None);
        let before = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_wires(),
            kernel.topo().num_faces(),
        );
        let face = kernel.topo().face(cap).expect("cap");
        let FaceSurface::Plane { normal, d } = face.surface() else {
            unreachable!("selected face must remain planar");
        };

        let error = kernel
            .plane_face_uv_bounds(cap, *normal, *d)
            .expect_err("missing rim authority must refuse");
        assert!(error.to_string().contains("require edge authority"));
        assert_eq!(
            before,
            (
                kernel.topo().num_vertices(),
                kernel.topo().num_edges(),
                kernel.topo().num_wires(),
                kernel.topo().num_faces(),
            )
        );
    }

    /// Handle arrays used to be parsed with
    /// `filter_map(|v| v.as_u64().map(|n| n as u32))`, which fails open twice:
    /// `filter_map` DROPS an element it cannot read, and `as u32` truncates.
    /// Both produced a wrong answer reported as success, which a caller cannot
    /// notice. Measured before the fix: `edges: [0, "not-a-handle", 1]` filleted
    /// two of the three edges and returned `{"ok":1}`, and the handle
    /// `4294967296` wrapped to `0` and filleted edge 0, also `{"ok":1}`.
    #[test]
    fn batch_handle_arrays_reject_malformed_and_out_of_range_elements() {
        const BOX: &str = r#"{"op":"makeBox","args":{"width":10,"height":10,"depth":10}}"#;

        // A well-formed selection still works, so the guard is not just refusing.
        let mut kernel = BrepKernel::new();
        let ok = parse(&kernel.execute_batch(&format!(
            r#"[{BOX},{{"op":"fillet","args":{{"solid":0,"edges":[0],"radius":0.5}}}}]"#
        )));
        assert_eq!(ok[1]["ok"], 1, "a valid edge selection must still fillet");

        // An element that is not a handle names its own index instead of
        // vanishing from the selection.
        let mut kernel = BrepKernel::new();
        let dropped = parse(&kernel.execute_batch(&format!(
            r#"[{BOX},{{"op":"fillet","args":{{"solid":0,"edges":[0,"not-a-handle",1],"radius":0.5}}}}]"#
        )));
        assert_eq!(
            dropped[1]["error"], "edges[1] is not a u32",
            "a malformed element must be reported, not silently dropped: {dropped}"
        );

        // 2^32 truncates to 0 under `as u32`, which is a DIFFERENT live entity.
        let mut kernel = BrepKernel::new();
        let wrapped = parse(&kernel.execute_batch(&format!(
            r#"[{BOX},{{"op":"fillet","args":{{"solid":0,"edges":[4294967296],"radius":0.5}}}}]"#
        )));
        assert_eq!(
            wrapped[1]["error"], "edges[0] is not a u32",
            "a handle above u32::MAX must be rejected, not wrapped: {wrapped}"
        );

        // chamfer shares the parser.
        let mut kernel = BrepKernel::new();
        let chamfer = parse(&kernel.execute_batch(&format!(
            r#"[{BOX},{{"op":"chamfer","args":{{"solid":0,"edges":[0,null],"distance":0.5}}}}]"#
        )));
        assert_eq!(chamfer[1]["error"], "edges[1] is not a u32");

        // An ABSENT optional key keeps its old meaning: an empty selection.
        let mut kernel = BrepKernel::new();
        let absent = parse(&kernel.execute_batch(&format!(
            r#"[{BOX},{{"op":"shell","args":{{"solid":0,"thickness":1.0}}}}]"#
        )));
        assert_eq!(
            absent[1]["ok"], 1,
            "an absent optional handle array must still mean 'none': {absent}"
        );
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
    fn v2_errors_carry_the_failure_category() {
        // Category is part of the public v2 contract: pinned per code.
        let mut kernel = BrepKernel::new();
        let cases = [
            (r"not json", "invalid_json", "invalid_input"),
            (r"[{}]", "missing_operation", "invalid_input"),
            (
                r#"[{"op":"notAnOperation","args":{}}]"#,
                "unknown_operation",
                "invalid_input",
            ),
            (
                r#"[{"op":"makeBox","args":{"height":2,"depth":3}}]"#,
                "invalid_argument",
                "invalid_input",
            ),
            (
                r#"[{"op":"volume","args":{"solid":42,"deflection":0.1}}]"#,
                "invalid_handle",
                "invalid_input",
            ),
        ];
        for (input, code, category) in cases {
            let error = v2_error(&mut kernel, input);
            assert_eq!(error["code"], code, "{input}");
            assert_eq!(error["category"], category, "{input}");
        }
    }

    #[test]
    fn v2_math_failures_surface_the_kernel_registry_code() {
        // A typed MathError carries its fine-grained native registry code in
        // details.kernelCode, additively to the coarse wire code.
        let mut kernel = BrepKernel::new();
        let error = v2_error(
            &mut kernel,
            r#"[{"op":"makeNurbsEdge","args":{
                "startX":0,"startY":0,"startZ":0,
                "endX":1,"endY":0,"endZ":0,
                "degree":0,
                "knots":[0,1],
                "controlPoints":[0,0,0, 1,0,0],
                "weights":[1,1]
            }}]"#,
        );
        assert_eq!(error["code"], "invalid_argument");
        assert_eq!(error["category"], "invalid_input");
        assert_eq!(error["details"]["kernelCode"], "invalid_nurbs_degree");
    }

    #[test]
    fn legacy_batch_contract_is_unchanged_by_categories() {
        // v1 must stay a bare string error with no structured fields.
        let mut kernel = BrepKernel::new();
        let response = parse(
            &kernel.execute_batch(r#"[{"op":"volume","args":{"solid":42,"deflection":0.1}}]"#),
        );
        assert!(response[0]["error"].is_string());
    }

    #[test]
    fn classifies_operations_before_topology_snapshotting() {
        assert_eq!(batch_op_kind("volume"), Some(BatchOpKind::ReadOnly));
        assert_eq!(batch_op_kind("projectEdges"), Some(BatchOpKind::ReadOnly));
        assert_eq!(
            batch_op_kind("getOpposingPlanarFacePairs"),
            Some(BatchOpKind::ReadOnly)
        );
        assert_eq!(batch_op_kind("makeBox"), Some(BatchOpKind::Mutating));
        assert_eq!(batch_op_kind("moveFaces"), Some(BatchOpKind::Mutating));
        assert_eq!(batch_op_kind("notAnOperation"), None);
    }

    #[test]
    fn planar_pair_query_and_move_faces_round_trip() {
        let mut kernel = BrepKernel::new();
        let solid = kernel
            .dispatch_op(
                "makeBox",
                &serde_json::json!({"width": 2.0, "height": 3.0, "depth": 4.0}),
            )
            .expect("box")
            .as_u64()
            .expect("solid handle") as u32;
        let pairs = kernel
            .dispatch_op(
                "getOpposingPlanarFacePairs",
                &serde_json::json!({"solid": solid}),
            )
            .expect("face pairs");
        let pairs = pairs.as_array().expect("pair array");
        assert_eq!(pairs.len(), 3);
        let width_pair = pairs
            .iter()
            .find(|pair| (pair["distance"].as_f64().unwrap_or_default() - 2.0).abs() < 1.0e-9)
            .expect("width pair");
        assert!((width_pair["overlapArea"].as_f64().unwrap_or_default() - 12.0).abs() < 1.0e-9);

        let face = width_pair["faceA"].as_u64().expect("face handle") as u32;
        let moved = kernel
            .dispatch_op(
                "moveFaces",
                &serde_json::json!({"solid": solid, "faces": [face], "distance": 1.0}),
            )
            .expect("move face")
            .as_u64()
            .expect("moved solid handle") as u32;
        let volume = kernel
            .dispatch_op(
                "volume",
                &serde_json::json!({"solid": moved, "deflection": 0.1}),
            )
            .expect("volume")
            .as_f64()
            .expect("numeric volume");
        assert!((volume - 36.0).abs() < 1.0e-6);
    }

    /// Classification must agree with what `dispatch_naming_op` can actually
    /// run. The serialized-reference arms there are `io`-gated, so without the
    /// feature these ops must be rejected outright rather than admitted as
    /// ReadOnly and failed after the batch has taken its rollback share.
    #[test]
    fn serialized_reference_ops_are_classified_only_with_io() {
        const REF_OPS: [&str; 5] = [
            "makeOperationOutputRef",
            "captureSignatureRef",
            "addRefDiscriminator",
            "resolveRef",
            "resolveRefFaceAttributes",
        ];
        for op in REF_OPS {
            let kind = batch_op_kind(op);
            if cfg!(feature = "io") {
                assert_eq!(kind, Some(BatchOpKind::ReadOnly), "{op} with io");
            } else {
                assert_eq!(kind, None, "{op} without io");
            }
        }
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
    fn batch_boolean_with_quality_accepts_ssi_budgets() {
        // Default omitted vs explicit budgets (the historical values, and 0
        // which disables the governed work): analytic operands never enter
        // NURBS SSI, so every case must produce the identical
        // exact result — the cap is additive, never a behavior change for
        // exact-analytic paths. The JS-value-to-context link is pinned by
        // `bindings::booleans` unit tests; math and FF regressions pin the
        // context's authority below that.
        for extra in [
            "",
            r#","newtonIterations":20,"subdivisionDepth":6,"marchSteps":200,"queueSize":100,"segments":50,"branchesPerDirection":10"#,
            r#","newtonIterations":0,"subdivisionDepth":0,"marchSteps":0,"queueSize":0,"segments":0,"branchesPerDirection":0"#,
        ] {
            let mut kernel = BrepKernel::new();
            let script = format!(
                r#"[
                    {{"op":"makeBox","args":{{"width":2,"height":2,"depth":2}}}},
                    {{"op":"makeBox","args":{{"width":1,"height":1,"depth":1}}}},
                    {{"op":"booleanWithQuality","args":{{"operation":"fuse","solidA":0,"solidB":1,"exactOnly":true{extra}}}}},
                    {{"op":"volume","args":{{"solid":2,"deflection":0.05}}}}
                ]"#
            );
            let response = parse(&kernel.execute_batch_v2(&script));
            assert_eq!(
                response[2]["ok"]["quality"], "exact",
                "budget '{extra}' must not disturb the exact analytic path: {response}"
            );
            let volume = response[3]["ok"].as_f64().expect("numeric volume");
            assert!(
                (volume - 8.0).abs() < 1e-6,
                "budget '{extra}' fused volume {volume}"
            );
        }
    }

    #[test]
    fn batch_general_position_sphere_fuse_is_exact() {
        let mut kernel = BrepKernel::new();
        let response = parse(&kernel.execute_batch_v2(
            r#"[
                {"op":"makeSphere","args":{"radius":1,"segments":16}},
                {"op":"makeSphere","args":{"radius":1,"segments":16}},
                {"op":"transform","args":{"solid":1,"matrix":[1,0,0,0.6666666666666666,0,1,0,0.6666666666666666,0,0,1,0.3333333333333333,0,0,0,1]}},
                {"op":"booleanWithQuality","args":{"operation":"fuse","solidA":0,"solidB":1,"exactOnly":true}},
                {"op":"volume","args":{"solid":2,"deflection":0.03}},
                {"op":"validateSolid","args":{"solid":2}}
            ]"#,
        ));

        assert_eq!(response[3]["ok"]["quality"], "exact", "{response}");
        assert_eq!(response[3]["ok"]["solid"], 2, "{response}");
        let volume = response[4]["ok"].as_f64().expect("numeric volume");
        let expected = 9.0 * std::f64::consts::PI / 4.0;
        assert!(
            (volume - expected).abs() / expected < 0.01,
            "sphere union volume {volume} against {expected}"
        );
        assert_eq!(response[5]["ok"], 0, "{response}");
    }

    #[test]
    fn batch_torus_box_intersect_is_exact() {
        let mut kernel = BrepKernel::new();
        let response = parse(&kernel.execute_batch_v2(
            r#"[
                {"op":"makeTorus","args":{"majorRadius":10,"minorRadius":3,"segments":32}},
                {"op":"makeBox","args":{"width":8,"height":8,"depth":8}},
                {"op":"transform","args":{"solid":1,"matrix":[1,0,0,6,0,1,0,-4,0,0,1,-4,0,0,0,1]}},
                {"op":"booleanWithQuality","args":{"operation":"intersect","solidA":0,"solidB":1,"exactOnly":true}},
                {"op":"volume","args":{"solid":2,"deflection":0.01}},
                {"op":"validateSolid","args":{"solid":2}},
                {"op":"getSolidFaces","args":{"solid":2}}
            ]"#,
        ));

        assert_eq!(response[3]["ok"]["quality"], "exact", "{response}");
        assert_eq!(response[3]["ok"]["solid"], 2, "{response}");
        let volume = response[4]["ok"].as_f64().expect("numeric volume");
        assert!(
            (volume - 232.45).abs() / 232.45 < 0.01,
            "torus-box intersection volume {volume}"
        );
        assert_eq!(response[5]["ok"], 0, "{response}");
        assert_eq!(
            response[6]["ok"].as_array().map(Vec::len),
            Some(5),
            "{response}"
        );
    }

    #[test]
    fn batch_centered_box_sphere_fuse_is_exact() {
        let mut kernel = BrepKernel::new();
        let response = parse(&kernel.execute_batch_v2(
            r#"[
                {"op":"makeBox","args":{"width":10,"height":10,"depth":10}},
                {"op":"makeSphere","args":{"radius":6,"segments":24}},
                {"op":"transform","args":{"solid":1,"matrix":[1,0,0,5,0,1,0,5,0,0,1,5,0,0,0,1]}},
                {"op":"booleanWithQuality","args":{"operation":"fuse","solidA":0,"solidB":1,"exactOnly":true}},
                {"op":"volume","args":{"solid":2,"deflection":0.01}},
                {"op":"validateSolid","args":{"solid":2}},
                {"op":"getSolidFaces","args":{"solid":2}}
            ]"#,
        ));

        assert_eq!(response[3]["ok"]["quality"], "exact", "{response}");
        assert_eq!(response[3]["ok"]["solid"], 2, "{response}");
        let volume = response[4]["ok"].as_f64().expect("numeric volume");
        assert!(
            (volume - 1106.75).abs() < 0.25,
            "centered box-sphere union volume {volume}"
        );
        assert_eq!(response[5]["ok"], 0, "{response}");
        assert_eq!(
            response[6]["ok"].as_array().map(Vec::len),
            Some(16),
            "{response}"
        );
    }

    #[test]
    fn batch_perpendicular_equal_radius_cylinder_intersect_is_exact() {
        let mut kernel = BrepKernel::new();
        let response = parse(&kernel.execute_batch_v2(
            r#"[
                {"op":"makeCylinder","args":{"radius":3,"height":20}},
                {"op":"transform","args":{"solid":0,"matrix":[1,0,0,0,0,1,0,0,0,0,1,-10,0,0,0,1]}},
                {"op":"makeCylinder","args":{"radius":3,"height":20}},
                {"op":"transform","args":{"solid":1,"matrix":[0,0,1,-10,0,1,0,0,-1,0,0,0,0,0,0,1]}},
                {"op":"booleanWithQuality","args":{"operation":"intersect","solidA":0,"solidB":1,"exactOnly":true}},
                {"op":"getSolidFaces","args":{"solid":2}},
                {"op":"solidEdges","args":{"solid":2}},
                {"op":"volume","args":{"solid":2,"deflection":0.001}},
                {"op":"validateSolid","args":{"solid":2}},
                {"op":"meshQuality","args":{"solid":2,"deflection":0.01}}
            ]"#,
        ));

        assert_eq!(response[4]["ok"]["quality"], "exact", "{response}");
        assert_eq!(response[4]["ok"]["solid"], 2, "{response}");
        assert_eq!(
            response[5]["ok"].as_array().map(Vec::len),
            Some(6),
            "{response}"
        );
        assert_eq!(
            response[6]["ok"].as_array().map(Vec::len),
            Some(10),
            "{response}"
        );
        let volume = response[7]["ok"].as_f64().expect("numeric volume");
        assert!((volume - 144.0).abs() / 144.0 < 1.0e-4, "{response}");
        assert_eq!(response[8]["ok"], 0, "{response}");
        assert_eq!(response[9]["ok"]["boundaryEdges"], 0, "{response}");
        assert_eq!(response[9]["ok"]["nonManifoldEdges"], 0, "{response}");
        assert_eq!(response[9]["ok"]["isWatertight"], true, "{response}");
    }

    #[test]
    fn batch_boolean_with_quality_rejects_invalid_newton_iterations() {
        for bad in ["-1", "2.5", r#""twenty""#, "10001", "true"] {
            let mut kernel = BrepKernel::new();
            let script = format!(
                r#"[
                    {{"op":"makeBox","args":{{"width":2,"height":2,"depth":2}}}},
                    {{"op":"makeBox","args":{{"width":1,"height":1,"depth":1}}}},
                    {{"op":"booleanWithQuality","args":{{"operation":"fuse","solidA":0,"solidB":1,"newtonIterations":{bad}}}}}
                ]"#
            );
            let response = parse(&kernel.execute_batch_v2(&script));
            assert_eq!(
                response[2]["error"]["code"], "invalid_argument",
                "newtonIterations={bad} must be a typed argument error: {response}"
            );
            assert_eq!(
                response[2]["error"]["details"]["argument"], "newtonIterations",
                "error must name the argument: {response}"
            );
        }
    }

    #[test]
    fn batch_boolean_with_quality_rejects_invalid_subdivision_depth() {
        for bad in ["-1", "2.5", r#""deep""#, "10001", "true"] {
            let mut kernel = BrepKernel::new();
            let script = format!(
                r#"[
                    {{"op":"makeBox","args":{{"width":2,"height":2,"depth":2}}}},
                    {{"op":"makeBox","args":{{"width":1,"height":1,"depth":1}}}},
                    {{"op":"booleanWithQuality","args":{{"operation":"fuse","solidA":0,"solidB":1,"subdivisionDepth":{bad}}}}}
                ]"#
            );
            let response = parse(&kernel.execute_batch_v2(&script));
            assert_eq!(
                response[2]["error"]["code"], "invalid_argument",
                "subdivisionDepth={bad} must be a typed argument error: {response}"
            );
            assert_eq!(
                response[2]["error"]["details"]["argument"], "subdivisionDepth",
                "error must name the argument: {response}"
            );
        }
    }

    #[test]
    fn batch_boolean_with_quality_rejects_invalid_marcher_budgets_without_mutation() {
        for field in [
            "marchSteps",
            "queueSize",
            "segments",
            "branchesPerDirection",
        ] {
            for bad in ["-1", "2.5", r#""many""#, "10001", "true"] {
                let mut kernel = BrepKernel::new();
                let script = format!(
                    r#"[
                        {{"op":"makeBox","args":{{"width":2,"height":2,"depth":2}}}},
                        {{"op":"makeBox","args":{{"width":1,"height":1,"depth":1}}}},
                        {{"op":"booleanWithQuality","args":{{"operation":"fuse","solidA":0,"solidB":1,"{field}":{bad}}}}},
                        {{"op":"volume","args":{{"solid":0,"deflection":0.05}}}}
                    ]"#
                );
                let response = parse(&kernel.execute_batch_v2(&script));
                assert_eq!(
                    response[2]["error"]["code"], "invalid_argument",
                    "{field}={bad} must be a typed argument error: {response}"
                );
                assert_eq!(
                    response[2]["error"]["details"]["argument"], field,
                    "error must name the argument: {response}"
                );
                assert_eq!(
                    response[3]["ok"], 8.0,
                    "rejected {field} must preserve the input topology: {response}"
                );
            }
        }
    }

    #[test]
    fn tangent_boss_batch_contract_refuses_exact_or_discloses_approximation() {
        let mut kernel = BrepKernel::new();
        let response = parse(&kernel.execute_batch_v2(
            r#"[
                {"op":"makeBox","args":{"width":60,"height":40,"depth":8}},
                {"op":"makeCylinder","args":{"radius":10,"height":16}},
                {"op":"transform","args":{"solid":1,"matrix":[1,0,0,9.999,0,1,0,20,0,0,1,0,0,0,0,1]}},
                {"op":"booleanWithQuality","args":{"operation":"fuse","solidA":0,"solidB":1,"exactOnly":true}},
                {"op":"volume","args":{"solid":0,"deflection":0.01}},
                {"op":"booleanWithQuality","args":{"operation":"fuse","solidA":0,"solidB":1}},
                {"op":"volume","args":{"solid":5,"deflection":0.01}},
                {"op":"validateSolid","args":{"solid":5}}
            ]"#,
        ));

        assert_eq!(response[3]["error"]["code"], "operation_failed");
        assert_eq!(response[3]["error"]["category"], "quality_refused");
        assert_eq!(
            response[3]["error"]["details"]["kernelCode"],
            "exact_only_unattainable"
        );
        assert_eq!(
            response[3]["error"]["details"]["operation"],
            "booleanWithQuality"
        );
        assert_eq!(response[4]["ok"], 19_200.0, "refusal must roll back");
        assert_eq!(
            response[5]["ok"]["solid"], 5,
            "rolled-back handles must not alias the later result"
        );
        assert_eq!(response[5]["ok"]["quality"], "approximate");
        assert_eq!(response[5]["ok"]["deflection"], 0.1);
        let volume = response[6]["ok"].as_f64().expect("numeric volume");
        let radius = 10.0_f64;
        let d = -0.001_f64;
        let wall_from_axis = radius + d;
        let outside_segment = radius.powi(2) * (wall_from_axis / radius).acos()
            - wall_from_axis * (radius.powi(2) - wall_from_axis.powi(2)).sqrt();
        let shared_area = std::f64::consts::PI * radius.powi(2) - outside_segment;
        let expected = 19_200.0 + std::f64::consts::PI * radius.powi(2) * 16.0 - shared_area * 8.0;
        assert!(
            (volume - expected).abs() / expected < 4e-3,
            "tangent-boss fallback volume {volume} against {expected}"
        );
        assert_eq!(response[7]["ok"], 0);
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
            StructuredWasmError::from(remus_topology::TopologyError::WireNotClosed)
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
            remus_operations::OperationsError::InvalidInput {
                reason: "bad argument".to_string(),
            },
            remus_operations::OperationsError::Check(
                remus_check::CheckError::ClassificationFailed("ambiguous".to_string()),
            ),
            remus_operations::OperationsError::NonManifoldResult,
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

    #[test]
    fn sheet_body_batch_contract_matches_direct_semantics() {
        let mut kernel = BrepKernel::new();
        let response = parse(&kernel.execute_batch_v2(
            r#"[
                {"op":"makeBox","args":{"width":4.0,"height":2.0,"depth":1.0}},
                {"op":"makeSheetBody","args":{"faces":[0]}},
                {"op":"sheetArea","args":{"sheet":1,"deflection":0.05}},
                {"op":"sheetBoundingBox","args":{"sheet":1}},
                {"op":"sheetCenterOfArea","args":{"sheet":1}},
                {"op":"validateSheetBody","args":{"sheet":1}},
                {"op":"tessellateSheet","args":{"sheet":1,"deflection":0.05}},
                {"op":"sheetVolume","args":{"sheet":1,"deflection":0.05}},
                {"op":"sheetArea","args":{"sheet":0,"deflection":0.05}},
                {"op":"sheetBoundingBox","args":{"sheet":0}},
                {"op":"sheetCenterOfArea","args":{"sheet":0}}
            ]"#,
        ));

        assert_eq!(response[0]["ok"], 0);
        assert_eq!(response[1]["ok"], 1);
        let area = response[2]["ok"].as_f64().expect("sheet area result");
        assert!((area - 8.0).abs() < 1e-10, "area={area}");
        assert_eq!(
            response[3]["ok"],
            serde_json::json!([0.0, 0.0, 0.0, 4.0, 2.0, 0.0])
        );
        let center = response[4]["ok"].as_array().expect("sheet center result");
        for (actual, expected) in center.iter().zip([2.0, 1.0, 0.0]) {
            assert!((actual.as_f64().expect("sheet center component") - expected).abs() < 1e-12);
        }
        assert_eq!(response[5]["ok"]["errorCount"], 0);
        assert!(
            response[5]["ok"]["warningCount"]
                .as_u64()
                .expect("sheet warning count")
                > 0
        );
        assert!(
            !response[6]["ok"]["indices"]
                .as_array()
                .expect("sheet mesh indices")
                .is_empty()
        );
        assert_eq!(response[7]["error"]["category"], "invalid_input");
        assert_eq!(
            response[7]["error"]["details"]["kernelCode"],
            "body_class_measure_mismatch"
        );
        assert_eq!(
            response[8]["error"]["details"]["kernelCode"],
            "body_class_measure_mismatch"
        );
        assert_eq!(
            response[9]["error"]["details"]["kernelCode"],
            "body_class_measure_mismatch"
        );
        assert_eq!(
            response[10]["error"]["details"]["kernelCode"],
            "body_class_measure_mismatch"
        );
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
