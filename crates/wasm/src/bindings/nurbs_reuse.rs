//! Read-only, opt-in NURBS carrier reduction and certified cubic fitting.
//!
//! Direct calls return JSON result envelopes. Batch calls use the same option
//! parser, carrier values, and structured diagnostics. No face trims, edge
//! ranges, handles, or topology are replaced. See `docs/design/truck-nurbs-reuse.md`.

use remus_math::context::{FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::nurbs::cubic_fit::{CubicFitOptions, fit_cubic_curve};
use remus_math::nurbs::reduction::{
    simplify_curve, simplify_surface, surface_knot_remove_u, surface_knot_remove_v,
};
use remus_math::nurbs::reuse::{ReductionOptions, ReuseError};
use remus_math::nurbs::{NurbsCurve, NurbsSurface};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use serde_json::{Map, Value, json};
use wasm_bindgen::prelude::*;

use crate::error::StructuredWasmError;
use crate::helpers::{get_f64, get_u32};
use crate::kernel::BrepKernel;

const MAX_OPTIONS_BYTES: usize = 4096;
const MAX_REUSE_WORK: usize = 10_000_000;
const MAX_FIT_DEPTH: usize = 16;
const MAX_FIT_SEGMENTS: usize = 4096;

fn invalid(reason: &'static str) -> StructuredWasmError {
    ReuseError::InvalidOptions { reason }.into()
}

fn options_object<'a>(
    args: &'a Value,
    allowed: &[&str],
) -> Result<Option<&'a Map<String, Value>>, StructuredWasmError> {
    let Some(value) = args.get("options") else {
        return Ok(None);
    };
    let object = value
        .as_object()
        .ok_or_else(|| invalid("options must be an object"))?;
    if object.keys().any(|key| !allowed.contains(&key.as_str())) {
        return Err(invalid("unknown NURBS reuse option"));
    }
    Ok(Some(object))
}

fn work_option(
    object: Option<&Map<String, Value>>,
    key: &str,
    default: usize,
    maximum: usize,
) -> Result<usize, StructuredWasmError> {
    let Some(value) = object.and_then(|options| options.get(key)) else {
        return Ok(default);
    };
    let value = value
        .as_u64()
        .and_then(|v| usize::try_from(v).ok())
        .filter(|v| *v > 0 && *v <= maximum)
        .ok_or_else(|| {
            invalid("work, depth, and segment limits must be positive bounded integers")
        })?;
    Ok(value)
}

fn number_option(
    object: Option<&Map<String, Value>>,
    key: &str,
    default: Option<f64>,
) -> Result<f64, StructuredWasmError> {
    object
        .and_then(|options| options.get(key))
        .map_or(default, Value::as_f64)
        .filter(|value| value.is_finite() && *value > 0.0)
        .ok_or_else(|| invalid("tolerances must be explicit finite positive numbers"))
}

fn reuse_context(
    object: Option<&Map<String, Value>>,
    tolerance: f64,
) -> Result<OperationContext, StructuredWasmError> {
    let exact = object
        .and_then(|options| options.get("exactOnly"))
        .map_or(Ok(false), |v| {
            v.as_bool()
                .ok_or_else(|| invalid("exactOnly must be boolean"))
        })?;
    Ok(OperationContext::new().with_fallback(if exact {
        FallbackPolicy::ExactOnly
    } else {
        FallbackPolicy::AllowApproximate { budget: tolerance }
    }))
}

fn reduction_options(
    args: &Value,
) -> Result<(ReductionOptions, OperationContext), StructuredWasmError> {
    let object = options_object(args, &["tolerance", "maxWork", "exactOnly"])?;
    let tolerance = number_option(object, "tolerance", Some(1e-7))?;
    let options = ReductionOptions {
        tolerance,
        max_work: work_option(object, "maxWork", 1_000_000, MAX_REUSE_WORK)?,
    };
    Ok((options, reuse_context(object, tolerance)?))
}

fn cubic_options(args: &Value) -> Result<(CubicFitOptions, OperationContext), StructuredWasmError> {
    let object = options_object(
        args,
        &[
            "positionTolerance",
            "derivativeTolerance",
            "maxWork",
            "maxDepth",
            "maxSegments",
            "exactOnly",
        ],
    )?;
    let options = CubicFitOptions {
        position_tolerance: number_option(object, "positionTolerance", None)?,
        derivative_tolerance: number_option(object, "derivativeTolerance", None)?,
        max_work: work_option(object, "maxWork", 1_000_000, MAX_REUSE_WORK)?,
        max_depth: work_option(object, "maxDepth", 12, MAX_FIT_DEPTH)?,
        max_segments: work_option(object, "maxSegments", 1024, MAX_FIT_SEGMENTS)?,
    };
    let context = reuse_context(object, options.position_tolerance)?.with_budgets(
        WorkBudgets::new()
            .with_subdivision_depth(options.max_depth)
            .with_segments(options.max_segments),
    );
    Ok((options, context))
}

fn curve_value(curve: &NurbsCurve) -> Value {
    let points: Vec<_> = curve
        .control_points()
        .iter()
        .map(|p| [p.x(), p.y(), p.z()])
        .collect();
    let (start, end) = curve.domain();
    json!({"degree":curve.degree(), "controlPoints":points, "weights":curve.weights(),
        "knots":curve.knots(), "domain":[start,end]})
}

fn surface_value(surface: &NurbsSurface) -> Value {
    let points: Vec<Vec<_>> = surface
        .control_points()
        .iter()
        .map(|row| row.iter().map(|p| [p.x(), p.y(), p.z()]).collect())
        .collect();
    let (u_start, u_end) = surface.domain_u();
    let (v_start, v_end) = surface.domain_v();
    json!({"degreeU":surface.degree_u(), "degreeV":surface.degree_v(),
        "controlPoints":points, "weights":surface.weights(),
        "knotsU":surface.knots_u(), "knotsV":surface.knots_v(),
        "domainU":[u_start,u_end], "domainV":[v_start,v_end]})
}

fn reduction_value(geometry: Value, removed: usize, bound: f64, work: usize) -> Value {
    json!({"geometry":geometry, "geometryOnly":true, "removedKnots":removed,
        "deviationBound":bound, "workUsed":work, "approximate":bound > 0.0,
        "quality":if bound == 0.0 {"unchanged_or_zero_bound"} else {"certified_within_tolerance"}})
}

impl BrepKernel {
    fn reuse_curve(&self, args: &Value) -> Result<&NurbsCurve, StructuredWasmError> {
        let edge = self.resolve_edge(get_u32(args, "edge")?)?;
        match self.topo.edge(edge)?.curve() {
            EdgeCurve::NurbsCurve(curve) => Ok(curve),
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => Err(ReuseError::Unsupported {
                reason: "source edge must store a NURBS carrier",
            }
            .into()),
        }
    }

    fn reuse_surface(&self, args: &Value) -> Result<&NurbsSurface, StructuredWasmError> {
        let face = self.resolve_face(get_u32(args, "face")?)?;
        match self.topo.face(face)?.surface() {
            FaceSurface::Nurbs(surface) => Ok(surface),
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => Err(ReuseError::Unsupported {
                reason: "source face must store a NURBS carrier",
            }
            .into()),
        }
    }

    pub(crate) fn dispatch_nurbs_reuse_op(
        &self,
        operation: &str,
        args: &Value,
    ) -> Option<Result<Value, StructuredWasmError>> {
        match operation {
            "surfaceKnotRemoveU" | "surfaceKnotRemoveV" | "simplifyNurbsSurface" => Some((|| {
                let (options, context) = reduction_options(args)?;
                let surface = self.reuse_surface(args)?;
                let outcome = match operation {
                    "surfaceKnotRemoveU" => {
                        surface_knot_remove_u(surface, get_f64(args, "knot")?, &options, &context)
                    }
                    "surfaceKnotRemoveV" => {
                        surface_knot_remove_v(surface, get_f64(args, "knot")?, &options, &context)
                    }
                    _ => simplify_surface(surface, &options, &context),
                }?;
                Ok(reduction_value(
                    surface_value(&outcome.geometry),
                    outcome.removed_knots,
                    outcome.deviation_bound,
                    outcome.work_used,
                ))
            })(
            )),
            "simplifyNurbsCurve" => Some((|| {
                let (options, context) = reduction_options(args)?;
                let outcome = simplify_curve(self.reuse_curve(args)?, &options, &context)?;
                Ok(reduction_value(
                    curve_value(&outcome.geometry),
                    outcome.removed_knots,
                    outcome.deviation_bound,
                    outcome.work_used,
                ))
            })()),
            "fitCubicCurve" => Some((|| {
                let (options, context) = cubic_options(args)?;
                let outcome = fit_cubic_curve(self.reuse_curve(args)?, &options, &context)?;
                Ok(
                    json!({"geometry":curve_value(&outcome.curve), "geometryOnly":true,
                    "positionBound":outcome.position_bound, "derivativeBound":outcome.derivative_bound,
                    "segments":outcome.segments, "workUsed":outcome.work_used,
                    "approximate":true, "quality":"certified_within_tolerance"}),
                )
            })()),
            _ => None,
        }
    }

    fn reuse_direct(&self, operation: &'static str, mut args: Value, options_json: &str) -> String {
        let result = (|| {
            if options_json.len() > MAX_OPTIONS_BYTES {
                return Err(invalid("options JSON exceeds 4096 bytes"));
            }
            args["options"] = serde_json::from_str(options_json)?;
            self.dispatch_nurbs_reuse_op(operation, &args)
                .ok_or_else(|| StructuredWasmError::unknown_operation(operation))?
        })();
        match result {
            Ok(value) => json!({"status":"ok", "value":value}).to_string(),
            Err(error) => {
                let (code, category, details) =
                    error.with_direct_operation(operation).into_direct_parts();
                json!({"status":"error", "code":code, "category":category,
                    "details":details, "value":null})
                .to_string()
            }
        }
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Returns reduced U carrier geometry and a certified bound as a JSON envelope.
    #[wasm_bindgen(js_name = "surfaceKnotRemoveU")]
    pub fn surface_knot_remove_u(&self, face: u32, knot: f64, options_json: &str) -> String {
        self.reuse_direct(
            "surfaceKnotRemoveU",
            json!({"face":face,"knot":knot}),
            options_json,
        )
    }

    /// Returns reduced V carrier geometry and a certified bound as a JSON envelope.
    #[wasm_bindgen(js_name = "surfaceKnotRemoveV")]
    pub fn surface_knot_remove_v(&self, face: u32, knot: f64, options_json: &str) -> String {
        self.reuse_direct(
            "surfaceKnotRemoveV",
            json!({"face":face,"knot":knot}),
            options_json,
        )
    }

    /// Returns simplified curve carrier geometry; the source edge remains unchanged.
    #[wasm_bindgen(js_name = "simplifyNurbsCurve")]
    pub fn simplify_nurbs_curve(&self, edge: u32, options_json: &str) -> String {
        self.reuse_direct("simplifyNurbsCurve", json!({"edge":edge}), options_json)
    }

    /// Returns simplified surface carrier geometry; trims and topology remain unchanged.
    #[wasm_bindgen(js_name = "simplifyNurbsSurface")]
    pub fn simplify_nurbs_surface(&self, face: u32, options_json: &str) -> String {
        self.reuse_direct("simplifyNurbsSurface", json!({"face":face}), options_json)
    }

    /// Fits approximate cubic carrier geometry with explicit position/derivative tolerances.
    #[wasm_bindgen(js_name = "fitCubicCurve")]
    pub fn fit_cubic_curve(&self, edge: u32, options_json: &str) -> String {
        self.reuse_direct("fitCubicCurve", json!({"edge":edge}), options_json)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp)]
    use super::*;
    use crate::handles::{edge_id_to_u32, face_id_to_u32};
    use crate::helpers::TOL;
    use remus_math::nurbs::knot_ops::{
        curve_knot_insert, surface_knot_insert_u, surface_knot_insert_v,
    };
    use remus_math::vec::Point3;
    use remus_topology::builder::{make_nurbs_edge_from_curve, make_nurbs_face};

    fn fixture() -> (BrepKernel, u32, u32) {
        let mut kernel = BrepKernel::new();
        let curve = NurbsCurve::new(
            2,
            vec![2., 2., 2., 5., 5., 5.],
            vec![
                Point3::new(0., 0., 0.),
                Point3::new(1.5, 0., 0.),
                Point3::new(3., 0., 0.),
            ],
            vec![1.; 3],
        )
        .unwrap();
        let curve = curve_knot_insert(&curve, 3., 1).unwrap();
        let edge = edge_id_to_u32(make_nurbs_edge_from_curve(kernel.topo_mut(), &curve, TOL));
        let surface = NurbsSurface::new(
            1,
            1,
            vec![2., 2., 5., 5.],
            vec![-1., -1., 1., 1.],
            vec![
                vec![Point3::new(0., 0., 0.), Point3::new(0., 2., 0.)],
                vec![Point3::new(3., 0., 0.), Point3::new(3., 2., 0.)],
            ],
            vec![vec![1.; 2]; 2],
        )
        .unwrap();
        let surface = surface_knot_insert_u(&surface, 3., 1).unwrap();
        let surface = surface_knot_insert_v(&surface, 0., 1).unwrap();
        let face = face_id_to_u32(make_nurbs_face(kernel.topo_mut(), surface, TOL).unwrap());
        (kernel, edge, face)
    }

    #[test]
    fn direct_and_batch_share_all_five_carrier_results_without_topology_mutation() {
        let (mut kernel, edge, face) = fixture();
        let before = (
            kernel.topo.num_faces(),
            kernel.topo.num_edges(),
            kernel.topo.num_vertices(),
        );
        let reduction = json!({"tolerance":1e-7,"maxWork":1_000_000});
        let cubic = json!({"positionTolerance":1e-7,"derivativeTolerance":1e-7});
        for (op, args, direct) in [
            (
                "surfaceKnotRemoveU",
                json!({"face":face,"knot":3.,"options":reduction}),
                kernel.surface_knot_remove_u(face, 3., &reduction.to_string()),
            ),
            (
                "surfaceKnotRemoveV",
                json!({"face":face,"knot":0.,"options":reduction}),
                kernel.surface_knot_remove_v(face, 0., &reduction.to_string()),
            ),
            (
                "simplifyNurbsCurve",
                json!({"edge":edge,"options":reduction}),
                kernel.simplify_nurbs_curve(edge, &reduction.to_string()),
            ),
            (
                "simplifyNurbsSurface",
                json!({"face":face,"options":reduction}),
                kernel.simplify_nurbs_surface(face, &reduction.to_string()),
            ),
            (
                "fitCubicCurve",
                json!({"edge":edge,"options":cubic}),
                kernel.fit_cubic_curve(edge, &cubic.to_string()),
            ),
        ] {
            let direct: Value = serde_json::from_str(&direct).unwrap();
            assert_eq!(direct["status"], "ok", "{op}: {direct}");
            let batch: Value = serde_json::from_str(
                &kernel.execute_batch(
                    &json!([
                {"op":op,"args":args}])
                    .to_string(),
                ),
            )
            .unwrap();
            assert_eq!(direct["value"], batch[0]["ok"], "{op}: {batch}");
            assert_eq!(direct["value"]["geometryOnly"], true);
        }
        assert_eq!(
            before,
            (
                kernel.topo.num_faces(),
                kernel.topo.num_edges(),
                kernel.topo.num_vertices()
            )
        );
        let data: Value =
            serde_json::from_str(&kernel.get_nurbs_curve_data(edge).unwrap()).unwrap();
        assert_eq!(data["controlPoints"].as_array().unwrap().len(), 4);
        assert_eq!(data["domain"], json!([2., 5.]));
    }

    #[test]
    fn analytic_carriers_refuse_without_implicit_nurbs_conversion() {
        let mut kernel = BrepKernel::new();
        kernel.make_box_solid(1., 1., 1.).unwrap();
        let edge = edge_id_to_u32(kernel.topo.edge_id_from_index(0).unwrap());
        let face = face_id_to_u32(kernel.topo.face_id_from_index(0).unwrap());
        for result in [
            kernel.simplify_nurbs_curve(edge, "{}"),
            kernel.simplify_nurbs_surface(face, "{}"),
        ] {
            let result: Value = serde_json::from_str(&result).unwrap();
            assert_eq!(result["code"], "unsupported_nurbs_reuse");
            assert_eq!(result["category"], "unsupported");
        }
        assert_eq!(kernel.topo.num_solids(), 1);
    }

    #[test]
    fn direct_refusals_keep_native_category_and_bounded_options() {
        let (kernel, edge, face) = fixture();
        for options in [
            "null",
            "{\"tolerance\":0}",
            "{\"maxWork\":1.5}",
            "{\"maxWork\":10000001}",
            "{\"typo\":1}",
            "{\"exactOnly\":1}",
        ] {
            let result: Value =
                serde_json::from_str(&kernel.simplify_nurbs_curve(edge, options)).unwrap();
            assert_eq!(result["status"], "error", "{result}");
            assert_eq!(result["category"], "invalid_input");
        }
        let budget: Value =
            serde_json::from_str(&kernel.simplify_nurbs_surface(face, "{\"maxWork\":1}")).unwrap();
        assert_eq!(budget["code"], "nurbs_reuse_work_limit");
        assert_eq!(budget["category"], "resource_limit");
        assert_eq!(budget["details"]["limit"], 1);
        let exact: Value = serde_json::from_str(&kernel.fit_cubic_curve(
            edge,
            "{\"positionTolerance\":1e-7,\"derivativeTolerance\":1e-7,\"exactOnly\":true}",
        ))
        .unwrap();
        assert_eq!(exact["code"], "unsupported_nurbs_reuse");
        assert_eq!(exact["category"], "unsupported");
        let invalid_handle: Value =
            serde_json::from_str(&kernel.simplify_nurbs_curve(u32::MAX, "{}")).unwrap();
        assert_eq!(invalid_handle["category"], "invalid_input");
        let invalid_knot: Value =
            serde_json::from_str(&kernel.surface_knot_remove_u(face, f64::NAN, "{}")).unwrap();
        assert_eq!(invalid_knot["status"], "error");
    }
}
