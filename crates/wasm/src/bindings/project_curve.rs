//! Directional curve-projection bindings (P-Class 7.4).
//!
//! Contract: `docs/design/p74-curve-projection.md` §8. Direct methods accept
//! typed objects and return typed objects; `executeBatch` serves the same
//! three operations and value schemas. Refusals retain the stable
//! `project-curve:<code>` kernel code.

#![allow(clippy::missing_errors_doc)]

use remus_math::curves2d::Curve2D;
use remus_math::frame::Frame3;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::knot_ops::curve_split;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::project_curve::{self as projection, ProjectCurveError};
use remus_topology::pcurve::PCurve;
use remus_topology::transaction::RollbackSnapshot;
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use crate::error::StructuredWasmError;
use crate::handles::{edge_id_to_u32, face_id_to_u32};
use crate::helpers::{get_f64, get_u32, get_u32_array};
use crate::kernel::BrepKernel;
use crate::types::{
    PlaneCurve2d, ProjectCurveOptions, ProjectedCurves, ProjectedEdge, ProjectionQuality,
    SketchFrame, SolidProjection, SourceProjection,
};

impl From<ProjectCurveError> for StructuredWasmError {
    fn from(error: ProjectCurveError) -> Self {
        use ProjectCurveError as P;
        let code = format!("project-curve:{}", error.code());
        match error {
            P::InvalidDirection | P::InvalidOptions { .. } | P::PlaneFrameMismatch => {
                let mut structured = Self::invalid_argument(error.to_string(), None);
                structured
                    .details_mut()
                    .insert("kernelCode".to_string(), code.into());
                structured
            }
            P::Operations(operations) => {
                let mut structured = Self::from(operations);
                structured
                    .details_mut()
                    .insert("kernelCode".to_string(), code.into());
                structured
            }
            other => {
                let mut structured = Self::operation_failed(other.to_string());
                structured
                    .details_mut()
                    .insert("kernelCode".to_string(), code.into());
                structured
            }
        }
    }
}

fn invalid_options(reason: impl std::fmt::Display) -> StructuredWasmError {
    ProjectCurveError::InvalidOptions {
        reason: reason.to_string(),
    }
    .into()
}

/// The same deserializer validates batch options and typed direct arguments.
fn parse_project_options(
    value: Option<&serde_json::Value>,
) -> Result<ProjectCurveOptions, StructuredWasmError> {
    match value {
        None | Some(serde_json::Value::Null) => Ok(ProjectCurveOptions::default()),
        Some(value) if value.is_object() => {
            serde_json::from_value(value.clone()).map_err(invalid_options)
        }
        Some(_) => Err(invalid_options("options must be an object")),
    }
}

fn parse_sketch_frame(value: &serde_json::Value) -> Result<SketchFrame, StructuredWasmError> {
    if !value.is_object() {
        return Err(invalid_options("frame must be an object"));
    }
    serde_json::from_value(value.clone()).map_err(invalid_options)
}

fn typed_object<T>(value: &tsify::Ts<T>, name: &str) -> Result<T, StructuredWasmError>
where
    T: tsify::Tsify + serde::de::DeserializeOwned,
    T::JsType: Clone,
{
    let js = value.js_value();
    if !js.is_object() || js.is_array() {
        return Err(invalid_options(format!("{name} must be an object")));
    }
    value.to_rust().map_err(invalid_options)
}

// Scale before normalization so large/small finite direction components
// remain usable without overflowing or underflowing their squared norm.
fn frame_axis(components: [f64; 3]) -> Result<Vec3, StructuredWasmError> {
    let scale = components.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
    if scale == 0.0 {
        return Err(invalid_options("frame axes must be non-zero"));
    }
    Vec3::new(
        components[0] / scale,
        components[1] / scale,
        components[2] / scale,
    )
    .normalize()
    .map_err(invalid_options)
}

fn native_frame(frame: &SketchFrame) -> Result<Frame3, StructuredWasmError> {
    for (key, components) in [
        ("origin", frame.origin),
        ("xAxis", frame.x_axis),
        ("normal", frame.normal),
    ] {
        if !components.iter().all(|v| v.is_finite()) {
            return Err(invalid_options(format!("frame '{key}' must be finite")));
        }
    }
    let x = frame_axis(frame.x_axis)?;
    let z = frame_axis(frame.normal)?;
    if x.dot(z).abs() > 1e-9 {
        return Err(invalid_options(
            "frame xAxis and normal must be perpendicular",
        ));
    }
    Ok(Frame3 {
        origin: Point3::new(frame.origin[0], frame.origin[1], frame.origin[2]),
        x,
        y: z.cross(x),
        z,
    })
}

fn native_options(
    options: &ProjectCurveOptions,
) -> Result<projection::ProjectCurveOptions, StructuredWasmError> {
    Ok(projection::ProjectCurveOptions {
        allow_approximate: options.allow_approximate.unwrap_or(false),
        approximation_tolerance: options.approximation_tolerance,
        max_control_points: options
            .max_control_points
            .unwrap_or(projection::DEFAULT_MAX_CONTROL_POINTS),
        plane_frame: options.plane_frame.as_ref().map(native_frame).transpose()?,
    })
}

fn point_2d(point: remus_math::vec::Point2) -> [f64; 2] {
    [point.x(), point.y()]
}

fn plane_curve(curve: &PCurve) -> Result<PlaneCurve2d, ProjectCurveError> {
    Ok(match curve.curve() {
        Curve2D::Line(line) => PlaneCurve2d::Line {
            start: point_2d(line.origin()),
            end: point_2d(line.origin() + line.direction() * (curve.t_end() - curve.t_start())),
        },
        Curve2D::Circle(circle) => PlaneCurve2d::Circle {
            center: point_2d(circle.center()),
            radius: circle.radius(),
            start_angle: curve.t_start(),
            end_angle: curve.t_end(),
        },
        Curve2D::Ellipse(ellipse) => PlaneCurve2d::Ellipse {
            center: point_2d(ellipse.center()),
            semi_major: ellipse.semi_major(),
            semi_minor: ellipse.semi_minor(),
            rotation: ellipse.rotation(),
            start_angle: curve.t_start(),
            end_angle: curve.t_end(),
        },
        Curve2D::Nurbs(nurbs) => {
            // Publish the restricted carrier with its original knot interval;
            // tStart/tEnd preserve the source's signed traversal.
            let mut restricted = NurbsCurve::new(
                nurbs.degree(),
                nurbs.knots().to_vec(),
                nurbs
                    .control_points()
                    .iter()
                    .map(|p| Point3::new(p.x(), p.y(), 0.0))
                    .collect(),
                nurbs.weights().to_vec(),
            )
            .map_err(OperationsError::from)?;
            let start = curve.t_start().min(curve.t_end());
            let end = curve.t_start().max(curve.t_end());
            let (lower, upper) = restricted.domain();
            if !start.is_finite()
                || !end.is_finite()
                || start < lower
                || end > upper
                || start >= end
            {
                return Err(ProjectCurveError::DegenerateSource {
                    reason: "invalid NURBS plane-curve interval",
                });
            }
            if start > lower {
                restricted = curve_split(&restricted, start)
                    .map_err(OperationsError::from)?
                    .1;
            }
            if end < upper {
                restricted = curve_split(&restricted, end)
                    .map_err(OperationsError::from)?
                    .0;
            }
            PlaneCurve2d::Nurbs {
                t_start: curve.t_start(),
                t_end: curve.t_end(),
                degree: restricted.degree(),
                knots: restricted.knots().to_vec(),
                control_points: restricted
                    .control_points()
                    .iter()
                    .map(|p| [p.x(), p.y()])
                    .collect(),
                weights: restricted.weights().to_vec(),
            }
        }
    })
}

fn quality(quality: &projection::ProjectionQuality) -> ProjectionQuality {
    match quality {
        projection::ProjectionQuality::Exact => ProjectionQuality::Exact,
        projection::ProjectionQuality::Approximate { max_deviation } => {
            ProjectionQuality::Approximate {
                max_deviation: *max_deviation,
            }
        }
    }
}

fn projected_edge(edge: &projection::ProjectedEdge) -> Result<ProjectedEdge, ProjectCurveError> {
    Ok(ProjectedEdge {
        edge: edge_id_to_u32(edge.edge),
        face: face_id_to_u32(edge.face),
        source_start: edge.source_range.0,
        source_end: edge.source_range.1,
        plane_curve: edge.plane_curve.as_ref().map(plane_curve).transpose()?,
    })
}

fn projected_curves(
    result: &projection::ProjectedCurves,
) -> Result<ProjectedCurves, ProjectCurveError> {
    Ok(ProjectedCurves {
        edges: result
            .edges
            .iter()
            .map(projected_edge)
            .collect::<Result<_, _>>()?,
        face: face_id_to_u32(result.face),
        quality: quality(&result.quality),
        clipped: result.clipped,
    })
}

fn solid_projection(
    result: &projection::SolidProjection,
) -> Result<SolidProjection, ProjectCurveError> {
    let sources = result
        .sources
        .iter()
        .enumerate()
        .map(|(index, source)| {
            let edges = source
                .edges
                .iter()
                .map(projected_edge)
                .collect::<Result<_, _>>()
                .map_err(|error| ProjectCurveError::SourceRefused {
                    index,
                    error: Box::new(error),
                })?;
            Ok(SourceProjection {
                source: edge_id_to_u32(source.source),
                edges,
                clipped: source.clipped,
            })
        })
        .collect::<Result<_, ProjectCurveError>>()?;
    Ok(SolidProjection {
        sources,
        quality: quality(&result.quality),
    })
}

fn resolution_error(error: crate::error::WasmError, index: Option<usize>) -> StructuredWasmError {
    let error = ProjectCurveError::Operations(OperationsError::InvalidInput {
        reason: error.to_string(),
    });
    match index {
        Some(index) => ProjectCurveError::SourceRefused {
            index,
            error: Box::new(error),
        }
        .into(),
        None => error.into(),
    }
}

impl BrepKernel {
    fn project_curve_onto_face_typed(
        &mut self,
        edge: u32,
        direction: [f64; 3],
        face: u32,
        options: &ProjectCurveOptions,
    ) -> Result<ProjectedCurves, StructuredWasmError> {
        let edge_id = self
            .resolve_edge(edge)
            .map_err(|error| resolution_error(error, None))?;
        let face_id = self
            .resolve_face(face)
            .map_err(|error| resolution_error(error, None))?;
        let options = native_options(options)?;
        let snapshot = RollbackSnapshot::capture(self.topo_mut());
        let result = projection::project_curve_onto_face(
            self.topo_mut(),
            edge_id,
            Vec3::new(direction[0], direction[1], direction[2]),
            face_id,
            &options,
        )
        .and_then(|result| projected_curves(&result));
        match result {
            Ok(result) => {
                snapshot.commit(self.topo_mut());
                Ok(result)
            }
            Err(error) => {
                snapshot.restore(self.topo_mut());
                Err(error.into())
            }
        }
    }

    fn project_curves_onto_solid_typed(
        &mut self,
        edges: &[u32],
        direction: [f64; 3],
        solid: u32,
        options: &ProjectCurveOptions,
    ) -> Result<SolidProjection, StructuredWasmError> {
        let edge_ids = edges
            .iter()
            .enumerate()
            .map(|(index, handle)| {
                self.resolve_edge(*handle)
                    .map_err(|error| resolution_error(error, Some(index)))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(|error| resolution_error(error, None))?;
        let options = native_options(options)?;
        let snapshot = RollbackSnapshot::capture(self.topo_mut());
        let result = projection::project_curves_onto_solid(
            self.topo_mut(),
            &edge_ids,
            Vec3::new(direction[0], direction[1], direction[2]),
            solid_id,
            &options,
        )
        .and_then(|result| solid_projection(&result));
        match result {
            Ok(result) => {
                snapshot.commit(self.topo_mut());
                Ok(result)
            }
            Err(error) => {
                snapshot.restore(self.topo_mut());
                Err(error.into())
            }
        }
    }

    fn project_curves_onto_sketch_plane_typed(
        &self,
        edges: &[u32],
        direction: [f64; 3],
        frame: &SketchFrame,
    ) -> Result<Vec<PlaneCurve2d>, StructuredWasmError> {
        let edge_ids = edges
            .iter()
            .enumerate()
            .map(|(index, handle)| {
                self.resolve_edge(*handle)
                    .map_err(|error| resolution_error(error, Some(index)))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let frame = native_frame(frame)?;
        let curves = projection::project_curves_onto_plane(
            self.topo(),
            &edge_ids,
            Vec3::new(direction[0], direction[1], direction[2]),
            &frame,
        )
        .map_err(StructuredWasmError::from)?;
        curves
            .iter()
            .enumerate()
            .map(|(index, curve)| {
                plane_curve(curve).map_err(|error| {
                    ProjectCurveError::SourceRefused {
                        index,
                        error: Box::new(error),
                    }
                    .into()
                })
            })
            .collect()
    }

    /// Dispatch one curve-projection batch op; `None` for other names.
    pub(crate) fn dispatch_project_curve_op(
        &mut self,
        op: &str,
        args: &serde_json::Value,
    ) -> Option<Result<serde_json::Value, StructuredWasmError>> {
        let direction = || -> Result<[f64; 3], StructuredWasmError> {
            Ok([
                get_f64(args, "dirX")?,
                get_f64(args, "dirY")?,
                get_f64(args, "dirZ")?,
            ])
        };
        match op {
            "projectCurveOntoFace" => Some((|| {
                let result = self.project_curve_onto_face_typed(
                    get_u32(args, "edge")?,
                    direction()?,
                    get_u32(args, "face")?,
                    &parse_project_options(args.get("options"))?,
                )?;
                serde_json::to_value(result).map_err(StructuredWasmError::from)
            })()),
            "projectCurvesOntoSolid" => Some((|| {
                let result = self.project_curves_onto_solid_typed(
                    &get_u32_array(args, "edges")?,
                    direction()?,
                    get_u32(args, "solid")?,
                    &parse_project_options(args.get("options"))?,
                )?;
                serde_json::to_value(result).map_err(StructuredWasmError::from)
            })()),
            "projectCurvesOntoSketchPlane" => Some((|| {
                let frame = args
                    .get("frame")
                    .ok_or_else(|| invalid_options("missing 'frame'"))?;
                let result = self.project_curves_onto_sketch_plane_typed(
                    &get_u32_array(args, "edges")?,
                    direction()?,
                    &parse_sketch_frame(frame)?,
                )?;
                serde_json::to_value(result).map_err(StructuredWasmError::from)
            })()),
            _ => None,
        }
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Project one edge along a direction onto a face's trimmed region.
    /// Returns the §8 typed ProjectedCurves object. Refusals throw with the
    /// stable `project-curve:<code>` prefix; omitted options use defaults.
    #[wasm_bindgen(js_name = "projectCurveOntoFace")]
    pub fn project_curve_onto_face_js(
        &mut self,
        edge: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        face: u32,
        options: Option<tsify::Ts<ProjectCurveOptions>>,
    ) -> Result<tsify::Ts<ProjectedCurves>, JsError> {
        let options = options
            .map(|options| typed_object(&options, "options"))
            .transpose()
            .map_err(structured_to_js)?
            .unwrap_or_default();
        Ok(self
            .project_curve_onto_face_typed(edge, [dir_x, dir_y, dir_z], face, &options)
            .map_err(structured_to_js)?
            .into_ts()?)
    }

    /// Project edges along a direction onto a solid's first-hit faces.
    /// Returns the §8 typed SolidProjection object, atomically over sources.
    #[wasm_bindgen(js_name = "projectCurvesOntoSolid")]
    pub fn project_curves_onto_solid_js(
        &mut self,
        edges: Vec<u32>,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        solid: u32,
        options: Option<tsify::Ts<ProjectCurveOptions>>,
    ) -> Result<tsify::Ts<SolidProjection>, JsError> {
        let options = options
            .map(|options| typed_object(&options, "options"))
            .transpose()
            .map_err(structured_to_js)?
            .unwrap_or_default();
        Ok(self
            .project_curves_onto_solid_typed(&edges, [dir_x, dir_y, dir_z], solid, &options)
            .map_err(structured_to_js)?
            .into_ts()?)
    }

    /// Project edges onto the unbounded plane of a typed SketchFrame object.
    /// Returns the §8 PlaneCurve2d object array without changing topology.
    #[wasm_bindgen(js_name = "projectCurvesOntoSketchPlane")]
    pub fn project_curves_onto_sketch_plane_js(
        &self,
        edges: Vec<u32>,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        frame: tsify::Ts<SketchFrame>,
    ) -> Result<Vec<tsify::Ts<PlaneCurve2d>>, JsError> {
        let frame = typed_object(&frame, "frame").map_err(structured_to_js)?;
        self.project_curves_onto_sketch_plane_typed(&edges, [dir_x, dir_y, dir_z], &frame)
            .map_err(structured_to_js)?
            .iter()
            .map(|curve| curve.into_ts().map_err(JsError::from))
            .collect()
    }
}

fn structured_to_js(error: StructuredWasmError) -> JsError {
    let message = error.message().to_string();
    let (code, _, _) = error.into_direct_parts();
    JsError::new(&format!("{code}: {message}"))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
    use crate::kernel::BrepKernel;

    fn batch(kernel: &mut BrepKernel, ops: &str) -> Vec<serde_json::Value> {
        let response = kernel.execute_batch(ops);
        serde_json::from_str(&response).expect("batch response parses")
    }

    fn batch_v2(kernel: &mut BrepKernel, ops: &str) -> Vec<serde_json::Value> {
        let response = kernel.execute_batch_v2(ops);
        serde_json::from_str(&response).expect("batch v2 response parses")
    }

    #[test]
    fn batch_projection_keeps_retained_reference_through_next_journaled_edit() {
        for v2 in [false, true] {
            for onto_solid in [false, true] {
                let mut kernel = BrepKernel::new();
                let made = batch(
                    &mut kernel,
                    r#"[
                    {"op":"makeBox","args":{"width":10,"height":8,"depth":4}},
                    {"op":"makeBox","args":{"width":1,"height":1,"depth":1}},
                    {"op":"makeBox","args":{"width":2,"height":3,"depth":4}},
                    {"op":"makeBox","args":{"width":1,"height":1,"depth":1}},
                    {"op":"makeLineEdge","args":{"x1":1,"y1":2,"z1":7,"x2":8,"y2":5,"z2":9}}
                ]"#,
                );
                let handles: Vec<_> = made
                    .iter()
                    .map(|row| u32::try_from(row["ok"].as_u64().unwrap()).unwrap())
                    .collect();
                let anchor: serde_json::Value =
                    serde_json::from_str(&kernel.fuse_journaled(handles[0], handles[1]).unwrap())
                        .unwrap();
                let target = u32::try_from(anchor["solid"].as_u64().unwrap()).unwrap();
                let operation = u32::try_from(anchor["op"].as_u64().unwrap()).unwrap();
                let reference = kernel
                    .make_operation_output_ref(operation, "face", 0)
                    .unwrap();
                let before = kernel.resolve_ref(&reference).unwrap();
                let face = kernel.get_solid_faces(target).unwrap().into_iter().find(|id| {
                    let face = kernel.resolve_face(*id).unwrap();
                    matches!(kernel.topo.face(face).unwrap().surface(), remus_topology::face::FaceSurface::Plane { normal, .. } if normal.z() > 0.9)
                }).unwrap();
                let request = if onto_solid {
                    serde_json::json!([{"op":"projectCurvesOntoSolid","args":{"edges":[handles[4]],"dirX":0,"dirY":0,"dirZ":-1,"solid":target}}])
                } else {
                    serde_json::json!([{"op":"projectCurveOntoFace","args":{"edge":handles[4],"dirX":0,"dirY":0,"dirZ":-1,"face":face}}])
                }.to_string();
                let result = if v2 {
                    batch_v2(&mut kernel, &request)
                } else {
                    batch(&mut kernel, &request)
                };
                assert!(result[0].get("ok").is_some(), "{result:?}");
                assert_eq!(kernel.topo.journal().len(), 2);
                kernel.fuse_journaled(handles[2], handles[3]).unwrap();
                assert_eq!(kernel.resolve_ref(&reference).unwrap(), before);
                assert_eq!(kernel.topo.journal().len(), 3);
            }
        }
    }

    fn box_top_face(kernel: &mut BrepKernel) -> (u32, u32) {
        let made = batch(
            kernel,
            r#"[{"op":"makeBox","args":{"width":10.0,"height":8.0,"depth":4.0}}]"#,
        );
        let solid = made[0]["ok"].as_u64().expect("solid") as u32;
        let faces = batch(
            kernel,
            &format!(r#"[{{"op":"getSolidFaces","args":{{"solid":{solid}}}}}]"#),
        );
        let list = faces[0]["ok"].as_array().expect("faces").clone();
        // Top face: normal +z (face 4 of the box primitive order is not
        // contractual here, so probe each face's normal).
        for face in list {
            let id = face.as_u64().expect("face") as u32;
            let probe = batch(
                kernel,
                &format!(r#"[{{"op":"getFaceNormal","args":{{"face":{id}}}}}]"#),
            );
            let normal = probe[0]["ok"].as_array().expect("normal").clone();
            let nz = normal[2].as_f64().expect("nz");
            if nz > 0.9 {
                return (solid, id);
            }
        }
        panic!("no top face");
    }

    #[test]
    fn batch_exact_segment_projection_reports_quality_and_range() {
        let mut kernel = BrepKernel::new();
        let (_solid, face) = box_top_face(&mut kernel);
        // A free segment above the box, built from two vertices and an edge.
        let made = batch(
            &mut kernel,
            r#"[{"op":"makeLineEdge","args":{"x1":1.0,"y1":2.0,"z1":7.0,"x2":8.0,"y2":5.0,"z2":9.0}}]"#,
        );
        let edge = made[0]["ok"].as_u64().expect("edge") as u32;
        let ops = format!(
            r#"[{{"op":"projectCurveOntoFace","args":{{"edge":{edge},"dirX":0.3,"dirY":-0.2,"dirZ":-1.0,"face":{face}}}}}]"#
        );
        let out = batch(&mut kernel, &ops);
        let result = &out[0]["ok"];
        assert_eq!(result["quality"]["kind"], "exact");
        assert_eq!(result["clipped"], false);
        assert_eq!(result["edges"].as_array().expect("edges").len(), 1);
        assert_eq!(result["edges"][0]["sourceStart"], 0.0);
        assert_eq!(result["edges"][0]["sourceEnd"], 1.0);
        assert!(result["edges"][0].get("planeCurve").is_none());
    }

    #[test]
    fn batch_refusals_carry_project_curve_codes() {
        let mut kernel = BrepKernel::new();
        let (_solid, face) = box_top_face(&mut kernel);
        let made = batch(
            &mut kernel,
            r#"[{"op":"makeLineEdge","args":{"x1":1.0,"y1":2.0,"z1":7.0,"x2":8.0,"y2":5.0,"z2":9.0}}]"#,
        );
        let edge = made[0]["ok"].as_u64().expect("edge") as u32;
        // Zero direction.
        let out = batch_v2(
            &mut kernel,
            &format!(
                r#"[{{"op":"projectCurveOntoFace","args":{{"edge":{edge},"dirX":0.0,"dirY":0.0,"dirZ":0.0,"face":{face}}}}}]"#
            ),
        );
        assert_eq!(
            out[0]["error"]["details"]["kernelCode"],
            "project-curve:invalid-direction"
        );
        // Grazing direction (in-plane).
        let out = batch_v2(
            &mut kernel,
            &format!(
                r#"[{{"op":"projectCurveOntoFace","args":{{"edge":{edge},"dirX":1.0,"dirY":1.0,"dirZ":0.0,"face":{face}}}}}]"#
            ),
        );
        assert_eq!(
            out[0]["error"]["details"]["kernelCode"],
            "project-curve:grazing-direction"
        );
    }

    #[test]
    fn batch_sketch_plane_line_is_read_only_and_exact() {
        let mut kernel = BrepKernel::new();
        let made = batch(
            &mut kernel,
            r#"[{"op":"makeLineEdge","args":{"x1":0.0,"y1":0.0,"z1":4.0,"x2":10.0,"y2":0.0,"z2":4.0}}]"#,
        );
        let edge = made[0]["ok"].as_u64().expect("edge") as u32;
        let before = batch(&mut kernel, r#"[{"op":"journalSummary","args":{}}]"#);
        let out = batch(
            &mut kernel,
            &format!(
                r#"[{{"op":"projectCurvesOntoSketchPlane","args":{{"edges":[{edge}],"dirX":0.0,"dirY":0.0,"dirZ":-1.0,"frame":{{"origin":[1.0,1.0,0.0],"xAxis":[1.0,0.0,0.0],"normal":[0.0,0.0,1.0]}}}}}}]"#
            ),
        );
        let curves = out[0]["ok"].as_array().expect("curves").clone();
        assert_eq!(curves.len(), 1);
        assert_eq!(curves[0]["kind"], "line");
        let after = batch(&mut kernel, r#"[{"op":"journalSummary","args":{}}]"#);
        assert_eq!(before, after, "sketch projection is read-only");
    }

    #[test]
    fn typed_helper_and_batch_projection_agree() {
        let mut kernel = BrepKernel::new();
        let (_solid, face) = box_top_face(&mut kernel);
        let made = batch(
            &mut kernel,
            r#"[{"op":"makeLineEdge","args":{"x1":1.0,"y1":2.0,"z1":7.0,"x2":8.0,"y2":5.0,"z2":9.0}}]"#,
        );
        let edge = made[0]["ok"].as_u64().expect("edge") as u32;
        let direct = serde_json::to_value(
            kernel
                .project_curve_onto_face_typed(
                    edge,
                    [0.3, -0.2, -1.0],
                    face,
                    &super::ProjectCurveOptions::default(),
                )
                .expect("typed projection"),
        )
        .expect("typed result serializes");
        let out = batch(
            &mut kernel,
            &format!(
                r#"[{{"op":"projectCurveOntoFace","args":{{"edge":{edge},"dirX":0.3,"dirY":-0.2,"dirZ":-1.0,"face":{face}}}}}]"#
            ),
        );
        // Batch allocates a second edge pair; compare shapes, not handles.
        assert_eq!(out[0]["ok"]["quality"], direct["quality"]);
        assert_eq!(out[0]["ok"]["clipped"], direct["clipped"]);
        assert_eq!(
            out[0]["ok"]["edges"].as_array().expect("e").len(),
            direct["edges"].as_array().expect("e").len()
        );
    }

    fn counts(kernel: &BrepKernel) -> (usize, usize, usize, usize) {
        (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_faces(),
            kernel.topo().num_solids(),
        )
    }

    fn line_above_box(kernel: &mut BrepKernel) -> u32 {
        batch(
            kernel,
            r#"[{"op":"makeLineEdge","args":{"x1":1,"y1":2,"z1":7,"x2":8,"y2":5,"z2":9}}]"#,
        )[0]["ok"]
            .as_u64()
            .unwrap() as u32
    }

    #[test]
    fn typed_projection_schema_matches_batch_for_all_three_operations() {
        let mut direct = BrepKernel::new();
        let (solid, face) = box_top_face(&mut direct);
        let edge = line_above_box(&mut direct);
        let frame = serde_json::json!({"origin":[0,0,4], "xAxis":[1,0,0], "normal":[0,0,1]});
        let options = serde_json::json!({"planeFrame":frame});
        let mut via_batch = BrepKernel::new();
        via_batch.topo = direct.topo().clone().into();
        let typed = serde_json::to_value(
            direct
                .project_curve_onto_face_typed(
                    edge,
                    [0.3, -0.2, -1.0],
                    face,
                    &super::parse_project_options(Some(&options)).unwrap(),
                )
                .unwrap(),
        )
        .unwrap();
        let response = batch(
            &mut via_batch,
            &serde_json::json!([{"op":"projectCurveOntoFace","args":{
            "edge":edge,"face":face,"dirX":0.3,"dirY":-0.2,"dirZ":-1,"options":options}}])
            .to_string(),
        );
        assert_eq!(typed, response[0]["ok"]);
        assert_eq!(typed["edges"][0]["planeCurve"]["kind"], "line");
        let typed = serde_json::to_value(
            direct
                .project_curves_onto_solid_typed(
                    &[edge],
                    [0.3, -0.2, -1.0],
                    solid,
                    &super::ProjectCurveOptions::default(),
                )
                .unwrap(),
        )
        .unwrap();
        let response = batch(
            &mut via_batch,
            &serde_json::json!([{"op":"projectCurvesOntoSolid","args":{
            "edges":[edge],"solid":solid,"dirX":0.3,"dirY":-0.2,"dirZ":-1}}])
            .to_string(),
        );
        assert_eq!(typed, response[0]["ok"]);
        assert!(typed["sources"][0]["edges"][0].get("planeCurve").is_none());
        let before = counts(&direct);
        let typed = serde_json::to_value(
            direct
                .project_curves_onto_sketch_plane_typed(
                    &[edge],
                    [0.3, -0.2, -1.0],
                    &super::parse_sketch_frame(&frame).unwrap(),
                )
                .unwrap(),
        )
        .unwrap();
        let response = batch(
            &mut via_batch,
            &serde_json::json!([{"op":"projectCurvesOntoSketchPlane","args":{
            "edges":[edge],"dirX":0.3,"dirY":-0.2,"dirZ":-1,"frame":frame}}])
            .to_string(),
        );
        assert_eq!(typed, response[0]["ok"]);
        assert!(typed.is_array());
        assert_eq!(counts(&direct), before);
    }

    #[test]
    fn malformed_option_and_frame_fields_refuse_without_mutation() {
        let mut kernel = BrepKernel::new();
        let (_solid, face) = box_top_face(&mut kernel);
        let edge = line_above_box(&mut kernel);
        let before = counts(&kernel);
        for options in [
            serde_json::json!("{}"),
            serde_json::json!([]),
            serde_json::json!({"allowApproximate":"false"}),
            serde_json::json!({"allowApproximate":null}),
            serde_json::json!({"approximationTolerance":"0.01"}),
            serde_json::json!({"approximationTolerance":null}),
            serde_json::json!({"maxControlPoints":4.5}),
            serde_json::json!({"maxControlPoints":-4}),
            serde_json::json!({"maxControlPoints":"4"}),
            serde_json::json!({"maxControlPoints":null}),
            serde_json::json!({"planeFrame":null}),
            serde_json::json!({"planeFrame":"{}"}),
        ] {
            let (code, _, _) = super::parse_project_options(Some(&options))
                .unwrap_err()
                .into_direct_parts();
            assert_eq!(code, "project-curve:invalid-options", "{options}");
            let out = batch_v2(
                &mut kernel,
                &serde_json::json!([{"op":"projectCurveOntoFace","args":{
                "edge":edge,"face":face,"dirX":0,"dirY":0,"dirZ":-1,"options":options}}])
                .to_string(),
            );
            assert_eq!(
                out[0]["error"]["details"]["kernelCode"],
                "project-curve:invalid-options"
            );
            assert_eq!(counts(&kernel), before);
        }
        for frame in [
            serde_json::json!("{}"),
            serde_json::json!({"origin":[0,0],"xAxis":[1,0,0],"normal":[0,0,1]}),
            serde_json::json!({"origin":[0,0,0],"xAxis":["1",0,0],"normal":[0,0,1]}),
            serde_json::json!({"origin":[0,0,0],"xAxis":[0,0,0],"normal":[0,0,1]}),
            serde_json::json!({"origin":[0,0,0],"xAxis":[1,0,0],"normal":[1,0,0]}),
        ] {
            let out = batch_v2(
                &mut kernel,
                &serde_json::json!([{"op":"projectCurvesOntoSketchPlane","args":{
                "edges":[edge],"dirX":0,"dirY":0,"dirZ":-1,"frame":frame}}])
                .to_string(),
            );
            assert_eq!(
                out[0]["error"]["details"]["kernelCode"], "project-curve:invalid-options",
                "{frame}"
            );
            assert_eq!(counts(&kernel), before);
        }
        let frame = super::SketchFrame {
            origin: [0.0; 3],
            x_axis: [f64::NAN, 0.0, 0.0],
            normal: [0.0, 0.0, 1.0],
        };
        assert!(super::native_frame(&frame).is_err());
        let frame = super::SketchFrame {
            origin: [0.0; 3],
            x_axis: [1e300, 0.0, 0.0],
            normal: [0.0, 0.0, 1e-300],
        };
        assert!(super::native_frame(&frame).is_ok());
    }

    #[test]
    fn stale_handle_refusals_keep_projection_codes_and_source_indices() {
        let mut kernel = BrepKernel::new();
        let (solid, face) = box_top_face(&mut kernel);
        let edge = line_above_box(&mut kernel);
        let before = counts(&kernel);
        let (code, _, _) = kernel
            .project_curve_onto_face_typed(
                u32::MAX,
                [0.0, 0.0, -1.0],
                face,
                &super::ProjectCurveOptions::default(),
            )
            .unwrap_err()
            .into_direct_parts();
        assert_eq!(code, "project-curve:operations");
        for args in [
            serde_json::json!({"op":"projectCurvesOntoSolid","args":{"edges":[edge,u32::MAX],"solid":solid,"dirX":0,"dirY":0,"dirZ":-1}}),
            serde_json::json!({"op":"projectCurvesOntoSketchPlane","args":{"edges":[edge,u32::MAX],"dirX":0,"dirY":0,"dirZ":-1,
                "frame":{"origin":[0,0,0],"xAxis":[1,0,0],"normal":[0,0,1]}}}),
        ] {
            let out = batch_v2(&mut kernel, &serde_json::json!([args]).to_string());
            assert_eq!(
                out[0]["error"]["details"]["kernelCode"],
                "project-curve:source-refused"
            );
            assert!(
                out[0]["error"]["message"]
                    .as_str()
                    .unwrap()
                    .contains("source 1 refused")
            );
            assert_eq!(counts(&kernel), before);
        }
    }

    #[test]
    fn nurbs_plane_schema_restricts_the_carrier_without_changing_parameters() {
        use remus_math::curves2d::{Curve2D, NurbsCurve2D};
        use remus_math::vec::Point2;
        use remus_topology::pcurve::PCurve;
        let original = NurbsCurve2D::new(
            2,
            vec![2.0, 2.0, 2.0, 5.0, 5.0, 5.0],
            vec![
                Point2::new(0.0, 0.0),
                Point2::new(2.0, 3.0),
                Point2::new(5.0, 1.0),
            ],
            vec![1.0, 2.0, 1.0],
        )
        .unwrap();
        for (start, end) in [(3.0, 4.0), (4.0, 3.0), (2.0, 5.0)] {
            let curve =
                super::plane_curve(&PCurve::new(Curve2D::Nurbs(original.clone()), start, end))
                    .unwrap();
            let value = serde_json::to_value(&curve).unwrap();
            assert_eq!(value["tStart"], start);
            assert_eq!(value["tEnd"], end);
            let super::PlaneCurve2d::Nurbs {
                degree,
                knots,
                control_points,
                weights,
                ..
            } = curve
            else {
                panic!("wrong kind")
            };
            assert_eq!(knots[degree].to_bits(), start.min(end).to_bits());
            assert_eq!(
                knots[control_points.len()].to_bits(),
                start.max(end).to_bits()
            );
            let restricted = NurbsCurve2D::new(
                degree,
                knots,
                control_points
                    .iter()
                    .map(|p| Point2::new(p[0], p[1]))
                    .collect(),
                weights,
            )
            .unwrap();
            for i in 0..=32 {
                let parameter = start + (end - start) * f64::from(i) / 32.0;
                assert!(
                    (restricted.evaluate(parameter) - original.evaluate(parameter)).length()
                        < 1e-12
                );
            }
        }
    }

    #[test]
    fn generated_types_expose_the_documented_objects_and_discriminants() {
        use tsify::Tsify;
        assert!(super::ProjectCurveOptions::DECL.contains("allowApproximate?: boolean"));
        assert!(super::ProjectCurveOptions::DECL.contains("planeFrame?: SketchFrame"));
        assert!(super::SketchFrame::DECL.contains("origin: [number, number, number]"));
        assert!(super::ProjectedCurves::DECL.contains("edges: ProjectedEdge[]"));
        assert!(super::SolidProjection::DECL.contains("sources: SourceProjection[]"));
        assert!(super::PlaneCurve2d::DECL.contains("tStart: number"));
        assert!(super::PlaneCurve2d::DECL.contains("tEnd: number"));
        assert!(super::ProjectionQuality::DECL.contains("maxDeviation: number"));
        let value = serde_json::to_value(super::ProjectionQuality::Approximate {
            max_deviation: 0.125,
        })
        .unwrap();
        assert_eq!(
            value,
            serde_json::json!({"kind":"approximate","maxDeviation":0.125})
        );
    }
}
