//! Directional curve-projection bindings (P-Class 7.4).
//!
//! Contract: `docs/design/p74-curve-projection.md` §8. Direct methods return
//! the §8 JSON shapes as strings; `executeBatch` serves the same three ops
//! value-for-value. Refusals surface as structured errors whose `kernelCode`
//! detail is `project-curve:<code>` from
//! [`ProjectCurveError::code`](remus_operations::project_curve::ProjectCurveError::code).

#![allow(clippy::missing_errors_doc)]

use remus_math::curves2d::Curve2D;
use remus_math::frame::Frame3;
use remus_math::vec::{Point3, Vec3};
use remus_operations::project_curve::{
    ProjectCurveError, ProjectCurveOptions, ProjectedCurves, ProjectionQuality, SolidProjection,
};
use remus_topology::pcurve::PCurve;
use wasm_bindgen::prelude::*;

use crate::error::StructuredWasmError;
use crate::handles::{edge_id_to_u32, face_id_to_u32};
use crate::helpers::{get_f64, get_u32, get_u32_array};
use crate::kernel::BrepKernel;

impl From<ProjectCurveError> for StructuredWasmError {
    fn from(error: ProjectCurveError) -> Self {
        use ProjectCurveError as P;
        let code = format!("project-curve:{}", error.code());
        match error {
            P::InvalidDirection | P::InvalidOptions { .. } | P::PlaneFrameMismatch => {
                let mut structured = Self::invalid_argument(error.to_string(), None);
                structured
                    .details_mut()
                    .insert("kernelCode".to_string(), serde_json::Value::from(code));
                structured
            }
            P::Operations(operations) => {
                let mut structured = Self::from(operations);
                structured
                    .details_mut()
                    .insert("kernelCode".to_string(), serde_json::Value::from(code));
                structured
            }
            other => {
                let mut structured = Self::operation_failed(other.to_string());
                structured
                    .details_mut()
                    .insert("kernelCode".to_string(), serde_json::Value::from(code));
                structured
            }
        }
    }
}

/// Parse the §8 `ProjectCurveOptions` object (`None` means defaults).
fn parse_project_options(
    value: Option<&serde_json::Value>,
) -> Result<ProjectCurveOptions, StructuredWasmError> {
    let Some(options) = value else {
        return Ok(ProjectCurveOptions::default());
    };
    if options.is_null() {
        return Ok(ProjectCurveOptions::default());
    }
    let object = options.as_object().ok_or_else(|| {
        StructuredWasmError::invalid_argument("'options' must be an object", Some("options"))
    })?;
    let allow_approximate = object
        .get("allowApproximate")
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(false);
    let approximation_tolerance = object
        .get("approximationTolerance")
        .and_then(serde_json::Value::as_f64);
    let max_control_points = object
        .get("maxControlPoints")
        .and_then(serde_json::Value::as_u64)
        .and_then(|n| usize::try_from(n).ok())
        .unwrap_or(remus_operations::project_curve::DEFAULT_MAX_CONTROL_POINTS);
    let plane_frame = object
        .get("planeFrame")
        .map(parse_sketch_frame)
        .transpose()?;
    Ok(ProjectCurveOptions {
        allow_approximate,
        approximation_tolerance,
        max_control_points,
        plane_frame,
    })
}

/// Parse a §8 `SketchFrame` (`origin`/`xAxis`/`normal` triples;
/// `y = normal × xAxis`).
fn parse_sketch_frame(value: &serde_json::Value) -> Result<Frame3, StructuredWasmError> {
    fn triple(value: &serde_json::Value, key: &str) -> Result<[f64; 3], StructuredWasmError> {
        let array = value
            .get(key)
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| {
                StructuredWasmError::invalid_argument(
                    format!("frame '{key}' must be a 3-component array"),
                    Some("frame"),
                )
            })?;
        if array.len() != 3 {
            return Err(StructuredWasmError::invalid_argument(
                format!("frame '{key}' must be a 3-component array"),
                Some("frame"),
            ));
        }
        let mut out = [0.0; 3];
        for (index, component) in out.iter_mut().enumerate() {
            *component = array[index].as_f64().ok_or_else(|| {
                StructuredWasmError::invalid_argument(
                    format!("frame '{key}' must hold numbers"),
                    Some("frame"),
                )
            })?;
        }
        Ok(out)
    }
    for key in ["origin", "xAxis", "normal"] {
        triple(value, key).map(|_| ())?;
    }
    let origin = triple(value, "origin")?;
    let x = triple(value, "xAxis")?;
    let normal = triple(value, "normal")?;
    for (key, components) in [("origin", origin), ("xAxis", x), ("normal", normal)] {
        if !components.iter().all(|v| v.is_finite()) {
            return Err(StructuredWasmError::invalid_argument(
                format!("frame '{key}' must be finite"),
                Some("frame"),
            ));
        }
    }
    let x_axis = Vec3::new(x[0], x[1], x[2]);
    let z_axis = Vec3::new(normal[0], normal[1], normal[2]);
    if x_axis.length() <= 0.0 || z_axis.length() <= 0.0 {
        return Err(StructuredWasmError::invalid_argument(
            "frame axes must be non-zero",
            Some("frame"),
        ));
    }
    let x_unit = x_axis * (1.0 / x_axis.length());
    let z_unit = z_axis * (1.0 / z_axis.length());
    Ok(Frame3 {
        origin: Point3::new(origin[0], origin[1], origin[2]),
        x: x_unit,
        y: z_unit.cross(x_unit),
        z: z_unit,
    })
}

fn point_2d(point: remus_math::vec::Point2) -> serde_json::Value {
    serde_json::json!([point.x(), point.y()])
}

fn plane_curve_json(curve: &PCurve) -> serde_json::Value {
    match curve.curve() {
        Curve2D::Line(line) => serde_json::json!({
            "kind": "line",
            "start": point_2d(line.origin()),
            "end": point_2d(line.origin() + line.direction() * (curve.t_end() - curve.t_start())),
        }),
        Curve2D::Circle(circle) => serde_json::json!({
            "kind": "circle",
            "center": point_2d(circle.center()),
            "radius": circle.radius(),
            "startAngle": curve.t_start(),
            "endAngle": curve.t_end(),
        }),
        Curve2D::Ellipse(ellipse) => serde_json::json!({
            "kind": "ellipse",
            "center": point_2d(ellipse.center()),
            "semiMajor": ellipse.semi_major(),
            "semiMinor": ellipse.semi_minor(),
            "rotation": ellipse.rotation(),
            "startAngle": curve.t_start(),
            "endAngle": curve.t_end(),
        }),
        Curve2D::Nurbs(nurbs) => serde_json::json!({
            "kind": "nurbs",
            "degree": nurbs.degree(),
            "knots": nurbs.knots(),
            "controlPoints": nurbs.control_points().iter().map(|p| point_2d(*p)).collect::<Vec<_>>(),
            "weights": nurbs.weights(),
        }),
    }
}

fn quality_json(quality: &ProjectionQuality) -> serde_json::Value {
    match quality {
        ProjectionQuality::Exact => serde_json::json!({ "kind": "exact" }),
        ProjectionQuality::Approximate { max_deviation } => serde_json::json!({
            "kind": "approximate",
            "maxDeviation": max_deviation,
        }),
    }
}

fn projected_curves_json(result: &ProjectedCurves) -> serde_json::Value {
    serde_json::json!({
        "edges": result.edges.iter().map(|edge| {
            let mut value = serde_json::json!({
                "edge": edge_id_to_u32(edge.edge),
                "face": face_id_to_u32(edge.face),
                "sourceStart": edge.source_range.0,
                "sourceEnd": edge.source_range.1,
            });
            if let Some(curve) = &edge.plane_curve {
                value["planeCurve"] = plane_curve_json(curve);
            }
            value
        }).collect::<Vec<_>>(),
        "face": face_id_to_u32(result.face),
        "quality": quality_json(&result.quality),
        "clipped": result.clipped,
    })
}

fn solid_projection_json(result: &SolidProjection) -> serde_json::Value {
    serde_json::json!({
        "sources": result.sources.iter().map(|source| {
            serde_json::json!({
                "source": edge_id_to_u32(source.source),
                "edges": source.edges.iter().map(|edge| {
                    let mut value = serde_json::json!({
                        "edge": edge_id_to_u32(edge.edge),
                        "face": face_id_to_u32(edge.face),
                        "sourceStart": edge.source_range.0,
                        "sourceEnd": edge.source_range.1,
                    });
                    if let Some(curve) = &edge.plane_curve {
                        value["planeCurve"] = plane_curve_json(curve);
                    }
                    value
                }).collect::<Vec<_>>(),
                "clipped": source.clipped,
            })
        }).collect::<Vec<_>>(),
        "quality": quality_json(&result.quality),
    })
}

impl BrepKernel {
    fn project_curve_onto_face_json(
        &mut self,
        edge: u32,
        direction: [f64; 3],
        face: u32,
        options: Option<&serde_json::Value>,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let edge_id = self.resolve_edge(edge).map_err(StructuredWasmError::from)?;
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let options = parse_project_options(options)?;
        let result = remus_operations::project_curve::project_curve_onto_face(
            self.topo_mut(),
            edge_id,
            Vec3::new(direction[0], direction[1], direction[2]),
            face_id,
            &options,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(projected_curves_json(&result))
    }

    fn project_curves_onto_solid_json(
        &mut self,
        edges: &[u32],
        direction: [f64; 3],
        solid: u32,
        options: Option<&serde_json::Value>,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let edge_ids = edges
            .iter()
            .map(|handle| {
                self.resolve_edge(*handle)
                    .map_err(StructuredWasmError::from)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let options = parse_project_options(options)?;
        let result = remus_operations::project_curve::project_curves_onto_solid(
            self.topo_mut(),
            &edge_ids,
            Vec3::new(direction[0], direction[1], direction[2]),
            solid_id,
            &options,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(solid_projection_json(&result))
    }

    fn project_curves_onto_sketch_plane_json(
        &self,
        edges: &[u32],
        direction: [f64; 3],
        frame: &serde_json::Value,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let edge_ids = edges
            .iter()
            .map(|handle| {
                self.resolve_edge(*handle)
                    .map_err(StructuredWasmError::from)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let frame = parse_sketch_frame(frame)?;
        let curves = remus_operations::project_curve::project_curves_onto_plane(
            self.topo(),
            &edge_ids,
            Vec3::new(direction[0], direction[1], direction[2]),
            &frame,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(curves.iter().map(plane_curve_json).collect())
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
                let edge = get_u32(args, "edge")?;
                let face = get_u32(args, "face")?;
                self.project_curve_onto_face_json(edge, direction()?, face, args.get("options"))
            })()),
            "projectCurvesOntoSolid" => Some((|| {
                let edges = get_u32_array(args, "edges")?;
                let solid = get_u32(args, "solid")?;
                self.project_curves_onto_solid_json(
                    &edges,
                    direction()?,
                    solid,
                    args.get("options"),
                )
            })()),
            "projectCurvesOntoSketchPlane" => Some((|| {
                let edges = get_u32_array(args, "edges")?;
                let frame = args.get("frame").ok_or_else(|| {
                    StructuredWasmError::invalid_argument("missing 'frame'", Some("frame"))
                })?;
                self.project_curves_onto_sketch_plane_json(&edges, direction()?, frame)
            })()),
            _ => None,
        }
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Project one edge along a direction onto a face's trimmed region.
    ///
    /// Returns the §8 `ProjectedCurves` JSON (`edges` with `edge`, `face`,
    /// `sourceStart`/`sourceEnd` and optional `planeCurve`; `quality` with
    /// `kind` `"exact"` or `"approximate"` plus `maxDeviation`; `clipped`).
    /// `optionsJson` carries the §8 `ProjectCurveOptions` object (or is
    /// `None`/`undefined` for defaults). Refusals reject with a
    /// `project-curve:<code>` kernel code.
    #[wasm_bindgen(js_name = "projectCurveOntoFace")]
    pub fn project_curve_onto_face_js(
        &mut self,
        edge: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        face: u32,
        options_json: Option<String>,
    ) -> Result<String, JsError> {
        let edge_id = self.resolve_edge(edge)?;
        let face_id = self.resolve_face(face)?;
        let options: Option<serde_json::Value> = options_json
            .map(|text| serde_json::from_str(&text))
            .transpose()
            .map_err(|error| {
                StructuredWasmError::invalid_argument(
                    format!("invalid options JSON: {error}"),
                    Some("optionsJson"),
                )
            })
            .map_err(structured_to_js)?;
        let options = parse_project_options(options.as_ref()).map_err(structured_to_js)?;
        let result = remus_operations::project_curve::project_curve_onto_face(
            self.topo_mut(),
            edge_id,
            Vec3::new(dir_x, dir_y, dir_z),
            face_id,
            &options,
        )
        .map_err(project_curve_to_js)?;
        Ok(projected_curves_json(&result).to_string())
    }

    /// Project edges along a direction onto a solid's first-hit faces.
    ///
    /// Returns the §8 `SolidProjection` JSON. Atomic: a per-source refusal
    /// rejects the whole call and creates nothing.
    #[wasm_bindgen(js_name = "projectCurvesOntoSolid")]
    pub fn project_curves_onto_solid_js(
        &mut self,
        edges: Vec<u32>,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        solid: u32,
        options_json: Option<String>,
    ) -> Result<String, JsError> {
        let edge_ids = edges
            .iter()
            .map(|handle| self.resolve_edge(*handle))
            .collect::<Result<Vec<_>, _>>()?;
        let solid_id = self.resolve_solid(solid)?;
        let options: Option<serde_json::Value> = options_json
            .map(|text| serde_json::from_str(&text))
            .transpose()
            .map_err(|error| {
                StructuredWasmError::invalid_argument(
                    format!("invalid options JSON: {error}"),
                    Some("optionsJson"),
                )
            })
            .map_err(structured_to_js)?;
        let options = parse_project_options(options.as_ref()).map_err(structured_to_js)?;
        let result = remus_operations::project_curve::project_curves_onto_solid(
            self.topo_mut(),
            &edge_ids,
            Vec3::new(dir_x, dir_y, dir_z),
            solid_id,
            &options,
        )
        .map_err(project_curve_to_js)?;
        Ok(solid_projection_json(&result).to_string())
    }

    /// Project edges along a direction onto an unbounded sketch plane.
    ///
    /// `frameJson` carries the §8 `SketchFrame` object. Returns the §8
    /// `PlaneCurve2d` array (read-only: the topology is untouched).
    #[wasm_bindgen(js_name = "projectCurvesOntoSketchPlane")]
    pub fn project_curves_onto_sketch_plane_js(
        &self,
        edges: Vec<u32>,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        frame_json: String,
    ) -> Result<String, JsError> {
        let edge_ids = edges
            .iter()
            .map(|handle| self.resolve_edge(*handle))
            .collect::<Result<Vec<_>, _>>()?;
        let frame: serde_json::Value = serde_json::from_str(&frame_json)
            .map_err(|error| {
                StructuredWasmError::invalid_argument(
                    format!("invalid frame JSON: {error}"),
                    Some("frameJson"),
                )
            })
            .map_err(structured_to_js)?;
        let frame = parse_sketch_frame(&frame).map_err(structured_to_js)?;
        let curves = remus_operations::project_curve::project_curves_onto_plane(
            self.topo(),
            &edge_ids,
            Vec3::new(dir_x, dir_y, dir_z),
            &frame,
        )
        .map_err(project_curve_to_js)?;
        Ok(curves
            .iter()
            .map(plane_curve_json)
            .collect::<serde_json::Value>()
            .to_string())
    }
}

fn structured_to_js(error: StructuredWasmError) -> JsError {
    JsError::new(error.message())
}

/// Direct-method refusal: the `project-curve:<code>` kernel code prefixes
/// the message (the blend-family precedent), so JS callers can branch
/// without parsing prose.
fn project_curve_to_js(error: ProjectCurveError) -> JsError {
    JsError::new(&format!("project-curve:{}: {error}", error.code()))
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
    fn direct_and_batch_projection_agree() {
        let mut kernel = BrepKernel::new();
        let (_solid, face) = box_top_face(&mut kernel);
        let made = batch(
            &mut kernel,
            r#"[{"op":"makeLineEdge","args":{"x1":1.0,"y1":2.0,"z1":7.0,"x2":8.0,"y2":5.0,"z2":9.0}}]"#,
        );
        let edge = made[0]["ok"].as_u64().expect("edge") as u32;
        let direct: serde_json::Value = serde_json::from_str(
            &kernel
                .project_curve_onto_face_js(edge, 0.3, -0.2, -1.0, face, None)
                .expect("direct projection"),
        )
        .expect("direct parses");
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
}
