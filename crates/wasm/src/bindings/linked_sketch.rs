//! Bounded linked planar sketch inputs (P-Class 6.6, core tier).
#![allow(clippy::missing_errors_doc)]

use remus_math::frame::Frame3;
use remus_math::vec::{Point3, Vec3};
use remus_operations::linked_sketch::{
    ExternalCircleReference, ExternalEllipseReference, ExternalLineReference, LinkedSketchError,
    PlanarSketchAttachment,
};
use remus_topology::naming::PersistentRef;
use serde::{Deserialize, Serialize};
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use crate::error::{StructuredWasmError, WasmError};
use crate::helpers::get_u32;
use crate::kernel::BrepKernel;
use crate::state::GcsSketchState;
use crate::types::SketchFrame;

/// Fixed GCS inputs derived from one linked model edge.
#[derive(Debug, Clone)]
pub struct ExternalLine {
    pub reference: ExternalLineReference,
    pub start: remus_sketch::PointId,
    pub end: remus_sketch::PointId,
}

/// A fixed center and a true fixed-radius GCS circle from construction history.
#[derive(Debug, Clone)]
pub struct ExternalCircle {
    pub reference: ExternalCircleReference,
    pub center: remus_sketch::PointId,
    pub circle: remus_sketch::CircleId,
}

/// A fixed center and three externally owned ellipse scalar parameters.
#[derive(Debug, Clone)]
pub struct ExternalEllipse {
    pub reference: ExternalEllipseReference,
    pub center: remus_sketch::PointId,
    pub ellipse: remus_sketch::EllipseId,
}

/// Ordinary constraint handles whose ellipse geometry is read-only.
#[derive(Debug, Clone, Serialize, Deserialize, tsify::Tsify)]
#[serde(rename_all = "camelCase")]
pub struct ExternalEllipseHandles {
    pub ellipse: u32,
    pub center: u32,
}

/// Ordinary constraint handles whose external center/radius are read-only.
#[derive(Debug, Clone, Serialize, Deserialize, tsify::Tsify)]
#[serde(rename_all = "camelCase")]
pub struct ExternalCircleHandles {
    pub circle: u32,
    pub center: u32,
}

/// Handles for ordinary GCS constraints; the two point inputs are read-only.
#[derive(Debug, Clone, Serialize, Deserialize, tsify::Tsify)]
#[serde(rename_all = "camelCase")]
pub struct ExternalLineHandles {
    pub line: u32,
    pub start: u32,
    pub end: u32,
}

/// Current attached plane and counts of refreshed external inputs.
#[derive(Debug, Serialize, Deserialize, tsify::Tsify)]
#[serde(rename_all = "camelCase")]
pub struct LinkedSketchResolution {
    pub frame: SketchFrame,
    pub external_lines: usize,
    #[serde(default)]
    pub external_circles: usize,
    #[serde(default)]
    pub external_ellipses: usize,
}

fn linked_error(error: LinkedSketchError) -> StructuredWasmError {
    let mut result = StructuredWasmError::linked_sketch_failure(&error);
    result.details_mut().insert(
        "kernelCode".into(),
        format!("linked-sketch:{}", error.code()).into(),
    );
    if let LinkedSketchError::Unbound { role, resolution } = &error {
        result
            .details_mut()
            .insert("referenceRole".into(), (*role).into());
        result.details_mut().insert(
            "resolution".into(),
            super::naming::resolution_json(resolution),
        );
    }
    result
}

impl From<LinkedSketchError> for StructuredWasmError {
    fn from(error: LinkedSketchError) -> Self {
        linked_error(error)
    }
}

fn invalid(message: impl Into<String>) -> StructuredWasmError {
    StructuredWasmError::invalid_argument(message, None)
}

fn reference(value: &str) -> Result<PersistentRef, StructuredWasmError> {
    remus_io::naming_io::deserialize_persistent_ref(value.as_bytes())
        .map_err(StructuredWasmError::from)
}

fn native_frame(frame: &SketchFrame) -> Frame3 {
    let x = Vec3::new(frame.x_axis[0], frame.x_axis[1], frame.x_axis[2]);
    let z = Vec3::new(frame.normal[0], frame.normal[1], frame.normal[2]);
    Frame3 {
        origin: Point3::new(frame.origin[0], frame.origin[1], frame.origin[2]),
        x,
        y: z.cross(x),
        z,
    }
}

fn frame_json(frame: Frame3) -> SketchFrame {
    SketchFrame {
        origin: [frame.origin.x(), frame.origin.y(), frame.origin.z()],
        x_axis: [frame.x.x(), frame.x.y(), frame.x.z()],
        normal: [frame.z.x(), frame.z.y(), frame.z.z()],
    }
}

impl BrepKernel {
    fn linked_state(&self, sketch: u32) -> Result<&GcsSketchState, StructuredWasmError> {
        self.gcs_sketches
            .get(sketch as usize)
            .ok_or_else(|| invalid("invalid gcs sketch handle"))
    }

    fn attach_sketch_impl(
        &mut self,
        sketch: u32,
        face_ref: &str,
        frame: &SketchFrame,
    ) -> Result<LinkedSketchResolution, StructuredWasmError> {
        let state = self.linked_state(sketch)?;
        if state.attachment.is_some() {
            return Err(invalid("sketch already has a linked attachment"));
        }
        let attachment =
            PlanarSketchAttachment::new(self.topo(), reference(face_ref)?, native_frame(frame))
                .map_err(linked_error)?;
        let resolved = attachment
            .resolve_frame(self.topo())
            .map_err(linked_error)?;
        self.gcs_sketches
            .get_mut(sketch as usize)
            .ok_or_else(|| invalid("invalid gcs sketch handle"))?
            .attachment = Some(attachment);
        Ok(LinkedSketchResolution {
            frame: frame_json(resolved),
            external_lines: 0,
            external_circles: 0,
            external_ellipses: 0,
        })
    }

    fn add_external_line_impl(
        &mut self,
        sketch: u32,
        edge_ref: &str,
        start_ref: &str,
        end_ref: &str,
    ) -> Result<ExternalLineHandles, StructuredWasmError> {
        let state = self.linked_state(sketch)?;
        let attachment = state
            .attachment
            .as_ref()
            .ok_or_else(|| invalid("sketch has no linked attachment"))?;
        let reference = ExternalLineReference {
            edge: reference(edge_ref)?,
            start: reference(start_ref)?,
            end: reference(end_ref)?,
        };
        let image = attachment
            .resolve_line(self.topo(), &reference)
            .map_err(linked_error)?;
        // Clone before publication: even handle exhaustion or GCS validation
        // failure must leave the last valid sketch and its namespace intact.
        let mut next = state.clone();
        let start_handle =
            crate::state::next_handle(next.points.len()).map_err(StructuredWasmError::from)?;
        let end_handle = crate::state::next_handle(
            next.points
                .len()
                .checked_add(1)
                .ok_or_else(|| invalid("point handle namespace exhausted"))?,
        )
        .map_err(StructuredWasmError::from)?;
        let line_handle =
            crate::state::next_handle(next.lines.len()).map_err(StructuredWasmError::from)?;
        let start = next
            .sys
            .add_point(remus_sketch::PointData {
                x: image.start.x(),
                y: image.start.y(),
                fixed: true,
            })
            .map_err(WasmError::from)
            .map_err(StructuredWasmError::from)?;
        let end = next
            .sys
            .add_point(remus_sketch::PointData {
                x: image.end.x(),
                y: image.end.y(),
                fixed: true,
            })
            .map_err(WasmError::from)
            .map_err(StructuredWasmError::from)?;
        let line = next
            .sys
            .add_line(start, end)
            .map_err(WasmError::from)
            .map_err(StructuredWasmError::from)?;
        next.points.extend([Some(start), Some(end)]);
        next.lines.push(Some(line));
        next.external_lines.push(ExternalLine {
            reference,
            start,
            end,
        });
        *self
            .gcs_sketches
            .get_mut(sketch as usize)
            .ok_or_else(|| invalid("invalid gcs sketch handle"))? = next;
        Ok(ExternalLineHandles {
            line: line_handle,
            start: start_handle,
            end: end_handle,
        })
    }

    fn resolve_sketch_refs_impl(
        &mut self,
        sketch: u32,
    ) -> Result<LinkedSketchResolution, StructuredWasmError> {
        self.resolve_sketch_refs_native(sketch)
            .map_err(StructuredWasmError::from)
    }

    fn add_external_circle_impl(
        &mut self,
        sketch: u32,
        edge_ref: &str,
    ) -> Result<ExternalCircleHandles, StructuredWasmError> {
        let state = self.linked_state(sketch)?;
        let attachment = state
            .attachment
            .as_ref()
            .ok_or_else(|| invalid("sketch has no linked attachment"))?;
        let reference = ExternalCircleReference {
            edge: reference(edge_ref)?,
        };
        let image = attachment
            .resolve_circle(self.topo(), &reference)
            .map_err(linked_error)?;
        let mut next = state.clone();
        let center_handle =
            crate::state::next_handle(next.points.len()).map_err(StructuredWasmError::from)?;
        let circle_handle =
            crate::state::next_handle(next.circles.len()).map_err(StructuredWasmError::from)?;
        let center = next
            .sys
            .add_point(remus_sketch::PointData {
                x: image.center.x(),
                y: image.center.y(),
                fixed: true,
            })
            .map_err(WasmError::from)?;
        let circle = next
            .sys
            .add_circle_with_fixed_radius(center, image.radius)
            .map_err(WasmError::from)?;
        next.points.push(Some(center));
        next.circles.push(Some(circle));
        next.external_circles.push(ExternalCircle {
            reference,
            center,
            circle,
        });
        *self
            .gcs_sketches
            .get_mut(sketch as usize)
            .ok_or_else(|| invalid("invalid gcs sketch handle"))? = next;
        Ok(ExternalCircleHandles {
            circle: circle_handle,
            center: center_handle,
        })
    }

    fn add_external_ellipse_impl(
        &mut self,
        sketch: u32,
        edge_ref: &str,
    ) -> Result<ExternalEllipseHandles, StructuredWasmError> {
        let state = self.linked_state(sketch)?;
        let attachment = state
            .attachment
            .as_ref()
            .ok_or_else(|| invalid("sketch has no linked attachment"))?;
        let reference = ExternalEllipseReference {
            edge: reference(edge_ref)?,
        };
        let image = attachment
            .resolve_ellipse(self.topo(), &reference)
            .map_err(linked_error)?;
        let mut next = state.clone();
        let center_handle =
            crate::state::next_handle(next.points.len()).map_err(StructuredWasmError::from)?;
        let ellipse_handle =
            crate::state::next_handle(next.ellipses.len()).map_err(StructuredWasmError::from)?;
        let center = next
            .sys
            .add_point(remus_sketch::PointData {
                x: image.center.x(),
                y: image.center.y(),
                fixed: true,
            })
            .map_err(WasmError::from)?;
        let ellipse = next
            .sys
            .add_ellipse_with_fixed_parameters(center, image.a, image.b, image.angle)
            .map_err(WasmError::from)?;
        next.points.push(Some(center));
        next.ellipses.push(Some(ellipse));
        next.external_ellipses.push(ExternalEllipse {
            reference,
            center,
            ellipse,
        });
        *self
            .gcs_sketches
            .get_mut(sketch as usize)
            .ok_or_else(|| invalid("invalid gcs sketch handle"))? = next;
        Ok(ExternalEllipseHandles {
            ellipse: ellipse_handle,
            center: center_handle,
        })
    }

    fn resolve_sketch_refs_native(
        &mut self,
        sketch: u32,
    ) -> Result<LinkedSketchResolution, WasmError> {
        let state = self
            .gcs_sketches
            .get(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "gcs sketch",
                index: sketch as usize,
            })?;
        let attachment = state
            .attachment
            .as_ref()
            .ok_or_else(|| WasmError::InvalidInput {
                reason: "sketch has no linked attachment".into(),
            })?;
        let frame = attachment
            .resolve_frame(self.topo())
            .map_err(WasmError::from)?;
        let mut next = state.clone();
        for external in &state.external_lines {
            let image = attachment
                .resolve_line(self.topo(), &external.reference)
                .map_err(WasmError::from)?;
            for (id, point) in [(external.start, image.start), (external.end, image.end)] {
                let data = next
                    .sys
                    .point_mut(id)
                    .ok_or_else(|| WasmError::InvalidInput {
                        reason: "external sketch point was removed".into(),
                    })?;
                data.x = point.x();
                data.y = point.y();
                data.fixed = true;
            }
        }
        for external in &state.external_circles {
            let image = attachment.resolve_circle(self.topo(), &external.reference)?;
            next.sys
                .set_fixed_circle_radius(external.circle, image.radius)
                .map_err(WasmError::from)?;
            let center =
                next.sys
                    .point_mut(external.center)
                    .ok_or_else(|| WasmError::InvalidInput {
                        reason: "external sketch circle center was removed".into(),
                    })?;
            center.x = image.center.x();
            center.y = image.center.y();
            center.fixed = true;
        }
        for external in &state.external_ellipses {
            let image = attachment.resolve_ellipse(self.topo(), &external.reference)?;
            next.sys
                .set_fixed_ellipse_parameters(external.ellipse, image.a, image.b, image.angle)
                .map_err(WasmError::from)?;
            let center =
                next.sys
                    .point_mut(external.center)
                    .ok_or_else(|| WasmError::InvalidInput {
                        reason: "external sketch ellipse center was removed".into(),
                    })?;
            center.x = image.center.x();
            center.y = image.center.y();
            center.fixed = true;
        }
        let external_lines = state.external_lines.len();
        let external_circles = state.external_circles.len();
        let external_ellipses = state.external_ellipses.len();
        *self
            .gcs_sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "gcs sketch",
                index: sketch as usize,
            })? = next;
        Ok(LinkedSketchResolution {
            frame: frame_json(frame),
            external_lines,
            external_circles,
            external_ellipses,
        })
    }

    /// A thrown solver error restores the sketch state before input refresh.
    /// Ordinary nonconverged solver outcomes retain their existing semantics.
    pub(crate) fn with_linked_sketch_inputs<T>(
        &mut self,
        sketch: u32,
        solve: impl FnOnce(&mut Self) -> Result<T, WasmError>,
    ) -> Result<T, WasmError> {
        let saved = self
            .gcs_sketches
            .get(sketch as usize)
            .filter(|state| state.attachment.is_some())
            .cloned();
        let result = self
            .refresh_linked_sketch_for_solve(sketch)
            .and_then(|()| solve(self));
        if result.is_err()
            && let Some(saved) = saved
            && let Some(state) = self.gcs_sketches.get_mut(sketch as usize)
        {
            *state = saved;
        }
        result
    }

    /// Existing solve entry points refresh linked inputs before evaluating.
    pub(crate) fn refresh_linked_sketch_for_solve(&mut self, sketch: u32) -> Result<(), WasmError> {
        if self
            .gcs_sketches
            .get(sketch as usize)
            .is_some_and(|state| state.attachment.is_some())
        {
            self.resolve_sketch_refs_native(sketch)?;
        }
        Ok(())
    }

    pub(crate) fn dispatch_linked_sketch_op(
        &mut self,
        op: &str,
        args: &serde_json::Value,
    ) -> Option<Result<serde_json::Value, StructuredWasmError>> {
        let result =
            match op {
                "gcsAttachToFace" => (|| {
                    let sketch = get_u32(args, "sketch")?;
                    let reference = args["ref"]
                        .as_str()
                        .ok_or_else(|| invalid("ref must be a serialized persistent reference"))?;
                    let frame: SketchFrame = serde_json::from_value(args["frame"].clone())
                        .map_err(|e| invalid(e.to_string()))?;
                    serde_json::to_value(self.attach_sketch_impl(sketch, reference, &frame)?)
                        .map_err(|e| invalid(e.to_string()))
                })(),
                "gcsAddExternalLine" => (|| {
                    let sketch = get_u32(args, "sketch")?;
                    let reference = args["ref"]
                        .as_str()
                        .ok_or_else(|| invalid("ref must be a serialized persistent reference"))?;
                    serde_json::to_value(self.add_external_line_impl(
                        sketch,
                        reference,
                        args["startRef"].as_str().ok_or_else(|| {
                            invalid("startRef must be a serialized vertex reference")
                        })?,
                        args["endRef"].as_str().ok_or_else(|| {
                            invalid("endRef must be a serialized vertex reference")
                        })?,
                    )?)
                    .map_err(|e| invalid(e.to_string()))
                })(),
                "gcsAddExternalCircle" => (|| {
                    let sketch = get_u32(args, "sketch")?;
                    let reference = args["ref"]
                        .as_str()
                        .ok_or_else(|| invalid("ref must be a serialized persistent reference"))?;
                    serde_json::to_value(self.add_external_circle_impl(sketch, reference)?)
                        .map_err(|e| invalid(e.to_string()))
                })(),
                "gcsAddExternalEllipse" => (|| {
                    let sketch = get_u32(args, "sketch")?;
                    let reference = args["ref"]
                        .as_str()
                        .ok_or_else(|| invalid("ref must be a serialized persistent reference"))?;
                    serde_json::to_value(self.add_external_ellipse_impl(sketch, reference)?)
                        .map_err(|e| invalid(e.to_string()))
                })(),
                "gcsResolveRefs" => get_u32(args, "sketch").and_then(|sketch| {
                    serde_json::to_value(self.resolve_sketch_refs_impl(sketch)?)
                        .map_err(|e| invalid(e.to_string()))
                }),
                _ => return None,
            };
        Some(result)
    }
}

fn direct_message(error: StructuredWasmError) -> String {
    let message = error.message().to_owned();
    let (code, _, _) = error.into_direct_parts();
    if message.starts_with(&format!("{code}:")) {
        message
    } else {
        format!("{code}: {message}")
    }
}

fn js_error(error: StructuredWasmError) -> JsError {
    JsError::new(&direct_message(error))
}

#[cfg(test)]
mod ellipse_tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::float_cmp
    )]
    use super::*;
    use remus_math::mat::Mat4;
    use remus_operations::journal_ops::{
        begin_scoped, extrude_journaled, move_faces_journaled, solid_entity_keys,
    };
    use remus_topology::builder::{make_ellipse_edge_with_ref, make_planar_face_from_wire};
    use remus_topology::explorer::{solid_edges, solid_faces};
    use remus_topology::face::{FaceId, FaceSurface};
    use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft, OpId};
    use remus_topology::naming::{Resolution, resolve};
    use remus_topology::solid::SolidId;
    use remus_topology::wire::{OrientedEdge, Wire};
    use serde_json::{Value, json};

    struct Fixture {
        k: BrepKernel,
        sketch: u32,
        source_solid: SolidId,
        attachment_solid: SolidId,
        attachment_face: FaceId,
        source: String,
        second_source: String,
        external: ExternalEllipseHandles,
    }

    fn row(k: &mut BrepKernel, v2: bool, op: &str, args: Value) -> Value {
        let program = json!([{"op":op,"args":args}]).to_string();
        let text = if v2 {
            k.execute_batch_v2(&program)
        } else {
            k.execute_batch(&program)
        };
        serde_json::from_str::<Vec<Value>>(&text).unwrap().remove(0)
    }
    fn call(k: &mut BrepKernel, v2: bool, op: &str, args: Value) -> Value {
        let result = row(k, v2, op, args);
        assert!(result.get("ok").is_some(), "{result}");
        result["ok"].clone()
    }
    fn output(k: &BrepKernel, op: OpId, key: EntityKey) -> String {
        let reference = (0..100).map(|i| PersistentRef::operation_output(op, key.kind, i))
            .find(|r| matches!(resolve(k.topo(),r), Resolution::Bound { entity, .. } if entity == key)).unwrap();
        String::from_utf8(remus_io::naming_io::serialize_persistent_ref(&reference).unwrap())
            .unwrap()
    }
    fn fixture(v2: bool) -> Fixture {
        let mut k = BrepKernel::new();
        let boxed = remus_operations::primitives::make_box(k.topo_mut(), 10.0, 8.0, 4.0).unwrap();
        let edge = make_ellipse_edge_with_ref(
            k.topo_mut(),
            Point3::new(5.0, 4.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            2.0,
            Vec3::new(0.37_f64.cos(), 0.37_f64.sin(), 0.0),
            1e-7,
        )
        .unwrap();
        let wire = k
            .topo_mut()
            .add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        let face = make_planar_face_from_wire(k.topo_mut(), wire).unwrap();
        let seeded = extrude_journaled(k.topo_mut(), face, Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
        let source_edge = solid_edges(k.topo(), seeded.solid)
            .unwrap()
            .into_iter()
            .find(|&e| {
                matches!(
                    k.topo().edge(e).unwrap().curve(),
                    remus_topology::edge::EdgeCurve::Ellipse(_)
                )
            })
            .unwrap();
        let source = output(&k, seeded.op, EntityKey::edge(source_edge.index()));
        let second_edge = solid_edges(k.topo(), seeded.solid)
            .unwrap()
            .into_iter()
            .find(|&e| {
                e != source_edge
                    && matches!(
                        k.topo().edge(e).unwrap().curve(),
                        remus_topology::edge::EdgeCurve::Ellipse(_)
                    )
            })
            .unwrap();
        let second_source = output(&k, seeded.op, EntityKey::edge(second_edge.index()));
        let top = solid_faces(k.topo(), boxed).unwrap().into_iter().find(|&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.99)).unwrap();
        let attachment = move_faces_journaled(k.topo_mut(), boxed, &[top], 1.0).unwrap();
        let top = solid_faces(k.topo(), attachment.solid).unwrap().into_iter().find(|&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.99)).unwrap();
        let face_ref = output(&k, attachment.op, EntityKey::face(top.index()));
        let sketch = call(&mut k, v2, "gcsNew", json!({})).as_u64().unwrap() as u32;
        call(
            &mut k,
            v2,
            "gcsAttachToFace",
            json!({"sketch":sketch,"ref":face_ref,"frame":{"origin":[0,0,5],"xAxis":[1,0,0],"normal":[0,0,1]}}),
        );
        let external = serde_json::from_value(call(
            &mut k,
            v2,
            "gcsAddExternalEllipse",
            json!({"sketch":sketch,"ref":source}),
        ))
        .unwrap();
        Fixture {
            k,
            sketch,
            source_solid: seeded.solid,
            attachment_solid: attachment.solid,
            attachment_face: top,
            source,
            second_source,
            external,
        }
    }
    fn params(f: &Fixture) -> [f64; 5] {
        f.k.gcs_ellipse_params_impl(f.sketch, f.external.ellipse)
            .unwrap()
    }

    // A native caller owns this explicit construction operation. Public raw
    // transforms do not acquire this history merely by preserving handles.
    fn authoritative_growth(f: &mut Fixture) {
        let keys = solid_entity_keys(f.k.topo(), f.source_solid).unwrap();
        let pending = begin_scoped(
            f.k.topo_mut(),
            "fixture_native_similarity",
            &[f.source_solid],
        )
        .unwrap();
        remus_operations::transform::transform_solid(
            f.k.topo_mut(),
            f.source_solid,
            &Mat4([
                [1.2, 0.0, 0.0, 1.0],
                [0.0, 1.2, 0.0, -1.0],
                [0.0, 0.0, 1.2, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        )
        .unwrap();
        assert_eq!(solid_entity_keys(f.k.topo(), f.source_solid).unwrap(), keys);
        let mut draft = EvolutionDraft::construction();
        for key in keys {
            draft.push(key, EventDraft::Modified { from: key });
        }
        f.k.topo_mut()
            .journal_record_evolution(pending, draft)
            .unwrap();
    }

    #[test]
    fn linked_ellipse_batches_constraints_read_only_inputs_and_undo() {
        for v2 in [false, true] {
            let mut f = fixture(v2);
            assert_eq!(
                call(&mut f.k, v2, "gcsDof", json!({"sketch":f.sketch}))["numParams"],
                0
            );
            let free = call(
                &mut f.k,
                v2,
                "gcsAddPoint",
                json!({"sketch":f.sketch,"x":6,"y":5,"fixed":false}),
            );
            let consumer = call(
                &mut f.k,
                v2,
                "gcsAddEllipse",
                json!({"sketch":f.sketch,"center":free,"a":1,"b":0.5,"angle":0}),
            );
            for constraint in [
                json!({"type":"concentricEllipseEllipse","ellipse1":f.external.ellipse,"ellipse2":consumer}),
                json!({"type":"equalEllipseRadii","ellipse1":f.external.ellipse,"ellipse2":consumer}),
                json!({"type":"ellipseAngle","ellipse":consumer,"value":0.37}),
            ] {
                call(
                    &mut f.k,
                    v2,
                    "gcsAddConstraint",
                    json!({"sketch":f.sketch,"constraint":constraint}),
                );
            }
            let solved = call(
                &mut f.k,
                v2,
                "gcsSolveDetailed",
                json!({"sketch":f.sketch,"maxIterations":100,"tolerance":1e-10}),
            );
            assert_eq!(solved["converged"], true);
            assert_eq!(solved["numParams"], 5);
            assert_eq!(solved["rank"], 5);
            let cp = f.k.checkpoint_impl().unwrap();
            let retired: ExternalEllipseHandles = serde_json::from_value(call(
                &mut f.k,
                v2,
                "gcsAddExternalEllipse",
                json!({"sketch":f.sketch,"ref":f.source}),
            ))
            .unwrap();
            authoritative_growth(&mut f);
            let result = call(
                &mut f.k,
                v2,
                "gcsSolve",
                json!({"sketch":f.sketch,"maxIterations":100,"tolerance":1e-10}),
            );
            assert_eq!(result["converged"], true);
            let expected = [7.0_f64, 3.8, 3.6, 2.4, 0.37];
            for (got, want) in params(&f).into_iter().zip(expected) {
                assert!((got - want).abs() < 1e-9, "{got} vs {want}");
            }
            let consumer =
                f.k.gcs_ellipse_params_impl(f.sketch, consumer.as_u64().unwrap() as u32)
                    .unwrap();
            for (got, want) in consumer.into_iter().zip(expected) {
                assert!((got - want).abs() < 1e-8);
            }
            let before = params(&f);
            let set = row(
                &mut f.k,
                v2,
                "gcsSetEllipse",
                json!({"sketch":f.sketch,"ellipse":f.external.ellipse,"x":99,"y":99,"a":9,"b":8,"angle":1}),
            );
            if v2 {
                assert_eq!(set["error"]["category"], "invalid_input");
                assert_eq!(
                    set["error"]["details"]["kernelCode"],
                    "linked-sketch:read_only_external_point"
                );
            } else {
                assert!(
                    set["error"]
                        .as_str()
                        .unwrap()
                        .starts_with("linked-sketch:read_only_external_point:")
                );
            }
            assert_eq!(params(&f).map(f64::to_bits), before.map(f64::to_bits));
            f.k.restore_checkpoint_impl(cp).unwrap();
            assert!(
                f.k.gcs_ellipse_params_impl(f.sketch, retired.ellipse)
                    .is_err()
            );
            assert!(
                f.k.gcs_point_position_impl(f.sketch, retired.center)
                    .is_err()
            );
            let next: ExternalEllipseHandles = serde_json::from_value(call(
                &mut f.k,
                v2,
                "gcsAddExternalEllipse",
                json!({"sketch":f.sketch,"ref":f.source}),
            ))
            .unwrap();
            assert!(next.ellipse > retired.ellipse && next.center > retired.center);
            assert_eq!(
                f.k.gcs_sketches
                    .get(f.sketch as usize)
                    .unwrap()
                    .sys
                    .fixed_ellipse_parameter_count(),
                2
            );
            let moved = move_faces_journaled(
                f.k.topo_mut(),
                f.attachment_solid,
                &[f.attachment_face],
                1.0,
            )
            .unwrap();
            assert!(
                remus_operations::validate::validate_solid(f.k.topo(), moved.solid)
                    .unwrap()
                    .is_valid()
            );
            let refreshed = call(&mut f.k, v2, "gcsResolveRefs", json!({"sketch":f.sketch}));
            assert_eq!(refreshed["externalEllipses"], 2);
            assert_eq!(refreshed["frame"]["origin"][2], 6.0);
        }
    }

    #[test]
    fn linked_ellipse_refresh_is_atomic_for_mixed_failures_and_unjournaled_growth() {
        for split in [false, true] {
            let mut f = fixture(true);
            let retired: ExternalEllipseHandles = serde_json::from_value(call(
                &mut f.k,
                true,
                "gcsAddExternalEllipse",
                json!({"sketch":f.sketch,"ref":f.second_source}),
            ))
            .unwrap();
            let before = params(&f);
            // This typed journal failure occurs after the first input could
            // refresh. Separate construction refs ensure only the second fails.
            let second =
                f.k.gcs_sketches
                    .get(f.sketch as usize)
                    .unwrap()
                    .external_ellipses[1]
                    .reference
                    .edge
                    .clone();
            authoritative_growth(&mut f);
            let Resolution::Bound { entity, .. } = resolve(f.k.topo(), &second) else {
                panic!()
            };
            let pending = f.k.topo_mut().journal_begin("fixture_ellipse_refusal");
            let mut draft = EvolutionDraft::construction();
            if split {
                let id = f.k.topo().edge_id_from_index(entity.index).unwrap();
                let copy = f.k.topo().edge(id).unwrap().clone();
                let new = f.k.topo_mut().add_edge(copy);
                draft.push(entity, EventDraft::Modified { from: entity });
                draft.push(
                    EntityKey::edge(new.index()),
                    EventDraft::Modified { from: entity },
                );
            } else {
                draft.push(entity, EventDraft::Deleted);
            }
            f.k.topo_mut()
                .journal_record_evolution(pending, draft)
                .unwrap();
            let state = f.k.gcs_sketches.get(f.sketch as usize).unwrap();
            let refreshed_first = state
                .attachment
                .as_ref()
                .unwrap()
                .resolve_ellipse(f.k.topo(), &state.external_ellipses[0].reference)
                .unwrap();
            assert!((refreshed_first.a - 3.6).abs() < 1e-12);
            for op in ["gcsResolveRefs", "gcsSolve", "gcsSolveDetailed", "gcsDof"] {
                let result = row(
                    &mut f.k,
                    true,
                    op,
                    json!({"sketch":f.sketch,"maxIterations":100,"tolerance":1e-10}),
                );
                assert_eq!(
                    result["error"]["details"]["kernelCode"],
                    "linked-sketch:reference_unbound"
                );
                assert_eq!(
                    result["error"]["details"]["resolution"]["status"],
                    if split { "boundMany" } else { "dangling" }
                );
                assert_eq!(params(&f).map(f64::to_bits), before.map(f64::to_bits));
                assert_eq!(
                    f.k.gcs_ellipse_params_impl(f.sketch, retired.ellipse)
                        .unwrap()
                        .map(f64::to_bits),
                    before.map(f64::to_bits)
                );
            }
        }
        let mut f = fixture(true);
        let before = params(&f);
        remus_operations::transform::transform_solid(
            f.k.topo_mut(),
            f.source_solid,
            &Mat4([
                [1.2, 0.0, 0.0, 0.0],
                [0.0, 1.2, 0.0, 0.0],
                [0.0, 0.0, 1.2, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        )
        .unwrap();
        let refused = row(&mut f.k, true, "gcsResolveRefs", json!({"sketch":f.sketch}));
        assert_eq!(refused["error"]["details"]["resolution"]["status"], "stale");
        assert_eq!(params(&f).map(f64::to_bits), before.map(f64::to_bits));
    }

    #[test]
    fn linked_ellipse_resource_errors_restore_pre_refresh_state() {
        for op in ["gcsSolve", "gcsSolveDetailed", "gcsDof"] {
            let mut f = fixture(true);
            let state = f.k.gcs_sketches.get(f.sketch as usize).unwrap().clone();
            let mut limited = remus_sketch::GcsSystem::with_limits(remus_sketch::GcsLimits {
                max_dense_bytes: 1,
                max_dense_qr_work: usize::MAX,
            });
            for point in state.points.iter().flatten() {
                assert_eq!(
                    limited
                        .add_point(*state.sys.point(*point).unwrap())
                        .unwrap(),
                    *point
                );
            }
            for id in state.ellipses.iter().flatten() {
                let ellipse = state.sys.ellipse(*id).unwrap();
                assert_eq!(
                    limited
                        .add_ellipse_with_fixed_parameters(
                            ellipse.center,
                            ellipse.a,
                            ellipse.b,
                            ellipse.angle
                        )
                        .unwrap(),
                    *id
                );
            }
            f.k.gcs_sketches.get_mut(f.sketch as usize).unwrap().sys = limited;
            let free = call(
                &mut f.k,
                true,
                "gcsAddPoint",
                json!({"sketch":f.sketch,"x":8,"y":9,"fixed":false}),
            );
            call(
                &mut f.k,
                true,
                "gcsAddConstraint",
                json!({"sketch":f.sketch,"constraint":{"type":"fixX","point":free,"value":7}}),
            );
            let before = params(&f);
            authoritative_growth(&mut f);
            let result = row(
                &mut f.k,
                true,
                op,
                json!({"sketch":f.sketch,"maxIterations":100,"tolerance":1e-10}),
            );
            assert_eq!(
                result["error"]["code"], "resource_limit_exceeded",
                "{result}"
            );
            assert_eq!(result["error"]["category"], "resource_limit");
            assert_eq!(params(&f).map(f64::to_bits), before.map(f64::to_bits));
            assert_eq!(
                f.k.gcs_sketches
                    .get(f.sketch as usize)
                    .unwrap()
                    .sys
                    .fixed_ellipse_parameter_count(),
                1
            );
            assert_eq!(
                f.k.gcs_point_position_impl(f.sketch, free.as_u64().unwrap() as u32)
                    .unwrap(),
                [8.0, 9.0]
            );
        }
    }

    #[test]
    fn old_linked_resolution_json_defaults_the_ellipse_count() {
        let result:LinkedSketchResolution=serde_json::from_value(json!({"frame":{"origin":[0,0,4],"xAxis":[1,0,0],"normal":[0,0,1]},"externalLines":2,"externalCircles":3})).unwrap();
        assert_eq!(result.external_ellipses, 0);
        assert_eq!(serde_json::to_value(result).unwrap()["externalCircles"], 3);
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Attach an existing GCS sketch to one construction-resolved planar face.
    /// Frame axes must be unit and perpendicular; this slice supports parallel
    /// plane moves. The reference uses the existing versioned JSON codec.
    #[wasm_bindgen(js_name = "gcsAttachToFace")]
    pub fn gcs_attach_to_face(
        &mut self,
        sketch: u32,
        reference: &str,
        frame: tsify::Ts<SketchFrame>,
    ) -> Result<tsify::Ts<LinkedSketchResolution>, JsError> {
        let frame = frame.to_rust().map_err(|e| JsError::new(&e.to_string()))?;
        Ok(self
            .attach_sketch_impl(sketch, reference, &frame)
            .map_err(js_error)?
            .into_ts()?)
    }

    /// Add a fixed external line whose returned handles can be constrained to.
    #[wasm_bindgen(js_name = "gcsAddExternalLine")]
    pub fn gcs_add_external_line(
        &mut self,
        sketch: u32,
        reference: &str,
        start_ref: &str,
        end_ref: &str,
    ) -> Result<tsify::Ts<ExternalLineHandles>, JsError> {
        Ok(self
            .add_external_line_impl(sketch, reference, start_ref, end_ref)
            .map_err(js_error)?
            .into_ts()?)
    }

    /// Atomically refresh every external input; a failed rebind preserves all
    /// previous coordinates. Solve and detailed solve refresh automatically.
    #[wasm_bindgen(js_name = "gcsResolveRefs")]
    pub fn gcs_resolve_refs(
        &mut self,
        sketch: u32,
    ) -> Result<tsify::Ts<LinkedSketchResolution>, JsError> {
        Ok(self
            .resolve_sketch_refs_impl(sketch)
            .map_err(js_error)?
            .into_ts()?)
    }

    /// Add a complete construction-linked native circle in the parallel sketch
    /// plane. Both the center and radius are driven read-only inputs; returned
    /// handles can be used by ordinary GCS constraints. Partial/oblique curves
    /// and nonunique construction identity refuse atomically.
    #[wasm_bindgen(js_name = "gcsAddExternalCircle")]
    pub fn gcs_add_external_circle(
        &mut self,
        sketch: u32,
        reference: &str,
    ) -> Result<tsify::Ts<ExternalCircleHandles>, JsError> {
        Ok(self
            .add_external_circle_impl(sketch, reference)
            .map_err(js_error)?
            .into_ts()?)
    }

    /// Add a complete construction-linked native ellipse in the parallel sketch
    /// plane. Both the center and three scalar parameters are driven read-only inputs; returned
    /// handles can be used by ordinary GCS constraints. Partial/oblique curves
    /// and nonunique construction identity refuse atomically.
    #[wasm_bindgen(js_name = "gcsAddExternalEllipse")]
    pub fn gcs_add_external_ellipse(
        &mut self,
        sketch: u32,
        reference: &str,
    ) -> Result<tsify::Ts<ExternalEllipseHandles>, JsError> {
        Ok(self
            .add_external_ellipse_impl(sketch, reference)
            .map_err(js_error)?
            .into_ts()?)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
    use super::*;
    use remus_operations::journal_ops::{begin_scoped, solid_entity_keys};
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;
    use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft};
    use remus_topology::naming::{Resolution, resolve};

    fn batch(k: &mut BrepKernel, ops: serde_json::Value) -> Vec<serde_json::Value> {
        serde_json::from_str(&k.execute_batch(&ops.to_string())).unwrap()
    }

    fn encode(reference: &PersistentRef) -> String {
        String::from_utf8(remus_io::naming_io::serialize_persistent_ref(reference).unwrap())
            .unwrap()
    }

    fn assert_refusal(
        k: &mut BrepKernel,
        op: &str,
        args: serde_json::Value,
        kernel_code: &str,
        category: &str,
    ) -> serde_json::Value {
        let ops = serde_json::json!([{"op":op,"args":args}]);
        let out: serde_json::Value =
            serde_json::from_str(&k.execute_batch_v2(&ops.to_string())).unwrap();
        let error = &out[0]["error"];
        assert_eq!(error["category"], category, "{out}");
        assert_eq!(error["details"]["kernelCode"], kernel_code, "{out}");
        assert_eq!(error["details"]["operation"], op, "{out}");
        assert!(
            error["message"]
                .as_str()
                .unwrap()
                .starts_with(&format!("{kernel_code}:")),
            "{out}"
        );
        let legacy = batch(k, ops);
        assert!(
            legacy[0]["error"]
                .as_str()
                .unwrap()
                .starts_with(&format!("{kernel_code}:")),
            "{legacy:?}"
        );
        error.clone()
    }

    fn setup() -> (
        BrepKernel,
        u32,
        remus_topology::SolidId,
        remus_topology::FaceId,
        PersistentRef,
        ExternalLineHandles,
    ) {
        let mut k = BrepKernel::new();
        let solid = remus_operations::primitives::make_box(k.topo_mut(), 10.0, 8.0, 4.0).unwrap();
        let top = solid_faces(k.topo(), solid).unwrap().into_iter().find(|&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z()>0.99)).unwrap();
        let right = solid_faces(k.topo(), solid).unwrap().into_iter().find(|&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.x()>0.99)).unwrap();
        let edge = k
            .topo()
            .wire(k.topo().face(top).unwrap().outer_wire())
            .unwrap()
            .edges()[0]
            .edge();
        let keys = solid_entity_keys(k.topo(), solid).unwrap();
        let pending = begin_scoped(k.topo_mut(), "fixture_primitive", &[solid]).unwrap();
        let mut draft = EvolutionDraft::construction();
        for key in keys {
            draft.push(key, EventDraft::Generated { sources: vec![] });
        }
        let op = k
            .topo_mut()
            .journal_record_evolution(pending, draft)
            .unwrap();
        let reference = |key: EntityKey| {
            (0..100)
                .map(|index| PersistentRef::operation_output(op, key.kind, index))
                .find(|r| matches!(resolve(k.topo(),r),Resolution::Bound{entity,..}if entity==key))
                .unwrap()
        };
        let face_ref = reference(EntityKey::face(top.index()));
        let edge_ref = reference(EntityKey::edge(edge.index()));
        let start_ref = reference(EntityKey::vertex(
            k.topo().edge(edge).unwrap().start().index(),
        ));
        let end_ref = reference(EntityKey::vertex(
            k.topo().edge(edge).unwrap().end().index(),
        ));
        let out = batch(
            &mut k,
            serde_json::json!([
                {"op":"gcsNew","args":{}},
                {"op":"gcsAttachToFace","args":{"sketch":0,"ref":encode(&face_ref),"frame":{"origin":[0,0,4],"xAxis":[1,0,0],"normal":[0,0,1]}}},
                {"op":"gcsAddExternalLine","args":{"sketch":0,"ref":encode(&edge_ref),"startRef":encode(&start_ref),"endRef":encode(&end_ref)}}
            ]),
        );
        assert_eq!(out[0]["ok"], 0);
        assert!(out[1].get("ok").is_some(), "{out:?}");
        assert!(out[2].get("ok").is_some(), "{out:?}");
        let handles: ExternalLineHandles = serde_json::from_value(out[2]["ok"].clone()).unwrap();
        (k, 0, solid, right, edge_ref, handles)
    }

    #[test]
    fn declared_linked_refusals_have_actionable_failure_categories() {
        use remus_topology::journal::{EntityKind, OpId};
        use remus_topology::naming::Anchor;

        let (mut k, sketch, _, right, edge_ref, external) = setup();
        let source = k.gcs_sketches.get(sketch as usize).unwrap().external_lines[0]
            .reference
            .clone();
        let face_ref = encode(
            k.gcs_sketches
                .get(sketch as usize)
                .unwrap()
                .attachment
                .as_ref()
                .unwrap()
                .reference(),
        );
        let unattached = batch(&mut k, serde_json::json!([{"op":"gcsNew","args":{}}]))[0]["ok"]
            .as_u64()
            .unwrap();
        let frame = serde_json::json!({"origin":[0,0,4],"xAxis":[1,0,0],"normal":[0,0,1]});
        let invalid_frame = serde_json::json!({"origin":[0,0,4],"xAxis":[0,0,0],"normal":[0,0,1]});
        assert_refusal(
            &mut k,
            "gcsAttachToFace",
            serde_json::json!({"sketch":unattached,"ref":face_ref,"frame":invalid_frame}),
            "linked-sketch:invalid_frame",
            "invalid_input",
        );
        assert_refusal(
            &mut k,
            "gcsAttachToFace",
            serde_json::json!({"sketch":unattached,"ref":encode(&edge_ref),"frame":frame}),
            "linked-sketch:invalid_reference_kind",
            "invalid_input",
        );
        let Anchor::OperationOutput { operation, .. } = edge_ref.anchor else {
            panic!()
        };
        let right_ref = (0..100).map(|index| PersistentRef::operation_output(operation, EntityKind::Face, index)).find(|reference| matches!(resolve(k.topo(),reference), Resolution::Bound{entity,..} if entity==EntityKey::face(right.index()))).unwrap();
        assert_refusal(
            &mut k,
            "gcsAttachToFace",
            serde_json::json!({"sketch":unattached,"ref":encode(&right_ref),"frame":frame}),
            "linked-sketch:unsupported_attachment",
            "unsupported",
        );
        // Every failed attachment leaves the sketch available for a valid one.
        assert!(batch(&mut k, serde_json::json!([{"op":"gcsAttachToFace","args":{"sketch":unattached,"ref":face_ref,"frame":frame}}]))[0].get("ok").is_some());

        let before = k.gcs_point_position_impl(sketch, external.end).unwrap();
        assert_refusal(
            &mut k,
            "gcsSetPoint",
            serde_json::json!({"sketch":sketch,"point":external.end,"x":99,"y":99}),
            "linked-sketch:read_only_external_point",
            "invalid_input",
        );
        let missing =
            PersistentRef::operation_output(OpId::from_value(u64::MAX), EntityKind::Edge, 0);
        let error = assert_refusal(
            &mut k,
            "gcsAddExternalLine",
            serde_json::json!({"sketch":sketch,"ref":encode(&missing),"startRef":encode(&source.start),"endRef":encode(&source.end)}),
            "linked-sketch:reference_unbound",
            "invalid_input",
        );
        assert_eq!(error["details"]["referenceRole"], "edge");
        assert_eq!(error["details"]["resolution"]["status"], "unknownOperation");

        // A real analytic cylinder supplies a well-formed unsupported circle.
        let pending = k.topo_mut().journal_begin("fixture_cylinder");
        let cylinder = remus_operations::primitives::make_cylinder(k.topo_mut(), 2.0, 4.0).unwrap();
        let mut draft = EvolutionDraft::construction();
        for key in solid_entity_keys(k.topo(), cylinder).unwrap() {
            draft.push(key, EventDraft::Generated { sources: vec![] });
        }
        let operation = k
            .topo_mut()
            .journal_record_evolution(pending, draft)
            .unwrap();
        let circle = remus_topology::explorer::solid_edges(k.topo(), cylinder)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                matches!(
                    k.topo().edge(edge).unwrap().curve(),
                    remus_topology::edge::EdgeCurve::Circle(_)
                )
            })
            .unwrap();
        let output = |key: EntityKey| {
            (0..100).map(|index| PersistentRef::operation_output(operation, key.kind, index)).find(|reference| matches!(resolve(k.topo(), reference), Resolution::Bound{entity,..} if entity==key)).unwrap()
        };
        let circle_ref = output(EntityKey::edge(circle.index()));
        let start_ref = output(EntityKey::vertex(
            k.topo().edge(circle).unwrap().start().index(),
        ));
        let end_ref = output(EntityKey::vertex(
            k.topo().edge(circle).unwrap().end().index(),
        ));
        assert_refusal(
            &mut k,
            "gcsAddExternalLine",
            serde_json::json!({"sketch":sketch,"ref":encode(&circle_ref),"startRef":encode(&start_ref),"endRef":encode(&end_ref)}),
            "linked-sketch:unsupported_curve",
            "unsupported",
        );
        assert_eq!(
            k.gcs_point_position_impl(sketch, external.end)
                .unwrap()
                .map(f64::to_bits),
            before.map(f64::to_bits)
        );
        assert_eq!(
            k.gcs_sketches
                .get(sketch as usize)
                .unwrap()
                .external_lines
                .len(),
            1
        );
        let state = k.gcs_sketches.get(sketch as usize).unwrap();
        assert_eq!(state.points.len(), 2);
        assert_eq!(state.lines.len(), 1);
    }

    #[test]
    fn ambiguous_external_identity_preserves_resolution_and_last_good_points() {
        let (mut k, sketch, _, _, edge_ref, external) = setup();
        let source = k.gcs_sketches.get(sketch as usize).unwrap().external_lines[0]
            .reference
            .clone();
        let Resolution::Bound { entity, .. } = resolve(k.topo(), &edge_ref) else {
            panic!()
        };
        let edge = k.topo().edge_id_from_index(entity.index).unwrap();
        let signature =
            remus_topology::naming::EntitySignature::capture_edge(k.topo(), edge, 1e-7).unwrap();
        let pending = k.topo_mut().journal_begin("fixture_duplicate_box");
        let duplicate =
            remus_operations::primitives::make_box(k.topo_mut(), 10.0, 8.0, 4.0).unwrap();
        let mut draft = EvolutionDraft::construction();
        for key in solid_entity_keys(k.topo(), duplicate).unwrap() {
            draft.push(key, EventDraft::Generated { sources: vec![] });
        }
        k.topo_mut()
            .journal_record_evolution(pending, draft)
            .unwrap();
        let ambiguous = PersistentRef::signature(signature);
        assert!(matches!(
            resolve(k.topo(), &ambiguous),
            Resolution::Ambiguous { .. }
        ));
        let before = k.gcs_point_position_impl(sketch, external.end).unwrap();
        let error = assert_refusal(
            &mut k,
            "gcsAddExternalLine",
            serde_json::json!({"sketch":sketch,"ref":encode(&ambiguous),"startRef":encode(&source.start),"endRef":encode(&source.end)}),
            "linked-sketch:reference_unbound",
            "invalid_input",
        );
        assert_eq!(error["details"]["referenceRole"], "edge");
        assert_eq!(error["details"]["resolution"]["status"], "ambiguous");
        assert_eq!(
            error["details"]["resolution"]["candidates"]
                .as_array()
                .unwrap()
                .len(),
            2
        );
        assert_eq!(
            k.gcs_point_position_impl(sketch, external.end)
                .unwrap()
                .map(f64::to_bits),
            before.map(f64::to_bits)
        );
    }

    #[test]
    fn nonconvergence_restores_free_parameters_after_successful_input_refresh() {
        for v2 in [false, true] {
            let (mut k, sketch, solid, right, _, external) = setup();
            let point = k.gcs_add_point_impl(sketch, 5.0, 0.0, false).unwrap();
            for constraint in [
                serde_json::json!({"type":"midpoint","point":point,"line":external.line}),
                serde_json::json!({"type":"fixX","point":point,"value":5}),
            ] {
                k.gcs_add_constraint_impl(sketch, &constraint.to_string())
                    .unwrap();
            }
            assert!(
                k.gcs_solve_detailed_impl(sketch, 100, 1e-10)
                    .unwrap()
                    .converged
            );
            let before = k.gcs_point_position_impl(sketch, point).unwrap();
            remus_operations::journal_ops::move_faces_journaled(k.topo_mut(), solid, &[right], 2.0)
                .unwrap();
            let ops = serde_json::json!([{"op":"gcsSolveDetailed","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}}]);
            let out: serde_json::Value = serde_json::from_str(&if v2 {
                k.execute_batch_v2(&ops.to_string())
            } else {
                k.execute_batch(&ops.to_string())
            })
            .unwrap();
            let solve = &out[0]["ok"];
            assert_eq!(solve["converged"], false, "{out}");
            assert_eq!(solve["rolledBack"], true, "{out}");
            assert_eq!(solve["classification"], "unsatisfied", "{out}");
            assert_eq!(
                k.gcs_point_position_impl(sketch, point)
                    .unwrap()
                    .map(f64::to_bits),
                before.map(f64::to_bits)
            );
            let start = k.gcs_point_position_impl(sketch, external.start).unwrap();
            let end = k.gcs_point_position_impl(sketch, external.end).unwrap();
            assert!(((end[0] - start[0]).abs() - 12.0).abs() < 1e-9);
            let published = (before[0] - start[0].midpoint(end[0]))
                .abs()
                .max((before[0] - 5.0).abs());
            assert!((published - 1.0).abs() < 1e-9);
            assert!(
                (solve["publishedMaxResidual"].as_f64().unwrap() - published).abs() < 1e-9,
                "{out}"
            );
            assert!(
                (solve["maxResidual"].as_f64().unwrap() - 0.5).abs() < 1e-9,
                "{out}"
            );
            // Plain solve keeps its established final-iterate behavior.
            assert!(!k.gcs_solve_impl(sketch, 100, 1e-10).unwrap().converged);
            assert!((k.gcs_point_position_impl(sketch, point).unwrap()[0] - 5.5).abs() < 1e-9);
        }
    }

    #[test]
    fn batch_constraints_follow_model_edits_and_checkpoint_restore() {
        let (mut k, sketch, solid, right, _, external) = setup();
        let out = batch(
            &mut k,
            serde_json::json!([
                {"op":"gcsAddPoint","args":{"sketch":sketch,"x":4,"y":3,"fixed":false}},
                {"op":"gcsAddConstraint","args":{"sketch":sketch,"constraint":{"type":"midpoint","point":2,"line":external.line}}},
                {"op":"gcsSolve","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}},
                {"op":"gcsPointPosition","args":{"sketch":sketch,"point":2}}
            ]),
        );
        assert!(out[2]["ok"]["converged"].as_bool().unwrap(), "{out:?}");
        assert!((out[3]["ok"][0].as_f64().unwrap() - 5.0).abs() < 1e-8);
        let cp = k.checkpoint_impl().unwrap();
        let out = batch(
            &mut k,
            serde_json::json!([
                {"op":"moveFacesJournaled","args":{"solid":solid.index(),"faces":[right.index()],"distance":2}},
                {"op":"gcsSolveDetailed","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}},
                {"op":"gcsPointPosition","args":{"sketch":sketch,"point":2}},
                {"op":"gcsResolveRefs","args":{"sketch":sketch}}
            ]),
        );
        assert!(out[0].get("ok").is_some(), "{out:?}");
        assert!(out[1]["ok"]["converged"].as_bool().unwrap(), "{out:?}");
        assert!(
            (out[2]["ok"][0].as_f64().unwrap() - 6.0).abs() < 1e-8,
            "{out:?}"
        );
        assert_eq!(out[3]["ok"]["externalLines"], 1);
        k.restore_checkpoint_impl(cp).unwrap();
        let out = batch(
            &mut k,
            serde_json::json!([
                {"op":"gcsResolveRefs","args":{"sketch":sketch}},
                {"op":"gcsPointPosition","args":{"sketch":sketch,"point":2}},
                {"op":"gcsSetPoint","args":{"sketch":sketch,"point":external.start,"x":99,"y":99}}
            ]),
        );
        assert!(out[0].get("ok").is_some(), "{out:?}");
        assert!((out[1]["ok"][0].as_f64().unwrap() - 5.0).abs() < 1e-8);
        assert!(
            out[2]["error"]
                .to_string()
                .contains("read_only_external_point")
        );
    }

    #[test]
    fn typed_refusal_keeps_last_valid_external_state_and_restore_retires_handles() {
        for split in [true, false] {
            let (mut k, sketch, _, _, edge_ref, external) = setup();
            let saved = k.gcs_point_position_impl(sketch, external.end).unwrap();
            let cp = k.checkpoint_impl().unwrap();
            let extra=batch(&mut k,serde_json::json!([{"op":"gcsAddPoint","args":{"sketch":sketch,"x":8,"y":9,"fixed":true}}]))[0]["ok"].as_u64().unwrap();
            let Resolution::Bound { entity, .. } = resolve(k.topo(), &edge_ref) else {
                panic!()
            };
            let pending = k.topo_mut().journal_begin("fixture_source_edit");
            let mut draft = EvolutionDraft::construction();
            if split {
                let id = k.topo().edge_id_from_index(entity.index).unwrap();
                let copy = k.topo().edge(id).unwrap().clone();
                let other = k.topo_mut().add_edge(copy);
                draft.push(entity, EventDraft::Modified { from: entity });
                draft.push(
                    EntityKey::edge(other.index()),
                    EventDraft::Modified { from: entity },
                );
            } else {
                draft.push(entity, EventDraft::Deleted);
            }
            k.topo_mut()
                .journal_record_evolution(pending, draft)
                .unwrap();
            let out:serde_json::Value=serde_json::from_str(&k.execute_batch_v2(&serde_json::json!([
                {"op":"gcsResolveRefs","args":{"sketch":sketch}},
                {"op":"gcsSolve","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}},
                {"op":"gcsSolveDetailed","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}},
                {"op":"gcsDof","args":{"sketch":sketch}}
            ]).to_string())).unwrap();
            for item in out.as_array().unwrap() {
                assert_eq!(
                    item["error"]["category"],
                    if split {
                        "unsupported"
                    } else {
                        "invalid_input"
                    },
                    "{item}"
                );
                let details = &item["error"]["details"];
                assert_eq!(
                    details["kernelCode"], "linked-sketch:reference_unbound",
                    "{item}"
                );
                assert_eq!(details["referenceRole"], "edge", "{item}");
                assert_eq!(
                    details["resolution"]["status"],
                    if split { "boundMany" } else { "dangling" },
                    "{item}"
                );
            }
            let legacy = batch(
                &mut k,
                serde_json::json!([
                    {"op":"gcsResolveRefs","args":{"sketch":sketch}},
                    {"op":"gcsSolve","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}},
                    {"op":"gcsSolveDetailed","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}},
                    {"op":"gcsDof","args":{"sketch":sketch}}
                ]),
            );
            for item in legacy {
                assert!(
                    item["error"]
                        .as_str()
                        .unwrap()
                        .starts_with("linked-sketch:reference_unbound:"),
                    "{item}"
                );
            }
            let direct_error = k.gcs_solve_impl(sketch, 100, 1e-10).unwrap_err();
            assert!(
                direct_error
                    .to_string()
                    .starts_with("linked-sketch:reference_unbound:")
            );
            assert_eq!(
                k.gcs_point_position_impl(sketch, external.end)
                    .unwrap()
                    .map(f64::to_bits),
                saved.map(f64::to_bits)
            );
            k.restore_checkpoint_impl(cp).unwrap();
            assert!(k.gcs_point_position_impl(sketch, extra as u32).is_err());
            let fresh = k.gcs_add_point_impl(sketch, 1.0, 2.0, true).unwrap();
            assert!(u64::from(fresh) > extra);
            let out = batch(
                &mut k,
                serde_json::json!([{"op":"gcsResolveRefs","args":{"sketch":sketch}}]),
            );
            assert!(out[0].get("ok").is_some(), "{out:?}");
        }
    }

    #[test]
    fn solve_errors_roll_back_refreshed_inputs_and_external_centers_are_read_only() {
        for detailed in [false, true] {
            let (mut k, sketch, solid, right, _, ext) = setup();
            let state = k.gcs_sketches.get(sketch as usize).unwrap().clone();
            let mut limited = remus_sketch::GcsSystem::with_limits(remus_sketch::GcsLimits {
                max_dense_bytes: 1,
                max_dense_qr_work: usize::MAX,
            });
            for id in state.points.iter().flatten() {
                assert_eq!(
                    limited.add_point(*state.sys.point(*id).unwrap()).unwrap(),
                    *id
                );
            }
            let line = &state.external_lines[0];
            assert_eq!(
                limited.add_line(line.start, line.end).unwrap(),
                state.lines[0].unwrap()
            );
            k.gcs_sketches.get_mut(sketch as usize).unwrap().sys = limited;
            let free = k.gcs_add_point_impl(sketch, 1.0, 2.0, false).unwrap();
            k.gcs_add_constraint_impl(
                sketch,
                &serde_json::json!({"type":"fixX","point":free,"value":3}).to_string(),
            )
            .unwrap();
            let before = k.gcs_point_position_impl(sketch, ext.end).unwrap();
            remus_operations::journal_ops::move_faces_journaled(k.topo_mut(), solid, &[right], 2.0)
                .unwrap();
            let out: serde_json::Value = serde_json::from_str(&k.execute_batch_v2(&serde_json::json!([
                {"op":if detailed {"gcsSolveDetailed"} else {"gcsSolve"}, "args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}}
            ]).to_string())).unwrap();
            assert_eq!(out[0]["error"]["code"], "resource_limit_exceeded", "{out}");
            assert_eq!(
                k.gcs_point_position_impl(sketch, ext.end)
                    .unwrap()
                    .map(f64::to_bits),
                before.map(f64::to_bits)
            );
            let circle = k
                .gcs_add_ellipse_impl(sketch, ext.start, 3.0, 2.0, 0.0)
                .unwrap();
            let out = batch(
                &mut k,
                serde_json::json!([
                    {"op":"gcsSetEllipse","args":{"sketch":sketch,"ellipse":circle,"x":50,"y":50,"a":3,"b":2,"angle":0}}
                ]),
            );
            assert!(
                out[0]["error"]
                    .to_string()
                    .contains("read_only_external_point"),
                "{out:?}"
            );
            let before_start = k.gcs_point_position_impl(sketch, ext.start).unwrap();
            assert!(before_start[0].abs() < 1e-9 && before_start[1].abs() < 1e-9);
        }
    }
}

#[cfg(test)]
mod circle_tests {
    #![allow(clippy::unwrap_used, clippy::panic, clippy::float_cmp)]
    use super::*;
    use remus_operations::journal_ops::{move_faces_journaled, resize_cylindrical_face_journaled};
    use remus_topology::face::{FaceId, FaceSurface};
    use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft};
    use remus_topology::naming::{Resolution, resolve};
    use serde_json::{Value, json};

    #[test]
    fn line_only_resolution_json_defaults_the_new_circle_count() {
        let old = json!({
            "frame": {"origin":[0.0,0.0,4.0], "xAxis":[1.0,0.0,0.0], "normal":[0.0,0.0,1.0]},
            "externalLines": 2,
        });
        let result: LinkedSketchResolution = serde_json::from_value(old).unwrap();
        assert_eq!(result.external_lines, 2);
        assert_eq!(result.external_circles, 0);
        let encoded = serde_json::to_value(result).unwrap();
        assert_eq!(encoded["externalLines"], 2);
        assert_eq!(encoded["externalCircles"], 0);
    }

    fn row(k: &mut BrepKernel, v2: bool, op: &str, args: Value) -> Value {
        let program = json!([{"op":op,"args":args}]).to_string();
        let text = if v2 {
            k.execute_batch_v2(&program)
        } else {
            k.execute_batch(&program)
        };
        serde_json::from_str::<Vec<Value>>(&text).unwrap().remove(0)
    }
    fn call(k: &mut BrepKernel, v2: bool, op: &str, args: Value) -> Value {
        let r = row(k, v2, op, args);
        assert!(r.get("ok").is_some(), "{r}");
        r["ok"].clone()
    }
    fn reference(k: &BrepKernel, op: remus_topology::journal::OpId, key: EntityKey) -> String {
        let r = (0..100)
            .map(|i| PersistentRef::operation_output(op, key.kind, i))
            .find(|r| matches!(resolve(k.topo(),r),Resolution::Bound{entity,..}if entity==key))
            .unwrap();
        String::from_utf8(remus_io::naming_io::serialize_persistent_ref(&r).unwrap()).unwrap()
    }
    fn bore(k: &BrepKernel, solid: remus_topology::SolidId) -> FaceId {
        remus_topology::explorer::solid_faces(k.topo(), solid)
            .unwrap()
            .into_iter()
            .find(|&f| {
                matches!(
                    k.topo().face(f).unwrap().surface(),
                    FaceSurface::Cylinder(_)
                )
            })
            .unwrap()
    }
    fn setup(
        v2: bool,
    ) -> (
        BrepKernel,
        u32,
        remus_topology::SolidId,
        ExternalCircleHandles,
        String,
    ) {
        let mut k = BrepKernel::new();
        let boxed = remus_operations::primitives::make_box(k.topo_mut(), 10.0, 8.0, 4.0).unwrap();
        let tool = remus_operations::primitives::make_cylinder(k.topo_mut(), 2.0, 6.0).unwrap();
        remus_operations::transform::transform_solid(
            k.topo_mut(),
            tool,
            &remus_math::mat::Mat4([
                [1.0, 0.0, 0.0, 5.0],
                [0.0, 1.0, 0.0, 4.0],
                [0.0, 0.0, 1.0, -1.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        )
        .unwrap();
        let cut = remus_operations::boolean::boolean(
            k.topo_mut(),
            remus_operations::boolean::BooleanOp::Cut,
            boxed,
            tool,
        )
        .unwrap();
        let face = bore(&k, cut);
        let seeded = resize_cylindrical_face_journaled(k.topo_mut(), cut, face, 2.5).unwrap();
        let top=remus_topology::explorer::solid_faces(k.topo(),seeded.solid).unwrap().into_iter().find(|&f|matches!(k.topo().face(f).unwrap().surface(),FaceSurface::Plane{normal,..}if normal.z()>0.99)).unwrap();
        let wire = k.topo().face(top).unwrap().inner_wires()[0];
        let edge = k.topo().wire(wire).unwrap().edges()[0].edge();
        let face_ref = reference(&k, seeded.op, EntityKey::face(top.index()));
        let edge_ref = reference(&k, seeded.op, EntityKey::edge(edge.index()));
        let sketch = call(&mut k, v2, "gcsNew", json!({})).as_u64().unwrap() as u32;
        call(
            &mut k,
            v2,
            "gcsAttachToFace",
            json!({"sketch":sketch,"ref":face_ref,"frame":{"origin":[0,0,4],"xAxis":[1,0,0],"normal":[0,0,1]}}),
        );
        let handles: ExternalCircleHandles = serde_json::from_value(call(
            &mut k,
            v2,
            "gcsAddExternalCircle",
            json!({"sketch":sketch,"ref":edge_ref}),
        ))
        .unwrap();
        (k, sketch, seeded.solid, handles, edge_ref)
    }
    fn consumer(
        k: &mut BrepKernel,
        v2: bool,
        sketch: u32,
        external: &ExternalCircleHandles,
    ) -> (u32, u32, Vec<u32>) {
        let center = call(
            k,
            v2,
            "gcsAddPoint",
            json!({"sketch":sketch,"x":6,"y":5,"fixed":false}),
        )
        .as_u64()
        .unwrap() as u32;
        let circle = call(
            k,
            v2,
            "gcsAddCircle",
            json!({"sketch":sketch,"center":center,"radius":1}),
        )
        .as_u64()
        .unwrap() as u32;
        let constraints = [
            json!({"type":"coincident","a":center,"b":external.center}),
            json!({"type":"equalRadiusCircleCircle","circle1":circle,"circle2":external.circle}),
        ]
        .into_iter()
        .map(|constraint| {
            call(
                k,
                v2,
                "gcsAddConstraint",
                json!({"sketch":sketch,"constraint":constraint}),
            )
            .as_u64()
            .unwrap() as u32
        })
        .collect();
        (center, circle, constraints)
    }
    fn close(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    #[test]
    fn linked_circle_batch_routes_drive_a_consumer_and_restore_retired_handles() {
        for v2 in [false, true] {
            let (mut k, sketch, solid, external, source) = setup(v2);
            let dof = call(&mut k, v2, "gcsDof", json!({"sketch":sketch}));
            assert_eq!(dof["numParams"], 0);
            let (center, circle, constraints) = consumer(&mut k, v2, sketch, &external);
            let solved = call(
                &mut k,
                v2,
                "gcsSolveDetailed",
                json!({"sketch":sketch,"maxIterations":100,"tolerance":1e-10}),
            );
            assert_eq!(solved["converged"], true);
            assert_eq!(solved["numParams"], 3);
            assert_eq!(solved["rank"], 3);
            assert_eq!(solved["dof"], 0);
            close(k.gcs_circle_radius_impl(sketch, circle).unwrap(), 2.5);
            let cp = k.checkpoint_impl().unwrap();
            let retired: ExternalCircleHandles = serde_json::from_value(call(
                &mut k,
                v2,
                "gcsAddExternalCircle",
                json!({"sketch":sketch,"ref":source}),
            ))
            .unwrap();
            let face = bore(&k, solid);
            let grown = resize_cylindrical_face_journaled(k.topo_mut(), solid, face, 3.0).unwrap();
            call(
                &mut k,
                v2,
                "gcsSolve",
                json!({"sketch":sketch,"maxIterations":100,"tolerance":1e-10}),
            );
            close(k.gcs_circle_radius_impl(sketch, circle).unwrap(), 3.0);
            let top=remus_topology::explorer::solid_faces(k.topo(),grown.solid).unwrap().into_iter().find(|&f|matches!(k.topo().face(f).unwrap().surface(),FaceSurface::Plane{normal,..}if normal.z()>0.99)).unwrap();
            move_faces_journaled(k.topo_mut(), grown.solid, &[top], 1.0).unwrap();
            let resolved = call(&mut k, v2, "gcsResolveRefs", json!({"sketch":sketch}));
            assert_eq!(resolved["externalCircles"], 2);
            assert_eq!(resolved["frame"]["origin"][2], 5.0);
            for point in [center, external.center] {
                let position = k.gcs_point_position_impl(sketch, point).unwrap();
                close(position[0], 5.0);
                close(position[1], 4.0);
            }
            let conflict = call(
                &mut k,
                v2,
                "gcsAddConstraint",
                json!({"sketch":sketch,"constraint":{"type":"circleRadius","circle":external.circle,"value":4}}),
            );
            assert_eq!(
                call(
                    &mut k,
                    v2,
                    "gcsSolve",
                    json!({"sketch":sketch,"maxIterations":100,"tolerance":1e-10})
                )["converged"],
                false
            );
            close(
                k.gcs_circle_radius_impl(sketch, external.circle).unwrap(),
                3.0,
            );
            assert_eq!(
                call(
                    &mut k,
                    v2,
                    "gcsSolveDetailed",
                    json!({"sketch":sketch,"maxIterations":100,"tolerance":1e-10})
                )["rolledBack"],
                true
            );
            close(
                k.gcs_circle_radius_impl(sketch, external.circle).unwrap(),
                3.0,
            );
            for constraint in constraints.into_iter().map(Value::from).chain([conflict]) {
                call(
                    &mut k,
                    v2,
                    "gcsRemoveConstraint",
                    json!({"sketch":sketch,"constraint":constraint}),
                );
            }
            assert_eq!(
                call(&mut k, v2, "gcsDof", json!({"sketch":sketch}))["numParams"],
                3
            );
            close(
                k.gcs_circle_radius_impl(sketch, external.circle).unwrap(),
                3.0,
            );
            let ellipse = k
                .gcs_add_ellipse_impl(sketch, external.center, 2.0, 1.0, 0.0)
                .unwrap();
            for (op, args) in [
                (
                    "gcsSetPoint",
                    json!({"sketch":sketch,"point":external.center,"x":99,"y":99}),
                ),
                (
                    "gcsSetEllipse",
                    json!({"sketch":sketch,"ellipse":ellipse,"x":99,"y":99,"a":3,"b":2,"angle":0}),
                ),
            ] {
                let before = format!("{:?}", k.gcs_sketches.get(sketch as usize).unwrap());
                let refused = row(&mut k, v2, op, args);
                assert!(
                    refused["error"]
                        .to_string()
                        .contains("linked-sketch:read_only_external_point"),
                    "{refused}"
                );
                if v2 {
                    assert_eq!(refused["error"]["category"], "invalid_input");
                }
                assert_eq!(
                    format!("{:?}", k.gcs_sketches.get(sketch as usize).unwrap()),
                    before
                );
            }
            k.restore_checkpoint_impl(cp).unwrap();
            close(
                k.gcs_circle_radius_impl(sketch, external.circle).unwrap(),
                2.5,
            );
            assert!(k.gcs_circle_radius_impl(sketch, retired.circle).is_err());
            assert!(k.gcs_point_position_impl(sketch, retired.center).is_err());
            assert_eq!(
                k.gcs_sketches
                    .get(sketch as usize)
                    .unwrap()
                    .sys
                    .fixed_circle_radius_count(),
                1
            );
            let next: ExternalCircleHandles = serde_json::from_value(call(
                &mut k,
                v2,
                "gcsAddExternalCircle",
                json!({"sketch":sketch,"ref":source}),
            ))
            .unwrap();
            assert!(next.circle > retired.circle && next.center > retired.center);
        }
    }

    #[test]
    fn linked_circle_nonunique_refs_keep_each_typed_refresh_route_and_last_state() {
        for split in [false, true] {
            let (mut k, sketch, _, external, source) = setup(true);
            let reference = super::reference(&source).unwrap();
            let Resolution::Bound { entity, .. } = resolve(k.topo(), &reference) else {
                panic!("circle")
            };
            let pending = k.topo_mut().journal_begin("fixture_circle_split");
            let mut draft = EvolutionDraft::construction();
            if split {
                let id = k.topo().edge_id_from_index(entity.index).unwrap();
                let copied = k.topo().edge(id).unwrap().clone();
                let other = k.topo_mut().add_edge(copied);
                for key in [entity, EntityKey::edge(other.index())] {
                    draft.push(key, EventDraft::Modified { from: entity });
                }
            } else {
                draft.push(entity, EventDraft::Deleted);
            }
            k.topo_mut()
                .journal_record_evolution(pending, draft)
                .unwrap();
            let before = format!("{:?}", k.gcs_sketches.get(sketch as usize).unwrap());
            for op in ["gcsResolveRefs", "gcsSolve", "gcsSolveDetailed", "gcsDof"] {
                let error = row(
                    &mut k,
                    true,
                    op,
                    json!({"sketch":sketch,"maxIterations":100,"tolerance":1e-10}),
                );
                assert_eq!(
                    error["error"]["category"],
                    if split {
                        "unsupported"
                    } else {
                        "invalid_input"
                    },
                    "{error}"
                );
                assert_eq!(
                    error["error"]["details"]["kernelCode"],
                    "linked-sketch:reference_unbound"
                );
                assert_eq!(
                    error["error"]["details"]["resolution"]["status"],
                    if split { "boundMany" } else { "dangling" }
                );
                assert_eq!(
                    format!("{:?}", k.gcs_sketches.get(sketch as usize).unwrap()),
                    before
                );
                let legacy = row(
                    &mut k,
                    false,
                    op,
                    json!({"sketch":sketch,"maxIterations":100,"tolerance":1e-10}),
                );
                assert!(
                    legacy["error"]
                        .as_str()
                        .unwrap()
                        .starts_with("linked-sketch:reference_unbound:")
                );
            }
            close(
                k.gcs_circle_radius_impl(sketch, external.circle).unwrap(),
                2.5,
            );
            assert!(
                k.gcs_solve_impl(sketch, 100, 1e-10)
                    .unwrap_err()
                    .to_string()
                    .starts_with("linked-sketch:reference_unbound:")
            );
        }
    }

    #[test]
    fn linked_circle_refresh_rolls_back_on_solver_or_rank_resource_failure() {
        for op in ["gcsSolve", "gcsSolveDetailed", "gcsDof"] {
            let (mut k, sketch, solid, external, _) = setup(true);
            consumer(&mut k, true, sketch, &external);
            let state = k.gcs_sketches.get(sketch as usize).unwrap().clone();
            let mut limited = remus_sketch::GcsSystem::with_limits(remus_sketch::GcsLimits {
                max_dense_bytes: 1,
                max_dense_qr_work: usize::MAX,
            });
            for (id, data) in state.sys.points() {
                assert_eq!(limited.add_point(*data).unwrap(), id);
            }
            for (id, data) in state.sys.circles() {
                let new = if state.sys.is_circle_radius_fixed(id) {
                    limited.add_circle_with_fixed_radius(data.center, data.radius)
                } else {
                    limited.add_circle(data.center, data.radius)
                }
                .unwrap();
                assert_eq!(new, id);
            }
            for id in state.constraints.iter().flatten() {
                assert_eq!(
                    limited
                        .add_constraint(state.sys.constraint(*id).unwrap().clone())
                        .unwrap(),
                    *id
                );
            }
            k.gcs_sketches.get_mut(sketch as usize).unwrap().sys = limited;
            let face = bore(&k, solid);
            resize_cylindrical_face_journaled(k.topo_mut(), solid, face, 3.0).unwrap();
            let before = format!("{:?}", k.gcs_sketches.get(sketch as usize).unwrap());
            let error = row(
                &mut k,
                true,
                op,
                json!({"sketch":sketch,"maxIterations":100,"tolerance":1e-10}),
            );
            assert_eq!(error["error"]["code"], "resource_limit_exceeded", "{error}");
            assert_eq!(
                format!("{:?}", k.gcs_sketches.get(sketch as usize).unwrap()),
                before
            );
            close(
                k.gcs_circle_radius_impl(sketch, external.circle).unwrap(),
                2.5,
            );
            assert_eq!(
                k.gcs_sketches
                    .get(sketch as usize)
                    .unwrap()
                    .sys
                    .fixed_circle_radius_count(),
                1
            );
        }
    }
}
