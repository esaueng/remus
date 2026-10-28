//! Bounded linked planar sketch inputs (P-Class 6.6, core tier).
#![allow(clippy::missing_errors_doc)]

use remus_math::frame::Frame3;
use remus_math::vec::{Point3, Vec3};
use remus_operations::linked_sketch::{
    ExternalLineReference, LinkedSketchError, PlanarSketchAttachment,
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

/// Handles for ordinary GCS constraints; the two point inputs are read-only.
#[derive(Debug, Clone, Serialize, Deserialize, tsify::Tsify)]
#[serde(rename_all = "camelCase")]
pub struct ExternalLineHandles {
    pub line: u32,
    pub start: u32,
    pub end: u32,
}

/// Current attached plane and count of refreshed external lines.
#[derive(Debug, Serialize, Deserialize, tsify::Tsify)]
#[serde(rename_all = "camelCase")]
pub struct LinkedSketchResolution {
    pub frame: SketchFrame,
    pub external_lines: usize,
}

fn linked_error(error: LinkedSketchError) -> StructuredWasmError {
    let mut result = StructuredWasmError::operation_failed(error.to_string());
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
        let state = self.linked_state(sketch)?;
        let attachment = state
            .attachment
            .as_ref()
            .ok_or_else(|| invalid("sketch has no linked attachment"))?;
        let frame = attachment
            .resolve_frame(self.topo())
            .map_err(linked_error)?;
        let mut next = state.clone();
        for external in &state.external_lines {
            let image = attachment
                .resolve_line(self.topo(), &external.reference)
                .map_err(linked_error)?;
            for (id, point) in [(external.start, image.start), (external.end, image.end)] {
                let data = next
                    .sys
                    .point_mut(id)
                    .ok_or_else(|| invalid("external sketch point was removed"))?;
                data.x = point.x();
                data.y = point.y();
                data.fixed = true;
            }
        }
        let external_lines = state.external_lines.len();
        *self
            .gcs_sketches
            .get_mut(sketch as usize)
            .ok_or_else(|| invalid("invalid gcs sketch handle"))? = next;
        Ok(LinkedSketchResolution {
            frame: frame_json(frame),
            external_lines,
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
            self.resolve_sketch_refs_impl(sketch)
                .map_err(|error| WasmError::InvalidInput {
                    reason: direct_message(error),
                })?;
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
    format!("{code}: {message}")
}

fn js_error(error: StructuredWasmError) -> JsError {
    JsError::new(&direct_message(error))
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
        let encode = |r: &PersistentRef| {
            String::from_utf8(remus_io::naming_io::serialize_persistent_ref(r).unwrap()).unwrap()
        };
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
                {"op":"gcsSolve","args":{"sketch":sketch,"maxIterations":100,"tolerance":1e-10}}
            ]).to_string())).unwrap();
            let text = out.to_string();
            assert!(
                text.contains(if split { "boundMany" } else { "dangling" }),
                "{text}"
            );
            assert!(text.contains("linked-sketch:reference_unbound"), "{text}");
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
