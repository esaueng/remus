//! Entity-evolution surfacing (Issue 12 → JS).
//!
//! Exposes the construction-derived vertex/edge/face history of GFA
//! booleans, one-call journaled blends, offsets, and patterns, and a
//! read-only journal summary. Event encodings are stable JSON; `unresolved`
//! is an honest event, never hidden.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use remus_operations::boolean::{BooleanOp, EdgeEvent, EntityEvolution, VertexEvent};
use remus_operations::journal_ops;
use remus_topology::journal::EntryPayload;

use crate::error::{StructuredWasmError, validate_finite};
use crate::helpers::{get_f64, get_optional_f64, get_u32, get_u32_array};
use crate::kernel::BrepKernel;

fn index_u32(index: usize) -> u32 {
    u32::try_from(index).unwrap_or(u32::MAX)
}

/// The stable JSON encoding of an Issue 12 entity-evolution payload.
fn entity_evolution_json(evolution: &EntityEvolution) -> serde_json::Value {
    let faces: Vec<serde_json::Value> = evolution
        .faces
        .iter()
        .map(|&(face, source)| {
            serde_json::json!({
                "face": index_u32(face),
                "source": source.map(index_u32),
            })
        })
        .collect();
    let edges: Vec<serde_json::Value> = evolution
        .edges
        .iter()
        .map(|(edge, event)| {
            let edge = index_u32(*edge);
            match event {
                EdgeEvent::Preserved(from) => serde_json::json!({
                    "edge": edge, "event": "preserved", "from": index_u32(*from),
                }),
                EdgeEvent::Modified(from) => serde_json::json!({
                    "edge": edge, "event": "modified", "from": index_u32(*from),
                }),
                EdgeEvent::Generated { face_a, face_b } => serde_json::json!({
                    "edge": edge,
                    "event": "generated",
                    "faceA": face_a.map(index_u32),
                    "faceB": face_b.map(index_u32),
                }),
                EdgeEvent::Unresolved => serde_json::json!({
                    "edge": edge, "event": "unresolved",
                }),
            }
        })
        .collect();
    let vertices: Vec<serde_json::Value> = evolution
        .vertices
        .iter()
        .map(|(vertex, event)| {
            let vertex = index_u32(*vertex);
            match event {
                VertexEvent::Preserved(from) => serde_json::json!({
                    "vertex": vertex, "event": "preserved", "from": index_u32(*from),
                }),
                VertexEvent::Created => serde_json::json!({
                    "vertex": vertex, "event": "created",
                }),
            }
        })
        .collect();
    serde_json::json!({ "faces": faces, "edges": edges, "vertices": vertices })
}

/// The stable JSON encoding of result-aware face, edge and vertex history
/// (B18): every result entity's event, typed reasons on every unresolved
/// record, and the per-kind completeness report against the result.
///
/// Face events: `modified` (`from`, one source), `merged` (`from`, several),
/// `generated` (`faces`), `unresolved` (`candidates`, `reason`); `deleted`
/// lists consumed source faces. Edge and vertex events: `modified` (`from`),
/// `merged` (`from`), `generated` (`faces`, the source faces the entity was
/// built between), `unresolved` (`candidates`, `reason`). Reasons are
/// `ambiguous_incidence`, `unmapped_incident_face` or
/// `unresolved_face_origin`.
fn entity_history_json(
    faces: &remus_operations::evolution::EvolutionMap,
    boundary: &remus_operations::boundary_evolution::BoundaryEvolution,
    completeness: &remus_operations::boundary_evolution::EntityCompletenessReport,
) -> serde_json::Value {
    use remus_operations::boundary_evolution::{BoundaryEvent, UnresolvedReason};
    use std::collections::BTreeMap;

    let indices = |values: &[usize]| values.iter().copied().map(index_u32).collect::<Vec<_>>();

    let mut modified: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (&input, outputs) in &faces.modified {
        for &output in outputs {
            modified.entry(output).or_default().push(input);
        }
    }
    let mut generated: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (&input, outputs) in &faces.generated {
        for &output in outputs {
            generated.entry(output).or_default().push(input);
        }
    }
    let mut face_events: BTreeMap<usize, serde_json::Value> = BTreeMap::new();
    for (output, mut inputs) in modified {
        inputs.sort_unstable();
        inputs.dedup();
        let event = match inputs.as_slice() {
            [single] => serde_json::json!({
                "face": index_u32(output), "event": "modified", "from": index_u32(*single),
            }),
            _ => serde_json::json!({
                "face": index_u32(output), "event": "merged", "from": indices(&inputs),
            }),
        };
        face_events.insert(output, event);
    }
    for (output, mut inputs) in generated {
        inputs.sort_unstable();
        inputs.dedup();
        face_events.insert(
            output,
            serde_json::json!({
                "face": index_u32(output), "event": "generated", "faces": indices(&inputs),
            }),
        );
    }
    for (&output, candidates) in &faces.unresolved {
        face_events.insert(
            output,
            serde_json::json!({
                "face": index_u32(output),
                "event": "unresolved",
                "candidates": indices(candidates),
                "reason": UnresolvedReason::UnresolvedFaceOrigin.as_str(),
            }),
        );
    }
    let mut deleted: Vec<usize> = faces.deleted.iter().copied().collect();
    deleted.sort_unstable();

    let boundary_events = |kind: &str, claims: &BTreeMap<usize, BoundaryEvent>| {
        claims
            .iter()
            .map(|(&subject, event)| {
                let mut value = match event {
                    BoundaryEvent::Modified { from } => {
                        serde_json::json!({ "event": "modified", "from": index_u32(*from) })
                    }
                    BoundaryEvent::Merged { from } => {
                        serde_json::json!({ "event": "merged", "from": indices(from) })
                    }
                    BoundaryEvent::Generated { faces } => {
                        serde_json::json!({ "event": "generated", "faces": indices(faces) })
                    }
                    BoundaryEvent::Unresolved { candidates, reason } => serde_json::json!({
                        "event": "unresolved",
                        "candidates": indices(candidates),
                        "reason": reason.as_str(),
                    }),
                };
                value[kind] = serde_json::json!(index_u32(subject));
                value
            })
            .collect::<Vec<_>>()
    };

    let report = |report: &remus_operations::evolution::CompletenessReport| {
        serde_json::json!({
            "omitted": indices(&report.omitted),
            "phantom": indices(&report.phantom),
            "unresolved": indices(&report.unresolved_outputs),
        })
    };

    serde_json::json!({
        "origin": faces.origin.as_str(),
        "faces": face_events.into_values().collect::<Vec<_>>(),
        "deleted": indices(&deleted),
        "edges": boundary_events("edge", &boundary.edges),
        "vertices": boundary_events("vertex", &boundary.vertices),
        "completeness": {
            "accounted": completeness.is_accounted(),
            "resolved": completeness.is_resolved(),
            "faces": report(&completeness.faces),
            "edges": report(&completeness.edges),
            "vertices": report(&completeness.vertices),
        },
    })
}

// A plane/cylinder replacement has a fixed handful of scalar fields. Bound
// direct JSON before serde builds its Value tree, including unknown fields.
const MAX_REPLACEMENT_JSON_BYTES: usize = 4096;

fn parse_replacement_json(replacement: &str) -> Result<serde_json::Value, StructuredWasmError> {
    if replacement.len() > MAX_REPLACEMENT_JSON_BYTES {
        return Err(StructuredWasmError::resource_limit(
            "replacement JSON exceeds byte limit",
            "replacement_json_bytes",
            MAX_REPLACEMENT_JSON_BYTES,
            replacement.len(),
        ));
    }
    serde_json::from_str(replacement).map_err(|error| StructuredWasmError::invalid_json(&error))
}

fn replacement_vector(
    value: &serde_json::Value,
    key: &str,
) -> Result<remus_math::vec::Vec3, StructuredWasmError> {
    let values = value[key].as_array().ok_or_else(|| {
        StructuredWasmError::invalid_argument(
            format!("missing or invalid '{key}' array"),
            Some(key),
        )
    })?;
    if values.len() != 3 {
        return Err(StructuredWasmError::invalid_argument(
            format!("'{key}' must have exactly 3 components"),
            Some(key),
        ));
    }
    let mut components = [0.0; 3];
    for (i, value) in values.iter().enumerate() {
        components[i] = value.as_f64().ok_or_else(|| {
            let argument = format!("{key}[{i}]");
            StructuredWasmError::invalid_argument(
                format!("{argument} is not a number"),
                Some(&argument),
            )
        })?;
    }
    if components.iter().any(|component| !component.is_finite()) {
        return Err(StructuredWasmError::invalid_argument(
            format!("'{key}' must be finite"),
            Some(key),
        ));
    }
    let [x, y, z] = components;
    Ok(remus_math::vec::Vec3::new(x, y, z))
}

fn parse_replacement_surface(
    value: &serde_json::Value,
    source: &remus_topology::face::FaceSurface,
) -> Result<remus_topology::face::FaceSurface, StructuredWasmError> {
    use remus_topology::face::FaceSurface;
    let object = value.as_object().ok_or_else(|| {
        StructuredWasmError::invalid_argument(
            "replacement must be a surface object",
            Some("replacement"),
        )
    })?;
    let kind = value["type"].as_str().ok_or_else(|| {
        StructuredWasmError::invalid_argument(
            "replacement requires a surface type",
            Some("replacement.type"),
        )
    })?;
    let fields: &[&str] = match kind {
        "plane" => &["type", "normal", "d"],
        "cylinder" => &["type", "origin", "axis", "radius"],
        _ => {
            return Err(StructuredWasmError::invalid_argument(
                "replacement type must be plane or cylinder",
                Some("replacement.type"),
            ));
        }
    };
    if let Some(key) = object.keys().find(|key| !fields.contains(&key.as_str())) {
        return Err(StructuredWasmError::invalid_argument(
            format!("unknown replacement field '{key}'"),
            Some(key),
        ));
    }
    if kind == "plane" {
        return Ok(FaceSurface::Plane {
            normal: replacement_vector(value, "normal")?,
            d: get_f64(value, "d")?,
        });
    }
    let origin = replacement_vector(value, "origin")?;
    let origin = remus_math::vec::Point3::new(origin.x(), origin.y(), origin.z());
    let axis = replacement_vector(value, "axis")?;
    let radius = get_f64(value, "radius")?;
    let cylinder = if let FaceSurface::Cylinder(source) = source {
        // Changing the radius must not silently rotate the periodic parameter seam.
        remus_math::surfaces::CylindricalSurface::with_ref_dir(
            origin,
            axis,
            radius,
            source.x_axis(),
        )
    } else {
        remus_math::surfaces::CylindricalSurface::new(origin, axis, radius)
    }
    .map_err(StructuredWasmError::from)?;
    Ok(FaceSurface::Cylinder(cylinder))
}

impl BrepKernel {
    fn boolean_entity_evolution_json(
        &mut self,
        op: BooleanOp,
        a: u32,
        b: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
        let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
        let (solid, evolution) = remus_operations::boolean::boolean_with_entity_evolution(
            self.topo_mut(),
            op,
            a_id,
            b_id,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(solid),
            "evolution": entity_evolution_json(&evolution),
        }))
    }

    fn fillet_journaled_json(
        &mut self,
        solid: u32,
        edges: &[u32],
        radius: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let edge_ids = edges
            .iter()
            .map(|&handle| self.resolve_edge(handle))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::fillet_journaled(self.topo_mut(), solid_id, &edge_ids, radius)
            .map_err(StructuredWasmError::blend_failure)?;
        Ok(Self::blend_json(&journaled))
    }

    fn chamfer_journaled_json(
        &mut self,
        solid: u32,
        edges: &[u32],
        d1: f64,
        d2: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let edge_ids = edges
            .iter()
            .map(|&handle| self.resolve_edge(handle))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        let journaled =
            journal_ops::chamfer_journaled(self.topo_mut(), solid_id, &edge_ids, d1, d2)
                .map_err(StructuredWasmError::blend_failure)?;
        Ok(Self::blend_json(&journaled))
    }

    fn blend_json(journaled: &journal_ops::JournaledBlend) -> serde_json::Value {
        serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(journaled.result.solid),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
            "isPartial": journaled.result.is_partial,
            "failedEdges": journaled
                .result
                .failed
                .iter()
                .map(|(edge, _)| crate::handles::edge_id_to_u32(*edge))
                .collect::<Vec<_>>(),
        })
    }

    fn linear_pattern_journaled_json(
        &mut self,
        solid: u32,
        direction: [f64; 3],
        spacing: f64,
        count: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let count =
            crate::error::validate_work_count(count, "count").map_err(StructuredWasmError::from)?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::linear_pattern_journaled(
            self.topo_mut(),
            solid_id,
            remus_math::vec::Vec3::new(direction[0], direction[1], direction[2]),
            spacing,
            count,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "compound": crate::handles::compound_id_to_u32(journaled.compound),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn circular_pattern_journaled_json(
        &mut self,
        solid: u32,
        axis: [f64; 3],
        count: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let count =
            crate::error::validate_work_count(count, "count").map_err(StructuredWasmError::from)?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::circular_pattern_journaled(
            self.topo_mut(),
            solid_id,
            remus_math::vec::Vec3::new(axis[0], axis[1], axis[2]),
            count,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "compound": crate::handles::compound_id_to_u32(journaled.compound),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
        }))
    }

    #[allow(clippy::too_many_arguments)]
    fn grid_pattern_journaled_json(
        &mut self,
        solid: u32,
        direction_x: [f64; 3],
        direction_y: [f64; 3],
        spacing_x: f64,
        spacing_y: f64,
        count_x: f64,
        count_y: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        // Keep JS counts as numbers until validated. A u32 binding would
        // truncate fractional counts and wrap values above 2^32 first.
        let count = |value, name| {
            let count = crate::error::validate_iteration_budget(value, name)
                .map_err(StructuredWasmError::from)?;
            if count == 0 {
                return Err(StructuredWasmError::invalid_argument(
                    format!("{name} must be at least 1"),
                    Some(name),
                ));
            }
            u32::try_from(count).map_err(|_| {
                StructuredWasmError::invalid_argument("count exceeds u32 range", Some(name))
            })
        };
        let count_x = count(count_x, "countX")?;
        let count_y = count(count_y, "countY")?;
        crate::error::validate_work_product(count_x, count_y, "grid copies")
            .map_err(StructuredWasmError::from)?;
        for (name, value) in [
            ("dirX[0]", direction_x[0]),
            ("dirX[1]", direction_x[1]),
            ("dirX[2]", direction_x[2]),
            ("dirY[0]", direction_y[0]),
            ("dirY[1]", direction_y[1]),
            ("dirY[2]", direction_y[2]),
            ("spacingX", spacing_x),
            ("spacingY", spacing_y),
        ] {
            validate_finite(value, name).map_err(StructuredWasmError::from)?;
        }
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::grid_pattern_journaled(
            self.topo_mut(),
            solid_id,
            remus_math::vec::Vec3::new(direction_x[0], direction_x[1], direction_x[2]),
            remus_math::vec::Vec3::new(direction_y[0], direction_y[1], direction_y[2]),
            spacing_x,
            spacing_y,
            count_x as usize,
            count_y as usize,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "compound": crate::handles::compound_id_to_u32(journaled.compound),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn imprint_json(
        &mut self,
        target: u32,
        tool: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let target_id = self
            .resolve_solid(target)
            .map_err(StructuredWasmError::from)?;
        let tool_id = self
            .resolve_solid(tool)
            .map_err(StructuredWasmError::from)?;
        let result = remus_operations::imprint::imprint(self.topo_mut(), target_id, tool_id)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn fix_shape_journaled_json(
        &mut self,
        solid: u32,
        config: &str,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let (report, op) = self
            .fix_shape_report_impl(solid, config, true)
            .map_err(StructuredWasmError::from)?;
        let mut value = serde_json::to_value(report)?;
        value["op"] = serde_json::json!(
            op.ok_or_else(|| StructuredWasmError::operation_failed(
                "healing journal entry missing"
            ))?
            .value()
        );
        Ok(value)
    }

    fn heal_pipeline_journaled_json(
        &mut self,
        solid: u32,
        steps: Vec<String>,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let (report, op) = self
            .heal_pipeline_report_impl(solid, steps, true)
            .map_err(StructuredWasmError::from)?;
        let mut value = serde_json::to_value(report)?;
        value["op"] = serde_json::json!(
            op.ok_or_else(|| StructuredWasmError::operation_failed(
                "healing journal entry missing"
            ))?
            .value()
        );
        Ok(value)
    }

    fn defeature_journaled_json(
        &mut self,
        solid: u32,
        faces: &[u32],
    ) -> Result<serde_json::Value, StructuredWasmError> {
        super::operations::validate_defeature_face_count(faces.len())
            .map_err(StructuredWasmError::from)?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_ids = faces
            .iter()
            .map(|&face| self.resolve_face(face))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        let result = journal_ops::defeature_journaled(self.topo_mut(), solid_id, &face_ids)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn draft_journaled_json(
        &mut self,
        solid: u32,
        faces: &[u32],
        pull_direction: &[f64],
        neutral_point: &[f64],
        angle_degrees: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        validate_finite(angle_degrees, "angleDegrees").map_err(StructuredWasmError::from)?;
        let angle_radians = super::operations::parse_draft_angle_radians(angle_degrees)
            .map_err(StructuredWasmError::from)?;
        let vector = |values: &[f64], name: &str| -> Result<[f64; 3], StructuredWasmError> {
            let [x, y, z] = values else {
                return Err(StructuredWasmError::invalid_argument(
                    format!("'{name}' must have exactly 3 components"),
                    Some(name),
                ));
            };
            for &value in values {
                validate_finite(value, name).map_err(StructuredWasmError::from)?;
            }
            Ok([*x, *y, *z])
        };
        let [dx, dy, dz] = vector(pull_direction, "pullDirection")?;
        let [nx, ny, nz] = vector(neutral_point, "neutralPoint")?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_ids = faces
            .iter()
            .map(|&face| self.resolve_face(face))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        let result = journal_ops::draft_journaled(
            self.topo_mut(),
            solid_id,
            &face_ids,
            remus_math::vec::Vec3::new(dx, dy, dz),
            remus_math::vec::Point3::new(nx, ny, nz),
            angle_radians,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn resize_blend_journaled_json(
        &mut self,
        solid: u32,
        face: u32,
        expected_radius: f64,
        new_radius: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let result = journal_ops::resize_blend_journaled(
            self.topo_mut(),
            solid_id,
            face_id,
            expected_radius,
            new_radius,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn remove_blends_journaled_json(
        &mut self,
        solid: u32,
        seeds: &[u32],
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let faces = seeds
            .iter()
            .map(|&face| self.resolve_face(face))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        let result = journal_ops::remove_blends_journaled(self.topo_mut(), solid_id, &faces)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn resize_cylindrical_face_journaled_json(
        &mut self,
        solid: u32,
        face: u32,
        radius: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        crate::error::validate_positive(radius, "new_radius").map_err(StructuredWasmError::from)?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let result = journal_ops::resize_cylindrical_face_journaled(
            self.topo_mut(),
            solid_id,
            face_id,
            radius,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn replace_surface_journaled_json(
        &mut self,
        solid: u32,
        face: u32,
        replacement: &serde_json::Value,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let source = self
            .topo()
            .face(face_id)
            .map_err(StructuredWasmError::from)?
            .surface();
        let replacement = parse_replacement_surface(replacement, source)?;
        super::operations::validate_move_faces_topology_work(self.topo(), solid_id, &[face_id])
            .map_err(StructuredWasmError::from)?;
        let result =
            journal_ops::replace_surface_journaled(self.topo_mut(), solid_id, face_id, replacement)
                .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    pub(crate) fn move_faces_journaled_json(
        &mut self,
        solid: u32,
        faces: &[u32],
        distance: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let face_count = u32::try_from(faces.len()).unwrap_or(u32::MAX);
        crate::error::validate_work_count(face_count, "faces")
            .map_err(StructuredWasmError::from)?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_ids = faces
            .iter()
            .map(|&handle| self.resolve_face(handle))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        super::operations::validate_move_faces_topology_work(self.topo(), solid_id, &face_ids)
            .map_err(StructuredWasmError::from)?;
        let result =
            journal_ops::move_faces_journaled(self.topo_mut(), solid_id, &face_ids, distance)
                .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn offset_journaled_json(
        &mut self,
        solid: u32,
        distance: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let journaled =
            journal_ops::offset_journaled_with_entities(self.topo_mut(), solid_id, distance)
                .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(journaled.solid),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
            "evolution": entity_history_json(
                &journaled.map,
                &journaled.boundary,
                &journaled.completeness,
            ),
        }))
    }

    fn extrude_journaled_json(
        &mut self,
        face: u32,
        direction: [f64; 3],
        distance: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        validate_finite(direction[0], "dir_x").map_err(StructuredWasmError::from)?;
        validate_finite(direction[1], "dir_y").map_err(StructuredWasmError::from)?;
        validate_finite(direction[2], "dir_z").map_err(StructuredWasmError::from)?;
        validate_finite(distance, "distance").map_err(StructuredWasmError::from)?;
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::extrude_journaled(
            self.topo_mut(),
            face_id,
            remus_math::vec::Vec3::new(direction[0], direction[1], direction[2]),
            distance,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(journaled.solid),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
            "evolution": entity_history_json(
                &journaled.map,
                &journaled.boundary,
                &journaled.completeness,
            ),
        }))
    }

    fn revolve_journaled_json(
        &mut self,
        face: u32,
        origin: [f64; 3],
        direction: [f64; 3],
        angle_degrees: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        for (value, name) in origin
            .into_iter()
            .zip(["origin_x", "origin_y", "origin_z"])
            .chain(direction.into_iter().zip(["axis_x", "axis_y", "axis_z"]))
        {
            validate_finite(value, name).map_err(StructuredWasmError::from)?;
        }
        validate_finite(angle_degrees, "angle").map_err(StructuredWasmError::from)?;
        let face = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::revolve_journaled(
            self.topo_mut(),
            face,
            remus_math::vec::Point3::new(origin[0], origin[1], origin[2]),
            remus_math::vec::Vec3::new(direction[0], direction[1], direction[2]),
            angle_degrees.to_radians(),
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(journaled.solid),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
            "evolution": entity_history_json(
                &journaled.map, &journaled.boundary, &journaled.completeness,
            ),
        }))
    }

    fn journal_summary_json(&self) -> serde_json::Value {
        let entries: Vec<serde_json::Value> = self
            .topo()
            .journal()
            .entries()
            .iter()
            .map(|entry| {
                let (entry_type, detail) = match entry.payload() {
                    EntryPayload::Evolution { origin, events, .. } => (
                        "evolution",
                        serde_json::json!({
                            "origin": origin.as_str(),
                            "events": events.len(),
                        }),
                    ),
                    EntryPayload::Barrier { affected } => {
                        ("barrier", serde_json::json!({ "affected": affected.len() }))
                    }
                    EntryPayload::GlobalBarrier => ("globalBarrier", serde_json::json!({})),
                };
                serde_json::json!({
                    "op": entry.op().value(),
                    "kind": entry.kind(),
                    "type": entry_type,
                    "detail": detail,
                })
            })
            .collect();
        serde_json::Value::Array(entries)
    }

    /// Batch dispatch for the evolution-surfacing ops; `None` when `op`
    /// is not one of them.
    #[allow(clippy::redundant_pub_crate)]
    pub(crate) fn dispatch_evolution_op(
        &mut self,
        op: &str,
        args: &serde_json::Value,
    ) -> Option<Result<serde_json::Value, StructuredWasmError>> {
        let result = match op {
            "fuseWithEntityEvolution"
            | "cutWithEntityEvolution"
            | "intersectWithEntityEvolution" => {
                let bool_op = match op {
                    "fuseWithEntityEvolution" => BooleanOp::Fuse,
                    "cutWithEntityEvolution" => BooleanOp::Cut,
                    _ => BooleanOp::Intersect,
                };
                get_u32(args, "solidA").and_then(|a| {
                    get_u32(args, "solidB")
                        .and_then(|b| self.boolean_entity_evolution_json(bool_op, a, b))
                })
            }
            "filletJournaled" => get_u32(args, "solid").and_then(|solid| {
                get_u32_array(args, "edges").and_then(|edges| {
                    get_f64(args, "radius")
                        .and_then(|radius| self.fillet_journaled_json(solid, &edges, radius))
                })
            }),
            "chamferJournaled" => get_u32(args, "solid").and_then(|solid| {
                get_u32_array(args, "edges").and_then(|edges| {
                    get_f64(args, "d1").and_then(|d1| {
                        get_f64(args, "d2")
                            .and_then(|d2| self.chamfer_journaled_json(solid, &edges, d1, d2))
                    })
                })
            }),
            "linearPatternJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let direction = crate::helpers::get_f64_array(args, "direction")?;
                let [dx, dy, dz] = direction.as_slice() else {
                    return Err(StructuredWasmError::invalid_argument(
                        "'direction' must have exactly 3 components",
                        Some("direction"),
                    ));
                };
                let spacing = get_f64(args, "spacing")?;
                let count = get_u32(args, "count")?;
                self.linear_pattern_journaled_json(solid, [*dx, *dy, *dz], spacing, count)
            })(),
            "circularPatternJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let axis = crate::helpers::get_f64_array(args, "axis")?;
                let [ax, ay, az] = axis.as_slice() else {
                    return Err(StructuredWasmError::invalid_argument(
                        "'axis' must have exactly 3 components",
                        Some("axis"),
                    ));
                };
                let count = get_u32(args, "count")?;
                self.circular_pattern_journaled_json(solid, [*ax, *ay, *az], count)
            })(),
            "gridPatternJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let x = replacement_vector(args, "dirX")?;
                let y = replacement_vector(args, "dirY")?;
                self.grid_pattern_journaled_json(
                    solid,
                    [x.x(), x.y(), x.z()],
                    [y.x(), y.y(), y.z()],
                    get_f64(args, "spacingX")?,
                    get_f64(args, "spacingY")?,
                    get_f64(args, "countX")?,
                    get_f64(args, "countY")?,
                )
            })(),
            "imprint" => get_u32(args, "target").and_then(|target| {
                get_u32(args, "tool").and_then(|tool| self.imprint_json(target, tool))
            }),
            "fixShapeWithConfigJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let config = args["configJson"].as_str().ok_or_else(|| {
                    StructuredWasmError::invalid_argument(
                        "'configJson' must be a JSON string",
                        Some("configJson"),
                    )
                })?;
                self.fix_shape_journaled_json(solid, config)
            })(),
            "runHealPipelineJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let steps: Vec<String> =
                    serde_json::from_value(args["steps"].clone()).map_err(|_| {
                        StructuredWasmError::invalid_argument(
                            "'steps' must be an array of operator names",
                            Some("steps"),
                        )
                    })?;
                self.heal_pipeline_journaled_json(solid, steps)
            })(),
            "defeatureJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                if let Some(faces) = args["faces"].as_array() {
                    super::operations::validate_defeature_face_count(faces.len())
                        .map_err(StructuredWasmError::from)?;
                }
                let faces = get_u32_array(args, "faces")?;
                self.defeature_journaled_json(solid, &faces)
            })(),
            "draftJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let faces = get_u32_array(args, "faces")?;
                let pull = crate::helpers::get_f64_array(args, "pullDirection")?;
                let neutral = crate::helpers::get_f64_array(args, "neutralPoint")?;
                let angle = get_f64(args, "angleDegrees")?;
                self.draft_journaled_json(solid, &faces, &pull, &neutral, angle)
            })(),
            "resizeBlendJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let face = get_u32(args, "face")?;
                let expected = get_f64(args, "expectedRadius")?;
                let radius = get_f64(args, "newRadius")?;
                self.resize_blend_journaled_json(solid, face, expected, radius)
            })(),
            "removeBlendsJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let seeds = get_u32_array(args, "seeds")?;
                self.remove_blends_journaled_json(solid, &seeds)
            })(),
            "resizeCylindricalFaceJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let face = get_u32(args, "face")?;
                let radius = get_f64(args, "radius")?;
                self.resize_cylindrical_face_journaled_json(solid, face, radius)
            })(),
            "replaceSurfaceJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let face = get_u32(args, "face")?;
                self.replace_surface_journaled_json(solid, face, &args["replacement"])
            })(),
            "moveFacesJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let faces = get_u32_array(args, "faces")?;
                let distance = get_f64(args, "distance")?;
                self.move_faces_journaled_json(solid, &faces, distance)
            })(),
            "offsetJournaled" => get_u32(args, "solid").and_then(|solid| {
                get_f64(args, "distance")
                    .and_then(|distance| self.offset_journaled_json(solid, distance))
            }),
            // Absent and null options retain the legacy extrude defaults;
            // present malformed values refuse before entering the operation.
            "extrudeJournaled" => (|| {
                let face = get_u32(args, "face")?;
                let direction = [
                    get_optional_f64(args, "dx")?.unwrap_or(0.0),
                    get_optional_f64(args, "dy")?.unwrap_or(0.0),
                    get_optional_f64(args, "dz")?.unwrap_or(1.0),
                ];
                let distance = get_optional_f64(args, "distance")?.unwrap_or(1.0);
                self.extrude_journaled_json(face, direction, distance)
            })(),
            "revolveJournaled" => (|| {
                let face = get_u32(args, "face")?;
                let angle = get_f64(args, "angle")?;
                let origin = [
                    get_optional_f64(args, "originX")?.unwrap_or(0.0),
                    get_optional_f64(args, "originY")?.unwrap_or(0.0),
                    get_optional_f64(args, "originZ")?.unwrap_or(0.0),
                ];
                let direction = [
                    get_optional_f64(args, "axisX")?.unwrap_or(0.0),
                    get_optional_f64(args, "axisY")?.unwrap_or(0.0),
                    get_optional_f64(args, "axisZ")?.unwrap_or(1.0),
                ];
                self.revolve_journaled_json(face, origin, direction, angle)
            })(),
            "journalSummary" => Ok(self.journal_summary_json()),
            _ => return None,
        };
        Some(result)
    }
}

#[cfg(test)]
mod revolve_history_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use crate::kernel::BrepKernel;

    fn cylinder_profile(kernel: &mut BrepKernel) -> u32 {
        let points = [[0.0, 0.0], [3.0, 0.0], [3.0, 2.0], [0.0, 2.0]];
        let edges = (0..4)
            .map(|index| {
                let a = points[index];
                let b = points[(index + 1) % 4];
                kernel
                    .make_line_edge(a[0], 0.0, a[1], b[0], 0.0, b[1])
                    .unwrap()
            })
            .collect();
        let wire = kernel.make_wire(edges, true).unwrap();
        kernel.make_planar_face_from_wire(wire).unwrap()
    }

    #[test]
    fn revolve_journaled_direct_batch_history_and_refusal_parity() {
        let mut direct = BrepKernel::new();
        let face = cylinder_profile(&mut direct);
        let payload: serde_json::Value = serde_json::from_str(
            &direct
                .revolve_journaled_js(face, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 360.0)
                .unwrap(),
        )
        .unwrap();
        let solid = u32::try_from(payload["solid"].as_u64().unwrap()).unwrap();
        assert!((direct.volume(solid, 0.01).unwrap() - 18.0 * std::f64::consts::PI).abs() < 1e-8);
        let history = &payload["evolution"];
        assert_eq!(history["origin"], "construction");
        assert_eq!(history["completeness"]["accounted"], true);
        assert_eq!(history["completeness"]["resolved"], true);
        assert_eq!(
            history["faces"].as_array().unwrap().len(),
            direct.get_solid_faces(solid).unwrap().len()
        );
        assert_eq!(
            history["edges"].as_array().unwrap().len(),
            direct.get_solid_edges(solid).unwrap().len()
        );
        assert_eq!(
            history["vertices"].as_array().unwrap().len(),
            direct.get_solid_vertices(solid).unwrap().len()
        );
        for v2 in [false, true] {
            let mut kernel = BrepKernel::new();
            let face = cylinder_profile(&mut kernel);
            let ops =
                serde_json::json!([{"op":"revolveJournaled", "args":{"face":face,"angle":360.0}}]);
            let response = if v2 {
                kernel.execute_batch_v2(&ops.to_string())
            } else {
                kernel.execute_batch(&ops.to_string())
            };
            let rows: serde_json::Value = serde_json::from_str(&response).unwrap();
            assert_eq!(rows[0]["ok"], payload);
            let before = kernel.journal_summary();
            let direct_error = kernel
                .revolve_journaled_json(face, [0.0; 3], [0.0, 0.0, 1.0], 180.0)
                .unwrap_err();
            assert_eq!(kernel.journal_summary(), before);
            let expected = serde_json::to_value(direct_error).unwrap();
            assert_eq!(expected["category"], "unsupported");
            assert_eq!(
                expected["details"]["kernelCode"],
                "revolve_history_unsupported"
            );
            let ops =
                serde_json::json!([{"op":"revolveJournaled", "args":{"face":face,"angle":180.0}}]);
            let response = if v2 {
                kernel.execute_batch_v2(&ops.to_string())
            } else {
                kernel.execute_batch(&ops.to_string())
            };
            let rows: serde_json::Value = serde_json::from_str(&response).unwrap();
            assert!(rows[0].get("error").is_some());
            if v2 {
                assert_eq!(rows[0]["error"]["code"], expected["code"]);
                assert_eq!(rows[0]["error"]["category"], expected["category"]);
            }
            assert_eq!(kernel.journal_summary(), before);
        }
    }
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Entity evolution (Issue 12) ─────────────────────────────────

    /// Fuse with full construction-derived vertex/edge/face history.
    ///
    /// Returns JSON `{"solid", "evolution": {"faces", "edges",
    /// "vertices"}}`; edge events are `preserved`/`modified` (with
    /// `from`), `generated` (with the generating `faceA`/`faceB` when
    /// they map), or the honest `unresolved`.
    #[wasm_bindgen(js_name = "fuseWithEntityEvolution")]
    pub fn fuse_with_entity_evolution(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.boolean_entity_evolution_json(BooleanOp::Fuse, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Cut with full entity history; see `fuseWithEntityEvolution`.
    #[wasm_bindgen(js_name = "cutWithEntityEvolution")]
    pub fn cut_with_entity_evolution(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.boolean_entity_evolution_json(BooleanOp::Cut, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Intersect with full entity history; see `fuseWithEntityEvolution`.
    #[wasm_bindgen(js_name = "intersectWithEntityEvolution")]
    pub fn intersect_with_entity_evolution(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.boolean_entity_evolution_json(BooleanOp::Intersect, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// V2 fillet journaled as one evolution entry (kind `fillet`).
    ///
    /// Returns JSON `{"solid", "op", "isPartial", "failedEdges"}`.
    #[wasm_bindgen(js_name = "filletJournaled")]
    pub fn fillet_journaled_js(
        &mut self,
        solid: u32,
        edges: Vec<u32>,
        radius: f64,
    ) -> Result<String, JsError> {
        validate_finite(radius, "radius")?;
        self.fillet_journaled_json(solid, &edges, radius)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// V2 chamfer journaled as one evolution entry (kind `chamfer`).
    #[wasm_bindgen(js_name = "chamferJournaled")]
    pub fn chamfer_journaled_js(
        &mut self,
        solid: u32,
        edges: Vec<u32>,
        d1: f64,
        d2: f64,
    ) -> Result<String, JsError> {
        validate_finite(d1, "d1")?;
        validate_finite(d2, "d2")?;
        self.chamfer_journaled_json(solid, &edges, d1, d2)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Linear pattern journaled as one evolution entry (kind
    /// `linear_pattern`). Returns JSON `{"compound", "op"}`.
    #[wasm_bindgen(js_name = "linearPatternJournaled")]
    pub fn linear_pattern_journaled_js(
        &mut self,
        solid: u32,
        dx: f64,
        dy: f64,
        dz: f64,
        spacing: f64,
        count: u32,
    ) -> Result<String, JsError> {
        for (name, value) in [("dx", dx), ("dy", dy), ("dz", dz), ("spacing", spacing)] {
            validate_finite(value, name)?;
        }
        self.linear_pattern_journaled_json(solid, [dx, dy, dz], spacing, count)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Circular pattern journaled as one evolution entry (kind
    /// `circular_pattern`). Returns JSON `{"compound", "op"}`.
    #[wasm_bindgen(js_name = "circularPatternJournaled")]
    pub fn circular_pattern_journaled_js(
        &mut self,
        solid: u32,
        ax: f64,
        ay: f64,
        az: f64,
        count: u32,
    ) -> Result<String, JsError> {
        for (name, value) in [("ax", ax), ("ay", ay), ("az", az)] {
            validate_finite(value, name)?;
        }
        self.circular_pattern_journaled_json(solid, [ax, ay, az], count)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Grid pattern journaled as one construction-derived F/E/V evolution
    /// entry (kind `grid_pattern`). Returns JSON `{"compound", "op"}`.
    /// Directions are independently normalized and must be nonparallel.
    /// Members are row-major (X changes first), including the original first.
    /// Counts must be positive integers, each and their product at most 10000.
    #[wasm_bindgen(js_name = "gridPatternJournaled")]
    #[allow(clippy::too_many_arguments)]
    pub fn grid_pattern_journaled_js(
        &mut self,
        solid: u32,
        dxx: f64,
        dxy: f64,
        dxz: f64,
        dyx: f64,
        dyy: f64,
        dyz: f64,
        spacing_x: f64,
        spacing_y: f64,
        count_x: f64,
        count_y: f64,
    ) -> Result<String, JsError> {
        self.grid_pattern_journaled_json(
            solid,
            [dxx, dxy, dxz],
            [dyx, dyy, dyz],
            spacing_x,
            spacing_y,
            count_x,
            count_y,
        )
        .map(|value| value.to_string())
        .map_err(structured_to_js)
    }

    /// Imprints one solid's intersection edges onto another without removing
    /// material. Returns JSON `{"solid", "op"}` for the new target and its
    /// construction-derived journal entry.
    #[wasm_bindgen(js_name = "imprint")]
    pub fn imprint_js(&mut self, target: u32, tool: u32) -> Result<String, JsError> {
        self.imprint_json(target, tool)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Run configured verified healing with entity history.
    /// Returns the `fixShapeWithConfig` report plus `op`. Untracked replacements
    /// remain unresolved. Batch uses `solid` and the JSON string `configJson`.
    #[wasm_bindgen(js_name = "fixShapeWithConfigJournaled")]
    pub fn fix_shape_journaled_js(
        &mut self,
        solid: u32,
        config_json: &str,
    ) -> Result<String, JsError> {
        self.fix_shape_journaled_json(solid, config_json)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Run verified healing steps with composed entity history.
    /// Returns the `runHealPipeline` report plus `op`. Topology and journal
    /// changes roll back together on failure. Batch uses `solid` and `steps`.
    #[wasm_bindgen(js_name = "runHealPipelineJournaled")]
    pub fn heal_pipeline_journaled_js(
        &mut self,
        solid: u32,
        steps: Vec<String>,
    ) -> Result<String, JsError> {
        self.heal_pipeline_journaled_json(solid, steps)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Remove selected feature faces with construction history.
    ///
    /// Returns JSON `{"solid", "op"}`. Capping retains copied boundary identities
    /// and records consumed boundaries as deleted. Unqualified reconstructed
    /// boundaries remain unresolved. Batch uses `solid` and `faces`.
    #[wasm_bindgen(js_name = "defeatureJournaled")]
    pub fn defeature_journaled_js(&mut self, solid: u32, faces: &[u32]) -> Result<String, JsError> {
        self.defeature_journaled_json(solid, faces)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Draft selected planar faces with construction history.
    ///
    /// `pull_direction` and `neutral_point` each have three components.
    /// The angle is in degrees for both direct and batch calls. Batch uses
    /// `faces`, `pullDirection`, `neutralPoint`, and `angleDegrees`.
    /// Returns JSON `{"solid", "op"}`. Boundary history requires a unique
    /// complete incidence correspondence; ambiguous boundaries stay unresolved.
    #[wasm_bindgen(js_name = "draftJournaled")]
    pub fn draft_journaled_js(
        &mut self,
        solid: u32,
        faces: &[u32],
        pull_direction: &[f64],
        neutral_point: &[f64],
        angle_degrees: f64,
    ) -> Result<String, JsError> {
        self.draft_journaled_json(solid, faces, pull_direction, neutral_point, angle_degrees)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Resize one cylindrical blend between planar supports with total history.
    ///
    /// Returns JSON `{"solid", "op"}`. Zero removes the band with explicit
    /// merges and deletions; ambiguous correspondence refuses atomically.
    /// Batch arguments use `expectedRadius` and `newRadius`.
    #[wasm_bindgen(js_name = "resizeBlendJournaled")]
    pub fn resize_blend_journaled_js(
        &mut self,
        solid: u32,
        face: u32,
        expected_radius: f64,
        new_radius: f64,
    ) -> Result<String, JsError> {
        self.resize_blend_journaled_json(solid, face, expected_radius, new_radius)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Remove selected blend regions atomically with complete construction history.
    ///
    /// Returns JSON `{"solid", "op"}`. Seeds expand to recognized blend regions;
    /// unsupported reconstruction or ambiguous history refuses without mutation.
    /// Batch arguments are `solid` and the face-handle array `seeds`.
    #[wasm_bindgen(js_name = "removeBlendsJournaled")]
    pub fn remove_blends_journaled_js(
        &mut self,
        solid: u32,
        seeds: &[u32],
    ) -> Result<String, JsError> {
        self.remove_blends_journaled_json(solid, seeds)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Resize a cylindrical wall and record its construction history.
    ///
    /// Returns JSON `{"solid", "op"}`. Qualified bore and quarter-wall
    /// replacements retain all boundary identities. Boss edits track cap
    /// subdivisions and their removal; ambiguous boundaries remain unresolved.
    /// Batch calls pass the new radius as `args.radius`.
    #[wasm_bindgen(js_name = "resizeCylindricalFaceJournaled")]
    pub fn resize_cylindrical_face_journaled_js(
        &mut self,
        solid: u32,
        face: u32,
        radius: f64,
    ) -> Result<String, JsError> {
        self.resize_cylindrical_face_journaled_json(solid, face, radius)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Replace a support surface with exact face, edge, and vertex history.
    ///
    /// `replacement` is JSON: `{type:"plane", normal:[x,y,z], d}` or
    /// `{type:"cylinder", origin:[x,y,z], axis:[x,y,z], radius}`. Plane
    /// coefficients represent `normal dot point = d`; cylinder replacements
    /// retain the source parameter reference direction. Unknown fields refuse.
    /// Returns JSON `{"solid", "op"}`. Batch calls pass the replacement object
    /// directly as `args.replacement`.
    #[wasm_bindgen(js_name = "replaceSurfaceJournaled")]
    pub fn replace_surface_journaled_js(
        &mut self,
        solid: u32,
        face: u32,
        replacement: &str,
    ) -> Result<String, JsError> {
        let replacement = parse_replacement_json(replacement).map_err(structured_to_js)?;
        self.replace_surface_journaled_json(solid, face, &replacement)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Move faces with construction history.
    ///
    /// Planar re-limitation and coaxial bore moves include edge and vertex
    /// history. Blend moves retain copy-derived history or a complete, unique
    /// boundary correspondence anchored on construction face identities.
    /// Ambiguous reconstructed boundaries retain faces-only history.
    /// Returns JSON `{"solid", "op"}`.
    #[wasm_bindgen(js_name = "moveFacesJournaled")]
    pub fn move_faces_journaled_js(
        &mut self,
        solid: u32,
        faces: &[u32],
        distance: f64,
    ) -> Result<String, JsError> {
        validate_finite(distance, "distance")?;
        self.move_faces_journaled_json(solid, faces, distance)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// V2 offset journaled as one construction-derived face, edge and vertex
    /// evolution entry (kind `offset`).
    ///
    /// Returns JSON `{"solid", "op", "evolution"}`. `evolution` lists every
    /// result face, edge and vertex as `modified`, `merged`, `generated` or
    /// `unresolved` (with `candidates` and a typed `reason`, such as
    /// `ambiguous_incidence` for a torus seam), plus a `completeness` report
    /// (`accounted`, `resolved`, and per-kind `omitted`/`phantom`/`unresolved`
    /// lists) checked against the actual result.
    #[wasm_bindgen(js_name = "offsetJournaled")]
    pub fn offset_journaled_js(&mut self, solid: u32, distance: f64) -> Result<String, JsError> {
        validate_finite(distance, "distance")?;
        self.offset_journaled_json(solid, distance)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Extrusion journaled as one construction-derived face, edge and vertex
    /// evolution entry (kind `extrude`).
    ///
    /// Returns JSON `{"solid", "op", "evolution"}`. `evolution` lists every
    /// result face, edge and vertex as `modified` or `generated` (both caps
    /// `modified` from the profile face, every side wall `generated` from
    /// it, shared boundary entities `modified` into themselves, translated
    /// copies `modified` from their source, longitudinal edges `generated`
    /// from the profile face), plus a `completeness` report (`accounted`,
    /// `resolved`, and per-kind `omitted`/`phantom`/`unresolved` lists)
    /// checked against the actual result. Nothing is deleted; the qualified
    /// profile classes leave nothing unresolved.
    #[wasm_bindgen(js_name = "extrudeJournaled")]
    pub fn extrude_journaled_js(
        &mut self,
        face: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        distance: f64,
    ) -> Result<String, JsError> {
        self.extrude_journaled_json(face, [dir_x, dir_y, dir_z], distance)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Full-turn analytic radial-profile revolution with complete construction
    /// history. Angles are degrees, as in `revolve`.
    ///
    /// Returns JSON `{"solid", "op", "evolution"}`. Faces and circular rims
    /// are generated from the profile face; reused profile seams and vertices
    /// are modified into themselves. Completeness is checked against every
    /// result entity. Only planar hole-free straight-edge radial profiles have
    /// qualified history; other profiles and partial turns refuse atomically.
    #[wasm_bindgen(js_name = "revolveJournaled")]
    #[allow(clippy::too_many_arguments)]
    pub fn revolve_journaled_js(
        &mut self,
        face: u32,
        origin_x: f64,
        origin_y: f64,
        origin_z: f64,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        angle_degrees: f64,
    ) -> Result<String, JsError> {
        self.revolve_journaled_json(
            face,
            [origin_x, origin_y, origin_z],
            [axis_x, axis_y, axis_z],
            angle_degrees,
        )
        .map(|value| value.to_string())
        .map_err(structured_to_js)
    }

    /// A read-only summary of the evolution journal: JSON array of
    /// `{"op", "kind", "type", "detail"}` where `type` is `evolution`
    /// (detail: origin, event count), `barrier` (detail: affected
    /// count), or `globalBarrier`.
    #[wasm_bindgen(js_name = "journalSummary")]
    #[must_use]
    pub fn journal_summary(&self) -> String {
        self.journal_summary_json().to_string()
    }
}

fn structured_to_js(error: StructuredWasmError) -> JsError {
    JsError::new(error.message())
}

#[cfg(test)]
mod grid_pattern_contract_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use crate::kernel::BrepKernel;
    use remus_topology::journal::{EntityEvent, EntityKey, EntityKind, EntryPayload};
    use serde_json::{Value, json};
    use std::collections::{BTreeMap, BTreeSet};

    const KINDS: [EntityKind; 3] = [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex];

    fn handle(value: &Value) -> u32 {
        u32::try_from(value.as_u64().unwrap()).unwrap()
    }

    fn entities(kernel: &BrepKernel, solid: u32, kind: EntityKind) -> BTreeSet<EntityKey> {
        let handles = match kind {
            EntityKind::Face => kernel.get_solid_faces(solid).unwrap(),
            EntityKind::Edge => kernel.get_solid_edges(solid).unwrap(),
            EntityKind::Vertex => kernel.get_solid_vertices(solid).unwrap(),
        };
        handles
            .into_iter()
            .map(|index| EntityKey {
                kind,
                index: index as usize,
            })
            .collect()
    }

    fn points(kernel: &BrepKernel, key: EntityKey) -> Vec<Vec<f64>> {
        let index = u32::try_from(key.index).unwrap();
        let vertices = match key.kind {
            EntityKind::Face => kernel.get_face_vertices(index).unwrap(),
            EntityKind::Edge => kernel.get_edge_vertex_handles(index).unwrap(),
            EntityKind::Vertex => vec![index],
        };
        vertices
            .into_iter()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .map(|vertex| kernel.get_vertex_position(vertex).unwrap())
            .collect()
    }

    fn assert_shift(
        kernel: &BrepKernel,
        source: EntityKey,
        subject: EntityKey,
        offset: [f64; 3],
        scale: f64,
    ) {
        let wanted = points(kernel, source);
        let actual = points(kernel, subject);
        assert_eq!(wanted.len(), actual.len());
        for point in wanted {
            assert!(
                actual
                    .iter()
                    .any(|candidate| (0..3).all(|axis| (candidate[axis]
                        - point[axis]
                        - offset[axis])
                        .abs()
                        <= 1e-7 * scale)),
                "{source:?} -> {subject:?} must have the independently expected translation {offset:?}"
            );
        }
    }

    fn batch(kernel: &mut BrepKernel, route: u8, ops: Value) -> Value {
        serde_json::from_str(&if route == 2 {
            kernel.execute_batch_v2(&ops.to_string())
        } else {
            kernel.execute_batch(&ops.to_string())
        })
        .unwrap()
    }

    fn grid(kernel: &mut BrepKernel, route: u8, solid: u32, settings: &Value) -> Value {
        if route == 0 {
            let x = &settings["dirX"];
            let y = &settings["dirY"];
            serde_json::from_str(
                &kernel
                    .grid_pattern_journaled_js(
                        solid,
                        x[0].as_f64().unwrap(),
                        x[1].as_f64().unwrap(),
                        x[2].as_f64().unwrap(),
                        y[0].as_f64().unwrap(),
                        y[1].as_f64().unwrap(),
                        y[2].as_f64().unwrap(),
                        settings["spacingX"].as_f64().unwrap(),
                        settings["spacingY"].as_f64().unwrap(),
                        settings["countX"].as_f64().unwrap(),
                        settings["countY"].as_f64().unwrap(),
                    )
                    .unwrap(),
            )
            .unwrap()
        } else {
            let mut args = settings.clone();
            args["solid"] = json!(solid);
            let response = batch(
                kernel,
                route,
                json!([{"op":"gridPatternJournaled", "args":args}]),
            );
            assert!(response[0].get("error").is_none(), "{response}");
            response[0]["ok"].clone()
        }
    }

    fn bound(kernel: &BrepKernel, reference: &str, expected: EntityKey) {
        let resolution: Value =
            serde_json::from_str(&kernel.resolve_ref(reference).unwrap()).unwrap();
        assert_eq!(resolution["status"], "bound", "{reference}: {resolution}");
        assert_eq!(resolution["provenance"], "construction");
        assert_eq!(
            resolution["entities"],
            json!([{"kind":expected.kind.as_str(), "handle":expected.index}])
        );
    }

    fn references(
        kernel: &BrepKernel,
        op: u32,
        expected: &BTreeSet<EntityKey>,
    ) -> Vec<(String, EntityKey)> {
        let mut references = Vec::new();
        for kind in KINDS {
            let mut seen = BTreeSet::new();
            let count = expected.iter().filter(|key| key.kind == kind).count();
            for index in 0..count {
                let index = u32::try_from(index).unwrap();
                let resolution: Value = serde_json::from_str(
                    &kernel
                        .resolve_operation_output(op, kind.as_str(), index)
                        .unwrap(),
                )
                .unwrap();
                assert_eq!(resolution["status"], "bound");
                assert_eq!(resolution["provenance"], "construction");
                assert_eq!(resolution["entities"].as_array().unwrap().len(), 1);
                assert_eq!(resolution["entities"][0]["kind"], kind.as_str());
                let key = EntityKey {
                    kind,
                    index: handle(&resolution["entities"][0]["handle"]) as usize,
                };
                assert!(expected.contains(&key));
                assert!(seen.insert(key), "duplicate output subject");
                references.push((
                    kernel
                        .make_operation_output_ref(op, kind.as_str(), index)
                        .unwrap(),
                    key,
                ));
            }
            assert_eq!(
                seen,
                expected
                    .iter()
                    .filter(|key| key.kind == kind)
                    .copied()
                    .collect()
            );
            let past_end: Value = serde_json::from_str(
                &kernel
                    .resolve_operation_output(op, kind.as_str(), u32::try_from(count).unwrap())
                    .unwrap(),
            )
            .unwrap();
            assert_eq!(past_end["status"], "noMatch");
        }
        references
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn grid_pattern_journaled_batch_surface_exists_and_censuses_all_kinds() {
        for route in 0..3 {
            for (count_x, count_y, scale, skew) in [
                (1, 1, 1.0, false),
                (1, 3, 0.001, false),
                (3, 1, 1000.0, false),
                (2, 3, 1.0, false),
                (2, 2, 1.0, true),
            ] {
                let mut kernel = BrepKernel::new();
                let source = kernel
                    .make_box_solid(10.0 * scale, 10.0 * scale, 10.0 * scale)
                    .unwrap();
                let anchor = kernel
                    .linear_pattern_journaled_json(source, [1.0, 0.0, 0.0], 25.0 * scale, 1)
                    .unwrap();
                let source_keys: BTreeSet<_> = KINDS
                    .into_iter()
                    .flat_map(|kind| entities(&kernel, source, kind))
                    .collect();
                let original_refs = references(&kernel, handle(&anchor["op"]), &source_keys);
                let checkpoint = kernel.checkpoint_impl().unwrap();
                let before = kernel.serialize_solids(&[source]).unwrap();
                let settings = json!({"dirX":[2.0,0.0,0.0], "dirY":if skew { [3.0,3.0,0.0] } else { [0.0,3.0,0.0] },
                "spacingX":25.0*scale,"spacingY":25.0*scale,"countX":count_x,"countY":count_y});
                let result = grid(&mut kernel, route, source, &settings);
                let op = handle(&result["op"]);
                let compound = handle(&result["compound"]);
                let members = kernel.get_compound_solids(compound).unwrap();
                assert_eq!(members.len(), count_x * count_y);
                assert_eq!(members[0], source);
                let mut live = BTreeSet::new();
                let mut offsets = BTreeMap::new();
                for (index, &member) in members.iter().enumerate() {
                    let ix = (index % count_x) as f64;
                    let iy = (index / count_x) as f64;
                    let dy = if skew { 1.0 / 2.0_f64.sqrt() } else { 0.0 };
                    let offset = [
                        (ix + iy * dy) * 25.0 * scale,
                        iy * 25.0 * scale * if skew { dy } else { 1.0 },
                        0.0,
                    ];
                    let keys: BTreeSet<_> = KINDS
                        .into_iter()
                        .flat_map(|kind| entities(&kernel, member, kind))
                        .collect();
                    assert_eq!(keys.len(), source_keys.len());
                    for key in keys {
                        assert!(live.insert(key));
                        offsets.insert(key, offset);
                    }
                    assert!(
                        (kernel.volume(member, 0.01 * scale).unwrap() - 1000.0 * scale.powi(3))
                            .abs()
                            <= 1e-6 * scale.powi(3)
                    );
                }
                let result_refs = references(&kernel, op, &live);
                let entry = kernel
                    .topo()
                    .journal()
                    .entries()
                    .iter()
                    .find(|entry| entry.op().value() == u64::from(op))
                    .unwrap();
                assert_eq!(entry.kind(), "grid_pattern");
                let EntryPayload::Evolution { events, origin, .. } = entry.payload() else {
                    panic!("grid must publish evolution")
                };
                assert_eq!(
                    *origin,
                    remus_topology::journal::RecordedOrigin::Construction
                );
                assert_eq!(events.len(), live.len());
                let mut subjects = BTreeSet::new();
                let mut generated = BTreeMap::<EntityKey, usize>::new();
                for (ordinal, event) in events {
                    let subject = kernel.topo().journal().key_of(*ordinal).unwrap();
                    assert!(subjects.insert(subject));
                    let from = match event {
                        EntityEvent::Modified { from } => {
                            let from = kernel.topo().journal().key_of(*from).unwrap();
                            assert_eq!(from, subject);
                            assert!(source_keys.contains(&subject));
                            from
                        }
                        EntityEvent::Generated { sources } => {
                            assert_eq!(sources.len(), 1);
                            let from = kernel.topo().journal().key_of(sources[0]).unwrap();
                            assert!(source_keys.contains(&from));
                            assert!(!source_keys.contains(&subject));
                            assert_eq!(from.kind, subject.kind);
                            *generated.entry(from).or_default() += 1;
                            from
                        }
                        other => panic!("unexpected grid disposition: {other:?}"),
                    };
                    assert_shift(&kernel, from, subject, offsets[&subject], scale);
                }
                assert_eq!(subjects, live);
                for source_key in &source_keys {
                    assert_eq!(
                        generated.get(source_key).copied().unwrap_or(0),
                        members.len() - 1
                    );
                }
                for (reference, key) in &original_refs {
                    bound(&kernel, reference, *key);
                }

                // A real edit to the last copy must preserve every untouched
                // original and copied-member reference, including E/V.
                if count_x == 2 && count_y == 3 {
                    let bytes = kernel.serialize_solids(&members).unwrap();
                    let mut imported = BrepKernel::new();
                    imported.make_box_solid(1.0, 1.0, 1.0).unwrap();
                    let roots = imported.deserialize_solids(&bytes).unwrap();
                    let imported_keys: BTreeSet<_> = roots
                        .iter()
                        .flat_map(|&root| {
                            KINDS
                                .into_iter()
                                .flat_map(|kind| entities(&imported, root, kind))
                                .collect::<Vec<_>>()
                        })
                        .collect();
                    let imported_refs = references(&imported, op, &imported_keys);
                    assert_eq!(imported_refs.len(), result_refs.len());
                    for ((reference, _), (_, expected)) in result_refs.iter().zip(&imported_refs) {
                        bound(&imported, reference, *expected);
                    }
                    let edited = *members.last().unwrap();
                    let untouched: BTreeSet<_> = members[..members.len() - 1]
                        .iter()
                        .flat_map(|&root| {
                            KINDS
                                .into_iter()
                                .flat_map(|kind| entities(&kernel, root, kind))
                                .collect::<Vec<_>>()
                        })
                        .collect();
                    let face = kernel.get_solid_faces(edited).unwrap()[0];
                    kernel
                        .move_faces_journaled_json(edited, &[face], scale)
                        .unwrap();
                    for (reference, key) in original_refs.iter().chain(
                        result_refs
                            .iter()
                            .filter(|(_, key)| untouched.contains(key)),
                    ) {
                        bound(&kernel, reference, *key);
                    }
                }
                kernel.restore_checkpoint_impl(checkpoint).unwrap();
                let mut restored: Value =
                    serde_json::from_slice(&kernel.serialize_solids(&[source]).unwrap()).unwrap();
                let mut saved: Value = serde_json::from_slice(&before).unwrap();
                // Restore deliberately preserves only the identifier high-water
                // counters; exact geometry and retained history must match.
                for counter in ["next_op", "next_ordinal"] {
                    assert!(
                        restored["journal"][counter].as_u64().unwrap()
                            >= saved["journal"][counter].as_u64().unwrap()
                    );
                    restored["journal"].as_object_mut().unwrap().remove(counter);
                    saved["journal"].as_object_mut().unwrap().remove(counter);
                }
                assert_eq!(restored, saved);
                assert!(kernel.resolve_compound(compound).is_err());
                for (reference, key) in &original_refs {
                    bound(&kernel, reference, *key);
                }
                for (reference, _) in &result_refs {
                    let resolution: Value =
                        serde_json::from_str(&kernel.resolve_ref(reference).unwrap()).unwrap();
                    assert_eq!(resolution["status"], "unknownOperation");
                }
                for &member in &members[1..] {
                    assert!(kernel.resolve_solid(member).is_err());
                }
                let next = grid(&mut kernel, route, source, &settings);
                assert!(handle(&next["op"]) > op);
                for &member in &members[1..] {
                    assert!(kernel.resolve_solid(member).is_err());
                }
                for (reference, _) in &result_refs {
                    let resolution: Value =
                        serde_json::from_str(&kernel.resolve_ref(reference).unwrap()).unwrap();
                    assert_eq!(resolution["status"], "unknownOperation");
                }
            }
        }
    }

    #[test]
    fn grid_pattern_journaled_rejects_malformed_counts_and_rolls_back_both_batches() {
        for route in 1..3 {
            let mut kernel = BrepKernel::new();
            let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
            let retired = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
            let retired_id = kernel.resolve_solid(retired).unwrap();
            kernel.topo_mut().delete_solid(retired_id).unwrap();
            let anchor = kernel
                .linear_pattern_journaled_json(source, [1.0, 0.0, 0.0], 25.0, 1)
                .unwrap();
            let source_keys: BTreeSet<_> = KINDS
                .into_iter()
                .flat_map(|kind| entities(&kernel, source, kind))
                .collect();
            let refs = references(&kernel, handle(&anchor["op"]), &source_keys);
            let normal = json!({"solid":source,"dirX":[1,0,0],"dirY":[0,1,0],"spacingX":25,"spacingY":25,"countX":2,"countY":2});
            let mut invalids = vec![
                json!({"solid":retired}),
                json!({"dirX":[]}),
                json!({"dirX":[1,0]}),
                json!({"dirY":[0,1,0,2]}),
                json!({"dirX":[1,"x",0]}),
                json!({"dirY":null}),
                json!({"countX":"2"}),
                json!({"countY":null}),
                json!({"countX":101,"countY":100}),
                json!({"spacingX":0}),
                json!({"spacingY":-1}),
                json!({"spacingX":1e-12}),
                json!({"dirX":[0,0,0]}),
                json!({"dirY":[1,0,0]}),
                json!({"dirY":[-1,0,0]}),
            ];
            for value in [0.0, -1.0, 1.5, 4_294_967_297.0, 10_001.0] {
                invalids.push(json!({"countX":value}));
                invalids.push(json!({"countY":value}));
            }
            invalids.push(json!({"spacingX":5,"countY":1}));
            for overrides in invalids {
                let mut args = normal.clone();
                for (key, value) in overrides.as_object().unwrap() {
                    args[key] = value.clone();
                }
                let overlap = args["spacingX"] == 5;
                let before = kernel.serialize_solids(&[source]).unwrap();
                let journal = kernel.journal_summary();
                let before_memory = kernel
                    .topo()
                    .memory_estimate(&mut std::collections::HashSet::new());
                let response = batch(
                    &mut kernel,
                    route,
                    json!([{"op":"gridPatternJournaled","args":args},{"op":"volume","args":{"solid":source,"deflection":0.01}}]),
                );
                assert!(response[0].get("error").is_some(), "{args}: {response}");
                if route == 2 {
                    let error = &response[0]["error"];
                    if overlap {
                        assert_eq!(error["code"], "operation_failed");
                        assert_eq!(error["category"], "unsupported");
                        assert_eq!(error["details"]["kernelCode"], "pattern_instances_overlap");
                    } else if args["solid"] == retired {
                        assert_eq!(error["code"], "invalid_handle");
                    } else {
                        assert_eq!(error["code"], "invalid_argument", "{args}: {error}");
                    }
                }
                assert!((response[1]["ok"].as_f64().unwrap() - 1000.0).abs() < 1e-6);
                assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);
                assert_eq!(kernel.journal_summary(), journal);
                let after_memory = kernel
                    .topo()
                    .memory_estimate(&mut std::collections::HashSet::new());
                assert_eq!(
                    after_memory.allocated_slots - after_memory.retired_slots,
                    before_memory.allocated_slots - before_memory.retired_slots,
                    "refusal must preserve every live entity arena"
                );
                if overlap {
                    assert!(after_memory.allocated_slots > before_memory.allocated_slots);
                    assert!(after_memory.retired_slots > before_memory.retired_slots);
                } else {
                    assert_eq!(after_memory.allocated_slots, before_memory.allocated_slots);
                    assert_eq!(after_memory.retired_slots, before_memory.retired_slots);
                }
                for (reference, key) in &refs {
                    bound(&kernel, reference, *key);
                }
            }
            // Non-finite JS numbers cannot appear in JSON. Test the shared
            // direct validator without constructing a native-only JsError.
            for (x, y, spacing) in [
                (f64::NAN, 2.0, 25.0),
                (2.0, f64::INFINITY, 25.0),
                (2.0, 2.0, f64::NAN),
            ] {
                assert!(
                    kernel
                        .grid_pattern_journaled_json(
                            source,
                            [1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            spacing,
                            25.0,
                            x,
                            y
                        )
                        .is_err()
                );
            }
        }
    }
}

#[cfg(test)]
mod evolution_contract_tests {
    #![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

    use crate::kernel::BrepKernel;

    fn run(kernel: &mut BrepKernel, ops: serde_json::Value) -> Vec<serde_json::Value> {
        let response = kernel.execute_batch(&ops.to_string());
        let parsed: serde_json::Value =
            serde_json::from_str(&response).expect("batch response must be valid JSON");
        parsed
            .as_array()
            .expect("batch response is an array")
            .iter()
            .map(|entry| {
                assert!(
                    entry.get("error").is_none(),
                    "unexpected batch error: {entry}"
                );
                entry["ok"].clone()
            })
            .collect()
    }

    #[test]
    fn whole_body_journaled_fillet_filters_periodic_seams_in_direct_and_batch_routes() {
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let made = run(
                &mut kernel,
                serde_json::json!([
                    {"op":"makeCylinder", "args":{"radius":5.0, "height":10.0}},
                    {"op":"solidEdges", "args":{"solid":0}}
                ]),
            );
            let edges: Vec<u32> = made[1]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| u32::try_from(v.as_u64().unwrap()).unwrap())
                .collect();
            let result =
                if batch {
                    run(&mut kernel, serde_json::json!([
                    {"op":"filletJournaled", "args":{"solid":0, "edges":edges, "radius":1.0}}
                ]))[0].clone()
                } else {
                    kernel.fillet_journaled_json(0, &edges, 1.0).unwrap()
                };
            assert_eq!(result["isPartial"], false);
            assert_eq!(result["failedEdges"], serde_json::json!([]));
            assert!(result["op"].is_u64());
            let out = u32::try_from(result["solid"].as_u64().unwrap()).unwrap();
            let id = kernel.resolve_solid(out).unwrap();
            assert!(
                remus_operations::validate::validate_solid(kernel.topo(), id)
                    .unwrap()
                    .is_valid()
            );
            assert_eq!(kernel.topo().journal().len(), 1);
        }
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn journaled_healing_direct_and_batch_preserve_repaired_references() {
        use remus_topology::{
            explorer::solid_edges,
            journal::{EntityKind, EventDraft, EvolutionDraft},
            naming::{PersistentRef, Provenance, Resolution, resolve},
            vertex::Vertex,
        };
        for scale in [0.1, 1.0, 10.0] {
            for pipeline in [false, true] {
                for batch in [false, true] {
                    let mut kernel = BrepKernel::new();
                    let solid = kernel
                        .make_box_solid(10.0 * scale, 10.0 * scale, 10.0 * scale)
                        .unwrap();
                    let id = kernel.resolve_solid(solid).unwrap();
                    let edge = solid_edges(kernel.topo(), id).unwrap()[0];
                    let original = kernel.topo().edge(edge).unwrap().start();
                    let point = kernel.topo().vertex(original).unwrap().point();
                    let duplicate = kernel.topo_mut().add_vertex(Vertex::new(point, 1e-7));
                    kernel
                        .topo_mut()
                        .edge_mut(edge)
                        .unwrap()
                        .set_start(duplicate);
                    let keys = remus_operations::journal_ops::solid_entity_keys(kernel.topo(), id)
                        .unwrap();
                    let pending = remus_operations::journal_ops::begin_scoped(
                        kernel.topo_mut(),
                        "repair_fixture",
                        &[id],
                    )
                    .unwrap();
                    let mut draft = EvolutionDraft::construction();
                    for &key in &keys {
                        draft.push(
                            key,
                            EventDraft::Generated {
                                sources: Vec::new(),
                            },
                        );
                    }
                    let anchor = kernel
                        .topo_mut()
                        .journal_record_evolution(pending, draft)
                        .unwrap();
                    let steps = vec![
                        "fix_shape".to_string(),
                        "merge_vertices".to_string(),
                        "fix_shape".to_string(),
                    ];
                    let payload = if batch {
                        let operation = if pipeline {
                            "runHealPipelineJournaled"
                        } else {
                            "fixShapeWithConfigJournaled"
                        };
                        run(&mut kernel, serde_json::json!([{"op":operation,"args":{"solid":solid,"steps":steps,"configJson":"{}"}}])).remove(0)
                    } else if pipeline {
                        serde_json::from_str(
                            &kernel.heal_pipeline_journaled_js(solid, steps).unwrap(),
                        )
                        .unwrap()
                    } else {
                        serde_json::from_str(&kernel.fix_shape_journaled_js(solid, "{}").unwrap())
                            .unwrap()
                    };
                    assert_eq!(payload["verified"], true);
                    assert!(payload["op"].as_u64().is_some());
                    let mut resolved = std::collections::BTreeSet::new();
                    for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                        for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                            match resolve(
                                kernel.topo(),
                                &PersistentRef::operation_output(anchor, kind, index),
                            ) {
                                Resolution::Bound {
                                    entity,
                                    provenance: Provenance::Construction,
                                } => {
                                    resolved.insert(entity);
                                }
                                other => panic!(
                                    "healing {scale}/{pipeline}/{batch} lost reference: {other:?}"
                                ),
                            }
                        }
                    }
                    assert_eq!(
                        resolved,
                        remus_operations::journal_ops::solid_entity_keys(kernel.topo(), id)
                            .unwrap()
                            .into_iter()
                            .collect()
                    );
                    let volume =
                        remus_operations::measure::solid_volume(kernel.topo(), id, 0.01 * scale)
                            .unwrap();
                    assert!((volume - 1000.0 * scale.powi(3)).abs() < 1e-6 * scale.powi(3));
                    let bytes = kernel.serialize_solids(&[solid]).unwrap();
                    let mut restored = BrepKernel::new();
                    restored.make_box_solid(1.0, 1.0, 1.0).unwrap();
                    let restored_solid = restored.deserialize_solids(&bytes).unwrap()[0];
                    let restored_id = restored.resolve_solid(restored_solid).unwrap();
                    let wall = remus_topology::explorer::solid_faces(restored.topo(), restored_id)
                        .unwrap()
                        .into_iter()
                        .find(|&face| {
                            restored
                                .topo()
                                .face(face)
                                .unwrap()
                                .effective_plane_normal()
                                .is_some_and(|normal| normal.x() > 0.9)
                        })
                        .unwrap();
                    let edited = restored
                        .draft_journaled_json(
                            restored_solid,
                            &[super::index_u32(wall.index())],
                            &[0.0, 0.0, 1.0],
                            &[0.0; 3],
                            5.0,
                        )
                        .unwrap();
                    let final_solid = restored
                        .resolve_solid(u32::try_from(edited["solid"].as_u64().unwrap()).unwrap())
                        .unwrap();
                    let mut final_refs = std::collections::BTreeSet::new();
                    for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                        for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                            match resolve(
                                restored.topo(),
                                &PersistentRef::operation_output(anchor, kind, index),
                            ) {
                                Resolution::Bound {
                                    entity,
                                    provenance: Provenance::Construction,
                                } => {
                                    final_refs.insert(entity);
                                }
                                other => panic!(
                                    "healing reference lost after arena and draft: {other:?}"
                                ),
                            }
                        }
                    }
                    assert_eq!(
                        final_refs,
                        remus_operations::journal_ops::solid_entity_keys(
                            restored.topo(),
                            final_solid
                        )
                        .unwrap()
                        .into_iter()
                        .collect()
                    );
                    let expected = (1000.0 + 500.0 * 5.0_f64.to_radians().tan()) * scale.powi(3);
                    let volume = remus_operations::measure::solid_volume(
                        restored.topo(),
                        final_solid,
                        0.01 * scale,
                    )
                    .unwrap();
                    assert!((volume - expected).abs() < expected * 1e-6);
                }
            }
        }
    }

    #[test]
    fn journaled_healing_batch_rejects_bad_inputs_without_history_changes() {
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let id = kernel.resolve_solid(solid).unwrap();
        let keys = remus_operations::journal_ops::solid_entity_keys(kernel.topo(), id).unwrap();
        let before = kernel.topo().journal().snapshot();
        for (op, args) in [
            (
                "fixShapeWithConfigJournaled",
                serde_json::json!({"solid":solid,"configJson":"{\"tolerance\":-1}"}),
            ),
            (
                "fixShapeWithConfigJournaled",
                serde_json::json!({"solid":solid,"configJson":{}}),
            ),
            (
                "runHealPipelineJournaled",
                serde_json::json!({"solid":solid,"steps":[]}),
            ),
            (
                "runHealPipelineJournaled",
                serde_json::json!({"solid":solid,"steps":["fix_shape","missing"]}),
            ),
            (
                "runHealPipelineJournaled",
                serde_json::json!({"solid":solid,"steps":[1]}),
            ),
            (
                "runHealPipelineJournaled",
                serde_json::json!({"solid":solid,"steps":vec!["fix_shape";33]}),
            ),
        ] {
            let response: serde_json::Value = serde_json::from_str(
                &kernel.execute_batch_v2(&serde_json::json!([{"op":op,"args":args}]).to_string()),
            )
            .unwrap();
            assert!(response[0]["error"].is_object(), "{response}");
            assert_eq!(kernel.topo().journal().snapshot(), before);
            assert_eq!(
                remus_operations::journal_ops::solid_entity_keys(kernel.topo(), id).unwrap(),
                keys
            );
        }
    }

    fn assert_assembly_rebuilds_are_typed(evolution: &serde_json::Value) {
        let events = evolution["edges"].as_array().unwrap();
        assert!(
            events.iter().any(|edge| edge["event"] == "modified"),
            "fixture must exercise typed edge reconstruction: {events:?}"
        );
        assert!(
            events.iter().all(|edge| edge["event"] != "unresolved"),
            "recorded assembly rebuilds must not surface as unresolved: {events:?}"
        );
    }

    #[test]
    fn direct_wasm_entity_evolution_types_assembly_rebuilds() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let b = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        kernel
            .transform_solid_binding(
                b,
                vec![
                    1.0, 0.0, 0.0, 5.0, 0.0, 1.0, 0.0, 5.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 0.0, 1.0,
                ],
            )
            .unwrap();

        let payload: serde_json::Value =
            serde_json::from_str(&kernel.fuse_with_entity_evolution(a, b).unwrap()).unwrap();
        assert_assembly_rebuilds_are_typed(&payload["evolution"]);
    }

    #[test]
    fn entity_evolution_surfaces_all_three_claim_strengths() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "transform", "args": {"solid": 1, "matrix":
                    [1.0,0.0,0.0,5.0, 0.0,1.0,0.0,5.0, 0.0,0.0,1.0,5.0, 0.0,0.0,0.0,1.0]}},
                {"op": "fuseWithEntityEvolution", "args": {"solidA": 0, "solidB": 1}},
            ]),
        );
        let payload = &results[3];
        assert!(payload["solid"].as_u64().is_some());
        let evolution = &payload["evolution"];
        assert!(!evolution["faces"].as_array().unwrap().is_empty());
        assert!(!evolution["vertices"].as_array().unwrap().is_empty());

        let edge_events: Vec<&str> = evolution["edges"]
            .as_array()
            .unwrap()
            .iter()
            .map(|e| e["event"].as_str().unwrap())
            .collect();
        for expected in ["preserved", "modified", "generated"] {
            assert!(
                edge_events.contains(&expected),
                "a cube fuse must show {expected} edges: {edge_events:?}"
            );
        }
        assert_assembly_rebuilds_are_typed(evolution);
        // Generated edges name their generating faces when they map.
        assert!(
            evolution["edges"]
                .as_array()
                .unwrap()
                .iter()
                .any(|e| e["event"] == "generated" && e["faceA"].is_u64()),
            "section edges name their generating faces"
        );
    }

    #[test]
    fn journaled_blends_and_patterns_populate_the_journal() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "solidEdges", "args": {"solid": 0}},
            ]),
        );
        let edge = results[1].as_array().expect("solidEdges returns an array")[0]
            .as_u64()
            .expect("edge handle");

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "setFaceName", "args": {"face": 0, "name": "base"}},
                {"op": "filletJournaled", "args": {"solid": 0, "edges": [edge], "radius": 1.0}},
            ]),
        );
        let fillet = &results[1];
        let fillet_op = fillet["op"].as_u64().unwrap();
        assert_eq!(fillet["isPartial"], serde_json::json!(false));
        assert_eq!(fillet["failedEdges"], serde_json::json!([]));
        let fillet_solid = fillet["solid"].as_u64().unwrap();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "propagateAttributesForOp", "args": {"op": fillet_op}},
                {"op": "linearPatternJournaled", "args": {
                    "solid": fillet_solid, "direction": [1.0, 0.0, 0.0],
                    "spacing": 20.0, "count": 3}},
                {"op": "journalSummary", "args": {}},
            ]),
        );
        assert!(
            results[0]["carried"].as_u64().unwrap() > 0,
            "the named face must ride the fillet's journal entry: {}",
            results[0]
        );
        let pattern_op = results[1]["op"].as_u64().unwrap();
        assert!(results[1]["compound"].as_u64().is_some());

        // The journal now holds both entries as construction evolution.
        let summary = results[2].as_array().unwrap();
        let find = |op: u64| {
            summary
                .iter()
                .find(|entry| entry["op"].as_u64() == Some(op))
                .unwrap_or_else(|| panic!("op {op} missing from journal summary"))
        };
        let fillet_entry = find(fillet_op);
        assert_eq!(fillet_entry["kind"], "fillet");
        assert_eq!(fillet_entry["type"], "evolution");
        let pattern_entry = find(pattern_op);
        assert_eq!(pattern_entry["kind"], "linear_pattern");
        assert_eq!(pattern_entry["type"], "evolution");
        assert_eq!(pattern_entry["detail"]["origin"], "construction");

        // Pattern provenance carries names onto instances.
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "propagateAttributesForOp", "args": {"op": pattern_op}},
            ]),
        );
        assert!(
            results[0]["carried"].as_u64().unwrap() > 0,
            "instance faces must inherit the original's name: {}",
            results[0]
        );
    }

    fn replacement_box() -> (BrepKernel, u32, u32) {
        let mut kernel = BrepKernel::new();
        let source = kernel.make_box_solid(3.0, 5.0, 7.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        let face = remus_topology::explorer::solid_faces(kernel.topo(), id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        (kernel, source, super::index_u32(face.index()))
    }

    #[test]
    fn replacement_vectors_reject_length_before_reading_elements() {
        let value = serde_json::json!({"normal":[0,0,1,"invalid"]});
        let error = super::replacement_vector(&value, "normal").unwrap_err();
        assert!(error.message().contains("exactly 3 components"));
        for key in ["normal", "origin", "axis"] {
            let value = serde_json::json!({key:vec![0.0; 10_001]});
            assert!(
                super::replacement_vector(&value, key)
                    .unwrap_err()
                    .message()
                    .contains("exactly 3 components")
            );
        }
    }

    #[test]
    fn replacement_json_is_bounded_before_deserialization() {
        let limit = super::MAX_REPLACEMENT_JSON_BYTES;
        // Invalid syntax past the limit must be refused before JSON parsing.
        for text in ["[".repeat(limit + 1), "é".repeat(limit / 2 + 1)] {
            let error = super::parse_replacement_json(&text).unwrap_err();
            let error = serde_json::to_value(error).unwrap();
            assert_eq!(error["code"], "resource_limit_exceeded");
            assert_eq!(error["details"]["actual"], text.len());
        }
        assert!(
            super::parse_replacement_json("[")
                .unwrap_err()
                .message()
                .contains("JSON")
        );
        let mut replacement =
            serde_json::json!({"type":"plane","normal":[0,0,1],"d":8}).to_string();
        replacement.push_str(&" ".repeat(limit - replacement.len()));
        let value = super::parse_replacement_json(&replacement).unwrap();
        let (mut kernel, source, face) = replacement_box();
        assert!(
            kernel
                .replace_surface_journaled_json(source, face, &value)
                .is_ok()
        );
    }

    #[test]
    fn defeature_journaled_preserves_the_face_count_budget() {
        let (mut kernel, source, face) = replacement_box();
        let before = kernel.topo().journal().snapshot();
        let counts = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_faces(),
        );
        let faces = vec![face; crate::error::MAX_WASM_WORK_ITEMS as usize + 1];
        let error = kernel.defeature_journaled_json(source, &faces).unwrap_err();
        assert!(
            error.message().contains("faces must be at most"),
            "{}",
            error.message()
        );
        for op in ["defeatureJournaled", "defeature"] {
            let response: serde_json::Value = serde_json::from_str(
                &kernel.execute_batch_v2(
                    &serde_json::json!([
                        {"op":op,"args":{"solid":source,"faces":faces}}
                    ])
                    .to_string(),
                ),
            )
            .unwrap();
            assert_eq!(response[0]["error"]["message"], error.message());
            // The raw JSON count is checked before handle-array collection.
            let mut invalid = serde_json::json!(faces);
            invalid[0] = serde_json::json!("invalid");
            let response: serde_json::Value = serde_json::from_str(
                &kernel.execute_batch_v2(
                    &serde_json::json!([
                        {"op":op,"args":{"solid":source,"faces":invalid}}
                    ])
                    .to_string(),
                ),
            )
            .unwrap();
            assert_eq!(response[0]["error"]["message"], error.message());
        }
        assert_eq!(kernel.topo().journal().snapshot(), before);
        assert_eq!(
            (
                kernel.topo().num_vertices(),
                kernel.topo().num_edges(),
                kernel.topo().num_faces()
            ),
            counts
        );
    }

    #[test]
    fn replacement_history_has_direct_batch_reference_parity() {
        use remus_topology::journal::{EntityKind, OpId};
        use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
        let mut results = Vec::new();
        for batch in [false, true] {
            let (mut kernel, source, face) = replacement_box();
            let first = kernel
                .replace_surface_journaled_json(
                    source,
                    face,
                    &serde_json::json!({"type":"plane", "normal":[0,0,1], "d":8}),
                )
                .unwrap();
            let anchor = OpId::from_value(first["op"].as_u64().unwrap());
            let solid = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
            let solid_id = kernel.resolve_solid(solid).unwrap();
            let face = remus_topology::explorer::solid_faces(kernel.topo(), solid_id)
                .unwrap()
                .into_iter()
                .find(|&face| {
                    kernel
                        .topo()
                        .face(face)
                        .unwrap()
                        .effective_plane_normal()
                        .is_some_and(|normal| normal.z() > 0.9)
                })
                .unwrap();
            let face = super::index_u32(face.index());
            let replacement = serde_json::json!({"type":"plane", "normal":[-0.1,0,1], "d":7.85});
            let second = if batch {
                run(&mut kernel, serde_json::json!([
                    {"op":"replaceSurfaceJournaled", "args":{"solid":solid,"face":face,"replacement":replacement}}
                ])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .replace_surface_journaled_js(solid, face, &replacement.to_string())
                        .unwrap(),
                )
                .unwrap()
            };
            let solid = u32::try_from(second["solid"].as_u64().unwrap()).unwrap();
            assert!((kernel.volume(solid, 0.01).unwrap() - 120.0).abs() < 1e-8);
            let solid_id = kernel.resolve_solid(solid).unwrap();
            let live: std::collections::BTreeSet<_> =
                remus_operations::journal_ops::solid_entity_keys(kernel.topo(), solid_id)
                    .unwrap()
                    .into_iter()
                    .collect();
            let mut resolved = std::collections::BTreeSet::new();
            for (kind, count) in [
                (EntityKind::Face, 6),
                (EntityKind::Edge, 12),
                (EntityKind::Vertex, 8),
            ] {
                for index in 0..count {
                    let resolution = resolve(
                        kernel.topo(),
                        &PersistentRef::operation_output(anchor, kind, index),
                    );
                    let Resolution::Bound { entity, provenance } = resolution else {
                        panic!("{resolution:?}")
                    };
                    assert_eq!(provenance, Provenance::Construction);
                    assert!(resolved.insert(entity));
                }
            }
            assert_eq!(resolved, live);
            results.push(second);
        }
        assert_eq!(results[0], results[1]);
    }

    #[test]
    fn replacement_refusals_have_matching_errors_and_restore_history() {
        let (mut kernel, source, face) = replacement_box();
        let first = kernel
            .replace_surface_journaled_json(
                source,
                face,
                &serde_json::json!({"type":"plane", "normal":[0,0,1], "d":8}),
            )
            .unwrap();
        let source = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
        let source_id = kernel.resolve_solid(source).unwrap();
        let face = remus_topology::explorer::solid_faces(kernel.topo(), source_id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        let face = super::index_u32(face.index());
        let before = kernel.topo().journal().snapshot();
        let counts = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_faces(),
            kernel.topo().num_pcurves(),
        );
        for replacement in [
            serde_json::json!(null),
            serde_json::json!({}),
            serde_json::json!({"type":"sphere"}),
            serde_json::json!({"type":"plane", "normal":[0,0], "d":9}),
            serde_json::json!({"type":"plane", "normal":[0,0,0], "d":9}),
            serde_json::json!({"type":"plane", "normal":[0,0,1], "d":-1}),
            serde_json::json!({"type":"plane", "normal":[0,0,1], "d":9, "offset":1}),
            serde_json::json!({"type":"cylinder", "origin":[0,0,0], "axis":[0,0,1], "radius":1}),
        ] {
            let direct = serde_json::to_value(
                kernel
                    .replace_surface_journaled_json(source, face, &replacement)
                    .unwrap_err(),
            )
            .unwrap();
            let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
                {"op":"replaceSurfaceJournaled", "args":{"solid":source,"face":face,"replacement":replacement}}
            ]).to_string())).unwrap();
            let mut batch = response[0]["error"].clone();
            let details = batch["details"].as_object_mut().unwrap();
            assert_eq!(
                details.remove("operation"),
                Some(serde_json::json!("replaceSurfaceJournaled"))
            );
            assert_eq!(details.remove("operationIndex"), Some(serde_json::json!(0)));
            assert_eq!(batch, direct);
            assert_eq!(kernel.topo().journal().snapshot(), before);
            assert_eq!(
                (
                    kernel.topo().num_vertices(),
                    kernel.topo().num_edges(),
                    kernel.topo().num_faces(),
                    kernel.topo().num_pcurves()
                ),
                counts
            );
        }
    }

    #[test]
    fn replacement_preserves_the_topology_work_budget() {
        let mut kernel = BrepKernel::new();
        let wire = kernel.make_regular_polygon_wire(10.0, 500).unwrap();
        let profile = kernel.make_face_from_wire(wire).unwrap();
        let source = kernel.extrude_face(profile, 0.0, 0.0, 1.0, 1.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        assert!(
            remus_operations::validate::validate_solid(kernel.topo(), id)
                .unwrap()
                .is_valid()
        );
        let face = remus_topology::explorer::solid_faces(kernel.topo(), id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        let face = super::index_u32(face.index());
        let before = kernel.topo().journal().snapshot();
        let replacement = serde_json::json!({"type":"plane","normal":[0,0,1],"d":1.25});
        let error = kernel
            .replace_surface_journaled_json(source, face, &replacement)
            .unwrap_err();
        assert!(
            error
                .message()
                .contains("moveFaces topology work must be at most"),
            "{}",
            error.message()
        );
        let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
            {"op":"replaceSurfaceJournaled","args":{"solid":source,"face":face,"replacement":replacement}}
        ]).to_string())).unwrap();
        assert_eq!(response[0]["error"]["message"], error.message());
        assert_eq!(kernel.topo().journal().snapshot(), before);
    }

    #[test]
    fn replacement_cylinder_parser_preserves_the_source_parameter_direction() {
        use remus_math::surfaces::CylindricalSurface;
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::face::FaceSurface;
        let source = CylindricalSurface::with_ref_dir(
            Point3::new(3.0, 4.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let replacement = super::parse_replacement_surface(
            &serde_json::json!({
                "type":"cylinder", "origin":[3,4,5], "axis":[0,0,1], "radius":2
            }),
            &FaceSurface::Cylinder(source.clone()),
        )
        .unwrap();
        let FaceSurface::Cylinder(replacement) = replacement else {
            panic!("cylinder required")
        };
        assert!((replacement.x_axis() - source.x_axis()).length() < 1e-12);
        assert!((replacement.radius() - 2.0).abs() < 1e-12);
    }

    #[test]
    fn defeature_history_has_direct_batch_parity_and_rollback() {
        let mut payloads = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let topo = kernel.topo_mut();
            let cube = remus_operations::primitives::make_box(topo, 10.0, 10.0, 10.0).unwrap();
            let tool = remus_operations::primitives::make_cylinder(topo, 1.0, 10.0).unwrap();
            remus_operations::transform::transform_solid(
                topo,
                tool,
                &remus_math::mat::Mat4::translation(5.0, 5.0, 0.0),
            )
            .unwrap();
            let source = remus_operations::boolean::boolean(
                topo,
                remus_operations::boolean::BooleanOp::Cut,
                cube,
                tool,
            )
            .unwrap();
            let faces: Vec<_> = remus_topology::explorer::solid_faces(topo, source)
                .unwrap()
                .into_iter()
                .filter(|&face| {
                    matches!(
                        topo.face(face).unwrap().surface(),
                        remus_topology::face::FaceSurface::Cylinder(_)
                    )
                })
                .map(|face| super::index_u32(face.index()))
                .collect();
            assert_eq!(faces.len(), 1);
            let solid = crate::handles::solid_id_to_u32(source);
            let result: serde_json::Value = if batch {
                run(&mut kernel, serde_json::json!([{"op":"defeatureJournaled", "args":{"solid":solid,"faces":faces}}])).remove(0)
            } else {
                serde_json::from_str(&kernel.defeature_journaled_js(solid, &faces).unwrap())
                    .unwrap()
            };
            let handle = u32::try_from(result["solid"].as_u64().unwrap()).unwrap();
            let result_id = kernel.resolve_solid(handle).unwrap();
            let volume =
                remus_operations::measure::solid_volume(kernel.topo(), result_id, 0.01).unwrap();
            assert!((volume - 1000.0).abs() < 1e-6);
            let before = kernel.topo().journal().snapshot();
            assert!(kernel.defeature_journaled_json(handle, &[]).is_err());
            assert!(kernel.defeature_journaled_json(handle, &faces).is_err());
            assert_eq!(kernel.topo().journal().snapshot(), before);
            payloads.push(result);
        }
        assert_eq!(payloads[0], payloads[1]);
    }

    #[test]
    fn draft_history_has_degree_units_and_direct_batch_parity() {
        use remus_operations::journal_ops;
        use remus_topology::{
            journal::{EntityKind, OpId},
            naming::{PersistentRef, Provenance, Resolution, resolve},
        };
        let mut payloads = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
            let wall = |kernel: &BrepKernel, solid| {
                let id = kernel.resolve_solid(solid).unwrap();
                super::index_u32(
                    remus_topology::explorer::solid_faces(kernel.topo(), id)
                        .unwrap()
                        .into_iter()
                        .find(|&face| {
                            kernel
                                .topo()
                                .face(face)
                                .unwrap()
                                .effective_plane_normal()
                                .is_some_and(|normal| normal.x() > 0.9)
                        })
                        .unwrap()
                        .index(),
                )
            };
            let face = wall(&kernel, source);
            let first = kernel
                .draft_journaled_json(source, &[face], &[0.0, 0.0, 1.0], &[0.0; 3], 5.0)
                .unwrap();
            let solid = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
            let first_id = kernel.resolve_solid(solid).unwrap();
            let volume =
                remus_operations::measure::solid_volume(kernel.topo(), first_id, 0.01).unwrap();
            let expected = 1000.0 + 500.0 * 5.0_f64.to_radians().tan();
            assert!((volume - expected).abs() < 1e-6);
            let keys = journal_ops::solid_entity_keys(kernel.topo(), first_id).unwrap();
            let op = OpId::from_value(first["op"].as_u64().unwrap());
            let face = wall(&kernel, solid);
            let second: serde_json::Value = if batch {
                run(&mut kernel, serde_json::json!([{"op":"draftJournaled", "args":{"solid":solid,"faces":[face],"pullDirection":[0,0,1],"neutralPoint":[0,0,0],"angleDegrees":-2.0}}])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .draft_journaled_js(solid, &[face], &[0.0, 0.0, 1.0], &[0.0; 3], -2.0)
                        .unwrap(),
                )
                .unwrap()
            };
            let result = u32::try_from(second["solid"].as_u64().unwrap()).unwrap();
            let live: std::collections::BTreeSet<_> = journal_ops::solid_entity_keys(
                kernel.topo(),
                kernel.resolve_solid(result).unwrap(),
            )
            .unwrap()
            .into_iter()
            .collect();
            let mut resolved = std::collections::BTreeSet::new();
            for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                    let reference = PersistentRef::operation_output(op, kind, index);
                    let outcome = resolve(kernel.topo(), &reference);
                    let Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } = outcome
                    else {
                        panic!("lost draft reference: {outcome:?}");
                    };
                    resolved.insert(entity);
                }
            }
            assert_eq!(resolved, live);
            let face = wall(&kernel, result);
            let before = kernel.topo().journal().snapshot();
            for (pull, neutral, angle) in [
                (vec![0.0, 0.0], vec![0.0; 3], 5.0),
                (vec![0.0, 0.0, 1.0], vec![0.0; 2], 5.0),
                (vec![0.0, 0.0, 1.0], vec![0.0; 3], f64::NAN),
                (vec![0.0, 0.0, 1.0], vec![0.0; 3], -80.0),
            ] {
                assert!(
                    kernel
                        .draft_journaled_json(result, &[face], &pull, &neutral, angle)
                        .is_err()
                );
                assert_eq!(kernel.topo().journal().snapshot(), before);
            }
            payloads.push(second);
        }
        assert_eq!(payloads[0], payloads[1]);
    }

    #[test]
    fn remove_blends_has_direct_batch_parity_total_history_and_rollback() {
        use remus_operations::{
            blend_ops::fillet_v2, journal_ops::solid_entity_keys, primitives::make_box,
        };
        use remus_topology::{
            explorer::{solid_edges, solid_faces},
            face::FaceSurface,
            journal::{EntityKind, EventDraft, EvolutionDraft},
            naming::{PersistentRef, Provenance, Resolution, resolve},
        };
        let mut payloads = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let sharp = make_box(kernel.topo_mut(), 10.0, 10.0, 10.0).unwrap();
            let edge = solid_edges(kernel.topo(), sharp).unwrap()[0];
            let solid = fillet_v2(kernel.topo_mut(), sharp, &[edge], 1.0)
                .unwrap()
                .solid;
            let keys = solid_entity_keys(kernel.topo(), solid).unwrap();
            let pending = kernel.topo_mut().journal_begin("group_fixture");
            let mut draft = EvolutionDraft::construction();
            draft.add_scope(keys.iter().copied());
            for &key in &keys {
                draft.push(key, EventDraft::Generated { sources: vec![] });
            }
            let anchor = kernel
                .topo_mut()
                .journal_record_evolution(pending, draft)
                .unwrap();
            let face = solid_faces(kernel.topo(), solid)
                .unwrap()
                .into_iter()
                .find(|&face| {
                    matches!(
                        kernel.topo().face(face).unwrap().surface(),
                        FaceSurface::Cylinder(_)
                    )
                })
                .unwrap();
            let source = super::index_u32(solid.index());
            let seed = super::index_u32(face.index());
            let before = kernel.topo().journal().snapshot();
            let slots = kernel.topo().allocated_slot_count();
            assert!(kernel.remove_blends_journaled_json(source, &[]).is_err());
            assert!(
                kernel
                    .remove_blends_journaled_json(source, &[u32::MAX])
                    .is_err()
            );
            let failed: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(
                &serde_json::json!([{"op":"removeBlendsJournaled", "args":{"solid":source,"seeds":[]}}]).to_string()
            )).unwrap();
            assert!(failed[0]["error"].is_object());
            assert_eq!(kernel.topo().journal().snapshot(), before);
            assert_eq!(kernel.topo().allocated_slot_count(), slots);
            let output: serde_json::Value = if batch {
                run(&mut kernel, serde_json::json!([{"op":"removeBlendsJournaled", "args":{"solid":source,"seeds":[seed,seed]}}])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .remove_blends_journaled_js(source, &[seed, seed])
                        .unwrap(),
                )
                .unwrap()
            };
            let result = kernel
                .resolve_solid(u32::try_from(output["solid"].as_u64().unwrap()).unwrap())
                .unwrap();
            assert!(
                (remus_operations::measure::solid_volume(kernel.topo(), result, 0.01).unwrap()
                    - 1000.0)
                    .abs()
                    < 1e-6
            );
            let live: std::collections::BTreeSet<_> = solid_entity_keys(kernel.topo(), result)
                .unwrap()
                .into_iter()
                .collect();
            let mut found = std::collections::BTreeSet::new();
            let mut deleted_faces = 0;
            for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                    match resolve(
                        kernel.topo(),
                        &PersistentRef::operation_output(anchor, kind, index),
                    ) {
                        Resolution::Bound {
                            entity,
                            provenance: Provenance::Construction,
                        } => {
                            found.insert(entity);
                        }
                        Resolution::Dangling { .. } => {
                            if kind == EntityKind::Face {
                                deleted_faces += 1;
                            }
                        }
                        other => panic!("unresolved group history: {other:?}"),
                    }
                }
            }
            assert_eq!(found, live);
            assert_eq!(deleted_faces, 1);
            payloads.push(output);
        }
        assert_eq!(payloads[0], payloads[1]);
    }

    #[test]
    fn blend_resize_history_has_direct_batch_parity_and_rollback() {
        use remus_operations::{
            blend_ops::fillet_v2, journal_ops::solid_entity_keys, primitives::make_box,
        };
        use remus_topology::{
            explorer::{solid_edges, solid_faces},
            face::FaceSurface,
            journal::{EntityKind, EventDraft, EvolutionDraft},
            naming::{PersistentRef, Provenance, Resolution, resolve},
        };
        let mut payloads = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let sharp = make_box(kernel.topo_mut(), 10.0, 10.0, 10.0).unwrap();
            let edge = solid_edges(kernel.topo(), sharp).unwrap()[0];
            let mut solid = fillet_v2(kernel.topo_mut(), sharp, &[edge], 1.0)
                .unwrap()
                .solid;
            let keys = solid_entity_keys(kernel.topo(), solid).unwrap();
            let pending = kernel.topo_mut().journal_begin("blend_fixture");
            let mut draft = EvolutionDraft::construction();
            draft.add_scope(keys.iter().copied());
            for &key in &keys {
                draft.push(key, EventDraft::Generated { sources: vec![] });
            }
            let anchor = kernel
                .topo_mut()
                .journal_record_evolution(pending, draft)
                .unwrap();
            let mut radius = 1.0;
            let mut outputs = Vec::new();
            for next in [2.0, 0.75] {
                let face = solid_faces(kernel.topo(), solid)
                    .unwrap()
                    .into_iter()
                    .find(|&face| {
                        matches!(
                            kernel.topo().face(face).unwrap().surface(),
                            FaceSurface::Cylinder(_)
                        )
                    })
                    .unwrap();
                let source = super::index_u32(solid.index());
                let face = super::index_u32(face.index());
                let output: serde_json::Value = if batch {
                    run(&mut kernel, serde_json::json!([{"op":"resizeBlendJournaled","args":{"solid":source,"face":face,"expectedRadius":radius,"newRadius":next}}])).remove(0)
                } else {
                    serde_json::from_str(
                        &kernel
                            .resize_blend_journaled_js(source, face, radius, next)
                            .unwrap(),
                    )
                    .unwrap()
                };
                solid = kernel
                    .resolve_solid(u32::try_from(output["solid"].as_u64().unwrap()).unwrap())
                    .unwrap();
                let mut found = std::collections::BTreeSet::new();
                for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                    for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                        let Resolution::Bound {
                            entity,
                            provenance: Provenance::Construction,
                        } = resolve(
                            kernel.topo(),
                            &PersistentRef::operation_output(anchor, kind, index),
                        )
                        else {
                            panic!("lost blend reference");
                        };
                        assert!(found.insert(entity));
                    }
                }
                assert_eq!(
                    found,
                    solid_entity_keys(kernel.topo(), solid)
                        .unwrap()
                        .into_iter()
                        .collect()
                );
                radius = next;
                outputs.push(output);
            }
            let face = solid_faces(kernel.topo(), solid)
                .unwrap()
                .into_iter()
                .find(|&face| {
                    matches!(
                        kernel.topo().face(face).unwrap().surface(),
                        FaceSurface::Cylinder(_)
                    )
                })
                .unwrap();
            let source = super::index_u32(solid.index());
            let face = super::index_u32(face.index());
            let before = kernel.topo().journal().snapshot();
            let error = kernel
                .resize_blend_journaled_json(source, face, radius, 50.0)
                .unwrap_err();
            let failed: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([{"op":"resizeBlendJournaled","args":{"solid":source,"face":face,"expectedRadius":radius,"newRadius":50.0}}]).to_string())).unwrap();
            assert_eq!(failed[0]["error"]["message"], error.message());
            let after = kernel.topo().journal().snapshot();
            assert_eq!(after.entries, before.entries);
            assert_eq!(after.index, before.index);
            outputs.push(serde_json::json!(error.message()));
            let removed: serde_json::Value = if batch {
                run(&mut kernel, serde_json::json!([{"op":"resizeBlendJournaled","args":{"solid":source,"face":face,"expectedRadius":radius,"newRadius":0.0}}])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .resize_blend_journaled_js(source, face, radius, 0.0)
                        .unwrap(),
                )
                .unwrap()
            };
            let result_id = kernel
                .resolve_solid(u32::try_from(removed["solid"].as_u64().unwrap()).unwrap())
                .unwrap();
            let removal_op =
                remus_topology::journal::OpId::from_value(removed["op"].as_u64().unwrap());
            let mut found = std::collections::BTreeSet::new();
            let mut deleted = 0;
            for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                    match resolve(
                        kernel.topo(),
                        &PersistentRef::operation_output(anchor, kind, index),
                    ) {
                        Resolution::Bound {
                            entity,
                            provenance: Provenance::Construction,
                        } => {
                            found.insert(entity);
                        }
                        Resolution::Dangling { deleted_at } => {
                            assert_eq!(deleted_at, removal_op);
                            deleted += 1;
                        }
                        other => panic!("lost removal reference: {other:?}"),
                    }
                }
            }
            assert_eq!(deleted, 3);
            assert_eq!(
                found,
                solid_entity_keys(kernel.topo(), result_id)
                    .unwrap()
                    .into_iter()
                    .collect()
            );
            assert!(
                (remus_operations::measure::solid_volume(kernel.topo(), result_id, 0.01).unwrap()
                    - 1000.0)
                    .abs()
                    < 1e-6
            );
            outputs.push(removed);

            payloads.push(outputs);
        }
        assert_eq!(payloads[0], payloads[1]);
    }

    #[test]
    fn cylindrical_radius_history_has_direct_batch_parity_and_rollback() {
        use remus_math::{mat::Mat4, vec::Vec3};
        use remus_operations::{
            boolean::{BooleanOp, boolean},
            primitives::{make_box, make_cylinder},
            transform::transform_solid,
        };
        use remus_topology::{
            explorer::solid_faces,
            face::FaceSurface,
            journal::{EntityKind, OpId},
            naming::{PersistentRef, Provenance, Resolution, resolve},
        };
        let mut payloads = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let block = make_box(kernel.topo_mut(), 40.0, 40.0, 10.0).unwrap();
            let drill = make_cylinder(kernel.topo_mut(), 3.0, 10.0).unwrap();
            transform_solid(
                kernel.topo_mut(),
                drill,
                &Mat4::translation(20.0, 20.0, 0.0),
            )
            .unwrap();
            let source_id = boolean(kernel.topo_mut(), BooleanOp::Cut, block, drill).unwrap();
            let wall = |kernel: &BrepKernel, solid| {
                solid_faces(kernel.topo(), solid)
                    .unwrap()
                    .into_iter()
                    .find(|&face| {
                        matches!(
                            kernel.topo().face(face).unwrap().surface(),
                            FaceSurface::Cylinder(_)
                        )
                    })
                    .unwrap()
            };
            let source = super::index_u32(source_id.index());
            let face = super::index_u32(wall(&kernel, source_id).index());
            let first = kernel
                .resize_cylindrical_face_journaled_json(source, face, 5.0)
                .unwrap();
            let solid = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
            let first_id = kernel.resolve_solid(solid).unwrap();
            let keys =
                remus_operations::journal_ops::solid_entity_keys(kernel.topo(), first_id).unwrap();
            let anchor = OpId::from_value(first["op"].as_u64().unwrap());
            let face = super::index_u32(wall(&kernel, first_id).index());
            let second: serde_json::Value = if batch {
                run(&mut kernel, serde_json::json!([{"op":"resizeCylindricalFaceJournaled", "args":{"solid":solid,"face":face,"radius":2.0}}])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .resize_cylindrical_face_journaled_js(solid, face, 2.0)
                        .unwrap(),
                )
                .unwrap()
            };
            let result = u32::try_from(second["solid"].as_u64().unwrap()).unwrap();
            let result_id = kernel.resolve_solid(result).unwrap();
            let live: std::collections::HashSet<_> =
                remus_operations::journal_ops::solid_entity_keys(kernel.topo(), result_id)
                    .unwrap()
                    .into_iter()
                    .collect();
            let mut resolved = std::collections::HashSet::new();
            for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                    let reference = PersistentRef::operation_output(anchor, kind, index);
                    let Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } = resolve(kernel.topo(), &reference)
                    else {
                        panic!("lost radius reference");
                    };
                    resolved.insert(entity);
                }
            }
            assert_eq!(resolved, live);
            let cylinder = wall(&kernel, result_id);
            let FaceSurface::Cylinder(cylinder_data) =
                kernel.topo().face(cylinder).unwrap().surface()
            else {
                unreachable!()
            };
            assert!((cylinder_data.radius() - 2.0).abs() < 1e-10);
            assert!((cylinder_data.axis() - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-10);
            let before = kernel.topo().journal().snapshot();
            let face = super::index_u32(cylinder.index());
            let error = kernel
                .resize_cylindrical_face_journaled_json(result, face, 25.0)
                .unwrap_err();
            let failed: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([{"op":"resizeCylindricalFaceJournaled", "args":{"solid":result,"face":face,"radius":25.0}}]).to_string())).unwrap();
            assert_eq!(failed[0]["error"]["message"], error.message());
            assert_eq!(kernel.topo().journal().snapshot(), before);
            payloads.push(second);
        }
        assert_eq!(payloads[0], payloads[1]);
    }

    #[test]
    fn move_faces_journaled_preserves_the_face_count_budget() {
        let mut kernel = BrepKernel::new();
        let source = kernel.make_box_solid(3.0, 5.0, 7.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        let face = super::index_u32(
            remus_topology::explorer::solid_faces(kernel.topo(), id).unwrap()[0].index(),
        );
        let faces = vec![face; crate::error::MAX_WASM_WORK_ITEMS as usize + 1];
        let before = kernel.topo().journal().snapshot();
        let error = kernel
            .move_faces_journaled_json(source, &faces, 0.25)
            .unwrap_err();
        assert!(
            error.message().contains("faces must be at most"),
            "{}",
            error.message()
        );
        let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
            {"op":"moveFacesJournaled", "args":{"solid":source,"faces":faces,"distance":0.25}}
        ]).to_string())).unwrap();
        assert_eq!(response[0]["error"]["message"], error.message());
        assert_eq!(kernel.topo().journal().snapshot(), before);
    }

    #[test]
    fn move_faces_journaled_preserves_the_topology_work_budget() {
        let mut kernel = BrepKernel::new();
        let wire = kernel.make_regular_polygon_wire(10.0, 500).unwrap();
        let face = kernel.make_face_from_wire(wire).unwrap();
        let source = kernel.extrude_face(face, 0.0, 0.0, 1.0, 1.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        assert!(
            remus_operations::validate::validate_solid(kernel.topo(), id)
                .unwrap()
                .is_valid()
        );
        let face = remus_topology::explorer::solid_faces(kernel.topo(), id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        let face = super::index_u32(face.index());
        let before = kernel.topo().journal().snapshot();
        let error = kernel
            .move_faces_journaled_json(source, &[face], 0.25)
            .unwrap_err();
        assert!(
            error
                .message()
                .contains("moveFaces topology work must be at most"),
            "{}",
            error.message()
        );
        let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
            {"op":"moveFacesJournaled", "args":{"solid":source,"faces":[face],"distance":0.25}}
        ]).to_string())).unwrap();
        assert_eq!(response[0]["error"]["message"], error.message());
        assert_eq!(kernel.topo().journal().snapshot(), before);
    }

    #[test]
    fn move_faces_journaled_preserves_all_entity_refs_in_direct_and_batch_calls() {
        use super::index_u32;
        use remus_operations::journal_ops;
        use remus_topology::journal::{EntityKind, OpId};
        use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

        let mut payloads = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let source = kernel.make_box_solid(3.0, 5.0, 7.0).unwrap();
            let source_id = kernel.resolve_solid(source).unwrap();
            let face = remus_topology::explorer::solid_faces(kernel.topo(), source_id).unwrap()[0];
            let face = index_u32(face.index());
            let first = kernel
                .move_faces_journaled_json(source, &[face], 0.25)
                .unwrap();
            let solid = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
            let anchor = OpId::from_value(first["op"].as_u64().unwrap());
            let first_id = kernel.resolve_solid(solid).unwrap();
            let face = remus_topology::explorer::solid_faces(kernel.topo(), first_id).unwrap()[0];
            let face = index_u32(face.index());
            let second = if batch {
                run(&mut kernel, serde_json::json!([
                    {"op": "moveFacesJournaled", "args": {"solid": solid, "faces": [face], "distance": -0.125}}
                ])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .move_faces_journaled_js(solid, &[face], -0.125)
                        .unwrap(),
                )
                .unwrap()
            };
            let result = u32::try_from(second["solid"].as_u64().unwrap()).unwrap();
            let result_id = kernel.resolve_solid(result).unwrap();
            let live: std::collections::BTreeSet<_> =
                journal_ops::solid_entity_keys(kernel.topo(), result_id)
                    .unwrap()
                    .into_iter()
                    .collect();
            let mut resolved = std::collections::BTreeSet::new();
            for (kind, count) in [
                (EntityKind::Face, 6),
                (EntityKind::Edge, 12),
                (EntityKind::Vertex, 8),
            ] {
                for index in 0..count {
                    let reference = PersistentRef::operation_output(anchor, kind, index);
                    let resolution = resolve(kernel.topo(), &reference);
                    let Resolution::Bound { entity, provenance } = resolution else {
                        panic!("lost direct-edit reference: {resolution:?}");
                    };
                    assert_eq!(provenance, Provenance::Construction);
                    assert!(resolved.insert(entity));
                }
            }
            assert_eq!(resolved, live);
            payloads.push(second);
        }
        assert_eq!(payloads[0], payloads[1]);
    }

    #[test]
    fn offset_journaled_has_direct_and_batch_contract_parity() {
        let mut direct = BrepKernel::new();
        let source = direct.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let payload: serde_json::Value =
            serde_json::from_str(&direct.offset_journaled_js(source, 0.5).unwrap()).unwrap();
        let result = u32::try_from(payload["solid"].as_u64().unwrap()).unwrap();
        assert!((direct.volume(result, 0.1).unwrap() - 27.0).abs() < 1e-9);
        let summary: serde_json::Value = serde_json::from_str(&direct.journal_summary()).unwrap();
        let entry = summary.as_array().unwrap().last().unwrap();
        assert_eq!(entry["kind"], "offset");
        assert_eq!(entry["type"], "evolution");
        assert_eq!(entry["detail"]["origin"], "construction");
        assert_eq!(entry["detail"]["events"], 6 + 12 + 8);
        assert_eq!(payload["evolution"]["completeness"]["resolved"], true);

        let mut batch = BrepKernel::new();
        run(
            &mut batch,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 2.0}},
            ]),
        );
        let results = run(
            &mut batch,
            serde_json::json!([
                {"op": "offsetJournaled", "args": {"solid": 0, "distance": 0.5}},
                {"op": "journalSummary", "args": {}},
            ]),
        );
        assert_eq!(results[0], payload);
        let batch_entry = results[1].as_array().unwrap().last().unwrap();
        assert_eq!(batch_entry["kind"], "offset");
        assert_eq!(batch_entry["type"], "evolution");
        assert_eq!(batch_entry["detail"]["events"], 6 + 12 + 8);
    }

    /// B18: one journaled extrusion carries total face/edge/vertex history
    /// with direct/batch parity. The expected roles below are built from
    /// the fixture (a rectangle profile) and live-topology queries — not
    /// from the implementation's own maps: both caps modified from the
    /// profile, every side generated from it, shared boundary entities
    /// modified into themselves, translated copies modified from their
    /// source, longitudinal edges generated from the profile.
    #[test]
    fn extrude_journaled_has_direct_batch_parity_census_and_rollback() {
        use remus_operations::journal_ops;

        let mut direct = BrepKernel::new();
        let face = direct.make_rectangle(4.0, 2.0).unwrap();
        let payload: serde_json::Value = serde_json::from_str(
            &direct
                .extrude_journaled_js(face, 0.0, 0.0, 1.0, 3.0)
                .unwrap(),
        )
        .unwrap();
        let result = u32::try_from(payload["solid"].as_u64().unwrap()).unwrap();
        assert!((direct.volume(result, 0.01).unwrap() - 24.0).abs() < 1e-9);
        let evolution = &payload["evolution"];
        assert_eq!(evolution["origin"], "construction");
        let completeness = &evolution["completeness"];
        assert_eq!(completeness["accounted"], true);
        assert_eq!(completeness["resolved"], true);
        for kind in ["faces", "edges", "vertices"] {
            assert_eq!(completeness[kind]["omitted"], serde_json::json!([]));
            assert_eq!(completeness[kind]["phantom"], serde_json::json!([]));
            assert_eq!(completeness[kind]["unresolved"], serde_json::json!([]));
        }
        // Face roles: two modified caps, four generated sides.
        let faces = evolution["faces"].as_array().unwrap();
        assert_eq!(faces.len(), 6);
        let (modified, generated): (Vec<_>, Vec<_>) =
            faces.iter().partition(|face| face["event"] == "modified");
        assert_eq!(modified.len(), 2);
        assert_eq!(generated.len(), 4);
        for face in &modified {
            assert_eq!(face["from"], 0);
        }
        for face in &generated {
            assert_eq!(face["faces"], serde_json::json!([0]));
        }
        // Edge roles against the live topology: shared profile edges
        // modified into themselves, translated copies modified from a
        // profile edge, longitudinal edges generated from the profile.
        let profile_edges: std::collections::BTreeSet<u64> =
            journal_ops::profile_entity_keys(direct.topo(), direct.resolve_face(face).unwrap())
                .unwrap()
                .into_iter()
                .filter(|key| key.kind == remus_topology::journal::EntityKind::Edge)
                .map(|key| u64::try_from(key.index).unwrap())
                .collect();
        let result_edges: Vec<u64> = direct
            .get_solid_edges(result)
            .unwrap()
            .into_iter()
            .map(u64::from)
            .collect();
        assert_eq!(result_edges.len(), 12);
        let edges = evolution["edges"].as_array().unwrap();
        assert_eq!(edges.len(), 12);
        let mut kinds = std::collections::BTreeMap::new();
        for edge in edges {
            let id = edge["edge"].as_u64().unwrap();
            assert!(
                result_edges.contains(&id),
                "claimed edge {id} must be live on the result"
            );
            *kinds
                .entry(edge["event"].as_str().unwrap().to_string())
                .or_default() += 1;
            match edge["event"].as_str().unwrap() {
                "modified" => {
                    let from = edge["from"].as_u64().unwrap();
                    assert!(
                        profile_edges.contains(&from),
                        "translated edge {id} must name a profile source"
                    );
                    if id != from {
                        assert!(
                            !profile_edges.contains(&id),
                            "translated copy {id} must be a new entity"
                        );
                    }
                }
                "generated" => {
                    assert_eq!(edge["faces"], serde_json::json!([0]));
                }
                other => panic!("unexpected edge event {other}"),
            }
        }
        assert_eq!(kinds.get("modified"), Some(&8));
        assert_eq!(kinds.get("generated"), Some(&4));
        // Vertex roles: shared corners modified into themselves, copies
        // modified from their source.
        let vertices = evolution["vertices"].as_array().unwrap();
        assert_eq!(vertices.len(), 8);
        for vertex in vertices {
            assert_eq!(vertex["event"], "modified");
        }
        let summary: serde_json::Value = serde_json::from_str(&direct.journal_summary()).unwrap();
        let entry = summary.as_array().unwrap().last().unwrap();
        assert_eq!(entry["kind"], "extrude");
        assert_eq!(entry["type"], "evolution");
        assert_eq!(entry["detail"]["origin"], "construction");
        assert_eq!(entry["detail"]["events"], 6 + 12 + 8);

        // The same payload through both batch contracts on twin kernels.
        for v2 in [false, true] {
            let mut batch = BrepKernel::new();
            let batch_face = batch.make_rectangle(4.0, 2.0).unwrap();
            assert_eq!(batch_face, face);
            let ops = serde_json::json!([
                {"op": "extrudeJournaled", "args": {"face": face, "dx": 0.0, "dy": 0.0, "dz": 1.0, "distance": 3.0}},
            ]);
            let response = if v2 {
                batch.execute_batch_v2(&ops.to_string())
            } else {
                batch.execute_batch(&ops.to_string())
            };
            let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
            assert_eq!(
                &parsed[0]["ok"], &payload,
                "extrudeJournaled batch parity (v2={v2})"
            );
            let batch_solid = u32::try_from(parsed[0]["ok"]["solid"].as_u64().unwrap()).unwrap();
            assert!((batch.volume(batch_solid, 0.01).unwrap() - 24.0).abs() < 1e-9);
        }

        // Refusals roll back and read identically on every envelope: zero
        // distance refuses with no journal entry and live counts unchanged.
        let before_summary = direct.journal_summary();
        let before_counts = (
            direct.topo().num_vertices(),
            direct.topo().num_edges(),
            direct.topo().num_faces(),
            direct.topo().num_solids(),
        );
        let direct_error = direct
            .extrude_journaled_json(face, [0.0, 0.0, 1.0], 0.0)
            .unwrap_err();
        assert!(
            direct_error.message().contains("zero"),
            "zero distance must refuse: {}",
            direct_error.message()
        );
        assert_eq!(direct.journal_summary(), before_summary);
        assert_eq!(
            (
                direct.topo().num_vertices(),
                direct.topo().num_edges(),
                direct.topo().num_faces(),
                direct.topo().num_solids(),
            ),
            before_counts
        );
        for v2 in [false, true] {
            let ops = serde_json::json!([
                {"op": "extrudeJournaled", "args": {"face": face, "dx": 0.0, "dy": 0.0, "dz": 1.0, "distance": 0.0}},
            ]);
            let response = if v2 {
                direct.execute_batch_v2(&ops.to_string())
            } else {
                direct.execute_batch(&ops.to_string())
            };
            let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
            assert!(
                parsed[0].get("error").is_some(),
                "refused batch extrusion must error (v2={v2}): {parsed}"
            );
            assert_eq!(direct.journal_summary(), before_summary);
        }
    }

    /// B18: a torus offset's two seams share one face-use multiset, so
    /// neither can be bound to a source seam. The typed unresolved record
    /// (`ambiguous_incidence`, both source seams as candidates) must reach
    /// JS unchanged through the direct binding, `executeBatch`, and
    /// `executeBatchV2`, beside an accounted-but-unresolved completeness
    /// report; the box's history is fully resolved with every result edge
    /// and vertex attributed.
    #[test]
    fn offset_journaled_typed_unresolved_survives_every_envelope() {
        let mut direct = BrepKernel::new();
        let torus = direct.make_torus_solid(3.0, 1.0, 16).unwrap();
        let source_edges: Vec<u64> = direct
            .get_solid_edges(torus)
            .unwrap()
            .into_iter()
            .map(u64::from)
            .collect();
        let payload: serde_json::Value =
            serde_json::from_str(&direct.offset_journaled_js(torus, 0.3).unwrap()).unwrap();
        let evolution = &payload["evolution"];
        assert_eq!(evolution["origin"], "construction");
        let completeness = &evolution["completeness"];
        assert_eq!(completeness["accounted"], true);
        assert_eq!(completeness["resolved"], false);
        let result_edges: Vec<u64> = direct
            .get_solid_edges(u32::try_from(payload["solid"].as_u64().unwrap()).unwrap())
            .unwrap()
            .into_iter()
            .map(u64::from)
            .collect();
        let unresolved: Vec<u64> = completeness["edges"]["unresolved"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_u64().unwrap())
            .collect();
        assert_eq!(unresolved, result_edges);
        for kind in ["faces", "edges", "vertices"] {
            assert_eq!(completeness[kind]["omitted"], serde_json::json!([]));
            assert_eq!(completeness[kind]["phantom"], serde_json::json!([]));
        }
        let edges = evolution["edges"].as_array().unwrap();
        assert_eq!(edges.len(), 2);
        for edge in edges {
            assert_eq!(edge["event"], "unresolved");
            assert_eq!(edge["reason"], "ambiguous_incidence");
            assert_eq!(edge["candidates"], serde_json::json!(source_edges));
        }
        assert_eq!(evolution["vertices"][0]["event"], "modified");
        assert_eq!(evolution["faces"][0]["event"], "modified");

        // The same record through both batch contracts.
        for v2 in [false, true] {
            let mut batch = BrepKernel::new();
            let ops = serde_json::json!([
                {"op": "makeTorus", "args": {"majorRadius": 3.0, "minorRadius": 1.0, "segments": 16}},
                {"op": "offsetJournaled", "args": {"solid": 0, "distance": 0.3}},
            ]);
            let response = if v2 {
                batch.execute_batch_v2(&ops.to_string())
            } else {
                batch.execute_batch(&ops.to_string())
            };
            let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
            assert_eq!(
                parsed[1]["ok"], payload,
                "v2={v2}: envelope must not drop the record"
            );
        }

        // A box: everything attributed, nothing unresolved.
        let mut kernel = BrepKernel::new();
        let cube = kernel.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let payload: serde_json::Value =
            serde_json::from_str(&kernel.offset_journaled_js(cube, 0.25).unwrap()).unwrap();
        let evolution = &payload["evolution"];
        assert_eq!(evolution["completeness"]["resolved"], true);
        assert_eq!(evolution["faces"].as_array().unwrap().len(), 6);
        assert_eq!(evolution["edges"].as_array().unwrap().len(), 12);
        assert_eq!(evolution["vertices"].as_array().unwrap().len(), 8);
        for kind in ["faces", "edges", "vertices"] {
            assert!(
                evolution[kind]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|event| event["event"] == "modified"),
                "{kind}"
            );
        }
    }

    /// A refused offset reaches `executeBatchV2` as a typed error and
    /// publishes no history, beside earlier results that stay usable.
    #[test]
    fn refused_offset_journaled_is_typed_and_atomic_in_batch_v2() {
        let mut kernel = BrepKernel::new();
        let response = kernel.execute_batch_v2(
            &serde_json::json!([
                {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 2.0}},
                {"op": "journalSummary", "args": {}},
                {"op": "offsetJournaled", "args": {"solid": 0, "distance": -1.5}},
                {"op": "journalSummary", "args": {}},
                {"op": "volume", "args": {"solid": 0, "deflection": 0.1}},
            ])
            .to_string(),
        );
        let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
        let error = &parsed[2]["error"];
        assert!(error["code"].is_string(), "{error}");
        assert!(
            error["message"].as_str().unwrap().contains("collapsed"),
            "{error}"
        );
        assert_eq!(
            parsed[1]["ok"], parsed[3]["ok"],
            "refusal published history"
        );
        assert!((parsed[4]["ok"].as_f64().unwrap() - 8.0).abs() < 1e-9);
    }

    #[test]
    fn chamfer_journaled_severs_edge_refs_like_any_faces_only_entry() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "transform", "args": {"solid": 1, "matrix":
                    [1.0,0.0,0.0,5.0, 0.0,1.0,0.0,5.0, 0.0,0.0,1.0,5.0, 0.0,0.0,0.0,1.0]}},
                {"op": "fuseJournaled", "args": {"solidA": 0, "solidB": 1}},
            ]),
        );
        let fuse_op = results[3]["op"].as_u64().unwrap();
        let fused = results[3]["solid"].as_u64().unwrap();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "resolveOperationOutput",
                 "args": {"op": fuse_op, "kind": "edge", "index": 0}},
                {"op": "solidEdges", "args": {"solid": fused}},
            ]),
        );
        assert_eq!(results[0]["status"], "bound");
        let edge = results[1].as_array().unwrap()[0].as_u64().unwrap();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "chamferJournaled",
                 "args": {"solid": fused, "edges": [edge], "d1": 0.5, "d2": 0.5}},
                {"op": "resolveOperationOutput",
                 "args": {"op": fuse_op, "kind": "edge", "index": 0}},
            ]),
        );
        assert_eq!(results[1]["status"], "unresolvedAcrossOperation");
        assert_eq!(results[1]["operationKind"], "chamfer");
    }

    fn imprint_handles(kernel: &mut BrepKernel) -> (u32, u32) {
        let target = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let tool = kernel.make_box_solid(6.0, 6.0, 6.0).unwrap();
        kernel
            .transform_solid_binding(
                tool,
                vec![
                    1.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, -3.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0,
                ],
            )
            .unwrap();
        (target, tool)
    }

    #[test]
    fn direct_and_batch_imprint_match_and_surface_the_journal_entry() {
        let mut signatures = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let (target, tool) = imprint_handles(&mut kernel);
            let payload = if batch {
                run(
                    &mut kernel,
                    serde_json::json!([{
                        "op": "imprint",
                        "args": {"target": target, "tool": tool},
                    }]),
                )[0]
                .clone()
            } else {
                serde_json::from_str(&kernel.imprint_js(target, tool).unwrap()).unwrap()
            };
            let solid = payload["solid"].as_u64().unwrap() as u32;
            let op = payload["op"].as_u64().unwrap();
            let volume = kernel.volume(solid, 0.01).unwrap();
            let faces = kernel.get_solid_faces(solid).unwrap();
            assert!((volume - 1000.0).abs() < 1.0e-9);
            assert!(faces.len() > 6);

            let summary = run(
                &mut kernel,
                serde_json::json!([{"op": "journalSummary", "args": {}}]),
            );
            let entry = summary[0]
                .as_array()
                .unwrap()
                .iter()
                .find(|entry| entry["op"].as_u64() == Some(op))
                .unwrap();
            assert_eq!(entry["kind"], "imprint");
            assert_eq!(entry["type"], "evolution");
            assert_eq!(entry["detail"]["origin"], "construction");
            signatures.push(((volume * 1.0e9).round() as i64, faces.len()));
        }
        assert_eq!(signatures[0], signatures[1]);
    }

    #[test]
    fn batch_v2_imprint_preserves_typed_atomic_refusal() {
        let mut kernel = BrepKernel::new();
        let target = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let counts_before = kernel.topo().allocated_slot_count();
        let response: Vec<serde_json::Value> = serde_json::from_str(
            &kernel.execute_batch_v2(
                &serde_json::json!([{
                    "op": "imprint",
                    "args": {"target": target, "tool": target},
                }])
                .to_string(),
            ),
        )
        .unwrap();
        assert_eq!(
            response[0]["error"]["details"]["kernelCode"],
            "unsupported_imprint"
        );
        assert_eq!(response[0]["error"]["category"], "unsupported");
        assert_eq!(kernel.topo().allocated_slot_count(), counts_before);
    }

    #[test]
    fn entity_evolution_face_indices_are_live_in_the_result() {
        use std::collections::BTreeSet;
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let b = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        kernel
            .transform_solid_binding(
                b,
                vec![
                    1.0, 0.0, 0.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
            )
            .unwrap();
        let inputs: BTreeSet<u32> = kernel
            .get_solid_faces(a)
            .unwrap()
            .into_iter()
            .chain(kernel.get_solid_faces(b).unwrap())
            .collect();
        let payload: serde_json::Value =
            serde_json::from_str(&kernel.fuse_with_entity_evolution(a, b).unwrap()).unwrap();
        let solid = u32::try_from(payload["solid"].as_u64().unwrap()).unwrap();
        let live: BTreeSet<u32> = kernel.get_solid_faces(solid).unwrap().into_iter().collect();
        // Native-to-WASM payload consistency: every reported result face
        // is a live face of the result solid, and every named source is a
        // pre-operation face of an operand. Consumed inputs are absent
        // rather than listed — the payload has no deleted bucket, so a
        // missing input is unknown, never implicitly surviving.
        let faces = payload["evolution"]["faces"].as_array().unwrap();
        assert!(!faces.is_empty());
        assert!(
            payload["evolution"].get("deleted").is_none(),
            "the Issue-12 payload has no deleted bucket: consumed inputs are \
             absent, never listed"
        );
        let mut sourced = 0;
        for face in faces {
            let output = u32::try_from(face["face"].as_u64().unwrap()).unwrap();
            assert!(
                live.contains(&output),
                "result face {output} must be live on the fused solid"
            );
            if !face["source"].is_null() {
                let source = u32::try_from(face["source"].as_u64().unwrap()).unwrap();
                assert!(
                    inputs.contains(&source),
                    "source {source} must be a pre-operation operand face"
                );
                sourced += 1;
            }
        }
        assert!(
            sourced > 0,
            "an all-generated payload would pass vacuously: overlapping boxes \
             must carry at least one sourced face"
        );
        let edges = payload["evolution"]["edges"].as_array().unwrap();
        assert!(!edges.is_empty());
        for edge in edges {
            assert!(
                ["preserved", "modified", "generated", "unresolved"]
                    .contains(&edge["event"].as_str().unwrap())
            );
        }
        assert!(
            edges
                .iter()
                .any(|edge| edge["event"].as_str().unwrap() == "modified"),
            "crossing edges of overlapping boxes must be modified, got {edges:?}"
        );
    }

    #[test]
    fn journaled_move_op_resolves_through_naming_and_summary() {
        let mut kernel = BrepKernel::new();
        let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let face = kernel.get_solid_faces(source).unwrap()[0];
        let moved: serde_json::Value = serde_json::from_str(
            &kernel
                .move_faces_journaled_js(source, &[face], 2.0)
                .unwrap(),
        )
        .unwrap();
        let op = u32::try_from(moved["op"].as_u64().unwrap()).unwrap();
        let result_solid = u32::try_from(moved["solid"].as_u64().unwrap()).unwrap();
        let live: std::collections::BTreeSet<u32> = kernel
            .get_solid_faces(result_solid)
            .unwrap()
            .into_iter()
            .collect();
        // The op id returned in the payload feeds the naming path: every
        // face output resolves bound through it, to a live result face.
        for index in 0..6u32 {
            let resolution: serde_json::Value =
                serde_json::from_str(&kernel.resolve_operation_output(op, "face", index).unwrap())
                    .unwrap();
            assert_eq!(
                resolution["status"], "bound",
                "face output {index} must resolve bound, got {resolution}"
            );
            let handles: Vec<u32> = resolution["entities"]
                .as_array()
                .unwrap()
                .iter()
                .map(|entity| u32::try_from(entity["handle"].as_u64().unwrap()).unwrap())
                .collect();
            assert_eq!(
                handles.len(),
                1,
                "one output binds one face, got {resolution}"
            );
            assert!(
                live.contains(&handles[0]),
                "bound face {} must be live on the result solid",
                handles[0]
            );
        }
        // The read-only summary exposes the same entry with its origin.
        let summary: serde_json::Value = serde_json::from_str(&kernel.journal_summary()).unwrap();
        let entry = summary
            .as_array()
            .unwrap()
            .iter()
            .find(|entry| entry["op"].as_u64() == Some(u64::from(op)))
            .unwrap();
        assert_eq!(entry["kind"], "move_faces");
        assert_eq!(entry["type"], "evolution");
        assert_eq!(entry["detail"]["origin"], "construction");
    }

    #[test]
    fn linear_pattern_journaled_resolves_all_kinds_direct_and_batch() {
        // Success cell: box count 2, disjoint. Direct and batch must agree,
        // and every result face/edge/vertex must resolve bound with
        // construction provenance (total F/E/V lineage, not faces-only).
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
            let (compound, op) = if batch {
                let response: serde_json::Value = serde_json::from_str(
                    &kernel.execute_batch(
                        &serde_json::json!([{
                            "op": "linearPatternJournaled",
                            "args": {
                                "solid": source,
                                "direction": [1.0, 0.0, 0.0],
                                "spacing": 25.0,
                                "count": 2,
                            },
                        }])
                        .to_string(),
                    ),
                )
                .unwrap();
                let ok = &response[0]["ok"];
                let compound = u32::try_from(ok["compound"].as_u64().unwrap()).unwrap();
                let op = u32::try_from(ok["op"].as_u64().unwrap()).unwrap();
                (compound, op)
            } else {
                let payload: serde_json::Value = serde_json::from_str(
                    &kernel
                        .linear_pattern_journaled_js(source, 1.0, 0.0, 0.0, 25.0, 2)
                        .unwrap(),
                )
                .unwrap();
                let compound = u32::try_from(payload["compound"].as_u64().unwrap()).unwrap();
                let op = u32::try_from(payload["op"].as_u64().unwrap()).unwrap();
                (compound, op)
            };
            // 6+6 faces, 12+12 edges, 8+8 vertices across two boxes.
            for (kind, expected) in [("face", 12u32), ("edge", 24u32), ("vertex", 16u32)] {
                let mut handles = std::collections::BTreeSet::new();
                for index in 0..expected {
                    let resolution: serde_json::Value = if batch {
                        let batch_response: serde_json::Value = serde_json::from_str(
                            &kernel.execute_batch(
                                &serde_json::json!([{
                                    "op": "resolveOperationOutput",
                                    "args": {"op": op, "kind": kind, "index": index},
                                }])
                                .to_string(),
                            ),
                        )
                        .unwrap();
                        batch_response[0]["ok"].clone()
                    } else {
                        serde_json::from_str(
                            &kernel.resolve_operation_output(op, kind, index).unwrap(),
                        )
                        .unwrap()
                    };
                    assert_eq!(
                        resolution["status"], "bound",
                        "{kind} output {index} must resolve bound, got {resolution}"
                    );
                    assert_eq!(resolution["provenance"], "construction");
                    let entities = resolution["entities"].as_array().unwrap();
                    assert_eq!(entities.len(), 1);
                    let handle = entities[0]["handle"].as_u64().unwrap();
                    assert!(handles.insert(handle), "{kind} outputs must be distinct");
                    assert_eq!(entities[0]["kind"], kind);
                }
                let _ = compound;
            }
            // Same entry, same origin through the read-only summary.
            let summary: serde_json::Value =
                serde_json::from_str(&kernel.journal_summary()).unwrap();
            let entry = summary
                .as_array()
                .unwrap()
                .iter()
                .find(|entry| entry["op"].as_u64() == Some(u64::from(op)))
                .unwrap();
            assert_eq!(entry["kind"], "linear_pattern");
            assert_eq!(entry["type"], "evolution");
            assert_eq!(entry["detail"]["origin"], "construction");
        }
        // Refusal cells: batch returns typed errors, native refuses the same
        // inputs, and the source geometry survives. Direct error paths are
        // not exercised natively (JsError cannot construct there); batch is
        // the WASM refusal surface.
        let mut kernel = BrepKernel::new();
        let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        for (direction, spacing, count, tag) in [
            ([1.0, 0.0, 0.0], 0.0, 2u32, "spacing"),
            ([1.0, 0.0, 0.0], 25.0, 0u32, "count"),
            ([0.0, 0.0, 0.0], 25.0, 2u32, "direction"),
            ([1.0, 0.0, 0.0], 5.0, 2u32, "overlap"),
        ] {
            let response: serde_json::Value = serde_json::from_str(
                &kernel.execute_batch(
                    &serde_json::json!([{
                        "op": "linearPatternJournaled",
                        "args": {
                            "solid": source,
                            "direction": direction,
                            "spacing": spacing,
                            "count": count,
                        },
                    }])
                    .to_string(),
                ),
            )
            .unwrap();
            assert!(
                response[0].get("error").is_some(),
                "{tag} refusal must surface a batch error, got {}",
                response[0]
            );
            let _ = tag;
        }
        // Foreign handle refuses on both surfaces without publishing history.
        let retired = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let retired_id = kernel.resolve_solid(retired).unwrap();
        kernel.topo_mut().delete_solid(retired_id).unwrap();
        let entries_before = kernel.topo().journal().entries().len();
        let response: serde_json::Value = serde_json::from_str(
            &kernel.execute_batch(
                &serde_json::json!([{
                    "op": "linearPatternJournaled",
                    "args": {
                        "solid": retired,
                        "direction": [1.0, 0.0, 0.0],
                        "spacing": 12.0,
                        "count": 2,
                    },
                }])
                .to_string(),
            ),
        )
        .unwrap();
        assert!(response[0].get("error").is_some());
        assert_eq!(
            kernel.topo().journal().entries().len(),
            entries_before,
            "foreign-handle refusal must not publish history"
        );
    }

    #[test]
    fn circular_pattern_journaled_resolves_all_kinds_direct_and_batch() {
        // Success cell: origin box count 2 about Z (point-touching, supported).
        // Direct and batch must agree, and every result face/edge/vertex must
        // resolve bound with construction provenance (total F/E/V lineage).
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
            let (compound, op) = if batch {
                let response: serde_json::Value = serde_json::from_str(
                    &kernel.execute_batch(
                        &serde_json::json!([{
                            "op": "circularPatternJournaled",
                            "args": {
                                "solid": source,
                                "axis": [0.0, 0.0, 1.0],
                                "count": 2,
                            },
                        }])
                        .to_string(),
                    ),
                )
                .unwrap();
                let ok = &response[0]["ok"];
                let compound = u32::try_from(ok["compound"].as_u64().unwrap()).unwrap();
                let op = u32::try_from(ok["op"].as_u64().unwrap()).unwrap();
                (compound, op)
            } else {
                let payload: serde_json::Value = serde_json::from_str(
                    &kernel
                        .circular_pattern_journaled_js(source, 0.0, 0.0, 1.0, 2)
                        .unwrap(),
                )
                .unwrap();
                let compound = u32::try_from(payload["compound"].as_u64().unwrap()).unwrap();
                let op = u32::try_from(payload["op"].as_u64().unwrap()).unwrap();
                (compound, op)
            };
            // 6+6 faces, 12+12 edges, 8+8 vertices across two boxes.
            for (kind, expected) in [("face", 12u32), ("edge", 24u32), ("vertex", 16u32)] {
                let mut handles = std::collections::BTreeSet::new();
                for index in 0..expected {
                    let resolution: serde_json::Value = if batch {
                        let batch_response: serde_json::Value = serde_json::from_str(
                            &kernel.execute_batch(
                                &serde_json::json!([{
                                    "op": "resolveOperationOutput",
                                    "args": {"op": op, "kind": kind, "index": index},
                                }])
                                .to_string(),
                            ),
                        )
                        .unwrap();
                        batch_response[0]["ok"].clone()
                    } else {
                        serde_json::from_str(
                            &kernel.resolve_operation_output(op, kind, index).unwrap(),
                        )
                        .unwrap()
                    };
                    assert_eq!(
                        resolution["status"], "bound",
                        "{kind} output {index} must resolve bound, got {resolution}"
                    );
                    assert_eq!(resolution["provenance"], "construction");
                    let entities = resolution["entities"].as_array().unwrap();
                    assert_eq!(entities.len(), 1);
                    let handle = entities[0]["handle"].as_u64().unwrap();
                    assert!(handles.insert(handle), "{kind} outputs must be distinct");
                    assert_eq!(entities[0]["kind"], kind);
                }
                let _ = compound;
            }
            let summary: serde_json::Value =
                serde_json::from_str(&kernel.journal_summary()).unwrap();
            let entry = summary
                .as_array()
                .unwrap()
                .iter()
                .find(|entry| entry["op"].as_u64() == Some(u64::from(op)))
                .unwrap();
            assert_eq!(entry["kind"], "circular_pattern");
            assert_eq!(entry["type"], "evolution");
            assert_eq!(entry["detail"]["origin"], "construction");
        }
        // Disjoint count 3 via an offset box: direct and batch agree on the
        // same compound/op shape and total F/E/V census (18/36/24).
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
            let source_id = kernel.resolve_solid(source).unwrap();
            remus_operations::transform::transform_solid(
                kernel.topo_mut(),
                source_id,
                &remus_math::mat::Mat4::translation(30.0, 0.0, 0.0),
            )
            .unwrap();
            let (compound, op) = if batch {
                let response: serde_json::Value = serde_json::from_str(
                    &kernel.execute_batch(
                        &serde_json::json!([{
                            "op": "circularPatternJournaled",
                            "args": {
                                "solid": source,
                                "axis": [0.0, 0.0, 1.0],
                                "count": 3,
                            },
                        }])
                        .to_string(),
                    ),
                )
                .unwrap();
                let ok = &response[0]["ok"];
                (
                    u32::try_from(ok["compound"].as_u64().unwrap()).unwrap(),
                    u32::try_from(ok["op"].as_u64().unwrap()).unwrap(),
                )
            } else {
                let payload: serde_json::Value = serde_json::from_str(
                    &kernel
                        .circular_pattern_journaled_js(source, 0.0, 0.0, 1.0, 3)
                        .unwrap(),
                )
                .unwrap();
                (
                    u32::try_from(payload["compound"].as_u64().unwrap()).unwrap(),
                    u32::try_from(payload["op"].as_u64().unwrap()).unwrap(),
                )
            };
            for (kind, expected) in [("face", 18u32), ("edge", 36u32), ("vertex", 24u32)] {
                let resolution: serde_json::Value = if batch {
                    let batch_response: serde_json::Value = serde_json::from_str(
                        &kernel.execute_batch(
                            &serde_json::json!([{
                                "op": "resolveOperationOutput",
                                "args": {"op": op, "kind": kind, "index": 0},
                            }])
                            .to_string(),
                        ),
                    )
                    .unwrap();
                    batch_response[0]["ok"].clone()
                } else {
                    serde_json::from_str(&kernel.resolve_operation_output(op, kind, 0).unwrap())
                        .unwrap()
                };
                // At least the first output of each kind resolves bound.
                assert_eq!(resolution["status"], "bound");
                assert_eq!(resolution["provenance"], "construction");
                let _ = expected;
                let _ = compound;
            }
            let summary: serde_json::Value =
                serde_json::from_str(&kernel.journal_summary()).unwrap();
            let entry = summary
                .as_array()
                .unwrap()
                .iter()
                .find(|entry| entry["op"].as_u64() == Some(u64::from(op)))
                .unwrap();
            assert_eq!(entry["kind"], "circular_pattern");
        }
        // Refusal cells: batch returns typed errors, and the source survives.
        // Direct error paths are not exercised natively (JsError cannot
        // construct there); batch is the WASM refusal surface.
        let mut kernel = BrepKernel::new();
        let source = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        for (axis, count, tag) in [
            ([0.0, 0.0, 0.0], 2u32, "axis"),
            ([1.0, 0.0, 0.0], 0u32, "count0"),
            ([1.0, 0.0, 0.0], 1u32, "count1"),
        ] {
            let response: serde_json::Value = serde_json::from_str(
                &kernel.execute_batch(
                    &serde_json::json!([{
                        "op": "circularPatternJournaled",
                        "args": {
                            "solid": source,
                            "axis": axis,
                            "count": count,
                        },
                    }])
                    .to_string(),
                ),
            )
            .unwrap();
            assert!(
                response[0].get("error").is_some(),
                "{tag} refusal must surface a batch error, got {}",
                response[0]
            );
        }
        // Material overlap refuses: a Z-coaxial cylinder rotated about Z sits
        // on itself, so instances 0 and 1 overlap by the full volume.
        let cylinder = kernel.make_cylinder_solid(2.0, 5.0).unwrap();
        let response: serde_json::Value = serde_json::from_str(
            &kernel.execute_batch(
                &serde_json::json!([{
                    "op": "circularPatternJournaled",
                    "args": {
                        "solid": cylinder,
                        "axis": [0.0, 0.0, 1.0],
                        "count": 2,
                    },
                }])
                .to_string(),
            ),
        )
        .unwrap();
        assert!(
            response[0].get("error").is_some(),
            "overlap refusal must surface a batch error, got {}",
            response[0]
        );
        // Foreign handle refuses without publishing history.
        let retired = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let retired_id = kernel.resolve_solid(retired).unwrap();
        kernel.topo_mut().delete_solid(retired_id).unwrap();
        let entries_before = kernel.topo().journal().entries().len();
        let response: serde_json::Value = serde_json::from_str(
            &kernel.execute_batch(
                &serde_json::json!([{
                    "op": "circularPatternJournaled",
                    "args": {
                        "solid": retired,
                        "axis": [0.0, 0.0, 1.0],
                        "count": 2,
                    },
                }])
                .to_string(),
            ),
        )
        .unwrap();
        assert!(response[0].get("error").is_some());
        assert_eq!(
            kernel.topo().journal().entries().len(),
            entries_before,
            "foreign-handle refusal must not publish history"
        );
    }
}
