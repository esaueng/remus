//! Shared free functions and constants used across WASM binding modules.

#![allow(
    clippy::missing_errors_doc,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_operations::boolean::BooleanOp;
use remus_operations::tessellate;
use remus_topology::Topology;
use wasm_bindgen::prelude::*;

use crate::error::{StructuredWasmError, WasmError};
use crate::handles::face_id_to_u32;
use crate::shapes::JsMesh;

/// Default tolerance for vertices created by the kernel.
pub const TOL: f64 = 1e-7;

// ── Parsing helpers ───────────────────────────────────────────────

/// Parse flat `[x,y,z, ...]` coordinates into `Vec<Point3>`.
pub fn parse_points(coords: &[f64]) -> Result<Vec<Point3>, JsError> {
    if !coords.len().is_multiple_of(3) {
        return Err(WasmError::InvalidInput {
            reason: format!(
                "coordinate array length must be a multiple of 3, got {}",
                coords.len()
            ),
        }
        .into());
    }
    Ok(coords
        .chunks_exact(3)
        .map(|c| Point3::new(c[0], c[1], c[2]))
        .collect())
}

/// Parse flat coordinates into a 2D grid of points.
pub fn parse_point_grid(
    coords: &[f64],
    rows: usize,
    cols: usize,
) -> Result<Vec<Vec<Point3>>, JsError> {
    if rows == 0 || cols == 0 {
        return Err(WasmError::InvalidInput {
            reason: format!("rows and cols must be > 0, got {rows}x{cols}"),
        }
        .into());
    }
    let total = rows
        .checked_mul(cols)
        .ok_or_else(|| WasmError::InvalidInput {
            reason: format!("rows*cols overflow: {rows}*{cols}"),
        })?;
    let points = parse_points(coords)?;
    if points.len() != total {
        return Err(WasmError::InvalidInput {
            reason: format!(
                "expected {total} points ({rows}x{cols}), got {}",
                points.len()
            ),
        }
        .into());
    }
    Ok(points.chunks(cols).map(<[Point3]>::to_vec).collect())
}

/// Parse a flat 16-element array into a `Mat4` (row-major).
pub fn parse_mat4(elems: &[f64]) -> Result<Mat4, JsError> {
    if elems.len() != 16 {
        return Err(WasmError::InvalidInput {
            reason: format!("matrix requires 16 elements, got {}", elems.len()),
        }
        .into());
    }
    let rows = std::array::from_fn(|i| std::array::from_fn(|j| elems[i * 4 + j]));
    Ok(Mat4(rows))
}

/// Convert a `Mat4` to a flat 16-element f64 array for JSON (row-major).
pub fn mat4_to_array(mat: &Mat4) -> Vec<f64> {
    let mut out = Vec::with_capacity(16);
    for row in &mat.0 {
        for &v in row {
            out.push(v);
        }
    }
    out
}

/// Parse a boolean operation string to the enum.
pub fn parse_boolean_op(op: &str) -> Result<BooleanOp, JsError> {
    match op {
        "fuse" | "union" => Ok(BooleanOp::Fuse),
        "cut" | "difference" => Ok(BooleanOp::Cut),
        "intersect" | "intersection" => Ok(BooleanOp::Intersect),
        _ => Err(WasmError::InvalidInput {
            reason: format!("unknown boolean op: {op}"),
        }
        .into()),
    }
}

/// Extract a required `f64` value from a JSON object.
pub fn get_f64(args: &serde_json::Value, key: &str) -> Result<f64, StructuredWasmError> {
    args[key].as_f64().ok_or_else(|| {
        StructuredWasmError::invalid_argument(format!("missing or invalid '{key}'"), Some(key))
    })
}

/// Extract a required array of `f64` from a JSON object.
///
/// # Errors
///
/// Returns a message naming `key` if it is missing or not an array, or
/// naming the offending index if an element is not a number.
pub fn get_f64_array(args: &serde_json::Value, key: &str) -> Result<Vec<f64>, StructuredWasmError> {
    args[key]
        .as_array()
        .ok_or_else(|| {
            StructuredWasmError::invalid_argument(
                format!("missing or invalid '{key}' array"),
                Some(key),
            )
        })?
        .iter()
        .enumerate()
        .map(|(i, v)| {
            v.as_f64().ok_or_else(|| {
                let argument = format!("{key}[{i}]");
                StructuredWasmError::invalid_argument(
                    format!("{argument} is not a number"),
                    Some(&argument),
                )
            })
        })
        .collect()
}

/// Extract a required array of `u32` from a JSON object.
///
/// # Errors
///
/// Returns a message naming `key` if it is missing or not an array, or
/// naming the offending index if an element is not a `u32`.
pub fn get_u32_array(args: &serde_json::Value, key: &str) -> Result<Vec<u32>, StructuredWasmError> {
    args[key]
        .as_array()
        .ok_or_else(|| {
            StructuredWasmError::invalid_argument(
                format!("missing or invalid '{key}' array"),
                Some(key),
            )
        })?
        .iter()
        .enumerate()
        .map(|(i, v)| {
            v.as_u64()
                .and_then(|n| u32::try_from(n).ok())
                .ok_or_else(|| {
                    let argument = format!("{key}[{i}]");
                    StructuredWasmError::invalid_argument(
                        format!("{argument} is not a u32"),
                        Some(&argument),
                    )
                })
        })
        .collect()
}

/// Extract a required `u32` value from a JSON object.
pub fn get_u32(args: &serde_json::Value, key: &str) -> Result<u32, StructuredWasmError> {
    args[key]
        .as_u64()
        .ok_or_else(|| {
            StructuredWasmError::invalid_argument(format!("missing or invalid '{key}'"), Some(key))
        })
        .and_then(|value| {
            u32::try_from(value).map_err(|_| {
                StructuredWasmError::invalid_argument(
                    format!("'{key}' exceeds the u32 range"),
                    Some(key),
                )
            })
        })
}

/// Extract a `usize` from a JSON value.
pub fn json_usize(val: &serde_json::Value, key: &str) -> Result<usize, JsError> {
    val[key].as_u64().map(|v| v as usize).ok_or_else(|| {
        WasmError::InvalidInput {
            reason: format!("missing or invalid '{key}'"),
        }
        .into()
    })
}

/// Extract an `f64` from a JSON value.
pub fn json_f64(val: &serde_json::Value, key: &str) -> Result<f64, JsError> {
    val[key].as_f64().ok_or_else(|| {
        WasmError::InvalidInput {
            reason: format!("missing or invalid '{key}'"),
        }
        .into()
    })
}

// ── Edge/face helpers ─────────────────────────────────────────────

/// Attempt a fillet, preferring the v2 walking engine and validating output.
///
/// Engine order is a product decision (2026-07): the v2 walking engine
/// (`blend_ops::fillet_v2`, the maintained `crates/blend` engine with
/// validated fail-closed contracts) runs first; the deprecated v1
/// rolling-ball engine is the fallback for cases v2 cannot yet complete,
/// and the v1 flat bevel is the last resort. On the box single-edge,
/// disjoint-multi-edge, and all-12-edge cases the two primary engines
/// produce identical volumes and face counts (the historical "v2
/// over-removes corner material" note predates the corner-solver fixes and
/// no longer reproduces).
///
/// Every candidate is validated as a closed (watertight) solid before being
/// accepted, so a malformed result is rejected in favour of the next engine.
/// This guard lets `filter_filletable_edges` be permissive about curved
/// neighbours without ever returning a degenerate solid.
///
/// # Errors
///
/// When no engine produces a valid closed solid, returns the v2 walking
/// engine's typed error — the engine with meaningful diagnostics
/// (`UnsupportedVertexBlend`, `TrimmingFailure`, `RadiusTooLarge`, …) —
/// rather than the historical silent input-handle no-op, which left callers
/// unable to distinguish "radius too large" from "unsupported topology"
/// (`try_chamfer`'s doc comment calls that the no-op trap). The input solid
/// is always rolled back to its untouched pre-attempt state on failure.
#[allow(deprecated)]
pub fn try_fillet(
    topo: &mut remus_topology::Topology,
    solid_id: remus_topology::solid::SolidId,
    edge_ids: &[remus_topology::edge::EdgeId],
    radius: f64,
) -> Result<remus_topology::solid::SolidId, remus_operations::OperationsError> {
    try_fillet_with_origins(topo, solid_id, edge_ids, radius).map(|(solid, _)| solid)
}

/// [`try_fillet`], also returning whatever face provenance the engine that
/// succeeded recorded.
///
/// Only the walking builder keeps a record; the rolling-ball rebuilds re-mint
/// the faces they touch and return `None`, which the caller must report as an
/// inference rather than passing off as fact.
///
/// # Errors
///
/// Same as [`try_fillet`].
#[allow(deprecated)]
pub fn try_fillet_with_origins(
    topo: &mut remus_topology::Topology,
    solid_id: remus_topology::solid::SolidId,
    edge_ids: &[remus_topology::edge::EdgeId],
    radius: f64,
) -> Result<
    (
        remus_topology::solid::SolidId,
        Option<remus_operations::blend_ops::BlendFaceOrigins>,
    ),
    remus_operations::OperationsError,
> {
    // Drop tangent / degenerate edges (e.g. a fillet face's G1 contact line
    // with its planar neighbour). If none qualify there is nothing to blend,
    // which is a selection problem the caller must hear about — not a
    // success.
    let edges = remus_operations::query::filter_filletable_edges(topo, solid_id, edge_ids)?;
    if edges.is_empty() {
        return Err(remus_operations::OperationsError::InvalidInput {
            reason:
                "no filletable edges in the selection (tangent and degenerate edges are skipped)"
                    .into(),
        });
    }
    let edges = edges.as_slice();

    // A candidate is acceptable only if its outer shell is a CLOSED 2-manifold
    // (every edge used by exactly two faces — no free/boundary edges). The
    // weaker manifold-only check silently accepted open shells (e.g. a fillet
    // that leaves a cap untrimmed at a contact circle), which tessellate to a
    // plausible-but-wrong volume; reject them so the next engine or the
    // unchanged input is used.
    let is_valid = |topo: &remus_topology::Topology, s: remus_topology::solid::SolidId| -> bool {
        topo.solid(s)
            .and_then(|sd| topo.shell(sd.outer_shell()))
            .map(|sh| remus_topology::validation::validate_shell_closed(sh, topo).is_ok())
            .unwrap_or(false)
    };

    // Every engine mutates the shared arena in place (the trimmer's
    // `propagate_split` rewrites the wires of each face touching a split
    // edge; the rolling-ball rebuild rewrites cap wires). A rejected attempt
    // therefore leaves the INPUT solid partly filleted — rounded corners plus
    // free edges where a split was applied but never closed. A caller that
    // reports the failure and keeps using its original handle then ships
    // that corrupted body: the OpenZCAD demo bracket meshed with 42 boundary
    // edges even though its fillet had "failed".
    //
    // Snapshot once, roll back after every rejected attempt so the next
    // engine starts clean and a total failure is a true no-op on the input.
    // Handle slots are preserved, so IDs held by the caller stay valid.
    let snapshot = topo.clone();

    // v2 (the walking blend) is tried first: it is the engine under active
    // development and matches v1 on every measured case. Its failure is
    // remembered verbatim — if the fallback engines cannot rescue the call,
    // that typed diagnosis is what the caller receives.
    let v2_failure = match remus_operations::blend_ops::fillet_v2(topo, solid_id, edges, radius) {
        Ok(r) if is_valid(topo, r.solid) => return Ok((r.solid, r.face_origins)),
        Ok(_) => remus_operations::OperationsError::InvalidInput {
            reason: "fillet produced an open shell".into(),
        },
        Err(e) => e,
    };
    topo.restore_preserving_handle_slots(&snapshot);

    if let Ok(s) = remus_operations::fillet::fillet_rolling_ball(topo, solid_id, edges, radius)
        && is_valid(topo, s)
    {
        return Ok((s, None));
    }
    topo.restore_preserving_handle_slots(&snapshot);

    if let Ok(s) = remus_operations::fillet::fillet(topo, solid_id, edges, radius)
        && is_valid(topo, s)
    {
        return Ok((s, None));
    }
    topo.restore_preserving_handle_slots(&snapshot);

    // No engine produced a valid solid — the input is unchanged, and the
    // walking engine's diagnosis names the blocker.
    Err(v2_failure)
}

/// [`try_fillet`], with the rule that the answer must cover every edge named.
///
/// The `fillet` binding used to retry on the PLANAR SUBSET of the selection
/// whenever the whole set failed, and return that subset's solid as if it were
/// the blend that was asked for. Nothing about the returned handle said
/// otherwise: a fresh, valid, watertight solid whose volume sat inside the
/// plausible envelope. A selection mixing a plate's top perimeter with its bore
/// rim came back byte-identical to the perimeter-only result, the rim silently
/// dropped — though that rim rounds perfectly when picked on its own. A caller
/// had no way to detect it.
///
/// The retry still runs, because it is a cheap way to find out whether the
/// non-planar edges are the blocker, but its solid is never the answer: when it
/// covers only part of the selection the result is a typed
/// [`BlendError::EdgesNotBlended`] naming the edges that carry no blend, and
/// the input is left untouched.
///
/// # Errors
///
/// Returns the engine chain's typed failure, or `EdgesNotBlended` when the only
/// thing that would have succeeded is a strict subset of the selection.
pub fn fillet_whole_selection(
    topo: &mut remus_topology::Topology,
    solid_id: remus_topology::solid::SolidId,
    edge_ids: &[remus_topology::edge::EdgeId],
    radius: f64,
) -> Result<remus_topology::solid::SolidId, remus_operations::OperationsError> {
    let primary = match try_fillet(topo, solid_id, edge_ids, radius) {
        Ok(solid) => return Ok(solid),
        Err(e) => e,
    };

    let planar_edges = remus_operations::query::filter_planar_edges(topo, solid_id, edge_ids)?;
    if planar_edges.is_empty() || planar_edges.len() == edge_ids.len() {
        return Err(primary);
    }
    let dropped: Vec<remus_topology::edge::EdgeId> = edge_ids
        .iter()
        .copied()
        .filter(|e| !planar_edges.contains(e))
        .collect();
    Err(remus_operations::OperationsError::Blend(
        remus_operations::blend_ops::BlendError::EdgesNotBlended {
            edges: dropped,
            reason: format!(
                "the blend engines refused the whole selection ({primary}); the {} \
                 edge(s) between planar faces would round on their own, but \
                 returning that subset would drop the rest without saying so",
                planar_edges.len()
            ),
        },
    ))
}

/// Chamfer edges, falling back to the v2 walking engine when the v1 flat-bevel
/// engine cannot handle the geometry.
///
/// v1 is tried FIRST so every case it already handles keeps its exact current
/// behaviour — this is purely additive, turning errors into successes rather
/// than moving work between engines. v1 is planar-only and errors out on a
/// curved neighbour (a cylinder rim reports "cannot normalize zero vector"),
/// which is what left the OpenZCAD flange demo unable to chamfer its rim once
/// the booleans went analytic and started handing it real circles.
///
/// Note `blend_ops::chamfer_v2` itself routes planar-line edge sets back to the
/// same v1 code, so the fallback only ever adds the builder path.
///
/// # Errors
///
/// Returns the v2 engine's error if both engines fail. Unlike `try_fillet`,
/// failure is NOT reported by returning the input handle: a chamfer that
/// silently does nothing while reporting success is the no-op trap, and the
/// caller (and its user) is better served by the error.
pub fn try_chamfer(
    topo: &mut remus_topology::Topology,
    solid_id: remus_topology::solid::SolidId,
    edge_ids: &[remus_topology::edge::EdgeId],
    distance: f64,
) -> Result<remus_topology::solid::SolidId, remus_operations::OperationsError> {
    try_chamfer_with_origins(topo, solid_id, edge_ids, distance).map(|(solid, _)| solid)
}

/// [`try_chamfer`] with construction history from whichever production engine
/// succeeds.
pub fn try_chamfer_with_origins(
    topo: &mut remus_topology::Topology,
    solid_id: remus_topology::solid::SolidId,
    edge_ids: &[remus_topology::edge::EdgeId],
    distance: f64,
) -> Result<
    (
        remus_topology::solid::SolidId,
        Option<remus_operations::blend_ops::BlendFaceOrigins>,
    ),
    remus_operations::OperationsError,
> {
    let is_valid = |topo: &remus_topology::Topology, s: remus_topology::solid::SolidId| -> bool {
        topo.solid(s)
            .and_then(|sd| topo.shell(sd.outer_shell()))
            .map(|sh| remus_topology::validation::validate_shell_closed(sh, topo).is_ok())
            .unwrap_or(false)
    };

    // Both engines mutate the shared arena in place, so a rejected attempt
    // leaves the input partly chamfered. Snapshot once and roll back after a
    // rejected attempt so the next engine starts clean — the same discipline
    // `try_fillet` uses, and for the same reason (a "failed" blend otherwise
    // ships a corrupted body).
    let snapshot = topo.clone();

    if let Ok((s, origins)) =
        remus_operations::blend_ops::planar_chamfer_with_origins(topo, solid_id, edge_ids, distance)
        && is_valid(topo, s)
    {
        return Ok((s, Some(origins)));
    }
    topo.restore_preserving_handle_slots(&snapshot);

    match remus_operations::blend_ops::chamfer_v2(topo, solid_id, edge_ids, distance, distance) {
        Ok(r) if is_valid(topo, r.solid) => Ok((r.solid, r.face_origins)),
        Ok(_) => {
            topo.restore_preserving_handle_slots(&snapshot);
            Err(remus_operations::OperationsError::InvalidInput {
                reason: "chamfer produced an open shell".into(),
            })
        }
        Err(e) => {
            topo.restore_preserving_handle_slots(&snapshot);
            Err(e)
        }
    }
}

/// Convert a fillet/chamfer failure into a `JsError` whose message starts
/// with the stable machine-readable code from
/// [`remus_operations::blend_ops::blend_failure_code`], e.g.
/// `unsupported-vertex-blend: blend: unsupported vertex blend at Id(16): 2
/// stripes meet`. Callers across the WASM boundary branch on the prefix.
pub fn fillet_failure_js_error(error: &remus_operations::OperationsError) -> JsError {
    JsError::new(&format!(
        "{}: {error}",
        remus_operations::blend_ops::blend_failure_code(error)
    ))
}

/// Extract a human-readable message from a `catch_unwind` panic payload.
pub fn panic_message(payload: &Box<dyn std::any::Any + Send>, operation: &str) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        format!("{operation} operation panicked: {s}")
    } else if let Some(s) = payload.downcast_ref::<String>() {
        format!("{operation} operation panicked: {s}")
    } else {
        format!("{operation} operation panicked (unknown cause)")
    }
}

/// Sample a closed periodic curve (period = TAU) into a flat `[x, y, z, ...]` buffer.
///
/// Produces `n` evenly-spaced points in `[0, TAU)` — the endpoint at `TAU` is
/// excluded because it duplicates `t = 0` on periodic curves. Callers that need
/// a closed polyline should append the first point or close the loop in JS.
///
/// Returns an empty buffer if `n == 0`.
pub fn sample_full_period_curve(n: usize, evaluate: impl Fn(f64) -> Point3) -> Vec<f64> {
    if n <= 1 {
        if n == 1 {
            let p = evaluate(0.0);
            return vec![p.x(), p.y(), p.z()];
        }
        return Vec::new();
    }
    let mut result = Vec::with_capacity(n * 3);
    for i in 0..n {
        let t = std::f64::consts::TAU * (i as f64) / (n as f64);
        let p = evaluate(t);
        result.push(p.x());
        result.push(p.y());
        result.push(p.z());
    }
    result
}

/// Sample an aperiodic curve over the closed parameter span `[t0, t1]`,
/// endpoints included.
///
/// The counterpart of [`sample_full_period_curve`] for curves with no
/// period — an unbounded conic branch has no `TAU` to wrap around, and its
/// extent comes entirely from the edge's two vertices, so both ends must be
/// emitted (unlike the periodic sampler, where the last sample would repeat
/// the first). Returns a flattened `[x, y, z, ...]` array.
#[must_use]
pub fn sample_open_span(n: usize, t0: f64, t1: f64, evaluate: impl Fn(f64) -> Point3) -> Vec<f64> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        let p = evaluate(t0);
        return vec![p.x(), p.y(), p.z()];
    }
    let mut result = Vec::with_capacity(n * 3);
    for i in 0..n {
        #[allow(clippy::cast_precision_loss)]
        let f = i as f64 / (n - 1) as f64;
        let p = evaluate((t1 - t0).mul_add(f, t0));
        result.push(p.x());
        result.push(p.y());
        result.push(p.z());
    }
    result
}

/// Create a tiny degenerate polygon face at a point, matching the vertex
/// count of the first existing profile. Used for loft start/end points.
pub fn create_apex_face(
    topo: &mut Topology,
    point: Point3,
    existing_profiles: &[remus_topology::face::FaceId],
) -> Result<remus_topology::face::FaceId, WasmError> {
    // Determine target vertex count from the first profile.
    let n = if let Some(&fid) = existing_profiles.first() {
        let verts = remus_operations::boolean::face_polygon(topo, fid)?;
        verts.len().max(3)
    } else {
        3
    };

    // Create a tiny polygon at the apex point.
    let epsilon = 1e-6;
    let mut pts = Vec::with_capacity(n);
    for i in 0..n {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
        pts.push(Point3::new(
            point.x() + epsilon * angle.cos(),
            point.y() + epsilon * angle.sin(),
            point.z(),
        ));
    }

    let wire_id = remus_topology::builder::make_polygon_wire(topo, &pts, TOL)?;
    let face_id = remus_topology::builder::make_face_from_wire(topo, wire_id)?;
    Ok(face_id)
}

// ── Mesh / tessellation helpers ───────────────────────────────────

/// Build a `TriangleMesh` from flat position/index arrays.
pub fn build_triangle_mesh(
    positions: &[f64],
    indices: &[u32],
) -> Result<tessellate::TriangleMesh, JsError> {
    if !positions.len().is_multiple_of(3) {
        return Err(WasmError::InvalidInput {
            reason: format!(
                "positions length must be a multiple of 3, got {}",
                positions.len()
            ),
        }
        .into());
    }
    let pts: Vec<Point3> = positions
        .chunks_exact(3)
        .map(|c| Point3::new(c[0], c[1], c[2]))
        .collect();
    // Compute normals as zero vectors (mesh_boolean recomputes them)
    let normals = vec![Vec3::new(0.0, 0.0, 0.0); pts.len()];
    Ok(tessellate::TriangleMesh {
        positions: pts,
        normals,
        indices: indices.to_vec(),
    })
}

/// Convert a `TriangleMesh` to `JsMesh`.
pub fn triangle_mesh_to_js(mesh: &tessellate::TriangleMesh) -> JsMesh {
    JsMesh::from(mesh.clone())
}

// ── Classification / serialization ────────────────────────────────

/// Convert a `PointClassification` to a string.
pub fn classify_to_string(c: remus_operations::classify::PointClassification) -> String {
    match c {
        remus_operations::classify::PointClassification::Inside => "inside".into(),
        remus_operations::classify::PointClassification::Outside => "outside".into(),
        remus_operations::classify::PointClassification::OnBoundary => "boundary".into(),
    }
}

/// Serialize a `Feature` enum to JSON.
pub fn serialize_feature(f: &remus_operations::feature_recognition::Feature) -> serde_json::Value {
    use remus_operations::feature_recognition::Feature;
    match f {
        Feature::Hole {
            faces,
            diameter,
            through,
        } => serde_json::json!({
            "type": "hole",
            "faces": faces.iter().map(|f| face_id_to_u32(*f)).collect::<Vec<_>>(),
            "diameter": diameter,
            "through": through,
        }),
        Feature::Chamfer {
            face,
            adjacent,
            angle,
        } => serde_json::json!({
            "type": "chamfer",
            "face": face_id_to_u32(*face),
            "adjacent": [face_id_to_u32(adjacent.0), face_id_to_u32(adjacent.1)],
            "angle": angle,
        }),
        Feature::FilletLike { face, area } => serde_json::json!({
            "type": "filletLike",
            "face": face_id_to_u32(*face),
            "area": area,
        }),
        Feature::Pocket { floor, walls } => serde_json::json!({
            "type": "pocket",
            "floor": face_id_to_u32(*floor),
            "walls": walls.iter().map(|f| face_id_to_u32(*f)).collect::<Vec<_>>(),
        }),
        Feature::Pattern {
            feature_indices,
            pattern_type,
            count,
            spacing,
        } => serde_json::json!({
            "type": "pattern",
            "featureIndices": feature_indices,
            "patternType": format!("{pattern_type:?}").to_lowercase(),
            "count": count,
            "spacing": spacing,
        }),
        _ => serde_json::json!({ "type": "unknown" }),
    }
}

#[cfg(test)]
mod feature_serialization_tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn hole_serialization_includes_through_classification() {
        let mut topo = remus_topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let face = remus_topology::explorer::solid_faces(&topo, solid).unwrap()[0];
        let feature = remus_operations::feature_recognition::Feature::Hole {
            faces: vec![face],
            diameter: Some(2.0),
            through: true,
        };

        let json = serialize_feature(&feature);
        assert_eq!(json["type"], "hole");
        assert_eq!(json["through"], true);
        assert_eq!(json["diameter"], 2.0);
    }
}

// ── Sketch constraint parsing ─────────────────────────────────────

/// Parse a sketch constraint from a JSON value.
pub fn parse_sketch_constraint(
    val: &serde_json::Value,
) -> Result<remus_operations::sketch::Constraint, JsError> {
    use remus_operations::sketch::Constraint;
    let ty = val["type"].as_str().unwrap_or("");
    match ty {
        "coincident" => {
            let p1 = json_usize(val, "p1")?;
            let p2 = json_usize(val, "p2")?;
            Ok(Constraint::Coincident(p1, p2))
        }
        "distance" => {
            let p1 = json_usize(val, "p1")?;
            let p2 = json_usize(val, "p2")?;
            let v = json_f64(val, "value")?;
            Ok(Constraint::Distance(p1, p2, v))
        }
        "fixX" => {
            let p = json_usize(val, "point")?;
            let v = json_f64(val, "value")?;
            Ok(Constraint::FixX(p, v))
        }
        "fixY" => {
            let p = json_usize(val, "point")?;
            let v = json_f64(val, "value")?;
            Ok(Constraint::FixY(p, v))
        }
        "vertical" => {
            let p1 = json_usize(val, "p1")?;
            let p2 = json_usize(val, "p2")?;
            Ok(Constraint::Vertical(p1, p2))
        }
        "horizontal" => {
            let p1 = json_usize(val, "p1")?;
            let p2 = json_usize(val, "p2")?;
            Ok(Constraint::Horizontal(p1, p2))
        }
        "angle" => {
            let p1 = json_usize(val, "p1")?;
            let p2 = json_usize(val, "p2")?;
            // Backward compat: old API was (p1, p2, value) for single-line angle.
            // New API is (p1, p2, p3, p4, value) for angle between two lines.
            // When p3/p4 are absent, default to p1/p2 (zero angle between same line).
            let p3 = val
                .get("p3")
                .and_then(serde_json::Value::as_u64)
                .map_or(p1, |v| v as usize);
            let p4 = val
                .get("p4")
                .and_then(serde_json::Value::as_u64)
                .map_or(p2, |v| v as usize);
            let v = json_f64(val, "value")?;
            Ok(Constraint::Angle(p1, p2, p3, p4, v))
        }
        "perpendicular" => {
            let p1 = json_usize(val, "p1")?;
            let p2 = json_usize(val, "p2")?;
            let p3 = json_usize(val, "p3")?;
            let p4 = json_usize(val, "p4")?;
            Ok(Constraint::Perpendicular(p1, p2, p3, p4))
        }
        "parallel" => {
            let p1 = json_usize(val, "p1")?;
            let p2 = json_usize(val, "p2")?;
            let p3 = json_usize(val, "p3")?;
            let p4 = json_usize(val, "p4")?;
            Ok(Constraint::Parallel(p1, p2, p3, p4))
        }
        _ => Err(WasmError::InvalidInput {
            reason: format!("unknown constraint type: {ty}"),
        }
        .into()),
    }
}

// ── 2D polygon helpers ────────────────────────────────────────────

/// Parse flat `[x,y, ...]` coordinates into `Vec<Point2>`.
pub fn parse_polygon_2d(coords: &[f64]) -> Result<Vec<Point2>, JsError> {
    Ok(parse_polygon_2d_checked(coords, "polygon")?)
}

/// [`parse_polygon_2d`] returning a [`WasmError`] instead of a `JsError`.
///
/// `JsError` cannot be constructed on non-wasm targets, so any parsing that
/// must stay reachable from native unit tests or from `executeBatch`
/// dispatch has to go through this form. `name` is used in the message so a
/// two-operand call can say which operand was malformed.
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] if the length is odd, fewer than
/// three points are supplied, or any coordinate is not finite.
pub fn parse_polygon_2d_checked(coords: &[f64], name: &str) -> Result<Vec<Point2>, WasmError> {
    if !coords.len().is_multiple_of(2) || coords.len() < 6 {
        return Err(WasmError::InvalidInput {
            reason: format!("{name} needs at least 3 points (6 coordinates)"),
        });
    }
    if let Some(pos) = coords.iter().position(|v| !v.is_finite()) {
        return Err(WasmError::InvalidInput {
            reason: format!("{name} coordinate at index {pos} is not finite"),
        });
    }
    Ok(coords
        .chunks_exact(2)
        .map(|c| Point2::new(c[0], c[1]))
        .collect())
}

/// Check if two 2D polygons overlap using vertex containment + edge crossing.
pub fn polygons_overlap_2d(a: &[Point2], b: &[Point2]) -> bool {
    use remus_math::predicates::point_in_polygon;

    // Check if any vertex of A is inside B or vice versa.
    for p in a {
        if point_in_polygon(*p, b) {
            return true;
        }
    }
    for p in b {
        if point_in_polygon(*p, a) {
            return true;
        }
    }

    // Check edge crossings.
    for i in 0..a.len() {
        let a1 = a[i];
        let a2 = a[(i + 1) % a.len()];
        for j in 0..b.len() {
            let b1 = b[j];
            let b2 = b[(j + 1) % b.len()];
            if segments_intersect_2d(a1, a2, b1, b2) {
                return true;
            }
        }
    }
    false
}

/// Test if two 2D line segments intersect (proper crossing).
pub fn segments_intersect_2d(a1: Point2, a2: Point2, b1: Point2, b2: Point2) -> bool {
    use remus_math::polygon2d::cross_2d;
    let d1 = cross_2d(b1, b2, a1);
    let d2 = cross_2d(b1, b2, a2);
    let d3 = cross_2d(a1, a2, b1);
    let d4 = cross_2d(a1, a2, b2);

    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

#[cfg(test)]
mod parsing_tests {
    use super::get_u32;

    #[test]
    fn get_u32_rejects_values_outside_the_handle_range() {
        let args = serde_json::json!({ "solid": u64::from(u32::MAX) + 1 });

        assert!(matches!(
            get_u32(&args, "solid"),
            Err(error) if error.message().contains("exceeds the u32 range")
        ));
    }
}

#[cfg(test)]
mod fillet_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::collections::HashSet;

    use remus_topology::Topology;
    use remus_topology::edge::EdgeId;
    use remus_topology::solid::SolidId;

    use super::try_fillet;

    fn solid_edge_ids(topo: &Topology, solid_id: SolidId) -> Vec<EdgeId> {
        let solid = topo.solid(solid_id).expect("solid");
        let shell = topo.shell(solid.outer_shell()).expect("shell");
        let mut seen = HashSet::new();
        let mut edges = Vec::new();
        for &fid in shell.faces() {
            let face = topo.face(fid).expect("face");
            let wire = topo.wire(face.outer_wire()).expect("wire");
            for oe in wire.edges() {
                if seen.insert(oe.edge().index()) {
                    edges.push(oe.edge());
                }
            }
        }
        edges
    }

    // A rejected fillet must be a typed error AND a true no-op on the input.
    // Every engine mutates the arena in place, so a partly-applied attempt
    // used to leave the INPUT solid rounded at some corners and
    // split-but-unclosed at others; and when failure was signalled by
    // returning the input handle, callers could not distinguish "radius too
    // large" from "unsupported topology" (the OpenZCAD plate misdiagnosis,
    // docs/qa 2026-08-01 in that repo).
    // A radius far too large for the box guarantees every engine fails.
    #[test]
    fn try_fillet_failure_leaves_the_input_untouched() {
        let mut topo = Topology::new();
        let cube = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edge_ids(&topo, cube);
        let before =
            remus_topology::explorer::solid_entity_counts(&topo, cube).expect("counts before");
        let vol_before = remus_operations::measure::solid_volume(&topo, cube, 0.01).unwrap();

        // r = 20 on a 10³ box: no engine can produce a valid solid.
        let error = try_fillet(&mut topo, cube, &edges, 20.0)
            .expect_err("an all-engine failure must be a typed error, not a silent no-op");
        assert!(
            !remus_operations::blend_ops::blend_failure_code(&error).is_empty(),
            "every failure maps to a machine-readable code"
        );

        let after =
            remus_topology::explorer::solid_entity_counts(&topo, cube).expect("counts after");
        assert_eq!(before, after, "failed fillet mutated the input topology");
        let vol_after = remus_operations::measure::solid_volume(&topo, cube, 0.01).unwrap();
        assert!(
            (vol_before - vol_after).abs() < 1e-9,
            "failed fillet changed the input volume: {vol_before} -> {vol_after}"
        );
        let shell = topo.shell(topo.solid(cube).unwrap().outer_shell()).unwrap();
        remus_topology::validation::validate_shell_closed(shell, &topo)
            .expect("input must still be watertight after a failed fillet");
    }

    // The OpenZCAD demo bracket: an L-blank (base plate + wall seated 0.5 mm
    // into it) with a boss, a bore, and two mount holes; the demo's Rev C
    // fillets the four vertical base-plate corners at r=3. The kernel used to
    // report "Fillet could not be created on 4 selected edges" — the
    // rolling-ball engine left an open shell and the walking engine failed to
    // trim, so `try_fillet` returned the input unchanged (the silent no-op).
    // The walking builder now handles it, so the consumer path must round the
    // corners: ≈60 mm³ removed, watertight, more faces than it started with.
    #[test]
    fn try_fillet_openzcad_bracket_corners() {
        use remus_math::mat::Mat4;
        use remus_operations::boolean::{BooleanOp, boolean};
        use remus_operations::primitives::{make_box, make_cylinder};
        use remus_operations::transform::transform_solid;

        let rot_x90_at = |tx: f64, ty: f64, tz: f64| {
            Mat4::translation(tx, ty, tz) * Mat4::rotation_x(std::f64::consts::FRAC_PI_2)
        };

        let mut topo = Topology::new();
        let base = make_box(&mut topo, 80.0, 40.0, 8.0).unwrap();
        let wall = make_box(&mut topo, 80.0, 8.0, 32.0).unwrap();
        transform_solid(&mut topo, wall, &Mat4::translation(0.0, 32.0, 7.5)).unwrap();
        let l_blank = boolean(&mut topo, BooleanOp::Fuse, base, wall).unwrap();

        let boss = make_cylinder(&mut topo, 10.0, 12.0).unwrap();
        transform_solid(&mut topo, boss, &rot_x90_at(40.0, 34.0, 24.0)).unwrap();
        let with_boss = boolean(&mut topo, BooleanOp::Fuse, l_blank, boss).unwrap();

        let bore = make_cylinder(&mut topo, 4.0, 48.0).unwrap();
        transform_solid(&mut topo, bore, &rot_x90_at(40.0, 48.0, 24.0)).unwrap();
        let bored = boolean(&mut topo, BooleanOp::Cut, with_boss, bore).unwrap();

        let mount_a = make_cylinder(&mut topo, 3.0, 12.0).unwrap();
        transform_solid(&mut topo, mount_a, &Mat4::translation(16.0, 20.0, -2.0)).unwrap();
        let cut_a = boolean(&mut topo, BooleanOp::Cut, bored, mount_a).unwrap();
        let mount_b = make_cylinder(&mut topo, 3.0, 12.0).unwrap();
        transform_solid(&mut topo, mount_b, &Mat4::translation(64.0, 20.0, -2.0)).unwrap();
        let bracket = boolean(&mut topo, BooleanOp::Cut, cut_a, mount_b).unwrap();

        // The four z-spanning corner edges of the base plate.
        let near = |a: f64, b: f64| (a - b).abs() < 0.1;
        let corners: Vec<EdgeId> = solid_edge_ids(&topo, bracket)
            .into_iter()
            .filter(|&eid| {
                let e = topo.edge(eid).expect("edge");
                let a = topo.vertex(e.start()).expect("v").point();
                let b = topo.vertex(e.end()).expect("v").point();
                let at_corner = |x: f64, y: f64| {
                    (near(x, 0.0) || near(x, 80.0)) && (near(y, 0.0) || near(y, 40.0))
                };
                at_corner(a.x(), a.y())
                    && at_corner(b.x(), b.y())
                    && (-0.1..=8.1).contains(&a.z())
                    && (-0.1..=8.1).contains(&b.z())
                    && (a.z() - b.z()).abs() >= 4.0
            })
            .collect();
        assert_eq!(corners.len(), 4, "expected 4 base-plate corner edges");

        let before_faces = topo
            .shell(topo.solid(bracket).unwrap().outer_shell())
            .unwrap()
            .faces()
            .len();
        let vol_before = remus_operations::measure::solid_volume(&topo, bracket, 0.1).unwrap();

        let result = try_fillet(&mut topo, bracket, &corners, 3.0).expect("bracket corner fillet");
        assert_ne!(result, bracket, "try_fillet returned the input unchanged");

        let vol_after = remus_operations::measure::solid_volume(&topo, result, 0.1).unwrap();
        let removed = vol_before - vol_after;
        assert!(
            (50.0..=70.0).contains(&removed),
            "expected ≈60 mm³ removed by the 4 corner fillets, got {removed:.3}"
        );

        let shell = topo
            .shell(topo.solid(result).unwrap().outer_shell())
            .unwrap();
        assert!(
            shell.faces().len() > before_faces,
            "fillet must add blend faces"
        );
        remus_topology::validation::validate_shell_closed(shell, &topo)
            .expect("filleted bracket must be watertight");
    }

    // The wasm `fillet` binding (and its batch sibling) route through
    // `try_fillet`. Filleting all 12 box edges must remove only the rounded
    // slivers (volume ≈ 975.6 for a 10³ box at r=1), not excise whole corner
    // octants. This guards the consumer path against regressing to the
    // over-removing walking engine.
    #[test]
    fn try_fillet_all_box_edges_no_corner_over_removal() {
        let mut topo = Topology::new();
        let cube = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edge_ids(&topo, cube);
        assert_eq!(edges.len(), 12, "box should have 12 edges");

        let result = try_fillet(&mut topo, cube, &edges, 1.0).expect("all-edges fillet");
        let vol = remus_operations::measure::solid_volume(&topo, result, 0.01).unwrap();
        assert!(
            vol > 970.0 && vol < 1000.0,
            "filleted box volume should be ≈975.6, got {vol}"
        );
    }

    // gh #967: filleting a plain cylinder's circular rim used to remove ~37% of
    // the volume (the rolling-ball engine collapses closed circular edges). The
    // rim now rounds into an exact quarter-torus: the rolling-ball degenerate
    // result is rejected, `try_fillet` falls through to the walking engine, and
    // the watertight rounded solid (≈6275.7, a ~0.12% rim round) is accepted —
    // never the corrupt ~3978.
    #[test]
    fn try_fillet_cylinder_rim_rounds_not_corrupts() {
        use remus_topology::face::FaceSurface;

        let mut topo = Topology::new();
        let cyl = remus_operations::primitives::make_cylinder(&mut topo, 10.0, 20.0).unwrap();
        let raw = remus_operations::measure::solid_volume(&topo, cyl, 0.01).unwrap();
        let edges = solid_edge_ids(&topo, cyl);

        let result = try_fillet(&mut topo, cyl, &edges, 0.5).expect("rim fillet");
        let vol = remus_operations::measure::solid_volume(&topo, result, 0.01).unwrap();

        // A tiny rim round — well under 1% removed, never the −37% corruption.
        assert!(
            vol < raw && vol > raw * 0.99,
            "cylinder rim fillet should round (~{:.0}), got {vol} vs raw {raw}",
            raw * 0.999
        );
        let sh = topo
            .shell(topo.solid(result).unwrap().outer_shell())
            .unwrap();
        let torus_count = sh
            .faces()
            .iter()
            .filter(|&&fid| matches!(topo.face(fid).unwrap().surface(), FaceSurface::Torus(_)))
            .count();
        assert_eq!(torus_count, 2, "both rims round into toroidal bands");
        assert!(
            remus_operations::validate::validate_solid(&topo, result)
                .unwrap()
                .is_valid(),
            "rounded rim solid must be watertight"
        );
    }

    #[test]
    fn try_fillet_second_pass_does_not_break_solid() {
        use remus_topology::face::FaceSurface;

        // #813: a second fillet whose target edge borders the first fillet's
        // NURBS blend face must not produce a self-intersecting solid — the
        // volume grew past the base to 1000.30 before the fix; such edges are
        // now skipped. Checked over *every* result edge so the guard doesn't
        // rely on a particular edge ordering.
        let mut topo = Topology::new();
        let cube = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edge_ids(&topo, cube);
        let first = try_fillet(&mut topo, cube, &[edges[0], edges[1]], 1.0).expect("first fillet");
        let v1 = remus_operations::measure::solid_volume(&topo, first, 0.05).unwrap();

        // The scenario under test only exists if the first fillet produced blend
        // faces for the later edges to border. It used to name NURBS
        // specifically: this corner is now built entirely from exact analytic
        // surfaces — two cylindrical bands, the ball octant, and the ledge — so
        // the precondition is "a curved blend face exists", which is the
        // property #813 is actually about.
        let sd = topo.solid(first).expect("solid");
        let sh = topo.shell(sd.outer_shell()).expect("shell");
        let has_blend = sh.faces().iter().any(|&fid| {
            topo.face(fid).is_ok_and(|f| {
                matches!(
                    f.surface(),
                    FaceSurface::Nurbs(_) | FaceSurface::Cylinder(_) | FaceSurface::Sphere(_)
                )
            })
        });
        assert!(has_blend, "first fillet should create blend faces");

        // Filleting any single result edge must stay a manifold solid and must
        // not self-intersect/inflate past the original box volume (the #813 bug
        // grew it to 1000.30). A blend-adjacent *concave* end-cap edge validly
        // *fills* (volume rises toward — but never beyond — the box), so the
        // guard is the box volume, not the pre-fillet volume.
        let _ = v1;
        let r_edges = solid_edge_ids(&topo, first);
        for &e in &r_edges {
            let mut t = topo.clone();
            // Tangent/degenerate selections (the blend's G1 contact lines)
            // are a typed refusal now, not a silent no-op; the refusal must
            // leave the input solid intact.
            let Ok(s) = try_fillet(&mut t, first, &[e], 0.5) else {
                let ssd = t.solid(first).expect("input solid");
                let ssh = t.shell(ssd.outer_shell()).expect("shell");
                remus_topology::validation::validate_shell_manifold(ssh, &t)
                    .expect("a refused second fillet must leave a manifold solid");
                continue;
            };
            let v2 = remus_operations::measure::solid_volume(&t, s, 0.05).unwrap();
            assert!(
                v2 <= 1000.0 + 0.1,
                "second fillet on edge {} inflated past the box: second={v2:.2}",
                e.index()
            );
            let ssd = t.solid(s).expect("result solid");
            let ssh = t.shell(ssd.outer_shell()).expect("shell");
            remus_topology::validation::validate_shell_manifold(ssh, &t)
                .expect("second fillet result must remain a manifold solid");
        }
    }

    #[test]
    fn try_fillet_blend_neighbor_is_watertight() {
        use std::collections::HashMap;

        use remus_topology::face::FaceSurface;
        use remus_topology::validation::{validate_shell_closed, validate_shell_manifold};

        // #834 via the consumer path: a single fillet creates a blend face
        // (an exact cylinder — a constant radius along a straight edge between
        // two planes is one); `try_fillet` on a non-tangent edge bordering it
        // must round that into a valid watertight manifold, not skip it.
        let mut topo = Topology::new();
        let cube = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edge_ids(&topo, cube);
        let first = try_fillet(&mut topo, cube, &[edges[0]], 1.0).expect("first fillet");
        {
            let sh = topo
                .shell(topo.solid(first).unwrap().outer_shell())
                .unwrap();
            validate_shell_closed(sh, &topo).expect("first fillet should be watertight");
        }

        // A box has no curved face of its own, so every cylinder here is the
        // blend.
        let blend: HashSet<usize> = {
            let sh = topo
                .shell(topo.solid(first).unwrap().outer_shell())
                .unwrap();
            sh.faces()
                .iter()
                .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
                .map(|f| f.index())
                .collect()
        };
        assert!(!blend.is_empty(), "first fillet must create a blend face");

        let mut ef: HashMap<usize, HashSet<usize>> = HashMap::new();
        {
            let sh = topo
                .shell(topo.solid(first).unwrap().outer_shell())
                .unwrap();
            for &fid in sh.faces() {
                for oe in topo
                    .wire(topo.face(fid).unwrap().outer_wire())
                    .unwrap()
                    .edges()
                {
                    ef.entry(oe.edge().index()).or_default().insert(fid.index());
                }
            }
        }

        let r_edges = solid_edge_ids(&topo, first);
        let filletable: HashSet<usize> =
            remus_operations::query::filter_filletable_edges(&topo, first, &r_edges)
                .unwrap()
                .iter()
                .map(|e| e.index())
                .collect();
        let target = r_edges
            .iter()
            .copied()
            .find(|e| {
                filletable.contains(&e.index())
                    && ef
                        .get(&e.index())
                        .is_some_and(|fs| fs.iter().any(|f| blend.contains(f)))
            })
            .expect("a filletable edge bordering the blend face");

        let result = try_fillet(&mut topo, first, &[target], 0.5).expect("second fillet");
        assert_ne!(
            result, first,
            "the blend-adjacent edge should be filleted, not skipped"
        );
        let sh = topo
            .shell(topo.solid(result).unwrap().outer_shell())
            .unwrap();
        validate_shell_manifold(sh, &topo).expect("second fillet must be manifold");
        validate_shell_closed(sh, &topo)
            .expect("second fillet on a blend-adjacent edge must be watertight");
    }

    // The OpenZCAD plate (80 x 60 x 6 with a bored hole). Corner chains and
    // hole rims used to fail in every engine; they now succeed, so this pins
    // both halves of that: the cases that work, and — for one that still
    // cannot work — that the caller receives a typed diagnosis rather than a
    // silent no-op, with the input rolled back untouched.
    #[test]
    fn try_fillet_failure_reports_the_walking_engine_diagnosis() {
        use remus_math::mat::Mat4;
        use remus_operations::blend_ops::{BlendError, blend_failure_code};
        use remus_operations::boolean::{BooleanOp, boolean};
        use remus_operations::primitives::{make_box, make_cylinder};
        use remus_operations::transform::transform_solid;

        let mut topo = Topology::new();
        let plate = make_box(&mut topo, 80.0, 60.0, 6.0).unwrap();
        let hole = make_cylinder(&mut topo, 2.25, 6.0).unwrap();
        transform_solid(&mut topo, hole, &Mat4::translation(10.0, 10.0, 0.0)).unwrap();
        let solid = boolean(&mut topo, BooleanOp::Cut, plate, hole).unwrap();

        let on_top = |topo: &Topology, eid: EdgeId| {
            let e = topo.edge(eid).unwrap();
            (topo.vertex(e.start()).unwrap().point().z() - 6.0).abs() < 1e-9
                && (topo.vertex(e.end()).unwrap().point().z() - 6.0).abs() < 1e-9
        };
        let edge_len = |topo: &Topology, eid: EdgeId| {
            let e = topo.edge(eid).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            (a - b).length()
        };
        let edges = solid_edge_ids(&topo, solid);
        let long = *edges
            .iter()
            .find(|&&e| on_top(&topo, e) && (edge_len(&topo, e) - 80.0).abs() < 1e-6)
            .unwrap();
        let short = *edges
            .iter()
            .find(|&&e| on_top(&topo, e) && (edge_len(&topo, e) - 60.0).abs() < 1e-6)
            .unwrap();
        let rim = *edges
            .iter()
            .find(|&&e| {
                on_top(&topo, e)
                    && matches!(
                        topo.edge(e).unwrap().curve(),
                        remus_topology::edge::EdgeCurve::Circle(_)
                    )
            })
            .unwrap();

        let before =
            remus_topology::explorer::solid_entity_counts(&topo, solid).expect("counts before");

        // A radius half again the plate's own thickness cannot be seated: the
        // contact would hang below the bottom face. This is the case that must
        // still refuse, and it is the one that carries the typed-diagnosis
        // contract this test exists for.
        let too_large = try_fillet(&mut topo, solid, &[long], 9.0)
            .expect_err("a radius larger than the plate is thick must fail typed");
        assert!(
            matches!(
                too_large,
                remus_operations::OperationsError::Blend(BlendError::RadiusTooLarge { .. })
            ),
            "expected RadiusTooLarge, got: {too_large}"
        );
        assert_eq!(blend_failure_code(&too_large), "radius-too-large");

        let after =
            remus_topology::explorer::solid_entity_counts(&topo, solid).expect("counts after");
        assert_eq!(before, after, "a failed fillet mutated the input topology");

        // Both of these used to be the failures this test asserted. They are
        // the point of the blend work, so pin them as successes at the wasm
        // boundary rather than dropping the coverage.
        let corner = try_fillet(&mut topo, solid, &[long, short], 2.0)
            .expect("a corner chain on a drilled plate now blends");
        let corner_shell = topo
            .shell(topo.solid(corner).expect("corner solid").outer_shell())
            .expect("corner shell");
        remus_topology::validation::validate_shell_closed(corner_shell, &topo)
            .expect("the corner-chain result must be watertight");

        let rim_solid = try_fillet(&mut topo, solid, &[rim], 1.0)
            .expect("a hole rim on a drilled plate now blends");
        let rim_shell = topo
            .shell(topo.solid(rim_solid).expect("rim solid").outer_shell())
            .expect("rim shell");
        remus_topology::validation::validate_shell_closed(rim_shell, &topo)
            .expect("the hole-rim result must be watertight");
    }

    // ── The blend must cover every edge named ──
    //
    // `fillet` used to retry on the PLANAR SUBSET of a failing selection and
    // return that subset's solid as the answer. A plate's top perimeter picked
    // together with its bore rim therefore came back byte-identical to the
    // perimeter-only result: fresh valid handle, volume inside the plausible
    // envelope, no error, no warning — and the rim, which rounds perfectly when
    // picked alone, simply missing. Nothing the caller could inspect said so.

    const PLATE_W: f64 = 80.0;
    const PLATE_D: f64 = 60.0;
    const PLATE_T: f64 = 20.0;
    const BORE_R: f64 = 8.0;

    /// A plate with one bore straight through, its four top perimeter edges,
    /// and its top bore rim.
    fn bored_plate(
        topo: &mut Topology,
    ) -> (
        remus_topology::solid::SolidId,
        Vec<remus_topology::edge::EdgeId>,
        remus_topology::edge::EdgeId,
    ) {
        use remus_math::mat::Mat4;
        use remus_operations::boolean::{BooleanOp, boolean};
        use remus_operations::primitives::{make_box, make_cylinder};
        use remus_operations::transform::transform_solid;
        use remus_topology::edge::EdgeCurve;

        let blank = make_box(topo, PLATE_W, PLATE_D, PLATE_T).unwrap();
        let drill = make_cylinder(topo, BORE_R, PLATE_T + 4.0).unwrap();
        transform_solid(
            topo,
            drill,
            &Mat4::translation(PLATE_W / 2.0, PLATE_D / 2.0, -2.0),
        )
        .unwrap();
        let body = boolean(topo, BooleanOp::Cut, blank, drill).unwrap();

        let mut edges: Vec<remus_topology::edge::EdgeId> = Vec::new();
        for fid in remus_topology::explorer::solid_faces(topo, body).unwrap() {
            let f = topo.face(fid).unwrap();
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in topo.wire(wid).unwrap().edges() {
                    if !edges.contains(&oe.edge()) {
                        edges.push(oe.edge());
                    }
                }
            }
        }
        let mut perimeter = Vec::new();
        let mut rim = None;
        for e in edges {
            let ed = topo.edge(e).unwrap();
            if ed.start() == ed.end() {
                if let EdgeCurve::Circle(c) = ed.curve()
                    && (c.center().z() - PLATE_T).abs() < 1e-9
                {
                    rim = Some(e);
                }
                continue;
            }
            let a = topo.vertex(ed.start()).unwrap().point();
            let b = topo.vertex(ed.end()).unwrap().point();
            if (a.z() - PLATE_T).abs() < 1e-9 && (b.z() - PLATE_T).abs() < 1e-9 {
                perimeter.push(e);
            }
        }
        assert_eq!(perimeter.len(), 4, "the top face has four perimeter edges");
        (body, perimeter, rim.expect("top bore rim"))
    }

    #[test]
    fn fillet_whole_selection_never_returns_the_planar_subset() {
        let radius = 1.0;

        // Both halves of the selection round on their own, so neither is at
        // fault: only the combination was.
        let mut topo = Topology::new();
        let (body, perimeter, rim) = bored_plate(&mut topo);
        let blank = remus_operations::measure::solid_volume(&topo, body, 0.001).unwrap();

        let perimeter_only = {
            let mut t = topo.clone();
            let s = super::fillet_whole_selection(&mut t, body, &perimeter, radius)
                .expect("the perimeter alone must round");
            remus_operations::measure::solid_volume(&t, s, 0.001).unwrap()
        };
        assert!(
            perimeter_only < blank,
            "the perimeter fillet must remove material"
        );
        let rim_only = {
            let mut t = topo.clone();
            let s = super::fillet_whole_selection(&mut t, body, &[rim], radius)
                .expect("the bore rim alone must round");
            remus_operations::measure::solid_volume(&t, s, 0.001).unwrap()
        };
        assert!(rim_only < blank, "the rim fillet must remove material");

        let mut mixed = perimeter;
        mixed.push(rim);
        match super::fillet_whole_selection(&mut topo, body, &mixed, radius) {
            Ok(s) => {
                // Blending everything is the ideal answer; blending only the
                // perimeter and calling it done is the defect.
                let volume = remus_operations::measure::solid_volume(&topo, s, 0.001).unwrap();
                let both = blank - (blank - perimeter_only) - (blank - rim_only);
                assert!(
                    (volume - both).abs() < 0.02 * (blank - both),
                    "the mixed selection returned {volume}; the perimeter alone \
                     gives {perimeter_only} and blending both gives {both}. A \
                     figure matching the perimeter means the rim was dropped."
                );
            }
            Err(e) => {
                assert_eq!(
                    remus_operations::blend_ops::blend_failure_code(&e),
                    "edges-not-blended",
                    "the refusal must say the selection was not fully blended: {e}"
                );
                let msg = format!("{e}");
                assert!(
                    msg.contains(&format!("{rim:?}")),
                    "the refusal must name the edge it could not blend, got: {msg}"
                );
                let after = remus_operations::measure::solid_volume(&topo, body, 0.001).unwrap();
                assert!(
                    (after - blank).abs() < 1e-9,
                    "a refused fillet must leave the input untouched ({blank} -> {after})"
                );
            }
        }
    }
}

/// Pack a `[curvature, tangent…, principal normal…]` result from a curve's
/// first and second derivatives.
///
/// The principal normal is the component of `d2` orthogonal to the unit
/// tangent, renormalized — the true Frenet normal, not the direction to a
/// centre point (which coincides with it only for a circle). Degenerate
/// cases (zero speed, or `d2` parallel to the tangent, i.e. an inflection)
/// fall back to a fixed axis, matching the other arms' convention.
#[must_use]
pub fn frenet_from_derivatives(curvature: f64, d1: Vec3, d2: Vec3) -> Vec<f64> {
    let tangent = d1.normalize().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let normal = (d2 - tangent * d2.dot(tangent))
        .normalize()
        .unwrap_or(Vec3::new(0.0, 1.0, 0.0));
    vec![
        curvature,
        tangent.x(),
        tangent.y(),
        tangent.z(),
        normal.x(),
        normal.y(),
        normal.z(),
    ]
}
