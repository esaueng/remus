//! IGES file reader.
//!
//! Parses IGES files and extracts geometric entities (lines, planes,
//! NURBS curves and surfaces) into topology.

use std::collections::HashMap;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use remus_topology::transaction::{AppendPath, run_append_only};

use crate::IoError;
use crate::limits::{ImportLimits, ensure_input_size, ensure_limit};

/// Read an IGES file and reconstruct topology.
///
/// Returns the list of solid IDs created. Each group of faces found
/// in the IGES file is assembled into a solid.
///
/// # Errors
///
/// Returns [`IoError`] if the file is malformed or contains unsupported entities.
pub fn read_iges(input: &str, topo: &mut Topology) -> Result<Vec<SolidId>, IoError> {
    read_iges_with_limits(input, topo, ImportLimits::default())
}

/// Read an IGES file with explicit hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError`] when a limit is exceeded or the IGES data is invalid.
pub fn read_iges_with_limits(
    input: &str,
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<Vec<SolidId>, IoError> {
    read_iges_impl_with_path(input, topo, limits).map(|(solids, _)| solids)
}

/// Transactional IGES construction with its storage path.
///
/// Parsing runs before any topology mutation and is shared across retries.
/// The generated-entity bound is checked before allocation with checked
/// arithmetic, so hostile counts fail honestly without expensive work.
/// Construction (`build_topology`: vertices, edges, wires, faces, shells,
/// solids, plus derived loops/coedges via `add_face`) runs inside a guarded
/// append-only scope (PERF-I02 over PERF-T03):
///
/// - Every write targets newly allocated entities — vertices, edges, wires,
///   faces, shells, solids, derived loops/coedges for new faces — never
///   pre-existing document state, so the qualified import commits on the
///   append-only path. Any pre-existing write would trip the guard and retry
///   once under the full transaction with an identical result; only the cost
///   differs.
/// - The importer writes no attributes, pcurves, or journal entries; those
///   stores are preserved by the transaction on both success and failure.
/// - The operation closure is re-runnable: the parsed `entities` slice is
///   shared, and intermediate `face_ids` handles are recreated inside the
///   closure on every try, so a guard-trip retry never emits duplicates nor
///   leaks handles from the abandoned try (those handles stay stale via
///   high-water preservation).
fn read_iges_impl_with_path(
    input: &str,
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<(Vec<SolidId>, AppendPath), IoError> {
    ensure_input_size(input.len(), limits)?;
    // IGES uses fixed-width ASCII records. Rejecting non-ASCII input before
    // fixed-column parsing keeps every subsequent byte offset on a UTF-8
    // character boundary and turns malformed input into a typed error.
    if !input.is_ascii() {
        return Err(IoError::ParseError {
            reason: "IGES input must contain only ASCII fixed-width records".to_string(),
        });
    }
    let entities = parse_iges_entities(input, limits)?;
    // Bound generated arena slots before expensive allocation: each type-108
    // plane materializes at most 4 vertices + 4 edges + 1 wire + 1 face +
    // 1 loop + 4 coedges (15 slots), plus one shell and one solid per import.
    // Checked arithmetic keeps hostile counts honest; overflow is a limit
    // refusal, never a wrap.
    let plane_count = entities.iter().filter(|e| e.entity_type == 108).count();
    let required_slots = if plane_count == 0 {
        0
    } else {
        plane_count
            .checked_mul(15)
            .and_then(|v| v.checked_add(2))
            .ok_or(IoError::LimitExceeded {
                resource: "IGES generated entities",
                limit: limits.max_model_entities,
                actual: usize::MAX,
            })?
    };
    ensure_limit(
        "IGES generated entities",
        required_slots,
        limits.max_model_entities,
    )?;
    // Building an IGES model allocates topology incrementally. Keep the
    // import transactional so an error in a later plane cannot expose
    // geometry from an otherwise rejected file to the caller. Mutation-local
    // rollback (PERF-T02) records only touched state; the append-only guard
    // (PERF-T03) proves the import wrote only new content and falls back to
    // the full path with an identical result if it ever trips.
    run_append_only(topo, |topo| build_topology(topo, &entities))
}

// ── IGES entity representation ──────────────────────────────────────

/// A parsed IGES entity with its directory entry and parameter data.
#[derive(Debug)]
struct IgesEntity {
    /// Entity type number (e.g., 110 for line, 128 for NURBS surface).
    entity_type: u32,
    /// The raw parameter data string (comma-separated values).
    params: String,
    /// Directory entry sequence number (used for cross-referencing).
    #[allow(dead_code)]
    de_seq: u32,
}

// ── Parsing ─────────────────────────────────────────────────────────

/// Parse all entities from an IGES file.
fn parse_iges_entities(input: &str, limits: ImportLimits) -> Result<Vec<IgesEntity>, IoError> {
    let mut d_lines: Vec<&str> = Vec::new();
    let mut p_lines: Vec<&str> = Vec::new();

    for line in input.lines() {
        if line.len() < 73 {
            // Truncated fixed-width records carry no parseable section tag;
            // skipping keeps short-line input from panicking on column
            // indexing. Such lines contribute no entities, so a truncated
            // file imports as fewer bodies (possibly zero) rather than
            // failing mid-construction with partial topology.
            continue;
        }
        let section = line.as_bytes().get(72).copied().unwrap_or(b' ');
        match section {
            b'D' => d_lines.push(line),
            b'P' => p_lines.push(line),
            _ => {} // Skip S, G, T sections.
        }
        // Checked arithmetic keeps hostile record counts honest: overflow is
        // a limit refusal, never a wrap into a small accepted count.
        let record_count =
            d_lines
                .len()
                .checked_add(p_lines.len())
                .ok_or(IoError::LimitExceeded {
                    resource: "IGES records",
                    limit: limits.max_model_entities,
                    actual: usize::MAX,
                })?;
        let record_limit =
            limits
                .max_model_entities
                .checked_mul(3)
                .ok_or(IoError::LimitExceeded {
                    resource: "IGES records",
                    limit: limits.max_model_entities,
                    actual: usize::MAX,
                })?;
        ensure_limit("IGES records", record_count, record_limit)?;
    }

    // Parse directory entries (pairs of lines).
    let mut dir_entries: Vec<(u32, u32, u32)> = Vec::new(); // (entity_type, pd_start, de_seq)

    let mut i = 0;
    while i + 1 < d_lines.len() {
        let line1 = d_lines[i];

        let entity_type = parse_int_field(line1, 0, 8)?;
        let pd_start = parse_int_field(line1, 8, 16)?;

        // DE sequence number is in columns 73-80.
        let de_seq = parse_int_field(line1, 73, 80)?;

        dir_entries.push((entity_type, pd_start, de_seq));
        ensure_limit(
            "IGES entities",
            dir_entries.len(),
            limits.max_model_entities,
        )?;
        i += 2; // Skip the second line of the DE pair.
    }

    // Collect parameter data by DE pointer.
    // P-section lines have format: data (cols 0-63), DE pointer (cols 64-72), "P" + seq.
    let mut pd_by_de: HashMap<u32, String> = HashMap::new();

    for p_line in &p_lines {
        let data_part = if p_line.len() >= 64 {
            &p_line[..64]
        } else {
            p_line
        };
        let de_ptr = if p_line.len() >= 72 {
            parse_int_field(p_line, 64, 72).unwrap_or(0)
        } else {
            0
        };

        pd_by_de
            .entry(de_ptr)
            .or_default()
            .push_str(data_part.trim_end());
    }

    let mut entities = Vec::new();
    for (entity_type, _pd_start, de_seq) in &dir_entries {
        let params = pd_by_de.get(de_seq).cloned().unwrap_or_default();
        // Strip the entity type number prefix from params (e.g., "110,..." → "...").
        let clean_params = strip_entity_prefix(&params, *entity_type);

        entities.push(IgesEntity {
            entity_type: *entity_type,
            params: clean_params,
            de_seq: *de_seq,
        });
    }

    Ok(entities)
}

/// Parse an integer from a fixed-width field in an IGES line.
fn parse_int_field(line: &str, start: usize, end: usize) -> Result<u32, IoError> {
    let end = end.min(line.len());
    if start >= end {
        return Ok(0);
    }
    let field = line[start..end].trim();
    if field.is_empty() {
        return Ok(0);
    }
    field.parse::<u32>().map_err(|e| IoError::ParseError {
        reason: format!("invalid IGES integer field '{field}': {e}"),
    })
}

/// Strip the entity type prefix from parameter data.
/// E.g., "110,1.0,2.0,..." → "1.0,2.0,..."
fn strip_entity_prefix(params: &str, entity_type: u32) -> String {
    let prefix = format!("{entity_type},");
    params.strip_prefix(&prefix).unwrap_or(params).to_string()
}

// ── Topology building ───────────────────────────────────────────────

/// Build topology from parsed IGES entities.
///
/// Every type-108 plane is constructed fallibly: a malformed plane is a hard
/// [`IoError`], not a silent skip, so a later bad entity after earlier good
/// allocations fails the whole import and the surrounding append-only scope
/// rewinds to the pre-import state. Entity types 110 (line), 126 (NURBS
/// curve), 128 (NURBS surface) remain skipped — they would be referenced by
/// higher-level entities, and broadening geometric support is out of scope.
fn build_topology(topo: &mut Topology, entities: &[IgesEntity]) -> Result<Vec<SolidId>, IoError> {
    // Restartable: recreated inside the append-only closure on every try.
    let mut face_ids = Vec::new();

    for entity in entities {
        if entity.entity_type == 108 {
            let face_id = build_plane_face(topo, &entity.params)?;
            face_ids.push(face_id);
        }
        // Entity types 110 (line), 126 (NURBS curve), 128 (NURBS surface)
        // are skipped — they would be referenced by higher-level entities.
    }

    if face_ids.is_empty() {
        return Ok(Vec::new());
    }

    let shell = Shell::new(face_ids).map_err(|e| IoError::ParseError {
        reason: format!("failed to build shell: {e}"),
    })?;
    let shell_id = topo.add_shell(shell);
    let solid_id = topo.add_solid(Solid::new(shell_id, Vec::new()));

    Ok(vec![solid_id])
}

/// Build a planar face from IGES entity type 108 parameters.
/// Format: A, B, C, D, ptr, x, y, z (plane Ax+By+Cz=D).
fn build_plane_face(
    topo: &mut Topology,
    params: &str,
) -> Result<remus_topology::face::FaceId, IoError> {
    let values = parse_float_params(params)?;
    if values.len() < 4 {
        return Err(IoError::ParseError {
            reason: format!("IGES plane entity needs 4 params, got {}", values.len()),
        });
    }

    // Strict finite checks before any allocation: nonfinite plane data must
    // refuse with a typed error, never materialize corrupt geometry.
    for (idx, value) in values.iter().take(4).enumerate() {
        if !value.is_finite() {
            return Err(IoError::ParseError {
                reason: format!("IGES plane param {idx} is nonfinite: {value}"),
            });
        }
    }

    let normal = Vec3::new(values[0], values[1], values[2]);
    let d = values[3];

    let norm_len = normal.length();
    if !norm_len.is_finite() || norm_len < 1e-10 {
        return Err(IoError::ParseError {
            reason: "IGES plane has zero or nonfinite normal".to_string(),
        });
    }
    if !d.is_finite() {
        return Err(IoError::ParseError {
            reason: "IGES plane offset D is nonfinite".to_string(),
        });
    }

    // Create a small square face on this plane for visualization.
    let unit_normal = Vec3::new(
        normal.x() / norm_len,
        normal.y() / norm_len,
        normal.z() / norm_len,
    );
    if !unit_normal.x().is_finite() || !unit_normal.y().is_finite() || !unit_normal.z().is_finite()
    {
        return Err(IoError::ParseError {
            reason: "IGES plane unit normal is nonfinite".to_string(),
        });
    }

    let origin = Point3::new(
        unit_normal.x() * d / norm_len,
        unit_normal.y() * d / norm_len,
        unit_normal.z() * d / norm_len,
    );
    if !origin.x().is_finite() || !origin.y().is_finite() || !origin.z().is_finite() {
        return Err(IoError::ParseError {
            reason: "IGES plane origin is nonfinite".to_string(),
        });
    }

    let ax = Vec3::new(1.0, 0.0, 0.0);
    let ay = Vec3::new(0.0, 1.0, 0.0);
    let candidate = if unit_normal.dot(ax).abs() < 0.9 {
        ax
    } else {
        ay
    };
    let u_dir = unit_normal.cross(candidate);
    let u_len = u_dir.length().max(1e-10);
    if !u_len.is_finite() {
        return Err(IoError::ParseError {
            reason: "IGES plane tangent frame is nonfinite".to_string(),
        });
    }
    let u_dir = Vec3::new(u_dir.x() / u_len, u_dir.y() / u_len, u_dir.z() / u_len);
    let v_dir = unit_normal.cross(u_dir);
    if !u_dir.x().is_finite()
        || !u_dir.y().is_finite()
        || !u_dir.z().is_finite()
        || !v_dir.x().is_finite()
        || !v_dir.y().is_finite()
        || !v_dir.z().is_finite()
    {
        return Err(IoError::ParseError {
            reason: "IGES plane tangent directions are nonfinite".to_string(),
        });
    }

    let half = 0.5;
    let p0 = offset_point(origin, u_dir, -half, v_dir, -half);
    let p1 = offset_point(origin, u_dir, half, v_dir, -half);
    let p2 = offset_point(origin, u_dir, half, v_dir, half);
    let p3 = offset_point(origin, u_dir, -half, v_dir, half);
    for (idx, point) in [p0, p1, p2, p3].iter().enumerate() {
        if !point.x().is_finite() || !point.y().is_finite() || !point.z().is_finite() {
            return Err(IoError::ParseError {
                reason: format!("IGES plane corner {idx} is nonfinite"),
            });
        }
    }

    let v0 = topo.add_vertex(Vertex::new(p0, 1e-7));
    let v1 = topo.add_vertex(Vertex::new(p1, 1e-7));
    let v2 = topo.add_vertex(Vertex::new(p2, 1e-7));
    let v3 = topo.add_vertex(Vertex::new(p3, 1e-7));

    let e01 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
    let e12 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
    let e23 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
    let e30 = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));

    let wire = Wire::new(
        vec![
            OrientedEdge::new(e01, true),
            OrientedEdge::new(e12, true),
            OrientedEdge::new(e23, true),
            OrientedEdge::new(e30, true),
        ],
        true,
    )
    .map_err(|e| IoError::ParseError {
        reason: format!("failed to build wire: {e}"),
    })?;
    let wire_id = topo.add_wire(wire);

    let surface = FaceSurface::Plane {
        normal: unit_normal,
        d: d / norm_len,
    };
    let face_id = topo.add_face(Face::new(wire_id, Vec::new(), surface));

    Ok(face_id)
}

/// Compute `origin + a*u + b*v` as a `Point3`.
fn offset_point(origin: Point3, u: Vec3, a: f64, v: Vec3, b: f64) -> Point3 {
    Point3::new(
        u.x().mul_add(a, v.x().mul_add(b, origin.x())),
        u.y().mul_add(a, v.y().mul_add(b, origin.y())),
        u.z().mul_add(a, v.z().mul_add(b, origin.z())),
    )
}

/// Parse comma-separated float parameters from IGES parameter data.
///
/// Every non-empty token must parse as a finite `f64`: malformed or
/// nonfinite tokens are a typed [`IoError::ParseError`], never silently
/// dropped. Empty tokens (from `,,`) are skipped, so a short list still
/// fails at the caller's arity check with an honest count.
fn parse_float_params(params: &str) -> Result<Vec<f64>, IoError> {
    let clean = params.trim_end_matches(';');
    let mut out = Vec::new();
    for token in clean.split(',') {
        let trimmed = token.trim();
        if trimmed.is_empty() {
            continue;
        }
        let value: f64 = trimmed.parse().map_err(|e| IoError::ParseError {
            reason: format!("invalid IGES float param '{trimmed}': {e}"),
        })?;
        if !value.is_finite() {
            return Err(IoError::ParseError {
                reason: format!("IGES float param '{trimmed}' is nonfinite"),
            });
        }
        out.push(value);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_cube_non_manifold;

    use super::*;
    use crate::iges::writer;

    #[test]
    fn roundtrip_unit_cube() {
        let mut write_topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut write_topo);

        let iges_str = writer::write_iges(&write_topo, &[solid]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_iges(&iges_str, &mut read_topo).unwrap();

        assert_eq!(solids.len(), 1);
        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();
        // Unit cube has 6 plane entities.
        assert_eq!(shell.faces().len(), 6);
    }

    #[test]
    fn roundtrip_box_primitive() {
        let mut write_topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut write_topo, 2.0, 3.0, 4.0).unwrap();

        let iges_str = writer::write_iges(&write_topo, &[solid]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_iges(&iges_str, &mut read_topo).unwrap();

        assert_eq!(solids.len(), 1);
    }

    #[test]
    fn empty_file_returns_empty() {
        let mut topo = Topology::new();
        let solids = read_iges("", &mut topo).unwrap();
        assert!(solids.is_empty());
    }

    #[test]
    fn parse_float_params_basic() {
        let floats = parse_float_params("1.0,2.5,-3.0,0.;").unwrap();
        assert_eq!(floats.len(), 4);
        assert!((floats[0] - 1.0).abs() < 1e-10);
        assert!((floats[1] - 2.5).abs() < 1e-10);
        assert!((floats[2] - (-3.0)).abs() < 1e-10);
        assert!((floats[3]).abs() < 1e-10);
    }

    #[test]
    fn parse_int_field_basic() {
        let val = parse_int_field("     108       1", 0, 8).unwrap();
        assert_eq!(val, 108);
    }

    #[test]
    fn non_ascii_fixed_width_input_returns_parse_error() {
        let mut line = " ".repeat(80);
        line.replace_range(63..65, "é");
        let mut topo = Topology::new();

        let result = read_iges(&line, &mut topo);

        assert!(matches!(result, Err(IoError::ParseError { .. })));
        assert!(topo.vertices().is_empty());
    }

    /// Corrupt the last type-108 plane's P data to a zero normal, so the file
    /// parses but fails during construction after earlier planes allocated.
    fn malform_last_plane_to_zero_normal(valid: &str) -> String {
        let mut lines: Vec<String> = valid.lines().map(str::to_owned).collect();
        let mut last_idx = None;
        for (i, line) in lines.iter().enumerate() {
            if line.len() >= 73
                && line.as_bytes().get(72).copied().unwrap_or(b' ') == b'P'
                && line[..64.min(line.len())].contains("108,")
            {
                last_idx = Some(i);
            }
        }
        let idx = last_idx.expect("valid IGES must contain a 108 plane");
        let suffix = lines[idx][64..].to_owned();
        let zero_plane = format!("{:<64}", "108,0.,0.,0.,0.,0,0,0,0;");
        lines[idx] = format!("{zero_plane}{suffix}");
        lines.join("\n")
    }

    #[test]
    fn iges_import_takes_the_append_only_path() {
        use remus_operations::primitives::make_box;
        use remus_topology::transaction::AppendPath;

        // Pre-existing document the import must not copy.
        let mut topo = Topology::new();
        for i in 0..10 {
            make_box(&mut topo, 1.0 + i as f64 * 0.01, 1.0, 1.0).unwrap();
        }
        let solids_before = topo.num_solids();
        let faces_before = topo.num_faces();
        let slots_before = topo.allocated_slot_count();

        let mut write_topo = Topology::new();
        let solid = make_box(&mut write_topo, 2.0, 3.0, 4.0).unwrap();
        let iges = writer::write_iges(&write_topo, &[solid]).unwrap();

        let (result, path) =
            read_iges_impl_with_path(&iges, &mut topo, ImportLimits::default()).unwrap();
        assert_eq!(path, AppendPath::AppendOnly);
        assert_eq!(result.len(), 1);
        assert_eq!(topo.num_solids(), solids_before + 1);
        // Six planes → six 1×1 preview faces; pre-existing faces untouched.
        let shell = topo.solid(result[0]).unwrap().outer_shell();
        assert_eq!(topo.shell(shell).unwrap().faces().len(), 6);
        assert_eq!(topo.num_faces(), faces_before + 6);
        // Exact slot growth: 6 planes × 15 slots + 1 shell + 1 solid.
        assert_eq!(topo.allocated_slot_count(), slots_before + 6 * 15 + 2);
    }

    #[test]
    fn iges_failed_import_retires_without_reusing_handles() {
        use remus_operations::primitives::make_box;

        let mut topo = Topology::new();
        let kept = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let mut write_topo = Topology::new();
        let solid = make_box(&mut write_topo, 2.0, 3.0, 4.0).unwrap();
        let valid = writer::write_iges(&write_topo, &[solid]).unwrap();
        let bad = malform_last_plane_to_zero_normal(&valid);

        let slots_before = topo.allocated_slot_count();
        let err = read_iges(&bad, &mut topo).unwrap_err();
        assert!(
            matches!(err, IoError::ParseError { .. }),
            "unexpected {err:?}"
        );
        assert!(topo.solid(kept).is_ok());
        assert_eq!(topo.num_solids(), 1);
        assert!(topo.allocated_slot_count() >= slots_before);
        // The abandoned body's slot stays stale; the next build does not reuse it.
        if let Some(abandoned) = topo.solid_id_from_index(1) {
            assert!(topo.solid(abandoned).is_err());
        }
        let fresh = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
        assert_ne!(fresh, kept);
        assert!(topo.solid(fresh).is_ok());
    }

    #[test]
    fn parse_float_params_rejects_malformed_and_nonfinite() {
        assert!(matches!(
            parse_float_params("1.0,abc,3.0,4.0;"),
            Err(IoError::ParseError { .. })
        ));
        assert!(matches!(
            parse_float_params("1.0,inf,3.0,4.0;"),
            Err(IoError::ParseError { .. })
        ));
        assert!(matches!(
            parse_float_params("1.0,NaN,3.0,4.0;"),
            Err(IoError::ParseError { .. })
        ));
        assert!(matches!(
            parse_float_params("1.0,1e999,3.0,4.0;"),
            Err(IoError::ParseError { .. })
        ));
    }

    #[test]
    fn generated_entity_bound_fails_before_allocation() {
        use remus_operations::primitives::make_box;

        let mut write_topo = Topology::new();
        let solid = make_box(&mut write_topo, 2.0, 3.0, 4.0).unwrap();
        let iges = writer::write_iges(&write_topo, &[solid]).unwrap();

        // Six planes need 92 slots; a budget of 10 must refuse without growth.
        let limits = ImportLimits {
            max_model_entities: 10,
            ..ImportLimits::default()
        };
        let mut topo = Topology::new();
        let slots_before = topo.allocated_slot_count();
        let err = read_iges_with_limits(&iges, &mut topo, limits).unwrap_err();
        assert!(
            matches!(err, IoError::LimitExceeded { .. }),
            "unexpected {err:?}"
        );
        assert_eq!(topo.allocated_slot_count(), slots_before);
        assert_eq!(topo.num_solids(), 0);

        // Exact boundary succeeds: 92 slots.
        let exact = ImportLimits {
            max_model_entities: 92,
            ..ImportLimits::default()
        };
        let mut topo2 = Topology::new();
        let solids = read_iges_with_limits(&iges, &mut topo2, exact).unwrap();
        assert_eq!(solids.len(), 1);
    }
}
