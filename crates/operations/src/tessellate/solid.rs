//! Solid-level tessellation orchestration.

use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;
use remus_topology::{BodyClass, BodyId};

use super::TriangleMesh;
#[cfg(not(target_arch = "wasm32"))]
use super::boundary_plan::SampledEdge;
use super::boundary_plan::{BoundaryPlan, BoundaryRefinementRequest, SampleChains, SampleParams};
use super::edge_sampling::{
    circle_param_range, sample_edge, sample_edge_with_params, segments_for_chord_deviation_a,
};
use super::face_cache;
use super::mesh_ops::{
    dedupe_coincident_triangles, fill_sub_deflection_triangular_gaps, weld_boundary_vertices,
};
use super::nonplanar::{
    tessellate_cone_apex_fan_shared, tessellate_converted_wall_band_shared,
    tessellate_latitude_band_shared, tessellate_nonplanar_cdt, tessellate_nonplanar_snap,
    tessellate_nurbs_blend_band_shared, tessellate_nurbs_pole_cap_shared,
    tessellate_revolution_band_shared, tessellate_sphere_cap_shared, tessellate_torus_notch_band,
    tessellate_torus_two_rim_band,
};
use super::nurbs::{compute_angular_range, compute_v_param_range};
use super::planar::{
    cdt_triangulate_simple, project_by_normal, remove_closing_duplicate_global,
    remove_closing_duplicate_ids, run_planar_cdt, tessellate_planar_shared_with_holes,
    unproject_point,
};
use super::{MERGE_GRID, point_merge_key};

const MAX_PLANAR_CONTACT_CANDIDATE_PAIRS: usize = 4_000_000;

fn add_planar_contact_work(
    total: &mut usize,
    line_count: usize,
    sample_count: usize,
) -> Result<(), crate::OperationsError> {
    *total = total.saturating_add(line_count.saturating_mul(sample_count));
    if *total > MAX_PLANAR_CONTACT_CANDIDATE_PAIRS {
        return Err(crate::OperationsError::InvalidInput {
            reason: "planar edge-contact refinement exceeds its work budget".into(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod contact_budget_tests {
    use super::{MAX_PLANAR_CONTACT_CANDIDATE_PAIRS, add_planar_contact_work};
    use crate::OperationsError;

    #[test]
    fn planar_contact_budget_rejects_excess_and_overflow() {
        let mut work = 0;
        assert!(
            add_planar_contact_work(&mut work, 4_000, 1_000).is_ok(),
            "the documented candidate budget should remain available"
        );
        assert_eq!(work, MAX_PLANAR_CONTACT_CANDIDATE_PAIRS);
        assert!(matches!(
            add_planar_contact_work(&mut work, 1, 1),
            Err(OperationsError::InvalidInput { .. })
        ));

        let mut overflow_work = 0;
        assert!(matches!(
            add_planar_contact_work(&mut overflow_work, usize::MAX, 2),
            Err(OperationsError::InvalidInput { .. })
        ));
    }
}

fn has_trimmed_same_sphere_neighbor<V>(
    topo: &Topology,
    face_id: FaceId,
    edge_face_map: &std::collections::BTreeMap<usize, V>,
) -> Result<bool, crate::OperationsError>
where
    V: std::ops::Deref<Target = [FaceId]>,
{
    let face = topo.face(face_id)?;
    let FaceSurface::Sphere(sphere) = face.surface() else {
        return Ok(false);
    };
    if !face.inner_wires().is_empty() {
        return Ok(false);
    }

    let tol = remus_math::tolerance::Tolerance::new().linear;
    let wire = topo.wire(face.outer_wire())?;
    for oriented_edge in wire.edges() {
        let Some(neighbors) = edge_face_map.get(&oriented_edge.edge().index()) else {
            continue;
        };
        for &neighbor_id in &**neighbors {
            if neighbor_id == face_id {
                continue;
            }
            let neighbor = topo.face(neighbor_id)?;
            let FaceSurface::Sphere(other) = neighbor.surface() else {
                continue;
            };
            let centers_match = (other.center() - sphere.center()).length_squared() < tol * tol;
            if neighbor.inner_wires().len() == 1
                && (other.radius() - sphere.radius()).abs() < tol
                && centers_match
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Tessellate all faces of a solid into a single watertight triangle mesh.
///
/// Unlike per-face `tessellate()`, this function coordinates tessellation across
/// all faces of the solid by pre-computing shared edge tessellations. When two
/// faces share an edge, the edge is tessellated once and both faces receive
/// identical vertices along that boundary -- eliminating cracks between adjacent
/// faces and producing a guaranteed 2-manifold mesh.
///
/// # Algorithm
///
/// Based on Stoger & Kurka (2003), "Watertight Tessellation of B-rep NURBS
/// CAD-Models Using Connectivity Information":
///
/// 1. Build edge-to-face adjacency map from the solid's topology.
/// 2. Tessellate each unique edge once, producing a shared polyline.
/// 3. For each face, tessellate using cached edge points as boundary vertices.
/// 4. Merge all per-face meshes into a single mesh with shared boundary vertices.
///
/// # Errors
///
/// Returns an error if any topology lookup or face tessellation fails.
pub fn tessellate_solid(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    tessellate_solid_with_tolerance(
        topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
}

/// Tessellate a solid at a density safe for mesh-boolean co-refinement.
///
/// Identical to [`tessellate_solid_with_tolerance`] except circular edges keep
/// the curvature floor: the mesh boolean's robustness depends on the denser
/// floored circle sampling, independent of the display deflection cap.
///
/// # Errors
///
/// Returns an error if any topology lookup or face tessellation fails.
pub fn tessellate_solid_for_boolean(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    tessellate_solid_core(topo, solid, deflection, angular_tol, false, true)
        .map(|(mesh, _, _)| mesh)
}

/// Tessellate a solid with explicit linear and angular tolerances.
///
/// `angular_tol` (radians) caps the per-segment tangent turn; pass `0.0` to
/// disable the angular criterion (linear-only, backward-compatible) path.
///
/// # Errors
///
/// Returns an error if any topology lookup or face tessellation fails.
pub fn tessellate_solid_with_tolerance(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    tessellate_solid_core(topo, solid, deflection, angular_tol, false, false)
        .map(|(mesh, _, _)| mesh)
}

/// The quantity path reads positions and indices, never shading normals.
/// Skip only boundary normal reconstruction: welding, dedupe and gap fill
/// depend exclusively on positions/indices and retain their usual policy.
/// Face meshes remain compatible with the display cache; uncached normals
/// stay Unknown and a later display replay computes them normally.
pub fn tessellate_solid_for_measurement(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;
    tessellate_faces_core(
        topo,
        &faces,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
        MeshBoundaryMode::ClosedMeasurement,
        false,
        false,
    )
    .map(|(mesh, _, _)| mesh)
}

/// Tessellate a first-class sheet body into a merged, boundary-preserving
/// triangle mesh using the default angular tolerance.
///
/// Unlike solid tessellation, an open mesh boundary is expected and is not
/// filled or reported as a failure.
///
/// # Errors
///
/// Returns an error if the shell is not tagged as a sheet body or any face
/// tessellation fails.
pub fn tessellate_sheet(
    topo: &Topology,
    sheet: ShellId,
    deflection: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    tessellate_sheet_with_tolerance(
        topo,
        sheet,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
}

/// Tessellate a first-class sheet body with explicit linear and angular
/// tolerances.
///
/// # Errors
///
/// Returns an error if the shell is not tagged as a sheet body or any face
/// tessellation fails.
pub fn tessellate_sheet_with_tolerance(
    topo: &Topology,
    sheet: ShellId,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    let actual = topo.body_class_of(BodyId::Shell(sheet))?;
    if actual != BodyClass::Sheet {
        return Err(crate::OperationsError::BodyClassOperationUnsupported {
            operation: "sheet tessellation",
            actual: actual.as_str(),
        });
    }
    let faces = topo.shell(sheet)?.faces().to_vec();
    tessellate_faces_core(
        topo,
        &faces,
        deflection,
        angular_tol,
        MeshBoundaryMode::OpenSheet,
        false,
        false,
    )
    .map(|(mesh, _, _)| mesh)
}

/// Watertight tessellation of a closed face set that is not itself a solid:
/// one edge-connected component of a solid's shell.
///
/// Runs the same shared-edge-pool pipeline as [`tessellate_solid_with_tolerance`]
/// over just `faces`, so every face mesh honours its trim. The standalone
/// per-face mesher (`tessellate_with_uvs`) skins a trimmed cone wall's whole
/// parametric rectangle instead, which is no surface to ray-cast or clash
/// against (B53).
pub fn tessellate_closed_face_set(
    topo: &Topology,
    faces: &[FaceId],
    deflection: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    tessellate_faces_core(
        topo,
        faces,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
        MeshBoundaryMode::ClosedSolid,
        false,
        false,
    )
    .map(|(mesh, _, _)| mesh)
}

/// Tessellate any currently supported body class.
///
/// Solid and sheet bodies are supported. Wire bodies refuse typed rather than
/// producing an empty surface mesh.
///
/// # Errors
///
/// Returns a typed unsupported-body error for wire bodies, or propagates
/// topology and tessellation failures.
pub fn tessellate_body_with_tolerance(
    topo: &Topology,
    body: BodyId,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    match body {
        BodyId::Solid(solid) => {
            tessellate_solid_with_tolerance(topo, solid, deflection, angular_tol)
        }
        BodyId::Shell(sheet) => {
            tessellate_sheet_with_tolerance(topo, sheet, deflection, angular_tol)
        }
        BodyId::Wire(wire) => {
            let actual = topo.body_class_of(BodyId::Wire(wire))?;
            Err(crate::OperationsError::BodyClassOperationUnsupported {
                operation: "body tessellation",
                actual: actual.as_str(),
            })
        }
    }
}

/// Watertight solid tessellation with per-face triangle grouping.
///
/// Runs the same shared-edge-pool pipeline as [`tessellate_solid_with_tolerance`],
/// then reorders triangles so each face's triangles are contiguous. Returns the
/// mesh plus `face_offsets`: one entry per face of
/// `explorer::solid_faces(topo, solid)` (in that order, including empty groups)
/// where `face_offsets[i]` is the start offset into `mesh.indices` for face `i`,
/// plus a final sentinel equal to `mesh.indices.len()`.
///
/// # Errors
///
/// Returns an error if any topology lookup or face tessellation fails.
pub fn tessellate_solid_grouped_with_tolerance(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angular_tol: f64,
) -> Result<(TriangleMesh, Vec<u32>), crate::OperationsError> {
    let (mut mesh, tri_faces, n_faces) =
        tessellate_solid_core(topo, solid, deflection, angular_tol, true, false)?;
    let tri_faces = tri_faces.unwrap_or_default();
    debug_assert_eq!(tri_faces.len() * 3, mesh.indices.len());

    let mut counts = vec![0_u32; n_faces];
    for &f in &tri_faces {
        if let Some(c) = counts.get_mut(f as usize) {
            *c += 1;
        }
    }

    let mut face_offsets = Vec::with_capacity(n_faces + 1);
    let mut acc = 0_u32;
    face_offsets.push(0_u32);
    for &c in &counts {
        acc += c * 3;
        face_offsets.push(acc);
    }

    // Stable counting-sort scatter: per-face triangle order is preserved.
    let mut cursors: Vec<usize> = face_offsets[..n_faces]
        .iter()
        .map(|&o| o as usize)
        .collect();
    let mut new_indices = vec![0_u32; mesh.indices.len()];
    for (t, &f) in tri_faces.iter().enumerate() {
        let Some(cursor) = cursors.get_mut(f as usize) else {
            continue;
        };
        let dst = *cursor;
        new_indices[dst..dst + 3].copy_from_slice(&mesh.indices[t * 3..t * 3 + 3]);
        *cursor += 3;
    }
    mesh.indices = new_indices;

    Ok((mesh, face_offsets))
}

/// Boundary-sharing tessellation pipeline for a face set.
///
/// When `track_faces` is set, also returns a parallel `tri -> face` array (one
/// entry per triangle, holding the index of the owning face within
/// `explorer::solid_faces` order); otherwise the attribution bookkeeping is
/// skipped and `None` is returned. The face count is always returned.
///
/// `circle_floor` selects whether circular edges keep the curvature floor.
/// Display callers pass `false` (constant-curvature circles are exact without
/// it); the boolean mesh-fallback passes `true` for co-refinement robustness.
/// Because the shared edge pool drives the cylinder/cone band density, this one
/// flag governs every circular feature in the solid consistently.
#[allow(clippy::too_many_lines, clippy::fn_params_excessive_bools)]
fn tessellate_solid_core(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angular_tol: f64,
    track_faces: bool,
    circle_floor: bool,
) -> Result<(TriangleMesh, Option<Vec<u32>>, usize), crate::OperationsError> {
    use remus_topology::explorer;

    let all_faces = explorer::solid_faces(topo, solid)?;
    tessellate_faces_core(
        topo,
        &all_faces,
        deflection,
        angular_tol,
        MeshBoundaryMode::ClosedSolid,
        track_faces,
        circle_floor,
    )
}

#[derive(Clone, Copy)]
enum MeshBoundaryMode {
    ClosedSolid,
    OpenSheet,
    ClosedMeasurement,
}

#[allow(clippy::too_many_lines, clippy::fn_params_excessive_bools)]
fn tessellate_faces_core(
    topo: &Topology,
    all_faces: &[FaceId],
    deflection: f64,
    angular_tol: f64,
    boundary_mode: MeshBoundaryMode,
    track_faces: bool,
    circle_floor: bool,
) -> Result<(TriangleMesh, Option<Vec<u32>>, usize), crate::OperationsError> {
    let _total = crate::performance::span("mesh.total");
    let sampling = crate::performance::span("mesh.boundary-sampling");
    let edge_face_map = remus_topology::explorer::edge_to_face_map_for_faces(topo, all_faces)?;

    // The map is a std `HashMap`, so sort its keys into ID order before use —
    // keeping all downstream iteration deterministic regardless of
    // insertion-order hashing.
    let mut edge_indices: Vec<usize> = edge_face_map.keys().copied().collect();
    edge_indices.sort_unstable();
    // Stage A1 (PERF-D03): sample every shared edge once, retaining the
    // authoritative curve parameter per sample. Parallel and serial paths
    // must agree exactly: results are collected then inserted into fresh
    // maps in sorted edge order, so global identities never depend on
    // thread scheduling. The `circle_floor` policy travels with the plan
    // (display vs boolean callers keep their distinct densities).
    #[cfg(not(target_arch = "wasm32"))]
    let (mut edge_points, mut edge_params): (SampleChains, SampleParams) = if edge_indices.len()
        >= 32
    {
        use rayon::prelude::*;
        let results: Vec<Result<SampledEdge, crate::OperationsError>> = edge_indices
            .par_iter()
            .filter_map(|&edge_idx| {
                let edge_id = topo.edge_id_from_index(edge_idx)?;
                let edge_data = match topo.edge(edge_id) {
                    Ok(d) => d,
                    Err(e) => return Some(Err(crate::OperationsError::Topology(e))),
                };
                Some(
                    sample_edge_with_params(topo, edge_data, deflection, angular_tol, circle_floor)
                        .map(|(pts, params)| (edge_idx, pts, params)),
                )
            })
            .collect();
        let mut points_map = DetHashMap::default();
        let mut params_map = DetHashMap::default();
        for r in results {
            let (idx, pts, params) = r?;
            points_map.insert(idx, pts);
            params_map.insert(idx, params);
        }
        (points_map, params_map)
    } else {
        let mut points_map = DetHashMap::default();
        let mut params_map = DetHashMap::default();
        for &edge_idx in &edge_indices {
            if let Some(edge_id) = topo.edge_id_from_index(edge_idx)
                && let Ok(edge_data) = topo.edge(edge_id)
            {
                let (points, params) = sample_edge_with_params(
                    topo,
                    edge_data,
                    deflection,
                    angular_tol,
                    circle_floor,
                )?;
                points_map.insert(edge_idx, points);
                params_map.insert(edge_idx, params);
            }
        }
        (points_map, params_map)
    };
    #[cfg(target_arch = "wasm32")]
    let (mut edge_points, mut edge_params): (SampleChains, SampleParams) = {
        let mut points_map = DetHashMap::default();
        let mut params_map = DetHashMap::default();
        for &edge_idx in &edge_indices {
            if let Some(edge_id) = topo.edge_id_from_index(edge_idx)
                && let Ok(edge_data) = topo.edge(edge_id)
            {
                let (points, params) = sample_edge_with_params(
                    topo,
                    edge_data,
                    deflection,
                    angular_tol,
                    circle_floor,
                )?;
                points_map.insert(edge_idx, points);
                params_map.insert(edge_idx, params);
            }
        }
        (points_map, params_map)
    };

    // Stage A2 (PERF-D03): synchronize circle edge samples with face grid
    // density so a face's rim points line up with its own analytic grid
    // columns. Resampled chains keep authoritative uniform parameters, so
    // the plan still knows each sample's curve identity.
    //
    // Only faces that will be gridded take part. A cylinder wall with an
    // inner wire is tessellated by `tessellate_cylinder_with_holes`, which
    // walks its wires with `sample_edge` and never sees these resampled
    // points, so densifying its circle edges here only pulls the neighbouring
    // faces off the wall's own polyline: a boss's section arcs went from 7 to
    // 64 points on the caps while the wall kept 7, and the fused shell
    // tessellated open along the whole hole rim.
    // Pre-plan stage counters: booked into the plan metrics at stage A5.
    let mut circle_sync_upsampled: usize = 0;
    let mut torus_densified: usize = 0;
    let mut seam_splits: usize = 0;
    {
        use super::edge_sampling::uniform_edge_params;
        for &face_id in all_faces {
            let face_data = topo.face(face_id)?;
            if matches!(
                face_data.surface(),
                FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
            ) && !face_data.inner_wires().is_empty()
            {
                continue;
            }
            let face_nu = match face_data.surface() {
                FaceSurface::Cone(cone) => {
                    let v_range =
                        compute_v_param_range(topo, face_data, |p| cone.project_point(p).1);
                    let u_range =
                        compute_angular_range(topo, face_data, |p| cone.project_point(p))?;
                    let max_radius = cone.radius_at(v_range.1.abs().max(v_range.0.abs()));
                    segments_for_chord_deviation_a(
                        max_radius.max(0.01),
                        u_range.1 - u_range.0,
                        deflection,
                        angular_tol,
                        circle_floor,
                    )
                }
                FaceSurface::Cylinder(cyl) => {
                    let u_range = compute_angular_range(topo, face_data, |p| cyl.project_point(p))?;
                    segments_for_chord_deviation_a(
                        cyl.radius(),
                        u_range.1 - u_range.0,
                        deflection,
                        angular_tol,
                        circle_floor,
                    )
                }
                _ => continue,
            };
            let expected_count = face_nu + 1;

            let mut wire_ids = vec![face_data.outer_wire()];
            wire_ids.extend_from_slice(face_data.inner_wires());
            for &wire_id in &wire_ids {
                let wire = topo.wire(wire_id)?;
                for oe in wire.edges() {
                    let edge_idx = oe.edge().index();
                    let Some(edge_id) = topo.edge_id_from_index(edge_idx) else {
                        continue;
                    };
                    let Ok(edge_data) = topo.edge(edge_id) else {
                        continue;
                    };
                    let EdgeCurve::Circle(circle) = edge_data.curve() else {
                        continue;
                    };

                    if let Some(pts) = edge_points.get(&edge_idx)
                        && pts.len() < expected_count
                    {
                        let (t_start, t_end) = circle_param_range(edge_data)?;
                        let mut new_pts = super::edge_sampling::sample_circle_uniform(
                            circle,
                            t_start,
                            t_end,
                            expected_count,
                        );
                        if let Some(first) = new_pts.first_mut() {
                            *first = topo.vertex(edge_data.start())?.point();
                        }
                        if let Some(last) = new_pts.last_mut() {
                            *last = topo.vertex(edge_data.end())?.point();
                        }
                        edge_points.insert(edge_idx, new_pts);
                        edge_params.insert(
                            edge_idx,
                            uniform_edge_params(t_start, t_end, expected_count),
                        );
                        circle_sync_upsampled += 1;
                    }
                }
            }
        }
    }

    // Stage A3 (PERF-D03): densify torus two-rim band rims to the band
    // mesher's own wrap density so the shared pool and the interior rows
    // agree by construction.
    //
    // `tessellate_torus_two_rim_band` sizes its interior rows with the
    // curvature floor (`R + r` over a full turn) while the shared pool
    // samples circles floor-free. At coarse deflection the sparse pool rims
    // stitch against far denser interior rows and the band cracks (B46: a
    // 0.05 fillet band open at 0.1 and 0.01). Densifying only (never
    // coarsening) the band's once-used rim circles to the mesher's density
    // keeps every neighbour on the same vertices. Split rims share the full
    // turn proportionally by arc so independently split rims stay matched.
    {
        for &face_id in all_faces {
            let face_data = topo.face(face_id)?;
            let FaceSurface::Torus(torus) = face_data.surface() else {
                continue;
            };
            if !face_data.inner_wires().is_empty() {
                continue;
            }
            let wire = topo.wire(face_data.outer_wire())?;
            let mut uses: DetHashMap<usize, usize> = DetHashMap::default();
            for oe in wire.edges() {
                *uses.entry(oe.edge().index()).or_default() += 1;
            }
            let mut seam_found = false;
            let mut rim_edges = Vec::new();
            let mut seen: DetHashSet<usize> = DetHashSet::default();
            let mut is_band = true;
            for oe in wire.edges() {
                if !seen.insert(oe.edge().index()) {
                    continue;
                }
                let edge = topo.edge(oe.edge())?;
                let count = uses.get(&oe.edge().index()).copied().unwrap_or(0);
                if count == 2 {
                    if edge.start() == edge.end() || seam_found {
                        is_band = false;
                        break;
                    }
                    seam_found = true;
                } else if count == 1 {
                    if matches!(edge.curve(), EdgeCurve::Circle(_)) {
                        rim_edges.push(oe.edge());
                    } else {
                        is_band = false;
                        break;
                    }
                } else {
                    is_band = false;
                    break;
                }
            }
            if !is_band || !seam_found || rim_edges.is_empty() {
                continue;
            }
            let wrap_radius = torus.major_radius() + torus.minor_radius();
            let full_cols = segments_for_chord_deviation_a(
                wrap_radius,
                std::f64::consts::TAU,
                deflection,
                angular_tol,
                true,
            );
            for rim in rim_edges {
                let edge_idx = rim.index();
                let Some(edge_id) = topo.edge_id_from_index(edge_idx) else {
                    continue;
                };
                let Ok(edge_data) = topo.edge(edge_id) else {
                    continue;
                };
                let EdgeCurve::Circle(circle) = edge_data.curve() else {
                    continue;
                };
                let (t_start, t_end) = match circle_param_range(edge_data) {
                    Ok(range) => range,
                    Err(_) => continue,
                };
                let arc = (t_end - t_start).abs();
                if arc <= 0.0 {
                    continue;
                }
                #[allow(
                    clippy::cast_precision_loss,
                    clippy::cast_possible_truncation,
                    clippy::cast_sign_loss
                )]
                let expected =
                    ((full_cols as f64 * arc / std::f64::consts::TAU).ceil() as usize + 1).max(2);
                let needs_densify = edge_points
                    .get(&edge_idx)
                    .is_some_and(|pts| pts.len() < expected);
                if needs_densify {
                    let mut new_pts = super::edge_sampling::sample_circle_uniform(
                        circle, t_start, t_end, expected,
                    );
                    if let Some(first) = new_pts.first_mut() {
                        first.clone_from(&topo.vertex(edge_data.start())?.point());
                    }
                    if let Some(last) = new_pts.last_mut() {
                        last.clone_from(&topo.vertex(edge_data.end())?.point());
                    }
                    edge_points.insert(edge_idx, new_pts);
                    edge_params.insert(
                        edge_idx,
                        super::edge_sampling::uniform_edge_params(t_start, t_end, expected),
                    );
                    torus_densified += 1;
                }
            }
        }
    }

    // Stage A4 (PERF-D03): pre-split holed periodic wall inner-wire
    // polylines at their seam-meridian crossings. A hole loop crossing the
    // chart's seam meridian gets cut there by the developed-chart mesher,
    // which fabricates a boundary vertex ON the seam that no shared edge
    // sample carries. The face sharing that hole edge (e.g. the bore
    // wall) stitches the shared polyline directly and skips the fabricated
    // point, leaving a micro-triangle hole at the seam. Pre-splitting every
    // inner-wire polyline of such a wall at its crossings gives both
    // consumers the same vertex (the chart mesher's own crossing then welds
    // to it via the 1e-6 boundary snap). Inserted crossings are geometric
    // (chart interpolation), so their retained curve parameters are unknown.
    {
        let refine_tol = remus_math::tolerance::Tolerance::new().linear * 10.0;
        // Keep the pool immutable while discovering crossings.  A malformed
        // non-manifold model may reuse one edge from many faces with different
        // seam meridians; growing that edge after every face would make each
        // later face rescan and clone all earlier insertions (quadratic work).
        let mut refinements: DetHashMap<usize, Vec<(usize, f64, Point3)>> = DetHashMap::default();
        for &face_id in all_faces {
            let face_data = topo.face(face_id)?;
            let FaceSurface::Cylinder(cyl) = face_data.surface() else {
                continue;
            };
            if face_data.inner_wires().is_empty() {
                continue;
            }
            // The seam meridian: an open outer-wire edge used twice (the
            // full-turn chart's cut line). A partial band has none — skip.
            let outer = topo.wire(face_data.outer_wire())?;
            let mut edge_uses: DetHashMap<usize, usize> = DetHashMap::default();
            for oe in outer.edges() {
                *edge_uses.entry(oe.edge().index()).or_default() += 1;
            }
            let mut seam_u = None;
            for oe in outer.edges() {
                if edge_uses.get(&oe.edge().index()).copied().unwrap_or(0) < 2 {
                    continue;
                }
                let e = topo.edge(oe.edge())?;
                if e.start() == e.end() {
                    continue;
                }
                let sp = topo.vertex(e.start())?.point();
                let ep = topo.vertex(e.end())?.point();
                let mid = sp + (ep - sp) * 0.5;
                seam_u = Some(cyl.project_point(mid).0);
                break;
            }
            let Some(seam_u) = seam_u else {
                continue;
            };
            let tau = std::f64::consts::TAU;
            // Signed angular offset from the seam meridian, in (-pi, pi].
            let meridian_offset = |p: Point3| {
                let d = (cyl.project_point(p).0 - seam_u).rem_euclid(tau);
                if d > std::f64::consts::PI { d - tau } else { d }
            };
            for &wire_id in face_data.inner_wires() {
                let wire = topo.wire(wire_id)?;
                for oe in wire.edges() {
                    let edge_idx = oe.edge().index();
                    let Some(pts) = edge_points.get(&edge_idx) else {
                        continue;
                    };
                    let offsets: Vec<f64> = pts.iter().map(|&p| meridian_offset(p)).collect();
                    for i in 0..pts.len().saturating_sub(1) {
                        let (a, b) = (offsets[i], offsets[i + 1]);
                        // A genuine meridian crossing changes sign over a
                        // short angular step; a sign change spanning >= pi is
                        // the far side of the period, not the seam.
                        if a == 0.0 || b == 0.0 || a.signum() == b.signum() {
                            continue;
                        }
                        if (a - b).abs() >= std::f64::consts::PI {
                            continue;
                        }
                        let t = a / (a - b);
                        let va = cyl.project_point(pts[i]).1;
                        let vb = cyl.project_point(pts[i + 1]).1;
                        let crossing =
                            if let EdgeCurve::NurbsCurve(curve) = topo.edge(oe.edge())?.curve() {
                                let project = |point| {
                                    remus_math::nurbs::projection::project_point_to_curve(
                                        curve, point, 1e-10,
                                    )
                                };
                                let mut lo = project(pts[i])?.parameter;
                                let mut hi = project(pts[i + 1])?.parameter;
                                let (start, end) = curve.domain();
                                let period = end - start;
                                let closed = topo.edge(oe.edge())?.is_closed();
                                if closed && (hi - lo).abs() > period * 0.5 {
                                    hi -= period * (hi - lo).signum();
                                }
                                let evaluate = |parameter: f64| {
                                    curve.evaluate(if closed {
                                        start + (parameter - start).rem_euclid(period)
                                    } else {
                                        parameter
                                    })
                                };
                                // Interpolating height in the cylinder chart moves
                                // this shared vertex off the other support surface.
                                // Solve on the intersection curve, including across
                                // the parameter origin of a closed edge.
                                for _ in 0..60 {
                                    let mid = f64::midpoint(lo, hi);
                                    let offset = meridian_offset(evaluate(mid));
                                    if offset.signum() == a.signum() {
                                        lo = mid;
                                    } else {
                                        hi = mid;
                                    }
                                }
                                evaluate(f64::midpoint(lo, hi))
                            } else {
                                cyl.evaluate(seam_u, (vb - va).mul_add(t, va))
                            };
                        if (crossing - pts[i]).length() < refine_tol
                            || (crossing - pts[i + 1]).length() < refine_tol
                        {
                            continue;
                        }
                        refinements
                            .entry(edge_idx)
                            .or_default()
                            .push((i + 1, t, crossing));
                    }
                }
            }
        }

        // Apply each shared edge's requested splits once. Sorting by original
        // segment and interpolation parameter both preserves polyline order
        // and puts coincident requests next to one another for linear dedup.
        for (edge_idx, mut insertions) in refinements {
            insertions.sort_by(|(at_a, t_a, _), (at_b, t_b, _)| {
                at_a.cmp(at_b).then_with(|| t_a.total_cmp(t_b))
            });
            insertions.dedup_by(|(at_b, _, point_b), (at_a, _, point_a)| {
                at_a == at_b && (*point_a - *point_b).length() < refine_tol
            });
            let Some(pts) = edge_points.get_mut(&edge_idx) else {
                continue;
            };
            pts.reserve(insertions.len());
            for &(at, _, point) in insertions.iter().rev() {
                pts.insert(at, point);
            }
            // Geometrically inserted crossings carry no curve parameter; the
            // params chain mirrors the points chain one-for-one so sample
            // identities stay aligned.
            let params = edge_params.entry(edge_idx).or_default();
            while params.len() < pts.len().saturating_sub(insertions.len()) {
                params.push(None);
            }
            params.reserve(insertions.len());
            seam_splits += insertions.len();
            for &(at, _, _) in insertions.iter().rev() {
                params.insert(at.min(params.len()), None);
            }
            debug_assert_eq!(pts.len(), params.len());
        }
    }

    // Stage A5 (PERF-D03): freeze the sampled chains into the explicit
    // boundary plan. From here on every stage reads sample geometry,
    // authority, and identities from `plan` — never from ad-hoc locals — so
    // the display-vs-boolean policy (`circle_floor`), authoritative domains,
    // and per-sample parameters travel with the data they govern.
    // Pre-plan stage work (A2–A4 ran before the plan existed) is booked
    // into the metrics here so the accounting covers the whole pipeline.
    let mut plan = BoundaryPlan::from_samples(
        topo,
        edge_points,
        edge_params,
        deflection,
        angular_tol,
        circle_floor,
    );
    plan.metrics.circle_sync_upsampled = circle_sync_upsampled;
    plan.metrics.torus_densified = torus_densified;
    plan.metrics.seam_splits = seam_splits;

    // Stage A6 (PERF-D03): planar contact refinement. A straight boundary
    // edge that a neighbouring curved trim touches tangentially must share
    // that subdivision with every incident face, or the planar CDT spans
    // the contact in one constraint while the neighbour stops on it (a
    // T-junction crack). Circle refinements likewise merge pool positions
    // lying on the circle into the shared chain. Operates on the plan's
    // chains and merged pool; counts feed the plan metrics.
    {
        let tol_linear = remus_math::tolerance::Tolerance::new().linear;
        let refine_tol = tol_linear * 10.0;
        // Keep each face's circular samples once, rather than materializing the
        // Cartesian product of every line and sample in `line_contacts`.
        let mut contact_groups = Vec::new();
        let mut contact_work = 0;
        for &face_id in all_faces {
            let face = topo.face(face_id)?;
            if !matches!(face.surface(), FaceSurface::Plane { .. }) {
                continue;
            }
            let mut lines = Vec::new();
            let mut curved_samples = Vec::new();
            for wire_id in
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oriented in topo.wire(wire_id)?.edges() {
                    let edge = topo.edge(oriented.edge())?;
                    let index = oriented.edge().index();
                    if matches!(edge.curve(), EdgeCurve::Line) {
                        lines.push(index);
                    } else if matches!(edge.curve(), EdgeCurve::Circle(_)) {
                        curved_samples
                            .extend(plan.edge_chains.get(&index).into_iter().flatten().copied());
                    }
                }
            }
            lines.sort_unstable();
            lines.dedup();
            curved_samples.sort_unstable();
            curved_samples.dedup();
            if !lines.is_empty() && !curved_samples.is_empty() {
                add_planar_contact_work(&mut contact_work, lines.len(), curved_samples.len())?;
                contact_groups.push((lines, curved_samples));
            }
        }

        // Retain only genuine contacts. Memory is now proportional to the
        // input groups plus the subdivisions that can reach the output.
        let mut line_contacts: DetHashMap<usize, DetHashSet<u32>> = DetHashMap::default();
        for (lines, curved_samples) in contact_groups {
            for index in lines {
                let Some(edge_id) = topo.edge_id_from_index(index) else {
                    continue;
                };
                let edge = topo.edge(edge_id)?;
                let start = topo.vertex(edge.start())?.point();
                let end = topo.vertex(edge.end())?.point();
                let direction = end - start;
                let length_squared = direction.length_squared();
                let boundary_tol = 1e-10;
                if length_squared <= boundary_tol * boundary_tol {
                    continue;
                }
                for &gid in &curved_samples {
                    let point = plan.merged.positions[gid as usize];
                    let t = (point - start).dot(direction) / length_squared;
                    if t > 0.0
                        && t < 1.0
                        && (point - (start + direction * t)).length() < boundary_tol
                    {
                        line_contacts.entry(index).or_default().insert(gid);
                    }
                }
            }
        }

        // Built on first use: a body without circle edges never pays for it.
        let mut pool_index: Option<Option<super::pool_index::PoolIndex>> = None;
        for &edge_idx in &edge_indices {
            let Some(edge_id) = topo.edge_id_from_index(edge_idx) else {
                continue;
            };
            let Ok(edge_data) = topo.edge(edge_id) else {
                continue;
            };
            if matches!(edge_data.curve(), EdgeCurve::Line) {
                let Some(candidates) = line_contacts.get(&edge_idx) else {
                    continue;
                };
                // CDT subdivides a straight constraint at an existing tangent
                // boundary sample. Share that subdivision with every edge user.
                let start = topo.vertex(edge_data.start())?.point();
                let end = topo.vertex(edge_data.end())?.point();
                let direction = end - start;
                let length_squared = direction.length_squared();
                let boundary_tol = 1e-10;
                if length_squared > boundary_tol * boundary_tol {
                    let mut samples: Vec<(f64, u32)> = plan
                        .edge_chains
                        .get(&edge_idx)
                        .into_iter()
                        .flatten()
                        .map(|&gid| {
                            (
                                (plan.merged.positions[gid as usize] - start).dot(direction)
                                    / length_squared,
                                gid,
                            )
                        })
                        .collect();
                    for &gid in candidates {
                        let point = plan.merged.positions[gid as usize];
                        let t = (point - start).dot(direction) / length_squared;
                        if t > 0.0
                            && t < 1.0
                            && (point - (start + direction * t)).length() < boundary_tol
                        {
                            samples.push((t, gid));
                        }
                    }
                    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
                    samples.dedup_by_key(|sample| sample.1);
                    plan.edge_chains
                        .insert(edge_idx, samples.into_iter().map(|(_, gid)| gid).collect());
                    plan.metrics.contact_lines_refined += 1;
                }
                continue;
            }
            let EdgeCurve::Circle(circle) = edge_data.curve() else {
                continue;
            };

            let Ok(start_vtx) = topo.vertex(edge_data.start()) else {
                continue;
            };
            let Ok(end_vtx) = topo.vertex(edge_data.end()) else {
                continue;
            };
            let start_pos = start_vtx.point();
            let end_pos = end_vtx.point();

            // Authoritative domain and closed flag come from the plan, which
            // retained them at sample time — the refinement never re-derives
            // trim authority from endpoints. Edges the plan never sampled
            // (no authority) fall back to a direct query; lines never reach
            // this arm, so a missing domain there skips the edge.
            let (t_min, t_max, is_closed) = if let Some(authority) = plan.authorities.get(&edge_idx)
            {
                let Some(domain) = authority.domain else {
                    continue;
                };
                (domain.0, domain.1, authority.is_closed)
            } else {
                let (domain_min, domain_max) =
                    crate::authoritative_edge_domain(edge_data, "body edge refinement")?;
                (domain_min, domain_max, edge_data.start() == edge_data.end())
            };

            let existing_gids_vec: Vec<u32> =
                plan.edge_chains.get(&edge_idx).cloned().unwrap_or_default();
            let existing_gids: DetHashSet<u32> = existing_gids_vec.iter().copied().collect();

            // PERF-D07: scan only the pool vertices along the arc (in
            // ascending id order, like the full scan) when the index can
            // bound them; the acceptance test below is unchanged.
            let index = pool_index.get_or_insert_with(|| {
                super::pool_index::PoolIndex::new(&plan.merged.positions, refine_tol)
            });
            let range = (!is_closed && t_min < t_max).then_some((t_min - 1e-8, t_max + 1e-8));
            let candidates = index
                .as_ref()
                .and_then(|index| index.circle_candidates(circle, range));
            #[cfg(test)]
            let candidates =
                candidates.filter(|_| !super::tests::mesh_passes::use_reference_passes());
            #[allow(clippy::cast_possible_truncation)]
            let candidates: Vec<u32> =
                candidates.unwrap_or_else(|| (0..plan.merged.positions.len() as u32).collect());
            let mut insertions: Vec<(f64, u32)> = Vec::new();
            for gid32 in candidates {
                let pos = &plan.merged.positions[gid32 as usize];
                if existing_gids.contains(&gid32) {
                    continue;
                }
                if (*pos - start_pos).length() < refine_tol {
                    continue;
                }
                if !is_closed && (*pos - end_pos).length() < refine_tol {
                    continue;
                }
                let t = circle.project(*pos);
                let on_circle = circle.evaluate(t);
                let dist = (*pos - on_circle).length();
                if dist >= refine_tol {
                    continue;
                }
                let in_range = if is_closed {
                    true
                } else if t_min < t_max {
                    t >= t_min - 1e-8 && t <= t_max + 1e-8
                } else {
                    t >= t_min - 1e-8 || t <= t_max + 1e-8
                };
                if in_range {
                    insertions.push((t, gid32));
                }
            }

            if insertions.is_empty() {
                continue;
            }

            insertions.sort_by(|a, b| a.0.total_cmp(&b.0));
            insertions.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-8);

            let mut all_with_t: Vec<(f64, u32)> = existing_gids_vec
                .iter()
                .map(|&gid| {
                    let pos = plan.merged.positions[gid as usize];
                    (circle.project(pos), gid)
                })
                .collect();
            plan.metrics.contact_circle_insertions += insertions.len();
            all_with_t.extend(insertions);
            all_with_t.sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut seen_gids = DetHashSet::default();
            all_with_t.retain(|(_, gid)| seen_gids.insert(*gid));

            let refined: Vec<u32> = all_with_t.into_iter().map(|(_, gid)| gid).collect();
            plan.edge_chains.insert(edge_idx, refined);
        }
    }

    // When tracking, `tri_faces` runs parallel to the mesh triangles:
    // tri_faces[t] is the index (into `all_faces`) of the face that produced
    // triangle t. The ungrouped caller skips this bookkeeping entirely.
    let mut tri_faces: Option<Vec<u32>> = track_faces.then(Vec::new);
    #[allow(clippy::items_after_statements)]
    struct CdtJob {
        face_index: u32,
        pts2d: Vec<remus_math::vec::Point2>,
        outer_count: usize,
        inner_wire_ranges: Vec<(usize, usize)>,
        all_global_ids: Vec<Option<u32>>,
        all_positions: Vec<Point3>,
        normal: Vec3,
        is_reversed: bool,
    }
    #[allow(clippy::items_after_statements)]
    type CdtResult = Result<super::planar::PlanarCdtOutput, crate::OperationsError>;
    /// A staged holed-planar triangulation: triangles plus the 3D/global
    /// data to emit them after the reconcile (stage C, part 3).
    #[allow(clippy::items_after_statements)]
    struct StagedCdt {
        face_index: u32,
        tris: Vec<(usize, usize, usize)>,
        n_input: usize,
        all_positions: Vec<Point3>,
        all_global_ids: Vec<Option<u32>>,
        steiner_positions: Vec<Point3>,
        steiner_gids: Vec<u32>,
        normal: Vec3,
        is_reversed: bool,
    }

    let mut cdt_jobs: Vec<CdtJob> = Vec::new();
    let mut other_face_indices: Vec<usize> = Vec::new();

    drop(sampling);
    let planar = crate::performance::span("mesh.planar-cdt");
    // Stage B (PERF-D03): local triangulation. Holed-planar faces snapshot
    // the plan's chains into independent CDT jobs; every other face waits
    // for stage C. Job construction only reads the plan — collecting a job
    // never mutates shared state, which is what makes the jobs safe to run
    // in parallel (native) or sequentially (WASM) with identical results.
    for (fi, &face_id) in all_faces.iter().enumerate() {
        let face_data = topo.face(face_id)?;
        let has_inner = !face_data.inner_wires().is_empty();
        if let FaceSurface::Plane { normal, .. } = face_data.surface()
            && has_inner
        {
            let normal = *normal;
            let is_reversed = face_data.is_reversed();
            let wire = topo.wire(face_data.outer_wire())?;
            let tol = 1e-10;

            let (mut all_positions, mut all_global_ids) = plan.collect_wire_vertices(wire, tol);

            remove_closing_duplicate_global(
                &mut all_positions,
                &mut all_global_ids,
                &plan.merged.positions,
                tol,
            );
            let outer_count = all_positions.len();

            let mut inner_wire_ranges: Vec<(usize, usize)> = Vec::new();
            for &iw_id in face_data.inner_wires() {
                let iw = topo.wire(iw_id)?;
                let start = all_positions.len();
                let (inner_pos, inner_gids) = plan.collect_wire_vertices(iw, tol);
                let mut inner_flat_ids: Vec<u32> = Vec::with_capacity(inner_gids.len());
                let mut next_sentinel = u32::MAX;
                for (pos, gid_opt) in inner_pos.into_iter().zip(inner_gids) {
                    let gid = gid_opt.unwrap_or_else(|| {
                        debug_assert!(false, "inner wire vertex had no global ID");
                        let s = next_sentinel;
                        next_sentinel = next_sentinel.wrapping_sub(1);
                        s
                    });
                    inner_flat_ids.push(gid);
                    all_positions.push(pos);
                    all_global_ids.push(Some(gid));
                }
                if inner_flat_ids.len() > 2 {
                    remove_closing_duplicate_ids(&mut inner_flat_ids, &plan.merged.positions, tol);
                    let expected_end = start + inner_flat_ids.len();
                    all_positions.truncate(expected_end);
                    all_global_ids.truncate(expected_end);
                }
                let end = all_positions.len();
                inner_wire_ranges.push((start, end));
            }

            let pts2d: Vec<remus_math::vec::Point2> = all_positions
                .iter()
                .map(|&p| project_by_normal(p, normal))
                .collect();

            #[allow(clippy::cast_possible_truncation)]
            cdt_jobs.push(CdtJob {
                face_index: fi as u32,
                pts2d,
                outer_count,
                inner_wire_ranges,
                all_global_ids,
                all_positions,
                normal,
                is_reversed,
            });
            continue;
        }
        other_face_indices.push(fi);
    }

    #[cfg(not(target_arch = "wasm32"))]
    let cdt_results: Vec<CdtResult> = if cdt_jobs.len() >= 2 {
        use rayon::prelude::*;
        cdt_jobs
            .par_iter()
            .map(|job| run_planar_cdt(&job.pts2d, job.outer_count, &job.inner_wire_ranges))
            .collect()
    } else {
        cdt_jobs
            .iter()
            .map(|job| run_planar_cdt(&job.pts2d, job.outer_count, &job.inner_wire_ranges))
            .collect()
    };
    #[cfg(target_arch = "wasm32")]
    let cdt_results: Vec<CdtResult> = cdt_jobs
        .iter()
        .map(|job| run_planar_cdt(&job.pts2d, job.outer_count, &job.inner_wire_ranges))
        .collect();

    // Where the planar CDT jobs' triangles start, so a job whose boundary
    // segment another job Steiner-split can be repaired after all of them
    // have emitted (see `split_triangles_spanning_boundary_splits`).
    let cdt_index_start = plan.merged.indices.len();
    let cdt_face_start = tri_faces.as_ref().map_or(0, Vec::len);

    // Stage C, part 1 (PERF-D03): stage local triangulation. Each job's
    // triangles wait while its boundary-refinement requests are collected
    // as data. Nothing here mutates the shared chains, so staging is
    // independent of job completion order; Steiner lifting keeps face order
    // (first writer of a merge-grid cell wins, as in the pool build).
    let mut staged: Vec<StagedCdt> = Vec::with_capacity(cdt_jobs.len());
    let mut refinement_requests: Vec<BoundaryRefinementRequest> = Vec::new();

    for (job, result) in cdt_jobs.into_iter().zip(cdt_results) {
        let (tris, steiner) = result?;

        // Lift constraint-recovery Steiner points to 3D global vertices. A
        // Steiner point that lies ON a shared boundary edge becomes a
        // refinement REQUEST on that edge's shared sample chain (reconciled
        // below) so the NEIGHBOUR faces pick it up — without this the
        // neighbour spans the original segment in one piece and the mesh
        // cracks at a T-junction. Requests are data: the chains are
        // untouched until every job has been staged.
        let n_input = job.all_positions.len();
        let mut steiner_positions: Vec<Point3> = Vec::with_capacity(steiner.len());
        for p2d in &steiner {
            steiner_positions.push(unproject_point(*p2d, job.normal, &job.all_positions[0]));
        }
        let steiner_gids = plan.lift_steiner_points(steiner_positions.clone(), job.normal);
        if !steiner.is_empty() {
            let ring_segments: Vec<(usize, usize)> = (0..job.outer_count)
                .map(|i| (i, (i + 1) % job.outer_count))
                .chain(job.inner_wire_ranges.iter().flat_map(|&(st, en)| {
                    (st..en).map(move |i| (i, if i + 1 == en { st } else { i + 1 }))
                }))
                .collect();
            let mut per_segment: DetHashMap<(usize, usize), Vec<(f64, u32)>> =
                DetHashMap::default();
            for (si, p2d) in steiner.iter().enumerate() {
                for &(i, j) in &ring_segments {
                    let a = job.pts2d[i];
                    let b = job.pts2d[j];
                    let ab = (b.x() - a.x(), b.y() - a.y());
                    let len2 = ab.0 * ab.0 + ab.1 * ab.1;
                    if len2 < 1e-24 {
                        continue;
                    }
                    let ap = (p2d.x() - a.x(), p2d.y() - a.y());
                    let t = (ap.0 * ab.0 + ap.1 * ab.1) / len2;
                    if !(1e-9..=1.0 - 1e-9).contains(&t) {
                        continue;
                    }
                    let cross = ap.0 * ab.1 - ap.1 * ab.0;
                    if cross * cross / len2 < 1e-18 {
                        per_segment
                            .entry((i, j))
                            .or_default()
                            .push((t, steiner_gids[si]));
                        break;
                    }
                }
            }
            for ((i, j), mut run) in per_segment {
                run.sort_by(|a, b| a.0.total_cmp(&b.0));
                let (Some(gi), Some(gj)) = (job.all_global_ids[i], job.all_global_ids[j]) else {
                    continue;
                };
                if gi == gj {
                    continue;
                }
                refinement_requests.push(BoundaryRefinementRequest::new(
                    job.face_index,
                    gi,
                    gj,
                    &run,
                    false,
                ));
            }
        }

        staged.push(StagedCdt {
            face_index: job.face_index,
            tris,
            n_input,
            all_positions: job.all_positions,
            all_global_ids: job.all_global_ids,
            steiner_positions,
            steiner_gids,
            normal: job.normal,
            is_reversed: job.is_reversed,
        });
    }

    // Stage C, part 2 (PERF-D03): reconcile. Merged runs splice each shared
    // segment exactly once, in sorted segment order, so every incident face
    // observes the same final subdivision regardless of job order. Bounded:
    // excess fails closed with no partial mesh.
    let boundary_splits = plan.reconcile(refinement_requests)?;
    // Structural self-check: tolerances, policy, points/params/chains, and
    // per-edge authorities agree with the topology. Debug-only; zero
    // release cost.
    debug_assert!(plan.validate_structure(topo).is_ok());

    // Stage C, part 3 (PERF-D03): deterministic assembly of the staged
    // holed-planar triangulations, in face-set order. Staged jobs keep the
    // order they were collected in (face-set order), so emission — and the
    // parallel `tri_faces` attribution — is identical on every run, thread
    // count, and platform.
    for staged_job in &staged {
        let pos_of = |i: usize| -> Point3 {
            if i < staged_job.n_input {
                staged_job.all_positions[i]
            } else {
                staged_job.steiner_positions[i - staged_job.n_input]
            }
        };
        let gid_of = |i: usize| -> u32 {
            if i < staged_job.n_input {
                staged_job.all_global_ids[i].unwrap_or(0)
            } else {
                staged_job.steiner_gids[i - staged_job.n_input]
            }
        };

        let needs_flip = if let Some(&(i0, i1, i2)) = staged_job.tris.first() {
            let p0 = pos_of(i0);
            let p1 = pos_of(i1);
            let p2 = pos_of(i2);
            let a = p1 - p0;
            let b = p2 - p0;
            let winding_matches = a.cross(b).dot(staged_job.normal) > 0.0;
            winding_matches == staged_job.is_reversed
        } else {
            false
        };

        for &(i0, i1, i2) in &staged_job.tris {
            let g0 = gid_of(i0);
            let g1 = gid_of(i1);
            let g2 = gid_of(i2);
            if let Some(tf) = tri_faces.as_mut() {
                tf.push(staged_job.face_index);
            }
            if needs_flip {
                plan.merged.indices.push(g0);
                plan.merged.indices.push(g2);
                plan.merged.indices.push(g1);
            } else {
                plan.merged.indices.push(g0);
                plan.merged.indices.push(g1);
                plan.merged.indices.push(g2);
            }
        }
    }

    // The reconcile above reaches only the faces tessellated AFTER the
    // CDT jobs. Every staged job was triangulated up front, so a neighbour
    // that is itself a holed plane still spans the split segment in one
    // triangle: a T-junction crack along the shared edge (a countersunk
    // bracket's floor cap Steiner-split its edge with the arm face that
    // carries an emboss hole, leaving three open mesh edges). Split those
    // triangles at the recorded Steiner points so both sides share every
    // vertex. This is the proven repair route: bounded by the chain lengths
    // (see the budget inside), never an open-ended fixed-point loop.
    if !boundary_splits.is_empty() {
        split_triangles_spanning_boundary_splits(
            &mut plan.merged.indices,
            cdt_index_start,
            tri_faces.as_mut().map(|tf| (tf, cdt_face_start)),
            boundary_splits,
        );
    }

    drop(planar);
    let curved = crate::performance::span("mesh.curved-faces");
    // Stage D (PERF-D03): every remaining face triangulates directly
    // against the final reconciled chains, so neighbours of a Steiner-split
    // segment pick up the same vertices the repair above gave the staged
    // jobs. Tolerances and the display-vs-boolean policy are read from the
    // plan (single source of truth — they match the locals by
    // construction). Curved families keep their established
    // structured/CDT/snap dispatch and fallback behaviour; only the plan
    // they read from is now explicit.
    //
    // PERF-D01: with this thread's face mesh cache enabled, a face whose
    // content key (surface, wires, final boundary chains, tolerances) matches
    // a retained capture up to a rigid translation is replayed instead of
    // re-meshed; every other face is meshed as before and captured.
    let mut cache_session =
        face_cache::Session::begin(matches!(boundary_mode, MeshBoundaryMode::ClosedMeasurement));
    for &fi in &other_face_indices {
        let face_id = all_faces[fi];
        let allow_latitude_cap = !plan.policy.circle_floor
            && has_trimmed_same_sphere_neighbor(topo, face_id, &edge_face_map)?;
        let mut capture = None;
        if let Some(session) = cache_session.as_mut()
            && let Ok((key, boundary)) = face_cache::face_key(
                topo,
                face_id,
                face_cache::MeshRequest {
                    deflection: plan.deflection,
                    angular_tol: plan.angular_tol,
                    circle_floor: plan.policy.circle_floor,
                    allow_latitude_cap,
                },
                &plan.edge_chains,
                &plan.merged.positions,
            )
        {
            match session.try_replay(
                face_id,
                &key,
                boundary,
                &mut plan.merged,
                &mut plan.point_to_global,
            ) {
                Ok(()) => {
                    if let Some(tf) = tri_faces.as_mut() {
                        #[allow(clippy::cast_possible_truncation)]
                        tf.resize(plan.merged.indices.len() / 3, fi as u32);
                    }
                    continue;
                }
                Err(boundary) => {
                    let start = face_cache::Session::capture_start(
                        &boundary,
                        &plan.merged,
                        &plan.point_to_global,
                    );
                    capture = Some((key, boundary, start));
                }
            }
        }
        let nurbs_frame_chart = tessellate_face_with_shared_edges(
            topo,
            face_id,
            plan.deflection,
            plan.angular_tol,
            plan.policy.circle_floor,
            allow_latitude_cap,
            &plan.edge_chains,
            &mut plan.merged,
            &mut plan.point_to_global,
        )?;
        if let (Some(session), Some((key, boundary, start))) = (cache_session.as_mut(), capture) {
            session.capture_finish(
                face_id,
                key,
                boundary,
                &start,
                nurbs_frame_chart,
                &plan.merged,
                &plan.point_to_global,
            );
        }
        // Attribute every triangle appended by this face so `tri_faces` stays
        // parallel to the triangle list.
        if let Some(tf) = tri_faces.as_mut() {
            #[allow(clippy::cast_possible_truncation)]
            tf.resize(plan.merged.indices.len() / 3, fi as u32);
        }
    }

    drop(curved);
    if !matches!(boundary_mode, MeshBoundaryMode::ClosedMeasurement) {
        let normals = crate::performance::span("mesh.boundary-normals");
        let n_verts = plan.merged.positions.len();
        let tri_count = plan.merged.indices.len() / 3;

        let mut needs_normal = vec![false; n_verts];
        for i in 0..n_verts {
            let n = &plan.merged.normals[i];
            if n.x().abs() < 1e-30 && n.y().abs() < 1e-30 && n.z().abs() < 1e-30 {
                needs_normal[i] = true;
            }
        }

        {
            let mut vertex_faces: DetHashMap<usize, DetHashSet<FaceId>> = DetHashMap::default();
            for (&edge_idx, global_ids) in &plan.edge_chains {
                if let Some(face_ids) = edge_face_map.get(&edge_idx) {
                    for &gid in global_ids {
                        let gi = gid as usize;
                        if gi < n_verts && needs_normal[gi] {
                            let entry = vertex_faces.entry(gi).or_default();
                            for &fid in face_ids {
                                entry.insert(fid);
                            }
                        }
                    }
                }
            }

            let mut fallback_needed = vec![false; n_verts];
            for i in 0..n_verts {
                if !needs_normal[i] {
                    continue;
                }
                let pos = plan.merged.positions[i];
                let mut normal_sum = Vec3::new(0.0, 0.0, 0.0);
                let mut count = 0_u32;
                if let Some(faces) = vertex_faces.get(&i) {
                    for &fid in faces {
                        if let Ok(face_data) = topo.face(fid) {
                            #[allow(clippy::cast_possible_truncation)]
                            let gid = i as u32;
                            // A replayed face supplies its cached oriented
                            // contribution; the summation order is unchanged.
                            let cached = cache_session
                                .as_ref()
                                .map_or(face_cache::NormalSlot::Unknown, |session| {
                                    session.cached_normal(fid, gid)
                                });
                            let contribution = match cached {
                                face_cache::NormalSlot::Known(normal) => normal,
                                face_cache::NormalSlot::Unknown => {
                                    let surf = face_data.surface();
                                    let fresh = crate::fillet::face_surface_normal_at(surf, pos)
                                        .map(|n| {
                                            if face_data.is_reversed() {
                                                Vec3::new(-n.x(), -n.y(), -n.z())
                                            } else {
                                                n
                                            }
                                        });
                                    if let Some(session) = cache_session.as_mut() {
                                        session.record_normal(fid, gid, fresh);
                                    }
                                    fresh
                                }
                            };
                            if let Some(oriented) = contribution {
                                normal_sum += oriented;
                                count += 1;
                            }
                        }
                    }
                }
                if count > 0 {
                    plan.merged.normals[i] =
                        normal_sum.normalize().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
                } else {
                    fallback_needed[i] = true;
                }
            }

            if fallback_needed.iter().any(|&f| f) {
                let mut accum: Vec<Vec3> = vec![Vec3::new(0.0, 0.0, 0.0); n_verts];
                for t in 0..tri_count {
                    let i0 = plan.merged.indices[t * 3] as usize;
                    let i1 = plan.merged.indices[t * 3 + 1] as usize;
                    let i2 = plan.merged.indices[t * 3 + 2] as usize;
                    let a = plan.merged.positions[i1] - plan.merged.positions[i0];
                    let b = plan.merged.positions[i2] - plan.merged.positions[i0];
                    let face_normal = a.cross(b);
                    if fallback_needed.get(i0).copied().unwrap_or(false) {
                        accum[i0] += face_normal;
                    }
                    if fallback_needed.get(i1).copied().unwrap_or(false) {
                        accum[i1] += face_normal;
                    }
                    if fallback_needed.get(i2).copied().unwrap_or(false) {
                        accum[i2] += face_normal;
                    }
                }
                for i in 0..n_verts {
                    if fallback_needed[i] {
                        plan.merged.normals[i] =
                            accum[i].normalize().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
                    }
                }
            }
        }

        drop(normals);
    }
    let _post = crate::performance::span("mesh.weld-dedupe-gap-fill");
    if let Some(session) = cache_session {
        session.commit();
    }

    let closed = matches!(
        boundary_mode,
        MeshBoundaryMode::ClosedSolid | MeshBoundaryMode::ClosedMeasurement
    );
    let had_boundary =
        closed && weld_boundary_vertices(&mut plan.merged, plan.deflection, tri_faces.as_mut());

    // Drop coincident/cancelling triangles left by booleans that
    // produced overlapping coplanar faces (issue #696). Keyed on quantized
    // positions so position-coincident triangles with distinct vertex IDs
    // are still caught.
    let deduped = dedupe_coincident_triangles(&mut plan.merged, tri_faces.as_mut());
    // Gap fill only acts on boundary half-edges: a mesh the weld found
    // closed, and that dedupe left unchanged, has none (PERF-D07).
    if closed && (had_boundary || deduped) {
        fill_sub_deflection_triangular_gaps(&mut plan.merged, plan.deflection, tri_faces.as_mut());
    }

    Ok((plan.merged, tri_faces, all_faces.len()))
}

/// Split every triangle in `indices[start..]` whose edge spans a boundary
/// segment that a planar CDT job's constraint recovery Steiner-split.
///
/// `splits` maps a segment's undirected global pair `(lo, hi)` to the Steiner
/// vertices recovery placed on it, each with its parameter from `lo` towards
/// `hi`. Several jobs may split the same segment (at the same or different
/// points), so the points are merged into one ordered chain per segment, and
/// ANY triangle edge joining two chain members that are not adjacent in it —
/// the unsplit segment itself, or one half of a differently split one — is
/// replaced by a fan from the opposite vertex through the members between
/// them. The fan keeps the triangle's winding (each piece reuses the edge's
/// direction) and adds no vertex that is not already shared, and a point
/// strictly inside a segment is never collinear with the opposite vertex of
/// a non-degenerate triangle, so every piece has positive area.
///
/// `tri_faces`, when tracked, is the parallel per-triangle face list and the
/// offset of the same range in it; each piece inherits its triangle's face.
pub(super) fn split_triangles_spanning_boundary_splits(
    indices: &mut Vec<u32>,
    start: usize,
    tri_faces: Option<(&mut Vec<u32>, usize)>,
    splits: DetHashMap<(u32, u32), Vec<(f64, u32)>>,
) {
    // One ordered vertex chain per split segment, and each member's place in
    // the chains it belongs to.
    let mut chains: Vec<Vec<u32>> = Vec::with_capacity(splits.len());
    let mut member: DetHashMap<u32, Vec<(usize, usize)>> = DetHashMap::default();
    let mut keys: Vec<_> = splits.into_iter().collect();
    keys.sort_by_key(|&(key, _)| key);
    for ((lo, hi), mut points) in keys {
        points.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut chain = vec![lo];
        for (_, gid) in points {
            if !chain.contains(&gid) && gid != hi {
                chain.push(gid);
            }
        }
        chain.push(hi);
        let ci = chains.len();
        for (pos, &gid) in chain.iter().enumerate() {
            member.entry(gid).or_default().push((ci, pos));
        }
        chains.push(chain);
    }
    // The chain members strictly between `a` and `b`, ordered from `a`, when
    // `a`–`b` skips over at least one of them.
    let between = |a: u32, b: u32| -> Option<Vec<u32>> {
        let (ma, mb) = (member.get(&a)?, member.get(&b)?);
        for &(ca, pa) in ma {
            for &(cb, pb) in mb {
                if ca != cb || pa.abs_diff(pb) < 2 {
                    continue;
                }
                let chain = &chains[ca];
                return Some(if pa < pb {
                    chain[pa + 1..pb].to_vec()
                } else {
                    chain[pb + 1..pa].iter().rev().copied().collect()
                });
            }
        }
        None
    };

    let tri_start = start / 3;
    let tri_count = indices.len() / 3;
    let (mut faces_out, face_offset) = match &tri_faces {
        Some((_, offset)) => (Some(Vec::with_capacity(tri_count - tri_start)), *offset),
        None => (None, 0),
    };
    let mut indices_out: Vec<u32> = Vec::with_capacity(indices.len() - start);
    let mut stack: Vec<[u32; 3]> = Vec::new();
    for t in tri_start..tri_count {
        let face = tri_faces
            .as_ref()
            .and_then(|(tf, offset)| tf.get(offset + (t - tri_start)).copied());
        stack.push([indices[t * 3], indices[t * 3 + 1], indices[t * 3 + 2]]);
        // Each split strictly shortens the spanning edge's chain gap, so the
        // pieces of one triangle are bounded by the chain lengths; the cap is
        // a backstop against malformed input, not a tuning knob.
        let mut budget = 4 * chains.iter().map(Vec::len).sum::<usize>() + 8;
        while let Some(tri) = stack.pop() {
            let split = if budget == 0 {
                None
            } else {
                (0..3).find_map(|e| {
                    let (a, b, c) = (tri[e], tri[(e + 1) % 3], tri[(e + 2) % 3]);
                    between(a, b).map(|mids| (a, b, c, mids))
                })
            };
            if let Some((a, b, c, mids)) = split {
                budget -= 1;
                let mut prev = a;
                for &m in mids.iter().chain(std::iter::once(&b)) {
                    stack.push([prev, m, c]);
                    prev = m;
                }
            } else {
                indices_out.extend_from_slice(&tri);
                if let Some(out) = faces_out.as_mut() {
                    // Tracked lists are parallel by construction, so the face
                    // is always present; the default only keeps the two
                    // lists the same length if that ever breaks.
                    out.push(face.unwrap_or_default());
                }
            }
        }
    }
    indices.truncate(start);
    indices.extend(indices_out);
    if let (Some((tf, _)), Some(out)) = (tri_faces, faces_out) {
        tf.truncate(face_offset);
        tf.extend(out);
    }
}

/// Tessellate a single face, reusing shared edge vertices from the global mesh.
///
/// Returns whether the face was a NURBS face meshed by the CDT in its own
/// frame (`NurbsFrame`): the face mesh cache replays a translated copy of
/// such a face only when its capture took that path.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub(super) fn tessellate_face_with_shared_edges(
    topo: &Topology,
    face_id: FaceId,
    deflection: f64,
    angular_tol: f64,
    circle_floor: bool,
    allow_latitude_cap: bool,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<bool, crate::OperationsError> {
    let mut nurbs_frame_chart = false;
    let face_data = topo.face(face_id)?;
    let is_reversed = face_data.is_reversed();

    let idx_start = merged.indices.len();
    let pos_start = merged.positions.len();

    if let FaceSurface::Plane { normal, .. } = face_data.surface() {
        let normal = *normal;
        let wire = topo.wire(face_data.outer_wire())?;

        let mut boundary_global_ids: Vec<u32> = Vec::new();
        let tol = 1e-10;

        for oe in wire.edges() {
            let edge_idx = oe.edge().index();
            if let Some(global_ids) = edge_global_indices.get(&edge_idx) {
                let is_fwd = oe.is_forward();
                let len = global_ids.len();
                for j in 0..len {
                    let gid = if is_fwd {
                        global_ids[j]
                    } else {
                        global_ids[len - 1 - j]
                    };
                    if j == 0 && !boundary_global_ids.is_empty() {
                        let last_gid = *boundary_global_ids.last().unwrap_or(&u32::MAX);
                        if last_gid == gid {
                            continue;
                        }
                        if (last_gid as usize) < merged.positions.len()
                            && (gid as usize) < merged.positions.len()
                        {
                            let last_pos = merged.positions[last_gid as usize];
                            let this_pos = merged.positions[gid as usize];
                            if (last_pos - this_pos).length() < tol {
                                continue;
                            }
                        }
                    }
                    boundary_global_ids.push(gid);
                }
            } else {
                let edge_data = topo.edge(oe.edge())?;
                let points = sample_edge(topo, edge_data, deflection, angular_tol, circle_floor)?;
                let ordered: Vec<Point3> = if oe.is_forward() {
                    points
                } else {
                    points.into_iter().rev().collect()
                };
                for (j, pt) in ordered.iter().enumerate() {
                    if j == 0 && !boundary_global_ids.is_empty() {
                        let last_gid = *boundary_global_ids.last().unwrap_or(&u32::MAX);
                        if (last_gid as usize) < merged.positions.len() {
                            let last_pos = merged.positions[last_gid as usize];
                            if (last_pos - *pt).length() < tol {
                                continue;
                            }
                        }
                    }
                    let key = point_merge_key(*pt, MERGE_GRID);
                    let gid = point_to_global.entry(key).or_insert_with(|| {
                        #[allow(clippy::cast_possible_truncation)]
                        let idx = merged.positions.len() as u32;
                        merged.positions.push(*pt);
                        merged.normals.push(Vec3::new(0.0, 0.0, 0.0));
                        idx
                    });
                    boundary_global_ids.push(*gid);
                }
            }
        }

        remove_closing_duplicate_ids(&mut boundary_global_ids, &merged.positions, tol);

        let n = boundary_global_ids.len();
        if n < 3 {
            return Ok(false);
        }

        let local_positions: Vec<Point3> = boundary_global_ids
            .iter()
            .map(|&gid| merged.positions[gid as usize])
            .collect();

        if face_data.inner_wires().is_empty() {
            let mut local_indices = cdt_triangulate_simple(&local_positions, normal);

            if local_indices.len() >= 3 {
                let i0 = local_indices[0] as usize;
                let i1 = local_indices[1] as usize;
                let i2 = local_indices[2] as usize;
                let a = local_positions[i1] - local_positions[i0];
                let b = local_positions[i2] - local_positions[i0];
                let tri_normal = a.cross(b);
                if tri_normal.dot(normal) < 0.0 {
                    for t in 0..local_indices.len() / 3 {
                        local_indices.swap(t * 3 + 1, t * 3 + 2);
                    }
                }
            }

            for &li in &local_indices {
                merged.indices.push(boundary_global_ids[li as usize]);
            }
        } else {
            tessellate_planar_shared_with_holes(
                topo,
                face_data,
                &boundary_global_ids,
                &local_positions,
                normal,
                edge_global_indices,
                merged,
                point_to_global,
            )?;
        }
    } else if matches!(face_data.surface(), FaceSurface::Nurbs(_)) {
        // The converted-wall band mesher only fires on the GFA band wire
        // shape ([closed NURBS rim, Line, closed Circle, Line]); a converted
        // PRIMITIVE wall (two NURBS rims + two seam lines, no section) must
        // keep the established CDT path the seam-wall regression test pins.
        let is_gfa_band = remus_algo::wall_chart(face_data.surface()).is_some_and(|_| {
            topo.wire(face_data.outer_wire()).is_ok_and(|w| {
                w.edges().iter().any(|oe| {
                    topo.edge(oe.edge())
                        .is_ok_and(|e| matches!(e.curve(), EdgeCurve::Circle(_)))
                })
            })
        });
        let handled = (is_gfa_band
            && tessellate_converted_wall_band_shared(
                topo,
                face_data,
                edge_global_indices,
                merged,
            )?)
            || tessellate_nurbs_pole_cap_shared(
                topo,
                face_data,
                deflection,
                angular_tol,
                edge_global_indices,
                merged,
                point_to_global,
            )?
            || tessellate_nurbs_blend_band_shared(
                topo,
                face_data,
                deflection,
                angular_tol,
                edge_global_indices,
                merged,
                point_to_global,
            )?;
        if !handled {
            let pos_save = merged.positions.len();
            let nrm_save = merged.normals.len();
            let idx_save = merged.indices.len();
            let cdt_ok = tessellate_nonplanar_cdt(
                topo,
                face_id,
                face_data,
                deflection,
                angular_tol,
                circle_floor,
                edge_global_indices,
                merged,
                point_to_global,
            );
            nurbs_frame_chart = cdt_ok.is_ok() && merged.indices.len() > idx_save;
            if cdt_ok.is_err() || merged.indices.len() == idx_save {
                merged.positions.truncate(pos_save);
                merged.normals.truncate(nrm_save);
                merged.indices.truncate(idx_save);
                point_to_global.retain(|_, gid| (*gid as usize) < pos_save);
                if face_data.inner_wires().is_empty() {
                    tessellate_nonplanar_snap(
                        topo,
                        face_id,
                        face_data,
                        deflection,
                        angular_tol,
                        circle_floor,
                        edge_global_indices,
                        merged,
                        point_to_global,
                    )?;
                } else {
                    // A trimmed non-planar cap must never fall back to the
                    // rectangular surface mesh: that path ignores inner wires
                    // and would silently fill the opening.
                    cdt_ok?;
                }
            }
        }
    } else if matches!(
        face_data.surface(),
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
    ) {
        let (all_line_circle, band_eligible) = {
            let wire = topo.wire(face_data.outer_wire())?;
            let lc = wire.edges().iter().all(|oe| {
                topo.edge(oe.edge())
                    .is_ok_and(|e| matches!(e.curve(), EdgeCurve::Line | EdgeCurve::Circle(_)))
            });
            // The structured band also handles wavy mixed rims (winding-chain
            // separators carry marched-NURBS pieces); it verifies the cycle
            // structure itself and declines anything else.
            let be = lc
                || wire.edges().iter().all(|oe| {
                    topo.edge(oe.edge()).is_ok_and(|e| {
                        matches!(
                            e.curve(),
                            EdgeCurve::Line | EdgeCurve::Circle(_) | EdgeCurve::NurbsCurve(_)
                        ) || matches!(e.curve(),EdgeCurve::Ellipse(ellipse) if (ellipse.semi_major()-ellipse.semi_minor()).abs() <= ellipse.semi_major()*1e-10)
                    })
                });
            (lc, be)
        };
        let is_standard_rect =
            all_line_circle && topo.wire(face_data.outer_wire())?.edges().len() <= 4;

        // Prefer a structured band built from the shared rim vertices — it
        // is watertight by construction and avoids the snap path's proximity
        // reconciliation, which cracks drilled holes at certain radius/
        // deflection combos (issue #696). Tried for ANY Line/Circle wire, not
        // just the 4-edge canonical shape: a boolean can deliver a full band
        // whose rims are split into arc chains (cone∪box inscribed-rim), which
        // the band mesher now handles; it still returns false for anything
        // that is not a two-full-rim band.
        let band_handled = band_eligible
            && tessellate_revolution_band_shared(topo, face_data, edge_global_indices, merged)?;
        // A point-tipped cone has only one rim, so the two-rim band path
        // declines it. Preserve the fork's apex fan for the canonical
        // Line/Circle boundary; without it, odd rim sample counts crack the
        // cone at its base.
        let apex_handled = is_standard_rect
            && tessellate_cone_apex_fan_shared(topo, face_data, edge_global_indices, merged)?;

        if band_handled || apex_handled {
            // done — watertight structured band or apex fan emitted
        } else if is_standard_rect {
            // Partial (non-full-revolution) hole-free bands have a genuine
            // simple polygon UV boundary, so CDT over the shared pool ids
            // is watertight by construction. The snap path re-samples the
            // rim independently and cracks at fine deflections when its
            // segment count diverges from the pool's (the #696 class, seen
            // on gridfinity socket cone/cylinder corner rings). Faces with
            // inner wires keep the snap path, whose face mesh uses the
            // dedicated hole-aware cylindrical CDT before snapping every
            // outer and inner boundary vertex into this shared pool.
            let mut cdt_handled = false;
            if face_data.inner_wires().is_empty() {
                let pos_save = merged.positions.len();
                let nrm_save = merged.normals.len();
                let idx_save = merged.indices.len();
                let cdt_ok = tessellate_nonplanar_cdt(
                    topo,
                    face_id,
                    face_data,
                    deflection,
                    angular_tol,
                    circle_floor,
                    edge_global_indices,
                    merged,
                    point_to_global,
                );
                if cdt_ok.is_err() || merged.indices.len() == idx_save {
                    merged.positions.truncate(pos_save);
                    merged.normals.truncate(nrm_save);
                    merged.indices.truncate(idx_save);
                    // The CDT attempt may have registered merge-map entries
                    // for the now-truncated vertices; a later lookup would
                    // return a global id past `positions.len()`.
                    point_to_global.retain(|_, gid| (*gid as usize) < pos_save);
                } else {
                    cdt_handled = true;
                }
            }
            if !cdt_handled {
                tessellate_nonplanar_snap(
                    topo,
                    face_id,
                    face_data,
                    deflection,
                    angular_tol,
                    circle_floor,
                    edge_global_indices,
                    merged,
                    point_to_global,
                )?;
            }
        } else {
            let pos_save = merged.positions.len();
            let nrm_save = merged.normals.len();
            let idx_save = merged.indices.len();
            let cdt_ok = tessellate_nonplanar_cdt(
                topo,
                face_id,
                face_data,
                deflection,
                angular_tol,
                circle_floor,
                edge_global_indices,
                merged,
                point_to_global,
            );
            if cdt_ok.is_err() || merged.indices.len() == idx_save {
                merged.positions.truncate(pos_save);
                merged.normals.truncate(nrm_save);
                merged.indices.truncate(idx_save);
                // Same stale-merge-map hazard as the partial-band rollback
                // above: drop entries referencing the truncated vertices.
                point_to_global.retain(|_, gid| (*gid as usize) < pos_save);
                tessellate_nonplanar_snap(
                    topo,
                    face_id,
                    face_data,
                    deflection,
                    angular_tol,
                    circle_floor,
                    edge_global_indices,
                    merged,
                    point_to_global,
                )?;
            }
        }
    } else {
        // A sphere/torus latitude band (the annular region between two
        // constant-v full-revolution boundaries, e.g. a cylinder bored through a
        // sphere) degenerates in UV: each latitude projects to a zero-area
        // back-and-forth segment, so the CDT below cannot bound the band and
        // fills the removed polar cap. Tessellate such bands structurally from
        // the shared boundary vertices instead. Returns false for any other
        // sphere/torus face, which then takes the CDT/snap path unchanged.
        // A torus notch band (torus − box: a kept patch wrapping the tube fully,
        // bounded by two v-wrapping seam-arc loops at the ends of a ring-angle
        // span) is swept along u, not v, so it is not a latitude band. Try it
        // first; it returns false for any other torus face.
        let handled_notch = matches!(face_data.surface(), FaceSurface::Torus(_))
            && tessellate_torus_notch_band(
                topo,
                face_data,
                deflection,
                angular_tol,
                edge_global_indices,
                merged,
                point_to_global,
            )?;

        // A full-revolution torus band between two closed rims (an analytic
        // revolve's arc-profile wall, seamed by its doubled profile arc) is
        // structured from the shared rim vertices, like the cylinder/cone
        // standard band — CDT degenerates on its fully-u-wrapping UV image and
        // the snap path re-samples the rims into cracks.
        let handled_band = handled_notch
            || (matches!(face_data.surface(), FaceSurface::Torus(_))
                && tessellate_torus_two_rim_band(
                    topo,
                    face_data,
                    deflection,
                    angular_tol,
                    edge_global_indices,
                    merged,
                    point_to_global,
                )?)
            || (matches!(
                face_data.surface(),
                FaceSurface::Sphere(_) | FaceSurface::Torus(_)
            ) && tessellate_latitude_band_shared(
                topo,
                face_data,
                deflection,
                angular_tol,
                edge_global_indices,
                merged,
                point_to_global,
            )?)
            // A spherical vertex-blend cap (fillet corner ball) is filled as a
            // structured web from the shared rim samples: its boundary arcs
            // project to (near-)collinear UV polylines that break the CDT
            // below (zero-UV-area flap triangles, deflection-dependent
            // cracks). Returns false for any other sphere face.
            || (matches!(face_data.surface(), FaceSurface::Sphere(_))
                && tessellate_sphere_cap_shared(
                    topo,
                    face_data,
                    deflection,
                    angular_tol,
                    allow_latitude_cap,
                    edge_global_indices,
                    merged,
                    point_to_global,
                )?);

        if !handled_band {
            let pos_save = merged.positions.len();
            let nrm_save = merged.normals.len();
            let idx_save = merged.indices.len();
            let ptg_count_save = point_to_global.len();

            let cdt_ok = tessellate_nonplanar_cdt(
                topo,
                face_id,
                face_data,
                deflection,
                angular_tol,
                circle_floor,
                edge_global_indices,
                merged,
                point_to_global,
            );
            let cdt_produced_tris = cdt_ok.is_ok() && merged.indices.len() > idx_save;
            if !cdt_produced_tris {
                merged.positions.truncate(pos_save);
                merged.normals.truncate(nrm_save);
                merged.indices.truncate(idx_save);
                if point_to_global.len() > ptg_count_save {
                    point_to_global.retain(|_, v| (*v as usize) < pos_save);
                }

                tessellate_nonplanar_snap(
                    topo,
                    face_id,
                    face_data,
                    deflection,
                    angular_tol,
                    circle_floor,
                    edge_global_indices,
                    merged,
                    point_to_global,
                )?;
            }
        }
    }

    if is_reversed {
        let idx_end = merged.indices.len();
        let tri_count = (idx_end - idx_start) / 3;
        for t in 0..tri_count {
            let base = idx_start + t * 3;
            merged.indices.swap(base + 1, base + 2);
        }
        for n in &mut merged.normals[pos_start..] {
            *n = -*n;
        }
    }

    Ok(nurbs_frame_chart)
}

#[cfg(test)]
mod boundary_plan_pipeline_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::super::edge_sampling::{circle_param_range, sample_edge, sample_edge_with_params};
    use super::*;
    use remus_topology::Topology;

    /// Drilled-box fixture: top and bottom holed planes (two CDT jobs) plus
    /// a cylindrical wall, exercising the stage B/C path at pipeline level.
    fn drilled_box() -> (Topology, remus_topology::solid::SolidId) {
        use remus_math::mat::Mat4;

        let mut topo = Topology::new();
        let box_s = crate::primitives::make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
        let cyl = crate::primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
        crate::transform::transform_solid(&mut topo, cyl, &Mat4::translation(10.0, 10.0, -5.0))
            .unwrap();
        let result =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, box_s, cyl).unwrap();
        (topo, result)
    }

    /// Canonical triangle multiset for cross-order comparison: sorted vertex
    /// positions per triangle, then sorted triangles under a TOTAL
    /// lexicographic order over all nine coordinates. Positions are computed
    /// per job independently of face order, so equality is exact. (A partial
    /// comparator leaves ties in emission order and falsely reports
    /// differences.)
    fn canonical_triangles(mesh: &TriangleMesh) -> Vec<[[f64; 3]; 3]> {
        fn cmp_verts(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> std::cmp::Ordering {
            for (ra, rb) in a.iter().zip(b.iter()) {
                for (x, y) in ra.iter().zip(rb.iter()) {
                    let ord = x.total_cmp(y);
                    if ord != std::cmp::Ordering::Equal {
                        return ord;
                    }
                }
            }
            std::cmp::Ordering::Equal
        }
        let mut tris: Vec<[[f64; 3]; 3]> = mesh
            .indices
            .chunks_exact(3)
            .map(|t| {
                let mut verts = [t[0], t[1], t[2]].map(|i| {
                    let p = mesh.positions[i as usize];
                    [p.x(), p.y(), p.z()]
                });
                verts.sort_by(|a, b| {
                    a[0].total_cmp(&b[0])
                        .then_with(|| a[1].total_cmp(&b[1]))
                        .then_with(|| a[2].total_cmp(&b[2]))
                });
                verts
            })
            .collect();
        tris.sort_by(cmp_verts);
        tris
    }

    #[test]
    fn faces_core_is_stable_under_face_order_permutation() {
        use remus_topology::face::FaceId;

        let (topo, solid) = drilled_box();
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        assert!(
            faces.len() >= 3,
            "fixture needs several faces, got {}",
            faces.len()
        );
        let deflection = 0.1;
        let angular = remus_math::chord::DEFAULT_ANGULAR_TOL;

        let mut orders: Vec<Vec<FaceId>> = vec![faces.clone()];
        let mut reversed = faces.clone();
        reversed.reverse();
        orders.push(reversed);
        let mut rotated = faces.clone();
        rotated.rotate_left(faces.len() / 2 + 1);
        orders.push(rotated);
        let mut by_index_desc = faces;
        by_index_desc.sort_by_key(|f| std::cmp::Reverse(f.index()));
        orders.push(by_index_desc);

        let mut references: Vec<Vec<[[f64; 3]; 3]>> = Vec::new();
        for order in &orders {
            let (mesh, _, _) = tessellate_faces_core(
                &topo,
                order,
                deflection,
                angular,
                MeshBoundaryMode::ClosedSolid,
                false,
                false,
            )
            .unwrap();
            let bd = super::super::mesh_ops::boundary_edge_count(&mesh);
            let nm = super::super::mesh_ops::non_manifold_edge_count(&mesh);
            assert_eq!(
                (bd, nm),
                (0, 0),
                "permuted face order must stay watertight, got bd={bd} nm={nm}"
            );
            references.push(canonical_triangles(&mesh));
        }
        for (i, candidate) in references.iter().enumerate().skip(1) {
            assert_eq!(
                candidate, &references[0],
                "face order permutation {i} changed the triangle multiset"
            );
        }
    }

    #[test]
    fn faces_core_repeated_runs_are_byte_identical() {
        let (topo, solid) = drilled_box();
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        let deflection = 0.1;
        let angular = remus_math::chord::DEFAULT_ANGULAR_TOL;

        let (first, _, _) = tessellate_faces_core(
            &topo,
            &faces,
            deflection,
            angular,
            MeshBoundaryMode::ClosedSolid,
            false,
            false,
        )
        .unwrap();
        let (second, _, _) = tessellate_faces_core(
            &topo,
            &faces,
            deflection,
            angular,
            MeshBoundaryMode::ClosedSolid,
            false,
            false,
        )
        .unwrap();

        assert_eq!(
            first.indices, second.indices,
            "triangle emission order drifted"
        );
        assert_eq!(
            first.positions.len(),
            second.positions.len(),
            "vertex pool size drifted"
        );
        for (a, b) in first.positions.iter().zip(second.positions.iter()) {
            assert_eq!((a.x(), a.y(), a.z()), (b.x(), b.y(), b.z()));
        }
    }

    #[test]
    fn edge_sampling_with_params_preserves_points() {
        use remus_topology::edge::EdgeCurve;

        let mut topo = Topology::new();
        let box_s = crate::primitives::make_box(&mut topo, 4.0, 5.0, 6.0).unwrap();
        let cyl = crate::primitives::make_cylinder(&mut topo, 2.0, 8.0).unwrap();
        let sphere = crate::primitives::make_sphere(&mut topo, 3.0, 24).unwrap();
        let torus = crate::primitives::make_torus(&mut topo, 5.0, 1.0, 48).unwrap();
        let (drilled_topo, drilled) = drilled_box();

        let jobs: Vec<(&Topology, remus_topology::solid::SolidId)> = vec![
            (&topo, box_s),
            (&topo, cyl),
            (&topo, sphere),
            (&topo, torus),
            (&drilled_topo, drilled),
        ];

        for (topo, solid) in jobs {
            let edges = remus_topology::explorer::solid_edges(topo, solid).unwrap();
            assert!(!edges.is_empty());
            for &edge_id in &edges {
                let edge = topo.edge(edge_id).unwrap();
                let plain = sample_edge(topo, edge, 0.1, 0.5, false).unwrap();
                let (with_points, params) =
                    sample_edge_with_params(topo, edge, 0.1, 0.5, false).unwrap();
                assert_eq!(plain.len(), with_points.len());
                for (a, b) in plain.iter().zip(with_points.iter()) {
                    assert_eq!((a.x(), a.y(), a.z()), (b.x(), b.y(), b.z()));
                }
                assert_eq!(params.len(), with_points.len());
                // Endpoint parameters stay pinned to the authoritative
                // domain ends on curved edges.
                if let EdgeCurve::Circle(_) = edge.curve() {
                    let (t0, t1) = circle_param_range(edge).unwrap();
                    assert_eq!(params.first().copied().flatten(), Some(t0));
                    assert_eq!(params.last().copied().flatten(), Some(t1));
                }
            }
        }
    }
}
