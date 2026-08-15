//! Planar and simple analytic face tessellation.

use brepkit_math::det_hash::{DetHashMap, DetHashSet};
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;

use super::edge_sampling::{
    measure_max_chord_deviation, sample_wire_positions, segments_for_chord_deviation_a,
};
use super::shorter_arc_range;
use super::{AnalyticKind, MERGE_GRID, TriangleMesh, TriangleMeshUV, point_merge_key};

type CylinderUv = (f64, f64);
type CylinderSample = (Point3, CylinderUv);
type CylinderLoop = (Vec<Point3>, Vec<CylinderUv>);

/// Maximum number of interior samples added by the hole-aware cylinder grid.
///
/// This grid is only a quality aid for the CDT; bounding it prevents extreme
/// radius-to-height ratios from turning a compact face into unbounded work.
const MAX_CYLINDER_GRID_POINTS: usize = 1_000_000;

fn cylinder_grid_rows(
    physical_u_step: f64,
    v_span: f64,
    u_columns: usize,
) -> Result<usize, crate::OperationsError> {
    let rows = (v_span / physical_u_step.max(f64::EPSILON)).ceil().max(2.0);
    let max_rows = MAX_CYLINDER_GRID_POINTS
        .checked_div(u_columns)
        .unwrap_or(0)
        .saturating_add(1);
    #[allow(clippy::cast_precision_loss)]
    if !rows.is_finite() || rows > max_rows as f64 {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "cylindrical face tessellation grid exceeds the {MAX_CYLINDER_GRID_POINTS}-point work limit"
            ),
        });
    }
    Ok(rows as usize)
}

/// Tessellate a cylindrical face using its actual boundary polygon (CDT-based).
///
/// Used for faces with non-rectangular boundaries (e.g., boolean sub-faces
/// bounded by intersection curves). Projects the boundary POLYLINES to UV,
/// CDT-triangulates, then evaluates each vertex on the cylinder surface.
///
/// The boundary is the wire's tessellated polyline, not its corner vertices.
/// Reading one vertex per edge (what this did before) is only equivalent when
/// every boundary edge is straight, and the faces routed here are precisely the
/// ones whose edges are NOT: the caller sends a cylinder this way when its outer
/// wire carries a NURBS edge — a boolean intersection curve. A bore wall cut
/// clean through a shaft is bounded by ONE closed such curve, which collapsed to
/// a single point, fell under the three-point floor, and returned an EMPTY mesh:
/// the drilled hole rendered as nothing at all. (In `tessellate_solid` that only
/// showed at deflections coarse enough for `tessellate_nonplanar_cdt` to decline
/// the face first; the single-face `tessellate` path had no cover and drew
/// nothing at any deflection.)
///
/// Only the outer wire is read. Cylindrical faces with inner wires use the
/// dedicated hole-aware path below.
pub(super) fn tessellate_analytic_with_boundary(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    cyl: &brepkit_math::surfaces::CylindricalSurface,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMeshUV, crate::OperationsError> {
    // NOTE: Do NOT handle is_reversed here -- `tessellate_with_uvs` applies
    // a common reversal pass for all face types after this function returns.
    let wire = topo.wire(face_data.outer_wire())?;

    let mut uv_pts: Vec<(f64, f64)> = Vec::new();
    let mut positions_3d: Vec<Point3> = Vec::new();

    // Joint tolerance: the polylines of two consecutive edges repeat the vertex
    // between them, and a closed edge repeats its seam vertex at both ends.
    let tol_dup = brepkit_math::tolerance::Tolerance::default().linear;

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let pts = super::edge_sampling::sample_edge(topo, edge, deflection, angular_tol, false)?;
        let ordered: Vec<Point3> = if oe.is_forward() {
            pts
        } else {
            pts.into_iter().rev().collect()
        };
        for pos in ordered {
            if let Some(&last) = positions_3d.last()
                && (last - pos).length() < tol_dup
            {
                continue;
            }
            let (mut u, v) = cyl.project_point(pos);

            // Unwrap u to be continuous with the previous sample.
            if let Some(&(prev_u, _)) = uv_pts.last() {
                let diff = u - prev_u;
                let shifts = (diff / std::f64::consts::TAU + 0.5).floor();
                u -= shifts * std::f64::consts::TAU;
            }

            uv_pts.push((u, v));
            positions_3d.push(pos);
        }
    }

    // Drop the closing repeat of the first point, if the walk came back to it.
    if positions_3d.len() > 2
        && let (Some(&first), Some(&last)) = (positions_3d.first(), positions_3d.last())
        && (first - last).length() < tol_dup
    {
        positions_3d.pop();
        uv_pts.pop();
    }

    if uv_pts.len() < 3 {
        return Ok(TriangleMeshUV::default());
    }

    // Ensure CCW winding in UV (positive signed area).
    // For non-reversed cylinder faces, CCW UV -> outward normal.
    // If the polygon is CW (negative area), reverse to get CCW.
    {
        let mut signed_area = 0.0;
        for i in 0..uv_pts.len() {
            let j = (i + 1) % uv_pts.len();
            signed_area += uv_pts[i].0 * uv_pts[j].1 - uv_pts[j].0 * uv_pts[i].1;
        }
        if signed_area < 0.0 {
            uv_pts.reverse();
            positions_3d.reverse();
        }
    }

    let uv_p2: Vec<brepkit_math::vec::Point2> = uv_pts
        .iter()
        .map(|&(u, v)| brepkit_math::vec::Point2::new(u, v))
        .collect();
    let u_min = uv_pts.iter().map(|p| p.0).fold(f64::INFINITY, f64::min) - 1.0;
    let u_max = uv_pts.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max) + 1.0;
    let v_min = uv_pts.iter().map(|p| p.1).fold(f64::INFINITY, f64::min) - 1.0;
    let v_max = uv_pts.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max) + 1.0;
    let mut cdt = brepkit_math::cdt::Cdt::with_capacity(
        (
            brepkit_math::vec::Point2::new(u_min, v_min),
            brepkit_math::vec::Point2::new(u_max, v_max),
        ),
        uv_p2.len() + 4,
    );

    let n_verts = uv_p2.len();
    let cdt_ids: Vec<usize> = match cdt.insert_points_hilbert(&uv_p2) {
        Ok(ids) => ids,
        Err(_) => return Ok(TriangleMeshUV::default()),
    };

    // Insert boundary constraints -- only record successfully inserted ones
    // so remove_exterior doesn't rely on non-existent barriers.
    let mut boundary_segs = Vec::with_capacity(n_verts);
    for i in 0..n_verts {
        let j = (i + 1) % n_verts;
        if cdt.insert_constraint(cdt_ids[i], cdt_ids[j]).is_ok() {
            boundary_segs.push((cdt_ids[i], cdt_ids[j]));
        }
    }

    cdt.remove_exterior(&boundary_segs);

    let tris = cdt.triangles();
    let cdt_verts = cdt.vertices();

    let mut positions = Vec::with_capacity(cdt_verts.len());
    let mut normals_out = Vec::with_capacity(cdt_verts.len());
    let mut uvs = Vec::with_capacity(cdt_verts.len());
    for pt in cdt_verts {
        let u = pt.x();
        let v = pt.y();
        positions.push(cyl.evaluate(u, v));
        let nm = cyl.normal(u, 0.0);
        normals_out.push(nm);
        uvs.push([u, v]);
    }

    // Override boundary vertices with exact 3D positions from topology.
    for (i, &cdt_id) in cdt_ids.iter().enumerate() {
        if cdt_id < positions.len() {
            positions[cdt_id] = positions_3d[i];
        }
    }

    // Build index buffer (standard CCW winding -- reversal handled by caller).
    let mut indices = Vec::with_capacity(tris.len() * 3);
    #[allow(clippy::cast_possible_truncation)]
    for &(a, b, c) in &tris {
        indices.push(a as u32);
        indices.push(b as u32);
        indices.push(c as u32);
    }

    Ok(TriangleMeshUV {
        mesh: TriangleMesh {
            positions,
            normals: normals_out,
            indices,
        },
        uvs,
    })
}

/// Sample one cylindrical face wire into exact 3D boundary positions and a
/// continuous UV loop.
fn sample_cylinder_wire(
    topo: &Topology,
    wire: &brepkit_topology::wire::Wire,
    cyl: &brepkit_math::surfaces::CylindricalSurface,
    deflection: f64,
    angular_tol: f64,
) -> Result<CylinderLoop, crate::OperationsError> {
    let tol_dup = brepkit_math::tolerance::Tolerance::default().linear;
    let mut positions = Vec::new();
    let mut uvs: Vec<CylinderUv> = Vec::new();

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let pts = super::edge_sampling::sample_edge(topo, edge, deflection, angular_tol, false)?;
        let ordered: Vec<Point3> = if oe.is_forward() {
            pts
        } else {
            pts.into_iter().rev().collect()
        };
        for pos in ordered {
            if positions
                .last()
                .is_some_and(|last: &Point3| (*last - pos).length() < tol_dup)
            {
                continue;
            }
            let (mut u, v) = cyl.project_point(pos);
            if let Some(&(prev_u, _)) = uvs.last() {
                let shifts = ((u - prev_u) / std::f64::consts::TAU + 0.5).floor();
                u -= shifts * std::f64::consts::TAU;
            }
            positions.push(pos);
            uvs.push((u, v));
        }
    }

    if positions.len() > 2
        && let (Some(&first), Some(&last)) = (positions.first(), positions.last())
        && (first - last).length() < tol_dup
    {
        positions.pop();
        uvs.pop();
    }

    Ok((positions, uvs))
}

/// Clip a cylindrical UV polygon to one side of a vertical chart boundary.
/// Intersections are evaluated on the analytic surface so both periodic copies
/// of a seam crossing retain the same exact 3D position.
fn clip_cylinder_pocket(
    samples: &[CylinderSample],
    boundary_u: f64,
    keep_greater: bool,
    cyl: &brepkit_math::surfaces::CylindricalSurface,
) -> Vec<CylinderSample> {
    let Some(&last) = samples.last() else {
        return Vec::new();
    };
    let mut previous = last;
    let mut previous_inside = if keep_greater {
        previous.1.0 >= boundary_u
    } else {
        previous.1.0 <= boundary_u
    };
    let mut clipped = Vec::new();

    for &current in samples {
        let current_inside = if keep_greater {
            current.1.0 >= boundary_u
        } else {
            current.1.0 <= boundary_u
        };
        if current_inside != previous_inside {
            let t = (boundary_u - previous.1.0) / (current.1.0 - previous.1.0);
            let v = (current.1.1 - previous.1.1).mul_add(t, previous.1.1);
            clipped.push((cyl.evaluate(boundary_u, v), (boundary_u, v)));
        }
        if current_inside {
            clipped.push(current);
        }
        previous = current;
        previous_inside = current_inside;
    }

    let tol_dup = brepkit_math::tolerance::Tolerance::default().linear;
    clipped.dedup_by(|a, b| (a.0 - b.0).length() < tol_dup);
    if clipped.len() > 2
        && let (Some(first), Some(last)) = (clipped.first(), clipped.last())
        && (first.0 - last.0).length() < tol_dup
    {
        clipped.pop();
    }
    clipped
}

/// Copy a contractible inner loop into the outer cylinder chart. If it crosses
/// the periodic seam, clipping two adjacent copies produces the two pocket
/// polygons that meet at that seam instead of one polygon outside the chart.
fn cylinder_pocket_pieces(
    positions: &[Point3],
    uvs: &[CylinderUv],
    outer_u: (f64, f64),
    cyl: &brepkit_math::surfaces::CylindricalSurface,
) -> Vec<CylinderLoop> {
    if uvs.len() < 3 {
        return Vec::new();
    }
    let (inner_min, inner_max) = uvs
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &(u, _)| {
            (lo.min(u), hi.max(u))
        });
    let outer_mid = 0.5 * (outer_u.0 + outer_u.1);
    let inner_mid = 0.5 * (inner_min + inner_max);
    let base_turn = ((outer_mid - inner_mid) / std::f64::consts::TAU).round();
    let mut pieces = Vec::new();

    for turn in [base_turn - 1.0, base_turn, base_turn + 1.0] {
        let shift = turn * std::f64::consts::TAU;
        if inner_max + shift < outer_u.0 || inner_min + shift > outer_u.1 {
            continue;
        }
        let shifted: Vec<CylinderSample> = positions
            .iter()
            .copied()
            .zip(uvs.iter().copied())
            .map(|(position, (u, v))| (position, (u + shift, v)))
            .collect();
        let clipped = clip_cylinder_pocket(&shifted, outer_u.0, true, cyl);
        let clipped = clip_cylinder_pocket(&clipped, outer_u.1, false, cyl);
        if clipped.len() < 3 {
            continue;
        }
        let signed_area = (0..clipped.len())
            .map(|i| {
                let a = clipped[i].1;
                let b = clipped[(i + 1) % clipped.len()].1;
                a.0 * b.1 - b.0 * a.1
            })
            .sum::<f64>();
        if signed_area.abs() <= f64::EPSILON {
            continue;
        }
        pieces.push(clipped.into_iter().unzip());
    }
    pieces
}

/// Signed shortest-step winding around the cylinder's periodic coordinate.
fn cylinder_loop_winding(uvs: &[CylinderUv]) -> f64 {
    let tau = std::f64::consts::TAU;
    (0..uvs.len())
        .map(|i| {
            let delta = uvs[(i + 1) % uvs.len()].0 - uvs[i].0;
            delta - tau * ((delta + std::f64::consts::PI) / tau).floor()
        })
        .sum()
}

/// Return the `v` crossings of a positive-u, period-wrapping constraint at
/// `u`, including its periodic closing segment.
fn cylinder_band_crossings(uvs: &[CylinderUv], range: (usize, usize), u: f64) -> Vec<f64> {
    let points = &uvs[range.0..range.1];
    let Some(&(u0, _)) = points.first() else {
        return Vec::new();
    };
    let tau = std::f64::consts::TAU;
    let uq = u0 + (u - u0).rem_euclid(tau);

    let mut crossings = Vec::new();
    for i in 0..points.len() {
        let a = points[i];
        let b = if i + 1 < points.len() {
            points[i + 1]
        } else {
            (points[0].0 + tau, points[0].1)
        };
        if uq < a.0 || uq >= b.0 || b.0 <= a.0 {
            continue;
        }
        let t = (uq - a.0) / (b.0 - a.0);
        crossings.push((b.1 - a.1).mul_add(t, a.1));
    }
    crossings
}

/// Count crossings of a period-wrapping constraint above a UV point. An odd
/// total across all wrapping loops identifies a removed band.
fn cylinder_band_strands_above(uvs: &[CylinderUv], range: (usize, usize), u: f64, v: f64) -> usize {
    cylinder_band_crossings(uvs, range, u)
        .into_iter()
        .filter(|&crossing_v| crossing_v > v)
        .count()
}

/// Tessellate a cylindrical face with inner wires in the cylinder's developed
/// UV plane. All wire polylines become CDT constraints; inner-loop cells are
/// then removed before the vertices are mapped back to the analytic surface.
pub(super) fn tessellate_cylinder_with_holes(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    cyl: &brepkit_math::surfaces::CylindricalSurface,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMeshUV, crate::OperationsError> {
    use brepkit_math::cdt::Cdt;
    use brepkit_math::vec::Point2;

    let outer_wire = topo.wire(face_data.outer_wire())?;
    let (mut all_positions, outer_uvs) =
        sample_cylinder_wire(topo, outer_wire, cyl, deflection, angular_tol)?;
    if outer_uvs.len() < 3 {
        return Ok(TriangleMeshUV::default());
    }

    let outer_count = outer_uvs.len();
    let outer_u = outer_uvs
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &(u, _)| {
            (lo.min(u), hi.max(u))
        });
    let mut all_uvs = outer_uvs;
    let mut pocket_ranges = Vec::new();
    let mut band_ranges = Vec::new();

    for &wire_id in face_data.inner_wires() {
        let wire = topo.wire(wire_id)?;
        let (mut positions, mut uvs) =
            sample_cylinder_wire(topo, wire, cyl, deflection, angular_tol)?;
        let winding = cylinder_loop_winding(&uvs);
        let wraps = winding.abs() >= std::f64::consts::TAU - 1e-6;
        if wraps {
            if winding < 0.0 {
                positions.reverse();
                uvs.reverse();
            }
            let mut samples: Vec<CylinderSample> = positions
                .into_iter()
                .zip(uvs)
                .map(|(position, (u, v))| {
                    let u = outer_u.0 + (u - outer_u.0).rem_euclid(std::f64::consts::TAU);
                    (position, (u, v))
                })
                .collect();
            samples.sort_by(|a, b| a.1.0.total_cmp(&b.1.0));
            (positions, uvs) = samples.into_iter().unzip();
            if let (Some(&(first_u, first_v)), Some(&(last_u, last_v))) = (uvs.first(), uvs.last())
            {
                let period = std::f64::consts::TAU;
                let gap = first_u + period - last_u;
                if gap > 1e-12 {
                    let t = (outer_u.1 - last_u) / gap;
                    let seam_v = (first_v - last_v).mul_add(t, last_v);
                    let seam_position = cyl.evaluate(outer_u.0, seam_v);
                    positions.insert(0, seam_position);
                    uvs.insert(0, (outer_u.0, seam_v));
                    positions.push(seam_position);
                    uvs.push((outer_u.1, seam_v));
                }
            }
            let start = all_uvs.len();
            all_positions.extend(positions);
            all_uvs.extend(uvs);
            band_ranges.push((start, all_uvs.len()));
        } else {
            for (positions, uvs) in cylinder_pocket_pieces(&positions, &uvs, outer_u, cyl) {
                let start = all_uvs.len();
                all_positions.extend(positions);
                all_uvs.extend(uvs);
                pocket_ranges.push((start, all_uvs.len()));
            }
        }
    }

    let outer_v = all_uvs[..outer_count]
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &(_, v)| {
            (lo.min(v), hi.max(v))
        });
    let nu = segments_for_chord_deviation_a(
        cyl.radius(),
        outer_u.1 - outer_u.0,
        deflection,
        angular_tol,
        false,
    )
    // A drilled wall mixes positive shaft faces with subtractive bore faces.
    // One extra angular column keeps the aggregate signed-volume error below
    // the error of the limiting inscribed polygon without changing the caller's
    // deflection or angular tolerance.
    .saturating_add(1);
    for iu in 0..=nu {
        #[allow(clippy::cast_precision_loss)]
        let u = (outer_u.1 - outer_u.0).mul_add(iu as f64 / nu as f64, outer_u.0);
        let mut crossings = vec![outer_v.0, outer_v.1];
        for &range in &band_ranges {
            crossings.extend(cylinder_band_crossings(&all_uvs, range, u));
        }
        crossings.sort_by(f64::total_cmp);
        crossings.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
        for limits in crossings.windows(2) {
            let v = f64::midpoint(limits[0], limits[1]);
            let kept = band_ranges
                .iter()
                .map(|&range| cylinder_band_strands_above(&all_uvs, range, u, v))
                .sum::<usize>()
                .is_multiple_of(2);
            if kept {
                all_positions.push(cyl.evaluate(u, v));
                all_uvs.push((u, v));
            }
        }
    }

    // The crossing midpoints above guarantee that every retained vertical
    // strip has a CDT seed, but they do not control triangle width in `u`.
    // On a tall drilled cylinder, Delaunay can otherwise connect a hole rim to
    // a midpoint several angular columns away. Those long chords still form a
    // closed, consistently oriented shell, but flatten the analytic wall far
    // beyond the requested deflection and materially understate mesh/STL
    // volume. Add a regular developed-surface grid whose cells are roughly
    // square in physical units (`radius * du` by `dv`). Points inside removed
    // pockets or bands are harmless: the pocket-parity and band predicates
    // below remove their triangles.
    let physical_u_step = cyl.radius() * (outer_u.1 - outer_u.0) / nu as f64;
    let nv = cylinder_grid_rows(physical_u_step, outer_v.1 - outer_v.0, nu.saturating_sub(1))?;
    for iu in 1..nu {
        #[allow(clippy::cast_precision_loss)]
        let u = (outer_u.1 - outer_u.0).mul_add(iu as f64 / nu as f64, outer_u.0);
        for iv in 1..nv {
            #[allow(clippy::cast_precision_loss)]
            let v = (outer_v.1 - outer_v.0).mul_add(iv as f64 / nv as f64, outer_v.0);
            all_positions.push(cyl.evaluate(u, v));
            all_uvs.push((u, v));
        }
    }

    let radius = cyl.radius();
    let pts2d: Vec<Point2> = all_uvs
        .iter()
        .map(|&(u, v)| Point2::new(radius * u, v))
        .collect();
    let mut cdt = Cdt::with_capacity(compute_cdt_bounds(&pts2d), pts2d.len());
    let cdt_indices = cdt
        .insert_points_hilbert(&pts2d)
        .map_err(crate::OperationsError::Math)?;

    let mut add_loop_constraints = |start: usize, end: usize| {
        let count = end - start;
        for i in 0..count {
            let a = cdt_indices[start + i];
            let b = cdt_indices[start + (i + 1) % count];
            if a != b {
                cdt.insert_constraint(a, b)
                    .map_err(crate::OperationsError::Math)?;
            }
        }
        Ok::<(), crate::OperationsError>(())
    };
    add_loop_constraints(0, outer_count)?;
    for &(start, end) in &pocket_ranges {
        add_loop_constraints(start, end)?;
    }
    for &(start, end) in &band_ranges {
        for i in start..end.saturating_sub(1) {
            let a = cdt_indices[i];
            let b = cdt_indices[i + 1];
            if a != b {
                cdt.insert_constraint(a, b)
                    .map_err(crate::OperationsError::Math)?;
            }
        }
    }
    let outer_constraints: Vec<(usize, usize)> = (0..outer_count)
        .filter_map(|i| {
            let a = cdt_indices[i];
            let b = cdt_indices[(i + 1) % outer_count];
            (a != b).then_some((a, b))
        })
        .collect();
    cdt.remove_exterior(&outer_constraints);

    let triangles: Vec<(usize, usize, usize)> = cdt
        .triangles()
        .into_iter()
        .filter(|&(a, b, c)| {
            let pa = cdt.vertices()[a];
            let pb = cdt.vertices()[b];
            let pc = cdt.vertices()[c];
            let u = (pa.x() + pb.x() + pc.x()) / (3.0 * radius);
            let v = (pa.y() + pb.y() + pc.y()) / 3.0;
            let centroid = Point2::new(radius * u, v);
            let pocket_depth = pocket_ranges
                .iter()
                .filter(|&&(start, end)| {
                    brepkit_math::predicates::point_in_polygon(centroid, &pts2d[start..end])
                })
                .count();
            pocket_depth.is_multiple_of(2)
                && band_ranges
                    .iter()
                    .map(|&range| cylinder_band_strands_above(&all_uvs, range, u, v))
                    .sum::<usize>()
                    .is_multiple_of(2)
        })
        .collect();
    let cdt_vertices = cdt.vertices();
    let mut positions = Vec::with_capacity(cdt_vertices.len());
    let mut normals = Vec::with_capacity(cdt_vertices.len());
    let mut uvs = Vec::with_capacity(cdt_vertices.len());
    for uv in cdt_vertices {
        let u = uv.x() / radius;
        positions.push(cyl.evaluate(u, uv.y()));
        normals.push(cyl.normal(u, uv.y()));
        uvs.push([u, uv.y()]);
    }
    for (i, &cdt_id) in cdt_indices.iter().enumerate() {
        positions[cdt_id] = all_positions[i];
    }

    let mut indices = Vec::with_capacity(triangles.len() * 3);
    #[allow(clippy::cast_possible_truncation)]
    for (a, b, c) in triangles {
        indices.extend_from_slice(&[a as u32, b as u32, c as u32]);
    }

    Ok(TriangleMeshUV {
        mesh: TriangleMesh {
            positions,
            normals,
            indices,
        },
        uvs,
    })
}

#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines
)]
pub(super) fn tessellate_analytic(
    surface_eval: impl Fn(f64, f64) -> Point3,
    normal_fn: impl Fn(f64, f64) -> Vec3,
    u_range: (f64, f64),
    v_range: (f64, f64),
    nu: usize,
    nv: usize,
    kind: AnalyticKind,
) -> TriangleMeshUV {
    let nu = nu.max(4);
    let nv = nv.max(1);

    let num_verts = (nu + 1) * (nv + 1);
    let num_indices = nu * nv * 6;
    let mut positions = Vec::with_capacity(num_verts);
    let mut normals = Vec::with_capacity(num_verts);
    let mut uvs = Vec::with_capacity(num_verts);
    let mut indices = Vec::with_capacity(num_indices);

    // Only wrap u when the range spans a full period (approx 2pi).
    // For partial arcs (e.g. quarter-cylinder), the last grid column is a
    // distinct point that must NOT wrap back to the first column.
    let u_periodic = (u_range.1 - u_range.0 - std::f64::consts::TAU).abs()
        < brepkit_math::tolerance::Tolerance::new().linear;

    let mut grid = vec![0u32; (nu + 1) * (nv + 1)];
    for iv in 0..=nv {
        let v = v_range.0 + (v_range.1 - v_range.0) * (iv as f64) / (nv as f64);
        for iu in 0..=nu {
            let u = u_range.0 + (u_range.1 - u_range.0) * (iu as f64) / (nu as f64);
            let idx = positions.len() as u32;
            positions.push(surface_eval(u, v));
            normals.push(normal_fn(u, v));
            uvs.push([u, v]);
            grid[iv * (nu + 1) + iu] = idx;
        }
    }

    // Get grid index, wrapping u only for periodic seams (full circles).
    let gi = |iu: usize, iv: usize| -> u32 {
        let iu_w = if u_periodic && iu >= nu { 0 } else { iu };
        grid[iv * (nu + 1) + iu_w]
    };

    let v_min_degenerate = matches!(kind, AnalyticKind::SpherePole | AnalyticKind::ConeApex);
    let v_max_degenerate = matches!(kind, AnalyticKind::SpherePole | AnalyticKind::VMaxPole);

    for iv in 0..nv {
        let is_bottom = iv == 0;
        let is_top = iv == nv - 1;

        for iu in 0..nu {
            let i00 = gi(iu, iv);
            let i10 = gi(iu + 1, iv);
            let i01 = gi(iu, iv + 1);
            let i11 = gi(iu + 1, iv + 1);

            if is_bottom && v_min_degenerate {
                // Bottom pole/apex: triangle fan from the degenerate row.
                indices.push(i00);
                indices.push(i11);
                indices.push(i01);
            } else if is_top && v_max_degenerate {
                // Top pole: triangle fan from the degenerate row.
                indices.push(i00);
                indices.push(i10);
                indices.push(i01);
            } else {
                // Standard two-triangle quad.
                indices.push(i00);
                indices.push(i10);
                indices.push(i11);

                indices.push(i00);
                indices.push(i11);
                indices.push(i01);
            }
        }
    }

    TriangleMeshUV {
        mesh: TriangleMesh {
            positions,
            normals,
            indices,
        },
        uvs,
    }
}

/// Tessellate a planar face using CDT (Constrained Delaunay Triangulation).
///
/// Works for both convex and non-convex (simple) polygons by
/// projecting to 2D and using CDT with fan-triangulation fallback for degenerate cases.
pub(super) fn tessellate_planar(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    normal: Vec3,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    use brepkit_topology::edge::EdgeCurve;

    let wire = topo.wire(face_data.outer_wire())?;
    let mut positions = Vec::new();
    let tol = 1e-10;

    // Sample a parametric curve into `positions`, skipping consecutive duplicates.
    // `t_for_index(i)` maps a sample index to a parameter value, with `0` the
    // curve's natural start and `n_samples` its natural end.
    //
    // Half-open in TRAVERSAL order: emit the vertex the wire arrives at, stop
    // one step short of the vertex it leaves at (the next edge supplies that).
    // A forward edge therefore walks `0..n`; a REVERSED one walks `n..=1` —
    // `(0..n).rev()` starts one step INSIDE the arc and drops the vertex the
    // wire arrives at, leaving a chord from the previous edge's last sample
    // straight into the arc's interior.
    let sample_curve = |evaluate_fn: &dyn Fn(f64) -> Point3,
                        t_for_index: &dyn Fn(usize) -> f64,
                        n_samples: usize,
                        forward: bool,
                        positions: &mut Vec<Point3>| {
        let indices: Box<dyn Iterator<Item = usize>> = if forward {
            Box::new(0..n_samples)
        } else {
            // Reversed traversal walks t_end -> t_start; the [traversal
            // start, traversal end) convention therefore needs indices
            // n..=1, not (0..n).rev(): excluding t_end here dropped the
            // junction vertex with the PREVIOUS edge (nobody else supplies
            // it), and the CDT outline then shortcut the polygon corner
            // with a chord whose area bite scales with the neighbour
            // edge's length. t_start is excluded instead - the next edge
            // supplies it, same as the forward case.
            Box::new((1..=n_samples).rev())
        };
        for i in indices {
            #[allow(clippy::cast_precision_loss)]
            let t = t_for_index(i);
            let pt = evaluate_fn(t);
            if positions
                .last()
                .is_none_or(|p: &Point3| (*p - pt).length() > tol)
            {
                positions.push(pt);
            }
        }
    };

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        match edge.curve() {
            EdgeCurve::Circle(circle) => {
                let (t_start, t_end) = if edge.is_closed() {
                    (0.0, std::f64::consts::TAU)
                } else {
                    shorter_arc_range(circle, topo, edge)?
                };
                let arc_range = (t_end - t_start).abs();
                let n_samples = segments_for_chord_deviation_a(
                    circle.radius(),
                    arc_range,
                    deflection,
                    angular_tol,
                    false,
                );
                #[allow(clippy::cast_precision_loss)]
                sample_curve(
                    &|t| circle.evaluate(t),
                    &|i| t_start + (t_end - t_start) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            EdgeCurve::Ellipse(ellipse) => {
                let (t_start, t_end) = if edge.is_closed() {
                    (0.0, std::f64::consts::TAU)
                } else {
                    let sp = topo.vertex(edge.start())?.point();
                    let ep = topo.vertex(edge.end())?.point();
                    let ts = ellipse.project(sp);
                    let mut te = ellipse.project(ep);
                    if te <= ts {
                        te += std::f64::consts::TAU;
                    }
                    (ts, te)
                };
                let arc_range = t_end - t_start;
                // Largest radius of curvature (a^2/b) governs uniform-parameter
                // sampling density; matches the wall edge sampling so the cap
                // boundary stays watertight against the side faces.
                let max_curv_radius =
                    ellipse.semi_major() * ellipse.semi_major() / ellipse.semi_minor();
                let n_samples = segments_for_chord_deviation_a(
                    max_curv_radius,
                    arc_range,
                    deflection,
                    angular_tol,
                    true,
                );
                #[allow(clippy::cast_precision_loss)]
                sample_curve(
                    &|t| ellipse.evaluate(t),
                    &|i| t_start + (t_end - t_start) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            // Unbounded branches: `project` inverts the parameterization
            // exactly, so the arc is the straight parameter interval between
            // the two vertices. Density comes from the tightest osculating
            // circle on that span (see `open_conic_segments`), never a chord.
            EdgeCurve::Hyperbola(h) => {
                let sp = topo.vertex(edge.start())?.point();
                let ep = topo.vertex(edge.end())?.point();
                let (t0, t1) = (h.project(sp), h.project(ep));
                let n_samples = super::edge_sampling::open_conic_segments(
                    h.min_curvature_radius(t0, t1),
                    h.arc_length(t0, t1),
                    deflection,
                    angular_tol,
                );
                #[allow(clippy::cast_precision_loss)]
                sample_curve(
                    &|t| h.evaluate(t),
                    &|i| t0 + (t1 - t0) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            EdgeCurve::Parabola(p) => {
                let sp = topo.vertex(edge.start())?.point();
                let ep = topo.vertex(edge.end())?.point();
                let (t0, t1) = (p.project(sp), p.project(ep));
                let n_samples = super::edge_sampling::open_conic_segments(
                    p.min_curvature_radius(t0, t1),
                    p.arc_length(t0, t1),
                    deflection,
                    angular_tol,
                );
                #[allow(clippy::cast_precision_loss)]
                sample_curve(
                    &|t| p.evaluate(t),
                    &|i| t0 + (t1 - t0) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            EdgeCurve::NurbsCurve(nurbs) => {
                let (u0, u1) = nurbs.domain();
                let n_spans = nurbs
                    .control_points()
                    .len()
                    .saturating_sub(nurbs.degree())
                    .max(1);
                let coarse_n = (n_spans * 4).clamp(8, 128);
                let max_dev = measure_max_chord_deviation(nurbs, u0, u1, coarse_n);
                #[allow(clippy::cast_sign_loss)]
                let n_samples = if max_dev <= deflection {
                    coarse_n
                } else {
                    ((coarse_n as f64) * (max_dev / deflection).sqrt()).ceil() as usize
                }
                .clamp(8, 4096);
                let forward = oe.is_forward()
                    != super::edge_sampling::nurbs_runs_end_to_start(topo, edge, nurbs)?;
                #[allow(clippy::cast_precision_loss)]
                sample_curve(
                    &|t| nurbs.evaluate(t),
                    &|i| u0 + (u1 - u0) * (i as f64) / (n_samples as f64),
                    n_samples,
                    forward,
                    &mut positions,
                );
            }
            EdgeCurve::Line => {
                let vid = if oe.is_forward() {
                    edge.start()
                } else {
                    edge.end()
                };
                let pt = topo.vertex(vid)?.point();
                if positions
                    .last()
                    .is_none_or(|p: &Point3| (*p - pt).length() > tol)
                {
                    positions.push(pt);
                }
            }
        }
    }

    // Remove last point if it duplicates the first (closed wire).
    if positions.len() > 2
        && let (Some(first), Some(last)) = (positions.first(), positions.last())
        && (*last - *first).length() < tol
    {
        positions.pop();
    }

    let n = positions.len();
    if n < 3 {
        // Degenerate face (e.g. sliver from boolean) -- return empty mesh
        // rather than failing the entire solid tessellation.
        return Ok(TriangleMesh::default());
    }

    if face_data.inner_wires().is_empty() {
        let normals_out = vec![normal; n];
        let mut indices = cdt_triangulate_simple(&positions, normal);

        // Ensure triangle winding matches the face normal.
        // cdt_triangulate_simple forces CCW in 2D projection, which may
        // disagree with the face normal for faces whose normal opposes
        // the projection direction.
        if indices.len() >= 3 {
            let i0 = indices[0] as usize;
            let i1 = indices[1] as usize;
            let i2 = indices[2] as usize;
            let a = positions[i1] - positions[i0];
            let b = positions[i2] - positions[i0];
            let tri_normal = a.cross(b);
            if tri_normal.dot(normal) < 0.0 {
                for t in 0..indices.len() / 3 {
                    indices.swap(t * 3 + 1, t * 3 + 2);
                }
            }
        }

        Ok(TriangleMesh {
            positions,
            normals: normals_out,
            indices,
        })
    } else {
        tessellate_planar_with_holes(topo, face_data, &positions, normal, deflection, angular_tol)
    }
}

/// Project a 3D point to 2D by dropping the dominant normal axis.
pub(super) fn project_by_normal(p: Point3, normal: Vec3) -> brepkit_math::vec::Point2 {
    let ax = normal.x().abs();
    let ay = normal.y().abs();
    let az = normal.z().abs();
    if az >= ax && az >= ay {
        brepkit_math::vec::Point2::new(p.x(), p.y())
    } else if ay >= ax {
        brepkit_math::vec::Point2::new(p.x(), p.z())
    } else {
        brepkit_math::vec::Point2::new(p.y(), p.z())
    }
}

/// Compute an axis-aligned bounding box with margin for a set of 2D points.
pub(super) fn compute_cdt_bounds(
    pts2d: &[brepkit_math::vec::Point2],
) -> (brepkit_math::vec::Point2, brepkit_math::vec::Point2) {
    use brepkit_math::vec::Point2;

    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;
    for &p in pts2d {
        min_x = min_x.min(p.x());
        min_y = min_y.min(p.y());
        max_x = max_x.max(p.x());
        max_y = max_y.max(p.y());
    }
    let margin = ((max_x - min_x).max(max_y - min_y)) * 0.1 + 1e-6;
    (
        Point2::new(min_x - margin, min_y - margin),
        Point2::new(max_x + margin, max_y + margin),
    )
}

/// Tessellate a planar face with inner wires (holes) using CDT.
#[allow(clippy::too_many_lines)]
fn tessellate_planar_with_holes(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    outer_positions: &[Point3],
    normal: Vec3,
    deflection: f64,
    angular_tol: f64,
) -> Result<TriangleMesh, crate::OperationsError> {
    use brepkit_math::cdt::Cdt;
    use brepkit_math::vec::Point2;

    let mut all_positions: Vec<Point3> = outer_positions.to_vec();
    let outer_count = all_positions.len();
    let mut inner_wire_ranges: Vec<(usize, usize)> = Vec::new();

    let tol = 1e-10;
    for &iw_id in face_data.inner_wires() {
        let iw = topo.wire(iw_id)?;
        let inner_pts = sample_wire_positions(topo, iw, tol, deflection, angular_tol)?;
        let start = all_positions.len();
        all_positions.extend_from_slice(&inner_pts);
        let end = all_positions.len();
        inner_wire_ranges.push((start, end));
    }

    let pts2d: Vec<Point2> = all_positions
        .iter()
        .map(|&p| project_by_normal(p, normal))
        .collect();
    let bounds = compute_cdt_bounds(&pts2d);

    let mut cdt = Cdt::with_capacity(bounds, pts2d.len());

    // Insert all points (Hilbert-ordered for O(1) amortized locate).
    let cdt_indices = cdt
        .insert_points_hilbert(&pts2d)
        .map_err(crate::OperationsError::Math)?;

    let mut all_constraints: Vec<(usize, usize)> = Vec::new();
    for i in 0..outer_count {
        let j = (i + 1) % outer_count;
        let ci = cdt_indices[i];
        let cj = cdt_indices[j];
        if ci != cj {
            cdt.insert_constraint(ci, cj)
                .map_err(crate::OperationsError::Math)?;
            all_constraints.push((ci, cj));
        }
    }

    for &(start, end) in &inner_wire_ranges {
        let count = end - start;
        for i in 0..count {
            let j = (i + 1) % count;
            let ci = cdt_indices[start + i];
            let cj = cdt_indices[start + j];
            if ci != cj {
                cdt.insert_constraint(ci, cj)
                    .map_err(crate::OperationsError::Math)?;
                all_constraints.push((ci, cj));
            }
        }
    }

    let outer_constraints: Vec<(usize, usize)> = (0..outer_count)
        .filter_map(|i| {
            let j = (i + 1) % outer_count;
            let ci = cdt_indices[i];
            let cj = cdt_indices[j];
            (ci != cj).then_some((ci, cj))
        })
        .collect();
    cdt.remove_exterior(&outer_constraints);

    // Constraint edges drive the flood-fill stop condition below.
    let constraint_set: DetHashSet<(usize, usize)> = all_constraints
        .iter()
        .flat_map(|&(a, b)| {
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            [(lo, hi), (hi, lo)]
        })
        .collect();

    for seed in hole_removal_seeds(&pts2d, &inner_wire_ranges)? {
        let _removed = cdt.flood_remove_from_point(seed, &constraint_set);
    }

    let cdt_triangles = cdt.triangles();
    let cdt_verts = cdt.vertices();
    let num_tris = cdt_triangles.len();
    let mut positions_out = Vec::with_capacity(cdt_verts.len());
    let mut normals_out = Vec::with_capacity(cdt_verts.len());
    let mut indices_out = Vec::with_capacity(num_tris * 3);

    // Build O(1) reverse map: CDT vertex index -> original position index.
    let mut vi_to_orig: DetHashMap<usize, usize> = DetHashMap::default();
    for (orig_idx, &cdt_vi) in cdt_indices.iter().enumerate() {
        vi_to_orig.entry(cdt_vi).or_insert(orig_idx);
    }

    let mut cdt_to_mesh: DetHashMap<usize, u32> = DetHashMap::default();
    for &(v0, v1, v2) in &cdt_triangles {
        for &vi in &[v0, v1, v2] {
            if let std::collections::hash_map::Entry::Vacant(e) = cdt_to_mesh.entry(vi) {
                #[allow(clippy::cast_possible_truncation)]
                let mesh_idx = positions_out.len() as u32;
                if let Some(&orig_idx) = vi_to_orig.get(&vi) {
                    positions_out.push(all_positions[orig_idx]);
                } else {
                    // Steiner point inserted by CDT -- reconstruct 3D from 2D.
                    let p2d = cdt_verts[vi];
                    let p3d = unproject_point(p2d, normal, &all_positions[0]);
                    positions_out.push(p3d);
                }
                normals_out.push(normal);
                e.insert(mesh_idx);
            }
        }
    }

    for &(v0, v1, v2) in &cdt_triangles {
        let i0 = cdt_to_mesh[&v0];
        let i1 = cdt_to_mesh[&v1];
        let i2 = cdt_to_mesh[&v2];
        indices_out.push(i0);
        indices_out.push(i1);
        indices_out.push(i2);
    }

    // Ensure winding matches face normal.
    if indices_out.len() >= 3 {
        let i0 = indices_out[0] as usize;
        let i1 = indices_out[1] as usize;
        let i2 = indices_out[2] as usize;
        let a = positions_out[i1] - positions_out[i0];
        let b = positions_out[i2] - positions_out[i0];
        let tri_normal = a.cross(b);
        if tri_normal.dot(normal) < 0.0 {
            for t in 0..indices_out.len() / 3 {
                indices_out.swap(t * 3 + 1, t * 3 + 2);
            }
        }
    }

    Ok(TriangleMesh {
        positions: positions_out,
        normals: normals_out,
        indices: indices_out,
    })
}

/// Find a point guaranteed to be inside a simple polygon in 2D.
fn find_interior_seed(polygon: &[brepkit_math::vec::Point2]) -> brepkit_math::vec::Point2 {
    use brepkit_math::predicates::point_in_polygon;
    use brepkit_math::vec::Point2;

    let n = polygon.len();
    if n == 0 {
        return Point2::new(0.0, 0.0);
    }
    if n < 3 {
        return polygon[0];
    }

    for i in 0..n {
        let prev = polygon[(i + n - 1) % n];
        let curr = polygon[i];
        let next = polygon[(i + 1) % n];

        let e_prev = Point2::new(prev.x() - curr.x(), prev.y() - curr.y());
        let e_next = Point2::new(next.x() - curr.x(), next.y() - curr.y());

        let len_prev = (e_prev.x() * e_prev.x() + e_prev.y() * e_prev.y()).sqrt();
        let len_next = (e_next.x() * e_next.x() + e_next.y() * e_next.y()).sqrt();
        if len_prev < 1e-30 || len_next < 1e-30 {
            continue;
        }

        let u_prev = Point2::new(e_prev.x() / len_prev, e_prev.y() / len_prev);
        let u_next = Point2::new(e_next.x() / len_next, e_next.y() / len_next);
        let bisector = Point2::new(u_prev.x() + u_next.x(), u_prev.y() + u_next.y());
        let bis_len = (bisector.x() * bisector.x() + bisector.y() * bisector.y()).sqrt();
        if bis_len < 1e-30 {
            continue;
        }

        let step = 1e-4 * len_prev.min(len_next);
        let candidate = Point2::new(
            curr.x() + step * bisector.x() / bis_len,
            curr.y() + step * bisector.y() / bis_len,
        );

        if point_in_polygon(candidate, polygon) {
            return candidate;
        }

        let candidate_flip = Point2::new(
            curr.x() - step * bisector.x() / bis_len,
            curr.y() - step * bisector.y() / bis_len,
        );

        if point_in_polygon(candidate_flip, polygon) {
            return candidate_flip;
        }
    }

    let mut cx = 0.0;
    let mut cy = 0.0;
    for p in polygon {
        cx += p.x();
        cy += p.y();
    }
    cx /= n as f64;
    cy /= n as f64;
    Point2::new(cx, cy)
}

/// Seeds for the cells that must be flood-removed from a CDT of a face with
/// inner wires, one per wire that actually bounds a hole.
///
/// Inner wires can nest: an O-shaped bin's top face carries the cavity opening,
/// the island band around the central hole, and that hole itself. Nesting
/// alternates material and void, so only odd-depth wires bound a hole — taking
/// every inner wire as a hole erases the islands. Stored winding cannot be used
/// to tell them apart; a boolean can emit a hole wound like its outer.
///
/// Each seed sits just inside its own wire (never at its centroid, which
/// concentric wires share) so the flood starts in that wire's own cell.
pub(super) fn hole_removal_seeds(
    pts2d: &[brepkit_math::vec::Point2],
    inner_wire_ranges: &[(usize, usize)],
) -> Result<Vec<brepkit_math::vec::Point2>, crate::OperationsError> {
    use brepkit_math::predicates::point_in_polygon;
    use brepkit_math::vec::Point2;

    const PARENT_SEARCH_BUDGET_PER_WIRE: usize = 64;

    let polys: Vec<Vec<Point2>> = inner_wire_ranges
        .iter()
        .map(|&(start, end)| (start..end).map(|i| pts2d[i]).collect())
        .collect();
    let seeds: Vec<Option<Point2>> = polys
        .iter()
        .map(|p| (p.len() >= 3).then(|| find_interior_seed(p)))
        .collect();

    // One wire cannot nest, so skip the containment scan entirely — the common
    // case, and it keeps this off the hot path for ordinary single-hole faces.
    if polys.len() < 2 {
        return Ok(seeds.into_iter().flatten().collect());
    }

    // Bounds gate the winding-number tests: honeycomb faces carry dozens of
    // disjoint holes whose boxes never overlap, so the scan stays near-linear.
    let boxes: Vec<Option<(f64, f64, f64, f64)>> = polys
        .iter()
        .map(|p| {
            (p.len() >= 3).then(|| {
                p.iter().fold(
                    (f64::MAX, f64::MAX, f64::MIN, f64::MIN),
                    |(x0, y0, x1, y1), q| {
                        (x0.min(q.x()), y0.min(q.y()), x1.max(q.x()), y1.max(q.y()))
                    },
                )
            })
        })
        .collect();

    // Process smaller boxes first.  For valid, non-intersecting face wires the
    // first larger polygon containing a wire is its immediate parent.  This
    // turns concentric inputs from an all-pairs containment pass into one test
    // per wire.
    let mut order: Vec<usize> = (0..polys.len()).collect();
    order.sort_by(|&a, &b| {
        let area = |bbox: Option<(f64, f64, f64, f64)>| {
            bbox.map_or(f64::INFINITY, |(x0, y0, x1, y1)| (x1 - x0) * (y1 - y0))
        };
        area(boxes[a]).total_cmp(&area(boxes[b]))
    });

    // Bounding boxes can overlap without their wires nesting.  Bound those
    // rejected candidates as well, so a face made from many interlocking thin
    // boxes cannot restore quadratic CPU work.  Ordinary wires find their
    // parent immediately (or have disjoint bounds), leaving ample headroom.
    let mut search_budget = polys.len().saturating_mul(PARENT_SEARCH_BUDGET_PER_WIRE);
    let mut parents = vec![None; polys.len()];
    let mut unconditional = vec![false; polys.len()];
    for (position, &i) in order.iter().enumerate() {
        let Some(seed) = seeds[i] else { continue };
        // `find_interior_seed` falls back to the centroid when no vertex
        // bisector candidate lands inside, and that fallback can sit outside a
        // sufficiently degenerate wire. Depth measured from a point that is not
        // in its own wire is meaningless, so drop back to the pre-nesting
        // behaviour — flood from it unconditionally — rather than guess.
        if !point_in_polygon(seed, &polys[i]) {
            unconditional[i] = true;
            continue;
        }
        for &j in &order[position + 1..] {
            let Some((x0, y0, x1, y1)) = boxes[j] else {
                continue;
            };
            if seed.x() < x0 || seed.x() > x1 || seed.y() < y0 || seed.y() > y1 {
                continue;
            }
            if search_budget == 0 {
                return Err(crate::OperationsError::InvalidInput {
                    reason: "hole nesting exceeds the bounded containment budget".into(),
                });
            }
            search_budget -= 1;
            if point_in_polygon(seed, &polys[j]) {
                parents[i] = Some(j);
                break;
            }
        }
    }

    // Parents always have larger boxes, so reverse order computes each parent
    // depth before its children without another containment query.
    let mut depths = vec![1usize; polys.len()];
    let mut out = Vec::new();
    for &i in order.iter().rev() {
        if let Some(parent) = parents[i] {
            depths[i] = depths[parent] + 1;
        }
        if let Some(seed) = seeds[i]
            && (unconditional[i] || depths[i] % 2 == 1)
        {
            out.push(seed);
        }
    }
    Ok(out)
}

/// Reconstruct a 3D point from a 2D projection, using the face plane.
pub(super) fn unproject_point(
    p2d: brepkit_math::vec::Point2,
    normal: Vec3,
    reference: &Point3,
) -> Point3 {
    let ax = normal.x().abs();
    let ay = normal.y().abs();
    let az = normal.z().abs();

    let d = normal.x() * reference.x() + normal.y() * reference.y() + normal.z() * reference.z();
    if az >= ax && az >= ay {
        let z = (d - normal.x() * p2d.x() - normal.y() * p2d.y()) / normal.z();
        Point3::new(p2d.x(), p2d.y(), z)
    } else if ay >= ax {
        let y = (d - normal.x() * p2d.x() - normal.z() * p2d.y()) / normal.y();
        Point3::new(p2d.x(), y, p2d.y())
    } else {
        let x = (d - normal.y() * p2d.x() - normal.z() * p2d.y()) / normal.x();
        Point3::new(x, p2d.x(), p2d.y())
    }
}

/// Triangulate a simple polygon (no holes) in 3D using CDT.
pub(super) fn cdt_triangulate_simple(positions: &[Point3], normal: Vec3) -> Vec<u32> {
    use brepkit_math::cdt::Cdt;
    use brepkit_math::vec::Point2;

    let n = positions.len();
    if n < 3 {
        return vec![];
    }
    if n == 3 {
        return vec![0, 1, 2];
    }

    let pts2d: Vec<Point2> = positions
        .iter()
        .map(|&p| project_by_normal(p, normal))
        .collect();

    let bounds = compute_cdt_bounds(&pts2d);
    let mut cdt = Cdt::with_capacity(bounds, n);

    let cdt_indices = match cdt.insert_points_hilbert(&pts2d) {
        Ok(indices) => indices,
        Err(_) => return fan_triangulate(n),
    };

    let mut constraints = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let ci = cdt_indices[i];
        let cj = cdt_indices[j];
        if ci != cj {
            if cdt.insert_constraint(ci, cj).is_err() {
                return fan_triangulate(n);
            }
            constraints.push((ci, cj));
        }
    }

    cdt.remove_exterior(&constraints);

    let cdt_triangles = cdt.triangles();

    let mut cdt_to_input: DetHashMap<usize, usize> = DetHashMap::default();
    for (input_idx, &cdt_idx) in cdt_indices.iter().enumerate() {
        cdt_to_input.entry(cdt_idx).or_insert(input_idx);
    }

    let mut indices = Vec::with_capacity(cdt_triangles.len() * 3);
    let mut mapped = 0usize;
    for &(v0, v1, v2) in &cdt_triangles {
        if let (Some(&i0), Some(&i1), Some(&i2)) = (
            cdt_to_input.get(&v0),
            cdt_to_input.get(&v1),
            cdt_to_input.get(&v2),
        ) {
            mapped += 1;
            #[allow(clippy::cast_possible_truncation)]
            {
                indices.push(i0 as u32);
                indices.push(i1 as u32);
                indices.push(i2 as u32);
            }
        }
    }

    // A boundary constraint can only cross another constraint when the polygon
    // self-intersects (booleans occasionally emit a planar face whose outer
    // wire pinches through zero width — two boundary arcs that overlap by a few
    // hundred microns). CDT recovers crossing constraints by inserting a Steiner
    // vertex at the crossing; triangles touching it have no input-vertex mapping
    // and would be dropped here, leaving a hole. The Steiner vertex also splits
    // the shared boundary edges, which cracks against the neighbouring faces.
    // Fall back to a fan, which uses only the original boundary vertices and is
    // manifold by construction (each boundary edge used once, each diagonal
    // twice) regardless of the self-overlap.
    if mapped < cdt_triangles.len() || indices.is_empty() {
        return fan_triangulate(n);
    }

    indices
}

/// Fan triangulation as a last-resort fallback.
pub(super) fn fan_triangulate(n: usize) -> Vec<u32> {
    let mut indices = Vec::with_capacity((n - 2) * 3);
    for i in 1..n - 1 {
        #[allow(clippy::cast_possible_truncation)]
        {
            indices.push(0_u32);
            indices.push(i as u32);
            indices.push((i + 1) as u32);
        }
    }
    indices
}

/// Collect global vertex IDs from a wire, deduplicating consecutive vertices.
pub(super) fn collect_wire_global_vertices(
    wire: &brepkit_topology::wire::Wire,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    positions: &[Point3],
    tol: f64,
) -> (Vec<Point3>, Vec<Option<u32>>) {
    let mut out_positions: Vec<Point3> = Vec::new();
    let mut out_global_ids: Vec<Option<u32>> = Vec::new();

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
                if j == 0 && !out_global_ids.is_empty() {
                    let last_gid = out_global_ids.last().and_then(|g| *g).unwrap_or(u32::MAX);
                    if last_gid == gid {
                        continue;
                    }
                    if (last_gid as usize) < positions.len()
                        && (gid as usize) < positions.len()
                        && (positions[last_gid as usize] - positions[gid as usize]).length() < tol
                    {
                        continue;
                    }
                }
                out_positions.push(positions[gid as usize]);
                out_global_ids.push(Some(gid));
            }
        }
    }

    (out_positions, out_global_ids)
}

/// Remove the last element from parallel position/ID vectors if it duplicates
/// the first (closed wire loop-back).
pub(super) fn remove_closing_duplicate_global(
    positions: &mut Vec<Point3>,
    global_ids: &mut Vec<Option<u32>>,
    all_positions: &[Point3],
    tol: f64,
) {
    if global_ids.len() > 2
        && let (Some(&Some(first)), Some(&Some(last))) = (global_ids.first(), global_ids.last())
        && (first == last
            || ((first as usize) < all_positions.len()
                && (last as usize) < all_positions.len()
                && (all_positions[first as usize] - all_positions[last as usize]).length() < tol))
    {
        positions.pop();
        global_ids.pop();
    }
}

/// Remove the last element from a global ID list if it duplicates the first.
pub(super) fn remove_closing_duplicate_ids(ids: &mut Vec<u32>, positions: &[Point3], tol: f64) {
    if ids.len() > 2
        && let (Some(&first), Some(&last)) = (ids.first(), ids.last())
        && (first == last
            || ((first as usize) < positions.len()
                && (last as usize) < positions.len()
                && (positions[first as usize] - positions[last as usize]).length() < tol))
    {
        ids.pop();
    }
}

/// CDT tessellation for a planar face with inner wires, writing into a shared mesh.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub(super) fn tessellate_planar_shared_with_holes(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    boundary_global_ids: &[u32],
    outer_positions: &[Point3],
    normal: Vec3,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<(), crate::OperationsError> {
    use brepkit_math::cdt::Cdt;
    use brepkit_math::vec::Point2;

    let mut all_positions: Vec<Point3> = outer_positions.to_vec();
    let mut all_global_ids: Vec<Option<u32>> =
        boundary_global_ids.iter().map(|&g| Some(g)).collect();
    let outer_count = all_positions.len();
    let mut inner_wire_ranges: Vec<(usize, usize)> = Vec::new();

    let tol = 1e-10;
    for &iw_id in face_data.inner_wires() {
        let iw = topo.wire(iw_id)?;
        let start = all_positions.len();
        let (inner_pos, inner_gids) =
            collect_wire_global_vertices(iw, edge_global_indices, &merged.positions, tol);
        let mut inner_flat_ids: Vec<u32> = Vec::with_capacity(inner_gids.len());
        for (pos, gid_opt) in inner_pos.into_iter().zip(inner_gids) {
            if let Some(gid) = gid_opt {
                inner_flat_ids.push(gid);
                all_positions.push(pos);
                all_global_ids.push(Some(gid));
            } else {
                let key = point_merge_key(pos, MERGE_GRID);
                let gid = *point_to_global.entry(key).or_insert_with(|| {
                    #[allow(clippy::cast_possible_truncation)]
                    let idx = merged.positions.len() as u32;
                    merged.positions.push(pos);
                    merged.normals.push(normal);
                    idx
                });
                inner_flat_ids.push(gid);
                all_positions.push(pos);
                all_global_ids.push(Some(gid));
            }
        }
        if inner_flat_ids.len() > 2 {
            remove_closing_duplicate_ids(&mut inner_flat_ids, &merged.positions, tol);
            let expected_end = start + inner_flat_ids.len();
            all_positions.truncate(expected_end);
            all_global_ids.truncate(expected_end);
        }
        let end = all_positions.len();
        inner_wire_ranges.push((start, end));
    }

    let pts2d: Vec<Point2> = all_positions
        .iter()
        .map(|&p| project_by_normal(p, normal))
        .collect();
    let bounds = compute_cdt_bounds(&pts2d);

    let mut cdt = Cdt::with_capacity(bounds, pts2d.len());
    let cdt_indices = cdt
        .insert_points_hilbert(&pts2d)
        .map_err(crate::OperationsError::Math)?;

    let mut all_constraints: Vec<(usize, usize)> = Vec::new();
    for i in 0..outer_count {
        let j = (i + 1) % outer_count;
        let ci = cdt_indices[i];
        let cj = cdt_indices[j];
        if ci != cj {
            cdt.insert_constraint(ci, cj)
                .map_err(crate::OperationsError::Math)?;
            all_constraints.push((ci, cj));
        }
    }

    for &(start, end) in &inner_wire_ranges {
        let count = end - start;
        for i in 0..count {
            let j = (i + 1) % count;
            let ci = cdt_indices[start + i];
            let cj = cdt_indices[start + j];
            if ci != cj {
                cdt.insert_constraint(ci, cj)
                    .map_err(crate::OperationsError::Math)?;
                all_constraints.push((ci, cj));
            }
        }
    }

    let outer_constraints: Vec<(usize, usize)> = (0..outer_count)
        .filter_map(|i| {
            let j = (i + 1) % outer_count;
            let ci = cdt_indices[i];
            let cj = cdt_indices[j];
            (ci != cj).then_some((ci, cj))
        })
        .collect();
    cdt.remove_exterior(&outer_constraints);

    let constraint_set: DetHashSet<(usize, usize)> = all_constraints
        .iter()
        .flat_map(|&(a, b)| {
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            [(lo, hi), (hi, lo)]
        })
        .collect();

    for seed in hole_removal_seeds(&pts2d, &inner_wire_ranges)? {
        let _removed = cdt.flood_remove_from_point(seed, &constraint_set);
    }

    let cdt_triangles = cdt.triangles();

    let mut cdt_to_global: DetHashMap<usize, u32> = DetHashMap::default();
    for (local_idx, &cdt_idx) in cdt_indices.iter().enumerate() {
        if let Some(gid) = all_global_ids[local_idx] {
            cdt_to_global.insert(cdt_idx, gid);
        }
    }

    for &(v0, v1, v2) in &cdt_triangles {
        for &vi in &[v0, v1, v2] {
            if let std::collections::hash_map::Entry::Vacant(e) = cdt_to_global.entry(vi) {
                let p2d = cdt.vertices()[vi];
                let p3d = unproject_point(p2d, normal, &all_positions[0]);
                #[allow(clippy::cast_possible_truncation)]
                let gid = merged.positions.len() as u32;
                merged.positions.push(p3d);
                merged.normals.push(normal);
                e.insert(gid);
            }
        }
    }

    let needs_flip = if let Some(&(v0, v1, v2)) = cdt_triangles.first() {
        let g0 = cdt_to_global[&v0] as usize;
        let g1 = cdt_to_global[&v1] as usize;
        let g2 = cdt_to_global[&v2] as usize;
        let a = merged.positions[g1] - merged.positions[g0];
        let b = merged.positions[g2] - merged.positions[g0];
        a.cross(b).dot(normal) < 0.0
    } else {
        false
    };

    for &(v0, v1, v2) in &cdt_triangles {
        let g0 = cdt_to_global[&v0];
        let g1 = cdt_to_global[&v1];
        let g2 = cdt_to_global[&v2];
        if needs_flip {
            merged.indices.push(g0);
            merged.indices.push(g2);
            merged.indices.push(g1);
        } else {
            merged.indices.push(g0);
            merged.indices.push(g1);
            merged.indices.push(g2);
        }
    }

    Ok(())
}

/// Pure CDT computation for parallel execution across faces.
#[allow(clippy::too_many_lines)]
/// Triangles (indices into the input points, then into the Steiner list)
/// plus the Steiner points constraint recovery inserted.
pub(super) type PlanarCdtOutput = (Vec<(usize, usize, usize)>, Vec<brepkit_math::vec::Point2>);

pub(super) fn run_planar_cdt(
    pts2d: &[brepkit_math::vec::Point2],
    outer_count: usize,
    inner_wire_ranges: &[(usize, usize)],
) -> Result<PlanarCdtOutput, crate::OperationsError> {
    use brepkit_math::cdt::Cdt;

    let bounds = compute_cdt_bounds(pts2d);

    let mut cdt = Cdt::with_capacity(bounds, pts2d.len());
    let cdt_indices = cdt
        .insert_points_hilbert(pts2d)
        .map_err(crate::OperationsError::Math)?;

    let mut all_constraints: Vec<(usize, usize)> = Vec::new();
    for i in 0..outer_count {
        let j = (i + 1) % outer_count;
        let ci = cdt_indices[i];
        let cj = cdt_indices[j];
        if ci != cj {
            cdt.insert_constraint(ci, cj)
                .map_err(crate::OperationsError::Math)?;
            all_constraints.push((ci, cj));
        }
    }

    for &(start, end) in inner_wire_ranges {
        let count = end - start;
        for i in 0..count {
            let j = (i + 1) % count;
            let ci = cdt_indices[start + i];
            let cj = cdt_indices[start + j];
            if ci != cj {
                cdt.insert_constraint(ci, cj)
                    .map_err(crate::OperationsError::Math)?;
                all_constraints.push((ci, cj));
            }
        }
    }

    let outer_constraints: Vec<(usize, usize)> = (0..outer_count)
        .filter_map(|i| {
            let j = (i + 1) % outer_count;
            let ci = cdt_indices[i];
            let cj = cdt_indices[j];
            (ci != cj).then_some((ci, cj))
        })
        .collect();
    cdt.remove_exterior(&outer_constraints);

    let constraint_set: DetHashSet<(usize, usize)> = all_constraints
        .iter()
        .flat_map(|&(a, b)| {
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            [(lo, hi), (hi, lo)]
        })
        .collect();

    for seed in hole_removal_seeds(pts2d, inner_wire_ranges)? {
        let _removed = cdt.flood_remove_from_point(seed, &constraint_set);
    }

    let cdt_triangles = cdt.triangles();

    let mut cdt_to_input: DetHashMap<usize, usize> = DetHashMap::default();
    for (input_idx, &cdt_idx) in cdt_indices.iter().enumerate() {
        cdt_to_input.entry(cdt_idx).or_insert(input_idx);
    }

    // Constraint recovery may have inserted Steiner points (midpoint splits
    // of a constraint whose flip recovery stalled). Triangles touching them
    // must NOT be dropped — that leaves a fan-shaped hole in the face — so
    // Steiner vertices are appended after the input points and returned to
    // the caller for 3D lifting.
    let mut steiner: Vec<brepkit_math::vec::Point2> = Vec::new();
    let mut result = Vec::with_capacity(cdt_triangles.len());
    for &(v0, v1, v2) in &cdt_triangles {
        let mut ids = [0usize; 3];
        for (slot, &cv) in ids.iter_mut().zip([v0, v1, v2].iter()) {
            *slot = if let Some(&input_idx) = cdt_to_input.get(&cv) {
                input_idx
            } else {
                let idx = pts2d.len() + steiner.len();
                steiner.push(cdt.vertices()[cv]);
                cdt_to_input.insert(cv, idx);
                idx
            };
        }
        result.push((ids[0], ids[1], ids[2]));
    }

    Ok((result, steiner))
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::{MAX_CYLINDER_GRID_POINTS, cylinder_grid_rows};

    #[test]
    fn cylinder_grid_rows_accepts_bounded_grid() {
        assert_eq!(cylinder_grid_rows(0.5, 10.0, 18).unwrap(), 20);
    }

    #[test]
    fn cylinder_grid_rows_rejects_excessive_work() {
        let error = cylinder_grid_rows(1.0e-9, 1.0, 18).unwrap_err();
        assert!(
            error
                .to_string()
                .contains(&format!("{MAX_CYLINDER_GRID_POINTS}-point work limit"))
        );
    }
}

#[cfg(test)]
mod hole_seed_tests {
    use super::hole_removal_seeds;
    use brepkit_math::vec::Point2;

    #[test]
    fn concentric_wires_keep_even_odd_parity_at_scale() {
        let wire_count = 1_000;
        let mut points = Vec::with_capacity(wire_count * 3);
        let mut ranges = Vec::with_capacity(wire_count);
        for i in 0..wire_count {
            let radius = (wire_count - i) as f64;
            let start = points.len();
            points.extend([
                Point2::new(0.0, radius),
                Point2::new(-radius, -radius),
                Point2::new(radius, -radius),
            ]);
            ranges.push((start, points.len()));
        }

        let seeds = hole_removal_seeds(&points, &ranges).unwrap_or_default();

        assert_eq!(seeds.len(), wire_count / 2);
    }
}
