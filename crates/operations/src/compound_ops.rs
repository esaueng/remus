//! Operations on compound entities.
//!
//! Provides utilities for working with compounds of solids:
//! extracting individual solids, fusing all solids in a compound,
//! and computing compound-level measurements.

use brepkit_math::aabb::Aabb3;
use brepkit_topology::Topology;
use brepkit_topology::compound::CompoundId;
use brepkit_topology::solid::SolidId;

/// Extract all solid IDs from a compound.
///
/// # Errors
///
/// Returns an error if the compound ID is invalid.
pub fn explode(
    topo: &Topology,
    compound: CompoundId,
) -> Result<Vec<SolidId>, crate::OperationsError> {
    let comp = topo.compound(compound)?;
    Ok(comp.solids().to_vec())
}

/// Fuse (union) all solids in a compound into a single solid.
///
/// Performs iterative boolean union on all solids. Requires at least
/// one solid in the compound.
///
/// # Errors
///
/// Returns an error if the compound is empty or a boolean operation fails.
pub fn fuse_all(
    topo: &mut Topology,
    compound: CompoundId,
) -> Result<SolidId, crate::OperationsError> {
    let solids = {
        let comp = topo.compound(compound)?;
        comp.solids().to_vec()
    };

    if solids.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "compound has no solids to fuse".into(),
        });
    }

    // Partition solids into overlapping groups. Disjoint solids can be merged
    // directly (no boolean needed), while overlapping groups use boolean fuse.
    let bboxes: Vec<Aabb3> = solids
        .iter()
        .map(|&sid| crate::measure::solid_bounding_box(topo, sid))
        .collect::<Result<_, _>>()?;

    // Per-solid polyhedral bounds (plane normals + vertices), or `None` for any
    // solid with a curved face. Lets `partition_touching` prove that two solids
    // whose loose AABBs overlap are actually disjoint (e.g. honeycomb hex prisms
    // packed tighter than their corner-to-corner AABB extent), keeping them off
    // the expensive boolean path.
    let margin = brepkit_math::tolerance::Tolerance::new().linear;
    let poly_bounds: Vec<Option<PolyhedralBounds>> =
        solids.iter().map(|&s| polyhedral_bounds(topo, s)).collect();
    let cylinder_bounds: Vec<Option<SimpleCylinder>> =
        solids.iter().map(|&s| simple_cylinder(topo, s)).collect();

    let groups = partition_touching(&bboxes, &poly_bounds, &cylinder_bounds, margin);

    let mut group_results: Vec<SolidId> = Vec::new();
    for group in &groups {
        let group_solids: Vec<SolidId> = group.iter().map(|&i| solids[i]).collect();
        if group_solids.len() == 1 {
            group_results.push(crate::copy::copy_solid(topo, group_solids[0])?);
            continue;
        }
        if let Some(fused) = fuse_parallel_cylinder_cluster(topo, &group_solids)? {
            group_results.push(fused);
            continue;
        }
        // Each group is a connected cluster of interpenetrating solids, or of
        // solids whose precise relation is unavailable to the partitioner
        // — fuse it in ONE GFA arrangement (via `fuse_cluster`, N-way with a
        // sequential fallback) instead of a pairwise reduction that re-processes
        // a growing accumulator O(n²).
        group_results.push(crate::boolean::fuse_cluster(topo, &group_solids)?);
    }

    if group_results.len() == 1 {
        return Ok(group_results[0]);
    }

    merge_disjoint_solids(topo, &group_results)
}

#[derive(Clone)]
struct SimpleCylinder {
    center: brepkit_math::vec::Point3,
    axis: brepkit_math::vec::Vec3,
    radius: f64,
    axial_min: f64,
    axial_max: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CylinderRelation {
    Separated,
    Touching,
    PositiveOverlap,
}

/// Classify two simple cylinders by actual solid overlap, rather than center
/// distance alone. `None` means their axes are not parallel enough for this
/// exact classifier and the caller must use its conservative fallback.
fn cylinder_relation(
    a: &SimpleCylinder,
    b: &SimpleCylinder,
    margin: f64,
) -> Option<CylinderRelation> {
    if a.axis.dot(b.axis) < 1.0 - 1e-10 {
        return None;
    }
    let delta = b.center - a.center;
    let radial = (delta - a.axis * delta.dot(a.axis)).length();
    let radial_overlap = a.radius + b.radius - radial;
    let axial_overlap = a.axial_max.min(b.axial_max) - a.axial_min.max(b.axial_min);
    if radial_overlap < -margin || axial_overlap < -margin {
        Some(CylinderRelation::Separated)
    } else if radial_overlap <= margin || axial_overlap <= margin {
        Some(CylinderRelation::Touching)
    } else {
        Some(CylinderRelation::PositiveOverlap)
    }
}

fn point_axis_coordinate(point: brepkit_math::vec::Point3, axis: brepkit_math::vec::Vec3) -> f64 {
    point.x() * axis.x() + point.y() * axis.y() + point.z() * axis.z()
}

/// Recognize an untrimmed cylindrical prism (two planar caps and one analytic
/// cylindrical wall). This intentionally declines bores, partial cylinders,
/// and already-booleaned solids.
fn simple_cylinder(topo: &Topology, solid: SolidId) -> Option<SimpleCylinder> {
    use brepkit_topology::face::FaceSurface;

    let solid_data = topo.solid(solid).ok()?;
    if !solid_data.inner_shells().is_empty() {
        return None;
    }
    let faces = topo.shell(solid_data.outer_shell()).ok()?.faces();
    if faces.len() != 3 {
        return None;
    }
    let mut cylinder = None;
    let mut plane_count = 0;
    for &face_id in faces {
        let face = topo.face(face_id).ok()?;
        if !face.inner_wires().is_empty() {
            return None;
        }
        match face.surface() {
            FaceSurface::Cylinder(value) if cylinder.is_none() => cylinder = Some(value),
            FaceSurface::Plane { .. } => plane_count += 1,
            _ => return None,
        }
    }
    if plane_count != 2 {
        return None;
    }
    let cylinder = cylinder?;
    let axis = cylinder.axis().normalize().ok()?;
    let mut axial_min = f64::INFINITY;
    let mut axial_max = f64::NEG_INFINITY;
    for vertex_id in brepkit_topology::explorer::solid_vertices(topo, solid).ok()? {
        let value = point_axis_coordinate(topo.vertex(vertex_id).ok()?.point(), axis);
        axial_min = axial_min.min(value);
        axial_max = axial_max.max(value);
    }
    (axial_min.is_finite() && axial_max > axial_min).then(|| SimpleCylinder {
        center: cylinder.origin(),
        axis,
        radius: cylinder.radius(),
        axial_min,
        axial_max,
    })
}

#[derive(Clone, Copy)]
struct VisibleArc {
    center: (f64, f64),
    radius: f64,
    start: f64,
    end: f64,
}

/// Exact union for a connected cluster of equal-radius, co-oriented cylinders
/// with the same axial extent. Their 3D union is the extrusion of a 2D circle
/// union, whose boundary consists only of exact circular arcs.
fn fuse_parallel_cylinder_cluster(
    topo: &mut Topology,
    solids: &[SolidId],
) -> Result<Option<SolidId>, crate::OperationsError> {
    use brepkit_math::curves::Circle3D;
    use brepkit_math::vec::Vec3;
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::face::{Face, FaceSurface};
    use brepkit_topology::vertex::{Vertex, VertexId};
    use brepkit_topology::wire::{OrientedEdge, Wire};

    if solids.len() < 2 {
        return Ok(None);
    }
    let Some(first) = simple_cylinder(topo, solids[0]) else {
        return Ok(None);
    };
    let tol = brepkit_math::tolerance::Tolerance::new().linear;
    let mut cylinders = Vec::with_capacity(solids.len());
    cylinders.push(first.clone());
    for &solid in &solids[1..] {
        let Some(cylinder) = simple_cylinder(topo, solid) else {
            return Ok(None);
        };
        if cylinder.axis.dot(first.axis) < 1.0 - 1e-10
            || (cylinder.radius - first.radius).abs() > tol
            || (cylinder.axial_min - first.axial_min).abs() > tol
            || (cylinder.axial_max - first.axial_max).abs() > tol
        {
            return Ok(None);
        }
        cylinders.push(cylinder);
    }

    let u_axis = {
        let center_vector = Vec3::new(first.center.x(), first.center.y(), first.center.z());
        let radial = center_vector - first.axis * point_axis_coordinate(first.center, first.axis);
        radial.normalize().unwrap_or_else(|_| {
            first
                .axis
                .cross(Vec3::new(1.0, 0.0, 0.0))
                .normalize()
                .unwrap_or(Vec3::new(0.0, 1.0, 0.0))
        })
    };
    let v_axis = first.axis.cross(u_axis).normalize()?;
    let centers: Vec<(f64, f64)> = cylinders
        .iter()
        .map(|cylinder| {
            let delta = cylinder.center - first.center;
            (delta.dot(u_axis), delta.dot(v_axis))
        })
        .collect();
    let radius = first.radius;

    // Coincident pattern instances add no boundary. Keep one representative.
    let mut unique_centers = Vec::<(f64, f64)>::new();
    for center in centers {
        if unique_centers
            .iter()
            .all(|&(x, y)| (center.0 - x).hypot(center.1 - y) > tol)
        {
            unique_centers.push(center);
        }
    }
    if unique_centers.len() == 1 {
        return Ok(Some(solids[0]));
    }

    // This constructor is only for a positive-overlap connected component.
    // Tangent or separated cylinders remain distinct solids and are handled by
    // the caller's disjoint-shell path.
    let mut connected = vec![false; unique_centers.len()];
    connected[0] = true;
    loop {
        let mut changed = false;
        for i in 0..unique_centers.len() {
            if !connected[i] {
                continue;
            }
            for j in 0..unique_centers.len() {
                let distance = (unique_centers[i].0 - unique_centers[j].0)
                    .hypot(unique_centers[i].1 - unique_centers[j].1);
                if !connected[j] && distance < 2.0 * radius - tol {
                    connected[j] = true;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    if connected.iter().any(|&value| !value) {
        return Ok(None);
    }

    let mut arcs = Vec::<VisibleArc>::new();
    for (i, &(cx, cy)) in unique_centers.iter().enumerate() {
        let mut angles = vec![0.0, std::f64::consts::TAU];
        for (j, &(ox, oy)) in unique_centers.iter().enumerate() {
            if i == j {
                continue;
            }
            let dx = ox - cx;
            let dy = oy - cy;
            let distance = dx.hypot(dy);
            if distance <= tol || distance >= 2.0 * radius - tol {
                continue;
            }
            let center_angle = dy.atan2(dx);
            let half = (distance / (2.0 * radius)).clamp(-1.0, 1.0).acos();
            angles.push((center_angle - half).rem_euclid(std::f64::consts::TAU));
            angles.push((center_angle + half).rem_euclid(std::f64::consts::TAU));
        }
        angles.sort_by(f64::total_cmp);
        angles.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
        for pair in angles.windows(2) {
            let start = pair[0];
            let end = pair[1];
            if end - start <= 1e-12 {
                continue;
            }
            let mid = f64::midpoint(start, end);
            let sample = (cx + radius * mid.cos(), cy + radius * mid.sin());
            let visible = unique_centers.iter().enumerate().all(|(j, &(ox, oy))| {
                i == j || (sample.0 - ox).hypot(sample.1 - oy) >= radius - tol
            });
            if !visible {
                continue;
            }
            let pieces = ((end - start) / std::f64::consts::FRAC_PI_2)
                .ceil()
                .max(1.0) as usize;
            for piece in 0..pieces {
                let a = (end - start).mul_add(piece as f64 / pieces as f64, start);
                let b = (end - start).mul_add((piece + 1) as f64 / pieces as f64, start);
                arcs.push(VisibleArc {
                    center: (cx, cy),
                    radius,
                    start: a,
                    end: b,
                });
            }
        }
    }
    if arcs.is_empty() {
        return Ok(None);
    }

    let axial_origin = first.center
        + first.axis * (first.axial_min - point_axis_coordinate(first.center, first.axis));
    let point3 = |x: f64, y: f64| axial_origin + u_axis * x + v_axis * y;
    let key = |x: f64, y: f64| {
        #[allow(clippy::cast_possible_truncation)]
        ((x / tol).round() as i64, (y / tol).round() as i64)
    };
    let mut vertices = std::collections::HashMap::<(i64, i64), VertexId>::new();
    let mut arc_edges = Vec::with_capacity(arcs.len());
    for arc in &arcs {
        let start_xy = (
            arc.center.0 + arc.radius * arc.start.cos(),
            arc.center.1 + arc.radius * arc.start.sin(),
        );
        let end_xy = (
            arc.center.0 + arc.radius * arc.end.cos(),
            arc.center.1 + arc.radius * arc.end.sin(),
        );
        let start_vertex = *vertices
            .entry(key(start_xy.0, start_xy.1))
            .or_insert_with(|| topo.add_vertex(Vertex::new(point3(start_xy.0, start_xy.1), tol)));
        let end_vertex = *vertices
            .entry(key(end_xy.0, end_xy.1))
            .or_insert_with(|| topo.add_vertex(Vertex::new(point3(end_xy.0, end_xy.1), tol)));
        let circle = Circle3D::with_axes(
            point3(arc.center.0, arc.center.1),
            first.axis,
            arc.radius,
            u_axis,
            v_axis,
        )?;
        let edge = topo.add_edge(Edge::new(
            start_vertex,
            end_vertex,
            EdgeCurve::Circle(circle),
        ));
        arc_edges.push((start_vertex, end_vertex, edge, *arc));
    }

    let mut by_start = std::collections::HashMap::<usize, Vec<usize>>::new();
    for (index, (start, _, _, _)) in arc_edges.iter().enumerate() {
        by_start.entry(start.index()).or_default().push(index);
    }
    let mut unused = vec![true; arc_edges.len()];
    let mut loops = Vec::<(brepkit_topology::wire::WireId, f64)>::new();
    while let Some(first_index) = unused.iter().position(|&value| value) {
        let loop_start = arc_edges[first_index].0;
        let mut current = first_index;
        let mut oriented = Vec::new();
        let mut signed_area = 0.0;
        loop {
            if !unused[current] {
                return Ok(None);
            }
            unused[current] = false;
            let (_start, end, edge, arc) = arc_edges[current];
            oriented.push(OrientedEdge::new(edge, true));
            signed_area += 0.5
                * (arc.radius * arc.center.0 * (arc.end.sin() - arc.start.sin())
                    + arc.radius * arc.center.1 * (arc.start.cos() - arc.end.cos())
                    + arc.radius * arc.radius * (arc.end - arc.start));
            if end == loop_start {
                break;
            }
            let Some(candidates) = by_start.get(&end.index()) else {
                return Ok(None);
            };
            let Some(next) = candidates.iter().copied().find(|&index| unused[index]) else {
                return Ok(None);
            };
            current = next;
        }
        let wire = Wire::new(oriented, true)?;
        loops.push((topo.add_wire(wire), signed_area));
    }

    let Some((outer_index, _)) = loops
        .iter()
        .enumerate()
        .filter(|(_, (_, area))| *area > tol * tol)
        .max_by(|(_, (_, a)), (_, (_, b))| a.total_cmp(b))
    else {
        return Ok(None);
    };
    let outer_wire = loops[outer_index].0;
    let inner_wires: Vec<_> = loops
        .iter()
        .enumerate()
        .filter_map(|(index, &(wire, area))| {
            (index != outer_index && area < -tol * tol).then_some(wire)
        })
        .collect();
    if inner_wires.len() + 1 != loops.len() {
        return Ok(None);
    }
    let d = point_axis_coordinate(axial_origin, first.axis);
    let face = topo.add_face(Face::new(
        outer_wire,
        inner_wires,
        FaceSurface::Plane {
            normal: first.axis,
            d,
        },
    ));
    let fused = crate::extrude::extrude(topo, face, first.axis, first.axial_max - first.axial_min)?;
    Ok(Some(fused))
}

/// Count the total number of solids in a compound.
///
/// # Errors
///
/// Returns an error if the compound ID is invalid.
pub fn solid_count(topo: &Topology, compound: CompoundId) -> Result<usize, crate::OperationsError> {
    let comp = topo.compound(compound)?;
    Ok(comp.solids().len())
}

/// Compute the combined bounding box of all solids in a compound.
///
/// # Errors
///
/// Returns an error if the compound is empty or measurement fails.
pub fn compound_bounding_box(
    topo: &Topology,
    compound: CompoundId,
) -> Result<brepkit_math::aabb::Aabb3, crate::OperationsError> {
    let comp = topo.compound(compound)?;
    let solids = comp.solids();

    if solids.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "compound is empty".into(),
        });
    }

    let mut combined = crate::measure::solid_bounding_box(topo, solids[0])?;
    for &sid in &solids[1..] {
        let bb = crate::measure::solid_bounding_box(topo, sid)?;
        combined = combined.union(bb);
    }

    Ok(combined)
}

/// Union-find path-compressed lookup.
fn uf_find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// Plane normals (candidate separating axes) and boundary vertices of a
/// solid, used to prove disjointness via the separating-axis theorem.
struct PolyhedralBounds {
    normals: Vec<brepkit_math::vec::Vec3>,
    verts: Vec<brepkit_math::vec::Point3>,
}

/// Collect a solid's plane normals and vertices — but only if *every* outer-shell
/// face is planar. A flat-faced solid is contained in the convex hull of its
/// vertices, which makes the vertex-projection separation test (below) sound.
/// A single curved face can bulge past that hull, so any non-`Plane` face makes
/// this return `None` (the caller then falls back to the conservative AABB test).
fn polyhedral_bounds(topo: &Topology, sid: SolidId) -> Option<PolyhedralBounds> {
    use brepkit_topology::face::FaceSurface;

    let solid = topo.solid(sid).ok()?;
    let shell = topo.shell(solid.outer_shell()).ok()?;

    let mut normals = Vec::new();
    let mut vert_ids = std::collections::HashSet::new();
    for &fid in shell.faces() {
        let face = topo.face(fid).ok()?;
        match face.surface() {
            // Normalize: stored plane normals aren't guaranteed unit length (e.g.
            // raw STEP `DIRECTION` data), and `polyhedral_separated` compares
            // projection gaps against a world-space margin, which is only valid
            // for unit axes. Bail the whole solid to the AABB path on a
            // degenerate normal.
            FaceSurface::Plane { normal, .. } => normals.push(normal.normalize().ok()?),
            _ => return None,
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            let wire = topo.wire(wid).ok()?;
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge()).ok()?;
                if !matches!(edge.curve(), brepkit_topology::edge::EdgeCurve::Line) {
                    return None;
                }
                vert_ids.insert(edge.start());
                vert_ids.insert(edge.end());
            }
        }
    }

    let mut verts = Vec::with_capacity(vert_ids.len());
    for vid in vert_ids {
        verts.push(topo.vertex(vid).ok()?.point());
    }
    if verts.is_empty() {
        return None;
    }
    Some(PolyhedralBounds { normals, verts })
}

/// Whether two flat-faced solids are provably disjoint: `true` iff some face
/// normal of either separates their vertex projections by a clear `margin`.
///
/// Soundness: each solid lies within the convex hull of its vertices (all faces
/// planar), so a gap between the vertex projections on any axis is a real gap
/// between the solids. Only face-normal axes are tried (not edge-edge cross
/// products), so the test is sound but not complete — an undetected separation
/// just falls through to the boolean, never a false "disjoint" for touching
/// inputs.
fn polyhedral_separated(a: &PolyhedralBounds, b: &PolyhedralBounds, margin: f64) -> bool {
    let project = |verts: &[brepkit_math::vec::Point3], axis: &brepkit_math::vec::Vec3| {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for p in verts {
            let d = p.x() * axis.x() + p.y() * axis.y() + p.z() * axis.z();
            lo = lo.min(d);
            hi = hi.max(d);
        }
        (lo, hi)
    };
    a.normals.iter().chain(b.normals.iter()).any(|axis| {
        let (a_lo, a_hi) = project(&a.verts, axis);
        let (b_lo, b_hi) = project(&b.verts, axis);
        b_lo - a_hi > margin || a_lo - b_hi > margin
    })
}

/// Partition indices into groups that may actually touch (union-find).
///
/// Two solids share a group when their AABBs overlap *unless* both are flat-faced
/// and a separating axis proves a real gap between them. This keeps geometrically
/// disjoint pieces whose loose AABBs overlap (honeycomb hex prisms, tightly
/// packed feet) in separate groups, so `fuse_all` merges them via the cheap
/// disjoint-shell path instead of an O(n) chain of boolean unions.
fn partition_touching(
    bboxes: &[Aabb3],
    poly_bounds: &[Option<PolyhedralBounds>],
    cylinder_bounds: &[Option<SimpleCylinder>],
    margin: f64,
) -> Vec<Vec<usize>> {
    let n = bboxes.len();
    let mut parent: Vec<usize> = (0..n).collect();

    for i in 0..n {
        for j in (i + 1)..n {
            if !bboxes[i].intersects(bboxes[j]) {
                continue;
            }
            if let (Some(a), Some(b)) = (&cylinder_bounds[i], &cylinder_bounds[j])
                && cylinder_relation(a, b, margin) != Some(CylinderRelation::PositiveOverlap)
            {
                continue;
            }
            // AABBs overlap. Only keep them apart if we can *prove* a gap.
            if let (Some(pi), Some(pj)) = (&poly_bounds[i], &poly_bounds[j])
                && polyhedral_separated(pi, pj, margin)
            {
                continue;
            }
            let ri = uf_find(&mut parent, i);
            let rj = uf_find(&mut parent, j);
            if ri != rj {
                parent[ri] = rj;
            }
        }
    }

    let mut groups: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
    for i in 0..n {
        groups.entry(uf_find(&mut parent, i)).or_default().push(i);
    }
    groups.into_values().collect()
}

/// Merge disjoint solids into a single solid by combining all faces.
///
/// Note: the resulting outer shell contains disconnected face groups,
/// which technically violates the connected-shell invariant. This is
/// acceptable for volume measurement and tessellation (which iterate
/// faces independently), but algorithms that assume shell connectivity
/// should be aware. A future improvement would return a `Compound`.
///
/// The result references the input solids' existing faces (no deep copy),
/// so callers that need an independent result must pass copies.
pub(crate) fn merge_disjoint_solids(
    topo: &mut Topology,
    solids: &[SolidId],
) -> Result<SolidId, crate::OperationsError> {
    use brepkit_topology::shell::Shell;
    use brepkit_topology::solid::Solid;

    let mut all_faces = Vec::new();
    let mut inner_shell_ids = Vec::new();

    // Snapshot phase: collect all face IDs and inner shell face sets.
    let mut inner_face_sets: Vec<Vec<brepkit_topology::face::FaceId>> = Vec::new();
    for &sid in solids {
        let solid_data = topo.solid(sid)?;
        let outer_shell = topo.shell(solid_data.outer_shell())?;
        all_faces.extend_from_slice(outer_shell.faces());

        let inner_ids: Vec<_> = solid_data.inner_shells().to_vec();
        for inner_id in inner_ids {
            let inner_shell = topo.shell(inner_id)?;
            inner_face_sets.push(inner_shell.faces().to_vec());
        }
    }

    // Allocate phase: create inner shells.
    for faces in inner_face_sets {
        let inner = Shell::new(faces).map_err(crate::OperationsError::Topology)?;
        inner_shell_ids.push(topo.add_shell(inner));
    }

    let outer = Shell::new(all_faces).map_err(crate::OperationsError::Topology)?;
    let outer_id = topo.add_shell(outer);
    Ok(topo.add_solid(Solid::new(outer_id, inner_shell_ids)))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use brepkit_math::tolerance::Tolerance;
    use brepkit_topology::Topology;
    use brepkit_topology::compound::Compound;

    use super::*;

    #[test]
    fn explode_returns_solids() {
        let mut topo = Topology::new();
        let s1 = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let s2 = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let cid = topo.add_compound(Compound::new(vec![s1, s2]));

        let solids = explode(&topo, cid).unwrap();
        assert_eq!(solids.len(), 2);
    }

    #[test]
    fn solid_count_works() {
        let mut topo = Topology::new();
        let s1 = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let cid = topo.add_compound(Compound::new(vec![s1]));

        assert_eq!(solid_count(&topo, cid).unwrap(), 1);
    }

    #[test]
    fn compound_bbox() {
        let mut topo = Topology::new();
        let s1 = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let s2 = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        crate::transform::transform_solid(
            &mut topo,
            s2,
            &brepkit_math::mat::Mat4::translation(5.0, 0.0, 0.0),
        )
        .unwrap();

        let cid = topo.add_compound(Compound::new(vec![s1, s2]));
        let bb = compound_bounding_box(&topo, cid).unwrap();

        let tol = Tolerance::loose();
        // s1 is [0,1], s2 translated by 5 is [5,6]
        assert!(tol.approx_eq(bb.min.x(), 0.0));
        assert!(tol.approx_eq(bb.max.x(), 6.0));
    }

    #[test]
    fn fuse_all_two_overlapping_boxes() {
        let mut topo = Topology::new();
        let s1 = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let s2 = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        // Offset s2 slightly — overlapping boxes.
        crate::transform::transform_solid(
            &mut topo,
            s2,
            &brepkit_math::mat::Mat4::translation(0.5, 0.0, 0.0),
        )
        .unwrap();

        let cid = topo.add_compound(Compound::new(vec![s1, s2]));
        let fused = fuse_all(&mut topo, cid).unwrap();

        let vol = crate::measure::solid_volume(&topo, fused, 0.1).unwrap();
        // Two overlapping unit cubes: total should be less than 2.0.
        assert!(
            vol > 1.0 && vol < 2.0,
            "fused volume should be between 1 and 2, got {vol}"
        );
    }

    /// A connected chain of four overlapping unit cubes forms ONE cluster, so
    /// `fuse_all` fuses it via the N-way path. The union is a solid
    /// [0,2.5]×[0,1]×[0,1] bar, so the result must be watertight with volume 2.5.
    #[test]
    fn fuse_all_connected_cluster_is_watertight_bar() {
        use brepkit_math::mat::Mat4;

        let offsets = [0.0, 0.5, 1.0, 1.5];

        // fuse_all (N-way) path.
        let mut topo = Topology::new();
        let boxes: Vec<SolidId> = offsets
            .iter()
            .map(|&dx| {
                let b = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
                crate::transform::transform_solid(&mut topo, b, &Mat4::translation(dx, 0.0, 0.0))
                    .unwrap();
                b
            })
            .collect();
        let cid = topo.add_compound(Compound::new(boxes));
        let fused = fuse_all(&mut topo, cid).unwrap();
        let vol = crate::measure::solid_volume(&topo, fused, 0.01).unwrap();

        // Every edge of a watertight solid is used by exactly two faces.
        let mut uses: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
        for fid in brepkit_topology::explorer::solid_faces(&topo, fused).unwrap() {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    *uses.entry(oe.edge().index()).or_default() += 1;
                }
            }
        }
        assert!(
            uses.values().all(|&c| c == 2),
            "fuse_all cluster result must be watertight"
        );
        assert!(
            (vol - 2.5).abs() < 0.01,
            "union of the overlapping row is a [0,2.5] bar (vol 2.5), got {vol}"
        );
    }

    /// Build a hexagonal prism (flat top at z=0..h) centred at the origin via
    /// convex hull — a polyhedral stand-in for a honeycomb pocket.
    fn make_hex_prism(topo: &mut Topology, circumradius: f64, height: f64) -> SolidId {
        use brepkit_math::vec::Point3;
        let mut pts = Vec::with_capacity(12);
        for k in 0..6 {
            let a = std::f64::consts::PI / 3.0 * k as f64;
            let (x, y) = (circumradius * a.cos(), circumradius * a.sin());
            pts.push(Point3::new(x, y, 0.0));
            pts.push(Point3::new(x, y, height));
        }
        crate::primitives::make_convex_hull(topo, &pts).unwrap()
    }

    /// Honeycomb-packed hex prisms with a real gap between every pair, but
    /// corner-to-corner AABBs that overlap. The AABB-only partition collapsed
    /// these into one giant group and unioned them with an O(n) boolean chain;
    /// `partition_touching` proves the gaps with the separating-axis test and
    /// keeps each prism in its own group, so `fuse_all` takes the cheap
    /// disjoint-shell merge.
    #[test]
    fn fuse_all_honeycomb_stays_disjoint() {
        let r = 1.0_f64; // circumradius; across-corners = 2r = 2.0
        let pitch = 2.3_f64; // clear gap on every neighbour, AABBs still overlap
        let height = 4.0_f64;
        let nx = 6;
        let ny = 6;

        let mut topo = Topology::new();
        let mut bboxes = Vec::new();
        let mut solids = Vec::new();
        for j in 0..ny {
            for i in 0..nx {
                let s = make_hex_prism(&mut topo, r, height);
                let x = i as f64 * pitch + (j % 2) as f64 * pitch / 2.0;
                let y = j as f64 * pitch * 0.9;
                crate::transform::transform_solid(
                    &mut topo,
                    s,
                    &brepkit_math::mat::Mat4::translation(x, y, 0.0),
                )
                .unwrap();
                bboxes.push(crate::measure::solid_bounding_box(&topo, s).unwrap());
                solids.push(s);
            }
        }
        let n = solids.len();

        // Every prism is provably disjoint from the others -> one group each.
        let margin = brepkit_math::tolerance::Tolerance::new().linear;
        let pb: Vec<Option<PolyhedralBounds>> = solids
            .iter()
            .map(|&s| polyhedral_bounds(&topo, s))
            .collect();
        let cylinders: Vec<Option<SimpleCylinder>> =
            solids.iter().map(|&s| simple_cylinder(&topo, s)).collect();
        let groups = partition_touching(&bboxes, &pb, &cylinders, margin);
        assert_eq!(
            groups.len(),
            n,
            "disjoint hex prisms should each be their own group, got {} groups",
            groups.len()
        );

        // Geometry is still the full disjoint union: volume == n * hex-prism volume.
        let cid = topo.add_compound(Compound::new(solids));
        let fused = fuse_all(&mut topo, cid).unwrap();
        let vol = crate::measure::solid_volume(&topo, fused, 0.05).unwrap();
        let hex_area = 3.0_f64.sqrt() * 1.5 * r * r; // (3*sqrt(3)/2) r^2
        let expected = n as f64 * hex_area * height;
        assert!(
            (vol - expected).abs() < expected * 0.02,
            "fused volume {vol:.2} should match {expected:.2} (n disjoint prisms)"
        );
    }

    fn assert_cylinder_union_health(topo: &Topology, solid: SolidId) {
        use brepkit_topology::face::FaceSurface;

        let validation = crate::validate::validate_solid(topo, solid).unwrap();
        assert!(
            validation.is_valid(),
            "overlapping cylinder union must be a valid solid: {:?}",
            validation.issues
        );
        let faces = brepkit_topology::explorer::solid_faces(topo, solid).unwrap();
        assert!(
            faces.iter().all(|&face_id| matches!(
                topo.face(face_id).unwrap().surface(),
                FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
            )),
            "exact cylinder union must retain analytic planes and cylinders"
        );
        let mesh = crate::tessellate::tessellate_solid(topo, solid, 0.05).unwrap();
        let quality = crate::tessellate::welded_mesh_quality(&mesh);
        assert_eq!(
            (quality.boundary_edges, quality.non_manifold_edges),
            (0, 0),
            "overlapping cylinder union mesh must be closed and manifold"
        );
    }

    fn circle_lens_area(radius: f64, distance: f64) -> f64 {
        if distance >= 2.0 * radius {
            return 0.0;
        }
        2.0 * radius * radius * (distance / (2.0 * radius)).acos()
            - 0.5 * distance * (4.0 * radius * radius - distance * distance).sqrt()
    }

    fn numeric_circle_union_area(centers: &[(f64, f64)], radius: f64) -> f64 {
        let x_min = centers
            .iter()
            .map(|&(x, _)| x - radius)
            .fold(f64::INFINITY, f64::min);
        let x_max = centers
            .iter()
            .map(|&(x, _)| x + radius)
            .fold(f64::NEG_INFINITY, f64::max);
        let slices = 100_000_usize;
        let dx = (x_max - x_min) / slices as f64;
        let union_height = |x: f64| {
            let mut intervals: Vec<(f64, f64)> = centers
                .iter()
                .filter_map(|&(cx, cy)| {
                    let square = radius * radius - (x - cx) * (x - cx);
                    (square > 0.0).then(|| {
                        let half = square.sqrt();
                        (cy - half, cy + half)
                    })
                })
                .collect();
            intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
            let Some(&(start, end)) = intervals.first() else {
                return 0.0;
            };
            let mut total = 0.0;
            let (mut lo, mut hi) = (start, end);
            for &(next_lo, next_hi) in &intervals[1..] {
                if next_lo > hi {
                    total += hi - lo;
                    (lo, hi) = (next_lo, next_hi);
                } else {
                    hi = hi.max(next_hi);
                }
            }
            total + hi - lo
        };
        let mut weighted = union_height(x_min) + union_height(x_max);
        for i in 1..slices {
            let weight = if i.is_multiple_of(2) { 2.0 } else { 4.0 };
            weighted += weight * union_height((i as f64).mul_add(dx, x_min));
        }
        weighted * dx / 3.0
    }

    #[test]
    fn simple_cylinders_classify_separated_touching_and_overlap() {
        use brepkit_math::mat::Mat4;

        let mut topo = Topology::new();
        let make_at = |topo: &mut Topology, x: f64| {
            let solid = crate::primitives::make_cylinder(topo, 5.0, 10.0).unwrap();
            crate::transform::transform_solid(topo, solid, &Mat4::translation(x, 0.0, 0.0))
                .unwrap();
            simple_cylinder(topo, solid).unwrap()
        };
        let origin = make_at(&mut topo, 0.0);
        let separated = make_at(&mut topo, 10.1);
        let touching = make_at(&mut topo, 10.0);
        let overlapping = make_at(&mut topo, 9.9);
        let margin = brepkit_math::tolerance::Tolerance::new().linear;
        assert_eq!(
            cylinder_relation(&origin, &separated, margin),
            Some(CylinderRelation::Separated)
        );
        assert_eq!(
            cylinder_relation(&origin, &touching, margin),
            Some(CylinderRelation::Touching)
        );
        assert_eq!(
            cylinder_relation(&origin, &overlapping, margin),
            Some(CylinderRelation::PositiveOverlap)
        );
    }

    #[test]
    fn fuse_all_overlapping_linear_cylinder_patterns_are_exact() {
        use brepkit_math::mat::Mat4;

        for spacing in [3.0, 0.5] {
            let mut topo = Topology::new();
            let solids: Vec<_> = (0..3)
                .map(|i| {
                    let solid = crate::primitives::make_cylinder(&mut topo, 5.0, 10.0).unwrap();
                    crate::transform::transform_solid(
                        &mut topo,
                        solid,
                        &Mat4::translation(i as f64 * spacing, 0.0, 0.0),
                    )
                    .unwrap();
                    solid
                })
                .collect();
            let cid = topo.add_compound(Compound::new(solids));
            let fused = fuse_all(&mut topo, cid).unwrap();
            let volume = crate::measure::solid_volume(&topo, fused, 0.01).unwrap();
            // Inclusion-exclusion: the first/third lens is also the triple
            // overlap and cancels. Subtracting it again is a tempting but
            // incorrect reference formula for this three-disc row.
            let one = std::f64::consts::PI * 25.0 * 10.0;
            let expected = 3.0 * one - 2.0 * circle_lens_area(5.0, spacing) * 10.0;
            assert!(
                (volume - expected).abs() / expected < 2e-4,
                "spacing {spacing}: exact union volume {volume} vs {expected}"
            );
            assert_cylinder_union_health(&topo, fused);
        }
    }

    #[test]
    fn fuse_all_overlapping_circular_cylinder_patterns_are_exact() {
        use brepkit_math::mat::Mat4;

        for count in [6, 12] {
            let mut topo = Topology::new();
            let centers: Vec<_> = (0..count)
                .map(|i| {
                    let angle = std::f64::consts::TAU * i as f64 / count as f64;
                    (6.0 * angle.cos(), 6.0 * angle.sin())
                })
                .collect();
            let solids: Vec<_> = centers
                .iter()
                .map(|&(x, y)| {
                    let solid = crate::primitives::make_cylinder(&mut topo, 5.0, 10.0).unwrap();
                    crate::transform::transform_solid(
                        &mut topo,
                        solid,
                        &Mat4::translation(x, y, 0.0),
                    )
                    .unwrap();
                    solid
                })
                .collect();
            let cid = topo.add_compound(Compound::new(solids));
            let fused = fuse_all(&mut topo, cid).unwrap();
            let volume = crate::measure::solid_volume(&topo, fused, 0.01).unwrap();
            let expected = numeric_circle_union_area(&centers, 5.0) * 10.0;
            assert!(
                (volume - expected).abs() / expected < 1e-6,
                "ring count {count}: exact union volume {volume} vs numerical reference {expected}"
            );
            assert_cylinder_union_health(&topo, fused);
        }
    }
}
