//! Sectioning (slicing) solids with planes.
//!
//! Computes the cross-section of a solid at a given cutting plane,
//! producing face(s) representing the intersection.

#![allow(clippy::too_many_lines, clippy::doc_markdown)]

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use remus_math::nurbs::intersection::IntersectionPoint;

use std::collections::{HashMap, HashSet};

use crate::boolean::face_polygon;
use crate::dot_normal_point;

// Finite-band certification stays private to the section module.

/// Chain intersection curve points into consecutive segment pairs.
///
/// Instead of connecting only the first and last point (chord approximation),
/// this chains all intermediate points as individual segments, faithfully
/// tracing the actual intersection curve.
fn chain_curve_points(points: &[IntersectionPoint], segments: &mut Vec<(Point3, Point3)>) {
    if points.len() < 2 {
        return;
    }
    for pair in points.windows(2) {
        segments.push((pair[0].point, pair[1].point));
    }
}

/// The points of each carrier section curve, in order.
fn section_polylines(
    curves: &[remus_math::nurbs::intersection::IntersectionCurve],
) -> Vec<Vec<Point3>> {
    curves
        .iter()
        .map(|curve| curve.points.iter().map(|p| p.point).collect())
        .collect()
}

/// A cross-section result: one or more planar faces on the cutting plane.
#[derive(Debug)]
pub struct Section {
    /// The face IDs of the cross-section faces in the topology.
    pub faces: Vec<FaceId>,
}

/// Compute the cross-section of a solid with a plane.
///
/// The cutting plane is defined by a point on the plane and its normal.
/// Returns the cross-section as one or more planar faces lying on the
/// cutting plane. For a simple convex solid this is typically one face;
/// for solids with holes or disconnected volumes there may be multiple.
/// A plane that misses the solid entirely yields an empty face list
/// (a successful empty result, not an error).
///
/// # Algorithm
///
/// 1. For each face of the solid, compute the intersection with the
///    cutting plane (line segments for planar faces).
/// 2. Collect all intersection segments.
/// 3. Assemble segments into closed wires.
/// 4. Create planar faces from the wires.
///
/// # Errors
///
/// Returns an error if NURBS intersection computation fails, or if
/// intersection segments exist but cannot be assembled into a closed
/// cross-section wire. Intersecting cylindrical faces must have a complete
/// two-rim band; cuts through its rims or parallel to its axis return
/// `Unsupported` until authoritative cap clipping is available. Contained
/// cylindrical sections retain the existing 64-segment polygon approximation.
/// Sphere, cone and torus sections are the same 64-chord polylines of the
/// complete carrier, clipped to the face's trims where they cross its
/// boundary; a curved section that leaves its face away from every boundary
/// crossing, or a cone face spanning its apex, returns `Unsupported`.
pub fn section(
    topo: &mut Topology,
    solid: SolidId,
    plane_point: Point3,
    plane_normal: Vec3,
) -> Result<Section, crate::OperationsError> {
    let tol = Tolerance::new();

    let normal = plane_normal.normalize()?;
    let d = dot_normal_point(normal, plane_point);

    let solid_data = topo.solid(solid)?;
    let all_shell_ids: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();

    let mut face_ids: Vec<FaceId> = Vec::new();
    for shell_id in &all_shell_ids {
        let shell = topo.shell(*shell_id)?;
        face_ids.extend_from_slice(shell.faces());
    }

    let mut segments: Vec<(Point3, Point3)> = Vec::new();

    for &fid in &face_ids {
        let face = topo.face(fid)?;
        match face.surface() {
            FaceSurface::Plane {
                normal: face_normal,
                d: face_d,
            } => {
                let face_normal = *face_normal;
                let face_d = *face_d;

                let verts = face_polygon(topo, fid)?;
                if let Some(seg) =
                    intersect_planar_face_with_plane(&verts, face_normal, face_d, normal, d, tol)
                {
                    segments.push(seg);
                }
            }
            FaceSurface::Nurbs(nurbs) => {
                let intersection_curves =
                    remus_math::nurbs::intersection::intersect_plane_nurbs(nurbs, normal, d, 50)?;
                for curve in &intersection_curves {
                    chain_curve_points(&curve.points, &mut segments);
                }
            }
            FaceSurface::Cylinder(cyl) => {
                cylinder::append_section(topo, face, cyl, plane_point, normal, tol, &mut segments)?;
            }
            FaceSurface::Cone(cone) => {
                let loops = curved::cone_section(topo, fid, cone, normal, d)?;
                curved::append_clipped(topo, fid, &loops, normal, d, tol, &mut segments)?;
            }
            FaceSurface::Sphere(sphere) => {
                let curves =
                    remus_math::analytic_intersection::intersect_plane_sphere(sphere, normal, d)?;
                let loops = section_polylines(&curves);
                curved::append_clipped(topo, fid, &loops, normal, d, tol, &mut segments)?;
            }
            FaceSurface::Torus(torus) => {
                let curves =
                    remus_math::analytic_intersection::intersect_plane_torus(torus, normal, d)?;
                let loops = section_polylines(&curves);
                curved::append_clipped(topo, fid, &loops, normal, d, tol, &mut segments)?;
            }
        }
    }

    // Analytic faces can share a section curve across two oppositely-oriented
    // faces — e.g. a sphere's two hemispheres each yield the full equatorial
    // circle — emitting every segment twice in opposing directions. Left in
    // place, the wire assembler chains both copies into a single zero-area
    // double loop, so collapse coincident duplicates first.
    dedup_coincident_segments(&mut segments, tol);

    // If no crossing segments were found, the cutting plane may be exactly
    // coplanar with one or more faces. In that case, extract the boundary
    // edges of those coplanar faces as the cross-section.
    if segments.is_empty() {
        let coplanar_segs = extract_coplanar_boundary(topo, &face_ids, normal, d, tol)?;
        segments = coplanar_segs;
    }

    // No crossing segments and no coplanar boundary across outer + inner
    // shells means the cutting plane lies wholly on one side of the solid:
    // a genuine miss. The empty set is a valid section, not an error.
    if segments.is_empty() {
        return Ok(Section { faces: Vec::new() });
    }

    let mut wires = assemble_wires(topo, &segments, normal, d, tol)?;

    // Fallback: if crossing-based segments didn't form closed wires,
    // try using coplanar face boundaries instead (handles the case where
    // the cutting plane exactly coincides with a face of the solid).
    if wires.is_empty() {
        let coplanar_segs = extract_coplanar_boundary(topo, &face_ids, normal, d, tol)?;
        if !coplanar_segs.is_empty() {
            wires = assemble_wires(topo, &coplanar_segs, normal, d, tol)?;
        }
    }

    if wires.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no closed cross-section could be assembled".into(),
        });
    }

    // Group wires by containment: outer wires get their inner wires as holes.
    // Without this, a hollow shape's section would produce two separate faces
    // (outer rectangle + inner hole rectangle) instead of one face-with-hole.
    let groups = group_wires_by_containment(topo, &wires, normal);

    let mut result_faces = Vec::with_capacity(groups.len());
    for (outer, inners) in groups {
        let face = topo.add_face(Face::new(outer, inners, FaceSurface::Plane { normal, d }));
        result_faces.push(face);
    }

    Ok(Section {
        faces: result_faces,
    })
}

/// Group section wires by containment: each outer wire collects all wires that
/// are spatially inside it as inner (hole) wires. Returns `(outer, [inner])` tuples.
///
/// A wire is "inner" when it sits inside another wire (in the section plane) and
/// is not itself inside any wire that's already inside another. Two-level nesting
/// (holes within holes within holes) is collapsed into the outermost containment;
/// CAD sections don't typically produce deeper nesting.
fn group_wires_by_containment(
    topo: &Topology,
    wires: &[WireId],
    normal: Vec3,
) -> Vec<(WireId, Vec<WireId>)> {
    if wires.len() <= 1 {
        return wires.iter().map(|&w| (w, vec![])).collect();
    }

    // Collect each wire's vertex polygon (used for both bounding-box and
    // point-in-polygon checks below).
    let polygons: Vec<Vec<Point3>> = wires.iter().map(|&wid| wire_vertices(topo, wid)).collect();

    // For each wire, find the smallest wire that strictly contains it.
    // A wire with no strict container is itself an outer wire.
    let mut parent: Vec<Option<usize>> = vec![None; wires.len()];
    for (i, poly_i) in polygons.iter().enumerate() {
        if poly_i.is_empty() {
            continue;
        }
        let sample = poly_i[0];
        let mut best_parent: Option<usize> = None;
        let mut best_area = f64::INFINITY;
        for (j, poly_j) in polygons.iter().enumerate() {
            if i == j || poly_j.len() < 3 {
                continue;
            }
            if crate::distance::point_in_polygon_3d(&sample, poly_j, &normal) {
                let area = polygon_area_3d(poly_j, normal);
                if area < best_area {
                    best_area = area;
                    best_parent = Some(j);
                }
            }
        }
        parent[i] = best_parent;
    }

    // Build groups: each outer wire (no parent) collects its direct children.
    let mut groups: Vec<(WireId, Vec<WireId>)> = Vec::new();
    let mut group_of: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    for (i, &wid) in wires.iter().enumerate() {
        if parent[i].is_none() {
            group_of.insert(i, groups.len());
            groups.push((wid, vec![]));
        }
    }
    for (i, &wid) in wires.iter().enumerate() {
        if let Some(p) = parent[i] {
            // Walk up to the outermost ancestor (in case of nested holes).
            let mut top = p;
            while let Some(next) = parent[top] {
                top = next;
            }
            if let Some(&group_idx) = group_of.get(&top) {
                groups[group_idx].1.push(wid);
            }
        }
    }

    groups
}

fn wire_vertices(topo: &Topology, wire_id: WireId) -> Vec<Point3> {
    let Ok(wire) = topo.wire(wire_id) else {
        return vec![];
    };
    let mut pts = Vec::with_capacity(wire.edges().len());
    for oe in wire.edges() {
        let Ok(edge) = topo.edge(oe.edge()) else {
            continue;
        };
        let vid = if oe.is_forward() {
            edge.start()
        } else {
            edge.end()
        };
        if let Ok(v) = topo.vertex(vid) {
            pts.push(v.point());
        }
    }
    pts
}

/// Unsigned area of a 3D planar polygon (projected to dominant axis plane).
fn polygon_area_3d(polygon: &[Point3], normal: Vec3) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    let ax = normal.x().abs();
    let ay = normal.y().abs();
    let az = normal.z().abs();
    let to_2d = |p: Point3| -> (f64, f64) {
        if az >= ax && az >= ay {
            (p.x(), p.y())
        } else if ay >= ax {
            (p.x(), p.z())
        } else {
            (p.y(), p.z())
        }
    };
    let mut sum = 0.0;
    let n = polygon.len();
    for i in 0..n {
        let (x0, y0) = to_2d(polygon[i]);
        let (x1, y1) = to_2d(polygon[(i + 1) % n]);
        sum += x0 * y1 - x1 * y0;
    }
    (sum * 0.5).abs()
}

/// Return whether two points coincide within the endpoint-matching tolerance.
fn coincident_points(a: Point3, b: Point3, endpoint_tol: f64) -> bool {
    (a.x() - b.x()).abs() <= endpoint_tol
        && (a.y() - b.y()).abs() <= endpoint_tol
        && (a.z() - b.z()).abs() <= endpoint_tol
}

type PointBucket = (u64, u64, u64);

/// Spatial index that assigns tolerance-coincident endpoints a stable ID.
///
/// The bucket coordinates deliberately remain floating-point bit patterns
/// rather than being cast to bounded integers. This keeps the index valid for
/// coordinates whose quotient by the tolerance is outside the `i64` range.
struct EndpointIndex {
    tolerance: f64,
    buckets: HashMap<PointBucket, Vec<(Point3, usize)>>,
    next_id: usize,
}

impl EndpointIndex {
    fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            buckets: HashMap::new(),
            next_id: 0,
        }
    }

    fn axis_bucket(value: f64, tolerance: f64) -> u64 {
        let scaled = (value / tolerance).floor();
        if scaled.is_finite() {
            scaled.to_bits()
        } else {
            // At this magnitude adjacent finite floats are much farther apart
            // than the modeling tolerance, so exact bits are sufficient.
            value.to_bits()
        }
    }

    fn bucket(&self, point: Point3) -> PointBucket {
        (
            Self::axis_bucket(point.x(), self.tolerance),
            Self::axis_bucket(point.y(), self.tolerance),
            Self::axis_bucket(point.z(), self.tolerance),
        )
    }

    fn id(&mut self, point: Point3) -> usize {
        let offsets = [-self.tolerance, 0.0, self.tolerance];
        for dx in offsets {
            for dy in offsets {
                for dz in offsets {
                    let nearby = Point3::new(point.x() + dx, point.y() + dy, point.z() + dz);
                    let bucket = self.bucket(nearby);
                    if let Some(entries) = self.buckets.get(&bucket)
                        && let Some((_, id)) = entries.iter().find(|(candidate, _)| {
                            coincident_points(point, *candidate, self.tolerance)
                        })
                    {
                        return *id;
                    }
                }
            }
        }

        let id = self.next_id;
        self.next_id += 1;
        self.buckets
            .entry(self.bucket(point))
            .or_default()
            .push((point, id));
        id
    }
}

fn endpoint_pair(index: &mut EndpointIndex, a: Point3, b: Point3) -> (usize, usize) {
    let a_id = index.id(a);
    let b_id = index.id(b);
    if a_id <= b_id {
        (a_id, b_id)
    } else {
        (b_id, a_id)
    }
}

/// Collapse geometrically-coincident segments, ignoring orientation.
///
/// A section curve shared by two oppositely-oriented faces is emitted once in
/// each direction (see the sphere-hemisphere case at the call site). Comparing
/// unordered endpoints keeps a single copy without quantizing absolute world
/// coordinates into a bounded integer type.
/// Distinct polygon edges share at most one endpoint, so real edges of the
/// section outline are never merged.
fn dedup_coincident_segments(segments: &mut Vec<(Point3, Point3)>, tol: Tolerance) {
    let endpoint_tol = tol.linear * 10.0;
    let mut endpoint_index = EndpointIndex::new(endpoint_tol);
    let mut seen = HashSet::with_capacity(segments.len());
    let mut unique = Vec::with_capacity(segments.len());
    for &(a, b) in segments.iter() {
        if seen.insert(endpoint_pair(&mut endpoint_index, a, b)) {
            unique.push((a, b));
        }
    }
    *segments = unique;
}

/// Extract boundary edges of faces coplanar with the cutting plane.
///
/// For each coplanar face, its edges are boundary edges if they are not shared
/// with another coplanar face. These boundary edges form the cross-section
/// outline where the solid meets the cutting plane.
fn extract_coplanar_boundary(
    topo: &Topology,
    face_ids: &[FaceId],
    cut_normal: Vec3,
    cut_d: f64,
    tol: Tolerance,
) -> Result<Vec<(Point3, Point3)>, crate::OperationsError> {
    // Use a relaxed tolerance for coplanar detection to handle
    // floating-point precision differences across platforms (e.g. WASM).
    let coplanar_tol = tol.linear * 100.0;

    let mut coplanar_faces = Vec::new();
    for &fid in face_ids {
        let verts = face_polygon(topo, fid)?;
        if verts.len() >= 3
            && verts
                .iter()
                .all(|v| (dot_normal_point(cut_normal, *v) - cut_d).abs() < coplanar_tol)
        {
            coplanar_faces.push(fid);
        }
    }

    if coplanar_faces.is_empty() {
        return Ok(Vec::new());
    }

    // Collect all edges of coplanar faces, matching their endpoints within the
    // modeling tolerance.
    // An edge shared by two coplanar faces appears twice and is internal.
    // An edge appearing once is a boundary edge.
    let endpoint_tol = tol.linear * 10.0;
    let mut endpoint_index = EndpointIndex::new(endpoint_tol);
    let mut edge_counts: HashMap<(usize, usize), (Point3, Point3, usize)> = HashMap::new();

    for &fid in &coplanar_faces {
        let verts = face_polygon(topo, fid)?;
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            let key = endpoint_pair(&mut endpoint_index, a, b);
            edge_counts
                .entry(key)
                .and_modify(|(_, _, count)| *count += 1)
                .or_insert((a, b, 1));
        }
    }

    // Boundary edges appear exactly once.
    let boundary: Vec<(Point3, Point3)> = edge_counts
        .into_iter()
        .filter(|(_, (_, _, count))| *count == 1)
        .map(|(_, (a, b, _))| (a, b))
        .collect();

    Ok(boundary)
}

/// Intersect a planar face polygon with a cutting plane.
///
/// Returns the line segment (if any) where the cutting plane crosses
/// the face polygon. Uses the Sutherland-Hodgman approach: classify
/// each vertex as above/below the plane, find edge crossings.
fn intersect_planar_face_with_plane(
    verts: &[Point3],
    _face_normal: Vec3,
    _face_d: f64,
    cut_normal: Vec3,
    cut_d: f64,
    tol: Tolerance,
) -> Option<(Point3, Point3)> {
    let n = verts.len();
    if n < 3 {
        return None;
    }

    let dists: Vec<f64> = verts
        .iter()
        .map(|v| dot_normal_point(cut_normal, *v) - cut_d)
        .collect();

    // If all vertices lie on the cutting plane the face is coplanar —
    // the cross-section boundary comes from adjacent faces, not this one.
    // Use a relaxed tolerance (100× linear) for cross-platform consistency.
    let coplanar_tol = tol.linear * 100.0;
    if dists.iter().all(|d| d.abs() < coplanar_tol) {
        return None;
    }

    let mut crossings = Vec::new();

    for i in 0..n {
        let j = (i + 1) % n;
        let di = dists[i];
        let dj = dists[j];

        if di.abs() < tol.linear {
            crossings.push(verts[i]);
            continue;
        }

        // Check for sign change (edge crosses plane).
        if (di > tol.linear && dj < -tol.linear) || (di < -tol.linear && dj > tol.linear) {
            let t = di / (di - dj);
            let pi = verts[i];
            let pj = verts[j];
            let ix = Point3::new(
                (pj.x() - pi.x()).mul_add(t, pi.x()),
                (pj.y() - pi.y()).mul_add(t, pi.y()),
                (pj.z() - pi.z()).mul_add(t, pi.z()),
            );
            crossings.push(ix);
        }
    }

    let mut unique = Vec::new();
    for p in &crossings {
        if !unique
            .iter()
            .any(|q: &Point3| (*p - *q).length_squared() < tol.linear * tol.linear)
        {
            unique.push(*p);
        }
    }

    if unique.len() >= 2 {
        Some((unique[0], unique[1]))
    } else {
        None
    }
}

/// Assemble intersection segments into closed wires.
///
/// Chains segments endpoint-to-endpoint using spatial proximity,
/// producing one or more closed wires.
fn assemble_wires(
    topo: &mut Topology,
    segments: &[(Point3, Point3)],
    _normal: Vec3,
    _d: f64,
    tol: Tolerance,
) -> Result<Vec<WireId>, crate::OperationsError> {
    if segments.is_empty() {
        return Ok(vec![]);
    }

    let mut remaining: Vec<(Point3, Point3)> = segments.to_vec();
    let mut wires = Vec::new();

    // Chaining tolerance must be tight enough to keep separate loops apart
    // (e.g., the outer perimeter and the inner hole of a hollow shape) while
    // still tolerating the small endpoint drift produced by tessellated NURBS
    // intersections. `tol.linear * 1000` (≈ 1e-4) balances both.
    //
    // Earlier this defaulted to 50% of the average segment length, which for
    // a hollow 20×20 box section was ≈ 15 — wildly generous, bridging the
    // outer and inner wires into one self-intersecting polygon.
    let chain_tol = tol.linear * 1000.0;

    while !remaining.is_empty() {
        let first = remaining.remove(0);
        let mut chain: Vec<Point3> = vec![first.0, first.1];

        let mut changed = true;
        while changed {
            changed = false;
            let chain_end = chain[chain.len() - 1];
            let threshold_sq = chain_tol * chain_tol;

            let mut best_idx = None;
            let mut best_dist = threshold_sq;
            let mut best_forward = true;

            for i in 0..remaining.len() {
                let (a, b) = remaining[i];
                let dist_a = (a - chain_end).length_squared();
                let dist_b = (b - chain_end).length_squared();

                if dist_a < best_dist {
                    best_dist = dist_a;
                    best_idx = Some(i);
                    best_forward = true;
                }
                if dist_b < best_dist {
                    best_dist = dist_b;
                    best_idx = Some(i);
                    best_forward = false;
                }
            }

            if let Some(idx) = best_idx {
                let (a, b) = remaining.remove(idx);
                chain.push(if best_forward { b } else { a });
                changed = true;
            }
        }

        if chain.len() < 3 {
            continue;
        }

        let start = chain[0];
        let end = chain[chain.len() - 1];
        let closed = (start - end).length_squared() < chain_tol * chain_tol;

        if !closed {
            continue;
        }

        if chain.len() > 3 {
            chain.pop();
        }

        let n = chain.len();
        let vert_ids: Vec<_> = chain
            .iter()
            .map(|&p| topo.add_vertex(Vertex::new(p, tol.linear)))
            .collect();

        let mut oriented_edges = Vec::with_capacity(n);
        for i in 0..n {
            let j = (i + 1) % n;
            let edge = topo.add_edge(Edge::new(vert_ids[i], vert_ids[j], EdgeCurve::Line));
            oriented_edges.push(OrientedEdge::new(edge, true));
        }

        let wire = Wire::new(oriented_edges, true).map_err(crate::OperationsError::Topology)?;
        wires.push(topo.add_wire(wire));
    }

    Ok(wires)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_cube_manifold;

    use super::*;

    #[test]
    fn section_cube_at_half_height() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        // Cut with a horizontal plane at z=0.5.
        let result = section(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        assert_eq!(
            result.faces.len(),
            1,
            "should produce one cross-section face"
        );

        // The cross section of a unit cube at z=0.5 should be a unit square.
        let area = crate::measure::face_area(&topo, result.faces[0], 0.1).unwrap();
        assert!(
            (area - 1.0).abs() < 1e-6,
            "cross-section area should be ~1.0, got {area}"
        );
    }

    #[test]
    fn dedup_preserves_large_coordinate_outline() {
        let x = 1.0e13;
        let mut segments = vec![
            (Point3::new(x, 0.0, 0.5), Point3::new(x + 1.0, 0.0, 0.5)),
            (
                Point3::new(x + 1.0, 0.0, 0.5),
                Point3::new(x + 1.0, 1.0, 0.5),
            ),
            (Point3::new(x + 1.0, 1.0, 0.5), Point3::new(x, 1.0, 0.5)),
            (Point3::new(x, 1.0, 0.5), Point3::new(x, 0.0, 0.5)),
        ];

        dedup_coincident_segments(&mut segments, Tolerance::new());

        assert_eq!(segments.len(), 4, "all distinct outline edges must remain");
    }

    #[test]
    fn dedup_scales_across_many_distinct_segments() {
        let mut segments: Vec<_> = (0..10_000)
            .map(|i| {
                let x = f64::from(i);
                (Point3::new(x, 0.0, 0.0), Point3::new(x, 0.5, 0.0))
            })
            .collect();
        segments.push((
            Point3::new(9_999.0, 0.5, 0.0),
            Point3::new(9_999.0, 0.0, 0.0),
        ));

        dedup_coincident_segments(&mut segments, Tolerance::new());

        assert_eq!(segments.len(), 10_000);
    }

    #[test]
    fn section_cube_at_quarter_height() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let result = section(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, 0.25),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        assert_eq!(result.faces.len(), 1);

        // Still a 1×1 square at any height for a cube.
        let area = crate::measure::face_area(&topo, result.faces[0], 0.1).unwrap();
        assert!(
            (area - 1.0).abs() < 1e-6,
            "cross-section area should be ~1.0, got {area}"
        );
    }

    #[test]
    fn section_cube_along_x() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        // Cut with a vertical plane at x=0.5.
        let result = section(
            &mut topo,
            cube,
            Point3::new(0.5, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();

        assert_eq!(result.faces.len(), 1);

        // Cross-section of unit cube at x=0.5 is a 1×1 square in YZ.
        let area = crate::measure::face_area(&topo, result.faces[0], 0.1).unwrap();
        assert!(
            (area - 1.0).abs() < 1e-6,
            "cross-section area should be ~1.0, got {area}"
        );
    }

    /// Section a box-minus-sphere at z=0.
    ///
    /// Box is (0,0,0)-(20,20,20), sphere at origin r=12.
    /// At z=0, the sphere intersects the box creating a circular cutout.
    /// Cross-section = 20×20 rectangle minus circle of radius 12
    ///   (but sphere center is at origin and box starts at 0, so the
    ///   sphere only clips a quarter-circle at the z=0 face corner).
    ///
    /// The exact area depends on how much of the sphere lies within the
    /// box at this plane. At minimum, we verify the section succeeds and
    /// produces a face with positive area less than the full 20×20 = 400.
    #[test]
    fn section_after_boolean_cut() {
        let mut topo = Topology::new();
        let b = crate::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
        let s = crate::primitives::make_sphere(&mut topo, 12.0, 16).unwrap();

        let solid =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, b, s).unwrap();

        let sec = section(
            &mut topo,
            solid,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        assert!(!sec.faces.is_empty(), "should produce at least one face");

        // The cross-section area should be positive and less than the full
        // box face (400). The sphere removes a quarter-disc of radius 12
        // from the corner, so area ≈ 400 - π(144)/4 ≈ 400 - 113.1 ≈ 286.9.
        let total_area: f64 = sec
            .faces
            .iter()
            .map(|&fid| crate::measure::face_area(&topo, fid, 0.1).unwrap())
            .sum();
        assert!(
            total_area > 200.0,
            "section area should be > 200 (box face minus sphere), got {total_area:.2}"
        );
        assert!(
            total_area < 400.0,
            "section area should be < 400 (full box face), got {total_area:.2}"
        );
    }

    #[test]
    fn section_plane_misses_solid() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        // Plane above the cube — misses entirely.
        let result = section(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        assert!(
            result.faces.is_empty(),
            "plane above cube should produce an empty section"
        );
    }

    #[test]
    fn section_plane_flush_with_top_face() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        // Plane coincident with the cube's top face at z=1 — the coplanar
        // boundary fallback must yield the face outline, not a miss.
        let result = section(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        assert_eq!(
            result.faces.len(),
            1,
            "flush plane should yield the face outline"
        );
        let area = crate::measure::face_area(&topo, result.faces[0], 0.1).unwrap();
        assert!(
            (area - 1.0).abs() < 1e-6,
            "flush-face section area should be ~1.0, got {area}"
        );
    }

    #[test]
    fn section_extruded_box() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();

        // Box extends from (0,0,0) to (2,3,4). Cut at z=2 (middle of the box).
        let result = section(
            &mut topo,
            solid,
            Point3::new(0.0, 0.0, 2.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        assert_eq!(result.faces.len(), 1);

        let area = crate::measure::face_area(&topo, result.faces[0], 0.1).unwrap();
        // 2×3 = 6
        assert!(
            (area - 6.0).abs() < 1e-6,
            "cross-section area should be ~6.0, got {area}"
        );
    }

    /// Diagonal section: plane x + y = 0.8, normal = (1,1,0).
    ///
    /// This plane intersects the unit cube at:
    ///   front (y=0): x=0.8 → line (0.8,0,0)-(0.8,0,1)
    ///   left (x=0): y=0.8 → line (0,0.8,0)-(0,0.8,1)
    ///   top (z=1): x+y=0.8 → line (0.8,0,1)-(0,0.8,1)
    ///   bottom (z=0): x+y=0.8 → line (0.8,0,0)-(0,0.8,0)
    ///
    /// The cross-section is a rectangle with:
    ///   width = distance between the two vertical lines
    ///         = |(0.8,0)-(0,0.8)| = √(0.64+0.64) = 0.8√2
    ///   height = 1.0 (z extent)
    ///   area = 0.8√2 ≈ 1.1314
    #[test]
    fn section_diagonal_plane() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let result = section(
            &mut topo,
            cube,
            Point3::new(0.4, 0.4, 0.5),
            Vec3::new(1.0, 1.0, 0.0),
        );

        assert!(
            result.is_ok(),
            "diagonal plane should intersect cube: {:?}",
            result.err()
        );
        let sec = result.unwrap();
        assert_eq!(sec.faces.len(), 1);

        // Area = width × height = 0.8√2 × 1.0 ≈ 1.1314
        let area = crate::measure::face_area(&topo, sec.faces[0], 0.01).unwrap();
        let expected = 0.8 * std::f64::consts::SQRT_2;
        let rel_err = (area - expected).abs() / expected;
        assert!(
            rel_err < 1e-4,
            "diagonal section area should be 0.8√2 ≈ {expected:.4}, got {area:.4} \
             (rel_err={rel_err:.2e})"
        );
    }

    /// Section of a cylinder at mid-height → circular cross-section.
    /// Cylinder r=5, h=10, section at z=5 → circle area = πr² = 25π ≈ 78.54.
    #[test]
    fn section_cylinder_at_midheight() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_cylinder(&mut topo, 5.0, 10.0).unwrap();

        let result = section(
            &mut topo,
            solid,
            Point3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
        );

        assert!(
            result.is_ok(),
            "section of cylinder should succeed: {:?}",
            result.err()
        );
        let sec = result.unwrap();
        assert!(!sec.faces.is_empty(), "should produce at least one face");

        let total_area: f64 = sec
            .faces
            .iter()
            .map(|&fid| crate::measure::face_area(&topo, fid, 0.01).unwrap())
            .sum();
        // Circle area = πr² = 25π ≈ 78.54
        let expected = std::f64::consts::PI * 25.0;
        let rel_err = (total_area - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "cylinder section area should be πr² = {expected:.2}, got {total_area:.2} \
             (rel_err={rel_err:.2e})"
        );
    }

    /// Section of a sphere through its center → a disk bounded by the great
    /// circle. A bare sphere is two hemisphere faces that each yield the full
    /// equatorial circle, so the section must collapse the duplicate into one
    /// disk rather than a zero-area double loop (#860). r=12 at z=0 → πr² =
    /// 144π ≈ 452.39.
    #[test]
    fn section_sphere_through_center() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_sphere(&mut topo, 12.0, 16).unwrap();

        let sec = section(
            &mut topo,
            solid,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        assert_eq!(sec.faces.len(), 1, "sphere section should be a single disk");

        let total_area: f64 = sec
            .faces
            .iter()
            .map(|&fid| crate::measure::face_area(&topo, fid, 0.01).unwrap())
            .sum();
        let expected = std::f64::consts::PI * 144.0;
        let rel_err = (total_area - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "sphere great-circle section area should be πr² = {expected:.2}, got \
             {total_area:.2} (rel_err={rel_err:.2e})"
        );
    }

    /// Section of a sphere off-center → a smaller disk. r=12 at z=5 →
    /// circle radius √(144−25)=√119, area = 119π ≈ 373.85.
    #[test]
    fn section_sphere_off_center() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_sphere(&mut topo, 12.0, 16).unwrap();

        let sec = section(
            &mut topo,
            solid,
            Point3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        assert_eq!(sec.faces.len(), 1, "sphere section should be a single disk");

        let total_area: f64 = sec
            .faces
            .iter()
            .map(|&fid| crate::measure::face_area(&topo, fid, 0.01).unwrap())
            .sum();
        let expected = std::f64::consts::PI * 119.0;
        let rel_err = (total_area - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "sphere off-center section area should be {expected:.2}, got {total_area:.2} \
             (rel_err={rel_err:.2e})"
        );
    }
}

mod cylinder {
    //! Sections admitted only by authoritative finite cylindrical boundaries.

    use std::f64::consts::TAU;

    use remus_math::surfaces::CylindricalSurface;
    use remus_math::tolerance::Tolerance;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::EdgeCurve;
    use remus_topology::face::Face;

    use crate::OperationsError;

    fn unsupported(reason: &str) -> OperationsError {
        OperationsError::Unsupported {
            operation: "section",
            reason: reason.into(),
        }
    }

    /// Recognize a periodic band bounded by two full circular rims and a paired
    /// axial seam. Closed endpoint identity alone does not certify a full turn:
    /// each authoritative circle span, carrier, endpoint and wire use is checked.
    fn axial_band(
        topo: &Topology,
        face: &Face,
        cylinder: &CylindricalSurface,
        tol: Tolerance,
    ) -> Result<(f64, f64), OperationsError> {
        let refuse =
            || unsupported("cylindrical section requires a complete two-rim band without holes");
        if !face.inner_wires().is_empty() {
            return Err(refuse());
        }
        let wire = topo.wire(face.outer_wire())?;
        let uses = wire.edges();
        if uses.len() != 4 || !wire.is_closed() {
            return Err(refuse());
        }
        let mut rims = Vec::new();
        let mut seam_uses = Vec::new();
        let mut winding = 0.0;
        for (i, oriented) in uses.iter().enumerate() {
            let edge = topo.edge(oriented.edge())?;
            let next = uses[(i + 1) % uses.len()];
            if oriented.oriented_end(edge) != next.oriented_start(topo.edge(next.edge())?) {
                return Err(refuse());
            }
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let (lo, hi) = crate::authoritative_edge_domain(edge, "cylindrical section boundary")?;
            if (edge.curve().evaluate_with_endpoints(lo, start, end) - start).length() > tol.linear
                || (edge.curve().evaluate_with_endpoints(hi, start, end) - end).length()
                    > tol.linear
            {
                return Err(refuse());
            }
            match edge.curve() {
                EdgeCurve::Circle(circle) => {
                    if edge.start() != edge.end()
                        || ((hi - lo).abs() - TAU).abs() > tol.angular
                        || !circle.radius().is_finite()
                        || circle.center().0.iter().any(|x| !x.is_finite())
                        || circle.normal().0.iter().any(|x| !x.is_finite())
                        || (circle.radius() - cylinder.radius()).abs() > tol.linear
                        || circle.normal().cross(cylinder.axis()).length() > tol.angular
                    {
                        return Err(refuse());
                    }
                    let delta = circle.center() - cylinder.origin();
                    let level = delta.dot(cylinder.axis());
                    if !level.is_finite() || (delta - cylinder.axis() * level).length() > tol.linear
                    {
                        return Err(refuse());
                    }
                    rims.push((edge.start(), level));
                    winding += (hi - lo).signum()
                        * circle.normal().dot(cylinder.axis()).signum()
                        * if oriented.is_forward() { 1.0 } else { -1.0 };
                }
                EdgeCurve::Line => {
                    if edge.start() == edge.end()
                        || (end - start).cross(cylinder.axis()).length() > tol.linear
                    {
                        return Err(refuse());
                    }
                    seam_uses.push(*oriented);
                }
                EdgeCurve::Ellipse(_)
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Parabola(_)
                | EdgeCurve::Hyperbola(_) => return Err(refuse()),
            }
        }
        if rims.len() != 2
            || seam_uses.len() != 2
            || winding.abs() > tol.angular
            || seam_uses[0].edge() != seam_uses[1].edge()
            || seam_uses[0].is_forward() == seam_uses[1].is_forward()
        {
            return Err(refuse());
        }
        let seam = topo.edge(seam_uses[0].edge())?;
        if !rims.iter().any(|&(vertex, _)| vertex == seam.start())
            || !rims.iter().any(|&(vertex, _)| vertex == seam.end())
        {
            return Err(refuse());
        }
        let low = rims[0].1.min(rims[1].1);
        let high = rims[0].1.max(rims[1].1);
        if high - low <= tol.linear {
            return Err(refuse());
        }
        Ok((low, high))
    }

    /// Append a contained plane/cylinder loop, or prove a finite-band miss.
    /// The complete carrier's axial sinusoid is bounded analytically; a sampled
    /// point or a midpoint cannot admit a loop crossing either authoritative rim.
    #[allow(
        clippy::too_many_arguments,
        clippy::cast_precision_loss,
        clippy::float_cmp
    )]
    pub(super) fn append_section(
        topo: &Topology,
        face: &Face,
        cylinder: &CylindricalSurface,
        plane_point: Point3,
        normal: Vec3,
        tol: Tolerance,
        segments: &mut Vec<(Point3, Point3)>,
    ) -> Result<(), OperationsError> {
        if !cylinder.radius().is_finite()
            || cylinder.radius() <= 0.0
            || cylinder.origin().0.iter().any(|x| !x.is_finite())
            || cylinder.axis().0.iter().any(|x| !x.is_finite())
            || (cylinder.axis().length_squared() - 1.0).abs() > tol.angular
        {
            return Err(unsupported(
                "cylindrical section requires a finite unit-axis carrier",
            ));
        }
        let axial = normal.dot(cylinder.axis());
        let radial_x = cylinder.radius() * normal.dot(cylinder.x_axis());
        let radial_y = cylinder.radius() * normal.dot(cylinder.y_axis());
        let amplitude = radial_x.hypot(radial_y);
        let offset = normal.dot(plane_point - cylinder.origin());
        // Only an exactly zero axial component proves a miss of the entire
        // unbounded carrier. A merely small component can intersect far along
        // the axis, so it still needs authoritative finite-boundary evidence.
        if axial == 0.0 && offset.is_finite() && offset.abs() > amplitude + tol.linear {
            return Ok(());
        }
        let (low, high) = axial_band(topo, face, cylinder, tol)?;
        let first = axial * low;
        let last = axial * high;
        if offset < first.min(last) - amplitude - tol.linear
            || offset > first.max(last) + amplitude + tol.linear
        {
            return Ok(());
        }
        if axial.abs() <= tol.angular {
            return Err(unsupported(
                "cylindrical section parallel to axis requires authoritative cap clipping",
            ));
        }
        let center = offset / axial;
        let extent = amplitude / axial.abs();
        if center - extent < low - tol.linear || center + extent > high + tol.linear {
            return Err(unsupported(
                "cylindrical section crossing a rim requires authoritative cap clipping",
            ));
        }
        let point_at = |angle: f64| {
            let (sine, cosine) = angle.sin_cos();
            let level = (offset - radial_x * cosine - radial_y * sine) / axial;
            cylinder.evaluate(angle, level)
        };
        // Preserve the existing section's 64 chords, with exact loop closure and
        // without the infinite-carrier helper's arbitrary |v| <= 100 cutoff.
        let first = point_at(0.0);
        let mut previous = first;
        for i in 1..=64 {
            let next = if i == 64 {
                first
            } else {
                point_at(TAU * i as f64 / 64.0)
            };
            segments.push((previous, next));
            previous = next;
        }
        Ok(())
    }
}

mod curved {
    //! Sections of sphere, cone and torus faces, clipped to the face's trims.
    //!
    //! The carrier's plane section is a set of polylines on the complete,
    //! untrimmed surface. A face is only a patch of it, so each sample is
    //! classified against the face's trims and the polylines are cut where
    //! they cross its boundary. The crossings are taken on the boundary
    //! polygons the planar neighbours section (`wire_polygon`), so a clipped
    //! arc ends exactly where the neighbouring face's segment does.

    use std::f64::consts::{PI, TAU};

    use remus_math::compact_sort::CompactSort;
    use remus_math::surfaces::ConicalSurface;
    use remus_math::tolerance::Tolerance;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeDomainError};
    use remus_topology::face::FaceId;

    use crate::OperationsError;
    use crate::dot_normal_point;

    fn unsupported(reason: String) -> OperationsError {
        OperationsError::Unsupported {
            operation: "section",
            reason,
        }
    }

    /// Samples per full turn of a cone section, matching the sphere and
    /// cylinder arms.
    const CONE_SAMPLES: usize = 64;

    /// Samples across the stretch of a cone section branch within the face's
    /// generator distances.
    const FACE_SAMPLES: u32 = 32;

    /// The plane section of the cone nappe that holds `face`, as polylines on
    /// the complete nappe: one closed loop (last point repeating the first)
    /// for an ellipse or circle, open branches for a parabola or hyperbola,
    /// cut off well beyond the face so clipping still sees them leave it.
    pub(super) fn cone_section(
        topo: &Topology,
        face: FaceId,
        cone: &ConicalSurface,
        normal: Vec3,
        d: f64,
    ) -> Result<Vec<Vec<Point3>>, OperationsError> {
        // Generator distances of the face's boundary; a face on both nappes,
        // or touching the apex, has no single section branch.
        let mut reach: f64 = 0.0;
        let mut nearest = f64::INFINITY;
        let mut nappe: Option<bool> = None;
        for point in boundary_samples(topo, face)? {
            let (_, v) = cone.project_point(point);
            let height = cone.axis().dot(point - cone.apex());
            if height.abs() <= Tolerance::new().linear {
                return Err(unsupported(
                    "conical section of a face reaching its apex".into(),
                ));
            }
            if nappe.is_some_and(|above| above != (height > 0.0)) {
                return Err(unsupported("conical face spans both nappes".into()));
            }
            nappe = Some(height > 0.0);
            reach = reach.max(v.abs());
            nearest = nearest.min(v.abs());
        }
        let Some(above) = nappe else {
            return Ok(Vec::new());
        };
        let limit = 4.0 * reach;
        let apex_offset = d - dot_normal_point(normal, cone.apex());
        if apex_offset.abs() <= Tolerance::new().linear {
            return Err(unsupported(
                "conical section by a plane through the apex".into(),
            ));
        }
        // On longitude u the generator is g(u) = cos a·radial(u) + sin a·axis,
        // and the plane meets it at v = apex_offset / n·g(u), where
        // n·g(u) = R cos(u − φ) + C. The point is on the face's nappe and
        // within the limit exactly when s·n·g(u) ≥ m, for the sign s that
        // puts v on that nappe and m = |apex_offset| / limit: one closed-form
        // interval of u, so a branch narrower than any sampling is found.
        let (sin_a, cos_a) = cone.half_angle().sin_cos();
        let (nx, ny, nz) = (
            normal.dot(cone.x_axis()),
            normal.dot(cone.y_axis()),
            normal.dot(cone.axis()),
        );
        let amplitude = cos_a * nx.hypot(ny);
        let phase = ny.atan2(nx);
        let constant = sin_a * nz;
        let side = if (apex_offset > 0.0) == above {
            1.0
        } else {
            -1.0
        };
        let floor = apex_offset.abs() / limit;
        let point = |u: f64| {
            let (sin_u, cos_u) = u.sin_cos();
            let generator =
                (cone.x_axis() * cos_u + cone.y_axis() * sin_u) * cos_a + cone.axis() * sin_a;
            cone.apex() + generator * (apex_offset / normal.dot(generator))
        };
        #[allow(clippy::cast_precision_loss)]
        if amplitude <= f64::EPSILON * constant.abs().max(1.0) {
            if side * constant < floor {
                return Ok(Vec::new());
            }
            let mut ring: Vec<Point3> = (0..CONE_SAMPLES)
                .map(|i| point(TAU * i as f64 / CONE_SAMPLES as f64))
                .collect();
            ring.push(ring[0]);
            return Ok(vec![ring]);
        }
        // s·(R cos w + C) ≥ |apex_offset| / l, with w measured from the
        // side's crest, holds for |v| ≤ l: cos w ≥ cos_at(l).
        let cos_at = |l: f64| (apex_offset.abs() / l - side * constant) / amplitude;
        let threshold = cos_at(limit);
        if threshold >= 1.0 {
            return Ok(Vec::new());
        }
        let closed = threshold <= -1.0;
        let half_width = if closed { PI } else { threshold.acos() };
        let crest = if side > 0.0 { phase } else { phase + PI };
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let steps = ((CONE_SAMPLES as f64 * half_width / PI).ceil() as usize).max(8);
        #[allow(clippy::cast_precision_loss)]
        let mut offsets: Vec<f64> = (0..=steps)
            .map(|k| half_width * (2.0 * k as f64 / steps as f64 - 1.0))
            .collect();
        // The face holds generator distances `nearest..=reach`, which on a
        // near-tangent plane is a stretch of the branch narrower than the
        // spacing above: sample it densely so the clipped section keeps its
        // shape. `cos_at` grows as `l` shrinks, so the band is
        // inner ≤ |w| ≤ outer, inside the branch.
        let near = cos_at(reach);
        if near < 1.0 {
            let outer = near.max(-1.0).acos();
            let far = cos_at(nearest);
            let inner = if far >= 1.0 {
                0.0
            } else {
                far.max(-1.0).acos()
            };
            for k in 0..=FACE_SAMPLES {
                let w = inner + (outer - inner) * f64::from(k) / f64::from(FACE_SAMPLES);
                offsets.extend([w, -w]);
            }
        }
        offsets.compact_sort_by(f64::total_cmp);
        offsets.dedup_by(|a, b| (*a - *b).abs() <= f64::EPSILON);
        let mut branch: Vec<Point3> = offsets.iter().map(|&w| point(crest + w)).collect();
        if closed {
            // −π and π are one longitude: close the loop exactly.
            branch.pop();
            branch.push(branch[0]);
        }
        Ok(vec![branch])
    }

    /// Points along every boundary edge of `face`.
    fn boundary_samples(topo: &Topology, face: FaceId) -> Result<Vec<Point3>, OperationsError> {
        let mut points = Vec::new();
        for edge in boundary_edges(topo, face)? {
            points.extend(edge.chain(8));
        }
        Ok(points)
    }

    /// One boundary edge of a face, with its endpoints and parameter range.
    struct BoundaryEdge {
        edge: Edge,
        start: Point3,
        end: Point3,
        range: (f64, f64),
    }

    impl BoundaryEdge {
        fn at(&self, s: f64) -> Point3 {
            let (t0, t1) = self.range;
            self.edge
                .curve()
                .evaluate_with_endpoints(t0 + (t1 - t0) * s, self.start, self.end)
        }

        /// The edge at `steps` equal parameter intervals.
        fn chain(&self, steps: u32) -> Vec<Point3> {
            (0..=steps)
                .map(|k| self.at(f64::from(k) / f64::from(steps)))
                .collect()
        }
    }

    fn boundary_edges(topo: &Topology, face: FaceId) -> Result<Vec<BoundaryEdge>, OperationsError> {
        let face = topo.face(face)?;
        let mut edges = Vec::new();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oriented in topo.wire(wire)?.edges() {
                let edge = topo.edge(oriented.edge())?.clone();
                let start = topo.vertex(edge.start())?.point();
                let end = topo.vertex(edge.end())?.point();
                let range = match edge.strict_domain() {
                    Ok(range) => range,
                    Err(EdgeDomainError::Missing { .. }) => {
                        edge.curve().reconstruct_domain_from_endpoints(start, end)
                    }
                    Err(error) => {
                        return Err(unsupported(format!(
                            "section of a face whose edge has invalid parameter authority: \
                             {error}"
                        )));
                    }
                };
                edges.push(BoundaryEdge {
                    edge,
                    start,
                    end,
                    range,
                });
            }
        }
        Ok(edges)
    }

    /// The boundary edges of `face` that lie in the plane, as polylines
    /// within an absolute sagitta of `100 × tol.linear` of their curves, and
    /// the largest sagitta reached. The bound is absolute so that scaling a
    /// body cannot widen it past the gap to another section loop; an edge
    /// that 2¹⁶ segments cannot resolve to it is refused.
    fn in_plane_edges(
        topo: &Topology,
        face: FaceId,
        normal: Vec3,
        d: f64,
        tol: Tolerance,
    ) -> Result<(Vec<(Point3, Point3)>, f64), OperationsError> {
        const MAX_STEPS: u32 = 1 << 16;
        let coplanar_tol = tol.linear * 100.0;
        let mut segments = Vec::new();
        let mut sagitta: f64 = 0.0;
        for edge in boundary_edges(topo, face)? {
            let mut steps = 32;
            let mut chain = edge.chain(steps);
            if !chain
                .iter()
                .all(|&p| (dot_normal_point(normal, p) - d).abs() < coplanar_tol)
            {
                continue;
            }
            loop {
                let deviation = (0..steps)
                    .map(|k| {
                        let mid = edge.at((f64::from(k) + 0.5) / f64::from(steps));
                        let (a, b) = (chain[k as usize], chain[k as usize + 1]);
                        (mid - (a + (b - a) * 0.5)).length()
                    })
                    .fold(0.0_f64, f64::max);
                if deviation <= coplanar_tol {
                    sagitta = sagitta.max(deviation);
                    break;
                }
                if steps >= MAX_STEPS {
                    return Err(unsupported(format!(
                        "section along an in-plane boundary edge that {MAX_STEPS} segments \
                         resolve only to {deviation:e}"
                    )));
                }
                steps *= 2;
                chain = edge.chain(steps);
            }
            segments.extend(chain.windows(2).map(|w| (w[0], w[1])));
        }
        Ok((segments, sagitta))
    }

    /// Where the cutting plane crosses the face's boundary polygons, by the
    /// same arithmetic the planar arm applies to the same polygons.
    fn boundary_crossings(
        topo: &Topology,
        face: FaceId,
        normal: Vec3,
        d: f64,
        tol: Tolerance,
    ) -> Result<Vec<Point3>, OperationsError> {
        let face = topo.face(face)?;
        let mut crossings = Vec::new();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            let polygon = crate::boolean::wire_polygon(topo, wire)?;
            let dists: Vec<f64> = polygon
                .iter()
                .map(|v| dot_normal_point(normal, *v) - d)
                .collect();
            let n = polygon.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let (di, dj) = (dists[i], dists[j]);
                if di.abs() < tol.linear {
                    crossings.push(polygon[i]);
                } else if (di > tol.linear && dj < -tol.linear)
                    || (di < -tol.linear && dj > tol.linear)
                {
                    let t = di / (di - dj);
                    let (pi, pj) = (polygon[i], polygon[j]);
                    crossings.push(Point3::new(
                        (pj.x() - pi.x()).mul_add(t, pi.x()),
                        (pj.y() - pi.y()).mul_add(t, pi.y()),
                        (pj.z() - pi.z()).mul_add(t, pi.z()),
                    ));
                }
            }
        }
        Ok(crossings)
    }

    /// Keep the parts of `polylines` (carrier section curves) that lie on
    /// `face`, ending each at the boundary crossing it leaves the face by.
    pub(super) fn append_clipped(
        topo: &Topology,
        face: FaceId,
        polylines: &[Vec<Point3>],
        normal: Vec3,
        d: f64,
        tol: Tolerance,
        segments: &mut Vec<(Point3, Point3)>,
    ) -> Result<(), OperationsError> {
        if polylines.iter().all(|p| p.len() < 2) {
            return Ok(());
        }
        let crossings = boundary_crossings(topo, face, normal, d, tol)?;
        // Every crossing belongs to the chord nearest it. Each chord is cut
        // at its crossings and each piece kept by its own midpoint, so a
        // trimmed span or a hole narrower than one chord is not dropped or
        // bridged by classifying only the chord's ends.
        let chords: Vec<(Point3, Point3)> = polylines
            .iter()
            .flat_map(|polyline| polyline.windows(2).map(|w| (w[0], w[1])))
            .collect();
        let mut cuts: Vec<Vec<(f64, Point3)>> = vec![Vec::new(); chords.len()];
        for &x in &crossings {
            let nearest = chords
                .iter()
                .enumerate()
                .map(|(i, &(a, b))| (segment_distance(x, a, b), i))
                .min_by(|l, r| l.0.total_cmp(&r.0));
            if let Some((distance, i)) = nearest {
                let (a, b) = chords[i];
                let ab = b - a;
                if distance <= ab.length().max(tol.linear * 1000.0) {
                    let along = (x - a).dot(ab) / ab.length_squared().max(f64::MIN_POSITIVE);
                    cuts[i].push((along, x));
                }
            }
        }
        // A piece running along a boundary edge that lies in the plane is the
        // face's own boundary, shared with its neighbour (and deduplicated);
        // it is kept without classifying a point on the trim itself. Both of
        // its ends (carrier samples or crossings) then lie on that edge's
        // curve, so within the edge polyline's sagitta of it. The band is
        // that absolute approximation error, not a fraction of a chord or of
        // the rim: on a slender torus the carrier's other loop runs closer
        // than either.
        let (in_plane, sagitta) = in_plane_edges(topo, face, normal, d, tol)?;
        let band = 2.0 * sagitta + tol.linear * 100.0;
        let near_boundary = |p: Point3| {
            in_plane
                .iter()
                .any(|&(a, b)| segment_distance(p, a, b) <= band)
        };
        let surface = topo.face(face)?.surface();
        let on_face = |p: Point3| -> Result<bool, OperationsError> {
            let on_carrier = surface
                .project_point(p)
                .and_then(|(u, v)| surface.evaluate(u, v))
                .unwrap_or(p);
            Ok(remus_check::classify::surface_point_in_face(
                topo, face, on_carrier,
            )?)
        };
        for (&(a, b), mut cut) in chords.iter().zip(cuts) {
            cut.compact_sort_by(|l, r| l.0.total_cmp(&r.0));
            let points: Vec<Point3> = std::iter::once(a)
                .chain(cut.into_iter().map(|(_, x)| x))
                .chain(std::iter::once(b))
                .collect();
            for piece in points.windows(2) {
                let (p, q) = (piece[0], piece[1]);
                if (q - p).length() > tol.linear
                    && ((near_boundary(p) && near_boundary(q)) || on_face(p + (q - p) * 0.5)?)
                {
                    segments.push((p, q));
                }
            }
        }
        Ok(())
    }

    fn segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
        let ab = b - a;
        let length_squared = ab.length_squared();
        let t = if length_squared > 0.0 {
            ((p - a).dot(ab) / length_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (p - (a + ab * t)).length()
    }

    #[cfg(test)]
    mod tests {
        #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

        use std::f64::consts::{PI, TAU};

        use remus_math::curves::Circle3D;
        use remus_math::tolerance::Tolerance;
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::Topology;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::explorer::solid_faces;
        use remus_topology::face::{Face, FaceSurface};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        use super::{append_clipped, cone_section};
        use crate::primitives::make_cone;
        use crate::revolve::revolve;

        /// A vertical plane at distance `c = 1.999` from the axis of the
        /// frustum `ρ = 2 − z/3` meets its lateral face for
        /// `|u − φ| ≤ acos(c/2) ≈ 0.032`, narrower than the 2π/64 longitude
        /// spacing. Turned to `φ = π/64`, midway between two samples, no
        /// sample saw it and the branch was dropped. It must be found, and
        /// sampled on the face densely enough to keep its shape.
        #[test]
        fn cone_branch_between_longitude_samples_is_found() {
            let mut topo = Topology::new();
            let solid = make_cone(&mut topo, 2.0, 1.0, 3.0).unwrap();
            let face = solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_)))
                .expect("a lateral face");
            let FaceSurface::Cone(cone) = topo.face(face).unwrap().surface().clone() else {
                panic!("a cone face");
            };
            let (c, phi) = (1.999, PI / 64.0);
            let normal = cone.x_axis() * phi.cos() + cone.y_axis() * phi.sin();
            assert!(normal.dot(cone.axis()).abs() < 1e-12);
            assert!(normal.dot(cone.apex() - Point3::new(0.0, 0.0, 0.0)).abs() < 1e-12);

            let branches = cone_section(&topo, face, &cone, normal, c).unwrap();
            let on_face = branches
                .iter()
                .flatten()
                .filter(|&&p| {
                    let (u, v) = cone.project_point(p);
                    (normal.dot(p - Point3::new(0.0, 0.0, 0.0)) - c).abs() < 1e-9
                        && (cone.evaluate(u, v) - p).length() < 1e-9
                        && remus_check::classify::surface_point_in_face(&topo, face, p).unwrap()
                })
                .count();
            assert!(on_face >= 16, "{on_face} branch samples on the face");
        }

        /// `append_clipped` on a slender torus band (`major`, r = 1,
        /// v ∈ [−π/3, π/3]) cut flush with its top rim at ρ = major + 0.5,
        /// given both carrier loops at that height. The other loop, at
        /// ρ = major − 0.5, runs 1 from the rim.
        fn clip_flush_with_slender_rim(
            major: f64,
        ) -> Result<Vec<(Point3, Point3)>, crate::OperationsError> {
            let mut topo = Topology::new();
            let h = (PI / 3.0).sin();
            let corner = |x: f64, z: f64| Point3::new(x, 0.0, z);
            let [a, b, c, d] = [
                corner(major - 2.0, -h),
                corner(major + 0.5, -h),
                corner(major + 0.5, h),
                corner(major - 2.0, h),
            ]
            .map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
            let tube = Circle3D::new_with_ref(
                Point3::new(major, 0.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                1.0,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
            let mut line = |s, e| topo.add_edge(Edge::new(s, e, EdgeCurve::Line));
            let (bottom, top, inner) = (line(a, b), line(c, d), line(d, a));
            let arc = topo.add_edge(Edge::new(b, c, EdgeCurve::Circle(tube)));
            let edges = [bottom, arc, top, inner].map(|e| OrientedEdge::new(e, true));
            let wire = topo.add_wire(Wire::new(edges.to_vec(), true).unwrap());
            let profile = topo.add_face(Face::new(
                wire,
                vec![],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, -1.0, 0.0),
                    d: 0.0,
                },
            ));
            let solid = revolve(
                &mut topo,
                profile,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                TAU,
            )
            .unwrap();
            let face = solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
                .expect("a torus band");

            let circle = |rho: f64| -> Vec<Point3> {
                (0..=128)
                    .map(|k| {
                        let u = TAU * f64::from(k) / 128.0;
                        Point3::new(rho * u.cos(), rho * u.sin(), h)
                    })
                    .collect()
            };
            let polylines = [circle(major + 0.5), circle(major - 0.5)];
            let mut segments = Vec::new();
            append_clipped(
                &topo,
                face,
                &polylines,
                Vec3::new(0.0, 0.0, 1.0),
                h,
                Tolerance::new(),
                &mut segments,
            )?;
            Ok(segments)
        }

        /// At R = 100 the far loop is well inside a quarter of a 32-segment
        /// rim chord (≈ 4.9), so a chord-scaled band kept it as the rim.
        /// Only the rim may survive.
        #[test]
        fn far_carrier_loop_beside_an_in_plane_rim_is_dropped() {
            let major = 100.0;
            let segments = clip_flush_with_slender_rim(major).unwrap();
            assert!(!segments.is_empty(), "the rim is part of the section");
            for (p, q) in segments {
                for x in [p, q] {
                    let rho = x.x().hypot(x.y());
                    assert!(
                        (rho - (major + 0.5)).abs() < 1e-6,
                        "section point at ρ = {rho}"
                    );
                }
            }
        }

        /// At R = 1e6 a band of `1e-6` of the rim's length (≈ 4.7) swallowed
        /// the far loop too. No band set by the rim's size may decide which
        /// loop survives: 2¹⁶ segments cannot hold this rim to the absolute
        /// bound, so the clip is refused rather than guessed.
        #[test]
        fn rim_too_long_to_resolve_is_refused() {
            assert!(matches!(
                clip_flush_with_slender_rim(1e6),
                Err(crate::OperationsError::Unsupported { .. })
            ));
        }
    }
}
