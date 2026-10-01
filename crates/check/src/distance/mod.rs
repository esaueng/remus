//! Minimum distance and extrema between shapes.

#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::suboptimal_flops
)]

pub(crate) mod analytic;
pub(crate) mod edge;
pub mod face_bounds;
pub mod prepared;

pub use prepared::{DistanceOptions, DistanceScratch, PreparedDistanceSolid};

use std::collections::HashSet;

use remus_math::aabb::Aabb3;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::CheckError;

/// Which topological element supports a closest point.
#[derive(Debug, Clone, Copy)]
pub enum SupportElement {
    /// Closest point is on a face.
    Face(FaceId, f64, f64),
}

/// A single distance solution.
#[derive(Debug, Clone)]
pub struct DistanceResult {
    /// The minimum distance.
    pub distance: f64,
    /// Closest point on shape A (or the query point).
    pub point_a: Point3,
    /// Closest point on shape B.
    pub point_b: Point3,
}

/// Statistics for a point-to-solid distance query.
///
/// Reports how much of the model the branch-and-bound traversal actually
/// evaluated, so PERF-Q04 pruning effectiveness is measurable rather than
/// assumed. All counts are deterministic for a fixed input.
#[derive(Debug, Clone, Copy)]
pub struct DistanceStats {
    /// Total faces visited (outer plus inner shells).
    pub faces_total: usize,
    /// Faces with certified conservative bounds (pruning candidates).
    pub faces_prunable: usize,
    /// Faces with unknown bounds, always evaluated exhaustively.
    pub faces_mandatory: usize,
    /// Narrow-phase evaluations actually performed.
    pub faces_evaluated: usize,
    /// Prunable faces skipped by a mathematically justified bound
    /// (`lower > best`, so ties are always evaluated).
    pub faces_skipped_by_bound: usize,
    /// Candidates whose narrow phase returned no result. They never improve
    /// the best distance and are reported here, never silently discarded.
    pub narrow_phase_failures: usize,
}

/// A prunable face candidate with its precomputed lower bound.
struct WorkItem {
    face: FaceId,
    lower_sq: f64,
}

/// Compute the minimum distance from a point to a solid.
///
/// Traverses prunable faces in ascending lower-bound order (a linear
/// best-first branch-and-bound scan) against a valid upper-bound witness,
/// while faces with unknown bounds stay on a mandatory exhaustive side
/// path. Dispatches per face type: planar (point-to-polygon), analytic
/// (closed-form), NURBS (Newton projection).
///
/// Measurement note (PERF-Q04): for a single query over precomputed face
/// bounds, a sorted linear scan dominates a BVH tree traversal — both visit
/// candidates in the same lower-bound order and skip the same provably
/// useless set, while the tree additionally pays SAH construction and heap
/// traffic (240 faces: 43µs linear versus 325µs tree in release; see the
/// `distance_q04_perf` probe). No BVH is built here; the mandatory side
/// list carries every face whose bound cannot justify pruning.
///
/// # Errors
///
/// Returns an error if any topology entity is missing.
pub fn point_to_solid(
    topo: &Topology,
    point: Point3,
    solid: SolidId,
) -> Result<DistanceResult, CheckError> {
    Ok(point_to_solid_impl(topo, point, solid, true)?.0)
}

/// Compute the minimum distance from a point to a solid, with statistics.
///
/// Uses the same branch-and-bound traversal as [`point_to_solid`] and
/// additionally reports pruning effectiveness.
///
/// # Errors
///
/// Returns an error if any topology entity is missing.
pub fn point_to_solid_with_stats(
    topo: &Topology,
    point: Point3,
    solid: SolidId,
) -> Result<(DistanceResult, DistanceStats), CheckError> {
    point_to_solid_impl(topo, point, solid, true)
}

/// Compute the minimum distance from a point to a solid without pruning.
///
/// Evaluates every face with the same narrow phase ([`point_to_face`]) in
/// the same deterministic order the branch-and-bound path uses, but never
/// skips a candidate. This is the independent oracle for the accelerated
/// path: both modes must agree on distance, and on the closest point
/// whenever the minimum is unique. Local Newton locality is shared, not
/// cured — this mode is exhaustive over faces, not a global NURBS
/// certificate.
///
/// # Errors
///
/// Returns an error if any topology entity is missing.
pub fn point_to_solid_exhaustive(
    topo: &Topology,
    point: Point3,
    solid: SolidId,
) -> Result<(DistanceResult, DistanceStats), CheckError> {
    point_to_solid_impl(topo, point, solid, false)
}

/// Shared point-to-solid implementation.
///
/// `prune` selects branch-and-bound (`true`) or forced-exhaustive
/// (`false`) traversal. Both modes build the same face bounds first (so
/// missing-entity errors are identical), evaluate the mandatory side path
/// first in face-index order, then walk prunable faces in ascending
/// lower-bound order with face-index tie-breaks, updating the best on
/// strict improvement only. The accelerated mode additionally skips a
/// prunable face only when `lower > best`, which can never change the
/// winner; ties are always evaluated.
#[allow(clippy::too_many_lines)]
fn point_to_solid_impl(
    topo: &Topology,
    point: Point3,
    solid: SolidId,
    prune: bool,
) -> Result<(DistanceResult, DistanceStats), CheckError> {
    let face_ids = collect_solid_faces(topo, solid)?;

    // Bounds first: identical errors in both modes, and every topology
    // entity the narrow phase can touch is validated here, so traversal
    // itself cannot hit a missing entity that exhaustive mode would report.
    let bounds: Vec<face_bounds::FaceBound> = face_ids
        .iter()
        .map(|&fid| face_bounds::face_bound(topo, fid))
        .collect::<Result<Vec<_>, _>>()?;

    let mut mandatory: Vec<FaceId> = Vec::new();
    let mut prunable: Vec<WorkItem> = Vec::new();
    for (fid, bound) in face_ids.iter().zip(bounds.iter()) {
        if bound.prunable {
            prunable.push(WorkItem {
                face: *fid,
                lower_sq: bound.aabb.distance_squared_to_point(point),
            });
        } else {
            mandatory.push(*fid);
        }
    }
    prunable.sort_by(|a, b| {
        a.lower_sq
            .total_cmp(&b.lower_sq)
            .then_with(|| a.face.index().cmp(&b.face.index()))
    });

    let mut best_dist = f64::INFINITY;
    let mut best_point = point;
    let mut evaluated = 0usize;
    let mut failures = 0usize;

    // Mandatory side path: exhaustive, in face-index order, establishing the
    // upper-bound witness for branch-and-bound.
    for &fid in &mandatory {
        if let Some((dist, closest)) = point_to_face(topo, point, fid)? {
            evaluated += 1;
            if dist < best_dist {
                best_dist = dist;
                best_point = closest;
            }
        } else {
            evaluated += 1;
            failures += 1;
        }
    }

    let mut stats = DistanceStats {
        faces_total: face_ids.len(),
        faces_prunable: prunable.len(),
        faces_mandatory: mandatory.len(),
        faces_evaluated: 0,
        faces_skipped_by_bound: 0,
        narrow_phase_failures: 0,
    };

    // Prunable faces in ascending lower-bound order. The accelerated mode
    // skips a face only when `lower > best`: the true face distance is at
    // least the box distance, so a skipped face can never beat — or tie —
    // the running best, and the winner matches forced-exhaustive traversal
    // bit for bit. Ties (`lower == best`) are always evaluated.
    for item in &prunable {
        if prune && item.lower_sq > best_dist * best_dist {
            stats.faces_skipped_by_bound += 1;
            continue;
        }
        if let Some((dist, closest)) = point_to_face(topo, point, item.face)? {
            evaluated += 1;
            if dist < best_dist {
                best_dist = dist;
                best_point = closest;
            }
        } else {
            evaluated += 1;
            failures += 1;
        }
    }

    stats.faces_evaluated = evaluated;
    stats.narrow_phase_failures = failures;

    Ok((
        DistanceResult {
            distance: best_dist,
            point_a: point,
            point_b: best_point,
        },
        stats,
    ))
}

/// Compute the minimum distance from many points to a solid, amortizing the
/// one preparation over the whole batch.
///
/// Builds a [`PreparedDistanceSolid`] once and reuses one [`DistanceScratch`]
/// across all points. Output order is deterministic input order.
///
/// # Errors
///
/// Returns an error if the solid is invalid or any referenced entity is
/// missing, or if any single query fails (all-or-nothing, matching a loop over
/// [`point_to_solid`] collected with `collect::<Result<Vec<_>, _>>()`).
pub fn point_to_solid_batch(
    topo: &Topology,
    points: &[Point3],
    solid: SolidId,
) -> Result<Vec<DistanceResult>, CheckError> {
    let prepared = PreparedDistanceSolid::prepare(topo, solid)?;
    if points.is_empty() {
        return Ok(Vec::new());
    }
    let mut scratch = DistanceScratch::new();
    prepared.batch(points, &mut scratch)
}

/// Compute distance from a point to a single face, dispatching by surface type.
///
/// Uses default numerical options ([`DistanceOptions::default`]).
///
/// # Errors
///
/// Returns an error if the face lookup fails.
pub fn point_to_face(
    topo: &Topology,
    point: Point3,
    face_id: FaceId,
) -> Result<Option<(f64, Point3)>, CheckError> {
    point_to_face_with_options(topo, point, face_id, DistanceOptions::default())
}

/// Compute distance from a point to a single face with explicit options.
///
/// The prepared path threads its frozen [`DistanceOptions`] here so repeated
/// queries share one configuration; the one-shot path passes the default.
///
/// # Errors
///
/// Returns an error if the face lookup fails or the options are invalid.
pub fn point_to_face_with_options(
    topo: &Topology,
    point: Point3,
    face_id: FaceId,
    options: DistanceOptions,
) -> Result<Option<(f64, Point3)>, CheckError> {
    options.validate()?;
    point_to_face_validated(topo, point, face_id, options)
}

fn point_to_face_validated(
    topo: &Topology,
    point: Point3,
    face_id: FaceId,
    options: DistanceOptions,
) -> Result<Option<(f64, Point3)>, CheckError> {
    let face = topo.face(face_id)?;
    match face.surface() {
        FaceSurface::Plane { normal, d } => {
            let polygon = crate::util::face_polygon(topo, face_id)?;
            Ok(point_to_polygon_distance(point, &polygon, *normal, *d))
        }
        FaceSurface::Cylinder(cyl) => {
            let (dist, closest) = analytic::point_to_cylinder(point, cyl);
            if crate::classify::surface_point_in_face(topo, face_id, closest)? {
                Ok(Some((dist, closest)))
            } else {
                Ok(closest_point_on_wire_edges(topo, face_id, point)?)
            }
        }
        FaceSurface::Cone(cone) => {
            let (dist, closest) = analytic::point_to_cone(point, cone);
            if crate::classify::surface_point_in_face(topo, face_id, closest)? {
                Ok(Some((dist, closest)))
            } else {
                Ok(closest_point_on_wire_edges(topo, face_id, point)?)
            }
        }
        FaceSurface::Sphere(sph) => {
            let (dist, closest) = analytic::point_to_sphere(point, sph);
            if crate::classify::surface_point_in_face(topo, face_id, closest)? {
                Ok(Some((dist, closest)))
            } else {
                Ok(closest_point_on_wire_edges(topo, face_id, point)?)
            }
        }
        FaceSurface::Torus(tor) => {
            let (dist, closest) = analytic::point_to_torus(point, tor);
            if crate::classify::surface_point_in_face(topo, face_id, closest)? {
                Ok(Some((dist, closest)))
            } else {
                Ok(closest_point_on_wire_edges(topo, face_id, point)?)
            }
        }
        FaceSurface::Nurbs(nurbs) => {
            match remus_math::nurbs::projection::project_point_to_surface(
                nurbs,
                point,
                options.projection_tolerance,
            ) {
                Ok(proj) => {
                    if is_point_in_face_boundary(topo, face_id, proj.point)? {
                        Ok(Some((proj.distance, proj.point)))
                    } else {
                        closest_point_on_wire_edges(topo, face_id, point)
                    }
                }
                Err(_) => Ok(None),
            }
        }
    }
}

/// Compute the minimum distance between two solids.
///
/// Checks vertex-to-vertex, vertex-to-face, and edge-to-edge pairs
/// with AABB pruning for acceleration.
///
/// # Errors
///
/// Returns an error if any topology entity is missing.
#[allow(clippy::too_many_lines)]
pub fn solid_to_solid(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<DistanceResult, CheckError> {
    let verts_a = collect_solid_vertices(topo, solid_a)?;
    let verts_b = collect_solid_vertices(topo, solid_b)?;

    let mut best_dist = f64::INFINITY;
    let mut best_a = Point3::new(0.0, 0.0, 0.0);
    let mut best_b = Point3::new(0.0, 0.0, 0.0);

    // Pass 1: Vertex-vertex (cheap upper bound).
    for &pa in &verts_a {
        for &pb in &verts_b {
            let dist = (pa - pb).length();
            if dist < best_dist {
                best_dist = dist;
                best_a = pa;
                best_b = pb;
            }
        }
    }

    // Pass 2: Vertices of A against faces of B.
    let faces_b = collect_solid_faces(topo, solid_b)?;
    let mut aabbs_b: Vec<(usize, Aabb3)> = Vec::with_capacity(faces_b.len());
    for (i, &fid) in faces_b.iter().enumerate() {
        let aabb = crate::util::face_aabb(topo, fid)?;
        aabbs_b.push((i, aabb));
    }

    for &pa in &verts_a {
        for &(idx, ref aabb) in &aabbs_b {
            if aabb.distance_squared_to_point(pa) > best_dist * best_dist {
                continue;
            }
            if let Ok(Some((dist, closest))) = point_to_face(topo, pa, faces_b[idx])
                && dist < best_dist
            {
                best_dist = dist;
                best_a = pa;
                best_b = closest;
            }
        }
    }

    // Pass 3: Vertices of B against faces of A.
    let faces_a = collect_solid_faces(topo, solid_a)?;
    let mut aabbs_a: Vec<(usize, Aabb3)> = Vec::with_capacity(faces_a.len());
    for (i, &fid) in faces_a.iter().enumerate() {
        let aabb = crate::util::face_aabb(topo, fid)?;
        aabbs_a.push((i, aabb));
    }

    for &pb in &verts_b {
        for &(idx, ref aabb) in &aabbs_a {
            if aabb.distance_squared_to_point(pb) > best_dist * best_dist {
                continue;
            }
            if let Ok(Some((dist, closest))) = point_to_face(topo, pb, faces_a[idx])
                && dist < best_dist
            {
                best_dist = dist;
                best_b = pb;
                best_a = closest;
            }
        }
    }

    // Pass 4: Edge-edge with AABB pruning.
    let edges_a = collect_solid_edge_segments(topo, solid_a)?;
    let edges_b = collect_solid_edge_segments(topo, solid_b)?;

    for &(p0a, p1a) in &edges_a {
        let aabb_a = Aabb3::try_from_points([p0a, p1a].iter().copied())
            .unwrap_or(Aabb3 { min: p0a, max: p0a });
        for &(p0b, p1b) in &edges_b {
            let aabb_b = Aabb3::try_from_points([p0b, p1b].iter().copied())
                .unwrap_or(Aabb3 { min: p0b, max: p0b });
            if aabb_distance(&aabb_a, &aabb_b) > best_dist {
                continue;
            }
            let (dist, ca, cb) = edge::segment_segment_distance(p0a, p1a, p0b, p1b);
            if dist < best_dist {
                best_dist = dist;
                best_a = ca;
                best_b = cb;
            }
        }
    }

    Ok(DistanceResult {
        distance: best_dist,
        point_a: best_a,
        point_b: best_b,
    })
}

/// Collect all unique vertex positions from a solid (outer + inner shells).
fn collect_solid_vertices(topo: &Topology, solid: SolidId) -> Result<Vec<Point3>, CheckError> {
    let solid_data = topo.solid(solid)?;
    let mut seen = HashSet::new();
    let mut points = Vec::new();
    let shell_ids: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();
    for sid in shell_ids {
        let shell = topo.shell(sid)?;
        for &fid in shell.faces() {
            let face = topo.face(fid)?;
            let mut wire_ids = vec![face.outer_wire()];
            wire_ids.extend(face.inner_wires().iter().copied());
            for wid in wire_ids {
                let wire = topo.wire(wid)?;
                for oe in wire.edges() {
                    let edge_data = topo.edge(oe.edge())?;
                    for vid in [edge_data.start(), edge_data.end()] {
                        if seen.insert(vid) {
                            points.push(topo.vertex(vid)?.point());
                        }
                    }
                }
            }
        }
    }
    Ok(points)
}

/// Collect all face IDs from a solid (outer + inner shells).
fn collect_solid_faces(topo: &Topology, solid: SolidId) -> Result<Vec<FaceId>, CheckError> {
    let solid_data = topo.solid(solid)?;
    let mut faces = Vec::new();
    let shell_ids: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();
    for sid in shell_ids {
        let shell = topo.shell(sid)?;
        faces.extend(shell.faces().iter().copied());
    }
    Ok(faces)
}

/// Collect edge segments as polylines for edge-edge distance computation.
///
/// Line edges produce a single segment. Curved edges (circle, ellipse, NURBS)
/// are sampled at multiple points to capture the curve geometry.
#[allow(clippy::cast_precision_loss)]
fn collect_solid_edge_segments(
    topo: &Topology,
    solid: SolidId,
) -> Result<Vec<(Point3, Point3)>, CheckError> {
    use remus_topology::edge::EdgeCurve;

    let solid_data = topo.solid(solid)?;
    let mut seen = HashSet::new();
    let mut segments = Vec::new();

    let n_samples = 8usize;

    let shell_ids: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();
    for sid in shell_ids {
        let shell = topo.shell(sid)?;
        for &fid in shell.faces() {
            let face = topo.face(fid)?;
            let mut wire_ids = vec![face.outer_wire()];
            wire_ids.extend(face.inner_wires().iter().copied());
            for wid in wire_ids {
                let wire = topo.wire(wid)?;
                for oe in wire.edges() {
                    let eid = oe.edge();
                    if !seen.insert(eid) {
                        continue;
                    }
                    let edge_data = topo.edge(eid)?;
                    let start_pt = topo.vertex(edge_data.start())?.point();
                    let end_pt = topo.vertex(edge_data.end())?.point();

                    match edge_data.curve() {
                        EdgeCurve::Line => {
                            segments.push((start_pt, end_pt));
                        }
                        EdgeCurve::Circle(c) => {
                            let is_closed = edge_data.start() == edge_data.end();
                            let (t0, t1) = if is_closed {
                                (0.0, std::f64::consts::TAU)
                            } else {
                                let t0 = c.project(start_pt);
                                let mut t1 = c.project(end_pt);
                                if t1 <= t0 {
                                    t1 += std::f64::consts::TAU;
                                }
                                (t0, t1)
                            };
                            let mut prev = c.evaluate(t0);
                            for i in 1..=n_samples {
                                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                                let curr = c.evaluate(t);
                                segments.push((prev, curr));
                                prev = curr;
                            }
                        }
                        EdgeCurve::Ellipse(e) => {
                            let is_closed = edge_data.start() == edge_data.end();
                            let (t0, t1) = if is_closed {
                                (0.0, std::f64::consts::TAU)
                            } else {
                                let t0 = e.project(start_pt);
                                let mut t1 = e.project(end_pt);
                                if t1 <= t0 {
                                    t1 += std::f64::consts::TAU;
                                }
                                (t0, t1)
                            };
                            let mut prev = e.evaluate(t0);
                            for i in 1..=n_samples {
                                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                                let curr = e.evaluate(t);
                                segments.push((prev, curr));
                                prev = curr;
                            }
                        }
                        EdgeCurve::Hyperbola(h) => {
                            // Unbounded branch: the vertices are the only
                            // trim, and `project` inverts the
                            // parameterization exactly, so the arc is the
                            // straight parameter interval — no periodic
                            // wrap-around to correct for.
                            let (t0, t1) = (h.project(start_pt), h.project(end_pt));
                            let mut prev = h.evaluate(t0);
                            for i in 1..=n_samples {
                                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                                let curr = h.evaluate(t);
                                segments.push((prev, curr));
                                prev = curr;
                            }
                        }
                        EdgeCurve::Parabola(p) => {
                            let (t0, t1) = (p.project(start_pt), p.project(end_pt));
                            let mut prev = p.evaluate(t0);
                            for i in 1..=n_samples {
                                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                                let curr = p.evaluate(t);
                                segments.push((prev, curr));
                                prev = curr;
                            }
                        }
                        EdgeCurve::NurbsCurve(nc) => {
                            let (t0, t1) = nc.domain();
                            let mut prev = nc.evaluate(t0);
                            for i in 1..=n_samples {
                                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                                let curr = nc.evaluate(t);
                                segments.push((prev, curr));
                                prev = curr;
                            }
                        }
                    }
                }
            }
        }
    }

    Ok(segments)
}

/// Check if a point lies within the face's boundary polygon, projected onto
/// the polygon's best-fit plane.
///
/// Only sound where that projection is injective. A full-turn curved face is
/// not: a cylinder wall's boundary flattens to a sliver, so most of the wall
/// read as off-face. The analytic arms use the UV trim test instead; the NURBS
/// arm still relies on this.
fn is_point_in_face_boundary(
    topo: &Topology,
    face_id: FaceId,
    point: Point3,
) -> Result<bool, CheckError> {
    let polygon = crate::util::face_polygon(topo, face_id)?;
    if polygon.len() < 3 {
        return Ok(true); // Full-surface face
    }
    let normal = crate::util::polygon_normal(&polygon);
    Ok(crate::util::point_in_polygon_3d(&point, &polygon, &normal))
}

/// Find the closest point on the wire edges of a face to a given point.
///
/// Iterates both the outer wire and inner wires (holes).
fn closest_point_on_wire_edges(
    topo: &Topology,
    face_id: FaceId,
    point: Point3,
) -> Result<Option<(f64, Point3)>, CheckError> {
    let face = topo.face(face_id)?;
    let mut best_dist = f64::INFINITY;
    let mut best_pt = point;

    let mut wire_ids = vec![face.outer_wire()];
    wire_ids.extend(face.inner_wires().iter().copied());

    for wid in wire_ids {
        let wire = topo.wire(wid)?;
        for oe in wire.edges() {
            let edge_data = topo.edge(oe.edge())?;
            let p0 = topo.vertex(edge_data.start())?.point();
            let p1 = topo.vertex(edge_data.end())?.point();
            let (dist, closest) = point_to_segment(point, p0, p1);
            if dist < best_dist {
                best_dist = dist;
                best_pt = closest;
            }
        }
    }
    if best_dist < f64::INFINITY {
        Ok(Some((best_dist, best_pt)))
    } else {
        Ok(None)
    }
}

/// Point-to-polygon distance for planar faces.
///
/// Projects the point onto the plane, checks if inside polygon, otherwise
/// finds the closest point on polygon edges.
fn point_to_polygon_distance(
    point: Point3,
    polygon: &[Point3],
    normal: Vec3,
    d: f64,
) -> Option<(f64, Point3)> {
    if polygon.len() < 3 {
        return None;
    }

    let (_, projected) = analytic::point_to_plane(point, normal, d);

    if crate::util::point_in_polygon_3d(&projected, polygon, &normal) {
        let dist = (point - projected).length();
        return Some((dist, projected));
    }

    let mut best_dist = f64::INFINITY;
    let mut best_pt = polygon[0];
    let n = polygon.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let (dist, closest) = point_to_segment(point, polygon[i], polygon[j]);
        if dist < best_dist {
            best_dist = dist;
            best_pt = closest;
        }
    }
    Some((best_dist, best_pt))
}

/// Distance from point to line segment.
fn point_to_segment(point: Point3, a: Point3, b: Point3) -> (f64, Point3) {
    let ab = b - a;
    let ap = point - a;
    let len_sq = ab.length_squared();
    if len_sq < 1e-30 {
        return ((point - a).length(), a);
    }
    let t = (ap.dot(ab) / len_sq).clamp(0.0, 1.0);
    let closest = Point3::new(
        ab.x().mul_add(t, a.x()),
        ab.y().mul_add(t, a.y()),
        ab.z().mul_add(t, a.z()),
    );
    ((point - closest).length(), closest)
}

/// Compute minimum distance between two AABBs.
fn aabb_distance(a: &Aabb3, b: &Aabb3) -> f64 {
    let dx = (a.min.x() - b.max.x()).max(b.min.x() - a.max.x()).max(0.0);
    let dy = (a.min.y() - b.max.y()).max(b.min.y() - a.max.y()).max(0.0);
    let dz = (a.min.z() - b.max.z()).max(b.min.z() - a.max.z()).max(0.0);
    (dx.mul_add(dx, dy.mul_add(dy, dz * dz))).sqrt()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::surfaces::{CylindricalSurface, SphericalSurface, ToroidalSurface};

    #[test]
    fn point_to_sphere_outside() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
        let (dist, closest) = analytic::point_to_sphere(Point3::new(3.0, 0.0, 0.0), &sphere);
        assert!(
            (dist - 2.0).abs() < 1e-10,
            "distance should be 2.0, got {dist}"
        );
        assert!((closest.x() - 1.0).abs() < 1e-10);
        assert!(closest.y().abs() < 1e-10);
        assert!(closest.z().abs() < 1e-10);
    }

    #[test]
    fn point_to_sphere_inside() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
        let (dist, closest) = analytic::point_to_sphere(Point3::new(0.5, 0.0, 0.0), &sphere);
        assert!(
            (dist - 0.5).abs() < 1e-10,
            "distance should be 0.5, got {dist}"
        );
        assert!((closest.x() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn point_to_cylinder_outside() {
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                .unwrap();
        let (dist, closest) = analytic::point_to_cylinder(Point3::new(2.0, 0.0, 0.0), &cyl);
        assert!(
            (dist - 1.0).abs() < 1e-10,
            "distance should be 1.0, got {dist}"
        );
        assert!((closest.x() - 1.0).abs() < 1e-10);
        assert!(closest.y().abs() < 1e-10);
        assert!(closest.z().abs() < 1e-10);
    }

    #[test]
    fn point_to_plane_above() {
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let d = 0.0;
        let (dist, closest) = analytic::point_to_plane(Point3::new(0.0, 0.0, 5.0), normal, d);
        assert!(
            (dist - 5.0).abs() < 1e-10,
            "distance should be 5.0, got {dist}"
        );
        assert!(closest.x().abs() < 1e-10);
        assert!(closest.y().abs() < 1e-10);
        assert!(closest.z().abs() < 1e-10);
    }

    #[test]
    fn segment_segment_parallel() {
        // Two parallel segments along X, separated by 2.0 in Y.
        let (dist, _, _) = edge::segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        );
        assert!(
            (dist - 2.0).abs() < 1e-10,
            "parallel segment distance should be 2.0, got {dist}"
        );
    }

    #[test]
    fn segment_segment_crossing() {
        // Two segments that cross: one along X, one along Y, both through origin.
        let (dist, _, _) = edge::segment_segment_distance(
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        assert!(
            dist < 1e-10,
            "crossing segment distance should be ~0, got {dist}"
        );
    }

    #[test]
    fn segment_segment_skew() {
        // Two skew segments separated by 3.0 in Z.
        let (dist, ca, cb) = edge::segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(0.0, 1.0, 3.0),
        );
        assert!(
            (dist - 3.0).abs() < 1e-10,
            "skew segment distance should be 3.0, got {dist}"
        );
        assert!(ca.z().abs() < 1e-10);
        assert!((cb.z() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn solid_to_solid_separated() {
        use remus_topology::test_utils::make_unit_cube_manifold_at;
        let mut topo = Topology::new();
        // Two unit cubes: one at origin, one at (3, 0, 0). Gap of 2.0 in X.
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let b = make_unit_cube_manifold_at(&mut topo, 3.0, 0.0, 0.0);
        let result = solid_to_solid(&topo, a, b).unwrap();
        assert!(
            (result.distance - 2.0).abs() < 1e-10,
            "distance should be 2.0, got {}",
            result.distance
        );
    }

    #[test]
    fn point_to_torus_outside() {
        // Torus at origin with major_radius=3, minor_radius=1, Z-axis.
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
        // Point at (6, 0, 0): major circle closest is (3,0,0), tube dist = 3, minor_r = 1.
        let (dist, closest) = analytic::point_to_torus(Point3::new(6.0, 0.0, 0.0), &torus);
        assert!(
            (dist - 2.0).abs() < 1e-10,
            "distance should be 2.0, got {dist}"
        );
        assert!((closest.x() - 4.0).abs() < 1e-10);
        assert!(closest.y().abs() < 1e-10);
        assert!(closest.z().abs() < 1e-10);
    }
}
