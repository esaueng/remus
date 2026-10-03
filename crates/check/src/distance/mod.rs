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

pub use edge::point_to_edge;
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
///
/// Point queries involving NURBS surfaces or trims retain local numerical
/// projection estimates; solid-pair queries return only supported global
/// minimum certificates.
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
    let mut deferred_errors = Vec::new();

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
        let result = match point_to_face(topo, point, item.face) {
            Ok(result) => result,
            Err(error) => {
                evaluated += 1;
                failures += 1;
                deferred_errors.push((item.lower_sq, error));
                continue;
            }
        };
        if let Some((dist, closest)) = result {
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
    discharge_bounded_errors(deferred_errors, best_dist)?;
    ensure_distance_witness(best_dist, point, best_point)?;

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
/// General NURBS carriers use a local Newton projection; NURBS trims use local
/// numerical boundary estimates even on analytic faces. These are not global
/// extremum certificates. Planar line/circle trims use
/// analytic membership and boundary extrema.
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
    let projection = match face.surface() {
        FaceSurface::Plane { normal, d } => Some(analytic::point_to_plane(point, *normal, *d)),
        FaceSurface::Cylinder(cyl) => Some(analytic::point_to_cylinder(point, cyl)),
        FaceSurface::Cone(cone) => Some(analytic::point_to_cone(point, cone)),
        FaceSurface::Sphere(sphere) => Some(analytic::point_to_sphere(point, sphere)),
        FaceSurface::Torus(torus) => Some(analytic::point_to_torus(point, torus)),
        FaceSurface::Nurbs(surface) => {
            if let Some(plane) =
                remus_geometry::convert::certified_plane::certify_affine_nurbs_plane(
                    surface,
                    options.projection_tolerance,
                )
            {
                Some(analytic::point_to_plane(
                    point,
                    plane.normal(),
                    plane.offset(),
                ))
            } else {
                let projection = remus_math::nurbs::projection::project_point_to_surface(
                    surface,
                    point,
                    options.projection_tolerance,
                )
                .map_err(|error| {
                    CheckError::DistanceFailed(format!(
                        "NURBS face projection could not establish a distance witness: {error}"
                    ))
                })?;
                Some((projection.distance, projection.point))
            }
        }
    };
    let mut best = if let Some((distance, closest)) = projection {
        let inside = match face.surface() {
            FaceSurface::Plane { normal, .. } => {
                plane_point_in_face(topo, face_id, closest, *normal)?
            }
            FaceSurface::Nurbs(surface) => {
                if let Some(plane) =
                    remus_geometry::convert::certified_plane::certify_affine_nurbs_plane(
                        surface,
                        options.projection_tolerance,
                    )
                {
                    plane
                        .parameters(closest, options.projection_tolerance)
                        .is_some()
                        && plane_point_in_face(topo, face_id, closest, plane.normal())?
                } else {
                    crate::classify::surface_point_in_face(topo, face_id, closest)?
                }
            }
            FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => {
                crate::classify::surface_point_in_face(topo, face_id, closest)?
            }
        };
        inside.then_some((distance, closest))
    } else {
        None
    };
    if let Some(equator) = analytic::native_sphere_equator(topo, face_id)? {
        let closest = equator.evaluate(equator.project(point));
        let candidate = ((point - closest).length(), closest);
        if best.is_none_or(|(distance, _)| candidate.0 < distance) {
            best = Some(candidate);
        }
        if let Some((distance, closest)) = best {
            ensure_distance_witness(distance, point, closest)?;
        }
        return Ok(best);
    }
    // Inner wires are excluded from carrier membership, but their actual
    // curves are part of the boundary. Include them even for a valid carrier
    // projection: imported sewn curves may differ from their carrier.
    let local_estimate = matches!(face.surface(), FaceSurface::Nurbs(_));
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wid)?.edges() {
            // An analytic carrier can have a sewn NURBS trim. Its local curve
            // estimate must compete with the carrier projection, just as it
            // does on a freeform face; the stored endpoints remain candidates.
            let nurbs_trim = matches!(
                topo.edge(oriented.edge())?.curve(),
                remus_topology::edge::EdgeCurve::NurbsCurve(_)
            );
            let candidate = if local_estimate || nurbs_trim {
                edge::point_to_edge_estimate(topo, point, oriented.edge())?
            } else {
                edge::point_to_edge(topo, point, oriented.edge())?
            };
            if best.is_none_or(|(distance, _)| candidate.0 < distance) {
                best = Some(candidate);
            }
        }
    }
    if let Some((distance, closest)) = best {
        ensure_distance_witness(distance, point, closest)?;
    }
    Ok(best)
}

/// Exact planar trim membership for polygonal and circular boundaries.
/// The parity test visits the outer loop and every hole independently.
fn plane_point_in_face(
    topo: &Topology,
    face_id: FaceId,
    point: Point3,
    normal: Vec3,
) -> Result<bool, CheckError> {
    let face = topo.face(face_id)?;
    if !plane_point_in_wire(topo, face.outer_wire(), point, normal)? {
        return Ok(false);
    }
    for &hole in face.inner_wires() {
        if plane_point_in_wire(topo, hole, point, normal)? {
            return Ok(false);
        }
    }
    Ok(true)
}

#[allow(clippy::too_many_lines, clippy::float_cmp)] // Exact half-open interval endpoints.
fn plane_point_in_wire(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
    point: Point3,
    normal: Vec3,
) -> Result<bool, CheckError> {
    use remus_topology::edge::EdgeCurve;
    let wire = topo.wire(wire_id)?;
    let frame = remus_math::frame::Frame3::from_normal(point, normal)?;
    let mut crossings = 0;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let (a, b) = edge
            .strict_domain()
            .map_err(crate::error::edge_domain_validation)?;
        match edge.curve() {
            curve if edge::is_linear_curve(curve) => {
                let p = curve.evaluate_with_endpoints(a, start, end) - point;
                let q = curve.evaluate_with_endpoints(b, start, end) - point;
                let (py, qy) = (p.dot(frame.y), q.dot(frame.y));
                if (py > 0.0) != (qy > 0.0) {
                    let t = -py / (qy - py);
                    if (q.dot(frame.x) - p.dot(frame.x)).mul_add(t, p.dot(frame.x)) > 0.0 {
                        crossings += 1;
                    }
                }
            }
            EdgeCurve::Circle(circle) => {
                let center = circle.center() - point;
                let cos_y = circle.radius() * circle.u_axis().dot(frame.y);
                let sin_y = circle.radius() * circle.v_axis().dot(frame.y);
                let amplitude = cos_y.hypot(sin_y);
                let ordinate = -center.dot(frame.y) / amplitude;
                // A tangent contributes no parity change.
                if ordinate.abs() >= 1.0 {
                    continue;
                }
                let phase = sin_y.atan2(cos_y);
                let angle = ordinate.acos();
                let (lo, hi) = (a.min(b), a.max(b));
                for raw in [phase - angle, phase + angle] {
                    let period = std::f64::consts::TAU;
                    let t = raw + period * ((lo - raw) / period).ceil();
                    if t > hi {
                        continue;
                    }
                    let derivative = (-cos_y).mul_add(t.sin(), sin_y * t.cos());
                    let interior = t > lo && t < hi;
                    let lower_crossing = t == lo && derivative > 0.0;
                    let upper_crossing = t == hi && derivative < 0.0;
                    if (interior || lower_crossing || upper_crossing)
                        && (circle.evaluate(t) - point).dot(frame.x) > 0.0
                    {
                        crossings += 1;
                    }
                }
            }
            EdgeCurve::Line
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::NurbsCurve(_) => {
                return Err(CheckError::DistanceFailed(
                    "certified planar trim membership requires line or circle boundaries".into(),
                ));
            }
        }
    }
    Ok(crossings % 2 != 0)
}

/// Compute the minimum distance between solid boundaries.
///
/// Complete native spheres use global analytic face extrema. Straight-edged
/// planar faces (including certified affine NURBS planes) use all vertex-face,
/// edge-edge and edge-face crossing candidates, including cavity shells and
/// hole rims. Unsupported curved pairs are refused; sampled chords cannot
/// certify a global minimum. Contained bodies measure boundary separation,
/// matching point-to-solid boundary-distance semantics.
///
/// # Errors
/// Returns an error for missing/invalid topology or unsupported carriers.
pub fn solid_to_solid(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<DistanceResult, CheckError> {
    solid_to_solid_with_face_probe(topo, solid_a, solid_b, || {})
}

/// [`solid_to_solid`] with a callback for each evaluated point-face candidate.
/// This lets higher layers retain their distance-work instrumentation without
/// duplicating the geometric minimum solver.
///
/// # Errors
/// Returns the same errors as [`solid_to_solid`].
#[allow(clippy::too_many_lines)]
pub fn solid_to_solid_with_face_probe(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    mut face_probe: impl FnMut(),
) -> Result<DistanceResult, CheckError> {
    // Resolve all referenced entities before an analytic shortcut as well.
    let faces_a = collect_solid_faces(topo, solid_a)?;
    let faces_b = collect_solid_faces(topo, solid_b)?;
    for &face in faces_a.iter().chain(&faces_b) {
        face_bounds::face_bound(topo, face)?;
    }
    if solid_a == solid_b {
        let vertex = collect_solid_vertices(topo, solid_a)?
            .into_iter()
            .find(|point| {
                [point.x(), point.y(), point.z()]
                    .iter()
                    .all(|value| value.is_finite())
            })
            .ok_or_else(unsupported_solid_minimum)?;
        return Ok(DistanceResult {
            distance: 0.0,
            point_a: vertex,
            point_b: vertex,
        });
    }
    if let (Some(a), Some(b)) = (
        analytic::complete_sphere(topo, solid_a)?,
        analytic::complete_sphere(topo, solid_b)?,
    ) {
        return analytic::sphere_pair(&a, &b);
    }
    let planes_a = certify_planar_faces(topo, &faces_a)?;
    let planes_b = certify_planar_faces(topo, &faces_b)?;
    let verts_a = collect_solid_vertices(topo, solid_a)?;
    let verts_b = collect_solid_vertices(topo, solid_b)?;

    let mut best_dist = f64::INFINITY;
    let mut best_a = Point3::new(0.0, 0.0, 0.0);
    let mut best_b = Point3::new(0.0, 0.0, 0.0);

    // Pass 1: Vertex-vertex (cheap upper bound).
    for &pa in &verts_a {
        for &pb in &verts_b {
            let dist = (pa - pb).length();
            ensure_distance_witness(dist, pa, pb)?;
            if dist < best_dist {
                best_dist = dist;
                best_a = pa;
                best_b = pb;
            }
        }
    }

    // Pass 2: Vertices of A against faces of B.
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
            face_probe();
            if let Some((dist, closest)) = point_to_face(topo, pa, faces_b[idx])?
                && dist < best_dist
            {
                best_dist = dist;
                best_a = pa;
                best_b = closest;
            }
        }
    }

    // Pass 3: Vertices of B against faces of A.
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
            face_probe();
            if let Some((dist, closest)) = point_to_face(topo, pb, faces_a[idx])?
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
            ensure_distance_witness(dist, ca, cb)?;
            if dist < best_dist {
                best_dist = dist;
                best_a = ca;
                best_b = cb;
            }
        }
    }

    // Nonparallel planar face interiors may cross without any vertex lying
    // on the other face or any pair of boundary edges intersecting.
    for (edges, faces, planes, bounds) in [
        (&edges_a, &faces_b, &planes_b, &aabbs_b),
        (&edges_b, &faces_a, &planes_a, &aabbs_a),
    ] {
        for &(start, end) in edges {
            let edge_bound =
                Aabb3::try_from_points([start, end]).ok_or_else(unsupported_solid_minimum)?;
            for ((&face, &(normal, offset)), (_, bound)) in faces.iter().zip(planes).zip(bounds) {
                if aabb_distance(&edge_bound, bound) > 0.0 {
                    continue;
                }
                let signed = normal.dot(Vec3::new(start.x(), start.y(), start.z())) - offset;
                let denominator = normal.dot(end - start);
                if denominator == 0.0 {
                    continue;
                }
                let t = -signed / denominator;
                if (0.0..=1.0).contains(&t) {
                    let point = start + (end - start) * t;
                    if plane_point_in_face(topo, face, point, normal)? {
                        return Ok(DistanceResult {
                            distance: 0.0,
                            point_a: point,
                            point_b: point,
                        });
                    }
                }
            }
        }
    }

    ensure_distance_witness(best_dist, best_a, best_b)?;
    Ok(DistanceResult {
        distance: best_dist,
        point_a: best_a,
        point_b: best_b,
    })
}

/// Numerical failure is a typed refusal, never an infinite/NaN minimum or
/// a missing witness silently discarded from a potentially closer candidate.
fn ensure_distance_witness(distance: f64, a: Point3, b: Point3) -> Result<(), CheckError> {
    if !distance.is_finite()
        || ![a.x(), a.y(), a.z(), b.x(), b.y(), b.z()]
            .iter()
            .all(|value| value.is_finite())
    {
        return Err(CheckError::DistanceFailed(
            "distance extremum has no finite witness".into(),
        ));
    }
    Ok(())
}

/// A failed bounded narrow phase is irrelevant only when its certified lower
/// bound proves it cannot beat the final finite witness. Processing order
/// cannot turn a failure on a potentially closer face into a success.
fn discharge_bounded_errors(
    errors: Vec<(f64, CheckError)>,
    final_distance: f64,
) -> Result<(), CheckError> {
    for (lower_sq, error) in errors {
        if final_distance.is_finite() && lower_sq > final_distance * final_distance {
            continue;
        }
        return Err(error);
    }
    Ok(())
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

/// Certify the bounded planar scope before enumerating minimum candidates.
fn certify_planar_faces(topo: &Topology, faces: &[FaceId]) -> Result<Vec<(Vec3, f64)>, CheckError> {
    let mut planes = Vec::with_capacity(faces.len());
    for &face_id in faces {
        let face = topo.face(face_id)?;
        let affine = if let FaceSurface::Nurbs(surface) = face.surface() {
            remus_geometry::convert::certified_plane::certify_affine_nurbs_plane(surface, 1e-7)
        } else {
            None
        };
        let (normal, offset) = match face.surface() {
            FaceSurface::Plane { normal, d } => (*normal, *d),
            FaceSurface::Nurbs(_) => {
                let Some(plane) = &affine else {
                    return Err(unsupported_solid_minimum());
                };
                (plane.normal(), plane.offset())
            }
            FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => return Err(unsupported_solid_minimum()),
        };
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id)?;
            if !wire.is_closed() {
                return Err(unsupported_solid_minimum());
            }
            for (i, oriented) in wire.edges().iter().enumerate() {
                let edge = topo.edge(oriented.edge())?;
                let (a, b) = edge
                    .strict_domain()
                    .map_err(crate::error::edge_domain_validation)?;
                let next = topo.edge(wire.edges()[(i + 1) % wire.edges().len()].edge())?;
                if !edge::is_linear_curve(edge.curve())
                    || oriented.oriented_end(edge)
                        != wire.edges()[(i + 1) % wire.edges().len()].oriented_start(next)
                {
                    return Err(unsupported_solid_minimum());
                }
                for (vertex, parameter) in [(edge.start(), a), (edge.end(), b)] {
                    let point = topo.vertex(vertex)?.point();
                    let scale = offset
                        .abs()
                        .max(point.x().abs())
                        .max(point.y().abs())
                        .max(point.z().abs())
                        .max(1.0);
                    let roundoff = 128.0 * f64::EPSILON * scale;
                    if (edge.curve().evaluate_with_endpoints(
                        parameter,
                        topo.vertex(edge.start())?.point(),
                        topo.vertex(edge.end())?.point(),
                    ) - point)
                        .length()
                        > roundoff
                        || (normal.dot(Vec3::new(point.x(), point.y(), point.z())) - offset).abs()
                            > roundoff
                        || affine.as_ref().is_some_and(|plane| {
                            plane
                                .parameters(point, roundoff.max(plane.max_deviation()))
                                .is_none()
                        })
                    {
                        return Err(unsupported_solid_minimum());
                    }
                }
            }
        }
        planes.push((normal, offset));
    }
    Ok(planes)
}

fn unsupported_solid_minimum() -> CheckError {
    CheckError::DistanceFailed(
        "a certified solid boundary minimum requires complete sphere pairs or straight-edged planar faces (including certified affine NURBS planes)".into(),
    )
}

/// Only reached after every edge has been qualified as an actual line segment.
fn collect_solid_edge_segments(
    topo: &Topology,
    solid: SolidId,
) -> Result<Vec<(Point3, Point3)>, CheckError> {
    let mut seen = HashSet::new();
    let mut segments = Vec::new();
    for face_id in collect_solid_faces(topo, solid)? {
        let face = topo.face(face_id)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                if seen.insert(oriented.edge()) {
                    let edge = topo.edge(oriented.edge())?;
                    segments.push((
                        topo.vertex(edge.start())?.point(),
                        topo.vertex(edge.end())?.point(),
                    ));
                }
            }
        }
    }
    Ok(segments)
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
    fn circular_hole_membership_and_witnesses_are_not_polygon_samples() {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::Face;
        use remus_topology::shell::Shell;
        use remus_topology::solid::Solid;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};
        let mut topo = Topology::new();
        let mut circle_wire = |radius: f64| {
            let circle =
                Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius)
                    .unwrap();
            let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
            let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
            edge.set_trim(Some((0.0, std::f64::consts::TAU)));
            let edge = topo.add_edge(edge);
            topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
        };
        let outer = circle_wire(3.0);
        let hole = circle_wire(1.0);
        let face = topo.add_face(Face::new(
            outer,
            vec![hole],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let angle = 0.071_f64;
        let points = [
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(2.0, 0.0, 1.0),
            Point3::new(2.9999 * angle.cos(), 2.9999 * angle.sin(), 1.0),
        ];
        let expected = [2.0_f64.sqrt(), 1.0, 1.0];
        let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
        let mut scratch = DistanceScratch::new();
        let batch = point_to_solid_batch(&topo, &points, solid).unwrap();
        for (i, point) in points.into_iter().enumerate() {
            let direct = point_to_solid(&topo, point, solid).unwrap();
            let cached = prepared.query(point, &mut scratch).unwrap();
            for result in [&direct, &cached, &batch[i]] {
                assert!((result.distance - expected[i]).abs() < 1e-10, "{result:?}");
                let radius = result.point_b.x().hypot(result.point_b.y());
                assert!((1.0 - 1e-10..=3.0 + 1e-10).contains(&radius));
                assert!(result.point_b.z().abs() < 1e-10);
            }
        }
    }

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
