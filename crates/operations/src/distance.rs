//! Distance measurement between shapes.
//!
//! Computes minimum distance between solids and point-to-solid distance.
//! Supports planar, NURBS, and analytic (cylinder, cone, sphere, torus) faces
//! with BVH spatial acceleration.

#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_precision_loss,
    clippy::doc_markdown,
    clippy::module_name_repetitions,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::manual_let_else,
    clippy::needless_pass_by_value,
    clippy::imprecise_flops
)]

use remus_geometry::extrema::{
    point_to_cone as geo_point_to_cone, point_to_cylinder as geo_point_to_cylinder,
    point_to_sphere as geo_point_to_sphere, point_to_torus as geo_point_to_torus,
};
use remus_math::aabb::Aabb3;
use remus_math::bvh::Bvh;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::boolean::face_polygon;

/// Result of a distance computation.
#[derive(Debug, Clone)]
pub struct DistanceResult {
    /// The minimum distance found.
    pub distance: f64,
    /// The closest point on the first shape.
    pub point_a: Point3,
    /// The closest point on the second shape.
    pub point_b: Point3,
}

/// Compute the minimum distance from a point to a solid.
///
/// Uses BVH over face AABBs for acceleration. Dispatches per face type:
/// planar (point-to-polygon), NURBS (Newton projection), and analytic
/// (closed-form for cylinder/cone/sphere/torus).
///
/// # Errors
///
/// Returns an error if the solid is invalid.
#[allow(clippy::too_many_lines)]
pub fn point_to_solid_distance(
    topo: &Topology,
    point: Point3,
    solid: SolidId,
) -> Result<DistanceResult, crate::OperationsError> {
    let tol = Tolerance::new();

    // Solid-scoped: a hollow body's cavity walls are real boundary and must be
    // considered, so walk outer + inner shells (CLAUDE.md, "Walking faces in a
    // solid"). This query is not one of the per-shell exceptions.
    let face_ids: Vec<FaceId> = remus_topology::explorer::solid_faces(topo, solid)?;

    let (face_aabbs, prunable) = build_face_aabbs(topo, &face_ids)?;
    let bvh = Bvh::build(&face_aabbs);

    let mut best_dist = f64::INFINITY;
    let mut best_point = point;

    let candidates = bvh_distance_candidates(&bvh, &face_aabbs, point);

    for idx in candidates {
        let fid = face_ids[idx];
        let aabb_dist_sq = face_aabbs[idx].1.distance_squared_to_point(point);
        if prunable[idx] && aabb_dist_sq > best_dist * best_dist {
            continue;
        }

        if let Some((dist, closest)) = point_to_face_distance(topo, point, fid, tol)?
            && dist < best_dist
        {
            best_dist = dist;
            best_point = closest;
        }
    }

    Ok(DistanceResult {
        distance: best_dist,
        point_a: point,
        point_b: best_point,
    })
}

/// Compute the minimum distance between two solids.
///
/// Checks vertices of each solid against faces of the other, with
/// BVH acceleration. Also checks edge-to-edge distances for the
/// closest vertex pairs.
///
/// # Errors
///
/// Returns an error if either solid is invalid.
#[allow(clippy::too_many_lines)]
pub fn solid_to_solid_distance(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<DistanceResult, crate::OperationsError> {
    let tol = Tolerance::new();

    let verts_a = collect_solid_points(topo, solid_a)?;
    let verts_b = collect_solid_points(topo, solid_b)?;

    let mut best_dist = f64::INFINITY;
    let mut best_a = Point3::new(0.0, 0.0, 0.0);
    let mut best_b = Point3::new(0.0, 0.0, 0.0);

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

    // Vertices of A against faces of B.
    // Solid-scoped: a hollow body's cavity walls are real boundary and must be
    // considered, so walk outer + inner shells (CLAUDE.md, "Walking faces in a
    // solid"). This query is not one of the per-shell exceptions.
    let faces_b: Vec<FaceId> = remus_topology::explorer::solid_faces(topo, solid_b)?;
    let (aabbs_b, prunable_b) = build_face_aabbs(topo, &faces_b)?;
    let bvh_b = Bvh::build(&aabbs_b);

    for &pa in &verts_a {
        let candidates = bvh_distance_candidates(&bvh_b, &aabbs_b, pa);
        for idx in candidates {
            let aabb_dist_sq = aabbs_b[idx].1.distance_squared_to_point(pa);
            if prunable_b[idx] && aabb_dist_sq > best_dist * best_dist {
                continue;
            }
            if let Some((dist, closest)) = point_to_face_distance(topo, pa, faces_b[idx], tol)?
                && dist < best_dist
            {
                best_dist = dist;
                best_a = pa;
                best_b = closest;
            }
        }
    }

    // Vertices of B against faces of A.
    // Solid-scoped: a hollow body's cavity walls are real boundary and must be
    // considered, so walk outer + inner shells (CLAUDE.md, "Walking faces in a
    // solid"). This query is not one of the per-shell exceptions.
    let faces_a: Vec<FaceId> = remus_topology::explorer::solid_faces(topo, solid_a)?;
    let (aabbs_a, prunable_a) = build_face_aabbs(topo, &faces_a)?;
    let bvh_a = Bvh::build(&aabbs_a);

    for &pb in &verts_b {
        let candidates = bvh_distance_candidates(&bvh_a, &aabbs_a, pb);
        for idx in candidates {
            let aabb_dist_sq = aabbs_a[idx].1.distance_squared_to_point(pb);
            if prunable_a[idx] && aabb_dist_sq > best_dist * best_dist {
                continue;
            }
            if let Some((dist, closest)) = point_to_face_distance(topo, pb, faces_a[idx], tol)?
                && dist < best_dist
            {
                best_dist = dist;
                best_a = closest;
                best_b = pb;
            }
        }
    }

    // Edge-to-edge pass for closest edge pairs.
    let edges_a = collect_solid_edges(topo, solid_a)?;
    let edges_b = collect_solid_edges(topo, solid_b)?;

    for &(a1, a2) in &edges_a {
        for &(b1, b2) in &edges_b {
            let (dist, ca, cb) = segment_to_segment_distance(a1, a2, b1, b2);
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

/// Compute the minimum distance from a point to a face.
///
/// # Errors
///
/// Returns an error if the face lookup fails.
pub fn point_to_face(
    topo: &Topology,
    point: Point3,
    face_id: FaceId,
) -> Result<DistanceResult, crate::OperationsError> {
    let tol = Tolerance::new();
    if let Some((dist, closest)) = point_to_face_distance(topo, point, face_id, tol)? {
        Ok(DistanceResult {
            distance: dist,
            point_a: point,
            point_b: closest,
        })
    } else {
        // Fallback: distance to closest wire vertex
        let face = topo.face(face_id)?;
        let wire = topo.wire(face.outer_wire())?;
        let mut best = f64::INFINITY;
        let mut best_pt = point;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let vp = topo.vertex(edge.start())?.point();
            let d = (point - vp).length();
            if d < best {
                best = d;
                best_pt = vp;
            }
        }
        Ok(DistanceResult {
            distance: best,
            point_a: point,
            point_b: best_pt,
        })
    }
}

/// Compute the minimum distance from a point to an edge.
///
/// For line edges, uses exact point-to-segment distance.
/// For curved edges, samples the curve and returns the closest sample.
///
/// # Errors
///
/// Returns an error if the edge lookup fails.
#[allow(clippy::cast_precision_loss)]
pub fn point_to_edge(
    topo: &Topology,
    point: Point3,
    edge_id: remus_topology::edge::EdgeId,
) -> Result<DistanceResult, crate::OperationsError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();

    if matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
        let closest = closest_point_on_segment(point, start, end);
        let dist = (point - closest).length();
        Ok(DistanceResult {
            distance: dist,
            point_a: point,
            point_b: closest,
        })
    } else {
        let (t0, t1) = match edge.curve() {
            remus_topology::edge::EdgeCurve::NurbsCurve(nc) => nc.domain(),
            remus_topology::edge::EdgeCurve::Circle(c) => {
                if edge.is_closed() {
                    (0.0, std::f64::consts::TAU)
                } else {
                    // Project start/end vertices to get actual arc parameter range.
                    let mut t0 = c.project(start);
                    let mut t1 = c.project(end);
                    if t0 < 0.0 {
                        t0 += std::f64::consts::TAU;
                    }
                    if t1 <= t0 {
                        t1 += std::f64::consts::TAU;
                    }
                    (t0, t1)
                }
            }
            remus_topology::edge::EdgeCurve::Ellipse(e) => {
                if edge.is_closed() {
                    (0.0, std::f64::consts::TAU)
                } else {
                    let mut t0 = e.project(start);
                    let mut t1 = e.project(end);
                    if t0 < 0.0 {
                        t0 += std::f64::consts::TAU;
                    }
                    if t1 <= t0 {
                        t1 += std::f64::consts::TAU;
                    }
                    (t0, t1)
                }
            }
            // Unbounded branches: `project` inverts the parameterization
            // exactly, so the vertices give the arc's true parameter span.
            remus_topology::edge::EdgeCurve::Hyperbola(h) => (h.project(start), h.project(end)),
            remus_topology::edge::EdgeCurve::Parabola(p) => (p.project(start), p.project(end)),
            // Line was handled above (early return via `if` branch).
            remus_topology::edge::EdgeCurve::Line => (0.0, 0.0),
        };
        let n_samples = 64;
        let mut best_dist = f64::INFINITY;
        let mut best_pt = start;
        for i in 0..=n_samples {
            let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
            let pt = match edge.curve() {
                remus_topology::edge::EdgeCurve::NurbsCurve(nc) => nc.evaluate(t),
                remus_topology::edge::EdgeCurve::Circle(c) => c.evaluate(t),
                remus_topology::edge::EdgeCurve::Ellipse(e) => e.evaluate(t),
                remus_topology::edge::EdgeCurve::Hyperbola(h) => h.evaluate(t),
                remus_topology::edge::EdgeCurve::Parabola(p) => p.evaluate(t),
                // Line was handled above.
                remus_topology::edge::EdgeCurve::Line => start,
            };
            let d = (point - pt).length();
            if d < best_dist {
                best_dist = d;
                best_pt = pt;
            }
        }
        Ok(DistanceResult {
            distance: best_dist,
            point_a: point,
            point_b: best_pt,
        })
    }
}

/// Closest point on a line segment to a point.
fn closest_point_on_segment(point: Point3, a: Point3, b: Point3) -> Point3 {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq < 1e-30 {
        return a;
    }
    let ap = point - a;
    let t = ap.dot(ab) / len_sq;
    let t = t.clamp(0.0, 1.0);
    a + ab * t
}

/// Compute the distance from a point to a single face, dispatching by type.
pub(crate) fn point_to_face_distance(
    topo: &Topology,
    point: Point3,
    face_id: FaceId,
    tol: Tolerance,
) -> Result<Option<(f64, Point3)>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let projection = match face.surface() {
        FaceSurface::Plane { normal, d } => {
            let verts = face_polygon(topo, face_id)?;
            point_to_polygon_distance(point, &verts, *normal, *d, tol)
        }
        FaceSurface::Nurbs(surface) => {
            let proj =
                remus_math::nurbs::projection::project_point_to_surface(surface, point, tol.linear);
            match proj {
                Ok(p) => Some((p.distance, p.point)),
                Err(_) => None,
            }
        }
        FaceSurface::Cylinder(cyl) => Some(point_to_cylinder(point, cyl)),
        FaceSurface::Cone(cone) => Some(point_to_cone(point, cone)),
        FaceSurface::Sphere(sph) => Some(point_to_sphere(point, sph)),
        FaceSurface::Torus(tor) => Some(point_to_torus(point, tor)),
    };
    if let Some((distance, closest)) = projection
        && remus_check::classify::surface_point_in_face(topo, face_id, closest)?
    {
        return Ok(Some((distance, closest)));
    }

    let mut best: Option<(f64, Point3)> = None;
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wid)?.edges() {
            let candidate = closest_boundary_point(topo, point, oriented.edge())?;
            if best.is_none_or(|(distance, _)| candidate.0 < distance) {
                best = Some(candidate);
            }
        }
    }
    Ok(best)
}

fn closest_boundary_point(
    topo: &Topology,
    point: Point3,
    edge_id: remus_topology::edge::EdgeId,
) -> Result<(f64, Point3), crate::OperationsError> {
    use remus_geometry::extrema::point_to_curve;
    use remus_topology::edge::EdgeCurve;

    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    if matches!(edge.curve(), EdgeCurve::Line) {
        let closest = closest_point_on_segment(point, start, end);
        return Ok(((point - closest).length(), closest));
    }
    let (a, b) = edge
        .strict_domain()
        .map_err(|error| crate::OperationsError::InvalidInput {
            reason: error.to_string(),
        })?;
    let (lo, hi) = (a.min(b), a.max(b));
    let closest = match edge.curve() {
        EdgeCurve::Circle(circle) => {
            let raw = circle.project(point);
            let center = lo + (hi - lo) * 0.5;
            let period = std::f64::consts::TAU;
            let t = raw + period * ((center - raw) / period).round();
            circle.evaluate(t.clamp(lo, hi))
        }
        EdgeCurve::Ellipse(curve) => point_to_curve(point, curve, lo, hi).point,
        EdgeCurve::NurbsCurve(curve) => point_to_curve(point, curve, lo, hi).point,
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
            let curve = BoundaryCurve {
                curve: edge.curve(),
                start,
                end,
                domain: (lo, hi),
            };
            point_to_curve(point, &curve, lo, hi).point
        }
        EdgeCurve::Line => closest_point_on_segment(point, start, end),
    };
    Ok(((point - closest).length(), closest))
}

struct BoundaryCurve<'a> {
    curve: &'a remus_topology::edge::EdgeCurve,
    start: Point3,
    end: Point3,
    domain: (f64, f64),
}

impl remus_math::traits::ParametricCurve for BoundaryCurve<'_> {
    fn evaluate(&self, t: f64) -> Point3 {
        self.curve.evaluate_with_endpoints(t, self.start, self.end)
    }

    fn tangent(&self, t: f64) -> Vec3 {
        self.curve.tangent_with_endpoints(t, self.start, self.end)
    }

    fn domain(&self) -> (f64, f64) {
        self.domain
    }
}

// -- Analytic point-to-surface distance (delegating to remus_geometry) ------

/// Closest point on a cylinder to a given point.
fn point_to_cylinder(
    point: Point3,
    cyl: &remus_math::surfaces::CylindricalSurface,
) -> (f64, Point3) {
    let proj = geo_point_to_cylinder(point, cyl);
    (proj.distance, proj.point)
}

/// Closest point on a cone to a given point.
fn point_to_cone(point: Point3, cone: &remus_math::surfaces::ConicalSurface) -> (f64, Point3) {
    let proj = geo_point_to_cone(point, cone);
    (proj.distance, proj.point)
}

/// Closest point on a sphere to a given point.
fn point_to_sphere(
    point: Point3,
    sphere: &remus_math::surfaces::SphericalSurface,
) -> (f64, Point3) {
    let proj = geo_point_to_sphere(point, sphere);
    (proj.distance, proj.point)
}

/// Closest point on a torus to a given point.
fn point_to_torus(point: Point3, torus: &remus_math::surfaces::ToroidalSurface) -> (f64, Point3) {
    let proj = geo_point_to_torus(point, torus);
    (proj.distance, proj.point)
}

// -- BVH helpers --------------------------------------------------------------

/// Vertex bounds order candidates; only planar, straight-edged bounds can prune.
#[allow(clippy::type_complexity)]
fn build_face_aabbs(
    topo: &Topology,
    face_ids: &[FaceId],
) -> Result<(Vec<(usize, Aabb3)>, Vec<bool>), crate::OperationsError> {
    let mut result = Vec::with_capacity(face_ids.len());
    let mut prunable = Vec::with_capacity(face_ids.len());
    for (i, &fid) in face_ids.iter().enumerate() {
        let face = topo.face(fid)?;
        let wire = topo.wire(face.outer_wire())?;
        let mut polygonal = face.surface().is_planar();
        let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            polygonal &= matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line);
            for vid in [edge.start(), edge.end()] {
                let p = topo.vertex(vid)?.point();
                min = Point3::new(min.x().min(p.x()), min.y().min(p.y()), min.z().min(p.z()));
                max = Point3::new(max.x().max(p.x()), max.y().max(p.y()), max.z().max(p.z()));
            }
        }
        // Curved faces can extend arbitrarily far beyond seam vertices.
        // Their boxes are ordering hints, never distance lower bounds.
        let margin = 0.01;
        min = Point3::new(min.x() - margin, min.y() - margin, min.z() - margin);
        max = Point3::new(max.x() + margin, max.y() + margin, max.z() + margin);
        result.push((i, Aabb3 { min, max }));
        prunable.push(polygonal);
    }
    Ok((result, prunable))
}

/// Get candidate face indices sorted by AABB distance to a point.
fn bvh_distance_candidates(bvh: &Bvh, aabbs: &[(usize, Aabb3)], point: Point3) -> Vec<usize> {
    // For simplicity, query all faces and sort by AABB distance.
    let mut candidates: Vec<(usize, f64)> = aabbs
        .iter()
        .map(|(i, aabb)| (*i, aabb.distance_squared_to_point(point)))
        .collect();
    candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    if let Some(closest_idx) = bvh.query_closest(point)
        && let Some(pos) = candidates.iter().position(|(i, _)| *i == closest_idx)
    {
        candidates.swap(0, pos);
    }

    candidates.into_iter().map(|(i, _)| i).collect()
}

// -- Segment-to-segment distance ----------------------------------------------

/// Compute the minimum distance between two 3D line segments.
///
/// Delegates to [`remus_geometry::extrema::segment_segment_distance`].
fn segment_to_segment_distance(
    a1: Point3,
    a2: Point3,
    b1: Point3,
    b2: Point3,
) -> (f64, Point3, Point3) {
    remus_geometry::extrema::segment_segment_distance(a1, a2, b1, b2)
}

/// Collect all edge segments from a solid.
fn collect_solid_edges(
    topo: &Topology,
    solid: SolidId,
) -> Result<Vec<(Point3, Point3)>, crate::OperationsError> {
    let mut seen = std::collections::HashSet::new();
    let mut edges = Vec::new();

    // Solid-scoped: a hollow body's cavity walls are real boundary and must be
    // considered, so walk outer + inner shells (CLAUDE.md, "Walking faces in a
    // solid"). This query is not one of the per-shell exceptions.
    // Inner wires matter for the same reason: a hole rim is boundary too.
    for fid in remus_topology::explorer::solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                if seen.insert(oe.edge().index()) {
                    let edge = topo.edge(oe.edge())?;
                    let p1 = topo.vertex(edge.start())?.point();
                    let p2 = topo.vertex(edge.end())?.point();
                    edges.push((p1, p2));
                }
            }
        }
    }

    Ok(edges)
}

// -- Existing helpers (preserved) ---------------------------------------------

/// Compute the distance from a point to a planar polygon.
///
/// Returns `(distance, closest_point)` or `None` if the polygon is degenerate.
fn point_to_polygon_distance(
    point: Point3,
    verts: &[Point3],
    normal: Vec3,
    d: f64,
    _tol: Tolerance,
) -> Option<(f64, Point3)> {
    if verts.len() < 3 {
        return None;
    }

    let signed_dist = normal.dot(Vec3::new(point.x(), point.y(), point.z())) - d;
    let projected = Point3::new(
        (-normal.x()).mul_add(signed_dist, point.x()),
        (-normal.y()).mul_add(signed_dist, point.y()),
        (-normal.z()).mul_add(signed_dist, point.z()),
    );

    if point_in_polygon_3d(&projected, verts, &normal) {
        return Some((signed_dist.abs(), projected));
    }

    let mut best_dist = f64::INFINITY;
    let mut best_point = verts[0];
    let n = verts.len();

    for i in 0..n {
        let j = (i + 1) % n;
        let (dist, closest) = point_to_segment_distance(point, verts[i], verts[j]);
        if dist < best_dist {
            best_dist = dist;
            best_point = closest;
        }
    }

    Some((best_dist, best_point))
}

/// Point-in-polygon test for 3D (projecting to 2D).
pub(crate) fn point_in_polygon_3d(point: &Point3, polygon: &[Point3], normal: &Vec3) -> bool {
    use remus_math::predicates::point_in_polygon;
    use remus_math::vec::Point2;

    let ax = normal.x().abs();
    let ay = normal.y().abs();
    let az = normal.z().abs();

    let (proj_pt, proj_poly): (Point2, Vec<Point2>) = if az >= ax && az >= ay {
        (
            Point2::new(point.x(), point.y()),
            polygon.iter().map(|p| Point2::new(p.x(), p.y())).collect(),
        )
    } else if ay >= ax {
        (
            Point2::new(point.x(), point.z()),
            polygon.iter().map(|p| Point2::new(p.x(), p.z())).collect(),
        )
    } else {
        (
            Point2::new(point.y(), point.z()),
            polygon.iter().map(|p| Point2::new(p.y(), p.z())).collect(),
        )
    };

    point_in_polygon(proj_pt, &proj_poly)
}

/// Distance from a point to a line segment.
fn point_to_segment_distance(point: Point3, a: Point3, b: Point3) -> (f64, Point3) {
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

/// Collect all unique vertex positions from a solid.
fn collect_solid_points(
    topo: &Topology,
    solid: SolidId,
) -> Result<Vec<Point3>, crate::OperationsError> {
    let mut seen = std::collections::HashSet::new();
    let mut points = Vec::new();

    // Solid-scoped: a hollow body's cavity walls are real boundary and must be
    // considered, so walk outer + inner shells (CLAUDE.md, "Walking faces in a
    // solid"). This query is not one of the per-shell exceptions.
    // Inner wires matter for the same reason: a hole rim is boundary too.
    for fid in remus_topology::explorer::solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge())?;
                for vid in [edge.start(), edge.end()] {
                    if seen.insert(vid.index()) {
                        points.push(topo.vertex(vid)?.point());
                    }
                }
            }
        }
    }

    Ok(points)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::tolerance::Tolerance;
    use remus_math::vec::Point3;
    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_cube_manifold_at;

    use super::*;

    /// A hollow solid's cavity wall is boundary. Walking only `outer_shell()`
    /// answers this with the distance to the OUTER wall, which is both wrong and
    /// larger — the failure mode CLAUDE.md's "Walking faces in a solid" warns
    /// about, and it was live here.
    #[test]
    fn distance_sees_the_cavity_wall_of_a_hollow_solid() {
        let mut topo = Topology::new();
        let block = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        // A fully-interior tool leaves a void: the result carries an inner shell
        // spanning 3.0 ..= 7.0 in every axis.
        let void = crate::primitives::make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            void,
            &remus_math::mat::Mat4::translation(3.0, 3.0, 3.0),
        )
        .unwrap();
        let hollow =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, block, void)
                .unwrap();
        assert!(
            !topo.solid(hollow).unwrap().inner_shells().is_empty(),
            "test needs a solid that actually has a cavity shell"
        );

        // From the cavity centre the nearest boundary is the cavity wall at 2.0.
        // Outer-shell-only walking reports 5.0 (the outer wall).
        let result = point_to_solid_distance(&topo, Point3::new(5.0, 5.0, 5.0), hollow).unwrap();
        assert!(
            (result.distance - 2.0).abs() < 1e-6,
            "expected the cavity wall at 2.0, got {} (5.0 means only the outer shell was walked)",
            result.distance
        );
    }

    #[test]
    fn point_inside_cube_distance_is_half() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);

        // Point at center of cube — closest face is 0.5 away.
        let result = point_to_solid_distance(&topo, Point3::new(0.5, 0.5, 0.5), cube).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 0.5),
            "center-to-face distance should be ~0.5, got {}",
            result.distance
        );
    }

    #[test]
    fn point_outside_cube_distance() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);

        // Point above the cube.
        let result = point_to_solid_distance(&topo, Point3::new(0.5, 0.5, 3.0), cube).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 2.0),
            "point 2 above cube top should be distance ~2.0, got {}",
            result.distance
        );
    }

    #[test]
    fn disjoint_cubes_distance() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let b = make_unit_cube_manifold_at(&mut topo, 5.0, 0.0, 0.0);

        let result = solid_to_solid_distance(&topo, a, b).unwrap();
        let tol = Tolerance::loose();
        // Cubes are [0,1] and [5,6], gap is 4.0.
        assert!(
            tol.approx_eq(result.distance, 4.0),
            "disjoint cubes should be ~4.0 apart, got {}",
            result.distance
        );
    }

    #[test]
    fn adjacent_cubes_distance_is_zero() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let b = make_unit_cube_manifold_at(&mut topo, 1.0, 0.0, 0.0);

        let result = solid_to_solid_distance(&topo, a, b).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 0.0),
            "touching cubes should have distance ~0, got {}",
            result.distance
        );
    }

    #[test]
    fn same_solid_distance_is_zero() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);

        let result = solid_to_solid_distance(&topo, a, a).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 0.0),
            "distance to self should be 0, got {}",
            result.distance
        );
    }

    #[test]
    fn point_to_sphere_distance() {
        let sphere =
            remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0).unwrap();
        let (dist, closest) = point_to_sphere(Point3::new(10.0, 0.0, 0.0), &sphere);
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 5.0),
            "distance to sphere should be ~5.0, got {dist}"
        );
        assert!(
            tol.approx_eq(closest.x(), 5.0),
            "closest x should be ~5.0, got {}",
            closest.x()
        );
    }

    #[test]
    fn point_to_cylinder_distance() {
        let cyl = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
        )
        .unwrap();
        let (dist, _closest) = point_to_cylinder(Point3::new(5.0, 0.0, 1.0), &cyl);
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 2.0),
            "distance to cylinder should be ~2.0, got {dist}"
        );
    }

    #[test]
    fn segment_to_segment_parallel() {
        let (dist, _, _) = segment_to_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
        );
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 3.0),
            "parallel segments 3 apart should have distance ~3.0, got {dist}"
        );
    }

    #[test]
    fn segment_to_segment_crossing() {
        let (dist, _, _) = segment_to_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, -1.0),
            Point3::new(0.5, 0.0, 1.0),
        );
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 0.0),
            "crossing segments should have distance ~0, got {dist}"
        );
    }
}
