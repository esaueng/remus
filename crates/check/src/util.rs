//! Shared utility functions for the check crate.

use brepkit_math::aabb::Aabb3;
use brepkit_math::vec::{Point2, Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::face::{FaceId, FaceSurface};

use crate::CheckError;

/// Compute the normal of a polygon via Newell's method.
///
/// Returns a unit-length normal, or `(0,0,1)` for degenerate polygons.
#[must_use]
pub fn polygon_normal(verts: &[Point3]) -> Vec3 {
    let mut nx = 0.0;
    let mut ny = 0.0;
    let mut nz = 0.0;
    let n = verts.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let vi = verts[i];
        let vj = verts[j];
        nx += (vi.y() - vj.y()) * (vi.z() + vj.z());
        ny += (vi.z() - vj.z()) * (vi.x() + vj.x());
        nz += (vi.x() - vj.x()) * (vi.y() + vj.y());
    }
    let len = (nx.mul_add(nx, ny.mul_add(ny, nz * nz))).sqrt();
    if len < 1e-30 {
        Vec3::new(0.0, 0.0, 1.0)
    } else {
        Vec3::new(nx / len, ny / len, nz / len)
    }
}

/// Number of sample points for closed-curve edges.
pub const CLOSED_CURVE_SAMPLES: usize = 32;

/// Number of samples for an OPEN curved edge (arc or marched conic piece) in
/// wire polygons; sized for sub-degree angular steps on typical arcs.
pub const OPEN_CURVE_SAMPLES: usize = 32;

/// Sample a closed-edge curve at `n` evenly spaced parameter values.
///
/// Returns an empty vector for `Line` edges (geometry determined by
/// vertices) and for `Hyperbola` / `Parabola` edges, which are unbounded
/// and can never be closed — there is no periodic domain to sample and
/// this entry point carries no endpoints to trim with. Callers that need
/// the geometry of an open conic edge must go through
/// [`EdgeCurve::domain_with_endpoints`].
#[must_use]
pub fn sample_edge_curve(curve: &EdgeCurve, n: usize) -> Vec<Point3> {
    match curve {
        EdgeCurve::Circle(c) => (0..n)
            .map(|i| {
                let t = std::f64::consts::TAU * (i as f64) / (n as f64);
                c.evaluate(t)
            })
            .collect(),
        EdgeCurve::Ellipse(e) => (0..n)
            .map(|i| {
                let t = std::f64::consts::TAU * (i as f64) / (n as f64);
                e.evaluate(t)
            })
            .collect(),
        EdgeCurve::NurbsCurve(nc) => {
            let (u0, u1) = nc.domain();
            let start_pt = nc.evaluate(u0);
            let end_pt = nc.evaluate(u1);
            let is_closed = (start_pt - end_pt).length() < 1e-6;
            let divisor = if is_closed { n } else { n - 1 };
            (0..n)
                .map(|i| {
                    let t = u0 + (u1 - u0) * (i as f64) / (divisor as f64);
                    nc.evaluate(t)
                })
                .collect()
        }
        // Never closed: an unbounded branch has no periodic domain.
        EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => vec![],
    }
}

/// Build a polygon from the outer wire of a face by sampling vertex positions
/// and closed-edge curves.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the face is missing.
pub fn face_polygon(topo: &Topology, face_id: FaceId) -> Result<Vec<Point3>, CheckError> {
    let face = topo.face(face_id)?;
    wire_polygon(topo, face.outer_wire())
}

/// Build a polygon from a wire by sampling vertex positions plus closed and
/// open curved edges.
///
/// Wires store edges in loop order, but the per-edge orientation flags are
/// not guaranteed to chain head-to-tail; each edge's traversal direction is
/// re-derived from vertex connectivity with the previous edge so the polygon
/// follows the actual loop.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the wire is missing.
pub fn wire_polygon(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
) -> Result<Vec<Point3>, CheckError> {
    wire_polygon_curve_sampled(topo, wire_id, CLOSED_CURVE_SAMPLES, OPEN_CURVE_SAMPLES)
}

/// [`wire_polygon`] with the number of samples a closed curved edge
/// contributes chosen by the caller.
///
/// A loop that is *subtracted* from an otherwise exact measurement is worth
/// outlining more finely than one that merely has to be walked: the chord
/// error falls with the square of the step, so the property integrator asks
/// for several times the default when it removes a hole from a curved face.
///
/// An OPEN curved edge still contributes only its own endpoint here — one
/// chord for the whole arc. [`wire_polygon_curve_sampled`] is the form that
/// outlines those too.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the wire is missing.
pub fn wire_polygon_sampled(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
    closed_samples: usize,
) -> Result<Vec<Point3>, CheckError> {
    wire_polygon_curve_sampled(topo, wire_id, closed_samples, 1)
}

/// [`wire_polygon_sampled`] with the samples an OPEN curved edge contributes
/// chosen as well.
///
/// A closed circle edge is laid down as a polyline, but an open arc used to
/// contribute only its own endpoint — one chord across the whole arc, however
/// far the arc bows away from it. That is invisible on a wire that merely has
/// to be walked, and decisive on one that trims an integration domain: the
/// rolling-ball corner patch of a rounded offset is bounded by three quarter
/// great circles and nothing else, so chording each of them to its endpoints
/// shrinks the region by a quarter and reads the patch's area 25 % low.
///
/// `open_samples` of 1 reproduces the old single-chord behaviour exactly, and
/// is what [`wire_polygon_sampled`] passes.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the wire is missing.
pub fn wire_polygon_curve_sampled(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
    closed_samples: usize,
    open_samples: usize,
) -> Result<Vec<Point3>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let mut pts: Vec<Point3> = Vec::new();
    let mut prev_end: Option<brepkit_topology::vertex::VertexId> = None;

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let curve = edge.curve();
        let start_vid = edge.start();
        let end_vid = edge.end();
        let forward = match prev_end {
            Some(pe) if start_vid == pe && end_vid != pe => true,
            Some(pe) if end_vid == pe && start_vid != pe => false,
            // A closed edge's endpoints coincide, so positional chaining is
            // meaningless — keep the stored traversal flag (the partial-turn
            // torus band's rim phase coherence depends on it).
            _ if start_vid == end_vid => oe.is_forward(),
            // OPEN edges: consecutive wire edges can hold position-equal but
            // DISTINCT vertex ids (assembly refinement mints sub-edge
            // vertices from a different pool than the neighbours). Fall back
            // to positional chaining against the last emitted point; an
            // orientation-flag guess can bow-tie the polygon and flip
            // containment inside the mis-ordered region.
            _ => match prev_end {
                Some(previous_end) => {
                    let last = topo.vertex(previous_end)?.point();
                    let sp = topo.vertex(start_vid)?.point();
                    let ep = topo.vertex(end_vid)?.point();
                    (sp - last).length_squared() <= (ep - last).length_squared()
                }
                None => oe.is_forward(),
            },
        };
        let is_closed_edge = start_vid == end_vid
            && matches!(
                curve,
                EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_)
            );
        if is_closed_edge {
            // Start sampling at the edge's seam vertex so the polygon chains
            // cleanly with adjacent edges; the curve's own parameter origin
            // is unrelated to the vertex.
            let seam_pt = topo.vertex(start_vid)?.point();
            // Traversal must start at the seam vertex in both directions:
            // forward covers [t0, t0 + period), reversed covers (t0, t0 + period]
            // walked backwards — the next edge supplies the closing point.
            #[allow(clippy::cast_precision_loss)]
            let params = |n: usize, period: f64| -> Vec<f64> {
                if forward {
                    (0..n).map(|i| period * (i as f64) / (n as f64)).collect()
                } else {
                    (1..=n)
                        .rev()
                        .map(|i| period * (i as f64) / (n as f64))
                        .collect()
                }
            };
            let sampled: Vec<Point3> = match curve {
                EdgeCurve::Circle(c) => {
                    let t0 = c.project(seam_pt);
                    params(closed_samples, std::f64::consts::TAU)
                        .into_iter()
                        .map(|dt| c.evaluate(t0 + dt))
                        .collect()
                }
                EdgeCurve::Ellipse(e) => {
                    let t0 = e.project(seam_pt);
                    params(closed_samples, std::f64::consts::TAU)
                        .into_iter()
                        .map(|dt| e.evaluate(t0 + dt))
                        .collect()
                }
                EdgeCurve::NurbsCurve(nc) => {
                    let (u0, u1) = nc.domain();
                    let span = u1 - u0;
                    if span.is_finite() && span > 0.0 {
                        let t0 = nurbs_seam_parameter(nc, seam_pt, u0, u1);
                        params(closed_samples, span)
                            .into_iter()
                            .map(|dt| nc.evaluate(u0 + (t0 - u0 + dt).rem_euclid(span)))
                            .collect()
                    } else {
                        let mut s = sample_edge_curve(curve, closed_samples);
                        if !forward {
                            s.reverse();
                        }
                        s
                    }
                }
                // Unreachable: `is_closed_edge` above only admits Circle,
                // Ellipse and NurbsCurve. Hyperbola and parabola branches
                // are unbounded and never satisfy start == end.
                EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => vec![],
            };
            pts.extend(sampled);
            prev_end = Some(start_vid);
        } else {
            let (from_vid, to_vid) = if forward {
                (start_vid, end_vid)
            } else {
                (end_vid, start_vid)
            };
            let is_open_curve = open_samples > 1 && !matches!(curve, EdgeCurve::Line);
            if is_open_curve {
                // Walk the edge's own span, half-open so the next edge
                // supplies the closing point — the same convention the closed
                // branch above uses.
                let start_pt = topo.vertex(start_vid)?.point();
                let end_pt = topo.vertex(end_vid)?.point();
                let (t0, t1) = curve.domain_with_endpoints(start_pt, end_pt);
                let traversal_start = topo.vertex(from_vid)?.point();
                #[allow(clippy::cast_precision_loss)]
                let mut seq: Vec<Point3> = (0..=open_samples)
                    .map(|i| {
                        let t = (t1 - t0).mul_add(i as f64 / open_samples as f64, t0);
                        curve.evaluate_with_endpoints(t, start_pt, end_pt)
                    })
                    .collect();
                // A marched conic's stored curve direction can oppose its
                // vertex order. Orient the sampled span positionally to the
                // actual wire traversal, then leave the far endpoint for the
                // next edge so the polygon stays half-open.
                if (seq[0] - traversal_start).length_squared()
                    > (seq[open_samples] - traversal_start).length_squared()
                {
                    seq.reverse();
                }
                seq.pop();
                pts.extend(seq);
            } else {
                pts.push(topo.vertex(from_vid)?.point());
            }
            prev_end = Some(to_vid);
        }
    }

    Ok(pts)
}

/// Build a polygon for each inner wire (hole boundary) of a face.
///
/// A trimmed face is bounded by its outer wire *minus* every inner wire, so
/// containment tests that ignore these polygons treat holes as material.
/// Wires that sample to fewer than 3 points carry no area and are skipped.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the face is missing.
pub fn face_hole_polygons(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<Vec<Point3>>, CheckError> {
    face_hole_polygons_sampled(topo, face_id, CLOSED_CURVE_SAMPLES)
}

/// [`face_hole_polygons`] with the closed-curve sample count chosen by the
/// caller, as [`wire_polygon_sampled`] takes it.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the face is missing.
pub fn face_hole_polygons_sampled(
    topo: &Topology,
    face_id: FaceId,
    closed_samples: usize,
) -> Result<Vec<Vec<Point3>>, CheckError> {
    face_hole_polygons_curve_sampled(topo, face_id, closed_samples, 1)
}

/// [`face_hole_polygons_sampled`] with the samples an OPEN curved edge
/// contributes chosen as well, as [`wire_polygon_curve_sampled`] takes it.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the face is missing.
pub fn face_hole_polygons_curve_sampled(
    topo: &Topology,
    face_id: FaceId,
    closed_samples: usize,
    open_samples: usize,
) -> Result<Vec<Vec<Point3>>, CheckError> {
    let face = topo.face(face_id)?;
    let mut holes = Vec::with_capacity(face.inner_wires().len());
    for &wire_id in face.inner_wires() {
        let poly = wire_polygon_curve_sampled(topo, wire_id, closed_samples, open_samples)?;
        if poly.len() >= 3 {
            holes.push(poly);
        }
    }
    Ok(holes)
}

/// Expand an AABB to account for surface curvature that may extend beyond
/// the wire vertices.
///
/// Plane and Cone surfaces are bounded by their vertices, so this is a no-op
/// for those types. Sphere, Cylinder, Torus, and NURBS surfaces can bulge
/// beyond the vertex-derived bounding box.
pub fn expand_aabb_for_surface(aabb: &mut Aabb3, surface: &FaceSurface) {
    match surface {
        FaceSurface::Sphere(s) => {
            let c = s.center();
            let r = s.radius();
            aabb_include(aabb, Point3::new(c.x() - r, c.y() - r, c.z() - r));
            aabb_include(aabb, Point3::new(c.x() + r, c.y() + r, c.z() + r));
        }
        FaceSurface::Cylinder(c) => {
            let origin = c.origin();
            let axis = c.axis();
            let r = c.radius();
            let rx = r * (1.0 - axis.x() * axis.x()).max(0.0).sqrt();
            let ry = r * (1.0 - axis.y() * axis.y()).max(0.0).sqrt();
            let rz = r * (1.0 - axis.z() * axis.z()).max(0.0).sqrt();
            for corner in [aabb.min, aabb.max] {
                let rel = Vec3::new(
                    corner.x() - origin.x(),
                    corner.y() - origin.y(),
                    corner.z() - origin.z(),
                );
                let t = axis.dot(rel);
                let coa = Point3::new(
                    origin.x() + axis.x() * t,
                    origin.y() + axis.y() * t,
                    origin.z() + axis.z() * t,
                );
                aabb_include(aabb, Point3::new(coa.x() - rx, coa.y() - ry, coa.z() - rz));
                aabb_include(aabb, Point3::new(coa.x() + rx, coa.y() + ry, coa.z() + rz));
            }
        }
        FaceSurface::Torus(t) => {
            let c = t.center();
            let outer_r = t.major_radius() + t.minor_radius();
            let axis = t.z_axis();
            let axial_offset = Vec3::new(
                axis.x() * t.minor_radius(),
                axis.y() * t.minor_radius(),
                axis.z() * t.minor_radius(),
            );
            aabb_include(
                aabb,
                Point3::new(
                    c.x() - outer_r + axial_offset.x().min(0.0),
                    c.y() - outer_r + axial_offset.y().min(0.0),
                    c.z() - outer_r + axial_offset.z().min(0.0),
                ),
            );
            aabb_include(
                aabb,
                Point3::new(
                    c.x() + outer_r + axial_offset.x().max(0.0),
                    c.y() + outer_r + axial_offset.y().max(0.0),
                    c.z() + outer_r + axial_offset.z().max(0.0),
                ),
            );
        }
        FaceSurface::Nurbs(nurbs) => {
            let (u_min, u_max) = nurbs.domain_u();
            let (v_min, v_max) = nurbs.domain_v();
            let n_samples = 8;
            for iu in 0..=n_samples {
                let u = u_min + (u_max - u_min) * (iu as f64) / (n_samples as f64);
                for iv in 0..=n_samples {
                    let v = v_min + (v_max - v_min) * (iv as f64) / (n_samples as f64);
                    aabb_include(aabb, nurbs.evaluate(u, v));
                }
            }
        }
        FaceSurface::Plane { .. } | FaceSurface::Cone(_) => {}
    }
}

/// Include a single point in an AABB.
fn aabb_include(aabb: &mut Aabb3, p: Point3) {
    *aabb = aabb.union(Aabb3 { min: p, max: p });
}

/// Squared distance below which the seam vertex counts as coincident with
/// the curve's domain start (linear tolerance 1e-7, squared).
const SEAM_COINCIDENT_SQ: f64 = 1e-14;

/// Parameter of the seam vertex on a closed NURBS rim.
///
/// Closed NURBS edges normally place their seam vertex at the curve's domain
/// start; when they do not, sampling from the domain origin breaks phase
/// coherence with adjacent edges, so the vertex is projected onto the curve.
fn nurbs_seam_parameter(
    nc: &brepkit_math::nurbs::curve::NurbsCurve,
    seam_pt: Point3,
    u0: f64,
    u1: f64,
) -> f64 {
    if (nc.evaluate(u0) - seam_pt).length_squared() <= SEAM_COINCIDENT_SQ {
        return u0;
    }
    brepkit_math::nurbs::projection::project_point_to_curve(nc, seam_pt, 1e-9)
        .map_or(u0, |proj| proj.parameter.clamp(u0, u1))
}

/// Expand an AABB to cover the full extent of a curved edge.
///
/// Vertex endpoints alone under-represent curved edges — a closed circle
/// edge has ONE vertex, collapsing the box to a point and starving any
/// AABB prefilter (a plane cap bounded by a single rim circle was never
/// offered to the classifier's BVH, dropping its ray crossings). Circle
/// and ellipse use the exact full-curve extent (a conservative superset
/// for partial arcs); NURBS uses the control-point convex hull.
///
/// Hyperbola and parabola branches are unbounded, so there is no
/// full-curve extent to fall back on: they are bounded by the exact
/// degree-2 Bézier control triangle of the trimmed arc, which requires
/// the edge's `start`/`end` positions.
fn expand_aabb_for_curve(aabb: &mut Aabb3, curve: &EdgeCurve, start: Point3, end: Point3) {
    match curve {
        EdgeCurve::Line => {}
        EdgeCurve::Hyperbola(h) => {
            let (t0, t1) = (h.project(start), h.project(end));
            aabb_include(aabb, h.tangent_intersection(t0, t1));
        }
        EdgeCurve::Parabola(p) => {
            let (t0, t1) = (p.project(start), p.project(end));
            aabb_include(aabb, p.tangent_intersection(t0, t1));
        }
        EdgeCurve::Circle(c) => {
            let cen = c.center();
            let r = c.radius();
            let (u, v) = (c.u_axis(), c.v_axis());
            let ext = [
                r * u.x().hypot(v.x()),
                r * u.y().hypot(v.y()),
                r * u.z().hypot(v.z()),
            ];
            aabb_include(
                aabb,
                Point3::new(cen.x() - ext[0], cen.y() - ext[1], cen.z() - ext[2]),
            );
            aabb_include(
                aabb,
                Point3::new(cen.x() + ext[0], cen.y() + ext[1], cen.z() + ext[2]),
            );
        }
        EdgeCurve::Ellipse(e) => {
            let cen = e.center();
            let (a, b) = (e.semi_major(), e.semi_minor());
            let (u, v) = (e.u_axis(), e.v_axis());
            let ext = [
                (a * u.x()).hypot(b * v.x()),
                (a * u.y()).hypot(b * v.y()),
                (a * u.z()).hypot(b * v.z()),
            ];
            aabb_include(
                aabb,
                Point3::new(cen.x() - ext[0], cen.y() - ext[1], cen.z() - ext[2]),
            );
            aabb_include(
                aabb,
                Point3::new(cen.x() + ext[0], cen.y() + ext[1], cen.z() + ext[2]),
            );
        }
        EdgeCurve::NurbsCurve(nc) => {
            for &p in nc.control_points() {
                aabb_include(aabb, p);
            }
        }
    }
}

/// Compute the axis-aligned bounding box of a face.
///
/// Starts from the wire vertex positions, expands for curved-edge extent,
/// then expands for surface curvature (spheres, cylinders, tori, NURBS).
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the face is missing.
pub fn face_aabb(topo: &Topology, face_id: FaceId) -> Result<Aabb3, CheckError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut points = Vec::new();
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        points.push(topo.vertex(edge.start())?.point());
        points.push(topo.vertex(edge.end())?.point());
    }
    let mut aabb = Aabb3::try_from_points(points.iter().copied())
        .ok_or_else(|| CheckError::ClassificationFailed("face has no vertices".into()))?;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        expand_aabb_for_curve(
            &mut aabb,
            edge.curve(),
            topo.vertex(edge.start())?.point(),
            topo.vertex(edge.end())?.point(),
        );
    }
    expand_aabb_for_surface(&mut aabb, face.surface());
    Ok(aabb)
}

/// Test whether a 3D point lies inside a 3D polygon by projecting onto the
/// dominant axis plane (the plane most aligned with the polygon normal).
#[must_use]
pub fn point_in_polygon_3d(point: &Point3, polygon: &[Point3], normal: &Vec3) -> bool {
    use brepkit_math::predicates::point_in_polygon;

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

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use brepkit_geometry::convert::curve_to_nurbs::circle_to_nurbs;
    use brepkit_math::curves::Circle3D;
    use brepkit_topology::edge::Edge;
    use brepkit_topology::vertex::Vertex;
    use brepkit_topology::wire::{OrientedEdge, Wire};

    #[test]
    fn wire_polygon_anchors_closed_nurbs_rim_at_seam_vertex() {
        let radius = 2.0;
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius).unwrap();
        let nurbs = circle_to_nurbs(&circle, 0.0, std::f64::consts::TAU).unwrap();
        // Seam vertex deliberately away from the curve's domain start.
        let seam_pt = circle.evaluate(1.1);

        let mut topo = Topology::new();
        let v = topo.add_vertex(Vertex::new(seam_pt, 1e-7));
        let e = topo.add_edge(Edge::new(v, v, EdgeCurve::NurbsCurve(nurbs)));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());

        let pts = wire_polygon(&topo, wire).unwrap();
        assert_eq!(pts.len(), CLOSED_CURVE_SAMPLES);
        assert!(
            (pts[0] - seam_pt).length() < 1e-6,
            "first sample {:?} not anchored at seam {:?}",
            pts[0],
            seam_pt
        );
        for p in &pts {
            assert!(
                (p.x().hypot(p.y()) - radius).abs() < 1e-9,
                "sample off the rim circle: {p:?}"
            );
        }
    }
}
