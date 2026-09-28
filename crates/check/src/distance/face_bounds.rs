//! Conservative face bounds for branch-and-bound distance queries.
//!
//! [`FaceBound`] pairs an axis-aligned box with a prunability flag. A
//! prunable (`Conservative`) box is certified to lower-bound what the
//! distance narrow phase ([`super::point_to_face`]) can return for the face,
//! so candidates may be skipped once the box is farther than the current
//! best distance. A non-prunable (`Unknown`) box is a non-empty fallback and
//! the face must stay on the mandatory exhaustive side path.
//!
//! # Soundness argument
//!
//! The bound needs to contain every point the narrow phase can return, which
//! is a weaker obligation than containing the true face — and it is the
//! correct one, because branch-and-bound must reproduce the exhaustive
//! result *of the same narrow phase*, not certify global minimization (a
//! local Newton projection does not become globally exact through BVH use).
//!
//! The narrow phase returns, per face, either a carrier projection accepted
//! by the narrow-phase trim predicate (the same function the narrow phase
//! calls, reused here for predicate-consistency) or a closest point on a
//! wire segment (a straight chord between stored vertices). The face bound
//! therefore covers:
//!
//! - every wire curve, via the geometry-layer span bounds over the edge's
//!   authoritative trim (`Edge::strict_domain`), unioned with the stored
//!   vertex positions (vertices can sit off-curve within sewing tolerance
//!   and the narrow phase walks vertex chords, which the convex box then
//!   contains);
//! - the carrier interior, per surface type:
//!   - *Plane*: a linear coordinate over a planar patch attains its maximum
//!     on the boundary (a non-constant linear function has no interior
//!     critical point), so the wire bound suffices;
//!   - *Cylinder*: a linear coordinate has no interior critical point on a
//!     cylinder (the axis is never parallel to the radial normal), and along
//!     a critical generator line the coordinate is constant, hence also
//!     attained where the line meets the boundary — the wire bound suffices;
//!   - *Cone*: as for the cylinder, plus the apex (the only singular
//!     point), included when the trim predicate accepts it;
//!   - *Sphere*: the six axis poles (the only interior critical points of
//!     world coordinates), each included when the trim predicate accepts it;
//!   - *Torus*: coordinate maxima along the torus axis are attained on
//!     critical *rings*, which a single test point cannot certify, so the
//!     box always spans the full axial range (loose but sound; see the
//!     `torus_ring_counterexample` test). Equatorial coordinates have
//!     isolated critical points, included when the predicate accepts them.
//!     Tilted tori (axis away from every world axis) fall back to the
//!     whole-torus box;
//!   - *NURBS*: the whole-surface control hull contains any trim, unioned
//!     with the wire bound.
//!
//! Predicate errors include the candidate point (conservative). Any unknown
//! edge bound, invalid carrier, or non-finite critical point makes the whole
//! face non-prunable rather than risking an unsound box.
//!
//! # Trim authority
//!
//! Edges without a stored trim (missing trim authority) fall back to
//! the whole-domain bound where the carrier is bounded (full circle/ellipse
//! extent, whole NURBS hull); unbounded carriers (hyperbola, parabola) with
//! no trim are unknown. An invalid stored trim is always unknown.

use remus_geometry::bounds::curve::{
    circle_arc_bounds, ellipse_arc_bounds, hyperbola_arc_bounds, line_segment_bounds,
    nurbs_curve_bounds, parabola_arc_bounds,
};
use remus_geometry::bounds::surface::{
    cone_apex, nurbs_surface_bounds, sphere_axis_poles, sphere_bounds, torus_bounds,
    torus_direction_criticals,
};
use remus_geometry::bounds::{BoundConfidence, FiniteBound};
use remus_math::aabb::Aabb3;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};

use crate::CheckError;

/// A face bound for distance pruning.
#[derive(Debug, Clone)]
pub struct FaceBound {
    /// The face this bound was built for.
    pub face: FaceId,
    /// The bounding box. Valid for pruning only when [`FaceBound::prunable`]
    /// holds; otherwise a non-empty fallback.
    pub aabb: Aabb3,
    /// Whether the box is certified conservative and may prune candidates.
    pub prunable: bool,
    /// Machine-readable reason when non-prunable (`None` when prunable).
    pub unprunable_reason: Option<&'static str>,
}

impl FaceBound {
    /// Build the bound for a face.
    ///
    /// # Errors
    ///
    /// Returns an error if any topology entity referenced by the face is
    /// missing. Missing entities fail identically in exhaustive and
    /// branch-and-bound traversal because bounds are always built first.
    pub fn build(topo: &Topology, face_id: FaceId) -> Result<Self, CheckError> {
        face_bound(topo, face_id)
    }
}

/// Build the bound for a face.
///
/// See the module docs for the soundness argument.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the face is missing.
pub fn face_bound(topo: &Topology, face_id: FaceId) -> Result<FaceBound, CheckError> {
    let face = topo.face(face_id)?;
    let mut wire_ids = vec![face.outer_wire()];
    wire_ids.extend(face.inner_wires().iter().copied());

    let mut edge_boxes: Vec<Aabb3> = Vec::new();
    for wid in wire_ids {
        let wire = topo.wire(wid)?;
        for oe in wire.edges() {
            let bound = edge_span_bound(topo, oe.edge())?;
            match bound.confidence() {
                BoundConfidence::Conservative => edge_boxes.push(bound.aabb()),
                BoundConfidence::Unknown { reason } => {
                    return Ok(FaceBound {
                        face: face_id,
                        aabb: unknown_fallback_box(topo, face_id, &edge_boxes),
                        prunable: false,
                        unprunable_reason: Some(reason),
                    });
                }
            }
        }
    }

    let surface = topo.face(face_id)?.surface().clone();
    let mut union = union_all(&edge_boxes);
    // Trim predicate shared by every carrier-critical test below. Built
    // once per face (planes and cylinders need no interior points, so they
    // skip it); the closure only borrows it.
    let needs_predicate = matches!(
        surface,
        FaceSurface::Cone(_) | FaceSurface::Sphere(_) | FaceSurface::Torus(_)
    );
    let predicate = needs_predicate.then(|| TrimPredicate::build(topo, face_id));
    let accepts = |point: &Point3| predicate.as_ref().is_none_or(|p| p.accepts(point));

    // Carrier interior. `None` means "no finite interior bound available":
    // unbounded carriers with no wire extent are unknown.
    let interior_unknown: Option<&'static str> = match &surface {
        FaceSurface::Plane { .. } | FaceSurface::Cylinder(_) => None,
        FaceSurface::Cone(cone) => {
            let apex = cone_apex(cone);
            if point_is_finite(apex) && accepts(&apex) {
                union = Some(union_point(union, apex));
            } else if !point_is_finite(apex) {
                return Ok(unknown(topo, face_id, &edge_boxes, "non_finite_carrier"));
            }
            None
        }
        FaceSurface::Sphere(sphere) => {
            for pole in sphere_axis_poles(sphere) {
                if !point_is_finite(pole) {
                    return Ok(unknown(topo, face_id, &edge_boxes, "non_finite_carrier"));
                }
                if accepts(&pole) {
                    union = Some(union_point(union, pole));
                }
            }
            None
        }
        FaceSurface::Torus(torus) => {
            match torus_axis_alignment(torus.z_axis()) {
                Some(axis) => {
                    // Isolated equatorial criticals, conditioned on the trim;
                    // the axial rings are covered by the full-span expansion
                    // below (see `torus_ring_counterexample`).
                    for dir in equatorial_directions(axis) {
                        for sign in [1.0, -1.0] {
                            match torus_direction_criticals(torus, dir * sign) {
                                Some(points) => {
                                    for p in points {
                                        if !point_is_finite(p) {
                                            return Ok(unknown(
                                                topo,
                                                face_id,
                                                &edge_boxes,
                                                "non_finite_carrier",
                                            ));
                                        }
                                        if accepts(&p) {
                                            union = Some(union_point(union, p));
                                        }
                                    }
                                }
                                // Ring or degenerate along an equatorial
                                // direction cannot happen for a valid torus
                                // (only the axial direction rings), but a
                                // refusal must stay sound: keep the face on
                                // the exhaustive path.
                                None => {
                                    return Ok(unknown(
                                        topo,
                                        face_id,
                                        &edge_boxes,
                                        "unsupported_carrier",
                                    ));
                                }
                            }
                        }
                    }
                    union = expand_along_axis(union, torus.center(), torus.minor_radius(), axis);
                    None
                }
                // Tilted torus: the ring expansion has no single world axis
                // to attach to, so carry the whole-torus box (sound, loose,
                // still prunable).
                None => match torus_bounds(torus) {
                    bound if bound.is_prunable() => {
                        union = union_or(union, Some(bound.aabb()));
                        None
                    }
                    _ => Some("non_finite_carrier"),
                },
            }
        }
        FaceSurface::Nurbs(nurbs) => match nurbs_surface_bounds(nurbs, None, None) {
            bound if bound.is_prunable() => {
                union = union_or(union, Some(bound.aabb()));
                None
            }
            bound => Some(match bound.confidence() {
                BoundConfidence::Unknown { reason } => reason,
                BoundConfidence::Conservative => "unsupported_carrier",
            }),
        },
    };
    if let Some(reason) = interior_unknown {
        return Ok(unknown(topo, face_id, &edge_boxes, reason));
    }

    match union {
        Some(aabb) => Ok(FaceBound {
            face: face_id,
            aabb,
            prunable: true,
            unprunable_reason: None,
        }),
        // A face with no wire extent (degenerate wires): fall back to the
        // carrier-only bound where one exists.
        None => Ok(carrier_only_bound(face_id, &surface)),
    }
}

/// Build the bound for a single edge span.
///
/// Unions the geometry-layer span bound with the stored vertex positions
/// (the narrow phase walks vertex chords). Returns the geometry confidence,
/// except that non-finite vertices force `Unknown`.
///
/// # Errors
///
/// Returns an error if the edge or its vertices are missing.
pub fn edge_span_bound(topo: &Topology, edge_id: EdgeId) -> Result<FiniteBound, CheckError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    if !point_is_finite(start) || !point_is_finite(end) {
        return Ok(FiniteBound::infinite_unknown("non_finite_input"));
    }

    let span_bound = match edge.curve() {
        EdgeCurve::Line => line_segment_bounds(start, end),
        EdgeCurve::Circle(circle) => circle_span_bound(edge, circle, start, end, circle_arc_bounds),
        EdgeCurve::Ellipse(ellipse) => {
            circle_span_bound(edge, ellipse, start, end, ellipse_arc_bounds)
        }
        EdgeCurve::Hyperbola(branch) => match edge.strict_domain() {
            Ok((a, b)) => hyperbola_arc_bounds(branch, a, b),
            // Unbounded carrier with no finite trim: no whole-domain
            // fallback exists.
            Err(_) => {
                return Ok(FiniteBound::unknown(
                    vertex_box(start, end),
                    "unbounded_carrier",
                ));
            }
        },
        EdgeCurve::Parabola(parabola) => match edge.strict_domain() {
            Ok((a, b)) => parabola_arc_bounds(parabola, a, b),
            Err(_) => {
                return Ok(FiniteBound::unknown(
                    vertex_box(start, end),
                    "unbounded_carrier",
                ));
            }
        },
        EdgeCurve::NurbsCurve(nurbs) => {
            if edge.trim().is_none() {
                let (d0, d1) = nurbs.domain();
                nurbs_curve_bounds(nurbs, d0, d1)
            } else {
                match edge.strict_domain() {
                    Ok((a, b)) => nurbs_curve_bounds(nurbs, a, b),
                    Err(_) => {
                        return Ok(FiniteBound::unknown(vertex_box(start, end), "invalid_trim"));
                    }
                }
            }
        }
    };

    match span_bound.confidence() {
        BoundConfidence::Conservative => {
            // Union the stored vertices: exact stored data, and the narrow
            // phase walks vertex chords, so the convex box then contains
            // both the span and the chords.
            Ok(FiniteBound::conservative(
                span_bound.aabb().union(vertex_box(start, end)),
            ))
        }
        BoundConfidence::Unknown { .. } => Ok(span_bound),
    }
}

/// Span bound for a periodic conic edge (circle/ellipse).
///
/// `EdgeDomainError` is non-exhaustive, so trim absence (which
/// [`Edge::strict_domain`] reports as missing authority) is detected via
/// [`Edge::trim`] instead of matching variants: no stored trim means the
/// whole-domain fallback, a stored but rejected trim means unknown.
fn circle_span_bound<C>(
    edge: &remus_topology::edge::Edge,
    carrier: &C,
    start: Point3,
    end: Point3,
    bound: impl Fn(&C, f64, f64) -> FiniteBound,
) -> FiniteBound {
    if edge.trim().is_none() {
        return bound(carrier, 0.0, std::f64::consts::TAU);
    }
    match edge.strict_domain() {
        Ok((a, b)) => bound(carrier, a, b),
        Err(_) => FiniteBound::unknown(vertex_box(start, end), "invalid_trim"),
    }
}

/// The narrow-phase trim predicate, shared for predicate-consistency.
///
/// Built once per face and reused for every carrier critical point: the
/// polygon construction (wire sampling) dominates the per-point test.
/// Semantics match [`super::is_point_in_face_boundary`] exactly (same
/// calls, same order) — including outer-wire-only testing, so hole
/// handling agrees with the narrow phase by construction.
///
/// A build error (unreachable after the edge-bound pass validated the same
/// entities) degrades to accept-all, which is conservative: a failed test
/// must not exclude a candidate.
struct TrimPredicate {
    /// `(polygon, normal)` of the outer wire, or `None` for a full-surface
    /// face whose predicate always accepts.
    shape: Option<(Vec<Point3>, Vec3)>,
}

impl TrimPredicate {
    fn build(topo: &Topology, face_id: FaceId) -> Self {
        let shape = (|| -> Result<_, CheckError> {
            let polygon = crate::util::face_polygon(topo, face_id)?;
            if polygon.len() < 3 {
                return Ok(None);
            }
            let normal = crate::util::polygon_normal(&polygon);
            Ok(Some((polygon, normal)))
        })();
        Self {
            shape: shape.unwrap_or(None),
        }
    }

    fn accepts(&self, point: &Point3) -> bool {
        match &self.shape {
            None => true,
            Some((polygon, normal)) => crate::util::point_in_polygon_3d(point, polygon, normal),
        }
    }
}

/// Carrier-only bound for faces with no wire extent.
fn carrier_only_bound(face_id: FaceId, surface: &FaceSurface) -> FaceBound {
    let supported = |aabb: Aabb3| FaceBound {
        face: face_id,
        aabb,
        prunable: true,
        unprunable_reason: None,
    };
    let unsupported = |reason: &'static str| FaceBound {
        face: face_id,
        aabb: whole_space_box(),
        prunable: false,
        unprunable_reason: Some(reason),
    };
    match surface {
        FaceSurface::Plane { .. } | FaceSurface::Cylinder(_) | FaceSurface::Cone(_) => {
            unsupported("unbounded_carrier")
        }
        FaceSurface::Sphere(sphere) => match sphere_bounds(sphere) {
            bound if bound.is_prunable() => supported(bound.aabb()),
            _ => unsupported("non_finite_carrier"),
        },
        FaceSurface::Torus(torus) => match torus_bounds(torus) {
            bound if bound.is_prunable() => supported(bound.aabb()),
            _ => unsupported("non_finite_carrier"),
        },
        FaceSurface::Nurbs(nurbs) => match nurbs_surface_bounds(nurbs, None, None) {
            bound if bound.is_prunable() => supported(bound.aabb()),
            bound => unsupported(match bound.confidence() {
                BoundConfidence::Unknown { reason } => reason,
                BoundConfidence::Conservative => "unsupported_carrier",
            }),
        },
    }
}

/// Non-prunable face result with a fallback box covering the known edges.
fn unknown(
    topo: &Topology,
    face_id: FaceId,
    edge_boxes: &[Aabb3],
    reason: &'static str,
) -> FaceBound {
    FaceBound {
        face: face_id,
        aabb: unknown_fallback_box(topo, face_id, edge_boxes),
        prunable: false,
        unprunable_reason: Some(reason),
    }
}

/// Fallback box for unknown bounds: the union of established edge boxes and
/// every wire vertex (never empty — a face always references vertices — and
/// never used for pruning).
fn unknown_fallback_box(topo: &Topology, face_id: FaceId, edge_boxes: &[Aabb3]) -> Aabb3 {
    let mut union = union_all(edge_boxes);
    let Ok(face) = topo.face(face_id) else {
        return union.unwrap_or_else(whole_space_box);
    };
    let mut wires = vec![face.outer_wire()];
    wires.extend(face.inner_wires().iter().copied());
    for wid in wires {
        let Ok(wire) = topo.wire(wid) else { continue };
        for oe in wire.edges() {
            let Ok(edge) = topo.edge(oe.edge()) else {
                continue;
            };
            for vid in [edge.start(), edge.end()] {
                if let Ok(vertex) = topo.vertex(vid) {
                    let p = vertex.point();
                    if point_is_finite(p) {
                        union = Some(union_point(union, p));
                    }
                }
            }
        }
    }
    union.unwrap_or_else(whole_space_box)
}

/// Whole-space fallback box (never empty, never prunes).
fn whole_space_box() -> Aabb3 {
    Aabb3 {
        min: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        max: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
    }
}

/// Box of the two stored vertices.
fn vertex_box(start: Point3, end: Point3) -> Aabb3 {
    Aabb3::try_from_points([start, end]).unwrap_or_else(whole_space_box_for_missing_vertices)
}

/// Whole-space fallback for the (unreachable in practice) empty vertex pair.
fn whole_space_box_for_missing_vertices() -> Aabb3 {
    whole_space_box()
}

/// Union of boxes, or `None` when empty.
fn union_all(boxes: &[Aabb3]) -> Option<Aabb3> {
    boxes.iter().copied().reduce(Aabb3::union)
}

/// Union an optional box with another optional box.
fn union_or(first: Option<Aabb3>, second: Option<Aabb3>) -> Option<Aabb3> {
    match (first, second) {
        (Some(a), Some(b)) => Some(a.union(b)),
        (Some(a), None) => Some(a),
        (None, Some(b)) => Some(b),
        (None, None) => None,
    }
}

/// Union an optional box with a finite point.
fn union_point(current: Option<Aabb3>, point: Point3) -> Aabb3 {
    let point_box = Aabb3 {
        min: point,
        max: point,
    };
    current.map_or(point_box, |aabb| aabb.union(point_box))
}

/// Whether a point is finite.
fn point_is_finite(p: Point3) -> bool {
    p.x().is_finite() && p.y().is_finite() && p.z().is_finite()
}

/// Which world axis the torus axis aligns with, if any.
///
/// Returns `0/1/2` when `|axis·ê| > 1 − 1e−9`; otherwise `None` (tilted).
fn torus_axis_alignment(axis: Vec3) -> Option<usize> {
    const ALIGN: f64 = 1.0 - 1e-9;
    if axis.x().abs() > ALIGN {
        Some(0)
    } else if axis.y().abs() > ALIGN {
        Some(1)
    } else if axis.z().abs() > ALIGN {
        Some(2)
    } else {
        None
    }
}

/// Unit directions of the two world axes other than `axis`.
fn equatorial_directions(axis: usize) -> [Vec3; 2] {
    match axis {
        0 => [Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)],
        1 => [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)],
        _ => [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)],
    }
}

/// Expand the box along world `axis` to cover `center ± radius`.
///
/// Carries the torus axial rings, whose exact `u` position no single test
/// point can certify (see `torus_ring_counterexample`).
fn expand_along_axis(
    union: Option<Aabb3>,
    center: Point3,
    radius: f64,
    axis: usize,
) -> Option<Aabb3> {
    let anchor = union?;
    if !radius.is_finite() || radius < 0.0 {
        return Some(anchor);
    }
    let (c, lo, hi) = match axis {
        0 => (center.x(), anchor.min.x(), anchor.max.x()),
        1 => (center.y(), anchor.min.y(), anchor.max.y()),
        _ => (center.z(), anchor.min.z(), anchor.max.z()),
    };
    if !c.is_finite() {
        return Some(anchor);
    }
    let (mut min, mut max) = (anchor.min, anchor.max);
    if lo > c - radius {
        min = match axis {
            0 => Point3::new(c - radius, min.y(), min.z()),
            1 => Point3::new(min.x(), c - radius, min.z()),
            _ => Point3::new(min.x(), min.y(), c - radius),
        };
    }
    if hi < c + radius {
        max = match axis {
            0 => Point3::new(c + radius, max.y(), max.z()),
            1 => Point3::new(max.x(), c + radius, max.y()),
            _ => Point3::new(max.x(), max.y(), c + radius),
        };
    }
    Some(Aabb3 { min, max })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_topology::edge::Edge;
    use remus_topology::solid::Solid;
    use remus_topology::test_utils::make_unit_cube_manifold_at;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    const TOL: f64 = 1e-7;

    #[test]
    fn box_face_bounds_are_tight_and_prunable() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        assert_eq!(faces.len(), 6);
        for fid in faces {
            let bound = face_bound(&topo, fid).unwrap();
            assert!(bound.prunable, "box faces must be prunable");
            // Every wire vertex must be inside the bound.
            let face = topo.face(fid).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                for vid in [edge.start(), edge.end()] {
                    let p = topo.vertex(vid).unwrap().point();
                    assert!(
                        bound.aabb.contains_point(p),
                        "bound must contain vertex {p:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn line_edge_without_authority_is_still_conservative() {
        // Lines are endpoint-local on [0, 1] and need no stored trim.
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
        let b = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), TOL));
        let e = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let bound = edge_span_bound(&topo, e).unwrap();
        assert!(bound.is_prunable());
        assert!(bound.aabb().contains_point(Point3::new(0.5, 1.0, 1.5)));
    }

    #[test]
    fn unbounded_edge_without_trim_is_unknown_but_nonempty() {
        use remus_math::curves::Parabola3D;
        // A parabola edge with no stored trim is unbounded: unknown, but the
        // fallback still covers the vertices and never prunes.
        let parabola = Parabola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.25,
        )
        .unwrap();
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(Point3::new(-1.0, 1.0, 0.0), TOL));
        let b = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), TOL));
        let e = topo.add_edge(Edge::new(a, b, EdgeCurve::Parabola(parabola)));
        let bound = edge_span_bound(&topo, e).unwrap();
        assert!(!bound.is_prunable());
        assert!(bound.aabb().contains_point(Point3::new(-1.0, 1.0, 0.0)));
        assert!(bound.aabb().contains_point(Point3::new(1.0, 1.0, 0.0)));
    }

    #[test]
    fn parabola_edge_with_trim_is_conservative() {
        use remus_math::curves::Parabola3D;
        let parabola = Parabola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.25,
        )
        .unwrap();
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(Point3::new(-1.0, 1.0, 0.0), TOL));
        let b = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), TOL));
        let mut edge = Edge::new(a, b, EdgeCurve::Parabola(parabola));
        edge.set_trim(Some((-1.0, 1.0)));
        let e = topo.add_edge(edge);
        let bound = edge_span_bound(&topo, e).unwrap();
        assert!(bound.is_prunable());
        // Vertex (0,0,0) of the parabola must be inside.
        assert!(bound.aabb().contains_point(Point3::new(0.0, 0.0, 0.0)));
    }

    /// Proof boundary for the torus axial expansion: a band covering the top
    /// ring only along a spoke-free arc still reaches `cz + r`, which no
    /// finite spoke test can certify — so the bound always spans the axis.
    #[test]
    fn torus_ring_counterexample_stays_sound() {
        use remus_math::surfaces::ToroidalSurface;
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
        // A face whose wires sit just below the top ring: the wire bound
        // alone would top out below cz + r = 1.
        let mut topo = Topology::new();
        let ring_pt = |u: f64| torus.evaluate(u, std::f64::consts::FRAC_PI_2 - 0.3);
        let v0 = topo.add_vertex(Vertex::new(ring_pt(0.1), TOL));
        let v1 = topo.add_vertex(Vertex::new(ring_pt(0.2), TOL));
        let v2 = topo.add_vertex(Vertex::new(ring_pt(0.2) + Vec3::new(0.0, 0.0, -0.5), TOL));
        let v3 = topo.add_vertex(Vertex::new(ring_pt(0.1) + Vec3::new(0.0, 0.0, -0.5), TOL));
        let edges = [
            topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line)),
        ];
        let wire = topo.add_wire(
            Wire::new(
                edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let fid = topo.add_face(remus_topology::face::Face::new(
            wire,
            vec![],
            FaceSurface::Torus(torus.clone()),
        ));
        let bound = face_bound(&topo, fid).unwrap();
        assert!(bound.prunable);
        // The top ring value cz + r must be covered even though no wire
        // reaches it and no spoke test certifies it.
        assert!(
            bound.aabb.max.z() >= 1.0,
            "torus bound must span the axial ring, got max.z={}",
            bound.aabb.max.z()
        );
    }

    #[test]
    fn hollow_solid_face_bounds_cover_both_shells() {
        // Two nested cube shells in one solid: every face (outer + cavity)
        // gets a prunable bound.
        let mut topo = Topology::new();
        let outer = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let inner = make_unit_cube_manifold_at(&mut topo, 3.0, 3.0, 3.0);
        let outer_shell = topo.solid(outer).unwrap().outer_shell();
        let inner_shell = topo.solid(inner).unwrap().outer_shell();
        let hollow = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));
        let faces = remus_topology::explorer::solid_faces(&topo, hollow).unwrap();
        assert_eq!(faces.len(), 12);
        for fid in faces {
            let bound = face_bound(&topo, fid).unwrap();
            assert!(bound.prunable, "cavity faces must stay prunable");
        }
    }
}
