//! Rays that pass too close to a face boundary to trust their crossing count.
//!
//! A ray through an edge or a vertex meets every face sharing it right at
//! that face's trim, and each face's trim test decides alone whether the hit
//! is inside it: one crossing can count twice or not at all. A tetrahedron
//! whose vertices sit on two of the three base rays reads Outside from an
//! interior point, both rays counting 0. The vote therefore treats a ray that
//! passes within a small margin of the boundary skeleton as an abstention and
//! draws a recovery direction instead
//! ([`ClassifyOptions::max_recovery_attempts`](super::ClassifyOptions)).
//!
//! The skeleton is every boundary edge of the classified faces: a line edge
//! as its exact segment, a curved edge as a chord polyline over its
//! parameter domain, joined to its topological vertices. A hit within the
//! margin of a straight edge is always caught; near a curved edge, only
//! within the margin of its chords (approximate by the chord sag, which the
//! trim polygons share). Each vertex adds its tolerance ball, where faces
//! meeting there may disagree.
//!
//! Building it reads one certified box per edge
//! ([`remus_geometry::bounds::curve`]) and samples nothing: a curved edge's
//! chords are sampled the first time a ray passes through its box, since the
//! one-shot classifier rebuilds the skeleton on every query.

use std::sync::OnceLock;

use remus_geometry::bounds::FiniteBound;
use remus_geometry::bounds::curve::{
    circle_arc_bounds, ellipse_arc_bounds, hyperbola_arc_bounds, nurbs_curve_bounds,
    parabola_arc_bounds,
};
use remus_math::aabb::Aabb3;
use remus_math::bvh::Bvh;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceId;

use crate::CheckError;

/// Chords per curved edge.
const CURVE_SEGMENTS: usize = 32;

/// Graze margin per unit of the skeleton's bounding-box diagonal: far above
/// the rounding of a computed hit point, far below any feature size.
const RELATIVE_MARGIN: f64 = 1e-9;

/// Graze margin per unit of coordinate magnitude: a few hundred ULPs, so a
/// small solid placed far from the world origin (one ULP near 1e9 is about
/// 1.2e-7) still catches rays its own vertices were rounded onto.
const ROUNDOFF_MARGIN: f64 = 256.0 * f64::EPSILON;

/// Up to this many edges a ray tests every edge box in turn; above it, a BVH
/// over the boxes is built once and queried per ray.
const LINEAR_SCAN_MAX: usize = 16;

type Segment = (Point3, Point3);

/// A vertex and its tolerance.
type Ball = (Point3, f64);

/// Boundary edges of a face set, as padded boxes over their segments.
#[derive(Debug, Clone)]
pub struct BoundarySkeleton {
    edges: Vec<SkeletonEdge>,
    /// Over the padded edge boxes, when there are more than
    /// [`LINEAR_SCAN_MAX`] edges.
    bvh: Option<Bvh>,
    margin: f64,
}

#[derive(Debug, Clone)]
struct SkeletonEdge {
    /// Contains every segment of the edge and both end balls, padded by the
    /// margin.
    padded: Aabb3,
    /// Each end vertex and its tolerance.
    ends: [Ball; 2],
    shape: EdgeShape,
}

#[derive(Debug, Clone)]
enum EdgeShape {
    Line(Segment),
    Curve {
        id: EdgeId,
        /// Sampled on first use (or at build when the curve has no
        /// certified box).
        chords: OnceLock<Vec<Segment>>,
        /// A circle carrier, for a closed-form test that rules a ray out
        /// before any chord is read.
        circle: Option<CircleReach>,
    },
}

/// A circle `(center, unit normal, radius)` and how far beyond it the
/// edge's segments reach: the chord sag, and the joins from the carrier's
/// ends to the edge's vertices.
type CircleReach = ((Point3, Vec3, f64), f64);

impl BoundarySkeleton {
    /// Collect every edge bounding `faces` (outer and hole loops).
    ///
    /// # Errors
    ///
    /// Returns an error if a face, wire, edge or vertex handle is invalid, or
    /// a curved edge has no valid parameter domain.
    pub fn build(topo: &Topology, faces: &[FaceId]) -> Result<Self, CheckError> {
        let mut ids: Vec<EdgeId> = Vec::new();
        for &fid in faces {
            let face = topo.face(fid)?;
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                ids.extend(
                    topo.wire(wid)?
                        .edges()
                        .iter()
                        .map(remus_topology::wire::OrientedEdge::edge),
                );
            }
        }
        // hot: 11,264+ calls, ~1,239,000 n*log2(n) work, n up to 120 in one criterion --test + native perf pass.
        #[allow(clippy::disallowed_methods)]
        ids.sort_unstable();
        ids.dedup();

        // Unpadded boxes first: the margin scales with their union.
        let mut boxed: Vec<(Aabb3, [Ball; 2], EdgeShape)> = Vec::with_capacity(ids.len());
        for id in ids {
            let edge = topo.edge(id)?;
            let curve = edge.curve();
            let ends = [
                end_ball(topo.vertex(edge.start())?),
                end_ball(topo.vertex(edge.end())?),
            ];
            let (a, b) = (ends[0].0, ends[1].0);
            if matches!(curve, EdgeCurve::Line) {
                if is_finite(a) && is_finite(b) {
                    boxed.push((segment_box(a, b), ends, EdgeShape::Line((a, b))));
                }
                continue;
            }
            let (t0, t1) = edge
                .strict_domain()
                .map_err(crate::error::edge_domain_validation)?;
            let bound = curve_bounds(curve, t0, t1);
            let circle = if let EdgeCurve::Circle(c) = curve {
                #[allow(clippy::cast_precision_loss)]
                let half_step = 0.5 * (t1 - t0).abs() / CURVE_SEGMENTS as f64;
                let sag = c.radius() * (1.0 - half_step.cos());
                let join = (a - c.evaluate(t0))
                    .length()
                    .max((b - c.evaluate(t1)).length());
                Some(((c.center(), c.normal(), c.radius()), sag + join))
            } else {
                None
            };
            if let Some(aabb) = bound.filter(FiniteBound::is_prunable).map(|b| b.aabb()) {
                // The joins to the vertices reach outside the carrier's box.
                let aabb = if is_finite(a) && is_finite(b) {
                    aabb.union(segment_box(a, b))
                } else {
                    aabb
                };
                boxed.push((
                    aabb,
                    ends,
                    EdgeShape::Curve {
                        id,
                        chords: OnceLock::new(),
                        circle,
                    },
                ));
            } else {
                // No certified box: sample now and box the chords, which is
                // all the graze test reads.
                let chords = sample_chords(topo, id)?;
                if let Some(aabb) = chords
                    .iter()
                    .map(|&(a, b)| segment_box(a, b))
                    .reduce(Aabb3::union)
                {
                    boxed.push((
                        aabb,
                        ends,
                        EdgeShape::Curve {
                            id,
                            chords: OnceLock::from(chords),
                            circle,
                        },
                    ));
                }
            }
        }
        // The end balls widen each box.
        for (aabb, ends, _) in &mut boxed {
            for &(point, radius) in ends.iter() {
                if is_finite(point) {
                    *aabb = aabb.union(segment_box(point, point).expanded(radius));
                }
            }
        }

        let (extent, magnitude) = boxed
            .iter()
            .map(|(aabb, _, _)| *aabb)
            .reduce(Aabb3::union)
            .map_or((0.0, 0.0), |aabb| {
                (
                    (aabb.max - aabb.min).length(),
                    max_abs(aabb.min).max(max_abs(aabb.max)),
                )
            });
        let margin = (RELATIVE_MARGIN * extent.max(1.0)).max(ROUNDOFF_MARGIN * magnitude);
        let edges: Vec<SkeletonEdge> = boxed
            .into_iter()
            .map(|(aabb, ends, shape)| SkeletonEdge {
                padded: aabb.expanded(margin),
                ends,
                shape,
            })
            .collect();
        let bvh = (edges.len() > LINEAR_SCAN_MAX).then(|| {
            let boxes: Vec<(usize, Aabb3)> = edges
                .iter()
                .enumerate()
                .map(|(i, e)| (i, e.padded))
                .collect();
            Bvh::build(&boxes)
        });
        Ok(Self { edges, bvh, margin })
    }

    /// Whether the ray from `origin` along `direction` passes within the
    /// graze margin of any boundary segment.
    ///
    /// # Errors
    ///
    /// Returns an error if a curved edge's chords, sampled on first use from
    /// `topo` (the topology the skeleton was built from), cannot be read.
    pub fn grazed_by(
        &self,
        topo: &Topology,
        origin: Point3,
        direction: Vec3,
    ) -> Result<bool, CheckError> {
        // A ray cast from far beyond the skeleton rounds at its own scale.
        let margin = self.margin.max(ROUNDOFF_MARGIN * max_abs(origin));
        let margin_sq = margin * margin;
        let near = |edge: &SkeletonEdge| -> Result<bool, CheckError> {
            for &(point, radius) in &edge.ends {
                let reach = margin + radius;
                if ray_segment_distance_sq(origin, direction, point, point) <= reach * reach {
                    return Ok(true);
                }
            }
            if let EdgeShape::Curve {
                circle: Some((circle, beyond)),
                ..
            } = edge.shape
                && !circle_may_graze(circle, origin, direction, margin + beyond)
            {
                return Ok(false);
            }
            Ok(edge_segments(topo, edge)?
                .iter()
                .any(|&(a, b)| ray_segment_distance_sq(origin, direction, a, b) <= margin_sq))
        };
        // The boxes were padded with the build margin; a wider query margin
        // (a distant origin) scans them all, each widened to match.
        let widen = margin - self.margin;
        if let Some(bvh) = self.bvh.as_ref().filter(|_| widen <= 0.0) {
            for i in bvh.query_ray(origin, direction) {
                if near(&self.edges[i])? {
                    return Ok(true);
                }
            }
        } else {
            let inv = Vec3::new(
                1.0 / direction.x(),
                1.0 / direction.y(),
                1.0 / direction.z(),
            );
            let reciprocal_overflow =
                (0..3).any(|axis| direction.0[axis].abs() > 0.0 && inv.0[axis].is_infinite());
            for edge in &self.edges {
                let padded = if widen > 0.0 {
                    edge.padded.expanded(widen)
                } else {
                    edge.padded
                };
                let intersects = if reciprocal_overflow {
                    padded.ray_intersects_direction(origin, direction)
                } else {
                    padded.ray_intersects(origin, inv)
                };
                if intersects && near(edge)? {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    /// Heap bytes retained, for cache accounting, counting every curved edge
    /// as sampled.
    #[must_use]
    pub fn retained_bytes(&self) -> usize {
        use std::mem::size_of;
        let curved = self
            .edges
            .iter()
            .filter(|e| matches!(e.shape, EdgeShape::Curve { .. }))
            .count();
        let edge_bytes = self
            .edges
            .capacity()
            .saturating_mul(size_of::<SkeletonEdge>());
        let chord_bytes = curved
            .saturating_mul(CURVE_SEGMENTS + 2)
            .saturating_mul(size_of::<Segment>());
        // `Bvh` is a flat node vector whose length is private: bound it by
        // the construction cap of two nodes per primitive.
        let bvh_bytes = if self.bvh.is_some() {
            self.edges
                .len()
                .saturating_mul(2)
                .saturating_mul(size_of::<Aabb3>().saturating_add(3 * size_of::<usize>()))
        } else {
            0
        };
        edge_bytes
            .saturating_add(chord_bytes)
            .saturating_add(bvh_bytes)
    }
}

/// An edge's segments, sampling a curved edge's chords on first use.
fn edge_segments<'s>(topo: &Topology, edge: &'s SkeletonEdge) -> Result<&'s [Segment], CheckError> {
    match &edge.shape {
        EdgeShape::Line(segment) => Ok(std::slice::from_ref(segment)),
        EdgeShape::Curve { id, chords, .. } => {
            if let Some(chords) = chords.get() {
                return Ok(chords);
            }
            let sampled = sample_chords(topo, *id)?;
            Ok(chords.get_or_init(|| sampled))
        }
    }
}

/// A vertex and its tolerance, a non-finite tolerance read as none.
fn end_ball(vertex: &remus_topology::vertex::Vertex) -> Ball {
    let tolerance = vertex.tolerance();
    (
        vertex.point(),
        if tolerance.is_finite() {
            tolerance.max(0.0)
        } else {
            0.0
        },
    )
}

/// Certified box of a curved edge over its domain; `None` for a line.
fn curve_bounds(curve: &EdgeCurve, t0: f64, t1: f64) -> Option<FiniteBound> {
    match curve {
        EdgeCurve::Line => None,
        EdgeCurve::Circle(c) => Some(circle_arc_bounds(c, t0, t1)),
        EdgeCurve::Ellipse(e) => Some(ellipse_arc_bounds(e, t0, t1)),
        EdgeCurve::Parabola(p) => Some(parabola_arc_bounds(p, t0, t1)),
        EdgeCurve::Hyperbola(h) => Some(hyperbola_arc_bounds(h, t0, t1)),
        EdgeCurve::NurbsCurve(n) => Some(nurbs_curve_bounds(n, t0, t1)),
    }
}

/// [`CURVE_SEGMENTS`] chords over a curved edge's domain, joined at each end
/// to the edge's vertex (which the carrier need only meet within the vertex
/// tolerance), non-finite segments dropped. Chord ends lie on the curve, so
/// every chord stays inside the curve's certified box; the joins stay inside
/// its union with the vertices.
fn sample_chords(topo: &Topology, id: EdgeId) -> Result<Vec<Segment>, CheckError> {
    let edge = topo.edge(id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (t0, t1) = edge
        .strict_domain()
        .map_err(crate::error::edge_domain_validation)?;
    // Closed trims use the projected topological seam, as the classifier's
    // wire polygon does. A tolerated angular discrepancy changes every
    // chord, including those far from either endpoint's tolerance ball.
    let closed_domain = if edge.is_closed() {
        match edge.curve() {
            EdgeCurve::Circle(c) => Some((c.project(start), std::f64::consts::TAU, None)),
            EdgeCurve::Ellipse(e) => Some((e.project(start), std::f64::consts::TAU, None)),
            EdgeCurve::NurbsCurve(n) => {
                let (a, b) = n.domain();
                Some((
                    crate::util::nurbs_seam_parameter(n, start, a, b),
                    b - a,
                    Some(a),
                ))
            }
            EdgeCurve::Line | EdgeCurve::Parabola(_) | EdgeCurve::Hyperbola(_) => None,
        }
    } else {
        None
    };
    let mut chords = Vec::with_capacity(CURVE_SEGMENTS + 2);
    let mut previous = start;
    for i in 0..=CURVE_SEGMENTS {
        #[allow(clippy::cast_precision_loss)]
        let fraction = i as f64 / CURVE_SEGMENTS as f64;
        let t = closed_domain.map_or_else(
            || (t1 - t0).mul_add(fraction, t0),
            |(seam, span, wrap)| {
                wrap.map_or(seam + span * fraction, |a| {
                    a + (seam - a + span * fraction).rem_euclid(span)
                })
            },
        );
        let point = edge.curve().evaluate_with_endpoints(t, start, end);
        if is_finite(previous) && is_finite(point) {
            chords.push((previous, point));
        }
        previous = point;
    }
    if is_finite(previous) && is_finite(end) {
        chords.push((previous, end));
    }
    Ok(chords)
}

/// Whether the ray can pass within `margin` of the full circle
/// `(center, unit normal, radius)`; `false` only when provably not.
///
/// Within `margin` of the circle means within `margin` of its plane, which
/// the ray is only for `t` in one interval; across that interval the
/// in-plane radius moves at most `|direction|` per unit `t` (it is
/// 1-Lipschitz in the point), so a radius error at the interval's middle
/// beyond that slack plus the margin rules the ray out. Near-parallel rays
/// get a wide interval and fall through to the chords; any NaN does too.
fn circle_may_graze(
    (center, normal, radius): (Point3, Vec3, f64),
    origin: Point3,
    direction: Vec3,
    margin: f64,
) -> bool {
    let w = origin - center;
    let h0 = w.dot(normal);
    let dn = direction.dot(normal);
    if dn == 0.0 {
        return h0.abs() <= margin || h0.is_nan();
    }
    let a = (-margin - h0) / dn;
    let b = (margin - h0) / dn;
    let (lo, hi) = (a.min(b).max(0.0), a.max(b));
    if hi < lo {
        return false;
    }
    let mid = 0.5 * (lo + hi);
    let x = w + direction * mid;
    let rho = (x - normal * x.dot(normal)).length();
    let slack = direction.length() * 0.5 * (hi - lo);
    let excess = (rho - radius).abs() - slack - margin;
    excess.is_nan() || excess <= 0.0
}

fn max_abs(p: Point3) -> f64 {
    p.x().abs().max(p.y().abs()).max(p.z().abs())
}

fn is_finite(p: Point3) -> bool {
    p.0.iter().all(|c| c.is_finite())
}

fn segment_box(a: Point3, b: Point3) -> Aabb3 {
    Aabb3 {
        min: Point3::new(a.x().min(b.x()), a.y().min(b.y()), a.z().min(b.z())),
        max: Point3::new(a.x().max(b.x()), a.y().max(b.y()), a.z().max(b.z())),
    }
}

/// Squared distance between the half-line `origin + t·direction` (`t ≥ 0`)
/// and the segment `[a, b]`.
///
/// Projected onto the plane across the ray, the segment's closest point to
/// the ray's line is a planar point-to-segment problem with no division by a
/// near-parallel determinant, so a segment crossing a nearly parallel ray
/// still reads 0. That point decides when it lies ahead of the origin;
/// otherwise the half-line's own end (the origin) or a segment end does, and
/// distance to a half-line is convex along the segment, so these candidates
/// are complete.
fn ray_segment_distance_sq(origin: Point3, direction: Vec3, a: Point3, b: Point3) -> f64 {
    let point_to_segment = |p: Point3| -> f64 {
        let ab = b - a;
        let ee = ab.dot(ab);
        let s = if ee > 0.0 {
            ((p - a).dot(ab) / ee).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (p - (a + ab * s)).length_squared()
    };
    let dd = direction.dot(direction);
    let point_to_ray = |p: Point3| -> f64 {
        let t = if dd > 0.0 {
            ((p - origin).dot(direction) / dd).max(0.0)
        } else {
            0.0
        };
        (p - (origin + direction * t)).length_squared()
    };

    let mut best = point_to_segment(origin)
        .min(point_to_ray(a))
        .min(point_to_ray(b));
    if dd > 0.0 {
        let across = |p: Point3| {
            let w = p - origin;
            w - direction * (w.dot(direction) / dd)
        };
        let (pa, pb) = (across(a), across(b));
        let span = pb - pa;
        let span_sq = span.dot(span);
        let s = if span_sq > 0.0 {
            (-pa.dot(span) / span_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let closest = a + (b - a) * s;
        if (closest - origin).dot(direction) >= 0.0 {
            best = best.min((pa + span * s).length_squared());
        }
    }
    best
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn circle_prefilter_keeps_every_ray_near_the_circle() {
        // Unit circle in z = 0 about the origin.
        let circle = (Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0);
        let m = 1e-9;
        let o = Point3::new(0.2, -0.1, -3.0);
        // Through a point of the circle, and through points just inside and
        // outside it by half the margin.
        for r in [1.0, 1.0 - 0.5 * m, 1.0 + 0.5 * m] {
            let target = Point3::new(r * 0.6, r * 0.8, 0.0);
            assert!(circle_may_graze(circle, o, target - o, m), "r = {r}");
        }
        // Through the center and well outside: ruled out.
        assert!(!circle_may_graze(
            circle,
            o,
            Point3::new(0.0, 0.0, 0.0) - o,
            m
        ));
        assert!(!circle_may_graze(
            circle,
            o,
            Point3::new(1.5, 0.0, 0.0) - o,
            m
        ));
        // Pointing away from the plane: ruled out.
        assert!(!circle_may_graze(circle, o, Vec3::new(0.0, 0.0, -1.0), m));
        // In the plane, and nearly parallel to it: left to the chords.
        let in_plane = Point3::new(0.0, 0.0, 0.0);
        assert!(circle_may_graze(
            circle,
            in_plane,
            Vec3::new(1.0, 0.0, 0.0),
            m
        ));
        assert!(circle_may_graze(
            circle,
            Point3::new(-5.0, 0.0, 1e-12),
            Vec3::new(1.0, 0.0, -1e-15),
            m
        ));
        // Parallel and off the plane by more than the margin: ruled out.
        assert!(!circle_may_graze(
            circle,
            Point3::new(-5.0, 0.0, 1e-6),
            Vec3::new(1.0, 0.0, 0.0),
            m
        ));
    }

    #[test]
    fn ray_segment_distance_closed_forms() {
        let o = Point3::new(0.0, 0.0, 0.0);
        let x = Vec3::new(1.0, 0.0, 0.0);
        // Crossing segment one unit above the ray.
        let d = ray_segment_distance_sq(
            o,
            x,
            Point3::new(2.0, -1.0, 1.0),
            Point3::new(2.0, 1.0, 1.0),
        );
        assert!((d - 1.0).abs() < 1e-15, "{d}");
        // The same segment behind the ray's origin: nearest is the origin.
        let d = ray_segment_distance_sq(
            o,
            x,
            Point3::new(-2.0, -1.0, 1.0),
            Point3::new(-2.0, 1.0, 1.0),
        );
        assert!((d - 5.0).abs() < 1e-15, "{d}");
        // Parallel segment 0.5 away, ahead of the origin.
        let d =
            ray_segment_distance_sq(o, x, Point3::new(1.0, 0.5, 0.0), Point3::new(3.0, 0.5, 0.0));
        assert!((d - 0.25).abs() < 1e-15, "{d}");
        // Segment end touching the ray.
        let d =
            ray_segment_distance_sq(o, x, Point3::new(4.0, 0.0, 0.0), Point3::new(4.0, 3.0, 2.0));
        assert!(d.abs() < 1e-30, "{d}");
        // Nearly parallel and crossing: a determinant test reads it as
        // parallel and misses the crossing at x = 2.
        let d = ray_segment_distance_sq(
            o,
            x,
            Point3::new(1.0, -1e-8, 0.0),
            Point3::new(3.0, 1e-8, 0.0),
        );
        assert!(d < 1e-30, "{d}");
        // Skew: nearest points interior to both, distance 2.
        let d = ray_segment_distance_sq(
            o,
            Vec3::new(1.0, 1.0, 0.0),
            Point3::new(3.0, -1.0, 2.0),
            Point3::new(-1.0, 3.0, 2.0),
        );
        assert!((d - 4.0).abs() < 1e-12, "{d}");
    }

    /// A plane disc bounded by one closed unit circle edge in z = 0, its
    /// seam vertex at angle 0 moved `offset` outward off the carrier.
    fn disc(offset: f64) -> (Topology, FaceId) {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::Edge;
        use remus_topology::face::{Face, FaceSurface};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let seam = topo.add_vertex(Vertex::new(Point3::new(1.0 + offset, 0.0, 0.0), 1e-7));
        let mut rim = Edge::new(seam, seam, EdgeCurve::Circle(circle));
        rim.set_trim(Some((0.0, std::f64::consts::TAU)));
        let rim = topo.add_edge(rim);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        (topo, face)
    }

    /// A ray through a chord's midpoint, 4.8e-3 inside the circle (the sag
    /// of a 1/32-turn chord), grazes: the prefilter must reach the chords.
    #[test]
    fn rays_through_circle_chords_graze() {
        let (topo, face) = disc(0.0);
        let skeleton = BoundarySkeleton::build(&topo, &[face]).unwrap();
        let half_step = std::f64::consts::PI / 32.0;
        let mid = Point3::new(
            half_step.cos() * half_step.cos(),
            half_step.cos() * half_step.sin(),
            0.0,
        );
        let o = Point3::new(0.1, 0.2, 1.0);
        assert!(skeleton.grazed_by(&topo, o, mid - o).unwrap());
        // Halfway between the chord and the circle: clear of both.
        let clear = Point3::new(mid.x() * 1.0024, mid.y() * 1.0024, 0.0);
        assert!(!skeleton.grazed_by(&topo, o, clear - o).unwrap());
    }

    /// A seam vertex 5e-8 off its carrier (within its 1e-7 tolerance): a ray
    /// through the vertex grazes, though it misses the carrier by 3.5e-8.
    #[test]
    fn rays_through_a_vertex_off_its_carrier_graze() {
        let (topo, face) = disc(5e-8);
        let skeleton = BoundarySkeleton::build(&topo, &[face]).unwrap();
        let o = Point3::new(0.0, 0.0, 1.0);
        let vertex = Point3::new(1.0 + 5e-8, 0.0, 0.0);
        assert!(skeleton.grazed_by(&topo, o, vertex - o).unwrap());
        // Within the tolerance ball, though off the vertex and its join.
        let ball = Point3::new(1.0 + 5e-8, 8e-8, 0.0);
        assert!(skeleton.grazed_by(&topo, o, ball - o).unwrap());
    }
    /// The stored trim starts at zero, while the tolerated seam projects
    /// slightly later. Graze tests must use the classifier trim's chords.
    #[test]
    fn closed_curve_chords_follow_the_projected_seam() {
        use remus_math::curves::{Circle3D, Ellipse3D};
        let origin = Point3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let circle = Circle3D::new(origin, normal, 1.0).unwrap();
        let nurbs = remus_geometry::convert::curve_to_nurbs::circle_to_nurbs(
            &circle,
            0.0,
            std::f64::consts::TAU,
        )
        .unwrap();
        let nurbs_domain = nurbs.domain();
        for (curve, phase, tolerance, domain) in [
            (
                EdgeCurve::Circle(circle),
                9e-8,
                1e-7,
                (0.0, std::f64::consts::TAU),
            ),
            (
                EdgeCurve::Ellipse(Ellipse3D::new(origin, normal, 1.0, 0.75).unwrap()),
                9e-8,
                1e-7,
                (0.0, std::f64::consts::TAU),
            ),
            (EdgeCurve::NurbsCurve(nurbs), 0.1, 0.2, nurbs_domain),
        ] {
            let (mut topo, face) = disc(0.0);
            let wire = topo.face(face).unwrap().outer_wire();
            let edge = topo.wire(wire).unwrap().edges()[0].edge();
            let vertex = topo.edge(edge).unwrap().start();
            let seam = curve.evaluate_with_endpoints(phase, origin, origin);
            topo.vertex_mut(vertex).unwrap().set_point(seam);
            topo.vertex_mut(vertex)
                .unwrap()
                .set_tolerance(tolerance)
                .unwrap();
            topo.edge_mut(edge).unwrap().set_curve(curve);
            topo.edge_mut(edge).unwrap().set_trim(Some(domain));
            let polygon = crate::util::wire_polygon_sampled(&topo, wire, CURVE_SEGMENTS).unwrap();
            let skeleton = BoundarySkeleton::build(&topo, &[face]).unwrap();
            let a = polygon[16];
            let b = polygon[17];
            let target = a + (b - a) * 0.1;
            let direction = Vec3::new(0.0, 0.0, -1.0);
            assert!(
                skeleton
                    .grazed_by(&topo, target - direction, direction)
                    .unwrap()
            );
        }
    }
}
