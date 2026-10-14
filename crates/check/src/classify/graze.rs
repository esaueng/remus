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
//! parameter domain. A hit within the margin of a straight edge is always
//! caught; near a curved edge, only within the margin of its chords
//! (approximate by the chord sag, which the trim polygons share).
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

/// Up to this many edges a ray tests every edge box in turn; above it, a BVH
/// over the boxes is built once and queried per ray.
const LINEAR_SCAN_MAX: usize = 16;

type Segment = (Point3, Point3);

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
    id: EdgeId,
    /// Contains every segment of the edge, padded by the margin.
    padded: Aabb3,
    shape: EdgeShape,
}

#[derive(Debug, Clone)]
enum EdgeShape {
    Line(Segment),
    Curve {
        /// Sampled on first use (or at build when the curve has no
        /// certified box).
        chords: OnceLock<Vec<Segment>>,
        /// A circle carrier's `(center, unit normal, radius)`, for a
        /// closed-form test that rules a ray out before any chord is read.
        circle: Option<(Point3, Vec3, f64)>,
    },
}

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
        ids.sort_unstable();
        ids.dedup();

        // Unpadded boxes first: the margin scales with their union.
        let mut boxed: Vec<(EdgeId, Aabb3, EdgeShape)> = Vec::with_capacity(ids.len());
        for id in ids {
            let edge = topo.edge(id)?;
            let curve = edge.curve();
            if matches!(curve, EdgeCurve::Line) {
                let a = topo.vertex(edge.start())?.point();
                let b = topo.vertex(edge.end())?.point();
                if is_finite(a) && is_finite(b) {
                    boxed.push((id, segment_box(a, b), EdgeShape::Line((a, b))));
                }
                continue;
            }
            let (t0, t1) = edge
                .strict_domain()
                .map_err(crate::error::edge_domain_validation)?;
            let bound = curve_bounds(curve, t0, t1);
            let circle = if let EdgeCurve::Circle(c) = curve {
                Some((c.center(), c.normal(), c.radius()))
            } else {
                None
            };
            if let Some(aabb) = bound.filter(FiniteBound::is_prunable).map(|b| b.aabb()) {
                boxed.push((
                    id,
                    aabb,
                    EdgeShape::Curve {
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
                        id,
                        aabb,
                        EdgeShape::Curve {
                            chords: OnceLock::from(chords),
                            circle,
                        },
                    ));
                }
            }
        }

        let extent = boxed
            .iter()
            .map(|(_, aabb, _)| *aabb)
            .reduce(Aabb3::union)
            .map_or(0.0, |aabb| (aabb.max - aabb.min).length());
        let margin = RELATIVE_MARGIN * extent.max(1.0);
        let edges: Vec<SkeletonEdge> = boxed
            .into_iter()
            .map(|(id, aabb, shape)| SkeletonEdge {
                id,
                padded: aabb.expanded(margin),
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
        let margin_sq = self.margin * self.margin;
        let near = |edge: &SkeletonEdge| -> Result<bool, CheckError> {
            if let EdgeShape::Curve {
                circle: Some(circle),
                ..
            } = edge.shape
                && !circle_may_graze(circle, origin, direction, self.margin)
            {
                return Ok(false);
            }
            Ok(edge_segments(topo, edge)?
                .iter()
                .any(|&(a, b)| ray_segment_distance_sq(origin, direction, a, b) <= margin_sq))
        };
        if let Some(bvh) = &self.bvh {
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
            for edge in &self.edges {
                if edge.padded.ray_intersects(origin, inv) && near(edge)? {
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
            .saturating_mul(CURVE_SEGMENTS)
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
        EdgeShape::Curve { chords, .. } => {
            if let Some(chords) = chords.get() {
                return Ok(chords);
            }
            let sampled = sample_chords(topo, edge.id)?;
            Ok(chords.get_or_init(|| sampled))
        }
    }
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

/// [`CURVE_SEGMENTS`] chords over a curved edge's domain, non-finite chords
/// dropped. Chord ends lie on the curve, so every chord stays inside the
/// curve's certified box.
fn sample_chords(topo: &Topology, id: EdgeId) -> Result<Vec<Segment>, CheckError> {
    let edge = topo.edge(id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (t0, t1) = edge
        .strict_domain()
        .map_err(crate::error::edge_domain_validation)?;
    let mut chords = Vec::with_capacity(CURVE_SEGMENTS);
    let mut previous = edge.curve().evaluate_with_endpoints(t0, start, end);
    for i in 1..=CURVE_SEGMENTS {
        #[allow(clippy::cast_precision_loss)]
        let t = (t1 - t0).mul_add(i as f64 / CURVE_SEGMENTS as f64, t0);
        let point = edge.curve().evaluate_with_endpoints(t, start, end);
        if is_finite(previous) && is_finite(point) {
            chords.push((previous, point));
        }
        previous = point;
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
/// Closest points of the two supporting lines, each parameter clamped to its
/// range and the other re-solved against the clamp (the standard
/// segment–segment construction with the ray's upper bound removed).
fn ray_segment_distance_sq(origin: Point3, direction: Vec3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let w = origin - a;
    let dd = direction.dot(direction);
    let ee = ab.dot(ab);
    let de = direction.dot(ab);
    let dw = direction.dot(w);
    let ew = ab.dot(w);

    let point_to_segment = |p: Point3| -> f64 {
        let s = if ee > 0.0 {
            ((p - a).dot(ab) / ee).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = a + ab * s;
        (p - q).length_squared()
    };
    let point_to_ray = |p: Point3| -> f64 {
        let t = if dd > 0.0 {
            ((p - origin).dot(direction) / dd).max(0.0)
        } else {
            0.0
        };
        let q = origin + direction * t;
        (p - q).length_squared()
    };

    let denom = dd * ee - de * de;
    // Interior candidate of the two supporting lines, when they are not
    // parallel; the boundary candidates cover every clamped case.
    let mut best = point_to_segment(origin)
        .min(point_to_ray(a))
        .min(point_to_ray(b));
    if denom > f64::EPSILON * dd * ee {
        let t = (de * ew - ee * dw) / denom;
        let s = (dd * ew - de * dw) / denom;
        if t >= 0.0 && (0.0..=1.0).contains(&s) {
            let p = origin + direction * t;
            let q = a + ab * s;
            best = best.min((p - q).length_squared());
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
        // Skew: nearest points interior to both, distance 2.
        let d = ray_segment_distance_sq(
            o,
            Vec3::new(1.0, 1.0, 0.0),
            Point3::new(3.0, -1.0, 2.0),
            Point3::new(-1.0, 3.0, 2.0),
        );
        assert!((d - 4.0).abs() < 1e-12, "{d}");
    }
}
