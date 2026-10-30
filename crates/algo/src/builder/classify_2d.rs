//! 2D interior point sampling and polygon classification.
//!
//! Given a closed wire loop in (u,v) parameter space, find a point
//! guaranteed to be inside the loop for solid classification.

use remus_math::vec::{Point2, Vec2};

/// Sample a point guaranteed to be inside a closed 2D polygon.
///
/// Strategy: try the centroid first (works for convex polygons). If the
/// centroid is outside (non-convex loop), walk inward from the midpoint
/// of a boundary edge along the inward normal.
///
/// `loop_pts` must be the vertices of a closed polygon (>=3 points).
pub fn sample_interior_point(loop_pts: &[Point2]) -> Point2 {
    // Graceful fallback for degenerate inputs instead of panicking.
    if loop_pts.is_empty() {
        return Point2::new(0.0, 0.0);
    }
    if loop_pts.len() < 3 {
        let n = loop_pts.len() as f64;
        let cx = loop_pts.iter().map(|p| p.x()).sum::<f64>() / n;
        let cy = loop_pts.iter().map(|p| p.y()).sum::<f64>() / n;
        return Point2::new(cx, cy);
    }

    let n = loop_pts.len() as f64;
    let cx = loop_pts.iter().map(|p| p.x()).sum::<f64>() / n;
    let cy = loop_pts.iter().map(|p| p.y()).sum::<f64>() / n;
    let centroid = Point2::new(cx, cy);

    // If centroid is inside, use it. A centroid that lands on the
    // boundary itself (e.g. the reflex corner of an L-shaped loop) passes
    // the even-odd test unpredictably and is not a safe interior sample.
    if point_in_polygon_2d(centroid, loop_pts)
        && distance_to_polygon_boundary(centroid, loop_pts) > boundary_eps(loop_pts)
    {
        return centroid;
    }

    // Edge-midpoint fallback for non-convex loops. For a bounded set of the
    // longest boundary edges, cast a ray inward from its midpoint and take the
    // midpoint of the resulting interior chord (the segment from the edge to
    // the first opposite boundary crossing). Keep the candidate with the
    // LONGEST such chord — the deepest, most robustly-interior point.
    //
    // Returning the *deepest* candidate rather than the *first* valid one is
    // what makes this rotation-invariant: the wire builder emits each loop with
    // a per-process nondeterministic starting edge (the rim boundary wire's
    // rotation tracks hash order), so a first-match scan picks a different edge
    // — and on a non-convex slice (a notched annular rim) a different pocket —
    // each run, flipping the sub-face's IN/OUT classification and producing an
    // intermittent mesh fallback. The longest-chord midpoint depends only on
    // the loop's geometry, not its starting index, and lands far from every
    // boundary so the downstream point-in-solid test is stable.
    let area = signed_area_2d(loop_pts);
    let n = loop_pts.len();
    let eps = boundary_eps(loop_pts);
    let mut best: Option<(Point2, f64)> = None;
    for i in candidate_edge_indices(loop_pts) {
        let j = (i + 1) % n;
        let a = loop_pts[i];
        let b = loop_pts[j];
        let mid = Point2::new((a.x() + b.x()) * 0.5, (a.y() + b.y()) * 0.5);
        let edge_dir = Vec2::new(b.x() - a.x(), b.y() - a.y());
        let len = edge_dir.length();
        if len < 1e-15 {
            continue;
        }
        // Inward normal depends on winding (left normal of the edge for a
        // CCW loop): CCW (positive area) -> (-dy, dx); CW -> (dy, -dx).
        let inward = if area > 0.0 {
            Vec2::new(-edge_dir.y() / len, edge_dir.x() / len)
        } else {
            Vec2::new(edge_dir.y() / len, -edge_dir.x() / len)
        };
        // First crossing of the inward ray with any other boundary edge
        // (every edge except the current one `i`).
        let mut t_hit = f64::INFINITY;
        for k in 0..n {
            if k == i {
                continue;
            }
            let c = loop_pts[k];
            let d = loop_pts[(k + 1) % n];
            if let Some(t) = ray_segment_param(mid, inward, c, d)
                && t > 1e-9
                && t < t_hit
            {
                t_hit = t;
            }
        }
        if !t_hit.is_finite() {
            continue;
        }
        let cand = Point2::new(
            mid.x() + inward.x() * t_hit * 0.5,
            mid.y() + inward.y() * t_hit * 0.5,
        );
        // Keep the longest interior chord; break exact ties by the candidate's
        // lexicographic coordinates so the result is loop-rotation-invariant
        // (the chosen start edge is HashMap-order-dependent upstream).
        let better = best.is_none_or(|(bp, bt)| {
            t_hit > bt + 1e-12
                || ((t_hit - bt).abs() <= 1e-12 && (cand.x(), cand.y()) < (bp.x(), bp.y()))
        });
        // The chord midpoint must also clear the boundary. When the inward ray
        // runs exactly ALONG a boundary edge instead of crossing it, it slips
        // past that edge's endpoint and reports the far side of the loop, so a
        // grazing ray wins the longest-chord contest with a chord that is not
        // interior at all and a midpoint that sits on the boundary. That
        // happens whenever an edge midpoint lands on a reflex feature, which
        // symmetric geometry produces exactly (a notch at half the face width),
        // not approximately -- so no epsilon nudge upstream avoids it.
        if better
            && point_in_polygon_2d(cand, loop_pts)
            && distance_to_polygon_boundary(cand, loop_pts) > eps
        {
            best = Some((cand, t_hit));
        }
    }
    if let Some((pt, _)) = best {
        return pt;
    }
    // All primary edge midpoints failed. Retry without a global-diagonal
    // clearance threshold so thin, high-aspect regions still receive a point
    // whose containment is actually verified.
    for i in 0..n {
        let a = loop_pts[i];
        let b = loop_pts[(i + 1) % n];
        let mid = Point2::new((a.x() + b.x()) * 0.5, (a.y() + b.y()) * 0.5);
        let candidate = Point2::new(
            0.75_f64.mul_add(mid.x(), centroid.x() * 0.25),
            0.75_f64.mul_add(mid.y(), centroid.y() * 0.25),
        );
        if point_in_polygon_2d(candidate, loop_pts) {
            return candidate;
        }
    }
    centroid
}

/// Select a bounded, geometry-ordered set of edges for chord sampling.
///
/// Each sampled chord requires a scan of the complete boundary. Bounding the
/// number of chords keeps that fallback linear after the `O(n log n)` edge
/// ordering, while ordering by geometry preserves the loop-rotation invariance
/// required by callers whose wire start edge is nondeterministic.
fn candidate_edge_indices(loop_pts: &[Point2]) -> Vec<usize> {
    const MAX_CHORD_CANDIDATES: usize = 64;

    let mut indices: Vec<usize> = (0..loop_pts.len()).collect();
    // ties: edges with equal length and midpoint tie, and the cut to 64 candidates keeps some of them.
    #[allow(clippy::disallowed_methods)]
    indices.sort_unstable_by(|&lhs, &rhs| {
        let edge_key = |i: usize| {
            let a = loop_pts[i];
            let b = loop_pts[(i + 1) % loop_pts.len()];
            let dx = b.x() - a.x();
            let dy = b.y() - a.y();
            (
                dx * dx + dy * dy,
                (a.x() + b.x()) * 0.5,
                (a.y() + b.y()) * 0.5,
            )
        };
        let left = edge_key(lhs);
        let right = edge_key(rhs);
        right
            .0
            .total_cmp(&left.0)
            .then_with(|| left.1.total_cmp(&right.1))
            .then_with(|| left.2.total_cmp(&right.2))
    });
    indices.truncate(MAX_CHORD_CANDIDATES);
    indices
}

/// Parameter `t >= 0` at which the ray `origin + t*dir` first crosses segment
/// `[s0, s1]`, or `None` if it does not. `dir` is assumed unit-length; `t` is a
/// world-space distance along the ray.
fn ray_segment_param(origin: Point2, dir: Vec2, s0: Point2, s1: Point2) -> Option<f64> {
    let e = Vec2::new(s1.x() - s0.x(), s1.y() - s0.y());
    let denom = dir.x() * e.y() - dir.y() * e.x();
    if denom.abs() < 1e-15 {
        return None; // parallel
    }
    let diff = Vec2::new(s0.x() - origin.x(), s0.y() - origin.y());
    // t along the ray, u along the segment.
    let t = (diff.x() * e.y() - diff.y() * e.x()) / denom;
    let u = (diff.x() * dir.y() - diff.y() * dir.x()) / denom;
    if t >= 0.0 && (0.0..=1.0).contains(&u) {
        Some(t)
    } else {
        None
    }
}

/// Scale-aware boundary tolerance for a 2D loop: a small fraction of the
/// loop's bounding-box diagonal, floored to avoid collapsing to zero for
/// degenerate inputs.
pub fn boundary_eps(loop_pts: &[Point2]) -> f64 {
    let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
    let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in loop_pts {
        min_x = min_x.min(p.x());
        min_y = min_y.min(p.y());
        max_x = max_x.max(p.x());
        max_y = max_y.max(p.y());
    }
    let diag = ((max_x - min_x).powi(2) + (max_y - min_y).powi(2)).sqrt();
    (diag * 1e-6).max(1e-12)
}

/// Minimum distance from a 2D point to the polygon's boundary segments.
pub fn distance_to_polygon_boundary(p: Point2, loop_pts: &[Point2]) -> f64 {
    let mut best = f64::INFINITY;
    let n = loop_pts.len();
    for i in 0..n {
        let a = loop_pts[i];
        let b = loop_pts[(i + 1) % n];
        let ab = Vec2::new(b.x() - a.x(), b.y() - a.y());
        let ap = Vec2::new(p.x() - a.x(), p.y() - a.y());
        let len_sq = ab.x() * ab.x() + ab.y() * ab.y();
        let t = if len_sq < 1e-30 {
            0.0
        } else {
            ((ap.x() * ab.x() + ap.y() * ab.y()) / len_sq).clamp(0.0, 1.0)
        };
        let dx = p.x() - (a.x() + ab.x() * t);
        let dy = p.y() - (a.y() + ab.y() * t);
        best = best.min((dx * dx + dy * dy).sqrt());
    }
    best
}

/// Test whether a 2D point is inside a closed polygon using ray-casting
/// (even-odd rule).
pub fn point_in_polygon_2d(p: Point2, polygon: &[Point2]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = polygon[i];
        let pj = polygon[j];
        // Check if a horizontal ray from p to +inf crosses edge (pi, pj).
        if (pi.y() > p.y()) != (pj.y() > p.y()) {
            let x_intersect = (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x();
            if p.x() < x_intersect {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// One edge of a planar loop, projected into the loop's plane frame, for
/// [`point_in_planar_loop`].
#[derive(Debug, Clone, Copy)]
pub enum PlanarLoopEdge {
    /// A straight edge, in traversal order.
    Segment(Point2, Point2),
    /// A circular arc on `center` and `radius`, from `start` through its
    /// parametric midpoint `mid` to `end`, in traversal order.
    Arc {
        /// Projected circle centre.
        center: Point2,
        /// Circle radius.
        radius: f64,
        /// Traversal start.
        start: Point2,
        /// Arc midpoint.
        mid: Point2,
        /// Traversal end.
        end: Point2,
    },
    /// One complete turn of a circle; `sense` is `+1.0` counter-clockwise.
    Circle {
        /// Projected circle centre.
        center: Point2,
        /// Circle radius.
        radius: f64,
        /// Traversal sense in the frame.
        sense: f64,
    },
}

impl PlanarLoopEdge {
    /// Project one wire edge into `frame`. `at(t)` evaluates the edge at the
    /// normalized traversal parameter `t` in `[0, 1]`; `closed` says the edge
    /// is one full turn. `None` for a curve other than a line or a circle
    /// lying in the frame's plane.
    pub fn project(
        curve: &remus_topology::edge::EdgeCurve,
        at: impl Fn(f64) -> remus_math::vec::Point3,
        closed: bool,
        frame: &super::plane_frame::PlaneFrame,
        angular: f64,
    ) -> Option<Self> {
        use remus_topology::edge::EdgeCurve;
        let point = |t: f64| frame.project(at(t));
        match curve {
            EdgeCurve::Line => Some(Self::Segment(point(0.0), point(1.0))),
            EdgeCurve::Circle(circle) => {
                let normal = frame.u_axis().cross(frame.v_axis());
                if circle.normal().cross(normal).length() > angular {
                    return None;
                }
                let center = frame.project(circle.center());
                let radius = circle.radius();
                if closed {
                    let (a, b, c) = (point(0.0), point(1.0 / 3.0), point(2.0 / 3.0));
                    Some(Self::Circle {
                        center,
                        radius,
                        sense: cross_2d(b - a, c - a).signum(),
                    })
                } else {
                    Some(Self::Arc {
                        center,
                        radius,
                        start: point(0.0),
                        mid: point(0.5),
                        end: point(1.0),
                    })
                }
            }
            EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::NurbsCurve(_) => None,
        }
    }
}

fn cross_2d(a: Vec2, b: Vec2) -> f64 {
    a.x() * b.y() - a.y() * b.x()
}

/// Whether `p` lies inside a closed planar loop of lines and circular arcs,
/// decided on the arcs themselves rather than their chords.
///
/// A sampled polygon of a circular boundary is inscribed, so a point in the
/// band between chord and arc reads as outside: 1 - cos(pi / n) of the radius
/// for n samples, which a thin annulus can lie inside entirely. Here the
/// winding number is summed per edge — a segment adds its subtended angle,
/// and an arc adds its chord's angle plus a full turn when `p` lies in the
/// circular segment the arc and chord enclose.
///
/// `None` when `p` lies within `eps` of an arc, of a vertex, or of an arc's
/// chord inside its circle: those cases stay with the caller's polygon test.
#[must_use]
pub fn point_in_planar_loop(p: Point2, edges: &[PlanarLoopEdge], eps: f64) -> Option<bool> {
    use std::f64::consts::TAU;
    let subtended = |a: Point2, b: Point2| -> Option<f64> {
        let (da, db) = (a - p, b - p);
        if da.length() <= eps || db.length() <= eps {
            return None;
        }
        Some(cross_2d(da, db).atan2(da.x() * db.x() + da.y() * db.y()))
    };
    let inside_circle = |center: Point2, radius: f64| -> Option<bool> {
        let offset = (p - center).length();
        ((offset - radius).abs() > eps).then_some(offset < radius)
    };
    if edges.is_empty() {
        return None;
    }
    let mut winding = 0.0;
    for &edge in edges {
        match edge {
            PlanarLoopEdge::Segment(a, b) => winding += subtended(a, b)?,
            PlanarLoopEdge::Circle {
                center,
                radius,
                sense,
            } => {
                if inside_circle(center, radius)? {
                    winding += TAU * sense;
                }
            }
            PlanarLoopEdge::Arc {
                center,
                radius,
                start,
                mid,
                end,
            } => {
                let inside = inside_circle(center, radius)?;
                winding += subtended(start, end)?;
                let chord = end - start;
                let side = cross_2d(chord, p - start);
                if inside && side.abs() <= eps * chord.length() {
                    return None;
                }
                if inside && side * cross_2d(chord, mid - start) > 0.0 {
                    // The arc and its chord close a loop around `p` in the
                    // arc's sense.
                    winding += TAU * cross_2d(mid - start, end - start).signum();
                }
            }
        }
    }
    Some((winding / TAU).round() != 0.0)
}

/// Compute the signed area of a 2D polygon (Shoelace formula).
///
/// Positive area = counterclockwise winding, negative = clockwise.
pub fn signed_area_2d(pts: &[Point2]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    let mut j = n - 1;
    for i in 0..n {
        area += (pts[j].x() - pts[i].x()) * (pts[j].y() + pts[i].y());
        j = i;
    }
    area * 0.5
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    #[test]
    fn interior_of_rectangle_is_inside() {
        let pts = vec![
            Point2::new(0.0, 0.0),
            Point2::new(10.0, 0.0),
            Point2::new(10.0, 10.0),
            Point2::new(0.0, 10.0),
        ];
        let interior = sample_interior_point(&pts);
        assert!(interior.x() > 0.5 && interior.x() < 9.5);
        assert!(interior.y() > 0.5 && interior.y() < 9.5);
        assert!(point_in_polygon_2d(interior, &pts));
    }

    #[test]
    fn point_in_polygon_simple_square() {
        let sq = vec![
            Point2::new(0.0, 0.0),
            Point2::new(10.0, 0.0),
            Point2::new(10.0, 10.0),
            Point2::new(0.0, 10.0),
        ];
        assert!(point_in_polygon_2d(Point2::new(5.0, 5.0), &sq));
        assert!(!point_in_polygon_2d(Point2::new(15.0, 5.0), &sq));
        assert!(!point_in_polygon_2d(Point2::new(-1.0, 5.0), &sq));
    }

    #[test]
    fn signed_area_ccw_is_positive() {
        let ccw = vec![
            Point2::new(0.0, 0.0),
            Point2::new(10.0, 0.0),
            Point2::new(10.0, 10.0),
            Point2::new(0.0, 10.0),
        ];
        let area = signed_area_2d(&ccw);
        assert!((area - 100.0).abs() < 1e-10, "area = {area}");
    }

    #[test]
    fn signed_area_cw_is_negative() {
        let cw = vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 10.0),
            Point2::new(10.0, 10.0),
            Point2::new(10.0, 0.0),
        ];
        let area = signed_area_2d(&cw);
        assert!((area + 100.0).abs() < 1e-10, "area = {area}");
    }

    #[test]
    fn interior_of_l_shaped_polygon() {
        let l_shape = vec![
            Point2::new(0.0, 0.0),
            Point2::new(10.0, 0.0),
            Point2::new(10.0, 5.0),
            Point2::new(5.0, 5.0),
            Point2::new(5.0, 10.0),
            Point2::new(0.0, 10.0),
        ];
        let interior = sample_interior_point(&l_shape);
        assert!(
            point_in_polygon_2d(interior, &l_shape),
            "interior ({}, {}) should be inside L-shape",
            interior.x(),
            interior.y()
        );
    }

    /// A notch whose wall sits a few ulps off half the face width: the midpoint
    /// of the opposite edge then falls just short of that wall, so the inward
    /// ray slips past the notch corner instead of crossing it and reports the
    /// far side of the loop. That spuriously long chord used to win the
    /// longest-chord contest, putting the sample 2.7e-15 from the boundary
    /// where the even-odd test is a coin flip.
    ///
    /// Verbatim from the kumiko corner cut (a strut wall straddling the base's
    /// bottom plane), where it kept a cutter face lying outside the base and
    /// left the result non-manifold. The literals matter: rounding them, or
    /// deriving the notch as `w * 0.5`, makes the ray hit the corner exactly
    /// and the case no longer reproduces.
    #[test]
    fn interior_of_notched_polygon_clears_the_boundary() {
        let w = 1.016_138_123_637_824_2_f64;
        let notch_x = 0.508_069_061_818_914_8_f64;
        assert!(notch_x > w * 0.5, "the notch must sit just past half-width");
        let c_shape = vec![
            Point2::new(0.0, 0.0),
            Point2::new(w, 0.0),
            Point2::new(w, 0.499_999_999_999_999_6),
            Point2::new(notch_x, 0.499_999_999_999_999_67),
            Point2::new(notch_x, 3.700_000_000_000_000_6),
            Point2::new(w, 3.700_000_000_000_000_6),
            Point2::new(w, 4.2),
            Point2::new(0.0, 4.2),
        ];
        let interior = sample_interior_point(&c_shape);
        assert!(
            point_in_polygon_2d(interior, &c_shape),
            "interior ({}, {}) should be inside the notched loop",
            interior.x(),
            interior.y()
        );
        let clearance = distance_to_polygon_boundary(interior, &c_shape);
        assert!(
            clearance > boundary_eps(&c_shape),
            "interior ({}, {}) sits on the boundary (clearance {clearance:e})",
            interior.x(),
            interior.y()
        );
    }

    #[test]
    fn chord_candidates_are_bounded_and_rotation_invariant() {
        let polygon: Vec<_> = (0..1_000)
            .map(|i| Point2::new(i as f64, (i % 7) as f64))
            .collect();
        let candidates = candidate_edge_indices(&polygon);
        assert_eq!(candidates.len(), 64);

        let mut rotated = polygon[317..].to_vec();
        rotated.extend_from_slice(&polygon[..317]);
        let candidate_midpoints = |points: &[Point2], indices: Vec<usize>| {
            let mut midpoints: Vec<_> = indices
                .into_iter()
                .map(|i| {
                    let a = points[i];
                    let b = points[(i + 1) % points.len()];
                    ((a.x() + b.x()) * 0.5, (a.y() + b.y()) * 0.5)
                })
                .collect();
            midpoints
                .sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
            midpoints
        };
        assert_eq!(
            candidate_midpoints(&polygon, candidates),
            candidate_midpoints(&rotated, candidate_edge_indices(&rotated))
        );
    }

    fn polar(radius: f64, degrees: f64) -> Point2 {
        let angle = degrees.to_radians();
        Point2::new(radius * angle.cos(), radius * angle.sin())
    }

    #[test]
    fn circle_loop_decides_the_chord_sagitta_band() {
        // A thin annulus left by a 0.05 resize of an r 12.3 bore: its interior
        // lies between the hole circle's inscribed octagon and the circle.
        let hole = [PlanarLoopEdge::Circle {
            center: Point2::new(0.0, 0.0),
            radius: 12.3,
            sense: -1.0,
        }];
        let octagon: Vec<_> = (0..8).map(|k| polar(12.3, 45.0 * f64::from(k))).collect();
        let sample = polar(12.275, 22.5);
        assert!(!point_in_polygon_2d(sample, &octagon));
        assert_eq!(point_in_planar_loop(sample, &hole, 1e-6), Some(true));
        assert_eq!(
            point_in_planar_loop(polar(12.325, 22.5), &hole, 1e-6),
            Some(false)
        );
        assert_eq!(point_in_planar_loop(polar(12.3, 10.0), &hole, 1e-6), None);
    }

    #[test]
    fn arc_loop_counts_its_circular_segment_in_either_sense() {
        // A D: the upper half of an r 1 circle closed by its diameter.
        let arc = PlanarLoopEdge::Arc {
            center: Point2::new(0.0, 0.0),
            radius: 1.0,
            start: polar(1.0, 0.0),
            mid: polar(1.0, 90.0),
            end: polar(1.0, 180.0),
        };
        let diameter = PlanarLoopEdge::Segment(polar(1.0, 180.0), polar(1.0, 0.0));
        let forward = [arc, diameter];
        let reversed = [
            PlanarLoopEdge::Segment(polar(1.0, 0.0), polar(1.0, 180.0)),
            PlanarLoopEdge::Arc {
                center: Point2::new(0.0, 0.0),
                radius: 1.0,
                start: polar(1.0, 180.0),
                mid: polar(1.0, 90.0),
                end: polar(1.0, 0.0),
            },
        ];
        for edges in [&forward[..], &reversed[..]] {
            // Beside the arc, outside the chords of any coarse polygon.
            assert_eq!(
                point_in_planar_loop(polar(0.99, 60.0), edges, 1e-9),
                Some(true)
            );
            assert_eq!(
                point_in_planar_loop(polar(1.01, 60.0), edges, 1e-9),
                Some(false)
            );
            assert_eq!(
                point_in_planar_loop(polar(0.5, -30.0), edges, 1e-9),
                Some(false)
            );
        }
        // A major arc: three quarters of the circle closed by its chord.
        let major = [
            PlanarLoopEdge::Arc {
                center: Point2::new(0.0, 0.0),
                radius: 1.0,
                start: polar(1.0, 0.0),
                mid: polar(1.0, 135.0),
                end: polar(1.0, 270.0),
            },
            PlanarLoopEdge::Segment(polar(1.0, 270.0), polar(1.0, 0.0)),
        ];
        assert_eq!(
            point_in_planar_loop(Point2::new(0.0, 0.0), &major, 1e-9),
            Some(true)
        );
        assert_eq!(
            point_in_planar_loop(polar(0.99, 200.0), &major, 1e-9),
            Some(true)
        );
        assert_eq!(
            point_in_planar_loop(polar(0.9, 315.0), &major, 1e-9),
            Some(false)
        );
    }
}
