//! 2D polygon operations: clipping, filleting, chamfering, and segment detection.

use crate::vec::{Point2, Vec2};

/// Cross product of vectors (b-a) and (c-a).
#[must_use]
pub fn cross_2d(a: Point2, b: Point2, c: Point2) -> f64 {
    (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x())
}

fn line_intersect_2d(a1: Point2, a2: Point2, b1: Point2, b2: Point2) -> Option<Point2> {
    let dx_a = a2.x() - a1.x();
    let dy_a = a2.y() - a1.y();
    let dx_b = b2.x() - b1.x();
    let dy_b = b2.y() - b1.y();
    let denom = dx_a * dy_b - dy_a * dx_b;
    if denom.abs() < 1e-15 {
        return None;
    }
    let t = ((b1.x() - a1.x()) * dy_b - (b1.y() - a1.y()) * dx_b) / denom;
    Some(Point2::new(a1.x() + t * dx_a, a1.y() + t * dy_a))
}

fn point_to_line_dist_sq_2d(p: Point2, a: Point2, b: Point2) -> f64 {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-30 {
        let ex = p.x() - a.x();
        let ey = p.y() - a.y();
        return ex * ex + ey * ey;
    }
    let cross = (p.x() - a.x()) * dy - (p.y() - a.y()) * dx;
    (cross * cross) / len_sq
}

/// Sutherland-Hodgman polygon clipping algorithm.
///
/// `clip` must be convex. Either polygon may wind either way; the result
/// follows `subject`'s vertex order.
#[must_use]
pub fn sutherland_hodgman_clip(subject: &[Point2], clip: &[Point2]) -> Vec<Point2> {
    // A clip edge's inside is its left in a counter-clockwise clip polygon
    // and its right in a clockwise one.
    let side = if signed_area_2x(clip) < 0.0 {
        -1.0
    } else {
        1.0
    };
    let mut output: Vec<Point2> = subject.to_vec();

    for i in 0..clip.len() {
        if output.is_empty() {
            return output;
        }
        let edge_start = clip[i];
        let edge_end = clip[(i + 1) % clip.len()];
        let input = output;
        output = Vec::new();

        for j in 0..input.len() {
            let current = input[j];
            let previous = input[(j + input.len() - 1) % input.len()];

            let curr_inside = side * cross_2d(edge_start, edge_end, current) >= 0.0;
            let prev_inside = side * cross_2d(edge_start, edge_end, previous) >= 0.0;

            if curr_inside {
                if !prev_inside
                    && let Some(p) = line_intersect_2d(previous, current, edge_start, edge_end)
                {
                    output.push(p);
                }
                output.push(current);
            } else if prev_inside
                && let Some(p) = line_intersect_2d(previous, current, edge_start, edge_end)
            {
                output.push(p);
            }
        }
    }

    output
}

/// Find common (collinear, overlapping) edges between two polygons.
#[must_use]
pub fn find_common_segments(a: &[Point2], b: &[Point2], tolerance: f64) -> Vec<(Point2, Point2)> {
    let mut results = Vec::new();
    let tol_sq = tolerance * tolerance;

    for i in 0..a.len() {
        let a1 = a[i];
        let a2 = a[(i + 1) % a.len()];
        for j in 0..b.len() {
            let b1 = b[j];
            let b2 = b[(j + 1) % b.len()];

            // Check if edge A and edge B are collinear and overlapping.
            // Both endpoints of B must be close to line through A, or vice versa.
            let dist_b1 = point_to_line_dist_sq_2d(b1, a1, a2);
            let dist_b2 = point_to_line_dist_sq_2d(b2, a1, a2);

            if dist_b1 < tol_sq && dist_b2 < tol_sq {
                // Edges are collinear. Check for overlap by projecting onto A's direction.
                let dx = a2.x() - a1.x();
                let dy = a2.y() - a1.y();
                let len_sq = dx * dx + dy * dy;
                if len_sq < tol_sq {
                    continue;
                }
                let t1 = ((b1.x() - a1.x()) * dx + (b1.y() - a1.y()) * dy) / len_sq;
                let t2 = ((b2.x() - a1.x()) * dx + (b2.y() - a1.y()) * dy) / len_sq;
                let t_min = t1.min(t2).max(0.0);
                let t_max = t1.max(t2).min(1.0);
                if t_max - t_min > tolerance / len_sq.sqrt() {
                    results.push((
                        Point2::new(a1.x() + t_min * dx, a1.y() + t_min * dy),
                        Point2::new(a1.x() + t_max * dx, a1.y() + t_max * dy),
                    ));
                }
            }
        }
    }
    results
}

/// Round all corners of a 2D polygon with arc approximations.
///
/// Each corner becomes a circular arc of `radius`, tangent to both of its
/// edges and sampled with 8 segments. The tangent points sit
/// `radius / tan(α/2)` from the corner, `α` the angle between its edges, so
/// the arc never leaves the corner. Where the tangent points of two corners
/// would overlap on their shared edge, the edge is split between them in
/// proportion to what each needs and their radii shrink to fit. Convex and
/// reflex corners and either winding are handled alike.
#[must_use]
pub fn fillet_polygon_2d(polygon: &[Point2], radius: f64) -> Vec<Point2> {
    const ARC_SEGMENTS: u32 = 8;
    let n = polygon.len();
    if n < 3 {
        return polygon.to_vec();
    }

    let edge_len = |i: usize| (polygon[(i + 1) % n] - polygon[i]).length();
    // `tan(α/2) = |a × b| / (|a||b| + a · b)` for the edges `a`, `b` leaving a
    // corner: 0 at a spike, unbounded where the edges run straight on.
    let half_tan = |i: usize| {
        let curr = polygon[i];
        let (a, b) = (polygon[(i + n - 1) % n] - curr, polygon[(i + 1) % n] - curr);
        let cross = (a.x() * b.y() - a.y() * b.x()).abs();
        cross / (a.length() * b.length() + a.x() * b.x() + a.y() * b.y())
    };
    // The tangent length each corner asks for; no corner can use more than
    // its shorter edge.
    let wanted: Vec<f64> = (0..n)
        .map(|i| {
            let tangent = radius / half_tan(i);
            let room = edge_len((i + n - 1) % n).min(edge_len(i));
            if tangent.is_nan() {
                0.0
            } else {
                tangent.min(room)
            }
        })
        .collect();
    let share = |edge: usize, mine: f64, theirs: f64| {
        let len = edge_len(edge);
        if mine + theirs > len {
            len * mine / (mine + theirs)
        } else {
            mine
        }
    };
    let setback: Vec<f64> = (0..n)
        .map(|i| {
            let prev = (i + n - 1) % n;
            wanted[i]
                .min(share(prev, wanted[i], wanted[prev]))
                .min(share(i, wanted[i], wanted[(i + 1) % n]))
        })
        .collect();

    let mut result = Vec::with_capacity(n * (ARC_SEGMENTS as usize + 1));
    for i in 0..n {
        let prev = (i + n - 1) % n;
        let curr = polygon[i];
        let t = setback[i];
        if t < 1e-10 {
            result.push(curr);
            continue;
        }

        let to_prev = (polygon[prev] - curr) * (1.0 / edge_len(prev));
        let to_next = (polygon[(i + 1) % n] - curr) * (1.0 / edge_len(i));
        let t1 = curr + to_prev * t;
        let t2 = curr + to_next * t;
        // The center lies on the bisector, `t / cos(α/2)` from the corner.
        let dot = to_prev.x() * to_next.x() + to_prev.y() * to_next.y();
        let center = curr + (to_prev + to_next) * (t / (1.0 + dot));

        // An edge used up by both of its corners would repeat its shared
        // tangent point; the previous corner has already placed it.
        if t + setback[prev] < edge_len(prev) * (1.0 - 1e-12) || setback[prev] < 1e-10 {
            result.push(t1);
        }
        let (start, end) = (t1 - center, t2 - center);
        let sweep = (start.x() * end.y() - start.y() * end.x()).atan2(start.dot(end));
        for k in 1..ARC_SEGMENTS {
            let (sin, cos) = (sweep * f64::from(k) / f64::from(ARC_SEGMENTS)).sin_cos();
            result.push(
                center
                    + Vec2::new(
                        start.x() * cos - start.y() * sin,
                        start.x() * sin + start.y() * cos,
                    ),
            );
        }
        result.push(t2);
    }

    result
}

/// Twice the signed area of a closed polygon; positive when counter-clockwise.
fn signed_area_2x(polygon: &[Point2]) -> f64 {
    // Relative to the first vertex, so the sum does not cancel far from the
    // origin.
    let Some(&o) = polygon.first() else {
        return 0.0;
    };
    polygon
        .windows(2)
        .map(|w| {
            let (p, q) = (w[0] - o, w[1] - o);
            p.x() * q.y() - q.x() * p.y()
        })
        .sum()
}

/// Cut all corners of a 2D polygon with flat bevels.
#[must_use]
pub fn chamfer_polygon_2d(polygon: &[Point2], distance: f64) -> Vec<Point2> {
    let n = polygon.len();
    if n < 3 {
        return polygon.to_vec();
    }

    let mut result = Vec::with_capacity(n * 2);

    for i in 0..n {
        let prev = polygon[(i + n - 1) % n];
        let curr = polygon[i];
        let next = polygon[(i + 1) % n];

        let d_prev = ((prev.x() - curr.x()).powi(2) + (prev.y() - curr.y()).powi(2)).sqrt();
        let d_next = ((next.x() - curr.x()).powi(2) + (next.y() - curr.y()).powi(2)).sqrt();

        let d = (d_prev.min(d_next) / 2.0).min(distance);

        if d < 1e-10 {
            result.push(curr);
            continue;
        }

        // Two chamfer points: one on previous edge, one on next edge
        result.push(Point2::new(
            curr.x() + (prev.x() - curr.x()) / d_prev * d,
            curr.y() + (prev.y() - curr.y()) / d_prev * d,
        ));
        result.push(Point2::new(
            curr.x() + (next.x() - curr.x()) / d_next * d,
            curr.y() + (next.y() - curr.y()) / d_next * d,
        ));
    }

    result
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use std::f64::consts::PI;

    fn pts(coords: &[(f64, f64)]) -> Vec<Point2> {
        coords.iter().map(|&(x, y)| Point2::new(x, y)).collect()
    }

    fn reversed(poly: &[Point2]) -> Vec<Point2> {
        poly.iter().rev().copied().collect()
    }

    fn area(poly: &[Point2]) -> f64 {
        signed_area_2x(poly) / 2.0
    }

    /// Area a sampled fillet of radius `r` with tangent length `t` and arc
    /// sweep `sweep` cuts from a convex corner (or adds at a reflex one): the
    /// kite between the corner and the arc center, less the 8-triangle fan.
    fn corner_piece(r: f64, t: f64, sweep: f64) -> f64 {
        r * t - 4.0 * r * r * (sweep / 8.0).sin()
    }

    fn assert_no_repeated_points(poly: &[Point2]) {
        for i in 0..poly.len() {
            let gap = (poly[(i + 1) % poly.len()] - poly[i]).length();
            assert!(gap > 1e-9, "points {i} and {} coincide", i + 1);
        }
    }

    fn octagon(circumradius: f64) -> Vec<Point2> {
        (0..8)
            .map(|k| {
                let t = f64::from(k) * PI / 4.0;
                Point2::new(circumradius * t.cos(), circumradius * t.sin())
            })
            .collect()
    }

    /// MATH-7a: the setback was `r` rather than `r / tan(α/2)`, and the arc a
    /// parabolic bulge, so a regular octagon of circumradius 10 filleted at 1
    /// reached |p| = 10.2346, outside its own outline.
    #[test]
    fn octagon_fillet_stays_inside_with_closed_form_area() {
        let t = 2.0_f64.sqrt() - 1.0; // tan(22.5°) = 1 / tan(67.5°)
        for poly in [octagon(10.0), reversed(&octagon(10.0))] {
            for r in [1.0, 2.0, 5.0] {
                let rounded = fillet_polygon_2d(&poly, r);
                assert_eq!(rounded.len(), 72);
                assert_no_repeated_points(&rounded);
                let reach = rounded
                    .iter()
                    .map(|p| p.x().hypot(p.y()))
                    .fold(0.0, f64::max);
                assert!(reach < 10.0, "fillet {r} reaches {reach}");
                let expected = 200.0 * 2.0_f64.sqrt() - 8.0 * corner_piece(r, r * t, PI / 4.0);
                assert!(
                    (area(&rounded).abs() - expected).abs() < 1e-9,
                    "fillet {r}: area {}, expected {expected}",
                    area(&rounded).abs()
                );
                assert_eq!(area(&rounded) > 0.0, area(&poly) > 0.0);
            }
        }
    }

    /// A radius too large for the edges shrinks to fit. Every octagon edge is
    /// shared by two equal corners, so each takes half of it and the fillet
    /// becomes the inscribed circle of radius 10·cos(π/8), sampled 64 times.
    #[test]
    fn oversized_octagon_fillet_becomes_the_inscribed_circle() {
        let apothem = 10.0 * (PI / 8.0).cos();
        let rounded = fillet_polygon_2d(&octagon(10.0), 10.0);
        assert_eq!(rounded.len(), 64, "shared tangent points appear once");
        assert_no_repeated_points(&rounded);
        for p in &rounded {
            assert!((p.x().hypot(p.y()) - apothem).abs() < 1e-9, "{p:?}");
        }
        let expected = 32.0 * apothem * apothem * (PI / 32.0).sin();
        assert!((area(&rounded) - expected).abs() < 1e-9);
    }

    /// Setbacks of 1.5 at both ends of a 2-long edge overlap, so the corners on
    /// it shrink to radius 1 and the 10×2 rectangle becomes a stadium.
    #[test]
    fn overlapping_setbacks_clamp_the_radius() {
        let rect = pts(&[(0.0, 0.0), (10.0, 0.0), (10.0, 2.0), (0.0, 2.0)]);
        for poly in [rect.clone(), reversed(&rect)] {
            let rounded = fillet_polygon_2d(&poly, 1.5);
            assert_eq!(rounded.len(), 34);
            assert_no_repeated_points(&rounded);
            for p in &rounded {
                assert!((0.0..=10.0).contains(&p.x()) && (0.0..=2.0).contains(&p.y()));
            }
            let expected = 20.0 - 4.0 * corner_piece(1.0, 1.0, PI / 2.0);
            assert!((area(&rounded).abs() - expected).abs() < 1e-9);
        }
    }

    /// A reflex corner gains the piece a convex corner loses: the L of area 7
    /// with five convex corners and one reflex corner, all right angles,
    /// filleted at 0.25 has area 7 − 4·piece.
    #[test]
    fn reflex_corner_fillet_adds_its_piece() {
        let l_shape = pts(&[
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 1.0),
            (1.0, 1.0),
            (1.0, 4.0),
            (0.0, 4.0),
        ]);
        let expected = 7.0 - 4.0 * corner_piece(0.25, 0.25, PI / 2.0);
        for poly in [l_shape.clone(), reversed(&l_shape)] {
            let rounded = fillet_polygon_2d(&poly, 0.25);
            assert_eq!(rounded.len(), 54);
            assert!((area(&rounded).abs() - expected).abs() < 1e-12);
        }
    }

    /// MATH-7b: the inside test assumed a counter-clockwise clip polygon, so a
    /// clockwise one clipped everything away. The overlap of [0, 2]² and
    /// [1, 3]² is [1, 2]² in every winding combination, in the subject's order.
    #[test]
    fn clip_accepts_either_winding() {
        let a = pts(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]);
        let b = pts(&[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]);
        for subject in [a.clone(), reversed(&a)] {
            for clip in [b.clone(), reversed(&b)] {
                let overlap = sutherland_hodgman_clip(&subject, &clip);
                assert!(
                    (area(&overlap).abs() - 1.0).abs() < 1e-12,
                    "overlap {overlap:?}"
                );
                assert_eq!(area(&overlap) > 0.0, area(&subject) > 0.0);
                for p in &overlap {
                    assert!((1.0..=2.0).contains(&p.x()) && (1.0..=2.0).contains(&p.y()));
                }
            }
        }
    }

    /// The clip polygon's winding was read from a shoelace sum of absolute
    /// coordinates, which cancels far from the origin: a unit-square clip
    /// near (1e10, 1e10) read clockwise for about half of the offsets tried
    /// and clipped everything away.
    #[test]
    fn clip_winding_holds_far_from_the_origin() {
        let square = |c: f64, h: f64| {
            pts(&[
                (c - h, c - h),
                (c + h, c - h),
                (c + h, c + h),
                (c - h, c + h),
            ])
        };
        for k in 0..200 {
            let c = 1e10 + 0.0137 * f64::from(k);
            for clip in [square(c, 0.5), reversed(&square(c, 0.5))] {
                let overlap = sutherland_hodgman_clip(&square(c, 2.0), &clip);
                assert_eq!(overlap.len(), 4, "c = {c}: {overlap:?}");
                for p in &overlap {
                    assert!((p.x() - c).abs() <= 0.5 + 1e-5 && (p.y() - c).abs() <= 0.5 + 1e-5);
                }
            }
        }
    }
}
