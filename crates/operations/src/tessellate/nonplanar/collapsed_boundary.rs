//! Chart coordinates for boundary samples on a collapsed NURBS domain side.
//!
//! A NURBS patch whose boundary iso-curve collapses to a single point over a
//! parameter interval (coincident boundary control points: a pole, or a blend
//! rolling around a convex corner whose radius equals its own, where the
//! contact rail stops at the corner while the cross-section keeps turning)
//! maps that whole UV segment onto one 3D point. A face whose boundary passes
//! through that point must traverse the segment in UV, but projecting its
//! samples there is ambiguous: every point of the segment is equally close,
//! so Newton returns arbitrary, non-monotone positions along it. The chart
//! polygon then zig-zags back over itself, constraint recovery stalls and the
//! face falls off the shared-boundary CDT.
//!
//! The collapsed intervals are read exactly from the control net, the samples
//! charted onto them (or lying at their 3D point) are re-placed monotonically
//! between the well-conditioned neighbours on either side. Their 3D positions
//! and shared ids are unchanged, so only the chart polygon's shape changes.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;

/// One domain side interval whose boundary iso-curve is a single point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CollapsedSide {
    /// `true` when the side is `v = fixed` (the collapse runs along `u`);
    /// `false` when it is `u = fixed` (the collapse runs along `v`).
    pub(super) along_u: bool,
    /// The constant parameter of the side.
    pub(super) fixed: f64,
    /// The collapsed interval of the varying parameter.
    pub(super) lo: f64,
    /// See [`Self::lo`].
    pub(super) hi: f64,
    /// The 3D point the interval collapses onto.
    pub(super) point: Point3,
}

/// Maximal parameter intervals on each of the four domain sides over which
/// the boundary iso-curve is one point within `tol`.
///
/// A clamped boundary iso-curve is the B-spline of the boundary row (or
/// column) of control points; the knot span `[t_k, t_k+1]` depends only on
/// points `k - p ..= k`, so a run `a ..= b` of mutually coincident points
/// (`b - a >= p`) collapses `[t_(a+p), t_(b+1)]` exactly. Positive weights
/// do not move a point that every basis function maps to the same place.
pub(super) fn collapsed_sides(surface: &NurbsSurface, tol: f64) -> Vec<CollapsedSide> {
    let net = surface.control_points();
    let Some(first_row) = net.first() else {
        return Vec::new();
    };
    let (n_u, n_v) = (net.len(), first_row.len());
    if n_u == 0 || n_v == 0 || net.iter().any(|row| row.len() != n_v) {
        return Vec::new();
    }
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let mut sides = Vec::new();
    // Sides u = u0 / u1: rows 0 and n_u - 1, varying v.
    for (row, fixed) in [(0, u0), (n_u - 1, u1)] {
        let points: Vec<Point3> = net[row].clone();
        collapsed_runs(
            &points,
            surface.degree_v(),
            surface.knots_v(),
            tol,
            |lo, hi, point| {
                sides.push(CollapsedSide {
                    along_u: false,
                    fixed,
                    lo,
                    hi,
                    point,
                });
            },
        );
    }
    // Sides v = v0 / v1: columns 0 and n_v - 1, varying u.
    for (column, fixed) in [(0, v0), (n_v - 1, v1)] {
        let points: Vec<Point3> = net.iter().map(|row| row[column]).collect();
        collapsed_runs(
            &points,
            surface.degree_u(),
            surface.knots_u(),
            tol,
            |lo, hi, point| {
                sides.push(CollapsedSide {
                    along_u: true,
                    fixed,
                    lo,
                    hi,
                    point,
                });
            },
        );
    }
    sides
}

fn collapsed_runs(
    points: &[Point3],
    degree: usize,
    knots: &[f64],
    tol: f64,
    mut emit: impl FnMut(f64, f64, Point3),
) {
    let n = points.len();
    if knots.len() != n + degree + 1 {
        return;
    }
    let mut a = 0;
    while a < n {
        let anchor = points[a];
        let mut b = a;
        while b + 1 < n && (points[b + 1] - anchor).length() <= tol {
            b += 1;
        }
        if b - a >= degree {
            let (lo, hi) = (knots[a + degree], knots[b + 1]);
            if hi > lo {
                emit(lo, hi, anchor);
            }
        }
        a = b + 1;
    }
}

/// Re-place boundary chart samples that sit on a collapsed side.
///
/// A sample is on side `s` when its chart point lies on `s`'s UV segment (the
/// projection landed somewhere along the collapse) or its 3D point is within
/// `tol` of the collapse point. Each maximal cyclic run of consecutive such
/// samples on one side, bounded by two samples that are not, is spread
/// monotonically over the collapsed interval at the side's fixed parameter,
/// from the end nearer the incoming neighbour to the end nearer the outgoing
/// one (each neighbour's coordinate clamped into the interval), so every
/// re-placed sample charts onto the collapse point. A run whose neighbours do
/// not separate along the side (the boundary touches the point and turns
/// back) is left alone, as is a boundary made entirely of collapsed samples.
pub(super) fn respread_collapsed_samples(
    sides: &[CollapsedSide],
    points: &[Point3],
    uv: &mut [(f64, f64)],
    domain: ((f64, f64), (f64, f64)),
    tol: f64,
) {
    let n = uv.len();
    if sides.is_empty() || n < 3 || points.len() != n {
        return;
    }
    let ((u0, u1), (v0, v1)) = domain;
    let side_eps = |along_u: bool| {
        let width = if along_u { v1 - v0 } else { u1 - u0 };
        width.abs().max(f64::MIN_POSITIVE) * 1e-9
    };
    let along_eps = |along_u: bool| {
        let width = if along_u { u1 - u0 } else { v1 - v0 };
        width.abs().max(f64::MIN_POSITIVE) * 1e-9
    };
    let side_of = |i: usize| -> Option<usize> {
        let (u, v) = uv[i];
        sides.iter().position(|side| {
            let (fixed, along) = if side.along_u { (v, u) } else { (u, v) };
            let eps = along_eps(side.along_u);
            let on_segment = (fixed - side.fixed).abs() <= side_eps(side.along_u)
                && along >= side.lo - eps
                && along <= side.hi + eps;
            on_segment || (points[i] - side.point).length() <= tol
        })
    };
    let tags: Vec<Option<usize>> = (0..n).map(side_of).collect();
    // Start the cyclic walk just after an untagged sample, so no run wraps.
    let Some(start) = tags.iter().position(Option::is_none) else {
        return;
    };
    let mut k = 0;
    while k < n {
        let i = (start + k) % n;
        let Some(side_index) = tags[i] else {
            k += 1;
            continue;
        };
        let mut len = 1;
        while k + len < n && tags[(start + k + len) % n] == Some(side_index) {
            len += 1;
        }
        let prev = (i + n - 1) % n;
        let next = (start + k + len) % n;
        let side = sides[side_index];
        let coordinate = |p: (f64, f64)| {
            let along = if side.along_u { p.0 } else { p.1 };
            along.clamp(side.lo, side.hi)
        };
        let (a, b) = (coordinate(uv[prev]), coordinate(uv[next]));
        if (b - a).abs() > along_eps(side.along_u) {
            #[allow(clippy::cast_precision_loss)]
            for j in 0..len {
                let t = (j as f64 + 0.5) / len as f64;
                let along = (b - a).mul_add(t, a);
                uv[(i + j) % n] = if side.along_u {
                    (along, side.fixed)
                } else {
                    (side.fixed, along)
                };
            }
        }
        k += len;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    /// A bilinear-in-v, quadratic-in-u patch whose `u = 1` row collapses onto
    /// one point over the middle of the v domain: the shape a constant-radius
    /// blend has where its contact rail stops at a corner of equal radius.
    fn corner_patch() -> NurbsSurface {
        let mut rows: Vec<Vec<Point3>> = Vec::new();
        for (u, (x, z)) in [(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)].into_iter().enumerate() {
            let mut row = Vec::new();
            for (j, y) in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0].into_iter().enumerate() {
                row.push(if u == 2 && j == 3 {
                    Point3::new(2.0, 2.0, 0.0)
                } else {
                    Point3::new(x, y, z)
                });
            }
            rows.push(row);
        }
        NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.0],
            rows,
            vec![vec![1.0; 6]; 3],
        )
        .unwrap()
    }

    #[test]
    fn collapsed_interval_comes_from_coincident_control_points() {
        let sides = collapsed_sides(&corner_patch(), 1e-9);
        assert_eq!(sides.len(), 1);
        let side = sides[0];
        assert!(!side.along_u);
        assert!((side.fixed - 1.0).abs() < 1e-15);
        // Points 2..=3 of a degree-1 row: knots[3]..knots[4].
        assert!((side.lo - 0.4).abs() < 1e-15 && (side.hi - 0.6).abs() < 1e-15);
        assert!((side.point - Point3::new(2.0, 2.0, 0.0)).length() < 1e-15);
        let surface = corner_patch();
        for t in [0.4, 0.45, 0.5, 0.6] {
            assert!((surface.evaluate(1.0, t) - side.point).length() < 1e-12);
        }
        assert!((surface.evaluate(1.0, 0.3) - side.point).length() > 0.1);
    }

    #[test]
    fn whole_side_collapse_is_a_pole() {
        // Degree 2 in u with all three points of column 0 coincident.
        let p = Point3::new(1.0, 1.0, 1.0);
        let rows = vec![
            vec![p, Point3::new(0.0, 1.0, 0.0)],
            vec![p, Point3::new(0.0, 2.0, 0.0)],
            vec![p, Point3::new(0.0, 3.0, 0.0)],
        ];
        let surface = NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            rows,
            vec![vec![1.0, 2.0]; 3],
        )
        .unwrap();
        let sides = collapsed_sides(&surface, 1e-9);
        assert_eq!(sides.len(), 1);
        assert!(sides[0].along_u && sides[0].fixed.abs() < 1e-15);
        assert!(sides[0].lo.abs() < 1e-15 && (sides[0].hi - 1.0).abs() < 1e-15);
    }

    #[test]
    fn zigzag_on_the_collapse_is_respread_monotonically() {
        let surface = corner_patch();
        let sides = collapsed_sides(&surface, 1e-9);
        let corner = sides[0].point;
        // Boundary walks u = 1 upward: two clean samples, four ambiguous ones
        // charted out of order on the collapse, two clean samples after.
        let mut uv = vec![
            (0.0, 0.0),
            (1.0, 0.1),
            (1.0, 0.3),
            (1.0, 0.55),
            (1.0, 0.42),
            (1.0, 0.5),
            (1.0, 0.47),
            (1.0, 0.7),
            (1.0, 0.9),
            (0.0, 1.0),
        ];
        let mut points: Vec<Point3> = uv.iter().map(|&(u, v)| surface.evaluate(u, v)).collect();
        // A pool sample a hair off the corner whose chart point is wrong.
        points[2] = Point3::new(2.0, 1.999_999_9, 0.0);
        uv[2] = (1.0, 0.31);
        respread_collapsed_samples(&sides, &points, &mut uv, ((0.0, 1.0), (0.0, 1.0)), 1e-6);
        let along: Vec<f64> = uv[1..=7].iter().map(|p| p.1).collect();
        assert!(along.windows(2).all(|w| w[1] > w[0]), "{along:?}");
        assert!(uv[1..=7].iter().all(|p| (p.0 - 1.0).abs() < 1e-15));
        // Untouched samples keep their chart points bit for bit.
        assert_eq!(uv[0], (0.0, 0.0));
        assert_eq!(uv[1], (1.0, 0.1));
        assert_eq!(uv[7], (1.0, 0.7));
        // Every re-placed sample charts onto the collapse point.
        for i in 2..=6 {
            assert!((surface.evaluate(uv[i].0, uv[i].1) - corner).length() < 1e-12);
        }
    }

    #[test]
    fn boundary_touching_the_collapse_and_turning_back_is_left_alone() {
        let surface = corner_patch();
        let sides = collapsed_sides(&surface, 1e-9);
        let mut uv = vec![(0.0, 0.0), (1.0, 0.5), (0.0, 0.5), (0.5, 0.0)];
        // The neighbours of the collapsed sample sit at the same v.
        uv[0] = (0.5, 0.5);
        let points: Vec<Point3> = uv.iter().map(|&(u, v)| surface.evaluate(u, v)).collect();
        let before = uv.clone();
        respread_collapsed_samples(&sides, &points, &mut uv, ((0.0, 1.0), (0.0, 1.0)), 1e-6);
        assert_eq!(uv, before);
    }
}
