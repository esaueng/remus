//! Plane-NURBS surface intersection.

use crate::MathError;
use crate::nurbs::surface::NurbsSurface;
use crate::vec::Vec3;

use super::chaining::build_curves_from_points;
use super::{IntersectionCurve, IntersectionPoint, MAX_NEWTON_ITER};

/// Certify one transverse constant-parameter ruling of a bilinear patch.
///
/// Only unit-weight, clamped, single-span degree-(1,1) patches qualify.
/// Opposite boundary directions must be plane-parallel to roundoff. A
/// merely modeling-tolerance-sized slope does not qualify as an exact ruling.
/// Coincident, boundary-only, degenerate, and unproved intersections return
/// `None` so callers can retain their general intersection path.
#[must_use]
pub fn plane_bilinear_ruling(
    surface: &NurbsSurface,
    plane_normal: Vec3,
    plane_d: f64,
    tolerance: crate::tolerance::Tolerance,
) -> Option<(crate::vec::Point3, crate::vec::Point3)> {
    let points = surface.control_points();
    if surface.degree_u() != 1
        || surface.degree_v() != 1
        || points.len() != 2
        || points.iter().any(|row| row.len() != 2)
        || surface.is_rational()
    {
        return None;
    }
    // Knot equality is structural multiplicity, not a geometric tolerance.
    let clamped = |knots: &[f64]| {
        knots.len() == 4
            && knots[0].to_bits() == knots[1].to_bits()
            && knots[2].to_bits() == knots[3].to_bits()
            && knots[1] < knots[2]
    };
    if !clamped(surface.knots_u()) || !clamped(surface.knots_v()) {
        return None;
    }
    let length = plane_normal.length();
    if !length.is_finite()
        || length <= f64::MIN_POSITIVE
        || !plane_d.is_finite()
        || !tolerance.linear.is_finite()
        || tolerance.linear <= 0.0
    {
        return None;
    }
    let normal = plane_normal * (1.0 / length);
    let distance = plane_d / length;
    let signed = |p: crate::vec::Point3| normal.dot(Vec3::new(p.x(), p.y(), p.z())) - distance;
    let distances = [
        [signed(points[0][0]), signed(points[0][1])],
        [signed(points[1][0]), signed(points[1][1])],
    ];
    if distances.iter().flatten().any(|d| !d.is_finite()) {
        return None;
    }
    // Modeling tolerance alone would certify a near-ruling curved section.
    let constant = |a: crate::vec::Point3, b: crate::vec::Point3| {
        let delta = b - a;
        let magnitude = (normal.x() * delta.x()).abs()
            + (normal.y() * delta.y()).abs()
            + (normal.z() * delta.z()).abs();
        normal.dot(delta).abs() <= (16.0 * f64::EPSILON * magnitude).min(tolerance.linear)
    };
    let u = surface.domain_u();
    let v = surface.domain_v();
    for along_u in [true, false] {
        let (a, b, proven) = if along_u {
            (
                distances[0][0],
                distances[1][0],
                constant(points[0][0], points[0][1]) && constant(points[1][0], points[1][1]),
            )
        } else {
            (
                distances[0][0],
                distances[0][1],
                constant(points[0][0], points[1][0]) && constant(points[0][1], points[1][1]),
            )
        };
        if !proven
            || a.abs() <= tolerance.linear
            || b.abs() <= tolerance.linear
            || a.is_sign_positive() == b.is_sign_positive()
        {
            continue;
        }
        let fraction = a / (a - b);
        if !fraction.is_finite() || !(0.0..1.0).contains(&fraction) {
            continue;
        }
        let (start, end) = if along_u {
            let parameter = u.0 + (u.1 - u.0) * fraction;
            (
                surface.evaluate(parameter, v.0),
                surface.evaluate(parameter, v.1),
            )
        } else {
            let parameter = v.0 + (v.1 - v.0) * fraction;
            (
                surface.evaluate(u.0, parameter),
                surface.evaluate(u.1, parameter),
            )
        };
        let chord_length = (end - start).length();
        if chord_length.is_finite()
            && chord_length > tolerance.linear
            && signed(start).abs() <= tolerance.linear
            && signed(end).abs() <= tolerance.linear
        {
            return Some((start, end));
        }
    }
    None
}

/// Intersect a plane with a NURBS surface.
///
/// Returns a list of intersection curves (there may be multiple
/// disconnected branches).
///
/// # Parameters
///
/// - `surface`: The NURBS surface
/// - `plane_normal`: Normal of the cutting plane
/// - `plane_d`: Signed distance from origin (`n · p = d` for points on plane)
/// - `samples`: Grid resolution for finding seed points (e.g., 50)
///
/// # Errors
///
/// Returns an error if NURBS evaluation fails or curve fitting fails.
pub fn intersect_plane_nurbs(
    surface: &NurbsSurface,
    plane_normal: Vec3,
    plane_d: f64,
    samples: usize,
) -> Result<Vec<IntersectionCurve>, MathError> {
    let n = samples.max(10);

    // Phase 1: Sample the signed distance field on a grid.
    let mut distances = vec![vec![0.0_f64; n]; n];

    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();

    #[allow(clippy::cast_precision_loss)]
    let u_step = (u_max - u_min) / (n - 1) as f64;
    #[allow(clippy::cast_precision_loss)]
    let v_step = (v_max - v_min) / (n - 1) as f64;

    #[allow(clippy::cast_precision_loss)]
    for (i, row) in distances.iter_mut().enumerate() {
        let u = u_min + i as f64 * u_step;
        for (j, dist) in row.iter_mut().enumerate() {
            let v = v_min + j as f64 * v_step;
            let pt = surface.evaluate(u, v);
            let pt_vec = Vec3::new(pt.x(), pt.y(), pt.z());
            *dist = plane_normal.dot(pt_vec) - plane_d;
        }
    }

    // Phase 2: Find zero-crossings between adjacent grid cells.
    //
    // A sign change found on a cell edge is pushed once per UNIQUE grid edge,
    // not once per incident cell. The old per-cell scan pushed the same
    // geometric crossing twice (as the right edge of cell (i,j) and the left
    // edge of cell (i+1,j), with bit-identical parameters), and the doubled
    // cloud drove the chaining threshold's nearest-neighbour average to zero
    // — every point's nearest neighbour was its own twin — so each twin pair
    // chained alone, deduped to one point, and was discarded as too short.
    // A full transversal loop (e.g. a plane slicing a NURBS cylinder wall)
    // therefore returned zero curves. Unique-edge emission keeps the average
    // at the true along-curve spacing.
    let mut crossing_points: Vec<(f64, f64, super::Point3)> = Vec::new();

    #[allow(clippy::cast_precision_loss)]
    for i in 0..n - 1 {
        for j in 0..n - 1 {
            let d00 = distances[i][j];
            let d10 = distances[i + 1][j];
            let d01 = distances[i][j + 1];
            let d11 = distances[i + 1][j + 1];

            // Check edges for sign changes.
            let u0 = u_min + i as f64 * u_step;
            let u1 = u_min + (i + 1) as f64 * u_step;
            let v0 = v_min + j as f64 * v_step;
            let v1 = v_min + (j + 1) as f64 * v_step;

            // Bottom edge (i,j) -> (i+1,j): every cell owns its bottom edge.
            if d00 * d10 < 0.0 {
                let t = d00 / (d00 - d10);
                let u = u0.mul_add(1.0 - t, u1 * t);
                let pt = surface.evaluate(u, v0);
                crossing_points.push((u, v0, pt));
            }

            // Left edge (i,j) -> (i,j+1): every cell owns its left edge.
            if d00 * d01 < 0.0 {
                let t = d00 / (d00 - d01);
                let v = v0.mul_add(1.0 - t, v1 * t);
                let pt = surface.evaluate(u0, v);
                crossing_points.push((u0, v, pt));
            }

            // Top edge (i,j+1) -> (i+1,j+1): owned by the last row only.
            if j + 1 == n - 1 && d01 * d11 < 0.0 {
                let t = d01 / (d01 - d11);
                let u = u0.mul_add(1.0 - t, u1 * t);
                let pt = surface.evaluate(u, v1);
                crossing_points.push((u, v1, pt));
            }

            // Right edge (i+1,j) -> (i+1,j+1): owned by the last column only.
            if i + 1 == n - 1 && d10 * d11 < 0.0 {
                let t = d10 / (d10 - d11);
                let v = v0.mul_add(1.0 - t, v1 * t);
                let pt = surface.evaluate(u1, v);
                crossing_points.push((u1, v, pt));
            }
        }
    }

    if crossing_points.is_empty() {
        return Ok(Vec::new());
    }

    // Phase 3: Refine crossing points with Newton iteration.
    let mut refined_points: Vec<IntersectionPoint> = Vec::new();
    for (u_guess, v_guess, _pt) in &crossing_points {
        if let Some(refined) =
            refine_plane_surface_point(surface, plane_normal, plane_d, *u_guess, *v_guess)
        {
            refined_points.push(refined);
        }
    }

    if refined_points.is_empty() {
        return Ok(Vec::new());
    }

    // Phase 4: Sort points and connect them into curves.
    // Simple approach: sort by parameter distance and group into chains.
    let mut curves = build_curves_from_points(&refined_points)?;

    // A closed transversal loop (a plane slicing a tube wall) chains into one
    // component whose nearest-neighbour walk starts and ends one grid spacing
    // apart: the fitted curve is an open arc with a grid-sized gap, and no
    // downstream weld band (1e-5 section reader, 1e-3 junction trigger) can
    // close a 0.3–0.6 model-unit gap — the loop never splits either face.
    // Close such loops exactly: when a single chain's endpoints land within a
    // grid spacing of each other AND the carrier domain continues across the
    // gap (the seam is interior to the carrier, not a boundary rim), append
    // the start point and interpolate so the stored curve is closed
    // (start == end) and the FF phase's closed-section machinery (seam
    // adoption, window clipping) owns it. Endpoint proximity alone never
    // closes: an open carrier whose arc stops a grid spacing short of its
    // own rim (e.g. a near-full tube with a real wedge gap) chains
    // identically in 3D but must stay open. Gated to the unambiguous case —
    // one chain, endpoints mutually nearest across the gap, carrier-domain
    // evidence — so open branches never gain a spurious closing segment.
    restore_open_isoparametric_order(&mut curves, surface)?;
    close_transversal_loop(&mut curves, &refined_points, u_step, v_step, surface);

    Ok(curves)
}

/// A nearest-neighbor walk can jump across a narrow physical wedge and leave
/// its gap inside the arc. On an open isoparametric section the chart gives
/// an unambiguous order, including the two real boundary endpoints.
fn restore_open_isoparametric_order(
    curves: &mut [IntersectionCurve],
    surface: &NurbsSurface,
) -> Result<(), MathError> {
    let [curve] = curves else {
        return Ok(());
    };
    let Some(first) = curve.points.first() else {
        return Ok(());
    };
    let u = surface.domain_u();
    let v = surface.domain_v();
    let constant_u = curve
        .points
        .iter()
        .all(|p| (p.param1.0 - first.param1.0).abs() <= (u.1 - u.0) * 1e-10);
    let constant_v = curve
        .points
        .iter()
        .all(|p| (p.param1.1 - first.param1.1).abs() <= (v.1 - v.0) * 1e-10);
    let along_u = if constant_v && !surface.is_periodic_u() {
        true
    } else if constant_u && !surface.is_periodic_v() {
        false
    } else {
        return Ok(());
    };
    curve.points.sort_by(|a, b| {
        let param = |p: &IntersectionPoint| if along_u { p.param1.0 } else { p.param1.1 };
        param(a).total_cmp(&param(b))
    });
    let positions: Vec<_> = curve.points.iter().map(|p| p.point).collect();
    curve.curve = crate::nurbs::fitting::interpolate(&positions, 3.min(positions.len() - 1))?;
    Ok(())
}

/// Require a closed carrier seam and coverage of that parameter direction.
/// Raw parameter jumps and 3D proximity cannot distinguish a real wedge gap.
fn chain_spans_closed_direction(
    surface: &NurbsSurface,
    params: &[(f64, f64)],
    u_step: f64,
    v_step: f64,
) -> bool {
    [true, false].into_iter().any(|along_u| {
        let (closed, (lo, hi), step) = if along_u {
            (surface.is_periodic_u(), surface.domain_u(), u_step)
        } else {
            (surface.is_periodic_v(), surface.domain_v(), v_step)
        };
        if !closed {
            return false;
        }
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for &(u, v) in params {
            let t = if along_u { u } else { v };
            min = min.min(t);
            max = max.max(t);
            // Control-point coincidence alone ignores rational weights.
            let (a, b) = if along_u {
                (surface.evaluate(lo, v), surface.evaluate(hi, v))
            } else {
                (surface.evaluate(u, lo), surface.evaluate(u, hi))
            };
            let separation = (a - b).length();
            if !separation.is_finite() || separation > 1e-7 {
                return false;
            }
        }
        min <= lo + step && max >= hi - step
    })
}

/// Close only a sampled loop spanning a geometrically closed carrier seam.
fn close_transversal_loop(
    curves: &mut [super::IntersectionCurve],
    refined: &[IntersectionPoint],
    u_step: f64,
    v_step: f64,
    surface: &NurbsSurface,
) {
    if curves.len() != 1 || refined.len() <= 8 {
        return;
    }
    let curve = &curves[0];
    if curve.points.len() <= 8 {
        return;
    }
    let first = curve.points[0].point;
    let last = curve.points[curve.points.len() - 1].point;
    let gap = (last - first).length();
    if !gap.is_finite() || gap <= 1e-6 {
        return;
    }
    // A branch touching an open carrier boundary must keep its endpoints.
    for point in &curve.points {
        for (parameter, (lo, hi), periodic) in [
            (point.param1.0, surface.domain_u(), surface.is_periodic_u()),
            (point.param1.1, surface.domain_v(), surface.is_periodic_v()),
        ] {
            let band = (hi - lo) * 1e-10;
            if !periodic && (parameter <= lo + band || parameter >= hi - band) {
                return;
            }
        }
    }
    let params: Vec<(f64, f64)> = curve.points.iter().map(|p| p.param1).collect();
    if !chain_spans_closed_direction(surface, &params, u_step, v_step) {
        return;
    }
    // Grid spacing in model units at the loop: the larger of the two
    // parametric steps pushed through the surface metric at the gap midpoint.
    // Project the midpoint onto the surface for the metric; fail closed (no
    // closure without a metric) when projection fails. The midpoint is used
    // ONLY for the metric scale here, never as closure evidence (see above).
    let mid = super::Point3::new(
        0.5 * (first.x() + last.x()),
        0.5 * (first.y() + last.y()),
        0.5 * (first.z() + last.z()),
    );
    let (u_m, v_m) = match crate::nurbs::projection::project_point_to_surface(surface, mid, 1e-7) {
        Ok(proj) => (proj.u, proj.v),
        Err(_) => return,
    };
    if !u_m.is_finite() || !v_m.is_finite() {
        return;
    }
    let derivs = surface.derivatives(u_m, v_m, 1);
    let grid_model = (derivs[1][0].length() * u_step)
        .max(derivs[0][1].length() * v_step)
        .max(1e-12);
    if gap > 2.0 * grid_model {
        return;
    }
    // Mutual-nearest gate: no THIRD refined point may lie strictly inside the
    // gap ball around either endpoint — otherwise the chain is an open branch
    // whose endpoint happens to sit near its start. Points coincident with an
    // endpoint (the two endpoints themselves) are skipped.
    let strictly_inside = |c: super::Point3| {
        refined.iter().any(|p| {
            let (df, dl) = ((p.point - first).length(), (p.point - last).length());
            if !(df.is_finite() && dl.is_finite()) {
                return false;
            }
            if df <= gap * 0.5 && dl >= gap * 0.5 || dl <= gap * 0.5 && df >= gap * 0.5 {
                // Coincident with one endpoint and far from the other: the
                // endpoints themselves (or their bit-identical twins).
                return false;
            }
            (df < gap && dl < gap) || (p.point - c).length() < gap * 0.5
        })
    };
    if strictly_inside(first) || strictly_inside(last) {
        return;
    }
    // Refit periodically: the closed interpolation keeps the chain's own
    // points (plus the exact start point re-appended) instead of inventing a
    // chord across the gap. There is no periodic-fit helper, so wrap the
    // point list (last == first) and interpolate openly: the clamped fit
    // passes through every point including the shared seam, and the FF phase
    // reads closure from coincident endpoints, not from curve periodicity.
    let mut closed: Vec<super::Point3> = curve.points.iter().map(|p| p.point).collect();
    closed.push(first);
    let degree = 3.min(closed.len().saturating_sub(1));
    let Ok(fitted) = crate::nurbs::fitting::interpolate(&closed, degree) else {
        return;
    };
    let start = fitted.evaluate(fitted.domain().0);
    if (start - first).length() > gap.max(1e-6) {
        return;
    }
    curves[0] = super::IntersectionCurve {
        curve: fitted,
        points: curve.points.clone(),
    };
}

/// Refine a plane-surface intersection point using Newton iteration.
fn refine_plane_surface_point(
    surface: &NurbsSurface,
    plane_normal: Vec3,
    plane_d: f64,
    u_guess: f64,
    v_guess: f64,
) -> Option<IntersectionPoint> {
    let mut u = u_guess;
    let mut v = v_guess;
    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();

    for _ in 0..MAX_NEWTON_ITER {
        let pt = surface.evaluate(u, v);
        let pt_vec = Vec3::new(pt.x(), pt.y(), pt.z());
        let f = plane_normal.dot(pt_vec) - plane_d;

        if f.abs() < 1e-12 {
            return Some(IntersectionPoint {
                point: pt,
                param1: (u, v),
                param2: (0.0, 0.0),
            });
        }

        // Compute gradient of f w.r.t. (u, v).
        let derivs = surface.derivatives(u, v, 1);
        let du = derivs[1][0]; // dS/du
        let dv = derivs[0][1]; // dS/dv

        let grad_u = plane_normal.dot(du);
        let grad_v = plane_normal.dot(dv);

        let grad_len_sq = grad_u.mul_add(grad_u, grad_v * grad_v);
        if grad_len_sq < 1e-20 {
            break; // Singular.
        }

        // Steepest descent step.
        let step_size = f / grad_len_sq;
        u -= grad_u * step_size;
        v -= grad_v * step_size;

        u = u.clamp(u_min, u_max);
        v = v.clamp(v_min, v_max);
    }

    // Final check.
    let pt = surface.evaluate(u, v);
    let pt_vec = Vec3::new(pt.x(), pt.y(), pt.z());
    let f = plane_normal.dot(pt_vec) - plane_d;

    if f.abs() < 1e-6 {
        Some(IntersectionPoint {
            point: pt,
            param1: (u, v),
            param2: (0.0, 0.0),
        })
    } else {
        None
    }
}

#[cfg(test)]
mod ruling_tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::{tolerance::Tolerance, vec::Point3};

    fn saddle(scale: f64, offset: Vec3) -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![2.0, 2.0, 5.0, 5.0],
            vec![-7.0, -7.0, -3.0, -3.0],
            [-2.0, 2.0]
                .into_iter()
                .map(|x| {
                    [-2.0, 2.0]
                        .into_iter()
                        .map(|y| Point3::new(x * scale, y * scale, 0.1 * x * y * scale) + offset)
                        .collect()
                })
                .collect(),
            vec![vec![1.0; 2]; 2],
        )
        .unwrap()
    }

    #[test]
    fn plane_bilinear_ruling_preserves_both_directions_scale_and_translation() {
        let tolerance = Tolerance::default();
        for scale in [1e-3, 1.0, 1e3] {
            for offset in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(13.0, -7.0, 5.0)] {
                let surface = saddle(scale, offset);
                for normal in [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)] {
                    for multiplier in [-3.0, 1.0] {
                        let (start, end) = plane_bilinear_ruling(
                            &surface,
                            normal * multiplier,
                            (0.5 * scale + normal.dot(offset)) * multiplier,
                            tolerance,
                        )
                        .unwrap();
                        for i in 0..=16 {
                            let p = start + (end - start) * (f64::from(i) / 16.0);
                            let local = p - offset;
                            assert!(
                                (normal.dot(Vec3::new(local.x(), local.y(), local.z()))
                                    - 0.5 * scale)
                                    .abs()
                                    < tolerance.linear
                            );
                            assert!(
                                (local.z() - 0.1 * local.x() * local.y() / scale).abs()
                                    < tolerance.linear
                            );
                        }
                        assert!((end - start).length() > 4.0 * scale);
                    }
                }
            }
        }
    }

    #[test]
    fn plane_bilinear_ruling_declines_overflowing_chord() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            [-1.0, 1.0]
                .into_iter()
                .map(|x| {
                    [-1e200, 1e200]
                        .into_iter()
                        .map(|y| Point3::new(x, y, 0.0))
                        .collect()
                })
                .collect(),
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();
        assert!(
            plane_bilinear_ruling(
                &surface,
                Vec3::new(1.0, 0.0, 0.0),
                0.0,
                Tolerance::default()
            )
            .is_none()
        );
    }

    #[test]
    fn plane_bilinear_ruling_declines_nonrulings_contacts_and_rational_patches() {
        let surface = saddle(1.0, Vec3::new(0.0, 0.0, 0.0));
        let tolerance = Tolerance::default();
        for (normal, distance) in [
            (Vec3::new(0.0, 0.0, 1.0), 0.1),
            (Vec3::new(0.0, 0.0, 1.0), 0.0),
            (Vec3::new(1.0, 0.0, 1e-9), 0.5),
            (Vec3::new(1.0, 0.0, 0.0), 2.0),
            (Vec3::new(1.0, 0.0, 0.0), 3.0),
            (Vec3::new(0.0, 0.0, 0.0), 0.0),
            (Vec3::new(1.0, 0.0, 0.0), f64::NAN),
        ] {
            assert!(plane_bilinear_ruling(&surface, normal, distance, tolerance).is_none());
        }
        let mut weights = surface.weights().to_vec();
        weights[0][0] = 1.0 + f64::EPSILON;
        let rational = NurbsSurface::new(
            1,
            1,
            surface.knots_u().to_vec(),
            surface.knots_v().to_vec(),
            surface.control_points().to_vec(),
            weights,
        )
        .unwrap();
        assert!(
            plane_bilinear_ruling(&rational, Vec3::new(1.0, 0.0, 0.0), 0.5, tolerance).is_none()
        );
        let unclamped = NurbsSurface::new(
            1,
            1,
            vec![1.0, 2.0, 5.0, 6.0],
            surface.knots_v().to_vec(),
            surface.control_points().to_vec(),
            surface.weights().to_vec(),
        )
        .unwrap();
        assert!(
            plane_bilinear_ruling(&unclamped, Vec3::new(1.0, 0.0, 0.0), 0.5, tolerance).is_none()
        );
        let mut planar = surface.control_points().to_vec();
        for p in planar.iter_mut().flatten() {
            *p = Point3::new(p.x(), p.y(), 0.0);
        }
        let planar = NurbsSurface::new(
            1,
            1,
            surface.knots_u().to_vec(),
            surface.knots_v().to_vec(),
            planar,
            surface.weights().to_vec(),
        )
        .unwrap();
        assert!(plane_bilinear_ruling(&planar, Vec3::new(0.0, 0.0, 1.0), 0.0, tolerance).is_none());
    }

    #[test]
    fn plane_bilinear_ruling_declines_multispan_degree_and_degenerate_sections() {
        let surface = saddle(1.0, Vec3::new(0.0, 0.0, 0.0));
        let mut points = surface.control_points().to_vec();
        points.insert(
            1,
            vec![Point3::new(0.0, -2.0, 0.0), Point3::new(0.0, 2.0, 0.0)],
        );
        for (degree, knots) in [
            (1, vec![0.0, 0.0, 0.5, 1.0, 1.0]),
            (2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
        ] {
            let patch = NurbsSurface::new(
                degree,
                1,
                knots,
                surface.knots_v().to_vec(),
                points.clone(),
                vec![vec![1.0; 2]; 3],
            )
            .unwrap();
            assert!(
                plane_bilinear_ruling(&patch, Vec3::new(1.0, 0.0, 0.0), 0.5, Tolerance::default())
                    .is_none()
            );
        }
        let points = vec![
            vec![Point3::new(-2.0, 0.0, 0.0); 2],
            vec![Point3::new(2.0, 0.0, 0.0); 2],
        ];
        let patch = NurbsSurface::new(
            1,
            1,
            surface.knots_u().to_vec(),
            surface.knots_v().to_vec(),
            points,
            surface.weights().to_vec(),
        )
        .unwrap();
        assert!(
            plane_bilinear_ruling(&patch, Vec3::new(1.0, 0.0, 0.0), 0.5, Tolerance::default())
                .is_none()
        );
    }
}
