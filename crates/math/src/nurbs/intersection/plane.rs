//! Plane-NURBS surface intersection.

use crate::MathError;
use crate::nurbs::surface::NurbsSurface;
use crate::vec::Vec3;

use super::chaining::build_curves_from_points;
use super::{IntersectionCurve, IntersectionPoint, MAX_NEWTON_ITER};

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
    // grid spacing of each other, append the start point and interpolate
    // periodically so the stored curve is closed (start == end) and the FF
    // phase's closed-section machinery (seam adoption, window clipping) owns
    // it. Gated to the unambiguous case — one chain, endpoints mutually
    // nearest across the gap — so open branches never gain a spurious
    // closing segment.
    close_transversal_loop(&mut curves, &refined_points, u_step, v_step, surface);

    Ok(curves)
}

/// Append-then-periodically-refit closure for a single-chain transversal loop.
///
/// See the call site for why this exists. Returns without touching `curves`
/// unless every gate holds: exactly one curve from more than eight refined
/// points, whose endpoints are each other's nearest refined neighbour across
/// the gap and sit within two grid spacings of each other in 3D.
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
    // Grid spacing in model units at the loop: the larger of the two
    // parametric steps pushed through the surface metric at the gap midpoint.
    let mid = super::Point3::new(
        0.5 * (first.x() + last.x()),
        0.5 * (first.y() + last.y()),
        0.5 * (first.z() + last.z()),
    );
    // Project the midpoint onto the surface for the metric; fail closed (no
    // closure without a metric) when projection fails.
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
