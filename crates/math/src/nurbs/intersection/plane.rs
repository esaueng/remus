//! Plane-NURBS surface intersection.

use crate::MathError;
use crate::compact_sort::CompactSort;
use crate::nurbs::surface::NurbsSurface;
use crate::vec::Vec3;

use super::chaining::build_curves_from_points;
use super::{IntersectionCurve, IntersectionPoint, MAX_NEWTON_ITER};

/// Seed samples per knot span in [`span_adaptive_samples`].
const SEEDS_PER_SPAN: usize = 4;

/// Most seed samples [`span_adaptive_samples`] gives one direction.
const MAX_SPAN_ADAPTIVE_SAMPLES: usize = 4097;

/// Most seed samples [`span_adaptive_samples`] gives the whole grid.
const MAX_SPAN_ADAPTIVE_GRID: usize = 1 << 20;

/// Number of distinct knot spans of a clamped knot vector over `domain`.
fn knot_spans(knots: &[f64], (lo, hi): (f64, f64)) -> usize {
    let mut spans = 1;
    let mut last = lo;
    for &k in knots {
        if k > last && k < hi {
            spans += 1;
            last = k;
        }
    }
    spans
}

/// Per-direction seed-grid sizes for [`intersect_plane_nurbs_grid`] that give
/// every knot span several samples.
///
/// A fixed square grid ignores the span structure: on a face of many spans in
/// one direction (a helical sweep side covering every turn has 70 spans a
/// turn) it puts under one sample in each span, the zero crossings come out
/// far apart along that direction and dense across it, and nearest-neighbour
/// chaining breaks one section into many fragments. Each direction here gets
/// `max(min_samples, 4 · spans + 1)` samples, at most 4097, and the larger
/// direction is reduced so the grid stays within 2²⁰ samples. A surface with
/// at most `(min_samples − 1) / 4` spans each way keeps the plain
/// `min_samples × min_samples` grid.
#[must_use]
pub fn span_adaptive_samples(surface: &NurbsSurface, min_samples: usize) -> (usize, usize) {
    let axis = |knots: &[f64], domain: (f64, f64)| {
        (SEEDS_PER_SPAN * knot_spans(knots, domain) + 1)
            .max(min_samples)
            .min(MAX_SPAN_ADAPTIVE_SAMPLES.max(min_samples))
    };
    let mut n_u = axis(surface.knots_u(), surface.domain_u());
    let mut n_v = axis(surface.knots_v(), surface.domain_v());
    if n_u.saturating_mul(n_v) > MAX_SPAN_ADAPTIVE_GRID {
        if n_u >= n_v {
            n_u = (MAX_SPAN_ADAPTIVE_GRID / n_v).max(min_samples);
        } else {
            n_v = (MAX_SPAN_ADAPTIVE_GRID / n_u).max(min_samples);
        }
    }
    (n_u, n_v)
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
/// - `samples`: Grid resolution for finding seed points (e.g., 50), the same
///   in both directions; see [`intersect_plane_nurbs_grid`] for a grid sized
///   per direction
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
    intersect_plane_nurbs_grid(surface, plane_normal, plane_d, samples, samples)
}

/// [`intersect_plane_nurbs`] seeded from a `samples_u × samples_v` grid
/// (each at least 10), for example sized by [`span_adaptive_samples`].
///
/// # Errors
///
/// Returns an error if NURBS evaluation fails or curve fitting fails.
#[allow(clippy::too_many_lines)]
pub fn intersect_plane_nurbs_grid(
    surface: &NurbsSurface,
    plane_normal: Vec3,
    plane_d: f64,
    samples_u: usize,
    samples_v: usize,
) -> Result<Vec<IntersectionCurve>, MathError> {
    let n_u = samples_u.max(10);
    let n_v = samples_v.max(10);

    // Phase 1: Sample the signed distance field on a grid.
    let mut distances = vec![vec![0.0_f64; n_v]; n_u];

    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();

    #[allow(clippy::cast_precision_loss)]
    let u_step = (u_max - u_min) / (n_u - 1) as f64;
    #[allow(clippy::cast_precision_loss)]
    let v_step = (v_max - v_min) / (n_v - 1) as f64;

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
    for i in 0..n_u - 1 {
        for j in 0..n_v - 1 {
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
            if j + 1 == n_v - 1 && d01 * d11 < 0.0 {
                let t = d01 / (d01 - d11);
                let u = u0.mul_add(1.0 - t, u1 * t);
                let pt = surface.evaluate(u, v1);
                crossing_points.push((u, v1, pt));
            }

            // Right edge (i+1,j) -> (i+1,j+1): owned by the last column only.
            if i + 1 == n_u - 1 && d10 * d11 < 0.0 {
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
    curve.points.compact_sort_by(|a, b| {
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

/// Whether both ends of a chain lie at least two grid steps inside every
/// open parameter boundary of the carrier.
///
/// A plane section of a smooth patch ends only on the patch boundary or
/// closes on itself, so a chain whose two ends are both well inside the
/// domain is a closed loop the nearest-neighbour walk left open by one grid
/// spacing. The open-rim case `close_transversal_loop` must keep open (an
/// arc stopping a grid spacing short of its own rim) has its ends on that
/// rim and is excluded here.
fn chain_ends_interior(
    surface: &NurbsSurface,
    params: &[(f64, f64)],
    u_step: f64,
    v_step: f64,
) -> bool {
    let (Some(&first), Some(&last)) = (params.first(), params.last()) else {
        return false;
    };
    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    [first, last].iter().all(|&(u, v)| {
        (surface.is_periodic_u() || (u > u_lo + 2.0 * u_step && u < u_hi - 2.0 * u_step))
            && (surface.is_periodic_v() || (v > v_lo + 2.0 * v_step && v < v_hi - 2.0 * v_step))
    })
}

/// Close sampled loops: the single chain spanning a geometrically closed
/// carrier seam, and any chain whose ends both lie inside the carrier
/// ([`chain_ends_interior`]).
fn close_transversal_loop(
    curves: &mut [super::IntersectionCurve],
    refined: &[IntersectionPoint],
    u_step: f64,
    v_step: f64,
    surface: &NurbsSurface,
) {
    if refined.len() <= 8 {
        return;
    }
    let single = curves.len() == 1;
    for curve in curves.iter_mut() {
        close_one_loop(curve, single, refined, u_step, v_step, surface);
    }
}

fn close_one_loop(
    curve: &mut super::IntersectionCurve,
    single: bool,
    refined: &[IntersectionPoint],
    u_step: f64,
    v_step: f64,
    surface: &NurbsSurface,
) {
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
    let seam_loop = single && chain_spans_closed_direction(surface, &params, u_step, v_step);
    if !seam_loop && !chain_ends_interior(surface, &params, u_step, v_step) {
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
    // An interior loop reads the metric at its own two ends instead: their
    // parameters are known, and a projection of the midpoint can seed on the
    // wrong turn of a surface that winds.
    let metric_at: Vec<(f64, f64)> = if seam_loop {
        match crate::nurbs::projection::project_point_to_surface(surface, mid, 1e-7) {
            Ok(proj) => vec![(proj.u, proj.v)],
            Err(_) => return,
        }
    } else {
        vec![params[0], params[params.len() - 1]]
    };
    let mut grid_model = 1e-12_f64;
    for (u_m, v_m) in metric_at {
        if !u_m.is_finite() || !v_m.is_finite() {
            return;
        }
        let derivs = surface.derivatives(u_m, v_m, 1);
        grid_model = grid_model
            .max(derivs[1][0].length() * u_step)
            .max(derivs[0][1].length() * v_step);
    }
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
    curve.curve = fitted;
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
