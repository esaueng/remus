//! Point projection onto NURBS curves and surfaces.
//!
//! Finds the closest point on a curve or surface to a given point in space.
//! Used for Boolean classification, snapping, distance queries, and
//! tessellation refinement.
//!
//! Algorithms follow NURBS Book A6.1–A6.6: subdivision for initial guess
//! followed by Newton–Raphson refinement.

use crate::MathError;
use crate::nurbs::curve::NurbsCurve;
use crate::nurbs::decompose::curve_to_bezier_segments;
use crate::nurbs::surface::NurbsSurface;
use crate::vec::Point3;

/// Maximum Newton iterations before declaring convergence failure.
const MAX_ITERATIONS: usize = 50;

/// Newton iterations allowed when the caller supplied the starting point.
///
/// Failing early is what lets a caller treat a seeded call as a cheap attempt
/// it can walk away from: the point of a cap is that a seed outside the
/// answer's basin costs less than the grid search it was trying to skip. That
/// search is `(SURFACE_GRID_SIZE + 1)²` = 81 surface evaluations, against
/// roughly two or three per Newton step, so the break-even is somewhere near
/// thirty steps and this leaves clear headroom.
///
/// It must not be tighter than a legitimate solve needs, though, and that is
/// what sets the floor. A seed already on the surface converges on the
/// point-coincidence test within a couple of steps, but one projecting a point
/// that sits *off* the surface has to reach the perpendicular foot instead,
/// which takes longer — cutting those short turns a working seed into a
/// rejection that pays for both paths. Measured over trimmed NURBS faces,
/// eight was tight enough to do exactly that; twelve costs the same as thirty.
const SEEDED_MAX_ITERATIONS: usize = 12;

/// Number of grid subdivisions per direction for surface coarse search.
const SURFACE_GRID_SIZE: usize = 8;

// ---------------------------------------------------------------------------
// Public result types
// ---------------------------------------------------------------------------

/// Result of projecting a point onto a curve.
#[derive(Debug, Clone, Copy)]
pub struct CurveProjection {
    /// Parameter value at the closest point.
    pub parameter: f64,
    /// The closest point on the curve.
    pub point: Point3,
    /// Distance from the input point to the closest point.
    pub distance: f64,
}

/// Result of projecting a point onto a surface.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceProjection {
    /// Parameter value u at the closest point.
    pub u: f64,
    /// Parameter value v at the closest point.
    pub v: f64,
    /// The closest point on the surface.
    pub point: Point3,
    /// Distance from the input point to the closest point.
    pub distance: f64,
}

// ---------------------------------------------------------------------------
// Curve projection
// ---------------------------------------------------------------------------

/// Find the closest point on a NURBS curve to the given point.
///
/// Uses Bezier decomposition for initial guess, then Newton–Raphson
/// refinement (NURBS Book A6.1 + A6.3–A6.4).
///
/// # Errors
///
/// Returns an error if Bezier decomposition fails (invalid curve data).
pub fn project_point_to_curve(
    curve: &NurbsCurve,
    point: Point3,
    tolerance: f64,
) -> Result<CurveProjection, MathError> {
    let knots = curve.knots();
    let p = curve.degree();
    let u_min = knots[p];
    let u_max = knots[knots.len() - p - 1];

    let candidates = curve_coarse_search(curve, point)?;

    // Run Newton from each candidate and keep the globally closest result.
    let mut best_u = u_min;
    let mut best_pt = curve.evaluate(u_min);
    let mut best_dist = (best_pt - point).length();

    for u_guess in candidates {
        let (u_refined, pt_refined) =
            curve_newton_refine(curve, point, u_guess, u_min, u_max, tolerance);
        let dist = (pt_refined - point).length();
        if dist < best_dist {
            best_dist = dist;
            best_u = u_refined;
            best_pt = pt_refined;
        }
    }

    Ok(CurveProjection {
        parameter: best_u,
        point: best_pt,
        distance: best_dist,
    })
}

/// Coarse search: decompose into Bezier segments and sample points to find
/// multiple candidate parameter values for Newton refinement.
///
/// Returns a sorted list of candidate parameters (best first) to use as
/// Newton seeds. Using multiple seeds avoids converging to a local minimum.
#[allow(clippy::cast_precision_loss)]
fn curve_coarse_search(curve: &NurbsCurve, point: Point3) -> Result<Vec<f64>, MathError> {
    let segments = curve_to_bezier_segments(curve)?;

    // Collect all (distance_sq, parameter) samples.
    let mut samples: Vec<(f64, f64)> = Vec::new();

    for seg in &segments {
        let knots = seg.knots();
        let p = seg.degree();
        let u_start = knots[p];
        let u_end = knots[knots.len() - p - 1];

        // Sample points along the segment.
        let n_samples = (p + 1).max(5) * 2;
        for i in 0..=n_samples {
            let t = i as f64 / n_samples as f64;
            let u = t.mul_add(u_end - u_start, u_start);
            let pt = seg.evaluate(u);
            let d_sq = (pt - point).length_squared();
            samples.push((d_sq, u));
        }
    }

    // Sort by distance and return the best candidates.
    samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Take the top few unique candidates (spatially separated).
    let mut candidates = Vec::new();
    let max_candidates = 5;
    for &(_, u) in &samples {
        if candidates.len() >= max_candidates {
            break;
        }
        // Skip candidates too close to one we already have.
        let dominated = candidates.iter().any(|&c: &f64| (c - u).abs() < 1e-10);
        if !dominated {
            candidates.push(u);
        }
    }

    Ok(candidates)
}

/// Newton–Raphson refinement for curve point projection.
///
/// Finds parameter u that minimizes ||C(u) - P|| starting from `u_init`.
/// Always returns a result — falls back to the best iterate if formal
/// convergence criteria are not met within [`MAX_ITERATIONS`].
#[allow(clippy::suspicious_operation_groupings)]
fn curve_newton_refine(
    curve: &NurbsCurve,
    point: Point3,
    u_init: f64,
    u_min: f64,
    u_max: f64,
    tolerance: f64,
) -> (f64, Point3) {
    let tol_sq = tolerance * tolerance;
    let mut u = u_init;
    let mut best_u = u;
    let mut best_dist_sq = f64::INFINITY;

    for _ in 0..MAX_ITERATIONS {
        let ders = curve.derivatives(u, 2);
        let c_pt = Point3::new(ders[0].x(), ders[0].y(), ders[0].z());
        let c_prime = ders[1]; // C'(u)
        let c_double_prime = ders[2]; // C''(u)
        let diff = c_pt - point; // C(u) - P

        let dist_sq = diff.length_squared();

        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_u = u;
        }

        // Convergence check 1: point coincidence.
        if dist_sq < tol_sq {
            return (u, c_pt);
        }

        // f(u) = C'(u) · (C(u) - P)
        let f_val = c_prime.dot(diff);

        // Convergence check 2: zero cosine (perpendicularity).
        // cos²(angle) = (C'·diff)² / (|C'|² · |diff|²) < tol²
        let c_prime_len_sq = c_prime.length_squared();
        if c_prime_len_sq > 1e-30 && dist_sq > tol_sq {
            let cos_sq = (f_val * f_val) / (c_prime_len_sq * dist_sq);
            if cos_sq < tol_sq {
                return (u, c_pt);
            }
        }

        // f'(u) = C''(u) · (C(u) - P) + |C'(u)|²
        let f_prime = c_double_prime.dot(diff) + c_prime_len_sq;

        // Guard against zero denominator.
        if f_prime.abs() < 1e-30 {
            return (u, c_pt);
        }

        let delta_u = f_val / f_prime;
        let u_new = (u - delta_u).clamp(u_min, u_max);

        // Guard NaN.
        if u_new.is_nan() {
            break;
        }

        // Convergence check 3: parameter step negligible.
        let du = (u_new - u).abs();
        if du < tolerance * (1.0 + u.abs()) {
            let pt = curve.evaluate(u_new);
            return (u_new, pt);
        }

        u = u_new;
    }

    // Return the best point found during iteration.
    let pt = curve.evaluate(best_u);
    (best_u, pt)
}

// ---------------------------------------------------------------------------
// Surface projection
// ---------------------------------------------------------------------------

/// Find the closest point on a NURBS surface to the given point.
///
/// Uses grid evaluation for initial guess, then 2D Newton–Raphson
/// refinement (NURBS Book A6.2 + A6.5–A6.6).
///
/// # Errors
///
/// Returns [`MathError::ConvergenceFailure`] if Newton iteration does not
/// converge within the maximum number of iterations.
pub fn project_point_to_surface(
    surface: &NurbsSurface,
    point: Point3,
    tolerance: f64,
) -> Result<SurfaceProjection, MathError> {
    let seed = surface_coarse_search(surface, point);
    refine_from(surface, point, tolerance, seed, MAX_ITERATIONS)
}

/// The coarse grid [`project_point_to_surface`] searches to find a Newton
/// start, evaluated once and reusable across many points.
///
/// The grid is a property of the surface alone — the query point only picks
/// the nearest node — yet [`project_point_to_surface`] rebuilds all
/// `(SURFACE_GRID_SIZE + 1)²` = 81 surface evaluations on every call. A caller
/// projecting many points onto one surface can build this once and hand it to
/// [`project_point_to_surface_with_grid`], which then costs 81 distance
/// comparisons instead of 81 surface evaluations — the same answer, bit for
/// bit, because it is the same grid and so the same seed.
///
/// Unlike a seed supplied by the caller, this changes nothing about where
/// Newton starts, so it carries none of the judgement
/// [`project_point_to_surface_seeded`] asks for.
#[derive(Debug, Clone)]
pub struct SurfaceSeedGrid {
    /// `(u, v, S(u, v))` in the same order `surface_coarse_search` walks them,
    /// so nearest-node ties resolve identically.
    nodes: Vec<(f64, f64, Point3)>,
}

impl SurfaceSeedGrid {
    /// Evaluate the coarse grid for `surface`.
    #[must_use]
    pub fn for_surface(surface: &NurbsSurface) -> Self {
        let (u_min, u_max, v_min, v_max) = surface_domain(surface);
        let n = SURFACE_GRID_SIZE;
        let mut nodes = Vec::with_capacity((n + 1) * (n + 1));
        for i in 0..=n {
            #[allow(clippy::cast_precision_loss)]
            let u = (i as f64 / n as f64).mul_add(u_max - u_min, u_min);
            for j in 0..=n {
                #[allow(clippy::cast_precision_loss)]
                let v = (j as f64 / n as f64).mul_add(v_max - v_min, v_min);
                nodes.push((u, v, surface.evaluate(u, v)));
            }
        }
        Self { nodes }
    }

    /// The grid node nearest `point`, matching `surface_coarse_search`'s scan
    /// order and its strict `<` tie-break.
    fn nearest(&self, point: Point3) -> (f64, f64) {
        let mut best = (0.0, 0.0);
        let mut best_dist_sq = f64::INFINITY;
        for &(u, v, pt) in &self.nodes {
            let d_sq = (pt - point).length_squared();
            if d_sq < best_dist_sq {
                best_dist_sq = d_sq;
                best = (u, v);
            }
        }
        best
    }
}

/// [`project_point_to_surface`] with the coarse grid supplied rather than
/// rebuilt.
///
/// Returns exactly what [`project_point_to_surface`] would, provided `grid`
/// was built from `surface`: same nodes, same scan order, same nearest node,
/// so the same Newton start and the same result. Passing a grid built from a
/// *different* surface is not unsafe but is meaningless — the seed would be a
/// point of some other surface.
///
/// # Errors
///
/// Returns [`MathError::ConvergenceFailure`] if Newton iteration does not
/// converge within the maximum number of iterations.
pub fn project_point_to_surface_with_grid(
    surface: &NurbsSurface,
    point: Point3,
    tolerance: f64,
    grid: &SurfaceSeedGrid,
) -> Result<SurfaceProjection, MathError> {
    refine_from(
        surface,
        point,
        tolerance,
        grid.nearest(point),
        MAX_ITERATIONS,
    )
}

/// Find the closest point on a NURBS surface to `point`, starting Newton from
/// `seed` rather than from a fresh coarse search.
///
/// [`project_point_to_surface`] spends a `(SURFACE_GRID_SIZE + 1)²` = 81-point
/// grid evaluation on every call purely to find somewhere to start. A caller
/// projecting many points onto one surface — successive samples along a trim
/// curve, a walk across a patch — already knows roughly where each one lands
/// and can say so, which is the bulk of the cost gone.
///
/// The seed is clamped into the knot domain, and a non-finite one is refused.
/// The refiner takes its starting iterate unclamped and has no NaN guard, and
/// several of its exits can hand that iterate straight back, so an unchecked
/// seed could surface as a result whose `(u, v)` lies outside the domain while
/// `distance` describes the clamped evaluation somewhere else entirely.
///
/// # `Ok` is not a result here — check the residual
///
/// Newton solves for perpendicularity, not for the nearest point, and takes no
/// descent test. From a seed outside the answer's basin it settles on whatever
/// stationary point of the distance it does reach: a second normal footpoint
/// across a concave patch, or the far side of a surface that closes on itself.
/// Two of the refiner's exits — a negligible step, a singular Jacobian — also
/// return successfully without having converged at all.
///
/// So treat this as an attempt, and judge it by
/// [`SurfaceProjection::distance`] against the residual the call ought to have
/// produced, falling back to [`project_point_to_surface`] when it disappoints.
/// That test is sound: `distance` is measured from the surface evaluated at
/// the returned parameters, so it reports where the iteration truly landed on
/// every one of those exits.
///
/// The one thing it cannot report is a degenerate row — a pole, a collapsed
/// edge — where every parameter along the row maps to the same point, so the
/// residual stays small whichever one comes back. [`project_point_to_surface`]
/// is no better there: its grid picks arbitrarily among the same equivalent
/// answers.
///
/// # Errors
///
/// Returns [`MathError::ParameterOutOfRange`] if either seed coordinate is not
/// finite, or [`MathError::ConvergenceFailure`] if Newton has not converged
/// within the seeded iteration budget — deliberately shorter than
/// [`project_point_to_surface`]'s, so that walking away from a poor seed costs
/// less than the grid search it was trying to skip.
pub fn project_point_to_surface_seeded(
    surface: &NurbsSurface,
    point: Point3,
    tolerance: f64,
    seed: (f64, f64),
) -> Result<SurfaceProjection, MathError> {
    if !seed.0.is_finite() || !seed.1.is_finite() {
        return Err(MathError::ParameterOutOfRange {
            value: if seed.0.is_finite() { seed.1 } else { seed.0 },
            min: f64::MIN,
            max: f64::MAX,
        });
    }
    refine_from(surface, point, tolerance, seed, SEEDED_MAX_ITERATIONS)
}

/// Newton-refine from `seed` — clamped into the knot domain — and package the
/// result. Shared by the seeded and coarse-searched entry points, so the two
/// differ only in where the starting iterate comes from and how long they are
/// willing to chase it.
fn refine_from(
    surface: &NurbsSurface,
    point: Point3,
    tolerance: f64,
    seed: (f64, f64),
    max_iterations: usize,
) -> Result<SurfaceProjection, MathError> {
    let (u_min, u_max, v_min, v_max) = surface_domain(surface);
    let (u_final, v_final, pt_final) = surface_newton_refine(
        surface,
        point,
        seed.0.clamp(u_min, u_max),
        seed.1.clamp(v_min, v_max),
        u_min,
        u_max,
        v_min,
        v_max,
        surface.is_periodic_u(),
        surface.is_periodic_v(),
        tolerance,
        max_iterations,
    )?;

    Ok(SurfaceProjection {
        u: u_final,
        v: v_final,
        point: pt_final,
        distance: (pt_final - point).length(),
    })
}

/// The surface's usable parameter domain `(u_min, u_max, v_min, v_max)`, read
/// off the knot vectors at the degree offsets.
fn surface_domain(surface: &NurbsSurface) -> (f64, f64, f64, f64) {
    let knots_u = surface.knots_u();
    let knots_v = surface.knots_v();
    let pu = surface.degree_u();
    let pv = surface.degree_v();
    (
        knots_u[pu],
        knots_u[knots_u.len() - pu - 1],
        knots_v[pv],
        knots_v[knots_v.len() - pv - 1],
    )
}

/// Coarse search: evaluate surface on a uniform grid and find the closest
/// grid point.
#[allow(clippy::cast_precision_loss)]
fn surface_coarse_search(surface: &NurbsSurface, point: Point3) -> (f64, f64) {
    let (u_min, u_max, v_min, v_max) = surface_domain(surface);

    let mut best_u = u_min;
    let mut best_v = v_min;
    let mut best_dist_sq = f64::INFINITY;

    let n = SURFACE_GRID_SIZE;
    for i in 0..=n {
        let u = (i as f64 / n as f64).mul_add(u_max - u_min, u_min);
        for j in 0..=n {
            let v = (j as f64 / n as f64).mul_add(v_max - v_min, v_min);
            let pt = surface.evaluate(u, v);
            let d_sq = (pt - point).length_squared();
            if d_sq < best_dist_sq {
                best_dist_sq = d_sq;
                best_u = u;
                best_v = v;
            }
        }
    }

    (best_u, best_v)
}

/// 2D Newton–Raphson refinement for surface point projection.
///
/// Solves the 2×2 system at each step to find the (u, v) that minimizes
/// ||S(u,v) - P||.
#[allow(clippy::too_many_arguments, clippy::similar_names)]
#[allow(clippy::suspicious_operation_groupings)]
fn surface_newton_refine(
    surface: &NurbsSurface,
    point: Point3,
    u_init: f64,
    v_init: f64,
    u_min: f64,
    u_max: f64,
    v_min: f64,
    v_max: f64,
    closed_u: bool,
    closed_v: bool,
    tolerance: f64,
    max_iterations: usize,
) -> Result<(f64, f64, Point3), MathError> {
    // On a closed (periodic) direction the domain bound is a seam, not a
    // boundary: clamping there pins Newton on the wrong side of the seam and
    // the small-step exit then returns the clamped point as a silent wrong
    // answer (a near-seam query projects up to half a period off). Wrap the
    // parameter across the seam instead so the walk reaches the true foot.
    let advance = |t: f64, delta: f64, lo: f64, hi: f64, closed: bool| {
        let t_new = t + delta;
        if closed && hi > lo {
            lo + (t_new - lo).rem_euclid(hi - lo)
        } else {
            t_new.clamp(lo, hi)
        }
    };
    let mut u = u_init;
    let mut v = v_init;

    for _ in 0..max_iterations {
        let ders = surface.derivatives(u, v, 1);
        let s_pt = Point3::new(ders[0][0].x(), ders[0][0].y(), ders[0][0].z());
        let deriv_u = ders[1][0]; // ∂S/∂u
        let deriv_v = ders[0][1]; // ∂S/∂v
        let r = s_pt - point; // S(u,v) - P

        // Convergence check 1: point coincidence.
        let dist = r.length();
        if dist < tolerance {
            return Ok((u, v, s_pt));
        }

        // Convergence check 2: zero cosine in both directions.
        let du_len = deriv_u.length();
        let dv_len = deriv_v.length();
        let dot_du_r = deriv_u.dot(r);
        let dot_dv_r = deriv_v.dot(r);
        if du_len > 0.0 && dv_len > 0.0 {
            let cos_u = dot_du_r.abs() / (du_len * dist);
            let cos_v = dot_dv_r.abs() / (dv_len * dist);
            if cos_u < tolerance && cos_v < tolerance {
                return Ok((u, v, s_pt));
            }
        }

        // Build the 2×2 Jacobian and right-hand side.
        // J = [S_u · S_u,  S_u · S_v]
        //     [S_v · S_u,  S_v · S_v]
        let j00 = deriv_u.dot(deriv_u);
        let j01 = deriv_u.dot(deriv_v);
        let j11 = deriv_v.dot(deriv_v);
        // rhs = [-S_u · r, -S_v · r]
        let rhs0 = -dot_du_r;
        let rhs1 = -dot_dv_r;

        // Solve 2×2 system via Cramer's rule: det = j00*j11 - j01²
        // Use a relative threshold so the singularity test stays meaningful
        // near surface poles / cone apex where both derivatives shrink to zero.
        let det = j00.mul_add(j11, -(j01 * j01));
        let (delta_u, delta_v) = if det.abs() < (j00 + j11).max(1e-30) * 1e-12 {
            // Near-singular: apply Tikhonov (Levenberg–Marquardt) regularisation
            // by adding λI to the normal equations.  This yields a step biased
            // toward zero rather than blowing up, preserving convergence near
            // poles and cone apices.
            let lambda = (j00 + j11).max(1e-10) * 1e-4;
            let j00r = j00 + lambda;
            let j11r = j11 + lambda;
            let det_r = j00r.mul_add(j11r, -(j01 * j01));
            if det_r.abs() < 1e-30 {
                // Still singular even after regularisation — fall back to a 1-D
                // search along whichever parameter axis has more gradient.
                if j00 > j11 {
                    (rhs0 / j00.max(1e-30), 0.0)
                } else if j11 > 1e-30 {
                    (0.0, rhs1 / j11.max(1e-30))
                } else {
                    return Ok((u, v, s_pt));
                }
            } else {
                (
                    rhs0.mul_add(j11r, -(rhs1 * j01)) / det_r,
                    j00r.mul_add(rhs1, -(j01 * rhs0)) / det_r,
                )
            }
        } else {
            (
                rhs0.mul_add(j11, -(rhs1 * j01)) / det,
                j00.mul_add(rhs1, -(j01 * rhs0)) / det,
            )
        };

        let u_new = advance(u, delta_u, u_min, u_max, closed_u);
        let v_new = advance(v, delta_v, v_min, v_max, closed_v);

        // Convergence check 3: parameter step negligible. A seam wrap makes
        // (u_new - u) span nearly the whole period, which reads as a large
        // step — never a spurious convergence, so the check stays sound.
        let step = (deriv_u * (u_new - u) + deriv_v * (v_new - v)).length();
        if step < tolerance {
            let pt = surface.evaluate(u_new, v_new);
            return Ok((u_new, v_new, pt));
        }

        u = u_new;
        v = v_new;
    }

    Err(MathError::ConvergenceFailure {
        iterations: max_iterations,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-8;

    /// A simple line from (0,0,0) to (10,0,0) as a degree-1 NURBS.
    fn line_curve() -> NurbsCurve {
        NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .expect("valid line")
    }

    /// Quarter circle arc as a rational NURBS (degree 2).
    fn quarter_circle() -> NurbsCurve {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![1.0, w, 1.0],
        )
        .expect("valid quarter circle")
    }

    /// Cubic Bezier curve.
    fn cubic_bezier() -> NurbsCurve {
        NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(3.0, 2.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            vec![1.0, 1.0, 1.0, 1.0],
        )
        .expect("valid cubic")
    }

    /// Bilinear flat patch (z=0 plane, from (0,0) to (1,1)).
    fn flat_patch() -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
                vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .expect("valid flat patch")
    }

    // -- Curve tests -------------------------------------------------------

    #[test]
    fn project_to_line() {
        let c = line_curve();
        // Point (5, 3, 0) — closest point should be (5, 0, 0) at u=0.5.
        let res =
            project_point_to_curve(&c, Point3::new(5.0, 3.0, 0.0), TOL).expect("should converge");
        assert!((res.parameter - 0.5).abs() < TOL, "u={}", res.parameter);
        assert!((res.point.x() - 5.0).abs() < TOL);
        assert!((res.point.y()).abs() < TOL);
        assert!((res.distance - 3.0).abs() < TOL, "dist={}", res.distance);
    }

    #[test]
    #[allow(clippy::suboptimal_flops)]
    fn project_to_circle() {
        let c = quarter_circle();
        // Point (2, 2, 0) — closest point should be on the unit circle at 45°.
        let res =
            project_point_to_curve(&c, Point3::new(2.0, 2.0, 0.0), TOL).expect("should converge");
        let expected = std::f64::consts::FRAC_1_SQRT_2;
        assert!(
            (res.point.x() - expected).abs() < 1e-6,
            "x={} expected={}",
            res.point.x(),
            expected
        );
        assert!(
            (res.point.y() - expected).abs() < 1e-6,
            "y={} expected={}",
            res.point.y(),
            expected
        );
        // Distance from (2,2) to unit circle at 45° = sqrt(8) - 1.
        let expected_dist = 2.0_f64.hypot(2.0) - 1.0;
        assert!(
            (res.distance - expected_dist).abs() < 1e-6,
            "dist={} expected={}",
            res.distance,
            expected_dist
        );
    }

    #[test]
    fn project_endpoint() {
        let c = cubic_bezier();
        // Project a point very close to the start endpoint.
        let res =
            project_point_to_curve(&c, Point3::new(0.0, 0.01, 0.0), TOL).expect("should converge");
        assert!(res.distance < 0.02, "dist={}", res.distance);
        assert!(res.parameter < 0.1, "u={}", res.parameter);
    }

    #[test]
    fn project_far_point() {
        let c = cubic_bezier();
        // A point far away should still converge.
        let res =
            project_point_to_curve(&c, Point3::new(2.0, 100.0, 0.0), TOL).expect("should converge");
        // The closest point should be roughly at the top of the curve (y ≈ 1.5).
        assert!(res.point.y() > 0.0);
        assert!(res.distance < 100.0);
    }

    #[test]
    fn project_on_curve() {
        let c = cubic_bezier();
        // Evaluate a point on the curve, then project it back.
        let u_orig = 0.3;
        let pt_on = c.evaluate(u_orig);
        let res = project_point_to_curve(&c, pt_on, TOL).expect("should converge");
        assert!(res.distance < TOL, "dist={}", res.distance);
        assert!(
            (res.parameter - u_orig).abs() < 1e-4,
            "u={} expected={}",
            res.parameter,
            u_orig
        );
    }

    // -- Surface tests -----------------------------------------------------

    #[test]
    fn project_to_flat_quad() {
        let s = flat_patch();
        // Point (0.5, 0.5, 3.0) — should project to (0.5, 0.5, 0.0).
        let res =
            project_point_to_surface(&s, Point3::new(0.5, 0.5, 3.0), TOL).expect("should converge");
        assert!((res.point.x() - 0.5).abs() < TOL, "x={}", res.point.x());
        assert!((res.point.y() - 0.5).abs() < TOL, "y={}", res.point.y());
        assert!((res.point.z()).abs() < TOL, "z={}", res.point.z());
        assert!((res.distance - 3.0).abs() < TOL, "dist={}", res.distance);
    }

    #[test]
    fn project_on_surface() {
        let s = flat_patch();
        // Point directly on the surface.
        let res =
            project_point_to_surface(&s, Point3::new(0.3, 0.7, 0.0), TOL).expect("should converge");
        assert!(res.distance < TOL, "dist={}", res.distance);
    }

    #[test]
    fn project_above_surface() {
        let s = flat_patch();
        // Point at height 1 above the center.
        let res =
            project_point_to_surface(&s, Point3::new(0.5, 0.5, 1.0), TOL).expect("should converge");
        assert!(
            (res.distance - 1.0).abs() < TOL,
            "dist={} expected=1.0",
            res.distance
        );
        assert!((res.u - 0.5).abs() < TOL, "u={}", res.u);
        assert!((res.v - 0.5).abs() < TOL, "v={}", res.v);
    }

    // -- Seeded surface projection ----------------------------------------

    /// A bi-quadratic dome over x,y in [0,1] peaking above the centre. Curved
    /// in both directions, so where Newton starts genuinely decides where it
    /// finishes.
    fn dome_patch() -> NurbsSurface {
        let row = |x: f64, mid_z: f64| {
            vec![
                Point3::new(x, 0.0, 0.0),
                Point3::new(x, 0.5, mid_z),
                Point3::new(x, 1.0, 0.0),
            ]
        };
        NurbsSurface::new(
            2,
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![row(0.0, 0.0), row(0.5, 2.0), row(1.0, 0.0)],
            vec![vec![1.0; 3]; 3],
        )
        .expect("valid dome patch")
    }

    #[test]
    fn a_supplied_grid_gives_bit_identical_results() {
        // The whole basis for using this on the boolean path: it is a speed
        // change, not a behaviour one. Same grid, same nearest node, same
        // Newton start — so every field must match exactly, not approximately.
        let s = dome_patch();
        let grid = SurfaceSeedGrid::for_surface(&s);
        for i in 0..=6 {
            for j in 0..=6 {
                for h in [-2.0, 0.0, 0.7, 40.0] {
                    let p = Point3::new(f64::from(i) / 3.0 - 0.5, f64::from(j) / 3.0 - 0.5, h);
                    let want = project_point_to_surface(&s, p, TOL);
                    let got = project_point_to_surface_with_grid(&s, p, TOL, &grid);
                    assert_eq!(
                        want.is_ok(),
                        got.is_ok(),
                        "one path converged and the other did not at {p:?}"
                    );
                    if let (Ok(w), Ok(g)) = (want, got) {
                        assert_eq!(w.u.to_bits(), g.u.to_bits(), "u at {p:?}");
                        assert_eq!(w.v.to_bits(), g.v.to_bits(), "v at {p:?}");
                        assert_eq!(
                            w.distance.to_bits(),
                            g.distance.to_bits(),
                            "distance at {p:?}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn a_grid_holds_every_node_of_the_coarse_search() {
        // Same node count and the same scan order as `surface_coarse_search`,
        // which is what makes the nearest-node tie-break identical.
        let s = dome_patch();
        let grid = SurfaceSeedGrid::for_surface(&s);
        assert_eq!(grid.nodes.len(), (SURFACE_GRID_SIZE + 1).pow(2));
        for (idx, &(u, v, pt)) in grid.nodes.iter().enumerate() {
            let (i, j) = (idx / (SURFACE_GRID_SIZE + 1), idx % (SURFACE_GRID_SIZE + 1));
            #[allow(clippy::cast_precision_loss)]
            let (want_u, want_v) = (
                i as f64 / SURFACE_GRID_SIZE as f64,
                j as f64 / SURFACE_GRID_SIZE as f64,
            );
            assert!((u - want_u).abs() < 1e-15 && (v - want_v).abs() < 1e-15);
            assert_eq!(pt.x().to_bits(), s.evaluate(u, v).x().to_bits());
        }
    }

    #[test]
    fn seeded_agrees_with_the_grid_search_from_a_good_seed() {
        let s = dome_patch();
        for &(u, v) in &[(0.2, 0.3), (0.5, 0.5), (0.85, 0.15)] {
            let on_surface = s.evaluate(u, v);
            let want = project_point_to_surface(&s, on_surface, TOL).expect("should converge");
            // Seeded from a neighbour, as a caller walking a curve would.
            let got = project_point_to_surface_seeded(&s, on_surface, TOL, (u - 0.05, v + 0.05))
                .expect("should converge");
            assert!((got.u - want.u).abs() < 1e-6, "u={} want={}", got.u, want.u);
            assert!((got.v - want.v).abs() < 1e-6, "v={} want={}", got.v, want.v);
        }
    }

    #[test]
    fn seeded_recovers_the_parameters_of_a_point_on_the_surface() {
        // The property a chaining caller leans on: a point taken off the
        // surface comes back with a residual at rounding level, and with the
        // parameters it was taken from.
        let s = dome_patch();
        let res = project_point_to_surface_seeded(&s, s.evaluate(0.42, 0.61), TOL, (0.40, 0.60))
            .expect("should converge");
        assert!(res.distance < TOL, "dist={}", res.distance);
        assert!((res.u - 0.42).abs() < 1e-5, "u={}", res.u);
        assert!((res.v - 0.61).abs() < 1e-5, "v={}", res.v);
    }

    #[test]
    fn seeded_clamps_a_seed_from_outside_the_domain() {
        // The refiner takes its starting iterate unclamped and several exits
        // return it verbatim, so an out-of-domain seed must not reach it: the
        // reported (u, v) would sit outside the domain while `distance`
        // described the clamped evaluation somewhere else.
        let s = dome_patch();
        let p = Point3::new(0.5, 0.5, 5.0);
        let res =
            project_point_to_surface_seeded(&s, p, TOL, (-7.0, 9.0)).expect("should converge");
        assert!(
            (0.0..=1.0).contains(&res.u),
            "u={} escaped the domain",
            res.u
        );
        assert!(
            (0.0..=1.0).contains(&res.v),
            "v={} escaped the domain",
            res.v
        );
        let reported = (s.evaluate(res.u, res.v) - p).length();
        assert!(
            (reported - res.distance).abs() < TOL,
            "distance {} disagrees with the surface at the returned parameters ({reported})",
            res.distance
        );
    }

    #[test]
    fn seeded_refuses_a_non_finite_seed() {
        // Unlike the curve refiner, the surface one has no NaN guard.
        let s = dome_patch();
        let p = Point3::new(0.5, 0.5, 5.0);
        for seed in [(f64::NAN, 0.5), (0.5, f64::INFINITY), (f64::NAN, f64::NAN)] {
            assert!(
                matches!(
                    project_point_to_surface_seeded(&s, p, TOL, seed),
                    Err(MathError::ParameterOutOfRange { .. })
                ),
                "seed {seed:?} must be refused"
            );
        }
    }

    #[test]
    fn distance_always_describes_the_returned_parameters() {
        // What makes the residual a sound accept test for callers: however
        // the refiner exited, `distance` is the surface at (u, v) against the
        // query point. Swept across seeds so the odd exits get hit too.
        let s = dome_patch();
        let p = Point3::new(0.3, 0.8, 1.5);
        for i in 0..=4 {
            for j in 0..=4 {
                let seed = (f64::from(i) / 4.0, f64::from(j) / 4.0);
                if let Ok(res) = project_point_to_surface_seeded(&s, p, TOL, seed) {
                    let reported = (s.evaluate(res.u, res.v) - p).length();
                    assert!(
                        (reported - res.distance).abs() < 1e-12,
                        "seed {seed:?}: distance {} but surface at ({}, {}) is {reported} away",
                        res.distance,
                        res.u,
                        res.v
                    );
                }
            }
        }
    }

    #[test]
    fn a_rejected_seed_costs_only_the_seeded_budget() {
        // A seed that never converges must not burn the full iteration count
        // before saying so — the caller is about to run the grid search too.
        let s = apex_patch();
        let p = Point3::new(80.0, 60.0, -40.0);
        match project_point_to_surface_seeded(&s, p, 1e-15, (0.5, 0.0)) {
            Err(e) => assert!(
                matches!(
                    e,
                    MathError::ConvergenceFailure { iterations }
                        if iterations == SEEDED_MAX_ITERATIONS
                ),
                "expected to give up after the seeded budget, got {e:?}"
            ),
            // Converging is fine too; what must not happen is a silent
            // disagreement between the parameters and the residual.
            Ok(res) => {
                let reported = (s.evaluate(res.u, res.v) - p).length();
                assert!(
                    (reported - res.distance).abs() < 1e-9,
                    "dist={}",
                    res.distance
                );
            }
        }
    }

    #[test]
    fn the_grid_search_path_is_unchanged_by_the_refactor() {
        // project_point_to_surface now delegates through the seeded plumbing.
        // Its seed is a grid point — always finite, always in domain — so the
        // clamp and the finite check are no-ops and the solve is the one that
        // was there before. Callers elsewhere read `distance` as a correctness
        // oracle, so this must stay true.
        let s = dome_patch();
        for &(x, y, z) in &[(0.5, 0.5, 3.0), (0.1, 0.9, -2.0), (1.4, -0.3, 0.7)] {
            let p = Point3::new(x, y, z);
            let res = project_point_to_surface(&s, p, TOL).expect("should converge");
            let reported = (s.evaluate(res.u, res.v) - p).length();
            assert!(
                (reported - res.distance).abs() < 1e-12,
                "dist={}",
                res.distance
            );
            assert!((0.0..=1.0).contains(&res.u) && (0.0..=1.0).contains(&res.v));
        }
    }

    /// Bilinear degenerate "cone apex" patch.
    ///
    /// Control grid:
    ///   v=0 row: apex=(0,0,0)  apex=(0,0,0)   ← S_u = 0 everywhere on this row
    ///   v=1 row: (-1,0,1)      (1,0,1)
    ///
    /// Parametric formula: S(u,v) = (v·(2u-1), 0, v)
    ///
    /// The Jacobian is rank-1 at v=0 (both S_u and S_v are degenerate there),
    /// which triggers the LM-regularisation branch in `project_point_to_surface`.
    fn apex_patch() -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)], // v=0: apex
                vec![Point3::new(-1.0, 0.0, 1.0), Point3::new(1.0, 0.0, 1.0)], // v=1: base
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .expect("valid apex patch")
    }

    /// Project a point whose nearest surface location is the degenerate apex.
    ///
    /// The surface S(u,v)=(v(2u−1), 0, v) lies in the xz-plane.  The query
    /// point (0, 1, 0) is displaced only in y, so its nearest surface point is
    /// the apex (0,0,0) — the only point that minimises the xz-distance.
    /// Without LM regularisation the Newton step blows up at v→0; with it the
    /// solver should converge and return (u≈0.5, v≈0, dist≈1).
    #[test]
    fn project_to_apex_singularity() {
        let s = apex_patch();
        let res = project_point_to_surface(&s, Point3::new(0.0, 1.0, 0.0), 1e-6)
            .expect("should converge at cone apex singularity");
        // Nearest point must be the apex.
        assert!(
            res.point.x().abs() < 1e-6 && res.point.y().abs() < 1e-6 && res.point.z().abs() < 1e-6,
            "nearest point should be apex, got ({:.4},{:.4},{:.4})",
            res.point.x(),
            res.point.y(),
            res.point.z()
        );
        assert!(
            (res.distance - 1.0).abs() < 1e-6,
            "distance to apex should be 1.0, got {:.8}",
            res.distance
        );
    }

    /// Project a point off-axis but close to the apex.  The solver must still
    /// converge despite starting near the singularity.
    #[test]
    fn project_near_apex_off_axis() {
        let s = apex_patch();
        // S(0.7, 0.05) = (0.05*(2*0.7-1), 0, 0.05) = (0.05*0.4, 0, 0.05) = (0.02, 0, 0.05)
        // Query close to that surface point but displaced in y.
        let res = project_point_to_surface(&s, Point3::new(0.02, 0.3, 0.05), 1e-6)
            .expect("should converge near apex");
        assert!(
            (res.distance - 0.3).abs() < 0.02,
            "expected distance ≈ 0.3, got {:.6}",
            res.distance
        );
        // Nearest surface point should be close to S(0.7, 0.05) = (0.02, 0, 0.05).
        assert!(
            (res.point.z() - 0.05).abs() < 0.02,
            "nearest point z should be ≈ 0.05, got z={:.4}",
            res.point.z()
        );
    }
}
