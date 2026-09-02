//! PCurve computation: project 3D edge curves into a face's (u,v) parameter space.
//!
//! For plane faces, uses [`PlaneFrame`] to project 3D lines to 2D lines.
//! For analytic surfaces (cylinder, cone, sphere, torus), samples points along
//! the 3D curve, projects via `surface.project_point()`, unwraps periodicity,
//! and fits a [`NurbsCurve2D`] (or a `Line2D` if collinear in UV).

use std::f64::consts::TAU;

use remus_math::curves2d::{Curve2D, Line2D, NurbsCurve2D};
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

use super::plane_frame::PlaneFrame;
use crate::error::AlgoError;

/// Number of sample points for pcurve fitting on non-plane surfaces.
const PCURVE_SAMPLES: usize = 16;

/// Compute the 2D pcurve for a 3D edge on a given surface.
///
/// For plane faces, uses `PlaneFrame` to project the 3D line endpoints
/// into (u,v) space and constructs a `Line2D`. For analytic surfaces,
/// samples points along the 3D edge, projects to UV, unwraps periodicity,
/// and fits a `NurbsCurve2D` (or `Line2D` if the UV curve is straight).
///
/// `wire_pts` is needed for plane faces to establish the `PlaneFrame` origin.
///
/// Returns a `Curve2D` parameterized on \[0, 1\] from start to end.
///
/// # Errors
///
/// Returns [`AlgoError::PcurveProjectionFailed`] when any sampled 3D point
/// cannot be projected to a finite UV coordinate.
pub fn compute_pcurve_on_surface(
    curve_3d: &EdgeCurve,
    start: Point3,
    end: Point3,
    surface: &FaceSurface,
    wire_pts: &[Point3],
    frame: Option<&PlaneFrame>,
) -> Result<Curve2D, AlgoError> {
    let domain = reconstruct_structural_sampling_domain(curve_3d, start, end);
    compute_pcurve_on_surface_in_domain(curve_3d, start, end, domain, surface, wire_pts, frame)
}

/// Reconstruct the face-splitter's historical structural sampling interval.
///
/// Open circles and ellipses use the signed shorter arc. Closed curves use
/// their intrinsic range, and other carriers use the named compatibility
/// reconstruction adapter. Exact topology/result authority is carried and
/// evaluated separately.
pub(super) fn reconstruct_structural_sampling_domain(
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
) -> (f64, f64) {
    match curve {
        EdgeCurve::Line => (0.0, 1.0),
        EdgeCurve::Circle(circle) if (start - end).length() > 1e-12 => {
            let t0 = circle.project(start);
            (t0, t0 + shorter_arc_delta(circle.project(end) - t0))
        }
        EdgeCurve::Ellipse(ellipse) if (start - end).length() > 1e-12 => {
            let t0 = ellipse.project(start);
            (t0, t0 + shorter_arc_delta(ellipse.project(end) - t0))
        }
        _ => curve.reconstruct_domain_from_endpoints(start, end),
    }
}

/// Compute a pcurve over an explicit authoritative 3D-curve parameter range.
///
/// Internal topology and face-splitter callers use this entry point.
/// [`compute_pcurve_on_surface`] remains the named raw-construction adapter
/// for callers that do not yet carry a range.
pub(super) fn compute_pcurve_on_surface_in_domain(
    curve_3d: &EdgeCurve,
    start: Point3,
    end: Point3,
    domain: (f64, f64),
    surface: &FaceSurface,
    wire_pts: &[Point3],
    frame: Option<&PlaneFrame>,
) -> Result<Curve2D, AlgoError> {
    if let FaceSurface::Plane { normal, .. } = surface {
        // For straight edges on planes, the pcurve is a Line2D.
        // For curved edges (Circle, Ellipse, NurbsCurve), fall through to the
        // sampling-based path to produce a proper curved pcurve.
        if matches!(curve_3d, EdgeCurve::Line) {
            let owned;
            let frame = if let Some(f) = frame {
                f
            } else {
                owned = PlaneFrame::from_plane_face(*normal, wire_pts);
                &owned
            };
            let p0 = finite_uv(frame.project(start), surface, "plane_line_start")?;
            let p1 = finite_uv(frame.project(end), surface, "plane_line_end")?;
            let dir = Vec2::new(p1.x() - p0.x(), p1.y() - p0.y());
            return Ok(Curve2D::Line(make_line2d_safe(p0, dir)));
        }
        // Curved edge on plane: sample and project via PlaneFrame below.
    }

    // For plane surfaces with curved edges, use PlaneFrame for projection.
    let uv_pts = if let FaceSurface::Plane { normal, .. } = surface {
        let owned;
        let f = if let Some(fr) = frame {
            fr
        } else {
            owned = PlaneFrame::from_plane_face(*normal, wire_pts);
            &owned
        };
        sample_edge_to_uv_via_frame(curve_3d, start, end, domain, surface, f)?
    } else {
        sample_edge_to_uv(curve_3d, start, end, domain, surface)?
    };
    if uv_pts.len() < 2 {
        return Err(projection_failure(surface, "insufficient_curve_samples"));
    }

    // Check collinearity -- if all points are (nearly) on a line in UV,
    // use a Line2D instead of a NURBS fit.
    // EXCEPTION: Line2D uses arc-length parameterization (evaluate(t) =
    // origin + unit_dir * t), not [0,1] mapping. For curves that need
    // evaluate(0)→start and evaluate(1)→end, use NURBS interpolation
    // instead, which naturally maps [0,1] to the full extent.
    if is_collinear_2d(&uv_pts, 1e-6) {
        let p0 = uv_pts[0];
        let pn = uv_pts[uv_pts.len() - 1];
        let dx = pn.x() - p0.x();
        let dy = pn.y() - p0.y();
        let len_sq = dx * dx + dy * dy;
        // For non-degenerate lines (p0 ≠ pn), use Line2D.
        // For closed collinear curves (p0 ≈ pn), fall through to NURBS
        // fit to preserve [0,1] parameterization.
        if len_sq >= 1e-12 {
            let dir = Vec2::new(dx, dy);
            return Ok(Curve2D::Line(make_line2d_safe(p0, dir)));
        }
        // p0 ≈ pn: fall through to NURBS interpolation.
    }

    // Fit a NURBS curve through the UV sample points.
    Ok(fit_nurbs2d_through_points(&uv_pts))
}

/// Project a 3D point onto a surface's parameter space.
///
/// For planes, uses `PlaneFrame`. Analytic surfaces use their closed-form
/// inverse. NURBS surfaces use the fallible Newton projector directly so its
/// convergence error cannot be replaced by the trait's compatibility
/// midpoint.
///
/// # Errors
///
/// Returns [`AlgoError::PcurveProjectionFailed`] when the point or projected
/// UV coordinate is non-finite, or when NURBS projection does not converge.
pub fn project_point_on_surface(
    p: Point3,
    surface: &FaceSurface,
    wire_pts: &[Point3],
    frame: Option<&PlaneFrame>,
) -> Result<Point2, AlgoError> {
    if let FaceSurface::Plane { normal, .. } = surface {
        let owned;
        let frame = if let Some(f) = frame {
            f
        } else {
            owned = PlaneFrame::from_plane_face(*normal, wire_pts);
            &owned
        };
        let projected = frame.project(p);
        return finite_uv(projected, surface, "plane_frame");
    }
    project_native_point_on_surface(p, surface, "point_projection")
}

/// Build a `PlaneFrame` for a plane face.
#[allow(dead_code)]
pub fn plane_frame_for_face(normal: Vec3, wire_pts: &[Point3]) -> PlaneFrame {
    PlaneFrame::from_plane_face(normal, wire_pts)
}

/// Create a `Line2D` safely, handling degenerate (zero-length) directions.
pub(super) fn make_line2d_safe(origin: Point2, dir: Vec2) -> Line2D {
    Line2D::new(origin, dir).unwrap_or_else(|_| {
        // Degenerate edge -- fallback to x-axis direction.
        // Safety: (1, 0) is non-zero, so Line2D::new cannot fail.
        #[allow(clippy::unwrap_used)]
        Line2D::new(origin, Vec2::new(1.0, 0.0)).unwrap()
    })
}

/// Unwrap periodic UV parameters (public wrapper for use by other modules).
pub(super) fn unwrap_periodic_params_pub(
    pts: &mut [Point2],
    u_period: Option<f64>,
    v_period: Option<f64>,
) {
    unwrap_periodic_params(pts, u_period, v_period);
}

/// Returns `(u_period, v_period)` for a surface -- `Some(TAU)` if periodic.
pub(super) fn surface_periods(surface: &FaceSurface) -> (Option<f64>, Option<f64>) {
    match surface {
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => (None, None),
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) => (Some(TAU), None),
        FaceSurface::Sphere(_) => (Some(TAU), None),
        FaceSurface::Torus(_) => (Some(TAU), Some(TAU)),
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Sample points along a 3D edge curve and project each to surface UV.
///
/// Returns UV points with periodicity unwrapped.
pub(super) fn sample_edge_to_uv(
    curve_3d: &EdgeCurve,
    start: Point3,
    end: Point3,
    domain: (f64, f64),
    surface: &FaceSurface,
) -> Result<Vec<Point2>, AlgoError> {
    let n = PCURVE_SAMPLES;
    let mut pts_3d = Vec::with_capacity(n + 1);
    for i in 0..=n {
        #[allow(clippy::cast_precision_loss)]
        let t = i as f64 / n as f64;
        let p = evaluate_edge_at_t(curve_3d, start, end, domain, t);
        pts_3d.push(p);
    }

    let mut uv_pts = Vec::with_capacity(pts_3d.len());
    for p in pts_3d {
        uv_pts.push(project_native_point_on_surface(p, surface, "curve_sample")?);
    }

    // Unwrap periodicity.
    let (u_period, v_period) = surface_periods(surface);
    unwrap_periodic_params(&mut uv_pts, u_period, v_period);

    Ok(uv_pts)
}

fn project_native_point_on_surface(
    point: Point3,
    surface: &FaceSurface,
    stage: &'static str,
) -> Result<Point2, AlgoError> {
    if !point.x().is_finite() || !point.y().is_finite() || !point.z().is_finite() {
        return Err(projection_failure(surface, stage));
    }

    let projected = match surface {
        FaceSurface::Plane { .. } => return Err(projection_failure(surface, stage)),
        FaceSurface::Nurbs(nurbs) => {
            let projection =
                remus_math::nurbs::projection::project_point_to_surface(nurbs, point, 1e-7)
                    .map_err(|_| projection_failure(surface, stage))?;
            Point2::new(projection.u, projection.v)
        }
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => {
            let (u, v) = surface
                .project_point(point)
                .ok_or_else(|| projection_failure(surface, stage))?;
            Point2::new(u, v)
        }
    };
    finite_uv(projected, surface, stage)
}

fn finite_uv(
    point: Point2,
    surface: &FaceSurface,
    stage: &'static str,
) -> Result<Point2, AlgoError> {
    if point.x().is_finite() && point.y().is_finite() {
        Ok(point)
    } else {
        Err(projection_failure(surface, stage))
    }
}

const fn projection_failure(surface: &FaceSurface, stage: &'static str) -> AlgoError {
    AlgoError::PcurveProjectionFailed {
        surface: surface.type_tag(),
        stage,
    }
}

/// Evaluate a 3D edge curve at parameter t in [0, 1].
///
/// For `Line`, linearly interpolates between start and end.
/// Non-Line curves are evaluated over the supplied authoritative domain.
pub(super) fn evaluate_edge_at_t(
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    domain: (f64, f64),
    t: f64,
) -> Point3 {
    if matches!(curve, EdgeCurve::Line) {
        Point3::new(
            start.x() + (end.x() - start.x()) * t,
            start.y() + (end.y() - start.y()) * t,
            start.z() + (end.z() - start.z()) * t,
        )
    } else {
        let (t0, t1) = domain;
        let param = t0 + (t1 - t0) * t;
        curve.evaluate_with_endpoints(param, start, end)
    }
}

/// Uniformly sample an edge curve at `n` points (`t = k/n`, `k` in `0..n`),
/// honouring the wire traversal direction (`forward = false` samples
/// `1 - k/n`).
///
/// Matches [`evaluate_edge_at_t`]'s conventions per curve arm while hoisting
/// the interval setup out of the sample loop.
pub(super) fn sample_edge_uniform(
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    domain: (f64, f64),
    n: usize,
    forward: bool,
    out: &mut Vec<Point3>,
) {
    #[allow(clippy::cast_precision_loss)]
    let frac_at = |k: usize| -> f64 {
        let f = k as f64 / n as f64;
        if forward { f } else { 1.0 - f }
    };
    if matches!(curve, EdgeCurve::Line) {
        let d = end - start;
        for k in 0..n {
            let t = frac_at(k);
            out.push(start + d * t);
        }
    } else {
        let (t0, t1) = domain;
        let span = t1 - t0;
        for k in 0..n {
            out.push(curve.evaluate_with_endpoints(frac_at(k).mul_add(span, t0), start, end));
        }
    }
}

/// Wrap an angular difference into (-pi, pi] — the shorter way around.
pub(super) fn shorter_arc_delta(d: f64) -> f64 {
    let w = d.rem_euclid(TAU);
    if w > std::f64::consts::PI { w - TAU } else { w }
}

/// Sample points along a 3D edge curve and project each to UV via `PlaneFrame`.
///
/// Used for curved edges (Circle, Ellipse) on plane surfaces where
/// `surface.project_point()` returns `None`.
fn sample_edge_to_uv_via_frame(
    curve_3d: &EdgeCurve,
    start: Point3,
    end: Point3,
    domain: (f64, f64),
    surface: &FaceSurface,
    frame: &PlaneFrame,
) -> Result<Vec<Point2>, AlgoError> {
    let n = PCURVE_SAMPLES;
    let mut uv_pts = Vec::with_capacity(n + 1);
    for i in 0..=n {
        #[allow(clippy::cast_precision_loss)]
        let t = i as f64 / n as f64;
        let p = evaluate_edge_at_t(curve_3d, start, end, domain, t);
        uv_pts.push(finite_uv(frame.project(p), surface, "curve_sample")?);
    }
    Ok(uv_pts)
}

/// Unwrap periodic UV parameters to remove seam jumps.
///
/// Shifts each point by the whole number of periods that brings it to the
/// copy nearest its predecessor, maintaining continuity across any copy
/// distance (a mixed-copy wire can jump multiple periods at once).
fn unwrap_periodic_params(pts: &mut [Point2], u_period: Option<f64>, v_period: Option<f64>) {
    if pts.len() < 2 {
        return;
    }

    // Shift each point to the period copy NEAREST its predecessor. A single
    // ±period step (the old form) cannot recover a multi-period jump, which
    // arises when a wire mixes period copies (an edge stored with a negative-u
    // endpoint following edges unwrapped upward) — the walk then folds and a
    // valid band loop measures zero area. Rounding the jump handles any copy
    // distance and is identical to the single step for ordinary seam jumps.
    for i in 1..pts.len() {
        if let Some(period) = u_period {
            let du = pts[i].x() - pts[i - 1].x();
            let k = (du / period).round();
            if k != 0.0 {
                pts[i] = Point2::new(period.mul_add(-k, pts[i].x()), pts[i].y());
            }
        }
        if let Some(period) = v_period {
            let dv = pts[i].y() - pts[i - 1].y();
            let k = (dv / period).round();
            if k != 0.0 {
                pts[i] = Point2::new(pts[i].x(), period.mul_add(-k, pts[i].y()));
            }
        }
    }
}

/// Check if a sequence of 2D points is approximately collinear.
fn is_collinear_2d(pts: &[Point2], tol: f64) -> bool {
    if pts.len() < 3 {
        return true;
    }
    let p0 = pts[0];
    let pn = pts[pts.len() - 1];
    let dx = pn.x() - p0.x();
    let dy = pn.y() - p0.y();
    let len_sq = dx * dx + dy * dy;
    if len_sq < tol * tol {
        // p0 ≈ pn — either all points are clustered (degenerate) or this
        // is a closed curve (circle). Check if intermediate points lie on
        // a LINE (collinear in UV, e.g. circle on cylinder at constant v)
        // or spread in 2D (actual closed loop, e.g. circle on plane).
        //
        // Use the line from p0 to the farthest intermediate point as the
        // collinearity reference. If all points are near that line, collinear.
        let mut farthest_idx = 1;
        let mut farthest_dist_sq = 0.0_f64;
        for (i, p) in pts[1..pts.len() - 1].iter().enumerate() {
            let ex = p.x() - p0.x();
            let ey = p.y() - p0.y();
            let d2 = ex * ex + ey * ey;
            if d2 > farthest_dist_sq {
                farthest_dist_sq = d2;
                farthest_idx = i + 1;
            }
        }
        if farthest_dist_sq < tol * tol {
            return true; // All points clustered — degenerate.
        }
        // Check collinearity against line p0→farthest.
        let pf = pts[farthest_idx];
        let fdx = pf.x() - p0.x();
        let fdy = pf.y() - p0.y();
        let flen = farthest_dist_sq.sqrt();
        let inv_flen = 1.0 / flen;
        for (i, p) in pts.iter().enumerate() {
            if i == 0 || i == farthest_idx {
                continue;
            }
            let ex = p.x() - p0.x();
            let ey = p.y() - p0.y();
            let dist = (ex * fdy - ey * fdx).abs() * inv_flen;
            if dist > tol {
                return false; // 2D spread — closed loop.
            }
        }
        return true; // All on a line — collinear (e.g. cylinder UV).
    }
    let inv_len = 1.0 / len_sq.sqrt();
    for p in &pts[1..pts.len() - 1] {
        let ex = p.x() - p0.x();
        let ey = p.y() - p0.y();
        // Perpendicular distance from p to line(p0, pn).
        let dist = (ex * dy - ey * dx).abs() * inv_len;
        if dist > tol {
            return false;
        }
    }
    true
}

/// Fit a `NurbsCurve2D` through 2D sample points via NURBS interpolation.
///
/// Lifts 2D points to 3D (z=0), uses the math crate's `interpolate`, then
/// extracts 2D control points from the result.
fn fit_nurbs2d_through_points(pts: &[Point2]) -> Curve2D {
    let fallback = || -> Curve2D {
        let p0 = pts[0];
        let p1 = pts[pts.len() - 1];
        let dir = Vec2::new(p1.x() - p0.x(), p1.y() - p0.y());
        Curve2D::Line(make_line2d_safe(p0, dir))
    };

    let pts_3d: Vec<Point3> = pts.iter().map(|p| Point3::new(p.x(), p.y(), 0.0)).collect();
    let degree = 3.min(pts_3d.len() - 1);
    let Ok(nurbs_3d) = remus_math::nurbs::fitting::interpolate(&pts_3d, degree) else {
        return fallback();
    };

    let cp_2d: Vec<Point2> = nurbs_3d
        .control_points()
        .iter()
        .map(|p| Point2::new(p.x(), p.y()))
        .collect();
    let weights = nurbs_3d.weights().to_vec();
    let knots = nurbs_3d.knots().to_vec();
    NurbsCurve2D::new(nurbs_3d.degree(), knots, cp_2d, weights)
        .map_or_else(|_| fallback(), Curve2D::Nurbs)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_math::nurbs::surface::NurbsSurface;
    use remus_math::traits::ParametricCurve;
    use remus_math::vec::Vec3;

    fn flat_nurbs_surface() -> FaceSurface {
        FaceSurface::Nurbs(
            NurbsSurface::new(
                1,
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![
                    vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                    vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
                ],
                vec![vec![1.0, 1.0], vec![1.0, 1.0]],
            )
            .expect("valid bilinear patch"),
        )
    }

    #[test]
    fn nurbs_projection_refuses_nonfinite_input_instead_of_using_midpoint() {
        let surface = flat_nurbs_surface();
        let error = project_point_on_surface(Point3::new(f64::NAN, 0.5, 0.0), &surface, &[], None)
            .expect_err("non-finite geometry must not receive a synthetic UV");

        assert!(matches!(
            error,
            AlgoError::PcurveProjectionFailed {
                surface: "nurbs",
                stage: "point_projection"
            }
        ));
    }

    #[test]
    fn public_pcurve_builder_propagates_sample_projection_failure() {
        let surface = flat_nurbs_surface();
        let error = compute_pcurve_on_surface(
            &EdgeCurve::Line,
            Point3::new(0.0, 0.5, 0.0),
            Point3::new(f64::INFINITY, 0.5, 0.0),
            &surface,
            &[],
            None,
        )
        .expect_err("a failed curve sample must abort pcurve construction");

        assert!(matches!(
            error,
            AlgoError::PcurveProjectionFailed {
                surface: "nurbs",
                stage: "curve_sample"
            }
        ));
    }

    #[test]
    fn plane_line_projection_refuses_nonfinite_endpoint() {
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let error = compute_pcurve_on_surface(
            &EdgeCurve::Line,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(f64::INFINITY, 1.0, 0.0),
            &surface,
            &[],
            None,
        )
        .expect_err("a plane pcurve must not retain a non-finite UV endpoint");

        assert!(matches!(
            error,
            AlgoError::PcurveProjectionFailed {
                surface: "plane",
                stage: "plane_line_end"
            }
        ));
    }

    #[test]
    fn finite_nurbs_projection_uses_the_actual_surface_foot() {
        let surface = flat_nurbs_surface();
        let uv = project_point_on_surface(Point3::new(0.25, 0.75, 2.0), &surface, &[], None)
            .expect("flat NURBS projection converges");

        assert!((uv.x() - 0.25).abs() < 1e-7);
        assert!((uv.y() - 0.75).abs() < 1e-7);
    }

    #[test]
    fn line_on_xy_plane_produces_line2d_with_roundtrip() {
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let start = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(3.0, 4.0, 0.0);
        let wire_pts = vec![
            start,
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ];

        let pcurve =
            compute_pcurve_on_surface(&EdgeCurve::Line, start, end, &surface, &wire_pts, None)
                .expect("plane line pcurve");

        let frame = PlaneFrame::from_plane_face(Vec3::new(0.0, 0.0, 1.0), &wire_pts);
        let expected_start = frame.project(start);
        let expected_end = frame.project(end);
        let len = ((expected_end.x() - expected_start.x()).powi(2)
            + (expected_end.y() - expected_start.y()).powi(2))
        .sqrt();

        let p_start = pcurve.evaluate(0.0);
        let p_end = pcurve.evaluate(len);

        assert!((p_start.x() - expected_start.x()).abs() < 1e-10);
        assert!((p_start.y() - expected_start.y()).abs() < 1e-10);
        assert!((p_end.x() - expected_end.x()).abs() < 1e-10);
        assert!((p_end.y() - expected_end.y()).abs() < 1e-10);
    }

    #[test]
    fn line_on_tilted_plane_roundtrips() {
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let surface = FaceSurface::Plane { normal, d: 5.0 };
        let start = Point3::new(1.0, 2.0, 5.0);
        let end = Point3::new(4.0, 6.0, 5.0);
        let wire_pts = vec![
            start,
            Point3::new(10.0, 0.0, 5.0),
            Point3::new(0.0, 10.0, 5.0),
        ];

        let pcurve =
            compute_pcurve_on_surface(&EdgeCurve::Line, start, end, &surface, &wire_pts, None)
                .expect("tilted plane line pcurve");

        let frame = PlaneFrame::from_plane_face(normal, &wire_pts);
        let expected_start = frame.project(start);
        let expected_end = frame.project(end);
        let len = ((expected_end.x() - expected_start.x()).powi(2)
            + (expected_end.y() - expected_start.y()).powi(2))
        .sqrt();

        let p0 = pcurve.evaluate(0.0);
        let p1 = pcurve.evaluate(len);

        let start_back = frame.evaluate(p0.x(), p0.y());
        let end_back = frame.evaluate(p1.x(), p1.y());
        assert!((start_back - start).length() < 1e-10);
        assert!((end_back - end).length() < 1e-10);
    }

    #[test]
    fn project_point_on_xy_plane() {
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let wire_pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ];
        let p = Point3::new(5.0, 3.0, 0.0);
        let uv = project_point_on_surface(p, &surface, &wire_pts, None)
            .expect("point projects to plane");

        let frame = PlaneFrame::from_plane_face(Vec3::new(0.0, 0.0, 1.0), &wire_pts);
        let back = frame.evaluate(uv.x(), uv.y());
        assert!((back - p).length() < 1e-10);
    }

    #[test]
    fn unwrap_periodic_removes_seam_jump() {
        let mut pts = vec![
            Point2::new(6.0, 0.0),
            Point2::new(6.2, 0.0),
            Point2::new(0.1, 0.0), // Jump from ~6.2 to ~0.1 (crossed 2pi)
            Point2::new(0.3, 0.0),
        ];
        unwrap_periodic_params(&mut pts, Some(TAU), None);
        assert!(
            (pts[2].x() - (0.1 + TAU)).abs() < 0.01,
            "got {}",
            pts[2].x()
        );
    }

    #[test]
    fn collinearity_detection() {
        let line_pts = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0),
            Point2::new(3.0, 3.0),
        ];
        assert!(is_collinear_2d(&line_pts, 1e-6));

        let curve_pts = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 0.0),
            Point2::new(3.0, 3.0),
        ];
        assert!(!is_collinear_2d(&curve_pts, 1e-6));
    }

    #[test]
    fn pcurve_line_on_cylinder_is_vertical() {
        let cyl = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        )
        .unwrap();
        let surface = FaceSurface::Cylinder(cyl);
        let start = Point3::new(1.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 5.0);
        let pcurve = compute_pcurve_on_surface(&EdgeCurve::Line, start, end, &surface, &[], None)
            .expect("cylinder ruling pcurve");

        assert!(
            matches!(pcurve, Curve2D::Line(_)),
            "expected Line, got {:?}",
            pcurve
        );
    }

    #[test]
    fn pcurve_circle_on_cylinder_is_horizontal() {
        let cyl = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        )
        .unwrap();
        let surface = FaceSurface::Cylinder(cyl);
        let circle = remus_math::curves::Circle3D::new(
            Point3::new(0.0, 0.0, 3.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        )
        .unwrap();
        let start = Point3::new(1.0, 0.0, 3.0);
        let end = Point3::new(-1.0, 0.0, 3.0);
        let curve_3d = EdgeCurve::Circle(circle);
        let pcurve = compute_pcurve_on_surface(&curve_3d, start, end, &surface, &[], None)
            .expect("cylinder ring pcurve");

        assert!(
            matches!(pcurve, Curve2D::Line(_)),
            "expected Line for equatorial circle, got {:?}",
            pcurve
        );
    }

    #[test]
    fn surface_periods_correct() {
        let plane = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        assert_eq!(surface_periods(&plane), (None, None));

        let cyl = FaceSurface::Cylinder(
            remus_math::surfaces::CylindricalSurface::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
            )
            .unwrap(),
        );
        assert_eq!(surface_periods(&cyl), (Some(TAU), None));

        let sphere = FaceSurface::Sphere(
            remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap(),
        );
        assert_eq!(surface_periods(&sphere), (Some(TAU), None));

        let torus = FaceSurface::Torus(
            remus_math::surfaces::ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0)
                .unwrap(),
        );
        assert_eq!(surface_periods(&torus), (Some(TAU), Some(TAU)));
    }

    /// The batch sampler must agree with per-sample `evaluate_edge_at_t` on
    /// every curve arm and both traversal directions — it exists only to
    /// hoist the per-arm setup, never to change the sampled points.
    #[test]
    fn sample_edge_uniform_matches_evaluate_edge_at_t() {
        use remus_math::curves::{Circle3D, Ellipse3D};
        use remus_math::nurbs::fitting::interpolate;
        use remus_math::vec::Vec3;
        use remus_topology::edge::EdgeCurve;

        let circle = Circle3D::new_with_ref(
            Point3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.5,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let ellipse = Ellipse3D::new(
            Point3::new(-1.0, 0.5, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            3.0,
            1.5,
        )
        .unwrap();
        let nurbs = {
            let pts: Vec<Point3> = (0..=6)
                .map(|k| {
                    let x = f64::from(k) * 0.5;
                    Point3::new(x, x * x * 0.2, 0.1 * x)
                })
                .collect();
            interpolate(&pts, 3).unwrap()
        };
        let nurbs_mid_a = ParametricCurve::evaluate(&nurbs, 0.2);
        let nurbs_mid_b = ParametricCurve::evaluate(&nurbs, 0.75);

        let cases: Vec<(EdgeCurve, Point3, Point3, (f64, f64))> = vec![
            (
                EdgeCurve::Line,
                Point3::new(0.0, 1.0, 2.0),
                Point3::new(3.0, -1.0, 0.5),
                (0.0, 1.0),
            ),
            (
                EdgeCurve::Circle(circle.clone()),
                circle.evaluate(0.3),
                circle.evaluate(2.1),
                (0.3, 2.1),
            ),
            // Closed circle keeps its non-zero seam anchor.
            (
                EdgeCurve::Circle(circle.clone()),
                circle.evaluate(0.3),
                circle.evaluate(0.3),
                (0.3, 0.3 + TAU),
            ),
            (
                EdgeCurve::Ellipse(ellipse.clone()),
                ellipse.evaluate(1.0),
                ellipse.evaluate(2.4),
                (1.0, 2.4),
            ),
            (
                EdgeCurve::NurbsCurve(nurbs),
                nurbs_mid_a,
                nurbs_mid_b,
                (0.2, 0.75),
            ),
        ];

        for (curve, start, end, domain) in cases {
            for forward in [true, false] {
                let n = 7usize;
                let mut batch = Vec::new();
                sample_edge_uniform(&curve, start, end, domain, n, forward, &mut batch);
                assert_eq!(batch.len(), n);
                for (k, got) in batch.iter().enumerate() {
                    #[allow(clippy::cast_precision_loss)]
                    let frac = k as f64 / n as f64;
                    let frac = if forward { frac } else { 1.0 - frac };
                    let want = evaluate_edge_at_t(&curve, start, end, domain, frac);
                    assert!(
                        (*got - want).length() < 1e-12,
                        "{} fwd={forward} k={k}: {got:?} vs {want:?}",
                        curve.type_tag()
                    );
                }
            }
        }
    }

    #[test]
    fn explicit_wrapped_domain_controls_pcurve_despite_perturbed_endpoints() {
        use remus_math::curves::Circle3D;

        let circle = Circle3D::new_with_ref(
            Point3::new(2.0, -1.0, 4.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let domain = (5.5, TAU + 0.5);
        let exact_start = circle.evaluate(domain.0);
        let exact_end = circle.evaluate(domain.1);
        let perturbed_start = exact_start + Vec3::new(0.0, 0.0, 1e-3);
        let perturbed_end = exact_end + Vec3::new(0.0, 0.0, -1e-3);
        let curve = EdgeCurve::Circle(circle.clone());
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 4.0,
        };
        let wire_pts = [
            exact_start,
            exact_end,
            circle.evaluate(f64::midpoint(domain.0, domain.1)),
        ];
        let frame = PlaneFrame::from_plane_face(Vec3::new(0.0, 0.0, 1.0), &wire_pts);

        let pcurve = compute_pcurve_on_surface_in_domain(
            &curve,
            perturbed_start,
            perturbed_end,
            domain,
            &surface,
            &wire_pts,
            Some(&frame),
        )
        .expect("stored curve domain projects through the plane frame");
        let expected_mid = circle.evaluate(f64::midpoint(domain.0, domain.1));
        let uv_mid = pcurve.evaluate(0.5);
        let actual_mid = frame.evaluate(uv_mid.x(), uv_mid.y());
        let chord_mid = perturbed_start + (perturbed_end - perturbed_start) * 0.5;

        assert!((actual_mid - expected_mid).length() < 1e-7);
        assert!((actual_mid - chord_mid).length() > 0.1);
    }
}
