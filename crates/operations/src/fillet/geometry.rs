//! Edge sampling and surface normal utilities for fillet operations.

use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;

/// Sample a point along an edge curve at normalised parameter `t` ∈ [0, 1].
///
/// For `Line` edges this is simple lerp between start/end.
/// For Circle/Ellipse/NurbsCurve the actual curve geometry is evaluated.
pub(super) fn sample_edge_point(
    curve: &EdgeCurve,
    p_start: Point3,
    p_end: Point3,
    t: f64,
) -> Point3 {
    match curve {
        EdgeCurve::Line => Point3::new(
            p_start.x().mul_add(1.0 - t, p_end.x() * t),
            p_start.y().mul_add(1.0 - t, p_end.y() * t),
            p_start.z().mul_add(1.0 - t, p_end.z() * t),
        ),
        EdgeCurve::Circle(circle) => {
            let ts = circle.project(p_start);
            let mut te = circle.project(p_end);
            if te <= ts {
                te += std::f64::consts::TAU;
            }
            circle.evaluate(ts + (te - ts) * t)
        }
        EdgeCurve::Ellipse(ellipse) => {
            let ts = ellipse.project(p_start);
            let mut te = ellipse.project(p_end);
            if te <= ts {
                te += std::f64::consts::TAU;
            }
            ellipse.evaluate(ts + (te - ts) * t)
        }
        // Unbounded branches: `project` inverts the parameterization
        // exactly and the sub-arc is the straight parameter interval, so
        // there is no periodic wrap to fix up as for circle/ellipse.
        EdgeCurve::Hyperbola(hyp) => {
            let (ts, te) = (hyp.project(p_start), hyp.project(p_end));
            hyp.evaluate((te - ts).mul_add(t, ts))
        }
        EdgeCurve::Parabola(par) => {
            let (ts, te) = (par.project(p_start), par.project(p_end));
            par.evaluate((te - ts).mul_add(t, ts))
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            let (u0, u1) = nurbs.domain();
            nurbs.evaluate(u0 + (u1 - u0) * t)
        }
    }
}

/// Compute the tangent direction along an edge curve at normalised parameter `t`.
///
/// Returns an unnormalised tangent vector. For `Line` edges this is the constant
/// `p_end - p_start` direction.
pub(super) fn sample_edge_tangent(
    curve: &EdgeCurve,
    p_start: Point3,
    p_end: Point3,
    t: f64,
) -> Vec3 {
    match curve {
        EdgeCurve::Line => p_end - p_start,
        EdgeCurve::Circle(circle) => {
            let ts = circle.project(p_start);
            let mut te = circle.project(p_end);
            if te <= ts {
                te += std::f64::consts::TAU;
            }
            circle.tangent(ts + (te - ts) * t)
        }
        EdgeCurve::Ellipse(ellipse) => {
            let ts = ellipse.project(p_start);
            let mut te = ellipse.project(p_end);
            if te <= ts {
                te += std::f64::consts::TAU;
            }
            ellipse.tangent(ts + (te - ts) * t)
        }
        EdgeCurve::Hyperbola(hyp) => {
            let (ts, te) = (hyp.project(p_start), hyp.project(p_end));
            hyp.tangent((te - ts).mul_add(t, ts))
        }
        EdgeCurve::Parabola(par) => {
            let (ts, te) = (par.project(p_start), par.project(p_end));
            par.tangent((te - ts).mul_add(t, ts))
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            let (u0, u1) = nurbs.domain();
            let u = u0 + (u1 - u0) * t;
            let d = nurbs.derivatives(u, 1);
            d[1]
        }
    }
}

/// Determine the number of v-direction samples needed for an edge curve.
///
/// Line edges need only 2 samples (start + end) for an exact linear surface.
/// Curved edges need more samples to capture the curvature.
pub(super) fn edge_v_samples(curve: &EdgeCurve) -> usize {
    match curve {
        EdgeCurve::Line => 2,
        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => 9,
        // Same station count as the periodic conics: a hyperbolic or
        // parabolic spine curves as strongly and needs the same resolution.
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => 9,
        EdgeCurve::NurbsCurve(_) => 7,
    }
}

/// The dihedral at an edge sample point, from the in-face directions normal
/// to the edge tangent that point away from the edge toward the other face's
/// material side.
///
/// `bisector` points into the dihedral; `half_angle` is half the interior
/// angle between the two in-face directions. The variable fillet's section
/// itself comes from the exact rolling ball (`VariableSection`), not from
/// these directions.
pub(super) struct CrossSection {
    pub(super) bisector: Vec3,
    pub(super) half_angle: f64,
}

/// Compute the cross-section directions at a sample point, given the edge
/// tangent and the two faces' outward normals there. Degenerate cross products
/// fall back to the supplied reference directions.
pub(super) fn cross_section_dirs(
    tan: Vec3,
    n1: Vec3,
    n2: Vec3,
    fallback_d1: Vec3,
    fallback_d2: Vec3,
) -> CrossSection {
    let c1 = tan.cross(n1);
    let c2 = tan.cross(n2);
    let ld1 = if c1.dot(n2) < 0.0 { c1 } else { -c1 };
    let ld2 = if c2.dot(n1) < 0.0 { c2 } else { -c2 };
    let ld1 = ld1.normalize().unwrap_or(fallback_d1);
    let ld2 = ld2.normalize().unwrap_or(fallback_d2);
    let cos_half = ld1.dot(ld2).clamp(-1.0, 1.0);
    let half_angle = cos_half.acos() / 2.0;
    let bisector = (ld1 + ld2).normalize().unwrap_or(fallback_d1);
    CrossSection {
        bisector,
        half_angle,
    }
}
