//! Exact axis-aligned bounds of circle and ellipse arcs.
//!
//! Broad-phase boxes built from an edge's endpoints and a few parameter
//! samples under-bound a conic: a full circle (start == end) sampled at its
//! endpoints and midpoint contributes a single chord, and how short the box
//! falls depends on where the seam sits. The per-axis extreme points below,
//! together with the arc's endpoints, bound the arc exactly wherever its seam
//! is.

use remus_math::vec::Point3;
use remus_topology::edge::EdgeCurve;

/// The points of a circle or ellipse arc that are extreme along each world
/// axis, restricted to the arc's parameter window `[t0, t1]`.
///
/// Along axis `k` the conic reads `c_k + a·u_k·cos t + b·v_k·sin t`, extreme
/// at `t* = atan2(b·v_k, a·u_k)` and `t* + π`. Each extremum whose periodic
/// copy lands inside the window is returned; together with the arc's endpoints
/// these bound the arc exactly (to rounding), wherever its seam sits. Other
/// carriers return nothing: lines are bounded by their endpoints, and the
/// remaining curves keep their caller's existing sampled bound.
pub fn conic_arc_axis_extrema(curve: &EdgeCurve, t0: f64, t1: f64) -> Vec<Point3> {
    let (u_axis, v_axis, a, b) = match curve {
        EdgeCurve::Circle(c) => (c.u_axis(), c.v_axis(), c.radius(), c.radius()),
        EdgeCurve::Ellipse(e) => (e.u_axis(), e.v_axis(), e.semi_major(), e.semi_minor()),
        // B24: exhaustive over `EdgeCurve` — only the trigonometric conics
        // have closed-form axis extrema here.
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => return Vec::new(),
    };
    let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
    if !(lo.is_finite() && hi.is_finite()) {
        return Vec::new();
    }
    let tau = std::f64::consts::TAU;
    let mut out = Vec::new();
    for (uk, vk) in [
        (u_axis.x(), v_axis.x()),
        (u_axis.y(), v_axis.y()),
        (u_axis.z(), v_axis.z()),
    ] {
        // Every point pushed lies on the arc, so the bound never widens past
        // it — even for an axis normal to the conic plane (`atan2(0, 0)`).
        let t_star = (b * vk).atan2(a * uk);
        for cand in [t_star, t_star + std::f64::consts::PI] {
            // First periodic copy of `cand` at or after `lo`.
            let t = ((lo - cand) / tau).ceil().mul_add(tau, cand);
            if t <= hi {
                // Circles and ellipses ignore the endpoint arguments.
                let unused = Point3::new(0.0, 0.0, 0.0);
                out.push(curve.evaluate_with_endpoints(t, unused, unused));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::aabb::Aabb3;
    use remus_math::curves::{Circle3D, Ellipse3D};
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::EdgeCurve;

    use super::conic_arc_axis_extrema;

    /// Ellipse arcs: the extrema plus the endpoints bound a dense sampling of
    /// every window, and every returned point lies on the ellipse (the bound
    /// never widens past the arc).
    #[test]
    fn ellipse_arc_extrema_bound_a_dense_sampling() {
        let (a, b) = (3.0, 1.25);
        let center = Point3::new(1.0, 2.0, 3.0);
        let ellipse = Ellipse3D::new_with_ref(
            center,
            Vec3::new(1.0, 2.0, 2.0),
            a,
            b,
            Vec3::new(2.0, -1.0, 0.0),
        )
        .unwrap();
        let curve = EdgeCurve::Ellipse(ellipse.clone());
        for (t0, t1) in [
            (0.2, 1.4),
            (-2.0, 3.5),
            (5.0, 9.0),
            (0.0, std::f64::consts::TAU),
        ] {
            let extrema = conic_arc_axis_extrema(&curve, t0, t1);
            for p in &extrema {
                let d = *p - center;
                let (x, y) = (d.dot(ellipse.u_axis()) / a, d.dot(ellipse.v_axis()) / b);
                assert!(
                    (x.hypot(y) - 1.0).abs() <= 1e-12 && d.dot(ellipse.normal()).abs() <= 1e-12,
                    "arc ({t0}, {t1}): extremum {p:?} off the ellipse"
                );
            }
            let mut pts = vec![ellipse.evaluate(t0), ellipse.evaluate(t1)];
            pts.extend(extrema);
            let boxed = Aabb3::from_points(pts).expanded(1e-12);
            for i in 0..=20_000 {
                let t = (t1 - t0).mul_add(f64::from(i) / 20_000.0, t0);
                assert!(
                    boxed.contains_point(ellipse.evaluate(t)),
                    "arc ({t0}, {t1}): point at t={t} escapes the box"
                );
            }
        }
    }

    /// A full circle yields all six axis extremes whatever its seam, and a
    /// window that misses every extremum yields nothing.
    #[test]
    fn full_circle_yields_every_axis_extreme() {
        let center = Point3::new(0.5, -1.5, 0.5);
        for k in 0..16 {
            let seam = f64::from(k).mul_add(std::f64::consts::TAU / 16.0, 0.3);
            let circle = Circle3D::new_with_ref(
                center,
                Vec3::new(0.0, 0.0, 1.0),
                2.0,
                Vec3::new(seam.cos(), seam.sin(), 0.0),
            )
            .unwrap();
            let curve = EdgeCurve::Circle(circle);
            let pts = conic_arc_axis_extrema(&curve, 0.0, std::f64::consts::TAU);
            let bb = Aabb3::from_points(pts);
            for (got, want) in [
                (bb.min.x(), -1.5),
                (bb.max.x(), 2.5),
                (bb.min.y(), -3.5),
                (bb.max.y(), 0.5),
            ] {
                assert!((got - want).abs() <= 1e-12, "seam {seam}: {got} vs {want}");
            }
        }
        let circle = Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, 0.0, 1.0),
            2.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let curve = EdgeCurve::Circle(circle);
        // With the seam on +x every axis extreme (the plane-normal z axis's
        // included) sits at a multiple of π/2, so this window holds none.
        assert!(conic_arc_axis_extrema(&curve, 0.1, 1.4).is_empty());
    }
}
