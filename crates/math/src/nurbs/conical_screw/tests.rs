#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use crate::nurbs::screw::ScrewMotion;

const ORIGIN: Point3 = Point3::new(1.0, -2.0, 0.5);

fn axis() -> Vec3 {
    Vec3::new(1.0, 2.0, 2.0) * (1.0 / 3.0)
}

/// An oblique axis through an off-origin point, so nothing relies on `z`.
fn motion(rate: f64, radial_rate: f64, height: f64) -> ConicalScrewMotion {
    ConicalScrewMotion::new(ORIGIN, Vec3::new(1.0, 2.0, 2.0), rate, radial_rate, height).unwrap()
}

fn gap(a: Point3, b: Point3) -> f64 {
    (a - b).length()
}

/// (radius, angle from `p`'s own radial direction, height) of `q`.
fn polar(m: &ConicalScrewMotion, p: Point3, q: Point3) -> (f64, f64, f64) {
    let across_p = m.split(p).1;
    let across_q = m.split(q).1;
    let cos = across_p.dot(across_q);
    let sin = axis().dot(across_p.cross(across_q));
    (across_q.length(), sin.atan2(cos), (q - p).dot(axis()))
}

#[test]
fn a_point_carried_along_its_radius_runs_a_conical_helix() {
    let (k, tau, h) = (0.7, 0.05, 3.0);
    let m = motion(k, tau, h);
    let p = Point3::new(4.0, 1.0, -1.0);
    let u = m.radial(p).unwrap();
    let rho = m.radius(p);
    for i in 0..=30 {
        let s = h * f64::from(i) / 30.0;
        let (radius, turn, rise) = polar(&m, p, m.point(p, u, s));
        // Radius linear in the advance, advance `s`, turn `k·s`.
        assert!((radius - tau.mul_add(s, rho)).abs() < 1e-12);
        assert!((rise - s).abs() < 1e-12);
        assert!((turn - k * s).abs() < 1e-12, "turn {turn} at s = {s}");
    }
    assert!((m.reach(p, u) - tau.mul_add(h, rho)).abs() < 1e-12);
    // Shrinking toward the axis, the path is farthest out at its start.
    let inward = motion(k, -tau, h);
    assert!((inward.reach(p, u) - rho).abs() < 1e-12);
}

#[test]
fn the_derivative_is_the_paths_tangent() {
    let m = motion(-1.1, 0.08, 2.0);
    let p = Point3::new(3.0, 2.0, 1.0);
    let u = Vec3::new(0.3, -0.4, 0.9);
    let step = 1e-5;
    for i in 1..20 {
        let s = 2.0 * f64::from(i) / 20.0;
        let (_, derivative) = m.sample(p, u, s);
        let finite = (m.point(p, u, s + step) - m.point(p, u, s - step)) * (0.5 / step);
        assert!((derivative - finite).length() < 1e-8, "at s = {s}");
    }
}

#[test]
fn the_fourth_derivative_bound_holds() {
    // Central fourth difference of the exact path against the closed form.
    let m = motion(1.7, -0.12, 1.5);
    let p = Point3::new(2.5, -1.0, 2.0);
    let u = m.radial(p).unwrap() * 1.3;
    let bound = m.fourth_derivative_bound(m.reach(p, u), 1.3);
    let step = 2e-2;
    let mut worst: f64 = 0.0;
    for i in 0..=100 {
        let s = 2.0f64.mul_add(step, (1.5 - 4.0 * step) * f64::from(i) / 100.0);
        let at = |offset: f64| m.point(p, u, offset.mul_add(step, s)) - ORIGIN;
        let fourth =
            (at(-2.0) - at(-1.0) * 4.0 + at(0.0) * 6.0 - at(1.0) * 4.0 + at(2.0)) * step.powi(-4);
        worst = worst.max(fourth.length());
    }
    // The difference quotient is the derivative to O(step²).
    assert!(worst <= bound * 1.001, "{worst} > {bound}");
    assert!(worst > 0.5 * bound, "{worst} vs {bound}");
}

#[test]
fn a_fitted_path_stays_within_its_bound_and_keeps_its_ends() {
    let m = motion(0.9, 0.0625, 4.0);
    let p = Point3::new(5.0, 3.0, -2.0);
    let u = m.radial(p).unwrap();
    let plan = m.plan(m.reach(p, u), 1.0, 1e-6, 512).unwrap();
    let helix = m.helix(&plan, p, u).unwrap();
    let mut worst: f64 = 0.0;
    for i in 0..=20_000 {
        let s = 4.0 * f64::from(i) / 20_000.0;
        worst = worst.max(gap(helix.evaluate(s), m.point(p, u, s)));
    }
    assert!(
        worst <= plan.remainder_bound,
        "{worst} > {}",
        plan.remainder_bound
    );
    assert!(plan.remainder_bound <= 1e-6);
    // The proof is not wildly loose: within three orders of the measured error.
    assert!(
        worst > plan.remainder_bound * 1e-3,
        "{worst} vs {}",
        plan.remainder_bound
    );
    let points = helix.control_points();
    assert_eq!(points[0], p);
    assert_eq!(*points.last().unwrap(), m.point(p, u, 4.0));
}

#[test]
fn every_fit_of_one_plan_shares_its_knots_and_refuses_what_it_does_not_cover() {
    let m = motion(1.3, 0.1, 2.0);
    let plan = m.plan(6.0, 1.6, 1e-7, 512).unwrap();
    let near = Point3::new(1.5, -2.0, 0.5) + m.radial(Point3::new(4.0, 1.0, -1.0)).unwrap();
    let far = Point3::new(4.0, 1.0, -1.0);
    let a = m.helix(&plan, near, m.radial(near).unwrap()).unwrap();
    let b = m.helix(&plan, far, m.radial(far).unwrap() * 1.5).unwrap();
    assert_eq!(a.knots(), b.knots());
    assert_eq!(a.control_points().len(), 2 * plan.spans + 2);
    // A path that leaves the plan's radius, or a longer direction, is
    // refused rather than under-fitted.
    let out = ORIGIN + m.radial(far).unwrap() * 5.9;
    assert!(m.helix(&plan, out, m.radial(far).unwrap()).is_err());
    assert!(m.helix(&plan, far, m.radial(far).unwrap() * 1.7).is_err());
}

/// A rational quarter circle concentric with the axis, radius 3, with the
/// unit radial direction of every control point scaled like the point.
fn concentric_arc(m: &ConicalScrewMotion) -> (NurbsCurve, Vec<Vec3>) {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let x = m.radial(Point3::new(4.0, 1.0, -1.0)).unwrap();
    let y = axis().cross(x);
    let centre = ORIGIN + axis() * 0.7;
    let offsets = [x * 3.0, (x + y) * 3.0, y * 3.0];
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        offsets.iter().map(|&offset| centre + offset).collect(),
        vec![1.0, w, 1.0],
    )
    .unwrap();
    let directions = offsets.iter().map(|&offset| offset * (1.0 / 3.0)).collect();
    (curve, directions)
}

#[test]
fn a_concentric_arc_sweeps_its_cone_within_the_bound() {
    let (k, tau, h) = (0.8, 0.0625, 2.5);
    let m = motion(k, tau, h);
    let (curve, directions) = concentric_arc(&m);
    let reach = curve
        .control_points()
        .iter()
        .zip(&directions)
        .map(|(&p, &u)| m.reach(p, u))
        .fold(0.0, f64::max);
    let longest = directions.iter().map(|u| u.length()).fold(0.0, f64::max);
    let plan = m.plan(reach, longest, 1e-6, 512).unwrap();
    let surface = m.surface(&plan, &curve, &directions).unwrap();
    let mut worst: f64 = 0.0;
    for i in 0..=120 {
        let s = h * f64::from(i) / 120.0;
        for j in 0..=120 {
            let v = f64::from(j) / 120.0;
            // The exact sweep carries each arc point along its own radius.
            let on_arc = curve.evaluate(v);
            let exact = m.point(on_arc, m.radial(on_arc).unwrap(), s);
            let got = surface.evaluate(s, v);
            worst = worst.max(gap(got, exact));
            // Which is the cone of radius 3 + τ·s.
            assert!((m.radius(got) - tau.mul_add(s, 3.0)).abs() <= plan.remainder_bound);
        }
    }
    assert!(
        worst <= plan.remainder_bound,
        "{worst} > {}",
        plan.remainder_bound
    );
    // The end rows are the arc and the arc of the far radius, turned.
    let rows = surface.control_points();
    assert_eq!(rows[0].as_slice(), curve.control_points());
    let moved: Vec<Point3> = curve
        .control_points()
        .iter()
        .zip(&directions)
        .map(|(&p, &u)| m.point(p, u, h))
        .collect();
    assert_eq!(rows.last().unwrap(), &moved);
}

#[test]
fn neighbouring_surfaces_and_edges_share_their_boundary_path_exactly() {
    // The arc ends where a radial line starts: the two sweeps and the edge
    // fitted from that corner must agree bit for bit.
    let m = motion(0.8, -0.05, 2.5);
    let (arc, arc_directions) = concentric_arc(&m);
    let joint = *arc.control_points().last().unwrap();
    let radial = *arc_directions.last().unwrap();
    let line = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![joint, joint + radial * 0.8],
        vec![1.0, 1.0],
    )
    .unwrap();
    let line_directions = [radial, radial];
    let reach = m
        .reach(joint + radial * 0.8, radial)
        .max(m.reach(arc.control_points()[1], arc_directions[1]));
    let plan = m.plan(reach, 1.5, 1e-6, 512).unwrap();
    let a = m.surface(&plan, &arc, &arc_directions).unwrap();
    let b = m.surface(&plan, &line, &line_directions).unwrap();
    let edge = m.helix(&plan, joint, radial).unwrap();
    let last_column: Vec<Point3> = a
        .control_points()
        .iter()
        .map(|row| *row.last().unwrap())
        .collect();
    let first_column: Vec<Point3> = b.control_points().iter().map(|row| row[0]).collect();
    assert_eq!(last_column, first_column);
    assert_eq!(last_column.as_slice(), edge.control_points());
    assert_eq!(a.knots_u(), edge.knots());
    assert_eq!(b.knots_u(), edge.knots());
    // The line's sweep is a ruled surface: radial at every advance.
    for i in 0..=20 {
        let s = 2.5 * f64::from(i) / 20.0;
        let (inner, outer) = (b.evaluate(s, 0.0), b.evaluate(s, 1.0));
        assert!((m.radius(outer) - m.radius(inner) - 0.8).abs() <= 2.0 * plan.remainder_bound);
        let middle = b.evaluate(s, 0.5);
        assert!(gap(middle, inner + (outer - inner) * 0.5) < 1e-12);
    }
}

#[test]
fn without_a_radial_rate_it_is_the_screw_motion() {
    let (k, h) = (0.8, 2.5);
    let m = motion(k, 0.0, h);
    let screw = ScrewMotion::new(ORIGIN, Vec3::new(1.0, 2.0, 2.0), k, h).unwrap();
    let p = Point3::new(4.0, 1.0, -1.0);
    let u = m.radial(p).unwrap();
    for i in 0..=25 {
        let s = h * f64::from(i) / 25.0;
        assert!(gap(m.point(p, u, s), screw.point(p, s)) < 1e-12);
    }
    let plan = m.plan(m.radius(p), 1.0, 1e-6, 512).unwrap();
    let screw_plan = screw.plan(screw.radius(p), 1e-6, 512).unwrap();
    assert_eq!(plan.spans, screw_plan.spans);
    assert!((plan.remainder_bound - screw_plan.remainder_bound).abs() < 1e-18);
}

#[test]
fn invalid_motions_plans_and_direction_nets_are_refused() {
    let o = Point3::new(0.0, 0.0, 0.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    assert!(ConicalScrewMotion::new(o, Vec3::new(0.0, 0.0, 0.0), 1.0, 0.1, 1.0).is_err());
    assert!(ConicalScrewMotion::new(o, z, 1.0, 0.1, 0.0).is_err());
    assert!(ConicalScrewMotion::new(o, z, f64::NAN, 0.1, 1.0).is_err());
    assert!(ConicalScrewMotion::new(o, z, 1.0, f64::INFINITY, 1.0).is_err());
    let m = ConicalScrewMotion::new(o, z, 1.0, 0.1, 1.0).unwrap();
    assert!(m.radial(Point3::new(0.0, 0.0, 3.0)).is_err());
    assert!(m.plan(-1.0, 1.0, 1e-6, 8).is_err());
    assert!(m.plan(1.0, -1.0, 1e-6, 8).is_err());
    assert!(m.plan(1.0, 1.0, 0.0, 8).is_err());
    // A tolerance the span cap cannot meet is a resource refusal.
    assert!(matches!(
        m.plan(1.0, 1.0, 1e-30, 4),
        Err(MathError::ResourceLimitExceeded { .. })
    ));
    // One direction per control point.
    let (curve, directions) = concentric_arc(&motion(1.0, 0.1, 1.0));
    let general = motion(1.0, 0.1, 1.0);
    let plan = general.plan(10.0, 2.0, 1e-6, 64).unwrap();
    assert!(general.surface(&plan, &curve, &directions[..2]).is_err());
}
