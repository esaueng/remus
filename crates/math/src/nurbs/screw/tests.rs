#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;

/// An oblique axis through an off-origin point, so nothing relies on `z`.
fn motion(rate: f64, height: f64) -> ScrewMotion {
    ScrewMotion::new(
        Point3::new(1.0, -2.0, 0.5),
        Vec3::new(1.0, 2.0, 2.0),
        rate,
        height,
    )
    .unwrap()
}

/// Distance between two points.
fn gap(a: Point3, b: Point3) -> f64 {
    (a - b).length()
}

#[test]
fn the_exact_motion_is_a_helix_about_the_axis() {
    let m = motion(0.7, 3.0);
    let axis = Vec3::new(1.0, 2.0, 2.0) * (1.0 / 3.0);
    let p = Point3::new(4.0, 1.0, -1.0);
    for i in 0..=30 {
        let s = 3.0 * f64::from(i) / 30.0;
        let q = m.point(p, s);
        // Constant radius, advance `s` along the axis, turn `0.7·s`.
        assert!((m.radius(q) - m.radius(p)).abs() < 1e-12);
        assert!(((q - p).dot(axis) - s).abs() < 1e-12);
        let (_, across_p) = m.split(p);
        let (_, across_q) = m.split(q);
        let cos = across_p.dot(across_q) / across_p.length().powi(2);
        let sin = axis.dot(across_p.cross(across_q)) / across_p.length().powi(2);
        let turn = sin.atan2(cos);
        assert!((turn - 0.7 * s).abs() < 1e-12, "turn {turn} at s = {s}");
    }
}

#[test]
fn a_fitted_helix_stays_within_its_bound_and_keeps_its_ends() {
    let m = motion(0.9, 4.0);
    let p = Point3::new(5.0, 3.0, -2.0);
    let plan = m.plan(m.radius(p), 1e-6, 512).unwrap();
    let helix = m.helix(&plan, p).unwrap();
    let mut worst: f64 = 0.0;
    for i in 0..=20_000 {
        let s = 4.0 * f64::from(i) / 20_000.0;
        worst = worst.max(gap(helix.evaluate(s), m.point(p, s)));
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
    assert_eq!(*points.last().unwrap(), m.point(p, 4.0));
}

#[test]
fn every_fit_of_one_plan_shares_its_knots() {
    let m = motion(1.3, 2.0);
    let plan = m.plan(6.0, 1e-7, 512).unwrap();
    let near = m.helix(&plan, Point3::new(1.5, -2.0, 0.5)).unwrap();
    let far = m.helix(&plan, Point3::new(4.0, 1.0, -1.0)).unwrap();
    assert_eq!(near.knots(), far.knots());
    assert_eq!(near.control_points().len(), 2 * plan.spans + 2);
    // A point beyond the plan's radius is refused rather than under-fitted.
    assert!(m.helix(&plan, Point3::new(20.0, 0.0, 0.0)).is_err());
}

/// A rational quarter circle away from the axis: positive non-unit weights.
fn quarter_arc() -> NurbsCurve {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(5.0, 0.0, 1.0),
            Point3::new(5.0, 2.0, 1.0),
            Point3::new(3.0, 2.0, 1.0),
        ],
        vec![1.0, w, 1.0],
    )
    .unwrap()
}

/// A polynomial cubic with interior knots.
fn cubic() -> NurbsCurve {
    NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(4.0, 1.0, 0.5),
            Point3::new(4.5, 2.5, 0.0),
            Point3::new(3.5, 3.5, -0.5),
            Point3::new(2.0, 4.0, 0.0),
        ],
        vec![1.0; 5],
    )
    .unwrap()
}

fn max_control_radius(m: &ScrewMotion, curve: &NurbsCurve) -> f64 {
    curve
        .control_points()
        .iter()
        .map(|&p| m.radius(p))
        .fold(0.0, f64::max)
}

#[test]
fn swept_surfaces_stay_within_the_bound_of_the_exact_sweep() {
    let m = motion(0.8, 2.5);
    for curve in [quarter_arc(), cubic()] {
        let plan = m.plan(max_control_radius(&m, &curve), 1e-6, 512).unwrap();
        let surface = m.surface(&plan, &curve).unwrap();
        let mut worst: f64 = 0.0;
        for i in 0..=120 {
            let s = 2.5 * f64::from(i) / 120.0;
            for j in 0..=120 {
                let v = f64::from(j) / 120.0;
                let exact = m.point(curve.evaluate(v), s);
                worst = worst.max(gap(surface.evaluate(s, v), exact));
            }
        }
        assert!(
            worst <= plan.remainder_bound,
            "{worst} > {}",
            plan.remainder_bound
        );
    }
}

#[test]
fn the_end_rows_are_the_curve_and_its_moved_copy() {
    let m = motion(0.8, 2.5);
    let curve = quarter_arc();
    let plan = m.plan(max_control_radius(&m, &curve), 1e-6, 512).unwrap();
    let surface = m.surface(&plan, &curve).unwrap();
    let rows = surface.control_points();
    assert_eq!(rows[0].as_slice(), curve.control_points());
    let moved: Vec<Point3> = curve
        .control_points()
        .iter()
        .map(|&p| m.point(p, 2.5))
        .collect();
    assert_eq!(rows.last().unwrap(), &moved);
    for i in 0..=50 {
        let v = f64::from(i) / 50.0;
        assert!(gap(surface.evaluate(0.0, v), curve.evaluate(v)) < 1e-13);
    }
}

#[test]
fn neighbouring_surfaces_and_edges_share_their_boundary_helix_exactly() {
    // The arc's last control point starts the cubic: the two sweeps and the
    // helix fitted from that point must agree bit for bit.
    let m = motion(0.8, 2.5);
    let arc = quarter_arc();
    let joint = *arc.control_points().last().unwrap();
    let mut points = cubic().control_points().to_vec();
    points[0] = joint;
    let next = NurbsCurve::new(3, cubic().knots().to_vec(), points, vec![1.0; 5]).unwrap();
    let radius = max_control_radius(&m, &arc).max(max_control_radius(&m, &next));
    let plan = m.plan(radius, 1e-6, 512).unwrap();
    let a = m.surface(&plan, &arc).unwrap();
    let b = m.surface(&plan, &next).unwrap();
    let edge = m.helix(&plan, joint).unwrap();
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
}

#[test]
fn a_motion_without_turn_is_a_straight_extrusion() {
    let m = motion(0.0, 2.0);
    let plan = m.plan(10.0, 1e-9, 8).unwrap();
    assert_eq!(plan.spans, 1);
    assert!(plan.remainder_bound <= 0.0);
    let surface = m.surface(&plan, &cubic()).unwrap();
    let axis = Vec3::new(1.0, 2.0, 2.0) * (1.0 / 3.0);
    for i in 0..=20 {
        let v = f64::from(i) / 20.0;
        let expected = cubic().evaluate(v) + axis * 1.2;
        assert!(gap(surface.evaluate(1.2, v), expected) < 1e-12);
    }
}

#[test]
fn invalid_motions_and_plans_are_refused() {
    let o = Point3::new(0.0, 0.0, 0.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    assert!(ScrewMotion::new(o, Vec3::new(0.0, 0.0, 0.0), 1.0, 1.0).is_err());
    assert!(ScrewMotion::new(o, z, 1.0, 0.0).is_err());
    assert!(ScrewMotion::new(o, z, 1.0, -1.0).is_err());
    assert!(ScrewMotion::new(o, z, f64::NAN, 1.0).is_err());
    assert!(ScrewMotion::new(o, z, 1.0, f64::INFINITY).is_err());
    let m = ScrewMotion::new(o, z, 1.0, 1.0).unwrap();
    assert!(m.plan(-1.0, 1e-6, 8).is_err());
    assert!(m.plan(1.0, 0.0, 8).is_err());
    // A tolerance the span cap cannot meet is a resource refusal.
    assert!(matches!(
        m.plan(1.0, 1e-30, 4),
        Err(MathError::ResourceLimitExceeded { .. })
    ));
}
