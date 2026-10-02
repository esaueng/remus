use super::*;
use crate::gcs::diagnostics::SolveClassification;

const TOL: f64 = 1e-10;

#[test]
fn plain_solve_skips_diagnostics_capture() {
    let mut sys = GcsSystem::new();
    let p = sys
        .add_point(PointData {
            x: 5.0,
            y: 7.0,
            fixed: false,
        })
        .unwrap();
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();

    let (plain, capture, _) = sys.solve_impl(100, TOL, false).unwrap();
    assert!(plain.converged);
    assert!(capture.is_none());
    assert!((sys.point(p).unwrap().x - 2.0).abs() < TOL);

    let (detailed, capture, _) = sys.solve_impl(100, TOL, true).unwrap();
    assert!(detailed.converged);
    assert!(capture.is_some());

    let mut fixed = GcsSystem::new();
    let p = fixed
        .add_point(PointData {
            x: 2.0,
            y: 3.0,
            fixed: true,
        })
        .unwrap();
    fixed.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    let (_, capture, _) = fixed.solve_impl(100, TOL, false).unwrap();
    assert!(capture.is_none());
    let (_, capture, _) = fixed.solve_impl(100, TOL, true).unwrap();
    assert!(capture.is_some());
}

#[test]
fn fix_xy_converges() {
    let mut sys = GcsSystem::new();
    let p = sys
        .add_point(PointData {
            x: 5.0,
            y: 7.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.add_constraint(Constraint::FixY(p, 3.0)).unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    let pt = sys.point(p).unwrap();
    assert!((pt.x - 2.0).abs() < TOL);
    assert!((pt.y - 3.0).abs() < TOL);
}

#[test]
fn distance_constraint() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 0.5,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    sys.add_constraint(Constraint::Distance(p0, p1, 3.0))
        .unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged, "max_r = {}", result.max_residual);
    let pt0 = sys.point(p0).unwrap();
    let pt1 = sys.point(p1).unwrap();
    let dist = ((pt1.x - pt0.x).powi(2) + (pt1.y - pt0.y).powi(2)).sqrt();
    assert!(
        (dist - 3.0).abs() < 1e-6,
        "distance should be 3.0, got {dist}"
    );
}

#[test]
fn coincident_constraint() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 1.0,
            y: 2.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 3.0,
            y: 4.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    sys.add_constraint(Constraint::Coincident(p0, p1)).unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    let pt = sys.point(p1).unwrap();
    assert!((pt.x - 1.0).abs() < TOL);
    assert!((pt.y - 2.0).abs() < TOL);
}

#[test]
fn horizontal_line() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 1.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 5.0,
            y: 3.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let l = sys.add_line(p0, p1).unwrap();
    sys.add_constraint(Constraint::Horizontal(l)).unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    assert!((sys.point(p1).unwrap().y - sys.point(p0).unwrap().y).abs() < TOL);
}

#[test]
fn vertical_line() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 2.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 5.0,
            y: 7.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let l = sys.add_line(p0, p1).unwrap();
    sys.add_constraint(Constraint::Vertical(l)).unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    assert!((sys.point(p1).unwrap().x - sys.point(p0).unwrap().x).abs() < TOL);
}

#[test]
fn perpendicular_lines() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p2 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p3 = sys
        .add_point(PointData {
            x: 0.5,
            y: 0.5,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let l1 = sys.add_line(p0, p1).unwrap();
    let l2 = sys.add_line(p2, p3).unwrap();
    sys.add_constraint(Constraint::Perpendicular(l1, l2))
        .unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    let pt3 = sys.point(p3).unwrap();
    // Line p0-p1 is along X. Perpendicular means p3.x - p2.x = 0
    assert!(pt3.x.abs() < TOL, "p3.x = {}", pt3.x);
}

#[test]
fn parallel_lines() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 1.0,
            y: 1.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p2 = sys
        .add_point(PointData {
            x: 2.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p3 = sys
        .add_point(PointData {
            x: 3.0,
            y: 0.5,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let l1 = sys.add_line(p0, p1).unwrap();
    let l2 = sys.add_line(p2, p3).unwrap();
    sys.add_constraint(Constraint::Parallel(l1, l2)).unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    let pt3 = sys.point(p3).unwrap();
    let dy = pt3.y - 0.0; // p2.y = 0
    let dx = pt3.x - 2.0; // p2.x = 2
    // Cross with (1,1) should be 0: dy - dx = 0
    assert!((dy - dx).abs() < TOL, "not parallel: dy={dy}, dx={dx}");
}

#[test]
fn rectangle_30x20() {
    let mut sys = GcsSystem::new();

    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 25.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p2 = sys
        .add_point(PointData {
            x: 26.0,
            y: 18.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p3 = sys
        .add_point(PointData {
            x: 1.0,
            y: 17.0,
            fixed: false,
        })
        .expect("test coordinates are finite");

    let bottom = sys.add_line(p0, p1).unwrap();
    let right = sys.add_line(p1, p2).unwrap();
    let top = sys.add_line(p2, p3).unwrap();
    let left = sys.add_line(p3, p0).unwrap();

    sys.add_constraint(Constraint::FixX(p0, 0.0)).unwrap();
    sys.add_constraint(Constraint::FixY(p0, 0.0)).unwrap();

    sys.add_constraint(Constraint::Horizontal(bottom)).unwrap();
    sys.add_constraint(Constraint::Distance(p0, p1, 30.0))
        .unwrap();

    sys.add_constraint(Constraint::Vertical(right)).unwrap();
    sys.add_constraint(Constraint::Distance(p1, p2, 20.0))
        .unwrap();

    sys.add_constraint(Constraint::Horizontal(top)).unwrap();
    sys.add_constraint(Constraint::Distance(p2, p3, 30.0))
        .unwrap();

    sys.add_constraint(Constraint::Vertical(left)).unwrap();
    sys.add_constraint(Constraint::Distance(p3, p0, 20.0))
        .unwrap();

    let result = sys.solve(200, 1e-8).unwrap();
    assert!(
        result.converged,
        "rectangle: max_r = {}",
        result.max_residual
    );

    let eps = 1e-4;
    let pt0 = sys.point(p0).unwrap();
    let pt1 = sys.point(p1).unwrap();
    let pt2 = sys.point(p2).unwrap();
    let pt3 = sys.point(p3).unwrap();

    assert!(pt0.x.abs() < eps, "p0.x = {}", pt0.x);
    assert!(pt0.y.abs() < eps, "p0.y = {}", pt0.y);
    assert!((pt1.x - 30.0).abs() < eps, "p1.x = {}", pt1.x);
    assert!(pt1.y.abs() < eps, "p1.y = {}", pt1.y);
    assert!((pt2.x - 30.0).abs() < eps, "p2.x = {}", pt2.x);
    assert!((pt2.y - 20.0).abs() < eps, "p2.y = {}", pt2.y);
    assert!(pt3.x.abs() < eps, "p3.x = {}", pt3.x);
    assert!((pt3.y - 20.0).abs() < eps, "p3.y = {}", pt3.y);
}

#[test]
fn dof_analysis() {
    let mut sys = GcsSystem::new();
    let p = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");

    // Free point: 2 DOF
    let dof = sys.dof().unwrap();
    assert_eq!(dof.dof, 2);

    // Fix X: 1 DOF
    let cx = sys.add_constraint(Constraint::FixX(p, 0.0)).unwrap();
    let dof = sys.dof().unwrap();
    assert_eq!(dof.dof, 1);

    // Fix Y: 0 DOF
    sys.add_constraint(Constraint::FixY(p, 0.0)).unwrap();
    let dof = sys.dof().unwrap();
    assert_eq!(dof.dof, 0);

    // Remove FixX: back to 1 DOF
    sys.remove_constraint(cx).unwrap();
    let dof = sys.dof().unwrap();
    assert_eq!(dof.dof, 1);
}

#[test]
fn remove_point_in_use_fails() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let _l = sys.add_line(p0, p1).unwrap();

    assert!(sys.remove_point(p0).is_err());
}

#[test]
fn remove_line_in_use_fails() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let l = sys.add_line(p0, p1).unwrap();
    sys.add_constraint(Constraint::Horizontal(l)).unwrap();

    assert!(sys.remove_line(l).is_err());
}

#[test]
fn stale_constraint_handle() {
    let mut sys = GcsSystem::new();
    let p = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let c = sys.add_constraint(Constraint::FixX(p, 0.0)).unwrap();
    sys.remove_constraint(c).unwrap();
    assert!(sys.remove_constraint(c).is_err());
}

#[test]
fn solve_after_removal() {
    let mut sys = GcsSystem::new();
    let p = sys
        .add_point(PointData {
            x: 5.0,
            y: 7.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let _cx = sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    let cy = sys.add_constraint(Constraint::FixY(p, 3.0)).unwrap();

    // Solve, then remove FixY, re-solve
    let r1 = sys.solve(100, TOL).unwrap();
    assert!(r1.converged);

    sys.remove_constraint(cy).unwrap();
    let r2 = sys.solve(100, TOL).unwrap();
    assert!(r2.converged);
    // X should still be at 2.0, Y should be unchanged from last solve
    assert!((sys.point(p).unwrap().x - 2.0).abs() < TOL);
}

#[test]
fn add_constraint_with_invalid_handle_fails() {
    let mut sys = GcsSystem::new();
    let p = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    sys.remove_point(p).unwrap();
    assert!(sys.add_constraint(Constraint::FixX(p, 0.0)).is_err());
}

#[test]
fn triangle_345() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p2 = sys
        .add_point(PointData {
            x: 0.5,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");

    let bottom = sys.add_line(p0, p1).unwrap();
    sys.add_constraint(Constraint::Horizontal(bottom)).unwrap();
    sys.add_constraint(Constraint::Distance(p0, p1, 3.0))
        .unwrap();
    sys.add_constraint(Constraint::Distance(p0, p2, 4.0))
        .unwrap();
    sys.add_constraint(Constraint::Distance(p1, p2, 5.0))
        .unwrap();

    let result = sys.solve(200, 1e-8).unwrap();
    assert!(
        result.converged,
        "triangle: max_r = {}",
        result.max_residual
    );

    let pt1 = sys.point(p1).unwrap();
    let d01 = (pt1.x.powi(2) + pt1.y.powi(2)).sqrt();
    assert!((d01 - 3.0).abs() < 1e-4, "d01 = {d01}");
}

#[test]
fn fixed_points_no_solve_needed() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    sys.add_constraint(Constraint::Distance(p0, p1, 1.0))
        .unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    assert_eq!(result.iterations, 0);
}

#[test]
fn add_arc_basic() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let e = sys
        .add_point(PointData {
            x: 0.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc = sys.add_arc(c, s, e).unwrap();
    assert_eq!(sys.arc_count(), 1);
    let data = sys.arc(arc).unwrap();
    assert_eq!(data.center, c);
    assert_eq!(data.start, s);
    assert_eq!(data.end, e);
}

#[test]
fn remove_arc_cleans_up() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let e = sys
        .add_point(PointData {
            x: 0.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc = sys.add_arc(c, s, e).unwrap();
    let count_before = sys.constraint_count();
    assert!(count_before > 0, "internal constraint should exist");
    sys.remove_arc(arc).unwrap();
    assert_eq!(sys.arc_count(), 0);
    assert!(
        sys.constraint_count() < count_before,
        "internal constraint should be removed"
    );
}

#[test]
fn point_on_circle_converges() {
    let mut sys = GcsSystem::new();
    let center = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let circ = sys.add_circle(center, 2.0).unwrap();
    let pt = sys
        .add_point(PointData {
            x: 3.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    sys.add_constraint(Constraint::PointOnCircle(pt, circ))
        .unwrap();
    let result = sys.solve(100, 1e-10).unwrap();
    assert!(result.converged);
    let p = sys.point(pt).unwrap();
    let dist = (p.x * p.x + p.y * p.y).sqrt();
    assert!(
        (dist - 2.0).abs() < 1e-6,
        "point should be on circle, dist={dist}"
    );
}

#[test]
fn point_on_arc_converges() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s = sys
        .add_point(PointData {
            x: 2.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let e = sys
        .add_point(PointData {
            x: 0.0,
            y: 2.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc = sys.add_arc(c, s, e).unwrap();
    let pt = sys
        .add_point(PointData {
            x: 3.0,
            y: 3.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    sys.add_constraint(Constraint::PointOnArc(pt, arc)).unwrap();
    let result = sys.solve(100, 1e-10).unwrap();
    assert!(result.converged);
    let p = sys.point(pt).unwrap();
    let dist = (p.x * p.x + p.y * p.y).sqrt();
    assert!(
        (dist - 2.0).abs() < 1e-6,
        "point should be on arc circle, dist={dist}"
    );
}

#[test]
fn tangent_line_arc_converges() {
    let mut sys = GcsSystem::new();
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 2.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let line = sys.add_line(p0, p1).unwrap();
    let c = sys
        .add_point(PointData {
            x: 2.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let s = sys
        .add_point(PointData {
            x: 2.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let e = sys
        .add_point(PointData {
            x: 3.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc = sys.add_arc(c, s, e).unwrap();
    sys.add_constraint(Constraint::Coincident(p1, s)).unwrap();
    sys.add_constraint(Constraint::TangentLineArc(line, arc, p1))
        .unwrap();
    let result = sys.solve(100, 1e-10).unwrap();
    assert!(result.converged, "tangent line-arc should converge");
    let sp = sys.point(s).unwrap();
    let cp = sys.point(c).unwrap();
    let radius_dir = (sp.x - cp.x, sp.y - cp.y);
    let dot = 1.0 * radius_dir.0 + 0.0 * radius_dir.1;
    assert!(dot.abs() < 1e-6, "line should be tangent to arc, dot={dot}");
}

#[test]
fn equal_radius_arc_arc_converges() {
    let mut sys = GcsSystem::new();
    let c1 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s1 = sys
        .add_point(PointData {
            x: 2.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let e1 = sys
        .add_point(PointData {
            x: 0.0,
            y: 2.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc1 = sys.add_arc(c1, s1, e1).unwrap();
    let c2 = sys
        .add_point(PointData {
            x: 5.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s2 = sys
        .add_point(PointData {
            x: 8.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let e2 = sys
        .add_point(PointData {
            x: 5.0,
            y: 3.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc2 = sys.add_arc(c2, s2, e2).unwrap();
    sys.add_constraint(Constraint::EqualRadiusArcArc(arc1, arc2))
        .unwrap();
    let result = sys.solve(100, 1e-10).unwrap();
    assert!(result.converged);
    let r1 = {
        let s = sys.point(s1).unwrap();
        (s.x * s.x + s.y * s.y).sqrt()
    };
    let r2 = {
        let cp = sys.point(c2).unwrap();
        let sp = sys.point(s2).unwrap();
        ((sp.x - cp.x).powi(2) + (sp.y - cp.y).powi(2)).sqrt()
    };
    assert!(
        (r1 - r2).abs() < 1e-6,
        "radii should be equal: r1={r1}, r2={r2}"
    );
}

#[test]
fn concentric_arc_arc_converges() {
    let mut sys = GcsSystem::new();
    let c1 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s1 = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let e1 = sys
        .add_point(PointData {
            x: 0.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc1 = sys.add_arc(c1, s1, e1).unwrap();
    let c2 = sys
        .add_point(PointData {
            x: 0.5,
            y: 0.5,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let s2 = sys
        .add_point(PointData {
            x: 2.5,
            y: 0.5,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let e2 = sys
        .add_point(PointData {
            x: 0.5,
            y: 2.5,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let arc2 = sys.add_arc(c2, s2, e2).unwrap();
    sys.add_constraint(Constraint::ConcentricArcArc(arc1, arc2))
        .unwrap();
    let result = sys.solve(100, 1e-10).unwrap();
    assert!(result.converged);
    let cp1 = sys.point(c1).unwrap();
    let cp2 = sys.point(c2).unwrap();
    assert!((cp1.x - cp2.x).abs() < 1e-6 && (cp1.y - cp2.y).abs() < 1e-6);
}

#[test]
fn slot_profile_line_arc_tangent() {
    let mut sys = GcsSystem::new();
    // 4 corner points for a 4-unit-long, 2-unit-wide slot
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let p1 = sys
        .add_point(PointData {
            x: 4.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p2 = sys
        .add_point(PointData {
            x: 4.0,
            y: 2.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let p3 = sys
        .add_point(PointData {
            x: 0.0,
            y: 2.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    // Two horizontal lines
    let bottom_line = sys.add_line(p0, p1).unwrap();
    let top_line = sys.add_line(p3, p2).unwrap();
    // Right semicircle: center at (4, 1), connecting p1 to p2
    let rc = sys
        .add_point(PointData {
            x: 4.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let right_arc = sys.add_arc(rc, p1, p2).unwrap();
    // Left semicircle: center at (0, 1), connecting p3 to p0
    let lc = sys
        .add_point(PointData {
            x: 0.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let left_arc = sys.add_arc(lc, p3, p0).unwrap();
    // Tangent constraints at all 4 junctions
    sys.add_constraint(Constraint::TangentLineArc(bottom_line, right_arc, p1))
        .unwrap();
    sys.add_constraint(Constraint::TangentLineArc(top_line, right_arc, p2))
        .unwrap();
    sys.add_constraint(Constraint::TangentLineArc(top_line, left_arc, p3))
        .unwrap();
    sys.add_constraint(Constraint::TangentLineArc(bottom_line, left_arc, p0))
        .unwrap();
    // Dimension constraints
    sys.add_constraint(Constraint::Distance(p0, p1, 4.0))
        .unwrap();
    sys.add_constraint(Constraint::Horizontal(bottom_line))
        .unwrap();
    sys.add_constraint(Constraint::Parallel(bottom_line, top_line))
        .unwrap();
    sys.add_constraint(Constraint::Distance(p0, p3, 2.0))
        .unwrap();

    let result = sys.solve(200, 1e-8).unwrap();
    assert!(result.converged, "slot profile should converge: {result:?}");
}

#[test]
fn arc_endpoints_equidistant_from_center() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    // End point starts off-circle — solver should move it onto the circle
    // (start is fixed, so the dynamic radius is pinned at 1.0)
    let e = sys
        .add_point(PointData {
            x: 0.0,
            y: 1.5,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let _arc = sys.add_arc(c, s, e).unwrap();
    let result = sys.solve(100, 1e-10).unwrap();
    assert!(result.converged);
    let ep = sys.point(e).unwrap();
    let dist = (ep.x * ep.x + ep.y * ep.y).sqrt();
    assert!(
        (dist - 1.0).abs() < 1e-6,
        "end should be on unit circle, dist={dist}"
    );

    // Also verify the dynamic behavior: when start moves, end tracks it.
    // Create a new system where start is free and moved by a FixX constraint.
    let mut sys2 = GcsSystem::new();
    let c2 = sys2
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("test coordinates are finite");
    let s2 = sys2
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let e2 = sys2
        .add_point(PointData {
            x: 0.0,
            y: 1.0,
            fixed: false,
        })
        .expect("test coordinates are finite");
    let _arc2 = sys2.add_arc(c2, s2, e2).unwrap();
    // Push start out to radius 2
    sys2.add_constraint(Constraint::Distance(c2, s2, 2.0))
        .unwrap();
    let result2 = sys2.solve(100, 1e-10).unwrap();
    assert!(result2.converged, "dynamic radius test should converge");
    let sp2 = sys2.point(s2).unwrap();
    let ep2 = sys2.point(e2).unwrap();
    let r_start = (sp2.x * sp2.x + sp2.y * sp2.y).sqrt();
    let r_end = (ep2.x * ep2.x + ep2.y * ep2.y).sqrt();
    assert!(
        (r_end - r_start).abs() < 1e-6,
        "end radius ({r_end}) should track start radius ({r_start})"
    );
}

// ── Solving with the constraints added for selection-first sketching ─

/// Helper: a free point at `(x, y)`.
fn free_pt(sys: &mut GcsSystem, x: f64, y: f64) -> PointId {
    sys.add_point(PointData { x, y, fixed: false })
        .expect("test coordinates are finite")
}

/// Helper: a pinned point at `(x, y)`.
fn fixed_pt(sys: &mut GcsSystem, x: f64, y: f64) -> PointId {
    sys.add_point(PointData { x, y, fixed: true })
        .expect("test coordinates are finite")
}

#[test]
fn circle_radius_drives_the_radius_parameter() {
    for scale in [1e-3, 1.0, 1e5] {
        let mut sys = GcsSystem::new();
        let c = fixed_pt(&mut sys, 0.0, 0.0);
        let circ = sys.add_circle(c, 1.0 * scale).unwrap();
        let target = 7.5 * scale;
        sys.add_constraint(Constraint::CircleRadius(circ, target))
            .unwrap();

        let r = sys.solve(100, TOL * scale.max(1.0)).unwrap();
        assert!(r.converged, "scale {scale}: max_r = {}", r.max_residual);
        let got = sys.circle(circ).unwrap().radius;
        assert!(
            (got - target).abs() < 1e-9 * scale.max(1.0),
            "scale {scale}: expected {target}, got {got}"
        );
    }
}

/// A non-positive or non-finite radius target is rejected at add time rather
/// than handed to the solver.
#[test]
fn circle_radius_rejects_invalid_targets() {
    let mut sys = GcsSystem::new();
    let c = fixed_pt(&mut sys, 0.0, 0.0);
    let circ = sys.add_circle(c, 2.0).unwrap();
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(
            sys.add_constraint(Constraint::CircleRadius(circ, bad))
                .is_err(),
            "radius target {bad} must be rejected"
        );
    }
    assert!(
        sys.add_constraint(Constraint::CircleRadius(circ, 3.0))
            .is_ok(),
        "a positive finite target must still be accepted"
    );
}

#[test]
fn equal_radius_circle_circle_converges() {
    let mut sys = GcsSystem::new();
    let c1 = fixed_pt(&mut sys, 0.0, 0.0);
    let c2 = fixed_pt(&mut sys, 20.0, 0.0);
    let circ1 = sys.add_circle(c1, 3.0).unwrap();
    let circ2 = sys.add_circle(c2, 8.0).unwrap();
    sys.add_constraint(Constraint::CircleRadius(circ1, 5.0))
        .unwrap();
    sys.add_constraint(Constraint::EqualRadiusCircleCircle(circ1, circ2))
        .unwrap();

    let r = sys.solve(100, TOL).unwrap();
    assert!(r.converged, "max_r = {}", r.max_residual);
    let r1 = sys.circle(circ1).unwrap().radius;
    let r2 = sys.circle(circ2).unwrap().radius;
    assert!((r1 - 5.0).abs() < 1e-9, "r1 = {r1}");
    assert!((r2 - 5.0).abs() < 1e-9, "r2 = {r2}");
}

#[test]
fn equal_length_converges_at_several_scales() {
    for scale in [1e-3, 1.0, 1e5] {
        let mut sys = GcsSystem::new();
        // Line 1 is pinned at length 5·scale (3-4-5 triangle).
        let a0 = fixed_pt(&mut sys, 0.0, 0.0);
        let a1 = fixed_pt(&mut sys, 3.0 * scale, 4.0 * scale);
        let l1 = sys.add_line(a0, a1).unwrap();
        // Line 2 shares a pinned start and has a free end well off-target.
        let b0 = fixed_pt(&mut sys, 10.0 * scale, 0.0);
        let b1 = free_pt(&mut sys, 11.0 * scale, 0.0);
        let l2 = sys.add_line(b0, b1).unwrap();
        // Keep line 2 horizontal so the solution is determinate.
        sys.add_constraint(Constraint::Horizontal(l2)).unwrap();
        sys.add_constraint(Constraint::EqualLength(l1, l2)).unwrap();

        let r = sys.solve(200, TOL * scale.max(1.0)).unwrap();
        assert!(r.converged, "scale {scale}: max_r = {}", r.max_residual);

        let p0 = sys.point(b0).unwrap();
        let p1 = sys.point(b1).unwrap();
        let len2 = (p1.x - p0.x).hypot(p1.y - p0.y);
        assert!(
            (len2 - 5.0 * scale).abs() < 1e-8 * scale.max(1.0),
            "scale {scale}: length should reach {}, got {len2}",
            5.0 * scale
        );
    }
}

#[test]
fn midpoint_pulls_a_point_to_the_line_centre() {
    for scale in [1e-3, 1.0, 1e5] {
        let mut sys = GcsSystem::new();
        let a = fixed_pt(&mut sys, -4.0 * scale, 2.0 * scale);
        let b = fixed_pt(&mut sys, 10.0 * scale, 8.0 * scale);
        let line = sys.add_line(a, b).unwrap();
        let mid = free_pt(&mut sys, 0.0, 0.0);
        sys.add_constraint(Constraint::Midpoint(mid, line)).unwrap();

        let r = sys.solve(100, TOL * scale.max(1.0)).unwrap();
        assert!(r.converged, "scale {scale}: max_r = {}", r.max_residual);
        let m = sys.point(mid).unwrap();
        assert!(
            (m.x - 3.0 * scale).abs() < 1e-9 * scale.max(1.0)
                && (m.y - 5.0 * scale).abs() < 1e-9 * scale.max(1.0),
            "scale {scale}: midpoint should be ({}, {}), got ({}, {})",
            3.0 * scale,
            5.0 * scale,
            m.x,
            m.y
        );
    }
}

/// A free point is mirrored onto the true reflection of its pinned partner.
#[test]
fn symmetric_mirrors_a_point_across_the_axis() {
    for scale in [1e-3, 1.0, 1e5] {
        let mut sys = GcsSystem::new();
        // Axis: the vertical line x = 2·scale.
        let ax = fixed_pt(&mut sys, 2.0 * scale, -scale);
        let bx = fixed_pt(&mut sys, 2.0 * scale, 5.0 * scale);
        let axis = sys.add_line(ax, bx).unwrap();

        let p1 = fixed_pt(&mut sys, -3.0 * scale, 4.0 * scale);
        // Start well away from the answer so the solver has to work.
        let p2 = free_pt(&mut sys, 0.0, 0.0);
        sys.add_constraint(Constraint::Symmetric(p1, p2, axis))
            .unwrap();

        let r = sys.solve(200, TOL * scale.max(1.0)).unwrap();
        assert!(r.converged, "scale {scale}: max_r = {}", r.max_residual);

        // Reflection of (-3, 4) about x = 2 is (7, 4).
        let m = sys.point(p2).unwrap();
        assert!(
            (m.x - 7.0 * scale).abs() < 1e-8 * scale.max(1.0)
                && (m.y - 4.0 * scale).abs() < 1e-8 * scale.max(1.0),
            "scale {scale}: expected ({}, {}), got ({}, {})",
            7.0 * scale,
            4.0 * scale,
            m.x,
            m.y
        );
    }
}

/// Symmetry about a slanted axis, verified by the two defining properties
/// rather than by a precomputed coordinate.
#[test]
fn symmetric_about_slanted_axis() {
    let mut sys = GcsSystem::new();
    // Axis through the origin at 30°.
    let (c, s) = (30.0_f64.to_radians()).sin_cos();
    let ax = fixed_pt(&mut sys, 0.0, 0.0);
    let bx = fixed_pt(&mut sys, 10.0 * s, 10.0 * c);
    let axis = sys.add_line(ax, bx).unwrap();

    let p1 = fixed_pt(&mut sys, 6.0, 1.0);
    let p2 = free_pt(&mut sys, -1.0, -1.0);
    sys.add_constraint(Constraint::Symmetric(p1, p2, axis))
        .unwrap();

    let r = sys.solve(200, TOL).unwrap();
    assert!(r.converged, "max_r = {}", r.max_residual);

    let a = sys.point(ax).unwrap();
    let b = sys.point(bx).unwrap();
    let q1 = sys.point(p1).unwrap();
    let q2 = sys.point(p2).unwrap();
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len = dx.hypot(dy);

    // 1. The midpoint lies on the axis.
    let (mx, my) = (0.5 * (q1.x + q2.x) - a.x, 0.5 * (q1.y + q2.y) - a.y);
    assert!(
        ((dx * my - dy * mx) / len).abs() < 1e-8,
        "midpoint must lie on the axis"
    );
    // 2. The joining segment is perpendicular to the axis.
    assert!(
        ((dx * (q2.x - q1.x) + dy * (q2.y - q1.y)) / len).abs() < 1e-8,
        "segment must be perpendicular to the axis"
    );
    // 3. Both points are the same distance from the axis, on opposite sides.
    let side = |p: &PointData| (dx * (p.y - a.y) - dy * (p.x - a.x)) / len;
    assert!(
        (side(q1) + side(q2)).abs() < 1e-8 && side(q1).abs() > 1.0,
        "points must straddle the axis at equal distance"
    );
}

/// Removing a constraint that references a line or circle must be possible,
/// and the entity must stay locked while the constraint lives.
#[test]
fn new_constraints_participate_in_entity_lifetime() {
    let mut sys = GcsSystem::new();
    let a = free_pt(&mut sys, 0.0, 0.0);
    let b = free_pt(&mut sys, 1.0, 0.0);
    let l1 = sys.add_line(a, b).unwrap();
    let c = free_pt(&mut sys, 0.0, 1.0);
    let d = free_pt(&mut sys, 1.0, 1.0);
    let l2 = sys.add_line(c, d).unwrap();
    let cid = sys.add_constraint(Constraint::EqualLength(l1, l2)).unwrap();

    assert!(
        sys.remove_line(l2).is_err(),
        "a line referenced by equalLength must not be removable"
    );
    sys.remove_constraint(cid).unwrap();
    assert!(
        sys.remove_line(l2).is_ok(),
        "removal must succeed once the constraint is gone"
    );

    // Same for a circle held by circleRadius.
    let centre = free_pt(&mut sys, 5.0, 5.0);
    let circ = sys.add_circle(centre, 2.0).unwrap();
    let rid = sys
        .add_constraint(Constraint::CircleRadius(circ, 4.0))
        .unwrap();
    assert!(sys.remove_circle(circ).is_err());
    sys.remove_constraint(rid).unwrap();
    assert!(sys.remove_circle(circ).is_ok());

    // And for a point held by midpoint / symmetric.
    let m = free_pt(&mut sys, 9.0, 9.0);
    let mid_id = sys.add_constraint(Constraint::Midpoint(m, l1)).unwrap();
    assert!(sys.remove_point(m).is_err());
    sys.remove_constraint(mid_id).unwrap();
    assert!(sys.remove_point(m).is_ok());
}

// ── Diagnostics ─────────────────────────────────────────────────────

#[test]
fn diagnostics_report_under_constrained_geometry() {
    let mut sys = GcsSystem::new();
    let a = fixed_pt(&mut sys, 0.0, 0.0);
    let b = free_pt(&mut sys, 3.0, 0.0);
    sys.add_constraint(Constraint::Distance(a, b, 5.0)).unwrap();

    let d = sys.solve_detailed(200, TOL).unwrap();
    assert!(d.converged);
    // b has 2 free params, one distance equation → 1 DOF left.
    assert_eq!(d.num_params, 2);
    assert_eq!(d.num_equations, 1);
    assert_eq!(d.rank, 1);
    assert_eq!(d.dof, 1);
    assert_eq!(d.classification, SolveClassification::UnderConstrained);
    assert!(!d.redundant);
    assert!(!d.rolled_back);
}

#[test]
fn diagnostics_report_fully_constrained_geometry() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 5.0, 7.0);
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.add_constraint(Constraint::FixY(p, 3.0)).unwrap();

    let d = sys.solve_detailed(100, TOL).unwrap();
    assert!(d.converged);
    assert_eq!(d.dof, 0);
    assert_eq!(d.rank, 2);
    assert_eq!(d.num_equations, 2);
    assert!(!d.redundant);
    assert_eq!(d.classification, SolveClassification::Solved);
    assert!(!d.rolled_back);
    assert!(d.published_max_residual < TOL);
}

/// A duplicated-but-consistent constraint is redundant, not a conflict: the
/// system still converges and the extra equation is simply dependent.
#[test]
fn diagnostics_report_redundant_constraints() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 5.0, 7.0);
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.add_constraint(Constraint::FixY(p, 3.0)).unwrap();
    // Exactly the same demand again.
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();

    let d = sys.solve_detailed(100, TOL).unwrap();
    assert!(d.converged, "consistent duplication must still solve");
    assert_eq!(d.num_equations, 3);
    assert_eq!(d.rank, 2, "the duplicate adds no rank");
    assert_eq!(d.dof, 0);
    assert!(d.redundant);
    assert_eq!(d.classification, SolveClassification::Redundant);
    assert!(!d.rolled_back);
}

/// Contradictory constraints cannot be satisfied. The report says exactly
/// that and nothing more — no constraint is named as *the* conflict.
#[test]
fn diagnostics_report_contradictory_constraints_without_blaming_one() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 0.0, 0.0);
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    // Irreconcilable with the line above.
    sys.add_constraint(Constraint::FixX(p, 9.0)).unwrap();

    let d = sys.solve_detailed(200, TOL).unwrap();
    assert!(!d.converged);
    assert_eq!(d.classification, SolveClassification::Unsatisfied);

    // Residuals are measured at the solver's best attempt, which lands on the
    // least-squares compromise x = 5.5. Both constraints are then 3.5 off:
    // the report shows *both* as unsatisfied and singles out neither, because
    // the solver has no basis to call either one the culprit.
    let residuals: Vec<f64> = d
        .residuals
        .iter()
        .filter(|r| !r.internal)
        .map(|r| r.max_abs_residual)
        .collect();
    assert_eq!(residuals.len(), 2);
    for v in &residuals {
        assert!(
            (v - 3.5).abs() < 1e-6,
            "each conflicting constraint should sit 3.5 from the compromise, \
             got {residuals:?}"
        );
    }

    // The geometry itself was rolled back, so what is published is the
    // untouched original, not the compromise.
    assert!(d.rolled_back);
    assert!(
        (d.published_max_residual - 9.0).abs() < 1e-9,
        "published state is the original x = 0, where fixX(9) is 9 off, got {}",
        d.published_max_residual
    );
}

/// A rejected solve must not leave partially moved geometry published.
#[test]
fn diagnostics_roll_back_a_failed_solve() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 1.25, -4.5);
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.add_constraint(Constraint::FixX(p, 9.0)).unwrap();

    let d = sys.solve_detailed(200, TOL).unwrap();
    assert!(!d.converged);
    assert!(d.rolled_back, "a non-converged solve must roll back");

    let pt = sys.point(p).unwrap();
    assert!(
        (pt.x - 1.25).abs() < 1e-15 && (pt.y + 4.5).abs() < 1e-15,
        "pre-solve position must be restored exactly, got ({}, {})",
        pt.x,
        pt.y
    );

    // Plain `solve` keeps its original behaviour: it publishes its last
    // iterate rather than rolling back.
    let mut sys2 = GcsSystem::new();
    let q = free_pt(&mut sys2, 1.25, -4.5);
    sys2.add_constraint(Constraint::FixX(q, 2.0)).unwrap();
    sys2.add_constraint(Constraint::FixX(q, 9.0)).unwrap();
    let r = sys2.solve(200, TOL).unwrap();
    assert!(!r.converged);
    let moved = sys2.point(q).unwrap();
    assert!(
        (moved.x - 1.25).abs() > 1e-6,
        "solve() must keep publishing its final iterate, got {}",
        moved.x
    );
}

/// A converged solve publishes its result — rollback is only for failures.
#[test]
fn diagnostics_publish_a_successful_solve() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 0.0, 0.0);
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.add_constraint(Constraint::FixY(p, 3.0)).unwrap();

    let d = sys.solve_detailed(100, TOL).unwrap();
    assert!(d.converged && !d.rolled_back);
    let pt = sys.point(p).unwrap();
    assert!((pt.x - 2.0).abs() < TOL && (pt.y - 3.0).abs() < TOL);
}

#[test]
fn diagnostics_track_dof_restored_by_removing_a_constraint() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 0.0, 0.0);
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    let cid = sys.add_constraint(Constraint::FixY(p, 3.0)).unwrap();

    let before = sys.solve_detailed(100, TOL).unwrap();
    assert_eq!(before.dof, 0);
    assert_eq!(before.classification, SolveClassification::Solved);

    sys.remove_constraint(cid).unwrap();
    let after = sys.solve_detailed(100, TOL).unwrap();
    assert_eq!(after.dof, 1, "removing one equation restores one DOF");
    assert_eq!(after.num_equations, 1);
    assert_eq!(after.classification, SolveClassification::UnderConstrained);
    assert_eq!(
        after.residuals.iter().filter(|r| !r.internal).count(),
        1,
        "the removed constraint must disappear from the report"
    );
}

/// A stale constraint handle is rejected, and the report never mentions it.
#[test]
fn diagnostics_ignore_stale_constraint_handles() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 0.0, 0.0);
    let cid = sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.remove_constraint(cid).unwrap();
    assert!(
        sys.remove_constraint(cid).is_err(),
        "a stale handle must be rejected"
    );

    let d = sys.solve_detailed(100, TOL).unwrap();
    assert!(
        d.residuals.iter().all(|r| r.constraint != cid),
        "a removed constraint must not appear in the report"
    );
    assert_eq!(d.num_equations, 0);
}

/// Internal arc constraints are flagged, counted in the equation total, and
/// summarised separately — never attributed to a caller's constraint.
#[test]
fn diagnostics_separate_internal_arc_constraints() {
    let mut sys = GcsSystem::new();
    let c = fixed_pt(&mut sys, 0.0, 0.0);
    let s = free_pt(&mut sys, 5.0, 0.0);
    // End point deliberately off-radius, so the internal tie has real work.
    let e = free_pt(&mut sys, 0.0, 2.0);
    sys.add_arc(c, s, e).unwrap();
    let user = sys.add_constraint(Constraint::Distance(c, s, 5.0)).unwrap();

    let d = sys.solve_detailed(200, TOL).unwrap();

    let internal: Vec<_> = d.residuals.iter().filter(|r| r.internal).collect();
    assert_eq!(
        internal.len(),
        1,
        "add_arc installs exactly one internal tie"
    );
    assert!(
        internal[0].constraint != user,
        "the internal tie must not share the user's handle"
    );
    assert!(
        sys.is_internal_constraint(internal[0].constraint),
        "is_internal_constraint must agree with the report"
    );
    assert!(!sys.is_internal_constraint(user));
    assert!(
        d.internal_max_residual.is_finite(),
        "internal residual must be reported, got {}",
        d.internal_max_residual
    );
    // Two equations: the caller's distance plus the internal tie.
    assert_eq!(d.num_equations, 2);
}

/// Repeated solves of the same system give bit-identical results.
#[test]
fn diagnostics_are_deterministic_across_repeated_solves() {
    let build = || {
        let mut sys = GcsSystem::new();
        let a = fixed_pt(&mut sys, 0.0, 0.0);
        let b = free_pt(&mut sys, 3.1, 0.7);
        let cpt = free_pt(&mut sys, 1.0, 4.2);
        let l1 = sys.add_line(a, b).unwrap();
        let l2 = sys.add_line(b, cpt).unwrap();
        sys.add_constraint(Constraint::Distance(a, b, 5.0)).unwrap();
        sys.add_constraint(Constraint::Perpendicular(l1, l2))
            .unwrap();
        sys.add_constraint(Constraint::EqualLength(l1, l2)).unwrap();
        (sys, b, cpt)
    };

    let (mut s1, b1, c1) = build();
    let (mut s2, b2, c2) = build();
    let d1 = s1.solve_detailed(300, TOL).unwrap();
    let d2 = s2.solve_detailed(300, TOL).unwrap();

    assert_eq!(d1.converged, d2.converged);
    assert_eq!(d1.iterations, d2.iterations);
    assert_eq!(d1.rank, d2.rank);
    assert_eq!(d1.dof, d2.dof);
    assert_eq!(d1.classification, d2.classification);
    assert!(
        (d1.max_residual - d2.max_residual).abs() < f64::EPSILON,
        "residuals must match bit-for-bit: {} vs {}",
        d1.max_residual,
        d2.max_residual
    );
    assert_eq!(d1.residuals.len(), d2.residuals.len());
    for (r1, r2) in d1.residuals.iter().zip(d2.residuals.iter()) {
        assert_eq!(
            r1.constraint, r2.constraint,
            "constraint order must be stable"
        );
        assert!((r1.max_abs_residual - r2.max_abs_residual).abs() < f64::EPSILON);
    }

    let (p1, p2) = (s1.point(b1).unwrap(), s2.point(b2).unwrap());
    assert!((p1.x - p2.x).abs() < f64::EPSILON && (p1.y - p2.y).abs() < f64::EPSILON);
    let (q1, q2) = (s1.point(c1).unwrap(), s2.point(c2).unwrap());
    assert!((q1.x - q2.x).abs() < f64::EPSILON && (q1.y - q2.y).abs() < f64::EPSILON);
}

/// Residual attribution points at the constraint that is actually violated,
/// and leaves satisfied constraints at (near) zero.
#[test]
fn diagnostics_attribute_residual_per_constraint() {
    let mut sys = GcsSystem::new();
    // Both points pinned, so nothing can move and each constraint's residual
    // is fully determined by the geometry.
    let a = fixed_pt(&mut sys, 0.0, 0.0);
    let b = fixed_pt(&mut sys, 3.0, 4.0);

    let satisfied = sys.add_constraint(Constraint::Distance(a, b, 5.0)).unwrap();
    let violated = sys.add_constraint(Constraint::FixX(b, 10.0)).unwrap();

    let d = sys.solve_detailed(50, TOL).unwrap();
    assert!(!d.converged);

    let find = |id| {
        d.residuals
            .iter()
            .find(|r| r.constraint == id)
            .expect("constraint must appear in the report")
    };
    assert!(
        find(satisfied).max_abs_residual < 1e-12,
        "the satisfied distance must read ~0, got {}",
        find(satisfied).max_abs_residual
    );
    assert!(
        (find(violated).max_abs_residual - 7.0).abs() < 1e-9,
        "fixX(10) against x=3 must read 7, got {}",
        find(violated).max_abs_residual
    );
}

/// A system with every point pinned has no free parameters. Its constraints
/// are then trivially dependent, and the report says so rather than claiming
/// a clean `Solved`. Pinning this corner so it stays a stated contract.
#[test]
fn diagnostics_classify_a_fully_pinned_system() {
    let mut sys = GcsSystem::new();
    let a = fixed_pt(&mut sys, 0.0, 0.0);
    let b = fixed_pt(&mut sys, 3.0, 4.0);
    sys.add_constraint(Constraint::Distance(a, b, 5.0)).unwrap();

    let d = sys.solve_detailed(50, TOL).unwrap();
    assert!(d.converged, "the pinned geometry already satisfies it");
    assert_eq!(d.num_params, 0, "nothing is free to move");
    assert_eq!(d.rank, 0);
    assert_eq!(d.dof, 0);
    assert!(d.redundant);
    assert_eq!(d.classification, SolveClassification::Redundant);

    // With no constraints at all there is nothing to be dependent on.
    let mut empty = GcsSystem::new();
    fixed_pt(&mut empty, 1.0, 1.0);
    let e = empty.solve_detailed(50, TOL).unwrap();
    assert_eq!(e.num_equations, 0);
    assert!(!e.redundant);
    assert_eq!(e.classification, SolveClassification::Solved);
}

// ── Non-finite input rejection (A-2/A-3) ─────────────────────────────

/// A NaN-poisoned system must never report success: before the fix, the
/// `f64::max` residual fold dropped NaN and `solve` returned
/// `converged: true, max_residual: 0.0, iterations: 0`.
#[test]
fn nan_point_never_reports_convergence() {
    // The entry points reject NaN outright now, but belt-and-braces: even a
    // system whose NaN arrives by another route (e.g. deserialized state)
    // must fail the convergence test rather than pass it.
    let mut sys = GcsSystem::new();
    let err = sys.add_point(PointData {
        x: f64::NAN,
        y: 0.0,
        fixed: false,
    });
    assert!(
        matches!(err, Err(crate::SketchError::InvalidValue)),
        "NaN point must be rejected, got {err:?}"
    );

    // Solver-level: a NaN residual must propagate, not fold to zero.
    let mut params = vec![f64::NAN];
    let result = crate::gcs::solver::solve_dogleg(
        &mut params,
        &|_: &[f64]| vec![f64::NAN],
        &|_: &[f64]| vec![1.0],
        1,
        100,
        1e-10,
    );
    assert!(
        !result.converged,
        "NaN residual must not converge: {result:?}"
    );
    assert!(
        result.max_residual.is_nan(),
        "NaN residual must surface as NaN, got {}",
        result.max_residual
    );

    // A fixed-point NaN target is rejected at the constraint boundary.
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 0.0, 0.0);
    let err = sys.add_constraint(Constraint::FixX(p, f64::NAN));
    assert!(
        matches!(err, Err(crate::SketchError::InvalidValue)),
        "NaN FixX target must be rejected, got {err:?}"
    );
}

/// Non-finite and out-of-domain values are rejected at every value-carrying
/// entry point, not just `CircleRadius`.
#[test]
fn non_finite_values_rejected_at_entry() {
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 0.0, 0.0);
    let q = free_pt(&mut sys, 1.0, 0.0);
    let line = sys.add_line(p, q).unwrap();
    let circ = sys.add_circle(p, 1.0).unwrap();
    let arc = sys.add_arc(p, p, q).unwrap();

    // Points: NaN and infinite coordinates.
    for (x, y) in [
        (f64::NAN, 0.0),
        (0.0, f64::NAN),
        (f64::INFINITY, 0.0),
        (0.0, f64::NEG_INFINITY),
    ] {
        assert!(
            matches!(
                sys.add_point(PointData { x, y, fixed: false }),
                Err(crate::SketchError::InvalidValue)
            ),
            "point ({x}, {y}) must be rejected"
        );
    }

    // Circles: NaN, infinite, zero, and negative radii.
    for radius in [f64::NAN, f64::INFINITY, 0.0, -2.0] {
        assert!(
            matches!(
                sys.add_circle(p, radius),
                Err(crate::SketchError::InvalidValue)
            ),
            "radius {radius} must be rejected"
        );
    }

    // Constraints: NaN and infinite scalar arguments.
    let bad_values = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
    for v in bad_values {
        assert!(
            matches!(
                sys.add_constraint(Constraint::Distance(p, q, v)),
                Err(crate::SketchError::InvalidValue)
            ),
            "Distance({v}) must be rejected"
        );
        assert!(
            matches!(
                sys.add_constraint(Constraint::FixX(p, v)),
                Err(crate::SketchError::InvalidValue)
            ),
            "FixX({v}) must be rejected"
        );
        assert!(
            matches!(
                sys.add_constraint(Constraint::FixY(p, v)),
                Err(crate::SketchError::InvalidValue)
            ),
            "FixY({v}) must be rejected"
        );
        assert!(
            matches!(
                sys.add_constraint(Constraint::PointLineDistance(p, line, v)),
                Err(crate::SketchError::InvalidValue)
            ),
            "PointLineDistance({v}) must be rejected"
        );
        assert!(
            matches!(
                sys.add_constraint(Constraint::Angle(line, line, v)),
                Err(crate::SketchError::InvalidValue)
            ),
            "Angle({v}) must be rejected"
        );
        assert!(
            matches!(
                sys.add_constraint(Constraint::ArcLength(arc, v)),
                Err(crate::SketchError::InvalidValue)
            ),
            "ArcLength({v}) must be rejected"
        );
        assert!(
            matches!(
                sys.add_constraint(Constraint::CircleRadius(circ, v)),
                Err(crate::SketchError::InvalidValue)
            ),
            "CircleRadius({v}) must be rejected"
        );
    }
    // CircleRadius additionally rejects non-positive finite values.
    assert!(
        matches!(
            sys.add_constraint(Constraint::CircleRadius(circ, 0.0)),
            Err(crate::SketchError::InvalidValue)
        ),
        "CircleRadius(0) must be rejected"
    );
    assert!(
        matches!(
            sys.add_constraint(Constraint::CircleRadius(circ, -1.0)),
            Err(crate::SketchError::InvalidValue)
        ),
        "CircleRadius(-1) must be rejected"
    );

    // Finite values still pass validation.
    sys.add_constraint(Constraint::Distance(p, q, 2.0)).unwrap();
    sys.add_constraint(Constraint::FixX(p, 0.0)).unwrap();
    sys.add_constraint(Constraint::Angle(line, line, 0.5))
        .unwrap();
    sys.add_constraint(Constraint::ArcLength(arc, 1.0)).unwrap();
}

// ── PERF-S02 dense-vs-component agreement ────────────────────────────────
// Each test solves one clone through the legacy dense loop (`solve_dense`
// plus dense DOF/residual measurement via `dense_detailed` below, the pre-S02
// behavior bit-for-bit) and another through the public entry points, which
// take the component path whenever the system genuinely splits. Agreement is
// exact on contracted outcomes and tolerance-based on geometry (analytic
// oracles); it is deliberately NOT bitwise on multi-component solutions or
// iteration counts (per-component trust regions may traverse different
// iterates — see `components.rs`). Single-component systems assert bitwise
// equality.

const MAX_ITER: usize = 100;

fn build_independent_under(n_params: usize) -> GcsSystem {
    let mut sys = GcsSystem::new();
    for i in 0..n_params / 2 {
        let ax = 10.0 * i as f64;
        let anchor = fixed_pt(&mut sys, ax, 0.0);
        let free = free_pt(&mut sys, ax + 1.0, 1.0);
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
    }
    sys
}

fn build_independent_solved(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_under(n_params);
    let mut ids: Vec<PointId> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for chunk in ids.chunks(2) {
        sys.add_constraint(Constraint::FixY(chunk[1], 4.0)).unwrap();
    }
    sys
}

fn build_coupled_chain(n_params: usize) -> GcsSystem {
    let n_pts = n_params / 2 + 1;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(n_pts);
    pts.push(fixed_pt(&mut sys, 0.0, 0.0));
    for i in 1..n_pts {
        pts.push(free_pt(&mut sys, i as f64, 0.5 * f64::from((i % 2) as u8)));
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
    }
    sys
}

fn build_redundant(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_solved(n_params);
    let mut ids: Vec<PointId> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for chunk in ids.chunks(2) {
        sys.add_constraint(Constraint::Distance(chunk[0], chunk[1], 5.0))
            .unwrap();
    }
    sys
}

fn build_inconsistent(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_under(n_params);
    let mut ids: Vec<PointId> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    sys.add_constraint(Constraint::Distance(ids[0], ids[1], 6.0))
        .unwrap();
    sys
}

/// True dense `solve_detailed`, assembled from the legacy pieces.
///
/// Mirrors `solve_detailed_counted`'s structure — dense solve, dense DOF, fresh
/// residuals, whole-call rollback — so the comparison target is exactly the
/// pre-S02 contracted behavior, not a reimplementation with its own drift.
fn dense_detailed(sys: &mut GcsSystem, max_iterations: usize, tolerance: f64) -> SolveDiagnostics {
    sys.rebuild_if_dirty();
    let before = sys.extract_params();
    let (result, _, _) = sys.solve_dense(max_iterations, tolerance, false).unwrap();

    let n = sys.param_map.len();
    let m: usize = sys
        .constraints
        .iter()
        .map(|(_, e)| super::super::constraint::residual_count(&e.constraint))
        .sum();
    let analysis = if n == 0 || m == 0 {
        sys.dof().unwrap()
    } else {
        let snap = sys.build_snapshot();
        let jac = sys.jacobian_for_snapshot(&snap, m, n);
        dof::analyze(&jac, m, n)
    };
    let (residuals, internal_max) = sys.constraint_residuals();
    let rolled_back = !result.converged;
    let published_max_residual = if rolled_back {
        sys.write_params(&before);
        sys.current_max_residual()
    } else {
        fold_max_residual(&residuals)
    };
    SolveDiagnostics {
        converged: result.converged,
        iterations: result.iterations,
        max_residual: result.max_residual,
        published_max_residual,
        dof: analysis.dof,
        rank: analysis.rank,
        num_params: analysis.num_params,
        num_equations: analysis.num_equations,
        residuals,
        internal_max_residual: internal_max,
        rolled_back,
        redundant: analysis.rank < analysis.num_equations,
        classification: classify_solve(
            result.converged,
            analysis.dof,
            analysis.rank,
            analysis.num_equations,
        ),
    }
}

fn ordered_points(sys: &GcsSystem) -> Vec<PointId> {
    let mut ids: Vec<PointId> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    ids
}

fn pair_distance(sys: &GcsSystem, a: PointId, b: PointId) -> f64 {
    let pa = sys.point(a).unwrap();
    let pb = sys.point(b).unwrap();
    (pa.x - pb.x).hypot(pa.y - pb.y)
}

/// Contracted outcomes must match exactly; geometry is checked by callers.
fn assert_contracted_eq(dense: &SolveDiagnostics, comp: &SolveDiagnostics, what: &str) {
    assert_eq!(dense.converged, comp.converged, "{what}: converged");
    assert_eq!(dense.dof, comp.dof, "{what}: dof");
    assert_eq!(dense.rank, comp.rank, "{what}: rank");
    assert_eq!(dense.num_params, comp.num_params, "{what}: num_params");
    assert_eq!(
        dense.num_equations, comp.num_equations,
        "{what}: num_equations"
    );
    assert_eq!(dense.redundant, comp.redundant, "{what}: redundant");
    assert_eq!(
        dense.classification, comp.classification,
        "{what}: classification"
    );
    assert_eq!(dense.rolled_back, comp.rolled_back, "{what}: rolled_back");
    assert_eq!(
        dense.residuals.len(),
        comp.residuals.len(),
        "{what}: report length"
    );
}

#[test]
fn single_component_stays_bitwise_dense() {
    // The coupled chain never splits: public entry points must reproduce the
    // dense loop bit-for-bit, including iterations and published geometry.
    for n in [10_usize, 100] {
        let mut via_public = build_coupled_chain(n);
        let det = via_public.solve_detailed(MAX_ITER, TOL).unwrap();
        let mut via_dense = build_coupled_chain(n);
        let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
        assert!(det.converged && expect.converged);
        assert_eq!(det.iterations, expect.iterations, "n={n}: iterations");
        assert_eq!(
            det.max_residual.to_bits(),
            expect.max_residual.to_bits(),
            "n={n}: max_residual"
        );
        assert_contracted_eq(&expect, &det, "chain");
        for (a, b) in ordered_points(&via_public)
            .iter()
            .zip(ordered_points(&via_dense).iter())
        {
            let pa = via_public.point(*a).unwrap();
            let pb = via_dense.point(*b).unwrap();
            assert_eq!(pa.x.to_bits(), pb.x.to_bits(), "n={n}: x bits");
            assert_eq!(pa.y.to_bits(), pb.y.to_bits(), "n={n}: y bits");
        }
    }
}

#[test]
fn independent_solved_matches_dense() {
    for n in [10_usize, 100] {
        let mut via_dense = build_independent_solved(n);
        let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
        let mut via_comp = build_independent_solved(n);
        let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
        assert!(expect.converged && got.converged, "n={n}");
        assert_contracted_eq(&expect, &got, "solved");
        assert!(got.max_residual < TOL && expect.max_residual < TOL);
        // Analytic oracle on both: every pair at distance 5, free y pinned.
        for (sys, tag) in [(&via_dense, "dense"), (&via_comp, "comp")] {
            for chunk in ordered_points(sys).chunks(2) {
                let dist = pair_distance(sys, chunk[0], chunk[1]);
                assert!((dist - 5.0).abs() <= 1e-6, "n={n} {tag}: dist {dist} != 5");
                let y = sys.point(chunk[1]).unwrap().y;
                assert!((y - 4.0).abs() <= 1e-8, "n={n} {tag}: y {y} != 4");
            }
        }
    }
}

#[test]
fn independent_under_matches_dense() {
    // Underconstrained pairs admit a continuum of solutions: agree on the
    // contracted outcomes plus the constrained quantity (pair distance), not
    // on unconstrained angles.
    let mut via_dense = build_independent_under(20);
    let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
    let mut via_comp = build_independent_under(20);
    let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(expect.converged && got.converged);
    assert_contracted_eq(&expect, &got, "under");
    for (sys, tag) in [(&via_dense, "dense"), (&via_comp, "comp")] {
        for chunk in ordered_points(sys).chunks(2) {
            let dist = pair_distance(sys, chunk[0], chunk[1]);
            assert!((dist - 5.0).abs() <= 1e-6, "{tag}: dist {dist} != 5");
        }
    }
}

#[test]
fn redundant_matches_dense() {
    let mut via_dense = build_redundant(20);
    let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
    let mut via_comp = build_redundant(20);
    let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(expect.converged && got.converged);
    assert_contracted_eq(&expect, &got, "redundant");
    for (sys, tag) in [(&via_dense, "dense"), (&via_comp, "comp")] {
        for chunk in ordered_points(sys).chunks(2) {
            let dist = pair_distance(sys, chunk[0], chunk[1]);
            assert!((dist - 5.0).abs() <= 1e-6, "{tag}: dist {dist} != 5");
        }
    }
}

#[test]
fn inconsistent_matches_dense_with_whole_call_rollback() {
    let mut via_dense = build_inconsistent(20);
    let expect = dense_detailed(&mut via_dense, 50, TOL);
    let mut via_comp = build_inconsistent(20);
    let before: Vec<(f64, f64)> = ordered_points(&via_comp)
        .iter()
        .map(|id| {
            let p = via_comp.point(*id).unwrap();
            (p.x, p.y)
        })
        .collect();
    let got = via_comp.solve_detailed(50, TOL).unwrap();
    assert!(!expect.converged && !got.converged);
    assert_contracted_eq(&expect, &got, "inconsistent");
    // Whole-call rollback: converged siblings are restored with the failure.
    let restored: Vec<(f64, f64)> = ordered_points(&via_comp)
        .iter()
        .map(|id| {
            let p = via_comp.point(*id).unwrap();
            (p.x, p.y)
        })
        .collect();
    assert_eq!(before, restored, "rollback restores every component");
    // Plain solve still publishes (per-component best attempts, like the
    // dense loop's global best attempt).
    let mut via_plain = build_inconsistent(20);
    let r = via_plain.solve(50, TOL).unwrap();
    assert!(!r.converged);
}

#[test]
fn mixed_coordinate_scales_agree_on_rank() {
    // One unit-scale pair beside one 1e3-scale pair: the global rank policy
    // must not re-rank either block when counting separately.
    let build = || {
        let mut sys = GcsSystem::new();
        let a0 = fixed_pt(&mut sys, 0.0, 0.0);
        let f0 = free_pt(&mut sys, 1.0, 1.0);
        sys.add_constraint(Constraint::Distance(a0, f0, 5.0))
            .unwrap();
        sys.add_constraint(Constraint::FixY(f0, 4.0)).unwrap();
        let a1 = fixed_pt(&mut sys, 1e4, 1e4);
        let f1 = free_pt(&mut sys, 1e4 + 1e3, 1e4 + 1e3);
        sys.add_constraint(Constraint::Distance(a1, f1, 5000.0))
            .unwrap();
        sys.add_constraint(Constraint::FixY(f1, 1e4 + 4000.0))
            .unwrap();
        sys
    };
    let mut via_dense = build();
    let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
    let mut via_comp = build();
    let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(expect.converged && got.converged);
    assert_contracted_eq(&expect, &got, "mixed scales");
}

#[test]
fn near_singular_degenerate_row_agrees() {
    // Distance(p, p) is an all-zero Jacobian row at every iterate; FixX on the
    // same point keeps it structurally joined. Both paths must report the
    // same unsatisfiable, redundant outcome.
    let build = || {
        let mut sys = GcsSystem::new();
        let p = free_pt(&mut sys, 1.0, 1.0);
        sys.add_constraint(Constraint::Distance(p, p, 1.0)).unwrap();
        sys.add_constraint(Constraint::FixX(p, 1.0)).unwrap();
        sys
    };
    let mut via_dense = build();
    let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
    let mut via_comp = build();
    let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(!expect.converged && !got.converged);
    assert_contracted_eq(&expect, &got, "near-singular");
}

#[test]
fn empty_system_tolerance_agrees_between_solve_apis() {
    for tolerance in [-1.0, 0.0, TOL] {
        let plain = GcsSystem::new().solve(0, tolerance).unwrap();
        let (detailed, counts) = GcsSystem::new()
            .solve_detailed_counted(0, tolerance)
            .unwrap();
        assert_eq!(plain.converged, tolerance > 0.0);
        assert_eq!(detailed.converged, plain.converged);
        assert_eq!(detailed.iterations, plain.iterations);
        assert_eq!(
            detailed.max_residual.to_bits(),
            plain.max_residual.to_bits()
        );
        assert_eq!(detailed.rolled_back, !plain.converged);
        assert_eq!(counts.solver.residual_evals, 1);
        assert_eq!(
            detailed.classification,
            if plain.converged {
                SolveClassification::Solved
            } else {
                SolveClassification::Unsatisfied
            }
        );
    }
}

#[test]
fn zero_iteration_calls_agree() {
    let mut via_dense = build_independent_solved(10);
    let expect = dense_detailed(&mut via_dense, 0, TOL);
    let mut via_comp = build_independent_solved(10);
    let got = via_comp.solve_detailed(0, TOL).unwrap();
    assert!(!expect.converged && !got.converged);
    assert_eq!(expect.iterations, 0);
    assert_eq!(got.iterations, 0);
    assert_contracted_eq(&expect, &got, "zero-iteration");
    assert_eq!(
        expect.max_residual.to_bits(),
        got.max_residual.to_bits(),
        "untouched state reads identically"
    );
}

#[test]
fn fixed_only_and_isolated_params_agree() {
    // Pinned constraint plus an isolated free point beside a normal pair.
    let build = || {
        let mut sys = build_independent_under(4);
        let pinned = fixed_pt(&mut sys, 7.0, 3.0);
        sys.add_constraint(Constraint::FixX(pinned, 7.0)).unwrap();
        free_pt(&mut sys, 100.0, 100.0);
        sys
    };
    let mut via_dense = build();
    let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
    let mut via_comp = build();
    let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(expect.converged && got.converged);
    assert_contracted_eq(&expect, &got, "pinned+free");
    // The isolated point never moves on either path.
    for (sys, tag) in [(&via_dense, "dense"), (&via_comp, "comp")] {
        let ids = ordered_points(sys);
        let p = sys.point(*ids.last().unwrap()).unwrap();
        assert!(
            (p.x - 100.0).abs() <= 1e-12 && (p.y - 100.0).abs() <= 1e-12,
            "{tag}: isolated point moved"
        );
    }
}

#[test]
fn edits_between_solves_agree() {
    let mut via_dense = build_independent_solved(10);
    let mut via_comp = build_independent_solved(10);
    for _ in 0..3 {
        let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
        let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
        assert!(expect.converged && got.converged);
        assert_contracted_eq(&expect, &got, "edit round");
        for sys in [&mut via_dense, &mut via_comp] {
            let ids = ordered_points(sys);
            let slot = sys.point_mut(ids[1]).unwrap();
            slot.x += 0.5;
            slot.y += 0.25;
        }
    }
}

#[test]
fn shared_line_circle_arc_coupling_agrees() {
    // Constraints coupled through line endpoints, a shared circle center and
    // an arc triple (with its internal tie), beside an unrelated pair. The
    // coupled block starts near its consistent state (unit arc from (1,0) to
    // (0,1)) so both paths converge it from the same perturbed start.
    let build = || {
        let mut sys = GcsSystem::new();
        let p0 = fixed_pt(&mut sys, 0.0, 0.0);
        let p1 = free_pt(&mut sys, 1.2, 0.3);
        let p2 = free_pt(&mut sys, 0.2, 1.1);
        let line = sys.add_line(p0, p1).unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
        sys.add_constraint(Constraint::FixY(p1, 0.0)).unwrap();
        let circ = sys.add_circle(p0, 1.0).unwrap();
        sys.add_constraint(Constraint::PointOnCircle(p2, circ))
            .unwrap();
        let arc = sys.add_arc(p0, p1, p2).unwrap();
        sys.add_constraint(Constraint::ArcLength(arc, std::f64::consts::FRAC_PI_2))
            .unwrap();
        // Unrelated pair.
        let a = fixed_pt(&mut sys, 50.0, 0.0);
        let f = free_pt(&mut sys, 51.0, 1.0);
        sys.add_constraint(Constraint::Distance(a, f, 5.0)).unwrap();
        sys
    };
    let mut via_dense = build();
    let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
    let mut via_comp = build();
    let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(expect.converged && got.converged);
    assert_contracted_eq(&expect, &got, "shared entities");
}

#[test]
fn arc_system_with_internal_tie_agrees() {
    let build = || {
        let mut sys = GcsSystem::new();
        let center = fixed_pt(&mut sys, 0.0, 0.0);
        let start = free_pt(&mut sys, 2.0, 0.0);
        let end = free_pt(&mut sys, 0.0, 1.0);
        let arc = sys.add_arc(center, start, end).unwrap();
        sys.add_constraint(Constraint::PointOnArc(start, arc))
            .unwrap();
        sys.add_constraint(Constraint::FixX(start, 2.0)).unwrap();
        sys
    };
    let mut via_dense = build();
    let expect = dense_detailed(&mut via_dense, MAX_ITER, TOL);
    let mut via_comp = build();
    let got = via_comp.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(expect.converged && got.converged);
    assert_contracted_eq(&expect, &got, "arc");
}

// ── B75 ellipse entity lifecycle ──────────────────────────────────

#[test]
fn add_ellipse_validates_inputs() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    // Valid.
    let e = sys.add_ellipse(c, 3.0, 2.0, 0.5).unwrap();
    assert_eq!(sys.ellipse_count(), 1);
    let data = sys.ellipse(e).unwrap();
    assert_eq!((data.a, data.b, data.angle), (3.0, 2.0, 0.5));
    // Bad center.
    let orphan = sys
        .add_point(PointData {
            x: 9.0,
            y: 9.0,
            fixed: true,
        })
        .unwrap();
    let _ = sys.remove_point(orphan).unwrap();
    // Non-positive / non-finite axes rejected.
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(sys.add_ellipse(c, bad, 1.0, 0.0).is_err(), "a={bad}");
        assert!(sys.add_ellipse(c, 1.0, bad, 0.0).is_err(), "b={bad}");
    }
    // Non-finite angle rejected; any finite angle (even huge) accepted.
    assert!(sys.add_ellipse(c, 1.0, 1.0, f64::NAN).is_err());
    assert!(sys.add_ellipse(c, 1.0, 1.0, 1e6).is_ok());
}

#[test]
fn add_ellipse_rejects_stale_center() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 3.0, 2.0, 0.0).unwrap();
    // Center is in use by the ellipse.
    assert!(sys.remove_point(c).is_err());
    // Free the center by removing the ellipse first.
    sys.remove_ellipse(e).unwrap();
    assert_eq!(sys.ellipse_count(), 0);
    assert!(sys.remove_point(c).is_ok());
}

#[test]
fn remove_ellipse_refuses_referenced_entity() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let p = sys
        .add_point(PointData {
            x: 3.0,
            y: 0.0,
            fixed: false,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 3.0, 2.0, 0.0).unwrap();
    let cid = sys
        .add_constraint(Constraint::PointOnEllipse(p, e))
        .unwrap();
    // Referenced: refusal, and the constraint still solves afterwards.
    assert!(sys.remove_ellipse(e).is_err());
    // Removing the constraint releases the ellipse; the handle never dangles.
    sys.remove_constraint(cid).unwrap();
    sys.remove_ellipse(e).unwrap();
    assert!(sys.ellipse(e).is_none());
    // Double remove is a typed stale-handle error, not a panic.
    assert!(sys.remove_ellipse(e).is_err());
}

#[test]
fn ellipse_mut_edits_between_solves() {
    // Drag sequence: solve, mutate the ellipse directly, re-solve.
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 3.0, 2.0, 0.0).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 5.0))
        .unwrap();
    let r = sys.solve(100, TOL).unwrap();
    assert!(r.converged);
    assert!((sys.ellipse(e).unwrap().a - 5.0).abs() < 1e-9);
    // Drag: reshape + rotate between solves.
    {
        let data = sys.ellipse_mut(e).unwrap();
        data.b = 4.0;
        data.angle = 0.25;
    }
    sys.add_constraint(Constraint::EllipseAxisB(e, 4.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.25))
        .unwrap();
    let r = sys.solve(100, TOL).unwrap();
    assert!(r.converged);
    let data = sys.ellipse(e).unwrap();
    assert!((data.a - 5.0).abs() < 1e-9, "a={}", data.a);
    assert!((data.b - 4.0).abs() < 1e-9, "b={}", data.b);
    assert!((data.angle - 0.25).abs() < 1e-9, "phi={}", data.angle);
}

#[test]
fn ellipse_free_dof_counts() {
    // One free ellipse: 5 parameters (cx, cy, a, b, phi), no equations.
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        })
        .unwrap();
    sys.add_ellipse(c, 3.0, 2.0, 0.0).unwrap();
    let dof = sys.dof().unwrap();
    assert_eq!(
        (dof.dof, dof.rank, dof.num_params, dof.num_equations),
        (5, 0, 5, 0)
    );
    // Fixed center: 3 left. Driving all three scalars closes the system.
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 1.0,
            y: 1.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 3.0, 2.0, 0.0).unwrap();
    let dof = sys.dof().unwrap();
    assert_eq!((dof.dof, dof.num_params, dof.num_equations), (3, 3, 0));
    sys.add_constraint(Constraint::EllipseAxisA(e, 3.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e, 2.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.0))
        .unwrap();
    let d = sys.solve_detailed(100, TOL).unwrap();
    assert!(d.converged);
    assert_eq!(d.classification, SolveClassification::Solved);
    assert_eq!(d.dof, 0);
}

// ── B75 ellipse solves ────────────────────────────────────────────

#[test]
fn ellipse_axes_and_angle_drive_to_targets() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 1.0, 1.0, 0.0).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 6.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e, 2.5))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.7))
        .unwrap();
    let r = sys.solve(200, TOL).unwrap();
    assert!(r.converged, "max_r={}", r.max_residual);
    let data = sys.ellipse(e).unwrap();
    assert!((data.a - 6.0).abs() < 1e-9, "a={}", data.a);
    assert!((data.b - 2.5).abs() < 1e-9, "b={}", data.b);
    assert!((data.angle - 0.7).abs() < 1e-9, "phi={}", data.angle);
}

#[test]
fn point_on_ellipse_pulls_free_point_onto_curve() {
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 4.0, 2.0, 0.0).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 4.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e, 2.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.0))
        .unwrap();
    let p = sys
        .add_point(PointData {
            x: 5.0,
            y: 5.0,
            fixed: false,
        })
        .unwrap();
    // Pin x off the axis tip (x=2): the implicit residual is linear in y
    // there (gradient y/2 ≈ 0.87), so the solve pins y tightly. Exactly at
    // the tip the residual goes quadratic (y²/4) and a 1e-10 residual
    // tolerance only pins y to ~2e-5 — geometry, not solver error.
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.add_constraint(Constraint::PointOnEllipse(p, e))
        .unwrap();
    let r = sys.solve(200, TOL).unwrap();
    assert!(r.converged, "max_r={}", r.max_residual);
    // (2/4)² + (y/2)² = 1 → y = √3 (positive branch from the y=5 start).
    let pt = sys.point(p).unwrap();
    assert!((pt.x - 2.0).abs() < 1e-9, "x={}", pt.x);
    assert!((pt.y - 3.0f64.sqrt()).abs() < 1e-8, "y={}", pt.y);
}

#[test]
fn tangent_line_ellipse_placement_solves() {
    // Fixed axis-aligned ellipse; a two-point line through a free contact
    // point becomes tangent there. Contact composition: point-on-ellipse +
    // point-on-line (signed distance 0) + tangency direction.
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 4.0, 2.0, 0.0).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 4.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e, 2.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.0))
        .unwrap();
    // Contact starts near the first-quadrant arc; line starts horizontal-ish.
    let q = sys
        .add_point(PointData {
            x: 3.0,
            y: 1.5,
            fixed: false,
        })
        .unwrap();
    let a = sys
        .add_point(PointData {
            x: -6.0,
            y: 6.0,
            fixed: true,
        })
        .unwrap();
    let b = sys
        .add_point(PointData {
            x: 6.0,
            y: 6.5,
            fixed: false,
        })
        .unwrap();
    let line = sys.add_line(a, b).unwrap();
    sys.add_constraint(Constraint::PointOnEllipse(q, e))
        .unwrap();
    sys.add_constraint(Constraint::PointLineDistance(q, line, 0.0))
        .unwrap();
    sys.add_constraint(Constraint::TangentLineEllipse(line, e, q))
        .unwrap();
    let r = sys.solve(500, 1e-10).unwrap();
    assert!(r.converged, "max_r={}", r.max_residual);
    // Independent geometric verification (not the solver residual): contact
    // on the ellipse, contact on the line, line direction ⊥ gradient.
    let (qx, qy) = (sys.point(q).unwrap().x, sys.point(q).unwrap().y);
    assert!(((qx / 4.0) * (qx / 4.0) + (qy / 2.0) * (qy / 2.0) - 1.0).abs() < 1e-8);
    let (ax, ay) = (sys.point(a).unwrap().x, sys.point(a).unwrap().y);
    let (bx, by) = (sys.point(b).unwrap().x, sys.point(b).unwrap().y);
    let (lx, ly) = (bx - ax, by - ay);
    let len = lx.hypot(ly);
    assert!((lx * (qy - ay) - ly * (qx - ax)).abs() / len < 1e-8);
    let (gx, gy) = (2.0 * qx / 16.0, 2.0 * qy / 4.0);
    assert!((lx * gx + ly * gy).abs() / (len * gx.hypot(gy)) < 1e-8);
}

#[test]
fn concentric_and_equal_radii_couple_ellipses() {
    let mut sys = GcsSystem::new();
    let c1 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let c2 = sys
        .add_point(PointData {
            x: 9.0,
            y: 9.0,
            fixed: false,
        })
        .unwrap();
    let e1 = sys.add_ellipse(c1, 5.0, 3.0, 0.2).unwrap();
    let e2 = sys.add_ellipse(c2, 1.0, 1.0, 0.0).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e1, 5.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e1, 3.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e1, 0.2))
        .unwrap();
    sys.add_constraint(Constraint::ConcentricEllipseEllipse(e1, e2))
        .unwrap();
    sys.add_constraint(Constraint::EqualEllipseRadii(e1, e2))
        .unwrap();
    let r = sys.solve(200, TOL).unwrap();
    assert!(r.converged, "max_r={}", r.max_residual);
    let p2 = sys.point(c2).unwrap();
    assert!(
        p2.x.abs() < 1e-9 && p2.y.abs() < 1e-9,
        "({},{})",
        p2.x,
        p2.y
    );
    let d2 = sys.ellipse(e2).unwrap();
    assert!(
        (d2.a - 5.0).abs() < 1e-9 && (d2.b - 3.0).abs() < 1e-9,
        "{d2:?}"
    );
}

#[test]
fn ellipse_couples_to_circle_and_arc_centers() {
    // Mixed-component system: ellipse concentric with a circle and an arc,
    // circle radius driven, arc radius tied — everything converges together.
    let mut sys = GcsSystem::new();
    let ce = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(ce, 4.0, 2.0, 0.3).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 4.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e, 2.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.3))
        .unwrap();
    let cc = sys
        .add_point(PointData {
            x: 7.0,
            y: -3.0,
            fixed: false,
        })
        .unwrap();
    let circ = sys.add_circle(cc, 1.0).unwrap();
    sys.add_constraint(Constraint::ConcentricEllipseCircle(e, circ))
        .unwrap();
    sys.add_constraint(Constraint::CircleRadius(circ, 2.0))
        .unwrap();
    let ca = sys
        .add_point(PointData {
            x: -4.0,
            y: 8.0,
            fixed: false,
        })
        .unwrap();
    let s0 = sys
        .add_point(PointData {
            x: 1.0,
            y: 0.0,
            fixed: false,
        })
        .unwrap();
    let s1 = sys
        .add_point(PointData {
            x: 0.0,
            y: 1.0,
            fixed: false,
        })
        .unwrap();
    let arc = sys.add_arc(ca, s0, s1).unwrap();
    sys.add_constraint(Constraint::ConcentricEllipseArc(e, arc))
        .unwrap();
    let r = sys.solve(300, TOL).unwrap();
    assert!(r.converged, "max_r={}", r.max_residual);
    for (id, name) in [(cc, "circle"), (ca, "arc")] {
        let p = sys.point(id).unwrap();
        assert!(
            p.x.abs() < 1e-8 && p.y.abs() < 1e-8,
            "{name}: ({},{})",
            p.x,
            p.y
        );
    }
    assert!((sys.circle(circ).unwrap().radius - 2.0).abs() < 1e-9);
}

#[test]
fn ellipse_redundant_and_inconsistent_systems() {
    // Redundant: duplicate axis drive on a fully-driven ellipse classifies
    // redundant, still solved. (A lone duplicate with free scalars left
    // would be underConstrained by the classify precedence — DOF first.)
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 3.0, 2.0, 0.0).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 3.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e, 2.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 3.0))
        .unwrap();
    let d = sys.solve_detailed(100, TOL).unwrap();
    assert!(d.converged);
    assert_eq!(d.classification, SolveClassification::Redundant);
    assert!(d.redundant);
    // Inconsistent: conflicting axis drives fail and roll back exactly.
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 3.0, 2.0, 0.1).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 3.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 9.0))
        .unwrap();
    let d = sys.solve_detailed(200, TOL).unwrap();
    assert!(!d.converged);
    assert_eq!(d.classification, SolveClassification::Unsatisfied);
    assert!(d.rolled_back);
    let data = sys.ellipse(e).unwrap();
    assert!(
        (data.a - 3.0).abs() < 1e-15,
        "rollback must restore a, got {}",
        data.a
    );
    assert!(
        (data.b - 2.0).abs() < 1e-15,
        "rollback must restore b, got {}",
        data.b
    );
    assert!(
        (data.angle - 0.1).abs() < 1e-15,
        "rollback must restore phi, got {}",
        data.angle
    );
}

#[test]
fn ellipse_translated_and_rotated_solves() {
    // Same sketch at unit scale and translated by (1e3, -1e3) with a
    // non-axis-aligned orientation: translation must not change the outcome.
    for (ox, oy) in [(0.0, 0.0), (1e3, -1e3)] {
        let mut sys = GcsSystem::new();
        let c = sys
            .add_point(PointData {
                x: ox,
                y: oy,
                fixed: true,
            })
            .unwrap();
        let e = sys.add_ellipse(c, 1.0, 1.0, 0.0).unwrap();
        sys.add_constraint(Constraint::EllipseAxisA(e, 6.0))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAxisB(e, 2.5))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAngle(e, 0.7))
            .unwrap();
        let r = sys.solve(300, TOL).unwrap();
        assert!(r.converged, "offset ({ox},{oy}): max_r={}", r.max_residual);
        let data = sys.ellipse(e).unwrap();
        assert!((data.a - 6.0).abs() < 1e-9, "a={}", data.a);
        assert!((data.angle - 0.7).abs() < 1e-9, "phi={}", data.angle);
    }
}

#[test]
fn ellipse_mixed_scale_components_stay_independent() {
    // Two disconnected ellipse systems at 1e-3 and 1e3 in one GcsSystem:
    // per-component trust regions keep both converging.
    let mut sys = GcsSystem::new();
    let mut driven = Vec::new();
    for (s, ox) in [(1e-3, 0.0), (1e3, 50.0 * 1e3)] {
        let c = sys
            .add_point(PointData {
                x: ox,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let e = sys.add_ellipse(c, s, s, 0.0).unwrap();
        sys.add_constraint(Constraint::EllipseAxisA(e, 6.0 * s))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAxisB(e, 2.5 * s))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAngle(e, 0.7))
            .unwrap();
        driven.push((e, s));
    }
    let r = sys.solve(300, 1e-10).unwrap();
    assert!(r.converged, "max_r={}", r.max_residual);
    for (e, s) in driven {
        let data = sys.ellipse(e).unwrap();
        assert!((data.a - 6.0 * s).abs() < 1e-9 * s.max(1.0), "a={}", data.a);
        assert!((data.angle - 0.7).abs() < 1e-9, "phi={}", data.angle);
    }
}

#[test]
fn ellipse_drag_sequence_between_solves() {
    // Drag the center point between solves (the browser drag pattern):
    // each solve starts from the previous result and re-converges. The
    // center is solver-fixed so the drag sticks — direct mutation still
    // applies to fixed points (as does the `gcsSetPoint` binding) — while
    // the free on-curve point re-settles onto the moved ellipse.
    let mut sys = GcsSystem::new();
    let c = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let e = sys.add_ellipse(c, 4.0, 2.0, 0.0).unwrap();
    sys.add_constraint(Constraint::EllipseAxisA(e, 4.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAxisB(e, 2.0))
        .unwrap();
    sys.add_constraint(Constraint::EllipseAngle(e, 0.0))
        .unwrap();
    let p = sys
        .add_point(PointData {
            x: 4.0,
            y: 0.0,
            fixed: false,
        })
        .unwrap();
    sys.add_constraint(Constraint::PointOnEllipse(p, e))
        .unwrap();
    sys.add_constraint(Constraint::FixY(p, 0.0)).unwrap();
    for (cx, expected_px) in [(0.0, 4.0), (1.0, 5.0), (1.0, 5.0), (-2.0, 2.0)] {
        sys.point_mut(c).unwrap().x = cx;
        let r = sys.solve(200, TOL).unwrap();
        assert!(r.converged, "drag to {cx}: max_r={}", r.max_residual);
        let pt = sys.point(p).unwrap();
        assert!(
            (pt.x - expected_px).abs() < 1e-8,
            "drag to {cx}: x={}",
            pt.x
        );
    }
}

// The supported equal-circle chain is underdetermined and must fall back to
// dense storage. A small configurable budget proves refusal without exhausting
// memory; the same fixture under normal limits proves its rank and solve result.
fn budget_circle_chain(limits: GcsLimits, count: usize) -> GcsSystem {
    let mut sys = GcsSystem::with_limits(limits);
    let mut previous = None;
    for i in 0..count {
        let center = sys
            .add_point(PointData {
                x: 3.0 * i as f64,
                y: 0.0,
                fixed: false,
            })
            .unwrap();
        let circle = sys.add_circle(center, 2.0 + i as f64 / 100.0).unwrap();
        if let Some(prev) = previous {
            sys.add_constraint(Constraint::EqualRadiusCircleCircle(prev, circle))
                .unwrap();
        }
        previous = Some(circle);
    }
    sys
}

fn assert_resource_refusal<T: std::fmt::Debug>(result: Result<T, SketchError>, resource: &str) {
    assert!(
        matches!(result, Err(SketchError::ResourceLimitExceeded { resource: got, .. }) if got == resource)
    );
}

#[test]
fn dense_budget_checked_arithmetic_and_boundary() {
    let limits = GcsLimits {
        max_dense_bytes: 1120,
        max_dense_qr_work: 24,
    };
    // 3*(2*3 + 2*3) +16*(2+3 +2+3) doubles = 1568 bytes.
    assert_resource_refusal(limits.check_dimensions([(2, 3), (2, 3)]), "gcs_dense_bytes");
    assert!(limits.check_dimensions([(2, 3)]).is_ok());
    assert_resource_refusal(
        limits.check_dimensions([(usize::MAX, 2)]),
        "gcs_dense_bytes",
    );
    assert_resource_refusal(
        limits.check_dimensions([(usize::MAX, 0), (1, 0)]),
        "gcs_dense_bytes",
    );
    let exact = GcsLimits {
        max_dense_bytes: 784,
        max_dense_qr_work: 12,
    };
    assert!(exact.check_dimensions([(2, 3)]).is_ok());
    assert_resource_refusal(
        GcsLimits {
            max_dense_bytes: 783,
            ..exact
        }
        .check_dimensions([(2, 3)]),
        "gcs_dense_bytes",
    );
    assert_resource_refusal(
        GcsLimits {
            max_dense_qr_work: 11,
            ..exact
        }
        .check_dimensions([(2, 3)]),
        "gcs_dense_qr_work",
    );
}

#[test]
fn dense_budget_circle_chain_refuses_all_entries_atomically() {
    let mut sys = budget_circle_chain(
        GcsLimits {
            max_dense_bytes: 4096,
            ..GcsLimits::default()
        },
        130,
    );
    let before = format!("{:?}", sys);
    for _ in 0..2 {
        assert_resource_refusal(sys.solve(0, TOL), "gcs_dense_bytes");
        assert_resource_refusal(sys.solve(100, TOL), "gcs_dense_bytes");
        assert_resource_refusal(sys.solve_detailed(100, TOL), "gcs_dense_bytes");
        assert_resource_refusal(sys.dof(), "gcs_dense_bytes");
    }
    // The parameter cache is allowed to rebuild; all live entities, constraints
    // and their handles remain identical.
    sys.dirty = true;
    sys.param_map.clear();
    sys.param_index.clear();
    assert_eq!(format!("{:?}", sys), before);
    let mut control = budget_circle_chain(GcsLimits::default(), 130);
    let initial = control.dof().unwrap();
    assert_eq!(
        (
            initial.num_params,
            initial.num_equations,
            initial.rank,
            initial.dof
        ),
        (390, 129, 129, 261)
    );
    assert!(control.solve(100, TOL).unwrap().converged);
    assert!(control.solve_detailed(100, TOL).unwrap().converged);
    assert_eq!(control.dof().unwrap().rank, 129);
}

#[test]
fn dense_budget_aggregates_components_and_refuses_before_publication() {
    let mut sys = GcsSystem::with_limits(GcsLimits {
        max_dense_bytes: 600,
        ..GcsLimits::default()
    });
    let p = sys
        .add_point(PointData {
            x: 7.0,
            y: 9.0,
            fixed: false,
        })
        .unwrap();
    sys.add_constraint(Constraint::FixX(p, 2.0)).unwrap();
    sys.add_constraint(Constraint::FixY(p, 3.0)).unwrap();
    // Two independent scalar components each fit (280 bytes), aggregate 560.
    assert!(sys.solve(100, TOL).unwrap().converged);
    sys.point_mut(p).unwrap().x = 8.0;
    let q = sys
        .add_point(PointData {
            x: 4.0,
            y: 5.0,
            fixed: false,
        })
        .unwrap();
    sys.add_constraint(Constraint::FixX(q, 1.0)).unwrap();
    assert_resource_refusal(sys.solve_detailed(100, TOL), "gcs_dense_bytes");
    assert_eq!(sys.point(p).unwrap().x.to_bits(), 8.0_f64.to_bits());
    assert_eq!(sys.point(q).unwrap().x.to_bits(), 4.0_f64.to_bits());
    assert_eq!(sys.constraint_count(), 3);
}

#[test]
fn dense_budget_ellipse_chain_and_sparse_rank_miss_refuse() {
    let mut sys = GcsSystem::with_limits(GcsLimits {
        max_dense_bytes: 1024,
        ..GcsLimits::default()
    });
    let mut previous = None;
    for i in 0..5 {
        let center = sys
            .add_point(PointData {
                x: i as f64,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let ellipse = sys.add_ellipse(center, 4.0 + i as f64, 2.0, 0.1).unwrap();
        if let Some(prev) = previous {
            sys.add_constraint(Constraint::EqualEllipseRadii(prev, ellipse))
                .unwrap();
        }
        previous = Some(ellipse);
    }
    let last = previous.unwrap();
    assert_resource_refusal(sys.solve(0, TOL), "gcs_dense_bytes");
    assert_resource_refusal(sys.dof(), "gcs_dense_bytes");
    assert_eq!(sys.ellipse(last).unwrap().a.to_bits(), 8.0_f64.to_bits());
    let mut rank_miss = budget_circle_chain(
        GcsLimits {
            max_dense_bytes: 4096,
            ..GcsLimits::default()
        },
        130,
    );
    // Add redundant rows so m >= n: sparse's structural entry condition can
    // pass, but numerical full rank cannot. Preflight still reserves fallback.
    let first = rank_miss.circles.iter().next().unwrap().0;
    for _ in 0..4 {
        rank_miss
            .add_constraint(Constraint::CircleRadius(first, 2.0))
            .unwrap();
    }
    assert_resource_refusal(rank_miss.solve_detailed(0, TOL), "gcs_dense_bytes");
}
