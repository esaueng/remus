use super::*;
use crate::gcs::diagnostics::SolveClassification;

const TOL: f64 = 1e-10;

#[test]
fn fix_xy_converges() {
    let mut sys = GcsSystem::new();
    let p = sys.add_point(PointData {
        x: 5.0,
        y: 7.0,
        fixed: false,
    });
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
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 0.5,
        y: 0.0,
        fixed: false,
    });
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
    let p0 = sys.add_point(PointData {
        x: 1.0,
        y: 2.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 3.0,
        y: 4.0,
        fixed: false,
    });
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
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 1.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 5.0,
        y: 3.0,
        fixed: false,
    });
    let l = sys.add_line(p0, p1).unwrap();
    sys.add_constraint(Constraint::Horizontal(l)).unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    assert!((sys.point(p1).unwrap().y - sys.point(p0).unwrap().y).abs() < TOL);
}

#[test]
fn vertical_line() {
    let mut sys = GcsSystem::new();
    let p0 = sys.add_point(PointData {
        x: 2.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 5.0,
        y: 7.0,
        fixed: false,
    });
    let l = sys.add_line(p0, p1).unwrap();
    sys.add_constraint(Constraint::Vertical(l)).unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    assert!((sys.point(p1).unwrap().x - sys.point(p0).unwrap().x).abs() < TOL);
}

#[test]
fn perpendicular_lines() {
    let mut sys = GcsSystem::new();
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: true,
    });
    let p2 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p3 = sys.add_point(PointData {
        x: 0.5,
        y: 0.5,
        fixed: false,
    });
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
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 1.0,
        y: 1.0,
        fixed: true,
    });
    let p2 = sys.add_point(PointData {
        x: 2.0,
        y: 0.0,
        fixed: true,
    });
    let p3 = sys.add_point(PointData {
        x: 3.0,
        y: 0.5,
        fixed: false,
    });
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

    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let p1 = sys.add_point(PointData {
        x: 25.0,
        y: 1.0,
        fixed: false,
    });
    let p2 = sys.add_point(PointData {
        x: 26.0,
        y: 18.0,
        fixed: false,
    });
    let p3 = sys.add_point(PointData {
        x: 1.0,
        y: 17.0,
        fixed: false,
    });

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
    let p = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });

    // Free point: 2 DOF
    let dof = sys.dof();
    assert_eq!(dof.dof, 2);

    // Fix X: 1 DOF
    let cx = sys.add_constraint(Constraint::FixX(p, 0.0)).unwrap();
    let dof = sys.dof();
    assert_eq!(dof.dof, 1);

    // Fix Y: 0 DOF
    sys.add_constraint(Constraint::FixY(p, 0.0)).unwrap();
    let dof = sys.dof();
    assert_eq!(dof.dof, 0);

    // Remove FixX: back to 1 DOF
    sys.remove_constraint(cx).unwrap();
    let dof = sys.dof();
    assert_eq!(dof.dof, 1);
}

#[test]
fn remove_point_in_use_fails() {
    let mut sys = GcsSystem::new();
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let p1 = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let _l = sys.add_line(p0, p1).unwrap();

    assert!(sys.remove_point(p0).is_err());
}

#[test]
fn remove_line_in_use_fails() {
    let mut sys = GcsSystem::new();
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let p1 = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let l = sys.add_line(p0, p1).unwrap();
    sys.add_constraint(Constraint::Horizontal(l)).unwrap();

    assert!(sys.remove_line(l).is_err());
}

#[test]
fn stale_constraint_handle() {
    let mut sys = GcsSystem::new();
    let p = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let c = sys.add_constraint(Constraint::FixX(p, 0.0)).unwrap();
    sys.remove_constraint(c).unwrap();
    assert!(sys.remove_constraint(c).is_err());
}

#[test]
fn solve_after_removal() {
    let mut sys = GcsSystem::new();
    let p = sys.add_point(PointData {
        x: 5.0,
        y: 7.0,
        fixed: false,
    });
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
    let p = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    sys.remove_point(p).unwrap();
    assert!(sys.add_constraint(Constraint::FixX(p, 0.0)).is_err());
}

#[test]
fn triangle_345() {
    let mut sys = GcsSystem::new();
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let p2 = sys.add_point(PointData {
        x: 0.5,
        y: 1.0,
        fixed: false,
    });

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
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: true,
    });
    sys.add_constraint(Constraint::Distance(p0, p1, 1.0))
        .unwrap();

    let result = sys.solve(100, TOL).unwrap();
    assert!(result.converged);
    assert_eq!(result.iterations, 0);
}

#[test]
fn add_arc_basic() {
    let mut sys = GcsSystem::new();
    let c = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let s = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let e = sys.add_point(PointData {
        x: 0.0,
        y: 1.0,
        fixed: false,
    });
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
    let c = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let s = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let e = sys.add_point(PointData {
        x: 0.0,
        y: 1.0,
        fixed: false,
    });
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
    let center = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let circ = sys.add_circle(center, 2.0).unwrap();
    let pt = sys.add_point(PointData {
        x: 3.0,
        y: 0.0,
        fixed: false,
    });
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
    let c = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let s = sys.add_point(PointData {
        x: 2.0,
        y: 0.0,
        fixed: true,
    });
    let e = sys.add_point(PointData {
        x: 0.0,
        y: 2.0,
        fixed: false,
    });
    let arc = sys.add_arc(c, s, e).unwrap();
    let pt = sys.add_point(PointData {
        x: 3.0,
        y: 3.0,
        fixed: false,
    });
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
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 2.0,
        y: 0.0,
        fixed: true,
    });
    let line = sys.add_line(p0, p1).unwrap();
    let c = sys.add_point(PointData {
        x: 2.0,
        y: 1.0,
        fixed: false,
    });
    let s = sys.add_point(PointData {
        x: 2.0,
        y: 0.0,
        fixed: false,
    });
    let e = sys.add_point(PointData {
        x: 3.0,
        y: 1.0,
        fixed: false,
    });
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
    let c1 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let s1 = sys.add_point(PointData {
        x: 2.0,
        y: 0.0,
        fixed: true,
    });
    let e1 = sys.add_point(PointData {
        x: 0.0,
        y: 2.0,
        fixed: false,
    });
    let arc1 = sys.add_arc(c1, s1, e1).unwrap();
    let c2 = sys.add_point(PointData {
        x: 5.0,
        y: 0.0,
        fixed: true,
    });
    let s2 = sys.add_point(PointData {
        x: 8.0,
        y: 0.0,
        fixed: false,
    });
    let e2 = sys.add_point(PointData {
        x: 5.0,
        y: 3.0,
        fixed: false,
    });
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
    let c1 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let s1 = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let e1 = sys.add_point(PointData {
        x: 0.0,
        y: 1.0,
        fixed: false,
    });
    let arc1 = sys.add_arc(c1, s1, e1).unwrap();
    let c2 = sys.add_point(PointData {
        x: 0.5,
        y: 0.5,
        fixed: false,
    });
    let s2 = sys.add_point(PointData {
        x: 2.5,
        y: 0.5,
        fixed: false,
    });
    let e2 = sys.add_point(PointData {
        x: 0.5,
        y: 2.5,
        fixed: false,
    });
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
    let p0 = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let p1 = sys.add_point(PointData {
        x: 4.0,
        y: 0.0,
        fixed: false,
    });
    let p2 = sys.add_point(PointData {
        x: 4.0,
        y: 2.0,
        fixed: false,
    });
    let p3 = sys.add_point(PointData {
        x: 0.0,
        y: 2.0,
        fixed: false,
    });
    // Two horizontal lines
    let bottom_line = sys.add_line(p0, p1).unwrap();
    let top_line = sys.add_line(p3, p2).unwrap();
    // Right semicircle: center at (4, 1), connecting p1 to p2
    let rc = sys.add_point(PointData {
        x: 4.0,
        y: 1.0,
        fixed: false,
    });
    let right_arc = sys.add_arc(rc, p1, p2).unwrap();
    // Left semicircle: center at (0, 1), connecting p3 to p0
    let lc = sys.add_point(PointData {
        x: 0.0,
        y: 1.0,
        fixed: false,
    });
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
    let c = sys.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let s = sys.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: true,
    });
    // End point starts off-circle — solver should move it onto the circle
    // (start is fixed, so the dynamic radius is pinned at 1.0)
    let e = sys.add_point(PointData {
        x: 0.0,
        y: 1.5,
        fixed: false,
    });
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
    let c2 = sys2.add_point(PointData {
        x: 0.0,
        y: 0.0,
        fixed: true,
    });
    let s2 = sys2.add_point(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let e2 = sys2.add_point(PointData {
        x: 0.0,
        y: 1.0,
        fixed: false,
    });
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
}

/// Helper: a pinned point at `(x, y)`.
fn fixed_pt(sys: &mut GcsSystem, x: f64, y: f64) -> PointId {
    sys.add_point(PointData { x, y, fixed: true })
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
