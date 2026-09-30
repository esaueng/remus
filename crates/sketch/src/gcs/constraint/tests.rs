use super::*;

/// Build a simple snapshot with two points at given positions.
fn two_point_snap(x1: f64, y1: f64, x2: f64, y2: f64) -> (PointId, PointId, EntitySnapshot) {
    use super::super::entity::GenArena;
    use super::super::entity::PointData;
    let mut arena = GenArena::new();
    let p1 = arena.insert(PointData {
        x: x1,
        y: y1,
        fixed: false,
    });
    let p2 = arena.insert(PointData {
        x: x2,
        y: y2,
        fixed: false,
    });
    let snap = EntitySnapshot {
        points: [(p1, (x1, y1)), (p2, (x2, y2))].into_iter().collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    (p1, p2, snap)
}

#[test]
fn coincident_at_solution() {
    let (p1, p2, snap) = two_point_snap(3.0, 4.0, 3.0, 4.0);
    let c = Constraint::Coincident(p1, p2);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert_eq!(r.len(), 2);
    assert!((r[0]).abs() < 1e-15);
    assert!((r[1]).abs() < 1e-15);
}

#[test]
fn coincident_away_from_solution() {
    let (p1, p2, snap) = two_point_snap(0.0, 0.0, 1.0, 2.0);
    let c = Constraint::Coincident(p1, p2);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert!((r[0] - (-1.0)).abs() < 1e-15);
    assert!((r[1] - (-2.0)).abs() < 1e-15);
}

#[test]
fn distance_at_solution() {
    let (p1, p2, snap) = two_point_snap(0.0, 0.0, 3.0, 4.0);
    let c = Constraint::Distance(p1, p2, 5.0);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert!(r[0].abs() < 1e-14, "residual = {}", r[0]);
}

#[test]
fn fix_x_residual() {
    let (p1, _, snap) = two_point_snap(7.0, 3.0, 0.0, 0.0);
    let c = Constraint::FixX(p1, 5.0);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert!((r[0] - 2.0).abs() < 1e-15);
}

/// Verify analytic Jacobian against finite differences for a constraint.
fn check_jacobian_fd(c: &Constraint, snap: &EntitySnapshot, params: &[ParamRef]) {
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let n = params.len();
    let m = residual_count(c);

    // Analytic Jacobian
    let mut jac = vec![0.0; m * n];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: n,
        param_index: &param_index,
    };
    eval_jacobian(c, snap, &mut jw, 0);

    // Finite-difference Jacobian
    let eps = 1e-7;
    let mut r0 = Vec::new();
    eval_residuals(c, snap, &mut r0);

    for (col, pr) in params.iter().enumerate() {
        let mut perturbed_points = snap.points.clone();
        match pr {
            ParamRef::PointX(pid) => {
                if let Some(xy) = perturbed_points.get_mut(pid) {
                    xy.0 += eps;
                }
            }
            ParamRef::PointY(pid) => {
                if let Some(xy) = perturbed_points.get_mut(pid) {
                    xy.1 += eps;
                }
            }
            ParamRef::CircleRadius(cid) => {
                // Perturb circle radius — need a mutable copy of circles
                let mut perturbed_circles = snap.circles.clone();
                if let Some(entry) = perturbed_circles.get_mut(cid) {
                    entry.1 += eps;
                }
                let perturbed_snap_circ = EntitySnapshot {
                    points: perturbed_points,
                    lines: snap.lines.clone(),
                    circles: perturbed_circles,
                    arcs: snap.arcs.clone(),
                    ellipses: snap.ellipses.clone(),
                };
                let mut r1 = Vec::new();
                eval_residuals(c, &perturbed_snap_circ, &mut r1);
                for row in 0..m {
                    let fd = (r1[row] - r0[row]) / eps;
                    let analytic = jac[row * n + col];
                    let err = (fd - analytic).abs();
                    let scale = 1.0_f64.max(analytic.abs());
                    assert!(
                        err < 1e-5 * scale + 1e-8,
                        "Jacobian mismatch at ({row},{col}): analytic={analytic}, fd={fd}, err={err}"
                    );
                }
                continue;
            }
            ParamRef::EllipseA(eid) | ParamRef::EllipseB(eid) | ParamRef::EllipsePhi(eid) => {
                // Perturb an ellipse scalar — needs a mutable copy of ellipses.
                let mut perturbed_ellipses = snap.ellipses.clone();
                if let Some(entry) = perturbed_ellipses.get_mut(eid) {
                    match pr {
                        ParamRef::EllipseA(_) => entry.1 += eps,
                        ParamRef::EllipseB(_) => entry.2 += eps,
                        _ => entry.3 += eps,
                    }
                }
                let perturbed_snap_ell = EntitySnapshot {
                    points: perturbed_points,
                    lines: snap.lines.clone(),
                    circles: snap.circles.clone(),
                    arcs: snap.arcs.clone(),
                    ellipses: perturbed_ellipses,
                };
                let mut r1 = Vec::new();
                eval_residuals(c, &perturbed_snap_ell, &mut r1);
                for row in 0..m {
                    let fd = (r1[row] - r0[row]) / eps;
                    let analytic = jac[row * n + col];
                    let err = (fd - analytic).abs();
                    let scale = 1.0_f64.max(analytic.abs());
                    assert!(
                        err < 1e-5 * scale + 1e-8,
                        "Jacobian mismatch at ({row},{col}): analytic={analytic}, fd={fd}, err={err}"
                    );
                }
                continue;
            }
        }
        let perturbed_snap = EntitySnapshot {
            points: perturbed_points,
            lines: snap.lines.clone(),
            circles: snap.circles.clone(),
            arcs: snap.arcs.clone(),
            ellipses: snap.ellipses.clone(),
        };
        let mut r1 = Vec::new();
        eval_residuals(c, &perturbed_snap, &mut r1);

        for row in 0..m {
            let fd = (r1[row] - r0[row]) / eps;
            let analytic = jac[row * n + col];
            let err = (fd - analytic).abs();
            let scale = 1.0_f64.max(analytic.abs());
            assert!(
                err < 1e-5 * scale + 1e-8,
                "Jacobian mismatch at ({row},{col}): analytic={analytic}, fd={fd}, err={err}"
            );
        }
    }
}

#[test]
fn jacobian_coincident() {
    let (p1, p2, snap) = two_point_snap(1.0, 2.0, 3.0, 5.0);
    let c = Constraint::Coincident(p1, p2);
    let params = vec![
        ParamRef::PointX(p1),
        ParamRef::PointY(p1),
        ParamRef::PointX(p2),
        ParamRef::PointY(p2),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_distance() {
    let (p1, p2, snap) = two_point_snap(1.0, 2.0, 4.0, 6.0);
    let c = Constraint::Distance(p1, p2, 5.0);
    let params = vec![
        ParamRef::PointX(p1),
        ParamRef::PointY(p1),
        ParamRef::PointX(p2),
        ParamRef::PointY(p2),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_fix_xy() {
    let (p1, _, snap) = two_point_snap(3.0, 7.0, 0.0, 0.0);
    check_jacobian_fd(&Constraint::FixX(p1, 5.0), &snap, &[ParamRef::PointX(p1)]);
    check_jacobian_fd(&Constraint::FixY(p1, 2.0), &snap, &[ParamRef::PointY(p1)]);
}

#[test]
fn jacobian_horizontal_vertical() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let mut pts = GenArena::new();
    let p1 = pts.insert(PointData {
        x: 1.0,
        y: 3.0,
        fixed: false,
    });
    let p2 = pts.insert(PointData {
        x: 5.0,
        y: 7.0,
        fixed: false,
    });
    let mut lines = GenArena::new();
    let l = lines.insert(LineData { p1, p2 });

    let snap = EntitySnapshot {
        points: [(p1, (1.0, 3.0)), (p2, (5.0, 7.0))].into_iter().collect(),
        lines: std::iter::once((l, (p1, p2))).collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let params = vec![
        ParamRef::PointX(p1),
        ParamRef::PointY(p1),
        ParamRef::PointX(p2),
        ParamRef::PointY(p2),
    ];
    check_jacobian_fd(&Constraint::Horizontal(l), &snap, &params);
    check_jacobian_fd(&Constraint::Vertical(l), &snap, &params);
}

#[test]
fn jacobian_parallel_perpendicular() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let mut pts = GenArena::new();
    let p1 = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let p2 = pts.insert(PointData {
        x: 3.0,
        y: 1.0,
        fixed: false,
    });
    let p3 = pts.insert(PointData {
        x: 1.0,
        y: 2.0,
        fixed: false,
    });
    let p4 = pts.insert(PointData {
        x: 4.0,
        y: 5.0,
        fixed: false,
    });
    let mut lines = GenArena::new();
    let l1 = lines.insert(LineData { p1, p2 });
    let l2 = lines.insert(LineData { p1: p3, p2: p4 });

    let snap = EntitySnapshot {
        points: [
            (p1, (0.0, 0.0)),
            (p2, (3.0, 1.0)),
            (p3, (1.0, 2.0)),
            (p4, (4.0, 5.0)),
        ]
        .into_iter()
        .collect(),
        lines: [(l1, (p1, p2)), (l2, (p3, p4))].into_iter().collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let params = vec![
        ParamRef::PointX(p1),
        ParamRef::PointY(p1),
        ParamRef::PointX(p2),
        ParamRef::PointY(p2),
        ParamRef::PointX(p3),
        ParamRef::PointY(p3),
        ParamRef::PointX(p4),
        ParamRef::PointY(p4),
    ];
    check_jacobian_fd(&Constraint::Parallel(l1, l2), &snap, &params);
    check_jacobian_fd(&Constraint::Perpendicular(l1, l2), &snap, &params);
}

#[test]
fn jacobian_angle() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let mut pts = GenArena::new();
    let p1 = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let p2 = pts.insert(PointData {
        x: 3.0,
        y: 1.0,
        fixed: false,
    });
    let p3 = pts.insert(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let p4 = pts.insert(PointData {
        x: 2.0,
        y: 4.0,
        fixed: false,
    });
    let mut lines = GenArena::new();
    let l1 = lines.insert(LineData { p1, p2 });
    let l2 = lines.insert(LineData { p1: p3, p2: p4 });

    let snap = EntitySnapshot {
        points: [
            (p1, (0.0, 0.0)),
            (p2, (3.0, 1.0)),
            (p3, (1.0, 0.0)),
            (p4, (2.0, 4.0)),
        ]
        .into_iter()
        .collect(),
        lines: [(l1, (p1, p2)), (l2, (p3, p4))].into_iter().collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let params = vec![
        ParamRef::PointX(p1),
        ParamRef::PointY(p1),
        ParamRef::PointX(p2),
        ParamRef::PointY(p2),
        ParamRef::PointX(p3),
        ParamRef::PointY(p3),
        ParamRef::PointX(p4),
        ParamRef::PointY(p4),
    ];
    check_jacobian_fd(&Constraint::Angle(l1, l2, 0.5), &snap, &params);
}

#[test]
fn jacobian_point_on_circle() {
    use super::super::entity::GenArena;
    use super::super::entity::{CircleData, PointData};
    let mut pts = GenArena::new();
    let center = pts.insert(PointData {
        x: 1.0,
        y: 2.0,
        fixed: false,
    });
    let pt = pts.insert(PointData {
        x: 4.0,
        y: 6.0,
        fixed: false,
    });
    let mut circles = GenArena::new();
    let circ = circles.insert(CircleData {
        center,
        radius: 3.0,
    });
    let snap = EntitySnapshot {
        points: [(center, (1.0, 2.0)), (pt, (4.0, 6.0))]
            .into_iter()
            .collect(),
        lines: HashMap::new(),
        circles: [(circ, (center, 3.0))].into_iter().collect(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::PointOnCircle(pt, circ);
    let params = vec![
        ParamRef::PointX(pt),
        ParamRef::PointY(pt),
        ParamRef::PointX(center),
        ParamRef::PointY(center),
        ParamRef::CircleRadius(circ),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_point_on_arc() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, PointData};
    let mut pts = GenArena::new();
    let center = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let start = pts.insert(PointData {
        x: 2.0,
        y: 0.0,
        fixed: false,
    });
    let end = pts.insert(PointData {
        x: 0.0,
        y: 2.0,
        fixed: false,
    });
    let pt = pts.insert(PointData {
        x: 1.5,
        y: 1.5,
        fixed: false,
    });
    let mut arcs = GenArena::new();
    let arc = arcs.insert(ArcData { center, start, end });
    let snap = EntitySnapshot {
        points: [
            (center, (0.0, 0.0)),
            (start, (2.0, 0.0)),
            (end, (0.0, 2.0)),
            (pt, (1.5, 1.5)),
        ]
        .into_iter()
        .collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: [(arc, (center, start, end))].into_iter().collect(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::PointOnArc(pt, arc);
    let params = vec![
        ParamRef::PointX(pt),
        ParamRef::PointY(pt),
        ParamRef::PointX(center),
        ParamRef::PointY(center),
        ParamRef::PointX(start),
        ParamRef::PointY(start),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_tangent_line_arc() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, LineData, PointData};
    let mut pts = GenArena::new();
    let p1 = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let p2 = pts.insert(PointData {
        x: 2.0,
        y: 0.0,
        fixed: false,
    });
    let center = pts.insert(PointData {
        x: 2.0,
        y: 1.0,
        fixed: false,
    });
    let start = pts.insert(PointData {
        x: 2.0,
        y: 0.0,
        fixed: false,
    });
    let end = pts.insert(PointData {
        x: 3.0,
        y: 1.0,
        fixed: false,
    });
    // shared point is p2 (same position as start)
    let mut lines = GenArena::new();
    let line = lines.insert(LineData { p1, p2 });
    let mut arcs = GenArena::new();
    let arc = arcs.insert(ArcData { center, start, end });
    let snap = EntitySnapshot {
        points: [
            (p1, (0.0, 0.0)),
            (p2, (2.0, 0.0)),
            (center, (2.0, 1.0)),
            (start, (2.0, 0.0)),
            (end, (3.0, 1.0)),
        ]
        .into_iter()
        .collect(),
        lines: [(line, (p1, p2))].into_iter().collect(),
        circles: HashMap::new(),
        arcs: [(arc, (center, start, end))].into_iter().collect(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::TangentLineArc(line, arc, p2);
    let params = vec![
        ParamRef::PointX(p1),
        ParamRef::PointY(p1),
        ParamRef::PointX(p2),
        ParamRef::PointY(p2),
        ParamRef::PointX(center),
        ParamRef::PointY(center),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_tangent_arc_arc() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, PointData};
    let mut pts = GenArena::new();
    let c1 = pts.insert(PointData {
        x: 0.0,
        y: 1.0,
        fixed: false,
    });
    let c2 = pts.insert(PointData {
        x: 2.0,
        y: 1.0,
        fixed: false,
    });
    let shared = pts.insert(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let s1 = pts.insert(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let e1 = pts.insert(PointData {
        x: -1.0,
        y: 1.0,
        fixed: false,
    });
    let s2 = pts.insert(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let e2 = pts.insert(PointData {
        x: 3.0,
        y: 1.0,
        fixed: false,
    });
    let mut arcs = GenArena::new();
    let arc1 = arcs.insert(ArcData {
        center: c1,
        start: s1,
        end: e1,
    });
    let arc2 = arcs.insert(ArcData {
        center: c2,
        start: s2,
        end: e2,
    });
    let snap = EntitySnapshot {
        points: [
            (c1, (0.0, 1.0)),
            (c2, (2.0, 1.0)),
            (shared, (1.0, 0.0)),
            (s1, (1.0, 0.0)),
            (e1, (-1.0, 1.0)),
            (s2, (1.0, 0.0)),
            (e2, (3.0, 1.0)),
        ]
        .into_iter()
        .collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: [(arc1, (c1, s1, e1)), (arc2, (c2, s2, e2))]
            .into_iter()
            .collect(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::TangentArcArc(arc1, arc2, shared);
    let params = vec![
        ParamRef::PointX(shared),
        ParamRef::PointY(shared),
        ParamRef::PointX(c1),
        ParamRef::PointY(c1),
        ParamRef::PointX(c2),
        ParamRef::PointY(c2),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_equal_radius_arc_arc() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, PointData};
    let mut pts = GenArena::new();
    let c1 = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let s1 = pts.insert(PointData {
        x: 2.0,
        y: 0.0,
        fixed: false,
    });
    let e1 = pts.insert(PointData {
        x: 0.0,
        y: 2.0,
        fixed: false,
    });
    let c2 = pts.insert(PointData {
        x: 5.0,
        y: 0.0,
        fixed: false,
    });
    let s2 = pts.insert(PointData {
        x: 8.0,
        y: 0.0,
        fixed: false,
    });
    let e2 = pts.insert(PointData {
        x: 5.0,
        y: 3.0,
        fixed: false,
    });
    let mut arcs = GenArena::new();
    let arc1 = arcs.insert(ArcData {
        center: c1,
        start: s1,
        end: e1,
    });
    let arc2 = arcs.insert(ArcData {
        center: c2,
        start: s2,
        end: e2,
    });
    let snap = EntitySnapshot {
        points: [
            (c1, (0.0, 0.0)),
            (s1, (2.0, 0.0)),
            (e1, (0.0, 2.0)),
            (c2, (5.0, 0.0)),
            (s2, (8.0, 0.0)),
            (e2, (5.0, 3.0)),
        ]
        .into_iter()
        .collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: [(arc1, (c1, s1, e1)), (arc2, (c2, s2, e2))]
            .into_iter()
            .collect(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::EqualRadiusArcArc(arc1, arc2);
    let params = vec![
        ParamRef::PointX(c1),
        ParamRef::PointY(c1),
        ParamRef::PointX(s1),
        ParamRef::PointY(s1),
        ParamRef::PointX(c2),
        ParamRef::PointY(c2),
        ParamRef::PointX(s2),
        ParamRef::PointY(s2),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_equal_radius_arc_circle() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, CircleData, PointData};
    let mut pts = GenArena::new();
    let ac = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let as_ = pts.insert(PointData {
        x: 2.0,
        y: 0.0,
        fixed: false,
    });
    let ae = pts.insert(PointData {
        x: 0.0,
        y: 2.0,
        fixed: false,
    });
    let cc = pts.insert(PointData {
        x: 5.0,
        y: 5.0,
        fixed: false,
    });
    let mut arcs = GenArena::new();
    let arc = arcs.insert(ArcData {
        center: ac,
        start: as_,
        end: ae,
    });
    let mut circles = GenArena::new();
    let circ = circles.insert(CircleData {
        center: cc,
        radius: 3.0,
    });
    let snap = EntitySnapshot {
        points: [
            (ac, (0.0, 0.0)),
            (as_, (2.0, 0.0)),
            (ae, (0.0, 2.0)),
            (cc, (5.0, 5.0)),
        ]
        .into_iter()
        .collect(),
        lines: HashMap::new(),
        circles: [(circ, (cc, 3.0))].into_iter().collect(),
        arcs: [(arc, (ac, as_, ae))].into_iter().collect(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::EqualRadiusArcCircle(arc, circ);
    let params = vec![
        ParamRef::PointX(ac),
        ParamRef::PointY(ac),
        ParamRef::PointX(as_),
        ParamRef::PointY(as_),
        ParamRef::CircleRadius(circ),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_arc_length() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, PointData};
    let mut pts = GenArena::new();
    let center = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let start = pts.insert(PointData {
        x: 2.0,
        y: 0.0,
        fixed: false,
    });
    let end = pts.insert(PointData {
        x: 0.0,
        y: 2.0,
        fixed: false,
    });
    let mut arcs = GenArena::new();
    let arc = arcs.insert(ArcData { center, start, end });
    let snap = EntitySnapshot {
        points: [(center, (0.0, 0.0)), (start, (2.0, 0.0)), (end, (0.0, 2.0))]
            .into_iter()
            .collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: [(arc, (center, start, end))].into_iter().collect(),
        ellipses: HashMap::new(),
    };
    let target = std::f64::consts::PI; // 90 degrees * r=2
    let c = Constraint::ArcLength(arc, target);
    let params = vec![
        ParamRef::PointX(center),
        ParamRef::PointY(center),
        ParamRef::PointX(start),
        ParamRef::PointY(start),
        ParamRef::PointX(end),
        ParamRef::PointY(end),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_concentric_arc_arc() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, PointData};
    let mut pts = GenArena::new();
    let c1 = pts.insert(PointData {
        x: 1.0,
        y: 2.0,
        fixed: false,
    });
    let s1 = pts.insert(PointData {
        x: 3.0,
        y: 2.0,
        fixed: false,
    });
    let e1 = pts.insert(PointData {
        x: 1.0,
        y: 4.0,
        fixed: false,
    });
    let c2 = pts.insert(PointData {
        x: 3.0,
        y: 4.0,
        fixed: false,
    });
    let s2 = pts.insert(PointData {
        x: 4.0,
        y: 4.0,
        fixed: false,
    });
    let e2 = pts.insert(PointData {
        x: 3.0,
        y: 5.0,
        fixed: false,
    });
    let mut arcs = GenArena::new();
    let arc1 = arcs.insert(ArcData {
        center: c1,
        start: s1,
        end: e1,
    });
    let arc2 = arcs.insert(ArcData {
        center: c2,
        start: s2,
        end: e2,
    });
    let snap = EntitySnapshot {
        points: [
            (c1, (1.0, 2.0)),
            (s1, (3.0, 2.0)),
            (e1, (1.0, 4.0)),
            (c2, (3.0, 4.0)),
            (s2, (4.0, 4.0)),
            (e2, (3.0, 5.0)),
        ]
        .into_iter()
        .collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: [(arc1, (c1, s1, e1)), (arc2, (c2, s2, e2))]
            .into_iter()
            .collect(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::ConcentricArcArc(arc1, arc2);
    let params = vec![
        ParamRef::PointX(c1),
        ParamRef::PointY(c1),
        ParamRef::PointX(c2),
        ParamRef::PointY(c2),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_concentric_arc_circle() {
    use super::super::entity::GenArena;
    use super::super::entity::{ArcData, CircleData, PointData};
    let mut pts = GenArena::new();
    let ac = pts.insert(PointData {
        x: 1.0,
        y: 2.0,
        fixed: false,
    });
    let as_ = pts.insert(PointData {
        x: 3.0,
        y: 2.0,
        fixed: false,
    });
    let ae = pts.insert(PointData {
        x: 1.0,
        y: 4.0,
        fixed: false,
    });
    let cc = pts.insert(PointData {
        x: 3.0,
        y: 4.0,
        fixed: false,
    });
    let mut arcs = GenArena::new();
    let arc = arcs.insert(ArcData {
        center: ac,
        start: as_,
        end: ae,
    });
    let mut circles = GenArena::new();
    let circ = circles.insert(CircleData {
        center: cc,
        radius: 2.0,
    });
    let snap = EntitySnapshot {
        points: [
            (ac, (1.0, 2.0)),
            (as_, (3.0, 2.0)),
            (ae, (1.0, 4.0)),
            (cc, (3.0, 4.0)),
        ]
        .into_iter()
        .collect(),
        lines: HashMap::new(),
        circles: [(circ, (cc, 2.0))].into_iter().collect(),
        arcs: [(arc, (ac, as_, ae))].into_iter().collect(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::ConcentricArcCircle(arc, circ);
    let params = vec![
        ParamRef::PointX(ac),
        ParamRef::PointY(ac),
        ParamRef::PointX(cc),
        ParamRef::PointY(cc),
    ];
    check_jacobian_fd(&c, &snap, &params);
}

#[test]
fn jacobian_point_line_distance() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let mut pts = GenArena::new();
    let pt = pts.insert(PointData {
        x: 2.0,
        y: 3.0,
        fixed: false,
    });
    let lp1 = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let lp2 = pts.insert(PointData {
        x: 4.0,
        y: 1.0,
        fixed: false,
    });
    let mut lines = GenArena::new();
    let l = lines.insert(LineData { p1: lp1, p2: lp2 });

    let snap = EntitySnapshot {
        points: [(pt, (2.0, 3.0)), (lp1, (0.0, 0.0)), (lp2, (4.0, 1.0))]
            .into_iter()
            .collect(),
        lines: std::iter::once((l, (lp1, lp2))).collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let params = vec![
        ParamRef::PointX(pt),
        ParamRef::PointY(pt),
        ParamRef::PointX(lp1),
        ParamRef::PointY(lp1),
        ParamRef::PointX(lp2),
        ParamRef::PointY(lp2),
    ];
    check_jacobian_fd(&Constraint::PointLineDistance(pt, l, 1.5), &snap, &params);
}

#[test]
fn point_line_distance_rejects_nonzero_target_on_degenerate_line() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let mut points = GenArena::new();
    let point = points.insert(PointData {
        x: 4.0,
        y: 5.0,
        fixed: false,
    });
    let line_point = points.insert(PointData {
        x: 1.0,
        y: 2.0,
        fixed: false,
    });
    let mut lines = GenArena::new();
    let line = lines.insert(LineData {
        p1: line_point,
        p2: line_point,
    });
    let snap = EntitySnapshot {
        points: [(point, (4.0, 5.0)), (line_point, (1.0, 2.0))]
            .into_iter()
            .collect(),
        lines: [(line, (line_point, line_point))].into_iter().collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let mut residuals = Vec::new();
    eval_residuals(
        &Constraint::PointLineDistance(point, line, 3.0),
        &snap,
        &mut residuals,
    );
    assert_eq!(residuals, vec![-3.0]);
}

// ── Constraints added for selection-first sketching ─────────────────
//
// circleRadius, equalRadiusCircleCircle, equalLength, midpoint, symmetric.
//
// Every analytic Jacobian below is checked against central differences at
// three coordinate scales. The shared `check_jacobian_fd` helper uses a fixed
// 1e-7 step, which loses too much to cancellation once coordinates reach 1e5,
// so these use a step sized to the geometry instead.

/// Perturb one parameter in a snapshot by `delta`, returning the new snapshot.
fn perturb(snap: &EntitySnapshot, pr: ParamRef, delta: f64) -> EntitySnapshot {
    let mut points = snap.points.clone();
    let mut circles = snap.circles.clone();
    let mut ellipses = snap.ellipses.clone();
    match pr {
        ParamRef::PointX(pid) => {
            if let Some(xy) = points.get_mut(&pid) {
                xy.0 += delta;
            }
        }
        ParamRef::PointY(pid) => {
            if let Some(xy) = points.get_mut(&pid) {
                xy.1 += delta;
            }
        }
        ParamRef::CircleRadius(cid) => {
            if let Some(entry) = circles.get_mut(&cid) {
                entry.1 += delta;
            }
        }
        ParamRef::EllipseA(eid) => {
            if let Some(entry) = ellipses.get_mut(&eid) {
                entry.1 += delta;
            }
        }
        ParamRef::EllipseB(eid) => {
            if let Some(entry) = ellipses.get_mut(&eid) {
                entry.2 += delta;
            }
        }
        ParamRef::EllipsePhi(eid) => {
            if let Some(entry) = ellipses.get_mut(&eid) {
                entry.3 += delta;
            }
        }
    }
    EntitySnapshot {
        points,
        lines: snap.lines.clone(),
        circles,
        arcs: snap.arcs.clone(),
        ellipses,
    }
}

/// Central-difference Jacobian check with a step proportional to `scale`.
///
/// All five constraints here have O(1) derivatives regardless of coordinate
/// magnitude (unit directions and constant factors), so a fixed absolute
/// tolerance is the right comparison at every scale.
fn check_jacobian_central(c: &Constraint, snap: &EntitySnapshot, params: &[ParamRef], scale: f64) {
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let n = params.len();
    let m = residual_count(c);

    let mut jac = vec![0.0; m * n];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: n,
        param_index: &param_index,
    };
    eval_jacobian(c, snap, &mut jw, 0);

    let eps = 1e-6 * scale;
    for (col, pr) in params.iter().enumerate() {
        let mut r_plus = Vec::new();
        eval_residuals(c, &perturb(snap, *pr, eps), &mut r_plus);
        let mut r_minus = Vec::new();
        eval_residuals(c, &perturb(snap, *pr, -eps), &mut r_minus);

        for row in 0..m {
            let fd = (r_plus[row] - r_minus[row]) / (2.0 * eps);
            let analytic = jac[row * n + col];
            let err = (fd - analytic).abs();
            assert!(
                err < 1e-6 * 1.0_f64.max(analytic.abs()),
                "Jacobian mismatch at ({row},{col}) scale={scale}: \
                 analytic={analytic}, fd={fd}, err={err}"
            );
        }
    }
}

/// Coordinate scales exercised by every new constraint: sub-millimetre,
/// ordinary part size, and large-assembly.
const SCALES: [f64; 3] = [1e-3, 1.0, 1e5];

// ── circleRadius ────────────────────────────────────────────────────

#[test]
fn circle_radius_residual_and_jacobian() {
    use super::super::entity::GenArena;
    use super::super::entity::{CircleData, PointData};
    for scale in SCALES {
        let mut pts = GenArena::new();
        let center = pts.insert(PointData {
            x: 1.0 * scale,
            y: 2.0 * scale,
            fixed: false,
        });
        let mut circles = GenArena::new();
        let radius = 3.0 * scale;
        let circ = circles.insert(CircleData { center, radius });
        let snap = EntitySnapshot {
            points: [(center, (1.0 * scale, 2.0 * scale))].into_iter().collect(),
            lines: HashMap::new(),
            circles: [(circ, (center, radius))].into_iter().collect(),
            arcs: HashMap::new(),
            ellipses: HashMap::new(),
        };

        // At the target: zero residual. Away from it: the signed difference.
        let mut r = Vec::new();
        eval_residuals(&Constraint::CircleRadius(circ, radius), &snap, &mut r);
        assert_eq!(r.len(), 1);
        assert!(r[0].abs() < 1e-12 * scale.max(1.0), "residual {}", r[0]);

        let mut r2 = Vec::new();
        eval_residuals(&Constraint::CircleRadius(circ, 2.0 * scale), &snap, &mut r2);
        assert!(
            (r2[0] - scale).abs() < 1e-12 * scale.max(1.0),
            "expected {scale}, got {}",
            r2[0]
        );

        check_jacobian_central(
            &Constraint::CircleRadius(circ, 2.0 * scale),
            &snap,
            &[ParamRef::CircleRadius(circ), ParamRef::PointX(center)],
            scale,
        );
    }
}

// ── equalRadiusCircleCircle ─────────────────────────────────────────

#[test]
fn equal_radius_circle_circle_residual_and_jacobian() {
    use super::super::entity::GenArena;
    use super::super::entity::{CircleData, PointData};
    for scale in SCALES {
        let mut pts = GenArena::new();
        let c1_center = pts.insert(PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        });
        let c2_center = pts.insert(PointData {
            x: 10.0 * scale,
            y: 0.0,
            fixed: false,
        });
        let mut circles = GenArena::new();
        let (r1, r2) = (3.0 * scale, 5.0 * scale);
        let circ1 = circles.insert(CircleData {
            center: c1_center,
            radius: r1,
        });
        let circ2 = circles.insert(CircleData {
            center: c2_center,
            radius: r2,
        });
        let snap = EntitySnapshot {
            points: [(c1_center, (0.0, 0.0)), (c2_center, (10.0 * scale, 0.0))]
                .into_iter()
                .collect(),
            lines: HashMap::new(),
            circles: [(circ1, (c1_center, r1)), (circ2, (c2_center, r2))]
                .into_iter()
                .collect(),
            arcs: HashMap::new(),
            ellipses: HashMap::new(),
        };

        let c = Constraint::EqualRadiusCircleCircle(circ1, circ2);
        let mut r = Vec::new();
        eval_residuals(&c, &snap, &mut r);
        assert!(
            (r[0] - (r1 - r2)).abs() < 1e-12 * scale.max(1.0),
            "residual {}",
            r[0]
        );

        check_jacobian_central(
            &c,
            &snap,
            &[
                ParamRef::CircleRadius(circ1),
                ParamRef::CircleRadius(circ2),
                ParamRef::PointX(c1_center),
            ],
            scale,
        );
    }
}

/// A circle constrained equal to itself must produce a zero row, not a
/// contradiction. `add` accumulation (rather than `set`) is what makes the
/// +1 and -1 cancel.
#[test]
fn equal_radius_circle_circle_self_reference_cancels() {
    use super::super::entity::GenArena;
    use super::super::entity::{CircleData, PointData};
    let mut pts = GenArena::new();
    let center = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let mut circles = GenArena::new();
    let circ = circles.insert(CircleData {
        center,
        radius: 4.0,
    });
    let snap = EntitySnapshot {
        points: [(center, (0.0, 0.0))].into_iter().collect(),
        lines: HashMap::new(),
        circles: [(circ, (center, 4.0))].into_iter().collect(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let c = Constraint::EqualRadiusCircleCircle(circ, circ);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert!(r[0].abs() < 1e-15);

    let params = [ParamRef::CircleRadius(circ)];
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let mut jac = vec![0.0; 1];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: 1,
        param_index: &param_index,
    };
    eval_jacobian(&c, &snap, &mut jw, 0);
    assert!(
        jac[0].abs() < 1e-15,
        "self-reference row must vanish: {jac:?}"
    );
}

// ── equalLength ─────────────────────────────────────────────────────

/// Build two independent lines at a given scale.
fn two_line_snap(scale: f64) -> (LineId, LineId, [PointId; 4], EntitySnapshot) {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let coords = [
        (0.0, 0.0),
        (3.0 * scale, 4.0 * scale),
        (10.0 * scale, 1.0 * scale),
        (13.0 * scale, 9.0 * scale),
    ];
    let mut pts = GenArena::new();
    let ids: Vec<PointId> = coords
        .iter()
        .map(|&(x, y)| pts.insert(PointData { x, y, fixed: false }))
        .collect();
    let mut lines = GenArena::new();
    let l1 = lines.insert(LineData {
        p1: ids[0],
        p2: ids[1],
    });
    let l2 = lines.insert(LineData {
        p1: ids[2],
        p2: ids[3],
    });
    let snap = EntitySnapshot {
        points: ids.iter().copied().zip(coords).collect(),
        lines: [(l1, (ids[0], ids[1])), (l2, (ids[2], ids[3]))]
            .into_iter()
            .collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    (l1, l2, [ids[0], ids[1], ids[2], ids[3]], snap)
}

#[test]
fn equal_length_residual_and_jacobian() {
    for scale in SCALES {
        let (l1, l2, p, snap) = two_line_snap(scale);
        let c = Constraint::EqualLength(l1, l2);

        // len1 = 5·scale (3-4-5), len2 = sqrt(9+64)·scale.
        let expected = (5.0 - 73.0_f64.sqrt()) * scale;
        let mut r = Vec::new();
        eval_residuals(&c, &snap, &mut r);
        assert_eq!(r.len(), 1);
        assert!(
            (r[0] - expected).abs() < 1e-10 * scale.max(1.0),
            "expected {expected}, got {}",
            r[0]
        );

        let params: Vec<ParamRef> = p
            .iter()
            .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
            .collect();
        check_jacobian_central(&c, &snap, &params, scale);
    }
}

/// Two lines of equal length give a zero residual regardless of orientation.
#[test]
fn equal_length_at_solution() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let coords = [(0.0, 0.0), (5.0, 0.0), (2.0, 2.0), (2.0, 7.0)];
    let mut pts = GenArena::new();
    let ids: Vec<PointId> = coords
        .iter()
        .map(|&(x, y)| pts.insert(PointData { x, y, fixed: false }))
        .collect();
    let mut lines = GenArena::new();
    let l1 = lines.insert(LineData {
        p1: ids[0],
        p2: ids[1],
    });
    let l2 = lines.insert(LineData {
        p1: ids[2],
        p2: ids[3],
    });
    let snap = EntitySnapshot {
        points: ids.iter().copied().zip(coords).collect(),
        lines: [(l1, (ids[0], ids[1])), (l2, (ids[2], ids[3]))]
            .into_iter()
            .collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    let mut r = Vec::new();
    eval_residuals(&Constraint::EqualLength(l1, l2), &snap, &mut r);
    assert!(r[0].abs() < 1e-15, "residual {}", r[0]);
}

/// A zero-length line has no direction. The residual stays finite and the
/// Jacobian drops that line's contribution instead of producing NaN.
#[test]
fn equal_length_degenerate_line_is_finite() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let coords = [(2.0, 2.0), (2.0, 2.0), (0.0, 0.0), (3.0, 4.0)];
    let mut pts = GenArena::new();
    let ids: Vec<PointId> = coords
        .iter()
        .map(|&(x, y)| pts.insert(PointData { x, y, fixed: false }))
        .collect();
    let mut lines = GenArena::new();
    let degenerate = lines.insert(LineData {
        p1: ids[0],
        p2: ids[1],
    });
    let normal = lines.insert(LineData {
        p1: ids[2],
        p2: ids[3],
    });
    let snap = EntitySnapshot {
        points: ids.iter().copied().zip(coords).collect(),
        lines: [(degenerate, (ids[0], ids[1])), (normal, (ids[2], ids[3]))]
            .into_iter()
            .collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };

    let c = Constraint::EqualLength(degenerate, normal);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert!(r[0].is_finite(), "residual must stay finite: {}", r[0]);
    assert!(
        (r[0] - (-5.0)).abs() < 1e-12,
        "0 - 5 expected, got {}",
        r[0]
    );

    let params: Vec<ParamRef> = ids
        .iter()
        .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
        .collect();
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let n = params.len();
    let mut jac = vec![0.0; n];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: n,
        param_index: &param_index,
    };
    eval_jacobian(&c, &snap, &mut jw, 0);
    assert!(
        jac.iter().all(|v| v.is_finite()),
        "degenerate line must not poison the Jacobian: {jac:?}"
    );
}

// ── midpoint ────────────────────────────────────────────────────────

#[test]
fn midpoint_residual_and_jacobian() {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    for scale in SCALES {
        let coords = [
            (0.0, 0.0),
            (10.0 * scale, 6.0 * scale),
            (2.0 * scale, 1.0 * scale),
        ];
        let mut pts = GenArena::new();
        let ids: Vec<PointId> = coords
            .iter()
            .map(|&(x, y)| pts.insert(PointData { x, y, fixed: false }))
            .collect();
        let (a, b, mid) = (ids[0], ids[1], ids[2]);
        let mut lines = GenArena::new();
        let line = lines.insert(LineData { p1: a, p2: b });
        let snap = EntitySnapshot {
            points: ids.iter().copied().zip(coords).collect(),
            lines: std::iter::once((line, (a, b))).collect(),
            circles: HashMap::new(),
            arcs: HashMap::new(),
            ellipses: HashMap::new(),
        };

        // mid sits at (2,1)·scale; the true midpoint is (5,3)·scale.
        let mut r = Vec::new();
        eval_residuals(&Constraint::Midpoint(mid, line), &snap, &mut r);
        assert_eq!(r.len(), 2);
        assert!((r[0] - (-3.0 * scale)).abs() < 1e-10 * scale.max(1.0));
        assert!((r[1] - (-2.0 * scale)).abs() < 1e-10 * scale.max(1.0));

        let params: Vec<ParamRef> = ids
            .iter()
            .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
            .collect();
        check_jacobian_central(&Constraint::Midpoint(mid, line), &snap, &params, scale);
    }
}

// ── symmetric ───────────────────────────────────────────────────────

/// Two points and an axis line, at a given scale.
fn symmetric_snap(
    scale: f64,
    p1: (f64, f64),
    p2: (f64, f64),
    axis_a: (f64, f64),
    axis_b: (f64, f64),
) -> (PointId, PointId, LineId, [PointId; 4], EntitySnapshot) {
    use super::super::entity::GenArena;
    use super::super::entity::{LineData, PointData};
    let coords = [
        (p1.0 * scale, p1.1 * scale),
        (p2.0 * scale, p2.1 * scale),
        (axis_a.0 * scale, axis_a.1 * scale),
        (axis_b.0 * scale, axis_b.1 * scale),
    ];
    let mut pts = GenArena::new();
    let ids: Vec<PointId> = coords
        .iter()
        .map(|&(x, y)| pts.insert(PointData { x, y, fixed: false }))
        .collect();
    let mut lines = GenArena::new();
    let axis = lines.insert(LineData {
        p1: ids[2],
        p2: ids[3],
    });
    let snap = EntitySnapshot {
        points: ids.iter().copied().zip(coords).collect(),
        lines: std::iter::once((axis, (ids[2], ids[3]))).collect(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    (ids[0], ids[1], axis, [ids[0], ids[1], ids[2], ids[3]], snap)
}

/// A genuinely mirrored pair produces zero residuals — checked about the
/// y-axis, the x-axis, and a slanted axis.
#[test]
fn symmetric_zero_at_true_mirror() {
    for scale in SCALES {
        // (p1, p2, axis start, axis end), each an (x, y) pair.
        type MirrorCase = ((f64, f64), (f64, f64), (f64, f64), (f64, f64));
        let cases: [MirrorCase; 3] = [
            // Mirror about the y-axis.
            ((-3.0, 2.0), (3.0, 2.0), (0.0, -1.0), (0.0, 5.0)),
            // Mirror about the x-axis.
            ((4.0, -7.0), (4.0, 7.0), (-2.0, 0.0), (6.0, 0.0)),
            // Mirror about the 45° line y = x.
            ((1.0, 5.0), (5.0, 1.0), (0.0, 0.0), (2.0, 2.0)),
        ];
        for (p1c, p2c, a, b) in cases {
            let (p1, p2, axis, _, snap) = symmetric_snap(scale, p1c, p2c, a, b);
            let mut r = Vec::new();
            eval_residuals(&Constraint::Symmetric(p1, p2, axis), &snap, &mut r);
            assert_eq!(r.len(), 2);
            let tol = 1e-10 * scale.max(1.0);
            assert!(
                r[0].abs() < tol && r[1].abs() < tol,
                "mirrored pair at scale {scale} must be symmetric, got {r:?}"
            );
        }
    }
}

/// Each residual isolates one failure mode: a pair straddling the axis
/// unevenly breaks the midpoint condition; a pair offset along the axis
/// breaks perpendicularity.
#[test]
fn symmetric_residuals_are_independent() {
    // Mirror about the y-axis, but p2 is too far out: midpoint off-axis,
    // still perpendicular.
    let (p1, p2, axis, _, snap) =
        symmetric_snap(1.0, (-3.0, 2.0), (5.0, 2.0), (0.0, 0.0), (0.0, 1.0));
    let mut r = Vec::new();
    eval_residuals(&Constraint::Symmetric(p1, p2, axis), &snap, &mut r);
    assert!(r[0].abs() > 0.5, "midpoint residual should fire: {r:?}");
    assert!(r[1].abs() < 1e-12, "perpendicularity still holds: {r:?}");

    // Symmetric horizontally but sheared vertically: midpoint on the axis,
    // segment no longer perpendicular.
    let (q1, q2, axis2, _, snap2) =
        symmetric_snap(1.0, (-3.0, 1.0), (3.0, 3.0), (0.0, 0.0), (0.0, 1.0));
    let mut r2 = Vec::new();
    eval_residuals(&Constraint::Symmetric(q1, q2, axis2), &snap2, &mut r2);
    assert!(r2[0].abs() < 1e-12, "midpoint is on the axis: {r2:?}");
    assert!(
        r2[1].abs() > 0.5,
        "perpendicularity residual should fire: {r2:?}"
    );
}

#[test]
fn symmetric_jacobian_matches_finite_differences() {
    for scale in SCALES {
        // Deliberately away from the solution and off-axis, so every partial
        // is exercised rather than vanishing at a symmetric configuration.
        let (p1, p2, axis, all, snap) =
            symmetric_snap(scale, (-2.0, 1.0), (4.0, 3.5), (0.5, -1.0), (2.0, 6.0));
        let params: Vec<ParamRef> = all
            .iter()
            .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
            .collect();
        check_jacobian_central(&Constraint::Symmetric(p1, p2, axis), &snap, &params, scale);
    }
}

/// A degenerate axis (both defining points coincident) has no direction to
/// mirror about; residuals and Jacobian stay finite instead of dividing by zero.
#[test]
fn symmetric_degenerate_axis_is_finite() {
    let (p1, p2, axis, all, snap) =
        symmetric_snap(1.0, (-3.0, 2.0), (3.0, 2.0), (1.0, 1.0), (1.0, 1.0));
    let c = Constraint::Symmetric(p1, p2, axis);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert_eq!(r.len(), 2, "residual count must not change when degenerate");
    assert!(
        r.iter().all(|v| v.is_finite()),
        "degenerate axis must not produce NaN: {r:?}"
    );

    let params: Vec<ParamRef> = all
        .iter()
        .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
        .collect();
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let n = params.len();
    let mut jac = vec![0.0; 2 * n];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: n,
        param_index: &param_index,
    };
    eval_jacobian(&c, &snap, &mut jw, 0);
    assert!(
        jac.iter().all(|v| v.is_finite()),
        "degenerate axis must not poison the Jacobian: {jac:?}"
    );
}

/// Snapshot with one line (points 0-1) and one circle (center point 2).
fn tangent_snap(
    scale: f64,
    a: (f64, f64),
    b: (f64, f64),
    center: (f64, f64),
    radius: f64,
) -> (LineId, CircleId, [PointId; 3], EntitySnapshot) {
    use super::super::entity::{CircleData, GenArena, LineData, PointData};
    let coords = [
        (a.0 * scale, a.1 * scale),
        (b.0 * scale, b.1 * scale),
        (center.0 * scale, center.1 * scale),
    ];
    let mut pts = GenArena::new();
    let ids: Vec<PointId> = coords
        .iter()
        .map(|&(x, y)| pts.insert(PointData { x, y, fixed: false }))
        .collect();
    let mut lines = GenArena::new();
    let line = lines.insert(LineData {
        p1: ids[0],
        p2: ids[1],
    });
    let mut circles = GenArena::new();
    let circle = circles.insert(CircleData {
        center: ids[2],
        radius: radius * scale,
    });
    let snap = EntitySnapshot {
        points: ids.iter().copied().zip(coords).collect(),
        lines: [(line, (ids[0], ids[1]))].into_iter().collect(),
        circles: [(circle, (ids[2], radius * scale))].into_iter().collect(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    (line, circle, [ids[0], ids[1], ids[2]], snap)
}

/// Tangency is unsigned: the circle may sit on either side of the line.
#[test]
fn tangent_line_circle_zero_at_tangency_on_both_sides() {
    for scale in SCALES {
        for center_y in [3.0, -3.0] {
            let (line, circle, _, snap) =
                tangent_snap(scale, (0.0, 0.0), (10.0, 0.0), (5.0, center_y), 3.0);
            let mut r = Vec::new();
            eval_residuals(&Constraint::TangentLineCircle(line, circle), &snap, &mut r);
            assert_eq!(r.len(), 1);
            let tol = 1e-10 * scale.max(1.0);
            assert!(
                r[0].abs() < tol,
                "tangent circle at scale {scale}, side {center_y}: residual {r:?}"
            );
        }
    }
}

#[test]
fn tangent_line_circle_fires_when_secant_or_clear() {
    // Center 1 above the line with radius 3: the line cuts the circle.
    let (line, circle, _, snap) = tangent_snap(1.0, (0.0, 0.0), (10.0, 0.0), (5.0, 1.0), 3.0);
    let mut r = Vec::new();
    eval_residuals(&Constraint::TangentLineCircle(line, circle), &snap, &mut r);
    assert!((r[0] - (1.0 - 3.0)).abs() < 1e-12, "secant residual: {r:?}");

    // Center 7 above with radius 3: the circle is clear of the line.
    let (line2, circle2, _, snap2) = tangent_snap(1.0, (0.0, 0.0), (10.0, 0.0), (5.0, 7.0), 3.0);
    let mut r2 = Vec::new();
    eval_residuals(
        &Constraint::TangentLineCircle(line2, circle2),
        &snap2,
        &mut r2,
    );
    assert!(
        (r2[0] - (7.0 - 3.0)).abs() < 1e-12,
        "clear residual: {r2:?}"
    );
}

#[test]
fn tangent_line_circle_jacobian_matches_finite_differences() {
    for scale in SCALES {
        // Off-tangency and skew, so every partial is exercised.
        let (line, circle, pts, snap) =
            tangent_snap(scale, (-2.0, 1.0), (7.0, 4.5), (3.0, 6.0), 2.5);
        let mut params: Vec<ParamRef> = pts
            .iter()
            .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
            .collect();
        params.push(ParamRef::CircleRadius(circle));
        check_jacobian_central(
            &Constraint::TangentLineCircle(line, circle),
            &snap,
            &params,
            scale,
        );
    }
}

/// A degenerate line (coincident endpoints) has no direction; residual and
/// Jacobian stay finite instead of dividing by zero.
#[test]
fn tangent_line_circle_degenerate_line_is_finite() {
    let (line, circle, pts, snap) = tangent_snap(1.0, (2.0, 2.0), (2.0, 2.0), (5.0, 6.0), 2.5);
    let c = Constraint::TangentLineCircle(line, circle);
    let mut r = Vec::new();
    eval_residuals(&c, &snap, &mut r);
    assert_eq!(r.len(), 1, "residual count must not change when degenerate");
    assert!(
        r[0].is_finite(),
        "degenerate line must not produce NaN: {r:?}"
    );

    let mut params: Vec<ParamRef> = pts
        .iter()
        .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
        .collect();
    params.push(ParamRef::CircleRadius(circle));
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let n = params.len();
    let mut jac = vec![0.0; n];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: n,
        param_index: &param_index,
    };
    eval_jacobian(&c, &snap, &mut jw, 0);
    assert!(
        jac.iter().all(|v| v.is_finite()),
        "degenerate line must not poison the Jacobian: {jac:?}"
    );
}

/// Snapshot with three free points for point-symmetry cases.
fn three_point_snap(
    scale: f64,
    a: (f64, f64),
    b: (f64, f64),
    center: (f64, f64),
) -> ([PointId; 3], EntitySnapshot) {
    use super::super::entity::{GenArena, PointData};
    let coords = [
        (a.0 * scale, a.1 * scale),
        (b.0 * scale, b.1 * scale),
        (center.0 * scale, center.1 * scale),
    ];
    let mut pts = GenArena::new();
    let ids: Vec<PointId> = coords
        .iter()
        .map(|&(x, y)| pts.insert(PointData { x, y, fixed: false }))
        .collect();
    let snap = EntitySnapshot {
        points: ids.iter().copied().zip(coords).collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    ([ids[0], ids[1], ids[2]], snap)
}

#[test]
fn symmetric_about_point_zero_at_point_mirror() {
    for scale in SCALES {
        let ([p1, p2, center], snap) =
            three_point_snap(scale, (-3.0, 2.0), (7.0, -6.0), (2.0, -2.0));
        let mut r = Vec::new();
        eval_residuals(
            &Constraint::SymmetricAboutPoint(p1, p2, center),
            &snap,
            &mut r,
        );
        assert_eq!(r.len(), 2);
        let tol = 1e-10 * scale.max(1.0);
        assert!(
            r[0].abs() < tol && r[1].abs() < tol,
            "point-mirrored pair at scale {scale}: {r:?}"
        );
    }
}

#[test]
fn symmetric_about_point_fires_off_mirror() {
    let ([p1, p2, center], snap) = three_point_snap(1.0, (-3.0, 2.0), (7.0, -6.0), (0.0, 0.0));
    let mut r = Vec::new();
    eval_residuals(
        &Constraint::SymmetricAboutPoint(p1, p2, center),
        &snap,
        &mut r,
    );
    assert!((r[0] - 2.0).abs() < 1e-12, "x residual: {r:?}");
    assert!((r[1] - (-2.0)).abs() < 1e-12, "y residual: {r:?}");
}

#[test]
fn symmetric_about_point_jacobian_matches_finite_differences() {
    for scale in SCALES {
        let ([p1, p2, center], snap) =
            three_point_snap(scale, (-2.0, 1.0), (4.0, 3.5), (0.5, -1.0));
        let params: Vec<ParamRef> = [p1, p2, center]
            .iter()
            .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
            .collect();
        check_jacobian_central(
            &Constraint::SymmetricAboutPoint(p1, p2, center),
            &snap,
            &params,
            scale,
        );
    }
}

// ── B16 qualification: scale-relative Jacobians at 1e3 and large translation ─
// Covers the missing cells from the pre-existing matrix (fixed-step FD at unit
// scale for the 19 older variants; central at [1e-3,1,1e5] for the 7 newest).
// 1e3 closes the required B16 scale; T = unit-size geometry at (+1e6,−1e6)
// proves translation invariance of the analytic gradients. eps = 1e-6·fd_scale
// with fd_scale = 1e3 for both (1e-03 step): at T the coordinate magnitude is
// 1e6 (ULP ≈ 2e-10), so a 1e-06 step would drown in representable-perturbation
// error (≈2e-10/1e-06 = 2e-04); 1e-03 keeps rounding (≈2e-07) below the 1e-06
// assert while truncation stays O(1e-06) for these residuals.

fn b16_translate_snap(mut snap: EntitySnapshot, dx: f64, dy: f64) -> EntitySnapshot {
    for xy in snap.points.values_mut() {
        xy.0 += dx;
        xy.1 += dy;
    }
    snap
}

#[test]
fn b16_jacobian_point_datum_at_1e3_and_translation() {
    for (scale, ox, oy, fd_scale) in [(1e3, 0.0, 0.0, 1e3), (1.0, 1e6, -1e6, 1e3)] {
        let (p1, p2, snap) = two_point_snap(
            1.0 * scale + ox,
            2.0 * scale + oy,
            3.0 * scale + ox,
            5.0 * scale + oy,
        );
        let params = vec![
            ParamRef::PointX(p1),
            ParamRef::PointY(p1),
            ParamRef::PointX(p2),
            ParamRef::PointY(p2),
        ];
        check_jacobian_central(&Constraint::Coincident(p1, p2), &snap, &params, fd_scale);
        check_jacobian_central(
            &Constraint::Distance(p1, p2, 5.0 * scale),
            &snap,
            &params,
            fd_scale,
        );
        check_jacobian_central(
            &Constraint::FixX(p1, 5.0 * scale + ox),
            &snap,
            &[ParamRef::PointX(p1)],
            fd_scale,
        );
        check_jacobian_central(
            &Constraint::FixY(p1, 2.0 * scale + oy),
            &snap,
            &[ParamRef::PointY(p1)],
            fd_scale,
        );
    }
}

#[test]
fn b16_jacobian_line_orient_at_1e3_and_translation() {
    use super::super::entity::{GenArena, LineData, PointData};
    for (scale, ox, oy, fd_scale) in [(1e3, 0.0, 0.0, 1e3), (1.0, 1e6, -1e6, 1e3)] {
        let mut pts = GenArena::new();
        let mut mk = |x: f64, y: f64| {
            pts.insert(PointData {
                x: x * scale + ox,
                y: y * scale + oy,
                fixed: false,
            })
        };
        let p1 = mk(0.0, 0.0);
        let p2 = mk(3.0, 1.0);
        let p3 = mk(1.0, 2.0);
        let p4 = mk(4.0, 5.0);
        let mut lines = GenArena::new();
        let l1 = lines.insert(LineData { p1, p2 });
        let l2 = lines.insert(LineData { p1: p3, p2: p4 });
        let snap = EntitySnapshot {
            points: [
                (p1, (0.0 * scale + ox, 0.0 * scale + oy)),
                (p2, (3.0 * scale + ox, 1.0 * scale + oy)),
                (p3, (1.0 * scale + ox, 2.0 * scale + oy)),
                (p4, (4.0 * scale + ox, 5.0 * scale + oy)),
            ]
            .into_iter()
            .collect(),
            lines: [(l1, (p1, p2)), (l2, (p3, p4))].into_iter().collect(),
            circles: HashMap::new(),
            arcs: HashMap::new(),
            ellipses: HashMap::new(),
        };
        let params = vec![
            ParamRef::PointX(p1),
            ParamRef::PointY(p1),
            ParamRef::PointX(p2),
            ParamRef::PointY(p2),
            ParamRef::PointX(p3),
            ParamRef::PointY(p3),
            ParamRef::PointX(p4),
            ParamRef::PointY(p4),
        ];
        check_jacobian_central(&Constraint::Horizontal(l1), &snap, &params[0..4], fd_scale);
        check_jacobian_central(&Constraint::Vertical(l1), &snap, &params[0..4], fd_scale);
        check_jacobian_central(&Constraint::Parallel(l1, l2), &snap, &params, fd_scale);
        check_jacobian_central(&Constraint::Perpendicular(l1, l2), &snap, &params, fd_scale);
        check_jacobian_central(&Constraint::Angle(l1, l2, 0.5), &snap, &params, fd_scale);
    }
}

#[test]
fn b16_jacobian_point_on_circle_arc_at_1e3_and_translation() {
    use super::super::entity::{ArcData, CircleData, GenArena, PointData};
    for (scale, ox, oy, fd_scale) in [(1e3, 0.0, 0.0, 1e3), (1.0, 1e6, -1e6, 1e3)] {
        // PointOnCircle
        {
            let mut pts = GenArena::new();
            let center = pts.insert(PointData {
                x: 1.0 * scale + ox,
                y: 2.0 * scale + oy,
                fixed: false,
            });
            let pt = pts.insert(PointData {
                x: 4.0 * scale + ox,
                y: 6.0 * scale + oy,
                fixed: false,
            });
            let mut circles = GenArena::new();
            let circ = circles.insert(CircleData {
                center,
                radius: 3.0 * scale,
            });
            let snap = EntitySnapshot {
                points: [
                    (center, (1.0 * scale + ox, 2.0 * scale + oy)),
                    (pt, (4.0 * scale + ox, 6.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: HashMap::new(),
                circles: [(circ, (center, 3.0 * scale))].into_iter().collect(),
                arcs: HashMap::new(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::PointOnCircle(pt, circ),
                &snap,
                &[
                    ParamRef::PointX(pt),
                    ParamRef::PointY(pt),
                    ParamRef::PointX(center),
                    ParamRef::PointY(center),
                    ParamRef::CircleRadius(circ),
                ],
                fd_scale,
            );
        }
        // PointOnArc
        {
            let mut pts = GenArena::new();
            let center = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let start = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let end = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 2.0 * scale + oy,
                fixed: false,
            });
            let pt = pts.insert(PointData {
                x: 1.5 * scale + ox,
                y: 1.5 * scale + oy,
                fixed: false,
            });
            let mut arcs = GenArena::new();
            let arc = arcs.insert(ArcData { center, start, end });
            let snap = EntitySnapshot {
                points: [
                    (center, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (start, (2.0 * scale + ox, 0.0 * scale + oy)),
                    (end, (0.0 * scale + ox, 2.0 * scale + oy)),
                    (pt, (1.5 * scale + ox, 1.5 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: HashMap::new(),
                circles: HashMap::new(),
                arcs: [(arc, (center, start, end))].into_iter().collect(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::PointOnArc(pt, arc),
                &snap,
                &[
                    ParamRef::PointX(pt),
                    ParamRef::PointY(pt),
                    ParamRef::PointX(center),
                    ParamRef::PointY(center),
                    ParamRef::PointX(start),
                    ParamRef::PointY(start),
                ],
                fd_scale,
            );
        }
        // PointLineDistance
        {
            use super::super::entity::LineData;
            let mut pts = GenArena::new();
            let a = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let b = pts.insert(PointData {
                x: 4.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let p = pts.insert(PointData {
                x: 1.0 * scale + ox,
                y: 3.0 * scale + oy,
                fixed: false,
            });
            let mut lines = GenArena::new();
            let l = lines.insert(LineData { p1: a, p2: b });
            let snap = EntitySnapshot {
                points: [
                    (a, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (b, (4.0 * scale + ox, 0.0 * scale + oy)),
                    (p, (1.0 * scale + ox, 3.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: [(l, (a, b))].into_iter().collect(),
                circles: HashMap::new(),
                arcs: HashMap::new(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::PointLineDistance(p, l, 1.5 * scale),
                &snap,
                &[
                    ParamRef::PointX(a),
                    ParamRef::PointY(a),
                    ParamRef::PointX(b),
                    ParamRef::PointY(b),
                    ParamRef::PointX(p),
                    ParamRef::PointY(p),
                ],
                fd_scale,
            );
        }
    }
}

#[test]
fn b16_jacobian_tangency_equal_arc_at_1e3_and_translation() {
    use super::super::entity::{ArcData, CircleData, GenArena, LineData, PointData};
    for (scale, ox, oy, fd_scale) in [(1e3, 0.0, 0.0, 1e3), (1.0, 1e6, -1e6, 1e3)] {
        // TangentLineArc
        {
            let mut pts = GenArena::new();
            let p1 = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let p2 = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let center = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let start = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let end = pts.insert(PointData {
                x: 3.0 * scale + ox,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let mut lines = GenArena::new();
            let line = lines.insert(LineData { p1, p2 });
            let mut arcs = GenArena::new();
            let arc = arcs.insert(ArcData { center, start, end });
            let snap = EntitySnapshot {
                points: [
                    (p1, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (p2, (2.0 * scale + ox, 0.0 * scale + oy)),
                    (center, (2.0 * scale + ox, 1.0 * scale + oy)),
                    (start, (2.0 * scale + ox, 0.0 * scale + oy)),
                    (end, (3.0 * scale + ox, 1.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: [(line, (p1, p2))].into_iter().collect(),
                circles: HashMap::new(),
                arcs: [(arc, (center, start, end))].into_iter().collect(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::TangentLineArc(line, arc, p2),
                &snap,
                &[
                    ParamRef::PointX(p1),
                    ParamRef::PointY(p1),
                    ParamRef::PointX(p2),
                    ParamRef::PointY(p2),
                    ParamRef::PointX(center),
                    ParamRef::PointY(center),
                ],
                fd_scale,
            );
        }
        // TangentArcArc
        {
            let mut pts = GenArena::new();
            let c1 = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let c2 = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let shared = pts.insert(PointData {
                x: 1.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let s1 = pts.insert(PointData {
                x: 1.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let e1 = pts.insert(PointData {
                x: ox - scale,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let s2 = pts.insert(PointData {
                x: 1.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let e2 = pts.insert(PointData {
                x: 3.0 * scale + ox,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let mut arcs = GenArena::new();
            let arc1 = arcs.insert(ArcData {
                center: c1,
                start: s1,
                end: e1,
            });
            let arc2 = arcs.insert(ArcData {
                center: c2,
                start: s2,
                end: e2,
            });
            let snap = EntitySnapshot {
                points: [
                    (c1, (0.0 * scale + ox, 1.0 * scale + oy)),
                    (c2, (2.0 * scale + ox, 1.0 * scale + oy)),
                    (shared, (1.0 * scale + ox, 0.0 * scale + oy)),
                    (s1, (1.0 * scale + ox, 0.0 * scale + oy)),
                    (e1, (ox - scale, 1.0 * scale + oy)),
                    (s2, (1.0 * scale + ox, 0.0 * scale + oy)),
                    (e2, (3.0 * scale + ox, 1.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: HashMap::new(),
                circles: HashMap::new(),
                arcs: [(arc1, (c1, s1, e1)), (arc2, (c2, s2, e2))]
                    .into_iter()
                    .collect(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::TangentArcArc(arc1, arc2, shared),
                &snap,
                &[
                    ParamRef::PointX(shared),
                    ParamRef::PointY(shared),
                    ParamRef::PointX(c1),
                    ParamRef::PointY(c1),
                    ParamRef::PointX(c2),
                    ParamRef::PointY(c2),
                ],
                fd_scale,
            );
        }
        // EqualRadiusArcArc + EqualRadiusArcCircle + ArcLength + Concentric
        {
            let mut pts = GenArena::new();
            let c1 = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let s1 = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let e1 = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 2.0 * scale + oy,
                fixed: false,
            });
            let c2 = pts.insert(PointData {
                x: 5.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let s2 = pts.insert(PointData {
                x: 8.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let e2 = pts.insert(PointData {
                x: 5.0 * scale + ox,
                y: 3.0 * scale + oy,
                fixed: false,
            });
            let cc = pts.insert(PointData {
                x: 5.0 * scale + ox,
                y: 5.0 * scale + oy,
                fixed: false,
            });
            let mut arcs = GenArena::new();
            let arc1 = arcs.insert(ArcData {
                center: c1,
                start: s1,
                end: e1,
            });
            let arc2 = arcs.insert(ArcData {
                center: c2,
                start: s2,
                end: e2,
            });
            let mut circles = GenArena::new();
            let circ = circles.insert(CircleData {
                center: cc,
                radius: 3.0 * scale,
            });
            let snap = EntitySnapshot {
                points: [
                    (c1, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (s1, (2.0 * scale + ox, 0.0 * scale + oy)),
                    (e1, (0.0 * scale + ox, 2.0 * scale + oy)),
                    (c2, (5.0 * scale + ox, 0.0 * scale + oy)),
                    (s2, (8.0 * scale + ox, 0.0 * scale + oy)),
                    (e2, (5.0 * scale + ox, 3.0 * scale + oy)),
                    (cc, (5.0 * scale + ox, 5.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: HashMap::new(),
                circles: [(circ, (cc, 3.0 * scale))].into_iter().collect(),
                arcs: [(arc1, (c1, s1, e1)), (arc2, (c2, s2, e2))]
                    .into_iter()
                    .collect(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::EqualRadiusArcArc(arc1, arc2),
                &snap,
                &[
                    ParamRef::PointX(c1),
                    ParamRef::PointY(c1),
                    ParamRef::PointX(s1),
                    ParamRef::PointY(s1),
                    ParamRef::PointX(c2),
                    ParamRef::PointY(c2),
                    ParamRef::PointX(s2),
                    ParamRef::PointY(s2),
                ],
                fd_scale,
            );
            check_jacobian_central(
                &Constraint::EqualRadiusArcCircle(arc1, circ),
                &snap,
                &[
                    ParamRef::PointX(c1),
                    ParamRef::PointY(c1),
                    ParamRef::PointX(s1),
                    ParamRef::PointY(s1),
                    ParamRef::CircleRadius(circ),
                ],
                fd_scale,
            );
            check_jacobian_central(
                &Constraint::ArcLength(arc1, std::f64::consts::PI * scale),
                &snap,
                &[
                    ParamRef::PointX(c1),
                    ParamRef::PointY(c1),
                    ParamRef::PointX(s1),
                    ParamRef::PointY(s1),
                    ParamRef::PointX(e1),
                    ParamRef::PointY(e1),
                ],
                fd_scale,
            );
            check_jacobian_central(
                &Constraint::ConcentricArcArc(arc1, arc2),
                &snap,
                &[
                    ParamRef::PointX(c1),
                    ParamRef::PointY(c1),
                    ParamRef::PointX(c2),
                    ParamRef::PointY(c2),
                ],
                fd_scale,
            );
            check_jacobian_central(
                &Constraint::ConcentricArcCircle(arc1, circ),
                &snap,
                &[
                    ParamRef::PointX(c1),
                    ParamRef::PointY(c1),
                    ParamRef::PointX(cc),
                    ParamRef::PointY(cc),
                ],
                fd_scale,
            );
        }
    }
}

#[test]
fn b16_jacobian_new_variants_at_1e3_and_translation() {
    // The 7 newest variants already have central cover at [1e-3,1,1e5]; this
    // closes the required 1e3 scale and the large-translation cell.
    for (scale, ox, oy, fd_scale) in [(1e3, 0.0, 0.0, 1e3), (1.0, 1e6, -1e6, 1e3)] {
        // CircleRadius + EqualRadiusCircleCircle
        {
            use super::super::entity::{CircleData, GenArena, PointData};
            let mut pts = GenArena::new();
            let c1c = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let c2c = pts.insert(PointData {
                x: 10.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let mut circles = GenArena::new();
            let (r1, r2) = (3.0 * scale, 5.0 * scale);
            let circ1 = circles.insert(CircleData {
                center: c1c,
                radius: r1,
            });
            let circ2 = circles.insert(CircleData {
                center: c2c,
                radius: r2,
            });
            let snap = EntitySnapshot {
                points: [
                    (c1c, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (c2c, (10.0 * scale + ox, 0.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: HashMap::new(),
                circles: [(circ1, (c1c, r1)), (circ2, (c2c, r2))]
                    .into_iter()
                    .collect(),
                arcs: HashMap::new(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::CircleRadius(circ1, 2.0 * scale),
                &snap,
                &[ParamRef::CircleRadius(circ1)],
                fd_scale,
            );
            check_jacobian_central(
                &Constraint::EqualRadiusCircleCircle(circ1, circ2),
                &snap,
                &[ParamRef::CircleRadius(circ1), ParamRef::CircleRadius(circ2)],
                fd_scale,
            );
        }
        // EqualLength + Midpoint via two_line_snap-style geometry
        {
            use super::super::entity::{GenArena, LineData, PointData};
            let mut pts = GenArena::new();
            let a = pts.insert(PointData {
                x: -4.0 * scale + ox,
                y: 2.0 * scale + oy,
                fixed: false,
            });
            let b = pts.insert(PointData {
                x: 10.0 * scale + ox,
                y: 8.0 * scale + oy,
                fixed: false,
            });
            let c = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let d = pts.insert(PointData {
                x: 3.0 * scale + ox,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let mut lines = GenArena::new();
            let l1 = lines.insert(LineData { p1: a, p2: b });
            let l2 = lines.insert(LineData { p1: c, p2: d });
            let snap = EntitySnapshot {
                points: [
                    (a, (-4.0 * scale + ox, 2.0 * scale + oy)),
                    (b, (10.0 * scale + ox, 8.0 * scale + oy)),
                    (c, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (d, (3.0 * scale + ox, 1.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: [(l1, (a, b)), (l2, (c, d))].into_iter().collect(),
                circles: HashMap::new(),
                arcs: HashMap::new(),
                ellipses: HashMap::new(),
            };
            let params = vec![
                ParamRef::PointX(a),
                ParamRef::PointY(a),
                ParamRef::PointX(b),
                ParamRef::PointY(b),
                ParamRef::PointX(c),
                ParamRef::PointY(c),
                ParamRef::PointX(d),
                ParamRef::PointY(d),
            ];
            check_jacobian_central(&Constraint::EqualLength(l1, l2), &snap, &params, fd_scale);
            check_jacobian_central(&Constraint::Midpoint(c, l1), &snap, &params, fd_scale);
        }
        // Symmetric + TangentLineCircle + SymmetricAboutPoint at the same
        // placements (review: these three were missing from the 1e3/T close).
        {
            use super::super::entity::{GenArena, LineData, PointData};
            let mut pts = GenArena::new();
            let ax = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: -scale + oy,
                fixed: false,
            });
            let bx = pts.insert(PointData {
                x: 2.0 * scale + ox,
                y: 5.0 * scale + oy,
                fixed: false,
            });
            let p1 = pts.insert(PointData {
                x: -3.0 * scale + ox,
                y: 4.0 * scale + oy,
                fixed: false,
            });
            let p2 = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let mut lines = GenArena::new();
            let axis = lines.insert(LineData { p1: ax, p2: bx });
            let snap = EntitySnapshot {
                points: [
                    (ax, (2.0 * scale + ox, -scale + oy)),
                    (bx, (2.0 * scale + ox, 5.0 * scale + oy)),
                    (p1, (-3.0 * scale + ox, 4.0 * scale + oy)),
                    (p2, (0.0 * scale + ox, 0.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: [(axis, (ax, bx))].into_iter().collect(),
                circles: HashMap::new(),
                arcs: HashMap::new(),
                ellipses: HashMap::new(),
            };
            let params = vec![
                ParamRef::PointX(ax),
                ParamRef::PointY(ax),
                ParamRef::PointX(bx),
                ParamRef::PointY(bx),
                ParamRef::PointX(p1),
                ParamRef::PointY(p1),
                ParamRef::PointX(p2),
                ParamRef::PointY(p2),
            ];
            check_jacobian_central(
                &Constraint::Symmetric(p1, p2, axis),
                &snap,
                &params,
                fd_scale,
            );
        }
        {
            use super::super::entity::{CircleData, GenArena, LineData, PointData};
            let mut pts = GenArena::new();
            let a = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let b = pts.insert(PointData {
                x: 4.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let cc = pts.insert(PointData {
                x: 1.0 * scale + ox,
                y: 5.0 * scale + oy,
                fixed: false,
            });
            let mut lines = GenArena::new();
            let line = lines.insert(LineData { p1: a, p2: b });
            let mut circles = GenArena::new();
            let circ = circles.insert(CircleData {
                center: cc,
                radius: 2.0 * scale,
            });
            let snap = EntitySnapshot {
                points: [
                    (a, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (b, (4.0 * scale + ox, 0.0 * scale + oy)),
                    (cc, (1.0 * scale + ox, 5.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: [(line, (a, b))].into_iter().collect(),
                circles: [(circ, (cc, 2.0 * scale))].into_iter().collect(),
                arcs: HashMap::new(),
                ellipses: HashMap::new(),
            };
            check_jacobian_central(
                &Constraint::TangentLineCircle(line, circ),
                &snap,
                &[
                    ParamRef::PointX(a),
                    ParamRef::PointY(a),
                    ParamRef::PointX(b),
                    ParamRef::PointY(b),
                    ParamRef::PointX(cc),
                    ParamRef::PointY(cc),
                    ParamRef::CircleRadius(circ),
                ],
                fd_scale,
            );
        }
        {
            use super::super::entity::{GenArena, PointData};
            let mut pts = GenArena::new();
            let p1 = pts.insert(PointData {
                x: -2.0 * scale + ox,
                y: 3.0 * scale + oy,
                fixed: false,
            });
            let p2 = pts.insert(PointData {
                x: 0.0 * scale + ox,
                y: 0.0 * scale + oy,
                fixed: false,
            });
            let cc = pts.insert(PointData {
                x: 1.0 * scale + ox,
                y: 1.0 * scale + oy,
                fixed: false,
            });
            let snap = EntitySnapshot {
                points: [
                    (p1, (-2.0 * scale + ox, 3.0 * scale + oy)),
                    (p2, (0.0 * scale + ox, 0.0 * scale + oy)),
                    (cc, (1.0 * scale + ox, 1.0 * scale + oy)),
                ]
                .into_iter()
                .collect(),
                lines: HashMap::new(),
                circles: HashMap::new(),
                arcs: HashMap::new(),
                ellipses: HashMap::new(),
            };
            let params: Vec<ParamRef> = [p1, p2, cc]
                .iter()
                .flat_map(|&id| [ParamRef::PointX(id), ParamRef::PointY(id)])
                .collect();
            check_jacobian_central(
                &Constraint::SymmetricAboutPoint(p1, p2, cc),
                &snap,
                &params,
                fd_scale,
            );
        }
    }
    // b16_translate helper must be exercised (translation invariance is asserted
    // cell-by-cell above via offset snaps); keep the helper live for the audit.
    let (_, _, snap) = two_point_snap(1.0, 2.0, 3.0, 4.0);
    let moved = b16_translate_snap(snap, 1e6, -1e6);
    assert!(
        moved
            .points
            .values()
            .all(|v| v.0.is_finite() && v.1.is_finite())
    );
}

// ── B75 ellipse constraints ─────────────────────────────────────────
// Every analytic Jacobian below is checked against central differences at
// three coordinate scales. Length-like parameters step with the geometry
// scale; the orientation parameter steps in absolute radians — a
// scale-relative step would span ~0.1 rad at 1e5 and drown the
// trigonometric derivatives in truncation error. Angle units are
// scale-invariant (see `EllipseData`), so the split is principled.

/// Central-difference Jacobian check for ellipse constraints, with
/// per-kind steps: `1e-6 * scale` for length-like parameters (point
/// coordinates, semiaxes), `1e-7` absolute for the orientation angle.
fn check_ellipse_jacobian_central(
    c: &Constraint,
    snap: &EntitySnapshot,
    params: &[ParamRef],
    scale: f64,
) {
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let n = params.len();
    let m = residual_count(c);

    let mut jac = vec![0.0; m * n];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: n,
        param_index: &param_index,
    };
    eval_jacobian(c, snap, &mut jw, 0);

    for (col, pr) in params.iter().enumerate() {
        // Orientation is scale-invariant; everything else scales.
        let eps = match pr {
            ParamRef::EllipsePhi(_) => 1e-7,
            _ => 1e-6 * scale,
        };
        let mut r_plus = Vec::new();
        eval_residuals(c, &perturb(snap, *pr, eps), &mut r_plus);
        let mut r_minus = Vec::new();
        eval_residuals(c, &perturb(snap, *pr, -eps), &mut r_minus);

        for row in 0..m {
            let fd = (r_plus[row] - r_minus[row]) / (2.0 * eps);
            let analytic = jac[row * n + col];
            let err = (fd - analytic).abs();
            assert!(
                err < 1e-6 * 1.0_f64.max(analytic.abs()),
                "Jacobian mismatch at ({row},{col}) scale={scale}: \
                 analytic={analytic}, fd={fd}, err={err}"
            );
        }
    }
}

/// Build a snapshot holding one ellipse plus caller-supplied extra points.
///
/// Returns `(center, ellipse, extra ids, snapshot)`. Arenas are dropped on
/// return; handles stay valid as snapshot keys (same pattern as
/// `two_point_snap`).
fn ellipse_fixture(
    cx: f64,
    cy: f64,
    a: f64,
    b: f64,
    phi: f64,
    extras: &[(f64, f64)],
) -> (PointId, EllipseId, Vec<PointId>, EntitySnapshot) {
    use super::super::entity::{EllipseData, GenArena, PointData};
    let mut pts = GenArena::new();
    let center = pts.insert(PointData {
        x: cx,
        y: cy,
        fixed: false,
    });
    let mut extra_ids = Vec::with_capacity(extras.len());
    for (x, y) in extras {
        extra_ids.push(pts.insert(PointData {
            x: *x,
            y: *y,
            fixed: false,
        }));
    }
    let mut ells = GenArena::new();
    let ell = ells.insert(EllipseData {
        center,
        a,
        b,
        angle: phi,
    });
    let mut points: HashMap<PointId, (f64, f64)> = HashMap::new();
    points.insert(center, (cx, cy));
    for (id, (x, y)) in extra_ids.iter().zip(extras.iter()) {
        points.insert(*id, (*x, *y));
    }
    let snap = EntitySnapshot {
        points,
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: [(ell, (center, a, b, phi))].into_iter().collect(),
    };
    (center, ell, extra_ids, snap)
}

/// Point on the ellipse curve at curve parameter `t`.
fn ellipse_point(cx: f64, cy: f64, a: f64, b: f64, phi: f64, t: f64) -> (f64, f64) {
    let (s, c) = phi.sin_cos();
    let (st, ct) = t.sin_cos();
    (cx + a * ct * c - b * st * s, cy + a * ct * s + b * st * c)
}

/// Tangent direction of the ellipse curve at parameter `t` (unnormalized).
fn ellipse_tangent(a: f64, b: f64, phi: f64, t: f64) -> (f64, f64) {
    let (s, c) = phi.sin_cos();
    let (st, ct) = t.sin_cos();
    (-a * st * c - b * ct * s, -a * st * s + b * ct * c)
}

#[test]
fn point_on_ellipse_residual_and_jacobian() {
    for scale in SCALES {
        let (cx, cy, a, b, phi) = (1.0 * scale, 2.0 * scale, 3.0 * scale, 2.0 * scale, 0.5);
        let t = 0.7;
        let (px, py) = ellipse_point(cx, cy, a, b, phi, t);
        let (center, ell, extras, snap) = ellipse_fixture(cx, cy, a, b, phi, &[(px, py)]);
        let pt = extras[0];

        // On-curve: zero residual.
        let c = Constraint::PointOnEllipse(pt, ell);
        let mut r = Vec::new();
        eval_residuals(&c, &snap, &mut r);
        assert_eq!(r.len(), 1);
        assert!(r[0].abs() < 1e-12, "on-curve residual {}", r[0]);

        // Off-curve (center): residual -1.
        let cc = Constraint::PointOnEllipse(center, ell);
        let mut r2 = Vec::new();
        eval_residuals(&cc, &snap, &mut r2);
        assert!((r2[0] + 1.0).abs() < 1e-12, "center residual {}", r2[0]);

        check_ellipse_jacobian_central(
            &c,
            &snap,
            &[
                ParamRef::PointX(pt),
                ParamRef::PointY(pt),
                ParamRef::PointX(center),
                ParamRef::PointY(center),
                ParamRef::EllipseA(ell),
                ParamRef::EllipseB(ell),
                ParamRef::EllipsePhi(ell),
            ],
            scale,
        );
    }
}

#[test]
fn point_on_ellipse_rotated_high_eccentricity() {
    // High eccentricity (10:1) with a near-quarter-turn orientation: the
    // phi column is large here, the opposite corner from near-circles.
    for scale in SCALES {
        let (cx, cy, a, b, phi) = (1.0 * scale, -scale, 10.0 * scale, 1.0 * scale, 1.4);
        let t = 2.1;
        let (px, py) = ellipse_point(cx, cy, a, b, phi, t);
        let (center, ell, extras, snap) = ellipse_fixture(cx, cy, a, b, phi, &[(px, py)]);
        let pt = extras[0];
        let c = Constraint::PointOnEllipse(pt, ell);
        let mut r = Vec::new();
        eval_residuals(&c, &snap, &mut r);
        assert!(r[0].abs() < 1e-12, "on-curve residual {}", r[0]);
        check_ellipse_jacobian_central(
            &c,
            &snap,
            &[
                ParamRef::PointX(pt),
                ParamRef::PointY(pt),
                ParamRef::PointX(center),
                ParamRef::PointY(center),
                ParamRef::EllipseA(ell),
                ParamRef::EllipseB(ell),
                ParamRef::EllipsePhi(ell),
            ],
            scale,
        );
    }
}

#[test]
fn point_on_ellipse_phi_column_vanishes_at_equal_axes() {
    // The documented orientation indeterminacy: at a == b the curve is a
    // circle, the residual is phi-independent, and the analytic phi entry
    // is exactly zero.
    let scale = 1.0;
    let (cx, cy, a, phi) = (1.0 * scale, 2.0 * scale, 3.0 * scale, 0.5);
    let (px, py) = ellipse_point(cx, cy, a, a, phi, 0.7);
    let (_center, ell, extras, snap) = ellipse_fixture(cx, cy, a, a, phi, &[(px, py)]);
    let pt = extras[0];
    let c = Constraint::PointOnEllipse(pt, ell);
    let params = vec![ParamRef::EllipsePhi(ell)];
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    let mut jac = vec![0.0; 1];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: 1,
        param_index: &param_index,
    };
    eval_jacobian(&c, &snap, &mut jw, 0);
    // dru*v - drv*u with bitwise-identical 1/a² factors: cancellation to
    // rounding level. The contract is "vanishes", not "bitwise zero".
    assert!(
        jac[0].abs() < 1e-15,
        "phi column must vanish for a == b, got {}",
        jac[0]
    );
    check_ellipse_jacobian_central(&c, &snap, &params, scale);
}

#[test]
fn point_on_ellipse_periodicity_contract() {
    // (a, b, phi) ~ (a, b, phi + π): identical residuals at the same point.
    let (cx, cy, a, b, phi) = (1.0, 2.0, 3.0, 2.0, 0.5);
    let (px, py) = ellipse_point(cx, cy, a, b, phi, 0.7);
    let (_, ell, extras, snap) = ellipse_fixture(cx, cy, a, b, phi, &[(px, py)]);
    let (_, ell2, _, snap2) =
        ellipse_fixture(cx, cy, a, b, phi + std::f64::consts::PI, &[(px, py)]);
    let pt = extras[0];
    let mut r1 = Vec::new();
    eval_residuals(&Constraint::PointOnEllipse(pt, ell), &snap, &mut r1);
    // Re-key: snap2 holds different handles; evaluate the same geometric
    // query by rebuilding with ell2's own point handle.
    let pt2 = snap2
        .points
        .iter()
        .find(|kv| kv.1 == &(px, py))
        .map(|(id, _)| *id)
        .unwrap();
    let mut r2 = Vec::new();
    eval_residuals(&Constraint::PointOnEllipse(pt2, ell2), &snap2, &mut r2);
    assert!((r1[0] - r2[0]).abs() < 1e-12, "{r1:?} vs {r2:?}");
}

#[test]
fn concentric_ellipse_residuals_and_jacobians() {
    use super::super::entity::{ArcData, CircleData, GenArena, LineData, PointData};
    for scale in SCALES {
        let mut pts = GenArena::new();
        let c1 = pts.insert(PointData {
            x: 1.0 * scale,
            y: 2.0 * scale,
            fixed: false,
        });
        let c2 = pts.insert(PointData {
            x: 4.0 * scale,
            y: -scale,
            fixed: false,
        });
        let mut ells = GenArena::new();
        let e1 = ells.insert(super::super::entity::EllipseData {
            center: c1,
            a: 3.0 * scale,
            b: 2.0 * scale,
            angle: 0.5,
        });
        let e2 = ells.insert(super::super::entity::EllipseData {
            center: c2,
            a: 5.0 * scale,
            b: 1.0 * scale,
            angle: -0.3,
        });
        let mut circs = GenArena::new();
        let circ = circs.insert(CircleData {
            center: c2,
            radius: 2.0 * scale,
        });
        let mut arc_arena = GenArena::<ArcData>::new();
        let arc = arc_arena.insert(ArcData {
            center: c2,
            start: c1,
            end: c1,
        });
        let _ = LineData { p1: c1, p2: c2 };
        let snap = EntitySnapshot {
            points: [
                (c1, (1.0 * scale, 2.0 * scale)),
                (c2, (4.0 * scale, -scale)),
            ]
            .into_iter()
            .collect(),
            lines: HashMap::new(),
            circles: [(circ, (c2, 2.0 * scale))].into_iter().collect(),
            arcs: [(arc, (c2, c1, c1))].into_iter().collect(),
            ellipses: [
                (e1, (c1, 3.0 * scale, 2.0 * scale, 0.5)),
                (e2, (c2, 5.0 * scale, 1.0 * scale, -0.3)),
            ]
            .into_iter()
            .collect(),
        };
        let params = vec![
            ParamRef::PointX(c1),
            ParamRef::PointY(c1),
            ParamRef::PointX(c2),
            ParamRef::PointY(c2),
        ];
        for (c, name) in [
            (Constraint::ConcentricEllipseEllipse(e1, e2), "ee"),
            (Constraint::ConcentricEllipseCircle(e1, circ), "ec"),
            (Constraint::ConcentricEllipseArc(e1, arc), "ea"),
        ] {
            let mut r = Vec::new();
            eval_residuals(&c, &snap, &mut r);
            assert_eq!(r.len(), 2, "{name}");
            assert!(
                (r[0] - (1.0 * scale - 4.0 * scale)).abs() < 1e-9 * scale.max(1.0),
                "{name} {r:?}"
            );
            assert!(
                (r[1] - (2.0 * scale + 1.0 * scale)).abs() < 1e-9 * scale.max(1.0),
                "{name} {r:?}"
            );
            check_ellipse_jacobian_central(&c, &snap, &params, scale);
        }
    }
}

#[test]
fn tangent_line_ellipse_residual_and_jacobian() {
    use super::super::entity::{GenArena, LineData, PointData};
    for scale in SCALES {
        let (cx, cy, a, b, phi) = (1.0 * scale, 2.0 * scale, 4.0 * scale, 2.0 * scale, 0.5);
        let t = 0.9;
        let (qx, qy) = ellipse_point(cx, cy, a, b, phi, t);
        let (tx, ty) = ellipse_tangent(a, b, phi, t);
        // Line through the contact along the tangent.
        let (ax, ay) = (qx - tx, qy - ty);
        let (bx, by) = (qx + tx, qy + ty);
        let mut pts = GenArena::new();
        let center = pts.insert(PointData {
            x: cx,
            y: cy,
            fixed: false,
        });
        let contact = pts.insert(PointData {
            x: qx,
            y: qy,
            fixed: false,
        });
        let p1 = pts.insert(PointData {
            x: ax,
            y: ay,
            fixed: false,
        });
        let p2 = pts.insert(PointData {
            x: bx,
            y: by,
            fixed: false,
        });
        let mut lines = GenArena::new();
        let line = lines.insert(LineData { p1, p2 });
        let mut ells = GenArena::new();
        let ell = ells.insert(super::super::entity::EllipseData {
            center,
            a,
            b,
            angle: phi,
        });
        let snap = EntitySnapshot {
            points: [
                (center, (cx, cy)),
                (contact, (qx, qy)),
                (p1, (ax, ay)),
                (p2, (bx, by)),
            ]
            .into_iter()
            .collect(),
            lines: [(line, (p1, p2))].into_iter().collect(),
            circles: HashMap::new(),
            arcs: HashMap::new(),
            ellipses: [(ell, (center, a, b, phi))].into_iter().collect(),
        };
        let c = Constraint::TangentLineEllipse(line, ell, contact);
        let mut r = Vec::new();
        eval_residuals(&c, &snap, &mut r);
        assert_eq!(r.len(), 1);
        // Tangent construction: residual ~0 up to the unnormalized scaling.
        // Normalize by |line_dir|·|gradient| for the assertion.
        let lx = bx - ax;
        let ly = by - ay;
        assert!(
            r[0].abs() / (lx.hypot(ly) + 1e-300) < 1e-9 * scale.max(1.0).max(lx.hypot(ly)),
            "tangent residual {}",
            r[0]
        );
        check_ellipse_jacobian_central(
            &c,
            &snap,
            &[
                ParamRef::PointX(p1),
                ParamRef::PointY(p1),
                ParamRef::PointX(p2),
                ParamRef::PointY(p2),
                ParamRef::PointX(contact),
                ParamRef::PointY(contact),
                ParamRef::PointX(center),
                ParamRef::PointY(center),
                ParamRef::EllipseA(ell),
                ParamRef::EllipseB(ell),
                ParamRef::EllipsePhi(ell),
            ],
            scale,
        );
    }
}

#[test]
fn ellipse_axis_and_angle_residuals_and_jacobians() {
    for scale in SCALES {
        let (cx, cy, a, b, phi) = (1.0 * scale, 2.0 * scale, 3.0 * scale, 2.0 * scale, 0.5);
        let (center, ell, _, snap) = ellipse_fixture(cx, cy, a, b, phi, &[]);
        let params_full = vec![
            ParamRef::PointX(center),
            ParamRef::PointY(center),
            ParamRef::EllipseA(ell),
            ParamRef::EllipseB(ell),
            ParamRef::EllipsePhi(ell),
        ];
        // At-target: zero; off-target: signed difference.
        let mut r = Vec::new();
        eval_residuals(&Constraint::EllipseAxisA(ell, a), &snap, &mut r);
        assert!(r[0].abs() < 1e-12 * scale.max(1.0), "{}", r[0]);
        let mut r = Vec::new();
        eval_residuals(&Constraint::EllipseAxisB(ell, 1.0 * scale), &snap, &mut r);
        assert!((r[0] - scale).abs() < 1e-12 * scale.max(1.0), "{r:?}");
        let mut r = Vec::new();
        eval_residuals(&Constraint::EllipseAngle(ell, phi), &snap, &mut r);
        assert!(r[0].abs() < 1e-15, "{}", r[0]);
        // π-shifted target describes the same orientation (sign flip only).
        let mut r = Vec::new();
        eval_residuals(
            &Constraint::EllipseAngle(ell, phi + std::f64::consts::PI),
            &snap,
            &mut r,
        );
        assert!(r[0].abs() < 1e-15, "{}", r[0]);

        check_ellipse_jacobian_central(
            &Constraint::EllipseAxisA(ell, a),
            &snap,
            &params_full,
            scale,
        );
        check_ellipse_jacobian_central(
            &Constraint::EllipseAxisB(ell, b),
            &snap,
            &params_full,
            scale,
        );
        check_ellipse_jacobian_central(
            &Constraint::EllipseAngle(ell, phi + 0.1),
            &snap,
            &params_full,
            scale,
        );
    }
}

#[test]
fn equal_ellipse_radii_residual_and_jacobian() {
    use super::super::entity::GenArena;
    for scale in SCALES {
        let mut pts = GenArena::new();
        let c1 = pts.insert(super::super::entity::PointData {
            x: 0.0,
            y: 0.0,
            fixed: false,
        });
        let c2 = pts.insert(super::super::entity::PointData {
            x: 5.0 * scale,
            y: 0.0,
            fixed: false,
        });
        let mut ells = GenArena::new();
        let e1 = ells.insert(super::super::entity::EllipseData {
            center: c1,
            a: 3.0 * scale,
            b: 2.0 * scale,
            angle: 0.1,
        });
        let e2 = ells.insert(super::super::entity::EllipseData {
            center: c2,
            a: 3.0 * scale,
            b: 4.0 * scale,
            angle: -0.2,
        });
        let snap = EntitySnapshot {
            points: [(c1, (0.0, 0.0)), (c2, (5.0 * scale, 0.0))]
                .into_iter()
                .collect(),
            lines: HashMap::new(),
            circles: HashMap::new(),
            arcs: HashMap::new(),
            ellipses: [
                (e1, (c1, 3.0 * scale, 2.0 * scale, 0.1)),
                (e2, (c2, 3.0 * scale, 4.0 * scale, -0.2)),
            ]
            .into_iter()
            .collect(),
        };
        let c = Constraint::EqualEllipseRadii(e1, e2);
        let mut r = Vec::new();
        eval_residuals(&c, &snap, &mut r);
        assert_eq!(r.len(), 2);
        assert!(r[0].abs() < 1e-12 * scale.max(1.0), "{r:?}");
        assert!((r[1] + 2.0 * scale).abs() < 1e-12 * scale.max(1.0), "{r:?}");
        check_ellipse_jacobian_central(
            &c,
            &snap,
            &[
                ParamRef::EllipseA(e1),
                ParamRef::EllipseB(e1),
                ParamRef::EllipsePhi(e1),
                ParamRef::EllipseA(e2),
                ParamRef::EllipseB(e2),
                ParamRef::EllipsePhi(e2),
                ParamRef::PointX(c1),
            ],
            scale,
        );
    }
}

#[test]
fn ellipse_degeneracy_contract() {
    use super::super::entity::{GenArena, PointData};
    // Degenerate axes read satisfied-with-zero-gradient (the line-axis
    // contract), never NaN or inf; non-finite axes poison to NaN.
    let mut pts = GenArena::new();
    let center = pts.insert(PointData {
        x: 0.0,
        y: 0.0,
        fixed: false,
    });
    let pt = pts.insert(PointData {
        x: 1.0,
        y: 0.0,
        fixed: false,
    });
    let mut ells = GenArena::new();
    let deg = ells.insert(super::super::entity::EllipseData {
        center,
        a: 0.0,
        b: 1.0,
        angle: 0.0,
    });
    let nan_ax = ells.insert(super::super::entity::EllipseData {
        center,
        a: f64::NAN,
        b: 1.0,
        angle: 0.0,
    });
    let snap = EntitySnapshot {
        points: [(center, (0.0, 0.0)), (pt, (1.0, 0.0))]
            .into_iter()
            .collect(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: [
            (deg, (center, 0.0, 1.0, 0.0)),
            (nan_ax, (center, f64::NAN, 1.0, 0.0)),
        ]
        .into_iter()
        .collect(),
    };
    let mut r = Vec::new();
    eval_residuals(&Constraint::PointOnEllipse(pt, deg), &snap, &mut r);
    // Degenerate axes push exactly 0.0 (the documented fail-quiet value).
    assert!(
        r[0] == 0.0,
        "degenerate axis must read satisfied, got {}",
        r[0]
    );
    let mut r = Vec::new();
    eval_residuals(&Constraint::PointOnEllipse(pt, nan_ax), &snap, &mut r);
    assert!(r[0].is_nan(), "non-finite axis must poison, got {}", r[0]);
    // Jacobian on the degenerate row writes nothing (stays zero).
    let params = [ParamRef::PointX(pt), ParamRef::EllipseA(deg)];
    let param_index: HashMap<ParamRef, usize> =
        params.iter().enumerate().map(|(i, p)| (*p, i)).collect();
    // Pre-zero like production does, then confirm nothing is written.
    let mut jac = vec![0.0; 2];
    let mut jw = JacobianWriter {
        data: &mut jac,
        ncols: 2,
        param_index: &param_index,
    };
    eval_jacobian(&Constraint::PointOnEllipse(pt, deg), &snap, &mut jw, 0);
    assert_eq!(jac, vec![0.0, 0.0]);
}
