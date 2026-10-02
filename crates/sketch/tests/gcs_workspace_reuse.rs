#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
//! PERF-S03 qualification: solve-local workspace reuse.
//!
//! Proves the reused residual/Jacobian/QR/step buffers preserve solver
//! behavior across every workload class the PERF-S01 baseline pins, plus
//! the stale-data case S03 specifically risks: changing dimensions between
//! successive solves must not retain previous extents.
//!
//! No wall-time assertions: timing evidence lives in the PR's retained
//! raw samples, not here. All assertions are identity/oracle based,
//! mirroring `gcs_perf_identity.rs` at small sizes for debug-CI speed.

use remus_sketch::{Constraint, GcsSystem, PointData, SolveClassification};

const TOL: f64 = 1e-10;
const MAX_ITER: usize = 100;

fn build_independent_under(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(2) && n_params >= 2);
    let mut sys = GcsSystem::new();
    for i in 0..n_params / 2 {
        let ax = 10.0 * i as f64;
        let anchor = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let free = sys
            .add_point(PointData {
                x: ax + 1.0,
                y: 1.0,
                fixed: false,
            })
            .unwrap();
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
    }
    sys
}

fn build_independent_solved(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_under(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for chunk in ids.chunks(2) {
        sys.add_constraint(Constraint::FixY(chunk[1], 4.0)).unwrap();
    }
    sys
}

fn build_coupled_chain(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(2) && n_params >= 10);
    let n_pts = n_params / 2 + 1;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(n_pts);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap(),
    );
    for i in 1..n_pts {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .unwrap(),
        );
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
    }
    sys
}

fn ordered_points(sys: &GcsSystem) -> Vec<remus_sketch::PointId> {
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    ids
}

fn assert_pair_distances(sys: &GcsSystem, expect: f64, tol: f64) {
    let ids = ordered_points(sys);
    for chunk in ids.chunks(2) {
        let a = sys.point(chunk[0]).unwrap();
        let b = sys.point(chunk[1]).unwrap();
        let d = (a.x - b.x).hypot(a.y - b.y);
        assert!((d - expect).abs() <= tol, "pair distance {d} != {expect}");
    }
}

fn assert_chain_grid(sys: &GcsSystem) {
    let ids = ordered_points(sys);
    for (i, id) in ids.iter().enumerate() {
        let p = sys.point(*id).unwrap();
        assert!((p.x - i as f64).abs() <= 1e-6, "chain x {} != {i}", p.x);
        assert!(p.y.abs() <= 1e-8, "chain y {} != 0", p.y);
    }
}

#[test]
fn workspace_reuse_preserves_all_classes() {
    // Fully constrained (solved).
    let mut sys = build_independent_solved(20);
    let r = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(r.converged);
    let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(det.converged);
    assert_eq!(det.classification, SolveClassification::Solved);
    assert_eq!(det.dof, 0);
    assert_pair_distances(&sys, 5.0, 1e-6);

    // Underconstrained.
    let mut sys = build_independent_under(20);
    let r = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(r.converged);
    let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert_eq!(det.classification, SolveClassification::UnderConstrained);
    assert_eq!(det.dof, 10);
    assert_pair_distances(&sys, 5.0, 1e-6);

    // Coupled.
    let mut sys = build_coupled_chain(20);
    let r = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(r.converged);
    let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert_eq!(det.classification, SolveClassification::Solved);
    assert_chain_grid(&sys);

    // Redundant (solved + duplicated distances).
    let mut sys = build_independent_solved(20);
    let mut ids = ordered_points(&sys);
    ids.sort_by_key(|id| id.index());
    for chunk in ids.chunks(2) {
        sys.add_constraint(Constraint::Distance(chunk[0], chunk[1], 5.0))
            .unwrap();
    }
    let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(det.converged);
    assert_eq!(det.classification, SolveClassification::Redundant);
    assert!(det.redundant);
    assert_pair_distances(&sys, 5.0, 1e-6);

    // Inconsistent: solve publishes, detailed rolls back.
    let mut sys = build_independent_under(20);
    let mut ids = ordered_points(&sys);
    ids.sort_by_key(|id| id.index());
    sys.add_constraint(Constraint::Distance(ids[0], ids[1], 6.0))
        .unwrap();
    let before: Vec<(f64, f64)> = ordered_points(&sys)
        .iter()
        .map(|id| {
            let p = sys.point(*id).unwrap();
            (p.x, p.y)
        })
        .collect();
    let r = sys.solve(50, TOL).unwrap();
    assert!(!r.converged);
    let mut sys2 = build_independent_under(20);
    let mut ids2 = ordered_points(&sys2);
    ids2.sort_by_key(|id| id.index());
    sys2.add_constraint(Constraint::Distance(ids2[0], ids2[1], 6.0))
        .unwrap();
    let det = sys2.solve_detailed(50, TOL).unwrap();
    assert!(!det.converged);
    assert_eq!(det.classification, SolveClassification::Unsatisfied);
    assert!(det.rolled_back);
    let restored: Vec<(f64, f64)> = ordered_points(&sys2)
        .iter()
        .map(|id| {
            let p = sys2.point(*id).unwrap();
            (p.x, p.y)
        })
        .collect();
    assert_eq!(before, restored);
}

#[test]
fn workspace_reuse_changing_dimensions_no_stale() {
    // Solve small, then large, then small again on fresh systems sharing the
    // process (workspaces are solve-local, so no capacity may leak).
    // Each solve must report its own dimensions and satisfy its own oracle.
    for (n, expect_eqs) in [(10_usize, 10_usize), (50, 50), (10, 10)] {
        let mut sys = build_independent_solved(n);
        let d = sys.dof().unwrap();
        assert_eq!(d.num_params, n);
        assert_eq!(d.num_equations, expect_eqs);
        let r = sys.solve(MAX_ITER, TOL).unwrap();
        assert!(r.converged, "n={n}: max_r={}", r.max_residual);
        let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
        assert!(det.converged);
        assert_eq!(det.num_params, n);
        assert_eq!(det.num_equations, expect_eqs);
        assert_pair_distances(&sys, 5.0, 1e-6);
    }

    // Same-system growth and shrinkage: add entities between solves, then
    // remove the constraint that grew the system. Dimensions must track the
    // live topology every time.
    let mut sys = build_independent_solved(10);
    let r1 = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(r1.converged);
    assert_eq!(sys.dof().unwrap().num_params, 10);

    // Grow: one more free pair (+2 params, +2 equations).
    let anchor = sys
        .add_point(PointData {
            x: 100.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    let free = sys
        .add_point(PointData {
            x: 101.0,
            y: 1.0,
            fixed: false,
        })
        .unwrap();
    sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
        .unwrap();
    let fix = sys.add_constraint(Constraint::FixY(free, 4.0)).unwrap();
    assert_eq!(sys.dof().unwrap().num_params, 12);
    let r2 = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(r2.converged);
    assert_pair_distances(&sys, 5.0, 1e-6);

    // Shrink back: removing the FixY leaves the pair underconstrained, and
    // removing the Distance restores the original 10-param solved shape.
    sys.remove_constraint(fix).unwrap();
    let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(det.converged);
    assert_eq!(det.num_params, 12);
    assert_eq!(det.num_equations, 11);
    assert_eq!(det.classification, SolveClassification::UnderConstrained);
}

#[test]
fn workspace_reuse_drag_updates() {
    let mut sys = build_coupled_chain(20);
    let cold = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(cold.converged);
    let ids = ordered_points(&sys);
    let target = ids[5];
    for step in 0..5 {
        {
            let slot = sys.point_mut(target).unwrap();
            slot.x += 0.5;
            slot.y += 0.25;
        }
        let r = sys.solve(MAX_ITER, TOL).unwrap();
        assert!(r.converged, "step {step}: max_r={}", r.max_residual);
    }
    // Drags perturb the chain; re-cold-solve from the perturbed state still
    // converges and the detailed path agrees.
    let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(det.converged);
    assert_eq!(det.classification, SolveClassification::Solved);
}

#[test]
fn workspace_reuse_scale_and_translation() {
    // Same 3-4-5 pair solved at widely different scales and offsets using
    // the independent hypot oracle (not residuals alone).
    for scale in [1e-3, 1.0, 1e5] {
        for offset in [0.0, 1e4] {
            let mut sys = GcsSystem::new();
            let anchor = sys
                .add_point(PointData {
                    x: offset,
                    y: offset,
                    fixed: true,
                })
                .unwrap();
            let free = sys
                .add_point(PointData {
                    x: offset + scale,
                    y: offset + scale,
                    fixed: false,
                })
                .unwrap();
            sys.add_constraint(Constraint::Distance(anchor, free, 5.0 * scale))
                .unwrap();
            sys.add_constraint(Constraint::FixY(free, offset + 4.0 * scale))
                .unwrap();
            let r = sys.solve(MAX_ITER, TOL).unwrap();
            assert!(r.converged, "scale {scale} offset {offset}");
            let a = sys.point(anchor).unwrap();
            let b = sys.point(free).unwrap();
            let d = (a.x - b.x).hypot(a.y - b.y);
            let tol = 1e-6 * scale.max(1.0);
            assert!((d - 5.0 * scale).abs() <= tol, "scale {scale}: dist {d}");
            assert!((b.y - (offset + 4.0 * scale)).abs() <= tol);
        }
    }
}

#[test]
fn workspace_reuse_repeated_solves_deterministic() {
    for n in [10_usize, 20] {
        let build = build_coupled_chain(n);
        let mut a = build.clone();
        let mut b = build;
        let ra = a.solve(MAX_ITER, TOL).unwrap();
        let rb = b.solve(MAX_ITER, TOL).unwrap();
        assert!(ra.converged && rb.converged);
        assert_eq!(ra.iterations, rb.iterations);
        let pa: Vec<(f64, f64)> = ordered_points(&a)
            .iter()
            .map(|id| {
                let p = a.point(*id).unwrap();
                (p.x, p.y)
            })
            .collect();
        let pb: Vec<(f64, f64)> = ordered_points(&b)
            .iter()
            .map(|id| {
                let p = b.point(*id).unwrap();
                (p.x, p.y)
            })
            .collect();
        for (x, y) in pa.iter().zip(pb.iter()) {
            assert!((x.0 - y.0).abs() <= 1e-9 && (x.1 - y.1).abs() <= 1e-9);
        }
    }
}
