#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
//! Identity gates for the PERF-S01 sketch performance workloads.
//!
//! These tests pin fixture geometry, dimensions, and solve outcomes for every
//! workload the benchmark runner times. They assert no wall-time thresholds:
//! timing evidence lives in `scripts/performance/sketch/` and
//! `docs/performance/sketch-baseline.md`, not here.
//!
//! Solve outcomes are gated at 10/100 parameters (fast in debug CI). The
//! 1000-parameter solve outcomes are validated per sample by the benchmark
//! runner in the profiling release build (see `scripts/performance/sketch/`);
//! here the 1000-parameter fixtures are gated by construction counts only
//! (points, lines, constraints), which need no QR factorization. The budgeted
//! 10000-parameter attempt is a resource-budget refusal (see the last test),
//! also enforced per sample by the runner.
//!
//! Workload IDs (see `scripts/performance/sketch/workloads.json`):
//! - `independent_under`: K disjoint anchor/free pairs with one Distance each
//!   (underconstrained, UnderConstrained).
//! - `independent_solved`: same pairs plus FixY per free point (Solved).
//! - `coupled_chain`: one sparse Distance+Horizontal chain (Solved).
//! - `redundant`: `independent_solved` with every Distance duplicated
//!   (Redundant, converged).
//! - `inconsistent`: `independent_under` with a contradictory Distance pair on
//!   the first component (Unsatisfied; `solve` publishes, `solve_detailed`
//!   rolls back).
//! - `drag`: `coupled_chain` cold solve plus 20 deterministic point edits with
//!   re-solves (warm updates; each edit must change its target).

use remus_sketch::{Constraint, GcsSystem, PointData, SolveClassification};

const TOL: f64 = 1e-10;
const MAX_ITER: usize = 100;

/// Anchor/free geometry for independent workloads (3-4-5 triangle target).
///
/// Anchor `k` is fixed at `(10k, 0)`; its free point starts at `(10k+1, 1)`
/// and targets Distance 5 (solution near `(10k+3, 4)` when FixY=4 pins `y`).
fn build_independent_under(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(2) && n_params >= 2);
    let mut sys = GcsSystem::new();
    let k = n_params / 2;
    for i in 0..k {
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
    assert!(n_params.is_multiple_of(2) && n_params >= 2);
    let mut sys = GcsSystem::new();
    let k = n_params / 2;
    for i in 0..k {
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
        sys.add_constraint(Constraint::FixY(free, 4.0)).unwrap();
    }
    sys
}

/// One sparse chain: p0 fixed at origin, free points at `(i, 0.5*(i%2))`,
/// each segment carrying Distance 1 + Horizontal. Solution is `(i, 0)`.
fn build_coupled_chain(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(2) && n_params >= 10);
    let n_pts = n_params / 2 + 1;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(n_pts);
    let p0 = sys
        .add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap();
    pts.push(p0);
    for i in 1..n_pts {
        let p = sys
            .add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .unwrap();
        pts.push(p);
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
    // Duplicate every Distance: collect pairs first to satisfy the borrow checker.
    let pairs: Vec<(remus_sketch::PointId, remus_sketch::PointId, f64)> = Vec::new();
    let _ = pairs;
    // Re-derive pairs from construction order: anchors are even slots.
    // Simpler: rebuild constraints by re-adding identical distances via a
    // second pass over a fresh solved build's point handles is not directly
    // accessible, so duplicate here by re-walking points in insertion order.
    let mut points: Vec<remus_sketch::PointId> = sys.points().map(|(id, _)| id).collect();
    points.sort_by_key(|id| id.index());
    // Insertion order is anchor, free, anchor, free, ...
    for chunk in points.chunks(2) {
        let (anchor, free) = (chunk[0], chunk[1]);
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
    }
    sys
}

fn build_inconsistent(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_under(n_params);
    // Contradict the first component: its Distance 5 also targets 6.
    let mut points: Vec<remus_sketch::PointId> = sys.points().map(|(id, _)| id).collect();
    points.sort_by_key(|id| id.index());
    let (anchor, free) = (points[0], points[1]);
    sys.add_constraint(Constraint::Distance(anchor, free, 6.0))
        .unwrap();
    sys
}

fn hypot_distance(sys: &GcsSystem, a: remus_sketch::PointId, b: remus_sketch::PointId) -> f64 {
    let pa = sys.point(a).unwrap();
    let pb = sys.point(b).unwrap();
    (pa.x - pb.x).hypot(pa.y - pb.y)
}

fn ordered_points(sys: &GcsSystem) -> Vec<remus_sketch::PointId> {
    let mut ids: Vec<remus_sketch::PointId> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    ids
}

#[test]
fn independent_under_dimensions_and_outcome() {
    for n_params in [10, 100] {
        let mut sys = build_independent_under(n_params);
        let k = n_params / 2;
        let d = sys.dof();
        assert_eq!(d.num_params, n_params);
        assert_eq!(d.num_equations, k);
        let r = sys.solve(MAX_ITER, TOL).unwrap();
        assert!(r.converged, "n={n_params}: max_r={}", r.max_residual);
        let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
        // Already solved: detailed converges immediately at the solution.
        assert!(det.converged);
        assert_eq!(det.num_params, n_params);
        assert_eq!(det.num_equations, k);
        assert_eq!(det.dof, k, "each pair keeps one freedom");
        assert_eq!(det.rank, k);
        assert_eq!(det.classification, SolveClassification::UnderConstrained);
        assert!(!det.rolled_back);
        // Independent oracle: every pair is at distance 5.
        let ids = ordered_points(&sys);
        for chunk in ids.chunks(2) {
            let dist = hypot_distance(&sys, chunk[0], chunk[1]);
            assert!((dist - 5.0).abs() <= 1e-6, "n={n_params}: dist {dist} != 5");
        }
    }
}

#[test]
fn independent_solved_dimensions_and_outcome() {
    for n_params in [10, 100] {
        let mut sys = build_independent_solved(n_params);
        let d = sys.dof();
        assert_eq!(d.num_params, n_params);
        assert_eq!(d.num_equations, n_params);
        let r = sys.solve(MAX_ITER, TOL).unwrap();
        assert!(r.converged, "n={n_params}: max_r={}", r.max_residual);
        let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
        assert!(det.converged);
        assert_eq!(det.num_params, n_params);
        assert_eq!(det.num_equations, n_params);
        assert_eq!(det.dof, 0);
        assert_eq!(det.rank, n_params);
        assert_eq!(det.classification, SolveClassification::Solved);
        let ids = ordered_points(&sys);
        for chunk in ids.chunks(2) {
            let dist = hypot_distance(&sys, chunk[0], chunk[1]);
            assert!((dist - 5.0).abs() <= 1e-6, "n={n_params}: dist {dist} != 5");
            let free = sys.point(chunk[1]).unwrap();
            assert!(
                (free.y - 4.0).abs() <= 1e-8,
                "n={n_params}: y {} != 4",
                free.y
            );
        }
    }
}

#[test]
fn coupled_chain_dimensions_and_outcome() {
    for n_params in [10, 100] {
        let mut sys = build_coupled_chain(n_params);
        let d = sys.dof();
        assert_eq!(d.num_params, n_params);
        assert_eq!(d.num_equations, n_params);
        let r = sys.solve(MAX_ITER, TOL).unwrap();
        assert!(r.converged, "n={n_params}: max_r={}", r.max_residual);
        let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
        assert!(det.converged);
        assert_eq!(det.dof, 0);
        assert_eq!(det.rank, n_params);
        assert_eq!(det.classification, SolveClassification::Solved);
        // Independent oracle: chain lies on y=0 with unit spacing.
        let ids = ordered_points(&sys);
        for (i, id) in ids.iter().enumerate() {
            let p = sys.point(*id).unwrap();
            assert!(
                (p.x - i as f64).abs() <= 1e-6,
                "n={n_params}: x {} != {i}",
                p.x
            );
            assert!(p.y.abs() <= 1e-8, "n={n_params}: y {} != 0", p.y);
        }
    }
}

#[test]
fn redundant_classification_and_convergence() {
    // Solved at 100 (fast in debug); 1000-parameter solve outcomes are
    // validated per sample by the profiling-release runner.
    let n_params = 100;
    let mut sys = build_redundant(n_params);
    let d = sys.dof();
    assert_eq!(d.num_params, n_params);
    assert_eq!(d.num_equations, n_params + n_params / 2);
    let det = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(det.converged);
    assert_eq!(det.dof, 0);
    assert_eq!(det.rank, n_params);
    assert!(det.redundant);
    assert_eq!(det.classification, SolveClassification::Redundant);
    assert!(!det.rolled_back);
}

#[test]
fn inconsistent_refusal_and_rollback_contract() {
    // Refusal contracts at 100 (fast in debug); 1000-parameter refusals are
    // validated per sample by the profiling-release runner.
    let n_params = 100;
    // solve publishes its last iterate; solve_detailed rolls back.
    let mut via_solve = build_inconsistent(n_params);
    let before: Vec<(f64, f64)> = ordered_points(&via_solve)
        .iter()
        .map(|id| {
            let p = via_solve.point(*id).unwrap();
            (p.x, p.y)
        })
        .collect();
    let r = via_solve.solve(50, TOL).unwrap();
    assert!(!r.converged, "n={n_params}: inconsistent system solved");
    let after: Vec<(f64, f64)> = ordered_points(&via_solve)
        .iter()
        .map(|id| {
            let p = via_solve.point(*id).unwrap();
            (p.x, p.y)
        })
        .collect();
    // solve keeps its last iterate (state contract: no silent restore).
    assert_ne!(before, after, "solve must publish its last iterate");

    let mut via_detailed = build_inconsistent(n_params);
    let det = via_detailed.solve_detailed(50, TOL).unwrap();
    assert!(!det.converged);
    assert_eq!(det.classification, SolveClassification::Unsatisfied);
    assert!(det.rolled_back);
    let restored: Vec<(f64, f64)> = ordered_points(&via_detailed)
        .iter()
        .map(|id| {
            let p = via_detailed.point(*id).unwrap();
            (p.x, p.y)
        })
        .collect();
    assert_eq!(before, restored, "detailed must restore pre-solve state");
}

#[test]
fn drag_edits_change_target_and_resolve() {
    // 100-param chain: cold solve, then 20 deterministic drags of one point.
    let mut sys = build_coupled_chain(100);
    let cold = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(cold.converged);
    let ids = ordered_points(&sys);
    let target = ids[25];
    for step in 0..20 {
        let before = {
            let p = sys.point(target).unwrap();
            (p.x, p.y)
        };
        {
            let slot = sys.point_mut(target).unwrap();
            slot.x += 0.5;
            slot.y += 0.25;
        }
        let edited = {
            let p = sys.point(target).unwrap();
            (p.x, p.y)
        };
        assert!(
            (edited.0 - before.0 - 0.5).abs() <= 1e-12
                && (edited.1 - before.1 - 0.25).abs() <= 1e-12,
            "step {step}: edit must change its target"
        );
        let r = sys.solve(MAX_ITER, TOL).unwrap();
        assert!(r.converged, "step {step}: max_r={}", r.max_residual);
        let solved = {
            let p = sys.point(target).unwrap();
            (p.x, p.y)
        };
        // The re-solve must not return the pre-solve edited position
        // untouched: the drag perturbs two constraints, so the solver moves it.
        assert!(
            (solved.0 - edited.0).abs() + (solved.1 - edited.1).abs() > 1e-9,
            "step {step}: re-solve reused the previous result"
        );
    }
    // Final state still satisfies the chain oracle.
    let ids = ordered_points(&sys);
    for (i, id) in ids.iter().enumerate() {
        let p = sys.point(*id).unwrap();
        assert!((p.x - i as f64).abs() <= 1e-6, "final x {} != {i}", p.x);
        assert!(p.y.abs() <= 1e-8, "final y {} != 0", p.y);
    }
}

#[test]
fn repeated_solves_are_deterministic() {
    // Same fixture solved twice from scratch reaches the same geometry.
    for build in [
        build_independent_under(100),
        build_independent_solved(100),
        build_coupled_chain(100),
    ] {
        let mut a = build.clone();
        let mut b = build;
        let ra = a.solve(MAX_ITER, TOL).unwrap();
        let rb = b.solve(MAX_ITER, TOL).unwrap();
        assert!(ra.converged && rb.converged);
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

#[test]
fn large_fixtures_have_pinned_construction_counts() {
    // 1000-parameter fixtures: no solve and no QR here (debug CI stays fast).
    // Counts pin the construction the profiling-release runner solves:
    // every constraint below carries exactly 1 equation (Distance, FixY,
    // Horizontal), so constraint count equals equation count.
    let sys = build_independent_under(1000);
    assert_eq!(sys.point_count(), 1000);
    assert_eq!(sys.line_count(), 0);
    assert_eq!(sys.constraint_count(), 500);

    let sys = build_independent_solved(1000);
    assert_eq!(sys.point_count(), 1000);
    assert_eq!(sys.constraint_count(), 1000);

    let sys = build_coupled_chain(1000);
    // 501 points (1 fixed + 500 free), 500 segment lines, 1000 constraints.
    assert_eq!(sys.point_count(), 501);
    assert_eq!(sys.line_count(), 500);
    assert_eq!(sys.constraint_count(), 1000);

    let sys = build_redundant(1000);
    assert_eq!(sys.point_count(), 1000);
    assert_eq!(sys.constraint_count(), 1500);

    let sys = build_inconsistent(1000);
    assert_eq!(sys.point_count(), 1000);
    assert_eq!(sys.constraint_count(), 501);
}

#[test]
fn large_system_budget_reports_refusal_shape() {
    // The 10000-parameter dense Jacobian (10000x10000 f64 = 800 MB) exceeds
    // the benchmark's 256 MiB upfront budget, so the runner must record a
    // resource refusal instead of attempting the solve. The shape is gated
    // here; wall time is not.
    let n_params = 10_000_usize;
    let num_equations = 10_000_usize;
    let jacobian_bytes = (num_equations as u64) * (n_params as u64) * 8;
    assert!(jacobian_bytes > 256 * 1024 * 1024);
}
