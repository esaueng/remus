#![allow(clippy::unwrap_used, clippy::expect_used)]
//! PERF-S06 qualification: solve-local final-iterate evaluation sharing.
//!
//! Every test compares the shared production path
//! (`GcsSystem::solve_detailed`, via `solve_detailed_counted`) against an
//! independent fresh-evaluation oracle (`solve_detailed_fresh` below) that
//! replicates the pre-S06 logic — `solve`, then a fresh `dof`, then fresh
//! per-constraint residuals, then a fresh restored-state measurement after a
//! rollback — without ever consulting a [`FinalEvaluation`]. A stale-cache
//! bug in the implementation cannot make both agree incorrectly, because the
//! oracle re-evaluates everything from live state.
//!
//! Agreement is bitwise (`to_bits`, NaN-aware): converged, iterations,
//! max_residual, published_max_residual, dof, rank, num_params,
//! num_equations, every per-constraint residual (id, magnitude, internal
//! flag), internal_max_residual, rolled_back, redundant, classification.
//! No wall-time assertions: timing evidence lives in the task's retained raw
//! samples, not here.

use super::FinalEvaluation;
use crate::gcs::system::{DetailedCounts, fold_max_residual, max_abs_residual};
use crate::{
    Constraint, ConstraintResidual, GcsSystem, PointData, SolveClassification, SolveDiagnostics,
    classify_solve,
};

const TOL: f64 = 1e-10;
const MAX_ITER: usize = 100;

// ── Builders (mirror the PERF-S01 workloads at small sizes) ──────────────

fn build_independent_under(n_params: usize) -> GcsSystem {
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

fn build_redundant(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_solved(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for chunk in ids.chunks(2) {
        sys.add_constraint(Constraint::Distance(chunk[0], chunk[1], 5.0))
            .unwrap();
    }
    sys
}

fn build_inconsistent(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_under(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    sys.add_constraint(Constraint::Distance(ids[0], ids[1], 6.0))
        .unwrap();
    sys
}

fn ordered_point_ids(sys: &GcsSystem) -> Vec<crate::PointId> {
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    ids
}

// ── Independent fresh-evaluation oracle (pre-S06 logic) ──────────────────

/// Counts for the oracle path, mirroring [`DetailedCounts`] fields.
#[derive(Debug, Default, Clone, Copy)]
struct OracleCounts {
    analysis_jacobian_evals: usize,
    analysis_qr_factorizations: usize,
    final_state_residual_passes: usize,
    restored_state_passes: usize,
}

/// Replicates the pre-S06 `solve_detailed`: solve, then measure everything
/// fresh from live state. Never consults a [`FinalEvaluation`].
fn solve_detailed_fresh(
    sys: &mut GcsSystem,
    max_iterations: usize,
    tolerance: f64,
) -> (SolveDiagnostics, OracleCounts) {
    let mut counts = OracleCounts::default();
    // Same ordering as production: rebuild first so the pre-solve snapshot
    // covers the live parameter map.
    sys.rebuild_if_dirty();
    let before = sys.extract_params();

    let result = sys.solve(max_iterations, tolerance).unwrap();

    let analysis = sys.dof();
    counts.analysis_jacobian_evals += 1;
    counts.analysis_qr_factorizations += 1;
    let (residuals, internal_max_residual) = sys.constraint_residuals();
    counts.final_state_residual_passes += 1;

    let rolled_back = !result.converged;
    let published_max_residual = if rolled_back {
        sys.write_params(&before);
        let (restored, _) = sys.constraint_residuals();
        counts.restored_state_passes += 1;
        fold_max_residual(&restored)
    } else {
        fold_max_residual(&residuals)
    };

    let redundant = analysis.rank < analysis.num_equations;
    (
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
            internal_max_residual,
            rolled_back,
            redundant,
            classification: classify_solve(
                result.converged,
                analysis.dof,
                analysis.rank,
                analysis.num_equations,
            ),
        },
        counts,
    )
}

// ── Bit-exact comparison ─────────────────────────────────────────────────

fn assert_f64_bits_eq(a: f64, b: f64, what: &str) {
    if a.is_nan() || b.is_nan() {
        assert!(
            a.is_nan() && b.is_nan(),
            "{what}: NaN mismatch ({a} vs {b})"
        );
    } else {
        assert_eq!(a.to_bits(), b.to_bits(), "{what}: {a} vs {b}");
    }
}

fn assert_residuals_eq(a: &[ConstraintResidual], b: &[ConstraintResidual]) {
    assert_eq!(a.len(), b.len(), "residual report length");
    for (x, y) in a.iter().zip(b.iter()) {
        assert_eq!(x.constraint, y.constraint, "residual constraint id");
        assert_eq!(x.internal, y.internal, "residual internal flag");
        assert_f64_bits_eq(x.max_abs_residual, y.max_abs_residual, "residual magnitude");
    }
}

fn assert_diagnostics_eq(a: &SolveDiagnostics, b: &SolveDiagnostics) {
    assert_eq!(a.converged, b.converged, "converged");
    assert_eq!(a.iterations, b.iterations, "iterations");
    assert_f64_bits_eq(a.max_residual, b.max_residual, "max_residual");
    assert_f64_bits_eq(
        a.published_max_residual,
        b.published_max_residual,
        "published_max_residual",
    );
    assert_eq!(a.dof, b.dof, "dof");
    assert_eq!(a.rank, b.rank, "rank");
    assert_eq!(a.num_params, b.num_params, "num_params");
    assert_eq!(a.num_equations, b.num_equations, "num_equations");
    assert_residuals_eq(&a.residuals, &b.residuals);
    assert_f64_bits_eq(
        a.internal_max_residual,
        b.internal_max_residual,
        "internal_max_residual",
    );
    assert_eq!(a.rolled_back, b.rolled_back, "rolled_back");
    assert_eq!(a.redundant, b.redundant, "redundant");
    assert_eq!(a.classification, b.classification, "classification");
}

fn params_bits(sys: &GcsSystem) -> Vec<u64> {
    sys.extract_params().iter().map(|v| v.to_bits()).collect()
}

/// Run production (shared) and oracle (fresh) on identical clones and
/// require bitwise agreement on diagnostics and published geometry.
fn assert_shared_matches_fresh(
    build: impl Fn() -> GcsSystem,
    max_iterations: usize,
    tolerance: f64,
) -> (SolveDiagnostics, DetailedCounts, OracleCounts) {
    let mut prod = build();
    let (diag, counts) = prod
        .solve_detailed_counted(max_iterations, tolerance)
        .unwrap();
    let mut oracle_sys = build();
    let (expect, ocounts) = solve_detailed_fresh(&mut oracle_sys, max_iterations, tolerance);
    assert_diagnostics_eq(&diag, &expect);
    // Same starting clone, same deterministic solver: published geometry
    // must agree bitwise too.
    assert_eq!(
        params_bits(&prod),
        params_bits(&oracle_sys),
        "published params"
    );
    (diag, counts, ocounts)
}

fn assert_shared_counts(
    counts: &DetailedCounts,
    rolled_back: bool,
    degenerate: bool,
    expected_analysis_blocks: usize,
) {
    if degenerate {
        assert!(
            !counts.shared_residuals_used,
            "degenerate systems measure fresh"
        );
        assert_eq!(counts.fallback_residual_passes, 1);
        assert_eq!(counts.analysis_jacobian_evals, 0);
        assert_eq!(counts.analysis_qr_factorizations, 0);
    } else {
        assert!(
            counts.shared_residuals_used,
            "non-degenerate systems share the final vector"
        );
        assert_eq!(counts.fallback_residual_passes, 0);
        // PERF-S02: one analysis Jacobian plus one QR per factorizable block,
        // never one giant dense pair. Single-component systems still report 1.
        assert_eq!(
            counts.analysis_jacobian_evals, expected_analysis_blocks,
            "one analysis Jacobian per independent block"
        );
        assert_eq!(
            counts.analysis_qr_factorizations, expected_analysis_blocks,
            "one analysis QR per independent block"
        );
    }
    assert_eq!(
        counts.restored_state_passes,
        usize::from(rolled_back),
        "restored-state pass happens exactly on rollback"
    );
    // The solver loop always evaluates residuals at least once per solve
    // (even the empty-param fast path performs one check), and never
    // factorizes without a Jacobian evaluation.
    assert!(
        counts.solver.residual_evals >= 1,
        "solver evaluates residuals"
    );
    assert!(
        counts.solver.qr_factorizations <= counts.solver.jacobian_evals,
        "one factorization per Jacobian at most"
    );
}

// ── Workload classes ─────────────────────────────────────────────────────

#[test]
fn shared_converged_solved() {
    for n in [4_usize, 20] {
        let (diag, counts, ocounts) =
            assert_shared_matches_fresh(|| build_independent_solved(n), MAX_ITER, TOL);
        assert!(diag.converged);
        assert_eq!(diag.classification, SolveClassification::Solved);
        assert!(!diag.rolled_back);
        assert_shared_counts(&counts, false, false, n / 2);
        // Oracle performs the same solver loop plus one extra final-state
        // residual pass the shared path eliminates.
        assert_eq!(ocounts.final_state_residual_passes, 1);
        assert_eq!(ocounts.restored_state_passes, 0);
    }
}

#[test]
fn shared_underconstrained() {
    let (diag, counts, _) =
        assert_shared_matches_fresh(|| build_independent_under(20), MAX_ITER, TOL);
    assert!(diag.converged);
    assert_eq!(diag.classification, SolveClassification::UnderConstrained);
    assert_eq!(diag.dof, 10);
    assert!(!diag.rolled_back);
    assert_shared_counts(&counts, false, false, 10);
}

#[test]
fn shared_coupled_chain() {
    let (diag, counts, _) = assert_shared_matches_fresh(|| build_coupled_chain(20), MAX_ITER, TOL);
    assert!(diag.converged);
    assert_eq!(diag.classification, SolveClassification::Solved);
    assert_shared_counts(&counts, false, false, 1);
}

#[test]
fn shared_redundant() {
    let (diag, counts, _) = assert_shared_matches_fresh(|| build_redundant(20), MAX_ITER, TOL);
    assert!(diag.converged);
    assert_eq!(diag.classification, SolveClassification::Redundant);
    assert!(diag.redundant);
    assert_shared_counts(&counts, false, false, 10);
}

#[test]
fn shared_inconsistent_rolls_back() {
    let (diag, counts, ocounts) = assert_shared_matches_fresh(|| build_inconsistent(20), 50, TOL);
    assert!(!diag.converged);
    assert_eq!(diag.classification, SolveClassification::Unsatisfied);
    assert!(diag.rolled_back);
    assert_shared_counts(&counts, true, false, 10);
    assert_eq!(ocounts.final_state_residual_passes, 1);
    assert_eq!(ocounts.restored_state_passes, 1);
}

// ── Degenerate dimensions ────────────────────────────────────────────────

#[test]
fn shared_empty_system_no_params() {
    // Every point fixed: zero free params, live constraints.
    let build = || {
        let mut sys = GcsSystem::new();
        let a = sys
            .add_point(PointData {
                x: 0.0,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let b = sys
            .add_point(PointData {
                x: 3.0,
                y: 4.0,
                fixed: true,
            })
            .unwrap();
        sys.add_constraint(Constraint::Distance(a, b, 5.0)).unwrap();
        sys
    };
    let (diag, counts, _) = assert_shared_matches_fresh(build, MAX_ITER, TOL);
    assert!(diag.converged);
    assert_eq!(diag.iterations, 0);
    assert_eq!(diag.num_params, 0);
    assert_shared_counts(&counts, false, true, 0);
}

#[test]
fn shared_empty_system_no_constraints() {
    let build = || {
        let mut sys = GcsSystem::new();
        sys.add_point(PointData {
            x: 1.0,
            y: 2.0,
            fixed: false,
        })
        .unwrap();
        sys
    };
    let (diag, counts, _) = assert_shared_matches_fresh(build, MAX_ITER, TOL);
    assert!(diag.converged);
    assert_eq!(diag.num_equations, 0);
    assert!(diag.residuals.is_empty());
    assert_eq!(diag.dof, 2);
    assert_shared_counts(&counts, false, true, 0);
}

#[test]
fn shared_zero_iterations() {
    // max_iter = 0 still evaluates once at the starting state; the shared
    // path must report exactly that state.
    let (diag, counts, _) = assert_shared_matches_fresh(|| build_independent_solved(10), 0, TOL);
    assert!(!diag.converged);
    assert_eq!(diag.iterations, 0);
    assert!(diag.rolled_back);
    assert_f64_bits_eq(
        diag.max_residual,
        diag.published_max_residual,
        "nothing moved",
    );
    assert_shared_counts(&counts, true, false, 5);
    // Zero iterations still evaluate once per component at the starting state.
    assert_eq!(counts.solver.residual_evals, 5);
    assert_eq!(counts.solver.jacobian_evals, 0);
    assert_eq!(counts.solver.qr_factorizations, 0);
}

#[test]
fn shared_iteration_limited() {
    // One iteration cannot converge the chain: rollback path with a genuine
    // attempted state behind it.
    let (diag, counts, _) = assert_shared_matches_fresh(|| build_coupled_chain(20), 1, TOL);
    assert!(!diag.converged);
    assert_eq!(diag.iterations, 1);
    assert!(diag.rolled_back);
    assert_shared_counts(&counts, true, false, 1);
}

// ── Internal arc constraints ─────────────────────────────────────────────

#[test]
fn shared_arc_internal_attribution() {
    let build = || {
        let mut sys = GcsSystem::new();
        let center = sys
            .add_point(PointData {
                x: 0.0,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let start = sys
            .add_point(PointData {
                x: 2.0,
                y: 0.0,
                fixed: false,
            })
            .unwrap();
        let end = sys
            .add_point(PointData {
                x: 0.0,
                y: 1.0,
                fixed: false,
            })
            .unwrap();
        let arc = sys.add_arc(center, start, end).unwrap();
        sys.add_constraint(Constraint::PointOnArc(start, arc))
            .unwrap();
        sys.add_constraint(Constraint::FixX(start, 2.0)).unwrap();
        sys
    };
    let (diag, counts, _) = assert_shared_matches_fresh(build, MAX_ITER, TOL);
    assert!(diag.converged);
    assert!(diag.residuals.iter().any(|r| r.internal));
    assert!(diag.residuals.iter().any(|r| !r.internal));
    assert_shared_counts(&counts, false, false, 1);
}

// ── Edits between solves: no cross-solve retention ───────────────────────

#[test]
fn shared_edits_between_solves() {
    let mut prod = build_independent_solved(10);
    let mut oracle = build_independent_solved(10);

    for _ in 0..3 {
        let (diag, counts) = prod.solve_detailed_counted(MAX_ITER, TOL).unwrap();
        let (expect, _) = solve_detailed_fresh(&mut oracle, MAX_ITER, TOL);
        assert_diagnostics_eq(&diag, &expect);
        assert_shared_counts(&counts, false, false, 5);

        // Perturb geometry between solves: the next capture must describe
        // the new state, never the previous one.
        for sys in [&mut prod, &mut oracle] {
            let ids = ordered_point_ids(sys);
            let slot = sys.point_mut(ids[1]).unwrap();
            slot.x += 0.5;
            slot.y += 0.25;
        }
    }

    // Add a contradictory constraint on both (ids captured at add time, so
    // both systems carry the same contradiction): the next detailed call
    // rolls back on both.
    let pids = ordered_point_ids(&prod);
    let extra_prod = prod
        .add_constraint(Constraint::Distance(pids[0], pids[1], 6.0))
        .unwrap();
    let qids = ordered_point_ids(&oracle);
    oracle
        .add_constraint(Constraint::Distance(qids[0], qids[1], 6.0))
        .unwrap();
    let (diag, counts) = prod.solve_detailed_counted(50, TOL).unwrap();
    let (expect, _) = solve_detailed_fresh(&mut oracle, 50, TOL);
    assert_diagnostics_eq(&diag, &expect);
    assert!(!diag.converged && diag.rolled_back);
    assert_shared_counts(&counts, true, false, 5);

    // Remove the contradiction: solved and shared again.
    prod.remove_constraint(extra_prod).unwrap();
    let (diag2, counts2) = prod.solve_detailed_counted(MAX_ITER, TOL).unwrap();
    assert!(diag2.converged);
    assert_shared_counts(&counts2, false, false, 5);
}

#[test]
fn shared_constraint_removal_between_solves() {
    // Redundant pair built inline so the duplicate's handle is known on
    // both systems; removing it changes the row layout the identity checks.
    let build = || {
        let mut sys = build_independent_solved(10);
        let mut ids = ordered_point_ids(&sys);
        ids.sort_by_key(|id| id.index());
        let extra = sys
            .add_constraint(Constraint::Distance(ids[0], ids[1], 5.0))
            .unwrap();
        (sys, extra)
    };
    let (mut prod, extra_prod) = build();
    let (mut oracle, extra_oracle) = build();

    let (diag, _) = prod.solve_detailed_counted(MAX_ITER, TOL).unwrap();
    let (expect, _) = solve_detailed_fresh(&mut oracle, MAX_ITER, TOL);
    assert_diagnostics_eq(&diag, &expect);
    assert_eq!(diag.classification, SolveClassification::Redundant);

    prod.remove_constraint(extra_prod).unwrap();
    oracle.remove_constraint(extra_oracle).unwrap();

    let (diag2, counts2) = prod.solve_detailed_counted(MAX_ITER, TOL).unwrap();
    let (expect2, _) = solve_detailed_fresh(&mut oracle, MAX_ITER, TOL);
    assert_diagnostics_eq(&diag2, &expect2);
    assert_eq!(diag2.classification, SolveClassification::Solved);
    assert_shared_counts(&counts2, diag2.rolled_back, false, 5);
}

// ── Extreme coordinate scales ────────────────────────────────────────────

#[test]
fn shared_extreme_scales() {
    for scale in [1e-3, 1.0, 1e5] {
        for offset in [0.0, 1e4] {
            let build = || {
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
                sys
            };
            let (diag, counts, _) = assert_shared_matches_fresh(build, MAX_ITER, TOL);
            assert!(diag.converged, "scale {scale} offset {offset}");
            assert_shared_counts(&counts, false, false, 1);
            // Independent geometric oracle, not residuals alone.
            let mut sys = build();
            sys.solve(MAX_ITER, TOL).unwrap();
            let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
            ids.sort_by_key(|id| id.index());
            let a = sys.point(ids[0]).unwrap();
            let b = sys.point(ids[1]).unwrap();
            let d = (a.x - b.x).hypot(a.y - b.y);
            let tol = 1e-6 * scale.max(1.0);
            assert!((d - 5.0 * scale).abs() <= tol, "scale {scale}: dist {d}");
        }
    }
}

// ── State semantics pins ─────────────────────────────────────────────────

#[test]
fn solve_publishes_its_final_iterate() {
    // On a miss, `solve` keeps the last iterate (no rollback).
    let mut sys = build_inconsistent(10);
    sys.rebuild_if_dirty();
    let before = params_bits(&sys);
    let result = sys.solve(50, TOL).unwrap();
    assert!(!result.converged);
    assert_ne!(
        before,
        params_bits(&sys),
        "solve must publish the attempted state"
    );
    // The reported max describes that published state.
    let (residuals, _) = sys.constraint_residuals();
    assert_f64_bits_eq(
        result.max_residual,
        fold_max_residual(&residuals),
        "solve max_residual",
    );
}

#[test]
fn detailed_restores_starting_geometry_on_miss() {
    let mut sys = build_inconsistent(10);
    sys.rebuild_if_dirty();
    let before = params_bits(&sys);
    let diag = sys.solve_detailed(50, TOL).unwrap();
    assert!(!diag.converged && diag.rolled_back);
    assert_eq!(
        before,
        params_bits(&sys),
        "detailed must restore pre-solve geometry"
    );
}

#[test]
fn attempted_state_residuals_stay_informative() {
    // Satisfiable constraints read ~0 at the attempt even though the system
    // as a whole misses; the restored starting state reads large instead.
    // (Pair 0 carries the contradiction, so the check skips its `5.0` row.)
    let mut sys = build_inconsistent(10);
    let diag = sys.solve_detailed(50, TOL).unwrap();
    assert!(!diag.converged);
    for r in diag.residuals.iter().skip(1).take(4) {
        assert!(
            r.max_abs_residual < 1e-6,
            "attempt must satisfy what it can: {}",
            r.max_abs_residual
        );
    }
    // The report is NOT the restored state: re-measuring now (restored)
    // reads large on those same constraints.
    let (restored, _) = sys.constraint_residuals();
    let restored_large = restored
        .iter()
        .skip(1)
        .take(4)
        .any(|r| r.max_abs_residual > 1e-3);
    assert!(restored_large, "restored state must read large");
    // And the published figure describes the restored state.
    assert_f64_bits_eq(
        diag.published_max_residual,
        fold_max_residual(&restored),
        "published_max_residual",
    );
}

#[test]
fn published_max_matches_attempt_when_converged() {
    let mut sys = build_independent_solved(10);
    let diag = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(diag.converged && !diag.rolled_back);
    assert_f64_bits_eq(
        diag.published_max_residual,
        diag.max_residual,
        "converged published_max",
    );
}

// ── Identity contract unit tests ─────────────────────────────────────────

#[test]
fn identity_matches_exact_state_rejects_edits() {
    let params = vec![1.0, 2.0, 3.0];
    let residuals = vec![0.1, 0.2];
    let sys = build_independent_under(4);
    let rows: Vec<(crate::ConstraintId, usize)> = sys
        .constraint_residuals()
        .0
        .iter()
        .map(|r| (r.constraint, 1_usize))
        .collect();
    assert_eq!(rows.len(), 2);

    let eval = FinalEvaluation::capture(&params, &residuals, rows.clone());
    assert!(eval.matches(&params, &rows, 2, 3));

    // One-bit param edit rejects.
    let mut tampered = params.clone();
    tampered[0] = f64::from_bits(params[0].to_bits() ^ 1);
    assert!(!eval.matches(&tampered, &rows, 2, 3));

    // Row-count change rejects.
    let mut rows_tampered = rows.clone();
    rows_tampered[0].1 = 2;
    assert!(!eval.matches(&params, &rows_tampered, 2, 3));

    // Row reorder over distinct ids rejects.
    let mut rows_swapped = rows.clone();
    rows_swapped.swap(0, 1);
    assert_ne!(rows[0].0, rows[1].0, "test needs distinct ids");
    assert!(!eval.matches(&params, &rows_swapped, 2, 3));

    // Dimension change rejects.
    assert!(!eval.matches(&params, &rows, 3, 3));
    assert!(!eval.matches(&params, &rows, 2, 4));
    assert!(!eval.matches(&params[..2], &rows, 2, 3));

    // Accessors round-trip what capture stored.
    assert_eq!(eval.residuals(), &[0.1, 0.2]);
    assert_eq!(eval.rows(), rows.as_slice());
}

#[test]
fn identity_rejects_constraint_removal() {
    // Capture, remove a constraint, compare: the live layout changed, so the
    // capture must not match — this is the proven invalidation behind
    // "edits between solves never reuse".
    let mut sys = build_independent_under(4);
    let rows_before: Vec<(crate::ConstraintId, usize)> = sys
        .constraint_residuals()
        .0
        .iter()
        .map(|r| (r.constraint, 1_usize))
        .collect();
    let live_params = sys.extract_params();
    let total: usize = rows_before.iter().map(|(_, c)| *c).sum();
    let live = FinalEvaluation::capture(&live_params, &vec![0.0; total], rows_before);
    let victim = sys
        .constraint_residuals()
        .0
        .last()
        .map(|r| r.constraint)
        .unwrap();
    sys.remove_constraint(victim).unwrap();
    let rows_after: Vec<(crate::ConstraintId, usize)> = sys
        .constraint_residuals()
        .0
        .iter()
        .map(|r| (r.constraint, 1_usize))
        .collect();
    assert_eq!(rows_after.len(), 1);
    assert!(!live.matches(&sys.extract_params(), &rows_after, 1, 4));
}

#[test]
fn solver_final_vector_matches_fresh_max() {
    // The capture handed to diagnostics really is the final iterate: the
    // solver's reported max equals a fresh fold at the published state
    // whenever the solve converged (no rollback involved).
    for mut sys in [
        build_independent_solved(10),
        build_coupled_chain(10),
        build_redundant(10),
    ] {
        let diag = sys.solve_detailed(MAX_ITER, TOL).unwrap();
        assert!(diag.converged);
        let (fresh, _) = sys.constraint_residuals();
        assert_f64_bits_eq(
            diag.max_residual,
            fold_max_residual(&fresh),
            "solver final vs fresh",
        );
        // max_abs_residual (flat fold) agrees with the report fold.
        let flat: Vec<f64> = fresh.iter().map(|r| r.max_abs_residual).collect();
        assert_f64_bits_eq(
            max_abs_residual(&flat),
            fold_max_residual(&fresh),
            "fold agreement",
        );
    }
}

// ── Measurement reporting (PERF-S06 evidence, not a gate) ─────────────────
// Asserts shared-vs-fresh agreement (like every test above) and appends one
// line per representative workload with the exact solver-loop counts and the
// shared-vs-fresh analysis counts, for both the production path and the
// fresh oracle (which replicates the pre-S06 work). Printing is denied by
// repo lints, so lines go to the file named by `PERF_S06_REPORT` (when set).
// Run with:
//   PERF_S06_REPORT=/tmp/perf-s06-counts.txt cargo test -p remus-sketch \
//     --lib final_eval::tests::report_counts

#[test]
fn report_counts() {
    use std::fmt::Write as _;

    type CaseBuilder = Box<dyn Fn() -> GcsSystem>;
    let cases: Vec<(&str, CaseBuilder, usize)> = vec![
        (
            "independent_solved_100",
            Box::new(|| build_independent_solved(100)),
            MAX_ITER,
        ),
        (
            "independent_under_100",
            Box::new(|| build_independent_under(100)),
            MAX_ITER,
        ),
        (
            "coupled_chain_100",
            Box::new(|| build_coupled_chain(100)),
            MAX_ITER,
        ),
        ("redundant_100", Box::new(|| build_redundant(100)), MAX_ITER),
        ("inconsistent_100", Box::new(|| build_inconsistent(100)), 50),
    ];
    let mut report = String::new();
    for (name, build, max_iter) in cases {
        let mut prod = build();
        let (diag, counts) = prod.solve_detailed_counted(max_iter, TOL).unwrap();
        let mut oracle_sys = build();
        let (expect, ocounts) = solve_detailed_fresh(&mut oracle_sys, max_iter, TOL);
        assert_diagnostics_eq(&diag, &expect);
        writeln!(
            report,
            "PERF-S06 {name} iters={} conv={} | solver res={} jac={} qr={} | shared={} fallback={} analysis_jac={} analysis_qr={} restored={} | oracle final_passes={} restored={}",
            diag.iterations,
            diag.converged,
            counts.solver.residual_evals,
            counts.solver.jacobian_evals,
            counts.solver.qr_factorizations,
            counts.shared_residuals_used,
            counts.fallback_residual_passes,
            counts.analysis_jacobian_evals,
            counts.analysis_qr_factorizations,
            counts.restored_state_passes,
            ocounts.final_state_residual_passes,
            ocounts.restored_state_passes,
        )
        .expect("report buffer");
    }
    if let Ok(path) = std::env::var("PERF_S06_REPORT") {
        std::fs::write(path, report).expect("PERF_S06_REPORT write");
    }
}
