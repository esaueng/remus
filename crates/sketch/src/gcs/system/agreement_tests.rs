//! Dense-vs-component agreement for the PERF-S02 component path.
//!
//! Each test solves one clone through the legacy dense loop
//! ([`GcsSystem::solve_dense`] plus dense DOF/residual measurement assembled
//! by [`dense_detailed`] below) and another through the public entry points,
//! which take the component path whenever the system genuinely splits. The
//! dense side is the pre-S02 behavior bit-for-bit; the component side must
//! agree on every contracted outcome.
//!
//! Agreement is exact on classifications, DOF, rank, dimensions and
//! convergence, and tolerance-based on geometry (analytic oracles, never one
//! solver parroting the other). It is deliberately NOT bitwise on solutions or
//! iteration counts for multi-component systems: per-component trust regions
//! and step decisions can legitimately traverse different iterates toward the
//! same contracted outcome (see `components.rs`). Single-component systems
//! stay on the dense loop and assert bitwise equality.

use super::super::constraint::Constraint;
use super::super::diagnostics::{SolveDiagnostics, classify as classify_solve};
use super::super::dof;
use super::super::entity::PointData;
use super::super::entity::PointId;
use super::{GcsSystem, fold_max_residual};

const TOL: f64 = 1e-10;
const MAX_ITER: usize = 100;

fn free_pt(sys: &mut GcsSystem, x: f64, y: f64) -> PointId {
    sys.add_point(PointData { x, y, fixed: false }).unwrap()
}

fn fixed_pt(sys: &mut GcsSystem, x: f64, y: f64) -> PointId {
    sys.add_point(PointData { x, y, fixed: true }).unwrap()
}

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
        sys.dof()
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
