//! Tests for the bounded sparse slice (PERF-S04).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use crate::gcs::constraint::{Constraint, JacobianWriter};
use crate::gcs::constraint::{eval_jacobian, residual_count};
use crate::{GcsSystem, PointData};
use std::collections::HashMap;

#[test]
fn dispatch_thresholds_pin_small_and_wide() {
    assert!(!should_use_sparse(100, 100, 4));
    assert!(!should_use_sparse(10, 10, 0));
    assert!(should_use_sparse(128, 128, 4));
    assert!(should_use_sparse(1000, 1000, 44));
    assert!(!should_use_sparse(1000, 1000, 65));
    assert!(!should_use_sparse(500, 1000, 4));
}

#[test]
fn s04_bands_pin_expected_dispatch() {
    // Chain/grid/linkage (point-only) stay banded; tangent/ellipse (scalar
    // mixing) exceed the bound and fall back — pinned to prevent silent
    // dispatch drift.
    for (name, mut sys) in [
        ("chain300", build_chain(300)),
        ("grid300", build_s04_grid(300)),
        ("linkage300", build_s04_linkage(300)),
        ("tangent300", build_s04_tangent(300)),
        ("ellipse300", build_s04_ellipse_chain(300)),
    ] {
        sys.rebuild_if_dirty();
        let param_index = sys.param_index_map().clone();
        let entries = sys.ordered_constraints();
        let cs: Vec<Constraint> = entries.iter().map(|(_, c)| c.clone()).collect();
        let rows: Vec<usize> = entries.iter().map(|(_, c)| residual_count(c)).collect();
        let m: usize = rows.iter().sum();
        let n = sys.param_map_slice().len();
        let pattern = build_pattern(&cs, &rows, &sys, &param_index, m, n);
        let uses = should_use_sparse(m, n, pattern.band);
        match name {
            "chain300" | "grid300" | "linkage300" => assert!(uses, "{name} must dispatch sparse"),
            _ => assert!(!uses, "{name} must fall back (scalar mixing)"),
        }
    }
}

fn build_chain(n_params: usize) -> GcsSystem {
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

fn build_s04_grid(n_params: usize) -> GcsSystem {
    let n_pts_needed = n_params / 2 + 1;
    let w = ((n_pts_needed as f64).sqrt().ceil() as usize).max(2);
    let mut sys = GcsSystem::new();
    let mut grid: Vec<Vec<_>> = Vec::new();
    let mut count = 0_usize;
    for r in 0.. {
        if count >= n_pts_needed {
            break;
        }
        let mut row = Vec::new();
        for c in 0..w {
            if count >= n_pts_needed {
                break;
            }
            let fixed = r == 0 && c == 0;
            let x = c as f64 + 0.1 * f64::from((c % 2) as u8);
            let y = r as f64 + 0.1 * f64::from((r % 2) as u8);
            row.push(sys.add_point(PointData { x, y, fixed }).unwrap());
            count += 1;
        }
        grid.push(row);
    }
    for (r, row) in grid.iter().enumerate() {
        for (c, &p) in row.iter().enumerate() {
            if c + 1 < row.len() {
                let q = row[c + 1];
                let line = sys.add_line(p, q).unwrap();
                sys.add_constraint(Constraint::Distance(p, q, 1.0)).unwrap();
                sys.add_constraint(Constraint::Horizontal(line)).unwrap();
            }
            if r + 1 < grid.len() && c < grid[r + 1].len() {
                let q = grid[r + 1][c];
                let line = sys.add_line(p, q).unwrap();
                sys.add_constraint(Constraint::Distance(p, q, 1.0)).unwrap();
                sys.add_constraint(Constraint::Vertical(line)).unwrap();
            }
        }
    }
    sys
}

fn build_s04_linkage(n_params: usize) -> GcsSystem {
    // Interleaved bracing (not a trailing block) so rows stay near their
    // columns: after each new segment, brace the previous triple.
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
        let w = &pts[(i - 1)..=i];
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
        if i >= 2 {
            sys.add_constraint(Constraint::Distance(pts[i - 2], pts[i], 2.0))
                .unwrap();
        }
    }
    sys
}

fn build_s04_tangent(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(3) && n_params >= 30);
    let segs = n_params / 3;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(segs + 1);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap(),
    );
    for i in 1..=segs {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.2 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .unwrap(),
        );
    }
    for (k, w) in pts.windows(2).enumerate() {
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        let center = sys
            .add_point(PointData {
                x: k as f64,
                y: 1.0,
                fixed: true,
            })
            .unwrap();
        let circ = sys.add_circle(center, 1.2).unwrap();
        sys.add_constraint(Constraint::TangentLineCircle(line, circ))
            .unwrap();
        sys.add_constraint(Constraint::CircleRadius(circ, 1.0))
            .unwrap();
    }
    sys
}

fn build_s04_ellipse_chain(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(5) && n_params >= 50);
    let segs = n_params / 5;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(segs + 1);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap(),
    );
    for i in 1..=segs {
        pts.push(
            sys.add_point(PointData {
                x: 4.0 * i as f64,
                y: 0.3 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .unwrap(),
        );
    }
    for w in pts.windows(2) {
        let _line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 4.0))
            .unwrap();
        let ell = sys.add_ellipse(w[0], 3.0, 1.5, 0.1).unwrap();
        sys.add_constraint(Constraint::EllipseAxisA(ell, 4.0))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAxisB(ell, 2.0))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAngle(ell, 0.0))
            .unwrap();
        sys.add_constraint(Constraint::PointOnEllipse(w[1], ell))
            .unwrap();
    }
    sys
}

fn perturb_all_free(sys: &mut GcsSystem, dx: f64) {
    let ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    for (k, id) in ids.iter().enumerate() {
        if let Some(p) = sys.point_mut(*id)
            && !p.fixed
        {
            p.x += dx * (k as f64 + 1.0);
            p.y -= 0.5 * dx * (k as f64 + 1.0);
        }
    }
}

fn degenerate_first_pair(sys: &mut GcsSystem) {
    let ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    let free: Vec<_> = ids
        .iter()
        .filter(|id| !sys.point(**id).unwrap().fixed)
        .copied()
        .collect();
    if free.len() >= 2 {
        let (x0, y0) = {
            let p0 = sys.point(free[0]).unwrap();
            (p0.x, p0.y)
        };
        if let Some(p1) = sys.point_mut(free[1]) {
            p1.x = x0;
            p1.y = y0;
        }
    }
}

fn dense_and_pattern(sys: &mut GcsSystem) -> (Vec<f64>, SparsePattern) {
    use crate::gcs::constraint::EntitySnapshot;
    sys.rebuild_if_dirty();
    let param_index = sys.param_index_map().clone();
    let entries = sys.ordered_constraints();
    let cs: Vec<Constraint> = entries.iter().map(|(_, c)| c.clone()).collect();
    let rows: Vec<usize> = entries.iter().map(|(_, c)| residual_count(c)).collect();
    let m: usize = rows.iter().sum();
    let n = sys.param_map_slice().len();
    let pattern = build_pattern(&cs, &rows, sys, &param_index, m, n);
    let mut snap = EntitySnapshot {
        points: HashMap::new(),
        lines: HashMap::new(),
        circles: HashMap::new(),
        arcs: HashMap::new(),
        ellipses: HashMap::new(),
    };
    for (id, p) in sys.points() {
        snap.points.insert(id, (p.x, p.y));
    }
    for (id, l) in sys.lines() {
        snap.lines.insert(id, (l.p1, l.p2));
    }
    for (id, c) in sys.circles() {
        snap.circles.insert(id, (c.center, c.radius));
    }
    for (id, a) in sys.arcs() {
        snap.arcs.insert(id, (a.center, a.start, a.end));
    }
    for (id, e) in sys.ellipses() {
        snap.ellipses.insert(id, (e.center, e.a, e.b, e.angle));
    }
    let mut dense = vec![0.0_f64; m * n];
    let mut row = 0_usize;
    for (_, c) in &entries {
        let mut jw = JacobianWriter {
            data: &mut dense,
            ncols: n,
            param_index: &param_index,
        };
        eval_jacobian(c, &snap, &mut jw, row);
        row += residual_count(c);
    }
    (dense, pattern)
}

#[test]
fn dense_entries_outside_pattern_are_zero() {
    for n in [10_usize, 30] {
        let mut sys = build_chain(n);
        for state in 0..3 {
            if state == 1 {
                perturb_all_free(&mut sys, 0.13);
            } else if state == 2 {
                degenerate_first_pair(&mut sys);
            }
            let (dense, pattern) = dense_and_pattern(&mut sys);
            let (m, nn) = (pattern.m, pattern.n);
            for i in 0..m {
                let start = pattern.row_ptr[i];
                let end = pattern.row_ptr[i + 1];
                let cols = &pattern.col_idx[start..end];
                for j in 0..nn {
                    if !cols.contains(&j) {
                        assert_eq!(
                            dense[i * nn + j].to_bits(),
                            0.0_f64.to_bits(),
                            "n={n} state={state} row={i} col={j} must stay structural-zero"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn sparse_matvecs_match_dense_skipping_zeros() {
    let pattern = SparsePattern {
        m: 2,
        n: 3,
        row_ptr: vec![0, 2, 3],
        col_idx: vec![0, 2, 1],
        band: 2,
        nnz: 3,
    };
    let values = vec![1.0, 2.0, 3.0];
    let x = vec![4.0, 5.0, 6.0];
    let mut y = vec![0.0; 2];
    sparse_matvec(&pattern, &values, &x, &mut y);
    assert_eq!(y, vec![1.0 * 4.0 + 2.0 * 6.0, 3.0 * 5.0]);
    let mut yt = vec![0.0; 3];
    sparse_matvec_transpose(&pattern, &values, &y, &mut yt);
    assert_eq!(yt, vec![y[0], 3.0 * y[1], 2.0 * y[0]]);
    let xn = vec![4.0, f64::NAN, 6.0];
    sparse_matvec(&pattern, &values, &xn, &mut y);
    assert!(y.iter().all(|v| v.is_nan()));
    sparse_matvec_transpose(&pattern, &values, &[1.0, f64::INFINITY], &mut yt);
    assert!(yt.iter().all(|v| v.is_nan()));
}

#[test]
fn givens_matches_dense_on_banded_full_rank() {
    let pattern = SparsePattern {
        m: 4,
        n: 4,
        row_ptr: vec![0, 2, 4, 6, 8],
        col_idx: vec![0, 1, 0, 1, 1, 2, 2, 3],
        band: 1,
        nnz: 8,
    };
    let values = vec![2.0, -1.0, -1.0, 2.0, -1.0, 2.0, -1.0, 2.0];
    let b = vec![1.0, 2.0, 3.0, 4.0];
    let h = sparse_givens_solve(&pattern, &values, &b).expect("full rank solves");
    let mut jh = vec![0.0; 4];
    sparse_matvec(&pattern, &values, &h, &mut jh);
    for (a, bb) in jh.iter().zip(b.iter()) {
        assert!((a - bb).abs() < 1e-9, "jh={jh:?} b={b:?}");
    }
}

#[test]
fn givens_falls_back_on_rank_deficient() {
    let pattern = SparsePattern {
        m: 2,
        n: 2,
        row_ptr: vec![0, 2, 4],
        col_idx: vec![0, 1, 0, 1],
        band: 1,
        nnz: 4,
    };
    let values = vec![1.0, 2.0, 2.0, 4.0];
    assert!(sparse_givens_solve(&pattern, &values, &[1.0, 2.0]).is_none());
    assert!(sparse_givens_solve(&pattern, &[1.0, f64::NAN, 2.0, 4.0], &[1.0, 2.0]).is_none());
}

#[test]
fn chain_200_dense_and_sparse_agree_on_contracted_outcomes() {
    use crate::gcs::diagnostics::classify as classify_solve;
    use crate::gcs::dof;
    // Forced dense reference (bypasses sparse dispatch).
    let mut dense_sys = build_chain(200);
    dense_sys.rebuild_if_dirty();
    let before_dense = dense_sys.extract_params();
    let param_index_d = dense_sys.param_index_map().clone();
    let entries_d = dense_sys.ordered_constraints();
    let cs_d: Vec<Constraint> = entries_d.iter().map(|(_, c)| c.clone()).collect();
    let m_d: usize = entries_d.iter().map(|(_, c)| residual_count(c)).sum();
    let n_d = dense_sys.param_map_slice().len();
    let (dense_res, _, _) = dense_sys
        .solve_dense_direct(&cs_d, &param_index_d, m_d, n_d, 100, 1e-10, false)
        .unwrap();
    // Rank via dense (same as production analysis for this size).
    let snap_d = {
        use crate::gcs::constraint::EntitySnapshot;
        let mut snap = EntitySnapshot {
            points: HashMap::new(),
            lines: HashMap::new(),
            circles: HashMap::new(),
            arcs: HashMap::new(),
            ellipses: HashMap::new(),
        };
        for (id, p) in dense_sys.points() {
            snap.points.insert(id, (p.x, p.y));
        }
        for (id, l) in dense_sys.lines() {
            snap.lines.insert(id, (l.p1, l.p2));
        }
        snap
    };
    let mut jac_d = vec![0.0_f64; m_d * n_d];
    {
        let mut row = 0_usize;
        let mut jw = JacobianWriter {
            data: &mut jac_d,
            ncols: n_d,
            param_index: &param_index_d,
        };
        for (_, c) in &entries_d {
            eval_jacobian(c, &snap_d, &mut jw, row);
            row += residual_count(c);
        }
    }
    let dense_analysis = dof::analyze(&jac_d, m_d, n_d);
    let dense_class = classify_solve(
        dense_res.converged,
        dense_analysis.dof,
        dense_analysis.rank,
        dense_analysis.num_equations,
    );

    // Public path (sparse-dispatched for n=200).
    let mut sparse_sys = build_chain(200);
    let sparse_res = sparse_sys.solve(100, 1e-10).unwrap();
    let mut sparse_det = build_chain(200);
    let sparse_diag = sparse_det.solve_detailed(100, 1e-10).unwrap();

    assert_eq!(sparse_res.converged, dense_res.converged);
    assert_eq!(sparse_diag.converged, dense_res.converged);
    assert_eq!(sparse_diag.rank, dense_analysis.rank);
    assert_eq!(sparse_diag.dof, dense_analysis.dof);
    assert_eq!(sparse_diag.classification, dense_class);
    assert!(sparse_diag.max_residual < 1e-10);
    assert!(dense_res.max_residual < 1e-10);
    // Iterations must match (same trust region, only roundoff differs).
    // Allow at most one iteration of drift with an explicit explanation;
    // currently they match exactly for the chain.
    assert!(
        sparse_res.iterations == dense_res.iterations
            || sparse_res.iterations + 1 == dense_res.iterations
            || dense_res.iterations + 1 == sparse_res.iterations,
        "dense iters={} sparse iters={}",
        dense_res.iterations,
        sparse_res.iterations
    );
    let _ = before_dense;
}
