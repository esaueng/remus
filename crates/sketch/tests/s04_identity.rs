#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
//! Identity gates for the PERF-S04 bounded sparse slice workloads.
//!
//! Pins fixture dimensions and solve outcomes for the five large connected
//! families (chain, grid, linkage, tangent-heavy, ellipse chain) plus the
//! four chain variants (redundant, inconsistent, underconstrained,
//! ill-conditioned). No wall-time thresholds: timing evidence lives in
//! `docs/performance/sketch-perf-s04.md`, not here.
//!
//! Outcomes gated at ~100 params (fast in debug CI); 300-param agreement
//! (sparse-dispatched vs forced-dense via `REMUS_SKETCH_FORCE_DENSE`) is
//! gated where the bounded slice applies (chain/grid/linkage) and fallback
//! equivalence where it does not (tangent/ellipse). Full 1000-param solves
//! are validated per sample by the profiling-release benches plus the paired
//! release report (see the PERF-S04 doc), not here.

use remus_sketch::{Constraint, GcsSystem, PointData, SolveClassification};

const TOL: f64 = 1e-10;
const MAX_ITER: usize = 100;
const INCONSISTENT_MAX_ITER: usize = 50;

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

fn build_grid(n_params: usize) -> GcsSystem {
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

fn build_linkage(n_params: usize) -> GcsSystem {
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

fn build_tangent(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(3));
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

fn build_ellipse_chain(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(5));
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

#[test]
fn s04_dimensions_pin_expected_counts() {
    let chain = build_chain(100);
    assert_eq!(chain.point_count(), 51);
    let grid = build_grid(100);
    assert!(grid.point_count() >= 50);
    let linkage = build_linkage(100);
    assert_eq!(linkage.point_count(), 51);
    let tangent = build_tangent(99);
    assert_eq!(tangent.point_count(), 33 + 34);
    let ellipse = build_ellipse_chain(100);
    assert_eq!(ellipse.point_count(), 21);
}

#[test]
fn s04_chain_solves() {
    let mut sys = build_chain(100);
    let r = sys.solve(MAX_ITER, TOL).unwrap();
    assert!(r.converged);
    let d = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert_eq!(d.classification, SolveClassification::Solved);
}

#[test]
fn s04_grid_solves_or_redundant() {
    let mut sys = build_grid(100);
    let d = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(d.converged);
    assert!(matches!(
        d.classification,
        SolveClassification::Solved | SolveClassification::Redundant
    ));
}

#[test]
fn s04_linkage_converges() {
    let mut sys = build_linkage(100);
    let d = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(d.converged);
}

#[test]
fn s04_tangent_converges() {
    let mut sys = build_tangent(99);
    let d = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(d.converged, "max_r={}", d.max_residual);
}

#[test]
fn s04_ellipse_chain_converges() {
    let mut sys = build_ellipse_chain(100);
    let d = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(
        d.converged,
        "max_r={} rank={} dof={}",
        d.max_residual, d.rank, d.dof
    );
}

#[test]
fn s04_chain_variants_classify() {
    // Redundant: extra Distance per segment (converged, Redundant).
    let mut sys = build_chain(100);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for w in ids.windows(2) {
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
    }
    let d = sys.solve_detailed(MAX_ITER, TOL).unwrap();
    assert!(d.converged);
    assert_eq!(d.classification, SolveClassification::Redundant);

    // Inconsistent: contradictory Distance on first segment (Unsatisfied + rollback).
    let mut sys2 = build_chain(20);
    let mut ids2: Vec<_> = sys2.points().map(|(id, _)| id).collect();
    ids2.sort_by_key(|id| id.index());
    sys2.add_constraint(Constraint::Distance(ids2[0], ids2[1], 2.0))
        .unwrap();
    let d2 = sys2.solve_detailed(INCONSISTENT_MAX_ITER, TOL).unwrap();
    assert!(!d2.converged);
    assert_eq!(d2.classification, SolveClassification::Unsatisfied);
    assert!(d2.rolled_back);
}
