//! Native sketch-performance worker for `scripts/performance/sketch/run.py`.
//!
//! Stdout is a JSONL protocol with schema `remus-sketch-perf-sample-v1`.
//! Construction, correctness checks, and teardown stay outside the timed
//! region; only `solve` / `solve_detailed` (or the 20 warm drag re-solves)
//! are timed. Dense Jacobians above the manifest byte budget are refused
//! before timing and reported as resource rows, never dropped.
#![allow(clippy::expect_used, clippy::unwrap_used, clippy::print_stdout)]

use std::{hint::black_box, time::Instant};

use remus_sketch::{Constraint, GcsSystem, PointData};
use serde_json::{Value, json};

const TOL: f64 = 1e-10;
const MAX_ITER: usize = 100;
const INCONSISTENT_MAX_ITER: usize = 50;
const JACOBIAN_BUDGET_BYTES: u64 = 256 * 1024 * 1024;
const DRAG_STEPS: usize = 20;

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
            .expect("finite");
        let free = sys
            .add_point(PointData {
                x: ax + 1.0,
                y: 1.0,
                fixed: false,
            })
            .expect("finite");
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .expect("valid");
    }
    sys
}

fn build_independent_solved(n_params: usize) -> GcsSystem {
    let mut sys = GcsSystem::new();
    for i in 0..n_params / 2 {
        let ax = 10.0 * i as f64;
        let anchor = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .expect("finite");
        let free = sys
            .add_point(PointData {
                x: ax + 1.0,
                y: 1.0,
                fixed: false,
            })
            .expect("finite");
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .expect("valid");
        sys.add_constraint(Constraint::FixY(free, 4.0))
            .expect("valid");
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
        .expect("finite"),
    );
    for i in 1..n_pts {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .expect("finite"),
        );
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).expect("valid");
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .expect("valid");
        sys.add_constraint(Constraint::Horizontal(line))
            .expect("valid");
    }
    sys
}

fn build_redundant(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_solved(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for chunk in ids.chunks(2) {
        sys.add_constraint(Constraint::Distance(chunk[0], chunk[1], 5.0))
            .expect("valid");
    }
    sys
}

fn build_inconsistent(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_under(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    sys.add_constraint(Constraint::Distance(ids[0], ids[1], 6.0))
        .expect("valid");
    sys
}

fn ordered_points(sys: &GcsSystem) -> Vec<remus_sketch::PointId> {
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    ids
}

/// Peak resident set size in KiB, Linux only; `None` elsewhere.
fn peak_rss_kib() -> Option<u64> {
    std::fs::read_to_string("/proc/self/status")
        .ok()?
        .lines()
        .find(|l| l.starts_with("VmHWM"))?
        .split_whitespace()
        .nth(1)?
        .parse()
        .ok()
}

fn emit(
    workload: &str,
    size: usize,
    mode: &str,
    sample: usize,
    warmup: bool,
    operation_ns: Option<u128>,
    record: Value,
) {
    let row = json!({
        "schema": "remus-sketch-perf-sample-v1",
        "workload": workload,
        "size": size,
        "mode": mode,
        "sample": sample,
        "warmup": warmup,
        "validation": "passed",
        "operation_ns": operation_ns,
    })
    .as_object()
    .expect("object")
    .clone()
    .into_iter()
    .chain(record.as_object().expect("object").clone())
    .collect::<serde_json::Map<_, _>>();
    println!("{}", Value::Object(row));
}

/// Independent geometric oracle outside the timer (never the residual alone).
fn check_geometry(sys: &GcsSystem, workload: &str, size: usize) -> Value {
    let ids = ordered_points(sys);
    match workload {
        "independent_under" | "independent_solved" | "redundant" => {
            let mut worst = 0.0_f64;
            for chunk in ids.chunks(2) {
                let a = sys.point(chunk[0]).expect("point");
                let b = sys.point(chunk[1]).expect("point");
                let d = (a.x - b.x).hypot(a.y - b.y);
                worst = worst.max((d - 5.0).abs());
                assert!((d - 5.0).abs() <= 1e-6, "{workload} {size}: dist {d}");
            }
            if workload == "independent_solved" {
                for chunk in ids.chunks(2) {
                    let b = sys.point(chunk[1]).expect("point");
                    assert!((b.y - 4.0).abs() <= 1e-8, "fixy {}", b.y);
                }
            }
            json!({"oracle": "hypot_distance_5", "worst_abs_err": worst})
        }
        "coupled_chain" | "drag" => {
            let mut worst_x = 0.0_f64;
            let mut worst_y = 0.0_f64;
            for (i, id) in ids.iter().enumerate() {
                let p = sys.point(*id).expect("point");
                worst_x = worst_x.max((p.x - i as f64).abs());
                worst_y = worst_y.max(p.y.abs());
                assert!((p.x - i as f64).abs() <= 1e-6, "chain x {}", p.x);
                assert!(p.y.abs() <= 1e-8, "chain y {}", p.y);
            }
            json!({"oracle": "chain_grid", "worst_x_err": worst_x, "worst_y_err": worst_y})
        }
        _ => json!({"oracle": "none_contradictory"}),
    }
}

#[allow(clippy::too_many_lines)]
fn run_once(
    build: fn(usize) -> GcsSystem,
    workload: &str,
    size: usize,
    mode: &str,
    sample: usize,
    warmup: bool,
) {
    let max_iter = if workload == "inconsistent" {
        INCONSISTENT_MAX_ITER
    } else {
        MAX_ITER
    };
    // Construction is setup and stays outside the timer.
    let mut sys = build(size);
    // Upfront dense-Jacobian budget: refuse before allocating the m*n buffer.
    // Equation counts are exact here (every constraint is 1 equation).
    let num_equations = sys.constraint_count();
    let mut n_free = 0_usize;
    for (_, p) in sys.points() {
        if !p.fixed {
            n_free += 1;
        }
    }
    let num_params = 2 * n_free;
    let jacobian_bytes = (num_equations as u64) * (num_params as u64) * 8;
    if jacobian_bytes > JACOBIAN_BUDGET_BYTES {
        emit(
            workload,
            size,
            mode,
            sample,
            warmup,
            None,
            json!({
                "resource": "refused",
                "resource_reason": "jacobian_bytes_over_budget",
                "jacobian_bytes": jacobian_bytes,
                "budget_bytes": JACOBIAN_BUDGET_BYTES,
                "num_params": num_params,
                "num_equations": num_equations,
                "classification": "resource_refused",
                "metrics": {"budget_checked": true, "solve_attempted": false},
            }),
        );
        return;
    }

    if workload == "drag" {
        // Cold solve is setup; the 20 warm drag re-solves are timed.
        let cold = sys.solve(MAX_ITER, TOL).expect("cold solve");
        assert!(cold.converged, "drag cold: {}", cold.max_residual);
        let ids = ordered_points(&sys);
        let target = ids[25];
        let start = Instant::now();
        let mut iters = 0_usize;
        if mode == "solve" {
            for _ in 0..DRAG_STEPS {
                {
                    let slot = sys.point_mut(target).expect("point");
                    slot.x += 0.5;
                    slot.y += 0.25;
                }
                let before = sys.point(target).expect("point");
                assert!(before.x.is_finite() && before.y.is_finite());
                let r = black_box(sys.solve(MAX_ITER, TOL)).expect("re-solve");
                assert!(r.converged, "drag step: {}", r.max_residual);
                iters += r.iterations;
            }
        } else {
            for _ in 0..DRAG_STEPS {
                {
                    let slot = sys.point_mut(target).expect("point");
                    slot.x += 0.5;
                    slot.y += 0.25;
                }
                let r = black_box(sys.solve_detailed(MAX_ITER, TOL)).expect("re-solve");
                assert!(r.converged, "drag step: {}", r.max_residual);
                assert!(!r.rolled_back);
                iters += r.iterations;
            }
        }
        let ns = start.elapsed().as_nanos();
        let oracle = check_geometry(&sys, "drag", size);
        let d = sys.dof();
        emit(
            workload,
            size,
            mode,
            sample,
            warmup,
            Some(ns),
            json!({
                "resource": "solved",
                "iterations": iters,
                "iterations_per_step": iters as f64 / DRAG_STEPS as f64,
                "drag_steps": DRAG_STEPS,
                "num_params": d.num_params,
                "num_equations": d.num_equations,
                "rank": d.rank,
                "dof": d.dof,
                "classification": "solved",
                "jacobian_bytes": (d.num_equations as u64) * (d.num_params as u64) * 8,
                "residual_bytes": (d.num_equations as u64) * 8,
                "param_bytes": (d.num_params as u64) * 8,
                "metrics": oracle,
                "peak_rss_kib": peak_rss_kib(),
            }),
        );
        return;
    }

    if mode == "solve" {
        let start = Instant::now();
        let r = black_box(sys.solve(max_iter, TOL)).expect("solve");
        let ns = start.elapsed().as_nanos();
        // Diagnostics for the report are measured outside the timer.
        let d = sys.dof();
        let classification = if r.converged {
            if d.dof > 0 {
                "underConstrained"
            } else if d.rank < d.num_equations {
                "redundant"
            } else {
                "solved"
            }
        } else {
            "unsatisfied"
        };
        let oracle = if r.converged {
            check_geometry(&sys, workload, size)
        } else {
            assert_eq!(workload, "inconsistent");
            json!({"oracle": "refusal_expected", "converged": false})
        };
        emit(
            workload,
            size,
            mode,
            sample,
            warmup,
            Some(ns),
            json!({
                "resource": "solved",
                "converged": r.converged,
                "iterations": r.iterations,
                "max_residual": r.max_residual,
                "num_params": d.num_params,
                "num_equations": d.num_equations,
                "rank": d.rank,
                "dof": d.dof,
                "classification": classification,
                "jacobian_bytes": (d.num_equations as u64) * (d.num_params as u64) * 8,
                "residual_bytes": (d.num_equations as u64) * 8,
                "param_bytes": (d.num_params as u64) * 8,
                "metrics": oracle,
                "peak_rss_kib": peak_rss_kib(),
            }),
        );
    } else {
        let start = Instant::now();
        let det = black_box(sys.solve_detailed(max_iter, TOL)).expect("detailed");
        let ns = start.elapsed().as_nanos();
        let oracle = if det.converged {
            check_geometry(&sys, workload, size)
        } else {
            assert_eq!(workload, "inconsistent");
            assert!(det.rolled_back);
            json!({"oracle": "refusal_expected", "converged": false, "rolled_back": true})
        };
        emit(
            workload,
            size,
            mode,
            sample,
            warmup,
            Some(ns),
            json!({
                "resource": "solved",
                "converged": det.converged,
                "iterations": det.iterations,
                "max_residual": det.max_residual,
                "published_max_residual": det.published_max_residual,
                "num_params": det.num_params,
                "num_equations": det.num_equations,
                "rank": det.rank,
                "dof": det.dof,
                "redundant": det.redundant,
                "rolled_back": det.rolled_back,
                "classification": det.classification.as_str(),
                "jacobian_bytes": (det.num_equations as u64) * (det.num_params as u64) * 8,
                "residual_bytes": (det.num_equations as u64) * 8,
                "param_bytes": (det.num_params as u64) * 8,
                "metrics": oracle,
                "peak_rss_kib": peak_rss_kib(),
            }),
        );
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 6 {
        return Err(
            "usage: sketch_baseline <workload> <size> <solve|detailed> <samples> <warmup>".into(),
        );
    }
    let workload = args[1].clone();
    let size: usize = args[2].parse()?;
    let mode = args[3].clone();
    let samples: usize = args[4].parse()?;
    let warmup: usize = args[5].parse()?;
    assert!(mode == "solve" || mode == "detailed");
    let build: fn(usize) -> GcsSystem = match workload.as_str() {
        "independent_under" => build_independent_under,
        "independent_solved" => build_independent_solved,
        "coupled_chain" | "drag" => build_coupled_chain,
        "redundant" => build_redundant,
        "inconsistent" => build_inconsistent,
        _ => return Err(format!("unknown workload: {workload}").into()),
    };
    for s in 0..samples + warmup {
        run_once(build, &workload, size, &mode, s, s < warmup);
    }
    Ok(())
}
