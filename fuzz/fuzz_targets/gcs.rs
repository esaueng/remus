//! Structured fuzzing of the 2D sketch constraint solver (GCS).
//!
//! Builds a small sketch from the fuzzer's bytes — a handful of points on a
//! coarse lattice, lines over them, and a few constraints with small
//! magnitudes — then runs the solver. The oracle is independent of the
//! solver under test: when the solver reports `converged`, every constraint
//! is re-evaluated *geometrically* from the solved point positions with
//! hand-written residual functions in this file (coincidence distance,
//! point distance, horizontal/vertical deviation, fixed-coordinate error).
//! A converged system with any geometric residual above tolerance is a
//! finding: the solver claims success on a solution that does not satisfy
//! the constraints.
//!
//! Only the constraint kinds with a hand-written geometric check are ever
//! generated (coincident, distance, horizontal, vertical, fix-X, fix-Y); the
//! torque-heavy angular kinds are out of scope and never constructed.
//!
//! **A typed refusal is a pass.** Builder or solver `Err` (including
//! non-convergence) stops the case silently. Panics only fire on
//! converged-but-violated output, or non-finite solved positions.

#![cfg_attr(not(test), no_main)]

use arbitrary::Arbitrary;
#[cfg(not(test))]
use libfuzzer_sys::fuzz_target;
use remus_sketch::{Constraint, GcsSystem, PointData};

/// Residual tolerance for the geometric re-check: the solver converges to
/// `tolerance` (1e-9 below), so healthy converged residuals sit far below
/// this band; anything above it is a wrong success claim, not precision.
const RESIDUAL_TOL: f64 = 1e-6;

/// Solver iteration cap: bounded so a fuzz iteration stays fast.
const MAX_ITER: usize = 50;

/// Lattice coordinate in [-4.0, 4.0] on a half-unit grid.
fn coord(b: u8) -> f64 {
    (f64::from(b % 17) - 8.0) * 0.5
}

/// Small constraint magnitude in (0, 4.0].
fn mag(b: u8) -> f64 {
    0.5 + f64::from(b % 8) * 0.5
}

#[derive(Debug, Clone, Copy, Arbitrary)]
enum Constr {
    Coincident { a: u8, b: u8 },
    Distance { a: u8, b: u8, d: u8 },
    Horizontal { a: u8, b: u8 },
    Vertical { a: u8, b: u8 },
    FixX { a: u8, v: u8 },
    FixY { a: u8, v: u8 },
}

#[derive(Debug, Arbitrary)]
struct Case {
    n_points: u8,
    px: [u8; 4],
    py: [u8; 4],
    constraints: [Constr; 3],
}

#[cfg(not(test))]
fuzz_target!(|case: Case| run_case(case));

fn run_case(case: Case) {
    // 2..=5 points: enough for over- and under-constrained systems, small
    // enough to stay fast.
    let n = 2 + usize::from(case.n_points % 4);
    let mut sys = GcsSystem::new();
    let mut ids = Vec::with_capacity(n);
    for i in 0..n {
        let id = sys.add_point(PointData {
            x: coord(case.px[i % 4]),
            y: coord(case.py[i % 4]),
            fixed: false,
        });
        ids.push(id);
    }
    let at = |k: u8| ids[usize::from(k) % n];

    for c in case.constraints {
        let r: Result<_, _> = match c {
            Constr::Coincident { a, b } => sys
                .add_constraint(Constraint::Coincident(at(a), at(b)))
                .map(|_| ()),
            Constr::Distance { a, b, d } => sys
                .add_constraint(Constraint::Distance(at(a), at(b), mag(d)))
                .map(|_| ()),
            Constr::Horizontal { a, b } => {
                let (p, q) = (at(a), at(b));
                match sys.add_line(p, q) {
                    Ok(l) => sys.add_constraint(Constraint::Horizontal(l)).map(|_| ()),
                    Err(e) => Err(e),
                }
            }
            Constr::Vertical { a, b } => {
                let (p, q) = (at(a), at(b));
                match sys.add_line(p, q) {
                    Ok(l) => sys.add_constraint(Constraint::Vertical(l)).map(|_| ()),
                    Err(e) => Err(e),
                }
            }
            Constr::FixX { a, v } => sys
                .add_constraint(Constraint::FixX(at(a), coord(v)))
                .map(|_| ()),
            Constr::FixY { a, v } => sys
                .add_constraint(Constraint::FixY(at(a), coord(v)))
                .map(|_| ()),
        };
        // A constraint the builder refuses (duplicate, degenerate) is a
        // pass for this case, not a finding.
        if r.is_err() {
            return;
        }
    }

    let Ok(result) = sys.solve(MAX_ITER, 1e-9) else {
        return; // solver refusal / non-convergence is a pass
    };
    if !result.converged {
        return;
    }
    // Independent oracle: re-derive every constraint residual geometrically
    // from the solved positions. `point()` returns None on stale handles —
    // treat as a skip (builder-level, not solver output).
    let mut pts = Vec::with_capacity(n);
    for (i, id) in ids.iter().enumerate().take(n) {
        let Some(q) = sys.point(*id) else { return };
        assert!(
            q.x.is_finite() && q.y.is_finite(),
            "solver converged with non-finite position for point {i}: {q:?}",
        );
        pts.push((q.x, q.y));
    }
    let idx = |k: u8| usize::from(k) % n;
    for c in case.constraints {
        let residual = match c {
            Constr::Coincident { a, b } => {
                let (x1, y1) = pts[idx(a)];
                let (x2, y2) = pts[idx(b)];
                ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
            }
            Constr::Distance { a, b, d } => {
                let (x1, y1) = pts[idx(a)];
                let (x2, y2) = pts[idx(b)];
                (((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt() - mag(d)).abs()
            }
            Constr::Horizontal { a, b } => (pts[idx(a)].1 - pts[idx(b)].1).abs(),
            Constr::Vertical { a, b } => (pts[idx(a)].0 - pts[idx(b)].0).abs(),
            Constr::FixX { a, v } => (pts[idx(a)].0 - coord(v)).abs(),
            Constr::FixY { a, v } => (pts[idx(a)].1 - coord(v)).abs(),
        };
        assert!(
            residual <= RESIDUAL_TOL,
            "solver claimed convergence with violated {c:?}: residual {residual:.3e} (> {RESIDUAL_TOL:.0e})",
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_seed_constrains_distinct_points() {
        let data = include_bytes!("../corpus/gcs/coincident-pair");
        let case = Case::arbitrary(&mut arbitrary::Unstructured::new(data)).unwrap();
        assert_eq!(case.n_points, 0);
        assert_ne!(case.px[0], case.px[1]);
        assert!(
            case.constraints
                .iter()
                .all(|c| matches!(c, Constr::Coincident { a: 0, b: 1 }))
        );
        run_case(case);
    }
}
