//! Bounded sparse coupled-sketch solver (PERF-S04).
//!
//! Large connected sketches pay the dense Householder QR cubic term once per
//! DogLeg iteration, even though each residual row touches only a handful of
//! parameters (a `Distance` touches four coordinates, a `Symmetric` eight at
//! most). This module provides a bounded production slice for those systems:
//! structural CSR Jacobian assembly plus a banded Givens QR least-squares
//! step with the same trust-region, convergence, rollback, rank and
//! diagnostic policies as the dense loop.
//!
//! # Bounded slice (explicit)
//!
//! The sparse path is taken only when all of these hold, otherwise the caller
//! falls back to the dense loop with identical results:
//!
//! * `n >= SPARSE_MIN_N` (small systems keep the dense loop bit-for-bit;
//!   this preserves the PERF-S02 single-component bitwise contract for the
//!   existing `10`/`100`-parameter tests).
//! * `m >= n` (square or overdetermined; underconstrained `m < n` needs
//!   column pivoting to align the diagonal and is explicitly out of scope —
//!   it falls back safely).
//! * Scaled-diagonal bandwidth `bw <= SPARSE_MAX_BAND`, where `bw` is the
//!   ceiling of `max |j*m - i*n| / m` over structural nonzeros (column units).
//!   Chain workloads measure `bw ≈ 3-4`, grid workloads `bw ≈ 44`; `64`
//!   covers all qualified families with margin while keeping Givens fill
//!   `O(n*bw²)` firmly below the dense `O(n³)`.
//! * Full column rank under the single global `1e-10·max_col_norm` policy
//!   (the same threshold `dof::analyze` uses). Any tiny pivot falls back to
//!   dense so rank, DOF and classification never silently change.
//!
//! Far loop closures (one row coupling distant parameters, `bw` in the
//! hundreds), general sparse patterns with large bandwidth, and all
//! rank-deficient/ill-conditioned-below-threshold systems are unsupported and
//! fall back — correctness preserved, no speedup claimed there.
//!
//! # No normal equations
//!
//! The GN step solves `min ||J h + r||` via orthogonal Givens rotations,
//! never forming `JᵀJ` (which would square the condition number). Gradient
//! (`Jᵀr`) and products (`J·g`, `J·h`) are sparse matvecs `O(nnz)` with the
//! same operation order as the dense loop (rows in order, columns sorted),
//! so finite results are bit-identical to dense skipping zeros; nonfinite
//! inputs fail closed to all-`NaN` (matching dense `0·NaN = NaN` poisoning
//! where dense poisons everything and sparse would otherwise poison only
//! coupled entries).
//!
//! # Numerical dependencies: none
//!
//! This slice introduces no new workspace or external dependency
//! (`remus-sketch` keeps zero workspace deps per the layer table; only `std`
//! plus the existing `thiserror` diagnostic type). External sparse
//! candidates were evaluated and explicitly declined:
//!
//! * `faer` (Apache-2.0/MIT, pure Rust, WASM-compatible, deterministic with
//!   fixed ordering) provides sparse QR/LU with column orderings —
//!   overkill for a `bw ≤ 64` banded slice and a new audited dependency for
//!   marginal gain over in-house Givens.
//! * `sprs` (Apache-2.0/MIT, WASM-compatible) is CSR matvec/ordering only,
//!   no rank-revealing QR — would still need an in-house solver.
//! * `nalgebra-sparse`/`nalgebra` (Apache-2.0, WASM-compatible) likewise
//!   lacks a rank-revealing sparse QR and pulls a large generic stack.
//! * SuiteSparse/CXSparse (LGPL-2.1+, C/Fortran, not WASM-compatible without
//!   Emscripten shims, non-deterministic ordering in some paths) is
//!   incompatible with the `no unsafe`, deterministic, WASM-first constraints.
//!
//! In-house banded Givens is deterministic (rows in order, columns sorted via
//! `BTreeMap`, no `HashMap` iteration in numeric paths), WASM-compatible
//! (only `std` collections), and preserves the existing `1e-10` rank policy
//! by construction (same scale, fallback on any tiny pivot).
//!
//! # Determinism and ownership
//!
//! Patterns are built from [`crate::gcs::components::constraint_param_indices`]
//! (the exhaustive structural contract — unions over rows, never a numeric
//! observation) with sorted columns and CSR row pointers in arena order.
//! Union-find ordering, `BTreeMap` grouping and sorted factorizations keep
//! repeated builds identical. All workspaces are solve-local (PERF-S03
//! style): sized per call, reused across iterations, dropped after — never
//! retained across solves, so no stale-factor reuse (PERF-S05 remains
//! explicitly declined).

use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap};

use super::components::constraint_param_indices;
use super::constraint::{Constraint, JacobianSink};
use super::entity::ParamRef;
use super::solver::{SolveResult, SolveStats, dogleg_step_into, max_abs_residual};
use super::system::GcsSystem;

/// Minimum free parameters for the sparse path.
///
/// Below this the dense loop is kept bit-for-bit (preserving the PERF-S02
/// `single_component_stays_bitwise_dense` contract at `10`/`100` params and
/// avoiding sparse overhead where the cubic term is negligible).
pub const SPARSE_MIN_N: usize = 128;

/// Maximum scaled-diagonal bandwidth (column units) for the sparse path.
///
/// `bw = ceil(max |j*m - i*n| / m)` over structural nonzeros. Chain `≈3-4`,
/// grid `≈44`; `64` covers all qualified families with margin while keeping
/// `O(n*bw²)` below `O(n³)`. Larger patterns fall back safely.
pub const SPARSE_MAX_BAND: usize = 64;

/// Rank threshold shared with [`crate::gcs::dof`] (`1e-10·scale`).
const RANK_TOL: f64 = 1e-10;

/// Tiny-pivot guard matching `qr.rs` (`1e-300`).
const TINY: f64 = 1e-300;

/// Structural CSR pattern for one (component) system.
///
/// Columns are sorted per row; rows follow constraint arena order with each
/// constraint's union replicated per residual row (conservative: may include
/// structural zeros where a formula never reads that parameter in that row,
/// but never misses a structural nonzero — see `matrix_contract` tests).
#[derive(Debug, Clone)]
pub struct SparsePattern {
    /// Number of residual rows.
    pub m: usize,
    /// Number of free parameters (columns).
    pub n: usize,
    /// CSR row pointers, length `m+1`.
    pub row_ptr: Vec<usize>,
    /// CSR column indices, length `nnz`, sorted per row.
    pub col_idx: Vec<usize>,
    /// Scaled-diagonal bandwidth in column units (`ceil(max|j*m-i*n|/m)`).
    pub band: usize,
    /// Number of structural nonzeros.
    pub nnz: usize,
}

/// Build the structural pattern from the exhaustive reference contract.
///
/// `constraints` and `row_counts` must align (one count per constraint);
/// `param_index` maps `ParamRef` to columns (`local` for components, global
/// for whole-system dispatch). Columns per row are the constraint's union
/// (sorted, deduped by `constraint_param_indices`), replicated per row.
pub fn build_pattern(
    constraints: &[Constraint],
    row_counts: &[usize],
    sys: &GcsSystem,
    param_index: &HashMap<ParamRef, usize>,
    m: usize,
    n: usize,
) -> SparsePattern {
    let mut row_ptr = Vec::with_capacity(m + 1);
    let mut col_idx = Vec::new();
    row_ptr.push(0);
    let mut max_num: u64 = 0;
    let mut row: usize = 0;
    for (c, k) in constraints.iter().zip(row_counts.iter()) {
        let union = constraint_param_indices(c, sys, param_index);
        for _ in 0..*k {
            for &col in &union {
                col_idx.push(col);
                // Scaled-diagonal distance numerator |j*m - i*n|.
                let num = (col as u64)
                    .saturating_mul(m as u64)
                    .abs_diff((row as u64).saturating_mul(n as u64));
                if num > max_num {
                    max_num = num;
                }
            }
            row += 1;
            row_ptr.push(col_idx.len());
        }
    }
    let band = if m == 0 {
        0
    } else {
        // Ceiling division max_num / m.
        max_num.div_ceil(m as u64) as usize
    };
    let nnz = col_idx.len();
    SparsePattern {
        m,
        n,
        row_ptr,
        col_idx,
        band,
        nnz,
    }
}

/// Whether the bounded sparse slice applies (else fall back to dense).
///
/// Requires square/overdetermined, large enough, and banded. Underconstrained
/// (`m < n`), small, or wide-band systems return `false` — the caller runs
/// the dense loop with identical results.
///
/// Measurement escape hatch `REMUS_SKETCH_FORCE_DENSE=1` forces `false` so
/// paired same-host dense baselines can be taken from the same binary
/// (same source, same profile); production with the variable unset uses the
/// structural gate. On `wasm32` the variable is absent (`Err`) so the gate
/// is purely structural there.
#[must_use]
pub fn should_use_sparse(m: usize, n: usize, band: usize) -> bool {
    if std::env::var("REMUS_SKETCH_FORCE_DENSE").is_ok() {
        return false;
    }
    n >= SPARSE_MIN_N && m >= n && band <= SPARSE_MAX_BAND
}

/// Sparse CSR writer implementing [`JacobianSink`].
///
/// `values` parallels the pattern's `col_idx`; `set` overwrites and `add`
/// accumulates, ignoring missing parameters (fixed geometry) exactly like
/// the dense writer. Entries for `ParamRef`s outside the structural union
/// (contract violation, proven absent by tests) are ignored.
pub struct SparseWriter<'a> {
    /// CSR values, length `nnz`.
    pub values: &'a mut [f64],
    /// CSR row pointers.
    pub row_ptr: &'a [usize],
    /// CSR column indices (sorted per row).
    pub col_idx: &'a [usize],
    /// Map from `ParamRef` to column.
    pub param_index: &'a HashMap<ParamRef, usize>,
}

impl JacobianSink for SparseWriter<'_> {
    fn set(&mut self, row: usize, pr: ParamRef, val: f64) {
        if let Some(&col) = self.param_index.get(&pr)
            && let Some(&start) = self.row_ptr.get(row)
            && let Some(&end) = self.row_ptr.get(row + 1)
            && let Some(slice) = self.col_idx.get(start..end)
            && let Ok(pos) = slice.binary_search(&col)
            && let Some(slot) = self.values.get_mut(start + pos)
        {
            *slot = val;
        }
    }

    fn add(&mut self, row: usize, pr: ParamRef, val: f64) {
        if let Some(&col) = self.param_index.get(&pr)
            && let Some(&start) = self.row_ptr.get(row)
            && let Some(&end) = self.row_ptr.get(row + 1)
            && let Some(slice) = self.col_idx.get(start..end)
            && let Ok(pos) = slice.binary_search(&col)
            && let Some(slot) = self.values.get_mut(start + pos)
        {
            *slot += val;
        }
    }
}

/// Sparse `y = J·x` (`y` length `m`, `x` length `n`).
///
/// Finite results match dense skipping zeros bit-for-bit (`x+0 == x`);
/// any nonfinite input (or values) fails closed to all-`NaN`, matching
/// dense `0·NaN/Inf = NaN` poisoning (dense poisons everything, sparse would
/// otherwise poison only coupled entries).
pub fn sparse_matvec(pattern: &SparsePattern, values: &[f64], x: &[f64], y: &mut [f64]) {
    if y.len() < pattern.m || x.len() < pattern.n || values.len() < pattern.nnz {
        // Defensive length mismatch: fail closed (callers size correctly;
        // tests pin sizes).
        let take = pattern.m.min(y.len());
        for v in y.iter_mut().take(take) {
            *v = f64::NAN;
        }
        return;
    }
    if x.iter().take(pattern.n).any(|v| !v.is_finite())
        || values.iter().take(pattern.nnz).any(|v| !v.is_finite())
    {
        for v in y.iter_mut().take(pattern.m) {
            *v = f64::NAN;
        }
        return;
    }
    for i in 0..pattern.m {
        let start = pattern.row_ptr[i];
        let end = pattern.row_ptr[i + 1];
        let mut acc = 0.0_f64;
        for k in start..end {
            let col = pattern.col_idx[k];
            acc += values[k] * x[col];
        }
        y[i] = acc;
    }
}

/// Sparse `y = Jᵀ·x` (`y` length `n`, `x` length `m`).
///
/// Same fail-closed nonfinite contract as [`sparse_matvec`]; accumulation
/// order is rows in order (deterministic), matching dense inner-order
/// skipping zeros bit-for-bit on finite inputs.
pub fn sparse_matvec_transpose(pattern: &SparsePattern, values: &[f64], x: &[f64], y: &mut [f64]) {
    if y.len() < pattern.n || x.len() < pattern.m || values.len() < pattern.nnz {
        let take = pattern.n.min(y.len());
        for v in y.iter_mut().take(take) {
            *v = f64::NAN;
        }
        return;
    }
    if x.iter().take(pattern.m).any(|v| !v.is_finite())
        || values.iter().take(pattern.nnz).any(|v| !v.is_finite())
    {
        for v in y.iter_mut().take(pattern.n) {
            *v = f64::NAN;
        }
        return;
    }
    for v in y.iter_mut().take(pattern.n) {
        *v = 0.0;
    }
    for i in 0..pattern.m {
        let xi = x[i];
        let start = pattern.row_ptr[i];
        let end = pattern.row_ptr[i + 1];
        for k in start..end {
            let col = pattern.col_idx[k];
            y[col] += values[k] * xi;
        }
    }
}

/// Maximum column norm (`sqrt(sum x²)`) of the CSR matrix.
///
/// Equals the dense pivoted `|R00|` (max column norm) for the same values;
/// used as the single global rank scale (same `1e-10` policy as dense).
pub fn max_column_norm(pattern: &SparsePattern, values: &[f64]) -> f64 {
    if pattern.m == 0 || pattern.n == 0 || pattern.nnz == 0 {
        return 0.0;
    }
    let mut sums = vec![0.0_f64; pattern.n];
    for i in 0..pattern.m {
        let start = pattern.row_ptr[i];
        let end = pattern.row_ptr[i + 1];
        for k in start..end {
            let col = pattern.col_idx[k];
            if let Some(s) = sums.get_mut(col) {
                let v = values[k];
                *s += v * v;
            }
        }
    }
    let mut best: f64 = 0.0;
    for s in sums {
        // NaN propagates: a poisoned norm must fail the rank gate.
        if s.is_nan() {
            return f64::NAN;
        }
        let n = s.sqrt();
        if n.is_nan() {
            return f64::NAN;
        }
        if n > best {
            best = n;
        }
    }
    best
}

/// Check full column rank under an explicit global threshold.
///
/// Factorizes `J` via banded Givens (same as [`sparse_givens_solve`] but
/// without a right-hand side) and returns `true` only when every diagonal
/// satisfies `|Rkk| > threshold`. Any tiny pivot, nonfinite, or structural
/// miss returns `false` so the caller falls back to dense for exact rank.
/// Used for final-rank analysis where the threshold is the single global
/// `1e-10·max_over_blocks` (PERF-S02 policy), not the block's own scale.
#[allow(clippy::too_many_lines)]
pub fn sparse_full_rank_with_threshold(
    pattern: &SparsePattern,
    values: &[f64],
    threshold: f64,
) -> bool {
    let m = pattern.m;
    let n = pattern.n;
    if m < n || m == 0 || n == 0 || values.len() < pattern.nnz || !threshold.is_finite() {
        return false;
    }
    if values.iter().take(pattern.nnz).any(|v| !v.is_finite()) {
        return false;
    }
    let mut rows: Vec<BTreeMap<usize, f64>> = Vec::with_capacity(m);
    for i in 0..m {
        let mut map = BTreeMap::new();
        let start = pattern.row_ptr[i];
        let end = pattern.row_ptr[i + 1];
        for k in start..end {
            let col = pattern.col_idx[k];
            let v = values[k];
            if v != 0.0 {
                map.insert(col, v);
            }
        }
        rows.push(map);
    }
    let bw = pattern.band;
    for k in 0..n {
        let needs_swap = match rows.get(k).and_then(|r| r.get(&k).copied()) {
            Some(v) => v.abs() < TINY,
            None => true,
        };
        if needs_swap {
            let hi = ((k as u64 + bw as u64 + 1)
                .saturating_mul(m as u64)
                .saturating_add(n as u64 - 1)
                / n.max(1) as u64) as usize;
            let hi = hi.min(m).saturating_add(1).min(m);
            let mut swap_with: Option<usize> = None;
            for i in (k + 1)..hi {
                if let Some(r) = rows.get(i)
                    && let Some(&v) = r.get(&k)
                    && v.abs() >= TINY
                {
                    swap_with = Some(i);
                    break;
                }
            }
            if let Some(i) = swap_with {
                rows.swap(k, i);
            } else {
                return false;
            }
        }
        let Some(akk) = rows.get(k).and_then(|r| r.get(&k).copied()) else {
            return false;
        };
        if akk.abs() < TINY {
            return false;
        }
        let hi = ((k as u64 + bw as u64 + 1)
            .saturating_mul(m as u64)
            .saturating_add(n as u64 - 1)
            / n.max(1) as u64) as usize;
        let hi = hi.min(m).saturating_add(1).min(m);
        let mut targets: Vec<usize> = Vec::new();
        for i in (k + 1)..hi {
            if let Some(r) = rows.get(i)
                && let Some(&v) = r.get(&k)
                && v.abs() >= TINY
            {
                targets.push(i);
            }
        }
        for i in targets {
            let a = rows.get(k).and_then(|r| r.get(&k).copied()).unwrap_or(0.0);
            let b = rows.get(i).and_then(|r| r.get(&k).copied()).unwrap_or(0.0);
            let r_norm = a.hypot(b);
            if r_norm < TINY {
                continue;
            }
            let c = a / r_norm;
            let s = b / r_norm;
            let mut cols: Vec<usize> = Vec::new();
            if let Some(rk) = rows.get(k) {
                for &j in rk.keys() {
                    if j > k {
                        cols.push(j);
                    }
                }
            }
            if let Some(ri) = rows.get(i) {
                for &j in ri.keys() {
                    if j > k {
                        cols.push(j);
                    }
                }
            }
            cols.sort_unstable();
            cols.dedup();
            for j in cols {
                let ak = rows.get(k).and_then(|r| r.get(&j).copied()).unwrap_or(0.0);
                let ai = rows.get(i).and_then(|r| r.get(&j).copied()).unwrap_or(0.0);
                let nk = c * ak + s * ai;
                let ni = -s * ak + c * ai;
                if let Some(rk) = rows.get_mut(k) {
                    if nk == 0.0 {
                        rk.remove(&j);
                    } else {
                        rk.insert(j, nk);
                    }
                }
                if let Some(ri) = rows.get_mut(i) {
                    if ni == 0.0 {
                        ri.remove(&j);
                    } else {
                        ri.insert(j, ni);
                    }
                }
            }
            if let Some(rk) = rows.get_mut(k) {
                rk.insert(k, r_norm);
            }
            if let Some(ri) = rows.get_mut(i) {
                ri.remove(&k);
            }
        }
    }
    for k in 0..n {
        let d = rows.get(k).and_then(|r| r.get(&k).copied()).unwrap_or(0.0);
        // Fail closed on NaN: partial_cmp returns None, which != Greater, so NaN falls back.
        if d.abs().partial_cmp(&threshold) != Some(Ordering::Greater) {
            return false;
        }
    }
    true
}

/// Solve `min ||J h + r||` via banded Givens QR.
///
/// Returns `Some(h)` only for full column rank under the global
/// `1e-10·max_col_norm` policy; any tiny pivot, nonfinite input/output, or
/// structural miss returns `None` so the caller falls back to dense with
/// identical diagnostics. Row swaps (smallest suitable row, deterministic)
/// handle extra rows whose pivot starts below `k`; column order is never
/// permuted (bounded slice requires `m >= n` banded alignment).
#[allow(clippy::too_many_lines)]
pub fn sparse_givens_solve(
    pattern: &SparsePattern,
    values: &[f64],
    neg_rhs: &[f64],
) -> Option<Vec<f64>> {
    let m = pattern.m;
    let n = pattern.n;
    if m < n || m == 0 || n == 0 || neg_rhs.len() < m || values.len() < pattern.nnz {
        return None;
    }
    if neg_rhs.iter().take(m).any(|v| !v.is_finite())
        || values.iter().take(pattern.nnz).any(|v| !v.is_finite())
    {
        return None;
    }
    let global = max_column_norm(pattern, values);
    if !global.is_finite() || global < TINY {
        return None;
    }
    let threshold = RANK_TOL * global;

    // R as ordered row maps (deterministic, sorted keys).
    let mut rows: Vec<BTreeMap<usize, f64>> = Vec::with_capacity(m);
    for i in 0..m {
        let mut map = BTreeMap::new();
        let start = pattern.row_ptr[i];
        let end = pattern.row_ptr[i + 1];
        for k in start..end {
            let col = pattern.col_idx[k];
            let v = values[k];
            if v != 0.0 {
                map.insert(col, v);
            }
        }
        rows.push(map);
    }
    let mut qtb: Vec<f64> = neg_rhs[..m].to_vec();

    let bw = pattern.band;
    for k in 0..n {
        // Pivot: ensure rows[k][k] nonzero via the smallest row swap.
        let needs_swap = match rows.get(k).and_then(|r| r.get(&k).copied()) {
            Some(v) => v.abs() < TINY,
            None => true,
        };
        if needs_swap {
            // Upper scan bound from the band: i <= (k+bw)*m/n (+1 margin).
            let hi = ((k as u64 + bw as u64 + 1)
                .saturating_mul(m as u64)
                .saturating_add(n as u64 - 1)
                / n.max(1) as u64) as usize;
            let hi = hi.min(m).saturating_add(1).min(m);
            let mut swap_with: Option<usize> = None;
            for i in (k + 1)..hi {
                if let Some(r) = rows.get(i)
                    && let Some(&v) = r.get(&k)
                    && v.abs() >= TINY
                {
                    swap_with = Some(i);
                    break;
                }
            }
            if let Some(i) = swap_with {
                rows.swap(k, i);
                qtb.swap(k, i);
            } else {
                return None;
            }
        }
        let akk = rows.get(k).and_then(|r| r.get(&k).copied())?;
        if akk.abs() < TINY {
            return None;
        }
        // Eliminate rows below k that carry column k (band-limited scan).
        let hi = ((k as u64 + bw as u64 + 1)
            .saturating_mul(m as u64)
            .saturating_add(n as u64 - 1)
            / n.max(1) as u64) as usize;
        let hi = hi.min(m).saturating_add(1).min(m);
        // Collect targets first (deterministic increasing order).
        let mut targets: Vec<usize> = Vec::new();
        for i in (k + 1)..hi {
            if let Some(r) = rows.get(i)
                && let Some(&v) = r.get(&k)
                && v.abs() >= TINY
            {
                targets.push(i);
            }
        }
        for i in targets {
            let a = rows.get(k).and_then(|r| r.get(&k).copied()).unwrap_or(0.0);
            let b = rows.get(i).and_then(|r| r.get(&k).copied()).unwrap_or(0.0);
            let r_norm = a.hypot(b);
            if r_norm < TINY {
                continue;
            }
            let c = a / r_norm;
            let s = b / r_norm;
            // Union of columns > k (sorted via BTreeSet for determinism).
            // Widths are tiny (<= ~2*bw), so a small sorted Vec suffices.
            let mut cols: Vec<usize> = Vec::new();
            if let Some(rk) = rows.get(k) {
                for &j in rk.keys() {
                    if j > k {
                        cols.push(j);
                    }
                }
            }
            if let Some(ri) = rows.get(i) {
                for &j in ri.keys() {
                    if j > k {
                        cols.push(j);
                    }
                }
            }
            cols.sort_unstable();
            cols.dedup();
            for j in cols {
                let ak = rows.get(k).and_then(|r| r.get(&j).copied()).unwrap_or(0.0);
                let ai = rows.get(i).and_then(|r| r.get(&j).copied()).unwrap_or(0.0);
                let nk = c * ak + s * ai;
                let ni = -s * ak + c * ai;
                if let Some(rk) = rows.get_mut(k) {
                    if nk == 0.0 {
                        rk.remove(&j);
                    } else {
                        rk.insert(j, nk);
                    }
                }
                if let Some(ri) = rows.get_mut(i) {
                    if ni == 0.0 {
                        ri.remove(&j);
                    } else {
                        ri.insert(j, ni);
                    }
                }
            }
            // Diagonal: R[k][k] = r, R[i][k] = 0 (exactly, by construction).
            if let Some(rk) = rows.get_mut(k) {
                rk.insert(k, r_norm);
            }
            if let Some(ri) = rows.get_mut(i) {
                ri.remove(&k);
            }
            let qk = qtb[k];
            let qi = qtb[i];
            qtb[k] = c * qk + s * qi;
            qtb[i] = -s * qk + c * qi;
        }
    }

    // Rank gate: every diagonal above the single global threshold.
    for k in 0..n {
        let d = rows.get(k).and_then(|r| r.get(&k).copied()).unwrap_or(0.0);
        // Fail closed on NaN: partial_cmp returns None, which != Greater, so NaN falls back.
        if d.abs().partial_cmp(&threshold) != Some(Ordering::Greater) {
            return None;
        }
    }

    // Back-substitution (upper-triangular, band-limited).
    let mut h = vec![0.0_f64; n];
    for rev in 0..n {
        let i = n - 1 - rev;
        let row = rows.get(i)?;
        let &diag = row.get(&i)?;
        if diag.abs() < TINY {
            return None;
        }
        let mut acc = qtb[i];
        for (&j, &v) in row {
            if j > i {
                if let Some(&hj) = h.get(j) {
                    acc -= v * hj;
                } else {
                    return None;
                }
            }
        }
        h[i] = acc / diag;
    }
    if h.iter().any(|v| !v.is_finite()) {
        return None;
    }
    Some(h)
}

/// Solve-local reusable scratch for the sparse DogLeg loop (PERF-S04).
///
/// Mirrors [`super::solver::DoglegWorkspace`] sizing (`m` residuals, `n`
/// params) but backs the banded Givens path: CSR values plus ordered row
/// maps reused across iterations. Nothing is retained across solves.
#[derive(Debug, Default)]
pub struct SparseWorkspace {
    /// CSR values (`nnz`), refilled per iteration.
    values: Vec<f64>,
    /// Current residuals (`m`).
    r: Vec<f64>,
    /// Trial residuals (`m`).
    r_trial: Vec<f64>,
    /// Negated residuals (`m`, RHS for the GN solve).
    neg_r: Vec<f64>,
    /// Gradient `Jᵀr` (`n`).
    g: Vec<f64>,
    /// `J·g` (`m`).
    jg: Vec<f64>,
    /// `J·h` (`m`).
    jh: Vec<f64>,
    /// Gauss-Newton step (`n`).
    h_gn: Vec<f64>,
    /// Steepest-descent step (`n`).
    h_sd: Vec<f64>,
    /// Selected DogLeg step (`n`).
    h: Vec<f64>,
    /// Trial params (`n`).
    trial: Vec<f64>,
    /// `h_gn - h_sd` scratch (`n`).
    diff: Vec<f64>,
    /// `Qᵀb` scratch is owned by [`sparse_givens_solve`] per call; this
    /// workspace keeps no factor across iterations (no factor reuse —
    /// an accepted step invalidates the previous factorization, same as
    /// dense).
    qtb_dummy: Vec<f64>,
}

impl SparseWorkspace {
    /// Create empty scratch; buffers are sized on first use.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Residual vector at the solver's final iterate (same contract as
    /// [`super::solver::DoglegWorkspace::final_residuals`]).
    #[must_use]
    pub fn final_residuals(&self) -> &[f64] {
        &self.r
    }

    /// Size all buffers for `m` residuals, `n` params and `nnz` nonzeros.
    pub fn ensure(&mut self, m: usize, n: usize, nnz: usize) {
        self.values.resize(nnz, 0.0);
        self.r.resize(m, 0.0);
        self.r_trial.resize(m, 0.0);
        self.neg_r.resize(m, 0.0);
        self.g.resize(n, 0.0);
        self.jg.resize(m, 0.0);
        self.jh.resize(m, 0.0);
        self.h_gn.resize(n, 0.0);
        self.h_sd.resize(n, 0.0);
        self.h.resize(n, 0.0);
        self.trial.resize(n, 0.0);
        self.diff.resize(n, 0.0);
        self.qtb_dummy.resize(m, 0.0);
    }
}

/// Sparse DogLeg loop mirroring [`super::solver::solve_dogleg_fill`].
///
/// Trust-region constants, convergence norm (`max_abs_residual < tol`),
/// `rho` thresholds (`0.75`/`0.25`), small-step exit
/// (`1e-15·(1+|p|)`), and iteration accounting match dense exactly; only
/// Jacobian storage (CSR), matvecs (sparse) and the GN factorization
/// (banded Givens) differ. Returns `None` when any iteration needs dense
/// fallback (tiny pivot, nonfinite, structural miss) so the caller reruns
/// dense from the same start with identical diagnostics.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub fn solve_dogleg_sparse<F, J>(
    params: &mut [f64],
    residual_fill: &mut F,
    jacobian_fill_sparse: &mut J,
    pattern: &SparsePattern,
    max_iter: usize,
    tol: f64,
    workspace: &mut SparseWorkspace,
    stats: &mut SolveStats,
) -> Option<SolveResult>
where
    F: FnMut(&[f64], &mut Vec<f64>),
    J: FnMut(&[f64], &mut [f64]),
{
    let n = params.len();
    let m = pattern.m;
    if n == 0 || m == 0 || n != pattern.n {
        return None;
    }
    workspace.ensure(m, n, pattern.nnz);
    // Cache slices to avoid borrow conflicts (same pattern as dense `ws`).
    let ws = workspace;

    let param_norm: f64 = params.iter().map(|x| x * x).sum::<f64>().sqrt();
    let delta_max = 1e4;
    let delta_min = 1e-15;
    let mut delta = (1.0_f64).max(0.1 * param_norm).min(delta_max);

    for iteration in 0..max_iter {
        ws.r.clear();
        residual_fill(params, &mut ws.r);
        stats.residual_evals += 1;
        let max_r = max_abs_residual(&ws.r);
        if max_r < tol {
            return Some(SolveResult {
                converged: true,
                iterations: iteration,
                max_residual: max_r,
            });
        }

        jacobian_fill_sparse(params, &mut ws.values);
        stats.jacobian_evals += 1;
        for (d, &v) in ws.neg_r.iter_mut().zip(ws.r.iter()) {
            *d = -v;
        }
        let h_gn = sparse_givens_solve(pattern, &ws.values, &ws.neg_r)?;
        stats.qr_factorizations += 1;
        ws.h_gn.copy_from_slice(&h_gn);

        // Gradient g = Jᵀ·r (sparse, same order as dense skipping zeros).
        sparse_matvec_transpose(pattern, &ws.values, &ws.r, &mut ws.g);

        let g_norm_sq: f64 = ws.g.iter().map(|&v| v * v).sum();
        if g_norm_sq < TINY {
            return Some(SolveResult {
                converged: max_r < tol,
                iterations: iteration,
                max_residual: max_r,
            });
        }

        sparse_matvec(pattern, &ws.values, &ws.g, &mut ws.jg);
        let jg_norm_sq: f64 = ws.jg.iter().map(|&v| v * v).sum();
        let alpha = if jg_norm_sq > TINY {
            g_norm_sq / jg_norm_sq
        } else {
            1.0
        };
        for (d, &v) in ws.h_sd.iter_mut().zip(ws.g.iter()) {
            *d = -alpha * v;
        }

        dogleg_step_into(&ws.h_gn, &ws.h_sd, delta, &mut ws.h, &mut ws.diff);
        let h_norm = ws.h.iter().map(|&v| v * v).sum::<f64>().sqrt();

        for (t, (&p, &d)) in ws.trial.iter_mut().zip(params.iter().zip(ws.h.iter())) {
            *t = p + d;
        }
        ws.r_trial.clear();
        residual_fill(&ws.trial, &mut ws.r_trial);
        stats.residual_evals += 1;

        let cost_current: f64 = ws.r.iter().map(|&v| v * v).sum::<f64>() * 0.5;
        let cost_trial: f64 = ws.r_trial.iter().map(|&v| v * v).sum::<f64>() * 0.5;
        let actual_reduction = cost_current - cost_trial;

        sparse_matvec(pattern, &ws.values, &ws.h, &mut ws.jh);
        let predicted: f64 = {
            let mut pred = 0.0;
            for i in 0..m {
                pred += ws.r[i] * ws.jh[i];
                pred += 0.5 * ws.jh[i] * ws.jh[i];
            }
            -pred
        };
        let rho = if predicted.abs() < TINY {
            if actual_reduction > 0.0 { 1.0 } else { 0.0 }
        } else {
            actual_reduction / predicted
        };

        if rho > 0.75 {
            delta = (2.0 * delta).min(delta_max);
        } else if rho < 0.25 {
            delta = (delta / 4.0).max(delta_min);
        }
        if rho > 0.0 {
            params.copy_from_slice(&ws.trial);
        }
        if h_norm < 1e-15 * (1.0 + param_norm) {
            ws.r_trial.clear();
            residual_fill(params, &mut ws.r_trial);
            stats.residual_evals += 1;
            let final_max = max_abs_residual(&ws.r_trial);
            ws.r.clear();
            ws.r.extend_from_slice(&ws.r_trial);
            return Some(SolveResult {
                converged: final_max < tol,
                iterations: iteration + 1,
                max_residual: final_max,
            });
        }
    }

    ws.r.clear();
    residual_fill(params, &mut ws.r);
    stats.residual_evals += 1;
    let max_r = max_abs_residual(&ws.r);
    Some(SolveResult {
        converged: max_r < tol,
        iterations: max_iter,
        max_residual: max_r,
    })
}

#[cfg(test)]
mod tests;
