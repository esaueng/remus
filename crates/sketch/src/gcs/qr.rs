//! Householder QR factorization with column pivoting.
//!
//! Rank-revealing factorization used by the DogLeg solver for least-squares
//! and by DOF analysis for rank detection. Column pivoting selects the
//! column with the largest remaining norm at each step, ensuring the
//! diagonal of R is non-increasing in magnitude.

/// Result of a QR factorization with column pivoting.
///
/// Stores the factored matrix in compact form: Householder vectors below
/// the diagonal, R on and above the diagonal.
pub struct QrResult {
    /// Row-major storage: Householder vectors below diagonal, R on/above.
    data: Vec<f64>,
    /// Householder scaling factors (one per column processed).
    #[allow(dead_code)]
    tau: Vec<f64>,
    /// Column permutation: `perm[k]` is the original column index of
    /// the k-th pivot column.
    #[allow(dead_code)]
    perm: Vec<usize>,
    /// Number of rows.
    m: usize,
    /// Number of columns.
    n: usize,
}

impl QrResult {
    /// Perform QR factorization with column pivoting on an m×n matrix.
    ///
    /// `data` is row-major, length `m * n`. It is modified in-place.
    /// # Panics
    ///
    /// Panics if `data.len() < m * n`. This is a precondition invariant —
    /// callers within the GCS module always provide correctly-sized buffers.
    #[allow(clippy::too_many_lines, clippy::missing_panics_doc)]
    pub fn factorize(data: &mut [f64], m: usize, n: usize) -> Self {
        debug_assert!(
            data.len() >= m * n,
            "data length {} < m*n = {}",
            data.len(),
            m * n
        );

        let k = m.min(n);
        let mut tau = vec![0.0; k];
        let mut perm: Vec<usize> = (0..n).collect();

        // Column norms for pivoting
        let mut col_norms = vec![0.0; n];
        for j in 0..n {
            let mut s = 0.0;
            for i in 0..m {
                let v = data[i * n + j];
                s += v * v;
            }
            col_norms[j] = s;
        }

        for step in 0..k {
            // Column pivoting: find column with largest remaining norm
            let mut best_col = step;
            let mut best_norm = col_norms[step];
            for j in (step + 1)..n {
                if col_norms[j] > best_norm {
                    best_norm = col_norms[j];
                    best_col = j;
                }
            }

            if best_col != step {
                for i in 0..m {
                    data.swap(i * n + step, i * n + best_col);
                }
                col_norms.swap(step, best_col);
                perm.swap(step, best_col);
            }

            // Compute Householder reflector for column `step`, rows step..m
            let mut norm_sq = 0.0;
            for i in step..m {
                let v = data[i * n + step];
                norm_sq += v * v;
            }

            if norm_sq < 1e-300 {
                tau[step] = 0.0;
                continue;
            }

            let norm = norm_sq.sqrt();
            let alpha = data[step * n + step];
            let beta = if alpha >= 0.0 { -norm } else { norm };
            tau[step] = (beta - alpha) / beta;
            let scale = 1.0 / (alpha - beta);

            for i in (step + 1)..m {
                data[i * n + step] *= scale;
            }
            data[step * n + step] = beta;

            // Apply reflector to remaining columns
            for j in (step + 1)..n {
                let mut dot = data[step * n + j];
                for i in (step + 1)..m {
                    dot += data[i * n + step] * data[i * n + j];
                }
                let t = tau[step] * dot;
                data[step * n + j] -= t;
                for i in (step + 1)..m {
                    data[i * n + j] -= data[i * n + step] * t;
                }
            }

            // Update remaining column norms (downdate).
            // Periodic recomputation prevents accumulated rounding errors
            // from corrupting pivot selection in large systems.
            let recompute_interval = (k / 4).max(1);
            let needs_recompute = (step + 1) % recompute_interval == 0;

            for j in (step + 1)..n {
                if needs_recompute {
                    // Full recomputation from the sub-column below the diagonal
                    let mut s = 0.0;
                    for i in (step + 1)..m {
                        let v = data[i * n + j];
                        s += v * v;
                    }
                    col_norms[j] = s;
                } else {
                    let v = data[step * n + j];
                    col_norms[j] -= v * v;
                    if col_norms[j] < 0.0 {
                        col_norms[j] = 0.0;
                    }
                }
            }
        }

        Self {
            data: data.to_vec(),
            tau,
            perm,
            m,
            n,
        }
    }

    /// Factorize in place, reusing caller-owned scratch (PERF-S03).
    ///
    /// Same matrix layout and numerical operation order as [`Self::factorize`],
    /// but `tau`, `perm` and `col_norms` are resized and fully overwritten
    /// instead of allocated, and the factored matrix is left in `data`
    /// without the owned `data.to_vec()` copy. `perm` is reinitialized to
    /// `0..n` on every call so a previous solve's permutation cannot leak
    /// into a resized system.
    #[allow(clippy::too_many_lines, clippy::missing_panics_doc)]
    pub fn factorize_reuse(
        data: &mut [f64],
        m: usize,
        n: usize,
        tau: &mut Vec<f64>,
        perm: &mut Vec<usize>,
        col_norms: &mut Vec<f64>,
    ) {
        debug_assert!(
            data.len() >= m * n,
            "data length {} < m*n = {}",
            data.len(),
            m * n
        );

        let k = m.min(n);
        tau.resize(k, 0.0);
        perm.clear();
        perm.extend(0..n);
        col_norms.resize(n, 0.0);

        // Column norms for pivoting
        for j in 0..n {
            let mut s = 0.0;
            for i in 0..m {
                let v = data[i * n + j];
                s += v * v;
            }
            col_norms[j] = s;
        }

        for step in 0..k {
            // Column pivoting: find column with largest remaining norm
            let mut best_col = step;
            let mut best_norm = col_norms[step];
            for j in (step + 1)..n {
                if col_norms[j] > best_norm {
                    best_norm = col_norms[j];
                    best_col = j;
                }
            }

            if best_col != step {
                for i in 0..m {
                    data.swap(i * n + step, i * n + best_col);
                }
                col_norms.swap(step, best_col);
                perm.swap(step, best_col);
            }

            // Compute Householder reflector for column `step`, rows step..m
            let mut norm_sq = 0.0;
            for i in step..m {
                let v = data[i * n + step];
                norm_sq += v * v;
            }

            if norm_sq < 1e-300 {
                tau[step] = 0.0;
                continue;
            }

            let norm = norm_sq.sqrt();
            let alpha = data[step * n + step];
            let beta = if alpha >= 0.0 { -norm } else { norm };
            tau[step] = (beta - alpha) / beta;
            let scale = 1.0 / (alpha - beta);

            for i in (step + 1)..m {
                data[i * n + step] *= scale;
            }
            data[step * n + step] = beta;

            // Apply reflector to remaining columns
            for j in (step + 1)..n {
                let mut dot = data[step * n + j];
                for i in (step + 1)..m {
                    dot += data[i * n + step] * data[i * n + j];
                }
                let t = tau[step] * dot;
                data[step * n + j] -= t;
                for i in (step + 1)..m {
                    data[i * n + j] -= data[i * n + step] * t;
                }
            }

            // Update remaining column norms (downdate).
            // Periodic recomputation prevents accumulated rounding errors
            // from corrupting pivot selection in large systems.
            let recompute_interval = (k / 4).max(1);
            let needs_recompute = (step + 1) % recompute_interval == 0;

            for j in (step + 1)..n {
                if needs_recompute {
                    // Full recomputation from the sub-column below the diagonal
                    let mut s = 0.0;
                    for i in (step + 1)..m {
                        let v = data[i * n + j];
                        s += v * v;
                    }
                    col_norms[j] = s;
                } else {
                    let v = data[step * n + j];
                    col_norms[j] -= v * v;
                    if col_norms[j] < 0.0 {
                        col_norms[j] = 0.0;
                    }
                }
            }
        }
    }

    /// Compute Q^T * b into `out` (PERF-S03).
    ///
    /// Same operation order as [`Self::qt_mul`]; `out` is fully overwritten
    /// from `b` so no stale entries survive a resize.
    pub fn qt_mul_into(
        factored: &[f64],
        tau: &[f64],
        m: usize,
        n: usize,
        b: &[f64],
        out: &mut [f64],
    ) {
        debug_assert!(out.len() >= m);
        debug_assert!(b.len() >= m);
        out[..m].copy_from_slice(&b[..m]);
        let k = m.min(n);
        for step in 0..k {
            if tau[step].abs() < 1e-300 {
                continue;
            }
            let mut dot = out[step];
            for i in (step + 1)..m {
                dot += factored[i * n + step] * out[i];
            }
            let t = tau[step] * dot;
            out[step] -= t;
            for i in (step + 1)..m {
                out[i] -= factored[i * n + step] * t;
            }
        }
    }

    /// Solve least-squares into `out` (PERF-S03).
    ///
    /// Same operation order as [`Self::solve_least_squares`]; `out` (len `n`)
    /// is fully overwritten via the permutation, and `tmp_qtb` (len `m`) /
    /// `tmp_z` (len `n`) are solve-local scratch.
    #[allow(clippy::too_many_arguments)]
    pub fn solve_least_squares_into(
        factored: &[f64],
        tau: &[f64],
        perm: &[usize],
        m: usize,
        n: usize,
        b: &[f64],
        out: &mut [f64],
        tmp_qtb: &mut [f64],
        tmp_z: &mut [f64],
    ) {
        debug_assert!(out.len() >= n);
        debug_assert!(tmp_qtb.len() >= m);
        debug_assert!(tmp_z.len() >= n);
        Self::qt_mul_into(factored, tau, m, n, b, &mut tmp_qtb[..m]);
        let k = m.min(n);

        for v in tmp_z.iter_mut().take(n) {
            *v = 0.0;
        }
        // Back-substitute R * z = qtb[0..k]
        for i in (0..k).rev() {
            let rii = factored[i * n + i];
            if rii.abs() < 1e-300 {
                continue;
            }
            let mut s = tmp_qtb[i];
            for j in (i + 1)..k.min(n) {
                s -= factored[i * n + j] * tmp_z[j];
            }
            tmp_z[i] = s / rii;
        }

        for (i, &pi) in perm.iter().enumerate().take(n) {
            out[pi] = tmp_z[i];
        }
    }

    /// Numerical rank, counting diagonal elements of R with
    /// `|R[i,i]| > tol * |R[0,0]|`.
    #[must_use]
    pub fn rank(&self, tol: f64) -> usize {
        let k = self.m.min(self.n);
        if k == 0 {
            return 0;
        }
        let r00 = self.data[0].abs();
        if r00 < 1e-300 {
            return 0;
        }
        let threshold = tol * r00;
        let mut rank = 0;
        for i in 0..k {
            if self.data[i * self.n + i].abs() > threshold {
                rank += 1;
            } else {
                break;
            }
        }
        rank
    }

    /// Leading magnitude `|R[0,0]|` the relative [`Self::rank`] threshold
    /// scales from (PERF-S02).
    ///
    /// Column pivoting promotes the largest remaining column norm first, so
    /// this equals the largest column norm of the factorized matrix — and the
    /// max over independent blocks equals the leading magnitude a global
    /// factorization of the block-diagonal assembly would produce, since
    /// Householder reflections never couple zero-separated blocks. Returns 0
    /// for an empty factorization.
    #[must_use]
    pub fn leading_magnitude(&self) -> f64 {
        let k = self.m.min(self.n);
        if k == 0 {
            return 0.0;
        }
        self.data[0].abs()
    }

    /// Numerical rank against an absolute threshold (PERF-S02).
    ///
    /// Counts diagonal elements of R with `|R[i,i]| > threshold`, stopping at
    /// the first one that fails (pivoting keeps the diagonal non-increasing in
    /// magnitude). This is [`Self::rank`] with the caller supplying
    /// `tol * global_scale` instead of the block's own scale, so independent
    /// blocks aggregate under one global rank policy without silently
    /// re-ranking mixed-scale systems per block.
    #[must_use]
    pub fn rank_absolute(&self, threshold: f64) -> usize {
        let k = self.m.min(self.n);
        let mut rank = 0;
        for i in 0..k {
            if self.data[i * self.n + i].abs() > threshold {
                rank += 1;
            } else {
                break;
            }
        }
        rank
    }

    /// Compute Q^T * b.
    ///
    /// Retained for the QR unit tests and `dof` callers; the solver loop
    /// uses [`Self::qt_mul_into`] with reused scratch instead.
    #[allow(dead_code)]
    pub fn qt_mul(&self, b: &[f64]) -> Vec<f64> {
        let mut result = b.to_vec();
        let k = self.m.min(self.n);
        for step in 0..k {
            if self.tau[step].abs() < 1e-300 {
                continue;
            }
            let mut dot = result[step];
            for i in (step + 1)..self.m {
                dot += self.data[i * self.n + step] * result[i];
            }
            let t = self.tau[step] * dot;
            result[step] -= t;
            for i in (step + 1)..self.m {
                result[i] -= self.data[i * self.n + step] * t;
            }
        }
        result
    }

    /// Solve the least-squares problem min ||Jx - b|| via back-substitution
    /// on the R factor, then unpermute.
    ///
    /// Retained for the QR unit tests; the solver loop uses
    /// [`Self::solve_least_squares_into`] with reused scratch instead.
    #[allow(dead_code)]
    pub fn solve_least_squares(&self, b: &[f64]) -> Vec<f64> {
        let qtb = self.qt_mul(b);
        let k = self.m.min(self.n);

        // Back-substitute R * z = qtb[0..k]
        let mut z = vec![0.0; self.n];
        for i in (0..k).rev() {
            let rii = self.data[i * self.n + i];
            if rii.abs() < 1e-300 {
                continue;
            }
            let mut s = qtb[i];
            for j in (i + 1)..k.min(self.n) {
                s -= self.data[i * self.n + j] * z[j];
            }
            z[i] = s / rii;
        }

        let mut x = vec![0.0; self.n];
        for (i, &pi) in self.perm.iter().enumerate() {
            x[pi] = z[i];
        }
        x
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn identity_3x3() {
        let mut data = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let qr = QrResult::factorize(&mut data, 3, 3);
        assert_eq!(qr.rank(1e-10), 3);
    }

    #[test]
    fn known_3x3() {
        // A = [[1, 2, 3], [4, 5, 6], [7, 8, 10]]
        let mut data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0];
        let qr = QrResult::factorize(&mut data, 3, 3);
        assert_eq!(qr.rank(1e-10), 3);

        // Solve Ax = [1, 1, 1]
        let x = qr.solve_least_squares(&[1.0, 1.0, 1.0]);
        // Verify Ax ≈ [1, 1, 1]
        let a = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0];
        for i in 0..3 {
            let row_sum = a[i * 3] * x[0] + a[i * 3 + 1] * x[1] + a[i * 3 + 2] * x[2];
            assert!((row_sum - 1.0).abs() < 1e-10, "row {i}: {row_sum} != 1.0");
        }
    }

    #[test]
    fn rank_deficient() {
        // Rows 2 = row 0 + row 1, so rank = 2
        let mut data = vec![1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let qr = QrResult::factorize(&mut data, 2, 4);
        assert_eq!(qr.rank(1e-10), 2);
    }

    #[test]
    fn overdetermined_least_squares() {
        // 3 equations, 2 unknowns: x + y = 1, x - y = 0, x = 0.5
        // Solution: x = 0.5, y = 0.5
        let mut data = vec![1.0, 1.0, 1.0, -1.0, 1.0, 0.0];
        let qr = QrResult::factorize(&mut data, 3, 2);
        let x = qr.solve_least_squares(&[1.0, 0.0, 0.5]);
        assert!((x[0] - 0.5).abs() < 1e-10, "x[0] = {}", x[0]);
        assert!((x[1] - 0.5).abs() < 1e-10, "x[1] = {}", x[1]);
    }

    #[test]
    fn empty_matrix() {
        let mut data = vec![];
        let qr = QrResult::factorize(&mut data, 0, 0);
        assert_eq!(qr.rank(1e-10), 0);
    }

    #[test]
    fn single_element() {
        let mut data = vec![5.0];
        let qr = QrResult::factorize(&mut data, 1, 1);
        assert_eq!(qr.rank(1e-10), 1);
        let x = qr.solve_least_squares(&[10.0]);
        assert!((x[0] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn leading_magnitude_tracks_max_column_norm() {
        // First pivot takes the largest column: |R00| = 3 here.
        let mut data = vec![1.0, 0.0, 0.0, 3.0];
        let qr = QrResult::factorize(&mut data, 2, 2);
        assert!((qr.leading_magnitude() - 3.0).abs() < 1e-12);
    }

    #[test]
    fn leading_magnitude_empty_is_zero() {
        let mut data = vec![];
        let qr = QrResult::factorize(&mut data, 0, 0);
        assert_eq!(qr.leading_magnitude().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn rank_absolute_matches_relative_rank() {
        // Rank-deficient rows: row 1 = row 0, so rank 1 either way.
        let mut data = vec![1.0, 2.0, 2.0, 4.0];
        let qr = QrResult::factorize(&mut data, 2, 2);
        let r00 = qr.leading_magnitude();
        assert_eq!(qr.rank(1e-10), 1);
        assert_eq!(qr.rank_absolute(1e-10 * r00), 1);
        // Full-rank identity counts everything above any tiny threshold.
        let mut id = vec![1.0, 0.0, 0.0, 1.0];
        let qr_id = QrResult::factorize(&mut id, 2, 2);
        assert_eq!(qr_id.rank_absolute(1e-10), 2);
        assert_eq!(qr_id.rank_absolute(2.0), 0);
    }
}
