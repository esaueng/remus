//! Compact band storage for the NURBS interpolation collocation solve.
//!
//! `solve_interpolation` in [`super::fitting`] assembles `N[i][j] =
//! B_{j,p}(t_i)` and solves `N * P = Q` for the three coordinate right-hand
//! sides with one factorization. The matrix is structurally banded, but the
//! historical implementation allocated it dense (`n x n`) even though the
//! elimination only ever touches a narrow diagonal band.
//!
//! # Proven active band (qualified domain)
//!
//! Interior interpolation knots use NURBS-Book averaging (`eq 9.69`):
//! `U_{p+j} = mean(t_j..t_{j+p-1})` with clamped ends. For strictly
//! increasing `params` (`t_0 = 0 < t_1 < .. < t_{n-1} = 1`, enforced by
//! `interpolate_with_params` validation and by the uniform fallback in
//! `chord_length_params` when all points coincide):
//!
//! * `U_i <= t_i` for every `i`, so `span_i >= i`. For `i <= p` the knot is
//!   the clamped `0`; otherwise `U_i` averages `p` parameters strictly below
//!   `t_i`.
//! * `U_{i+p+1} > t_i` whenever the index exists, so `span_i <= i + p`. The
//!   knot averages `p` parameters strictly above `t_i` (or is the clamped
//!   `1` above any `t_i < 1`).
//!
//! Hence `span_i in [i, i+p]`, and row `i`'s only structural nonzeros,
//! columns `span_i-p ..= span_i`, satisfy `|i-j| <= p`. Both one-sided
//! bandwidths are at most `p`.
//!
//! # Pivot-induced fill
//!
//! Partial pivoting searches rows `k..=k+kl` (`kl = max(p,1)`, matching the
//! historical `band.max(1)`). Eliminating with that choice spreads fill to
//! `ku = kl + p = 2*kl` above the diagonal (standard band-LU-with-pivoting
//! bound: `U` gains the lower bandwidth on top of the initial upper
//! bandwidth). At step `k` every participating row (`k..=k+kl`) already has
//! zeros in columns `< k` and nonzeros only in `k..=k+ku`, so swapping rows
//! `k` and `m <= k+kl` only needs columns `k..=k+ku` exchanged. Rows below
//! `k+kl` are untouched so far and still carry only their initial band.
//!
//! # Outside the qualified domain
//!
//! Chord-length parameters can repeat when input points coincide, so
//! `solve_interpolation` does NOT assume the bound. It computes spans first
//! and checks `span_i in [i, i+p]` for every row. Any violation (repeated
//! parameters, floating-point averaging ties, non-finite knots) falls back
//! to the historical dense allocation with the verbatim band-limited
//! elimination, preserving bit-identical singularity decisions and results.
//! Small systems (`n <= kl+ku+1`, where dense is no larger) also use the
//! dense path, so tiny inputs see no allocation or indexing regression.

use crate::MathError;

/// Compact band matrix for the interpolation solve.
///
/// Stores rows `i` over columns `i-kl ..= i+ku` (clipped to `0..n`) in a
/// single flat allocation of `n * (kl+ku+1)` zeros. Out-of-band entries read
/// as `0.0`; writing a nonzero outside the band is a qualification failure
/// handled by the caller falling back to dense.
#[derive(Debug)]
pub struct BandMatrix {
    n: usize,
    kl: usize,
    ku: usize,
    w: usize,
    data: Vec<f64>,
}

impl BandMatrix {
    /// Allocate an `n x n` zero matrix with lower bandwidth `kl` and upper
    /// bandwidth `ku`.
    pub fn new(n: usize, kl: usize, ku: usize) -> Self {
        let w = kl.saturating_add(ku).saturating_add(1);
        Self {
            n,
            kl,
            ku,
            w,
            data: vec![0.0; n.saturating_mul(w)],
        }
    }

    /// Number of stored `f64` values (`n * (kl+ku+1)`).
    #[allow(dead_code)]
    pub fn stored_len(&self) -> usize {
        self.data.len()
    }

    /// Lower bandwidth.
    #[allow(dead_code)]
    pub fn lower(&self) -> usize {
        self.kl
    }

    /// Upper bandwidth (includes pivot fill).
    #[allow(dead_code)]
    pub fn upper(&self) -> usize {
        self.ku
    }

    #[inline]
    fn offset(&self, row: usize, col: usize) -> Option<usize> {
        // All-`usize` band test, no signed casts: in-band iff
        // `row-kl <= col <= row+ku`.
        if col.saturating_add(self.kl) < row {
            return None;
        }
        if col > row.saturating_add(self.ku) {
            return None;
        }
        // Columns outside `0..n` never occur for qualified assembly
        // (`span-p >= 0`, `span < n`), but guard anyway.
        if col >= self.n {
            return None;
        }
        let off = col.saturating_add(self.kl).saturating_sub(row);
        Some(row.saturating_mul(self.w).saturating_add(off))
    }

    /// Read entry `(row, col)`; out-of-band reads as `0.0`.
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        match self.offset(row, col) {
            Some(idx) => self.data[idx],
            None => 0.0,
        }
    }

    /// Write entry `(row, col)`.
    ///
    /// Returns `false` when a nonzero value falls outside the stored band;
    /// the caller must fall back to dense in that case to preserve the
    /// historical numerical contract. Exact zeros outside the band are fine.
    pub fn set(&mut self, row: usize, col: usize, value: f64) -> bool {
        match self.offset(row, col) {
            Some(idx) => {
                self.data[idx] = value;
                true
            }
            None => value == 0.0,
        }
    }

    /// Whether `(row, col)` lies inside the stored band.
    #[inline]
    pub fn contains(&self, row: usize, col: usize) -> bool {
        self.offset(row, col).is_some()
    }

    /// Unchecked read for indices the caller has proven in-band.
    #[inline]
    fn get_inband(&self, row: usize, col: usize) -> f64 {
        debug_assert!(self.contains(row, col), "out-of-band read");
        let off = col.saturating_add(self.kl).saturating_sub(row);
        self.data[row.saturating_mul(self.w).saturating_add(off)]
    }

    /// Unchecked write for indices the caller has proven in-band.
    #[inline]
    fn set_inband(&mut self, row: usize, col: usize, value: f64) {
        debug_assert!(self.contains(row, col), "out-of-band write");
        let off = col.saturating_add(self.kl).saturating_sub(row);
        let idx = row.saturating_mul(self.w).saturating_add(off);
        self.data[idx] = value;
    }

    /// Exchange the active segments of rows `a` and `b` at elimination step
    /// `k`: columns `k..=min(n-1, k+ku)`.
    ///
    /// Both rows have zeros in columns `< k` (already eliminated) and no
    /// nonzeros past `k+ku` (band invariant), so this preserves every
    /// coefficient the historical dense row swap preserved.
    fn swap_active_segments(&mut self, a: usize, b: usize, k: usize) {
        if a == b || self.n == 0 {
            return;
        }
        let end = (k.saturating_add(self.ku)).min(self.n.saturating_sub(1));
        for col in k..=end {
            // Both columns are in-band for both rows here:
            // `col >= k >= a-kl` (since `a <= k+kl`) and `col <= k+ku <= a+ku`
            // (since `a >= k`), likewise for `b`.
            debug_assert!(self.contains(a, col) && self.contains(b, col));
            let (Some(ia), Some(ib)) = (self.offset(a, col), self.offset(b, col)) else {
                continue;
            };
            self.data.swap(ia, ib);
        }
    }
}

/// Check the proven structural band: every span must satisfy
/// `i <= span_i <= i + p`.
///
/// This is exactly `|i-j| <= p` for all structural columns
/// `j in span_i-p..=span_i`. Any failure means the caller must use the dense
/// fallback; the band-limited elimination would otherwise skip load-bearing
/// coefficients (and the historical dense path did the same band-limited
/// arithmetic, so the fallback reproduces it verbatim).
pub fn collocation_band_ok(spans: &[usize], degree: usize) -> bool {
    for (i, &span) in spans.iter().enumerate() {
        if span < i {
            return false;
        }
        if span > i.saturating_add(degree) {
            return false;
        }
    }
    true
}

/// Banded Gaussian elimination with partial pivoting confined to the band,
/// solving several right-hand sides against one factorization.
///
/// Arithmetic, pivot search window, fill window (`2*kl` above the diagonal),
/// singularity threshold (`1e-15`), and the `factor == 0.0` skip match the
/// historical dense `banded_gauss_solve_multi` exactly; only storage differs.
pub fn banded_solve_multi(
    band: &mut BandMatrix,
    rhs: &mut [Vec<f64>],
    bw: usize,
) -> Result<(), MathError> {
    let n = band.n;
    if n == 0 {
        return Ok(());
    }
    // `bw` mirrors the historical `band.max(1)` lower bandwidth; the stored
    // upper bandwidth already includes pivot fill (`ku == 2*kl`).
    debug_assert_eq!(bw, band.kl, "band lower must match elimination bandwidth");
    let kl = bw;
    let ku = band.ku;
    for k in 0..n {
        let row_end = (k.saturating_add(kl)).min(n.saturating_sub(1));
        let mut max_row = k;
        let mut max_val = band.get_inband(k, k).abs();
        for i in (k + 1)..=row_end {
            let v = band.get(i, k).abs();
            if v > max_val {
                max_val = v;
                max_row = i;
            }
        }
        if max_val < 1e-15 {
            return Err(MathError::SingularMatrix);
        }
        if max_row != k {
            band.swap_active_segments(k, max_row, k);
            for r in rhs.iter_mut() {
                r.swap(k, max_row);
            }
        }
        let col_end = (k.saturating_add(ku)).min(n.saturating_sub(1));
        for i in (k + 1)..=row_end {
            let factor = band.get(i, k) / band.get_inband(k, k);
            if factor == 0.0 {
                continue;
            }
            band.set_inband(i, k, 0.0);
            for j in (k + 1)..=col_end {
                let updated = band.get(i, j) - factor * band.get(k, j);
                // `j` is in-band for row `i` here: `j <= k+ku <= i+ku`
                // (since `i >= k+1`, `k+ku < i+ku`) and `j >= k+1 > i-kl`
                // (since `i <= k+kl`). Use the checked accessor's in-band
                // path via offset to keep release behavior total.
                if let Some(idx) = band.offset(i, j) {
                    band.data[idx] = updated;
                } else {
                    debug_assert!(updated == 0.0, "fill escaped the stored band");
                    if updated != 0.0 {
                        // Should be unreachable when `collocation_band_ok`
                        // held at assembly; treat as singular rather than
                        // silently dropping a coefficient.
                        return Err(MathError::SingularMatrix);
                    }
                }
            }
            for r in rhs.iter_mut() {
                r[i] -= factor * r[k];
            }
        }
    }
    for k in (0..n).rev() {
        let col_end = (k.saturating_add(ku)).min(n.saturating_sub(1));
        for r in rhs.iter_mut() {
            let mut sum = r[k];
            for j in (k + 1)..=col_end {
                sum -= band.get(k, j) * r[j];
            }
            r[k] = sum / band.get_inband(k, k);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn band_matrix_round_trips_inband_values() {
        let mut band = BandMatrix::new(6, 2, 4);
        assert!(band.set(2, 1, 0.5));
        assert!(band.set(2, 5, -1.25));
        assert!((band.get(2, 1) - 0.5).abs() < 1e-15);
        assert!((band.get(2, 5) + 1.25).abs() < 1e-15);
        // Out-of-band reads as zero; nonzero writes refuse.
        assert!(band.get(0, 5).abs() < 1e-15);
        assert!(!band.set(0, 5, 1.0));
        assert!(band.set(0, 5, 0.0));
    }

    #[test]
    fn band_ok_accepts_proven_spans() {
        // n=5, p=3 example from the averaging construction.
        assert!(collocation_band_ok(&[3, 3, 4, 4, 4], 3));
    }

    #[test]
    #[allow(clippy::unreadable_literal)]
    fn band_ok_rejects_repeated_param_drift() {
        // Span 8 at row 4 with p=3 (observed on duplicate-point fuzz):
        // structural columns 5..=8 escape |i-j| <= 3.
        assert!(!collocation_band_ok(&[3, 3, 3, 3, 8, 8, 8, 8, 9, 9], 3));
    }

    #[test]
    fn swap_preserves_active_coefficients() {
        let mut band = BandMatrix::new(5, 1, 2);
        // Row 1: [k=1] 2.0 at (1,1), 3.0 at (1,2), 4.0 at (1,3)
        assert!(band.set(1, 1, 2.0));
        assert!(band.set(1, 2, 3.0));
        assert!(band.set(1, 3, 4.0));
        // Row 2: 0.5 at (2,1), 6.0 at (2,2), 7.0 at (2,3)
        assert!(band.set(2, 1, 0.5));
        assert!(band.set(2, 2, 6.0));
        assert!(band.set(2, 3, 7.0));
        band.swap_active_segments(1, 2, 1);
        assert!((band.get(1, 1) - 0.5).abs() < 1e-15);
        assert!((band.get(1, 2) - 6.0).abs() < 1e-15);
        assert!((band.get(2, 1) - 2.0).abs() < 1e-15);
        assert!((band.get(2, 2) - 3.0).abs() < 1e-15);
    }
}
