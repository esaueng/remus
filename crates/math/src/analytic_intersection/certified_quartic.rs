//! The real-root structure of a monic quartic, certified by sign.
//!
//! An iterative solver finds a root of multiplicity `m` only to about
//! `ε^(1/m)` relative, so a fixed threshold on the imaginary parts of its
//! iterates decides whether a multiple real root is kept at all. A triple
//! root of the line–torus quartic (a line along an asymptotic direction of
//! the saddle-shaped inner tube, which crosses the surface) came back with
//! imaginary parts far above any such cutoff and was dropped.
//!
//! Here no root is accepted or rejected by a threshold on a root. The
//! quartic's critical points split the line into stretches on which it is
//! monotone; its sign at the ends of each stretch decides what the stretch
//! holds. A value counts as signed only when it clears an evaluation-error
//! bound `noise(x)` that the caller derives from the coefficients' own
//! magnitudes, so the decision is invariant under scaling the problem. Between
//! two signed points with no unsigned point between them the quartic is
//! monotone: a sign change there is exactly one simple root. A run of unsigned
//! critical points is a contact the arithmetic cannot resolve; the signs on its
//! two sides still fix its parity exactly: unchanged is an even contact (a
//! touch), changed an odd one (a crossing, such as a triple root).

use crate::fma::FusedMulAdd;

/// Steps after which a bracketed refinement stops: past the 64 halvings that
/// shrink any finite bracket to adjacent doubles, with room for Newton steps.
const REFINE_STEPS: usize = 160;

/// `x⁴ + c[3] x³ + c[2] x² + c[1] x + c[0]`, with a bound on the error of its
/// computed value: at `x` the computed value differs from the exact quartic's
/// by at most `noise(x) = n[4] |x|⁴ + n[3] |x|³ + … + n[0]`.
pub(super) struct BoundedQuartic {
    /// Coefficients below the leading one, constant term first.
    pub(super) c: [f64; 4],
    /// Error-bound coefficients, constant term first; all non-negative.
    pub(super) n: [f64; 5],
}

/// What a root of a [`BoundedQuartic`] is, as far as its signs certify.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum QuarticContact {
    /// A simple root: the quartic is monotone on a bracket whose two ends
    /// carry certified opposite signs.
    Simple,
    /// An even contact: the quartic comes within its error bound of zero at
    /// one or more critical points and has the same certified sign on both
    /// sides. Touching (a double or quadruple root), two roots too close to
    /// separate, or a near miss; in every case an even number of crossings.
    Touch,
    /// An odd contact: as [`Self::Touch`], but the certified signs on its two
    /// sides differ. A triple root, or a simple root too close to a pair to
    /// separate; in every case an odd number of crossings.
    MultipleCrossing,
}

/// One root, or one unresolved contact, of a [`BoundedQuartic`].
#[derive(Debug, Clone, Copy)]
pub(super) struct QuarticRoot {
    /// The root, or the contact's representative point.
    pub(super) x: f64,
    /// What the signs certify about it.
    pub(super) contact: QuarticContact,
    /// Points with certified signs on either side that enclose it and no
    /// other root: a simple root's monotone bracket, or, for a contact, the
    /// edges of the stretch where the sign is not certified.
    pub(super) enclosure: (f64, f64),
}

impl BoundedQuartic {
    fn value(&self, x: f64) -> f64 {
        let [c0, c1, c2, c3] = self.c;
        (x + c3).fma(x, c2).fma(x, c1).fma(x, c0)
    }

    fn value_and_slope(&self, x: f64) -> (f64, f64) {
        let [_, c1, c2, c3] = self.c;
        let slope = 4.0f64.fma(x, 3.0 * c3).fma(x, 2.0 * c2).fma(x, c1);
        (self.value(x), slope)
    }

    fn slope_and_curvature(&self, x: f64) -> (f64, f64) {
        let [_, c1, c2, c3] = self.c;
        let slope = 4.0f64.fma(x, 3.0 * c3).fma(x, 2.0 * c2).fma(x, c1);
        let curvature = 12.0f64.fma(x, 6.0 * c3).fma(x, 2.0 * c2);
        (slope, curvature)
    }

    /// The bound on the computed value's error at `x`.
    pub(super) fn noise(&self, x: f64) -> f64 {
        let a = x.abs();
        let [n0, n1, n2, n3, n4] = self.n;
        n4.fma(a, n3).fma(a, n2).fma(a, n1).fma(a, n0)
    }

    /// The value's certified sign at `x`: `±1`, or `0` within the bound.
    fn sign(&self, x: f64) -> i8 {
        let (p, e) = (self.value(x), self.noise(x));
        if p > e {
            1
        } else if p < -e {
            -1
        } else {
            0
        }
    }

    /// The inflection points (real roots of the second derivative), in
    /// ascending order, each a simple root of a quadratic in closed form.
    fn inflections(&self) -> ([f64; 2], usize) {
        let [_, _, c2, c3] = self.c;
        // p''/12 = x² + 2h x + q.
        let (h, q) = (0.25 * c3, c2 / 6.0);
        let disc = h.fma(h, -q);
        if disc.is_nan() || disc <= 0.0 {
            return ([0.0; 2], 0);
        }
        // Cancellation-free: the larger-magnitude root directly, the other
        // from the product of the two.
        let big = -h - h.signum() * disc.sqrt();
        if big == 0.0 {
            return ([0.0; 2], 0);
        }
        let small = q / big;
        ([big.min(small), big.max(small)], 2)
    }

    /// An inflection point inside `(lo, hi)` where the quartic's sign is not
    /// certified, if any (the one nearest zero): where an odd contact sits
    /// when it is a triple root. The second derivative has a simple root
    /// there, so this places such a contact to about `ε` relative, where the
    /// quartic's own sign places it only to about `ε^(1/3)`.
    fn unsigned_inflection_within(&self, inflections: &[f64], lo: f64, hi: f64) -> Option<f64> {
        inflections
            .iter()
            .copied()
            .filter(|&w| w > lo && w < hi && self.sign(w) == 0)
            .min_by(|a, b| self.value(*a).abs().total_cmp(&self.value(*b).abs()))
    }

    /// The real critical points (roots of the derivative) inside
    /// `(-bound, bound)`, ascending. The derivative is monotone between the
    /// inflection points, so each stretch whose ends it has opposite signs
    /// on holds exactly one.
    fn critical_points(&self, bound: f64, inflections: &[f64]) -> ([f64; 3], usize) {
        let mut cuts = [-bound, 0.0, 0.0, bound];
        let mut cut_count = 1;
        for &x in inflections {
            if x > -bound && x < bound {
                cuts[cut_count] = x;
                cut_count += 1;
            }
        }
        cuts[cut_count] = bound;
        cut_count += 1;
        let mut out = [0.0; 3];
        let mut len = 0;
        let mut push = |x: f64| {
            if len < 3 && (len == 0 || out[len - 1] < x) {
                out[len] = x;
                len += 1;
            }
        };
        for k in 0..cut_count - 1 {
            let (lo, hi) = (cuts[k], cuts[k + 1]);
            let (s_lo, _) = self.slope_and_curvature(lo);
            let (s_hi, _) = self.slope_and_curvature(hi);
            if s_lo == 0.0 && k > 0 {
                push(lo);
            }
            if (s_lo < 0.0 && s_hi > 0.0) || (s_lo > 0.0 && s_hi < 0.0) {
                push(refine_bracketed(
                    |x| self.slope_and_curvature(x),
                    lo,
                    hi,
                    s_lo < 0.0,
                ));
            }
        }
        (out, len)
    }

    /// The point nearest `to` on the way from `from`, whose certified sign is
    /// `sign`, to `to` at which the sign is still certified (`from` itself
    /// when nothing nearer is).
    fn certified_edge(&self, from: f64, to: f64, sign: i8) -> f64 {
        let (mut good, mut bad) = (from, to);
        for _ in 0..REFINE_STEPS {
            let mid = f64::midpoint(good, bad);
            // Adjacent doubles: nothing lies between them.
            if !(good.min(bad) < mid && mid < good.max(bad)) {
                break;
            }
            if self.sign(mid) == sign {
                good = mid;
            } else {
                bad = mid;
            }
        }
        good
    }

    /// The stretch around a root where the quartic's sign is not certified:
    /// every point the root may be. For a contact this is its enclosure.
    pub(super) fn uncertain_window(&self, root: &QuarticRoot) -> (f64, f64) {
        let (a, b) = root.enclosure;
        if root.contact != QuarticContact::Simple || self.sign(root.x) != 0 {
            return if root.contact == QuarticContact::Simple {
                (root.x, root.x)
            } else {
                (a, b)
            };
        }
        (
            self.certified_edge(a, root.x, self.sign(a)),
            self.certified_edge(b, root.x, self.sign(b)),
        )
    }

    /// Every real root, ascending, each simple root refined to full precision
    /// and each unresolved contact reported once; `None` when the
    /// coefficients or the bound are not finite.
    pub(super) fn roots(&self) -> Option<Vec<QuarticRoot>> {
        if !self.c.iter().chain(&self.n).all(|v| v.is_finite()) {
            return None;
        }
        let [c0, c1, c2, c3] = self.c;
        // Fujiwara's bound on the magnitude of every root (and, by
        // Gauss–Lucas, of every critical point).
        let mut bound = 2.0
            * c3.abs()
                .max(c2.abs().sqrt())
                .max(c1.abs().cbrt())
                .max((0.5 * c0).abs().sqrt().sqrt());
        if bound.is_nan() || bound <= 0.0 {
            bound = 1.0;
        }
        let mut grown = 0;
        while self.sign(-bound) != 1 || self.sign(bound) != 1 {
            bound *= 2.0;
            grown += 1;
            if grown > 64 || !bound.is_finite() {
                return None;
            }
        }
        let (inflections, inflection_count) = self.inflections();
        let inflections = &inflections[..inflection_count];
        let (crit, crit_len) = self.critical_points(bound, inflections);
        if crit_len == 0 {
            return None;
        }
        // Evaluation points with their certified signs; both ends positive.
        let mut points = [(0.0, 0_i8); 5];
        points[0] = (-bound, 1);
        for (k, &x) in crit[..crit_len].iter().enumerate() {
            points[k + 1] = (x, self.sign(x));
        }
        let len = crit_len + 2;
        points[len - 1] = (bound, 1);
        let points = &points[..len];

        let mut out = Vec::new();
        let mut i = 0;
        while i + 1 < len {
            let mut j = i + 1;
            while points[j].1 == 0 {
                j += 1;
            }
            let ((xi, si), (xj, sj)) = (points[i], points[j]);
            if j == i + 1 {
                if si != sj {
                    let x = refine_bracketed(|x| self.value_and_slope(x), xi, xj, si < 0);
                    let mut root = QuarticRoot {
                        x,
                        contact: QuarticContact::Simple,
                        enclosure: (xi, xj),
                    };
                    // A triple root whose derivative's two nearby roots the
                    // rounding merged away leaves no critical point next to
                    // it, only a stretch where the slope nearly vanishes
                    // too. It still changes sign once, but it is not
                    // resolved: report it as the odd contact it is.
                    if let Some(w) = self.unsigned_inflection_within(inflections, xi, xj) {
                        let (lo, hi) = self.uncertain_window(&root);
                        if lo <= w && w <= hi {
                            root = QuarticRoot {
                                x: w,
                                contact: QuarticContact::MultipleCrossing,
                                enclosure: (lo, hi),
                            };
                        }
                    }
                    out.push(root);
                }
            } else {
                let lo = self.certified_edge(xi, points[i + 1].0, si);
                let hi = self.certified_edge(xj, points[j - 1].0, sj);
                let (x, contact) = if si == sj {
                    // The critical point nearest to crossing over.
                    let side = f64::from(si);
                    let x = points[i + 1..j]
                        .iter()
                        .map(|&(x, _)| x)
                        .min_by(|&a, &b| (side * self.value(a)).total_cmp(&(side * self.value(b))))
                        .unwrap_or(xi);
                    (x, QuarticContact::Touch)
                } else {
                    let x = self
                        .unsigned_inflection_within(inflections, lo, hi)
                        .unwrap_or_else(|| {
                            refine_bracketed(|x| self.value_and_slope(x), lo, hi, si < 0)
                        });
                    (x, QuarticContact::MultipleCrossing)
                };
                out.push(QuarticRoot {
                    x,
                    contact,
                    enclosure: (lo, hi),
                });
            }
            i = j;
        }
        Some(out)
    }
}

/// The root in `(lo, hi)` of a function monotone there, negative at `lo`
/// when `lo_negative` and of the opposite sign at `hi`: Newton's method kept
/// inside the bracket, bisecting whenever a step would leave it or would not
/// halve the step before last, until a step is down to rounding or the
/// bracket to adjacent doubles.
fn refine_bracketed(
    f: impl Fn(f64) -> (f64, f64),
    mut lo: f64,
    mut hi: f64,
    lo_negative: bool,
) -> f64 {
    let mut x = f64::midpoint(lo, hi);
    let mut step_before = hi - lo;
    let mut step = step_before;
    for _ in 0..REFINE_STEPS {
        let (fx, dfx) = f(x);
        if fx == 0.0 {
            return x;
        }
        if (fx < 0.0) == lo_negative {
            lo = x;
        } else {
            hi = x;
        }
        let newton = x - fx / dfx;
        // Converged: the step is down to rounding (it may land a rounding
        // error outside the bracket just moved to `x`).
        if (newton - x).abs() <= 2.0 * f64::EPSILON * x.abs() {
            return if newton >= lo && newton <= hi {
                newton
            } else {
                x
            };
        }
        let stalls = (2.0 * fx).abs() > (step_before * dfx).abs();
        step_before = step;
        if newton > lo && newton < hi && !stalls {
            step = (newton - x).abs();
            x = newton;
        } else {
            let mid = f64::midpoint(lo, hi);
            if mid <= lo || mid >= hi {
                return x;
            }
            step = 0.5 * (hi - lo);
            x = mid;
        }
    }
    x
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::{BoundedQuartic, QuarticContact};

    /// The monic quartic with roots `r`, its error bound a plain relative one.
    fn from_roots(r: [f64; 4]) -> BoundedQuartic {
        let [a, b, c, d] = r;
        let e1 = a + b + c + d;
        let e2 = a * b + a * c + a * d + b * c + b * d + c * d;
        let e3 = a * b * c + a * b * d + a * c * d + b * c * d;
        let e4 = a * b * c * d;
        let m = r.iter().map(|x| x.abs()).fold(1.0_f64, f64::max);
        let noise = 64.0 * f64::EPSILON;
        BoundedQuartic {
            c: [e4, -e3, e2, -e1],
            n: [
                noise * m.powi(4),
                noise * m.powi(3),
                noise * m * m,
                noise * m,
                noise,
            ],
        }
    }

    fn kinds(q: &BoundedQuartic) -> Vec<(f64, QuarticContact)> {
        q.roots()
            .unwrap()
            .iter()
            .map(|r| (r.x, r.contact))
            .collect()
    }

    #[test]
    fn simple_roots_are_refined_to_full_precision() {
        let q = from_roots([-3.0, -0.5, 0.25, 2.0]);
        let got = kinds(&q);
        assert_eq!(got.len(), 4);
        for ((x, kind), want) in got.iter().zip([-3.0, -0.5, 0.25, 2.0]) {
            assert_eq!(*kind, QuarticContact::Simple);
            assert!(
                (x - want).abs() <= 4.0 * f64::EPSILON * 3.0,
                "{x} vs {want}"
            );
        }
    }

    #[test]
    fn multiplicities_are_classified_by_sign() {
        use QuarticContact::{MultipleCrossing, Simple, Touch};
        let cases: [([f64; 4], Vec<QuarticContact>); 5] = [
            ([1.0, 1.0, -2.0, 3.0], vec![Simple, Touch, Simple]),
            ([0.7, 0.7, 0.7, -1.0], vec![Simple, MultipleCrossing]),
            ([0.3, 0.3, 0.3, 0.3], vec![Touch]),
            ([-1.0, -1.0, 2.0, 2.0], vec![Touch, Touch]),
            // Two simple roots 1e-12 apart: too close to separate, an even
            // contact either way.
            ([1.0, 1.0 + 1e-12, 4.0, 5.0], vec![Touch, Simple, Simple]),
        ];
        for (roots, want) in cases {
            let got: Vec<_> = kinds(&from_roots(roots)).into_iter().map(|g| g.1).collect();
            assert_eq!(got, want, "{roots:?}");
        }
    }

    #[test]
    fn a_triple_root_is_placed_at_its_inflection() {
        for scale in [1e-3, 1.0, 1e6] {
            let q = from_roots([0.2 * scale, 0.2 * scale, 0.2 * scale, -2.0 * scale]);
            let got = kinds(&q);
            let triple = got
                .iter()
                .find(|g| g.1 == QuarticContact::MultipleCrossing)
                .unwrap();
            assert!((triple.0 - 0.2 * scale).abs() <= 1e-12 * scale, "{got:?}");
        }
    }
}
