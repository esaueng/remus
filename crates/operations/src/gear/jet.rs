//! Interval bounds on a function and its first four derivatives, carried
//! through compositions and products: the fourth-derivative bounds the
//! certified Hermite fits of gear curves need.

/// Bounds on `|f|, |f'|, … , |f⁗|` over an interval.
pub(super) type Jet = [f64; 5];

/// Bounds for `f ∘ g` from bounds of `f`'s derivatives over the range of
/// `g` and bounds of `g`'s derivatives (Faà di Bruno, every term by its
/// absolute value).
pub(super) fn compose(f: Jet, g: Jet) -> Jet {
    let [_, g1, g2, g3, g4] = g;
    [
        f[0],
        f[1] * g1,
        f[2].mul_add(g1 * g1, f[1] * g2),
        f[3] * g1.powi(3) + 3.0 * f[2] * g1 * g2 + f[1] * g3,
        f[4] * g1.powi(4)
            + 6.0 * f[3] * g1 * g1 * g2
            + 3.0 * f[2] * g2 * g2
            + 4.0 * f[2] * g1 * g3
            + f[1] * g4,
    ]
}

/// Bounds for `a·b` (Leibniz, every term by its absolute value).
pub(super) fn product(a: Jet, b: Jet) -> Jet {
    const BINOMIAL: [[f64; 5]; 5] = [
        [1.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0, 0.0, 0.0],
        [1.0, 2.0, 1.0, 0.0, 0.0],
        [1.0, 3.0, 3.0, 1.0, 0.0],
        [1.0, 4.0, 6.0, 4.0, 1.0],
    ];
    let mut out = [0.0; 5];
    for (n, value) in out.iter_mut().enumerate() {
        for i in 0..=n {
            *value += BINOMIAL[n][i] * a[i] * b[n - i];
        }
    }
    out
}

/// Bounds of `√x` and its derivatives over `[lo, hi]`, `lo > 0`: each
/// derivative's magnitude falls with `x`, so it peaks at `lo`.
pub(super) fn sqrt_jet(lo: f64, hi: f64) -> Jet {
    let root = lo.sqrt();
    [
        hi.sqrt(),
        0.5 / root,
        0.25 / (lo * root),
        0.375 / (lo * lo * root),
        0.9375 / (lo * lo * lo * root),
    ]
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    /// A smooth function with known derivatives: the bounds of a
    /// composition and a product must cover the true values.
    #[test]
    fn jet_bounds_cover_exact_derivatives_of_a_composition() {
        // f(s) = √(4 − s²) on |s| ≤ 1: f⁗ at 0 is −3·4⁻³ᐟ²·… ; check by
        // central differences against the bound everywhere on a grid.
        let q: Jet = [4.0, 2.0, 2.0, 0.0, 0.0];
        let bound = compose(sqrt_jet(3.0, 4.0), q);
        let f = |s: f64| s.mul_add(-s, 4.0).sqrt();
        let h = 1e-2;
        for i in 0..=20 {
            let s = -0.9 + 1.8 * f64::from(i) / 20.0;
            let d4 = (f(s - 2.0 * h) - 4.0 * f(s - h) + 6.0 * f(s) - 4.0 * f(s + h)
                + f(s + 2.0 * h))
                / h.powi(4);
            assert!(d4.abs() <= bound[4] * 1.01, "{s}: {d4} > {}", bound[4]);
        }
        let squared = product(bound, bound);
        let g = |s: f64| s.mul_add(-s, 4.0);
        let d2 = (g(0.3 - h) - 2.0 * g(0.3) + g(0.3 + h)) / (h * h);
        assert!(d2.abs() <= squared[2]);
    }
}
