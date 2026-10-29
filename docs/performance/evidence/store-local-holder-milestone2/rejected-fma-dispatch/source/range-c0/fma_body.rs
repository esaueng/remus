//! Correctly rounded fused multiply-add that stays fast on `wasm32`.
//!
//! `f64::mul_add` rounds `a·b + c` once. Native targets do that in one
//! instruction; WebAssembly has no fused multiply-add, so there it is a call
//! into `compiler_builtins`' integer-arithmetic `fma` (with 128-bit multiplies
//! through `__multi3`). In the kernel's WASM build that fallback was the
//! largest single cost of ray classification (over half of the samples of
//! `solidEdgeRelations` on the hammer-holder fixture).
//!
//! [`fma`] returns exactly the value `f64::mul_add` returns, on every target:
//!
//! - on everything except `wasm32` it *is* `f64::mul_add`;
//! - on `wasm32`, operands in a range where no intermediate can underflow or
//!   overflow go through `emulated_fma`: Boldo and Melquiond's emulation from
//!   error-free transformations and one rounding to odd ("Emulation of FMA
//!   and correctly rounded sums: proved algorithms using rounding to odd",
//!   IEEE Transactions on Computers, 2008). A zero factor with finite
//!   operands is an exact signed-zero product, so `a * b + c` is already the
//!   fused result. Everything else (subnormal, huge, infinite or NaN factors,
//!   huge or non-finite addend) takes `f64::mul_add`.
//!
//! The correctly rounded result is unique, so a correct emulation is
//! bit-identical to `f64::mul_add`. The tests check that against the native
//! `mul_add` on random, cancelling, tie-producing and range-boundary inputs:
//! the emulation is ordinary IEEE-754 binary64 arithmetic in round-to-nearest,
//! which WebAssembly specifies exactly, and Rust never contracts `a * b + c`.

/// Smallest factor magnitude the emulated path accepts: `2^-450`.
///
/// With both factors at least this large the product is at least `2^-900`,
/// so its exact low half and every Dekker partial product stay far above
/// the subnormal range (`2^-1022`) and are computed exactly.
const FACTOR_MIN: f64 = f64::from_bits((1023 - 450) << 52);
/// Largest factor magnitude the emulated path accepts: `2^450`.
///
/// Keeps the Veltkamp split (`134217729 · a`) and the product far below
/// overflow.
const FACTOR_MAX: f64 = f64::from_bits((1023 + 450) << 52);
/// Largest addend magnitude the emulated path accepts: `2^900`.
///
/// Every intermediate sum then stays below `2^902`.
const ADDEND_MAX: f64 = f64::from_bits((1023 + 900) << 52);

/// `a·b + c` rounded once, bit-identical to `a.mul_add(b, c)`.
///
/// See the [module documentation](self) for how `wasm32` avoids the slow
/// software `fma` without changing a single result bit. On `wasm32` this is
/// one call to an out-of-line emulation, like the `mul_add` call it replaces,
/// so the module does not grow (inlining the emulation at every call site
/// added ~0.4 MB to the optimized kernel, past the 10 MiB raw budget it had
/// then).
#[inline]
#[must_use]
pub fn fma(a: f64, b: f64, c: f64) -> f64 {
    #[cfg(target_arch = "wasm32")]
    {
        wasm_fma(a, b, c)
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        a.mul_add(b, c)
    }
}

/// The out-of-line `wasm32` body of [`fma`]: the emulation where it is
/// exact, `f64::mul_add` everywhere else. Compiled natively for the tests,
/// which compare it with the native `mul_add`.
#[inline(never)]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
#[allow(clippy::suboptimal_flops)]
fn wasm_fma(a: f64, b: f64, c: f64) -> f64 {
    if in_emulated_range(a, b, c) {
        // The range guarantees finite nonzero factors and a finite, normal
        // product. Adding either signed zero leaves its rounded value unchanged.
        if c == 0.0 {
            a * b
        } else {
            emulated_fma(a, b, c)
        }
    } else if is_zero_product(a, b, c) {
        // `a·b` is an exact signed zero, so the one rounding is the sum's,
        // and IEEE 754 gives a zero `fma` result the sign rule of that sum.
        a * b + c
    } else {
        a.mul_add(b, c)
    }
}

/// Whether one factor is zero and every operand finite (axis-aligned
/// vectors make zero factors common). Then `a * b + c` is exactly the fused
/// result.
#[inline]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
fn is_zero_product(a: f64, b: f64, c: f64) -> bool {
    ((a == 0.0 && b.is_finite()) || (b == 0.0 && a.is_finite())) && c.is_finite()
}

/// Method form of [`fma`], so `x.mul_add(y, z)` call chains keep their shape.
pub trait FusedMulAdd {
    /// `self·b + c` rounded once, bit-identical to `f64::mul_add`.
    #[must_use]
    fn fma(self, b: Self, c: Self) -> Self;
}

impl FusedMulAdd for f64 {
    #[inline]
    fn fma(self, b: Self, c: Self) -> Self {
        fma(self, b, c)
    }
}

/// Whether `emulated_fma` is exact for these operands: finite factors with
/// magnitudes in `[2^-450, 2^450]` and a finite addend below `2^900` (zero and
/// subnormal addends included). Non-finite values are excluded.
#[inline]
#[must_use]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
fn in_emulated_range(a: f64, b: f64, c: f64) -> bool {
    const FACTORS: std::ops::RangeInclusive<f64> = FACTOR_MIN..=FACTOR_MAX;
    FACTORS.contains(&a.abs()) && FACTORS.contains(&b.abs()) && c.abs() <= ADDEND_MAX
}

/// Knuth's TwoSum: `s = RN(a + b)` and the exact error `e = a + b − s`.
#[inline]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let b_virtual = s - a;
    let a_virtual = s - b_virtual;
    (s, (a - a_virtual) + (b - b_virtual))
}

/// Veltkamp split of `a` into two 26-bit halves, `a = hi + lo` exactly.
#[inline]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
fn split(a: f64) -> (f64, f64) {
    const SPLITTER: f64 = 134_217_729.0; // 2^27 + 1
    let t = SPLITTER * a;
    let hi = t - (t - a);
    (hi, a - hi)
}

/// Dekker's product: `p = RN(a·b)` and the exact error `e = a·b − p`
/// (exact while no partial product underflows). The expression must stay
/// unfused: each step is exact only as two separately rounded operations.
#[inline]
#[allow(clippy::suboptimal_flops)]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
fn two_prod(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    let (ah, al) = split(a);
    let (bh, bl) = split(b);
    let e = (((ah * bh - p) + ah * bl) + al * bh) + al * bl;
    (p, e)
}

/// `RO(a + b)`: the sum rounded to odd — exact when representable, otherwise
/// whichever neighbour of the exact sum has an odd last significand bit.
#[inline]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
fn round_to_odd_sum(a: f64, b: f64) -> f64 {
    let (s, e) = two_sum(a, b);
    let bits = s.to_bits();
    if e == 0.0 || bits & 1 == 1 {
        return s;
    }
    // Inexact with an even significand: the exact sum lies strictly between
    // `s` and its neighbour in the direction of `e`, and that neighbour is
    // odd. `s` is nonzero here (a rounded-to-zero sum is exact), so stepping
    // the bit pattern moves one ulp away from or towards zero.
    if (e > 0.0) == (s > 0.0) {
        f64::from_bits(bits + 1)
    } else {
        f64::from_bits(bits - 1)
    }
}

/// Boldo–Melquiond emulated FMA: `RN(a·b + c)` from `RN` arithmetic and one
/// rounding to odd. Exact for operands accepted by `in_emulated_range`.
#[inline]
#[must_use]
#[cfg_attr(not(any(target_arch = "wasm32", test)), allow(dead_code))]
fn emulated_fma(a: f64, b: f64, c: f64) -> f64 {
    let (uh, ul) = two_prod(a, b);
    let (th, tl) = two_sum(c, uh);
    let v = round_to_odd_sum(tl, ul);
    th + v
}

#[cfg(test)]
mod tests {
    //! The emulation against the native `f64::mul_add` (a single hardware
    //! instruction on the development and CI hosts' FMA-capable CPUs, or the
    //! platform's correctly rounded `fma`): every case must agree bit for bit.
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_possible_wrap)]

    use super::*;

    /// SplitMix64: a small deterministic generator, so failures reproduce.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = self.0;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
        fn range(&mut self, lo: i64, hi: i64) -> i64 {
            lo + self.below((hi - lo + 1) as u64) as i64
        }
        fn sign(&mut self) -> f64 {
            if self.next() & 1 == 0 { 1.0 } else { -1.0 }
        }
        /// A normal number `±1.m × 2^exp` whose significand keeps only its
        /// `bits` leading fraction bits (few bits make exact products, exact
        /// sums and ties likely).
        fn normal(&mut self, exp: i64, bits: u32) -> f64 {
            let fraction = self.next() & ((1_u64 << 52) - 1);
            let keep = if bits >= 52 {
                fraction
            } else {
                fraction & !((1_u64 << (52 - bits)) - 1)
            };
            let biased = u64::try_from(exp + 1023).unwrap();
            self.sign() * f64::from_bits((biased << 52) | keep)
        }
        fn normal_in(&mut self, lo: i64, hi: i64, bits: u32) -> f64 {
            let exp = self.range(lo, hi);
            self.normal(exp, bits)
        }
        fn fraction_bits(&mut self) -> u32 {
            match self.below(4) {
                0 => 52,
                1 => u32::try_from(self.below(53)).unwrap(),
                2 => u32::try_from(self.below(27)).unwrap(),
                _ => u32::try_from(self.below(6)).unwrap(),
            }
        }
    }

    fn exponent(x: f64) -> i64 {
        ((x.to_bits() >> 52) & 0x7ff) as i64 - 1023
    }

    fn assert_same(a: f64, b: f64, c: f64) {
        let expected = a.mul_add(b, c);
        let got = wasm_fma(a, b, c);
        assert!(
            got.to_bits() == expected.to_bits() || (got.is_nan() && expected.is_nan()),
            "fma({a:e}, {b:e}, {c:e}) [{:#x}, {:#x}, {:#x}]: emulated {got:e} ({:#x}), \
             mul_add {expected:e} ({:#x})",
            a.to_bits(),
            b.to_bits(),
            c.to_bits(),
            got.to_bits(),
            expected.to_bits(),
        );
    }

    /// An addend whose magnitude is placed relative to the product: from far
    /// below its last bit to far above its leading bit, with `uh`-cancelling
    /// and tie-making shapes mixed in.
    fn addend(rng: &mut Rng, a: f64, b: f64) -> f64 {
        let p = a * b;
        let ep = exponent(p);
        match rng.below(8) {
            0 => 0.0,
            1 => -p,
            // Cancel the rounded product and leave a few-ulp remainder.
            2 => {
                let ulps = rng.range(-4, 4) as f64;
                let ulp = f64::from_bits(u64::try_from(ep - 52 + 1023).unwrap() << 52);
                -p + ulps * ulp
            }
            // Half-ulp multiples around the product: exact midpoints.
            3 => {
                let half = f64::from_bits(u64::try_from(ep - 53 + 1023).unwrap() << 52);
                rng.sign() * half * (2 * rng.below(8) + 1) as f64
            }
            _ => {
                let exp = (ep + rng.range(-130, 70)).clamp(-1000, 899);
                let bits = rng.fraction_bits();
                rng.normal(exp, bits)
            }
        }
    }

    #[test]
    fn emulation_matches_native_fma_on_random_and_adversarial_operands() {
        let mut rng = Rng(0x05EE_D0F0_FA11_CA5E);
        let mut eligible = 0_u32;
        for _ in 0..1_500_000 {
            // Mostly moderate exponents (where kernel values live), with the
            // full accepted range and its edges mixed in.
            let span = if rng.below(4) == 0 { 450 } else { 40 };
            let (bits_a, bits_b) = (rng.fraction_bits(), rng.fraction_bits());
            let mut a = rng.normal_in(-span, span, bits_a);
            let b = rng.normal_in(-span, span, bits_b);
            let c = addend(&mut rng, a, b);
            if rng.below(64) == 0 {
                a = rng.sign() * 0.0;
            }
            if in_emulated_range(a, b, c) {
                eligible += 1;
                // Check the arithmetic body independently of dispatch selection.
                assert_eq!(emulated_fma(a, b, c).to_bits(), a.mul_add(b, c).to_bits());
            }
            assert_same(a, b, c);
        }
        // Non-vacuous coverage of the emulation's accepted range.
        assert!(eligible > 1_350_000, "only {eligible} cases were eligible");
    }

    #[test]
    fn emulation_handles_exact_ties_both_ways() {
        // Products of two 26-bit significands are exact; adding an odd
        // multiple of half their last place lands exactly on a midpoint, which
        // must round to even.
        let mut rng = Rng(0x7135_7135);
        for _ in 0..300_000 {
            let a = rng.normal_in(-30, 30, 25);
            let b = rng.normal_in(-30, 30, 25);
            let p = a * b;
            assert_eq!(
                a.mul_add(b, -p).abs().to_bits(),
                0,
                "26-bit product must be exact"
            );
            let ep = exponent(p);
            let shift = rng.range(53, 60);
            let unit = f64::from_bits(u64::try_from(ep - shift + 1023).unwrap() << 52);
            let c = rng.sign() * unit * (2 * rng.below(1 << 10) + 1) as f64;
            assert_same(a, b, c);
            assert_same(a, b, -c);
        }
    }

    #[test]
    fn emulation_matches_at_the_range_boundaries_and_falls_back_outside() {
        let edges = [
            FACTOR_MIN,
            f64::from_bits(FACTOR_MIN.to_bits() + 1),
            f64::from_bits(FACTOR_MAX.to_bits() - 1),
            FACTOR_MAX,
            1.0,
            1.0 + f64::EPSILON,
            2.0 - f64::EPSILON,
        ];
        let addends = [
            0.0,
            -0.0,
            f64::MIN_POSITIVE,
            f64::from_bits(1),
            -f64::from_bits(1),
            ADDEND_MAX,
            -ADDEND_MAX,
            1.0,
            -1.0,
        ];
        for &a in &edges {
            for &b in &edges {
                for &c in &addends {
                    for (sa, sb) in [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)] {
                        assert!(in_emulated_range(sa * a, sb * b, c));
                        assert_same(sa * a, sb * b, c);
                    }
                }
            }
        }
        // Outside the range the dispatch must never take the emulation.
        // The zero-product identity or `mul_add` retains the same result.
        let outside = [
            0.0,
            -0.0,
            f64::from_bits(FACTOR_MIN.to_bits() - 1),
            f64::from_bits(FACTOR_MAX.to_bits() + 1),
            f64::MIN_POSITIVE,
            f64::from_bits(1),
            f64::MAX,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
        ];
        for &x in &outside {
            assert!(!in_emulated_range(x, 1.0, 1.0));
            assert!(!in_emulated_range(1.0, x, 1.0));
            for &y in edges.iter().chain(&outside) {
                for &c in addends.iter().chain(&outside) {
                    assert_same(x, y, c);
                    assert_same(y, x, c);
                }
            }
        }
        for c in [
            f64::from_bits(ADDEND_MAX.to_bits() + 1),
            f64::MAX,
            f64::INFINITY,
            f64::NAN,
        ] {
            assert!(!in_emulated_range(1.0, 1.0, c));
            assert_same(1.5, 3.0, c);
        }
    }

    #[test]
    fn zero_addend_matches_native_across_every_finite_exponent() {
        let fractions = [0, 1, (1_u64 << 51) - 1, 1_u64 << 51, (1_u64 << 52) - 1];
        let factors = [
            f64::from_bits(1),
            f64::MIN_POSITIVE,
            0.5,
            f64::from_bits(1.0_f64.to_bits() - 1),
            1.0,
            1.0 + f64::EPSILON,
            1.5,
            2.0,
            f64::MAX,
        ];
        for biased_exponent in 0_u64..0x7ff {
            for fraction in fractions {
                let a = f64::from_bits((biased_exponent << 52) | fraction);
                if a == 0.0 {
                    continue;
                }
                for b in factors {
                    for (sa, sb) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                        for c in [0.0, -0.0] {
                            assert_same(sa * a, sb * b, c);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn unit_factors_match_native_across_every_finite_exponent() {
        let fractions = [0, 1, (1_u64 << 51) - 1, 1_u64 << 51, (1_u64 << 52) - 1];
        let addends = [
            0.0,
            -0.0,
            f64::from_bits(1),
            -f64::from_bits(1),
            f64::MIN_POSITIVE,
            -f64::MIN_POSITIVE,
            1.0,
            -1.0,
            f64::MAX,
            -f64::MAX,
        ];
        for biased_exponent in 0_u64..0x7ff {
            for fraction in fractions {
                let value = f64::from_bits((biased_exponent << 52) | fraction);
                for signed_value in [value, -value] {
                    for unit in [1.0, -1.0] {
                        for c in addends.into_iter().chain([-signed_value, signed_value]) {
                            assert_same(unit, signed_value, c);
                            assert_same(signed_value, unit, c);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn dispatch_preserves_ties_underflow_overflow_and_zero_signs() {
        let tiny = f64::from_bits(1);
        let half_ulp = f64::from_bits((1023_u64 - 53) << 52);
        let overflow_half_ulp = f64::from_bits((1023_u64 + 970) << 52);
        let cases = [
            (tiny, 0.5, 0.0, 0.0),
            (-tiny, 0.5, 0.0, -0.0),
            (3.0 * tiny, 0.5, -0.0, 2.0 * tiny),
            (-3.0 * tiny, 0.5, -0.0, -2.0 * tiny),
            (f64::MIN_POSITIVE, 0.5, 0.0, f64::MIN_POSITIVE / 2.0),
            (
                f64::MIN_POSITIVE,
                f64::from_bits(1.0_f64.to_bits() - 1),
                0.0,
                f64::MIN_POSITIVE,
            ),
            (f64::MAX, 2.0, 0.0, f64::INFINITY),
            (-f64::MAX, 2.0, -0.0, f64::NEG_INFINITY),
            (1.0, 1.0, half_ulp, 1.0),
            (1.0, 1.0 + f64::EPSILON, half_ulp, 1.0 + 2.0 * f64::EPSILON),
            (-1.0, 1.0, -half_ulp, -1.0),
            (
                -1.0,
                1.0 + f64::EPSILON,
                -half_ulp,
                -(1.0 + 2.0 * f64::EPSILON),
            ),
            (1.0, f64::MAX, overflow_half_ulp, f64::INFINITY),
            (
                1.0,
                f64::MAX,
                f64::from_bits(overflow_half_ulp.to_bits() - 1),
                f64::MAX,
            ),
            (1.0, f64::MAX, -f64::MAX, 0.0),
            (-1.0, f64::MAX, f64::MAX, 0.0),
        ];
        for (a, b, c, expected) in cases {
            assert_eq!(a.mul_add(b, c).to_bits(), expected.to_bits());
            assert_eq!(wasm_fma(a, b, c).to_bits(), expected.to_bits());
            assert_eq!(wasm_fma(b, a, c).to_bits(), expected.to_bits());
        }

        // Exact zero products deliberately retain the existing multiply/add
        // sign rule, including opposite signed zeros and unit factors.
        for zero in [0.0, -0.0] {
            for factor in [0.0, -0.0, 1.0, -1.0, tiny, -tiny, f64::MAX, -f64::MAX] {
                for c in [0.0, -0.0] {
                    assert_same(zero, factor, c);
                    assert_same(factor, zero, c);
                }
            }
        }
    }

    #[test]
    fn dispatch_matches_native_on_random_finite_bit_patterns() {
        let mut rng = Rng(0xF1A0_5EED_0001_0000);
        let mut zero_addends = 0;
        let mut unit_products = 0;
        for _ in 0..200_000 {
            let a = f64::from_bits(rng.next());
            let b = f64::from_bits(rng.next());
            if a.is_finite() && b.is_finite() {
                for c in [0.0, -0.0] {
                    assert_same(a, b, c);
                }
                zero_addends += 1;
                for unit in [1.0, -1.0] {
                    assert_same(unit, a, b);
                    assert_same(a, unit, b);
                }
                unit_products += 1;
            }
        }
        assert!(zero_addends > 199_000);
        assert!(unit_products > 199_000);
    }

    #[test]
    fn dispatch_preserves_nonfinite_and_nonunit_results() {
        let special = [
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::from_bits(0x7ff8_0000_0000_1234),
            f64::from_bits(0xfff8_0000_0000_4321),
            f64::from_bits(0x7ff0_0000_0000_0001),
        ];
        let finite = [0.0, -0.0, 1.0, -1.0, f64::from_bits(1), f64::MAX];
        for x in special {
            for y in finite.into_iter().chain(special) {
                for c in finite.into_iter().chain(special) {
                    // Nonfinite inputs keep the original mul_add fallback
                    // and the existing NaN-equivalence test policy.
                    assert_same(x, y, c);
                    assert_same(y, x, c);
                    assert_same(y, c, x);
                }
            }
        }
        for unit in [1.0_f64, -1.0] {
            for adjacent in [
                f64::from_bits(unit.to_bits() - 1),
                f64::from_bits(unit.to_bits() + 1),
            ] {
                for other in [1.0 + f64::EPSILON, f64::MAX, f64::MIN_POSITIVE] {
                    assert_same(adjacent, other, -(adjacent * other));
                    assert_same(other, adjacent, -(adjacent * other));
                }
            }
        }
        // Dropping a nonzero addend after a product overflow is invalid:
        // the original fused fallback still avoids that intermediate overflow.
        assert_eq!(
            wasm_fma(f64::MAX, 2.0, -f64::MAX).to_bits(),
            f64::MAX.to_bits()
        );
    }

    #[test]
    fn method_and_free_forms_agree_with_mul_add() {
        let cases: [(f64, f64, f64); 4] = [
            (0.1, 0.2, 0.3),
            (1e300, 1e10, -1.0),
            (-0.0, 5.0, 0.0),
            (3.0, f64::NAN, 1.0),
        ];
        for (a, b, c) in cases {
            let reference = a.mul_add(b, c);
            for got in [fma(a, b, c), a.fma(b, c)] {
                assert!(
                    got.to_bits() == reference.to_bits() || (got.is_nan() && reference.is_nan())
                );
            }
        }
    }

    /// Long run for local validation: `REMUS_FMA_CASES=1000000000 cargo test
    /// --release -p remus-math fma::tests::long_random_differential -- --ignored`.
    #[test]
    #[ignore = "long-running exhaustive-style sweep; run by hand"]
    fn long_random_differential() {
        let cases: u64 = std::env::var("REMUS_FMA_CASES")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(100_000_000);
        let seed: u64 = std::env::var("REMUS_FMA_SEED")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(1);
        let mut rng = Rng(seed);
        for _ in 0..cases {
            let span = if rng.below(4) == 0 { 450 } else { 40 };
            let (bits_a, bits_b) = (rng.fraction_bits(), rng.fraction_bits());
            let a = rng.normal_in(-span, span, bits_a);
            let b = rng.normal_in(-span, span, bits_b);
            let c = addend(&mut rng, a, b);
            assert_same(a, b, c);
        }
    }
    #[test]
    fn zero_addend_dispatch_matches_native_throughout_emulated_factor_range() {
        let mut eligible = 0_u32;
        for biased_exponent in (1023_u64 - 450)..=(1023 + 450) {
            for fraction in [0, 1, (1_u64 << 52) - 1] {
                let value = f64::from_bits((biased_exponent << 52) | fraction);
                if value > FACTOR_MAX {
                    continue;
                }
                for b in [FACTOR_MIN, 1.0, FACTOR_MAX] {
                    for (sa, sb) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                        let a = sa * value;
                        let b = sb * b;
                        let product = a * b;
                        assert!(product.is_normal());
                        for c in [0.0, -0.0] {
                            assert!(in_emulated_range(a, b, c));
                            assert_eq!(wasm_fma(a, b, c).to_bits(), product.to_bits());
                            assert_same(a, b, c);
                            eligible += 1;
                        }
                    }
                }
            }
        }
        assert!(eligible > 64_000);
        // Outside this range, zero addends retain the existing dispatch.
        for a in [
            0.0,
            -0.0,
            f64::from_bits(1),
            f64::MIN_POSITIVE,
            f64::from_bits(FACTOR_MIN.to_bits() - 1),
            f64::from_bits(FACTOR_MAX.to_bits() + 1),
            f64::MAX,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
        ] {
            for c in [0.0, -0.0] {
                assert!(!in_emulated_range(a, 1.0, c));
                assert_same(a, 1.0, c);
                assert_same(1.0, a, c);
            }
        }
    }
}
