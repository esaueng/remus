//! Bit-identity oracle for `NurbsCurve::evaluate`'s basis buffer.
//!
//! `evaluate_pre_stack_widening` is `evaluate` as it stood when its basis
//! buffer held `MAX_STACK_OUTPUT + 1` values and degrees 9 and 10 took a heap
//! `Vec`, kept verbatim. Only the storage moved, so every case compares the
//! two bit for bit at degrees 1 through 12 — both sides of the old (8 | 9)
//! and new (10 | 11) stack/heap boundaries — on rational and non-rational
//! curves, single- and multi-span knot vectors, at interior, knot, boundary,
//! out-of-domain, signed-zero and NaN parameters.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_lossless,
    clippy::cast_precision_loss
)]

use super::*;

/// Highest degree exercised; must clear the new boundary by two so a heap
/// degree is still compared after any future widening by one.
const TOP_DEGREE: usize = 12;
const _: () = assert!(basis::MAX_STACK_OUTPUT < basis::MAX_STACK_DEGREE);
const _: () = assert!(basis::MAX_STACK_DEGREE + 2 <= TOP_DEGREE);

impl NurbsCurve {
    /// `evaluate` before its basis buffer was sized to `MAX_STACK_DEGREE`,
    /// verbatim.
    fn evaluate_pre_stack_widening(&self, u: f64) -> Point3 {
        let p = self.degree;
        let n = self.control_points.len();
        let u = u.clamp(self.knots[p], self.knots[n]);
        let span = basis::find_span(n, p, u, &self.knots);
        let mut bf_stack = [0.0f64; basis::MAX_STACK_OUTPUT + 1];
        let mut bf_heap;
        let bf: &mut [f64] = if p <= basis::MAX_STACK_OUTPUT {
            &mut bf_stack[..=p]
        } else {
            bf_heap = vec![0.0; p + 1];
            &mut bf_heap
        };
        basis::basis_funs_into(span, u, p, &self.knots, bf);

        // Scale homogeneous terms before summation. NURBS weights are
        // projective, so this preserves the point while preventing a common
        // factor such as 1e-300 from making the perspective divide unstable.
        let scale = bf
            .iter()
            .enumerate()
            .take(p + 1)
            .map(|(j, &basis_val)| (basis_val * self.weights[span - p + j]).abs())
            .fold(0.0_f64, f64::max);
        let mut wx = 0.0;
        let mut wy = 0.0;
        let mut wz = 0.0;
        let mut ww = 0.0;
        for (j, &basis_val) in bf.iter().enumerate().take(p + 1) {
            let idx = span - p + j;
            let pt = &self.control_points[idx];
            let w = self.weights[idx];
            let bw = basis_val * w / scale;
            wx += bw * pt.x();
            wy += bw * pt.y();
            wz += bw * pt.z();
            ww += bw;
        }

        debug_assert!(scale.is_finite() && scale > 0.0);
        debug_assert!(ww.is_finite() && ww > 0.0);
        Point3::new(wx / ww, wy / ww, wz / ww)
    }
}

fn bits(p: Point3) -> [u64; 3] {
    [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]
}

/// Clamped knots over `[lo, hi]` with `interior` knots between the end
/// blocks of multiplicity `degree + 1`.
fn clamped(degree: usize, lo: f64, hi: f64, interior: &[f64]) -> Vec<f64> {
    let mut knots = vec![lo; degree + 1];
    knots.extend_from_slice(interior);
    knots.extend(std::iter::repeat_n(hi, degree + 1));
    knots
}

#[derive(Clone, Copy, Debug)]
enum Weights {
    /// All `1.0`.
    Unit,
    /// Varied positive weights.
    Rational,
    /// Varied weights times `1e-300`, so the scale guard does real work.
    Tiny,
}

/// One curve per (degree, layout, weights): a single Bezier span over
/// `[0, 1]`, or a multi-span curve over `[-1.5, 2.25]` with a double
/// interior knot.
fn curve(degree: usize, multi_span: bool, weights: Weights) -> NurbsCurve {
    let knots = if multi_span {
        clamped(degree, -1.5, 2.25, &[-0.75, 0.0, 0.0, 1.125])
    } else {
        clamped(degree, 0.0, 1.0, &[])
    };
    let n = knots.len() - degree - 1;
    let control_points = (0..n)
        .map(|i| {
            let t = i as f64;
            Point3::new(
                (t * 1.37).sin() * 5.0,
                (t * 0.73).cos().mul_add(3.0, -1.0),
                t.mul_add(0.11, -0.4),
            )
        })
        .collect();
    let weights = (0..n)
        .map(|i| {
            let varied = (((i * 7) % 5) as f64).mul_add(0.37, 0.3);
            match weights {
                Weights::Unit => 1.0,
                Weights::Rational => varied,
                Weights::Tiny => varied * 1e-300,
            }
        })
        .collect();
    NurbsCurve::new(degree, knots, control_points, weights).expect("valid oracle curve")
}

/// Interior fractions, every knot value, the domain ends and their
/// neighbours, signed zeros (`-0.0` survives the clamp at a `0.0` domain
/// start and reaches the span search and the basis), and out-of-domain
/// values out to infinity.
fn parameters(curve: &NurbsCurve) -> Vec<f64> {
    let (lo, hi) = curve.domain();
    let mut params: Vec<f64> = [0.013, 0.25, 0.37, 0.5, 0.61, 0.875, 0.999_999_9]
        .iter()
        .map(|t| (hi - lo).mul_add(*t, lo))
        .collect();
    params.extend_from_slice(curve.knots());
    params.extend_from_slice(&[
        lo,
        hi,
        lo.next_up(),
        hi.next_down(),
        0.0,
        -0.0,
        f64::MIN_POSITIVE,
        -f64::MIN_POSITIVE,
        lo - 1.0,
        hi + 1.0,
        f64::MIN,
        f64::MAX,
        f64::NEG_INFINITY,
        f64::INFINITY,
    ]);
    params
}

const WEIGHT_SETS: [Weights; 3] = [Weights::Unit, Weights::Rational, Weights::Tiny];

#[test]
fn evaluate_matches_pre_change_bits_across_stack_heap_boundaries() {
    let mut compared = 0usize;
    for degree in 1..=TOP_DEGREE {
        for multi_span in [false, true] {
            for weights in WEIGHT_SETS {
                let c = curve(degree, multi_span, weights);
                assert_eq!(c.is_rational(), !matches!(weights, Weights::Unit));
                for u in parameters(&c) {
                    let new = c.evaluate(u);
                    let old = c.evaluate_pre_stack_widening(u);
                    assert_eq!(
                        bits(new),
                        bits(old),
                        "degree {degree}, multi_span {multi_span}, {weights:?}, u {u:e}: \
                         {new:?} vs {old:?}"
                    );
                    compared += 1;
                }
            }
        }
    }
    // 12 degrees x 2 layouts x 3 weight sets, each with at least 21 + knots
    // parameters: guards against the loops silently shrinking.
    assert!(
        compared > TOP_DEGREE * 2 * 3 * 21,
        "compared only {compared}"
    );
}

#[test]
fn nan_parameter_outcome_matches_pre_change() {
    // NaN survives the clamp and zeroes the scale. Under debug assertions
    // both versions trip the scale `debug_assert!`; otherwise both return
    // the same NaN bits.
    for degree in 1..=TOP_DEGREE {
        for weights in WEIGHT_SETS {
            let c = curve(degree, true, weights);
            let new = std::panic::catch_unwind(|| bits(c.evaluate(f64::NAN)));
            let old = std::panic::catch_unwind(|| bits(c.evaluate_pre_stack_widening(f64::NAN)));
            if cfg!(debug_assertions) {
                assert!(new.is_err() && old.is_err(), "degree {degree}, {weights:?}");
            } else {
                assert_eq!(new.unwrap(), old.unwrap(), "degree {degree}, {weights:?}");
            }
        }
    }
}
