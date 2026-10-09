//! Bit-identity oracle for `NurbsSurface::evaluate`'s basis buffers.
//!
//! `evaluate_pre_stack_widening` is `evaluate` as it stood when its u and v
//! basis buffers held `MAX_STACK_OUTPUT + 1` values and degrees 9 and 10 took
//! heap `Vec`s, kept verbatim. Only the storage moved, so every case compares
//! the two bit for bit over the full grid of u and v degrees 1 through 12 —
//! both sides of the old (8 | 9) and new (10 | 11) stack/heap boundaries in
//! each direction independently — on rational and non-rational surfaces,
//! single- and multi-span knot vectors, at interior, knot, boundary,
//! out-of-domain, signed-zero and NaN parameters.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_lossless,
    clippy::cast_precision_loss
)]

use super::*;

/// Highest degree exercised in each direction; must clear the new boundary
/// by two so a heap degree is still compared after any future widening by
/// one.
const TOP_DEGREE: usize = 12;
const _: () = assert!(basis::MAX_STACK_OUTPUT < basis::MAX_STACK_DEGREE);
const _: () = assert!(basis::MAX_STACK_DEGREE + 2 <= TOP_DEGREE);

impl NurbsSurface {
    /// `evaluate` before its basis buffers were sized to `MAX_STACK_DEGREE`,
    /// verbatim.
    fn evaluate_pre_stack_widening(&self, u: f64, v: f64) -> Point3 {
        let pu = self.degree_u;
        let pv = self.degree_v;
        let n_rows = self.control_points.len();
        let n_cols = self.control_points[0].len();
        let u = u.clamp(self.knots_u[pu], self.knots_u[n_rows]);
        let v = v.clamp(self.knots_v[pv], self.knots_v[n_cols]);

        let span_u = basis::find_span(n_rows, pu, u, &self.knots_u);
        let span_v = basis::find_span(n_cols, pv, v, &self.knots_v);
        let mut nu_stack = [0.0f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nu_heap;
        let nu: &mut [f64] = if pu <= basis::MAX_STACK_OUTPUT {
            &mut nu_stack[..=pu]
        } else {
            nu_heap = vec![0.0; pu + 1];
            &mut nu_heap
        };
        basis::basis_funs_into(span_u, u, pu, &self.knots_u, nu);
        let mut nv_stack = [0.0f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nv_heap;
        let nv: &mut [f64] = if pv <= basis::MAX_STACK_OUTPUT {
            &mut nv_stack[..=pv]
        } else {
            nv_heap = vec![0.0; pv + 1];
            &mut nv_heap
        };
        basis::basis_funs_into(span_v, v, pv, &self.knots_v, nv);

        // Contract along v first for each relevant u-row, then along u.
        let scale = nu
            .iter()
            .enumerate()
            .take(pu + 1)
            .flat_map(|(i, &nu_i)| {
                nv.iter().enumerate().take(pv + 1).map(move |(j, &nv_j)| {
                    let u_idx = span_u - pu + i;
                    let v_idx = span_v - pv + j;
                    (nu_i * nv_j * self.weights[u_idx][v_idx]).abs()
                })
            })
            .fold(0.0_f64, f64::max);
        let mut wx = 0.0;
        let mut wy = 0.0;
        let mut wz = 0.0;
        let mut ww = 0.0;

        for (i, &nu_i) in nu.iter().enumerate().take(pu + 1) {
            let u_idx = span_u - pu + i;
            // Evaluate the v-direction for this row.
            let mut row_x = 0.0;
            let mut row_y = 0.0;
            let mut row_z = 0.0;
            let mut row_w = 0.0;
            for (j, &nv_j) in nv.iter().enumerate().take(pv + 1) {
                let v_idx = span_v - pv + j;
                let pt = &self.control_points[u_idx][v_idx];
                let w = self.weights[u_idx][v_idx];
                let bw = nv_j * w / scale;
                row_x += bw * pt.x();
                row_y += bw * pt.y();
                row_z += bw * pt.z();
                row_w += bw;
            }
            wx += nu_i * row_x;
            wy += nu_i * row_y;
            wz += nu_i * row_z;
            ww += nu_i * row_w;
        }

        debug_assert!(scale.is_finite() && scale > 0.0);
        debug_assert!(ww.is_finite() && ww > 0.0);
        Point3::new(wx / ww, wy / ww, wz / ww)
    }
}

fn bits(p: Point3) -> [u64; 3] {
    [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]
}

/// Clamped knots of `degree`: a single Bezier span over `[0, 1]`, or a
/// multi-span vector over `[-1.5, 2.25]` with a double interior knot at 0.
fn knots(degree: usize, multi_span: bool) -> Vec<f64> {
    let (lo, hi, interior): (f64, f64, &[f64]) = if multi_span {
        (-1.5, 2.25, &[-0.75, 0.0, 0.0, 1.125])
    } else {
        (0.0, 1.0, &[])
    };
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

const WEIGHT_SETS: [Weights; 3] = [Weights::Unit, Weights::Rational, Weights::Tiny];

fn surface(
    degree_u: usize,
    degree_v: usize,
    multi_span_u: bool,
    multi_span_v: bool,
    weights: Weights,
) -> NurbsSurface {
    let knots_u = knots(degree_u, multi_span_u);
    let knots_v = knots(degree_v, multi_span_v);
    let n_rows = knots_u.len() - degree_u - 1;
    let n_cols = knots_v.len() - degree_v - 1;
    let control_points = (0..n_rows)
        .map(|i| {
            (0..n_cols)
                .map(|j| {
                    let (s, t) = (i as f64, j as f64);
                    Point3::new(
                        s.mul_add(0.8, (t * 1.37).sin()),
                        t.mul_add(0.6, -(s * 0.73).cos()),
                        ((s * 0.5).sin() * (t * 0.9).cos()).mul_add(2.0, -0.25),
                    )
                })
                .collect()
        })
        .collect();
    let weights = (0..n_rows)
        .map(|i| {
            (0..n_cols)
                .map(|j| {
                    let varied = (((i * 7 + j * 3) % 5) as f64).mul_add(0.37, 0.3);
                    match weights {
                        Weights::Unit => 1.0,
                        Weights::Rational => varied,
                        Weights::Tiny => varied * 1e-300,
                    }
                })
                .collect()
        })
        .collect();
    NurbsSurface::new(
        degree_u,
        degree_v,
        knots_u,
        knots_v,
        control_points,
        weights,
    )
    .expect("valid oracle surface")
}

/// Interior fractions, every distinct knot value, the domain ends and their
/// neighbours, signed zeros (`-0.0` survives the clamp at a `0.0` domain
/// start and reaches the span search and the basis), and out-of-domain
/// values out to infinity.
fn parameters((lo, hi): (f64, f64), knots: &[f64]) -> Vec<f64> {
    let mut params: Vec<f64> = [0.013, 0.37, 0.61, 0.999_999_9]
        .iter()
        .map(|t| (hi - lo).mul_add(*t, lo))
        .collect();
    let mut distinct = knots.to_vec();
    distinct.dedup_by(|a, b| a.to_bits() == b.to_bits());
    params.extend_from_slice(&distinct);
    params.extend_from_slice(&[
        lo.next_up(),
        hi.next_down(),
        -0.0,
        lo - 1.0,
        hi + 1.0,
        f64::NEG_INFINITY,
        f64::INFINITY,
    ]);
    params
}

/// The u/v span layouts compared at every degree pair.
const LAYOUTS: [(bool, bool); 3] = [(false, false), (true, true), (false, true)];

#[test]
fn evaluate_matches_pre_change_bits_across_stack_heap_boundaries() {
    let mut compared = 0usize;
    for degree_u in 1..=TOP_DEGREE {
        for degree_v in 1..=TOP_DEGREE {
            for (multi_u, multi_v) in LAYOUTS {
                for weights in WEIGHT_SETS {
                    let s = surface(degree_u, degree_v, multi_u, multi_v, weights);
                    assert_eq!(s.is_rational(), !matches!(weights, Weights::Unit));
                    let us = parameters(s.domain_u(), s.knots_u());
                    let vs = parameters(s.domain_v(), s.knots_v());
                    for &u in &us {
                        for &v in &vs {
                            let new = s.evaluate(u, v);
                            let old = s.evaluate_pre_stack_widening(u, v);
                            assert_eq!(
                                bits(new),
                                bits(old),
                                "degrees ({degree_u}, {degree_v}), multi ({multi_u}, \
                                 {multi_v}), {weights:?}, (u, v) ({u:e}, {v:e}): \
                                 {new:?} vs {old:?}"
                            );
                            compared += 1;
                        }
                    }
                }
            }
        }
    }
    // Each surface compares at least 13 x 13 parameter pairs; guards against
    // the loops silently shrinking.
    let surfaces = TOP_DEGREE * TOP_DEGREE * LAYOUTS.len() * WEIGHT_SETS.len();
    assert!(compared >= surfaces * 13 * 13, "compared only {compared}");
}

#[test]
fn nan_parameter_outcome_matches_pre_change() {
    // NaN survives the clamp and zeroes the scale. Under debug assertions
    // both versions trip the scale `debug_assert!`; otherwise both return
    // the same NaN bits. Either direction may carry the NaN.
    for degree_u in 1..=TOP_DEGREE {
        for degree_v in [1, 8, 9, 10, 11, TOP_DEGREE] {
            for weights in WEIGHT_SETS {
                let s = surface(degree_u, degree_v, true, true, weights);
                for (u, v) in [(f64::NAN, 0.5), (0.5, f64::NAN), (f64::NAN, f64::NAN)] {
                    let new = std::panic::catch_unwind(|| bits(s.evaluate(u, v)));
                    let old =
                        std::panic::catch_unwind(|| bits(s.evaluate_pre_stack_widening(u, v)));
                    let case = format!("degrees ({degree_u}, {degree_v}), {weights:?}, ({u}, {v})");
                    if cfg!(debug_assertions) {
                        assert!(new.is_err() && old.is_err(), "{case}");
                    } else {
                        assert_eq!(new.unwrap(), old.unwrap(), "{case}");
                    }
                }
            }
        }
    }
}
