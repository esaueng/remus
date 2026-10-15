//! Bit-identity oracles for the loop-based NURBS surface contractions.
//!
//! `evaluate_pre_contraction` and `point_and_partials_with_spans_pre_contraction`
//! are `NurbsSurface::evaluate` and the first-order scratch solve as they
//! stood before their contractions became plain `[x, y, z, 1]` lane loops
//! (and before the scale fold left its iterator chain), kept verbatim.
//! `normal_pre_contraction` is `NurbsSurface::normal` as it stood: a fresh
//! `DerivativeScratch` (unhinted spans, empty u memo) feeding the old
//! first-order solve, or the general derivative solve above the memo bound,
//! with the old degenerate-point perturbation fallback.
//!
//! Every case compares bit for bit (`Err` included) over rational and
//! non-rational surfaces (unit, uniform non-unit, varied and `1e-300`-scaled
//! weights), Bezier, uniform and non-uniform knot vectors with a repeated
//! interior knot, every u/v degree pair from 1 to 5 plus 9 (above the u memo
//! bound) and 11 (above the stack basis bound, heap fallbacks), at interior,
//! knot, domain-end, out-of-domain, signed-zero and infinite parameters, and
//! on collapsed, pinched and fully degenerate patches that take the
//! perturbation fallback.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_lossless,
    clippy::cast_precision_loss,
    clippy::many_single_char_names,
    clippy::suboptimal_flops
)]
// The `simd` feature keeps the general 4-lane solve, which these scalar
// oracles do not describe; only the `evaluate` comparisons run there.
#![cfg_attr(feature = "simd", allow(dead_code, unused_variables))]

use super::*;

impl NurbsSurface {
    /// `evaluate` before the loop-based contraction, verbatim.
    fn evaluate_pre_contraction(&self, u: f64, v: f64) -> Point3 {
        let pu = self.degree_u;
        let pv = self.degree_v;
        let n_rows = self.control_points.len();
        let n_cols = self.control_points[0].len();
        let u = u.clamp(self.knots_u[pu], self.knots_u[n_rows]);
        let v = v.clamp(self.knots_v[pv], self.knots_v[n_cols]);

        let span_u = basis::find_span(n_rows, pu, u, &self.knots_u);
        let span_v = basis::find_span(n_cols, pv, v, &self.knots_v);
        // Sized to `basis_funs_into`'s own stack bound: a degree it serves
        // without allocating must not pay for a heap buffer here.
        let mut nu_stack = [0.0f64; basis::MAX_STACK_DEGREE + 1];
        let mut nu_heap;
        let nu: &mut [f64] = if pu <= basis::MAX_STACK_DEGREE {
            &mut nu_stack[..=pu]
        } else {
            nu_heap = vec![0.0; pu + 1];
            &mut nu_heap
        };
        basis::basis_funs_into(span_u, u, pu, &self.knots_u, nu);
        let mut nv_stack = [0.0f64; basis::MAX_STACK_DEGREE + 1];
        let mut nv_heap;
        let nv: &mut [f64] = if pv <= basis::MAX_STACK_DEGREE {
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

    /// The first-order scratch solve before the lane contraction, verbatim.
    #[allow(clippy::many_single_char_names)]
    fn point_and_partials_with_spans_pre_contraction(
        &self,
        u: f64,
        v: f64,
        span_u: usize,
        span_v: usize,
        basis_buf: &mut [f64],
        u_memo: &mut UBasisMemo,
    ) -> Option<(Vec3, Vec3, Vec3)> {
        let pu = self.degree_u;
        let pv = self.degree_v;
        let u = u.clamp(self.knots_u[pu], self.knots_u[self.control_points.len()]);
        let v = v.clamp(self.knots_v[pv], self.knots_v[self.control_points[0].len()]);
        let ders_u = u_memo.ders(span_u, u, pu, &self.knots_u)?;
        let stride_u = pu + 1;
        let stride_v = pv + 1;
        let ders_v = &mut basis_buf[..2 * stride_v];
        basis::ders_basis_funs_first_into(span_v, v, pv, &self.knots_v, ders_v);
        let (nu, nu1) = ders_u.split_at(stride_u);
        let (nv, nv1) = ders_v.split_at(stride_v);

        let weight_scale = self.max_weight();
        debug_assert!(weight_scale.is_finite() && weight_scale > 0.0);
        let uniform = self.has_uniform_weights();
        let (mut a00, mut a01, mut a10) = ([0.0f64; 4], [0.0f64; 4], [0.0f64; 4]);
        for i in 0..=pu {
            let u_idx = span_u - pu + i;
            let row = &self.control_points[u_idx][span_v - pv..=span_v];
            let weights = &self.weights[u_idx][span_v - pv..=span_v];
            let (n_i, n1_i) = (nu[i], nu1[i]);
            for j in 0..=pv {
                let pt = row[j];
                let (c00, c01, c10) = (n_i * nv[j], n_i * nv1[j], n1_i * nv[j]);
                if uniform {
                    for (cell, c) in [(&mut a00, c00), (&mut a01, c01), (&mut a10, c10)] {
                        cell[0] += c * pt.x();
                        cell[1] += c * pt.y();
                        cell[2] += c * pt.z();
                        cell[3] += c;
                    }
                } else {
                    let w = weights[j] / weight_scale;
                    for (cell, c) in [(&mut a00, c00), (&mut a01, c01), (&mut a10, c10)] {
                        cell[0] += c * pt.x() * w;
                        cell[1] += c * pt.y() * w;
                        cell[2] += c * pt.z() * w;
                        cell[3] += c * w;
                    }
                }
            }
        }

        let w0 = a00[3];
        debug_assert!(w0.is_finite() && w0 > 0.0);
        let p = Vec3::new(a00[0] / w0, a00[1] / w0, a00[2] / w0);
        let (wv, wu) = (a01[3], a10[3]);
        let dv = Vec3::new(
            (a01[0] - wv * p.x()) / w0,
            (a01[1] - wv * p.y()) / w0,
            (a01[2] - wv * p.z()) / w0,
        );
        let du = Vec3::new(
            (a10[0] - wu * p.x()) / w0,
            (a10[1] - wu * p.y()) / w0,
            (a10[2] - wu * p.z()) / w0,
        );
        Some((p, du, dv))
    }

    /// `(∂S/∂u, ∂S/∂v)` as a fresh `DerivativeScratch::partials_from` computed
    /// them before the lane contraction: unhinted spans, a cold u memo, the
    /// old first-order solve or the general solve above the memo bound.
    fn partials_pre_contraction(&self, u: f64, v: f64) -> (Vec3, Vec3) {
        let n_rows = self.control_points.len();
        let n_cols = self.control_points[0].len();
        let uc = u.clamp(self.knots_u[self.degree_u], self.knots_u[n_rows]);
        let vc = v.clamp(self.knots_v[self.degree_v], self.knots_v[n_cols]);
        let span_u = basis::find_span(n_rows, self.degree_u, uc, &self.knots_u);
        let span_v = basis::find_span(n_cols, self.degree_v, vc, &self.knots_v);
        let mut basis_buf = vec![0.0; 2 * (self.degree_v + 1)];
        let mut memo = UBasisMemo::default();
        if let Some((_, du, dv)) = self.point_and_partials_with_spans_pre_contraction(
            u,
            v,
            span_u,
            span_v,
            &mut basis_buf,
            &mut memo,
        ) {
            return (du, dv);
        }
        let d = self.derivatives(u, v, 1);
        (d[1][0], d[0][1])
    }

    /// `normal` before the allocation-free path: the old degenerate-aware
    /// normal over the old partials, verbatim in its fallback order.
    fn normal_pre_contraction(&self, u: f64, v: f64) -> Result<Vec3, MathError> {
        let (du, dv) = self.partials_pre_contraction(u, v);
        let cross = du.cross(dv);
        if cross.length_squared() > 1e-30 {
            return cross.normalize();
        }

        // Degenerate point — try perturbing the parameter slightly.
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let eps_u = (u1 - u0) * 1e-6;
        let eps_v = (v1 - v0) * 1e-6;

        let perturbations = [
            (u + eps_u, v),
            (u - eps_u, v),
            (u, v + eps_v),
            (u, v - eps_v),
        ];

        for (pu, pv) in perturbations {
            let pu = pu.clamp(u0, u1);
            let pv = pv.clamp(v0, v1);
            let (pdu, pdv) = self.partials_pre_contraction(pu, pv);
            let pcross = pdu.cross(pdv);
            if pcross.length_squared() > 1e-30 {
                return pcross.normalize();
            }
        }

        Err(crate::MathError::ZeroVector)
    }
}

fn point_bits(p: Point3) -> [u64; 3] {
    [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]
}

fn vec_bits(v: Vec3) -> [u64; 3] {
    [v.x().to_bits(), v.y().to_bits(), v.z().to_bits()]
}

fn normal_bits(n: &Result<Vec3, MathError>) -> Result<[u64; 3], String> {
    n.as_ref()
        .map(|v| vec_bits(*v))
        .map_err(|e| format!("{e:?}"))
}

/// Degrees compared in each direction: 1 through 5, one above the u memo
/// bound and one above the stack basis bound (heap basis temporaries).
const DEGREES: [usize; 7] = [1, 2, 3, 4, 5, 9, 11];
const _: () = assert!(MEMO_MAX_DEGREE < 9 && basis::MAX_STACK_DEGREE < 11);

#[derive(Clone, Copy, Debug)]
enum Knots {
    /// One Bezier span over `[0, 1]`.
    Bezier,
    /// Uniform interior knots over `[0, 4]`.
    Uniform,
    /// Non-uniform over `[-1.5, 2.25]` with a repeated interior knot.
    Repeated,
}

const KNOT_SETS: [Knots; 3] = [Knots::Bezier, Knots::Uniform, Knots::Repeated];

fn knots(degree: usize, layout: Knots) -> Vec<f64> {
    let (lo, hi, interior): (f64, f64, &[f64]) = match layout {
        Knots::Bezier => (0.0, 1.0, &[]),
        Knots::Uniform => (0.0, 4.0, &[1.0, 2.0, 3.0]),
        Knots::Repeated => (-1.5, 2.25, &[-0.75, 0.0, 0.0, 1.125]),
    };
    let mut knots = vec![lo; degree + 1];
    knots.extend_from_slice(interior);
    knots.extend(std::iter::repeat_n(hi, degree + 1));
    knots
}

#[derive(Clone, Copy, Debug)]
enum Weights {
    /// All `1.0`: the uniform-weight shortcut.
    Unit,
    /// All `2.5`: the shortcut with a non-unit common weight.
    UniformTwoAndAHalf,
    /// Varied positive weights.
    Rational,
    /// Varied weights times `1e-300`, so the scale guards do real work.
    Tiny,
}

const WEIGHT_SETS: [Weights; 4] = [
    Weights::Unit,
    Weights::UniformTwoAndAHalf,
    Weights::Rational,
    Weights::Tiny,
];

fn surface(
    degree_u: usize,
    degree_v: usize,
    ku: Knots,
    kv: Knots,
    weights: Weights,
) -> NurbsSurface {
    let knots_u = knots(degree_u, ku);
    let knots_v = knots(degree_v, kv);
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
                        Weights::UniformTwoAndAHalf => 2.5,
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
/// neighbours, signed zeros, and out-of-domain values out to infinity.
fn parameters((lo, hi): (f64, f64), knots: &[f64]) -> Vec<f64> {
    let mut params: Vec<f64> = [0.013, 0.37, 0.5, 0.61, 0.999_999_9]
        .iter()
        .map(|t| (hi - lo).mul_add(*t, lo))
        .collect();
    let mut distinct = knots.to_vec();
    distinct.dedup_by(|a, b| a.to_bits() == b.to_bits());
    params.extend_from_slice(&distinct);
    params.extend_from_slice(&[
        lo.next_up(),
        hi.next_down(),
        0.0,
        -0.0,
        lo - 1.0,
        hi + 1.0,
        f64::NEG_INFINITY,
        f64::INFINITY,
    ]);
    params
}

/// Every fixture surface of the degree/knot/weight grid, with a label.
fn corpus() -> Vec<(String, NurbsSurface)> {
    let mut out = Vec::new();
    for degree_u in DEGREES {
        for degree_v in DEGREES {
            for (ku, kv) in [
                (Knots::Bezier, Knots::Bezier),
                (Knots::Uniform, Knots::Repeated),
                (Knots::Repeated, Knots::Uniform),
            ] {
                for weights in WEIGHT_SETS {
                    let label = format!("({degree_u}, {degree_v}) {ku:?}/{kv:?} {weights:?}");
                    out.push((label, surface(degree_u, degree_v, ku, kv, weights)));
                }
            }
        }
    }
    // Both knot layouts on both axes at the common degrees.
    for degree in [1, 2, 3] {
        for ku in KNOT_SETS {
            for kv in KNOT_SETS {
                let label = format!("({degree}, {degree}) {ku:?}/{kv:?} Rational");
                out.push((label, surface(degree, degree, ku, kv, Weights::Rational)));
            }
        }
    }
    out
}

/// Patches whose normal takes the perturbation fallback somewhere: a first
/// row collapsed to a point, a last row collapsed, an interior pinch, and a
/// fully degenerate patch on which every perturbation fails.
fn degenerate_corpus() -> Vec<(&'static str, NurbsSurface)> {
    let bilinear = |pts: [[Point3; 2]; 2]| {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            pts.iter().map(|row| row.to_vec()).collect(),
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .expect("bilinear degenerate fixture")
    };
    let o = Point3::new(0.0, 0.0, 0.0);
    let pinch = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.25, 0.0, 0.0), Point3::new(0.25, 1.0, -0.5)],
            vec![Point3::new(-0.25, 0.0, 0.0), Point3::new(-0.25, 1.0, 0.0)],
            vec![Point3::new(0.25, 0.0, 0.0), Point3::new(0.25, 1.0, 0.5)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .expect("pinch fixture");
    // A rational cubic cone-like patch: row 0 collapses to an apex.
    let apex = NurbsSurface::new(
        3,
        2,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        (0..4)
            .map(|i| {
                let r = i as f64 / 3.0;
                vec![
                    Point3::new(r, 0.0, r),
                    Point3::new(r, r, r),
                    Point3::new(0.0, r, r),
                ]
            })
            .collect(),
        vec![vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0]; 4],
    )
    .expect("apex fixture");
    vec![
        (
            "collapsed u0",
            bilinear([
                [o, o],
                [Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ]),
        ),
        (
            "collapsed u1",
            bilinear([
                [o, Point3::new(1.0, 0.0, 0.0)],
                [Point3::new(0.0, 1.0, 0.0); 2],
            ]),
        ),
        ("pinch", pinch),
        ("rational apex", apex),
        ("point", bilinear([[Point3::new(1.0, 2.0, 3.0); 2]; 2])),
    ]
}

#[test]
fn evaluate_matches_pre_contraction_bits() {
    let mut compared = 0usize;
    for (label, s) in corpus() {
        let us = parameters(s.domain_u(), s.knots_u());
        let vs = parameters(s.domain_v(), s.knots_v());
        for &u in &us {
            for &v in &vs {
                let new = s.evaluate(u, v);
                let old = s.evaluate_pre_contraction(u, v);
                assert_eq!(
                    point_bits(new),
                    point_bits(old),
                    "{label} at ({u:e}, {v:e}): {new:?} vs {old:?}"
                );
                compared += 1;
            }
        }
    }
    assert!(compared > 150_000, "compared only {compared}");
}

#[test]
fn nan_evaluate_outcome_matches_pre_contraction() {
    for (label, s) in corpus().into_iter().step_by(5) {
        for (u, v) in [(f64::NAN, 0.5), (0.5, f64::NAN), (f64::NAN, f64::NAN)] {
            let new = std::panic::catch_unwind(|| point_bits(s.evaluate(u, v)));
            let old = std::panic::catch_unwind(|| point_bits(s.evaluate_pre_contraction(u, v)));
            if cfg!(debug_assertions) {
                assert!(new.is_err() && old.is_err(), "{label} ({u}, {v})");
            } else {
                assert_eq!(new.unwrap(), old.unwrap(), "{label} ({u}, {v})");
            }
        }
    }
}

/// The lane contraction behind every first-order entry point, and the
/// allocation-free `normal`, against the old solve and the old `normal`.
#[cfg(not(feature = "simd"))]
#[test]
fn first_order_and_normal_match_pre_contraction_bits() {
    let mut compared = 0usize;
    for (label, s) in corpus() {
        let us = parameters(s.domain_u(), s.knots_u());
        let vs = parameters(s.domain_v(), s.knots_v());
        let mut hinted = DerivativeScratch::new();
        for &u in &us {
            for &v in &vs {
                let (du, dv) = s.partials_pre_contraction(u, v);
                let general = s.derivatives(u, v, 1);
                assert_eq!(
                    [vec_bits(du), vec_bits(dv)],
                    [vec_bits(general[1][0]), vec_bits(general[0][1])],
                    "{label}: old solve vs general at ({u:e}, {v:e})"
                );
                let (p, du2, dv2, _, _) = hinted.span_hinted_point_and_partials_from(&s, u, v);
                assert_eq!(
                    [vec_bits(p), vec_bits(du2), vec_bits(dv2)],
                    [vec_bits(general[0][0]), vec_bits(du), vec_bits(dv)],
                    "{label}: first-order solve at ({u:e}, {v:e})"
                );
                assert_eq!(
                    normal_bits(&s.normal(u, v)),
                    normal_bits(&s.normal_pre_contraction(u, v)),
                    "{label}: normal at ({u:e}, {v:e})"
                );
                compared += 1;
            }
        }
    }
    assert!(compared > 150_000, "compared only {compared}");
}

#[cfg(not(feature = "simd"))]
#[test]
fn degenerate_normals_match_pre_contraction_bits() {
    let mut fallbacks = 0usize;
    for (label, s) in degenerate_corpus() {
        let us = parameters(s.domain_u(), s.knots_u());
        let vs = parameters(s.domain_v(), s.knots_v());
        let mut scratch = DerivativeScratch::new();
        for &u in &us {
            for &v in &vs {
                let (du, dv) = s.partials_pre_contraction(u, v);
                if du.cross(dv).length_squared() <= 1e-30 {
                    fallbacks += 1;
                }
                let old = normal_bits(&s.normal_pre_contraction(u, v));
                assert_eq!(
                    normal_bits(&s.normal(u, v)),
                    old,
                    "{label} at ({u:e}, {v:e})"
                );
                assert_eq!(
                    normal_bits(&scratch.normal_from(&s, u, v)),
                    old,
                    "{label}: scratch normal at ({u:e}, {v:e})"
                );
            }
        }
    }
    // Every fixture is degenerate along a row or everywhere.
    assert!(fallbacks > 100, "only {fallbacks} fallback points");
}

/// The same net moved by a fixed offset: equal degrees, knots and weights,
/// different control points, as a tessellation chart frame builds it.
fn moved(s: &NurbsSurface, offset: Vec3) -> NurbsSurface {
    NurbsSurface::new(
        s.degree_u(),
        s.degree_v(),
        s.knots_u().to_vec(),
        s.knots_v().to_vec(),
        s.control_points()
            .iter()
            .map(|row| row.iter().map(|&p| p + offset).collect())
            .collect(),
        s.weights().to_vec(),
    )
    .expect("moved net")
}

/// The fused position-and-partials solve against the separate pre-change
/// `evaluate` and first-order solve, through one scratch walked along each
/// surface's grid (span hints and u memo hitting and missing) and handed
/// from surface to surface.
#[test]
fn evaluate_and_partials_match_the_separate_pre_change_solves() {
    let mut shared = DerivativeScratch::new();
    let mut compared = 0usize;
    for (label, s) in corpus() {
        let us = parameters(s.domain_u(), s.knots_u());
        let vs = parameters(s.domain_v(), s.knots_v());
        let mut own = DerivativeScratch::new();
        for &u in &us {
            for &v in &vs {
                let point = point_bits(s.evaluate_pre_contraction(u, v));
                let (du, dv) = if cfg!(feature = "simd") {
                    let d = s.derivatives(u, v, 1);
                    (d[1][0], d[0][1])
                } else {
                    s.partials_pre_contraction(u, v)
                };
                for scratch in [&mut own, &mut shared] {
                    let (p, du2, dv2) = scratch.evaluate_and_partials_from(&s, u, v);
                    assert_eq!(
                        (point_bits(p), vec_bits(du2), vec_bits(dv2)),
                        (point, vec_bits(du), vec_bits(dv)),
                        "{label} at ({u:e}, {v:e})"
                    );
                }
                compared += 1;
            }
        }
    }
    assert!(compared > 150_000, "compared only {compared}");
}

/// `point_and_normal` and `point_and_normal_with` against the pre-change
/// `evaluate` and `normal`: on the surface itself, on a moved copy of its
/// net (shared bases), and on carriers whose knots differ (separate calls).
#[test]
fn point_and_normal_match_the_separate_pre_change_calls() {
    let normal_of = |s: &NurbsSurface, u: f64, v: f64| {
        if cfg!(feature = "simd") {
            normal_bits(&DerivativeScratch::new().normal_from(s, u, v))
        } else {
            normal_bits(&s.normal_pre_contraction(u, v))
        }
    };
    let mut compared = 0usize;
    for (label, s) in corpus() {
        let chart = moved(&s, Vec3::new(-3.25, 0.5, 1e3));
        let us = parameters(s.domain_u(), s.knots_u());
        let vs = parameters(s.domain_v(), s.knots_v());
        for &u in &us {
            for &v in &vs {
                let point = point_bits(s.evaluate_pre_contraction(u, v));
                let (p, n) = s.point_and_normal(u, v);
                assert_eq!(
                    (point_bits(p), normal_bits(&n)),
                    (point, normal_of(&s, u, v)),
                    "{label} at ({u:e}, {v:e})"
                );
                let (p, n) = s.point_and_normal_with(&chart, u, v);
                assert_eq!(
                    (point_bits(p), normal_bits(&n)),
                    (point, normal_of(&chart, u, v)),
                    "{label} with a moved chart at ({u:e}, {v:e})"
                );
                compared += 1;
            }
        }
    }
    assert!(compared > 150_000, "compared only {compared}");

    // Degenerate patches: the perturbation fallback runs on the carrier.
    for (label, s) in degenerate_corpus() {
        let chart = moved(&s, Vec3::new(0.5, -2.0, 7.0));
        for &u in &parameters(s.domain_u(), s.knots_u()) {
            for &v in &parameters(s.domain_v(), s.knots_v()) {
                let point = point_bits(s.evaluate_pre_contraction(u, v));
                let (p, n) = s.point_and_normal(u, v);
                assert_eq!(
                    (point_bits(p), normal_bits(&n)),
                    (point, normal_of(&s, u, v)),
                    "{label} at ({u:e}, {v:e})"
                );
                let (p, n) = s.point_and_normal_with(&chart, u, v);
                assert_eq!(
                    (point_bits(p), normal_bits(&n)),
                    (point, normal_of(&chart, u, v)),
                    "{label} with a moved chart at ({u:e}, {v:e})"
                );
            }
        }
    }

    // Carriers that do not share the bases: other interior knots, another
    // degree, and a knot vector differing only in the sign of a zero.
    let a = surface(3, 2, Knots::Repeated, Knots::Uniform, Weights::Rational);
    let other_knots = surface(3, 2, Knots::Uniform, Knots::Uniform, Weights::Rational);
    let other_degree = surface(2, 2, Knots::Repeated, Knots::Uniform, Weights::Rational);
    let signed_zero = {
        let mut knots_u = a.knots_u().to_vec();
        for k in &mut knots_u {
            if k.to_bits() == 0.0f64.to_bits() {
                *k = -0.0;
            }
        }
        assert!(knots_u.iter().any(|k| k.to_bits() == (-0.0f64).to_bits()));
        NurbsSurface::new(
            3,
            2,
            knots_u,
            a.knots_v().to_vec(),
            a.control_points().to_vec(),
            a.weights().to_vec(),
        )
        .expect("signed-zero knots")
    };
    for carrier in [&other_knots, &other_degree, &signed_zero] {
        #[cfg(not(feature = "simd"))]
        assert!(!a.shares_bases_with(carrier));
        for &u in &parameters(a.domain_u(), a.knots_u()) {
            for &v in &parameters(a.domain_v(), a.knots_v()) {
                let (p, n) = a.point_and_normal_with(carrier, u, v);
                assert_eq!(
                    (point_bits(p), normal_bits(&n)),
                    (
                        point_bits(a.evaluate_pre_contraction(u, v)),
                        normal_of(carrier, u, v)
                    ),
                    "unshared carrier at ({u:e}, {v:e})"
                );
            }
        }
    }
}
