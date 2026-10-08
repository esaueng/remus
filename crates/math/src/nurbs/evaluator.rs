//! Cached evaluator for repeated NURBS surface evaluation.
//!
//! [`SurfaceEvaluator`] wraps a `&NurbsSurface` and lazily precomputes
//! [`PowerBasis1D`] coefficients for both u and v directions. All subsequent
//! evaluations use Horner's method with zero heap allocations.

use crate::nurbs::basis;
use crate::nurbs::power_basis::PowerBasis1D;
use crate::nurbs::surface::NurbsSurface;
use crate::vec::{Point3, Vec3};

/// Cached evaluator for a NURBS surface.
///
/// Wraps a `&NurbsSurface` and lazily precomputes `PowerBasis1D` for both
/// u and v directions on first use. All subsequent evaluations use Horner's
/// method with zero heap allocations.
pub struct SurfaceEvaluator<'a> {
    surface: &'a NurbsSurface,
    power_u: Option<PowerBasis1D>,
    power_v: Option<PowerBasis1D>,
    uniform_step_u: Option<f64>,
    uniform_step_v: Option<f64>,
}

impl<'a> SurfaceEvaluator<'a> {
    /// Create a new evaluator for the given surface.
    ///
    /// Detects uniform knot spacing for O(1) span lookup. Power-basis
    /// coefficients are computed lazily on first evaluation.
    #[must_use]
    pub fn new(surface: &'a NurbsSurface) -> Self {
        let uniform_step_u = basis::uniform_knot_step(surface.knots_u(), surface.degree_u());
        let uniform_step_v = basis::uniform_knot_step(surface.knots_v(), surface.degree_v());
        Self {
            surface,
            power_u: None,
            power_v: None,
            uniform_step_u,
            uniform_step_v,
        }
    }

    /// Ensure power-basis coefficients are computed.
    fn ensure_power_basis(&mut self) {
        if self.power_u.is_none() {
            self.power_u = Some(PowerBasis1D::from_knots(
                self.surface.knots_u(),
                self.surface.degree_u(),
            ));
        }
        if self.power_v.is_none() {
            self.power_v = Some(PowerBasis1D::from_knots(
                self.surface.knots_v(),
                self.surface.degree_v(),
            ));
        }
    }

    /// Find span in u direction, using uniform O(1) lookup if possible.
    fn find_span_u(&self, u: f64) -> usize {
        let n = self.surface.control_points().len();
        let pu = self.surface.degree_u();
        if let Some(step) = self.uniform_step_u {
            basis::find_span_uniform(n, pu, u, self.surface.knots_u(), step)
        } else {
            basis::find_span(n, pu, u, self.surface.knots_u())
        }
    }

    /// Find span in v direction, using uniform O(1) lookup if possible.
    fn find_span_v(&self, v: f64) -> usize {
        let n = self.surface.control_points()[0].len();
        let pv = self.surface.degree_v();
        if let Some(step) = self.uniform_step_v {
            basis::find_span_uniform(n, pv, v, self.surface.knots_v(), step)
        } else {
            basis::find_span(n, pv, v, self.surface.knots_v())
        }
    }

    /// Evaluate the surface position at parameters `(u, v)`.
    ///
    /// Uses precomputed power-basis coefficients with Horner evaluation,
    /// avoiding the O(p^2) Cox-de Boor recurrence on each call.
    #[allow(clippy::many_single_char_names)]
    pub fn point(&mut self, u: f64, v: f64) -> Point3 {
        self.ensure_power_basis();

        let pu = self.surface.degree_u();
        let pv = self.surface.degree_v();
        let u_domain = self.surface.domain_u();
        let v_domain = self.surface.domain_v();
        let u = u.clamp(u_domain.0, u_domain.1);
        let v = v.clamp(v_domain.0, v_domain.1);
        let span_u = self.find_span_u(u);
        let span_v = self.find_span_v(v);

        let mut nu_stack = [0.0_f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nu_heap;
        let nu: &mut [f64] = if pu <= basis::MAX_STACK_OUTPUT {
            &mut nu_stack[..=pu]
        } else {
            nu_heap = vec![0.0; pu + 1];
            &mut nu_heap
        };
        let mut nv_stack = [0.0_f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nv_heap;
        let nv: &mut [f64] = if pv <= basis::MAX_STACK_OUTPUT {
            &mut nv_stack[..=pv]
        } else {
            nv_heap = vec![0.0; pv + 1];
            &mut nv_heap
        };

        // SAFETY of indexing: power_u/power_v are guaranteed Some after ensure_power_basis.
        // Using if-let to satisfy no-panic lint.
        if let Some(ref pb_u) = self.power_u {
            pb_u.horner(span_u, u, nu);
        }
        if let Some(ref pb_v) = self.power_v {
            pb_v.horner(span_v, v, nv);
        }

        let cps = self.surface.control_points();
        let ws = self.surface.weights();
        // Normalize only the contributing control patch. A remote large
        // weight must not underflow every weight in the active patch, and a
        // query must not scan the entire surface's weight net.
        let weight_scale = nu
            .iter()
            .enumerate()
            .filter(|(_, value)| **value != 0.0)
            .flat_map(|(i, _)| {
                nv.iter()
                    .enumerate()
                    .filter(|(_, value)| **value != 0.0)
                    .map(move |(j, _)| ws[span_u - pu + i][span_v - pv + j])
            })
            .fold(0.0_f64, f64::max);
        debug_assert!(weight_scale.is_finite() && weight_scale > 0.0);

        // Use the same normalized weights for the local scale and sums.
        // Form the whole basis product before dividing by the scale: scaling
        // a row first can overflow even when its final contribution is small.
        let (scale, needs_exponent_scaling) = nu
            .iter()
            .enumerate()
            .take(pu + 1)
            .filter(|(_, value)| **value != 0.0)
            .flat_map(|(i, &nu_i)| {
                nv.iter()
                    .enumerate()
                    .take(pv + 1)
                    .filter(|(_, value)| **value != 0.0)
                    .map(move |(j, &nv_j)| {
                        let u_idx = span_u - pu + i;
                        let v_idx = span_v - pv + j;
                        let term = (nu_i * nv_j * (ws[u_idx][v_idx] / weight_scale)).abs();
                        (term, term == 0.0)
                    })
            })
            .fold((0.0_f64, false), |(scale, lost), (term, zero)| {
                (scale.max(term), lost || zero)
            });
        // Scale complete terms by their exponents only when a separately
        // formed weight ratio or basis product loses a nonzero contribution.
        if needs_exponent_scaling || !scale.is_finite() || scale == 0.0 {
            let sum = scaled_homogeneous_sum(cps, ws, span_u - pu, span_v - pv, nu, nv);
            return Point3::new(sum[0] / sum[3], sum[1] / sum[3], sum[2] / sum[3]);
        }
        let mut wx = 0.0;
        let mut wy = 0.0;
        let mut wz = 0.0;
        let mut ww = 0.0;

        for (i, &nu_i) in nu.iter().enumerate().take(pu + 1) {
            if nu_i == 0.0 {
                continue;
            }
            let u_idx = span_u - pu + i;
            for (j, &nv_j) in nv.iter().enumerate().take(pv + 1) {
                if nv_j == 0.0 {
                    continue;
                }
                let v_idx = span_v - pv + j;
                let pt = &cps[u_idx][v_idx];
                let w = ws[u_idx][v_idx] / weight_scale;
                let bw = (nu_i * nv_j * w) / scale;
                wx += bw * pt.x();
                wy += bw * pt.y();
                wz += bw * pt.z();
                ww += bw;
            }
        }

        debug_assert!(scale.is_finite() && scale > 0.0);
        debug_assert!(ww.is_finite() && ww > 0.0);
        Point3::new(wx / ww, wy / ww, wz / ww)
    }

    /// Evaluate the unit normal at parameters `(u, v)`.
    ///
    /// Uses precomputed power-basis coefficients with Horner evaluation for
    /// both basis values and first derivatives. If the cross product of the
    /// partial derivatives is degenerate, falls back to the surface's own
    /// `normal()` method, and ultimately to `(0, 0, 1)`.
    #[allow(clippy::many_single_char_names, clippy::too_many_lines)]
    pub fn normal(&mut self, u: f64, v: f64) -> Vec3 {
        self.ensure_power_basis();

        let pu = self.surface.degree_u();
        let pv = self.surface.degree_v();
        let u_domain = self.surface.domain_u();
        let v_domain = self.surface.domain_v();
        let u = u.clamp(u_domain.0, u_domain.1);
        let v = v.clamp(v_domain.0, v_domain.1);
        let span_u = self.find_span_u(u);
        let span_v = self.find_span_v(v);

        let mut nu_stack = [0.0_f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nu_heap;
        let nu: &mut [f64] = if pu <= basis::MAX_STACK_OUTPUT {
            &mut nu_stack[..=pu]
        } else {
            nu_heap = vec![0.0; pu + 1];
            &mut nu_heap
        };
        let mut dnu_stack = [0.0_f64; basis::MAX_STACK_OUTPUT + 1];
        let mut dnu_heap;
        let dnu: &mut [f64] = if pu <= basis::MAX_STACK_OUTPUT {
            &mut dnu_stack[..=pu]
        } else {
            dnu_heap = vec![0.0; pu + 1];
            &mut dnu_heap
        };
        let mut nv_stack = [0.0_f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nv_heap;
        let nv: &mut [f64] = if pv <= basis::MAX_STACK_OUTPUT {
            &mut nv_stack[..=pv]
        } else {
            nv_heap = vec![0.0; pv + 1];
            &mut nv_heap
        };
        let mut dnv_stack = [0.0_f64; basis::MAX_STACK_OUTPUT + 1];
        let mut dnv_heap;
        let dnv: &mut [f64] = if pv <= basis::MAX_STACK_OUTPUT {
            &mut dnv_stack[..=pv]
        } else {
            dnv_heap = vec![0.0; pv + 1];
            &mut dnv_heap
        };

        if let Some(ref pb_u) = self.power_u {
            pb_u.horner_with_derivs(span_u, u, nu, dnu);
        }
        if let Some(ref pb_v) = self.power_v {
            pb_v.horner_with_derivs(span_v, v, nv, dnv);
        }

        let cps = self.surface.control_points();
        let ws = self.surface.weights();
        let weight_scale = nu
            .iter()
            .zip(dnu.iter())
            .enumerate()
            .flat_map(|(i, (&a, &da))| {
                nv.iter()
                    .zip(dnv.iter())
                    .enumerate()
                    .filter_map(move |(j, (&b, &db))| {
                        (((a != 0.0 || da != 0.0) && b != 0.0) || (a != 0.0 && db != 0.0))
                            .then_some(ws[span_u - pu + i][span_v - pv + j])
                    })
            })
            .fold(0.0_f64, f64::max);

        // Compute homogeneous sums for position and partial derivatives.
        let mut s0 = [0.0_f64; 3]; // sum(nu * nv * w * P)
        let mut w0 = 0.0_f64; // sum(nu * nv * w)
        let mut su = [0.0_f64; 3]; // sum(dnu * nv * w * P)
        let mut wu = 0.0_f64; // sum(dnu * nv * w)
        let mut sv = [0.0_f64; 3]; // sum(nu * dnv * w * P)
        let mut wv = 0.0_f64; // sum(nu * dnv * w)

        for (i, (&nu_i, &dnu_i)) in nu.iter().zip(dnu.iter()).enumerate().take(pu + 1) {
            let u_idx = span_u - pu + i;
            for (j, (&nv_j, &dnv_j)) in nv.iter().zip(dnv.iter()).enumerate().take(pv + 1) {
                if !(((nu_i != 0.0 || dnu_i != 0.0) && nv_j != 0.0)
                    || (nu_i != 0.0 && dnv_j != 0.0))
                {
                    continue;
                }
                let v_idx = span_v - pv + j;
                let pt = &cps[u_idx][v_idx];
                let w = ws[u_idx][v_idx] / weight_scale;
                let px = pt.x();
                let py = pt.y();
                let pz = pt.z();

                let nn_w = nu_i * nv_j * w;
                s0[0] += nn_w * px;
                s0[1] += nn_w * py;
                s0[2] += nn_w * pz;
                w0 += nn_w;

                let dn_w = dnu_i * nv_j * w;
                su[0] += dn_w * px;
                su[1] += dn_w * py;
                su[2] += dn_w * pz;
                wu += dn_w;

                let nd_w = nu_i * dnv_j * w;
                sv[0] += nd_w * px;
                sv[1] += nd_w * py;
                sv[2] += nd_w * pz;
                wv += nd_w;
            }
        }

        // Apply rational quotient rule: d/du = (S_u - W_u * P) / W_0
        if !w0.is_finite() || w0 <= 0.0 {
            return scaled_normal(cps, ws, span_u - pu, span_v - pv, nu, nv, dnu, dnv);
        }

        let inv_w0 = 1.0 / w0;
        let px = s0[0] * inv_w0;
        let py = s0[1] * inv_w0;
        let pz = s0[2] * inv_w0;

        let du = Vec3::new(
            (su[0] - wu * px) * inv_w0,
            (su[1] - wu * py) * inv_w0,
            (su[2] - wu * pz) * inv_w0,
        );
        let dv = Vec3::new(
            (sv[0] - wv * px) * inv_w0,
            (sv[1] - wv * py) * inv_w0,
            (sv[2] - wv * pz) * inv_w0,
        );

        let cross = du.cross(dv);
        if cross.length_squared() > 1e-30 {
            cross
                .normalize()
                .unwrap_or_else(|_| Vec3::new(0.0, 0.0, 1.0))
        } else {
            // Fall back to the surface's own normal method.
            self.surface
                .normal(u, v)
                .unwrap_or_else(|_| Vec3::new(0.0, 0.0, 1.0))
        }
    }
}

// Split a nonzero finite number without losing subnormal significands.
fn binary_parts(value: f64) -> (f64, i32) {
    let (value, correction) = if value.abs() < f64::MIN_POSITIVE {
        (value * 18_014_398_509_481_984.0, -54)
    } else {
        (value, 0)
    };
    let bits = value.to_bits();
    let exponent = i32::try_from((bits >> 52) & 0x7ff).unwrap_or(0) - 1023 + correction;
    (
        f64::from_bits((bits & 0x800f_ffff_ffff_ffff) | (1023_u64 << 52)),
        exponent,
    )
}

fn weighted_parts(a: f64, b: f64, weight: f64) -> (f64, i32) {
    let (a, ea) = binary_parts(a);
    let (b, eb) = binary_parts(b);
    let (weight, ew) = binary_parts(weight);
    (a * b * weight, ea + eb + ew)
}

fn downscale(value: f64, exponent: i32) -> f64 {
    if exponent >= -1022 {
        value * f64::from_bits(u64::try_from(exponent + 1023).unwrap_or(0) << 52)
    } else if exponent >= -1074 {
        value * f64::from_bits(1_u64 << (exponent + 1074))
    } else if exponent >= -1077 {
        (value / f64::from(1_u32 << (-1074 - exponent))) * f64::from_bits(1)
    } else {
        0.0
    }
}

fn scaled_homogeneous_sum(
    cps: &[Vec<Point3>],
    weights: &[Vec<f64>],
    row: usize,
    column: usize,
    a: &[f64],
    b: &[f64],
) -> [f64; 4] {
    let mut exponent = i32::MIN;
    for (i, &a) in a.iter().enumerate().filter(|(_, x)| **x != 0.0) {
        for (j, &b) in b.iter().enumerate().filter(|(_, x)| **x != 0.0) {
            exponent = exponent.max(weighted_parts(a, b, weights[row + i][column + j]).1);
        }
    }
    let mut sum = [0.0; 4];
    for (i, &a) in a.iter().enumerate().filter(|(_, x)| **x != 0.0) {
        for (j, &b) in b.iter().enumerate().filter(|(_, x)| **x != 0.0) {
            let (mantissa, power) = weighted_parts(a, b, weights[row + i][column + j]);
            let term = downscale(mantissa, power - exponent);
            let point = cps[row + i][column + j];
            sum[0] += term * point.x();
            sum[1] += term * point.y();
            sum[2] += term * point.z();
            sum[3] += term;
        }
    }
    sum
}

#[allow(clippy::too_many_arguments)]
fn scaled_normal(
    cps: &[Vec<Point3>],
    weights: &[Vec<f64>],
    row: usize,
    column: usize,
    nu: &[f64],
    nv: &[f64],
    dnu: &[f64],
    dnv: &[f64],
) -> Vec3 {
    let p = scaled_homogeneous_sum(cps, weights, row, column, nu, nv);
    let u = scaled_homogeneous_sum(cps, weights, row, column, dnu, nv);
    let v = scaled_homogeneous_sum(cps, weights, row, column, nu, dnv);
    let direction = |s: [f64; 4]| {
        let d = Vec3::new(
            s[0] - s[3] * (p[0] / p[3]),
            s[1] - s[3] * (p[1] / p[3]),
            s[2] - s[3] * (p[2] / p[3]),
        );
        let max = d.x().abs().max(d.y().abs()).max(d.z().abs());
        if max > 0.0 {
            Vec3::new(d.x() / max, d.y() / max, d.z() / max)
        } else {
            d
        }
    };
    direction(u)
        .cross(direction(v))
        .normalize()
        .unwrap_or_else(|_| Vec3::new(0.0, 0.0, 1.0))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::vec::Point3;

    #[test]
    fn cached_normal_ignores_inactive_extreme_weights_at_patch_corners() {
        let cps: Vec<_> = (0..3)
            .map(|i| {
                (0..3)
                    .map(|j| {
                        let (x, y) = (f64::from(i), f64::from(j));
                        Point3::new(x, y, x + 2.0 * y)
                    })
                    .collect()
            })
            .collect();
        let expected = Vec3::new(-1.0, -2.0, 1.0).normalize().unwrap();
        for (u, v) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)] {
            let mut weights = vec![vec![1e-100; 3]; 3];
            weights[if u == 0.0 { 2 } else { 0 }][if v == 0.0 { 2 } else { 0 }] = 1e300;
            let surface = NurbsSurface::new(
                2,
                2,
                vec![0., 0., 0., 1., 1., 1.],
                vec![0., 0., 0., 1., 1., 1.],
                cps.clone(),
                weights,
            )
            .unwrap();
            let got = surface.evaluator().normal(u, v);
            assert!((got - expected).length() < 1e-12, "{got:?}");
        }
    }

    #[test]
    fn cached_point_preserves_complete_extreme_weighted_contributions() {
        let cps = vec![
            vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 2.)],
            vec![Point3::new(1., 0., 1.), Point3::new(1., 1., 3.)],
        ];
        for (weight, expected) in [
            (1e240, Point3::new(0.5, 0.5, 1.5)),
            (1e300, Point3::new(1., 1., 3.)),
        ] {
            let surface = NurbsSurface::new(
                1,
                1,
                vec![0., 0., 1., 1.],
                vec![0., 0., 1., 1.],
                cps.clone(),
                vec![vec![1e-100, 1e-100], vec![1e-100, weight]],
            )
            .unwrap();
            let got = surface.evaluator().point(1e-170, 1e-170);
            assert!((got - expected).length() < 1e-12, "{got:?}");
        }
    }

    fn bicubic_surface() -> NurbsSurface {
        let mut cps = Vec::new();
        let mut ws = Vec::new();
        for i in 0..4 {
            let mut row = Vec::new();
            let mut wrow = Vec::new();
            #[allow(clippy::cast_precision_loss)]
            for j in 0..4 {
                row.push(Point3::new(
                    j as f64,
                    i as f64,
                    ((i + j) as f64 * 0.5).sin(),
                ));
                wrow.push(1.0);
            }
            cps.push(row);
            ws.push(wrow);
        }
        NurbsSurface::new(
            3,
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            cps,
            ws,
        )
        .expect("valid bicubic surface")
    }

    #[test]
    fn surface_evaluator_matches_evaluate() {
        let surface = bicubic_surface();
        let mut eval = surface.evaluator();
        // Test on a grid
        for i in 0..=10 {
            for j in 0..=10 {
                #[allow(clippy::cast_precision_loss)]
                let u = i as f64 / 10.0;
                #[allow(clippy::cast_precision_loss)]
                let v = j as f64 / 10.0;
                let expected = surface.evaluate(u, v);
                let got = eval.point(u, v);
                assert!(
                    (expected.x() - got.x()).abs() < 1e-10
                        && (expected.y() - got.y()).abs() < 1e-10
                        && (expected.z() - got.z()).abs() < 1e-10,
                    "mismatch at ({u},{v}): {expected:?} vs {got:?}"
                );
            }
        }
    }

    #[test]
    fn surface_evaluator_normal_matches() {
        let surface = bicubic_surface();
        let mut eval = surface.evaluator();
        for i in 1..10 {
            for j in 1..10 {
                #[allow(clippy::cast_precision_loss)]
                let u = i as f64 / 10.0;
                #[allow(clippy::cast_precision_loss)]
                let v = j as f64 / 10.0;
                let expected = surface.normal(u, v).expect("non-degenerate");
                let got = eval.normal(u, v);
                let dot = expected.x() * got.x() + expected.y() * got.y() + expected.z() * got.z();
                assert!(
                    (dot - 1.0).abs() < 1e-8,
                    "normal mismatch at ({u},{v}): dot={dot}"
                );
            }
        }
    }
}
