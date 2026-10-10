//! Independent, rounding-inclusive Bernstein certificates for NURBS reuse.
//!
//! Supplied finite `f64` values describe exact real inputs. Every arithmetic
//! operation used to extract Bernstein coefficients is outward rounded; no
//! floating-point knot insertion or sampled evaluation is treated as exact.
//! On each common knot span, rational cross multiplication bounds the complete
//! position difference, and optionally its derivative in the input parameter.

use super::reuse::{ReuseBudget, ReuseError};
use super::{NurbsCurve, NurbsSurface};
use crate::vec::Point3;

const MAX_DEGREE: usize = 12;

fn unavailable(reason: &'static str) -> ReuseError {
    ReuseError::BoundUnavailable { reason }
}

#[derive(Clone, Copy, Debug)]
struct Interval {
    lo: f64,
    hi: f64,
}

impl Interval {
    const ZERO: Self = Self::point(0.0);
    const ONE: Self = Self::point(1.0);

    const fn point(value: f64) -> Self {
        Self {
            lo: value,
            hi: value,
        }
    }

    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn rounded(lo: f64, hi: f64) -> Result<Self, ReuseError> {
        let result = Self {
            lo: lo.next_down(),
            hi: hi.next_up(),
        };
        if result.lo.is_finite() && result.hi.is_finite() {
            Ok(result)
        } else {
            Err(unavailable("interval arithmetic overflow"))
        }
    }

    fn is_zero(self) -> bool {
        self.lo == 0.0 && self.hi == 0.0
    }

    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn add(self, other: Self) -> Result<Self, ReuseError> {
        if self.is_zero() {
            return Ok(other);
        }
        if other.is_zero() {
            return Ok(self);
        }
        Self::rounded(self.lo + other.lo, self.hi + other.hi)
    }

    // Singleton equality here is an exact real-arithmetic identity proof.
    #[allow(clippy::float_cmp)]
    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn sub(self, other: Self) -> Result<Self, ReuseError> {
        if other.is_zero() {
            return Ok(self);
        }
        if self.lo == self.hi && other.lo == other.hi && self.lo == other.lo {
            return Ok(Self::ZERO);
        }
        Self::rounded(self.lo - other.hi, self.hi - other.lo)
    }

    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn mul(self, other: Self) -> Result<Self, ReuseError> {
        if self.is_zero() || other.is_zero() {
            return Ok(Self::ZERO);
        }
        let products = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi,
        ];
        Self::rounded(
            products.into_iter().fold(f64::INFINITY, f64::min),
            products.into_iter().fold(f64::NEG_INFINITY, f64::max),
        )
    }

    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn div(self, other: Self) -> Result<Self, ReuseError> {
        if other.lo <= 0.0 && other.hi >= 0.0 {
            return Err(unavailable(
                "interval denominator is not separated from zero",
            ));
        }
        if self.is_zero() {
            return Ok(Self::ZERO);
        }
        // Direct quotient endpoints avoid overflowing an intermediate
        // reciprocal when both numerator and denominator are subnormal.
        let quotients = [
            self.lo / other.lo,
            self.lo / other.hi,
            self.hi / other.lo,
            self.hi / other.hi,
        ];
        Self::rounded(
            quotients.into_iter().fold(f64::INFINITY, f64::min),
            quotients.into_iter().fold(f64::NEG_INFINITY, f64::max),
        )
    }

    fn abs_upper(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }
}

/// Bounds in position and in first derivative with respect to the input parameter.
pub(super) struct CurveDifferenceBound {
    pub(super) position: f64,
    pub(super) derivative: f64,
}

pub(super) fn validate_curve_domain(curve: &NurbsCurve) -> Result<(), ReuseError> {
    validate_knots(curve.degree(), curve.knots(), curve.control_points().len())?;
    if curve.weights().len() != curve.control_points().len()
        || curve
            .control_points()
            .iter()
            .any(|p| p.0.iter().any(|v| !v.is_finite()))
        || curve.weights().iter().any(|w| !w.is_finite() || *w <= 0.0)
    {
        return Err(ReuseError::Unsupported {
            reason: "finite control points and strictly positive finite weights required",
        });
    }
    Ok(())
}

pub(super) fn validate_surface_domain(surface: &NurbsSurface) -> Result<(), ReuseError> {
    let points = surface.control_points();
    let Some(first) = points.first() else {
        return Err(ReuseError::Unsupported {
            reason: "empty surface control net",
        });
    };
    validate_knots(surface.degree_u(), surface.knots_u(), points.len())?;
    validate_knots(surface.degree_v(), surface.knots_v(), first.len())?;
    if points.iter().any(|row| row.len() != first.len())
        || surface.weights().len() != points.len()
        || surface.weights().iter().any(|row| row.len() != first.len())
        || points
            .iter()
            .flatten()
            .any(|p| p.0.iter().any(|v| !v.is_finite()))
        || surface
            .weights()
            .iter()
            .flatten()
            .any(|w| !w.is_finite() || *w <= 0.0)
    {
        return Err(ReuseError::Unsupported {
            reason: "rectangular finite control net and strictly positive finite weights required",
        });
    }
    Ok(())
}

// Certification needs exact knot ordering, multiplicity, and endpoint identity;
// tolerance comparisons would certify a different spline than the input.
#[allow(clippy::float_cmp)]
fn validate_knots(degree: usize, knots: &[f64], points: usize) -> Result<(), ReuseError> {
    if degree == 0 || degree > MAX_DEGREE || degree >= points {
        return Err(ReuseError::Unsupported {
            reason: "supported degrees are 1 through 12",
        });
    }
    if points.checked_add(degree + 1) != Some(knots.len())
        || knots.iter().any(|k| !k.is_finite())
        || knots.windows(2).any(|k| k[0] > k[1])
    {
        return Err(ReuseError::Unsupported {
            reason: "exactly nondecreasing finite knot vector required",
        });
    }
    let start = knots[degree];
    let end = knots[points];
    if start >= end
        || knots[..=degree].iter().any(|k| *k != start)
        || knots[points..].iter().any(|k| *k != end)
    {
        return Err(ReuseError::Unsupported {
            reason: "clamped nonempty parameter domain required",
        });
    }
    let mut index = degree + 1;
    while index < points {
        let value = knots[index];
        let mut next = index + 1;
        while next < points && knots[next] == value {
            next += 1;
        }
        if next - index > degree || value <= start || value >= end {
            return Err(ReuseError::Unsupported {
                reason: "interior knot multiplicity must not exceed the degree",
            });
        }
        index = next;
    }
    Ok(())
}

fn charge_curve(curve: &NurbsCurve, budget: &mut ReuseBudget<'_>) -> Result<(), ReuseError> {
    budget.spend(curve.knots().len())?;
    budget.spend(curve.control_points().len())?;
    budget.spend(curve.weights().len())?;
    validate_curve_domain(curve)
}

fn charge_surface(surface: &NurbsSurface, budget: &mut ReuseBudget<'_>) -> Result<(), ReuseError> {
    budget.spend(surface.knots_u().len())?;
    budget.spend(surface.knots_v().len())?;
    budget.spend(surface.control_points().len())?;
    for row in surface.control_points() {
        budget.spend(row.len())?;
    }
    for row in surface.weights() {
        budget.spend(row.len())?;
    }
    validate_surface_domain(surface)
}

fn common_breaks(
    first: &[f64],
    second: &[f64],
    range: (f64, f64),
    budget: &mut ReuseBudget<'_>,
) -> Result<Vec<f64>, ReuseError> {
    budget.spend(first.len())?;
    budget.spend(second.len())?;
    budget.spend(2)?;
    let mut values = Vec::with_capacity(first.len() + second.len() + 2);
    values.push(range.0);
    // Both vectors were validated as exactly sorted. Merge in linear work
    // rather than paying unaccounted sorting comparisons on large inputs.
    let (mut left, mut right) = (0, 0);
    let mut previous = range.0;
    while left < first.len() || right < second.len() {
        let value = if right == second.len() || (left < first.len() && first[left] <= second[right])
        {
            let value = first[left];
            left += 1;
            value
        } else {
            let value = second[right];
            right += 1;
            value
        };
        if value > previous && value < range.1 {
            values.push(value);
            previous = value;
        }
    }
    values.push(range.1);
    Ok(values)
}

/// Interval Bernstein coefficients for every active B-spline basis on a span.
/// Cox--de Boor's affine factors are multiplied in Bernstein form throughout.
fn basis_on_span(
    degree: usize,
    knots: &[f64],
    range: (f64, f64),
    budget: &mut ReuseBudget<'_>,
) -> Result<(usize, Vec<Vec<Interval>>), ReuseError> {
    budget.spend(knots.len())?;
    let span = knots
        .windows(2)
        .position(|k| k[0] <= range.0 && k[1] >= range.1 && k[0] < k[1])
        .ok_or_else(|| unavailable("comparison interval crosses an unpartitioned knot"))?;
    budget.spend(1)?;
    let mut previous = vec![vec![Interval::ONE]];
    for d in 1..=degree {
        budget.spend((d + 1) * (d + 1))?;
        let mut current = vec![vec![Interval::ZERO; d + 1]; d + 1];
        let first = span - d;
        for (local, coefficients) in current.iter_mut().enumerate() {
            let index = first + local;
            let left = knots[index + d] - knots[index];
            if local > 0 && left > 0.0 {
                let denominator =
                    Interval::point(knots[index + d]).sub(Interval::point(knots[index]))?;
                let a = Interval::point(range.0)
                    .sub(Interval::point(knots[index]))?
                    .div(denominator)?;
                let b = Interval::point(range.1)
                    .sub(Interval::point(knots[index]))?
                    .div(denominator)?;
                multiply_affine_add(coefficients, &previous[local - 1], a, b)?;
            }
            let right = knots[index + d + 1] - knots[index + 1];
            if local < d && right > 0.0 {
                let denominator =
                    Interval::point(knots[index + d + 1]).sub(Interval::point(knots[index + 1]))?;
                let a = Interval::point(knots[index + d + 1])
                    .sub(Interval::point(range.0))?
                    .div(denominator)?;
                let b = Interval::point(knots[index + d + 1])
                    .sub(Interval::point(range.1))?
                    .div(denominator)?;
                multiply_affine_add(coefficients, &previous[local], a, b)?;
            }
        }
        previous = current;
    }
    Ok((span - degree, previous))
}

#[allow(clippy::cast_precision_loss)]
#[cfg_attr(target_arch = "wasm32", inline(never))]
fn multiply_affine_add(
    output: &mut [Interval],
    input: &[Interval],
    start: Interval,
    end: Interval,
) -> Result<(), ReuseError> {
    let degree = input.len();
    for (index, value) in output.iter_mut().enumerate() {
        if index < degree {
            let factor =
                Interval::point((degree - index) as f64).div(Interval::point(degree as f64))?;
            *value = value.add(input[index].mul(start)?.mul(factor)?)?;
        }
        if index > 0 {
            let factor = Interval::point(index as f64).div(Interval::point(degree as f64))?;
            *value = value.add(input[index - 1].mul(end)?.mul(factor)?)?;
        }
    }
    Ok(())
}

#[derive(Clone)]
struct Polynomial {
    du: usize,
    dv: usize,
    coefficients: Vec<Interval>,
}

impl Polynomial {
    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn zero(du: usize, dv: usize, budget: &mut ReuseBudget<'_>) -> Result<Self, ReuseError> {
        budget.spend((du + 1) * (dv + 1))?;
        Ok(Self {
            du,
            dv,
            coefficients: vec![Interval::ZERO; (du + 1) * (dv + 1)],
        })
    }

    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn sub(&self, other: &Self, budget: &mut ReuseBudget<'_>) -> Result<Self, ReuseError> {
        if self.du != other.du || self.dv != other.dv {
            return Err(unavailable("incompatible Bernstein polynomial degrees"));
        }
        let mut output = Self::zero(self.du, self.dv, budget)?;
        for ((value, a), b) in output
            .coefficients
            .iter_mut()
            .zip(&self.coefficients)
            .zip(&other.coefficients)
        {
            *value = a.sub(*b)?;
        }
        Ok(output)
    }

    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn product(&self, other: &Self, budget: &mut ReuseBudget<'_>) -> Result<Self, ReuseError> {
        // All degrees stay below 48 under MAX_DEGREE; binomial integers are
        // then exactly representable as f64, and ratios are interval divisions.
        if self.du + other.du > 48 || self.dv + other.dv > 48 {
            return Err(unavailable(
                "Bernstein product degree exceeds certificate limit",
            ));
        }
        let mut output = Self::zero(self.du + other.du, self.dv + other.dv, budget)?;
        for au in 0..=self.du {
            budget.spend((self.dv + 1) * other.coefficients.len())?;
            for av in 0..=self.dv {
                for bu in 0..=other.du {
                    let fu = product_factor(self.du, au, other.du, bu)?;
                    for bv in 0..=other.dv {
                        let fv = product_factor(self.dv, av, other.dv, bv)?;
                        let index = (au + bu) * (output.dv + 1) + av + bv;
                        let term = self.coefficients[au * (self.dv + 1) + av]
                            .mul(other.coefficients[bu * (other.dv + 1) + bv])?
                            .mul(fu)?
                            .mul(fv)?;
                        output.coefficients[index] = output.coefficients[index].add(term)?;
                    }
                }
            }
        }
        Ok(output)
    }

    #[allow(clippy::cast_precision_loss)]
    #[cfg_attr(target_arch = "wasm32", inline(never))]
    fn derivative(
        &self,
        range: (f64, f64),
        budget: &mut ReuseBudget<'_>,
    ) -> Result<Self, ReuseError> {
        if self.du == 0 || self.dv != 0 {
            return Err(unavailable("invalid univariate derivative polynomial"));
        }
        let factor = Interval::point(self.du as f64)
            .div(Interval::point(range.1).sub(Interval::point(range.0))?)?;
        let mut output = Self::zero(self.du - 1, 0, budget)?;
        for (index, coefficient) in output.coefficients.iter_mut().enumerate() {
            *coefficient = self.coefficients[index + 1]
                .sub(self.coefficients[index])?
                .mul(factor)?;
        }
        Ok(output)
    }

    fn positive_lower(&self) -> Result<f64, ReuseError> {
        let lower = self
            .coefficients
            .iter()
            .map(|c| c.lo)
            .fold(f64::INFINITY, f64::min);
        if lower.is_finite() && lower > 0.0 {
            Ok(lower)
        } else {
            Err(unavailable(
                "Bernstein denominator lower bound is not strictly positive",
            ))
        }
    }

    fn abs_upper(&self) -> f64 {
        self.coefficients
            .iter()
            .map(|c| c.abs_upper())
            .fold(0.0, f64::max)
    }
}

#[allow(clippy::cast_precision_loss)]
fn binomial(n: usize, k: usize) -> f64 {
    let mut value = 1_u64;
    for index in 0..k.min(n - k) {
        value = value * (n - index) as u64 / (index + 1) as u64;
    }
    value as f64
}

#[cfg_attr(target_arch = "wasm32", inline(never))]
fn product_factor(m: usize, i: usize, n: usize, j: usize) -> Result<Interval, ReuseError> {
    Interval::point(binomial(m, i))
        .mul(Interval::point(binomial(n, j)))?
        .div(Interval::point(binomial(m + n, i + j)))
}

fn homogeneous_curve(
    curve: &NurbsCurve,
    range: (f64, f64),
    origin: Point3,
    budget: &mut ReuseBudget<'_>,
) -> Result<[Polynomial; 4], ReuseError> {
    let (first, basis) = basis_on_span(curve.degree(), curve.knots(), range, budget)?;
    let mut output = [
        Polynomial::zero(curve.degree(), 0, budget)?,
        Polynomial::zero(curve.degree(), 0, budget)?,
        Polynomial::zero(curve.degree(), 0, budget)?,
        Polynomial::zero(curve.degree(), 0, budget)?,
    ];
    // Common weight normalization is performed in interval arithmetic, so its
    // rounding error is included rather than silently changing the input.
    budget.spend(curve.weights().len())?;
    let maximum = curve.weights().iter().copied().fold(0.0, f64::max);
    for (local, coefficients) in basis.iter().enumerate() {
        budget.spend(4 * coefficients.len())?;
        let weight =
            Interval::point(curve.weights()[first + local]).div(Interval::point(maximum))?;
        if weight.lo <= 0.0 {
            return Err(unavailable("weight normalization lost strict positivity"));
        }
        let point = curve.control_points()[first + local];
        for axis in 0..4 {
            let coordinate = if axis == 3 {
                Interval::ONE
            } else {
                Interval::point(point.0[axis]).sub(Interval::point(origin.0[axis]))?
            };
            for (result, basis_value) in output[axis].coefficients.iter_mut().zip(coefficients) {
                *result = result.add(basis_value.mul(weight)?.mul(coordinate)?)?;
            }
        }
    }
    Ok(output)
}

fn homogeneous_surface(
    surface: &NurbsSurface,
    urange: (f64, f64),
    vrange: (f64, f64),
    origin: Point3,
    budget: &mut ReuseBudget<'_>,
) -> Result<[Polynomial; 4], ReuseError> {
    let (first_u, ubasis) = basis_on_span(surface.degree_u(), surface.knots_u(), urange, budget)?;
    let (first_v, vbasis) = basis_on_span(surface.degree_v(), surface.knots_v(), vrange, budget)?;
    let du = surface.degree_u();
    let dv = surface.degree_v();
    let mut output = [
        Polynomial::zero(du, dv, budget)?,
        Polynomial::zero(du, dv, budget)?,
        Polynomial::zero(du, dv, budget)?,
        Polynomial::zero(du, dv, budget)?,
    ];
    for row in surface.weights() {
        budget.spend(row.len())?;
    }
    let maximum = surface
        .weights()
        .iter()
        .flatten()
        .copied()
        .fold(0.0, f64::max);
    for (u, uc) in ubasis.iter().enumerate() {
        for (v, vc) in vbasis.iter().enumerate() {
            budget.spend(4 * (du + 1) * (dv + 1))?;
            let weight = Interval::point(surface.weights()[first_u + u][first_v + v])
                .div(Interval::point(maximum))?;
            if weight.lo <= 0.0 {
                return Err(unavailable("weight normalization lost strict positivity"));
            }
            let point = surface.control_points()[first_u + u][first_v + v];
            for axis in 0..4 {
                let coordinate = if axis == 3 {
                    Interval::ONE
                } else {
                    Interval::point(point.0[axis]).sub(Interval::point(origin.0[axis]))?
                };
                for (cu, bu) in uc.iter().enumerate() {
                    for (cv, bv) in vc.iter().enumerate() {
                        let result = &mut output[axis].coefficients[cu * (dv + 1) + cv];
                        *result = result.add(bu.mul(*bv)?.mul(weight)?.mul(coordinate)?)?;
                    }
                }
            }
        }
    }
    Ok(output)
}

fn vector_quotient_bound(
    numerators: &[Polynomial; 3],
    denominator: &Polynomial,
) -> Result<f64, ReuseError> {
    let lower = denominator.positive_lower()?;
    let mut square = Interval::ZERO;
    for numerator in numerators {
        let component = Interval::point(numerator.abs_upper())
            .div(Interval::point(lower))?
            .hi;
        square = square.add(Interval::point(component).mul(Interval::point(component))?)?;
    }
    let bound = square.hi.sqrt().next_up();
    if bound.is_finite() {
        Ok(bound)
    } else {
        Err(unavailable("Euclidean norm bound overflow"))
    }
}

fn position_bound(
    a: &[Polynomial; 4],
    b: &[Polynomial; 4],
    budget: &mut ReuseBudget<'_>,
) -> Result<f64, ReuseError> {
    let denominator = a[3].product(&b[3], budget)?;
    let cross = |axis: usize, budget: &mut ReuseBudget<'_>| -> Result<Polynomial, ReuseError> {
        a[axis]
            .product(&b[3], budget)?
            .sub(&b[axis].product(&a[3], budget)?, budget)
    };
    vector_quotient_bound(
        &[cross(0, budget)?, cross(1, budget)?, cross(2, budget)?],
        &denominator,
    )
}

fn derivative_bound(
    a: &[Polynomial; 4],
    b: &[Polynomial; 4],
    range: (f64, f64),
    budget: &mut ReuseBudget<'_>,
) -> Result<f64, ReuseError> {
    let aw2 = a[3].product(&a[3], budget)?;
    let bw2 = b[3].product(&b[3], budget)?;
    let denominator = aw2.product(&bw2, budget)?;
    let adw = a[3].derivative(range, budget)?;
    let bdw = b[3].derivative(range, budget)?;
    let cross = |axis: usize, budget: &mut ReuseBudget<'_>| -> Result<Polynomial, ReuseError> {
        let an = a[axis]
            .derivative(range, budget)?
            .product(&a[3], budget)?
            .sub(&a[axis].product(&adw, budget)?, budget)?;
        let bn = b[axis]
            .derivative(range, budget)?
            .product(&b[3], budget)?
            .sub(&b[axis].product(&bdw, budget)?, budget)?;
        an.product(&bw2, budget)?
            .sub(&bn.product(&aw2, budget)?, budget)
    };
    vector_quotient_bound(
        &[cross(0, budget)?, cross(1, budget)?, cross(2, budget)?],
        &denominator,
    )
}

pub(super) fn curve_deviation_bound(
    original: &NurbsCurve,
    candidate: &NurbsCurve,
    budget: &mut ReuseBudget<'_>,
) -> Result<f64, ReuseError> {
    charge_curve(original, budget)?;
    charge_curve(candidate, budget)?;
    if original.domain() != candidate.domain() {
        return Err(ReuseError::Unsupported {
            reason: "comparison requires identical parameter domains",
        });
    }
    if original == candidate {
        return Ok(0.0);
    }
    let breaks = common_breaks(
        original.knots(),
        candidate.knots(),
        original.domain(),
        budget,
    )?;
    let origin = original.control_points()[0];
    let mut bound = 0.0_f64;
    for span in breaks.windows(2) {
        let range = (span[0], span[1]);
        let a = homogeneous_curve(original, range, origin, budget)?;
        let b = homogeneous_curve(candidate, range, origin, budget)?;
        bound = bound.max(position_bound(&a, &b, budget)?);
    }
    Ok(bound)
}

pub(super) fn curve_difference_on_interval(
    original: &NurbsCurve,
    candidate: &NurbsCurve,
    range: (f64, f64),
    budget: &mut ReuseBudget<'_>,
) -> Result<CurveDifferenceBound, ReuseError> {
    charge_curve(original, budget)?;
    charge_curve(candidate, budget)?;
    if !range.0.is_finite()
        || !range.1.is_finite()
        || range.0 >= range.1
        || range.0 < original.domain().0
        || range.1 > original.domain().1
        || range.0 < candidate.domain().0
        || range.1 > candidate.domain().1
    {
        return Err(ReuseError::InvalidOptions {
            reason: "comparison range must lie inside both nonempty domains",
        });
    }
    if original == candidate {
        return Ok(CurveDifferenceBound {
            position: 0.0,
            derivative: 0.0,
        });
    }
    let breaks = common_breaks(original.knots(), candidate.knots(), range, budget)?;
    let origin = original.control_points()[0];
    let mut result = CurveDifferenceBound {
        position: 0.0,
        derivative: 0.0,
    };
    for span in breaks.windows(2) {
        let span_range = (span[0], span[1]);
        let a = homogeneous_curve(original, span_range, origin, budget)?;
        let b = homogeneous_curve(candidate, span_range, origin, budget)?;
        result.position = result.position.max(position_bound(&a, &b, budget)?);
        result.derivative = result
            .derivative
            .max(derivative_bound(&a, &b, span_range, budget)?);
    }
    Ok(result)
}

pub(super) fn surface_deviation_bound(
    original: &NurbsSurface,
    candidate: &NurbsSurface,
    budget: &mut ReuseBudget<'_>,
) -> Result<f64, ReuseError> {
    charge_surface(original, budget)?;
    charge_surface(candidate, budget)?;
    if original.domain_u() != candidate.domain_u() || original.domain_v() != candidate.domain_v() {
        return Err(ReuseError::Unsupported {
            reason: "comparison requires identical parameter domains",
        });
    }
    if original == candidate {
        return Ok(0.0);
    }
    let ubreaks = common_breaks(
        original.knots_u(),
        candidate.knots_u(),
        original.domain_u(),
        budget,
    )?;
    let vbreaks = common_breaks(
        original.knots_v(),
        candidate.knots_v(),
        original.domain_v(),
        budget,
    )?;
    let origin = original.control_points()[0][0];
    let mut bound = 0.0_f64;
    for u in ubreaks.windows(2) {
        for v in vbreaks.windows(2) {
            let a = homogeneous_surface(original, (u[0], u[1]), (v[0], v[1]), origin, budget)?;
            let b = homogeneous_surface(candidate, (u[0], u[1]), (v[0], v[1]), origin, budget)?;
            bound = bound.max(position_bound(&a, &b, budget)?);
        }
    }
    Ok(bound)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

    use super::*;
    use crate::context::{CancellationToken, OperationContext};

    fn bezier(degree: usize, x: &[f64], weights: &[f64]) -> NurbsCurve {
        let mut knots = vec![0.0; degree + 1];
        knots.extend(vec![1.0; degree + 1]);
        NurbsCurve::new(
            degree,
            knots,
            x.iter().map(|v| Point3::new(*v, 0.0, 0.0)).collect(),
            weights.to_vec(),
        )
        .unwrap()
    }

    fn assert_contains(interval: Interval, exact: f64) {
        assert!(
            interval.lo <= exact && interval.hi >= exact,
            "{interval:?} does not contain {exact}"
        );
    }

    #[test]
    fn interval_arithmetic_encloses_underflow_and_cancellation() {
        let smallest = f64::from_bits(1);
        assert_contains(
            Interval::point(smallest).mul(Interval::point(0.5)).unwrap(),
            0.0,
        );
        let scaled = Interval::point(smallest).mul(Interval::point(0.5)).unwrap();
        assert!(scaled.lo <= 0.0 && scaled.hi >= smallest);
        assert!(
            Interval::point(1.0)
                .sub(Interval::point(1.0))
                .unwrap()
                .is_zero()
        );
        assert!(Interval::point(f64::MAX).mul(Interval::point(2.0)).is_err());
        assert!(Interval::ONE.div(Interval { lo: -1.0, hi: 1.0 }).is_err());
        assert_contains(
            Interval::point(smallest)
                .div(Interval::point(smallest))
                .unwrap(),
            1.0,
        );
    }

    #[test]
    fn cox_de_boor_coefficients_match_independent_quadratic_oracle() {
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(100_000, &context).unwrap();
        let knots = [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0];
        let (start, first) = basis_on_span(2, &knots, (0.0, 0.5), &mut budget).unwrap();
        assert_eq!(start, 0);
        for (computed, expected) in
            first
                .iter()
                .zip([[1.0, 0.0, 0.0], [0.0, 1.0, 0.5], [0.0, 0.0, 0.5]])
        {
            for (interval, exact) in computed.iter().zip(expected) {
                assert_contains(*interval, exact);
            }
        }
        let (start, last) = basis_on_span(2, &knots, (0.5, 1.0), &mut budget).unwrap();
        assert_eq!(start, 1);
        for (computed, expected) in
            last.iter()
                .zip([[0.5, 0.0, 0.0], [0.5, 1.0, 0.0], [0.0, 0.0, 1.0]])
        {
            for (interval, exact) in computed.iter().zip(expected) {
                assert_contains(*interval, exact);
            }
        }
    }

    #[test]
    fn subspan_coefficients_are_not_sampled_endpoint_interpolation() {
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(100_000, &context).unwrap();
        let (_, basis) = basis_on_span(
            2,
            &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            (0.25, 0.75),
            &mut budget,
        )
        .unwrap();
        // B_1(t) = 2t(1-t): on this subspan its middle Bernstein
        // coefficient is 5/8, although both endpoints evaluate to 3/8.
        for (computed, exact) in basis[1].iter().zip([0.375, 0.625, 0.375]) {
            assert_contains(*computed, exact);
        }
    }

    #[test]
    fn position_and_parameter_derivative_use_different_degree_cross_products() {
        let source = bezier(4, &[0.0, 0.0, 0.0, 0.0, 1.0], &[1.0; 5]);
        let candidate = bezier(1, &[0.0, 0.0], &[1.0; 2]);
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
        let bounds =
            curve_difference_on_interval(&source, &candidate, (0.0, 1.0), &mut budget).unwrap();
        assert!((1.0..1.000_001).contains(&bounds.position));
        assert!((4.0..4.000_001).contains(&bounds.derivative));
    }

    #[test]
    fn rational_line_certificate_covers_analytic_interior_maximum_and_derivative() {
        let source = bezier(1, &[0.0, 1.0], &[1.0, 2.0]);
        let candidate = bezier(1, &[0.0, 1.0], &[1.0, 1.0]);
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
        let bounds =
            curve_difference_on_interval(&source, &candidate, (0.0, 1.0), &mut budget).unwrap();
        // Exact position difference is t(1-t)/(1+t), with maximum
        // 3-2sqrt(2). Exact derivative difference reaches 1 at t=0.
        assert!(bounds.position >= 3.0 - 2.0 * 2.0_f64.sqrt());
        assert!(bounds.position < 0.500_001);
        assert!(bounds.derivative >= 1.0);
        for sample in 0..=1000 {
            let t = sample as f64 / 1000.0;
            let position = t * (1.0 - t) / (1.0 + t);
            let derivative = (2.0 / (1.0 + t).powi(2) - 1.0).abs();
            assert!(position <= bounds.position && derivative <= bounds.derivative);
        }
    }

    #[test]
    fn unsampled_quartic_bump_cannot_receive_zero_bound() {
        let source = bezier(4, &[0.0; 5], &[1.0; 5]);
        let candidate = bezier(4, &[0.0, 0.0, 1.0, 0.0, 0.0], &[1.0; 5]);
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
        let bound = curve_deviation_bound(&source, &candidate, &mut budget).unwrap();
        assert!(bound >= 0.375);
        assert!(bound.is_finite());
    }

    #[test]
    fn curve_refinement_roundoff_is_covered_and_disclosed() {
        let source = bezier(2, &[0.0, 0.3, 1.0], &[1.0, 0.7, 1.0]);
        let refined = super::super::knot_ops::curve_knot_insert(&source, 0.37, 1).unwrap();
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
        let bound = curve_deviation_bound(&source, &refined, &mut budget).unwrap();
        assert!(bound > 0.0 && bound < 1e-10);
        for sample in 0..=1000 {
            let t = sample as f64 / 1000.0;
            assert!((source.evaluate(t) - refined.evaluate(t)).length() <= bound);
        }
    }

    fn plane(z: f64, weight_scale: f64) -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, z), Point3::new(0.0, 1.0, z)],
                vec![Point3::new(1.0, 0.0, z), Point3::new(1.0, 1.0, z)],
            ],
            vec![vec![weight_scale; 2]; 2],
        )
        .unwrap()
    }

    #[test]
    fn tensor_surface_bound_covers_constant_normal_displacement() {
        let source = plane(0.0, 1.0);
        let candidate = plane(3.0, 1e-280);
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
        let bound = surface_deviation_bound(&source, &candidate, &mut budget).unwrap();
        assert!((3.0..3.000_001).contains(&bound));
    }

    #[test]
    fn surface_refinement_certificate_includes_floating_roundoff() {
        let source = plane(7.0, 1.0);
        let refined = super::super::knot_ops::surface_knot_insert_u(&source, 0.29, 1).unwrap();
        let refined = super::super::knot_ops::surface_knot_insert_v(&refined, 0.71, 1).unwrap();
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
        let bound = surface_deviation_bound(&source, &refined, &mut budget).unwrap();
        assert!(bound > 0.0 && bound < 1e-10);
        for (u, v) in [(0.0, 0.0), (0.13, 0.91), (0.29, 0.71), (1.0, 1.0)] {
            assert!((source.evaluate(u, v) - refined.evaluate(u, v)).length() <= bound);
        }
    }

    #[test]
    fn strict_knot_domain_declines_wobbles_unclamped_and_discontinuous_inputs() {
        let points = vec![Point3::new(0.0, 0.0, 0.0); 4];
        // Constructors allow representation wobbles; certification does not.
        let wobble = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 0.5, 0.5_f64.next_down(), 1.0, 1.0],
            points.clone(),
            vec![1.0; 4],
        )
        .unwrap();
        assert!(matches!(
            validate_curve_domain(&wobble),
            Err(ReuseError::Unsupported { .. })
        ));
        let jump =
            NurbsCurve::new(1, vec![0.0, 0.0, 0.5, 0.5, 1.0, 1.0], points, vec![1.0; 4]).unwrap();
        assert!(matches!(
            validate_curve_domain(&jump),
            Err(ReuseError::Unsupported { .. })
        ));
        let unclamped = NurbsCurve::new(
            1,
            vec![-1.0, 0.0, 1.0, 2.0],
            vec![Point3::new(0.0, 0.0, 0.0); 2],
            vec![1.0; 2],
        )
        .unwrap();
        assert!(matches!(
            validate_curve_domain(&unclamped),
            Err(ReuseError::Unsupported { .. })
        ));
    }

    #[test]
    fn overflow_underflow_and_mismatched_domains_refuse_instead_of_guessing() {
        let huge = bezier(1, &[-f64::MAX, f64::MAX], &[1.0; 2]);
        let zero = bezier(1, &[0.0, 0.0], &[1.0; 2]);
        let tiny_weight = bezier(1, &[0.0, 1.0], &[f64::from_bits(1), f64::MAX]);
        let context = OperationContext::new();
        for source in [&huge, &tiny_weight] {
            let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
            assert!(matches!(
                curve_deviation_bound(source, &zero, &mut budget),
                Err(ReuseError::BoundUnavailable { .. })
            ));
        }
        let shifted = NurbsCurve::new(
            1,
            vec![1.0, 1.0, 2.0, 2.0],
            vec![Point3::new(0.0, 0.0, 0.0); 2],
            vec![1.0; 2],
        )
        .unwrap();
        let mut budget = ReuseBudget::new(100, &context).unwrap();
        assert!(matches!(
            curve_deviation_bound(&zero, &shifted, &mut budget),
            Err(ReuseError::Unsupported { .. })
        ));
    }

    #[test]
    fn identity_is_the_only_zero_certificate_and_still_checks_budget_cancellation() {
        let source = bezier(1, &[0.0, 1.0], &[1.0; 2]);
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(100, &context).unwrap();
        assert_eq!(
            curve_deviation_bound(&source, &source, &mut budget).unwrap(),
            0.0
        );
        let mut budget = ReuseBudget::new(1, &context).unwrap();
        assert!(matches!(
            curve_deviation_bound(&source, &source, &mut budget),
            Err(ReuseError::WorkLimit { .. })
        ));
        let token = CancellationToken::new();
        let cancelled = OperationContext::new().with_cancellation(token.clone());
        let mut budget = ReuseBudget::new(100, &cancelled).unwrap();
        token.cancel();
        assert!(matches!(
            curve_deviation_bound(&source, &source, &mut budget),
            Err(ReuseError::Math(crate::MathError::Cancelled))
        ));
    }

    #[test]
    fn bounded_degree_products_enclose_constant_one() {
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(1_000_000, &context).unwrap();
        let a = Polynomial {
            du: 23,
            dv: 0,
            coefficients: vec![Interval::ONE; 24],
        };
        let b = Polynomial {
            du: 24,
            dv: 0,
            coefficients: vec![Interval::ONE; 25],
        };
        let product = a.product(&b, &mut budget).unwrap();
        for value in product.coefficients {
            assert_contains(value, 1.0);
        }
    }
}
