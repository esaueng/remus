//! Bounded algebraic events for an actual Line/Sphere carrier pair.
//!
//! The line parameter is native: `start + (end - start) * t`. A tolerance
//! well is never a root locator. Each emitted point encloses the complete
//! Cartesian image of its algebraic root within one eighth of linear
//! tolerance; uncertainty that cannot meet this allocation declines.

// Equality here establishes exact arithmetic and stored endpoint identity.
// A tolerance comparison would invalidate the cancellation certificates.
#![allow(clippy::float_cmp)]

use remus_math::surfaces::SphericalSurface;
use remus_math::vec::Point3;

#[derive(Clone, Copy, Debug)]
struct I {
    lo: f64,
    hi: f64,
}

impl I {
    const fn point(value: f64) -> Self {
        Self {
            lo: value,
            hi: value,
        }
    }

    const fn unknown() -> Self {
        Self {
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
        }
    }

    fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi
    }

    // Error-free addition recognizes an exactly representable endpoint.
    // Keeping exact zero is essential at an algebraic tangency: gratuitous
    // ulps before sqrt would turn it into a sqrt(EPS)*model-size cluster.
    fn add_bound(a: f64, b: f64, upper: bool) -> f64 {
        let sum = a + b;
        if !sum.is_finite() {
            return sum;
        }
        let virtual_b = sum - a;
        let error = (a - (sum - virtual_b)) + (b - virtual_b);
        if !error.is_finite() {
            return if upper {
                sum.next_up()
            } else {
                sum.next_down()
            };
        }
        if (upper && error > 0.0) || (!upper && error < 0.0) {
            if upper {
                sum.next_up()
            } else {
                sum.next_down()
            }
        } else {
            sum
        }
    }

    fn add(self, other: Self) -> Self {
        if !self.finite() || !other.finite() {
            return Self::unknown();
        }
        Self {
            lo: Self::add_bound(self.lo, other.lo, false),
            hi: Self::add_bound(self.hi, other.hi, true),
        }
    }

    fn neg(self) -> Self {
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
    fn sub(self, other: Self) -> Self {
        self.add(other.neg())
    }

    fn product(a: f64, b: f64) -> Self {
        if a == 0.0 || b == 0.0 {
            return Self::point(0.0);
        }
        let product = a * b;
        // Below this floor an FMA residual could itself underflow, so zero
        // would no longer certify an exact multiplication.
        if !product.is_finite() || product.abs() < f64::MIN_POSITIVE / (f64::EPSILON * f64::EPSILON)
        {
            return Self::unknown();
        }
        let error = a.mul_add(b, -product);
        Self {
            lo: if error < 0.0 {
                product.next_down()
            } else {
                product
            },
            hi: if error > 0.0 {
                product.next_up()
            } else {
                product
            },
        }
    }

    fn mul(self, other: Self) -> Self {
        if !self.finite() || !other.finite() {
            return Self::unknown();
        }
        let products = [
            Self::product(self.lo, other.lo),
            Self::product(self.lo, other.hi),
            Self::product(self.hi, other.lo),
            Self::product(self.hi, other.hi),
        ];
        if products.iter().any(|p| !p.finite()) {
            return Self::unknown();
        }
        Self {
            lo: products.iter().map(|p| p.lo).fold(f64::INFINITY, f64::min),
            hi: products
                .iter()
                .map(|p| p.hi)
                .fold(f64::NEG_INFINITY, f64::max),
        }
    }

    fn div_positive(self, denominator: Self) -> Self {
        if !self.finite() || !denominator.finite() || denominator.lo <= 0.0 {
            return Self::unknown();
        }
        if self.lo == 0.0 && self.hi == 0.0 {
            return Self::point(0.0);
        }
        let quotients = [
            self.lo / denominator.lo,
            self.lo / denominator.hi,
            self.hi / denominator.lo,
            self.hi / denominator.hi,
        ];
        if quotients.iter().any(|q| !q.is_finite()) {
            return Self::unknown();
        }
        Self {
            lo: quotients
                .into_iter()
                .fold(f64::INFINITY, f64::min)
                .next_down(),
            hi: quotients
                .into_iter()
                .fold(f64::NEG_INFINITY, f64::max)
                .next_up(),
        }
    }

    fn sqrt_nonnegative(self) -> Self {
        if !self.finite() || self.hi < 0.0 {
            return Self::unknown();
        }
        let lo = self.lo.max(0.0).sqrt();
        let hi = self.hi.sqrt();
        let exact = self.lo == self.hi
            && Self::product(lo, lo).lo == self.lo
            && Self::product(lo, lo).hi == self.hi;
        if exact {
            return Self::point(lo);
        }
        Self {
            lo: lo.next_down().max(0.0),
            hi: hi.next_up(),
        }
    }

    fn abs_max(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }
    fn midpoint(self) -> f64 {
        f64::midpoint(self.lo, self.hi)
    }
    fn hull(self, other: Self) -> Self {
        Self {
            lo: self.lo.min(other.lo),
            hi: self.hi.max(other.hi),
        }
    }
}

fn dot(a: [I; 3], b: [I; 3]) -> I {
    a.into_iter()
        .zip(b)
        .map(|(a, b)| a.mul(b))
        .fold(I::point(0.0), I::add)
}

/// Fixed-capacity exact floating expansion. Error-free sums/products retain
/// cancellation before the final outward interval conversion. Unsupported
/// exponent ranges and capacity exhaustion decline instead of approximating.
#[derive(Clone)]
struct Expansion(Vec<f64>);

impl Expansion {
    const CAPACITY: usize = 32;

    fn point(value: f64) -> Option<Self> {
        value.is_finite().then(|| Self(vec![value]))
    }

    fn grow(&mut self, value: f64) -> Option<()> {
        let mut result = Vec::with_capacity(self.0.len() + 1);
        let mut sum = value;
        for &term in &self.0 {
            let total = sum + term;
            let virtual_term = total - sum;
            let error = (sum - (total - virtual_term)) + (term - virtual_term);
            if !total.is_finite() || !error.is_finite() {
                return None;
            }
            if error != 0.0 {
                result.push(error);
            }
            sum = total;
        }
        if sum != 0.0 || result.is_empty() {
            result.push(sum);
        }
        if result.len() > Self::CAPACITY {
            return None;
        }
        self.0 = result;
        Some(())
    }

    fn add(mut self, other: Self) -> Option<Self> {
        for term in other.0 {
            self.grow(term)?;
        }
        Some(self)
    }

    fn sub(self, mut other: Self) -> Option<Self> {
        for term in &mut other.0 {
            *term = -*term;
        }
        self.add(other)
    }

    fn mul(self, other: Self) -> Option<Self> {
        let mut result = Self::point(0.0)?;
        for a in self.0 {
            for &b in &other.0 {
                if a == 0.0 || b == 0.0 {
                    continue;
                }
                let product = a * b;
                if !product.is_finite()
                    || product.abs() < f64::MIN_POSITIVE / (f64::EPSILON * f64::EPSILON)
                {
                    return None;
                }
                let error = a.mul_add(b, -product);
                result.grow(error)?;
                result.grow(product)?;
            }
        }
        Some(result)
    }

    fn interval(self) -> I {
        self.0.into_iter().map(I::point).fold(I::point(0.0), I::add)
    }
}

fn exact_difference(a: Point3, b: Point3) -> Option<[Expansion; 3]> {
    Some([
        Expansion::point(a.x())?.sub(Expansion::point(b.x())?)?,
        Expansion::point(a.y())?.sub(Expansion::point(b.y())?)?,
        Expansion::point(a.z())?.sub(Expansion::point(b.z())?)?,
    ])
}

fn exact_dot(a: &[Expansion; 3], b: &[Expansion; 3]) -> Option<Expansion> {
    let mut sum = Expansion::point(0.0)?;
    for (a, b) in a.iter().zip(b) {
        sum = sum.add(a.clone().mul(b.clone())?)?;
    }
    Some(sum)
}

fn exact_axis_endpoint(point: Point3, sphere: &SphericalSurface) -> bool {
    let offset: [I; 3] =
        std::array::from_fn(|j| I::point(point.0[j]).sub(I::point(sphere.center().0[j])));
    (0..3).any(|axis| {
        offset.iter().enumerate().all(|(j, value)| {
            value.lo == value.hi
                && if j == axis {
                    value.lo.abs() == sphere.radius()
                } else {
                    value.lo == 0.0
                }
        })
    })
}

fn point_residual(point: Point3, sphere: &SphericalSurface) -> I {
    let offset = std::array::from_fn(|j| I::point(point.0[j]).sub(I::point(sphere.center().0[j])));
    dot(offset, offset)
        .sqrt_nonnegative()
        .sub(I::point(sphere.radius()))
}

fn qualify_point(
    start: Point3,
    end: Point3,
    direction: [I; 3],
    root: I,
    parameter: f64,
    sphere: &SphericalSurface,
    budget: f64,
) -> Option<Point3> {
    let point = if parameter == 0.0 {
        start
    } else if parameter == 1.0 {
        end
    } else {
        start + (end - start) * parameter
    };
    let radius = (0..3)
        .map(|j| {
            let image = I::point(start.0[j]).add(direction[j].mul(root));
            let difference = image.sub(I::point(point.0[j]));
            if difference.finite() {
                I::point(difference.abs_max())
            } else {
                I::unknown()
            }
        })
        .fold(I::point(0.0), I::add);
    let residual = point_residual(point, sphere);
    (radius.finite() && radius.hi <= budget && residual.finite() && residual.abs_max() <= budget)
        .then_some(point)
}

/// `None` preserves the existing solver for an unqualified arithmetic case.
/// `Some(empty)` certifies that no actual carrier root meets the native span.
#[allow(clippy::too_many_arguments)]
pub(super) fn crossings(
    start: Point3,
    end: Point3,
    t0: f64,
    t1: f64,
    sphere: &SphericalSurface,
    tolerance: f64,
) -> Option<Vec<(f64, Point3)>> {
    if !tolerance.is_finite()
        || tolerance <= 0.0
        || !t0.is_finite()
        || !t1.is_finite()
        || t0 == t1
        || !sphere.radius().is_finite()
        || sphere.radius() <= 0.0
        || start
            .0
            .iter()
            .chain(end.0.iter())
            .chain(sphere.center().0.iter())
            .any(|v| !v.is_finite())
    {
        return None;
    }
    let direction = std::array::from_fn(|j| I::point(end.0[j]).sub(I::point(start.0[j])));
    // Recenter the polynomial at a structurally exact spherical endpoint
    // when available. The shared stored radius then cancels symbolically;
    // independent outward squares must not invent two roots at a tangency.
    let (anchor, origin) = if exact_axis_endpoint(start, sphere) {
        (0.0, start)
    } else if exact_axis_endpoint(end, sphere) {
        (1.0, end)
    } else {
        (0.0, start)
    };
    let exact_direction = exact_difference(end, start)?;
    let exact_offset = exact_difference(origin, sphere.center())?;
    let exact_a = exact_dot(&exact_direction, &exact_direction)?;
    let exact_b = exact_dot(&exact_offset, &exact_direction)?.mul(Expansion::point(2.0)?)?;
    let exact_c = if exact_axis_endpoint(origin, sphere) {
        Expansion::point(0.0)?
    } else {
        exact_dot(&exact_offset, &exact_offset)?
            .sub(Expansion::point(sphere.radius())?.mul(Expansion::point(sphere.radius())?)?)?
    };
    let discriminant = exact_b
        .clone()
        .mul(exact_b.clone())?
        .sub(exact_a.clone().mul(exact_c)?.mul(Expansion::point(4.0)?)?)?
        .interval();
    let a = exact_a.interval();
    let b = exact_b.interval();
    if !a.finite() || a.lo <= 0.0 {
        return None;
    }
    if !discriminant.finite() {
        return None;
    }
    if discriminant.hi < 0.0 {
        return Some(Vec::new());
    }
    let radical = discriminant.sqrt_nonnegative();
    let denominator = a.mul(I::point(2.0));
    let first = b
        .neg()
        .sub(radical)
        .div_positive(denominator)
        .add(I::point(anchor));
    let last = b
        .neg()
        .add(radical)
        .div_positive(denominator)
        .add(I::point(anchor));
    if !first.finite() || !last.finite() {
        return None;
    }
    let roots = if discriminant.lo <= 0.0 {
        vec![first.hull(last)]
    } else {
        vec![first, last]
    };
    let lo = t0.min(t1);
    let hi = t0.max(t1);
    let mut events: Vec<(f64, Point3)> = Vec::new();
    for root in roots {
        if root.hi < lo || root.lo > hi {
            continue;
        }
        let parameter = if root.lo <= lo {
            lo
        } else if root.hi >= hi {
            hi
        } else {
            root.midpoint()
        };
        let point = qualify_point(
            start,
            end,
            direction,
            root,
            parameter,
            sphere,
            tolerance / 8.0,
        )?;
        events.push((parameter, point));
    }
    Some(events)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]
    use super::*;

    #[test]
    fn square_root_interval_encloses_both_sides_of_rounded_root() {
        // This binary64 value lies strictly between sqrt(2) and
        // sqrt(next_up(2)). Compare its square with both endpoints using
        // exact integer arithmetic, independently of the interval helpers.
        let significand = 6_369_051_672_525_773_u128;
        let square = significand * significand;
        assert!(square > (1_u128 << 105));
        assert!(square < (1_u128 << 105) + (1_u128 << 53));
        let rounded_root = f64::from_bits(0x3ff6_a09e_667f_3bcd);
        let bounds = I {
            lo: 2.0,
            hi: 2.0_f64.next_up(),
        }
        .sqrt_nonnegative();
        assert!(bounds.lo < rounded_root, "{bounds:?}");
        assert!(bounds.hi > rounded_root, "{bounds:?}");
    }

    #[test]
    fn square_root_interval_preserves_exact_roots_and_declines_unknowns() {
        for (square, root) in [(0.0, 0.0), (0.25, 0.5), (4.0, 2.0), (256.0, 16.0)] {
            let bounds = I::point(square).sqrt_nonnegative();
            assert_eq!((bounds.lo, bounds.hi), (root, root));
        }
        let crossing_zero = I { lo: -4.0, hi: 9.0 }.sqrt_nonnegative();
        assert_eq!(crossing_zero.lo, 0.0);
        assert!(crossing_zero.hi >= 3.0);
        for interval in [
            I::unknown(),
            I { lo: 4.0, hi: 1.0 },
            I { lo: -9.0, hi: -1.0 },
            I::point(f64::NAN),
        ] {
            let bounds = interval.sqrt_nonnegative();
            assert_eq!(bounds.lo, f64::NEG_INFINITY);
            assert_eq!(bounds.hi, f64::INFINITY);
        }
    }

    #[test]
    fn exact_axis_tangencies_retain_native_endpoint_authority() {
        for radius in [0.001, 1.0, 1000.0] {
            let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), radius).unwrap();
            for (a, b, expected) in [
                (
                    Point3::new(0.0, 0.0, radius),
                    Point3::new(0.0, radius, radius),
                    0.0,
                ),
                (
                    Point3::new(0.0, radius, radius),
                    Point3::new(0.0, 0.0, radius),
                    1.0,
                ),
            ] {
                let events = crossings(a, b, 0.0, 1.0, &sphere, 1e-7).unwrap();
                assert_eq!(events.len(), 1, "radius={radius}: {events:?}");
                assert_eq!(events[0].0, expected);
                assert_eq!(events[0].1, if expected == 0.0 { a } else { b });
            }
        }
    }

    #[test]
    fn radial_crossings_and_native_subspans_use_cartesian_roots() {
        for radius in [0.001, 1.0, 1000.0] {
            let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), radius).unwrap();
            let a = Point3::new(-2.0 * radius, 0.0, 0.0);
            let b = Point3::new(2.0 * radius, 0.0, 0.0);
            for (lo, hi, expected) in [
                (0.0, 1.0, vec![0.25, 0.75]),
                (1.0, 0.0, vec![0.25, 0.75]),
                (0.4, 0.6, vec![]),
                (0.25, 0.5, vec![0.25]),
            ] {
                let events = crossings(a, b, lo, hi, &sphere, 1e-7).unwrap();
                assert_eq!(events.len(), expected.len());
                for ((parameter, point), expected) in events.iter().zip(expected) {
                    assert!((parameter - expected).abs() < 1e-12);
                    let exact = if expected < 0.5 { -radius } else { radius };
                    assert!((point.x() - exact).abs() <= 1e-7 / 8.0);
                    assert_eq!(point.y(), 0.0);
                    assert_eq!(point.z(), 0.0);
                }
            }
        }
    }

    #[test]
    fn separated_near_tangent_roots_are_not_a_tolerance_well() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1000.0).unwrap();
        for height in [999.0, 1001.0] {
            let a = Point3::new(-1000.0, height, 0.0);
            let b = Point3::new(1000.0, height, 0.0);
            let events = crossings(a, b, 0.0, 1.0, &sphere, 1e-7).unwrap();
            if height > 1000.0 {
                assert!(events.is_empty());
            } else {
                assert_eq!(events.len(), 2);
                let exact = (1000.0_f64.powi(2) - height.powi(2)).sqrt();
                assert!((events[0].1.x() + exact).abs() < 1e-7 / 8.0);
                assert!((events[1].1.x() - exact).abs() < 1e-7 / 8.0);
            }
        }
        assert!(
            crossings(
                Point3::new(-2.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                0.0,
                1.0,
                &SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap(),
                1e-30
            )
            .is_none()
        );
    }

    #[test]
    fn invalid_and_underflow_inputs_decline() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
        for (a, b, tolerance) in [
            (
                Point3::new(f64::NAN, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                1e-7,
            ),
            (Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0), 1e-7),
            (
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1e-300, 0.0, 0.0),
                1e-7,
            ),
            (
                Point3::new(-1e308, 0.0, 0.0),
                Point3::new(1e308, 0.0, 0.0),
                1e-7,
            ),
            (Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 0.0),
        ] {
            assert!(crossings(a, b, 0.0, 1.0, &sphere, tolerance).is_none());
        }
    }

    #[test]
    fn rigid_placed_secants_enclose_independent_cartesian_points() {
        use remus_math::mat::Mat4;
        for radius in [0.001, 1.0, 1000.0] {
            let matrix = Mat4::translation(13.0 * radius, -7.0 * radius, 3.0 * radius)
                * Mat4::rotation_z(0.43)
                * Mat4::rotation_x(0.71);
            let center = matrix.mul_point(Point3::new(0.0, 0.0, 0.0));
            let sphere = SphericalSurface::new(center, radius).unwrap();
            let start = matrix.mul_point(Point3::new(-2.0 * radius, 0.0, 0.0));
            let end = matrix.mul_point(Point3::new(2.0 * radius, 0.0, 0.0));
            let events = crossings(start, end, 0.0, 1.0, &sphere, 1e-7).unwrap();
            assert_eq!(events.len(), 2);
            for (event, sign) in events.iter().zip([-1.0, 1.0]) {
                let expected = matrix.mul_point(Point3::new(sign * radius, 0.0, 0.0));
                assert!(
                    (event.1 - expected).length() <= 1e-7 / 8.0,
                    "radius={radius} event={event:?} expected={expected:?}"
                );
            }
        }
    }

    #[test]
    fn stored_large_placed_miss_does_not_create_a_tolerance_well_contact() {
        let sphere = SphericalSurface::new(Point3::new(10000.0, -7000.0, 3000.0), 1000.0).unwrap();
        let a = Point3::new(
            9_772.022_476_464_812,
            -6_453.839_626_403_805,
            4_284.420_139_625_67,
        );
        let b = Point3::new(10000.0, -7_361.615_431_964_962, 3_932.327_345_606_034_7);
        // Independent85-digit arithmetic on these exact binary64 inputs:
        // D=-0.0025516389320453505745188310085900358331664268230832...
        // The historical unsigned scan emitted a point5.9e-3 from b.
        assert!(crossings(a, b, 0.0, 1.0, &sphere, 1e-7).unwrap().is_empty());

        let a = Point3::new(
            10_745.688_871_470_187,
            -5_879.674_515_063_443,
            3_434.532_984_671_115_6,
        );
        let b = Point3::new(
            9_772.022_476_464_812,
            -6_092.224_194_438_842_5,
            3_352.092_794_019_636,
        );
        // This different stored line is genuinely secant. Both roots must
        // survive when the native span contains them, despite proximity.
        let events = crossings(a, b, 0.0, 2.0, &sphere, 1e-7).unwrap();
        assert_eq!(events.len(), 2);
        for (event, expected) in events
            .iter()
            .zip([0.999_999_981_383_436_2, 1.000_000_018_616_563_6])
        {
            assert!((event.0 - expected).abs() < 1e-13);
        }
        assert!((events[1].1 - events[0].1).length() > 3e-5);
    }

    #[test]
    fn expansion_cancellation_is_exact_and_resource_failures_decline() {
        let value = Expansion::point(1e16)
            .unwrap()
            .add(Expansion::point(1.0).unwrap())
            .unwrap()
            .sub(Expansion::point(1e16).unwrap())
            .unwrap()
            .interval();
        assert_eq!(value.lo, 1.0);
        assert_eq!(value.hi, 1.0);
        for (a, b) in [(1e-300, 1e-300), (1e300, 1e300)] {
            assert!(
                Expansion::point(a)
                    .unwrap()
                    .mul(Expansion::point(b).unwrap())
                    .is_none()
            );
        }
        let mut value = Expansion::point(0.0).unwrap();
        for exponent in
            (0..Expansion::CAPACITY).map(|index| -900 + 54 * i32::try_from(index).unwrap())
        {
            value.grow(2.0_f64.powi(exponent)).unwrap();
        }
        assert!(
            value
                .grow(2.0_f64.powi(-900 + 54 * i32::try_from(Expansion::CAPACITY).unwrap()))
                .is_none()
        );
    }
}
