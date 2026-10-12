//! Exact in-circle sign by integer arithmetic, for the inputs where the
//! floating-point filter in [`super::fast_in_circle`] cannot decide.
//!
//! The filter fails exactly when the four points are (nearly) cocircular,
//! and every rectangular planar face hands the CDT four exactly cocircular
//! corners. `robust::incircle` then runs Shewchuk's adaptive stages, which
//! zero-fill several kilobytes of expansion scratch per call and, when the
//! coordinate differences are not exact (decimal coordinates such as 1.6 or
//! 40.4), go all the way to the full expansion. On a shelled 42 mm bin that
//! was two thirds of the cost of triangulating its quads.
//!
//! # Why the sign is the same
//!
//! The legalizer only reads the sign of the in-circle value. When the eight
//! coordinates span a narrow enough binary range, every one is an integer
//! multiple of `2^lo`, the smallest weight of any set bit among them.
//! Dividing by `2^lo` (an exact power-of-two scaling) gives integers below
//! `2^61`. The differences then lie below `2^62`, the 2×2 minors and lifted
//! squares below `2^125`, and each of the three products below `2^250`, so
//! the determinant is evaluated exactly in 128- and 256-bit integers. Scaling
//! every coordinate by `2^-lo` multiplies the determinant by `2^-4lo > 0`,
//! so its sign is the sign of the exact real determinant of the inputs.
//!
//! `robust::incircle` returns that same exact sign whenever none of its
//! expansion arithmetic underflows or overflows (Shewchuk's precondition).
//! The window below guarantees it: every coordinate is a multiple of
//! `2^-200` and below `2^201` in magnitude, so every exact difference,
//! product of up to four of them, and expansion component is a multiple of
//! `2^-800` (far above the subnormal range) and below `2^812` (far below
//! overflow, including Dekker's splitter). Inputs outside the window, and
//! non-finite ones, return `None` and the caller keeps `robust::incircle`.

use crate::vec::Point2;

/// Largest bit span (`top − lo + 1`) for which scaled coordinates stay below
/// `2^61`.
const MAX_SPAN_BITS: i32 = 61;

/// Smallest admitted weight of a set coordinate bit (no underflow anywhere in
/// the reference predicate).
const MIN_LSB_EXP: i32 = -200;

/// Largest admitted weight of a set coordinate bit (no overflow anywhere in
/// the reference predicate).
const MAX_TOP_EXP: i32 = 200;

/// A finite double as `±mant·2^lsb` with `mant` odd, or zero (`mant == 0`).
#[derive(Clone, Copy)]
struct Dyadic {
    negative: bool,
    mant: u64,
    lsb: i32,
}

impl Dyadic {
    const ZERO: Self = Self {
        negative: false,
        mant: 0,
        lsb: 0,
    };

    /// Exact decomposition, `None` for NaN or infinity.
    fn of(x: f64) -> Option<Self> {
        if !x.is_finite() {
            return None;
        }
        let bits = x.to_bits();
        let negative = bits >> 63 == 1;
        // The biased exponent is 11 bits wide, so the cast is lossless.
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let biased = ((bits >> 52) & 0x7ff) as i32;
        let fraction = bits & ((1_u64 << 52) - 1);
        let (mant, exp) = if biased == 0 {
            (fraction, -1074)
        } else {
            (fraction | (1_u64 << 52), biased - 1075)
        };
        if mant == 0 {
            return Some(Self::ZERO);
        }
        let shift = mant.trailing_zeros();
        // `shift` is below 64.
        #[allow(clippy::cast_possible_wrap)]
        let lsb = exp + shift as i32;
        Some(Self {
            negative,
            mant: mant >> shift,
            lsb,
        })
    }

    /// Weight of the highest set bit; only meaningful when `mant != 0`.
    fn top(self) -> i32 {
        // `leading_zeros` is at most 63 for a nonzero `mant`.
        #[allow(clippy::cast_possible_wrap)]
        let width = 63 - self.mant.leading_zeros() as i32;
        self.lsb + width
    }

    /// `self / 2^lo` as an integer; the caller guarantees `lo <= lsb` and
    /// `top − lo < 61`.
    fn scaled(self, lo: i32) -> i64 {
        if self.mant == 0 {
            return 0;
        }
        // `lsb − lo` is in `[0, 60]`, and the shifted mantissa is below
        // `2^61`, so neither the shift nor the cast loses bits.
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_wrap)]
        let magnitude = (self.mant << (self.lsb - lo) as u32) as i64;
        if self.negative { -magnitude } else { magnitude }
    }
}

/// An unsigned 256-bit integer as `(high, low)` 128-bit halves.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct U256(u128, u128);

impl U256 {
    /// The full product of two 128-bit integers.
    fn product(a: u128, b: u128) -> Self {
        const MASK: u128 = (1 << 64) - 1;
        let (a1, a0) = (a >> 64, a & MASK);
        let (b1, b0) = (b >> 64, b & MASK);
        let p00 = a0 * b0;
        let p01 = a0 * b1;
        let p10 = a1 * b0;
        let p11 = a1 * b1;
        // Below 3·2^64: no overflow.
        let middle = (p00 >> 64) + (p01 & MASK) + (p10 & MASK);
        let low = (p00 & MASK) | (middle << 64);
        let high = p11 + (p01 >> 64) + (p10 >> 64) + (middle >> 64);
        Self(high, low)
    }

    /// `self + other`; the callers' sums stay below `2^252`.
    fn add(self, other: Self) -> Self {
        let (low, carry) = self.1.overflowing_add(other.1);
        Self(self.0 + other.0 + u128::from(carry), low)
    }
}

/// The sign (`1.0`, `-1.0` or `0.0`) of the in-circle determinant of
/// `(a, b, c; d)`, the same determinant `robust::incircle` evaluates, or
/// `None` when the inputs are outside the window where both are proven exact
/// (see the module docs).
pub(super) fn exact_in_circle_sign(a: Point2, b: Point2, c: Point2, d: Point2) -> Option<f64> {
    let coords = [a.x(), a.y(), b.x(), b.y(), c.x(), c.y(), d.x(), d.y()];
    let mut parts = [Dyadic::ZERO; 8];
    let mut lo = i32::MAX;
    let mut top = i32::MIN;
    for (part, &x) in parts.iter_mut().zip(&coords) {
        let dyadic = Dyadic::of(x)?;
        if dyadic.mant != 0 {
            lo = lo.min(dyadic.lsb);
            top = top.max(dyadic.top());
        }
        *part = dyadic;
    }
    if lo == i32::MAX {
        // All four points at the origin.
        return Some(0.0);
    }
    if lo < MIN_LSB_EXP || top > MAX_TOP_EXP || top - lo + 1 > MAX_SPAN_BITS {
        return None;
    }
    let [ax, ay, bx, by, cx, cy, dx, dy] = parts.map(|part| part.scaled(lo));

    // |coordinate| < 2^61, so each difference is below 2^62.
    let adx = i128::from(ax - dx);
    let ady = i128::from(ay - dy);
    let bdx = i128::from(bx - dx);
    let bdy = i128::from(by - dy);
    let cdx = i128::from(cx - dx);
    let cdy = i128::from(cy - dy);

    // Minors and lifts are below 2^125.
    let bc = bdx * cdy - cdx * bdy;
    let ca = cdx * ady - adx * cdy;
    let ab = adx * bdy - bdx * ady;
    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;

    // det = alift·bc + blift·ca + clift·ab, each term below 2^250. The lifts
    // are non-negative, so a term's sign is its minor's.
    let mut positive = U256(0, 0);
    let mut negative = U256(0, 0);
    for (lift, minor) in [(alift, bc), (blift, ca), (clift, ab)] {
        let term = U256::product(lift.unsigned_abs(), minor.unsigned_abs());
        if minor >= 0 {
            positive = positive.add(term);
        } else {
            negative = negative.add(term);
        }
    }
    Some(match positive.cmp(&negative) {
        std::cmp::Ordering::Greater => 1.0,
        std::cmp::Ordering::Less => -1.0,
        std::cmp::Ordering::Equal => 0.0,
    })
}
