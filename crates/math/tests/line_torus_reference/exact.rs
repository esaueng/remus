//! The exact sign of the line–torus quartic at an `f64` parameter.
//!
//! Every input is an `f64`, hence a dyadic rational, and the quartic is a
//! polynomial in them. [`Big`] is an exact dyadic number (a sign, a
//! little-endian magnitude in base `2^32` and a binary exponent) with exact
//! addition, subtraction and multiplication: no rounding, no error bound,
//! nothing to trust but schoolbook arithmetic. So the sign returned by
//! [`quartic_sign`] is that of the quartic of the represented line and
//! torus, evaluated exactly at the represented parameter, however far the
//! torus is from anything.
//!
//! It shares nothing with the solver under test or with the double-double
//! reference next to it.

use std::cmp::Ordering;

use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};

/// An exact dyadic number: `±mag · 2^exp`.
#[derive(Clone, Debug)]
pub struct Big {
    negative: bool,
    /// Little-endian base-`2^32` digits, no leading (most significant) zero
    /// digit; empty for zero.
    mag: Vec<u32>,
    exp: i64,
}

fn trim(mag: &mut Vec<u32>) {
    while mag.last() == Some(&0) {
        mag.pop();
    }
}

fn cmp_mag(a: &[u32], b: &[u32]) -> Ordering {
    a.len()
        .cmp(&b.len())
        .then_with(|| a.iter().rev().cmp(b.iter().rev()))
}

fn add_mag(a: &[u32], b: &[u32]) -> Vec<u32> {
    let (long, short) = if a.len() >= b.len() { (a, b) } else { (b, a) };
    let mut out = Vec::with_capacity(long.len() + 1);
    let mut carry = 0_u64;
    for (i, &digit) in long.iter().enumerate() {
        let sum = u64::from(digit) + u64::from(short.get(i).copied().unwrap_or(0)) + carry;
        out.push(sum as u32);
        carry = sum >> 32;
    }
    if carry != 0 {
        out.push(carry as u32);
    }
    out
}

/// `a − b` for `a ≥ b`.
fn sub_mag(a: &[u32], b: &[u32]) -> Vec<u32> {
    let mut out = Vec::with_capacity(a.len());
    let mut borrow = 0_i64;
    for (i, &digit) in a.iter().enumerate() {
        let mut diff = i64::from(digit) - i64::from(b.get(i).copied().unwrap_or(0)) - borrow;
        borrow = i64::from(diff < 0);
        if diff < 0 {
            diff += 1 << 32;
        }
        out.push(diff as u32);
    }
    assert_eq!(borrow, 0, "sub_mag needs a ≥ b");
    trim(&mut out);
    out
}

/// `mag · 2^bits`.
fn shl_mag(mag: &[u32], bits: u64) -> Vec<u32> {
    if mag.is_empty() {
        return Vec::new();
    }
    let (words, shift) = ((bits / 32) as usize, (bits % 32) as u32);
    let mut out = vec![0_u32; words];
    let mut carry = 0_u32;
    for &digit in mag {
        if shift == 0 {
            out.push(digit);
        } else {
            out.push((digit << shift) | carry);
            carry = digit >> (32 - shift);
        }
    }
    if carry != 0 {
        out.push(carry);
    }
    out
}

impl Big {
    pub fn zero() -> Self {
        Self {
            negative: false,
            mag: Vec::new(),
            exp: 0,
        }
    }

    /// The finite `f64` `x`, exactly.
    pub fn of(x: f64) -> Self {
        assert!(x.is_finite(), "{x}");
        let bits = x.to_bits();
        let fraction = bits & ((1 << 52) - 1);
        let biased = ((bits >> 52) & 0x7ff) as i64;
        let (significand, exp) = if biased == 0 {
            (fraction, -1074)
        } else {
            (fraction | (1 << 52), biased - 1075)
        };
        let mut mag = vec![significand as u32, (significand >> 32) as u32];
        trim(&mut mag);
        Self {
            negative: x.is_sign_negative() && significand != 0,
            mag,
            exp,
        }
    }

    /// The exact sum of some `f64` values.
    pub fn sum(terms: &[f64]) -> Self {
        terms
            .iter()
            .fold(Self::zero(), |acc, &x| acc.add(&Self::of(x)))
    }

    pub fn sign(&self) -> i8 {
        if self.mag.is_empty() {
            0
        } else if self.negative {
            -1
        } else {
            1
        }
    }

    pub fn neg(&self) -> Self {
        Self {
            negative: !self.negative && !self.mag.is_empty(),
            mag: self.mag.clone(),
            exp: self.exp,
        }
    }

    pub fn add(&self, o: &Self) -> Self {
        if self.mag.is_empty() {
            return o.clone();
        }
        if o.mag.is_empty() {
            return self.clone();
        }
        let exp = self.exp.min(o.exp);
        let a = shl_mag(&self.mag, (self.exp - exp) as u64);
        let b = shl_mag(&o.mag, (o.exp - exp) as u64);
        let (negative, mut mag) = if self.negative == o.negative {
            (self.negative, add_mag(&a, &b))
        } else {
            match cmp_mag(&a, &b) {
                Ordering::Equal => return Self::zero(),
                Ordering::Greater => (self.negative, sub_mag(&a, &b)),
                Ordering::Less => (o.negative, sub_mag(&b, &a)),
            }
        };
        // Keep the digits few: whole zero digits at the bottom go into the
        // exponent.
        let low_zeros = mag.iter().take_while(|&&d| d == 0).count();
        mag.drain(..low_zeros);
        Self {
            negative,
            mag,
            exp: exp + 32 * low_zeros as i64,
        }
    }

    pub fn sub(&self, o: &Self) -> Self {
        self.add(&o.neg())
    }

    pub fn mul(&self, o: &Self) -> Self {
        if self.mag.is_empty() || o.mag.is_empty() {
            return Self::zero();
        }
        let mut mag = vec![0_u32; self.mag.len() + o.mag.len()];
        for (i, &a) in self.mag.iter().enumerate() {
            let mut carry = 0_u64;
            for (j, &b) in o.mag.iter().enumerate() {
                let cell = u64::from(mag[i + j]) + u64::from(a) * u64::from(b) + carry;
                mag[i + j] = cell as u32;
                carry = cell >> 32;
            }
            mag[i + o.mag.len()] = carry as u32;
        }
        trim(&mut mag);
        Self {
            negative: self.negative != o.negative,
            mag,
            exp: self.exp + o.exp,
        }
    }

    pub fn square(&self) -> Self {
        self.mul(self)
    }
}

/// The exact sign of the line–torus quartic `G² − 4R²N` of `origin + t·dir`
/// with `torus` at the parameter `t = Σ terms` (one `f64`, or the two words
/// of a double-double).
///
/// `(a, b, c)` are the coordinates of `origin − centre + t·dir` along the
/// torus frame's three stored axes, `N = a² + b²` and
/// `G = N + c² + R² − r²`. On a ring torus the sign is that of the tube's
/// implicit function: positive outside the tube, negative inside, zero on
/// the surface.
pub fn quartic_sign(torus: &ToroidalSurface, origin: Point3, dir: Vec3, terms: &[f64]) -> i8 {
    let t = Big::sum(terms);
    let centre = torus.center();
    let point = [
        (origin.x(), centre.x(), dir.x()),
        (origin.y(), centre.y(), dir.y()),
        (origin.z(), centre.z(), dir.z()),
    ]
    .map(|(o, c, d)| Big::of(o).sub(&Big::of(c)).add(&t.mul(&Big::of(d))));
    let [a, b, c] = [torus.x_axis(), torus.y_axis(), torus.z_axis()].map(|axis| {
        Big::of(axis.x())
            .mul(&point[0])
            .add(&Big::of(axis.y()).mul(&point[1]))
            .add(&Big::of(axis.z()).mul(&point[2]))
    });
    let big = Big::of(torus.major_radius()).square();
    let small = Big::of(torus.minor_radius()).square();
    let n = a.square().add(&b.square());
    let g = n.add(&c.square()).add(&big).sub(&small);
    let four_rr_n = Big::of(4.0).mul(&big).mul(&n);
    g.square().sub(&four_rr_n).sign()
}
