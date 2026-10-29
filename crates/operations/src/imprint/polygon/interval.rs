//! Outward arithmetic for finite plane and complete convex-halfspace proofs.

use remus_math::vec::{Point3, Vec3};

#[derive(Clone, Copy)]
pub(super) struct I {
    pub lo: f64,
    pub hi: f64,
}

impl I {
    pub const fn exact(value: f64) -> Self {
        Self {
            lo: value,
            hi: value,
        }
    }

    pub fn add(self, other: Self) -> Self {
        let lo = self.lo + other.lo;
        let hi = self.hi + other.hi;
        if lo.is_nan() || hi.is_nan() {
            Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            }
        } else {
            Self {
                lo: lo.next_down(),
                hi: hi.next_up(),
            }
        }
    }

    pub fn neg(self) -> Self {
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }

    pub fn sub(self, other: Self) -> Self {
        self.add(other.neg())
    }

    pub fn mul(self, other: Self) -> Self {
        let values = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi,
        ];
        if values.iter().any(|value| value.is_nan()) {
            return Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            };
        }
        Self {
            lo: values.into_iter().fold(f64::INFINITY, f64::min).next_down(),
            hi: values
                .into_iter()
                .fold(f64::NEG_INFINITY, f64::max)
                .next_up(),
        }
    }

    pub fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }

    pub fn abs_max(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }
}

pub(super) fn difference(a: Point3, b: Point3) -> [I; 3] {
    std::array::from_fn(|j| I::exact(a.0[j]).sub(I::exact(b.0[j])))
}

pub(super) fn dot(a: [I; 3], b: [I; 3]) -> I {
    (0..3).fold(I::exact(0.0), |sum, j| sum.add(a[j].mul(b[j])))
}

pub(super) fn norm(vector: [I; 3]) -> I {
    let squared = dot(vector, vector);
    I {
        lo: squared.lo.max(0.0).sqrt().next_down().max(0.0),
        hi: squared.hi.sqrt().next_up(),
    }
}

pub(super) fn side(normal: Vec3, a: Point3, b: Point3, point: Point3) -> I {
    let edge = difference(b, a);
    let cross = std::array::from_fn(|j| {
        I::exact(normal.0[(j + 1) % 3])
            .mul(edge[(j + 2) % 3])
            .sub(I::exact(normal.0[(j + 2) % 3]).mul(edge[(j + 1) % 3]))
    });
    dot(cross, difference(point, a))
}
