//! Certified contacts of a line with a torus.
//!
//! # Contract
//!
//! For a finite line `origin + t·dir` and a torus, [`contacts_of_line`]
//! returns every real root of the line–torus quartic, each inside a window
//! `[t_lo, t_hi]`, ascending:
//!
//! - a **resolved** contact holds exactly one simple root, and the quartic
//!   has certified opposite signs at the window's two ends. The window is at
//!   most 128 units in the last place of the larger of `|t|` and the
//!   torus's size in units of `t`, and usually a few;
//! - an **unresolved** contact is a stretch the arithmetic below cannot
//!   separate: the quartic has a certified sign at both ends of the window
//!   and comes within its evaluation error of zero inside. Equal end signs
//!   make it an even contact (a tangency, or nothing), different ones an odd
//!   one (an inflectional crossing). Every root not reported as resolved lies
//!   in such a window.
//!
//! Between consecutive windows the quartic is certified nonzero. (The
//! windows are disjoint in the chart below; mapped to `t` their ends are
//! rounded outward, so neighbours may share an ulp.)
//!
//! Roots are those of the line and torus *as represented*: the `f64`
//! coordinates taken as exact, and the torus frame's three axes taken as the
//! linear map they are (they are orthonormal only to rounding). Nothing is
//! decided by a threshold on a root or on a residual. Every decision is the
//! sign of a value whose error bound was carried through the arithmetic
//! that produced it, under IEEE 754 round-to-nearest.
//!
//! # Method
//!
//! 1. **Chart.** `o = origin − centre` is formed exactly (an error-free
//!    sum), and it is the only place absolute coordinates enter. Lengths are
//!    scaled by a power of two so that `R + r` lies in `[1, 2)`, and the
//!    direction so that its largest component lies in `[0.5, 1)`; both are
//!    exact, so nothing depends on the scale of the torus or on `|dir|`. The
//!    line is re-based at its point nearest the centre,
//!    `q = o + τ0·d`, summed from exact products with a computed bound, so a
//!    far origin costs no accuracy either.
//! 2. **Quartic.** In the torus frame `G(σ) = a² + b² + c² + R² − r²` and
//!    `N(σ) = a² + b²` are quadratics, and the quartic is `Q = G² − 4R²N`.
//!    It is never expanded into power-basis coefficients. On an interval its
//!    Bernstein coefficients come from the quadratics' values and blossoms at
//!    the interval's ends.
//! 3. **Isolation.** By Descartes' rule of signs, an interval whose Bernstein
//!    coefficients all have one certified sign holds no root, and one whose
//!    certified signs change exactly once holds exactly one simple root.
//!    Anything else is split, at the quartic's local critical points where
//!    they exist, until one of the two holds or the split point's own sign
//!    cannot be certified. Roots lie within the bounding sphere `R + r` and
//!    the slab `|c| ≤ r`, which gives the first interval. The split points
//!    only steer: a poor one costs work, never a root.
//! 4. **Two precisions.** Every value is a ball: a number with a rigorous
//!    radius, updated by each operation. The solve first runs in plain `f64`
//!    balls. If any sign it needs is inside its radius it runs again in
//!    double-double balls, whose radius is about `1e-31` of the magnitudes
//!    involved, so two roots are separated whenever the quartic between them
//!    clears that: far below anything `f64` input can express short of an
//!    exact tangency.
//! 5. **Refinement.** A simple root is located by Newton's method on the
//!    quartic's two factors `(ρ ∓ R)² + c² − r²`, which do not cancel, and
//!    then enclosed by certified signs to either side: first in plain balls,
//!    which certify a crossing no closer than a few ulps of the torus's size
//!    over the sine of the crossing angle. A root they cannot enclose within
//!    128 ulps (a shallow crossing) is polished and enclosed with
//!    double-double values, to two ulps.
//!
//! A line with no finite chart (a zero or non-finite direction or origin, a
//! tube thinner than `2^-400` of the torus, an origin beyond `2^400` torus
//! sizes) is not certified, and neither is one whose double-double solve
//! runs out of its work budget (none is known); the caller falls back to the
//! iterative solver and says so.

#![allow(clippy::suboptimal_flops, clippy::many_single_char_names)]

use std::ops::{Add, Mul, Sub};

use crate::fma::fma;
use crate::intersect::ContactKind;
use crate::surfaces::ToroidalSurface;
use crate::vec::{Point3, Vec3};

/// Unit roundoff of `f64`: `2^-53`.
const U: f64 = 0.5 * f64::EPSILON;
/// Unit roundoff of a double-double: `2^-106`.
const U2: f64 = U * U;
/// Added to every radius an operation that can underflow produces: far above
/// the `2^-1074` such an operation can lose.
const TINY: f64 = f64::MIN_POSITIVE;
/// Factor on every radius, covering the rounding of the radius's own
/// arithmetic (a few operations on non-negative terms, each within `2^-53`).
const INFLATE: f64 = 1.000_000_001;

/// Below this angle (radians) between the line and the torus at a simple
/// root, the crossing is reported [`ContactKind::Unclassified`]: the root is
/// certified, but a ray that shallow is a poor witness for anything trimmed
/// near it.
const TORUS_GRAZE_SLOPE: f64 = 1e-6;

/// Thinnest tube, relative to `R + r`, the chart accepts: `r²` must stay
/// far above the subnormal range.
const MIN_TUBE: f64 = f64::from_bits((1023 - 400) << 52);
/// Farthest origin, in units of `R + r`, the chart accepts.
const MAX_REACH: f64 = f64::from_bits((1023 + 400) << 52);
/// Smallest spacing, in the chart's unit-scale parameter, at which signs are
/// probed: `2^-60`.
const GRAIN_FLOOR: f64 = f64::from_bits((1023 - 60) << 52);

/// Subdivision depth a plain-ball solve gives up at (it then reruns wide).
const PLAIN_DEPTH: u32 = 6;
/// Subdivision depth of a wide solve.
const WIDE_DEPTH: u32 = 96;
/// Evaluations a plain-ball solve may spend.
const PLAIN_WORK: u32 = 64;
/// Evaluations a wide solve may spend; past it the line is not certified.
const WIDE_WORK: u32 = 4000;
/// Steps of a bracketed refinement: past the halvings that shrink any
/// bracket to adjacent doubles, with room for Newton steps.
const REFINE_STEPS: usize = 160;
/// How far past its radius a value must be for its point to bound an
/// unresolved window. Next to a multiple root the quartic sits at its
/// radius over a stretch, where a bare sign is certified at one point and
/// not at the next; a window fenced there would be followed by another, a
/// grain on, without end. At sixteen radii the quartic is clear of the
/// stretch and rising away from it.
const WINDOW_MARGIN: f64 = 16.0;
/// Wide polish steps of one root.
const POLISH_STEPS: usize = 80;
/// Widenings of the plain enclosure of a root before it is polished wide:
/// up to `4^3` grains to either side.
const PLAIN_PROBES: usize = 4;

/// Knuth's `TwoSum`: `s = RN(a + b)` and the exact error `a + b − s`.
#[inline]
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let b_virtual = s - a;
    let a_virtual = s - b_virtual;
    (s, (a - a_virtual) + (b - b_virtual))
}

/// Dekker's `Fast2Sum`, exact when `a` is a multiple of `ulp(b)` (in
/// particular when `|a| ≥ |b|`).
#[inline]
fn fast_two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    (s, b - (s - a))
}

/// `p = RN(a·b)` and the exact error `a·b − p` (exact unless it underflows).
#[inline]
fn two_prod(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    (p, fma(a, b, -p))
}

/// `floor(log2 |x|)` of a finite nonzero `x`.
#[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
fn exponent(x: f64) -> i32 {
    let bits = x.to_bits();
    let biased = ((bits >> 52) & 0x7ff) as i32;
    if biased == 0 {
        // Subnormal: the significand's highest set bit.
        let significand = bits & ((1_u64 << 52) - 1);
        63 - significand.leading_zeros() as i32 - 1074
    } else {
        biased - 1023
    }
}

/// `2^k` for `-1022 ≤ k ≤ 1023`.
#[allow(clippy::cast_sign_loss)]
fn pow2(k: i32) -> f64 {
    f64::from_bits(((k + 1023) as u64) << 52)
}

/// `x · 2^k`, exact unless the result itself is subnormal or overflows.
fn ldexp(mut x: f64, mut k: i32) -> f64 {
    while k > 1023 {
        x *= pow2(1023);
        k -= 1023;
    }
    while k < -1022 {
        x *= pow2(-1022);
        k += 1022;
    }
    x * pow2(k)
}

/// A real number known to within a radius, in one of two precisions.
///
/// Every operation returns a ball that contains the exact result for every
/// pair of points of its operands' balls.
trait Ball: Copy + Add<Output = Self> + Sub<Output = Self> + Mul<Output = Self> {
    /// Whether an uncertified sign is final (no wider arithmetic follows).
    const FINAL: bool;
    /// Subdivision depth the isolation stops at.
    const DEPTH: u32;
    /// Evaluations the isolation may spend.
    const WORK: u32;
    /// The ball of radius zero at `x`.
    fn exact(x: f64) -> Self;
    /// The ball holding a wide ball.
    fn from_wide(w: Wide) -> Self;
    /// Scaled by `p`, a positive power of two.
    fn scale(self, p: f64) -> Self;
    /// The certified sign: `±1`, or `0` when zero is not excluded.
    fn sign(self) -> i8;
    /// Whether the value is at least `factor` radii from zero.
    fn clears(self, factor: f64) -> bool;
    /// The ball's centre, rounded.
    fn mid(self) -> f64;
}

/// A ball around an `f64`.
#[derive(Debug, Clone, Copy)]
struct Plain {
    v: f64,
    e: f64,
}

impl Plain {
    /// The square root, or `None` when the ball reaches zero.
    fn sqrt(self) -> Option<Self> {
        let low = self.v - self.e;
        (low > 0.0).then(|| {
            let v = self.v.sqrt();
            // |√x − √v| = |x − v| / (√x + √v) ≤ e / √low for x in the ball.
            Self {
                v,
                e: (self.e / low.sqrt() + U * v) * INFLATE,
            }
        })
    }
}

impl Add for Plain {
    type Output = Self;
    #[inline]
    fn add(self, o: Self) -> Self {
        let v = self.v + o.v;
        Self {
            v,
            e: (self.e + o.e + U * v.abs()) * INFLATE,
        }
    }
}

impl Sub for Plain {
    type Output = Self;
    #[inline]
    fn sub(self, o: Self) -> Self {
        let v = self.v - o.v;
        Self {
            v,
            e: (self.e + o.e + U * v.abs()) * INFLATE,
        }
    }
}

impl Mul for Plain {
    type Output = Self;
    #[inline]
    fn mul(self, o: Self) -> Self {
        let v = self.v * o.v;
        Self {
            v,
            e: (self.v.abs() * o.e + o.v.abs() * self.e + self.e * o.e + U * v.abs()) * INFLATE
                + TINY,
        }
    }
}

impl Ball for Plain {
    const FINAL: bool = false;
    const DEPTH: u32 = PLAIN_DEPTH;
    const WORK: u32 = PLAIN_WORK;

    #[inline]
    fn exact(x: f64) -> Self {
        Self { v: x, e: 0.0 }
    }

    #[inline]
    fn from_wide(w: Wide) -> Self {
        Self {
            v: w.hi,
            e: (w.lo.abs() + w.e) * INFLATE,
        }
    }

    #[inline]
    fn scale(self, p: f64) -> Self {
        Self {
            v: self.v * p,
            e: self.e * p + TINY,
        }
    }

    #[inline]
    fn sign(self) -> i8 {
        if self.v > self.e {
            1
        } else if self.v < -self.e {
            -1
        } else {
            0
        }
    }

    #[inline]
    fn clears(self, factor: f64) -> bool {
        self.v.abs() > factor * self.e
    }

    #[inline]
    fn mid(self) -> f64 {
        self.v
    }
}

/// A ball around a double-double `hi + lo` (`|lo| ≤ ulp(hi) / 2`).
///
/// The sum and product are the accurate double-word algorithms of Joldes,
/// Muller and Popescu, "Tight and rigorous error bounds for basic building
/// blocks of double-word arithmetic" (ACM TOMS 44, 2017): relative errors of
/// at most `3u²` (`AccurateDWPlusDW`, whatever the cancellation) and `7u²`
/// (`DWTimesDW1`), with `u = 2^-53`. The radii below take `4u²` and `8u²`.
#[derive(Debug, Clone, Copy)]
struct Wide {
    hi: f64,
    lo: f64,
    e: f64,
}

impl Add for Wide {
    type Output = Self;
    #[inline]
    fn add(self, o: Self) -> Self {
        let (sh, sl) = two_sum(self.hi, o.hi);
        let (th, tl) = two_sum(self.lo, o.lo);
        let c = sl + th;
        let (vh, vl) = fast_two_sum(sh, c);
        let w = tl + vl;
        let (hi, lo) = fast_two_sum(vh, w);
        Self {
            hi,
            lo,
            e: (self.e + o.e + 4.0 * U2 * hi.abs()) * INFLATE + TINY,
        }
    }
}

impl Sub for Wide {
    type Output = Self;
    #[inline]
    fn sub(self, o: Self) -> Self {
        self + Self {
            hi: -o.hi,
            lo: -o.lo,
            e: o.e,
        }
    }
}

impl Mul for Wide {
    type Output = Self;
    #[inline]
    fn mul(self, o: Self) -> Self {
        let (ch, cl) = two_prod(self.hi, o.hi);
        let cross = self.hi * o.lo + self.lo * o.hi;
        let (hi, lo) = fast_two_sum(ch, cl + cross);
        Self {
            hi,
            lo,
            e: (self.hi.abs() * o.e + o.hi.abs() * self.e + self.e * o.e + 8.0 * U2 * hi.abs())
                * INFLATE
                + TINY,
        }
    }
}

impl Ball for Wide {
    const FINAL: bool = true;
    const DEPTH: u32 = WIDE_DEPTH;
    const WORK: u32 = WIDE_WORK;

    #[inline]
    fn exact(x: f64) -> Self {
        Self {
            hi: x,
            lo: 0.0,
            e: 0.0,
        }
    }

    #[inline]
    fn from_wide(w: Wide) -> Self {
        w
    }

    #[inline]
    fn scale(self, p: f64) -> Self {
        Self {
            hi: self.hi * p,
            lo: self.lo * p,
            e: self.e * p + TINY,
        }
    }

    /// `|lo| ≤ u·|hi|`, so the value has `hi`'s sign once `|hi|` clears the
    /// radius by more than that.
    #[inline]
    fn sign(self) -> i8 {
        if self.hi > 1.5 * self.e {
            1
        } else if self.hi < -1.5 * self.e {
            -1
        } else {
            0
        }
    }

    #[inline]
    fn clears(self, factor: f64) -> bool {
        self.hi.abs() > 1.5 * factor * self.e
    }

    #[inline]
    fn mid(self) -> f64 {
        self.hi
    }
}

/// The line in exactly scaled, centre-relative coordinates.
///
/// Lengths are in units of `2^m` with `R + r` in `[1, 2)`, the direction is
/// `d = dir / 2^e` with its largest component in `[0.5, 1)`, and the line is
/// `q + σ·d`, where `q = o + τ0·d` is (to the rounding of `τ0`) its point
/// nearest the centre. The caller's parameter is `t = (τ0 + σ)·2^(m − e)`.
struct Chart {
    /// The base point `q`, with the bound on what its summation lost.
    q: [Wide; 3],
    /// The scaled direction, exact.
    d: [f64; 3],
    /// The torus frame's axes, rows `x`, `y`, `z`.
    axes: [[f64; 3]; 3],
    /// Scaled major radius, exact.
    big_r: f64,
    /// Scaled minor radius, exact.
    small_r: f64,
    /// Chart parameter of the caller's origin, negated.
    tau0: f64,
    /// `m − e`.
    t_exp: i32,
}

impl Chart {
    fn new(torus: &ToroidalSurface, origin: Point3, dir: Vec3) -> Option<Self> {
        let (big, small) = (torus.major_radius(), torus.minor_radius());
        let size = big + small;
        if !(big > 0.0 && small > 0.0 && size.is_finite()) {
            return None;
        }
        let dir = [dir.x(), dir.y(), dir.z()];
        let longest = dir.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if !(dir.iter().all(|v| v.is_finite()) && longest > 0.0) {
            return None;
        }
        let m = exponent(size);
        let e = exponent(longest) + 1;
        let d = dir.map(|v| ldexp(v, -e));
        let (big_r, small_r) = (ldexp(big, -m), ldexp(small, -m));
        if small_r < MIN_TUBE {
            return None;
        }
        let axes = [torus.x_axis(), torus.y_axis(), torus.z_axis()].map(|a| [a.x(), a.y(), a.z()]);
        if !axes.iter().flatten().all(|v| v.abs() <= 2.0) {
            return None;
        }
        // The one place absolute coordinates enter: `origin − centre`,
        // exactly, as a sum of two doubles.
        let c = torus.center();
        let o = [
            (origin.x(), c.x()),
            (origin.y(), c.y()),
            (origin.z(), c.z()),
        ]
        .map(|(p, c)| {
            let (hi, lo) = two_sum(p, -c);
            (ldexp(hi, -m), ldexp(lo, -m))
        });
        if !o.iter().all(|(hi, _)| hi.abs() <= MAX_REACH) {
            return None;
        }
        let along = o[0].0 * d[0] + o[1].0 * d[1] + o[2].0 * d[2];
        let tau0 = -along / (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if !tau0.is_finite() {
            return None;
        }
        let q = std::array::from_fn(|k| {
            // o + τ0·d as four exact terms, summed with every rounding
            // error kept but those of the three smallest.
            let (ph, pl) = two_prod(tau0, d[k]);
            let (s1, e1) = two_sum(o[k].0, ph);
            let (s2, e2) = two_sum(o[k].1, pl);
            let (s3, e3) = two_sum(s1, s2);
            let (hi, lo) = two_sum(s3, e1 + e2 + e3);
            Wide {
                hi,
                lo,
                e: 2.0 * U * (e1.abs() + e2.abs() + e3.abs()) * INFLATE + 8.0 * TINY,
            }
        });
        Some(Self {
            q,
            d,
            axes,
            big_r,
            small_r,
            tau0,
            t_exp: m - e,
        })
    }

    /// The spacing at which signs are probed around chart parameter `x`: two
    /// ulps of the caller's parameter there (and of `x` itself), no finer
    /// than [`GRAIN_FLOOR`].
    fn grain(&self, x: f64) -> f64 {
        (4.0 * U * (self.tau0 + x).abs().max(x.abs())).max(GRAIN_FLOOR)
    }

    /// The caller's parameter of chart parameter `x`.
    fn to_caller(&self, x: f64) -> f64 {
        ldexp(self.tau0 + x, self.t_exp)
    }
}

/// The line's quadratics in one precision.
struct Poly<B> {
    /// Frame coordinates `(a, b, c)(σ) = p0 + p1·σ`.
    p0: [B; 3],
    p1: [B; 3],
    /// `G = a² + b² + c² + R² − r²`, constant term first.
    g: [B; 3],
    /// `N = a² + b²`, constant term first.
    n: [B; 3],
    /// `4R²`.
    k: B,
    big_r: B,
    small_r: B,
}

/// The quartic at one chart parameter.
#[derive(Clone, Copy)]
struct Pt<B> {
    x: f64,
    g: B,
    n: B,
    q: B,
    /// Certified sign of `q`.
    s: i8,
}

impl<B: Ball> Poly<B> {
    fn new(chart: &Chart) -> Self {
        let dot = |axis: &[f64; 3], v: [B; 3]| {
            B::exact(axis[0]) * v[0] + B::exact(axis[1]) * v[1] + B::exact(axis[2]) * v[2]
        };
        let q = chart.q.map(B::from_wide);
        let d = chart.d.map(B::exact);
        let p0 = [0, 1, 2].map(|j| dot(&chart.axes[j], q));
        let p1 = [0, 1, 2].map(|j| dot(&chart.axes[j], d));
        let (big_r, small_r) = (B::exact(chart.big_r), B::exact(chart.small_r));
        let n = [
            p0[0] * p0[0] + p0[1] * p0[1],
            (p0[0] * p1[0] + p0[1] * p1[1]).scale(2.0),
            p1[0] * p1[0] + p1[1] * p1[1],
        ];
        let g = [
            n[0] + p0[2] * p0[2] + (big_r * big_r - small_r * small_r),
            n[1] + (p0[2] * p1[2]).scale(2.0),
            n[2] + p1[2] * p1[2],
        ];
        Self {
            p0,
            p1,
            g,
            n,
            k: (big_r * big_r).scale(4.0),
            big_r,
            small_r,
        }
    }

    fn quadratic(c: &[B; 3], x: f64) -> B {
        let x = B::exact(x);
        (c[2] * x + c[1]) * x + c[0]
    }

    fn point(&self, x: f64) -> Pt<B> {
        let g = Self::quadratic(&self.g, x);
        let n = Self::quadratic(&self.n, x);
        let q = g * g - self.k * n;
        Pt {
            x,
            g,
            n,
            q,
            s: q.sign(),
        }
    }

    /// The quartic's Bernstein coefficients on `[l.x, r.x]`, the middle one
    /// times six.
    ///
    /// A quadratic `c2 σ² + c1 σ + c0` has Bernstein coefficients
    /// `(f(u), f[u, v], f(v))` on `[u, v]`, with the blossom
    /// `f[u, v] = c2·u·v + c1·(u + v)/2 + c0`. The product of two such
    /// quadratics `(p0, p1, p2)` with themselves is
    /// `(p0², p0 p1, (2 p1² + p0 p2)/3, p1 p2, p2²)`, and a quadratic
    /// `(n0, n1, n2)` raised to degree four is
    /// `(n0, (n0 + n1)/2, (n0 + 4 n1 + n2)/6, (n1 + n2)/2, n2)`.
    fn bernstein(&self, l: &Pt<B>, r: &Pt<B>) -> [B; 5] {
        let (u, v) = (B::exact(l.x), B::exact(r.x));
        let (uv, half) = (u * v, (u + v).scale(0.5));
        let blossom = |c: &[B; 3]| c[2] * uv + c[1] * half + c[0];
        let (g1, n1) = (blossom(&self.g), blossom(&self.n));
        let b1 = l.g * g1 - self.k * (l.n + n1).scale(0.5);
        let b3 = g1 * r.g - self.k * (n1 + r.n).scale(0.5);
        let six_b2 =
            (g1 * g1).scale(4.0) + (l.g * r.g).scale(2.0) - self.k * (l.n + n1.scale(4.0) + r.n);
        [l.q, b1, six_b2, b3, r.q]
    }
}

impl Poly<Plain> {
    /// The quartic's certified sign at `x` from its factors
    /// `(ρ − R)² + c² − r²` and `(ρ + R)² + c² − r²`, which do not cancel
    /// the way `G² − 4R²N` does next to the surface.
    fn factored_sign(&self, x: f64) -> i8 {
        let x = Plain::exact(x);
        let [a, b, c] = [0, 1, 2].map(|k| self.p1[k] * x + self.p0[k]);
        let Some(rho) = (a * a + b * b).sqrt() else {
            return 0;
        };
        let axial = (c - self.small_r) * (c + self.small_r);
        let (inner, outer) = (rho - self.big_r, rho + self.big_r);
        (inner * inner + axial).sign() * (outer * outer + axial).sign()
    }

    /// The parameter interval every root lies in, `None` when the line
    /// certifiably has none, `Err` when plain balls cannot bound it.
    ///
    /// A root has `a² + b² + c² ≤ (R + r)²` (the bounding sphere) and
    /// `|c| ≤ r` (the slab). Both ends are moved outward well past the
    /// rounding of their own computation.
    fn root_interval(&self) -> Result<Option<(f64, f64)>, ()> {
        const PAD: f64 = 1.0 / 16_777_216.0; // 2^-24
        const REL: f64 = 1.0 / 1_048_576.0; // 2^-20
        // a² + b² + c² − (R + r)² = g2 σ² + g1 σ + (g0 − 2R(R + r)).
        let reach = self.big_r * (self.big_r + self.small_r);
        let w0 = self.g[0] - reach.scale(2.0);
        let disc = self.g[1] * self.g[1] - (self.g[2] * w0).scale(4.0);
        if disc.sign() < 0 {
            return Ok(None);
        }
        let g2 = self.g[2].v - self.g[2].e;
        // The padding below assumes radii at rounding level.
        let rounding = 1024.0 * U * (1.0 + self.g[0].v.abs());
        if !(g2 > 0.0 && self.g.iter().all(|c| c.e <= rounding)) {
            return Err(());
        }
        let centre = -self.g[1].v / (2.0 * self.g[2].v);
        let half = (disc.v + disc.e).max(0.0).sqrt() / (2.0 * g2);
        let pad = PAD + REL * half;
        let (mut lo, mut hi) = (centre - half - pad, centre + half + pad);
        let (c0, c1) = (self.p0[2], self.p1[2]);
        if c1.v.abs() >= 1.0 / 1024.0 && c0.e <= 16.0 * U && c1.e <= 16.0 * U {
            // Both ends are below 2^12 in magnitude and carry at most
            // 2^-27 from the radii of `c0` and `c1`: far inside the pad.
            let ends = [
                (-self.small_r.v - c0.v) / c1.v,
                (self.small_r.v - c0.v) / c1.v,
            ];
            let (first, last) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
            let pad = PAD + REL * (last - first);
            lo = lo.max(first - pad);
            hi = hi.min(last + pad);
        } else {
            // A line that never leaves one side of the slab.
            let farthest = lo.abs().max(hi.abs());
            let drift = (c1.v.abs() + c1.e) * farthest;
            if c0.v.abs() - c0.e - drift > self.small_r.v * (1.0 + REL) {
                return Ok(None);
            }
        }
        if !(lo.is_finite() && hi.is_finite()) {
            return Err(());
        }
        Ok((lo < hi).then_some((lo, hi)))
    }
}

/// The line in plain `f64`, for the iterations that only steer.
struct Flat {
    p0: [f64; 3],
    p1: [f64; 3],
    big_r: f64,
    small_r: f64,
    /// `|d|` in the frame's metric.
    speed: f64,
}

impl Flat {
    fn new(plain: &Poly<Plain>) -> Self {
        Self {
            p0: plain.p0.map(Ball::mid),
            p1: plain.p1.map(Ball::mid),
            big_r: plain.big_r.v,
            small_r: plain.small_r.v,
            speed: plain.g[2].v.sqrt(),
        }
    }

    /// The two factors of the quartic at `x` and their derivatives:
    /// `(F₋, F₊, F₋', F₊')` with `F∓ = (ρ ∓ R)² + c² − r²`.
    fn factors(&self, x: f64) -> (f64, f64, f64, f64) {
        let [a, b, c] = [0, 1, 2].map(|k| self.p1[k] * x + self.p0[k]);
        let rho = (a * a + b * b).sqrt();
        let axial = (c - self.small_r) * (c + self.small_r);
        let (inner, outer) = (rho - self.big_r, rho + self.big_r);
        let d_rho = if rho > 0.0 {
            (a * self.p1[0] + b * self.p1[1]) / rho
        } else {
            0.0
        };
        let d_axial = 2.0 * c * self.p1[2];
        (
            inner * inner + axial,
            outer * outer + axial,
            2.0 * inner * d_rho + d_axial,
            2.0 * outer * d_rho + d_axial,
        )
    }

    /// The quartic and its derivative at `x`, from the factors.
    fn value_slope(&self, x: f64) -> (f64, f64) {
        let (fm, fp, dfm, dfp) = self.factors(x);
        (fm * fp, dfm * fp + fm * dfp)
    }

    /// Sine of the angle between the line and the torus at the root `x`:
    /// the slope of the tube distance (of the factor that vanishes there)
    /// per unit length.
    fn crossing_sine(&self, x: f64) -> f64 {
        let (fm, fp, dfm, dfp) = self.factors(x);
        let slope = if fm.abs() <= fp.abs() { dfm } else { dfp };
        slope.abs() / (2.0 * self.small_r * self.speed)
    }
}

/// What the isolation found on one stretch of the line.
#[derive(Debug, Clone, Copy)]
enum Raw {
    /// Exactly one simple root in `(lo, hi)`; the quartic is negative at
    /// `lo` when `rising`.
    Root { lo: f64, hi: f64, rising: bool },
    /// An unresolved stretch `[lo, hi]` with certified signs at its ends,
    /// different ones when `odd`; `at` is where the sign first failed.
    Window {
        lo: f64,
        hi: f64,
        at: f64,
        odd: bool,
    },
}

/// Why an isolation stopped short.
#[derive(Debug, Clone, Copy)]
enum Stop {
    /// A sign this precision cannot certify.
    Precision,
    /// The evaluation budget ran out.
    Work,
}

/// The most sign changes a coefficient sequence can have, its uncertified
/// entries (`0`) taken whichever way gives more. The first and last entries
/// are certified.
fn max_variations(signs: [i8; 5]) -> u32 {
    let mut variations = 0;
    let mut last = signs[0];
    let mut unknown = 0;
    for &s in &signs[1..] {
        if s == 0 {
            unknown += 1;
            continue;
        }
        // `unknown + 1` gaps between `last` and `s`: every one can flip,
        // less one when that would give the wrong parity.
        let gaps = unknown + 1;
        variations += if (last != s) == (gaps % 2 == 1) {
            gaps
        } else {
            gaps - 1
        };
        last = s;
        unknown = 0;
    }
    variations
}

/// The root in `(lo, hi)` of a function negative at `lo` when `lo_negative`
/// and of the opposite sign at `hi`: Newton's method kept inside the
/// bracket, bisecting whenever a step would leave it or would not halve the
/// step before last, until a step is down to rounding or the bracket to
/// adjacent doubles. It only steers: nothing is certified from its result.
fn newton_bracketed(
    f: impl Fn(f64) -> (f64, f64),
    mut lo: f64,
    mut hi: f64,
    lo_negative: bool,
) -> f64 {
    let mut x = f64::midpoint(lo, hi);
    let mut step_before = hi - lo;
    let mut step = step_before;
    for _ in 0..REFINE_STEPS {
        let (fx, dfx) = f(x);
        if fx == 0.0 {
            return x;
        }
        if (fx < 0.0) == lo_negative {
            lo = x;
        } else {
            hi = x;
        }
        let newton = x - fx / dfx;
        if (newton - x).abs() <= 2.0 * f64::EPSILON * x.abs() + GRAIN_FLOOR {
            return if newton >= lo && newton <= hi {
                newton
            } else {
                x
            };
        }
        let stalls = (2.0 * fx).abs() > (step_before * dfx).abs();
        step_before = step;
        if newton > lo && newton < hi && !stalls {
            step = (newton - x).abs();
            x = newton;
        } else {
            let mid = f64::midpoint(lo, hi);
            if mid <= lo || mid >= hi {
                return x;
            }
            step = 0.5 * (hi - lo);
            x = mid;
        }
    }
    x
}

/// The roots in `(0, 1)` of the cubic with Bernstein coefficients `d`,
/// ascending: where to split an interval. Approximate; it only steers.
fn cubic_roots_in_unit(d: [f64; 4]) -> ([f64; 3], usize) {
    let c = [
        d[0],
        3.0 * (d[1] - d[0]),
        3.0 * (d[2] - 2.0 * d[1] + d[0]),
        d[3] - 3.0 * d[2] + 3.0 * d[1] - d[0],
    ];
    let mut out = ([0.0; 3], 0);
    if !c.iter().all(|v| v.is_finite()) {
        return out;
    }
    let value_slope = |s: f64| {
        (
            ((c[3] * s + c[2]) * s + c[1]) * s + c[0],
            (3.0 * c[3] * s + 2.0 * c[2]) * s + c[1],
        )
    };
    // The cubic is monotone between its own stationary points.
    let mut cuts = [0.0, 1.0, 1.0, 1.0];
    let mut cut_count = 1;
    let disc = c[2] * c[2] - 3.0 * c[3] * c[1];
    if disc > 0.0 {
        let q = -(c[2] + c[2].signum() * disc.sqrt());
        let mut turns = [q / (3.0 * c[3]), c[1] / q];
        if turns[0] > turns[1] {
            turns.swap(0, 1);
        }
        for s in turns {
            if s > 0.0 && s < 1.0 && s > cuts[cut_count - 1] {
                cuts[cut_count] = s;
                cut_count += 1;
            }
        }
    }
    cuts[cut_count] = 1.0;
    cut_count += 1;
    for pair in cuts[..cut_count].windows(2) {
        let (lo, hi) = (pair[0], pair[1]);
        let (f_lo, f_hi) = (value_slope(lo).0, value_slope(hi).0);
        let root = if f_lo == 0.0 && lo > 0.0 {
            lo
        } else if (f_lo < 0.0 && f_hi > 0.0) || (f_lo > 0.0 && f_hi < 0.0) {
            newton_bracketed(value_slope, lo, hi, f_lo < 0.0)
        } else {
            continue;
        };
        if out.1 < 3 && (out.1 == 0 || out.0[out.1 - 1] < root) {
            out.0[out.1] = root;
            out.1 += 1;
        }
    }
    out
}

/// Root isolation of one line in one precision.
struct Isolator<'a, B> {
    poly: &'a Poly<B>,
    chart: &'a Chart,
    out: Vec<Raw>,
    work: u32,
}

impl<'a, B: Ball> Isolator<'a, B> {
    fn new(poly: &'a Poly<B>, chart: &'a Chart) -> Self {
        Self {
            poly,
            chart,
            out: Vec::with_capacity(4),
            work: B::WORK,
        }
    }

    fn spend(&mut self) -> Result<(), Stop> {
        self.work = self.work.checked_sub(1).ok_or(Stop::Work)?;
        Ok(())
    }

    fn point(&mut self, x: f64) -> Result<Pt<B>, Stop> {
        self.spend()?;
        Ok(self.poly.point(x))
    }

    /// Every root in `[lo, hi]`, which must hold them all.
    fn run(mut self, lo: f64, hi: f64) -> Result<Vec<Raw>, Stop> {
        let (l, r) = (self.point(lo)?, self.point(hi)?);
        // Both ends lie outside the torus's bounding sphere or slab.
        if l.s != 1 || r.s != 1 {
            return Err(Stop::Precision);
        }
        self.resolve(l, r, 0)?;
        Ok(self.out)
    }

    /// Record what `[l.x, r.x]` holds; both ends carry certified signs.
    fn resolve(&mut self, l: Pt<B>, r: Pt<B>, depth: u32) -> Result<(), Stop> {
        self.spend()?;
        let b = self.poly.bernstein(&l, &r);
        let signs = b.map(Ball::sign);
        match (max_variations(signs), l.s == r.s) {
            (0, true) => return Ok(()),
            (1, false) => {
                self.out.push(Raw::Root {
                    lo: l.x,
                    hi: r.x,
                    rising: l.s < 0,
                });
                return Ok(());
            }
            _ => {}
        }
        let width = r.x - l.x;
        let mid = f64::midpoint(l.x, r.x);
        if depth >= B::DEPTH || width <= 6.0 * self.chart.grain(mid) {
            return self.window(&l, &r, mid);
        }
        if !B::FINAL && signs.contains(&0) {
            return Err(Stop::Precision);
        }
        let (splits, count) = self.splits(&l, &r, &b);
        let mut left = l;
        for &m in &splits[..count] {
            if m - left.x <= self.chart.grain(m) {
                continue;
            }
            let at = self.point(m)?;
            if at.s != 0 {
                self.resolve(left, at, depth + 1)?;
                left = at;
                continue;
            }
            if !B::FINAL {
                return Err(Stop::Precision);
            }
            // The split point is a root to within the arithmetic. Fence it
            // between the nearest points whose signs are certified.
            let before = self.fence_towards(m, &left, -1.0)?;
            let after = self.fence_towards(m, &r, 1.0)?;
            if before.x > left.x {
                self.resolve(left, before, depth + 1)?;
            }
            if after.x - before.x <= 0.5 * width {
                self.resolve(before, after, depth + 1)?;
            } else {
                self.window(&before, &after, m)?;
            }
            left = after;
        }
        if left.x < r.x {
            self.resolve(left, r, depth + 1)?;
        }
        Ok(())
    }

    fn window(&mut self, l: &Pt<B>, r: &Pt<B>, at: f64) -> Result<(), Stop> {
        if !B::FINAL {
            return Err(Stop::Precision);
        }
        self.out.push(Raw::Window {
            lo: l.x,
            hi: r.x,
            at: at.max(l.x).min(r.x),
            odd: l.s != r.s,
        });
        Ok(())
    }

    /// A point near `m` on the way to `end` (`side = ±1`) whose value
    /// clears its radius by [`WINDOW_MARGIN`]: the first to do so stepping
    /// out by fours from one grain, then brought back towards `m` by three
    /// bisections against the last step that did not; `end` itself when
    /// nothing nearer clears.
    fn fence_towards(&mut self, m: f64, end: &Pt<B>, side: f64) -> Result<Pt<B>, Stop> {
        let grain = self.chart.grain(m);
        let mut step = grain;
        let mut inner = 0.0;
        let mut found = loop {
            let x = m + side * step;
            if (end.x - x) * side <= 0.0 || !x.is_finite() {
                return Ok(*end);
            }
            let at = self.point(x)?;
            if at.q.clears(WINDOW_MARGIN) {
                break at;
            }
            inner = step;
            step *= 4.0;
        };
        for _ in 0..3 {
            let half = 0.5 * (inner + step);
            if half - inner < grain {
                break;
            }
            let at = self.point(m + side * half)?;
            if at.q.clears(WINDOW_MARGIN) {
                (found, step) = (at, half);
            } else {
                inner = half;
            }
        }
        Ok(found)
    }

    /// Where to split `[l.x, r.x]`: the quartic's critical points inside
    /// it, read off the interval's own Bernstein coefficients (so they are
    /// as accurate as the interval is narrow), or its midpoint.
    fn splits(&self, l: &Pt<B>, r: &Pt<B>, b: &[B; 5]) -> ([f64; 3], usize) {
        let six = |v: B| v.scale(4.0) + v.scale(2.0);
        let slope = [
            (b[1] - b[0]).mid(),
            (b[2] - six(b[1])).mid() / 6.0,
            (six(b[3]) - b[2]).mid() / 6.0,
            (b[4] - b[3]).mid(),
        ];
        let (roots, count) = cubic_roots_in_unit(slope);
        let width = r.x - l.x;
        let mut out = ([0.0; 3], 0);
        for &s in &roots[..count] {
            let x = l.x + s * width;
            let margin = 2.0 * self.chart.grain(x);
            let after_last = out.1 == 0 || x - out.0[out.1 - 1] > margin;
            if x - l.x > margin && r.x - x > margin && after_last {
                out.0[out.1] = x;
                out.1 += 1;
            }
        }
        if out.1 == 0 {
            out = ([f64::midpoint(l.x, r.x), 0.0, 0.0], 1);
        }
        out
    }
}

/// One contact of a line with a torus: see
/// [`intersect_line_torus_contacts`](super::intersect_line_torus_contacts).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineTorusContact {
    /// The contact's parameter, in units of the caller's direction: the
    /// root when `resolved`, otherwise a point of the unresolved stretch.
    pub t: f64,
    /// Lower end of the window that holds every root of this contact.
    pub t_lo: f64,
    /// Upper end of that window.
    pub t_hi: f64,
    /// What the contact certifies about crossing parity.
    pub kind: ContactKind,
    /// Whether the window holds exactly one simple root, with certified
    /// opposite signs of the quartic at its ends.
    pub resolved: bool,
}

/// Turns isolated roots into contacts: refines each simple root and
/// encloses it.
struct Finisher<'a> {
    chart: &'a Chart,
    plain: &'a Poly<Plain>,
    flat: Flat,
    wide: Option<Poly<Wide>>,
}

impl Finisher<'_> {
    fn wide(&mut self) -> &Poly<Wide> {
        let chart = self.chart;
        self.wide.get_or_insert_with(|| Poly::new(chart))
    }

    /// The quartic's certified sign at `x`: from its factors in plain
    /// balls, else from the wide quartic.
    fn sign(&mut self, x: f64) -> i8 {
        match self.plain.factored_sign(x) {
            0 => self.wide().point(x).s,
            s => s,
        }
    }

    /// Certified signs either side of `x` inside `(a, b)`, whose ends carry
    /// `below` and `-below`: the narrowest of up to `tries` windows, each
    /// four times the last, that the sign oracle certifies.
    fn enclose(
        &mut self,
        x: f64,
        (a, b): (f64, f64),
        below: i8,
        tries: usize,
        wide: bool,
    ) -> Option<(f64, f64)> {
        let mut reach = self.chart.grain(x);
        for _ in 0..tries {
            let (lo, hi) = ((x - reach).max(a), (x + reach).min(b));
            let mut sign = |p: f64| {
                if wide {
                    self.sign(p)
                } else {
                    self.plain.factored_sign(p)
                }
            };
            let s_lo = if lo <= a { below } else { sign(lo) };
            let s_hi = if hi >= b { -below } else { sign(hi) };
            if s_lo == below && s_hi == -below {
                return Some((lo, hi));
            }
            if lo <= a && hi >= b {
                break;
            }
            reach *= 4.0;
        }
        None
    }

    /// The simple root in `(lo, hi)` and a window around it with certified
    /// signs at both ends.
    fn refine(&mut self, lo: f64, hi: f64, rising: bool) -> (f64, f64, f64) {
        let below = if rising { -1 } else { 1 };
        let mut x = newton_bracketed(|s| self.flat.value_slope(s), lo, hi, rising);
        if let Some((a, b)) = self.enclose(x, (lo, hi), below, PLAIN_PROBES, false) {
            return (x, a, b);
        }
        // Too shallow for plain balls: Newton on the wide quartic's value
        // with the plain slope, inside a bracket of certified signs.
        let (mut a, mut b) = (lo, hi);
        for _ in 0..POLISH_STEPS {
            if !(x > a && x < b) {
                x = f64::midpoint(a, b);
                if !(x > a && x < b) {
                    break;
                }
            }
            let at = self.wide().point(x);
            if at.s == 0 {
                break;
            }
            if at.s == below {
                a = x;
            } else {
                b = x;
            }
            let grain = self.chart.grain(x);
            let next = x - at.q.mid() / self.flat.value_slope(x).1;
            let settled = (next - x).abs() <= 0.5 * grain;
            if next > a && next < b {
                x = next;
            } else if !settled {
                x = f64::midpoint(a, b);
            }
            if settled || b - a <= 2.0 * grain {
                break;
            }
        }
        let x = x.max(a).min(b);
        let (a, b) = self.enclose(x, (a, b), below, 64, true).unwrap_or((a, b));
        (x, a, b)
    }

    fn contact(&mut self, raw: Raw) -> LineTorusContact {
        let chart = self.chart;
        // The window's ends in the caller's parameter, moved one ulp out
        // for the rounding of `τ0 + σ`.
        let ends = |a: f64, b: f64| {
            (
                ldexp((chart.tau0 + a).next_down(), chart.t_exp),
                ldexp((chart.tau0 + b).next_up(), chart.t_exp),
            )
        };
        match raw {
            Raw::Root { lo, hi, rising } => {
                let (x, a, b) = self.refine(lo, hi, rising);
                let (t_lo, t_hi) = ends(a, b);
                let steep = self.flat.crossing_sine(x) >= TORUS_GRAZE_SLOPE;
                LineTorusContact {
                    t: chart.to_caller(x).max(t_lo).min(t_hi),
                    t_lo,
                    t_hi,
                    kind: if steep {
                        ContactKind::Transversal
                    } else {
                        ContactKind::Unclassified
                    },
                    resolved: true,
                }
            }
            Raw::Window { lo, hi, at, odd } => {
                let (t_lo, t_hi) = ends(lo, hi);
                LineTorusContact {
                    t: chart.to_caller(at).max(t_lo).min(t_hi),
                    t_lo,
                    t_hi,
                    kind: if odd {
                        ContactKind::Unclassified
                    } else {
                        ContactKind::Tangential
                    },
                    resolved: false,
                }
            }
        }
    }
}

/// The certified contacts of `origin + t·dir` with `torus`, ascending, or
/// `None` when the line has no chart or the wide solve ran out of budget.
pub(super) fn contacts_of_line(
    torus: &ToroidalSurface,
    origin: Point3,
    dir: Vec3,
) -> Option<Vec<LineTorusContact>> {
    let chart = Chart::new(torus, origin, dir)?;
    let plain = Poly::<Plain>::new(&chart);
    let Some((lo, hi)) = plain.root_interval().ok()? else {
        return Some(Vec::new());
    };
    let mut finisher = Finisher {
        chart: &chart,
        plain: &plain,
        flat: Flat::new(&plain),
        wide: None,
    };
    let found = match Isolator::new(&plain, &chart).run(lo, hi) {
        Ok(found) => found,
        Err(_) => Isolator::new(finisher.wide(), &chart).run(lo, hi).ok()?,
    };
    Some(found.into_iter().map(|raw| finisher.contact(raw)).collect())
}

#[cfg(test)]
mod tests;
