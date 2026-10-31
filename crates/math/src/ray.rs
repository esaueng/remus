//! Qualified ray–quadric intersection (RFC 0008 §3.2).
//!
//! A ray `origin + t·direction` meets a cylinder, cone or sphere where a
//! quadratic `a t² + 2h t + c = 0` vanishes. Every root comes back as a
//! [`RayHit`] whose [`ContactKind`] says whether its crossing parity is
//! certified:
//!
//! - [`ContactKind::Transversal`]: a simple root; the ray crosses the surface.
//! - [`ContactKind::Tangential`]: the discriminant is zero within its rounding
//!   bound, so the two roots coincide at `t` up to `spread`. The ray touches
//!   the surface without a parity change, or crosses it twice within
//!   `[t - spread, t + spread]`; either way an even count.
//! - [`ContactKind::Unclassified`]: the parity is not certified (a ray through
//!   a cone apex, a ray lying in the surface, a root whose position the
//!   rounding cannot fix). Parity counting abstains on it.
//!
//! A torus goes through the quartic solver instead (see [`ray_torus`]).
//!
//! The tangency band is relative: each discriminant is formed as a product
//! `(p − m)(p + m)` of two distances times the direction's length, so its
//! rounding bound scales with the coordinates' magnitude instead of being a
//! fixed absolute threshold that only ever fires at small scale.

use std::ops::Deref;

use crate::intersect::ContactKind;
use crate::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use crate::vec::{Point3, Vec3};

/// Relative rounding bound on the quantities below: well above the handful of
/// roundings each one takes (about ten `ε` in total), so a computed value
/// within `ERR · scale` of zero cannot be told from zero.
const ERR: f64 = 64.0 * f64::EPSILON;

/// One intersection of a ray with a surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// Ray parameter of the contact (not filtered by sign).
    pub t: f64,
    /// Whether the contact's crossing parity is certified.
    pub kind: ContactKind,
    /// Half-width, in `t`, of the window the contact's roots may occupy.
    /// Zero for a transversal root; infinite when the position is unknown.
    pub spread: f64,
}

impl RayHit {
    const fn transversal(t: f64) -> Self {
        Self {
            t,
            kind: ContactKind::Transversal,
            spread: 0.0,
        }
    }

    const fn unresolved(t: f64) -> Self {
        Self {
            t,
            kind: ContactKind::Unclassified,
            spread: f64::INFINITY,
        }
    }
}

/// Up to two hits, ordered by `t`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHits {
    hits: [RayHit; 2],
    len: usize,
}

impl RayHits {
    const EMPTY: Self = Self {
        hits: [RayHit::transversal(0.0); 2],
        len: 0,
    };

    fn push(&mut self, hit: RayHit) {
        if self.len < 2 {
            self.hits[self.len] = hit;
            self.len += 1;
        }
    }
}

impl Deref for RayHits {
    type Target = [RayHit];

    fn deref(&self) -> &[RayHit] {
        &self.hits[..self.len]
    }
}

/// `a t² + 2h t + c = 0` with its discriminant `h² − a c` supplied by the
/// caller, who can usually form it without cancellation, and an absolute
/// rounding bound on each quantity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quadratic {
    /// Leading coefficient.
    pub a: f64,
    /// Rounding bound on `a`.
    pub a_err: f64,
    /// Half the linear coefficient.
    pub h: f64,
    /// Rounding bound on `h`.
    pub h_err: f64,
    /// Constant coefficient.
    pub c: f64,
    /// Rounding bound on `c`.
    pub c_err: f64,
    /// Reduced discriminant `h² − a c`.
    pub disc: f64,
    /// Rounding bound on `disc`.
    pub disc_err: f64,
}

impl Quadratic {
    /// The quadratic from bare coefficients, with the discriminant formed
    /// directly and bounded relative to its terms (so cancellation widens the
    /// tangency band instead of flipping a sign).
    #[must_use]
    pub fn from_coefficients(a: f64, h: f64, c: f64) -> Self {
        Self {
            a,
            a_err: ERR * a.abs(),
            h,
            h_err: ERR * h.abs(),
            c,
            c_err: ERR * c.abs(),
            disc: h * h - a * c,
            disc_err: ERR * (h * h + (a * c).abs()),
        }
    }
}

/// All real roots of `q`, qualified (see the module docs).
///
/// A leading coefficient within its bound of zero leaves one finite root,
/// `−c / 2h`, transversal; the other root is at `−2h / a`, whose sign the
/// rounding cannot fix, so it comes back unresolved (and is omitted when
/// `a` is exactly zero, where it is at infinity). With `h` and `c` also
/// within their bounds of zero the whole line may lie in the surface: one
/// unresolved hit at `t = 0`. Non-finite input gives no hits.
#[must_use]
pub fn quadratic_roots(q: &Quadratic) -> RayHits {
    let mut hits = RayHits::EMPTY;
    if !(q.a.is_finite() && q.h.is_finite() && q.c.is_finite() && q.disc.is_finite()) {
        return hits;
    }
    if q.a.abs() <= q.a_err {
        if q.h.abs() > q.h_err {
            let near = RayHit::transversal(-q.c / (2.0 * q.h));
            if q.a == 0.0 {
                hits.push(near);
            } else {
                let far = RayHit::unresolved(-2.0 * q.h / q.a);
                let (first, second) = if far.t < near.t {
                    (far, near)
                } else {
                    (near, far)
                };
                hits.push(first);
                hits.push(second);
            }
        } else if q.c.abs() <= q.c_err {
            hits.push(RayHit::unresolved(0.0));
        }
        return hits;
    }
    if q.disc < -q.disc_err {
        return hits;
    }
    if q.disc <= q.disc_err {
        hits.push(RayHit {
            t: -q.h / q.a,
            kind: ContactKind::Tangential,
            spread: q.disc_err.sqrt() / q.a.abs(),
        });
        return hits;
    }
    // Cancellation-free pair: |k| >= sqrt(disc) > 0.
    let k = -(q.h + q.disc.sqrt().copysign(q.h));
    let (t0, t1) = (k / q.a, q.c / k);
    let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
    hits.push(RayHit::transversal(lo));
    hits.push(RayHit::transversal(hi));
    hits
}

/// The ray parameter where a ray meets a plane, unfiltered by sign.
///
/// For the plane `normal · x = d`, pass `denom = normal · direction` and
/// `numer = d − normal · origin`. `None` when `|denom| < parallel_below`:
/// the ray runs parallel to the plane.
#[must_use]
pub fn plane_parameter(denom: f64, numer: f64, parallel_below: f64) -> Option<f64> {
    if denom.abs() < parallel_below {
        return None;
    }
    Some(numer / denom)
}

/// How [`ray_cylinder_with`] and [`ray_cone_with`] treat a near-tangent
/// contact.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TangentPolicy {
    /// Qualify every root, as [`ray_cylinder`] and [`ray_cone`] do.
    Qualify,
    /// The GFA ray classifier's rule: a discriminant below
    /// `1e-12 · |a| · scale²` reads as a miss and gives no hits, and every
    /// root returned is [`ContactKind::Transversal`]. An axis-parallel ray
    /// (`a < 1e-14`) misses a cylinder; a cone ray along a generator
    /// (`|a| < 1e-14`) gives its one linear root, or none when the linear
    /// term is below `1e-14` too. `scale` is the cylinder's radius, or the
    /// largest radius of the cone's face.
    DropRelative {
        /// The length the tangency band scales with.
        scale: f64,
    },
}

/// [`ray_cylinder`] under a choice of [`TangentPolicy`].
#[must_use]
pub fn ray_cylinder_with(
    origin: Point3,
    direction: Vec3,
    cyl: &CylindricalSurface,
    policy: TangentPolicy,
) -> RayHits {
    match policy {
        TangentPolicy::Qualify => ray_cylinder(origin, direction, cyl),
        TangentPolicy::DropRelative { scale } => {
            ray_cylinder_dropping_tangents(origin, direction, cyl, scale)
        }
    }
}

/// [`ray_cone`] under a choice of [`TangentPolicy`].
#[must_use]
pub fn ray_cone_with(
    origin: Point3,
    direction: Vec3,
    cone: &ConicalSurface,
    policy: TangentPolicy,
) -> RayHits {
    match policy {
        TangentPolicy::Qualify => ray_cone(origin, direction, cone),
        TangentPolicy::DropRelative { scale } => {
            ray_cone_dropping_tangents(origin, direction, cone, scale)
        }
    }
}

/// Up to two transversal hits in increasing `t`.
fn transversal_pair(t0: f64, t1: f64) -> RayHits {
    let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
    let mut hits = RayHits::EMPTY;
    hits.push(RayHit::transversal(lo));
    hits.push(RayHit::transversal(hi));
    hits
}

/// [`TangentPolicy::DropRelative`] for a cylinder. The arithmetic, `mul_add`
/// placement included, is the GFA classifier's, so its roots are bit for bit
/// the ones it always computed.
fn ray_cylinder_dropping_tangents(
    origin: Point3,
    direction: Vec3,
    cyl: &CylindricalSurface,
    scale: f64,
) -> RayHits {
    let axis = cyl.axis();
    let m = origin - cyl.origin();
    let d_perp = direction - axis * direction.dot(axis);
    let m_perp = m - axis * m.dot(axis);
    let a = d_perp.dot(d_perp);
    if a < 1e-14 {
        return RayHits::EMPTY;
    }
    let b = 2.0 * m_perp.dot(d_perp);
    let c = cyl.radius().mul_add(-cyl.radius(), m_perp.dot(m_perp));
    let disc = b.mul_add(b, -4.0 * a * c);
    // `a` is a sum of squares, so `a.abs()` is `a` bit for bit.
    if disc < 1e-12 * a.abs() * scale * scale {
        return RayHits::EMPTY;
    }
    let sqrt_disc = disc.sqrt();
    transversal_pair((-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a))
}

/// [`TangentPolicy::DropRelative`] for a cone, on the cone condition
/// `h² − sin²a·|w|² = 0` (`h` axial, `w` from the apex). The arithmetic is
/// the GFA classifier's, bit for bit.
fn ray_cone_dropping_tangents(
    origin: Point3,
    direction: Vec3,
    cone: &ConicalSurface,
    scale: f64,
) -> RayHits {
    let axis = cone.axis();
    let m = origin - cone.apex();
    let sin_a = cone.half_angle().sin();
    let sin2 = sin_a * sin_a;
    let d_a = direction.dot(axis);
    let m_a = m.dot(axis);
    let a = sin2.mul_add(-direction.dot(direction), d_a * d_a);
    let half_b = sin2.mul_add(-direction.dot(m), d_a * m_a);
    let c = sin2.mul_add(-m.dot(m), m_a * m_a);
    if a.abs() < 1e-14 {
        if half_b.abs() < 1e-14 {
            return RayHits::EMPTY;
        }
        let mut hits = RayHits::EMPTY;
        hits.push(RayHit::transversal(-c / (2.0 * half_b)));
        return hits;
    }
    let disc = half_b.mul_add(half_b, -(a * c));
    if disc < 1e-12 * a.abs() * scale * scale {
        return RayHits::EMPTY;
    }
    let sqrt_disc = disc.sqrt();
    transversal_pair((-half_b - sqrt_disc) / a, (-half_b + sqrt_disc) / a)
}

/// Sum of absolute coordinates: the magnitude that rounding of a point's
/// coordinates scales with.
fn l1(p: Point3) -> f64 {
    p.x().abs() + p.y().abs() + p.z().abs()
}

/// Intersect a ray with an infinite cylinder.
///
/// A ray parallel to the axis (within rounding) meets no wall, or lies in
/// it: one unresolved hit at `t = 0`.
#[must_use]
pub fn ray_cylinder(origin: Point3, direction: Vec3, cyl: &CylindricalSurface) -> RayHits {
    let axis = cyl.axis();
    let r = cyl.radius();
    let ov = origin - cyl.origin();
    let o_perp = ov - axis * ov.dot(axis);
    let d_perp = direction - axis * direction.dot(axis);
    let scale = l1(origin) + l1(cyl.origin()) + r;
    let d_len = direction.length();
    let o_len = o_perp.length();
    let s = d_perp.length();
    if s <= ERR * d_len {
        return if (o_len - r).abs() <= ERR * scale {
            let mut hits = RayHits::EMPTY;
            hits.push(RayHit::unresolved(0.0));
            hits
        } else {
            RayHits::EMPTY
        };
    }
    // h² − a c = s² r² − |o⊥ × d⊥|² (Lagrange): `m / s` is the distance from
    // the axis to the line, so the band below is relative to the coordinates.
    let m = o_perp.cross(d_perp).length();
    let p = s * r;
    let gap_err = ERR * d_len * scale;
    quadratic_roots(&Quadratic {
        a: s * s,
        a_err: 0.0,
        h: o_perp.dot(d_perp),
        h_err: gap_err,
        c: (o_len - r) * (o_len + r),
        c_err: ERR * scale * (o_len + r),
        disc: (p - m) * (p + m),
        disc_err: gap_err * (p + m + gap_err),
    })
}

/// Intersect a ray with a sphere.
#[must_use]
pub fn ray_sphere(origin: Point3, direction: Vec3, sph: &SphericalSurface) -> RayHits {
    let r = sph.radius();
    let ov = origin - sph.center();
    let d_len = direction.length();
    let o_len = ov.length();
    let scale = l1(origin) + l1(sph.center()) + r;
    // h² − a c = |d|² r² − |ov × d|²: `m / |d|` is the distance from the
    // centre to the line.
    let m = ov.cross(direction).length();
    let p = d_len * r;
    let gap_err = ERR * d_len * scale;
    quadratic_roots(&Quadratic {
        a: d_len * d_len,
        a_err: 0.0,
        h: ov.dot(direction),
        h_err: gap_err,
        c: (o_len - r) * (o_len + r),
        c_err: ERR * scale * (o_len + r),
        disc: (p - m) * (p + m),
        disc_err: gap_err * (p + m + gap_err),
    })
}

/// Intersect a ray with an infinite double cone (both nappes).
///
/// A ray through the apex (within rounding) meets both nappes there: that
/// contact is a singular point, not a tangency, and comes back unresolved.
#[must_use]
#[allow(clippy::similar_names)]
pub fn ray_cone(origin: Point3, direction: Vec3, cone: &ConicalSurface) -> RayHits {
    let axis = cone.axis();
    let (sn, cs) = cone.half_angle().sin_cos();
    let ov = origin - cone.apex();
    let (oa, da) = (ov.dot(axis), direction.dot(axis));
    let o_perp = ov - axis * oa;
    let d_perp = direction - axis * da;
    let (op, dp) = (o_perp.length(), d_perp.length());
    let d_len = direction.length();
    let scale = l1(origin) + l1(cone.apex());
    // With Q(x) = cos²·x_a² − sin²·|x⊥|² and w = ov × d, the generalized
    // Lagrange identity gives B(ov, d)² − Q(ov) Q(d) =
    // sin²·(cos²·|w⊥|² − sin²·w_a²).
    let w = ov.cross(direction);
    let wa = w.dot(axis);
    let p = cs * (w - axis * wa).length();
    let m = sn * wa.abs();
    let gap_err = ERR * d_len * scale;
    let mut hits = quadratic_roots(&Quadratic {
        a: (cs * da - sn * dp) * (cs * da + sn * dp),
        a_err: ERR * d_len * d_len,
        h: cs * cs * oa * da - sn * sn * o_perp.dot(d_perp),
        h_err: gap_err,
        c: (cs * oa - sn * op) * (cs * oa + sn * op),
        c_err: ERR * scale * (oa.abs() + op),
        disc: sn * sn * (p - m) * (p + m),
        disc_err: sn * sn * gap_err * (p + m + gap_err),
    });
    if p + m <= gap_err {
        for hit in &mut hits.hits[..hits.len] {
            if hit.kind == ContactKind::Tangential {
                hit.kind = ContactKind::Unclassified;
            }
        }
    }
    hits
}

/// Below this angle (radians) between a ray and the torus at a lone root,
/// the root's partner may have been lost to the quartic solver's filters, so
/// the root's parity is not certified. A double root resolves to about
/// `sqrt(ε)` relative, i.e. a slope of order `1e-8`; this keeps two decades
/// of margin while leaving any usefully transversal crossing alone.
const TORUS_GRAZE_SLOPE: f64 = 1e-6;

/// Intersect a ray with a torus: every real root of the quartic, qualified.
///
/// Wraps [`crate::analytic_intersection::intersect_line_torus`], whose
/// de-dup merges a tangent line's double root into one root that a parity
/// count would read as one crossing. Here a root that absorbed an even number
/// of candidates is [`ContactKind::Tangential`], an odd number above one is
/// [`ContactKind::Unclassified`] (an inflectional contact may cross), and a
/// lone root meeting the surface within `1e-6` rad of tangency is
/// [`ContactKind::Unclassified`] too. `spread` is the solver's merge radius
/// `1e-9 (1 + |t|)` for every non-transversal hit. The roots are the same
/// values, in the same order, as `intersect_line_torus` returns.
#[must_use]
pub fn ray_torus(origin: Point3, direction: Vec3, torus: &ToroidalSurface) -> Vec<RayHit> {
    let c = torus.center();
    let (xa, ya, za) = (torus.x_axis(), torus.y_axis(), torus.z_axis());
    let big_r = torus.major_radius();
    let d = Vec3::new(xa.dot(direction), ya.dot(direction), za.dot(direction));
    let d_len = d.length();
    crate::analytic_intersection::intersect_line_torus_counted(torus, origin, direction)
        .into_iter()
        .map(|(t, count)| {
            let kind = if count % 2 == 0 {
                ContactKind::Tangential
            } else if count > 1 {
                ContactKind::Unclassified
            } else {
                // Gradient of (ρ − R)² + z² in the torus frame.
                let q = origin + direction * t - c;
                let (a, b, z) = (xa.dot(q), ya.dot(q), za.dot(q));
                let rho = a.hypot(b);
                let k = if rho > 0.0 { (rho - big_r) / rho } else { 0.0 };
                let grad = Vec3::new(k * a, k * b, z);
                let norm = grad.length() * d_len;
                if norm > 0.0 && grad.dot(d).abs() >= TORUS_GRAZE_SLOPE * norm {
                    ContactKind::Transversal
                } else {
                    ContactKind::Unclassified
                }
            };
            let spread = if kind == ContactKind::Transversal {
                0.0
            } else {
                1e-9 * (1.0 + t.abs())
            };
            RayHit { t, kind, spread }
        })
        .collect()
}

#[cfg(test)]
mod tests;
