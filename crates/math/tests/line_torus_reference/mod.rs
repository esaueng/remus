//! An independent double-double reference for line–torus contacts, the
//! seeded line families it is run over, and the judge that compares a
//! solver's hits with it.
//!
//! The reference shares nothing with the solver under test. It takes the
//! `f64` inputs as exact, subtracts the torus centre from the line origin
//! without error, and works in double-double arithmetic (about 32 digits)
//! throughout:
//!
//! 1. the line–torus quartic `Q = G² − 4R²N` and its derivatives are
//!    evaluated from the two quadratics `G` and `N`, never from expanded
//!    coefficients;
//! 2. the inflection points (roots of `Q''`, a quadratic) come in closed
//!    form; between them `Q'` is monotone, so each critical point is
//!    bracketed and bisected;
//! 3. between consecutive critical points `Q` is monotone, so each sign
//!    change is one root, bisected on the tube's own implicit
//!    `(ρ − R)² + z² − r²` (the signed distance times a positive factor).
//!
//! A critical value the reference cannot tell from zero (an exact tangency)
//! is reported as an ambiguous contact with the parity of the signs on its
//! two sides, and nothing is asserted about how a solver splits it.
//!
//! The reference locates roots; it does not decide anything at an `f64`
//! value of the parameter. For that, [`exact`] evaluates the quartic's sign
//! at an `f64` exactly, in dyadic big-number arithmetic, and
//! [`judge_windows`] holds a solver's windows to the contract of the
//! caller's parameter with it: opposite exact signs at the two ends of a
//! resolved window, the exact parity of an unresolved one, disjoint windows,
//! and no two separable roots in one window.
//!
//! Apart from [`judge_windows`], which reads
//! `intersect_line_torus_contacts`, only `remus_math::ray::ray_torus` and
//! public value types are used, so the harness also compiles against older
//! trees.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::many_single_char_names,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    dead_code
)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::ops::{Add, Mul, Neg, Sub};

use remus_math::analytic_intersection::LineTorusContact;
use remus_math::intersect::ContactKind;
use remus_math::ray::RayHit;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};

pub mod exact;

/// A double-double: the unevaluated sum of two doubles.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dd(pub f64, pub f64);

fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}

fn two_prod(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    (p, a.mul_add(b, -p))
}

impl Dd {
    pub const ZERO: Self = Self(0.0, 0.0);

    pub fn of(x: f64) -> Self {
        Self(x, 0.0)
    }

    /// `a − b` exactly.
    pub fn diff(a: f64, b: f64) -> Self {
        let (s, e) = two_sum(a, -b);
        Self(s, e)
    }

    pub fn value(self) -> f64 {
        self.0 + self.1
    }

    pub fn sign(self) -> i8 {
        if self.0 > 0.0 || (self.0 == 0.0 && self.1 > 0.0) {
            1
        } else if self.0 < 0.0 || (self.0 == 0.0 && self.1 < 0.0) {
            -1
        } else {
            0
        }
    }

    pub fn abs(self) -> Self {
        if self.sign() < 0 { -self } else { self }
    }

    pub fn half(self) -> Self {
        Self(0.5 * self.0, 0.5 * self.1)
    }

    pub fn times(self, k: f64) -> Self {
        self * Self::of(k)
    }

    pub fn div(self, o: Self) -> Self {
        let q1 = self.0 / o.0;
        let r = self - o.times(q1);
        let q2 = r.0 / o.0;
        let r = r - o.times(q2);
        let q3 = r.0 / o.0;
        Self::of(q1) + Self::of(q2) + Self::of(q3)
    }

    pub fn sqrt(self) -> Self {
        if self.0 <= 0.0 {
            return Self::ZERO;
        }
        let x = 1.0 / self.0.sqrt();
        let ax = self.0 * x;
        let correction = (self - Self::of(ax) * Self::of(ax)).0 * x * 0.5;
        // One more Newton step in double-double.
        let y = Self::of(ax) + Self::of(correction);
        y + (self - y * y).div(y.times(2.0))
    }

    pub fn lt(self, o: Self) -> bool {
        (self - o).sign() < 0
    }
}

impl Add for Dd {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        let (s, e) = two_sum(self.0, o.0);
        let (t, f) = two_sum(self.1, o.1);
        let (s, e) = two_sum(s, e + t);
        let (hi, lo) = two_sum(s, e + f);
        Self(hi, lo)
    }
}

impl Neg for Dd {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0, -self.1)
    }
}

impl Sub for Dd {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        self + (-o)
    }
}

impl Mul for Dd {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let (p, e) = two_prod(self.0, o.0);
        let (hi, lo) = two_sum(p, e + (self.0 * o.1 + self.1 * o.0));
        Self(hi, lo)
    }
}

/// What the reference finds on one line.
#[derive(Debug, Clone)]
pub struct Truth {
    /// The simple roots, ascending, in the caller's parameter.
    pub roots: Vec<Dd>,
    /// Contacts the reference cannot resolve (exact tangencies): where, and
    /// whether the quartic's sign differs across them.
    pub ambiguous: Vec<(f64, bool)>,
    /// `(R + r) / |dir|`: the torus's size in units of the parameter.
    pub size_t: f64,
}

/// The line in the torus frame, in double-double, about its point nearest
/// the centre.
struct Line {
    p0: [Dd; 3],
    p1: [Dd; 3],
    g: [Dd; 3],
    n: [Dd; 3],
    four_rr: Dd,
    big_r: Dd,
    small_r: Dd,
    /// Parameter of the base point.
    base: Dd,
}

impl Line {
    fn new(torus: &ToroidalSurface, origin: Point3, dir: Vec3) -> Self {
        let c = torus.center();
        let o = [
            Dd::diff(origin.x(), c.x()),
            Dd::diff(origin.y(), c.y()),
            Dd::diff(origin.z(), c.z()),
        ];
        let d = [Dd::of(dir.x()), Dd::of(dir.y()), Dd::of(dir.z())];
        let dot = |a: [Dd; 3], b: [Dd; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let base = -dot(o, d).div(dot(d, d));
        let q = [0, 1, 2].map(|k| o[k] + base * d[k]);
        let axes = [torus.x_axis(), torus.y_axis(), torus.z_axis()]
            .map(|a| [Dd::of(a.x()), Dd::of(a.y()), Dd::of(a.z())]);
        let p0 = axes.map(|a| dot(a, q));
        let p1 = axes.map(|a| dot(a, d));
        let (big_r, small_r) = (Dd::of(torus.major_radius()), Dd::of(torus.minor_radius()));
        let n = [
            p0[0] * p0[0] + p0[1] * p0[1],
            (p0[0] * p1[0] + p0[1] * p1[1]).times(2.0),
            p1[0] * p1[0] + p1[1] * p1[1],
        ];
        let g = [
            n[0] + p0[2] * p0[2] + big_r * big_r - small_r * small_r,
            n[1] + (p0[2] * p1[2]).times(2.0),
            n[2] + p1[2] * p1[2],
        ];
        Self {
            p0,
            p1,
            g,
            n,
            four_rr: (big_r * big_r).times(4.0),
            big_r,
            small_r,
            base,
        }
    }

    fn quadratic(c: &[Dd; 3], x: Dd) -> (Dd, Dd) {
        ((c[2] * x + c[1]) * x + c[0], c[2].times(2.0) * x + c[1])
    }

    /// The quartic `G² − 4R²N`.
    fn q(&self, x: Dd) -> Dd {
        let (g, _) = Self::quadratic(&self.g, x);
        let (n, _) = Self::quadratic(&self.n, x);
        g * g - self.four_rr * n
    }

    /// Its derivative `2GG' − 4R²N'`.
    fn dq(&self, x: Dd) -> Dd {
        let (g, dg) = Self::quadratic(&self.g, x);
        let (_, dn) = Self::quadratic(&self.n, x);
        (g * dg).times(2.0) - self.four_rr * dn
    }

    /// The tube implicit `(ρ − R)² + z² − r²`: the signed distance to the
    /// tube times the positive `distance + 2r`.
    fn tube(&self, x: Dd) -> Dd {
        let [a, b, c] = [0, 1, 2].map(|k| self.p0[k] + self.p1[k] * x);
        let w = (a * a + b * b).sqrt() - self.big_r;
        w * w + c * c - self.small_r * self.small_r
    }
}

/// The root of `f` in `(lo, hi)`, where it has opposite signs, by bisection
/// to double-double precision.
fn bisect(f: impl Fn(Dd) -> Dd, mut lo: Dd, mut hi: Dd) -> Dd {
    let rising = f(lo).sign() < 0;
    for _ in 0..125 {
        let mid = (lo + hi).half();
        let s = f(mid).sign();
        if s == 0 {
            return mid;
        }
        if (s < 0) == rising {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi).half()
}

/// Every real contact of `origin + t·dir` with `torus` (a ring torus,
/// `R > r`), from the represented inputs taken as exact.
pub fn reference(torus: &ToroidalSurface, origin: Point3, dir: Vec3) -> Truth {
    let line = Line::new(torus, origin, dir);
    let size = torus.major_radius() + torus.minor_radius();
    let speed = line.g[2].value().sqrt();
    let size_t = size / speed;
    let mut truth = Truth {
        roots: Vec::new(),
        ambiguous: Vec::new(),
        size_t,
    };
    // Every root is within R + r of the centre, the base point is the
    // nearest one, so |x| ≤ (R + r) / |dir|.
    let reach = Dd::of(1.001 * size_t);
    // Q'' = 12 g2² x² + 12 g1 g2 x + 2 g1² + 4 g0 g2 − 8 R² n2.
    let [g0, g1, g2] = line.g;
    let a = (g2 * g2).times(12.0);
    let b = (g1 * g2).times(12.0);
    let c = (g1 * g1).times(2.0) + (g0 * g2).times(4.0) - line.four_rr * line.n[2].times(2.0);
    let disc = b * b - (a * c).times(4.0);
    let mut cuts = vec![-reach];
    if disc.sign() >= 0 {
        let root = disc.sqrt();
        for x in [(-b - root).div(a.times(2.0)), (-b + root).div(a.times(2.0))] {
            if cuts.last().unwrap().lt(x) && x.lt(reach) {
                cuts.push(x);
            }
        }
    }
    cuts.push(reach);
    // Critical points: one per monotone stretch of Q' that changes sign.
    let mut points = vec![-reach];
    for pair in cuts.windows(2) {
        let (lo, hi) = (pair[0], pair[1]);
        let (s_lo, s_hi) = (line.dq(lo).sign(), line.dq(hi).sign());
        if s_lo == 0 && points.last().unwrap().lt(lo) {
            points.push(lo);
        } else if s_lo * s_hi < 0 {
            points.push(bisect(|x| line.dq(x), lo, hi));
        }
    }
    points.push(reach);
    // The reference's own noise on Q is about 1e-31 of (R + r)⁴ |dir|⁰ in
    // these units; anything under 1e-24 of it is called ambiguous.
    let floor = 1e-24 * size.powi(4);
    let signs: Vec<i8> = points
        .iter()
        .map(|&x| {
            let q = line.q(x);
            if q.abs().value() <= floor {
                0
            } else {
                q.sign()
            }
        })
        .collect();
    assert!(signs[0] == 1 && signs[signs.len() - 1] == 1, "{signs:?}");
    let mut i = 0;
    while i + 1 < points.len() {
        let mut j = i + 1;
        while signs[j] == 0 {
            j += 1;
        }
        if j == i + 1 {
            if signs[i] != signs[j] {
                let (lo, hi) = (points[i], points[j]);
                // The tube implicit has the quartic's sign on a ring torus.
                assert_eq!(line.tube(lo).sign(), signs[i]);
                assert_eq!(line.tube(hi).sign(), signs[j]);
                let x = bisect(|x| line.tube(x), lo, hi);
                truth.roots.push(line.base + x);
            }
        } else {
            let at = (points[i + 1] + points[j - 1]).half();
            truth
                .ambiguous
                .push(((line.base + at).value(), signs[i] != signs[j]));
        }
        i = j;
    }
    truth
}

/// `SplitMix64`: deterministic samples without a dependency.
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1_u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.unit(), lo)
    }

    pub fn sign(&mut self) -> f64 {
        if self.next() & 1 == 0 { 1.0 } else { -1.0 }
    }

    pub fn direction(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            let l = v.length();
            if l > 0.1 && l <= 1.0 {
                return v * (1.0 / l);
            }
        }
    }
}

/// One line of a family.
#[derive(Debug, Clone)]
pub struct Case {
    pub family: &'static str,
    pub torus: ToroidalSurface,
    pub origin: Point3,
    pub dir: Vec3,
}

/// The point at `(u, v)` with the unit circumferential and meridional
/// tangents and the outward normal there.
pub fn frame(t: &ToroidalSurface, u: f64, v: f64) -> (Point3, Vec3, Vec3, Vec3) {
    let radial = t.x_axis() * u.cos() + t.y_axis() * u.sin();
    let e_u = t.y_axis() * u.cos() - t.x_axis() * u.sin();
    let e_v = t.z_axis() * v.cos() - radial * v.sin();
    let n = radial * v.cos() + t.z_axis() * v.sin();
    (t.evaluate(u, v), e_u, e_v, n)
}

/// The tangent direction at `(u, v)` on the inner half (`cos v < 0`) whose
/// normal curvature is `ratio` times the asymptotic one's: `ratio = 1` is an
/// asymptotic direction (a triple contact), below one the line enters the
/// tube.
pub fn inner_direction(t: &ToroidalSurface, u: f64, v: f64, ratio: f64, sign: f64) -> Vec3 {
    let (_, e_u, e_v, _) = frame(t, u, v);
    let rho = t.major_radius() + t.minor_radius() * v.cos();
    let slope = ratio * (-t.minor_radius() * v.cos() / rho).sqrt();
    (e_u + e_v * (sign * slope)).normalize().unwrap()
}

/// The kinds of tangent line the families build, each with its contact
/// parameter `v` range.
#[derive(Clone, Copy, Debug)]
pub enum Tangent {
    /// Any tangent direction on the convex outer half: a double contact.
    Exterior,
    /// A direction entering the tube on the saddle-shaped inner half: a
    /// double contact from inside.
    Interior,
    /// An asymptotic direction of the inner half: a triple contact, where
    /// the line crosses.
    Asymptotic,
    /// The tube top along its circle: a quadruple contact.
    Quadruple,
}

/// A tangent line of the given kind through a random point of the torus:
/// the contact point, the unit direction and the outward normal.
pub fn tangent(rng: &mut Rng, torus: &ToroidalSurface, kind: Tangent) -> (Point3, Vec3, Vec3) {
    let u = rng.range(0.0, TAU);
    match kind {
        Tangent::Exterior => {
            let v = rng.range(-FRAC_PI_2 + 0.05, FRAC_PI_2 - 0.05);
            let (p, e_u, e_v, n) = frame(torus, u, v);
            let theta = rng.range(0.0, PI);
            (p, e_u * theta.cos() + e_v * theta.sin(), n)
        }
        Tangent::Interior => {
            let v = rng.range(FRAC_PI_2 + 0.05, 3.0 * FRAC_PI_2 - 0.05);
            let (p, _, _, n) = frame(torus, u, v);
            let (ratio, sign) = (rng.range(0.0, 0.8), rng.sign());
            (p, inner_direction(torus, u, v, ratio, sign), n)
        }
        Tangent::Asymptotic => {
            let v = rng.range(FRAC_PI_2 + 0.1, 3.0 * FRAC_PI_2 - 0.1);
            let (p, _, _, n) = frame(torus, u, v);
            let sign = rng.sign();
            (p, inner_direction(torus, u, v, 1.0, sign), n)
        }
        Tangent::Quadruple => {
            let v = FRAC_PI_2 * rng.sign();
            let (p, e_u, _, n) = frame(torus, u, v);
            (p, e_u, n)
        }
    }
}

const TANGENTS: [Tangent; 4] = [
    Tangent::Exterior,
    Tangent::Interior,
    Tangent::Asymptotic,
    Tangent::Quadruple,
];

/// Offsets from tangency, in torus sizes, either side.
const OFFSETS: [f64; 6] = [0.0, 1e-15, 1e-12, 1e-9, 1e-6, 1e-3];

/// A torus of the given size and aspect ratio `R / r`, tilted or
/// axis-aligned, centred at `centre` plus a few sizes.
fn torus(rng: &mut Rng, size: f64, aspect: f64, tilted: bool, centre: f64) -> ToroidalSurface {
    let small = size / (1.0 + aspect) * rng.range(0.8, 1.0);
    let big = small * aspect;
    let c = Point3::new(
        centre + rng.range(-3.0, 3.0) * size,
        centre + rng.range(-3.0, 3.0) * size,
        centre + rng.range(-3.0, 3.0) * size,
    );
    if tilted {
        ToroidalSurface::with_axis(c, big, small, rng.direction()).unwrap()
    } else {
        ToroidalSurface::new(c, big, small).unwrap()
    }
}

/// A chord aimed at a random point within `1.3 r` of the tube's core
/// circle, from `back` sizes away.
fn chord(rng: &mut Rng, t: &ToroidalSurface, back: f64) -> (Point3, Vec3) {
    let (u, v) = (rng.range(0.0, TAU), rng.range(0.0, TAU));
    let (_, _, _, n) = frame(t, u, v);
    let radial = t.x_axis() * u.cos() + t.y_axis() * u.sin();
    let core = t.center() + radial * t.major_radius();
    let target = core + n * (t.minor_radius() * rng.range(0.0, 1.3));
    let dir = rng.direction();
    let size = t.major_radius() + t.minor_radius();
    (target - dir * (back * size), dir)
}

/// The seeded line families.
pub fn cases() -> Vec<Case> {
    cases_seeded(0x1638_F185_D1FF_0004)
}

/// The line families drawn from `seed`.
pub fn cases_seeded(seed: u64) -> Vec<Case> {
    let mut rng = Rng(seed);
    let mut out = Vec::new();
    let mut push = |family: &'static str, torus: &ToroidalSurface, origin: Point3, dir: Vec3| {
        out.push(Case {
            family,
            torus: torus.clone(),
            origin,
            dir,
        });
    };
    let scales = [1e-3, 1.0, 1e6];
    let aspects = [1.01, 3.0, 100.0, 3000.0];

    // Random chords.
    for &size in &scales {
        for &aspect in &aspects {
            for tilted in [false, true] {
                let t = torus(&mut rng, size, aspect, tilted, 0.0);
                for _ in 0..36 {
                    let back = rng.range(0.0, 4.0);
                    let (origin, dir) = chord(&mut rng, &t, back);
                    push("chords", &t, origin, dir);
                }
            }
        }
    }

    // Lines moved off a tangency along the normal, either side.
    for &size in &scales {
        for &aspect in &aspects {
            for tilted in [false, true] {
                let t = torus(&mut rng, size, aspect, tilted, 0.0);
                let total = t.major_radius() + t.minor_radius();
                for kind in TANGENTS {
                    for &offset in &OFFSETS {
                        for side in [-1.0, 1.0] {
                            let (p, unit, n) = tangent(&mut rng, &t, kind);
                            let back = rng.range(0.2, 3.0) * total;
                            let origin = p + n * (side * offset * total) - unit * back;
                            let family = match kind {
                                Tangent::Exterior => "tangent/exterior",
                                Tangent::Interior => "tangent/interior",
                                Tangent::Asymptotic => "tangent/asymptotic",
                                Tangent::Quadruple => "tangent/quadruple",
                            };
                            push(family, &t, origin, unit);
                        }
                    }
                }
            }
        }
    }

    // Large thin tori, the origin a hair inside or outside the tube.
    for &size in &scales {
        for &aspect in &[100.0, 1000.0, 3000.0] {
            let t = torus(&mut rng, size, aspect, false, 0.0);
            let small = t.minor_radius();
            for &depth in &[2e-8, 1e-6, 1e-3] {
                for side in [-1.0, 1.0] {
                    for v in [0.0, FRAC_PI_2, PI, rng.range(0.0, TAU)] {
                        let u = rng.range(0.0, TAU);
                        let (p, e_u, e_v, n) = frame(&t, u, v);
                        let origin = p + n * (side * depth * small);
                        for dir in [
                            t.z_axis(),
                            -t.z_axis(),
                            e_u,
                            -e_u,
                            e_v,
                            n,
                            -n,
                            rng.direction(),
                        ] {
                            push("thin/near-tube", &t, origin, dir);
                        }
                    }
                }
            }
        }
    }

    // Tori far from the world origin: the coordinates carry the centre's
    // rounding.
    for (centre, family) in [
        (1e6, "far-centre/1e6"),
        (1e9, "far-centre/1e9"),
        (1e12, "far-centre/1e12"),
    ] {
        for &aspect in &[3.0, 100.0] {
            for tilted in [false, true] {
                let t = torus(&mut rng, 2.0, aspect, tilted, centre);
                let total = t.major_radius() + t.minor_radius();
                for _ in 0..20 {
                    let back = rng.range(0.0, 2.0);
                    let (origin, dir) = chord(&mut rng, &t, back);
                    push(family, &t, origin, dir);
                }
                for kind in TANGENTS {
                    for &offset in &[0.0, 1e-9, 1e-6, 1e-4, 1e-2] {
                        for side in [-1.0, 1.0] {
                            let (p, unit, n) = tangent(&mut rng, &t, kind);
                            let back = rng.range(0.2, 2.0) * total;
                            let origin = p + n * (side * offset * total) - unit * back;
                            push(family, &t, origin, unit);
                        }
                    }
                }
            }
        }
    }

    // Ray origins far from the torus.
    for (back, family) in [
        (1e2, "far-origin/1e2"),
        (1e3, "far-origin/1e3"),
        (1e4, "far-origin/1e4"),
    ] {
        for &size in &scales {
            for &aspect in &[3.0, 100.0] {
                let t = torus(&mut rng, size, aspect, true, 0.0);
                let total = t.major_radius() + t.minor_radius();
                for _ in 0..12 {
                    let (origin, dir) = chord(&mut rng, &t, back);
                    push(family, &t, origin, dir);
                }
                for kind in TANGENTS {
                    for &offset in &[0.0, 1e-9, 1e-6, 1e-3] {
                        for side in [-1.0, 1.0] {
                            let (p, unit, n) = tangent(&mut rng, &t, kind);
                            let origin = p + n * (side * offset * total) - unit * (back * total);
                            push(family, &t, origin, unit);
                        }
                    }
                }
            }
        }
    }

    // Directions that are not unit length.
    for (length, family) in [
        (1e-5, "direction/1e-5"),
        (1e-2, "direction/1e-2"),
        (1e2, "direction/1e2"),
        (1e4, "direction/1e4"),
    ] {
        for &size in &scales {
            let t = torus(&mut rng, size, 3.0, true, 0.0);
            let total = t.major_radius() + t.minor_radius();
            for _ in 0..12 {
                let back = rng.range(0.0, 3.0);
                let (origin, dir) = chord(&mut rng, &t, back);
                push(family, &t, origin, dir * length);
            }
            for kind in TANGENTS {
                for &offset in &[0.0, 1e-9, 1e-6, 1e-3] {
                    for side in [-1.0, 1.0] {
                        let (p, unit, n) = tangent(&mut rng, &t, kind);
                        let back = rng.range(0.2, 3.0) * total;
                        let origin = p + n * (side * offset * total) - unit * back;
                        push(family, &t, origin, unit * length);
                    }
                }
            }
        }
    }
    out
}

/// What went wrong on a family's lines.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Tally {
    pub lines: usize,
    /// Reference roots, and how many came back as their own hit.
    pub roots: usize,
    pub separate: usize,
    /// Reference roots no hit accounts for.
    pub lost: usize,
    /// Crossing hits with no reference root, and touches where the
    /// reference finds neither a root pair nor a tangency.
    pub phantom: usize,
    /// A touch holding an odd number of roots, or a crossing an even one.
    pub mislabelled: usize,
    /// Roots held by a window they are not inside.
    pub outside_window: usize,
    /// Roots reported on their own but off by more than the stated
    /// accuracy.
    pub inaccurate: usize,
    /// Roots merged into one tangential hit although they are farther
    /// apart than `1e-12` torus sizes.
    pub merged: usize,
    /// Hits with an infinite spread: nothing certified.
    pub unbounded: usize,
}

impl Tally {
    pub fn failures(&self) -> usize {
        self.lost
            + self.phantom
            + self.mislabelled
            + self.outside_window
            + self.inaccurate
            + self.merged
            + self.unbounded
    }
}

/// Compare one line's hits with the reference. Returns a description of
/// the first disagreement, if any.
pub fn judge(hits: &[RayHit], truth: &Truth, tally: &mut Tally) -> Option<String> {
    tally.lines += 1;
    tally.roots += truth.roots.len();
    let mut complaint: Option<String> = None;
    let mut complain = |what: String| {
        if complaint.is_none() {
            complaint = Some(what);
        }
    };
    let size = truth.size_t;
    // How far a hit may sit from the root it is matched with at all.
    let reach = 1e-7 * size;
    // The accuracy a root reported on its own must have: 128 units in the
    // last place of the larger of the root and the torus's size, the widest
    // enclosure the solver accepts without polishing (plus rounding).
    let accuracy = |root: f64| 160.0 * f64::EPSILON * root.abs().max(size);
    // Hits next to an ambiguous contact are judged only by their parity.
    let near_ambiguous = |t: f64, spread: f64| {
        truth
            .ambiguous
            .iter()
            .position(|&(at, _)| (t - at).abs() <= spread + 1e-5 * size)
    };
    let mut ambiguous_parity = vec![0_usize; truth.ambiguous.len()];
    let mut holds: Vec<Vec<Dd>> = vec![Vec::new(); hits.len()];
    let mut settled = vec![false; hits.len()];
    for (i, hit) in hits.iter().enumerate() {
        if !hit.spread.is_finite() {
            tally.unbounded += 1;
            complain(format!("unbounded hit {hit:?}"));
            settled[i] = true;
            continue;
        }
        if let Some(k) = near_ambiguous(hit.t, hit.spread) {
            ambiguous_parity[k] += usize::from(hit.kind != ContactKind::Tangential);
            settled[i] = true;
        }
    }
    for (k, &(at, odd)) in truth.ambiguous.iter().enumerate() {
        if (ambiguous_parity[k] % 2 == 1) != odd {
            tally.mislabelled += 1;
            complain(format!("parity at the ambiguous contact {at}"));
        }
    }
    for &root in &truth.roots {
        let r = root.value();
        let owner = hits
            .iter()
            .enumerate()
            .filter(|(i, hit)| !settled[*i] && (hit.t - r).abs() <= hit.spread + reach)
            .min_by(|a, b| (a.1.t - r).abs().total_cmp(&(b.1.t - r).abs()))
            .map(|(i, _)| i);
        if let Some(i) = owner {
            holds[i].push(root);
        } else {
            tally.lost += 1;
            complain(format!("lost root {r}"));
        }
    }
    for (i, hit) in hits.iter().enumerate() {
        if settled[i] {
            continue;
        }
        let held = &holds[i];
        let odd = held.len() % 2 == 1;
        for root in held {
            let off = (Dd::of(hit.t) - *root).abs().value();
            if hit.spread > 0.0 && off > hit.spread * (1.0 + 1e-12) + f64::MIN_POSITIVE {
                tally.outside_window += 1;
                complain(format!("root {} outside {hit:?}", root.value()));
            }
        }
        match hit.kind {
            ContactKind::Tangential => {
                if odd {
                    tally.mislabelled += 1;
                    complain(format!("a crossing labelled a touch: {hit:?}"));
                } else if held.is_empty() {
                    tally.phantom += 1;
                    complain(format!("a touch where there is none: {hit:?}"));
                } else {
                    let apart = (held[held.len() - 1] - held[0]).value();
                    if apart > 1e-12 * size {
                        tally.merged += held.len();
                        complain(format!("roots {apart:e} apart merged into {hit:?}"));
                    }
                }
            }
            ContactKind::Transversal | ContactKind::Unclassified | ContactKind::Coincident => {
                if held.is_empty() {
                    tally.phantom += 1;
                    complain(format!("phantom root {hit:?}"));
                } else if !odd {
                    tally.mislabelled += 1;
                    complain(format!("a touch labelled a crossing: {hit:?}"));
                } else if held.len() > 1 {
                    tally.merged += held.len();
                    complain(format!("{} roots merged into {hit:?}", held.len()));
                } else {
                    tally.separate += 1;
                    let off = (Dd::of(hit.t) - held[0]).abs().value();
                    let allowed = accuracy(held[0].value()).max(hit.spread);
                    if off > allowed {
                        tally.inaccurate += 1;
                        complain(format!(
                            "root {} reported as {} ({off:e} off, {allowed:e} allowed)",
                            held[0].value(),
                            hit.t
                        ));
                    }
                }
            }
        }
    }
    complaint
}

/// The spacing of `f64` values at `x`: the distance to the next one up.
pub fn spacing(x: f64) -> f64 {
    x.abs().next_up() - x.abs()
}

/// Direction lengths for the families of the caller's parameter: unit
/// three times in seven, else `1e-12`, `1e-6`, `1e6` or `1e12`.
fn length(rng: &mut Rng) -> f64 {
    [1.0, 1.0, 1.0, 1e-12, 1e-6, 1e6, 1e12][(rng.next() % 7) as usize]
}

/// The line families that put the caller's parameter at its limit.
pub fn caller_cases() -> Vec<Case> {
    caller_cases_seeded(0x1638_FDA8_17A0_0005)
}

/// The families of the caller's parameter drawn from `seed`: lines on
/// which neighbouring `f64` values of `t` are about as far apart as two
/// crossings, from well below to well above, so that some pairs resolve
/// and some cannot.
///
/// - `far-centre-ulps/*`: a torus `1e9` to `1e15` from the world origin,
///   its tube `0.3` to `30` spacings of `t` in radius, crossed from near
///   the world origin.
/// - `far-origin-graze/*`: axis-aligned lines `1e3` to `1e12` from a torus
///   sized so that a line four units in the last place of `R + r` inside
///   the outer equator (or of `r` under the tube's top) cuts a chord one
///   spacing of `t` long; the offsets run from outside the tube to a
///   thousand units inside.
/// - `far-origin-thin/*`: tilted tori whose tube is `0.05` to `50` spacings
///   of `t` in radius, chords from `1e3` to `1e12` away.
/// - `far-centre/1e15`, `non-sterbenz/*`: chords and tangents of a torus
///   `1e15` out, and lines whose direction spans the whole way from an
///   origin on the far side of the world origin (so `t` is near one and
///   `origin − centre` is no exact `f64`).
/// - `direction/1e±12`, `direction/1e±6`: chords and tangents with
///   directions `1e-12` to `1e12` long.
///
/// Directions in the first three are unit, or `1e-12` to `1e12` long.
pub fn caller_cases_seeded(seed: u64) -> Vec<Case> {
    let mut rng = Rng(seed);
    let mut out = Vec::new();
    let mut push = |family: &'static str, torus: &ToroidalSurface, origin: Point3, dir: Vec3| {
        out.push(Case {
            family,
            torus: torus.clone(),
            origin,
            dir,
        });
    };

    for (centre, family) in [
        (1e9, "far-centre-ulps/1e9"),
        (1e12, "far-centre-ulps/1e12"),
        (1e15, "far-centre-ulps/1e15"),
    ] {
        for &tube in &[0.3, 1.0, 3.0, 10.0, 30.0] {
            for &aspect in &[3.0, 30.0] {
                for tilted in [false, true] {
                    let away = rng.direction();
                    let c = Point3::new(away.x() * centre, away.y() * centre, away.z() * centre);
                    let small = tube * spacing(centre);
                    let big = small * aspect;
                    let t = if tilted {
                        ToroidalSurface::with_axis(c, big, small, rng.direction()).unwrap()
                    } else {
                        ToroidalSurface::new(c, big, small).unwrap()
                    };
                    for _ in 0..16 {
                        let (u, v) = (rng.range(0.0, TAU), rng.range(0.0, TAU));
                        let (_, _, _, n) = frame(&t, u, v);
                        let radial = t.x_axis() * u.cos() + t.y_axis() * u.sin();
                        let target = c + radial * big + n * (small * rng.range(0.0, 1.2));
                        let origin = Point3::new(
                            rng.range(-1.0, 1.0),
                            rng.range(-1.0, 1.0),
                            rng.range(-1.0, 1.0),
                        );
                        let unit = (target - origin).normalize().unwrap();
                        push(family, &t, origin, unit * length(&mut rng));
                    }
                }
            }
        }
    }

    for (dist, family) in [
        (1e3, "far-origin-graze/1e3"),
        (1e6, "far-origin-graze/1e6"),
        (1e9, "far-origin-graze/1e9"),
        (1e12, "far-origin-graze/1e12"),
    ] {
        for &aspect in &[10.0, 1e4] {
            // A line `d` inside the outer equator cuts a chord
            // `2 √(2 r d)`. With `r (R + r) = dist² 2^-52 / 32` and `d`
            // four units in the last place of `R + r`, that is one spacing
            // of `t` at `dist` (to a factor of two).
            let product = dist * dist * f64::EPSILON / 32.0;
            let small = (product / (1.0 + aspect)).sqrt();
            let big = small * aspect;
            let t = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), big, small).unwrap();
            let total = big + small;
            for steps in [
                -4.0_f64, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 24.0,
                32.0, 48.0, 64.0, 128.0, 256.0, 1024.0,
            ] {
                let lines = [
                    // Along the axis, inside the outer equator.
                    (
                        Point3::new(total - steps * spacing(total), 0.0, -dist),
                        Vec3::new(0.0, 0.0, 1.0),
                    ),
                    // The same from the other end.
                    (
                        Point3::new(0.0, -(total - steps * spacing(total)), dist),
                        Vec3::new(0.0, 0.0, -1.0),
                    ),
                    // Under the tube's top, along its circle.
                    (
                        Point3::new(big, -dist, small - steps * spacing(small)),
                        Vec3::new(0.0, 1.0, 0.0),
                    ),
                    // Under the tube's top, across both tubes.
                    (
                        Point3::new(-dist, 0.0, small - steps * spacing(small)),
                        Vec3::new(1.0, 0.0, 0.0),
                    ),
                ];
                for (origin, unit) in lines {
                    push(family, &t, origin, unit * length(&mut rng));
                }
            }
        }
    }

    for (dist, family) in [
        (1e3, "far-origin-thin/1e3"),
        (1e6, "far-origin-thin/1e6"),
        (1e9, "far-origin-thin/1e9"),
        (1e12, "far-origin-thin/1e12"),
    ] {
        for &tube in &[0.05, 0.2, 1.0, 3.0, 10.0, 50.0] {
            for &aspect in &[3.0, 100.0] {
                let small = tube * spacing(dist);
                let big = small * aspect;
                let c = Point3::new(
                    rng.range(-1.0, 1.0) * big,
                    rng.range(-1.0, 1.0) * big,
                    rng.range(-1.0, 1.0) * big,
                );
                let t = ToroidalSurface::with_axis(c, big, small, rng.direction()).unwrap();
                for _ in 0..16 {
                    let (origin, unit) = chord(&mut rng, &t, dist / (big + small));
                    push(family, &t, origin, unit * length(&mut rng));
                }
            }
        }
    }

    for &aspect in &[3.0, 100.0] {
        for tilted in [false, true] {
            let t = torus(&mut rng, 2.0, aspect, tilted, 1e15);
            let total = t.major_radius() + t.minor_radius();
            for _ in 0..20 {
                let back = rng.range(0.0, 2.0);
                let (origin, dir) = chord(&mut rng, &t, back);
                push("far-centre/1e15", &t, origin, dir);
            }
            for kind in TANGENTS {
                for &offset in &[0.0, 1e-6, 1e-4, 1e-2] {
                    for side in [-1.0, 1.0] {
                        let (p, unit, n) = tangent(&mut rng, &t, kind);
                        let back = rng.range(0.2, 2.0) * total;
                        let origin = p + n * (side * offset * total) - unit * back;
                        push("far-centre/1e15", &t, origin, unit);
                    }
                }
            }
        }
    }
    for (centre, family) in [
        (1e6, "non-sterbenz/1e6"),
        (1e9, "non-sterbenz/1e9"),
        (1e12, "non-sterbenz/1e12"),
        (1e15, "non-sterbenz/1e15"),
    ] {
        let c = Point3::new(centre, centre / 3.0, centre * 0.127);
        let t = ToroidalSurface::new(c, 1.5, 0.5).unwrap();
        for _ in 0..64 {
            let origin = Point3::new(
                -centre * 0.271_828,
                -centre * 0.314_159,
                -centre * 0.123_456,
            );
            let dir = Vec3::new(
                c.x() - origin.x() + rng.range(-1.0, 1.0),
                c.y() - origin.y() + rng.range(-1.0, 1.0),
                c.z() - origin.z() + rng.range(-1.0, 1.0),
            );
            push(family, &t, origin, dir);
        }
    }

    for (scale, family) in [
        (1e-12, "direction/1e-12"),
        (1e-6, "direction/1e-6"),
        (1e6, "direction/1e6"),
        (1e12, "direction/1e12"),
    ] {
        for &size in &[1e-3, 1.0, 1e6] {
            let t = torus(&mut rng, size, 3.0, true, 0.0);
            let total = t.major_radius() + t.minor_radius();
            for _ in 0..16 {
                let back = rng.range(0.0, 3.0);
                let (origin, dir) = chord(&mut rng, &t, back);
                push(family, &t, origin, dir * scale);
            }
            for kind in TANGENTS {
                for &offset in &[0.0, 1e-9, 1e-6, 1e-3] {
                    for side in [-1.0, 1.0] {
                        let (p, unit, n) = tangent(&mut rng, &t, kind);
                        let back = rng.range(0.2, 3.0) * total;
                        let origin = p + n * (side * offset * total) - unit * back;
                        push(family, &t, origin, unit * scale);
                    }
                }
            }
        }
    }
    out
}

/// How one family's windows stand against the contract of the caller's
/// parameter.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct WindowTally {
    pub lines: usize,
    /// Reference roots.
    pub roots: usize,
    /// Windows, and how many are resolved.
    pub contacts: usize,
    pub resolved: usize,
    /// Unresolved windows holding two or more reference roots (roots the
    /// parameter, or the arithmetic, could not tell apart), and those
    /// holding fewer (stretches about a tangency).
    pub fused: usize,
    pub stretches: usize,
    /// Resolved windows next to a contact the reference cannot resolve:
    /// judged by their exact end signs only.
    pub unjudged: usize,
    // The violations.
    /// A window that is empty, misses its representative, or does not end
    /// strictly below the next one's start.
    pub disorder: usize,
    /// A window with an infinite end: nothing certified.
    pub unbounded: usize,
    /// A window end the quartic vanishes at exactly.
    pub root_at_end: usize,
    /// A resolved window with equal exact signs at its ends (or labelled a
    /// touch).
    pub resolved_parity: usize,
    /// A resolved window that does not hold exactly one reference root.
    pub resolved_count: usize,
    /// An unresolved window whose label disagrees with the exact parity of
    /// its end signs, or with the number of reference roots inside.
    pub unresolved_parity: usize,
    /// Reference roots strictly inside no window.
    pub lost: usize,
    /// Pairs of neighbouring roots left in one window although separable.
    pub separable: usize,
}

impl WindowTally {
    pub fn violations(&self) -> usize {
        self.disorder
            + self.unbounded
            + self.root_at_end
            + self.resolved_parity
            + self.resolved_count
            + self.unresolved_parity
            + self.lost
            + self.separable
    }

    pub fn add(&mut self, o: &Self) {
        self.lines += o.lines;
        self.roots += o.roots;
        self.contacts += o.contacts;
        self.resolved += o.resolved;
        self.fused += o.fused;
        self.stretches += o.stretches;
        self.unjudged += o.unjudged;
        self.disorder += o.disorder;
        self.unbounded += o.unbounded;
        self.root_at_end += o.root_at_end;
        self.resolved_parity += o.resolved_parity;
        self.resolved_count += o.resolved_count;
        self.unresolved_parity += o.unresolved_parity;
        self.lost += o.lost;
        self.separable += o.separable;
    }
}

/// The `f64` values of the parameter strictly between two neighbouring
/// simple roots, counted up to `cap`.
///
/// An `f64` clear of both roots by more than the reference's own error is
/// placed by comparison. One within that error of a root is placed by the
/// quartic's exact sign there: between the roots it is the exact sign at
/// their midpoint, just outside either it is the other, and on a root it
/// is zero.
fn doubles_between(case: &Case, lo: Dd, hi: Dd, size_t: f64, cap: usize) -> usize {
    let sign = |terms: &[f64]| exact::quartic_sign(&case.torus, case.origin, case.dir, terms);
    let middle = (lo + hi).half();
    let inside = sign(&[middle.0, middle.1]);
    if inside == 0 {
        return 0;
    }
    let mut t = lo.value().next_down();
    let end = hi.value().next_up();
    let mut count = 0;
    while t <= end && count < cap {
        let noise = 1e-20 * t.abs().max(size_t);
        let (above, below) = ((Dd::of(t) - lo).value(), (hi - Dd::of(t)).value());
        let between = if above > noise && below > noise {
            true
        } else if above < -noise || below < -noise {
            false
        } else {
            sign(&[t]) == inside
        };
        count += usize::from(between);
        t = t.next_up();
    }
    count
}

/// Hold one line's windows to the contract of the caller's parameter.
/// Returns a description of the first violation, if any.
///
/// With the windows' `f64` ends taken as exact and the quartic's sign at
/// them evaluated exactly ([`exact::quartic_sign`]):
///
/// - every window is non-empty, holds its representative, and ends at an
///   `f64` strictly below the one the next begins at;
/// - a resolved window has opposite exact signs at its ends and holds
///   exactly one reference root, strictly inside;
/// - an unresolved window is labelled a touch exactly when its end signs
///   are equal, and the number of reference roots inside has that parity;
/// - every reference root is strictly inside a window;
/// - no unresolved window holds two neighbouring roots that are
///   *separable*: more than `1e-12` torus sizes apart (the reach of the
///   solver's own arithmetic) with at least two `f64` values of the
///   parameter strictly between them (what two disjoint windows need).
pub fn judge_windows(
    case: &Case,
    contacts: &[LineTorusContact],
    truth: &Truth,
    tally: &mut WindowTally,
) -> Option<String> {
    tally.lines += 1;
    tally.roots += truth.roots.len();
    tally.contacts += contacts.len();
    let mut complaint: Option<String> = None;
    let mut complain = |what: String| {
        if complaint.is_none() {
            complaint = Some(what);
        }
    };
    let sign = |t: f64| exact::quartic_sign(&case.torus, case.origin, case.dir, &[t]);
    // Strictly inside the window; an infinite end holds everything.
    let inside = |root: Dd, c: &LineTorusContact| {
        (c.t_lo == f64::NEG_INFINITY || (root - Dd::of(c.t_lo)).sign() > 0)
            && (c.t_hi == f64::INFINITY || (Dd::of(c.t_hi) - root).sign() > 0)
    };
    let slack = 1e-5 * truth.size_t;
    for pair in contacts.windows(2) {
        if pair[0].t_hi.partial_cmp(&pair[1].t_lo) != Some(std::cmp::Ordering::Less) {
            tally.disorder += 1;
            complain(format!("windows meet: {pair:?}"));
        }
    }
    for c in contacts {
        if !(c.t_lo < c.t_hi && c.t_lo <= c.t && c.t <= c.t_hi) {
            tally.disorder += 1;
            complain(format!("malformed window {c:?}"));
        }
        if !(c.t_lo.is_finite() && c.t_hi.is_finite()) {
            tally.unbounded += 1;
            complain(format!("unbounded window {c:?}"));
            continue;
        }
        let (s_lo, s_hi) = (sign(c.t_lo), sign(c.t_hi));
        if s_lo == 0 || s_hi == 0 {
            tally.root_at_end += 1;
            complain(format!("a root at an end of {c:?}"));
            continue;
        }
        let odd = s_lo != s_hi;
        let held: Vec<Dd> = truth
            .roots
            .iter()
            .copied()
            .filter(|&r| inside(r, c))
            .collect();
        let ambiguous = truth
            .ambiguous
            .iter()
            .any(|&(at, _)| at >= c.t_lo - slack && at <= c.t_hi + slack);
        if c.resolved {
            tally.resolved += 1;
            if !odd || c.kind == ContactKind::Tangential {
                tally.resolved_parity += 1;
                complain(format!("end signs {s_lo} {s_hi} of resolved {c:?}"));
            }
            if held.len() != 1 {
                if ambiguous {
                    tally.unjudged += 1;
                } else {
                    tally.resolved_count += 1;
                    complain(format!("{} roots in resolved {c:?}", held.len()));
                }
            }
            continue;
        }
        if held.len() >= 2 {
            tally.fused += 1;
        } else {
            tally.stretches += 1;
        }
        let labelled_odd = c.kind != ContactKind::Tangential;
        if labelled_odd != odd || (!ambiguous && (held.len() % 2 == 1) != odd) {
            tally.unresolved_parity += 1;
            complain(format!(
                "end signs {s_lo} {s_hi}, {} roots in unresolved {c:?}",
                held.len()
            ));
        }
        for pair in held.windows(2) {
            let apart = (pair[1] - pair[0]).value();
            if apart > 1e-12 * truth.size_t
                && doubles_between(case, pair[0], pair[1], truth.size_t, 2) >= 2
            {
                tally.separable += 1;
                complain(format!(
                    "separable roots {} and {} in {c:?}",
                    pair[0].value(),
                    pair[1].value()
                ));
            }
        }
    }
    for &root in &truth.roots {
        if !contacts.iter().any(|c| inside(root, c)) {
            tally.lost += 1;
            complain(format!("root {} in no window", root.value()));
        }
    }
    complaint
}
