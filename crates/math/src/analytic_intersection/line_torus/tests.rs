#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::{
    Ball, Chart, Plain, Poly, Wide, contacts_of_line, cubic_roots_in_unit, exponent, ldexp,
    max_variations, two_prod, two_sum,
};
use crate::intersect::ContactKind;
use crate::surfaces::ToroidalSurface;
use crate::vec::{Point3, Vec3};

/// `SplitMix64`: deterministic samples without a dependency.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A signed integer of at most `bits` bits.
    #[allow(clippy::cast_possible_wrap)]
    fn int(&mut self, bits: u32) -> i64 {
        (self.next() >> (64 - bits)) as i64 - (1_i64 << (bits - 1))
    }

    /// A double with a full random significand and an exponent in
    /// `-span..=span`.
    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    fn float(&mut self, span: i32) -> f64 {
        let significand = (self.next() >> 11) as f64 / (1_u64 << 53) as f64 + 0.5;
        let exp = (self.next() % (2 * span.unsigned_abs() as u64 + 1)) as i32 - span;
        let sign = if self.next() & 1 == 0 { 1.0 } else { -1.0 };
        sign * ldexp(significand, exp)
    }
}

/// `|n / 2^60 − (hi + lo)|` for an integer `|n| < 2^63` and a double-double
/// close to it: both differences are exact or rounded once.
#[allow(clippy::cast_precision_loss)]
fn distance(n: i128, hi: f64, lo: f64) -> f64 {
    let top = (n >> 32) as f64;
    let bottom = (n & 0xFFFF_FFFF) as f64;
    ((ldexp(top, -28) - hi) + (ldexp(bottom, -60) - lo)).abs()
}

/// A cancelling degree-three form on 20-bit dyadic inputs, against exact
/// integer arithmetic: the exact value lies inside both balls, and each
/// ball's certified sign is the exact sign.
#[test]
fn balls_hold_the_exact_value() {
    let mut rng = Rng(0x00BA_115E_ED00);
    let mut signed = 0;
    for _ in 0..20_000 {
        let ints: [i64; 6] = std::array::from_fn(|_| rng.int(20));
        let [a, b, c, d, e, f] = ints.map(i128::from);
        // (a·b − c·d)·(e + f) − a·c·e over 2^60.
        let exact = (a * b - c * d) * (e + f) - a * c * e;
        #[allow(clippy::cast_precision_loss)]
        let v = ints.map(|n| ldexp(n as f64, -20));
        let p = v.map(Plain::exact);
        let w = v.map(Wide::exact);
        let plain = (p[0] * p[1] - p[2] * p[3]) * (p[4] + p[5]) - p[0] * p[2] * p[4];
        let wide = (w[0] * w[1] - w[2] * w[3]) * (w[4] + w[5]) - w[0] * w[2] * w[4];
        assert!(distance(exact, plain.v, 0.0) <= plain.e, "{plain:?}");
        assert!(distance(exact, wide.hi, wide.lo) <= wide.e, "{wide:?}");
        // 62 bits fit a double-double: the wide value is exact.
        assert!(distance(exact, wide.hi, wide.lo) == 0.0, "{wide:?}");
        let sign = exact.signum();
        assert!(plain.sign() == 0 || i128::from(plain.sign()) == sign);
        assert_eq!(i128::from(wide.sign()), sign, "{wide:?}");
        signed += usize::from(plain.sign() != 0);
    }
    assert!(signed > 19_000, "{signed}");
}

/// Identities that vanish exactly, on full-significand inputs across
/// exponents, where every wide operation rounds: zero lies inside each ball
/// (so no sign is certified), and the wide radius is of order `2^-100` of
/// the magnitudes summed.
#[test]
fn balls_hold_zero_for_cancelling_identities() {
    fn square_difference<B: Ball>(x: f64, y: f64) -> B {
        let (x, y) = (B::exact(x), B::exact(y));
        (x + y) * (x - y) - x * x + y * y
    }
    fn quartic<B: Ball>(x: f64, y: f64, z: f64) -> B {
        // (x² + y² + z)² − 4x²y² − ((x² − y²)² + z² + 2z(x² + y²)).
        let (x, y, z) = (B::exact(x), B::exact(y), B::exact(z));
        let (xx, yy) = (x * x, y * y);
        let g = xx + yy + z;
        let d = xx - yy;
        g * g - (xx * yy).scale(4.0) - (d * d + z * z + (z * (xx + yy)).scale(2.0))
    }
    let mut rng = Rng(0x1D_E471_7135);
    let mut worst: f64 = 0.0;
    for _ in 0..20_000 {
        let (x, y, z) = (rng.float(8), rng.float(8), rng.float(8));
        let plain: [Plain; 2] = [square_difference(x, y), quartic(x, y, z)];
        let wide: [Wide; 2] = [square_difference(x, y), quartic(x, y, z)];
        let scale = [x * x + y * y, (x * x + y * y + z.abs()).powi(2)];
        for k in 0..2 {
            assert!(plain[k].v.abs() <= plain[k].e, "{:?} {x} {y} {z}", plain[k]);
            assert_eq!(plain[k].sign(), 0);
            let value = wide[k].hi + wide[k].lo;
            assert!(value.abs() <= wide[k].e, "{:?} {x} {y} {z}", wide[k]);
            assert_eq!(wide[k].sign(), 0);
            assert!(plain[k].e <= 1e-13 * scale[k], "{:?}", plain[k]);
            worst = worst.max(wide[k].e / scale[k]);
        }
    }
    assert!(worst < 1e-28, "{worst}");
}

/// A wide product and sum keep what a plain one rounds away.
#[test]
fn wide_arithmetic_is_error_free_where_it_can_be() {
    let (a, b) = (1.0 + 2.0_f64.powi(-30), 1.0 - 2.0_f64.powi(-31));
    let p = Wide::exact(a) * Wide::exact(b);
    let (hi, lo) = two_prod(a, b);
    assert_eq!((p.hi, p.lo), (hi, lo));
    let s = Wide::exact(1.0) + Wide::exact(2.0_f64.powi(-80));
    assert_eq!((s.hi, s.lo), two_sum(1.0, 2.0_f64.powi(-80)));
    // Cancellation leaves the low parts exact.
    let d = s - Wide::exact(1.0);
    assert_eq!((d.hi, d.lo), (2.0_f64.powi(-80), 0.0));
    assert_eq!(d.sign(), 1);
}

#[test]
#[allow(clippy::float_cmp)]
fn exact_scaling_helpers() {
    for (x, want) in [
        (1.0, 0),
        (1.999, 0),
        (2.0, 1),
        (0.5, -1),
        (f64::MIN_POSITIVE, -1022),
        (5e-324, -1074),
        (f64::MAX, 1023),
        (-3.0, 1),
    ] {
        assert_eq!(exponent(x), want, "{x}");
    }
    assert_eq!(ldexp(3.0, 2000).to_bits(), f64::INFINITY.to_bits());
    assert_eq!(ldexp(ldexp(3.0, -1060), 1060), 3.0);
    assert_eq!(ldexp(ldexp(1.5e300, -1500), 1500), 1.5e300);
    assert_eq!(ldexp(0.75, 10), 768.0);
}

#[test]
fn variations_take_uncertified_coefficients_adversely() {
    assert_eq!(max_variations([1, 1, 1, 1, 1]), 0);
    assert_eq!(max_variations([1, 1, -1, -1, -1]), 1);
    assert_eq!(max_variations([1, 0, -1, -1, -1]), 1);
    assert_eq!(max_variations([1, 0, 1, 1, 1]), 2);
    assert_eq!(max_variations([1, 0, 0, 1, 1]), 2);
    assert_eq!(max_variations([1, 0, 0, 0, 1]), 4);
    assert_eq!(max_variations([1, 0, 0, 0, -1]), 3);
    assert_eq!(max_variations([1, -1, 1, -1, 1]), 4);
    assert_eq!(max_variations([-1, 0, 0, 1, 1]), 3);
}

#[test]
fn cubic_split_points() {
    // (s − 0.25)(s − 0.5)(s − 0.75) in Bernstein form on [0, 1].
    let power = [-0.09375, 0.6875, -1.5, 1.0];
    let bernstein = [
        power[0],
        power[0] + power[1] / 3.0,
        power[0] + 2.0 * power[1] / 3.0 + power[2] / 3.0,
        power[0] + power[1] + power[2] + power[3],
    ];
    let (roots, count) = cubic_roots_in_unit(bernstein);
    assert_eq!(count, 3);
    for (got, want) in roots.iter().zip([0.25, 0.5, 0.75]) {
        assert!((got - want).abs() < 1e-14, "{roots:?}");
    }
    assert_eq!(cubic_roots_in_unit([1.0, 2.0, 3.0, 4.0]).1, 0);
    assert_eq!(cubic_roots_in_unit([f64::NAN, 2.0, 3.0, 4.0]).1, 0);
}

/// The Bernstein coefficients of the quartic on an interval reproduce it:
/// at the ends, and (by de Casteljau) at the midpoint.
#[test]
fn bernstein_coefficients_reproduce_the_quartic() {
    let torus = ToroidalSurface::with_axis(
        Point3::new(0.3, -0.2, 0.1),
        1.5,
        0.5,
        Vec3::new(0.2, -0.3, 1.0),
    )
    .unwrap();
    let chart = Chart::new(
        &torus,
        Point3::new(-4.0, 0.7, 0.3),
        Vec3::new(1.0, -0.1, 0.05),
    )
    .unwrap();
    let wide = Poly::<Wide>::new(&chart);
    for (u, v) in [(-1.0, 0.5), (0.25, 0.26), (-3.0, 3.0)] {
        let (l, r) = (wide.point(u), wide.point(v));
        let b = wide.bernstein(&l, &r);
        let mut level: Vec<f64> = vec![b[0].hi, b[1].hi, b[2].hi / 6.0, b[3].hi, b[4].hi];
        while level.len() > 1 {
            level = level.windows(2).map(|p| 0.5 * (p[0] + p[1])).collect();
        }
        let middle = wide.point(f64::midpoint(u, v)).q.hi;
        let scale = b.iter().map(|c| c.hi.abs()).fold(0.0, f64::max);
        assert!((level[0] - middle).abs() <= 1e-13 * scale, "{u} {v}");
    }
}

fn contacts(torus: &ToroidalSurface, origin: Point3, dir: Vec3) -> Vec<super::LineTorusContact> {
    contacts_of_line(torus, origin, dir).unwrap()
}

/// Review round 3, finding 1: a large thin torus, the origin `1e-5` inside
/// the tube at the outer equator. Along `+Z` and `+Y` the line leaves the
/// tube on both sides of the origin; the two roots were merged into one
/// tangential contact at `t = 0`.
#[test]
fn close_roots_about_the_origin_are_two_roots() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1_500_000.0, 500.0).unwrap();
    let origin = Point3::new(1_500_499.999_99, 0.0, 0.0);
    for (dir, want) in [
        (Vec3::new(0.0, 0.0, 1.0), 0.100_000_380_223_938_04),
        (Vec3::new(0.0, 1.0, 0.0), 5.478_159_226_496_254),
    ] {
        let got = contacts(&torus, origin, dir);
        assert_eq!(got.len(), 2, "{got:?}");
        for (contact, want) in got.iter().zip([-want, want]) {
            assert!(contact.resolved, "{contact:?}");
            assert_eq!(contact.kind, ContactKind::Transversal, "{contact:?}");
            assert!(
                contact.t_lo <= want && want <= contact.t_hi,
                "{contact:?} vs {want}"
            );
            assert!(contact.t_hi - contact.t_lo <= 1e-9, "{contact:?}");
        }
    }
    let ahead = contacts(&torus, origin, Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(ahead.len(), 4, "{ahead:?}");
    assert!(
        (ahead[3].t - 1.000_007_614_493_37e-5).abs() <= 1e-9,
        "{ahead:?}"
    );
}

/// Review round 3, finding 2: a torus `1e12` from the world origin. Forming
/// the line's closest point in absolute coordinates moved it by `3.5e-5`,
/// and both roots were lost.
#[test]
fn a_far_centre_costs_no_roots() {
    let c = 1e12;
    let torus = ToroidalSurface::new(Point3::new(c, c, c), 1.5, 0.5).unwrap();
    let origin = Point3::new(
        1_000_000_000_001.078_1,
        1_000_000_000_001.284_8,
        999_999_999_999.447_9,
    );
    let dir = Vec3::new(
        0.228_554_044_149_031_2,
        0.648_657_396_426_808_1,
        0.725_952_223_609_734_2,
    );
    let got = contacts(&torus, origin, dir);
    assert_eq!(got.len(), 2, "{got:?}");
    for (contact, want) in got
        .iter()
        .zip([0.296_538_242_679_434_4, 0.303_598_593_044_409_3])
    {
        assert!(contact.resolved, "{contact:?}");
        assert!(
            contact.t_lo <= want && want <= contact.t_hi,
            "{contact:?} vs {want}"
        );
        assert!(contact.t_hi - contact.t_lo <= 1e-13, "{contact:?}");
    }
}

/// Review round 3, finding 3: a line `1e-13` inside the tube's top, along
/// its circle. The two crossings, `2e-3` apart, came back as one tangential
/// representative between them.
#[test]
fn a_shallow_chord_keeps_both_ends() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap();
    let got = contacts(
        &torus,
        Point3::new(1.5, -2.0, 0.499_999_999_999_9),
        Vec3::new(0.0, 1.0, 0.0),
    );
    assert_eq!(got.len(), 2, "{got:?}");
    for (contact, want) in got
        .iter()
        .zip([1.999_026_055_662_423, 2.000_973_944_337_577])
    {
        assert!(contact.resolved, "{contact:?}");
        assert!(
            contact.t_lo <= want && want <= contact.t_hi,
            "{contact:?} vs {want}"
        );
        assert!(contact.t_hi - contact.t_lo <= 1e-14, "{contact:?}");
    }
    // The larger example: the representative lay 0.05 inside the torus.
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1_500_000.0, 500.0).unwrap();
    let got = contacts(
        &torus,
        Point3::new(1_500_499.95, 0.0, -15_000_000_000.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    assert_eq!(got.len(), 2, "{got:?}");
    for (contact, want) in got
        .iter()
        .zip([14_999_999_992.929_11, 15_000_000_007.070_89])
    {
        assert!(contact.resolved, "{contact:?}");
        assert!(
            contact.t_lo <= want && want <= contact.t_hi,
            "{contact:?} vs {want}"
        );
        assert!(contact.t_hi - contact.t_lo <= 1e-4, "{contact:?}");
    }
}

/// Exact tangencies are the unresolved contacts: one window each, of the
/// width the multiplicity allows, holding the contact.
#[test]
fn exact_tangencies_are_unresolved_windows() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap();
    // Outer equator, along the circle: a double root at t = 3.
    let got = contacts(
        &torus,
        Point3::new(2.0, -3.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    assert_eq!(got.len(), 1, "{got:?}");
    assert!(!got[0].resolved && got[0].kind == ContactKind::Tangential);
    assert!(got[0].t_lo <= 3.0 && 3.0 <= got[0].t_hi, "{got:?}");
    assert!(got[0].t_hi - got[0].t_lo <= 1e-12, "{got:?}");
    // Tube top, across the circle: two double roots, at x = ±1.5.
    let got = contacts(
        &torus,
        Point3::new(-4.0, 0.0, 0.5),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert_eq!(got.len(), 2, "{got:?}");
    for (contact, want) in got.iter().zip([2.5, 5.5]) {
        assert!(!contact.resolved && contact.kind == ContactKind::Tangential);
        assert!(contact.t_lo <= want && want <= contact.t_hi, "{got:?}");
        assert!(contact.t_hi - contact.t_lo <= 1e-12, "{got:?}");
    }
    // Tube top, along the circle: a quadruple root at t = 2.
    let got = contacts(
        &torus,
        Point3::new(1.5, -2.0, 0.5),
        Vec3::new(0.0, 1.0, 0.0),
    );
    assert_eq!(got.len(), 1, "{got:?}");
    assert!(!got[0].resolved && got[0].kind == ContactKind::Tangential);
    assert!(got[0].t_lo <= 2.0 && 2.0 <= got[0].t_hi, "{got:?}");
    assert!(got[0].t_hi - got[0].t_lo <= 1e-6, "{got:?}");
    // A clean miss and a clean chord.
    assert!(
        contacts(
            &torus,
            Point3::new(-4.0, 0.0, 0.6),
            Vec3::new(1.0, 0.0, 0.0)
        )
        .is_empty()
    );
    let got = contacts(
        &torus,
        Point3::new(-4.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    let ts: Vec<f64> = got.iter().map(|c| c.t).collect();
    assert_eq!(ts, vec![2.0, 3.0, 5.0, 6.0]);
    assert!(
        got.iter()
            .all(|c| c.resolved && c.kind == ContactKind::Transversal)
    );
}

/// A spindle torus (`R < r`) meets a line on its outer apple and on its
/// inner lemon; the quartic holds both, and so do the contacts. Through the
/// centre in the equatorial plane the roots are `±(r − R)` and `±(r + R)`;
/// along the axis the two apex points, where the surface crosses itself,
/// are exact double roots.
#[test]
fn spindle_tori_keep_their_inner_roots() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 0.5, 1.0).unwrap();
    let got = contacts(
        &torus,
        Point3::new(-4.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    let ts: Vec<f64> = got.iter().map(|c| c.t).collect();
    assert_eq!(ts.len(), 4, "{got:?}");
    for (t, want) in ts.iter().zip([2.5, 3.5, 4.5, 5.5]) {
        assert!((t - want).abs() <= 1e-14, "{got:?}");
    }
    assert!(got.iter().all(|c| c.resolved));
    let got = contacts(
        &torus,
        Point3::new(0.0, 0.0, -2.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    assert_eq!(got.len(), 2, "{got:?}");
    let apex = 0.75_f64.sqrt();
    for (contact, want) in got.iter().zip([2.0 - apex, 2.0 + apex]) {
        assert!(!contact.resolved && contact.kind == ContactKind::Tangential);
        assert!(contact.t_lo <= want && want <= contact.t_hi, "{got:?}");
        assert!(contact.t_hi - contact.t_lo <= 1e-7, "{got:?}");
    }
    // Off the axis by a hair the apex contacts open into pairs or vanish;
    // parity stays even and nothing is unresolved.
    let got = contacts(
        &torus,
        Point3::new(1e-9, 0.0, -2.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    assert!(
        got.len().is_multiple_of(2) && got.iter().all(|c| c.resolved),
        "{got:?}"
    );
}

/// The contacts do not depend on how the line is parameterised: sliding the
/// origin along the line and rescaling the direction moves every parameter
/// by the same affine map, to rounding.
#[test]
fn contacts_follow_the_parameterisation() {
    let torus = ToroidalSurface::with_axis(
        Point3::new(0.4, -1.1, 2.0),
        3.0,
        0.7,
        Vec3::new(0.3, 0.5, 1.0),
    )
    .unwrap();
    let origin = Point3::new(-6.0, -0.7, 2.4);
    let dir = Vec3::new(0.93, 0.06, -0.05);
    let base = contacts(&torus, origin, dir);
    assert_eq!(base.len(), 2, "{base:?}");
    for (shift, scale) in [
        (3.5, 1.0),
        (-250.0, 1.0),
        (0.0, 1e-7),
        (1e4, 3e5),
        (-7.25, 1e-3),
    ] {
        let moved = contacts(&torus, origin + dir * shift, dir * scale);
        assert_eq!(moved.len(), base.len(), "{moved:?}");
        for (a, b) in base.iter().zip(&moved) {
            let want = (a.t - shift) / scale;
            // The moved origin is itself rounded: a few ulps of its
            // coordinates over the direction's length.
            let slack = 64.0 * f64::EPSILON * (want.abs() + (shift.abs() + 10.0) / scale);
            assert!((b.t - want).abs() <= slack, "{b:?} vs {want}");
            assert_eq!((a.kind, a.resolved), (b.kind, b.resolved));
        }
    }
}

/// A line within rounding of a quadruple tangency, found by soaking the
/// differential harness: the quartic sits at its error radius over a
/// stretch `1e-7` wide, where a bare sign is certified at one point and not
/// at the next. Fencing the contact at the first certified point left a
/// window followed by another a grain on, and so on until the work budget
/// ran out and the line went uncertified. The window is fenced where the
/// quartic clears its radius well, and the line is one contact.
#[test]
fn a_near_quadruple_contact_is_one_window() {
    let torus = ToroidalSurface::new(
        Point3::new(
            0.001_291_645_887_731_195,
            0.001_084_183_731_540_427,
            -0.000_626_656_906_824_692_8,
        ),
        0.000_471_194_291_225_880_46,
        0.000_466_529_001_213_743,
    )
    .unwrap();
    let origin = Point3::new(
        0.000_402_732_564_997_629,
        0.001_591_616_993_808_487_7,
        -0.000_160_127_905_610_949_76,
    );
    let dir = Vec3::new(0.542_740_331_278_747_9, -0.839_900_549_352_978_4, 0.0);
    let got = contacts(&torus, origin, dir);
    assert_eq!(got.len(), 1, "{got:?}");
    let contact = got[0];
    assert!(!contact.resolved && contact.kind == ContactKind::Tangential);
    // Where the double-double reference puts the contact.
    let at = 0.000_908_642_586_997_426_3;
    assert!(contact.t_lo <= at && at <= contact.t_hi, "{got:?}");
    let size = torus.major_radius() + torus.minor_radius();
    assert!(contact.t_hi - contact.t_lo <= 1e-6 * size, "{got:?}");
}
