//! Closed-form oracles for the SSI marcher's private branch geometry.
//!
//! Every expected value below is derived by hand from the fixture's
//! construction (an exact polynomial patch, a known affine chart, a known
//! quadratic form), never by re-running the code under test. The fixtures are
//! chosen so the answer changes under the mutations the weekly mutation run
//! reported as surviving: skewed charts (so every entry of the parameter maps
//! is live), both surfaces curved (so the difference and the normal-sign
//! convention are live), exact gate boundaries, and degenerate charts that
//! sit between the two refusal gates.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp // exact boundaries and bit-exact dyadic fixtures are the point
)]

use super::*;
use crate::context::OperationContext;
use crate::nurbs::intersection::surface_seeding::refine_ssi_point;

// ---------------------------------------------------------------- fixtures

fn bilinear(p: [[Point3; 2]; 2], ku: (f64, f64), kv: (f64, f64)) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![ku.0, ku.0, ku.1, ku.1],
        vec![kv.0, kv.0, kv.1, kv.1],
        vec![vec![p[0][0], p[0][1]], vec![p[1][0], p[1][1]]],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap()
}

/// Flat parallelogram `S(u, v) = su (u - ½) + sv (v - ½)`: its partials are
/// `su` and `sv` everywhere, bit for bit when they are dyadic.
fn affine_patch(su: Vec3, sv: Vec3) -> NurbsSurface {
    let at = |a: f64, b: f64| Point3::new(0.0, 0.0, 0.0) + su * (a - 0.5) + sv * (b - 0.5);
    bilinear(
        [[at(0.0, 0.0), at(0.0, 1.0)], [at(1.0, 0.0), at(1.0, 1.0)]],
        (0.0, 1.0),
        (0.0, 1.0),
    )
}

/// `z = c·x·y` over `[-1, 1]²` on the knot domain `[0, k]²` (bilinear, exact).
fn saddle_on(c: f64, k: f64) -> NurbsSurface {
    bilinear(
        [
            [Point3::new(-1.0, -1.0, c), Point3::new(-1.0, 1.0, -c)],
            [Point3::new(1.0, -1.0, -c), Point3::new(1.0, 1.0, c)],
        ],
        (0.0, k),
        (0.0, k),
    )
}

fn saddle(c: f64) -> NurbsSurface {
    saddle_on(c, 1.0)
}

/// `z = 0` over `[-1, 1]²` on the knot domain `[0, k]²`: the same chart as
/// [`saddle_on`], so the two share partials at every parameter.
fn plane_on(k: f64) -> NurbsSurface {
    saddle_on(0.0, k)
}

fn plane() -> NurbsSurface {
    plane_on(1.0)
}

/// Bivariate polynomial `sum c[i][j] u^i v^j`, `i, j <= 2`.
type Poly = [[f64; 3]; 3];

fn affine_poly(c0: f64, cu: f64, cv: f64) -> Poly {
    let mut p = [[0.0; 3]; 3];
    p[0][0] = c0;
    p[1][0] = cu;
    p[0][1] = cv;
    p
}

fn mul_affine(a: &Poly, b: &Poly) -> Poly {
    let mut out = [[0.0; 3]; 3];
    for i in 0..2 {
        for j in 0..2 - i {
            for k in 0..2 {
                for l in 0..2 - k {
                    out[i + k][j + l] += a[i][j] * b[k][l];
                }
            }
        }
    }
    out
}

fn add_scaled(acc: &mut Poly, s: f64, p: &Poly) {
    for i in 0..3 {
        for j in 0..3 {
            acc[i][j] += s * p[i][j];
        }
    }
}

/// Bézier control value of a degree-(2, 2) polynomial: its blossom at `k` ones
/// among the two `u` arguments and `l` among the two `v` arguments.
fn blossom(p: &Poly, k: usize, l: usize) -> f64 {
    let polar = |i: usize, m: usize| match i {
        0 => 1.0,
        1 => f64::from(u8::try_from(m).unwrap()) / 2.0,
        _ => f64::from(u8::from(m == 2)),
    };
    let mut s = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            s += p[i][j] * polar(i, k) * polar(j, l);
        }
    }
    s
}

/// Exact biquadratic patch of the graph `z = p x² + q x y + r y²` over the
/// affine chart `(x, y) = A (u - ½, v - ½)`, `A = [[xu, xv], [yu, yv]]`,
/// placed by `place`. The parameter centre `(½, ½)` is the origin, where the
/// tangent plane is `z = 0` and the Hessian of the height is
/// `[[2p, q], [q, 2r]]`.
fn quadric_patch(
    a: [[f64; 2]; 2],
    (p, q, r): (f64, f64, f64),
    place: fn(Point3) -> Point3,
) -> NurbsSurface {
    let x = affine_poly(-(a[0][0] + a[0][1]) / 2.0, a[0][0], a[0][1]);
    let y = affine_poly(-(a[1][0] + a[1][1]) / 2.0, a[1][0], a[1][1]);
    let mut z = [[0.0; 3]; 3];
    add_scaled(&mut z, p, &mul_affine(&x, &x));
    add_scaled(&mut z, q, &mul_affine(&x, &y));
    add_scaled(&mut z, r, &mul_affine(&y, &y));
    let control = (0..3)
        .map(|k| {
            (0..3)
                .map(|l| {
                    place(Point3::new(
                        blossom(&x, k, l),
                        blossom(&y, k, l),
                        blossom(&z, k, l),
                    ))
                })
                .collect()
        })
        .collect();
    let s = NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        control,
        vec![vec![1.0; 3]; 3],
    )
    .unwrap();
    // The fixture is the graph it claims to be.
    for &(u, v) in &[(0.1, 0.2), (0.5, 0.5), (0.9, 0.35), (0.3, 0.8)] {
        let xx = a[0][0] * (u - 0.5) + a[0][1] * (v - 0.5);
        let yy = a[1][0] * (u - 0.5) + a[1][1] * (v - 0.5);
        let want = place(Point3::new(xx, yy, p * xx * xx + q * xx * yy + r * yy * yy));
        assert!(
            (s.evaluate(u, v) - want).length() < 1e-14,
            "fixture off its graph"
        );
    }
    s
}

fn identity(p: Point3) -> Point3 {
    p
}

/// Exact quarter turn about `x`: `(x, y, z) -> (x, -z, y)`.
fn rot90x(p: Point3) -> Point3 {
    Point3::new(p.x(), -p.z(), p.y())
}

fn origin_point() -> IntersectionPoint {
    IntersectionPoint {
        point: Point3::new(0.0, 0.0, 0.0),
        param1: (0.5, 0.5),
        param2: (0.5, 0.5),
    }
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn close_vec(a: Vec3, b: Vec3, tol: f64) -> bool {
    (a - b).length() <= tol
}

fn diff_at_centres(s1: &NurbsSurface, s2: &NurbsSurface) -> Option<OrthoDiff> {
    orthonormal_difference(s1, s2, 0.5, 0.5, 0.5, 0.5, &mut SsiScratch::new())
}

/// `[[c00, c01], [c10, c11]]` inverse of a chart matrix, row-major as
/// [`OrthoDiff`] stores its parameter maps.
fn inverse(a: [[f64; 2]; 2]) -> [f64; 4] {
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    [a[1][1] / det, -a[0][1] / det, -a[1][0] / det, a[0][0] / det]
}

/// The generic pair: both surfaces curved, both on skewed charts with every
/// chart entry non-zero, both normals `+z` at the centre. The height
/// difference has Hessian `Q = [[2, 5/2], [5/2, -3]] = H1 - H2`, whose nulls
/// are the lines `(1, 2)` and `(3, -1)`: `(2x - y)(x + 3y) = 0`.
const A1: [[f64; 2]; 2] = [[0.75, 0.25], [-0.5, 1.25]];
const A2: [[f64; 2]; 2] = [[1.5, -0.5], [0.25, 0.5]];
const H1: (f64, f64, f64) = (0.75, 3.0, -1.125);
const H2: (f64, f64, f64) = (-0.25, 0.5, 0.375);

fn generic_pair(a1: [[f64; 2]; 2], a2: [[f64; 2]; 2]) -> (NurbsSurface, NurbsSurface) {
    (
        quadric_patch(a1, H1, identity),
        quadric_patch(a2, H2, identity),
    )
}

// ------------------------------------------------- orthonormal_difference

/// `z = c·x·y` against its tangent plane at the origin: the height
/// difference is `c·x·y`, so in the frame `(x, y)` it is
/// `Q = [[0, c], [c, 0]]`, `det = -c²`, `||Q|| = √2·c`, and both charts map
/// `(alpha, beta)` to `(alpha, beta) / 2`. Swapping the operands negates `Q`
/// (`b = -c`), and so does nothing else: the flipped saddle (normal `-z`)
/// in second place is measured along `n1` through `s = sign(n1·n2) = -1`.
#[test]
fn saddle_difference_is_the_closed_form_form() {
    let c = 0.5;
    let half = [0.5, 0.0, 0.0, 0.5];
    let d = diff_at_centres(&saddle(c), &plane()).unwrap();
    assert_eq!((d.a, d.b, d.d), (0.0, c, 0.0));
    assert_eq!(d.det, -c * c);
    assert!(close(d.norm, 2.0_f64.sqrt() * c, 1e-15));
    assert_eq!(d.c1, half);
    assert_eq!(d.c2, half);
    assert!(close_vec(d.e1, Vec3::new(1.0, 0.0, 0.0), 0.0));
    assert!(close_vec(d.e2, Vec3::new(0.0, 1.0, 0.0), 0.0));

    // Plane first: Q = 0 - Q_saddle.
    let d = diff_at_centres(&plane(), &saddle(c)).unwrap();
    assert_eq!((d.a, d.b, d.d), (0.0, -c, 0.0));

    // Plane first, saddle with reversed u second (normal -z, cos = -1): the
    // saddle's height along its own normal is -c·x·y, re-signed by s = -1.
    let flipped = bilinear(
        [
            [Point3::new(1.0, -1.0, -c), Point3::new(1.0, 1.0, c)],
            [Point3::new(-1.0, -1.0, c), Point3::new(-1.0, 1.0, -c)],
        ],
        (0.0, 1.0),
        (0.0, 1.0),
    );
    let d = diff_at_centres(&plane(), &flipped).unwrap();
    assert_eq!((d.a, d.b, d.d), (0.0, -c, 0.0));
    assert_eq!(d.c2, [-0.5, 0.0, 0.0, 0.5]);
}

/// Both surfaces curved, both charts skewed: `Q = H1 - H2` exactly, and each
/// parameter map is the inverse of its chart. Every entry of the metric, the
/// shape operator and both maps is non-zero here, so each term of
/// `C = I⁻¹B` and `Q = Cᵀ II C` carries weight.
#[test]
fn generic_difference_is_the_hessian_difference_on_skewed_charts() {
    let (s1, s2) = generic_pair(A1, A2);
    let d = diff_at_centres(&s1, &s2).unwrap();
    let (a, b, dd) = (2.0, 2.5, -3.0);
    assert!(close(d.a, a, 1e-13), "a = {}", d.a);
    assert!(close(d.b, b, 1e-13), "b = {}", d.b);
    assert!(close(d.d, dd, 1e-13), "d = {}", d.d);
    assert!(close(d.det, a * dd - b * b, 1e-12), "det = {}", d.det);
    assert!(
        close(d.norm, (a * a + 2.0 * b * b + dd * dd).sqrt(), 1e-12),
        "norm = {}",
        d.norm
    );
    for (got, want) in [(d.c1, inverse(A1)), (d.c2, inverse(A2))] {
        for i in 0..4 {
            assert!(close(got[i], want[i], 1e-14), "{got:?} vs {want:?}");
        }
    }
    // Reversed operands: the same form, negated.
    let d = diff_at_centres(&s2, &s1).unwrap();
    assert!(close(d.a, -a, 1e-13) && close(d.b, -b, 1e-13) && close(d.d, -dd, 1e-13));
}

/// The frame is built from the normal alone, projecting the axis of the
/// normal's smallest component: `n = z` gives `(x, y)`; `n = x` gives
/// `(y, z)`; `n ∝ (2, 3, 1)` (smallest `z`) gives
/// `e1 = (-2, -3, 13)/√182`, `e2 = n × e1 = (3, -2, 0)/√13`.
#[test]
fn frame_projects_the_smallest_normal_component() {
    let cases = [
        (
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ),
        (
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        (
            Vec3::new(1.0, 0.0, -2.0),
            Vec3::new(0.0, 1.0, -3.0),
            Vec3::new(-2.0, -3.0, 13.0) * (1.0 / 182.0_f64.sqrt()),
            Vec3::new(3.0, -2.0, 0.0) * (1.0 / 13.0_f64.sqrt()),
        ),
    ];
    for (su, sv, e1, e2) in cases {
        let p = affine_patch(su, sv);
        let d = diff_at_centres(&p, &p).expect("coincident planes share a plane");
        assert!(close_vec(d.e1, e1, 1e-15), "e1 {:?} vs {e1:?}", d.e1);
        assert!(close_vec(d.e2, e2, 1e-15), "e2 {:?} vs {e2:?}", d.e2);
        assert_eq!((d.a, d.b, d.d, d.norm), (0.0, 0.0, 0.0, 0.0));
    }
}

/// The shared-plane gate is `|n1·n2| < 0.99` (strict): perpendicular planes
/// are refused, and a tilt whose normal has `n·z == 0.99` exactly is kept.
#[test]
fn shared_plane_gate_is_strict_at_cos_0_99() {
    let vertical = affine_patch(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    assert!(diff_at_centres(&plane(), &vertical).is_none());

    // n2 = normalize(-t, 0, 1); t is the f64 whose normal has z == 0.99.
    let t = f64::from_bits(0x3fc2_3d2f_e6e6_1aaf);
    let tilted = affine_patch(Vec3::new(1.0, 0.0, t), Vec3::new(0.0, 1.0, 0.0));
    let n2 = tilted.normal(0.5, 0.5).unwrap();
    assert!(
        (n2.z() - 0.99).abs() == 0.0,
        "fixture must sit exactly on the gate"
    );
    assert!(diff_at_centres(&plane(), &tilted).is_some());
}

/// A chart with partial lengths `2^10` and `2^-10` whose sine is `2^-22`:
/// it clears the `sin² > 1e-24` gate (`sin² = 2^-44`) but its metric
/// determinant `2^-44` is below `1e-24·(E + G)² ≈ 1.1e-12`, so the
/// per-surface guard must refuse it, in either parameter order.
#[test]
fn anisotropic_chart_is_refused_by_the_metric_guard() {
    let long = Vec3::new(1024.0, 0.0, 0.0);
    let short = Vec3::new(1.0 / 1024.0, 2.0_f64.powi(-32), 0.0);
    for (su, sv) in [(long, short), (short, long)] {
        let sliver = affine_patch(su, sv);
        let ders = sliver.derivatives(0.5, 0.5, 1);
        let (pu, pv) = (ders[1][0], ders[0][1]);
        let (e, f, g) = (pu.dot(pu), pu.dot(pv), pv.dot(pv));
        let sin2 = pu.cross(pv).length_squared() / (e * g);
        let det_i = e * g - f * f;
        assert!(sin2 > 1e-24, "fixture must pass the sine gate: {sin2:e}");
        assert!(
            det_i > 0.0 && det_i <= 1e-24 * (e + g) * (e + g),
            "fixture must fail only the metric guard: {det_i:e}"
        );
        assert!(diff_at_centres(&sliver, &plane()).is_none());
        assert!(diff_at_centres(&plane(), &sliver).is_none());
    }
}

/// Partials parallel to within `sin² ≈ 1e-26`, whose metric determinant
/// nevertheless rounds to a positive value above the metric guard: only the
/// `sin² <= 1e-24` gate refuses this chart, on either side.
#[test]
fn near_collinear_chart_is_refused_by_the_sine_gate() {
    let k = 1.0137;
    let sliver = affine_patch(
        Vec3::new(0.3, 0.7, 0.0),
        Vec3::new(0.3 * k - 0.7e-13, 0.7 * k + 0.3e-13, 0.0),
    );
    let ders = sliver.derivatives(0.5, 0.5, 1);
    let (pu, pv) = (ders[1][0], ders[0][1]);
    let (e, f, g) = (pu.dot(pu), pu.dot(pv), pv.dot(pv));
    let sin2 = pu.cross(pv).length_squared() / (e * g);
    let det_i = e * g - f * f;
    assert!(sin2 <= 1e-24, "fixture must fail the sine gate: {sin2:e}");
    assert!(
        det_i > 1e-24 * (e + g) * (e + g),
        "fixture must pass the metric guard: {det_i:e}"
    );
    assert!(pu.cross(pv).normalize().is_ok());
    assert!(diff_at_centres(&sliver, &plane()).is_none());
    assert!(diff_at_centres(&plane(), &sliver).is_none());
}

/// A knot domain of `2^43` shrinks both partials to `2^-42` (`E = G = 2^-84`),
/// a pure reparameterization: `Q` is unchanged and the maps scale to
/// `2^42·I`. Any sine computed without the `|Su|²|Sv|²` normalisation falls
/// below `1e-24` here.
#[test]
fn tiny_partials_from_a_wide_knot_domain_keep_the_form() {
    let k = 2.0_f64.powi(43);
    let c = 0.5;
    let m = k / 2.0;
    let d = orthonormal_difference(
        &saddle_on(c, k),
        &plane_on(k),
        m,
        m,
        m,
        m,
        &mut SsiScratch::new(),
    )
    .unwrap();
    assert!(close(d.a, 0.0, 1e-12) && close(d.d, 0.0, 1e-12));
    assert!(close(d.b, c, 1e-12), "b = {}", d.b);
    assert!(close(d.c1[0], k / 2.0, 1e-3) && close(d.c2[3], k / 2.0, 1e-3));
}

// ---------------------------------------------------------- null_directions

fn nulls(a: f64, b: f64, d: f64, e1: Vec3, e2: Vec3) -> Vec<Vec3> {
    let norm = (a * a + 2.0 * b * b + d * d).sqrt();
    let det = a * d - b * b;
    null_directions(a, b, d, e1, e2, norm, det)
}

fn xy_frame() -> (Vec3, Vec3) {
    (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0))
}

/// A tilted orthonormal frame, so frame composition is exercised in 3D.
fn tilted_frame() -> (Vec3, Vec3) {
    let e1 = Vec3::new(0.6, 0.0, 0.8).normalize().unwrap();
    (e1, Vec3::new(0.0, 1.0, 0.0))
}

/// Every returned direction is a unit vector in the frame's plane and a null
/// of `Q`: `|Q(t, t)| <= 1e-12·||Q||`.
fn assert_all_null(a: f64, b: f64, d: f64, (e1, e2): (Vec3, Vec3), dirs: &[Vec3]) {
    let norm = (a * a + 2.0 * b * b + d * d).sqrt();
    let n = e1.cross(e2);
    for t in dirs {
        assert!(close(t.length(), 1.0, 1e-14), "not unit: {t:?}");
        assert!(t.dot(n).abs() < 1e-14, "off the frame plane: {t:?}");
        let (al, be) = (t.dot(e1), t.dot(e2));
        let q = a * al * al + 2.0 * b * al * be + d * be * be;
        assert!(
            q.abs() <= 1e-12 * norm,
            "Q=({a},{b},{d}): {t:?} is not a null, Q(t,t)={q:e}"
        );
    }
}

/// An indefinite 2×2 form has exactly two null lines. Cases cover both
/// diagonal orders, a zero diagonal on either side, the pure saddle, and a
/// generic form, each in a flat and a tilted frame.
#[test]
fn indefinite_forms_return_their_two_null_lines() {
    let forms = [
        (2.0, 0.0, -1.0),
        (1.0, 3.0, 0.0),
        (0.0, 3.0, 5.0),
        (0.0, 1.0, 0.0),
        (2.0, 2.5, -3.0),
        (-3.0, 1.0, 4.0),
        (1.0, -2.0, 1.0),
    ];
    for frame in [xy_frame(), tilted_frame()] {
        for &(a, b, d) in &forms {
            let dirs = nulls(a, b, d, frame.0, frame.1);
            assert_eq!(dirs.len(), 2, "Q=({a},{b},{d}): {dirs:?}");
            assert_all_null(a, b, d, frame, &dirs);
            assert!(
                dirs[0].cross(dirs[1]).length() > 1e-3,
                "Q=({a},{b},{d}): one line returned twice: {dirs:?}"
            );
        }
    }
}

/// Classification is scale invariant: `λ·Q` keeps both nulls for `λ` from
/// `1e-6` to `1e6`, though `|det|` then spans 24 decades.
#[test]
fn indefinite_classification_is_scale_invariant() {
    for lambda in [1e-6, 1.0, 1e6] {
        let (a, b, d) = (lambda, 0.0, -1e-6 * lambda);
        let dirs = nulls(a, b, d, xy_frame().0, xy_frame().1);
        assert_eq!(dirs.len(), 2, "λ={lambda}: {dirs:?}");
        assert_all_null(a, b, d, xy_frame(), &dirs);
    }
}

/// Definite forms and `Q = 0` have no null direction; neither does a definite
/// form whose Frobenius norm overflows (`a = 1e200`, `d = 1e-200`).
#[test]
fn definite_zero_and_non_finite_forms_have_no_null() {
    let (e1, e2) = xy_frame();
    assert!(nulls(1.0, 0.0, 2.0, e1, e2).is_empty());
    assert!(nulls(-1.0, 0.5, -2.0, e1, e2).is_empty());
    assert!(nulls(0.0, 0.0, 0.0, e1, e2).is_empty());
    assert!(nulls(1e200, 0.0, 1e-200, e1, e2).is_empty());
}

/// A parabolic form has its zero eigenvector as the single ruling. The cases
/// cover each arm of the eigenvector choice (dominant `a`, dominant `d`,
/// dominant `b` on either side, and a zero row).
#[test]
fn parabolic_forms_return_their_single_ruling() {
    let eps = 2.0_f64.powi(-45);
    let forms = [
        (4.0, 2.0, 1.0),
        (1.0, 2.0, 4.0),
        (1.0, 1.0, 1.0),
        (1.0, 1.0 + eps, 1.0 + eps),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
    ];
    for frame in [xy_frame(), tilted_frame()] {
        for &(a, b, d) in &forms {
            let dirs = nulls(a, b, d, frame.0, frame.1);
            assert_eq!(dirs.len(), 1, "Q=({a},{b},{d}): {dirs:?}");
            assert_all_null(a, b, d, frame, &dirs);
        }
    }
    // `[[0, 0], [0, 1]]` has the ruling `e1`.
    assert!(close_vec(
        nulls(0.0, 0.0, 1.0, xy_frame().0, xy_frame().1)[0],
        Vec3::new(1.0, 0.0, 0.0),
        0.0
    ));
    // Deterministic orientation of the `b`-dominant ruling: `(-b, a)`.
    let r = nulls(1.0, 1.0, 1.0, xy_frame().0, xy_frame().1)[0];
    assert!(close_vec(
        r,
        Vec3::new(-1.0, 1.0, 0.0) * 0.5_f64.sqrt(),
        1e-15
    ));
}

/// The classification gates are `det < -REL·||Q||²` (indefinite) and
/// `det > REL·||Q||²` (definite), both strict: at `det = ±REL·||Q||²` exactly
/// the form is parabolic within roundoff and returns one ruling.
#[test]
fn classification_gates_are_strict() {
    let (e1, e2) = xy_frame();
    // ||Q|| = 1 exactly (1 + 1e-24 rounds to 1), so the guard is 1e-12.
    for d in [-1e-12, 1e-12] {
        let dirs = null_directions(1.0, 0.0, d, e1, e2, 1.0, d);
        assert_eq!(dirs.len(), 1, "det = {d:e}: {dirs:?}");
        assert!(close_vec(dirs[0], Vec3::new(0.0, 1.0, 0.0), 0.0));
    }
}

// ---------------------------------------------------- find_branch_directions

fn branches_at_origin(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    tangent: Vec3,
    step: f64,
    tol: f64,
) -> Vec<IntersectionPoint> {
    find_branch_directions(
        s1,
        s2,
        &origin_point(),
        tangent,
        step,
        tol,
        &OperationContext::new(),
        &mut SsiScratch::new(),
    )
    .unwrap()
}

/// On the generic pair the intersection is the pair of parabolas over the
/// null lines `(1, 2)` and `(3, -1)`. Arriving along `A = (1, 2)/√5`, the
/// transverse null is `B = (3, -1)/√10`; each seed starts exactly on the
/// branch, so it is the closed-form point `s·B` lifted onto `s1`, with
/// `s = ±0.5·step/peak` and `peak` the largest parameter component of
/// `A_k⁻¹ B` over both charts. Four chart variants make each of the four
/// parameter components the dominant one in turn.
#[test]
fn transverse_seeds_sit_at_the_normalised_offset_along_the_null() {
    let a_dir = Vec3::new(1.0, 2.0, 0.0) * (1.0 / 5.0_f64.sqrt());
    let bx = 3.0 / 10.0_f64.sqrt();
    let by = -1.0 / 10.0_f64.sqrt();
    let squeeze_u = |a: [[f64; 2]; 2]| [[a[0][0] / 8.0, a[0][1]], [a[1][0] / 8.0, a[1][1]]];
    let squeeze_v = |a: [[f64; 2]; 2]| [[a[0][0], a[0][1] / 8.0], [a[1][0], a[1][1] / 8.0]];
    let variants = [
        (squeeze_u(A1), A2),
        (squeeze_v(A1), A2),
        (A1, squeeze_u(A2)),
        (A1, squeeze_v(A2)),
    ];
    let step = 0.05;
    for (k, (a1, a2)) in variants.into_iter().enumerate() {
        let (s1, s2) = generic_pair(a1, a2);
        let m1 = inverse(a1);
        let m2 = inverse(a2);
        let comps = [
            m1[0] * bx + m1[1] * by,
            m1[2] * bx + m1[3] * by,
            m2[0] * bx + m2[1] * by,
            m2[2] * bx + m2[3] * by,
        ];
        let peak = comps.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
        assert_eq!(
            comps.iter().position(|c| c.abs() == peak),
            Some(k),
            "variant {k} must make component {k} dominant: {comps:?}"
        );
        let s = 0.5 * step / peak;
        let lift = |t: f64| {
            let (x, y) = (t * bx, t * by);
            Point3::new(x, y, H1.0 * x * x + H1.1 * x * y + H1.2 * y * y)
        };
        let seeds = branches_at_origin(&s1, &s2, a_dir, step, 1e-9);
        assert_eq!(seeds.len(), 2, "variant {k}: {seeds:?}");
        for want in [lift(s), lift(-s)] {
            assert!(
                seeds.iter().any(|p| (p.point - want).length() < 1e-12),
                "variant {k}: no seed at {want:?} in {seeds:?}"
            );
        }
    }
}

/// Bisector arrival on the saddle `z = x·y` (nulls `x` and `y`): both nulls
/// are equally transverse, so both are seeded, on both sides, at `±step`
/// (the chart maps `x = 2u - 1`, so `peak = ½` and the offset is `step`).
#[test]
fn bisector_arrival_seeds_both_nulls_on_both_sides() {
    let step = 0.0625;
    let bis = Vec3::new(1.0, 1.0, 0.0) * 0.5_f64.sqrt();
    let seeds = branches_at_origin(&saddle(1.0), &plane(), bis, step, 1e-9);
    assert_eq!(seeds.len(), 4, "{seeds:?}");
    for want in [
        Point3::new(step, 0.0, 0.0),
        Point3::new(-step, 0.0, 0.0),
        Point3::new(0.0, step, 0.0),
        Point3::new(0.0, -step, 0.0),
    ] {
        assert!(
            seeds.iter().any(|p| (p.point - want).length() < 1e-15),
            "missing {want:?} in {seeds:?}"
        );
    }
    // Arriving along y: only the x null is transverse.
    let seeds = branches_at_origin(&saddle(1.0), &plane(), Vec3::new(0.0, 1.0, 0.0), step, 1e-9);
    assert_eq!(seeds.len(), 2, "{seeds:?}");
    assert!(
        seeds
            .iter()
            .all(|p| p.point.y() == 0.0 && close(p.point.x().abs(), step, 1e-15))
    );
}

/// Seed acceptance boundaries are strict. The seed offset is first measured
/// with a negligible tolerance; re-running with `tolerance` equal to that
/// distance must keep the seed (the skip is `dist < tol`, and the second
/// side then merges with the first), and with `100·tolerance` equal to the
/// two seeds' separation must keep both (the merge is `< 100·tol`), while a
/// tolerance a quarter larger merges them.
#[test]
fn seed_distance_and_merge_boundaries_are_strict() {
    let along_y = Vec3::new(0.0, 1.0, 0.0);
    let (sad, pl) = (saddle(1.0), plane());
    let o = Point3::new(0.0, 0.0, 0.0);
    let step = 2.0_f64.powi(-20);
    let probe = branches_at_origin(&sad, &pl, along_y, step, 1e-300);
    assert_eq!(probe.len(), 2, "{probe:?}");
    let dist = (probe[0].point - o).length();
    let sep = (probe[0].point - probe[1].point).length();

    // tol == dist: the first side is kept, the second merges into it.
    let seeds = branches_at_origin(&sad, &pl, along_y, step, dist);
    assert_eq!(seeds.len(), 1, "{seeds:?}");
    assert_eq!((seeds[0].point - o).length(), dist);

    // 100·tol == separation exactly: both kept.
    let mut tol = sep / 100.0;
    for _ in 0..64 {
        if tol * 100.0 == sep {
            break;
        }
        tol = if tol * 100.0 < sep {
            f64::from_bits(tol.to_bits() + 1)
        } else {
            f64::from_bits(tol.to_bits() - 1)
        };
    }
    assert!(tol * 100.0 == sep, "no tolerance with 100·tol == {sep:e}");
    let seeds = branches_at_origin(&sad, &pl, along_y, step, tol);
    assert_eq!(seeds.len(), 2, "{seeds:?}");

    // 100·tol = 1.25·separation: merged.
    let seeds = branches_at_origin(&sad, &pl, along_y, step, 1.25 * sep / 100.0);
    assert_eq!(seeds.len(), 1, "{seeds:?}");
}

/// A crossing of curvature `1e-7` (`det = -1e-14`, far below any absolute
/// threshold) is still a crossing: the gate is the relative sign of `det`.
#[test]
fn nearly_flat_crossing_still_seeds_its_transverse_branch() {
    let step = 0.0625;
    let seeds = branches_at_origin(
        &saddle(1e-7),
        &plane(),
        Vec3::new(0.0, 1.0, 0.0),
        step,
        1e-12,
    );
    assert_eq!(seeds.len(), 2, "{seeds:?}");
    assert!(
        seeds
            .iter()
            .all(|p| p.point.y() == 0.0 && close(p.point.x().abs(), step, 1e-15))
    );
}

/// The 30° branch-angle gate is `acos(|cos|) > 30°`. No `f64` cosine has an
/// `acos` exactly equal to `30°.to_radians()`: acos is monotone, and the
/// whole neighbourhood of `cos 30°` is checked here, so `>` and `>=` agree on
/// every input (the equivalence proof for that comparison, made executable).
#[test]
fn no_cosine_lands_exactly_on_the_branch_angle() {
    let target = 30.0_f64.to_radians();
    let centre = target.cos().to_bits();
    let below = f64::from_bits(centre - 4000).acos();
    let above = f64::from_bits(centre + 4000).acos();
    assert!(below > target && above < target, "window must bracket 30°");
    for bits in centre - 4000..=centre + 4000 {
        assert!(f64::from_bits(bits).acos().to_bits() != target.to_bits());
    }
}

// -------------------------------- singular tangent choice (incoming direction)

/// Among the nulls `±n0, ±n1`, the most aligned with `incoming` wins; on an
/// exact tie the first in sorted order is kept.
#[test]
fn singular_tangent_takes_the_most_aligned_oriented_null() {
    let ctx = OperationContext::new();
    let pick = |s1: &NurbsSurface, s2: &NurbsSurface, inc: Vec3| {
        singular_tangent_direction(
            s1,
            s2,
            &origin_point(),
            Some(inc),
            &ctx,
            &mut SsiScratch::new(),
        )
        .unwrap()
        .unwrap()
    };
    // Saddle: nulls ±x, ±y. Bisector tie keeps +x (sorted first among the
    // maxima); y-leaning picks +y.
    let (sad, pl) = (saddle(1.0), plane());
    let bis = Vec3::new(1.0, 1.0, 0.0) * 0.5_f64.sqrt();
    assert!(close_vec(
        pick(&sad, &pl, bis),
        Vec3::new(1.0, 0.0, 0.0),
        0.0
    ));
    assert!(close_vec(
        pick(&sad, &pl, Vec3::new(0.0, 1.0, 0.0)),
        Vec3::new(0.0, 1.0, 0.0),
        0.0
    ));
    // Generic pair: arriving along -A returns -A exactly (not a reflection).
    let a_dir = Vec3::new(1.0, 2.0, 0.0) * (1.0 / 5.0_f64.sqrt());
    let (g1, g2) = generic_pair(A1, A2);
    assert!(close_vec(pick(&g1, &g2, -a_dir), -a_dir, 1e-12));
    // Same pair turned a quarter about x, so both nulls carry a z component.
    let r1 = quadric_patch(A1, H1, rot90x);
    let r2 = quadric_patch(A2, H2, rot90x);
    let ra = Vec3::new(1.0, 0.0, 2.0) * (1.0 / 5.0_f64.sqrt());
    assert!(close_vec(pick(&r1, &r2, -ra), -ra, 1e-12));
}

/// At an exactly tangential point `n1 × n2 = 0` and the parameter tangent
/// follows the null most aligned with `prev` (which carries the march sign).
/// Saddle/plane share the chart `x = 2u - 1`, so the `±y` null maps to
/// `(0, ±1, 0, ±1)` and the `±x` null to `(±1, 0, ±1, 0)`.
#[test]
fn singular_parameter_tangent_follows_the_incoming_null() {
    let ctx = OperationContext::new();
    let tangent = |s1: &NurbsSurface, s2: &NurbsSurface, prev: Vec3| {
        ssi_tangent_params(
            s1,
            s2,
            0.5,
            0.5,
            0.5,
            0.5,
            1.0,
            Some(prev),
            &ctx,
            &mut SsiScratch::new(),
        )
        .unwrap()
        .unwrap()
    };
    let (sad, pl) = (saddle(1.0), plane());
    assert_eq!(
        tangent(&sad, &pl, Vec3::new(0.6, -0.8, 0.0)),
        [0.0, -1.0, 0.0, -1.0]
    );
    assert_eq!(
        tangent(&sad, &pl, Vec3::new(0.8, -0.6, 0.0)),
        [1.0, 0.0, 1.0, 0.0]
    );
    // Quarter turn about x: the nulls are ±x and ±z.
    let rot_saddle = quadric_patch([[2.0, 0.0], [0.0, 2.0]], (0.0, 1.0, 0.0), rot90x);
    let rot_plane = quadric_patch([[2.0, 0.0], [0.0, 2.0]], (0.0, 0.0, 0.0), rot90x);
    let t = tangent(&rot_saddle, &rot_plane, Vec3::new(0.6, 0.0, -0.8));
    assert_eq!(t, [0.0, -1.0, 0.0, -1.0]);
}

// ------------------------------------------------------------------ marching

/// A march through the nearly flat saddle `z = 1e-7·x·y` still reports the
/// transverse branch (relative `det` gate in the trace scan as well).
#[test]
fn march_through_a_nearly_flat_crossing_reports_its_branch() {
    let c = 1e-7;
    let tol = 1e-7;
    let (s1, s2) = (saddle(c), plane());
    let seed = refine_ssi_point(&s1, &s2, 0.5, 0.2, 0.5, 0.2, tol).unwrap();
    let (traced, branches) = march_with_branches(
        &s1,
        &s2,
        &seed,
        0.05,
        tol,
        &OperationContext::new(),
        &mut SsiScratch::new(),
    )
    .unwrap();
    assert!(traced.iter().any(|p| p.point.y() > 0.0) && traced.iter().any(|p| p.point.y() < 0.0));
    assert!(
        !branches.is_empty(),
        "nearly flat crossing passed without a branch"
    );
    for b in &branches {
        assert!((s1.evaluate(b.param1.0, b.param1.1) - b.point).length() <= tol);
        assert!((s2.evaluate(b.param2.0, b.param2.1) - b.point).length() <= tol);
        assert!(
            b.point.x().abs() > 10.0 * tol,
            "branch seed on the traced line: {b:?}"
        );
    }
}

/// A march seeded exactly at the crossing: its first traced point is the
/// valley minimum (index 0 of the trace), and the branch scan there must
/// still produce seeds on the intersection.
#[test]
fn march_seeded_at_the_crossing_scans_its_first_point() {
    let tol = 1e-7;
    let (s1, s2) = (saddle(0.001), plane());
    let (traced, branches) = march_with_branches(
        &s1,
        &s2,
        &origin_point(),
        0.05,
        tol,
        &OperationContext::new(),
        &mut SsiScratch::new(),
    )
    .unwrap();
    assert!(traced.len() >= 3, "{traced:?}");
    for b in &branches {
        assert!((s1.evaluate(b.param1.0, b.param1.1) - b.point).length() <= tol);
        assert!((s2.evaluate(b.param2.0, b.param2.1) - b.point).length() <= tol);
        assert!(b.point.z().abs() <= tol);
    }
}

/// `z = x³/27` (over `x ∈ [-3, 3]`) touches `z = 0` along `x = 0` to third
/// order: the curvature difference vanishes there (`Q = 0`), so only the
/// perturbation search can start the march. The trace must leave the seed
/// and stay on both surfaces.
#[test]
fn third_order_contact_line_is_marched_by_the_perturbation_search() {
    let cubic = NurbsSurface::new(
        3,
        1,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        [-3.0, -1.0, 1.0, 3.0]
            .iter()
            .zip([-1.0, 1.0, -1.0, 1.0])
            .map(|(&x, z)| vec![Point3::new(x, -2.0, z), Point3::new(x, 2.0, z)])
            .collect(),
        vec![vec![1.0; 2]; 4],
    )
    .unwrap();
    for &(u, v) in &[(0.2, 0.3), (0.5, 0.5), (0.85, 0.9)] {
        let p = cubic.evaluate(u, v);
        assert!(
            (p.z() - p.x().powi(3) / 27.0).abs() < 1e-14,
            "fixture off z = x³/27"
        );
    }
    let flat = bilinear(
        [
            [Point3::new(-4.0, -3.0, 0.0), Point3::new(-4.0, 3.0, 0.0)],
            [Point3::new(4.0, -3.0, 0.0), Point3::new(4.0, 3.0, 0.0)],
        ],
        (0.0, 1.0),
        (0.0, 1.0),
    );
    let d = diff_at_centres(&cubic, &flat).unwrap();
    assert_eq!(d.norm, 0.0, "third-order contact has Q = 0");
    let tol = 1e-7;
    let traced = march_intersection(&cubic, &flat, &origin_point(), 0.05, tol);
    assert!(
        traced.len() >= 3,
        "march stalled at the third-order seed: {traced:?}"
    );
    let span = traced
        .iter()
        .map(|p| p.point.y())
        .fold(0.0_f64, |m, y| m.max(y.abs()));
    assert!(span > 0.5, "march barely moved: {span}");
    for p in &traced {
        assert!(p.point.z().abs() <= tol, "off the plane: {p:?}");
        assert!(
            (p.point.z() - p.point.x().powi(3) / 27.0).abs() <= tol,
            "off the cubic: {p:?}"
        );
    }
}
