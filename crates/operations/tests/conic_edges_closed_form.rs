//! End-to-end checks for the `EdgeCurve::Hyperbola` / `EdgeCurve::Parabola`
//! variants against HAND-DERIVED closed forms.
//!
//! Deliberately avoids cross-checking one kernel routine against another:
//! every expected value below is either an analytic formula written out here
//! or an independently-implemented numerical integral. In particular nothing
//! here is validated by "`mass_properties` agrees with `solid_volume`" —
//! those two share `integrate_face` and their agreement proves nothing.
//!
//! Each case is also run at 1x, 1000x and 0.001x, because an absolute
//! tolerance carrying units of length is a recurring defect class in this
//! codebase and only a scale sweep exposes it.

#![allow(clippy::unwrap_used, clippy::panic)]

use remus_math::curves::{Hyperbola3D, Parabola3D};
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::edge_length;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::vertex::Vertex;

/// Model scales the whole suite is swept over.
const SCALES: [f64; 3] = [1.0, 1000.0, 0.001];

/// Composite Simpson's rule, implemented locally so it shares no code with
/// anything under test.
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let n = if n.is_multiple_of(2) { n } else { n + 1 };
    #[allow(clippy::cast_precision_loss)]
    let h = (b - a) / n as f64;
    let mut total = f(a) + f(b);
    for i in 1..n {
        #[allow(clippy::cast_precision_loss)]
        let x = h.mul_add(i as f64, a);
        total += f(x) * if i.is_multiple_of(2) { 2.0 } else { 4.0 };
    }
    total * h / 3.0
}

fn add_conic_edge(topo: &mut Topology, curve: EdgeCurve, start: Point3, end: Point3) -> f64 {
    let tol = 1e-7;
    let v0 = topo.add_vertex(Vertex::new(start, tol));
    let v1 = topo.add_vertex(Vertex::new(end, tol));
    let eid = topo.add_edge(Edge::new(v0, v1, curve));
    edge_length(topo, eid).unwrap()
}

#[test]
fn parabolic_edge_length_matches_the_analytic_integral_at_every_scale() {
    // Focal length f = k/4 makes the curve y = x^2 / k in its own plane, whose
    // arc length from x = 0 to x = k is k times the unit case:
    //     integral_0^1 sqrt(1 + 4x^2) dx = sqrt(5)/2 + asinh(2)/4
    let unit = 5.0_f64.sqrt() / 2.0 + 2.0_f64.asinh() / 4.0;

    for k in SCALES {
        let p = Parabola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.25 * k,
        )
        .unwrap();
        let (t0, t1) = (0.0, k);

        let mut topo = Topology::new();
        let got = add_conic_edge(
            &mut topo,
            EdgeCurve::Parabola(p.clone()),
            p.evaluate(t0),
            p.evaluate(t1),
        );
        let expected = k * unit;

        // Relative tolerance only — an absolute one would pass at 0.001x for
        // any implementation and fail at 1000x for a correct one.
        let rel = (got - expected).abs() / expected;
        assert!(
            rel < 1e-12,
            "parabolic edge length at {k}x: got {got}, closed form {expected} (rel {rel:.3e})"
        );
    }
}

#[test]
fn parabolic_edge_length_is_not_the_chord_or_the_full_curve() {
    // Guards the specific failure modes a silent fallback would produce:
    // collapsing to the chord (too short) or ignoring the trim (too long).
    let p = Parabola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.25,
    )
    .unwrap();
    let (start, end) = (p.evaluate(0.0), p.evaluate(1.0));
    let chord = (end - start).length();

    let mut topo = Topology::new();
    let got = add_conic_edge(&mut topo, EdgeCurve::Parabola(p), start, end);

    assert!(
        got > chord * 1.02,
        "parabolic edge length {got} collapsed toward its chord {chord}"
    );
    assert!((got - 1.478_942_857_544_597_5).abs() < 1e-12, "{got}");
}

#[test]
fn hyperbolic_edge_length_matches_independent_quadrature_at_every_scale() {
    for k in SCALES {
        let (a, b) = (2.0 * k, 3.0 * k);
        let h = Hyperbola3D::with_axes(
            Point3::new(-k, 2.0 * k, 0.5 * k),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            a,
            b,
        )
        .unwrap();
        // Hyperbola parameters are dimensionless, so the SAME span is used at
        // every scale; the length must scale purely with k.
        let (t0, t1) = (-1.25, 2.0);

        let mut topo = Topology::new();
        let got = add_conic_edge(
            &mut topo,
            EdgeCurve::Hyperbola(h.clone()),
            h.evaluate(t0),
            h.evaluate(t1),
        );
        let expected = simpson(|t| (a * t.sinh()).hypot(b * t.cosh()), t0, t1, 200_000);

        let rel = (got - expected).abs() / expected;
        assert!(
            rel < 1e-9,
            "hyperbolic edge length at {k}x: got {got}, Simpson {expected} (rel {rel:.3e})"
        );
    }
}

#[test]
fn conic_edge_length_is_orientation_independent() {
    // Swapping the vertices reverses the recovered parameter span; the length
    // must not change sign or magnitude.
    let h = Hyperbola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        2.0,
        3.0,
    )
    .unwrap();
    let (s, e) = (h.evaluate(-0.8), h.evaluate(1.4));

    let mut topo = Topology::new();
    let fwd = add_conic_edge(&mut topo, EdgeCurve::Hyperbola(h.clone()), s, e);
    let rev = add_conic_edge(&mut topo, EdgeCurve::Hyperbola(h), e, s);
    assert!(fwd > 0.0);
    assert!(
        (fwd - rev).abs() < 1e-12 * fwd,
        "forward {fwd} vs reversed {rev}"
    );
}
