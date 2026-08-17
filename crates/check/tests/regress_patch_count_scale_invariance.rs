//! Quadrature density must not depend on the model's units.
//!
//! `face_integrator::patch_count` tiled a quadrature axis by comparing the raw
//! parameter span against the absolute constant `PI/4`. That is dimensionless
//! only for an ANGULAR parameter, because radians are dimensionless. A NURBS
//! knot vector carries whatever units its control points were built in, so the
//! same surface, uniformly scaled, was tiled into a different number of patches
//! and integrated to a different answer — with no error and no warning.
//!
//! The surface here is the one `remus#57` measured the defect on: the ruled
//! parabolic wall of an extruded parabolic segment, built directly as a NURBS
//! so this test does not depend on `operations`. Its knot domain is `[0, 2w]`,
//! the width the parabola spans, which is what carries the length.
//!
//! The reference is a hand closed form — the arc length of `y = x^2/w` from
//! `-w` to `w`, times the extrusion height — never another kernel measurement.
//! `mass_properties` and `solid_volume` meet in `integrate_face`, so their
//! agreement would prove nothing about this.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout
)]

use remus_check::properties::face_integrator::integrate_face;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// The ruled wall over `y = x^2 / w`, `x` in `[-w, w]`, swept `h` along +Z.
///
/// The parabola is exactly the quadratic Bezier through `(-w, w)`, `(0, -w)`,
/// `(w, w)`: that curve is `(w*s, w*s^2)` for `s = 2t - 1`, i.e. `y = x^2/w`.
///
/// Its u knot vector is `[0, 0, 0, 2w, 2w, 2w]` — a domain of length `2w`, the
/// width the curve spans. That is the shape of parameterisation a fitter
/// produces, and it is exactly what the defect keyed on: `2w` crossing `PI/4`
/// flipped the axis between one patch and two.
fn ruled_parabolic_wall(w: f64, h: f64) -> NurbsSurface {
    let span = 2.0 * w;
    let ctrl = vec![
        vec![Point3::new(-w, w, 0.0), Point3::new(-w, w, h)],
        vec![Point3::new(0.0, -w, 0.0), Point3::new(0.0, -w, h)],
        vec![Point3::new(w, w, 0.0), Point3::new(w, w, h)],
    ];
    let weights = vec![vec![1.0, 1.0], vec![1.0, 1.0], vec![1.0, 1.0]];
    NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, span, span, span],
        vec![0.0, 0.0, h, h],
        ctrl,
        weights,
    )
    .unwrap()
}

/// Exact area: arc length of `y = x^2/w` over `[-w, w]`, times `h`.
///
/// The half-arc `integral_0^1 sqrt(1 + 4x^2) dx = sqrt(5)/2 + asinh(2)/4` is
/// derived by hand in `operations/tests/conic_edges_closed_form.rs`.
fn wall_closed_form(w: f64, h: f64) -> f64 {
    let arc_unit = 2.0 * (5.0_f64.sqrt() / 2.0 + 2.0_f64.asinh() / 4.0);
    arc_unit * w * h
}

/// Integrate the area of a face covering the whole surface domain.
fn wall_area(w: f64, h: f64) -> f64 {
    let surface = ruled_parabolic_wall(w, h);
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();

    let mut topo = Topology::new();
    let vtol = 1e-12 * w;
    let corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        .map(|(u, v)| topo.add_vertex(Vertex::new(surface.evaluate(u, v), vtol)));

    let mut oriented = Vec::new();
    for i in 0..4 {
        let e = topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line));
        oriented.push(OrientedEdge::new(e, true));
    }
    let wire = topo.add_wire(Wire::new(oriented, true).unwrap());
    let face = topo.add_face(Face::new(wire, Vec::new(), FaceSurface::Nurbs(surface)));

    integrate_face(&topo, face, 8).unwrap().area
}

#[test]
fn a_nurbs_face_area_does_not_move_with_model_scale() {
    let mut readings = Vec::new();
    for k in [1000.0_f64, 1.0, 0.001] {
        let (w, h) = (k, 2.0 * k);
        let expected = wall_closed_form(w, h);
        let got = wall_area(w, h);
        let rel = (got - expected).abs() / expected;
        println!("scale {k:>8}x: closed form {expected:e}, got {got:e}, rel {rel:.4e}");
        readings.push(rel);

        assert!(
            rel < 1e-10,
            "ruled NURBS wall at {k}x read {got} against a closed-form {expected} \
             (relative {rel:e})"
        );
    }

    // The fix is not merely that each reading is accurate but that they are
    // the SAME reading. A defect keyed on the model's units shows up as a
    // spread across scales: before the fix this shape read 3.50e-14, 2.67e-11
    // and 1.24e-5 at 1000x, 1x and 0.001x — nine orders apart for one surface,
    // because its u axis was tiled into 16, 3 and 1 patches purely by size.
    let worst = readings.iter().copied().fold(0.0_f64, f64::max);
    let best = readings.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(
        worst - best < 1e-12,
        "the same surface integrated to materially different accuracy at \
         different scales ({readings:?}) — quadrature density is still keyed \
         on the model's units"
    );
}

#[test]
fn the_knot_span_crossing_pi_over_4_is_not_a_cliff() {
    // The defect's signature was a discontinuity where the raw parameter span
    // crossed PI/4. The u domain here is 2w, so the crossing is at
    // w = PI/8 = 0.3926990... Walk straight across it. remus#57 measured the
    // jump on the assembled prism as 530x (8.5e-7 -> 4.5e-4); on this bare
    // wall it was 33,600x — 3.69e-10 at w = 0.393 against 1.24e-5 at 0.3925,
    // one thousandth of a unit of half-width apart.
    let mut worst = 0.0_f64;
    let mut readings = Vec::new();
    for w in [
        1.0_f64, 0.5, 0.4, 0.3935, 0.393, 0.3925, 0.392, 0.39, 0.2, 0.1, 0.001,
    ] {
        let h = 2.0 * w;
        let expected = wall_closed_form(w, h);
        let got = wall_area(w, h);
        let rel = (got - expected).abs() / expected;
        println!("w={w:<8} (u span {:.6}): rel {rel:.4e}", 2.0 * w);
        worst = worst.max(rel);
        readings.push(rel);
    }
    assert!(
        worst < 1e-10,
        "a NURBS wall's area accuracy still depends on where its knot span \
         falls relative to PI/4 (worst relative {worst:e}, all {readings:?})"
    );
}
