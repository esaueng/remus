//! The fixed Gauss rule must converge on a NURBS face of many knot spans.
//!
//! `integrate_parametric` tiled each axis into at most `MAX_PATCHES` uniform
//! patches whatever the carrier's knots, so on a carrier of more spans than
//! that nearly every Gauss rule straddled a continuity break and the result
//! stopped converging with the order: a 35 × 55-span bicubic face read its
//! area ~1e-4 off at orders 8, 12 and 16 alike, and the 42 NURBS faces of
//! the imported hammer holder up to 2.7e-5 (B131). The fixed rule now keeps
//! every cell inside one knot span.
//!
//! The face is a ruled wall: a 37-span uniform C¹ quadratic B-spline zigzag
//! in the xy plane swept along z. Each span is a quadratic Bézier, so the
//! references are hand closed forms per span, never another kernel reading:
//! the area is the height times the sum of the spans' Bézier arc lengths, and
//! the volume term `(1/3) ∫ P · n dA` is `H/3` times the sum of each span's
//! `∫ B × B' dt = (2/3)(P0×P1 + P1×P2) + (1/3)(P0×P2)`. The profile is kept
//! shallow: a sharper zigzag puts near-cusps inside the spans, which no
//! fixed rule resolves quickly whether or not it is aligned.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::properties::face_integrator::{integrate_face, integrate_face_area};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const SPANS: usize = 37;
const HEIGHT: f64 = 3.0;

/// Clamped uniform quadratic knots over `[0, SPANS]`.
fn knots() -> Vec<f64> {
    let mut k = vec![0.0, 0.0];
    k.extend((0..=SPANS).map(|i| i as f64));
    k.extend([SPANS as f64; 2]);
    k
}

/// The zigzag's control polygon: unit steps in x, a shallow alternating y.
fn profile() -> Vec<Point3> {
    (0..SPANS + 2)
        .map(|i| {
            let side = if i % 2 == 0 { 1.0 } else { -1.0 };
            Point3::new(i as f64, side * 0.2 * (1.0 + 0.3 * (i as f64).sin()), 0.0)
        })
        .collect()
}

fn wall(topo: &mut Topology) -> (FaceId, NurbsCurve) {
    let base = profile();
    let lift = |p: Point3| Point3::new(p.x(), p.y(), HEIGHT);
    let surface = NurbsSurface::new(
        2,
        1,
        knots(),
        vec![0.0, 0.0, 1.0, 1.0],
        base.iter().map(|&p| vec![p, lift(p)]).collect(),
        vec![vec![1.0, 1.0]; base.len()],
    )
    .unwrap();
    let curve =
        |points: Vec<Point3>| NurbsCurve::new(2, knots(), points, vec![1.0; SPANS + 2]).unwrap();
    let bottom = curve(base.clone());
    let top = curve(base.iter().map(|&p| lift(p)).collect());
    let corners = [
        bottom.evaluate(0.0),
        bottom.evaluate(SPANS as f64),
        top.evaluate(SPANS as f64),
        top.evaluate(0.0),
    ];
    let v: Vec<_> = corners
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect();
    let mut trimmed = |a, b, c: NurbsCurve| {
        let mut edge = Edge::new(a, b, EdgeCurve::NurbsCurve(c));
        edge.set_trim(Some((0.0, SPANS as f64)));
        topo.add_edge(edge)
    };
    let e0 = trimmed(v[0], v[1], bottom.clone());
    let e2 = trimmed(v[3], v[2], top);
    let e1 = topo.add_edge(Edge::new(v[1], v[2], EdgeCurve::Line));
    let e3 = topo.add_edge(Edge::new(v[3], v[0], EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, true),
                OrientedEdge::new(e2, false),
                OrientedEdge::new(e3, true),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface)));
    (face, bottom)
}

/// Each span's quadratic Bézier control points, read off the curve: the span
/// ends, and the middle point from `B(1/2) = (B0 + 2 B1 + B2) / 4`.
fn bezier_spans(curve: &NurbsCurve) -> Vec<[(f64, f64); 3]> {
    (0..SPANS)
        .map(|i| {
            let (a, b) = (i as f64, (i + 1) as f64);
            let p0 = curve.evaluate(a);
            let p2 = curve.evaluate(b);
            let m = curve.evaluate(f64::midpoint(a, b));
            [
                (p0.x(), p0.y()),
                (
                    2.0f64.mul_add(m.x(), -0.5 * (p0.x() + p2.x())),
                    2.0f64.mul_add(m.y(), -0.5 * (p0.y() + p2.y())),
                ),
                (p2.x(), p2.y()),
            ]
        })
        .collect()
}

/// Arc length of a planar quadratic Bézier in closed form.
///
/// `|B'(t)| = 2 √(a t² + b t + c)` with `A = B1 − B0`, `D = B2 − 2 B1 + B0`,
/// `a = D·D`, `b = 2 A·D`, `c = A·A`; the antiderivative of `√Q` is
/// `(2at + b)√Q / (4a) + Δ / (8 a^{3/2}) · asinh((2at + b) / √Δ)` with
/// `Δ = 4ac − b² = 4 |A × D|² > 0` for a span that is not straight.
fn bezier_length([p0, p1, p2]: [(f64, f64); 3]) -> f64 {
    let ax = (p1.0 - p0.0, p1.1 - p0.1);
    let d = (p2.0 - 2.0 * p1.0 + p0.0, p2.1 - 2.0 * p1.1 + p0.1);
    let a = d.0 * d.0 + d.1 * d.1;
    let b = 2.0 * (ax.0 * d.0 + ax.1 * d.1);
    let c = ax.0 * ax.0 + ax.1 * ax.1;
    let cross = ax.0 * d.1 - ax.1 * d.0;
    let delta = 4.0 * cross * cross;
    assert!(delta > 0.0, "every zigzag span bends");
    let antiderivative = |t: f64| {
        let q = (a * t).mul_add(t, b.mul_add(t, c));
        let s = 2.0 * a * t + b;
        s * q.sqrt() / (4.0 * a) + delta / (8.0 * a * a.sqrt()) * (s / delta.sqrt()).asinh()
    };
    2.0 * (antiderivative(1.0) - antiderivative(0.0))
}

/// `∫ B × B' dt` of a planar quadratic Bézier (z component).
fn bezier_swept([p0, p1, p2]: [(f64, f64); 3]) -> f64 {
    let x = |p: (f64, f64), q: (f64, f64)| p.0 * q.1 - p.1 * q.0;
    (2.0 / 3.0) * (x(p0, p1) + x(p1, p2)) + x(p0, p2) / 3.0
}

#[test]
fn many_span_ruled_wall_matches_closed_form_at_every_order() {
    let mut topo = Topology::new();
    let (face, profile) = wall(&mut topo);
    let spans = bezier_spans(&profile);
    let area: f64 = HEIGHT * spans.iter().copied().map(bezier_length).sum::<f64>();
    let volume: f64 = HEIGHT / 3.0 * spans.iter().copied().map(bezier_swept).sum::<f64>();

    for order in [5, 8] {
        let c = integrate_face(&topo, face, order).unwrap();
        let area_err = (c.area - area).abs() / area;
        let volume_err = (c.volume - volume).abs() / area;
        assert!(
            area_err <= 1e-12,
            "order {order}: area {} vs closed form {area} (rel {area_err:e})",
            c.area
        );
        assert!(
            volume_err <= 1e-12,
            "order {order}: volume {} vs closed form {volume} (rel {volume_err:e})",
            c.volume
        );
        // The area-only rule integrates the same samples.
        let area_only = integrate_face_area(&topo, face, order).unwrap();
        assert_eq!(area_only.to_bits(), c.area.to_bits(), "order {order}");
    }
}
