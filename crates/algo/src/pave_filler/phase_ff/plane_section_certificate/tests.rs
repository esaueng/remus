//! Closed-form oracles for the certificate's own parts: the parameter
//! intervals a sinusoid stays within a band on, their merging round the
//! turn, the side test on loops with arcs, and the gate's three answers.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI, TAU};

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Vec2, Vec3};
use remus_topology::face::Face;

use super::super::plane_hole_tests::{circle_wire, plane_at, polygon_wire};
use super::*;

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn span_close(span: Span, lo: f64, hi: f64) -> bool {
    (span.lo - lo.rem_euclid(TAU)).abs() < 1e-12 && (span.length() - (hi - lo)).abs() < 1e-12
}

/// `cos(t − φ) ≥ ½` is the arc of half-width π/3 about φ; a band strictly
/// inside the amplitude is two arcs; one that holds the whole sinusoid is
/// the whole turn, and one it never reaches is empty.
#[test]
fn a_sinusoid_band_is_its_closed_form_arcs() {
    let phase = 0.7;
    let one = sinusoid_band(2.0, phase, 1.0, 2.0, 10.0);
    assert_eq!(one.len(), 1);
    assert!(
        span_close(one[0], phase - FRAC_PI_3, phase + FRAC_PI_3),
        "{one:?}"
    );

    // 0 ≤ cos ≤ ½: π/3 ≤ |t − φ| ≤ π/2.
    let two = sinusoid_band(1.0, phase, 0.0, 0.0, 0.5);
    assert_eq!(two.len(), 2);
    assert!(
        span_close(two[0], phase + FRAC_PI_3, phase + FRAC_PI_2),
        "{two:?}"
    );
    assert!(
        span_close(two[1], phase - FRAC_PI_2, phase - FRAC_PI_3),
        "{two:?}"
    );

    // cos ≤ −½: the arc of half-width π/3 about φ + π.
    let far = sinusoid_band(1.0, phase, 0.0, -5.0, -0.5);
    assert_eq!(far.len(), 1);
    assert!(
        span_close(far[0], phase + 2.0 * FRAC_PI_3, phase + 4.0 * FRAC_PI_3),
        "{far:?}"
    );

    let whole = sinusoid_band(1.0, phase, 0.0, -1.0, 1.0);
    assert_eq!(whole, vec![Span { lo: 0.0, hi: TAU }]);
    assert!(sinusoid_band(1.0, phase, 0.0, 1.5, 2.0).is_empty());
    assert!(sinusoid_band(1.0, phase, 0.0, -3.0, -1.5).is_empty());
    // No amplitude: the constant is in the band or it is not.
    assert_eq!(sinusoid_band(0.0, 0.0, 0.3, 0.0, 1.0).len(), 1);
    assert!(sinusoid_band(0.0, 0.0, 1.3, 0.0, 1.0).is_empty());
}

/// At a tangency the band's half-width is `√(2 δ / amp)`, a number `acos`
/// loses half the digits of near ±1: with δ = 1e-12 it is 1.4142e-6, and it
/// comes out to a part in a million.
#[test]
fn a_band_at_a_tangency_keeps_its_width() {
    for (amp, delta) in [(1.0, 1e-12), (5.0e6, 1e-7), (1.0e-3, 1e-9)] {
        let band = sinusoid_band(amp, 0.0, 0.0, amp - delta, f64::INFINITY);
        assert_eq!(band.len(), 1);
        let half = 0.5 * band[0].length();
        // The depth as the doubles hold it.
        let delta = amp - (amp - delta);
        let wanted = (2.0 * delta / amp).sqrt();
        assert!(
            (half - wanted).abs() <= 1e-6 * wanted,
            "amp {amp}: half-width {half:e} vs {wanted:e}"
        );
    }
}

/// Stretches merge where they overlap, round the turn as well, and report
/// the edges they came from; stretches that cover the turn are no zones.
#[test]
fn stretches_merge_into_zones_round_the_turn() {
    let stretch = |lo: f64, hi: f64, edge: usize| Stretch {
        span: Span::new(lo, hi),
        edge,
    };
    let zones = merge_stretches(vec![
        stretch(1.0, 1.2, 0),
        stretch(1.1, 1.5, 1),
        stretch(3.0, 3.1, 2),
        stretch(6.2, 6.4, 3),
        stretch(0.05, 0.2, 4),
    ])
    .unwrap();
    assert_eq!(zones.len(), 3, "{zones:?}");
    assert!(span_close(zones[0].0, 1.0, 1.5) && zones[0].1 == vec![0, 1]);
    assert!(span_close(zones[1].0, 3.0, 3.1) && zones[1].1 == vec![2]);
    // 6.2 .. 6.4 runs through the turn into 0.05 .. 0.2.
    assert!(span_close(zones[2].0, 6.2, TAU + 0.2), "{zones:?}");
    assert_eq!(zones[2].1, vec![3, 4]);

    assert!(merge_stretches(vec![stretch(0.0, 3.5, 0), stretch(3.0, 6.5, 1)]).is_none());
    assert_eq!(merge_stretches(Vec::new()).unwrap().len(), 0);
}

/// The side of a loop with arcs, against the same loop as a fine polygon:
/// a "D" (a diameter and a half circle) and a rounded square. A point on
/// an arc's circle away from the arc, or on its chord, is decided; only
/// within `eps` of the loop itself is it not.
#[test]
fn the_side_of_a_loop_with_arcs_matches_its_polygon() {
    let p2 = |x: f64, y: f64| Point2::new(x, y);
    let polar = |c: Point2, r: f64, deg: f64| {
        let t = deg.to_radians();
        Point2::new(c.x() + r * t.cos(), c.y() + r * t.sin())
    };
    let arc = |c: Point2, r: f64, from: f64, to: f64| PlanarLoopEdge::Arc {
        center: c,
        radius: r,
        start: polar(c, r, from),
        mid: polar(c, r, 0.5 * (from + to)),
        end: polar(c, r, to),
    };
    let o = p2(0.0, 0.0);
    let d_shape = vec![
        PlanarLoopEdge::Segment(polar(o, 1.0, 180.0), polar(o, 1.0, 0.0)),
        arc(o, 1.0, 0.0, 180.0),
    ];
    // A square of half-size 1 with its corners rounded at radius 0.25.
    let corner = |sx: f64, sy: f64| p2(0.75 * sx, 0.75 * sy);
    let rounded = vec![
        PlanarLoopEdge::Segment(p2(-0.75, -1.0), p2(0.75, -1.0)),
        arc(corner(1.0, -1.0), 0.25, 270.0, 360.0),
        PlanarLoopEdge::Segment(p2(1.0, -0.75), p2(1.0, 0.75)),
        arc(corner(1.0, 1.0), 0.25, 0.0, 90.0),
        PlanarLoopEdge::Segment(p2(0.75, 1.0), p2(-0.75, 1.0)),
        arc(corner(-1.0, 1.0), 0.25, 90.0, 180.0),
        PlanarLoopEdge::Segment(p2(-1.0, 0.75), p2(-1.0, -0.75)),
        arc(corner(-1.0, -1.0), 0.25, 180.0, 270.0),
    ];
    for (name, outline) in [("D", d_shape), ("rounded", rounded)] {
        // The loop as a polygon, each arc in 720 steps.
        let mut polygon: Vec<Point2> = Vec::new();
        for edge in &outline {
            match *edge {
                PlanarLoopEdge::Segment(a, _) => polygon.push(a),
                PlanarLoopEdge::Arc {
                    center,
                    radius,
                    start,
                    mid,
                    end,
                } => {
                    let angle = |p: Point2| (p.y() - center.y()).atan2(p.x() - center.x());
                    let (a0, am, a1) = (angle(start), angle(mid), angle(end));
                    let sweep = 2.0 * ((am - a0 + PI).rem_euclid(TAU) - PI);
                    assert!(((a0 + sweep - a1 + PI).rem_euclid(TAU) - PI).abs() < 1e-9);
                    for k in 0..720 {
                        let t = a0 + sweep * f64::from(k) / 720.0;
                        polygon.push(p2(
                            center.x() + radius * t.cos(),
                            center.y() + radius * t.sin(),
                        ));
                    }
                }
                PlanarLoopEdge::Circle { .. } => panic!("no full circle here"),
            }
        }
        let distance = |p: Point2| {
            (0..polygon.len())
                .map(|i| {
                    let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
                    let ab: Vec2 = b - a;
                    let along = (dot_2d(p - a, ab) / dot_2d(ab, ab)).clamp(0.0, 1.0);
                    ((p - a) - ab * along).length()
                })
                .fold(f64::INFINITY, f64::min)
        };
        let reversed: Vec<PlanarLoopEdge> = outline
            .iter()
            .rev()
            .map(|edge| match *edge {
                PlanarLoopEdge::Segment(a, b) => PlanarLoopEdge::Segment(b, a),
                PlanarLoopEdge::Arc {
                    center,
                    radius,
                    start,
                    mid,
                    end,
                } => PlanarLoopEdge::Arc {
                    center,
                    radius,
                    start: end,
                    mid,
                    end: start,
                },
                circle @ PlanarLoopEdge::Circle { .. } => circle,
            })
            .collect();
        let mut decided = 0;
        for i in 0..=80 {
            for j in 0..=80 {
                let p = p2(
                    -1.5 + 3.0 * f64::from(i) / 80.0,
                    -1.5 + 3.0 * f64::from(j) / 80.0,
                );
                let side = loop_side(p, &outline, 1e-7);
                // The second evaluation, by a different method, and with
                // the loop traversed the other way round.
                let ray = ray_side(p, &outline, 1e-7);
                if distance(p) > 1e-3 {
                    let inside = crate::builder::classify_2d::point_in_polygon_2d(p, &polygon);
                    assert_eq!(side, Some(inside), "{name}: {p:?}");
                    assert_eq!(ray, Some(inside), "{name}: ray at {p:?}");
                    assert_eq!(ray_side(p, &reversed, 1e-7), Some(inside), "{name}: {p:?}");
                    decided += 1;
                } else if distance(p) < 1e-9 {
                    assert_eq!(side, None, "{name}: on the loop at {p:?}");
                    assert_eq!(ray, None, "{name}: ray on the loop at {p:?}");
                }
            }
        }
        assert!(decided > 6000, "{name}: {decided}");
    }

    // The D again: on its arc's circle below the diameter (the other half
    // of the circle, not part of the loop) and on the diameter's line
    // outside it, the point is decided — outside.
    let d_shape = [
        PlanarLoopEdge::Segment(polar(o, 1.0, 180.0), polar(o, 1.0, 0.0)),
        arc(o, 1.0, 0.0, 180.0),
    ];
    assert_eq!(loop_side(polar(o, 1.0, 270.0), &d_shape, 1e-7), Some(false));
    assert_eq!(loop_side(p2(1.5, 0.0), &d_shape, 1e-7), Some(false));
    assert_eq!(loop_side(p2(0.0, 0.5), &d_shape, 1e-7), Some(true));
    assert_eq!(loop_side(polar(o, 1.0, 90.0), &d_shape, 1e-7), None);
    assert_eq!(loop_side(p2(0.3, 0.0), &d_shape, 1e-7), None);
    // And by the ray count: the same answers, the arc's carrier and the
    // diameter's line away from the loop included.
    assert_eq!(ray_side(polar(o, 1.0, 270.0), &d_shape, 1e-7), Some(false));
    assert_eq!(ray_side(p2(1.5, 0.0), &d_shape, 1e-7), Some(false));
    assert_eq!(ray_side(p2(0.0, 0.5), &d_shape, 1e-7), Some(true));
    assert_eq!(ray_side(polar(o, 1.0, 90.0), &d_shape, 1e-7), None);
    assert_eq!(ray_side(p2(0.3, 0.0), &d_shape, 1e-7), None);
    // A full circle, and a reflex arc (three quarters of a turn) closed by
    // its chord: inside the chord's triangle, inside the far side of the
    // arc, and outside.
    let disc = [PlanarLoopEdge::Circle {
        center: p2(2.0, 1.0),
        radius: 0.5,
        sense: -1.0,
    }];
    let pacman = [
        arc(o, 1.0, 45.0, 315.0),
        PlanarLoopEdge::Segment(polar(o, 1.0, 315.0), polar(o, 1.0, 45.0)),
    ];
    for (outline, p, inside) in [
        (&disc[..], p2(2.2, 1.1), true),
        (&disc[..], p2(2.6, 1.0), false),
        (&pacman[..], p2(-0.5, 0.0), true),
        (&pacman[..], p2(0.5, 0.0), true),
        (&pacman[..], p2(0.9, 0.0), false),
        (&pacman[..], p2(0.0, 1.2), false),
    ] {
        assert_eq!(ray_side(p, outline, 1e-7), Some(inside), "{p:?}");
        assert_eq!(loop_side(p, outline, 1e-7), Some(inside), "{p:?}");
    }
}

/// An arc's zone is the arc's, not its circle's.
///
/// The unit circle against an arc of the same circle of half-width `half`:
/// the zone is the arc and the band's angle past each end, whatever the
/// arc's length. (Cut off by the arc's chord moved a band inward, the zone
/// of an arc of ±1e-4 rad reached ±4.6e-4.) An arc of more than a half turn
/// is the union of its two half-planes; one whose circle is a little off
/// the section's has the zone where the two are within the band, inside
/// the arc's own turn only; and an arc too small to bracket is not read.
#[test]
fn an_arcs_zone_ends_at_the_arcs_ends() {
    let model = SectionTolerance::at(1e-7, 1.0);
    let section =
        Circle3D::new_with_ref(Point3::new(0.0, 0.0, 0.0), Z, 1.0, Vec3::new(1.0, 0.0, 0.0))
            .unwrap();
    let conic = SectionConic::of_circle(&section);
    let total = |spans: &[Span]| spans.iter().map(|span| span.length()).sum::<f64>();
    let o = Point3::new(0.0, 0.0, 0.0);
    for half in [1e-4_f64, 1e-2, 0.7, FRAC_PI_2, 2.5, 3.1] {
        for mid in [0.0_f64, 2.3] {
            let direction = Vec3::new(mid.cos(), mid.sin(), 0.0);
            let (spans, _) = arc_stretch(&conic, o, 1.0, Some((direction, half)), model).unwrap();
            // The stretches of one arc may overlap (a reflex arc's two
            // half-planes): merge them.
            let merged = merge_stretches(
                spans
                    .iter()
                    .map(|&span| Stretch { span, edge: 0 })
                    .collect(),
            )
            .unwrap();
            assert_eq!(merged.len(), 1, "half {half}, mid {mid}: {spans:?}");
            let zone = merged[0].0;
            let wanted = 2.0 * (half + 1e-7);
            assert!(
                (zone.length() - wanted).abs() < 1e-9,
                "half {half}, mid {mid}: zone {:e} long, the arc and its ends {wanted:e}",
                zone.length()
            );
            // Centred on the arc.
            assert!(zone.clearance(mid) <= 0.0 && zone.clearance(mid + half) <= 0.0);
            assert!(zone.clearance(mid + half + 3e-7) > 0.0);
            assert!(zone.clearance(mid - half - 3e-7) > 0.0);
        }
    }
    // The whole circle has no ends.
    let (whole, _) = arc_stretch(&conic, o, 1.0, None, model).unwrap();
    assert!((total(&whole) - TAU).abs() < 1e-12, "{whole:?}");

    // An arc of ±0.5 on the circle of radius 1 about (0.2, 0): the section
    // crosses that circle at x = 0.1, y = ±√0.99 — angles ±1.67 from its
    // centre, outside the arc. No zone; the carrier is not the edge.
    let off = Point3::new(0.2, 0.0, 0.0);
    let east = Vec3::new(1.0, 0.0, 0.0);
    let (spans, _) = arc_stretch(&conic, off, 1.0, Some((east, 0.5)), model).unwrap();
    assert!(spans.is_empty(), "{spans:?}");
    // The same circle's arc of ±1.8 holds both crossings: two zones, each
    // where the circles are within the band (2e-7 / sin of their angle).
    let (spans, _) = arc_stretch(&conic, off, 1.0, Some((east, 1.8)), model).unwrap();
    assert_eq!(spans.len(), 2, "{spans:?}");
    let crossing = 0.99_f64.sqrt().atan2(0.1);
    for span in &spans {
        assert!(span.clearance(crossing) <= 0.0 || span.clearance(-crossing) <= 0.0);
        assert!(span.length() < 1e-5, "{span:?}");
    }

    // An arc of radius 1e-6 (ten bands): its sector's corner lies farther
    // than the zone's reach from it. Not read.
    assert!(
        arc_stretch(
            &conic,
            Point3::new(1.0, 0.0, 0.0),
            1e-6,
            Some((east, 0.5)),
            model
        )
        .is_none()
    );
}

/// A square plate `[−10, 10]²` at z = 0 with the given holes, and the unit
/// circle about `at` as the section of a vertical cylinder.
fn plate(
    holes: impl FnOnce(&mut Topology) -> Vec<WireId>,
    at: (f64, f64),
) -> (Topology, FaceId, EdgeCurve, FaceSurface) {
    let mut topo = Topology::new();
    let outer = polygon_wire(
        &mut topo,
        &[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)],
        0.0,
    );
    let inner = holes(&mut topo);
    let face = topo.add_face(Face::new(outer, inner, plane_at(0.0)));
    let centre = Point3::new(at.0, at.1, 0.0);
    // Parameter zero on +x, so that `on_circle` gives a point's parameter.
    let section = EdgeCurve::Circle(
        Circle3D::new_with_ref(centre, Z, 1.0, Vec3::new(1.0, 0.0, 0.0)).unwrap(),
    );
    let partner =
        FaceSurface::Cylinder(CylindricalSurface::new(centre + Z * -1.0, Z, 1.0).unwrap());
    (topo, face, section, partner)
}

fn on_circle(at: (f64, f64), t: f64) -> (f64, Point3) {
    (t, Point3::new(at.0 + t.cos(), at.1 + t.sin(), 0.0))
}

/// The gate's three answers. The unit circle about the origin crosses a
/// square hole `[0.5, 3] × [−0.3, 0.3]` at y = ±0.3 (parameters ±asin 0.3).
/// A span that ends on the hole is certified on the face or off it; a span
/// that ends on no hole is kept as the collector says, whatever the hole
/// does elsewhere; a span the collector drops is dropped.
#[test]
fn the_gate_certifies_spans_on_a_hole_and_leaves_the_rest_alone() {
    let (topo, face, section, partner) = plate(
        |topo| {
            vec![polygon_wire(
                topo,
                &[(0.5, -0.3), (3.0, -0.3), (3.0, 0.3), (0.5, 0.3)],
                0.0,
            )]
        },
        (0.0, 0.0),
    );
    let certificate =
        PlaneSectionCertificate::of_face(&topo, face, &section, Some(&partner), Tolerance::new())
            .unwrap();
    let up = 0.3_f64.asin();
    let crossings: Vec<f64> = certificate.hole_crossings().map(|(_, c)| c.t).collect();
    assert_eq!(crossings.len(), 2);
    for wanted in [up, TAU - up] {
        assert!(
            crossings.iter().any(|t| (t - wanted).abs() < 1e-12),
            "{crossings:?}"
        );
    }
    // Split at the two crossings and at two points that are no loop's
    // (a partner face's seam, say): π/2 and π.
    let at = (0.0, 0.0);
    let splits = [
        on_circle(at, up),
        on_circle(at, FRAC_PI_2),
        on_circle(at, PI),
        on_circle(at, TAU - up),
    ];
    let plan = certificate.plan(&splits);
    assert!(plan.first_refusal().is_none());
    // Crossing → π/2: on the face, certified.
    assert_eq!(
        plan.keep(0, (up, FRAC_PI_2), Elsewhere::On).unwrap(),
        Kept::Certified
    );
    // π/2 → π: ends on no hole.
    assert_eq!(
        plan.keep(1, (FRAC_PI_2, PI), Elsewhere::On).unwrap(),
        Kept::AsBefore
    );
    // π → the other crossing: on the face.
    assert_eq!(
        plan.keep(2, (PI, TAU - up), Elsewhere::On).unwrap(),
        Kept::Certified
    );
    // Crossing → crossing, through the hole: certified off the face.
    assert_eq!(
        plan.keep(3, (TAU - up, TAU + up), Elsewhere::On).unwrap(),
        Kept::No
    );
    // Whatever the collector drops stays dropped.
    for (index, span) in [(0, (up, FRAC_PI_2)), (1, (FRAC_PI_2, PI))] {
        assert_eq!(plan.keep(index, span, Elsewhere::Off).unwrap(), Kept::No);
    }
    // The splitter's rule agrees with the kept spans' ends.
    plan.certify_hole_splits(&[0, 1, 2]).unwrap();

    // A hole crossing left out of the split points: the span that would
    // run across the hole is refused, the one clear of it is not.
    let plan = certificate.plan(&splits[..3]);
    let error = plan
        .keep(2, (PI, TAU + up), Elsewhere::On)
        .expect_err("a span over a crossing that is not a split point");
    assert!(
        error
            .to_string()
            .contains("not among the points it is split at"),
        "{error}"
    );
    assert_eq!(
        plan.keep(1, (FRAC_PI_2, PI), Elsewhere::On).unwrap(),
        Kept::AsBefore
    );
}

/// An elliptical hole (a full ellipse about `at`, semi-axes `a` along x and
/// `b`), which the certificate cannot read.
fn elliptical_hole(topo: &mut Topology, at: (f64, f64), (a, b): (f64, f64)) -> WireId {
    let ellipse = Ellipse3D::new_with_ref(
        Point3::new(at.0, at.1, 0.0),
        Z,
        a,
        b,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let seam = super::super::plane_hole_tests::vertex(topo, ellipse.evaluate(0.0));
    let mut edge =
        super::super::Edge::with_tolerance(seam, seam, EdgeCurve::Ellipse(ellipse), Some(1e-7));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    topo.add_wire(
        remus_topology::wire::Wire::new(
            vec![remus_topology::wire::OrientedEdge::new(edge, true)],
            true,
        )
        .unwrap(),
    )
}

/// A hole the certificate cannot read (an elliptical one) refuses every
/// span that reaches it, whether the span ends on another hole or on none:
/// a span that ends on no hole is emitted only when it is shown to pass
/// through none. A span shown clear of the ellipse is kept — its box may
/// well meet the ellipse's — and one shown to lie inside it is dropped.
#[test]
fn an_unread_hole_refuses_every_span_that_reaches_it() {
    let up = 0.3_f64.asin();
    let at = (0.0, 0.0);
    // The ellipse: across the section at (−1, 0); just inside it, its far
    // end 2e-7 short of the section; well inside it; far away.
    for (centre, axes, reached) in [
        ((-1.0, 0.0), (0.2, 0.1), true),
        ((-0.8 + 2e-7, 0.0), (0.2, 0.1), false),
        ((-0.5, 0.0), (0.2, 0.1), false),
        ((0.0, 5.0), (0.2, 0.1), false),
    ] {
        let (topo, face, section, partner) = plate(
            |topo| {
                vec![
                    polygon_wire(
                        topo,
                        &[(0.5, -0.3), (3.0, -0.3), (3.0, 0.3), (0.5, 0.3)],
                        0.0,
                    ),
                    elliptical_hole(topo, centre, axes),
                ]
            },
            at,
        );
        let certificate = PlaneSectionCertificate::of_face(
            &topo,
            face,
            &section,
            Some(&partner),
            Tolerance::new(),
        )
        .unwrap();
        let splits = [
            on_circle(at, up),
            on_circle(at, FRAC_PI_2),
            on_circle(at, 1.5 * PI),
            on_circle(at, TAU - up),
        ];
        let plan = certificate.plan(&splits);
        // Crossing → π/2 stays in x > 0, clear of every one of them.
        assert_eq!(
            plan.keep(0, (up, FRAC_PI_2), Elsewhere::On).unwrap(),
            Kept::Certified,
            "{centre:?}"
        );
        // π/2 → 3π/2 passes (−1, 0) and ends on no hole.
        let passing = plan.keep(1, (FRAC_PI_2, 1.5 * PI), Elsewhere::On);
        // With the split at π/2 gone, the span from the crossing runs on
        // to 3π/2, through (−1, 0).
        let plan = certificate.plan(&[splits[0], splits[2], splits[3]]);
        let long = plan.keep(0, (up, 1.5 * PI), Elsewhere::On);
        if reached {
            for refused in [passing, long] {
                let error = refused.expect_err("a span that reaches an unread hole");
                assert!(error.to_string().contains("no exact contact"), "{error}");
            }
        } else {
            assert_eq!(passing.unwrap(), Kept::AsBefore, "{centre:?}");
            assert_eq!(long.unwrap(), Kept::Certified, "{centre:?}");
        }
    }

    // An ellipse that holds the whole section: every span lies inside the
    // hole, and is dropped. One the section holds: every span is clear.
    for (axes, kept) in [((3.0, 2.0), Kept::No), ((0.6, 0.3), Kept::AsBefore)] {
        let (topo, face, section, partner) =
            plate(|topo| vec![elliptical_hole(topo, (0.1, 0.0), axes)], at);
        let certificate = PlaneSectionCertificate::of_face(
            &topo,
            face,
            &section,
            Some(&partner),
            Tolerance::new(),
        )
        .unwrap();
        let splits = [on_circle(at, 1.0), on_circle(at, 4.0)];
        let plan = certificate.plan(&splits);
        assert_eq!(
            plan.keep(0, (1.0, 4.0), Elsewhere::On).unwrap(),
            kept,
            "{axes:?}"
        );
        assert_eq!(
            plan.keep(1, (4.0, 1.0 + TAU), Elsewhere::On).unwrap(),
            kept,
            "{axes:?}"
        );
    }
}

/// A hole bounded by one closed spline: the rational quadratic circle of
/// radius `r` about `at`, which the certificate cannot read.
fn spline_hole(topo: &mut Topology, at: (f64, f64), r: f64) -> WireId {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let net: Vec<Point3> = [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (-1.0, 0.0),
        (-1.0, -1.0),
        (0.0, -1.0),
        (1.0, -1.0),
        (1.0, 0.0),
    ]
    .iter()
    .map(|&(x, y)| Point3::new(r.mul_add(x, at.0), r.mul_add(y, at.1), 0.0))
    .collect();
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0];
    let curve = remus_math::nurbs::curve::NurbsCurve::new(
        2,
        knots,
        net.clone(),
        vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0],
    )
    .unwrap();
    let seam = super::super::plane_hole_tests::vertex(topo, net[0]);
    let mut edge =
        super::super::Edge::with_tolerance(seam, seam, EdgeCurve::NurbsCurve(curve), Some(1e-7));
    edge.set_trim(Some((0.0, 4.0)));
    let edge = topo.add_edge(edge);
    topo.add_wire(
        remus_topology::wire::Wire::new(
            vec![remus_topology::wire::OrientedEdge::new(edge, true)],
            true,
        )
        .unwrap(),
    )
}

/// A spline hole is followed by the hulls of its control points: clear of
/// the section it leaves a span to the collector, although the box of its
/// control net meets the span's; across the section it refuses the span;
/// around the section it holds it; and the same within a few bands.
#[test]
fn a_spline_hole_is_shown_clear_inside_or_refused() {
    let at = (0.0, 0.0);
    // A circle of radius 0.2: well inside the unit circle, 2e-7 short of
    // it, across it, 2e-7 past it outside, and far away; then one of
    // radius 3 around it.
    for (centre, r, wanted) in [
        ((-0.5, 0.0), 0.2, Some(Kept::AsBefore)),
        ((-0.8 + 2e-7, 0.0), 0.2, Some(Kept::AsBefore)),
        ((-1.0, 0.0), 0.2, None),
        ((-0.8 + 5e-8, 0.0), 0.2, None),
        ((-1.2 - 2e-7, 0.0), 0.2, Some(Kept::AsBefore)),
        ((0.0, 5.0), 0.2, Some(Kept::AsBefore)),
        ((0.3, 0.2), 3.0, Some(Kept::No)),
    ] {
        let (topo, face, section, partner) = plate(|topo| vec![spline_hole(topo, centre, r)], at);
        let certificate = PlaneSectionCertificate::of_face(
            &topo,
            face,
            &section,
            Some(&partner),
            Tolerance::new(),
        )
        .unwrap();
        let splits = [on_circle(at, FRAC_PI_2), on_circle(at, 1.5 * PI)];
        let plan = certificate.plan(&splits);
        // π/2 → 3π/2 passes (−1, 0).
        let kept = plan.keep(0, (FRAC_PI_2, 1.5 * PI), Elsewhere::On);
        if let Some(wanted) = wanted {
            assert_eq!(kept.unwrap(), wanted, "{centre:?} r {r}");
        } else {
            let error = kept.expect_err("a spline hole on the span");
            assert!(error.to_string().contains("no exact contact"), "{error}");
        }
    }
}

/// A span the collector drops for the planar face is dropped whole, and
/// must not run through a hole whose crossings are not split points: it
/// would take the material either side of the hole with it. Dropped for
/// the partner face, it is not this face's to refuse.
#[test]
fn a_span_dropped_for_the_plane_face_must_not_run_through_an_unread_hole() {
    let at = (0.0, 0.0);
    for (centre, crossed) in [((-1.0, 0.0), true), ((-0.5, 0.0), false)] {
        let (topo, face, section, partner) =
            plate(|topo| vec![elliptical_hole(topo, centre, (0.2, 0.1))], at);
        let certificate = PlaneSectionCertificate::of_face(
            &topo,
            face,
            &section,
            Some(&partner),
            Tolerance::new(),
        )
        .unwrap();
        let splits = [on_circle(at, FRAC_PI_2), on_circle(at, 1.5 * PI)];
        let plan = certificate.plan(&splits);
        let span = (FRAC_PI_2, 1.5 * PI);
        assert_eq!(plan.keep(0, span, Elsewhere::Off).unwrap(), Kept::No);
        let dropped = plan.keep(0, span, Elsewhere::OffThePlane);
        if crossed {
            let error = dropped.expect_err("a dropped span through an unread hole");
            assert!(error.to_string().contains("no exact contact"), "{error}");
        } else {
            assert_eq!(dropped.unwrap(), Kept::No);
        }
    }
}

/// A span that ends on no hole is emitted only clear of every hole.
///
/// The unit circle inside a square hole `[−3, 3]²` is crossed nowhere and
/// ends on nothing: before, its spans were the collector's to keep. They
/// lie inside the hole, and are dropped. Beside a hole it only touches
/// (the square `[1, 3] × [−1, 1]`, tangent at (1, 0)) the span through the
/// touch is kept: it stays within the band of the hole there, and outside
/// it everywhere else.
#[test]
fn a_span_that_ends_on_no_hole_is_kept_only_clear_of_every_hole() {
    let at = (0.0, 0.0);
    for (corners, kept, touches) in [
        (
            [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)],
            Kept::No,
            0,
        ),
        (
            [(1.0, -1.0), (3.0, -1.0), (3.0, 1.0), (1.0, 1.0)],
            Kept::AsBefore,
            1,
        ),
        (
            [(1.5, -1.0), (3.0, -1.0), (3.0, 1.0), (1.5, 1.0)],
            Kept::AsBefore,
            0,
        ),
    ] {
        let (topo, face, section, partner) =
            plate(|topo| vec![polygon_wire(topo, &corners, 0.0)], at);
        let certificate = PlaneSectionCertificate::of_face(
            &topo,
            face,
            &section,
            Some(&partner),
            Tolerance::new(),
        )
        .unwrap();
        assert_eq!(certificate.hole_crossings().count(), 0, "{corners:?}");
        assert_eq!(certificate.hole_touches().count(), touches, "{corners:?}");
        let splits = [on_circle(at, 2.0), on_circle(at, 5.0)];
        let plan = certificate.plan(&splits);
        assert_eq!(
            plan.keep(0, (2.0, 5.0), Elsewhere::On).unwrap(),
            kept,
            "{corners:?}"
        );
        // 5 → 2 + τ passes (1, 0).
        assert_eq!(
            plan.keep(1, (5.0, 2.0 + TAU), Elsewhere::On).unwrap(),
            kept,
            "{corners:?}"
        );
    }
}

/// One parameter. A split point stands for a hole's crossing only at the
/// crossing's own parameter: handed in at a parameter 3e-6 further along
/// (its point re-projected, say), a span kept beside it is refused, as the
/// span would run that far past the crossing. Within the band it is the
/// same point.
#[test]
fn a_split_at_another_parameter_than_its_crossing_is_refused() {
    let at = (0.0, 0.0);
    let (topo, face, section, partner) = plate(
        |topo| {
            vec![polygon_wire(
                topo,
                &[(0.5, -0.3), (3.0, -0.3), (3.0, 0.3), (0.5, 0.3)],
                0.0,
            )]
        },
        at,
    );
    let certificate =
        PlaneSectionCertificate::of_face(&topo, face, &section, Some(&partner), Tolerance::new())
            .unwrap();
    let up = 0.3_f64.asin();
    let crossing = on_circle(at, up).1;
    for (slip, refused) in [(0.0, false), (5e-8, false), (3e-6, true), (-3e-6, true)] {
        let splits = [
            (up + slip, crossing),
            on_circle(at, FRAC_PI_2),
            on_circle(at, PI),
            on_circle(at, TAU - up),
        ];
        let plan = certificate.plan(&splits);
        let kept = plan.keep(0, (up + slip, FRAC_PI_2), Elsewhere::On);
        if refused {
            let error = kept.expect_err("a span that ends past its crossing");
            assert!(
                error.to_string().contains("run past the crossing"),
                "slip {slip}: {error}"
            );
        } else {
            assert_eq!(kept.unwrap(), Kept::Certified, "slip {slip}");
        }
        // The span that does not end there is not the crossing's to refuse.
        assert_eq!(
            plan.keep(1, (FRAC_PI_2, PI), Elsewhere::On).unwrap(),
            Kept::AsBefore
        );
    }
}

/// A crossing through a corner of the hole, on an eccentric section.
///
/// The ellipse `(cos t, 0.001 sin t)` and a triangular hole whose corner
/// lies 4e-8 off it at `t = 0.8`, pointing along the tangent: the section
/// enters the hole within 1e-7 of `t = 0.8`, and the corner is the
/// crossing's vertex. Its chart angle is 0.80002786743: 2e-5 further along
/// the section, 3.5e-6 inside the hole. The crossing keeps its own parameter;
/// and since the face splitter still reads a section end's parameter off
/// its vertex, a corner that re-projects past the band is refused as a
/// vertex for the section to end on. The same corner 4e-11 off the section
/// re-projects within the band and is kept.
#[test]
fn a_corner_off_an_eccentric_section_keeps_the_crossings_parameter_or_is_refused() {
    let ellipse = Ellipse3D::new_with_ref(
        Point3::new(0.0, 0.0, 0.0),
        Z,
        1.0,
        0.001,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let t0 = 0.8_f64;
    let on = ellipse.evaluate(t0);
    let tangent = ellipse.tangent(t0).normalize().unwrap();
    let normal = Vec3::new(tangent.y(), -tangent.x(), 0.0);
    // The cylinder of radius 0.001 tilted so that z = 0 cuts it in the
    // ellipse.
    let cos = 0.001_f64;
    let axis = Vec3::new(-(1.0 - cos * cos).sqrt(), 0.0, cos);
    let partner = FaceSurface::Cylinder(
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0) + axis * -4.0, axis, 0.001).unwrap(),
    );
    let section = EdgeCurve::Ellipse(ellipse.clone());
    for (off, refused) in [(4e-8_f64, true), (4e-11, false)] {
        let corner = on + normal * off;
        let far = |side: f64| on + tangent * 7e-4 + normal * (side * 4e-4);
        let mut topo = Topology::new();
        let outer = polygon_wire(
            &mut topo,
            &[(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)],
            0.0,
        );
        let xy = |p: Point3| (p.x(), p.y());
        let hole = polygon_wire(&mut topo, &[xy(corner), xy(far(1.0)), xy(far(-1.0))], 0.0);
        let face = topo.add_face(Face::new(outer, vec![hole], plane_at(0.0)));
        let certificate = PlaneSectionCertificate::of_face(
            &topo,
            face,
            &section,
            Some(&partner),
            Tolerance::new(),
        )
        .unwrap();
        let crossings: Vec<(f64, Point3)> = certificate
            .hole_crossings()
            .map(|(_, c)| (c.t, c.at))
            .collect();
        assert_eq!(crossings.len(), 2, "{crossings:?}");
        let (t, vertex) = crossings
            .iter()
            .copied()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap();
        // At the corner, with the parameter of a root of the section with
        // one of the corner's two sides: a point of the section within the
        // band of the corner, a hair either side of t0.
        assert!((vertex - corner).length() < 1e-15, "{vertex:?}");
        assert!(
            (t - t0).abs() < 2e-7 && (ellipse.evaluate(t) - corner).length() <= 1e-7,
            "off {off}: crossing parameter {t}"
        );
        // What its chart angle would have given.
        let reprojected = ellipse.project(corner);
        let slip = (ellipse.evaluate(reprojected) - ellipse.evaluate(t)).length();
        assert_eq!(slip > 1e-7, refused, "off {off}: slip {slip:e}");

        let mut splits = crossings.clone();
        splits.push((3.0, ellipse.evaluate(3.0)));
        splits.sort_by(|a, b| a.0.total_cmp(&b.0));
        let plan = certificate.plan(&splits);
        // The span from the far split round to the corner.
        let kept = plan.keep(2, (3.0, t + TAU), Elsewhere::On);
        if refused {
            let error = kept.expect_err("a corner that re-projects past the band");
            assert!(
                error.to_string().contains("run past the crossing"),
                "{error}"
            );
        } else {
            assert_eq!(kept.unwrap(), Kept::Certified);
        }
    }
}

/// A round hole the section runs along all the way round (a cutter the
/// size of the bore, on its axis) has no zones and no crossings: it is as
/// if the hole were not read, which is how it was treated before.
#[test]
fn a_hole_the_section_runs_along_is_left_alone() {
    let (topo, face, section, partner) = plate(
        |topo| {
            let bore = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Z, 1.0).unwrap();
            vec![circle_wire(topo, &bore, false)]
        },
        (0.0, 0.0),
    );
    let certificate =
        PlaneSectionCertificate::of_face(&topo, face, &section, Some(&partner), Tolerance::new())
            .unwrap();
    assert_eq!(certificate.hole_crossings().count(), 0);
    assert_eq!(certificate.hole_touches().count(), 0);
    let splits = [on_circle((0.0, 0.0), 1.0), on_circle((0.0, 0.0), 4.0)];
    let plan = certificate.plan(&splits);
    assert_eq!(
        plan.keep(0, (1.0, 4.0), Elsewhere::On).unwrap(),
        Kept::AsBefore
    );
    assert_eq!(
        plan.keep(1, (4.0, 1.0 + TAU), Elsewhere::On).unwrap(),
        Kept::AsBefore
    );
}

/// The splitter's rule, run over the kept spans' ends, must put a vertex on
/// a hole edge exactly where a crossing was certified. A span end that
/// lies on a hole edge but is no crossing of it (handed in here as a split
/// point at the hole's side x = 0.5, 0.1 above the certified crossing) is a
/// vertex the hole would get and the certificate does not know: refused.
#[test]
fn a_vertex_on_a_hole_edge_that_is_no_crossing_is_refused() {
    // The unit circle about (−0.3, 0) crosses the hole's side x = 0.5 at
    // y = ±0.6.
    let at = (-0.3, 0.0);
    let (topo, face, section, partner) = plate(
        |topo| {
            vec![polygon_wire(
                topo,
                &[(0.5, -2.0), (3.0, -2.0), (3.0, 2.0), (0.5, 2.0)],
                0.0,
            )]
        },
        at,
    );
    let certificate =
        PlaneSectionCertificate::of_face(&topo, face, &section, Some(&partner), Tolerance::new())
            .unwrap();
    let up = 0.6_f64.asin();
    let crossings: Vec<Point3> = certificate.hole_crossings().map(|(_, c)| c.at).collect();
    assert_eq!(crossings.len(), 2);
    for p in &crossings {
        assert!(
            (p.x() - 0.5).abs() < 1e-12 && (p.y().abs() - 0.6).abs() < 1e-12,
            "{p:?}"
        );
    }
    let splits = [
        on_circle(at, up),
        on_circle(at, PI),
        on_circle(at, TAU - up),
    ];
    let plan = certificate.plan(&splits);
    plan.certify_hole_splits(&[0, 1]).unwrap();
    // The same spans with a stray end on the hole's side.
    let stray = (0.5 * PI, Point3::new(0.5, 0.7, 0.0));
    let plan = certificate.plan(&[splits[0], stray, splits[1], splits[2]]);
    let error = plan
        .certify_hole_splits(&[0, 1, 2])
        .expect_err("a vertex the certificate did not put on the hole");
    assert!(
        error.to_string().contains("the face splitter would"),
        "{error}"
    );
}
