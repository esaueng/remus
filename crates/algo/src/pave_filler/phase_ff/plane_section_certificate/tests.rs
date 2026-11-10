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
        let mut decided = 0;
        for i in 0..=80 {
            for j in 0..=80 {
                let p = p2(
                    -1.5 + 3.0 * f64::from(i) / 80.0,
                    -1.5 + 3.0 * f64::from(j) / 80.0,
                );
                let side = loop_side(p, &outline, 1e-7);
                if distance(p) > 1e-3 {
                    let inside = crate::builder::classify_2d::point_in_polygon_2d(p, &polygon);
                    assert_eq!(side, Some(inside), "{name}: {p:?}");
                    decided += 1;
                } else if distance(p) < 1e-9 {
                    assert_eq!(side, None, "{name}: on the loop at {p:?}");
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
        plan.keep(0, (up, FRAC_PI_2), true).unwrap(),
        Kept::Certified
    );
    // π/2 → π: ends on no hole.
    assert_eq!(plan.keep(1, (FRAC_PI_2, PI), true).unwrap(), Kept::AsBefore);
    // π → the other crossing: on the face.
    assert_eq!(plan.keep(2, (PI, TAU - up), true).unwrap(), Kept::Certified);
    // Crossing → crossing, through the hole: certified off the face.
    assert_eq!(plan.keep(3, (TAU - up, TAU + up), true).unwrap(), Kept::No);
    // Whatever the collector drops stays dropped.
    for (index, span) in [(0, (up, FRAC_PI_2)), (1, (FRAC_PI_2, PI))] {
        assert_eq!(plan.keep(index, span, false).unwrap(), Kept::No);
    }
    // The splitter's rule agrees with the kept spans' ends.
    plan.certify_hole_splits(&[0, 1, 2]).unwrap();

    // A hole crossing left out of the split points: the span that would
    // run across the hole is refused, the one clear of it is not.
    let plan = certificate.plan(&splits[..3]);
    let error = plan
        .keep(2, (PI, TAU + up), true)
        .expect_err("a span over a crossing that is not a split point");
    assert!(
        error
            .to_string()
            .contains("not among the points it is split at"),
        "{error}"
    );
    assert_eq!(plan.keep(1, (FRAC_PI_2, PI), true).unwrap(), Kept::AsBefore);
}

/// A hole the certificate cannot read (an elliptical one) refuses a span
/// that ends on another hole and comes within reach of it, and no other:
/// not a span that stays clear of it, and not a span that ends on no hole.
#[test]
fn an_unread_hole_refuses_only_the_spans_that_reach_it() {
    let unread = |at: (f64, f64)| {
        move |topo: &mut Topology| {
            let ellipse = Ellipse3D::new(Point3::new(at.0, at.1, 0.0), Z, 0.2, 0.1).unwrap();
            let seam = super::super::plane_hole_tests::vertex(topo, ellipse.evaluate(0.0));
            let mut edge = super::super::Edge::with_tolerance(
                seam,
                seam,
                EdgeCurve::Ellipse(ellipse),
                Some(1e-7),
            );
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
    };
    let up = 0.3_f64.asin();
    let at = (0.0, 0.0);
    for (elliptical_at, refused) in [((-1.0, 0.0), true), ((0.0, 5.0), false)] {
        let (topo, face, section, partner) = plate(
            |topo| {
                vec![
                    polygon_wire(
                        topo,
                        &[(0.5, -0.3), (3.0, -0.3), (3.0, 0.3), (0.5, 0.3)],
                        0.0,
                    ),
                    unread(elliptical_at)(topo),
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
        // Crossing → π/2 stays in x > 0, clear of an ellipse at (−1, 0).
        assert_eq!(
            plan.keep(0, (up, FRAC_PI_2), true).unwrap(),
            Kept::Certified
        );
        // π/2 → 3π/2 passes (−1, 0) but ends on no hole.
        assert_eq!(
            plan.keep(1, (FRAC_PI_2, 1.5 * PI), true).unwrap(),
            Kept::AsBefore
        );
        // With the split at π/2 gone, the span from the crossing runs on
        // to 3π/2, through (−1, 0).
        let plan = certificate.plan(&[splits[0], splits[2], splits[3]]);
        let long = plan.keep(0, (up, 1.5 * PI), true);
        if refused {
            let error = long.expect_err("a span that reaches an unread hole");
            assert!(error.to_string().contains("no exact contact"), "{error}");
        } else {
            assert_eq!(long.unwrap(), Kept::Certified);
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
    assert_eq!(plan.keep(0, (1.0, 4.0), true).unwrap(), Kept::AsBefore);
    assert_eq!(
        plan.keep(1, (4.0, 1.0 + TAU), true).unwrap(),
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
