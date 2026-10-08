//! Independent-oracle tests for fill-images section helpers: cap-disc and
//! equal-radius recognition, closed-section containment and coincidence, the
//! curved-face segment classifier, and the winding-loop ordering helpers.
//!
//! Expected answers come from the fixtures' own construction (which circle
//! bounds which face, which axial band a generator segment occupies), not
//! from the helpers under test. The winding-loop helpers are also checked bit
//! for bit against the `rem_euclid` / `min_by` code they replace.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use proptest::prelude::*;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::CylindricalSurface;

use remus_topology::wire::WireId;

use super::*;

const Z: remus_math::vec::Vec3 = remus_math::vec::Vec3::new(0.0, 0.0, 1.0);

fn closed_wire(topo: &mut Topology, curve: EdgeCurve, seam: Point3, domain: (f64, f64)) -> WireId {
    let v = topo.add_vertex(Vertex::new(seam, 1e-7));
    let mut edge = Edge::with_tolerance(v, v, curve, Some(1e-7));
    edge.set_trim(Some(domain));
    let e = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap())
}

fn circle_wire(topo: &mut Topology, circle: &Circle3D) -> WireId {
    closed_wire(
        topo,
        EdgeCurve::Circle(circle.clone()),
        circle.evaluate(0.0),
        (0.0, TAU),
    )
}

fn z_plane(z: f64) -> FaceSurface {
    FaceSurface::Plane { normal: Z, d: z }
}

/// A disc cap of `radius` about `center` in its z plane, optionally holed.
fn disc(topo: &mut Topology, center: Point3, radius: f64, hole: Option<f64>) -> FaceId {
    let rim = Circle3D::new(center, Z, radius).unwrap();
    let outer = circle_wire(topo, &rim);
    let inner = hole
        .map(|r| {
            let c = Circle3D::new(center, Z, r).unwrap();
            vec![circle_wire(topo, &c)]
        })
        .unwrap_or_default();
    topo.add_face(Face::new(outer, inner, z_plane(center.z())))
}

fn square(topo: &mut Topology, half: f64) -> FaceId {
    let corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
        .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)));
    let edges = (0..4)
        .map(|i| {
            let e = topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line));
            OrientedEdge::new(e, true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(wire, vec![], z_plane(0.0)))
}

fn cylinder_face(
    topo: &mut Topology,
    origin: Point3,
    axis: remus_math::vec::Vec3,
    r: f64,
) -> FaceId {
    let surface = FaceSurface::Cylinder(CylindricalSurface::new(origin, axis, r).unwrap());
    let rim = Circle3D::new(origin, axis, r).unwrap();
    let wire = circle_wire(topo, &rim);
    topo.add_face(Face::new(wire, vec![], surface))
}

#[test]
fn equal_radius_cylinder_pairs_are_recognised_by_radius_alone() {
    let mut topo = Topology::new();
    let a = cylinder_face(&mut topo, Point3::new(0.0, 0.0, 0.0), Z, 1.5);
    let b = cylinder_face(
        &mut topo,
        Point3::new(4.0, -1.0, 2.0),
        remus_math::vec::Vec3::new(1.0, 0.0, 0.0),
        1.5,
    );
    let c = cylinder_face(&mut topo, Point3::new(0.0, 0.0, 0.0), Z, 1.8);
    let plane = square(&mut topo, 1.0);
    assert!(equal_radius_cylinder_pair(&topo, a, b));
    assert!(equal_radius_cylinder_pair(&topo, b, a));
    assert!(!equal_radius_cylinder_pair(&topo, a, c));
    assert!(!equal_radius_cylinder_pair(&topo, a, plane));
    assert!(!equal_radius_cylinder_pair(&topo, plane, a));
}

#[test]
fn a_cap_disc_is_a_plane_bounded_by_one_closed_circle() {
    let mut topo = Topology::new();
    let center = Point3::new(1.0, -2.0, 0.5);
    let cap = disc(&mut topo, center, 2.5, None);
    let circle = cap_disc_circle(&topo, cap).expect("a disc cap");
    assert!((circle.center() - center).length() < 1e-12);
    assert!((circle.radius() - 2.5).abs() < 1e-12);
    let holed = disc(&mut topo, center, 2.5, Some(1.0));
    assert!(
        cap_disc_circle(&topo, holed).is_some(),
        "the outer wire decides"
    );
    let sq = square(&mut topo, 1.0);
    assert!(cap_disc_circle(&topo, sq).is_none());
    let wall = cylinder_face(&mut topo, center, Z, 2.5);
    assert!(cap_disc_circle(&topo, wall).is_none());
}

#[test]
fn a_closed_section_is_inside_a_face_only_when_it_stays_within_its_extent() {
    let mut topo = Topology::new();
    let center = Point3::new(1.0, 1.0, 0.0);
    let cap = disc(&mut topo, center, 2.0, None);
    let tol = 1e-7;
    let circle = |c: Point3, n: remus_math::vec::Vec3, r: f64| {
        EdgeCurve::Circle(Circle3D::new(c, n, r).unwrap())
    };
    let inside = [
        circle(center, Z, 1.0),
        circle(center, Z, 2.0),
        circle(center, -Z, 1.99),
        circle(center + remus_math::vec::Vec3::new(0.5, -0.5, 0.0), Z, 1.0),
        EdgeCurve::Ellipse(Ellipse3D::new(center, Z, 1.8, 0.7).unwrap()),
    ];
    for curve in &inside {
        assert!(
            circle_inside_face(&topo, cap, curve, tol).unwrap(),
            "{curve:?} fits the cap"
        );
    }
    let outside = [
        circle(center, Z, 2.5),
        circle(center + remus_math::vec::Vec3::new(1.8, 0.0, 0.0), Z, 0.5),
        circle(center, remus_math::vec::Vec3::new(1.0, 0.0, 0.0), 1.0),
        circle(center + remus_math::vec::Vec3::new(0.0, 0.0, 0.3), Z, 1.0),
        EdgeCurve::Ellipse(Ellipse3D::new(center, Z, 2.6, 0.7).unwrap()),
        EdgeCurve::Line,
    ];
    for curve in &outside {
        assert!(
            !circle_inside_face(&topo, cap, curve, tol).unwrap(),
            "{curve:?} overhangs"
        );
    }
}

#[test]
fn a_closed_section_coincides_with_a_boundary_only_when_it_is_that_ring() {
    let mut topo = Topology::new();
    let center = Point3::new(-0.4, 0.3, 2.0);
    let cap = disc(&mut topo, center, 2.0, Some(0.75));
    let tol = 1e-7;
    let circle = |c: Point3, n: remus_math::vec::Vec3, r: f64| {
        EdgeCurve::Circle(Circle3D::new(c, n, r).unwrap())
    };
    for curve in [
        circle(center, Z, 2.0),
        circle(center, -Z, 2.0),
        // The hole's rim is a boundary too.
        circle(center, Z, 0.75),
    ] {
        assert!(
            closed_curve_coincides_with_boundary(&topo, cap, &curve, tol),
            "{curve:?}"
        );
    }
    for curve in [
        circle(center, Z, 1.2),
        circle(center + remus_math::vec::Vec3::new(1e-3, 0.0, 0.0), Z, 2.0),
        circle(center, remus_math::vec::Vec3::new(0.0, 0.1, 1.0), 2.0),
        EdgeCurve::Line,
    ] {
        assert!(
            !closed_curve_coincides_with_boundary(&topo, cap, &curve, tol),
            "{curve:?}"
        );
    }

    // An elliptical cap matches its own ellipse in either direction.
    let mut topo = Topology::new();
    let ellipse = Ellipse3D::new(center, Z, 3.0, 1.0).unwrap();
    let wire = closed_wire(
        &mut topo,
        EdgeCurve::Ellipse(ellipse.clone()),
        ellipse.evaluate(0.0),
        (0.0, TAU),
    );
    let cap = topo.add_face(Face::new(wire, vec![], z_plane(center.z())));
    assert!(closed_curve_coincides_with_boundary(
        &topo,
        cap,
        &EdgeCurve::Ellipse(Ellipse3D::new(center, -Z, 3.0, 1.0).unwrap()),
        tol
    ));
    assert!(!closed_curve_coincides_with_boundary(
        &topo,
        cap,
        &EdgeCurve::Ellipse(Ellipse3D::new(center, Z, 3.0, 1.1).unwrap()),
        tol
    ));
}

/// The two rims of a cylinder wall `z ∈ [z0, z1]` of radius `r` about the z
/// axis through `origin`, as the classifier's boundary arcs.
fn wall_rims(origin: Point3, r: f64, z0: f64, z1: f64) -> Vec<Option<BoundaryArc>> {
    [z0, z1]
        .into_iter()
        .map(|z| {
            let rim =
                Circle3D::new(origin + remus_math::vec::Vec3::new(0.0, 0.0, z), Z, r).unwrap();
            let seam = rim.evaluate(0.3);
            Some((
                EdgeCurve::Circle(rim),
                seam,
                seam,
                true,
                Some((0.3, 0.3 + TAU)),
            ))
        })
        .collect()
}

#[test]
fn a_generator_segment_is_in_a_wall_exactly_when_it_lies_between_the_rims() {
    let tol = 1e-7;
    // Radii on both sides of 1: the classifier's reach past each rim must
    // not depend on the radius being large or small.
    for r in [0.1, 5.0] {
        let origin = Point3::new(0.2, -0.6, 1.0);
        let (z0, z1) = (0.0, 2.0);
        let arcs = wall_rims(origin, r, z0, z1);
        let theta: f64 = 0.9;
        let on = |z: f64| origin + remus_math::vec::Vec3::new(r * theta.cos(), r * theta.sin(), z);
        // (from z, to z, lies in the wall band)
        for (a, b, inside) in [
            (0.9, 1.1, true),
            (1.1, 0.9, true),
            (0.2, 1.8, true),
            (z0, 1.0, true),
            (1.0, z1, true),
            (z0, z1, true),
            (2.5, 3.5, false),
            (-1.5, -0.5, false),
            (1.5, 2.5, false),
            (-0.5, 0.5, false),
            (-0.5, 2.5, false),
        ] {
            assert_eq!(
                segment_between_boundary_arcs(&arcs, on(a), on(b), tol),
                inside,
                "r = {r}: generator z {a} -> {b} against the band [{z0}, {z1}]"
            );
        }
        // A degenerate segment classifies as nothing.
        assert!(!segment_between_boundary_arcs(&arcs, on(1.0), on(1.0), tol));
    }
}

// ── Winding-loop ordering: fmod-free wrap and nearest-sample fold ─────────

/// The wrap `loops_strictly_ordered` and `compute_winding_loop_cuts` used
/// before the fmod-free path, kept verbatim as the bit-identity oracle.
fn wrap_reference(d: f64) -> f64 {
    (d + PI).rem_euclid(TAU) - PI
}

/// The nearest-sample lookup as it was: `min_by`, both keys per comparison.
fn nearest_v_reference(s: &[(f64, f64)], u: f64) -> Option<f64> {
    s.iter()
        .min_by(|a, b| {
            wrap_reference(a.0 - u)
                .abs()
                .partial_cmp(&wrap_reference(b.0 - u).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|&(_, v)| v)
}

fn loops_strictly_ordered_reference(lo: &[(f64, f64)], hi: &[(f64, f64)], gap: f64) -> bool {
    lo.iter()
        .all(|&(u, v)| nearest_v_reference(hi, u).is_some_and(|h| h - v > gap))
        && hi
            .iter()
            .all(|&(u, v)| nearest_v_reference(lo, u).is_some_and(|l| v - l > gap))
}

fn bits(v: Option<f64>) -> Option<u64> {
    v.map(f64::to_bits)
}

#[test]
fn rem_tau_fast_takes_exactly_its_three_ranges_and_matches_rem_euclid() {
    let two_tau = 2.0 * TAU;
    // (x, whether the fmod-free path may answer it)
    let table = [
        (-two_tau.next_up(), false),
        (-two_tau, false),
        (-TAU.next_up(), false),
        (-TAU, false),
        (-TAU.next_down(), true),
        (-PI, true),
        (-5e-324, true),
        (0.0_f64.next_down(), true),
        (-0.0, true),
        (0.0, true),
        (5e-324, true),
        (PI, true),
        (TAU.next_down(), true),
        (TAU, true),
        (TAU.next_up(), true),
        (3.0 * PI, true),
        (10.0, true),
        (two_tau.next_down(), true),
        (two_tau, false),
        (two_tau.next_up(), false),
        (3.0 * TAU, false),
        (1e300, false),
        (-1e300, false),
        (f64::INFINITY, false),
        (f64::NEG_INFINITY, false),
        (f64::NAN, false),
    ];
    for (x, fast) in table {
        let got = rem_tau_fast(x);
        assert_eq!(got.is_some(), fast, "x = {x:e}: fast path taken");
        if let Some(r) = got {
            let want = x.rem_euclid(TAU);
            assert_eq!(r.to_bits(), want.to_bits(), "x = {x:e}: {r:e} vs {want:e}");
        }
    }
    // Why -TAU stays out: the exact answer there is a NEGATIVE zero.
    assert_eq!((-TAU).rem_euclid(TAU).to_bits(), (-0.0_f64).to_bits());
    assert_eq!(two_tau.rem_euclid(TAU).to_bits(), 0.0_f64.to_bits());
}

#[test]
fn wrap_pi_exact_matches_the_rem_euclid_wrap_at_every_branch_point() {
    // `PI + PI == TAU` and `3·PI + PI == 2·TAU` exactly, so these reach the
    // branch points through `d` itself.
    let mut ds = vec![-0.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
    for base in [
        -3.0 * PI,
        -TAU - PI,
        -TAU,
        -PI,
        0.0,
        PI,
        TAU,
        3.0 * PI,
        2.0 * TAU,
        3.0 * TAU,
        -3.0 * TAU,
    ] {
        ds.extend([base.next_down(), base, base.next_up()]);
    }
    for d in ds {
        assert_eq!(
            wrap_pi_exact(d).to_bits(),
            wrap_reference(d).to_bits(),
            "d = {d:e}"
        );
    }
}

#[test]
fn nearest_v_keeps_the_first_of_equally_near_samples() {
    type Case<'a> = (&'a [(f64, f64)], f64, Option<f64>);
    let cases: [Case; 9] = [
        (&[], 1.0, None),
        // Repeated u: the first one.
        (&[(1.0, 10.0), (1.0, 20.0)], 1.0, Some(10.0)),
        // 0.5 either side of u = 1 (both wraps exact): the first one.
        (&[(0.5, 1.0), (1.5, 2.0)], 1.0, Some(1.0)),
        (&[(1.5, 2.0), (0.5, 1.0)], 1.0, Some(2.0)),
        // A strictly nearer later sample wins.
        (&[(0.5, 1.0), (1.25, 2.0), (0.75, 3.0)], 1.0, Some(2.0)),
        // Exactly half a turn either way: a tie.
        (&[(PI, 1.0), (-PI, 2.0)], 0.0, Some(1.0)),
        // A NaN key first never gives way; a later NaN is passed over.
        (&[(f64::NAN, 1.0), (0.0, 2.0)], 0.0, Some(1.0)),
        (&[(1.0, 1.0), (f64::NAN, 2.0), (0.125, 3.0)], 0.0, Some(3.0)),
        // Round the seam: 6.25 is 0.033 short of a turn from 0.
        (&[(1.0, 1.0), (6.25, 2.0)], 0.0, Some(2.0)),
    ];
    for (samples, u, want) in cases {
        assert_eq!(
            bits(nearest_v(samples, u)),
            bits(want),
            "{samples:?} at {u}"
        );
        assert_eq!(
            bits(nearest_v(samples, u)),
            bits(nearest_v_reference(samples, u)),
            "{samples:?} at {u}"
        );
    }
    // The seam pair u = 0 / u = TAU in both orders.
    for u in [0.0, TAU, 1e-9, TAU - 1e-9] {
        for s in [[(0.0, 1.0), (TAU, 2.0)], [(TAU, 2.0), (0.0, 1.0)]] {
            assert_eq!(bits(nearest_v(&s, u)), bits(nearest_v_reference(&s, u)));
        }
    }
}

#[test]
fn loops_strictly_ordered_needs_more_than_gap_in_both_directions() {
    let flat = |v: f64| [(0.0, v), (2.0, v), (4.0, v)];
    assert!(loops_strictly_ordered(&flat(0.0), &flat(1.0), 0.5));
    assert!(!loops_strictly_ordered(&flat(1.0), &flat(0.0), 0.5));
    assert!(!loops_strictly_ordered(&flat(0.0), &flat(1.0), 1.0));
    assert!(!loops_strictly_ordered(&flat(0.0), &[], 0.5));

    // Only `hi`'s third sample sits exactly `gap` above its nearest `lo`
    // sample: the lo -> hi scan passes and the hi -> lo scan decides.
    let lo = [(0.0, 0.0), (1.0, 0.5)];
    let hi = [(0.0, 2.0), (1.0, 2.0), (2.0, 1.5)];
    assert!(!loops_strictly_ordered(&lo, &hi, 1.0));
    assert!(loops_strictly_ordered(&lo, &hi, 0.75));
    // And the mirror image: only lo -> hi meets exactly `gap`.
    let lo = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.5)];
    let hi = [(0.0, 2.0), (1.0, 1.5)];
    assert!(!loops_strictly_ordered(&lo, &hi, 1.0));
    assert!(loops_strictly_ordered(&lo, &hi, 0.75));
}

/// Angles from a pool that forces ties (the seam from both sides, half turns,
/// repeats, dyadic steps whose wraps are exact) mixed with arbitrary ones.
fn tie_angle() -> impl Strategy<Value = f64> {
    prop_oneof![
        prop::sample::select(vec![0.0, -0.0, TAU, PI, -PI, 0.5, 1.0, 1.5, 3.0, 6.0]),
        -TAU..2.0 * TAU,
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]
    #[test]
    fn wrap_pi_exact_is_the_rem_euclid_wrap(d in -3.0 * TAU..3.0 * TAU) {
        prop_assert_eq!(wrap_pi_exact(d).to_bits(), wrap_reference(d).to_bits());
    }

    #[test]
    fn nearest_v_is_the_min_by_choice(
        samples in prop::collection::vec((tie_angle(), -4.0f64..4.0), 0..24),
        u in tie_angle(),
    ) {
        prop_assert_eq!(bits(nearest_v(&samples, u)), bits(nearest_v_reference(&samples, u)));
    }

    #[test]
    fn loops_strictly_ordered_is_the_min_by_verdict(
        lo in prop::collection::vec((tie_angle(), -1.0f64..1.0), 0..16),
        hi in prop::collection::vec((tie_angle(), -1.0f64..3.0), 0..16),
        gap in prop::sample::select(vec![0.0, 1e-5, 0.5, 1.0]),
    ) {
        prop_assert_eq!(
            loops_strictly_ordered(&lo, &hi, gap),
            loops_strictly_ordered_reference(&lo, &hi, gap)
        );
    }
}
