//! Independent-oracle tests for fill-images section helpers: cap-disc and
//! equal-radius recognition, closed-section containment and coincidence, the
//! curved-face segment classifier, and the winding-loop ordering helpers.
//!
//! Expected answers come from the fixtures' own construction (which circle
//! bounds which face, which axial band a generator segment occupies), not
//! from the helpers under test. The winding-loop helpers are also checked bit
//! for bit against the `rem_euclid` / `min_by` code they replace, and the
//! presplit's reach-box gate against the ungated presplit.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use proptest::prelude::*;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
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

// ── Winding-loop cuts ─────────────────────────────────────────────────────

/// The exact rational quadratic circle of radius `r` about the z axis at
/// height `z`: nine control points from world angle `phi`, corner weights
/// `√2/2`, and the last point the first one bit for bit.
fn nurbs_circle(r: f64, z: f64, phi: f64) -> NurbsCurve {
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_4, SQRT_2};
    let mut points: Vec<Point3> = (0..9)
        .map(|k| {
            let theta = phi + f64::from(k) * FRAC_PI_4;
            let s = if k % 2 == 0 { r } else { r * SQRT_2 };
            Point3::new(s * theta.cos(), s * theta.sin(), z)
        })
        .collect();
    points[8] = points[0];
    let weights = (0..9)
        .map(|k| if k % 2 == 0 { 1.0 } else { FRAC_1_SQRT_2 })
        .collect();
    let knots = vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ];
    NurbsCurve::new(2, knots, points, weights).unwrap()
}

/// A z-axis cylinder wall of radius `r` and height 4 whose seam generator
/// stands at world angle `seam_phi`, each loop of `loops` (height, start
/// angle) sectioning it against a plane face, and the wall's surface.
fn winding_fixture(
    r: f64,
    seam_phi: f64,
    loops: &[(f64, f64)],
) -> (Topology, GfaArena, CylindricalSurface) {
    let mut topo = Topology::new();
    let cyl = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z, r).unwrap();
    let foot = Point3::new(r * seam_phi.cos(), r * seam_phi.sin(), 0.0);
    let head = foot + remus_math::vec::Vec3::new(0.0, 0.0, 4.0);
    let (v0, v1) = (
        topo.add_vertex(Vertex::new(foot, 1e-7)),
        topo.add_vertex(Vertex::new(head, 1e-7)),
    );
    let rim = |topo: &mut Topology, v, z: f64| {
        let c = Circle3D::new(Point3::new(0.0, 0.0, z), Z, r).unwrap();
        topo.add_edge(Edge::with_tolerance(v, v, EdgeCurve::Circle(c), Some(1e-7)))
    };
    let bottom = rim(&mut topo, v0, 0.0);
    let top = rim(&mut topo, v1, 4.0);
    let seam = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(bottom, true),
            OrientedEdge::new(seam, true),
            OrientedEdge::new(top, false),
            OrientedEdge::new(seam, false),
        ],
        true,
    )
    .unwrap();
    let wire = topo.add_wire(wire);
    let wall = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cyl.clone())));
    let plane = square(&mut topo, 1.0);

    let mut arena = GfaArena::new();
    for &(z, phi) in loops {
        let curve = nurbs_circle(r, z, phi);
        arena.curves.push(crate::ds::IntersectionCurveDS {
            bbox: curve.aabb(),
            curve: EdgeCurve::NurbsCurve(curve),
            face_a: wall,
            face_b: plane,
            pave_blocks: vec![],
            t_range: (0.0, 1.0),
        });
    }
    (topo, arena, cyl)
}

#[test]
fn winding_loops_are_cut_on_the_seam_and_two_more_meridians() {
    let (r, seam_phi) = (2.0, 0.4);
    // The lower loop starts ON the seam generator (its first and last samples
    // project exactly onto the seam meridian, so that meridian has no strict
    // bracket and takes the on-sample fallback); the upper one starts a
    // radian past it and brackets all three meridians.
    let (topo, arena, cyl) =
        winding_fixture(r, seam_phi, &[(3.0, seam_phi + 1.0), (1.0, seam_phi)]);
    let cuts = compute_winding_loop_cuts(&topo, &arena, Tolerance::new());

    let seam_u = cyl
        .project_point(Point3::new(r * seam_phi.cos(), r * seam_phi.sin(), 0.0))
        .0;
    let mut want = Vec::new();
    // Lower loop first: the loops are taken in ascending v.
    for z in [1.0, 3.0] {
        for k in 0..3 {
            want.push(cyl.evaluate(seam_u + f64::from(k) * TAU / 3.0, z));
        }
    }
    assert_eq!(cuts.len(), want.len(), "{cuts:?}");
    for (got, want) in cuts.iter().zip(&want) {
        assert!((*got - *want).length() < 1e-9, "{got:?} vs {want:?}");
    }
}

#[test]
fn winding_loops_closer_than_the_gap_are_not_cut() {
    let tol = Tolerance::new();
    let gap = 100.0 * tol.linear;
    // Parallel but only half the gap apart: no band between them.
    let (topo, arena, _) = winding_fixture(2.0, 0.4, &[(1.0, 0.4), (1.0 + 0.5 * gap, 1.4)]);
    assert!(compute_winding_loop_cuts(&topo, &arena, tol).is_empty());
    // A lone separator is left to the chain-band path.
    let (topo, arena, _) = winding_fixture(2.0, 0.4, &[(1.0, 0.4)]);
    assert!(compute_winding_loop_cuts(&topo, &arena, tol).is_empty());
    // Twice the gap apart, they are cut.
    let (topo, arena, _) = winding_fixture(2.0, 0.4, &[(1.0, 0.4), (1.0 + 2.0 * gap, 1.4)]);
    assert_eq!(compute_winding_loop_cuts(&topo, &arena, tol).len(), 6);
}

// ── Winding-loop presplit: the reach-box gate ─────────────────────────────

/// `presplit_closed_winding_loops` as it was before the reach-box gate,
/// kept verbatim (every cut projected) as the bit-identity oracle.
fn presplit_closed_winding_loops_reference(
    sections: &[SectionEdge],
    cuts: &[Point3],
    surface: &FaceSurface,
    rank: Rank,
    wire_pts: &[Point3],
    tol: f64,
) -> Result<Vec<SectionEdge>, AlgoError> {
    use remus_math::traits::ParametricCurve;

    let weld = tol * 100.0;
    let mut out = Vec::with_capacity(sections.len() + cuts.len());
    for s in sections {
        let EdgeCurve::NurbsCurve(nurbs) = &s.curve_3d else {
            out.push(s.clone());
            continue;
        };
        if (s.start - s.end).length() > weld {
            out.push(s.clone());
            continue;
        }
        let (d0, d1) = ParametricCurve::domain(nurbs);
        let margin = (d1 - d0) * 1e-6;
        let mut ts: Vec<f64> = cuts
            .iter()
            .filter_map(|p| {
                let hit =
                    remus_math::nurbs::projection::project_point_to_curve(nurbs, *p, 1e-9).ok()?;
                (hit.distance <= weld && hit.parameter > d0 + margin && hit.parameter < d1 - margin)
                    .then_some(hit.parameter)
            })
            .collect();
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        ts.dedup_by(|a, b| (*a - *b).abs() < margin);
        if ts.len() < 2 {
            out.push(s.clone());
            continue;
        }
        let bounds: Vec<f64> = std::iter::once(d0).chain(ts).chain([d1]).collect();
        let Some(pieces) = split_nurbs_at(nurbs, &bounds) else {
            out.push(s.clone());
            continue;
        };
        for sub in pieces {
            let (a0, a1) = ParametricCurve::domain(&sub);
            let (start, end) = (
                ParametricCurve::evaluate(&sub, a0),
                ParametricCurve::evaluate(&sub, a1),
            );
            let curve_3d = EdgeCurve::NurbsCurve(sub);
            let pcurve = crate::builder::pcurve_compute::compute_pcurve_on_surface_in_domain(
                &curve_3d,
                start,
                end,
                (a0, a1),
                surface,
                wire_pts,
                None,
            )?;
            let mut piece = s.clone();
            piece.curve_3d = curve_3d;
            piece.trim = Some((a0, a1));
            piece.start = start;
            piece.end = end;
            match rank {
                Rank::A => piece.pcurve_a = pcurve,
                Rank::B => piece.pcurve_b = pcurve,
            }
            piece.start_uv_a = None;
            piece.end_uv_a = None;
            piece.start_uv_b = None;
            piece.end_uv_b = None;
            piece.pave_block_id = None;
            out.push(piece);
        }
    }
    Ok(out)
}

/// A closed clamped NURBS loop of `degree` round `center`: one control
/// point per weight, spread over a wobbly, warped ring of `radius`, the
/// last one the first again so the curve closes exactly.
fn closed_loop(
    degree: usize,
    center: Point3,
    radius: f64,
    weights: &[f64],
    wobble: &[f64],
) -> NurbsCurve {
    let n = weights.len();
    let ring = n - 1;
    #[allow(clippy::cast_precision_loss)]
    let mut points: Vec<Point3> = (0..n)
        .map(|i| {
            let theta = TAU * i as f64 / ring as f64;
            let s = radius * (1.0 + wobble[i % wobble.len()]);
            center
                + remus_math::vec::Vec3::new(
                    s * theta.cos(),
                    s * theta.sin(),
                    0.3 * radius * (2.0 * theta).sin(),
                )
        })
        .collect();
    points[ring] = points[0];
    let inner = n - degree - 1;
    #[allow(clippy::cast_precision_loss)]
    let knots = std::iter::repeat_n(0.0, degree + 1)
        .chain((1..=inner).map(|k| k as f64 / (inner + 1) as f64))
        .chain(std::iter::repeat_n(1.0, degree + 1))
        .collect();
    NurbsCurve::new(degree, knots, points, weights.to_vec()).unwrap()
}

fn closed_section(curve: NurbsCurve) -> SectionEdge {
    use remus_math::curves2d::{Curve2D, Line2D};
    let line = || {
        Curve2D::Line(
            Line2D::new(
                remus_math::vec::Point2::new(0.0, 0.0),
                remus_math::vec::Vec2::new(1.0, 0.0),
            )
            .unwrap(),
        )
    };
    let (start, end) = (curve.evaluate(0.0), curve.evaluate(1.0));
    SectionEdge {
        curve_3d: EdgeCurve::NurbsCurve(curve),
        trim: Some((0.0, 1.0)),
        pcurve_a: line(),
        pcurve_b: line(),
        start,
        end,
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: Some(7),
    }
}

/// Run the gated presplit and the ungated reference on the same input and
/// require the same sections, every float compared through its `Debug`
/// form (shortest round-trip, so equal text is equal bits).
fn assert_presplit_matches_reference(sections: &[SectionEdge], cuts: &[Point3], tol: f64) {
    let surface = FaceSurface::Plane { normal: Z, d: 0.0 };
    let wire_pts = [
        Point3::new(-1.0, -1.0, 0.0),
        Point3::new(1.0, -1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    for rank in [Rank::A, Rank::B] {
        let got = presplit_closed_winding_loops(sections, cuts, &surface, rank, &wire_pts, tol);
        let want =
            presplit_closed_winding_loops_reference(sections, cuts, &surface, rank, &wire_pts, tol);
        assert_eq!(format!("{got:?}"), format!("{want:?}"));
    }
}

#[test]
fn weld_reach_box_is_the_control_box_grown_by_two_welds_and_rounding_slack() {
    let weld = 1e-5;
    for (degree, center) in [
        (2, Point3::new(0.0, 0.0, 0.0)),
        (3, Point3::new(-1e6, 2.5, 3e5)),
        (5, Point3::new(4.0, -7.0, 1e-3)),
    ] {
        let curve = closed_loop(
            degree,
            center,
            2.0,
            &[1.0, 0.2, 5.0, 0.7, 1.3, 2.0, 0.4, 1.0],
            &[0.0, 0.1, -0.15],
        );
        let pts = curve.control_points();
        let lo = |f: fn(&Point3) -> f64| pts.iter().map(f).fold(f64::INFINITY, f64::min);
        let hi = |f: fn(&Point3) -> f64| pts.iter().map(f).fold(f64::NEG_INFINITY, f64::max);
        let (min, max) = (
            [lo(|p| p.x()), lo(|p| p.y()), lo(|p| p.z())],
            [hi(|p| p.x()), hi(|p| p.y()), hi(|p| p.z())],
        );
        let scale = min.iter().chain(&max).fold(0.0_f64, |m, c| m.max(c.abs()));
        #[allow(clippy::cast_precision_loss)]
        let grow = 2.0 * weld + 1e-9 * (1.0 + scale) * (degree + 1) as f64;
        // Never tighter than two welds plus 1e-9 per unit of coordinate.
        assert!(grow >= 2.0 * weld + 1e-9 * (1.0 + scale));
        let b = weld_reach_box(&curve, weld).expect("a validated curve is certified");
        let got = [
            b.min.x(),
            b.min.y(),
            b.min.z(),
            b.max.x(),
            b.max.y(),
            b.max.z(),
        ];
        let want = [
            min[0] - grow,
            min[1] - grow,
            min[2] - grow,
            max[0] + grow,
            max[1] + grow,
            max[2] + grow,
        ];
        for (g, w) in got.iter().zip(&want) {
            assert_eq!(
                g.to_bits(),
                w.to_bits(),
                "degree {degree}: {got:?} vs {want:?}"
            );
        }
    }
}

#[test]
fn weld_reach_box_refuses_weight_ratios_past_1e100() {
    let at = |w: f64| {
        closed_loop(
            2,
            Point3::new(0.0, 0.0, 0.0),
            1.0,
            &[1.0, w, 1.0, 1.0, 1.0],
            &[0.0],
        )
    };
    assert!(weld_reach_box(&at(1e-100), 1e-5).is_some());
    assert!(weld_reach_box(&at(1e-100_f64.next_down()), 1e-5).is_none());
    assert!(weld_reach_box(&at(1e-300), 1e-5).is_none());
}

#[test]
fn far_cuts_are_skipped_and_the_split_is_unchanged() {
    let tol = 1e-7;
    let weld = 100.0 * tol;
    let weights = [1.0, 0.6, 1.4, 1.0, 0.8, 1.2, 1.0];
    let a = closed_loop(3, Point3::new(-3.0, 0.0, 0.0), 1.0, &weights, &[0.0, 0.05]);
    let b = closed_loop(3, Point3::new(3.0, 0.0, 0.0), 1.0, &weights, &[0.0, 0.05]);
    let mut cuts: Vec<Point3> = [0.2, 0.5, 0.8].iter().map(|&t| a.evaluate(t)).collect();
    cuts.extend([0.3, 0.6, 0.9].iter().map(|&t| b.evaluate(t)));
    // Exactly one weld out from `a` along x at its outermost control point:
    // inside the reach box, so projected (and kept or not as before).
    let reach = weld_reach_box(&a, weld).unwrap();
    let rim = a
        .control_points()
        .iter()
        .copied()
        .fold(
            a.control_points()[0],
            |m, p| if p.x() < m.x() { p } else { m },
        );
    cuts.push(rim - remus_math::vec::Vec3::new(weld, 0.0, 0.0));
    // Just inside and just outside a reach-box corner the curve never nears.
    let step = remus_math::vec::Vec3::new(1e-12, 1e-12, 1e-12);
    cuts.extend([reach.min + step, reach.min - step, reach.max + step]);

    let sections = [closed_section(a), closed_section(b)];
    let out = presplit_closed_winding_loops(
        &sections,
        &cuts,
        &FaceSurface::Plane { normal: Z, d: 0.0 },
        Rank::A,
        &[],
        tol,
    )
    .unwrap();
    // Each loop opens at its own three cuts into four arcs.
    assert_eq!(out.len(), 8, "{out:?}");
    assert_presplit_matches_reference(&sections, &cuts, tol);
}

/// Strategy: 2..=4 closed loops (degree 2..=5, weights 0.2..5) laid out
/// apart, overlapping, or near 1e6, and cuts on them, within a weld of
/// them, and hugging their reach boxes from both sides.
fn presplit_case() -> impl Strategy<Value = (Vec<SectionEdge>, Vec<Point3>)> {
    let one_loop = (
        2usize..=5,
        prop::collection::vec(0.2f64..5.0, 9),
        prop::collection::vec(-0.2f64..0.2, 3),
        0.5f64..3.0,
        prop::collection::vec(0.02f64..0.98, 1..5),
        prop::collection::vec((-1.0f64..1.0, -1.0f64..1.0, -1.0f64..1.0), 3),
    );
    (
        prop::collection::vec(one_loop, 2..=4),
        prop::sample::select(vec![0.0, 2.0, 40.0]),
        prop::sample::select(vec![0.0, 1e6]),
    )
        .prop_map(|(loops, spacing, offset)| {
            let weld = 1e-5;
            let mut sections = Vec::new();
            let mut cuts = Vec::new();
            for (i, (degree, weights, wobble, radius, ts, dirs)) in loops.into_iter().enumerate() {
                #[allow(clippy::cast_precision_loss)]
                let center = Point3::new(offset + spacing * i as f64, 0.5 * i as f64, -offset);
                let n = (degree + 3).min(weights.len());
                let curve = closed_loop(degree, center, radius, &weights[..n], &wobble);
                for &t in &ts {
                    cuts.push(curve.evaluate(t));
                }
                for (k, &(x, y, z)) in dirs.iter().enumerate() {
                    let d = remus_math::vec::Vec3::new(x, y, z);
                    let Ok(d) = d.normalize() else { continue };
                    let p = curve.evaluate(ts[k % ts.len()]);
                    cuts.push(p + d * (0.9 * weld));
                    cuts.push(p + d * (3.0 * weld));
                }
                if let Some(b) = weld_reach_box(&curve, weld) {
                    let s = remus_math::vec::Vec3::new(1e-11, 1e-11, 1e-11) * (1.0 + offset);
                    cuts.extend([b.min + s, b.min - s, b.max - s, b.max + s]);
                }
                sections.push(closed_section(curve));
            }
            (sections, cuts)
        })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]
    #[test]
    fn the_reach_box_gate_never_changes_a_presplit((sections, cuts) in presplit_case()) {
        assert_presplit_matches_reference(&sections, &cuts, 1e-7);
    }
}

/// Every face carries every loop's cuts. With the gate, L disjoint loops
/// project 3 cuts each — 3L — instead of all 3L cuts each — 3L².
#[test]
fn scaling_winding_cut_projections_stay_linear_in_the_loop_count() {
    let weights = [1.0, 0.6, 1.4, 1.0, 0.8, 1.2, 1.0];
    for loops in [1usize, 2, 4, 8] {
        let mut sections = Vec::new();
        let mut cuts = Vec::new();
        for i in 0..loops {
            #[allow(clippy::cast_precision_loss)]
            let center = Point3::new(5.0 * i as f64, 0.0, 0.0);
            let curve = closed_loop(3, center, 1.0, &weights, &[0.0, 0.05]);
            cuts.extend([0.2, 0.5, 0.8].iter().map(|&t| curve.evaluate(t)));
            sections.push(closed_section(curve));
        }
        let _ = crate::perf::take_winding_cut_projections();
        let out = presplit_closed_winding_loops(
            &sections,
            &cuts,
            &FaceSurface::Plane { normal: Z, d: 0.0 },
            Rank::A,
            &[],
            1e-7,
        )
        .unwrap();
        assert_eq!(out.len(), 4 * loops);
        assert_eq!(
            crate::perf::take_winding_cut_projections(),
            3 * loops as u64
        );
    }
}
