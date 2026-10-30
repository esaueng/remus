#![allow(clippy::expect_used, clippy::unwrap_used)]

use super::*;
use remus_math::nurbs::NurbsCurve;
use remus_topology::{
    edge::Edge,
    face::Face,
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

fn capped_curve(curve: NurbsCurve, trim: (f64, f64), reverse_storage: bool) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let a = topo.add_vertex(Vertex::new(curve.evaluate(trim.0.min(trim.1)), 1e-9));
    let b = topo.add_vertex(Vertex::new(curve.evaluate(trim.0.max(trim.1)), 1e-9));
    let (start, end) = if reverse_storage { (b, a) } else { (a, b) };
    let mut edge = Edge::new(start, end, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some(trim));
    let edge = topo.add_edge(edge);
    let closing = topo.add_edge(Edge::new(b, a, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(edge, !reverse_storage),
                OrientedEdge::new(closing, true),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 5.0,
        },
    ));
    (topo, face)
}

fn closed_curve(curve: NurbsCurve, at: Point3, trim: Option<(f64, f64)>) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let vertex = topo.add_vertex(Vertex::new(at, 1e-9));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(trim);
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 5.0,
        },
    ));
    (topo, face)
}

#[test]
fn closed_edges_honor_explicit_subloops_and_reject_invalid_trim_authority() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0],
        [
            (-10.0, 0.0),
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 0.0),
            (0.0, -20.0),
            (-10.0, 0.0),
        ]
        .map(|(x, y)| Point3::new(x, y, 5.0))
        .to_vec(),
        vec![1.0; 7],
    )
    .unwrap();
    for trim in [(1.0, 4.0), (4.0, 1.0)] {
        let (topo, face) = closed_curve(curve.clone(), Point3::new(0.0, 0.0, 5.0), Some(trim));
        let got = integrate_planar_face_exact(
            &topo,
            face,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
        )
        .unwrap()
        .unwrap();
        // The stored triangular subloop has area 2; the unused full carrier
        // encloses signed area -98 and must never enter this measurement.
        assert!((got.area - 2.0).abs() < 1e-12, "{}", got.area);
    }
    for trim in [(-1.0, 4.0), (1.0, 7.0), (1.0, 1.0)] {
        let (topo, face) = closed_curve(curve.clone(), Point3::new(0.0, 0.0, 5.0), Some(trim));
        assert!(
            integrate_planar_face_exact(
                &topo,
                face,
                Vec3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 0.0),
            )
            .is_err(),
            "invalid explicit trim must not become a full carrier"
        );
    }
}

#[test]
fn a_closed_polynomial_edge_without_trim_retains_full_carrier_compatibility() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0],
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
            .map(|(x, y)| Point3::new(x, y, 5.0))
            .to_vec(),
        vec![1.0; 5],
    )
    .unwrap();
    let (topo, face) = closed_curve(curve, Point3::new(0.0, 0.0, 5.0), None);
    let got = integrate_planar_face_exact(
        &topo,
        face,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
    )
    .unwrap()
    .unwrap();
    assert!((got.area - 4.0).abs() < 1e-12);
}

#[test]
fn unused_nonplanar_carrier_spans_cannot_tilt_the_trimmed_planar_frame() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0],
        vec![
            Point3::new(0.0, 0.0, 13.0),
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(2.0, 0.0, 5.0),
            Point3::new(2.0, 2.0, 5.0),
            Point3::new(2.0, 2.0, -11.0),
        ],
        vec![1.0; 5],
    )
    .unwrap();
    for trim in [(1.0, 3.0), (3.0, 1.0)] {
        for reverse_storage in [false, true] {
            let (topo, face) = capped_curve(curve.clone(), trim, reverse_storage);
            let got = integrate_planar_face_exact(
                &topo,
                face,
                Vec3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 0.0),
            )
            .unwrap()
            .expect("the authoritative trim is a planar triangle");
            // Triangle (0,0), (2,0), (2,2), at z=5. These are elementary
            // polygon moments, independent of NURBS evaluation/quadrature.
            for (actual, expected) in [
                (got.area, 2.0),
                (got.centroid_x, 8.0 / 3.0),
                (got.centroid_y, 4.0 / 3.0),
                (got.centroid_z, 10.0),
                (got.volume, 10.0 / 3.0),
                (got.volume_moment_z, 25.0),
                (got.volume_second_z, 250.0 / 3.0),
            ] {
                assert!((actual - expected).abs() < 1e-11, "{actual} vs {expected}");
            }
        }
    }
}

#[test]
fn degree_six_green_moments_match_exact_polynomial_integrals() {
    // x=t^6, y=t-t^6, closed by y=0. For each i+j<=3,
    // M_ij = 6/(j+1) integral_0^1 t^(6i+5) (t-t^6)^(j+1) dt.
    // Expanding this polynomial gives the rational constants below; no
    // sampled outline or Gauss rule participates in the oracle.
    let curve = NurbsCurve::new(
        6,
        [vec![0.0; 7], vec![1.0; 7]].concat(),
        (0..=6)
            .map(|i| {
                if i == 6 {
                    Point3::new(1.0, 0.0, 5.0)
                } else {
                    Point3::new(0.0, f64::from(i) / 6.0, 5.0)
                }
            })
            .collect(),
        vec![1.0; 7],
    )
    .unwrap();
    let (topo, face) = capped_curve(curve, (0.0, 1.0), false);
    let moments = planar_wire_monomial_moments(
        &topo,
        topo.face(face).unwrap().outer_wire(),
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
    .expect("degree-six polynomial moments have a proven exact rule");
    let expected = [
        5.0 / 14.0,
        5.0 / 39.0,
        25.0 / 312.0,
        5.0 / 76.0,
        25.0 / 1064.0,
        125.0 / 4788.0,
        1.0 / 25.0,
        1.0 / 100.0,
        1.0 / 150.0,
        1.0 / 100.0,
    ];
    for (actual, expected) in moments.into_iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-13, "{actual} vs {expected}");
    }
}

/// `x = tⁿ, y = t − tⁿ` as a degree-`n` Bézier at `z = 5`: control points
/// `(0, i/n)` and `(1, 0)` last.
fn power_cusp(n: usize, weights: Vec<f64>) -> NurbsCurve {
    let points = (0..=n)
        .map(|i| {
            if i == n {
                Point3::new(1.0, 0.0, 5.0)
            } else {
                Point3::new(0.0, i as f64 / n as f64, 5.0)
            }
        })
        .collect();
    NurbsCurve::new(
        n,
        [vec![0.0; n + 1], vec![1.0; n + 1]].concat(),
        points,
        weights,
    )
    .unwrap()
}

fn outer_moments(topo: &Topology, face: FaceId) -> Option<[f64; 10]> {
    planar_wire_monomial_moments(
        topo,
        topo.face(face).unwrap().outer_wire(),
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

#[test]
fn degree_seven_green_moments_converge_on_exact_polynomial_integrals() {
    // Past the exact rule's degree six, the adaptive rule. For x = t⁷,
    // y = t − t⁷ closed by y = 0, M_ij = 7/(j+1) ∫₀¹ t^(7i+6) (t − t⁷)^(j+1) dt,
    // expanded binomially: an oracle with no quadrature in it.
    let n = 7;
    let (topo, face) = capped_curve(power_cusp(n, vec![1.0; n + 1]), (0.0, 1.0), false);
    let moments = outer_moments(&topo, face).expect("the adaptive rule certifies a polynomial");
    let exponents = [
        (0, 0),
        (1, 0),
        (0, 1),
        (2, 0),
        (1, 1),
        (0, 2),
        (3, 0),
        (2, 1),
        (1, 2),
        (0, 3),
    ];
    for ((i, j), actual) in exponents.into_iter().zip(moments) {
        let (a, m) = (n * i + n - 1, j + 1);
        let mut binomial = 1.0;
        let mut integral = 0.0;
        for k in 0..=m {
            let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
            integral += sign * binomial / (a + m - k + n * k + 1) as f64;
            binomial = binomial * (m - k) as f64 / (k + 1) as f64;
        }
        let expected = n as f64 / m as f64 * integral;
        assert!(
            (actual - expected).abs() < 1e-14,
            "M{i}{j}: {actual} vs {expected}"
        );
    }
}

/// The chord polygon's area of `curve` closed by its chord, whose error is
/// even in the step, Richardson-extrapolated over two step halvings: an
/// oracle with no Gauss rule in it.
fn extrapolated_shoelace(curve: &NurbsCurve) -> f64 {
    let shoelace = |samples: u32| {
        let mut twice = 0.0;
        for k in 0..samples {
            let a = curve.evaluate(f64::from(k) / f64::from(samples));
            let b = curve.evaluate(f64::from(k + 1) / f64::from(samples));
            twice += a.x() * b.y() - b.x() * a.y();
        }
        (twice / 2.0).abs()
    };
    let (coarse, mid, fine) = (shoelace(1 << 12), shoelace(1 << 13), shoelace(1 << 14));
    let once = [(4.0 * mid - coarse) / 3.0, (4.0 * fine - mid) / 3.0];
    (16.0 * once[1] - once[0]) / 15.0
}

#[test]
fn rational_boundaries_converge_on_an_extrapolated_shoelace_area() {
    // No fixed rule is exact for a rational span; the adaptive one certifies
    // itself and must land on the independent oracle.
    let curve = power_cusp(3, vec![1.0, 3.0, 0.5, 1.0]);
    let expected = extrapolated_shoelace(&curve);
    let (topo, face) = capped_curve(curve, (0.0, 1.0), false);
    let moments = outer_moments(&topo, face).expect("a smooth rational span certifies");
    assert!(
        (moments[0] - expected).abs() < 1e-12 * expected,
        "{} vs {expected}",
        moments[0]
    );
    let exact_area = integrate_planar_face_exact(
        &topo,
        face,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
    )
    .unwrap()
    .expect("the exact planar path takes the certified rule")
    .area;
    // Its own in-plane frame, so equal to rounding rather than bits.
    assert!(
        (exact_area - expected).abs() < 1e-12 * expected,
        "{exact_area}"
    );
    assert_eq!(
        planar_face_exact_area(&topo, face).unwrap(),
        Some(exact_area)
    );
}

#[test]
fn an_extreme_rational_weight_refuses_or_converges_but_never_misreads() {
    // A 1e6 weight packs the curve's turn into a sliver of its parameter;
    // the rule may refuse it (the face keeps its sampled fallback) but must
    // not certify a wrong value.
    let curve = power_cusp(3, vec![1.0, 1e6, 1.0, 1.0]);
    let expected = extrapolated_shoelace(&curve);
    let (topo, face) = capped_curve(curve, (0.0, 1.0), false);
    if let Some(moments) = outer_moments(&topo, face) {
        assert!(
            (moments[0] - expected).abs() < 1e-12 * expected,
            "{} vs {expected}",
            moments[0]
        );
    }
}

#[test]
fn an_adaptive_cell_that_never_converges_refuses() {
    // A non-finite integrand never compares as converged, so bisection runs
    // to its depth cap and refuses rather than accumulating anything.
    let nan = |_: f64| (Point3::new(f64::NAN, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    let (origin, e1, e2) = (
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    let (whole, _) = green_cell_moments((0.0, 1.0), &nan, origin, e1, e2);
    let mut adaptive = AdaptiveGreen {
        eval: &nan,
        origin,
        e1,
        e2,
        allowance: [1.0; 10],
        cells: 0,
    };
    let mut out = [0.0; 10];
    assert!(!adaptive.cell(&mut out, (0.0, 1.0), &whole, 0));
    assert_eq!((out, adaptive.cells), ([0.0; 10], 0));
}

#[test]
fn an_edge_past_its_cell_budget_refuses() {
    // A converged cell still counts against the budget, so an edge cannot
    // spend more than its share of the error allowance.
    let line = |u: f64| (Point3::new(u, u, 0.0), Vec3::new(1.0, 1.0, 0.0));
    let mut adaptive = AdaptiveGreen {
        eval: &line,
        origin: Point3::new(0.0, 0.0, 0.0),
        e1: Vec3::new(1.0, 0.0, 0.0),
        e2: Vec3::new(0.0, 1.0, 0.0),
        allowance: [1.0; 10],
        cells: ADAPTIVE_GREEN_MAX_CELLS,
    };
    let (whole, _) =
        green_cell_moments((0.0, 1.0), &line, adaptive.origin, adaptive.e1, adaptive.e2);
    assert!(!adaptive.cell(&mut [0.0; 10], (0.0, 1.0), &whole, 0));
}

// Four wire edges share positions but deliberately retain separate vertex
// handles, as assembly refinement can. The cubic is stored opposite the
// wire's traversal while every orientation flag is forward.
fn position_chained_cubic(reverse_storage: bool, shear: bool) -> (Topology, FaceId) {
    let point = |x, y| Point3::new(x, if shear { x + y } else { y }, 5.0);
    let curve = NurbsCurve::new(
        3,
        [vec![0.0; 4], vec![1.0; 4]].concat(),
        vec![
            point(2.0, 0.0),
            point(2.0, 1.0 / 3.0),
            point(2.0, 2.0 / 3.0),
            point(3.0, 1.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    let a = point(1.0, 0.0);
    let b = point(2.0, 0.0);
    let c = point(3.0, 1.0);
    let d = point(1.0, 1.0);
    let (curve_start, curve_end) = if reverse_storage { (c, b) } else { (b, c) };
    let mut topo = Topology::new();
    let edges = [
        (a, b, EdgeCurve::Line),
        (curve_start, curve_end, EdgeCurve::NurbsCurve(curve)),
        (c, d, EdgeCurve::Line),
        (d, a, EdgeCurve::Line),
    ]
    .into_iter()
    .map(|(start, end, curve)| {
        let start = topo.add_vertex(Vertex::new(start, 1e-9));
        let end = topo.add_vertex(Vertex::new(end, 1e-9));
        let mut edge = Edge::new(start, end, curve);
        edge.set_trim(Some((0.0, 1.0)));
        OrientedEdge::new(topo.add_edge(edge), true)
    })
    .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 5.0,
        },
    ));
    (topo, face)
}

#[test]
fn polynomial_newell_normal_chains_position_equal_distinct_vertices() {
    for shear in [false, true] {
        let normals = [false, true].map(|reverse_storage| {
            let (topo, face) = position_chained_cubic(reverse_storage, shear);
            wire_newell_normal(&topo, topo.face(face).unwrap().outer_wire())
                .unwrap()
                .expect("the cubic boundary spans a plane")
        });
        // Reversing endpoint storage must preserve the same sampled contour,
        // rather than merely produce some nonzero normal in the same plane.
        assert!(
            (normals[0] - normals[1]).length() < 1e-12,
            "shear={shear}: {:?} vs {:?}",
            normals[0],
            normals[1]
        );
    }
}

#[test]
fn polynomial_moments_chain_position_equal_distinct_vertices() {
    // The region is 0 <= y <= 1, 1 <= x <= 2 + y^3. Each oracle is the
    // elementary integral of x^i y^j over that region, without curve sampling
    // or quadrature: integral_0^1 y^j ((2+y^3)^(i+1)-1)/(i+1) dy.
    let expected = [
        5.0 / 4.0,
        29.0 / 14.0,
        7.0 / 10.0,
        767.0 / 210.0,
        97.0 / 80.0,
        1.0 / 2.0,
        3106.0 / 455.0,
        1483.0 / 660.0,
        8.0 / 9.0,
        11.0 / 28.0,
    ];
    for reverse_storage in [false, true] {
        let (topo, face) = position_chained_cubic(reverse_storage, false);
        let moments = planar_wire_monomial_moments(
            &topo,
            topo.face(face).unwrap().outer_wire(),
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap()
        .expect("the unit-weight cubic has an exact Green rule");
        for (actual, expected) in moments.into_iter().zip(expected) {
            assert!(
                (actual - expected).abs() < 1e-12,
                "reverse_storage={reverse_storage}: {actual} vs {expected}"
            );
        }
    }
}

#[test]
fn planar_polynomial_contributions_chain_position_equal_distinct_vertices() {
    for reverse_storage in [false, true] {
        // The area-preserving shear (x,y) -> (x,x+y) prevents the other
        // boundary edges from making the wrong curve direction cancel out.
        let (topo, face) = position_chained_cubic(reverse_storage, true);
        let got = integrate_planar_face_exact(
            &topo,
            face,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
        )
        .unwrap()
        .expect("the sheared cubic boundary remains planar");
        for (actual, expected) in [
            (got.area, 5.0 / 4.0),
            (got.centroid_x, 29.0 / 14.0),
            (got.centroid_y, 97.0 / 35.0),
            (got.centroid_z, 25.0 / 4.0),
            (got.volume, 25.0 / 12.0),
            (got.volume_moment_z, 125.0 / 8.0),
            (got.volume_second_z, 625.0 / 12.0),
        ] {
            assert!(
                (actual - expected).abs() < 1e-11,
                "reverse_storage={reverse_storage}: {actual} vs {expected}"
            );
        }
    }
}
