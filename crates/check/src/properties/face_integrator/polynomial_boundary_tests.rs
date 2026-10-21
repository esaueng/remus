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

#[test]
fn unsupported_rational_and_high_degree_boundaries_keep_the_approximate_fallback() {
    for (degree, weights) in [(3, vec![1.0, 1e6, 1.0, 1.0]), (7, vec![1.0; 8])] {
        let points = (0..=degree)
            .map(|i| {
                let y = if i == 0 || i == degree {
                    0.0
                } else {
                    (i % 3 + 1) as f64
                };
                Point3::new(i as f64, y, 5.0)
            })
            .collect();
        let mut knots = vec![0.0; degree + 1];
        knots.extend(vec![1.0; degree + 1]);
        let curve = NurbsCurve::new(degree, knots, points, weights).unwrap();
        let (topo, face) = capped_curve(curve, (0.0, 1.0), false);
        let got = integrate_planar_face_exact(
            &topo,
            face,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
        )
        .unwrap();
        assert!(
            got.is_none(),
            "degree {degree} must retain the existing approximation"
        );
    }
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
