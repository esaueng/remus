//! Adaptive tolerances cannot remove fixed sampled-boundary error.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::CheckError;
use remus_check::properties::{
    PropertiesOptions,
    face_integrator::{integrate_face, integrate_face_with_options},
};
use remus_math::{
    curves::Circle3D,
    nurbs::surface::NurbsSurface,
    surfaces::CylindricalSurface,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::{Edge, EdgeCurve},
    face::{Face, FaceId, FaceSurface},
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

fn check_fixed_contract(topo: &Topology, face: FaceId) {
    let options = PropertiesOptions::default();
    let fixed = integrate_face(topo, face, options.gauss_order).unwrap();
    let compatible = integrate_face_with_options(topo, face, &options).unwrap();
    assert_eq!(format!("{fixed:?}"), format!("{compatible:?}"));
    for options in [
        PropertiesOptions {
            adaptive_eps: 1e-10,
            ..Default::default()
        },
        PropertiesOptions {
            max_depth: 0,
            ..Default::default()
        },
    ] {
        let error = integrate_face_with_options(topo, face, &options).unwrap_err();
        assert!(matches!(error, CheckError::IntegrationFailed(_)));
        assert!(error.to_string().contains("unsupported"), "{error}");
    }
    let invalid = PropertiesOptions {
        adaptive_eps: f64::NAN,
        ..Default::default()
    };
    assert!(integrate_face_with_options(topo, face, &invalid).is_err());
}

#[test]
fn nurbs_face_retains_default_result_and_refuses_adaptive_claim() {
    let s = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap();
    let mut topo = Topology::new();
    let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        .map(|(u, v)| topo.add_vertex(Vertex::new(s.evaluate(u, v), 1e-7)));
    let edges = (0..4)
        .map(|i| {
            OrientedEdge::new(
                topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line)),
                true,
            )
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(s)));
    check_fixed_contract(&topo, face);
}

#[test]
fn polygon_trimmed_cylinder_retains_default_and_refuses_tightening() {
    let mut topo = Topology::new();
    let s =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0).unwrap();
    let points = [(0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (0.0, 3.0)].map(|(u, v)| s.evaluate(u, v));
    let vertices = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let mut edges = Vec::new();
    for i in 0..4 {
        let mut edge = if i == 0 || i == 2 {
            let circle = Circle3D::new(
                Point3::new(0.0, 0.0, points[i].z()),
                Vec3::new(0.0, 0.0, if i == 0 { 1.0 } else { -1.0 }),
                5.0,
            )
            .unwrap();
            let start = circle.project(points[i]);
            let mut edge = Edge::new(
                vertices[i],
                vertices[(i + 1) % 4],
                EdgeCurve::Circle(circle),
            );
            edge.set_trim(Some((start, start + 2.0)));
            edge
        } else {
            Edge::new(vertices[i], vertices[(i + 1) % 4], EdgeCurve::Line)
        };
        // Exercise an explicit stored line trim as well as circular ones.
        if i == 1 || i == 3 {
            edge.set_trim(Some((0.0, 1.0)));
        }
        edges.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(s)));
    check_fixed_contract(&topo, face);
    assert!((integrate_face(&topo, face, 5).unwrap().area - 30.0).abs() < 1e-8);
}
