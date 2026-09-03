//! Exact equal-radius perpendicular cylinder intersection.
//!
//! The two implicit quadrics factor into planar ellipse branches.  The
//! boolean must retain those analytic section curves instead of returning the
//! faceted mesh fallback.

#![allow(clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn perpendicular_pair(topo: &mut Topology, radius: f64) -> (SolidId, SolidId) {
    let height = 20.0;
    let first = make_cylinder(topo, radius, height).expect("first cylinder");
    transform_solid(topo, first, &Mat4::translation(0.0, 0.0, -height / 2.0))
        .expect("centre first cylinder");

    let second = make_cylinder(topo, radius, height).expect("second cylinder");
    transform_solid(topo, second, &Mat4::rotation_y(std::f64::consts::FRAC_PI_2))
        .expect("rotate second cylinder");
    transform_solid(topo, second, &Mat4::translation(-height / 2.0, 0.0, 0.0))
        .expect("centre second cylinder");
    (first, second)
}

fn assert_exact_steinmetz(topo: &Topology, solid: SolidId, radius: f64) {
    let report = validate_solid(topo, solid).expect("validate exact result");
    assert!(
        report.is_valid(),
        "invalid exact result: {:?}",
        report.issues
    );

    let faces = solid_faces(topo, solid).expect("result faces");
    assert_eq!(
        faces.len(),
        6,
        "the exact result is six cylinder patches, got {faces:?}"
    );

    for &face_id in &faces {
        let face = topo.face(face_id).expect("result face");
        assert!(
            matches!(face.surface(), FaceSurface::Cylinder(_)),
            "Steinmetz intersection retained a non-cylinder face: {}",
            face.surface().type_tag()
        );
        let wire = topo.wire(face.outer_wire()).expect("outer wire");
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge()).expect("wire edge");
            match edge.curve() {
                EdgeCurve::Ellipse(_) => {}
                EdgeCurve::Line => {}
                other => panic!("unexpected section carrier: {}", other.type_tag()),
            }
        }
    }

    let edges = solid_edges(topo, solid).expect("result edges");
    assert_eq!(
        edges.len(),
        10,
        "eight ellipse arcs plus two cylinder seams"
    );
    let adjacency = edge_to_face_map(topo, solid).expect("edge adjacency");
    assert!(
        adjacency.values().all(|uses| uses.len() == 2),
        "every result edge must have exactly two face uses: {adjacency:?}"
    );

    let mut ellipse_edges = 0_usize;
    for edge_id in edges {
        let edge = topo.edge(edge_id).expect("result edge");
        let EdgeCurve::Ellipse(ellipse) = edge.curve() else {
            continue;
        };
        ellipse_edges += 1;
        let (t0, t1) = edge.strict_domain().expect("ellipse authority");
        assert!(t0.is_finite() && t1.is_finite() && t1 > t0);

        let start = topo.vertex(edge.start()).expect("ellipse start").point();
        let end = topo.vertex(edge.end()).expect("ellipse end").point();
        let scale = radius.max(1.0);
        assert!((ellipse.evaluate(t0) - start).length() <= 1.0e-8 * scale);
        assert!((ellipse.evaluate(t1) - end).length() <= 1.0e-8 * scale);

        let midpoint = ellipse.evaluate(0.5 * (t0 + t1));
        for face_id in adjacency[&edge_id.index()].iter().copied() {
            let surface = topo.face(face_id).expect("adjacent face").surface();
            let (u, v) = surface
                .project_point(midpoint)
                .expect("cylinder projection");
            let on_surface = surface.evaluate(u, v).expect("cylinder evaluation");
            assert!(
                (on_surface - midpoint).length() <= 1.0e-8 * scale,
                "ellipse arc left adjacent cylinder face {face_id:?}"
            );
        }
    }
    assert_eq!(ellipse_edges, 8, "two ellipses split into quarter arcs");

    let exact_volume = 16.0 / 3.0 * radius.powi(3);
    let measured = solid_volume(topo, solid, 0.001).expect("volume");
    let relative_error = (measured - exact_volume).abs() / exact_volume;
    assert!(
        relative_error < 1.0e-4,
        "volume {measured} vs {exact_volume}, relative error {relative_error:.3e}"
    );

    for deflection in [0.05, 0.01, 0.001] {
        let mesh = tessellate_solid(topo, solid, deflection).expect("tessellate");
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "open mesh at deflection {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "non-manifold mesh at deflection {deflection}"
        );
    }
}

#[test]
fn perpendicular_equal_radius_intersect_is_exact_analytic() {
    for radius in [2.0_f64, 3.0, 5.0] {
        let mut topo = Topology::new();
        let (first, second) = perpendicular_pair(&mut topo, radius);
        let result =
            boolean(&mut topo, BooleanOp::Intersect, first, second).expect("Steinmetz intersect");
        assert_exact_steinmetz(&topo, result, radius);
    }
}

#[test]
fn rigidly_transformed_steinmetz_intersect_is_exact_analytic() {
    let radius = 3.0;
    let mut topo = Topology::new();
    let (first, second) = perpendicular_pair(&mut topo, radius);
    let motion =
        Mat4::translation(7.0, -4.0, 11.0) * Mat4::rotation_z(0.61) * Mat4::rotation_x(-0.37);
    transform_solid(&mut topo, first, &motion).expect("move first cylinder");
    transform_solid(&mut topo, second, &motion).expect("move second cylinder");
    let result = boolean(&mut topo, BooleanOp::Intersect, first, second)
        .expect("transformed Steinmetz intersect");
    assert_exact_steinmetz(&topo, result, radius);
}
