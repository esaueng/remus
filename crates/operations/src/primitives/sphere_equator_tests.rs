//! Independent carrier and manifold oracles for the sphere's equator.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;

fn placement(radius: f64) -> Mat4 {
    Mat4::translation(13.0 * radius, -7.0 * radius, 3.0 * radius)
        * Mat4::rotation_z(0.43)
        * Mat4::rotation_x(0.71)
}

fn direction(matrix: Mat4, vector: Vec3) -> Vec3 {
    Vec3::new(
        matrix.0[0][0] * vector.x() + matrix.0[0][1] * vector.y() + matrix.0[0][2] * vector.z(),
        matrix.0[1][0] * vector.x() + matrix.0[1][1] * vector.y() + matrix.0[1][2] * vector.z(),
        matrix.0[2][0] * vector.x() + matrix.0[2][1] * vector.y() + matrix.0[2][2] * vector.z(),
    )
}

#[test]
fn sphere_equator_is_a_partition_of_exact_circle_carriers() {
    for radius in [0.001, 1.0, 1000.0] {
        for segments in [4, 7, 16, 32] {
            for placed in [false, true] {
                let mut topo = Topology::new();
                let solid = super::make_sphere(&mut topo, radius, segments).unwrap();
                let matrix = if placed {
                    placement(radius)
                } else {
                    Mat4::identity()
                };
                if placed {
                    crate::transform::transform_solid(&mut topo, solid, &matrix).unwrap();
                }
                let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
                let edges = remus_topology::explorer::solid_edges(&topo, solid).unwrap();
                let vertices = remus_topology::explorer::solid_vertices(&topo, solid).unwrap();
                assert_eq!(
                    (faces.len(), edges.len(), vertices.len()),
                    (2, segments, segments)
                );
                let north = topo
                    .wire(topo.face(faces[0]).unwrap().outer_wire())
                    .unwrap();
                let south = topo
                    .wire(topo.face(faces[1]).unwrap().outer_wire())
                    .unwrap();
                for (a, b) in north.edges().iter().zip(south.edges().iter().rev()) {
                    assert_eq!(a.edge(), b.edge());
                    assert!(a.is_forward());
                    assert!(!b.is_forward());
                }
                let center = matrix.mul_point(Point3::new(0.0, 0.0, 0.0));
                let normal = direction(matrix, Vec3::new(0.0, 0.0, 1.0));
                for (index, use_) in north.edges().iter().enumerate() {
                    let edge = topo.edge(use_.edge()).unwrap();
                    let circle = match edge.curve() {
                        EdgeCurve::Circle(circle) => circle,
                        EdgeCurve::Line
                        | EdgeCurve::Ellipse(_)
                        | EdgeCurve::NurbsCurve(_)
                        | EdgeCurve::Hyperbola(_)
                        | EdgeCurve::Parabola(_) => {
                            panic!("sphere equator must retain a circle carrier")
                        }
                    };
                    let a = std::f64::consts::TAU * index as f64 / segments as f64;
                    let b = std::f64::consts::TAU * (index + 1) as f64 / segments as f64;
                    assert_eq!(edge.trim(), Some((a, b)));
                    // The Gram defect bounds the radial error for EVERY
                    // parameter, independently of sampling the implementation.
                    let u = circle.u_axis();
                    let v = circle.v_axis();
                    let gram =
                        (u.dot(u) - 1.0).abs() + 2.0 * u.dot(v).abs() + (v.dot(v) - 1.0).abs();
                    let center_gap = (circle.center() - center).length();
                    let radial_bound = center_gap + radius * gram;
                    let plane_bound =
                        center_gap + radius * (normal.dot(u).abs() + normal.dot(v).abs());
                    assert!(radial_bound < 2.5e-8 && plane_bound < 2.5e-8);
                    for (t, vertex) in [(a, edge.start()), (b, edge.end())] {
                        assert!(
                            (circle.evaluate(t) - topo.vertex(vertex).unwrap().point()).length()
                                < 1e-7
                        );
                    }
                    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
                        let point = circle.evaluate(a + fraction * (b - a));
                        assert!(((point - center).length() - radius).abs() < 1e-7);
                        assert!((point - center).dot(normal).abs() < 1e-7);
                    }
                }
            }
        }
    }
}

#[test]
fn exact_equator_keeps_both_primitive_mesh_hemispheres_closed_and_outward() {
    for segments in [4, 7, 16, 32] {
        let mut topo = Topology::new();
        let sphere = super::make_sphere(&mut topo, 1.0, segments).unwrap();
        let mut previous_volume = 0.0;
        for deflection in [0.1, 0.01] {
            let mesh = crate::tessellate::tessellate_solid(&topo, sphere, deflection).unwrap();
            assert_eq!(crate::tessellate::boundary_edge_count(&mesh), 0);
            assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
            let mut volume = 0.0;
            let mut solid_angle = 0.0;
            let mut minimum_height = 1.0_f64;
            for triangle in mesh.indices.chunks_exact(3) {
                let [a, b, c] = [0, 1, 2].map(|i| mesh.positions[triangle[i] as usize]);
                let geometric = (b - a).cross(c - a);
                let radial = a - Point3::new(0.0, 0.0, 0.0);
                assert!(geometric.dot(radial) > 0.0);
                let [u, v, w] = [a, b, c].map(|p| p - Point3::new(0.0, 0.0, 0.0));
                for direction in [u, v, w] {
                    assert!((direction.length() - 1.0).abs() < 1e-12);
                }
                minimum_height = minimum_height.min(geometric.dot(radial) / geometric.length());
                solid_angle += 2.0
                    * u.dot(v.cross(w))
                        .atan2(1.0 + u.dot(v) + v.dot(w) + w.dot(u));
                volume += radial
                    .dot((b - Point3::new(0.0, 0.0, 0.0)).cross(c - Point3::new(0.0, 0.0, 0.0)))
                    / 6.0;
            }
            let exact = 4.0 * std::f64::consts::PI / 3.0;
            assert!((solid_angle - 4.0 * std::f64::consts::PI).abs() < 1e-10);
            // Radial projection covers the unit sphere once. Each triangular
            // cone contains its spherical cone scaled by the plane height,
            // giving a geometric volume bound at either mesh deflection.
            assert!(
                volume >= exact * minimum_height.powi(3) * (1.0 - 1e-12)
                    && volume <= exact * (1.0 + 1e-12),
                "segments={segments} deflection={deflection} V={volume} height={minimum_height}"
            );
            assert!(volume > previous_volume);
            previous_volume = volume;
        }
    }
}
