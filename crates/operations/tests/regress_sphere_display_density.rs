//! OpenZCAD #682: exact circular equators must retain the display mesh budget.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean_with_context};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_grouped_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

#[test]
fn sphere_display_density_survives_identity_cut_and_rigid_placement() {
    for scale in [0.001, 1.0, 1000.0] {
        for placed in [false, true] {
            for segments in [7, 32, 64] {
                let mut topo = Topology::new();
                let sphere = make_sphere(&mut topo, 10.0 * scale, segments).unwrap();
                if placed {
                    transform_solid(
                        &mut topo,
                        sphere,
                        &(Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                            * Mat4::rotation_x(0.37)
                            * Mat4::rotation_z(0.23)),
                    )
                    .unwrap();
                }
                let tool = make_box(&mut topo, 20.0 * scale, 20.0 * scale, 20.0 * scale).unwrap();
                transform_solid(
                    &mut topo,
                    tool,
                    &Mat4::translation(1000.0 * scale, 0.0, 0.0),
                )
                .unwrap();
                let cut = boolean_with_context(
                    &mut topo,
                    BooleanOp::Cut,
                    sphere,
                    tool,
                    &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                )
                .unwrap()
                .solid;
                for solid in [sphere, cut] {
                    let (mesh, groups) =
                        tessellate_solid_grouped_with_tolerance(&topo, solid, 0.004 * scale, 0.06)
                            .unwrap();
                    assert_eq!(
                        mesh.indices.len() / 3,
                        24_964,
                        "scale={scale} placed={placed}"
                    );
                    assert_eq!(groups.len(), 3);
                    // Float32 positions/normals and Uint32 indices, as sent to the viewport.
                    assert!(24 * mesh.positions.len() + 4 * mesh.indices.len() <= 599_904);
                    let center = if placed {
                        Point3::new(17.0 * scale, -23.0 * scale, 31.0 * scale)
                    } else {
                        Point3::new(0.0, 0.0, 0.0)
                    };
                    let radius = 10.0 * scale;
                    for (&p, &n) in mesh.positions.iter().zip(&mesh.normals) {
                        let radial = p - center;
                        assert!((radial.length() - radius).abs() < radius * 1e-10);
                        assert!(n.dot(radial) > radius * (1.0 - 1e-10));
                    }
                    for t in mesh.indices.chunks_exact(3) {
                        let [a, b, c] =
                            [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize] - center);
                        let centroid = (a + b + c) * (1.0 / 3.0);
                        assert!((b - a).cross(c - a).dot(centroid) > 0.0);
                        assert!(radius - centroid.length() <= 0.004 * scale);
                        for (p, q) in [(a, b), (b, c), (c, a)] {
                            assert!(radius - ((p + q) * 0.5).length() <= 0.004 * scale);
                            assert!(p.dot(q) / radius.powi(2) >= 0.06_f64.cos() - 1e-12);
                        }
                    }
                    assert_eq!(boundary_edge_count(&mesh), 0);
                    assert_eq!(non_manifold_edge_count(&mesh), 0);
                }
            }
        }
    }
}

#[test]
fn hemisphere_with_planar_cap_preserves_the_shared_partitioned_equator() {
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;

    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 10.0, 32).unwrap();
    remus_operations::tessellate::enable_face_mesh_cache();
    let (warm, _) = tessellate_solid_grouped_with_tolerance(&topo, sphere, 0.004, 0.06).unwrap();
    assert_eq!(warm.indices.len() / 3, 24_964);
    let faces = remus_topology::explorer::solid_faces(&topo, sphere).unwrap();
    let rim = topo.face(faces[1]).unwrap().outer_wire();
    let disc = topo.add_face(Face::new(
        rim,
        vec![],
        FaceSurface::Plane {
            d: 0.0,
            normal: Vec3::new(0.0, 0.0, -1.0),
        },
    ));
    let shell = topo.add_shell(Shell::new(vec![faces[0], disc]).unwrap());
    let capped = topo.add_solid(Solid::new(shell, vec![]));
    let (mesh, _) = tessellate_solid_grouped_with_tolerance(&topo, capped, 0.004, 0.06).unwrap();
    assert!(!mesh.indices.is_empty());
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);
    assert!(mesh.positions.iter().all(|p| p.z() >= -1e-10));
    remus_operations::tessellate::disable_face_mesh_cache();
}

#[test]
fn mixed_stored_equator_directions_preserve_both_hemispheres() {
    use remus_topology::wire::OrientedEdge;
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 10.0, 4).unwrap();
    let faces = remus_topology::explorer::solid_faces(&topo, sphere).unwrap();
    let wires: Vec<_> = faces
        .iter()
        .map(|&id| topo.face(id).unwrap().outer_wire())
        .collect();
    let edges: Vec<_> = topo
        .wire(wires[0])
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::wire::OrientedEdge::edge)
        .collect();
    for &edge_id in edges.iter().skip(1).step_by(2) {
        let edge = topo.edge(edge_id).unwrap();
        let (start, end) = (edge.start(), edge.end());
        let (a, b) = edge.strict_domain().unwrap();
        let edge = topo.edge_mut(edge_id).unwrap();
        edge.set_start(end);
        edge.set_end(start);
        edge.set_trim(Some((b, a)));
        for &wire in &wires {
            for oriented in topo.wire_mut(wire).unwrap().edges_mut() {
                if oriented.edge() == edge_id {
                    *oriented = OrientedEdge::new(edge_id, !oriented.is_forward());
                }
            }
        }
    }
    let (mesh, _) = tessellate_solid_grouped_with_tolerance(&topo, sphere, 0.004, 0.06).unwrap();
    assert!(mesh.positions.iter().any(|p| p.z() > 9.9));
    assert!(mesh.positions.iter().any(|p| p.z() < -9.9));
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);
}

#[test]
fn small_sphere_display_obeys_angular_budget() {
    for radius in [0.001, 0.003, 0.01] {
        let mut topo = Topology::new();
        let sphere = make_sphere(&mut topo, radius, 7).unwrap();
        let deflection = radius * 0.0004;
        let (mesh, _) =
            tessellate_solid_grouped_with_tolerance(&topo, sphere, deflection, 0.06).unwrap();
        let mut worst: f64 = 0.0;
        let mut worst_cos: f64 = 1.0;
        for t in mesh.indices.chunks_exact(3) {
            let [a, b, c] =
                [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize] - Point3::new(0.0, 0.0, 0.0));
            for (p, q) in [(a, b), (b, c), (c, a)] {
                worst = worst.max(radius - ((p + q) * 0.5).length());
                worst_cos = worst_cos.min(p.dot(q) / radius.powi(2));
            }
        }
        assert!(
            worst <= deflection,
            "r={radius} worst={worst} limit={deflection}"
        );
        assert!(
            worst_cos >= 0.06_f64.cos() - 1e-12,
            "r={radius} cos={worst_cos}"
        );
        assert_eq!(boundary_edge_count(&mesh), 0, "r={radius}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "r={radius}");
    }
}

#[test]
fn unequal_equator_arcs_retain_the_complete_native_turn() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 10.0, 4).unwrap();
    let faces = remus_topology::explorer::solid_faces(&topo, sphere).unwrap();
    let wire = topo.face(faces[0]).unwrap().outer_wire();
    let edges: Vec<_> = topo
        .wire(wire)
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::wire::OrientedEdge::edge)
        .collect();
    let angles = [0.0_f64, 0.1, 0.2, 0.3, std::f64::consts::TAU];
    for (i, &id) in edges.iter().enumerate() {
        let vertex = topo.edge(id).unwrap().start();
        topo.vertex_mut(vertex).unwrap().set_point(Point3::new(
            10.0 * angles[i].cos(),
            10.0 * angles[i].sin(),
            0.0,
        ));
        topo.edge_mut(id)
            .unwrap()
            .set_trim(Some((angles[i], angles[i + 1])));
    }
    let (mesh, _) = tessellate_solid_grouped_with_tolerance(&topo, sphere, 0.004, 0.06).unwrap();
    assert_eq!(mesh.indices.len() / 3, 24_964);
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);
    assert!(mesh.positions.iter().any(|p| p.x() > 9.9));
    assert!(mesh.positions.iter().any(|p| p.x() < -9.9));
}

#[test]
fn near_equal_sphere_carriers_keep_the_shared_rim_at_a_chord_count_boundary() {
    use remus_math::surfaces::SphericalSurface;
    use remus_topology::face::FaceSurface;
    let radius = 10.0;
    let delta = 1e-9;
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, radius, 7).unwrap();
    let faces = remus_topology::explorer::solid_faces(&topo, sphere).unwrap();
    topo.face_mut(faces[1])
        .unwrap()
        .set_surface(FaceSurface::Sphere(
            SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), radius + delta).unwrap(),
        ));
    let deflection = 2.0 * (radius + delta * 0.5) * (1.0 - (std::f64::consts::PI / 80.0).cos());
    let (mesh, _) =
        tessellate_solid_grouped_with_tolerance(&topo, sphere, deflection, 0.0).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);
}
