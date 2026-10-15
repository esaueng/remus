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
