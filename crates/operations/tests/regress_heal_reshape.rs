//! `fix_shape` must apply recorded repairs to cavity shells, not merely count
//! them. The regression builds a real closed shell, stores its exterior and
//! cavity as separate shells, and adds one sub-tolerance face to a cavity wall.
#![allow(clippy::unwrap_used)]

use remus_heal::fix::{FixConfig, fix_shape};
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::shell_op::shell;
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn signed_mesh_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            Vec3::new(a.x(), a.y(), a.z()).dot(Vec3::new(b.x(), b.y(), b.z()).cross(Vec3::new(
                c.x(),
                c.y(),
                c.z(),
            ))) / 6.0
        })
        .sum()
}

fn add_cavity_sliver(topo: &mut Topology) -> FaceId {
    let points = [
        Point3::new(0.1, 0.2, 0.2),
        Point3::new(0.1, 0.2 + 1e-9, 0.2),
        Point3::new(0.1, 0.2, 0.2 + 1e-9),
    ];
    let vertices: Vec<_> = points
        .into_iter()
        .map(|point| topo.add_vertex(Vertex::new(point, 1e-7)))
        .collect();
    let edges = [
        topo.add_edge(Edge::new(vertices[0], vertices[1], EdgeCurve::Line)),
        topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line)),
        topo.add_edge(Edge::new(vertices[2], vertices[0], EdgeCurve::Line)),
    ];
    let wire = topo.add_wire(
        Wire::new(
            edges
                .into_iter()
                .map(|edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(-1.0, 0.0, 0.0),
            d: -0.1,
        },
    ))
}

#[test]
fn fix_shape_removes_a_recorded_sliver_from_the_cavity_shell() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let hollow = shell(&mut topo, block, 0.1, &[]).unwrap();

    // `shell_op` emits the six exterior faces followed by the six cavity
    // faces in one connected-components shell. Store those two closed
    // components using the solid's native outer/inner-shell representation.
    let generated_shell = topo.solid(hollow).unwrap().outer_shell();
    let generated_faces = topo.shell(generated_shell).unwrap().faces().to_vec();
    assert_eq!(generated_faces.len(), 12);
    let outer_shell = topo.add_shell(Shell::new(generated_faces[..6].to_vec()).unwrap());
    let sliver = add_cavity_sliver(&mut topo);
    let mut cavity_faces = generated_faces[6..].to_vec();
    cavity_faces.push(sliver);
    let cavity_shell = topo.add_shell(Shell::new(cavity_faces).unwrap());
    *topo.solid_mut(hollow).unwrap() = Solid::new(outer_shell, vec![cavity_shell]);

    let (healed, result) = fix_shape(&mut topo, hollow, &FixConfig::default()).unwrap();
    assert!(
        result.actions_taken > 0,
        "the sliver must be detected as a repair"
    );

    let healed_data = topo.solid(healed).unwrap();
    assert_eq!(healed_data.inner_shells().len(), 1);
    let healed_cavity_faces = topo.shell(healed_data.inner_shells()[0]).unwrap().faces();
    assert!(
        !healed_cavity_faces.contains(&sliver),
        "the reported cavity repair must remove the sliver from the result"
    );
    assert_eq!(healed_cavity_faces.len(), 6);

    let validation = validate_solid(&topo, healed).unwrap();
    assert!(validation.is_valid(), "healed hollow body: {validation:?}");
    let expected = 1.0 - 0.8_f64.powi(3);
    for deflection in [5e-5, 2.5e-5] {
        let mesh = tessellate_solid(&topo, healed, deflection).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "deflection={deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "deflection={deflection}");
        let signed = signed_mesh_volume(&mesh);
        assert!(
            (signed - expected).abs() < 1e-9,
            "deflection={deflection}, signed={signed}, expected={expected}"
        );

        let measured = solid_volume(&topo, healed, deflection).unwrap();
        assert!(
            (measured - expected).abs() < 1e-9,
            "deflection={deflection}, measured={measured}, expected={expected}"
        );
    }
}
