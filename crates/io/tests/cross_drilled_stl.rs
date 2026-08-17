//! Regression coverage for cross-drilled shaft STL fidelity.

#![allow(clippy::unwrap_used)]

use remus_io::stl::writer::StlFormat;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::{TriangleMesh, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn cross_drilled_shaft(bore_radius: f64) -> (Topology, remus_topology::solid::SolidId) {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, 3.0, 30.0).unwrap();
    let bore = make_cylinder(&mut topo, bore_radius, 40.0).unwrap();
    transform_solid(
        &mut topo,
        bore,
        &Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
    )
    .unwrap();
    transform_solid(&mut topo, bore, &Mat4::translation(-20.0, 0.0, 15.0)).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, shaft, bore).unwrap();
    (topo, drilled)
}

fn signed_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            let av = Vec3::new(a.x(), a.y(), a.z());
            let bv = Vec3::new(b.x(), b.y(), b.z());
            let cv = Vec3::new(c.x(), c.y(), c.z());
            av.dot(bv.cross(cv)) / 6.0
        })
        .sum()
}

#[test]
fn cross_drilled_stl_is_closed_manifold_and_matches_brep_volume() {
    for bore_radius in [3.0, 2.0, 1.0] {
        let (topo, solid) = cross_drilled_shaft(bore_radius);
        let brep_volume = solid_volume(&topo, solid, 0.05).unwrap();
        assert_eq!(
            classify_point(&topo, solid, Point3::new(0.0, 0.0, 15.0), 0.05, 1e-7).unwrap(),
            PointClassification::Outside,
            "bore r={bore_radius}: the bore axis must remain removed"
        );

        let display_mesh =
            remus_operations::tessellate::tessellate_solid(&topo, solid, 0.05).unwrap();
        let display_quality = welded_mesh_quality(&display_mesh);
        assert_eq!(
            (
                display_quality.boundary_edges,
                display_quality.non_manifold_edges
            ),
            (0, 0),
            "bore r={bore_radius}: source tessellation must be closed and manifold"
        );

        let bytes = remus_io::stl::write_stl(&topo, &[solid], 0.05, StlFormat::Ascii).unwrap();
        let mesh = remus_io::stl::read_stl(&bytes).unwrap();
        let quality = welded_mesh_quality(&mesh);
        assert_eq!(
            (quality.boundary_edges, quality.non_manifold_edges),
            (0, 0),
            "bore r={bore_radius}: exported STL must be closed and manifold"
        );
        let stl_volume = signed_volume(&mesh).abs();
        assert!(
            (stl_volume - brep_volume).abs() / brep_volume < 0.02,
            "bore r={bore_radius}: STL volume {stl_volume:.6} vs B-rep {brep_volume:.6}"
        );
    }
}
