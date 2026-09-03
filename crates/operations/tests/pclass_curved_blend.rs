//! P-Class 5.2 regressions for blends between curved analytic supports.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_cone, make_cylinder, make_sphere};
use remus_operations::resize_blend::resize_blend;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_edges;
use remus_topology::face::FaceSurface;

fn signed_mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            (a.x() * (b.y() * c.z() - b.z() * c.y())
                + a.y() * (b.z() * c.x() - b.x() * c.z())
                + a.z() * (b.x() * c.y() - b.y() * c.x()))
                / 6.0
        })
        .sum()
}

fn assert_watertight_with_mesh_oracle(topo: &Topology, solid: remus_topology::solid::SolidId) {
    let validation = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(validation.is_valid(), "{validation:?}");
    let adjacency = topo.build_adjacency(solid).unwrap();
    assert_eq!(adjacency.boundary_edges().len(), 0, "free edges");
    assert_eq!(
        adjacency.non_manifold_edges().len(),
        0,
        "non-manifold edges"
    );

    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, 0.01).unwrap();
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "{quality:?}");
    let brep_volume = remus_operations::measure::solid_volume(topo, solid, 0.01).unwrap();
    let mesh_volume = signed_mesh_volume(&mesh).abs();
    assert!(
        (mesh_volume - brep_volume).abs() / brep_volume.abs().max(1.0) < 0.02,
        "mesh volume {mesh_volume} vs B-rep {brep_volume}"
    );
}

fn cylinder_cone_shoulder(topo: &mut Topology) -> (remus_topology::solid::SolidId, EdgeId) {
    let cylinder = make_cylinder(topo, 3.0, 5.0).unwrap();
    let cone = make_cone(topo, 3.0, 1.0, 4.0).unwrap();
    transform_solid(topo, cone, &Mat4::translation(0.0, 0.0, 5.0)).unwrap();
    let solid = boolean(topo, BooleanOp::Fuse, cylinder, cone).unwrap();
    let adjacency = topo.build_adjacency(solid).unwrap();
    let shoulder = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|edge| {
            let adjacent = adjacency.faces_for_edge(*edge);
            adjacent.len() == 2
                && matches!(
                    (
                        topo.face(adjacent[0]).unwrap().surface(),
                        topo.face(adjacent[1]).unwrap().surface(),
                    ),
                    (FaceSurface::Cylinder(_), FaceSurface::Cone(_))
                        | (FaceSurface::Cone(_), FaceSurface::Cylinder(_))
                )
        })
        .expect("fused body must retain its cylinder-cone shoulder");
    (solid, shoulder)
}

#[test]
fn cylinder_cone_shoulder_fillet_is_watertight() {
    let mut topo = Topology::new();
    let (solid, shoulder) = cylinder_cone_shoulder(&mut topo);
    let input_volume = remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
    let result = fillet_v2(&mut topo, solid, &[shoulder], 0.25).unwrap();
    assert!(!result.is_partial);
    assert!(
        remus_operations::measure::solid_volume(&topo, result.solid, 0.01).unwrap() < input_volume
    );
    assert_watertight_with_mesh_oracle(&topo, result.solid);
}

fn cross_drilled_shaft(topo: &mut Topology) -> (remus_topology::solid::SolidId, EdgeId) {
    let shaft = make_cylinder(topo, 3.0, 12.0).unwrap();
    let tool = make_cylinder(topo, 1.0, 10.0).unwrap();
    transform_solid(topo, tool, &Mat4::rotation_y(std::f64::consts::FRAC_PI_2)).unwrap();
    transform_solid(topo, tool, &Mat4::translation(-5.0, 0.0, 6.0)).unwrap();
    let solid = boolean(topo, BooleanOp::Cut, shaft, tool).unwrap();
    let adjacency = topo.build_adjacency(solid).unwrap();
    let rim = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|edge| {
            let adjacent = adjacency.faces_for_edge(*edge);
            adjacent.len() == 2
                && adjacent[0] != adjacent[1]
                && adjacent.iter().all(|face| {
                    matches!(
                        topo.face(*face).unwrap().surface(),
                        FaceSurface::Cylinder(_)
                    )
                })
        })
        .expect("cross-drilled body must have a cylinder-cylinder hole rim");
    (solid, rim)
}

#[test]
fn cross_drilled_hole_rim_fillet_is_watertight() {
    let mut topo = Topology::new();
    let (solid, rim) = cross_drilled_shaft(&mut topo);
    let before = remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
    let result = fillet_v2(&mut topo, solid, &[rim], 0.15).unwrap();
    let after = remus_operations::measure::solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(!result.is_partial);
    assert!(
        after > before,
        "a concave hole-rim fillet must add material"
    );
    assert_watertight_with_mesh_oracle(&topo, result.solid);
}

#[test]
fn cylinder_cone_band_resizes_through_exact_reconstruction() {
    let mut topo = Topology::new();
    let (sharp, shoulder) = cylinder_cone_shoulder(&mut topo);
    let sharp_volume = remus_operations::measure::solid_volume(&topo, sharp, 0.01).unwrap();
    let old = fillet_v2(&mut topo, sharp, &[shoulder], 0.25).unwrap();
    let old_volume = remus_operations::measure::solid_volume(&topo, old.solid, 0.01).unwrap();
    let band = remus_topology::explorer::solid_faces(&topo, old.solid)
        .unwrap()
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Torus(torus)
                    if (torus.minor_radius() - 0.25).abs() < 1e-7
            )
        })
        .expect("the cylinder-cone shoulder must produce an exact torus band");

    let removed = resize_blend(&mut topo, old.solid, band, 0.25, 0.0).unwrap();
    let removed_volume =
        remus_operations::measure::solid_volume(&topo, removed.solid, 0.01).unwrap();
    assert!(
        (removed_volume - sharp_volume).abs() / sharp_volume < 1e-7,
        "removal must recover the original sharp shoulder: {removed_volume} vs {sharp_volume}"
    );
    let resized = resize_blend(&mut topo, old.solid, band, 0.25, 0.15).unwrap();
    let resized_volume =
        remus_operations::measure::solid_volume(&topo, resized.solid, 0.01).unwrap();

    assert!(
        resized_volume > old_volume,
        "shrinking a convex shoulder fillet must restore material"
    );
    assert!(
        resized_volume < sharp_volume,
        "the smaller fillet must still remove material from the sharp shoulder"
    );
    assert_watertight_with_mesh_oracle(&topo, resized.solid);
}

#[test]
fn cylinder_sphere_shoulder_fillet_is_watertight() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
    let sphere = make_sphere(&mut topo, 3.0, 16).unwrap();
    transform_solid(&mut topo, sphere, &Mat4::translation(0.0, 0.0, 4.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Fuse, cylinder, sphere).unwrap();
    let adjacency = topo.build_adjacency(solid).unwrap();
    let shoulder = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|edge| {
            let faces = adjacency.faces_for_edge(*edge);
            faces.len() == 2
                && matches!(
                    (
                        topo.face(faces[0]).unwrap().surface(),
                        topo.face(faces[1]).unwrap().surface(),
                    ),
                    (FaceSurface::Cylinder(_), FaceSurface::Sphere(_))
                        | (FaceSurface::Sphere(_), FaceSurface::Cylinder(_))
                )
        })
        .expect("the fused body must retain its cylinder-sphere shoulder");

    let result = fillet_v2(&mut topo, solid, &[shoulder], 0.15).unwrap();

    assert!(!result.is_partial);
    assert_watertight_with_mesh_oracle(&topo, result.solid);
}

#[test]
fn cone_cone_shoulder_fillet_is_watertight() {
    let mut topo = Topology::new();
    let lower = make_cone(&mut topo, 3.0, 2.0, 3.0).unwrap();
    let upper = make_cone(&mut topo, 2.0, 0.5, 3.0).unwrap();
    transform_solid(&mut topo, upper, &Mat4::translation(0.0, 0.0, 3.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Fuse, lower, upper).unwrap();
    let adjacency = topo.build_adjacency(solid).unwrap();
    let shoulder = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|edge| {
            let faces = adjacency.faces_for_edge(*edge);
            faces.len() == 2
                && faces[0] != faces[1]
                && faces
                    .iter()
                    .all(|face| matches!(topo.face(*face).unwrap().surface(), FaceSurface::Cone(_)))
        })
        .expect("the fused body must retain its cone-cone shoulder");

    let result = fillet_v2(&mut topo, solid, &[shoulder], 0.15).unwrap();

    assert!(!result.is_partial);
    assert_watertight_with_mesh_oracle(&topo, result.solid);
}
