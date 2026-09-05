//! Regression coverage for finite, trimmed solid boundaries.

#![allow(clippy::unwrap_used)]

use remus_math::{mat::Mat4, vec::Point3};
use remus_operations::{
    distance::{point_to_face, point_to_solid_distance},
    primitives::{make_box, make_cylinder},
    transform::transform_solid,
};
use remus_topology::{Topology, face::FaceSurface, solid::Solid};

#[test]
fn cavity_wall_is_the_nearest_boundary() {
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 10., 10., 10.).unwrap();
    let inner = make_box(&mut topo, 6., 6., 6.).unwrap();
    transform_solid(&mut topo, inner, &Mat4::translation(2., 2., 2.)).unwrap();
    let outer_shell = topo.solid(outer).unwrap().outer_shell();
    let inner_shell = topo.solid(inner).unwrap().outer_shell();
    for f in topo.shell(inner_shell).unwrap().faces().to_vec() {
        topo.face_mut(f).unwrap().set_reversed(true);
    }
    let hollow = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));
    let validation = remus_operations::validate::validate_solid(&topo, hollow).unwrap();
    assert!(
        validation.is_valid(),
        "hollow fixture invalid: {validation:?}"
    );
    let result = point_to_solid_distance(&topo, Point3::new(5., 5., 5.), hollow).unwrap();
    assert!(
        (result.distance - 3.).abs() < 1e-7,
        "expected cavity wall at 3 mm, got {:?}",
        result
    );
}

#[test]
fn cylinder_wall_projection_stays_inside_axial_trim() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1., 2.).unwrap();
    let shell = topo.solid(cylinder).unwrap().outer_shell();
    let wall = *topo
        .shell(shell)
        .unwrap()
        .faces()
        .iter()
        .find(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let result = point_to_face(&topo, Point3::new(1., 0., 5.), wall).unwrap();
    assert!(
        (result.distance - 3.).abs() < 1e-7,
        "expected finite wall at 3 mm, got {:?}",
        result
    );
}

#[test]
fn box_rejects_nonfinite_dimensions() {
    for dx in [f64::NAN, f64::INFINITY] {
        let mut topo = Topology::new();
        let result = make_box(&mut topo, dx, 1., 1.);
        assert!(
            result.is_err(),
            "expected invalid-input error for {dx}, got {result:?}"
        );
    }
}

#[test]
fn disjoint_fuse_distance_keeps_nearer_curved_face() {
    use remus_operations::boolean::{BooleanOp, boolean};
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1., 2.).unwrap();
    let box_solid = make_box(&mut topo, 1., 2., 2.).unwrap();
    transform_solid(&mut topo, box_solid, &Mat4::translation(-5., -1., 0.)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, cylinder, box_solid).unwrap();
    let validation = remus_operations::validate::validate_solid(&topo, fused).unwrap();
    assert!(
        validation.is_valid(),
        "fused fixture invalid: {validation:?}"
    );
    let result = point_to_solid_distance(&topo, Point3::new(-2., 0., 1.), fused).unwrap();
    assert!(
        (result.distance - 1.).abs() < 1e-7,
        "expected cylinder wall at 1 mm, got {:?}",
        result
    );
}

#[test]
fn cylinder_distance_does_not_depend_on_seam_side() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1., 2.).unwrap();
    for point in [
        Point3::new(2., 0., 1.),
        Point3::new(-2., 0., 1.),
        Point3::new(0., 2., 1.),
        Point3::new(0., -2., 1.),
    ] {
        let result = point_to_solid_distance(&topo, point, cylinder).unwrap();
        assert!(
            (result.distance - 1.).abs() < 1e-7,
            "expected wall at 1 mm from {:?}, got {:?}",
            point,
            result
        );
    }
}

#[test]
fn trimmed_cylinder_rim_projection_is_exact_away_from_samples() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 2.0, 3.0).unwrap();
    let angle = 0.137_f64;
    let point = Point3::new(2.0 * angle.cos(), 2.0 * angle.sin(), 5.0);
    let result = point_to_solid_distance(&topo, point, cylinder).unwrap();
    assert!((result.distance - 2.0).abs() < 1e-7, "{result:?}");
    assert!((result.point_b - Point3::new(point.x(), point.y(), 3.0)).length() < 1e-7);
}

#[test]
fn sphere_and_torus_surface_points_stay_on_the_boundary() {
    use remus_operations::primitives::{make_sphere, make_torus};
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 2.0, 16).unwrap();
    for point in [
        Point3::new(0.0, 0.0, 2.0),
        Point3::new(0.0, 0.0, -2.0),
        Point3::new(0.0, -2.0, 0.0),
    ] {
        let result = point_to_solid_distance(&topo, point, sphere).unwrap();
        assert!(result.distance < 1e-7, "sphere {point:?}: {result:?}");
    }
    let torus = make_torus(&mut topo, 3.0, 1.0, 16).unwrap();
    for point in [
        Point3::new(-4.0, 0.0, 0.0),
        Point3::new(0.0, -3.0, 1.0),
        Point3::new(2.0, 0.0, 0.0),
    ] {
        let result = point_to_solid_distance(&topo, point, torus).unwrap();
        assert!(result.distance < 1e-7, "torus {point:?}: {result:?}");
    }
}

#[test]
fn solid_distance_respects_cavity_faces_in_both_orders() {
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::distance::solid_to_solid_distance;
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let void = make_box(&mut topo, 6.0, 6.0, 6.0).unwrap();
    transform_solid(&mut topo, void, &Mat4::translation(2.0, 2.0, 2.0)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, void).unwrap();
    let inside = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, inside, &Mat4::translation(4.0, 4.0, 4.0)).unwrap();
    for (a, b) in [(hollow, inside), (inside, hollow)] {
        let result = solid_to_solid_distance(&topo, a, b).unwrap();
        assert!((result.distance - 2.0).abs() < 1e-7, "{result:?}");
    }
}

#[test]
fn nurbs_projection_respects_the_face_wire() {
    use remus_math::nurbs::surface::NurbsSurface;
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let face = remus_topology::explorer::solid_faces(&topo, solid).unwrap().into_iter().find(|&id| {
        matches!(topo.face(id).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.9)
    }).unwrap();
    let support = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 10.0, 1.0)],
            vec![Point3::new(10.0, 0.0, 1.0), Point3::new(10.0, 10.0, 1.0)],
        ],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap();
    topo.face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(support));
    let result = point_to_face(&topo, Point3::new(5.0, 0.5, 2.0), face).unwrap();
    assert!(
        (result.distance - 17.0_f64.sqrt()).abs() < 1e-7,
        "{result:?}"
    );
    assert!((result.point_b - Point3::new(1.0, 0.5, 1.0)).length() < 1e-7);
}

#[test]
fn planar_face_holes_are_excluded_from_distance_projection() {
    use remus_operations::boolean::{BooleanOp, boolean};
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 4.0, 4.0, 2.0).unwrap();
    let drill = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(2.0, 2.0, 0.0)).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();
    let top = remus_topology::explorer::solid_faces(&topo, drilled).unwrap().into_iter().find(|&id| {
        matches!(topo.face(id).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.9)
    }).unwrap();
    assert!(!topo.face(top).unwrap().inner_wires().is_empty());
    let result = point_to_face(&topo, Point3::new(2.0, 2.0, 2.0), top).unwrap();
    assert!((result.distance - 1.0).abs() < 1e-7, "{result:?}");
}

#[test]
fn cylinder_trims_follow_rigid_transforms() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 2.0, 3.0).unwrap();
    let transform = Mat4::translation(5.0, -2.0, 7.0) * Mat4::rotation_y(0.73);
    transform_solid(&mut topo, cylinder, &transform).unwrap();
    let point = transform.mul_point(Point3::new(0.0, 2.0, 5.0));
    let expected = transform.mul_point(Point3::new(0.0, 2.0, 3.0));
    let result = point_to_solid_distance(&topo, point, cylinder).unwrap();
    assert!((result.distance - 2.0).abs() < 1e-7, "{result:?}");
    assert!((result.point_b - expected).length() < 1e-7, "{result:?}");
}
