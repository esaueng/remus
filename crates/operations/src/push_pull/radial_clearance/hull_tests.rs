//! Oracles for the control-hull radial bounds of NURBS faces and for the
//! clearance of faces a swept selection carries.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{
    axis_clearance, axis_core, companion_clears_core, face_clears_coaxial_core,
    hull_axis_clearance, hull_core,
};
use crate::boolean::{BooleanOp, boolean};
use crate::primitives::{make_box, make_cylinder, make_torus};
use crate::thread::{CylinderThreadOptions, ThreadForm, ThreadKind, cut_thread};
use crate::transform::transform_solid;
use remus_math::{
    mat::Mat4,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    explorer::solid_faces,
    face::{FaceId, FaceSurface},
    solid::SolidId,
};

const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn square(centre: (f64, f64), half: f64, z: f64) -> Vec<Point3> {
    [
        (-1.0, -1.0),
        (1.0, -1.0),
        (1.0, 1.0),
        (-1.0, 1.0),
        (0.0, 0.0),
    ]
    .iter()
    .map(|&(a, b): &(f64, f64)| {
        Point3::new(a.mul_add(half, centre.0), b.mul_add(half, centre.1), z)
    })
    .collect()
}

#[test]
fn hull_clearance_is_the_distance_to_the_projected_hull() {
    let origin = Point3::new(0.0, 0.0, 0.0);
    // Corners at heights apart project onto one square across the axis.
    let mut points = square((5.0, 0.0), 1.0, 3.0);
    points.extend(square((5.0, 0.0), 1.0, -7.0));
    let clearance = hull_axis_clearance(&points, origin, UP).unwrap();
    assert!(clearance <= 4.0 && clearance > 4.0 - 1e-9, "{clearance}");
    // Diagonally off: the nearest corner.
    let corner = hull_axis_clearance(&square((4.0, 4.0), 1.0, 0.0), origin, -UP).unwrap();
    assert!((corner - 18.0_f64.sqrt()).abs() < 1e-9, "{corner}");
    // An axis through the hull, or on its boundary, proves nothing.
    assert!(hull_axis_clearance(&square((0.5, 0.0), 1.0, 0.0), origin, UP).is_none());
    assert!(hull_axis_clearance(&square((1.0, 0.0), 1.0, 0.0), origin, UP).is_none());
    // Degenerate hulls: a point and a segment, beside and across the axis.
    let point = [Point3::new(0.0, 3.0, 1.0), Point3::new(0.0, 3.0, 9.0)];
    let clearance = hull_axis_clearance(&point, origin, UP).unwrap();
    assert!(clearance <= 3.0 && clearance > 3.0 - 1e-9, "{clearance}");
    let beside = [Point3::new(-4.0, 2.0, 0.0), Point3::new(4.0, 2.0, 0.0)];
    let clearance = hull_axis_clearance(&beside, origin, UP).unwrap();
    assert!(clearance <= 2.0 && clearance > 2.0 - 1e-9, "{clearance}");
    let across = [Point3::new(-4.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0)];
    assert!(hull_axis_clearance(&across, origin, UP).is_none());
}

#[test]
fn hull_core_bounds_every_control_point_about_a_central_axis() {
    // An octagon of radius 3 about (10, -4), at several heights.
    let points: Vec<Point3> = (0..16)
        .map(|i| {
            let angle = f64::from(i % 8) * std::f64::consts::FRAC_PI_4;
            Point3::new(
                3.0f64.mul_add(angle.cos(), 10.0),
                3.0f64.mul_add(angle.sin(), -4.0),
                f64::from(i),
            )
        })
        .collect();
    let (origin, reach) = hull_core(&points, UP).unwrap();
    for p in &points {
        let radial = (*p - origin).cross(UP).length();
        assert!(radial <= reach, "{radial} > {reach}");
    }
    assert!(reach < 3.1, "{reach}");
    assert!(
        (origin.x() - 10.0).hypot(origin.y() + 4.0) < 0.1,
        "{origin:?}"
    );
}

/// A post of `radius` about (`x`, 0) standing 10 tall from z = 0, threaded
/// externally at pitch 1, and its NURBS flank faces.
fn threaded_post(topo: &mut Topology, radius: f64, x: f64) -> (SolidId, Vec<FaceId>) {
    let post = make_cylinder(topo, radius, 10.0).unwrap();
    let wall = solid_faces(topo, post)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let threaded = cut_thread(topo, post, wall, &form, &CylinderThreadOptions::default())
        .unwrap()
        .solid;
    transform_solid(topo, threaded, &Mat4::translation(x, 0.0, 0.0)).unwrap();
    let flanks: Vec<FaceId> = solid_faces(topo, threaded)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .collect();
    assert!(!flanks.is_empty());
    (threaded, flanks)
}

/// The top of a 30 x 18 x 10 block with an R6 boss through its corner: it
/// stays 6 from the z axis.
fn block_top_around_boss(topo: &mut Topology) -> FaceId {
    let block = make_box(topo, 30.0, 18.0, 10.0).unwrap();
    let boss = make_cylinder(topo, 6.0, 20.0).unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, boss, block).unwrap();
    solid_faces(topo, fused)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.99)
                && remus_topology::explorer::face_vertices(topo, face)
                    .unwrap()
                    .iter()
                    .all(|&v| (topo.vertex(v).unwrap().point().z() - 10.0).abs() < 1e-9)
        })
        .unwrap()
}

#[test]
fn nurbs_thread_inside_the_face_clearance_is_unreachable() {
    let mut topo = Topology::new();
    let top = block_top_around_boss(&mut topo);
    // Crests of radius 3 sit within the hull's 3 sqrt(2) of the axis.
    let (_, inside) = threaded_post(&mut topo, 3.0, 0.0);
    for &flank in &inside {
        let (_, reach) = axis_core(&topo, flank, UP).unwrap().unwrap();
        assert!(reach < 3.0 * 2.0_f64.sqrt() + 1e-6, "{reach}");
        assert!(face_clears_coaxial_core(&topo, top, flank, UP).unwrap());
    }
    // Crests of radius 6.5 run under the block top: some flank reaches it.
    let (_, crossing) = threaded_post(&mut topo, 6.5, 0.0);
    assert!(
        crossing
            .iter()
            .any(|&flank| !face_clears_coaxial_core(&topo, top, flank, UP).unwrap())
    );
    for &flank in &crossing {
        if let Some((_, reach)) = axis_core(&topo, flank, UP).unwrap() {
            assert!(reach >= 6.0, "a flank reaching r = 6.5 bounded by {reach}");
        }
    }
}

fn torus_face(topo: &mut Topology, major: f64, minor: f64, height: f64) -> FaceId {
    let torus = make_torus(topo, major, minor, 16).unwrap();
    transform_solid(topo, torus, &Mat4::translation(0.0, 0.0, height)).unwrap();
    solid_faces(topo, torus)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap()
}

#[test]
fn carried_torus_clears_a_thread_only_inside_its_tube() {
    let mut topo = Topology::new();
    // A tube 8 to 12 from the z axis, above the posts.
    let band = torus_face(&mut topo, 10.0, 2.0, 20.0);
    let lower = axis_clearance(&topo, band, Point3::new(0.0, 0.0, 0.0), UP)
        .unwrap()
        .unwrap();
    assert!(lower <= 8.0 && lower > 8.0 - 1e-9, "{lower}");
    let (_, inside) = threaded_post(&mut topo, 4.0, 0.0);
    for &flank in &inside {
        assert!(companion_clears_core(&topo, band, flank, UP).unwrap());
        assert!(companion_clears_core(&topo, band, flank, -UP).unwrap());
    }
    // A post whose crests reach r = 9 runs into the tube.
    let (_, under) = threaded_post(&mut topo, 9.0, 0.0);
    assert!(
        under
            .iter()
            .all(|&flank| !companion_clears_core(&topo, band, flank, UP).unwrap())
    );
    // A narrow post off the axis crosses the tube at x = 8.
    let (_, off_axis) = threaded_post(&mut topo, 3.0, 6.0);
    assert!(
        off_axis
            .iter()
            .any(|&flank| !companion_clears_core(&topo, band, flank, UP).unwrap())
    );
    // A torus across the move gives no bound.
    let tilted = make_torus(&mut topo, 10.0, 2.0, 16).unwrap();
    transform_solid(&mut topo, tilted, &Mat4::rotation_x(0.3)).unwrap();
    let tilted = solid_faces(&topo, tilted)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    assert!(
        axis_clearance(&topo, tilted, Point3::new(0.0, 0.0, 0.0), UP)
            .unwrap()
            .is_none()
    );
}
