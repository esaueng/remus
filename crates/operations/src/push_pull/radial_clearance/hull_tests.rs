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

/// Review fixture (Sol): a hull spanning 2e155 across overflows its edge
/// spans; the NaN distances must refuse rather than drop out of the
/// minimum, which read the true clearance of 1 as about 1e155.
#[test]
fn overflowing_hull_gives_no_bound() {
    let points = [
        Point3::new(-1e155, 1.0, 0.0),
        Point3::new(1e155, 1.0, 0.0),
        Point3::new(1e155, 1e150, 0.0),
        Point3::new(-1e155, 1e150, 0.0),
    ];
    let origin = Point3::new(0.0, 0.0, 0.0);
    assert!(hull_axis_clearance(&points, origin, UP).is_none());
    assert!(hull_core(&points, UP).is_none());
}

fn cylinder_wall(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap()
}

/// Review fixture (Sol): a sewn rim stored 0.005 outside its radius-10
/// cylinder bounds the wall's reach, so a coaxial torus band whose tube
/// comes to 10.002 is not cleared from it.
#[test]
fn sewn_rim_past_its_cylinder_bounds_the_core() {
    use remus_math::curves::Circle3D;
    use remus_topology::edge::EdgeCurve;
    let mut topo = Topology::new();
    let post = make_cylinder(&mut topo, 10.0, 10.0).unwrap();
    let wall = cylinder_wall(&topo, post);
    let rim = remus_topology::explorer::face_edges(&topo, wall)
        .unwrap()
        .into_iter()
        .find(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_)))
        .unwrap();
    let EdgeCurve::Circle(circle) = topo.edge(rim).unwrap().curve().clone() else {
        unreachable!()
    };
    let wide = Circle3D::new(circle.center(), circle.normal(), 10.005).unwrap();
    let trim = topo.edge(rim).unwrap().trim();
    let edge = topo.edge_mut(rim).unwrap();
    edge.set_curve(EdgeCurve::Circle(wide));
    edge.set_trim(trim);
    let (_, reach) = axis_core(&topo, wall, UP).unwrap().unwrap();
    assert!(reach >= 10.005, "{reach}");
    let band = torus_face(&mut topo, 11.002, 1.0, 5.0);
    assert!(!companion_clears_core(&topo, band, wall, UP).unwrap());
    // The carrier alone would have cleared it.
    let lower = axis_clearance(&topo, band, Point3::new(0.0, 0.0, 0.0), UP)
        .unwrap()
        .unwrap();
    assert!(lower > 10.0 + 1e-3, "{lower}");
}

/// Review fixture (Sol): the tilt gate and allowance read the sine of the
/// angle from a unit direction; a half-length move normal must not halve
/// them.
#[test]
fn short_move_normal_keeps_the_tilt_allowance() {
    let mut topo = Topology::new();
    // Tilted past the angular tolerance: no bound along either normal (a
    // half-length normal read the tilt as 7.5e-13 and let it through).
    let steep = make_cylinder(&mut topo, 1.0, 1e12).unwrap();
    transform_solid(&mut topo, steep, &Mat4::rotation_y(1.5e-12)).unwrap();
    let steep = cylinder_wall(&topo, steep);
    assert!(axis_core(&topo, steep, UP).unwrap().is_none());
    assert!(axis_core(&topo, steep, UP * 0.5).unwrap().is_none());
    // Within it: the same reach, with the axis's whole drift charged.
    let post = make_cylinder(&mut topo, 1.0, 1e12).unwrap();
    transform_solid(&mut topo, post, &Mat4::rotation_y(0.9e-12)).unwrap();
    let wall = cylinder_wall(&topo, post);
    let (_, unit) = axis_core(&topo, wall, UP).unwrap().unwrap();
    let (_, short) = axis_core(&topo, wall, UP * 0.5).unwrap().unwrap();
    assert_eq!(unit.to_bits(), short.to_bits());
    assert!(
        unit >= 1.9 - 1e-9,
        "the axis drifts 0.9 over its length: {unit}"
    );
}

/// Review fixture (Sol): an annular cap with an inner 45° chamfer from
/// radius 10 down to 9, around a central radius-4 post whose top disc at
/// z = 3 lies inside the chamfer's swept box. The cap clears the disc
/// radially, and the chamfer it carries clears it by its own cone profile;
/// requiring companions on the radial branch must not refuse the move.
#[test]
fn carried_chamfer_cleared_by_its_own_sweep_passes_a_post_in_the_opening() {
    use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
    let mut topo = Topology::new();
    let section = [
        (0., -5.),
        (20., -5.),
        (20., 0.),
        (10., 0.),
        (9., -1.),
        (9., -3.),
        (4., -3.),
        (4., 3.),
        (0., 3.),
    ]
    .map(|(r, z)| Point3::new(r, 0., z));
    let wire = make_polygon_wire(&mut topo, &section, 1e-7).unwrap();
    let profile = make_planar_face_from_wire(&mut topo, wire).unwrap();
    let ring = crate::revolve::revolve(
        &mut topo,
        profile,
        Point3::new(0., 0., 0.),
        UP,
        std::f64::consts::TAU,
    )
    .unwrap();
    let cap = solid_faces(&topo, ring)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.99)
                && remus_topology::explorer::face_vertices(&topo, face)
                    .unwrap()
                    .iter()
                    .all(|&v| topo.vertex(v).unwrap().point().z().abs() < 1e-9)
        })
        .unwrap();
    let before = crate::measure::solid_volume(&topo, ring, 0.01).unwrap();
    for distance in [1.0, 5.0] {
        let mut scratch = topo.clone();
        let moved = crate::push_pull::move_faces(&mut scratch, ring, &[cap], distance).unwrap();
        let report = crate::validate::validate_solid(&scratch, moved).unwrap();
        assert!(report.is_valid(), "{:?}", report.issues);
        // The roof from the chamfer foot at 9 out to 20 rises; the bore wall
        // below the chamfer stretches.
        let grown = crate::measure::solid_volume(&scratch, moved, 0.01).unwrap() - before;
        let expected = std::f64::consts::PI * (400.0 - 81.0) * distance;
        assert!(
            (grown - expected).abs() < 1e-6 * before,
            "{grown} vs {expected}"
        );
    }
}
