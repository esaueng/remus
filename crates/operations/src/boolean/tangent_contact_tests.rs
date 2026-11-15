//! Oracle tests for the tangent line-contact gate (#1739).
//!
//! The tangent lines are checked against closed-form placements, and the
//! gate against results taken below the operations gate, straight from the
//! engine: a hole tangent to a wall from the inside is a contact, the same
//! hole a tenth of a millimetre away or touching at one point is not, and
//! two bodies fused along a line are left alone.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use super::tangent_contact::{result_touches_itself_along_a_line, tangent_line};
use super::{BooleanOp, boolean};
use crate::copy::copy_and_transform_solid;
use crate::primitives::{make_box, make_cylinder};

const TOL: f64 = 1.0e-7;

fn cylinder(x: f64, y: f64, radius: f64) -> FaceSurface {
    FaceSurface::Cylinder(
        CylindricalSurface::new(Point3::new(x, y, 0.0), Vec3::new(0.0, 0.0, 1.0), radius).unwrap(),
    )
}

fn plane_x(d: f64) -> FaceSurface {
    FaceSurface::Plane {
        normal: Vec3::new(1.0, 0.0, 0.0),
        d,
    }
}

/// The line `(x, y, ·)` along z, as `(point, direction)` reduced to the two
/// coordinates and the axis the tests place it by.
fn assert_vertical_line_at(first: &FaceSurface, second: &FaceSurface, x: f64, y: f64) {
    for (a, b) in [(first, second), (second, first)] {
        let line = tangent_line(a, b, TOL).expect("the surfaces are tangent along a line");
        assert!(
            (line.point.x() - x).abs() < 1e-12 && (line.point.y() - y).abs() < 1e-12,
            "contact at ({}, {}), expected ({x}, {y})",
            line.point.x(),
            line.point.y()
        );
        assert!(
            line.dir.cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12,
            "the contact runs along the cylinder axis"
        );
        assert!(
            line.normal.cross(Vec3::new(1.0, 0.0, 0.0)).length() < 1e-12,
            "the shared tangent plane is x = const"
        );
    }
}

#[test]
fn a_plane_holding_one_ruling_of_a_cylinder_is_tangent_along_it() {
    // Axis at x = 23, radius 3: the plane x = 20 touches at (20, 10), the
    // plane x = 26 at (26, 10).
    assert_vertical_line_at(&plane_x(20.0), &cylinder(23.0, 10.0, 3.0), 20.0, 10.0);
    assert_vertical_line_at(&plane_x(26.0), &cylinder(23.0, 10.0, 3.0), 26.0, 10.0);
    // A secant plane, a clear plane and a plane across the axis are not,
    // and neither is a plane a linear tolerance either side of tangent: the
    // one leaves a wall and the other is cut along two lines.
    for other in [
        plane_x(20.1),
        plane_x(19.9),
        plane_x(23.0),
        plane_x(20.0 - 1e-7),
        plane_x(20.0 + 1e-7),
    ] {
        assert!(tangent_line(&other, &cylinder(23.0, 10.0, 3.0), TOL).is_none());
    }
    let across = FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1.0),
        d: 5.0,
    };
    assert!(tangent_line(&across, &cylinder(23.0, 10.0, 3.0), TOL).is_none());
}

#[test]
fn parallel_cylinders_touching_are_tangent_along_one_ruling() {
    // Outside each other: axes 5 apart, radii 3 and 2.
    assert_vertical_line_at(
        &cylinder(0.0, 10.0, 3.0),
        &cylinder(5.0, 10.0, 2.0),
        3.0,
        10.0,
    );
    // One inside the other: axes 1 apart, radii 3 and 2, touching at x = 3.
    assert_vertical_line_at(
        &cylinder(0.0, 10.0, 3.0),
        &cylinder(1.0, 10.0, 2.0),
        3.0,
        10.0,
    );
    // Crossing, clear, nested clear and coaxial pairs are not tangent, nor
    // is a pair a linear tolerance short of touching.
    for (other, radius) in [
        (4.0, 2.0),
        (6.0, 2.0),
        (0.5, 2.0),
        (0.0, 2.0),
        (0.0, 3.0),
        (5.0 + 1e-7, 2.0),
    ] {
        assert!(
            tangent_line(
                &cylinder(0.0, 10.0, 3.0),
                &cylinder(other, 10.0, radius),
                TOL
            )
            .is_none(),
            "axis at {other}, radius {radius}"
        );
    }
    // Skew axes never share a ruling.
    let tilted = FaceSurface::Cylinder(
        CylindricalSurface::new(Point3::new(5.0, 10.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 2.0)
            .unwrap(),
    );
    assert!(tangent_line(&cylinder(0.0, 10.0, 3.0), &tilted, TOL).is_none());
}

/// A 40×20×10 block with a 5 mm step down at x = 20, tall for x > 20.
fn stepped(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, 40.0, 20.0, 10.0).unwrap();
    let notch = make_box(topo, 20.0, 20.0, 10.0).unwrap();
    let notch = copy_and_transform_solid(topo, notch, &Mat4::translation(0.0, 0.0, 5.0)).unwrap();
    boolean(topo, BooleanOp::Cut, block, notch).unwrap()
}

fn hole(topo: &mut Topology, x: f64) -> SolidId {
    let tool = make_cylinder(topo, 3.0, 30.0).unwrap();
    copy_and_transform_solid(topo, tool, &Mat4::translation(x, 10.0, -10.0)).unwrap()
}

/// Whether the engine's own cut of a hole at `x` from the stepped block
/// touches itself along a tangent line.
fn raw_cut_touches_itself(x: f64) -> bool {
    let mut topo = Topology::new();
    let blank = stepped(&mut topo);
    let tool = hole(&mut topo, x);
    let result = remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Cut, blank, tool)
        .unwrap_or_else(|error| panic!("hole at {x}: the engine must build a result: {error}"));
    result_touches_itself_along_a_line(&topo, blank, tool, result, Tolerance::new()).unwrap()
}

#[test]
fn a_hole_tangent_to_a_wall_from_inside_touches_it_along_a_line() {
    // Axis 3 from the wall on the tall side: the hole wall and the step wall
    // share the line (20, 10, 5..10), each with it in its interior.
    assert!(raw_cut_touches_itself(23.0));
}

#[test]
fn a_hole_clear_of_the_wall_or_touching_it_at_a_point_is_not_a_line_contact() {
    // A tenth of a millimetre of wall is a wall.
    assert!(!raw_cut_touches_itself(23.1));
    // On the low side the tool is tangent to the same plane, but its wall
    // survives only below the floor, where the step wall is not: the two
    // meet at (20, 10, 5) alone.
    assert!(!raw_cut_touches_itself(17.0));
}

/// A cylinder fused against a box wall touches it along a line as a second
/// body. That union is two pieces, not one boundary touching itself.
#[test]
fn two_bodies_touching_along_a_line_are_left_alone() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 60.0, 40.0, 40.0).unwrap();
    let post = make_cylinder(&mut topo, 8.0, 55.0).unwrap();
    let post =
        copy_and_transform_solid(&mut topo, post, &Mat4::translation(-8.0, 20.0, -5.0)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, block, post)
        .expect("a tangent-contact fuse is a two-body union");
    assert!(
        !result_touches_itself_along_a_line(&topo, block, post, fused, Tolerance::new()).unwrap()
    );
}

/// Two holes of radius 3 with axes 6 apart touch along a line through the
/// plate: no material separates them there. The second cut is refused.
#[test]
fn a_hole_tangent_to_an_earlier_hole_is_refused() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 40.0, 20.0, 10.0).unwrap();
    let first = hole(&mut topo, 15.0);
    let drilled = boolean(&mut topo, BooleanOp::Cut, plate, first).unwrap();
    let second = hole(&mut topo, 21.0);
    match boolean(&mut topo, BooleanOp::Cut, drilled, second) {
        Err(crate::OperationsError::ExactOnlyUnattainable) => {}
        Ok(result) => panic!(
            "two tangent holes came back as a solid with {} faces",
            remus_topology::explorer::solid_faces(&topo, result)
                .unwrap()
                .len()
        ),
        Err(error) => panic!("expected the exact-only refusal, got {error}"),
    }
    // Clear of each other by a tenth of a millimetre they are two holes.
    let apart = hole(&mut topo, 21.1);
    let result = boolean(&mut topo, BooleanOp::Cut, drilled, apart).unwrap();
    let volume = crate::measure::solid_volume(&topo, result, 1e-4).unwrap();
    let expected = 8000.0 - 2.0 * std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (volume - expected).abs() < 0.01,
        "two clear holes: volume {volume:.4}, expected {expected:.4}"
    );
}
