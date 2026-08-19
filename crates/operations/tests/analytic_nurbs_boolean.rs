//! Regression: analytic (non-plane) × NURBS face pairs must genuinely
//! intersect in the FF phase.
//!
//! The GFA table's analytic×NURBS arm used to return no curves ("deferred to
//! later phases" — no later phase existed), so a boolean pairing a curved
//! analytic wall with any NURBS face silently skipped face splitting and
//! misbuilt or leaned on the mesh fallback. These tests pin the repaired arm
//! end to end: a cylinder cut by an all-B-spline box must split its wall and
//! land on the closed-form volume.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::convert_to_bspline;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;

/// Cylinder (analytic wall) minus a box whose every face is a B-spline:
/// each cut face crossing the wall is an analytic-cylinder × NURBS pair.
#[test]
fn cylinder_cut_by_bspline_box_splits_the_wall() {
    let mut topo = Topology::new();
    // r=5, h=10, axis +z, base at origin.
    let cyl = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    // A slab spanning z in [7.5, 12.5] that decapitates the cylinder. Wide
    // enough in x/y to swallow the whole wall circle.
    let slab = make_box(&mut topo, 20.0, 20.0, 5.0).unwrap();
    transform_solid(&mut topo, slab, &Mat4::translation(-10.0, -10.0, 7.5)).unwrap();
    let converted = convert_to_bspline(&mut topo, slab).unwrap();
    assert!(
        converted > 0,
        "slab faces should have converted to B-spline"
    );

    let result = boolean(&mut topo, BooleanOp::Cut, cyl, slab).unwrap();

    let report = validate_solid(&topo, result).unwrap();
    assert!(report.is_valid(), "cut result should validate: {report:?}");
    // Closed form: the surviving cylinder is r=5, h=7.5.
    let expected = std::f64::consts::PI * 25.0 * 7.5;
    let volume = solid_volume(&topo, result, 0.05).unwrap();
    assert!(
        (volume - expected).abs() / expected < 0.01,
        "volume {volume} should be within 1% of {expected}"
    );
}

/// The disjoint case must stay quiet: an all-B-spline box whose AABB is
/// nowhere near the cylinder wall's curves produces an empty intersection
/// from a genuine computation, and the cut leaves the cylinder unchanged.
#[test]
fn cylinder_cut_by_distant_bspline_box_is_a_no_op() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let cube = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    transform_solid(&mut topo, cube, &Mat4::translation(40.0, 40.0, 0.0)).unwrap();
    convert_to_bspline(&mut topo, cube).unwrap();

    let result = boolean(&mut topo, BooleanOp::Cut, cyl, cube).unwrap();

    let expected = std::f64::consts::PI * 25.0 * 10.0;
    let volume = solid_volume(&topo, result, 0.05).unwrap();
    assert!(
        (volume - expected).abs() / expected < 0.001,
        "volume {volume} should remain the full cylinder {expected}"
    );
}
