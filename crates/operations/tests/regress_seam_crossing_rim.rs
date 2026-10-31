//! A boss fused half into a plate, then drilled off its axis: the boss's
//! remaining wall wraps across its cylinder's u-seam, and its rim arcs must
//! keep their own sweep in the wall's UV chart for the drill to split it.

#![allow(clippy::unwrap_used, clippy::expect_used, missing_docs)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

/// Plate 60 × 8 × 30, a boss r 12, h 24 on the plate's back edge, and a bore
/// r 6 along z through the plate's front corner. The second cut used to read
/// a rim arc of the boss's wall as its complement and fall back to a mesh.
#[test]
fn bore_through_a_wall_that_wraps_its_seam_cuts_exactly() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 60.0, 8.0, 30.0).unwrap();
    let boss = make_cylinder(&mut topo, 12.0, 24.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(0.0, 8.0, 0.0)).unwrap();
    let bossed = boolean(&mut topo, BooleanOp::Fuse, plate, boss).unwrap();
    let bore = make_cylinder(&mut topo, 6.0, 48.0).unwrap();
    let bracket = boolean(&mut topo, BooleanOp::Cut, bossed, bore).unwrap();

    let report = validate_solid(&topo, bracket).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let cylinders = solid_faces(&topo, bracket)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count();
    assert!(cylinders >= 2, "boss and bore walls stay analytic");

    // The boss's part in the plate: the disc r 12 at (0, 8) over x >= 0,
    // 0 <= y <= 8, over the boss's height.
    let in_plate = 4.0 * 80.0_f64.sqrt() + 72.0 * (2.0_f64 / 3.0).asin();
    let bossed_volume = 60.0 * 8.0 * 30.0 + PI * 144.0 * 24.0 - in_plate * 24.0;
    // The bore takes its lens with the boss disc below z = 24 (its quarter in
    // the plate lies inside the boss disc) and its quarter in the plate above.
    let (r1, r2, d) = (6.0_f64, 12.0_f64, 8.0_f64);
    let lens = r1 * r1 * ((d * d + r1 * r1 - r2 * r2) / (2.0 * d * r1)).acos()
        + r2 * r2 * ((d * d + r2 * r2 - r1 * r1) / (2.0 * d * r2)).acos()
        - 0.5 * ((-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2)).sqrt();
    let expected = bossed_volume - lens * 24.0 - 9.0 * PI * 6.0;
    for deflection in [1e-1, 1e-4] {
        let volume = solid_volume(&topo, bracket, deflection).unwrap();
        assert!(
            ((volume - expected) / expected).abs() < 1e-12,
            "volume {volume} vs {expected} at deflection {deflection:e}"
        );
    }
}
