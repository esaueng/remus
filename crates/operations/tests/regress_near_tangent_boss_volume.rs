//! A boss whose wall pokes a hair past a block side fuses into an exact
//! solid with a cylinder sliver along the side. Its seam lines are not quite
//! rulings (the vertex closing the near-tangent crossing leans along the
//! side), and at the smallest gaps its rim arcs are written as lines. The
//! volume must still come from the exact face integrals, not a chorded mesh.

#![allow(clippy::unwrap_used, clippy::expect_used, missing_docs)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;

/// Block 60 × 40 × 8, boss r 10, h 16 centred at (10 − gap, 20): the wall
/// pokes `gap` past the block's x = 0 side. The fuse used to measure
/// 21712.33 at gap 1e-7, short by the chord loss of the boss above the block.
#[test]
fn boss_a_hair_past_the_block_side_measures_exactly() {
    for gap in [1e-7, 1e-8] {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 60.0, 40.0, 8.0).unwrap();
        let boss = make_cylinder(&mut topo, 10.0, 16.0).unwrap();
        transform_solid(&mut topo, boss, &Mat4::translation(10.0 - gap, 20.0, 0.0)).unwrap();
        let fused = boolean(&mut topo, BooleanOp::Fuse, block, boss).unwrap();
        let report = validate_solid(&topo, fused).unwrap();
        assert!(report.is_valid(), "gap {gap:e}: {:?}", report.issues);

        // The block, the boss above it, and the segment of the boss disc
        // outside the block's side over the block's height.
        let half_chord = (gap * (20.0 - gap)).sqrt();
        let segment = 100.0 * ((10.0 - gap) / 10.0).acos() - (10.0 - gap) * half_chord;
        let expected = 60.0 * 40.0 * 8.0 + PI * 100.0 * 8.0 + segment * 8.0;
        for deflection in [1e-1, 1e-3] {
            let volume = solid_volume(&topo, fused, deflection).unwrap();
            assert!(
                ((volume - expected) / expected).abs() < 1e-12,
                "gap {gap:e}: volume {volume} vs {expected} at deflection {deflection:e}"
            );
        }
    }
}
