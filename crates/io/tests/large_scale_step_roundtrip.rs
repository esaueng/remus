//! A body exported at a large model scale reads back.
//!
//! The reader classifies planar face bounds on polygons sampled from their
//! curves to a chordal deflection. That deflection was a fixed 1e-4 mm, so a
//! circular bound of radius 6000 mm needed more samples than the per-edge
//! budget and the reader refused the writer's own file ("FACE_BOUND curve
//! exceeds the adaptive planar-classification sampling limit"). OpenZCAD's
//! K10 matrix met it on a box ∪ cylinder rebuilt at 1000×.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::{read_step, write_step};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

/// The K10 box (60 × 60 × 20) and an upright r 6 h 30 cylinder at
/// (30, 30, 10), every length times `scale`.
fn box_and_cylinder(
    topo: &mut Topology,
    scale: f64,
    op: BooleanOp,
) -> remus_topology::solid::SolidId {
    let block = make_box(topo, 60.0 * scale, 60.0 * scale, 20.0 * scale).unwrap();
    let boss = make_cylinder(topo, 6.0 * scale, 30.0 * scale).unwrap();
    transform_solid(
        topo,
        boss,
        &Mat4::translation(30.0 * scale, 30.0 * scale, 10.0 * scale),
    )
    .unwrap();
    boolean(topo, op, block, boss).unwrap()
}

#[test]
fn box_and_cylinder_round_trip_at_every_scale() {
    for scale in [1.0, 10.0, 100.0, 1000.0] {
        for op in [BooleanOp::Fuse, BooleanOp::Cut] {
            let mut topo = Topology::new();
            let solid = box_and_cylinder(&mut topo, scale, op);
            let expected = solid_volume(&topo, solid, 1e-3 * scale).unwrap();
            let step = write_step(&topo, &[solid]).unwrap();
            let mut read = Topology::new();
            let solids = read_step(&step, &mut read)
                .unwrap_or_else(|e| panic!("scale {scale} {op:?}: read back: {e}"));
            assert_eq!(solids.len(), 1, "scale {scale} {op:?}");
            let volume = solid_volume(&read, solids[0], 1e-3 * scale).unwrap();
            assert!(
                (volume - expected).abs() <= 1e-9 * expected,
                "scale {scale} {op:?}: {volume} vs {expected}"
            );
        }
    }
}
