//! Ready-repro: micron-scale boolean cut mis-assembles the hole walls.
//!
//! At model scale 1e-3 (a 1 µm-featured body under the kernel's fixed
//! 1e-7 mm linear tolerance) a plain box ∖ box through-cut returns a solid
//! whose hole walls keep the TOOL's full extent — they protrude beyond the
//! blank on both sides and the measured volume EXCEEDS the blank's own
//! (1.2e-9 vs the correct 0.84e-9). At scales 1 and 1e3 the same
//! configuration is exact.
//!
//! This is an Unsupported-untyped cell of the boolean family's scale axis
//! (capability-matrix: scale requires "the tolerance scaled
//! correspondingly", which the legacy entry point cannot express yet). The
//! productized fix is tolerance through `OperationContext` (RFC 0001)
//! reaching the GFA pave/weld bands; until then micron-scale booleans are a
//! declared gap, discovered and filed 2026-08-20 during defeature
//! qualification.

#![allow(clippy::unwrap_used)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

#[test]
#[ignore = "known gap: micron-scale cut carries untrimmed tool walls; needs scaled tolerance via OperationContext through GFA"]
fn micron_scale_through_cut_volume_is_correct() {
    let s = 1e-3;
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, s, s, s).unwrap();
    let cutter = make_box(&mut topo, 0.4 * s, 0.4 * s, 2.0 * s).unwrap();
    transform_solid(
        &mut topo,
        cutter,
        &Mat4::translation(0.3 * s, 0.3 * s, -0.5 * s),
    )
    .unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, cube, cutter).unwrap();
    let vol = solid_volume(&topo, holed, 0.01 * s).unwrap();
    let expected = 0.84 * s * s * s;
    assert!(
        ((vol - expected) / expected).abs() < 1e-6,
        "expected {expected:.6e}, got {vol:.6e}"
    );
}

/// The same configuration is exact at unit scale — the boundary's good side.
#[test]
fn unit_scale_through_cut_volume_is_correct() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let cutter = make_box(&mut topo, 0.4, 0.4, 2.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(0.3, 0.3, -0.5)).unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, cube, cutter).unwrap();
    let vol = solid_volume(&topo, holed, 0.01).unwrap();
    assert!(
        ((vol - 0.84) / 0.84).abs() < 1e-9,
        "expected 0.84, got {vol}"
    );
}
