//! Structured fuzzing of the solid offset engine.
//!
//! Builds a bounded PRIMITIVE (never a boolean result — see below), placed
//! rigidly, and offsets it by a small signed distance drawn from the
//! fuzzer's bytes. The oracle is independent of the offset machinery under
//! test: the offset result's measured volume must move in the *right
//! direction* (outward offsets grow, inward offsets shrink) and stay under
//! a generous convex ceiling, while the result itself must be a closed
//! 2-manifold.
//!
//! Concretely, per case with operand volume `V`, surface area `A`, distance
//! `d`, and result volume `V'`:
//!
//! * `sign(V' - V) == sign(d)` (within `VOL_SLACK`/`VOL_FLOOR` of zero
//!   movement — a zero-width offset regime is a pass, not a finding);
//! * `V' <= 2·(V + A·|d|) + floor` — admits exact second-order growth
//!   (edges/corners) while catching gross
//!   volume inflation. One-sided by design: offsets may grow
//!   super-linearly, never explode.
//!
//! Inputs are primitives only (`shapegen::Prim`, placed rigidly), NOT
//! boolean trees: compound operands entangle offset defects with boolean
//! misclassification (a fuse that keeps the wrong piece shrinks the
//! operand the oracle trusts), and that failure class is owned by the
//! `boolean_tree` target. Restricting the generator keeps every offset
//! finding attributable to the offset engine.
//!
//! The area `A` comes from `solid_surface_area`, a different kernel path
//! from the offset pipeline — but both are kernel code, so this oracle is
//! *weaker* than the closed-form oracles elsewhere by design, and the band
//! is correspondingly wide. What it catches: offsets that go the wrong way,
//! or exceed a broad volume ceiling. Inward collapse and smaller volume
//! errors can pass this deliberately one-sided bound.
//!
//! **A typed refusal is a pass.** Offset, construction, or measurement
//! errors stop the case silently. Panics only fire on
//! malformed-but-successful output.

#![cfg_attr(not(test), no_main)]

#[cfg(not(test))]
use libfuzzer_sys::fuzz_target;

mod invariants;
mod shapegen;

use invariants::{VOL_FLOOR, VOL_SLACK};
use remus_offset::offset_solid;
use remus_offset::{JointType, OffsetOptions};
use remus_operations::measure::{solid_bounding_box, solid_surface_area, solid_volume};
use remus_topology::Topology;
use shapegen::Refusal;

/// Cap on faces for the offset battery.
const FACE_LIMIT: usize = 120;

/// Offset distances stay small relative to the half-unit feature lattice so
/// collapse refusals are reachable but do not dominate.
fn distance(b: u8) -> f64 {
    let mag = 0.05 + f64::from(b % 8) * 0.05;
    if b.is_multiple_of(2) { mag } else { -mag }
}

#[derive(Debug, Clone, Copy, arbitrary::Arbitrary)]
struct Case {
    prim: shapegen::Prim,
    place: shapegen::Xform,
    dist: u8,
    joint_arc: bool,
}

#[cfg(not(test))]
fuzz_target!(|case: Case| run_case(case));

fn run_case(case: Case) {
    let mut topo = Topology::new();
    // Primitives only: boolean trees would entangle offset findings with
    // boolean misclassification (see the module docs). The placement is
    // rigid, so the hand-derived closed form survives it.
    let (solid, _expected) = match shapegen::build_prim_measured(&mut topo, case.prim) {
        Ok(pair) => pair,
        Err(Refusal::Engine(_) | Refusal::Degenerate) => return,
    };
    if remus_operations::transform::transform_solid(&mut topo, solid, &case.place.matrix()).is_err()
    {
        return;
    }
    let Ok(census) = invariants::census(&topo, solid) else {
        return;
    };
    if census.faces > FACE_LIMIT {
        return;
    }
    let root = solid;
    let d = distance(case.dist);
    if !d.is_finite() || d == 0.0 {
        return;
    }
    // Operand volume and area: the independent (if kernel-internal) inputs
    // to the prediction. Refusal of either is a pass. Both use the
    // operand's own deflection policy so the prediction is self-consistent.
    let op_diag = invariants::measure(&topo, root)
        .map(|m| (m.aabb.max - m.aabb.min).length())
        .unwrap_or(1.0);
    let op_defl = invariants::volume_deflection(op_diag);
    let (Some(v0), Some(a0)) = (
        invariants::measure(&topo, root).map(|m| m.volume),
        solid_surface_area(&topo, root, op_defl).ok(),
    ) else {
        return;
    };
    if !(v0.is_finite() && a0.is_finite()) || v0 <= 0.0 || a0 <= 0.0 {
        return;
    }

    let mut t = topo.clone();
    let options = OffsetOptions {
        joint: if case.joint_arc {
            JointType::Arc
        } else {
            JointType::Intersection
        },
        ..OffsetOptions::default()
    };
    // The clone preserves ids, so the same handle resolves in the fork.
    let result = match offset_solid(&mut t, root, d, options) {
        Ok(s) => s,
        Err(_) => return, // typed refusal is a pass
    };

    // Structural gate: a closed manifold at the B-Rep level (the
    // load-bearing claim — the tessellator can stitch a closed mesh over a
    // leaky B-Rep and can crack a closed B-Rep at per-face seams, so mesh
    // watertightness is a tessellation remark here, not an offset finding).
    let Ok(after) = invariants::census(&t, result) else {
        return;
    };
    invariants::assert_closed_manifold("offset", &after);
    let res_diag = solid_bounding_box(&t, result)
        .map(|a| (a.max - a.min).length())
        .unwrap_or(op_diag);
    let Ok(v1) = solid_volume(&t, result, invariants::volume_deflection(res_diag)) else {
        return;
    };
    assert!(
        v1.is_finite(),
        "offset: result volume is {v1} — successful output must be finite",
    );

    // Direction: outward grows, inward shrinks (up to slack at ~zero).
    let moved = v1 - v0;
    let zero_band = v0.abs().mul_add(VOL_SLACK, VOL_FLOOR);
    if moved.abs() > zero_band {
        assert!(
            moved.signum() == d.signum(),
            "offset: distance {d} moved volume {v0:.9} -> {v1:.9} the wrong way",
        );
    }

    // Magnitude: under the convex ceiling (see the module docs).
    let ceiling = 2.0 * (v0 + a0 * d.abs()) + v0.abs().mul_add(VOL_SLACK, VOL_FLOOR);
    assert!(
        v1 <= ceiling,
        "offset: result volume {v1:.9} escapes the convex ceiling {ceiling:.9} \
         (operand {v0:.9}, area {a0:.9}, distance {d})",
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use arbitrary::Arbitrary;

    #[test]
    fn committed_seed_offsets_a_box_outward() {
        let data = include_bytes!("../corpus/offset/box-outward");
        let case = Case::arbitrary(&mut arbitrary::Unstructured::new(data)).unwrap();
        assert!(matches!(
            case.prim,
            shapegen::Prim::Cuboid {
                dx: 2,
                dy: 2,
                dz: 2
            }
        ));
        assert!(distance(case.dist) > 0.0);
        assert!(!case.joint_arc);
        run_case(case);
    }
}
