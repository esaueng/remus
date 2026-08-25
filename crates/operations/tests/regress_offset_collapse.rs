//! An offset larger than the body's own half-thickness must be refused, not
//! silently returned as an inside-out solid.
//!
//! The per-face offset guards each curved surface against its radius going
//! non-positive (`crates/offset/src/offset.rs`), but a plane has no radius —
//! it is simply translated by the distance — so an all-planar solid has no
//! per-face collapse condition at all. Past the half-thickness every face
//! crosses its opposite number, assembly succeeds on the inverted
//! arrangement, and the caller gets `Ok` with a solid that is inside out.
//!
//! Measured on a 10 mm box before the guard (half-extent 5, so anything at or
//! past -5 must collapse):
//!
//! | distance | returned |
//! | --- | --- |
//! | -4.9 | 0.008 mm^3 — correct |
//! | -5.0 | assembly happened to fail |
//! | -6.0 | Ok, 8 mm^3 |
//! | -10.0 | Ok, 1000 mm^3 — the untouched input |
//! | -1e6 | Ok, 8e18 mm^3 — grown, not shrunk |
//!
//! `remus_check::validate::validate_solid`, which the offset postcondition
//! already ran, passed all three: the check crate has no shell-orientation
//! check, and it is L2 so it cannot reach the L3 signed-volume machinery.
//! A negative signed volume on the outer shell is the one signature every
//! collapsed case shares, so the postcondition tests for it directly.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_operations::{measure, offset_v2, primitives};
use remus_topology::Topology;

/// Closed form for a cube offset by `d` on every face.
fn expected_volume(side: f64, d: f64) -> Option<f64> {
    let s = d.mul_add(2.0, side);
    (s > 0.0).then(|| s.powi(3))
}

#[test]
fn legal_offsets_are_still_exact() {
    for d in [0.5, 1.0, 2.0, 4.0, 4.9, -0.5, -1.0, -2.0, -4.0, -4.9] {
        let mut topo = Topology::new();
        let solid = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let result = offset_v2::offset_solid_v2(&mut topo, solid, d)
            .unwrap_or_else(|e| panic!("legal offset {d} must succeed: {e}"));
        let got = measure::solid_volume(&topo, result, 0.01).unwrap();
        let want = expected_volume(10.0, d).unwrap();
        assert!(
            (got - want).abs() / want < 1e-9,
            "offset {d}: want {want}, got {got}"
        );
    }
}

#[test]
fn collapsing_offsets_are_refused() {
    // Half-extent is 5, so -5 and beyond carry every face past its opposite.
    for d in [-5.0, -6.0, -10.0, -1e6] {
        let mut topo = Topology::new();
        let solid = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let result = offset_v2::offset_solid_v2(&mut topo, solid, d);
        assert!(
            result.is_err(),
            "offset {d} collapses a 10mm box and must be refused, got a solid with volume {:?}",
            result
                .ok()
                .map(|r| measure::solid_volume(&topo, r, 0.01).unwrap())
        );
    }
}

#[test]
fn a_collapsed_offset_never_returns_an_inside_out_solid() {
    // The specific defect: `Ok` carrying a solid whose outer shell is wound
    // inward. Whatever the operation decides, it must never be this.
    for d in [-6.0, -10.0, -1e6] {
        let mut topo = Topology::new();
        let solid = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        if let Ok(result) = offset_v2::offset_solid_v2(&mut topo, solid, d) {
            let report = remus_operations::validate::validate_solid(&topo, result).unwrap();
            assert!(
                report.is_valid(),
                "offset {d} returned Ok with an invalid solid: {:?}",
                report
                    .issues
                    .iter()
                    .map(|i| i.description.as_str())
                    .collect::<Vec<_>>()
            );
        }
    }
}

#[test]
fn shell_v2_inherits_the_guard() {
    // shell_v2 drives the same engine, so it collapsed the same way.
    for th in [1.0, 4.0, 4.9] {
        let mut topo = Topology::new();
        let solid = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        assert!(
            offset_v2::shell_v2(&mut topo, solid, -th, &[]).is_ok(),
            "shell_v2 at -{th} is legal on a 10mm box"
        );
    }
    for th in [5.0, 6.0, 10.0, 1e6] {
        let mut topo = Topology::new();
        let solid = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        assert!(
            offset_v2::shell_v2(&mut topo, solid, -th, &[]).is_err(),
            "shell_v2 at -{th} collapses a 10mm box and must be refused"
        );
    }
}
