//! Finding from qualifying the planar collapse refusal (PR #1545): an inward
//! offset that carries a cylinder's caps through each other returned `Ok`.
//!
//! `make_cylinder(r = 5, h = 2)` offset by -1.5 should leave nothing: the caps
//! are 2 apart, so both pass the mid-plane. The engine instead returned a
//! solid of volume `pi * 3.5^2 * 1` (about 38.48): the radius shrank
//! correctly, and the height came out as `|2 - 3| = 1`, the caps having
//! swapped sides. The radius guard in `crates/offset/src/offset.rs` only
//! checks `r + d > 0`, and the planar collapse certificate in
//! `crates/operations/src/offset_v2.rs` covers only all-planar, straight-edged
//! bodies.
//!
//! Fixed in P-Class 5.7b's first slice: the slab certificate
//! (`remus_offset::shell_slab_thickness`) bounds a capped cylinder or cone
//! along its cap normal, so the same typed half-thickness refusal now covers
//! it before construction. `qualify_offset_interference.rs` holds the full
//! matrix; this file keeps the original repro.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_operations::measure::solid_volume;
use remus_operations::offset_v2::offset_solid_v2;
use remus_operations::primitives::make_cylinder;
use remus_topology::Topology;

fn live_state(t: &Topology) -> String {
    format!(
        "{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}",
        t.vertices().iter().collect::<Vec<_>>(),
        t.edges().iter().collect::<Vec<_>>(),
        t.wires().iter().collect::<Vec<_>>(),
        t.faces().iter().collect::<Vec<_>>(),
        t.shells().iter().collect::<Vec<_>>(),
        t.solids().iter().collect::<Vec<_>>(),
        t.journal().snapshot(),
        t.mutation_ticks()
    )
}

#[test]
fn cylinder_offset_past_half_height_is_refused() {
    for scale in [1.0, 1000.0] {
        // Zero height (d = -h/2) and pass-through (d > h/2), both with the
        // radius still positive so the existing radius guard does not fire.
        for d in [-1.0, -1.5] {
            let mut t = Topology::new();
            let source = make_cylinder(&mut t, 5.0 * scale, 2.0 * scale).unwrap();
            let before = live_state(&t);
            let distance = d * scale;
            match offset_solid_v2(&mut t, source, distance) {
                Ok(result) => panic!(
                    "scale {scale}: offset {distance} of an r5 h2 cylinder must not \
                     succeed, got volume {:?}",
                    solid_volume(&t, result, 1e-3 * scale)
                ),
                Err(_) => assert_eq!(live_state(&t), before, "scale {scale} d {distance}"),
            }
        }
    }
}
