//! Synthetic hose-hook fixture and its mixed-junction family pins.
//!
//! Dimensions (mm, explicit): backplate box 60(X) x 50(Y) x 6(Z) at the
//! origin; retaining lip box 60 x 8 x 10 on top of the plate back edge;
//! two reinforcing rib boxes 6(X) x 20(Y) x 18(Z) standing on the plate
//! front face at x = 12 and x = 42; one mounting through-hole (r = 4,
//! axis +Z) at (30, 25).
//!
//! The hook exercises three mixed-junction families beyond the L-bracket's
//! convex-majority notch (two convex + one concave):
//!
//! - rib-base junctions: one convex + two concave (concave-majority), where
//!   each rib meets the plate;
//! - hole-rim junctions: the concave closed rim where the bore meets the
//!   plate faces;
//! - lip junctions: all-convex (same family as the box corners).
//!
//! Whole-edge fillet at r = 1 succeeds in one operation (twelve tori:
//! eight rib-base mirrors, two lip notches over coplanar-split caps, and
//! two hole-rim bands); the test pins the junction census, the torus count,
//! validity, and material removal. The lip-only convex subset is covered by
//! the existing box-corner pins.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use std::collections::HashMap;

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::query::{EdgeConcavity, edge_concavity, filter_filletable_edges};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::solid::SolidId;

const HOLE_R: f64 = 4.0;
const FILLET_R: f64 = 1.0;

/// Backplate 60x50x6 + lip 60x8x10 at the back top + two 6x20x18 ribs +
/// one r=4 through-hole at (30,25).
fn hose_hook(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 60.0, 50.0, 6.0).unwrap();
    let lip = make_box(topo, 60.0, 8.0, 10.0).unwrap();
    transform_solid(topo, lip, &Mat4::translation(0.0, 42.0, 6.0)).unwrap();
    let mut solid = boolean(topo, BooleanOp::Fuse, plate, lip).unwrap();
    for rib_x in [12.0_f64, 42.0] {
        let rib = make_box(topo, 6.0, 20.0, 18.0).unwrap();
        transform_solid(topo, rib, &Mat4::translation(rib_x, 10.0, 6.0)).unwrap();
        solid = boolean(topo, BooleanOp::Fuse, solid, rib).unwrap();
    }
    let hole = make_cylinder(topo, HOLE_R, 10.0).unwrap();
    transform_solid(topo, hole, &Mat4::translation(30.0, 25.0, -2.0)).unwrap();
    boolean(topo, BooleanOp::Cut, solid, hole).unwrap()
}

#[derive(Debug, Default)]
struct Census {
    all_convex: usize,
    convex_majority_mixed: usize,
    concave_majority_mixed: usize,
    other: usize,
}

fn junction_census(topo: &Topology, solid: SolidId, selected: &[EdgeId]) -> Census {
    let selected_set: std::collections::HashSet<usize> =
        selected.iter().map(|e| e.index()).collect();
    let mut at_vertex: HashMap<usize, (Vec<EdgeId>, Vec<EdgeId>, usize)> = HashMap::new();
    for eid in solid_edges(topo, solid).unwrap() {
        if !selected_set.contains(&eid.index()) {
            continue;
        }
        let edge = topo.edge(eid).unwrap();
        let concavity = edge_concavity(topo, solid, eid, 0.1).unwrap();
        for vid in [edge.start(), edge.end()] {
            let entry = at_vertex
                .entry(vid.index())
                .or_insert_with(|| (vec![], vec![], 0));
            match concavity {
                EdgeConcavity::Convex => entry.0.push(eid),
                EdgeConcavity::Concave => entry.1.push(eid),
                EdgeConcavity::Tangent | EdgeConcavity::Unknown => entry.2 += 1,
            }
        }
    }
    let mut census = Census::default();
    for (_, (convex, concave, other)) in at_vertex {
        if other > 0 || convex.is_empty() && concave.is_empty() {
            census.other += 1;
        } else if concave.is_empty() {
            census.all_convex += 1;
        } else if convex.is_empty() {
            census.other += 1;
        } else if convex.len() >= concave.len() {
            census.convex_majority_mixed += 1;
        } else {
            census.concave_majority_mixed += 1;
        }
    }
    census
}

#[test]
fn hook_fixture_is_valid_with_documented_topology() {
    let mut topo = Topology::new();
    let solid = hose_hook(&mut topo);
    let report = validate_solid(&topo, solid, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "hook fixture must validate");
    let volume = solid_volume(&topo, solid, 0.05).unwrap();
    // 60*50*6 + 60*8*10 + 2*6*20*18 - pi*16*6.
    let expected = 60.0 * 50.0 * 6.0 + 60.0 * 8.0 * 10.0 + 2.0 * 6.0 * 20.0 * 18.0
        - std::f64::consts::PI * HOLE_R * HOLE_R * 6.0;
    assert!(
        (volume - expected).abs() < 1.0,
        "hook volume {volume:.3} must meet closed form {expected:.3}"
    );
    eprintln!(
        "hook: {} faces, {} edges, vol {volume:.3}",
        solid_faces(&topo, solid).unwrap().len(),
        solid_edges(&topo, solid).unwrap().len()
    );
}

/// Whole-hook fillet succeeds in one operation: every sharp physical edge
/// (46) through the public cascade — plate and lip perimeters, both rib
/// bases (convex-singleton mirror tori), lip-notch corners (concave
/// singleton tori over coplanar-split caps), and both hole rims.
/// The junction census documents the families involved.
#[test]
fn hook_whole_edge_fillet_succeeds_with_family_census() {
    let mut topo = Topology::new();
    let solid = hose_hook(&mut topo);
    let all = solid_edges(&topo, solid).unwrap();
    let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
    assert_eq!(
        physical.len(),
        46,
        "whole-edge selection must stay complete"
    );
    let census = junction_census(&topo, solid, &physical);
    assert_eq!(
        (
            census.all_convex,
            census.convex_majority_mixed,
            census.concave_majority_mixed,
            census.other
        ),
        (22, 2, 8, 0),
        "hook junction census must be stable: {census:?}"
    );

    let vol_before = solid_volume(&topo, solid, 0.05).unwrap();
    let result = fillet_cascade(&mut topo, solid, &physical, FILLET_R)
        .unwrap_or_else(|e| panic!("hook whole-edge fillet must build: {e}"));
    assert!(
        !result.is_partial,
        "hook whole-edge fillet: no partial blends"
    );
    let report = validate_solid(&topo, result.solid, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "filleted hook must validate");
    // Twelve tori: eight rib-base mirrors, two lip notches, and the two
    // hole-rim bands (each rim rounds into one torus).
    let torus = solid_faces(&topo, result.solid)
        .unwrap()
        .iter()
        .filter(|fid| {
            matches!(
                topo.face(**fid).unwrap().surface(),
                remus_topology::face::FaceSurface::Torus(_)
            )
        })
        .count();
    assert_eq!(
        torus, 12,
        "all mixed junctions must close with torus patches"
    );
    let vol_after = solid_volume(&topo, result.solid, 0.05).unwrap();
    assert!(
        vol_after < vol_before,
        "a convex fillet must remove material: {vol_before:.4} -> {vol_after:.4}"
    );
    eprintln!("hook whole-edge r=1: {vol_before:.4} -> {vol_after:.4}");
}

/// Oversized radius still refuses atomically: r=5 exceeds the rib
/// half-width (3) and the plate thickness (6), so no feature can take it.
#[test]
fn hook_oversized_fillet_refuses_atomically() {
    let mut topo = Topology::new();
    let solid = hose_hook(&mut topo);
    let all = solid_edges(&topo, solid).unwrap();
    let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
    let faces_before = solid_faces(&topo, solid).unwrap().len();
    let vol_before = solid_volume(&topo, solid, 0.05).unwrap();
    let err = match fillet_cascade(&mut topo, solid, &physical, 5.0) {
        Ok(_) => panic!("hook oversized fillet must refuse"),
        Err(e) => e,
    };
    eprintln!("hook oversized refusal: {err}");
    assert_eq!(
        solid_faces(&topo, solid).unwrap().len(),
        faces_before,
        "oversized refusal must not change the face inventory"
    );
    let vol_after = solid_volume(&topo, solid, 0.05).unwrap();
    assert!(
        (vol_after - vol_before).abs() < 1e-9,
        "oversized refusal must not move volume"
    );
}
