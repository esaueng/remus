//! Equivalence oracle for the opt-in store-local booleans.
//!
//! Every case runs twice from the same topology: once with the store-local
//! path on, once with the general GFA. The two results must be
//! the same solid up to face order and entity numbering: the same multiset
//! of faces, each with its surface, orientation and wires (every edge's
//! curve, trim and endpoints exactly), the same entity counts, the same
//! volume, and both valid and watertight.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::store_local::{BUILT, set_thread_store_local_cuts};
use super::{BooleanOp, boolean};
use crate::primitives::{make_box, make_cylinder};
use crate::transform::transform_solid;

/// Each face as text: orientation, surface, and its wires as cyclic edge
/// sequences, each edge with its curve, trim and endpoints (`Debug` prints
/// every `f64` exactly). Sorted, so face order does not matter; a wire is
/// rotated to start at its smallest edge text, so its start does not either.
fn canonical(topo: &Topology, solid: SolidId) -> Vec<String> {
    let mut faces: Vec<String> = remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face_id| {
            let face = topo.face(face_id).unwrap();
            let mut wires: Vec<String> = std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .enumerate()
                .map(|(index, wire_id)| {
                    let mut uses: Vec<String> = topo
                        .wire(wire_id)
                        .unwrap()
                        .edges()
                        .iter()
                        .map(|oriented| {
                            let edge = topo.edge(oriented.edge()).unwrap();
                            let start = topo.vertex(edge.start()).unwrap().point();
                            let end = topo.vertex(edge.end()).unwrap().point();
                            // A line is the same use whichever way it is
                            // stored: compare where the traversal runs.
                            if matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
                                let (from, to) = if oriented.is_forward() {
                                    (start, end)
                                } else {
                                    (end, start)
                                };
                                return format!("Line {from:?} -> {to:?}");
                            }
                            format!(
                                "{} {:?} {:?} {start:?} -> {end:?}",
                                oriented.is_forward(),
                                edge.curve(),
                                edge.trim(),
                            )
                        })
                        .collect();
                    let start = (0..uses.len())
                        .min_by(|&i, &j| uses[i].cmp(&uses[j]))
                        .unwrap_or(0);
                    uses.rotate_left(start);
                    format!(
                        "{} [{}]",
                        if index == 0 { "outer" } else { "inner" },
                        uses.join(" | ")
                    )
                })
                .collect();
            // Inner wires carry no order of their own.
            wires[1..].sort();
            format!(
                "reversed={} {:?}\n  {}",
                face.is_reversed(),
                face.surface(),
                wires.join("\n  ")
            )
        })
        .collect();
    faces.sort();
    faces
}

fn counts(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    (
        remus_topology::explorer::solid_faces(topo, solid)
            .unwrap()
            .len(),
        remus_topology::explorer::solid_edges(topo, solid)
            .unwrap()
            .len(),
        remus_topology::explorer::solid_vertices(topo, solid)
            .unwrap()
            .len(),
    )
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::solid_volume(topo, solid, 0.01).unwrap()
}

fn assert_sound(topo: &Topology, solid: SolidId, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: invalid result {report:?}");
    let mesh = crate::tessellate::tessellate_solid(topo, solid, 0.05).unwrap();
    assert_eq!(
        crate::tessellate::boundary_edge_count(&mesh),
        0,
        "{label}: open mesh"
    );
}

/// Cut store-locally and through the general GFA; the two must be the same
/// solid. Returns the store-local result.
fn assert_equivalent(
    topo: &Topology,
    blank: SolidId,
    tool: SolidId,
    label: &str,
) -> (Topology, SolidId) {
    assert_equivalent_op(topo, BooleanOp::Cut, blank, tool, label)
}

/// Run `op` store-locally and through the general GFA; the two must be the
/// same solid. Returns the store-local result.
fn assert_equivalent_op(
    topo: &Topology,
    op: BooleanOp,
    blank: SolidId,
    tool: SolidId,
    label: &str,
) -> (Topology, SolidId) {
    let mut general = topo.clone();
    let general_result = boolean(&mut general, op, blank, tool).unwrap();
    let mut local = topo.clone();
    let built = BUILT.with(std::cell::Cell::get);
    set_thread_store_local_cuts(true);
    let local_result = boolean(&mut local, op, blank, tool);
    set_thread_store_local_cuts(false);
    let local_result = local_result.unwrap();
    assert_eq!(
        BUILT.with(std::cell::Cell::get),
        built + 1,
        "{label}: the store-local path declined"
    );
    assert_eq!(
        counts(&local, local_result),
        counts(&general, general_result),
        "{label}: entity counts differ"
    );
    let (ours, theirs) = (
        canonical(&local, local_result),
        canonical(&general, general_result),
    );
    if ours != theirs {
        let missing: Vec<_> = theirs.iter().filter(|f| !ours.contains(f)).collect();
        let extra: Vec<_> = ours.iter().filter(|f| !theirs.contains(f)).collect();
        panic!("{label}: faces differ\nonly in the GFA's: {missing:#?}\nonly in ours: {extra:#?}");
    }
    let (v_local, v_general) = (
        volume(&local, local_result),
        volume(&general, general_result),
    );
    assert!(
        (v_local - v_general).abs() <= 1e-9 * v_general.abs().max(1.0),
        "{label}: volume {v_local} vs GFA {v_general}"
    );
    assert_sound(&local, local_result, label);
    (local, local_result)
}

fn placed(topo: &mut Topology, solid: SolidId, matrix: &Mat4) -> SolidId {
    transform_solid(topo, solid, matrix).unwrap();
    solid
}

/// A 64 × 64 × 10 plate drilled with an `n × n` grid of r = 2 holes on a
/// 12 pitch (through the local cell).
fn drilled_plate(topo: &mut Topology, n: u32) -> SolidId {
    let mut plate = make_box(topo, 64.0, 64.0, 10.0).unwrap();
    for row in 0..n {
        for col in 0..n {
            let tool = make_cylinder(topo, 2.0, 20.0).unwrap();
            let at = Mat4::translation(
                8.0 + f64::from(col) * 12.0,
                8.0 + f64::from(row) * 12.0,
                -5.0,
            );
            let tool = placed(topo, tool, &at);
            plate = boolean(topo, BooleanOp::Cut, plate, tool).unwrap();
        }
    }
    plate
}

#[test]
fn slot_through_a_drilled_plate_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 4);
    let slot = make_box(&mut topo, 2.0, 6.0, 20.0).unwrap();
    let slot = placed(&mut topo, slot, &Mat4::translation(13.0, 11.0, -5.0));
    let (local, result) = assert_equivalent(&topo, plate, slot, "slot");
    let expected = 64.0 * 64.0 * 10.0 - 16.0 * PI * 4.0 * 10.0 - 2.0 * 6.0 * 10.0;
    let measured = volume(&local, result);
    assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
}

#[test]
fn blind_pocket_in_a_drilled_plate_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // 4 deep from the top, between the hole rows.
    let pocket = make_box(&mut topo, 6.0, 3.0, 10.0).unwrap();
    let pocket = placed(&mut topo, pocket, &Mat4::translation(11.0, 12.5, 6.0));
    assert_equivalent(&topo, plate, pocket, "pocket");
}

#[test]
fn rotated_slot_in_a_drilled_plate_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Turned about z: oblique side planes, sections at an angle.
    let tool = make_box(&mut topo, 6.0, 2.0, 20.0).unwrap();
    let tool = placed(
        &mut topo,
        tool,
        &(Mat4::translation(50.0, 50.0, -5.0) * Mat4::rotation_z(0.4)),
    );
    assert_equivalent(&topo, plate, tool, "rotated slot");
}

#[test]
fn edge_notch_touching_a_side_wall_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Through the plate and out of its y = 0 side.
    let notch = make_box(&mut topo, 4.0, 3.0, 20.0).unwrap();
    let notch = placed(&mut topo, notch, &Mat4::translation(12.0, -1.0, -5.0));
    assert_equivalent(&topo, plate, notch, "edge notch");
}

#[test]
fn hole_breaking_out_of_the_plate_edge_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // A cylinder across the x = 64 side: no local cell takes a breakout.
    let tool = make_cylinder(&mut topo, 3.0, 20.0).unwrap();
    let tool = placed(&mut topo, tool, &Mat4::translation(64.0, 20.0, -5.0));
    assert_equivalent(&topo, plate, tool, "edge breakout");
}

#[test]
fn slot_into_an_existing_hole_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // From between two holes into the hole at (20, 20): its wall joins the
    // patch.
    let slot = make_box(&mut topo, 6.0, 2.0, 20.0).unwrap();
    let slot = placed(&mut topo, slot, &Mat4::translation(14.0, 19.0, -5.0));
    assert_equivalent(&topo, plate, slot, "slot into a hole");
}

#[test]
fn successive_slots_stay_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    let mut local = topo.clone();
    let mut general = topo;
    let (mut local_plate, mut general_plate) = (plate, plate);
    for (i, x) in [13.0, 25.0, 37.0].into_iter().enumerate() {
        let at = Mat4::translation(x, 11.0, -5.0);
        let tool = make_box(&mut general, 2.0, 6.0, 20.0).unwrap();
        let tool = placed(&mut general, tool, &at);
        general_plate = boolean(&mut general, BooleanOp::Cut, general_plate, tool).unwrap();
        let tool = make_box(&mut local, 2.0, 6.0, 20.0).unwrap();
        let tool = placed(&mut local, tool, &at);
        set_thread_store_local_cuts(true);
        let cut = boolean(&mut local, BooleanOp::Cut, local_plate, tool);
        set_thread_store_local_cuts(false);
        local_plate = cut.unwrap();
        assert_eq!(
            canonical(&local, local_plate),
            canonical(&general, general_plate),
            "slot {i}: faces differ"
        );
    }
    assert_sound(&local, local_plate, "successive slots");
}

#[test]
fn rib_fused_onto_a_drilled_plate_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Sunk 2 into the top face between the hole rows, standing 2 proud.
    let rib = make_box(&mut topo, 6.0, 3.0, 4.0).unwrap();
    let rib = placed(&mut topo, rib, &Mat4::translation(11.0, 12.5, 8.0));
    let (local, result) = assert_equivalent_op(&topo, BooleanOp::Fuse, plate, rib, "rib");
    let expected = 64.0 * 64.0 * 10.0 - 9.0 * PI * 4.0 * 10.0 + 6.0 * 3.0 * 2.0;
    let measured = volume(&local, result);
    assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
}

#[test]
fn boss_fused_into_a_drilled_plate_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // A cylinder embedded 3 deep in the top face, clear of the holes.
    let boss = make_cylinder(&mut topo, 3.0, 6.0).unwrap();
    let boss = placed(&mut topo, boss, &Mat4::translation(26.0, 14.0, 7.0));
    assert_equivalent_op(&topo, BooleanOp::Fuse, plate, boss, "boss");
}

#[test]
fn boss_standing_on_a_drilled_plate_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Its base lies on the top face: a coplanar contact.
    let boss = make_box(&mut topo, 6.0, 3.0, 5.0).unwrap();
    let boss = placed(&mut topo, boss, &Mat4::translation(11.0, 12.5, 10.0));
    assert_equivalent_op(&topo, BooleanOp::Fuse, plate, boss, "standing boss");
}

#[test]
fn lug_fused_across_a_side_wall_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Out through the x = 64 side, inside the plate's height.
    let lug = make_box(&mut topo, 10.0, 6.0, 4.0).unwrap();
    let lug = placed(&mut topo, lug, &Mat4::translation(60.0, 23.0, 3.0));
    assert_equivalent_op(&topo, BooleanOp::Fuse, plate, lug, "lug");
}

#[test]
fn corner_of_a_drilled_plate_intersected_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Keeps a 20 × 20 window holding four holes; the untouched faces drop.
    let window = make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let window = placed(&mut topo, window, &Mat4::translation(4.0, 4.0, -5.0));
    let (local, result) =
        assert_equivalent_op(&topo, BooleanOp::Intersect, plate, window, "window");
    let expected = 20.0 * 20.0 * 10.0 - 4.0 * PI * 4.0 * 10.0;
    let measured = volume(&local, result);
    assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
}

#[test]
fn window_through_one_hole_intersected_is_equivalent() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Cuts off part of the hole at (32, 32) and keeps a thinner slab.
    let window = make_box(&mut topo, 8.0, 6.0, 6.0).unwrap();
    let window = placed(&mut topo, window, &Mat4::translation(28.0, 31.0, 2.0));
    assert_equivalent_op(&topo, BooleanOp::Intersect, plate, window, "half hole");
}

#[test]
fn fuse_with_a_tool_clear_of_the_material_declines() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    // Inside the hole at (20, 20) without touching its wall: the hole
    // wall's box reaches it, but the fuse leaves two lumps.
    let pin = make_box(&mut topo, 1.0, 1.0, 20.0).unwrap();
    let pin = placed(&mut topo, pin, &Mat4::translation(19.5, 19.5, -5.0));
    let mut general = topo.clone();
    let general_result = boolean(&mut general, BooleanOp::Fuse, plate, pin);
    let mut local = topo.clone();
    let built = BUILT.with(std::cell::Cell::get);
    set_thread_store_local_cuts(true);
    let local_result = boolean(&mut local, BooleanOp::Fuse, plate, pin);
    set_thread_store_local_cuts(false);
    assert_eq!(
        BUILT.with(std::cell::Cell::get),
        built,
        "built two lumps store-locally"
    );
    match (general_result, local_result) {
        (Ok(theirs), Ok(ours)) => assert_eq!(
            canonical(&local, ours),
            canonical(&general, theirs),
            "declined, yet the result differs"
        ),
        (Err(_), Err(_)) => {}
        (theirs, ours) => panic!("general {theirs:?} vs store-local {ours:?}"),
    }
}

/// Complexity guard for the store-local booleans: the GFA store holds the
/// faces the tool touches and the tool, not the whole blank. A slot cut, a
/// rib fuse and a window intersect on plates with 9 and 16 holes copy as
/// many faces into the store either way (the slot: the plate's top and
/// bottom and its own six); the general path copies far more at 16 holes
/// (54 for the slot: the 28-face plate and the slot, copied more than
/// once). Runs only with `--features perf-counters`, in the
/// `test(scaling_)` step.
#[cfg(feature = "perf-counters")]
#[test]
fn scaling_store_local_booleans_copy_only_touched_faces() {
    // (op, tool size, tool corner)
    let cases = [
        (BooleanOp::Cut, (2.0, 6.0, 20.0), (13.0, 11.0, -5.0)),
        (BooleanOp::Fuse, (6.0, 3.0, 4.0), (11.0, 12.5, 8.0)),
        (BooleanOp::Intersect, (20.0, 20.0, 20.0), (4.0, 4.0, -5.0)),
    ];
    for (op, (dx, dy, dz), (x, y, z)) in cases {
        let store_faces = |n: u32, local: bool| -> u64 {
            let mut topo = Topology::new();
            let plate = drilled_plate(&mut topo, n);
            let tool = make_box(&mut topo, dx, dy, dz).unwrap();
            let tool = placed(&mut topo, tool, &Mat4::translation(x, y, z));
            remus_algo::perf::reset();
            set_thread_store_local_cuts(local);
            let result = boolean(&mut topo, op, plate, tool);
            set_thread_store_local_cuts(false);
            result.unwrap();
            remus_algo::perf::snapshot().store_faces_copied
        };
        let (small, large) = (store_faces(3, true), store_faces(4, true));
        let general = store_faces(4, false);
        eprintln!("store-local guard {op:?}: store faces {small} -> {large}, general {general}");
        assert!(
            large > 0,
            "{op:?}: store copy instrumentation not exercised"
        );
        assert_eq!(small, large, "{op:?}: store grows with untouched faces");
        assert!(
            large * 2 < general,
            "{op:?}: store-local store {large} faces against the general path's {general}"
        );
    }
}

#[test]
fn store_local_cut_is_off_by_default_and_declines_whole_blank_cuts() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 2);
    let slot = make_box(&mut topo, 2.0, 6.0, 20.0).unwrap();
    let slot = placed(&mut topo, slot, &Mat4::translation(13.0, 11.0, -5.0));
    let built = BUILT.with(std::cell::Cell::get);
    let mut copy = topo.clone();
    boolean(&mut copy, BooleanOp::Cut, plate, slot).unwrap();
    assert_eq!(BUILT.with(std::cell::Cell::get), built, "ran while off");

    // A tool spanning the whole plate touches every face: nothing to save.
    let big = make_box(&mut topo, 80.0, 80.0, 4.0).unwrap();
    let big = placed(&mut topo, big, &Mat4::translation(-8.0, -8.0, 8.0));
    let mut copy = topo.clone();
    set_thread_store_local_cuts(true);
    let _ = boolean(&mut copy, BooleanOp::Cut, plate, big);
    set_thread_store_local_cuts(false);
    assert_eq!(
        BUILT.with(std::cell::Cell::get),
        built,
        "took a cut touching every face"
    );
}
