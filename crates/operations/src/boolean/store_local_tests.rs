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

fn vertex_content(topo: &Topology, id: remus_topology::vertex::VertexId) -> String {
    let vertex = topo.vertex(id).unwrap();
    format!(
        "{:?} tol={:x}",
        vertex.point(),
        vertex.tolerance().to_bits()
    )
}

fn edge_content(topo: &Topology, id: remus_topology::edge::EdgeId) -> String {
    let edge = topo.edge(id).unwrap();
    let mut endpoints = [
        vertex_content(topo, edge.start()),
        vertex_content(topo, edge.end()),
    ];
    if matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
        endpoints.sort();
    }
    format!(
        "{:?} {:?} {endpoints:?} tol={:?}",
        edge.curve(),
        edge.trim(),
        edge.tolerance().map(f64::to_bits)
    )
}

/// Content of each distinct edge with all its boundary uses, and of each
/// distinct vertex with all its incident edges. Repeated uses remain repeated
/// and pcurves/winding are read from authoritative coedges. This strengthens
/// the bounded fixtures' sharing checks; it is not a general graph-isomorphism
/// proof for arbitrary coincident, indistinguishable entities.
fn boundary_authority(topo: &Topology, solid: SolidId) -> (Vec<String>, Vec<String>) {
    use std::collections::BTreeMap;

    let mut uses = BTreeMap::<_, Vec<String>>::new();
    for face_id in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        remus_topology::validation::validate_face_loops(topo, face_id).unwrap();
        let face = topo.face(face_id).unwrap();
        for (role, &loop_id) in face.boundary_loops().iter().enumerate() {
            for &use_id in topo.face_loop(loop_id).unwrap().coedges() {
                let coedge = topo.coedge(use_id).unwrap();
                let edge = topo.edge(coedge.edge()).unwrap();
                let traversal = if matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
                    let (from, to) = if coedge.is_forward() {
                        (edge.start(), edge.end())
                    } else {
                        (edge.end(), edge.start())
                    };
                    format!(
                        "{} -> {}",
                        vertex_content(topo, from),
                        vertex_content(topo, to)
                    )
                } else {
                    format!("forward={}", coedge.is_forward())
                };
                uses.entry(coedge.edge()).or_default().push(format!(
                    "{:?} reversed={} outer={} {traversal} pcurve={:?} winding={:?}",
                    face.surface(),
                    face.is_reversed(),
                    role == 0,
                    coedge.pcurve(),
                    coedge.periodic_winding()
                ));
            }
        }
    }
    let mut incident = BTreeMap::<_, Vec<String>>::new();
    let mut edges = Vec::new();
    for (id, mut edge_uses) in uses {
        edge_uses.sort();
        let content = edge_content(topo, id);
        let edge = topo.edge(id).unwrap();
        for vertex in [edge.start(), edge.end()] {
            incident.entry(vertex).or_default().push(content.clone());
        }
        edges.push(format!("{content} uses={edge_uses:?}"));
    }
    let mut vertices: Vec<_> = incident
        .into_iter()
        .map(|(id, mut edges)| {
            edges.sort();
            format!("{} incident={edges:?}", vertex_content(topo, id))
        })
        .collect();
    vertices.sort();
    edges.sort();
    (vertices, edges)
}

/// Source handles, incidence, coedges and authority must survive unchanged;
/// content-only equality would miss rewriting a source with fresh entities.
fn source_identity(topo: &Topology, solid: SolidId) -> Vec<String> {
    let mut records = vec![format!("{solid:?} {:?}", topo.solid(solid).unwrap())];
    for id in remus_topology::explorer::solid_vertices(topo, solid).unwrap() {
        records.push(format!("{id:?} {:?}", topo.vertex(id).unwrap()));
    }
    for id in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        records.push(format!("{id:?} {:?}", topo.edge(id).unwrap()));
    }
    for id in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(id).unwrap();
        records.push(format!("{id:?} {face:?}"));
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            records.push(format!("{wire:?} {:?}", topo.wire(wire).unwrap()));
        }
        for &loop_id in face.boundary_loops() {
            let boundary = topo.face_loop(loop_id).unwrap();
            records.push(format!("{loop_id:?} {boundary:?}"));
            for &use_id in boundary.coedges() {
                records.push(format!("{use_id:?} {:?}", topo.coedge(use_id).unwrap()));
            }
        }
    }
    records.sort();
    records
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
    assert_eq!(
        crate::tessellate::non_manifold_edge_count(&mesh),
        0,
        "{label}: non-manifold mesh"
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
    crate::performance::configure(Some(|| 42.0));
    let general_result = boolean(&mut general, op, blank, tool);
    let general_trace = crate::performance::drain();
    crate::performance::configure(None);
    let general_result = general_result.unwrap();
    assert!(general_trace.iter().all(|phase| !matches!(
        phase.name,
        "boolean.route.store_local_attempt" | "boolean.route.store_local_success"
    )));
    let mut local = topo.clone();
    let built = BUILT.with(std::cell::Cell::get);
    #[cfg(feature = "perf-counters")]
    let _ = crate::performance::take_boolean_routes();
    crate::performance::configure(Some(|| 42.0));
    set_thread_store_local_cuts(true);
    let local_result = boolean(&mut local, op, blank, tool);
    set_thread_store_local_cuts(false);
    let local_trace = crate::performance::drain();
    crate::performance::configure(None);
    let local_result = local_result.unwrap();
    let routes: Vec<_> = local_trace
        .iter()
        .filter(|phase| phase.name.starts_with("boolean.route."))
        .collect();
    assert_eq!(
        routes.iter().map(|phase| phase.name).collect::<Vec<_>>(),
        [
            "boolean.route.store_local_attempt",
            "boolean.route.store_local_success"
        ],
        "{label}: trace does not prove local completion"
    );
    assert!(
        routes
            .iter()
            .all(|phase| phase.start_ms.to_bits() == 42.0_f64.to_bits()
                && phase.duration_ms.to_bits() == 0.0_f64.to_bits())
    );
    #[cfg(feature = "perf-counters")]
    {
        let routes = crate::performance::take_boolean_routes();
        assert_eq!(routes.store_local_successes, 1, "{label}: {routes:?}");
        assert_eq!(routes.store_local_attempts, 1, "{label}: {routes:?}");
        assert_eq!(routes.general_gfa_calls, 0, "{label}: {routes:?}");
    }
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
    assert_eq!(
        boundary_authority(&local, local_result),
        boundary_authority(&general, general_result),
        "{label}: edge/vertex authority or boundary sharing differs"
    );
    for source in [blank, tool] {
        assert_eq!(
            source_identity(&local, source),
            source_identity(topo, source),
            "{label}: source identity or authority changed"
        );
        let source_edges = remus_topology::explorer::solid_edges(topo, source).unwrap();
        let source_vertices = remus_topology::explorer::solid_vertices(topo, source).unwrap();
        assert!(
            remus_topology::explorer::solid_edges(&local, local_result)
                .unwrap()
                .iter()
                .all(|id| !source_edges.contains(id)),
            "{label}: result aliases a source edge"
        );
        assert!(
            remus_topology::explorer::solid_vertices(&local, local_result)
                .unwrap()
                .iter()
                .all(|id| !source_vertices.contains(id)),
            "{label}: result aliases a source vertex"
        );
    }
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
    assert_sound(&general, general_result, label);
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

/// Membership checks are independent of the differential geometry dump: two
/// equally wrong solids must not qualify merely because their volumes match.
#[test]
fn store_local_material_survives_scale_and_placement() {
    use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid};
    use remus_math::vec::Point3;

    let cases = [
        (
            BooleanOp::Cut,
            (2.0, 6.0, 20.0),
            (13.0, 11.0, -5.0),
            [
                ((14.0, 14.0, 5.0), PointClassification::Outside),
                ((30.0, 12.0, 5.0), PointClassification::Inside),
                ((8.0, 8.0, 5.0), PointClassification::Outside),
                ((13.0, 14.0, 5.0), PointClassification::OnBoundary),
            ],
        ),
        (
            BooleanOp::Fuse,
            (6.0, 3.0, 4.0),
            (11.0, 12.5, 8.0),
            [
                ((14.0, 14.0, 11.0), PointClassification::Inside),
                ((14.0, 14.0, 13.0), PointClassification::Outside),
                ((8.0, 8.0, 5.0), PointClassification::Outside),
                ((17.0, 14.0, 11.0), PointClassification::OnBoundary),
            ],
        ),
        (
            BooleanOp::Intersect,
            (20.0, 20.0, 20.0),
            (4.0, 4.0, -5.0),
            [
                ((14.0, 14.0, 5.0), PointClassification::Inside),
                ((30.0, 12.0, 5.0), PointClassification::Outside),
                ((8.0, 8.0, 5.0), PointClassification::Outside),
                ((4.0, 14.0, 5.0), PointClassification::OnBoundary),
            ],
        ),
    ];
    for scale in [0.5, 1.0, 2.0] {
        let matrix = Mat4::translation(128.0, -64.0, 32.0) * Mat4::scale(scale, scale, scale);
        for (op, (dx, dy, dz), (x, y, z), probes) in cases {
            let mut topo = Topology::new();
            let plate = drilled_plate(&mut topo, 3);
            let tool = make_box(&mut topo, dx, dy, dz).unwrap();
            let tool = placed(&mut topo, tool, &Mat4::translation(x, y, z));
            placed(&mut topo, plate, &matrix);
            placed(&mut topo, tool, &matrix);
            let label = format!("{op:?} scale={scale}");
            let (local, result) = assert_equivalent_op(&topo, op, plate, tool, &label);
            let prepared = PreparedSolid::prepare(&local, result).unwrap();
            let options = ClassifyOptions {
                tolerance: scale * 1e-7,
                ..ClassifyOptions::default()
            };
            for ((x, y, z), expected) in probes {
                let point = matrix.mul_point(Point3::new(x, y, z));
                assert_eq!(
                    prepared.classify_point(point, &options).unwrap(),
                    expected,
                    "{label}: {point:?}"
                );
            }
        }
    }
}

fn assert_direct_decline_leaves_inputs_unchanged(
    topo: &mut Topology,
    blank: SolidId,
    tool: SolidId,
    label: &str,
) {
    let before = format!("{topo:?}");
    let context = remus_math::context::OperationContext::new();
    let built = BUILT.with(std::cell::Cell::get);
    for op in [BooleanOp::Cut, BooleanOp::Fuse, BooleanOp::Intersect] {
        #[cfg(feature = "perf-counters")]
        let _ = crate::performance::take_boolean_routes();
        crate::performance::configure(Some(|| 42.0));
        set_thread_store_local_cuts(true);
        let attempt = super::store_local::try_store_local_boolean(topo, op, blank, tool, &context);
        set_thread_store_local_cuts(false);
        let trace = crate::performance::drain();
        crate::performance::configure(None);
        assert_eq!(
            trace.iter().map(|phase| phase.name).collect::<Vec<_>>(),
            ["boolean.route.store_local_attempt"],
            "{label}: {op:?} decline claimed a successful route"
        );
        #[cfg(feature = "perf-counters")]
        {
            let routes = crate::performance::take_boolean_routes();
            assert_eq!(routes.store_local_attempts, 1, "{label}: {op:?} {routes:?}");
            assert_eq!(
                routes.store_local_successes, 0,
                "{label}: {op:?} {routes:?}"
            );
        }
        assert!(
            attempt.unwrap().is_none(),
            "{label}: {op:?} did not decline"
        );
        assert_eq!(BUILT.with(std::cell::Cell::get), built, "{label}: {op:?}");
        assert_eq!(
            format!("{topo:?}"),
            before,
            "{label}: {op:?} changed input state"
        );
    }
}

#[test]
fn cavity_blank_declines_without_mutating_inputs() {
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let inner = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let inner = placed(&mut topo, inner, &Mat4::translation(4.0, 4.0, 4.0));
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, inner).unwrap();
    assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 1);
    let tool = make_box(&mut topo, 1.0, 1.0, 20.0).unwrap();
    let tool = placed(&mut topo, tool, &Mat4::translation(1.0, 1.0, -5.0));
    assert_direct_decline_leaves_inputs_unchanged(&mut topo, hollow, tool, "cavity");
}

#[test]
fn nurbs_patch_declines_without_mutating_inputs() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    crate::heal::convert_to_bspline(&mut topo, blank).unwrap();
    assert!(
        remus_topology::explorer::solid_faces(&topo, blank)
            .unwrap()
            .iter()
            .all(|&face| matches!(
                topo.face(face).unwrap().surface(),
                remus_topology::face::FaceSurface::Nurbs(_)
            ))
    );
    let tool = make_box(&mut topo, 2.0, 2.0, 20.0).unwrap();
    let tool = placed(&mut topo, tool, &Mat4::translation(4.0, 4.0, -5.0));
    assert_direct_decline_leaves_inputs_unchanged(&mut topo, blank, tool, "NURBS patch");
}

#[test]
fn cancelled_store_local_boolean_keeps_geometry_and_history() {
    use remus_math::context::{CancellationToken, FallbackPolicy, OperationContext};

    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    let tool = make_box(&mut topo, 2.0, 6.0, 20.0).unwrap();
    let tool = placed(&mut topo, tool, &Mat4::translation(13.0, 11.0, -5.0));
    let input = (canonical(&topo, plate), canonical(&topo, tool));
    let history = format!("{:?}", topo.journal());
    let counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    );
    let cancellation = CancellationToken::new();
    cancellation.cancel();
    let context = OperationContext::new()
        .with_fallback(FallbackPolicy::ExactOnly)
        .with_cancellation(cancellation);
    set_thread_store_local_cuts(true);
    let refused = super::boolean_with_context(&mut topo, BooleanOp::Cut, plate, tool, &context);
    set_thread_store_local_cuts(false);
    assert!(matches!(
        refused,
        Err(crate::OperationsError::Math(
            remus_math::MathError::Cancelled
        ))
    ));
    assert_eq!((canonical(&topo, plate), canonical(&topo, tool)), input);
    assert_eq!(format!("{:?}", topo.journal()), history);
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_solids()
        ),
        counts
    );
    // Cancellation does not leave a sticky failure or consume the operands.
    assert_equivalent(&topo, plate, tool, "after cancellation");
}

#[test]
fn entity_evolution_keeps_the_general_path_when_store_local_is_enabled() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    let tool = make_box(&mut topo, 2.0, 6.0, 20.0).unwrap();
    let tool = placed(&mut topo, tool, &Mat4::translation(13.0, 11.0, -5.0));
    let mut general = topo.clone();
    let (expected, evolution) =
        super::boolean_with_entity_evolution(&mut general, BooleanOp::Cut, plate, tool).unwrap();
    let built = BUILT.with(std::cell::Cell::get);
    set_thread_store_local_cuts(true);
    let actual = super::boolean_with_entity_evolution(&mut topo, BooleanOp::Cut, plate, tool);
    set_thread_store_local_cuts(false);
    let (actual, actual_evolution) = actual.unwrap();
    assert_eq!(
        BUILT.with(std::cell::Cell::get),
        built,
        "evolution bypassed the general path"
    );
    assert_eq!(actual, expected, "evolution output numbering changed");
    assert_eq!(canonical(&topo, actual), canonical(&general, expected));
    assert_eq!(format!("{actual_evolution:?}"), format!("{evolution:?}"));
}

#[test]
fn cross_drilled_shaft_preserves_material_with_store_local_requested() {
    use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid};
    use remus_math::vec::Point3;

    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, 3.0, 12.0).unwrap();
    let tool = make_cylinder(&mut topo, 1.0, 10.0).unwrap();
    let tool = placed(
        &mut topo,
        tool,
        &(Mat4::translation(-5.0, 0.0, 6.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2)),
    );
    let source = (canonical(&topo, shaft), canonical(&topo, tool));
    let mut general = topo.clone();
    let expected = boolean(&mut general, BooleanOp::Cut, shaft, tool).unwrap();
    set_thread_store_local_cuts(true);
    let result = boolean(&mut topo, BooleanOp::Cut, shaft, tool);
    set_thread_store_local_cuts(false);
    let result = result.unwrap();
    // A request is not proof of locality: this case may decline to the
    // general path. It still must preserve the geometry and material.
    assert_eq!(canonical(&topo, result), canonical(&general, expected));
    assert_eq!(counts(&topo, result), counts(&general, expected));
    assert_eq!(
        line_tolerance_inventory(&topo, result, false),
        line_tolerance_inventory(&general, expected, false)
    );
    assert_eq!((canonical(&topo, shaft), canonical(&topo, tool)), source);
    assert_sound(&topo, result, "cross-drilled shaft");
    let prepared = PreparedSolid::prepare(&topo, result).unwrap();
    for (point, expected) in [
        (Point3::new(0.0, 0.0, 6.0), PointClassification::Outside),
        (Point3::new(0.0, 2.0, 6.0), PointClassification::Inside),
        (Point3::new(0.0, 0.0, 3.0), PointClassification::Inside),
        (Point3::new(0.0, 4.0, 6.0), PointClassification::Outside),
    ] {
        assert_eq!(
            prepared
                .classify_point(point, &ClassifyOptions::default())
                .unwrap(),
            expected,
            "{point:?}"
        );
    }
}

fn line_tolerance_inventory(
    topo: &Topology,
    solid: SolidId,
    stored: bool,
) -> Vec<(String, Option<u64>)> {
    let mut lines: Vec<_> = remus_topology::explorer::solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|id| {
            let edge = topo.edge(id).unwrap();
            if !matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
                return None;
            }
            let start = topo.vertex(edge.start()).unwrap();
            let end = topo.vertex(edge.end()).unwrap();
            let mut endpoints = [format!("{:?}", start.point()), format!("{:?}", end.point())];
            endpoints.sort();
            let tolerance = if stored {
                edge.tolerance()
            } else {
                Some(edge.effective_tolerance(start.tolerance().max(end.tolerance())))
            };
            Some((format!("{endpoints:?}"), tolerance.map(f64::to_bits)))
        })
        .collect();
    lines.sort();
    lines
}

/// Exact stored authority is a stronger contract than current material and
/// effective tube equality. The rebuilt seam must retain the GFA's explicit
/// tolerance instead of substituting the blank's inherited tolerance.
#[test]
fn cross_drilled_store_local_preserves_stored_line_tolerances() {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, 3.0, 12.0).unwrap();
    let tool = make_cylinder(&mut topo, 1.0, 10.0).unwrap();
    let tool = placed(
        &mut topo,
        tool,
        &(Mat4::translation(-5.0, 0.0, 6.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2)),
    );
    let mut general = topo.clone();
    let expected = boolean(&mut general, BooleanOp::Cut, shaft, tool).unwrap();
    let built = BUILT.with(std::cell::Cell::get);
    set_thread_store_local_cuts(true);
    let result = boolean(&mut topo, BooleanOp::Cut, shaft, tool);
    set_thread_store_local_cuts(false);
    let result = result.unwrap();
    assert_eq!(
        BUILT.with(std::cell::Cell::get),
        built + 1,
        "the stored-authority witness must actually complete store-locally"
    );
    assert_eq!(
        line_tolerance_inventory(&topo, result, false),
        line_tolerance_inventory(&general, expected, false),
        "effective tolerance changed too"
    );
    assert_eq!(
        line_tolerance_inventory(&topo, result, true),
        line_tolerance_inventory(&general, expected, true),
        "stored tolerance authority changed"
    );
}

#[test]
fn cross_drilled_authority_and_sharing_survive_nondefault_source_edge_tolerances() {
    for (edge_tolerance, vertex_tolerance) in [(None, 1e-7), (Some(1e-7), 1e-7), (Some(5e-5), 1e-7)]
    {
        let mut topo = Topology::new();
        let shaft = make_cylinder(&mut topo, 3.0, 12.0).unwrap();
        for id in remus_topology::explorer::solid_edges(&topo, shaft).unwrap() {
            if matches!(
                topo.edge(id).unwrap().curve(),
                remus_topology::edge::EdgeCurve::Line
            ) {
                topo.edge_mut(id)
                    .unwrap()
                    .set_tolerance(edge_tolerance)
                    .unwrap();
            }
        }
        for id in remus_topology::explorer::solid_vertices(&topo, shaft).unwrap() {
            topo.vertex_mut(id)
                .unwrap()
                .set_tolerance(vertex_tolerance)
                .unwrap();
        }
        let tool = make_cylinder(&mut topo, 1.0, 10.0).unwrap();
        let tool = placed(
            &mut topo,
            tool,
            &(Mat4::translation(-5.0, 0.0, 6.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2)),
        );
        assert_equivalent(
            &topo,
            shaft,
            tool,
            &format!("cross-drilled edge={edge_tolerance:?} vertex={vertex_tolerance}"),
        );
    }
}

#[test]
fn raised_boundary_vertex_authority_declines_without_mutation_then_falls_back_exactly() {
    use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid};
    use remus_math::vec::Point3;

    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, 3.0, 12.0).unwrap();
    for id in remus_topology::explorer::solid_vertices(&topo, shaft).unwrap() {
        topo.vertex_mut(id).unwrap().set_tolerance(7e-6).unwrap();
    }
    let tool = make_cylinder(&mut topo, 1.0, 10.0).unwrap();
    let tool = placed(
        &mut topo,
        tool,
        &(Mat4::translation(-5.0, 0.0, 6.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2)),
    );
    let before = format!("{topo:?}");
    let sources = [
        (shaft, source_identity(&topo, shaft)),
        (tool, source_identity(&topo, tool)),
    ];
    let built = BUILT.with(std::cell::Cell::get);
    set_thread_store_local_cuts(true);
    let attempt = super::store_local::try_store_local_boolean(
        &mut topo,
        BooleanOp::Cut,
        shaft,
        tool,
        &remus_math::context::OperationContext::new(),
    );
    set_thread_store_local_cuts(false);
    assert!(
        attempt.unwrap().is_none(),
        "vertex authority mismatch was accepted"
    );
    assert_eq!(format!("{topo:?}"), before, "decline changed source state");
    assert_eq!(BUILT.with(std::cell::Cell::get), built);

    let mut general = topo.clone();
    let expected = boolean(&mut general, BooleanOp::Cut, shaft, tool).unwrap();
    crate::performance::configure(Some(|| 42.0));
    set_thread_store_local_cuts(true);
    let actual = boolean(&mut topo, BooleanOp::Cut, shaft, tool);
    set_thread_store_local_cuts(false);
    let trace = crate::performance::drain();
    crate::performance::configure(None);
    let actual = actual.unwrap();
    assert_eq!(
        trace
            .iter()
            .filter(|phase| phase.name.starts_with("boolean.route."))
            .map(|phase| phase.name)
            .collect::<Vec<_>>(),
        [
            "boolean.route.store_local_attempt",
            "boolean.route.general_gfa"
        ]
    );
    assert_eq!(BUILT.with(std::cell::Cell::get), built);
    assert_eq!(canonical(&topo, actual), canonical(&general, expected));
    assert_eq!(counts(&topo, actual), counts(&general, expected));
    assert_eq!(
        boundary_authority(&topo, actual),
        boundary_authority(&general, expected)
    );
    for (source, before) in sources {
        assert_eq!(source_identity(&topo, source), before);
        assert_eq!(source_identity(&general, source), before);
    }
    assert_sound(&topo, actual, "raised-vertex fallback");
    let prepared = PreparedSolid::prepare(&topo, actual).unwrap();
    for (point, expected) in [
        (Point3::new(0.0, 0.0, 6.0), PointClassification::Outside),
        (Point3::new(0.0, 2.0, 6.0), PointClassification::Inside),
    ] {
        assert_eq!(
            prepared
                .classify_point(point, &ClassifyOptions::default())
                .unwrap(),
            expected
        );
    }
}

#[test]
fn kept_boundary_preserves_inherited_and_nondefault_explicit_edge_authority() {
    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    let mut marked = 0;
    for id in remus_topology::explorer::solid_edges(&topo, plate).unwrap() {
        let edge = topo.edge(id).unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        if matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line)
            && (end - start).length() > 60.0
        {
            // Outer plate edges join the touched top/bottom to kept side
            // walls. Leave half inherited and give half explicit authority.
            let tolerance = if marked % 2 == 0 { None } else { Some(5e-5) };
            topo.edge_mut(id).unwrap().set_tolerance(tolerance).unwrap();
            marked += 1;
        }
    }
    assert_eq!(marked, 8, "fixture must cover both rims' outer boundaries");
    let tool = make_box(&mut topo, 2.0, 6.0, 20.0).unwrap();
    let tool = placed(&mut topo, tool, &Mat4::translation(13.0, 11.0, -5.0));
    let (local, result) = assert_equivalent(&topo, plate, tool, "mixed boundary authority");
    let outer_lines: Vec<_> = remus_topology::explorer::solid_edges(&local, result)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = local.edge(id).unwrap();
            matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line)
                && (local.vertex(edge.end()).unwrap().point()
                    - local.vertex(edge.start()).unwrap().point())
                .length()
                    > 60.0
        })
        .map(|id| local.edge(id).unwrap().tolerance().map(f64::to_bits))
        .collect();
    assert_eq!(
        outer_lines.iter().filter(|value| value.is_none()).count(),
        4
    );
    assert_eq!(
        outer_lines
            .iter()
            .filter(|&&value| value == Some(5e-5_f64.to_bits()))
            .count(),
        4
    );
}

#[test]
fn kept_periodic_boundary_retains_pcurves_winding_and_shared_entities() {
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::vec::{Point2, Vec2};
    use remus_topology::coedge::PeriodicWinding;
    use remus_topology::pcurve::PCurve;

    let mut topo = Topology::new();
    let plate = drilled_plate(&mut topo, 3);
    let face_id = remus_topology::explorer::solid_faces(&topo, plate)
        .unwrap()
        .into_iter()
        .find(|&id| {
            matches!(
                topo.face(id).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .unwrap();
    let face = topo.face(face_id).unwrap();
    let surface = face.surface().clone();
    let seam_uses: Vec<_> = face
        .boundary_loops()
        .iter()
        .flat_map(|&id| topo.face_loop(id).unwrap().coedges().iter().copied())
        .filter(|&id| {
            matches!(
                topo.edge(topo.coedge(id).unwrap().edge()).unwrap().curve(),
                remus_topology::edge::EdgeCurve::Line
            )
        })
        .collect();
    assert_eq!(seam_uses.len(), 2, "fixture needs both seam uses");
    assert_eq!(
        topo.coedge(seam_uses[0]).unwrap().edge(),
        topo.coedge(seam_uses[1]).unwrap().edge(),
        "fixture needs one shared seam edge"
    );
    for (branch, use_id) in [0_i32, 1].into_iter().zip(seam_uses) {
        let coedge = topo.coedge(use_id).unwrap();
        let edge = topo.edge(coedge.edge()).unwrap();
        let (u, start) = surface
            .project_point(topo.vertex(edge.start()).unwrap().point())
            .unwrap();
        let (_, end) = surface
            .project_point(topo.vertex(edge.end()).unwrap().point())
            .unwrap();
        let (start, end) = if coedge.is_forward() {
            (start, end)
        } else {
            (end, start)
        };
        let curve = Line2D::new(
            Point2::new(u + f64::from(branch) * std::f64::consts::TAU, 0.0),
            Vec2::new(0.0, 1.0),
        )
        .unwrap();
        topo.set_coedge_pcurve(use_id, PCurve::new(Curve2D::Line(curve), start, end))
            .unwrap();
        topo.set_coedge_periodic_winding(use_id, PeriodicWinding::new(branch, 0))
            .unwrap();
    }
    let tool = make_box(&mut topo, 2.0, 6.0, 20.0).unwrap();
    let tool = placed(&mut topo, tool, &Mat4::translation(50.0, 50.0, -5.0));
    let (local, result) = assert_equivalent(&topo, plate, tool, "kept periodic boundary");
    let pcurve_uses = remus_topology::explorer::solid_faces(&local, result)
        .unwrap()
        .into_iter()
        .flat_map(|id| local.face(id).unwrap().boundary_loops().iter().copied())
        .flat_map(|id| local.face_loop(id).unwrap().coedges().iter().copied())
        .filter(|&id| local.coedge(id).unwrap().pcurve().is_some())
        .count();
    assert!(pcurve_uses >= 2, "kept seam authority was discarded");
}
