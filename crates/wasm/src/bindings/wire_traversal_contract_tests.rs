//! B16 ordered wire traversal contract.
//!
//! `getWireEdges` returns `wire.edges()` in STORED ORDER, one entry per use
//! (a seam edge used twice appears twice). `Wire` is an ordered chain of
//! oriented edges (`crates/topology/src/wire.rs`); face loops/coedges are
//! authoritative and topology-owned mutation keeps wires synchronized, so on
//! a valid solid the stored order IS the traversal. `getFaceWires` returns
//! the outer wire first, then inner wires.
//!
//! `isEdgeForwardInWire` reports the FIRST use of `edge` in the wire. A seam
//! edge used twice in one wire (cylinder lateral) gets the first use's
//! orientation for both reads. Planar faces have no seams, which is why the
//! consumer slice is scoped to them.
//!
//! Every kernel call on success paths uses the direct `#[wasm_bindgen]`
//! methods (`JsError` is only constructed on failure, so success reads are
//! native-safe). Batch (`execute_batch`) builds every fixture. There is no
//! `batch_*` companion and no new binding: this PR pins the existing
//! contract with doc fixes only.
//!
//! Fixture table:
//!
//! | Fixture | What is asserted |
//! |---|---|
//! | unit box | all 6 faces, every wire closed, 4 edges, no repeats, head-to-tail via first-use orientation, outer-first |
//! | box with through-hole | holed planar faces carry outer + inner wires, each closed and head-to-tail, no repeats |
//! | plate with pocket | holed opening plane carries outer + inner wires, each closed and head-to-tail |
//! | cylinder | caps are single closed circle edges; lateral wire holds the seam twice (documented exception) |
//! | imported STEP bored plate | every non-seam wire head-to-tail via first-use; seam wires pin the exception |
//! | rigid placement + boolean (box minus cylinder) | non-primitive wires still traverse; placement preserves order |

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashSet;

#[cfg(feature = "io")]
use crate::handles::solid_id_to_u32;
use crate::kernel::BrepKernel;

fn run(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&k.execute_batch(&json)).unwrap()
}

fn run_all_ok(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let results = run(k, ops);
    for (i, r) in results.iter().enumerate() {
        assert!(
            r.get("ok").is_some(),
            "op {i} ({}) failed: {r}",
            ops[i]["op"]
        );
    }
    results
        .into_iter()
        .map(|r| r.get("ok").cloned().unwrap())
        .collect()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn as_u32(v: &serde_json::Value) -> u32 {
    u32::try_from(v.as_u64().unwrap()).unwrap()
}

fn batch_solid(k: &mut BrepKernel, make: &str, args: serde_json::Value) -> u32 {
    as_u32(&run_all_ok(k, &[op(make, args)])[0])
}

fn rigid_transform(k: &mut BrepKernel, solid: u32, scale: f64) {
    let c = 0.37_f64.cos();
    let s = 0.37_f64.sin();
    let _ = run_all_ok(
        k,
        &[op(
            "transform",
            serde_json::json!({"solid": solid, "matrix": [
                c, 0.0, s, 17.0 * scale,
                0.0, 1.0, 0.0, -23.0 * scale,
                -s, 0.0, c, 31.0 * scale,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
}

/// Oriented `(start, end)` vertex handles for `edge` read in `forward`.
fn oriented_endpoints(k: &BrepKernel, edge: u32, forward: bool) -> (u32, u32) {
    let handles = k.get_edge_vertex_handles(edge).unwrap();
    assert_eq!(handles.len(), 2, "edge {edge} must report two vertices");
    if forward {
        (handles[0], handles[1])
    } else {
        (handles[1], handles[0])
    }
}

/// Strict contract: no repeated handles, head-to-tail via
/// `isEdgeForwardInWire`, closed back on the first vertex.
fn assert_wire_head_to_tail_via_first_use(k: &BrepKernel, wire: u32) {
    let edges = k.get_wire_edges(wire).unwrap();
    assert!(!edges.is_empty(), "wire {wire} must hold at least one edge");
    let unique: HashSet<u32> = edges.iter().copied().collect();
    assert_eq!(
        unique.len(),
        edges.len(),
        "wire {wire} repeats an edge handle — planar wires must not; seam wires use the exception path"
    );
    let mut starts = Vec::with_capacity(edges.len());
    let mut ends = Vec::with_capacity(edges.len());
    for &edge in &edges {
        let forward = k.is_edge_forward_in_wire(edge, wire).unwrap();
        let (s, e) = oriented_endpoints(k, edge, forward);
        starts.push(s);
        ends.push(e);
    }
    for i in 0..edges.len().saturating_sub(1) {
        assert_eq!(
            ends[i],
            starts[i + 1],
            "wire {wire} edge {} end must meet edge {} start in stored order",
            edges[i],
            edges[i + 1]
        );
    }
    assert!(
        k.is_wire_closed(wire).unwrap(),
        "wire {wire} must report closed"
    );
    assert_eq!(
        ends[edges.len() - 1],
        starts[0],
        "wire {wire} last edge must close back on the first"
    );
}

/// True traversal via per-use orientations from the topology. Holds even for
/// seam wires where the first-use readout is ambiguous.
fn assert_wire_true_traversal_via_topo(k: &BrepKernel, wire: u32) {
    let wire_id = k.resolve_wire(wire).unwrap();
    let wire_data = k.topo().wire(wire_id).unwrap();
    let uses = wire_data.edges();
    assert!(!uses.is_empty());
    let mut starts = Vec::with_capacity(uses.len());
    let mut ends = Vec::with_capacity(uses.len());
    for oe in uses {
        let edge_data = k.topo().edge(oe.edge()).unwrap();
        let (s, e) = if oe.is_forward() {
            (edge_data.start(), edge_data.end())
        } else {
            (edge_data.end(), edge_data.start())
        };
        starts.push(s);
        ends.push(e);
    }
    for i in 0..uses.len().saturating_sub(1) {
        assert_eq!(
            ends[i],
            starts[i + 1],
            "wire {wire} per-use traversal must connect in stored order"
        );
    }
    assert!(wire_data.is_closed());
    assert_eq!(
        ends[uses.len() - 1],
        starts[0],
        "wire {wire} per-use traversal must close"
    );
}

/// Either the strict first-use contract (no repeats) or the documented seam
/// exception (a repeated handle, true traversal still closes, binding reports
/// the first use).
fn assert_wire_contract_or_seam_exception(k: &BrepKernel, wire: u32) -> bool {
    let edges = k.get_wire_edges(wire).unwrap();
    let unique: HashSet<u32> = edges.iter().copied().collect();
    if unique.len() == edges.len() {
        assert_wire_head_to_tail_via_first_use(k, wire);
        false
    } else {
        assert_wire_true_traversal_via_topo(k, wire);
        // Pin the ambiguity: the repeated handle reads as its first use.
        let wire_id = k.resolve_wire(wire).unwrap();
        let uses = k.topo().wire(wire_id).unwrap().edges().to_vec();
        let mut seen = HashSet::new();
        let mut repeated = None;
        for oe in &uses {
            let handle = crate::handles::edge_id_to_u32(oe.edge());
            if !seen.insert(handle) {
                repeated = Some(handle);
            }
        }
        let repeated = repeated.expect("repeat detection must name a handle");
        let first = uses
            .iter()
            .find(|oe| crate::handles::edge_id_to_u32(oe.edge()) == repeated)
            .unwrap()
            .is_forward();
        assert_eq!(
            k.is_edge_forward_in_wire(repeated, wire).unwrap(),
            first,
            "repeated edge {repeated} in wire {wire} must report its first use"
        );
        true
    }
}

fn assert_all_wires_contract(k: &BrepKernel, solid: u32) -> usize {
    let faces = k.get_solid_faces(solid).unwrap();
    assert!(!faces.is_empty());
    let mut wire_count = 0;
    for &face in &faces {
        let outer = k.get_face_outer_wire(face).unwrap();
        let wires = k.get_face_wires(face).unwrap();
        assert!(!wires.is_empty());
        assert_eq!(
            wires[0], outer,
            "face {face}: getFaceWires must list the outer wire first"
        );
        for &wire in &wires {
            assert_wire_contract_or_seam_exception(k, wire);
            wire_count += 1;
        }
    }
    wire_count
}

fn holed_planar_faces(k: &BrepKernel, solid: u32) -> Vec<u32> {
    let faces = k.get_solid_faces(solid).unwrap();
    faces
        .into_iter()
        .filter(|&face| k.get_face_wires(face).unwrap().len() == 2)
        .collect()
}

fn make_through_holed_box(k: &mut BrepKernel) -> u32 {
    let plate = batch_solid(
        k,
        "makeBox",
        serde_json::json!({"width": 20.0, "height": 20.0, "depth": 6.0}),
    );
    let drill = batch_solid(
        k,
        "makeCylinder",
        serde_json::json!({"radius": 3.0, "height": 10.0}),
    );
    run_all_ok(
        k,
        &[op(
            "transform",
            serde_json::json!({"solid": drill, "matrix": [
                1.0, 0.0, 0.0, 10.0,
                0.0, 1.0, 0.0, 10.0,
                0.0, 0.0, 1.0, -2.0,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
    as_u32(
        &run_all_ok(
            k,
            &[op(
                "cut",
                serde_json::json!({"solidA": plate, "solidB": drill}),
            )],
        )[0],
    )
}

fn make_pocket_plate(k: &mut BrepKernel) -> u32 {
    let base = batch_solid(
        k,
        "makeBox",
        serde_json::json!({"width": 20.0, "height": 20.0, "depth": 10.0}),
    );
    let tool = batch_solid(
        k,
        "makeBox",
        serde_json::json!({"width": 10.0, "height": 10.0, "depth": 6.0}),
    );
    run_all_ok(
        k,
        &[op(
            "transform",
            serde_json::json!({"solid": tool, "matrix": [
                1.0, 0.0, 0.0, 5.0,
                0.0, 1.0, 0.0, 5.0,
                0.0, 0.0, 1.0, 4.0,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
    as_u32(
        &run_all_ok(
            k,
            &[op(
                "cut",
                serde_json::json!({"solidA": base, "solidB": tool}),
            )],
        )[0],
    )
}

#[test]
fn box_wires_are_ordered_closed_loops() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 10.0, "height": 10.0, "depth": 10.0}),
    );
    let faces = k.get_solid_faces(solid).unwrap();
    assert_eq!(faces.len(), 6, "unit box must expose 6 faces");
    for &face in &faces {
        let wires = k.get_face_wires(face).unwrap();
        assert_eq!(wires.len(), 1, "box face {face} has only its outer wire");
        assert_eq!(wires[0], k.get_face_outer_wire(face).unwrap());
        let edges = k.get_wire_edges(wires[0]).unwrap();
        assert_eq!(edges.len(), 4, "box face {face} outer wire holds 4 edges");
        assert_wire_head_to_tail_via_first_use(&k, wires[0]);
    }
}

#[test]
fn through_hole_faces_carry_outer_plus_inner_wires() {
    let mut k = BrepKernel::new();
    let bored = make_through_holed_box(&mut k);
    let holed = holed_planar_faces(&k, bored);
    assert!(
        holed.len() >= 2,
        "through-hole must hole at least the entry and exit planes, found {}",
        holed.len()
    );
    for &face in &holed {
        let wires = k.get_face_wires(face).unwrap();
        assert_eq!(wires.len(), 2);
        assert_eq!(wires[0], k.get_face_outer_wire(face).unwrap());
        for &wire in &wires {
            assert_wire_head_to_tail_via_first_use(&k, wire);
        }
    }
    // Every other wire on the solid still meets the contract (or seam path).
    assert_all_wires_contract(&k, bored);
}

#[test]
fn pocket_opening_plane_is_holed_and_ordered() {
    let mut k = BrepKernel::new();
    let pocket = make_pocket_plate(&mut k);
    let holed = holed_planar_faces(&k, pocket);
    assert!(
        !holed.is_empty(),
        "pocket plate must expose a holed opening plane"
    );
    for &face in &holed {
        for &wire in &k.get_face_wires(face).unwrap() {
            assert_wire_head_to_tail_via_first_use(&k, wire);
        }
    }
    assert_all_wires_contract(&k, pocket);
}

#[test]
fn cylinder_caps_are_single_closed_edges_and_seam_repeats() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeCylinder",
        serde_json::json!({"radius": 3.0, "height": 10.0}),
    );
    let faces = k.get_solid_faces(solid).unwrap();
    assert_eq!(faces.len(), 3);
    let mut caps = 0;
    let mut laterals = 0;
    for &face in &faces {
        let wires = k.get_face_wires(face).unwrap();
        assert_eq!(wires.len(), 1);
        let edges = k.get_wire_edges(wires[0]).unwrap();
        if edges.len() == 1 {
            caps += 1;
            // Single closed circle edge: start meets end, loop closes.
            let handles = k.get_edge_vertex_handles(edges[0]).unwrap();
            assert_eq!(
                handles[0], handles[1],
                "cap circle edge must start and end on one vertex"
            );
            assert!(k.is_wire_closed(wires[0]).unwrap());
            assert_wire_head_to_tail_via_first_use(&k, wires[0]);
        } else {
            laterals += 1;
            // Documented exception: the seam edge is used twice.
            assert_eq!(edges.len(), 4, "lateral wire holds rim-seam-rim-seam");
            let unique: HashSet<u32> = edges.iter().copied().collect();
            assert_eq!(
                unique.len(),
                3,
                "lateral wire must repeat exactly one handle"
            );
            assert!(k.is_wire_closed(wires[0]).unwrap());
            assert_wire_true_traversal_via_topo(&k, wires[0]);
            // Pin first-use ambiguity on the repeated handle.
            let wire_id = k.resolve_wire(wires[0]).unwrap();
            let uses = k.topo().wire(wire_id).unwrap().edges().to_vec();
            let mut counts = std::collections::HashMap::new();
            for oe in &uses {
                *counts.entry(oe.edge()).or_insert(0) += 1;
            }
            let seam = counts
                .iter()
                .find(|(_, n)| **n == 2)
                .map(|(&eid, _)| eid)
                .expect("one seam edge used twice");
            let seam_handle = crate::handles::edge_id_to_u32(seam);
            let first = uses
                .iter()
                .find(|oe| oe.edge() == seam)
                .unwrap()
                .is_forward();
            let second = uses
                .iter()
                .filter(|oe| oe.edge() == seam)
                .nth(1)
                .unwrap()
                .is_forward();
            assert_ne!(
                first, second,
                "lateral seam uses must run opposite directions"
            );
            assert_eq!(
                k.is_edge_forward_in_wire(seam_handle, wires[0]).unwrap(),
                first,
                "repeated edge must report its first use"
            );
        }
    }
    assert_eq!(caps, 2, "cylinder must expose two single-edge caps");
    assert_eq!(laterals, 1, "cylinder must expose one seamed lateral face");
}

#[test]
fn rigid_placement_preserves_wire_order() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 4.0, "height": 2.0, "depth": 1.0}),
    );
    rigid_transform(&mut k, solid, 1.0);
    assert_all_wires_contract(&k, solid);
}

#[test]
fn boolean_result_wires_traverse() {
    let mut k = BrepKernel::new();
    let stock = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 10.0, "height": 10.0, "depth": 10.0}),
    );
    let drill = batch_solid(
        &mut k,
        "makeCylinder",
        serde_json::json!({"radius": 2.0, "height": 14.0}),
    );
    run_all_ok(
        &mut k,
        &[op(
            "transform",
            serde_json::json!({"solid": drill, "matrix": [
                1.0, 0.0, 0.0, 5.0,
                0.0, 1.0, 0.0, 5.0,
                0.0, 0.0, 1.0, -2.0,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
    let bored = as_u32(
        &run_all_ok(
            &mut k,
            &[op(
                "cut",
                serde_json::json!({"solidA": stock, "solidB": drill}),
            )],
        )[0],
    );
    // A non-primitive wire: the cut introduces holed faces the primitives lack.
    assert!(!holed_planar_faces(&k, bored).is_empty());
    // Rigid placement on top of the boolean result must preserve order too.
    rigid_transform(&mut k, bored, 1.0);
    let mut saw_seam_or_holed = false;
    let faces = k.get_solid_faces(bored).unwrap();
    assert!(faces.len() > 6, "bored box must gain faces beyond the box");
    for &face in &faces {
        for &wire in &k.get_face_wires(face).unwrap() {
            if assert_wire_contract_or_seam_exception(&k, wire) {
                saw_seam_or_holed = true;
            }
            if k.get_face_wires(face).unwrap().len() == 2 {
                saw_seam_or_holed = true;
            }
        }
    }
    assert!(saw_seam_or_holed);
}

#[cfg(feature = "io")]
#[test]
fn imported_step_solid_wires_traverse() {
    let mut k = BrepKernel::new();
    let step = include_str!("../../../io/tests/data/openzcad_a_export_bored_plate.step");
    let solids = remus_io::step::reader::read_step(step, k.topo_mut()).unwrap();
    assert!(!solids.is_empty(), "STEP fixture must import solids");
    let mut holed = 0;
    for solid_id in solids {
        let solid = solid_id_to_u32(solid_id);
        for &face in &k.get_solid_faces(solid).unwrap() {
            let wires = k.get_face_wires(face).unwrap();
            if wires.len() == 2 {
                holed += 1;
            }
            for &wire in &wires {
                assert_wire_contract_or_seam_exception(&k, wire);
            }
        }
    }
    assert!(
        holed >= 1,
        "imported bored plate must expose at least one holed face"
    );
}
