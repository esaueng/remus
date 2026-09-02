//! Structured fuzzing of the topology-mutation contracts (RFC 0002/0003).
//!
//! Every accepted input builds a bounded box solid — six planar faces, twelve
//! edges, eight vertices, volume `dx * dy * dz` known by construction — and
//! then runs a short, byte-driven sequence of topology mutations over it:
//!
//! * authoritative face-loop/coedge identity (`build_face_loops` is
//!   read-only on a derived face) and retirement through the sanctioned
//!   wire replacement (`replace_boundary_wire`),
//! * validated mutation rollback after a wire is deliberately broken
//!   (`run_validated` + `validate_wire_closed`),
//! * rollback of staged allocations (`run_transacted`),
//! * rollback of an in-transaction wire replacement or solid deletion (the
//!   full-undo half of the transaction contract),
//! * checkpoint restoration preserving stale-handle safety
//!   (`restore_preserving_handle_slots` — the tombstone barrier: window
//!   retirements stick and the derivation map must never dangle),
//! * deletion of an unreferenced solid (`delete_solid`), and
//! * deletion refused because a compound or comp-solid still references the
//!   solid.
//!
//! The value is entirely in the oracles, not in reaching a panic. After every
//! mutation the solid must still pass comprehensive validation and the
//! closed-manifold census, and its measured volume must still equal the
//! closed form. Replaced loop/coedge handles and handles leaked from a
//! rolled-back transaction must fail typed lookups forever and never be
//! reissued. A transaction rollback must reproduce the *exact* live public
//! topology state — every live entity's contents, id included; a checkpoint
//! restore must reproduce it exactly *except* that retirements staged in
//! the window stay retired, with no dangling derivation left behind. A
//! refused deletion must leave no partial mutation; an accepted one must
//! retire the complete unshared topology tree and nothing else — a second
//! guard box, and a compound referencing only that guard, must survive
//! untouched.

#![no_main]

use std::collections::BTreeMap;
use std::collections::BTreeSet;

use libfuzzer_sys::fuzz_target;

mod invariants;

use invariants as inv;
use remus_math::vec::Point3;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::validate::validate_solid;
use remus_topology::compound::Compound;
use remus_topology::compsolid::CompSolid;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer;
use remus_topology::transaction::{run_transacted, run_validated};
use remus_topology::validation::{validate_face_loops, validate_wire_closed};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{DeleteSolidError, LoopId, SolidId, Topology, TopologyError};

/// Longest mutation sequence one input may request.
const MAX_OPS: usize = 8;

struct Bytes<'a> {
    data: &'a [u8],
    index: usize,
}

impl<'a> Bytes<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, index: 0 }
    }

    fn next(&mut self) -> u8 {
        if self.data.is_empty() {
            return 0;
        }
        let value = self.data[self.index % self.data.len()];
        self.index += 1;
        value
    }

    fn unit(&mut self) -> f64 {
        f64::from(self.next()) / f64::from(u8::MAX)
    }

    fn signed(&mut self) -> f64 {
        self.unit().mul_add(2.0, -1.0)
    }

    /// A dimension in `[0.25, 4]`, quantized to sixteenths so the closed-form
    /// volume below is exact in binary floating point.
    fn dim(&mut self) -> f64 {
        ((0.25 + self.unit() * 3.75) * 16.0).round() / 16.0
    }
}

/// The exact public topology state a rollback, restore, or refused deletion
/// must reproduce: live counts, plus every live entity's contents keyed by
/// its stable handle index. Ids are part of the digest on purpose — a
/// restore must bring back the *same* live entities, not equivalent ones.
#[derive(Debug, PartialEq)]
struct LiveState {
    counts: [usize; 10],
    vertices: Vec<(usize, [u64; 4])>,
    edges: Vec<(usize, (usize, usize, &'static str))>,
    wires: Vec<(usize, (bool, Vec<(usize, bool)>))>,
    faces: Vec<(usize, (usize, Vec<usize>, &'static str, bool))>,
    shells: Vec<(usize, Vec<usize>)>,
    solids: Vec<(usize, (usize, Vec<usize>))>,
    compounds: Vec<(usize, Vec<usize>)>,
    compsolids: Vec<(usize, (Vec<usize>, Vec<usize>))>,
    loops: Vec<(usize, (usize, bool, Vec<usize>))>,
    coedges: Vec<(usize, (usize, bool, usize))>,
    face_loops: Vec<(usize, Vec<usize>)>,
}

fn counts(topo: &Topology) -> [usize; 10] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_compounds(),
        topo.num_compsolids(),
        topo.num_loops(),
        topo.num_coedges(),
    ]
}

fn live_state(topo: &Topology) -> LiveState {
    let vertices = topo
        .vertices()
        .iter()
        .map(|(id, v)| {
            let p = v.point();
            (
                id.index(),
                [
                    p.x().to_bits(),
                    p.y().to_bits(),
                    p.z().to_bits(),
                    v.tolerance().to_bits(),
                ],
            )
        })
        .collect();
    let edges = topo
        .edges()
        .iter()
        .map(|(id, e)| {
            (
                id.index(),
                (e.start().index(), e.end().index(), e.curve().type_tag()),
            )
        })
        .collect();
    let wires = topo
        .wires()
        .iter()
        .map(|(id, w)| {
            (
                id.index(),
                (
                    w.is_closed(),
                    w.edges()
                        .iter()
                        .map(|oe| (oe.edge().index(), oe.is_forward()))
                        .collect(),
                ),
            )
        })
        .collect();
    let mut face_loops = Vec::new();
    let mut loops = Vec::new();
    let mut coedges = Vec::new();
    let faces = topo
        .faces()
        .iter()
        .map(|(id, f)| {
            if let Some(loop_ids) = topo.loops_of_face(id) {
                face_loops.push((id.index(), loop_ids.iter().map(|id| id.index()).collect()));
                for &loop_id in loop_ids {
                    let boundary = topo
                        .face_loop(loop_id)
                        .unwrap_or_else(|error| panic!("registered loop lookup failed: {error}"));
                    loops.push((
                        loop_id.index(),
                        (
                            boundary.face().index(),
                            boundary.is_closed(),
                            boundary.coedges().iter().map(|id| id.index()).collect(),
                        ),
                    ));
                    for &coedge_id in boundary.coedges() {
                        let coedge = topo
                            .coedge(coedge_id)
                            .unwrap_or_else(|error| panic!("loop coedge lookup failed: {error}"));
                        coedges.push((
                            coedge_id.index(),
                            (
                                coedge.edge().index(),
                                coedge.is_forward(),
                                coedge.parent_loop().index(),
                            ),
                        ));
                    }
                }
            }
            (
                id.index(),
                (
                    f.outer_wire().index(),
                    f.inner_wires().iter().map(|w| w.index()).collect(),
                    f.surface().type_tag(),
                    f.is_reversed(),
                ),
            )
        })
        .collect();
    let shells = topo
        .shells()
        .iter()
        .map(|(id, s)| (id.index(), s.faces().iter().map(|f| f.index()).collect()))
        .collect();
    let solids = topo
        .solids()
        .iter()
        .map(|(id, s)| {
            (
                id.index(),
                (
                    s.outer_shell().index(),
                    s.inner_shells().iter().map(|sh| sh.index()).collect(),
                ),
            )
        })
        .collect();
    let compounds = topo
        .compounds()
        .iter()
        .map(|(id, c)| (id.index(), c.solids().iter().map(|s| s.index()).collect()))
        .collect();
    let compsolids = topo
        .compsolids()
        .iter()
        .map(|(id, c)| {
            (
                id.index(),
                (
                    c.solids().iter().map(|s| s.index()).collect(),
                    c.shared_faces().iter().map(|f| f.index()).collect(),
                ),
            )
        })
        .collect();
    LiveState {
        counts: counts(topo),
        vertices,
        edges,
        wires,
        faces,
        shells,
        solids,
        compounds,
        compsolids,
        loops,
        coedges,
        face_loops,
    }
}

/// The full entity tree of one solid, collected by an independent walk
/// (solid → shells → faces → wires → edges → vertices, plus derived loops
/// and coedges). The deletion oracles compare arena counts against these
/// sets exactly.
#[derive(Debug, Default)]
struct Tree {
    shells: BTreeSet<usize>,
    faces: BTreeSet<usize>,
    wires: BTreeSet<usize>,
    edges: BTreeSet<usize>,
    vertices: BTreeSet<usize>,
    loops: BTreeSet<usize>,
    coedges: BTreeSet<usize>,
}

fn collect_tree(topo: &Topology, solid: SolidId) -> Tree {
    let mut tree = Tree::default();
    let solid_data = topo
        .solid(solid)
        .unwrap_or_else(|error| panic!("tree walk hit a stale solid: {error}"));
    for shell_id in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        tree.shells.insert(shell_id.index());
        let shell = topo
            .shell(shell_id)
            .unwrap_or_else(|error| panic!("tree walk hit a stale shell: {error}"));
        for &face_id in shell.faces() {
            tree.faces.insert(face_id.index());
            let face = topo
                .face(face_id)
                .unwrap_or_else(|error| panic!("tree walk hit a stale face: {error}"));
            for wire_id in
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                tree.wires.insert(wire_id.index());
                let wire = topo
                    .wire(wire_id)
                    .unwrap_or_else(|error| panic!("tree walk hit a stale wire: {error}"));
                for oriented in wire.edges() {
                    let edge_id = oriented.edge();
                    tree.edges.insert(edge_id.index());
                    let edge = topo
                        .edge(edge_id)
                        .unwrap_or_else(|error| panic!("tree walk hit a stale edge: {error}"));
                    tree.vertices.insert(edge.start().index());
                    tree.vertices.insert(edge.end().index());
                }
            }
            if let Some(loop_ids) = topo.loops_of_face(face_id) {
                for &loop_id in loop_ids {
                    tree.loops.insert(loop_id.index());
                    let boundary = topo
                        .face_loop(loop_id)
                        .unwrap_or_else(|error| panic!("tree walk hit a stale loop: {error}"));
                    for &coedge_id in boundary.coedges() {
                        tree.coedges.insert(coedge_id.index());
                    }
                }
            }
        }
    }
    tree
}

/// Re-derive a face's loops from its wires and require the stored
/// derivation to match exactly: loop order, closure, coedge order, edge
/// identity, orientation, and parent links.
fn assert_loop_derivation(topo: &Topology, face_id: remus_topology::FaceId, loops: &[LoopId]) {
    let face = topo
        .face(face_id)
        .unwrap_or_else(|error| panic!("derived face lookup failed: {error}"));
    let mut wire_ids = vec![face.outer_wire()];
    wire_ids.extend(face.inner_wires().iter().copied());
    assert_eq!(
        loops.len(),
        wire_ids.len(),
        "derivation produced {} loops for {} wires",
        loops.len(),
        wire_ids.len(),
    );
    let registered = topo
        .loops_of_face(face_id)
        .unwrap_or_else(|| panic!("derivation left no registered loops"));
    assert_eq!(
        registered, loops,
        "registered loops differ from the returned handles"
    );
    for (&loop_id, &wire_id) in loops.iter().zip(&wire_ids) {
        let boundary = topo
            .face_loop(loop_id)
            .unwrap_or_else(|error| panic!("derived loop lookup failed: {error}"));
        let wire = topo
            .wire(wire_id)
            .unwrap_or_else(|error| panic!("derived wire lookup failed: {error}"));
        assert!(
            boundary.face() == face_id
                && boundary.is_closed() == wire.is_closed()
                && boundary.coedges().len() == wire.edges().len(),
            "loop/wire shape mismatch: loop {boundary:?} vs wire {wire:?}",
        );
        for (&coedge_id, oriented) in boundary.coedges().iter().zip(wire.edges()) {
            let coedge = topo
                .coedge(coedge_id)
                .unwrap_or_else(|error| panic!("derived coedge lookup failed: {error}"));
            assert!(
                coedge.edge() == oriented.edge()
                    && coedge.is_forward() == oriented.is_forward()
                    && coedge.parent_loop() == loop_id,
                "coedge {coedge:?} does not reproduce its wire use {oriented:?}",
            );
        }
    }
    validate_face_loops(topo, face_id)
        .unwrap_or_else(|error| panic!("derivation failed its own checker: {error}"));
}

/// The authoritative loops of one live face. A face built through
/// `add_face` always has them; a missing derivation is a broken contract.
fn authoritative_loops(topo: &Topology, face_id: remus_topology::FaceId) -> Vec<LoopId> {
    topo.loops_of_face(face_id)
        .map(<[LoopId]>::to_vec)
        .unwrap_or_else(|| panic!("live face {face_id:?} has no authoritative loops"))
}

/// The registered derivations of every face of `solid`, keyed by face slot.
fn registered_derivations(topo: &Topology, solid: SolidId) -> BTreeMap<usize, Vec<LoopId>> {
    explorer::solid_faces(topo, solid)
        .unwrap_or_else(|error| panic!("live solid has no faces: {error}"))
        .into_iter()
        .map(|face_id| (face_id.index(), authoritative_loops(topo, face_id)))
        .collect()
}

/// Replace a face's outer wire with an identical copy through the sanctioned
/// mutation path. The boundary is unchanged; what changes is the derivation:
/// the previous Loop/Coedge handles are retired and fresh ones issued.
fn replace_outer_wire_in_place(
    topo: &mut Topology,
    face_id: remus_topology::FaceId,
) -> Result<(), TopologyError> {
    let wire_id = topo.face(face_id)?.outer_wire();
    let replacement: Wire = topo.wire(wire_id)?.clone();
    topo.replace_boundary_wire(wire_id, replacement)
}

/// Read one face's loops and retire them. `build_face_loops` on a derived
/// face must be read-only — the same handles back, nothing allocated — and
/// the sanctioned wire replacement is what retires a derivation: its handles
/// must then fail typed lookups and never be reissued to the replacement.
fn op_derive(topo: &mut Topology, solid: SolidId, pick: u8) {
    let faces = explorer::solid_faces(topo, solid)
        .unwrap_or_else(|error| panic!("live solid has no faces: {error}"));
    let face_id = faces[usize::from(pick) % faces.len()];

    let prior_loops = authoritative_loops(topo, face_id);
    let mut prior_coedges = Vec::new();
    for &loop_id in &prior_loops {
        let boundary = topo
            .face_loop(loop_id)
            .unwrap_or_else(|error| panic!("prior derivation is broken: {error}"));
        prior_coedges.extend(boundary.coedges().iter().copied());
    }

    let counts_before = (topo.num_loops(), topo.num_coedges());
    let same_loops = topo
        .build_face_loops(face_id)
        .unwrap_or_else(|error| panic!("box face derivation failed: {error}"));
    assert_eq!(
        same_loops, prior_loops,
        "build_face_loops replaced the loops of an already-derived face"
    );
    assert_eq!(
        (topo.num_loops(), topo.num_coedges()),
        counts_before,
        "build_face_loops allocated on an already-derived face"
    );
    assert_loop_derivation(topo, face_id, &same_loops);

    replace_outer_wire_in_place(topo, face_id)
        .unwrap_or_else(|error| panic!("sanctioned wire replacement failed: {error}"));
    let new_loops = authoritative_loops(topo, face_id);
    assert_loop_derivation(topo, face_id, &new_loops);

    for loop_id in &prior_loops {
        assert!(
            topo.face_loop(*loop_id).is_err(),
            "replaced loop handle {loop_id:?} still resolves",
        );
    }
    for coedge_id in &prior_coedges {
        assert!(
            topo.coedge(*coedge_id).is_err(),
            "replaced coedge handle {coedge_id:?} still resolves",
        );
    }
    let retired_loop_slots: BTreeSet<usize> = prior_loops.iter().map(|id| id.index()).collect();
    let retired_coedge_slots: BTreeSet<usize> =
        prior_coedges.iter().map(|id| id.index()).collect();
    for &loop_id in &new_loops {
        assert!(
            !retired_loop_slots.contains(&loop_id.index()),
            "a retired loop slot was reissued to the new derivation",
        );
        let boundary = topo
            .face_loop(loop_id)
            .unwrap_or_else(|error| panic!("new loop lookup failed: {error}"));
        for &coedge_id in boundary.coedges() {
            assert!(
                !retired_coedge_slots.contains(&coedge_id.index()),
                "a retired coedge slot was reissued to the new derivation",
            );
        }
    }
}

/// Break one face's outer wire inside a validated transaction. The
/// validator must veto, and the veto must restore the exact pre-operation
/// state — the flipped orientation included.
fn op_break_wire(topo: &mut Topology, solid: SolidId, face_pick: u8, edge_pick: u8) {
    let faces = explorer::solid_faces(topo, solid)
        .unwrap_or_else(|error| panic!("live solid has no faces: {error}"));
    let face_id = faces[usize::from(face_pick) % faces.len()];
    let wire_id = topo
        .face(face_id)
        .unwrap_or_else(|error| panic!("face lookup failed: {error}"))
        .outer_wire();
    let edge_count = topo
        .wire(wire_id)
        .unwrap_or_else(|error| panic!("wire lookup failed: {error}"))
        .edges()
        .len();
    let pick = usize::from(edge_pick) % edge_count;

    let pre = live_state(topo);
    let result = run_validated(
        topo,
        |topo| {
            let wire = topo.wire_mut(wire_id)?;
            let oriented = wire.edges()[pick];
            wire.edges_mut()[pick] = OrientedEdge::new(oriented.edge(), !oriented.is_forward());
            Ok::<_, TopologyError>(())
        },
        |topo, ()| {
            let wire = topo.wire(wire_id)?;
            validate_wire_closed(wire, topo)
        },
    );
    assert!(
        result.is_err(),
        "the closure validator accepted a deliberately broken wire"
    );
    assert_eq!(
        live_state(topo),
        pre,
        "a vetoed mutation must roll back to the exact pre-operation state"
    );
}

/// Stage allocations inside a transaction that then fails. Every staged
/// handle must stay stale, never be reissued, and the live state must match
/// the pre-operation state exactly.
fn op_staged_alloc(topo: &mut Topology, bytes: &mut Bytes<'_>) {
    let pre = live_state(topo);
    let slots_before = topo.allocated_slot_count();

    let mut leaked_vertices = Vec::new();
    let mut leaked_edges = Vec::new();
    let result = run_transacted(topo, |topo| {
        let extra = 1 + usize::from(bytes.next()) % 3;
        let mut staged = Vec::new();
        for _ in 0..extra {
            let point = Point3::new(bytes.signed(), bytes.signed(), bytes.signed());
            staged.push(topo.add_vertex(Vertex::new(point, 1e-7)));
        }
        leaked_vertices.extend_from_slice(&staged);
        for pair in staged.windows(2) {
            leaked_edges.push(topo.add_edge(Edge::new(pair[0], pair[1], EdgeCurve::Line)));
        }
        Err::<(), _>(TopologyError::Empty {
            entity: "fuzz staged allocation",
        })
    });
    assert!(result.is_err(), "the staged transaction must fail");

    assert_eq!(
        live_state(topo),
        pre,
        "a failed transaction must roll back to the exact pre-operation state"
    );
    assert!(
        topo.allocated_slot_count() >= slots_before,
        "rollback must preserve the arena high-water marks"
    );
    for &vertex_id in &leaked_vertices {
        assert!(
            topo.vertex(vertex_id).is_err(),
            "handle {vertex_id:?} leaked from a rolled-back transaction still resolves",
        );
    }
    for &edge_id in &leaked_edges {
        assert!(
            topo.edge(edge_id).is_err(),
            "handle {edge_id:?} leaked from a rolled-back transaction still resolves",
        );
    }

    // Slots retired by the rollback are never reissued: the next allocation
    // must land above every leaked handle.
    let fresh = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
    for &vertex_id in &leaked_vertices {
        assert_ne!(
            fresh.index(),
            vertex_id.index(),
            "a rolled-back vertex slot was reissued",
        );
    }
}

/// Remove one face's derivation from a captured state: the map entry, the
/// loop and coedge rows, and the two counts. Used to state the exact
/// postcondition of a checkpoint restore whose window retired a
/// pre-snapshot derivation.
fn strip_derivation(state: &mut LiveState, face_index: usize) {
    let Some(position) = state
        .face_loops
        .iter()
        .position(|(face, _)| *face == face_index)
    else {
        return;
    };
    let (_, loop_slots) = state.face_loops.remove(position);
    let loop_slots: BTreeSet<usize> = loop_slots.into_iter().collect();
    let mut coedge_slots = BTreeSet::new();
    state.loops.retain(|(slot, (_, _, coedges))| {
        if loop_slots.contains(slot) {
            coedge_slots.extend(coedges.iter().copied());
            return false;
        }
        true
    });
    state
        .coedges
        .retain(|(slot, _)| !coedge_slots.contains(slot));
    state.counts[8] -= loop_slots.len();
    state.counts[9] -= coedge_slots.len();
}

/// Snapshot, mutate, restore — the *checkpoint barrier* contract. Arena
/// slots allocated in the window are tombstoned and never reissued, and a
/// retirement staged in the window *sticks*: it may already have been
/// reported to an external handle holder. The derivation map must never
/// dangle — a face whose derivation stayed retired simply has none.
fn op_snapshot_restore(
    topo: &mut Topology,
    solid: SolidId,
    bytes: &mut Bytes<'_>,
    derived: &mut BTreeMap<usize, Vec<LoopId>>,
) {
    let faces = explorer::solid_faces(topo, solid)
        .unwrap_or_else(|error| panic!("live solid has no faces: {error}"));
    let face_id = faces[usize::from(bytes.next()) % faces.len()];

    let pre = live_state(topo);
    let prior = authoritative_loops(topo, face_id);
    let snapshot = topo.clone();

    let junk = topo.add_vertex(Vertex::new(
        Point3::new(bytes.signed(), bytes.signed(), bytes.signed()),
        1e-7,
    ));
    // The window retires the prior derivation and issues a fresh one.
    replace_outer_wire_in_place(topo, face_id)
        .unwrap_or_else(|error| panic!("in-window wire replacement failed: {error}"));
    let window_loops = authoritative_loops(topo, face_id);

    topo.restore_preserving_handle_slots(&snapshot);

    // Window allocations are tombstoned and never reissued.
    assert!(
        topo.vertex(junk).is_err(),
        "a vertex allocated after the snapshot still resolves after restore",
    );
    for &loop_id in &window_loops {
        assert!(
            topo.face_loop(loop_id).is_err(),
            "a loop derived after the snapshot still resolves after restore",
        );
    }

    // The window retirement sticks: the prior derivation stays retired, and
    // the restored face is promoted onto fresh handles rather than left
    // dangling on retired ones.
    for &loop_id in &prior {
        assert!(
            topo.face_loop(loop_id).is_err(),
            "checkpoint restore revived a retired loop",
        );
    }
    let restored = authoritative_loops(topo, face_id);
    let stale_slots: BTreeSet<usize> = prior
        .iter()
        .chain(&window_loops)
        .map(|id| id.index())
        .collect();
    for loop_id in &restored {
        assert!(
            !stale_slots.contains(&loop_id.index()),
            "checkpoint restore reissued a retired loop slot",
        );
    }
    assert_loop_derivation(topo, face_id, &restored);
    validate_face_loops(topo, face_id)
        .unwrap_or_else(|error| panic!("restored face failed the loop checker: {error}"));
    derived.insert(face_id.index(), restored);

    // Everything but that face's re-issued derivation is restored exactly.
    let mut expected = pre;
    strip_derivation(&mut expected, face_id.index());
    let mut actual = live_state(topo);
    strip_derivation(&mut actual, face_id.index());
    assert_eq!(
        actual, expected,
        "restore after a retirement window must reproduce the exact state \
         minus the re-issued derivation"
    );

    let fresh = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 1.0), 1e-7));
    assert_ne!(
        fresh.index(),
        junk.index(),
        "a slot retired by the restore was reissued",
    );
}

/// Replace a wire inside a transaction that then fails. The rollback must
/// undo the in-window retirement of the previous derivation — the failed
/// transaction was never observed, so the original handles resolve again
/// and the live state matches the pre-transaction state exactly.
fn op_transacted_rederive_rollback(topo: &mut Topology, solid: SolidId, face_pick: u8) {
    let faces = explorer::solid_faces(topo, solid)
        .unwrap_or_else(|error| panic!("live solid has no faces: {error}"));
    let face_id = faces[usize::from(face_pick) % faces.len()];

    let pre = live_state(topo);
    let result = run_transacted(topo, |topo| {
        replace_outer_wire_in_place(topo, face_id)?;
        Err::<(), _>(TopologyError::Empty {
            entity: "fuzz injected failure",
        })
    });
    assert!(result.is_err(), "the staged transaction must fail");
    assert_eq!(
        live_state(topo),
        pre,
        "rollback must undo an in-window wire replacement exactly"
    );
}

/// Delete the box inside a transaction that then fails. When the solid is
/// unreferenced the deletion stages a full tree retirement; when a root
/// references it the deletion refuses without touching anything. Either
/// way the rollback must restore the exact pre-transaction state.
fn op_transacted_delete_rollback(topo: &mut Topology, solid: SolidId) {
    let pre = live_state(topo);
    let result = run_transacted(topo, |topo| {
        topo.delete_solid(solid).map_err(|_| TopologyError::Empty {
            entity: "delete in fuzz",
        })?;
        Err::<(), _>(TopologyError::Empty {
            entity: "fuzz injected failure",
        })
    });
    assert!(result.is_err(), "the staged transaction must fail");
    assert_eq!(
        live_state(topo),
        pre,
        "rollback must undo an in-window deletion exactly"
    );
    assert!(
        topo.solid(solid).is_ok(),
        "a rolled-back deletion left the solid retired"
    );
}

/// Delete the box while it is unreferenced. The complete topology tree —
/// and nothing else — must be retired: a guard box, and a compound
/// referencing only that guard, must survive with every handle resolving.
/// Returns the guard's id, closed-form volume, and volume deflection when a
/// guard was built.
fn op_delete_unreferenced(
    topo: &mut Topology,
    solid: SolidId,
    bytes: &mut Bytes<'_>,
    scale: f64,
) -> Option<(SolidId, f64, f64)> {
    let with_guard = bytes.next() & 1 == 0;
    let with_unrelated_compound = bytes.next() & 1 == 0;

    let guard = if with_guard {
        let (gdx, gdy, gdz) = (
            bytes.dim() * scale,
            bytes.dim() * scale,
            bytes.dim() * scale,
        );
        let guard_id = make_box(topo, gdx, gdy, gdz)
            .unwrap_or_else(|error| panic!("bounded guard box was rejected: {error}"));
        Some((
            guard_id,
            gdx * gdy * gdz,
            inv::volume_deflection((gdx * gdx + gdy * gdy + gdz * gdz).sqrt()),
        ))
    } else {
        None
    };

    let tree = collect_tree(topo, solid);
    let guard_tree = guard.map(|(guard_id, _, _)| collect_tree(topo, guard_id));
    let pre_counts = counts(topo);
    let compound_id = if with_unrelated_compound {
        guard.map(|(guard_id, _, _)| topo.add_compound(Compound::new(vec![guard_id])))
    } else {
        None
    };

    topo.delete_solid(solid)
        .unwrap_or_else(|error| panic!("unreferenced deletion was refused: {error}"));

    // The solid and its complete tree fail typed lookups.
    assert!(
        topo.solid(solid).is_err(),
        "a deleted solid handle still resolves"
    );
    for &index in &tree.shells {
        let stale = index;
        assert!(
            topo.shells().iter().all(|(id, _)| id.index() != stale),
            "a retired shell slot {stale} is still live",
        );
    }
    let post = live_state(topo);
    for (slot, _) in &post.faces {
        assert!(
            !tree.faces.contains(slot),
            "a retired face slot {slot} is still live"
        );
    }
    for (slot, _) in &post.wires {
        assert!(
            !tree.wires.contains(slot),
            "a retired wire slot {slot} is still live"
        );
    }
    for (slot, _) in &post.edges {
        assert!(
            !tree.edges.contains(slot),
            "a retired edge slot {slot} is still live"
        );
    }
    for (slot, _) in &post.vertices {
        assert!(
            !tree.vertices.contains(slot),
            "a retired vertex slot {slot} is still live"
        );
    }
    for (slot, _) in &post.loops {
        assert!(
            !tree.loops.contains(slot),
            "a retired loop slot {slot} is still live"
        );
    }
    for (slot, _) in &post.coedges {
        assert!(
            !tree.coedges.contains(slot),
            "a retired coedge slot {slot} is still live"
        );
    }

    // Counts drop by exactly the tree sizes — nothing shared, nothing extra.
    let post_counts = counts(topo);
    let expected = [
        pre_counts[0] - tree.vertices.len(),
        pre_counts[1] - tree.edges.len(),
        pre_counts[2] - tree.wires.len(),
        pre_counts[3] - tree.faces.len(),
        pre_counts[4] - tree.shells.len(),
        pre_counts[5] - 1,
        pre_counts[6] + usize::from(compound_id.is_some()),
        pre_counts[7],
        pre_counts[8] - tree.loops.len(),
        pre_counts[9] - tree.coedges.len(),
    ];
    assert_eq!(
        post_counts, expected,
        "deletion retired a different set than the complete unshared tree \
         (pre {pre_counts:?}, post {post_counts:?}, tree {tree:?})",
    );

    // The guard and its referencing compound survive untouched.
    if let Some((guard_id, guard_volume, guard_deflection)) = guard {
        let guard_tree = guard_tree.unwrap_or_else(|| unreachable!("guard tree captured"));
        let post_guard = collect_tree(topo, guard_id);
        assert_eq!(
            post_guard.shells, guard_tree.shells,
            "deletion disturbed the guard's shells"
        );
        assert_eq!(
            post_guard.faces, guard_tree.faces,
            "deletion disturbed the guard's faces"
        );
        assert_eq!(
            post_guard.vertices, guard_tree.vertices,
            "deletion disturbed the guard's vertices"
        );
        if let Some(compound) = compound_id {
            let root = topo
                .compound(compound)
                .unwrap_or_else(|error| panic!("unrelated compound was retired: {error}"));
            assert_eq!(
                root.solids(),
                &[guard_id],
                "unrelated compound lost its reference"
            );
        }
        return Some((guard_id, guard_volume, guard_deflection));
    }
    None
}

/// A live compound or comp-solid referencing the box must turn deletion
/// into a typed, atomic refusal: the error names the dependent, and the
/// live state afterwards is byte-identical.
fn op_delete_referenced(
    topo: &mut Topology,
    solid: SolidId,
    via_compound: bool,
    reference: &mut Option<(bool, usize)>,
) {
    if reference.is_none() {
        let root = if via_compound {
            topo.add_compound(Compound::new(vec![solid])).index()
        } else {
            topo.add_compsolid(CompSolid::new(vec![solid], Vec::new()))
                .index()
        };
        *reference = Some((via_compound, root));
    }
    let (kind_is_compound, root_index) =
        reference.unwrap_or_else(|| unreachable!("reference recorded above"));

    let pre = live_state(topo);
    let error = topo
        .delete_solid(solid)
        .err()
        .unwrap_or_else(|| panic!("referenced deletion unexpectedly succeeded"));
    match error {
        DeleteSolidError::Referenced {
            solid: refused,
            dependent,
            dependent_index,
        } => {
            assert_eq!(refused, solid, "refusal names the wrong solid");
            assert_eq!(
                dependent,
                if kind_is_compound {
                    "compound"
                } else {
                    "comp-solid"
                },
                "refusal names the wrong dependent kind",
            );
            assert_eq!(
                dependent_index, root_index,
                "refusal names the wrong dependent root",
            );
        }
        other => panic!("referenced deletion failed with the wrong type: {other}"),
    }
    assert_eq!(
        live_state(topo),
        pre,
        "a refused deletion produced a partial mutation"
    );

    if kind_is_compound {
        let root = topo
            .compound(
                topo.compound_id_from_index(root_index)
                    .unwrap_or_else(|| panic!("referencing compound vanished")),
            )
            .unwrap_or_else(|error| panic!("referencing compound lookup failed: {error}"));
        assert!(
            root.solids().contains(&solid),
            "the referencing compound lost its reference"
        );
    } else {
        let root = topo
            .compsolid(
                topo.compsolid_id_from_index(root_index)
                    .unwrap_or_else(|| panic!("referencing comp-solid vanished")),
            )
            .unwrap_or_else(|error| panic!("referencing comp-solid lookup failed: {error}"));
        assert!(
            root.solids().contains(&solid),
            "the referencing comp-solid lost its reference"
        );
    }
}

/// The post-mutation sweep: comprehensive validation, the closed-manifold
/// census, the closed-form volume, and the derivation census. Runs after
/// every mutation while the box is alive.
fn sweep(
    topo: &Topology,
    solid: SolidId,
    expected_volume: f64,
    deflection: f64,
    derived: &BTreeMap<usize, Vec<LoopId>>,
) {
    let report = validate_solid(topo, solid)
        .unwrap_or_else(|error| panic!("validation declined a mutated box: {error}"));
    assert!(
        report.is_valid(),
        "comprehensive validation fails after mutation: {report:?}"
    );

    let census = inv::census(topo, solid)
        .unwrap_or_else(|error| panic!("census failed on a mutated box: {error}"));
    assert!(
        census.faces == 6 && census.edges == 12 && census.vertices == 8,
        "box census moved: {census:?}"
    );
    assert!(
        census.inner_wires == 0 && census.surfaces.get("plane") == Some(&6),
        "box surface census moved: {census:?}"
    );
    inv::assert_closed_manifold("topology_mutation", &census);
    assert_eq!(
        census.shells.len(),
        1,
        "a single box must stay a single component: {census:?}"
    );

    let measured = solid_volume(topo, solid, deflection)
        .unwrap_or_else(|error| panic!("volume measurement declined a mutated box: {error}"));
    inv::assert_exact_volume("topology_mutation", expected_volume, measured);

    // The derivation census: live loops and coedges are exactly the
    // registered derivations, and every one still matches its wires.
    let mut expected_loops = 0;
    let mut expected_coedges = 0;
    for (&face_index, loop_ids) in derived {
        let face_id = topo
            .face_id_from_index(face_index)
            .unwrap_or_else(|| panic!("tracked face index {face_index} is stale"));
        assert_loop_derivation(topo, face_id, loop_ids);
        expected_loops += loop_ids.len();
        for &loop_id in loop_ids {
            let boundary = topo
                .face_loop(loop_id)
                .unwrap_or_else(|error| panic!("tracked loop lookup failed: {error}"));
            expected_coedges += boundary.coedges().len();
        }
    }
    assert_eq!(
        topo.num_loops(),
        expected_loops,
        "live loop count disagrees with the registered derivations"
    );
    assert_eq!(
        topo.num_coedges(),
        expected_coedges,
        "live coedge count disagrees with the registered derivations"
    );
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let mut bytes = Bytes::new(data);
    let scale = [1.0e-3, 1.0, 1.0e3][usize::from(bytes.next()) % 3];
    let (dx, dy, dz) = (
        bytes.dim() * scale,
        bytes.dim() * scale,
        bytes.dim() * scale,
    );
    let expected_volume = dx * dy * dz;
    let deflection = inv::volume_deflection((dx * dx + dy * dy + dz * dz).sqrt());

    let mut topo = Topology::new();
    let solid = make_box(&mut topo, dx, dy, dz)
        .unwrap_or_else(|error| panic!("bounded box was rejected: {error}"));

    let mut derived = registered_derivations(&topo, solid);
    let mut reference: Option<(bool, usize)> = None;
    let mut alive = true;
    let op_count = 1 + usize::from(bytes.next()) % MAX_OPS;

    for _ in 0..op_count {
        if !alive {
            break;
        }
        let face_pick = bytes.next();
        match bytes.next() % 10 {
            0 | 1 => {
                op_derive(&mut topo, solid, face_pick);
                if bytes.next() & 1 == 0 {
                    // Replace the same face's wire again immediately, so
                    // the retirement oracle sees a second generation.
                    op_derive(&mut topo, solid, face_pick);
                }
                derived = registered_derivations(&topo, solid);
            }
            2 => op_break_wire(&mut topo, solid, face_pick, bytes.next()),
            3 => op_staged_alloc(&mut topo, &mut bytes),
            4 => op_snapshot_restore(&mut topo, solid, &mut bytes, &mut derived),
            5 => {
                if reference.is_some() {
                    // A referenced solid cannot be deleted; the refusal
                    // half of the contract is covered by arms 6 and 7.
                    continue;
                }
                let guard = op_delete_unreferenced(&mut topo, solid, &mut bytes, scale);
                alive = false;
                if let Some((guard_id, guard_volume, guard_deflection)) = guard {
                    sweep(
                        &topo,
                        guard_id,
                        guard_volume,
                        guard_deflection,
                        &registered_derivations(&topo, guard_id),
                    );
                }
            }
            6 => op_delete_referenced(&mut topo, solid, true, &mut reference),
            7 => op_delete_referenced(&mut topo, solid, false, &mut reference),
            8 => {
                // Retire one generation first so the rolled-back
                // replacement acts on a derivation that already has
                // retired predecessors behind it.
                op_derive(&mut topo, solid, face_pick);
                derived = registered_derivations(&topo, solid);
                op_transacted_rederive_rollback(&mut topo, solid, face_pick);
            }
            _ => op_transacted_delete_rollback(&mut topo, solid),
        }
        if alive {
            sweep(&topo, solid, expected_volume, deflection, &derived);
        }
    }
});
