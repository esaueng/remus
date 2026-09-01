//! Central context holding all topological arenas.
//!
//! [`Topology`] is the single owner of every arena. All operations that
//! create or query topological entities take a reference to this struct.

use std::collections::{HashMap, HashSet};

use crate::adjacency::AdjacencyIndex;
use crate::arena::Arena;
use crate::attributes::AttributeStore;
use crate::coedge::{Coedge, CoedgeId};
use crate::compound::{Compound, CompoundId};
use crate::compsolid::{CompSolid, CompSolidId};
use crate::edge::{Edge, EdgeId};
use crate::face::{Face, FaceId};
use crate::face_loop::{Loop, LoopId};
use crate::journal::{EntityKey, EvolutionDraft, Journal, OpId, PendingOp};
use crate::pcurve::{PCurve, PCurveRegistry};
use crate::shell::{Shell, ShellId};
use crate::solid::{Solid, SolidId};
use crate::vertex::{Vertex, VertexId};
use crate::wire::{OrientedEdge, Wire, WireId};
use crate::{DeleteSolidError, TopologyError};

/// Central context owning all topological entity arenas.
///
/// Arena fields are private to enforce invariants through the public API.
/// Use the typed accessor methods for lookups and the `add_*` methods
/// for allocation.
#[derive(Debug, Default, Clone)]
pub struct Topology {
    /// All vertices in the model.
    vertices: Arena<Vertex>,
    /// All edges in the model.
    edges: Arena<Edge>,
    /// All wires in the model.
    wires: Arena<Wire>,
    /// All faces in the model.
    faces: Arena<Face>,
    /// All shells in the model.
    shells: Arena<Shell>,
    /// All solids in the model.
    solids: Arena<Solid>,
    /// All compounds in the model.
    compounds: Arena<Compound>,
    /// All comp-solids in the model.
    compsolids: Arena<CompSolid>,
    /// `PCurves`: 2D parametric curves mapping edges to face surface parameters.
    pcurves: PCurveRegistry,
    /// All face-boundary loops (RFC 0002, Stage 1: derived from wires on
    /// request, never authoritative).
    loops: Arena<Loop>,
    /// All coedge uses (RFC 0002).
    coedges: Arena<Coedge>,
    /// Loops derived for each face by [`Self::build_face_loops`].
    face_loops: HashMap<FaceId, Vec<LoopId>>,
    /// Semantic names and display colors (Issue 14).
    attributes: AttributeStore,
    /// Append-only evolution journal (RFC 0003, Stage 1).
    journal: Journal,
    /// Counts every model mutation (allocation, exclusive access, retire,
    /// pcurve change). The journal compares this against its last entry to
    /// detect mutations no entry accounts for — see
    /// [`Self::journal_begin`]. Deliberately conservative: taking an
    /// exclusive reference counts even if nothing is written, because a
    /// false gap fails closed while a missed mutation would fake
    /// continuity.
    mutation_ticks: u64,
}

#[derive(Default)]
struct SolidEntities {
    vertices: HashSet<VertexId>,
    edges: HashSet<EdgeId>,
    wires: HashSet<WireId>,
    faces: HashSet<FaceId>,
    shells: HashSet<ShellId>,
}

#[derive(Clone)]
struct BoundaryLoopSpec {
    oriented_edges: Vec<OrientedEdge>,
    closed: bool,
}

/// Generates an immutable arena accessor method on [`Topology`].
///
/// Usage: `arena_get!(method_name, arena_field, EntityType, IdType, ErrorVariant)`
macro_rules! arena_get {
    ($method:ident, $field:ident, $T:ty, $Id:ty, $err:ident) => {
        /// Returns a shared reference to the entity with the given ID.
        ///
        /// # Errors
        ///
        /// Returns a not-found error if the ID is invalid.
        pub fn $method(&self, id: $Id) -> Result<&$T, TopologyError> {
            self.$field.get(id).ok_or(TopologyError::$err(id))
        }
    };
}

/// Generates a mutable arena accessor method on [`Topology`].
///
/// Usage: `arena_get_mut!(method_name, arena_field, EntityType, IdType, ErrorVariant)`
macro_rules! arena_get_mut {
    ($method:ident, $field:ident, $T:ty, $Id:ty, $err:ident) => {
        /// Returns an exclusive reference to the entity with the given ID.
        ///
        /// Counts as a model mutation for journal gap detection (see
        /// [`Topology::journal_begin`]), even if nothing is written.
        ///
        /// # Errors
        ///
        /// Returns a not-found error if the ID is invalid.
        pub fn $method(&mut self, id: $Id) -> Result<&mut $T, TopologyError> {
            self.mutation_ticks = self.mutation_ticks.saturating_add(1);
            self.$field.get_mut(id).ok_or(TopologyError::$err(id))
        }
    };
}

/// Generates allocation, read-only arena access, count, and index
/// reconstruction methods for a single entity type.
macro_rules! arena_api {
    (
        add = $add:ident,
        arena = $arena:ident,
        arena_fn = $arena_fn:ident,
        count = $count:ident,
        id_from_index = $id_from_index:ident,
        T = $T:ty,
        Id = $Id:ty
    ) => {
        /// Allocates a new entity in the arena and returns its typed handle.
        pub fn $add(&mut self, value: $T) -> $Id {
            self.mutation_ticks = self.mutation_ticks.saturating_add(1);
            self.$arena.alloc(value)
        }

        /// Returns a shared reference to the arena for iteration and queries.
        #[must_use]
        pub fn $arena_fn(&self) -> &Arena<$T> {
            &self.$arena
        }

        /// Returns the number of entities in this arena.
        #[must_use]
        pub fn $count(&self) -> usize {
            self.$arena.len()
        }

        /// Reconstructs a typed ID from a raw index, returning `None` if
        /// out of bounds. Intended for FFI boundaries (e.g. WASM).
        #[must_use]
        pub fn $id_from_index(&self, index: usize) -> Option<$Id> {
            self.$arena.id_from_index(index)
        }
    };
}

impl Topology {
    /// Creates a new, empty topology context.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Total arena slots, including retired slots reserved to prevent stale
    /// numeric handles from aliasing later entities.
    ///
    /// This is a lifetime high-water mark, **not** a measure of current model
    /// size: arenas only ever append, retiring an entity just clears its
    /// liveness bit, and a checkpoint restore re-extends each arena back to its
    /// previous slot count. The value therefore never decreases for the life of
    /// a `Topology`, and rises with every operation — including ones that are
    /// rolled back. Use the per-arena `len()` accessors (`vertex_count`,
    /// `face_count`, …) for live entity counts, and never use this as a proxy
    /// for how much geometry a model currently holds.
    #[must_use]
    pub fn allocated_slot_count(&self) -> usize {
        [
            self.vertices.slot_len(),
            self.edges.slot_len(),
            self.wires.slot_len(),
            self.faces.slot_len(),
            self.shells.slot_len(),
            self.solids.slot_len(),
            self.compounds.slot_len(),
            self.compsolids.slot_len(),
            self.loops.slot_len(),
            self.coedges.slot_len(),
        ]
        .into_iter()
        .fold(0usize, usize::saturating_add)
    }

    /// Restore a topology snapshot while permanently retiring arena slots
    /// allocated after that snapshot.
    ///
    /// This is intended for external-handle runtimes such as the WASM kernel.
    /// Preserving each arena's high-water mark prevents a raw numeric handle
    /// from aliasing an unrelated entity created after a restore.
    ///
    /// This is the *checkpoint barrier* semantics: an entity retired after
    /// the snapshot stays retired, because the retirement may already have
    /// been reported to an external handle holder (e.g. a committed
    /// `deleteSolid` call) and must never be silently undone. For a failed
    /// transactional operation — whose retirements were never observed —
    /// use [`Self::restore_for_rollback`], which undoes them.
    pub fn restore_preserving_handle_slots(&mut self, snapshot: &Self) {
        self.vertices.restore_preserving_slots(&snapshot.vertices);
        self.edges.restore_preserving_slots(&snapshot.edges);
        self.wires.restore_preserving_slots(&snapshot.wires);
        self.faces.restore_preserving_slots(&snapshot.faces);
        self.shells.restore_preserving_slots(&snapshot.shells);
        self.solids.restore_preserving_slots(&snapshot.solids);
        self.compounds.restore_preserving_slots(&snapshot.compounds);
        self.compsolids
            .restore_preserving_slots(&snapshot.compsolids);
        self.loops.restore_preserving_slots(&snapshot.loops);
        self.coedges.restore_preserving_slots(&snapshot.coedges);
        self.face_loops.clone_from(&snapshot.face_loops);
        self.attributes.clone_from(&snapshot.attributes);
        self.pcurves.clone_from(&snapshot.pcurves);
        // Journal entries recorded after the snapshot are truncated with the
        // restore — the journal and the model roll back together, so the
        // history matches the state again — while `OpId`s and ordinals
        // issued by rolled-back operations are high-water preserved and
        // never reissued (the journal analogue of arena slot preservation).
        // The tick count rolls back with the state: the restored model is
        // exactly the one the restored journal describes.
        self.journal.restore_preserving_ids(&snapshot.journal);
        self.mutation_ticks = snapshot.mutation_ticks;
        let retired_edges = snapshot
            .edges
            .iter()
            .filter_map(|(id, _)| self.edges.get(id).is_none().then_some(id))
            .collect();
        let retired_faces = snapshot
            .faces
            .iter()
            .filter_map(|(id, _)| self.faces.get(id).is_none().then_some(id))
            .collect();
        let retired_solids = snapshot
            .solids
            .iter()
            .filter_map(|(id, _)| self.solids.get(id).is_none().then_some(id))
            .collect();
        self.pcurves
            .remove_for_retired_entities(&retired_edges, &retired_faces);
        self.attributes
            .remove_for_retired_entities(&retired_solids, &retired_faces);
        // Retirement is sticky here, so the derivation map — restored from
        // the snapshot — can reference loops that stayed retired, or be
        // keyed by a face that stayed retired. Drop those entries so the
        // map never dangles; a face whose derivation is lost this way
        // simply has no derivation, which Stage 1 treats as "not yet
        // derived" (wires remain authoritative).
        let faces = &self.faces;
        let loops = &self.loops;
        let coedges = &self.coedges;
        self.face_loops.retain(|face_id, loop_ids| {
            faces.get(*face_id).is_some()
                && loop_ids.iter().all(|loop_id| {
                    loops.get(*loop_id).is_some_and(|boundary| {
                        boundary
                            .coedges()
                            .iter()
                            .all(|coedge_id| coedges.get(*coedge_id).is_some())
                    })
                })
        });
    }

    /// Restore the exact pre-transaction state after a failed transactional
    /// operation, undoing every mutation the operation staged — allocations
    /// *and* retirements.
    ///
    /// This is the rollback half of the stage → validate → commit / roll
    /// back contract ([`transaction`](crate::transaction)): the failed
    /// operation was never observed by any caller, so its retirements are
    /// undone along with its allocations, and live entity counts and
    /// contents match the snapshot exactly. Handles the operation allocated
    /// stay permanently invalid (arena slots are high-water preserved,
    /// never reused).
    ///
    /// Use [`Self::restore_preserving_handle_slots`] for the checkpoint
    /// barrier instead: there a retirement may already have been reported
    /// to an external handle holder and must stay retired.
    pub fn restore_for_rollback(&mut self, snapshot: &Self) {
        self.vertices.restore_for_rollback(&snapshot.vertices);
        self.edges.restore_for_rollback(&snapshot.edges);
        self.wires.restore_for_rollback(&snapshot.wires);
        self.faces.restore_for_rollback(&snapshot.faces);
        self.shells.restore_for_rollback(&snapshot.shells);
        self.solids.restore_for_rollback(&snapshot.solids);
        self.compounds.restore_for_rollback(&snapshot.compounds);
        self.compsolids.restore_for_rollback(&snapshot.compsolids);
        self.loops.restore_for_rollback(&snapshot.loops);
        self.coedges.restore_for_rollback(&snapshot.coedges);
        self.face_loops.clone_from(&snapshot.face_loops);
        self.attributes.clone_from(&snapshot.attributes);
        self.pcurves.clone_from(&snapshot.pcurves);
        // The journal rolls back with the model, exactly as in
        // [`Self::restore_preserving_handle_slots`].
        self.journal.restore_preserving_ids(&snapshot.journal);
        self.mutation_ticks = snapshot.mutation_ticks;
    }

    /// Reserves capacity for the given number of additional entities in the
    /// six primary entity arenas (vertices, edges, wires, faces, shells, solids).
    ///
    /// Does **not** cover compounds, comp-solids, or the pcurve registry.
    ///
    /// Useful for pre-allocating before bulk insertion (e.g. boolean assembly)
    /// to avoid repeated reallocations.
    pub fn reserve(
        &mut self,
        vertices: usize,
        edges: usize,
        wires: usize,
        faces: usize,
        shells: usize,
        solids: usize,
    ) {
        self.vertices.reserve(vertices);
        self.edges.reserve(edges);
        self.wires.reserve(wires);
        self.faces.reserve(faces);
        self.shells.reserve(shells);
        self.solids.reserve(solids);
    }

    arena_get!(vertex, vertices, Vertex, VertexId, VertexNotFound);
    arena_get_mut!(vertex_mut, vertices, Vertex, VertexId, VertexNotFound);

    arena_get!(edge, edges, Edge, EdgeId, EdgeNotFound);
    arena_get_mut!(edge_mut, edges, Edge, EdgeId, EdgeNotFound);

    arena_get!(wire, wires, Wire, WireId, WireNotFound);
    arena_get_mut!(wire_mut, wires, Wire, WireId, WireNotFound);

    arena_get!(face, faces, Face, FaceId, FaceNotFound);
    arena_get_mut!(face_mut, faces, Face, FaceId, FaceNotFound);

    arena_get!(shell, shells, Shell, ShellId, ShellNotFound);
    arena_get_mut!(shell_mut, shells, Shell, ShellId, ShellNotFound);

    arena_get!(solid, solids, Solid, SolidId, SolidNotFound);
    arena_get_mut!(solid_mut, solids, Solid, SolidId, SolidNotFound);

    arena_get!(compound, compounds, Compound, CompoundId, CompoundNotFound);
    arena_get_mut!(
        compound_mut,
        compounds,
        Compound,
        CompoundId,
        CompoundNotFound
    );

    arena_get!(
        compsolid,
        compsolids,
        CompSolid,
        CompSolidId,
        CompSolidNotFound
    );
    arena_get!(face_loop, loops, Loop, LoopId, LoopNotFound);
    arena_get!(coedge, coedges, Coedge, CoedgeId, CoedgeNotFound);

    /// The attribute store (semantic names, display colors).
    #[must_use]
    pub fn attributes(&self) -> &AttributeStore {
        &self.attributes
    }

    /// Sets or clears a solid's attributes after validating that it is live.
    ///
    /// An empty value clears the entry.
    ///
    /// Attribute changes are not model mutations for journal purposes:
    /// they never change which entity an entity *is*, so they cannot break
    /// a lineage claim.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::SolidNotFound`] for a stale or non-live handle.
    pub fn set_solid_attributes(
        &mut self,
        solid: SolidId,
        attributes: crate::attributes::EntityAttributes,
    ) -> Result<(), TopologyError> {
        let _ = self.solid(solid)?;
        self.attributes.set_solid(solid, attributes);
        Ok(())
    }

    /// Sets or clears a face's attributes after validating that it is live.
    ///
    /// An empty value clears the entry.
    ///
    /// Attribute changes are not model mutations for journal purposes:
    /// they never change which entity an entity *is*, so they cannot break
    /// a lineage claim.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::FaceNotFound`] for a stale or non-live handle.
    pub fn set_face_attributes(
        &mut self,
        face: FaceId,
        attributes: crate::attributes::EntityAttributes,
    ) -> Result<(), TopologyError> {
        let _ = self.face(face)?;
        self.attributes.set_face(face, attributes);
        Ok(())
    }

    /// The evolution journal (RFC 0003, Stage 1). Read-only; record through
    /// [`Self::journal_begin`] and the `journal_record_*` methods.
    #[must_use]
    pub fn journal(&self) -> &Journal {
        &self.journal
    }

    /// Opens a journaled operation, detecting unjournaled history first.
    ///
    /// If any mutation has happened since the journal's newest entry — an
    /// unjournaled operation, a failed operation's partial work before its
    /// rollback pattern was skipped, a direct edit — a synthetic
    /// **global barrier** entry
    /// ([`UNJOURNALED_MUTATIONS`](crate::journal::UNJOURNALED_MUTATIONS))
    /// is recorded before the returned token is issued, so no gap can
    /// impersonate continuity. An empty journal records no barrier:
    /// pre-journal history is absent by definition, not severed.
    ///
    /// The returned token is consumed by
    /// [`Self::journal_record_evolution`] or
    /// [`Self::journal_record_barrier`]. Dropping it without recording is
    /// safe — the operation's mutations surface as a gap at the next
    /// `journal_begin`.
    pub fn journal_begin(&mut self, kind: impl Into<String>) -> PendingOp {
        if let Some(last) = self.journal.last_ticks()
            && last != self.mutation_ticks
        {
            self.journal.record_global_barrier(self.mutation_ticks);
        }
        PendingOp {
            kind: kind.into(),
            scope: Vec::new(),
        }
    }

    /// Records the evolution entry for a journaled operation.
    ///
    /// Call after the operation completed, with the events it recorded
    /// while building its result. Entities the draft does not mention have
    /// no continuity across this operation — absent claims are gaps, not
    /// implicit preservation.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::JournalDuplicateEvent`] if the draft makes
    /// two claims about one entity; nothing is recorded.
    pub fn journal_record_evolution(
        &mut self,
        pending: PendingOp,
        draft: EvolutionDraft,
    ) -> Result<OpId, TopologyError> {
        self.journal
            .record_evolution(pending.kind, pending.scope, draft, self.mutation_ticks)
    }

    /// Copies face attributes forward across one journaled operation,
    /// driven by its entry's events (RFC 0003, Stage 4).
    ///
    /// The journal-driven generalization of the Issue 14 propagation
    /// rules, claim for claim:
    ///
    /// - **`Preserved` / `Modified`** subjects receive their source
    ///   face's attributes — a split's pieces are each still the same
    ///   semantic surface, so every piece gets the name **unchanged**
    ///   (the kernel never synthesizes, suffixes, or concatenates names).
    /// - **`Merged`** subjects receive attributes only when every
    ///   attributed input agrees; disagreeing inputs are a counted
    ///   conflict and the output stays bare — a merge does not toss coins
    ///   between names.
    /// - **`Generated`** and **`Unresolved`** subjects receive nothing
    ///   (`Unresolved` is counted) — an attribute never rides on a
    ///   guessed binding.
    /// - Inputs keep their own attributes (copy-forward only); non-face
    ///   subjects are skipped (the attribute store's v1 scope is solids
    ///   and faces).
    ///
    /// A geometry-derived entry propagates only when `allow_inferred` is
    /// set — riding attributes on inference is a policy the caller must
    /// opt into knowingly; a refusal is reported, not silent. Barrier
    /// entries carry nothing (they have no claims to ride).
    ///
    /// Attribute writes are not model mutations, so propagating between
    /// journaled operations never creates an unjournaled-mutation gap.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::RefUnknownOperation`] if `op` is not in
    /// the journal (never journaled, or truncated by a rollback).
    pub fn propagate_attributes_for_op(
        &mut self,
        op: OpId,
        allow_inferred: bool,
    ) -> Result<crate::journal::JournalAttributePropagation, TopologyError> {
        use crate::journal::{EntityEvent, EntityKind, EntryPayload, JournalAttributePropagation};

        let mut report = JournalAttributePropagation::default();
        let Some(entry) = self.journal.entries().iter().find(|entry| entry.op() == op) else {
            return Err(TopologyError::RefUnknownOperation { op: op.value() });
        };
        let (origin, events) = match entry.payload() {
            EntryPayload::Evolution { origin, events, .. } => (*origin, events),
            // A barrier has no claims to ride: nothing carries.
            EntryPayload::Barrier { .. } | EntryPayload::GlobalBarrier => return Ok(report),
        };
        if origin == crate::journal::RecordedOrigin::Geometry && !allow_inferred {
            report.refused_inferred = true;
            return Ok(report);
        }

        // Snapshot phase: read every carried attribute before mutating.
        // Events are subject-ordinal-sorted, so the pass is deterministic.
        let face_attributes = |journal_ordinal: crate::journal::JournalOrdinal| {
            self.journal
                .key_of(journal_ordinal)
                .filter(|key| key.kind == EntityKind::Face)
                .and_then(|key| self.faces.id_from_index(key.index))
                .and_then(|id| self.attributes.face(id))
        };
        let mut writes: Vec<(usize, crate::attributes::EntityAttributes)> = Vec::new();
        for (subject, event) in events {
            let Some(subject_key) = self.journal.key_of(*subject) else {
                continue;
            };
            if subject_key.kind != EntityKind::Face {
                continue;
            }
            match event {
                EntityEvent::Preserved { from } | EntityEvent::Modified { from } => {
                    if let Some(attributes) = face_attributes(*from) {
                        writes.push((subject_key.index, attributes.clone()));
                    }
                }
                EntityEvent::Merged { from } => {
                    let mut distinct: Vec<&crate::attributes::EntityAttributes> = Vec::new();
                    for &source in from {
                        if let Some(attributes) = face_attributes(source)
                            && !distinct.contains(&attributes)
                        {
                            distinct.push(attributes);
                        }
                    }
                    match distinct.as_slice() {
                        [] => {}
                        [agreed] => writes.push((subject_key.index, (*agreed).clone())),
                        _ => report.merge_conflicts += 1,
                    }
                }
                EntityEvent::Unresolved { .. } => report.unresolved_outputs += 1,
                EntityEvent::Generated { .. } | EntityEvent::Deleted => {}
            }
        }

        // Deterministic write order regardless of upstream ordering.
        writes.sort_by_key(|(index, _)| *index);
        for (index, attributes) in writes {
            if let Some(face) = self.faces.id_from_index(index)
                && self.set_face_attributes(face, attributes).is_ok()
            {
                report.carried += 1;
            }
        }
        Ok(report)
    }

    /// Installs a journal restored from a serialized snapshot
    /// (RFC 0003, Stage 5), replacing the current journal.
    ///
    /// The mutation counter is synced to the loaded journal's newest
    /// entry, so a topology restored together with its journal reads as
    /// consistent — a clean load is not an unjournaled gap. Any mutation
    /// after this call diverges the counter again and severs as usual.
    ///
    /// This is the deserialization path; installing a journal that does
    /// not describe this topology's history would fake continuity, and
    /// the caller (the document reader) owns that consistency.
    pub fn load_journal(&mut self, journal: Journal) {
        if let Some(ticks) = journal.last_ticks() {
            self.mutation_ticks = ticks;
        }
        self.journal = journal;
    }

    /// Records an explicit barrier entry for an operation that produces no
    /// evolution records: every entity in `affected` (the result's
    /// entities) is unresolved across it, and a resolver chasing a
    /// reference through this entry fails closed naming the operation.
    pub fn journal_record_barrier(&mut self, pending: PendingOp, affected: Vec<EntityKey>) -> OpId {
        self.journal
            .record_barrier(pending.kind, pending.scope, affected, self.mutation_ticks)
    }
    arena_get_mut!(
        compsolid_mut,
        compsolids,
        CompSolid,
        CompSolidId,
        CompSolidNotFound
    );

    arena_api!(
        add = add_vertex,
        arena = vertices,
        arena_fn = vertices,
        count = num_vertices,
        id_from_index = vertex_id_from_index,
        T = Vertex,
        Id = VertexId
    );

    arena_api!(
        add = add_edge,
        arena = edges,
        arena_fn = edges,
        count = num_edges,
        id_from_index = edge_id_from_index,
        T = Edge,
        Id = EdgeId
    );

    arena_api!(
        add = add_wire,
        arena = wires,
        arena_fn = wires,
        count = num_wires,
        id_from_index = wire_id_from_index,
        T = Wire,
        Id = WireId
    );

    arena_api!(
        add = add_face,
        arena = faces,
        arena_fn = faces,
        count = num_faces,
        id_from_index = face_id_from_index,
        T = Face,
        Id = FaceId
    );

    arena_api!(
        add = add_shell,
        arena = shells,
        arena_fn = shells,
        count = num_shells,
        id_from_index = shell_id_from_index,
        T = Shell,
        Id = ShellId
    );

    arena_api!(
        add = add_solid,
        arena = solids,
        arena_fn = solids,
        count = num_solids,
        id_from_index = solid_id_from_index,
        T = Solid,
        Id = SolidId
    );

    arena_api!(
        add = add_compound,
        arena = compounds,
        arena_fn = compounds,
        count = num_compounds,
        id_from_index = compound_id_from_index,
        T = Compound,
        Id = CompoundId
    );

    arena_api!(
        add = add_compsolid,
        arena = compsolids,
        arena_fn = compsolids,
        count = num_compsolids,
        id_from_index = compsolid_id_from_index,
        T = CompSolid,
        Id = CompSolidId
    );

    fn boundary_loop_specs(
        &self,
        wire_ids: &[WireId],
        replacement: Option<(WireId, &Wire)>,
    ) -> Result<Vec<BoundaryLoopSpec>, TopologyError> {
        let mut specs = Vec::with_capacity(wire_ids.len());
        for &wire_id in wire_ids {
            let wire = match replacement {
                Some((replacement_id, replacement_wire)) if replacement_id == wire_id => {
                    replacement_wire
                }
                _ => self.wire(wire_id)?,
            };
            for oriented in wire.edges() {
                self.edge(oriented.edge())?;
            }
            specs.push(BoundaryLoopSpec {
                oriented_edges: wire.edges().to_vec(),
                closed: wire.is_closed(),
            });
        }
        Ok(specs)
    }

    fn install_face_loop_specs(
        &mut self,
        face_id: FaceId,
        specs: Vec<BoundaryLoopSpec>,
    ) -> Vec<LoopId> {
        if let Some(old_loops) = self.face_loops.remove(&face_id) {
            for loop_id in old_loops {
                if let Some(old_loop) = self.loops.get(loop_id) {
                    for coedge_id in old_loop.coedges().to_vec() {
                        self.coedges.retire(coedge_id);
                    }
                }
                self.loops.retire(loop_id);
            }
        }

        let mut new_loops = Vec::with_capacity(specs.len());
        for spec in specs {
            let loop_id = self
                .loops
                .alloc(Loop::new(face_id, Vec::new(), spec.closed));
            let coedge_ids: Vec<CoedgeId> = spec
                .oriented_edges
                .iter()
                .map(|oriented| {
                    self.coedges
                        .alloc(Coedge::new(oriented.edge(), oriented.is_forward(), loop_id))
                })
                .collect();
            if let Some(loop_entity) = self.loops.get_mut(loop_id) {
                *loop_entity = Loop::new(face_id, coedge_ids, spec.closed);
            }
            new_loops.push(loop_id);
        }
        self.face_loops.insert(face_id, new_loops.clone());
        new_loops
    }

    fn retain_face_pcurves_for_specs(&mut self, face_id: FaceId, specs: &[BoundaryLoopSpec]) {
        let live_uses: HashSet<(EdgeId, bool)> = specs
            .iter()
            .flat_map(|spec| {
                spec.oriented_edges
                    .iter()
                    .map(|oriented| (oriented.edge(), oriented.is_forward()))
            })
            .collect();
        self.pcurves.retain_face_uses(face_id, &live_uses);
    }

    /// Atomically replaces a stored wire used by one or more face boundaries.
    ///
    /// The complete replacement is validated before any state changes. Every
    /// face that references `wire_id` keeps only pcurves for uses still present
    /// in its boundary, and an existing derived Loop/Coedge view is rebuilt in
    /// the same commit. A free wire can use this method too; with no owning
    /// faces, only the wire itself changes.
    ///
    /// This is the sanctioned RFC 0002 Stage-1 mutation path. It composes with
    /// [`transaction::run_transacted`](crate::transaction::run_transacted): an
    /// enclosing failed operation restores the prior wire, pcurves, and exact
    /// derived-loop handles while permanently retiring handles allocated by
    /// the rolled-back mutation.
    ///
    /// # Errors
    ///
    /// Returns a not-found error when `wire_id` or any edge referenced by the
    /// replacement is invalid. No topology state changes on error.
    pub fn replace_boundary_wire(
        &mut self,
        wire_id: WireId,
        replacement: Wire,
    ) -> Result<(), TopologyError> {
        self.wire(wire_id)?;

        let mut affected = Vec::new();
        for (face_id, face) in self.faces.iter() {
            if face.outer_wire() != wire_id && !face.inner_wires().contains(&wire_id) {
                continue;
            }
            let mut wire_ids = vec![face.outer_wire()];
            wire_ids.extend(face.inner_wires().iter().copied());
            let specs = self.boundary_loop_specs(&wire_ids, Some((wire_id, &replacement)))?;
            affected.push((face_id, self.face_loops.contains_key(&face_id), specs));
        }
        if affected.is_empty() {
            self.boundary_loop_specs(&[wire_id], Some((wire_id, &replacement)))?;
        }

        let stored = self
            .wires
            .get_mut(wire_id)
            .ok_or(TopologyError::WireNotFound(wire_id))?;
        *stored = replacement;
        self.mutation_ticks = self.mutation_ticks.saturating_add(1);

        for (face_id, had_derivation, specs) in affected {
            self.retain_face_pcurves_for_specs(face_id, &specs);
            if had_derivation {
                let _ = self.install_face_loop_specs(face_id, specs);
            }
        }
        Ok(())
    }

    /// Atomically replaces a face's complete outer/inner boundary-wire set.
    ///
    /// Every referenced wire and edge is validated before commit. Stale
    /// pcurve uses are removed and an existing derived Loop/Coedge view is
    /// rebuilt from the new outer-then-inner order without exposing an
    /// intermediate torn boundary.
    ///
    /// # Errors
    ///
    /// Returns a not-found error when the face, a replacement wire, or any
    /// referenced edge is invalid. No topology state changes on error.
    pub fn set_face_boundary_wires(
        &mut self,
        face_id: FaceId,
        outer_wire: WireId,
        inner_wires: Vec<WireId>,
    ) -> Result<(), TopologyError> {
        self.face(face_id)?;
        let mut wire_ids = vec![outer_wire];
        wire_ids.extend(inner_wires.iter().copied());
        let specs = self.boundary_loop_specs(&wire_ids, None)?;
        let had_derivation = self.face_loops.contains_key(&face_id);

        let face = self
            .faces
            .get_mut(face_id)
            .ok_or(TopologyError::FaceNotFound(face_id))?;
        face.replace_boundary_wires(outer_wire, inner_wires);
        self.mutation_ticks = self.mutation_ticks.saturating_add(1);
        self.retain_face_pcurves_for_specs(face_id, &specs);
        if had_derivation {
            let _ = self.install_face_loop_specs(face_id, specs);
        }
        Ok(())
    }

    /// Derives (or re-derives) this face's boundary loops and coedges from
    /// its current wires (RFC 0002, Stage 1).
    ///
    /// One loop is created per wire (outer first, then inner), with one
    /// coedge per oriented-edge occurrence — a seam edge used twice by one
    /// wire yields two coedges. A previous derivation for this face is
    /// retired first; its loop and coedge handles become permanently
    /// invalid (no slot reuse).
    ///
    /// Wires remain authoritative in Stage 1: this derivation is a view
    /// with stable handles, checked against its source by
    /// [`validate_face_loops`](crate::validation::validate_face_loops).
    ///
    /// # Errors
    ///
    /// Returns a not-found error if the face, any of its wires, or any
    /// referenced edge is invalid. Nothing is retired or allocated on
    /// error.
    pub fn build_face_loops(&mut self, face_id: FaceId) -> Result<Vec<LoopId>, TopologyError> {
        let face = self.face(face_id)?;
        let mut wire_ids = vec![face.outer_wire()];
        wire_ids.extend(face.inner_wires().iter().copied());
        let specs = self.boundary_loop_specs(&wire_ids, None)?;
        Ok(self.install_face_loop_specs(face_id, specs))
    }

    /// The loops previously derived for a face, in outer-then-inner order,
    /// or `None` when [`Self::build_face_loops`] has not been called for it.
    #[must_use]
    pub fn loops_of_face(&self, face_id: FaceId) -> Option<&[LoopId]> {
        self.face_loops.get(&face_id).map(Vec::as_slice)
    }

    /// Every live coedge use of the given edge, across all derived loops.
    ///
    /// A seam edge on a periodic face reports two uses; an edge shared by
    /// two faces reports one use per face boundary that has been derived.
    #[must_use]
    pub fn coedges_of_edge(&self, edge: EdgeId) -> Vec<CoedgeId> {
        self.coedges
            .iter()
            .filter(|(_, coedge)| coedge.edge() == edge)
            .map(|(id, _)| id)
            .collect()
    }

    /// Number of live loops.
    #[must_use]
    pub fn num_loops(&self) -> usize {
        self.loops.len()
    }

    /// Number of live coedges.
    #[must_use]
    pub fn num_coedges(&self) -> usize {
        self.coedges.len()
    }

    /// Retires a solid and every entity in its topology tree that no other
    /// live solid references.
    ///
    /// Retirement invalidates the solid handle and unshared shell, face,
    /// wire, edge, vertex, and pcurve handles. It does **not** compact the
    /// arenas or reclaim their allocated memory; future entities append to
    /// new slots so stale handles can never alias them.
    ///
    /// # Errors
    ///
    /// Returns [`DeleteSolidError`] if `solid` is invalid, if a live compound or
    /// comp-solid still references it, or if any live solid contains an
    /// invalid topology reference. No entities are retired when validation or
    /// reference discovery fails.
    pub fn delete_solid(&mut self, solid: SolidId) -> Result<(), DeleteSolidError> {
        self.mutation_ticks = self.mutation_ticks.saturating_add(1);
        let mut retiring = self.collect_solid_entities(solid)?;
        if let Some((compound_id, _)) = self
            .compounds
            .iter()
            .find(|(_, compound)| compound.solids().contains(&solid))
        {
            return Err(DeleteSolidError::Referenced {
                solid,
                dependent: "compound",
                dependent_index: compound_id.index(),
            });
        }
        if let Some((compsolid_id, _)) = self
            .compsolids
            .iter()
            .find(|(_, compsolid)| compsolid.solids().contains(&solid))
        {
            return Err(DeleteSolidError::Referenced {
                solid,
                dependent: "comp-solid",
                dependent_index: compsolid_id.index(),
            });
        }

        let mut retained = SolidEntities::default();
        for (other_id, _) in self.solids.iter() {
            if other_id != solid {
                self.collect_solid_entities_into(other_id, &mut retained)?;
            }
        }

        retiring
            .vertices
            .retain(|id| !retained.vertices.contains(id));
        retiring.edges.retain(|id| !retained.edges.contains(id));
        retiring.wires.retain(|id| !retained.wires.contains(id));
        retiring.faces.retain(|id| !retained.faces.contains(id));
        retiring.shells.retain(|id| !retained.shells.contains(id));

        for face_id in &retiring.faces {
            if let Some(loop_ids) = self.face_loops.remove(face_id) {
                for loop_id in loop_ids {
                    if let Some(retired_loop) = self.loops.get(loop_id) {
                        for coedge_id in retired_loop.coedges().to_vec() {
                            self.coedges.retire(coedge_id);
                        }
                    }
                    self.loops.retire(loop_id);
                }
            }
        }
        self.pcurves
            .remove_for_retired_entities(&retiring.edges, &retiring.faces);
        self.attributes
            .remove_for_retired_entities(&std::iter::once(solid).collect(), &retiring.faces);
        self.solids.retire(solid);
        for id in retiring.shells {
            self.shells.retire(id);
        }
        for id in retiring.faces {
            self.faces.retire(id);
        }
        for id in retiring.wires {
            self.wires.retire(id);
        }
        for id in retiring.edges {
            self.edges.retire(id);
        }
        for id in retiring.vertices {
            self.vertices.retire(id);
        }

        Ok(())
    }

    fn collect_solid_entities(&self, solid: SolidId) -> Result<SolidEntities, TopologyError> {
        let mut entities = SolidEntities::default();
        self.collect_solid_entities_into(solid, &mut entities)?;
        Ok(entities)
    }

    fn collect_solid_entities_into(
        &self,
        solid: SolidId,
        entities: &mut SolidEntities,
    ) -> Result<(), TopologyError> {
        let solid_data = self.solid(solid)?;
        for shell_id in std::iter::once(solid_data.outer_shell())
            .chain(solid_data.inner_shells().iter().copied())
        {
            if !entities.shells.insert(shell_id) {
                continue;
            }
            let shell = self.shell(shell_id)?;
            for &face_id in shell.faces() {
                if !entities.faces.insert(face_id) {
                    continue;
                }
                let face = self.face(face_id)?;
                for wire_id in
                    std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
                {
                    if !entities.wires.insert(wire_id) {
                        continue;
                    }
                    let wire = self.wire(wire_id)?;
                    for oriented_edge in wire.edges() {
                        let edge_id = oriented_edge.edge();
                        if !entities.edges.insert(edge_id) {
                            continue;
                        }
                        let edge = self.edge(edge_id)?;
                        self.vertex(edge.start())?;
                        self.vertex(edge.end())?;
                        entities.vertices.insert(edge.start());
                        entities.vertices.insert(edge.end());
                    }
                }
            }
        }

        Ok(())
    }

    /// Allocates an empty-result solid: a solid backed by a faceless
    /// [`Shell::empty`].
    ///
    /// Booleans whose algebraic outcome is the empty set (e.g. the
    /// intersection of disjoint solids) return this so the result is a
    /// valid, queryable handle reporting zero faces and zero volume,
    /// distinct from a malformed-input error. A shell cannot otherwise
    /// hold zero faces, so this is the only path that produces one.
    pub fn add_empty_solid(&mut self) -> SolidId {
        let shell = self.add_shell(Shell::empty());
        self.add_solid(Solid::new(shell, Vec::new()))
    }

    /// Returns `true` when `solid` is an empty-result sentinel — its
    /// outer shell is faceless and it has no inner shells (see
    /// [`Topology::add_empty_solid`]).
    #[must_use]
    pub fn is_empty_solid(&self, solid: SolidId) -> bool {
        self.solids.get(solid).is_some_and(|s| {
            s.inner_shells().is_empty()
                && self
                    .shells
                    .get(s.outer_shell())
                    .is_some_and(Shell::is_empty)
        })
    }

    /// The pcurve of one edge use, addressed exactly by orientation.
    ///
    /// This — not the `(edge, face)` pair — is how the two parameter-space
    /// branches of a seam edge are addressed (RFC 0002, Stage 2).
    #[must_use]
    pub fn pcurve_oriented(&self, edge: EdgeId, face: FaceId, forward: bool) -> Option<&PCurve> {
        self.pcurves.get_use(edge, face, forward)
    }

    /// Sets the pcurve of one edge use, addressed exactly by orientation.
    pub fn set_pcurve_oriented(
        &mut self,
        edge: EdgeId,
        face: FaceId,
        forward: bool,
        pcurve: PCurve,
    ) {
        self.mutation_ticks = self.mutation_ticks.saturating_add(1);
        self.pcurves.set_use(edge, face, forward, pcurve);
    }

    /// Removes the pcurve of one edge use.
    pub fn remove_pcurve_oriented(
        &mut self,
        edge: EdgeId,
        face: FaceId,
        forward: bool,
    ) -> Option<PCurve> {
        self.mutation_ticks = self.mutation_ticks.saturating_add(1);
        self.pcurves.remove_use(edge, face, forward)
    }

    /// The pcurve of `edge` on `face`, when that pair identifies at most
    /// one stored use.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::SeamPcurveAmbiguous`] when both orientation
    /// branches are stored (a seam edge): the pair no longer identifies a
    /// use, and answering with either branch would be arbitrary. Seam-aware
    /// callers address the use with [`Self::pcurve_oriented`].
    pub fn pcurve(&self, edge: EdgeId, face: FaceId) -> Result<Option<&PCurve>, TopologyError> {
        let uses = self.pcurves.uses_on_face(edge, face);
        match uses.as_slice() {
            [] => Ok(None),
            [(_, pcurve)] => Ok(Some(pcurve)),
            _ => Err(TopologyError::SeamPcurveAmbiguous { edge, face }),
        }
    }

    /// Whether `edge` has a pcurve on `face`.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::SeamPcurveAmbiguous`] when both orientation
    /// branches are stored; a bare boolean would hide the seam.
    pub fn has_pcurve(&self, edge: EdgeId, face: FaceId) -> Result<bool, TopologyError> {
        match self.pcurves.uses_on_face(edge, face).len() {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(TopologyError::SeamPcurveAmbiguous { edge, face }),
        }
    }

    /// Sets the pcurve of `edge` on `face`, resolving which use it is.
    ///
    /// Resolution: if the face's wires use the edge exactly once, the entry
    /// is stored under that use's orientation and any stale opposite entry
    /// is replaced (preserving the legacy single-slot overwrite semantics).
    /// If the edge is not currently in the boundary (construction-order
    /// tolerance), an existing single entry is overwritten in place, or a
    /// new entry is stored under the forward orientation.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::SeamPcurveAmbiguous`] when the face uses
    /// the edge twice (a seam): the pair does not identify a use, and the
    /// legacy behavior — silently destroying one branch — is exactly the
    /// defect this API retires. Callers storing seam branches use
    /// [`Self::set_pcurve_oriented`]. Returns a not-found error when the
    /// face or one of its wires is invalid.
    pub fn set_pcurve(
        &mut self,
        edge: EdgeId,
        face: FaceId,
        pcurve: PCurve,
    ) -> Result<(), TopologyError> {
        let uses = self.face_edge_uses(edge, face)?;
        let forward = match uses.as_slice() {
            [forward] => *forward,
            [] => match self.pcurves.uses_on_face(edge, face).as_slice() {
                [] => true,
                [(forward, _)] => *forward,
                _ => return Err(TopologyError::SeamPcurveAmbiguous { edge, face }),
            },
            _ => return Err(TopologyError::SeamPcurveAmbiguous { edge, face }),
        };
        self.mutation_ticks = self.mutation_ticks.saturating_add(1);
        self.pcurves.remove_use(edge, face, !forward);
        self.pcurves.set_use(edge, face, forward, pcurve);
        Ok(())
    }

    /// Removes the pcurve of `edge` on `face`, when the pair identifies at
    /// most one stored use.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::SeamPcurveAmbiguous`] when both orientation
    /// branches are stored.
    pub fn remove_pcurve(
        &mut self,
        edge: EdgeId,
        face: FaceId,
    ) -> Result<Option<PCurve>, TopologyError> {
        let stored: Vec<bool> = self
            .pcurves
            .uses_on_face(edge, face)
            .iter()
            .map(|(forward, _)| *forward)
            .collect();
        match stored.as_slice() {
            [] => Ok(None),
            [forward] => {
                self.mutation_ticks = self.mutation_ticks.saturating_add(1);
                Ok(self.pcurves.remove_use(edge, face, *forward))
            }
            _ => Err(TopologyError::SeamPcurveAmbiguous { edge, face }),
        }
    }

    /// All stored pcurve uses for a face: `(edge, forward, pcurve)`, in
    /// deterministic (edge index, forward-first) order. A seam edge yields
    /// two entries.
    #[must_use]
    pub fn pcurves_for_face(&self, face: FaceId) -> Vec<(EdgeId, bool, &PCurve)> {
        self.pcurves.uses_for_face(face)
    }

    /// All stored pcurve uses for an edge: `(face, forward, pcurve)`, in
    /// deterministic order.
    #[must_use]
    pub fn pcurves_for_edge(&self, edge: EdgeId) -> Vec<(FaceId, bool, &PCurve)> {
        self.pcurves.uses_for_edge(edge)
    }

    /// Number of stored pcurve uses.
    #[must_use]
    pub fn num_pcurves(&self) -> usize {
        self.pcurves.len()
    }

    /// The pcurve of one derived coedge use, resolved through its owning
    /// loop's face and its orientation.
    ///
    /// # Errors
    ///
    /// Returns a not-found error when the coedge or its loop is stale.
    pub fn coedge_pcurve(&self, coedge_id: CoedgeId) -> Result<Option<&PCurve>, TopologyError> {
        let coedge = self.coedge(coedge_id)?;
        let face = self.face_loop(coedge.parent_loop())?.face();
        Ok(self
            .pcurves
            .get_use(coedge.edge(), face, coedge.is_forward()))
    }

    /// The orientations with which `face`'s wires use `edge`, in boundary
    /// order (outer wire first).
    fn face_edge_uses(&self, edge: EdgeId, face: FaceId) -> Result<Vec<bool>, TopologyError> {
        let face_data = self.face(face)?;
        let mut wire_ids = vec![face_data.outer_wire()];
        wire_ids.extend(face_data.inner_wires().iter().copied());
        let mut uses = Vec::new();
        for wire_id in wire_ids {
            let wire = self.wire(wire_id)?;
            for oriented in wire.edges() {
                if oriented.edge() == edge {
                    uses.push(oriented.is_forward());
                }
            }
        }
        Ok(uses)
    }

    /// Builds an adjacency index for the given solid.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError`] if any referenced entity does not exist.
    pub fn build_adjacency(&self, solid: SolidId) -> Result<AdjacencyIndex, TopologyError> {
        AdjacencyIndex::build(self, solid)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};

    use crate::compound::Compound;
    use crate::compsolid::CompSolid;
    use crate::edge::{Edge, EdgeCurve};
    use crate::face::{Face, FaceSurface};
    use crate::pcurve::PCurve;
    use crate::shell::Shell;
    use crate::solid::Solid;
    use crate::wire::{OrientedEdge, Wire};

    use super::*;

    fn make_triangle_solid(topo: &mut Topology, x_offset: f64) -> (SolidId, FaceId, EdgeId) {
        let v0 = topo.add_vertex(Vertex::new(Point3::new(x_offset, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(x_offset + 1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(x_offset, 1.0, 0.0), 1e-7));
        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        let wire = Wire::new(
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, true),
                OrientedEdge::new(e2, true),
            ],
            true,
        )
        .unwrap();
        let wire_id = topo.add_wire(wire);
        let face = topo.add_face(Face::new(
            wire_id,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        (topo.add_solid(Solid::new(shell, Vec::new())), face, e0)
    }

    fn wire_signature(topo: &Topology, wire: WireId) -> Vec<(EdgeId, bool)> {
        topo.wire(wire)
            .unwrap()
            .edges()
            .iter()
            .map(|oriented| (oriented.edge(), oriented.is_forward()))
            .collect()
    }

    fn test_pcurve(offset: f64) -> PCurve {
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(offset, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
            0.0,
            1.0,
        )
    }

    #[test]
    fn sanctioned_wire_replacement_rederives_loops_and_prunes_stale_pcurves() {
        let mut topo = Topology::new();
        let (_, face, edge) = make_triangle_solid(&mut topo, 0.0);
        let wire = topo.face(face).unwrap().outer_wire();
        topo.set_pcurve_oriented(edge, face, true, test_pcurve(0.0));
        topo.set_pcurve_oriented(edge, face, false, test_pcurve(10.0));
        let retired_loops = topo.build_face_loops(face).unwrap();
        let retired_coedges = topo.face_loop(retired_loops[0]).unwrap().coedges().to_vec();

        let reversed: Vec<_> = topo
            .wire(wire)
            .unwrap()
            .edges()
            .iter()
            .rev()
            .map(|oriented| OrientedEdge::new(oriented.edge(), !oriented.is_forward()))
            .collect();
        topo.replace_boundary_wire(wire, Wire::new(reversed, true).unwrap())
            .unwrap();

        assert!(topo.pcurve_oriented(edge, face, true).is_none());
        assert!(topo.pcurve_oriented(edge, face, false).is_some());
        for retired in retired_loops {
            assert!(topo.face_loop(retired).is_err());
        }
        for retired in retired_coedges {
            assert!(topo.coedge(retired).is_err());
        }
        crate::validation::validate_face_loops(&topo, face).unwrap();
        let loop_id = topo.loops_of_face(face).unwrap()[0];
        let loop_ = topo.face_loop(loop_id).unwrap();
        assert_eq!(loop_.coedges().len(), 3);
        assert_eq!(
            topo.coedge(loop_.coedges()[0]).unwrap().is_forward(),
            topo.wire(wire).unwrap().edges()[0].is_forward()
        );
    }

    #[test]
    fn sanctioned_face_boundary_replacement_is_preflight_atomic() {
        let mut topo = Topology::new();
        let (_, face, edge) = make_triangle_solid(&mut topo, 0.0);
        let original_wire = topo.face(face).unwrap().outer_wire();
        topo.set_pcurve_oriented(edge, face, true, test_pcurve(0.0));
        let original_loops = topo.build_face_loops(face).unwrap();
        let counts = (topo.num_wires(), topo.num_loops(), topo.num_coedges());

        let snapshot = topo.clone();
        let stale_wire =
            topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        topo.restore_for_rollback(&snapshot);
        assert!(matches!(
            topo.set_face_boundary_wires(face, stale_wire, Vec::new()),
            Err(TopologyError::WireNotFound(id)) if id == stale_wire
        ));

        assert_eq!(topo.face(face).unwrap().outer_wire(), original_wire);
        assert_eq!(topo.loops_of_face(face).unwrap(), original_loops.as_slice());
        assert!(topo.pcurve_oriented(edge, face, true).is_some());
        assert_eq!(
            (topo.num_wires(), topo.num_loops(), topo.num_coedges()),
            counts
        );
        crate::validation::validate_face_loops(&topo, face).unwrap();
    }

    #[test]
    fn sanctioned_face_boundary_replacement_commits_one_coherent_loop_set() {
        let mut topo = Topology::new();
        let (_, face, _) = make_triangle_solid(&mut topo, 0.0);
        let original_wire = topo.face(face).unwrap().outer_wire();
        let old_loops = topo.build_face_loops(face).unwrap();
        let (_, replacement_face, _) = make_triangle_solid(&mut topo, 10.0);
        let replacement_wire = topo.face(replacement_face).unwrap().outer_wire();

        topo.set_face_boundary_wires(face, replacement_wire, vec![original_wire])
            .unwrap();

        let boundary = topo.face(face).unwrap();
        assert_eq!(boundary.outer_wire(), replacement_wire);
        assert_eq!(boundary.inner_wires(), &[original_wire]);
        for retired in old_loops {
            assert!(topo.face_loop(retired).is_err());
        }
        let loops = topo.loops_of_face(face).unwrap();
        assert_eq!(loops.len(), 2);
        assert_eq!(topo.face_loop(loops[0]).unwrap().coedges().len(), 3);
        assert_eq!(topo.face_loop(loops[1]).unwrap().coedges().len(), 3);
        crate::validation::validate_face_loops(&topo, face).unwrap();
    }

    #[test]
    fn enclosing_checkpoint_rolls_back_sanctioned_boundary_mutation_exactly() {
        let mut topo = Topology::new();
        let (_, face, edge) = make_triangle_solid(&mut topo, 0.0);
        let wire = topo.face(face).unwrap().outer_wire();
        topo.set_pcurve_oriented(edge, face, true, test_pcurve(0.0));
        let original_signature = wire_signature(&topo, wire);
        let original_loops = topo.build_face_loops(face).unwrap();
        let original_coedges = topo
            .face_loop(original_loops[0])
            .unwrap()
            .coedges()
            .to_vec();
        let counts = (topo.num_wires(), topo.num_loops(), topo.num_coedges());
        let mut rolled_back_loops = Vec::new();

        let error = crate::transaction::run_transacted(&mut topo, |topo| {
            let reversed: Vec<_> = topo
                .wire(wire)?
                .edges()
                .iter()
                .rev()
                .map(|oriented| OrientedEdge::new(oriented.edge(), !oriented.is_forward()))
                .collect();
            topo.replace_boundary_wire(wire, Wire::new(reversed, true)?)?;
            rolled_back_loops = topo.loops_of_face(face).unwrap().to_vec();
            Err::<(), _>(TopologyError::WireNotClosed)
        })
        .unwrap_err();
        assert!(matches!(error, TopologyError::WireNotClosed));

        assert_eq!(wire_signature(&topo, wire), original_signature);
        assert_eq!(topo.loops_of_face(face).unwrap(), original_loops.as_slice());
        for loop_id in &original_loops {
            assert!(topo.face_loop(*loop_id).is_ok());
        }
        for coedge_id in original_coedges {
            assert!(topo.coedge(coedge_id).is_ok());
        }
        for loop_id in rolled_back_loops {
            assert!(topo.face_loop(loop_id).is_err());
        }
        assert!(topo.pcurve_oriented(edge, face, true).is_some());
        assert_eq!(
            (topo.num_wires(), topo.num_loops(), topo.num_coedges()),
            counts
        );
        crate::validation::validate_face_loops(&topo, face).unwrap();
    }

    #[test]
    fn allocate_and_lookup_vertex() {
        let mut topo = Topology::new();
        let vid = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));

        let v = topo.vertex(vid).unwrap();
        assert!((v.point().x() - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn clone_preserves_entities() {
        let mut topo = Topology::new();
        let vid = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));

        let snapshot = topo.clone();

        topo.add_vertex(Vertex::new(Point3::new(4.0, 5.0, 6.0), 1e-7));
        assert_eq!(topo.num_vertices(), 2);

        assert_eq!(snapshot.num_vertices(), 1);
        let v = snapshot.vertex(vid).unwrap();
        assert!((v.point().x() - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn restore_from_clone() {
        let mut topo = Topology::new();
        let vid = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));

        let snapshot = topo.clone();

        topo.add_vertex(Vertex::new(Point3::new(9.0, 9.0, 9.0), 1e-7));

        topo = snapshot;
        assert_eq!(topo.num_vertices(), 1);
        let v = topo.vertex(vid).unwrap();
        assert!((v.point().x() - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn restore_preserving_handle_slots_does_not_alias_retired_ids() {
        let mut topo = Topology::new();
        let original = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));
        let snapshot = topo.clone();
        let stale = topo.add_vertex(Vertex::new(Point3::new(4.0, 5.0, 6.0), 1e-7));

        topo.restore_preserving_handle_slots(&snapshot);
        assert!(topo.vertex(stale).is_err());
        assert_eq!(topo.num_vertices(), 1);

        let fresh = topo.add_vertex(Vertex::new(Point3::new(7.0, 8.0, 9.0), 1e-7));
        assert!(fresh.index() > stale.index());
        assert!(topo.vertex(stale).is_err());
        assert!(topo.vertex(original).is_ok());
        assert_eq!(topo.num_vertices(), 2);
    }

    #[test]
    fn invalid_id_returns_error() {
        use crate::arena::Id;
        let topo = Topology::new();
        let mut dummy_arena: Arena<Vertex> = Arena::new();
        let vid = dummy_arena.alloc(Vertex::new(Point3::new(0.0, 0.0, 0.0), 0.0));
        let _ = Id::<Vertex>::index(vid);
        assert!(topo.vertex(vid).is_err());
    }

    #[test]
    fn arena_accessors_and_counts() {
        let mut topo = Topology::new();
        assert_eq!(topo.num_vertices(), 0);
        assert!(topo.vertices().is_empty());

        let vid = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));
        assert_eq!(topo.num_vertices(), 1);
        assert!(topo.vertices().get(vid).is_some());
    }

    #[test]
    fn id_from_index_roundtrip() {
        let mut topo = Topology::new();
        let vid = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));

        let reconstructed = topo.vertex_id_from_index(vid.index()).unwrap();
        assert_eq!(reconstructed, vid);
        assert!(topo.vertex_id_from_index(999).is_none());
    }

    #[test]
    fn reserve_preserves_existing_entities() {
        let mut topo = Topology::new();
        let vid = topo.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));
        assert_eq!(topo.num_vertices(), 1);

        topo.reserve(100, 50, 25, 25, 2, 2);
        assert_eq!(topo.num_vertices(), 1);

        let v = topo.vertex(vid).unwrap();
        assert!((v.point().x() - 1.0).abs() < f64::EPSILON);

        let vid2 = topo.add_vertex(Vertex::new(Point3::new(4.0, 5.0, 6.0), 1e-7));
        assert_eq!(topo.num_vertices(), 2);
        let v2 = topo.vertex(vid2).unwrap();
        assert!((v2.point().x() - 4.0).abs() < f64::EPSILON);
    }

    #[test]
    fn delete_solid_retires_only_its_unshared_subtree_and_pcurves() {
        let mut topo = Topology::new();
        let (deleted, deleted_face, deleted_edge) = make_triangle_solid(&mut topo, 0.0);
        let (kept, kept_face, kept_edge) = make_triangle_solid(&mut topo, 10.0);
        let line = || {
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
                0.0,
                1.0,
            )
        };
        topo.set_pcurve(deleted_edge, deleted_face, line()).unwrap();
        topo.set_pcurve(kept_edge, kept_face, line()).unwrap();
        let deleted_entities = topo.collect_solid_entities(deleted).unwrap();

        topo.delete_solid(deleted).unwrap();

        assert!(topo.solid(deleted).is_err());
        assert!(topo.solid(kept).is_ok());
        assert_eq!(topo.num_solids(), 1);
        assert_eq!(topo.num_shells(), 1);
        assert_eq!(topo.num_faces(), 1);
        assert_eq!(topo.num_wires(), 1);
        assert_eq!(topo.num_edges(), 3);
        assert_eq!(topo.num_vertices(), 3);
        assert_eq!(topo.num_pcurves(), 1);
        assert!(!topo.has_pcurve(deleted_edge, deleted_face).unwrap());
        assert!(topo.has_pcurve(kept_edge, kept_face).unwrap());
        assert!(
            deleted_entities
                .shells
                .iter()
                .all(|&id| topo.shell(id).is_err())
        );
        assert!(
            deleted_entities
                .faces
                .iter()
                .all(|&id| topo.face(id).is_err())
        );
        assert!(
            deleted_entities
                .wires
                .iter()
                .all(|&id| topo.wire(id).is_err())
        );
        assert!(
            deleted_entities
                .edges
                .iter()
                .all(|&id| topo.edge(id).is_err())
        );
        assert!(
            deleted_entities
                .vertices
                .iter()
                .all(|&id| topo.vertex(id).is_err())
        );
    }

    #[test]
    fn delete_solid_preserves_entities_referenced_by_another_live_solid() {
        let mut topo = Topology::new();
        let (first, face, edge) = make_triangle_solid(&mut topo, 0.0);
        let shell = topo.solid(first).unwrap().outer_shell();
        let second = topo.add_solid(Solid::new(shell, Vec::new()));

        topo.delete_solid(first).unwrap();

        assert!(topo.solid(first).is_err());
        assert!(topo.solid(second).is_ok());
        assert!(topo.shell(shell).is_ok());
        assert!(topo.face(face).is_ok());
        assert!(topo.edge(edge).is_ok());
        assert_eq!(topo.num_solids(), 1);
        assert_eq!(topo.num_shells(), 1);
        assert_eq!(topo.num_faces(), 1);
        assert_eq!(topo.num_wires(), 1);
        assert_eq!(topo.num_edges(), 3);
        assert_eq!(topo.num_vertices(), 3);

        topo.delete_solid(second).unwrap();
        assert_eq!(topo.num_solids(), 0);
        assert_eq!(topo.num_shells(), 0);
        assert_eq!(topo.num_faces(), 0);
        assert_eq!(topo.num_wires(), 0);
        assert_eq!(topo.num_edges(), 0);
        assert_eq!(topo.num_vertices(), 0);
    }

    #[test]
    fn delete_solid_walks_outer_and_inner_shells() {
        let mut topo = Topology::new();
        let (outer_owner, _, _) = make_triangle_solid(&mut topo, 0.0);
        let (inner_owner, _, _) = make_triangle_solid(&mut topo, 10.0);
        let outer_shell = topo.solid(outer_owner).unwrap().outer_shell();
        let inner_shell = topo.solid(inner_owner).unwrap().outer_shell();
        let hollow = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));

        topo.delete_solid(outer_owner).unwrap();
        topo.delete_solid(inner_owner).unwrap();
        assert_eq!(topo.num_solids(), 1);
        assert_eq!(topo.num_shells(), 2);
        assert_eq!(topo.num_faces(), 2);
        assert_eq!(topo.num_wires(), 2);
        assert_eq!(topo.num_edges(), 6);
        assert_eq!(topo.num_vertices(), 6);

        topo.delete_solid(hollow).unwrap();
        assert_eq!(topo.num_solids(), 0);
        assert_eq!(topo.num_shells(), 0);
        assert_eq!(topo.num_faces(), 0);
        assert_eq!(topo.num_wires(), 0);
        assert_eq!(topo.num_edges(), 0);
        assert_eq!(topo.num_vertices(), 0);
    }

    #[test]
    fn delete_solid_deduplicates_shared_and_repeated_shell_roots() {
        let mut topo = Topology::new();
        let (owner, face, edge) = make_triangle_solid(&mut topo, 0.0);
        let shell = topo.solid(owner).unwrap().outer_shell();
        let repeated = topo.add_solid(Solid::new(shell, vec![shell; 1_000]));
        let shared = topo.add_solid(Solid::new(shell, Vec::new()));

        topo.delete_solid(owner).unwrap();
        topo.delete_solid(repeated).unwrap();

        assert!(topo.solid(shared).is_ok());
        assert!(topo.shell(shell).is_ok());
        assert!(topo.face(face).is_ok());
        assert!(topo.edge(edge).is_ok());

        topo.delete_solid(shared).unwrap();
        assert_eq!(topo.num_solids(), 0);
        assert_eq!(topo.num_shells(), 0);
        assert_eq!(topo.num_faces(), 0);
        assert_eq!(topo.num_wires(), 0);
        assert_eq!(topo.num_edges(), 0);
        assert_eq!(topo.num_vertices(), 0);
    }

    #[test]
    fn delete_solid_never_reuses_retired_slots() {
        let mut topo = Topology::new();
        let (stale, _, _) = make_triangle_solid(&mut topo, 0.0);

        topo.delete_solid(stale).unwrap();
        let (fresh, _, _) = make_triangle_solid(&mut topo, 10.0);

        assert!(fresh.index() > stale.index());
        assert!(topo.solid(stale).is_err());
        assert!(topo.solid(fresh).is_ok());
    }

    #[test]
    fn delete_solid_rejects_live_compound_and_compsolid_roots() {
        let mut compound_topo = Topology::new();
        let (compound_solid, _, _) = make_triangle_solid(&mut compound_topo, 0.0);
        let compound = compound_topo.add_compound(Compound::new(vec![compound_solid]));

        assert!(matches!(
            compound_topo.delete_solid(compound_solid),
            Err(DeleteSolidError::Referenced {
                dependent: "compound",
                ..
            })
        ));
        assert_eq!(
            compound_topo.compound(compound).unwrap().solids(),
            &[compound_solid]
        );
        assert!(compound_topo.solid(compound_solid).is_ok());

        let mut compsolid_topo = Topology::new();
        let (compsolid_solid, _, _) = make_triangle_solid(&mut compsolid_topo, 0.0);
        let compsolid = compsolid_topo.add_compsolid(CompSolid::new(vec![compsolid_solid], vec![]));

        assert!(matches!(
            compsolid_topo.delete_solid(compsolid_solid),
            Err(DeleteSolidError::Referenced {
                dependent: "comp-solid",
                ..
            })
        ));
        assert_eq!(
            compsolid_topo.compsolid(compsolid).unwrap().solids(),
            &[compsolid_solid]
        );
        assert!(compsolid_topo.solid(compsolid_solid).is_ok());
    }

    #[test]
    fn restore_preserves_deleted_solid_and_pcurve_retirement() {
        let mut topo = Topology::new();
        let (retired, face, edge) = make_triangle_solid(&mut topo, 0.0);
        topo.set_pcurve(
            edge,
            face,
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
                0.0,
                1.0,
            ),
        )
        .unwrap();
        let snapshot = topo.clone();

        topo.delete_solid(retired).unwrap();
        topo.restore_preserving_handle_slots(&snapshot);

        assert!(topo.solid(retired).is_err());
        assert!(topo.face(face).is_err());
        assert!(topo.edge(edge).is_err());
        assert!(!topo.has_pcurve(edge, face).unwrap());
        let (fresh, _, _) = make_triangle_solid(&mut topo, 10.0);
        assert!(fresh.index() > retired.index());
    }
}
