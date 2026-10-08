//! Mutation-local rollback storage (PERF-T02) and the append-only guard (PERF-T03).
//!
//! This module is a child of [`Topology`](crate::Topology), so it may touch
//! the private arenas directly. All cross-module access goes through the
//! `pub(crate)` methods here; [`transaction`](crate::transaction) drives
//! scopes through those methods and never touches arena fields.
//!
//! # Cost model
//!
//! A scope mark is O(1): three lengths and a tick count. Every record is
//! proportional to state the scope actually wrote: one cloned entity per
//! overwritten slot, one `(arena, index)` pair per allocation, one old key
//! value per registry/attribute write, and the created ordinal pairs per
//! journal record. A changed-clone or unrelated full restore inside a live
//! scope takes one document-sized inverse; ordinary mutation-local scopes
//! do not. Records live only while scopes are active: the last commit clears
//! the log, and a rewind drops everything above its mark.
//!
//! # Soundness argument
//!
//! The log and the live state satisfy one invariant: popping every record
//! above a surviving mark's length and applying the inverses in reverse
//! yields exactly the mark's entry state. The invariant holds initially
//! (empty log) and is preserved because every mutation route in
//! [`Topology`](crate::Topology) records before writing (see the write-guard
//! rule in `docs/design/perf-t02-t03-transactions.md`), rewinds apply exact
//! inverses, and an unchanged same-lineage restore truncates to its source
//! prefix. A changed clone or unrelated restore instead records the complete
//! pre-restore state so older records remain valid after that inverse lands.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::attributes::EntityAttributes;
use crate::coedge::{Coedge, CoedgeId};
use crate::compound::Compound;
use crate::compsolid::CompSolid;
use crate::edge::{Edge, EdgeId};
use crate::face::{Face, FaceId};
use crate::face_loop::{Loop, LoopId};
use crate::journal::{EntityKey, Journal, JournalOrdinal};
use crate::pcurve::PCurveKey;
use crate::shell::{Shell, ShellId};
use crate::solid::{Solid, SolidId};
use crate::vertex::Vertex;
use crate::wire::{Wire, WireId};
use crate::{Topology, TopologyError, VertexId};

/// Arena tags in the fixed order used by [`AppendGuard::slot_marks`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArenaTag {
    Vertex,
    Edge,
    Wire,
    Face,
    Shell,
    Solid,
    Compound,
    CompSolid,
    Loop,
    Coedge,
}

impl ArenaTag {
    /// Short name used by the append-only guard trip error.
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Self::Vertex => "vertex",
            Self::Edge => "edge",
            Self::Wire => "wire",
            Self::Face => "face",
            Self::Shell => "shell",
            Self::Solid => "solid",
            Self::Compound => "compound",
            Self::CompSolid => "comp-solid",
            Self::Loop => "loop",
            Self::Coedge => "coedge",
        }
    }
}

/// One reversible state change, recorded before the write lands.
///
/// Slot records carry the previous live value (`Some`) or mark a fresh
/// allocation (`None`, whose inverse retires the slot and keeps it as a
/// tombstone so the handle stays stale forever). Key records carry the
/// previous index/attribute value (`None` means the key was absent).
#[derive(Debug, Clone)]
pub enum UndoRecord {
    Vertex {
        index: usize,
        old: Option<Vertex>,
    },
    Edge {
        index: usize,
        old: Option<Edge>,
    },
    Wire {
        index: usize,
        old: Option<Wire>,
    },
    Face {
        index: usize,
        old: Option<Face>,
    },
    Shell {
        index: usize,
        old: Option<Shell>,
    },
    Solid {
        index: usize,
        old: Option<Solid>,
    },
    Compound {
        index: usize,
        old: Option<Compound>,
    },
    CompSolid {
        index: usize,
        old: Option<CompSolid>,
    },
    Loop {
        index: usize,
        old: Option<Loop>,
    },
    Coedge {
        index: usize,
        old: Option<Coedge>,
    },
    PcurveUse {
        key: PCurveKey,
        old: Option<CoedgeId>,
    },
    SolidAttribute {
        id: SolidId,
        old: Option<EntityAttributes>,
    },
    FaceAttribute {
        id: FaceId,
        old: Option<EntityAttributes>,
    },
    /// Ordinal pairs one journal record call created; removed on rewind.
    JournalCreated {
        pairs: Vec<(EntityKey, JournalOrdinal)>,
    },
    /// Whole journal installed by `load_journal` with scopes active.
    JournalReplace {
        old: Journal,
    },
    /// State before a changed or unrelated full restore inside a live scope.
    /// This rare path preserves the outer scope's complete rollback contract.
    ForeignRestore {
        old: Box<Topology>,
    },
}

impl UndoRecord {
    /// Builds the allocation record for `tag` (no previous value).
    pub(crate) fn alloc(tag: ArenaTag, index: usize) -> Self {
        match tag {
            ArenaTag::Vertex => Self::Vertex { index, old: None },
            ArenaTag::Edge => Self::Edge { index, old: None },
            ArenaTag::Wire => Self::Wire { index, old: None },
            ArenaTag::Face => Self::Face { index, old: None },
            ArenaTag::Shell => Self::Shell { index, old: None },
            ArenaTag::Solid => Self::Solid { index, old: None },
            ArenaTag::Compound => Self::Compound { index, old: None },
            ArenaTag::CompSolid => Self::CompSolid { index, old: None },
            ArenaTag::Loop => Self::Loop { index, old: None },
            ArenaTag::Coedge => Self::Coedge { index, old: None },
        }
    }
}

/// A scope's entry position: log length, journal length, and tick count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UndoMark {
    /// Sequence number giving marks a total order (equal log lengths occur
    /// when scopes nest without intervening writes).
    pub(crate) seq: u64,
    /// Undo-log length at capture.
    pub(crate) log_len: usize,
    /// Journal entry count at capture.
    pub(crate) journal_len: usize,
    /// Mutation-tick count at capture.
    pub(crate) ticks: u64,
}

/// One live scope entry in the topology's scope stack.
#[derive(Debug, Clone)]
pub struct ActiveScope {
    /// The scope's entry position.
    pub(crate) mark: UndoMark,
    /// Liveness token shared with the owning snapshot: dropping the
    /// snapshot leaves a dead entry that the next transaction access
    /// reclaims as a commit (records merge into the enclosing scope).
    pub(crate) alive: Arc<()>,
}

/// The owning half of a scope entry, handed to the capturing snapshot.
#[derive(Debug, Clone)]
pub struct ScopeTicket {
    /// The scope's entry position.
    pub(crate) mark: UndoMark,
    /// Liveness token; the topology holds the other half.
    pub(crate) alive: Arc<()>,
}

/// Append-only guard state (PERF-T03): slot high-water marks at arming time.
#[derive(Debug, Clone)]
pub struct AppendGuard {
    /// Per-arena slot counts when the scope was armed, in [`ArenaTag`] order.
    pub(crate) slot_marks: [usize; 10],
    /// Whether a pre-existing write was attempted (forces full-path retry).
    pub(crate) tripped: bool,
}

/// Lineage of a clone: which log it was cloned from and how long that log was.
#[derive(Debug, Clone, Copy)]
pub struct UndoBase {
    /// Process-local identity of the topology's original undo lineage.
    pub(crate) lineage_id: u64,
    /// Generation of the source log at clone time.
    pub(crate) generation: u64,
    /// Source log length at clone time.
    pub(crate) log_len: usize,
    /// Mutation count of the copied state, used to detect changed intermediates.
    pub(crate) ticks: u64,
}

/// Mutation-local rollback storage plus append-only guards.
#[derive(Clone)]
pub struct UndoLog {
    /// Reversible records for all active scopes, oldest first.
    pub(crate) records: Vec<UndoRecord>,
    /// Active scope entries, outermost first. Entries whose snapshot was
    /// dropped without restore or commit are reclaimed lazily as commits.
    pub(crate) scopes: Vec<ActiveScope>,
    /// Bumped whenever a foreign-lineage restore swaps state underneath
    /// active scopes, invalidating their marks.
    pub(crate) generation: u64,
    /// Distinguishes unrelated topology values with coincident counters.
    pub(crate) lineage_id: u64,
    /// Next scope sequence number.
    pub(crate) next_seq: u64,
    /// Armed append-only guards, outermost first.
    pub(crate) append: Vec<AppendGuard>,
    /// Lineage of this topology value (set by `Clone`).
    pub(crate) base: UndoBase,
    /// Whether `base` came from another topology value and must propagate
    /// through further clones even after this clone is mutated.
    pub(crate) inherited_lineage: bool,
}

impl Default for UndoLog {
    fn default() -> Self {
        static NEXT_LINEAGE_ID: AtomicU64 = AtomicU64::new(1);
        let lineage_id = NEXT_LINEAGE_ID.fetch_add(1, Ordering::Relaxed);
        Self {
            records: Vec::new(),
            scopes: Vec::new(),
            generation: 0,
            lineage_id,
            next_seq: 0,
            append: Vec::new(),
            base: UndoBase {
                lineage_id,
                generation: 0,
                log_len: 0,
                ticks: 0,
            },
            inherited_lineage: false,
        }
    }
}

impl UndoLog {
    /// Fresh log for a clone: no active scopes or guards, but carrying the
    /// source's lineage so same-lineage foreign restores stay exact.
    pub(crate) fn fresh_for_clone(base: UndoBase) -> Self {
        Self {
            lineage_id: base.lineage_id,
            base,
            inherited_lineage: true,
            ..Self::default()
        }
    }

    /// See [`Topology::scopes_alive`]. Lives on the log so a caller holding
    /// another `Topology` field mutably (the pcurve registry) can still gate.
    fn scopes_alive(&mut self) -> bool {
        self.purge_dead_scopes();
        !self.scopes.is_empty()
    }

    /// See [`Topology::purge_dead_scopes`].
    fn purge_dead_scopes(&mut self) {
        let live = self
            .scopes
            .iter()
            .filter(|scope| Arc::strong_count(&scope.alive) > 1)
            .count();
        if live < self.scopes.len() {
            self.scopes
                .retain(|scope| Arc::strong_count(&scope.alive) > 1);
        }
        if self.scopes.is_empty() && !self.records.is_empty() {
            self.records.clear();
        }
    }

    /// See [`Topology::guard_append_use`].
    fn guard_append_use(&mut self, edge: EdgeId, face: FaceId) -> Result<(), TopologyError> {
        if let Some(guard) = self.append.last_mut() {
            let marks = &guard.slot_marks;
            if edge.index() < marks[ArenaTag::Edge as usize]
                || face.index() < marks[ArenaTag::Face as usize]
            {
                guard.tripped = true;
                return Err(TopologyError::AppendOnlyGuardTrip { entity: "pcurve" });
            }
        }
        Ok(())
    }
}

impl std::fmt::Debug for UndoLog {
    /// Coordination renders constant: scope sequence numbers, retained
    /// records, and lineage are transaction machinery, not model state, and
    /// must never disturb whole-topology snapshots (the former coordinator
    /// rendered its weak handle constantly for the same reason).
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("UndoLog { .. }")
    }
}

/// Generates the overwrite/retire record helpers for one arena.
macro_rules! undo_slot_api {
    ($record:ident, $retire:ident, $field:ident, $Id:ty, $tag:expr, $err:ident, $variant:ident) => {
        /// Records the live value of `id` before an in-place write.
        ///
        /// Returns the typed not-found error for a dead or out-of-range
        /// handle (recording nothing) and refuses pre-existing slots while
        /// an append-only guard is armed, before any state changes.
        pub(crate) fn $record(&mut self, id: $Id) -> Result<(), TopologyError> {
            if self.$field.get(id).is_none() {
                return Err(TopologyError::$err(id));
            }
            self.guard_append_slot($tag, id.index())?;
            if !self.scopes_alive() {
                return Ok(());
            }
            // Cloned only once a record is kept: untransacted writes (a bare
            // `add_face`, say) would otherwise deep-copy the entity to drop it.
            let Some(old) = self.$field.get(id).cloned() else {
                return Err(TopologyError::$err(id));
            };
            self.undo.records.push(UndoRecord::$variant {
                index: id.index(),
                old: Some(old),
            });
            Ok(())
        }

        /// Records `id`'s live value, then retires its slot without reuse.
        ///
        /// An already-dead handle is a silent no-op recording nothing.
        /// Refuses pre-existing slots while an append-only guard is armed,
        /// before any state changes.
        pub(crate) fn $retire(&mut self, id: $Id) -> Result<(), TopologyError> {
            if self.$field.get(id).is_none() {
                return Ok(());
            }
            self.guard_append_slot($tag, id.index())?;
            if self.scopes_alive()
                && let Some(old) = self.$field.get(id).cloned()
            {
                self.undo.records.push(UndoRecord::$variant {
                    index: id.index(),
                    old: Some(old),
                });
            }
            self.$field.retire(id);
            Ok(())
        }
    };
}

/// Generates only the overwrite record helper for an arena no API retires
/// yet (compounds, comp-solids). When a retirement path lands, it must grow
/// a retire helper and route through it before retiring — see the
/// write-guard rule in `docs/design/perf-t02-t03-transactions.md`.
macro_rules! undo_record_only {
    ($record:ident, $field:ident, $Id:ty, $tag:expr, $err:ident, $variant:ident) => {
        /// Records the live value of `id` before an in-place write.
        ///
        /// Returns the typed not-found error for a dead or out-of-range
        /// handle (recording nothing) and refuses pre-existing slots while
        /// an append-only guard is armed, before any state changes.
        pub(crate) fn $record(&mut self, id: $Id) -> Result<(), TopologyError> {
            if self.$field.get(id).is_none() {
                return Err(TopologyError::$err(id));
            }
            self.guard_append_slot($tag, id.index())?;
            if !self.scopes_alive() {
                return Ok(());
            }
            // Cloned only once a record is kept: untransacted writes (a bare
            // `add_face`, say) would otherwise deep-copy the entity to drop it.
            let Some(old) = self.$field.get(id).cloned() else {
                return Err(TopologyError::$err(id));
            };
            self.undo.records.push(UndoRecord::$variant {
                index: id.index(),
                old: Some(old),
            });
            Ok(())
        }
    };
}

impl Topology {
    undo_slot_api!(
        record_vertex_overwrite,
        record_vertex_retire,
        vertices,
        VertexId,
        ArenaTag::Vertex,
        VertexNotFound,
        Vertex
    );
    undo_slot_api!(
        record_edge_overwrite,
        record_edge_retire,
        edges,
        EdgeId,
        ArenaTag::Edge,
        EdgeNotFound,
        Edge
    );
    undo_slot_api!(
        record_wire_overwrite,
        record_wire_retire,
        wires,
        WireId,
        ArenaTag::Wire,
        WireNotFound,
        Wire
    );
    undo_slot_api!(
        record_face_overwrite,
        record_face_retire,
        faces,
        FaceId,
        ArenaTag::Face,
        FaceNotFound,
        Face
    );
    undo_slot_api!(
        record_shell_overwrite,
        record_shell_retire,
        shells,
        ShellId,
        ArenaTag::Shell,
        ShellNotFound,
        Shell
    );
    undo_slot_api!(
        record_solid_overwrite,
        record_solid_retire,
        solids,
        SolidId,
        ArenaTag::Solid,
        SolidNotFound,
        Solid
    );
    undo_record_only!(
        record_compound_overwrite,
        compounds,
        crate::compound::CompoundId,
        ArenaTag::Compound,
        CompoundNotFound,
        Compound
    );
    undo_record_only!(
        record_compsolid_overwrite,
        compsolids,
        crate::compsolid::CompSolidId,
        ArenaTag::CompSolid,
        CompSolidNotFound,
        CompSolid
    );
    undo_slot_api!(
        record_loop_overwrite,
        record_loop_retire,
        loops,
        LoopId,
        ArenaTag::Loop,
        LoopNotFound,
        Loop
    );
    undo_slot_api!(
        record_coedge_overwrite,
        record_coedge_retire,
        coedges,
        CoedgeId,
        ArenaTag::Coedge,
        CoedgeNotFound,
        Coedge
    );

    /// Records a fresh allocation. Allocations always append above the
    /// high-water mark, so they never trip the append-only guard.
    pub(crate) fn record_alloc(&mut self, tag: ArenaTag, index: usize) {
        if !self.scopes_alive() {
            return;
        }
        self.undo.records.push(UndoRecord::alloc(tag, index));
    }

    /// Current per-arena slot counts, in [`ArenaTag`] order.
    pub(crate) fn slot_mark_lens(&self) -> [usize; 10] {
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
    }

    /// Refuses a write to a pre-existing slot while an append-only guard is
    /// armed, marking the guard tripped so the scope retries on the full
    /// path. Call before recording or writing.
    pub(crate) fn guard_append_slot(
        &mut self,
        tag: ArenaTag,
        index: usize,
    ) -> Result<(), TopologyError> {
        if let Some(guard) = self.undo.append.last_mut()
            && index < guard.slot_marks[tag as usize]
        {
            guard.tripped = true;
            return Err(TopologyError::AppendOnlyGuardTrip { entity: tag.name() });
        }
        Ok(())
    }

    /// Refuses destruction of a pre-existing pcurve index entry while an
    /// append-only guard is armed. Pure additions (new coedge uses) are
    /// always allowed: they accompany new content at O(new) cost.
    pub(crate) fn guard_append_use(
        &mut self,
        edge: EdgeId,
        face: FaceId,
    ) -> Result<(), TopologyError> {
        self.undo.guard_append_use(edge, face)
    }

    /// Marks every armed append-only guard tripped (used when a foreign full
    /// restore swaps state underneath the scope, invalidating its marks).
    pub(crate) fn trip_append_if_armed(&mut self) {
        for guard in &mut self.undo.append {
            guard.tripped = true;
        }
    }

    /// Arms an append-only guard over the current high-water marks.
    pub(crate) fn arm_append(&mut self) {
        self.undo.append.push(AppendGuard {
            slot_marks: self.slot_mark_lens(),
            tripped: false,
        });
    }

    /// Disarms the innermost append-only guard, reporting whether it tripped.
    pub(crate) fn disarm_append(&mut self) -> bool {
        self.undo.append.pop().is_some_and(|guard| guard.tripped)
    }

    /// Records one pcurve index write: the previous value for overwrites and
    /// removals, `None` for pure additions. Overwrites and removals of
    /// pre-existing uses trip an armed append-only guard first.
    pub(crate) fn record_pcurve_write(
        &mut self,
        edge: EdgeId,
        face: FaceId,
        forward: bool,
    ) -> Result<(), TopologyError> {
        let old = self.pcurves.get_use(edge, face, forward);
        if old.is_some() {
            self.guard_append_use(edge, face)?;
        }
        if !self.scopes_alive() {
            return Ok(());
        }
        self.undo.records.push(UndoRecord::PcurveUse {
            key: PCurveKey::new(edge, face, forward),
            old,
        });
        Ok(())
    }

    /// Indexes one authoritative coedge use, recording the previous value.
    ///
    /// Equivalent to [`Self::record_pcurve_write`] followed by the registry
    /// insert, with one key lookup instead of two: the guard and the record
    /// still run on the previous value before the write lands.
    pub(crate) fn index_pcurve_use(
        &mut self,
        edge: EdgeId,
        face: FaceId,
        forward: bool,
        coedge: CoedgeId,
    ) -> Result<(), TopologyError> {
        let key = PCurveKey::new(edge, face, forward);
        let undo = &mut self.undo;
        self.pcurves.upsert_use(key, coedge, |old| {
            // Only an occupied key may fail: `upsert_use` has already grown
            // the table for a vacant one.
            if old.is_some() {
                undo.guard_append_use(edge, face)?;
            }
            if undo.scopes_alive() {
                undo.records.push(UndoRecord::PcurveUse { key, old });
            }
            Ok(())
        })
    }

    /// Records a solid-attribute write. Any write to a pre-existing solid
    /// trips an armed append-only guard: attributes are observable entity
    /// state with no derivation to absorb them.
    pub(crate) fn record_solid_attribute(&mut self, id: SolidId) -> Result<(), TopologyError> {
        self.guard_append_slot(ArenaTag::Solid, id.index())?;
        if !self.scopes_alive() {
            return Ok(());
        }
        self.undo.records.push(UndoRecord::SolidAttribute {
            id,
            old: self.attributes.solid(id).cloned(),
        });
        Ok(())
    }

    /// Records a face-attribute write; see
    /// [`Self::record_solid_attribute`].
    pub(crate) fn record_face_attribute(&mut self, id: FaceId) -> Result<(), TopologyError> {
        self.guard_append_slot(ArenaTag::Face, id.index())?;
        if !self.scopes_alive() {
            return Ok(());
        }
        self.undo.records.push(UndoRecord::FaceAttribute {
            id,
            old: self.attributes.face(id).cloned(),
        });
        Ok(())
    }

    /// Records the ordinal pairs one journal record call created.
    pub(crate) fn record_journal_created(&mut self, pairs: Vec<(EntityKey, JournalOrdinal)>) {
        if pairs.is_empty() || !self.scopes_alive() {
            return;
        }
        self.undo.records.push(UndoRecord::JournalCreated { pairs });
    }

    /// Records a wholesale journal replacement (`load_journal`) while scopes
    /// are active, so a rewind can restore the previous history.
    pub(crate) fn record_journal_replace(&mut self) {
        if !self.scopes_alive() {
            return;
        }
        self.undo.records.push(UndoRecord::JournalReplace {
            old: self.journal.clone(),
        });
    }

    /// Current undo-log generation for scope-staleness checks.
    pub(crate) fn undo_generation(&self) -> u64 {
        self.undo.generation
    }

    /// Number of retained undo records (diagnostic for tests and benchmarks).
    #[cfg(test)]
    pub(crate) fn undo_record_count(&self) -> usize {
        self.undo.records.len()
    }

    /// Opens a scope, returning its ticket. Capture is O(1).
    pub(crate) fn undo_begin_scope(&mut self) -> ScopeTicket {
        self.purge_dead_scopes();
        let mark = UndoMark {
            seq: self.undo.next_seq,
            log_len: self.undo.records.len(),
            journal_len: self.journal.len(),
            ticks: self.mutation_ticks,
        };
        self.undo.next_seq = self.undo.next_seq.wrapping_add(1);
        let alive = Arc::new(());
        self.undo.scopes.push(ActiveScope {
            mark,
            alive: Arc::clone(&alive),
        });
        ScopeTicket { mark, alive }
    }

    /// Whether any live scope is active, reclaiming dropped snapshots first.
    ///
    /// Every record gate funnels through here, so a dropped snapshot's
    /// records merge into the enclosing scope (commit) and the last close
    /// releases the log: retained storage is bounded by live scopes only.
    pub(crate) fn scopes_alive(&mut self) -> bool {
        self.undo.scopes_alive()
    }

    /// Reclaims scope entries whose snapshot was dropped, keeping their
    /// records for the enclosing scopes. Empties the log when no live
    /// scope remains.
    fn purge_dead_scopes(&mut self) {
        self.undo.purge_dead_scopes();
    }

    /// Commits a scope: its records merge into the enclosing scope. Dropping
    /// the last scope clears the log, bounding retained storage to active
    /// scopes only.
    pub(crate) fn undo_commit_scope(&mut self, mark: UndoMark) {
        self.purge_dead_scopes();
        self.undo.scopes.retain(|scope| scope.mark.seq != mark.seq);
        if self.undo.scopes.is_empty() {
            self.undo.records.clear();
        }
    }

    /// Rewinds to a scope mark, undoing every record above it in reverse,
    /// truncating the journal tail, and restoring the tick count. Scopes at
    /// or above the mark are closed; surviving outer scopes keep their
    /// records. Rewinding past available records saturates (a hand-rolled
    /// double restore leaves the foreign state in place; the transaction
    /// API restores each snapshot at most once, so this is unreachable
    /// through it).
    pub(crate) fn undo_rewind_scope(&mut self, mark: UndoMark) {
        // Any rewind invalidates persistent preparation, even when the
        // restored tick equals an earlier generation (ABA). The cache
        // generation only moves forward; the journal tick still rolls back.
        self.bump_cache_generation();
        self.purge_dead_scopes();
        if self.undo.records.len() >= mark.log_len {
            let mut created = Vec::new();
            while self.undo.records.len() > mark.log_len {
                let Some(record) = self.undo.records.pop() else {
                    break;
                };
                match record {
                    UndoRecord::JournalCreated { mut pairs } => created.append(&mut pairs),
                    UndoRecord::JournalReplace { old } => {
                        self.journal.restore_preserving_ids(&old);
                    }
                    other => self.apply_undo_record(other),
                }
            }
            self.journal.rollback_scope(mark.journal_len, &created);
            self.mutation_ticks = mark.ticks;
        }
        self.undo.scopes.retain(|scope| scope.mark.seq < mark.seq);
        if self.undo.scopes.is_empty() {
            self.undo.records.clear();
        }
    }

    /// Prepares for a full restore. Unchanged same-lineage snapshots can
    /// truncate to their source prefix. A changed clone or foreign snapshot
    /// needs one full inverse while scopes are live; otherwise its state
    /// cannot be reconstructed from the destination's mutation log.
    /// An armed append-only guard always trips: its slot marks no longer
    /// describe the swapped state.
    pub(crate) fn undo_truncate_for_foreign_restore(&mut self, snapshot: &Self) {
        // Foreign restores swap state underneath live scopes. The caller
        // (`restore_*`) already bumped, but invalidate here as well so a
        // direct caller can never miss it; extra bumps are harmless.
        self.bump_cache_generation();
        self.trip_append_if_armed();
        self.purge_dead_scopes();
        let base = snapshot.undo.base;
        if base.lineage_id == self.undo.lineage_id
            && base.generation == self.undo.generation
            && base.log_len <= self.undo.records.len()
            && base.ticks == snapshot.mutation_ticks
        {
            self.undo.records.truncate(base.log_len);
            self.undo
                .scopes
                .retain(|scope| scope.mark.log_len <= base.log_len);
            if self.undo.scopes.is_empty() {
                self.undo.records.clear();
            }
        } else if self.scopes_alive() {
            self.undo.records.push(UndoRecord::ForeignRestore {
                old: Box::new(self.clone()),
            });
        } else {
            self.undo.records.clear();
            self.undo.scopes.clear();
            self.undo.generation = self.undo.generation.wrapping_add(1);
            // The swapped state has no source prefix in this log. A later
            // clone must not claim an inherited prefix of the old topology.
            let lineage_id = UndoLog::default().lineage_id;
            self.undo.lineage_id = lineage_id;
            self.undo.base = UndoBase {
                lineage_id,
                generation: self.undo.generation,
                log_len: 0,
                ticks: snapshot.mutation_ticks,
            };
            self.undo.inherited_lineage = false;
        }
    }

    /// Applies one record's inverse. Slot restores write the old value back
    /// and revive liveness; allocation records retire the slot, keeping its
    /// post-scope value as an inaccessible tombstone so the handle stays
    /// stale forever without reusing its index.
    fn apply_undo_record(&mut self, record: UndoRecord) {
        match record {
            UndoRecord::Vertex { index, old } => self.vertices.rewind_slot(index, old),
            UndoRecord::Edge { index, old } => self.edges.rewind_slot(index, old),
            UndoRecord::Wire { index, old } => self.wires.rewind_slot(index, old),
            UndoRecord::Face { index, old } => self.faces.rewind_slot(index, old),
            UndoRecord::Shell { index, old } => self.shells.rewind_slot(index, old),
            UndoRecord::Solid { index, old } => self.solids.rewind_slot(index, old),
            UndoRecord::Compound { index, old } => self.compounds.rewind_slot(index, old),
            UndoRecord::CompSolid { index, old } => self.compsolids.rewind_slot(index, old),
            UndoRecord::Loop { index, old } => self.loops.rewind_slot(index, old),
            UndoRecord::Coedge { index, old } => self.coedges.rewind_slot(index, old),
            UndoRecord::PcurveUse { key, old } => match old {
                Some(coedge) => self
                    .pcurves
                    .index_use(key.edge, key.face, key.forward, coedge),
                None => self.pcurves.remove_key(key),
            },
            UndoRecord::SolidAttribute { id, old } => match old {
                Some(attributes) => self.attributes.set_solid(id, attributes),
                None => {
                    self.attributes.remove_solid(id);
                }
            },
            UndoRecord::FaceAttribute { id, old } => match old {
                Some(attributes) => self.attributes.set_face(id, attributes),
                None => {
                    self.attributes.remove_face(id);
                }
            },
            UndoRecord::JournalCreated { .. } | UndoRecord::JournalReplace { .. } => {
                // Handled by the rewind driver, never applied here.
            }
            UndoRecord::ForeignRestore { old } => self.restore_rollback_fields(&old),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use remus_math::vec::{Point3, Vec3};

    use crate::edge::EdgeCurve;
    use crate::face::FaceSurface;
    use crate::transaction::{AppendPath, run_append_only, run_transacted};
    use crate::wire::OrientedEdge;

    use super::*;

    type UseKey = (EdgeId, FaceId, bool);

    /// The two-lookup sequence `index_pcurve_use` replaced, kept as its oracle.
    fn legacy_index_pcurve_use(
        topo: &mut Topology,
        (edge, face, forward): UseKey,
        coedge: CoedgeId,
    ) -> Result<(), TopologyError> {
        topo.record_pcurve_write(edge, face, forward)?;
        topo.pcurves.index_use(edge, face, forward, coedge);
        Ok(())
    }

    fn triangle(topo: &mut Topology) -> (FaceId, [EdgeId; 3]) {
        let points = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let v = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
        let e = [(0, 1), (1, 2), (2, 0)]
            .map(|(a, b)| topo.add_edge(Edge::new(v[a], v[b], EdgeCurve::Line)));
        let wire = Wire::new(e.map(|id| OrientedEdge::new(id, true)).to_vec(), true).unwrap();
        let wire = topo.add_wire(wire);
        (topo.add_face(Face::new(wire, vec![], plane())), e)
    }

    fn plane() -> FaceSurface {
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        }
    }

    /// Full state, including the retained records `Topology`'s `Debug` hides.
    fn snapshot(topo: &Topology) -> String {
        format!("{topo:?}\n{:?}", topo.undo.records)
    }

    /// Runs the single-lookup index and its two-step oracle on clones of
    /// `topo` (optionally inside a scope and under an armed guard), asserts
    /// identical outcomes, and returns the single-lookup result.
    fn assert_matches_legacy(
        topo: &Topology,
        key: UseKey,
        coedge: CoedgeId,
        scope: bool,
        guard: bool,
    ) -> (Topology, Result<(), TopologyError>) {
        let run = |legacy: bool| {
            let mut topo = topo.clone();
            let ticket = scope.then(|| topo.undo_begin_scope());
            if guard {
                topo.arm_append();
            }
            let result = if legacy {
                legacy_index_pcurve_use(&mut topo, key, coedge)
            } else {
                let (edge, face, forward) = key;
                topo.index_pcurve_use(edge, face, forward, coedge)
            };
            let tripped = guard && topo.disarm_append();
            let outcome = (format!("{result:?}"), tripped, snapshot(&topo));
            drop(ticket);
            (topo, result, outcome)
        };
        let (after, result, outcome) = run(false);
        let (_, _, legacy_outcome) = run(true);
        assert_eq!(outcome, legacy_outcome);
        (after, result)
    }

    #[test]
    fn single_lookup_pcurve_index_matches_two_step_oracle() {
        let mut topo = Topology::new();
        let (face, [e0, e1, _]) = triangle(&mut topo);
        let c0 = topo.coedges_of_edge(e0)[0];
        let c1 = topo.coedges_of_edge(e1)[0];
        let before = snapshot(&topo);

        // Fresh key, without and inside a scope.
        for scope in [false, true] {
            let (after, result) = assert_matches_legacy(&topo, (e0, face, false), c0, scope, false);
            result.unwrap();
            assert_eq!(after.undo_record_count(), usize::from(scope));
        }

        // Occupied key inside a scope: records the previous coedge, and a
        // rewind restores the registry exactly.
        let (_, result) = assert_matches_legacy(&topo, (e0, face, true), c1, true, false);
        result.unwrap();
        let mut scoped = topo.clone();
        let ticket = scoped.undo_begin_scope();
        scoped.index_pcurve_use(e0, face, true, c1).unwrap();
        assert!(matches!(
            scoped.undo.records.as_slice(),
            [UndoRecord::PcurveUse { old: Some(old), .. }] if *old == c0
        ));
        assert_eq!(scoped.pcurves.get_use(e0, face, true), Some(c1));
        scoped.undo_rewind_scope(ticket.mark);
        assert_eq!(
            format!("{:?}", scoped.pcurves),
            format!("{:?}", topo.pcurves)
        );

        // Occupied key over a pre-existing edge under an armed guard: the
        // typed trip lands before any registry, coedge, or record change.
        for scope in [false, true] {
            let (after, result) = assert_matches_legacy(&topo, (e0, face, true), c1, scope, true);
            assert!(matches!(
                result,
                Err(TopologyError::AppendOnlyGuardTrip { entity: "pcurve" })
            ));
            assert_eq!(after.undo_record_count(), 0);
            assert_eq!(format!("{after:?}"), format!("{topo:?}"));
        }
        assert_eq!(snapshot(&topo), before);
    }

    #[test]
    fn duplicated_boundary_use_still_trips_the_append_guard() {
        let mut base = Topology::new();
        let (_, [e0, ..]) = triangle(&mut base);
        // A degenerate boundary using one pre-existing edge twice, forward.
        let add = |topo: &mut Topology| {
            let uses = vec![OrientedEdge::new(e0, true), OrientedEdge::new(e0, true)];
            let wire = topo.add_wire(Wire::new(uses, true).unwrap());
            Ok::<_, TopologyError>(topo.add_face(Face::new(wire, vec![], plane())))
        };
        let mut transacted = base.clone();
        let face = run_transacted(&mut transacted, add).unwrap();
        let mut append = base.clone();
        let (fallback_face, path) = run_append_only(&mut append, add).unwrap();
        assert_eq!(path, AppendPath::FullFallback);

        let uses = |topo: &Topology, face| {
            topo.loops_of_face(face)
                .unwrap()
                .iter()
                .flat_map(|&id| topo.face_loop(id).unwrap().coedges().to_vec())
                .map(|id| {
                    let coedge = topo.coedge(id).unwrap();
                    (coedge.edge(), coedge.is_forward())
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(uses(&append, fallback_face), uses(&transacted, face));
        assert_eq!(append.num_pcurves(), transacted.num_pcurves());
        assert_eq!(append.num_coedges(), transacted.num_coedges());
        assert_eq!(append.num_faces(), transacted.num_faces());
    }

    #[test]
    fn record_helpers_clone_only_into_kept_records() {
        let mut topo = Topology::new();
        let (face, _) = triangle(&mut topo);
        let mut stale = None;
        let _ = run_transacted(&mut topo, |topo| {
            stale = Some(topo.add_vertex(Vertex::new(Point3::new(5.0, 0.0, 0.0), 1e-7)));
            Err::<(), _>(TopologyError::WireNotClosed)
        });
        let stale = stale.unwrap();
        let live = topo.vertices().iter().next().unwrap().0;

        for scope in [false, true] {
            let mut topo = topo.clone();
            let ticket = scope.then(|| topo.undo_begin_scope());
            assert!(matches!(
                topo.record_vertex_overwrite(stale),
                Err(TopologyError::VertexNotFound(id)) if id == stale
            ));
            topo.record_vertex_retire(stale).unwrap();
            assert_eq!(topo.undo_record_count(), 0, "dead handles record nothing");

            topo.record_vertex_overwrite(live).unwrap();
            topo.record_face_overwrite(face).unwrap();
            if scope {
                let vertex = format!("{:?}", topo.vertices.get(live));
                let face = format!("{:?}", topo.faces.get(face));
                assert!(matches!(
                    topo.undo.records.as_slice(),
                    [
                        UndoRecord::Vertex { old: Some(old_vertex), .. },
                        UndoRecord::Face { old: Some(old_face), .. },
                    ] if format!("{:?}", Some(old_vertex)) == vertex
                        && format!("{:?}", Some(old_face)) == face
                ));
            } else {
                assert_eq!(topo.undo_record_count(), 0, "no scope keeps no record");
            }
            drop(ticket);
        }
    }
}
