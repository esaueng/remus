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
//! journal record. Nothing here scales with unrelated document size. Records
//! live only while scopes are active: the last commit clears the log, and a
//! rewind drops everything above its mark.
//!
//! # Soundness argument
//!
//! The log and the live state satisfy one invariant: popping every record
//! above a surviving mark's length and applying the inverses in reverse
//! yields exactly the mark's entry state. The invariant holds initially
//! (empty log) and is preserved because every mutation route in
//! [`Topology`](crate::Topology) records before writing (see the write-guard
//! rule in `docs/design/perf-t02-t03-transactions.md`), rewinds apply exact
//! inverses, and foreign full restores truncate the log to the snapshot's
//! clone-time length before swapping (a snapshot cloned from live state at
//! log length `k` is exactly base plus records `[0..k)`).

use std::sync::Arc;

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
    /// Generation of the source log at clone time.
    pub(crate) generation: u64,
    /// Source log length at clone time.
    pub(crate) log_len: usize,
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
    /// Next scope sequence number.
    pub(crate) next_seq: u64,
    /// Armed append-only guards, outermost first.
    pub(crate) append: Vec<AppendGuard>,
    /// Lineage of this topology value (set by `Clone`).
    pub(crate) base: UndoBase,
}

impl Default for UndoLog {
    fn default() -> Self {
        Self {
            records: Vec::new(),
            scopes: Vec::new(),
            generation: 0,
            next_seq: 0,
            append: Vec::new(),
            base: UndoBase {
                generation: 0,
                log_len: 0,
            },
        }
    }
}

impl UndoLog {
    /// Fresh log for a clone: no active scopes or guards, but carrying the
    /// source's lineage so same-lineage foreign restores stay exact.
    pub(crate) fn fresh_for_clone(source_generation: u64, source_log_len: usize) -> Self {
        Self {
            base: UndoBase {
                generation: source_generation,
                log_len: source_log_len,
            },
            ..Self::default()
        }
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
            let Some(old) = self.$field.get(id).cloned() else {
                return Err(TopologyError::$err(id));
            };
            self.guard_append_slot($tag, id.index())?;
            if !self.scopes_alive() {
                return Ok(());
            }
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
            let Some(old) = self.$field.get(id).cloned() else {
                return Ok(());
            };
            self.guard_append_slot($tag, id.index())?;
            if self.scopes_alive() {
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
            let Some(old) = self.$field.get(id).cloned() else {
                return Err(TopologyError::$err(id));
            };
            self.guard_append_slot($tag, id.index())?;
            if !self.scopes_alive() {
                return Ok(());
            }
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
        if let Some(guard) = self.undo.append.last_mut() {
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
    pub(crate) fn index_pcurve_use(
        &mut self,
        edge: EdgeId,
        face: FaceId,
        forward: bool,
        coedge: CoedgeId,
    ) -> Result<(), TopologyError> {
        self.record_pcurve_write(edge, face, forward)?;
        self.pcurves.index_use(edge, face, forward, coedge);
        Ok(())
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
        self.purge_dead_scopes();
        !self.undo.scopes.is_empty()
    }

    /// Reclaims scope entries whose snapshot was dropped, keeping their
    /// records for the enclosing scopes. Empties the log when no live
    /// scope remains.
    fn purge_dead_scopes(&mut self) {
        let live = self
            .undo
            .scopes
            .iter()
            .filter(|scope| Arc::strong_count(&scope.alive) > 1)
            .count();
        if live < self.undo.scopes.len() {
            self.undo
                .scopes
                .retain(|scope| Arc::strong_count(&scope.alive) > 1);
        }
        if self.undo.scopes.is_empty() && !self.undo.records.is_empty() {
            self.undo.records.clear();
        }
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

    /// Prepares for a foreign full restore from `snapshot`: same-lineage
    /// snapshots truncate the log to the snapshot's clone-time length (the
    /// snapshot is exactly base plus those records, so surviving marks stay
    /// valid); anything else clears coordination and invalidates live marks.
    /// An armed append-only guard always trips: its slot marks no longer
    /// describe the swapped state.
    pub(crate) fn undo_truncate_for_foreign_restore(&mut self, snapshot: &Self) {
        self.trip_append_if_armed();
        self.purge_dead_scopes();
        let base = snapshot.undo.base;
        if base.generation == self.undo.generation && base.log_len <= self.undo.records.len() {
            self.undo.records.truncate(base.log_len);
            self.undo
                .scopes
                .retain(|scope| scope.mark.log_len <= base.log_len);
            if self.undo.scopes.is_empty() {
                self.undo.records.clear();
            }
        } else {
            self.undo.records.clear();
            self.undo.scopes.clear();
            self.undo.generation = self.undo.generation.wrapping_add(1);
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
        }
    }
}
