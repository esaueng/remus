//! Mutation-local rollback storage (PERF-T02) and the append-only guard (PERF-T03).
//!
//! This module is a child of [`Topology`](crate::Topology), so it may touch
//! the private arenas directly. All cross-module access goes through the
//! `pub(crate)` methods here; [`transaction`](crate::transaction) drives
//! scopes through those methods and never touches arena fields.
//!
//! # Cost model
//!
//! A scope mark is O(1): three lengths, a tick count and the ten per-arena
//! slot counts. Every record is proportional to state the scope actually
//! wrote: one cloned entity per overwritten slot that predates the innermost
//! live scope, one old key value per registry/attribute write, and the
//! created ordinal pairs per journal record. Allocations, and writes to
//! slots the innermost live scope allocated, record nothing: a rewind
//! retires every slot at or above its mark's slot counts. A changed-clone or
//! unrelated full restore inside a live scope takes one document-sized
//! inverse; ordinary mutation-local scopes do not. Records live only while
//! scopes are active: the last commit clears the log, and a rewind drops
//! everything above its mark.
//!
//! # Soundness argument
//!
//! The log and the live state satisfy one invariant: popping every record
//! above a surviving mark's length, applying the inverses in reverse, and
//! then retiring every slot at or above the mark's slot counts yields
//! exactly the mark's entry state, up to the contents of inaccessible
//! tombstones. The invariant holds initially (empty log) and is preserved
//! because every mutation route in [`Topology`](crate::Topology) records
//! before writing a slot that was live at some surviving mark (see the
//! write-guard rule and its §8 amendment in
//! `docs/design/perf-t02-t03-transactions.md`), slots are never reused and
//! slot counts never shrink, rewinds apply exact inverses, and an unchanged
//! same-lineage restore that lands exactly on its snapshot truncates to its
//! source prefix only while that prefix still rewinds the snapshot to every
//! live scope's entry state (an unchanged clone of this very value, taken
//! after every live scope opened, cut at its own copy point with that prefix
//! intact; see `Topology::undo_truncate_for_foreign_restore`). Any other restore inside a
//! live scope records the complete pre-restore state so older records remain
//! valid after that inverse lands.

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
use crate::pcurve::{PCurveKey, PCurveRegistry};
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
/// Slot records carry the previous live value of a slot that existed when
/// the innermost live scope opened. Allocations are not recorded: a rewind
/// retires every slot at or above its mark's per-arena slot counts (see
/// [`UndoMark::slot_marks`]), keeping each as a tombstone so the handle
/// stays stale forever. Key records carry the previous index/attribute
/// value (`None` means the key was absent).
#[derive(Debug, Clone)]
pub enum UndoRecord {
    Vertex {
        index: usize,
        old: Vertex,
    },
    Edge {
        index: usize,
        old: Edge,
    },
    Wire {
        index: usize,
        old: Wire,
    },
    Face {
        index: usize,
        old: Face,
    },
    Shell {
        index: usize,
        old: Shell,
    },
    Solid {
        index: usize,
        old: Solid,
    },
    Compound {
        index: usize,
        old: Compound,
    },
    CompSolid {
        index: usize,
        old: CompSolid,
    },
    Loop {
        index: usize,
        old: Loop,
    },
    Coedge {
        index: usize,
        old: Coedge,
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

/// A scope's entry position: log length, journal length, tick count, and
/// per-arena slot counts.
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
    /// Per-arena slot counts at capture, in [`ArenaTag`] order.
    ///
    /// Arena slots are never reused and slot counts never shrink, so every
    /// slot at or above these counts came into existence after the scope
    /// opened — by allocation, or by a foreign restore that grew the arena.
    /// None of them was live at the mark, which is why allocations need no
    /// record: a rewind retires the whole range instead.
    pub(crate) slot_marks: [usize; 10],
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

impl ActiveScope {
    /// Whether the owning snapshot still exists.
    fn is_alive(&self) -> bool {
        Arc::strong_count(&self.alive) > 1
    }
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
    /// Next scope sequence number at clone time: every scope with a lower
    /// `seq` was opened before the clone, so its log prefix covers it.
    pub(crate) seq: u64,
}

/// The exact point in its direct source's history at which a clone was
/// copied: the soundness half of the truncation certificate (see
/// `Topology::undo_truncate_for_foreign_restore`).
#[derive(Debug, Clone, Copy)]
pub struct UndoOrigin {
    /// Cache lineage of the source value: unique per value, kept by restores.
    pub(crate) cache_lineage: u64,
    /// Source cache generation at clone time. Every mutation route bumps it,
    /// tick-free writes included, so an equal value proves the clone (which
    /// carries it) unchanged.
    pub(crate) cache_generation: u64,
    /// Source journal fingerprint at clone time; journal appends do not bump
    /// the cache generation.
    pub(crate) journal: (usize, u64, u64),
    /// Source log length at clone time.
    pub(crate) log_len: usize,
    /// Serial of the source log's last record at clone time (0: empty log).
    pub(crate) last_serial: u64,
}

/// Mutation-local rollback storage plus append-only guards.
#[derive(Clone)]
pub struct UndoLog {
    /// Reversible records for all active scopes, oldest first.
    pub(crate) records: Vec<UndoRecord>,
    /// One serial per record, parallel to `records`: never reissued, so a
    /// serial still at its index proves every record up to it unchanged.
    serials: Vec<u64>,
    /// Serial of the most recently pushed record (0: none yet).
    last_serial: u64,
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
    /// Where in its direct source's history this value was cloned (`None`
    /// for a value never cloned).
    pub(crate) origin: Option<UndoOrigin>,
}

impl Default for UndoLog {
    fn default() -> Self {
        static NEXT_LINEAGE_ID: AtomicU64 = AtomicU64::new(1);
        let lineage_id = NEXT_LINEAGE_ID.fetch_add(1, Ordering::Relaxed);
        Self {
            records: Vec::new(),
            serials: Vec::new(),
            last_serial: 0,
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
                seq: 0,
            },
            inherited_lineage: false,
            origin: None,
        }
    }
}

impl UndoLog {
    /// Fresh log for a clone: no active scopes or guards, but carrying the
    /// source's lineage so same-lineage foreign restores stay exact, and the
    /// source's exact position for the soundness half of that certificate.
    pub(crate) fn fresh_for_clone(base: UndoBase, origin: UndoOrigin) -> Self {
        Self {
            lineage_id: base.lineage_id,
            // Scopes the clone opens order after the clone point.
            next_seq: base.seq,
            base,
            inherited_lineage: true,
            origin: Some(origin),
            ..Self::default()
        }
    }

    /// Appends one record, issuing its serial.
    fn push(&mut self, record: UndoRecord) {
        self.last_serial = self.last_serial.wrapping_add(1);
        self.records.push(record);
        self.serials.push(self.last_serial);
    }

    /// Removes the newest record.
    fn pop(&mut self) -> Option<UndoRecord> {
        self.serials.pop();
        self.records.pop()
    }

    /// Keeps the oldest `len` records.
    fn truncate(&mut self, len: usize) {
        self.records.truncate(len);
        self.serials.truncate(len);
    }

    /// Drops every record.
    fn clear(&mut self) {
        self.records.clear();
        self.serials.clear();
    }

    /// Serial of the newest record (0 for an empty log).
    pub(crate) fn newest_serial(&self) -> u64 {
        self.serials.last().copied().unwrap_or(0)
    }

    /// Whether the oldest `len` records are the ones present when the
    /// newest of them carried `serial`: records are only ever appended or
    /// cut from the end, and serials are never reissued.
    fn holds_prefix(&self, len: usize, serial: u64) -> bool {
        len == 0 || self.serials.get(len - 1) == Some(&serial)
    }

    /// See [`Topology::scopes_alive`]. Lives on the log so a caller holding
    /// another `Topology` field mutably (the pcurve registry) can still gate.
    ///
    /// Every record gate calls this, so the common case — the innermost
    /// entry is live — answers without scanning the stack. Dead entries
    /// below a live innermost scope are left for the next full purge: a
    /// dead entry only marks where its records merge into the enclosing
    /// scope, and every operation that reads marks (begin, commit, rewind,
    /// foreign restore) purges first. When the innermost entry is live, it
    /// is also the innermost *live* scope that [`Self::keeps_slot_record`]
    /// consults.
    #[inline]
    fn scopes_alive(&mut self) -> bool {
        match self.scopes.last() {
            Some(scope) if scope.is_alive() => true,
            // Untransacted mutation: nothing to purge or release.
            None if self.records.is_empty() => false,
            _ => {
                self.purge_dead_scopes();
                !self.scopes.is_empty()
            }
        }
    }

    /// See [`Topology::purge_dead_scopes`].
    fn purge_dead_scopes(&mut self) {
        if !self.scopes.iter().all(ActiveScope::is_alive) {
            self.scopes.retain(ActiveScope::is_alive);
        }
        if self.scopes.is_empty() && !self.records.is_empty() {
            self.clear();
        }
    }

    /// Whether an overwrite or retirement of slot `index` in arena `tag`
    /// must be recorded before it lands.
    ///
    /// Nothing is recorded without a live scope. Inside one, a slot at or
    /// above the innermost live scope's slot mark did not exist when that
    /// scope opened, nor when any enclosing scope opened (outer marks are
    /// never higher). Rewinding to any live mark therefore retires the slot
    /// without consulting a record, and restoring its pre-write value would
    /// only rewrite an inaccessible tombstone: the record is skipped and the
    /// tombstone keeps the slot's last value — exactly what
    /// [`Arena::restore_for_rollback`](crate::arena::Arena) leaves behind for
    /// post-snapshot slots. A retirement record would not even change the
    /// tombstone (retiring never writes the value).
    ///
    /// Index, attribute and journal records are never skipped: their
    /// inverses remove entries that would otherwise outlive the retired slot.
    pub(crate) fn keeps_slot_record(&mut self, tag: ArenaTag, index: usize) -> bool {
        self.scopes_alive()
            && self
                .scopes
                .last()
                .is_some_and(|scope| index < scope.mark.slot_marks[tag as usize])
    }

    /// Field-level [`Topology::index_pcurve_use`], for callers that hold
    /// other `Topology` fields borrowed: indexes one authoritative coedge
    /// use with a single key lookup, guarding and recording the previous
    /// value before the write lands.
    pub(crate) fn index_pcurve_use(
        &mut self,
        pcurves: &mut PCurveRegistry,
        key: PCurveKey,
        coedge: CoedgeId,
    ) -> Result<(), TopologyError> {
        pcurves.upsert_use(key, coedge, |old| {
            // Only an occupied key may fail: `upsert_use` has already grown
            // the table for a vacant one.
            if old.is_some() {
                self.guard_append_use(key.edge, key.face)?;
            }
            if self.scopes_alive() {
                self.push(UndoRecord::PcurveUse { key, old });
            }
            Ok(())
        })
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
            if !self.undo.keeps_slot_record($tag, id.index()) {
                return Ok(());
            }
            // Cloned only once a record is kept: untransacted writes, and
            // writes to slots the innermost scope allocated, would otherwise
            // deep-copy the entity to drop it.
            let Some(old) = self.$field.get(id).cloned() else {
                return Err(TopologyError::$err(id));
            };
            self.undo.push(UndoRecord::$variant {
                index: id.index(),
                old,
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
            if self.undo.keeps_slot_record($tag, id.index())
                && let Some(old) = self.$field.get(id).cloned()
            {
                self.undo.push(UndoRecord::$variant {
                    index: id.index(),
                    old,
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
            if !self.undo.keeps_slot_record($tag, id.index()) {
                return Ok(());
            }
            // Cloned only once a record is kept: untransacted writes, and
            // writes to slots the innermost scope allocated, would otherwise
            // deep-copy the entity to drop it.
            let Some(old) = self.$field.get(id).cloned() else {
                return Err(TopologyError::$err(id));
            };
            self.undo.push(UndoRecord::$variant {
                index: id.index(),
                old,
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
        self.undo.push(UndoRecord::PcurveUse {
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
        self.undo.index_pcurve_use(
            &mut self.pcurves,
            PCurveKey::new(edge, face, forward),
            coedge,
        )
    }

    /// Records a solid-attribute write. Any write to a pre-existing solid
    /// trips an armed append-only guard: attributes are observable entity
    /// state with no derivation to absorb them.
    pub(crate) fn record_solid_attribute(&mut self, id: SolidId) -> Result<(), TopologyError> {
        self.guard_append_slot(ArenaTag::Solid, id.index())?;
        if !self.scopes_alive() {
            return Ok(());
        }
        self.undo.push(UndoRecord::SolidAttribute {
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
        self.undo.push(UndoRecord::FaceAttribute {
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
        self.undo.push(UndoRecord::JournalCreated { pairs });
    }

    /// Records a wholesale journal replacement (`load_journal`) while scopes
    /// are active, so a rewind can restore the previous history.
    pub(crate) fn record_journal_replace(&mut self) {
        if !self.scopes_alive() {
            return;
        }
        self.undo.push(UndoRecord::JournalReplace {
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
            slot_marks: self.slot_mark_lens(),
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
            self.undo.clear();
        }
    }

    /// Rewinds to a scope mark, undoing every record above it in reverse,
    /// retiring every slot at or above the mark's slot counts (the scope's
    /// allocations), truncating the journal tail, and restoring the tick
    /// count. Scopes at or above the mark are closed; surviving outer scopes
    /// keep their records. Rewinding past available records saturates (a
    /// hand-rolled double restore leaves the foreign state in place; the
    /// transaction API restores each snapshot at most once, so this is
    /// unreachable through it).
    pub(crate) fn undo_rewind_scope(&mut self, mark: UndoMark) {
        // Any rewind invalidates persistent preparation, even when the
        // restored tick equals an earlier generation (ABA). The cache
        // generation only moves forward; the journal tick still rolls back.
        self.bump_cache_generation();
        self.purge_dead_scopes();
        if self.undo.records.len() >= mark.log_len {
            let mut created = Vec::new();
            while self.undo.records.len() > mark.log_len {
                let Some(record) = self.undo.pop() else {
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
            // After every record inverse: a foreign-restore inverse may
            // revive a slot the scope allocated before that restore.
            self.retire_slots_from(mark.slot_marks);
            self.journal.rollback_scope(mark.journal_len, &created);
            self.mutation_ticks = mark.ticks;
        }
        self.undo.scopes.retain(|scope| scope.mark.seq < mark.seq);
        if self.undo.scopes.is_empty() {
            self.undo.clear();
        }
    }

    /// Prepares for a full restore. `lands_on_snapshot` answers, for this
    /// value before the restore, whether the restore lands exactly on
    /// `snapshot`'s state; only then can an unchanged same-lineage snapshot,
    /// cloned while every live scope was already open, truncate to its
    /// source prefix, and while scopes are live only if that prefix still
    /// rewinds the snapshot to their entry states (see
    /// [`Self::copy_prefix_is_intact`]). It is asked last, once the rest of
    /// the certificate holds. A changed clone, a foreign snapshot or an
    /// inexact restore needs one full inverse while scopes are live;
    /// otherwise its state cannot be reconstructed from the destination's
    /// mutation log. An armed append-only guard always trips: its slot marks
    /// no longer describe the swapped state.
    pub(crate) fn undo_truncate_for_foreign_restore(
        &mut self,
        snapshot: &Self,
        lands_on_snapshot: impl FnOnce(&Self) -> bool,
    ) {
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
            // A scope opened after the clone has an entry state the snapshot
            // does not hold, whatever the log lengths say.
            && self.undo.scopes.iter().all(|scope| scope.mark.seq < base.seq)
            && (self.undo.scopes.is_empty() || self.copy_prefix_is_intact(snapshot, base.log_len))
            && lands_on_snapshot(self)
        {
            self.undo.truncate(base.log_len);
            self.undo
                .scopes
                .retain(|scope| scope.mark.log_len <= base.log_len);
            if self.undo.scopes.is_empty() {
                self.undo.clear();
            }
        } else if self.scopes_alive() {
            self.undo.push(UndoRecord::ForeignRestore {
                old: Box::new(self.clone()),
            });
        } else {
            self.undo.clear();
            self.undo.scopes.clear();
            self.undo.generation = self.undo.generation.wrapping_add(1);
            // The swapped state has no source prefix in this log (an inexact
            // restore rolls the tick back to a state it does not restore). A
            // later clone must not claim an inherited prefix of the old
            // topology.
            let lineage_id = UndoLog::default().lineage_id;
            self.undo.lineage_id = lineage_id;
            self.undo.base = UndoBase {
                lineage_id,
                generation: self.undo.generation,
                log_len: 0,
                ticks: snapshot.mutation_ticks,
                seq: self.undo.next_seq,
            };
            self.undo.inherited_lineage = false;
        }
    }

    /// Whether cutting this log to its first `cut` records leaves records
    /// that rewind `snapshot` to every live scope's entry state, given that
    /// every live scope was opened before the snapshot's clone point. The
    /// lineage counters cannot tell on their own: a clone of a clone carries
    /// its ancestor's position, attribute, body-class and journal writes
    /// move no tick, and a bare length cannot see records replaced by a
    /// rewind.
    ///
    /// - The snapshot was cloned from this very value, at a position `cut`
    ///   records long, and is unchanged since (cache generation, which
    ///   every write bumps, and the journal fingerprint).
    /// - This log still holds those records: the serial at the copy point
    ///   is the clone-time one (serials are never reissued, so a rewind or
    ///   an earlier cut that replaced a record shows).
    fn copy_prefix_is_intact(&self, snapshot: &Self, cut: usize) -> bool {
        snapshot.undo.origin.is_some_and(|origin| {
            origin.cache_lineage == self.cache_lineage
                && !snapshot.is_cache_poisoned()
                && origin.cache_generation == snapshot.cache_generation
                && origin.journal == snapshot.journal.append_fingerprint()
                && origin.log_len == cut
                && self.undo.holds_prefix(cut, origin.last_serial)
        })
    }

    /// Retires every live slot at or above `marks` (in [`ArenaTag`] order):
    /// the allocations of a rewound scope. Each slot keeps its last value as
    /// an inaccessible tombstone, so the handle stays stale forever without
    /// its index ever being reused.
    fn retire_slots_from(&mut self, marks: [usize; 10]) {
        let [v, e, w, f, sh, so, c, cs, l, ce] = marks;
        self.vertices.retire_from(v);
        self.edges.retire_from(e);
        self.wires.retire_from(w);
        self.faces.retire_from(f);
        self.shells.retire_from(sh);
        self.solids.retire_from(so);
        self.compounds.retire_from(c);
        self.compsolids.retire_from(cs);
        self.loops.retire_from(l);
        self.coedges.retire_from(ce);
    }

    /// Applies one record's inverse. Slot restores write the old value back
    /// and revive liveness.
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
                        UndoRecord::Vertex { old: old_vertex, .. },
                        UndoRecord::Face { old: old_face, .. },
                    ] if format!("{:?}", Some(old_vertex)) == vertex
                        && format!("{:?}", Some(old_face)) == face
                ));
            } else {
                assert_eq!(topo.undo_record_count(), 0, "no scope keeps no record");
            }
            drop(ticket);
        }
    }

    /// Live model state: every live slot, the use index, attributes, the
    /// journal and the tick count. Tombstone contents are excluded.
    fn live_state(topo: &Topology) -> String {
        let mut rows = vec![
            format!("{:?}", topo.vertices.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.edges.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.wires.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.faces.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.shells.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.solids.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.loops.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.coedges.iter().collect::<Vec<_>>()),
            format!("{:?}", topo.pcurves),
            format!("{:?}", topo.journal),
            format!("{}", topo.mutation_ticks),
        ];
        for (id, _) in topo.faces.iter() {
            rows.push(format!("{:?}", topo.attributes.face(id)));
        }
        rows.join("\n")
    }

    /// The spec-copying `add_face` body that `install_fresh_face_loops`
    /// replaced, kept as its oracle.
    fn legacy_add_face(topo: &mut Topology, mut value: Face) -> FaceId {
        let mut wire_ids = vec![value.outer_wire()];
        wire_ids.extend(value.inner_wires().iter().copied());
        let specs = topo.boundary_loop_specs(&wire_ids, None);
        value.replace_boundary_loops(Vec::new());
        topo.mutation_ticks = topo.mutation_ticks.saturating_add(1);
        topo.bump_cache_generation();
        let face_id = topo.faces.alloc(value);
        if let Ok(specs) = specs {
            let _ = topo.install_face_loop_specs_carrying(
                face_id,
                specs,
                std::collections::HashMap::new(),
                false,
            );
        }
        face_id
    }

    /// A closed polygon wire over fresh vertices and edges.
    fn polygon(topo: &mut Topology, points: &[(f64, f64)]) -> WireId {
        let v: Vec<_> = points
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)))
            .collect();
        let edges = (0..v.len())
            .map(|i| {
                let edge = topo.add_edge(Edge::new(v[i], v[(i + 1) % v.len()], EdgeCurve::Line));
                OrientedEdge::new(edge, true)
            })
            .collect();
        topo.add_wire(Wire::new(edges, true).unwrap())
    }

    type FaceBuild = fn(&mut Topology, [EdgeId; 2], (WireId, EdgeId)) -> Face;

    #[test]
    fn fresh_face_install_matches_the_spec_install_oracle() {
        let mut base = Topology::new();
        let (_, [e0, e1, _]) = triangle(&mut base);
        // Stale handles: allocated inside a rolled-back scope.
        let mut stale = None;
        let _ = run_transacted(&mut base, |topo| {
            let wire = polygon(topo, &[(5.0, 5.0), (6.0, 5.0), (6.0, 6.0)]);
            stale = Some((wire, topo.wire(wire)?.edges()[0].edge()));
            Err::<(), _>(TopologyError::WireNotClosed)
        });
        let stale = stale.unwrap();

        let cases: &[(&str, FaceBuild)] = &[
            ("outer only", |topo, _, _| {
                let outer = polygon(topo, &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]);
                Face::new(outer, vec![], plane())
            }),
            ("outer and inner", |topo, _, _| {
                let outer = polygon(topo, &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]);
                let inner = polygon(topo, &[(1.0, 1.0), (1.0, 2.0), (2.0, 1.0)]);
                let hole = polygon(topo, &[(3.0, 3.0), (3.0, 3.5), (3.5, 3.0)]);
                Face::new_reversed(outer, vec![inner, hole], plane())
            }),
            ("open wire over pre-existing edges", |topo, [e0, e1], _| {
                let uses = vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, false)];
                let wire = topo.add_wire(Wire::new(uses, false).unwrap());
                Face::new(wire, vec![], plane())
            }),
            ("duplicated pre-existing use", |topo, [e0, _], _| {
                let uses = vec![OrientedEdge::new(e0, true), OrientedEdge::new(e0, true)];
                let wire = topo.add_wire(Wire::new(uses, true).unwrap());
                Face::new(wire, vec![], plane())
            }),
            ("stale outer wire", |_, _, (wire, _)| {
                Face::new(wire, vec![], plane())
            }),
            ("stale inner edge", |topo, [e0, _], (_, edge)| {
                let outer = polygon(topo, &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]);
                let uses = vec![OrientedEdge::new(e0, false), OrientedEdge::new(edge, true)];
                let inner = topo.add_wire(Wire::new(uses, true).unwrap());
                Face::new(outer, vec![inner], plane())
            }),
        ];
        for &(name, build) in cases {
            for (scope, guard) in [(false, false), (true, false), (true, true)] {
                let run = |legacy: bool| {
                    let mut topo = base.clone();
                    let ticket = scope.then(|| topo.undo_begin_scope());
                    if guard {
                        topo.arm_append();
                    }
                    let face = build(&mut topo, [e0, e1], stale);
                    let id = if legacy {
                        legacy_add_face(&mut topo, face)
                    } else {
                        topo.add_face(face)
                    };
                    let tripped = guard && topo.disarm_append();
                    let generation = topo.cache_identity().generation;
                    let outcome = (id, tripped, snapshot(&topo), generation);
                    drop(ticket);
                    outcome
                };
                let outcome = run(false);
                assert_eq!(outcome, run(true), "{name}, scope {scope}, guard {guard}");
                let expect_trip = guard && name == "duplicated pre-existing use";
                assert_eq!(outcome.1, expect_trip, "{name}");
            }
        }
    }

    /// Builds an axis-aligned unit box from scratch (8 vertices, 12 edges,
    /// 6 wires and faces, a shell and a solid).
    fn unit_box(topo: &mut Topology, ox: f64) -> SolidId {
        let corner = |i: usize| {
            let bit = |set: bool| if set { 1.0 } else { 0.0 };
            Point3::new(
                ox + bit(matches!(i % 4, 1 | 2)),
                bit(matches!(i % 4, 2 | 3)),
                bit(i >= 4),
            )
        };
        let v: Vec<_> = (0..8)
            .map(|i| topo.add_vertex(Vertex::new(corner(i), 1e-7)))
            .collect();
        let pairs = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];
        let e: Vec<_> = pairs
            .iter()
            .map(|&(a, b)| topo.add_edge(Edge::new(v[a], v[b], EdgeCurve::Line)))
            .collect();
        let faces = [
            [(0, false), (3, false), (2, false), (1, false)],
            [(4, true), (5, true), (6, true), (7, true)],
            [(0, true), (9, true), (4, false), (8, false)],
            [(2, true), (11, true), (6, false), (10, false)],
            [(3, true), (8, true), (7, false), (11, false)],
            [(1, true), (10, true), (5, false), (9, false)],
        ]
        .map(|uses| {
            let uses = uses
                .iter()
                .map(|&(i, forward)| OrientedEdge::new(e[i], forward))
                .collect();
            let wire = topo.add_wire(Wire::new(uses, true).unwrap());
            topo.add_face(Face::new(wire, vec![], plane()))
        });
        let shell = topo.add_shell(Shell::new(faces.to_vec()).unwrap());
        topo.add_solid(Solid::new(shell, vec![]))
    }

    /// Work guard: construction inside a scope records exactly one entry per
    /// indexed boundary use (24 for a box) — no allocation or fresh-slot
    /// overwrite records — whatever the size of the surrounding document,
    /// and a rewind still retires every staged slot.
    #[test]
    fn construction_in_a_scope_records_only_index_uses() {
        for preexisting in [0_u8, 1, 8] {
            let mut topo = Topology::new();
            for i in 0..preexisting {
                unit_box(&mut topo, f64::from(i) * 2.0);
            }
            let before = live_state(&topo);
            let slots = topo.slot_mark_lens();
            let ticket = topo.undo_begin_scope();
            let solid = unit_box(&mut topo, -5.0);
            let staged = crate::explorer::solid_faces(&topo, solid).unwrap();
            // Fresh-slot writes through the public API record nothing.
            topo.solid_mut(solid).unwrap();
            topo.face_mut(staged[0]).unwrap().set_reversed(true);
            assert_eq!(topo.undo_record_count(), 24, "preexisting {preexisting}");
            assert!(
                topo.undo
                    .records
                    .iter()
                    .all(|record| matches!(record, UndoRecord::PcurveUse { old: None, .. })),
                "only pure index additions are recorded"
            );
            topo.undo_rewind_scope(ticket.mark);
            drop(ticket);
            assert_eq!(live_state(&topo), before, "preexisting {preexisting}");
            assert_eq!(topo.undo_record_count(), 0);
            assert!(topo.solid(solid).is_err());
            for face in staged {
                assert!(topo.face(face).is_err());
                assert!(topo.pcurves_for_face(face).is_empty());
            }
            // Staged slots stay allocated as tombstones; none is reissued.
            let after = topo.slot_mark_lens();
            for (tag, (&now, then)) in after.iter().zip(slots).enumerate() {
                let grown = !matches!(tag, 6 | 7); // no compounds or comp-solids
                assert_eq!(now > then, grown, "arena {tag}");
            }
            assert!(topo.add_empty_solid().index() > solid.index());
        }
    }

    #[test]
    fn nested_overwrite_of_an_outer_allocation_still_restores() {
        let point = |x: f64| Point3::new(x, 0.0, 0.0);
        let mut topo = Topology::new();
        let (face, _) = triangle(&mut topo);
        let before = live_state(&topo);
        let outer = topo.undo_begin_scope();

        // Writes to the scope's own allocation record nothing.
        let v = topo.add_vertex(Vertex::new(point(1.0), 1e-7));
        topo.vertex_mut(v).unwrap().set_point(point(2.0));
        assert_eq!(topo.undo_record_count(), 0);
        // A pre-existing face still records its prior value.
        topo.face_mut(face).unwrap().set_reversed(true);
        assert_eq!(topo.undo_record_count(), 1);

        // For a nested scope the outer allocation is pre-existing: the
        // overwrite is recorded and a caught inner failure restores it.
        let inner = topo.undo_begin_scope();
        topo.vertex_mut(v).unwrap().set_point(point(3.0));
        assert_eq!(topo.undo_record_count(), 2);
        let w = topo.add_vertex(Vertex::new(point(4.0), 1e-7));
        topo.vertex_mut(w).unwrap().set_point(point(5.0));
        topo.record_vertex_retire(w).unwrap();
        assert_eq!(
            topo.undo_record_count(),
            2,
            "inner-own slots record nothing"
        );
        topo.undo_rewind_scope(inner.mark);
        drop(inner);
        assert_eq!(topo.vertex(v).unwrap().point(), point(2.0));
        assert!(topo.vertex(w).is_err());
        assert_eq!(topo.undo_record_count(), 1);

        // A dropped (lazily committed) inner scope hands its records to the
        // outer scope; afterwards the outer allocation is own-slot again.
        {
            let dropped = topo.undo_begin_scope();
            topo.vertex_mut(v).unwrap().set_point(point(6.0));
            assert_eq!(topo.undo_record_count(), 2);
            drop(dropped);
        }
        topo.vertex_mut(v).unwrap().set_point(point(7.0));
        assert_eq!(topo.undo_record_count(), 2);

        // The outer rewind retires the allocation and restores the face.
        topo.undo_rewind_scope(outer.mark);
        drop(outer);
        assert!(topo.vertex(v).is_err());
        assert_eq!(live_state(&topo), before);
        assert_eq!(topo.undo_record_count(), 0);
        let fresh = topo.add_vertex(Vertex::new(point(8.0), 1e-7));
        assert!(
            fresh.index() > w.index(),
            "rolled-back slots are never reissued"
        );
    }

    #[test]
    fn foreign_restores_keep_skipped_slots_rewindable() {
        let point = |x: f64| Point3::new(x, 0.0, 0.0);
        // Same-lineage clone taken after the allocation: the restore brings
        // back the clone-time value, and the scope rewind still retires it.
        let mut topo = Topology::new();
        let (face, _) = triangle(&mut topo);
        let before = live_state(&topo);
        let mut staged = None;
        let result = run_transacted(&mut topo, |topo| {
            let v = topo.add_vertex(Vertex::new(point(1.0), 1e-7));
            staged = Some(v);
            let snapshot = topo.clone();
            topo.vertex_mut(v)?.set_point(point(2.0));
            topo.restore_for_rollback(&snapshot);
            assert_eq!(topo.vertex(v)?.point(), point(1.0));
            topo.vertex_mut(v)?.set_point(point(3.0));
            topo.face_mut(face)?.set_reversed(true);
            Err::<(), _>(TopologyError::WireNotClosed)
        });
        assert!(result.is_err());
        assert_eq!(live_state(&topo), before);
        assert!(topo.vertex(staged.unwrap()).is_err());

        // Unrelated, larger snapshot: its slots above the scope's marks are
        // written without records; the full pre-restore inverse plus the
        // slot-mark retirement still return the exact entry state.
        let mut larger = Topology::new();
        for i in 0..3 {
            unit_box(&mut larger, f64::from(i) * 2.0);
        }
        let mut topo = Topology::new();
        let (face, _) = triangle(&mut topo);
        let before = live_state(&topo);
        let marks = topo.slot_mark_lens();
        let result = run_transacted(&mut topo, |topo| {
            topo.restore_for_rollback(&larger);
            let high = marks[ArenaTag::Vertex as usize] + 2;
            let high = topo.vertex_id_from_index(high).unwrap();
            let low = topo.vertex_id_from_index(0).unwrap();
            topo.vertex_mut(high)?.set_point(point(9.0));
            topo.vertex_mut(low)?.set_point(point(9.0));
            topo.add_vertex(Vertex::new(point(10.0), 1e-7));
            Err::<(), _>(TopologyError::WireNotClosed)
        });
        assert!(result.is_err());
        assert_eq!(live_state(&topo), before);
        assert!(topo.face(face).is_ok());
    }

    /// Documents off the restore fixed point: a standalone face whose index
    /// entries went with a deleted solid that shared its edges (a profile
    /// face whose extrusion was deleted), a face whose loop was retired, and
    /// a stale index entry.
    fn off_fixed_point_documents() -> Vec<(&'static str, Topology, FaceId)> {
        let mut shared = Topology::new();
        let (profile, edges) = triangle(&mut shared);
        let uses = edges
            .iter()
            .rev()
            .map(|&edge| OrientedEdge::new(edge, false))
            .collect();
        let cap_wire = shared.add_wire(Wire::new(uses, true).unwrap());
        let cap = shared.add_face(Face::new(cap_wire, vec![], plane()));
        let shell = shared.add_shell(Shell::new(vec![cap]).unwrap());
        let body = shared.add_solid(Solid::new(shell, vec![]));
        unit_box(&mut shared, 5.0);
        assert!(shared.is_restore_fixed_point());
        shared.delete_solid(body).unwrap();

        let mut dead_loop = Topology::new();
        let (face, _) = triangle(&mut dead_loop);
        unit_box(&mut dead_loop, 5.0);
        let loop_id = dead_loop.loops_of_face(face).unwrap()[0];
        assert!(dead_loop.loops.retire(loop_id));

        let mut stale_use = Topology::new();
        let (stale_face, [edge, ..]) = triangle(&mut stale_use);
        unit_box(&mut stale_use, 5.0);
        let coedge = stale_use.coedges_of_edge(edge)[0];
        stale_use.pcurves.index_use(edge, stale_face, false, coedge);

        let documents = vec![
            ("shared edges deleted", shared, profile),
            ("dead loop", dead_loop, face),
            ("stale use", stale_use, stale_face),
        ];
        for (name, document, _) in &documents {
            assert!(!document.is_restore_fixed_point(), "{name}");
        }
        documents
    }

    #[test]
    fn barrier_restore_off_the_fixed_point_inside_a_scope_keeps_the_outer_rollback_exact() {
        // A barrier restore rebuilds the pcurve index from loop/coedge
        // authority and re-derives faces whose authority is not live, so on
        // a snapshot off the restore fixed point it lands on that rebuild,
        // not on the snapshot, even with nothing retired. The enclosing
        // scope must still rewind to its own entry state.
        for (name, mut t, face) in off_fixed_point_documents() {
            let before = live_state(&t);
            let outer = crate::transaction::RollbackSnapshot::capture(&mut t);
            let barrier = t.clone();
            t.face_mut(face).unwrap().set_reversed(true);
            t.restore_preserving_handle_slots(&barrier);
            assert!(t.is_restore_fixed_point(), "{name}");
            assert_ne!(
                live_state(&t),
                before,
                "{name}: the rebuild changed the index"
            );
            outer.restore(&mut t);
            assert_eq!(live_state(&t), before, "{name}");
        }
    }
}
