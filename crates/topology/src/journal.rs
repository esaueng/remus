//! Append-only evolution journal (RFC 0003, Stage 1).
//!
//! Persistent topological naming needs history, not just the final state:
//! a reference like "the face this face evolved into" is resolved by
//! chasing operation records, and those records must survive arena churn,
//! checkpoint restores, and (later) copy compaction. The journal is that
//! history: an append-only, per-topology log of operation entries, each
//! describing what happened to the entities an operation touched.
//!
//! # Journal-local ordinals
//!
//! Entries never store arena indices. Each entity the journal has seen is
//! assigned a [`JournalOrdinal`] — a journal-local identity that is stable
//! for the life of the journal and **never reused** — and a live index maps
//! ordinals to current arena keys. Copy compaction (`deferred-e6b`) or a
//! restore rewrites only the index, never the entries, which is what makes
//! the journal the persistent spine while arena handles stay session-local.
//!
//! # Fail closed on gaps
//!
//! The stage-1 exit gate of RFC 0003 is that no operation is silently
//! absent from history. That guarantee is structural, not a caller
//! discipline: [`Topology`](crate::Topology) counts every mutation
//! (a *mutation tick*), each entry records the tick count at completion,
//! and [`Topology::journal_begin`](crate::Topology::journal_begin) compares
//! the current count against the last entry. Any mutation the journal was
//! not told about — an unjournaled operation, a failed operation's partial
//! work, a direct edit — triggers a synthetic **global barrier** entry, so
//! a later resolver refuses continuity across the gap instead of pretending
//! the history is complete. Operations that produce no evolution records
//! journal an explicit scoped [`EntryPayload::Barrier`] themselves.
//!
//! # What an entry claims, and its scope
//!
//! Every entry carries a **scope**: the set of entities the operation may
//! have touched (its operands' and result's entities), declared by the
//! recording operation as a construction fact. The scope is what makes the
//! fail-closed rule usable on multi-body models:
//!
//! - an entity **outside** the scope was untouched and carries through the
//!   entry unchanged;
//! - an entity **inside** the scope follows its claim if the entry makes
//!   one, and is **severed** (fails closed) if it does not — a faces-only
//!   entry from a blend leaves that solid's edge and vertex references
//!   unresolvable across the operation, while every other solid's entities
//!   are unaffected.
//!
//! Absent claims are gaps within the declared scope, never implicit
//! preservation — the "wrong is worse than none" discipline from
//! `operations::evolution`, applied to history. A scope that omits touched
//! entities would fake continuity, so it is a claim held to the same
//! standard as the events themselves; the ingestion helpers in
//! `operations::journal_ops` derive it from the operand and result solids.

use std::collections::HashMap;

use crate::TopologyError;

/// Identifier of one journaled operation invocation.
///
/// Issued monotonically by the journal and **never reused**, including
/// across checkpoint restores (the counter is high-water preserved exactly
/// like arena slots), so a persistent reference to a rolled-back operation
/// can never silently rebind to a later one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OpId(u64);

impl OpId {
    /// The raw monotonic value, for display and serialization.
    #[must_use]
    pub fn value(self) -> u64 {
        self.0
    }

    /// Rebuilds an id from its raw value (deserialization). A value the
    /// journal never issued simply resolves
    /// [`UnknownOperation`](crate::naming::Resolution::UnknownOperation) —
    /// constructing one cannot forge history.
    #[must_use]
    pub fn from_value(value: u64) -> Self {
        Self(value)
    }
}

/// Journal-local stable identity of one entity.
///
/// Ordinals are assigned when an entity is first mentioned in an entry and
/// are never reused, including across restores. They are the identity that
/// persistent references ride on; arena indices appear only in the live
/// index, which maps each ordinal to the entity's *current* arena key.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JournalOrdinal(u64);

impl JournalOrdinal {
    /// The raw monotonic value, for display and serialization.
    #[must_use]
    pub fn value(self) -> u64 {
        self.0
    }
}

/// The entity kinds the journal tracks (RFC 0003 reference model).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EntityKind {
    /// A vertex.
    Vertex,
    /// An edge.
    Edge,
    /// A face.
    Face,
}

impl EntityKind {
    /// Stable lowercase name, for diagnostics and serialization.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Vertex => "vertex",
            Self::Edge => "edge",
            Self::Face => "face",
        }
    }
}

/// An entity's current arena identity: kind plus arena index.
///
/// This is the *session-local* half of the mapping — the journal's entries
/// hold ordinals, and the live index translates between the two.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EntityKey {
    /// Which arena the entity lives in.
    pub kind: EntityKind,
    /// The entity's arena index (`Id::index()`).
    pub index: usize,
}

impl EntityKey {
    /// The placeholder arena index for an entity that exists in journal
    /// history but not in the current session — an entity that was not
    /// exported to (or restored from) a serialized document. No live
    /// entity ever has this index, so lookups through such a key fail
    /// typed and resolution reports the entity as not present rather
    /// than binding it. The kind half of the key stays meaningful, which
    /// keeps `OperationOutput` output ordering replay-stable across a
    /// round trip.
    pub const UNMAPPED: usize = usize::MAX;

    /// A face key.
    #[must_use]
    pub fn face(index: usize) -> Self {
        Self {
            kind: EntityKind::Face,
            index,
        }
    }

    /// An edge key.
    #[must_use]
    pub fn edge(index: usize) -> Self {
        Self {
            kind: EntityKind::Edge,
            index,
        }
    }

    /// A vertex key.
    #[must_use]
    pub fn vertex(index: usize) -> Self {
        Self {
            kind: EntityKind::Vertex,
            index,
        }
    }
}

/// How an evolution entry's events were derived.
///
/// Mirrors the evolution discipline: `Construction` events are what the
/// operation itself recorded while building its result; `Geometry` events
/// are an inference that can be wrong even when it reports no ambiguity.
/// A resolver must surface the difference to its callers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordedOrigin {
    /// The operation reported each event from its own construction records.
    Construction,
    /// Events were inferred from geometry after the fact.
    Geometry,
}

impl RecordedOrigin {
    /// Stable lowercase name, for diagnostics and serialization.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Construction => "construction",
            Self::Geometry => "geometry",
        }
    }
}

/// One entity's event within one journal entry, over journal ordinals.
///
/// The subject of the event is the entity the event is keyed under in the
/// entry. `Preserved`, `Modified`, `Generated` and `Merged` subjects are
/// *result* entities; a `Deleted` subject is an *input* entity that ceased
/// to exist; an `Unresolved` subject is a result entity the operation could
/// not attribute (the honest bucket — never guessed).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EntityEvent {
    /// The subject IS `from`, carried through unchanged.
    Preserved {
        /// The input entity the subject is.
        from: JournalOrdinal,
    },
    /// The subject is `from`, modified (re-trimmed, split — a split is
    /// several `Modified` subjects sharing one `from`).
    Modified {
        /// The input entity the subject is a modified piece of.
        from: JournalOrdinal,
    },
    /// The subject is new geometry, built from `sources` (possibly of a
    /// different entity kind — a section edge names its generating faces;
    /// empty when the operation synthesised it from nothing it can name).
    Generated {
        /// The input entities the subject was built from.
        sources: Vec<JournalOrdinal>,
    },
    /// The subject is the merge of several inputs flowing into one output
    /// (a same-domain merge). References to any input resolve to the
    /// subject.
    Merged {
        /// The input entities that merged into the subject.
        from: Vec<JournalOrdinal>,
    },
    /// The subject (an input entity) was deleted by the operation.
    Deleted,
    /// The subject's origin could not be established; `candidates` are the
    /// inputs that could not be told apart (empty if nothing was
    /// plausible). A resolver fails closed here.
    Unresolved {
        /// Input entities that were plausible but inseparable sources.
        candidates: Vec<JournalOrdinal>,
    },
}

/// One entity's event in a caller-supplied draft, over arena keys.
///
/// Drafts are what operations hand the journal; the journal interns every
/// key into a [`JournalOrdinal`] when the entry is recorded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EventDraft {
    /// See [`EntityEvent::Preserved`].
    Preserved {
        /// The input entity the subject is.
        from: EntityKey,
    },
    /// See [`EntityEvent::Modified`].
    Modified {
        /// The input entity the subject is a modified piece of.
        from: EntityKey,
    },
    /// See [`EntityEvent::Generated`].
    Generated {
        /// The input entities the subject was built from.
        sources: Vec<EntityKey>,
    },
    /// See [`EntityEvent::Merged`].
    Merged {
        /// The input entities that merged into the subject.
        from: Vec<EntityKey>,
    },
    /// See [`EntityEvent::Deleted`].
    Deleted,
    /// See [`EntityEvent::Unresolved`].
    Unresolved {
        /// Input entities that were plausible but inseparable sources.
        candidates: Vec<EntityKey>,
    },
}

/// A complete evolution record for one operation, over arena keys.
#[derive(Debug, Clone)]
pub struct EvolutionDraft {
    /// The subject entities and their events. One event per subject; a
    /// duplicate subject is refused
    /// ([`TopologyError::JournalDuplicateEvent`]).
    pub events: Vec<(EntityKey, EventDraft)>,
    /// Whether the events are construction records or geometric inference —
    /// a claim the recording operation must make explicitly.
    pub origin: RecordedOrigin,
    /// Entities the operation may have touched beyond those its events
    /// mention (typically the result solid's full entity set for a
    /// partial record). The entry's effective scope is these plus every
    /// entity the events mention plus the pre-operation scope captured on
    /// the [`PendingOp`]. In-scope entities without a claim are severed;
    /// out-of-scope entities carry through.
    pub scope: Vec<EntityKey>,
}

impl EvolutionDraft {
    /// An empty construction-origin draft.
    #[must_use]
    pub fn construction() -> Self {
        Self {
            events: Vec::new(),
            origin: RecordedOrigin::Construction,
            scope: Vec::new(),
        }
    }

    /// An empty geometry-origin (inferred) draft.
    #[must_use]
    pub fn geometry() -> Self {
        Self {
            events: Vec::new(),
            origin: RecordedOrigin::Geometry,
            scope: Vec::new(),
        }
    }

    /// Adds one subject's event.
    pub fn push(&mut self, subject: EntityKey, event: EventDraft) {
        self.events.push((subject, event));
    }

    /// Adds entities to the declared scope (see [`Self::scope`]).
    pub fn add_scope(&mut self, keys: impl IntoIterator<Item = EntityKey>) {
        self.scope.extend(keys);
    }
}

/// What one journal entry records.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EntryPayload {
    /// A real evolution record: per-entity events, sorted by subject
    /// ordinal (deterministic).
    Evolution {
        /// Whether the events are construction records or inference.
        origin: RecordedOrigin,
        /// The entities the operation may have touched (sorted, deduped;
        /// always a superset of everything the events mention). In-scope
        /// entities without a claim are severed across this entry;
        /// out-of-scope entities carry through.
        scope: Vec<JournalOrdinal>,
        /// Subject ordinal → its event, sorted by subject.
        events: Vec<(JournalOrdinal, EntityEvent)>,
    },
    /// An explicit barrier: the operation produced no evolution records,
    /// and every listed entity is unresolved across it. A resolver chasing
    /// a reference through this entry fails closed naming the operation.
    Barrier {
        /// The entities (post-operation) whose history is severed here,
        /// sorted by ordinal.
        affected: Vec<JournalOrdinal>,
    },
    /// A synthetic barrier the journal inserted itself because mutations
    /// happened that no entry accounts for. Scope is unknown, so it severs
    /// continuity for **every** entity.
    GlobalBarrier,
}

/// One recorded operation in the journal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JournalEntry {
    op: OpId,
    kind: String,
    payload: EntryPayload,
    ticks_after: u64,
}

impl JournalEntry {
    /// The operation's journal identifier.
    #[must_use]
    pub fn op(&self) -> OpId {
        self.op
    }

    /// The stable operation name (e.g. `boolean_fuse`). Synthetic global
    /// barriers use `unjournaled_mutations`.
    #[must_use]
    pub fn kind(&self) -> &str {
        &self.kind
    }

    /// What the entry records.
    #[must_use]
    pub fn payload(&self) -> &EntryPayload {
        &self.payload
    }

    /// Whether this entry severs continuity (explicit or global barrier).
    #[must_use]
    pub fn is_barrier(&self) -> bool {
        matches!(
            self.payload,
            EntryPayload::Barrier { .. } | EntryPayload::GlobalBarrier
        )
    }

    /// The topology's mutation-tick count when the entry was recorded.
    /// [`Topology::journal_begin`](crate::Topology::journal_begin) compares
    /// this against the live count to detect unjournaled mutations.
    #[must_use]
    pub fn ticks_after(&self) -> u64 {
        self.ticks_after
    }
}

/// Report of one journal-driven attribute propagation pass
/// (RFC 0003, Stage 4); see
/// [`Topology::propagate_attributes_for_op`](crate::Topology::propagate_attributes_for_op).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct JournalAttributePropagation {
    /// Result faces that received attributes.
    pub carried: usize,
    /// Face-kind `unresolved` outputs, left bare (reported, never
    /// guessed).
    pub unresolved_outputs: usize,
    /// Merged outputs whose attributed inputs disagreed, left bare — a
    /// merge does not toss coins between names.
    pub merge_conflicts: usize,
    /// The entry is geometry-derived and the caller did not opt into
    /// inferred propagation; nothing was carried.
    pub refused_inferred: bool,
}

/// Kind name of the synthetic entry inserted when mutations happened that
/// no journal entry accounts for.
pub const UNJOURNALED_MUTATIONS: &str = "unjournaled_mutations";

/// Proof that [`Topology::journal_begin`](crate::Topology::journal_begin)
/// ran before an operation was recorded.
///
/// `journal_begin` is where unjournaled-mutation gaps are detected, so
/// recording requires this token: it cannot be constructed any other way,
/// is not cloneable, and is consumed by the record call. Dropping it
/// without recording is safe — the operation's mutations then surface as a
/// gap (global barrier) at the next `journal_begin`.
///
/// Because the token exists *before* the operation runs, it is also where
/// the **pre-operation** half of the entry's scope is captured: entities
/// of the operands that the operation may touch (they may be retired by
/// the time the entry is recorded). Add them with [`Self::add_scope`].
#[derive(Debug)]
pub struct PendingOp {
    pub(crate) kind: String,
    pub(crate) scope: Vec<EntityKey>,
}

impl PendingOp {
    /// Adds pre-operation entities to the entry's scope: entities the
    /// operation may touch, captured before it runs. Merged into the
    /// recorded entry's effective scope (evolution and barrier alike).
    pub fn add_scope(&mut self, keys: impl IntoIterator<Item = EntityKey>) {
        self.scope.extend(keys);
    }
}

/// The append-only evolution journal, owned by
/// [`Topology`](crate::Topology).
///
/// Record through [`Topology::journal_begin`](crate::Topology::journal_begin)
/// / [`Topology::journal_record_evolution`](crate::Topology::journal_record_evolution)
/// / [`Topology::journal_record_barrier`](crate::Topology::journal_record_barrier);
/// read through [`Topology::journal`](crate::Topology::journal).
#[derive(Debug, Default, Clone)]
pub struct Journal {
    entries: Vec<JournalEntry>,
    next_op: u64,
    next_ordinal: u64,
    /// Live index: ordinal → the entity's current arena key.
    key_by_ordinal: HashMap<u64, EntityKey>,
    /// Reverse live index: arena key → ordinal.
    ordinal_by_key: HashMap<EntityKey, JournalOrdinal>,
}

impl Journal {
    /// All recorded entries, oldest first.
    #[must_use]
    pub fn entries(&self) -> &[JournalEntry] {
        &self.entries
    }

    /// Number of recorded entries.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether nothing has been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The ordinal assigned to an entity, if the journal has seen it.
    #[must_use]
    pub fn ordinal_of(&self, key: EntityKey) -> Option<JournalOrdinal> {
        self.ordinal_by_key.get(&key).copied()
    }

    /// The current arena key of a journaled entity (the live index).
    ///
    /// The key identifies the arena slot the entity occupied when last
    /// journaled; because arena slots are never reused, a retired entity's
    /// key simply fails lookups rather than aliasing anything.
    #[must_use]
    pub fn key_of(&self, ordinal: JournalOrdinal) -> Option<EntityKey> {
        self.key_by_ordinal.get(&ordinal.0).copied()
    }

    /// Every evolution event whose subject is `ordinal`, oldest first.
    ///
    /// Barriers are not events; check [`Self::barriers_crossing`] as well
    /// before trusting a continuity chain.
    #[must_use]
    pub fn events_for(&self, ordinal: JournalOrdinal) -> Vec<(OpId, &EntityEvent)> {
        let mut out = Vec::new();
        for entry in &self.entries {
            if let EntryPayload::Evolution { events, .. } = &entry.payload
                && let Ok(found) = events.binary_search_by_key(&ordinal, |(subject, _)| *subject)
            {
                out.push((entry.op, &events[found].1));
            }
        }
        out
    }

    /// Every barrier that severs `ordinal`'s continuity, oldest first: an
    /// explicit barrier listing it, or any global barrier (unknown scope
    /// severs everything).
    #[must_use]
    pub fn barriers_crossing(&self, ordinal: JournalOrdinal) -> Vec<OpId> {
        self.entries
            .iter()
            .filter(|entry| match &entry.payload {
                EntryPayload::Barrier { affected } => affected.binary_search(&ordinal).is_ok(),
                EntryPayload::GlobalBarrier => true,
                EntryPayload::Evolution { .. } => false,
            })
            .map(|entry| entry.op)
            .collect()
    }

    /// The tick count recorded by the newest entry, if any.
    pub(crate) fn last_ticks(&self) -> Option<u64> {
        self.entries.last().map(|entry| entry.ticks_after)
    }

    /// Interns an arena key, assigning a fresh ordinal on first sight.
    fn intern(&mut self, key: EntityKey) -> JournalOrdinal {
        if let Some(&ordinal) = self.ordinal_by_key.get(&key) {
            return ordinal;
        }
        let ordinal = JournalOrdinal(self.next_ordinal);
        self.next_ordinal = self.next_ordinal.saturating_add(1);
        self.ordinal_by_key.insert(key, ordinal);
        self.key_by_ordinal.insert(ordinal.0, key);
        ordinal
    }

    fn issue_op(&mut self) -> OpId {
        let op = OpId(self.next_op);
        self.next_op = self.next_op.saturating_add(1);
        op
    }

    /// Records an evolution entry. Duplicate subjects are refused: one
    /// entry making two claims about one entity is a recording bug, and a
    /// resolver must never have to pick between them.
    pub(crate) fn record_evolution(
        &mut self,
        kind: String,
        pre_scope: Vec<EntityKey>,
        draft: EvolutionDraft,
        ticks_after: u64,
    ) -> Result<OpId, TopologyError> {
        let mut scope: Vec<JournalOrdinal> = pre_scope
            .into_iter()
            .chain(draft.scope)
            .map(|key| self.intern(key))
            .collect();
        let mut events: Vec<(JournalOrdinal, EntityEvent)> = Vec::with_capacity(draft.events.len());
        for (subject, event) in draft.events {
            let subject = self.intern(subject);
            scope.push(subject);
            let event = match event {
                EventDraft::Preserved { from } => EntityEvent::Preserved {
                    from: self.intern(from),
                },
                EventDraft::Modified { from } => EntityEvent::Modified {
                    from: self.intern(from),
                },
                EventDraft::Generated { sources } => EntityEvent::Generated {
                    sources: sources.into_iter().map(|key| self.intern(key)).collect(),
                },
                EventDraft::Merged { from } => EntityEvent::Merged {
                    from: from.into_iter().map(|key| self.intern(key)).collect(),
                },
                EventDraft::Deleted => EntityEvent::Deleted,
                EventDraft::Unresolved { candidates } => EntityEvent::Unresolved {
                    candidates: candidates.into_iter().map(|key| self.intern(key)).collect(),
                },
            };
            // The scope is a superset of everything the events mention.
            match &event {
                EntityEvent::Preserved { from } | EntityEvent::Modified { from } => {
                    scope.push(*from);
                }
                EntityEvent::Generated { sources } => scope.extend(sources.iter().copied()),
                EntityEvent::Merged { from } => scope.extend(from.iter().copied()),
                EntityEvent::Unresolved { candidates } => scope.extend(candidates.iter().copied()),
                EntityEvent::Deleted => {}
            }
            events.push((subject, event));
        }
        events.sort_by_key(|(subject, _)| *subject);
        if let Some(window) = events.windows(2).find(|window| window[0].0 == window[1].0) {
            return Err(TopologyError::JournalDuplicateEvent {
                ordinal: window[0].0.value(),
            });
        }
        scope.sort_unstable();
        scope.dedup();
        let op = self.issue_op();
        self.entries.push(JournalEntry {
            op,
            kind,
            payload: EntryPayload::Evolution {
                origin: draft.origin,
                scope,
                events,
            },
            ticks_after,
        });
        Ok(op)
    }

    /// Records an explicit barrier over `affected` plus the pre-operation
    /// scope captured on the pending token.
    pub(crate) fn record_barrier(
        &mut self,
        kind: String,
        pre_scope: Vec<EntityKey>,
        affected: Vec<EntityKey>,
        ticks_after: u64,
    ) -> OpId {
        let mut affected: Vec<JournalOrdinal> = pre_scope
            .into_iter()
            .chain(affected)
            .map(|key| self.intern(key))
            .collect();
        affected.sort_unstable();
        affected.dedup();
        let op = self.issue_op();
        self.entries.push(JournalEntry {
            op,
            kind,
            payload: EntryPayload::Barrier { affected },
            ticks_after,
        });
        op
    }

    /// Records the synthetic global barrier for unaccounted mutations.
    pub(crate) fn record_global_barrier(&mut self, ticks_after: u64) -> OpId {
        let op = self.issue_op();
        self.entries.push(JournalEntry {
            op,
            kind: UNJOURNALED_MUTATIONS.to_owned(),
            payload: EntryPayload::GlobalBarrier,
            ticks_after,
        });
        op
    }

    /// Restores the journal to `snapshot`'s entries and index while
    /// preserving the `OpId` and ordinal high-water marks, so identifiers
    /// issued by rolled-back operations are never reissued to later ones
    /// (the journal analogue of arena slot preservation).
    pub(crate) fn restore_preserving_ids(&mut self, snapshot: &Self) {
        self.entries.clone_from(&snapshot.entries);
        self.key_by_ordinal.clone_from(&snapshot.key_by_ordinal);
        self.ordinal_by_key.clone_from(&snapshot.ordinal_by_key);
        self.next_op = self.next_op.max(snapshot.next_op);
        self.next_ordinal = self.next_ordinal.max(snapshot.next_ordinal);
    }
}

// ─── Serialization snapshots (RFC 0003, Stage 5) ────────────────────────

/// One entity event in a [`JournalSnapshot`], over raw ordinal values.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EventSnapshot {
    /// See [`EntityEvent::Preserved`].
    Preserved {
        /// The input entity's ordinal value.
        from: u64,
    },
    /// See [`EntityEvent::Modified`].
    Modified {
        /// The input entity's ordinal value.
        from: u64,
    },
    /// See [`EntityEvent::Generated`].
    Generated {
        /// The source entities' ordinal values.
        sources: Vec<u64>,
    },
    /// See [`EntityEvent::Merged`].
    Merged {
        /// The merged inputs' ordinal values.
        from: Vec<u64>,
    },
    /// See [`EntityEvent::Deleted`].
    Deleted,
    /// See [`EntityEvent::Unresolved`].
    Unresolved {
        /// The candidate inputs' ordinal values.
        candidates: Vec<u64>,
    },
}

/// One entry payload in a [`JournalSnapshot`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PayloadSnapshot {
    /// See [`EntryPayload::Evolution`].
    Evolution {
        /// Whether the events are construction records.
        construction: bool,
        /// The entry scope, as ordinal values (sorted).
        scope: Vec<u64>,
        /// Subject ordinal value → event, sorted by subject.
        events: Vec<(u64, EventSnapshot)>,
    },
    /// See [`EntryPayload::Barrier`].
    Barrier {
        /// The severed entities' ordinal values (sorted).
        affected: Vec<u64>,
    },
    /// See [`EntryPayload::GlobalBarrier`].
    GlobalBarrier,
}

/// One entry in a [`JournalSnapshot`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EntrySnapshot {
    /// The operation's `OpId` value.
    pub op: u64,
    /// The stable operation kind name.
    pub kind: String,
    /// What the entry records.
    pub payload: PayloadSnapshot,
}

/// A plain-data snapshot of the journal's full state, for serialization
/// (RFC 0003, Stage 5).
///
/// Mutation-tick counts are deliberately **not** part of a snapshot: they
/// are session-local gap-detection state, and
/// [`Journal::from_snapshot`] re-derives a consistent sequence
/// (`Topology::load_journal` then syncs the topology's counter so a
/// clean load is not a gap).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct JournalSnapshot {
    /// The `OpId` high-water counter.
    pub next_op: u64,
    /// The ordinal high-water counter.
    pub next_ordinal: u64,
    /// The live index: ordinal value → current entity key, sorted by
    /// ordinal. Keys with index [`EntityKey::UNMAPPED`] describe entities
    /// that are not present in this session (kind preserved).
    pub index: Vec<(u64, EntityKey)>,
    /// The entries, oldest first.
    pub entries: Vec<EntrySnapshot>,
}

impl Journal {
    /// Captures the journal's full state as plain data.
    #[must_use]
    pub fn snapshot(&self) -> JournalSnapshot {
        let mut index: Vec<(u64, EntityKey)> = self
            .key_by_ordinal
            .iter()
            .map(|(&ordinal, &key)| (ordinal, key))
            .collect();
        index.sort_unstable_by_key(|(ordinal, _)| *ordinal);
        let entries = self
            .entries
            .iter()
            .map(|entry| EntrySnapshot {
                op: entry.op.value(),
                kind: entry.kind.clone(),
                payload: match &entry.payload {
                    EntryPayload::Evolution {
                        origin,
                        scope,
                        events,
                    } => PayloadSnapshot::Evolution {
                        construction: *origin == RecordedOrigin::Construction,
                        scope: scope.iter().map(|o| o.value()).collect(),
                        events: events
                            .iter()
                            .map(|(subject, event)| {
                                let event = match event {
                                    EntityEvent::Preserved { from } => {
                                        EventSnapshot::Preserved { from: from.value() }
                                    }
                                    EntityEvent::Modified { from } => {
                                        EventSnapshot::Modified { from: from.value() }
                                    }
                                    EntityEvent::Generated { sources } => {
                                        EventSnapshot::Generated {
                                            sources: sources.iter().map(|o| o.value()).collect(),
                                        }
                                    }
                                    EntityEvent::Merged { from } => EventSnapshot::Merged {
                                        from: from.iter().map(|o| o.value()).collect(),
                                    },
                                    EntityEvent::Deleted => EventSnapshot::Deleted,
                                    EntityEvent::Unresolved { candidates } => {
                                        EventSnapshot::Unresolved {
                                            candidates: candidates
                                                .iter()
                                                .map(|o| o.value())
                                                .collect(),
                                        }
                                    }
                                };
                                (subject.value(), event)
                            })
                            .collect(),
                    },
                    EntryPayload::Barrier { affected } => PayloadSnapshot::Barrier {
                        affected: affected.iter().map(|o| o.value()).collect(),
                    },
                    EntryPayload::GlobalBarrier => PayloadSnapshot::GlobalBarrier,
                },
            })
            .collect();
        JournalSnapshot {
            next_op: self.next_op,
            next_ordinal: self.next_ordinal,
            index,
            entries,
        }
    }

    /// Rebuilds a journal from a snapshot, validating its invariants.
    ///
    /// Ticks are re-derived (entry position), so a snapshot is
    /// session-portable; install the result with
    /// [`Topology::load_journal`](crate::Topology::load_journal), which
    /// syncs the topology's mutation counter so a clean load does not
    /// read as an unjournaled gap.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::JournalSnapshotInvalid`] when the
    /// snapshot violates a journal invariant: duplicate or out-of-range
    /// ordinals in the index, duplicate keys (other than
    /// [`EntityKey::UNMAPPED`] placeholders), non-increasing or
    /// out-of-range `OpId`s, an event referencing an ordinal absent from
    /// the index, or one entry claiming a subject twice.
    #[allow(clippy::too_many_lines)]
    pub fn from_snapshot(snapshot: JournalSnapshot) -> Result<Self, TopologyError> {
        let invalid = |reason: &str| TopologyError::JournalSnapshotInvalid {
            reason: reason.to_owned(),
        };
        let mut key_by_ordinal = HashMap::with_capacity(snapshot.index.len());
        let mut ordinal_by_key = HashMap::new();
        for &(ordinal, key) in &snapshot.index {
            if ordinal >= snapshot.next_ordinal {
                return Err(invalid("index ordinal at or above the high-water counter"));
            }
            if key_by_ordinal.insert(ordinal, key).is_some() {
                return Err(invalid("duplicate ordinal in the index"));
            }
            if key.index != EntityKey::UNMAPPED
                && ordinal_by_key
                    .insert(key, JournalOrdinal(ordinal))
                    .is_some()
            {
                return Err(invalid("two ordinals share one entity key"));
            }
        }
        let known = |ordinal: u64| key_by_ordinal.contains_key(&ordinal);
        let resolve = |ordinal: u64| -> Result<JournalOrdinal, TopologyError> {
            if known(ordinal) {
                Ok(JournalOrdinal(ordinal))
            } else {
                Err(invalid("event references an ordinal absent from the index"))
            }
        };
        let resolve_all = |ordinals: Vec<u64>| -> Result<Vec<JournalOrdinal>, TopologyError> {
            ordinals.into_iter().map(resolve).collect()
        };

        let mut entries = Vec::with_capacity(snapshot.entries.len());
        let mut previous_op: Option<u64> = None;
        for (position, entry) in snapshot.entries.into_iter().enumerate() {
            if entry.op >= snapshot.next_op {
                return Err(invalid("entry op at or above the high-water counter"));
            }
            if previous_op.is_some_and(|previous| entry.op <= previous) {
                return Err(invalid("entry ops must be strictly increasing"));
            }
            previous_op = Some(entry.op);
            let payload = match entry.payload {
                PayloadSnapshot::Evolution {
                    construction,
                    scope,
                    events,
                } => {
                    let mut scope = resolve_all(scope)?;
                    scope.sort_unstable();
                    scope.dedup();
                    let mut rebuilt = Vec::with_capacity(events.len());
                    for (subject, event) in events {
                        let subject = resolve(subject)?;
                        let event = match event {
                            EventSnapshot::Preserved { from } => EntityEvent::Preserved {
                                from: resolve(from)?,
                            },
                            EventSnapshot::Modified { from } => EntityEvent::Modified {
                                from: resolve(from)?,
                            },
                            EventSnapshot::Generated { sources } => EntityEvent::Generated {
                                sources: resolve_all(sources)?,
                            },
                            EventSnapshot::Merged { from } => EntityEvent::Merged {
                                from: resolve_all(from)?,
                            },
                            EventSnapshot::Deleted => EntityEvent::Deleted,
                            EventSnapshot::Unresolved { candidates } => EntityEvent::Unresolved {
                                candidates: resolve_all(candidates)?,
                            },
                        };
                        rebuilt.push((subject, event));
                    }
                    rebuilt.sort_by_key(|(subject, _)| *subject);
                    if let Some(window) =
                        rebuilt.windows(2).find(|window| window[0].0 == window[1].0)
                    {
                        return Err(TopologyError::JournalDuplicateEvent {
                            ordinal: window[0].0.value(),
                        });
                    }
                    // Every subject and referenced ordinal is in scope.
                    let mut scope_with_events = scope;
                    for (subject, event) in &rebuilt {
                        scope_with_events.push(*subject);
                        match event {
                            EntityEvent::Preserved { from } | EntityEvent::Modified { from } => {
                                scope_with_events.push(*from);
                            }
                            EntityEvent::Generated { sources } => {
                                scope_with_events.extend(sources.iter().copied());
                            }
                            EntityEvent::Merged { from } => {
                                scope_with_events.extend(from.iter().copied());
                            }
                            EntityEvent::Unresolved { candidates } => {
                                scope_with_events.extend(candidates.iter().copied());
                            }
                            EntityEvent::Deleted => {}
                        }
                    }
                    scope_with_events.sort_unstable();
                    scope_with_events.dedup();
                    EntryPayload::Evolution {
                        origin: if construction {
                            RecordedOrigin::Construction
                        } else {
                            RecordedOrigin::Geometry
                        },
                        scope: scope_with_events,
                        events: rebuilt,
                    }
                }
                PayloadSnapshot::Barrier { affected } => {
                    let mut affected = resolve_all(affected)?;
                    affected.sort_unstable();
                    affected.dedup();
                    EntryPayload::Barrier { affected }
                }
                PayloadSnapshot::GlobalBarrier => EntryPayload::GlobalBarrier,
            };
            let ticks_after = u64::try_from(position).unwrap_or(u64::MAX);
            entries.push(JournalEntry {
                op: OpId(entry.op),
                kind: entry.kind,
                payload,
                ticks_after,
            });
        }
        Ok(Self {
            entries,
            next_op: snapshot.next_op,
            next_ordinal: snapshot.next_ordinal,
            key_by_ordinal,
            ordinal_by_key,
        })
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::vec::Point3;

    use crate::Topology;
    use crate::vertex::Vertex;

    use super::*;

    fn draft_one(subject: EntityKey, event: EventDraft) -> EvolutionDraft {
        let mut draft = EvolutionDraft::construction();
        draft.push(subject, event);
        draft
    }

    // ── Journal-driven attribute propagation (RFC 0003, Stage 4) ────────

    /// A minimal live face (one open-wire edge, planar surface) so
    /// attribute propagation has real arena entities to write to.
    fn add_test_face(topo: &mut Topology) -> crate::FaceId {
        use crate::edge::{Edge, EdgeCurve};
        use crate::face::{Face, FaceSurface};
        use crate::wire::{OrientedEdge, Wire};

        let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let e = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let w = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], false).unwrap());
        topo.add_face(Face::new(
            w,
            Vec::new(),
            FaceSurface::Plane {
                normal: remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    fn named(name: &str) -> crate::attributes::EntityAttributes {
        crate::attributes::EntityAttributes {
            name: Some(name.to_owned()),
            ..Default::default()
        }
    }

    #[test]
    fn split_pieces_each_carry_the_name_unchanged() {
        let mut topo = Topology::new();
        let input = add_test_face(&mut topo);
        let piece_a = add_test_face(&mut topo);
        let piece_b = add_test_face(&mut topo);
        topo.set_face_attributes(input, named("mounting face"))
            .unwrap();

        let pending = topo.journal_begin("boolean_cut");
        let mut draft = EvolutionDraft::construction();
        for piece in [piece_a, piece_b] {
            draft.push(
                EntityKey::face(piece.index()),
                EventDraft::Modified {
                    from: EntityKey::face(input.index()),
                },
            );
        }
        let op = topo.journal_record_evolution(pending, draft).unwrap();

        let report = topo.propagate_attributes_for_op(op, false).unwrap();
        assert_eq!(report.carried, 2);
        assert_eq!(report.merge_conflicts, 0);
        for piece in [piece_a, piece_b] {
            assert_eq!(
                topo.attributes().face(piece).unwrap().name.as_deref(),
                Some("mounting face"),
                "a split's pieces keep the name unchanged — never suffixed"
            );
        }
        // Copy-forward only: the input keeps its own attributes.
        assert!(topo.attributes().face(input).is_some());
    }

    #[test]
    fn merges_carry_agreement_and_refuse_conflict() {
        let mut topo = Topology::new();
        let agree_a = add_test_face(&mut topo);
        let agree_b = add_test_face(&mut topo);
        let merged_ok = add_test_face(&mut topo);
        let clash_a = add_test_face(&mut topo);
        let clash_b = add_test_face(&mut topo);
        let merged_clash = add_test_face(&mut topo);
        topo.set_face_attributes(agree_a, named("wall")).unwrap();
        topo.set_face_attributes(agree_b, named("wall")).unwrap();
        topo.set_face_attributes(clash_a, named("wall")).unwrap();
        topo.set_face_attributes(clash_b, named("floor")).unwrap();

        let pending = topo.journal_begin("unify_same_domain");
        let mut draft = EvolutionDraft::construction();
        draft.push(
            EntityKey::face(merged_ok.index()),
            EventDraft::Merged {
                from: vec![
                    EntityKey::face(agree_a.index()),
                    EntityKey::face(agree_b.index()),
                ],
            },
        );
        draft.push(
            EntityKey::face(merged_clash.index()),
            EventDraft::Merged {
                from: vec![
                    EntityKey::face(clash_a.index()),
                    EntityKey::face(clash_b.index()),
                ],
            },
        );
        let op = topo.journal_record_evolution(pending, draft).unwrap();

        let report = topo.propagate_attributes_for_op(op, false).unwrap();
        assert_eq!(report.carried, 1);
        assert_eq!(report.merge_conflicts, 1);
        assert_eq!(
            topo.attributes().face(merged_ok).unwrap().name.as_deref(),
            Some("wall")
        );
        assert!(
            topo.attributes().face(merged_clash).is_none(),
            "disagreeing inputs must not be coin-tossed onto the merge"
        );
    }

    #[test]
    fn generated_and_unresolved_stay_bare() {
        let mut topo = Topology::new();
        let input = add_test_face(&mut topo);
        let band = add_test_face(&mut topo);
        let mystery = add_test_face(&mut topo);
        topo.set_face_attributes(input, named("base")).unwrap();

        let pending = topo.journal_begin("fillet");
        let mut draft = EvolutionDraft::construction();
        draft.push(
            EntityKey::face(band.index()),
            EventDraft::Generated {
                sources: vec![EntityKey::face(input.index())],
            },
        );
        draft.push(
            EntityKey::face(mystery.index()),
            EventDraft::Unresolved {
                candidates: vec![EntityKey::face(input.index())],
            },
        );
        let op = topo.journal_record_evolution(pending, draft).unwrap();

        let report = topo.propagate_attributes_for_op(op, false).unwrap();
        assert_eq!(report.carried, 0);
        assert_eq!(report.unresolved_outputs, 1);
        assert!(topo.attributes().face(band).is_none());
        assert!(topo.attributes().face(mystery).is_none());
    }

    #[test]
    fn inferred_entries_require_opt_in() {
        let mut topo = Topology::new();
        let input = add_test_face(&mut topo);
        let output = add_test_face(&mut topo);
        topo.set_face_attributes(input, named("legacy")).unwrap();

        let pending = topo.journal_begin("legacy_op");
        let mut draft = EvolutionDraft::geometry();
        draft.push(
            EntityKey::face(output.index()),
            EventDraft::Modified {
                from: EntityKey::face(input.index()),
            },
        );
        let op = topo.journal_record_evolution(pending, draft).unwrap();

        let refused = topo.propagate_attributes_for_op(op, false).unwrap();
        assert!(refused.refused_inferred);
        assert_eq!(refused.carried, 0);
        assert!(topo.attributes().face(output).is_none());

        let allowed = topo.propagate_attributes_for_op(op, true).unwrap();
        assert!(!allowed.refused_inferred);
        assert_eq!(allowed.carried, 1);
        assert_eq!(
            topo.attributes().face(output).unwrap().name.as_deref(),
            Some("legacy")
        );
    }

    #[test]
    fn barriers_and_unknown_ops_carry_nothing() {
        let mut topo = Topology::new();
        let face = add_test_face(&mut topo);
        let pending = topo.journal_begin("offset_solid");
        let barrier = topo.journal_record_barrier(pending, vec![EntityKey::face(face.index())]);

        let report = topo.propagate_attributes_for_op(barrier, false).unwrap();
        assert_eq!(
            report,
            crate::journal::JournalAttributePropagation::default()
        );

        let snapshot = topo.clone();
        let pending = topo.journal_begin("rolled_back");
        let rolled_back = topo
            .journal_record_evolution(
                pending,
                draft_one(EntityKey::face(face.index()), EventDraft::Deleted),
            )
            .unwrap();
        topo.restore_preserving_handle_slots(&snapshot);
        let err = topo
            .propagate_attributes_for_op(rolled_back, false)
            .unwrap_err();
        assert!(matches!(err, TopologyError::RefUnknownOperation { .. }));
    }

    // ── Serialization snapshots (RFC 0003, Stage 5) ─────────────────────

    #[test]
    fn snapshot_round_trip_preserves_everything_but_ticks() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("boolean_fuse");
        let mut draft = EvolutionDraft::construction();
        draft.push(
            EntityKey::face(1),
            EventDraft::Modified {
                from: EntityKey::face(0),
            },
        );
        draft.push(
            EntityKey::edge(2),
            EventDraft::Generated {
                sources: vec![EntityKey::face(0)],
            },
        );
        draft.push(EntityKey::vertex(3), EventDraft::Deleted);
        draft.add_scope([EntityKey::face(9)]);
        topo.journal_record_evolution(pending, draft).unwrap();
        let pending = topo.journal_begin("offset_solid");
        topo.journal_record_barrier(pending, vec![EntityKey::face(1)]);

        let snapshot = topo.journal().snapshot();
        let rebuilt = Journal::from_snapshot(snapshot.clone()).unwrap();
        assert_eq!(rebuilt.snapshot(), snapshot, "snapshotting is lossless");
        assert_eq!(rebuilt.entries().len(), 2);
        assert_eq!(
            rebuilt.ordinal_of(EntityKey::face(1)),
            topo.journal().ordinal_of(EntityKey::face(1))
        );
    }

    #[test]
    fn invalid_snapshots_are_refused_whole() {
        use crate::journal::{EntrySnapshot, EventSnapshot, JournalSnapshot, PayloadSnapshot};

        let base = |entries: Vec<EntrySnapshot>, index: Vec<(u64, EntityKey)>| JournalSnapshot {
            next_op: 10,
            next_ordinal: 10,
            index,
            entries,
        };

        // Duplicate ordinal in the index.
        let err = Journal::from_snapshot(base(
            Vec::new(),
            vec![(0, EntityKey::face(0)), (0, EntityKey::face(1))],
        ))
        .unwrap_err();
        assert!(matches!(err, TopologyError::JournalSnapshotInvalid { .. }));

        // Ordinal at the high-water counter.
        let err =
            Journal::from_snapshot(base(Vec::new(), vec![(10, EntityKey::face(0))])).unwrap_err();
        assert!(matches!(err, TopologyError::JournalSnapshotInvalid { .. }));

        // Event referencing an ordinal absent from the index.
        let err = Journal::from_snapshot(base(
            vec![EntrySnapshot {
                op: 0,
                kind: "op".into(),
                payload: PayloadSnapshot::Evolution {
                    construction: true,
                    scope: Vec::new(),
                    events: vec![(0, EventSnapshot::Preserved { from: 7 })],
                },
            }],
            vec![(0, EntityKey::face(0))],
        ))
        .unwrap_err();
        assert!(matches!(err, TopologyError::JournalSnapshotInvalid { .. }));

        // Non-increasing ops.
        let entry = |op: u64| EntrySnapshot {
            op,
            kind: "op".into(),
            payload: PayloadSnapshot::GlobalBarrier,
        };
        let err = Journal::from_snapshot(base(vec![entry(3), entry(3)], Vec::new())).unwrap_err();
        assert!(matches!(err, TopologyError::JournalSnapshotInvalid { .. }));

        // One entry claiming a subject twice.
        let err = Journal::from_snapshot(base(
            vec![EntrySnapshot {
                op: 0,
                kind: "op".into(),
                payload: PayloadSnapshot::Evolution {
                    construction: true,
                    scope: Vec::new(),
                    events: vec![(0, EventSnapshot::Deleted), (0, EventSnapshot::Deleted)],
                },
            }],
            vec![(0, EntityKey::face(0))],
        ))
        .unwrap_err();
        assert!(matches!(err, TopologyError::JournalDuplicateEvent { .. }));
    }

    #[test]
    fn load_journal_syncs_ticks_so_a_clean_load_is_not_a_gap() {
        let mut source = Topology::new();
        let pending = source.journal_begin("op_a");
        source
            .journal_record_evolution(pending, draft_one(EntityKey::face(0), EventDraft::Deleted))
            .unwrap();
        let snapshot = source.journal().snapshot();

        let mut fresh = Topology::new();
        // Simulate the document reader's entity rebuild: mutations before
        // the journal is installed.
        fresh.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        fresh.load_journal(Journal::from_snapshot(snapshot).unwrap());

        // A journaled operation right after the load: no false gap.
        let pending = fresh.journal_begin("op_b");
        fresh
            .journal_record_evolution(pending, draft_one(EntityKey::face(1), EventDraft::Deleted))
            .unwrap();
        assert!(
            fresh
                .journal()
                .entries()
                .iter()
                .all(|entry| entry.kind() != UNJOURNALED_MUTATIONS),
            "a clean load must not read as an unjournaled gap"
        );

        // But a real post-load mutation still severs.
        fresh.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let pending = fresh.journal_begin("op_c");
        fresh.journal_record_barrier(pending, Vec::new());
        assert!(
            fresh
                .journal()
                .entries()
                .iter()
                .any(|entry| entry.kind() == UNJOURNALED_MUTATIONS),
            "gap detection must survive the load"
        );
    }

    #[test]
    fn ordinals_are_stable_and_indexed_both_ways() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op_a");
        topo.journal_record_evolution(
            pending,
            draft_one(
                EntityKey::face(7),
                EventDraft::Modified {
                    from: EntityKey::face(3),
                },
            ),
        )
        .unwrap();

        let subject = topo.journal().ordinal_of(EntityKey::face(7)).unwrap();
        let source = topo.journal().ordinal_of(EntityKey::face(3)).unwrap();
        assert_ne!(subject, source);
        assert_eq!(topo.journal().key_of(subject), Some(EntityKey::face(7)));
        assert_eq!(topo.journal().key_of(source), Some(EntityKey::face(3)));

        // A second entry mentioning face 7 reuses its ordinal.
        let pending = topo.journal_begin("op_b");
        topo.journal_record_evolution(
            pending,
            draft_one(
                EntityKey::face(9),
                EventDraft::Preserved {
                    from: EntityKey::face(7),
                },
            ),
        )
        .unwrap();
        assert_eq!(
            topo.journal().ordinal_of(EntityKey::face(7)),
            Some(subject),
            "an ordinal is the entity's stable journal identity"
        );
    }

    #[test]
    fn kinds_do_not_collide_in_the_index() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op");
        let mut draft = EvolutionDraft::construction();
        draft.push(EntityKey::face(1), EventDraft::Deleted);
        draft.push(EntityKey::edge(1), EventDraft::Deleted);
        draft.push(EntityKey::vertex(1), EventDraft::Deleted);
        topo.journal_record_evolution(pending, draft).unwrap();
        let face = topo.journal().ordinal_of(EntityKey::face(1)).unwrap();
        let edge = topo.journal().ordinal_of(EntityKey::edge(1)).unwrap();
        let vertex = topo.journal().ordinal_of(EntityKey::vertex(1)).unwrap();
        assert_ne!(face, edge);
        assert_ne!(edge, vertex);
    }

    #[test]
    fn duplicate_subject_in_one_entry_is_refused() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op");
        let mut draft = EvolutionDraft::construction();
        draft.push(EntityKey::face(2), EventDraft::Deleted);
        draft.push(
            EntityKey::face(2),
            EventDraft::Preserved {
                from: EntityKey::face(2),
            },
        );
        let err = topo.journal_record_evolution(pending, draft).unwrap_err();
        assert!(matches!(err, TopologyError::JournalDuplicateEvent { .. }));
        assert!(
            topo.journal().is_empty(),
            "a refused entry must not be partially recorded"
        );
    }

    #[test]
    fn unjournaled_mutation_inserts_a_global_barrier() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op_a");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(0), EventDraft::Deleted))
            .unwrap();

        // No mutation between entries: no barrier.
        let pending = topo.journal_begin("op_b");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(1), EventDraft::Deleted))
            .unwrap();
        assert_eq!(topo.journal().len(), 2);

        // An unjournaled mutation (any mutation the journal was not told
        // about) surfaces as a synthetic global barrier at the next begin.
        topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let pending = topo.journal_begin("op_c");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(2), EventDraft::Deleted))
            .unwrap();

        let entries = topo.journal().entries();
        assert_eq!(entries.len(), 4);
        assert_eq!(entries[2].kind(), UNJOURNALED_MUTATIONS);
        assert_eq!(entries[2].payload(), &EntryPayload::GlobalBarrier);
        assert!(entries[2].is_barrier());

        // A global barrier severs every entity, journaled or not.
        let any = topo.journal().ordinal_of(EntityKey::face(0)).unwrap();
        assert_eq!(topo.journal().barriers_crossing(any), vec![entries[2].op()]);
    }

    #[test]
    fn an_abandoned_pending_op_still_surfaces_as_a_gap() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op_a");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(0), EventDraft::Deleted))
            .unwrap();

        // A journaled operation that fails midway: begin ran, mutations
        // happened, but nothing was recorded.
        let abandoned = topo.journal_begin("op_that_failed");
        topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        drop(abandoned);

        let pending = topo.journal_begin("op_b");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(1), EventDraft::Deleted))
            .unwrap();
        assert_eq!(
            topo.journal()
                .entries()
                .iter()
                .filter(|entry| entry.kind() == UNJOURNALED_MUTATIONS)
                .count(),
            1,
            "the failed operation's mutations must not look like continuity"
        );
    }

    #[test]
    fn explicit_barrier_severs_only_listed_entities() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op");
        let mut draft = EvolutionDraft::construction();
        draft.push(EntityKey::face(0), EventDraft::Deleted);
        draft.push(EntityKey::face(1), EventDraft::Deleted);
        topo.journal_record_evolution(pending, draft).unwrap();

        let pending = topo.journal_begin("offset");
        let barrier_op = topo.journal_record_barrier(pending, vec![EntityKey::face(0)]);

        let severed = topo.journal().ordinal_of(EntityKey::face(0)).unwrap();
        let untouched = topo.journal().ordinal_of(EntityKey::face(1)).unwrap();
        assert_eq!(topo.journal().barriers_crossing(severed), vec![barrier_op]);
        assert!(topo.journal().barriers_crossing(untouched).is_empty());
    }

    #[test]
    fn events_for_walks_a_lineage_chain() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op_a");
        topo.journal_record_evolution(
            pending,
            draft_one(
                EntityKey::face(5),
                EventDraft::Modified {
                    from: EntityKey::face(1),
                },
            ),
        )
        .unwrap();
        let pending = topo.journal_begin("op_b");
        topo.journal_record_evolution(
            pending,
            draft_one(
                EntityKey::face(8),
                EventDraft::Preserved {
                    from: EntityKey::face(5),
                },
            ),
        )
        .unwrap();

        let mid = topo.journal().ordinal_of(EntityKey::face(5)).unwrap();
        let events = topo.journal().events_for(mid);
        assert_eq!(events.len(), 1, "face 5 is a subject only in op_a");
        assert!(matches!(events[0].1, EntityEvent::Modified { .. }));

        let last = topo.journal().ordinal_of(EntityKey::face(8)).unwrap();
        let events = topo.journal().events_for(last);
        assert!(
            matches!(events[0].1, EntityEvent::Preserved { from } if *from == mid),
            "the chain links through ordinals, not arena indices"
        );
    }

    #[test]
    fn restore_truncates_entries_but_never_reissues_ids() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op_a");
        let op_a = topo
            .journal_record_evolution(pending, draft_one(EntityKey::face(0), EventDraft::Deleted))
            .unwrap();

        let snapshot = topo.clone();

        let pending = topo.journal_begin("op_b");
        let op_b = topo
            .journal_record_evolution(
                pending,
                draft_one(
                    EntityKey::face(1),
                    EventDraft::Generated {
                        sources: vec![EntityKey::face(0)],
                    },
                ),
            )
            .unwrap();
        let post_ordinal = topo.journal().ordinal_of(EntityKey::face(1)).unwrap();

        topo.restore_preserving_handle_slots(&snapshot);

        // Entries after the checkpoint are truncated with the restore …
        assert_eq!(topo.journal().len(), 1);
        assert_eq!(topo.journal().entries()[0].op(), op_a);
        assert_eq!(topo.journal().ordinal_of(EntityKey::face(1)), None);

        // … but identifiers issued by the rolled-back operation are never
        // reused: a reference held across the rollback can dangle, never
        // silently rebind.
        let pending = topo.journal_begin("op_c");
        let op_c = topo
            .journal_record_evolution(pending, draft_one(EntityKey::face(2), EventDraft::Deleted))
            .unwrap();
        assert!(op_c > op_b, "OpIds are high-water preserved");
        let new_ordinal = topo.journal().ordinal_of(EntityKey::face(2)).unwrap();
        assert!(
            new_ordinal > post_ordinal,
            "ordinals are high-water preserved"
        );
    }

    #[test]
    fn restore_keeps_journal_and_ticks_consistent() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("op_a");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(0), EventDraft::Deleted))
            .unwrap();

        let snapshot = topo.clone();
        // Mutations after the snapshot, rolled back with it:
        topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        topo.restore_preserving_handle_slots(&snapshot);

        // The rolled-back mutations no longer exist, so they are not a gap:
        // the journal and the model state agree again.
        let pending = topo.journal_begin("op_b");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(1), EventDraft::Deleted))
            .unwrap();
        assert!(
            topo.journal()
                .entries()
                .iter()
                .all(|entry| entry.kind() != UNJOURNALED_MUTATIONS),
            "a clean rollback must not read as an unjournaled gap"
        );
    }

    #[test]
    fn first_entry_on_a_pre_populated_topology_is_not_a_gap() {
        let mut topo = Topology::new();
        topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let pending = topo.journal_begin("op_a");
        topo.journal_record_evolution(pending, draft_one(EntityKey::face(0), EventDraft::Deleted))
            .unwrap();
        assert_eq!(
            topo.journal().len(),
            1,
            "pre-journal history is absent, not a barrier: there is no \
             continuity claim before the first entry to protect"
        );
    }
}
