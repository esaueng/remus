//! Persistent topological references and their resolver (RFC 0003,
//! Stage 2).
//!
//! A [`PersistentRef`] names an entity independently of arena handles: an
//! **anchor** establishes a starting point in the evolution journal, and
//! resolution chases the entity's construction lineage forward through
//! every subsequent entry to the present model. References are value
//! objects — serializable, hashable, holding no arena ids — so nothing
//! about them dangles when arenas change.
//!
//! # Resolution discipline
//!
//! Inherited from the evolution rules and binding here:
//!
//! - **Wrong is worse than none**: a reference never silently rebinds.
//!   Every failure mode is a typed [`Resolution`] state convertible to a
//!   stable diagnostic.
//! - **Fail closed**: an entity whose lineage crosses a barrier, an
//!   in-scope entry that makes no claim about it, or an `unresolved`
//!   event that might absorb it, resolves
//!   [`Resolution::UnresolvedAcrossOperation`] naming the operation —
//!   never a guess.
//! - **Splits are normal modeling**: a reference to an entity that later
//!   splits resolves [`Resolution::BoundMany`] over all pieces; callers
//!   wanting one piece add discriminators.
//! - **Provenance is disclosed**: a chain that hops through a
//!   geometry-derived (inferred) entry resolves with
//!   [`Provenance::Inferred`], and consumers decide per policy whether
//!   inferred rebinding is acceptable.
//!
//! Identity flows only through identity claims (`Preserved`, `Modified`,
//! `Merged`). `Generated` is an adjacency claim — a face that generated a
//! section edge did not *become* that edge — so lineage never follows it.

use crate::Topology;
use crate::journal::{
    EntityEvent, EntityKey, EntityKind, EntryPayload, JournalEntry, JournalOrdinal, OpId,
    RecordedOrigin,
};

/// A persistent reference: an anchor plus optional discriminators.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PersistentRef {
    /// Where the reference is anchored in history.
    pub anchor: Anchor,
    /// Filters applied, in order, to the resolved entity set. A
    /// discriminator that empties the set fails the resolution loudly
    /// ([`Resolution::NoMatch`]) naming itself — never silently.
    pub discriminators: Vec<Discriminator>,
    /// The entity kind this reference addresses.
    pub entity_kind: EntityKind,
}

impl PersistentRef {
    /// A reference to the `index`-th output of kind `entity_kind` recorded
    /// by operation `operation` (in the entry's deterministic event
    /// order). Exact; construction-derived.
    #[must_use]
    pub fn operation_output(operation: OpId, entity_kind: EntityKind, index: usize) -> Self {
        Self {
            anchor: Anchor::OperationOutput { operation, index },
            discriminators: Vec::new(),
            entity_kind,
        }
    }

    /// A reference to whatever `base` has evolved into.
    #[must_use]
    pub fn lineage_of(base: Self) -> Self {
        let entity_kind = base.entity_kind;
        Self {
            anchor: Anchor::LineageOf {
                base: Box::new(base),
            },
            discriminators: Vec::new(),
            entity_kind,
        }
    }

    /// Adds a discriminator (builder style).
    #[must_use]
    pub fn with_discriminator(mut self, discriminator: Discriminator) -> Self {
        self.discriminators.push(discriminator);
        self
    }
}

/// Where a [`PersistentRef`] is anchored in history.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Anchor {
    /// The `index`-th output entity of the reference's kind recorded by
    /// `operation`, in the entry's deterministic event order (outputs are
    /// the entry's subjects except `Deleted` ones, which are inputs).
    ///
    /// Replay-stable: identical model history journals identical entries,
    /// so the same `(operation, index)` addresses the same entity.
    OperationOutput {
        /// The journaled operation.
        operation: OpId,
        /// Zero-based position among that entry's outputs of the
        /// reference's entity kind.
        index: usize,
    },
    /// Whatever the entity referenced by `base` has evolved into.
    ///
    /// Resolution already chases every anchor to the present, so this
    /// adds nothing over `base` alone today; it exists so a reference can
    /// wrap another reference (re-anchoring, and composition with the
    /// Stage 3 signature tier) without changing its meaning.
    LineageOf {
        /// The wrapped reference.
        base: Box<PersistentRef>,
    },
}

/// A typed filter on the resolved entity set.
///
/// Stage 2 carries the geometry-type discriminators; adjacency, proximity
/// (`NearestTo`), and operation-declared output roles are queued with the
/// Stage 3 signature tier.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Discriminator {
    /// Keep only faces whose surface type tag matches (e.g. `"plane"`,
    /// `"cylinder"`; see `FaceSurface::type_tag`).
    SurfaceType(String),
    /// Keep only edges whose curve type tag matches (e.g. `"line"`,
    /// `"circle"`; see `EdgeCurve::type_tag`).
    CurveType(String),
}

impl Discriminator {
    /// Stable name used when a resolution reports which discriminator
    /// eliminated every candidate.
    #[must_use]
    pub fn describe(&self) -> String {
        match self {
            Self::SurfaceType(tag) => format!("surface_type:{tag}"),
            Self::CurveType(tag) => format!("curve_type:{tag}"),
        }
    }
}

/// Whether a resolution rests on construction records alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Provenance {
    /// Every hop in the chain was a construction record.
    Construction,
    /// At least one hop rode on a geometry-derived (inferred) entry.
    Inferred,
}

/// The result of resolving a [`PersistentRef`] against the current model.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Resolution {
    /// The reference binds one entity.
    Bound {
        /// The entity's current arena key.
        entity: EntityKey,
        /// Whether the chain was construction-derived throughout.
        provenance: Provenance,
    },
    /// The reference binds a set (the target split; all pieces, in
    /// deterministic order). Callers wanting one piece add
    /// discriminators.
    BoundMany {
        /// The pieces' current arena keys, sorted.
        entities: Vec<EntityKey>,
        /// Whether the chain was construction-derived throughout.
        provenance: Provenance,
    },
    /// Several entities satisfy the reference equally and resolution will
    /// not pick one. Unused by the Stage 2 anchors (splits are
    /// `BoundMany`, unresolved events fail closed); the Stage 3 signature
    /// tier resolves here when a signature matches more than one entity.
    Ambiguous {
        /// The candidates, sorted.
        candidates: Vec<EntityKey>,
        /// Why they cannot be separated.
        reason: String,
    },
    /// The target was deleted.
    Dangling {
        /// The operation that deleted the last live piece.
        deleted_at: OpId,
    },
    /// The lineage crosses an operation whose records cannot carry it: a
    /// barrier, an in-scope entry with no claim about the entity, or an
    /// `unresolved` event that might absorb it. Fails closed naming the
    /// operation whose records are missing.
    UnresolvedAcrossOperation {
        /// The operation the lineage could not cross.
        op: OpId,
        /// That operation's stable kind name.
        kind: String,
    },
    /// The anchor's operation is not in the journal — never journaled, or
    /// truncated by a checkpoint rollback (its `OpId` is never reissued,
    /// so the reference dangles rather than rebinding).
    UnknownOperation {
        /// The unknown operation id.
        op: OpId,
    },
    /// The anchor or a discriminator eliminated every candidate; `reason`
    /// names which (diagnosable, not just "not found").
    NoMatch {
        /// What eliminated the candidates.
        reason: String,
    },
}

impl Resolution {
    /// Converts a binding resolution into its entities, and every
    /// non-binding state into the matching typed error
    /// (`ref_*` diagnostic codes).
    ///
    /// # Errors
    ///
    /// [`TopologyError`](crate::TopologyError) variants `RefAmbiguous`,
    /// `RefDangling`, `RefUnresolvedAcrossOperation`,
    /// `RefUnknownOperation`, `RefNoMatch` — one per non-binding state.
    pub fn into_entities(self) -> Result<(Vec<EntityKey>, Provenance), crate::TopologyError> {
        match self {
            Self::Bound { entity, provenance } => Ok((vec![entity], provenance)),
            Self::BoundMany {
                entities,
                provenance,
            } => Ok((entities, provenance)),
            Self::Ambiguous { candidates, reason } => Err(crate::TopologyError::RefAmbiguous {
                candidates: candidates.len(),
                reason,
            }),
            Self::Dangling { deleted_at } => Err(crate::TopologyError::RefDangling {
                deleted_at: deleted_at.value(),
            }),
            Self::UnresolvedAcrossOperation { op, kind } => {
                Err(crate::TopologyError::RefUnresolvedAcrossOperation {
                    op: op.value(),
                    operation_kind: kind,
                })
            }
            Self::UnknownOperation { op } => {
                Err(crate::TopologyError::RefUnknownOperation { op: op.value() })
            }
            Self::NoMatch { reason } => Err(crate::TopologyError::RefNoMatch { reason }),
        }
    }
}

/// Resolves a reference against the current model.
///
/// The anchor establishes a starting ordinal set; the chase applies every
/// subsequent journal entry's claims to it (identity claims only); the
/// discriminators filter the surviving entities; the final set maps
/// through the live index to current arena keys. Deterministic: identical
/// history plus identical reference resolves identically.
#[must_use]
pub fn resolve(topo: &Topology, reference: &PersistentRef) -> Resolution {
    let journal = topo.journal();
    let entries = journal.entries();

    // 1. Anchor.
    let (start_after, ordinals, mut provenance) = match &reference.anchor {
        Anchor::OperationOutput { operation, index } => {
            let Some(position) = entries.iter().position(|entry| entry.op() == *operation) else {
                return Resolution::UnknownOperation { op: *operation };
            };
            let entry = &entries[position];
            let EntryPayload::Evolution { origin, events, .. } = entry.payload() else {
                return Resolution::NoMatch {
                    reason: format!(
                        "anchor: operation {} is a barrier and has no outputs",
                        operation.value()
                    ),
                };
            };
            let outputs: Vec<JournalOrdinal> = events
                .iter()
                .filter(|(subject, event)| {
                    !matches!(event, EntityEvent::Deleted)
                        && journal
                            .key_of(*subject)
                            .is_some_and(|key| key.kind == reference.entity_kind)
                })
                .map(|(subject, _)| *subject)
                .collect();
            let Some(&ordinal) = outputs.get(*index) else {
                return Resolution::NoMatch {
                    reason: format!(
                        "anchor: operation {} has {} {} outputs, index {} out of range",
                        operation.value(),
                        outputs.len(),
                        reference.entity_kind.as_str(),
                        index
                    ),
                };
            };
            let provenance = if *origin == RecordedOrigin::Construction {
                Provenance::Construction
            } else {
                Provenance::Inferred
            };
            (position + 1, vec![ordinal], provenance)
        }
        Anchor::LineageOf { base } => {
            // The base already chases to the present; only this
            // reference's own discriminators remain.
            let (mut entities, provenance) = match resolve(topo, base) {
                Resolution::Bound { entity, provenance } => (vec![entity], provenance),
                Resolution::BoundMany {
                    entities,
                    provenance,
                } => (entities, provenance),
                other => return other,
            };
            if let Some(no_match) =
                apply_discriminators(topo, &reference.discriminators, &mut entities)
            {
                return no_match;
            }
            return bind(entities, provenance);
        }
    };

    // 2. Chase forward through every subsequent entry.
    let mut current = ordinals;
    for entry in &entries[start_after..] {
        match chase_one_entry(entry, &current, &mut provenance) {
            ChaseStep::Continue(next) => current = next,
            ChaseStep::Stop(resolution) => return resolution,
        }
    }

    // 3. Live index → current arena keys.
    let mut entities: Vec<EntityKey> = current
        .iter()
        .filter_map(|&ordinal| journal.key_of(ordinal))
        .filter(|key| key.kind == reference.entity_kind)
        .collect();
    entities.sort_unstable();

    // 4. Discriminators.
    if let Some(no_match) = apply_discriminators(topo, &reference.discriminators, &mut entities) {
        return no_match;
    }
    bind(entities, provenance)
}

fn bind(entities: Vec<EntityKey>, provenance: Provenance) -> Resolution {
    match entities.as_slice() {
        [] => Resolution::NoMatch {
            reason: "no entity survived resolution".to_owned(),
        },
        [single] => Resolution::Bound {
            entity: *single,
            provenance,
        },
        _ => Resolution::BoundMany {
            entities,
            provenance,
        },
    }
}

enum ChaseStep {
    Continue(Vec<JournalOrdinal>),
    Stop(Resolution),
}

/// Applies one entry's claims to the current ordinal set.
fn chase_one_entry(
    entry: &JournalEntry,
    current: &[JournalOrdinal],
    provenance: &mut Provenance,
) -> ChaseStep {
    let sever = || {
        ChaseStep::Stop(Resolution::UnresolvedAcrossOperation {
            op: entry.op(),
            kind: entry.kind().to_owned(),
        })
    };
    let (origin, scope, events) = match entry.payload() {
        EntryPayload::GlobalBarrier => return sever(),
        EntryPayload::Barrier { affected } => {
            if current
                .iter()
                .any(|ordinal| affected.binary_search(ordinal).is_ok())
            {
                return sever();
            }
            return ChaseStep::Continue(current.to_vec());
        }
        EntryPayload::Evolution {
            origin,
            scope,
            events,
        } => (*origin, scope, events),
    };

    let mut next: Vec<JournalOrdinal> = Vec::with_capacity(current.len());
    let mut hopped = false;
    for &ordinal in current {
        if scope.binary_search(&ordinal).is_err() {
            // Untouched by this operation: carries through.
            next.push(ordinal);
            continue;
        }
        // In scope. An `unresolved` output that might absorb the entity
        // makes every other claim about it unsafe to follow.
        let contested = events.iter().any(|(_, event)| {
            matches!(event, EntityEvent::Unresolved { candidates } if candidates.contains(&ordinal))
        });
        if contested {
            return sever();
        }
        let mut deleted = false;
        let mut followed = false;
        for (subject, event) in events {
            match event {
                EntityEvent::Deleted if *subject == ordinal => deleted = true,
                EntityEvent::Preserved { from } | EntityEvent::Modified { from }
                    if *from == ordinal =>
                {
                    next.push(*subject);
                    followed = true;
                }
                EntityEvent::Merged { from } if from.contains(&ordinal) => {
                    next.push(*subject);
                    followed = true;
                }
                _ => {}
            }
        }
        if deleted && followed {
            // Contradictory claims are a recording defect; fail closed.
            return sever();
        }
        if deleted {
            // This branch of the lineage ends here; survivors continue.
            continue;
        }
        if !followed {
            // In scope, no claim: the operation touched this entity's
            // solid but did not account for the entity.
            return sever();
        }
        hopped = true;
    }
    if hopped && origin == RecordedOrigin::Geometry {
        *provenance = Provenance::Inferred;
    }
    next.sort_unstable();
    next.dedup();
    if next.is_empty() {
        return ChaseStep::Stop(Resolution::Dangling {
            deleted_at: entry.op(),
        });
    }
    ChaseStep::Continue(next)
}

/// Filters `entities` through the discriminators in order. Returns the
/// failing resolution if one empties the set.
fn apply_discriminators(
    topo: &Topology,
    discriminators: &[Discriminator],
    entities: &mut Vec<EntityKey>,
) -> Option<Resolution> {
    for discriminator in discriminators {
        entities.retain(|key| match discriminator {
            Discriminator::SurfaceType(tag) => {
                key.kind == EntityKind::Face
                    && topo
                        .face_id_from_index(key.index)
                        .and_then(|id| topo.face(id).ok())
                        .is_some_and(|face| face.surface().type_tag() == tag)
            }
            Discriminator::CurveType(tag) => {
                key.kind == EntityKind::Edge
                    && topo
                        .edge_id_from_index(key.index)
                        .and_then(|id| topo.edge(id).ok())
                        .is_some_and(|edge| edge.curve().type_tag() == tag)
            }
        });
        if entities.is_empty() {
            return Some(Resolution::NoMatch {
                reason: format!(
                    "discriminator {} eliminated every candidate",
                    discriminator.describe()
                ),
            });
        }
    }
    None
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use crate::TopologyError;
    use crate::journal::{EventDraft, EvolutionDraft, UNJOURNALED_MUTATIONS};

    use super::*;

    /// Records one construction entry and returns its op id.
    fn record(
        topo: &mut Topology,
        kind: &str,
        events: Vec<(EntityKey, EventDraft)>,
        extra_scope: Vec<EntityKey>,
    ) -> OpId {
        let pending = topo.journal_begin(kind);
        let mut draft = EvolutionDraft::construction();
        draft.events = events;
        draft.add_scope(extra_scope);
        topo.journal_record_evolution(pending, draft).unwrap()
    }

    fn modified(from: usize, to: usize) -> (EntityKey, EventDraft) {
        (
            EntityKey::face(to),
            EventDraft::Modified {
                from: EntityKey::face(from),
            },
        )
    }

    #[test]
    fn operation_output_addresses_the_kth_output_of_the_kind() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "boolean_fuse",
            vec![
                (
                    EntityKey::face(10),
                    EventDraft::Generated {
                        sources: Vec::new(),
                    },
                ),
                (
                    EntityKey::edge(20),
                    EventDraft::Generated {
                        sources: Vec::new(),
                    },
                ),
                (EntityKey::face(3), EventDraft::Deleted),
                (
                    EntityKey::face(11),
                    EventDraft::Generated {
                        sources: Vec::new(),
                    },
                ),
            ],
            Vec::new(),
        );

        // Face outputs are faces 10 and 11 (deleted face 3 is an input,
        // edge 20 is another kind), in event (ordinal) order.
        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::Bound {
                entity: EntityKey::face(10),
                provenance: Provenance::Construction
            }
        );
        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 1),
        );
        assert_eq!(
            r,
            Resolution::Bound {
                entity: EntityKey::face(11),
                provenance: Provenance::Construction
            }
        );
        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Edge, 0),
        );
        assert_eq!(
            r,
            Resolution::Bound {
                entity: EntityKey::edge(20),
                provenance: Provenance::Construction
            }
        );
        // Out of range is diagnosable, not "not found".
        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 2),
        );
        assert!(matches!(r, Resolution::NoMatch { ref reason } if reason.contains("out of range")));
    }

    #[test]
    fn lineage_follows_modified_chains_to_the_present() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        record(&mut topo, "op_b", vec![modified(1, 5)], Vec::new());
        record(&mut topo, "op_c", vec![modified(5, 9)], Vec::new());

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::Bound {
                entity: EntityKey::face(9),
                provenance: Provenance::Construction
            }
        );
    }

    #[test]
    fn a_split_resolves_bound_many_over_all_pieces() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        record(
            &mut topo,
            "boolean_cut",
            vec![modified(1, 7), modified(1, 8)],
            Vec::new(),
        );

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::BoundMany {
                entities: vec![EntityKey::face(7), EntityKey::face(8)],
                provenance: Provenance::Construction
            }
        );
    }

    #[test]
    fn a_merge_resolves_every_input_to_the_merged_entity() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![
                (
                    EntityKey::face(1),
                    EventDraft::Generated {
                        sources: Vec::new(),
                    },
                ),
                (
                    EntityKey::face(2),
                    EventDraft::Generated {
                        sources: Vec::new(),
                    },
                ),
            ],
            Vec::new(),
        );
        record(
            &mut topo,
            "unify_same_domain",
            vec![(
                EntityKey::face(9),
                EventDraft::Merged {
                    from: vec![EntityKey::face(1), EntityKey::face(2)],
                },
            )],
            Vec::new(),
        );

        for index in 0..2 {
            let r = resolve(
                &topo,
                &PersistentRef::operation_output(op, EntityKind::Face, index),
            );
            assert_eq!(
                r,
                Resolution::Bound {
                    entity: EntityKey::face(9),
                    provenance: Provenance::Construction
                },
                "both merge inputs resolve to the merged face"
            );
        }
    }

    #[test]
    fn a_deleted_target_dangles_naming_the_deleting_operation() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        let deleter = record(
            &mut topo,
            "boolean_cut",
            vec![(EntityKey::face(1), EventDraft::Deleted)],
            Vec::new(),
        );

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::Dangling {
                deleted_at: deleter
            }
        );
        let err = r.into_entities().unwrap_err();
        assert!(
            matches!(err, TopologyError::RefDangling { deleted_at } if deleted_at == deleter.value())
        );
    }

    #[test]
    fn an_explicit_barrier_fails_closed_naming_the_operation() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        let pending = topo.journal_begin("offset_solid");
        let barrier = topo.journal_record_barrier(pending, vec![EntityKey::face(1)]);

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::UnresolvedAcrossOperation {
                op: barrier,
                kind: "offset_solid".to_owned()
            }
        );
    }

    #[test]
    fn entities_outside_an_entry_scope_carry_through_unchanged() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        // A later operation on a DIFFERENT solid: face 1 is not in its
        // scope, so the reference survives it — this is what makes the
        // journal usable on multi-body models.
        record(
            &mut topo,
            "boolean_fuse",
            vec![modified(50, 60)],
            vec![EntityKey::face(51), EntityKey::edge(52)],
        );

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::Bound {
                entity: EntityKey::face(1),
                provenance: Provenance::Construction
            }
        );
    }

    #[test]
    fn in_scope_entities_without_a_claim_are_severed() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        // A faces-partial entry that declares face 1 in scope but makes
        // no claim about it (the fillet-rebuilt-my-edges case).
        let severer = record(
            &mut topo,
            "fillet",
            vec![modified(2, 3)],
            vec![EntityKey::face(1)],
        );

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::UnresolvedAcrossOperation {
                op: severer,
                kind: "fillet".to_owned()
            },
            "absent claims are gaps within the declared scope"
        );
    }

    #[test]
    fn an_unresolved_output_contests_every_candidate() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        // Face 1 flowed into face 7 — but output 8 might ALSO be face 1;
        // following the claim would risk a wrong binding.
        let contester = record(
            &mut topo,
            "boolean_fuse",
            vec![
                modified(1, 7),
                (
                    EntityKey::face(8),
                    EventDraft::Unresolved {
                        candidates: vec![EntityKey::face(1)],
                    },
                ),
            ],
            Vec::new(),
        );

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::UnresolvedAcrossOperation {
                op: contester,
                kind: "boolean_fuse".to_owned()
            }
        );
    }

    #[test]
    fn a_geometry_origin_hop_downgrades_provenance_to_inferred() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        let pending = topo.journal_begin("legacy_fillet");
        let mut draft = EvolutionDraft::geometry();
        draft.push(
            EntityKey::face(5),
            EventDraft::Modified {
                from: EntityKey::face(1),
            },
        );
        topo.journal_record_evolution(pending, draft).unwrap();

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert_eq!(
            r,
            Resolution::Bound {
                entity: EntityKey::face(5),
                provenance: Provenance::Inferred
            },
            "a chain that rode on inference must say so"
        );
    }

    #[test]
    fn a_rolled_back_operation_id_resolves_unknown_never_rebinds() {
        let mut topo = Topology::new();
        record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        let snapshot = topo.clone();
        let rolled_back = record(&mut topo, "op_b", vec![modified(1, 5)], Vec::new());
        topo.restore_preserving_handle_slots(&snapshot);
        // A new operation after the rollback — its id is fresh, never the
        // rolled-back one.
        let fresh = record(&mut topo, "op_c", vec![modified(1, 6)], Vec::new());
        assert_ne!(fresh, rolled_back);

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(rolled_back, EntityKind::Face, 0),
        );
        assert_eq!(r, Resolution::UnknownOperation { op: rolled_back });
    }

    #[test]
    fn lineage_of_wraps_without_changing_meaning() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        record(&mut topo, "op_b", vec![modified(1, 5)], Vec::new());

        let base = PersistentRef::operation_output(op, EntityKind::Face, 0);
        let wrapped = PersistentRef::lineage_of(base.clone());
        assert_eq!(resolve(&topo, &base), resolve(&topo, &wrapped));
    }

    #[test]
    fn anchoring_on_a_barrier_entry_reports_no_outputs() {
        let mut topo = Topology::new();
        let pending = topo.journal_begin("offset_solid");
        let barrier = topo.journal_record_barrier(pending, vec![EntityKey::face(1)]);

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(barrier, EntityKind::Face, 0),
        );
        assert!(matches!(r, Resolution::NoMatch { ref reason } if reason.contains("barrier")));
    }

    #[test]
    fn a_global_barrier_severs_resolution() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        // Unjournaled mutation → synthetic global barrier at next begin.
        topo.add_vertex(crate::vertex::Vertex::new(
            brepkit_math::vec::Point3::new(0.0, 0.0, 0.0),
            1e-7,
        ));
        record(&mut topo, "op_b", vec![modified(50, 51)], Vec::new());

        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, 0),
        );
        assert!(
            matches!(
                r,
                Resolution::UnresolvedAcrossOperation { ref kind, .. }
                    if kind == UNJOURNALED_MUTATIONS
            ),
            "{r:?}"
        );
    }

    #[test]
    fn a_discriminator_that_empties_the_set_reports_itself() {
        let mut topo = Topology::new();
        let op = record(
            &mut topo,
            "op_a",
            vec![(
                EntityKey::face(1),
                EventDraft::Generated {
                    sources: Vec::new(),
                },
            )],
            Vec::new(),
        );
        // Face index 1 has no live arena face in this synthetic topology,
        // so any surface-type filter eliminates it.
        let reference = PersistentRef::operation_output(op, EntityKind::Face, 0)
            .with_discriminator(Discriminator::SurfaceType("plane".into()));
        let r = resolve(&topo, &reference);
        assert!(
            matches!(r, Resolution::NoMatch { ref reason } if reason.contains("surface_type:plane")),
            "{r:?}"
        );
        let err = r.into_entities().unwrap_err();
        assert!(matches!(err, TopologyError::RefNoMatch { .. }));
    }
}
