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

    /// A reference resolved by signature matching (inference tier).
    #[must_use]
    pub fn signature(signature: EntitySignature) -> Self {
        let entity_kind = signature.kind;
        Self {
            anchor: Anchor::Signature { signature },
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
    /// An entity matching a typed geometric + adjacency signature — the
    /// **inference tier** (RFC 0003, Stage 3). Resolves against the
    /// current model only (no journal chase), always
    /// [`Provenance::Inferred`]; a signature matching several entities is
    /// [`Resolution::Ambiguous`], never first-match. For recovery —
    /// imported models with no journal, gaps the caller accepts — never
    /// the primary path.
    Signature {
        /// The signature to match.
        signature: EntitySignature,
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

/// Resolves a reference and reads the bound faces' attributes (RFC 0003,
/// Stage 4).
///
/// This is the composition that makes the attribute store
/// reference-keyed: the durable key is the reference; resolution turns
/// it into current entities, and the attribute lookup follows the
/// binding.
///
/// Each bound entity is returned with its attributes (`None` when unset,
/// or for non-face entities — the attribute store's v1 scope). Non-binding
/// resolutions convert to their typed `ref_*` errors, so an attribute can
/// never be read through a dangling, severed, or ambiguous reference.
///
/// # Errors
///
/// The [`Resolution::into_entities`] errors: `RefAmbiguous`,
/// `RefDangling`, `RefUnresolvedAcrossOperation`, `RefUnknownOperation`,
/// `RefNoMatch`.
pub fn resolve_face_attributes<'t>(
    topo: &'t Topology,
    reference: &PersistentRef,
) -> Result<Vec<(EntityKey, Option<&'t crate::attributes::EntityAttributes>)>, crate::TopologyError>
{
    let (entities, _provenance) = resolve(topo, reference).into_entities()?;
    Ok(entities
        .into_iter()
        .map(|key| {
            let attributes = (key.kind == EntityKind::Face)
                .then(|| {
                    topo.face_id_from_index(key.index)
                        .and_then(|id| topo.attributes().face(id))
                })
                .flatten();
            (key, attributes)
        })
        .collect())
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
        Anchor::Signature { signature } => {
            if signature.kind != reference.entity_kind {
                return Resolution::NoMatch {
                    reason: format!(
                        "anchor: signature describes a {}, reference addresses a {}",
                        signature.kind.as_str(),
                        reference.entity_kind.as_str()
                    ),
                };
            }
            let mut entities = resolve_signature(topo, signature);
            if entities.is_empty() {
                return Resolution::NoMatch {
                    reason: "anchor: signature matched nothing".to_owned(),
                };
            }
            if let Some(no_match) =
                apply_discriminators(topo, &reference.discriminators, &mut entities)
            {
                return no_match;
            }
            // A signature is an identity question, not a lineage: several
            // survivors are an ambiguity, never a BoundMany, and never a
            // first-match pick.
            return match entities.as_slice() {
                [single] => Resolution::Bound {
                    entity: *single,
                    provenance: Provenance::Inferred,
                },
                _ => Resolution::Ambiguous {
                    reason: format!("signature matches {} entities", entities.len()),
                    candidates: entities,
                },
            };
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

    // 3. Live index → current arena keys. Keys with the
    // [`EntityKey::UNMAPPED`] placeholder describe entities that are not
    // present in this session (not exported to, or restored from, the
    // document) — they are filtered here so a reference to one reports
    // no match rather than binding an index that resolves nowhere.
    let mut entities: Vec<EntityKey> = current
        .iter()
        .filter_map(|&ordinal| journal.key_of(ordinal))
        .filter(|key| key.kind == reference.entity_kind && key.index != EntityKey::UNMAPPED)
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

// ─── Signature tier (RFC 0003, Stage 3) ─────────────────────────────────

/// Adjacency counts of one entity, structural only.
///
/// Faces: `primary` = boundary edge uses across all wires (a seam edge
/// used twice counts twice), `secondary` = wire count. Edges: `primary` =
/// face boundary uses, `secondary` = 0. Vertices: `primary` = incident
/// live edge uses (a closed edge contributes both ends), `secondary` = 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AdjacencySignature {
    /// The kind-specific primary count (see type docs).
    pub primary: u32,
    /// The kind-specific secondary count (see type docs).
    pub secondary: u32,
}

/// A typed geometric + adjacency signature of one entity (the inference
/// tier of RFC 0003).
///
/// Signatures are for *recovery* — imported models with no journal, or
/// journal gaps the caller knowingly accepts — never the primary path.
/// Everything about them resolves [`Provenance::Inferred`], and a
/// signature matching several entities is [`Resolution::Ambiguous`] —
/// never first-match.
///
/// Parameters are stored **quantized** (integer multiples of the capture
/// quantum, derived from the operation tolerance) so the signature is a
/// hashable value object; matching compares a candidate's raw parameters
/// against the stored multiples within one quantum, never by raw float
/// equality. Directions of orientation-free geometry (a cylinder or torus
/// axis, a circle normal) are sign-canonicalized before quantization so
/// the same surface captured twice signs identically; orientation-bearing
/// directions (a plane normal, a cone axis) are not. NURBS geometry
/// carries no analytic parameters — its signature is the type tag plus
/// adjacency, which usually resolves `Ambiguous` and is meant to.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct EntitySignature {
    /// The entity kind this signature describes.
    pub kind: EntityKind,
    /// Surface/curve type tag (`"plane"`, `"circle"`, …); vertices use
    /// `"vertex"`.
    pub type_tag: String,
    /// Quantized analytic parameters, in a fixed per-type order.
    pub params: Vec<i64>,
    /// The capture quantum as `f64` bits (bit-stable, hashable).
    pub quantum_bits: u64,
    /// Structural adjacency counts.
    pub adjacency: AdjacencySignature,
    /// For edges: the `(start, end)` vertex signatures, structural.
    pub endpoints: Option<Box<(Self, Self)>>,
}

impl EntitySignature {
    /// The quantum this signature was captured with.
    #[must_use]
    pub fn quantum(&self) -> f64 {
        f64::from_bits(self.quantum_bits)
    }

    /// The quantization an [`OperationContext`] implies (its linear
    /// tolerance) — the RFC-designated source; never raw float equality.
    ///
    /// [`OperationContext`]: remus_math::context::OperationContext
    #[must_use]
    pub fn context_quantum(context: &remus_math::context::OperationContext) -> f64 {
        context.tolerance.linear
    }

    /// Captures a face's signature.
    ///
    /// # Errors
    ///
    /// Returns a not-found error if the face or one of its wires is
    /// invalid.
    pub fn capture_face(
        topo: &Topology,
        face: crate::face::FaceId,
        quantum: f64,
    ) -> Result<Self, crate::TopologyError> {
        let face_data = topo.face(face)?;
        let mut edge_uses = 0u32;
        let mut wires = 0u32;
        for wire_id in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id)?;
            edge_uses =
                edge_uses.saturating_add(u32::try_from(wire.edges().len()).unwrap_or(u32::MAX));
            wires = wires.saturating_add(1);
        }
        Ok(Self {
            kind: EntityKind::Face,
            type_tag: face_data.surface().type_tag().to_owned(),
            params: quantize_all(&face_raw_params(face_data.surface(), quantum), quantum),
            quantum_bits: quantum.to_bits(),
            adjacency: AdjacencySignature {
                primary: edge_uses,
                secondary: wires,
            },
            endpoints: None,
        })
    }

    /// Captures an edge's signature, including its endpoint vertex
    /// signatures.
    ///
    /// # Errors
    ///
    /// Returns a not-found error if the edge or one of its vertices is
    /// invalid.
    pub fn capture_edge(
        topo: &Topology,
        edge: crate::edge::EdgeId,
        quantum: f64,
    ) -> Result<Self, crate::TopologyError> {
        let counts = adjacency_counts(topo);
        Self::capture_edge_with_counts(topo, edge, quantum, &counts)
    }

    fn capture_edge_with_counts(
        topo: &Topology,
        edge: crate::edge::EdgeId,
        quantum: f64,
        counts: &AdjacencyCounts,
    ) -> Result<Self, crate::TopologyError> {
        let edge_data = topo.edge(edge)?;
        let start = Self::capture_vertex_with_counts(topo, edge_data.start(), quantum, counts)?;
        let end = Self::capture_vertex_with_counts(topo, edge_data.end(), quantum, counts)?;
        Ok(Self {
            kind: EntityKind::Edge,
            type_tag: edge_data.curve().type_tag().to_owned(),
            params: quantize_all(&edge_raw_params(edge_data.curve(), quantum), quantum),
            quantum_bits: quantum.to_bits(),
            adjacency: AdjacencySignature {
                primary: counts.edge_face_uses(edge.index()),
                secondary: 0,
            },
            endpoints: Some(Box::new((start, end))),
        })
    }

    /// Captures a vertex's signature.
    ///
    /// # Errors
    ///
    /// Returns a not-found error if the vertex is invalid.
    pub fn capture_vertex(
        topo: &Topology,
        vertex: crate::vertex::VertexId,
        quantum: f64,
    ) -> Result<Self, crate::TopologyError> {
        let counts = adjacency_counts(topo);
        Self::capture_vertex_with_counts(topo, vertex, quantum, &counts)
    }

    fn capture_vertex_with_counts(
        topo: &Topology,
        vertex: crate::vertex::VertexId,
        quantum: f64,
        counts: &AdjacencyCounts,
    ) -> Result<Self, crate::TopologyError> {
        let vertex_data = topo.vertex(vertex)?;
        let p = vertex_data.point();
        Ok(Self {
            kind: EntityKind::Vertex,
            type_tag: "vertex".to_owned(),
            params: quantize_all(&[p.x(), p.y(), p.z()], quantum),
            quantum_bits: quantum.to_bits(),
            adjacency: AdjacencySignature {
                primary: counts.vertex_edge_uses(vertex.index()),
                secondary: 0,
            },
            endpoints: None,
        })
    }

    /// Whether a candidate's raw description matches this signature: same
    /// type tag and adjacency, every raw parameter within one quantum of
    /// the stored multiple.
    fn matches_raw(
        &self,
        type_tag: &str,
        raw_params: &[f64],
        adjacency: AdjacencySignature,
    ) -> bool {
        if self.type_tag != type_tag || self.adjacency != adjacency {
            return false;
        }
        if self.params.len() != raw_params.len() {
            return false;
        }
        let quantum = self.quantum();
        if !(quantum.is_finite() && quantum > 0.0) {
            return false;
        }
        self.params.iter().zip(raw_params).all(|(&stored, &raw)| {
            #[allow(clippy::cast_precision_loss)]
            let center = stored as f64 * quantum;
            (raw - center).abs() <= quantum
        })
    }
}

/// Quantizes raw parameters to integer multiples of the quantum.
fn quantize_all(raw: &[f64], quantum: f64) -> Vec<i64> {
    raw.iter().map(|&value| quantize(value, quantum)).collect()
}

fn quantize(value: f64, quantum: f64) -> i64 {
    if !(quantum.is_finite() && quantum > 0.0) {
        return i64::MAX;
    }
    let scaled = (value / quantum).round();
    if !scaled.is_finite() {
        return i64::MAX;
    }
    // Guarded: the clamp keeps the cast in range.
    #[allow(clippy::cast_possible_truncation)]
    if scaled >= 9.2e18 {
        i64::MAX
    } else if scaled <= -9.2e18 {
        i64::MIN
    } else {
        scaled as i64
    }
}

/// Sign-canonicalizes an orientation-free direction: flipped so its first
/// component larger than the quantum is positive.
fn canonical_direction(v: remus_math::vec::Vec3, quantum: f64) -> remus_math::vec::Vec3 {
    for component in [v.x(), v.y(), v.z()] {
        if component.abs() > quantum {
            return if component < 0.0 { -v } else { v };
        }
    }
    v
}

/// A face surface's raw signature parameters, in a fixed per-type order.
fn face_raw_params(surface: &crate::face::FaceSurface, quantum: f64) -> Vec<f64> {
    use crate::face::FaceSurface;
    match surface {
        FaceSurface::Plane { normal, d } => vec![normal.x(), normal.y(), normal.z(), *d],
        FaceSurface::Cylinder(c) => {
            let axis = canonical_direction(c.axis(), quantum);
            // Anchor the axis at its point nearest the world origin so
            // two parameterizations of one cylinder sign identically.
            let anchor = c.axis().normalize().map_or_else(
                |_| c.origin(),
                |unit| {
                    let o = c.origin();
                    let along = o
                        .x()
                        .mul_add(unit.x(), o.y().mul_add(unit.y(), o.z() * unit.z()));
                    remus_math::vec::Point3::new(
                        unit.x().mul_add(-along, o.x()),
                        unit.y().mul_add(-along, o.y()),
                        unit.z().mul_add(-along, o.z()),
                    )
                },
            );
            vec![
                axis.x(),
                axis.y(),
                axis.z(),
                anchor.x(),
                anchor.y(),
                anchor.z(),
                c.radius(),
            ]
        }
        FaceSurface::Cone(c) => {
            let apex = c.apex();
            vec![
                c.axis().x(),
                c.axis().y(),
                c.axis().z(),
                apex.x(),
                apex.y(),
                apex.z(),
                c.half_angle(),
            ]
        }
        FaceSurface::Sphere(s) => {
            let center = s.center();
            vec![center.x(), center.y(), center.z(), s.radius()]
        }
        FaceSurface::Torus(t) => {
            let axis = canonical_direction(t.z_axis(), quantum);
            let center = t.center();
            vec![
                axis.x(),
                axis.y(),
                axis.z(),
                center.x(),
                center.y(),
                center.z(),
                t.major_radius(),
                t.minor_radius(),
            ]
        }
        FaceSurface::Nurbs(_) => Vec::new(),
    }
}

/// An edge curve's raw signature parameters, in a fixed per-type order.
///
/// `Line` carries no parameters on purpose: a line edge's geometry is its
/// endpoints, and the endpoint vertex signatures carry it. NURBS and the
/// open conics are tag-plus-endpoints only.
fn edge_raw_params(curve: &crate::edge::EdgeCurve, quantum: f64) -> Vec<f64> {
    use crate::edge::EdgeCurve;
    match curve {
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => Vec::new(),
        EdgeCurve::Circle(c) => {
            let normal = canonical_direction(c.normal(), quantum);
            let center = c.center();
            vec![
                normal.x(),
                normal.y(),
                normal.z(),
                center.x(),
                center.y(),
                center.z(),
                c.radius(),
            ]
        }
        EdgeCurve::Ellipse(e) => {
            let normal = canonical_direction(e.normal(), quantum);
            let center = e.center();
            vec![
                normal.x(),
                normal.y(),
                normal.z(),
                center.x(),
                center.y(),
                center.z(),
                e.semi_major(),
                e.semi_minor(),
            ]
        }
    }
}

/// Global structural adjacency counts, built by one model walk.
struct AdjacencyCounts {
    edge_face_uses: std::collections::HashMap<usize, u32>,
    vertex_edge_uses: std::collections::HashMap<usize, u32>,
}

impl AdjacencyCounts {
    fn edge_face_uses(&self, edge_index: usize) -> u32 {
        self.edge_face_uses.get(&edge_index).copied().unwrap_or(0)
    }

    fn vertex_edge_uses(&self, vertex_index: usize) -> u32 {
        self.vertex_edge_uses
            .get(&vertex_index)
            .copied()
            .unwrap_or(0)
    }
}

fn adjacency_counts(topo: &Topology) -> AdjacencyCounts {
    let mut edge_face_uses: std::collections::HashMap<usize, u32> =
        std::collections::HashMap::new();
    for (_, face) in topo.faces().iter() {
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            if let Ok(wire) = topo.wire(wire_id) {
                for oriented in wire.edges() {
                    *edge_face_uses.entry(oriented.edge().index()).or_insert(0) += 1;
                }
            }
        }
    }
    let mut vertex_edge_uses: std::collections::HashMap<usize, u32> =
        std::collections::HashMap::new();
    for (_, edge) in topo.edges().iter() {
        *vertex_edge_uses.entry(edge.start().index()).or_insert(0) += 1;
        *vertex_edge_uses.entry(edge.end().index()).or_insert(0) += 1;
    }
    AdjacencyCounts {
        edge_face_uses,
        vertex_edge_uses,
    }
}

/// Resolves a signature against the current model (no journal chase:
/// signatures describe the present, not history).
fn resolve_signature(topo: &Topology, signature: &EntitySignature) -> Vec<EntityKey> {
    let counts = adjacency_counts(topo);
    let mut matches = Vec::new();
    match signature.kind {
        EntityKind::Face => {
            for (id, face) in topo.faces().iter() {
                let mut edge_uses = 0u32;
                let mut wires = 0u32;
                for wire_id in
                    std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
                {
                    if let Ok(wire) = topo.wire(wire_id) {
                        edge_uses = edge_uses
                            .saturating_add(u32::try_from(wire.edges().len()).unwrap_or(u32::MAX));
                    }
                    wires = wires.saturating_add(1);
                }
                let adjacency = AdjacencySignature {
                    primary: edge_uses,
                    secondary: wires,
                };
                let raw = face_raw_params(face.surface(), signature.quantum());
                if signature.matches_raw(face.surface().type_tag(), &raw, adjacency) {
                    matches.push(EntityKey::face(id.index()));
                }
            }
        }
        EntityKind::Edge => {
            for (id, edge) in topo.edges().iter() {
                let adjacency = AdjacencySignature {
                    primary: counts.edge_face_uses(id.index()),
                    secondary: 0,
                };
                let raw = edge_raw_params(edge.curve(), signature.quantum());
                if !signature.matches_raw(edge.curve().type_tag(), &raw, adjacency) {
                    continue;
                }
                // Endpoint signatures are structural discriminators: both
                // must match in order (start, end).
                let endpoints_match = signature.endpoints.as_ref().is_none_or(|expected| {
                    vertex_matches(topo, &counts, edge.start(), &expected.0)
                        && vertex_matches(topo, &counts, edge.end(), &expected.1)
                });
                if endpoints_match {
                    matches.push(EntityKey::edge(id.index()));
                }
            }
        }
        EntityKind::Vertex => {
            for (id, _) in topo.vertices().iter() {
                if vertex_matches(topo, &counts, id, signature) {
                    matches.push(EntityKey::vertex(id.index()));
                }
            }
        }
    }
    matches.sort_unstable();
    matches
}

fn vertex_matches(
    topo: &Topology,
    counts: &AdjacencyCounts,
    vertex: crate::vertex::VertexId,
    signature: &EntitySignature,
) -> bool {
    let Ok(vertex_data) = topo.vertex(vertex) else {
        return false;
    };
    let p = vertex_data.point();
    let adjacency = AdjacencySignature {
        primary: counts.vertex_edge_uses(vertex.index()),
        secondary: 0,
    };
    signature.matches_raw("vertex", &[p.x(), p.y(), p.z()], adjacency)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

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
            remus_math::vec::Point3::new(0.0, 0.0, 0.0),
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

    // ── Signature tier (Stage 3) ────────────────────────────────────────

    const QUANTUM: f64 = 1e-7;

    fn topo_with_vertex(x: f64) -> (Topology, crate::VertexId) {
        let mut topo = Topology::new();
        let v = topo.add_vertex(crate::vertex::Vertex::new(
            remus_math::vec::Point3::new(x, 2.0, 3.0),
            1e-7,
        ));
        (topo, v)
    }

    #[test]
    fn vertex_signature_binds_within_one_quantum_across_topologies() {
        let (topo_a, v) = topo_with_vertex(1.0);
        let signature = EntitySignature::capture_vertex(&topo_a, v, QUANTUM).unwrap();

        // A different topology whose vertex sits within one quantum.
        let (topo_b, v_b) = topo_with_vertex(1.0 + 0.4 * QUANTUM);
        let r = resolve(&topo_b, &PersistentRef::signature(signature.clone()));
        assert_eq!(
            r,
            Resolution::Bound {
                entity: EntityKey::vertex(v_b.index()),
                provenance: Provenance::Inferred
            },
            "signatures are tolerance-aware and always inferred"
        );

        // Beyond the window: no match, never a nearest-pick.
        let (topo_c, _) = topo_with_vertex(1.0 + 3.0 * QUANTUM);
        let r = resolve(&topo_c, &PersistentRef::signature(signature));
        assert!(matches!(r, Resolution::NoMatch { .. }), "{r:?}");
    }

    #[test]
    fn identical_entities_are_ambiguous_never_first_match() {
        let mut topo = Topology::new();
        let p = remus_math::vec::Point3::new(1.0, 1.0, 1.0);
        let a = topo.add_vertex(crate::vertex::Vertex::new(p, 1e-7));
        let b = topo.add_vertex(crate::vertex::Vertex::new(p, 1e-7));
        let signature = EntitySignature::capture_vertex(&topo, a, QUANTUM).unwrap();

        let r = resolve(&topo, &PersistentRef::signature(signature));
        let Resolution::Ambiguous { candidates, .. } = &r else {
            panic!("two identical vertices must be ambiguous: {r:?}");
        };
        assert_eq!(
            candidates,
            &vec![EntityKey::vertex(a.index()), EntityKey::vertex(b.index())]
        );
        let err = r.into_entities().unwrap_err();
        assert!(matches!(
            err,
            TopologyError::RefAmbiguous { candidates: 2, .. }
        ));
    }

    #[test]
    fn edge_signatures_discriminate_by_endpoints() {
        use crate::edge::{Edge, EdgeCurve};

        // Two parallel line edges: identical curve type (Line carries no
        // parameters), identical adjacency — only the endpoint vertex
        // signatures tell them apart.
        let mut topo = Topology::new();
        let mk = |topo: &mut Topology, y: f64| {
            let a = topo.add_vertex(crate::vertex::Vertex::new(
                remus_math::vec::Point3::new(0.0, y, 0.0),
                1e-7,
            ));
            let b = topo.add_vertex(crate::vertex::Vertex::new(
                remus_math::vec::Point3::new(1.0, y, 0.0),
                1e-7,
            ));
            topo.add_edge(Edge::new(a, b, EdgeCurve::Line))
        };
        let e0 = mk(&mut topo, 0.0);
        let e1 = mk(&mut topo, 5.0);

        for (edge, other) in [(e0, e1), (e1, e0)] {
            let signature = EntitySignature::capture_edge(&topo, edge, QUANTUM).unwrap();
            let r = resolve(&topo, &PersistentRef::signature(signature));
            assert_eq!(
                r,
                Resolution::Bound {
                    entity: EntityKey::edge(edge.index()),
                    provenance: Provenance::Inferred
                },
                "endpoints must separate parallel lines (not {other:?})"
            );
        }
    }

    #[test]
    fn signature_kind_must_match_the_reference_kind() {
        let (topo, v) = topo_with_vertex(1.0);
        let signature = EntitySignature::capture_vertex(&topo, v, QUANTUM).unwrap();
        let mut reference = PersistentRef::signature(signature);
        reference.entity_kind = EntityKind::Face;
        let r = resolve(&topo, &reference);
        assert!(
            matches!(r, Resolution::NoMatch { ref reason } if reason.contains("describes a vertex")),
            "{r:?}"
        );
    }

    #[test]
    fn quantization_is_total_and_poisoned_on_nonsense() {
        assert_eq!(quantize(1.0, 0.0), i64::MAX, "zero quantum cannot match");
        assert_eq!(quantize(f64::NAN, 1e-7), i64::MAX);
        assert_eq!(quantize(f64::INFINITY, 1e-7), i64::MAX);
        assert_eq!(quantize(-1e30, 1e-9), i64::MIN, "clamped, not wrapped");
        assert_eq!(quantize(2.5e-7, 1e-7), 3, "round to nearest multiple");
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
