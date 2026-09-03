//! Journal ingestion for modeling operations (RFC 0003, Stage 1).
//!
//! The journal (`remus_topology::journal`) is the persistent history
//! spine; this module is where operations feed it:
//!
//! - [`boolean_journaled`] runs a GFA boolean and journals its full
//!   construction-derived vertex/edge/face evolution (Issue 12) as one
//!   entry.
//! - [`record_face_evolution`] journals any operation that produces an
//!   [`EvolutionMap`] (v2 blends via
//!   [`fillet_with_evolution`](crate::blend_ops::fillet_with_evolution),
//!   pattern and boolean face maps) as a faces-only entry.
//! - [`record_barrier_over_solid`] journals an explicit barrier for an
//!   operation that produces no evolution records: every entity of the
//!   result solid is unresolved across it, and a resolver fails closed
//!   naming the operation instead of pretending continuity.
//!
//! The calling pattern is the same for all three: open the operation with
//! [`Topology::journal_begin`] **before** running it (that is where
//! unjournaled-mutation gaps are detected), run the operation, then record.
//! A faces-only entry claims nothing about edges and vertices — absent
//! claims are gaps, never implicit preservation.

use remus_algo::bop::BooleanOp;
use remus_algo::gfa::{self, EdgeEvent, EntityEvolution, VertexEvent};
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft, OpId, PendingOp};
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::evolution::EvolutionMap;

/// A journaled boolean's result: the solid and the journal entry that
/// records its history.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JournaledBoolean {
    /// The result solid.
    pub solid: SolidId,
    /// The journal entry recording the operation's evolution.
    pub op: OpId,
}

/// Stable journal kind name for one boolean operation.
#[must_use]
pub fn boolean_kind(op: BooleanOp) -> &'static str {
    match op {
        BooleanOp::Fuse => "boolean_fuse",
        BooleanOp::Cut => "boolean_cut",
        BooleanOp::Intersect => "boolean_intersect",
    }
}

/// Every face, edge, and vertex of a solid as journal entity keys.
///
/// # Errors
///
/// Returns [`OperationsError`] if the solid's topology tree contains an
/// invalid handle.
pub fn solid_entity_keys(
    topo: &Topology,
    solid: SolidId,
) -> Result<Vec<EntityKey>, OperationsError> {
    let mut keys = Vec::new();
    keys.extend(
        solid_faces(topo, solid)?
            .into_iter()
            .map(|id| EntityKey::face(id.index())),
    );
    keys.extend(
        solid_edges(topo, solid)?
            .into_iter()
            .map(|id| EntityKey::edge(id.index())),
    );
    keys.extend(
        solid_vertices(topo, solid)?
            .into_iter()
            .map(|id| EntityKey::vertex(id.index())),
    );
    Ok(keys)
}

/// Opens a journaled operation, capturing the pre-operation half of its
/// scope.
///
/// The scope half captured here is every entity of the listed solids,
/// walked **before** the operation runs (they may be retired by the time
/// the entry is recorded). Pre-operation entities the entry then makes no
/// claim about are severed — an operand entity the operation consumed
/// without a record fails closed instead of resolving to a retired
/// handle.
///
/// # Errors
///
/// Returns [`OperationsError`] if a solid's topology tree contains an
/// invalid handle; nothing is journaled (the begin gap-check has already
/// run, which is harmless).
pub fn begin_scoped(
    topo: &mut Topology,
    kind: &str,
    solids: &[SolidId],
) -> Result<PendingOp, OperationsError> {
    let mut pending = topo.journal_begin(kind);
    for &solid in solids {
        pending.add_scope(solid_entity_keys(topo, solid)?);
    }
    Ok(pending)
}

/// Runs a GFA boolean and journals its construction-derived vertex, edge,
/// and face history as one evolution entry.
///
/// This is the exact GFA path only ([`gfa::boolean_with_entity_evolution`],
/// Issue 12): there is no approximate fallback here, because a mesh
/// fallback has no construction records to journal — a caller accepting
/// approximate results journals that operation as a barrier instead. Like
/// the underlying entry point, identical operands are not special-cased.
///
/// # Errors
///
/// Returns [`OperationsError`] if the boolean fails (nothing is recorded;
/// the failed operation's partial mutations surface as a global barrier at
/// the next [`Topology::journal_begin`]) or if the evolution record is
/// malformed (duplicate claims — a kernel defect, refused whole).
pub fn boolean_journaled(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<JournaledBoolean, OperationsError> {
    // Pre-operation scope: both operands' entities, so an operand entity
    // the boolean consumed without a record (the GFA does not record face
    // deletions) severs instead of resolving to a retired handle.
    let pending = begin_scoped(topo, boolean_kind(op), &[solid_a, solid_b])?;
    let (solid, evolution) = gfa::boolean_with_entity_evolution(topo, op, solid_a, solid_b)?;
    let draft = draft_from_entity_evolution(&evolution);
    let op = topo.journal_record_evolution(pending, draft)?;
    Ok(JournaledBoolean { solid, op })
}

/// Converts an Issue-12 [`EntityEvolution`] into a journal draft.
///
/// Event mapping, claim for claim:
/// - face `(out, Some(src))` → `Modified` (the result face is a piece of
///   the input face);
/// - face `(out, None)` → `Generated` with no named sources (the GFA's
///   construction-derived claim that the face was synthesised);
/// - edge events map directly (`Generated` names the generating faces the
///   store could translate);
/// - vertex `Created` → `Generated` with no named sources (existence is
///   construction-derived; the generating interference is not yet
///   recorded).
#[must_use]
pub fn draft_from_entity_evolution(evolution: &EntityEvolution) -> EvolutionDraft {
    let mut draft = EvolutionDraft::construction();
    for &(out, src) in &evolution.faces {
        let event = src.map_or(
            EventDraft::Generated {
                sources: Vec::new(),
            },
            |src| EventDraft::Modified {
                from: EntityKey::face(src),
            },
        );
        draft.push(EntityKey::face(out), event);
    }
    for (out, event) in &evolution.edges {
        let event = match event {
            EdgeEvent::Preserved(src) => EventDraft::Preserved {
                from: EntityKey::edge(*src),
            },
            EdgeEvent::Modified(src) => EventDraft::Modified {
                from: EntityKey::edge(*src),
            },
            EdgeEvent::Generated { face_a, face_b } => EventDraft::Generated {
                sources: [*face_a, *face_b]
                    .into_iter()
                    .flatten()
                    .map(EntityKey::face)
                    .collect(),
            },
            EdgeEvent::Unresolved => EventDraft::Unresolved {
                candidates: Vec::new(),
            },
        };
        draft.push(EntityKey::edge(*out), event);
    }
    for (out, event) in &evolution.vertices {
        let event = match event {
            VertexEvent::Preserved(src) => EventDraft::Preserved {
                from: EntityKey::vertex(*src),
            },
            VertexEvent::Created => EventDraft::Generated {
                sources: Vec::new(),
            },
        };
        draft.push(EntityKey::vertex(*out), event);
    }
    draft
}

/// Journals an operation's face evolution from its [`EvolutionMap`].
///
/// This is the generic ingestion for every operation that reports face
/// evolution — v2 blends, pattern instances, boolean face maps. The entry
/// is faces-only: it claims nothing about edges and vertices, so
/// references to them do not resolve across this operation (fail closed,
/// never implicit preservation).
///
/// Mapping, claim for claim:
/// - an output under exactly one input's `modified` list → `Modified`;
/// - an output under several inputs' `modified` lists (a same-domain
///   merge) → `Merged` naming all of them;
/// - `generated` outputs → `Generated` naming their source inputs;
/// - `deleted` inputs → `Deleted`;
/// - `unresolved` outputs → `Unresolved` with their candidate inputs.
///
/// The entry's origin mirrors [`EvolutionMap::origin`]: a geometry-derived
/// map journals as inference, and a resolver must surface that to callers.
///
/// `result_solids` declares the post-operation half of the entry's scope:
/// every entity of those solids. With the pre-operation half captured by
/// [`begin_scoped`], the entry's scope covers everything the operation may
/// have touched, so its edges and vertices sever honestly while other
/// solids' entities carry through.
///
/// # Errors
///
/// Returns [`OperationsError`] if the map makes conflicting claims about
/// one face (e.g. an output listed as both modified and generated), or if
/// a result solid's topology tree contains an invalid handle; nothing is
/// recorded.
pub fn record_face_evolution(
    topo: &mut Topology,
    pending: PendingOp,
    map: &EvolutionMap,
    result_solids: &[SolidId],
) -> Result<OpId, OperationsError> {
    use std::collections::BTreeMap;

    let mut draft = if map.origin.is_exact() {
        EvolutionDraft::construction()
    } else {
        EvolutionDraft::geometry()
    };
    for &solid in result_solids {
        draft.add_scope(solid_entity_keys(topo, solid)?);
    }

    // Group by output so a same-domain merge (one output claimed by several
    // inputs) becomes one `Merged` event rather than duplicate claims.
    let mut modified_by_output: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (&input, outputs) in &map.modified {
        for &output in outputs {
            modified_by_output.entry(output).or_default().push(input);
        }
    }
    for (output, mut inputs) in modified_by_output {
        inputs.sort_unstable();
        let event = match inputs.as_slice() {
            [single] => EventDraft::Modified {
                from: EntityKey::face(*single),
            },
            _ => EventDraft::Merged {
                from: inputs.into_iter().map(EntityKey::face).collect(),
            },
        };
        draft.push(EntityKey::face(output), event);
    }

    let mut generated_by_output: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (&input, outputs) in &map.generated {
        for &output in outputs {
            generated_by_output.entry(output).or_default().push(input);
        }
    }
    for (output, mut inputs) in generated_by_output {
        inputs.sort_unstable();
        draft.push(
            EntityKey::face(output),
            EventDraft::Generated {
                sources: inputs.into_iter().map(EntityKey::face).collect(),
            },
        );
    }

    let mut deleted: Vec<usize> = map.deleted.iter().copied().collect();
    deleted.sort_unstable();
    for input in deleted {
        draft.push(EntityKey::face(input), EventDraft::Deleted);
    }

    for (&output, candidates) in &map.unresolved {
        draft.push(
            EntityKey::face(output),
            EventDraft::Unresolved {
                candidates: candidates.iter().map(|&c| EntityKey::face(c)).collect(),
            },
        );
    }

    Ok(topo.journal_record_evolution(pending, draft)?)
}

/// A journaled blend's result: the blend outcome, its journal entry,
/// and the face-evolution map the entry recorded.
pub struct JournaledBlend {
    /// The blend outcome (result solid, per-edge failures, partiality).
    pub result: remus_blend::BlendResult,
    /// The journal entry recording the blend's face evolution.
    pub op: OpId,
    /// The recorded face-evolution map.
    pub map: EvolutionMap,
}

/// Runs a v2 fillet and journals its face evolution as one entry
/// (kind `fillet`).
///
/// The entry scope is the solid's full pre- and post-operation entity
/// set, so edge and vertex references across the fillet sever honestly
/// while face claims carry.
///
/// The whole call is transactional: if the blend fails, or the journal
/// refuses the recorded draft, the topology AND the journal roll back to
/// their pre-call state — a `fillet_journaled` error never leaves a
/// half-journaled blend behind.
///
/// # Errors
///
/// Returns [`OperationsError`] if the fillet or the recording fails.
pub fn fillet_journaled(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[remus_topology::EdgeId],
    radius: f64,
) -> Result<JournaledBlend, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let pending = begin_scoped(topo, "fillet", &[solid])?;
        let (result, map) = crate::blend_ops::fillet_with_evolution(topo, solid, edges, radius)?;
        let op = record_face_evolution(topo, pending, &map, &[result.solid])?;
        Ok(JournaledBlend { result, op, map })
    })
}

/// Runs a v2 chamfer and journals its face evolution as one entry
/// (kind `chamfer`); see [`fillet_journaled`].
///
/// # Errors
///
/// Returns [`OperationsError`] if the chamfer or the recording fails.
pub fn chamfer_journaled(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[remus_topology::EdgeId],
    d1: f64,
    d2: f64,
) -> Result<JournaledBlend, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let pending = begin_scoped(topo, "chamfer", &[solid])?;
        let (result, map) = crate::blend_ops::chamfer_with_evolution(topo, solid, edges, d1, d2)?;
        let op = record_face_evolution(topo, pending, &map, &[result.solid])?;
        Ok(JournaledBlend { result, op, map })
    })
}

/// A journaled pattern's result.
#[derive(Debug)]
pub struct JournaledPattern {
    /// The compound of pattern instances.
    pub compound: remus_topology::CompoundId,
    /// The journal entry recording per-instance face provenance.
    pub op: OpId,
    /// The recorded face-evolution map.
    pub map: EvolutionMap,
}

/// Runs a linear pattern and journals its construction-derived
/// per-instance face provenance as one entry (kind `linear_pattern`),
/// scoped over the original solid and every instance.
///
/// # Errors
///
/// Returns [`OperationsError`] if the pattern or the recording fails.
pub fn linear_pattern_journaled(
    topo: &mut Topology,
    solid: SolidId,
    direction: remus_math::vec::Vec3,
    spacing: f64,
    count: usize,
) -> Result<JournaledPattern, OperationsError> {
    let pending = begin_scoped(topo, "linear_pattern", &[solid])?;
    let (compound, map) =
        crate::pattern::linear_pattern_with_evolution(topo, solid, direction, spacing, count)?;
    let members = topo.compound(compound)?.solids().to_vec();
    let op = record_face_evolution(topo, pending, &map, &members)?;
    Ok(JournaledPattern { compound, op, map })
}

/// A journaled single-solid operation's result.
#[derive(Debug)]
pub struct JournaledSolidOp {
    /// The result solid.
    pub solid: SolidId,
    /// The journal entry recording the operation's face evolution.
    pub op: OpId,
    /// The recorded face-evolution map.
    pub map: EvolutionMap,
}

/// Runs an offset and journals its one-to-one, construction-derived face
/// evolution as one entry (kind `offset`).
///
/// The whole call is transactional: a failed offset, postcondition, or
/// journal recording restores both topology and history.
///
/// # Errors
///
/// Returns [`OperationsError`] if the offset or recording fails.
pub fn offset_journaled(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<JournaledSolidOp, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let pending = begin_scoped(topo, "offset", &[solid])?;
        let (result, map) =
            crate::offset_v2::offset_solid_v2_with_evolution(topo, solid, distance)?;
        let op = record_face_evolution(topo, pending, &map, &[result])?;
        Ok(JournaledSolidOp {
            solid: result,
            op,
            map,
        })
    })
}

/// Runs a draft and journals its construction-derived face evolution as
/// one entry (kind `draft`).
///
/// # Errors
///
/// Returns [`OperationsError`] if the draft or the recording fails.
pub fn draft_journaled(
    topo: &mut Topology,
    solid: SolidId,
    draft_faces: &[remus_topology::FaceId],
    pull_direction: remus_math::vec::Vec3,
    neutral_point: remus_math::vec::Point3,
    angle_radians: f64,
) -> Result<JournaledSolidOp, OperationsError> {
    let pending = begin_scoped(topo, "draft", &[solid])?;
    let (result, map) = crate::draft::draft_with_evolution(
        topo,
        solid,
        draft_faces,
        pull_direction,
        neutral_point,
        angle_radians,
    )?;
    let op = record_face_evolution(topo, pending, &map, &[result])?;
    Ok(JournaledSolidOp {
        solid: result,
        op,
        map,
    })
}

/// Runs a defeature and journals its construction-derived face evolution
/// as one entry (kind `defeature`).
///
/// # Errors
///
/// Returns [`OperationsError`] if the defeature or the recording fails.
pub fn defeature_journaled(
    topo: &mut Topology,
    solid: SolidId,
    faces_to_remove: &[remus_topology::FaceId],
) -> Result<JournaledSolidOp, OperationsError> {
    let pending = begin_scoped(topo, "defeature", &[solid])?;
    let (result, map) = crate::defeature::defeature_with_evolution(topo, solid, faces_to_remove)?;
    let op = record_face_evolution(topo, pending, &map, &[result])?;
    Ok(JournaledSolidOp {
        solid: result,
        op,
        map,
    })
}

/// Runs a shell (hollow) and journals its construction-derived face
/// evolution as one entry (kind `shell`).
///
/// # Errors
///
/// Returns [`OperationsError`] if the shell or the recording fails.
pub fn shell_journaled(
    topo: &mut Topology,
    solid: SolidId,
    thickness: f64,
    open_faces: &[remus_topology::FaceId],
) -> Result<JournaledSolidOp, OperationsError> {
    let pending = begin_scoped(topo, "shell", &[solid])?;
    let (result, map) = crate::shell_op::shell_with_evolution(topo, solid, thickness, open_faces)?;
    let op = record_face_evolution(topo, pending, &map, &[result])?;
    Ok(JournaledSolidOp {
        solid: result,
        op,
        map,
    })
}

/// A journaled split's result.
#[derive(Debug)]
pub struct JournaledSplit {
    /// The two halves.
    pub result: crate::split::SplitResult,
    /// The journal entry recording both halves' face evolution.
    pub op: OpId,
    /// The positive half's face-evolution map.
    pub positive_map: EvolutionMap,
    /// The negative half's face-evolution map.
    pub negative_map: EvolutionMap,
}

/// Runs a plane split and journals both halves' construction-derived face
/// evolution as one entry (kind `split`), scoped over the input and both
/// halves.
///
/// # Errors
///
/// Returns [`OperationsError`] if the split or the recording fails.
pub fn split_journaled(
    topo: &mut Topology,
    solid: SolidId,
    plane_point: remus_math::vec::Point3,
    plane_normal: remus_math::vec::Vec3,
) -> Result<JournaledSplit, OperationsError> {
    let pending = begin_scoped(topo, "split", &[solid])?;
    let (result, evo) = crate::split::split_with_evolution(topo, solid, plane_point, plane_normal)?;
    let mut combined = EvolutionMap::exact();
    for map in [&evo.positive, &evo.negative] {
        for (&input, outputs) in &map.modified {
            for &output in outputs {
                combined.add_modified(input, output);
            }
        }
        for (&output, candidates) in &map.unresolved {
            combined.add_unresolved(output, candidates.clone());
        }
    }
    let op = record_face_evolution(
        topo,
        pending,
        &combined,
        &[result.positive, result.negative],
    )?;
    Ok(JournaledSplit {
        result,
        op,
        positive_map: evo.positive,
        negative_map: evo.negative,
    })
}

/// Journals an explicit barrier over every entity of `solid`.
///
/// This is the honest entry for an operation that produces no evolution
/// records (direct edits are the stability matrix's remaining declared gap;
/// offset, draft, defeature, split, and shell journal real evolution via
/// their `*_journaled` wrappers): the result solid's faces, edges, and
/// vertices are all unresolved across it, and a resolver chasing a reference
/// through this entry fails closed naming the operation. Coverage grows
/// operation by operation by replacing barriers with real evolution.
///
/// # Errors
///
/// Returns [`OperationsError`] if `solid` or its topology tree contains an
/// invalid handle; nothing is recorded.
pub fn record_barrier_over_solid(
    topo: &mut Topology,
    pending: PendingOp,
    solid: SolidId,
) -> Result<OpId, OperationsError> {
    let mut affected = Vec::new();
    affected.extend(
        solid_faces(topo, solid)?
            .into_iter()
            .map(|id| EntityKey::face(id.index())),
    );
    affected.extend(
        solid_edges(topo, solid)?
            .into_iter()
            .map(|id| EntityKey::edge(id.index())),
    );
    affected.extend(
        solid_vertices(topo, solid)?
            .into_iter()
            .map(|id| EntityKey::vertex(id.index())),
    );
    Ok(topo.journal_record_barrier(pending, affected))
}
