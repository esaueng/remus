//! Result-aware edge and vertex evolution induced from an exact face map
//! (bridge row B18).
//!
//! A face-only [`EvolutionMap`] claims nothing about the boundary: edge and
//! vertex references sever across such an operation even when every face
//! identity is construction-derived. This module closes that gap for
//! operations whose face map is exact and identity-carrying, without
//! re-running or changing the operation's geometry.
//!
//! # Induction by exact incidence
//!
//! Each result edge is characterised by the multiset of result faces that
//! use it (a seam counts its face twice), and each result vertex by the set
//! of result faces around it. Mapping those faces back through the exact
//! `modified` claims gives a source incidence key, which is compared with
//! the keys of the source solid's own edges and vertices, captured before
//! the operation ran:
//!
//! - exactly one source entity with that key: the result entity is a
//!   [`BoundaryEvent::Modified`] piece of it (several result entities naming
//!   one source is a split, and every piece is attributed);
//! - no source entity with that key: the faces meet where they did not
//!   before, so the entity is [`BoundaryEvent::Generated`] from them;
//! - several source entities with that key: picking one would guess, so the
//!   entity is [`BoundaryEvent::Unresolved`] with
//!   [`UnresolvedReason::AmbiguousIncidence`] and the candidates named;
//! - an incident face without exactly one `modified` source: the key does
//!   not exist, so the entity is unresolved with
//!   [`UnresolvedReason::UnmappedIncidentFace`].
//!
//! No coordinate, tolerance or geometric proximity is consulted: the claims
//! are combinatorial consequences of the operation's construction face
//! identities and the exact result topology. They carry the face map's
//! [`EvolutionOrigin`](crate::evolution::EvolutionOrigin).
//!
//! # Result-aware completeness
//!
//! [`EntityCompletenessReport`] compares every claim against the actual
//! result entity sets, per kind, so an omitted record (a result entity no
//! claim mentions) or a phantom one (a claim about an entity that is not in
//! the result) is reported instead of hidden by a count.

use std::collections::{BTreeMap, BTreeSet};

use remus_topology::Topology;
use remus_topology::arena::Id;
use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityKey, EventDraft};
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::evolution::{CompletenessReport, EvolutionMap};

/// Why a result entity could not be attributed.
///
/// Every unresolved record carries one of these, so a consumer can tell a
/// genuinely ambiguous correspondence from a missing input record without
/// parsing a message.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum UnresolvedReason {
    /// Several source entities have the result entity's exact face
    /// incidence (for example a torus's two seams, or two half-cylinders
    /// joined along two lines); binding one would be a guess.
    AmbiguousIncidence,
    /// A face incident to the result entity has no single `modified`
    /// source, so the entity's source incidence cannot be formed.
    UnmappedIncidentFace,
    /// The operation's own face map recorded this result face as
    /// unresolved.
    UnresolvedFaceOrigin,
}

impl UnresolvedReason {
    /// Stable snake-case name, used in the JSON encodings.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::AmbiguousIncidence => "ambiguous_incidence",
            Self::UnmappedIncidentFace => "unmapped_incident_face",
            Self::UnresolvedFaceOrigin => "unresolved_face_origin",
        }
    }
}

/// One result edge's or vertex's claim.
///
/// Source indices name entities of the same kind as the subject, except
/// [`BoundaryEvent::Generated`], which names the source faces the new
/// entity was built between.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BoundaryEvent {
    /// The result entity is a modified piece of one source entity. Several
    /// result entities naming one source record a split.
    Modified {
        /// Source entity index.
        from: usize,
    },
    /// The result entity is the merge of several source entities.
    Merged {
        /// Source entity indices, sorted.
        from: Vec<usize>,
    },
    /// The result entity is new, where the named source faces now meet.
    Generated {
        /// Source face indices, sorted and unique.
        faces: Vec<usize>,
    },
    /// The result entity's origin could not be established.
    Unresolved {
        /// Source entities that could not be told apart, sorted.
        candidates: Vec<usize>,
        /// Why the claim was refused.
        reason: UnresolvedReason,
    },
}

/// Edge and vertex claims for one operation, keyed by result entity index.
///
/// Keying by result entity makes every claim one record: a split source
/// appears under each of its pieces, and a merge is one record naming all
/// of its sources.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BoundaryEvolution {
    /// Result edge index to its claim.
    pub edges: BTreeMap<usize, BoundaryEvent>,
    /// Result vertex index to its claim.
    pub vertices: BTreeMap<usize, BoundaryEvent>,
}

impl BoundaryEvolution {
    /// Every unresolved record, as `(kind, result index, candidates,
    /// reason)`, edges before vertices, each in index order.
    #[must_use]
    pub fn unresolved(
        &self,
    ) -> Vec<(
        remus_topology::journal::EntityKind,
        usize,
        &[usize],
        UnresolvedReason,
    )> {
        use remus_topology::journal::EntityKind;
        let mut out = Vec::new();
        for (kind, claims) in [
            (EntityKind::Edge, &self.edges),
            (EntityKind::Vertex, &self.vertices),
        ] {
            for (&index, event) in claims {
                if let BoundaryEvent::Unresolved { candidates, reason } = event {
                    out.push((kind, index, candidates.as_slice(), *reason));
                }
            }
        }
        out
    }

    /// The journal events for these claims, over arena keys.
    #[must_use]
    pub fn journal_events(&self) -> Vec<(EntityKey, EventDraft)> {
        let mut events = Vec::with_capacity(self.edges.len() + self.vertices.len());
        for (claims, key) in [
            (&self.edges, EntityKey::edge as fn(usize) -> EntityKey),
            (&self.vertices, EntityKey::vertex as fn(usize) -> EntityKey),
        ] {
            for (&subject, event) in claims {
                let draft = match event {
                    BoundaryEvent::Modified { from } => EventDraft::Modified { from: key(*from) },
                    BoundaryEvent::Merged { from } => EventDraft::Merged {
                        from: from.iter().copied().map(key).collect(),
                    },
                    BoundaryEvent::Generated { faces } => EventDraft::Generated {
                        sources: faces.iter().copied().map(EntityKey::face).collect(),
                    },
                    BoundaryEvent::Unresolved { candidates, .. } => EventDraft::Unresolved {
                        candidates: candidates.iter().copied().map(key).collect(),
                    },
                };
                events.push((key(subject), draft));
            }
        }
        events
    }
}

/// Result-aware completeness of a face map plus its boundary claims.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EntityCompletenessReport {
    /// Face claims against the result faces.
    pub faces: CompletenessReport,
    /// Edge claims against the result edges.
    pub edges: CompletenessReport,
    /// Vertex claims against the result vertices.
    pub vertices: CompletenessReport,
}

impl EntityCompletenessReport {
    /// Every result face, edge and vertex has a claim (explicit unresolved
    /// records count), and no claim names an entity outside the result.
    #[must_use]
    pub fn is_accounted(&self) -> bool {
        self.faces.is_accounted() && self.edges.is_accounted() && self.vertices.is_accounted()
    }

    /// Accounted, and no result entity is unresolved.
    #[must_use]
    pub fn is_resolved(&self) -> bool {
        self.faces.is_resolved() && self.edges.is_resolved() && self.vertices.is_resolved()
    }
}

fn claims_completeness(
    claims: &BTreeMap<usize, BoundaryEvent>,
    result: impl IntoIterator<Item = usize>,
) -> CompletenessReport {
    let result: BTreeSet<usize> = result.into_iter().collect();
    let attributed: BTreeSet<usize> = claims.keys().copied().collect();
    CompletenessReport {
        omitted: result.difference(&attributed).copied().collect(),
        phantom: attributed.difference(&result).copied().collect(),
        unresolved_outputs: claims
            .iter()
            .filter(|(index, event)| {
                result.contains(index) && matches!(event, BoundaryEvent::Unresolved { .. })
            })
            .map(|(&index, _)| index)
            .collect(),
    }
}

/// Compare a face map and its boundary claims against explicit result
/// entity sets.
#[must_use]
pub fn completeness_for_result_entities(
    faces: &EvolutionMap,
    boundary: &BoundaryEvolution,
    result_faces: impl IntoIterator<Item = usize>,
    result_edges: impl IntoIterator<Item = usize>,
    result_vertices: impl IntoIterator<Item = usize>,
) -> EntityCompletenessReport {
    EntityCompletenessReport {
        faces: faces.completeness_for_result(result_faces),
        edges: claims_completeness(&boundary.edges, result_edges),
        vertices: claims_completeness(&boundary.vertices, result_vertices),
    }
}

/// Compare a face map and its boundary claims against every face, edge and
/// vertex of the result solids (outer and cavity shells alike).
///
/// # Errors
///
/// Returns [`OperationsError`] if a result solid contains an invalid handle.
pub fn completeness_for_result_solids(
    topo: &Topology,
    faces: &EvolutionMap,
    boundary: &BoundaryEvolution,
    result_solids: &[SolidId],
) -> Result<EntityCompletenessReport, OperationsError> {
    let mut result_faces = BTreeSet::new();
    let mut result_edges = BTreeSet::new();
    let mut result_vertices = BTreeSet::new();
    for &solid in result_solids {
        result_faces.extend(solid_faces(topo, solid)?.into_iter().map(Id::index));
        result_edges.extend(solid_edges(topo, solid)?.into_iter().map(Id::index));
        result_vertices.extend(solid_vertices(topo, solid)?.into_iter().map(Id::index));
    }
    Ok(completeness_for_result_entities(
        faces,
        boundary,
        result_faces,
        result_edges,
        result_vertices,
    ))
}

/// Exact face incidence of one solid's edges and vertices.
///
/// Capture it from the source solid **before** the operation runs; the
/// operation may retire or re-use the source's entities afterwards.
#[derive(Debug, Clone, Default)]
pub struct SourceIncidence {
    /// Sorted face-use multiset (a seam lists its face twice) to the
    /// source edges with exactly that incidence.
    edges: BTreeMap<Vec<usize>, Vec<usize>>,
    /// Sorted face set to the source vertices with exactly that incidence.
    vertices: BTreeMap<Vec<usize>, Vec<usize>>,
}

impl SourceIncidence {
    /// Capture the face incidence of every edge and vertex of `solid`.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the solid contains an invalid handle.
    pub fn capture(topo: &Topology, solid: SolidId) -> Result<Self, OperationsError> {
        let (edges, vertices) = incidence(topo, solid)?;
        let mut by_edge_key: BTreeMap<Vec<usize>, Vec<usize>> = BTreeMap::new();
        for (edge, faces) in edges {
            by_edge_key.entry(faces).or_default().push(edge);
        }
        let mut by_vertex_key: BTreeMap<Vec<usize>, Vec<usize>> = BTreeMap::new();
        for (vertex, faces) in vertices {
            by_vertex_key.entry(faces).or_default().push(vertex);
        }
        for list in by_edge_key.values_mut().chain(by_vertex_key.values_mut()) {
            list.sort_unstable();
        }
        Ok(Self {
            edges: by_edge_key,
            vertices: by_vertex_key,
        })
    }
}

type IncidenceMaps = (BTreeMap<usize, Vec<usize>>, BTreeMap<usize, Vec<usize>>);

/// Edge index to its sorted face-use multiset, and vertex index to its
/// sorted face set, over every shell of `solid`.
fn incidence(topo: &Topology, solid: SolidId) -> Result<IncidenceMaps, OperationsError> {
    let mut edges: BTreeMap<usize, Vec<usize>> = edge_to_face_map(topo, solid)?
        .into_iter()
        .map(|(edge, faces)| (edge, faces.iter().map(|face| face.index()).collect()))
        .collect();
    let mut vertices: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
    for (&edge, faces) in &edges {
        let Some(edge_id) = topo.edge_id_from_index(edge) else {
            return Err(OperationsError::InvalidInput {
                reason: format!("edge index {edge} is not live"),
            });
        };
        let data = topo.edge(edge_id)?;
        for vertex in [data.start(), data.end()] {
            vertices
                .entry(vertex.index())
                .or_default()
                .extend(faces.iter().copied());
        }
    }
    for faces in edges.values_mut() {
        faces.sort_unstable();
    }
    Ok((
        edges,
        vertices
            .into_iter()
            .map(|(vertex, faces)| (vertex, faces.into_iter().collect()))
            .collect(),
    ))
}

/// Result face to its single `modified` source, for every result face the
/// map carries forward from exactly one input; everything else is absent.
fn identity_sources(map: &EvolutionMap) -> BTreeMap<usize, usize> {
    let mut sources: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (&input, outputs) in &map.modified {
        for &output in outputs {
            sources.entry(output).or_default().push(input);
        }
    }
    let generated_or_unresolved: BTreeSet<usize> = map
        .generated
        .values()
        .flatten()
        .copied()
        .chain(map.unresolved.keys().copied())
        .collect();
    sources
        .into_iter()
        .filter_map(|(output, inputs)| match inputs.as_slice() {
            [single] if !generated_or_unresolved.contains(&output) => Some((output, *single)),
            _ => None,
        })
        .collect()
}

fn induced_event(
    result_faces: &[usize],
    identity: &BTreeMap<usize, usize>,
    source: &BTreeMap<Vec<usize>, Vec<usize>>,
    dedup: bool,
) -> BoundaryEvent {
    let mut key = Vec::with_capacity(result_faces.len());
    for face in result_faces {
        let Some(&source_face) = identity.get(face) else {
            return BoundaryEvent::Unresolved {
                candidates: Vec::new(),
                reason: UnresolvedReason::UnmappedIncidentFace,
            };
        };
        key.push(source_face);
    }
    key.sort_unstable();
    if dedup {
        key.dedup();
    }
    match source.get(&key).map(Vec::as_slice) {
        Some([single]) => BoundaryEvent::Modified { from: *single },
        Some(candidates) if candidates.len() > 1 => BoundaryEvent::Unresolved {
            candidates: candidates.to_vec(),
            reason: UnresolvedReason::AmbiguousIncidence,
        },
        _ => {
            key.dedup();
            BoundaryEvent::Generated { faces: key }
        }
    }
}

/// Induce every result edge's and vertex's claim from an exact face map.
///
/// `source` must be captured from the operation's input solid before the
/// operation ran; `faces` is the operation's face map onto `result`. See
/// the [module docs](self) for the induction rules. The returned claims
/// cover every edge and vertex of `result` by construction.
///
/// # Errors
///
/// Returns [`OperationsError`] if `result` contains an invalid handle.
pub fn induce_boundary_evolution(
    topo: &Topology,
    source: &SourceIncidence,
    faces: &EvolutionMap,
    result: SolidId,
) -> Result<BoundaryEvolution, OperationsError> {
    let identity = identity_sources(faces);
    let (result_edges, result_vertices) = incidence(topo, result)?;
    let edges = result_edges
        .iter()
        .map(|(&edge, incident)| {
            (
                edge,
                induced_event(incident, &identity, &source.edges, false),
            )
        })
        .collect();
    let vertices = result_vertices
        .iter()
        .map(|(&vertex, incident)| {
            (
                vertex,
                induced_event(incident, &identity, &source.vertices, true),
            )
        })
        .collect();
    Ok(BoundaryEvolution { edges, vertices })
}

/// Refuse a claim set that does not account for every result entity.
///
/// # Errors
///
/// Returns [`OperationsError::InvalidInput`] naming the first omitted or
/// phantom entities of each kind when `report` is not accounted.
pub fn require_accounted(
    operation: &str,
    report: &EntityCompletenessReport,
) -> Result<(), OperationsError> {
    if report.is_accounted() {
        return Ok(());
    }
    Err(OperationsError::InvalidInput {
        reason: format!(
            "{operation} history does not account for its result: \
             faces omitted {:?} phantom {:?}; edges omitted {:?} phantom {:?}; \
             vertices omitted {:?} phantom {:?}",
            report.faces.omitted,
            report.faces.phantom,
            report.edges.omitted,
            report.edges.phantom,
            report.vertices.omitted,
            report.vertices.phantom,
        ),
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

    use super::*;

    fn claims(pairs: &[(usize, BoundaryEvent)]) -> BTreeMap<usize, BoundaryEvent> {
        pairs.iter().cloned().collect()
    }

    #[test]
    fn induced_events_follow_the_incidence_rules() {
        let identity: BTreeMap<usize, usize> = [(10, 0), (11, 1), (12, 2)].into_iter().collect();
        let source: BTreeMap<Vec<usize>, Vec<usize>> = [
            (vec![0, 1], vec![5]),
            (vec![1, 1], vec![6, 7]),
            (vec![0, 1, 2], vec![8]),
        ]
        .into_iter()
        .collect();

        assert_eq!(
            induced_event(&[11, 10], &identity, &source, false),
            BoundaryEvent::Modified { from: 5 }
        );
        assert_eq!(
            induced_event(&[11, 11], &identity, &source, false),
            BoundaryEvent::Unresolved {
                candidates: vec![6, 7],
                reason: UnresolvedReason::AmbiguousIncidence,
            }
        );
        assert_eq!(
            induced_event(&[12, 10], &identity, &source, false),
            BoundaryEvent::Generated { faces: vec![0, 2] }
        );
        assert_eq!(
            induced_event(&[10, 99], &identity, &source, false),
            BoundaryEvent::Unresolved {
                candidates: Vec::new(),
                reason: UnresolvedReason::UnmappedIncidentFace,
            }
        );
        // Vertex keys are sets: a repeated face does not change the key.
        assert_eq!(
            induced_event(&[12, 11, 10, 10], &identity, &source, true),
            BoundaryEvent::Modified { from: 8 }
        );
    }

    #[test]
    fn identity_excludes_merged_generated_and_unresolved_faces() {
        let mut map = EvolutionMap::exact();
        map.add_modified(0, 10);
        map.add_modified(1, 11);
        map.add_modified(2, 11); // merge: no single identity
        map.add_generated(3, 12);
        map.add_modified(3, 12); // contradictory: generated wins, no identity
        map.add_unresolved(13, vec![4]);
        let identity = identity_sources(&map);
        assert_eq!(identity, std::iter::once((10, 0)).collect());
    }

    #[test]
    fn claim_completeness_reports_omitted_phantom_and_unresolved() {
        let set = claims(&[
            (1, BoundaryEvent::Modified { from: 9 }),
            (
                2,
                BoundaryEvent::Unresolved {
                    candidates: vec![7, 8],
                    reason: UnresolvedReason::AmbiguousIncidence,
                },
            ),
            (40, BoundaryEvent::Generated { faces: vec![0, 1] }),
        ]);
        let report = claims_completeness(&set, [1, 2, 3]);
        assert_eq!(report.omitted, vec![3]);
        assert_eq!(report.phantom, vec![40]);
        assert_eq!(report.unresolved_outputs, vec![2]);
        assert!(!report.is_accounted());
    }

    #[test]
    fn journal_events_keep_kinds_and_drop_only_the_reason() {
        let boundary = BoundaryEvolution {
            edges: claims(&[
                (20, BoundaryEvent::Modified { from: 2 }),
                (21, BoundaryEvent::Merged { from: vec![3, 4] }),
            ]),
            vertices: claims(&[
                (30, BoundaryEvent::Generated { faces: vec![0, 1] }),
                (
                    31,
                    BoundaryEvent::Unresolved {
                        candidates: vec![5, 6],
                        reason: UnresolvedReason::AmbiguousIncidence,
                    },
                ),
            ]),
        };
        let events = boundary.journal_events();
        assert_eq!(
            events,
            vec![
                (
                    EntityKey::edge(20),
                    EventDraft::Modified {
                        from: EntityKey::edge(2)
                    }
                ),
                (
                    EntityKey::edge(21),
                    EventDraft::Merged {
                        from: vec![EntityKey::edge(3), EntityKey::edge(4)]
                    }
                ),
                (
                    EntityKey::vertex(30),
                    EventDraft::Generated {
                        sources: vec![EntityKey::face(0), EntityKey::face(1)]
                    }
                ),
                (
                    EntityKey::vertex(31),
                    EventDraft::Unresolved {
                        candidates: vec![EntityKey::vertex(5), EntityKey::vertex(6)]
                    }
                ),
            ]
        );
        assert_eq!(boundary.unresolved().len(), 1);
    }
}
