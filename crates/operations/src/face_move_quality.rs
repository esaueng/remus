//! Face moves with explicit permission for sampled blend reconstruction.

use remus_blend::cap_reblend::CapReblendApproximation;
use remus_topology::{FaceId, SolidId, Topology};

use crate::OperationsError;
use crate::push_pull::{DirectEditEvolution, MoveFacesResult};

/// Permission and bounded work for sampled cap-corner reconstruction.
pub use remus_blend::cap_reblend::CapReblendApproximation as FaceMoveApproximation;

/// Construction quality of a successful face move.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceMoveQuality {
    /// No additional sampled geometry was constructed by the move.
    Exact,
    /// Cap corners were reconstructed using sampled circular sections.
    SampledCapJoin {
        /// Stations used on each corner contour.
        stations: usize,
    },
}

/// A moved body with its face evolution and explicit construction quality.
#[derive(Debug)]
pub struct QualifiedFaceMove {
    /// Body and construction-derived face evolution.
    pub result: MoveFacesResult,
    /// Exact or explicitly approximate construction.
    pub quality: FaceMoveQuality,
}

/// Move faces, permitting a bounded sampled cap reconstruction only when requested.
///
/// `None` retains the exact-only contract of [`crate::push_pull::move_faces`].
/// Approximation permission never bypasses solid validation or lineage checks.
///
/// # Errors
/// Returns a geometry refusal, preserving topology and history atomically.
pub fn move_faces_with_quality(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
    approximation: Option<FaceMoveApproximation>,
) -> Result<QualifiedFaceMove, OperationsError> {
    let (result, quality) = edit_with_quality(topo, solid, faces, distance, approximation)?;
    Ok(QualifiedFaceMove {
        result: result.0,
        quality,
    })
}

pub(crate) fn edit_with_quality(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
    approximation: Option<CapReblendApproximation>,
) -> Result<(DirectEditEvolution, FaceMoveQuality), OperationsError> {
    check_permission(approximation)?;
    match crate::push_pull::move_faces_with_entity_evolution(topo, solid, faces, distance) {
        Ok(result) => Ok((result, FaceMoveQuality::Exact)),
        Err(error) => {
            let Some(permission) = approximation else {
                return Err(error);
            };
            remus_topology::transaction::run_transacted(topo, |topo| {
                let Some(moved) = crate::resize_blend::move_with_cap_reblend(
                    topo, solid, faces, distance, permission,
                )?
                else {
                    return Err(error);
                };
                Ok((
                    (
                        MoveFacesResult {
                            solid: moved.solid,
                            evolution: moved.evolution,
                        },
                        moved.boundary_pairs,
                    ),
                    FaceMoveQuality::SampledCapJoin {
                        stations: permission.stations,
                    },
                ))
            })
        }
    }
}

pub(crate) fn check_permission(
    permission: Option<FaceMoveApproximation>,
) -> Result<(), OperationsError> {
    if permission.is_some_and(|permission| !(16..=2048).contains(&permission.stations)) {
        return Err(OperationsError::InvalidInput {
            reason: "cap reconstruction requires 16..=2048 stations".into(),
        });
    }
    Ok(())
}

/// Complete boundary ancestry for the cap builder's newly constructed wound.
/// Identity is supplied only by `pairs`; new edges and vertices name the
/// construction origins of their incident faces and never inherit an old pick.
pub(crate) fn boundary_outputs(
    topo: &Topology,
    source: SolidId,
    result: &MoveFacesResult,
    pairs: &[(
        remus_topology::journal::EntityKey,
        remus_topology::journal::EntityKey,
    )],
) -> Result<
    Vec<(
        remus_topology::journal::EntityKey,
        remus_topology::journal::EventDraft,
    )>,
    OperationsError,
> {
    use remus_topology::explorer::{face_edges, face_vertices, solid_faces};
    use remus_topology::journal::{EntityKey, EntityKind, EventDraft};
    use std::collections::{BTreeMap, BTreeSet};
    let invalid = || OperationsError::InvalidInput {
        reason: "cap reconstruction has incomplete construction ancestry".into(),
    };
    if !result.evolution.origin.is_exact() || !result.evolution.unresolved.is_empty() {
        return Err(invalid());
    }
    let mut parents = BTreeMap::<usize, BTreeSet<usize>>::new();
    for map in [&result.evolution.modified, &result.evolution.generated] {
        for (&source, outputs) in map {
            for &output in outputs {
                parents.entry(output).or_default().insert(source);
            }
        }
    }
    let mapped_sources: BTreeSet<_> = pairs.iter().map(|&(source, _)| source).collect();
    let mapped_targets: BTreeSet<_> = pairs.iter().map(|&(_, target)| target).collect();
    let mut outputs: Vec<_> = crate::evolution::solid_entity_keys(topo, source)?
        .into_iter()
        .filter(|key| key.kind != EntityKind::Face && !mapped_sources.contains(key))
        .map(|key| (key, EventDraft::Deleted))
        .collect();
    let mut generated = BTreeMap::<EntityKey, BTreeSet<usize>>::new();
    for face in solid_faces(topo, result.solid)? {
        let origins = parents
            .get(&face.index())
            .filter(|set| !set.is_empty())
            .ok_or_else(invalid)?;
        for key in face_edges(topo, face)?
            .into_iter()
            .map(|edge| EntityKey::edge(edge.index()))
            .chain(
                face_vertices(topo, face)?
                    .into_iter()
                    .map(|vertex| EntityKey::vertex(vertex.index())),
            )
        {
            if !mapped_targets.contains(&key) {
                generated.entry(key).or_default().extend(origins);
            }
        }
    }
    outputs.extend(generated.into_iter().map(|(key, sources)| {
        (
            key,
            EventDraft::Generated {
                sources: sources.into_iter().map(EntityKey::face).collect(),
            },
        )
    }));
    Ok(outputs)
}
