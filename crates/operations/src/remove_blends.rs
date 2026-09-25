//! Atomic removal of an explicitly selected connected blend group.
//!
//! Callers provide seed faces, not a trusted list of faces to delete. Each
//! non-spherical seed is expanded to the complete connected portion of its
//! exact analytic carrier within a proven blend region. Duplicate seeds, and
//! multiple seeds on split faces of that same carrier, therefore identify one
//! band. Adjacent sibling bands are included only when one of their own faces
//! is seeded. A spherical corner patch joins the group only when every
//! incident band in its recognized region was selected; partial corners are
//! refused rather than left hanging.
//!
//! The recognized union is reconstructed as one wound. No intermediate solid
//! with only one band removed is published or used as the input to another
//! removal. Face evolution and edge/vertex evolution both come from the
//! reconstruction's construction records; this module never pairs entities by
//! distance or nearest geometry.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::{EntityKey, EntityKind};
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::evolution::EvolutionMap;
use crate::resize_blend::{ResizeBlendError, blend_region};

/// Exact result of removing a connected group of analytic blend bands.
#[derive(Debug)]
pub struct RemoveBlendsResult {
    /// Reconstructed solid with the complete selected group removed.
    pub solid: SolidId,
    /// Construction-derived evolution for every source and result face.
    pub evolution: EvolutionMap,
    /// Construction-derived edge and vertex replacement/deletion records.
    ///
    /// A `None` target means the source boundary was consumed by the edit.
    /// Records are sorted by source and target for deterministic replay.
    pub boundary_history: Vec<(EntityKey, Option<EntityKey>)>,
}

/// Remove one explicitly seeded, connected group of analytic blend bands.
///
/// A seed may name any face of a split cylindrical or toroidal band; the
/// operation expands it to the whole connected face set on that exact carrier.
/// Repeated seeds and input order do not affect recognition or output. Separate
/// adjacent bands require separate seeds, even when they share a radius. Exact
/// spherical corner patches are included only after all of their incident
/// bands have been seeded. Every selected band must have one common radius;
/// mixed-radius group healing remains unqualified and is refused atomically.
///
/// Recognition, reconstruction, validation and construction-history checks
/// run in one topology transaction. Any refusal restores the complete arena,
/// including handle slots.
///
/// # Errors
///
/// Returns a typed resize-blend refusal when the seed set is empty, names a
/// foreign or unsupported face, leaves a corner partial, is disconnected, or
/// cannot be reconstructed with complete construction history.
pub fn remove_blends(
    topo: &mut Topology,
    solid: SolidId,
    seeds: &[FaceId],
) -> Result<RemoveBlendsResult, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        remove_blends_inner(topo, solid, seeds)
    })
}

fn invalid(reason: impl Into<String>) -> OperationsError {
    ResizeBlendError::InvalidInput {
        reason: reason.into(),
    }
    .into()
}

fn reconstruction(reason: impl Into<String>) -> OperationsError {
    ResizeBlendError::ReconstructionFailed {
        reason: reason.into(),
    }
    .into()
}

#[derive(Debug)]
struct RecognizedGroup {
    faces: Vec<FaceId>,
}

fn remove_blends_inner(
    topo: &mut Topology,
    solid: SolidId,
    seeds: &[FaceId],
) -> Result<RemoveBlendsResult, OperationsError> {
    let group = recognize_group(topo, solid, seeds)?;
    let source_faces = remus_topology::explorer::solid_faces(topo, solid)?;
    let removed: BTreeSet<usize> = group.faces.iter().map(|face| face.index()).collect();

    let reconstructed =
        crate::resize_blend::remove_recognized_blend_faces(topo, solid, &group.faces)?;

    let result_faces = remus_topology::explorer::solid_faces(topo, reconstructed.solid)?;
    let result_face_set: BTreeSet<FaceId> = result_faces.iter().copied().collect();
    let mapped_sources: BTreeSet<usize> = reconstructed.face_map.keys().copied().collect();
    let expected_sources: BTreeSet<usize> = source_faces
        .iter()
        .filter(|face| !removed.contains(&face.index()))
        .map(|face| face.index())
        .collect();
    let mapped_targets: BTreeSet<FaceId> = reconstructed.face_map.values().copied().collect();
    if mapped_sources != expected_sources
        || mapped_targets != result_face_set
        || reconstructed.face_map.len() != result_faces.len()
    {
        return Err(reconstruction(
            "connected blend removal has incomplete or ambiguous face construction history",
        ));
    }

    let mut evolution = EvolutionMap::exact();
    let mut face_map: Vec<_> = reconstructed.face_map.into_iter().collect();
    face_map.sort_unstable_by_key(|(source, target)| (*source, target.index()));
    for (source, target) in face_map {
        evolution.add_modified(source, target.index());
    }
    let mut removed_faces: Vec<_> = removed.into_iter().collect();
    removed_faces.sort_unstable();
    for face in removed_faces {
        evolution.add_deleted(face);
    }
    if !evolution.is_construction_resolved_for_result(result_faces.iter().map(|face| face.index()))
    {
        return Err(reconstruction(
            "connected blend removal face evolution does not account for the result",
        ));
    }

    let boundary_history = validate_boundary_history(
        topo,
        solid,
        reconstructed.solid,
        reconstructed.boundary_history,
    )?;
    Ok(RemoveBlendsResult {
        solid: reconstructed.solid,
        evolution,
        boundary_history,
    })
}

fn recognize_group(
    topo: &Topology,
    solid: SolidId,
    seeds: &[FaceId],
) -> Result<RecognizedGroup, OperationsError> {
    if seeds.is_empty() {
        return Err(invalid("at least one blend seed is required"));
    }
    let solid_faces: BTreeSet<FaceId> = remus_topology::explorer::solid_faces(topo, solid)?
        .into_iter()
        .collect();
    let mut seeds = seeds.to_vec();
    seeds.sort_unstable_by_key(|face| face.index());
    seeds.dedup();
    if let Some(face) = seeds.iter().find(|face| !solid_faces.contains(face)) {
        return Err(invalid(format!(
            "blend seed face {} is not part of solid {}",
            face.index(),
            solid.index()
        )));
    }

    let adjacency = topo.build_adjacency(solid)?;
    let mut selected = BTreeSet::<FaceId>::new();
    let mut regions = BTreeMap::<Vec<usize>, Vec<FaceId>>::new();
    let mut corner_seeds = BTreeSet::<FaceId>::new();
    let mut band_keys = BTreeSet::<Vec<usize>>::new();

    for seed in seeds {
        let surface = topo.face(seed)?.surface();
        if !matches!(
            surface,
            FaceSurface::Cylinder(_) | FaceSurface::Torus(_) | FaceSurface::Sphere(_)
        ) {
            return Err(ResizeBlendError::BandNotAnalytic {
                surface: surface.type_tag(),
            }
            .into());
        }
        let region = blend_region(topo, solid, seed)?;
        let region_key: Vec<_> = region.faces.iter().map(|face| face.index()).collect();
        regions
            .entry(region_key)
            .or_insert_with(|| region.faces.clone());
        if matches!(surface, FaceSurface::Sphere(_)) {
            corner_seeds.insert(seed);
            continue;
        }

        let band = carrier_band(topo, &adjacency, seed, &region.faces)?;
        let key: Vec<_> = band.iter().map(|face| face.index()).collect();
        if band_keys.insert(key) {
            selected.extend(band);
        }
    }

    if selected.is_empty() {
        return Err(reconstruction(
            "a spherical corner seed does not identify its incident edge bands; seed every band to remove",
        ));
    }

    // Corner patches are closure faces, not independent edge bands. Include a
    // patch only if every incident non-spherical blend face is already in the
    // explicitly seeded band union. Selecting just one incident band must not
    // silently consume its same-radius siblings through the corner.
    for region in regions.values() {
        let region_set: BTreeSet<_> = region.iter().copied().collect();
        for &corner in region.iter().filter(|face| {
            topo.face(**face)
                .is_ok_and(|face| matches!(face.surface(), FaceSurface::Sphere(_)))
        }) {
            let incident: BTreeSet<_> = face_edges(topo, corner)?
                .into_iter()
                .flat_map(|edge| adjacency.faces_for_edge(edge).iter().copied())
                .filter(|face| {
                    *face != corner
                        && region_set.contains(face)
                        && topo.face(*face).is_ok_and(|face| {
                            matches!(
                                face.surface(),
                                FaceSurface::Cylinder(_) | FaceSurface::Torus(_)
                            )
                        })
                })
                .collect();
            let chosen = incident
                .iter()
                .filter(|face| selected.contains(face))
                .count();
            if chosen == 0 {
                if corner_seeds.contains(&corner) {
                    return Err(reconstruction(format!(
                        "corner patch {} was seeded without its incident blend bands",
                        corner.index()
                    )));
                }
                continue;
            }
            if chosen != incident.len() {
                return Err(reconstruction(format!(
                    "selected blend bands leave corner patch {} partial ({chosen}/{} incident bands)",
                    corner.index(),
                    incident.len()
                )));
            }
            selected.insert(corner);
        }
    }
    if let Some(corner) = corner_seeds
        .iter()
        .find(|corner| !selected.contains(corner))
    {
        return Err(reconstruction(format!(
            "corner patch {} is outside the completely selected band group",
            corner.index()
        )));
    }

    let faces: Vec<_> = selected.into_iter().collect();
    require_connected(topo, &adjacency, &faces)?;
    Ok(RecognizedGroup { faces })
}

fn carrier_band(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    seed: FaceId,
    region: &[FaceId],
) -> Result<Vec<FaceId>, OperationsError> {
    let region: BTreeSet<_> = region.iter().copied().collect();
    let seed_surface = topo.face(seed)?.surface().clone();
    let mut found = BTreeSet::from([seed]);
    let mut queue = VecDeque::from([seed]);
    while let Some(current) = queue.pop_front() {
        for edge in face_edges(topo, current)? {
            for &candidate in adjacency.faces_for_edge(edge) {
                if candidate == current
                    || found.contains(&candidate)
                    || !region.contains(&candidate)
                    || !crate::heal::surfaces_equivalent_pub(
                        &seed_surface,
                        topo.face(candidate)?.surface(),
                    )
                {
                    continue;
                }
                found.insert(candidate);
                queue.push_back(candidate);
            }
        }
    }
    Ok(found.into_iter().collect())
}

fn require_connected(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    faces: &[FaceId],
) -> Result<(), OperationsError> {
    let group: BTreeSet<_> = faces.iter().copied().collect();
    let Some(&first) = faces.first() else {
        return Err(invalid("recognized blend group is empty"));
    };
    let mut reached = BTreeSet::from([first]);
    let mut queue = VecDeque::from([first]);
    while let Some(current) = queue.pop_front() {
        for edge in face_edges(topo, current)? {
            for &candidate in adjacency.faces_for_edge(edge) {
                if group.contains(&candidate) && reached.insert(candidate) {
                    queue.push_back(candidate);
                }
            }
        }
    }
    if reached.len() != faces.len() {
        return Err(reconstruction(format!(
            "blend seeds identify {} disconnected groups",
            connected_component_count(topo, adjacency, &group)?
        )));
    }
    Ok(())
}

fn connected_component_count(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    remaining: &BTreeSet<FaceId>,
) -> Result<usize, OperationsError> {
    let mut remaining = remaining.clone();
    let mut components = 0;
    while let Some(&first) = remaining.iter().min_by_key(|face| face.index()) {
        components += 1;
        remaining.remove(&first);
        let mut queue = VecDeque::from([first]);
        while let Some(current) = queue.pop_front() {
            for edge in face_edges(topo, current)? {
                for &candidate in adjacency.faces_for_edge(edge) {
                    if remaining.remove(&candidate) {
                        queue.push_back(candidate);
                    }
                }
            }
        }
    }
    Ok(components)
}

fn face_edges(topo: &Topology, face: FaceId) -> Result<Vec<EdgeId>, OperationsError> {
    let face = topo.face(face)?;
    let mut edges = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        edges.extend(
            topo.wire(wire)?
                .edges()
                .iter()
                .map(remus_topology::wire::OrientedEdge::edge),
        );
    }
    edges.sort_unstable_by_key(|edge| edge.index());
    edges.dedup();
    Ok(edges)
}

fn validate_boundary_history(
    topo: &Topology,
    source: SolidId,
    result: SolidId,
    mut history: Vec<(EntityKey, Option<EntityKey>)>,
) -> Result<Vec<(EntityKey, Option<EntityKey>)>, OperationsError> {
    history.sort_unstable();
    if history.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(reconstruction(
            "connected blend removal has duplicate boundary history records",
        ));
    }
    let source_boundaries: BTreeSet<_> = crate::journal_ops::solid_entity_keys(topo, source)?
        .into_iter()
        .filter(|key| key.kind != EntityKind::Face)
        .collect();
    let result_boundaries: BTreeSet<_> = crate::journal_ops::solid_entity_keys(topo, result)?
        .into_iter()
        .filter(|key| key.kind != EntityKind::Face)
        .collect();
    let recorded_sources: BTreeSet<_> = history.iter().map(|(source, _)| *source).collect();
    let recorded_targets: BTreeSet<_> = history.iter().filter_map(|(_, target)| *target).collect();
    let deleted_sources: BTreeSet<_> = history
        .iter()
        .filter_map(|(source, target)| target.is_none().then_some(*source))
        .collect();
    if recorded_sources != source_boundaries
        || recorded_targets != result_boundaries
        || history.iter().any(|(source, target)| {
            target.is_some_and(|target| {
                source.kind != target.kind || deleted_sources.contains(source)
            })
        })
    {
        return Err(reconstruction(
            "connected blend removal boundary construction history is incomplete or ambiguous",
        ));
    }
    Ok(history)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use remus_topology::explorer::solid_edges;

    use super::*;
    use crate::test_helpers::{blended_box, cylinder_faces, split_band};

    /// Run group recognition with a watchdog: a carrier walk that re-queues
    /// faces it already found never terminates, which must fail the test
    /// rather than stall the suite.
    fn recognize_with_watchdog(
        topo: Topology,
        solid: SolidId,
        seeds: Vec<FaceId>,
    ) -> Result<Vec<FaceId>, OperationsError> {
        let (sender, receiver) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let result = recognize_group(&topo, solid, &seeds).map(|group| group.faces);
            let _ = sender.send(result);
        });
        receiver
            .recv_timeout(std::time::Duration::from_secs(60))
            .expect("blend group recognition must terminate")
    }

    #[test]
    fn any_seed_on_a_split_band_recognizes_the_whole_carrier_band() {
        let (mut topo, blended) = blended_box(&[0]);
        let (split, halves) = split_band(&mut topo, blended);
        let mut expected = halves.to_vec();
        expected.sort_unstable_by_key(|face| face.index());
        for seeds in [
            vec![halves[0]],
            vec![halves[1]],
            vec![halves[1], halves[0], halves[1]],
        ] {
            let group = recognize_with_watchdog(topo.clone(), split, seeds.clone())
                .unwrap_or_else(|error| panic!("seeds {seeds:?}: {error}"));
            assert_eq!(
                group, expected,
                "seeds {seeds:?} name the one split carrier band"
            );
        }
    }

    #[test]
    fn disconnected_seeds_name_the_number_of_groups() {
        // Two blends on edges that share no face: two separate groups.
        let mut topo = Topology::new();
        let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, sharp).unwrap();
        let faces_of = |edge: EdgeId| -> BTreeSet<FaceId> {
            let adjacency = topo.build_adjacency(sharp).unwrap();
            adjacency.faces_for_edge(edge).iter().copied().collect()
        };
        let first = edges[0];
        let second = edges
            .iter()
            .copied()
            .find(|edge| faces_of(*edge).is_disjoint(&faces_of(first)))
            .expect("an edge sharing no face with the first");
        let solid = crate::blend_ops::fillet_v2(&mut topo, sharp, &[first, second], 1.0)
            .unwrap()
            .solid;
        let bands = cylinder_faces(&topo, solid);
        assert_eq!(bands.len(), 2);
        let error = remove_blends(&mut topo, solid, &bands).unwrap_err();
        assert!(
            error.to_string().contains("identify 2 disconnected groups"),
            "two separated bands are two groups: {error}"
        );
    }
}
