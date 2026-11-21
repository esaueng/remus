//! Sharp chamfer details that ride on a translating blend cluster.
//!
//! A chamfer is deliberately not tangent to the faces it bevels, so the
//! tangent walk that collects a moved plane's blend cluster stops at it. On a
//! filleted block whose foot is chamfered, the chamfer band at the moved end
//! (its planar strip and the cones it turns round the vertical rounds) is
//! bounded only by faces that travel with the end and faces the move slides
//! along. Leaving it behind would tear the body; carrying it rigidly is the
//! exact prismatic extension the rest of the cluster already makes.

use crate::OperationsError;
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::face::{FaceId, FaceSurface};
use std::collections::{HashSet, VecDeque};

/// More faces than any chamfer band round one moved end carries; a component
/// this large is a body region, not a detail, and abstains.
const MAX_DETAIL_FACES: usize = 64;

/// A plane or cone that the translation along `direction` does not leave in
/// place: the surfaces a chamfer cuts.
fn is_chamfer_surface(
    topo: &Topology,
    face: FaceId,
    direction: Vec3,
) -> Result<bool, OperationsError> {
    let data = topo.face(face)?;
    Ok(match data.surface() {
        // A riding strip bevels the front of the moved cluster, so it faces
        // forward and obliquely. A plane facing back along the move is never
        // carried, and neither is one square to it: that is a cap of its own
        // (a boss top the moved shoulder reaches), not a chamfer.
        FaceSurface::Plane { .. } => data.effective_plane_normal().is_some_and(|normal| {
            let along = normal.dot(direction);
            let angular = Tolerance::new().angular;
            along > angular && along < 1.0 - angular
        }),
        FaceSurface::Cone(_) => true,
        FaceSurface::Cylinder(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => false,
    })
}

/// The chamfer faces that must translate with `moving`, a selection and its
/// blend cluster, when the move runs along `direction`.
///
/// Each connected component of non-invariant faces next to `moving` is
/// carried whole when every face in it is a chamfer surface and every face
/// it meets is moving, in the component, or translation-invariant (so the
/// component's boundary slides along it). Any other component is left out
/// and the caller's boundary checks refuse it as before.
pub(super) fn riding_chamfer_faces(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    moving: &HashSet<FaceId>,
    direction: Vec3,
) -> Result<HashSet<FaceId>, OperationsError> {
    let is_invariant = |face: FaceId| -> Result<bool, OperationsError> {
        let surface = topo.face(face)?.surface();
        Ok(!matches!(surface, FaceSurface::Nurbs(_))
            && super::surface_translation_invariant(surface, direction))
    };

    // Seeds in face order so the components, and any refusal downstream,
    // do not depend on set iteration order.
    let mut seeds = Vec::new();
    let mut ordered_moving: Vec<FaceId> = moving.iter().copied().collect();
    ordered_moving.compact_sort_unstable_by_key(|face| face.index());
    for &face in &ordered_moving {
        for edge in super::face_edges(topo, face)? {
            for adjacent in super::distinct_faces(adjacency.faces_for_edge(edge)) {
                if !moving.contains(&adjacent) && !is_invariant(adjacent)? {
                    seeds.push(adjacent);
                }
            }
        }
    }

    let mut carried = HashSet::new();
    let mut visited = HashSet::new();
    for seed in seeds {
        if !visited.insert(seed) {
            continue;
        }
        let mut component = vec![seed];
        let mut queue = VecDeque::from([seed]);
        let mut admissible = true;
        while let Some(current) = queue.pop_front() {
            if !is_chamfer_surface(topo, current, direction)? {
                admissible = false;
            }
            for edge in super::face_edges(topo, current)? {
                for adjacent in super::distinct_faces(adjacency.faces_for_edge(edge)) {
                    if adjacent == current || moving.contains(&adjacent) || is_invariant(adjacent)?
                    {
                        continue;
                    }
                    if visited.insert(adjacent) {
                        component.push(adjacent);
                        queue.push_back(adjacent);
                    }
                }
            }
        }
        // The walk always finishes the component, so no later seed can
        // start a partial copy of one refused here.
        if admissible && component.len() <= MAX_DETAIL_FACES {
            carried.extend(component);
        }
    }
    Ok(carried)
}
