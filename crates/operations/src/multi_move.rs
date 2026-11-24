//! Multi-face moves: several planar faces of one solid moved at once, each
//! along its own outward normal (B287).
//!
//! A selection may hold faces on different parallel planes (the tops beside
//! a slot and the slot floor), faces with different normals sharing edges
//! (the walls of a pocket or a boss) and faces with opposite normals (the
//! two sides of a rib). Every selected support is offset by its own signed
//! distance and every source edge and vertex is rebuilt once from its final
//! supports ([`remus_offset::move_faces_multi_with_entity_map`]), so the
//! result has the source's adjacency and an exact, total face, edge and
//! vertex map. Each face's swept region is refused when it reaches a
//! nonadjacent face that stays fixed, as for one coplanar group; a distance
//! at which selected faces reach or pass each other is refused by the
//! engine naming the face that folds. The moved region is then checked as a
//! whole on the result: a face whose
//! footprint the other moves re-limited (the corner two moved walls build)
//! must clear every fixed face it does not touch, and nonadjacent moved
//! faces must clear each other (two walls advancing into one gap). The
//! result passes strict validation.
//!
//! A selection that is one coplanar group moved by one distance (or a
//! single face) is the ordinary face move: it runs
//! `push_pull::move_faces_with_entity_evolution` unchanged, blend
//! handling included, so its results are byte-identical to `move_faces`.
//!
//! Distances: either one signed distance for every face, or one per face in
//! selection order. Positive moves a face outward (adds material).

use remus_math::aabb::Aabb3;
use remus_math::det_hash::DetHashSet;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::explorer::{face_vertices, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::journal_ops::JournaledSolidOp;
use crate::push_pull::{
    DirectEditEvolution, MoveFacesResult, PreparedPlanarMove, SweptRegionCheck,
    boundary_entity_pairs, exact_face_evolution, validate_moved_solid,
};

/// Moves `faces` of `solid` together, each along its own outward normal.
///
/// See the [module docs](self) for the selection, the distances and the
/// postconditions.
///
/// # Errors
///
/// Returns a typed [`remus_offset::OffsetError`] through
/// [`OperationsError::Offset`]: `InvalidInput` for an empty selection, more
/// than [`remus_offset::MAX_MULTI_MOVE_FACES`] faces, a distance list of the
/// wrong length or a zero or non-finite distance; `TopologyChange` naming
/// the face (and edge, where known) when the move collapses, inverts or
/// splits a face or sweeps it into a fixed face. Any failure restores the
/// pre-call topology.
pub fn move_faces_multi(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<SolidId, OperationsError> {
    Ok(move_faces_multi_with_evolution(topo, solid, faces, distances)?.solid)
}

/// [`move_faces_multi`] with its exact, total face evolution.
///
/// # Errors
///
/// Returns the same typed refusals as [`move_faces_multi`].
pub fn move_faces_multi_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<MoveFacesResult, OperationsError> {
    Ok(move_faces_multi_with_entity_evolution(topo, solid, faces, distances)?.0)
}

/// [`move_faces_multi`] journaled as one `move_faces` entry with its exact
/// face, edge and vertex maps. Topology and history form one transaction.
///
/// # Errors
///
/// Returns the refusals of [`move_faces_multi`] or a recording error.
pub fn move_faces_multi_journaled(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<JournaledSolidOp, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let pending = crate::journal_ops::begin_scoped(topo, "move_faces", &[solid])?;
        let edit = move_faces_multi_with_entity_evolution(topo, solid, faces, distances)?;
        crate::journal_ops::record_face_move(topo, pending, edit)
    })
}

pub(crate) fn move_faces_multi_with_entity_evolution(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<DirectEditEvolution, OperationsError> {
    check_counts(faces, distances)?;
    if let Some(distance) = remus_offset::coplanar_group_distance(topo, faces, distances)? {
        return crate::push_pull::move_faces_with_entity_evolution(topo, solid, faces, distance);
    }
    if let Some(refusal) = crate::resize_blend::multi_move_rim_refusal(topo, solid, faces)? {
        return Err(refusal.to_error());
    }
    let checks = MultiChecks::prepare(topo, solid, faces)?;
    multi_edit(topo, solid, faces, distances, checks.as_ref())
}

/// The selection-size refusals, before anything reads the faces.
fn check_counts(faces: &[FaceId], distances: &[f64]) -> Result<(), OperationsError> {
    let reason = if faces.is_empty() {
        "move-face requires at least one selected face".to_owned()
    } else if faces.len() > remus_offset::MAX_MULTI_MOVE_FACES {
        format!(
            "multi-face move accepts at most {} faces, got {}",
            remus_offset::MAX_MULTI_MOVE_FACES,
            faces.len()
        )
    } else if distances.len() != 1 && distances.len() != faces.len() {
        format!(
            "multi-face move needs one distance or one per face ({} faces), got {}",
            faces.len(),
            distances.len()
        )
    } else {
        return Ok(());
    };
    Err(remus_offset::OffsetError::InvalidInput { reason }.into())
}

/// What a multi-face move reads from the source alone: its face list and,
/// for each selected face, its swept-region boxes against every fixed face.
#[derive(Debug, Clone)]
pub(crate) struct MultiChecks {
    source_faces: Vec<FaceId>,
    /// One per selected face, in selection order.
    swept: Vec<SweptRegionCheck>,
    /// One per selected face, in selection order.
    regions: Vec<MovedRegion>,
    /// Every unselected source face with its box, in face order.
    fixed: Vec<(FaceId, Aabb3)>,
}

/// A selected face's source extent, for the whole-region clearance check.
#[derive(Debug, Clone)]
struct MovedRegion {
    face: FaceId,
    normal: Vec3,
    source: Aabb3,
    /// Source faces sharing a vertex with this one, itself included: the
    /// rebuild judges where these meet it.
    touching: DetHashSet<FaceId>,
}

impl MultiChecks {
    /// `Ok(None)` for a selection the engine refuses on its own (a face
    /// outside `solid`, a repeated or non-planar face): the engine then
    /// names the refusal.
    pub(crate) fn prepare(
        topo: &Topology,
        solid: SolidId,
        faces: &[FaceId],
    ) -> Result<Option<Self>, OperationsError> {
        let source_faces = solid_faces(topo, solid)?;
        let selected: DetHashSet<FaceId> = faces.iter().copied().collect();
        if selected.len() != faces.len() || !faces.iter().all(|face| source_faces.contains(face)) {
            return Ok(None);
        }
        let mut swept = Vec::with_capacity(faces.len());
        let mut regions = Vec::with_capacity(faces.len());
        let face_vertex_sets = source_faces
            .iter()
            .map(|&face| {
                Ok((
                    face,
                    face_vertices(topo, face)?
                        .into_iter()
                        .collect::<DetHashSet<_>>(),
                ))
            })
            .collect::<Result<Vec<_>, OperationsError>>()?;
        for &face in faces {
            let Some(normal) = topo.face(face)?.effective_plane_normal() else {
                return Ok(None);
            };
            let own = face_vertex_sets
                .iter()
                .find(|(candidate, _)| *candidate == face)
                .map(|(_, vertices)| vertices);
            let touching = face_vertex_sets
                .iter()
                .filter(|(candidate, vertices)| {
                    *candidate == face
                        || own.is_some_and(|own| vertices.iter().any(|v| own.contains(v)))
                })
                .map(|(candidate, _)| *candidate)
                .collect();
            regions.push(MovedRegion {
                face,
                normal,
                source: crate::measure::face_bounding_box(topo, face)?,
                touching,
            });
            // Other selected faces move too: the engine judges where they
            // meet, so they are no obstacles here.
            let mut others = selected.clone();
            others.remove(&face);
            swept.push(SweptRegionCheck::prepare(
                topo,
                solid,
                // `SweptRegionCheck` takes std sets (`push_pull.rs` is on the
                // det-hash grandfather list) and only looks faces up in them.
                &std::iter::once(face).collect(),
                &others.iter().copied().collect(),
                normal,
            )?);
        }
        let fixed = source_faces
            .iter()
            .filter(|face| !selected.contains(face))
            .map(|&face| Ok((face, crate::measure::face_bounding_box(topo, face)?)))
            .collect::<Result<_, OperationsError>>()?;
        Ok(Some(Self {
            source_faces,
            swept,
            regions,
            fixed,
        }))
    }

    /// Refuses a result whose moved region, taken as a whole, reaches
    /// material it must not: the swept regions of the source checks follow
    /// each face's source footprint, so they miss the corner two moved
    /// walls build together and two selected faces advancing toward each
    /// other.
    ///
    /// Face `i` sweeps from its source to its rebuilt face. Its vertices are
    /// intersections of planes that each move linearly with the move, so
    /// they move linearly too and the swept region lies in the box of the
    /// source and rebuilt faces. A face whose rebuilt box is not its
    /// translated source box was re-limited by the moves around it: that
    /// box must clear every fixed face it shares no vertex with. And the
    /// boxes of two moved faces sharing no vertex must not overlap. Overlap
    /// means positive extent on every axis, beyond the linear tolerance
    /// (a flat box counts where it lies strictly inside the other's span),
    /// so faces that only touch pass.
    ///
    /// The test is conservative: a refusal names a moved face whose
    /// clearance is not certified, never a body known to be wrong.
    fn check_moved_regions(
        &self,
        topo: &Topology,
        moved: &remus_offset::MoveFacesEntityResult,
        distances: &[f64],
    ) -> Result<(), OperationsError> {
        let tolerance = Tolerance::new().linear;
        let mut swept = Vec::with_capacity(self.regions.len());
        for (index, region) in self.regions.iter().enumerate() {
            let distance = distances.get(index).copied().unwrap_or(distances[0]);
            let delta = region.normal * distance;
            let translated = Aabb3 {
                min: region.source.min + delta,
                max: region.source.max + delta,
            };
            let rebuilt = match moved.face_map.get(&region.face.index()) {
                Some(&face) => crate::measure::face_bounding_box(topo, face)?,
                None => translated,
            };
            let relimited = !boxes_match(rebuilt, translated, tolerance);
            swept.push((region.source.union(translated).union(rebuilt), relimited));
        }
        let rebuilt_face = |region: &MovedRegion| moved.face_map.get(&region.face.index()).copied();
        for region in &self.regions {
            let Some(rebuilt) = rebuilt_face(region) else {
                continue;
            };
            for &(fixed, _) in &self.fixed {
                if !region.touching.contains(&fixed)
                    && planar_faces_meet(topo, rebuilt, fixed, tolerance)?
                {
                    return Err(region_refusal(
                        region.face,
                        format!(
                            "this face ends on nonadjacent fixed face {}: the two would share \
                             a face patch",
                            fixed.index()
                        ),
                    ));
                }
            }
        }
        for (region, &(bounds, relimited)) in self.regions.iter().zip(&swept) {
            if !relimited {
                continue;
            }
            if let Some(&(fixed, _)) = self.fixed.iter().find(|(fixed, fixed_bounds)| {
                !region.touching.contains(fixed) && overlaps(bounds, *fixed_bounds, tolerance)
            }) {
                return Err(region_refusal(
                    region.face,
                    format!(
                        "the region the moves sweep at this face reaches nonadjacent fixed face \
                         {}: its clearance is not certified",
                        fixed.index()
                    ),
                ));
            }
        }
        for (first, (region, &(bounds, _))) in self.regions.iter().zip(&swept).enumerate() {
            for (other, &(other_bounds, _)) in self.regions.iter().zip(&swept).skip(first + 1) {
                let meet = match (rebuilt_face(region), rebuilt_face(other)) {
                    (Some(a), Some(b)) => planar_faces_meet(topo, a, b, tolerance)?,
                    _ => false,
                };
                if !region.touching.contains(&other.face)
                    && (meet || overlaps(bounds, other_bounds, tolerance))
                {
                    return Err(region_refusal(
                        region.face,
                        format!(
                            "the regions swept by this face and moved face {} overlap: their \
                             clearance is not certified",
                            other.face.index()
                        ),
                    ));
                }
            }
        }
        Ok(())
    }
}

fn region_refusal(face: FaceId, reason: String) -> OperationsError {
    remus_offset::OffsetError::TopologyChange {
        face: Some(face),
        edge: None,
        reason,
    }
    .into()
}

/// Whether planar faces `first` and `second` lie on one plane facing each
/// other (opposite normals, plane offset within `tolerance`) with boxes
/// that touch or come within `tolerance`. Two cavity shells sharing such a
/// patch still pass strict validation, so a move must not end there; the
/// box overlap test alone lets touching boxes pass, as faces that only
/// meet along a box side (stepped tops and a slot floor) must.
fn planar_faces_meet(
    topo: &Topology,
    first: FaceId,
    second: FaceId,
    tolerance: f64,
) -> Result<bool, OperationsError> {
    let (Some(first_normal), Some(second_normal)) = (
        topo.face(first)?.effective_plane_normal(),
        topo.face(second)?.effective_plane_normal(),
    ) else {
        return Ok(false);
    };
    if first_normal.dot(second_normal) > -(1.0 - 1e-9) {
        return Ok(false);
    }
    let point_on = |face: FaceId| -> Result<Option<remus_math::vec::Point3>, OperationsError> {
        Ok(match face_vertices(topo, face)?.first() {
            Some(&vertex) => Some(topo.vertex(vertex)?.point()),
            None => None,
        })
    };
    let (Some(first_point), Some(second_point)) = (point_on(first)?, point_on(second)?) else {
        return Ok(false);
    };
    if (second_point - first_point).dot(first_normal).abs() > tolerance {
        return Ok(false);
    }
    let first_box = crate::measure::face_bounding_box(topo, first)?;
    let second_box = crate::measure::face_bounding_box(topo, second)?;
    let spans = |b: Aabb3| {
        [
            (b.min.x(), b.max.x()),
            (b.min.y(), b.max.y()),
            (b.min.z(), b.max.z()),
        ]
    };
    Ok(spans(first_box)
        .into_iter()
        .zip(spans(second_box))
        .all(|((a_min, a_max), (b_min, b_max))| a_max.min(b_max) - a_min.max(b_min) >= -tolerance))
}

/// Whether two boxes agree within `tolerance` on every bound.
fn boxes_match(first: Aabb3, second: Aabb3, tolerance: f64) -> bool {
    (first.min - second.min).length() <= tolerance && (first.max - second.max).length() <= tolerance
}

/// Whether two boxes share an open region: on every axis their spans
/// overlap by more than `tolerance`, or one span is flat (within
/// `tolerance`) and lies strictly inside the other.
fn overlaps(first: Aabb3, second: Aabb3, tolerance: f64) -> bool {
    let spans = |b: Aabb3| {
        [
            (b.min.x(), b.max.x()),
            (b.min.y(), b.max.y()),
            (b.min.z(), b.max.z()),
        ]
    };
    spans(first)
        .into_iter()
        .zip(spans(second))
        .all(|((a_min, a_max), (b_min, b_max))| {
            let inside = |(min, max): (f64, f64), (outer_min, outer_max): (f64, f64)| {
                max - min <= tolerance && min > outer_min + tolerance && max < outer_max - tolerance
            };
            a_max.min(b_max) - a_min.max(b_min) > tolerance
                || inside((a_min, a_max), (b_min, b_max))
                || inside((b_min, b_max), (a_min, a_max))
        })
}

/// The multi-face solve, its swept-region refusals and postconditions.
fn multi_edit(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
    checks: Option<&MultiChecks>,
) -> Result<DirectEditEvolution, OperationsError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let outcome = (|| -> Result<DirectEditEvolution, OperationsError> {
        if let Some(checks) = checks {
            for (index, check) in checks.swept.iter().enumerate() {
                let distance = distances.get(index).copied().unwrap_or(distances[0]);
                if distance.is_finite() {
                    check.check_offset(topo, distance)?;
                }
            }
        }
        let moved = crate::performance::timed("move_faces.multi_engine", || {
            remus_offset::move_faces_multi_with_entity_map(topo, solid, faces, distances)
        })?;
        if let Some(checks) = checks {
            checks.check_moved_regions(topo, &moved, distances)?;
        }
        let boundary_pairs = boundary_entity_pairs(&moved);
        let source_faces = match checks {
            Some(checks) => checks.source_faces.clone(),
            None => solid_faces(topo, solid)?,
        };
        let result = MoveFacesResult {
            solid: moved.solid,
            evolution: exact_face_evolution(topo, &source_faces, moved.solid, moved.face_map)?,
        };
        validate_moved_solid(topo, faces, result.solid)?;
        Ok((result, boundary_pairs))
    })();
    if outcome.is_err() {
        snapshot.restore(topo);
    }
    outcome
}

/// A multi-face selection prepared for repeated moves by a scalar drag
/// distance: face `i` moves `weights[i] × distance`.
pub(crate) struct PreparedMultiMove {
    weights: Vec<f64>,
    /// The coplanar move's preparation, when every weight is the same and
    /// the selection is one coplanar group (so every call is that move).
    group: Option<PreparedPlanarMove>,
    checks: Option<MultiChecks>,
    /// The rim refusal every call that is not the coplanar move returns.
    rim_refusal: Option<crate::resize_blend::MultiRimRefusal>,
}

impl PreparedMultiMove {
    /// Prepares `faces` with per-face `weights` (one per face, or empty for
    /// all ones).
    pub(crate) fn prepare(
        topo: &Topology,
        solid: SolidId,
        faces: &[FaceId],
        weights: &[f64],
    ) -> Result<Self, OperationsError> {
        let weights = if weights.is_empty() {
            vec![1.0; faces.len()]
        } else {
            weights.to_vec()
        };
        if weights.len() != faces.len() || weights.iter().any(|weight| !weight.is_finite()) {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "multi-face move weights must be finite, one per face ({} faces), got {}",
                    faces.len(),
                    weights.len()
                ),
            });
        }
        check_counts(faces, &weights)?;
        let uniform = weights
            .iter()
            .all(|weight| weight.to_bits() == weights[0].to_bits());
        let group =
            if uniform && remus_offset::coplanar_group_distance(topo, faces, &[1.0])?.is_some() {
                PreparedPlanarMove::prepare(topo, solid, faces)?
            } else {
                None
            };
        Ok(Self {
            weights,
            group,
            checks: MultiChecks::prepare(topo, solid, faces)?,
            rim_refusal: crate::resize_blend::multi_move_rim_refusal(topo, solid, faces)?,
        })
    }

    /// The per-face distances of a drag by `distance`.
    pub(crate) fn distances(&self, distance: f64) -> Vec<f64> {
        self.weights
            .iter()
            .map(|weight| weight * distance)
            .collect()
    }

    /// [`move_faces_multi_with_entity_evolution`] of the prepared selection
    /// by `distance` (scaled by the weights).
    pub(crate) fn move_faces(
        &mut self,
        topo: &mut Topology,
        solid: SolidId,
        faces: &[FaceId],
        distance: f64,
    ) -> Result<DirectEditEvolution, OperationsError> {
        let distances = self.distances(distance);
        if let Some(group_distance) =
            remus_offset::coplanar_group_distance(topo, faces, &distances)?
        {
            return match &mut self.group {
                Some(group) => group.move_faces(topo, solid, faces, group_distance),
                None => crate::push_pull::move_faces_with_entity_evolution(
                    topo,
                    solid,
                    faces,
                    group_distance,
                ),
            };
        }
        if let Some(refusal) = &self.rim_refusal {
            return Err(refusal.to_error());
        }
        multi_edit(topo, solid, faces, &distances, self.checks.as_ref())
    }
}
