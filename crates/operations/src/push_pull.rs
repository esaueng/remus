//! Direct push/pull editing of an existing solid's faces.
//!
//! These operations modify a solid in place (returning a new solid) by moving
//! one of its faces, as opposed to [`crate::offset_face`], which offsets a
//! standalone face and produces a new face.
//!
//! Both operations follow the same shape: derive an exact tool solid from the
//! selected face's own geometry, apply a boolean, merge the coplanar/coaxial
//! seams the boolean leaves behind, and refuse to return a result whose shell
//! is not closed.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::f64::consts::PI;

use remus_math::aabb::Aabb3;
use remus_math::mat::Mat4;
use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::EntityKey;
use remus_topology::solid::SolidId;

use crate::boolean::{BooleanOp, boolean};
use crate::copy::{copy_face, copy_solid_with_face_map};
use crate::evolution::EvolutionMap;
use crate::extrude::extrude;
use crate::heal::unify_faces;
use crate::measure::solid_volume;
use crate::primitives::make_cylinder;
use crate::transform::transform_solid;

/// How a cylindrical face sits in its solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Concavity {
    /// A bore: material lies outside the cylinder, the solid's outward normal
    /// points at the axis.
    Hole,
    /// A boss: material lies inside the cylinder.
    Boss,
}

/// A topology-preserving face move and its construction-derived evolution.
#[derive(Debug)]
pub struct MoveFacesResult {
    /// Edited solid.
    pub solid: SolidId,
    /// Exact face evolution when the construction could prove it; otherwise
    /// every uncertain result face is reported in `unresolved`.
    pub evolution: EvolutionMap,
}

/// Move a supported face selection while preserving the solid's adjacency
/// graph.
///
/// A coplanar group of planar faces moves along its common outward normal.
/// Recognized constant-radius blend regions that meet the selection are
/// removed and rebuilt exactly around the moved faces.
/// One cylindrical bore wall moves radially along its outward normal, so a
/// positive distance shrinks the bore and a negative distance widens it.
/// Cylindrical moves currently require two circular rims on planar supports
/// perpendicular to the bore axis. The edit is transactional: any
/// intersection, topology, validation, or volume failure restores the
/// topology to its pre-call state.
///
/// # Errors
///
/// Returns a structured [`remus_offset::OffsetError`] through
/// [`crate::OperationsError::Offset`] when the selection is unsupported or
/// the move would change topology. A failed postcondition also returns an
/// error and leaves the input topology unchanged.
pub fn move_faces(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<SolidId, crate::OperationsError> {
    Ok(move_faces_with_evolution(topo, solid, faces, distance)?.solid)
}

/// [`move_faces`] with construction-derived face evolution.
///
/// Planar re-limitation and coaxial bore moves report a total one-to-one face
/// map. Blend-aware moves keep the same identity for each rebuilt band when
/// its support pair proves a unique correspondence; more complex regions fail
/// closed in [`EvolutionMap::unresolved`] rather than guessing.
///
/// # Errors
///
/// Returns the same typed refusals as [`move_faces`]. Any failure restores the
/// topology to its pre-call state.
pub fn move_faces_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<MoveFacesResult, crate::OperationsError> {
    Ok(move_faces_with_entity_evolution(topo, solid, faces, distance)?.0)
}

pub(crate) type DirectEditEvolution = (MoveFacesResult, Vec<(EntityKey, EntityKey)>);

pub(crate) fn move_faces_with_entity_evolution(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<DirectEditEvolution, crate::OperationsError> {
    let snapshot = topo.clone();
    let outcome = (|| -> Result<DirectEditEvolution, crate::OperationsError> {
        let boundary_pairs;
        let source_faces = solid_faces(topo, solid)?;
        let result = if let Some((face, new_radius)) =
            cylindrical_bore_move_request(topo, solid, faces, distance)?
        {
            let source_counts = remus_topology::explorer::solid_entity_counts(topo, solid)?;
            let cylinder = match topo.face(face)?.surface() {
                FaceSurface::Cylinder(cylinder) => cylinder,
                surface => {
                    return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                        face,
                        surface_type: surface.type_tag(),
                        reason: "cylindrical move classification changed before replacement".into(),
                    }
                    .into());
                }
            };
            let replacement = CylindricalSurface::with_ref_dir(
                cylinder.origin(),
                cylinder.axis(),
                new_radius,
                cylinder.x_axis(),
            )?;
            let replaced = crate::replace_surface::replace_surface_with_entity_map(
                topo,
                solid,
                face,
                FaceSurface::Cylinder(replacement),
            )?;
            boundary_pairs = boundary_entity_pairs(&replaced);
            let result_counts =
                remus_topology::explorer::solid_entity_counts(topo, replaced.solid)?;
            if result_counts != source_counts {
                return Err(remus_offset::OffsetError::TopologyChange {
                    face: Some(face),
                    edge: None,
                    reason: format!(
                        "radial move changed entity counts from {source_counts:?} to {result_counts:?} (F, E, V)"
                    ),
                }
                .into());
            }
            MoveFacesResult {
                solid: replaced.solid,
                evolution: exact_face_evolution(
                    topo,
                    &source_faces,
                    replaced.solid,
                    replaced.face_map,
                )?,
            }
        } else if let Some(result) =
            crate::resize_blend::move_planar_faces_with_blends(topo, solid, faces, distance)?
        {
            boundary_pairs = result.boundary_pairs;
            MoveFacesResult {
                solid: result.solid,
                evolution: result.evolution,
            }
        } else {
            refuse_swept_face_intersections(topo, solid, faces, distance)?;
            let moved = remus_offset::move_faces_with_entity_map(topo, solid, faces, distance)?;

            boundary_pairs = boundary_entity_pairs(&moved);
            if move_is_prismatic(topo, solid, faces)? {
                let deflection = verify_deflection(topo, solid);
                let before = solid_volume(topo, solid, deflection)?;
                let area = faces.iter().try_fold(0.0, |sum, &face| {
                    crate::measure::face_area(topo, face, deflection).map(|value| sum + value)
                })?;
                let expected = distance.mul_add(area, before);
                let actual = solid_volume(topo, moved.solid, verify_deflection(topo, moved.solid))?;
                let slack = expected.abs().mul_add(2e-3, 1e-6);
                if (actual - expected).abs() > slack {
                    return Err(remus_offset::OffsetError::TopologyChange {
                        face: faces.first().copied(),
                        edge: None,
                        reason: format!("volume is {actual}, expected {expected}"),
                    }
                    .into());
                }
            }
            MoveFacesResult {
                solid: moved.solid,
                evolution: exact_face_evolution(topo, &source_faces, moved.solid, moved.face_map)?,
            }
        };

        let report = crate::validate::validate_solid(topo, result.solid)?;
        if !report.is_valid() {
            let summary = report
                .issues
                .iter()
                .filter(|issue| issue.severity == crate::validate::Severity::Error)
                .take(3)
                .map(|issue| issue.description.as_str())
                .collect::<Vec<_>>()
                .join("; ");
            return Err(remus_offset::OffsetError::TopologyChange {
                face: faces.first().copied(),
                edge: None,
                reason: format!(
                    "strict validation failed with {} error(s): {summary}",
                    report.error_count()
                ),
            }
            .into());
        }

        Ok((result, boundary_pairs))
    })();

    if outcome.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    outcome
}

pub(crate) fn boundary_entity_pairs(
    result: &remus_offset::MoveFacesEntityResult,
) -> Vec<(EntityKey, EntityKey)> {
    result
        .edge_map
        .iter()
        .map(|(&source, target)| (EntityKey::edge(source), EntityKey::edge(target.index())))
        .chain(result.vertex_map.iter().map(|(&source, target)| {
            (EntityKey::vertex(source), EntityKey::vertex(target.index()))
        }))
        .collect()
}

pub(crate) fn exact_face_evolution(
    topo: &Topology,
    source_faces: &[FaceId],
    result: SolidId,
    pairs: impl IntoIterator<Item = (usize, FaceId)>,
) -> Result<EvolutionMap, crate::OperationsError> {
    let map: std::collections::BTreeMap<_, _> = pairs.into_iter().collect();
    let source: std::collections::BTreeSet<_> =
        source_faces.iter().map(|face| face.index()).collect();
    let result_faces = solid_faces(topo, result)?;
    let outputs: std::collections::BTreeSet<_> = map.values().map(|face| face.index()).collect();
    let expected_outputs: std::collections::BTreeSet<_> =
        result_faces.iter().map(|face| face.index()).collect();
    if map
        .keys()
        .copied()
        .collect::<std::collections::BTreeSet<_>>()
        != source
        || outputs != expected_outputs
        || map.len() != result_faces.len()
    {
        return Err(remus_offset::OffsetError::TopologyChange {
            face: source_faces.first().copied(),
            edge: None,
            reason: "move-face construction map is not total and one-to-one".into(),
        }
        .into());
    }

    let mut evolution = EvolutionMap::exact();
    for (input, output) in map {
        evolution.add_modified(input, output.index());
    }
    Ok(evolution)
}

/// Validate and translate the Phase 4.3 cylindrical move contract.
///
/// The lower-layer move engine preserves an existing adjacency graph and is
/// intentionally independent of booleans. Radial edits reuse the established
/// exact cylindrical resize construction here at L3, after refusing every
/// trim configuration that construction cannot preserve exactly.
fn cylindrical_bore_move_request(
    topo: &Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<Option<(FaceId, f64)>, crate::OperationsError> {
    let Some(&reference) = faces.first() else {
        return Ok(None);
    };
    let source_faces = solid_faces(topo, solid)?;
    let source_face_indices: std::collections::HashSet<_> =
        source_faces.iter().map(|face| face.index()).collect();
    if !source_face_indices.contains(&reference.index()) {
        return Err(remus_offset::OffsetError::FaceNotInSolid {
            face: reference,
            solid,
        }
        .into());
    }

    let reference_data = topo.face(reference)?;
    let FaceSurface::Cylinder(cylinder) = reference_data.surface() else {
        return Ok(None);
    };
    let cylinder = cylinder.clone();
    let tolerance = Tolerance::new();
    if !distance.is_finite() || distance.abs() <= tolerance.linear {
        return Err(remus_offset::OffsetError::InvalidInput {
            reason: "move-face distance must be non-zero and finite".into(),
        }
        .into());
    }

    let mut selected = std::collections::HashSet::with_capacity(faces.len());
    for &face in faces {
        if !source_face_indices.contains(&face.index()) {
            return Err(remus_offset::OffsetError::FaceNotInSolid { face, solid }.into());
        }
        if !selected.insert(face.index()) {
            return Err(remus_offset::OffsetError::InvalidInput {
                reason: format!(
                    "move-face selection contains face {} more than once",
                    face.index()
                ),
            }
            .into());
        }
    }
    if let Some(&face) = faces.get(1) {
        return Err(remus_offset::OffsetError::MoveGroupMismatch {
            reference,
            face,
            reason: "a radial move requires exactly one cylindrical bore face".into(),
        }
        .into());
    }
    if !reference_data.is_reversed() {
        return Err(remus_offset::OffsetError::UnsupportedMoveFace {
            face: reference,
            surface_type: reference_data.surface().type_tag(),
            reason: "radial move supports inward-facing bore walls only".into(),
        }
        .into());
    }
    if !reference_data.inner_wires().is_empty() {
        return Err(remus_offset::OffsetError::UnsupportedMoveFace {
            face: reference,
            surface_type: reference_data.surface().type_tag(),
            reason: "a moved bore wall cannot carry inner trim wires".into(),
        }
        .into());
    }

    validate_cylindrical_bore_boundary(topo, solid, reference, &cylinder)?;
    let new_radius = cylinder.radius() - distance;
    if new_radius <= tolerance.linear {
        return Err(remus_offset::OffsetError::TopologyChange {
            face: Some(reference),
            edge: None,
            reason: format!(
                "radial move collapses the bore: radius {} - distance {distance} <= {}",
                cylinder.radius(),
                tolerance.linear
            ),
        }
        .into());
    }
    Ok(Some((reference, new_radius)))
}

fn validate_cylindrical_bore_boundary(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    cylinder: &CylindricalSurface,
) -> Result<(), crate::OperationsError> {
    let edge_faces = remus_topology::explorer::edge_to_face_map(topo, solid)?;
    let face_data = topo.face(face)?;
    let wire = topo.wire(face_data.outer_wire())?;
    let mut visited = std::collections::HashSet::new();
    let mut rim_count = 0_usize;

    for oriented in wire.edges() {
        let edge = oriented.edge();
        if !visited.insert(edge.index()) {
            continue;
        }
        let adjacent = edge_faces.get(&edge.index()).ok_or_else(|| {
            remus_offset::OffsetError::TopologyChange {
                face: Some(face),
                edge: Some(edge),
                reason: "bore boundary edge has no solid adjacency record".into(),
            }
        })?;
        if adjacent.len() != 2 {
            return Err(remus_offset::OffsetError::TopologyChange {
                face: Some(face),
                edge: Some(edge),
                reason: format!(
                    "bore boundary edge has {} face uses, expected 2",
                    adjacent.len()
                ),
            }
            .into());
        }
        if adjacent[0] == adjacent[1] {
            continue;
        }

        let neighbor = adjacent
            .iter()
            .copied()
            .find(|candidate| *candidate != face)
            .ok_or_else(|| remus_offset::OffsetError::TopologyChange {
                face: Some(face),
                edge: Some(edge),
                reason: "bore rim does not identify a distinct support face".into(),
            })?;
        let neighbor_data = topo.face(neighbor)?;
        let FaceSurface::Plane { normal, .. } = neighbor_data.surface() else {
            return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                face: neighbor,
                surface_type: neighbor_data.surface().type_tag(),
                reason: format!("face bounds cylindrical bore rim edge {}", edge.index()),
            }
            .into());
        };
        if normal.dot(cylinder.axis()).abs() < 1.0 - Tolerance::new().angular {
            return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                face: neighbor,
                surface_type: neighbor_data.surface().type_tag(),
                reason: "bore support plane is not perpendicular to the cylinder axis".into(),
            }
            .into());
        }
        if !matches!(
            topo.edge(edge)?.curve(),
            remus_topology::edge::EdgeCurve::Circle(_)
        ) {
            return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                face,
                surface_type: face_data.surface().type_tag(),
                reason: format!("bore rim edge {} is not an exact circle", edge.index()),
            }
            .into());
        }
        rim_count += 1;
    }

    if rim_count != 2 {
        return Err(remus_offset::OffsetError::UnsupportedMoveFace {
            face,
            surface_type: face_data.surface().type_tag(),
            reason: format!("bore wall has {rim_count} supported circular rims, expected 2"),
        }
        .into());
    }
    Ok(())
}

fn refuse_swept_face_intersections(
    topo: &Topology,
    solid: SolidId,
    selected_faces: &[FaceId],
    distance: f64,
) -> Result<(), crate::OperationsError> {
    let Some(&reference) = selected_faces.first() else {
        return Ok(());
    };
    let Some(normal) = topo.face(reference)?.effective_plane_normal() else {
        return Ok(());
    };
    let selected: std::collections::HashSet<_> = selected_faces.iter().copied().collect();
    refuse_swept_region_intersections(
        topo,
        solid,
        &selected,
        &std::collections::HashSet::new(),
        normal,
        distance,
    )
}

pub(crate) fn refuse_swept_region_intersections(
    topo: &Topology,
    solid: SolidId,
    swept_faces: &std::collections::HashSet<FaceId>,
    excluded_candidates: &std::collections::HashSet<FaceId>,
    normal: Vec3,
    distance: f64,
) -> Result<(), crate::OperationsError> {
    let delta = normal * distance;
    let edge_faces = remus_topology::explorer::edge_to_face_map(topo, solid)?;
    let source_faces = solid_faces(topo, solid)?;

    for &moved_face in swept_faces {
        let mut adjacent = std::collections::HashSet::new();
        for faces in edge_faces.values() {
            if faces.contains(&moved_face) {
                adjacent.extend(faces.iter().copied());
            }
        }

        let original = crate::measure::face_set_bounding_box(topo, &[moved_face])?;
        let destination = Aabb3 {
            min: original.min + delta,
            max: original.max + delta,
        };
        let swept = original.union(destination);
        for &candidate in &source_faces {
            if swept_faces.contains(&candidate)
                || excluded_candidates.contains(&candidate)
                || adjacent.contains(&candidate)
            {
                continue;
            }
            let candidate_box = crate::measure::face_set_bounding_box(topo, &[candidate])?;
            if !original.intersects(candidate_box) && swept.intersects(candidate_box) {
                return Err(remus_offset::OffsetError::TopologyChange {
                    face: Some(moved_face),
                    edge: None,
                    reason: format!(
                        "swept face reaches nonadjacent face {} before completing the move",
                        candidate.index()
                    ),
                }
                .into());
            }
        }
    }
    Ok(())
}

pub(crate) fn move_is_prismatic(
    topo: &Topology,
    solid: SolidId,
    selected_faces: &[FaceId],
) -> Result<bool, crate::OperationsError> {
    let Some(&reference) = selected_faces.first() else {
        return Ok(false);
    };
    let Some(move_normal) = topo.face(reference)?.effective_plane_normal() else {
        return Ok(false);
    };
    let selected: std::collections::HashSet<_> =
        selected_faces.iter().map(|face| face.index()).collect();
    let edge_faces = remus_topology::explorer::edge_to_face_map(topo, solid)?;
    let tolerance = Tolerance::new();
    for faces in edge_faces.values() {
        if faces.len() != 2 || faces[0] == faces[1] {
            continue;
        }
        let first_selected = selected.contains(&faces[0].index());
        let second_selected = selected.contains(&faces[1].index());
        if first_selected == second_selected {
            continue;
        }
        let neighbor = if first_selected { faces[1] } else { faces[0] };
        let invariant = match topo.face(neighbor)?.surface() {
            FaceSurface::Plane { normal, .. } => normal.dot(move_normal).abs() <= tolerance.angular,
            FaceSurface::Cylinder(cylinder) => {
                cylinder.axis().dot(move_normal).abs() >= 1.0 - tolerance.angular
            }
            _ => false,
        };
        if !invariant {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Move a planar face of `solid` along its outward normal.
///
/// A positive `distance` adds material (the face is pulled outward), a
/// negative one removes it (the face is pushed into the solid). The tool is
/// extruded from the face itself, so inner wires are carried through and a
/// face with holes keeps them as holes.
///
/// Coplanar seams left where the tool meets the original solid are merged, so
/// pulling a face twice by 1 gives the same topology as pulling it once by 2.
///
/// # Errors
///
/// Returns an error if `distance` is zero or non-finite, the face is not part
/// of `solid`, the face is not planar, or the result's shell is not closed.
pub fn push_pull_face(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    distance: f64,
) -> Result<SolidId, crate::OperationsError> {
    let tol = Tolerance::new();

    if !distance.is_finite() {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("push/pull distance must be finite, got {distance}"),
        });
    }
    if distance.abs() <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("push/pull distance must be non-zero, got {distance}"),
        });
    }

    ensure_face_in_solid(topo, solid, face)?;

    if let Some(result) = push_pull_simple_cylinder_cap(topo, solid, face, distance)? {
        return Ok(result);
    }

    let face_data = topo.face(face)?;
    let normal =
        face_data
            .effective_plane_normal()
            .ok_or_else(|| crate::OperationsError::InvalidInput {
                reason: format!(
                    "push/pull requires a planar face, face {} is {}",
                    face.index(),
                    face_data.surface().type_tag()
                ),
            })?;

    // Extrude a COPY: `extrude` reuses the profile wire's edges for its bottom
    // cap, and a tool sharing edges with the operand it is cut from feeds the
    // boolean two solids that alias the same topology.
    let profile = copy_face(topo, face)?;

    // `extrude` walks the profile along `direction * distance`; give it the
    // outward normal for a pull and the inward one for a push, always with a
    // positive length, so the tool occupies the slab actually being added or
    // removed and stays flush with the face's own plane.
    let (direction, op) = if distance > 0.0 {
        (normal, BooleanOp::Fuse)
    } else {
        (-normal, BooleanOp::Cut)
    };
    let tool = extrude(topo, profile, direction, distance.abs())?;

    // A prismatic push/pull moves exactly `area * |distance|` of material, so
    // the result's volume is known before the boolean runs. Checking it is
    // what stops a silently-degraded result reaching the caller: a face whose
    // hole walls must merge with a coaxial wall already in the solid can come
    // back closed, correctly shaped at a glance, and short a bore.
    let area = crate::measure::face_area(topo, face, verify_deflection(topo, solid))?;
    let before = solid_volume(topo, solid, verify_deflection(topo, solid))?;
    let expected = distance.mul_add(area, before);

    let result = boolean(topo, op, solid, tool)?;
    unify_faces(topo, result)?;
    drop_stranded_inner_wires(topo, result)?;
    ensure_closed_shell(topo, result, "push/pull")?;
    ensure_volume(topo, result, expected, "push/pull")?;
    Ok(result)
}

/// Rebuild a three-face analytic cylinder when either cap moves.
///
/// The generic cut path can select the swept cap slab instead of the
/// remaining cylinder when the top cap moves inward. Restrict this exact
/// construction to a fully proved primitive topology; every decorated or
/// trimmed cylinder continues through the general push/pull path.
fn push_pull_simple_cylinder_cap(
    topo: &mut Topology,
    solid: SolidId,
    selected: FaceId,
    distance: f64,
) -> Result<Option<SolidId>, crate::OperationsError> {
    let faces = solid_faces(topo, solid)?;
    if faces.len() != 3 {
        return Ok(None);
    }

    let mut cylinder = None;
    let mut caps = Vec::with_capacity(2);
    for candidate in faces {
        let data = topo.face(candidate)?;
        match data.surface() {
            FaceSurface::Cylinder(surface)
                if cylinder.is_none() && !data.is_reversed() && data.inner_wires().is_empty() =>
            {
                cylinder = Some((candidate, surface.clone()));
            }
            FaceSurface::Plane { .. } if data.inner_wires().is_empty() => caps.push(candidate),
            _ => return Ok(None),
        }
    }
    let Some((cylinder_face, cylinder)) = cylinder else {
        return Ok(None);
    };
    if caps.len() != 2 || !caps.contains(&selected) {
        return Ok(None);
    }

    let axis = unit(cylinder.axis())?;
    let (base, height) = axial_extent(topo, cylinder_face, &cylinder)?;
    let scale = cylinder.radius().max(height).max(1.0);
    let tolerance = Tolerance::new();
    let match_tolerance = tolerance.linear.max(scale * 1e-9);

    let mut bottom = None;
    let mut top = None;
    for cap in caps {
        let Some(position) =
            simple_cylinder_cap_position(topo, cap, &cylinder, base, axis, match_tolerance)?
        else {
            return Ok(None);
        };
        if position.abs() <= match_tolerance && bottom.replace(cap).is_none() {
            continue;
        }
        if (position - height).abs() <= match_tolerance && top.replace(cap).is_none() {
            continue;
        }
        return Ok(None);
    }
    let (Some(bottom), Some(top)) = (bottom, top) else {
        return Ok(None);
    };

    let new_height = height + distance;
    if new_height <= tolerance.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "push/pull distance {distance} would collapse cylinder height {height}"
            ),
        });
    }

    let new_base = if selected == bottom {
        base - axis * distance
    } else if selected == top {
        base
    } else {
        return Ok(None);
    };
    let seam_direction = cylinder_seam_direction(topo, cylinder_face, &cylinder)?;
    place_cylinder(
        topo,
        new_base,
        axis,
        seam_direction,
        cylinder.radius(),
        new_height,
    )
    .map(Some)
}

/// Return a cap's axial coordinate from `base` when its topology proves that
/// it is the untrimmed circular boundary of `cylinder`.
fn simple_cylinder_cap_position(
    topo: &Topology,
    face: FaceId,
    cylinder: &CylindricalSurface,
    base: Point3,
    axis: Vec3,
    match_tolerance: f64,
) -> Result<Option<f64>, crate::OperationsError> {
    let data = topo.face(face)?;
    let Some(normal) = data.effective_plane_normal() else {
        return Ok(None);
    };
    if normal.dot(axis).abs() < 1.0 - Tolerance::new().angular {
        return Ok(None);
    }
    let wire = topo.wire(data.outer_wire())?;
    let [oriented] = wire.edges() else {
        return Ok(None);
    };
    let edge = topo.edge(oriented.edge())?;
    let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve() else {
        return Ok(None);
    };
    if edge.start() != edge.end()
        || (circle.radius() - cylinder.radius()).abs() > match_tolerance
        || circle.normal().dot(axis).abs() < 1.0 - Tolerance::new().angular
    {
        return Ok(None);
    }

    let center_offset = circle.center() - cylinder.origin();
    let radial_offset = center_offset - axis * center_offset.dot(axis);
    if radial_offset.length() > match_tolerance {
        return Ok(None);
    }
    Ok(Some((circle.center() - base).dot(axis)))
}

/// Change the radius of a cylindrical face of `solid`.
///
/// Works for both a bore (material outside the cylinder) and a boss (material
/// inside it); the concavity is read from the face's own orientation. The
/// cylinder's axial extent is taken from the face, so the caps at either end
/// are preserved and only the wall moves. Outward quarter walls with radial
/// planar sides and perpendicular planar caps are re-limited exactly; other
/// partial-wall configurations return a typed refusal.
///
/// # Errors
///
/// Returns an error if `new_radius` is not positive and finite, the face is
/// not part of `solid`, the face is not cylindrical, the new radius equals the
/// current one, or the result's shell is not closed.
pub fn resize_cylindrical_face(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    new_radius: f64,
) -> Result<SolidId, crate::OperationsError> {
    let tol = Tolerance::new();
    let cyl = validate_cylindrical_resize(topo, solid, face, new_radius)?;

    if !cylindrical_face_is_full_turn(topo, face, &cyl)? {
        return resize_partial_cylindrical_face(topo, solid, face, &cyl, new_radius);
    }

    let axis = unit(cyl.axis())?;
    if axis.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - tol.angular {
        return resize_cylindrical_face_aligned(topo, solid, face, new_radius, false)
            .map(|result| result.0.solid);
    }

    // The analytic boolean pipeline is most robust in its canonical +Z frame.
    // Rigidly normalize a copied operand, perform the exact edit there, then
    // return the result to world space. The face map keeps selection exact;
    // no geometric re-matching is involved.
    let (base, _) = axial_extent(topo, face, &cyl)?;
    let seam_direction = cylinder_seam_direction(topo, face, &cyl)?;
    let to_world = frame_matrix(base, axis, seam_direction)?;
    let to_local = inverse_rigid_frame(&to_world);
    let (local_solid, face_map) = copy_solid_with_face_map(topo, solid)?;
    let local_face_index = face_map.get(&face.index()).copied().ok_or_else(|| {
        crate::OperationsError::InvalidInput {
            reason: format!("copied solid lost cylindrical face {}", face.index()),
        }
    })?;
    let local_face = topo.face_id_from_index(local_face_index).ok_or_else(|| {
        crate::OperationsError::InvalidInput {
            reason: format!("copied cylindrical face {local_face_index} is unavailable"),
        }
    })?;
    transform_solid(topo, local_solid, &to_local)?;
    let result = resize_cylindrical_face_aligned(topo, local_solid, local_face, new_radius, false)?
        .0
        .solid;
    transform_solid(topo, result, &to_world)?;
    Ok(result)
}

fn validate_cylindrical_resize(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    new_radius: f64,
) -> Result<CylindricalSurface, crate::OperationsError> {
    let tol = Tolerance::new();
    if !new_radius.is_finite() || new_radius <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylinder radius must be positive, got {new_radius}"),
        });
    }
    ensure_face_in_solid(topo, solid, face)?;
    let face_data = topo.face(face)?;
    let FaceSurface::Cylinder(cyl) = face_data.surface() else {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "resize requires a cylindrical face, face {} is {}",
                face.index(),
                face_data.surface().type_tag()
            ),
        });
    };
    let cyl = cyl.clone();
    if (new_radius - cyl.radius()).abs() <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylinder radius is already {}", cyl.radius()),
        });
    }

    Ok(cyl)
}

pub(crate) fn resize_cylindrical_face_with_entity_evolution(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    new_radius: f64,
) -> Result<DirectEditEvolution, crate::OperationsError> {
    let cylinder = validate_cylindrical_resize(topo, solid, face, new_radius)?;
    let full_turn = cylindrical_face_is_full_turn(topo, face, &cylinder)?;
    let reversed = topo.face(face)?.is_reversed();
    if !full_turn && reversed {
        return Err(remus_offset::OffsetError::UnsupportedMoveFace {
            face, surface_type: "cylinder",
            reason: "partial-cylinder resize currently requires an outward quarter wall with radial planar sides".into(),
        }.into());
    }
    let source_faces = solid_faces(topo, solid)?;
    let mut replacement_carriers = true;
    for &source in &source_faces {
        replacement_carriers &= matches!(
            topo.face(source)?.surface(),
            FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
        );
    }
    if !full_turn || (reversed && replacement_carriers) {
        let (base, height) = axial_extent(topo, face, &cylinder)?;
        let before = solid_volume(topo, solid, verify_deflection(topo, solid))?;
        let fraction = if full_turn { 1.0 } else { 0.25 };
        let sleeve = fraction
            * PI
            * (new_radius * new_radius - cylinder.radius() * cylinder.radius())
            * height;
        let expected = if reversed {
            before - sleeve
        } else {
            before + sleeve
        };
        let replacement = CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            new_radius,
            cylinder.x_axis(),
        )?;
        let result = crate::replace_surface::replace_surface_with_entity_map(
            topo,
            solid,
            face,
            FaceSurface::Cylinder(replacement),
        )?;
        ensure_closed_shell(topo, result.solid, "journaled cylindrical resize")?;
        ensure_volume(topo, result.solid, expected, "journaled cylindrical resize")?;
        ensure_resized_cylinder(
            topo,
            result.solid,
            base,
            cylinder.axis(),
            height,
            cylinder.radius(),
            new_radius,
        )?;
        let pairs = boundary_entity_pairs(&result);
        return Ok((
            MoveFacesResult {
                solid: result.solid,
                evolution: exact_face_evolution(
                    topo,
                    &source_faces,
                    result.solid,
                    result.face_map,
                )?,
            },
            pairs,
        ));
    }
    let axis = unit(cylinder.axis())?;
    if axis.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - Tolerance::new().angular {
        return resize_cylindrical_face_aligned(topo, solid, face, new_radius, true);
    }
    let (base, _) = axial_extent(topo, face, &cylinder)?;
    let seam = cylinder_seam_direction(topo, face, &cylinder)?;
    let to_world = frame_matrix(base, axis, seam)?;
    let snapshot = topo.clone();
    let copied = crate::copy::copy_solid_between_with_entity_map(&snapshot, topo, solid)?;
    let local_face = *copied.face_map.get(&face.index()).ok_or_else(|| {
        crate::OperationsError::InvalidInput {
            reason: "cylindrical resize copy lost its selected face".into(),
        }
    })?;
    transform_solid(topo, copied.solid, &inverse_rigid_frame(&to_world))?;
    let (mut result, pairs) =
        resize_cylindrical_face_aligned(topo, copied.solid, local_face, new_radius, true)?;
    transform_solid(topo, result.solid, &to_world)?;
    let face_sources: HashMap<_, _> = copied
        .face_map
        .into_iter()
        .map(|(source, local)| (local.index(), source))
        .collect();
    result.evolution = remap_radius_history_sources(result.evolution, &face_sources)?;
    let boundary_sources: HashMap<_, _> =
        copied
            .edge_map
            .into_iter()
            .map(|(source, local)| (EntityKey::edge(local.index()), EntityKey::edge(source)))
            .chain(copied.vertex_map.into_iter().map(|(source, local)| {
                (EntityKey::vertex(local.index()), EntityKey::vertex(source))
            }))
            .collect();
    let pairs = pairs
        .into_iter()
        .map(|(local, target)| {
            boundary_sources
                .get(&local)
                .copied()
                .map(|source| (source, target))
                .ok_or_else(|| crate::OperationsError::InvalidInput {
                    reason: "cylindrical resize history names a boundary outside its copied source"
                        .into(),
                })
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok((result, pairs))
}

fn remap_radius_history_sources(
    history: EvolutionMap,
    sources: &HashMap<usize, usize>,
) -> Result<EvolutionMap, crate::OperationsError> {
    let original = |source| {
        sources
            .get(&source)
            .copied()
            .ok_or_else(|| crate::OperationsError::InvalidInput {
                reason: "cylindrical resize history names a face outside its copied source".into(),
            })
    };
    let mut mapped = EvolutionMap::exact();
    mapped.origin = history.origin;
    for (source, outputs) in history.modified {
        for output in outputs {
            mapped.add_modified(original(source)?, output);
        }
    }
    for (source, outputs) in history.generated {
        for output in outputs {
            mapped.add_generated(original(source)?, output);
        }
    }
    for source in history.deleted {
        mapped.add_deleted(original(source)?);
    }
    for (output, candidates) in history.unresolved {
        mapped.add_unresolved(
            output,
            candidates
                .into_iter()
                .filter_map(|source| sources.get(&source).copied())
                .collect(),
        );
    }
    Ok(mapped)
}

fn cylindrical_face_is_full_turn(
    topo: &Topology,
    face: FaceId,
    cylinder: &remus_math::surfaces::CylindricalSurface,
) -> Result<bool, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;
    let tolerance = Tolerance::new();
    let face = topo.face(face)?;
    let mut rims: Vec<(f64, f64, f64)> = Vec::new();
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            match edge.curve() {
                EdgeCurve::Line => {
                    let direction =
                        topo.vertex(edge.end())?.point() - topo.vertex(edge.start())?.point();
                    if direction.cross(cylinder.axis()).length() > tolerance.linear {
                        return Ok(false);
                    }
                }
                EdgeCurve::Circle(circle) => {
                    let delta = circle.center() - cylinder.origin();
                    let level = delta.dot(cylinder.axis());
                    if circle.normal().cross(cylinder.axis()).length() > tolerance.angular
                        || (delta - cylinder.axis() * level).length() > tolerance.linear
                        || (circle.radius() - cylinder.radius()).abs() > tolerance.linear
                    {
                        return Ok(false);
                    }
                    let span = if let Some((lo, hi)) = edge.trim() {
                        hi - lo
                    } else if edge.start() == edge.end() {
                        std::f64::consts::TAU
                    } else {
                        return Ok(false);
                    };
                    let winding = span
                        * circle
                            .u_axis()
                            .cross(circle.v_axis())
                            .dot(cylinder.axis())
                            .signum()
                        * if oriented.is_forward() { 1.0 } else { -1.0 };
                    if let Some(rim) = rims
                        .iter_mut()
                        .find(|rim| (rim.0 - level).abs() <= tolerance.linear)
                    {
                        rim.1 += winding;
                        rim.2 += span.abs();
                    } else {
                        rims.push((level, winding, span.abs()));
                    }
                }
                _ => return Ok(false),
            }
        }
    }
    Ok(rims.len() == 2
        && rims.iter().all(|rim| {
            (rim.1.abs() - std::f64::consts::TAU).abs() <= tolerance.angular
                && (rim.2 - std::f64::consts::TAU).abs() <= tolerance.angular
        }))
}

fn resize_partial_cylindrical_face(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    cylinder: &remus_math::surfaces::CylindricalSurface,
    new_radius: f64,
) -> Result<SolidId, crate::OperationsError> {
    if topo.face(face)?.is_reversed() {
        return Err(remus_offset::OffsetError::UnsupportedMoveFace {
            face,
            surface_type: "cylinder",
            reason: "partial-cylinder resize currently requires an outward quarter wall with radial planar sides".into(),
        }.into());
    }
    let (base, height) = axial_extent(topo, face, cylinder)?;
    let before = solid_volume(topo, solid, verify_deflection(topo, solid))?;
    let replacement = remus_math::surfaces::CylindricalSurface::with_ref_dir(
        cylinder.origin(),
        cylinder.axis(),
        new_radius,
        cylinder.x_axis(),
    )?;
    remus_topology::transaction::run_transacted(topo, |topo| -> Result<_, crate::OperationsError> {
        // The replacement layer certifies radial quarter-sector supports and
        // clearance before rebuilding their shared edges and cap boundaries.
        let result = crate::replace_surface::replace_surface(
            topo,
            solid,
            face,
            FaceSurface::Cylinder(replacement),
        )?
        .solid;
        let expected = before
            + 0.25
                * PI
                * (new_radius * new_radius - cylinder.radius() * cylinder.radius())
                * height;
        ensure_closed_shell(topo, result, "partial cylindrical resize")?;
        ensure_volume(topo, result, expected, "partial cylindrical resize")?;
        ensure_resized_cylinder(
            topo,
            result,
            base,
            cylinder.axis(),
            height,
            cylinder.radius(),
            new_radius,
        )?;
        Ok(result)
    })
}

/// [`resize_cylindrical_face`] after any required rigid normalization has put
/// the selected cylinder on the canonical +Z axis.
fn resize_cylindrical_face_aligned(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    new_radius: f64,
    track_history: bool,
) -> Result<DirectEditEvolution, crate::OperationsError> {
    let tol = Tolerance::new();

    if !new_radius.is_finite() || new_radius <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylinder radius must be positive, got {new_radius}"),
        });
    }

    ensure_face_in_solid(topo, solid, face)?;

    let face_data = topo.face(face)?;
    let FaceSurface::Cylinder(cyl) = face_data.surface() else {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "resize requires a cylindrical face, face {} is {}",
                face.index(),
                face_data.surface().type_tag()
            ),
        });
    };
    let cyl = cyl.clone();
    let old_radius = cyl.radius();
    let reversed = face_data.is_reversed();

    if (new_radius - old_radius).abs() <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylinder radius is already {old_radius}"),
        });
    }

    // A cylindrical surface's natural normal points away from the axis. When
    // the face is reversed the solid's outward normal points AT the axis, so
    // the material is outside the cylinder — a bore.
    let concavity = if reversed {
        Concavity::Hole
    } else {
        Concavity::Boss
    };

    let (base, height) = axial_extent(topo, face, &cyl)?;
    if height <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylindrical face {} has no axial extent", face.index()),
        });
    }

    let axis = unit(cyl.axis())?;
    let seam_direction = cylinder_seam_direction(topo, face, &cyl)?;
    let grows = new_radius > old_radius;
    let before = solid_volume(topo, solid, verify_deflection(topo, solid))?;
    // Sweeping the wall outward adds material on a boss and removes it from a
    // bore; inward does the reverse. The magnitude is the annular sleeve
    // between the two radii over the face's own extent.
    let sleeve = PI * (new_radius * new_radius - old_radius * old_radius) * height;
    let expected = if concavity == Concavity::Boss {
        before + sleeve
    } else {
        before - sleeve
    };

    // Growing the wall sweeps it into open space, shrinking it sweeps back
    // through material already there. Either way the material that moves is the
    // annular sleeve between the two radii over the face's own extent — a plain
    // cylinder when growing (the sleeve's inner radius is the axis), a tube
    // when shrinking. Only whether it is added or removed changes.
    let (op, tool) = match (concavity, grows) {
        (Concavity::Boss, true) => (
            BooleanOp::Fuse,
            place_cylinder(topo, base, axis, seam_direction, new_radius, height)?,
        ),
        (Concavity::Hole, true) => (
            BooleanOp::Cut,
            place_cylinder(topo, base, axis, seam_direction, new_radius, height)?,
        ),
        (Concavity::Hole, false) => (
            BooleanOp::Fuse,
            make_tube(
                topo,
                base,
                axis,
                seam_direction,
                new_radius,
                old_radius,
                height,
            )?,
        ),
        (Concavity::Boss, false) => (
            BooleanOp::Cut,
            make_tube(
                topo,
                base,
                axis,
                seam_direction,
                new_radius,
                old_radius,
                height,
            )?,
        ),
    };

    let source_faces = if track_history {
        solid_faces(topo, solid)?
    } else {
        Vec::new()
    };
    let tool_sources = if track_history {
        radius_tool_face_sources(topo, solid, face, tool, new_radius)?
    } else {
        HashMap::new()
    };
    let (result, boolean_history) = if track_history {
        crate::boolean::boolean_with_evolution(topo, op, solid, tool)?
    } else {
        (boolean(topo, op, solid, tool)?, EvolutionMap::exact())
    };
    let unification = crate::heal::unify_faces_with_history(topo, result)?;
    drop_stranded_inner_wires(topo, result)?;
    ensure_closed_shell(topo, result, "cylindrical resize")?;
    repair_resized_cylinder_rim_orientation(topo, result, base, axis, height, new_radius)?;
    ensure_volume(topo, result, expected, "cylindrical resize")?;
    ensure_resized_cylinder(topo, result, base, axis, height, old_radius, new_radius)?;
    let evolution = if track_history {
        radius_face_history(
            topo,
            &source_faces,
            &tool_sources,
            result,
            &boolean_history,
            &unification.modified,
        )?
    } else {
        EvolutionMap::exact()
    };
    let boundary_pairs = if track_history {
        radius_boundary_pairs(topo, solid, result, &evolution)?
    } else {
        Vec::new()
    };
    Ok((
        MoveFacesResult {
            solid: result,
            evolution,
        },
        boundary_pairs,
    ))
}

// A complete boundary correspondence proves that unmatched internal cap
// boundaries were introduced by subdivision, rather than replacing old edges.
pub(crate) fn radius_subdivision_outputs(
    topo: &Topology,
    source: SolidId,
    result: SolidId,
    history: &EvolutionMap,
    pairs: &[(EntityKey, EntityKey)],
) -> Result<Vec<(EntityKey, remus_topology::journal::EventDraft)>, crate::OperationsError> {
    use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_vertices};
    use remus_topology::journal::EventDraft;
    let mapped_sources: HashSet<_> = pairs.iter().map(|&(source, _)| source).collect();
    let Some(supports) = radius_support_groups(topo, source, history)? else {
        return Ok(Vec::new());
    };
    if pairs.is_empty() {
        return Ok(Vec::new());
    }
    let source_adjacency = edge_to_face_map(topo, source)?;
    let mut retired = Vec::new();
    let mut retired_edges = HashSet::new();
    let mut source_vertex_edges = BTreeMap::<usize, Vec<usize>>::new();
    for edge in solid_edges(topo, source)? {
        let data = topo.edge(edge)?;
        for vertex in [data.start(), data.end()] {
            source_vertex_edges
                .entry(vertex.index())
                .or_default()
                .push(edge.index());
        }
        if mapped_sources.contains(&EntityKey::edge(edge.index())) {
            continue;
        }
        let faces: HashSet<_> = source_adjacency[&edge.index()]
            .iter()
            .map(|face| face.index())
            .collect();
        let groups: HashSet<_> = faces.iter().map(|face| supports[face]).collect();
        if faces.len() != 2
            || groups.len() != 1
            || !faces.iter().any(|face| history.deleted.contains(face))
        {
            return Ok(Vec::new());
        }
        retired_edges.insert(edge.index());
        retired.push((EntityKey::edge(edge.index()), EventDraft::Deleted));
    }
    for vertex in solid_vertices(topo, source)? {
        if mapped_sources.contains(&EntityKey::vertex(vertex.index())) {
            continue;
        }
        if !source_vertex_edges
            .get(&vertex.index())
            .is_some_and(|edges| edges.iter().all(|edge| retired_edges.contains(edge)))
        {
            return Ok(Vec::new());
        }
        retired.push((EntityKey::vertex(vertex.index()), EventDraft::Deleted));
    }
    let mut parents = HashMap::new();
    for (&source, images) in &history.modified {
        for &image in images {
            if parents.insert(image, source).is_some() {
                return Ok(Vec::new());
            }
        }
    }
    let mapped_targets: HashSet<_> = pairs.iter().map(|&(_, target)| target).collect();
    let adjacency = edge_to_face_map(topo, result)?;
    let mut generated_edges = BTreeMap::new();
    let mut vertex_edges = BTreeMap::<usize, Vec<usize>>::new();
    for edge in solid_edges(topo, result)? {
        let data = topo.edge(edge)?;
        for vertex in [data.start(), data.end()] {
            vertex_edges
                .entry(vertex.index())
                .or_default()
                .push(edge.index());
        }
        if mapped_targets.contains(&EntityKey::edge(edge.index())) {
            continue;
        }
        let faces: HashSet<_> = adjacency[&edge.index()]
            .iter()
            .map(|id| id.index())
            .collect();
        if faces.len() != 2 {
            continue;
        }
        let Some(sources) = faces
            .iter()
            .map(|face| parents.get(face).copied())
            .collect::<Option<HashSet<_>>>()
        else {
            continue;
        };
        if sources.len() == 1 {
            generated_edges.insert(edge.index(), sources.into_iter().collect::<Vec<_>>());
        }
    }
    let mut outputs: Vec<_> = generated_edges
        .iter()
        .map(|(&edge, sources)| {
            (
                EntityKey::edge(edge),
                EventDraft::Generated {
                    sources: sources.iter().copied().map(EntityKey::face).collect(),
                },
            )
        })
        .collect();
    for (vertex, edges) in vertex_edges {
        if mapped_targets.contains(&EntityKey::vertex(vertex)) {
            continue;
        }
        if edges.iter().all(|edge| generated_edges.contains_key(edge)) {
            let mut sources: Vec<_> = edges
                .iter()
                .flat_map(|edge| generated_edges[edge].iter().copied())
                .collect();
            sources.sort_unstable();
            sources.dedup();
            outputs.push((
                EntityKey::vertex(vertex),
                EventDraft::Generated {
                    sources: sources.into_iter().map(EntityKey::face).collect(),
                },
            ));
        }
    }
    outputs.extend(retired);
    Ok(outputs)
}

// Deleted cap subdivisions may be absorbed into their one adjacent planar
// support. This groups boundary roles only; the deleted face stays Deleted.
fn radius_support_groups(
    topo: &Topology,
    source: SolidId,
    history: &EvolutionMap,
) -> Result<Option<HashMap<usize, usize>>, crate::OperationsError> {
    let faces = solid_faces(topo, source)?;
    if !history.origin.is_exact()
        || !history.is_complete()
        || history.modified.len() + history.deleted.len() != faces.len()
    {
        return Ok(None);
    }
    let adjacency = remus_topology::explorer::edge_to_face_map(topo, source)?;
    let tolerance = Tolerance::new();
    let mut groups = HashMap::new();
    for face in faces {
        if history
            .modified
            .get(&face.index())
            .is_some_and(|images| !images.is_empty())
        {
            groups.insert(face.index(), face.index());
            continue;
        }
        if !history.deleted.contains(&face.index()) {
            return Ok(None);
        }
        let FaceSurface::Plane { normal, d } = topo.face(face)?.surface() else {
            return Ok(None);
        };
        let mut candidates = HashSet::new();
        for neighbor in adjacency
            .values()
            .filter(|uses| uses.contains(&face))
            .flatten()
        {
            if !history.modified.contains_key(&neighbor.index()) {
                continue;
            }
            let FaceSurface::Plane {
                normal: other,
                d: other_d,
            } = topo.face(*neighbor)?.surface()
            else {
                continue;
            };
            let length = normal.length();
            let other_length = other.length();
            if length <= f64::EPSILON || other_length <= f64::EPSILON {
                continue;
            }
            let dot = (*normal * (1.0 / length)).dot(*other * (1.0 / other_length));
            if (dot.abs() - 1.0).abs() <= tolerance.angular
                && (d / length - other_d / other_length * dot.signum()).abs() <= tolerance.linear
            {
                candidates.insert(neighbor.index());
            }
        }
        if candidates.len() != 1 {
            return Ok(None);
        }
        for candidate in candidates {
            groups.insert(face.index(), candidate);
        }
    }
    Ok(Some(groups))
}

// A cap may be rebuilt as several faces. Internal edges between those
// construction images are new subdivisions, not the source wall boundary.
// Require a unique boundary/vertex map and equal oriented aggregate loops.
#[allow(clippy::too_many_lines)]
fn radius_boundary_pairs(
    topo: &Topology,
    source: SolidId,
    result: SolidId,
    history: &EvolutionMap,
) -> Result<Vec<(EntityKey, EntityKey)>, crate::OperationsError> {
    use remus_topology::explorer::{edge_to_face_map, solid_edges};
    let source_faces = solid_faces(topo, source)?;
    let Some(supports) = radius_support_groups(topo, source, history)? else {
        return Ok(Vec::new());
    };
    let mut face_sources = HashMap::new();
    for (&source, images) in &history.modified {
        for &image in images {
            if face_sources
                .insert(image, supports[&source])
                .is_some_and(|old| old != supports[&source])
            {
                return Ok(Vec::new());
            }
        }
    }
    if solid_faces(topo, result)?
        .iter()
        .any(|face| !face_sources.contains_key(&face.index()))
    {
        return Ok(Vec::new());
    }
    let old_adjacency = edge_to_face_map(topo, source)?;
    let new_adjacency = edge_to_face_map(topo, result)?;
    let mut groups = BTreeMap::<Vec<usize>, Vec<usize>>::new();
    for (&edge, faces) in &new_adjacency {
        let Some(mut key) = faces
            .iter()
            .map(|face| face_sources.get(&face.index()).copied())
            .collect::<Option<Vec<_>>>()
        else {
            return Ok(Vec::new());
        };
        key.sort_unstable();
        key.dedup();
        groups.entry(key).or_default().push(edge);
    }
    let mut edge_map = HashMap::new();
    let mut used = HashSet::new();
    for (&edge, faces) in &old_adjacency {
        let mut key: Vec<_> = faces.iter().map(|face| supports[&face.index()]).collect();
        let distinct: HashSet<_> = faces.iter().map(|face| face.index()).collect();
        if distinct.len() > 1 && key.iter().all(|parent| *parent == key[0]) {
            continue;
        }
        key.sort_unstable();
        key.dedup();
        let Some(candidates) = groups.get(&key) else {
            return Ok(Vec::new());
        };
        let [target] = candidates.as_slice() else {
            return Ok(Vec::new());
        };
        if !used.insert(*target) {
            return Ok(Vec::new());
        }
        edge_map.insert(edge, *target);
    }
    let mut new_vertices = BTreeMap::<usize, Vec<usize>>::new();
    for edge in solid_edges(topo, result)? {
        if !used.contains(&edge.index()) {
            continue;
        }
        let data = topo.edge(edge)?;
        for vertex in [data.start(), data.end()] {
            new_vertices
                .entry(vertex.index())
                .or_default()
                .push(edge.index());
        }
    }
    let mut vertex_groups = BTreeMap::<Vec<usize>, Vec<usize>>::new();
    for (vertex, mut edges) in new_vertices {
        edges.sort_unstable();
        vertex_groups.entry(edges).or_default().push(vertex);
    }
    let mut old_vertices = BTreeMap::<usize, Vec<usize>>::new();
    for edge in solid_edges(topo, source)? {
        let Some(&mapped) = edge_map.get(&edge.index()) else {
            continue;
        };
        let data = topo.edge(edge)?;
        for vertex in [data.start(), data.end()] {
            old_vertices.entry(vertex.index()).or_default().push(mapped);
        }
    }
    let mut vertex_map = HashMap::new();
    let mut used_vertices = HashSet::new();
    for (vertex, mut edges) in old_vertices {
        edges.sort_unstable();
        let Some(candidates) = vertex_groups.get(&edges) else {
            return Ok(Vec::new());
        };
        let [target] = candidates.as_slice() else {
            return Ok(Vec::new());
        };
        if !used_vertices.insert(*target) {
            return Ok(Vec::new());
        }
        vertex_map.insert(vertex, *target);
    }
    if used_vertices.len() != vertex_groups.values().map(Vec::len).sum::<usize>() {
        return Ok(Vec::new());
    }
    let support_ids: HashSet<_> = supports.values().copied().collect();
    for support in support_ids {
        let mut expected = Vec::new();
        for &source_face in source_faces
            .iter()
            .filter(|face| supports[&face.index()] == support)
        {
            let before = topo.face(source_face)?;
            for wire in
                std::iter::once(before.outer_wire()).chain(before.inner_wires().iter().copied())
            {
                for oriented in topo.wire(wire)?.edges() {
                    let Some(&mapped_edge) = edge_map.get(&oriented.edge().index()) else {
                        continue;
                    };
                    let edge = topo.edge(oriented.edge())?;
                    let (start, end) = if oriented.is_forward() ^ before.is_reversed() {
                        (edge.start(), edge.end())
                    } else {
                        (edge.end(), edge.start())
                    };
                    expected.push((
                        mapped_edge,
                        vertex_map[&start.index()],
                        vertex_map[&end.index()],
                    ));
                }
            }
        }
        let mut actual = Vec::new();
        for (&image, _) in face_sources
            .iter()
            .filter(|(_, parent)| **parent == support)
        {
            let Some(id) = topo.face_id_from_index(image) else {
                return Ok(Vec::new());
            };
            let face = topo.face(id)?;
            for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oriented in topo.wire(wire)?.edges() {
                    let adjacent = &new_adjacency[&oriented.edge().index()];
                    let distinct: HashSet<_> = adjacent.iter().map(|face| face.index()).collect();
                    if distinct.len() > 1
                        && distinct
                            .iter()
                            .all(|face| face_sources.get(face) == Some(&support))
                    {
                        continue;
                    }
                    let edge = topo.edge(oriented.edge())?;
                    let (start, end) = if oriented.is_forward() ^ face.is_reversed() {
                        (edge.start(), edge.end())
                    } else {
                        (edge.end(), edge.start())
                    };
                    actual.push((oriented.edge().index(), start.index(), end.index()));
                }
            }
        }
        expected.sort_unstable();
        actual.sort_unstable();
        if expected != actual {
            return Ok(Vec::new());
        }
    }
    Ok(edge_map
        .into_iter()
        .map(|(from, to)| (EntityKey::edge(from), EntityKey::edge(to)))
        .chain(
            vertex_map
                .into_iter()
                .map(|(from, to)| (EntityKey::vertex(from), EntityKey::vertex(to))),
        )
        .collect())
}

// Tool caps extend the selected wall's existing planar supports. Their
// source is fixed by that adjacency and the tool's construction plane, not
// by matching finished result faces. Multiple eligible supports stay unknown.
fn radius_tool_face_sources(
    topo: &Topology,
    solid: SolidId,
    selected: FaceId,
    tool: SolidId,
    new_radius: f64,
) -> Result<HashMap<usize, FaceId>, crate::OperationsError> {
    let tolerance = Tolerance::new();
    let adjacency = remus_topology::explorer::edge_to_face_map(topo, solid)?;
    let supports: HashSet<_> = adjacency
        .values()
        .filter(|faces| faces.contains(&selected))
        .flatten()
        .copied()
        .filter(|&face| face != selected)
        .collect();
    let mut origins = HashMap::new();
    for face in solid_faces(topo, tool)? {
        match topo.face(face)?.surface() {
            FaceSurface::Cylinder(cylinder)
                if (cylinder.radius() - new_radius).abs() <= tolerance.linear =>
            {
                origins.insert(face.index(), selected);
            }
            FaceSurface::Plane { normal, d } => {
                let mut candidates = Vec::new();
                for &support in &supports {
                    let FaceSurface::Plane {
                        normal: support_normal,
                        d: support_d,
                    } = topo.face(support)?.surface()
                    else {
                        continue;
                    };
                    let length = normal.length();
                    let support_length = support_normal.length();
                    if length <= f64::EPSILON || support_length <= f64::EPSILON {
                        continue;
                    }
                    let dot =
                        (*normal * (1.0 / length)).dot(*support_normal * (1.0 / support_length));
                    if (dot.abs() - 1.0).abs() <= tolerance.angular
                        && (d / length - support_d / support_length * dot.signum()).abs()
                            <= tolerance.linear
                    {
                        candidates.push(support);
                    }
                }
                if let [support] = candidates.as_slice() {
                    origins.insert(face.index(), *support);
                }
            }
            _ => {}
        }
    }
    Ok(origins)
}

// The new-radius wall is a named construction role of the tool made above.
// Compose only the boolean's construction claims and the actual unify groups;
// a geometric fallback does not establish persistent source identities.
fn radius_face_history(
    topo: &Topology,
    source_faces: &[FaceId],
    tool_sources: &HashMap<usize, FaceId>,
    result: SolidId,
    boolean_history: &EvolutionMap,
    unified: &[(FaceId, FaceId)],
) -> Result<EvolutionMap, crate::OperationsError> {
    let result_faces = solid_faces(topo, result)?;
    let live: HashSet<_> = result_faces.iter().map(|face| face.index()).collect();
    let remap: HashMap<_, _> = unified
        .iter()
        .map(|(before, after)| (before.index(), after.index()))
        .collect();
    let mut history = EvolutionMap::exact();
    let mut claimed = HashSet::new();
    if boolean_history.origin.is_exact() {
        for &source in source_faces {
            let inputs = std::iter::once(source.index())
                .chain(
                    tool_sources
                        .iter()
                        .filter_map(|(&tool, &support)| (support == source).then_some(tool)),
                )
                .collect::<Vec<_>>();
            let mut outputs = inputs
                .iter()
                .filter_map(|input| boolean_history.modified.get(input))
                .flatten()
                .filter_map(|output| {
                    let final_face = remap.get(output).copied().unwrap_or(*output);
                    live.contains(&final_face).then_some(final_face)
                })
                .collect::<Vec<_>>();
            outputs.sort_unstable();
            outputs.dedup();
            if outputs.is_empty() && boolean_history.deleted.contains(&source.index()) {
                history.add_deleted(source.index());
            }
            for output in outputs {
                history.add_modified(source.index(), output);
                claimed.insert(output);
            }
        }
    }
    for face in result_faces {
        if !claimed.contains(&face.index()) {
            history.add_unresolved(face.index(), Vec::new());
        }
    }
    Ok(history)
}

/// Require the resized wall to remain exact analytic cylinder geometry.
///
/// Volume and shell closure do not distinguish a cylinder from a faceted
/// boolean fallback. Accept multiple coaxial bands when their union covers the
/// selected wall's full axial span, but reject an old-radius band that still
/// overlaps that span.
fn ensure_resized_cylinder(
    topo: &Topology,
    solid: SolidId,
    base: Point3,
    axis: Vec3,
    height: f64,
    old_radius: f64,
    new_radius: f64,
) -> Result<(), crate::OperationsError> {
    let tol = Tolerance::new();
    let axis = unit(axis)?;
    let model_scale = [
        base.x().abs(),
        base.y().abs(),
        base.z().abs(),
        height.abs(),
        old_radius.abs(),
        new_radius.abs(),
    ]
    .into_iter()
    .fold(1.0_f64, f64::max);
    let linear_tol = tol.linear.max(model_scale * tol.relative);
    let mut requested = Vec::new();
    let mut seen = Vec::new();

    for fid in solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        let FaceSurface::Cylinder(candidate) = face.surface() else {
            continue;
        };
        seen.push((candidate.radius(), candidate.origin(), candidate.axis()));
        let candidate_axis = unit(candidate.axis())?;
        if candidate_axis.dot(axis).abs() < 1.0 - tol.angular {
            continue;
        }
        let origin_offset = candidate.origin() - base;
        let perpendicular = origin_offset - axis * origin_offset.dot(axis);
        if perpendicular.length() > linear_tol {
            continue;
        }

        let (candidate_base, candidate_height) = axial_extent(topo, fid, candidate)?;
        let candidate_end = candidate_base + candidate_axis * candidate_height;
        let t0 = (candidate_base - base).dot(axis);
        let t1 = (candidate_end - base).dot(axis);
        let interval = (t0.min(t1), t0.max(t1));
        let overlap = interval.1.min(height) - interval.0.max(0.0);
        if overlap <= linear_tol {
            continue;
        }

        if tol.approx_eq(candidate.radius(), old_radius) {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!(
                    "cylindrical resize left the old radius {old_radius} over the edited span"
                ),
            });
        }
        if tol.approx_eq(candidate.radius(), new_radius) {
            requested.push(interval);
        }
    }

    requested.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut covered = 0.0;
    for &(lo, hi) in &requested {
        if lo > covered + linear_tol {
            break;
        }
        covered = covered.max(hi);
        if covered >= height - linear_tol {
            return Ok(());
        }
    }

    Err(crate::OperationsError::InvalidInput {
        reason: format!(
            "cylindrical resize did not preserve an analytic radius {new_radius} wall over height {height}; coaxial spans: {requested:?}; cylinders: {seen:?}"
        ),
    })
}

/// The tube between `inner_r` and `outer_r` over the wall's axial span.
///
/// The bore is overshot at both ends so its caps never land on the outer
/// cylinder's: coincident caps would make the difference a coplanar-face
/// boolean for no benefit. The tube's own caps stay flush with the wall being
/// replaced, so the sleeve covers exactly the material that moves.
fn make_tube(
    topo: &mut Topology,
    base: Point3,
    axis: Vec3,
    x_axis: Vec3,
    inner_r: f64,
    outer_r: f64,
    height: f64,
) -> Result<SolidId, crate::OperationsError> {
    let outer = place_cylinder(topo, base, axis, x_axis, outer_r, height)?;
    let overshoot = (height * 0.1).max(1e-3);
    let inner = place_cylinder(
        topo,
        base - unit(axis)? * overshoot,
        axis,
        x_axis,
        inner_r,
        overshoot.mul_add(2.0, height),
    )?;
    boolean(topo, BooleanOp::Cut, outer, inner)
}

/// A deflection fine enough that the volume check resolves the sleeve.
fn verify_deflection(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::solid_bounding_box(topo, solid).map_or(0.01, |bb| {
        ((bb.max - bb.min).length() * 5e-4).clamp(1e-4, 0.05)
    })
}

/// Repair a reversed closed rim on the cylinder created by a resize.
///
/// A closed circle has the same start and end vertex, so reversing its local
/// wire use cannot disconnect the wire or move geometry. Keep this repair
/// deliberately narrower than a general orientation healer: only a same-sense
/// edge on the requested new-radius cylinder is eligible, and any other shell
/// orientation defect still fails closed.
fn repair_resized_cylinder_rim_orientation(
    topo: &mut Topology,
    solid: SolidId,
    base: Point3,
    axis: Vec3,
    height: f64,
    new_radius: f64,
) -> Result<usize, crate::OperationsError> {
    use std::collections::HashMap;

    #[derive(Clone, Copy)]
    struct EdgeUse {
        face: FaceId,
        wire: remus_topology::wire::WireId,
        position: usize,
        stored_forward: bool,
        effective_forward: bool,
    }

    let shell_id = topo.solid(solid)?.outer_shell();
    let face_ids = topo.shell(shell_id)?.faces().to_vec();
    let axis = unit(axis)?;
    let model_scale = [
        base.x().abs(),
        base.y().abs(),
        base.z().abs(),
        height.abs(),
        new_radius.abs(),
    ]
    .into_iter()
    .fold(1.0_f64, f64::max);
    let linear_tol = Tolerance::new()
        .linear
        .max(model_scale * Tolerance::new().relative);

    let mut resized_faces = Vec::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let FaceSurface::Cylinder(candidate) = face.surface() else {
            continue;
        };
        if (candidate.radius() - new_radius).abs() > linear_tol {
            continue;
        }
        let candidate_axis = unit(candidate.axis())?;
        if candidate_axis.dot(axis).abs() < 1.0 - Tolerance::new().angular {
            continue;
        }
        let offset = candidate.origin() - base;
        let perpendicular = offset - axis * offset.dot(axis);
        if perpendicular.length() > linear_tol {
            continue;
        }
        let (candidate_base, candidate_height) = axial_extent(topo, fid, candidate)?;
        let candidate_end = candidate_base + candidate_axis * candidate_height;
        let t0 = (candidate_base - base).dot(axis);
        let t1 = (candidate_end - base).dot(axis);
        let overlap_start = t0.min(t1).max(0.0);
        let overlap_end = t0.max(t1).min(height);
        if overlap_end - overlap_start > linear_tol {
            resized_faces.push(fid);
        }
    }

    let mut edge_uses: HashMap<remus_topology::edge::EdgeId, Vec<EdgeUse>> = HashMap::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let reversed = face.is_reversed();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for (position, oe) in topo.wire(wid)?.edges().iter().enumerate() {
                edge_uses.entry(oe.edge()).or_default().push(EdgeUse {
                    face: fid,
                    wire: wid,
                    position,
                    stored_forward: oe.is_forward(),
                    effective_forward: oe.is_forward() != reversed,
                });
            }
        }
    }

    let mut repairs = Vec::new();
    for (&edge_id, uses) in &edge_uses {
        let [first, second] = uses.as_slice() else {
            continue;
        };
        if first.effective_forward != second.effective_forward {
            continue;
        }
        let candidates: Vec<_> = [*first, *second]
            .into_iter()
            .filter(|edge_use| resized_faces.contains(&edge_use.face))
            .collect();
        match candidates.as_slice() {
            [candidate] if topo.edge(edge_id)?.is_closed() => repairs.push(*candidate),
            [] => {}
            _ => {
                return Err(crate::OperationsError::InvalidInput {
                    reason: "cylindrical resize produced an ambiguous shell orientation defect"
                        .into(),
                });
            }
        }
    }

    for repair in &repairs {
        let wire = topo.wire(repair.wire)?;
        let mut edges = wire.edges().to_vec();
        let Some(oriented) = edges.get_mut(repair.position) else {
            return Err(crate::OperationsError::InvalidInput {
                reason: "cylindrical resize lost a rim during orientation repair".into(),
            });
        };
        *oriented =
            remus_topology::wire::OrientedEdge::new(oriented.edge(), !repair.stored_forward);
        let replacement = remus_topology::wire::Wire::new(edges, wire.is_closed())?;
        topo.replace_boundary_wire(repair.wire, replacement)?;
    }

    let remaining = remus_check::validate::shell::check_shell_orientation(topo, shell_id)?;
    if !remaining.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "cylindrical resize left {} shell orientation issue(s)",
                remaining.len()
            ),
        });
    }
    Ok(repairs.len())
}

/// Reject a result whose volume is not the one the edit must produce.
///
/// The construction above is geometric rather than exact, so this is the gate
/// that makes it trustworthy: a tool that reached material it should not have,
/// or a boolean that silently dropped it, moves the volume off the analytic
/// target and the attempt is rejected instead of returned.
fn ensure_volume(
    topo: &Topology,
    solid: SolidId,
    expected: f64,
    what: &str,
) -> Result<(), crate::OperationsError> {
    let actual = solid_volume(topo, solid, verify_deflection(topo, solid))?;
    // Volume is measured from a tessellation, so allow its discretisation
    // error — wide enough for a curved wall, far tighter than any real defect.
    let slack = expected.abs().mul_add(2e-3, 1e-6);
    if (actual - expected).abs() <= slack {
        return Ok(());
    }
    Err(crate::OperationsError::InvalidInput {
        reason: format!("{what} produced volume {actual}, expected {expected}"),
    })
}

/// Drop inner wires that bound nothing, returning how many were removed.
///
/// Replacing a coaxial cylindrical feature can leave the OLD rim behind as an
/// inner wire on the face that absorbed it — growing a boss from r=5 to r=8
/// leaves the r=5 circle as a hole in the new r=8 cap. Every edge of such a
/// wire is used by that one face alone, so it borders no second face and the
/// shell is open along it.
///
/// A wire in that state cannot be the boundary of a real cavity (a cavity
/// would have faces on the other side), so the hole is spurious and the face's
/// own surface already covers it. Removing the wire closes the shell without
/// moving any geometry — and the caller's volume gate confirms it.
fn drop_stranded_inner_wires(
    topo: &mut Topology,
    solid: SolidId,
) -> Result<usize, crate::OperationsError> {
    let mut uses: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    for fid in solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid)?.edges() {
                *uses.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }

    let mut stranded: Vec<(FaceId, Vec<usize>)> = Vec::new();
    for fid in solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        let mut drop_idx = Vec::new();
        for (i, &wid) in face.inner_wires().iter().enumerate() {
            let wire = topo.wire(wid)?;
            let all_free = wire
                .edges()
                .iter()
                .all(|oe| uses.get(&oe.edge().index()).copied().unwrap_or(0) == 1);
            if all_free && !wire.edges().is_empty() {
                drop_idx.push(i);
            }
        }
        if !drop_idx.is_empty() {
            stranded.push((fid, drop_idx));
        }
    }

    let mut removed = 0;
    for (fid, drop_idx) in stranded {
        let face = topo.face(fid)?;
        let outer = face.outer_wire();
        let mut inner = face.inner_wires().to_vec();
        // Remove from the back so earlier indices stay valid.
        for &i in drop_idx.iter().rev() {
            inner.remove(i);
            removed += 1;
        }
        topo.set_face_boundary_wires(fid, outer, inner)?;
    }
    Ok(removed)
}

/// Reject a face that does not belong to `solid` (including its inner shells).
fn ensure_face_in_solid(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
) -> Result<(), crate::OperationsError> {
    if solid_faces(topo, solid)?.contains(&face) {
        return Ok(());
    }
    Err(crate::OperationsError::InvalidInput {
        reason: format!(
            "face {} is not part of solid {}",
            face.index(),
            solid.index()
        ),
    })
}

/// The closed-shell gate.
///
/// `validate_solid_relaxed` does not check shell closure, so a result can
/// measure the right volume and still be unexportable — a stale rim left on
/// one face is invisible to a volume check but leaves the shell open.
fn ensure_closed_shell(
    topo: &Topology,
    solid: SolidId,
    what: &str,
) -> Result<(), crate::OperationsError> {
    use remus_check::validate::checks::{CheckId, Severity};
    use remus_check::validate::{ValidateOptions, validate_solid};

    let report = validate_solid(topo, solid, &ValidateOptions::default())?;
    let open: Vec<&str> = report
        .issues
        .iter()
        .filter(|i| i.check == CheckId::ShellClosed && i.severity == Severity::Error)
        .map(|i| i.description.as_str())
        .collect();
    if open.is_empty() {
        return Ok(());
    }
    Err(crate::OperationsError::InvalidInput {
        reason: format!("{what} left an open shell: {}", open.join("; ")),
    })
}

/// The face's extent along its cylinder axis, as a base point and a height.
///
/// Taken from the face's own vertices rather than the surface (which is
/// unbounded), so the tool spans exactly the wall being moved.
fn axial_extent(
    topo: &Topology,
    face: FaceId,
    cyl: &CylindricalSurface,
) -> Result<(Point3, f64), crate::OperationsError> {
    let axis = unit(cyl.axis())?;
    let origin = cyl.origin();

    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    let face_data = topo.face(face)?;
    for wid in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        let wire = topo.wire(wid)?;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            for vid in [edge.start(), edge.end()] {
                let t = (topo.vertex(vid)?.point() - origin).dot(axis);
                lo = lo.min(t);
                hi = hi.max(t);
            }
        }
    }

    if !lo.is_finite() || !hi.is_finite() {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylindrical face {} has no vertices", face.index()),
        });
    }
    Ok((origin + axis * lo, hi - lo))
}

/// The radial direction of the selected cylindrical face's stored seam.
///
/// A [`CylindricalSurface`]'s parameter-frame X axis is not necessarily where
/// the face's closed seam edge was constructed. Read the topology itself so a
/// rigidly transformed resize tool reuses the exact seam angle.
fn cylinder_seam_direction(
    topo: &Topology,
    face: FaceId,
    cyl: &CylindricalSurface,
) -> Result<Vec3, crate::OperationsError> {
    let axis = unit(cyl.axis())?;
    let face_data = topo.face(face)?;
    for wid in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        for oriented in topo.wire(wid)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            for vertex in [edge.start(), edge.end()] {
                let offset = topo.vertex(vertex)?.point() - cyl.origin();
                let radial = offset - axis * offset.dot(axis);
                if radial.length() > Tolerance::new().linear {
                    return unit(radial);
                }
            }
        }
    }
    Err(crate::OperationsError::InvalidInput {
        reason: format!("cylindrical face {} has no seam direction", face.index()),
    })
}

/// Normalize a direction, mapping a degenerate one onto an operations error.
fn unit(v: Vec3) -> Result<Vec3, crate::OperationsError> {
    v.normalize().map_err(crate::OperationsError::Math)
}

/// Build the matrix taking the canonical +Z cylinder to the selected
/// cylinder's own analytic frame at `base`.
fn frame_matrix(base: Point3, axis: Vec3, x_axis: Vec3) -> Result<Mat4, crate::OperationsError> {
    let z = unit(axis)?;
    // Preserve the source surface's radial frame rather than choosing an
    // arbitrary perpendicular direction. The cylinder is rotationally
    // symmetric geometrically, but its closed seam is topological: rotating
    // that seam relative to the selected wall prevents coincident edges from
    // merging and can force a faceted boolean fallback.
    let x = unit(x_axis - z * x_axis.dot(z))?;
    let y = z.cross(x);
    Ok(Mat4([
        [x.x(), y.x(), z.x(), base.x()],
        [x.y(), y.y(), z.y(), base.y()],
        [x.z(), y.z(), z.z(), base.z()],
        [0.0, 0.0, 0.0, 1.0],
    ]))
}

/// Invert an orthonormal affine frame by transposing its rotation block.
///
/// Exact where the generic adjugate `Mat4::inverse` is not: the rotation
/// entries come back bit-identical and the bottom row is literally
/// `[0, 0, 0, 1]`, so the `to_local` → edit → `to_world` round trip does not
/// accumulate inversion round-off in the frame itself. It is also infallible,
/// which `Mat4::inverse` is not. Valid only for an orthonormal frame —
/// `frame_matrix` builds one.
fn inverse_rigid_frame(frame: &Mat4) -> Mat4 {
    let m = &frame.0;
    let tx = m[0][3];
    let ty = m[1][3];
    let tz = m[2][3];
    Mat4([
        [
            m[0][0],
            m[1][0],
            m[2][0],
            -m[0][0].mul_add(tx, m[1][0].mul_add(ty, m[2][0] * tz)),
        ],
        [
            m[0][1],
            m[1][1],
            m[2][1],
            -m[0][1].mul_add(tx, m[1][1].mul_add(ty, m[2][1] * tz)),
        ],
        [
            m[0][2],
            m[1][2],
            m[2][2],
            -m[0][2].mul_add(tx, m[1][2].mul_add(ty, m[2][2] * tz)),
        ],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

/// A cylinder of `radius`/`height` based at `base` and running along `axis`.
fn place_cylinder(
    topo: &mut Topology,
    base: Point3,
    axis: Vec3,
    x_axis: Vec3,
    radius: f64,
    height: f64,
) -> Result<SolidId, crate::OperationsError> {
    let solid = make_cylinder(topo, radius, height)?;
    transform_solid(topo, solid, &frame_matrix(base, axis, x_axis)?)?;
    Ok(solid)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use std::collections::HashMap;
    use std::f64::consts::PI;

    use remus_math::mat::Mat4;

    use super::*;
    use crate::measure::solid_volume;
    use crate::primitives::make_box;

    const DEFLECTION: f64 = 0.01;

    fn cylinder_at(topo: &mut Topology, r: f64, h: f64, x: f64, y: f64, z: f64) -> SolidId {
        let c = make_cylinder(topo, r, h).unwrap();
        transform_solid(topo, c, &Mat4::translation(x, y, z)).unwrap();
        c
    }

    /// Volume within the tessellation's deflection error.
    fn assert_volume(topo: &Topology, solid: SolidId, expected: f64) {
        let v = solid_volume(topo, solid, DEFLECTION).unwrap();
        assert!(
            (v - expected).abs() < expected.abs().mul_add(1e-3, 1.0),
            "volume {v} != expected {expected}"
        );
    }

    /// Every edge must be used exactly twice across the solid's faces.
    ///
    /// This is the property the coaxial-bore bug broke while volume and
    /// relaxed validation both still passed.
    fn assert_watertight(topo: &Topology, solid: SolidId) {
        let mut counts: HashMap<usize, usize> = HashMap::new();
        for fid in solid_faces(topo, solid).unwrap() {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    *counts.entry(oe.edge().index()).or_insert(0) += 1;
                }
            }
        }
        let free: Vec<_> = counts.iter().filter(|&(_, &c)| c != 2).collect();
        assert!(
            free.is_empty(),
            "edges not shared by exactly 2 faces: {free:?}"
        );
    }

    fn face_count(topo: &Topology, solid: SolidId, tag: &str) -> usize {
        solid_faces(topo, solid)
            .unwrap()
            .iter()
            .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == tag)
            .count()
    }

    /// The planar face whose outward normal is `dir` and which lies furthest
    /// along it — i.e. the visible face on that side.
    fn face_facing(topo: &Topology, solid: SolidId, dir: Vec3) -> FaceId {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&f| {
                topo.face(f)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| n.dot(dir) > 0.99)
            })
            .max_by(|&a, &b| {
                let along = |f: FaceId| {
                    let w = topo.face(f).unwrap().outer_wire();
                    let e = topo.wire(w).unwrap().edges()[0].edge();
                    (topo.vertex(topo.edge(e).unwrap().start()).unwrap().point()
                        - Point3::new(0.0, 0.0, 0.0))
                    .dot(dir)
                };
                along(a).partial_cmp(&along(b)).unwrap()
            })
            .expect("no face with the requested normal")
    }

    fn only_cylinder(topo: &Topology, solid: SolidId) -> FaceId {
        let cyls: Vec<_> = solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
            .collect();
        assert_eq!(cyls.len(), 1, "expected exactly one cylindrical face");
        cyls[0]
    }

    /// A 40x40x10 block with an r=3 through-bore at (20, 20).
    fn drilled_block(topo: &mut Topology) -> SolidId {
        let block = make_box(topo, 40.0, 40.0, 10.0).unwrap();
        let drill = cylinder_at(topo, 3.0, 10.0, 20.0, 20.0, 0.0);
        boolean(topo, BooleanOp::Cut, block, drill).unwrap()
    }

    /// A 40x40x10 block with an r=5 h=10 boss standing on its top face.
    fn bossed_block(topo: &mut Topology) -> SolidId {
        let block = make_box(topo, 40.0, 40.0, 10.0).unwrap();
        let boss = cylinder_at(topo, 5.0, 10.0, 20.0, 20.0, 10.0);
        boolean(topo, BooleanOp::Fuse, block, boss).unwrap()
    }

    // --- push_pull_face -------------------------------------------------

    #[test]
    fn simple_cylinder_cap_moves_stay_exact_across_offset_and_scale_bands() {
        for scale in [1e-3, 1.0, 1e3] {
            for distance in [-0.001, -5.0, -20.0, 5.0].map(|value| value * scale) {
                for direction in [Vec3::new(0.0, 0.0, -1.0), Vec3::new(0.0, 0.0, 1.0)] {
                    let mut topo = Topology::new();
                    let radius = 10.0 * scale;
                    let height = 30.0 * scale;
                    let cylinder = make_cylinder(&mut topo, radius, height).unwrap();
                    let cap = face_facing(&topo, cylinder, direction);

                    let result = push_pull_face(&mut topo, cylinder, cap, distance).unwrap();
                    let expected = PI * radius * radius * (height + distance);
                    let actual = solid_volume(&topo, result, DEFLECTION * scale).unwrap();
                    let relative = (actual - expected).abs() / expected;

                    assert!(
                        relative < 1e-12,
                        "scale {scale}, distance {distance}, direction {direction:?}: \
                         volume {actual} vs {expected} ({relative:e})"
                    );
                    assert_eq!(face_count(&topo, result, "cylinder"), 1);
                    assert_eq!(face_count(&topo, result, "plane"), 2);
                    assert_watertight(&topo, result);
                }
            }
        }
    }

    #[test]
    fn simple_cylinder_cap_move_preserves_a_noncanonical_frame() {
        let mut topo = Topology::new();
        let axis = Vec3::new(1.0, 2.0, 3.0).normalize().unwrap();
        let cylinder = place_cylinder(
            &mut topo,
            Point3::new(4.0, -7.0, 2.0),
            axis,
            Vec3::new(0.0, 3.0, -2.0),
            10.0,
            30.0,
        )
        .unwrap();
        let top = face_facing(&topo, cylinder, axis);

        let result = push_pull_face(&mut topo, cylinder, top, -5.0).unwrap();

        let actual = solid_volume(&topo, result, DEFLECTION).unwrap();
        let expected = PI * 10.0 * 10.0 * 25.0;
        assert!((actual - expected).abs() / expected < 1e-12);
        assert_eq!(face_count(&topo, result, "cylinder"), 1);
        assert_eq!(face_count(&topo, result, "plane"), 2);
        assert_watertight(&topo, result);
    }

    #[test]
    fn simple_cylinder_cap_move_refuses_height_collapse_boundary() {
        for distance in [-30.0, -31.0] {
            let mut topo = Topology::new();
            let cylinder = make_cylinder(&mut topo, 10.0, 30.0).unwrap();
            let top = face_facing(&topo, cylinder, Vec3::new(0.0, 0.0, 1.0));
            let error = push_pull_face(&mut topo, cylinder, top, distance).unwrap_err();
            assert!(
                matches!(error, crate::OperationsError::InvalidInput { .. }),
                "distance {distance}: {error}"
            );
        }

        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 10.0, 30.0).unwrap();
        let top = face_facing(&topo, cylinder, Vec3::new(0.0, 0.0, 1.0));
        let result = push_pull_face(&mut topo, cylinder, top, -29.0).unwrap();
        assert_eq!(face_count(&topo, result, "cylinder"), 1);
        assert_eq!(face_count(&topo, result, "plane"), 2);
    }

    #[test]
    fn pulling_a_box_face_adds_a_slab() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));

        let out = push_pull_face(&mut topo, block, top, 5.0).unwrap();

        assert_volume(&topo, out, 10.0 * 10.0 * 15.0);
        assert_watertight(&topo, out);
        // The seam where the tool met the block must be merged away.
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn pushing_a_box_face_removes_a_slab() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));

        let out = push_pull_face(&mut topo, block, top, -3.0).unwrap();

        assert_volume(&topo, out, 10.0 * 10.0 * 7.0);
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn pulling_twice_matches_pulling_once() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));
        let once = push_pull_face(&mut topo, block, top, 4.0).unwrap();

        let block2 = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top2 = face_facing(&topo, block2, Vec3::new(0.0, 0.0, 1.0));
        let step1 = push_pull_face(&mut topo, block2, top2, 2.0).unwrap();
        let top3 = face_facing(&topo, step1, Vec3::new(0.0, 0.0, 1.0));
        let twice = push_pull_face(&mut topo, step1, top3, 2.0).unwrap();

        assert_volume(&topo, twice, solid_volume(&topo, once, DEFLECTION).unwrap());
        assert_eq!(
            face_count(&topo, twice, "plane"),
            face_count(&topo, once, "plane")
        );
        assert_watertight(&topo, twice);
    }

    #[test]
    fn pulling_a_face_with_a_hole_keeps_the_hole() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let top = face_facing(&topo, drilled, Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(
            topo.face(top).unwrap().inner_wires().len(),
            1,
            "the picked cap should carry the bore as an inner wire"
        );

        let out = push_pull_face(&mut topo, drilled, top, 5.0).unwrap();

        // The block grows to 15 tall and the bore grows with it.
        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 15.0, -(PI * 9.0 * 15.0)));
        assert_watertight(&topo, out);
        // The bore stays ONE cylindrical face, not two stacked bands.
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn pushing_a_face_with_a_hole_keeps_the_hole() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let top = face_facing(&topo, drilled, Vec3::new(0.0, 0.0, 1.0));

        let out = push_pull_face(&mut topo, drilled, top, -4.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 6.0, -(PI * 9.0 * 6.0)));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn push_pull_rejects_bad_input() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));

        assert!(push_pull_face(&mut topo, block, top, 0.0).is_err());
        assert!(push_pull_face(&mut topo, block, top, f64::NAN).is_err());

        // A face belonging to a different solid.
        let other = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let other_top = face_facing(&topo, other, Vec3::new(0.0, 0.0, 1.0));
        assert!(push_pull_face(&mut topo, block, other_top, 1.0).is_err());

        // A cylindrical face is not push/pull-able.
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);
        assert!(push_pull_face(&mut topo, drilled, bore, 1.0).is_err());
    }

    // --- resize_cylindrical_face ----------------------------------------

    #[test]
    fn widening_a_bore() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);

        let out = resize_cylindrical_face(&mut topo, drilled, bore, 5.0).unwrap();

        assert_volume(
            &topo,
            out,
            40.0f64.mul_add(40.0 * 10.0, -(PI * 25.0 * 10.0)),
        );
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn shrinking_a_bore() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);

        let out = resize_cylindrical_face(&mut topo, drilled, bore, 2.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, -(PI * 4.0 * 10.0)));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn journaled_radius_covers_source_and_result_entities() {
        use crate::journal_ops::{resize_cylindrical_face_journaled, solid_entity_keys};
        use remus_topology::journal::{EntityKind, EventDraft, EvolutionDraft};
        use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
        for transform in [
            Mat4::identity(),
            Mat4::translation(12.0, -7.0, 5.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
            Mat4::translation(-9.0, 4.0, 13.0) * Mat4::rotation_x(0.7) * Mat4::rotation_y(-0.4),
            Mat4::translation(3.0, 8.0, 21.0) * Mat4::rotation_x(PI),
        ] {
            for (bore, radius) in [(false, 3.0), (false, 8.0), (true, 2.0), (true, 5.0)] {
                let mut topo = Topology::new();
                let source = if bore {
                    drilled_block(&mut topo)
                } else {
                    bossed_block(&mut topo)
                };
                transform_solid(&mut topo, source, &transform).unwrap();
                let wall = only_cylinder(&topo, source);
                let source_keys = solid_entity_keys(&topo, source).unwrap();
                let pending = topo.journal_begin("radius_fixture");
                let mut draft = EvolutionDraft::construction();
                draft.add_scope(source_keys.iter().copied());
                for &key in &source_keys {
                    draft.push(
                        key,
                        EventDraft::Generated {
                            sources: Vec::new(),
                        },
                    );
                }
                let anchor = topo.journal_record_evolution(pending, draft).unwrap();
                let result = resize_cylindrical_face_journaled(&mut topo, source, wall, radius)
                    .unwrap_or_else(|error| panic!("bore {bore}, radius {radius}: {error}"));
                let result_keys: HashSet<_> = solid_entity_keys(&topo, result.solid)
                    .unwrap()
                    .into_iter()
                    .collect();
                for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                    for index in 0..source_keys.iter().filter(|key| key.kind == kind).count() {
                        let reference = PersistentRef::operation_output(anchor, kind, index);
                        match resolve(&topo, &reference) {
                            Resolution::Bound {
                                entity,
                                provenance: Provenance::Construction,
                            } => assert!(result_keys.contains(&entity)),
                            Resolution::BoundMany {
                                entities,
                                provenance: Provenance::Construction,
                            } => {
                                assert_eq!(kind, EntityKind::Face);
                                assert!(entities.iter().all(|entity| result_keys.contains(entity)));
                            }
                            other => panic!(
                                "bore {bore}, radius {radius}, source {kind:?}/{index}: {other:?}"
                            ),
                        }
                    }
                    let mut recorded = HashSet::new();
                    for index in 0..result_keys.iter().filter(|key| key.kind == kind).count() {
                        let reference = PersistentRef::operation_output(result.op, kind, index);
                        match resolve(&topo, &reference) {
                            Resolution::Bound {
                                entity,
                                provenance: Provenance::Construction,
                            } => {
                                recorded.insert(entity);
                            }
                            other => panic!(
                                "bore {bore}, radius {radius}, output {kind:?}/{index}: {other:?}"
                            ),
                        }
                    }
                    assert_eq!(
                        recorded,
                        result_keys
                            .iter()
                            .filter(|key| key.kind == kind)
                            .copied()
                            .collect()
                    );
                }
                assert_watertight(&topo, result.solid);
                let next_wall = only_cylinder(&topo, result.solid);
                let next_radius = if bore { 3.0 } else { 5.0 };
                let next = resize_cylindrical_face_journaled(
                    &mut topo,
                    result.solid,
                    next_wall,
                    next_radius,
                )
                .unwrap_or_else(|error| panic!("second bore {bore}, radius {radius}: {error}"));
                for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                    for index in 0..source_keys.iter().filter(|key| key.kind == kind).count() {
                        let reference = PersistentRef::operation_output(anchor, kind, index);
                        assert!(
                            matches!(
                                resolve(&topo, &reference),
                                Resolution::Bound {
                                    provenance: Provenance::Construction,
                                    ..
                                } | Resolution::BoundMany {
                                    provenance: Provenance::Construction,
                                    ..
                                }
                            ),
                            "second bore {bore}, radius {radius}, {kind:?}/{index}: {:?}; map {:?}",
                            resolve(&topo, &reference),
                            next.map
                        );
                    }
                }
                for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                    for index in 0..result_keys.iter().filter(|key| key.kind == kind).count() {
                        let reference = PersistentRef::operation_output(result.op, kind, index);
                        assert!(
                            matches!(
                                resolve(&topo, &reference),
                                Resolution::Bound {
                                    provenance: Provenance::Construction,
                                    ..
                                } | Resolution::BoundMany {
                                    provenance: Provenance::Construction,
                                    ..
                                } | Resolution::Dangling { .. }
                            ),
                            "second output bore {bore}, radius {radius}, {kind:?}/{index}: {:?}",
                            resolve(&topo, &reference)
                        );
                    }
                }
                assert_watertight(&topo, next.solid);
            }
        }
    }

    #[test]
    fn journaled_radius_is_scale_aware() {
        use crate::journal_ops::resize_cylindrical_face_journaled;
        for scale in [1e-3_f64, 1e3] {
            for bore in [false, true] {
                let mut topo = Topology::new();
                let source = if bore {
                    drilled_block(&mut topo)
                } else {
                    bossed_block(&mut topo)
                };
                transform_solid(&mut topo, source, &Mat4::scale(scale, scale, scale)).unwrap();
                let source_volume = solid_volume(&topo, source, DEFLECTION * scale).unwrap();
                let first_wall = only_cylinder(&topo, source);
                let first =
                    resize_cylindrical_face_journaled(&mut topo, source, first_wall, 4.0 * scale)
                        .unwrap();
                let second_wall = only_cylinder(&topo, first.solid);
                let second = resize_cylindrical_face_journaled(
                    &mut topo,
                    first.solid,
                    second_wall,
                    3.5 * scale,
                )
                .unwrap();
                assert!(first.map.origin.is_exact() && first.map.is_complete());
                assert!(second.map.origin.is_exact() && second.map.is_complete());
                let expected = (16000.0
                    + if bore { -1.0 } else { 1.0 } * PI * 3.5_f64.powi(2) * 10.0)
                    * scale.powi(3);
                let actual = solid_volume(&topo, second.solid, DEFLECTION * scale).unwrap();
                assert!(
                    (actual - expected).abs() <= expected.abs() * 1e-4,
                    "bore {bore}, scale {scale}: {actual} != {expected}"
                );
                let unchanged = solid_volume(&topo, source, DEFLECTION * scale).unwrap();
                assert!((unchanged - source_volume).abs() <= source_volume.abs() * 1e-12);
                assert_watertight(&topo, second.solid);
            }
        }
    }

    #[test]
    fn journaled_radius_refusals_restore_topology_and_history() {
        use crate::journal_ops::resize_cylindrical_face_journaled;
        for radius in [-1.0, 0.0, f64::NAN, f64::INFINITY, 20.0, 25.0] {
            let mut topo = Topology::new();
            let solid = drilled_block(&mut topo);
            let face = only_cylinder(&topo, solid);
            let before = topo.journal().snapshot();
            let counts = |topo: &Topology| {
                (
                    topo.num_vertices(),
                    topo.num_edges(),
                    topo.num_wires(),
                    topo.num_faces(),
                    topo.num_shells(),
                    topo.num_solids(),
                    topo.num_pcurves(),
                )
            };
            let before_counts = counts(&topo);
            let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
            assert!(
                resize_cylindrical_face_journaled(&mut topo, solid, face, radius).is_err(),
                "radius {radius}"
            );
            assert_eq!(topo.journal().snapshot(), before);
            assert_eq!(counts(&topo), before_counts);
            assert_volume(&topo, solid, volume);
        }
    }

    #[test]
    fn boss_radius_construction_history_covers_the_result() {
        for radius in [3.0, 8.0] {
            let mut topo = Topology::new();
            let source = bossed_block(&mut topo);
            let wall = only_cylinder(&topo, source);
            let source_faces = solid_faces(&topo, source).unwrap();
            let (result, pairs) =
                resize_cylindrical_face_aligned(&mut topo, source, wall, radius, true).unwrap();
            assert!(result.evolution.origin.is_exact());
            let unresolved: Vec<_> = result
                .evolution
                .unresolved
                .keys()
                .map(|&index| {
                    let face = topo.face(topo.face_id_from_index(index).unwrap()).unwrap();
                    (index, face.surface().clone())
                })
                .collect();
            assert!(
                result.evolution.is_complete(),
                "radius {radius}: {:?}; unresolved surfaces: {unresolved:?}",
                result.evolution
            );
            assert_eq!(result.evolution.modified.len(), source_faces.len());
            let counts = remus_topology::explorer::solid_entity_counts(&topo, source).unwrap();
            assert_eq!(
                pairs.len(),
                counts.1 + counts.2,
                "radius {radius}: every boundary needs history"
            );
            assert_volume(
                &topo,
                result.solid,
                40.0f64.mul_add(40.0 * 10.0, PI * radius * radius * 10.0),
            );
            assert_watertight(&topo, result.solid);
        }
    }

    #[test]
    fn growing_a_boss() {
        let mut topo = Topology::new();
        let bossed = bossed_block(&mut topo);
        let wall = only_cylinder(&topo, bossed);

        let out = resize_cylindrical_face(&mut topo, bossed, wall, 8.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, PI * 64.0 * 10.0));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn shrinking_a_boss() {
        let mut topo = Topology::new();
        let bossed = bossed_block(&mut topo);
        let wall = only_cylinder(&topo, bossed);

        let out = resize_cylindrical_face(&mut topo, bossed, wall, 3.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, PI * 9.0 * 10.0));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn resizing_a_bore_twice_stays_watertight() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);
        let wide = resize_cylindrical_face(&mut topo, drilled, bore, 5.0).unwrap();
        let bore2 = only_cylinder(&topo, wide);
        let narrow = resize_cylindrical_face(&mut topo, wide, bore2, 4.0).unwrap();

        assert_volume(
            &topo,
            narrow,
            40.0f64.mul_add(40.0 * 10.0, -(PI * 16.0 * 10.0)),
        );
        assert_watertight(&topo, narrow);
        assert_eq!(face_count(&topo, narrow, "cylinder"), 1);
    }

    #[test]
    fn resizing_a_rigidly_transformed_bore_preserves_an_exact_wall() {
        let transforms = [
            Mat4::translation(6.0, 11.0, -3.0) * Mat4::rotation_z(0.63),
            Mat4::translation(12.0, -7.0, 5.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
            Mat4::translation(-9.0, 4.0, 13.0) * Mat4::rotation_x(0.7) * Mat4::rotation_y(-0.4),
            Mat4::translation(3.0, 8.0, 21.0) * Mat4::rotation_x(PI),
        ];

        for transform in transforms {
            let mut topo = Topology::new();
            let drilled = drilled_block(&mut topo);
            transform_solid(&mut topo, drilled, &transform).unwrap();

            let bore = only_cylinder(&topo, drilled);
            let wide = resize_cylindrical_face(&mut topo, drilled, bore, 5.0).unwrap();
            let wide_bore = only_cylinder(&topo, wide);
            let narrow = resize_cylindrical_face(&mut topo, wide, wide_bore, 4.0).unwrap();

            assert_volume(
                &topo,
                narrow,
                40.0f64.mul_add(40.0 * 10.0, -(PI * 16.0 * 10.0)),
            );
            assert_watertight(&topo, narrow);
            let FaceSurface::Cylinder(cyl) =
                topo.face(only_cylinder(&topo, narrow)).unwrap().surface()
            else {
                unreachable!();
            };
            assert!(Tolerance::new().approx_eq(cyl.radius(), 4.0));
        }
    }

    #[test]
    fn resizing_a_bore_is_scale_aware() {
        for scale in [1e-3_f64, 1e3_f64] {
            let mut topo = Topology::new();
            let block = make_box(&mut topo, 40.0 * scale, 40.0 * scale, 10.0 * scale).unwrap();
            let drill = cylinder_at(
                &mut topo,
                3.0 * scale,
                10.0 * scale,
                20.0 * scale,
                20.0 * scale,
                0.0,
            );
            let drilled = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();
            let bore = only_cylinder(&topo, drilled);
            let wide = resize_cylindrical_face(&mut topo, drilled, bore, 5.0 * scale).unwrap();
            let wide_bore = only_cylinder(&topo, wide);
            let narrow = resize_cylindrical_face(&mut topo, wide, wide_bore, 4.0 * scale).unwrap();

            let expected = (40.0 * scale).mul_add(
                40.0 * scale * 10.0 * scale,
                -(PI * (4.0 * scale).powi(2) * 10.0 * scale),
            );
            let deflection = (DEFLECTION * scale).clamp(1e-6, 10.0);
            let actual = solid_volume(&topo, narrow, deflection).unwrap();
            assert!(
                (actual - expected).abs() <= expected.abs().max(scale.powi(3)) * 1e-3,
                "scale {scale}: volume {actual} != {expected}"
            );
            assert_watertight(&topo, narrow);
            let FaceSurface::Cylinder(cyl) =
                topo.face(only_cylinder(&topo, narrow)).unwrap().surface()
            else {
                unreachable!();
            };
            assert!(Tolerance::new().approx_eq(cyl.radius(), 4.0 * scale));
        }
    }

    #[test]
    fn widening_a_bore_into_other_geometry_fails_closed() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);
        let before = solid_volume(&topo, drilled, DEFLECTION).unwrap();

        assert!(resize_cylindrical_face(&mut topo, drilled, bore, 25.0).is_err());
        assert_volume(&topo, drilled, before);
        assert_watertight(&topo, drilled);
    }

    /// Regression: an annular sleeve fused into a matching bore.
    ///
    /// Every contact is coincident — the sleeve's outer wall IS the bore wall,
    /// and its end caps sit in the caps' own planes inside their holes. The
    /// annuli used to classify inconsistently (one kept, one dropped), and the
    /// coplanar merge then carried the filled r=3 rim onto the merged cap,
    /// leaving free edges. Exercised directly here, below `resize`.
    #[test]
    fn sleeve_fused_into_a_matching_bore_closes_the_shell() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);

        let outer = cylinder_at(&mut topo, 3.0, 10.0, 20.0, 20.0, 0.0);
        let inner = cylinder_at(&mut topo, 2.0, 12.0, 20.0, 20.0, -1.0);
        let sleeve = boolean(&mut topo, BooleanOp::Cut, outer, inner).unwrap();

        let out = boolean(&mut topo, BooleanOp::Fuse, drilled, sleeve).unwrap();
        unify_faces(&mut topo, out).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, -(PI * 4.0 * 10.0)));
        assert_watertight(&topo, out);
        // The r=3 wall is gone and the r=2 one replaces it — one bore, not two.
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    /// Regression: two coaxial bore bands of equal radius must merge into one
    /// face. `unify_faces` used to treat each band's seam edge — which appears
    /// twice in the same wire — as a shared internal edge and delete it,
    /// leaving two disjoint rim circles that reassembled as an outer wire plus
    /// a bogus inner wire on a cylinder.
    #[test]
    fn stacked_coaxial_bore_bands_merge_into_one_wall() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);

        // A slab with a coaxial bore, stacked directly on top.
        let slab = make_box(&mut topo, 40.0, 40.0, 5.0).unwrap();
        transform_solid(&mut topo, slab, &Mat4::translation(0.0, 0.0, 10.0)).unwrap();
        let slab_bore = cylinder_at(&mut topo, 3.0, 5.0, 20.0, 20.0, 10.0);
        let holed_slab = boolean(&mut topo, BooleanOp::Cut, slab, slab_bore).unwrap();

        let out = boolean(&mut topo, BooleanOp::Fuse, drilled, holed_slab).unwrap();
        unify_faces(&mut topo, out).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 15.0, -(PI * 9.0 * 15.0)));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        let bore = only_cylinder(&topo, out);
        assert!(
            topo.face(bore).unwrap().inner_wires().is_empty(),
            "a merged bore wall must not acquire an inner wire"
        );
    }

    #[test]
    fn resize_rejects_bad_input() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);

        assert!(resize_cylindrical_face(&mut topo, drilled, bore, 0.0).is_err());
        assert!(resize_cylindrical_face(&mut topo, drilled, bore, -1.0).is_err());
        assert!(resize_cylindrical_face(&mut topo, drilled, bore, f64::INFINITY).is_err());
        // Already at this radius.
        assert!(resize_cylindrical_face(&mut topo, drilled, bore, 3.0).is_err());
        // A planar face is not resizable.
        let top = face_facing(&topo, drilled, Vec3::new(0.0, 0.0, 1.0));
        assert!(resize_cylindrical_face(&mut topo, drilled, top, 5.0).is_err());
    }
}
