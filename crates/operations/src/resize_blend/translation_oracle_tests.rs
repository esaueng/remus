//! Differential oracle for the blend translation path: the version that
//! translated a whole-document scratch copy, kept verbatim (only the
//! identity face pairing is spelled out for the new
//! `translated_faces_volume_change` signature), against the copy written from
//! the input with the translated geometry in place, over filleted bodies,
//! NURBS bands, refusals and documents holding other bodies. Both must leave
//! the document in the same state and return the same result, maps and
//! refusal text, with and without the content-keyed memos, on the face
//! integral path and on the whole-body mesh fallback.

#![allow(clippy::unwrap_used, clippy::too_many_lines)]

use std::cell::Cell;

use super::*;

thread_local! {
    /// Makes `translated_faces_volume_change` decline, so the move takes the
    /// whole-body mesh comparison.
    pub(super) static FORCE_MESH_VOLUME: Cell<bool> = const { Cell::new(false) };
}

fn legacy_move_translation_invariant_blend_region(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<BlendMoveEntities, OperationsError> {
    if !distance.is_finite() || distance.abs() <= Tolerance::new().linear {
        return Err(remus_offset::OffsetError::InvalidInput {
            reason: "move-face distance must be non-zero and finite".into(),
        }
        .into());
    }
    let Some(&reference) = faces.first() else {
        return Err(remus_offset::OffsetError::InvalidInput {
            reason: "move-face requires at least one selected face".into(),
        }
        .into());
    };
    let source_faces = remus_topology::explorer::solid_faces(topo, solid)?;
    let source_face_set: HashSet<FaceId> = source_faces.iter().copied().collect();
    if faces.iter().any(|face| !source_face_set.contains(face)) {
        return Err(remus_offset::OffsetError::FaceNotInSolid {
            face: faces
                .iter()
                .copied()
                .find(|face| !source_face_set.contains(face))
                .unwrap_or(reference),
            solid,
        }
        .into());
    }
    let (direction, reference_d) = match topo.face(reference)?.surface() {
        FaceSurface::Plane { .. } => {
            let normal = topo
                .face(reference)?
                .effective_plane_normal()
                .ok_or_else(|| reconstruction("selected planar face has no effective normal"))?;
            let anchor = face_anchor(topo, reference)?;
            (
                normal,
                normal.dot(anchor - remus_math::vec::Point3::new(0.0, 0.0, 0.0)),
            )
        }
        surface => {
            return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                face: reference,
                surface_type: surface.type_tag(),
                reason: "blend-aware translation requires planar selected faces".into(),
            }
            .into());
        }
    };
    for &face in faces.iter().skip(1) {
        let FaceSurface::Plane { .. } = topo.face(face)?.surface() else {
            return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                face,
                surface_type: topo.face(face)?.surface().type_tag(),
                reason: "blend-aware translation requires planar selected faces".into(),
            }
            .into());
        };
        let normal = topo
            .face(face)?
            .effective_plane_normal()
            .ok_or_else(|| reconstruction("selected planar face has no effective normal"))?;
        let anchor = face_anchor(topo, face)?;
        let d = normal.dot(anchor - remus_math::vec::Point3::new(0.0, 0.0, 0.0));
        if normal.dot(direction) < 1.0 - Tolerance::new().angular
            || !Tolerance::new().approx_eq(d, reference_d)
        {
            return Err(remus_offset::OffsetError::MoveGroupMismatch {
                reference,
                face,
                reason: "selected faces are not coplanar with the same outward normal".into(),
            }
            .into());
        }
    }

    let region = translation_invariant_blend_region(topo, solid, faces, direction)?;
    let delta = direction * distance;
    let source_counts = remus_topology::explorer::solid_entity_counts(topo, solid)?;
    let mut work = topo.clone();
    let mut moved_faces: HashSet<FaceId> = faces.iter().copied().collect();
    moved_faces.extend(region.translated_faces);
    moved_faces.extend(region.translated_nurbs_supports);
    let adjacency = work.build_adjacency(solid)?;
    let invariant_carriers =
        invariant_boundary_carriers(&work, &adjacency, &moved_faces, direction)?;
    let swept_faces = faces.iter().copied().collect();
    let excluded_candidates = moved_faces.union(&invariant_carriers).copied().collect();
    crate::push_pull::refuse_swept_region_intersections(
        topo,
        solid,
        &swept_faces,
        &excluded_candidates,
        direction,
        distance,
    )?;

    let mut moved_vertices = HashSet::new();
    for &face in &moved_faces {
        moved_vertices.extend(remus_topology::explorer::face_vertices(&work, face)?);
    }
    // `solid_edges` yields each edge once, so this stays duplicate-free.
    let mut translated_edges = Vec::new();
    for edge in remus_topology::explorer::solid_edges(&work, solid)? {
        let edge_data = work.edge(edge)?;
        let start_moves = moved_vertices.contains(&edge_data.start());
        let end_moves = moved_vertices.contains(&edge_data.end());
        if start_moves || end_moves {
            for adjacent in distinct_faces(adjacency.faces_for_edge(edge)) {
                if !moved_faces.contains(&adjacent)
                    && !surface_translation_invariant(work.face(adjacent)?.surface(), direction)
                {
                    return Err(reconstruction(format!(
                        "boundary edge {} reaches non-invariant unmoved face {}",
                        edge.index(),
                        adjacent.index()
                    )));
                }
            }
        }
        match (start_moves, end_moves) {
            (true, true) => {
                translated_edges.push(edge);
            }
            (true, false) | (false, true) => {
                if !matches!(edge_data.curve(), EdgeCurve::Line) {
                    return Err(reconstruction(format!(
                        "boundary edge {} would change one endpoint of a curved edge",
                        edge.index()
                    )));
                }
                let span =
                    work.vertex(edge_data.end())?.point() - work.vertex(edge_data.start())?.point();
                if span.cross(direction).length() > Tolerance::new().linear {
                    return Err(reconstruction(format!(
                        "boundary edge {} is not parallel to the planar move",
                        edge.index()
                    )));
                }
            }
            (false, false) => {}
        }
    }

    for vertex in moved_vertices {
        let point = work.vertex(vertex)?.point();
        work.vertex_mut(vertex)?.set_point(point + delta);
    }
    let matrix = remus_math::mat::Mat4::translation(delta.x(), delta.y(), delta.z());
    crate::transform::transform_edges(&mut work, &translated_edges, &matrix)?;
    for &face in &moved_faces {
        translate_face_surface(&mut work, face, delta)?;
    }
    // Validate the deliverable rather than the scratch body: the copy is
    // entity-for-entity, so one strict pass serves both, and running it here
    // keeps validation ahead of the count and volume refusals as before. A
    // refusal is reported against the scratch body, whose ids are the input's
    // and stay live after the rollback; the copy's do not.
    let copied = crate::copy::copy_solid_between_with_entity_map(&work, topo, solid)?;
    let result = copied.solid;
    if let Err(copy_error) = validate_exact_result(topo, result, "translation-invariant blend move")
    {
        validate_exact_result(&work, solid, "translation-invariant blend move")?;
        return Err(copy_error);
    }
    let result_counts = remus_topology::explorer::solid_entity_counts(&work, solid)?;
    if result_counts != source_counts {
        return Err(reconstruction(format!(
            "blend translation changed (faces, edges, vertices) from {source_counts:?} to {result_counts:?}"
        )));
    }
    // The body's volume change is read off the translated faces alone; meshing
    // the whole body twice for this one sign was most of the cost of a planar
    // move on NURBS-heavy imports. On that path the change is `distance` times
    // the region's shadow along the move, so the slack is that shadow against
    // the region's own area: only a degenerate region, whose shadow is
    // negligible next to its area, is refused, independent of body size. A
    // region the integral cannot vouch for keeps the whole-body mesh
    // comparison with its original slack, so those bodies behave as before.
    let (volume_change, volume_slack) = if let Some(read) =
        translated_faces_volume_change(topo, &work, solid, &moved_faces, Some, delta)?
    {
        (
            read.volume,
            (read.area * 1e-9).mul_add(distance.abs(), 1e-7),
        )
    } else {
        let source_volume = crate::measure::solid_volume(topo, solid, 0.05)?;
        let result_volume = crate::measure::solid_volume(&work, solid, 0.05)?;
        (
            result_volume - source_volume,
            source_volume.abs().mul_add(1e-9, 1e-7),
        )
    };
    if volume_change.abs() <= volume_slack
        || volume_change.is_sign_positive() != distance.is_sign_positive()
    {
        return Err(reconstruction(format!(
            "blend-aware move changed volume by {volume_change}, inconsistent with distance {distance}"
        )));
    }

    let boundary_pairs =
        copied
            .edge_map
            .into_iter()
            .map(|(source, target)| (EntityKey::edge(source), EntityKey::edge(target.index())))
            .chain(copied.vertex_map.into_iter().map(|(source, target)| {
                (EntityKey::vertex(source), EntityKey::vertex(target.index()))
            }))
            .collect();
    Ok(BlendMoveEntities {
        solid: result,
        evolution: crate::push_pull::exact_face_evolution(
            topo,
            &source_faces,
            result,
            copied.face_map,
        )?,
        boundary_pairs,
    })
}

/// Everything a run leaves behind, in canonical order.
#[derive(Debug, PartialEq)]
enum Outcome {
    Refused(String),
    Moved {
        solid: usize,
        evolution: String,
        boundary_pairs: Vec<(EntityKey, EntityKey)>,
    },
}

fn outcome(result: Result<BlendMoveEntities, OperationsError>) -> Outcome {
    match result {
        Err(error) => Outcome::Refused(error.to_string()),
        Ok(moved) => {
            let ev = &moved.evolution;
            let mut modified: Vec<_> = ev.modified.iter().collect();
            modified.sort();
            let mut generated: Vec<_> = ev.generated.iter().collect();
            generated.sort();
            let mut deleted: Vec<_> = ev.deleted.iter().collect();
            deleted.sort();
            let mut boundary_pairs = moved.boundary_pairs;
            boundary_pairs.sort_unstable();
            Outcome::Moved {
                solid: moved.solid.index(),
                evolution: format!(
                    "{modified:?} {generated:?} {deleted:?} {:?} {:?}",
                    ev.unresolved, ev.origin
                ),
                boundary_pairs,
            }
        }
    }
}

/// The memos a WASM kernel turns on, emptied so each run starts alike.
fn set_memos(on: bool) {
    use remus_check::properties::face_cache;
    if on {
        face_cache::enable_thread_face_cache();
        crate::measure::enable_thread_volume_memo();
        crate::measure::enable_thread_nurbs_area_memo();
        crate::measure::enable_thread_face_box_memo();
        crate::tessellate::enable_face_mesh_cache();
        face_cache::clear_thread_face_cache();
        crate::measure::clear_thread_volume_memo();
        crate::measure::clear_thread_nurbs_area_memo();
        crate::measure::clear_thread_face_box_memo();
        crate::tessellate::clear_face_mesh_cache();
    } else {
        face_cache::set_thread_face_cache_limits(0, 0);
        crate::measure::set_thread_volume_memo_capacity(0);
        crate::measure::disable_thread_nurbs_area_memo();
        crate::measure::disable_thread_face_box_memo();
        crate::tessellate::disable_face_mesh_cache();
    }
}

struct Reset;

impl Drop for Reset {
    fn drop(&mut self) {
        set_memos(false);
        FORCE_MESH_VOLUME.with(|force| force.set(false));
    }
}

type Mover =
    fn(&mut Topology, SolidId, &[FaceId], f64) -> Result<BlendMoveEntities, OperationsError>;

fn run(
    source: &Topology,
    mover: Mover,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
    memos: bool,
) -> (Outcome, String) {
    set_memos(memos);
    let mut topo = source.clone();
    let _scope = DiscoveryMemoScope::enter();
    let result = outcome(mover(&mut topo, solid, faces, distance));
    (result, format!("{topo:?}"))
}

fn assert_same_moves(
    source: &Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> usize {
    let _reset = Reset;
    let mut moved = 0;
    for &distance in distances {
        for force in [false, true] {
            FORCE_MESH_VOLUME.with(|cell| cell.set(force));
            for memos in [false, true] {
                let legacy = run(
                    source,
                    legacy_move_translation_invariant_blend_region,
                    solid,
                    faces,
                    distance,
                    memos,
                );
                let current = run(
                    source,
                    move_translation_invariant_blend_region,
                    solid,
                    faces,
                    distance,
                    memos,
                );
                assert_eq!(
                    legacy.0, current.0,
                    "distance {distance}, mesh fallback {force}, memos {memos}"
                );
                assert!(
                    legacy.1 == current.1,
                    "document state differs: distance {distance}, mesh fallback {force}, memos {memos}"
                );
                moved += usize::from(matches!(current.0, Outcome::Moved { .. }));
            }
        }
    }
    moved
}

fn face_facing(topo: &Topology, solid: SolidId, axis: Vec3) -> FaceId {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(axis) > 0.99)
        })
        .unwrap()
}

#[test]
fn translation_matches_the_scratch_document_path_on_an_analytic_band() {
    let (topo, solid, support, _, _) = super::tests::filleted_cube_support_and_band();
    let moved = assert_same_moves(&topo, solid, &[support], &[1.0, 0.25, -0.5, -4.0, 1e-9]);
    assert!(moved >= 8, "{moved} runs moved");
}

#[test]
fn translation_matches_the_scratch_document_path_on_side_bands() {
    // Four vertical edges blended: pulling a side drags its two bands along
    // between the invariant top, bottom and neighbouring sides.
    let mut topo = Topology::new();
    let sharp = crate::primitives::make_box(&mut topo, 10.0, 8.0, 6.0).unwrap();
    let vertical: Vec<EdgeId> = remus_topology::explorer::solid_edges(&topo, sharp)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let edge = topo.edge(edge).unwrap();
            let span = topo.vertex(edge.end()).unwrap().point()
                - topo.vertex(edge.start()).unwrap().point();
            span.cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
        })
        .collect();
    assert_eq!(vertical.len(), 4);
    let solid = fillet_v2(&mut topo, sharp, &vertical, 1.5).unwrap().solid;
    for axis in [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0)] {
        let face = face_facing(&topo, solid, axis);
        let moved = assert_same_moves(&topo, solid, &[face], &[2.0, -1.0, -3.5]);
        assert!(moved >= 8, "{moved} runs moved");
    }
}

#[test]
fn translation_matches_the_scratch_document_path_on_a_nurbs_band() {
    // The band is not analytic, so the region is empty and both refuse alike.
    let (topo, solid, support, _, _) = super::tests::filleted_cube_with_nurbs_band();
    assert_eq!(
        assert_same_moves(&topo, solid, &[support], &[0.75, -0.5]),
        0
    );
}

#[test]
fn translation_matches_the_scratch_document_path_among_other_bodies() {
    let (mut topo, solid, support, _, _) = super::tests::filleted_cube_support_and_band();
    for k in 0..40 {
        let other = crate::primitives::make_box(&mut topo, 2.0, 1.0, 1.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            other,
            &remus_math::mat::Mat4::translation(20.0 + f64::from(k) * 3.0, 0.0, 0.0),
        )
        .unwrap();
    }
    let moved = assert_same_moves(&topo, solid, &[support], &[1.0, -0.5]);
    assert!(moved >= 4, "{moved} runs moved");
}

#[test]
fn translation_refusals_match_the_scratch_document_path() {
    // Every edge blended: the corner patches are not translation-invariant,
    // and every face of the box refuses with the face it names.
    let mut topo = Topology::new();
    let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(&topo, sharp).unwrap();
    let solid = fillet_v2(&mut topo, sharp, &edges, 1.0).unwrap().solid;
    for axis in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ] {
        let face = face_facing(&topo, solid, axis);
        assert_same_moves(&topo, solid, &[face], &[1.0, -0.5]);
    }
    // A selection holding the (non-planar) band itself.
    let (topo, solid, support, band, _) = super::tests::filleted_cube_support_and_band();
    assert_same_moves(&topo, solid, &[support, band], &[1.0]);
}

/// The edge images written into the copy are `transform_edges`'s, field for
/// field, on every curve kind the move translates, with and without trims.
#[test]
fn edge_images_match_transform_edges() {
    use remus_math::curves::{Circle3D, Ellipse3D};
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_topology::edge::Edge;
    use remus_topology::vertex::Vertex;

    let mut topo = Topology::new();
    let a = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
    let b = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let ellipse = Ellipse3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        1.0,
    )
    .unwrap();
    let nurbs = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
    )
    .unwrap();
    let curves = [
        (EdgeCurve::Line, None),
        (EdgeCurve::Line, Some((0.0, 1.0))),
        (
            EdgeCurve::Circle(circle.clone()),
            Some((0.0, std::f64::consts::FRAC_PI_2)),
        ),
        (
            EdgeCurve::Circle(circle),
            Some((std::f64::consts::FRAC_PI_2, 0.0)),
        ),
        (
            EdgeCurve::Ellipse(ellipse),
            Some((0.0, std::f64::consts::FRAC_PI_2)),
        ),
        (EdgeCurve::NurbsCurve(nurbs.clone()), Some((0.0, 1.0))),
        (EdgeCurve::NurbsCurve(nurbs), Some((0.25, 0.75))),
    ];
    let mut edges = Vec::new();
    for (curve, trim) in curves {
        let mut edge = Edge::new(a, b, curve);
        edge.set_trim(trim);
        edges.push(topo.add_edge(edge));
    }
    for delta in [Vec3::new(0.5, -2.0, 0.125), Vec3::new(-1e-3, 0.0, 7.0)] {
        let matrix = remus_math::mat::Mat4::translation(delta.x(), delta.y(), delta.z());
        let mut moved = topo.clone();
        crate::transform::transform_edges(&mut moved, &edges, &matrix).unwrap();
        for &edge in &edges {
            let (curve, trim) =
                crate::transform::transformed_edge_geometry(topo.edge(edge).unwrap(), &matrix)
                    .unwrap();
            let written = moved.edge(edge).unwrap();
            assert_eq!(format!("{curve:?}"), format!("{:?}", written.curve()));
            assert_eq!(
                trim.map(|(t0, t1)| (t0.to_bits(), t1.to_bits())),
                written.trim().map(|(t0, t1)| (t0.to_bits(), t1.to_bits()))
            );
        }
    }
}
