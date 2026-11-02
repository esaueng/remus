//! Explicitly approximate reconstruction after a shoulder reaches a cap rim.

use remus_blend::cap_reblend::{
    CapReblendApproximation, reblend_planar_cap_group, sampled_section_normal,
    sampled_section_radius,
};
use remus_topology::explorer::{face_edges, solid_edges, solid_vertices};

use super::{
    BlendMoveEntities, BlendTranslation, TranslatedEntities, analyze_translation, axial_clearance,
    check_translation_distance, reconstruction, surface_translation_invariant, tangent_across,
    translated_solid_copy, validate_exact_result,
};
use crate::OperationsError;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::journal::EntityKey;
use remus_topology::{FaceId, SolidId, Topology};
use std::collections::{HashMap, HashSet, VecDeque};

pub fn move_with_cap_reblend(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
    permission: CapReblendApproximation,
) -> Result<BlendMoveEntities, OperationsError> {
    check_translation_distance(distance)?;
    let _total = crate::performance::span("move_faces.cap_join");
    let plan = original_plan(topo, solid, faces, distance)
        .or_else(|_| joined_plan(topo, solid, faces, distance))?;
    let translation = BlendTranslation {
        vertices: plan.entities.vertices,
        edges: plan.entities.edges,
        faces: plan.entities.faces,
        delta: plan.direction * distance,
    };
    let source_volume = crate::performance::timed("move_faces.cap_join.source_volume", || {
        crate::measure::solid_volume(topo, solid, 0.05)
    })?;
    let mut copied = translated_solid_copy(topo, solid, &translation)?;
    let image = |face: FaceId| {
        copied
            .face_map
            .get(&face.index())
            .copied()
            .ok_or_else(|| reconstruction("translation lost a required cap face"))
    };
    let cap_image = image(plan.cap)?;
    let rim_images = plan
        .rims
        .iter()
        .copied()
        .map(image)
        .collect::<Result<Vec<_>, _>>()?;
    let touching = remus_blend::cap_reblend::restore_touching_support_limits_in_place(
        topo,
        copied.solid,
        &rim_images,
        plan.direction,
    )?;
    let extended = if touching.is_some() {
        touching
    } else if distance < 0.0 && rim_images.len() > 1 {
        let extension = remus_blend::cap_reblend::restore_joined_support_limits_in_place(
            topo,
            copied.solid,
            &rim_images,
            plan.direction,
        )?;
        for target in copied.edge_map.values_mut() {
            if let Some((_, image)) = extension
                .edge_images
                .iter()
                .find(|(source, _)| source == target)
            {
                *target = *image;
            }
        }
        Some(extension)
    } else {
        None
    };
    let separate = if let Some(extension) = &extended {
        let FaceSurface::Torus(shoulder) = topo.face(extension.shoulder)?.surface() else {
            return Err(reconstruction("restored shoulder changed carrier"));
        };
        let FaceSurface::Plane { normal, d } = topo.face(cap_image)?.surface() else {
            return Err(reconstruction("restored cap changed carrier"));
        };
        let level = *d * normal.dot(plan.direction) - plan.radius;
        (shoulder.center() - remus_math::vec::Point3::new(0.0, 0.0, 0.0)).dot(plan.direction)
            < level - 1e-7
    } else {
        false
    };
    let rebuilt = crate::performance::timed("move_faces.cap_join.rebuild", || {
        if let Some(touching) = remus_blend::cap_reblend::rebuild_touching_cap_in_place(
            topo,
            copied.solid,
            cap_image,
            &rim_images,
            plan.radius,
            permission,
        )? {
            Ok(touching)
        } else if separate {
            let extension =
                extended
                    .as_ref()
                    .ok_or_else(|| remus_blend::BlendError::InvalidInput {
                        reason: "separated cap has no restored lower limits".into(),
                    })?;
            remus_blend::cap_reblend::restore_unjoined_cap_in_place(
                topo,
                copied.solid,
                cap_image,
                &rim_images,
                plan.radius,
                extension,
                permission,
            )
        } else {
            reblend_planar_cap_group(
                topo,
                copied.solid,
                cap_image,
                &rim_images,
                plan.radius,
                permission,
            )
        }
    })?;
    let result = rebuilt.result.solid;
    crate::performance::timed("move_faces.cap_join.validate", || {
        validate_exact_result(topo, result, "cap contour reconstruction")
    })?;
    let volume = crate::performance::timed("move_faces.cap_join.result_volume", || {
        crate::measure::solid_volume(topo, result, 0.05)
    })? - source_volume;
    if volume * distance <= 0.0 || volume.abs() <= source_volume.abs().mul_add(1e-9, 1e-7) {
        return Err(reconstruction(
            "cap reconstruction has inconsistent volume change",
        ));
    }
    crate::push_pull::validate_moved_solid(topo, faces, result)?;
    let inverse: HashMap<_, _> = copied
        .face_map
        .iter()
        .map(|(&source, &copy)| (copy, source))
        .collect();
    let mut origins = rebuilt
        .result
        .face_origins
        .ok_or_else(|| reconstruction("cap reconstruction omitted face origins"))?;
    let original = |face: FaceId| {
        inverse
            .get(&face)
            .copied()
            .and_then(|index| {
                plan.source_faces
                    .iter()
                    .find(|face| face.index() == index)
                    .copied()
            })
            .ok_or_else(|| reconstruction("cap origin is outside the translated source"))
    };
    for (source, _) in &mut origins.survived {
        *source = original(*source)?;
    }
    for source in &mut origins.deleted {
        *source = original(*source)?;
    }
    for (_, sources) in &mut origins.created {
        for source in sources {
            *source = original(*source)?;
        }
    }
    let evolution =
        crate::blend_ops::evolution_from_blend_origins(topo, result, Some(&origins), &[])?;
    let edges: HashSet<_> = solid_edges(topo, result)?.into_iter().collect();
    let vertices: HashSet<_> = solid_vertices(topo, result)?.into_iter().collect();
    let split: HashMap<_, Vec<_>> =
        rebuilt
            .edge_images
            .into_iter()
            .fold(HashMap::new(), |mut map, (source, child)| {
                map.entry(source).or_default().push(child);
                map
            });
    let mut boundary_pairs = Vec::new();
    for (source, target) in copied.edge_map {
        if edges.contains(&target) {
            boundary_pairs.push((EntityKey::edge(source), EntityKey::edge(target.index())));
        } else if let Some(children) = split.get(&target) {
            for child in children {
                boundary_pairs.push((EntityKey::edge(source), EntityKey::edge(child.index())));
            }
        }
    }
    boundary_pairs.extend(
        copied
            .vertex_map
            .into_iter()
            .filter(|(_, target)| vertices.contains(target))
            .map(|(source, target)| (EntityKey::vertex(source), EntityKey::vertex(target.index()))),
    );
    log::debug!(target: "remus_approx", "move_faces: sampled cap reconstruction uses {} stations per corner", permission.stations);
    Ok(BlendMoveEntities {
        solid: result,
        evolution,
        boundary_pairs,
    })
}

struct CapJoinPlan {
    source_faces: Vec<FaceId>,
    direction: Vec3,
    cap: FaceId,
    rims: Vec<FaceId>,
    radius: f64,
    entities: TranslatedEntities,
}

fn original_plan(
    topo: &Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<CapJoinPlan, OperationsError> {
    let analysis = crate::performance::timed("move_faces.cap_join.analysis", || {
        analyze_translation(topo, solid, faces)
    })?;
    analysis.swept.check(distance)?;
    let reached: HashSet<_> = analysis
        .axial_clearance
        .iter()
        .filter(|limit| limit.reached(distance))
        .map(axial_clearance::AxialBlendClearance::fixed)
        .collect();
    let reached: Vec<_> = reached.into_iter().collect();
    let [rim] = reached.as_slice() else {
        return Err(reconstruction(
            "cap reconstruction requires one reached fixed rim",
        ));
    };
    let rim = *rim;
    let FaceSurface::Torus(torus) = topo.face(rim)?.surface() else {
        return Err(reconstruction("fixed rim is not toroidal"));
    };
    let radius = torus.minor_radius();
    let adjacency = topo.build_adjacency(solid)?;
    let caps: HashSet<_> = face_edges(topo, rim)?
        .into_iter()
        .flat_map(|edge| adjacency.faces_for_edge(edge).iter().copied())
        .filter(|&face| {
            topo.face(face).is_ok_and(|face| {
                face.effective_plane_normal().is_some_and(|normal| {
                    normal.dot(analysis.direction) > 1.0 - Tolerance::new().angular
                })
            })
        })
        .collect();
    let caps: Vec<_> = caps.into_iter().collect();
    let [cap] = caps.as_slice() else {
        return Err(reconstruction(
            "fixed rim does not have one parallel planar cap",
        ));
    };
    let cap = *cap;
    let entities = analysis
        .entities
        .as_ref()
        .map_err(|reason| reconstruction(reason.clone()))?;
    Ok(CapJoinPlan {
        source_faces: analysis.source_faces,
        direction: analysis.direction,
        cap,
        rims: vec![rim],
        radius,
        entities: TranslatedEntities {
            vertices: entities.vertices.clone(),
            edges: entities.edges.clone(),
            faces: entities.faces.clone(),
        },
    })
}

fn section_radius(topo: &Topology, face: FaceId) -> Option<f64> {
    match topo.face(face).ok()?.surface() {
        FaceSurface::Cylinder(surface) => Some(surface.radius()),
        FaceSurface::Torus(surface) => Some(surface.minor_radius()),
        FaceSurface::Nurbs(_) => sampled_section_radius(topo.face(face).ok()?).ok(),
        _ => None,
    }
}

fn section_contact_is_tangent(
    topo: &Topology,
    edge: remus_topology::EdgeId,
    a: FaceId,
    b: FaceId,
) -> Result<bool, OperationsError> {
    let data = topo.edge(edge)?;
    let (lo, hi) = crate::authoritative_edge_domain(data, "sampled cap contact")?;
    let start = topo.vertex(data.start())?.point();
    let end = topo.vertex(data.end())?.point();
    for fraction in [0.25, 0.5, 0.75] {
        let point =
            data.curve()
                .evaluate_with_endpoints((hi - lo).mul_add(fraction, lo), start, end);
        let normal = |face| -> Result<Option<Vec3>, OperationsError> {
            let f = topo.face(face)?;
            Ok(if matches!(f.surface(), FaceSurface::Nurbs(_)) {
                sampled_section_normal(f, point).ok()
            } else {
                crate::query::effective_face_normal(topo, face, point)
            })
        };
        let (Some(a), Some(b)) = (normal(a)?, normal(b)?) else {
            return Ok(false);
        };
        if 1.0 - a.dot(b) > 1e-10 {
            return Ok(false);
        }
    }
    Ok(true)
}

/// A previously joined cap is a fixed group of bands. Grow the moving
/// shoulder by its section radius, stopping at that complete fixed group and
/// at invariant supports. Every retained boundary must still translate or
/// slide exactly; only edges entirely inside the consumed cap group may be
/// temporarily inconsistent in the disposable reconstruction candidate.
fn joined_plan(
    topo: &Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<CapJoinPlan, OperationsError> {
    let [selected] = faces else {
        return Err(reconstruction(
            "joined cap edits require one planar shoulder",
        ));
    };
    let source_faces = remus_topology::explorer::solid_faces(topo, solid)?;
    if !source_faces.contains(selected) {
        return Err(reconstruction("selected shoulder is outside the source"));
    }
    let direction = topo
        .face(*selected)?
        .effective_plane_normal()
        .ok_or_else(|| reconstruction("joined shoulder must be planar"))?;
    let adjacency = topo.build_adjacency(solid)?;
    let neighbours = |face| -> Result<HashSet<FaceId>, OperationsError> {
        Ok(face_edges(topo, face)?
            .into_iter()
            .flat_map(|edge| adjacency.faces_for_edge(edge).iter().copied())
            .filter(|&other| other != face)
            .collect())
    };
    let seed_faces: Vec<_> = neighbours(*selected)?
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .is_ok_and(|data| !surface_translation_invariant(data.surface(), direction))
        })
        .collect();
    let shoulder_radius = seed_faces
        .first()
        .and_then(|&face| section_radius(topo, face))
        .ok_or_else(|| reconstruction("selected shoulder has no qualified section radius"))?;
    if seed_faces.iter().any(|&face| {
        section_radius(topo, face).is_none_or(|radius| (radius - shoulder_radius).abs() > 1e-7)
    }) {
        return Err(reconstruction(
            "selected shoulder has mixed or unsupported section radii",
        ));
    }
    let mut caps = Vec::new();
    for &cap in &source_faces {
        if cap == *selected
            || !topo
                .face(cap)?
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(direction) > 1.0 - 1e-10)
        {
            continue;
        }
        let bands: Vec<_> = neighbours(cap)?.into_iter().collect();
        if !(1..=12).contains(&bands.len()) {
            continue;
        }
        let Some(radius) = bands
            .iter()
            .find_map(|&face| match topo.face(face).ok()?.surface() {
                FaceSurface::Torus(torus) if torus.z_axis().cross(direction).length() < 1e-10 => {
                    Some(torus.minor_radius())
                }
                _ => None,
            })
        else {
            continue;
        };
        if radius >= shoulder_radius
            || bands
                .iter()
                .any(|&face| section_radius(topo, face).is_none_or(|r| (r - radius).abs() > 1e-7))
        {
            continue;
        }
        caps.push((cap, bands, radius));
    }
    let [(cap, rims, radius)] = caps.as_slice() else {
        return Err(reconstruction(
            "joined shoulder does not have one qualified cap group",
        ));
    };
    let mut moved = HashSet::from([*selected]);
    let mut queue = VecDeque::from([*selected]);
    while let Some(current) = queue.pop_front() {
        for edge in face_edges(topo, current)? {
            for &face in adjacency.faces_for_edge(edge) {
                if face == current || moved.contains(&face) || rims.contains(&face) {
                    continue;
                }
                let surface = topo.face(face)?.surface();
                if surface_translation_invariant(surface, direction) {
                    continue;
                }
                if face == *cap
                    || section_radius(topo, face).is_none_or(|r| (r - shoulder_radius).abs() > 1e-7)
                    || !section_contact_is_tangent(topo, edge, current, face)?
                {
                    return Err(reconstruction(format!(
                        "joined shoulder {current:?} reaches unqualified support {face:?} across {edge:?}; radius {:?}; tangency {:?}",
                        section_radius(topo, face),
                        tangent_across(topo, edge, current, face)
                    )));
                }
                moved.insert(face);
                queue.push_back(face);
            }
        }
    }
    let mut vertices = HashSet::new();
    for &face in &moved {
        vertices.extend(remus_topology::explorer::face_vertices(topo, face)?);
    }
    let mut edges = Vec::new();
    let mut invariant = HashSet::new();
    for edge in solid_edges(topo, solid)? {
        let data = topo.edge(edge)?;
        let a = vertices.contains(&data.start());
        let b = vertices.contains(&data.end());
        if !a && !b {
            continue;
        }
        let adjacent = adjacency.faces_for_edge(edge);
        let consumed = adjacent.iter().all(|face| rims.contains(face))
            || (rims.len() == 1
                && adjacent.iter().any(|face| rims.contains(face))
                && adjacent.iter().all(|face| {
                    rims.contains(face)
                        || topo.face(*face).is_ok_and(|face| {
                            surface_translation_invariant(face.surface(), direction)
                        })
                }));
        for &face in adjacent {
            if moved.contains(&face) || rims.contains(&face) {
                continue;
            }
            if !surface_translation_invariant(topo.face(face)?.surface(), direction) {
                return Err(reconstruction(
                    "moving shoulder has a fixed non-invariant boundary",
                ));
            }
            invariant.insert(face);
        }
        if a && b {
            edges.push(edge);
        } else if !consumed {
            let span = topo.vertex(data.end())?.point() - topo.vertex(data.start())?.point();
            if !matches!(data.curve(), EdgeCurve::Line)
                || span.cross(direction).length() > Tolerance::new().linear
            {
                return Err(reconstruction(
                    "moving shoulder would stretch a retained curved boundary",
                ));
            }
        }
    }
    let mut excluded = moved.union(&invariant).copied().collect::<HashSet<_>>();
    excluded.extend(rims);
    let selected_faces = faces.iter().copied().collect();
    let swept = crate::push_pull::SweptRegionCheck::prepare(
        topo,
        solid,
        &selected_faces,
        &excluded,
        direction,
    )?;
    swept.check(distance)?;
    Ok(CapJoinPlan {
        source_faces,
        direction,
        cap: *cap,
        rims: rims.clone(),
        radius: *radius,
        entities: TranslatedEntities {
            vertices,
            edges,
            faces: moved,
        },
    })
}
