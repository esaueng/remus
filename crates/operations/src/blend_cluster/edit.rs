//! Direct edits through a recognized blend group: recognize the group,
//! remove it to sharp, edit the sharp body exactly, re-fillet in planned
//! order.
//!
//! Three edits take this route when their per-band paths refuse:
//!
//! - [`resize_band_through_group`]: a band's radius
//!   ([`crate::resize_blend::resize_blend`]). The whole group is removed and
//!   re-filleted with the band at its new radius; every other member keeps
//!   its recorded radius.
//! - [`resize_cylinder_through_group`]: a cylindrical support's radius
//!   ([`crate::push_pull::resize_cylindrical_face`]), such as a boss whose
//!   seams, shoulder and the block edges running into them are filleted. The
//!   group is removed, the sharp wall is resized by the exact partial-turn
//!   path, the restored sharp edges are carried through that edit's edge
//!   history, and the group is re-filleted at its recorded radii.
//! - [`fillet_through_blend_group`]: a new fillet whose tangent chain runs
//!   into a group ([`crate::blend_ops::fillet_cascade`], after the engines
//!   and the smaller-band reorder refuse). The group is removed, the new
//!   edges are found again on the sharp body through the removal's edge
//!   history, and they join the re-fillet plan as one more stage, placed in
//!   the planner's order (concave before convex, larger radii first).
//!
//! The whole route runs on a private copy of the solid; only the finished,
//! verified result is copied into the caller's arena (inside a
//! transaction). Any refusal (the sharp edit, an unbuildable radius set, a
//! postcondition) leaves the caller's topology exactly as it was, handle
//! slots included, and no intermediate body is left behind on success. A
//! group that is not recognized, or cannot be removed to sharp, is not the
//! route's to answer: it returns `None` and the caller keeps its own refusal.
//!
//! Face evolution is composed source to sharp to edited to result: member
//! faces with a unique recognized successor retain their identity by support
//! incidence. These correspondences are geometric inferences, never claimed
//! as construction history. Ambiguous members remain deleted/generated.
//!
//! WASM module tier: `core` (reached from `resizeBlend` and
//! `resizeCylindricalFace`).

use std::cell::Cell;
use std::collections::BTreeMap;

use remus_blend::{BlendEngine, BlendResult};
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::journal::{EntityKey, EntityKind};
use remus_topology::solid::SolidId;

use super::reblend::{ReblendStage, compose_evolution, plan_reblend, reblend_restoration};
use super::{
    BlendCluster, BlendClusterError, BlendMemberKind, Convexity, StageBlend,
    recognize_blend_cluster, remove_blend_cluster,
};
use crate::OperationsError;
use crate::copy::{CopiedSolidEntities, copy_solid_between_with_entity_map};
use crate::evolution::EvolutionMap;
use crate::resize_blend::{ResizeBlendError, ResizeBlendResult};

/// A direct edit that went through a blend group.
#[derive(Debug, Clone)]
pub struct GroupEdit {
    /// The edited solid.
    pub solid: SolidId,
    /// Face evolution from the input solid to [`Self::solid`].
    pub evolution: EvolutionMap,
    /// The re-fillet plan that rebuilt the group.
    pub stages: Vec<ReblendStage>,
    /// The engine each stage ran on.
    pub engines: Vec<BlendEngine>,
}

/// What a route built on the private arena.
struct Built {
    solid: SolidId,
    /// The face on the private copy the group was recognized from.
    seed: FaceId,
    evolution: EvolutionMap,
    stages: Vec<ReblendStage>,
    engines: Vec<BlendEngine>,
}

/// Change the radius of the band containing `face` by removing its whole
/// blend group to sharp and re-filleting it with the band at `new_radius`.
///
/// Returns `Ok(None)` when `face` is not a band of a recognized group, or
/// the group cannot be removed to its sharp parent (the caller keeps its own
/// refusal). `expected_radius` must match the band's measured radius.
///
/// # Errors
///
/// [`ResizeBlendError::RadiusMismatch`] for a wrong witness,
/// [`BlendClusterError::InvalidReblend`] for a non-positive radius, and
/// [`BlendClusterError::ReblendFailed`] when the radius set does not build.
/// The topology is unchanged on every error.
pub fn resize_band_through_group(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    expected_radius: f64,
    new_radius: f64,
) -> Result<Option<ResizeBlendResult>, OperationsError> {
    let edit = in_private_arena(topo, solid, face, |work, solid, face| {
        let Ok(cluster) = recognize_blend_cluster(work, solid, face) else {
            return Ok(None);
        };
        let Some(member) = cluster.member_of(face) else {
            return Ok(None);
        };
        let band = &cluster.members[member];
        if band.kind != BlendMemberKind::Band {
            return Ok(None);
        }
        let tolerance = Tolerance::new();
        if !tolerance.approx_eq(expected_radius, band.radius) {
            return Err(ResizeBlendError::RadiusMismatch {
                expected: expected_radius,
                actual: band.radius,
            }
            .into());
        }
        if !new_radius.is_finite() || new_radius <= tolerance.linear {
            return Err(BlendClusterError::InvalidReblend {
                reason: format!("band radius must be finite and positive, got {new_radius}"),
            }
            .into());
        }
        let radii = BTreeMap::from([(member, new_radius)]);
        // A group that cannot be removed to sharp is not this route's to
        // answer: the caller keeps its own refusal.
        let Ok(restoration) = remove_blend_cluster(work, solid, &cluster) else {
            return Ok(None);
        };
        let plans = plan_reblend(&cluster, &restoration.removed, &radii)?;
        let rebuilt =
            reblend_restoration(work, restoration.solid, &restoration.sharp_edges, &plans)?;
        Ok(Some(Built {
            solid: rebuilt.solid,
            seed: face,
            evolution: compose_evolution(&restoration.evolution, &rebuilt.evolution),
            stages: rebuilt.stages,
            engines: rebuilt.engines,
        }))
    })?;
    Ok(edit.map(|edit| ResizeBlendResult {
        solid: edit.solid,
        evolution: edit.evolution,
    }))
}

/// Resize the cylindrical support `face` to `new_radius` through the blend
/// group it supports: remove the group to sharp, resize the sharp wall
/// exactly, and re-fillet the group at its recorded radii.
///
/// Returns `Ok(None)` when no blend group is recognized on `face`, when
/// `face` is itself a blend, or when the group cannot be removed to its
/// sharp parent (the caller keeps its own refusal).
///
/// # Errors
///
/// The exact sharp resize's refusals (a wall that would meet another face is a typed topology
/// change), and [`BlendClusterError::ReblendFailed`] when the group does not
/// build on the resized body. The topology is unchanged on every error.
pub fn resize_cylinder_through_group(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    new_radius: f64,
) -> Result<Option<GroupEdit>, OperationsError> {
    in_private_arena(topo, solid, face, |work, solid, face| {
        let Ok(cluster) = recognize_blend_cluster(work, solid, face) else {
            return Ok(None);
        };
        if cluster.seed_is_member {
            return Ok(None);
        }
        // A group that cannot be removed to sharp is not this route's to
        // answer: the caller keeps its own refusal.
        let Ok(restoration) = remove_blend_cluster(work, solid, &cluster) else {
            return Ok(None);
        };
        let wall = restoration
            .faces
            .iter()
            .find_map(|&(source, result)| (source == face).then_some(result))
            .ok_or_else(|| lost("the edited wall did not survive the group removal"))?;
        let (edited, pairs) = crate::push_pull::resize_cylindrical_face_with_entity_evolution(
            work,
            restoration.solid,
            wall,
            new_radius,
        )
        .map_err(|error| diameter_refusal(error, new_radius))?;
        let edge_images: BTreeMap<usize, usize> = pairs
            .iter()
            .filter(|(source, target)| {
                source.kind == EntityKind::Edge && target.kind == EntityKind::Edge
            })
            .map(|(source, target)| (source.index, target.index))
            .collect();
        let mut sharp_edges: Vec<Vec<EdgeId>> = Vec::with_capacity(restoration.sharp_edges.len());
        for edges in &restoration.sharp_edges {
            let mut carried = Vec::with_capacity(edges.len());
            for edge in edges {
                let image = edge_images
                    .get(&edge.index())
                    .and_then(|&index| work.edge_id_from_index(index))
                    .ok_or_else(|| {
                        lost(format!(
                            "restored sharp edge {} has no image through the resize",
                            edge.index()
                        ))
                    })?;
                carried.push(image);
            }
            sharp_edges.push(carried);
        }
        let plans = plan_reblend(&cluster, &restoration.removed, &BTreeMap::new())?;
        let rebuilt = reblend_restoration(work, edited.solid, &sharp_edges, &plans)
            .map_err(|error| diameter_refusal(error, new_radius))?;
        Ok(Some(Built {
            solid: rebuilt.solid,
            seed: face,
            evolution: compose_evolution(
                &compose_evolution(&restoration.evolution, &edited.evolution),
                &rebuilt.evolution,
            ),
            stages: rebuilt.stages,
            engines: rebuilt.engines,
        }))
    })
}

thread_local! {
    /// Set while [`fillet_through_blend_group`] re-fillets a group: its own
    /// stages run [`crate::blend_ops::fillet_cascade`], which must not route
    /// back into another group removal.
    static FILLETING_THROUGH_GROUP: Cell<bool> = const { Cell::new(false) };
}

/// Resets [`FILLETING_THROUGH_GROUP`] when the route ends, however it ends.
struct RouteGuard;

impl Drop for RouteGuard {
    fn drop(&mut self) {
        FILLETING_THROUGH_GROUP.with(|flag| flag.set(false));
    }
}

/// Whether a group fillet is already running on this thread.
fn filleting_through_group() -> bool {
    FILLETING_THROUGH_GROUP.with(Cell::get)
}

/// Fillet `edges` at `radius` through the blend group their tangent chain
/// runs into.
///
/// The group is removed to sharp, the edges are found again on the sharp
/// body, and the group is re-filleted with them as one more stage in the
/// planner's order.
///
/// Returns `Ok(None)` when the chain meets no recognized group (or meets
/// more than one), a selected edge borders a member or is consumed by the
/// removal, the group cannot be removed to sharp, or a group fillet is
/// already running on this thread. The result's face history is reported as
/// the removal, then the re-fillet; the topology is unchanged on every error.
///
/// # Errors
///
/// [`BlendClusterError::ReblendFailed`] when no planned order builds, and the
/// planner's own refusals. The topology is unchanged on every error.
pub fn fillet_through_blend_group(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<Option<GroupEdit>, OperationsError> {
    let tolerance = Tolerance::new();
    if edges.is_empty()
        || !radius.is_finite()
        || radius <= tolerance.linear
        || filleting_through_group()
    {
        return Ok(None);
    }
    FILLETING_THROUGH_GROUP.with(|flag| flag.set(true));
    let _guard = RouteGuard;
    run_private(topo, solid, |work, copied| {
        let mut selected = Vec::with_capacity(edges.len());
        for edge in edges {
            let Some(&mapped) = copied.edge_map.get(&edge.index()) else {
                return Ok(None);
            };
            selected.push(mapped);
        }
        group_fillet(work, copied.solid, &selected, radius)
    })
}

fn group_fillet(
    work: &mut Topology,
    solid: SolidId,
    selected: &[EdgeId],
    radius: f64,
) -> Result<Option<Built>, OperationsError> {
    let tolerance = Tolerance::new();
    let Ok(chain) = remus_blend::g1_chain::expand_g1_chain(work, solid, selected, tolerance) else {
        return Ok(None);
    };
    let adjacency = work.build_adjacency(solid)?;
    let mut touched: Vec<FaceId> = chain
        .iter()
        .flat_map(|&edge| adjacency.faces_for_edge(edge).iter().copied())
        .collect();
    touched.compact_sort_unstable_by_key(|face| face.index());
    touched.dedup();
    // The one group the chain runs into.
    let mut cluster: Option<BlendCluster> = None;
    for &face in &touched {
        if cluster
            .as_ref()
            .is_some_and(|cluster| cluster.member_of(face).is_some())
        {
            continue;
        }
        let Ok(found) = recognize_blend_cluster(work, solid, face) else {
            continue;
        };
        if !found.seed_is_member {
            continue;
        }
        if cluster.is_some() {
            return Ok(None);
        }
        cluster = Some(found);
    }
    let Some(cluster) = cluster else {
        return Ok(None);
    };
    // A selected edge bordering a member would not survive the removal.
    for &edge in selected {
        if adjacency
            .faces_for_edge(edge)
            .iter()
            .any(|&face| cluster.member_of(face).is_some())
        {
            return Ok(None);
        }
    }
    let Ok(restoration) = remove_blend_cluster(work, solid, &cluster) else {
        return Ok(None);
    };
    // The selected edges on the sharp body, through the removal's history.
    let mut sharp_selection = Vec::with_capacity(selected.len());
    for &edge in selected {
        let key = EntityKey {
            kind: EntityKind::Edge,
            index: edge.index(),
        };
        let Ok(at) = restoration
            .boundary_history
            .binary_search_by(|(source, _)| source.cmp(&key))
        else {
            return Ok(None);
        };
        let Some(image) = restoration.boundary_history[at]
            .1
            .and_then(|target| work.edge_id_from_index(target.index))
        else {
            return Ok(None);
        };
        sharp_selection.push(image);
    }
    let mut convexity = None;
    for &edge in &sharp_selection {
        let side = match crate::query::edge_concavity(work, restoration.solid, edge, 0.1 * radius)?
        {
            crate::query::EdgeConcavity::Convex => Convexity::Convex,
            crate::query::EdgeConcavity::Concave => Convexity::Concave,
            crate::query::EdgeConcavity::Tangent | crate::query::EdgeConcavity::Unknown => {
                return Ok(None);
            }
        };
        if convexity.is_some_and(|known| known != side) {
            return Ok(None);
        }
        convexity = Some(side);
    }
    let Some(convexity) = convexity else {
        return Ok(None);
    };

    let extra = cluster.sharp_edges.len();
    let mut sharp_edges = restoration.sharp_edges.clone();
    sharp_edges.push(sharp_selection);
    let stage = ReblendStage {
        radius,
        blend: StageBlend::Fillet,
        sharp_edges: vec![extra],
    };
    // The planner's order: concave before convex, larger radii first; the
    // new stage after the group's stages of its own side and radius, and
    // before every chamfer.
    let key = |convexity: Convexity, radius: f64| (convexity, -radius);
    let mut plans = plan_reblend(&cluster, &restoration.removed, &BTreeMap::new())?;
    for plan in &mut plans {
        let position = plan
            .iter()
            .position(|existing| match existing.blend {
                StageBlend::Chamfer { .. } => true,
                StageBlend::Fillet => {
                    let side = existing
                        .sharp_edges
                        .first()
                        .and_then(|&index| cluster.sharp_edges.get(index))
                        .map_or(convexity, |sharp| cluster.members[sharp.member].convexity);
                    key(side, existing.radius)
                        .partial_cmp(&key(convexity, radius))
                        .is_some_and(std::cmp::Ordering::is_gt)
                }
            })
            .unwrap_or(plan.len());
        plan.insert(position, stage.clone());
    }
    let rebuilt = reblend_restoration(work, restoration.solid, &sharp_edges, &plans)?;
    Ok(Some(Built {
        solid: rebuilt.solid,
        seed: cluster.seed,
        evolution: compose_evolution(&restoration.evolution, &rebuilt.evolution),
        stages: rebuilt.stages,
        engines: rebuilt.engines,
    }))
}

/// A group fillet as a [`BlendResult`]: every selected edge rounded, the
/// engine the stages agree on (or mixed), no construction face origins (the
/// group was rebuilt).
pub fn group_fillet_result(edges: &[EdgeId], edit: GroupEdit) -> BlendResult {
    let engine = match edit.engines.split_first() {
        Some((first, rest)) if rest.iter().all(|engine| engine == first) => *first,
        Some(_) => BlendEngine::Mixed,
        None => BlendEngine::RollingBall,
    };
    BlendResult {
        solid: edit.solid,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: None,
        engine,
    }
}

/// Run `route` on a private copy of `solid` (with `face` mapped onto the
/// copy), then copy its result into `topo` and translate its face evolution
/// from copy indices to the caller's.
fn in_private_arena(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    route: impl FnOnce(&mut Topology, SolidId, FaceId) -> Result<Option<Built>, OperationsError>,
) -> Result<Option<GroupEdit>, OperationsError> {
    run_private(topo, solid, |work, copied| {
        let Some(&work_face) = copied.face_map.get(&face.index()) else {
            return Ok(None);
        };
        route(work, copied.solid, work_face)
    })
}

/// Run `route` on a private copy of `solid`, then copy its result into
/// `topo` and translate its face evolution from copy indices to the
/// caller's.
fn run_private(
    topo: &mut Topology,
    solid: SolidId,
    route: impl FnOnce(&mut Topology, &CopiedSolidEntities) -> Result<Option<Built>, OperationsError>,
) -> Result<Option<GroupEdit>, OperationsError> {
    let mut work = Topology::new();
    let copied = copy_solid_between_with_entity_map(topo, &mut work, solid)?;
    let Some(built) = route(&mut work, &copied)? else {
        return Ok(None);
    };
    let mut inputs: Vec<(usize, usize)> = copied
        .face_map
        .iter()
        .map(|(&source, work)| (work.index(), source))
        .collect();
    inputs.compact_sort_unstable();
    let face = inputs
        .binary_search_by_key(&built.seed.index(), |&(work, _)| work)
        .ok()
        .and_then(|at| topo.face_id_from_index(inputs[at].1))
        .ok_or_else(|| lost("the group's seed face has no source on the edited solid"))?;
    let (output, evolution) = remus_topology::transaction::run_transacted(topo, |topo| {
        let output = copy_solid_between_with_entity_map(&work, topo, built.solid)?;
        let mut outputs: Vec<(usize, usize)> = output
            .face_map
            .iter()
            .map(|(&work, result)| (work, result.index()))
            .collect();
        outputs.compact_sort_unstable();
        let mut evolution = translate(&built.evolution, &inputs, &outputs);
        carry_member_identities(topo, solid, face, output.solid, &mut evolution)?;
        Ok::<_, OperationsError>((output, evolution))
    })?;
    Ok(Some(GroupEdit {
        solid: output.solid,
        evolution,
        stages: built.stages,
        engines: built.engines,
    }))
}

/// Match recognized members by their carrier kind and mapped supports.
/// Radius and proximity are deliberately not identity keys: this operation
/// edits radii, and neighbouring corner patches may be very close together.
fn carry_member_identities(
    topo: &Topology,
    source: SolidId,
    seed: FaceId,
    result: SolidId,
    map: &mut EvolutionMap,
) -> Result<(), OperationsError> {
    let before = recognize_blend_cluster(topo, source, seed)?;
    let outputs = remus_topology::explorer::solid_faces(topo, result)?;
    let adjacency = topo.build_adjacency(result)?;
    let source_adjacency = topo.build_adjacency(source)?;
    // Bands first acquire their mapped plane/cylinder supports, then corner
    // patches acquire the mapped bands. Recognizing the entire result again
    // would require equal-radius corner recognition after an unequal edit.
    for _ in 0..before.members.len() {
        let mut matches = Vec::new();
        for member in &before.members {
            let [source] = member.faces.as_slice() else {
                continue;
            };
            if map.modified.contains_key(&source.index()) {
                continue;
            }
            // Unequal radii can move a corner's contact off a root support
            // onto its neighbouring band. Its incident bands still identify
            // the same sharp corner; requiring the old direct root contact
            // would incorrectly delete that corner's handle.
            let supports = if member.kind == BlendMemberKind::CornerPatch {
                source_adjacency
                    .neighbors_of_face(*source)
                    .iter()
                    .copied()
                    .filter(|&face| before.member_of(face).is_some())
                    .collect::<Vec<_>>()
            } else {
                member.supports.clone()
            };
            if member.kind == BlendMemberKind::CornerPatch && supports.len() < 2 {
                continue;
            }
            let Some(mut roots) = supports
                .iter()
                .map(|face| {
                    let [image] = map.modified.get(&face.index())?.as_slice() else {
                        return None;
                    };
                    Some(*image)
                })
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            roots.compact_sort_unstable();
            if roots.is_empty() {
                continue;
            }
            let source_data = topo.face(*source)?;
            let mut candidates = Vec::new();
            for &target in &outputs {
                let data = topo.face(target)?;
                // Regenerated corner patches can reverse their parameterization;
                // the face's reversal flag alone is not an orientation witness.
                if data.surface().type_tag() != source_data.surface().type_tag()
                    || map
                        .modified
                        .values()
                        .any(|images| images.contains(&target.index()))
                {
                    continue;
                }
                let neighbours = adjacency.neighbors_of_face(target);
                if roots
                    .iter()
                    .all(|index| neighbours.iter().any(|f| f.index() == *index))
                {
                    candidates.push(target);
                }
            }
            if let [target] = candidates.as_slice() {
                matches.push((source.index(), target.index()));
            }
        }
        // A many-to-one match is ambiguous, even when each source found only one
        // candidate. Keep the original deleted/generated claims in that case.
        if matches.is_empty() {
            break;
        }
        let mut changed = false;
        for &(source, target) in &matches {
            if matches.iter().filter(|(_, image)| *image == target).count() != 1 {
                continue;
            }
            map.generated
                .values_mut()
                .for_each(|outputs| outputs.retain(|&out| out != target));
            map.generated.retain(|_, outputs| !outputs.is_empty());
            map.unresolved.remove(&target);
            map.deleted.remove(&source);
            map.add_modified(source, target);
            map.origin = crate::evolution::EvolutionOrigin::Geometry;
            changed = true;
        }
        if !changed {
            break;
        }
    }
    Ok(())
}

fn diameter_refusal(error: OperationsError, radius: f64) -> OperationsError {
    if matches!(
        &error,
        OperationsError::Offset(remus_offset::OffsetError::TopologyChange { .. })
            | OperationsError::BlendCluster(BlendClusterError::ReblendFailed { .. })
    ) {
        BlendClusterError::DiameterUnbuildable {
            diameter: 2.0 * radius,
            reason: error.to_string(),
        }
        .into()
    } else {
        error
    }
}

/// Re-index an evolution: inputs through `inputs`, outputs through
/// `outputs` (both sorted `(from, to)` pairs). An index without a pair is
/// dropped, and an output that loses an attributed input becomes unresolved.
fn translate(
    evolution: &EvolutionMap,
    inputs: &[(usize, usize)],
    outputs: &[(usize, usize)],
) -> EvolutionMap {
    let find = |pairs: &[(usize, usize)], key: usize| {
        pairs
            .binary_search_by_key(&key, |&(from, _)| from)
            .ok()
            .map(|at| pairs[at].1)
    };
    let mut out = if evolution.origin.is_exact() {
        EvolutionMap::exact()
    } else {
        EvolutionMap::new()
    };
    let keyed = |entries: Vec<(usize, Vec<usize>)>, generated: bool, out: &mut EvolutionMap| {
        for (input, results) in entries {
            let Some(input) = find(inputs, input) else {
                continue;
            };
            for result in results {
                let Some(result) = find(outputs, result) else {
                    continue;
                };
                if generated {
                    out.add_generated(input, result);
                } else {
                    out.add_modified(input, result);
                }
            }
        }
    };
    let sorted = |entries: Vec<(usize, Vec<usize>)>| {
        let mut entries = entries;
        entries.compact_sort_unstable_by_key(|(key, _)| *key);
        entries
    };
    keyed(
        sorted(
            evolution
                .modified
                .iter()
                .map(|(&k, v)| (k, v.clone()))
                .collect(),
        ),
        false,
        &mut out,
    );
    keyed(
        sorted(
            evolution
                .generated
                .iter()
                .map(|(&k, v)| (k, v.clone()))
                .collect(),
        ),
        true,
        &mut out,
    );
    let mut deleted: Vec<usize> = evolution
        .deleted
        .iter()
        .filter_map(|&input| find(inputs, input))
        .collect();
    deleted.compact_sort_unstable();
    for input in deleted {
        out.add_deleted(input);
    }
    for (&result, candidates) in &evolution.unresolved {
        let Some(result) = find(outputs, result) else {
            continue;
        };
        let mapped: Vec<usize> = candidates
            .iter()
            .filter_map(|&candidate| find(inputs, candidate))
            .collect();
        out.add_unresolved(result, mapped);
    }
    out
}

fn lost(reason: impl Into<String>) -> OperationsError {
    BlendClusterError::ReblendFailed {
        reason: reason.into(),
    }
    .into()
}
