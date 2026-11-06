//! Direct edits through a recognized blend group: recognize the group,
//! remove it to sharp, edit the sharp body exactly, re-fillet in planned
//! order.
//!
//! Two edits take this route when their per-band paths refuse:
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
//! faces are reported deleted and rebuilt blend faces as generated from their
//! supports, as the per-band path reports them.
//!
//! WASM module tier: `core` (reached from `resizeBlend` and
//! `resizeCylindricalFace`).

use std::collections::BTreeMap;

use remus_blend::BlendEngine;
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::journal::EntityKind;
use remus_topology::solid::SolidId;

use super::reblend::{ReblendStage, compose_evolution, plan_reblend, reblend_restoration};
use super::{BlendClusterError, BlendMemberKind, recognize_blend_cluster, remove_blend_cluster};
use crate::OperationsError;
use crate::copy::copy_solid_between_with_entity_map;
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
        )?;
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
        let rebuilt = reblend_restoration(work, edited.solid, &sharp_edges, &plans)?;
        Ok(Some(Built {
            solid: rebuilt.solid,
            evolution: compose_evolution(
                &compose_evolution(&restoration.evolution, &edited.evolution),
                &rebuilt.evolution,
            ),
            stages: rebuilt.stages,
            engines: rebuilt.engines,
        }))
    })
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
    let mut work = Topology::new();
    let copied = copy_solid_between_with_entity_map(topo, &mut work, solid)?;
    let Some(&work_face) = copied.face_map.get(&face.index()) else {
        return Ok(None);
    };
    let Some(built) = route(&mut work, copied.solid, work_face)? else {
        return Ok(None);
    };
    let output = remus_topology::transaction::run_transacted(topo, |topo| {
        copy_solid_between_with_entity_map(&work, topo, built.solid)
    })?;
    let mut inputs: Vec<(usize, usize)> = copied
        .face_map
        .iter()
        .map(|(&source, work)| (work.index(), source))
        .collect();
    inputs.compact_sort_unstable();
    let mut outputs: Vec<(usize, usize)> = output
        .face_map
        .iter()
        .map(|(&work, result)| (work, result.index()))
        .collect();
    outputs.compact_sort_unstable();
    Ok(Some(GroupEdit {
        solid: output.solid,
        evolution: translate(&built.evolution, &inputs, &outputs),
        stages: built.stages,
        engines: built.engines,
    }))
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
