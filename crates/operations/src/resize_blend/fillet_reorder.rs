//! Fillets that run onto earlier blend bands.
//!
//! A new fillet's tangent chain follows an earlier band's arc whenever the
//! selected edge runs into that band. A rolling ball can follow a convex band
//! only while its radius is no larger than the band's: above that the band's
//! offset is empty and both engines refuse. Filleting the larger edges first
//! never meets that limit, so the refusal depends only on the order the
//! fillets were made in.
//!
//! This module removes that order dependence. It restores the sharp edge
//! under every smaller band on the chain, fillets the selection at its radius
//! on the sharp body, then rebuilds the removed bands in descending radius
//! order: the result of applying the same fillets largest first. A rebuilt
//! band follows its own tangent chain, so it may continue around a corner the
//! larger fillet created, exactly as it would have with that order.
//!
//! Some corners build only in one order whatever the radii: a concave edge
//! filleted after the two convex edges mitred over its reflex corner runs into
//! that mitre, while the convex pair filleted after the concave edge rides its
//! band's end as a torus. When the smaller-band pass does not apply, every
//! band the chain runs onto is rebuilt after the selection the same way.

use remus_math::det_hash::DetHashSet;

use remus_blend::{BlendEngine, BlendResult};
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::{
    BandDescription, BlendMovePlan, blend_region_supports, blend_surface, describe_band,
    distinct_faces, face_edges, reconstruction, remap_plan, remove_blend_region,
    resolve_support_pair_edges, shared_edges, solid_adjacency, support_pairs_for_edges,
    validate_exact_result,
};
use crate::OperationsError;

/// Fillet `edges` at `radius` by rebuilding the earlier bands its tangent
/// chain runs onto after it: the smaller ones first, every one if that does
/// not apply or build.
///
/// `fillet` is the blend that runs on each sharp or rebuilt stage; it must not
/// itself call back into this function. Returns `Ok(None)`, having changed
/// nothing, when the chain meets no band or a selected edge borders one (that
/// edge would not survive the removal). The caller owns the transaction: any
/// error leaves partial stages in the arena.
///
/// The result carries no face origins: its bands were rebuilt, so faces are
/// not traced through the intermediate sharp stages.
///
/// # Errors
///
/// Returns the first refusal of a removal, a fillet stage, or the final
/// validation.
pub fn fillet_across_earlier_blends(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
    mut fillet: impl FnMut(
        &mut Topology,
        SolidId,
        &[EdgeId],
        f64,
    ) -> Result<BlendResult, OperationsError>,
) -> Result<Option<BlendResult>, OperationsError> {
    // Smaller bands first: a larger band on the chain stays where it is, so
    // nothing that pass builds rebuilds a band it did not have to.
    let smaller = remus_topology::transaction::run_transacted(topo, |topo| {
        reorder(topo, solid, edges, radius, true, &mut fillet)
    });
    if let Ok(Some(result)) = smaller {
        return Ok(Some(result));
    }
    reorder(topo, solid, edges, radius, false, &mut fillet)
}

#[allow(clippy::too_many_lines)]
fn reorder<F>(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
    smaller_only: bool,
    fillet: &mut F,
) -> Result<Option<BlendResult>, OperationsError>
where
    F: FnMut(&mut Topology, SolidId, &[EdgeId], f64) -> Result<BlendResult, OperationsError>,
{
    let tol = Tolerance::new();
    let Ok(chain) = remus_blend::g1_chain::expand_g1_chain(topo, solid, edges, tol) else {
        return Ok(None);
    };
    let adjacency = solid_adjacency(topo, solid)?;
    let is_seed_band = |face: FaceId| -> Result<bool, OperationsError> {
        Ok(blend_surface(topo.face(face)?.surface())
            .is_some_and(|(_, band)| !smaller_only || band < radius - tol.linear))
    };
    let mut seeds = Vec::new();
    for &edge in &chain {
        for &face in adjacency.faces_for_edge(edge) {
            if is_seed_band(face)? {
                seeds.push(face);
            }
        }
    }
    seeds.compact_sort_unstable_by_key(|face| face.index());
    seeds.dedup();
    if seeds.is_empty() {
        return Ok(None);
    }
    let mut selected_faces = Vec::new();
    for &edge in edges {
        for &face in adjacency.faces_for_edge(edge) {
            if is_seed_band(face)? {
                return Ok(None);
            }
            selected_faces.push(face);
        }
    }
    let mut selection = BlendMovePlan {
        radius,
        volume_effect: 0.0,
        support_pairs: support_pairs_for_edges(
            topo,
            solid,
            edges,
            &distinct_faces(&selected_faces),
        )?,
        band_faces: Vec::new(),
    };

    // Remove one band group per pass, re-recognized on the current stage, so
    // seeds on a band already taken off simply drop out of the map. Adjacent
    // bands of one radius (two stripes mitred over a reflex corner) end on
    // each other, so they come off together as one wound.
    let mut current = solid;
    let mut plans: Vec<BlendMovePlan> = Vec::new();
    while let Some(seed) = seeds.pop() {
        let band = band_group(topo, current, seed, &seeds, tol)?;
        let sharp = remove_blend_region(topo, current, &band)?;
        validate_exact_result(topo, sharp.solid, "sharp edge under an earlier blend")?;
        remap_plan(&mut selection, &sharp.face_map)?;
        for plan in &mut plans {
            remap_plan(plan, &sharp.face_map)?;
        }
        let supports = super::remap_faces(&band.supports, &sharp.face_map, "blend support")?;
        plans.push(BlendMovePlan {
            radius: band.radius,
            volume_effect: 0.0,
            support_pairs: support_pairs_for_edges(topo, sharp.solid, &sharp.edges, &supports)?,
            band_faces: Vec::new(),
        });
        seeds = seeds
            .iter()
            .filter_map(|face| sharp.face_map.get(&face.index()).copied())
            .collect();
        current = sharp.solid;
    }

    let mut engine = None;
    let mut stage = |topo: &mut Topology,
                     current: SolidId,
                     stage_edges: &[EdgeId],
                     stage_radius: f64,
                     later: &mut [BlendMovePlan]|
     -> Result<SolidId, OperationsError> {
        let result = fillet(topo, current, stage_edges, stage_radius)?;
        let origins = result.face_origins.as_ref().ok_or_else(|| {
            reconstruction("fillet stage did not report construction face origins")
        })?;
        let survivors = origins
            .survived
            .iter()
            .map(|(source, target)| (source.index(), *target))
            .collect();
        for plan in later {
            remap_plan(plan, &survivors)?;
        }
        engine = Some(match engine {
            Some(previous) if previous != result.engine => BlendEngine::Mixed,
            _ => result.engine,
        });
        Ok(result.solid)
    };

    let selected_edges = resolve_support_pair_edges(topo, current, &selection)?;
    current = stage(topo, current, &selected_edges, radius, &mut plans)?;
    // Largest first, so each rebuilt band only ever rolls across bands at
    // least its own size; one band per call, as the fillets would have been
    // made. Stable: equal radii rebuild in removal order.
    plans.compact_sort_by(|a, b| b.radius.total_cmp(&a.radius));
    for index in 0..plans.len() {
        let (done, later) = plans.split_at_mut(index + 1);
        let plan = &done[index];
        let plan_edges = rebuild_edges(topo, current, plan, tol)?;
        if !plan_edges.is_empty() {
            current = stage(topo, current, &plan_edges, plan.radius, later)?;
        }
    }
    validate_exact_result(topo, current, "fillet across smaller blends")?;

    Ok(Some(BlendResult {
        solid: current,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: None,
        engine: engine.unwrap_or(BlendEngine::RollingBall),
    }))
}

/// The band through `seed`, grown by every band of `others` with the same
/// radius it shares an edge with, directly or through another.
fn band_group(
    topo: &Topology,
    solid: SolidId,
    seed: FaceId,
    others: &[FaceId],
    tol: Tolerance,
) -> Result<BandDescription, OperationsError> {
    let mut band = describe_band(topo, solid, seed)?;
    let adjacency = solid_adjacency(topo, solid)?;
    let mut merged = false;
    loop {
        let members: DetHashSet<FaceId> = band.faces.iter().copied().collect();
        let mut touching: DetHashSet<FaceId> = DetHashSet::default();
        for &face in &band.faces {
            for edge in face_edges(topo, face)? {
                touching.extend(adjacency.faces_for_edge(edge).iter().copied());
            }
        }
        let mut grown = false;
        for &other in others {
            if members.contains(&other) || !touching.contains(&other) {
                continue;
            }
            let neighbour = describe_band(topo, solid, other)?;
            if tol.approx_eq(neighbour.radius, band.radius) {
                band.faces.extend(neighbour.faces);
                grown = true;
            }
        }
        if !grown {
            break;
        }
        band.faces.compact_sort_unstable_by_key(|face| face.index());
        band.faces.dedup();
        merged = true;
    }
    if merged {
        band.supports = blend_region_supports(topo, solid, &band.faces)?;
    }
    Ok(band)
}

/// The sharp edges a removed band is rebuilt on.
///
/// A support pair that no longer shares an edge was rounded by an earlier
/// stage whose tangent chain ran onto it: filleting largest first rounds it
/// the same way. That is accepted only when a band at least this radius now
/// joins the pair; any other change in the shared edges is a refusal.
fn rebuild_edges(
    topo: &Topology,
    solid: SolidId,
    plan: &BlendMovePlan,
    tol: Tolerance,
) -> Result<Vec<EdgeId>, OperationsError> {
    let adjacency = solid_adjacency(topo, solid)?;
    let neighbours = |face: FaceId| -> Result<Vec<FaceId>, OperationsError> {
        let mut found = Vec::new();
        for edge in face_edges(topo, face)? {
            found.extend(
                adjacency
                    .faces_for_edge(edge)
                    .iter()
                    .copied()
                    .filter(|&other| other != face),
            );
        }
        Ok(distinct_faces(&found))
    };
    let mut edges = Vec::new();
    for pair in &plan.support_pairs {
        let shared = distinct_edges(shared_edges(topo, solid, pair.first, pair.second)?);
        if shared.len() == pair.edge_count {
            edges.extend(shared);
            continue;
        }
        let second = neighbours(pair.second)?;
        let mut bridged = false;
        if shared.is_empty() {
            for face in neighbours(pair.first)? {
                if second.contains(&face)
                    && blend_surface(topo.face(face)?.surface())
                        .is_some_and(|(_, band)| band > plan.radius - tol.linear)
                {
                    bridged = true;
                    break;
                }
            }
        }
        if !bridged {
            return Err(reconstruction(format!(
                "rebuilt blend supports {} and {} share {} sharp edges; expected {}",
                pair.first.index(),
                pair.second.index(),
                shared.len(),
                pair.edge_count
            )));
        }
    }
    Ok(edges)
}

fn distinct_edges(mut edges: Vec<EdgeId>) -> Vec<EdgeId> {
    edges.compact_sort_unstable_by_key(|edge| edge.index());
    edges.dedup();
    edges
}
