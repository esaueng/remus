//! Staged re-fillet of a sharp restoration: the last stage of the
//! blend-aware direct edit (recognize the group, remove it to sharp, edit the
//! sharp body, re-fillet).
//!
//! A recognized group carries a suggested order
//! ([`BlendCluster::reblend_order`]): concave runs first, then the convex
//! edges that ride into their bands, larger radii first, then its chamfer
//! stripes, larger distances first. A chamfer stage cuts the whole tangent
//! chain through its sharp edges when the stripes were cut along one
//! (joined stripe to stripe, or round a blend's foot), and exactly its sharp
//! edges otherwise; a fillet that rolls on a removed chamfer stripe was made
//! after it, which this order cannot reproduce, and is refused. A radius edit
//! overrides some bands' radii; [`plan_reblend`] turns order and overrides
//! into candidate stage plans, and [`reblend_restoration`] runs them through
//! [`fillet_cascade`] (or [`fillet_cascade_radii`] for a mixed-radius stage)
//! until one builds.
//!
//! # Plans
//!
//! Each suggested step is split by radius (larger first). When the concave
//! stages carry several radii (a seam edited away from its shoulder), the
//! first candidate builds them as one mixed-radius stage, each run at its
//! own radius: a seam and a shoulder of different radii meet in a mitre only
//! when built together, and the convex round after them closes their corner
//! with an unequal-radius corner patch. Built one radius at a time, the
//! second band would have to end on the first, which no engine does. The
//! next candidate merges neighbouring stages of equal radius into one call,
//! which is how the group is built in the first place (an all-R4 boss group
//! is one five-edge fillet, the only order that yields its equal-radius
//! corners).
//! The unmerged split and, where one convexity carries several radii, the
//! smaller-first split follow. Each carries the chamfer stages last, larger
//! distance first; with several chamfer stages the same fillet candidates
//! follow again with the chamfers in the reverse order (two chains that meet
//! commute in exact geometry, but either may be the one whose end closes).
//! Nothing else is tried: a radius set no candidate builds is a typed
//! [`BlendClusterError::ReblendFailed`] naming every attempt.
//!
//! # Edge identity across stages
//!
//! `fillet_cascade` builds a new solid and reports no edge history, so a
//! later stage's edges are found again on the earlier stage's result by
//! their closed-form carrier: the line segment or circle each one spanned on
//! the stage-0 body (a trimmed straight edge keeps its line inside that
//! segment, a trimmed arc its circle). Every stage must find at least one
//! edge for each of its sharp edges, or the attempt is refused.
//!
//! # Postconditions
//!
//! A fillet can return a body whose B-rep closes while its faces do not mesh
//! together (a lone seam band of a different radius than the shoulder it
//! runs into does, see the W3 spike),
//! so the finished body must pass strict validation and mesh watertight, with
//! its mesh volume agreeing with the API volume, before it is accepted. Each
//! attempt runs in its own transaction, so a refused one leaves nothing
//! behind.
//!
//! WASM module tier: `core` (it edits the live session through
//! `resizeBlend` and `resizeCylindricalFace`).

use std::collections::BTreeMap;

use remus_blend::BlendEngine;
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use super::{BlendCluster, BlendClusterError, BlendMemberKind, Convexity, StageBlend};
use crate::OperationsError;
use crate::blend_ops::{
    SeedCarrier, chamfer_v2, chamfer_v2_tangent_chain, fillet_cascade, fillet_cascade_radii,
};
use crate::evolution::EvolutionMap;

/// Deflection of the watertightness postcondition, as a fraction of the
/// result's vertex bounding-box diagonal.
const MESH_DEFLECTION: f64 = 2e-4;
/// Largest accepted disagreement between the API and the independent mesh
/// volume of the result (relative).
const MESH_VOLUME_AGREEMENT: f64 = 1e-3;
/// Distance (in multiples of the linear tolerance) within which a trimmed
/// edge still lies on its stage-0 carrier.
const CARRIER_TOLERANCE: f64 = 100.0;

/// One re-blend call: the sharp edges it rounds at one radius, or
/// chamfers at one distance.
#[derive(Debug, Clone, PartialEq)]
pub struct ReblendStage {
    /// Ball radius of the call, or a chamfer's distance.
    pub radius: f64,
    /// Fillet or chamfer.
    pub blend: StageBlend,
    /// Indices into [`BlendCluster::sharp_edges`], ascending.
    pub sharp_edges: Vec<usize>,
    /// A mixed-radius fillet stage's radius for each of
    /// [`Self::sharp_edges`] (then [`Self::radius`] is the largest); empty
    /// when every edge takes [`Self::radius`].
    pub radii: Vec<f64>,
}

/// How much of the acceptance work a re-fillet does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Gate {
    /// Every guard: each stage's blend postconditions (the volume oracle and
    /// the convex material guard included) and the module's postconditions
    /// (strict validation, a watertight manifold mesh, API/mesh volume
    /// agreement). What every edit that keeps its result runs.
    Full,
    /// A live preview frame: the stages run as display-only blends
    /// ([`crate::blend_ops::set_display_only_blends`]), which keep their
    /// structural checks (every named edge rounded, a closed outer shell, no
    /// validation error the input did not have) and skip the two measuring
    /// guards; the finished body must still pass strict validation with a
    /// closed outer shell. The mesh and volume postconditions are left to the
    /// commit, which runs [`Gate::Full`].
    Preview,
}

/// A re-filleted group.
#[derive(Debug, Clone)]
pub struct GroupReblend {
    /// The re-filleted solid.
    pub solid: SolidId,
    /// The plan that built it.
    pub stages: Vec<ReblendStage>,
    /// The engine each stage ran on.
    pub engines: Vec<BlendEngine>,
    /// Face evolution from the body handed to [`reblend_restoration`] to
    /// [`Self::solid`]: construction-derived when every stage reported its
    /// face origins.
    pub evolution: EvolutionMap,
}

/// Candidate stage plans that re-fillet the `removed` members of `cluster`.
///
/// `radii` overrides the recorded radius of some bands (member index to
/// radius). The first candidate is the preferred one; see the module
/// documentation for the order.
///
/// # Errors
///
/// [`BlendClusterError::InvalidReblend`] when an override names a member
/// that is not a removed band, or a radius that is not finite and positive.
pub fn plan_reblend(
    cluster: &BlendCluster,
    removed: &[usize],
    radii: &BTreeMap<usize, f64>,
) -> Result<Vec<Vec<ReblendStage>>, OperationsError> {
    let tolerance = Tolerance::new();
    for (&member, &radius) in radii {
        let band = cluster
            .members
            .get(member)
            .is_some_and(|data| data.kind == BlendMemberKind::Band);
        if !band || !removed.contains(&member) {
            return Err(invalid(format!(
                "member {member} is not a removed band of the group"
            )));
        }
        if !radius.is_finite() || radius <= tolerance.linear {
            return Err(invalid(format!(
                "radius {radius} for member {member} is not finite and positive"
            )));
        }
    }
    // Chamfers are rebuilt after every fillet; a removed fillet that rolls on
    // a removed chamfer stripe was made after it, which that order cannot
    // reproduce.
    for &member in removed {
        let Some(data) = cluster.members.get(member) else {
            continue;
        };
        if data.kind.is_chamfer() {
            continue;
        }
        for &support in &data.supports {
            if let Some(other) = cluster.member_of(support)
                && cluster.members[other].kind.is_chamfer()
                && removed.contains(&other)
            {
                return Err(invalid(format!(
                    "fillet member {member} rolls on chamfer member {other}; chamfers are rebuilt after fillets"
                )));
            }
        }
    }
    let radius_of = |member: usize| {
        radii
            .get(&member)
            .copied()
            .unwrap_or(cluster.members[member].radius)
    };

    // Each suggested step, split by radius (larger first). Chamfer steps
    // carry one distance and stay last.
    let mut split: Vec<(Convexity, ReblendStage)> = Vec::new();
    let mut chamfers: Vec<ReblendStage> = Vec::new();
    for step in &cluster.reblend_order {
        if let StageBlend::Chamfer { .. } = step.blend {
            let mut sharp_edges: Vec<usize> = step
                .sharp_edges
                .iter()
                .copied()
                .filter(|&index| {
                    cluster
                        .sharp_edges
                        .get(index)
                        .is_some_and(|edge| removed.contains(&edge.member))
                })
                .collect();
            sharp_edges.compact_sort_unstable();
            if !sharp_edges.is_empty() {
                chamfers.push(ReblendStage {
                    radius: step.radius,
                    blend: step.blend,
                    sharp_edges,
                    radii: Vec::new(),
                });
            }
            continue;
        }
        let mut by_radius: Vec<ReblendStage> = Vec::new();
        for &index in &step.sharp_edges {
            let Some(edge) = cluster.sharp_edges.get(index) else {
                continue;
            };
            if !removed.contains(&edge.member) {
                continue;
            }
            let radius = radius_of(edge.member);
            match by_radius
                .iter_mut()
                .find(|stage| tolerance.approx_eq(stage.radius, radius))
            {
                Some(stage) => stage.sharp_edges.push(index),
                None => by_radius.push(ReblendStage {
                    radius,
                    blend: StageBlend::Fillet,
                    sharp_edges: vec![index],
                    radii: Vec::new(),
                }),
            }
        }
        by_radius.compact_sort_unstable_by(|a, b| b.radius.total_cmp(&a.radius));
        split.extend(by_radius.into_iter().map(|stage| (step.convexity, stage)));
    }
    if split.is_empty() && chamfers.is_empty() {
        return Err(invalid("no removed band has a sharp edge to re-fillet"));
    }
    // A chamfer corner is re-cut by the chain of the stripes it continues;
    // with none of them removed, nothing would rebuild it.
    for &member in removed {
        if cluster
            .members
            .get(member)
            .is_some_and(|data| data.kind == BlendMemberKind::ChamferCorner)
            && !cluster.reblend_order.iter().any(|step| {
                step.members.contains(&member)
                    && step.sharp_edges.iter().any(|&index| {
                        cluster
                            .sharp_edges
                            .get(index)
                            .is_some_and(|edge| removed.contains(&edge.member))
                    })
            })
        {
            return Err(invalid(format!(
                "chamfer corner {member} has no removed stripe to re-cut it from"
            )));
        }
    }
    for (_, stage) in &mut split {
        stage.sharp_edges.compact_sort_unstable();
    }

    // The concave runs as one call when they differ in radius, each run at
    // its own (they may be suggested as separate steps once a seam already
    // has a radius of its own): a seam and a shoulder of different radii
    // meet in a mitre only when built together, and a later convex round
    // closes their corner (see `fillet_cascade_radii`). Built one radius at
    // a time the second band would have to end on the first.
    let mut mixed: Vec<ReblendStage> = Vec::new();
    let mut any_mixed = false;
    let mut index = 0;
    while index < split.len() {
        let (convexity, first) = &split[index];
        let mut end = index + 1;
        while end < split.len()
            && split[end].0 == *convexity
            && *convexity == Convexity::Concave
            && split[end].1.blend == StageBlend::Fillet
            && first.blend == StageBlend::Fillet
        {
            end += 1;
        }
        if end - index > 1 {
            let mut pairs: Vec<(usize, f64)> = split[index..end]
                .iter()
                .flat_map(|(_, stage)| stage.sharp_edges.iter().map(|&edge| (edge, stage.radius)))
                .collect();
            pairs.compact_sort_unstable_by_key(|&(edge, _)| edge);
            mixed.push(ReblendStage {
                radius: pairs.iter().map(|&(_, r)| r).fold(0.0, f64::max),
                blend: StageBlend::Fillet,
                sharp_edges: pairs.iter().map(|&(edge, _)| edge).collect(),
                radii: pairs.iter().map(|&(_, r)| r).collect(),
            });
            any_mixed = true;
        } else {
            mixed.push(first.clone());
        }
        index = end;
    }

    let unmerged: Vec<ReblendStage> = split.iter().map(|(_, stage)| stage.clone()).collect();
    let mut merged: Vec<ReblendStage> = Vec::new();
    for stage in &unmerged {
        match merged.last_mut() {
            Some(last) if tolerance.approx_eq(last.radius, stage.radius) => {
                last.sharp_edges.extend(stage.sharp_edges.iter().copied());
                last.sharp_edges.compact_sort_unstable();
            }
            _ => merged.push(stage.clone()),
        }
    }
    // Smaller radii first within each convexity, keeping concave before
    // convex.
    let mut ascending = split.clone();
    ascending.compact_sort_by(|(a, first), (b, second)| {
        a.cmp(b).then(first.radius.total_cmp(&second.radius))
    });
    let ascending: Vec<ReblendStage> = ascending.into_iter().map(|(_, stage)| stage).collect();

    // Chamfer chains that meet (one ending on another's stripe) commute in
    // exact geometry, but an assembler may close only one of the two ends;
    // each fillet order is tried with the chamfers in their planned order
    // and then in the reverse one.
    let mut chamfer_orders = vec![chamfers.clone()];
    if chamfers.len() > 1 {
        chamfer_orders.push(chamfers.iter().rev().cloned().collect());
    }
    let mut plans = Vec::new();
    for chamfer_order in &chamfer_orders {
        let candidates: Vec<&Vec<ReblendStage>> = if any_mixed {
            vec![&mixed, &merged, &unmerged, &ascending]
        } else {
            vec![&merged, &unmerged, &ascending]
        };
        for fillets in candidates {
            let mut candidate = fillets.clone();
            candidate.extend(chamfer_order.iter().cloned());
            if !plans.contains(&candidate) {
                plans.push(candidate);
            }
        }
    }
    Ok(plans)
}

/// Re-fillet `solid` by the first of `plans` that builds.
///
/// `sharp_edges[i]` are the edges of `solid` along the cluster's sharp edge
/// `i` (a [`SharpRestoration::sharp_edges`](super::SharpRestoration::sharp_edges)
/// carried through any edit of the sharp body). Each plan runs in its own
/// transaction and must pass the module's postconditions.
///
/// # Errors
///
/// [`BlendClusterError::InvalidReblend`] for an empty plan list or a stage
/// naming a sharp edge without edges; [`BlendClusterError::ReblendFailed`]
/// when no plan builds, naming each attempt's refusal. The topology is
/// unchanged on every error.
pub fn reblend_restoration(
    topo: &mut Topology,
    solid: SolidId,
    sharp_edges: &[Vec<EdgeId>],
    plans: &[Vec<ReblendStage>],
) -> Result<GroupReblend, OperationsError> {
    reblend_restoration_gated(topo, solid, sharp_edges, plans, Gate::Full)
}

/// [`reblend_restoration`] under `gate`.
pub(super) fn reblend_restoration_gated(
    topo: &mut Topology,
    solid: SolidId,
    sharp_edges: &[Vec<EdgeId>],
    plans: &[Vec<ReblendStage>],
    gate: Gate,
) -> Result<GroupReblend, OperationsError> {
    if plans.is_empty() {
        return Err(invalid("no re-fillet plan"));
    }
    let mut carriers: Vec<Vec<SeedCarrier>> = Vec::with_capacity(sharp_edges.len());
    for (index, edges) in sharp_edges.iter().enumerate() {
        let mut found = Vec::with_capacity(edges.len());
        for &edge in edges {
            let carrier = SeedCarrier::of(topo, edge).ok_or_else(|| {
                invalid(format!(
                    "sharp edge {index} (edge {}) is not a line or a circle",
                    edge.index()
                ))
            })?;
            found.push(carrier);
        }
        carriers.push(found);
    }
    let mut refusals = Vec::with_capacity(plans.len());
    for plan in plans {
        for stage in plan {
            if stage
                .sharp_edges
                .iter()
                .any(|&index| sharp_edges.get(index).is_none_or(Vec::is_empty))
            {
                return Err(invalid(format!(
                    "a stage names sharp edges {:?} the body does not carry",
                    stage.sharp_edges
                )));
            }
        }
        let _attempt = crate::performance::span("blend_group.plan_attempt");
        match crate::blend_ops::preview_checks::transaction(topo, |topo| match gate {
            Gate::Full => run_plan(topo, solid, sharp_edges, &carriers, plan, gate),
            Gate::Preview => crate::blend_ops::display_only_scope(|| {
                run_plan(topo, solid, sharp_edges, &carriers, plan, gate)
            }),
        }) {
            Ok(result) => return Ok(result),
            Err(error) => refusals.push(format!("{}: {error}", describe(plan))),
        }
    }
    Err(BlendClusterError::ReblendFailed {
        reason: refusals.join("; "),
    }
    .into())
}

fn invalid(reason: impl Into<String>) -> OperationsError {
    BlendClusterError::InvalidReblend {
        reason: reason.into(),
    }
    .into()
}

fn describe(plan: &[ReblendStage]) -> String {
    plan.iter()
        .map(|stage| match stage.blend {
            StageBlend::Fillet if !stage.radii.is_empty() => {
                format!("R{:?} {:?}", stage.radii, stage.sharp_edges)
            }
            StageBlend::Fillet => format!("R{} {:?}", stage.radius, stage.sharp_edges),
            StageBlend::Chamfer { .. } => {
                format!("chamfer {} {:?}", stage.radius, stage.sharp_edges)
            }
        })
        .collect::<Vec<_>>()
        .join(" then ")
}

fn run_plan(
    topo: &mut Topology,
    solid: SolidId,
    sharp_edges: &[Vec<EdgeId>],
    carriers: &[Vec<SeedCarrier>],
    plan: &[ReblendStage],
    gate: Gate,
) -> Result<GroupReblend, OperationsError> {
    let tolerance = Tolerance::new().linear * CARRIER_TOLERANCE;
    let mut current = solid;
    let mut evolution: Option<EvolutionMap> = None;
    let mut engines = Vec::with_capacity(plan.len());
    for (position, stage) in plan.iter().enumerate() {
        // The stage's edges, by sharp edge.
        let per_sharp: Vec<Vec<EdgeId>> = if position == 0 {
            stage
                .sharp_edges
                .iter()
                .map(|&index| sharp_edges[index].clone())
                .collect()
        } else {
            let live = solid_edges(topo, current)?;
            let mut per_sharp = Vec::with_capacity(stage.sharp_edges.len());
            for &index in &stage.sharp_edges {
                let on: Vec<EdgeId> = live
                    .iter()
                    .copied()
                    .filter(|&edge| {
                        carriers[index]
                            .iter()
                            .any(|carrier| carrier.carries(topo, edge, tolerance))
                    })
                    .collect();
                if on.is_empty() {
                    return Err(BlendClusterError::ReblendFailed {
                        reason: format!(
                            "sharp edge {index} has no surviving edge after stage {position}"
                        ),
                    }
                    .into());
                }
                per_sharp.push(on);
            }
            per_sharp
        };
        let edges: Vec<EdgeId> = per_sharp.iter().flatten().copied().collect();
        let stage_span = crate::performance::span("blend_group.stage");
        let result = match stage.blend {
            StageBlend::Fillet if !stage.radii.is_empty() => {
                let mut sets: Vec<(Vec<EdgeId>, f64)> = Vec::new();
                for (on, &radius) in per_sharp.iter().zip(&stage.radii) {
                    match sets
                        .iter_mut()
                        .find(|(_, existing)| existing.to_bits() == radius.to_bits())
                    {
                        Some((set, _)) => set.extend(on.iter().copied()),
                        None => sets.push((on.clone(), radius)),
                    }
                }
                fillet_cascade_radii(topo, current, &sets)?
            }
            StageBlend::Fillet => fillet_cascade(topo, current, &edges, stage.radius)?,
            StageBlend::Chamfer {
                tangent_chain: true,
            } => chamfer_v2_tangent_chain(topo, current, &edges, stage.radius, stage.radius)?,
            StageBlend::Chamfer {
                tangent_chain: false,
            } => chamfer_v2(topo, current, &edges, stage.radius, stage.radius)?,
        };
        if result.is_partial || !result.failed.is_empty() {
            return Err(BlendClusterError::ReblendFailed {
                reason: format!(
                    "stage {position} at {} built only part of its edges",
                    describe(std::slice::from_ref(stage))
                ),
            }
            .into());
        }
        drop(stage_span);
        let _history = crate::performance::span("blend_group.stage_history");
        let step = match result.face_origins.as_ref() {
            Some(origins) => crate::blend_ops::evolution_from_blend_origins(
                topo,
                result.solid,
                Some(origins),
                &[],
            )?,
            None => carrier_evolution(topo, current, result.solid)?,
        };
        evolution = Some(match evolution {
            None => step,
            Some(before) => compose_evolution(&before, &step),
        });
        engines.push(result.engine);
        current = result.solid;
    }
    crate::performance::timed("blend_group.postconditions", || match gate {
        Gate::Full => postconditions(topo, current),
        Gate::Preview => {
            preview_postconditions(topo, current)?;
            crate::blend_ops::preview_checks::group(topo, current);
            Ok(())
        }
    })?;
    Ok(GroupReblend {
        solid: current,
        stages: plan.to_vec(),
        engines,
        evolution: evolution.unwrap_or_else(EvolutionMap::exact),
    })
}

/// A fillet call's face history when its engine reported none, inferred
/// from carriers rather than centroids: a fillet trims the faces it rolls on
/// but never changes their surfaces, and every face it creates borders the
/// faces it was built between.
///
/// A result face on exactly one input face's carrier (same surface, same
/// orientation) is that face, modified. A result face on no input carrier is
/// new, generated from the inputs whose images it borders. Anything else is
/// unresolved, and an input with no image is deleted. The map is reported
/// as an inference ([`EvolutionMap::new`]).
fn carrier_evolution(
    topo: &Topology,
    input: SolidId,
    result: SolidId,
) -> Result<EvolutionMap, OperationsError> {
    let inputs = solid_faces(topo, input)?;
    let outputs = solid_faces(topo, result)?;
    let mut evolution = EvolutionMap::new();
    let mut image_of: BTreeMap<usize, usize> = BTreeMap::new();
    let mut fresh = Vec::new();
    for &output in &outputs {
        let data = topo.face(output)?;
        let mut matches = Vec::new();
        for &candidate in &inputs {
            let other = topo.face(candidate)?;
            if other.is_reversed() == data.is_reversed()
                && same_carrier(other.surface(), data.surface())
            {
                matches.push(candidate.index());
            }
        }
        match matches.as_slice() {
            [single] => {
                evolution.add_modified(*single, output.index());
                image_of.insert(output.index(), *single);
            }
            [] => fresh.push(output),
            _ => evolution.add_unresolved(output.index(), matches),
        }
    }
    let adjacency = topo.build_adjacency(result)?;
    for output in fresh {
        let mut sources: Vec<usize> = adjacency
            .neighbors_of_face(output)
            .iter()
            .filter_map(|neighbour| image_of.get(&neighbour.index()).copied())
            .collect();
        sources.compact_sort_unstable();
        sources.dedup();
        if sources.is_empty() {
            evolution.add_unresolved(output.index(), Vec::new());
        }
        for source in sources {
            evolution.add_generated(source, output.index());
        }
    }
    for face in inputs {
        if !image_of.values().any(|&source| source == face.index())
            && !evolution
                .unresolved
                .values()
                .any(|candidates| candidates.contains(&face.index()))
        {
            evolution.add_deleted(face.index());
        }
    }
    Ok(evolution)
}

/// Whether two surfaces are one carrier: the same plane, cylinder, cone,
/// sphere or torus within the default tolerance, or one NURBS definition.
fn same_carrier(a: &FaceSurface, b: &FaceSurface) -> bool {
    let tolerance = Tolerance::new();
    let close = |p: f64, q: f64| (p - q).abs() <= tolerance.linear;
    let parallel = |p: Vec3, q: Vec3| p.cross(q).length() <= tolerance.angular;
    match a {
        FaceSurface::Plane { normal: n, d } => {
            if let FaceSurface::Plane { normal: m, d: e } = b {
                (n.dot(*m) > 1.0 - tolerance.angular && close(*d, *e))
                    || (n.dot(*m) < tolerance.angular - 1.0 && close(*d, -*e))
            } else {
                false
            }
        }
        FaceSurface::Cylinder(a) => {
            if let FaceSurface::Cylinder(b) = b {
                let offset = b.origin() - a.origin();
                parallel(a.axis(), b.axis())
                    && (offset - a.axis() * offset.dot(a.axis())).length() <= tolerance.linear
                    && close(a.radius(), b.radius())
            } else {
                false
            }
        }
        FaceSurface::Cone(a) => {
            if let FaceSurface::Cone(b) = b {
                (a.apex() - b.apex()).length() <= tolerance.linear
                    && a.axis().dot(b.axis()) > 1.0 - tolerance.angular
                    && (a.half_angle() - b.half_angle()).abs() <= tolerance.angular
            } else {
                false
            }
        }
        FaceSurface::Sphere(a) => {
            if let FaceSurface::Sphere(b) = b {
                (a.center() - b.center()).length() <= tolerance.linear
                    && close(a.radius(), b.radius())
            } else {
                false
            }
        }
        FaceSurface::Torus(a) => {
            if let FaceSurface::Torus(b) = b {
                (a.center() - b.center()).length() <= tolerance.linear
                    && parallel(a.z_axis(), b.z_axis())
                    && close(a.major_radius(), b.major_radius())
                    && close(a.minor_radius(), b.minor_radius())
            } else {
                false
            }
        }
        FaceSurface::Nurbs(a) => matches!(b, FaceSurface::Nurbs(b) if a == b),
    }
}

/// `first` (A to B) followed by `second` (B to C), as A to C.
///
/// A face's modified images carry on as modified (or as generated, where
/// `second` generates from them); generated faces stay generated. A face
/// whose every image `second` deleted, and that is not modified into
/// anything any more, is deleted. Unresolved outputs keep their candidates,
/// mapped back to A where `first` names their sources. The result is
/// construction-derived only when both maps are.
#[must_use]
pub fn compose_evolution(first: &EvolutionMap, second: &EvolutionMap) -> EvolutionMap {
    let mut out = if first.origin.is_exact() && second.origin.is_exact() {
        EvolutionMap::exact()
    } else {
        EvolutionMap::new()
    };
    let second_modified = sorted(second.modified.iter());
    let second_generated = sorted(second.generated.iter());
    let lookup = |entries: &[(usize, Vec<usize>)], key: usize| -> Vec<usize> {
        entries
            .binary_search_by_key(&key, |(k, _)| *k)
            .map_or_else(|_| Vec::new(), |at| entries[at].1.clone())
    };
    // B -> A, for mapping unresolved candidates back.
    let mut sources_of: BTreeMap<usize, Vec<usize>> = BTreeMap::new();

    for (source, images) in sorted(first.modified.iter()) {
        let mut alive = false;
        for image in images {
            sources_of.entry(image).or_default().push(source);
            for output in lookup(&second_modified, image) {
                out.add_modified(source, output);
                alive = true;
            }
            for output in lookup(&second_generated, image) {
                out.add_generated(source, output);
            }
            if !second.deleted.contains(&image) && !second.modified.contains_key(&image) {
                // `second` did not account for this image; it is not ours to
                // delete.
                alive = true;
            }
        }
        if !alive {
            out.add_deleted(source);
        }
    }
    for (source, images) in sorted(first.generated.iter()) {
        for image in images {
            sources_of.entry(image).or_default().push(source);
            for output in lookup(&second_modified, image) {
                out.add_generated(source, output);
            }
            for output in lookup(&second_generated, image) {
                out.add_generated(source, output);
            }
        }
    }
    let mut deleted: Vec<usize> = first.deleted.iter().copied().collect();
    deleted.compact_sort_unstable();
    for source in deleted {
        out.add_deleted(source);
    }
    for (&image, candidates) in &first.unresolved {
        for output in lookup(&second_modified, image) {
            out.add_unresolved(output, candidates.clone());
        }
    }
    for (&output, candidates) in &second.unresolved {
        let mut mapped: Vec<usize> = candidates
            .iter()
            .flat_map(|candidate| sources_of.get(candidate).cloned().unwrap_or_default())
            .collect();
        mapped.compact_sort_unstable();
        mapped.dedup();
        out.add_unresolved(output, mapped);
    }
    for outputs in out.modified.values_mut().chain(out.generated.values_mut()) {
        outputs.compact_sort_unstable();
        outputs.dedup();
    }
    out
}

/// Map entries in key order.
fn sorted<'a>(
    entries: impl Iterator<Item = (&'a usize, &'a Vec<usize>)>,
) -> Vec<(usize, Vec<usize>)> {
    let mut entries: Vec<(usize, Vec<usize>)> = entries
        .map(|(&key, values)| (key, values.clone()))
        .collect();
    entries.compact_sort_unstable_by_key(|(key, _)| *key);
    entries
}

/// A preview frame's acceptance: a closed outer shell and strict
/// validation, the B-rep half of [`postconditions`]. Every stage has already
/// passed its blend's structural checks.
fn preview_postconditions(topo: &Topology, solid: SolidId) -> Result<(), OperationsError> {
    let shell = topo.shell(topo.solid(solid)?.outer_shell())?;
    if remus_topology::validation::validate_shell_closed(shell, topo).is_err() {
        return Err(BlendClusterError::ReblendFailed {
            reason: "the re-filleted preview has an open outer shell".into(),
        }
        .into());
    }
    strict_validation(topo, solid)
}

/// Strict check-crate validation of the re-filleted body.
fn strict_validation(topo: &Topology, solid: SolidId) -> Result<(), OperationsError> {
    let report = crate::performance::timed("blend_group.post.validate", || {
        remus_check::validate::validate_solid(
            topo,
            solid,
            &remus_check::validate::ValidateOptions::default(),
        )
    })?;
    if report.is_valid() {
        return Ok(());
    }
    let detail = report
        .issues
        .iter()
        .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
        .take(3)
        .map(|issue| format!("{} ({:?})", issue.description, issue.entity))
        .collect::<Vec<_>>()
        .join("; ");
    Err(BlendClusterError::ReblendFailed {
        reason: format!(
            "the re-filleted body fails validation with {} error(s): {detail}",
            report.error_count()
        ),
    }
    .into())
}

/// Strict validation, a watertight manifold mesh and API/mesh volume
/// agreement.
fn postconditions(topo: &Topology, solid: SolidId) -> Result<(), OperationsError> {
    strict_validation(topo, solid)?;
    measuring_postconditions(topo, solid)
}

/// The remaining guards of an immutable frame that already passed validation.
pub fn measuring_postconditions(topo: &Topology, solid: SolidId) -> Result<(), OperationsError> {
    let failed =
        |reason: String| -> OperationsError { BlendClusterError::ReblendFailed { reason }.into() };
    let bounds = crate::measure::solid_bounding_box(topo, solid)?;
    let diagonal = (bounds.max - bounds.min).length();
    let deflection = diagonal * MESH_DEFLECTION;
    let mesh = crate::performance::timed("blend_group.post.mesh", || {
        crate::tessellate::tessellate_solid(topo, solid, deflection)
    })?;
    let boundary = crate::tessellate::boundary_edge_count(&mesh);
    let non_manifold = crate::tessellate::non_manifold_edge_count(&mesh);
    if boundary != 0 || non_manifold != 0 {
        return Err(failed(format!(
            "the re-filleted body meshes with {boundary} boundary and {non_manifold} non-manifold edges"
        )));
    }
    let volume = crate::performance::timed("blend_group.post.volume", || {
        crate::measure::solid_volume(topo, solid, deflection)
    })?;
    let Some(&origin) = mesh.positions.first() else {
        return Err(failed("the re-filleted body has an empty mesh".into()));
    };
    let independent: f64 = mesh
        .indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize] - origin;
            let b = mesh.positions[triangle[1] as usize] - origin;
            let c = mesh.positions[triangle[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum();
    if volume <= 0.0 || (independent - volume).abs() > MESH_VOLUME_AGREEMENT * volume {
        return Err(failed(format!(
            "the re-filleted body's volume {volume} disagrees with its mesh volume {independent}"
        )));
    }
    Ok(())
}
