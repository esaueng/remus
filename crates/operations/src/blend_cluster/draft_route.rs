//! A draft through recognized blend groups: remove the groups in the way to
//! sharp, draft the sharp body exactly, re-fillet in planned order.
//!
//! [`crate::draft::draft`] tilts planar faces by relocating their corners to
//! three-plane intersections. A fillet along an edge of a drafted face puts
//! those corners on its band, and the band's carrier (a cylinder between the
//! two untilted planes) no longer fits the tilted one, so the sharp
//! construction refuses and names the curved faces in its way. This route
//! answers when those faces are recognized blends:
//!
//! 1. **Remove only what blocks.** The sharp construction is asked again
//!    after each removal, and each refusal's curved faces seed the next
//!    group: the bands on the drafted faces first, then any band whose end
//!    arc lies on a face the draft has to re-trim (a rounded vertical edge
//!    two walls away still ends on the top face the tilted wall re-trims). A
//!    blend the draft does not reach stays as it is.
//! 2. **Draft the sharp body** by the sharp construction, unchanged.
//! 3. **Carry the sharp edges through the draft.** The draft relocates
//!    corners and rebuilds the edges between them, so a restored sharp edge
//!    is found again as the edge between its two drafted corners (an edge
//!    the draft did not touch is the copy with its own ends and midpoint).
//! 4. **Re-fillet** every removed group at its recorded radii. One group
//!    runs the planner's candidates as they are; several groups are woven
//!    into one plan in the planner's order (concave before convex, larger
//!    radii first, chamfers last), with equal stages of different groups
//!    joined into one call (the four rounded vertical edges of a box are two
//!    groups, and were built as one four-edge fillet), then left apart.
//!
//! The whole route runs on a private copy of the solid; only the finished
//! result, which passed the re-fillet's postconditions (strict validation, a
//! watertight manifold mesh, API/mesh volume agreement), is copied into the
//! caller's arena. A curved face in the way that is not a recognized blend
//! is not the route's to answer: it returns `None` and the draft keeps its
//! own refusal. Everything after the first removal is a typed
//! [`OperationsError::Unsupported`] naming the step that refused.
//!
//! WASM module tier: `core` (reached from `draft` and `draftJournaled`).

use std::collections::BTreeMap;

use remus_blend::BlendError;
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::journal::EntityKind;
use remus_topology::solid::SolidId;

use super::edit::{carry_member_identities, copy_in, reindexed, translate};
use super::reblend::{
    Gate, ReblendRefusal, ReblendStage, compose_evolution, plan_reblend,
    reblend_restoration_attempts,
};
use super::{
    BlendCluster, BlendClusterError, Convexity, StageBlend, recognize_blend_cluster,
    remove_blend_cluster,
};
use crate::OperationsError;
use crate::copy::copy_solid_between_with_entity_map;
use crate::draft::{Taper, draft_sharp};
use crate::evolution::EvolutionMap;

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: "draft",
        reason: reason.into(),
    }
}

/// A refusal's own reason, without the operation prefix a nested
/// [`OperationsError::Unsupported`] would repeat.
fn reason_of(error: &OperationsError) -> String {
    match error {
        OperationsError::Unsupported { reason, .. } => reason.clone(),
        other => other.to_string(),
    }
}

/// A blend group the route removed.
struct RemovedGroup {
    /// The group, on the body it was removed from.
    cluster: BlendCluster,
    /// The group with its faces named on the route's copy of the input, for
    /// carrying member identities; `None` when a face has no image there.
    on_copy: Option<BlendCluster>,
    /// The removed member indices.
    removed: Vec<usize>,
    /// The edges along each of the group's sharp edges, on the route's
    /// current body.
    sharp_edges: Vec<Vec<EdgeId>>,
}

/// What the route built on its private arena.
struct Built {
    solid: SolidId,
    /// Face evolution from the copy of the input to [`Self::solid`].
    evolution: EvolutionMap,
}

/// Draft `draft_faces` of `solid` through the blend groups in the way.
///
/// Returns `Ok(None)` when the curved faces the sharp construction refuses
/// on are not recognized blends (the caller keeps its own refusal).
///
/// # Errors
///
/// [`OperationsError::Unsupported`] when a group in the way does not remove
/// to its sharp edges, the sharp body does not draft, a restored sharp edge
/// does not survive the draft, or the groups do not re-fillet on the drafted
/// body. The topology is unchanged on every error.
pub fn draft_through_blend_groups(
    topo: &mut Topology,
    solid: SolidId,
    draft_faces: &[FaceId],
    taper: Taper,
) -> Result<Option<(SolidId, EvolutionMap)>, OperationsError> {
    let _span = crate::performance::span("draft.blend_groups");
    let (mut work, copied, inputs) = copy_in(topo, solid)?;
    let mut drafted = Vec::with_capacity(draft_faces.len());
    for face in draft_faces {
        let Some(&image) = copied.face_map.get(&face.index()) else {
            return Ok(None);
        };
        drafted.push(image);
    }
    let Some(built) = build(&mut work, copied.solid, drafted, taper)? else {
        return Ok(None);
    };
    remus_topology::transaction::run_transacted(topo, |topo| {
        let output = copy_solid_between_with_entity_map(&work, topo, built.solid)?;
        let mut outputs: Vec<(usize, usize)> = output
            .face_map
            .iter()
            .map(|(&work, result)| (work, result.index()))
            .collect();
        outputs.compact_sort_unstable();
        Ok::<_, OperationsError>(Some((
            output.solid,
            translate(&built.evolution, &inputs, &outputs),
        )))
    })
}

/// The sharp body the route has reached: the copy of the input with the
/// groups removed so far.
struct SharpBody {
    solid: SolidId,
    /// The faces to draft, on [`Self::solid`].
    drafted: Vec<FaceId>,
    /// The groups removed so far, in removal order.
    groups: Vec<RemovedGroup>,
    /// Face evolution from the copy of the input to [`Self::solid`]; `None`
    /// until a group is removed.
    evolution: Option<EvolutionMap>,
    /// Each face of [`Self::solid`], by index, as the face of the copy it
    /// derives from.
    on_copy: BTreeMap<usize, FaceId>,
}

impl SharpBody {
    /// Remove `cluster`, recognized on [`Self::solid`], and carry the faces
    /// to draft and the sharp edges restored so far onto the result.
    fn remove(
        &mut self,
        work: &mut Topology,
        cluster: BlendCluster,
    ) -> Result<(), OperationsError> {
        let restoration = match remove_blend_cluster(work, self.solid, &cluster) {
            Ok(restoration) => restoration,
            Err(OperationsError::BlendCluster(error)) => {
                return Err(unsupported(format!(
                    "a blend on a face the draft re-trims has to be removed and \
                     re-applied, and it does not remove to its sharp edges: {error}"
                )));
            }
            Err(error) => return Err(error),
        };

        let image: BTreeMap<usize, FaceId> = restoration
            .faces
            .iter()
            .map(|&(source, result)| (source.index(), result))
            .collect();
        for face in &mut self.drafted {
            *face = *image.get(&face.index()).ok_or_else(|| {
                unsupported(
                    "a drafted face is itself a chamfer of the blend group the draft \
                     has to remove",
                )
            })?;
        }
        // The sharp edges earlier removals restored, through this one.
        let edge_image: BTreeMap<usize, usize> = restoration
            .boundary_history
            .iter()
            .filter_map(|(source, target)| {
                let target = (*target)?;
                (source.kind == EntityKind::Edge && target.kind == EntityKind::Edge)
                    .then_some((source.index, target.index))
            })
            .collect();
        for edge in self
            .groups
            .iter_mut()
            .flat_map(|group| group.sharp_edges.iter_mut().flatten())
        {
            *edge = edge_image
                .get(&edge.index())
                .and_then(|&index| work.edge_id_from_index(index))
                .ok_or_else(|| {
                    unsupported(
                        "removing a second blend group consumed a sharp edge the first \
                         one restored; the groups cannot be re-applied apart",
                    )
                })?;
        }
        let named_on_copy = reindexed(&cluster, |face| self.on_copy.get(&face.index()).copied());
        self.on_copy = restoration
            .faces
            .iter()
            .filter_map(|&(source, result)| {
                self.on_copy
                    .get(&source.index())
                    .map(|&face| (result.index(), face))
            })
            .collect();
        self.evolution = Some(match &self.evolution {
            None => restoration.evolution.clone(),
            Some(before) => compose_evolution(before, &restoration.evolution),
        });
        self.solid = restoration.solid;
        self.groups.push(RemovedGroup {
            cluster,
            on_copy: named_on_copy,
            removed: restoration.removed,
            sharp_edges: restoration.sharp_edges,
        });
        Ok(())
    }
}

/// The route on its private arena: `copy` is the copy of the input and
/// `drafted` its faces to draft.
fn build(
    work: &mut Topology,
    copy: SolidId,
    drafted: Vec<FaceId>,
    taper: Taper,
) -> Result<Option<Built>, OperationsError> {
    let mut body = SharpBody {
        solid: copy,
        drafted,
        groups: Vec::new(),
        evolution: None,
        on_copy: solid_faces(work, copy)?
            .into_iter()
            .map(|face| (face.index(), face))
            .collect(),
    };
    let sharp = loop {
        let refusal = match draft_sharp(work, body.solid, &body.drafted, taper) {
            Ok(sharp) => break sharp,
            Err(refusal) => refusal,
        };
        let Some(cluster) = blocking_group(work, body.solid, &refusal.curved) else {
            if body.groups.is_empty() {
                return Ok(None);
            }
            return Err(unsupported(format!(
                "with the blends in the way removed to their sharp edges the body \
                 still does not draft (faces and corners are those of the sharp \
                 body): {}",
                reason_of(&refusal.error)
            )));
        };
        body.remove(work, cluster)?;
    };
    let Some(evolution) = body.evolution else {
        // The sharp construction drafts the body as it stands; the caller's
        // own attempt has already said so or refused for another reason.
        return Ok(None);
    };

    let live = solid_edges(work, sharp.solid)?;
    let mut sharp_edges: Vec<Vec<EdgeId>> = Vec::new();
    for edges in body.groups.iter().flat_map(|group| &group.sharp_edges) {
        let mut carried = Vec::with_capacity(edges.len());
        for &edge in edges {
            carried.push(drafted_edge(work, edge, &sharp.moved, &live)?);
        }
        sharp_edges.push(carried);
    }
    let plans = combined_plans(&body.groups).map_err(|error| reblend_refusal(error.into()))?;
    let evolution = compose_evolution(&evolution, &sharp.evolution);
    let rebuilt = reblend_restoration_attempts(work, sharp.solid, &sharp_edges, &plans, Gate::Full)
        .map_err(reblend_refusal)?;
    let mut evolution = compose_evolution(&evolution, &rebuilt.evolution);
    for before in body
        .groups
        .iter()
        .filter_map(|group| group.on_copy.as_ref())
    {
        carry_member_identities(work, before, copy, rebuilt.solid, &mut evolution)?;
    }
    Ok(Some(Built {
        solid: rebuilt.solid,
        evolution,
    }))
}

/// The recognized blend group one of `curved` belongs to, if any: the group
/// whose removal clears that face out of the draft's way. A curved face that
/// only carries blends (a boss wall) or is no blend at all names none.
fn blocking_group(topo: &Topology, solid: SolidId, curved: &[FaceId]) -> Option<BlendCluster> {
    let mut seeds = curved.to_vec();
    seeds.compact_sort_unstable_by_key(|face| face.index());
    seeds.dedup();
    seeds.into_iter().find_map(|seed| {
        recognize_blend_cluster(topo, solid, seed)
            .ok()
            .filter(|cluster| cluster.seed_is_member)
    })
}

/// The point halfway along `edge`'s parameter range.
fn midpoint(edge: &Edge, start: Point3, end: Point3) -> Point3 {
    let (t0, t1) = edge.domain_with_endpoints(start, end);
    edge.curve()
        .evaluate_with_endpoints(0.5 * (t0 + t1), start, end)
}

/// The edge of the drafted body (`live` are its edges) that `edge` of the
/// sharp body became.
///
/// The sharp construction rebuilds every edge at a relocated corner as the
/// straight edge between the drafted corners, and copies every other edge
/// with its curve, so the image is the one edge with the drafted ends and
/// the drafted midpoint.
fn drafted_edge(
    topo: &Topology,
    edge: EdgeId,
    moved: &BTreeMap<usize, Point3>,
    live: &[EdgeId],
) -> Result<EdgeId, OperationsError> {
    let tolerance = Tolerance::new().linear;
    let close = |a: Point3, b: Point3| (a - b).length() <= tolerance;
    let data = topo.edge(edge)?;
    let was = [
        topo.vertex(data.start())?.point(),
        topo.vertex(data.end())?.point(),
    ];
    let ends = [
        moved.get(&data.start().index()).copied().unwrap_or(was[0]),
        moved.get(&data.end().index()).copied().unwrap_or(was[1]),
    ];
    let relocated =
        moved.contains_key(&data.start().index()) || moved.contains_key(&data.end().index());
    let middle = if relocated {
        let span: Vec3 = ends[1] - ends[0];
        ends[0] + span * 0.5
    } else {
        midpoint(data, was[0], was[1])
    };

    let mut found = None;
    for &candidate in live {
        let other = topo.edge(candidate)?;
        let (start, end) = (
            topo.vertex(other.start())?.point(),
            topo.vertex(other.end())?.point(),
        );
        let same_ends = (close(start, ends[0]) && close(end, ends[1]))
            || (close(start, ends[1]) && close(end, ends[0]));
        if !same_ends || !close(midpoint(other, start, end), middle) {
            continue;
        }
        if found.replace(candidate).is_some() {
            return Err(unsupported(
                "a sharp edge restored for the draft has more than one image on the \
                 drafted body",
            ));
        }
    }
    found.ok_or_else(|| {
        unsupported(
            "a sharp edge a blend was removed from does not survive the draft, so \
             the blend has nothing to be re-applied to",
        )
    })
}

/// Where a stage sorts in the planner's order: fillets before chamfers,
/// concave before convex, larger radii first.
#[derive(Clone, Copy)]
struct StageKey {
    chamfer: bool,
    convexity: Convexity,
    radius: f64,
}

impl StageKey {
    fn of(cluster: &BlendCluster, stage: &ReblendStage) -> Self {
        let convexity = stage
            .sharp_edges
            .first()
            .and_then(|&index| cluster.sharp_edges.get(index))
            .and_then(|edge| cluster.members.get(edge.member))
            .map_or(Convexity::Convex, |member| member.convexity);
        Self {
            chamfer: matches!(stage.blend, StageBlend::Chamfer { .. }),
            convexity,
            radius: stage.radius,
        }
    }

    fn order(&self, other: &Self) -> std::cmp::Ordering {
        (self.chamfer, self.convexity)
            .cmp(&(other.chamfer, other.convexity))
            .then(other.radius.total_cmp(&self.radius))
    }

    /// Whether two stages are one fillet call: the same side at one radius.
    fn joins(&self, other: &Self) -> bool {
        !self.chamfer
            && !other.chamfer
            && self.convexity == other.convexity
            && Tolerance::new().approx_eq(self.radius, other.radius)
    }
}

/// A stage with its place in the planner's order.
type KeyedStage = (StageKey, ReblendStage);

/// Candidate plans that re-fillet every group of `groups`, their stages
/// indexing the groups' sharp edges laid end to end in group order.
///
/// One group keeps the planner's candidates. Several are woven candidate by
/// candidate ([`weave`]): first with equal fillet stages of neighbouring
/// groups joined into one call, then with every stage left its own call.
fn combined_plans(groups: &[RemovedGroup]) -> Result<Vec<Vec<ReblendStage>>, OperationsError> {
    let mut per_group: Vec<Vec<Vec<KeyedStage>>> = Vec::with_capacity(groups.len());
    let mut offset = 0;
    for group in groups {
        let plans = plan_reblend(&group.cluster, &group.removed, &BTreeMap::new())?;
        per_group.push(
            plans
                .into_iter()
                .map(|plan| {
                    plan.into_iter()
                        .map(|mut stage| {
                            let key = StageKey::of(&group.cluster, &stage);
                            for index in &mut stage.sharp_edges {
                                *index += offset;
                            }
                            (key, stage)
                        })
                        .collect()
                })
                .collect(),
        );
        offset += group.cluster.sharp_edges.len();
    }
    if let [only] = per_group.as_slice() {
        return Ok(only
            .iter()
            .map(|plan| plan.iter().map(|(_, stage)| stage.clone()).collect())
            .collect());
    }

    let depth = per_group.iter().map(Vec::len).max().unwrap_or(0);
    let mut plans: Vec<Vec<ReblendStage>> = Vec::new();
    for join in [true, false] {
        for candidate in 0..depth {
            let chosen: Vec<&[KeyedStage]> = per_group
                .iter()
                .filter_map(|plans| plans.get(candidate).or_else(|| plans.last()))
                .map(Vec::as_slice)
                .collect();
            let plan = weave(&chosen, join);
            if !plans.contains(&plan) {
                plans.push(plan);
            }
        }
    }
    Ok(plans)
}

/// One plan from one plan of each group: each group's stages in their own
/// order, the next stage taken from the group whose stage sorts first in the
/// planner's order (the earlier group on a tie). With `join`, a fillet stage
/// that sorts with the one before it joins that call.
fn weave(chosen: &[&[KeyedStage]], join: bool) -> Vec<ReblendStage> {
    let mut next = vec![0_usize; chosen.len()];
    let mut woven: Vec<KeyedStage> = Vec::new();
    loop {
        let mut first: Option<(usize, StageKey)> = None;
        for (group, plan) in chosen.iter().enumerate() {
            if let Some((key, _)) = plan.get(next[group])
                && first.is_none_or(|(_, best)| key.order(&best).is_lt())
            {
                first = Some((group, *key));
            }
        }
        let Some((group, key)) = first else {
            break;
        };
        let stage = chosen[group][next[group]].1.clone();
        next[group] += 1;
        match woven.last_mut() {
            Some((last_key, last))
                if join
                    && key.joins(last_key)
                    && stage.radii.is_empty()
                    && last.radii.is_empty() =>
            {
                last.sharp_edges.extend(stage.sharp_edges);
                last.sharp_edges.compact_sort_unstable();
            }
            _ => woven.push((key, stage)),
        }
    }
    woven.into_iter().map(|(_, stage)| stage).collect()
}

/// The draft's refusal for groups that do not re-fillet on the drafted body.
///
/// When every planned order stopped at the same engine limit, the vertex
/// blend where two re-filleted edges meet, the refusal names it: a taper
/// turns the square corners of a rounded loop (a lid's top edges) into
/// corners the fillet engines do not blend, whatever the order.
fn reblend_refusal(refusal: ReblendRefusal) -> OperationsError {
    let reason = match &refusal.error {
        OperationsError::BlendCluster(BlendClusterError::ReblendFailed { reason }) => {
            reason.clone()
        }
        other => other.to_string(),
    };
    let vertex_blend = !refusal.attempts.is_empty()
        && refusal.attempts.iter().all(|error| {
            matches!(
                error,
                OperationsError::Blend(BlendError::UnsupportedVertexBlend { .. })
            )
        });
    if vertex_blend {
        unsupported(format!(
            "the sharp body drafts, but its blends cannot be re-applied to it: the \
             taper leaves the re-filleted edges meeting at corners that are no \
             longer square, and the fillet engines do not blend the vertex where \
             stripes meet there (unsupported vertex blend; {reason})"
        ))
    } else {
        unsupported(format!(
            "the sharp body drafts, but the blends removed for it cannot be \
             re-applied at their recorded radii: {reason}"
        ))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_topology::vertex::Vertex;

    use super::*;

    fn reason(error: OperationsError) -> String {
        match error {
            OperationsError::Unsupported {
                operation: "draft",
                reason,
            } => reason,
            other => panic!("expected a typed draft refusal, got {other:?}"),
        }
    }

    fn failed(attempts: Vec<OperationsError>) -> ReblendRefusal {
        ReblendRefusal {
            error: BlendClusterError::ReblendFailed {
                reason: "R2 [0, 1, 2, 3]: blend: unsupported vertex blend".into(),
            }
            .into(),
            attempts,
        }
    }

    /// A re-fillet every planned order of which stops at a vertex blend is
    /// refused by that name; any other mix keeps the general refusal, and
    /// both carry the attempts' own text.
    #[test]
    fn a_refillet_stopped_at_vertex_blends_is_named() {
        let mut topo = Topology::new();
        let vertex = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let vertex_blend =
            || OperationsError::Blend(BlendError::UnsupportedVertexBlend { vertex, stripes: 2 });

        let named = reason(reblend_refusal(failed(vec![
            vertex_blend(),
            vertex_blend(),
        ])));
        assert!(named.contains("unsupported vertex blend") && named.contains("R2 [0, 1, 2, 3]"));
        assert!(named.contains("no longer square"), "{named}");

        let mixed = reason(reblend_refusal(failed(vec![
            vertex_blend(),
            OperationsError::InvalidInput {
                reason: "other".into(),
            },
        ])));
        assert!(!mixed.contains("no longer square"), "{mixed}");
        assert!(mixed.contains("cannot be re-applied at their recorded radii"));

        // Refused before any plan ran: nothing to name.
        let unplanned = reason(reblend_refusal(failed(Vec::new())));
        assert!(!unplanned.contains("no longer square"), "{unplanned}");
    }

    fn stage(radius: f64, sharp_edges: &[usize]) -> KeyedStage {
        (
            StageKey {
                chamfer: false,
                convexity: Convexity::Convex,
                radius,
            },
            ReblendStage {
                radius,
                blend: StageBlend::Fillet,
                sharp_edges: sharp_edges.to_vec(),
                radii: Vec::new(),
            },
        )
    }

    /// Two groups' equal stages join into one call (the four rounded
    /// vertical edges of a box are two groups of two), a larger radius sorts
    /// first whichever group carries it, and without joining every stage
    /// stays its own call in the same order.
    #[test]
    fn stages_of_several_groups_are_woven_in_planner_order() {
        let first = [stage(5.0, &[0, 1])];
        let second = [stage(8.0, &[2]), stage(5.0, &[3])];
        let chosen = [first.as_slice(), second.as_slice()];
        let read = |plan: &[ReblendStage]| -> Vec<(f64, Vec<usize>)> {
            plan.iter()
                .map(|stage| (stage.radius, stage.sharp_edges.clone()))
                .collect()
        };

        let joined = weave(&chosen, true);
        assert_eq!(read(&joined), vec![(8.0, vec![2]), (5.0, vec![0, 1, 3])]);

        let apart = weave(&chosen, false);
        assert_eq!(
            read(&apart),
            vec![(8.0, vec![2]), (5.0, vec![0, 1]), (5.0, vec![3])]
        );
    }
}
