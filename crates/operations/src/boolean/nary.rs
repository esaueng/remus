//! N-ary planar General Fuse returning regions, total lineage and a
//! per-operand disposition (P-Class 4.8, first slice).
//!
//! [`fuse_regions`] fuses a list of solids in one arrangement per touching
//! cluster and reports what the list-based [`fuse_solids`] cannot: which
//! result face came from which operand, which result region each operand
//! ended up in, and which operands left no face behind because the others
//! cover them. It is exact-only and additive: [`fuse_solids`] and `fuseAll`
//! are unchanged and do not route through it.
//!
//! The qualified domain is deliberately narrow. Operands that may touch must
//! be all-planar with straight edges; anything the exact arrangement cannot
//! decide — a curved cluster, an unplaced face, an unrecorded edge, an
//! edge-only or vertex-only contact — is a typed refusal that leaves the
//! topology as it found it.
//!
//! [`fuse_solids`]: crate::compound_ops::fuse_solids

use std::collections::{BTreeMap, BTreeSet};

use remus_algo::error::{AlgoError, NaryFuseRefusal};
use remus_algo::gfa::{BooleanRegion, EdgeEvent, EntityEvolution};
use remus_math::aabb::Aabb3;
use remus_math::compact_sort::CompactSort;
use remus_math::vec::Point3;
use remus_topology::compound::{Compound, CompoundId};
use remus_topology::solid::SolidId;
use remus_topology::{BodyClass, BodyId, Topology};

use crate::OperationsError;
use crate::classify::PointClassification;

/// Name this operation reports in typed body-class refusals.
const OPERATION: &str = "fuse_regions";

/// What became of one operand of an N-ary fuse.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OperandDisposition {
    /// The operand may touch no other operand, so it is its own region: the
    /// region's solid IS the operand's handle and its lineage is the
    /// identity.
    Preserved {
        /// Index of that region in [`NaryFuseResult::regions`].
        region: usize,
    },
    /// At least one result face descends from this operand.
    Contributes {
        /// Ascending indices of the regions carrying its faces.
        regions: Vec<usize>,
    },
    /// No result face descends from this operand, and that is proven rather
    /// than assumed: the classifier placed every one of its faces strictly
    /// inside another operand or on an interior coincident contact, and every
    /// one of its vertices lies in a result region.
    ///
    /// The claim is about the operand's boundary. An operand that fills
    /// another's cavity is reported here too: it leaves no face, although the
    /// union gained its material.
    Absorbed {
        /// Ascending indices of the regions holding its vertices.
        regions: Vec<usize>,
    },
}

/// One result region of an N-ary fuse with its construction history.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NaryFuseRegion {
    /// The region's solid: a new solid, or the operand itself for a
    /// [`OperandDisposition::Preserved`] operand.
    pub solid: SolidId,
    /// Face, edge and vertex history in caller indices. Total: every result
    /// face names its source face and no edge is unresolved.
    pub evolution: EntityEvolution,
    /// The operand index owning each entry of `evolution.faces`, in the same
    /// order: result face `evolution.faces[i].0` descends from operand
    /// `face_operands[i]`.
    pub face_operands: Vec<usize>,
    /// Ascending indices of the operands with a face in this region.
    pub operands: Vec<usize>,
}

/// The result of [`fuse_regions`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NaryFuseResult {
    /// Compound holding one solid per region, in `regions` order.
    pub compound: CompoundId,
    /// The regions of the union, ordered by the lowest operand index
    /// contributing a face to each.
    pub regions: Vec<NaryFuseRegion>,
    /// One disposition per operand, in input order.
    pub operands: Vec<OperandDisposition>,
}

/// Fuse `solids` into the regions of their union, with total face lineage
/// and a disposition for every operand.
///
/// Operands are grouped by conservative bounding-box contact. An operand
/// that can touch no other is returned as its own region untouched
/// ([`OperandDisposition::Preserved`]), whatever its geometry. Each larger
/// group is fused in one exact arrangement and may yield one region or
/// several; every one of its operands must be all-planar with straight edges.
///
/// Region order is input order: regions are sorted by the lowest operand
/// index that contributes a face.
///
/// # Errors
///
/// Every error leaves the live model untouched. The typed refusals and the
/// input errors below are raised before anything is written to `topo`, so
/// they leave its arena slot counts unchanged as well: the union is built
/// and qualified in an isolated store and exported only once accepted.
///
/// - [`OperationsError::InvalidInput`] for a dead handle, an operand that is
///   not a closed manifold, or operands sharing a face.
/// - `OperationsError::Algo(AlgoError::UnsupportedNaryFuse { cause, .. })`
///   for the typed refusals, by [`NaryFuseRefusal`] cause: fewer than two
///   operands; the same handle twice; a curved operand in a touching group;
///   more touching pairs than one arrangement accepts; a face the classifier
///   cannot place; a result face or edge with no construction record; an
///   edge-only or vertex-only contact; and a result the exact engine cannot
///   stand behind — an arrangement that fails to split, classify or
///   assemble, a region that fails validation, a result that does not
///   account for an operand, a dropped section chain or uncertified chart.
/// - A resource-limit or cancellation error of the exact engine.
pub fn fuse_regions(
    topo: &mut Topology,
    solids: &[SolidId],
) -> Result<NaryFuseResult, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| fuse_regions_impl(topo, solids))
}

/// [`fuse_regions`] over body roots of any dimensional class.
///
/// A general fuse of solids with sheets and wires needs the mixed-dimensional
/// body model (RFC 0005), which does not exist yet. Until it does, a list
/// holding a sheet or a wire is refused by class, naming the first such
/// operand; an all-solid list is exactly [`fuse_regions`].
///
/// # Errors
///
/// [`OperationsError::BodyClassOperationUnsupported`] for a sheet, wire or
/// general body in the list, [`OperationsError::InvalidInput`] for a shell
/// that bounds a solid, a topology error for a dead handle, and otherwise
/// the errors of [`fuse_regions`]. Every error rolls back.
pub fn fuse_body_regions(
    topo: &mut Topology,
    bodies: &[BodyId],
) -> Result<NaryFuseResult, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let mut solids = Vec::with_capacity(bodies.len());
        let mut refused = None;
        for &body in bodies {
            // Resolve every handle before refusing, so a dead handle is
            // reported as such whatever its position in the list.
            let class = topo.body_class_of(body)?;
            match (body, class) {
                (BodyId::Solid(solid), BodyClass::Solid) => solids.push(solid),
                // A shell tagged `solid` bounds a solid; it is not a body.
                (BodyId::Shell(_), BodyClass::Solid) => {
                    return Err(OperationsError::InvalidInput {
                        reason: "fuse_regions operand is a solid-owned shell, not a body root"
                            .into(),
                    });
                }
                (_, class) => {
                    refused.get_or_insert(class);
                }
            }
        }
        if let Some(class) = refused {
            return Err(OperationsError::BodyClassOperationUnsupported {
                operation: OPERATION,
                actual: class.as_str(),
            });
        }
        fuse_regions_impl(topo, &solids)
    })
}

fn refuse(cause: NaryFuseRefusal, reason: impl Into<String>) -> OperationsError {
    OperationsError::Algo(AlgoError::UnsupportedNaryFuse {
        cause,
        reason: reason.into(),
    })
}

/// A region before the final input-order sort: `operands` are global operand
/// indices, and the dispositions that mention it use its position in the
/// unsorted list.
struct PendingRegion {
    solid: SolidId,
    evolution: EntityEvolution,
    face_operands: Vec<usize>,
    operands: Vec<usize>,
}

/// An operand's disposition against unsorted region positions.
enum PendingDisposition {
    Preserved(usize),
    Contributes(Vec<usize>),
    Absorbed(Vec<usize>),
}

fn fuse_regions_impl(
    topo: &mut Topology,
    solids: &[SolidId],
) -> Result<NaryFuseResult, OperationsError> {
    if solids.len() < 2 {
        return Err(refuse(
            NaryFuseRefusal::TooFewOperands,
            format!("needs at least two operands, got {}", solids.len()),
        ));
    }
    let mut seen: BTreeMap<usize, usize> = BTreeMap::new();
    for (index, &solid) in solids.iter().enumerate() {
        topo.solid(solid)?;
        if let Some(&first) = seen.get(&solid.index()) {
            return Err(refuse(
                NaryFuseRefusal::DuplicateOperand,
                format!("operands {first} and {index} are the same solid"),
            ));
        }
        seen.insert(solid.index(), index);
    }

    let context = super::exact_only_context();
    let tolerance = context.tolerance;

    // Which operand owns each caller face. Lineage is reported per operand,
    // so a face reachable from two operands has no answer.
    let mut face_owner: BTreeMap<usize, usize> = BTreeMap::new();
    for (index, &solid) in solids.iter().enumerate() {
        if !super::is_closed_manifold(topo, solid)? {
            return Err(OperationsError::InvalidInput {
                reason: format!("fuse_regions operand {index} is not a closed manifold solid"),
            });
        }
        for face in remus_topology::explorer::solid_faces(topo, solid)? {
            if face_owner.insert(face.index(), index).is_some() {
                return Err(OperationsError::InvalidInput {
                    reason: format!("fuse_regions operand {index} shares a face with another"),
                });
            }
        }
    }

    // Operands whose conservative boxes stay apart cannot interact; the rest
    // form the touching groups, each fused in one arrangement.
    let boxes = super::cluster_contact_boxes(topo, solids, tolerance.linear).ok_or_else(|| {
        OperationsError::InvalidInput {
            reason: "fuse_regions operand bounds are unavailable or not finite".into(),
        }
    })?;
    let mut pairs = Vec::new();
    for i in 0..boxes.len() {
        for j in (i + 1)..boxes.len() {
            if boxes[i].intersects(boxes[j]) {
                if pairs.len() == remus_algo::gfa::MAX_NWAY_SOURCE_PAIRS {
                    return Err(refuse(
                        NaryFuseRefusal::PairBudgetExceeded,
                        format!(
                            "more than {} operand pairs may touch",
                            remus_algo::gfa::MAX_NWAY_SOURCE_PAIRS
                        ),
                    ));
                }
                pairs.push((i, j));
            }
        }
    }
    let groups = touching_groups(solids.len(), &pairs);

    let mut regions: Vec<PendingRegion> = Vec::new();
    let mut dispositions: Vec<Option<PendingDisposition>> =
        (0..solids.len()).map(|_| None).collect();
    for group in &groups {
        if let [only] = group.as_slice() {
            let identity = super::identity_boolean_region(topo, solids[*only])?;
            let face_operands = vec![*only; identity.evolution.faces.len()];
            dispositions[*only] = Some(PendingDisposition::Preserved(regions.len()));
            regions.push(PendingRegion {
                solid: identity.solid,
                evolution: identity.evolution,
                face_operands,
                operands: vec![*only],
            });
            continue;
        }
        fuse_group(
            topo,
            solids,
            group,
            &pairs,
            &boxes,
            &face_owner,
            &context,
            &mut regions,
            &mut dispositions,
        )?;
    }

    // Input order: by the lowest contributing operand, ties in arrangement
    // order.
    let mut order: Vec<usize> = (0..regions.len()).collect();
    order.compact_sort_by_key(|&position| {
        (
            regions[position]
                .operands
                .first()
                .copied()
                .unwrap_or(usize::MAX),
            position,
        )
    });
    let mut final_index = vec![0usize; regions.len()];
    for (sorted, &position) in order.iter().enumerate() {
        final_index[position] = sorted;
    }
    let remap = |positions: Vec<usize>| -> Vec<usize> {
        let mut mapped: Vec<usize> = positions.into_iter().map(|p| final_index[p]).collect();
        mapped.compact_sort_unstable();
        mapped.dedup();
        mapped
    };
    let operands = dispositions
        .into_iter()
        .enumerate()
        .map(|(index, disposition)| match disposition {
            Some(PendingDisposition::Preserved(position)) => Ok(OperandDisposition::Preserved {
                region: final_index[position],
            }),
            Some(PendingDisposition::Contributes(positions)) => {
                Ok(OperandDisposition::Contributes {
                    regions: remap(positions),
                })
            }
            Some(PendingDisposition::Absorbed(positions)) => Ok(OperandDisposition::Absorbed {
                regions: remap(positions),
            }),
            None => Err(refuse(
                NaryFuseRefusal::ResultUnqualified,
                format!("operand {index} has no disposition"),
            )),
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut slots: Vec<Option<PendingRegion>> = regions.into_iter().map(Some).collect();
    let mut sorted_regions = Vec::with_capacity(slots.len());
    for &position in &order {
        if let Some(region) = slots[position].take() {
            sorted_regions.push(NaryFuseRegion {
                solid: region.solid,
                evolution: region.evolution,
                face_operands: region.face_operands,
                operands: region.operands,
            });
        }
    }
    let compound = topo.add_compound(Compound::new(
        sorted_regions.iter().map(|region| region.solid).collect(),
    ));
    Ok(NaryFuseResult {
        compound,
        regions: sorted_regions,
        operands,
    })
}

/// Connected components of the touching-pair graph, each ascending, ordered
/// by their lowest operand index.
fn touching_groups(count: usize, pairs: &[(usize, usize)]) -> Vec<Vec<usize>> {
    fn root(parent: &mut [usize], mut index: usize) -> usize {
        while parent[index] != index {
            parent[index] = parent[parent[index]];
            index = parent[index];
        }
        index
    }
    let mut parent: Vec<usize> = (0..count).collect();
    for &(i, j) in pairs {
        let (a, b) = (root(&mut parent, i), root(&mut parent, j));
        // The lower index stays the root, so a group's root is its minimum.
        parent[a.max(b)] = a.min(b);
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for index in 0..count {
        let group = root(&mut parent, index);
        groups.entry(group).or_default().push(index);
    }
    groups.into_values().collect()
}

/// Fuse one touching group of two or more operands, qualify the regions it
/// yields and record them and its operands' dispositions.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn fuse_group(
    topo: &mut Topology,
    solids: &[SolidId],
    group: &[usize],
    pairs: &[(usize, usize)],
    boxes: &[Aabb3],
    face_owner: &BTreeMap<usize, usize>,
    context: &remus_math::context::OperationContext,
    regions: &mut Vec<PendingRegion>,
    dispositions: &mut [Option<PendingDisposition>],
) -> Result<(), OperationsError> {
    let tolerance = context.tolerance;
    for &operand in group {
        if let Some(tag) = first_curved_entity(topo, solids[operand])? {
            return Err(refuse(
                NaryFuseRefusal::CurvedCluster,
                format!("operand {operand} carries `{tag}` geometry and may touch another"),
            ));
        }
    }

    // The group's pairs in group-local indices. `group` is ascending and
    // `pairs` is in all-pairs order, so the local list is too.
    let local: BTreeMap<usize, usize> = group
        .iter()
        .enumerate()
        .map(|(position, &operand)| (operand, position))
        .collect();
    let local_pairs: Vec<(usize, usize)> = pairs
        .iter()
        .filter_map(|&(i, j)| Some((*local.get(&i)?, *local.get(&j)?)))
        .collect();
    let group_solids: Vec<SolidId> = group.iter().map(|&operand| solids[operand]).collect();
    let group_boxes: Vec<Aabb3> = group.iter().map(|&operand| boxes[operand]).collect();

    // The result is qualified where it is built, in the arrangement's own
    // store, so a refusal never reaches the caller's arena. The gate also
    // reports which regions hold each operand's vertices, for the
    // dispositions of operands that keep no face.
    let mut vertices_in: Vec<Vec<usize>> = Vec::new();
    let drops = remus_algo::gfa::SectionDropScope::enter();
    let fused = remus_algo::gfa::fuse_n_regions_with_entity_evolution(
        topo,
        &group_solids,
        Some(&local_pairs),
        context,
        &mut |store, store_operands, store_regions| {
            // A section chain dropped building this result, or an
            // uncertified chart, is the exact engine declining.
            if drops.dropped() > 0 || super::require_chart_certificates(&drops).is_err() {
                return Err(AlgoError::UnsupportedNaryFuse {
                    cause: NaryFuseRefusal::ResultUnqualified,
                    reason: "the arrangement dropped a section chain or left a chart uncertified"
                        .into(),
                });
            }
            qualify_store_result(
                store,
                store_operands,
                &group_boxes,
                store_regions,
                tolerance,
            )
            .map(|held| vertices_in = held)
            .map_err(|error| match error {
                OperationsError::Algo(error) => error,
                other => AlgoError::UnsupportedNaryFuse {
                    cause: NaryFuseRefusal::ResultUnqualified,
                    reason: format!("the result could not be qualified: {other}"),
                },
            })
        },
    )
    .map_err(|error| match error {
        // The arrangement itself gave up (a split, a classification sample
        // or shell assembly failed). To the caller that is one more operand
        // list this entry point cannot answer exactly, so it gets the typed
        // refusal, with the engine's own words kept in the reason.
        AlgoError::IntersectionFailed(_)
        | AlgoError::FaceSplitFailed(_)
        | AlgoError::AssemblyFailed(_) => AlgoError::UnsupportedNaryFuse {
            cause: NaryFuseRefusal::ResultUnqualified,
            reason: format!("the exact arrangement failed: {error}"),
        },
        AlgoError::ClassificationFailed(_) => AlgoError::UnsupportedNaryFuse {
            cause: NaryFuseRefusal::ClassificationUnresolved,
            reason: format!("the exact arrangement failed: {error}"),
        },
        other => other,
    })?;

    // Lineage: every result face claimed once by a face of a group operand.
    // (Unresolved edges were refused in the store.)
    let base = regions.len();
    let mut contributes: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut pending = Vec::with_capacity(fused.regions.len());
    for (position, region) in fused.regions.iter().enumerate() {
        let face_operands = region_face_operands(topo, region, face_owner, group, position)?;
        let mut operands = face_operands.clone();
        operands.compact_sort_unstable();
        operands.dedup();
        for &operand in &operands {
            contributes
                .entry(operand)
                .or_default()
                .push(base + position);
        }
        pending.push((face_operands, operands));
    }

    for (position, &operand) in group.iter().enumerate() {
        let kept = fused.sources.get(position).map_or(0, |tally| tally.kept);
        let disposition = match (contributes.remove(&operand), kept) {
            (Some(positions), kept) if kept > 0 => PendingDisposition::Contributes(positions),
            (None, 0) => PendingDisposition::Absorbed(
                vertices_in
                    .get(position)
                    .into_iter()
                    .flatten()
                    .map(|&region| base + region)
                    .collect(),
            ),
            // The builder's tally and the exported lineage disagree about
            // whether this operand has a face in the result.
            _ => {
                return Err(refuse(
                    NaryFuseRefusal::LineageUnresolved,
                    format!("operand {operand}: kept-face tally and face lineage disagree"),
                ));
            }
        };
        dispositions[operand] = Some(disposition);
    }

    for (region, (face_operands, operands)) in fused.regions.into_iter().zip(pending) {
        regions.push(PendingRegion {
            solid: region.solid,
            evolution: region.evolution,
            face_operands,
            operands,
        });
    }
    Ok(())
}

/// Everything a group's result must satisfy besides its lineage, read in
/// the topology that holds both the operands and the regions: structure of
/// every region, contact between regions, and material against the
/// operands. Returns, per operand, the regions holding its vertices.
fn qualify_store_result(
    topo: &Topology,
    operands: &[SolidId],
    operand_boxes: &[Aabb3],
    regions: &[SolidId],
    tolerance: remus_math::tolerance::Tolerance,
) -> Result<Vec<Vec<usize>>, OperationsError> {
    if regions.is_empty() {
        return Err(refuse(
            NaryFuseRefusal::ResultUnqualified,
            "the arrangement produced no region",
        ));
    }
    for (position, &region) in regions.iter().enumerate() {
        qualify_region_structure(topo, region, position)?;
    }
    refuse_coincident_vertices(topo, regions, tolerance.linear)?;
    refuse_touching_regions(topo, regions, tolerance.linear)?;
    let held = operand_vertex_regions(topo, operands, regions, tolerance)?;
    refuse_unowned_region_vertices(topo, operands, operand_boxes, regions, tolerance)?;
    Ok(held)
}

/// The first non-planar face or non-line edge of `solid`, as its type tag.
fn first_curved_entity(
    topo: &Topology,
    solid: SolidId,
) -> Result<Option<&'static str>, OperationsError> {
    for face_id in remus_topology::explorer::solid_faces(topo, solid)? {
        let face = topo.face(face_id)?;
        if !face.surface().is_planar() {
            return Ok(Some(face.surface().type_tag()));
        }
        for &wire_id in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
            for oriented in topo.wire(wire_id)?.edges() {
                let curve = topo.edge(oriented.edge())?.curve();
                if !matches!(curve, remus_topology::edge::EdgeCurve::Line) {
                    return Ok(Some(curve.type_tag()));
                }
            }
        }
    }
    Ok(None)
}

/// Refuse a region that is not a valid closed two-manifold.
///
/// Three independent readings: the boolean result validator, the by-edge
/// closed-manifold count, and the vertex links. The last catches what the
/// first two cannot see — two sheets of the boundary meeting at one vertex,
/// where every edge still has exactly two uses.
fn qualify_region_structure(
    topo: &Topology,
    solid: SolidId,
    position: usize,
) -> Result<(), OperationsError> {
    if !super::is_closed_manifold(topo, solid)? {
        return Err(refuse(
            NaryFuseRefusal::NonManifoldContact,
            format!("region {position} has an edge that is not used exactly twice"),
        ));
    }
    if let Some(vertex) = pinched_vertex(topo, solid)? {
        return Err(refuse(
            NaryFuseRefusal::NonManifoldContact,
            format!("region {position} touches itself at vertex {vertex}"),
        ));
    }
    if super::validate_boolean_result(topo, solid).is_err()
        || !crate::validate::validate_solid(topo, solid)?.is_valid()
    {
        return Err(refuse(
            NaryFuseRefusal::ResultUnqualified,
            format!("region {position} failed validation"),
        ));
    }
    Ok(())
}

/// A vertex of `solid` whose incident faces do not form one fan, if any.
///
/// Every loop corner at a vertex joins the edge entering it to the edge
/// leaving it. On a two-manifold those joins link all of the vertex's edges
/// into a single cycle; two cycles are two sheets touching at a point.
fn pinched_vertex(topo: &Topology, solid: SolidId) -> Result<Option<usize>, OperationsError> {
    // vertex index → the (edge, edge) joins of the corners at it.
    let mut corners: BTreeMap<usize, Vec<(usize, usize)>> = BTreeMap::new();
    for face_id in remus_topology::explorer::solid_faces(topo, solid)? {
        let face = topo.face(face_id)?;
        for &wire_id in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
            let edges = topo.wire(wire_id)?.edges();
            for (position, oriented) in edges.iter().enumerate() {
                let next = &edges[(position + 1) % edges.len()];
                let edge = topo.edge(oriented.edge())?;
                // The vertex this use ends at is where the next one starts.
                let corner = if oriented.is_forward() {
                    edge.end()
                } else {
                    edge.start()
                };
                corners
                    .entry(corner.index())
                    .or_default()
                    .push((oriented.edge().index(), next.edge().index()));
            }
        }
    }
    for (vertex, joins) in corners {
        let mut component: BTreeMap<usize, usize> = BTreeMap::new();
        for &(a, b) in &joins {
            component.entry(a).or_insert(a);
            component.entry(b).or_insert(b);
        }
        // Label propagation over at most a vertex's handful of edges.
        let mut changed = true;
        while changed {
            changed = false;
            for &(a, b) in &joins {
                let (label_a, label_b) = (component[&a], component[&b]);
                if label_a != label_b {
                    let low = label_a.min(label_b);
                    component.insert(a, low);
                    component.insert(b, low);
                    changed = true;
                }
            }
        }
        let labels: BTreeSet<usize> = component.into_values().collect();
        if labels.len() > 1 {
            return Ok(Some(vertex));
        }
    }
    Ok(None)
}

/// Refuse two distinct result vertices at one position.
///
/// The arrangement merges coincident vertices, so two that survive apart mark
/// a contact it did not resolve: two regions meeting at a corner, or one
/// region folded onto itself. Entity ids cannot show it; positions do.
fn refuse_coincident_vertices(
    topo: &Topology,
    regions: &[SolidId],
    tolerance: f64,
) -> Result<(), OperationsError> {
    let mut points: Vec<(usize, Point3)> = Vec::new();
    let mut seen = BTreeSet::new();
    for &region in regions {
        for vertex in remus_topology::explorer::solid_vertices(topo, region)? {
            if seen.insert(vertex.index()) {
                points.push((vertex.index(), topo.vertex(vertex)?.point()));
            }
        }
    }
    points.compact_sort_by(|a, b| a.1.x().total_cmp(&b.1.x()));
    for (position, &(first, p)) in points.iter().enumerate() {
        for &(second, q) in &points[position + 1..] {
            if q.x() - p.x() > tolerance {
                break;
            }
            if (q - p).length() <= tolerance {
                return Err(refuse(
                    NaryFuseRefusal::NonManifoldContact,
                    format!("result vertices {first} and {second} coincide"),
                ));
            }
        }
    }
    Ok(())
}

/// Refuse two regions of one arrangement whose boundaries touch.
///
/// Regions of a union are disjoint as point sets. Two that meet along an
/// edge or at a point are one non-manifold body the solid model cannot
/// represent, and two that meet over an area are a union left unmerged.
fn refuse_touching_regions(
    topo: &Topology,
    regions: &[SolidId],
    tolerance: f64,
) -> Result<(), OperationsError> {
    if regions.len() < 2 {
        return Ok(());
    }
    let boxes: Vec<Option<Aabb3>> = regions
        .iter()
        .map(|&region| crate::measure::solid_vertex_bounding_box(topo, region))
        .collect();
    for first in 0..regions.len() {
        for second in (first + 1)..regions.len() {
            let apart = match (boxes[first], boxes[second]) {
                (Some(a), Some(b)) => !a.expanded(tolerance).intersects(b.expanded(tolerance)),
                _ => false,
            };
            if apart {
                continue;
            }
            let gap =
                crate::distance::solid_to_solid_distance(topo, regions[first], regions[second])?;
            if gap.distance <= tolerance {
                return Err(refuse(
                    NaryFuseRefusal::NonManifoldContact,
                    format!("regions {first} and {second} touch without sharing a face"),
                ));
            }
        }
    }
    Ok(())
}

/// The operand owning each face of `region`, parallel to its
/// `evolution.faces`, after checking that the lineage is total.
fn region_face_operands(
    topo: &Topology,
    region: &BooleanRegion,
    face_owner: &BTreeMap<usize, usize>,
    group: &[usize],
    position: usize,
) -> Result<Vec<usize>, OperationsError> {
    let unresolved = |what: String| refuse(NaryFuseRefusal::LineageUnresolved, what);
    let result_faces: BTreeSet<usize> = remus_topology::explorer::solid_faces(topo, region.solid)?
        .into_iter()
        .map(remus_topology::FaceId::index)
        .collect();
    let claimed: BTreeSet<usize> = region
        .evolution
        .faces
        .iter()
        .map(|&(face, _)| face)
        .collect();
    if claimed != result_faces || claimed.len() != region.evolution.faces.len() {
        return Err(unresolved(format!(
            "region {position}: face lineage is not one claim per result face"
        )));
    }
    let mut face_operands = Vec::with_capacity(region.evolution.faces.len());
    for &(face, source) in &region.evolution.faces {
        let owner = source
            .and_then(|source| face_owner.get(&source).copied())
            .filter(|owner| group.contains(owner))
            .ok_or_else(|| {
                unresolved(format!(
                    "region {position}: result face {face} names no operand face"
                ))
            })?;
        face_operands.push(owner);
    }
    // Belt and braces: the store refused unresolved edges before export.
    if region
        .evolution
        .edges
        .iter()
        .any(|(_, event)| matches!(event, EdgeEvent::Unresolved))
    {
        return Err(unresolved(format!(
            "region {position}: a result edge has no construction record"
        )));
    }
    Ok(face_operands)
}

fn solid_vertex_points(topo: &Topology, solid: SolidId) -> Result<Vec<Point3>, OperationsError> {
    remus_topology::explorer::solid_vertices(topo, solid)?
        .into_iter()
        .map(|vertex| Ok(topo.vertex(vertex)?.point()))
        .collect()
}

/// Whether each point lies in `solid` or on its boundary.
///
/// The boundary is decided by certified distance to it, the interior by the
/// point classifier. The classifier alone is not enough: it reports some
/// corners of a plain box as outside (three of the eight, for `make_box`),
/// and every point asked about here is a vertex of something.
fn closed_solid_holds(
    topo: &Topology,
    solid: SolidId,
    points: &[Point3],
    tolerance: f64,
) -> Result<Vec<bool>, OperationsError> {
    let distances = crate::distance::point_to_solid_batch(topo, points, solid)?;
    let mut held: Vec<bool> = distances
        .iter()
        .map(|result| result.distance <= tolerance)
        .collect();
    let (indices, interior): (Vec<usize>, Vec<Point3>) = points
        .iter()
        .enumerate()
        .filter(|&(index, _)| !held[index])
        .map(|(index, &point)| (index, point))
        .unzip();
    if !interior.is_empty() {
        let classes = crate::classify::classify_points(topo, solid, &interior, 0.0, tolerance)?;
        for (index, class) in indices.into_iter().zip(classes) {
            held[index] = class == PointClassification::Inside;
        }
    }
    Ok(held)
}

/// For each operand, the regions holding at least one of its vertices —
/// after checking that every vertex is in or on some region.
///
/// A union contains its operands, so an operand vertex outside every region
/// is material the result lost. This reads the result with certified
/// distance and the point classifier, not with the arrangement that built it.
fn operand_vertex_regions(
    topo: &Topology,
    operands: &[SolidId],
    regions: &[SolidId],
    tolerance: remus_math::tolerance::Tolerance,
) -> Result<Vec<Vec<usize>>, OperationsError> {
    let region_boxes: Vec<Option<Aabb3>> = regions
        .iter()
        .map(|&region| crate::measure::solid_vertex_bounding_box(topo, region))
        .collect();
    let mut held = Vec::with_capacity(operands.len());
    for (operand, &solid) in operands.iter().enumerate() {
        let points = solid_vertex_points(topo, solid)?;
        let mut found = vec![false; points.len()];
        let mut regions_hit = Vec::new();
        for (position, &region) in regions.iter().enumerate() {
            // A region whose box holds none of the points holds none of them.
            let near = region_boxes[position].is_none_or(|bounds| {
                let bounds = bounds.expanded(tolerance.linear);
                points.iter().any(|&point| bounds.contains_point(point))
            });
            if !near {
                continue;
            }
            let mut hit = false;
            for (index, in_region) in closed_solid_holds(topo, region, &points, tolerance.linear)?
                .into_iter()
                .enumerate()
            {
                if in_region {
                    found[index] = true;
                    hit = true;
                }
            }
            if hit {
                regions_hit.push(position);
            }
        }
        if found.iter().any(|&inside| !inside) {
            return Err(refuse(
                NaryFuseRefusal::ResultUnqualified,
                format!("operand {operand} of its group has a vertex outside every result region"),
            ));
        }
        held.push(regions_hit);
    }
    Ok(held)
}

/// Refuse a region vertex that lies in or on no operand.
///
/// A union reaches no further than its operands, so such a vertex is
/// geometry no operand entitled the result to. `operand_boxes` are the
/// operands' conservative boxes grown by the contact margin, so a point
/// outside one is outside that operand.
fn refuse_unowned_region_vertices(
    topo: &Topology,
    operands: &[SolidId],
    operand_boxes: &[Aabb3],
    regions: &[SolidId],
    tolerance: remus_math::tolerance::Tolerance,
) -> Result<(), OperationsError> {
    for (position, &region) in regions.iter().enumerate() {
        let points = solid_vertex_points(topo, region)?;
        let mut owned = vec![false; points.len()];
        for (operand, &solid) in operands.iter().enumerate() {
            let candidates: Vec<usize> = (0..points.len())
                .filter(|&index| {
                    !owned[index]
                        && operand_boxes
                            .get(operand)
                            .is_none_or(|bounds| bounds.contains_point(points[index]))
                })
                .collect();
            if candidates.is_empty() {
                continue;
            }
            let subset: Vec<Point3> = candidates.iter().map(|&index| points[index]).collect();
            let in_operand = closed_solid_holds(topo, solid, &subset, tolerance.linear)?;
            for (&index, in_operand) in candidates.iter().zip(in_operand) {
                if in_operand {
                    owned[index] = true;
                }
            }
        }
        if owned.iter().any(|&inside| !inside) {
            return Err(refuse(
                NaryFuseRefusal::ResultUnqualified,
                format!("region {position} has a vertex outside every operand"),
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
