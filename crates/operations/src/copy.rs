//! Deep copy of topological entities.
//!
//! Creates independent copies of solids and all their sub-entities
//! (shells, faces, wires, edges, vertices) in the arena.

use std::collections::HashMap;
use std::collections::hash_map::Entry;
use std::ops::Range;

use remus_math::det_hash::DetHashMap;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::attributes::EntityAttributes;
use remus_topology::coedge::{CoedgeId, PeriodicWinding};
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::transform::{TransformPolicy, TransformRecorder, TransformReport};

struct VertexSnap {
    old_index: usize,
    point: Point3,
    tol: f64,
}

struct EdgeSnap {
    old_index: usize,
    start_index: usize,
    end_index: usize,
    curve: EdgeCurve,
    tolerance: Option<f64>,
    trim: Option<(f64, f64)>,
}

struct WireSnap {
    old_index: usize,
    edges: Vec<(usize, bool)>, // (edge_old_index, forward)
    closed: bool,
}

struct CoedgeAuthoritySnap {
    face_index: usize,
    edge_index: usize,
    forward: bool,
    pcurve: Option<PCurve>,
    periodic_winding: PeriodicWinding,
}

/// An edge of a [`CopyPlan`], its endpoints named by vertex ordinal.
struct PlanEdge {
    old_index: usize,
    start: usize,
    end: usize,
    curve: EdgeCurve,
    tolerance: Option<f64>,
    trim: Option<(f64, f64)>,
}

/// A wire of a [`CopyPlan`]: one `(edge ordinal, forward)` per use.
struct PlanWire {
    edges: Vec<(usize, bool)>,
    closed: bool,
}

/// One visit of a source face, its wires named by wire ordinal.
struct PlanFace {
    old_index: usize,
    outer: usize,
    inner: Vec<usize>,
    surface: FaceSurface,
    reversed: bool,
    attributes: Option<EntityAttributes>,
    /// This visit's snapshots in [`CopyPlan::authority`].
    authority: Range<usize>,
}

/// Read-phase snapshot of one solid for the solid copy paths.
///
/// Vertices, edges and wires are numbered by dense first-seen ordinal in
/// the traversal and faces by visit, so the write phase indexes vectors of
/// new handles instead of hashing source indices.
struct CopyPlan {
    vertices: Vec<VertexSnap>,
    edges: Vec<PlanEdge>,
    wires: Vec<PlanWire>,
    shells: Vec<Vec<PlanFace>>,
    /// Every visit's coedge authority, in snapshot (loop) order.
    authority: Vec<CoedgeAuthoritySnap>,
    /// The edge ordinal of each `authority` entry.
    authority_edges: Vec<usize>,
    /// Source edge index to edge ordinal.
    edge_ordinals: DetHashMap<usize, usize>,
    solid_attributes: Option<EntityAttributes>,
}

/// Copies minted from a [`CopyPlan`], each paired with its source index:
/// one per vertex and edge ordinal, and one per face visit.
struct CopyIds {
    solid: SolidId,
    vertices: Vec<(usize, VertexId)>,
    edges: Vec<(usize, EdgeId)>,
    faces: Vec<(usize, FaceId)>,
}

impl CopyIds {
    fn into_entities(self) -> CopiedSolidEntities {
        CopiedSolidEntities {
            solid: self.solid,
            // Visit order: a face listed twice maps to its last copy.
            face_map: self.faces.into_iter().collect(),
            edge_map: self.edges.into_iter().collect(),
            vertex_map: self.vertices.into_iter().collect(),
        }
    }
}

fn plan_error(reason: String) -> crate::OperationsError {
    remus_topology::TopologyError::NonManifold { reason }.into()
}

/// The entry minted for plan ordinal `ordinal`.
fn planned<T: Copy>(copies: &[T], ordinal: usize) -> Result<T, crate::OperationsError> {
    copies
        .get(ordinal)
        .copied()
        .ok_or_else(|| plan_error(format!("copy plan has no target for ordinal {ordinal}")))
}

fn face_coedges(topo: &Topology, face_id: FaceId) -> Result<Vec<CoedgeId>, crate::OperationsError> {
    let loop_ids = topo
        .loops_of_face(face_id)
        .ok_or(remus_topology::TopologyError::LoopWireMismatch { face: face_id })?;
    let mut uses = Vec::new();
    for &loop_id in loop_ids {
        uses.extend(topo.face_loop(loop_id)?.coedges().iter().copied());
    }
    Ok(uses)
}

fn snapshot_face_coedge_authority(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<CoedgeAuthoritySnap>, crate::OperationsError> {
    let mut snapshots = Vec::new();
    snapshot_face_coedge_authority_into(topo, face_id, &mut snapshots)?;
    Ok(snapshots)
}

fn snapshot_face_coedge_authority_into(
    topo: &Topology,
    face_id: FaceId,
    snapshots: &mut Vec<CoedgeAuthoritySnap>,
) -> Result<(), crate::OperationsError> {
    remus_topology::validation::validate_face_loops(topo, face_id)?;
    let loop_ids = topo
        .loops_of_face(face_id)
        .ok_or(remus_topology::TopologyError::LoopWireMismatch { face: face_id })?;
    // Validation resolved every loop and coedge, so walking loop by loop
    // cannot surface a different error than collecting all loops first.
    for &loop_id in loop_ids {
        for &coedge_id in topo.face_loop(loop_id)?.coedges() {
            let coedge = topo.coedge(coedge_id)?;
            snapshots.push(CoedgeAuthoritySnap {
                face_index: face_id.index(),
                edge_index: coedge.edge().index(),
                forward: coedge.is_forward(),
                pcurve: coedge.pcurve().cloned(),
                periodic_winding: coedge.periodic_winding(),
            });
        }
    }
    Ok(())
}

fn remapped_authority_edge(
    edge_map: &HashMap<usize, remus_topology::edge::EdgeId>,
    snapshot: &CoedgeAuthoritySnap,
) -> Result<remus_topology::edge::EdgeId, crate::OperationsError> {
    edge_map
        .get(&snapshot.edge_index)
        .copied()
        .ok_or_else(|| remus_topology::TopologyError::NonManifold {
            reason: format!(
                "copy plan has no target edge for authoritative source edge index {}",
                snapshot.edge_index
            ),
        })
        .map_err(Into::into)
}

fn restore_face_coedge_authority(
    topo: &mut Topology,
    face: FaceId,
    edge: remus_topology::edge::EdgeId,
    snapshot: CoedgeAuthoritySnap,
) -> Result<(), crate::OperationsError> {
    let matching: Vec<_> = face_coedges(topo, face)?
        .into_iter()
        .filter(|&coedge_id| {
            topo.coedge(coedge_id).is_ok_and(|coedge| {
                coedge.edge() == edge && coedge.is_forward() == snapshot.forward
            })
        })
        .collect();
    let [coedge_id] = matching.as_slice() else {
        return Err(remus_topology::TopologyError::NonManifold {
            reason: format!(
                "copied face {face:?} does not contain exactly one {} use of edge {edge:?}",
                if snapshot.forward {
                    "forward"
                } else {
                    "reverse"
                }
            ),
        }
        .into());
    };
    topo.set_coedge_periodic_winding(*coedge_id, snapshot.periodic_winding)?;
    if let Some(pcurve) = snapshot.pcurve {
        topo.set_coedge_pcurve(*coedge_id, pcurve)?;
    }
    Ok(())
}

/// Snapshots `solid_id` for copying, with every lookup in the order the
/// copy paths have always made them (so the first error is unchanged).
fn build_copy_plan(
    source: &Topology,
    solid_id: SolidId,
) -> Result<CopyPlan, crate::OperationsError> {
    let solid = source.solid(solid_id)?;
    let mut plan = CopyPlan {
        vertices: Vec::new(),
        edges: Vec::new(),
        wires: Vec::new(),
        shells: Vec::new(),
        authority: Vec::new(),
        authority_edges: Vec::new(),
        edge_ordinals: DetHashMap::default(),
        solid_attributes: source.attributes().solid(solid_id).cloned(),
    };
    let mut vertex_ordinals = DetHashMap::default();
    let mut wire_ordinals = DetHashMap::default();
    for shell_id in std::iter::once(solid.outer_shell()).chain(solid.inner_shells().iter().copied())
    {
        let shell = source.shell(shell_id)?;
        let face_count = shell.faces().len();
        wire_ordinals.reserve(face_count);
        plan.edge_ordinals.reserve(face_count.saturating_mul(2));
        vertex_ordinals.reserve(face_count.saturating_mul(2));
        let mut faces = Vec::with_capacity(face_count);
        for &face_id in shell.faces() {
            let face = source.face(face_id)?;
            let authority_start = plan.authority.len();
            snapshot_face_coedge_authority_into(source, face_id, &mut plan.authority)?;
            let outer = plan_wire_ordinal(
                source,
                face.outer_wire(),
                &mut plan,
                &mut wire_ordinals,
                &mut vertex_ordinals,
            )?;
            let mut inner = Vec::with_capacity(face.inner_wires().len());
            for &wire_id in face.inner_wires() {
                inner.push(plan_wire_ordinal(
                    source,
                    wire_id,
                    &mut plan,
                    &mut wire_ordinals,
                    &mut vertex_ordinals,
                )?);
            }
            // `validate_face_loops` matched coedge k of loop j to edge k of
            // wire j, so each snapshot's edge ordinal follows by position.
            for wire in std::iter::once(outer).chain(inner.iter().copied()) {
                let wire = plan
                    .wires
                    .get(wire)
                    .ok_or_else(|| plan_error(format!("copy plan has no wire {wire}")))?;
                plan.authority_edges
                    .extend(wire.edges.iter().map(|&(edge, _)| edge));
            }
            if plan.authority_edges.len() != plan.authority.len() {
                return Err(
                    remus_topology::TopologyError::LoopWireMismatch { face: face_id }.into(),
                );
            }
            faces.push(PlanFace {
                old_index: face_id.index(),
                outer,
                inner,
                surface: face.surface().clone(),
                reversed: face.is_reversed(),
                attributes: source.attributes().face(face_id).cloned(),
                authority: authority_start..plan.authority.len(),
            });
        }
        plan.shells.push(faces);
    }
    Ok(plan)
}

/// The ordinal of `wire_id`, snapshotting it (and its unseen edges and
/// vertices) on first sight.
fn plan_wire_ordinal(
    source: &Topology,
    wire_id: WireId,
    plan: &mut CopyPlan,
    wire_ordinals: &mut DetHashMap<usize, usize>,
    vertex_ordinals: &mut DetHashMap<usize, usize>,
) -> Result<usize, crate::OperationsError> {
    let slot = match wire_ordinals.entry(wire_id.index()) {
        Entry::Occupied(seen) => return Ok(*seen.get()),
        Entry::Vacant(slot) => slot,
    };
    let ordinal = plan.wires.len();
    slot.insert(ordinal);
    let wire = source.wire(wire_id)?;
    let mut edges = Vec::with_capacity(wire.edges().len());
    for oriented in wire.edges() {
        let edge_ordinal = match plan.edge_ordinals.entry(oriented.edge().index()) {
            Entry::Occupied(seen) => *seen.get(),
            Entry::Vacant(slot) => {
                let edge_ordinal = plan.edges.len();
                slot.insert(edge_ordinal);
                let edge = source.edge(oriented.edge())?;
                let start =
                    plan_vertex_ordinal(source, edge.start(), &mut plan.vertices, vertex_ordinals)?;
                let end =
                    plan_vertex_ordinal(source, edge.end(), &mut plan.vertices, vertex_ordinals)?;
                plan.edges.push(PlanEdge {
                    old_index: oriented.edge().index(),
                    start,
                    end,
                    curve: edge.curve().clone(),
                    tolerance: edge.tolerance(),
                    trim: edge.trim(),
                });
                edge_ordinal
            }
        };
        edges.push((edge_ordinal, oriented.is_forward()));
    }
    plan.wires.push(PlanWire {
        edges,
        closed: wire.is_closed(),
    });
    Ok(ordinal)
}

/// The ordinal of `vertex_id`, snapshotting it on first sight.
fn plan_vertex_ordinal(
    source: &Topology,
    vertex_id: VertexId,
    vertices: &mut Vec<VertexSnap>,
    vertex_ordinals: &mut DetHashMap<usize, usize>,
) -> Result<usize, crate::OperationsError> {
    match vertex_ordinals.entry(vertex_id.index()) {
        Entry::Occupied(seen) => Ok(*seen.get()),
        Entry::Vacant(slot) => {
            let ordinal = vertices.len();
            slot.insert(ordinal);
            let vertex = source.vertex(vertex_id)?;
            vertices.push(VertexSnap {
                old_index: vertex_id.index(),
                point: vertex.point(),
                tol: vertex.tolerance(),
            });
            Ok(ordinal)
        }
    }
}

fn add_planned_wires(
    topo: &mut Topology,
    wires: Vec<PlanWire>,
    edges: &[(usize, EdgeId)],
) -> Result<Vec<WireId>, crate::OperationsError> {
    let mut new_wires = Vec::with_capacity(wires.len());
    for wire in wires {
        let oriented = wire
            .edges
            .into_iter()
            .map(|(edge, forward)| Ok(OrientedEdge::new(planned(edges, edge)?.1, forward)))
            .collect::<Result<Vec<_>, crate::OperationsError>>()?;
        let wire = Wire::new(oriented, wire.closed).map_err(crate::OperationsError::Topology)?;
        new_wires.push(topo.add_wire(wire));
    }
    Ok(new_wires)
}

fn add_planned_face(
    topo: &mut Topology,
    (outer, inner): (usize, &[usize]),
    surface: FaceSurface,
    reversed: bool,
    wires: &[WireId],
) -> Result<FaceId, crate::OperationsError> {
    let outer = planned(wires, outer)?;
    let inner = inner
        .iter()
        .map(|&wire| planned(wires, wire))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(topo.add_face(if reversed {
        Face::new_reversed(outer, inner, surface)
    } else {
        Face::new(outer, inner, surface)
    }))
}

fn add_planned_solid(
    topo: &mut Topology,
    shells: &[ShellId],
    attributes: Option<EntityAttributes>,
) -> Result<SolidId, crate::OperationsError> {
    let (&outer, inner) = shells
        .split_first()
        .ok_or_else(|| plan_error("copy plan has no outer shell".to_owned()))?;
    let solid = topo.add_solid(Solid::new(outer, inner.to_vec()));
    if let Some(attributes) = attributes {
        topo.set_solid_attributes(solid, attributes)?;
    }
    Ok(solid)
}

/// For each face visit, the copy a source-index face map names: the copy
/// made at the last visit of the same source face.
fn last_visit_targets(faces: &[(usize, FaceId)]) -> Vec<FaceId> {
    let mut targets: Vec<FaceId> = faces.iter().map(|&(_, face)| face).collect();
    let mut visits: Vec<(usize, usize)> = faces
        .iter()
        .enumerate()
        .map(|(visit, &(source, _))| (source, visit))
        .collect();
    visits.sort_unstable();
    for run in visits.chunk_by(|a, b| a.0 == b.0) {
        if let [.., (_, last)] = run
            && run.len() > 1
            && let Some(&face) = targets.get(*last)
        {
            for &(_, visit) in run {
                if let Some(target) = targets.get_mut(visit) {
                    *target = face;
                }
            }
        }
    }
    targets
}

/// Whether coedge `k` of `coedges` is the one use of `(edges[k], forward)`
/// for every snapshot `k`. Then the map-based search for each snapshot
/// can only find the coedge at the same position.
fn matches_by_position(
    topo: &Topology,
    coedges: &[CoedgeId],
    snapshots: &[CoedgeAuthoritySnap],
    snapshot_edges: &[usize],
    edges: &[(usize, EdgeId)],
) -> bool {
    if coedges.len() != snapshots.len() || snapshot_edges.len() != snapshots.len() {
        return false;
    }
    for ((&coedge, snapshot), &edge) in coedges.iter().zip(snapshots).zip(snapshot_edges) {
        let (Some(&(_, edge)), Ok(coedge)) = (edges.get(edge), topo.coedge(coedge)) else {
            return false;
        };
        if coedge.edge() != edge || coedge.is_forward() != snapshot.forward {
            return false;
        }
    }
    let keys = || {
        snapshot_edges
            .iter()
            .copied()
            .zip(snapshots.iter().map(|snapshot| snapshot.forward))
    };
    if snapshots.len() <= 16 {
        keys()
            .enumerate()
            .all(|(i, key)| keys().take(i).all(|other| other != key))
    } else {
        let mut sorted: Vec<_> = keys().collect();
        sorted.sort_unstable();
        sorted.windows(2).all(|pair| pair[0] != pair[1])
    }
}

/// Restores every face visit's coedge authority, in snapshot order, onto
/// the copy a source-index face map names (see [`last_visit_targets`]).
///
/// A visit whose copied coedges match its snapshots by position writes
/// them directly; any other visit replays the per-snapshot search, with its
/// `NonManifold` refusal for an ambiguous `(edge, orientation)`.
fn restore_planned_authority(
    topo: &mut Topology,
    faces: &[(usize, FaceId)],
    ranges: &[Range<usize>],
    mut authority: Vec<CoedgeAuthoritySnap>,
    authority_edges: &[usize],
    edges: &[(usize, EdgeId)],
) -> Result<(), crate::OperationsError> {
    for (range, target) in ranges.iter().zip(last_visit_targets(faces)) {
        let (Some(snapshots), Some(snapshot_edges)) = (
            authority.get_mut(range.clone()),
            authority_edges.get(range.clone()),
        ) else {
            return Err(plan_error(format!(
                "copy plan has no authority snapshots {range:?}"
            )));
        };
        if snapshots.is_empty() {
            continue;
        }
        let coedges = face_coedges(topo, target)?;
        if matches_by_position(topo, &coedges, snapshots, snapshot_edges, edges) {
            for (&coedge, snapshot) in coedges.iter().zip(snapshots) {
                topo.set_coedge_periodic_winding(coedge, snapshot.periodic_winding)?;
                if let Some(pcurve) = snapshot.pcurve.take() {
                    topo.set_coedge_pcurve(coedge, pcurve)?;
                }
            }
        } else {
            for (snapshot, &edge) in snapshots.iter_mut().zip(snapshot_edges) {
                let (_, edge) = planned(edges, edge)?;
                let snapshot = CoedgeAuthoritySnap {
                    face_index: snapshot.face_index,
                    edge_index: snapshot.edge_index,
                    forward: snapshot.forward,
                    pcurve: snapshot.pcurve.take(),
                    periodic_winding: snapshot.periodic_winding,
                };
                restore_face_coedge_authority(topo, target, edge, snapshot)?;
            }
        }
    }
    Ok(())
}

/// Writes a plan into `destination`: vertices, edges, wires, then each
/// shell's faces (with attributes) and the shell, then coedge authority,
/// then the solid. `face_count` is the caller's face reservation.
fn materialize_copy(
    destination: &mut Topology,
    plan: CopyPlan,
    face_count: usize,
) -> Result<CopyIds, crate::OperationsError> {
    let CopyPlan {
        vertices,
        edges,
        wires,
        shells,
        authority,
        authority_edges,
        edge_ordinals: _,
        solid_attributes,
    } = plan;
    destination.reserve(
        vertices.len(),
        edges.len(),
        wires.len(),
        face_count,
        shells.len(),
        1,
    );
    let vertices: Vec<_> = vertices
        .into_iter()
        .map(|vertex| {
            let copy = destination.add_vertex(Vertex::new(vertex.point, vertex.tol));
            (vertex.old_index, copy)
        })
        .collect();
    let mut new_edges = Vec::with_capacity(edges.len());
    for edge in edges {
        let mut copied = Edge::with_tolerance(
            planned(&vertices, edge.start)?.1,
            planned(&vertices, edge.end)?.1,
            edge.curve,
            edge.tolerance,
        );
        copied.set_trim(edge.trim);
        new_edges.push((edge.old_index, destination.add_edge(copied)));
    }
    let new_wires = add_planned_wires(destination, wires, &new_edges)?;
    let mut new_shells = Vec::with_capacity(shells.len());
    let mut new_faces = Vec::with_capacity(face_count);
    let mut ranges = Vec::with_capacity(face_count);
    for shell in shells {
        let mut shell_faces = Vec::with_capacity(shell.len());
        for face in shell {
            let new_face = add_planned_face(
                destination,
                (face.outer, &face.inner),
                face.surface,
                face.reversed,
                &new_wires,
            )?;
            if let Some(attributes) = face.attributes {
                destination.set_face_attributes(new_face, attributes)?;
            }
            new_faces.push((face.old_index, new_face));
            ranges.push(face.authority);
            shell_faces.push(new_face);
        }
        new_shells.push(
            destination
                .add_shell(Shell::new(shell_faces).map_err(crate::OperationsError::Topology)?),
        );
    }
    restore_planned_authority(
        destination,
        &new_faces,
        &ranges,
        authority,
        &authority_edges,
        &new_edges,
    )?;
    let solid = add_planned_solid(destination, &new_shells, solid_attributes)?;
    Ok(CopyIds {
        solid,
        vertices,
        edges: new_edges,
        faces: new_faces,
    })
}

/// Copies a solid within one arena.
fn copy_solid_ids(
    topo: &mut Topology,
    solid_id: SolidId,
) -> Result<CopyIds, crate::OperationsError> {
    let plan = build_copy_plan(topo, solid_id)?;
    let face_count = plan
        .shells
        .iter()
        .map(Vec::len)
        .fold(0usize, usize::saturating_add);
    materialize_copy(topo, plan, face_count)
}

/// Copies a solid between independent arenas.
fn copy_solid_between_ids(
    source: &Topology,
    destination: &mut Topology,
    solid_id: SolidId,
) -> Result<CopyIds, crate::OperationsError> {
    let plan = build_copy_plan(source, solid_id)?;
    let face_count = plan.shells.iter().map(Vec::len).sum();
    materialize_copy(destination, plan, face_count)
}

/// Copy one solid between independent topology arenas.
///
/// This is used by speculative operations that run in a cloned topology: only
/// the accepted result is materialized back into the caller's arena. Stored
/// oriented pcurves on boundary uses are copied with their owning faces.
pub(crate) fn copy_solid_between(
    source: &Topology,
    destination: &mut Topology,
    solid_id: SolidId,
) -> Result<SolidId, crate::OperationsError> {
    copy_solid_between_ids(source, destination, solid_id).map(|copied| copied.solid)
}

pub(crate) struct CopiedSolidEntities {
    pub solid: SolidId,
    pub face_map: HashMap<usize, FaceId>,
    pub edge_map: HashMap<usize, remus_topology::EdgeId>,
    pub vertex_map: HashMap<usize, VertexId>,
}

pub(crate) fn copy_solid_between_with_entity_map(
    source: &Topology,
    destination: &mut Topology,
    solid_id: SolidId,
) -> Result<CopiedSolidEntities, crate::OperationsError> {
    copy_solid_between_ids(source, destination, solid_id).map(CopyIds::into_entities)
}

/// Create a deep copy of a solid and all its topology.
///
/// Returns a new `SolidId` for the copy. The original solid is not modified.
/// All vertices, edges, wires, faces, shells, and oriented boundary pcurves are
/// duplicated.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn copy_solid(
    topo: &mut Topology,
    solid_id: SolidId,
) -> Result<SolidId, crate::OperationsError> {
    copy_solid_ids(topo, solid_id).map(|copied| copied.solid)
}

/// [`copy_solid`], additionally reporting which copied face came from which
/// original face.
///
/// The mapping is `original face index -> copied face index`. It is exact by
/// construction — the copy walks the original's shells in order and mints one
/// face per face — so an operation built on top of a copy (a pattern instance,
/// say) can hand a consumer real provenance instead of matching geometry that
/// is, by definition, identical for every instance.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn copy_solid_with_face_map(
    topo: &mut Topology,
    solid_id: SolidId,
) -> Result<(SolidId, HashMap<usize, usize>), crate::OperationsError> {
    let copied = copy_solid_ids(topo, solid_id)?;
    Ok((
        copied.solid,
        copied
            .faces
            .into_iter()
            .map(|(source, face)| (source, face.index()))
            .collect(),
    ))
}

pub(crate) fn copy_solid_with_entity_map(
    topo: &mut Topology,
    solid_id: SolidId,
) -> Result<CopiedSolidEntities, crate::OperationsError> {
    copy_solid_ids(topo, solid_id).map(CopyIds::into_entities)
}

/// Create a deep copy of a solid with a simultaneous affine transform.
///
/// Equivalent to `copy_solid` followed by `transform_solid`, but performs both
/// in a single traversal — applying the matrix during the write phase instead
/// of allocating untransformed entities and then mutating them.
///
/// Runs transacted: a refused edge or surface image rolls back every
/// allocated copy entity instead of leaking a partial copy into the arena.
///
/// This is the legacy entry point: see [`transform_solid_detailed`] for the
/// quality contract. Anisotropic maps convert carriers without disclosure
/// here; use [`copy_and_transform_solid_detailed`] for the typed report.
///
/// [`transform_solid_detailed`]: crate::transform::transform_solid_detailed
///
/// # Errors
///
/// Returns an error if any topology lookup fails or the matrix is degenerate.
#[allow(clippy::too_many_lines)]
pub fn copy_and_transform_solid(
    topo: &mut Topology,
    solid_id: SolidId,
    matrix: &remus_math::mat::Mat4,
) -> Result<SolidId, crate::OperationsError> {
    crate::transform::reject_degenerate_transform(matrix)?;
    let normal_matrix = matrix.inverse()?.transpose();
    let mut recorder =
        TransformRecorder::new(crate::transform::linear_determinant(matrix) < 0.0, true);
    remus_topology::transaction::run_transacted(topo, |live| {
        copy_and_transform_solid_impl(live, solid_id, matrix, &normal_matrix, &mut recorder)
    })
}

/// Quality-aware additive twin of [`copy_and_transform_solid`].
///
/// Commits the same copy and returns a [`TransformReport`] naming every
/// carrier-family change and disclosing fitted output. Under
/// [`TransformPolicy::ExactOnly`] a face outside the exact sphere class
/// refuses with `ExactOnlyUnattainable` before allocating anything.
///
/// Runs transacted like the legacy entry point.
///
/// # Errors
///
/// Returns an error if any topology lookup fails, the matrix is degenerate,
/// an edge or surface image is unrepresentable, or the exact-only policy
/// declines a sampled fit.
#[allow(clippy::too_many_lines)]
pub fn copy_and_transform_solid_detailed(
    topo: &mut Topology,
    solid_id: SolidId,
    matrix: &remus_math::mat::Mat4,
    policy: TransformPolicy,
) -> Result<(SolidId, TransformReport), crate::OperationsError> {
    copy_and_transform_solid_detailed_with_refusal(topo, solid_id, matrix, policy, &mut None)
}

/// Like [`copy_and_transform_solid_detailed`], retaining the source face on
/// an execution-time exact-only refusal after the copied topology rolls back.
///
/// # Errors
///
/// Returns the same errors as [`copy_and_transform_solid_detailed`].
pub fn copy_and_transform_solid_detailed_with_refusal(
    topo: &mut Topology,
    solid_id: SolidId,
    matrix: &remus_math::mat::Mat4,
    policy: TransformPolicy,
    refused_face: &mut Option<FaceId>,
) -> Result<(SolidId, TransformReport), crate::OperationsError> {
    *refused_face = None;
    crate::transform::reject_degenerate_transform(matrix)?;
    let normal_matrix = matrix.inverse()?.transpose();
    if matches!(policy, TransformPolicy::ExactOnly) {
        let plans = crate::transform::preflight_solid_transform(topo, solid_id, matrix)?;
        crate::transform::refuse_unless_exact(&plans)?;
    }
    let determinant = crate::transform::linear_determinant(matrix);
    let reversing = determinant < 0.0;
    let similarity = crate::transform::is_similarity(matrix);
    let mut recorder = TransformRecorder::new(
        reversing,
        matches!(policy, TransformPolicy::AllowApproximate),
    );
    let result = remus_topology::transaction::run_transacted(topo, |live| {
        copy_and_transform_solid_impl(live, solid_id, matrix, &normal_matrix, &mut recorder)
    });
    *refused_face = recorder.refused_face();
    let copied = result?;
    Ok((copied, recorder.into_report(determinant, similarity)))
}

#[allow(clippy::too_many_lines)]
fn copy_and_transform_solid_impl(
    topo: &mut Topology,
    solid_id: SolidId,
    matrix: &remus_math::mat::Mat4,
    normal_matrix: &remus_math::mat::Mat4,
    recorder: &mut TransformRecorder,
) -> Result<SolidId, crate::OperationsError> {
    let chart_reversing = crate::transform::linear_determinant(matrix) < 0.0;
    let certificates = crate::transform::translation_edge_certificates(
        topo,
        &remus_topology::explorer::solid_edges(topo, solid_id)?,
        matrix,
    )?;

    // Read phase mirrors copy_solid.
    let CopyPlan {
        vertices,
        edges,
        wires,
        shells,
        authority,
        authority_edges,
        edge_ordinals,
        solid_attributes,
    } = build_copy_plan(topo, solid_id)?;

    topo.reserve(
        vertices.len(),
        edges.len(),
        wires.len(),
        shells
            .iter()
            .map(Vec::len)
            .fold(0usize, usize::saturating_add),
        shells.len(),
        1,
    );

    let vertices: Vec<_> = vertices
        .into_iter()
        .map(|vertex| {
            let new_point = matrix.mul_point(vertex.point);
            (
                vertex.old_index,
                topo.add_vertex(Vertex::new(new_point, vertex.tol)),
            )
        })
        .collect();

    let mut new_edges = Vec::with_capacity(edges.len());
    for edge in edges {
        let (_, new_start) = planned(&vertices, edge.start)?;
        let (_, new_end) = planned(&vertices, edge.end)?;
        // Shared with `transform::transform_edges` — including its exact trim
        // policy: retain where the map provably preserves the
        // parameterization, remap the handled Circle→Ellipse axis swap,
        // drop otherwise (RFC 0002).
        let from = edge.curve.type_tag();
        let (new_curve, new_trim) =
            crate::transform::transform_edge_curve_with_trim(&edge.curve, edge.trim, matrix)?;
        let to = new_curve.as_ref().map_or(from, |curve| curve.type_tag());
        let mut copied_edge = Edge::with_tolerance(
            new_start,
            new_end,
            new_curve.unwrap_or(EdgeCurve::Line),
            edge.tolerance,
        );
        copied_edge.set_trim(new_trim);
        let copied = topo.add_edge(copied_edge);
        recorder.record_edge(copied, from, to);
        new_edges.push((edge.old_index, copied));
    }

    crate::transform::restore_translation_certificates(
        topo,
        certificates
            .into_iter()
            .map(|(id, tolerance, budget)| {
                let ordinal = edge_ordinals
                    .get(&id.index())
                    .copied()
                    .ok_or_else(|| plan_error(format!("copy plan has no edge {}", id.index())))?;
                Ok((planned(&new_edges, ordinal)?.1, tolerance, budget))
            })
            .collect::<Result<_, crate::OperationsError>>()?,
    )?;

    // Wires carry no geometry to transform.
    let new_wires = add_planned_wires(topo, wires, &new_edges)?;

    let mut new_shell_ids = Vec::with_capacity(shells.len());
    let mut new_faces = Vec::new();
    let mut ranges = Vec::new();
    let mut chart_replaced_faces = Vec::new();
    for shell in shells {
        let mut new_face_ids = Vec::with_capacity(shell.len());
        for face in shell {
            // Copy the surface verbatim; the shared transformer below rewrites
            // it once the face exists. The old inline math here diverged from
            // `transform_solid` — it never scaled cylinder/sphere/torus radii
            // and had no anisotropic-scale handling at all, so "equivalent to
            // copy_solid followed by transform_solid" was untrue for any
            // scaling matrix.
            let source_is_nurbs = matches!(face.surface, FaceSurface::Nurbs(_));
            let new_fid = add_planned_face(
                topo,
                (face.outer, &face.inner),
                face.surface,
                face.reversed,
                &new_wires,
            )?;
            // Vertices and edge curves were written at their transformed
            // positions above, which is exactly the state
            // `transform_face_surface` expects (its non-uniform branches map
            // boundary probes back through the inverse).
            recorder.set_refusal_origin(topo.face_id_from_index(face.old_index));
            crate::transform::transform_face_surface_recorded(
                topo,
                new_fid,
                matrix,
                normal_matrix,
                recorder,
            )?;
            recorder.set_refusal_origin(None);
            if matches!(topo.face(new_fid)?.surface(), FaceSurface::Nurbs(_))
                && (!source_is_nurbs || chart_reversing)
            {
                chart_replaced_faces.push(new_fid);
            }
            if let Some(attributes) = face.attributes {
                topo.set_face_attributes(new_fid, attributes)?;
            }
            new_faces.push((face.old_index, new_fid));
            ranges.push(face.authority);
            new_face_ids.push(new_fid);
        }
        let new_shell = Shell::new(new_face_ids).map_err(crate::OperationsError::Topology)?;
        new_shell_ids.push(topo.add_shell(new_shell));
    }

    restore_planned_authority(
        topo,
        &new_faces,
        &ranges,
        authority,
        &authority_edges,
        &new_edges,
    )?;
    for face in chart_replaced_faces {
        let uses: Vec<_> = topo
            .pcurves_for_face(face)
            .into_iter()
            .map(|(edge, forward, _)| (edge, forward))
            .collect();
        for (edge, forward) in uses {
            topo.remove_pcurve_oriented(edge, face, forward)?;
        }
    }

    add_planned_solid(topo, &new_shell_ids, solid_attributes)
}

/// Create a deep copy of a wire and all its sub-entities.
///
/// Returns a new `WireId` for the copy. The original wire is not modified.
/// All vertices and edges are duplicated.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn copy_wire(topo: &mut Topology, wire_id: WireId) -> Result<WireId, crate::OperationsError> {
    let wire = topo.wire(wire_id)?;
    let closed = wire.is_closed();

    let mut vertex_snaps: Vec<VertexSnap> = Vec::new();
    let mut edge_snaps: Vec<EdgeSnap> = Vec::new();
    let mut edge_refs: Vec<(usize, bool)> = Vec::new();

    let mut seen_vertices = std::collections::HashSet::new();
    let mut seen_edges = std::collections::HashSet::new();

    for oe in wire.edges() {
        let edge_idx = oe.edge().index();
        edge_refs.push((edge_idx, oe.is_forward()));

        if !seen_edges.insert(edge_idx) {
            continue;
        }
        let edge = topo.edge(oe.edge())?;
        let start_idx = edge.start().index();
        let end_idx = edge.end().index();

        for &vid_idx in &[start_idx, end_idx] {
            if seen_vertices.insert(vid_idx) {
                let vid = if vid_idx == start_idx {
                    edge.start()
                } else {
                    edge.end()
                };
                let v = topo.vertex(vid)?;
                vertex_snaps.push(VertexSnap {
                    old_index: vid_idx,
                    point: v.point(),
                    tol: v.tolerance(),
                });
            }
        }

        edge_snaps.push(EdgeSnap {
            old_index: edge_idx,
            start_index: start_idx,
            end_index: end_idx,
            curve: edge.curve().clone(),
            tolerance: edge.tolerance(),
            trim: edge.trim(),
        });
    }

    let mut vertex_map: HashMap<usize, VertexId> = HashMap::new();
    for vsnap in &vertex_snaps {
        let new_vid = topo.add_vertex(Vertex::new(vsnap.point, vsnap.tol));
        vertex_map.insert(vsnap.old_index, new_vid);
    }

    let mut edge_map: HashMap<usize, remus_topology::edge::EdgeId> = HashMap::new();
    for esnap in &edge_snaps {
        let new_start = vertex_map[&esnap.start_index];
        let new_end = vertex_map[&esnap.end_index];
        let copied_edge = topo.add_edge({
            let mut copied =
                Edge::with_tolerance(new_start, new_end, esnap.curve.clone(), esnap.tolerance);
            copied.set_trim(esnap.trim);
            copied
        });
        edge_map.insert(esnap.old_index, copied_edge);
    }

    let new_edges: Vec<OrientedEdge> = edge_refs
        .iter()
        .map(|&(edge_idx, fwd)| OrientedEdge::new(edge_map[&edge_idx], fwd))
        .collect();
    let new_wire = Wire::new(new_edges, closed).map_err(crate::OperationsError::Topology)?;
    Ok(topo.add_wire(new_wire))
}

/// Create a deep copy of a single face and all its sub-entities.
///
/// Returns a new [`FaceId`] for the copy. The original face and any shape that
/// shares its sub-entities are left untouched, so the copy can be translated to
/// form a pocket or boss profile without corrupting the donor solid. The
/// surface carrier and orientation (`reversed`) flag are cloned verbatim — no
/// geometry is recomputed.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn copy_face(topo: &mut Topology, face_id: FaceId) -> Result<FaceId, crate::OperationsError> {
    let coedge_authority = snapshot_face_coedge_authority(topo, face_id)?;
    let face = topo.face(face_id)?;
    let surface = face.surface().clone();
    let reversed = face.is_reversed();
    let outer_wire_index = face.outer_wire().index();
    let inner_wire_indices: Vec<usize> = face.inner_wires().iter().map(|w| w.index()).collect();

    let mut vertex_snaps: Vec<VertexSnap> = Vec::new();
    let mut edge_snaps: Vec<EdgeSnap> = Vec::new();
    let mut wire_snaps: Vec<WireSnap> = Vec::new();

    let mut seen_vertices = std::collections::HashSet::new();
    let mut seen_edges = std::collections::HashSet::new();
    let mut seen_wires = std::collections::HashSet::new();

    for wire_id_val in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
    {
        if !seen_wires.insert(wire_id_val.index()) {
            continue;
        }
        let wire = topo.wire(wire_id_val)?;
        let mut edge_refs = Vec::new();

        for oe in wire.edges() {
            let edge_idx = oe.edge().index();
            edge_refs.push((edge_idx, oe.is_forward()));

            if !seen_edges.insert(edge_idx) {
                continue;
            }
            let edge = topo.edge(oe.edge())?;
            let start_idx = edge.start().index();
            let end_idx = edge.end().index();

            for &vid_idx in &[start_idx, end_idx] {
                if seen_vertices.insert(vid_idx) {
                    let vid = if vid_idx == start_idx {
                        edge.start()
                    } else {
                        edge.end()
                    };
                    let v = topo.vertex(vid)?;
                    vertex_snaps.push(VertexSnap {
                        old_index: vid_idx,
                        point: v.point(),
                        tol: v.tolerance(),
                    });
                }
            }

            edge_snaps.push(EdgeSnap {
                old_index: edge_idx,
                start_index: start_idx,
                end_index: end_idx,
                curve: edge.curve().clone(),
                tolerance: edge.tolerance(),
                trim: edge.trim(),
            });
        }

        wire_snaps.push(WireSnap {
            old_index: wire_id_val.index(),
            edges: edge_refs,
            closed: wire.is_closed(),
        });
    }

    topo.reserve(
        vertex_snaps.len(),
        edge_snaps.len(),
        wire_snaps.len(),
        1,
        0,
        0,
    );

    let mut vertex_map: HashMap<usize, VertexId> = HashMap::new();
    for vsnap in &vertex_snaps {
        let new_vid = topo.add_vertex(Vertex::new(vsnap.point, vsnap.tol));
        vertex_map.insert(vsnap.old_index, new_vid);
    }

    let mut edge_map: HashMap<usize, remus_topology::edge::EdgeId> = HashMap::new();
    for esnap in &edge_snaps {
        let new_start = vertex_map[&esnap.start_index];
        let new_end = vertex_map[&esnap.end_index];
        let copied_edge = topo.add_edge({
            let mut copied =
                Edge::with_tolerance(new_start, new_end, esnap.curve.clone(), esnap.tolerance);
            copied.set_trim(esnap.trim);
            copied
        });
        edge_map.insert(esnap.old_index, copied_edge);
    }

    let mut wire_map: HashMap<usize, WireId> = HashMap::new();
    for wsnap in &wire_snaps {
        let new_edges: Vec<OrientedEdge> = wsnap
            .edges
            .iter()
            .map(|&(edge_idx, fwd)| OrientedEdge::new(edge_map[&edge_idx], fwd))
            .collect();
        let new_wire =
            Wire::new(new_edges, wsnap.closed).map_err(crate::OperationsError::Topology)?;
        wire_map.insert(wsnap.old_index, topo.add_wire(new_wire));
    }

    let new_outer = wire_map[&outer_wire_index];
    let new_inner: Vec<WireId> = inner_wire_indices.iter().map(|idx| wire_map[idx]).collect();
    let new_face = if reversed {
        Face::new_reversed(new_outer, new_inner, surface)
    } else {
        Face::new(new_outer, new_inner, surface)
    };
    let new_face_id = topo.add_face(new_face);
    for snapshot in coedge_authority {
        let edge = remapped_authority_edge(&edge_map, &snapshot)?;
        restore_face_coedge_authority(topo, new_face_id, edge, snapshot)?;
    }
    Ok(new_face_id)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::tolerance::Tolerance;
    use remus_math::{
        curves2d::{Curve2D, Line2D},
        vec::{Point2, Vec2},
    };
    use remus_topology::Topology;
    use remus_topology::pcurve::PCurve;
    use remus_topology::test_utils::make_unit_cube_manifold;

    use super::*;

    mod legacy_oracle;

    fn assert_single_lifted_pcurve(topo: &Topology, face: FaceId) {
        let pcurves = topo.pcurves_for_face(face);
        assert_eq!(pcurves.len(), 1, "copied face must retain one pcurve");
        let (edge, forward, pcurve) = &pcurves[0];
        let coedge = face_coedges(topo, face)
            .unwrap()
            .into_iter()
            .find(|&coedge_id| {
                topo.coedge(coedge_id)
                    .is_ok_and(|coedge| coedge.edge() == *edge && coedge.is_forward() == *forward)
            })
            .unwrap();
        assert_eq!(
            topo.coedge(coedge).unwrap().periodic_winding(),
            PeriodicWinding::new(3, -2)
        );
        assert_eq!(pcurve.t_start().to_bits(), 4.0_f64.to_bits());
        assert_eq!(pcurve.t_end().to_bits(), 5.0_f64.to_bits());
        assert!((pcurve.evaluate(4.5) - Point2::new(6.5, 3.0)).length() < 1e-14);
    }

    #[test]
    fn copy_creates_new_solid() {
        let mut topo = Topology::new();
        let orig = make_unit_cube_manifold(&mut topo);
        let copy = copy_solid(&mut topo, orig).unwrap();
        assert_ne!(orig.index(), copy.index());
    }

    #[test]
    fn copy_preserves_face_count() {
        let mut topo = Topology::new();
        let orig = make_unit_cube_manifold(&mut topo);
        let copy = copy_solid(&mut topo, orig).unwrap();

        let orig_faces = topo
            .shell(topo.solid(orig).unwrap().outer_shell())
            .unwrap()
            .faces()
            .len();
        let copy_faces = topo
            .shell(topo.solid(copy).unwrap().outer_shell())
            .unwrap()
            .faces()
            .len();

        assert_eq!(orig_faces, copy_faces);
    }

    #[test]
    fn copy_between_topologies_carries_attributes() {
        let mut source = Topology::new();
        let solid = make_unit_cube_manifold(&mut source);
        let face = remus_topology::explorer::solid_faces(&source, solid).unwrap()[0];
        source
            .set_solid_attributes(
                solid,
                remus_topology::attributes::EntityAttributes {
                    name: Some("source solid".to_owned()),
                    ..Default::default()
                },
            )
            .unwrap();
        source
            .set_face_attributes(
                face,
                remus_topology::attributes::EntityAttributes {
                    name: Some("source face".to_owned()),
                    ..Default::default()
                },
            )
            .unwrap();

        let mut destination = Topology::new();
        let copied = copy_solid_between(&source, &mut destination, solid).unwrap();

        assert_eq!(
            destination
                .attributes()
                .solid(copied)
                .and_then(|attributes| attributes.name.as_deref()),
            Some("source solid")
        );
        let face_names: Vec<_> = remus_topology::explorer::solid_faces(&destination, copied)
            .unwrap()
            .into_iter()
            .filter_map(|copied_face| destination.attributes().face(copied_face))
            .filter_map(|attributes| attributes.name.as_deref())
            .collect();
        assert_eq!(face_names, vec!["source face"]);
    }

    #[test]
    fn copy_between_topologies_carries_oriented_coedge_authority() {
        let mut source = Topology::new();
        let solid = make_unit_cube_manifold(&mut source);
        let face = remus_topology::explorer::solid_faces(&source, solid).unwrap()[0];
        let oriented = source
            .wire(source.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()[0];
        let pcurve = PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(2.0, 3.0), Vec2::new(1.0, 0.0)).unwrap()),
            4.0,
            5.0,
        );
        source
            .set_pcurve_oriented(oriented.edge(), face, oriented.is_forward(), pcurve)
            .unwrap();
        let source_coedge = face_coedges(&source, face)
            .unwrap()
            .into_iter()
            .find(|&coedge_id| {
                source.coedge(coedge_id).is_ok_and(|coedge| {
                    coedge.edge() == oriented.edge() && coedge.is_forward() == oriented.is_forward()
                })
            })
            .unwrap();
        source
            .set_coedge_periodic_winding(source_coedge, PeriodicWinding::new(3, -2))
            .unwrap();

        let mut destination = Topology::new();
        let copied = copy_solid_between(&source, &mut destination, solid).unwrap();
        let copied_face = remus_topology::explorer::solid_faces(&destination, copied).unwrap()[0];
        let copied_use = destination
            .wire(destination.face(copied_face).unwrap().outer_wire())
            .unwrap()
            .edges()[0];
        let copied_pcurve = destination
            .pcurve_oriented(copied_use.edge(), copied_face, copied_use.is_forward())
            .unwrap();
        let copied_coedge = face_coedges(&destination, copied_face)
            .unwrap()
            .into_iter()
            .find(|&coedge_id| {
                destination.coedge(coedge_id).is_ok_and(|coedge| {
                    coedge.edge() == copied_use.edge()
                        && coedge.is_forward() == copied_use.is_forward()
                })
            })
            .unwrap();

        assert_eq!(destination.num_pcurves(), 1);
        assert_eq!(
            destination
                .coedge(copied_coedge)
                .unwrap()
                .periodic_winding(),
            PeriodicWinding::new(3, -2)
        );
        assert_eq!(copied_pcurve.t_start().to_bits(), 4.0_f64.to_bits());
        assert_eq!(copied_pcurve.t_end().to_bits(), 5.0_f64.to_bits());
        assert!((copied_pcurve.evaluate(4.5) - Point2::new(6.5, 3.0)).length() < 1e-14);

        let copied_face_only = copy_face(&mut source, face).unwrap();
        assert_single_lifted_pcurve(&source, copied_face_only);

        let (_, face_map) = copy_solid_with_face_map(&mut source, solid).unwrap();
        let copied_same_face = source.face_id_from_index(face_map[&face.index()]).unwrap();
        assert_single_lifted_pcurve(&source, copied_same_face);

        let transformed = copy_and_transform_solid(
            &mut source,
            solid,
            &remus_math::mat::Mat4::translation(10.0, 0.0, 0.0),
        )
        .unwrap();
        let transformed_face = remus_topology::explorer::solid_faces(&source, transformed)
            .unwrap()
            .into_iter()
            .find(|&candidate| !source.pcurves_for_face(candidate).is_empty())
            .unwrap();
        assert_single_lifted_pcurve(&source, transformed_face);
    }

    #[test]
    fn copy_preserves_volume() {
        let mut topo = Topology::new();
        let orig = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let copy = copy_solid(&mut topo, orig).unwrap();

        let vol_orig = crate::measure::solid_volume(&topo, orig, 0.1).unwrap();
        let vol_copy = crate::measure::solid_volume(&topo, copy, 0.1).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(vol_orig, vol_copy),
            "copy should preserve volume: {vol_orig} vs {vol_copy}"
        );
    }

    #[test]
    fn copy_is_independent() {
        use remus_math::mat::Mat4;

        let mut topo = Topology::new();
        let orig = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let copy = copy_solid(&mut topo, orig).unwrap();

        // Transform the copy; original should be unchanged.
        crate::transform::transform_solid(&mut topo, copy, &Mat4::translation(10.0, 0.0, 0.0))
            .unwrap();

        let bbox_orig = crate::measure::solid_bounding_box(&topo, orig).unwrap();
        let bbox_copy = crate::measure::solid_bounding_box(&topo, copy).unwrap();

        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(bbox_orig.min.x(), 0.0),
            "original should be unchanged, min_x = {}",
            bbox_orig.min.x()
        );
        assert!(
            tol.approx_eq(bbox_copy.min.x(), 10.0),
            "copy should be shifted, min_x = {}",
            bbox_copy.min.x()
        );
    }

    #[test]
    fn copy_wire_creates_new_wire() {
        use remus_math::vec::Point3;
        use remus_topology::builder::make_polygon_wire;

        let mut topo = Topology::new();
        let orig = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let copy = copy_wire(&mut topo, orig).unwrap();

        assert_ne!(orig.index(), copy.index());
    }

    #[test]
    fn copy_wire_preserves_edge_count() {
        use remus_math::vec::Point3;
        use remus_topology::builder::make_polygon_wire;

        let mut topo = Topology::new();
        let orig = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let copy = copy_wire(&mut topo, orig).unwrap();

        let orig_edges = topo.wire(orig).unwrap().edges().len();
        let copy_edges = topo.wire(copy).unwrap().edges().len();
        assert_eq!(orig_edges, copy_edges);
    }

    #[test]
    fn copy_wire_is_independent() {
        use remus_math::mat::Mat4;
        use remus_math::vec::Point3;
        use remus_topology::builder::make_polygon_wire;

        let mut topo = Topology::new();
        let orig = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let copy = copy_wire(&mut topo, orig).unwrap();

        // Transform the copy; original should be unchanged.
        crate::transform::transform_wire(&mut topo, copy, &Mat4::translation(10.0, 0.0, 0.0))
            .unwrap();

        // Check that original vertex positions are unchanged.
        let tol = Tolerance::new();
        let orig_wire = topo.wire(orig).unwrap();
        let first_edge = orig_wire.edges().first().unwrap();
        let start = topo.edge(first_edge.edge()).unwrap().start();
        let pos = topo.vertex(start).unwrap().point();
        assert!(
            tol.approx_eq(pos.x(), 0.0),
            "original wire should be unchanged, x = {}",
            pos.x()
        );
    }

    #[test]
    fn copy_wire_with_circle_edge() {
        use remus_math::curves::Circle3D;
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        let mut topo = Topology::new();

        // Create a closed circular wire (single circle edge, start == end).
        let v = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let edge = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(circle)));
        let wire = Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap();
        let wid = topo.add_wire(wire);

        let copy_wid = copy_wire(&mut topo, wid).unwrap();
        assert_ne!(wid.index(), copy_wid.index());

        // Verify the copied wire has a circle edge.
        let copy_wire = topo.wire(copy_wid).unwrap();
        let copy_edge = topo.edge(copy_wire.edges()[0].edge()).unwrap();
        assert!(
            matches!(copy_edge.curve(), EdgeCurve::Circle(_)),
            "copied edge should be a Circle"
        );
    }

    fn make_plane_quad(topo: &mut Topology) -> remus_topology::face::FaceId {
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::builder::make_polygon_wire;
        use remus_topology::face::{Face, FaceSurface};

        let outer = make_polygon_wire(
            topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
                Point3::new(4.0, 4.0, 0.0),
                Point3::new(0.0, 4.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        topo.add_face(Face::new(outer, Vec::new(), surface))
    }

    fn distinct_vertex_count(topo: &Topology, face_id: remus_topology::face::FaceId) -> usize {
        let face = topo.face(face_id).unwrap();
        let mut seen = std::collections::HashSet::new();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                seen.insert(edge.start().index());
                seen.insert(edge.end().index());
            }
        }
        seen.len()
    }

    fn total_edge_count(topo: &Topology, face_id: remus_topology::face::FaceId) -> usize {
        let face = topo.face(face_id).unwrap();
        let mut seen = std::collections::HashSet::new();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                seen.insert(oe.edge().index());
            }
        }
        seen.len()
    }

    fn loop_count(topo: &Topology, face_id: remus_topology::face::FaceId) -> usize {
        let face = topo.face(face_id).unwrap();
        1 + face.inner_wires().len()
    }

    #[test]
    fn copy_face_creates_new_face() {
        let mut topo = Topology::new();
        let orig = make_plane_quad(&mut topo);
        let copy = copy_face(&mut topo, orig).unwrap();
        assert_ne!(orig.index(), copy.index());
    }

    #[test]
    fn copy_face_preserves_topology_counts() {
        use remus_topology::explorer::solid_faces;

        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let box_face = *solid_faces(&topo, solid).unwrap().first().unwrap();
        let copy = copy_face(&mut topo, box_face).unwrap();

        assert_eq!(loop_count(&topo, box_face), loop_count(&topo, copy));
        assert_eq!(
            total_edge_count(&topo, box_face),
            total_edge_count(&topo, copy)
        );
        assert_eq!(
            distinct_vertex_count(&topo, box_face),
            distinct_vertex_count(&topo, copy)
        );

        // Face with one inner loop (hole).
        let holed = make_holed_face(&mut topo);
        let holed_copy = copy_face(&mut topo, holed).unwrap();
        assert_eq!(loop_count(&topo, holed), 2);
        assert_eq!(loop_count(&topo, holed_copy), 2);
        assert_eq!(
            total_edge_count(&topo, holed),
            total_edge_count(&topo, holed_copy)
        );
        assert_eq!(
            distinct_vertex_count(&topo, holed),
            distinct_vertex_count(&topo, holed_copy)
        );
    }

    fn make_holed_face(topo: &mut Topology) -> remus_topology::face::FaceId {
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::builder::make_polygon_wire;
        use remus_topology::face::{Face, FaceSurface};

        let outer = make_polygon_wire(
            topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(10.0, 10.0, 0.0),
                Point3::new(0.0, 10.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let inner = make_polygon_wire(
            topo,
            &[
                Point3::new(3.0, 3.0, 0.0),
                Point3::new(7.0, 3.0, 0.0),
                Point3::new(7.0, 7.0, 0.0),
                Point3::new(3.0, 7.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        topo.add_face(Face::new(outer, vec![inner], surface))
    }

    #[test]
    fn copy_face_is_independent() {
        use remus_math::mat::Mat4;

        let mut topo = Topology::new();
        let orig = make_plane_quad(&mut topo);

        let orig_first_vertex = {
            let face = topo.face(orig).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            topo.edge(wire.edges()[0].edge()).unwrap().start()
        };
        let orig_start_x = topo.vertex(orig_first_vertex).unwrap().point().x();

        let copy = copy_face(&mut topo, orig).unwrap();
        crate::transform::transform_face(&mut topo, copy, &Mat4::translation(10.0, 0.0, 0.0))
            .unwrap();

        let tol = Tolerance::new();
        let orig_x_after = topo.vertex(orig_first_vertex).unwrap().point().x();
        assert!(
            tol.approx_eq(orig_x_after, orig_start_x),
            "original face vertex should be unchanged, x = {orig_x_after}"
        );

        let copy_first_vertex = {
            let face = topo.face(copy).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            topo.edge(wire.edges()[0].edge()).unwrap().start()
        };
        let copy_x = topo.vertex(copy_first_vertex).unwrap().point().x();
        assert!(
            tol.approx_eq(copy_x, orig_start_x + 10.0),
            "copy face vertex should be shifted by 10, x = {copy_x}"
        );
    }

    #[test]
    fn copy_face_preserves_orientation() {
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::builder::make_polygon_wire;
        use remus_topology::face::{Face, FaceSurface};

        let mut topo = Topology::new();
        let outer = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let orig = topo.add_face(Face::new_reversed(outer, Vec::new(), surface));

        let copy = copy_face(&mut topo, orig).unwrap();
        assert!(
            topo.face(copy).unwrap().is_reversed(),
            "copied face should preserve the reversed orientation flag"
        );
    }

    #[test]
    fn copy_face_with_circle_edge() {
        use remus_math::curves::Circle3D;
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::{Face, FaceSurface};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        let mut topo = Topology::new();
        let v = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let edge = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(circle)));
        let wire = Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap();
        let wid = topo.add_wire(wire);
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let orig = topo.add_face(Face::new(wid, Vec::new(), surface));

        let copy = copy_face(&mut topo, orig).unwrap();
        assert_eq!(distinct_vertex_count(&topo, copy), 1);

        let copy_face_data = topo.face(copy).unwrap();
        let copy_wire = topo.wire(copy_face_data.outer_wire()).unwrap();
        let copy_edge = topo.edge(copy_wire.edges()[0].edge()).unwrap();
        assert!(
            matches!(copy_edge.curve(), EdgeCurve::Circle(_)),
            "copied edge should be a Circle"
        );
    }
}
