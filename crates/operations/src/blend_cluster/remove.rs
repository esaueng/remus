//! Exact removal of a recognized blend group back to its sharp parent.
//!
//! The second stage of the blend-aware direct edit: once
//! [`recognize_blend_cluster`] has proven a dependent group, the group (or a
//! dependency-closed part of it) is deleted and the sharp body behind it is
//! rebuilt exactly. Nothing is pierced, extended or sampled:
//!
//! - **Member faces go.** Bands and sampled corner patches are deleted
//!   whole; a corner patch is never trimmed or extended.
//! - **Supports keep their carriers.** Every retained face keeps its exact
//!   surface. Only plane and cylinder supports are qualified; anything else is
//!   a typed refusal.
//! - **Sharp edges are closed form.** Each removed band contributes the exact
//!   section of its two root supports (a plane/plane or plane/cylinder line,
//!   or a plane/cylinder circle) between the predicted sharp vertices, which
//!   are three-face intersections. A closed sharp circle (a rim) is one
//!   closed edge, split only where a retained edge (a cylinder seam) lands on
//!   it.
//! - **Retained wires are re-cut, not re-derived.** Each retained face's new
//!   loop is its old loop in the old order: an edge between two retained
//!   faces is kept on its own carrier with the endpoints that touched the
//!   group moved to their sharp image; a run of edges into the group is
//!   replaced by the sharp edges of the bands the face supports, in run order;
//!   every other group edge (corner contacts, band cross arcs on end caps) is
//!   dropped. This is what answers which edges of a boundary face (an end
//!   cap) are re-trimmed: exactly the retained edges with a moved endpoint,
//!   reported in [`SharpRestoration::retrimmed_edges`].
//!
//! A moved endpoint's sharp image must be unique: the predicted vertices of
//! the members at that point, lying on every retained face there and on
//! every retained edge's carrier. Zero or several candidates, a retained edge
//! that would reverse or collapse, or a loop that does not close is an
//! [`AmbiguousSharp`](BlendClusterError::AmbiguousSharp) refusal.
//!
//! The rebuild runs on a private copy of the solid. The result is validated
//! (strict check-crate validation and a positive volume) before it is copied
//! into the caller's arena, so every refusal leaves the caller's topology
//! untouched, handle slots included.
//!
//! WASM module tier: `core` once bound (it edits the live session); no binding
//! yet.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::EntityKey;
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use super::{BlendCluster, BlendClusterError, MemberSharp, SharpCarrier, recognize_blend_cluster};
use crate::OperationsError;
use crate::copy::{CopiedSolidEntities, copy_solid_between_with_entity_map};
use crate::evolution::EvolutionMap;

/// Distance (relative to `max(1, |p|)`) within which a point lies on a
/// closed-form carrier.
const ON_CARRIER: f64 = 1e-7;
/// Smallest `|cos|` between a contact edge and its sharp edge accepted as an
/// orientation decision.
const ORIENTATION_COSINE: f64 = 0.5;

/// The sharp body a blend removal produced, with the construction history a
/// downstream re-fillet needs.
#[derive(Debug, Clone)]
pub struct SharpRestoration {
    /// The rebuilt sharp solid.
    pub solid: SolidId,
    /// Member indices (into [`BlendCluster::members`]) that were removed,
    /// ascending.
    pub removed: Vec<usize>,
    /// Every retained source face and the result face it became, in source
    /// shell order. Each keeps its exact surface and orientation.
    pub faces: Vec<(FaceId, FaceId)>,
    /// Result edges along each predicted sharp edge (indexed like
    /// [`BlendCluster::sharp_edges`]), in carrier order. Empty for the sharp
    /// edges of members that were not removed. A closed circle is one edge
    /// unless retained edges land on it.
    pub sharp_edges: Vec<Vec<EdgeId>>,
    /// Result vertex of each predicted sharp vertex (indexed like
    /// [`BlendCluster::sharp_vertices`]); `None` when no removed member
    /// restores it.
    pub sharp_vertices: Vec<Option<VertexId>>,
    /// Retained source edges whose endpoints moved to their sharp image (the
    /// re-trimmed wires of supports and boundary faces), with their result
    /// edges, in source arena order.
    pub retrimmed_edges: Vec<(EdgeId, EdgeId)>,
    /// Construction-derived face evolution: retained faces `modified`,
    /// removed member faces `deleted`.
    pub evolution: EvolutionMap,
    /// Construction-derived edge and vertex history over the source solid. A
    /// `None` target means the source entity was consumed with the group.
    /// Sorted by source.
    pub boundary_history: Vec<(EntityKey, Option<EntityKey>)>,
}

impl SharpRestoration {
    /// Result edges a re-fillet stage rounds: the sharp edges of
    /// `cluster.reblend_order[step]` that this removal restored.
    #[must_use]
    pub fn reblend_edges(&self, cluster: &BlendCluster, step: usize) -> Vec<EdgeId> {
        cluster
            .reblend_order
            .get(step)
            .map_or_else(Vec::new, |step| {
                step.sharp_edges
                    .iter()
                    .filter_map(|&index| self.sharp_edges.get(index))
                    .flatten()
                    .copied()
                    .collect()
            })
    }
}

/// Remove every member of `cluster` from `solid`, restoring its sharp parent.
///
/// # Errors
///
/// See [`remove_blend_members`].
pub fn remove_blend_cluster(
    topo: &mut Topology,
    solid: SolidId,
    cluster: &BlendCluster,
) -> Result<SharpRestoration, OperationsError> {
    let all: Vec<usize> = (0..cluster.members.len()).collect();
    remove_blend_members(topo, solid, cluster, &all)
}

/// Remove the members `members` (indices into [`BlendCluster::members`]) of
/// `cluster` from `solid`, restoring their sharp edges and vertices exactly.
///
/// The cluster is re-recognized from its seed first: a cluster that no longer
/// describes the solid is refused. The selection must be dependency closed
/// (no retained member may share an edge or vertex with a removed one);
/// otherwise the refusal names the closure, so a caller can remove that
/// instead. The input solid is never modified; the result is a new solid.
///
/// # Errors
///
/// [`BlendClusterError::InvalidRemoval`] for an empty or out-of-range
/// selection, a stale cluster, or a solid with cavity shells;
/// [`BlendClusterError::DependentMembers`] when the selection would leave a
/// retained blend hanging; [`BlendClusterError::UnsupportedSupport`] when a
/// removed member rolls on anything but a plane or cylinder;
/// [`BlendClusterError::AmbiguousSharp`] when the sharp parent is not
/// uniquely and exactly representable; [`BlendClusterError::SharpRebuildFailed`]
/// when the rebuilt body fails validation. Recognition refusals and topology
/// errors propagate. On every error the caller's arena is unchanged.
pub fn remove_blend_members(
    topo: &mut Topology,
    solid: SolidId,
    cluster: &BlendCluster,
    members: &[usize],
) -> Result<SharpRestoration, OperationsError> {
    let removed = plan(topo, solid, cluster, members)?;
    let source_faces = remus_topology::explorer::solid_faces(topo, solid)?;
    let mut work = Topology::new();
    let copied = copy_solid_between_with_entity_map(topo, &mut work, solid)?;
    let built = Rebuild::new(&work, &copied, &source_faces, cluster, &removed)?.run(&mut work)?;
    postconditions(&work, built.solid)?;
    let output = remus_topology::transaction::run_transacted(topo, |topo| {
        copy_solid_between_with_entity_map(&work, topo, built.solid)
    })?;
    history(topo, solid, cluster, &removed, &copied, &built, &output)
}

fn invalid(reason: impl Into<String>) -> OperationsError {
    BlendClusterError::InvalidRemoval {
        reason: reason.into(),
    }
    .into()
}

fn ambiguous(reason: impl Into<String>) -> OperationsError {
    BlendClusterError::AmbiguousSharp {
        reason: reason.into(),
    }
    .into()
}

fn rebuild_failed(reason: impl Into<String>) -> OperationsError {
    BlendClusterError::SharpRebuildFailed {
        reason: reason.into(),
    }
    .into()
}

fn tolerance_at(point: Point3) -> f64 {
    ON_CARRIER
        * point
            .x()
            .abs()
            .max(point.y().abs())
            .max(point.z().abs())
            .max(1.0)
}

// ---------------------------------------------------------------------------
// Planning: read-only checks on the caller's solid.

/// Validate the request and return the removed member indices, ascending.
fn plan(
    topo: &Topology,
    solid: SolidId,
    cluster: &BlendCluster,
    members: &[usize],
) -> Result<Vec<usize>, OperationsError> {
    if members.is_empty() {
        return Err(invalid("no blend members selected"));
    }
    if let Some(&bad) = members.iter().find(|&&m| m >= cluster.members.len()) {
        return Err(invalid(format!(
            "member {bad} is outside the cluster's {} members",
            cluster.members.len()
        )));
    }
    let requested: BTreeSet<usize> = members.iter().copied().collect();
    let solid_data = topo.solid(solid)?;
    if !solid_data.inner_shells().is_empty() {
        return Err(invalid(format!(
            "solid has {} cavity shell(s); only single-shell solids are qualified",
            solid_data.inner_shells().len()
        )));
    }
    let faces: BTreeSet<FaceId> = remus_topology::explorer::solid_faces(topo, solid)?
        .into_iter()
        .collect();
    let named = std::iter::once(cluster.seed)
        .chain(cluster.supports.iter().copied())
        .chain(cluster.boundary_faces.iter().copied())
        .chain(cluster.members.iter().flat_map(|member| {
            member
                .faces
                .iter()
                .chain(&member.supports)
                .chain(&member.root_supports)
                .copied()
        }));
    for face in named {
        if !faces.contains(&face) {
            return Err(invalid(format!(
                "the cluster names face {}, which is not part of the solid",
                face.index()
            )));
        }
    }
    check_supports(topo, cluster, &requested)?;
    // The cluster is evidence about one solid state; re-measure it rather
    // than trusting a caller-held copy.
    let fresh = recognize_blend_cluster(topo, solid, cluster.seed)?;
    if fresh != *cluster {
        return Err(invalid(
            "the cluster does not describe the solid's current blends; recognize it again",
        ));
    }

    // Dependency closure over shared edges and vertices.
    let mut face_member: BTreeMap<FaceId, usize> = BTreeMap::new();
    for (index, member) in cluster.members.iter().enumerate() {
        for &face in &member.faces {
            face_member.insert(face, index);
        }
    }
    let mut vertex_faces: BTreeMap<VertexId, BTreeSet<FaceId>> = BTreeMap::new();
    for face in remus_topology::explorer::solid_faces(topo, solid)? {
        for vertex in remus_topology::explorer::face_vertices(topo, face)? {
            vertex_faces.entry(vertex).or_default().insert(face);
        }
    }
    let mut touching: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
    for faces in vertex_faces.values() {
        let at: BTreeSet<usize> = faces
            .iter()
            .filter_map(|face| face_member.get(face).copied())
            .collect();
        for &a in &at {
            touching.entry(a).or_default().extend(at.iter().copied());
        }
    }
    let mut closure = requested.clone();
    let mut queue: Vec<usize> = requested.iter().copied().collect();
    while let Some(member) = queue.pop() {
        for &other in touching.get(&member).into_iter().flatten() {
            if closure.insert(other) {
                queue.push(other);
            }
        }
    }
    if closure != requested {
        return Err(BlendClusterError::DependentMembers {
            requested: requested.into_iter().collect(),
            required: closure.into_iter().collect(),
        }
        .into());
    }
    Ok(requested.into_iter().collect())
}

/// Only plane and cylinder supports extend exactly.
fn check_supports(
    topo: &Topology,
    cluster: &BlendCluster,
    requested: &BTreeSet<usize>,
) -> Result<(), OperationsError> {
    for &index in requested {
        for &support in &cluster.members[index].root_supports {
            let surface = topo.face(support)?.surface();
            if !matches!(
                surface,
                FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
            ) {
                return Err(BlendClusterError::UnsupportedSupport {
                    face: support.index(),
                    surface: surface.type_tag(),
                }
                .into());
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Rebuild on the private copy.

/// Where a retained edge's moved endpoint lands.
#[derive(Clone, Copy, Debug)]
enum Image {
    /// A predicted sharp vertex.
    Sharp(usize),
    /// A point on the closed sharp edge `edge`.
    OnClosed { edge: usize, point: Point3 },
}

/// One entry of a rebuilt loop before orientation.
#[derive(Clone, Copy, Debug)]
enum Item {
    /// A retained edge, already re-cut, in its old orientation.
    Kept(OrientedEdge),
    /// The sharp edge of a removed band, with the first contact edge of the
    /// run that produced it (for its orientation).
    Sharp { edge: usize, contact: OrientedEdge },
}

struct Built {
    solid: SolidId,
    /// Work face (copied source) -> same work face in the result.
    faces: Vec<FaceId>,
    /// Work edge -> its re-cut replacement.
    edge_image: BTreeMap<EdgeId, EdgeId>,
    /// Work vertex -> its sharp image.
    vertex_image: BTreeMap<VertexId, VertexId>,
    sharp_edges: Vec<Vec<EdgeId>>,
    sharp_vertices: Vec<Option<VertexId>>,
}

struct Rebuild<'a> {
    cluster: &'a BlendCluster,
    removed: &'a [usize],
    /// Source face -> work face.
    source_to_work: BTreeMap<FaceId, FaceId>,
    /// Work face -> source face.
    work_to_source: BTreeMap<FaceId, FaceId>,
    /// Work shell faces in order.
    shell: Vec<FaceId>,
    /// Work faces of removed members -> member index.
    removed_face: BTreeMap<FaceId, usize>,
    /// Work edge -> the work faces using it (a seam lists its face twice).
    edge_faces: BTreeMap<EdgeId, Vec<FaceId>>,
    /// Work vertex -> work faces through it.
    vertex_faces: BTreeMap<VertexId, BTreeSet<FaceId>>,
}

impl<'a> Rebuild<'a> {
    fn new(
        work: &Topology,
        copied: &CopiedSolidEntities,
        source_faces: &[FaceId],
        cluster: &'a BlendCluster,
        removed: &'a [usize],
    ) -> Result<Self, OperationsError> {
        let mut source_to_work = BTreeMap::new();
        let mut work_to_source = BTreeMap::new();
        for &source in source_faces {
            let target = copied
                .face_map
                .get(&source.index())
                .copied()
                .ok_or_else(|| rebuild_failed("the private copy lost a face"))?;
            source_to_work.insert(source, target);
            work_to_source.insert(target, source);
        }
        let map = |face: FaceId| {
            source_to_work.get(&face).copied().ok_or_else(|| {
                invalid(format!("cluster face {} is not in the solid", face.index()))
            })
        };
        let mut removed_face = BTreeMap::new();
        for &member in removed {
            for &face in &cluster.members[member].faces {
                removed_face.insert(map(face)?, member);
            }
        }
        let shell = remus_topology::explorer::solid_faces(work, copied.solid)?;
        let mut edge_faces: BTreeMap<EdgeId, Vec<FaceId>> = BTreeMap::new();
        let mut vertex_faces: BTreeMap<VertexId, BTreeSet<FaceId>> = BTreeMap::new();
        for &face in &shell {
            for oe in work.face_oriented_edges(face)? {
                edge_faces.entry(oe.edge()).or_default().push(face);
                let edge = work.edge(oe.edge())?;
                for vertex in [edge.start(), edge.end()] {
                    vertex_faces.entry(vertex).or_default().insert(face);
                }
            }
        }
        Ok(Self {
            cluster,
            removed,
            source_to_work,
            work_to_source,
            shell,
            removed_face,
            edge_faces,
            vertex_faces,
        })
    }

    fn is_removed(&self, face: FaceId) -> bool {
        self.removed_face.contains_key(&face)
    }

    /// Whether every face using `edge` is retained.
    fn is_kept(&self, edge: EdgeId) -> bool {
        self.edge_faces
            .get(&edge)
            .is_some_and(|faces| faces.iter().all(|face| !self.is_removed(*face)))
    }

    fn touches_group(&self, vertex: VertexId) -> bool {
        self.vertex_faces
            .get(&vertex)
            .is_some_and(|faces| faces.iter().any(|face| self.is_removed(*face)))
    }

    fn run(&self, work: &mut Topology) -> Result<Built, OperationsError> {
        let tol = Tolerance::new().linear;

        // Retained edges and the endpoints of theirs that touch the group.
        let kept: Vec<EdgeId> = self
            .edge_faces
            .keys()
            .copied()
            .filter(|edge| self.is_kept(*edge))
            .collect();
        let mut moved: BTreeSet<VertexId> = BTreeSet::new();
        for &edge in &kept {
            let data = work.edge(edge)?;
            for vertex in [data.start(), data.end()] {
                if self.touches_group(vertex) {
                    moved.insert(vertex);
                }
            }
        }
        let mut images: BTreeMap<VertexId, Image> = BTreeMap::new();
        for &vertex in &moved {
            images.insert(vertex, self.image_of(work, vertex, &kept)?);
        }

        // Sharp vertices: every end of a removed band's open sharp edge and
        // every image above.
        let mut sharp_vertices: Vec<Option<VertexId>> =
            vec![None; self.cluster.sharp_vertices.len()];
        let mut need_vertex = |index: usize, work: &mut Topology| -> VertexId {
            *sharp_vertices[index].get_or_insert_with(|| {
                work.add_vertex(Vertex::new(self.cluster.sharp_vertices[index].point, tol))
            })
        };
        let mut vertex_image: BTreeMap<VertexId, VertexId> = BTreeMap::new();
        // Points retained edges land on along each closed sharp edge.
        let mut closed_points: BTreeMap<usize, Vec<(Point3, VertexId)>> = BTreeMap::new();
        for (&vertex, image) in &images {
            let target = match *image {
                Image::Sharp(index) => need_vertex(index, work),
                Image::OnClosed { edge, point } => {
                    let points = closed_points.entry(edge).or_default();
                    if let Some((_, existing)) = points
                        .iter()
                        .find(|(other, _)| (*other - point).length() <= tolerance_at(point))
                    {
                        *existing
                    } else {
                        let created = work.add_vertex(Vertex::new(point, tol));
                        points.push((point, created));
                        created
                    }
                }
            };
            vertex_image.insert(vertex, target);
        }

        // Sharp edges of the removed bands.
        let mut sharp_edges: Vec<Vec<EdgeId>> = vec![Vec::new(); self.cluster.sharp_edges.len()];
        for &member in self.removed {
            let MemberSharp::Edge(index) = self.cluster.members[member].sharp else {
                continue;
            };
            let sharp = &self.cluster.sharp_edges[index];
            sharp_edges[index] = match (sharp.ends, sharp.carrier) {
                (Some([a, b]), SharpCarrier::Line { .. }) => {
                    let (va, vb) = (need_vertex(a, work), need_vertex(b, work));
                    if va == vb {
                        return Err(ambiguous(format!("sharp edge {index} has coincident ends")));
                    }
                    vec![work.add_edge(Edge::new(va, vb, EdgeCurve::Line))]
                }
                (
                    Some([a, b]),
                    SharpCarrier::Circle {
                        center,
                        axis,
                        radius,
                    },
                ) => {
                    let circle =
                        Circle3D::new(center, axis, radius).map_err(OperationsError::Math)?;
                    let (pa, pb) = (
                        self.cluster.sharp_vertices[a].point,
                        self.cluster.sharp_vertices[b].point,
                    );
                    let (va, vb) = (need_vertex(a, work), need_vertex(b, work));
                    // The arc through the predicted midpoint, counter-clockwise
                    // about the axis.
                    let (start, end, from, to) = if on_ccw_arc(&circle, pa, pb, sharp.midpoint) {
                        (va, vb, pa, pb)
                    } else {
                        (vb, va, pb, pa)
                    };
                    let t0 = circle.project(from);
                    let span = (circle.project(to) - t0).rem_euclid(TAU);
                    if span <= tol || span >= TAU - tol {
                        return Err(ambiguous(format!("sharp arc {index} has no extent")));
                    }
                    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
                    edge.set_trim(Some((t0, t0 + span)));
                    vec![work.add_edge(edge)]
                }
                (
                    None,
                    SharpCarrier::Circle {
                        center,
                        axis,
                        radius,
                    },
                ) => {
                    let circle =
                        Circle3D::new(center, axis, radius).map_err(OperationsError::Math)?;
                    let mut points = closed_points.get(&index).cloned().unwrap_or_default();
                    if points.is_empty() {
                        // No retained edge lands on the circle: one closed edge
                        // whose vertex sits above the band's first contact
                        // vertex.
                        let anchor = self.closed_anchor(work, member, &circle)?;
                        points.push((anchor, work.add_vertex(Vertex::new(anchor, tol))));
                    }
                    closed_pieces(work, &circle, points)?
                }
                (None, SharpCarrier::Line { .. }) => {
                    return Err(ambiguous(format!("sharp line {index} has no ends")));
                }
            };
        }

        // Re-cut retained edges with a moved endpoint.
        let mut edge_image: BTreeMap<EdgeId, EdgeId> = BTreeMap::new();
        for &edge in &kept {
            let data = work.edge(edge)?.clone();
            let (start, end) = (data.start(), data.end());
            if !vertex_image.contains_key(&start) && !vertex_image.contains_key(&end) {
                continue;
            }
            let new_start = vertex_image.get(&start).copied().unwrap_or(start);
            let new_end = vertex_image.get(&end).copied().unwrap_or(end);
            let replacement = recut(work, edge, &data, new_start, new_end)?;
            edge_image.insert(edge, work.add_edge(replacement));
        }

        // Rebuild the loops of every retained face that touches the group.
        let mut faces = Vec::new();
        for &face in &self.shell {
            if self.is_removed(face) {
                continue;
            }
            faces.push(face);
            let data = work.face(face)?.clone();
            let wires: Vec<_> = std::iter::once(data.outer_wire())
                .chain(data.inner_wires().iter().copied())
                .collect();
            let mut rebuilt = Vec::with_capacity(wires.len());
            let mut changed = false;
            for wire in wires {
                let old: Vec<OrientedEdge> = work.wire(wire)?.edges().to_vec();
                if old
                    .iter()
                    .all(|oe| self.is_kept(oe.edge()) && !edge_image.contains_key(&oe.edge()))
                {
                    rebuilt.push(wire);
                    continue;
                }
                changed = true;
                let edges = self.rebuild_loop(work, face, &old, &edge_image, &sharp_edges)?;
                rebuilt.push(work.add_wire(Wire::new(edges, true)?));
            }
            if changed {
                let outer = rebuilt[0];
                work.set_face_boundary_wires(face, outer, rebuilt[1..].to_vec())?;
            }
        }
        let shell = work.add_shell(Shell::new(faces.clone())?);
        let solid = work.add_solid(Solid::new(shell, Vec::new()));
        Ok(Built {
            solid,
            faces,
            edge_image,
            vertex_image,
            sharp_edges,
            sharp_vertices,
        })
    }

    /// The unique sharp image of a retained-edge endpoint touching the group.
    fn image_of(
        &self,
        work: &Topology,
        vertex: VertexId,
        kept: &[EdgeId],
    ) -> Result<Image, OperationsError> {
        let point = work.vertex(vertex)?.point();
        let faces = self.vertex_faces.get(&vertex).cloned().unwrap_or_default();
        let members: BTreeSet<usize> = faces
            .iter()
            .filter_map(|face| self.removed_face.get(face).copied())
            .collect();
        let retained: BTreeSet<FaceId> = faces
            .iter()
            .filter(|face| !self.is_removed(**face))
            .filter_map(|face| self.work_to_source.get(face).copied())
            .collect();
        let mut carriers = Vec::new();
        for &edge in kept {
            let data = work.edge(edge)?;
            if data.start() == vertex || data.end() == vertex {
                carriers.push(edge);
            }
        }
        let on_carriers = |candidate: Point3| -> Result<bool, OperationsError> {
            for &edge in &carriers {
                if carrier_distance(work, edge, candidate)? > tolerance_at(candidate) {
                    return Ok(false);
                }
            }
            Ok(true)
        };

        let mut candidates = BTreeSet::new();
        for &member in &members {
            match self.cluster.members[member].sharp {
                MemberSharp::Vertex(index) => {
                    candidates.insert(index);
                }
                MemberSharp::Edge(index) => {
                    if let Some(ends) = self.cluster.sharp_edges[index].ends {
                        candidates.extend(ends);
                    }
                }
            }
        }
        let mut found = Vec::new();
        for index in candidates {
            let sharp = &self.cluster.sharp_vertices[index];
            if retained.iter().all(|face| sharp.faces.contains(face)) && on_carriers(sharp.point)? {
                found.push(index);
            }
        }
        match found.as_slice() {
            [index] => return Ok(Image::Sharp(*index)),
            [] => {}
            _ => {
                return Err(ambiguous(format!(
                    "the retained boundary at {point:?} has {} sharp images",
                    found.len()
                )));
            }
        }

        // A closed sharp circle the retained edge runs into (a seam reaching
        // a removed rim).
        let mut closed = Vec::new();
        for &member in &members {
            let MemberSharp::Edge(index) = self.cluster.members[member].sharp else {
                continue;
            };
            let sharp = &self.cluster.sharp_edges[index];
            let SharpCarrier::Circle {
                center,
                axis,
                radius,
            } = sharp.carrier
            else {
                continue;
            };
            if sharp.ends.is_some() || !retained.iter().all(|face| sharp.supports.contains(face)) {
                continue;
            }
            let planar = radial(point - center, axis);
            let length = planar.length();
            if length <= tolerance_at(center) {
                continue;
            }
            let image = center + planar * (radius / length);
            if on_carriers(image)? {
                closed.push((index, image));
            }
        }
        match closed.as_slice() {
            [(edge, point)] => Ok(Image::OnClosed {
                edge: *edge,
                point: *point,
            }),
            [] => Err(ambiguous(format!(
                "the retained boundary at {point:?} has no sharp image"
            ))),
            _ => Err(ambiguous(format!(
                "the retained boundary at {point:?} reaches {} closed sharp edges",
                closed.len()
            ))),
        }
    }

    /// A point of a closed sharp circle no retained edge lands on: above the
    /// smallest contact vertex of its band.
    fn closed_anchor(
        &self,
        work: &Topology,
        member: usize,
        circle: &Circle3D,
    ) -> Result<Point3, OperationsError> {
        let mut best: Option<VertexId> = None;
        for &face in &self.cluster.members[member].faces {
            let face = self.source_to_work[&face];
            for oe in work.face_oriented_edges(face)? {
                let others = &self.edge_faces[&oe.edge()];
                if others.iter().any(|other| !self.is_removed(*other)) {
                    let edge = work.edge(oe.edge())?;
                    for vertex in [edge.start(), edge.end()] {
                        if best.is_none_or(|current| vertex < current) {
                            best = Some(vertex);
                        }
                    }
                }
            }
        }
        let vertex = best.ok_or_else(|| ambiguous("a removed rim has no retained contact"))?;
        let point = work.vertex(vertex)?.point();
        let planar = radial(point - circle.center(), circle.normal());
        let length = planar.length();
        if length <= tolerance_at(circle.center()) {
            return Err(ambiguous("a removed rim's contact lies on its axis"));
        }
        Ok(circle.center() + planar * (circle.radius() / length))
    }

    /// The new loop of retained `face` from its old loop.
    fn rebuild_loop(
        &self,
        work: &Topology,
        face: FaceId,
        old: &[OrientedEdge],
        edge_image: &BTreeMap<EdgeId, EdgeId>,
        sharp_edges: &[Vec<EdgeId>],
    ) -> Result<Vec<OrientedEdge>, OperationsError> {
        let source = self.work_to_source[&face];
        let mut items: Vec<Item> = Vec::new();
        for &oe in old {
            if self.is_kept(oe.edge()) {
                let edge = edge_image
                    .get(&oe.edge())
                    .copied()
                    .unwrap_or_else(|| oe.edge());
                items.push(Item::Kept(OrientedEdge::new(edge, oe.is_forward())));
                continue;
            }
            let Some(member) = self.edge_faces[&oe.edge()]
                .iter()
                .find_map(|other| self.removed_face.get(other).copied())
            else {
                continue;
            };
            let MemberSharp::Edge(index) = self.cluster.members[member].sharp else {
                // A corner contact: the corner collapses to a vertex.
                continue;
            };
            if !self.cluster.sharp_edges[index].supports.contains(&source) {
                // A cross arc on an end cap: the band's sharp edge ends here.
                continue;
            }
            if matches!(items.last(), Some(Item::Sharp { edge, .. }) if *edge == index) {
                continue;
            }
            items.push(Item::Sharp {
                edge: index,
                contact: oe,
            });
        }
        if items.len() > 1
            && let (Some(Item::Sharp { edge: first, .. }), Some(Item::Sharp { edge: last, .. })) =
                (items.first(), items.last())
            && first == last
        {
            items.pop();
        }
        if items.is_empty() {
            return Err(ambiguous(format!(
                "face {} loses a whole boundary loop",
                source.index()
            )));
        }

        // Orient each sharp edge along its contact edge: both run with the
        // face on their left.
        let mut runs: Vec<Vec<OrientedEdge>> = Vec::with_capacity(items.len());
        for item in &items {
            runs.push(match *item {
                Item::Kept(oe) => vec![oe],
                Item::Sharp { edge, contact } => {
                    let pieces = &sharp_edges[edge];
                    if pieces.is_empty() {
                        return Err(ambiguous(format!("sharp edge {edge} was not built")));
                    }
                    if sharp_forward(
                        work,
                        contact,
                        pieces,
                        &self.cluster.sharp_edges[edge].carrier,
                    )? {
                        pieces.iter().map(|e| OrientedEdge::new(*e, true)).collect()
                    } else {
                        pieces
                            .iter()
                            .rev()
                            .map(|e| OrientedEdge::new(*e, false))
                            .collect()
                    }
                }
            });
        }
        // A closed circle split into arcs starts where the loop arrives.
        for index in 0..runs.len() {
            let Item::Sharp { edge, .. } = items[index] else {
                continue;
            };
            if self.cluster.sharp_edges[edge].ends.is_some() || runs[index].len() < 2 {
                continue;
            }
            if runs.len() == 1 {
                continue;
            }
            let previous = &runs[(index + runs.len() - 1) % runs.len()];
            let arrive = oriented_end(work, previous[previous.len() - 1])?;
            let mut run = runs[index].clone();
            if let Some(offset) = run
                .iter()
                .map(|oe| oriented_start(work, *oe))
                .collect::<Result<Vec<_>, _>>()?
                .iter()
                .position(|vertex| *vertex == arrive)
            {
                run.rotate_left(offset);
                runs[index] = run;
            }
        }
        let edges: Vec<OrientedEdge> = runs.into_iter().flatten().collect();
        for (index, &oe) in edges.iter().enumerate() {
            let next = edges[(index + 1) % edges.len()];
            if oriented_end(work, oe)? != oriented_start(work, next)? {
                return Err(ambiguous(format!(
                    "the rebuilt loop of face {} does not close",
                    source.index()
                )));
            }
        }
        Ok(edges)
    }
}

fn radial(vector: Vec3, axis: Vec3) -> Vec3 {
    vector - axis * vector.dot(axis)
}

fn oriented_start(work: &Topology, oe: OrientedEdge) -> Result<VertexId, OperationsError> {
    Ok(oe.oriented_start(work.edge(oe.edge())?))
}

fn oriented_end(work: &Topology, oe: OrientedEdge) -> Result<VertexId, OperationsError> {
    Ok(oe.oriented_end(work.edge(oe.edge())?))
}

/// Whether `point` lies on the counter-clockwise arc from `from` to `to`.
fn on_ccw_arc(circle: &Circle3D, from: Point3, to: Point3, point: Point3) -> bool {
    let t0 = circle.project(from);
    let span = (circle.project(to) - t0).rem_euclid(TAU);
    let at = (circle.project(point) - t0).rem_euclid(TAU);
    at < span
}

/// Pieces of a closed circle split at `points`, counter-clockwise.
fn closed_pieces(
    work: &mut Topology,
    circle: &Circle3D,
    mut points: Vec<(Point3, VertexId)>,
) -> Result<Vec<EdgeId>, OperationsError> {
    points.compact_sort_by(|a, b| circle.project(a.0).total_cmp(&circle.project(b.0)));
    if points.len() == 1 {
        let t0 = circle.project(points[0].0);
        let mut edge = Edge::new(points[0].1, points[0].1, EdgeCurve::Circle(circle.clone()));
        edge.set_trim(Some((t0, t0 + TAU)));
        return Ok(vec![work.add_edge(edge)]);
    }
    let mut pieces = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let (from, start) = points[index];
        let (to, end) = points[(index + 1) % points.len()];
        let t0 = circle.project(from);
        let span = (circle.project(to) - t0).rem_euclid(TAU);
        if span <= Tolerance::new().angular {
            return Err(ambiguous("two retained edges land on one point of a rim"));
        }
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
        edge.set_trim(Some((t0, t0 + span)));
        pieces.push(work.add_edge(edge));
    }
    Ok(pieces)
}

/// Distance from `point` to the closed-form carrier of retained `edge`.
fn carrier_distance(work: &Topology, edge: EdgeId, point: Point3) -> Result<f64, OperationsError> {
    let data = work.edge(edge)?;
    match data.curve() {
        EdgeCurve::Line => {
            let a = work.vertex(data.start())?.point();
            let b = work.vertex(data.end())?.point();
            let direction = (b - a)
                .normalize()
                .map_err(|_| ambiguous(format!("retained edge {} is degenerate", edge.index())))?;
            Ok(radial(point - a, direction).length())
        }
        EdgeCurve::Circle(circle) => {
            let offset = point - circle.center();
            let height = offset.dot(circle.normal());
            let planar = radial(offset, circle.normal()).length();
            Ok(height.abs().max((planar - circle.radius()).abs()))
        }
        other => Err(ambiguous(format!(
            "retained {} edge {} would have to be re-trimmed",
            other.type_tag(),
            edge.index()
        ))),
    }
}

/// Retained `edge` with its endpoints replaced, on its own carrier and in its
/// own direction.
fn recut(
    work: &Topology,
    edge: EdgeId,
    data: &Edge,
    start: VertexId,
    end: VertexId,
) -> Result<Edge, OperationsError> {
    let old_a = work.vertex(data.start())?.point();
    let old_b = work.vertex(data.end())?.point();
    let new_a = work.vertex(start)?.point();
    let new_b = work.vertex(end)?.point();
    for point in [new_a, new_b] {
        if carrier_distance(work, edge, point)? > tolerance_at(point) {
            return Err(ambiguous(format!(
                "retained edge {} cannot reach its sharp image {point:?}",
                edge.index()
            )));
        }
    }
    if start == end {
        return Err(ambiguous(format!(
            "retained edge {} collapses onto one sharp vertex",
            edge.index()
        )));
    }
    match data.curve() {
        EdgeCurve::Line => {
            if (new_b - new_a).dot(old_b - old_a) <= 0.0
                || (new_b - new_a).length() <= tolerance_at(new_a)
            {
                return Err(ambiguous(format!(
                    "retained edge {} would reverse or vanish",
                    edge.index()
                )));
            }
            Ok(Edge::new(start, end, EdgeCurve::Line))
        }
        EdgeCurve::Circle(circle) => {
            if data.is_closed() {
                return Err(ambiguous(format!(
                    "retained closed circle {} touches the group",
                    edge.index()
                )));
            }
            let (t0, t1) = data.domain_with_endpoints(old_a, old_b);
            let shift = |t: f64, point: Point3| {
                let delta = (circle.project(point) - t + PI).rem_euclid(TAU) - PI;
                t + delta
            };
            let (n0, n1) = (shift(t0, new_a), shift(t1, new_b));
            if n1 - n0 <= Tolerance::new().angular || n1 - n0 >= TAU {
                return Err(ambiguous(format!(
                    "retained arc {} would reverse or wrap",
                    edge.index()
                )));
            }
            let mut replacement = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
            replacement.set_trim(Some((n0, n1)));
            Ok(replacement)
        }
        other => Err(ambiguous(format!(
            "retained {} edge {} would have to be re-trimmed",
            other.type_tag(),
            edge.index()
        ))),
    }
}

/// Whether a sharp edge runs forward (its pieces' natural direction) in the
/// loop that used `contact`.
fn sharp_forward(
    work: &Topology,
    contact: OrientedEdge,
    pieces: &[EdgeId],
    carrier: &SharpCarrier,
) -> Result<bool, OperationsError> {
    let data = work.edge(contact.edge())?;
    let a = work.vertex(data.start())?.point();
    let b = work.vertex(data.end())?.point();
    let (t0, t1) = data.domain_with_endpoints(a, b);
    let middle = 0.5 * (t0 + t1);
    let point = data.curve().evaluate_with_endpoints(middle, a, b);
    let mut tangent = data
        .curve()
        .tangent_with_endpoints(middle, a, b)
        .normalize()
        .map_err(|_| ambiguous("a contact edge has no tangent"))?;
    if !contact.is_forward() {
        tangent = tangent * -1.0;
    }
    let forward = match *carrier {
        SharpCarrier::Line { .. } => {
            let first = work.edge(pieces[0])?;
            let start = work.vertex(first.start())?.point();
            let end = work.vertex(first.end())?.point();
            (end - start)
                .normalize()
                .map_err(|_| ambiguous("a sharp line has no direction"))?
        }
        SharpCarrier::Circle { center, axis, .. } => axis
            .cross(radial(point - center, axis))
            .normalize()
            .map_err(|_| ambiguous("a contact edge lies on its sharp circle's axis"))?,
    };
    let cosine = tangent.dot(forward);
    if cosine.abs() < ORIENTATION_COSINE {
        return Err(ambiguous(
            "a contact edge does not run along its sharp edge",
        ));
    }
    Ok(cosine > 0.0)
}

// ---------------------------------------------------------------------------
// Postconditions and history.

fn postconditions(work: &Topology, solid: SolidId) -> Result<(), OperationsError> {
    let report = remus_check::validate::validate_solid(
        work,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )?;
    if !report.is_valid() {
        let detail = report
            .issues
            .iter()
            .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
            .take(3)
            .map(|issue| format!("{} ({:?})", issue.description, issue.entity))
            .collect::<Vec<_>>()
            .join("; ");
        return Err(rebuild_failed(format!(
            "the sharp body fails validation with {} error(s): {detail}",
            report.error_count()
        )));
    }
    if crate::measure::solid_volume(work, solid, 0.05)? <= 0.0 {
        return Err(rebuild_failed("the sharp body encloses no volume"));
    }
    Ok(())
}

fn history(
    topo: &Topology,
    solid: SolidId,
    cluster: &BlendCluster,
    removed: &[usize],
    copied: &CopiedSolidEntities,
    built: &Built,
    output: &CopiedSolidEntities,
) -> Result<SharpRestoration, OperationsError> {
    let lost = || rebuild_failed("the sharp body lost an entity in its final copy");
    let final_edge = |edge: EdgeId| output.edge_map.get(&edge.index()).copied();
    let final_vertex = |vertex: VertexId| output.vertex_map.get(&vertex.index()).copied();

    let mut work_to_source: BTreeMap<FaceId, FaceId> = BTreeMap::new();
    for source in remus_topology::explorer::solid_faces(topo, solid)? {
        let work = copied
            .face_map
            .get(&source.index())
            .copied()
            .ok_or_else(lost)?;
        work_to_source.insert(work, source);
    }
    let mut faces = Vec::with_capacity(built.faces.len());
    let mut evolution = EvolutionMap::exact();
    for &work in &built.faces {
        let source = work_to_source[&work];
        let result = output
            .face_map
            .get(&work.index())
            .copied()
            .ok_or_else(lost)?;
        faces.push((source, result));
        evolution.add_modified(source.index(), result.index());
    }
    for &member in removed {
        for &face in &cluster.members[member].faces {
            evolution.add_deleted(face.index());
        }
    }

    let sharp_edges = built
        .sharp_edges
        .iter()
        .map(|pieces| {
            pieces
                .iter()
                .map(|piece| final_edge(*piece).ok_or_else(lost))
                .collect::<Result<Vec<_>, _>>()
        })
        .collect::<Result<Vec<_>, _>>()?;
    let sharp_vertices = built
        .sharp_vertices
        .iter()
        .map(|vertex| {
            vertex
                .map(|vertex| final_vertex(vertex).ok_or_else(lost))
                .transpose()
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut boundary_history = Vec::new();
    let mut retrimmed_edges = Vec::new();
    let mut source_edges: Vec<EdgeId> = remus_topology::explorer::solid_edges(topo, solid)?;
    source_edges.compact_sort_unstable();
    for source in source_edges {
        let work = copied
            .edge_map
            .get(&source.index())
            .copied()
            .ok_or_else(lost)?;
        let image = built.edge_image.get(&work).copied().unwrap_or(work);
        let result = final_edge(image);
        if built.edge_image.contains_key(&work)
            && let Some(result) = result
        {
            retrimmed_edges.push((source, result));
        }
        boundary_history.push((
            EntityKey::edge(source.index()),
            result.map(|edge| EntityKey::edge(edge.index())),
        ));
    }
    let mut source_vertices: Vec<VertexId> = remus_topology::explorer::solid_vertices(topo, solid)?;
    source_vertices.compact_sort_unstable();
    for source in source_vertices {
        let work = copied
            .vertex_map
            .get(&source.index())
            .copied()
            .ok_or_else(lost)?;
        let image = built.vertex_image.get(&work).copied().unwrap_or(work);
        boundary_history.push((
            EntityKey::vertex(source.index()),
            final_vertex(image).map(|vertex| EntityKey::vertex(vertex.index())),
        ));
    }
    Ok(SharpRestoration {
        solid: output.solid,
        removed: removed.to_vec(),
        faces,
        sharp_edges,
        sharp_vertices,
        retrimmed_edges,
        evolution,
        boundary_history,
    })
}

#[cfg(test)]
mod tests;
