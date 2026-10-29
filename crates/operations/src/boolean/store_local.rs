//! Opt-in store-local booleans: the GFA run on the faces of the blank (the
//! first operand) that the tool (the second) touches, spliced back onto the
//! faces it does not (stage 4 of `docs/performance/perf-local-boolean.md`).
//!
//! The general GFA copies the whole blank into its store, pave-fills,
//! splits, classifies and assembles every face, and copies the whole result
//! back, so a boolean on a body with many faces pays for all of them. This
//! path partitions the blank instead:
//!
//! - **touched** faces: those whose outer box comes within the clearance of
//!   the tool's box;
//! - **untouched** faces: the rest. Each lies farther than the clearance from
//!   the tool, so it does not meet it, and it lies outside the tool. A cut or
//!   a fuse keeps such a face as it is; an intersect drops it. Every edge it
//!   shares with a touched face is one of its own edges, so that edge is
//!   equally far from the tool and the boolean leaves it untouched too.
//!
//! [`remus_algo::gfa::boolean_patch`] runs the GFA on the touched faces and
//! the whole tool: it classifies tool pieces against the whole blank, and it
//! returns the faces the operation keeps after assembly's cleanup passes,
//! without closing them into a shell. For a cut or fuse the patch is then
//! spliced into a copy of the untouched faces, sharing the blank's own edges
//! and vertices wherever the boolean left them unchanged; for an intersect
//! the patch is the whole result. The result must pass the strict
//! boolean-result gate and form one connected shell (the general GFA keeps
//! separate lumps as separate solids), or the attempt is rolled back and the
//! general path runs.
//!
//! The result equals the general GFA's in geometry and topology (the oracle
//! tests compare faces, edges, vertices, volume and validity), but not in
//! face order or entity numbering. That is why the path is off unless a
//! thread enables it (`set_thread_store_local_cuts`, which covers fuse and
//! intersect as well as cut).
//!
//! Assumption: the untouched faces need none of assembly's cleanup (vertex
//! welds, T-junction splits, duplicate-edge merges), as holds for a clean
//! blank such as a primitive or an earlier boolean's result. The general
//! GFA would also run those passes on them.

use std::cell::Cell;

use remus_math::context::OperationContext;
use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use crate::copy::{
    ShellSlot, Splice, SpliceEdge, SpliceEdgeRef, SpliceFace, SpliceVertex, SpliceWire,
};

/// Clearance, in linear tolerances, between the tool's box and an untouched
/// face, as the local drilled-hole cells use.
const CLEARANCE_TOLERANCES: f64 = 1.0e3;

thread_local! {
    static ENABLED: Cell<bool> = const { Cell::new(false) };
}

#[cfg(test)]
thread_local! {
    /// Booleans this thread built store-locally, so tests can tell which path
    /// ran.
    pub(super) static BUILT: Cell<usize> = const { Cell::new(0) };
}

/// Turn store-local booleans on or off for this thread (off by default).
///
/// When on, a cut, fuse or intersect whose tool (second operand) reaches
/// only part of the blank (first operand) runs the GFA on that part and
/// splices the result back. Its result equals the general path's in geometry
/// and topology, but faces and entities come out in a different order. The
/// name predates fuse and intersect.
pub fn set_thread_store_local_cuts(enabled: bool) {
    ENABLED.with(|flag| flag.set(enabled));
}

/// Whether this thread runs store-local booleans.
#[must_use]
pub fn thread_store_local_cuts_enabled() -> bool {
    ENABLED.with(Cell::get)
}

/// Run `op` on `a` and `b` store-locally when enabled and the tool `b`
/// touches less than half of the blank `a`'s faces.
///
/// Returns `Ok(None)`, having changed nothing, when the path is off, does
/// not apply, the patch GFA fails, or the result fails the strict
/// boolean-result gate or is not one connected shell; the caller then runs
/// the general path.
///
/// # Errors
///
/// Returns an error only for a failed topology lookup while reading the
/// operands.
pub(super) fn try_store_local_boolean(
    topo: &mut Topology,
    op: super::BooleanOp,
    a: SolidId,
    b: SolidId,
    context: &OperationContext,
) -> Result<Option<SolidId>, crate::OperationsError> {
    if !thread_store_local_cuts_enabled() || a == b {
        return Ok(None);
    }
    crate::performance::bump_boolean_route(crate::performance::BooleanRoute::StoreLocalAttempt);
    let tol = context.tolerance;
    let Some((touched, untouched)) = partition(topo, a, b, tol)? else {
        return Ok(None);
    };
    let algo_op = match op {
        super::BooleanOp::Fuse => remus_algo::bop::BooleanOp::Fuse,
        super::BooleanOp::Cut => remus_algo::bop::BooleanOp::Cut,
        super::BooleanOp::Intersect => remus_algo::bop::BooleanOp::Intersect,
    };
    let patch = match remus_algo::gfa::boolean_patch(topo, algo_op, a, &touched, b, context) {
        Ok(patch) => patch,
        Err(error) => {
            log::debug!("store-local {op:?} declined: patch GFA failed: {error}");
            return Ok(None);
        }
    };
    if patch.faces.is_empty() {
        return Ok(None);
    }
    // A cut or fuse keeps the untouched faces; an intersect drops them, and
    // its patch is the whole result.
    let kept = match op {
        super::BooleanOp::Fuse | super::BooleanOp::Cut => untouched.clone(),
        super::BooleanOp::Intersect => Vec::new(),
    };
    let replaced: Vec<FaceId> = match op {
        super::BooleanOp::Fuse | super::BooleanOp::Cut => touched.clone(),
        super::BooleanOp::Intersect => touched.iter().chain(&untouched).copied().collect(),
    };
    let Some(splice) = patch_splice(topo, &touched, &kept, replaced, &patch)? else {
        return Ok(None);
    };
    let attempt = remus_topology::transaction::run_transacted(topo, |topo| {
        let result = crate::copy::copy_solid_spliced(topo, a, &splice)?;
        super::assembly::validate_boolean_result_with_tolerance(topo, result, tol)?;
        if !shell_is_connected(topo, result)? {
            return Err(crate::OperationsError::InvalidInput {
                reason: "store-local result is not one connected shell".into(),
            });
        }
        Ok::<_, crate::OperationsError>(result)
    });
    match attempt {
        Ok(result) => {
            crate::performance::bump_boolean_route(
                crate::performance::BooleanRoute::StoreLocalSuccess,
            );
            log::debug!(
                "{op:?} built store-locally: {} of {} faces touched",
                touched.len(),
                touched.len() + untouched.len()
            );
            #[cfg(test)]
            BUILT.with(|built| built.set(built.get() + 1));
            Ok(Some(result))
        }
        Err(error) => {
            log::debug!("store-local {op:?} declined after splicing: {error}");
            Ok(None)
        }
    }
}

/// The blank's outer-shell faces: `(touched, untouched)`.
type Partition = (Vec<FaceId>, Vec<FaceId>);

/// Split the blank's faces into those that may meet the tool and those that
/// cannot, or `None` when the path does not apply: a blank with cavities,
/// no touched face, or a cut touching at least half the faces (where the
/// general path costs no more).
fn partition(
    topo: &Topology,
    a: SolidId,
    b: SolidId,
    tol: remus_math::tolerance::Tolerance,
) -> Result<Option<Partition>, crate::OperationsError> {
    let solid = topo.solid(a)?;
    if !solid.inner_shells().is_empty() {
        return Ok(None);
    }
    let Some(entity_tolerance) = super::local_cut::max_entity_tolerance(topo, a)
        .zip(super::local_cut::max_entity_tolerance(topo, b))
        .map(|(ta, tb)| ta.max(tb))
    else {
        return Ok(None);
    };
    let clearance = CLEARANCE_TOLERANCES * tol.linear + entity_tolerance;
    let tool_faces = remus_topology::explorer::solid_faces(topo, b)?;
    let reach =
        crate::measure::conservative_face_set_bounding_box(topo, &tool_faces)?.expanded(clearance);
    let (mut touched, mut untouched) = (Vec::new(), Vec::new());
    for &face_id in topo.shell(solid.outer_shell())?.faces() {
        let bounds = match super::local_cut::ruled_face_bound(topo, face_id)? {
            Some(bounds) => bounds,
            None => crate::measure::conservative_face_set_bounding_box(topo, &[face_id])?,
        };
        if reach.intersects(bounds) {
            touched.push(face_id);
        } else {
            untouched.push(face_id);
        }
    }
    Ok((!touched.is_empty() && untouched.len() >= touched.len()).then_some((touched, untouched)))
}

/// Exact bits of an edge's geometry: curve, trim and endpoint positions,
/// and whether the edge runs against that key. A line's key takes its
/// endpoints in a fixed order, so a line rebuilt end to start matches and
/// reports `true`. `None` for a curve kind the key does not cover.
fn edge_key(
    topo: &Topology,
    edge_id: EdgeId,
) -> Result<Option<(Vec<u64>, bool)>, crate::OperationsError> {
    let edge = topo.edge(edge_id)?;
    let (start, end) = (
        topo.vertex(edge.start())?.point(),
        topo.vertex(edge.end())?.point(),
    );
    let line = matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line);
    let flipped = line && point_key(end) < point_key(start);
    let (first, second) = if flipped { (end, start) } else { (start, end) };
    let mut words: Vec<f64> = Vec::with_capacity(24);
    let mut point = |p: remus_math::vec::Point3| words.extend([p.x(), p.y(), p.z()]);
    point(first);
    point(second);
    match edge.curve() {
        remus_topology::edge::EdgeCurve::Line => words.push(0.0),
        remus_topology::edge::EdgeCurve::Circle(c) => {
            words.push(1.0);
            for v in [
                c.center() - remus_math::vec::Point3::new(0.0, 0.0, 0.0),
                c.normal(),
                c.u_axis(),
                c.v_axis(),
            ] {
                words.extend([v.x(), v.y(), v.z()]);
            }
            words.push(c.radius());
        }
        remus_topology::edge::EdgeCurve::Ellipse(e) => {
            words.push(2.0);
            for v in [
                e.center() - remus_math::vec::Point3::new(0.0, 0.0, 0.0),
                e.normal(),
                e.u_axis(),
                e.v_axis(),
            ] {
                words.extend([v.x(), v.y(), v.z()]);
            }
            words.extend([e.semi_major(), e.semi_minor()]);
        }
        remus_topology::edge::EdgeCurve::NurbsCurve(_)
        | remus_topology::edge::EdgeCurve::Hyperbola(_)
        | remus_topology::edge::EdgeCurve::Parabola(_) => return Ok(None),
    }
    match edge.trim() {
        Some((t0, t1)) => words.extend([1.0, t0, t1]),
        None => words.push(0.0),
    }
    Ok(Some((
        words.into_iter().map(f64::to_bits).collect(),
        flipped,
    )))
}

fn point_key(p: remus_math::vec::Point3) -> [u64; 3] {
    [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]
}

/// The splice that rebuilds the blank with the patch in place of the
/// touched faces, or `None` when the patch holds a NURBS face or per-use
/// authority that the splice representation cannot carry.
///
/// The builder rebuilds every patch face from fresh entities, so the patch
/// is matched back to the blank only at the boundary with kept faces: a
/// patch edge whose curve, trim and endpoints equal a shared source edge
/// bit for bit is that edge, and a patch vertex at a shared source vertex
/// position is that vertex. Everything inside the patch retains the GFA's
/// own edges, vertices and stored tolerances, even when its geometry equals
/// a source entity. A changed border stays spliced and leaves the shell
/// open, which the result gate refuses. The copied kept face supplies its
/// original edge authority: full GFA assembly also keeps that earlier input
/// edge when merging a rebuilt duplicate into it. Different vertex authority
/// and ambiguous matches decline rather than choosing between distinct
/// entities or replacing a vertex's stored tolerance.
fn patch_splice(
    topo: &Topology,
    touched: &[FaceId],
    kept: &[FaceId],
    replaced: Vec<FaceId>,
    patch: &remus_algo::gfa::CutPatch,
) -> Result<Option<Splice>, crate::OperationsError> {
    let mut kept_edges = DetHashSet::default();
    let mut kept_vertices = DetHashSet::default();
    for &face_id in kept {
        for edge_id in remus_topology::explorer::face_edges(topo, face_id)? {
            kept_edges.insert(edge_id);
            let edge = topo.edge(edge_id)?;
            kept_vertices.extend([edge.start(), edge.end()]);
        }
    }
    let mut caller_edges: DetHashMap<Vec<u64>, (EdgeId, bool)> = DetHashMap::default();
    let mut caller_vertices: DetHashMap<[u64; 3], VertexId> = DetHashMap::default();
    for &face_id in touched {
        for edge_id in remus_topology::explorer::face_edges(topo, face_id)? {
            if kept_edges.contains(&edge_id)
                && let Some((key, flipped)) = edge_key(topo, edge_id)?
            {
                let previous = caller_edges.entry(key).or_insert((edge_id, flipped));
                if previous.0 != edge_id {
                    return Ok(None);
                }
            }
            let edge = topo.edge(edge_id)?;
            for vertex in [edge.start(), edge.end()] {
                if kept_vertices.contains(&vertex) {
                    let previous = caller_vertices
                        .entry(point_key(topo.vertex(vertex)?.point()))
                        .or_insert(vertex);
                    if *previous != vertex {
                        return Ok(None);
                    }
                }
            }
        }
    }
    let store = &patch.topo;
    let mut splice = Splice {
        replaced,
        ..Splice::default()
    };
    let mut new_vertices: DetHashMap<usize, usize> = DetHashMap::default();
    let mut new_edges: DetHashMap<usize, usize> = DetHashMap::default();
    let mut matched_vertices = DetHashMap::default();
    let mut matched_edges = DetHashMap::default();
    let mut vertex_ref = |splice: &mut Splice,
                          vertex: VertexId|
     -> Result<Option<SpliceVertex>, crate::OperationsError> {
        let data = store.vertex(vertex)?;
        if let Some(caller) = caller_vertices.get(&point_key(data.point())) {
            if topo.vertex(*caller)?.tolerance().to_bits() != data.tolerance().to_bits() {
                return Ok(None);
            }
            if let Some(previous) = matched_vertices.insert(*caller, vertex)
                && previous != vertex
            {
                return Ok(None);
            }
            return Ok(Some(SpliceVertex::Source(*caller)));
        }
        if let Some(&index) = new_vertices.get(&vertex.index()) {
            return Ok(Some(SpliceVertex::New(index)));
        }
        splice.vertices.push((data.point(), data.tolerance()));
        let index = splice.vertices.len() - 1;
        new_vertices.insert(vertex.index(), index);
        Ok(Some(SpliceVertex::New(index)))
    };
    for (slot, &face_id) in patch.faces.iter().enumerate() {
        let face = store.face(face_id)?;
        if matches!(face.surface(), FaceSurface::Nurbs(_)) {
            return Ok(None);
        }
        for &loop_id in face.boundary_loops() {
            for &use_id in store.face_loop(loop_id)?.coedges() {
                let coedge = store.coedge(use_id)?;
                if coedge.pcurve().is_some()
                    || coedge.periodic_winding() != remus_topology::coedge::PeriodicWinding::ZERO
                {
                    return Ok(None);
                }
            }
        }
        let mut wire_indices = Vec::new();
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let mut uses = Vec::new();
            for oriented in store.wire(wire_id)?.edges() {
                let edge_id = oriented.edge();
                let edge = store.edge(edge_id)?;
                let Some(start) = vertex_ref(&mut splice, edge.start())? else {
                    return Ok(None);
                };
                let Some(end) = vertex_ref(&mut splice, edge.end())? else {
                    return Ok(None);
                };
                let mut forward = oriented.is_forward();
                let edge_ref = if let Some((caller, flip)) =
                    edge_key(store, edge_id)?.and_then(|(key, flipped)| {
                        caller_edges
                            .get(&key)
                            .map(|&(caller, caller_flipped)| (caller, flipped != caller_flipped))
                    }) {
                    if let Some(previous) = matched_edges.insert(caller, edge_id)
                        && previous != edge_id
                    {
                        return Ok(None);
                    }
                    // A line rebuilt end to start is used the other way round.
                    if flip {
                        forward = !forward;
                    }
                    SpliceEdgeRef::Source(caller)
                } else if let Some(&index) = new_edges.get(&edge_id.index()) {
                    SpliceEdgeRef::New(index)
                } else {
                    splice.edges.push(SpliceEdge {
                        start,
                        end,
                        curve: edge.curve().clone(),
                        trim: edge.trim(),
                        tolerance: edge.tolerance(),
                    });
                    let index = splice.edges.len() - 1;
                    new_edges.insert(edge_id.index(), index);
                    SpliceEdgeRef::New(index)
                };
                uses.push((edge_ref, forward));
            }
            splice.wires.push(SpliceWire {
                edges: uses,
                closed: store.wire(wire_id)?.is_closed(),
            });
            wire_indices.push(splice.wires.len() - 1);
        }
        splice.faces.push(SpliceFace {
            outer: wire_indices[0],
            inner: wire_indices[1..].to_vec(),
            surface: face.surface().clone(),
            reversed: face.is_reversed(),
        });
        splice.outer_shell.push(ShellSlot::Spliced(slot));
    }
    let patch_slots = std::mem::take(&mut splice.outer_shell);
    splice.outer_shell = kept
        .iter()
        .map(|&face| ShellSlot::Source(face))
        .chain(patch_slots)
        .collect();
    Ok(Some(splice))
}

/// Whether the result's outer shell is one edge-connected piece. A fuse can
/// leave the patch as a lump of its own (a tool that the clearance reached
/// but that does not meet the blank), which the general path would return
/// as a separate solid.
fn shell_is_connected(topo: &Topology, solid: SolidId) -> Result<bool, crate::OperationsError> {
    let faces = topo
        .shell(topo.solid(solid)?.outer_shell())?
        .faces()
        .to_vec();
    let mut edge_faces: DetHashMap<usize, Vec<usize>> = DetHashMap::default();
    let mut face_edges = Vec::with_capacity(faces.len());
    for (ordinal, &face_id) in faces.iter().enumerate() {
        let face = topo.face(face_id)?;
        let mut edges = Vec::new();
        for &wire_id in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
            for oriented in topo.wire(wire_id)?.edges() {
                let edge = oriented.edge().index();
                edge_faces.entry(edge).or_default().push(ordinal);
                edges.push(edge);
            }
        }
        face_edges.push(edges);
    }
    let mut seen = vec![false; faces.len()];
    let mut stack = vec![0];
    let mut reached = 0;
    while let Some(ordinal) = stack.pop() {
        if std::mem::replace(&mut seen[ordinal], true) {
            continue;
        }
        reached += 1;
        for edge in &face_edges[ordinal] {
            stack.extend(edge_faces[edge].iter().filter(|&&next| !seen[next]));
        }
    }
    Ok(reached == faces.len())
}

#[cfg(test)]
mod boundary_match_tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::mat::Mat4;
    use remus_math::vec::Point3;

    use super::*;

    fn boundary_fixture() -> (
        Topology,
        Vec<FaceId>,
        Vec<FaceId>,
        remus_algo::gfa::CutPatch,
    ) {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        let touched = vec![faces[0]];
        let kept = faces[1..].to_vec();
        let patch = remus_algo::gfa::CutPatch {
            topo: topo.clone(),
            faces: touched.clone(),
            edge_to_caller: DetHashMap::default(),
            vertex_to_caller: DetHashMap::default(),
        };
        assert!(
            patch_splice(&topo, &touched, &kept, touched.clone(), &patch)
                .unwrap()
                .is_some()
        );
        (topo, touched, kept, patch)
    }

    #[test]
    fn shared_boundary_vertex_authority_must_match_exactly() {
        let (topo, touched, kept, mut patch) = boundary_fixture();
        let edge_id = remus_topology::explorer::face_edges(&patch.topo, patch.faces[0]).unwrap()[0];
        let vertex = patch.topo.edge(edge_id).unwrap().start();
        patch
            .topo
            .vertex_mut(vertex)
            .unwrap()
            .set_tolerance(2e-7)
            .unwrap();
        assert!(
            patch_splice(&topo, &touched, &kept, touched.clone(), &patch)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn distinct_patch_edges_cannot_collapse_onto_one_source_edge() {
        use remus_topology::face::Face;
        use remus_topology::wire::{OrientedEdge, Wire};

        let (topo, touched, kept, mut patch) = boundary_fixture();
        let face = patch.topo.face(patch.faces[0]).unwrap();
        let surface = face.surface().clone();
        let mut uses = patch.topo.wire(face.outer_wire()).unwrap().edges().to_vec();
        let duplicate = patch.topo.edge(uses[0].edge()).unwrap().clone();
        let edge = patch.topo.add_edge(duplicate);
        uses[0] = OrientedEdge::new(edge, uses[0].is_forward());
        let wire = patch.topo.add_wire(Wire::new(uses, true).unwrap());
        let face = patch.topo.add_face(Face::new(wire, Vec::new(), surface));
        patch.faces.push(face);
        assert!(
            patch_splice(&topo, &touched, &kept, touched.clone(), &patch)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn distinct_patch_vertices_cannot_collapse_onto_one_source_vertex() {
        let (topo, touched, kept, mut patch) = boundary_fixture();
        let duplicate = crate::primitives::make_box(&mut patch.topo, 1.0, 1.0, 1.0).unwrap();
        let face = remus_topology::explorer::solid_faces(&patch.topo, duplicate).unwrap()[0];
        patch.faces.push(face);
        assert!(
            patch_splice(&topo, &touched, &kept, touched.clone(), &patch)
                .unwrap()
                .is_none()
        );
    }

    fn ambiguous_boundary_declines(offset: f64) {
        let mut topo = Topology::new();
        let first = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let second = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            second,
            &Mat4::translation(offset, offset, offset),
        )
        .unwrap();
        let common = Point3::new(1.0, 1.0, 1.0);
        let mut touched = Vec::new();
        let mut kept = Vec::new();
        for solid in [first, second] {
            let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
            let selected = *faces
                .iter()
                .find(|&&id| {
                    remus_topology::explorer::face_edges(&topo, id)
                        .unwrap()
                        .iter()
                        .any(|&id| {
                            let edge = topo.edge(id).unwrap();
                            [edge.start(), edge.end()].iter().any(|&id| {
                                point_key(topo.vertex(id).unwrap().point()) == point_key(common)
                            })
                        })
                })
                .unwrap();
            touched.push(selected);
            kept.extend(faces.into_iter().filter(|&id| id != selected));
        }
        let before = format!("{topo:?}");
        // Two independent source boxes deliberately supply distinct boundary
        // identities with the same exact geometry. This tests lookup refusal,
        // not acceptance of a multi-body blank or a malformed solid.
        let patch = remus_algo::gfa::CutPatch {
            topo: Topology::new(),
            faces: Vec::new(),
            edge_to_caller: DetHashMap::default(),
            vertex_to_caller: DetHashMap::default(),
        };
        assert!(
            patch_splice(&topo, &touched, &kept, touched.clone(), &patch)
                .unwrap()
                .is_none()
        );
        assert_eq!(format!("{topo:?}"), before);
    }

    #[test]
    fn coincident_distinct_kept_boundary_edges_decline() {
        ambiguous_boundary_declines(0.0);
    }

    #[test]
    fn coincident_distinct_kept_boundary_vertices_decline() {
        // The boxes meet only at one point, so no edge keys coincide.
        ambiguous_boundary_declines(1.0);
    }

    #[test]
    fn patch_per_use_authority_declines_instead_of_discarding_it() {
        use remus_math::curves2d::{Curve2D, Line2D};
        use remus_math::vec::{Point2, Vec2};
        use remus_topology::coedge::PeriodicWinding;
        use remus_topology::pcurve::PCurve;

        let mut store = Topology::new();
        let solid = crate::primitives::make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let faces = remus_topology::explorer::solid_faces(&store, solid).unwrap();
        let loop_id = store.face(faces[0]).unwrap().boundary_loops()[0];
        let use_id = store.face_loop(loop_id).unwrap().coedges()[0];
        let topo = Topology::new();
        for pcurve in [false, true] {
            let mut patch = remus_algo::gfa::CutPatch {
                topo: store.clone(),
                faces: faces.clone(),
                edge_to_caller: DetHashMap::default(),
                vertex_to_caller: DetHashMap::default(),
            };
            if pcurve {
                let curve = Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap();
                patch
                    .topo
                    .set_coedge_pcurve(use_id, PCurve::new(Curve2D::Line(curve), 0.0, 1.0))
                    .unwrap();
            } else {
                patch
                    .topo
                    .set_coedge_periodic_winding(use_id, PeriodicWinding::new(1, 0))
                    .unwrap();
            }
            assert!(
                patch_splice(&topo, &[], &[], Vec::new(), &patch)
                    .unwrap()
                    .is_none()
            );
        }
    }
}
