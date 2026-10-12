//! Opt-in store-local cut: the GFA run on the faces a cut touches, spliced
//! back onto the faces it does not (stage 4 of
//! `docs/performance/perf-local-boolean.md`).
//!
//! The general GFA copies the whole blank into its store, pave-fills,
//! splits, classifies and assembles every face, and copies the whole result
//! back, so a cut through a body with many faces pays for all of them. This
//! path partitions the blank instead:
//!
//! - **touched** faces: those whose outer box comes within the clearance of
//!   the tool's box;
//! - **untouched** faces: the rest. Each lies farther than the clearance from
//!   the tool, so it does not meet it, and it lies outside the tool, so a cut
//!   keeps it as it is. Every edge it shares with a touched face is one of its
//!   own edges, so that edge is equally far from the tool and the cut leaves it
//!   untouched too.
//!
//! [`remus_algo::gfa::cut_patch`] runs the GFA on the touched faces and the
//! whole tool: it classifies tool pieces against the whole blank, and it
//! returns the patch faces after assembly's cleanup passes, without closing
//! them into a shell. The patch is then spliced into a copy of the
//! untouched faces, sharing the blank's own edges and vertices wherever the
//! cut left them unchanged. The result must pass the strict boolean-result
//! gate, or the attempt is rolled back and the general path runs.
//!
//! The result equals the general GFA's in geometry and topology (the oracle
//! tests compare faces, edges, vertices, volume and validity), but not in
//! face order or entity numbering. That is why the path is off unless a
//! thread enables it (`set_thread_store_local_cuts`).
//!
//! Assumption: the untouched faces need none of assembly's cleanup (vertex
//! welds, T-junction splits, duplicate-edge merges), as holds for a clean
//! blank such as a primitive or an earlier boolean's result. The general
//! GFA would also run those passes on them.

use std::cell::Cell;

use remus_math::context::OperationContext;
use remus_math::det_hash::DetHashMap;
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
    /// Cuts this thread built store-locally, so tests can tell which path ran.
    pub(super) static BUILT: Cell<usize> = const { Cell::new(0) };
}

/// Turn the store-local cut on or off for this thread (off by default).
///
/// When on, a cut whose tool reaches only part of the blank runs the GFA on
/// that part and splices the result back. Its result equals the general
/// path's in geometry and topology, but faces and entities come out in a
/// different order.
pub fn set_thread_store_local_cuts(enabled: bool) {
    ENABLED.with(|flag| flag.set(enabled));
}

/// Whether this thread runs store-local cuts.
#[must_use]
pub fn thread_store_local_cuts_enabled() -> bool {
    ENABLED.with(Cell::get)
}

/// Cut `b` from `a` store-locally when enabled and the cut touches less
/// than half of the blank's faces.
///
/// Returns `Ok(None)`, having changed nothing, when the path is off, does
/// not apply, the patch GFA fails, or the spliced result fails the strict
/// boolean-result gate; the caller then runs the general path.
///
/// # Errors
///
/// Returns an error only for a failed topology lookup while reading the
/// operands.
pub(super) fn try_store_local_cut(
    topo: &mut Topology,
    a: SolidId,
    b: SolidId,
    context: &OperationContext,
) -> Result<Option<SolidId>, crate::OperationsError> {
    if !thread_store_local_cuts_enabled() || a == b {
        return Ok(None);
    }
    let tol = context.tolerance;
    let Some((touched, untouched)) = partition(topo, a, b, tol)? else {
        return Ok(None);
    };
    let patch = match remus_algo::gfa::cut_patch(topo, a, &touched, b, context) {
        Ok(patch) => patch,
        Err(error) => {
            log::debug!("store-local cut declined: patch GFA failed: {error}");
            return Ok(None);
        }
    };
    let Some(splice) = patch_splice(topo, &touched, &untouched, &patch)? else {
        return Ok(None);
    };
    let attempt = remus_topology::transaction::run_transacted(topo, |topo| {
        let result = crate::copy::copy_solid_spliced(topo, a, &splice)?;
        super::assembly::validate_boolean_result_with_tolerance(topo, result, tol)?;
        Ok::<_, crate::OperationsError>(result)
    });
    match attempt {
        Ok(result) => {
            log::debug!(
                "Cut built store-locally: {} of {} faces touched",
                touched.len(),
                touched.len() + untouched.len()
            );
            #[cfg(test)]
            BUILT.with(|built| built.set(built.get() + 1));
            Ok(Some(result))
        }
        Err(error) => {
            log::debug!("store-local cut declined after splicing: {error}");
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
/// touched faces, or `None` when the patch holds a NURBS face (whose stored
/// pcurves a spliced face would not carry).
///
/// The builder rebuilds every patch face from fresh entities, so the patch
/// is matched back to the blank by geometry: a patch edge whose curve, trim
/// and endpoints equal a touched face's edge bit for bit is that edge, and
/// a patch vertex at a touched face's vertex position is that vertex. Those
/// are shared with the copied untouched faces; everything else is spliced
/// from the patch's store. A border edge the cut changed in any bit stays
/// spliced and leaves the shell open, which the result gate refuses.
fn patch_splice(
    topo: &Topology,
    touched: &[FaceId],
    untouched: &[FaceId],
    patch: &remus_algo::gfa::CutPatch,
) -> Result<Option<Splice>, crate::OperationsError> {
    let mut caller_edges: DetHashMap<Vec<u64>, (EdgeId, bool)> = DetHashMap::default();
    let mut caller_vertices: DetHashMap<[u64; 3], VertexId> = DetHashMap::default();
    for &face_id in touched {
        for edge_id in remus_topology::explorer::face_edges(topo, face_id)? {
            if let Some((key, flipped)) = edge_key(topo, edge_id)? {
                caller_edges.entry(key).or_insert((edge_id, flipped));
            }
            let edge = topo.edge(edge_id)?;
            for vertex in [edge.start(), edge.end()] {
                caller_vertices
                    .entry(point_key(topo.vertex(vertex)?.point()))
                    .or_insert(vertex);
            }
        }
    }
    let store = &patch.topo;
    let mut splice = Splice {
        replaced: touched.to_vec(),
        ..Splice::default()
    };
    let mut new_vertices: DetHashMap<usize, usize> = DetHashMap::default();
    let mut new_edges: DetHashMap<usize, usize> = DetHashMap::default();
    let mut vertex_ref =
        |splice: &mut Splice, vertex: VertexId| -> Result<SpliceVertex, crate::OperationsError> {
            let data = store.vertex(vertex)?;
            if let Some(caller) = caller_vertices.get(&point_key(data.point())) {
                return Ok(SpliceVertex::Source(*caller));
            }
            if let Some(&index) = new_vertices.get(&vertex.index()) {
                return Ok(SpliceVertex::New(index));
            }
            splice.vertices.push((data.point(), data.tolerance()));
            let index = splice.vertices.len() - 1;
            new_vertices.insert(vertex.index(), index);
            Ok(SpliceVertex::New(index))
        };
    for (slot, &face_id) in patch.faces.iter().enumerate() {
        let face = store.face(face_id)?;
        if matches!(face.surface(), FaceSurface::Nurbs(_)) {
            return Ok(None);
        }
        let mut wire_indices = Vec::new();
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let mut uses = Vec::new();
            for oriented in store.wire(wire_id)?.edges() {
                let edge_id = oriented.edge();
                let mut forward = oriented.is_forward();
                let edge_ref = if let Some((caller, flip)) =
                    edge_key(store, edge_id)?.and_then(|(key, flipped)| {
                        caller_edges
                            .get(&key)
                            .map(|&(caller, caller_flipped)| (caller, flipped != caller_flipped))
                    }) {
                    // A line rebuilt end to start is used the other way round.
                    if flip {
                        forward = !forward;
                    }
                    SpliceEdgeRef::Source(caller)
                } else if let Some(&index) = new_edges.get(&edge_id.index()) {
                    SpliceEdgeRef::New(index)
                } else {
                    let edge = store.edge(edge_id)?;
                    let start = vertex_ref(&mut splice, edge.start())?;
                    let end = vertex_ref(&mut splice, edge.end())?;
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
    splice.outer_shell = untouched
        .iter()
        .map(|&face| ShellSlot::Source(face))
        .chain(patch_slots)
        .collect();
    Ok(Some(splice))
}
