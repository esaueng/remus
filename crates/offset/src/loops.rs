//! Wire loop construction from trimmed intersection edges.
//!
//! After earlier phases compute intersection curves between adjacent offset
//! faces and create preliminary edges, this phase trims those edges to their
//! mutual intersections and assembles them into closed wire loops for each
//! offset face.

use std::collections::{HashMap, HashSet};

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::data::{OffsetData, OffsetStatus, VertexCache, find_or_create_vertex};
use crate::error::OffsetError;

type LoopBuild = (Vec<WireId>, Vec<(EdgeId, EdgeId)>);

/// Squared sine of the angle below which two intersection lines are treated
/// as parallel. Dimensionless, so it means the same thing at every scale.
const PARALLEL_SIN_SQ: f64 = 1e-20;

/// Build closed wire loops for each offset face from the trimmed
/// intersection curves and split edges.
///
/// For each non-excluded offset face, collects intersection edges that
/// touch the face, computes their pairwise intersections to find corner
/// vertices, creates trimmed edges between those corners, and assembles
/// them into closed wire loops.
///
/// # Errors
///
/// Returns [`OffsetError`] if a wire loop cannot be closed or topology
/// lookups fail.
pub fn build_wire_loops(topo: &mut Topology, data: &mut OffsetData) -> Result<(), OffsetError> {
    let mut active_faces: Vec<FaceId> = data
        .offset_faces
        .iter()
        .filter(|(_, of)| of.status == OffsetStatus::Done)
        .map(|(&fid, _)| fid)
        .collect();
    active_faces.sort_by_key(|face_id| face_id.index());

    // Planar faces reconstruct their corners independently from the same
    // geometric lines. Keep one tolerance-deduplicated vertex and one
    // topological edge for every corner pair so adjacent faces form a
    // manifold shell instead of merely coincident, disconnected polygons.
    let mut corner_cache = VertexCache::new(data.options.tolerance.linear);
    let mut edge_cache: HashMap<(usize, usize), EdgeId> = HashMap::new();

    for face_id in active_faces {
        let (wires, boundary_edges) =
            build_loops_for_face(topo, data, face_id, &mut corner_cache, &mut edge_cache)?;
        for (original_edge, offset_edge) in boundary_edges {
            let entries = data
                .boundary_offset_edges
                .entry(original_edge.index())
                .or_default();
            if !entries.contains(&offset_edge) {
                entries.push(offset_edge);
            }
        }
        if !wires.is_empty() {
            data.face_wires.insert(face_id, wires);
        }
    }

    Ok(())
}

/// A line segment in 3D representing an intersection edge's geometry.
struct LineSeg {
    /// Start point of the intersection line.
    p0: Point3,
    /// End point of the intersection line.
    p1: Point3,
    /// Original boundary edge when this line closes a thick-solid opening.
    boundary_edge: Option<EdgeId>,
}

/// Build wire loops for a single face.
///
/// Tries three strategies in order:
/// 1. **Closed conic/seam pattern** — build wires from full-turn circle or
///    ellipse rims, with a seam for cylinder/cone lateral faces.
/// 2. **Direct chain** — if intersection edges already share vertices and
///    form closed loops, chain them directly (sphere polygon faces).
/// 3. **Line intersection** — find corners via pairwise line-line intersection,
///    create trimmed edges, walk loops (box faces).
#[allow(clippy::too_many_lines)]
fn build_loops_for_face(
    topo: &mut Topology,
    data: &OffsetData,
    face_id: FaceId,
    corner_cache: &mut VertexCache,
    edge_cache: &mut HashMap<(usize, usize), EdgeId>,
) -> Result<LoopBuild, OffsetError> {
    let mut face_edges: Vec<EdgeId> = Vec::new();
    for intersection in &data.intersections {
        if intersection.face_a != face_id && intersection.face_b != face_id {
            continue;
        }
        face_edges.extend_from_slice(&intersection.new_edges);
    }

    if let Some(boundary) = data.boundary_edges.get(&face_id) {
        face_edges.extend_from_slice(boundary);
    }

    // A full doubly-periodic torus offset face carries only degenerate v0->v0
    // seam edges, which the generic strategies below can't use; rebuild its
    // fundamental-polygon wire directly from the offset torus surface. Gate on
    // the absence of any real (non-degenerate) edge so a TRIMMED torus patch
    // (e.g. a fillet's torus face, which carries real boundary/intersection
    // edges) still flows through the normal strategies.
    let has_real_edge = face_edges
        .iter()
        .any(|&eid| topo.edge(eid).is_ok_and(|e| e.start() != e.end()));
    if !has_real_edge
        && let Some(off) = data.offset_faces.get(&face_id)
        && let FaceSurface::Torus(tor) = &off.surface
    {
        return Ok((
            build_torus_wire(topo, tor, data.options.tolerance.linear)?,
            Vec::new(),
        ));
    }

    if face_edges.is_empty() {
        return Ok((Vec::new(), Vec::new()));
    }

    if let Some(wires) = try_closed_conic_seam_wire(
        topo,
        &face_edges,
        data.offset_faces.get(&face_id).map(|face| &face.surface),
        data.options.tolerance,
    )? {
        return Ok((wires, Vec::new()));
    }

    // Planar intersection lines must be clipped with the shared caches below.
    // Using the direct-chain fast path here makes success depend on whether
    // oversized line endpoints happen to coincide, which varies with map
    // iteration order and produced intermittent non-manifold shells.
    if data
        .offset_faces
        .get(&face_id)
        .is_some_and(|face| matches!(face.surface, FaceSurface::Plane { .. }))
    {
        return build_loops_via_line_intersection(topo, data, face_id, corner_cache, edge_cache);
    }

    // The chained walk below starts at an arbitrary edge, so it fixes the
    // loop's traversal sense arbitrarily. On an open surface that is harmless
    // — a loop bounds one finite region whichever way it is walked. On a
    // CLOSED one the sense IS the region: the same equatorial loop bounds the
    // northern hemisphere walked one way and the southern hemisphere walked
    // the other. Hand the walk the source face's own sense so the offset face
    // covers the offset image of the region the source face covered.
    let source_sense = source_loop_sense(topo, face_id)?;
    if let Some(wires) = try_direct_chain(topo, &face_edges, source_sense)? {
        return Ok((wires, Vec::new()));
    }

    build_loops_via_line_intersection(topo, data, face_id, corner_cache, edge_cache)
}

/// Build a wire from closed circle/ellipse rims and a seam when needed.
///
/// Handles two patterns:
/// - **Single rim**: one full-turn edge gives a cap wire.
/// - **Two rims + seam**: a lateral face uses both rims and a shared seam.
fn try_closed_conic_seam_wire(
    topo: &mut Topology,
    edges: &[EdgeId],
    surface: Option<&FaceSurface>,
    tol: Tolerance,
) -> Result<Option<Vec<WireId>>, OffsetError> {
    let mut rims: Vec<EdgeId> = Vec::new();
    let mut others: Vec<EdgeId> = Vec::new();
    for &eid in edges {
        let edge = topo.edge(eid)?;
        if edge.start() == edge.end()
            && matches!(edge.curve(), EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_))
        {
            rims.push(eid);
        } else {
            others.push(eid);
        }
    }

    if rims.is_empty() {
        return Ok(None);
    }

    // Single full-turn rim: cap face.
    if rims.len() == 1 && others.is_empty() {
        let wire = Wire::new(vec![OrientedEdge::new(rims[0], true)], true)?;
        return Ok(Some(vec![topo.add_wire(wire)]));
    }

    // Two full-turn rims: cylinder/cone lateral face.
    if rims.len() == 2 && others.is_empty() {
        if rims.iter().any(|&eid| {
            topo.edge(eid)
                .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::Ellipse(_)))
        }) {
            let Some(FaceSurface::Cylinder(cylinder)) = surface else {
                return Err(OffsetError::AssemblyFailed {
                    reason: "mixed ellipse rims require a cylindrical lateral carrier".into(),
                });
            };
            align_second_rim_to_cylinder(topo, rims[0], rims[1], cylinder, tol)?;
        }
        let va = topo.edge(rims[0])?.start();
        let vb = topo.edge(rims[1])?.start();

        if va == vb {
            // Degenerate: same vertex — shouldn't happen, but handle gracefully.
            return Ok(None);
        }

        let seam = topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));

        // Wire: rim_a(fwd) → seam(fwd) → rim_b(rev) → seam(rev)
        let wire = Wire::new(
            vec![
                OrientedEdge::new(rims[0], true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(rims[1], false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )?;
        return Ok(Some(vec![topo.add_wire(wire)]));
    }

    // Mixed rims and other edges require a different trimming strategy.
    Ok(None)
}

/// Reanchor both full-turn rims at one cylinder azimuth. Their trims retain
/// traversal sense, and the cap wires reference the same reanchored edges.
fn align_second_rim_to_cylinder(
    topo: &mut Topology,
    first: EdgeId,
    second: EdgeId,
    cylinder: &remus_math::surfaces::CylindricalSurface,
    tol: Tolerance,
) -> Result<(), OffsetError> {
    let first_point = topo.vertex(topo.edge(first)?.start())?.point();
    let from_origin = first_point - cylinder.origin();
    let radial = from_origin - cylinder.axis() * from_origin.dot(cylinder.axis());
    let limit = tol.linear.max(tol.relative * cylinder.radius());
    if !radial.length().is_finite() || (radial.length() - cylinder.radius()).abs() > limit {
        return Err(OffsetError::AssemblyFailed {
            reason: "first rim seam is off its cylindrical carrier".into(),
        });
    }
    let azimuth = radial
        .dot(cylinder.y_axis())
        .atan2(radial.dot(cylinder.x_axis()));
    let base = cylinder.evaluate(azimuth, 0.0);
    let mut updates = Vec::with_capacity(2);
    for edge_id in [first, second] {
        let edge = topo.edge(edge_id)?;
        let (normal, center) = match edge.curve() {
            EdgeCurve::Circle(circle) => (circle.normal(), circle.center()),
            EdgeCurve::Ellipse(ellipse) => (ellipse.normal(), ellipse.center()),
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => {
                return Err(OffsetError::AssemblyFailed {
                    reason: "rim is not a closed conic".into(),
                });
            }
        };
        let denominator = normal.dot(cylinder.axis());
        if !denominator.is_finite() || denominator.abs() < 1e-6 {
            return Err(OffsetError::AssemblyFailed {
                reason: "rim plane is parallel to the cylinder generator".into(),
            });
        }
        let station = normal.dot(center - base) / denominator;
        let target = cylinder.evaluate(azimuth, station);
        let parameter = match edge.curve() {
            EdgeCurve::Circle(circle) => circle.project(target),
            EdgeCurve::Ellipse(ellipse) => ellipse.project(target),
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => {
                return Err(OffsetError::AssemblyFailed {
                    reason: "rim changed before seam alignment".into(),
                });
            }
        };
        let evaluated = edge
            .curve()
            .evaluate_with_endpoints(parameter, target, target);
        let range = edge
            .strict_domain()
            .map_err(|error| OffsetError::AssemblyFailed {
                reason: format!("rim lacks full-turn authority: {error}"),
            })?;
        if !station.is_finite()
            || !parameter.is_finite()
            || !target.0.iter().all(|value| value.is_finite())
            || (evaluated - target).length() > limit
        {
            return Err(OffsetError::AssemblyFailed {
                reason: "rim cannot be reanchored on its cylindrical carrier".into(),
            });
        }
        let trim = (parameter, parameter + range.1 - range.0);
        let mut candidate = edge.clone();
        candidate.set_trim(Some(trim));
        candidate
            .strict_domain()
            .map_err(|error| OffsetError::AssemblyFailed {
                reason: format!("reanchored rim has invalid trim: {error}"),
            })?;
        updates.push((edge_id, target, trim));
    }
    for (edge_id, target, trim) in updates {
        let vertex = topo.add_vertex(Vertex::new(target, tol.linear));
        let edge = topo.edge_mut(edge_id)?;
        edge.set_start(vertex);
        edge.set_end(vertex);
        edge.set_trim(Some(trim));
        edge.strict_domain()
            .map_err(|error| OffsetError::AssemblyFailed {
                reason: format!("reanchored rim has invalid trim: {error}"),
            })?;
    }
    Ok(())
}

/// Build the fundamental-polygon wire for a torus face: 1 seam vertex, two
/// degenerate seam edges, wire `a -> b -> a^-1 -> b^-1` (mirrors `make_torus`).
fn build_torus_wire(
    topo: &mut Topology,
    tor: &remus_math::surfaces::ToroidalSurface,
    tol: f64,
) -> Result<Vec<WireId>, OffsetError> {
    let seam = tor.evaluate(0.0, 0.0);
    let v0 = topo.add_vertex(Vertex::new(seam, tol));
    let ea = topo.add_edge(Edge::new(v0, v0, EdgeCurve::Line));
    let eb = topo.add_edge(Edge::new(v0, v0, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(ea, true),
            OrientedEdge::new(eb, true),
            OrientedEdge::new(ea, false),
            OrientedEdge::new(eb, false),
        ],
        true,
    )?;
    Ok(vec![topo.add_wire(wire)])
}

/// Vector area (Newell) of a boundary walk, from the vertices in traversal
/// order.
///
/// Only the DIRECTION carries information here: it flips when the walk is
/// reversed. Two walks of the same boundary are compared through the sign of a
/// dot product of two such vectors, so the test is a pure sign — no length
/// constant, no tolerance, identical at every model scale.
fn traversal_vector_area(topo: &Topology, walk: &[OrientedEdge]) -> Result<Vec3, OffsetError> {
    let mut points = Vec::with_capacity(walk.len());
    for oriented in walk {
        let edge = topo.edge(oriented.edge())?;
        let vertex = if oriented.is_forward() {
            edge.start()
        } else {
            edge.end()
        };
        points.push(topo.vertex(vertex)?.point());
    }
    let mut area = Vec3::new(0.0, 0.0, 0.0);
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        area = Vec3::new(
            area.x() + (current.y() - next.y()) * (current.z() + next.z()),
            area.y() + (current.z() - next.z()) * (current.x() + next.x()),
            area.z() + (current.x() - next.x()) * (current.y() + next.y()),
        );
    }
    Ok(area)
}

/// The traversal sense of a source face's outer boundary, as a vector area.
///
/// `None` when the face carries inner wires or its boundary is too degenerate
/// to give a direction — in those cases the reconstructed loop keeps whatever
/// sense the walk produced (the behaviour before this was added). Inner wires
/// wind opposite to the outer one, so a single outer-wire reference cannot
/// orient a multi-loop reconstruction and must not try.
fn source_loop_sense(topo: &Topology, face_id: FaceId) -> Result<Option<Vec3>, OffsetError> {
    let face = topo.face(face_id)?;
    if !face.inner_wires().is_empty() {
        return Ok(None);
    }
    let walk = topo.wire(face.outer_wire())?.edges().to_vec();
    let area = traversal_vector_area(topo, &walk)?;
    Ok((area.length() > 0.0).then_some(area))
}

/// Try to chain edges into closed loops using vertex adjacency.
///
/// Works when edges already share vertices (e.g., projected polygon edges
/// for sphere faces). Returns `None` if edges can't form closed loops.
///
/// `source_sense`, when present and when exactly one loop is reconstructed,
/// fixes that loop's traversal direction to agree with the source face's own
/// (see the call site). Without it the direction falls out of the walk's
/// arbitrary start edge, which on a closed surface silently selects the wrong
/// half of the surface.
fn try_direct_chain(
    topo: &mut Topology,
    edges: &[EdgeId],
    source_sense: Option<Vec3>,
) -> Result<Option<Vec<WireId>>, OffsetError> {
    let edge_info: Vec<(EdgeId, VertexId, VertexId)> = edges
        .iter()
        .map(|&eid| {
            let edge = topo.edge(eid)?;
            Ok((eid, edge.start(), edge.end()))
        })
        .collect::<Result<Vec<_>, OffsetError>>()?;

    let mut adjacency: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
    for (list_idx, &(_, start, end)) in edge_info.iter().enumerate() {
        if start == end {
            continue;
        }
        adjacency
            .entry(start.index())
            .or_default()
            .push((end.index(), list_idx));
        adjacency
            .entry(end.index())
            .or_default()
            .push((start.index(), list_idx));
    }

    // Every vertex must have exactly 2 incident edges for simple closed loops.
    for neighbors in adjacency.values() {
        if neighbors.len() != 2 {
            return Ok(None);
        }
    }

    if adjacency.is_empty() {
        return Ok(None);
    }

    let mut visited: HashSet<usize> = HashSet::new();
    let mut all_loops: Vec<Vec<OrientedEdge>> = Vec::new();

    for &(_, start, end) in &edge_info {
        if start == end {
            continue;
        }
        let start_idx = edge_info
            .iter()
            .enumerate()
            .find(|(i, (_, s, e))| *s == start && *e == end && !visited.contains(i))
            .map(|(i, _)| i)
            .unwrap_or(usize::MAX);
        if start_idx == usize::MAX {
            continue;
        }

        let start_vertex = start.index();
        let mut current = start_vertex;
        let mut loop_edges: Vec<OrientedEdge> = Vec::new();

        loop {
            let neighbors = match adjacency.get(&current) {
                Some(n) => n,
                None => return Ok(None),
            };

            let next = neighbors.iter().find(|(_, idx)| !visited.contains(idx));
            let Some(&(next_vertex, list_idx)) = next else {
                return Ok(None);
            };

            visited.insert(list_idx);

            let (eid, si, _) = edge_info[list_idx];
            let is_forward = si.index() == current;
            loop_edges.push(OrientedEdge::new(eid, is_forward));

            current = next_vertex;
            if current == start_vertex {
                break;
            }
        }

        all_loops.push(loop_edges);
    }

    // All non-closed edges must be consumed.
    let non_closed = edge_info.iter().filter(|(_, s, e)| s != e).count();
    if visited.len() != non_closed {
        return Ok(None);
    }

    // One reconstructed loop and a known source sense: make the walk agree
    // with it. With several loops the outer/inner pairing is ambiguous, so
    // leave them as walked.
    if all_loops.len() == 1
        && let Some(sense) = source_sense
        && let Some(loop_edges) = all_loops.first_mut()
        && traversal_vector_area(topo, loop_edges)?.dot(sense) < 0.0
    {
        loop_edges.reverse();
        for oriented in loop_edges.iter_mut() {
            *oriented = OrientedEdge::new(oriented.edge(), !oriented.is_forward());
        }
    }

    let mut wire_ids = Vec::new();
    for loop_edges in all_loops {
        let wire = Wire::new(loop_edges, true)?;
        wire_ids.push(topo.add_wire(wire));
    }

    if wire_ids.is_empty() {
        Ok(None)
    } else {
        Ok(Some(wire_ids))
    }
}

/// Build wire loops using the original line-intersection approach.
///
/// Collects intersection line segments, finds corners via pairwise
/// line-line intersection, creates trimmed edges, and walks loops.
#[allow(clippy::too_many_lines)]
fn build_loops_via_line_intersection(
    topo: &mut Topology,
    data: &OffsetData,
    face_id: FaceId,
    corner_cache: &mut VertexCache,
    edge_cache: &mut HashMap<(usize, usize), EdgeId>,
) -> Result<LoopBuild, OffsetError> {
    let mut line_segs: Vec<LineSeg> = Vec::new();

    for intersection in &data.intersections {
        if intersection.face_a != face_id && intersection.face_b != face_id {
            continue;
        }
        for &eid in &intersection.new_edges {
            let edge = topo.edge(eid)?;
            let p0 = topo.vertex(edge.start())?.point();
            let p1 = topo.vertex(edge.end())?.point();
            line_segs.push(LineSeg {
                p0,
                p1,
                boundary_edge: None,
            });
        }
    }

    if let Some(boundary) = data.boundary_edges.get(&face_id)
        && let Some(offset_face) = data.offset_faces.get(&face_id)
    {
        for &eid in boundary {
            let edge = topo.edge(eid)?;
            let orig_p0 = topo.vertex(edge.start())?.point();
            let orig_p1 = topo.vertex(edge.end())?.point();
            let (p0, p1) = project_boundary_edge(orig_p0, orig_p1, &offset_face.surface);
            line_segs.push(LineSeg {
                p0,
                p1,
                boundary_edge: Some(eid),
            });
        }
    }

    if line_segs.is_empty() {
        return Ok((Vec::new(), Vec::new()));
    }

    let tol = data.options.tolerance.linear;
    let mut corners_on_line: Vec<Vec<(VertexId, f64)>> = vec![Vec::new(); line_segs.len()];

    for i in 0..line_segs.len() {
        for j in (i + 1)..line_segs.len() {
            if let Some((pt, ti, tj)) = line_line_closest_point(&line_segs[i], &line_segs[j], tol) {
                let vid = find_or_create_vertex(topo, corner_cache, pt, tol);
                corners_on_line[i].push((vid, ti));
                corners_on_line[j].push((vid, tj));
            }
        }
    }

    let mut trimmed_edges: Vec<EdgeId> = Vec::new();
    let mut boundary_edges = Vec::new();
    for (line_index, corners) in corners_on_line.iter_mut().enumerate() {
        if corners.len() < 2 {
            continue;
        }
        corners.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        for pair in corners.windows(2) {
            let v_start = pair[0].0;
            let v_end = pair[1].0;
            if v_start == v_end {
                continue;
            }
            let key = if v_start.index() < v_end.index() {
                (v_start.index(), v_end.index())
            } else {
                (v_end.index(), v_start.index())
            };
            let eid = if let Some(&existing) = edge_cache.get(&key) {
                existing
            } else {
                let created = topo.add_edge(Edge::new(v_start, v_end, EdgeCurve::Line));
                edge_cache.insert(key, created);
                created
            };
            trimmed_edges.push(eid);
            if let Some(original_edge) = line_segs[line_index].boundary_edge {
                boundary_edges.push((original_edge, eid));
            }
        }
    }

    if trimmed_edges.is_empty() {
        return Ok((Vec::new(), Vec::new()));
    }

    let edge_info: Vec<(EdgeId, usize, usize)> = trimmed_edges
        .iter()
        .map(|&eid| {
            let edge = topo.edge(eid)?;
            Ok((eid, edge.start().index(), edge.end().index()))
        })
        .collect::<Result<Vec<_>, OffsetError>>()?;

    let mut adjacency: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
    for (list_idx, &(_, si, ei)) in edge_info.iter().enumerate() {
        adjacency.entry(si).or_default().push((ei, list_idx));
        adjacency.entry(ei).or_default().push((si, list_idx));
    }

    let mut visited: HashSet<usize> = HashSet::new();
    let mut all_loops: Vec<Vec<OrientedEdge>> = Vec::new();

    for (start_idx, &(_, start_si, _)) in edge_info.iter().enumerate() {
        if visited.contains(&start_idx) {
            continue;
        }

        let start_vertex = start_si;
        let mut current = start_vertex;
        let mut loop_edges: Vec<OrientedEdge> = Vec::new();

        loop {
            let neighbors = adjacency
                .get(&current)
                .ok_or_else(|| OffsetError::AssemblyFailed {
                    reason: format!("wire loop walk: vertex index {current} not in adjacency"),
                })?;

            let next = neighbors.iter().find(|(_, idx)| !visited.contains(idx));
            let Some(&(next_vertex, list_idx)) = next else {
                return Err(OffsetError::AssemblyFailed {
                    reason: format!(
                        "wire loop walk: no unvisited edge from vertex {current} \
                         ({} visited, {} in loop)",
                        visited.len(),
                        loop_edges.len()
                    ),
                });
            };

            visited.insert(list_idx);

            let (eid, si, _ei) = edge_info[list_idx];
            let is_forward = si == current;
            loop_edges.push(OrientedEdge::new(eid, is_forward));

            current = next_vertex;
            if current == start_vertex {
                break;
            }
        }

        all_loops.push(loop_edges);
    }

    let mut wire_ids = Vec::with_capacity(all_loops.len());
    // Wind against the STORED surface normal, never the effective one. A
    // face's reversal flag mirrors its normal and its edge traversal
    // together, so winding a loop to the flipped normal leaves the stored
    // winding disagreeing with the stored surface — and every consumer that
    // reads orientation off the wire (shell orientation in `assemble`, the
    // planar tessellator) then derives the opposite face from the one the
    // integrator sees. Thick solids used to negate here, which is what turned
    // their outer skin inside out.
    let surface_normal = match data.offset_faces.get(&face_id) {
        Some(offset_face) => match &offset_face.surface {
            FaceSurface::Plane { normal, .. } => *normal,
            _ => {
                return Err(OffsetError::AssemblyFailed {
                    reason: format!(
                        "line-intersection loop builder received non-planar face {}",
                        face_id.index()
                    ),
                });
            }
        },
        None => {
            return Err(OffsetError::AssemblyFailed {
                reason: format!("offset face {} is missing", face_id.index()),
            });
        }
    };
    for mut loop_edges in all_loops {
        orient_loop_to_normal(topo, &mut loop_edges, surface_normal)?;
        let wire = Wire::new(loop_edges, true)?;
        wire_ids.push(topo.add_wire(wire));
    }

    Ok((wire_ids, boundary_edges))
}

/// Orient a planar loop so its geometric winding agrees with `normal`.
fn orient_loop_to_normal(
    topo: &Topology,
    loop_edges: &mut [OrientedEdge],
    normal: remus_math::vec::Vec3,
) -> Result<(), OffsetError> {
    let points = loop_edges
        .iter()
        .map(|oriented| {
            let edge = topo.edge(oriented.edge())?;
            let vertex = if oriented.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            Ok(topo.vertex(vertex)?.point())
        })
        .collect::<Result<Vec<_>, OffsetError>>()?;

    // Newell's method is stable for convex and mildly non-convex planar
    // polygons and avoids selecting an arbitrary nearly-collinear triple.
    let mut winding = remus_math::vec::Vec3::new(0.0, 0.0, 0.0);
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        winding = remus_math::vec::Vec3::new(
            winding.x() + (current.y() - next.y()) * (current.z() + next.z()),
            winding.y() + (current.z() - next.z()) * (current.x() + next.x()),
            winding.z() + (current.x() - next.x()) * (current.y() + next.y()),
        );
    }
    if winding.dot(normal) < 0.0 {
        loop_edges.reverse();
        for oriented in loop_edges.iter_mut() {
            *oriented = OrientedEdge::new(oriented.edge(), !oriented.is_forward());
        }
    }
    Ok(())
}

/// Compute the closest-approach point of two infinite lines, each defined
/// by a `LineSeg`'s endpoints.
///
/// Returns `Some((point, t_a, t_b))` if the lines are not parallel and their
/// closest-approach distance is below a threshold. `t_a` and `t_b` are
/// parameters along each line (`0.0` = `p0`, `1.0` = `p1`).
fn line_line_closest_point(a: &LineSeg, b: &LineSeg, tol: f64) -> Option<(Point3, f64, f64)> {
    let da = pt_sub(a.p1, a.p0);
    let db = pt_sub(b.p1, b.p0);
    let w0 = pt_sub(a.p0, b.p0);

    let aa = dot3(da, da);
    let bb = dot3(db, db);
    let ab = dot3(da, db);
    let aw = dot3(da, w0);
    let bw = dot3(db, w0);

    // A zero-length segment has no direction to intersect along.
    if aa <= 0.0 || bb <= 0.0 {
        return None;
    }

    // `denom` is |da x db|^2, so it carries the fourth power of the model's
    // units. Comparing it against a fixed number is a statement about the
    // model's size, not its shape: at metre scale nothing trips it, while a
    // body a few microns across has every corner of every face rejected as
    // parallel and loses its wire loops entirely. Dividing by |da|^2|db|^2
    // leaves sin^2 of the angle between the lines, which is what the test
    // was always about, and the threshold then means the same angle at every
    // scale. The value matches the old one at unit scale.
    let denom = aa * bb - ab * ab;
    if denom.abs() < aa * bb * PARALLEL_SIN_SQ {
        return None;
    }

    let t = (ab * bw - bb * aw) / denom;
    let s = (aa * bw - ab * aw) / denom;

    let pa = Point3::new(
        a.p0.x() + t * da.0,
        a.p0.y() + t * da.1,
        a.p0.z() + t * da.2,
    );
    let pb = Point3::new(
        b.p0.x() + s * db.0,
        b.p0.y() + s * db.1,
        b.p0.z() + s * db.2,
    );

    let dx = pa.x() - pb.x();
    let dy = pa.y() - pb.y();
    let dz = pa.z() - pb.z();
    let dist_sq = dx * dx + dy * dy + dz * dz;

    if dist_sq > tol * tol {
        return None;
    }

    // When lines truly intersect (coplanar), `pa` and `pb` are the same
    // point up to floating-point rounding.  Use `pa` directly — computing
    // the point on line `a` from its own origin avoids mixing two
    // independent rounding chains (one per line).  This gives exact
    // corners for planar offset faces where all intersection lines are
    // coplanar by construction.
    Some((pa, t, s))
}

/// Subtract two points, returning a direction tuple.
fn pt_sub(a: Point3, b: Point3) -> (f64, f64, f64) {
    (a.x() - b.x(), a.y() - b.y(), a.z() - b.z())
}

/// Dot product of two 3-tuples.
fn dot3(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1 + a.2 * b.2
}

/// Project a boundary edge's endpoints onto an offset surface.
///
/// For planar surfaces, this projects the point onto the plane (translates
/// along the normal). For other surfaces, it returns the original points
/// (approximation — proper projection requires parametric solvers).
fn project_boundary_edge(
    p0: Point3,
    p1: Point3,
    surface: &remus_topology::face::FaceSurface,
) -> (Point3, Point3) {
    match surface {
        remus_topology::face::FaceSurface::Plane { normal, d } => {
            // Project each point onto the plane: p' = p + (d - n·p) * n
            let project = |p: Point3| -> Point3 {
                let n_dot_p = normal.x() * p.x() + normal.y() * p.y() + normal.z() * p.z();
                let dist = d - n_dot_p;
                Point3::new(
                    p.x() + dist * normal.x(),
                    p.y() + dist * normal.y(),
                    p.z() + dist * normal.z(),
                )
            };
            (project(p0), project(p1))
        }
        _ => {
            // Non-planar: return original positions as approximation.
            (p0, p1)
        }
    }
}

// Uses crate::data::find_or_create_vertex (shared helper).

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_topology::Topology;
    use remus_topology::solid::SolidId;

    use crate::data::{OffsetData, OffsetOptions};

    #[test]
    fn exact_ellipse_rims_form_cap_and_mixed_lateral_wires() {
        use remus_math::curves::{Circle3D, Ellipse3D};

        let mut topo = Topology::new();
        let normal = Vec3::new(0.6, 0.0, 0.8);
        let ellipse = Ellipse3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            normal,
            2.5,
            2.0,
            Vec3::new(-0.8, 0.0, 0.6),
        )
        .unwrap();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 5.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let cylinder = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0,
        )
        .unwrap();
        let ve = topo.add_vertex(Vertex::new(ellipse.evaluate(0.0), 1e-7));
        let vc = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let mut ellipse_edge =
            Edge::with_tolerance(ve, ve, EdgeCurve::Ellipse(ellipse), Some(1e-7));
        ellipse_edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let ellipse_edge = topo.add_edge(ellipse_edge);
        let mut circle_edge = Edge::with_tolerance(vc, vc, EdgeCurve::Circle(circle), Some(1e-7));
        circle_edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let circle_edge = topo.add_edge(circle_edge);

        let cap = try_closed_conic_seam_wire(&mut topo, &[ellipse_edge], None, Tolerance::new())
            .unwrap()
            .unwrap();
        assert_eq!(topo.wire(cap[0]).unwrap().edges().len(), 1);
        let lateral = try_closed_conic_seam_wire(
            &mut topo,
            &[ellipse_edge, circle_edge],
            Some(&FaceSurface::Cylinder(cylinder.clone())),
            Tolerance::new(),
        )
        .unwrap()
        .unwrap();
        let walk = topo.wire(lateral[0]).unwrap().edges();
        assert_eq!(walk.len(), 4);
        let a = topo
            .vertex(topo.edge(ellipse_edge).unwrap().start())
            .unwrap()
            .point();
        let b = topo
            .vertex(topo.edge(circle_edge).unwrap().start())
            .unwrap()
            .point();
        assert!((a - b).length() > 1.0);
        for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let point = a + (b - a) * fraction;
            let radial = point
                - cylinder.origin()
                - cylinder.axis() * (point - cylinder.origin()).dot(cylinder.axis());
            assert!((radial.length() - cylinder.radius()).abs() < 1e-7);
        }
        for pair in walk
            .iter()
            .zip(walk.iter().cycle().skip(1))
            .take(walk.len())
        {
            let current = topo.edge(pair.0.edge()).unwrap();
            let next = topo.edge(pair.1.edge()).unwrap();
            assert_eq!(pair.0.oriented_end(current), pair.1.oriented_start(next));
        }

        topo.edge_mut(ellipse_edge).unwrap().set_start(ve);
        topo.edge_mut(ellipse_edge).unwrap().set_end(ve);
        topo.edge_mut(ellipse_edge)
            .unwrap()
            .set_trim(Some((0.0, std::f64::consts::TAU)));
        topo.edge_mut(circle_edge).unwrap().set_start(vc);
        topo.edge_mut(circle_edge).unwrap().set_end(vc);
        topo.edge_mut(circle_edge)
            .unwrap()
            .set_trim(Some((0.0, std::f64::consts::TAU)));
        try_closed_conic_seam_wire(
            &mut topo,
            &[circle_edge, ellipse_edge],
            Some(&FaceSurface::Cylinder(cylinder.clone())),
            Tolerance::new(),
        )
        .unwrap()
        .unwrap();
        let a = topo
            .vertex(topo.edge(ellipse_edge).unwrap().start())
            .unwrap()
            .point();
        let b = topo
            .vertex(topo.edge(circle_edge).unwrap().start())
            .unwrap()
            .point();
        for fraction in [0.0, 0.5, 1.0] {
            let point = a + (b - a) * fraction;
            let radial = point
                - cylinder.origin()
                - cylinder.axis() * (point - cylinder.origin()).dot(cylinder.axis());
            assert!((radial.length() - cylinder.radius()).abs() < 1e-7);
        }
    }

    fn run_phases_1_to_7(topo: &mut Topology, solid: SolidId, distance: f64) -> OffsetData {
        let mut data = OffsetData::new(distance, OffsetOptions::default(), vec![]);
        crate::analyse::analyse_edges(topo, solid, &mut data).unwrap();
        crate::offset::build_offset_faces(topo, solid, &mut data).unwrap();
        crate::inter3d::intersect_faces_3d(topo, solid, &mut data).unwrap();
        crate::inter2d::intersect_pcurves_2d(topo, solid, &mut data).unwrap();
        build_wire_loops(topo, &mut data).unwrap();
        data
    }

    #[test]
    fn box_each_face_has_one_wire() {
        let mut topo = Topology::new();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let data = run_phases_1_to_7(&mut topo, solid, 0.5);
        assert_eq!(data.face_wires.len(), 6, "each face should have wire loops");
        for wires in data.face_wires.values() {
            assert_eq!(
                wires.len(),
                1,
                "each box face should have exactly 1 wire loop"
            );
        }
    }

    #[test]
    fn box_wires_have_4_edges() {
        let mut topo = Topology::new();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let data = run_phases_1_to_7(&mut topo, solid, 0.5);
        for (&face_id, wires) in &data.face_wires {
            for &wire_id in wires {
                let wire = topo.wire(wire_id).unwrap();
                assert_eq!(
                    wire.edges().len(),
                    4,
                    "box face {face_id:?} wire should have 4 edges, got {}",
                    wire.edges().len()
                );
            }
        }
    }

    #[test]
    fn box_wires_are_closed() {
        let mut topo = Topology::new();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let data = run_phases_1_to_7(&mut topo, solid, 0.5);
        for wires in data.face_wires.values() {
            for &wire_id in wires {
                let wire = topo.wire(wire_id).unwrap();
                assert!(wire.is_closed(), "wire should be closed");
            }
        }
    }

    #[test]
    fn box_wire_edges_chain_correctly() {
        let mut topo = Topology::new();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let data = run_phases_1_to_7(&mut topo, solid, 0.5);
        for wires in data.face_wires.values() {
            for &wire_id in wires {
                let wire = topo.wire(wire_id).unwrap();
                let edges = wire.edges();
                for i in 0..edges.len() {
                    let curr = &edges[i];
                    let next = &edges[(i + 1) % edges.len()];
                    let curr_edge = topo.edge(curr.edge()).unwrap();
                    let next_edge = topo.edge(next.edge()).unwrap();
                    let curr_end = curr.oriented_end(curr_edge);
                    let next_start = next.oriented_start(next_edge);
                    assert_eq!(curr_end, next_start, "wire edge chain broken at index {i}");
                }
            }
        }
    }

    #[test]
    fn cylinder_each_face_has_one_wire() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_cylinder(&mut topo, 2.0, 5.0).unwrap();
        let data = run_phases_1_to_7(&mut topo, solid, 0.5);
        assert_eq!(
            data.face_wires.len(),
            3,
            "cylinder has 3 faces, each should get a wire loop"
        );
    }

    #[test]
    fn sphere_each_face_has_one_wire() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_sphere(&mut topo, 3.0, 16).unwrap();
        let data = run_phases_1_to_7(&mut topo, solid, 0.5);
        assert_eq!(
            data.face_wires.len(),
            2,
            "sphere has 2 faces, each should get a wire loop"
        );
    }
}
