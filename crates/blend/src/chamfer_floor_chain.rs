//! One exact chamfer along runs of edges round a planar floor face.
//!
//! The bottom outline of a block with a round boss on its corner and concave
//! seam rounds between them is one smooth run of edges — straight block
//! edges, the seam rounds' concave feet and the boss's foot circle — and
//! every one of them lies on the bottom face, each between it and a
//! different wall. A per-edge chamfer cannot build that: every stripe trims
//! the shared bottom face in turn, and a stripe running out at a tangent
//! vertex has no end face to close against.
//!
//! This assembler builds the whole selection at once. Each edge's band is
//! the exact analytic chamfer of its own face pair — a plane along a straight
//! edge, a cone along a circular one — and consecutive bands meet in a joint:
//!
//! * where the outline is tangent, both bands end on the same cross-section
//!   (the plane is tangent to the cone along it), so they share its straight
//!   generator and its two points: one inside the floor, one on the tangent
//!   seam between the two walls;
//! * where the outline turns a sharp corner between two straight edges whose
//!   walls are planes meeting in a straight edge, the two bands carry on until
//!   they meet in a mitre: the floor contacts cross inside the floor and the
//!   wall contacts on the corner edge.
//!
//! A run that does not close round the floor ends at a sharp vertex whose
//! third face is a plane square to the edge; that face is notched along the
//! band's end cross-section, as a single straight chamfer's end is.
//!
//! Every boundary vertex the result needs is made first, by splitting the
//! body's edges, and each face is then cut exactly once along all of its
//! contacts: the floor along the whole selection, each wall along its own
//! run. Consecutive edges on one wall (a circle split into arcs) share one
//! band. Every surface and every edge is exact: planes, cones, lines and
//! circles.

use std::f64::consts::TAU;

use remus_math::curves::Circle3D;
use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::Solid;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::body_edit::EditScope;
use crate::builder_utils::add_certified_curve_edge;
use crate::rail_chain::{effective_use, reversed_loop};
use crate::trimmer::{self, CutPiece};
use crate::{BlendEngine, BlendError, BlendFaceOrigins, BlendResult};

/// Tolerance of the vertices this assembler creates.
const VERTEX_TOL: f64 = 1e-7;

/// One band of the chamfer: consecutive selected edges on one wall.
struct Element {
    /// The selected edges the band runs along, in run order.
    edges: Vec<EdgeId>,
    /// The face across the edges from the floor.
    wall: FaceId,
    /// The band's exact surface.
    surface: FaceSurface,
    /// For a circular run: the circle's centre on the floor and its normal,
    /// oriented so the run is counter-clockwise about it, and the swept angle.
    arc: Option<(Point3, Vec3, f64)>,
    /// Floor contact at the band's start and end cross-sections.
    floor: [Point3; 2],
    /// Wall contact at the band's start and end cross-sections.
    wall_points: [Point3; 2],
}

/// Where two consecutive bands meet.
#[derive(Clone, Copy)]
enum JointKind {
    /// Both bands end on one cross-section.
    Tangent,
    /// Two straight bands carried on to a mitre: the floor contacts cross at
    /// `floor`, the wall contacts at `wall`.
    Mitre { floor: Point3, wall: Point3 },
}

/// The straight edge across the band at a joint or a run end.
#[derive(Clone, Copy)]
struct Cross {
    floor: VertexId,
    wall: VertexId,
    edge: EdgeId,
}

/// A maximal run of consecutive selected edges round the floor's boundary.
struct Run {
    elements: Vec<Element>,
    /// The run is the floor's whole outer boundary.
    closed: bool,
}

/// Chamfer `edges` by `distance` along both faces of each, as one assembly
/// round the planar face they all share, or `None` when the selection is not
/// such a set of runs.
///
/// Only a selection holding a circular edge and two consecutive edges of the
/// floor's boundary qualifies, so selections the per-edge builder already
/// handles keep their outcome.
///
/// # Errors
///
/// Returns [`BlendError`] when the selection qualifies but a boundary split,
/// a cut or the closing assembly fails.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub fn chamfer(
    topo: &mut Topology,
    adjacency: &AdjacencyIndex,
    edges: &[EdgeId],
    distance: f64,
    scope: &mut EditScope,
    original_faces: &[FaceId],
    inner_shells: &[ShellId],
) -> Result<Option<BlendResult>, BlendError> {
    let Some((floor, runs)) = detect(topo, adjacency, edges, distance)? else {
        return Ok(None);
    };
    let touch = 1e-6 * distance.max(1.0);
    let mut replacements: DetHashMap<FaceId, FaceId> = DetHashMap::default();
    let current = |replacements: &DetHashMap<FaceId, FaceId>, face: FaceId| {
        replacements.get(&face).copied().unwrap_or(face)
    };
    let floor_failure = || BlendError::TrimmingFailure { face: floor };

    // ── Every boundary vertex first: the cuts below read current wires. ──
    // Per run: the cross at each joint (index k joins element k to k + 1,
    // cyclically on a closed run) and, on an open run, at its two ends.
    let mut joint_crosses: Vec<Vec<Cross>> = Vec::with_capacity(runs.len());
    let mut end_crosses: Vec<Option<[Cross; 2]>> = Vec::with_capacity(runs.len());
    let mut joint_kinds: Vec<Vec<JointKind>> = Vec::with_capacity(runs.len());
    for run in &runs {
        let count = run.elements.len();
        let joints = if run.closed { count } else { count - 1 };
        let mut kinds = Vec::with_capacity(joints);
        for k in 0..joints {
            let (a, b) = (&run.elements[k], &run.elements[(k + 1) % count]);
            let Some(kind) = joint_kind(topo, adjacency, floor, a, b, touch)? else {
                return Ok(None);
            };
            kinds.push(kind);
        }
        joint_kinds.push(kinds);
        // An open run ends on a planar face square to the band there, which
        // its end cross-section notches.
        if !run.closed {
            for (element, side) in [(&run.elements[0], 0), (&run.elements[count - 1], 1)] {
                let corner = run_vertex(topo, element, side == 1)?;
                if !square_end_face(topo, original_faces, floor, element, side, corner, touch)? {
                    return Ok(None);
                }
            }
        }
    }
    for (run, kinds) in runs.iter().zip(&joint_kinds) {
        let count = run.elements.len();
        let mut crosses = Vec::with_capacity(kinds.len());
        for (k, kind) in kinds.iter().enumerate() {
            let a = &run.elements[k];
            let (floor_point, wall_point) = match *kind {
                JointKind::Tangent => (a.floor[1], a.wall_points[1]),
                JointKind::Mitre { floor, wall } => (floor, wall),
            };
            let floor_vertex = topo.add_vertex(Vertex::new(floor_point, VERTEX_TOL));
            // The point is on the seam or corner edge this wall shares with
            // the next band's, so the split gives both walls the vertex.
            let wall_vertex = {
                let mut edit = scope.edit(&mut replacements);
                let face = current(edit.replacements(), a.wall);
                trimmer::split_body_boundary_at(topo, &mut edit, face, wall_point, touch)?
                    .ok_or(BlendError::TrimmingFailure { face: a.wall })?
            };
            let edge = topo.add_edge(Edge::new(floor_vertex, wall_vertex, EdgeCurve::Line));
            crosses.push(Cross {
                floor: floor_vertex,
                wall: wall_vertex,
                edge,
            });
        }
        joint_crosses.push(crosses);
        if run.closed {
            end_crosses.push(None);
            continue;
        }
        let first = &run.elements[0];
        let last = &run.elements[count - 1];
        let mut ends = Vec::with_capacity(2);
        for (element, side) in [(first, 0), (last, 1)] {
            let mut edit = scope.edit(&mut replacements);
            let floor_face = current(edit.replacements(), floor);
            let floor_vertex = trimmer::split_body_boundary_at(
                topo,
                &mut edit,
                floor_face,
                element.floor[side],
                touch,
            )?
            .ok_or_else(floor_failure)?;
            let wall_face = current(edit.replacements(), element.wall);
            let wall_vertex = trimmer::split_body_boundary_at(
                topo,
                &mut edit,
                wall_face,
                element.wall_points[side],
                touch,
            )?
            .ok_or(BlendError::TrimmingFailure { face: element.wall })?;
            let edge = topo.add_edge(Edge::new(floor_vertex, wall_vertex, EdgeCurve::Line));
            ends.push(Cross {
                floor: floor_vertex,
                wall: wall_vertex,
                edge,
            });
        }
        end_crosses.push(Some([ends[0], ends[1]]));
    }

    // ── Contact edges, per element, between the crosses at its two sides. ──
    // (start cross, end cross) of element k of run r.
    let side_crosses = |r: usize, k: usize| -> (Cross, Cross) {
        let run = &runs[r];
        let count = run.elements.len();
        let start = if k == 0 {
            match end_crosses[r] {
                Some(ends) => ends[0],
                None => joint_crosses[r][count - 1],
            }
        } else {
            joint_crosses[r][k - 1]
        };
        let end = if k + 1 == count {
            match end_crosses[r] {
                Some(ends) => ends[1],
                None => joint_crosses[r][count - 1],
            }
        } else {
            joint_crosses[r][k]
        };
        (start, end)
    };
    // (floor contact, wall contact), each running in run order.
    let mut contacts: Vec<Vec<(EdgeId, EdgeId)>> = Vec::with_capacity(runs.len());
    for (r, run) in runs.iter().enumerate() {
        let mut run_contacts = Vec::with_capacity(run.elements.len());
        for (k, element) in run.elements.iter().enumerate() {
            let (start, end) = side_crosses(r, k);
            let floor_edge = contact_edge(topo, element, start.floor, end.floor)?;
            let wall_edge = contact_edge(topo, element, start.wall, end.wall)?;
            run_contacts.push((floor_edge, wall_edge));
        }
        contacts.push(run_contacts);
    }

    // ── Cut each face once. ──
    let selected: Vec<EdgeId> = runs
        .iter()
        .flat_map(|run| run.elements.iter().flat_map(|e| e.edges.iter().copied()))
        .collect();
    // The floor, along every run.
    {
        let face = current(&replacements, floor);
        let data = topo.face(face)?;
        let (surface, reversed, inner) = (
            data.surface().clone(),
            data.is_reversed(),
            data.inner_wires().to_vec(),
        );
        let wire = topo.wire(data.outer_wire())?.edges().to_vec();
        let trimmed = if let [run] = runs.as_slice()
            && run.closed
        {
            let loop_edges: Vec<OrientedEdge> = contacts[0]
                .iter()
                .map(|&(floor_edge, _)| OrientedEdge::new(floor_edge, true))
                .collect();
            let wire_id = topo.add_wire(Wire::new(loop_edges, true)?);
            let mut trimmed = Face::new(wire_id, inner, surface);
            trimmed.set_reversed(reversed);
            topo.add_face(trimmed)
        } else {
            let mut pieces = Vec::with_capacity(runs.len());
            for (r, run) in runs.iter().enumerate() {
                let Some(ends) = end_crosses[r] else {
                    return Err(floor_failure());
                };
                let (Some(from), Some(to)) = (
                    trimmer::vertex_occurrence(
                        topo,
                        face,
                        &wire,
                        ends[0].floor,
                        run.elements[0].floor[1],
                    )?,
                    trimmer::vertex_occurrence(
                        topo,
                        face,
                        &wire,
                        ends[1].floor,
                        run.elements[run.elements.len() - 1].floor[0],
                    )?,
                ) else {
                    return Err(floor_failure());
                };
                pieces.push(CutPiece {
                    from,
                    to,
                    edges: contacts[r]
                        .iter()
                        .map(|&(floor_edge, _)| OrientedEdge::new(floor_edge, true))
                        .collect(),
                });
            }
            trimmer::cut_face(topo, face, &wire, &pieces, &selected)?
        };
        replacements.insert(floor, trimmed);
    }
    // Each wall, along its own bands.
    let mut walls: Vec<FaceId> = Vec::new();
    for run in &runs {
        for element in &run.elements {
            if !walls.contains(&element.wall) {
                walls.push(element.wall);
            }
        }
    }
    for &wall in &walls {
        let face = current(&replacements, wall);
        let wire = topo.wire(topo.face(face)?.outer_wire())?.edges().to_vec();
        let failure = BlendError::TrimmingFailure { face: wall };
        let mut pieces = Vec::new();
        for (r, run) in runs.iter().enumerate() {
            for (k, element) in run.elements.iter().enumerate() {
                if element.wall != wall {
                    continue;
                }
                let (start, end) = side_crosses(r, k);
                let (Some(from), Some(to)) = (
                    trimmer::vertex_occurrence(
                        topo,
                        face,
                        &wire,
                        start.wall,
                        element.wall_points[1],
                    )?,
                    trimmer::vertex_occurrence(
                        topo,
                        face,
                        &wire,
                        end.wall,
                        element.wall_points[0],
                    )?,
                ) else {
                    return Err(failure);
                };
                pieces.push(CutPiece {
                    from,
                    to,
                    edges: vec![OrientedEdge::new(contacts[r][k].1, true)],
                });
            }
        }
        let trimmed = trimmer::cut_face(topo, face, &wire, &pieces, &selected)?;
        replacements.insert(wall, trimmed);
    }
    // The end faces of open runs, notched along the end cross-sections.
    for (r, run) in runs.iter().enumerate() {
        let Some(ends) = end_crosses[r] else {
            continue;
        };
        let count = run.elements.len();
        let corners = [
            run_vertex(topo, &run.elements[0], false)?,
            run_vertex(topo, &run.elements[count - 1], true)?,
        ];
        for (cross, corner) in ends.into_iter().zip(corners) {
            let corner_point = topo.vertex(corner)?.point();
            let floor_point = topo.vertex(cross.floor)?.point();
            let wall_point = topo.vertex(cross.wall)?.point();
            let axis = (floor_point - corner_point)
                .cross(wall_point - corner_point)
                .normalize()?;
            let mut notched = false;
            for &original in original_faces {
                let face = current(&replacements, original);
                if let Some(cap) = crate::chamfer_extrusion::notch_cap(
                    topo,
                    face,
                    (cross.edge, cross.floor, cross.wall),
                    corner,
                    axis,
                )? {
                    replacements.insert(original, cap);
                    notched = true;
                    break;
                }
            }
            if !notched {
                return Err(BlendError::TrimmingFailure { face: floor });
            }
        }
    }

    // ── The bands. ──
    let floor_face = current(&replacements, floor);
    let mut created: Vec<(FaceId, Vec<FaceId>)> = Vec::new();
    for (r, run) in runs.iter().enumerate() {
        for (k, element) in run.elements.iter().enumerate() {
            let (start, end) = side_crosses(r, k);
            let (floor_edge, wall_edge) = contacts[r][k];
            // Along the floor contact in run order, across the end, back
            // along the wall contact and across the start.
            let mut loop_edges = vec![
                OrientedEdge::new(floor_edge, true),
                OrientedEdge::new(end.edge, topo.edge(end.edge)?.start() == end.floor),
                OrientedEdge::new(wall_edge, false),
                OrientedEdge::new(start.edge, topo.edge(start.edge)?.start() == start.wall),
            ];
            // Each contact runs against the trimmed face's use of it.
            match effective_use(topo, floor_face, floor_edge)? {
                Some(true) => loop_edges = reversed_loop(&loop_edges),
                Some(false) => {}
                None => return Err(BlendError::TrimmingFailure { face: floor }),
            }
            let reversed = band_reversed(
                topo,
                floor_face,
                current(&replacements, element.wall),
                element,
            )?;
            let stored = if reversed {
                reversed_loop(&loop_edges)
            } else {
                loop_edges
            };
            let wire = topo.add_wire(Wire::new(stored, true)?);
            let mut band = Face::new(wire, Vec::new(), element.surface.clone());
            band.set_reversed(reversed);
            let band = topo.add_face(band);
            let wall_use = effective_use(topo, current(&replacements, element.wall), wall_edge)?;
            let band_use = effective_use(topo, band, wall_edge)?;
            if wall_use.is_none() || wall_use == band_use {
                return Err(BlendError::TrimmingFailure { face: element.wall });
            }
            created.push((band, vec![floor, element.wall]));
        }
    }

    let mut result_faces: Vec<FaceId> = original_faces
        .iter()
        .map(|&face| current(&replacements, face))
        .collect();
    result_faces.extend(created.iter().map(|(face, _)| *face));
    let shell = Shell::new(result_faces)?;
    if remus_topology::validation::validate_shell_closed(&shell, topo).is_err()
        || remus_topology::validation::validate_shell_manifold(&shell, topo).is_err()
    {
        return Err(BlendError::TrimmingFailure { face: floor });
    }
    let shell = topo.add_shell(shell);
    let solid = topo.add_solid(Solid::new(shell, inner_shells.to_vec()));
    log::debug!(
        "chamfer: floor chain of {} band(s) round face {floor:?}",
        created.len()
    );
    Ok(Some(BlendResult {
        solid,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: Some(BlendFaceOrigins {
            survived: original_faces
                .iter()
                .map(|&face| (face, current(&replacements, face)))
                .collect(),
            deleted: Vec::new(),
            created,
            created_unattributed: Vec::new(),
        }),
        engine: BlendEngine::Walking,
    }))
}

/// The floor and the runs of the selection round it, or `None` when the
/// selection is not one this assembler builds.
fn detect(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    edges: &[EdgeId],
    distance: f64,
) -> Result<Option<(FaceId, Vec<Run>)>, BlendError> {
    if edges.len() < 2 {
        return Ok(None);
    }
    let mut circular = false;
    for &edge in edges {
        match topo.edge(edge)?.curve() {
            EdgeCurve::Line => {}
            EdgeCurve::Circle(_) => circular = true,
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Ok(None),
        }
        if topo.edge(edge)?.is_closed() {
            return Ok(None);
        }
    }
    if !circular {
        return Ok(None);
    }
    // The one planar face every edge lies on.
    let pair = |edge: EdgeId| -> Option<[FaceId; 2]> {
        match adjacency.faces_for_edge(edge) {
            &[a, b] if a != b => Some([a, b]),
            _ => None,
        }
    };
    let Some(first) = pair(edges[0]) else {
        return Ok(None);
    };
    let mut common = Vec::new();
    for face in first {
        let mut on_all = true;
        for &edge in edges {
            if !pair(edge).is_some_and(|p| p.contains(&face)) {
                on_all = false;
                break;
            }
        }
        if on_all && matches!(topo.face(face)?.surface(), FaceSurface::Plane { .. }) {
            common.push(face);
        }
    }
    let &[floor] = common.as_slice() else {
        return Ok(None);
    };

    // Runs of consecutive selected edges round the floor's outer boundary.
    let wire = topo.wire(topo.face(floor)?.outer_wire())?.edges().to_vec();
    let n = wire.len();
    let chosen: Vec<bool> = wire.iter().map(|oe| edges.contains(&oe.edge())).collect();
    if chosen.iter().filter(|&&c| c).count() != edges.len() {
        return Ok(None);
    }
    let closed = chosen.iter().all(|&c| c);
    let mut raw_runs: Vec<Vec<OrientedEdge>> = Vec::new();
    if closed {
        raw_runs.push(wire);
    } else {
        let Some(start) = (0..n).find(|&i| !chosen[i]) else {
            return Ok(None);
        };
        let mut current: Vec<OrientedEdge> = Vec::new();
        for step in 1..=n {
            let i = (start + step) % n;
            if chosen[i] {
                current.push(wire[i]);
            } else if !current.is_empty() {
                raw_runs.push(std::mem::take(&mut current));
            }
        }
        if !current.is_empty() {
            raw_runs.push(current);
        }
    }
    if !raw_runs.iter().any(|run| run.len() >= 2) {
        return Ok(None);
    }

    let mut runs = Vec::with_capacity(raw_runs.len());
    for raw in raw_runs {
        let mut elements: Vec<Element> = Vec::with_capacity(raw.len());
        for oriented in raw {
            let Some(element) = element_for(topo, adjacency, floor, oriented, distance)? else {
                return Ok(None);
            };
            match elements.last_mut() {
                Some(last) if mergeable(last, &element, distance) => merge(last, element),
                _ => elements.push(element),
            }
        }
        if closed && elements.len() >= 2 {
            let count = elements.len();
            if mergeable(&elements[count - 1], &elements[0], distance) {
                let first = elements.remove(0);
                let last = elements
                    .last_mut()
                    .ok_or(BlendError::TrimmingFailure { face: floor })?;
                merge(last, first);
            }
        }
        if closed && elements.len() < 2 {
            // A whole rim on one wall is the closed-rim assembler's.
            return Ok(None);
        }
        runs.push(Run { elements, closed });
    }
    Ok(Some((floor, runs)))
}

/// The band along one floor edge, oriented in run order.
fn element_for(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    floor: FaceId,
    oriented: OrientedEdge,
    distance: f64,
) -> Result<Option<Element>, BlendError> {
    let edge = oriented.edge();
    let Ok(result) =
        crate::chamfer_builder::compute_chamfer_stripe(topo, adjacency, edge, distance, distance)
    else {
        return Ok(None);
    };
    let stripe = result.stripe;
    if !matches!(
        stripe.surface,
        FaceSurface::Plane { .. } | FaceSurface::Cone(_)
    ) {
        return Ok(None);
    }
    let (wall, floor_first) = if stripe.face1 == floor {
        (stripe.face2, true)
    } else if stripe.face2 == floor {
        (stripe.face1, false)
    } else {
        return Ok(None);
    };
    let [start, end] = match stripe.sections.as_slice() {
        [start, .., end] => [start, end],
        _ => return Ok(None),
    };
    let pick = |section: &crate::section::CircSection| {
        if floor_first {
            (section.p1, section.p2)
        } else {
            (section.p2, section.p1)
        }
    };
    // Sections run from the edge's start to its end.
    let (mut f0, mut w0) = pick(start);
    let (mut f1, mut w1) = pick(end);
    let data = topo.edge(edge)?;
    let arc = match data.curve() {
        EdgeCurve::Line => {
            if !matches!(stripe.surface, FaceSurface::Plane { .. }) {
                return Ok(None);
            }
            None
        }
        EdgeCurve::Circle(circle) => {
            if !matches!(stripe.surface, FaceSurface::Cone(_)) {
                return Ok(None);
            }
            let (t0, t1) = data
                .strict_domain()
                .map_err(|error| BlendError::InvalidInput {
                    reason: error.to_string(),
                })?;
            let sweep = (t1 - t0).abs();
            if !(sweep > 0.0 && sweep < TAU) {
                return Ok(None);
            }
            // The edge runs counter-clockwise about its circle's normal from
            // its start; the run follows the floor's use of it.
            let normal = if oriented.is_forward() == (t1 > t0) {
                circle.normal()
            } else {
                -circle.normal()
            };
            Some((circle.center(), normal, sweep))
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => return Ok(None),
    };
    if !oriented.is_forward() {
        std::mem::swap(&mut f0, &mut f1);
        std::mem::swap(&mut w0, &mut w1);
    }
    // A degenerate band (setback beyond the edge's own reach) is not ours.
    if (f0 - w0).length() < 1e-9 * distance.max(1.0) {
        return Ok(None);
    }
    Ok(Some(Element {
        edges: vec![edge],
        wall,
        surface: stripe.surface,
        arc,
        floor: [f0, f1],
        wall_points: [w0, w1],
    }))
}

/// Whether `next` continues `last`'s band: the same wall, the same carrier
/// and a shared cross-section.
fn mergeable(last: &Element, next: &Element, distance: f64) -> bool {
    let touch = 1e-6 * distance.max(1.0);
    if last.wall != next.wall {
        return false;
    }
    if (last.floor[1] - next.floor[0]).length() > touch
        || (last.wall_points[1] - next.wall_points[0]).length() > touch
    {
        return false;
    }
    match (last.arc, next.arc) {
        (Some((c0, n0, _)), Some((c1, n1, _))) => {
            (c0 - c1).length() <= touch && n0.dot(n1) > 1.0 - 1e-9
        }
        (None, None) => true,
        _ => false,
    }
}

fn merge(last: &mut Element, next: Element) {
    last.edges.extend(next.edges);
    last.floor[1] = next.floor[1];
    last.wall_points[1] = next.wall_points[1];
    if let (Some((_, _, sweep)), Some((_, _, more))) = (&mut last.arc, next.arc) {
        *sweep += more;
    }
}

/// How bands `a` and `b` meet, or `None` when they meet in a way this
/// assembler does not build.
fn joint_kind(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    floor: FaceId,
    a: &Element,
    b: &Element,
    touch: f64,
) -> Result<Option<JointKind>, BlendError> {
    if a.wall == b.wall {
        return Ok(None);
    }
    if (a.floor[1] - b.floor[0]).length() <= touch
        && (a.wall_points[1] - b.wall_points[0]).length() <= touch
    {
        return Ok(Some(JointKind::Tangent));
    }
    // A mitre between two straight bands whose planar walls meet in a
    // straight corner edge at the shared vertex.
    if a.arc.is_some() || b.arc.is_some() {
        return Ok(None);
    }
    for wall in [a.wall, b.wall] {
        if !matches!(topo.face(wall)?.surface(), FaceSurface::Plane { .. }) {
            return Ok(None);
        }
    }
    let FaceSurface::Plane {
        normal: floor_normal,
        ..
    } = *topo.face(floor)?.surface()
    else {
        return Ok(None);
    };
    let corner = run_vertex(topo, a, true)?;
    let wall_wire = topo.wire(topo.face(a.wall)?.outer_wire())?;
    let Some(corner_edge) = wall_wire
        .edges()
        .iter()
        .map(OrientedEdge::edge)
        .find(|&edge| {
            adjacency.faces_for_edge(edge).contains(&b.wall)
                && topo.edge(edge).is_ok_and(|e| {
                    (e.start() == corner || e.end() == corner)
                        && matches!(e.curve(), EdgeCurve::Line)
                })
        })
    else {
        return Ok(None);
    };
    let Some(floor_point) = line_line(
        a.floor[0],
        a.floor[1] - a.floor[0],
        b.floor[0],
        b.floor[1] - b.floor[0],
        floor_normal,
    ) else {
        return Ok(None);
    };
    let corner_data = topo.edge(corner_edge)?;
    let c0 = topo.vertex(corner_data.start())?.point();
    let c1 = topo.vertex(corner_data.end())?.point();
    let (Some(on_a), Some(on_b)) = (
        closest_on_line(
            a.wall_points[0],
            a.wall_points[1] - a.wall_points[0],
            c0,
            c1 - c0,
            touch,
        ),
        closest_on_line(
            b.wall_points[0],
            b.wall_points[1] - b.wall_points[0],
            c0,
            c1 - c0,
            touch,
        ),
    ) else {
        return Ok(None);
    };
    if (on_a - on_b).length() > touch {
        return Ok(None);
    }
    // The mitre lies on both bands.
    for element in [a, b] {
        let FaceSurface::Plane { normal, d } = element.surface else {
            return Ok(None);
        };
        for point in [floor_point, on_a] {
            if (normal.dot(point - Point3::new(0.0, 0.0, 0.0)) - d).abs() > touch {
                return Ok(None);
            }
        }
    }
    Ok(Some(JointKind::Mitre {
        floor: floor_point,
        wall: on_a,
    }))
}

/// Where the line `p + s·u` crosses the line `q + t·v`, both in the plane
/// with normal `n`; `None` when they are parallel.
fn line_line(p: Point3, u: Vec3, q: Point3, v: Vec3, n: Vec3) -> Option<Point3> {
    let denominator = u.cross(v).dot(n);
    if denominator.abs() <= 1e-12 * u.length() * v.length() {
        return None;
    }
    let s = (q - p).cross(v).dot(n) / denominator;
    Some(p + u * s)
}

/// The point of the line `q + t·v` where the line `p + s·u` crosses it, or
/// `None` when the two lines pass further apart than `touch`.
fn closest_on_line(p: Point3, u: Vec3, q: Point3, v: Vec3, touch: f64) -> Option<Point3> {
    let w = p - q;
    let (a, b, c) = (u.dot(u), u.dot(v), v.dot(v));
    let (d, e) = (u.dot(w), v.dot(w));
    let denominator = a * c - b * b;
    if denominator.abs() <= 1e-12 * a * c {
        return None;
    }
    let s = (b * e - c * d) / denominator;
    let t = (a * e - b * d) / denominator;
    let on_p = p + u * s;
    let on_q = q + v * t;
    ((on_p - on_q).length() <= touch).then_some(on_q)
}

/// Whether a face other than the floor and the band's wall meets `corner`
/// as a plane holding the band's cross-section on `side` there.
fn square_end_face(
    topo: &Topology,
    original_faces: &[FaceId],
    floor: FaceId,
    element: &Element,
    side: usize,
    corner: VertexId,
    touch: f64,
) -> Result<bool, BlendError> {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let section = [
        topo.vertex(corner)?.point(),
        element.floor[side],
        element.wall_points[side],
    ];
    for &face in original_faces {
        if face == floor || face == element.wall {
            continue;
        }
        let data = topo.face(face)?;
        let FaceSurface::Plane { normal, d } = *data.surface() else {
            continue;
        };
        if section
            .iter()
            .any(|&point| (normal.dot(point - origin) - d).abs() > touch)
        {
            continue;
        }
        let at_corner = topo.wire(data.outer_wire())?.edges().iter().any(|oe| {
            topo.edge(oe.edge())
                .is_ok_and(|e| e.start() == corner || e.end() == corner)
        });
        if at_corner {
            return Ok(true);
        }
    }
    Ok(false)
}

/// The vertex at the band's start (`end == false`) or end, in run order.
fn run_vertex(topo: &Topology, element: &Element, end: bool) -> Result<VertexId, BlendError> {
    let edge = if end {
        element.edges[element.edges.len() - 1]
    } else {
        element.edges[0]
    };
    let data = topo.edge(edge)?;
    let (start_vertex, end_vertex) = (data.start(), data.end());
    let target = if end {
        element.floor[1]
    } else {
        element.floor[0]
    };
    // The run vertex is the edge end nearer the band's cross-section there.
    let distance = |vertex: VertexId| -> Result<f64, BlendError> {
        Ok((topo.vertex(vertex)?.point() - target).length())
    };
    Ok(if distance(start_vertex)? <= distance(end_vertex)? {
        start_vertex
    } else {
        end_vertex
    })
}

/// The exact contact edge of `element` from `from` to `to`: a line along a
/// straight band, an arc about the run's circle along a conical one.
fn contact_edge(
    topo: &mut Topology,
    element: &Element,
    from: VertexId,
    to: VertexId,
) -> Result<EdgeId, BlendError> {
    let Some((center, normal, sweep)) = element.arc else {
        return Ok(topo.add_edge(Edge::new(from, to, EdgeCurve::Line)));
    };
    let start = topo.vertex(from)?.point();
    let end = topo.vertex(to)?.point();
    // The contact's own centre: the floor circle's centre lifted along the
    // axis to the contact's level.
    let level = normal.dot(start - center);
    let own_center = center + normal * level;
    let radius = (start - own_center).length();
    let circle = Circle3D::new_with_ref(own_center, normal, radius, start - own_center)?;
    if (circle.evaluate(sweep) - end).length() > 1e-6 * radius.max(1.0) {
        return Err(BlendError::InvalidInput {
            reason: "chamfer floor chain: an arc contact does not reach its end".to_owned(),
        });
    }
    add_certified_curve_edge(topo, from, to, EdgeCurve::Circle(circle), (0.0, sweep))
}

/// Whether a band must carry `reversed` so its normal faces out: it faces
/// between the two faces it replaces, so its outward normal agrees with the
/// sum of theirs at the contacts.
fn band_reversed(
    topo: &Topology,
    floor: FaceId,
    wall: FaceId,
    element: &Element,
) -> Result<bool, BlendError> {
    let floor_point = element.floor[0];
    let wall_point = element.wall_points[0];
    let outward = |face: FaceId, point: Point3| -> Result<Vec3, BlendError> {
        let data = topo.face(face)?;
        let normal =
            surface_normal_at(data.surface(), point).ok_or(BlendError::TrimmingFailure { face })?;
        Ok(if data.is_reversed() { -normal } else { normal })
    };
    let sum = outward(floor, floor_point)? + outward(wall, wall_point)?;
    let mid = Point3::new(
        (floor_point.x() + wall_point.x()) * 0.5,
        (floor_point.y() + wall_point.y()) * 0.5,
        (floor_point.z() + wall_point.z()) * 0.5,
    );
    let normal = surface_normal_at(&element.surface, mid)
        .ok_or(BlendError::TrimmingFailure { face: floor })?;
    Ok(normal.dot(sum) < 0.0)
}

/// The geometric normal of `surface` at the point of it nearest `point`.
fn surface_normal_at(surface: &FaceSurface, point: Point3) -> Option<Vec3> {
    match surface {
        FaceSurface::Plane { normal, .. } => Some(*normal),
        surface @ (FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)) => {
            let (u, v) = surface.project_point(point)?;
            Some(surface.normal(u, v))
        }
    }
}
