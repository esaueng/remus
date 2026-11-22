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
//! A straight band may also end on planes that are not square to it — an
//! earlier chamfer's 45° stripe across the corner, say. The band then runs
//! on until its plane leaves the corner: its wall contact stops on the wall's
//! edge with the first such plane, its floor contact where it leaves the
//! floor, and its end is the band plane's section through the planes between
//! (one straight edge on each). When the floor contact runs past the first
//! plane's floor edge — a wider band ending on a narrower stripe — the end
//! crosses the stripe and carries on into the face beyond it, so the corner
//! is closed by a polygon of straight edges on exact planes. Each of those
//! planes loses the corner of it on the removed side of the band plane.
//!
//! Every boundary vertex the result needs is made first, by splitting the
//! body's edges, and each face is then cut exactly once along all of its
//! contacts: the floor along the whole selection, each wall along its own
//! run. Consecutive edges on one wall (a circle split into arcs) share one
//! band. Every surface and every edge is exact: planes, cones, lines and
//! circles.
//!
//! On a drafted body the rounds between the walls lean: a round's axis is
//! not square to the floor, so its foot is an ellipse and the band over it
//! is no circular cone. Every band of the chain then sets its contacts the
//! same distance `ρ` off the opposite face: the floor contact `ρ` inside the
//! wall (or round), the wall contact `ρ` above the floor. A straight band's
//! `d` along each face is that with `ρ = d·sin α`, `α` the angle between
//! floor and wall, so the rule is the one the straight bands already follow.
//! Over the round the floor contact is the floor's section of the coaxial
//! cylinder `ρ` inside the round, the wall contact the round's section by the
//! floor moved `ρ` along its normal: two exact ellipses, and similar ones
//! (both are sections of coaxial cylinders by parallel planes). The band
//! joins them point for point at each angle about the axis, and so is the
//! cone from their centre of similarity: an elliptic cone, stored as the
//! rational surface that is exactly it. At each of the round's seams with a
//! flanking wall its rulings lie in that wall's straight band, so the
//! straight band ends on that ruling (before or after the chain's vertex)
//! and the two meet as tangent bands. Over the round the distance along each
//! face is `ρ / sin α` with `α` the local angle, which varies slightly round
//! a leaning axis; `ρ` is one value only when both flanks make the same angle
//! with the floor, and other rounds are declined.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::det_hash::DetHashMap;
use remus_math::nurbs::surface::{ExactForm, NurbsSurface};
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
    /// For a band over a leaning round: its exact contact ellipses, each
    /// running in run order with its parameter.
    oblique: Option<[Ellipse3D; 2]>,
    /// Floor contact at the band's start and end cross-sections.
    floor: [Point3; 2],
    /// Wall contact at the band's start and end cross-sections.
    wall_points: [Point3; 2],
}

impl Element {
    /// A straight band: a plane along a line.
    const fn is_straight(&self) -> bool {
        self.arc.is_none() && self.oblique.is_none()
    }
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

/// The edges across the band at a joint or a run end: one straight edge,
/// or at an end crossing several planes, one per plane.
#[derive(Clone)]
struct Cross {
    floor: VertexId,
    wall: VertexId,
    /// The vertices from `floor` to `wall`, both included.
    path: Vec<VertexId>,
    /// `edges[i]` joins `path[i]` to `path[i + 1]`.
    edges: Vec<EdgeId>,
}

impl Cross {
    fn single(floor: VertexId, wall: VertexId, edge: EdgeId) -> Self {
        Self {
            floor,
            wall,
            path: vec![floor, wall],
            edges: vec![edge],
        }
    }

    /// The edges in order from the floor contact to the wall contact
    /// (`to_wall`) or back.
    fn uses(&self, topo: &Topology, to_wall: bool) -> Result<Vec<OrientedEdge>, BlendError> {
        let mut uses = Vec::with_capacity(self.edges.len());
        for (i, &edge) in self.edges.iter().enumerate() {
            let forward = topo.edge(edge)?.start() == self.path[i];
            uses.push(OrientedEdge::new(edge, forward));
        }
        Ok(if to_wall {
            uses
        } else {
            uses.iter()
                .rev()
                .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
                .collect()
        })
    }
}

/// Most planes an open run's end may cross past its corner.
const MAX_END_FACES: usize = 3;

/// How an open run's end closes against the faces past its corner.
struct EndPlan {
    /// The floor contact's end.
    floor: Point3,
    /// The wall contact's end.
    wall: Point3,
    /// For an end that runs past the corner: the planes the end section
    /// crosses, from the wall's side to the floor's, with `crossings[i]` the
    /// point where it crosses the edge `faces[i]` shares with `faces[i + 1]`
    /// and `swallowed[i]` that edge's floor vertex. Empty for a square end.
    /// An end on a single plane is notched as a square one is.
    faces: Vec<FaceId>,
    crossings: Vec<Point3>,
    swallowed: Vec<VertexId>,
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
    let mut end_plans: Vec<Option<[EndPlan; 2]>> = Vec::with_capacity(runs.len());
    let walls: Vec<FaceId> = runs
        .iter()
        .flat_map(|run| run.elements.iter().map(|element| element.wall))
        .collect();
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
        // its end cross-section notches, or runs on over planes oblique to it.
        if run.closed {
            end_plans.push(None);
            continue;
        }
        let mut plans = Vec::with_capacity(2);
        for (element, side) in [(&run.elements[0], 0), (&run.elements[count - 1], 1)] {
            let corner = run_vertex(topo, element, side == 1)?;
            let Some(plan) = end_plan(
                topo,
                adjacency,
                original_faces,
                floor,
                &walls,
                element,
                side,
                corner,
                touch,
            )?
            else {
                return Ok(None);
            };
            plans.push(plan);
        }
        let (Some(last), Some(first)) = (plans.pop(), plans.pop()) else {
            return Ok(None);
        };
        end_plans.push(Some([first, last]));
    }
    for ((run, kinds), plans) in runs.iter().zip(&joint_kinds).zip(&end_plans) {
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
            crosses.push(Cross::single(floor_vertex, wall_vertex, edge));
        }
        joint_crosses.push(crosses);
        let Some(plans) = plans.as_ref() else {
            end_crosses.push(None);
            continue;
        };
        let first = &run.elements[0];
        let last = &run.elements[count - 1];
        let mut ends = Vec::with_capacity(2);
        for (element, plan) in [(first, &plans[0]), (last, &plans[1])] {
            let mut edit = scope.edit(&mut replacements);
            let floor_face = current(edit.replacements(), floor);
            let floor_vertex =
                trimmer::split_body_boundary_at(topo, &mut edit, floor_face, plan.floor, touch)?
                    .ok_or_else(floor_failure)?;
            let wall_face = current(edit.replacements(), element.wall);
            let wall_vertex =
                trimmer::split_body_boundary_at(topo, &mut edit, wall_face, plan.wall, touch)?
                    .ok_or(BlendError::TrimmingFailure { face: element.wall })?;
            // Where the end section crosses from one plane to the next, on
            // the edge they share: listed from the floor's side.
            let mut path = vec![floor_vertex];
            for (i, &point) in plan.crossings.iter().enumerate().rev() {
                let face = current(edit.replacements(), plan.faces[i]);
                let vertex = trimmer::split_body_boundary_at(topo, &mut edit, face, point, touch)?
                    .ok_or(BlendError::TrimmingFailure { face })?;
                path.push(vertex);
            }
            path.push(wall_vertex);
            let mut edges = Vec::with_capacity(path.len() - 1);
            for pair in path.windows(2) {
                edges.push(topo.add_edge(Edge::new(pair[0], pair[1], EdgeCurve::Line)));
            }
            ends.push(Cross {
                floor: floor_vertex,
                wall: wall_vertex,
                path,
                edges,
            });
        }
        let (Some(end), Some(start)) = (ends.pop(), ends.pop()) else {
            return Err(floor_failure());
        };
        end_crosses.push(Some([start, end]));
    }

    // ── Contact edges, per element, between the crosses at its two sides. ──
    // (start cross, end cross) of element k of run r.
    let side_crosses = |r: usize, k: usize| -> (&Cross, &Cross) {
        let run = &runs[r];
        let count = run.elements.len();
        let start = if k == 0 {
            match &end_crosses[r] {
                Some(ends) => &ends[0],
                None => &joint_crosses[r][count - 1],
            }
        } else {
            &joint_crosses[r][k - 1]
        };
        let end = if k + 1 == count {
            match &end_crosses[r] {
                Some(ends) => &ends[1],
                None => &joint_crosses[r][count - 1],
            }
        } else {
            &joint_crosses[r][k]
        };
        (start, end)
    };
    // (floor contact, wall contact), each running in run order.
    let mut contacts: Vec<Vec<(EdgeId, EdgeId)>> = Vec::with_capacity(runs.len());
    for (r, run) in runs.iter().enumerate() {
        let mut run_contacts = Vec::with_capacity(run.elements.len());
        for (k, element) in run.elements.iter().enumerate() {
            let (start, end) = side_crosses(r, k);
            let floor_edge = contact_edge(topo, element, 0, start.floor, end.floor)?;
            let wall_edge = contact_edge(topo, element, 1, start.wall, end.wall)?;
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
                let Some(ends) = &end_crosses[r] else {
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
        let (Some(ends), Some(plans)) = (&end_crosses[r], &end_plans[r]) else {
            continue;
        };
        let count = run.elements.len();
        let corners = [
            run_vertex(topo, &run.elements[0], false)?,
            run_vertex(topo, &run.elements[count - 1], true)?,
        ];
        for ((cross, plan), corner) in ends.iter().zip(plans).zip(corners) {
            if !plan.crossings.is_empty() {
                cut_end_planes(topo, &mut replacements, cross, plan, corner)?;
                continue;
            }
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
                    (cross.edges[0], cross.floor, cross.wall),
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
            let mut loop_edges = vec![OrientedEdge::new(floor_edge, true)];
            loop_edges.extend(end.uses(topo, true)?);
            loop_edges.push(OrientedEdge::new(wall_edge, false));
            loop_edges.extend(start.uses(topo, false)?);
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
        skipped: Vec::new(),
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
            // An ellipse qualifies only as the foot of a leaning round
            // (`oblique_element`).
            EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => circular = true,
            EdgeCurve::NurbsCurve(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Ok(None);
            }
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
        // A band over a leaning round must hand over to a straight band on
        // both sides; an open run cannot end on one.
        if !closed
            && [elements.first(), elements.last()]
                .into_iter()
                .flatten()
                .any(|element| element.oblique.is_some())
        {
            return Ok(None);
        }
        if !hand_over(&mut elements, closed, distance) {
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
    if matches!(topo.edge(edge)?.curve(), EdgeCurve::Ellipse(_)) {
        return oblique_element(topo, adjacency, floor, oriented, distance);
    }
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
        oblique: None,
        floor: [f0, f1],
        wall_points: [w0, w1],
    }))
}

/// The band along `oriented`, the elliptic foot of a leaning round on the
/// floor (see the module notes), or `None` when the edge is not one.
///
/// Declined: a wall that is not a cylinder, an axis lying in the floor or
/// square to it, a round turning half a turn or more, flanks at different angles to the
/// floor, and a setback that reaches the axis.
fn oblique_element(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    floor: FaceId,
    oriented: OrientedEdge,
    distance: f64,
) -> Result<Option<Element>, BlendError> {
    let edge = oriented.edge();
    let wall = match *adjacency.faces_for_edge(edge) {
        [a, b] if a == floor && b != floor => b,
        [a, b] if b == floor && a != floor => a,
        _ => return Ok(None),
    };
    let FaceSurface::Plane { normal, d: level } = *topo.face(floor)?.surface() else {
        return Ok(None);
    };
    let FaceSurface::Cylinder(cylinder) = topo.face(wall)?.surface() else {
        return Ok(None);
    };
    let level = level / normal.length();
    let (Ok(normal), Ok(axis)) = (normal.normalize(), cylinder.axis().normalize()) else {
        return Ok(None);
    };
    let (origin, radius) = (cylinder.origin(), cylinder.radius());
    // An axis in the floor makes the edge a ruling, not an arc; one square
    // to it is a circle (stored as an ellipse here), the conical bands' case.
    let lean = normal.dot(axis).abs();
    if lean <= 1e-6 || lean >= 1.0 - 1e-9 {
        return Ok(None);
    }
    let Some(frames) = crate::analytic::edge_material_frames_at(topo, edge, floor, wall)? else {
        return Ok(None);
    };
    let data = topo.edge(edge)?;
    let (Ok((t0, t1)), start, end) = (
        data.strict_domain(),
        topo.vertex(data.start())?.point(),
        topo.vertex(data.end())?.point(),
    ) else {
        return Ok(None);
    };
    let middle = data
        .curve()
        .evaluate_with_endpoints(f64::midpoint(t0, t1), start, end);
    let (start, end) = if oriented.is_forward() {
        (start, end)
    } else {
        (end, start)
    };
    let radial = |p: Point3| {
        let v = p - origin;
        (v - axis * axis.dot(v)).normalize()
    };
    let (Ok(u0), Ok(u_end), Ok(u_mid)) = (radial(start), radial(end), radial(middle)) else {
        return Ok(None);
    };
    // Angles about the axis turned with the run.
    let axis = if u0.cross(u_mid).dot(axis) >= 0.0 {
        axis
    } else {
        -axis
    };
    let w0 = axis.cross(u0);
    let angle = |u: Vec3| {
        let a = u.dot(w0).atan2(u.dot(u0));
        if a < 0.0 { a + TAU } else { a }
    };
    let (sweep, mid) = (angle(u_end), angle(u_mid));
    if !(sweep > 1e-9 && sweep < PI && mid > 0.0 && mid < sweep) {
        return Ok(None);
    }
    let along = |phi: f64| u0 * phi.cos() + w0 * phi.sin();
    // The round's tangent plane at each seam is its flank's: one setback
    // serves both only when they make the same angle with the floor.
    let (sin_start, sin_end) = (
        normal.cross(along(0.0)).length(),
        normal.cross(along(sweep)).length(),
    );
    if (sin_start - sin_end).abs() > 1e-9 {
        return Ok(None);
    }
    let rho = distance * sin_start;
    // The floor's material lies toward the axis or away from it; the wall's
    // on one side of the floor's plane.
    let floor_radius = if frames.into1.dot(u_mid) < 0.0 {
        radius - rho
    } else {
        radius + rho
    };
    if floor_radius <= 1e-9 * radius.max(1.0) {
        return Ok(None);
    }
    let wall_level = level + rho * frames.into2.dot(normal).signum();
    // Where the cylinder of `r` about the axis meets the plane `n·x = c` at
    // the axis angle whose radial is `c0 u0 + s0 w0` (unit or not: the map
    // is affine in `(c0, s0)`, so it carries rational control points too).
    let at = |r: f64, c: f64, cos: f64, sin: f64| {
        let base = origin + (u0 * cos + w0 * sin) * r;
        base + axis * ((c - normal.dot(base - Point3::new(0.0, 0.0, 0.0))) / normal.dot(axis))
    };
    let ellipse = |r: f64, c: f64| -> Result<Ellipse3D, BlendError> {
        let centre = at(0.0, c, 1.0, 0.0);
        let minor = normal.cross(axis).normalize()?;
        let major = normal.cross(minor);
        // Increasing parameter turns with the run.
        let turn = if normal.dot(axis) > 0.0 {
            normal
        } else {
            -normal
        };
        Ok(Ellipse3D::new_with_ref(
            centre,
            turn,
            r / normal.dot(axis).abs(),
            r,
            major,
        )?)
    };
    let contacts = [ellipse(floor_radius, level)?, ellipse(radius, wall_level)?];
    // The band: each row a rational quadratic arc of at most a quarter turn.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    let spans = ((sweep / FRAC_PI_2).ceil() as usize).max(1);
    #[allow(clippy::cast_precision_loss)]
    let step = sweep / spans as f64;
    let mut rows = [Vec::new(), Vec::new()];
    let mut weights = Vec::new();
    let mut knots = vec![0.0; 3];
    for span in 0..spans {
        #[allow(clippy::cast_precision_loss)]
        let a = step * span as f64;
        let half = (step / 2.0).cos();
        let stations = [
            (a.cos(), a.sin(), 1.0),
            (
                (a + step / 2.0).cos() / half,
                (a + step / 2.0).sin() / half,
                half,
            ),
        ];
        for &(cos, sin, weight) in &stations {
            rows[0].push(at(floor_radius, level, cos, sin));
            rows[1].push(at(radius, wall_level, cos, sin));
            weights.push(weight);
        }
        #[allow(clippy::cast_precision_loss)]
        let knot = (span + 1) as f64 / spans as f64;
        knots.extend([knot, knot]);
    }
    rows[0].push(at(floor_radius, level, sweep.cos(), sweep.sin()));
    rows[1].push(at(radius, wall_level, sweep.cos(), sweep.sin()));
    weights.push(1.0);
    knots.push(1.0);
    // Exact, not fitted: `at` is affine in `(cos, sin)`, so each row is the
    // affine image of an exact rational arc of the unit circle, and affine
    // jointly in `(r, c)`, so with equal weights in both rows the straight
    // line between them at each angle is `at` of the interpolated radius
    // and level: the cone `h + w = ρ`. Recorded so an exact-only caller
    // takes the band as exact (sampled against the cone in
    // `regress_issue_1743_two_stripe_vertex.rs`).
    let surface = NurbsSurface::new(
        1,
        2,
        vec![0.0, 0.0, 1.0, 1.0],
        knots,
        rows.to_vec(),
        vec![weights.clone(), weights],
    )?
    .with_exact_form(ExactForm::EllipticCone);
    let ends = |r: f64, c: f64| [at(r, c, 1.0, 0.0), at(r, c, sweep.cos(), sweep.sin())];
    Ok(Some(Element {
        edges: vec![edge],
        wall,
        surface: FaceSurface::Nurbs(surface),
        arc: None,
        oblique: Some(contacts),
        floor: ends(floor_radius, level),
        wall_points: ends(radius, wall_level),
    }))
}

/// End each straight band beside a band over a leaning round on that band's
/// seam ruling, which lies in it (see the module notes). `false` when a
/// neighbour is not a straight band through the ruling.
fn hand_over(elements: &mut [Element], closed: bool, distance: f64) -> bool {
    let touch = 1e-6 * distance.max(1.0);
    let count = elements.len();
    let off_line = |p: Point3, a: Point3, b: Point3| {
        let Ok(dir) = (b - a).normalize() else {
            return f64::INFINITY;
        };
        let v = p - a;
        (v - dir * dir.dot(v)).length()
    };
    for k in 0..count {
        if elements[k].oblique.is_none() {
            continue;
        }
        let neighbours = [
            (k > 0 || closed).then(|| (k + count - 1) % count),
            (k + 1 < count || closed).then(|| (k + 1) % count),
        ];
        for (side, neighbour) in neighbours.into_iter().enumerate() {
            let Some(n) = neighbour else {
                return false;
            };
            // The ruling at the round's start meets the band before it at
            // that band's end, and the other way round.
            let (floor_point, wall_point) =
                (elements[k].floor[side], elements[k].wall_points[side]);
            let other = &mut elements[n];
            if n == k || !other.is_straight() {
                return false;
            }
            let end = 1 - side;
            if off_line(floor_point, other.floor[0], other.floor[1]) > touch
                || off_line(wall_point, other.wall_points[0], other.wall_points[1]) > touch
            {
                return false;
            }
            let (keep_floor, keep_wall) = (other.floor[side], other.wall_points[side]);
            let sense = |kept: Point3, old: Point3, new: Point3| {
                let (before, after) = if end == 1 {
                    (old - kept, new - kept)
                } else {
                    (kept - old, kept - new)
                };
                before.dot(after) > 0.0 && after.length() > touch
            };
            if !sense(keep_floor, other.floor[end], floor_point)
                || !sense(keep_wall, other.wall_points[end], wall_point)
            {
                return false;
            }
            other.floor[end] = floor_point;
            other.wall_points[end] = wall_point;
        }
    }
    true
}

/// Whether `next` continues `last`'s band: the same wall, the same carrier
/// and a shared cross-section.
fn mergeable(last: &Element, next: &Element, distance: f64) -> bool {
    let touch = 1e-6 * distance.max(1.0);
    if last.wall != next.wall || last.oblique.is_some() || next.oblique.is_some() {
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
    if !a.is_straight() || !b.is_straight() {
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

/// How the band of `element` ends at `corner` (its start for `side` 0, its
/// end for 1), or `None` when this assembler does not close it there.
///
/// A plane square to the band through its end section takes the section as
/// it is. Otherwise a straight band runs on over the planes past the
/// corner, in the floor boundary's order: its wall contact must stop on the
/// wall's straight edge with the first of them, and its floor contact on
/// one of their straight floor edges (at most [`MAX_END_FACES`] of them).
/// Every vertex those planes keep must lie on the band plane's material
/// side, so each loses one corner polygon.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn end_plan(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    original_faces: &[FaceId],
    floor: FaceId,
    walls: &[FaceId],
    element: &Element,
    side: usize,
    corner: VertexId,
    touch: f64,
) -> Result<Option<EndPlan>, BlendError> {
    if square_end_face(topo, original_faces, floor, element, side, corner, touch)? {
        return Ok(Some(EndPlan {
            floor: element.floor[side],
            wall: element.wall_points[side],
            faces: Vec::new(),
            crossings: Vec::new(),
            swallowed: Vec::new(),
        }));
    }
    if element.arc.is_some() {
        return Ok(None);
    }
    let FaceSurface::Plane {
        normal: band_normal,
        d: band_d,
    } = element.surface
    else {
        return Ok(None);
    };
    let FaceSurface::Plane {
        normal: floor_normal,
        ..
    } = *topo.face(floor)?.surface()
    else {
        return Ok(None);
    };
    let origin = Point3::new(0.0, 0.0, 0.0);
    let band_at = |point: Point3| band_normal.dot(point - origin) - band_d;
    let corner_point = topo.vertex(corner)?.point();
    // The corner is on the removed side of the band plane.
    let removed = band_at(corner_point);
    if removed.abs() <= touch {
        return Ok(None);
    }
    let kept = |point: Point3| band_at(point) * removed.signum() < -touch;
    let along = element.floor[1] - element.floor[0];
    let (floor_from, wall_from) = (element.floor[side], element.wall_points[side]);
    let outward = if side == 1 { along } else { -along };
    let other_face = |edge: EdgeId| -> Option<FaceId> {
        match *adjacency.faces_for_edge(edge) {
            [a, b] if a == floor && b != floor => Some(b),
            [a, b] if b == floor && a != floor => Some(a),
            _ => None,
        }
    };
    // The straight edge of `face`'s outer boundary at `vertex` that it
    // shares with `other`, from `vertex` to its far end.
    let shared_line = |face: FaceId,
                       other: FaceId,
                       vertex: VertexId|
     -> Result<Option<(Point3, Point3)>, BlendError> {
        for oriented in topo.wire(topo.face(face)?.outer_wire())?.edges() {
            let edge = topo.edge(oriented.edge())?;
            if !matches!(edge.curve(), EdgeCurve::Line)
                || !adjacency.faces_for_edge(oriented.edge()).contains(&other)
            {
                continue;
            }
            let far = if edge.start() == vertex {
                edge.end()
            } else if edge.end() == vertex {
                edge.start()
            } else {
                continue;
            };
            return Ok(Some((
                topo.vertex(vertex)?.point(),
                topo.vertex(far)?.point(),
            )));
        }
        Ok(None)
    };
    // Where the band plane crosses the segment `from -> to` strictly past
    // `from`, at most at `to`.
    let band_crossing = |from: Point3, to: Point3| -> Option<Point3> {
        let (a, b) = (band_at(from), band_at(to));
        if (a - b).abs() <= f64::EPSILON {
            return None;
        }
        let t = a / (a - b);
        let length = (to - from).length();
        (t * length > touch && t <= 1.0 + touch / length).then(|| from + (to - from) * t.min(1.0))
    };

    let wire = topo.wire(topo.face(floor)?.outer_wire())?.edges().to_vec();
    let n = wire.len();
    let end_edge = if side == 1 {
        element.edges[element.edges.len() - 1]
    } else {
        element.edges[0]
    };
    let Some(position) = wire.iter().position(|oe| oe.edge() == end_edge) else {
        return Ok(None);
    };
    let mut near = corner;
    let mut faces: Vec<FaceId> = Vec::new();
    let mut crossings: Vec<Point3> = Vec::new();
    let mut swallowed: Vec<VertexId> = Vec::new();
    let mut wall_end = None;
    for i in 1..=MAX_END_FACES.min(n.saturating_sub(element.edges.len())) {
        let oriented = if side == 1 {
            wire[(position + i) % n]
        } else {
            wire[(position + n - i % n) % n]
        };
        let data = topo.edge(oriented.edge())?;
        if !matches!(data.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        let (from, far) = if side == 1 {
            (oriented.oriented_start(data), oriented.oriented_end(data))
        } else {
            (oriented.oriented_end(data), oriented.oriented_start(data))
        };
        let Some(face) = other_face(oriented.edge()) else {
            return Ok(None);
        };
        if from != near
            || walls.contains(&face)
            || faces.contains(&face)
            || !matches!(topo.face(face)?.surface(), FaceSurface::Plane { .. })
        {
            return Ok(None);
        }
        if i == 1 {
            // The wall contact stops on the wall's edge with this plane.
            let Some((at, to)) = shared_line(element.wall, face, corner)? else {
                return Ok(None);
            };
            let Some(point) = band_crossing(at, to) else {
                return Ok(None);
            };
            if (point - wall_from).dot(outward) < -touch {
                return Ok(None);
            }
            wall_end = Some(point);
        }
        faces.push(face);
        let (near_point, far_point) = (topo.vertex(from)?.point(), topo.vertex(far)?.point());
        // The floor contact runs out over this edge, or past its far end.
        let Some(hit) = line_line(
            floor_from,
            along,
            near_point,
            far_point - near_point,
            floor_normal,
        ) else {
            return Ok(None);
        };
        if (hit - floor_from).dot(outward) < -touch {
            return Ok(None);
        }
        let span = far_point - near_point;
        let length = span.length();
        let reach = (hit - near_point).dot(span) / (length * length);
        if reach * length <= touch {
            return Ok(None);
        }
        if (reach - 1.0) * length <= touch {
            let floor_end = if (reach - 1.0).abs() * length <= touch {
                far_point
            } else {
                hit
            };
            let Some(wall_end) = wall_end else {
                return Ok(None);
            };
            // Each plane keeps all of its boundary but the corner polygon.
            let mut cut_off = vec![corner];
            cut_off.extend(swallowed.iter().copied());
            let mut on_section = vec![floor_end, wall_end];
            on_section.extend(crossings.iter().copied());
            for &plane in &faces {
                for oriented in topo.wire(topo.face(plane)?.outer_wire())?.edges() {
                    let vertex = oriented.oriented_start(topo.edge(oriented.edge())?);
                    let point = topo.vertex(vertex)?.point();
                    if !cut_off.contains(&vertex)
                        && on_section.iter().all(|&at| (point - at).length() > touch)
                        && !kept(point)
                    {
                        return Ok(None);
                    }
                }
            }
            return Ok(Some(EndPlan {
                floor: floor_end,
                wall: wall_end,
                faces,
                crossings,
                swallowed,
            }));
        }
        // Past the far end: the section crosses from this plane into the
        // next across their shared edge at `far`.
        if kept(far_point) {
            return Ok(None);
        }
        let next = if side == 1 {
            wire[(position + i + 1) % n]
        } else {
            wire[(position + n - (i + 1) % n) % n]
        };
        let Some(next_face) = other_face(next.edge()) else {
            return Ok(None);
        };
        let Some((at, to)) = shared_line(face, next_face, far)? else {
            return Ok(None);
        };
        let Some(point) = band_crossing(at, to) else {
            return Ok(None);
        };
        crossings.push(point);
        swallowed.push(far);
        near = far;
    }
    Ok(None)
}

/// Cut each plane an end section crosses past its corner along its piece of
/// the section, dropping the corner polygon the band removes.
fn cut_end_planes(
    topo: &mut Topology,
    replacements: &mut DetHashMap<FaceId, FaceId>,
    cross: &Cross,
    plan: &EndPlan,
    corner: VertexId,
) -> Result<(), BlendError> {
    // The cross runs floor -> X_{k-1} -> ... -> X_1 -> wall; plane i holds
    // the piece from X_i (the wall vertex for the first) to X_{i+1} (the
    // floor vertex for the last), counted from the wall's side.
    let k = plan.faces.len();
    let mut fan = vec![corner];
    fan.extend(plan.swallowed.iter().copied());
    for (i, &original) in plan.faces.iter().enumerate() {
        let face = replacements.get(&original).copied().unwrap_or(original);
        let failure = BlendError::TrimmingFailure { face: original };
        let edge_index = k - 1 - i;
        let (from_vertex, to_vertex) = (cross.path[edge_index + 1], cross.path[edge_index]);
        let edge = cross.edges[edge_index];
        let wire = topo.wire(topo.face(face)?.outer_wire())?.edges().to_vec();
        let corners: Vec<VertexId> = fan[i..(i + 2).min(fan.len())].to_vec();
        let mut avoid = Vec::new();
        for oriented in &wire {
            let data = topo.edge(oriented.edge())?;
            if corners.contains(&data.start()) || corners.contains(&data.end()) {
                avoid.push(oriented.edge());
            }
        }
        let corner_point = topo.vertex(fan[i])?.point();
        let (Some(from), Some(to)) = (
            trimmer::vertex_occurrence(topo, face, &wire, from_vertex, corner_point)?,
            trimmer::vertex_occurrence(topo, face, &wire, to_vertex, corner_point)?,
        ) else {
            return Err(failure);
        };
        let forward = topo.edge(edge)?.start() == from_vertex;
        let piece = CutPiece {
            from,
            to,
            edges: vec![OrientedEdge::new(edge, forward)],
        };
        let trimmed = trimmer::cut_face(topo, face, &wire, &[piece], &avoid)?;
        replacements.insert(original, trimmed);
    }
    Ok(())
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
/// straight band, an arc about the run's circle along a conical one, an arc
/// of the contact's ellipse over a leaning round. `side` is 0 for the floor
/// contact, 1 for the wall's.
fn contact_edge(
    topo: &mut Topology,
    element: &Element,
    side: usize,
    from: VertexId,
    to: VertexId,
) -> Result<EdgeId, BlendError> {
    if let Some(contacts) = &element.oblique {
        let ellipse = contacts[side].clone();
        let start = topo.vertex(from)?.point();
        let end = topo.vertex(to)?.point();
        let t0 = ellipse.project(start);
        // Both lie in (-π, π]: one turn puts the end after the start.
        let mut t1 = ellipse.project(end);
        if t1 <= t0 {
            t1 += TAU;
        }
        let scale = ellipse.semi_major().max(1.0);
        if t1 - t0 >= PI
            || (ellipse.evaluate(t0) - start).length() > 1e-6 * scale
            || (ellipse.evaluate(t1) - end).length() > 1e-6 * scale
        {
            return Err(BlendError::InvalidInput {
                reason: "chamfer floor chain: an ellipse contact does not reach its ends"
                    .to_owned(),
            });
        }
        return add_certified_curve_edge(topo, from, to, EdgeCurve::Ellipse(ellipse), (t0, t1));
    }
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
