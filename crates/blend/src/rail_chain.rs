//! One fillet band along a concave tangent chain whose second face changes.
//!
//! A round boss fused against a block meets it in a concave seam that can run
//! up the block's wall, over an earlier round and across its top: one
//! tangent-continuous chain of edges, every one of them on the boss, each
//! between the boss and a different face of the block. A stripe is computed
//! against one face pair, so the walking builder refuses such a chain
//! ([`BlendError::UnsupportedSeamCrossing`]); this module fillets it as one
//! band.
//!
//! The face every edge shares is the *rail*; the others, in chain order, are
//! the *supports*. The ball rolls along the rail throughout and against one
//! support at a time. It leaves a support where its contact crosses the
//! tangent seam onto the next one. That is a station of its own, not a chain
//! vertex: over an earlier round the ball reaches the top face before the
//! chain does. The march finds those stations by bisection, so at each the
//! ball touches both supports at one seam point and consecutive segments
//! share that section.
//!
//! The band is one NURBS surface through every section, linear between
//! stations, so its long boundaries are exactly the contact polylines the
//! faces are trimmed along; like every walked band it is an approximation.
//! Each support is trimmed along its own run of the support contact and the
//! rail along the whole rail contact. Where the rail contact crosses the
//! rail's periodic seam a station is inserted on the seam, and the rail keeps
//! the seam's far part. Where the contact ends inside the rail (the boss
//! rising past the block's last face) it closes back to the chain's end
//! vertex along the rail, in the end face's plane. Both ends are then closed
//! like any straight fillet's.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::surfaces::CylindricalSurface;
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::analytic::edge_material_frames_at;
use crate::blend_func::{BlendParams, ConstRadBlend};
use crate::body_edit::EditScope;
use crate::builder_utils::{
    FlippedNormalSurface, PlaneAdapter, add_certified_curve_edge, surface_ref_or_adapter,
};
use crate::fillet_builder::{
    ball_side, live_boundary_wires, make_end_arc, sections_to_contact_curve, stitch_end,
};
use crate::section::CircSection;
use crate::spine::Spine;
use crate::trimmer::{self, CutPiece};
use crate::walker::{Walker, WalkerConfig, approximate_blend_surface_linear_v};
use crate::{BlendError, BlendFaceOrigins, BlendResult};

/// Largest station step, as a fraction of the chain's length (the walker's).
const MAX_STEP_FRACTION: f64 = 0.05;
/// Largest turn of either contact direction between consecutive stations
/// (2°): it bounds how far the band's linear spans sag off the true blend.
const MAX_TURN: f64 = 0.035;
/// Bisection steps for a support switch or a rail-seam crossing.
const BISECTIONS: usize = 80;

/// The faces along the chain.
struct RailChain {
    /// The face every chain edge lies on.
    rail: FaceId,
    /// The other faces, in chain order, each over a run of chain edges.
    supports: Vec<Support>,
}

struct Support {
    face: FaceId,
    /// Indices into the spine's edges.
    edges: std::ops::Range<usize>,
}

/// The tangent seam between consecutive supports: the edge they share, as
/// the later support uses it.
pub struct SupportSeam {
    pub next: FaceId,
    pub edge: OrientedEdge,
}

/// Where the rail contact meets the rail's boundary.
struct RailNode {
    /// Parameter on the rail contact.
    t: f64,
    /// The contact's vertex there.
    vertex: VertexId,
    /// Contact points just after and just before.
    ahead: Point3,
    behind: Point3,
    /// The chain's end vertex, when the contact ends inside the rail and
    /// closes back to it.
    closed: Option<VertexId>,
}

/// The sections of the march.
struct March {
    sections: Vec<CircSection>,
    /// Index of each segment's last section; the next segment starts there.
    ends: Vec<usize>,
    /// Where the rail contact crosses a seam edge of the rail: between
    /// section `index` and the next, at the seam's angle.
    rail_crossings: Vec<(usize, f64)>,
}

/// Fillet `spine` as one band when it is a concave chain whose faces change
/// on one side only, or `None` when it is not one.
///
/// Called with the whole build's state; on `Some` the returned result is the
/// build's.
///
/// # Errors
///
/// Returns [`BlendError`] when the chain qualifies but the march, a trim or
/// the assembly fails.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub fn fillet(
    topo: &mut Topology,
    solid: SolidId,
    adjacency: &AdjacencyIndex,
    spine: &Spine,
    radius: f64,
    scope: &mut EditScope,
    original_faces: &[FaceId],
    inner_shells: &[ShellId],
    requested: &[EdgeId],
) -> Result<Option<BlendResult>, BlendError> {
    let Some(chain) = detect(topo, adjacency, spine)? else {
        return Ok(None);
    };
    let spine_edges = spine.edges();
    let length = spine.length();
    let (v_start, v_end) = chain_ends(topo, spine_edges)?;
    let mut seams = Vec::with_capacity(chain.supports.len() - 1);
    for pair in chain.supports.windows(2) {
        let joint = shared_vertex(
            topo,
            spine_edges[pair[0].edges.end - 1],
            spine_edges[pair[1].edges.start],
        )?;
        let near = topo.vertex(joint)?.point();
        let Some(seam) = support_seam(topo, pair[0].face, pair[1].face, near)? else {
            return Ok(None);
        };
        seams.push(seam);
    }

    let rail_data = topo.face(chain.rail)?;
    let rail_surface = rail_data.surface().clone();
    let (rail_cylinder, rail_seams) = match &rail_surface {
        FaceSurface::Cylinder(cylinder) => (
            Some(cylinder.clone()),
            seam_angles(topo, chain.rail, cylinder)?,
        ),
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => (None, Vec::new()),
    };
    let mut surfaces = vec![(rail_surface.clone(), rail_data.is_reversed())];
    for support in &chain.supports {
        let face = topo.face(support.face)?;
        surfaces.push((face.surface().clone(), face.is_reversed()));
    }

    let march = {
        let mut adapters: Vec<Option<PlaneAdapter>> = surfaces.iter().map(|_| None).collect();
        let bases: Vec<&dyn ParametricSurface> = surfaces
            .iter()
            .zip(adapters.iter_mut())
            .map(|((surface, _), adapter)| surface_ref_or_adapter(surface, adapter))
            .collect();
        let flipped: Vec<FlippedNormalSurface<'_>> = bases
            .iter()
            .map(|&base| FlippedNormalSurface::new(base))
            .collect();
        // A concave edge: the ball sits off every face's outward side.
        let sides: Vec<&dyn ParametricSurface> = bases
            .iter()
            .zip(&flipped)
            .zip(&surfaces)
            .map(|((&base, flipped), (_, reversed))| ball_side(base, flipped, *reversed, true))
            .collect();
        let blend = ConstRadBlend { radius };
        let config = WalkerConfig {
            // Quadric supports; see `compute_stripe_for_spine`.
            tol_3d: 5.0 * remus_math::tolerance::Tolerance::new().linear,
            ..WalkerConfig::default()
        };
        let walkers: Vec<Walker<'_, ConstRadBlend>> = sides[1..]
            .iter()
            .map(|&support| Walker::new(&blend, sides[0], support, spine, topo, config))
            .collect();
        let past_seam = |segment: usize, point: Point3| -> Result<f64, BlendError> {
            seam_offset(topo, &seams[segment], point)?.ok_or(BlendError::StartSolutionFailure {
                edge: spine_edges[0],
                t: 0.0,
            })
        };
        march(
            spine_edges[0],
            length,
            &walkers,
            &sides[1..],
            rail_cylinder.as_ref(),
            &rail_seams,
            past_seam,
        )?
    };
    let sections = &march.sections;
    let count = sections.len();

    #[allow(clippy::cast_precision_loss)]
    let station = |index: usize| index as f64 / (count - 1) as f64;
    log::debug!(
        target: "remus_approx",
        "fillet: rail chain over {} supports — {count} walked sections, approximate NURBS blend surface",
        chain.supports.len()
    );
    let rail_contact = sections_to_contact_curve(sections, |section| section.p1)?;
    let support_contact = sections_to_contact_curve(sections, |section| section.p2)?;
    let band = FaceSurface::Nurbs(approximate_blend_surface_linear_v(sections)?);
    let touch = 1e-5 * radius.max(1.0);
    let (first_section, last_section) = (&sections[0], &sections[count - 1]);
    let mut replacements = remus_math::det_hash::DetHashMap::default();
    let current = |replacements: &remus_math::det_hash::DetHashMap<FaceId, FaceId>,
                   face: FaceId| replacements.get(&face).copied().unwrap_or(face);

    // Supports, in chain order: each run ends where the next one starts, on
    // the vertex the earlier trim made.
    let mut support_edges: Vec<EdgeId> = Vec::with_capacity(chain.supports.len());
    let mut support_ends: Vec<(VertexId, VertexId)> = Vec::with_capacity(chain.supports.len());
    for (index, support) in chain.supports.iter().enumerate() {
        let first = if index == 0 { 0 } else { march.ends[index - 1] };
        let last = march.ends[index];
        let face = current(&replacements, support.face);
        let mut wire = topo.wire(topo.face(face)?.outer_wire())?.edges().to_vec();
        let failure = BlendError::TrimmingFailure { face: support.face };
        let (from_vertex, to_vertex) = {
            let mut edit = scope.edit(&mut replacements);
            let from = trimmer::boundary_vertex_at(
                topo,
                &mut edit,
                face,
                &mut wire,
                sections[first].p2,
                touch,
                spine_edges,
            )?;
            let to = trimmer::boundary_vertex_at(
                topo,
                &mut edit,
                face,
                &mut wire,
                sections[last].p2,
                touch,
                spine_edges,
            )?;
            let (Some(from), Some(to)) = (from, to) else {
                return Err(failure);
            };
            (from, to)
        };
        let edge = add_certified_curve_edge(
            topo,
            from_vertex,
            to_vertex,
            EdgeCurve::NurbsCurve(support_contact.clone()),
            (station(first), station(last)),
        )?;
        let (Some(from), Some(to)) = (
            trimmer::vertex_occurrence(topo, face, &wire, from_vertex, sections[first + 1].p2)?,
            trimmer::vertex_occurrence(topo, face, &wire, to_vertex, sections[last - 1].p2)?,
        ) else {
            return Err(failure);
        };
        let trimmed = trimmer::cut_face(
            topo,
            face,
            &wire,
            &[CutPiece {
                from,
                to,
                edges: vec![OrientedEdge::new(edge, true)],
            }],
            spine_edges,
        )?;
        replacements.insert(support.face, trimmed);
        support_edges.push(edge);
        support_ends.push((from_vertex, to_vertex));
    }

    // The rail, along the whole contact: split at every crossing of a seam
    // edge, and closed back to the chain's end vertex where it ends inside.
    let rail_face = current(&replacements, chain.rail);
    let mut wire = topo
        .wire(topo.face(rail_face)?.outer_wire())?
        .edges()
        .to_vec();
    let rail_failure = BlendError::TrimmingFailure { face: chain.rail };
    // Where the contact meets the boundary: its contact parameter and point,
    // the contact points on either side, and how far the point may sit off
    // the boundary.
    let mut stops: Vec<(f64, Point3, Point3, Point3, f64)> = vec![(
        0.0,
        first_section.p1,
        sections[1].p1,
        first_section.p1,
        touch,
    )];
    if let Some(cylinder) = &rail_cylinder {
        for &(index, seam) in &march.rail_crossings {
            let (a, b) = (sections[index].p1, sections[index + 1].p1);
            let Some(fraction) = seam_plane_crossing(cylinder, seam, a, b) else {
                return Err(rail_failure);
            };
            let point = a + (b - a) * fraction;
            // The contact's chord runs inside the rail between stations; the
            // seam edge is on it.
            let sag = (radial_distance(cylinder, point) - cylinder.radius()).abs();
            stops.push((
                (station(index + 1) - station(index)).mul_add(fraction, station(index)),
                point,
                b,
                a,
                touch + 2.0 * sag,
            ));
        }
    }
    stops.push((
        1.0,
        last_section.p1,
        last_section.p1,
        sections[count - 2].p1,
        touch,
    ));
    let mut nodes: Vec<RailNode> = Vec::with_capacity(stops.len());
    {
        let mut edit = scope.edit(&mut replacements);
        for (rank, &(t, point, ahead, behind, tolerance)) in stops.iter().enumerate() {
            let at_end = rank == 0 || rank == stops.len() - 1;
            match trimmer::boundary_vertex_at(
                topo,
                &mut edit,
                rail_face,
                &mut wire,
                point,
                tolerance,
                spine_edges,
            )? {
                Some(vertex) => nodes.push(RailNode {
                    t,
                    vertex,
                    ahead,
                    behind,
                    closed: None,
                }),
                None if at_end => {
                    let open = topo.add_vertex(Vertex::new(point, 1e-7));
                    nodes.push(RailNode {
                        t,
                        vertex: open,
                        ahead,
                        behind,
                        closed: Some(if rank == 0 { v_start } else { v_end }),
                    });
                }
                // The face runs across the seam without a seam edge here.
                None => {}
            }
        }
    }
    let mut pieces = Vec::with_capacity(nodes.len() - 1);
    let mut rail_edges: Vec<EdgeId> = Vec::with_capacity(nodes.len() - 1);
    for pair in nodes.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let mut edges = Vec::with_capacity(3);
        if let Some(chain_vertex) = a.closed {
            let Some(closure) = closure_edge(topo, &rail_surface, chain_vertex, a.vertex)? else {
                return Err(rail_failure);
            };
            edges.push(OrientedEdge::new(
                closure,
                topo.edge(closure)?.start() == chain_vertex,
            ));
        }
        let contact = add_certified_curve_edge(
            topo,
            a.vertex,
            b.vertex,
            EdgeCurve::NurbsCurve(rail_contact.clone()),
            (a.t, b.t),
        )?;
        edges.push(OrientedEdge::new(contact, true));
        rail_edges.push(contact);
        if let Some(chain_vertex) = b.closed {
            let Some(closure) = closure_edge(topo, &rail_surface, b.vertex, chain_vertex)? else {
                return Err(rail_failure);
            };
            edges.push(OrientedEdge::new(
                closure,
                topo.edge(closure)?.start() == b.vertex,
            ));
        }
        // Each end joins the boundary at the vertex the cut reaches, on the
        // side of it the piece runs into.
        let open_point = |node: &RailNode| topo.vertex(node.vertex).map(Vertex::point);
        let (from_vertex, from_toward) = match a.closed {
            Some(chain_vertex) => (chain_vertex, open_point(a)?),
            None => (a.vertex, a.ahead),
        };
        let (to_vertex, to_toward) = match b.closed {
            Some(chain_vertex) => (chain_vertex, open_point(b)?),
            None => (b.vertex, b.behind),
        };
        let (Some(from), Some(to)) = (
            trimmer::vertex_occurrence(topo, rail_face, &wire, from_vertex, from_toward)?,
            trimmer::vertex_occurrence(topo, rail_face, &wire, to_vertex, to_toward)?,
        ) else {
            return Err(rail_failure);
        };
        pieces.push(CutPiece { from, to, edges });
    }
    let trimmed_rail = trimmer::cut_face(topo, rail_face, &wire, &pieces, spine_edges)?;
    replacements.insert(chain.rail, trimmed_rail);

    // The band: along the rail contact, across the end section, back along
    // the supports and across the start section.
    let rail_from = nodes[0].vertex;
    let rail_to = nodes[nodes.len() - 1].vertex;
    let support_from = support_ends[0].0;
    let support_to = support_ends[support_ends.len() - 1].1;
    let start_arc = make_end_arc(
        topo,
        support_from,
        rail_from,
        first_section.center,
        spine.tangent(topo, 0.0)?,
        radius,
        spine.evaluate(topo, 0.0)?,
    )?
    .ok_or(BlendError::TrimmingFailure { face: chain.rail })?;
    let end_arc = make_end_arc(
        topo,
        rail_to,
        support_to,
        last_section.center,
        spine.tangent(topo, length)?,
        radius,
        spine.evaluate(topo, length)?,
    )?
    .ok_or(BlendError::TrimmingFailure { face: chain.rail })?;
    let mut loop_edges: Vec<OrientedEdge> = rail_edges
        .iter()
        .map(|&edge| OrientedEdge::new(edge, true))
        .collect();
    loop_edges.push(OrientedEdge::new(
        end_arc,
        topo.edge(end_arc)?.start() == rail_to,
    ));
    loop_edges.extend(
        support_edges
            .iter()
            .rev()
            .map(|&edge| OrientedEdge::new(edge, false)),
    );
    loop_edges.push(OrientedEdge::new(
        start_arc,
        topo.edge(start_arc)?.start() == support_from,
    ));
    // Every contact edge must run against its trimmed face's use of it; the
    // trims fix which way that is.
    match effective_use(topo, trimmed_rail, rail_edges[0])? {
        Some(true) => loop_edges = reversed_loop(&loop_edges),
        Some(false) => {}
        None => return Err(BlendError::TrimmingFailure { face: chain.rail }),
    }
    for (support, &edge) in chain.supports.iter().zip(&support_edges) {
        let along_band = loop_edges
            .iter()
            .find(|oe| oe.edge() == edge)
            .map(OrientedEdge::is_forward);
        let along_face = effective_use(topo, current(&replacements, support.face), edge)?;
        if along_face.is_none() || along_band == along_face {
            return Err(BlendError::TrimmingFailure { face: support.face });
        }
    }
    // Concave: the band faces the ball's centre.
    let middle = &sections[count / 2];
    let (u, v) = (0.5, station(count / 2));
    let point = band
        .evaluate(u, v)
        .ok_or(BlendError::TrimmingFailure { face: chain.rail })?;
    let reversed = band.normal(u, v).dot(middle.center - point) < 0.0;
    if reversed {
        loop_edges = reversed_loop(&loop_edges);
    }
    let effective_from =
        |loop_edges: &[OrientedEdge], arc: EdgeId| -> Result<VertexId, BlendError> {
            let forward = loop_edges
                .iter()
                .find(|oe| oe.edge() == arc)
                .is_some_and(|oe| oe.is_forward() != reversed);
            let edge = topo.edge(arc)?;
            Ok(if forward { edge.start() } else { edge.end() })
        };
    let start_from = effective_from(&loop_edges, start_arc)?;
    let end_from = effective_from(&loop_edges, end_arc)?;
    let band_wire = topo.add_wire(Wire::new(loop_edges, true)?);
    let mut band_face = Face::new(band_wire, Vec::new(), band);
    band_face.set_reversed(reversed);
    let band_face = topo.add_face(band_face);

    let mut supports_faces: Vec<FaceId> = vec![chain.rail];
    supports_faces.extend(chain.supports.iter().map(|support| support.face));
    let mut created = vec![(band_face, supports_faces)];
    let ends = [
        (
            v_start,
            start_arc,
            start_from,
            rail_from,
            support_from,
            spine.tangent(topo, 0.0)?,
            chain.supports[0].face,
        ),
        (
            v_end,
            end_arc,
            end_from,
            rail_to,
            support_to,
            spine.tangent(topo, length)?,
            chain.supports[chain.supports.len() - 1].face,
        ),
    ];
    for (vertex, arc, arc_from, rail_vertex, support_vertex, normal, support) in ends {
        let mut edit = scope.edit(&mut replacements);
        let live = live_boundary_wires(topo, solid, edit.replacements());
        if let Some(patch) = stitch_end(
            topo,
            spine_edges,
            vertex,
            arc,
            arc_from,
            rail_vertex,
            support_vertex,
            normal,
            &live,
            &mut edit,
        )? {
            created.push((patch, vec![chain.rail, support]));
        }
    }

    let mut result_faces: Vec<FaceId> = original_faces
        .iter()
        .map(|&face| current(&replacements, face))
        .collect();
    result_faces.extend(created.iter().map(|(face, _)| *face));
    let shell = topo.add_shell(Shell::new(result_faces)?);
    let result = topo.add_solid(Solid::new(shell, inner_shells.to_vec()));
    Ok(Some(BlendResult {
        solid: result,
        succeeded: requested.to_vec(),
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
        engine: crate::BlendEngine::Walking,
    }))
}

/// The rail and its supports, or `None` when `spine` is not an open concave
/// chain with one face common to every edge and two or more others in turn.
fn detect(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    spine: &Spine,
) -> Result<Option<RailChain>, BlendError> {
    let edges = spine.edges();
    if edges.len() < 2 || spine.is_closed() {
        return Ok(None);
    }
    let mut pairs = Vec::with_capacity(edges.len());
    for &edge in edges {
        let &[a, b] = adjacency.faces_for_edge(edge) else {
            return Ok(None);
        };
        if a == b {
            return Ok(None);
        }
        pairs.push([a, b]);
    }
    let common: Vec<FaceId> = pairs[0]
        .iter()
        .copied()
        .filter(|face| pairs.iter().all(|pair| pair.contains(face)))
        .collect();
    let &[rail] = common.as_slice() else {
        return Ok(None);
    };
    if !matches!(
        topo.face(rail)?.surface(),
        FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
    ) {
        return Ok(None);
    }
    let mut supports: Vec<Support> = Vec::new();
    for (index, pair) in pairs.iter().enumerate() {
        let other = if pair[0] == rail { pair[1] } else { pair[0] };
        match supports.last_mut() {
            Some(last) if last.face == other => last.edges.end = index + 1,
            _ => {
                if supports.iter().any(|support| support.face == other) {
                    return Ok(None);
                }
                supports.push(Support {
                    face: other,
                    edges: index..index + 1,
                });
            }
        }
    }
    if supports.len() < 2 {
        return Ok(None);
    }
    for support in &supports {
        let frames = edge_material_frames_at(topo, edges[support.edges.start], rail, support.face)?;
        if frames.and_then(|frames| frames.concave()) != Some(true) {
            return Ok(None);
        }
    }
    Ok(Some(RailChain { rail, supports }))
}

/// Walk the ball from the chain's start to its end, against each support in
/// turn.
#[allow(clippy::too_many_lines)]
fn march(
    report_edge: EdgeId,
    length: f64,
    walkers: &[Walker<'_, ConstRadBlend>],
    supports: &[&dyn ParametricSurface],
    rail_cylinder: Option<&CylindricalSurface>,
    rail_seams: &[f64],
    past_seam: impl Fn(usize, Point3) -> Result<f64, BlendError>,
) -> Result<March, BlendError> {
    let failure = |s: f64| BlendError::WalkingFailure {
        edge: report_edge,
        t: s / length,
        residual: f64::NAN,
    };
    let solve = |segment: usize, s: f64, guess: BlendParams| {
        walkers[segment]
            .solve_section(s, Some(guess))
            .or_else(|| walkers[segment].solve_section(s, None))
    };
    // Signed angle of a rail contact from a seam edge of the rail.
    let seam_angle = |point: Point3, seam: f64| {
        rail_cylinder.map_or(0.0, |cylinder| {
            let angle = (cylinder.project_point(point).0 - seam).rem_euclid(TAU);
            if angle > PI { angle - TAU } else { angle }
        })
    };
    // The seam the rail contact crosses from `a` to `b`: opposite signs near
    // it, not across the half turn.
    let crosses = |a: &CircSection, b: &CircSection| {
        rail_seams.iter().copied().find(|&seam| {
            let (x, y) = (seam_angle(a.p1, seam), seam_angle(b.p1, seam));
            (x < 0.0) != (y < 0.0) && x.abs() < FRAC_PI_2 && y.abs() < FRAC_PI_2
        })
    };
    let resolution = 1e-13 * length.max(1.0);
    let min_step = 1e-9 * length.max(1.0);
    let max_step = MAX_STEP_FRACTION * length;

    let (mut params, first) =
        walkers[0]
            .solve_section(0.0, None)
            .ok_or(BlendError::StartSolutionFailure {
                edge: report_edge,
                t: 0.0,
            })?;
    if walkers.len() > 1 && past_seam(0, first.p2)? > 0.0 {
        return Err(failure(0.0));
    }
    let mut march = March {
        sections: vec![first.clone()],
        ends: Vec::with_capacity(walkers.len()),
        rail_crossings: Vec::new(),
    };
    let mut segment = 0;
    let mut s = 0.0;
    let mut current = first;
    let mut previous: Option<(f64, BlendParams)> = None;
    let mut step = max_step;

    #[allow(clippy::while_float)]
    while length - s > min_step {
        let next_s = (s + step).min(length);
        let guess = previous.map_or(params, |(previous_s, previous_params)| {
            let ratio = (next_s - s) / (s - previous_s);
            BlendParams {
                u1: ratio.mul_add(params.u1 - previous_params.u1, params.u1),
                v1: ratio.mul_add(params.v1 - previous_params.v1, params.v1),
                u2: ratio.mul_add(params.u2 - previous_params.u2, params.u2),
                v2: ratio.mul_add(params.v2 - previous_params.v2, params.v2),
            }
        });
        let Some((next_params, next)) = solve(segment, next_s, guess) else {
            step *= 0.5;
            if step < min_step {
                return Err(failure(next_s));
            }
            continue;
        };
        if turn(&current, &next) > MAX_TURN && step > 64.0 * min_step {
            step *= 0.5;
            continue;
        }

        if segment + 1 < walkers.len() && past_seam(segment, next.p2)? > 0.0 {
            // The contact left this support between `s` and `next_s`: find
            // where it reaches the seam, and hand over there. The handover is
            // the last station still on this support. Where the chain is only
            // nearly tangent at a vertex (a boolean's fitted intersection
            // edge), the section plane turns there and the contact jumps
            // across the seam instead of reaching it; the station before the
            // jump is the one on the seam.
            let mut high = next_s;
            let mut joint = (s, params, current.clone());
            if past_seam(segment, current.p2)? < 0.0 {
                for _ in 0..BISECTIONS {
                    let mid = f64::midpoint(joint.0, high);
                    let (mid_params, section) =
                        solve(segment, mid, joint.1).ok_or_else(|| failure(mid))?;
                    let offset = past_seam(segment, section.p2)?;
                    if offset <= 0.0 {
                        joint = (mid, mid_params, section);
                    } else {
                        high = mid;
                    }
                    if offset == 0.0 || high - joint.0 <= resolution {
                        break;
                    }
                }
            }
            let (joint_s, joint_params, joint_section) = joint;
            if joint_s - s > min_step {
                if let Some(seam) = crosses(&current, &joint_section) {
                    march.rail_crossings.push((march.sections.len() - 1, seam));
                }
                march.sections.push(joint_section.clone());
            }
            march.ends.push(march.sections.len() - 1);
            segment += 1;
            // The next support takes the ball over where it touches both.
            let (u2, v2) = supports[segment].project_point(joint_section.p2);
            let handover = BlendParams {
                u2,
                v2,
                ..joint_params
            };
            let (handed_params, handed) =
                solve(segment, joint_s, handover).ok_or_else(|| failure(joint_s))?;
            let join = 1e-5 * handed.radius.max(1.0);
            if (handed.p1 - joint_section.p1).length() > join
                || (handed.p2 - joint_section.p2).length() > join
            {
                return Err(failure(joint_s));
            }
            params = handed_params;
            s = joint_s;
            current = joint_section;
            previous = None;
            step = max_step;
            continue;
        }

        if let Some(seam) = crosses(&current, &next) {
            march.rail_crossings.push((march.sections.len() - 1, seam));
        }
        march.sections.push(next.clone());
        previous = Some((s, params));
        params = next_params;
        s = next_s;
        current = next;
        step = (step * 1.5).min(max_step);
    }
    march.ends.push(march.sections.len() - 1);
    if march.ends.len() != walkers.len()
        || march.ends.windows(2).any(|pair| pair[1] <= pair[0])
        || march.ends[0] < 1
    {
        // A segment too short to carry a contact of its own.
        return Err(failure(length));
    }
    Ok(march)
}

/// The angular positions of the rail's seam edges: rulings its boundary runs
/// along once each way, which a contact crossing them must be split at.
fn seam_angles(
    topo: &Topology,
    rail: FaceId,
    cylinder: &CylindricalSurface,
) -> Result<Vec<f64>, BlendError> {
    let uses = topo.wire(topo.face(rail)?.outer_wire())?.edges().to_vec();
    let mut angles = Vec::new();
    for (index, oriented) in uses.iter().enumerate() {
        if uses[..index]
            .iter()
            .any(|seen| seen.edge() == oriented.edge())
            || uses
                .iter()
                .filter(|other| other.edge() == oriented.edge())
                .count()
                != 2
        {
            continue;
        }
        let edge = topo.edge(oriented.edge())?;
        if matches!(edge.curve(), EdgeCurve::Line) {
            angles.push(cylinder.project_point(topo.vertex(edge.start())?.point()).0);
        }
    }
    Ok(angles)
}

/// Where the chord from `a` to `b` crosses the half-plane of the cylinder's
/// ruling at angle `seam`, as a fraction of the chord.
///
/// The crossing is not a station of its own: a station there would change
/// the band, which must not depend on where the rail's parameter seam lies.
fn seam_plane_crossing(
    cylinder: &CylindricalSurface,
    seam: f64,
    a: Point3,
    b: Point3,
) -> Option<f64> {
    let origin = cylinder.origin();
    let radial = (cylinder.evaluate(seam, 0.0) - origin).normalize().ok()?;
    let normal = cylinder.axis().cross(radial);
    let (da, db) = ((a - origin).dot(normal), (b - origin).dot(normal));
    // Opposite sides, or one end on the half-plane.
    if da * db > 0.0 || (da - db).abs() <= f64::EPSILON * da.abs().max(db.abs()) {
        return None;
    }
    Some((da / (da - db)).clamp(0.0, 1.0))
}

/// Distance of `point` from the cylinder's axis.
fn radial_distance(cylinder: &CylindricalSurface, point: Point3) -> f64 {
    let offset = point - cylinder.origin();
    (offset - cylinder.axis() * cylinder.axis().dot(offset)).length()
}

/// The larger angle either contact direction turns through between two
/// sections.
pub fn turn(a: &CircSection, b: &CircSection) -> f64 {
    let angle = |x: Vec3, y: Vec3| x.cross(y).length().atan2(x.dot(y));
    angle(a.p1 - a.center, b.p1 - b.center).max(angle(a.p2 - a.center, b.p2 - b.center))
}

/// The seam `from` and `to` share, as `to` uses it: of their common edges, the
/// one nearest `near`.
pub fn support_seam(
    topo: &Topology,
    from: FaceId,
    to: FaceId,
    near: Point3,
) -> Result<Option<SupportSeam>, BlendError> {
    let from_edges: Vec<EdgeId> = topo
        .wire(topo.face(from)?.outer_wire())?
        .edges()
        .iter()
        .map(OrientedEdge::edge)
        .collect();
    let mut best: Option<(OrientedEdge, f64)> = None;
    for oriented in topo.wire(topo.face(to)?.outer_wire())?.edges() {
        if !from_edges.contains(&oriented.edge()) {
            continue;
        }
        let (_, _, distance) = trimmer::closest_on_oriented_edge(topo, *oriented, near)?;
        if best.is_none_or(|(_, closest)| distance < closest) {
            best = Some((*oriented, distance));
        }
    }
    Ok(best.map(|(edge, _)| SupportSeam { next: to, edge }))
}

/// How far `point` lies past `seam` onto the later support, along that
/// support's surface: negative while it is still on the earlier one. `None`
/// when the frame at the seam is undefined.
pub fn seam_offset(
    topo: &Topology,
    seam: &SupportSeam,
    point: Point3,
) -> Result<Option<f64>, BlendError> {
    let (fraction, foot, _) = trimmer::closest_on_oriented_edge(topo, seam.edge, point)?;
    let edge = topo.edge(seam.edge.edge())?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let tangent = if matches!(edge.curve(), EdgeCurve::Line) {
        end - start
    } else {
        let (t0, t1) = edge.strict_domain().map_err(crate::edge_domain_input)?;
        let parameter = if seam.edge.is_forward() {
            (t1 - t0).mul_add(fraction, t0)
        } else {
            (t0 - t1).mul_add(fraction, t1)
        };
        edge.curve().tangent_with_endpoints(parameter, start, end)
    };
    let face = topo.face(seam.next)?;
    // The traversal direction the face makes of the edge: its wire flag,
    // turned round on a reversed face.
    let along = if seam.edge.is_forward() == face.is_reversed() {
        -tangent
    } else {
        tangent
    };
    let raw = match face.surface() {
        FaceSurface::Plane { normal, .. } => *normal,
        surface => {
            let Some((u, v)) = surface.project_point(foot) else {
                return Ok(None);
            };
            surface.normal(u, v)
        }
    };
    let outward = if face.is_reversed() { -raw } else { raw };
    let Ok(into) = outward.cross(along).normalize() else {
        return Ok(None);
    };
    Ok(Some((point - foot).dot(into)))
}

/// The chain's first and last vertices.
pub fn chain_ends(topo: &Topology, edges: &[EdgeId]) -> Result<(VertexId, VertexId), BlendError> {
    let first = topo.edge(edges[0])?;
    let second = topo.edge(edges[1])?;
    let start = if [second.start(), second.end()].contains(&first.start()) {
        first.end()
    } else {
        first.start()
    };
    let last = topo.edge(edges[edges.len() - 1])?;
    let before = topo.edge(edges[edges.len() - 2])?;
    let end = if [before.start(), before.end()].contains(&last.start()) {
        last.end()
    } else {
        last.start()
    };
    Ok((start, end))
}

/// The vertex two consecutive chain edges meet at.
pub fn shared_vertex(topo: &Topology, a: EdgeId, b: EdgeId) -> Result<VertexId, BlendError> {
    let a = topo.edge(a)?;
    let b = topo.edge(b)?;
    if [b.start(), b.end()].contains(&a.start()) {
        Ok(a.start())
    } else {
        Ok(a.end())
    }
}

/// The edge along the rail from `from` to `to`, two points of one end
/// section: a ruling of a cylinder whose axis lies in the end face (the
/// boss's axis in the block's back face), the arc round it when the end face
/// is perpendicular to the axis, or a line on a plane. `None` for anything
/// else.
fn closure_edge(
    topo: &mut Topology,
    rail: &FaceSurface,
    from: VertexId,
    to: VertexId,
) -> Result<Option<EdgeId>, BlendError> {
    let a = topo.vertex(from)?.point();
    let b = topo.vertex(to)?.point();
    let cylinder = match rail {
        FaceSurface::Plane { .. } => {
            return Ok(Some(topo.add_edge(Edge::new(from, to, EdgeCurve::Line))));
        }
        FaceSurface::Cylinder(cylinder) => cylinder,
        FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => return Ok(None),
    };
    let axis = cylinder.axis();
    let origin = cylinder.origin();
    let radial = |p: Point3| {
        let d = p - origin;
        (d - axis * axis.dot(d)).normalize()
    };
    let (Ok(ra), Ok(rb)) = (radial(a), radial(b)) else {
        return Ok(None);
    };
    let tolerance = 1e-6;
    if ra.cross(rb).length() <= tolerance && ra.dot(rb) > 0.0 {
        return Ok(Some(topo.add_edge(Edge::new(from, to, EdgeCurve::Line))));
    }
    let height = axis.dot(b - origin);
    if (axis.dot(a - origin) - height).abs() > tolerance * cylinder.radius().max(1.0) {
        return Ok(None);
    }
    let Ok(circle) = Circle3D::new(origin + axis * height, axis, cylinder.radius()) else {
        return Ok(None);
    };
    let t_from = circle.project(a);
    // The short way round: a fillet never sets back half a turn.
    let delta = (circle.project(b) - t_from + PI).rem_euclid(TAU) - PI;
    let edge = if delta > 0.0 {
        add_certified_curve_edge(
            topo,
            from,
            to,
            EdgeCurve::Circle(circle),
            (t_from, t_from + delta),
        )?
    } else {
        add_certified_curve_edge(
            topo,
            to,
            from,
            EdgeCurve::Circle(circle),
            (t_from + delta, t_from),
        )?
    };
    Ok(Some(edge))
}

/// Whether `face` traverses `edge` forward (wire flag, turned round on a
/// reversed face); `None` when it does not use it.
pub fn effective_use(
    topo: &Topology,
    face: FaceId,
    edge: EdgeId,
) -> Result<Option<bool>, BlendError> {
    let face = topo.face(face)?;
    Ok(topo
        .wire(face.outer_wire())?
        .edges()
        .iter()
        .find(|oe| oe.edge() == edge)
        .map(|oe| oe.is_forward() != face.is_reversed()))
}

/// The same loop traversed the other way.
pub fn reversed_loop(edges: &[OrientedEdge]) -> Vec<OrientedEdge> {
    edges
        .iter()
        .rev()
        .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
        .collect()
}
