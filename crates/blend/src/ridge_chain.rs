//! One blend band along a tangent chain whose faces change on both sides.
//!
//! After a boss's seam with a block has been filleted, the block's sharp top
//! edge no longer ends at the boss. It runs into the seam's torus band and
//! carries on tangentially as the band's end section on the front wall, then
//! up the mitre between the band and the wall's own seam cylinder, until it
//! fades out where those two meet the boss tangentially. That is one smooth
//! ridgeline, but its faces change on both sides along the way: the wall
//! hands over to the seam cylinder on one side, the top to the torus on the
//! other. A stripe is computed against one face pair, so the walking builder
//! refuses it ([`BlendError::UnsupportedSeamCrossing`]), and
//! [`crate::rail_chain`] only covers a chain that keeps one face throughout.
//!
//! The ball rolls against one face on each side at a time and leaves a face
//! where its contact crosses the tangent seam onto the next one: a station of
//! its own, not a chain vertex (the ball reaches the torus before the chain
//! does). Each side's switches are found by bisection, so at each the ball
//! touches both faces at one seam point and the two runs share that section.
//! Where the chain ends on a vertex at which its last two faces are tangent,
//! the band runs out there: the ball's contacts close onto that vertex and the
//! band ends in a point instead of a section.
//!
//! The band is one NURBS surface through every section, linear between
//! stations, so its long boundaries are exactly the contact polylines the
//! faces are trimmed along; like every walked band it is an approximation. A
//! chamfer is the ruled surface between the contacts of the ball whose chord
//! it is, which sets back the requested distance from a right-angled edge and
//! runs out with the fillet's contacts.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::analytic::edge_material_frames_at;
use crate::blend_func::{BlendFunction, BlendParams, ChamferBlend, ConstRadBlend};
use crate::body_edit::EditScope;
use crate::builder_utils::{
    FlippedNormalSurface, PlaneAdapter, add_certified_curve_edge, surface_ref_or_adapter,
};
use crate::fillet_builder::{
    ball_side, live_boundary_wires, make_end_arc, sections_to_contact_curve, stitch_end,
};
use crate::rail_chain::{
    SupportSeam, chain_ends, effective_use, reversed_loop, seam_offset, shared_vertex,
    support_seam, turn,
};
use crate::section::CircSection;
use crate::spine::Spine;
use crate::trimmer::{self, CutPiece};
use crate::walker::{Walker, WalkerConfig, approximate_blend_surface_linear_v};
use crate::{BlendError, BlendFaceOrigins, BlendResult};

/// The blend to put along the ridge.
#[derive(Debug, Clone, Copy)]
pub enum RidgeBlend {
    /// A rolling-ball fillet of this radius.
    Fillet(f64),
    /// A chamfer setting back this far along both faces of a right-angled
    /// edge: the chord of the ball of that radius.
    Chamfer(f64),
}

impl RidgeBlend {
    fn size(self) -> f64 {
        match self {
            Self::Fillet(size) | Self::Chamfer(size) => size,
        }
    }
}

/// Largest station step, as a fraction of the chain's length (the walker's).
const MAX_STEP_FRACTION: f64 = 0.05;
/// Largest turn of either contact direction between consecutive stations
/// (2°): it bounds how far the band's linear spans sag off the true blend.
const MAX_TURN: f64 = 0.035;
/// Largest distance, relative to the blend's size, between a section solved
/// halfway between two stations and the straight interpolation of theirs:
/// it bounds how far the band's linear spans sag off the true blend where its
/// sections move and twist rather than turn (up a mitre).
const MAX_SAG: f64 = 1e-4;
/// Bisection steps for a face switch.
const BISECTIONS: usize = 80;
/// Normals closer than this (as `1 − cos`) make the faces tangent at a chain
/// end, so the band runs out there.
const TANGENT_END: f64 = 1e-9;
/// The run-out march stops once the band is this narrow, relative to its
/// size, and closes onto the end vertex.
const RUNOUT_WIDTH: f64 = 1e-4;

/// The faces along one side of the chain, in march order.
struct Side {
    faces: Vec<FaceId>,
    /// `seams[i]` is where `faces[i]` hands over to `faces[i + 1]`.
    seams: Vec<SupportSeam>,
}

/// The chain's two sides, oriented in march order.
struct Ridge {
    sides: [Side; 2],
    concave: bool,
    /// The march runs from the spine's end to its start.
    reversed: bool,
    /// The chain end the march stops at is a run-out vertex.
    runout: bool,
}

/// The sections of the march.
struct March {
    sections: Vec<CircSection>,
    /// For each side, the index of the section where it hands over to its
    /// next face; the next run starts there.
    switches: [Vec<usize>; 2],
}

/// Blend each chain in `chains` as one band, in turn, when every one of them
/// is a ridge whose faces change along the way; `None` when one is not.
///
/// Each chain is blended on the result of the one before, so chains that do
/// not share a face (the block's two top edges into a seam band) come off
/// independently.
///
/// # Errors
///
/// Returns [`BlendError`] when a chain qualifies but its march, a trim or the
/// assembly fails.
#[allow(clippy::too_many_arguments)]
pub fn blend_all(
    topo: &mut Topology,
    solid: SolidId,
    chains: &[Vec<EdgeId>],
    blend: RidgeBlend,
    original_faces: &[FaceId],
    inner_shells: &[ShellId],
    requested: &[EdgeId],
) -> Result<Option<BlendResult>, BlendError> {
    if chains.is_empty() {
        return Ok(None);
    }
    // Every chain must qualify on the input before any is built.
    {
        let adjacency = topo.build_adjacency(solid)?;
        for chain in chains {
            let spine = Spine::from_chain(topo, chain.clone())?;
            if detect(topo, &adjacency, &spine)?.is_none() {
                return Ok(None);
            }
        }
    }
    let mut current = solid;
    let mut current_faces = original_faces.to_vec();
    let mut origins: Option<BlendFaceOrigins> = None;
    for chain in chains {
        let adjacency = topo.build_adjacency(current)?;
        let spine = Spine::from_chain(topo, chain.clone())?;
        let mut scope = EditScope::new(topo, current_faces.iter().copied())?;
        let Some(result) = blend_one(
            topo,
            current,
            &adjacency,
            &spine,
            blend,
            &mut scope,
            &current_faces,
            inner_shells,
        )?
        else {
            return Ok(None);
        };
        let Some(step) = result.face_origins else {
            return Ok(None);
        };
        origins = Some(match origins {
            None => step,
            Some(before) => compose(&before, &step),
        });
        current = result.solid;
        current_faces = remus_topology::explorer::solid_faces(topo, current)?;
    }
    Ok(Some(BlendResult {
        solid: current,
        succeeded: requested.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: origins,
        engine: crate::BlendEngine::Walking,
    }))
}

/// The face lineage of two blends applied one after the other.
fn compose(first: &BlendFaceOrigins, second: &BlendFaceOrigins) -> BlendFaceOrigins {
    let through = |face: FaceId| -> FaceId {
        second
            .survived
            .iter()
            .find(|(from, _)| *from == face)
            .map_or(face, |&(_, to)| to)
    };
    let mut created: Vec<(FaceId, Vec<FaceId>)> = first
        .created
        .iter()
        .map(|(face, sources)| (through(*face), sources.clone()))
        .collect();
    // A face the second blend made from one the first blend made traces back
    // to the first blend's sources; one made from an input face, to it.
    for (face, sources) in &second.created {
        let mut traced = Vec::new();
        for &source in sources {
            match first
                .survived
                .iter()
                .find(|(_, to)| *to == source)
                .map(|&(from, _)| from)
            {
                Some(original) => traced.push(original),
                None => {
                    if let Some((_, earlier)) =
                        first.created.iter().find(|(made, _)| *made == source)
                    {
                        traced.extend(earlier.iter().copied());
                    }
                }
            }
        }
        created.push((*face, traced));
    }
    BlendFaceOrigins {
        survived: first
            .survived
            .iter()
            .map(|&(from, to)| (from, through(to)))
            .collect(),
        deleted: first
            .deleted
            .iter()
            .chain(&second.deleted)
            .copied()
            .collect(),
        created,
        created_unattributed: first
            .created_unattributed
            .iter()
            .chain(&second.created_unattributed)
            .copied()
            .collect(),
    }
}

/// Blend `spine` as one band when it is a ridge whose faces change along the
/// way, or `None` when it is not one.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn blend_one(
    topo: &mut Topology,
    solid: SolidId,
    adjacency: &AdjacencyIndex,
    spine: &Spine,
    blend: RidgeBlend,
    scope: &mut EditScope,
    original_faces: &[FaceId],
    inner_shells: &[ShellId],
) -> Result<Option<BlendResult>, BlendError> {
    let Some(ridge) = detect(topo, adjacency, spine)? else {
        return Ok(None);
    };
    let spine_edges = spine.edges();
    let length = spine.length();
    let size = blend.size();
    let (spine_start, spine_end) = chain_ends(topo, spine_edges)?;
    let (v_start, v_end) = if ridge.reversed {
        (spine_end, spine_start)
    } else {
        (spine_start, spine_end)
    };
    let s_of = |d: f64| if ridge.reversed { length - d } else { d };

    let march = {
        let mut surfaces: [Vec<(FaceSurface, bool)>; 2] = [Vec::new(), Vec::new()];
        for (side, list) in ridge.sides.iter().zip(surfaces.iter_mut()) {
            for &face in &side.faces {
                let face = topo.face(face)?;
                list.push((face.surface().clone(), face.is_reversed()));
            }
        }
        let mut adapters: [Vec<Option<PlaneAdapter>>; 2] = [
            surfaces[0].iter().map(|_| None).collect(),
            surfaces[1].iter().map(|_| None).collect(),
        ];
        let [adapters_a, adapters_b] = &mut adapters;
        let bases: [Vec<&dyn ParametricSurface>; 2] = [
            surfaces[0]
                .iter()
                .zip(adapters_a.iter_mut())
                .map(|((surface, _), adapter)| surface_ref_or_adapter(surface, adapter))
                .collect(),
            surfaces[1]
                .iter()
                .zip(adapters_b.iter_mut())
                .map(|((surface, _), adapter)| surface_ref_or_adapter(surface, adapter))
                .collect(),
        ];
        let flipped: [Vec<FlippedNormalSurface<'_>>; 2] = [
            bases[0]
                .iter()
                .map(|&base| FlippedNormalSurface::new(base))
                .collect(),
            bases[1]
                .iter()
                .map(|&base| FlippedNormalSurface::new(base))
                .collect(),
        ];
        let side_surfaces = |k: usize| -> Vec<&dyn ParametricSurface> {
            bases[k]
                .iter()
                .zip(&flipped[k])
                .zip(&surfaces[k])
                .map(|((&base, flipped), (_, reversed))| {
                    ball_side(base, flipped, *reversed, ridge.concave)
                })
                .collect()
        };
        let sides = [side_surfaces(0), side_surfaces(1)];
        let config = WalkerConfig {
            // Quadric supports; see `compute_stripe_for_spine`.
            tol_3d: 5.0 * remus_math::tolerance::Tolerance::new().linear,
            ..WalkerConfig::default()
        };
        let context = MarchContext {
            topo,
            ridge: &ridge,
            spine,
            sides: &sides,
            config,
            s_of: &s_of,
        };
        match blend {
            RidgeBlend::Fillet(radius) => march(&context, &ConstRadBlend { radius }, size)?,
            RidgeBlend::Chamfer(distance) => march(
                &context,
                &ChamferBlend {
                    d1: distance,
                    d2: distance,
                },
                size,
            )?,
        }
    };
    let mut sections = march.sections;
    let count = sections.len();
    if ridge.runout {
        // The ball's contacts close onto the end vertex.
        let point = topo.vertex(v_end)?.point();
        let last = &sections[count - 1];
        let chord_mid = Point3::new(
            f64::midpoint(last.p1.x(), last.p2.x()),
            f64::midpoint(last.p1.y(), last.p2.y()),
            f64::midpoint(last.p1.z(), last.p2.z()),
        );
        let center = point + (last.center - chord_mid);
        sections[count - 1] = CircSection {
            p1: point,
            p2: point,
            center,
            radius: last.radius,
            uv1: last.uv1,
            uv2: last.uv2,
            t: last.t,
        };
    }

    // Every run starts and ends on its face's boundary: an end edge, or the
    // seam it hands over at. The walker leaves the contact within its own
    // tolerance of that edge; put it on the edge, so the vertex the trims
    // split it at is certified.
    let touch = 1e-5 * size.max(1.0);
    for k in 0..2 {
        let faces = &ridge.sides[k].faces;
        let mut stops = vec![(0, faces[0])];
        for (run, &at) in march.switches[k].iter().enumerate() {
            stops.push((at, faces[run]));
        }
        if !ridge.runout {
            stops.push((count - 1, faces[faces.len() - 1]));
        }
        for (at, face) in stops {
            let section = &mut sections[at];
            let contact = if k == 0 {
                &mut section.p1
            } else {
                &mut section.p2
            };
            if let Some(foot) = boundary_foot(topo, face, *contact, touch)? {
                *contact = foot;
            }
        }
    }

    #[allow(clippy::cast_precision_loss)]
    let station = |index: usize| index as f64 / (count - 1) as f64;
    log::debug!(
        target: "remus_approx",
        "blend: ridge chain over {}+{} faces — {count} walked sections, approximate NURBS blend surface",
        ridge.sides[0].faces.len(),
        ridge.sides[1].faces.len(),
    );
    let contacts = [
        sections_to_contact_curve(&sections, |section| section.p1)?,
        sections_to_contact_curve(&sections, |section| section.p2)?,
    ];
    let band = match blend {
        RidgeBlend::Fillet(_) => FaceSurface::Nurbs(approximate_blend_surface_linear_v(&sections)?),
        RidgeBlend::Chamfer(_) => FaceSurface::Nurbs(ruled_band(&sections)?),
    };
    let mut replacements = remus_math::det_hash::DetHashMap::default();
    let current = |replacements: &remus_math::det_hash::DetHashMap<FaceId, FaceId>,
                   face: FaceId| replacements.get(&face).copied().unwrap_or(face);
    let pick = |section: &CircSection, k: usize| if k == 0 { section.p1 } else { section.p2 };

    // Each side's faces, in march order: each run ends where the next one
    // starts, on the vertex the earlier trim made.
    let mut side_edges: [Vec<EdgeId>; 2] = [Vec::new(), Vec::new()];
    let mut side_ends: [(VertexId, VertexId); 2] = [(v_start, v_start), (v_start, v_start)];
    for k in 0..2 {
        let side = &ridge.sides[k];
        let runs = side.faces.len();
        let mut first_vertex = None;
        let mut last_vertex = None;
        for (index, &original) in side.faces.iter().enumerate() {
            let first = if index == 0 {
                0
            } else {
                march.switches[k][index - 1]
            };
            let last = if index + 1 == runs {
                count - 1
            } else {
                march.switches[k][index]
            };
            let face = current(&replacements, original);
            let mut wire = topo.wire(topo.face(face)?.outer_wire())?.edges().to_vec();
            let failure = BlendError::TrimmingFailure { face: original };
            let (from_vertex, to_vertex) = {
                let mut edit = scope.edit(&mut replacements);
                let from = trimmer::boundary_vertex_at(
                    topo,
                    &mut edit,
                    face,
                    &mut wire,
                    pick(&sections[first], k),
                    touch,
                    spine_edges,
                )?;
                let to = trimmer::boundary_vertex_at(
                    topo,
                    &mut edit,
                    face,
                    &mut wire,
                    pick(&sections[last], k),
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
                EdgeCurve::NurbsCurve(contacts[k].clone()),
                (station(first), station(last)),
            )?;
            let (Some(from), Some(to)) = (
                trimmer::vertex_occurrence(
                    topo,
                    face,
                    &wire,
                    from_vertex,
                    pick(&sections[first + 1], k),
                )?,
                trimmer::vertex_occurrence(
                    topo,
                    face,
                    &wire,
                    to_vertex,
                    pick(&sections[last - 1], k),
                )?,
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
            replacements.insert(original, trimmed);
            side_edges[k].push(edge);
            first_vertex.get_or_insert(from_vertex);
            last_vertex = Some(to_vertex);
        }
        let (Some(first), Some(last)) = (first_vertex, last_vertex) else {
            return Err(BlendError::TrimmingFailure {
                face: side.faces[0],
            });
        };
        side_ends[k] = (first, last);
    }
    if ridge.runout && side_ends[0].1 != side_ends[1].1 {
        return Err(BlendError::TrimmingFailure {
            face: ridge.sides[0].faces[ridge.sides[0].faces.len() - 1],
        });
    }

    // The band: along side 0, across the end section (none at a run-out),
    // back along side 1 and across the start section.
    let (first_section, last_section) = (&sections[0], &sections[count - 1]);
    let start_tangent = spine.tangent(topo, s_of(0.0))?;
    let end_tangent = spine.tangent(topo, s_of(length))?;
    let start_edge = end_edge(
        topo,
        blend,
        side_ends[1].0,
        side_ends[0].0,
        first_section,
        start_tangent,
        spine.evaluate(topo, s_of(0.0))?,
    )?
    .ok_or(BlendError::TrimmingFailure {
        face: ridge.sides[0].faces[0],
    })?;
    let end = if ridge.runout {
        None
    } else {
        Some(
            end_edge(
                topo,
                blend,
                side_ends[0].1,
                side_ends[1].1,
                last_section,
                end_tangent,
                spine.evaluate(topo, s_of(length))?,
            )?
            .ok_or(BlendError::TrimmingFailure {
                face: ridge.sides[0].faces[0],
            })?,
        )
    };
    let mut loop_edges: Vec<OrientedEdge> = side_edges[0]
        .iter()
        .map(|&edge| OrientedEdge::new(edge, true))
        .collect();
    if let Some(end) = end {
        loop_edges.push(OrientedEdge::new(
            end,
            topo.edge(end)?.start() == side_ends[0].1,
        ));
    }
    loop_edges.extend(
        side_edges[1]
            .iter()
            .rev()
            .map(|&edge| OrientedEdge::new(edge, false)),
    );
    loop_edges.push(OrientedEdge::new(
        start_edge,
        topo.edge(start_edge)?.start() == side_ends[1].0,
    ));
    // Every contact edge must run against its trimmed face's use of it; the
    // trims fix which way that is.
    let first_face = current(&replacements, ridge.sides[0].faces[0]);
    match effective_use(topo, first_face, side_edges[0][0])? {
        Some(true) => loop_edges = reversed_loop(&loop_edges),
        Some(false) => {}
        None => {
            return Err(BlendError::TrimmingFailure {
                face: ridge.sides[0].faces[0],
            });
        }
    }
    for k in 0..2 {
        for (&original, &edge) in ridge.sides[k].faces.iter().zip(&side_edges[k]) {
            let along_band = loop_edges
                .iter()
                .find(|oe| oe.edge() == edge)
                .map(OrientedEdge::is_forward);
            let along_face = effective_use(topo, current(&replacements, original), edge)?;
            if along_face.is_none() || along_band == along_face {
                return Err(BlendError::TrimmingFailure { face: original });
            }
        }
    }
    // The band faces the ridge it replaces on a convex chain (the corner it
    // cuts away) and away from it on a concave one (the corner it fills). A
    // chamfer section has no ball centre to judge by, so both go by the
    // spine.
    let middle = &sections[count / 2];
    let (u, v) = (0.5, station(count / 2));
    let point = band.evaluate(u, v).ok_or(BlendError::TrimmingFailure {
        face: ridge.sides[0].faces[0],
    })?;
    let ridge_point = spine.evaluate(topo, middle.t * length)?;
    let toward_ridge = band.normal(u, v).dot(ridge_point - point) > 0.0;
    let reversed = toward_ridge == ridge.concave;
    if reversed {
        loop_edges = reversed_loop(&loop_edges);
    }
    let effective_from =
        |loop_edges: &[OrientedEdge], edge: EdgeId| -> Result<VertexId, BlendError> {
            let forward = loop_edges
                .iter()
                .find(|oe| oe.edge() == edge)
                .is_some_and(|oe| oe.is_forward() != reversed);
            let edge = topo.edge(edge)?;
            Ok(if forward { edge.start() } else { edge.end() })
        };
    let start_from = effective_from(&loop_edges, start_edge)?;
    let end_from = match end {
        Some(end) => Some(effective_from(&loop_edges, end)?),
        None => None,
    };
    let band_wire = topo.add_wire(Wire::new(loop_edges, true)?);
    let mut band_face = Face::new(band_wire, Vec::new(), band);
    band_face.set_reversed(reversed);
    let band_face = topo.add_face(band_face);

    let supports_faces: Vec<FaceId> = ridge
        .sides
        .iter()
        .flat_map(|side| side.faces.iter().copied())
        .collect();
    let mut created = vec![(band_face, supports_faces)];
    let mut ends = vec![(
        v_start,
        start_edge,
        start_from,
        side_ends[0].0,
        side_ends[1].0,
        start_tangent,
        [ridge.sides[0].faces[0], ridge.sides[1].faces[0]],
    )];
    if let (Some(end), Some(end_from)) = (end, end_from) {
        ends.push((
            v_end,
            end,
            end_from,
            side_ends[0].1,
            side_ends[1].1,
            end_tangent,
            [
                ridge.sides[0].faces[ridge.sides[0].faces.len() - 1],
                ridge.sides[1].faces[ridge.sides[1].faces.len() - 1],
            ],
        ));
    }
    for (vertex, edge, edge_from, vertex_a, vertex_b, normal, faces) in ends {
        let mut edit = scope.edit(&mut replacements);
        let live = live_boundary_wires(topo, solid, edit.replacements());
        if let Some(patch) = stitch_end(
            topo,
            spine_edges,
            vertex,
            edge,
            edge_from,
            vertex_a,
            vertex_b,
            normal,
            &live,
            &mut edit,
        )? {
            created.push((patch, faces.to_vec()));
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
        succeeded: spine_edges.to_vec(),
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

/// The edge across one end section, from `from` to `to`: the ball's arc for
/// a fillet, the chord for a chamfer.
fn end_edge(
    topo: &mut Topology,
    blend: RidgeBlend,
    from: VertexId,
    to: VertexId,
    section: &CircSection,
    tangent: Vec3,
    on_spine: Point3,
) -> Result<Option<EdgeId>, BlendError> {
    match blend {
        RidgeBlend::Fillet(radius) => {
            make_end_arc(topo, from, to, section.center, tangent, radius, on_spine)
        }
        RidgeBlend::Chamfer(_) => Ok(Some(topo.add_edge(Edge::new(from, to, EdgeCurve::Line)))),
    }
}

/// The ruled surface between the two contacts of every section, linear
/// between stations.
fn ruled_band(sections: &[CircSection]) -> Result<NurbsSurface, BlendError> {
    let n = sections.len();
    if n < 2 {
        return Err(BlendError::Math(remus_math::MathError::EmptyInput));
    }
    let mut knots_v = vec![0.0, 0.0];
    for i in 1..n - 1 {
        #[allow(clippy::cast_precision_loss)]
        knots_v.push(i as f64 / (n - 1) as f64);
    }
    knots_v.extend([1.0, 1.0]);
    Ok(NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        knots_v,
        vec![
            sections.iter().map(|section| section.p1).collect(),
            sections.iter().map(|section| section.p2).collect(),
        ],
        vec![vec![1.0; n], vec![1.0; n]],
    )?)
}

/// The two sides of `spine` when it is an open convex chain whose faces
/// change on both sides across tangent seams and which runs out at one end, or `None`
/// when it is not one.
fn detect(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    spine: &Spine,
) -> Result<Option<Ridge>, BlendError> {
    let edges = spine.edges();
    if edges.len() < 2 || spine.is_closed() {
        return Ok(None);
    }
    let mut pairs: Vec<[FaceId; 2]> = Vec::with_capacity(edges.len());
    for (index, &edge) in edges.iter().enumerate() {
        let &[a, b] = adjacency.faces_for_edge(edge) else {
            return Ok(None);
        };
        if a == b {
            return Ok(None);
        }
        let Some(&previous) = pairs.last() else {
            pairs.push([a, b]);
            continue;
        };
        let pair = if a == previous[0] || b == previous[1] {
            [a, b]
        } else if b == previous[0] || a == previous[1] {
            [b, a]
        } else {
            // Both faces change at one vertex: pair them by their normals
            // there.
            let joint = topo
                .vertex(shared_vertex(topo, edges[index - 1], edge)?)?
                .point();
            let normal = |face: FaceId| outward_normal(topo, face, joint);
            let (Some(p0), Some(na), Some(nb)) = (normal(previous[0]), normal(a), normal(b)) else {
                return Ok(None);
            };
            if p0.dot(na) >= p0.dot(nb) {
                [a, b]
            } else {
                [b, a]
            }
        };
        pairs.push(pair);
    }

    // Each side's faces, in spine order, with the chain vertex where each
    // hands over to the next.
    let mut faces: [Vec<FaceId>; 2] = [Vec::new(), Vec::new()];
    let mut joints: [Vec<VertexId>; 2] = [Vec::new(), Vec::new()];
    for (index, pair) in pairs.iter().enumerate() {
        for k in 0..2 {
            match faces[k].last() {
                Some(&last) if last == pair[k] => {}
                Some(_) => {
                    if faces[k].contains(&pair[k]) {
                        return Ok(None);
                    }
                    faces[k].push(pair[k]);
                    joints[k].push(shared_vertex(topo, edges[index - 1], edges[index])?);
                }
                None => faces[k].push(pair[k]),
            }
        }
    }
    // Faces change on both sides. A chain that keeps one face is a rail
    // chain, or a box edge running over an earlier band, which the cascade
    // rebuilds exactly.
    if faces[0].len() < 2 || faces[1].len() < 2 {
        return Ok(None);
    }
    if !faces[0].iter().all(|face| !faces[1].contains(face)) {
        return Ok(None);
    }
    for face in faces.iter().flatten() {
        if !matches!(
            topo.face(*face)?.surface(),
            FaceSurface::Plane { .. }
                | FaceSurface::Cylinder(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_)
                | FaceSurface::Torus(_)
        ) {
            return Ok(None);
        }
    }

    // One convexity throughout, read where each edge's dihedral is decided.
    let mut concave = None;
    for (&edge, pair) in edges.iter().zip(&pairs) {
        match edge_material_frames_at(topo, edge, pair[0], pair[1])?
            .and_then(|frames| frames.concave())
        {
            Some(value) if concave.is_none_or(|c| c == value) => concave = Some(value),
            Some(_) => return Ok(None),
            None => {}
        }
    }
    // A concave chain over earlier bands (a notch edge into a mitred convex
    // pair) is rebuilt exactly by the cascade; only a convex one is a ridge.
    let Some(concave @ false) = concave else {
        return Ok(None);
    };

    // A run-out end: the chain's end faces are tangent there. The march
    // starts from the other end.
    let (v_start, v_end) = chain_ends(topo, edges)?;
    let tangent_at = |vertex: VertexId, a: FaceId, b: FaceId| -> Result<bool, BlendError> {
        let point = topo.vertex(vertex)?.point();
        Ok(
            match (
                outward_normal(topo, a, point),
                outward_normal(topo, b, point),
            ) {
                (Some(na), Some(nb)) => 1.0 - na.dot(nb) < TANGENT_END,
                _ => false,
            },
        )
    };
    let start_runs_out = tangent_at(v_start, faces[0][0], faces[1][0])?;
    let end_runs_out = tangent_at(
        v_end,
        faces[0][faces[0].len() - 1],
        faces[1][faces[1].len() - 1],
    )?;
    // It runs out at one end, so it has nowhere else to stop.
    if start_runs_out == end_runs_out {
        return Ok(None);
    }
    let reversed = start_runs_out;
    let runout = start_runs_out || end_runs_out;
    if reversed {
        for k in 0..2 {
            faces[k].reverse();
            joints[k].reverse();
        }
    }

    let mut sides = Vec::with_capacity(2);
    for k in 0..2 {
        let mut seams = Vec::with_capacity(faces[k].len().saturating_sub(1));
        for (pair, &joint) in faces[k].windows(2).zip(&joints[k]) {
            let near = topo.vertex(joint)?.point();
            let Some(seam) = support_seam(topo, pair[0], pair[1], near)? else {
                return Ok(None);
            };
            seams.push(seam);
        }
        sides.push(Side {
            faces: faces[k].clone(),
            seams,
        });
    }
    let [side_a, side_b]: [Side; 2] = sides
        .try_into()
        .map_err(|_| BlendError::TrimmingFailure { face: faces[0][0] })?;
    Ok(Some(Ridge {
        sides: [side_a, side_b],
        concave,
        reversed,
        runout,
    }))
}

/// The nearest point of `face`'s outer boundary to `point`, when it lies
/// within `tolerance`.
fn boundary_foot(
    topo: &Topology,
    face: FaceId,
    point: Point3,
    tolerance: f64,
) -> Result<Option<Point3>, BlendError> {
    let mut best: Option<(Point3, f64)> = None;
    for oriented in topo.wire(topo.face(face)?.outer_wire())?.edges() {
        let (_, foot, distance) = trimmer::closest_on_oriented_edge(topo, *oriented, point)?;
        if distance <= tolerance && best.is_none_or(|(_, closest)| distance < closest) {
            best = Some((foot, distance));
        }
    }
    Ok(best.map(|(foot, _)| foot))
}

/// Outward unit normal of `face` at `point`.
fn outward_normal(topo: &Topology, face: FaceId, point: Point3) -> Option<Vec3> {
    let face = topo.face(face).ok()?;
    let raw = match face.surface() {
        FaceSurface::Plane { normal, .. } => *normal,
        surface => {
            let (u, v) = surface.project_point(point)?;
            surface.normal(u, v)
        }
    };
    (if face.is_reversed() { -raw } else { raw })
        .normalize()
        .ok()
}

/// What the march needs besides the blend function.
struct MarchContext<'a> {
    topo: &'a Topology,
    ridge: &'a Ridge,
    spine: &'a Spine,
    /// Each side's surfaces, as the ball sees them, in march order.
    sides: &'a [Vec<&'a dyn ParametricSurface>; 2],
    config: WalkerConfig,
    /// Spine parameter of a distance along the march.
    s_of: &'a dyn Fn(f64) -> f64,
}

/// Walk the ball from the march's start to its end, handing each side over
/// to its next face where the contact crosses onto it.
#[allow(clippy::too_many_lines)]
fn march<F: BlendFunction>(
    context: &MarchContext<'_>,
    func: &F,
    size: f64,
) -> Result<March, BlendError> {
    let MarchContext {
        topo,
        ridge,
        spine,
        sides,
        config,
        s_of,
    } = *context;
    let length = spine.length();
    let report_edge = spine.edges()[0];
    let failure = |d: f64| BlendError::WalkingFailure {
        edge: report_edge,
        t: s_of(d) / length,
        residual: f64::NAN,
    };
    let walker = |index: [usize; 2]| {
        Walker::new(
            func,
            sides[0][index[0]],
            sides[1][index[1]],
            spine,
            topo,
            config,
        )
    };
    let solve = |index: [usize; 2], d: f64, guess: BlendParams| {
        let walker = walker(index);
        walker
            .solve_section(s_of(d), Some(guess))
            .or_else(|| walker.solve_section(s_of(d), None))
    };
    let pick = |section: &CircSection, k: usize| if k == 0 { section.p1 } else { section.p2 };
    // How far a section's contact on side `k` lies past that side's next
    // seam; negative while it is still on the current face.
    let past = |index: [usize; 2], k: usize, section: &CircSection| -> Result<f64, BlendError> {
        seam_offset(topo, &ridge.sides[k].seams[index[k]], pick(section, k))?
            .ok_or_else(|| failure(0.0))
    };
    let has_next = |index: [usize; 2], k: usize| index[k] + 1 < ridge.sides[k].faces.len();
    let width = |section: &CircSection| (section.p1 - section.p2).length();
    let resolution = 1e-13 * length.max(1.0);
    let min_step = 1e-9 * length.max(1.0);
    let max_step = MAX_STEP_FRACTION * length;
    let runout_width = RUNOUT_WIDTH * size;
    let max_sag = MAX_SAG * size;

    let mut index = [0, 0];
    let (mut params, first) = walker(index)
        .solve_section(s_of(0.0), None)
        .ok_or_else(|| BlendError::StartSolutionFailure {
            edge: report_edge,
            t: s_of(0.0) / length,
        })?;
    for k in 0..2 {
        if has_next(index, k) && past(index, k, &first)? > 0.0 {
            return Err(failure(0.0));
        }
    }
    let mut march = March {
        sections: vec![first.clone()],
        switches: [Vec::new(), Vec::new()],
    };
    let mut d = 0.0;
    let mut current = first;
    let mut previous: Option<(f64, BlendParams)> = None;
    let mut step = max_step;
    let mut ran_out = false;

    #[allow(clippy::while_float)]
    while length - d > min_step {
        let next_d = (d + step).min(length);
        let guess = previous.map_or(params, |(previous_d, previous_params)| {
            let ratio = (next_d - d) / (d - previous_d);
            BlendParams {
                u1: ratio.mul_add(params.u1 - previous_params.u1, params.u1),
                v1: ratio.mul_add(params.v1 - previous_params.v1, params.v1),
                u2: ratio.mul_add(params.u2 - previous_params.u2, params.u2),
                v2: ratio.mul_add(params.v2 - previous_params.v2, params.v2),
            }
        });
        let solved = solve(index, next_d, guess).filter(|(_, next)| {
            // Past the run-out the system has no ball on the right side.
            !ridge.runout || width(next) <= 2.0 * width(&current) + runout_width
        });
        let Some((next_params, next)) = solved else {
            step *= 0.5;
            if step < min_step {
                if ridge.runout && width(&current) <= runout_width {
                    ran_out = true;
                    break;
                }
                return Err(failure(next_d));
            }
            continue;
        };
        if turn(&current, &next) > MAX_TURN && step > 64.0 * min_step {
            step *= 0.5;
            continue;
        }
        if step > 64.0 * min_step {
            let mid_d = f64::midpoint(d, next_d);
            let halfway = |a: Point3, b: Point3| {
                Point3::new(
                    f64::midpoint(a.x(), b.x()),
                    f64::midpoint(a.y(), b.y()),
                    f64::midpoint(a.z(), b.z()),
                )
            };
            let sags = solve(index, mid_d, params).is_none_or(|(_, mid)| {
                (mid.p1 - halfway(current.p1, next.p1)).length() > max_sag
                    || (mid.p2 - halfway(current.p2, next.p2)).length() > max_sag
            });
            if sags {
                step *= 0.5;
                continue;
            }
        }

        let crossing: Vec<usize> = (0..2)
            .filter(|&k| has_next(index, k))
            .map(|k| past(index, k, &next).map(|offset| (k, offset)))
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .filter(|&(_, offset)| offset > 0.0)
            .map(|(k, _)| k)
            .collect();
        if !crossing.is_empty() {
            // A contact left its face between `d` and `next_d`: find where
            // each reaches its seam, and hand the earliest over there. The
            // handover is the last station still on the face. Where the
            // chain is only nearly tangent at a vertex the contact jumps
            // across the seam instead of reaching it; the station before the
            // jump is the one on the seam.
            let mut joints = Vec::with_capacity(crossing.len());
            for &k in &crossing {
                let mut high = next_d;
                let mut joint = (d, params, current.clone());
                if past(index, k, &current)? < 0.0 {
                    for _ in 0..BISECTIONS {
                        let mid = f64::midpoint(joint.0, high);
                        let (mid_params, section) =
                            solve(index, mid, joint.1).ok_or_else(|| failure(mid))?;
                        let offset = past(index, k, &section)?;
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
                joints.push((k, joint));
            }
            let (joint_d, joint_params, joint_section) = joints
                .iter()
                .map(|(_, joint)| joint)
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .cloned()
                .ok_or_else(|| failure(next_d))?;
            let earliest = joint_d;
            // Sides whose seams the ball reaches together switch together.
            let switching: Vec<usize> = joints
                .iter()
                .filter(|(_, joint)| joint.0 - earliest <= 64.0 * resolution)
                .map(|&(k, _)| k)
                .collect();
            if joint_d - d > min_step {
                march.sections.push(joint_section.clone());
            }
            let at = march.sections.len() - 1;
            let mut handover = joint_params;
            for &k in &switching {
                march.switches[k].push(at);
                index[k] += 1;
                let (u, v) = sides[k][index[k]].project_point(pick(&joint_section, k));
                if k == 0 {
                    handover.u1 = u;
                    handover.v1 = v;
                } else {
                    handover.u2 = u;
                    handover.v2 = v;
                }
            }
            let (handed_params, handed) =
                solve(index, joint_d, handover).ok_or_else(|| failure(joint_d))?;
            let join = 1e-5 * handed.radius.max(1.0);
            if (handed.p1 - joint_section.p1).length() > join
                || (handed.p2 - joint_section.p2).length() > join
            {
                return Err(failure(joint_d));
            }
            params = handed_params;
            d = joint_d;
            current = joint_section;
            previous = None;
            step = max_step;
            continue;
        }

        march.sections.push(next.clone());
        previous = Some((d, params));
        params = next_params;
        d = next_d;
        current = next;
        step = (step * 1.5).min(max_step);
        if ridge.runout && width(&current) <= runout_width {
            ran_out = true;
            break;
        }
    }
    if ridge.runout {
        if !ran_out && width(&current) > runout_width {
            return Err(failure(length));
        }
        // The run-out section, which the caller closes onto the end vertex:
        // the last walked one when the march reached the vertex, else one
        // more beyond it (a repeated section would leave the band a span of
        // zero length).
        if length - d > min_step {
            march.sections.push(current);
        }
    }
    for k in 0..2 {
        let switches = &march.switches[k];
        if switches.len() + 1 != ridge.sides[k].faces.len()
            || switches.first().is_some_and(|&at| at < 1)
            || switches.windows(2).any(|pair| pair[1] <= pair[0])
            || switches
                .last()
                .is_some_and(|&at| at + 1 >= march.sections.len())
        {
            // A run too short to carry a contact of its own.
            return Err(failure(length));
        }
    }
    Ok(march)
}
