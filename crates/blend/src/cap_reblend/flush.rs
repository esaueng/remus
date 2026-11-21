//! Analytic topology at the instant a rounded cap becomes flush with the
//! planar top its boss stands on.
//!
//! A boss fused into the corner or edge of a block has a convex rim round
//! on its cap and a concave shoulder round where it meets the block top.
//! Where the boss leaves the block, a concave seam round runs up each wall
//! junction and meets the block top's convex edge round in a corner mitre.
//! When the cap and the block top reach one plane, the boss wall between
//! them, the shoulder and the mitres vanish. What remains is the outline
//! round of one merged top. Outside the block, it is the rim round. At each
//! seam, it is a torus about the seam round's own axis. That torus meets the
//! rim round in the meridian plane through both axes and meets the block's
//! edge round where that edge round's axis is nearest the seam axis. Every
//! carrier and every new contact is exact.

use std::collections::BTreeSet;
use std::f64::consts::{FRAC_PI_2, TAU};

use remus_math::curves::Circle3D;
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{face_edges, face_vertices, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{FaceId, SolidId, Topology};

use super::{CapReblendApproximation, CapReblendOutcome, height, refusal};
use crate::builder_utils::add_certified_curve_edge;
use crate::{BlendEngine, BlendError, BlendFaceOrigins, BlendResult};

const LINEAR: f64 = 1e-7;
const PARALLEL: f64 = 1e-10;

/// Rebuild a cap that is flush with the planar top its boss stands on.
///
/// Returns `None` unless `rims` holds exactly one convex round of `radius`
/// on a coaxial boss wall and the candidate has a concave shoulder round on
/// the same axis whose planar top is at the cap's level. Other members of a
/// joined cap's band group are removed with the shoulder, and the rim round
/// is rebuilt at the cap's band whether or not it travelled with the cap. A
/// recognized flush candidate without exactly two qualified seam corners
/// refuses atomically.
///
/// # Errors
/// Refuses ambiguous shoulders, unqualified corner neighborhoods and
/// boundaries that do not close; the candidate is restored on refusal.
pub fn rebuild_flush_cap_in_place(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    permission: CapReblendApproximation,
) -> Result<Option<CapReblendOutcome>, BlendError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = rebuild(topo, solid, cap, rims, radius, permission);
    if matches!(result, Ok(Some(_))) {
        snapshot.commit(topo);
    } else {
        snapshot.restore(topo);
    }
    result
}

/// One seam corner of the vanishing boss: the seam round, the block's edge
/// round, the mitre between them and the shoulder, and where they meet.
struct Corner {
    mitre: FaceId,
    seam: FaceId,
    edge_round: FaceId,
    /// The seam round's axis point at the band's centre height.
    seam_axis: Point3,
    seam_radius: f64,
    /// Unit direction, normal to the cap axis, from the boss axis to the
    /// seam axis.
    outward: Vec3,
    /// Unit direction, normal to the cap axis, from the seam axis to the
    /// nearest point of the edge round's axis.
    toward_edge: Vec3,
    /// The edge round's axis point nearest the seam axis.
    edge_foot: Point3,
    /// The seam round's top arc, which the corner torus shares.
    seam_top: EdgeId,
    /// Lower and upper ends of the edge round's cross-section at
    /// `edge_foot`.
    edge_low: VertexId,
    edge_high: VertexId,
    /// Where the boss wall, the seam round and the rim band meet.
    boss_foot: VertexId,
}

#[allow(clippy::too_many_lines)]
fn rebuild(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    permission: CapReblendApproximation,
) -> Result<Option<CapReblendOutcome>, BlendError> {
    let cap_data = topo.face(cap)?;
    let Some(axis) = cap_data.effective_plane_normal() else {
        return Ok(None);
    };
    let FaceSurface::Plane { normal, d } = cap_data.surface() else {
        return Ok(None);
    };
    let top = *d * normal.dot(axis);
    let band = top - radius;
    let adjacency = topo.build_adjacency(solid)?;
    let neighbors = |face: FaceId| -> Result<BTreeSet<FaceId>, BlendError> {
        Ok(face_edges(topo, face)?
            .into_iter()
            .flat_map(|e| adjacency.faces_for_edge(e).iter().copied())
            .filter(|&f| f != face)
            .collect())
    };
    // The rim band proper: a convex round of the cap's radius on a coaxial
    // boss wall. A joined cap's group also holds the bands that carried its
    // rim over the shoulder; they go with the shoulder.
    let group: BTreeSet<FaceId> = rims.iter().copied().collect();
    let mut bands: Vec<(FaceId, FaceId, Point3, f64)> = Vec::new();
    for &candidate in rims {
        let data = topo.face(candidate)?;
        let FaceSurface::Torus(torus) = data.surface() else {
            continue;
        };
        if data.is_reversed()
            || torus.z_axis().dot(axis) < 1.0 - PARALLEL
            || (torus.minor_radius() - radius).abs() > LINEAR
        {
            continue;
        }
        let boss_radius = torus.major_radius() + radius;
        let center = torus.center();
        for &member in rims {
            for wall in neighbors(member)? {
                if group.contains(&wall) || wall == cap {
                    continue;
                }
                let wall_data = topo.face(wall)?;
                let qualified = match wall_data.surface() {
                    FaceSurface::Cylinder(c) => {
                        let offset = c.origin() - center;
                        !wall_data.is_reversed()
                            && c.axis().dot(axis).abs() > 1.0 - PARALLEL
                            && (offset - axis * offset.dot(axis)).length() < LINEAR
                            && (c.radius() - boss_radius).abs() < LINEAR
                    }
                    FaceSurface::Plane { .. }
                    | FaceSurface::Nurbs(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_) => false,
                };
                if qualified && !bands.iter().any(|&(r, w, ..)| r == candidate && w == wall) {
                    bands.push((candidate, wall, center, boss_radius));
                }
            }
        }
    }
    let [(rim, wall, rim_center, boss_radius)] = bands.as_slice() else {
        return Ok(None);
    };
    let (rim, wall, boss_radius) = (*rim, *wall, *boss_radius);
    let low_center = *rim_center + axis * (band - height(*rim_center, axis));
    let top_center = low_center + axis * radius;
    let plan = |v: Vec3| v - axis * v.dot(axis);
    let on_axis = |p: Point3| plan(p - low_center).length() < LINEAR;

    // The shoulder: a concave round on the boss axis whose planar top is at
    // the cap's level.
    let mut shoulders = Vec::new();
    for face in solid_faces(topo, solid)? {
        if group.contains(&face) {
            continue;
        }
        let data = topo.face(face)?;
        let FaceSurface::Torus(torus) = data.surface() else {
            continue;
        };
        if !data.is_reversed()
            || torus.z_axis().dot(axis) < 1.0 - PARALLEL
            || !on_axis(torus.center())
            || (torus.major_radius() - torus.minor_radius() - boss_radius).abs() > LINEAR
            || (height(torus.center(), axis) - top - torus.minor_radius()).abs() > LINEAR
        {
            continue;
        }
        for other in neighbors(face)? {
            let other_data = topo.face(other)?;
            if other != cap
                && let FaceSurface::Plane { normal, d } = other_data.surface()
                && other_data
                    .effective_plane_normal()
                    .is_some_and(|n| n.dot(axis) > 1.0 - PARALLEL)
                && (*d * normal.dot(axis) - top).abs() < LINEAR
            {
                shoulders.push((face, other));
            }
        }
    }
    if shoulders.is_empty() {
        return Ok(None);
    }
    let [(shoulder, block_top)] = shoulders.as_slice() else {
        return Err(refusal("flush cap has ambiguous shoulders"));
    };
    let (shoulder, block_top) = (*shoulder, *block_top);
    if [cap, block_top, wall]
        .iter()
        .any(|&f| topo.face(f).is_ok_and(|f| !f.inner_wires().is_empty()))
        || rims
            .iter()
            .any(|&f| topo.face(f).is_ok_and(|f| f.inner_wires().len() > 1))
    {
        return Err(refusal("flush cap faces have holes"));
    }

    // Each mitre between the shoulder and the block meets one seam round and
    // one edge round.
    let mut corners = Vec::new();
    for mitre in neighbors(shoulder)? {
        if mitre == block_top || mitre == wall || mitre == cap || group.contains(&mitre) {
            continue;
        }
        let around = neighbors(mitre)?;
        let mut seams = Vec::new();
        let mut rounds = Vec::new();
        for &face in &around {
            if face == shoulder || face == wall || face == cap || group.contains(&face) {
                continue;
            }
            let data = topo.face(face)?;
            match data.surface() {
                FaceSurface::Cylinder(c)
                    if data.is_reversed() && c.axis().dot(axis).abs() > 1.0 - PARALLEL =>
                {
                    seams.push((face, c.clone()));
                }
                FaceSurface::Cylinder(c)
                    if !data.is_reversed()
                        && c.axis().dot(axis).abs() < PARALLEL
                        && (c.radius() - radius).abs() < LINEAR =>
                {
                    rounds.push((face, c.clone()));
                }
                FaceSurface::Cylinder(_)
                | FaceSurface::Plane { .. }
                | FaceSurface::Nurbs(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_)
                | FaceSurface::Torus(_) => {
                    return Err(refusal("flush mitre borders an unqualified face"));
                }
            }
        }
        let ([(seam, seam_surface)], [(edge_round, round_surface)]) =
            (seams.as_slice(), rounds.as_slice())
        else {
            return Err(refusal(
                "flush mitre needs one seam round and one edge round",
            ));
        };
        let seam_radius = seam_surface.radius();
        let seam_axis = seam_surface.origin()
            + seam_surface.axis()
                * ((band - height(seam_surface.origin(), axis)) / seam_surface.axis().dot(axis));
        let offset = plan(seam_axis - low_center);
        if (offset.length() - boss_radius - seam_radius).abs() > LINEAR {
            return Err(refusal("flush seam round is not tangent to the boss"));
        }
        let outward = offset.normalize()?;
        let along = round_surface.axis();
        let edge_foot =
            round_surface.origin() + along * (seam_axis - round_surface.origin()).dot(along);
        let reach = edge_foot - seam_axis;
        if (height(edge_foot, axis) - band).abs() > LINEAR
            || reach.dot(axis).abs() > LINEAR
            || (reach.length() - seam_radius - radius).abs() > LINEAR
        {
            return Err(refusal(
                "flush edge round does not continue the seam corner",
            ));
        }
        let toward_edge = reach.normalize()?;
        let find_vertex = |face: FaceId, at: Point3| -> Result<VertexId, BlendError> {
            face_vertices(topo, face)?
                .into_iter()
                .find(|&v| {
                    topo.vertex(v)
                        .is_ok_and(|v| (v.point() - at).length() < LINEAR)
                })
                .ok_or_else(|| refusal("flush corner lacks an expected vertex"))
        };
        let boss_foot = find_vertex(*seam, low_center + outward * boss_radius)?;
        let edge_low = find_vertex(*seam, seam_axis + toward_edge * seam_radius)?;
        let edge_high = find_vertex(*edge_round, edge_foot + axis * radius)?;
        let seam_tops: Vec<_> = face_edges(topo, *seam)?
            .into_iter()
            .filter(|&e| adjacency.faces_for_edge(e).contains(&mitre))
            .collect();
        let [seam_top] = seam_tops.as_slice() else {
            return Err(refusal("flush seam round meets its mitre more than once"));
        };
        let top_edge = topo.edge(*seam_top)?;
        let ends = [top_edge.start(), top_edge.end()];
        let EdgeCurve::Circle(circle) = top_edge.curve() else {
            return Err(refusal("flush seam round top is not an arc"));
        };
        if !ends.contains(&boss_foot)
            || !ends.contains(&edge_low)
            || (circle.radius() - seam_radius).abs() > LINEAR
            || (circle.center() - seam_axis).length() > LINEAR
        {
            return Err(refusal("flush seam round top is not the corner arc"));
        }
        corners.push(Corner {
            mitre,
            seam: *seam,
            edge_round: *edge_round,
            seam_axis,
            seam_radius,
            outward,
            toward_edge,
            edge_foot,
            seam_top: *seam_top,
            edge_low,
            edge_high,
            boss_foot,
        });
    }
    let [first_corner, second_corner] = corners.as_slice() else {
        return Err(refusal("flush cap needs exactly two seam corners"));
    };

    // The rim and wall keep the span away from the block. The shoulder's
    // spring on the wall lies on the other span.
    let inner = shoulder_direction(topo, shoulder, low_center, axis)?;
    let angle = |from: Vec3, to: Vec3| {
        let a = from.cross(to).dot(axis).atan2(from.dot(to));
        if a < 0.0 { a + TAU } else { a }
    };
    // Counter-clockwise about the axis from `start` to `end` must avoid the
    // inner direction.
    let (start, end) = {
        let (a, b) = (first_corner, second_corner);
        let span = angle(a.outward, b.outward);
        if angle(a.outward, inner) < span {
            (b, a)
        } else {
            (a, b)
        }
    };
    let span = angle(start.outward, end.outward);
    if span <= 1e-6 || angle(start.outward, inner) < span {
        return Err(refusal("flush rim span is ambiguous"));
    }
    let reference = start.outward;

    // Shared vertices at the junction of the rim band and each corner torus.
    let junction = |c: &Corner| low_center + c.outward * (boss_radius - radius);
    let upper = |c: &Corner| junction(c) + axis * radius;
    let start_upper = topo.add_vertex(Vertex::new(upper(start), LINEAR));
    let end_upper = topo.add_vertex(Vertex::new(upper(end), LINEAR));

    // Rim band contacts over the outer span, counter-clockwise from `start`.
    let rim_low = add_certified_curve_edge(
        topo,
        start.boss_foot,
        end.boss_foot,
        EdgeCurve::Circle(Circle3D::new_with_ref(
            low_center,
            axis,
            boss_radius,
            reference,
        )?),
        (0.0, span),
    )?;
    let rim_high = add_certified_curve_edge(
        topo,
        start_upper,
        end_upper,
        EdgeCurve::Circle(Circle3D::new_with_ref(
            top_center,
            axis,
            boss_radius - radius,
            reference,
        )?),
        (0.0, span),
    )?;

    let mut created = Vec::new();
    let mut corner_edges = Vec::new();
    for (corner, corner_upper) in [(start, start_upper), (end, end_upper)] {
        // The rim band's end section in the meridian plane through both axes.
        let section = add_certified_curve_edge(
            topo,
            corner.boss_foot,
            corner_upper,
            EdgeCurve::Circle(Circle3D::new_with_ref(
                junction(corner),
                corner.outward.cross(axis),
                radius,
                corner.outward,
            )?),
            (0.0, FRAC_PI_2),
        )?;
        // The edge round's end section where its axis is nearest the seam.
        let edge_section = add_certified_curve_edge(
            topo,
            corner.edge_low,
            corner.edge_high,
            EdgeCurve::Circle(Circle3D::new_with_ref(
                corner.edge_foot,
                axis.cross(corner.toward_edge),
                radius,
                -corner.toward_edge,
            )?),
            (0.0, FRAC_PI_2),
        )?;
        // The corner torus's contact with the merged top, from the edge
        // round to the rim band, the short way about the seam axis.
        let toward_boss = -corner.outward;
        let sweep = angle(corner.toward_edge, toward_boss);
        let (turn, sweep) = if sweep <= std::f64::consts::PI {
            (axis, sweep)
        } else {
            (-axis, TAU - sweep)
        };
        let outline_radius = corner.seam_radius + radius;
        let outline = add_certified_curve_edge(
            topo,
            corner.edge_high,
            corner_upper,
            EdgeCurve::Circle(Circle3D::new_with_ref(
                corner.seam_axis + axis * radius,
                turn,
                outline_radius,
                corner.toward_edge,
            )?),
            (0.0, sweep),
        )?;
        let torus = ToroidalSurface::with_axis_and_ref_dir(
            corner.seam_axis,
            outline_radius,
            radius,
            axis,
            (corner.toward_edge + toward_boss).normalize()?,
        )?;
        let loop_edges = [
            (corner.seam_top, corner.boss_foot),
            (edge_section, corner.edge_low),
            (outline, corner.edge_high),
            (section, corner_upper),
        ];
        let wire = oriented_loop(topo, &loop_edges, |p| {
            let radial = plan(p - corner.seam_axis);
            let centre = corner.seam_axis
                + radial.normalize().unwrap_or(corner.toward_edge) * outline_radius;
            (p - centre).normalize().unwrap_or(axis)
        })?;
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(torus)));
        created.push((
            face,
            vec![corner.mitre, corner.seam, corner.edge_round, rim],
        ));
        corner_edges.push((corner, section, edge_section, outline));
    }

    // The edge rounds end on their exact section.
    for (corner, _, edge_section, _) in &corner_edges {
        replace_edges_shared_with(
            topo,
            &adjacency,
            corner.edge_round,
            corner.mitre,
            *edge_section,
            corner.edge_low,
            corner.edge_high,
        )?;
    }

    // The rim band over the outer span.
    let [(_, start_section, _, _), (_, end_section, _, _)] = corner_edges.as_slice() else {
        return Err(refusal("flush corner bookkeeping lost a corner"));
    };
    let rim_wire = oriented_loop(
        topo,
        &[
            (rim_low, start.boss_foot),
            (*end_section, end.boss_foot),
            (rim_high, end_upper),
            (*start_section, start_upper),
        ],
        |p| {
            let radial = plan(p - low_center);
            let centre =
                low_center + radial.normalize().unwrap_or(reference) * (boss_radius - radius);
            (p - centre).normalize().unwrap_or(axis)
        },
    )?;
    topo.face_mut(rim)?
        .set_surface(FaceSurface::Torus(ToroidalSurface::with_axis_and_ref_dir(
            low_center,
            boss_radius - radius,
            radius,
            axis,
            -inner,
        )?));
    topo.set_face_boundary_wires(rim, rim_wire, vec![])?;

    // The boss wall keeps its lower boundary and ends on the rim band.
    let doomed: BTreeSet<_> = [cap, shoulder, first_corner.mitre, second_corner.mitre]
        .into_iter()
        .chain(rims.iter().copied())
        .collect();
    let old_wall = topo.wire(topo.face(wall)?.outer_wire())?.edges().to_vec();
    let kept: Vec<_> = old_wall
        .iter()
        .copied()
        .filter(|oe| {
            let others = adjacency.faces_for_edge(oe.edge());
            others.iter().any(|f| *f != wall) && !others.iter().any(|f| doomed.contains(f))
        })
        .collect();
    let path = contiguous_path(topo, &old_wall, &kept)?;
    let (from, to) = (
        path.first()
            .map(|oe| oriented_start(topo, *oe))
            .transpose()?,
        path.last().map(|oe| oriented_end(topo, *oe)).transpose()?,
    );
    let forward = match (from, to) {
        (Some(from), Some(to)) if to == start.boss_foot && from == end.boss_foot => true,
        (Some(from), Some(to)) if to == end.boss_foot && from == start.boss_foot => false,
        _ => return Err(refusal("flush boss wall does not end at both seam corners")),
    };
    let mut wall_wire = path;
    wall_wire.push(OrientedEdge::new(rim_low, forward));
    let wall_wire = topo.add_wire(Wire::new(wall_wire, true)?);
    let FaceSurface::Cylinder(wall_surface) = topo.face(wall)?.surface() else {
        return Err(refusal("flush boss wall changed carrier"));
    };
    let wall_surface = CylindricalSurface::with_ref_dir(
        wall_surface.origin(),
        wall_surface.axis(),
        wall_surface.radius(),
        -inner,
    )?;
    topo.face_mut(wall)?
        .set_surface(FaceSurface::Cylinder(wall_surface));
    topo.set_face_boundary_wires(wall, wall_wire, vec![])?;

    // The block top runs round the boss on the new outline.
    let old_top = topo
        .wire(topo.face(block_top)?.outer_wire())?
        .edges()
        .to_vec();
    let shoulder_run: Vec<_> = old_top
        .iter()
        .copied()
        .filter(|oe| adjacency.faces_for_edge(oe.edge()).contains(&shoulder))
        .collect();
    let rest: Vec<_> = old_top
        .iter()
        .copied()
        .filter(|oe| !holds(&shoulder_run, *oe))
        .collect();
    let rest = contiguous_path(topo, &old_top, &rest)?;
    let (Some(first), Some(last)) = (rest.first(), rest.last()) else {
        return Err(refusal(
            "flush block top has no boundary beyond its shoulder",
        ));
    };
    let (rest_start, rest_end) = (oriented_start(topo, *first)?, oriented_end(topo, *last)?);
    let mut outline = Vec::new();
    let find = |corner_vertex: VertexId| {
        corner_edges
            .iter()
            .find(|(c, ..)| c.edge_high == corner_vertex)
            .map(|(c, _, _, outline)| (c.edge_high, *outline))
    };
    let (Some((_, leave)), Some((_, arrive))) = (find(rest_end), find(rest_start)) else {
        return Err(refusal("flush block top does not meet both edge rounds"));
    };
    // Leave the edge round at `rest_end`, go round the rim band and arrive
    // at `rest_start`.
    let leave_upper = topo.edge(leave)?.end();
    let arrive_upper = topo.edge(arrive)?.end();
    outline.push(OrientedEdge::new(leave, true));
    let rim_forward = if leave_upper == start_upper && arrive_upper == end_upper {
        true
    } else if leave_upper == end_upper && arrive_upper == start_upper {
        false
    } else {
        return Err(refusal("flush outline does not run round the rim band"));
    };
    outline.push(OrientedEdge::new(rim_high, rim_forward));
    outline.push(OrientedEdge::new(arrive, false));
    let mut top_wire = rest;
    top_wire.extend(outline);
    let top_wire = topo.add_wire(Wire::new(top_wire, true)?);
    topo.set_face_boundary_wires(block_top, top_wire, vec![])?;

    let removed: Vec<_> = [cap, shoulder, first_corner.mitre, second_corner.mitre]
        .into_iter()
        .chain(rims.iter().copied().filter(|&f| f != rim))
        .collect();
    let mut faces: Vec<_> = solid_faces(topo, solid)?
        .into_iter()
        .filter(|f| !removed.contains(f))
        .collect();
    let survived: Vec<_> = faces.iter().map(|&f| (f, f)).collect();
    faces.extend(created.iter().map(|(f, _)| *f));
    let shell = topo.add_shell(Shell::new(faces)?);
    let result = topo.add_solid(Solid::new(shell, vec![]));
    Ok(Some(CapReblendOutcome {
        corner_stations: permission.stations,
        edge_images: vec![],
        result: BlendResult {
            solid: result,
            succeeded: vec![],
            failed: vec![],
            is_partial: false,
            engine: BlendEngine::Walking,
            face_origins: Some(BlendFaceOrigins {
                survived,
                deleted: removed,
                created,
                created_unattributed: vec![],
            }),
        },
    }))
}

/// Unit direction, normal to `axis`, from the boss axis toward the shoulder:
/// the mean of the shoulder's vertices, which all lie over the block.
fn shoulder_direction(
    topo: &Topology,
    shoulder: FaceId,
    center: Point3,
    axis: Vec3,
) -> Result<Vec3, BlendError> {
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    for vertex in face_vertices(topo, shoulder)? {
        let radial = topo.vertex(vertex)?.point() - center;
        sum += (radial - axis * radial.dot(axis)).normalize()?;
    }
    Ok(sum.normalize()?)
}

fn holds(set: &[OrientedEdge], oe: OrientedEdge) -> bool {
    set.iter()
        .any(|s| s.edge() == oe.edge() && s.is_forward() == oe.is_forward())
}

fn oriented_start(topo: &Topology, oe: OrientedEdge) -> Result<VertexId, BlendError> {
    Ok(oe.oriented_start(topo.edge(oe.edge())?))
}

fn oriented_end(topo: &Topology, oe: OrientedEdge) -> Result<VertexId, BlendError> {
    Ok(oe.oriented_end(topo.edge(oe.edge())?))
}

/// `kept`, a subset of the closed `wire`, as one path in wire order.
fn contiguous_path(
    topo: &Topology,
    wire: &[OrientedEdge],
    kept: &[OrientedEdge],
) -> Result<Vec<OrientedEdge>, BlendError> {
    let n = wire.len();
    let inside = |i: usize| holds(kept, wire[i % n]);
    let starts: Vec<_> = (0..n)
        .filter(|&i| inside(i) && !inside(i + n - 1))
        .collect();
    let [start] = starts.as_slice() else {
        return Err(refusal("flush boundary is not one contiguous run"));
    };
    let path: Vec<_> = (0..kept.len()).map(|i| wire[(start + i) % n]).collect();
    if path.len() != kept.len() || !path.iter().all(|oe| holds(kept, *oe)) {
        return Err(refusal("flush boundary is not one contiguous run"));
    }
    for pair in path.windows(2) {
        if oriented_end(topo, pair[0])? != oriented_start(topo, pair[1])? {
            return Err(refusal("flush boundary run is disconnected"));
        }
    }
    Ok(path)
}

/// Replace `face`'s edges shared with `other` by the single `edge` from
/// `low` to `high`.
fn replace_edges_shared_with(
    topo: &mut Topology,
    adjacency: &AdjacencyIndex,
    face: FaceId,
    other: FaceId,
    edge: EdgeId,
    low: VertexId,
    high: VertexId,
) -> Result<(), BlendError> {
    let old = topo.wire(topo.face(face)?.outer_wire())?.edges().to_vec();
    let shared: Vec<_> = old
        .iter()
        .copied()
        .filter(|oe| adjacency.faces_for_edge(oe.edge()).contains(&other))
        .collect();
    let rest: Vec<_> = old
        .iter()
        .copied()
        .filter(|oe| !holds(&shared, *oe))
        .collect();
    let mut path = contiguous_path(topo, &old, &rest)?;
    let (Some(first), Some(last)) = (path.first(), path.last()) else {
        return Err(refusal("flush edge round has no boundary beyond its mitre"));
    };
    let (from, to) = (oriented_start(topo, *first)?, oriented_end(topo, *last)?);
    let forward = if to == low && from == high {
        true
    } else if to == high && from == low {
        false
    } else {
        return Err(refusal(
            "flush edge round does not end on its corner section",
        ));
    };
    path.push(OrientedEdge::new(edge, forward));
    let wire = topo.add_wire(Wire::new(path, true)?);
    topo.set_face_boundary_wires(face, wire, vec![])?;
    Ok(())
}

/// A closed wire through `edges`, each given with the vertex it leaves in
/// traversal order, wound counter-clockwise about `outward`.
fn oriented_loop(
    topo: &mut Topology,
    edges: &[(EdgeId, VertexId)],
    outward: impl Fn(Point3) -> Vec3,
) -> Result<remus_topology::wire::WireId, BlendError> {
    let mut oriented = Vec::with_capacity(edges.len());
    let mut samples = Vec::new();
    for &(edge, leaving) in edges {
        let data = topo.edge(edge)?;
        let forward = if data.start() == leaving {
            true
        } else if data.end() == leaving {
            false
        } else {
            return Err(refusal("flush loop edge does not leave its vertex"));
        };
        let (a, b) = (
            topo.vertex(data.start())?.point(),
            topo.vertex(data.end())?.point(),
        );
        let (t0, t1) = data.trim().unwrap_or((0.0, 1.0));
        let mut points: Vec<_> = (0..8)
            .map(|i| {
                let t = t0 + (t1 - t0) * f64::from(i) / 8.0;
                data.curve().evaluate_with_endpoints(t, a, b)
            })
            .collect();
        if !forward {
            points.push(data.curve().evaluate_with_endpoints(t1, a, b));
            points.reverse();
            points.pop();
        }
        samples.extend(points);
        oriented.push(OrientedEdge::new(edge, forward));
    }
    for pair in oriented.windows(2) {
        if oriented_end(topo, pair[0])? != oriented_start(topo, pair[1])? {
            return Err(refusal("flush loop is disconnected"));
        }
    }
    // Newell's normal of the sampled boundary against the surface normal at
    // the boundary's centroid.
    let mut newell = Vec3::new(0.0, 0.0, 0.0);
    let mut centroid = Vec3::new(0.0, 0.0, 0.0);
    for (i, p) in samples.iter().enumerate() {
        let q = samples[(i + 1) % samples.len()];
        newell += (*p - Point3::new(0.0, 0.0, 0.0)).cross(q - Point3::new(0.0, 0.0, 0.0));
        centroid += *p - Point3::new(0.0, 0.0, 0.0);
    }
    #[allow(clippy::cast_precision_loss)]
    let centroid = Point3::new(0.0, 0.0, 0.0) + centroid * (1.0 / samples.len() as f64);
    let winding = newell.dot(outward(centroid));
    if winding.abs() < 1e-12 {
        return Err(refusal("flush loop winding is degenerate"));
    }
    if winding < 0.0 {
        oriented = oriented
            .into_iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
    }
    Ok(topo.add_wire(Wire::new(oriented, true)?))
}
