//! Restore retained sampled shoulder limits before reversing a joined-cap edit.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::nurbs::{curve::NurbsCurve, project_point_to_curve};
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{face_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{EdgeId, FaceId, SolidId, Topology, VertexId};

use super::{CanalNormal, refusal};
use crate::BlendError;
use crate::builder_utils::add_certified_curve_edge;

/// Construction record of restoring the full lower support boundaries.
pub struct SupportExtension {
    /// Extended source edges and their constructed images.
    pub edge_images: Vec<(EdgeId, EdgeId)>,
    /// The original inner shoulder spring rebuilt on its analytic supports.
    pub spring: EdgeId,
    /// Axial cylindrical support of the restored shoulder.
    pub boss: FaceId,
    /// Concave toroidal shoulder support.
    pub shoulder: FaceId,
}

struct Corner {
    face: FaceId,
    boss: FaceId,
    shoulder: FaceId,
    contact: EdgeId,
    p: EdgeId,
    q: EdgeId,
    old_p: VertexId,
    old_q: VertexId,
    end: Point3,
    p_curve: NurbsCurve,
    /// The shoulder rail when the shoulder contact is not a latitude circle
    /// (a walked transition's rail across the shoulder).
    q_curve: Option<NurbsCurve>,
}

/// Restore the full lower shoulder on an owned disposable reconstruction body.
///
/// This mutates the candidate on success. Its original source must have been
/// copied by the caller: no entity in this candidate may be shared with a
/// deliverable source. The old cap bands deliberately remain until the caller
/// replaces them. The intermediate candidate is not a deliverable solid.
///
/// # Errors
/// Refuses ambiguous corner roles, an unavailable full boundary carrier or
/// uncertified extension. A refusal restores the candidate atomically.
pub fn restore_joined_support_limits_in_place(
    topo: &mut Topology,
    solid: SolidId,
    rims: &[FaceId],
    axis: Vec3,
) -> Result<SupportExtension, BlendError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = restore(topo, solid, rims, axis);
    if result.is_ok() {
        snapshot.commit(topo);
    } else {
        snapshot.restore(topo);
    }
    result
}

fn restore(
    topo: &mut Topology,
    solid: SolidId,
    rims: &[FaceId],
    axis: Vec3,
) -> Result<SupportExtension, BlendError> {
    let adjacency = topo.build_adjacency(solid)?;
    let mut corners = Vec::new();
    for face in solid_faces(topo, solid)? {
        if rims.contains(&face) {
            continue;
        }
        let data = topo.face(face)?;
        let FaceSurface::Nurbs(surface) = data.surface() else {
            continue;
        };
        let edges = face_edges(topo, face)?;
        let contacts: Vec<_> = edges
            .iter()
            .copied()
            .filter(|&edge| {
                adjacency
                    .faces_for_edge(edge)
                    .iter()
                    .any(|f| rims.contains(f))
            })
            .collect();
        if contacts.is_empty() {
            continue;
        }
        let field = CanalNormal::contour_field(data)?;
        // A walked transition (a smaller convex band run into a seam round
        // and up the shoulder) rides the shoulder with its own radius; a
        // rolling-ball corner shares the shoulder's.
        let walked = CanalNormal::new(data).is_err();
        let [contact] = contacts.as_slice() else {
            return Err(refusal("restored corner has multiple old cap contacts"));
        };
        let mut boss_edges = Vec::new();
        let mut shoulder_edges = Vec::new();
        for &edge in &edges {
            for &other in adjacency.faces_for_edge(edge) {
                if other == face || rims.contains(&other) {
                    continue;
                }
                match topo.face(other)?.surface() {
                    FaceSurface::Cylinder(c) if c.axis().dot(axis) > 1.0 - 1e-10 => {
                        boss_edges.push((edge, other));
                    }
                    FaceSurface::Torus(t)
                        if t.z_axis().dot(axis) > 1.0 - 1e-10
                            && topo.face(other)?.is_reversed()
                            && (walked || (t.minor_radius() - field.radius).abs() < 1e-7) =>
                    {
                        shoulder_edges.push((edge, other));
                    }
                    FaceSurface::Plane { .. }
                    | FaceSurface::Cylinder(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_)
                    | FaceSurface::Nurbs(_) => {}
                }
            }
        }
        let cap_edge = topo.edge(*contact)?;
        let shares = |edge: EdgeId| -> Result<bool, BlendError> {
            let e = topo.edge(edge)?;
            Ok([e.start(), e.end()]
                .iter()
                .any(|v| *v == cap_edge.start() || *v == cap_edge.end()))
        };
        let mut clipped = Vec::new();
        for pair in boss_edges {
            if shares(pair.0)? {
                clipped.push(pair);
            }
        }
        let boss_edges = clipped;
        let [(p, boss)] = boss_edges.as_slice() else {
            return Err(refusal(
                "restored corner has no unique clipped boss boundary",
            ));
        };
        let [(q, shoulder)] = shoulder_edges.as_slice() else {
            return Err(refusal(
                "restored corner has no unique clipped shoulder boundary",
            ));
        };
        let common = |edge| -> Result<VertexId, BlendError> {
            let e = topo.edge(edge)?;
            [e.start(), e.end()]
                .into_iter()
                .find(|v| *v == cap_edge.start() || *v == cap_edge.end())
                .ok_or_else(|| refusal("corner boundary misses its cap endpoint"))
        };
        let old_p = common(*p)?;
        let old_q = common(*q)?;
        if old_p == old_q {
            return Err(refusal("restored corner cap is already collapsed"));
        }
        let end = surface.evaluate(0.0, 1.0);
        for support in [*boss, *shoulder] {
            let s = topo.face(support)?.surface();
            let (u, v) = s
                .project_point(end)
                .ok_or_else(|| refusal("full corner endpoint has no support projection"))?;
            if s.evaluate(u, v).is_none_or(|p| (p - end).length() > 1e-7) {
                return Err(refusal("full corner endpoint leaves its analytic supports"));
            }
        }
        let rail = |row: usize| {
            NurbsCurve::new(
                1,
                surface.knots_v().to_vec(),
                surface.control_points()[row].clone(),
                surface.weights()[row].clone(),
            )
        };
        let p_curve = rail(0)?;
        // A walked corner closes on a collapsed section at the spring: both
        // rails end there, and its shoulder contact is the far rail.
        let q_curve = if matches!(topo.edge(*q)?.curve(), EdgeCurve::Circle(_)) {
            None
        } else if walked && (surface.evaluate(1.0, 1.0) - end).length() <= 1e-7 {
            Some(rail(2)?)
        } else {
            return Err(refusal("full shoulder contact has no circular authority"));
        };
        corners.push(Corner {
            face,
            boss: *boss,
            shoulder: *shoulder,
            contact: *contact,
            p: *p,
            q: *q,
            old_p,
            old_q,
            end,
            p_curve,
            q_curve,
        });
    }
    let [a, b] = corners.as_slice() else {
        return Err(refusal(
            "restoring a joined shoulder requires two qualified corner carriers",
        ));
    };
    if a.shoulder != b.shoulder {
        return Err(refusal("restored corners do not share one shoulder"));
    }
    let shoulder = a.shoulder;
    let FaceSurface::Torus(torus) = topo.face(shoulder)?.surface() else {
        return Err(refusal("restored shoulder changed carrier"));
    };
    let springs = |face: FaceId| -> Result<bool, BlendError> {
        Ok(
            matches!(topo.face(face)?.surface(), FaceSurface::Cylinder(c)
            if (torus.major_radius() - torus.minor_radius() - c.radius()).abs() <= 1e-7),
        )
    };
    // Both corners stand on the boss, or one stands on a seam round the cap
    // also cut: a seam tangent to the boss along the ruling through that
    // corner's spring end (the walked corner of a smaller top band).
    let (boss, seam) = if a.boss == b.boss {
        (a.boss, None)
    } else if springs(a.boss)? && !springs(b.boss)? {
        (a.boss, Some(1))
    } else if springs(b.boss)? && !springs(a.boss)? {
        (b.boss, Some(0))
    } else {
        return Err(refusal("restored corners do not share one boss"));
    };
    let FaceSurface::Cylinder(cylinder) = topo.face(boss)?.surface() else {
        return Err(refusal("restored boss changed carrier"));
    };
    if (torus.major_radius() - torus.minor_radius() - cylinder.radius()).abs() > 1e-7 {
        return Err(refusal("restored spring has inconsistent radii"));
    }
    let circle =
        Circle3D::new_with_ref(torus.center(), axis, cylinder.radius(), cylinder.x_axis())?;
    let start = circle.project(a.end);
    let mut span = (circle.project(b.end) - start).rem_euclid(TAU);
    if span > PI {
        span -= TAU;
    }
    if span.abs() >= PI || span.abs() < 1e-8 {
        return Err(refusal(
            "restored shoulder spring is not a unique minor arc",
        ));
    }
    let ends = [
        topo.add_vertex(Vertex::new(a.end, 1e-7)),
        topo.add_vertex(Vertex::new(b.end, 1e-7)),
    ];
    let spring = add_certified_curve_edge(
        topo,
        ends[0],
        ends[1],
        EdgeCurve::Circle(circle),
        (start, start + span),
    )?;
    let mut edge_images = Vec::new();
    let mut replacements = BTreeMap::new();
    let mut vertex_images = BTreeMap::new();
    let mut removed = BTreeSet::new();
    for (corner, end) in corners.iter().zip(ends) {
        vertex_images.insert(corner.old_p, end);
        vertex_images.insert(corner.old_q, end);
        removed.insert(corner.contact);
        let p = extend_rail(topo, corner.p, corner.old_p, &corner.p_curve, end)?;
        let q = match &corner.q_curve {
            Some(curve) => extend_rail(topo, corner.q, corner.old_q, curve, end)?,
            None => extend_q(topo, corner, end)?,
        };
        replacements.insert(corner.p, p);
        replacements.insert(corner.q, q);
        edge_images.push((corner.p, p));
        edge_images.push((corner.q, q));
    }
    // A seam round the cap cut: its cap contact goes, and its ruling on the
    // boss runs back up to the spring end its corner closes on.
    let mut restored = vec![boss, shoulder];
    if let Some(index) = seam {
        let corner = &corners[index];
        let end = ends[index];
        let seam = corner.boss;
        let contacts: Vec<_> = face_edges(topo, seam)?
            .into_iter()
            .filter(|&edge| {
                adjacency
                    .faces_for_edge(edge)
                    .iter()
                    .any(|f| rims.contains(f))
            })
            .collect();
        let [contact] = contacts.as_slice() else {
            return Err(refusal("restored seam has no unique old cap boundary"));
        };
        let data = topo.edge(*contact)?;
        let ruling_top = if data.start() == corner.old_p {
            data.end()
        } else if data.end() == corner.old_p {
            data.start()
        } else {
            return Err(refusal("restored seam's cap boundary misses its corner"));
        };
        let rulings: Vec<_> = face_edges(topo, seam)?
            .into_iter()
            .filter(|&edge| {
                adjacency.faces_for_edge(edge).contains(&boss)
                    && topo.edge(edge).is_ok_and(|e| {
                        matches!(e.curve(), EdgeCurve::Line)
                            && (e.start() == ruling_top || e.end() == ruling_top)
                    })
            })
            .collect();
        let [ruling] = rulings.as_slice() else {
            return Err(refusal("restored seam has no unique ruling on its boss"));
        };
        let data = topo.edge(*ruling)?;
        let (from, to) = (data.start(), data.end());
        let bottom = if from == ruling_top { to } else { from };
        let foot = topo.vertex(bottom)?.point();
        let top = topo.vertex(ruling_top)?.point();
        let direction = (top - foot).normalize()?;
        let offset = corner.end - foot;
        if (offset - direction * offset.dot(direction)).length() > 1e-7
            || offset.dot(direction) <= (top - foot).length()
        {
            return Err(refusal(
                "restored seam ruling does not reach the spring end",
            ));
        }
        let line = if from == ruling_top {
            remus_topology::edge::Edge::new(end, bottom, EdgeCurve::Line)
        } else {
            remus_topology::edge::Edge::new(bottom, end, EdgeCurve::Line)
        };
        let line = topo.add_edge(line);
        vertex_images.insert(ruling_top, end);
        removed.insert(*contact);
        replacements.insert(*ruling, line);
        edge_images.push((*ruling, line));
        restored.push(seam);
    }
    let mut spring_contacts = BTreeSet::new();
    for support in [boss, shoulder] {
        let contacts: Vec<_> = face_edges(topo, support)?
            .into_iter()
            .filter(|&edge| {
                adjacency
                    .faces_for_edge(edge)
                    .iter()
                    .any(|f| rims.contains(f))
            })
            .collect();
        let [contact] = contacts.as_slice() else {
            return Err(refusal("restored support has no unique old cap boundary"));
        };
        spring_contacts.insert(*contact);
    }
    for face in corners.iter().map(|c| c.face).chain(restored) {
        let data = topo.face(face)?;
        if !data.inner_wires().is_empty() {
            return Err(refusal("restoring a shoulder with holes is unsupported"));
        }
        let old = topo.wire(data.outer_wire())?.edges().to_vec();
        let mut edges = Vec::new();
        for oe in old {
            if removed.contains(&oe.edge()) {
                continue;
            }
            if let Some(&edge) = replacements.get(&oe.edge()) {
                edges.push(OrientedEdge::new(edge, oe.is_forward()));
            } else if spring_contacts.contains(&oe.edge()) {
                let from = oe.oriented_start(topo.edge(oe.edge())?);
                let from = vertex_images
                    .get(&from)
                    .ok_or_else(|| refusal("restored spring has no constructed endpoint image"))?;
                edges.push(OrientedEdge::new(spring, *from == ends[0]));
            } else {
                edges.push(oe);
            }
        }
        let wire = topo.add_wire(Wire::new(edges, true)?);
        topo.set_face_boundary_wires(face, wire, vec![])?;
    }
    Ok(SupportExtension {
        edge_images,
        spring,
        boss,
        shoulder,
    })
}

/// The clipped rail `edge` (ending at `old` on the cap contact) run on along
/// its corner's full rail `curve` to the spring `end` at the rail's `v = 1`.
fn extend_rail(
    topo: &mut Topology,
    edge: EdgeId,
    old: VertexId,
    curve: &NurbsCurve,
    end: VertexId,
) -> Result<EdgeId, BlendError> {
    let data = topo.edge(edge)?;
    let at_start = data.start() == old;
    let other = if at_start { data.end() } else { data.start() };
    let foot = project_point_to_curve(curve, topo.vertex(other)?.point(), 1e-9)?;
    if foot.distance > 1e-7 {
        return Err(refusal(
            "full boss contact does not retain its old endpoint",
        ));
    }
    let (a, b, domain) = if at_start {
        (end, other, (1.0, foot.parameter))
    } else {
        (other, end, (foot.parameter, 1.0))
    };
    add_certified_curve_edge(topo, a, b, EdgeCurve::NurbsCurve(curve.clone()), domain)
}

fn extend_q(topo: &mut Topology, corner: &Corner, end: VertexId) -> Result<EdgeId, BlendError> {
    let data = topo.edge(corner.q)?;
    let EdgeCurve::Circle(circle) = data.curve() else {
        return Err(refusal("full shoulder contact has no circular authority"));
    };
    let (lo, hi) = data
        .strict_domain()
        .map_err(|_| refusal("clipped shoulder contact has no parameter authority"))?;
    let at_start = data.start() == corner.old_q;
    let other = if at_start { data.end() } else { data.start() };
    let projected = circle.project(corner.end);
    let near = if at_start { lo } else { hi };
    let base = projected + TAU * ((near - projected) / TAU).round();
    let mut candidates = Vec::new();
    for k in -1..=1 {
        let t = base + f64::from(k) * TAU;
        let (a, b) = if at_start { (t, hi) } else { (lo, t) };
        if (b - a).abs() < PI && lo.min(hi) >= a.min(b) - 1e-10 && lo.max(hi) <= a.max(b) + 1e-10 {
            candidates.push((a, b));
        }
    }
    let [domain] = candidates.as_slice() else {
        return Err(refusal("full shoulder contact has no unique original arc"));
    };
    let curve = EdgeCurve::Circle(circle.clone());
    let (a, b) = if at_start { (end, other) } else { (other, end) };
    add_certified_curve_edge(topo, a, b, curve, *domain)
}
