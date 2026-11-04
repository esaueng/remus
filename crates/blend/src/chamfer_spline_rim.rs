//! Ruled chamfers on monotone spline rims of linear extrusions.

use crate::{
    BlendError, analytic,
    builder_utils::{PlaneAdapter, surface_ref_or_adapter},
    chamfer_extrusion::{ExtrusionSupport, ProfileAxis, profile_axis},
    section::CircSection,
    spine::Spine,
    stripe::{Stripe, StripeResult},
};
use remus_math::{
    compact_sort::CompactSort,
    context::{OperationContext, WorkBudgets},
    curves2d::{Curve2D, Line2D, NurbsCurve2D},
    nurbs::{
        NurbsCurve, NurbsSurface, curve_degree_elevate, curve_knot_insert,
        planar_offset::{PlanarOffsetOptions, fit_planar_offset},
    },
    tolerance::Tolerance,
    traits::ParametricSurface,
    vec::{Point2, Point3, Vec2, Vec3},
};
use remus_topology::{
    Topology,
    adjacency::AdjacencyIndex,
    edge::{EdgeCurve, EdgeId},
    face::{FaceId, FaceSurface},
    vertex::VertexId,
};

fn boundary_points(topo: &Topology, face: FaceId) -> Result<Vec<Point3>, BlendError> {
    let f = topo.face(face)?;
    let mut result = Vec::new();
    for use_ in topo.wire(f.outer_wire())?.edges() {
        let e = topo.edge(use_.edge())?;
        match e.curve() {
            EdgeCurve::NurbsCurve(c) => result.extend_from_slice(c.control_points()),
            EdgeCurve::Line => {
                result.push(topo.vertex(e.start())?.point());
                result.push(topo.vertex(e.end())?.point());
            }
            EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Err(BlendError::TrimmingFailure { face }),
        }
    }
    Ok(result)
}

fn cap_clearance(
    topo: &Topology,
    cap: FaceId,
    seed: EdgeId,
    contact: &NurbsCurve,
    normal: Vec3,
) -> Result<bool, BlendError> {
    let f = topo.face(cap)?;
    let points = boundary_points(topo, cap)?;
    let tol = Tolerance::new().linear;
    for use_ in topo.wire(f.outer_wire())?.edges() {
        if use_.edge() == seed {
            continue;
        }
        let e = topo.edge(use_.edge())?;
        if !matches!(e.curve(), EdgeCurve::Line) {
            return Ok(false);
        }
        let a = topo.vertex(e.start())?.point();
        let b = topo.vertex(e.end())?.point();
        let mut along = (b - a).normalize()?;
        if use_.is_forward() == f.is_reversed() {
            along = -along;
        }
        let into = normal.cross(along).normalize()?;
        if points
            .iter()
            .chain(contact.control_points())
            .any(|p| into.dot(*p - a) < -tol)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn normal_end(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    wall: FaceId,
    cap: FaceId,
    vertex: VertexId,
    axis: Vec3,
) -> Result<bool, BlendError> {
    for &face in adjacency.neighbors_of_face(wall) {
        if face == cap {
            continue;
        }
        let f = topo.face(face)?;
        let FaceSurface::Plane { normal, .. } = f.surface() else {
            continue;
        };
        if normal.normalize()?.cross(axis).length() > Tolerance::new().angular {
            continue;
        }
        if topo.wire(f.outer_wire())?.edges().iter().any(|use_| {
            topo.edge(use_.edge())
                .is_ok_and(|e| e.start() == vertex || e.end() == vertex)
        }) {
            return Ok(true);
        }
    }
    Ok(false)
}

#[allow(clippy::float_cmp)] // Knot multiplicities are exact carrier data.
fn common_basis(a: &NurbsCurve, b: &NurbsCurve) -> Result<(NurbsCurve, NurbsCurve), BlendError> {
    let degree = a.degree().max(b.degree());
    let mut a = curve_degree_elevate(a, degree - a.degree())?;
    let mut b = curve_degree_elevate(b, degree - b.degree())?;
    let mut knots = a
        .knots()
        .iter()
        .chain(b.knots())
        .copied()
        .collect::<Vec<_>>();
    knots.compact_sort_by(f64::total_cmp);
    knots.dedup();
    let (start, end) = a.domain();
    for value in knots {
        if value <= start || value >= end {
            continue;
        }
        let ca = a.knots().iter().filter(|&&k| k == value).count();
        let cb = b.knots().iter().filter(|&&k| k == value).count();
        let count = ca.max(cb);
        if count > ca {
            a = curve_knot_insert(&a, value, count - ca)?;
        }
        if count > cb {
            b = curve_knot_insert(&b, value, count - cb)?;
        }
    }
    Ok((a, b))
}

fn ruled_surface(a: &NurbsCurve, b: &NurbsCurve) -> Result<NurbsSurface, BlendError> {
    let (a, b) = common_basis(a, b)?;
    let weights = |curve: &NurbsCurve| {
        curve
            .weights()
            .iter()
            .map(|w| w / curve.weights()[0])
            .collect::<Vec<_>>()
    };
    if a.weights().first() != a.weights().last() || b.weights().first() != b.weights().last() {
        return Err(BlendError::InvalidInput {
            reason: "spline rim cross edges require matching endpoint weight gauges".into(),
        });
    }
    Ok(NurbsSurface::new(
        1,
        a.degree(),
        vec![0.0, 0.0, 1.0, 1.0],
        a.knots().to_vec(),
        vec![a.control_points().to_vec(), b.control_points().to_vec()],
        vec![weights(&a), weights(&b)],
    )?)
}

fn wall_pcurve(
    surface: &FaceSurface,
    profile: ProfileAxis,
    contact: &NurbsCurve,
) -> Result<Curve2D, BlendError> {
    let Some((u, v)) = surface.project_point(contact.evaluate(contact.domain().0)) else {
        return Err(BlendError::InvalidInput {
            reason: "spline rim wall contact projection failed".into(),
        });
    };
    Ok(Curve2D::Line(match profile {
        ProfileAxis::U => Line2D::new(Point2::new(0.0, v), Vec2::new(1.0, 0.0))?,
        ProfileAxis::V => Line2D::new(Point2::new(u, 0.0), Vec2::new(0.0, 1.0))?,
    }))
}

fn plane_pcurve(
    surface: &dyn ParametricSurface,
    contact: &NurbsCurve,
) -> Result<Curve2D, BlendError> {
    let points = contact
        .control_points()
        .iter()
        .map(|&p| {
            let (u, v) = surface.project_point(p);
            Point2::new(u, v)
        })
        .collect();
    Ok(Curve2D::Nurbs(NurbsCurve2D::new(
        contact.degree(),
        contact.knots().to_vec(),
        points,
        contact.weights().to_vec(),
    )?))
}

/// Compute a ruled band with a certified parallel planar contact and an exact
/// translated wall contact. Every end and retained cap boundary is qualified
/// before the builder mutates topology.
#[allow(clippy::too_many_arguments, clippy::float_cmp)]
pub fn stripe(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    spine: Spine,
    face1: FaceId,
    face2: FaceId,
    d1: f64,
    d2: f64,
) -> Result<Option<StripeResult>, BlendError> {
    let edge_id = spine.edges()[0];
    let edge = topo.edge(edge_id)?;
    let EdgeCurve::NurbsCurve(guide) = edge.curve() else {
        return Ok(None);
    };
    if spine.is_closed() || edge.strict_domain().ok() != Some(guide.domain()) {
        return Ok(None);
    }
    let f1 = topo.face(face1)?;
    let f2 = topo.face(face2)?;
    let (cap, wall, dc, dw, cap_first) = match f1.surface() {
        FaceSurface::Plane { .. } => match f2.surface() {
            FaceSurface::Nurbs(_) => (face1, face2, d1, d2, true),
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => return Ok(None),
        },
        FaceSurface::Nurbs(_) => match f2.surface() {
            FaceSurface::Plane { .. } => (face2, face1, d2, d1, false),
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => return Ok(None),
        },
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => return Ok(None),
    };
    let cap_face = topo.face(cap)?;
    let FaceSurface::Plane { normal, d } = cap_face.surface() else {
        return Ok(None);
    };
    let raw_normal = normal.normalize()?;
    let cap_out = if cap_face.is_reversed() {
        -raw_normal
    } else {
        raw_normal
    };
    let wall_surface = topo.face(wall)?.surface();
    let Some(ExtrusionSupport::Spline(profile)) = profile_axis(wall_surface, cap_out) else {
        return Ok(None);
    };
    let Some(frames) = analytic::edge_material_frames(topo, &spine, face1, face2)? else {
        return Ok(None);
    };
    let (into_cap, into_wall) = if cap_first {
        (frames.into1, frames.into2)
    } else {
        (frames.into2, frames.into1)
    };
    if into_wall.dot(-cap_out) < 1.0 - 1e-10 {
        return Ok(None);
    }
    let (a, b) = guide.domain();
    let axis = (guide.evaluate(b) - guide.evaluate(a)).normalize()?;
    if [a, b].iter().any(|&t| {
        !guide
            .tangent(t)
            .is_ok_and(|v| v.cross(axis).length() <= Tolerance::new().angular)
    }) {
        return Ok(None);
    }
    if !normal_end(topo, adjacency, wall, cap, edge.start(), axis)?
        || !normal_end(topo, adjacency, wall, cap, edge.end(), axis)?
    {
        return Ok(None);
    }
    let middle = a.midpoint(b);
    let tangent = guide.tangent(middle)?;
    let offset_normal = if raw_normal.cross(tangent).dot(into_cap) > 0.0 {
        raw_normal
    } else {
        -raw_normal
    };
    let context = OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_segments(1364)
            .with_subdivision_depth(14),
    );
    let options = PlanarOffsetOptions {
        max_work: 16_000_000,
        max_segments: 1364,
        max_depth: 14,
        ..PlanarOffsetOptions::new(Tolerance::new().linear * 0.1)
    };
    let fit = fit_planar_offset(guide, offset_normal, dc, &options, &context).map_err(|e| {
        BlendError::InvalidInput {
            reason: format!("spline rim contact could not be certified: {e}"),
        }
    })?;
    if !cap_clearance(topo, cap, edge_id, &fit.curve, cap_out)? {
        return Ok(None);
    }
    log::debug!(target:"remus_approx","spline rim chamfer: bounded parallel contact, error <= {}, {} cubic pieces, {} charged work",fit.position_bound,fit.segments,fit.work_used);
    let shift = -cap_out * dw;
    let wall_contact = NurbsCurve::new(
        guide.degree(),
        guide.knots().to_vec(),
        guide.control_points().iter().map(|&p| p + shift).collect(),
        guide.weights().to_vec(),
    )?;
    let region = remus_math::aabb::Aabb3::from_points(
        guide
            .control_points()
            .iter()
            .chain(fit.curve.control_points())
            .chain(wall_contact.control_points())
            .copied(),
    );
    if !crate::chamfer_clearance::holes_clear(topo, adjacency, region)? {
        return Ok(None);
    }
    let cap_adapter = PlaneAdapter::from_normal_and_d(*normal, *d);
    let wall_pc = wall_pcurve(wall_surface, profile, &wall_contact)?;
    let cap_pc = plane_pcurve(&cap_adapter, &fit.curve)?;
    let (contact1, contact2, pcurve1, pcurve2) = if cap_first {
        (fit.curve, wall_contact, cap_pc, wall_pc)
    } else {
        (wall_contact, fit.curve, wall_pc, cap_pc)
    };
    let surface = FaceSurface::Nurbs(ruled_surface(&contact1, &contact2)?);
    let mut adapter1 = None;
    let mut adapter2 = None;
    let ps1 = surface_ref_or_adapter(f1.surface(), &mut adapter1);
    let ps2 = surface_ref_or_adapter(f2.surface(), &mut adapter2);
    let sections = [a, b]
        .iter()
        .enumerate()
        .map(|(i, &t)| {
            let p1 = contact1.evaluate(t);
            let p2 = contact2.evaluate(t);
            CircSection {
                p1,
                p2,
                center: p1 + (p2 - p1) * 0.5,
                radius: 0.0,
                uv1: ps1.project_point(p1),
                uv2: ps2.project_point(p2),
                t: if i == 0 { 0.0 } else { 1.0 },
            }
        })
        .collect();
    Ok(Some(StripeResult {
        stripe: Stripe {
            spine,
            surface,
            pcurve1,
            pcurve2,
            contact1,
            contact2,
            face1,
            face2,
            sections,
        },
        new_edges: Vec::new(),
    }))
}

/// The ruled sheet's natural normal may face either way. Its cap-normal
/// component has one sign throughout the certified monotone strip.
pub fn orient_surface(
    topo: &mut Topology,
    band: FaceId,
    stripe: &Stripe,
) -> Result<(), BlendError> {
    let FaceSurface::Nurbs(surface) = &stripe.surface else {
        return Ok(());
    };
    let cap = [stripe.face1, stripe.face2].iter().find_map(|&f| {
        topo.face(f).ok().and_then(|f| match f.surface() {
            FaceSurface::Plane { normal, .. } => {
                Some(if f.is_reversed() { -*normal } else { *normal })
            }
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => None,
        })
    });
    if let Some(cap) = cap {
        let domain = surface.domain_v();
        let reversed = surface.normal(0.5, domain.0.midpoint(domain.1))?.dot(cap) < 0.0;
        topo.face_mut(band)?.set_reversed(reversed);
    }
    Ok(())
}
