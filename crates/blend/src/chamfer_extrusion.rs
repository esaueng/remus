//! Exact straight-ridge chamfers on proven spline extrusion supports.
//!
//! The rational control net proves translation symmetry. Only the section
//! contacts require a numerical solve; their full longitudinal carriers are
//! exact lines, and the band is an exact plane. No sampled surface fit is used.

use remus_math::{
    curves2d::{Curve2D, NurbsCurve2D},
    nurbs::{curve::NurbsCurve, knot_ops::curve_split},
    tolerance::Tolerance,
    traits::ParametricSurface,
    vec::{Point2, Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    face::{FaceId, FaceSurface},
};

use crate::{
    BlendError, analytic,
    blend_func::{BlendParams, ChamferBlend},
    builder_utils::surface_ref_or_adapter,
    spine::Spine,
    stripe::{Stripe, StripeResult},
    walker::{Walker, WalkerConfig},
};

/// Which parameter follows the profile rather than the extrusion direction.
#[derive(Clone, Copy)]
pub enum ProfileAxis {
    U,
    V,
}

#[derive(Clone, Copy)]
pub enum ExtrusionSupport {
    Plane,
    Spline(ProfileAxis),
}

#[allow(clippy::float_cmp)] // Equal weight pairs prove affine translation.
pub fn profile_axis(surface: &FaceSurface, axis: Vec3) -> Option<ExtrusionSupport> {
    let tol = Tolerance::new().linear;
    match surface {
        FaceSurface::Plane { normal, .. } if normal.dot(axis).abs() < Tolerance::new().angular => {
            Some(ExtrusionSupport::Plane)
        }
        FaceSurface::Nurbs(s) => {
            let cp = s.control_points();
            let weights = s.weights();
            for (degree, count, profile) in [
                (s.degree_v(), cp[0].len(), ProfileAxis::U),
                (s.degree_u(), cp.len(), ProfileAxis::V),
            ] {
                if degree != 1 || count != 2 {
                    continue;
                }
                let (rows, cols) = match profile {
                    ProfileAxis::U => (cp.len(), cp[0].len()),
                    ProfileAxis::V => (cp[0].len(), cp.len()),
                };
                let at = |i: usize, j: usize| match profile {
                    ProfileAxis::U => (cp[i][j], weights[i][j]),
                    ProfileAxis::V => (cp[j][i], weights[j][i]),
                };
                let shift = at(0, cols - 1).0 - at(0, 0).0;
                if shift.length() <= tol || shift.cross(axis).length() > tol {
                    continue;
                }
                if (0..rows).all(|i| {
                    let (a, wa) = at(i, 0);
                    let (b, wb) = at(i, cols - 1);
                    wa.is_finite() && wa > 0.0 && wa == wb && (b - a - shift).length() <= tol
                }) {
                    return Some(ExtrusionSupport::Spline(profile));
                }
            }
            None
        }
        _ => None,
    }
}

/// Prove that the consumed profile interval stays inside the setback ball.
/// Positive rational weights make this a complete control-hull bound, so a
/// distant Newton branch across an intervening outside interval is refused.
fn certify_profile(
    surface: &FaceSurface,
    profile: ExtrusionSupport,
    guide: Point3,
    axis: Vec3,
    into: Vec3,
    uv: (f64, f64),
    setback: f64,
) -> Result<bool, BlendError> {
    let (FaceSurface::Nurbs(s), ExtrusionSupport::Spline(profile)) = (surface, profile) else {
        return Ok(true);
    };
    let Some(uv0) = surface.project_point(guide) else {
        return Ok(false);
    };
    let (degree, knots, points, weights, q0, q, derivative) = match profile {
        ProfileAxis::U => (
            s.degree_u(),
            s.knots_u().to_vec(),
            s.control_points()
                .iter()
                .map(|row| row[0])
                .collect::<Vec<_>>(),
            s.weights().iter().map(|row| row[0]).collect::<Vec<_>>(),
            uv0.0,
            uv.0,
            s.partial_u(uv0.0, uv0.1),
        ),
        ProfileAxis::V => (
            s.degree_v(),
            s.knots_v().to_vec(),
            s.control_points()[0].clone(),
            s.weights()[0].clone(),
            uv0.1,
            uv.1,
            s.partial_v(uv0.0, uv0.1),
        ),
    };
    if derivative.dot(into) * (q - q0) <= 0.0 {
        return Ok(false);
    }
    let points = points
        .into_iter()
        .map(|p| p - axis * axis.dot(p - guide))
        .collect();
    let mut curve = NurbsCurve::new(degree, knots, points, weights)?;
    let (lo, hi) = (q.min(q0), q.max(q0));
    let (start, end) = curve.domain();
    if lo < start || hi > end || hi - lo <= f64::EPSILON {
        return Ok(false);
    }
    // Keep a conservative superset when a parameter is indistinguishable
    // from an endpoint to the knot-insertion routine.
    let endpoint_band = 1e-10 * (end - start);
    if hi < end - endpoint_band {
        curve = curve_split(&curve, hi)?.0;
    }
    if lo > start + endpoint_band {
        curve = curve_split(&curve, lo)?.1;
    }
    Ok(curve
        .control_points()
        .iter()
        .all(|p| (*p - guide).length() <= setback + Tolerance::new().linear))
}

fn line(a: Point3, b: Point3) -> Result<NurbsCurve, BlendError> {
    Ok(NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![a, b],
        vec![1.0; 2],
    )?)
}

fn pcurve(surface: &dyn ParametricSurface, a: Point3, b: Point3) -> Result<Curve2D, BlendError> {
    let a = surface.project_point(a);
    let b = surface.project_point(b);
    Ok(Curve2D::Nurbs(NurbsCurve2D::from_line(
        Point2::new(a.0, a.1),
        Point2::new(b.0, b.1),
    )?))
}

/// Construct a stripe only after proving both supports have a constant
/// transverse section along this straight ridge.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub fn stripe(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    spine: Spine,
    face1: FaceId,
    face2: FaceId,
    d1: f64,
    d2: f64,
) -> Result<Option<StripeResult>, BlendError> {
    let edge = topo.edge(spine.edges()[0])?;
    if !matches!(edge.curve(), EdgeCurve::Line) || spine.is_closed() {
        return Ok(None);
    }
    let axis = spine.tangent(topo, 0.0)?;
    let s1 = topo.face(face1)?.surface();
    let s2 = topo.face(face2)?.surface();
    let (Some(profile1), Some(profile2)) = (profile_axis(s1, axis), profile_axis(s2, axis)) else {
        return Ok(None);
    };
    let Some(frames) = analytic::edge_material_frames(topo, &spine, face1, face2)? else {
        return Ok(None);
    };
    if frames.concave().is_none() {
        return Ok(None);
    }
    let guide = spine.evaluate(topo, 0.0)?;
    let mut adapter1 = None;
    let mut adapter2 = None;
    let ps1 = surface_ref_or_adapter(s1, &mut adapter1);
    let ps2 = surface_ref_or_adapter(s2, &mut adapter2);
    let blend = ChamferBlend { d1, d2 };
    let (u1, v1) = ps1.project_point(guide + frames.into1 * d1);
    let (u2, v2) = ps2.project_point(guide + frames.into2 * d2);
    let config = WalkerConfig {
        tol_3d: Tolerance::new().linear * 0.01,
        ..WalkerConfig::default()
    };
    let walker = Walker::new(&blend, ps1, ps2, &spine, topo, config);
    let Some((_, start)) = walker.solve_section(0.0, Some(BlendParams { u1, v1, u2, v2 })) else {
        log::debug!("extrusion chamfer: initial section did not converge");
        return Ok(None);
    };
    if (start.p1 - guide).dot(frames.into1) <= 0.0
        || (start.p2 - guide).dot(frames.into2) <= 0.0
        || !certify_profile(s1, profile1, guide, axis, frames.into1, start.uv1, d1)?
        || !certify_profile(s2, profile2, guide, axis, frames.into2, start.uv2, d2)?
    {
        log::debug!("extrusion chamfer: consumed profile interval was not certified");
        return Ok(None);
    }
    let shift = spine.evaluate(topo, spine.length())? - guide;
    let region = remus_math::aabb::Aabb3::from_points([guide, guide + shift]).expanded(d1.max(d2));
    if !crate::chamfer_clearance::holes_clear(topo, adjacency, region)? {
        return Ok(None);
    }
    let tol = Tolerance::new().linear;
    // The control-net proof makes these translations exact across the full
    // span. Solving again at a clamped carrier endpoint is unnecessary and
    // can stagnate by a few ulps. Check finite-carrier membership instead.
    let mut end = start.clone();
    end.t = 1.0;
    end.p1 = start.p1 + shift;
    end.p2 = start.p2 + shift;
    end.center = start.center + shift;
    end.uv1 = ps1.project_point(end.p1);
    end.uv2 = ps2.project_point(end.p2);
    if (ps1.evaluate(end.uv1.0, end.uv1.1) - end.p1).length() > tol
        || (ps2.evaluate(end.uv2.0, end.uv2.1) - end.p2).length() > tol
    {
        return Ok(None);
    }
    let mut normal = axis.cross(start.p2 - start.p1).normalize()?;
    if normal.dot(frames.outward1 + frames.outward2) < 0.0 {
        normal = -normal;
    }
    let surface = FaceSurface::Plane {
        normal,
        d: normal.dot(start.p1 - Point3::new(0.0, 0.0, 0.0)),
    };
    let result = StripeResult {
        stripe: Stripe {
            pcurve1: pcurve(ps1, start.p1, end.p1)?,
            pcurve2: pcurve(ps2, start.p2, end.p2)?,
            contact1: line(start.p1, end.p1)?,
            contact2: line(start.p2, end.p2)?,
            face1,
            face2,
            spine,
            surface,
            sections: vec![start, end],
        },
        new_edges: Vec::new(),
    };
    Ok(Some(result))
}

/// Relimit a normal planar end cap through the original ridge vertex. The
/// source path may contain exact curved profile edges; the replacement shares
/// the band's straight cross edge and leaves every other carrier untouched.
pub fn notch_cap(
    topo: &mut Topology,
    face: FaceId,
    cross: (
        remus_topology::edge::EdgeId,
        remus_topology::vertex::VertexId,
        remus_topology::vertex::VertexId,
    ),
    corner: remus_topology::vertex::VertexId,
    axis: Vec3,
) -> Result<Option<FaceId>, BlendError> {
    use remus_topology::{
        face::Face,
        wire::{OrientedEdge, Wire},
    };
    let f = topo.face(face)?;
    let FaceSurface::Plane { normal, d } = f.surface() else {
        return Ok(None);
    };
    if normal.cross(axis).length() > Tolerance::new().angular {
        return Ok(None);
    }
    let (edge, a, b) = cross;
    let tol = Tolerance::new().linear;
    if [a, b].iter().any(|&v| {
        !topo
            .vertex(v)
            .is_ok_and(|v| (normal.dot(v.point() - Point3::new(0.0, 0.0, 0.0)) - d).abs() <= tol)
    }) {
        return Ok(None);
    }
    let wires = std::iter::once(f.outer_wire())
        .chain(f.inner_wires().iter().copied())
        .collect::<Vec<_>>();
    for (wi, wire) in wires.into_iter().enumerate() {
        let uses = topo.wire(wire)?.edges().to_vec();
        if uses.len() < 3 {
            continue;
        }
        for i in 0..uses.len() {
            let j = (i + 1) % uses.len();
            let first = topo.edge(uses[i].edge())?;
            let second = topo.edge(uses[j].edge())?;
            let (start, middle, end) = (
                uses[i].oriented_start(first),
                uses[i].oriented_end(first),
                uses[j].oriented_end(second),
            );
            if middle != corner || uses[j].oriented_start(second) != corner {
                continue;
            }
            let forward = start == a && end == b;
            if !(forward || start == b && end == a) {
                continue;
            }
            let mut edges = Vec::with_capacity(uses.len() - 1);
            for (k, use_) in uses.iter().enumerate() {
                if k == i {
                    edges.push(OrientedEdge::new(edge, forward));
                } else if k != j {
                    edges.push(*use_);
                }
            }
            let new_wire = topo.add_wire(Wire::new(edges, true)?);
            let f = topo.face(face)?;
            let (mut outer, mut inner) = (f.outer_wire(), f.inner_wires().to_vec());
            if wi == 0 {
                outer = new_wire;
            } else {
                inner[wi - 1] = new_wire;
            }
            let mut replacement = Face::new(outer, inner, f.surface().clone());
            replacement.set_reversed(f.is_reversed());
            return Ok(Some(topo.add_face(replacement)));
        }
    }
    Ok(None)
}

/// Choose the band's wire traversal from its actual shared contact use.
/// Surface normals keep their independently derived outward direction.
pub fn orient_band(
    topo: &mut Topology,
    band: FaceId,
    neighbor: FaceId,
) -> Result<FaceId, BlendError> {
    use remus_topology::{
        face::Face,
        wire::{OrientedEdge, Wire},
    };
    let face = topo.face(band)?;
    let edges = topo.wire(face.outer_wire())?.edges().to_vec();
    let neighbor = topo.face(neighbor)?;
    let mut flip = None;
    for wire in std::iter::once(neighbor.outer_wire()).chain(neighbor.inner_wires().iter().copied())
    {
        for usage in topo.wire(wire)?.edges() {
            if let Some(own) = edges.iter().find(|own| own.edge() == usage.edge()) {
                flip = Some(
                    (own.is_forward() != face.is_reversed())
                        == (usage.is_forward() != neighbor.is_reversed()),
                );
                break;
            }
        }
    }
    match flip {
        Some(false) => Ok(band),
        Some(true) => {
            let surface = face.surface().clone();
            let face_reversed = face.is_reversed();
            let reversed = edges
                .iter()
                .rev()
                .map(|e| OrientedEdge::new(e.edge(), !e.is_forward()))
                .collect();
            let wire = topo.add_wire(Wire::new(reversed, true)?);
            let mut replacement = Face::new(wire, Vec::new(), surface);
            replacement.set_reversed(face_reversed);
            Ok(topo.add_face(replacement))
        }
        None => Err(BlendError::TrimmingFailure { face: band }),
    }
}

/// Store and validate parameter authority on every boundary of newly built
/// planar/spline faces. Spline profile uses are exact isoparametric pcurves;
/// planar spline uses are affine images of their original rational carriers.
pub fn register_pcurves(topo: &mut Topology, faces: &[FaceId]) -> Result<(), BlendError> {
    use remus_math::{
        curves2d::{Circle2D, Line2D},
        vec::Vec2,
    };
    use remus_topology::pcurve::PCurve;
    for &face in faces {
        let f = topo.face(face)?;
        let surface = f.surface().clone();
        if !matches!(surface, FaceSurface::Plane { .. } | FaceSurface::Nurbs(_)) {
            continue;
        }
        let uses = std::iter::once(f.outer_wire())
            .chain(f.inner_wires().iter().copied())
            .map(|wire| topo.wire(wire).map(|w| w.edges().to_vec()))
            .collect::<Result<Vec<_>, _>>()?;
        let mut adapter = None;
        let support = surface_ref_or_adapter(&surface, &mut adapter);
        for usage in uses.into_iter().flatten() {
            if topo
                .pcurve_oriented(usage.edge(), face, usage.is_forward())
                .is_some()
            {
                continue;
            }
            let edge = topo.edge(usage.edge())?;
            let domain = edge.strict_domain().map_err(|e| BlendError::InvalidInput {
                reason: format!("freeform chamfer boundary lacks parameter authority: {e}"),
            })?;
            let mut pc_domain = domain;
            let pc = match (edge.curve(), &surface) {
                (EdgeCurve::Circle(circle), FaceSurface::Plane { .. }) => {
                    let c = support.project_point(circle.center());
                    let curve = Circle2D::new(Point2::new(c.0, c.1), circle.radius())?;
                    let start = support.project_point(circle.evaluate(domain.0));
                    let parameter = curve.project(Point2::new(start.0, start.1));
                    let sign = if support.normal(0.0, 0.0).dot(circle.normal()) >= 0.0 {
                        1.0
                    } else {
                        -1.0
                    };
                    pc_domain = (parameter, parameter + sign * (domain.1 - domain.0));
                    Curve2D::Circle(curve)
                }
                (EdgeCurve::NurbsCurve(curve), FaceSurface::Plane { .. }) => {
                    let points = curve
                        .control_points()
                        .iter()
                        .map(|&p| {
                            let uv = support.project_point(p);
                            Point2::new(uv.0, uv.1)
                        })
                        .collect();
                    Curve2D::Nurbs(NurbsCurve2D::new(
                        curve.degree(),
                        curve.knots().to_vec(),
                        points,
                        curve.weights().to_vec(),
                    )?)
                }
                (EdgeCurve::Line, FaceSurface::Nurbs(s)) => {
                    let a = support.project_point(topo.vertex(edge.start())?.point());
                    let b = support.project_point(topo.vertex(edge.end())?.point());
                    let snap = |v: f64, d: (f64, f64)| {
                        if (v - d.0).abs() < 1e-10 {
                            d.0
                        } else if (v - d.1).abs() < 1e-10 {
                            d.1
                        } else {
                            v
                        }
                    };
                    if (a.1 - b.1).abs() < 1e-6 && (a.0 - b.0).abs() > 1e-12 {
                        let sign = (b.0 - a.0).signum();
                        pc_domain = (0.0, (b.0 - a.0).abs());
                        Curve2D::Line(Line2D::new(
                            Point2::new(a.0, snap(a.1, s.domain_v())),
                            Vec2::new(sign, 0.0),
                        )?)
                    } else if (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() > 1e-12 {
                        let sign = (b.1 - a.1).signum();
                        pc_domain = (0.0, (b.1 - a.1).abs());
                        Curve2D::Line(Line2D::new(
                            Point2::new(snap(a.0, s.domain_u()), a.1),
                            Vec2::new(0.0, sign),
                        )?)
                    } else {
                        return Err(BlendError::TrimmingFailure { face });
                    }
                }
                (EdgeCurve::NurbsCurve(_), FaceSurface::Nurbs(s)) => {
                    let a = support.project_point(topo.vertex(edge.start())?.point());
                    let b = support.project_point(topo.vertex(edge.end())?.point());
                    // Projection supplies a candidate chart, not acceptance.
                    // A small physical residual can occupy more UV space on
                    // scaled supports; the full-interval certificate below
                    // remains the authority at the unchanged model tolerance.
                    let close = |a: f64, b: f64| (a - b).abs() <= 1e-6;
                    let snap = |value: f64, domain: (f64, f64)| {
                        if (value - domain.0).abs() <= 1e-10 {
                            domain.0
                        } else if (value - domain.1).abs() <= 1e-10 {
                            domain.1
                        } else {
                            value
                        }
                    };
                    if close(a.0, domain.0) && close(b.0, domain.1) && close(a.1, b.1) {
                        Curve2D::Line(Line2D::new(
                            Point2::new(0.0, snap(a.1, s.domain_v())),
                            Vec2::new(1.0, 0.0),
                        )?)
                    } else if close(a.1, domain.0) && close(b.1, domain.1) && close(a.0, b.0) {
                        Curve2D::Line(Line2D::new(
                            Point2::new(snap(a.0, s.domain_u()), 0.0),
                            Vec2::new(0.0, 1.0),
                        )?)
                    } else if close(a.0, s.domain_u().0 + s.domain_u().1 - domain.0)
                        && close(b.0, s.domain_u().0 + s.domain_u().1 - domain.1)
                        && close(a.1, b.1)
                    {
                        Curve2D::Line(Line2D::new(
                            Point2::new(s.domain_u().0 + s.domain_u().1, snap(a.1, s.domain_v())),
                            Vec2::new(-1.0, 0.0),
                        )?)
                    } else if close(a.1, s.domain_v().0 + s.domain_v().1 - domain.0)
                        && close(b.1, s.domain_v().0 + s.domain_v().1 - domain.1)
                        && close(a.0, b.0)
                    {
                        Curve2D::Line(Line2D::new(
                            Point2::new(snap(a.0, s.domain_u()), s.domain_v().0 + s.domain_v().1),
                            Vec2::new(0.0, -1.0),
                        )?)
                    } else {
                        log::debug!(
                            "extrusion chamfer: boundary {:?} UV endpoints {:?}/{:?} do not match native domain {:?}",
                            usage.edge(),
                            a,
                            b,
                            domain
                        );
                        return Err(BlendError::TrimmingFailure { face });
                    }
                }
                (EdgeCurve::Line, _) => {
                    let a = support.project_point(topo.vertex(edge.start())?.point());
                    let b = support.project_point(topo.vertex(edge.end())?.point());
                    let direction = Vec2::new(b.0 - a.0, b.1 - a.1);
                    pc_domain = (0.0, direction.length());
                    Curve2D::Line(Line2D::new(Point2::new(a.0, a.1), direction)?)
                }
                _ => return Err(BlendError::TrimmingFailure { face }),
            };
            let (t0, t1) = if usage.is_forward() {
                pc_domain
            } else {
                (pc_domain.1, pc_domain.0)
            };
            topo.set_pcurve_oriented(
                usage.edge(),
                face,
                usage.is_forward(),
                PCurve::new(pc, t0, t1),
            )?;
            remus_topology::validation::validate_same_parameter_strict(
                topo,
                usage.edge(),
                face,
                usage.is_forward(),
                Tolerance::new().linear,
                0,
            )
            .map_err(|e| BlendError::InvalidInput {
                reason: format!("freeform chamfer contact parameter certificate: {e}"),
            })?;
            remus_topology::validation::validate_same_range_strict(
                topo,
                usage.edge(),
                face,
                usage.is_forward(),
                Tolerance::new().linear,
            )
            .map_err(|e| BlendError::InvalidInput {
                reason: format!("freeform chamfer contact range certificate: {e}"),
            })?;
        }
    }
    Ok(())
}
