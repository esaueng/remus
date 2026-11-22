//! Conservative radial separation proofs for periodic face sweeps.

use remus_math::{
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    face::{FaceId, FaceSurface},
};

/// The projection of the candidate box fits in a disk about its projected
/// center. If even this disk misses the trimmed face, their boxes overlapped
/// across empty space, as on a ledge curving around a threaded boss.
pub(super) fn outside_planar_sweep(
    topo: &Topology,
    moved: FaceId,
    candidate: FaceId,
    delta: Vec3,
) -> bool {
    let Ok(face) = topo.face(moved) else {
        return false;
    };
    let FaceSurface::Plane { normal, d } = face.surface() else {
        return false;
    };
    // Only line and circle trims have certified boundary extrema.
    let Ok(edges) = remus_topology::explorer::face_edges(topo, moved) else {
        return false;
    };
    if !edges.iter().all(|&e| {
        topo.edge(e).is_ok_and(|e| {
            matches!(
                e.curve(),
                remus_topology::edge::EdgeCurve::Line | remus_topology::edge::EdgeCurve::Circle(_)
            )
        })
    }) {
        return false;
    }
    let Ok(candidate_face) = topo.face(candidate) else {
        return false;
    };
    let origin = Point3::new(0.0, 0.0, 0.0);
    let (projected, radius) = if let FaceSurface::Cylinder(cylinder) = candidate_face.surface() {
        // On a transverse plane a cylinder cuts an ellipse with major radius
        // r / |axis · normal|. The ellipse center travels along a segment as
        // the plane moves. Enclose the whole segment and all its ellipses.
        let axis = cylinder.axis();
        let cosine = axis.dot(*normal);
        if cosine.abs() <= Tolerance::new().angular {
            return false;
        }
        // Fitted trim curves and sewn endpoints may extend beyond their
        // analytic carrier. Include their complete radial extent in the proof.
        let Some(radius) = cylinder_envelope(topo, candidate, cylinder) else {
            return false;
        };
        let center =
            cylinder.origin() + axis * ((*d - normal.dot(cylinder.origin() - origin)) / cosine);
        let travel = axis * (normal.dot(delta) / cosine) - delta;
        (
            center + travel * 0.5,
            radius / cosine.abs() + travel.length() * 0.5,
        )
    } else {
        let Ok(bound) = remus_check::distance::face_bounds::face_bound(topo, candidate) else {
            return false;
        };
        if !bound.prunable {
            return false;
        }
        let half = (bound.aabb.max - bound.aabb.min) * 0.5;
        let center = bound.aabb.min + half;
        (
            center - *normal * (normal.dot(center - origin) - *d),
            half.length() + (delta - *normal * normal.dot(delta)).length(),
        )
    };
    remus_check::distance::point_to_face(topo, projected, moved).is_ok_and(|result| {
        result.is_some_and(|(distance, _)| distance > radius + Tolerance::new().linear)
    })
}

fn cylinder_envelope(
    topo: &Topology,
    face: FaceId,
    cylinder: &remus_math::surfaces::CylindricalSurface,
) -> Option<f64> {
    use remus_topology::edge::EdgeCurve;
    let radial = |p: Point3| (p - cylinder.origin()).cross(cylinder.axis()).length();
    let mut radius = cylinder.radius();
    for id in remus_topology::explorer::face_edges(topo, face).ok()? {
        let edge = topo.edge(id).ok()?;
        for vertex in [edge.start(), edge.end()] {
            radius = radius.max(radial(topo.vertex(vertex).ok()?.point()));
        }
        match edge.curve() {
            EdgeCurve::Line => {}
            EdgeCurve::Circle(c) => radius = radius.max(radial(c.center()) + c.radius()),
            EdgeCurve::NurbsCurve(c) if c.weights().iter().all(|&w| w > 0.0) => {
                for &p in c.control_points() {
                    radius = radius.max(radial(p));
                }
            }
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::Hyperbola(_) => return None,
        }
    }
    radius.is_finite().then_some(radius)
}

/// Whether a planar face translated along its normal provably never reaches
/// `candidate`, because the two stay on opposite sides of a cylinder about
/// an axis parallel to the move. Translation along that axis keeps every
/// point's distance from it, so the proof holds for any distance, and the
/// candidate's full periodic bounding box (which always contains its axis)
/// no longer decides the pair.
///
/// Two separations qualify: a circular face inside the cylinder with the
/// candidate wholly outside it ([`disc_clears_coaxial_shell`]), and a face
/// wholly outside the cylinder with the candidate inside it
/// ([`face_clears_coaxial_core`]). The second also holds with the roles
/// exchanged, for a fixed planar candidate parallel to the move lying
/// outside a cylinder that holds the moved face: distance from the axis does
/// not depend on which of the two travels along it. This is only an
/// exclusion proof; unqualified pairs retain the box check.
pub(super) fn coaxial_sweep_is_radially_disjoint(
    topo: &Topology,
    moved: FaceId,
    candidate: FaceId,
    direction: Vec3,
) -> Result<bool, crate::OperationsError> {
    let FaceSurface::Plane { normal, .. } = topo.face(moved)?.surface() else {
        return Ok(false);
    };
    if normal.cross(direction).length() > Tolerance::new().angular {
        return Ok(false);
    }
    let candidate_parallel = matches!(
        topo.face(candidate)?.surface(),
        FaceSurface::Plane { normal, .. } if normal.cross(direction).length() <= Tolerance::new().angular
    );
    Ok(
        disc_clears_coaxial_shell(topo, moved, candidate, direction)?
            || face_clears_coaxial_core(topo, moved, candidate, direction)?
            || (candidate_parallel && face_clears_coaxial_core(topo, candidate, moved, direction)?),
    )
}

/// A circular planar face stays inside its circular boundary when translated
/// along its axis. Coaxial curved supports outside that cylinder cannot be
/// reached.
fn disc_clears_coaxial_shell(
    topo: &Topology,
    moved: FaceId,
    candidate: FaceId,
    direction: Vec3,
) -> Result<bool, crate::OperationsError> {
    let tol = Tolerance::new();
    let Some(CircularWire {
        center,
        max_reach: radius,
        ..
    }) = circular_wire(topo, topo.face(moved)?.outer_wire(), direction)?
    else {
        return Ok(false);
    };
    let coaxial = |origin: Point3, axis: Vec3| {
        axis.cross(direction).length() <= tol.angular
            && (origin - center).cross(direction).length() <= tol.linear
    };
    let lower_radius = match topo.face(candidate)?.surface() {
        FaceSurface::Cylinder(cylinder) if coaxial(cylinder.origin(), cylinder.axis()) => {
            cylinder.radius()
        }
        FaceSurface::Torus(torus) if coaxial(torus.center(), torus.z_axis()) => {
            torus_strip_lower_radius(topo, candidate, torus)?
                .unwrap_or_else(|| (torus.major_radius() - torus.minor_radius()).max(0.0))
        }
        FaceSurface::Cone(cone) if coaxial(cone.apex(), cone.axis()) => {
            // Project a sound face box onto the cone axis. On either side of
            // the apex radius is |height| tan(angle); an interval crossing
            // the apex supplies no positive lower bound.
            let bounds = crate::measure::face_bounding_box(topo, candidate)?;
            let middle = Point3::new(
                f64::midpoint(bounds.min.x(), bounds.max.x()),
                f64::midpoint(bounds.min.y(), bounds.max.y()),
                f64::midpoint(bounds.min.z(), bounds.max.z()),
            );
            let half = (bounds.max - bounds.min) * 0.5;
            let axis = cone.axis();
            let extent =
                axis.x().abs() * half.x() + axis.y().abs() * half.y() + axis.z().abs() * half.z();
            // `half_angle` is measured from the radial plane, so radius is
            // |height| / tan(angle), not |height| tan(angle).
            ((middle - cone.apex()).dot(axis).abs() - extent).max(0.0) / cone.half_angle().tan()
        }
        // A face parallel to the move whose holes leave the axis clear is
        // bounded on its boundary. A face whose only hole is coaxial and
        // circular is bounded by that hole's rim however its arcs are
        // split; with one hole, no material can nest inside it. Any other
        // planar face keeps the box check.
        FaceSurface::Plane { normal, .. } => {
            if normal.cross(direction).length() > tol.angular {
                return Ok(false);
            }
            let rim = match topo.face(candidate)?.inner_wires() {
                [hole] => circular_wire(topo, *hole, direction)?
                    .filter(|hole| coaxial(hole.center, direction))
                    .map_or(0.0, |hole| hole.min_reach),
                _ => 0.0,
            };
            planar_face_axis_clearance(topo, candidate, center, direction)?
                .unwrap_or(0.0)
                .max(rim)
        }
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => return Ok(false),
    };
    Ok(lower_radius > radius + tol.linear)
}

/// A planar face lying wholly outside a cylinder about an axis parallel to
/// the move cannot reach a candidate wholly inside that cylinder: a boss's
/// rounded rim, its cap or its wall, seen from a block top that wraps the
/// boss. The face's distance from the candidate's axis is bounded below on
/// its boundary, which holds for the whole face only when the axis does not
/// pierce it.
fn face_clears_coaxial_core(
    topo: &Topology,
    moved: FaceId,
    candidate: FaceId,
    direction: Vec3,
) -> Result<bool, crate::OperationsError> {
    let tol = Tolerance::new();
    let tilt = |axis: Vec3| {
        let tilt = axis.cross(direction).length();
        (tilt <= tol.angular).then_some(tilt)
    };
    // An axis through `origin` along the move, the greatest distance from
    // the carrier's own axis of any point of the candidate, and the sine of
    // the angle between the two axes. Each bound holds for every trim of the
    // carrier.
    let core = match topo.face(candidate)?.surface() {
        FaceSurface::Cylinder(cylinder) => {
            tilt(cylinder.axis()).map(|tilt| (cylinder.origin(), cylinder.radius(), tilt))
        }
        FaceSurface::Torus(torus) => tilt(torus.z_axis()).map(|tilt| {
            (
                torus.center(),
                torus.major_radius() + torus.minor_radius(),
                tilt,
            )
        }),
        FaceSurface::Sphere(sphere) => Some((sphere.center(), sphere.radius(), 0.0)),
        // Every point of a circular cap lies within its wire's reach of the
        // centre in 3D, so no tilt allowance is needed.
        FaceSurface::Plane { .. } => {
            circular_wire(topo, topo.face(candidate)?.outer_wire(), direction)?
                .map(|wire| (wire.center, wire.max_reach, 0.0))
        }
        FaceSurface::Cone(_) | FaceSurface::Nurbs(_) => None,
    };
    let Some((origin, reach, tilt)) = core else {
        return Ok(false);
    };
    // A carrier axis tilted from the move drifts from the parallel axis by
    // the sine of the tilt times the axial distance, which no point of the
    // face exceeds its box's farthest corner from `origin`.
    let bounds = crate::measure::face_bounding_box(topo, candidate)?;
    let mut far = 0.0_f64;
    for x in [bounds.min.x(), bounds.max.x()] {
        for y in [bounds.min.y(), bounds.max.y()] {
            for z in [bounds.min.z(), bounds.max.z()] {
                far = far.max((Point3::new(x, y, z) - origin).length());
            }
        }
    }
    let upper_radius = tilt.mul_add(far, reach);
    Ok(planar_face_axis_clearance(topo, moved, origin, direction)?
        .is_some_and(|lower_radius| lower_radius > upper_radius + tol.linear))
}

/// A lower bound on the distance from the axis through `origin` along
/// `direction` to every point of the planar face `face`, whose normal is
/// parallel to `direction`.
///
/// Each line edge contributes its exact distance from the axis's pierce
/// point and each circular edge the distance from that point to its whole
/// circle, which no arc of it can undercut. The minimum bounds the face only
/// when the pierce point lies outside it, which is decided exactly on the
/// lines and arcs. `None` for any other edge curve, an unreadable domain, a
/// pierce point inside the face, or one too close to the boundary to decide.
///
/// The frame is anchored on the axis itself, so the pierce point is the
/// frame origin and every coordinate is measured from it. An arc whose plane
/// is tilted from the frame projects to an ellipse within `r · sin(tilt)` of
/// the circle it is read as, and the result also gives up a relative
/// rounding allowance on the coordinates' scale.
fn planar_face_axis_clearance(
    topo: &Topology,
    face: FaceId,
    origin: Point3,
    direction: Vec3,
) -> Result<Option<f64>, crate::OperationsError> {
    use remus_algo::{PlanarLoopEdge, PlaneFrame, point_in_planar_loop};

    let tol = Tolerance::new();
    let data = topo.face(face)?;
    let frame = PlaneFrame::from_normal_and_point(direction, origin);
    let pierce = frame.project(origin);
    let unit = direction
        .normalize()
        .map_err(|_| crate::OperationsError::InvalidInput {
            reason: "radial clearance requires a nonzero move direction".into(),
        })?;
    let mut lower = f64::INFINITY;
    let mut tilt_allowance = 0.0_f64;
    // Rounding in `p - origin` and in the projection is relative to the
    // world coordinates' magnitude, however small the projected ones are.
    let magnitude = |p: Point3| p.x().abs().max(p.y().abs()).max(p.z().abs());
    let mut scale = magnitude(origin);
    let mut loops = Vec::with_capacity(1 + data.inner_wires().len());
    for wire in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied()) {
        let mut edges = Vec::new();
        for oriented in topo.wire(wire)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            scale = scale.max(magnitude(start)).max(magnitude(end));
            if let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve() {
                scale = scale.max(magnitude(circle.center()) + circle.radius());
            }
            let Ok((first, last)) = edge.strict_domain() else {
                return Ok(None);
            };
            let at = |t: f64| {
                let t = if oriented.is_forward() { t } else { 1.0 - t };
                edge.curve()
                    .evaluate_with_endpoints((last - first).mul_add(t, first), start, end)
            };
            let Some(projected) = PlanarLoopEdge::project(
                edge.curve(),
                at,
                edge.start() == edge.end(),
                &frame,
                tol.angular,
            ) else {
                return Ok(None);
            };
            let distance = match projected {
                PlanarLoopEdge::Segment(a, b) => {
                    let (along, offset) = (b - a, pierce - a);
                    let span = along.length_squared();
                    let t = if span > 0.0 {
                        (offset.dot(along) / span).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    (offset - along * t).length()
                }
                PlanarLoopEdge::Arc { center, radius, .. }
                | PlanarLoopEdge::Circle { center, radius, .. } => {
                    if let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve() {
                        let tilt = circle
                            .normal()
                            .normalize()
                            .map_or(1.0, |normal| normal.cross(unit).length());
                        tilt_allowance = tilt_allowance.max(radius * tilt);
                    }
                    ((pierce - center).length() - radius).abs()
                }
            };
            lower = lower.min(distance);
            edges.push(projected);
        }
        loops.push(edges);
    }
    let eps = tol.linear * 10.0;
    let mut decisions = loops
        .iter()
        .map(|edges| point_in_planar_loop(pierce, edges, eps));
    let Some(Some(in_outer)) = decisions.next() else {
        return Ok(None);
    };
    // Inside the outer wire, the axis misses the face only inside exactly
    // one hole. Holes that nest, as an imported face may carry, leave
    // material around the axis that their nearest rim does not bound.
    let mut holes = 0_usize;
    for decision in decisions {
        let Some(inside) = decision else {
            return Ok(None);
        };
        holes += usize::from(inside);
    }
    if in_outer && holes > 1 {
        return Ok(None);
    }
    let in_hole = holes == 1;
    // Each coordinate and the distance read from it carry a few rounding
    // steps relative to the world `scale`; 1e-12 of it is orders of
    // magnitude above that and still far below the modeling tolerance at
    // part scale.
    let lower = 1e-12_f64.mul_add(-scale, lower - tilt_allowance);
    Ok((!in_outer || in_hole).then_some(lower))
}

/// A wire of arcs of one circle, up to tolerance, about the first arc's
/// centre.
struct CircularWire {
    center: Point3,
    /// No point of any arc lies closer to `center` than this.
    min_reach: f64,
    /// No point of any arc lies farther from `center` than this.
    max_reach: f64,
}

/// A wire made only of arcs of one circle whose normal is parallel to
/// `direction`. A closed wire of such arcs runs the full circle.
///
/// Arcs that agree with the first only within tolerance may each reach a
/// little beyond it, so the reach is bounded over every arc: a point of an
/// arc lies within its radius of its own centre, which lies within the
/// centres' offset of the first.
fn circular_wire(
    topo: &Topology,
    wire: remus_topology::wire::WireId,
    direction: Vec3,
) -> Result<Option<CircularWire>, crate::OperationsError> {
    let tol = Tolerance::new();
    let mut boundary: Option<CircularWire> = None;
    for oriented in topo.wire(wire)?.edges() {
        let remus_topology::edge::EdgeCurve::Circle(circle) = topo.edge(oriented.edge())?.curve()
        else {
            return Ok(None);
        };
        if circle.normal().cross(direction).length() > tol.angular {
            return Ok(None);
        }
        let Some(first) = boundary.as_mut() else {
            boundary = Some(CircularWire {
                center: circle.center(),
                min_reach: circle.radius(),
                max_reach: circle.radius(),
            });
            continue;
        };
        let offset = (circle.center() - first.center).length();
        if offset > tol.linear || (circle.radius() - first.max_reach).abs() > 2.0 * tol.linear {
            return Ok(None);
        }
        first.min_reach = first.min_reach.min(circle.radius() - offset);
        first.max_reach = first.max_reach.max(circle.radius() + offset);
    }
    Ok(boundary)
}

/// Radial bound of a rectangular torus strip from its authoritative meridian
/// seam. Coaxial circular rims, whole or split into arcs, close the strip; a
/// duplicated seam bounds its minor-angle interval. Other trims keep the
/// full-torus bound.
fn torus_strip_lower_radius(
    topo: &Topology,
    face: FaceId,
    torus: &remus_math::surfaces::ToroidalSurface,
) -> Result<Option<f64>, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;
    let tol = Tolerance::new();
    let face = topo.face(face)?;
    let uses = topo.wire(face.outer_wire())?.edges();
    if !face.inner_wires().is_empty() || uses.len() < 4 {
        return Ok(None);
    }
    let axis = torus.z_axis();
    let mut rims = 0;
    let mut seam = None;
    let mut lower = f64::INFINITY;
    for oriented in uses {
        let edge = topo.edge(oriented.edge())?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return Ok(None);
        };
        let delta = circle.center() - torus.center();
        let axial = delta.dot(axis);
        let radial = delta - axis * axial;
        if circle.normal().cross(axis).length() <= tol.angular && radial.length() <= tol.linear {
            rims += 1;
            lower = lower.min(circle.radius());
            continue;
        }
        if axial.abs() > tol.linear
            || (radial.length() - torus.major_radius()).abs() > tol.linear
            || circle.normal().dot(axis).abs() > tol.angular
            || circle.normal().dot(radial).abs() > tol.linear
            || (circle.radius() - torus.minor_radius()).abs() > tol.linear
        {
            return Ok(None);
        }
        if let Some(previous) = seam {
            if previous != oriented.edge() {
                return Ok(None);
            }
        } else {
            seam = Some(oriented.edge());
        }
        let Ok((first, last)) = edge.strict_domain() else {
            return Ok(None);
        };
        let unit = radial * torus.major_radius().recip();
        let a = circle.radius() * circle.u_axis().dot(unit);
        let b = circle.radius() * circle.v_axis().dot(unit);
        let lo = first.min(last);
        let hi = first.max(last);
        let at = |t: f64| torus.major_radius() + a * t.cos() + b * t.sin();
        lower = lower.min(at(lo)).min(at(hi));
        let minimum = b.atan2(a) + std::f64::consts::PI;
        let turn = ((lo - minimum) / std::f64::consts::TAU).ceil();
        let critical = minimum + turn * std::f64::consts::TAU;
        if critical <= hi {
            lower = lower.min(torus.major_radius() - a.hypot(b));
        }
    }
    Ok((rims >= 2 && seam.is_some()).then_some(lower.max(0.0)))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::{circular_wire, coaxial_sweep_is_radially_disjoint, planar_face_axis_clearance};
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_box, make_cone, make_cylinder, make_sphere};
    use crate::transform::transform_solid;
    use remus_math::{
        mat::Mat4,
        vec::{Point3, Vec3},
    };
    use remus_topology::{
        Topology,
        explorer::solid_faces,
        face::{FaceId, FaceSurface},
        solid::SolidId,
    };

    const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    /// The +z cap of a radius-12.5 cylinder: the swept disc.
    fn disc(topo: &mut Topology) -> FaceId {
        let cylinder = make_cylinder(topo, 12.5, 5.0).unwrap();
        planar_face(topo, cylinder, 5.0)
    }

    fn planar_face(topo: &Topology, solid: SolidId, z: f64) -> FaceId {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                topo.face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.99)
                    && remus_topology::explorer::face_vertices(topo, face)
                        .unwrap()
                        .iter()
                        .all(|&vertex| (topo.vertex(vertex).unwrap().point().z() - z).abs() < 1e-9)
            })
            .expect("upward planar face")
    }

    fn cone_face(topo: &mut Topology, bottom: f64, top: f64, height: f64) -> FaceId {
        let cone = make_cone(topo, bottom, top, height).unwrap();
        solid_faces(topo, cone)
            .unwrap()
            .into_iter()
            .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_)))
            .expect("conical face")
    }

    /// The top annulus of a radius-20 cylinder bored through at `hole`.
    fn annulus(topo: &mut Topology, hole: f64) -> FaceId {
        let blank = make_cylinder(topo, 20.0, 10.0).unwrap();
        let tool = make_cylinder(topo, hole, 30.0).unwrap();
        transform_solid(topo, tool, &Mat4::translation(0.0, 0.0, -10.0)).unwrap();
        let tube = boolean(topo, BooleanOp::Cut, blank, tool).unwrap();
        planar_face(topo, tube, 10.0)
    }

    #[test]
    fn swept_annulus_excludes_a_cylinder_in_the_opening_but_keeps_contact() {
        let mut topo = Topology::new();
        let face = annulus(&mut topo, 8.0);
        for (offset, clear) in [(0.0, true), (6.0, false), (7.0, false)] {
            let cylinder = make_cylinder(&mut topo, 2.0, 15.0).unwrap();
            transform_solid(&mut topo, cylinder, &Mat4::translation(offset, 0.0, 0.0)).unwrap();
            let wall = solid_faces(&topo, cylinder)
                .unwrap()
                .into_iter()
                .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
                .unwrap();
            for sign in [-1.0, 1.0] {
                assert_eq!(
                    super::outside_planar_sweep(&topo, face, wall, UP * sign),
                    clear
                );
            }
        }
    }

    #[test]
    fn inward_tapering_ledge_cannot_sweep_through_a_nonadjacent_post() {
        use remus_math::vec::Point3;
        use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
        let mut topo = Topology::new();
        // At z=0 the rim clears the post by 4. At z=5 the cone closes
        // the opening to radius 5 and intersects the radius-6 post.
        let section = [
            (0., -4.),
            (20., -4.),
            (20., 0.),
            (10., 0.),
            (12., -2.),
            (4., -2.),
            (4., 1.),
            (6., 1.),
            (6., 10.),
            (0., 10.),
        ]
        .map(|(r, z)| Point3::new(r, 0., z));
        let wire = make_polygon_wire(&mut topo, &section, 1e-7).unwrap();
        let profile = make_planar_face_from_wire(&mut topo, wire).unwrap();
        let source = crate::revolve::revolve(
            &mut topo,
            profile,
            Point3::new(0., 0., 0.),
            UP,
            std::f64::consts::TAU,
        )
        .unwrap();
        let ledge = planar_face(&topo, source, 0.);
        let err = crate::push_pull::move_faces(&mut topo, source, &[ledge], 5.).unwrap_err();
        assert!(
            err.to_string()
                .contains("swept face reaches nonadjacent face"),
            "{err}"
        );
    }

    /// Review fixture (Sol, #1905): an annular cap whose opening has radius
    /// 10 with a 45° inner chamfer down to radius 9, beside a fixed tab held
    /// by a central post. The cap's own planar sweep clears the tab, but the
    /// chamfer travels with the cap and its moved cone reaches the tab near
    /// (9.195, 0.3, 1.6). The move must refuse and leave the input untouched.
    #[test]
    fn cap_chamfer_carried_by_a_translation_cannot_sweep_through_a_fixed_tab() {
        use remus_math::vec::Point3;
        use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
        let assert_valid_and_watertight = |topo: &Topology, solid: SolidId| {
            let report = crate::validate::validate_solid(topo, solid).unwrap();
            assert!(report.is_valid(), "{:?}", report.issues);
            let mesh =
                crate::tessellate::tessellate_solid_with_tolerance(topo, solid, 0.01, 0.2).unwrap();
            assert!(crate::tessellate::is_watertight(&mesh));
        };
        let mut topo = Topology::new();
        let section = [
            (0., -5.),
            (20., -5.),
            (20., 0.),
            (10., 0.),
            (9., -1.),
            (9., -3.),
            (4., -3.),
            (4., 3.),
            (0., 3.),
        ]
        .map(|(r, z)| Point3::new(r, 0., z));
        let wire = make_polygon_wire(&mut topo, &section, 1e-7).unwrap();
        let profile = make_planar_face_from_wire(&mut topo, wire).unwrap();
        let ring = crate::revolve::revolve(
            &mut topo,
            profile,
            Point3::new(0., 0., 0.),
            UP,
            std::f64::consts::TAU,
        )
        .unwrap();
        let tab = make_box(&mut topo, 6.0, 0.6, 0.6).unwrap();
        transform_solid(&mut topo, tab, &Mat4::translation(3.5, -0.3, 1.35)).unwrap();
        let body = boolean(&mut topo, BooleanOp::Fuse, ring, tab).unwrap();
        assert_valid_and_watertight(&topo, body);
        let cap = planar_face(&topo, body, 0.);
        assert!(
            solid_faces(&topo, body)
                .unwrap()
                .iter()
                .any(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_))),
            "the opening keeps its chamfer"
        );
        let before = crate::measure::solid_volume(&topo, body, 0.01).unwrap();
        let faces_before = solid_faces(&topo, body).unwrap();
        let err = crate::push_pull::move_faces(&mut topo, body, &[cap], 2.4).unwrap_err();
        assert!(
            err.to_string()
                .contains("swept face reaches nonadjacent face"),
            "{err}"
        );
        assert_eq!(solid_faces(&topo, body).unwrap(), faces_before);
        let after = crate::measure::solid_volume(&topo, body, 0.01).unwrap();
        assert_eq!(before.to_bits(), after.to_bits());
        assert_valid_and_watertight(&topo, body);
        // Moves the tab does not obstruct still succeed: down, and up short
        // of it (the chamfer's foot reaches z = 1.35 only past +2.35).
        for distance in [-1.0, 1.0] {
            let mut scratch = topo.clone();
            let moved = crate::push_pull::move_faces(&mut scratch, body, &[cap], distance).unwrap();
            assert_valid_and_watertight(&scratch, moved);
        }
    }

    #[test]
    fn cone_radius_uses_the_angle_from_the_radial_plane() {
        let mut topo = Topology::new();
        let moved = disc(&mut topo);
        // Half-angle atan(15/20) ≈ 36.9°: radii 16..20 clear the disc. Taking
        // |height| tan(angle) gave 9 and refused the sweep.
        let shallow = cone_face(&mut topo, 20.0, 16.0, 3.0);
        assert!(coaxial_sweep_is_radially_disjoint(&topo, moved, shallow, UP).unwrap());
        // Half-angle atan(70/14) ≈ 78.7°: radii 10..14 overlap the disc.
        // Taking |height| tan(angle) gave 250 and exempted a real contact.
        let steep = cone_face(&mut topo, 14.0, 10.0, 20.0);
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, moved, steep, UP).unwrap());
    }

    #[test]
    fn coaxial_hole_bounds_a_planar_shoulder() {
        let mut topo = Topology::new();
        let moved = disc(&mut topo);
        let wide = annulus(&mut topo, 16.0);
        assert!(coaxial_sweep_is_radially_disjoint(&topo, moved, wide, UP).unwrap());
        // A hole of the disc's own radius is where the disc meets the face.
        let flush = annulus(&mut topo, 12.5);
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, moved, flush, UP).unwrap());
        // An off-axis hole is bounded by its rim's nearest approach to the
        // disc's axis: 15 clears the disc, 12 does not.
        for (shift, clear) in [(1.0, true), (4.0, false)] {
            let blank = make_cylinder(&mut topo, 24.0, 10.0).unwrap();
            let tool = make_cylinder(&mut topo, 16.0, 30.0).unwrap();
            transform_solid(&mut topo, tool, &Mat4::translation(shift, 0.0, -10.0)).unwrap();
            let tube = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
            let offset = planar_face(&topo, tube, 10.0);
            assert_eq!(
                coaxial_sweep_is_radially_disjoint(&topo, moved, offset, UP).unwrap(),
                clear,
                "hole shifted by {shift}"
            );
        }
    }

    fn face_on(topo: &Topology, solid: SolidId, kind: fn(&FaceSurface) -> bool) -> FaceId {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| kind(topo.face(face).unwrap().surface()))
            .expect("face of the requested carrier")
    }

    /// An R6 boss through the corner of a 30 x 18 x 10 block: the block top
    /// wraps the boss on a quarter arc of radius 6 about its axis.
    fn block_top_around_boss(topo: &mut Topology) -> FaceId {
        let block = make_box(topo, 30.0, 18.0, 10.0).unwrap();
        let boss = make_cylinder(topo, 6.0, 20.0).unwrap();
        let fused = boolean(topo, BooleanOp::Fuse, boss, block).unwrap();
        planar_face(topo, fused, 10.0)
    }

    #[test]
    fn face_outside_a_coaxial_core_is_bounded_on_its_boundary() {
        let mut topo = Topology::new();
        let top = block_top_around_boss(&mut topo);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let clearance = planar_face_axis_clearance(&topo, top, origin, UP)
            .unwrap()
            .expect("the boss axis does not pierce the block top");
        assert!((clearance - 6.0).abs() < 1e-9, "{clearance}");
        // An axis through the face proves nothing about its distance.
        let inside = Point3::new(15.0, 9.0, 0.0);
        assert!(
            planar_face_axis_clearance(&topo, top, inside, UP)
                .unwrap()
                .is_none()
        );
        // An axis through a hole is bounded by the hole's own rim.
        let ring = annulus(&mut topo, 16.0);
        let clearance = planar_face_axis_clearance(&topo, ring, origin, UP)
            .unwrap()
            .expect("the axis runs through the hole");
        assert!((clearance - 16.0).abs() < 1e-9, "{clearance}");
    }

    #[test]
    fn coaxial_core_inside_the_face_clearance_is_unreachable() {
        let mut topo = Topology::new();
        let ring = annulus(&mut topo, 16.0);
        let core = make_cylinder(&mut topo, 12.5, 30.0).unwrap();
        let wall = face_on(&topo, core, |surface| {
            matches!(surface, FaceSurface::Cylinder(_))
        });
        let cap = planar_face(&topo, core, 30.0);
        assert!(coaxial_sweep_is_radially_disjoint(&topo, ring, wall, UP).unwrap());
        assert!(coaxial_sweep_is_radially_disjoint(&topo, ring, cap, UP).unwrap());
        // A core as wide as the clearance meets the face at its rim.
        let flush = make_cylinder(&mut topo, 16.0, 30.0).unwrap();
        let flush_wall = face_on(&topo, flush, |surface| {
            matches!(surface, FaceSurface::Cylinder(_))
        });
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, ring, flush_wall, UP).unwrap());
        // So does one about another axis, whatever its radius.
        let shifted = make_cylinder(&mut topo, 2.0, 30.0).unwrap();
        transform_solid(&mut topo, shifted, &Mat4::translation(17.0, 0.0, 0.0)).unwrap();
        let shifted_wall = face_on(&topo, shifted, |surface| {
            matches!(surface, FaceSurface::Cylinder(_))
        });
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, ring, shifted_wall, UP).unwrap());
    }

    #[test]
    fn swept_core_cannot_reach_a_parallel_face_outside_it() {
        // A cap descending through a block top that wraps its boss: the
        // moved face is the core and the fixed block top the outer face.
        let mut topo = Topology::new();
        let top = block_top_around_boss(&mut topo);
        let narrow = make_cylinder(&mut topo, 4.0, 40.0).unwrap();
        let narrow_cap = planar_face(&topo, narrow, 40.0);
        assert!(coaxial_sweep_is_radially_disjoint(&topo, narrow_cap, top, UP).unwrap());
        // A cap as wide as the block top's clearance meets it at the arc.
        let wide = make_cylinder(&mut topo, 6.0, 40.0).unwrap();
        let wide_cap = planar_face(&topo, wide, 40.0);
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, wide_cap, top, UP).unwrap());
        // A candidate across the move is not a parallel face; its sweep
        // contact is not decided by the axis distance of its plane.
        let side = solid_faces(&topo, narrow)
            .unwrap()
            .into_iter()
            .find(|&face| {
                matches!(
                    topo.face(face).unwrap().surface(),
                    FaceSurface::Plane { .. }
                ) && topo
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| n.z() < -0.99)
            })
            .expect("bottom cap");
        let block = make_box(&mut topo, 30.0, 18.0, 10.0).unwrap();
        let wall = solid_faces(&topo, block)
            .unwrap()
            .into_iter()
            .find(|&face| {
                topo.face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| n.x() > 0.99)
            })
            .expect("x-max wall");
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, side, wall, UP).unwrap());
    }

    #[test]
    fn tilted_core_axis_drifts_by_its_axial_reach() {
        // A unit cylinder 2e9 long whose axis leans 5e-13 rad off the move,
        // inside the angular tolerance: its far end has drifted 1e-3 off
        // the parallel axis, so a plate 2e-4 outside its radius is reached.
        let mut topo = Topology::new();
        let core = make_cylinder(&mut topo, 1.0, 2.0e9).unwrap();
        transform_solid(&mut topo, core, &Mat4::rotation_y(5.0e-13)).unwrap();
        transform_solid(&mut topo, core, &Mat4::translation(0.0, 0.0, 10.0)).unwrap();
        let wall = face_on(&topo, core, |surface| {
            matches!(surface, FaceSurface::Cylinder(_))
        });
        let plate = make_box(&mut topo, 0.9998, 0.2, 1.0).unwrap();
        transform_solid(&mut topo, plate, &Mat4::translation(1.0002, -0.1, -1.0)).unwrap();
        let top = planar_face(&topo, plate, 0.0);
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, top, wall, UP).unwrap());
    }

    #[test]
    fn split_coaxial_hole_still_bounds_a_planar_shoulder() {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        // The R16 hole of a bore floor's shoulder, split into two exact
        // semicircles: the axis lies on their chord, but one coaxial hole
        // still keeps every point of the face 16 from it.
        let mut topo = Topology::new();
        let moved = disc(&mut topo);
        let ring = annulus(&mut topo, 16.0);
        let circle = Circle3D::new(Point3::new(0.0, 0.0, 10.0), UP, 16.0).unwrap();
        let east = topo.add_vertex(Vertex::new(Point3::new(16.0, 0.0, 10.0), 1e-7));
        let west = topo.add_vertex(Vertex::new(Point3::new(-16.0, 0.0, 10.0), 1e-7));
        let mut arc = |from, to, span: (f64, f64)| {
            let mut edge = Edge::new(from, to, EdgeCurve::Circle(circle.clone()));
            edge.set_trim(Some(span));
            topo.add_edge(edge)
        };
        let north = arc(east, west, (0.0, std::f64::consts::PI));
        let south = arc(west, east, (std::f64::consts::PI, std::f64::consts::TAU));
        // A hole winds against the face's outer wire.
        let hole = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(south, false),
                    OrientedEdge::new(north, false),
                ],
                true,
            )
            .unwrap(),
        );
        let outer = topo.face(ring).unwrap().outer_wire();
        topo.set_face_boundary_wires(ring, outer, vec![hole])
            .unwrap();
        assert!(coaxial_sweep_is_radially_disjoint(&topo, moved, ring, UP).unwrap());
    }

    #[test]
    fn nested_holes_around_the_axis_prove_nothing() {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        // An R20 face with an R10 hole and an R5 wire inside that hole: by
        // parity the R5 disc is material, and an R2 cap on the axis meets
        // it, whichever of the two travels.
        let mut topo = Topology::new();
        let ring = annulus(&mut topo, 10.0);
        let centre = Point3::new(0.0, 0.0, 10.0);
        let seam = topo.add_vertex(Vertex::new(Point3::new(5.0, 0.0, 10.0), 1e-7));
        let mut island = Edge::new(
            seam,
            seam,
            EdgeCurve::Circle(Circle3D::new(centre, UP, 5.0).unwrap()),
        );
        island.set_trim(Some((0.0, std::f64::consts::TAU)));
        let island = topo.add_edge(island);
        let island = topo.add_wire(Wire::new(vec![OrientedEdge::new(island, true)], true).unwrap());
        let face = topo.face(ring).unwrap();
        let (outer, mut inner) = (face.outer_wire(), face.inner_wires().to_vec());
        inner.push(island);
        topo.set_face_boundary_wires(ring, outer, inner).unwrap();
        let cap = make_cylinder(&mut topo, 2.0, 30.0).unwrap();
        let cap = planar_face(&topo, cap, 30.0);
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, cap, ring, UP).unwrap());
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, ring, cap, UP).unwrap());
    }

    #[test]
    fn circular_wire_reaches_as_far_as_its_widest_arc() {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        // Two arcs within tolerance of one unit circle: the second's centre
        // sits 7.5e-8 off and its radius 6.75e-8 over, so it reaches
        // 1.425e-7 beyond the first arc's radius.
        let mut topo = Topology::new();
        let first = Circle3D::new(Point3::new(0.0, 0.0, 0.0), UP, 1.0).unwrap();
        let second = Circle3D::new(Point3::new(7.5e-8, 0.0, 0.0), UP, 1.0 + 6.75e-8).unwrap();
        let top = topo.add_vertex(Vertex::new(Point3::new(-0.9, 0.435_889_894, 0.0), 1e-7));
        let bottom = topo.add_vertex(Vertex::new(Point3::new(-0.9, -0.435_889_894, 0.0), 1e-7));
        let left = topo.add_edge(Edge::new(top, bottom, EdgeCurve::Circle(first)));
        let right = topo.add_edge(Edge::new(bottom, top, EdgeCurve::Circle(second)));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(left, true),
                    OrientedEdge::new(right, true),
                ],
                true,
            )
            .unwrap(),
        );
        let bound = circular_wire(&topo, wire, UP).unwrap().expect("one circle");
        assert!(bound.max_reach >= 1.0 + 1.42e-7, "{}", bound.max_reach);
        assert!(bound.min_reach <= 1.0, "{}", bound.min_reach);
    }

    #[test]
    fn distant_face_vertices_do_not_round_the_clearance_up() {
        // A plate from 1.0000025 out to 2^35 beside a sphere of radius
        // 1.000003 on the axis: the sweep reaches the sphere, whatever
        // vertex the plate's wire starts at.
        let mut topo = Topology::new();
        let near = 1.000_002_5;
        let plate = make_box(&mut topo, 2.0_f64.powi(35) - near, 2.0, 1.0).unwrap();
        transform_solid(&mut topo, plate, &Mat4::translation(near, -1.0, -1.0)).unwrap();
        let top = planar_face(&topo, plate, 0.0);
        let ball = make_sphere(&mut topo, 1.000_003, 16).unwrap();
        transform_solid(&mut topo, ball, &Mat4::translation(0.0, 0.0, 5.0)).unwrap();
        let shell = face_on(&topo, ball, |surface| {
            matches!(surface, FaceSurface::Sphere(_))
        });
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, top, shell, UP).unwrap());
    }

    #[test]
    fn far_axial_coordinates_keep_their_rounding_allowance() {
        // A plate moving along (0, 1, 1) passes 1.0000025 from the axis of
        // a sphere of radius 1.000003 centred 2^33 out along that axis: the
        // sweep reaches it. The large axial coordinates cancel in the
        // projection, so the allowance must follow their scale.
        let mut topo = Topology::new();
        let plate = make_box(&mut topo, 2.0, 2.0, 1.0).unwrap();
        transform_solid(
            &mut topo,
            plate,
            &Mat4::translation(1.000_002_5, -1.0, -1.0),
        )
        .unwrap();
        transform_solid(
            &mut topo,
            plate,
            &Mat4::rotation_x(-std::f64::consts::FRAC_PI_4),
        )
        .unwrap();
        let diagonal = Vec3::new(0.0, 1.0, 1.0).normalize().unwrap();
        let top = solid_faces(&topo, plate)
            .unwrap()
            .into_iter()
            .find(|&face| {
                topo.face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.dot(diagonal) > 1.0 - 1e-12)
            })
            .expect("plate face along the diagonal");
        let direction = topo.face(top).unwrap().effective_plane_normal().unwrap();
        let far = 2.0_f64.powi(33);
        let ball = make_sphere(&mut topo, 1.000_003, 16).unwrap();
        transform_solid(&mut topo, ball, &Mat4::translation(0.0, far, far)).unwrap();
        let shell = face_on(&topo, ball, |surface| {
            matches!(surface, FaceSurface::Sphere(_))
        });
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, top, shell, direction).unwrap());
    }
}
