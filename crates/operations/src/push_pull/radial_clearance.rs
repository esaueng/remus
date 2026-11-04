//! Conservative radial separation proofs for periodic face sweeps.

use remus_math::{
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    face::{FaceId, FaceSurface},
};

/// A circular planar face stays inside its circular boundary when translated
/// along its axis. Coaxial curved supports outside that cylinder cannot be
/// reached, even though their full periodic bounding boxes include the axis.
/// This is only an exclusion proof; unqualified pairs retain the box check.
pub(super) fn coaxial_sweep_is_radially_disjoint(
    topo: &Topology,
    moved: FaceId,
    candidate: FaceId,
    direction: Vec3,
) -> Result<bool, crate::OperationsError> {
    let tol = Tolerance::new();
    let FaceSurface::Plane { normal, .. } = topo.face(moved)?.surface() else {
        return Ok(false);
    };
    if normal.cross(direction).length() > tol.angular {
        return Ok(false);
    }
    let Some((center, radius)) = circular_wire(topo, topo.face(moved)?.outer_wire(), direction)?
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
        FaceSurface::Plane { .. } => {
            // A coaxial circular hole removes the open disc of its radius
            // from a planar face, so no point of the face lies closer to the
            // axis. Faces without such a hole keep the box check.
            let mut lower = 0.0_f64;
            for &inner in topo.face(candidate)?.inner_wires() {
                if let Some((origin, hole)) = circular_wire(topo, inner, direction)?
                    && coaxial(origin, direction)
                {
                    lower = lower.max(hole);
                }
            }
            lower
        }
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => return Ok(false),
    };
    Ok(lower_radius > radius + tol.linear)
}

/// Center and radius of a wire made only of arcs of one circle whose normal
/// is parallel to `direction`. A closed wire of such arcs runs the full circle.
fn circular_wire(
    topo: &Topology,
    wire: remus_topology::wire::WireId,
    direction: Vec3,
) -> Result<Option<(Point3, f64)>, crate::OperationsError> {
    let tol = Tolerance::new();
    let mut boundary: Option<(Point3, f64)> = None;
    for oriented in topo.wire(wire)?.edges() {
        let remus_topology::edge::EdgeCurve::Circle(circle) = topo.edge(oriented.edge())?.curve()
        else {
            return Ok(None);
        };
        if circle.normal().cross(direction).length() > tol.angular {
            return Ok(None);
        }
        if let Some((center, radius)) = boundary {
            if (circle.center() - center).length() > tol.linear
                || (circle.radius() - radius).abs() > tol.linear
            {
                return Ok(None);
            }
        } else {
            boundary = Some((circle.center(), circle.radius()));
        }
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

    use super::coaxial_sweep_is_radially_disjoint;
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_cone, make_cylinder};
    use crate::transform::transform_solid;
    use remus_math::{mat::Mat4, vec::Vec3};
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
        // An off-axis hole proves nothing about the disc's path.
        let blank = make_cylinder(&mut topo, 20.0, 10.0).unwrap();
        let tool = make_cylinder(&mut topo, 16.0, 30.0).unwrap();
        transform_solid(&mut topo, tool, &Mat4::translation(1.0, 0.0, -10.0)).unwrap();
        let tube = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
        let offset = planar_face(&topo, tube, 10.0);
        assert!(!coaxial_sweep_is_radially_disjoint(&topo, moved, offset, UP).unwrap());
    }
}
