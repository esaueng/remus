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
    let wire = topo.wire(topo.face(moved)?.outer_wire())?;
    let mut boundary: Option<(Point3, f64)> = None;
    for oriented in wire.edges() {
        let remus_topology::edge::EdgeCurve::Circle(circle) = topo.edge(oriented.edge())?.curve()
        else {
            return Ok(false);
        };
        if circle.normal().cross(direction).length() > tol.angular {
            return Ok(false);
        }
        if let Some((center, radius)) = boundary {
            if (circle.center() - center).length() > tol.linear
                || (circle.radius() - radius).abs() > tol.linear
            {
                return Ok(false);
            }
        } else {
            boundary = Some((circle.center(), circle.radius()));
        }
    }
    let Some((center, radius)) = boundary else {
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
            ((middle - cone.apex()).dot(axis).abs() - extent).max(0.0) * cone.half_angle().tan()
        }
        FaceSurface::Plane { .. }
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => return Ok(false),
    };
    Ok(lower_radius > radius + tol.linear)
}

/// Radial bound of a rectangular torus strip from its authoritative meridian
/// seam. The two circular rims close the strip; a duplicated seam bounds its
/// minor-angle interval. Other trims keep the full-torus bound.
fn torus_strip_lower_radius(
    topo: &Topology,
    face: FaceId,
    torus: &remus_math::surfaces::ToroidalSurface,
) -> Result<Option<f64>, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;
    let tol = Tolerance::new();
    let face = topo.face(face)?;
    let uses = topo.wire(face.outer_wire())?.edges();
    if !face.inner_wires().is_empty() || uses.len() != 4 {
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
    Ok((rims == 2 && seam.is_some()).then_some(lower.max(0.0)))
}
