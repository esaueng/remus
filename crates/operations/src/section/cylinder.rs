//! Sections admitted only by authoritative finite cylindrical boundaries.

use std::f64::consts::TAU;

use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::Face;

use crate::OperationsError;

fn unsupported(reason: &str) -> OperationsError {
    OperationsError::Unsupported {
        operation: "section",
        reason: reason.into(),
    }
}

/// Recognize a periodic band bounded by two full circular rims and a paired
/// axial seam. Closed endpoint identity alone does not certify a full turn:
/// each authoritative circle span, carrier, endpoint and wire use is checked.
fn axial_band(
    topo: &Topology,
    face: &Face,
    cylinder: &CylindricalSurface,
    tol: Tolerance,
) -> Result<(f64, f64), OperationsError> {
    let refuse =
        || unsupported("cylindrical section requires a complete two-rim band without holes");
    if !face.inner_wires().is_empty()
        || !cylinder.radius().is_finite()
        || cylinder.radius() <= 0.0
        || cylinder.origin().0.iter().any(|x| !x.is_finite())
        || cylinder.axis().0.iter().any(|x| !x.is_finite())
        || (cylinder.axis().length_squared() - 1.0).abs() > tol.angular
    {
        return Err(refuse());
    }
    let wire = topo.wire(face.outer_wire())?;
    let uses = wire.edges();
    if uses.len() != 4 || !wire.is_closed() {
        return Err(refuse());
    }
    let mut rims = Vec::new();
    let mut seam_uses = Vec::new();
    let mut winding = 0.0;
    for (i, oriented) in uses.iter().enumerate() {
        let edge = topo.edge(oriented.edge())?;
        let next = uses[(i + 1) % uses.len()];
        if oriented.oriented_end(edge) != next.oriented_start(topo.edge(next.edge())?) {
            return Err(refuse());
        }
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let (lo, hi) = crate::authoritative_edge_domain(edge, "cylindrical section boundary")?;
        if (edge.curve().evaluate_with_endpoints(lo, start, end) - start).length() > tol.linear
            || (edge.curve().evaluate_with_endpoints(hi, start, end) - end).length() > tol.linear
        {
            return Err(refuse());
        }
        match edge.curve() {
            EdgeCurve::Circle(circle) => {
                if edge.start() != edge.end()
                    || ((hi - lo).abs() - TAU).abs() > tol.angular
                    || !circle.radius().is_finite()
                    || circle.center().0.iter().any(|x| !x.is_finite())
                    || circle.normal().0.iter().any(|x| !x.is_finite())
                    || (circle.radius() - cylinder.radius()).abs() > tol.linear
                    || circle.normal().cross(cylinder.axis()).length() > tol.angular
                {
                    return Err(refuse());
                }
                let delta = circle.center() - cylinder.origin();
                let level = delta.dot(cylinder.axis());
                if !level.is_finite() || (delta - cylinder.axis() * level).length() > tol.linear {
                    return Err(refuse());
                }
                rims.push((edge.start(), level));
                winding += (hi - lo).signum()
                    * circle.normal().dot(cylinder.axis()).signum()
                    * if oriented.is_forward() { 1.0 } else { -1.0 };
            }
            EdgeCurve::Line => {
                if edge.start() == edge.end()
                    || (end - start).cross(cylinder.axis()).length() > tol.linear
                {
                    return Err(refuse());
                }
                seam_uses.push(*oriented);
            }
            EdgeCurve::Ellipse(_)
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::Hyperbola(_) => return Err(refuse()),
        }
    }
    if rims.len() != 2
        || seam_uses.len() != 2
        || winding.abs() > tol.angular
        || seam_uses[0].edge() != seam_uses[1].edge()
        || seam_uses[0].is_forward() == seam_uses[1].is_forward()
    {
        return Err(refuse());
    }
    let seam = topo.edge(seam_uses[0].edge())?;
    if !rims.iter().any(|&(vertex, _)| vertex == seam.start())
        || !rims.iter().any(|&(vertex, _)| vertex == seam.end())
    {
        return Err(refuse());
    }
    let low = rims[0].1.min(rims[1].1);
    let high = rims[0].1.max(rims[1].1);
    if high - low <= tol.linear {
        return Err(refuse());
    }
    Ok((low, high))
}

/// Append a contained plane/cylinder loop, or prove a finite-band miss.
/// The complete carrier's axial sinusoid is bounded analytically; a sampled
/// point or a midpoint cannot admit a loop crossing either authoritative rim.
#[allow(clippy::too_many_arguments, clippy::cast_precision_loss)]
pub(super) fn append_section(
    topo: &Topology,
    face: &Face,
    cylinder: &CylindricalSurface,
    plane_point: Point3,
    normal: Vec3,
    tol: Tolerance,
    segments: &mut Vec<(Point3, Point3)>,
) -> Result<(), OperationsError> {
    let (low, high) = axial_band(topo, face, cylinder, tol)?;
    let axial = normal.dot(cylinder.axis());
    let radial_x = cylinder.radius() * normal.dot(cylinder.x_axis());
    let radial_y = cylinder.radius() * normal.dot(cylinder.y_axis());
    let amplitude = radial_x.hypot(radial_y);
    let offset = normal.dot(plane_point - cylinder.origin());
    let first = axial * low;
    let last = axial * high;
    if offset < first.min(last) - amplitude - tol.linear
        || offset > first.max(last) + amplitude + tol.linear
    {
        return Ok(());
    }
    if axial.abs() <= tol.angular {
        return Err(unsupported(
            "cylindrical section parallel to axis requires authoritative cap clipping",
        ));
    }
    let center = offset / axial;
    let extent = amplitude / axial.abs();
    if center - extent < low - tol.linear || center + extent > high + tol.linear {
        return Err(unsupported(
            "cylindrical section crossing a rim requires authoritative cap clipping",
        ));
    }
    let point_at = |angle: f64| {
        let (sine, cosine) = angle.sin_cos();
        let level = (offset - radial_x * cosine - radial_y * sine) / axial;
        cylinder.evaluate(angle, level)
    };
    // Preserve the existing section's 64 chords, with exact loop closure and
    // without the infinite-carrier helper's arbitrary |v| <= 100 cutoff.
    let first = point_at(0.0);
    let mut previous = first;
    for i in 1..=64 {
        let next = if i == 64 {
            first
        } else {
            point_at(TAU * i as f64 / 64.0)
        };
        segments.push((previous, next));
        previous = next;
    }
    Ok(())
}
