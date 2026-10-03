//! Closed-form point-to-surface distance for analytic surfaces.
//!
//! These are thin wrappers around `remus_geometry::extrema` that extract
//! `(distance, point)` from a [`SurfaceProjection`].

use remus_geometry::extrema::{
    SurfaceProjection, point_to_cone as geo_point_to_cone,
    point_to_cylinder as geo_point_to_cylinder, point_to_plane as geo_point_to_plane,
    point_to_sphere as geo_point_to_sphere, point_to_torus as geo_point_to_torus,
};
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use super::DistanceResult;
use crate::CheckError;

/// Qualify the native two-hemisphere representation as a complete sphere.
/// Cropped spherical faces, holes, extra shells and incomplete ring traversals
/// do not authorize a whole-carrier extremum.
#[allow(clippy::cast_precision_loss, clippy::float_cmp, clippy::too_many_lines)]
pub(super) fn complete_sphere(
    topo: &Topology,
    solid_id: SolidId,
) -> Result<Option<SphericalSurface>, CheckError> {
    let solid = topo.solid(solid_id)?;
    if !solid.inner_shells().is_empty() {
        return Ok(None);
    }
    let shell = topo.shell(solid.outer_shell())?;
    let [north_id, south_id] = shell.faces() else {
        return Ok(None);
    };
    let north = topo.face(*north_id)?;
    let south = topo.face(*south_id)?;
    let (FaceSurface::Sphere(a), FaceSurface::Sphere(b)) = (north.surface(), south.surface())
    else {
        return Ok(None);
    };
    if !north.inner_wires().is_empty()
        || !south.inner_wires().is_empty()
        || a.center() != b.center()
        || a.radius() != b.radius()
        || a.z_axis() != b.z_axis()
    {
        return Ok(None);
    }
    let ring = topo.wire(north.outer_wire())?;
    let other = topo.wire(south.outer_wire())?;
    let edges = ring.edges();
    if !ring.is_closed()
        || !other.is_closed()
        || edges.len() < 4
        || edges.len() != other.edges().len()
        || !edges
            .iter()
            .zip(other.edges().iter().rev())
            .all(|(a, b)| a.edge() == b.edge() && a.is_forward() != b.is_forward())
    {
        return Ok(None);
    }
    if native_sphere_equator(topo, *north_id)?.is_none() {
        return Ok(None);
    }
    Ok(Some(a.clone()))
}

/// Native spherical caps use a line-edged equatorial ring as a topological
/// seam. Its spherical boundary is the complete equator, not the inscribed
/// chord polygon; treating those chords as geometry puts witnesses in air.
#[allow(clippy::cast_precision_loss, clippy::float_cmp)]
pub(super) fn native_sphere_equator(
    topo: &Topology,
    face_id: remus_topology::face::FaceId,
) -> Result<Option<remus_math::curves::Circle3D>, CheckError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Sphere(a) = face.surface() else {
        return Ok(None);
    };
    if !face.inner_wires().is_empty() {
        return Ok(None);
    }
    let ring = topo.wire(face.outer_wire())?;
    let edges = ring.edges();
    if !ring.is_closed() || edges.len() < 4 {
        return Ok(None);
    }
    let scale = a
        .radius()
        .max(a.center().x().abs())
        .max(a.center().y().abs())
        .max(a.center().z().abs());
    let roundoff = 128.0 * f64::EPSILON * scale;
    let mut angles = Vec::with_capacity(edges.len());
    for (i, oriented) in edges.iter().enumerate() {
        let edge = topo.edge(oriented.edge())?;
        edge.strict_domain()
            .map_err(crate::error::edge_domain_validation)?;
        let next = topo.edge(edges[(i + 1) % edges.len()].edge())?;
        if !matches!(edge.curve(), EdgeCurve::Line)
            || oriented.oriented_end(edge) != edges[(i + 1) % edges.len()].oriented_start(next)
        {
            return Ok(None);
        }
        let offset = topo.vertex(oriented.oriented_start(edge))?.point() - a.center();
        if offset.dot(a.z_axis()).abs() > roundoff
            || (stable_length(offset) - a.radius()).abs() > roundoff
        {
            return Ok(None);
        }
        angles.push(offset.dot(a.y_axis()).atan2(offset.dot(a.x_axis())));
    }
    let mut winding = 0.0;
    let mut sign = 0.0_f64;
    for i in 0..angles.len() {
        let difference = angles[(i + 1) % angles.len()] - angles[i];
        let step = difference.sin().atan2(difference.cos());
        if step.abs() <= f64::EPSILON || (sign != 0.0 && step.signum() != sign) {
            return Ok(None);
        }
        sign = step.signum();
        winding += step;
    }
    if (winding.abs() - std::f64::consts::TAU).abs() > 128.0 * f64::EPSILON * edges.len() as f64 {
        return Ok(None);
    }
    Ok(Some(remus_math::curves::Circle3D::new(
        a.center(),
        a.z_axis(),
        a.radius(),
    )?))
}

/// Global boundary extrema of two complete spheres, including nested and
/// intersecting carriers. The returned points belong to both actual carriers.
pub(super) fn sphere_pair(
    a: &SphericalSurface,
    b: &SphericalSurface,
) -> Result<DistanceResult, CheckError> {
    let delta = b.center() - a.center();
    let distance = stable_length(delta);
    if !distance.is_finite() {
        return Err(CheckError::DistanceFailed(
            "sphere separation exceeds the finite arithmetic range".into(),
        ));
    }
    let direction = if distance == 0.0 {
        a.x_axis()
    } else {
        delta * (1.0 / distance)
    };
    let (ra, rb) = (a.radius(), b.radius());
    let scale = distance.max(ra).max(rb);
    let (da, na, nb) = (distance / scale, ra / scale, rb / scale);
    let (point_a, point_b) = if da > na + nb {
        (a.center() + direction * ra, b.center() - direction * rb)
    } else if na > da + nb {
        (a.center() + direction * ra, b.center() + direction * rb)
    } else if nb > da + na {
        (a.center() - direction * ra, b.center() - direction * rb)
    } else if distance == 0.0 {
        (a.center() + direction * ra, b.center() + direction * rb)
    } else {
        // Radical-plane circle. Scaling the radius algebra avoids squaring
        // large radii merely to locate the common point.
        let normalized_x = 0.5 * (da + (na - nb) * ((na + nb) / da));
        let x = normalized_x * scale;
        let h = (na - normalized_x).max(0.0).sqrt() * (na + normalized_x).max(0.0).sqrt() * scale;
        let reference = if direction.x().abs() < 0.8 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let transverse = direction.cross(reference);
        let transverse = transverse * (1.0 / transverse.length());
        let common = a.center() + direction * x + transverse * h;
        (common, common)
    };
    let minimum = stable_length(point_a - point_b);
    if !minimum.is_finite()
        || ![
            point_a.x(),
            point_a.y(),
            point_a.z(),
            point_b.x(),
            point_b.y(),
            point_b.z(),
        ]
        .iter()
        .all(|coordinate| coordinate.is_finite())
    {
        return Err(CheckError::DistanceFailed(
            "sphere extremum has no finite distance witness".into(),
        ));
    }
    Ok(DistanceResult {
        distance: minimum,
        point_a,
        point_b,
    })
}

fn stable_length(vector: Vec3) -> f64 {
    vector.x().hypot(vector.y()).hypot(vector.z())
}

/// Extract `(distance, point)` from a [`SurfaceProjection`].
fn extract(proj: SurfaceProjection) -> (f64, Point3) {
    (proj.distance, proj.point)
}

/// Closest point on a cylinder to a given point.
///
/// Returns `(distance, closest_point)`.
pub fn point_to_cylinder(point: Point3, cyl: &CylindricalSurface) -> (f64, Point3) {
    extract(geo_point_to_cylinder(point, cyl))
}

/// Closest point on a cone to a given point.
///
/// Returns `(distance, closest_point)`.
pub fn point_to_cone(point: Point3, cone: &ConicalSurface) -> (f64, Point3) {
    extract(geo_point_to_cone(point, cone))
}

/// Closest point on a sphere to a given point.
///
/// Returns `(distance, closest_point)`.
pub fn point_to_sphere(point: Point3, sphere: &SphericalSurface) -> (f64, Point3) {
    extract(geo_point_to_sphere(point, sphere))
}

/// Closest point on a torus to a given point.
///
/// Returns `(distance, closest_point)`.
pub fn point_to_torus(point: Point3, torus: &ToroidalSurface) -> (f64, Point3) {
    extract(geo_point_to_torus(point, torus))
}

/// Perpendicular distance from a point to an infinite plane.
///
/// The plane is defined by `normal · x = d` where `normal` is unit length.
/// Returns `(distance, closest_point)`.
pub fn point_to_plane(point: Point3, normal: Vec3, d: f64) -> (f64, Point3) {
    // Convert plane equation `normal · x = d` to an origin point on the plane.
    // For unit `normal`, the nearest origin is `normal * d`.
    let origin = Point3::new(normal.x() * d, normal.y() * d, normal.z() * d);
    extract(geo_point_to_plane(point, origin, normal))
}
