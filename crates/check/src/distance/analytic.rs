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

/// Native spherical caps use a complete equatorial ring as a topological
/// seam, represented by legacy Lines or certified native Circle spans.
/// The legacy chord polygon must not put spherical witnesses in air.
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
        if !matches!(edge.curve(), EdgeCurve::Line | EdgeCurve::Circle(_))
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
    let all_lines = edges.iter().all(|oe| {
        topo.edge(oe.edge())
            .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::Line))
    });
    if !all_lines && !circular_sphere_equator_partition(topo, a, edges, roundoff) {
        return Ok(None);
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

/// Certify the new native Circle partition without broadening the legacy
/// polygonal seam rule. Shared frames and directed native intervals, rather
/// than just their endpoint polygon, prove coverage of the actual equator.
#[allow(clippy::cast_precision_loss, clippy::float_cmp, clippy::too_many_lines)]
fn circular_sphere_equator_partition(
    topo: &Topology,
    sphere: &remus_math::surfaces::SphericalSurface,
    edges: &[remus_topology::wire::OrientedEdge],
    roundoff: f64,
) -> bool {
    use std::f64::consts::{FRAC_PI_2, TAU};
    let Some(first) = edges.first().and_then(|oe| topo.edge(oe.edge()).ok()) else {
        return false;
    };
    let EdgeCurve::Circle(reference) = first.curve() else {
        return false;
    };
    let angular_roundoff = 128.0 * f64::EPSILON;
    let (u, v, normal) = (reference.u_axis(), reference.v_axis(), reference.normal());
    if !reference.radius().is_finite()
        || !sphere.radius().is_finite()
        || !roundoff.is_finite()
        || reference.radius() <= 0.0
        || sphere.radius() <= 0.0
        || ![u, v, normal, sphere.z_axis()]
            .iter()
            .all(|axis| axis.0.iter().all(|value| value.is_finite()))
        || !reference.center().0.iter().all(|value| value.is_finite())
        || reference.center() != sphere.center()
        || (reference.radius() - sphere.radius()).abs() > angular_roundoff * sphere.radius()
        || (u.length_squared() - 1.0).abs() > angular_roundoff
        || (v.length_squared() - 1.0).abs() > angular_roundoff
        || (normal.length_squared() - 1.0).abs() > angular_roundoff
        || u.dot(v).abs() > angular_roundoff
        || (u.cross(v) - normal).length() > angular_roundoff
        // Reuse the parent's physical equator allowance: the sphere and
        // circle frame writers can accrue different transform roundoff.
        || normal.cross(sphere.z_axis()).length() * sphere.radius() > roundoff
    {
        return false;
    }
    let near = |a: Point3, b: Point3| {
        let offset = a - b;
        let distance = offset.x().hypot(offset.y()).hypot(offset.z());
        distance.is_finite() && distance <= roundoff
    };
    let mut turn = 0.0_f64;
    let mut sign = 0.0_f64;
    for (index, oriented) in edges.iter().enumerate() {
        let (Ok(edge), Ok(next)) = (
            topo.edge(oriented.edge()),
            topo.edge(edges[(index + 1) % edges.len()].edge()),
        ) else {
            return false;
        };
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return false;
        };
        if circle.center() != reference.center()
            || circle.radius().to_bits() != reference.radius().to_bits()
            || circle.normal() != normal
            || circle.u_axis() != u
            || circle.v_axis() != v
            || oriented.oriented_end(edge) != edges[(index + 1) % edges.len()].oriented_start(next)
        {
            return false;
        }
        let (Ok((a, b)), Ok((next_a, next_b))) = (edge.strict_domain(), next.strict_domain())
        else {
            return false;
        };
        let (start, end) = if oriented.is_forward() {
            (a, b)
        } else {
            (b, a)
        };
        let next_start = if edges[(index + 1) % edges.len()].is_forward() {
            next_a
        } else {
            next_b
        };
        let delta = end - start;
        if start.abs() > 64.0 * TAU
            || end.abs() > 64.0 * TAU
            || !delta.is_finite()
            || delta.abs() <= f64::EPSILON
            || delta.abs() > FRAC_PI_2 + angular_roundoff
            || (sign != 0.0 && delta.signum() != sign)
        {
            return false;
        }
        let (Ok(start_vertex), Ok(end_vertex)) = (
            topo.vertex(oriented.oriented_start(edge)),
            topo.vertex(oriented.oriented_end(edge)),
        ) else {
            return false;
        };
        if !near(circle.evaluate(start), start_vertex.point())
            || !near(circle.evaluate(end), end_vertex.point())
        {
            return false;
        }
        let gap = (end - next_start).rem_euclid(TAU);
        if gap.min(TAU - gap) > angular_roundoff * TAU {
            return false;
        }
        sign = delta.signum();
        turn += delta;
    }
    (turn.abs() - TAU).abs() <= angular_roundoff * TAU * edges.len() as f64
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

#[cfg(test)]
mod sphere_equator_tests {
    #![allow(clippy::unwrap_used, clippy::panic, clippy::cast_precision_loss)]

    use super::*;
    use remus_math::curves::Circle3D;
    use remus_topology::edge::{Edge, EdgeId};
    use remus_topology::face::Face;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    fn sphere(
        radius: f64,
        center: Point3,
        axis: Vec3,
        seam: f64,
        descending: bool,
    ) -> (Topology, SolidId, Vec<EdgeId>) {
        let mut topo = Topology::new();
        let reference = Vec3::new(1.0, 0.0, 0.0);
        let circle = Circle3D::new_with_ref(center, axis, radius, reference).unwrap();
        let surface = SphericalSurface::with_frame(center, radius, axis, reference).unwrap();
        let parameters: Vec<_> = (0..=8)
            .map(|i| seam + std::f64::consts::TAU * f64::from(i) / 8.0)
            .collect();
        let vertices: Vec<_> = parameters[..8]
            .iter()
            .map(|&t| topo.add_vertex(Vertex::new(circle.evaluate(t), 1e-7)))
            .collect();
        let edges: Vec<_> = (0..8)
            .map(|i| {
                let (start, end, domain) = if descending {
                    (
                        vertices[(i + 1) % 8],
                        vertices[i],
                        (parameters[i + 1], parameters[i]),
                    )
                } else {
                    (
                        vertices[i],
                        vertices[(i + 1) % 8],
                        (parameters[i], parameters[i + 1]),
                    )
                };
                let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
                edge.set_trim(Some(domain));
                topo.add_edge(edge)
            })
            .collect();
        let north = edges
            .iter()
            .map(|&edge| {
                if descending {
                    OrientedEdge::new(edge, false)
                } else {
                    OrientedEdge::new(edge, true)
                }
            })
            .collect();
        let south = edges
            .iter()
            .rev()
            .map(|&edge| {
                if descending {
                    OrientedEdge::new(edge, true)
                } else {
                    OrientedEdge::new(edge, false)
                }
            })
            .collect();
        let north = topo.add_wire(Wire::new(north, true).unwrap());
        let south = topo.add_wire(Wire::new(south, true).unwrap());
        let north = topo.add_face(Face::new(
            north,
            vec![],
            FaceSurface::Sphere(surface.clone()),
        ));
        let south = topo.add_face(Face::new(south, vec![], FaceSurface::Sphere(surface)));
        let shell = topo.add_shell(Shell::new(vec![north, south]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        (topo, solid, edges)
    }

    #[test]
    fn complete_circle_spheres_keep_native_descending_and_legacy_authority() {
        for radius in [0.001, 1.0, 1000.0, 1e154] {
            for axis in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.3, -0.4, 0.5)] {
                for descending in [false, true] {
                    let center = Point3::new(3.0 * radius, -2.0 * radius, radius);
                    let (mut topo, solid, edges) = sphere(radius, center, axis, 1.3, descending);
                    assert!(complete_sphere(&topo, solid).unwrap().is_some());
                    for edge in edges {
                        topo.edge_mut(edge).unwrap().set_curve(EdgeCurve::Line);
                    }
                    assert!(
                        complete_sphere(&topo, solid).unwrap().is_some(),
                        "legacy seam"
                    );
                }
            }
        }
    }

    #[test]
    fn complete_circle_sphere_rejects_false_native_spans_carriers_and_chains() {
        let (original, solid, edges) = sphere(
            1.0,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
            false,
        );
        let edge_id = edges[0];
        let (a, b) = original.edge(edge_id).unwrap().strict_domain().unwrap();
        for mutation in 0..13 {
            let mut topo = original.clone();
            match mutation {
                0 => topo.edge_mut(edge_id).unwrap().set_trim(None),
                1 => topo
                    .edge_mut(edge_id)
                    .unwrap()
                    .set_trim(Some((a, (a + b) * 0.5))),
                2 => topo
                    .edge_mut(edge_id)
                    .unwrap()
                    .set_trim(Some((a, b - std::f64::consts::TAU))),
                3 => topo
                    .edge_mut(edge_id)
                    .unwrap()
                    .set_trim(Some((a, b + std::f64::consts::TAU))),
                4 => topo
                    .edge_mut(edge_id)
                    .unwrap()
                    .set_trim(Some((a + 0.01, b + 0.01))),
                5 => topo.edge_mut(edge_id).unwrap().set_curve(EdgeCurve::Line),
                6 => {
                    let start = topo.edge(edge_id).unwrap().start();
                    let twin = topo.add_vertex(topo.vertex(start).unwrap().clone());
                    topo.edge_mut(edge_id).unwrap().set_start(twin);
                }
                11 => topo.edge_mut(edge_id).unwrap().set_trim(Some((
                    a + 128.0 * std::f64::consts::TAU,
                    b + 128.0 * std::f64::consts::TAU,
                ))),
                12 => {
                    let vertex = topo.edge(edge_id).unwrap().start();
                    topo.vertex_mut(vertex)
                        .unwrap()
                        .set_point(Point3::new(1.0, 0.0, 0.1));
                }
                _ => {
                    for &id in &edges {
                        let edge = topo.edge(id).unwrap();
                        let EdgeCurve::Circle(circle) = edge.curve() else {
                            unreachable!()
                        };
                        let carrier = Circle3D::with_axes(
                            circle.center()
                                + if mutation == 7 {
                                    Vec3::new(0.0, 0.0, 0.1)
                                } else {
                                    Vec3::new(0.0, 0.0, 0.0)
                                },
                            circle.normal(),
                            if mutation == 8 { 1.1 } else { circle.radius() },
                            circle.u_axis() * if mutation == 9 { 1.1 } else { 1.0 },
                            if mutation == 10 {
                                circle.u_axis()
                            } else {
                                circle.v_axis()
                            },
                        )
                        .unwrap();
                        let trim = edge.trim();
                        topo.edge_mut(id)
                            .unwrap()
                            .set_curve(EdgeCurve::Circle(carrier));
                        topo.edge_mut(id).unwrap().set_trim(trim);
                    }
                }
            }
            let result = complete_sphere(&topo, solid);
            if mutation == 0 || mutation == 3 {
                assert!(
                    matches!(result, Err(CheckError::ValidationFailed(_))),
                    "mutation {mutation}: {result:?}"
                );
            } else {
                assert!(result.unwrap().is_none(), "mutation {mutation}");
            }
        }
    }
}
