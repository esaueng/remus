//! Exact two-ridge corner on a right quarter-cylinder, including its sharp-edge ledge.

use std::collections::BTreeMap;
use std::f64::consts::FRAC_PI_2;

use remus_math::curves::Circle3D;
use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{EdgeId, FaceId, SolidId, Topology, VertexId};

use crate::builder_utils::add_certified_curve_edge;
use crate::radius_law::RadiusLaw;
use crate::{BlendEngine, BlendError, BlendFaceOrigins, BlendResult};

struct Cell {
    origin: Point3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
    radius: f64,
    height: f64,
    // Bottom, shared side, other side, cylinder, top.
    faces: [FaceId; 5],
    edges: [EdgeId; 2],
}

impl Cell {
    fn point(&self, x: f64, y: f64, z: f64) -> Point3 {
        self.origin + self.x * x + self.y * y + self.z * z
    }
}

pub fn try_build(
    topo: &mut Topology,
    solid: SolidId,
    chains: &[(Vec<EdgeId>, usize)],
    laws: &[RadiusLaw],
) -> Result<Option<BlendResult>, BlendError> {
    let [(first, first_law), (second, second_law)] = chains else {
        return Ok(None);
    };
    let ([a], [b]) = (first.as_slice(), second.as_slice()) else {
        return Ok(None);
    };
    let (RadiusLaw::Constant(radius), RadiusLaw::Constant(other)) =
        (&laws[*first_law], &laws[*second_law])
    else {
        return Ok(None);
    };
    if radius.to_bits() != other.to_bits() {
        return Ok(None);
    }
    let Some(cell) = recognize(topo, solid, [*a, *b])? else {
        return Ok(None);
    };
    let tol = Tolerance::new();
    // Both stripes must retain positive length; the ball must fit between the radial planes.
    if *radius >= cell.radius * 0.5 - tol.linear || *radius >= cell.height - tol.linear {
        return Err(BlendError::RadiusTooLarge {
            edge: *a,
            max_radius: (cell.radius * 0.5).min(cell.height),
        });
    }
    if *radius <= tol.linear || !radius.is_finite() {
        return Ok(None);
    }
    build(topo, solid, &cell, *radius).map(Some)
}

#[allow(clippy::too_many_lines)]
fn recognize(
    topo: &Topology,
    solid: SolidId,
    selected: [EdgeId; 2],
) -> Result<Option<Cell>, BlendError> {
    let tol = Tolerance::new();
    let faces = solid_faces(topo, solid)?;
    let vertices = solid_vertices(topo, solid)?;
    let edges = solid_edges(topo, solid)?;
    if !topo.solid(solid)?.inner_shells().is_empty()
        || faces.len() != 5
        || vertices.len() != 6
        || edges.len() != 9
    {
        return Ok(None);
    }
    let cylinders: Vec<_> = faces
        .iter()
        .copied()
        .filter(|&face| {
            matches!(
                topo.face(face).map(Face::surface),
                Ok(FaceSurface::Cylinder(_))
            )
        })
        .collect();
    let [wall] = cylinders.as_slice() else {
        return Ok(None);
    };
    let wall_data = topo.face(*wall)?;
    if wall_data.is_reversed() {
        return Ok(None);
    }
    let FaceSurface::Cylinder(cylinder) = wall_data.surface() else {
        return Ok(None);
    };
    let adjacency = topo.build_adjacency(solid)?;
    if !adjacency.boundary_edges().is_empty() || !adjacency.non_manifold_edges().is_empty() {
        return Ok(None);
    }
    let axial: Vec<_> = selected
        .iter()
        .copied()
        .filter(|&edge| adjacency.faces_for_edge(edge).contains(wall))
        .collect();
    let [axial] = axial.as_slice() else {
        return Ok(None);
    };
    let radial = if *axial == selected[0] {
        selected[1]
    } else {
        selected[0]
    };
    let a = topo.edge(*axial)?;
    let b = topo.edge(radial)?;
    if !matches!(a.curve(), EdgeCurve::Line) || !matches!(b.curve(), EdgeCurve::Line) {
        return Ok(None);
    }
    let common: Vec<_> = [a.start(), a.end()]
        .into_iter()
        .filter(|v| [b.start(), b.end()].contains(v))
        .collect();
    let [corner] = common.as_slice() else {
        return Ok(None);
    };
    let top_axis = topo
        .vertex(if b.start() == *corner {
            b.end()
        } else {
            b.start()
        })?
        .point();
    let bottom_rim = topo
        .vertex(if a.start() == *corner {
            a.end()
        } else {
            a.start()
        })?
        .point();
    let corner_point = topo.vertex(*corner)?.point();
    let z = (corner_point - bottom_rim).normalize()?;
    let height = (corner_point - bottom_rim).length();
    let x = (corner_point - top_axis).normalize()?;
    let radius = cylinder.radius();
    if ((corner_point - top_axis).length() - radius).abs() > tol.linear
        || x.dot(z).abs() > tol.angular
        || cylinder.axis().dot(z).abs() < 1.0 - tol.angular
    {
        return Ok(None);
    }
    let origin = top_axis - z * height;
    let radial_axis = origin - cylinder.origin();
    if (radial_axis - z * radial_axis.dot(z)).length() > tol.linear {
        return Ok(None);
    }
    let mut y = z.cross(x);
    if vertices
        .iter()
        .map(|&v| topo.vertex(v).map(|v| (v.point() - origin).dot(y)))
        .collect::<Result<Vec<_>, _>>()?
        .iter()
        .sum::<f64>()
        < 0.0
    {
        y = -y;
    }
    let positions = [
        origin,
        origin + x * radius,
        origin + y * radius,
        top_axis,
        corner_point,
        top_axis + y * radius,
    ];
    let mut ids = Vec::new();
    for point in positions {
        let matches: Vec<_> = vertices
            .iter()
            .copied()
            .filter(|&v| {
                topo.vertex(v)
                    .is_ok_and(|v| (v.point() - point).length() <= tol.linear)
            })
            .collect();
        let [id] = matches.as_slice() else {
            return Ok(None);
        };
        ids.push(*id);
    }
    let expected_edges = [
        (0, 1),
        (1, 2),
        (2, 0),
        (3, 4),
        (4, 5),
        (5, 3),
        (0, 3),
        (1, 4),
        (2, 5),
    ];
    for (i, j) in expected_edges {
        let matching: Vec<_> = edges
            .iter()
            .copied()
            .filter(|&e| {
                topo.edge(e).is_ok_and(|e| {
                    [e.start(), e.end()].contains(&ids[i]) && [e.start(), e.end()].contains(&ids[j])
                })
            })
            .collect();
        let [edge] = matching.as_slice() else {
            return Ok(None);
        };
        let edge = topo.edge(*edge)?;
        if (i, j) == (1, 2) || (i, j) == (4, 5) {
            let EdgeCurve::Circle(circle) = edge.curve() else {
                return Ok(None);
            };
            let Ok((t0, t1)) = edge.strict_domain() else {
                return Ok(None);
            };
            let center = if i == 1 { origin } else { top_axis };
            if (circle.center() - center).length() > tol.linear
                || (circle.radius() - radius).abs() > tol.linear
                || circle.normal().dot(z).abs() < 1.0 - tol.angular
                || ((t1 - t0).abs() - FRAC_PI_2).abs() > tol.angular
            {
                return Ok(None);
            }
            for (t, v) in [(t0, edge.start()), (t1, edge.end())] {
                if (circle.evaluate(t) - topo.vertex(v)?.point()).length() > tol.linear {
                    return Ok(None);
                }
            }
            let midpoint = circle.evaluate(f64::midpoint(t0, t1)) - center;
            if midpoint.dot(x) <= 0.0 || midpoint.dot(y) <= 0.0 {
                return Ok(None);
            }
        } else if !matches!(edge.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
    }
    let face_vertices: [&[usize]; 5] = [
        &[0, 1, 2],
        &[0, 1, 4, 3],
        &[0, 2, 5, 3],
        &[1, 2, 5, 4],
        &[3, 4, 5],
    ];
    let normals = [-z, -y, -x, x, z];
    let mut ordered = Vec::new();
    for (slot, indices) in face_vertices.iter().enumerate() {
        let expected: std::collections::BTreeSet<_> = indices.iter().map(|&i| ids[i]).collect();
        let mut matching = Vec::new();
        for &face in &faces {
            let data = topo.face(face)?;
            if !data.inner_wires().is_empty() {
                return Ok(None);
            }
            let wire = topo.wire(data.outer_wire())?;
            let mut actual = std::collections::BTreeSet::new();
            for oriented in wire.edges() {
                let edge = topo.edge(oriented.edge())?;
                actual.insert(edge.start());
                actual.insert(edge.end());
            }
            if actual != expected || wire.edges().len() != expected.len() || !wire.is_closed() {
                continue;
            }
            if slot == 3 {
                if face != *wall {
                    continue;
                }
            } else {
                let Some(normal) = data.effective_plane_normal() else {
                    continue;
                };
                let FaceSurface::Plane { normal: stored, d } = data.surface() else {
                    continue;
                };
                if normal.dot(normals[slot]) < 1.0 - tol.angular
                    || indices.iter().any(|&i| {
                        (stored.dot(positions[i] - Point3::new(0.0, 0.0, 0.0)) - *d).abs()
                            > tol.linear
                    })
                {
                    continue;
                }
            }
            remus_topology::validation::validate_face_loops(topo, face)?;
            matching.push(face);
        }
        let [face] = matching.as_slice() else {
            return Ok(None);
        };
        ordered.push(*face);
    }
    remus_topology::validation::validate_solid_pcurve_contracts(topo, solid, tol.linear, 32)
        .map_err(|error| BlendError::InvalidInput {
            reason: format!("quarter-cylinder source pcurve contract: {error}"),
        })?;
    let Ok(faces) = ordered.try_into() else {
        return Ok(None);
    };
    Ok(Some(Cell {
        origin,
        x,
        y,
        z,
        radius,
        height,
        faces,
        edges: selected,
    }))
}

fn arc(
    topo: &mut Topology,
    vertices: &[VertexId],
    start: usize,
    end: usize,
    center: Point3,
) -> Result<EdgeId, BlendError> {
    let a = topo.vertex(vertices[start])?.point() - center;
    let b = topo.vertex(vertices[end])?.point() - center;
    let normal = a.cross(b).normalize()?;
    let angle = a.normalize()?.dot(b.normalize()?).clamp(-1.0, 1.0).acos();
    let circle = Circle3D::new_with_ref(center, normal, a.length(), a)?;
    add_certified_curve_edge(
        topo,
        vertices[start],
        vertices[end],
        EdgeCurve::Circle(circle),
        (0.0, angle),
    )
}

#[allow(clippy::too_many_lines)]
fn build(
    topo: &mut Topology,
    source: SolidId,
    cell: &Cell,
    r: f64,
) -> Result<BlendResult, BlendError> {
    let tol = Tolerance::new();
    // The ball center lies on both offset supports: radial distance R-r and y=r.
    let xc = ((cell.radius - r).powi(2) - r * r).sqrt();
    let qx = xc / (cell.radius - r);
    let qy = r / (cell.radius - r);
    let q = cell.x * qx + cell.y * qy;
    let h = cell.height;
    let wall_x = cell.radius * qx;
    let wall_y = cell.radius * qy;
    let coords = [
        (0.0, 0.0, 0.0),
        (xc, 0.0, 0.0),
        (wall_x, wall_y, 0.0),
        (0.0, cell.radius, 0.0),
        (0.0, cell.radius, h),
        (wall_x, wall_y, h),
        (wall_x, wall_y, h - r),
        (xc, r, h),
        (xc, 0.0, h - r),
        (0.0, 0.0, h - r),
        (0.0, r, h),
    ];
    let points: Vec<_> = coords
        .iter()
        .map(|&(x, y, z)| cell.point(x, y, z))
        .collect();
    let vertices: Vec<_> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, tol.linear)))
        .collect();
    let center = cell.point(xc, r, h - r);
    let mut edges = BTreeMap::new();
    for (a, b, c) in [
        (1, 2, cell.point(xc, r, 0.0)),
        (2, 3, cell.origin),
        (4, 5, cell.point(0.0, 0.0, h)),
        (6, 8, center),
        (9, 10, cell.point(0.0, r, h - r)),
        (7, 8, center),
        (6, 7, center),
    ] {
        edges.insert((a.min(b), a.max(b)), arc(topo, &vertices, a, b, c)?);
    }
    let mean = (-cell.y + cell.z + q).normalize()?;
    let polar = remus_math::frame::Frame3::from_normal(center, mean)?.x;
    let sphere = SphericalSurface::with_frame(center, r, polar, -mean)?;
    let sphere_surface = FaceSurface::Sphere(sphere);
    // The third sphere arc needs a ledge to retain the unselected cap/wall rim as sharp.
    let ledge_normal = cell.x * qy - cell.y * qx;
    let plane = |normal: Vec3, point: Point3| FaceSurface::Plane {
        normal,
        d: normal.dot(point - Point3::new(0.0, 0.0, 0.0)),
    };
    let surfaces = [
        plane(-cell.z, cell.origin),
        plane(-cell.y, cell.origin),
        plane(-cell.x, cell.origin),
        topo.face(cell.faces[3])?.surface().clone(),
        plane(cell.z, cell.point(0.0, 0.0, h)),
        FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
            cell.point(xc, r, 0.0),
            cell.z,
            r,
            -cell.y,
        )?),
        FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
            cell.point(0.0, r, h - r),
            cell.x,
            r,
            -cell.y,
        )?),
        sphere_surface,
        plane(ledge_normal, center),
    ];
    let outlines: [&[usize]; 9] = [
        &[0, 1, 2, 3],
        &[0, 1, 8, 9],
        &[0, 3, 4, 10, 9],
        &[2, 3, 4, 5, 6],
        &[10, 7, 5, 4],
        &[1, 2, 6, 8],
        &[9, 10, 7, 8],
        &[8, 7, 6],
        &[7, 5, 6],
    ];
    let normals = [
        -cell.z,
        -cell.y,
        -cell.x,
        (q + cell.y).normalize()?,
        cell.z,
        (-cell.y + q).normalize()?,
        (-cell.y + cell.z).normalize()?,
        mean,
        ledge_normal,
    ];
    let mut faces = Vec::new();
    for (i, (outline, surface)) in outlines.iter().zip(surfaces).enumerate() {
        let mut cycle = outline.to_vec();
        let mut winding = Vec3::new(0.0, 0.0, 0.0);
        for j in 1..cycle.len() - 1 {
            winding += (points[cycle[j]] - points[cycle[0]])
                .cross(points[cycle[j + 1]] - points[cycle[0]]);
        }
        if winding.dot(normals[i]) < 0.0 {
            cycle.reverse();
        }
        let mut oriented = Vec::new();
        for j in 0..cycle.len() {
            let a = cycle[j];
            let b = cycle[(j + 1) % cycle.len()];
            let edge = *edges.entry((a.min(b), a.max(b))).or_insert_with(|| {
                topo.add_edge(Edge::new(vertices[a], vertices[b], EdgeCurve::Line))
            });
            oriented.push(OrientedEdge::new(
                edge,
                topo.edge(edge)?.start() == vertices[a],
            ));
        }
        let wire = topo.add_wire(Wire::new(oriented, true)?);
        let face = topo.add_face(Face::new(wire, vec![], surface));
        if i < 5
            && let Some(attributes) = topo.attributes().face(cell.faces[i]).cloned()
        {
            topo.set_face_attributes(face, attributes)?;
        }
        faces.push(face);
    }
    let origins = BlendFaceOrigins {
        survived: cell
            .faces
            .iter()
            .copied()
            .zip(faces.iter().copied())
            .collect(),
        deleted: vec![],
        created: vec![
            (faces[5], vec![cell.faces[1], cell.faces[3]]),
            (faces[6], vec![cell.faces[1], cell.faces[4]]),
            (faces[7], vec![cell.faces[1], cell.faces[3], cell.faces[4]]),
            (faces[8], vec![cell.faces[1], cell.faces[3], cell.faces[4]]),
        ],
        created_unattributed: vec![],
    };
    let shell = topo.add_shell(Shell::new(faces)?);
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    if let Some(attributes) = topo.attributes().solid(source).cloned() {
        topo.set_solid_attributes(solid, attributes)?;
    }
    Ok(BlendResult {
        solid,
        succeeded: cell.edges.to_vec(),
        failed: vec![],
        is_partial: false,
        face_origins: Some(origins),
        engine: BlendEngine::Walking,
    })
}
