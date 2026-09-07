use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::FRAC_PI_2;

use remus_math::{surfaces::CylindricalSurface, tolerance::Tolerance, vec::Vec3};
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    face::{FaceId, FaceSurface},
};

use crate::OffsetError;

fn unsupported(face: FaceId, reason: &str) -> OffsetError {
    OffsetError::UnsupportedMoveFace {
        face,
        surface_type: "cylinder",
        reason: reason.into(),
    }
}

pub(super) fn validate_clearance<V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    selected: FaceId,
    replacement: &CylindricalSurface,
    edge_faces: &BTreeMap<usize, V>,
    tolerance: Tolerance,
) -> Result<(), OffsetError> {
    let face = topo.face(selected)?;
    let FaceSurface::Cylinder(source) = face.surface() else {
        return Err(unsupported(
            selected,
            "quarter-wall source is not cylindrical",
        ));
    };
    let wire = topo.wire(face.outer_wire())?;
    if !face.inner_wires().is_empty() || wire.edges().len() != 4 {
        return Err(unsupported(
            selected,
            "quarter-wall replacement requires two circular rims and two axial sides",
        ));
    }
    let mut levels = Vec::new();
    let mut middle = None;
    let mut neighbors = BTreeMap::new();
    for oriented in wire.edges() {
        let edge_id = oriented.edge();
        let edge = topo.edge(edge_id)?;
        match edge.curve() {
            EdgeCurve::Circle(circle) => {
                let Some((lo, hi)) = edge.trim() else {
                    return Err(unsupported(
                        selected,
                        "quarter-wall rims require authoritative trims",
                    ));
                };
                let offset = circle.center() - source.origin();
                let level = offset.dot(source.axis());
                if ((hi - lo).abs() - FRAC_PI_2).abs() > tolerance.angular
                    || circle.normal().cross(source.axis()).length() > tolerance.angular
                    || (circle.radius() - source.radius()).abs() > tolerance.linear
                    || (offset - source.axis() * level).length() > tolerance.linear
                {
                    return Err(unsupported(
                        selected,
                        "quarter-wall rims must be coaxial quarter circles",
                    ));
                }
                let radial = circle.evaluate(f64::midpoint(lo, hi)) - circle.center();
                if let Some(previous) = middle {
                    if (radial - previous).length() > tolerance.linear {
                        return Err(unsupported(selected, "quarter-wall rim sectors disagree"));
                    }
                } else {
                    middle = Some(radial);
                }
                levels.push(level);
            }
            EdgeCurve::Line => {
                let direction =
                    topo.vertex(edge.end())?.point() - topo.vertex(edge.start())?.point();
                if direction.cross(source.axis()).length() > tolerance.linear {
                    return Err(unsupported(
                        selected,
                        "quarter-wall side edges must be axial",
                    ));
                }
            }
            _ => {
                return Err(unsupported(
                    selected,
                    "quarter-wall boundaries must be lines and circles",
                ));
            }
        }
        let adjacent = edge_faces
            .get(&edge_id.index())
            .ok_or_else(|| unsupported(selected, "quarter-wall edge has no adjacency"))?;
        for &neighbor in adjacent.iter() {
            if neighbor != selected {
                neighbors.insert(neighbor.index(), neighbor);
            }
        }
    }
    let Some(middle) = middle else {
        return Err(unsupported(selected, "quarter-wall has no circular rims"));
    };
    if levels.len() != 2 || neighbors.len() != 4 {
        return Err(unsupported(
            selected,
            "quarter-wall requires four distinct planar supports",
        ));
    }
    levels.sort_by(f64::total_cmp);
    if levels[1] - levels[0] <= tolerance.linear {
        return Err(unsupported(selected, "quarter-wall has no axial span"));
    }
    let mut radial_normals: Vec<Vec3> = Vec::new();
    let mut caps = Vec::new();
    for &neighbor in neighbors.values() {
        let FaceSurface::Plane { normal, d } = topo.face(neighbor)?.surface() else {
            return Err(unsupported(
                selected,
                "quarter-wall supports must be planar",
            ));
        };
        let origin_dot = normal.dot(source.origin() - remus_math::vec::Point3::new(0.0, 0.0, 0.0));
        let alignment = normal.dot(source.axis());
        if alignment.abs() > 1.0 - tolerance.angular {
            caps.push((d - origin_dot) / alignment);
        } else if alignment.abs() <= tolerance.angular && (origin_dot - d).abs() <= tolerance.linear
        {
            radial_normals.push(if normal.dot(middle) > 0.0 {
                *normal
            } else {
                -*normal
            });
        } else {
            return Err(unsupported(
                selected,
                "quarter-wall sides must be radial planes and caps must be perpendicular to its axis",
            ));
        }
    }
    caps.sort_by(f64::total_cmp);
    if radial_normals.len() != 2
        || caps.len() != 2
        || radial_normals[0].dot(radial_normals[1]).abs() > tolerance.angular
        || (caps[0] - levels[0]).abs() > tolerance.linear
        || (caps[1] - levels[1]).abs() > tolerance.linear
    {
        return Err(unsupported(
            selected,
            "quarter-wall supports do not bound one quarter-sector prism",
        ));
    }
    let mut excluded: BTreeSet<_> = neighbors.keys().copied().collect();
    excluded.insert(selected.index());
    let source_faces: BTreeMap<_, _> = edge_faces
        .values()
        .flat_map(|faces| faces.iter().copied())
        .map(|face| (face.index(), face))
        .collect();
    let radius_min = source.radius().min(replacement.radius());
    let radius_max = source.radius().max(replacement.radius());
    for (index, other) in source_faces {
        if excluded.contains(&index) {
            continue;
        }
        let data = topo.face(other)?;
        if !matches!(data.surface(), FaceSurface::Plane { .. }) {
            return Err(unsupported(
                selected,
                "quarter-wall clearance currently requires planar nonadjacent faces",
            ));
        }
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for wire_id in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                let edge = topo.edge(oriented.edge())?;
                if !matches!(edge.curve(), EdgeCurve::Line) {
                    return Err(unsupported(
                        selected,
                        "quarter-wall clearance requires straight boundaries on nonadjacent planar faces",
                    ));
                }
                for vertex in [edge.start(), edge.end()] {
                    let delta = topo.vertex(vertex)?.point() - source.origin();
                    let coordinates = [
                        delta.dot(radial_normals[0]),
                        delta.dot(radial_normals[1]),
                        delta.dot(source.axis()),
                    ];
                    if coordinates.iter().any(|value| !value.is_finite()) {
                        return Err(unsupported(
                            selected,
                            "quarter-wall clearance requires finite geometry",
                        ));
                    }
                    for coordinate in 0..3 {
                        lo[coordinate] = lo[coordinate].min(coordinates[coordinate]);
                        hi[coordinate] = hi[coordinate].max(coordinates[coordinate]);
                    }
                }
            }
        }
        if lo.iter().chain(hi.iter()).any(|value| !value.is_finite()) {
            return Err(unsupported(
                selected,
                "quarter-wall clearance requires nonempty face boundaries",
            ));
        }
        if hi[2] < levels[0] - tolerance.linear
            || lo[2] > levels[1] + tolerance.linear
            || hi[0] < -tolerance.linear
            || hi[1] < -tolerance.linear
        {
            continue;
        }
        let nearest = lo[0].max(0.0).hypot(lo[1].max(0.0));
        let farthest = hi[0].max(0.0).hypot(hi[1].max(0.0));
        // The projected box contains the entire straight-bounded planar face,
        // so clearance covers face interiors as well as their boundary edges.
        if nearest <= radius_max + tolerance.linear && farthest >= radius_min - tolerance.linear {
            return Err(OffsetError::TopologyChange {
                face: Some(other),
                edge: None,
                reason: "quarter-wall sweep may contact a nonadjacent source face".into(),
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
    use super::*;
    use remus_math::{curves::Circle3D, vec::Point3};
    use remus_topology::{
        edge::Edge,
        face::Face,
        vertex::Vertex,
        wire::{OrientedEdge, Wire},
    };

    #[test]
    fn clearance_includes_face_interior_when_all_its_edges_are_outside() {
        let mut topo = Topology::new();
        let points = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
            Point3::new(0.0, 0.0, 12.0),
            Point3::new(4.0, 0.0, 12.0),
            Point3::new(0.0, 4.0, 12.0),
        ];
        let vertices: Vec<_> = points
            .into_iter()
            .map(|point| topo.add_vertex(Vertex::new(point, 1e-7)))
            .collect();
        let mut edges = Vec::new();
        for (start, end, circular) in [
            (1, 2, true),
            (2, 5, false),
            (4, 5, true),
            (1, 4, false),
            (0, 1, false),
            (0, 2, false),
            (3, 4, false),
            (3, 5, false),
            (0, 3, false),
        ] {
            let mut edge = Edge::new(
                vertices[start],
                vertices[end],
                if circular {
                    EdgeCurve::Circle(
                        Circle3D::new_with_ref(
                            Point3::new(0.0, 0.0, points[start].z()),
                            Vec3::new(0.0, 0.0, 1.0),
                            4.0,
                            Vec3::new(1.0, 0.0, 0.0),
                        )
                        .unwrap(),
                    )
                } else {
                    EdgeCurve::Line
                },
            );
            if circular {
                edge.set_trim(Some((0.0, FRAC_PI_2)));
            }
            edges.push(topo.add_edge(edge));
        }
        let cylinder =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 4.0)
                .unwrap();
        let mut face_ids = Vec::new();
        for (boundary, surface) in [
            (
                vec![(0, true), (1, true), (2, false), (3, false)],
                FaceSurface::Cylinder(cylinder.clone()),
            ),
            (
                vec![(4, true), (0, true), (5, false)],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            ),
            (
                vec![(6, true), (2, true), (7, false)],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 12.0,
                },
            ),
            (
                vec![(4, true), (3, true), (6, false), (8, false)],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, -1.0, 0.0),
                    d: 0.0,
                },
            ),
            (
                vec![(5, false), (8, true), (7, true), (1, false)],
                FaceSurface::Plane {
                    normal: Vec3::new(-1.0, 0.0, 0.0),
                    d: 0.0,
                },
            ),
        ] {
            let wire = topo.add_wire(
                Wire::new(
                    boundary
                        .into_iter()
                        .map(|(edge, forward)| OrientedEdge::new(edges[edge], forward))
                        .collect(),
                    true,
                )
                .unwrap(),
            );
            face_ids.push(topo.add_face(Face::new(wire, vec![], surface)));
        }
        topo.face_mut(face_ids[1]).unwrap().set_reversed(true);
        let shell = topo.add_shell(remus_topology::shell::Shell::new(face_ids.clone()).unwrap());
        let solid = topo.add_solid(remus_topology::solid::Solid::new(shell, vec![]));
        let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
        assert_eq!((report.error_count(), report.warning_count()), (0, 0));
        assert!(
            (remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap()
                - 48.0 * std::f64::consts::PI)
                .abs()
                < 1e-8
        );
        let obstacle_points = [
            Point3::new(4.2, -10.0, -10.0),
            Point3::new(4.2, 10.0, -10.0),
            Point3::new(4.2, 10.0, 20.0),
            Point3::new(4.2, -10.0, 20.0),
        ];
        for index in 0..4 {
            let a = obstacle_points[index];
            let b = obstacle_points[(index + 1) % 4];
            assert!(
                a.z().max(b.z()) < 0.0
                    || a.z().min(b.z()) > 12.0
                    || a.y().min(b.y()) > 4.5
                    || a.y().max(b.y()) < 0.0
            );
        }
        let wire =
            remus_topology::builder::make_polygon_wire(&mut topo, &obstacle_points, 1e-7).unwrap();
        let obstacle =
            remus_topology::builder::make_planar_face_from_wire(&mut topo, wire).unwrap();
        face_ids.push(obstacle);
        let mut adjacency: BTreeMap<usize, Vec<FaceId>> = BTreeMap::new();
        for &face in &face_ids {
            for oriented in topo
                .wire(topo.face(face).unwrap().outer_wire())
                .unwrap()
                .edges()
            {
                adjacency
                    .entry(oriented.edge().index())
                    .or_default()
                    .push(face);
            }
        }
        let replacement = CylindricalSurface::new(cylinder.origin(), cylinder.axis(), 4.5).unwrap();
        let error = validate_clearance(
            &topo,
            face_ids[0],
            &replacement,
            &adjacency,
            Tolerance::new(),
        )
        .unwrap_err();
        assert!(
            matches!(error, OffsetError::TopologyChange {face: Some(face), ..} if face == obstacle)
        );
        // The same face is clear of a smaller outward sweep.
        let replacement = CylindricalSurface::new(cylinder.origin(), cylinder.axis(), 4.1).unwrap();
        validate_clearance(
            &topo,
            face_ids[0],
            &replacement,
            &adjacency,
            Tolerance::new(),
        )
        .unwrap();
    }
}
