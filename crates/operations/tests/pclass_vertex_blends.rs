//! P-Class 5.3 exit gates for N-way vertex blends.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::HashMap;
use std::f64::consts::TAU;

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{blend_failure_code, fillet_v2};
use remus_operations::extrude::extrude;
use remus_operations::heal::unify_faces;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_convex_hull};
use remus_operations::query::effective_face_normal;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::{validate_shell_closed, validate_shell_manifold};

fn assert_watertight(topo: &Topology, solid: SolidId) {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).unwrap();
    validate_shell_manifold(shell, topo).unwrap();

    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).unwrap();
    let mut canonical = HashMap::<(i64, i64, i64), u32>::new();
    let mut remap = vec![0_u32; mesh.positions.len()];
    for (index, point) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (point.x() * 1e7).round() as i64,
            (point.y() * 1e7).round() as i64,
            (point.z() * 1e7).round() as i64,
        );
        #[allow(clippy::cast_possible_truncation)]
        let next = canonical.len() as u32;
        remap[index] = *canonical.entry(key).or_insert(next);
    }
    let mut uses = HashMap::<(u32, u32), usize>::new();
    let mut signed_six_volume = 0.0;
    for triangle in mesh.indices.chunks_exact(3) {
        let points = [
            mesh.positions[triangle[0] as usize],
            mesh.positions[triangle[1] as usize],
            mesh.positions[triangle[2] as usize],
        ];
        signed_six_volume += points[0].x()
            * (points[1].y() * points[2].z() - points[1].z() * points[2].y())
            - points[0].y() * (points[1].x() * points[2].z() - points[1].z() * points[2].x())
            + points[0].z() * (points[1].x() * points[2].y() - points[1].y() * points[2].x());
        let vertices = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for (a, b) in [
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[0]),
        ] {
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_default() += 1;
        }
    }
    assert_eq!(uses.values().filter(|&&count| count != 2).count(), 0);
    let mesh_volume = (signed_six_volume / 6.0).abs();
    let brep_volume = solid_volume(topo, solid, 0.005).unwrap();
    let relative = (mesh_volume - brep_volume).abs() / brep_volume.abs().max(1.0);
    assert!(
        mesh_volume > 0.0 && brep_volume > 0.0 && relative < 0.03,
        "mesh volume {mesh_volume} vs B-Rep {brep_volume} ({:.2}%)",
        relative * 100.0
    );
}

fn assert_sphere_stripe_g1(topo: &Topology, solid: SolidId, expected_seams: usize) {
    let adjacency = topo.build_adjacency(solid).unwrap();
    let angular = Tolerance::new().angular;
    let mut seams = 0;
    for edge_id in solid_edges(topo, solid).unwrap() {
        let faces = adjacency.faces_for_edge(edge_id);
        if faces.len() != 2 {
            continue;
        }
        let tags = [
            topo.face(faces[0]).unwrap().surface().type_tag(),
            topo.face(faces[1]).unwrap().surface().type_tag(),
        ];
        if !tags.contains(&"sphere") || !tags.contains(&"cylinder") {
            continue;
        }

        let edge = topo.edge(edge_id).unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let (t0, t1) = edge.strict_domain().unwrap();
        let point = edge
            .curve()
            .evaluate_with_endpoints(f64::midpoint(t0, t1), start, end);
        let n0 = effective_face_normal(topo, faces[0], point).unwrap();
        let n1 = effective_face_normal(topo, faces[1], point).unwrap();
        let angle = n0.cross(n1).length().atan2(n0.dot(n1));
        assert!(
            angle <= angular,
            "{edge_id:?}: sphere/stripe normal angle {angle} exceeds {angular}"
        );
        seams += 1;
    }
    assert_eq!(seams, expected_seams);
}

fn apex_edges(topo: &Topology, solid: SolidId, apex: Point3) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            (start - apex).length() < 1e-8 || (end - apex).length() < 1e-8
        })
        .collect()
}

fn edge_between(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < 1e-8 && (end - b).length() < 1e-8)
                || ((start - b).length() < 1e-8 && (end - a).length() < 1e-8)
        })
        .unwrap()
}

#[test]
fn all_edges_box_has_watertight_g1_three_stripe_corners() {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 12.0, 10.0, 8.0).unwrap();
    let edges = solid_edges(&topo, input).unwrap();
    assert_eq!(edges.len(), 12);
    let result = fillet_v2(&mut topo, input, &edges, 0.5).unwrap();
    assert!(result.failed.is_empty());

    assert_watertight(&topo, result.solid);
    assert_sphere_stripe_g1(&topo, result.solid, 24);
    assert_eq!(
        solid_faces(&topo, result.solid)
            .unwrap()
            .into_iter()
            .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Sphere(_)))
            .count(),
        8
    );
}

#[test]
fn four_stripe_pyramid_apex_is_watertight_and_g1() {
    let mut topo = Topology::new();
    let mut points = Vec::new();
    for index in 0..4 {
        let angle = TAU * f64::from(index) / 4.0;
        points.push(Point3::new(6.0 * angle.cos(), 6.0 * angle.sin(), 0.0));
    }
    let apex = Point3::new(0.0, 0.0, 9.0);
    points.push(apex);
    let input = make_convex_hull(&mut topo, &points).unwrap();
    assert_eq!(unify_faces(&mut topo, input).unwrap(), 1);
    let edges = apex_edges(&topo, input, apex);
    assert_eq!(edges.len(), 4);
    let result = fillet_v2(&mut topo, input, &edges, 0.5).unwrap();
    assert!(result.failed.is_empty());

    assert_watertight(&topo, result.solid);
    assert_sphere_stripe_g1(&topo, result.solid, 4);
    let spheres: Vec<_> = solid_faces(&topo, result.solid)
        .unwrap()
        .into_iter()
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Sphere(sphere) => Some(sphere),
            _ => None,
        })
        .collect();
    assert_eq!(spheres.len(), 1);
    assert!((spheres[0].radius() - 0.5).abs() < 1e-10);

    let ellipse_edges: Vec<_> = solid_edges(&topo, result.solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Ellipse(_)))
        .collect();
    assert_eq!(ellipse_edges.len(), 4);
    for edge_id in ellipse_edges {
        let edge = topo.edge(edge_id).unwrap();
        let (t0, t1) = edge.strict_domain().unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        assert!((edge.curve().evaluate_with_endpoints(t0, start, end) - start).length() < 1e-10);
        assert!((edge.curve().evaluate_with_endpoints(t1, start, end) - end).length() < 1e-10);
    }
}

#[test]
fn alternating_material_sides_at_a_planar_vertex_fail_closed() {
    let mut topo = Topology::new();
    let profile = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(7.0, 0.0, 0.0),
        Point3::new(7.0, 2.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(2.0, 7.0, 0.0),
        Point3::new(0.0, 7.0, 0.0),
    ];
    let wire = make_polygon_wire(&mut topo, &profile, 1e-7).unwrap();
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let input = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 5.0).unwrap();
    let corner = Point3::new(2.0, 2.0, 5.0);
    let edges = [
        edge_between(&topo, input, Point3::new(2.0, 2.0, 0.0), corner),
        edge_between(&topo, input, Point3::new(7.0, 2.0, 5.0), corner),
        edge_between(&topo, input, corner, Point3::new(2.0, 7.0, 5.0)),
    ];
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );
    let error = match fillet_v2(&mut topo, input, &edges, 0.4) {
        Ok(_) => panic!("alternating material sides unexpectedly built"),
        Err(error) => error,
    };
    assert_eq!(blend_failure_code(&error), "unsupported-vertex-blend");
    assert_eq!(
        before,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        ),
        "typed refusal must be transactional"
    );
}
