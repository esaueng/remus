//! Independent spherical-patch geometry and shared-boundary oracles.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves::Circle3D;
use remus_math::det_hash::DetHashMap;
use remus_math::mat::Mat4;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{TriangleMesh, point_merge_key};

type ChartFixture = (
    Topology,
    Face,
    TriangleMesh,
    DetHashMap<usize, Vec<u32>>,
    DetHashMap<(i64, i64, i64), u32>,
);

fn direction(matrix: Mat4, vector: Vec3) -> Vec3 {
    Vec3::new(
        matrix.0[0][0] * vector.x() + matrix.0[0][1] * vector.y() + matrix.0[0][2] * vector.z(),
        matrix.0[1][0] * vector.x() + matrix.0[1][1] * vector.y() + matrix.0[1][2] * vector.z(),
        matrix.0[2][0] * vector.x() + matrix.0[2][1] * vector.y() + matrix.0[2][2] * vector.z(),
    )
}

fn patch(radius: f64, matrix: Mat4, wrong_stored_axis: bool, start: usize) -> ChartFixture {
    let mut topo = Topology::new();
    let center = matrix.mul_point(Point3::new(0.0, 0.0, 0.0));
    let points = [
        Point3::new(radius, 0.0, 0.0),
        Point3::new(0.0, 0.0, radius),
        Point3::new(0.0, radius, 0.0),
    ]
    .map(|p| matrix.mul_point(p));
    let vertices = points.map(|point| topo.add_vertex(Vertex::new(point, 1e-7)));
    let mut mesh = TriangleMesh {
        positions: points.to_vec(),
        normals: points.map(|p| (p - center) * (1.0 / radius)).to_vec(),
        indices: vec![],
    };
    let mut map = DetHashMap::default();
    for (index, point) in points.into_iter().enumerate() {
        map.insert(point_merge_key(point, super::MERGE_GRID), index as u32);
    }
    let frames = [
        (
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-1.0, 0.0, 0.0),
            1.5 * std::f64::consts::PI,
            std::f64::consts::TAU,
            0,
            1,
            8,
        ),
        (
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, -1.0, 0.0),
            std::f64::consts::PI,
            1.5 * std::f64::consts::PI,
            1,
            2,
            8,
        ),
        (
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            0.5 * std::f64::consts::PI,
            std::f64::consts::TAU,
            2,
            0,
            24,
        ),
    ];
    let mut edges = Vec::new();
    let mut pool = DetHashMap::default();
    for (normal, u, v, a, b, first, last, count) in frames {
        let circle = Circle3D::with_axes(
            center,
            direction(matrix, normal),
            radius,
            direction(matrix, u),
            direction(matrix, v),
        )
        .unwrap();
        let mut ids = vec![first as u32];
        for i in 1..count {
            let point = circle.evaluate(a + (b - a) * f64::from(i) / f64::from(count));
            let id = mesh.positions.len() as u32;
            mesh.positions.push(point);
            mesh.normals.push((point - center) * (1.0 / radius));
            map.insert(point_merge_key(point, super::MERGE_GRID), id);
            ids.push(id);
        }
        ids.push(last as u32);
        let mut edge = Edge::new(vertices[first], vertices[last], EdgeCurve::Circle(circle));
        edge.set_trim(Some((a, b)));
        let id = topo.add_edge(edge);
        pool.insert(id.index(), ids);
        edges.push(OrientedEdge::new(id, true));
    }
    edges.rotate_left(start);
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let axis = if wrong_stored_axis {
        Vec3::new(0.0, -1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    let sphere = SphericalSurface::with_axis(center, radius, direction(matrix, axis)).unwrap();
    let face = Face::new(wire, vec![], FaceSurface::Sphere(sphere));
    (topo, face, mesh, pool, map)
}

#[test]
fn sphere_chart_uses_the_complete_patch_and_keeps_shared_boundary_ids() {
    for radius in [0.001, 1.0, 1000.0] {
        for placed in [false, true] {
            let matrix = if placed {
                Mat4::translation(13.0 * radius, -7.0 * radius, 3.0 * radius)
                    * Mat4::rotation_z(0.43)
                    * Mat4::rotation_x(0.71)
            } else {
                Mat4::identity()
            };
            let inverse = matrix.inverse().unwrap();
            for wrong in [false, true] {
                for start in 0..3 {
                    for deflection in [0.05, 0.005] {
                        let (topo, face, mut mesh, pool, mut map) =
                            patch(radius, matrix, wrong, start);
                        let edges = topo.wire(face.outer_wire()).unwrap().edges();
                        let mut boundary = Vec::new();
                        for use_ in edges {
                            let ids = &pool[&use_.edge().index()];
                            boundary.extend_from_slice(&ids[..ids.len() - 1]);
                        }
                        let before = mesh.positions.clone();
                        assert!(
                            super::tessellate_sphere_cap_shared(
                                &topo,
                                &face,
                                deflection * radius,
                                0.35,
                                false,
                                &pool,
                                &mut mesh,
                                &mut map
                            )
                            .unwrap()
                        );
                        assert_eq!(&mesh.positions[..before.len()], before.as_slice());
                        let mut incidence = std::collections::BTreeMap::new();
                        let mut area = 0.0;
                        for t in mesh.indices.chunks_exact(3) {
                            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                                let key = if a < b { (a, b) } else { (b, a) };
                                *incidence.entry(key).or_insert(0) += 1;
                            }
                            let [a, b, c] =
                                [0, 1, 2].map(|i| inverse.mul_point(mesh.positions[t[i] as usize]));
                            let geometric = (b - a).cross(c - a);
                            let centroid = Point3::new(
                                (a.x() + b.x() + c.x()) / 3.0,
                                (a.y() + b.y() + c.y()) / 3.0,
                                (a.z() + b.z() + c.z()) / 3.0,
                            );
                            assert!(geometric.dot(centroid - Point3::new(0.0, 0.0, 0.0)) > 0.0);
                            assert!(centroid.z() >= -radius * 1e-10);
                            assert!(
                                !(centroid.x() > radius * 1e-10
                                    && centroid.y() > radius * 1e-10
                                    && centroid.z() > radius * 1e-10)
                            );
                            area += geometric.length() / 2.0;
                        }
                        for i in 0..boundary.len() {
                            let a = boundary[i];
                            let b = boundary[(i + 1) % boundary.len()];
                            let key = if a < b { (a, b) } else { (b, a) };
                            assert_eq!(incidence.remove(&key), Some(1));
                        }
                        assert!(incidence.values().all(|&count| count == 2));
                        let expected = 1.5 * std::f64::consts::PI * radius * radius;
                        assert!(
                            (area - expected).abs() / expected < 0.02,
                            "area {area} vs {expected}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn sphere_hemisphere_decline_restores_mesh_and_vertex_index() {
    let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
    // Crossing constraints form a bow tie, so the face cannot preserve its
    // original boundary incidence. It must not leak its attempted interior.
    let positions = [(-0.6, -0.4), (0.6, 0.4), (-0.6, 0.4), (0.6, -0.4)].map(|(x, y)| {
        let d = 1.0 + x * x + y * y;
        Point3::new(2.0 * x / d, 2.0 * y / d, (2.0 - d) / d)
    });
    let mut mesh = TriangleMesh {
        positions: positions.to_vec(),
        normals: positions.map(|p| p - Point3::new(0.0, 0.0, 0.0)).to_vec(),
        indices: vec![],
    };
    let mut map = DetHashMap::default();
    for (i, p) in positions.into_iter().enumerate() {
        map.insert(point_merge_key(p, super::MERGE_GRID), i as u32);
    }
    let before = mesh.clone();
    let before_map = map.clone();
    assert!(
        !super::fill_sphere_hemisphere_patch(
            &sphere,
            &[0, 1, 2, 3],
            0.05,
            0.35,
            &mut mesh,
            &mut map
        )
        .unwrap()
    );
    assert_eq!(mesh.positions, before.positions);
    assert_eq!(mesh.normals, before.normals);
    assert_eq!(mesh.indices, before.indices);
    assert_eq!(map, before_map);
}

#[test]
fn native_arc_guard_rejects_an_unsampled_excursion_into_the_opposite_hemisphere() {
    let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
    // z(t)=cos(t): the ends of the major arc are both north, while its
    // interior passes through the south pole. Endpoint/sample qualification
    // alone must not select the north chart for that stored native span.
    let circle = Circle3D::new_with_ref(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        1.0,
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    for ((a, b), expected) in [
        ((0.1, 6.0), false),
        ((6.0, 0.1), false),
        ((-0.5, 0.5), true),
        ((0.5, -0.5), true),
        ((1.5 * std::f64::consts::PI, std::f64::consts::TAU), true),
    ] {
        let mut topo = Topology::new();
        let first = topo.add_vertex(Vertex::new(circle.evaluate(a), 1e-7));
        let last = topo.add_vertex(Vertex::new(circle.evaluate(b), 1e-7));
        let mut edge = Edge::new(first, last, EdgeCurve::Circle(circle.clone()));
        edge.set_trim(Some((a, b)));
        let edge = topo.add_edge(edge);
        assert_eq!(
            super::sphere_native_arcs_in_hemisphere(
                &topo,
                &sphere,
                &[OrientedEdge::new(edge, true)],
                1.0
            )
            .unwrap(),
            expected
        );
    }
}

#[test]
fn new_chart_selection_leaves_unsupported_carriers_and_broken_id_chains_on_legacy_path() {
    for unsupported in [true, false] {
        let (mut topo, face, mesh, pool, _) = patch(1.0, Mat4::identity(), false, 0);
        let wire = topo.wire(face.outer_wire()).unwrap().edges().to_vec();
        let equator = wire[2].edge();
        if unsupported {
            topo.edge_mut(equator).unwrap().set_curve(EdgeCurve::Line);
        } else {
            let end = topo.edge(equator).unwrap().end();
            let point = topo.vertex(end).unwrap().point();
            let duplicate = topo.add_vertex(Vertex::new(point, 1e-7));
            topo.edge_mut(equator).unwrap().set_end(duplicate);
        }
        let sphere = match face.surface() {
            FaceSurface::Sphere(sphere) => sphere,
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => panic!("fixture sphere"),
        };
        assert!(!super::sphere_great_circle_loop(&topo, sphere, &wire).unwrap());
        let mut boundary = Vec::new();
        for use_ in &wire {
            let ids = &pool[&use_.edge().index()];
            boundary.extend_from_slice(&ids[..ids.len() - 1]);
        }
        let chart =
            super::sphere_hemisphere_chart(&topo, sphere, &wire, &boundary, &mesh.positions)
                .unwrap()
                .unwrap();
        assert!((chart.z_axis() - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-12);
        assert!(super::sphere_boundary_hemisphere(&chart, &boundary, &mesh.positions).is_none());
    }
}

#[test]
fn native_vector_area_declines_the_complement_and_keeps_descending_carrier_identity() {
    for radius in [0.001, 1.0, 1000.0] {
        for placed in [false, true] {
            let matrix = if placed {
                Mat4::translation(13.0 * radius, -7.0 * radius, 3.0 * radius)
                    * Mat4::rotation_z(0.43)
                    * Mat4::rotation_x(0.71)
            } else {
                Mat4::identity()
            };
            let (mut topo, face, mesh, pool, _) = patch(radius, matrix, false, 0);
            let edges = topo.wire(face.outer_wire()).unwrap().edges().to_vec();
            let FaceSurface::Sphere(sphere) = face.surface() else {
                panic!("fixture sphere")
            };
            let mut boundary = Vec::new();
            for use_ in &edges {
                let ids = &pool[&use_.edge().index()];
                boundary.extend_from_slice(&ids[..ids.len() - 1]);
            }
            assert!(
                super::sphere_hemisphere_chart(&topo, sphere, &edges, &boundary, &mesh.positions)
                    .unwrap()
                    .is_some()
            );
            let complement: Vec<_> = edges
                .iter()
                .rev()
                .map(|e| OrientedEdge::new(e.edge(), !e.is_forward()))
                .collect();
            // Identical carriers, native spans and boundary hemisphere. The
            // reversed loop chooses the complementary spherical region.
            assert!(super::sphere_great_circle_loop(&topo, sphere, &complement).unwrap());
            assert!(
                super::sphere_native_arcs_in_hemisphere(&topo, sphere, &complement, 1.0).unwrap()
            );
            assert!(
                super::sphere_hemisphere_chart(
                    &topo,
                    sphere,
                    &complement,
                    &boundary,
                    &mesh.positions
                )
                .unwrap()
                .is_none()
            );

            // Swapping stored endpoints/native bounds and reversing the
            // coedge preserves the original directed major/minor arcs.
            let descending: Vec<_> = edges
                .iter()
                .map(|use_| {
                    let e = topo.edge(use_.edge()).unwrap();
                    let (start, end, trim) = (e.start(), e.end(), e.trim().unwrap());
                    let e = topo.edge_mut(use_.edge()).unwrap();
                    e.set_start(end);
                    e.set_end(start);
                    e.set_trim(Some((trim.1, trim.0)));
                    OrientedEdge::new(use_.edge(), false)
                })
                .collect();
            assert!(super::sphere_great_circle_loop(&topo, sphere, &descending).unwrap());
            assert!(
                super::sphere_hemisphere_chart(
                    &topo,
                    sphere,
                    &descending,
                    &boundary,
                    &mesh.positions
                )
                .unwrap()
                .is_some()
            );
        }
    }
}
