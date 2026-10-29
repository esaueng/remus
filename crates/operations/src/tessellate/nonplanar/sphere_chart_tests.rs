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
    super::super::PointMergeMap,
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
    let mut map = super::super::PointMergeMap::default();
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
    let mut map = super::super::PointMergeMap::default();
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

fn holed_hemisphere(radius: f64, matrix: Mat4, side: f64, hole_z: f64) -> ChartFixture {
    holed_hemisphere_tilt(radius, matrix, side, hole_z, 0.0)
}

fn holed_hemisphere_tilt(
    radius: f64,
    matrix: Mat4,
    side: f64,
    hole_z: f64,
    tilt: f64,
) -> ChartFixture {
    let mut topo = Topology::new();
    let mut mesh = TriangleMesh {
        positions: vec![],
        normals: vec![],
        indices: vec![],
    };
    let mut pool = DetHashMap::default();
    let mut map = super::super::PointMergeMap::default();
    let center = matrix.mul_point(Point3::new(0.0, 0.0, 0.0));
    let mut wires = Vec::new();
    for (ring, z) in [0.0_f64, hole_z].into_iter().enumerate() {
        let angle = if ring == 0 { 0.0 } else { tilt };
        let axis = Vec3::new(angle.sin(), 0.0, angle.cos());
        let ring_radius = radius * (1.0 - z * z).sqrt();
        let circle = Circle3D::with_axes(
            matrix.mul_point(Point3::new(
                axis.x() * z * radius,
                0.0,
                axis.z() * z * radius,
            )),
            direction(matrix, axis),
            ring_radius,
            direction(matrix, Vec3::new(angle.cos(), 0.0, -angle.sin())),
            direction(matrix, Vec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
        let mut vertices = Vec::new();
        let mut ring_ids = Vec::new();
        for i in 0..64 {
            let p = circle.evaluate(std::f64::consts::TAU * f64::from(i) / 64.0);
            let id = mesh.positions.len() as u32;
            mesh.positions.push(p);
            mesh.normals.push((p - center) * (1.0 / radius));
            map.insert(point_merge_key(p, super::MERGE_GRID), id);
            ring_ids.push(id);
            if i % 16 == 0 {
                vertices.push(topo.add_vertex(Vertex::new(p, 1e-7)));
            }
        }
        let mut uses = Vec::new();
        for i in 0..4 {
            let mut edge = Edge::new(
                vertices[i],
                vertices[(i + 1) % 4],
                EdgeCurve::Circle(circle.clone()),
            );
            edge.set_trim(Some((
                std::f64::consts::FRAC_PI_2 * i as f64,
                std::f64::consts::FRAC_PI_2 * (i + 1) as f64,
            )));
            let id = topo.add_edge(edge);
            pool.insert(
                id.index(),
                (0..=16).map(|j| ring_ids[(i * 16 + j) % 64]).collect(),
            );
            uses.push(OrientedEdge::new(id, true));
        }
        // The inner wire has opposite traversal. A south hemisphere reverses
        // both loops, preserving the same retained latitude band.
        if (z != 0.0) == (side > 0.0) {
            uses = uses
                .into_iter()
                .rev()
                .map(|e| OrientedEdge::new(e.edge(), false))
                .collect();
        }
        wires.push(topo.add_wire(Wire::new(uses, true).unwrap()));
    }
    let sphere =
        SphericalSurface::with_axis(center, radius, direction(matrix, Vec3::new(0.0, 0.0, 1.0)))
            .unwrap();
    let face = Face::new(wires[0], vec![wires[1]], FaceSurface::Sphere(sphere));
    (topo, face, mesh, pool, map)
}

#[test]
fn holed_hemisphere_keeps_both_shared_loops_and_the_selected_side() {
    for radius in [0.001, 1.0, 1000.0] {
        for side in [-1.0, 1.0] {
            for (hole_height, tilt) in [(0.5, 0.0_f64), (0.8, 0.58)] {
                for deflection in [0.03, 0.003] {
                    let matrix = Mat4::translation(13.0 * radius, -7.0 * radius, 3.0 * radius)
                        * Mat4::rotation_y(0.53)
                        * Mat4::rotation_x(0.21);
                    let inverse = matrix.inverse().unwrap();
                    let (topo, face, mut mesh, pool, mut map) =
                        holed_hemisphere_tilt(radius, matrix, side, hole_height * side, tilt);
                    let positions = mesh.positions.clone();
                    let normals = mesh.normals.clone();
                    assert!(
                        super::tessellate_sphere_cap_shared(
                            &topo,
                            &face,
                            deflection * radius,
                            0.35,
                            false,
                            false,
                            &pool,
                            &mut mesh,
                            &mut map
                        )
                        .unwrap()
                    );
                    assert_eq!(&mesh.positions[..positions.len()], positions);
                    assert_eq!(&mesh.normals[..normals.len()], normals);
                    let mut uses = std::collections::BTreeMap::new();
                    let mut area = 0.0;
                    for t in mesh.indices.chunks_exact(3) {
                        let [a, b, c] = [t[0], t[1], t[2]].map(|i| {
                            (inverse.mul_point(mesh.positions[i as usize])
                                - Point3::new(0.0, 0.0, 0.0))
                                * (1.0 / radius)
                        });
                        let normal = (b - a).cross(c - a);
                        assert!(normal.dot((a + b + c) * (1.0 / 3.0)) > 0.0);
                        area += normal.length() * 0.5;
                        for p in [a, b, c] {
                            assert!((p.length() - 1.0).abs() < 1e-10);
                            assert!(p.z() * side >= -1e-10);
                            // The removed tilted cap is exactly n.p > hole_height.
                            // A linear halfspace check at all facet vertices also
                            // excludes every interior point of the flat triangle.
                            let axis = Vec3::new(tilt.sin(), 0.0, tilt.cos());
                            assert!(side * p.dot(axis) <= hole_height + 1e-10);
                        }
                        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                            uses.entry((a.min(b), a.max(b)))
                                .or_insert_with(Vec::new)
                                .push(a < b);
                        }
                    }
                    let boundary: std::collections::BTreeSet<_> = pool
                        .values()
                        .flat_map(|ids| ids.windows(2).map(|p| (p[0].min(p[1]), p[0].max(p[1]))))
                        .collect();
                    for edge in &boundary {
                        assert_eq!(uses.get(edge).unwrap().len(), 1, "shared segment {edge:?}");
                    }
                    for (edge, directions) in uses {
                        if !boundary.contains(&edge) {
                            assert_eq!(directions.len(), 2);
                            assert_ne!(directions[0], directions[1]);
                        }
                    }
                    // Hemisphere minus a cap of height (1-hole_height), independent
                    // of tilt while that entire cap remains inside the hemisphere.
                    let exact_area = std::f64::consts::TAU * hole_height;
                    assert!(
                        (area - exact_area).abs() < exact_area * deflection,
                        "area={area} expected={exact_area} tilt={tilt}"
                    );
                }
            }
        }
    }
}

#[test]
fn holed_hemisphere_defers_unsupported_boundaries_without_mutation() {
    for variant in 0..4 {
        let (mut topo, mut face, mut mesh, pool, mut map) = holed_hemisphere(
            1.0,
            Mat4::identity(),
            1.0,
            if variant == 0 { -0.5 } else { 0.5 },
        );
        if variant == 1 {
            face = Face::new(face.outer_wire(), vec![], face.surface().clone());
        }
        if variant == 2 {
            face = Face::new(
                face.outer_wire(),
                vec![face.inner_wires()[0]; 2],
                face.surface().clone(),
            );
        }
        if variant == 3 {
            let partial = topo.wire(face.outer_wire()).unwrap().edges()[..3].to_vec();
            let outer = topo.add_wire(Wire::new(partial, false).unwrap());
            face = Face::new(outer, face.inner_wires().to_vec(), face.surface().clone());
        }
        let before = (
            mesh.positions.clone(),
            mesh.normals.clone(),
            mesh.indices.clone(),
            map.clone(),
        );
        let FaceSurface::Sphere(sphere) = face.surface() else {
            panic!("sphere fixture")
        };
        assert!(
            !super::tessellate_sphere_holed_hemisphere_shared(
                &topo, &face, sphere, 0.01, 0.35, &pool, &mut mesh, &mut map
            )
            .unwrap()
        );
        assert_eq!((mesh.positions, mesh.normals, mesh.indices, map), before);
    }
}

#[test]
fn holed_hemisphere_budget_refusal_restores_the_attempt() {
    let (topo, face, mut mesh, pool, mut map) = holed_hemisphere(1.0, Mat4::identity(), 1.0, 0.5);
    let before = (
        mesh.positions.clone(),
        mesh.normals.clone(),
        mesh.indices.clone(),
        map.clone(),
    );
    let FaceSurface::Sphere(sphere) = face.surface() else {
        panic!("sphere fixture")
    };
    let result = super::tessellate_sphere_holed_hemisphere_shared(
        &topo, &face, sphere, 1e-14, 0.0, &pool, &mut mesh, &mut map,
    );
    assert!(
        matches!(result,Err(crate::OperationsError::InvalidInput{reason}) if reason.contains("work limit"))
    );
    assert_eq!((mesh.positions, mesh.normals, mesh.indices, map), before);
}

/// opseq81 two-sphere fuse: both hole-free sphere remainders take the
/// stereographic hemisphere chart. Its interior grid once spread a fixed
/// `2 * angular_segments` samples per axis over the patch's chart bounding
/// box, ignoring the chart's up-to-twofold angular stretch and the patch's
/// own extent, so the solid meshed at about 2.5 times the triangles it needs
/// (112,894 here; 2,672,542 at the repro's `diag * 1e-5` deflection, where
/// it dominated a CI timeout). Sizing the grid in arc angle keeps the mesh
/// watertight with the same volume accuracy at well under that count.
#[test]
fn sphere_fuse_hemisphere_chart_grid_follows_arc_angle() {
    let mut topo = Topology::new();
    let small = crate::primitives::make_sphere(&mut topo, 4.5, 16).unwrap();
    let large = crate::primitives::make_sphere(&mut topo, 5.5, 16).unwrap();
    let placement =
        Mat4::translation(0.0, 1.0, -4.0) * Mat4::rotation_x(std::f64::consts::FRAC_PI_4);
    crate::transform::transform_solid(&mut topo, small, &placement).unwrap();
    let fused =
        crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, small, large).unwrap();

    let deflection = 5e-3;
    let mesh = super::super::tessellate_solid(&topo, fused, deflection).unwrap();
    assert_eq!(super::super::boundary_edge_count(&mesh), 0);
    assert_eq!(super::super::non_manifold_edge_count(&mesh), 0);

    let triangles = mesh.indices.len() / 3;
    assert!(
        triangles < 64_000,
        "sphere-fuse mesh has {triangles} triangles; the arc-angle grid gives about 45,000"
    );

    // Independent overlapping-sphere union volume (opseq81 bundle). An
    // inscribed mesh loses at most `deflection` times the surface area.
    let volume = mesh
        .indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [0, 1, 2].map(|k| {
                let p = mesh.positions[t[k] as usize];
                Vec3::new(p.x(), p.y(), p.z())
            });
            a.dot(b.cross(c))
        })
        .sum::<f64>()
        / 6.0;
    let exact = 867.071_429_497_175_1;
    let area_bound = 4.0 * std::f64::consts::PI * (4.5_f64.powi(2) + 5.5_f64.powi(2));
    assert!(
        volume <= exact && exact - volume <= deflection * area_bound,
        "mesh volume {volume} against exact {exact}"
    );
}
