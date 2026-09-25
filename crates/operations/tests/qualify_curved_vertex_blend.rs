//! Exact two-ridge corner of a right quarter-cylinder, including its sharp-edge ledge.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{fillet_v2, fillet_with_evolution};
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{EdgeId, SolidId, Topology};

fn fixture(topo: &mut Topology, scale: f64, height: f64, angle: f64) -> (SolidId, Vec<EdgeId>) {
    let radius = 5.0 * scale;
    let origin = Point3::new(0.0, 0.0, 0.0);
    let circle = Circle3D::new_with_ref(
        origin,
        Vec3::new(0.0, 0.0, 1.0),
        radius,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let o = topo.add_vertex(Vertex::new(origin, 1e-7));
    let x = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let y = topo.add_vertex(Vertex::new(circle.evaluate(angle), 1e-7));
    let line1 = topo.add_edge(Edge::new(o, x, EdgeCurve::Line));
    let mut arc = Edge::new(x, y, EdgeCurve::Circle(circle));
    arc.set_trim(Some((0.0, angle)));
    let arc = topo.add_edge(arc);
    let line2 = topo.add_edge(Edge::new(y, o, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            [line1, arc, line2]
                .map(|edge| OrientedEdge::new(edge, true))
                .to_vec(),
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let solid = extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), height * scale).unwrap();
    let vertex = solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| {
            (topo.vertex(id).unwrap().point() - Point3::new(radius, 0.0, height * scale)).length()
                < 1e-7
        })
        .unwrap();
    let edges: Vec<_> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            matches!(edge.curve(), EdgeCurve::Line) && [edge.start(), edge.end()].contains(&vertex)
        })
        .collect();
    assert_eq!(edges.len(), 2);
    (solid, edges)
}

fn mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    let reference = mesh.positions[0];
    mesh.indices
        .chunks_exact(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize] - reference;
            let b = mesh.positions[tri[1] as usize] - reference;
            let c = mesh.positions[tri[2] as usize] - reference;
            a.dot(b.cross(c)) / 6.0
        })
        .sum::<f64>()
        .abs()
}

// Integrate the circular cross-sections of the two bands, ball and radial ledge independently.
fn exact_volume() -> f64 {
    let radius: f64 = 5.0;
    let r: f64 = 0.25;
    let height = 10.0;
    let x = (radius * radius - 2.0 * radius * r).sqrt();
    let sine = r / (radius - r);
    let cosine = x / (radius - r);
    let angle = sine.asin();
    let tail = radius * radius * (std::f64::consts::FRAC_PI_4 - 0.5 * (sine * cosine + angle));
    let constant = tail + 0.5 * x / r * ((radius * sine).powi(2) - r * r);
    let bottom_area = constant + x * r + 0.5 * r * r * (angle + std::f64::consts::FRAC_PI_2);
    bottom_area * height
        - x * r * r * (1.0 - std::f64::consts::FRAC_PI_4)
        - r.powi(3) * (angle + std::f64::consts::FRAC_PI_2) / 6.0
}

fn assert_boundary_uses_and_tangency(topo: &Topology, solid: SolidId, radius: f64) {
    let adjacency = topo.build_adjacency(solid).unwrap();
    remus_topology::validation::validate_solid_pcurve_contracts(topo, solid, 1e-7, 32).unwrap();
    for face in solid_faces(topo, solid).unwrap() {
        remus_topology::validation::validate_face_loops(topo, face).unwrap();
        for &boundary in topo.loops_of_face(face).unwrap() {
            remus_topology::validation::validate_loop_connected(topo, boundary).unwrap();
        }
    }
    let mut smooth_edges = 0;
    for edge in solid_edges(topo, solid).unwrap() {
        let faces = adjacency.faces_for_edge(edge);
        assert_eq!(faces.len(), 2);
        let first = topo.face(faces[0]).unwrap().surface();
        let second = topo.face(faces[1]).unwrap().surface();
        let is_band = |surface: &FaceSurface| matches!(surface,FaceSurface::Cylinder(cylinder) if (cylinder.radius()-radius).abs()<1e-7);
        let tangent_pair = |a: &FaceSurface, b: &FaceSurface| {
            if !is_band(a) {
                return false;
            }
            match b {
                FaceSurface::Sphere(_) | FaceSurface::Cylinder(_) => true,
                FaceSurface::Plane { normal, .. } => {
                    let FaceSurface::Cylinder(cylinder) = a else {
                        unreachable!()
                    };
                    normal.dot(cylinder.axis()).abs() < 1e-10
                }
                _ => false,
            }
        };
        let smooth = tangent_pair(first, second) || tangent_pair(second, first);
        if smooth {
            smooth_edges += 1;
        }
        let data = topo.edge(edge).unwrap();
        let (t0, t1) = data.strict_domain().unwrap();
        let start = topo.vertex(data.start()).unwrap().point();
        let end = topo.vertex(data.end()).unwrap().point();
        for station in 0..=32 {
            let point = data.curve().evaluate_with_endpoints(
                t0 + (t1 - t0) * f64::from(station) / 32.0,
                start,
                end,
            );
            let normal = |surface: &FaceSurface| {
                if let FaceSurface::Plane { normal, d } = surface {
                    assert!((normal.dot(point - Point3::new(0.0, 0.0, 0.0)) - *d).abs() < 1e-7);
                    *normal
                } else {
                    let (u, v) = surface.project_point(point).unwrap();
                    assert!((surface.evaluate(u, v).unwrap() - point).length() < 1e-7);
                    surface.normal(u, v)
                }
            };
            let a = normal(first);
            let b = normal(second);
            if smooth {
                assert!(a.dot(b) > 1.0 - 1e-10, "edge {edge:?}, station {station}");
            }
        }
    }
    assert_eq!(smooth_edges, 6);
}

#[test]
#[allow(clippy::too_many_lines)]
fn curved_two_ridge_corner_is_exact_closed_and_order_independent() {
    for scale in [1e-3, 1.0, 1e3] {
        for placed in [false, true] {
            let mut volumes = Vec::new();
            for reverse in [false, true] {
                let mut topo = Topology::new();
                let (source, mut edges) =
                    fixture(&mut topo, scale, 10.0, std::f64::consts::FRAC_PI_2);
                if placed {
                    let matrix = Mat4::translation(17.0 * scale, -11.0 * scale, 7.0 * scale)
                        * Mat4::rotation_x(0.4)
                        * Mat4::rotation_y(-0.7);
                    transform_solid(&mut topo, source, &matrix).unwrap();
                }
                if reverse {
                    edges.reverse();
                }
                let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
                let (result, history) =
                    fillet_with_evolution(&mut topo, source, &edges, 0.25 * scale).unwrap();
                assert!(!result.is_partial);
                assert!(result.failed.is_empty());
                assert_eq!(result.succeeded, edges);
                assert_eq!(
                    (
                        solid_faces(&topo, result.solid).unwrap().len(),
                        solid_edges(&topo, result.solid).unwrap().len(),
                        solid_vertices(&topo, result.solid).unwrap().len()
                    ),
                    (9, 18, 11)
                );
                let faces = solid_faces(&topo, result.solid).unwrap();
                assert!(history.origin.is_exact());
                assert!(
                    history
                        .completeness_for_result(faces.iter().map(|id| id.index()))
                        .is_resolved()
                );
                let mut types = (0, 0, 0);
                for face in faces {
                    match topo.face(face).unwrap().surface() {
                        FaceSurface::Plane { .. } => types.0 += 1,
                        FaceSurface::Cylinder(_) => types.1 += 1,
                        FaceSurface::Sphere(sphere) => {
                            types.2 += 1;
                            assert!((sphere.radius() - 0.25 * scale).abs() < 1e-7);
                        }
                        other => panic!("unexpected carrier {other:?}"),
                    }
                }
                assert_eq!(types, (5, 3, 1));
                let report =
                    remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
                assert!(report.is_valid(), "{report:?}");
                let adjacency = topo.build_adjacency(result.solid).unwrap();
                assert!(adjacency.boundary_edges().is_empty());
                assert!(adjacency.non_manifold_edges().is_empty());
                assert_boundary_uses_and_tangency(&topo, result.solid, 0.25 * scale);
                let expected = exact_volume() * scale.powi(3);
                let mut measurements = Vec::new();
                for deflection in [0.02 * scale, 0.005 * scale] {
                    let mesh = tessellate_solid(&topo, result.solid, deflection).unwrap();
                    let quality = welded_mesh_quality(&mesh);
                    assert!(
                        quality.is_watertight(),
                        "scale {scale}, placed {placed}, {quality:?}"
                    );
                    let volume = solid_volume(&topo, result.solid, deflection).unwrap();
                    let mesh_volume = mesh_volume(&mesh);
                    assert!(volume < 62.5 * std::f64::consts::PI * scale.powi(3));
                    assert!(
                        (volume - expected).abs() / expected < 2e-4,
                        "scale {scale}: volume {volume}, oracle {expected}"
                    );
                    assert!(
                        (mesh_volume - expected).abs() / expected < 0.003,
                        "mesh {mesh_volume}, oracle {expected}"
                    );
                    measurements.push(volume);
                }
                volumes.push(measurements);
                assert_eq!(
                    remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
                    before
                );
            }
            for i in 0..2 {
                assert!((volumes[0][i] - volumes[1][i]).abs() / volumes[0][i] < 1e-10);
            }
        }
    }
}

#[test]
fn unsupported_curved_corner_layouts_and_setbacks_refuse_atomically() {
    for (height, angle, radius) in [
        (10.0, std::f64::consts::FRAC_PI_2, 2.6),
        (0.2, std::f64::consts::FRAC_PI_2, 0.25),
        (10.0, std::f64::consts::FRAC_PI_3, 0.25),
    ] {
        let mut topo = Topology::new();
        let (source, edges) = fixture(&mut topo, 1.0, height, angle);
        let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
        let counts = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_solids(),
        );
        let error = fillet_v2(&mut topo, source, &edges, radius)
            .err()
            .expect("unqualified corner must refuse");
        assert!(
            matches!(
                error,
                remus_operations::OperationsError::Blend(
                    remus_blend::BlendError::RadiusTooLarge { .. }
                        | remus_blend::BlendError::UnsupportedVertexBlend { .. }
                )
            ),
            "{error:?}"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
            before
        );
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_faces(),
                topo.num_solids()
            ),
            counts
        );
    }
}

#[test]
fn curved_corner_refuses_unequal_laws_and_inconsistent_source_trims() {
    for invalid_trim in [false, true] {
        let mut topo = Topology::new();
        let (source, edges) = fixture(&mut topo, 1.0, 10.0, std::f64::consts::FRAC_PI_2);
        if invalid_trim {
            let arc = solid_edges(&topo, source)
                .unwrap()
                .into_iter()
                .find(|&id| matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Circle(_)))
                .unwrap();
            topo.edge_mut(arc)
                .unwrap()
                .set_trim(Some((0.0, std::f64::consts::FRAC_PI_3)));
        }
        let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
        let counts = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_solids(),
        );
        let mut builder = remus_blend::fillet_builder::FilletBuilder::new(&mut topo, source);
        builder.add_edges(&edges[..1], 0.25);
        builder.add_edges(&edges[1..], if invalid_trim { 0.25 } else { 0.3 });
        assert!(matches!(
            builder.build(),
            Err(remus_blend::BlendError::UnsupportedVertexBlend { .. })
        ));
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
            before
        );
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_faces(),
                topo.num_solids()
            ),
            counts
        );
    }
}
