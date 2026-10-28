//! B61 ready-repro: the variable-radius fillet closes each stripe end with a
//! straight chord, so its blend face is bounded by edges that do not lie on
//! it.
//!
//! Found diagnosing B63's in-range volume gap (2026-09-25): a constant-law
//! radius-9 stripe on one vertical edge of a 10³ box measures 824.41 on its
//! mesh at every deflection (0.1, 0.01, 0.001) and 749.12 by Gauss, against
//! the closed form 826.1725 that the walking engine hits to 1e-10. The blend
//! wall itself is exact — its samples sit 1.8e-15 from the rolling-ball axis —
//! but the assembler mints `Line` edges between the vertex positions of every
//! spec, so the wall's two end boundaries and the matching cap-face edges are
//! chords of the quarter circle, `r(1 − cos 45°)` = 2.64 away from the wall
//! at their midpoints. `validateSolid` accepts it. The gap scales with r²
//! (0.0083 on the mesh at r = 1), which is why every earlier variable-fillet
//! check passed inside its tolerance.
//!
//! Acceptance: the stripe's end boundaries lie on the blend wall, and the
//! body measures the closed form by Gauss and on its mesh.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::classify::{PointClassification, classify_points};
use remus_operations::fillet::{FilletEdgeSetback, FilletRadiusLaw, fillet_variable_with_setbacks};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_with_tolerance,
};
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;

#[test]
fn variable_fillet_end_boundaries_lie_on_the_blend_wall() {
    let radius = 9.0;
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let top = Point3::new(0.0, 0.0, 10.0);
    let edge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let data = topo.edge(edge).unwrap();
            let a = topo.vertex(data.start()).unwrap().point();
            let b = topo.vertex(data.end()).unwrap().point();
            ((a - origin).length() < 1e-9 && (b - top).length() < 1e-9)
                || ((a - top).length() < 1e-9 && (b - origin).length() < 1e-9)
        })
        .expect("box edge through the origin along z");

    let result = fillet_variable_with_setbacks(
        &mut topo,
        solid,
        &[FilletEdgeSetback {
            edge,
            law: FilletRadiusLaw::Constant(radius),
            start_setback: 0.0,
            end_setback: 0.0,
        }],
    )
    .expect("radius 9 fits the 10-wide support faces");

    // Every boundary edge of the blend wall lies on it: sample each edge and
    // measure its distance from the rolling-ball axis x = y = r.
    let wall = solid_faces(&topo, result)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
        .expect("the variable blend wall");
    let wire = topo.wire(topo.face(wall).unwrap().outer_wire()).unwrap();
    let mut worst: f64 = 0.0;
    for oriented in wire.edges() {
        let data = topo.edge(oriented.edge()).unwrap();
        let a = topo.vertex(data.start()).unwrap().point();
        let b = topo.vertex(data.end()).unwrap().point();
        let (t0, t1) = data.strict_domain().unwrap();
        for i in 0..=16 {
            let t = (t1 - t0).mul_add(f64::from(i) / 16.0, t0);
            let p = data.curve().evaluate_with_endpoints(t, a, b);
            let distance = (p.x() - radius).hypot(p.y() - radius);
            worst = worst.max((distance - radius).abs());
        }
    }
    assert!(
        worst < 1e-7,
        "a blend-wall boundary edge leaves the wall by {worst:.3e}"
    );

    let exact = (1.0 - std::f64::consts::FRAC_PI_4).mul_add(-10.0 * radius * radius, 1000.0);
    let gauss = mass_properties(&topo, result).unwrap().mass;
    assert!(
        (gauss - exact).abs() < 1e-6 * exact,
        "Gauss {gauss:.6} against the closed form {exact:.6}"
    );
    let mesh = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (mesh - exact).abs() < 1e-3 * exact,
        "mesh {mesh:.6} against the closed form {exact:.6}"
    );
}

#[test]
fn variable_fillet_shared_boundaries_and_quantities_survive_scale_and_placement() {
    for scale in [0.001, 1.0, 1000.0] {
        for radius_ratio in [1.0, 9.0] {
            for placed in [false, true] {
                let size = 10.0 * scale;
                let radius = radius_ratio * scale;
                let mut topo = Topology::new();
                let input = make_box(&mut topo, size, size, size).unwrap();
                let edge = solid_edges(&topo, input)
                    .unwrap()
                    .into_iter()
                    .find(|&id| {
                        let edge = topo.edge(id).unwrap();
                        let a = topo.vertex(edge.start()).unwrap().point();
                        let b = topo.vertex(edge.end()).unwrap().point();
                        a.x() == 0.0 && a.y() == 0.0 && b.x() == 0.0 && b.y() == 0.0
                    })
                    .unwrap();
                let placement = if placed {
                    Mat4::translation(37.0 * scale, -19.0 * scale, 11.0 * scale)
                        * Mat4::rotation_y(0.31)
                        * Mat4::rotation_z(-0.47)
                } else {
                    Mat4::identity()
                };
                remus_operations::transform::transform_solid(&mut topo, input, &placement).unwrap();
                let before = remus_io::arena_io::serialize_solids(&topo, &[input]).unwrap();
                let result = fillet_variable_with_setbacks(
                    &mut topo,
                    input,
                    &[FilletEdgeSetback {
                        edge,
                        law: FilletRadiusLaw::Constant(radius),
                        start_setback: 0.0,
                        end_setback: 0.0,
                    }],
                )
                .unwrap();
                assert_eq!(
                    remus_io::arena_io::serialize_solids(&topo, &[input]).unwrap(),
                    before,
                    "the original operand must be unchanged"
                );
                assert!(
                    remus_operations::validate::validate_solid(&topo, result)
                        .unwrap()
                        .is_valid()
                );
                let expected =
                    size.powi(3) - (1.0 - std::f64::consts::FRAC_PI_4) * radius * radius * size;
                let gauss = mass_properties(&topo, result).unwrap().mass;
                assert!(
                    (gauss - expected).abs() / expected < 1e-6,
                    "scale={scale}, r={radius_ratio}, placed={placed}: {gauss} vs {expected}"
                );
                let adjacency = topo.build_adjacency(result).unwrap();
                let walls: Vec<_> = solid_faces(&topo, result)
                    .unwrap()
                    .into_iter()
                    .filter(|&face| {
                        matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_))
                    })
                    .collect();
                assert_eq!(walls.len(), 1);
                let certificates = remus_topology::validation::validate_solid_pcurve_contracts(
                    &topo, result, 1e-7, 0,
                )
                .unwrap();
                assert_eq!(certificates.stored_pcurves, 8);
                assert_eq!(certificates.validated_uses, 8);
                let wall = topo.face(walls[0]).unwrap();
                let axis_origin = placement.mul_point(Point3::new(radius, radius, 0.0));
                let axis = (placement.mul_point(Point3::new(radius, radius, size)) - axis_origin)
                    .normalize()
                    .unwrap();
                for use_ in topo.wire(wall.outer_wire()).unwrap().edges() {
                    let edge = topo.edge(use_.edge()).unwrap();
                    assert_eq!(adjacency.faces_for_edge(use_.edge()).len(), 2);
                    for &incident in adjacency.faces_for_edge(use_.edge()) {
                        let data = topo.face(incident).unwrap();
                        let usage = topo
                            .wire(data.outer_wire())
                            .unwrap()
                            .edges()
                            .iter()
                            .find(|usage| usage.edge() == use_.edge())
                            .unwrap();
                        let pc = topo
                            .pcurve_oriented(use_.edge(), incident, usage.is_forward())
                            .unwrap();
                        // Independently lift the per-use curve, including its
                        // orientation, rather than treating the certificate
                        // implementation as the geometric oracle.
                        for step in 0..=16 {
                            let fraction = f64::from(step) / 16.0;
                            let pt = pc.t_start() + (pc.t_end() - pc.t_start()) * fraction;
                            let uv = pc.evaluate(pt);
                            let lifted = match data.surface() {
                                FaceSurface::Plane { normal, d } => {
                                    let origin = Point3::new(
                                        normal.x() * d / normal.dot(*normal),
                                        normal.y() * d / normal.dot(*normal),
                                        normal.z() * d / normal.dot(*normal),
                                    );
                                    let n = normal.normalize().unwrap();
                                    let seed = if n.x().abs() < 0.9 {
                                        remus_math::vec::Vec3::new(1.0, 0.0, 0.0)
                                    } else {
                                        remus_math::vec::Vec3::new(0.0, 1.0, 0.0)
                                    };
                                    let u = n.cross(seed).normalize().unwrap();
                                    origin + u * uv.x() + n.cross(u) * uv.y()
                                }
                                surface => surface.evaluate(uv.x(), uv.y()).unwrap(),
                            };
                            let (a, b) = edge.strict_domain().unwrap();
                            let native = if usage.is_forward() {
                                a + (b - a) * fraction
                            } else {
                                b + (a - b) * fraction
                            };
                            let p = edge.curve().evaluate_with_endpoints(
                                native,
                                topo.vertex(edge.start()).unwrap().point(),
                                topo.vertex(edge.end()).unwrap().point(),
                            );
                            assert!((lifted - p).length() < 1e-7);
                        }
                    }
                    let a = topo.vertex(edge.start()).unwrap().point();
                    let b = topo.vertex(edge.end()).unwrap().point();
                    let (start, end) = edge.strict_domain().unwrap();
                    for step in 0..=16 {
                        let t = (end - start).mul_add(f64::from(step) / 16.0, start);
                        let p = edge.curve().evaluate_with_endpoints(t, a, b);
                        let delta = p - axis_origin;
                        let radial = (delta - axis * delta.dot(axis)).length();
                        assert!((radial - radius).abs() < 1e-7);
                    }
                }
                for deflection in [0.01 * scale, 0.001 * scale] {
                    let mesh =
                        tessellate_solid_with_tolerance(&topo, result, deflection, 0.1).unwrap();
                    assert_eq!(boundary_edge_count(&mesh), 0);
                    assert_eq!(non_manifold_edge_count(&mesh), 0);
                    let mesh_volume = solid_volume(&topo, result, deflection).unwrap();
                    assert!((mesh_volume - expected).abs() / expected < 1e-3);
                }
                let probes = [
                    Point3::new(0.15 * radius, 0.15 * radius, size / 2.0),
                    Point3::new(0.8 * radius, 0.8 * radius, size / 2.0),
                    Point3::new(0.95 * size, 0.95 * size, size / 2.0),
                ]
                .map(|p| placement.mul_point(p));
                assert_eq!(
                    classify_points(&topo, result, &probes, 0.01 * scale, 1e-7).unwrap(),
                    [
                        PointClassification::Outside,
                        PointClassification::Inside,
                        PointClassification::Inside
                    ]
                );
            }
        }
    }
}

#[test]
fn variable_laws_keep_curved_support_contacts_and_match_integrated_radius_oracles() {
    for (law, mean_square_radius) in [
        (
            FilletRadiusLaw::Linear {
                start: 1.0,
                end: 3.0,
            },
            13.0 / 3.0,
        ),
        (
            FilletRadiusLaw::SCurve {
                start: 1.0,
                end: 3.0,
            },
            3.0 + 52.0 / 35.0,
        ),
    ] {
        let mut topo = Topology::new();
        let input = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edge = solid_edges(&topo, input)
            .unwrap()
            .into_iter()
            .find(|&id| {
                let edge = topo.edge(id).unwrap();
                let a = topo.vertex(edge.start()).unwrap().point();
                let b = topo.vertex(edge.end()).unwrap().point();
                a.x() == 0.0 && a.y() == 0.0 && b.x() == 0.0 && b.y() == 0.0
            })
            .unwrap();
        let result = fillet_variable_with_setbacks(
            &mut topo,
            input,
            &[FilletEdgeSetback {
                edge,
                law,
                start_setback: 0.0,
                end_setback: 0.0,
            }],
        )
        .unwrap();
        // Integrate r(z)^2 independently: linear's mean is
        // (a²+ab+b²)/3; smoothstep's is a²+a(b-a)+13(b-a)²/35.
        let expected = 1000.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 10.0 * mean_square_radius;
        let gauss = mass_properties(&topo, result).unwrap().mass;
        assert!(
            (gauss - expected).abs() < 1e-6 * expected,
            "{gauss} vs {expected}"
        );
        assert!((solid_volume(&topo, result, 0.001).unwrap() - expected).abs() < 1e-3 * expected);
    }
}
