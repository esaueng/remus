//! Exact one-rim cone notches and refusal of pinched side-wall contacts.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
    vec::Point3,
};
use remus_operations::{
    boolean::{BooleanOp, BooleanQuality, boolean_with_context},
    primitives::{make_box, make_cone},
    transform::transform_solid,
};
use remus_topology::{Topology, solid::SolidId};
use std::f64::consts::PI;

fn operands(up: bool, distance: f64, z: f64, placement: &Mat4) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let cone = if up {
        make_cone(&mut topo, 0.5, 0.0, 4.0)
    } else {
        make_cone(&mut topo, 0.0, 0.5, 4.0)
    }
    .unwrap();
    transform_solid(&mut topo, cone, &Mat4::translation(distance, 1.5, z)).unwrap();
    transform_solid(&mut topo, block, placement).unwrap();
    transform_solid(&mut topo, cone, placement).unwrap();
    for solid in [block, cone] {
        let report = validate_solid(&topo, solid, &ValidateOptions::default()).unwrap();
        assert!(report.is_valid(), "input: {:?}", report.issues);
    }
    (topo, block, cone)
}
fn snapshot(topo: &Topology, solid: SolidId) -> Vec<u8> {
    remus_io::arena_io::serialize_solid(topo, solid).unwrap()
}
fn exact_cut(
    topo: &mut Topology,
    block: SolidId,
    cone: SolidId,
) -> Result<remus_operations::boolean::BooleanOutcome, remus_operations::OperationsError> {
    let counts = [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_solids(),
        topo.num_shells(),
        topo.num_coedges(),
        topo.num_loops(),
        topo.num_pcurves(),
        topo.journal().len(),
    ];
    let before = [snapshot(topo, block), snapshot(topo, cone)];
    let result = boolean_with_context(
        topo,
        BooleanOp::Cut,
        block,
        cone,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    );
    assert_eq!(
        before,
        [snapshot(topo, block), snapshot(topo, cone)],
        "operands must remain unchanged"
    );
    if result.is_err() {
        assert_eq!(
            counts,
            [
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_solids(),
                topo.num_shells(),
                topo.num_coedges(),
                topo.num_loops(),
                topo.num_pcurves(),
                topo.journal().len()
            ],
            "refusal must roll back all live topology"
        );
    }
    result
}
/// Integrate circular-segment area over the cone's height. The part beyond
/// x=0 is H/(3R) [R³ acos(a/R) - 2aR sqrt(R²-a²) + a³ acosh(R/a)].
fn retained_volume(distance: f64) -> f64 {
    let r = 0.5_f64;
    let h = 4.0;
    let a = distance;
    let outside = h / (3.0 * r)
        * (r.powi(3) * (a / r).acos() - 2.0 * a * r * (r * r - a * a).sqrt()
            + a.powi(3) * (r / a).acosh());
    24.0 - PI * r * r * h / 3.0 + outside
}

#[test]
fn crossing_side_wall_keeps_exact_cone_and_material() {
    let _ = env_logger::try_init();
    for up in [true, false] {
        for distance in [0.25, 0.4] {
            for scale in [0.5, 1.0, 2.0] {
                let placement =
                    Mat4::translation(13.0, -7.0, 9.0) * Mat4::scale(scale, scale, scale);
                let (mut topo, block, cone) = operands(up, distance, 0.0, &placement);
                let outcome = exact_cut(&mut topo, block, cone)
                    .expect("one-rim crossing has an exact solution");
                assert_eq!(outcome.quality, BooleanQuality::Exact);
                let report =
                    validate_solid(&topo, outcome.solid, &ValidateOptions::default()).unwrap();
                assert!(
                    report.is_valid(),
                    "{up} {distance} {scale}: {:?}",
                    report.issues
                );
                let faces = remus_topology::explorer::solid_faces(&topo, outcome.solid).unwrap();
                assert!(faces.len() < 12);
                assert!(faces.iter().any(|&f| matches!(
                    topo.face(f).unwrap().surface(),
                    remus_topology::face::FaceSurface::Cone(_)
                )));
                for &face in &faces {
                    if matches!(
                        topo.face(face).unwrap().surface(),
                        remus_topology::face::FaceSurface::Cone(_)
                    ) {
                        let mesh =
                            remus_operations::tessellate::tessellate(&topo, face, 0.002 * scale)
                                .unwrap();
                        assert!(!mesh.indices.is_empty());
                        for point in &mesh.positions {
                            assert!(
                                point.x() >= 13.0 - 1e-7,
                                "standalone face skins discarded notch: {point:?}"
                            );
                        }
                    }
                }
                // Default native mass integration resolves the sampled trim
                // polygon. Keep that existing accuracy contract separate from
                // the exact rational boundary and analytic material probes.
                let volume = remus_operations::measure::mass_properties(&topo, outcome.solid)
                    .unwrap()
                    .mass
                    / scale.powi(3);
                assert!(
                    (volume - retained_volume(distance)).abs() < 5e-4,
                    "volume {volume} != {}",
                    retained_volume(distance)
                );
                for deflection in [0.01, 0.002] {
                    let mesh = remus_operations::tessellate::tessellate_solid(
                        &topo,
                        outcome.solid,
                        deflection * scale,
                    )
                    .unwrap();
                    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
                    assert!(
                        quality.is_watertight(),
                        "{up} {distance} {scale}: {quality:?}"
                    );
                    assert_eq!(quality.euler_characteristic, 2);
                    assert_eq!(
                        remus_operations::tessellate::boundary_edge_count(&mesh),
                        0,
                        "{up} {distance} {scale}: inconsistent triangle winding"
                    );
                }
                let local_z = |z| if up { z } else { 4.0 - z };
                for (p, expected) in [
                    (
                        Point3::new(distance, 1.5, local_z(2.0)),
                        PointClassification::Outside,
                    ),
                    (
                        Point3::new(0.05, 1.5, local_z(0.25)),
                        PointClassification::Outside,
                    ),
                    (
                        Point3::new(0.05, 1.5, local_z(3.75)),
                        PointClassification::Inside,
                    ),
                    (Point3::new(1.8, 2.8, 2.0), PointClassification::Inside),
                    (Point3::new(-0.1, 1.5, 2.0), PointClassification::Outside),
                ] {
                    assert_eq!(
                        classify_point(
                            &topo,
                            outcome.solid,
                            placement.mul_point(p),
                            &ClassifyOptions::default()
                        )
                        .unwrap(),
                        expected,
                        "{up} {distance} {scale}: {p:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn tangent_opening_fails_closed_without_changing_operands() {
    for up in [true, false] {
        // Full cone tangent at its base on the floor/ceiling; also a cone
        // clipped at half height, whose opening is tangent there.
        for (distance, z) in [(0.5, 0.0), (0.25, if up { -2.0 } else { 2.0 })] {
            let (mut topo, block, cone) = operands(up, distance, z, &Mat4::identity());
            let error =
                exact_cut(&mut topo, block, cone).expect_err("pinched opening must not be Exact");
            assert!(matches!(
                error,
                remus_operations::OperationsError::ExactOnlyUnattainable
            ));
        }
    }
}

#[test]
fn unsupported_apex_plane_crossing_stays_fail_closed() {
    for up in [true, false] {
        let (mut topo, block, cone) = operands(up, 0.0, 0.0, &Mat4::identity());
        assert!(matches!(
            exact_cut(&mut topo, block, cone),
            Err(remus_operations::OperationsError::ExactOnlyUnattainable)
        ));
    }
}

#[test]
fn clear_side_wall_still_accepts_exact_cut() {
    for up in [true, false] {
        let (mut topo, block, cone) = operands(up, 0.50002, 0.0, &Mat4::identity());
        let outcome = exact_cut(&mut topo, block, cone).unwrap();
        assert_eq!(outcome.quality, BooleanQuality::Exact);
        assert!(
            validate_solid(&topo, outcome.solid, &ValidateOptions::default())
                .unwrap()
                .is_valid()
        );
        let volume = remus_operations::measure::mass_properties(&topo, outcome.solid)
            .unwrap()
            .mass;
        assert!((volume - (24.0 - PI / 3.0)).abs() < 1e-9);
    }
}

/// The transformed rim subdivision used to report a crossing here: a section
/// piece ending at its closed loop's closure point re-projected to the loop's
/// other end and traced the complementary arc, and the native validation
/// error refused the cut. With the piece's own parameter kept the cut is
/// exact and valid, and its tessellated volume extrapolates to the closed
/// form (the face integrator reads it about 3e-5 high, so the volume is
/// judged on the mesh).
#[test]
fn rotated_crossing_cuts_exactly() {
    let scale = 0.5_f64;
    let placement = Mat4::translation(13.0, -7.0, 9.0)
        * Mat4::rotation_y(0.31)
        * Mat4::rotation_z(0.27)
        * Mat4::scale(scale, scale, scale);
    let inverse = placement.inverse().unwrap();
    for up in [true, false] {
        for distance in [0.25, 0.4] {
            let (mut topo, block, cone) = operands(up, distance, 0.0, &placement);
            let outcome = exact_cut(&mut topo, block, cone)
                .expect("rotated one-rim crossing has an exact solution");
            assert_eq!(outcome.quality, BooleanQuality::Exact);
            let report = validate_solid(&topo, outcome.solid, &ValidateOptions::default()).unwrap();
            assert!(report.is_valid(), "{up} {distance}: {:?}", report.issues);
            let faces = remus_topology::explorer::solid_faces(&topo, outcome.solid).unwrap();
            assert!(faces.len() < 12);
            assert!(faces.iter().any(|&f| matches!(
                topo.face(f).unwrap().surface(),
                remus_topology::face::FaceSurface::Cone(_)
            )));
            for &face in &faces {
                if matches!(
                    topo.face(face).unwrap().surface(),
                    remus_topology::face::FaceSurface::Cone(_)
                ) {
                    let mesh = remus_operations::tessellate::tessellate(&topo, face, 0.002 * scale)
                        .unwrap();
                    assert!(!mesh.indices.is_empty());
                    for point in &mesh.positions {
                        let local = inverse.mul_point(*point);
                        assert!(
                            local.x() >= -1e-7,
                            "{up} {distance}: discarded notch skin: {local:?}"
                        );
                    }
                }
            }

            // Cross-check the independent segment integral against its closed form.
            let n = 20_000;
            let removed: f64 = (0..n)
                .map(|i| {
                    let z = (f64::from(i) + 0.5) / f64::from(n) * 4.0;
                    let r = 0.5 * (1.0 - z / 4.0);
                    let segment = if r > distance {
                        r * r * (distance / r).acos()
                            - distance * (r * r - distance * distance).sqrt()
                    } else {
                        0.0
                    };
                    (PI * r * r - segment) * 4.0 / f64::from(n)
                })
                .sum();
            let expected = retained_volume(distance) * scale.powi(3);
            assert!(((24.0 - removed) * scale.powi(3) - expected).abs() < 1e-8);
            let mesh_volume = |deflection: f64| {
                let mesh = remus_operations::tessellate::tessellate_solid(
                    &topo,
                    outcome.solid,
                    deflection,
                )
                .unwrap();
                let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
                assert!(quality.is_watertight(), "{up} {distance}: {quality:?}");
                assert_eq!(quality.euler_characteristic, 2);
                assert_eq!(
                    remus_operations::tessellate::boundary_edge_count(&mesh),
                    0,
                    "{up} {distance}: inconsistent triangle winding"
                );
                let corner = |i: u32| {
                    let p = mesh.positions[i as usize];
                    remus_math::vec::Vec3::new(p.x(), p.y(), p.z())
                };
                mesh.indices
                    .chunks_exact(3)
                    .map(|t| corner(t[0]).dot(corner(t[1]).cross(corner(t[2]))) / 6.0)
                    .sum::<f64>()
            };
            let (coarse, fine) = (mesh_volume(0.002), mesh_volume(0.001));
            let extrapolated = fine + (fine - coarse);
            assert!(
                (extrapolated - expected).abs() < 1e-5,
                "{up} {distance}: volume {extrapolated}, expected {expected}"
            );
            let local_z = |z| if up { z } else { 4.0 - z };
            for (point, expected_class) in [
                (
                    Point3::new(distance, 1.5, local_z(2.0)),
                    PointClassification::Outside,
                ),
                (
                    Point3::new(0.05, 1.5, local_z(0.25)),
                    PointClassification::Outside,
                ),
                (
                    Point3::new(0.05, 1.5, local_z(3.75)),
                    PointClassification::Inside,
                ),
                (Point3::new(1.8, 2.8, 2.0), PointClassification::Inside),
                (Point3::new(-0.1, 1.5, 2.0), PointClassification::Outside),
            ] {
                assert_eq!(
                    classify_point(
                        &topo,
                        outcome.solid,
                        placement.mul_point(point),
                        &ClassifyOptions::default(),
                    )
                    .unwrap(),
                    expected_class,
                    "{up} {distance}: {point:?}"
                );
            }
        }
    }
}
