//! Copy/empty shortcuts require whole-geometry containment certificates.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn live_counts(t: &Topology) -> [usize; 6] {
    [
        t.num_vertices(),
        t.num_edges(),
        t.num_wires(),
        t.num_faces(),
        t.num_shells(),
        t.num_solids(),
    ]
}

#[test]
fn box_crossing_torus_hole_never_uses_vertex_containment() {
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for translation in [0.0, 10.0, 100.0] {
        for op in [BooleanOp::Fuse, BooleanOp::Intersect, BooleanOp::Cut] {
            let mut topo = Topology::new();
            let torus = make_torus(&mut topo, 4.0, 3.0, 16).unwrap();
            transform_solid(
                &mut topo,
                torus,
                &Mat4::translation(translation, translation, translation),
            )
            .unwrap();
            let blank = make_box(&mut topo, 3.5, 3.0, 0.2).unwrap();
            transform_solid(
                &mut topo,
                blank,
                &Mat4::translation(translation + 0.5, translation - 1.5, translation - 0.1),
            )
            .unwrap();
            for input in [blank, torus] {
                assert!(
                    remus_operations::validate::validate_solid(&topo, input)
                        .unwrap()
                        .is_valid()
                );
            }
            let point = Point3::new(translation + 0.75, translation, translation);
            assert_eq!(
                classify_point(&topo, blank, point, 0.01, 1e-7).unwrap(),
                PointClassification::Inside
            );
            assert_eq!(
                classify_point(&topo, torus, point, 0.01, 1e-7).unwrap(),
                PointClassification::Outside
            );
            let before = live_counts(&topo);
            match boolean_with_context(&mut topo, op, blank, torus, &context) {
                Ok(outcome) => {
                    assert_eq!(outcome.quality, BooleanQuality::Exact);
                    // Independently integrated circle-segment area through
                    // the torus hole: q(z)=4-sqrt(9-z²), x>=0.5, |z|<=0.1.
                    let cut_volume = 0.123_069_899_796_805_94;
                    let expected = match op {
                        BooleanOp::Cut => cut_volume,
                        BooleanOp::Intersect => 2.1 - cut_volume,
                        BooleanOp::Fuse => 72.0 * std::f64::consts::PI.powi(2) + cut_volume,
                    };
                    let actual = solid_volume(&topo, outcome.solid, 0.001).unwrap();
                    assert!(
                        (actual - expected).abs() < 1e-4,
                        "{op:?} at {translation}: expected {expected}, got {actual}"
                    );
                    assert_eq!(
                        classify_point(&topo, outcome.solid, point, 0.001, 1e-7).unwrap(),
                        if op == BooleanOp::Intersect {
                            PointClassification::Outside
                        } else {
                            PointClassification::Inside
                        }
                    );
                }
                Err(error) => {
                    assert!(
                        matches!(
                            error,
                            remus_operations::OperationsError::ExactOnlyUnattainable
                        ),
                        "{error:?}"
                    );
                    assert_eq!(
                        live_counts(&topo),
                        before,
                        "refusal must roll back live entities"
                    );
                    assert!((solid_volume(&topo, blank, 0.01).unwrap() - 2.1).abs() < 1e-10);
                }
            }
        }
    }
}

fn shift(topo: &mut Topology, solid: remus_topology::SolidId, x: f64, y: f64, z: f64) {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

#[test]
fn convex_box_contains_complete_curved_and_planar_tools() {
    use remus_operations::primitives::{make_cone, make_cylinder, make_sphere};
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        for family in 0..5 {
            let mut topo = Topology::new();
            let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
            shift(&mut topo, blank, placement, placement, placement);
            let (tool, expected) = match family {
                0 => (make_box(&mut topo, 2.0, 2.0, 2.0).unwrap(), 8.0),
                1 => (
                    make_cylinder(&mut topo, 1.0, 2.0).unwrap(),
                    2.0 * std::f64::consts::PI,
                ),
                2 => (
                    make_cone(&mut topo, 1.0, 0.5, 2.0).unwrap(),
                    2.0 * std::f64::consts::PI * 1.75 / 3.0,
                ),
                3 => (
                    make_sphere(&mut topo, 1.0, 16).unwrap(),
                    4.0 * std::f64::consts::PI / 3.0,
                ),
                _ => (
                    make_torus(&mut topo, 1.5, 0.5, 16).unwrap(),
                    0.75 * std::f64::consts::PI.powi(2),
                ),
            };
            shift(
                &mut topo,
                tool,
                placement + 4.0,
                placement + 4.0,
                placement + 4.0,
            );
            if placement == 10.0 {
                let rotation = Mat4::rotation_z(0.4) * Mat4::rotation_x(0.3);
                for solid in [blank, tool] {
                    transform_solid(&mut topo, solid, &rotation).unwrap();
                }
            }

            for (op, expected_volume) in [
                (BooleanOp::Fuse, 1000.0),
                (BooleanOp::Intersect, expected),
                (BooleanOp::Cut, 1000.0 - expected),
            ] {
                let outcome = boolean_with_context(&mut topo, op, blank, tool, &context)
                    .unwrap_or_else(|error| {
                        panic!("family {family}, {op:?}, at {placement}: {error:?}")
                    });
                assert_eq!(outcome.quality, BooleanQuality::Exact);
                let actual = solid_volume(&topo, outcome.solid, 0.01).unwrap();
                // Curved cavity properties use numerical integration at
                // deflection 0.01; this tolerance concerns the measurement,
                // while the boolean geometry must retain Exact quality.
                let limit = if op == BooleanOp::Cut && family >= 2 {
                    0.01 * expected
                } else {
                    1e-7
                };
                assert!(
                    (actual - expected_volume).abs() < limit,
                    "family {family}, {op:?}, at {placement}: {actual} vs {expected_volume}"
                );
                assert!(
                    remus_operations::validate::validate_solid(&topo, outcome.solid)
                        .unwrap()
                        .is_valid()
                );
            }
        }
    }
}

#[test]
fn sphere_and_coaxial_cylinder_containment_preserve_exact_results() {
    use remus_operations::primitives::{make_cylinder, make_sphere};
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        for family in 0..2 {
            let mut topo = Topology::new();
            let (blank, tool, big_volume, small_volume) = if family == 0 {
                let a = make_sphere(&mut topo, 4.0, 16).unwrap();
                let b = make_sphere(&mut topo, 3.5, 16).unwrap();

                (
                    a,
                    b,
                    4.0 * std::f64::consts::PI * 64.0 / 3.0,
                    4.0 * std::f64::consts::PI * 3.5_f64.powi(3) / 3.0,
                )
            } else {
                let a = make_cylinder(&mut topo, 3.0, 6.0).unwrap();
                let b = make_cylinder(&mut topo, 2.8, 2.0).unwrap();
                shift(&mut topo, b, 0.0, 0.0, 2.0);
                (
                    a,
                    b,
                    54.0 * std::f64::consts::PI,
                    2.0 * 2.8_f64.powi(2) * std::f64::consts::PI,
                )
            };
            for solid in [blank, tool] {
                shift(&mut topo, solid, placement, placement, placement);
            }
            for (op, expected) in [
                (BooleanOp::Fuse, big_volume),
                (BooleanOp::Intersect, small_volume),
            ] {
                let outcome = boolean_with_context(&mut topo, op, blank, tool, &context)
                    .unwrap_or_else(|error| {
                        panic!("family {family}, {op:?}, at {placement}: {error:?}")
                    });
                assert_eq!(outcome.quality, BooleanQuality::Exact);
                let actual = solid_volume(&topo, outcome.solid, 0.01).unwrap();
                assert!(
                    (actual - expected).abs() < 1e-7,
                    "family {family}, {op:?}, at {placement}: {actual} vs {expected}"
                );
            }
            let before = live_counts(&topo);
            assert!(matches!(
                boolean_with_context(&mut topo, BooleanOp::Cut, tool, blank, &context),
                Err(remus_operations::OperationsError::EmptyResult { .. })
            ));
            assert_eq!(live_counts(&topo), before);
        }
    }
}

#[test]
fn independent_identical_primitive_copies_keep_identity_shortcut() {
    use remus_operations::copy::copy_solid;
    use remus_operations::primitives::{make_cone, make_cylinder, make_sphere};
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        for family in 0..5 {
            let mut topo = Topology::new();
            let original = match family {
                0 => make_box(&mut topo, 2.0, 2.0, 2.0).unwrap(),
                1 => make_cylinder(&mut topo, 1.0, 2.0).unwrap(),
                2 => make_cone(&mut topo, 1.0, 0.5, 2.0).unwrap(),
                3 => make_sphere(&mut topo, 1.0, 16).unwrap(),
                _ => make_torus(&mut topo, 4.0, 3.0, 16).unwrap(),
            };
            shift(&mut topo, original, placement, placement, placement);
            let copied = copy_solid(&mut topo, original).unwrap();
            let expected = solid_volume(&topo, original, 0.01).unwrap();
            for op in [BooleanOp::Fuse, BooleanOp::Intersect] {
                let outcome =
                    boolean_with_context(&mut topo, op, original, copied, &context).unwrap();
                assert_eq!(outcome.quality, BooleanQuality::Exact);
                assert!(
                    (solid_volume(&topo, outcome.solid, 0.01).unwrap() - expected).abs() < 1e-9
                );
            }
            let before = live_counts(&topo);
            assert!(matches!(
                boolean_with_context(&mut topo, BooleanOp::Cut, original, copied, &context),
                Err(remus_operations::OperationsError::EmptyResult { .. })
            ));
            assert_eq!(live_counts(&topo), before);
        }
    }
}

#[test]
fn full_coaxial_torus_radius_algebra_remains_exact() {
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        let mut topo = Topology::new();
        let blank = make_torus(&mut topo, 4.0, 3.0, 16).unwrap();
        let tool = make_torus(&mut topo, 4.0, 2.0, 16).unwrap();
        for solid in [blank, tool] {
            shift(&mut topo, solid, placement, placement, placement);
        }
        for (op, expected) in [
            (BooleanOp::Fuse, 72.0 * std::f64::consts::PI.powi(2)),
            (BooleanOp::Intersect, 32.0 * std::f64::consts::PI.powi(2)),
        ] {
            let outcome = boolean_with_context(&mut topo, op, blank, tool, &context).unwrap();
            assert_eq!(outcome.quality, BooleanQuality::Exact);
            assert!((solid_volume(&topo, outcome.solid, 0.01).unwrap() - expected).abs() < 1e-7);
        }
        let before = live_counts(&topo);
        assert!(matches!(
            boolean_with_context(&mut topo, BooleanOp::Cut, tool, blank, &context),
            Err(remus_operations::OperationsError::EmptyResult { .. })
        ));
        assert_eq!(live_counts(&topo), before);
    }
}

#[test]
fn uncertain_torus_enclosure_honors_approximation_policy() {
    let context =
        OperationContext::new().with_fallback(FallbackPolicy::AllowApproximate { budget: 0.1 });
    for op in [BooleanOp::Fuse, BooleanOp::Intersect, BooleanOp::Cut] {
        let mut topo = Topology::new();
        let torus = make_torus(&mut topo, 4.0, 3.0, 16).unwrap();
        let blank = make_box(&mut topo, 3.5, 3.0, 0.2).unwrap();
        shift(&mut topo, blank, 0.5, -1.5, -0.1);
        let before = live_counts(&topo);
        match boolean_with_context(&mut topo, op, blank, torus, &context) {
            Ok(outcome) => {
                assert_eq!(
                    outcome.quality,
                    BooleanQuality::Approximate { deflection: 0.1 }
                );
                let cut_volume = 0.123_069_899_796_805_94;
                let expected = match op {
                    BooleanOp::Cut => cut_volume,
                    BooleanOp::Intersect => 2.1 - cut_volume,
                    BooleanOp::Fuse => 72.0 * std::f64::consts::PI.powi(2) + cut_volume,
                };
                // Coarse fixture bounds at deflection 0.1; these are volume
                // regression limits, not a certified geometric error bound.
                let volume_limit = if op == BooleanOp::Fuse { 10.0 } else { 0.02 };
                let actual = solid_volume(&topo, outcome.solid, 0.01).unwrap();
                assert!((actual - expected).abs() < volume_limit, "{op:?}: {actual}");
            }
            Err(error) => {
                assert!(
                    !matches!(error, remus_operations::OperationsError::EmptyResult { .. }),
                    "the requested material is nonempty: {error:?}"
                );
                assert_eq!(live_counts(&topo), before);
            }
        }
    }
}

#[test]
fn cropped_cylinder_copy_cannot_expand_to_complete_carrier() {
    use remus_operations::boolean::{
        boolean_regions, boolean_with_entity_evolution, boolean_with_evolution,
    };
    use remus_operations::copy::copy_solid;
    use remus_operations::primitives::make_cylinder;
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
        let clip = make_box(&mut topo, 2.0, 4.0, 2.0).unwrap();
        shift(&mut topo, cylinder, placement, placement, placement);
        shift(&mut topo, clip, placement, placement - 2.0, placement);
        let half = boolean_with_context(&mut topo, BooleanOp::Intersect, cylinder, clip, &context)
            .unwrap();
        assert_eq!(half.quality, BooleanQuality::Exact);
        assert!(
            remus_operations::validate::validate_solid(&topo, half.solid)
                .unwrap()
                .is_valid()
        );
        let expected = 4.0 * std::f64::consts::PI;
        assert!((solid_volume(&topo, half.solid, 0.01).unwrap() - expected).abs() < 1e-7);
        let copied = copy_solid(&mut topo, half.solid).unwrap();
        assert!(
            remus_operations::validate::validate_solid(&topo, copied)
                .unwrap()
                .is_valid()
        );
        let material = Point3::new(placement + 1.0, placement, placement + 1.0);
        let absent = Point3::new(placement - 1.0, placement, placement + 1.0);
        assert_eq!(
            classify_point(&topo, half.solid, material, 0.01, 1e-7).unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            classify_point(&topo, half.solid, absent, 0.01, 1e-7).unwrap(),
            PointClassification::Outside
        );
        for op in [BooleanOp::Fuse, BooleanOp::Intersect] {
            let before = live_counts(&topo);
            assert!(matches!(
                boolean_with_context(&mut topo, op, half.solid, copied, &context),
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
            ));
            assert_eq!(live_counts(&topo), before);
            assert!(matches!(
                boolean_with_evolution(&mut topo, op, half.solid, copied),
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
            ));
            assert_eq!(live_counts(&topo), before);
            assert!(matches!(
                boolean_with_entity_evolution(&mut topo, op, half.solid, copied),
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
            ));
            assert_eq!(live_counts(&topo), before);
            assert!(matches!(
                boolean_regions(&mut topo, op, half.solid, copied),
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
            ));
            assert_eq!(live_counts(&topo), before);
            let journal_before = topo.journal().snapshot();
            assert!(matches!(
                remus_operations::journal_ops::boolean_journaled_with_operation(
                    &mut topo, op, half.solid, copied
                ),
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
            ));
            assert_eq!(live_counts(&topo), before);
            assert_eq!(topo.journal().snapshot(), journal_before);
        }
    }
}

#[test]
fn canonical_pointed_cone_identity_preserves_exact_results() {
    use remus_operations::copy::copy_solid;
    use remus_operations::primitives::make_cone;
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        for radii in [(1.0, 0.0), (0.0, 1.0)] {
            for copied in [false, true] {
                let mut topo = Topology::new();
                let first = make_cone(&mut topo, radii.0, radii.1, 2.0).unwrap();
                let second = if copied {
                    copy_solid(&mut topo, first).unwrap()
                } else {
                    make_cone(&mut topo, radii.0, radii.1, 2.0).unwrap()
                };
                for solid in [first, second] {
                    shift(&mut topo, solid, placement, placement, placement);
                    if placement == 10.0 {
                        transform_solid(
                            &mut topo,
                            solid,
                            &(Mat4::rotation_z(0.4) * Mat4::rotation_x(0.3)),
                        )
                        .unwrap();
                    }
                }
                for op in [BooleanOp::Fuse, BooleanOp::Intersect] {
                    let outcome = boolean_with_context(&mut topo, op, first, second, &context)
                        .unwrap_or_else(|error| {
                            panic!("{radii:?}, {placement}, copy={copied}, {op:?}: {error:?}")
                        });
                    assert_eq!(outcome.quality, BooleanQuality::Exact);
                    let expected = 2.0 * std::f64::consts::PI / 3.0;
                    assert!(
                        (solid_volume(&topo, outcome.solid, 0.001).unwrap() - expected).abs()
                            < 1e-7
                    );
                }
                let before = live_counts(&topo);
                assert!(matches!(
                    boolean_with_context(&mut topo, BooleanOp::Cut, first, second, &context),
                    Err(remus_operations::OperationsError::EmptyResult { .. })
                ));
                assert_eq!(live_counts(&topo), before);
            }
        }
    }
}

#[test]
fn journaled_torus_enclosure_refuses_before_publishing_history() {
    for placement in [0.0, 10.0, 100.0] {
        for op in [BooleanOp::Fuse, BooleanOp::Intersect, BooleanOp::Cut] {
            let mut topo = Topology::new();
            let torus = make_torus(&mut topo, 4.0, 3.0, 16).unwrap();
            let blank = make_box(&mut topo, 3.5, 3.0, 0.2).unwrap();
            shift(&mut topo, torus, placement, placement, placement);
            shift(
                &mut topo,
                blank,
                placement + 0.5,
                placement - 1.5,
                placement - 0.1,
            );
            let counts_before = live_counts(&topo);
            let journal_before = topo.journal().snapshot();
            assert!(matches!(
                remus_operations::journal_ops::boolean_journaled_with_operation(
                    &mut topo, op, blank, torus
                ),
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
            ));
            assert_eq!(live_counts(&topo), counts_before);
            assert_eq!(topo.journal().snapshot(), journal_before);
        }
    }
}

#[test]
fn canonical_opposite_axis_carriers_keep_exact_identity() {
    use remus_operations::primitives::make_cylinder;
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        for family in 0..2 {
            let mut topo = Topology::new();
            let (first, second, expected) = if family == 0 {
                let a = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
                let b = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
                transform_solid(
                    &mut topo,
                    b,
                    &(Mat4::translation(0.0, 0.0, 2.0) * Mat4::rotation_x(std::f64::consts::PI)),
                )
                .unwrap();
                (a, b, 2.0 * std::f64::consts::PI)
            } else {
                let a = make_torus(&mut topo, 3.0, 0.5, 16).unwrap();
                let b = make_torus(&mut topo, 3.0, 0.5, 16).unwrap();
                transform_solid(&mut topo, b, &Mat4::rotation_x(std::f64::consts::PI)).unwrap();
                (a, b, 1.5 * std::f64::consts::PI.powi(2))
            };
            for solid in [first, second] {
                shift(&mut topo, solid, placement, placement, placement);
            }
            for op in [BooleanOp::Fuse, BooleanOp::Intersect] {
                let outcome = boolean_with_context(&mut topo, op, first, second, &context).unwrap();
                assert_eq!(outcome.quality, BooleanQuality::Exact);
                assert!(
                    (solid_volume(&topo, outcome.solid, 0.001).unwrap() - expected).abs() < 1e-7
                );
            }
            let before = live_counts(&topo);
            assert!(matches!(
                boolean_with_context(&mut topo, BooleanOp::Cut, first, second, &context),
                Err(remus_operations::OperationsError::EmptyResult { .. })
            ));
            assert_eq!(live_counts(&topo), before);
        }
    }
}
