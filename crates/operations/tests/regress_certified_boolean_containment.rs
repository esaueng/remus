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

fn operand_geometry(topo: &Topology, solid: remus_topology::solid::SolidId) -> String {
    use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
    let root = topo.solid(solid).unwrap();
    let mut entities = vec![
        format!("{root:?}"),
        format!("{:?}", topo.shell(root.outer_shell()).unwrap()),
    ];
    for face in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face).unwrap();
        entities.push(format!("{face:?}"));
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            entities.push(format!("{:?}", topo.wire(wire).unwrap()));
        }
    }
    for edge in solid_edges(topo, solid).unwrap() {
        entities.push(format!("{:?}", topo.edge(edge).unwrap()));
    }
    for vertex in solid_vertices(topo, solid).unwrap() {
        entities.push(format!("{:?}", topo.vertex(vertex).unwrap()));
    }
    entities.join("\n")
}

fn assert_complete_sphere_cavity(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
    placement: Mat4,
    center_x: f64,
) {
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;
    let root = topo.solid(solid).unwrap();
    assert_eq!(root.inner_shells().len(), 1);
    assert_eq!(topo.shell(root.outer_shell()).unwrap().faces().len(), 3);
    let cavity = topo.shell(root.inner_shells()[0]).unwrap();
    assert_eq!(cavity.faces().len(), 2);
    for &face in cavity.faces() {
        let face = topo.face(face).unwrap();
        assert!(face.is_reversed());
        assert!(matches!(face.surface(), FaceSurface::Sphere(_)));
    }
    assert_eq!(solid_faces(topo, solid).unwrap().len(), 5);
    assert!(
        remus_operations::validate::validate_solid(topo, solid)
            .unwrap()
            .is_valid()
    );
    // Independent support geometry: cylinder volume minus the whole ball.
    // Use trimmed quadrature, rather than chord-based volume, for this oracle.
    let expected = std::f64::consts::PI * (10.0_f64.powi(2) * 20.0 - 4.0 * 8.0_f64.powi(3) / 3.0);
    let props = remus_operations::measure::mass_properties_with_options(
        topo,
        solid,
        &remus_check::properties::PropertiesOptions {
            adaptive_eps: 1e-10,
            gauss_order: 8,
            max_depth: 12,
        },
    )
    .unwrap();
    assert!(
        (props.mass - expected).abs() < expected * 1e-8,
        "{} vs {expected}",
        props.mass
    );
    for (point, expected) in [
        (
            Point3::new(center_x, 0.0, 10.0),
            PointClassification::Outside,
        ),
        (Point3::new(-9.0, 0.0, 10.0), PointClassification::Inside),
        (Point3::new(0.0, 0.0, 1.0), PointClassification::Inside),
        (Point3::new(0.0, 0.0, 19.0), PointClassification::Inside),
        (Point3::new(11.0, 0.0, 10.0), PointClassification::Outside),
    ] {
        assert_eq!(
            classify_point(topo, solid, placement.mul_point(point), 0.01, 1e-7).unwrap(),
            expected
        );
    }
}

#[test]
fn whole_sphere_in_finite_cylinder_restores_exact_cavity_and_face_evolution() {
    use remus_operations::boolean::{boolean, boolean_with_evolution};
    use remus_operations::primitives::{make_cylinder, make_sphere};
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for (translation, angle, center_x) in [
        (0.0, 0.0, 0.0),
        (100.0, 0.7, 1.0),
        (-100.0, std::f64::consts::PI, 0.0),
    ] {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
        let sphere = make_sphere(&mut topo, 8.0, 32).unwrap();
        shift(&mut topo, sphere, center_x, 0.0, 10.0);
        let placement =
            Mat4::translation(translation, -translation, translation) * Mat4::rotation_x(angle);
        for operand in [cylinder, sphere] {
            transform_solid(&mut topo, operand, &placement).unwrap();
            assert!(
                remus_operations::validate::validate_solid(&topo, operand)
                    .unwrap()
                    .is_valid()
            );
        }
        let operands = [
            operand_geometry(&topo, cylinder),
            operand_geometry(&topo, sphere),
        ];
        let journal_before = topo.journal().snapshot();
        let plain = boolean(&mut topo, BooleanOp::Cut, cylinder, sphere).unwrap();
        assert_complete_sphere_cavity(&topo, plain, placement, center_x);
        let qualified =
            boolean_with_context(&mut topo, BooleanOp::Cut, cylinder, sphere, &context).unwrap();
        assert_eq!(qualified.quality, BooleanQuality::Exact);
        assert_complete_sphere_cavity(&topo, qualified.solid, placement, center_x);
        let (evolved, history) =
            boolean_with_evolution(&mut topo, BooleanOp::Cut, cylinder, sphere).unwrap();
        assert_complete_sphere_cavity(&topo, evolved, placement, center_x);
        assert_eq!(
            history.origin,
            remus_operations::evolution::EvolutionOrigin::Geometry
        );
        let outputs: std::collections::BTreeSet<_> =
            remus_topology::explorer::solid_faces(&topo, evolved)
                .unwrap()
                .into_iter()
                .map(remus_topology::arena::Id::index)
                .collect();
        // The legacy geometric face map attributes the copied outer faces;
        // reversing the cavity faces prevents its normal-based attribution.
        // Keep that established, explicitly inferred scope, never inventing
        // construction-derived cavity lineage.
        let attributed = history.attributed_outputs();
        assert!(attributed.is_subset(&outputs));
        assert_eq!(attributed.len(), 3);
        assert_eq!(
            [
                operand_geometry(&topo, cylinder),
                operand_geometry(&topo, sphere)
            ],
            operands
        );
        assert_eq!(topo.journal().snapshot(), journal_before);
    }
}

#[test]
fn sphere_cylinder_containment_copy_and_empty_results_use_complete_geometry() {
    use remus_operations::primitives::{make_cylinder, make_sphere};
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let sphere = make_sphere(&mut topo, 8.0, 32).unwrap();
    shift(&mut topo, sphere, 1.0, 1.0, 10.0);
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for (op, expected) in [
        (BooleanOp::Fuse, std::f64::consts::PI * 2000.0),
        (
            BooleanOp::Intersect,
            std::f64::consts::PI * 4.0 * 512.0 / 3.0,
        ),
    ] {
        for (a, b) in [(cylinder, sphere), (sphere, cylinder)] {
            let result = boolean_with_context(&mut topo, op, a, b, &context).unwrap();
            assert_eq!(result.quality, BooleanQuality::Exact);
            let volume = remus_check::properties::solid_volume(
                &topo,
                result.solid,
                &remus_check::properties::PropertiesOptions::default(),
            )
            .unwrap();
            assert!((volume - expected).abs() < expected * 1e-6);
        }
    }
    let before = live_counts(&topo);
    assert!(matches!(
        boolean_with_context(&mut topo, BooleanOp::Cut, sphere, cylinder, &context),
        Err(remus_operations::OperationsError::EmptyResult { .. })
    ));
    assert_eq!(live_counts(&topo), before);
}

#[test]
fn sphere_cylinder_entity_history_stays_fail_closed_and_transactional() {
    use remus_operations::primitives::{make_cylinder, make_sphere};
    for journaled in [false, true] {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
        let sphere = make_sphere(&mut topo, 8.0, 32).unwrap();
        shift(&mut topo, sphere, 0.0, 0.0, 10.0);
        let pending = topo.journal_begin("operand_boundary");
        remus_operations::journal_ops::record_barrier_over_solid(&mut topo, pending, cylinder)
            .unwrap();
        let before = live_counts(&topo);
        let slots_before = topo.allocated_slot_count();
        let journal_before = topo.journal().snapshot();
        let operands = [
            operand_geometry(&topo, cylinder),
            operand_geometry(&topo, sphere),
        ];
        let error = if journaled {
            remus_operations::journal_ops::boolean_journaled_with_operation(
                &mut topo,
                BooleanOp::Cut,
                cylinder,
                sphere,
            )
            .unwrap_err()
        } else {
            remus_operations::boolean::boolean_with_entity_evolution(
                &mut topo,
                BooleanOp::Cut,
                cylinder,
                sphere,
            )
            .unwrap_err()
        };
        assert!(
            matches!(error, remus_operations::OperationsError::Algo(_)),
            "{error:?}"
        );
        assert_eq!(live_counts(&topo), before);
        assert!(topo.allocated_slot_count() >= slots_before);
        assert_eq!(topo.journal().snapshot(), journal_before);
        assert_eq!(
            [
                operand_geometry(&topo, cylinder),
                operand_geometry(&topo, sphere)
            ],
            operands
        );
    }
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

fn noncanonical_cylinder(topo: &mut Topology, split_lateral: bool) -> remus_topology::SolidId {
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_operations::primitives::make_cylinder;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    let cylinder = make_cylinder(topo, 2.0, 2.0).unwrap();
    if !split_lateral {
        let clip = make_box(topo, 2.0, 4.0, 2.0).unwrap();
        shift(topo, clip, 0.0, -2.0, 0.0);
        return boolean_with_context(
            topo,
            BooleanOp::Intersect,
            cylinder,
            clip,
            &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
        )
        .unwrap()
        .solid;
    }

    // Partition the complete cylinder's lateral face at z=1. The two bands
    // share one exact full-circle edge; the original planar caps are reused.
    let shell = topo.solid(cylinder).unwrap().outer_shell();
    let faces = topo.shell(shell).unwrap().faces().to_vec();
    let lateral = topo.face(faces[0]).unwrap();
    let surface = lateral.surface().clone();
    let edges = topo.wire(lateral.outer_wire()).unwrap().edges();
    let bottom = edges[0].edge();
    let top = edges[2].edge();
    let bottom_vertex = topo.edge(bottom).unwrap().start();
    let top_vertex = topo.edge(top).unwrap().start();
    let point = Point3::new(2.0, 0.0, 1.0);
    let middle_vertex = topo.add_vertex(Vertex::new(point, 1e-7));
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    let start = circle.project(point);
    let mut edge = Edge::new(middle_vertex, middle_vertex, EdgeCurve::Circle(circle));
    edge.set_trim(Some((start, start + std::f64::consts::TAU)));
    let middle = topo.add_edge(edge);
    let lower_seam = topo.add_edge(Edge::new(bottom_vertex, middle_vertex, EdgeCurve::Line));
    let upper_seam = topo.add_edge(Edge::new(middle_vertex, top_vertex, EdgeCurve::Line));
    let mut result_faces = vec![faces[1], faces[2]];
    for (lo, hi, seam) in [(bottom, middle, lower_seam), (middle, top, upper_seam)] {
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(lo, true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(hi, false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        assert!(matches!(surface, FaceSurface::Cylinder(_)));
        result_faces.push(topo.add_face(Face::new(wire, vec![], surface.clone())));
    }
    let shell = topo.add_shell(Shell::new(result_faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

#[test]
fn separated_noncanonical_coaxial_cylinders_preserve_exact_material_in_both_orders() {
    use remus_operations::boolean::{
        boolean, boolean_with_entity_evolution, boolean_with_evolution,
    };
    use remus_operations::copy::copy_solid;
    use remus_operations::journal_ops::boolean_journaled_with_operation;
    use remus_topology::explorer::solid_faces;
    use remus_topology::journal::EntityKind;
    use remus_topology::naming::{PersistentRef, Resolution, resolve};

    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for split_lateral in [false, true] {
        let mut original = Topology::new();
        let a = noncanonical_cylinder(&mut original, split_lateral);
        let b = copy_solid(&mut original, a).unwrap();
        shift(&mut original, b, 0.0, 0.0, 4.0);
        for input in [a, b] {
            assert!(
                remus_operations::validate::validate_solid(&original, input)
                    .unwrap()
                    .is_valid()
            );
            assert!(matches!(
                remus_algo::classifier::try_build_analytic_classifier(&original, input),
                Some(remus_algo::classifier::AnalyticClassifier::Cylinder { .. })
            ));
        }
        let expected = if split_lateral { 8.0 } else { 4.0 } * std::f64::consts::PI;
        for (blank, tool, blank_z) in [(a, b, 1.0), (b, a, 5.0)] {
            for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                for route in 0..5 {
                    let mut topo = original.clone();
                    let input_faces: Vec<_> = solid_faces(&topo, a)
                        .unwrap()
                        .into_iter()
                        .chain(solid_faces(&topo, b).unwrap())
                        .collect();
                    if op == BooleanOp::Intersect && route >= 3 {
                        // The GFA history APIs retain their existing typed
                        // empty-result convention rather than the context
                        // API's faceless-solid sentinel. The carrier precheck
                        // must admit the proven gap before that decision.
                        let before = live_counts(&topo);
                        let journal = topo.journal().snapshot();
                        let outcome = if route == 3 {
                            boolean_with_entity_evolution(&mut topo, op, blank, tool)
                                .map(|(solid, _)| solid)
                        } else {
                            boolean_journaled_with_operation(&mut topo, op, blank, tool)
                                .map(|outcome| outcome.solid)
                        };
                        match outcome {
                            Ok(result) => {
                                assert!(solid_faces(&topo, result).unwrap().is_empty());
                                assert_eq!(solid_volume(&topo, result, 0.01).unwrap(), 0.0);
                            }
                            Err(remus_operations::OperationsError::EmptyResult { .. }) => {
                                assert_eq!(live_counts(&topo), before);
                                assert_eq!(topo.journal().snapshot(), journal);
                            }
                            Err(remus_operations::OperationsError::Algo(
                                remus_algo::error::AlgoError::AssemblyFailed(reason),
                            )) if reason == "no faces selected" => {
                                assert_eq!(live_counts(&topo), before);
                                assert_eq!(topo.journal().snapshot(), journal);
                            }
                            Err(error) => panic!("disjoint history intersection: {error:?}"),
                        }
                        continue;
                    }
                    let result = match route {
                        0 => boolean(&mut topo, op, blank, tool).unwrap(),
                        1 => {
                            let outcome =
                                boolean_with_context(&mut topo, op, blank, tool, &context).unwrap();
                            assert_eq!(outcome.quality, BooleanQuality::Exact);
                            outcome.solid
                        }
                        2 => {
                            boolean_with_evolution(&mut topo, op, blank, tool)
                                .unwrap()
                                .0
                        }
                        3 => {
                            let (solid, evolution) =
                                boolean_with_entity_evolution(&mut topo, op, blank, tool).unwrap();
                            let result_faces = solid_faces(&topo, solid).unwrap();
                            assert!(!result_faces.is_empty());
                            assert_eq!(evolution.faces.len(), result_faces.len());
                            for face in result_faces {
                                assert!(evolution.faces.iter().any(|(result, source)| {
                                    *result == face.index()
                                        && source.is_some_and(|source| {
                                            input_faces.iter().any(|input| input.index() == source)
                                        })
                                }));
                            }
                            solid
                        }
                        4 => {
                            let outcome =
                                boolean_journaled_with_operation(&mut topo, op, blank, tool)
                                    .unwrap();
                            if op != BooleanOp::Intersect {
                                let reference = PersistentRef::operation_output(
                                    outcome.op,
                                    EntityKind::Face,
                                    0,
                                );
                                let Resolution::Bound { entity, .. } = resolve(&topo, &reference)
                                else {
                                    panic!("disjoint journal result must have a bound face output");
                                };
                                assert!(
                                    solid_faces(&topo, outcome.solid)
                                        .unwrap()
                                        .iter()
                                        .any(|face| face.index() == entity.index)
                                );
                            }
                            outcome.solid
                        }
                        _ => unreachable!(),
                    };
                    let volume = solid_volume(&topo, result, 0.01).unwrap();
                    let wanted = match op {
                        BooleanOp::Fuse => 2.0 * expected,
                        BooleanOp::Cut => expected,
                        BooleanOp::Intersect => 0.0,
                    };
                    assert!(
                        (volume - wanted).abs() < 1e-7,
                        "split={split_lateral}/{op:?}/route={route}: {volume} vs {wanted}"
                    );
                    assert!(
                        solid_faces(&topo, result)
                            .unwrap()
                            .iter()
                            .all(|face| !input_faces.contains(face))
                    );
                    for input in [a, b] {
                        assert!(
                            (solid_volume(&topo, input, 0.01).unwrap() - expected).abs() < 1e-7
                        );
                    }
                    if op == BooleanOp::Intersect {
                        assert!(solid_faces(&topo, result).unwrap().is_empty());
                        continue;
                    }
                    assert!(
                        remus_operations::validate::validate_solid(&topo, result)
                            .unwrap()
                            .is_valid()
                    );
                    assert_eq!(
                        classify_point(&topo, result, Point3::new(1.0, 0.0, blank_z), 0.01, 1e-7)
                            .unwrap(),
                        PointClassification::Inside
                    );
                    assert_eq!(
                        classify_point(&topo, result, Point3::new(-1.0, 0.0, blank_z), 0.01, 1e-7)
                            .unwrap(),
                        if split_lateral {
                            PointClassification::Inside
                        } else {
                            PointClassification::Outside
                        }
                    );
                    assert_eq!(
                        classify_point(&topo, result, Point3::new(1.0, 0.0, 3.0), 0.01, 1e-7)
                            .unwrap(),
                        PointClassification::Outside
                    );
                }
            }
        }
    }
}

#[test]
fn noncanonical_carrier_overlap_touch_and_nonclear_gaps_still_refuse_atomically() {
    use remus_math::tolerance::Tolerance;
    use remus_operations::OperationsError;
    use remus_operations::boolean::boolean_with_entity_evolution;
    use remus_operations::copy::copy_solid;
    use remus_operations::journal_ops::boolean_journaled_with_operation;

    for split_lateral in [false, true] {
        for (offset, linear) in [
            (1.0, 1e-7),        // overlapping axial ranges
            (2.0, 1e-7),        // touching caps
            (2.0 + 5e-8, 1e-7), // less than the default clear-gap margin
            (2.0 + 5e-5, 1e-4), // less than the caller's custom margin
        ] {
            let mut original = Topology::new();
            let a = noncanonical_cylinder(&mut original, split_lateral);
            let b = copy_solid(&mut original, a).unwrap();
            shift(&mut original, b, 0.0, 0.0, offset);
            let context = OperationContext::new()
                .with_fallback(FallbackPolicy::ExactOnly)
                .with_tolerance(Tolerance {
                    linear,
                    ..Tolerance::new()
                });
            for (blank, tool) in [(a, b), (b, a)] {
                for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                    let mut topo = original.clone();
                    let before = live_counts(&topo);
                    let slots = topo.allocated_slot_count();
                    let ticks = topo.mutation_ticks();
                    let cache = topo.cache_identity();
                    let journal = topo.journal().snapshot();
                    assert!(
                        matches!(
                            boolean_with_context(&mut topo, op, blank, tool, &context),
                            Err(OperationsError::ExactOnlyUnattainable)
                        ),
                        "split={split_lateral}, offset={offset}, tol={linear}, {op:?}"
                    );
                    if linear == Tolerance::new().linear {
                        assert!(matches!(
                            boolean_with_entity_evolution(&mut topo, op, blank, tool),
                            Err(OperationsError::ExactOnlyUnattainable)
                        ));
                        assert!(matches!(
                            boolean_journaled_with_operation(&mut topo, op, blank, tool),
                            Err(OperationsError::ExactOnlyUnattainable)
                        ));
                    }
                    assert_eq!(live_counts(&topo), before);
                    assert_eq!(topo.allocated_slot_count(), slots);
                    assert_eq!(topo.mutation_ticks(), ticks);
                    // Rollback invalidates spatial preparations while retaining
                    // this document's identity; generations never rewind.
                    assert_eq!(topo.cache_identity().lineage, cache.lineage);
                    assert!(topo.cache_identity().generation > cache.generation);
                    assert_eq!(topo.journal().snapshot(), journal);
                }
            }
        }
    }
}

#[test]
fn separated_cropped_cylinders_keep_translated_exact_context_and_region_results() {
    use remus_operations::boolean::boolean_regions;
    use remus_operations::copy::copy_solid;

    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for placement in [0.0, 10.0, 100.0] {
        let mut original = Topology::new();
        let a = noncanonical_cylinder(&mut original, false);
        shift(&mut original, a, placement, placement, placement);
        let b = copy_solid(&mut original, a).unwrap();
        shift(&mut original, b, 0.0, 0.0, 4.0);
        for (blank, tool, blank_z) in [(a, b, placement + 1.0), (b, a, placement + 5.0)] {
            for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                let mut topo = original.clone();
                let outcome = boolean_with_context(&mut topo, op, blank, tool, &context).unwrap();
                assert_eq!(outcome.quality, BooleanQuality::Exact);
                let expected = match op {
                    BooleanOp::Fuse => 8.0 * std::f64::consts::PI,
                    BooleanOp::Cut => 4.0 * std::f64::consts::PI,
                    BooleanOp::Intersect => 0.0,
                };
                assert!(
                    (solid_volume(&topo, outcome.solid, 0.01).unwrap() - expected).abs() < 1e-7
                );
                let before = live_counts(&topo);
                let slots = topo.allocated_slot_count();
                let journal = topo.journal().snapshot();
                if op == BooleanOp::Intersect {
                    let error = boolean_regions(&mut topo, op, blank, tool).unwrap_err();
                    match error {
                        remus_operations::OperationsError::EmptyResult { .. } => {}
                        remus_operations::OperationsError::Algo(
                            remus_algo::error::AlgoError::AssemblyFailed(reason),
                        ) if reason == "no faces selected" => {}
                        error => panic!("disjoint region intersection: {error:?}"),
                    }
                    assert_eq!(live_counts(&topo), before);
                    assert_eq!(topo.allocated_slot_count(), slots);
                    assert_eq!(topo.journal().snapshot(), journal);
                    continue;
                }
                let result = boolean_regions(&mut topo, op, blank, tool).unwrap();
                assert_eq!(
                    result.regions.len(),
                    if op == BooleanOp::Fuse { 2 } else { 1 }
                );
                assert_eq!(
                    topo.compound(result.compound).unwrap().solids().len(),
                    result.regions.len()
                );
                let volume: f64 = result
                    .regions
                    .iter()
                    .map(|region| {
                        assert!(
                            remus_operations::validate::validate_solid(&topo, region.solid)
                                .unwrap()
                                .is_valid()
                        );
                        assert!(!region.evolution.faces.is_empty());
                        solid_volume(&topo, region.solid, 0.01).unwrap()
                    })
                    .sum();
                assert!((volume - expected).abs() < 1e-7);
                assert!(result.regions.iter().any(|region| {
                    classify_point(
                        &topo,
                        region.solid,
                        Point3::new(placement + 1.0, placement, blank_z),
                        0.01,
                        1e-7,
                    )
                    .unwrap()
                        == PointClassification::Inside
                }));
                for region in &result.regions {
                    assert_eq!(
                        classify_point(
                            &topo,
                            region.solid,
                            Point3::new(placement - 1.0, placement, blank_z),
                            0.01,
                            1e-7
                        )
                        .unwrap(),
                        PointClassification::Outside
                    );
                }
                for input in [a, b] {
                    assert!(
                        (solid_volume(&topo, input, 0.01).unwrap() - 4.0 * std::f64::consts::PI)
                            .abs()
                            < 1e-7
                    );
                }
            }
        }
    }
}

#[test]
fn separated_carriers_with_unknown_boundary_authority_still_refuse_before_mutation() {
    use remus_operations::boolean::{boolean_regions, boolean_with_entity_evolution};
    use remus_operations::copy::copy_solid;
    use remus_operations::journal_ops::boolean_journaled_with_operation;
    use remus_topology::edge::EdgeCurve;

    let mut original = Topology::new();
    let a = noncanonical_cylinder(&mut original, false);
    let b = copy_solid(&mut original, a).unwrap();
    shift(&mut original, b, 0.0, 0.0, 4.0);
    // Stored vertices still imply the separated slabs, but an invalid stored
    // NURBS trim has no authoritative whole-span bound. No sampled box or
    // classifier may turn that uncertainty into a new disjoint certificate.
    let edge = remus_topology::explorer::solid_edges(&original, a)
        .unwrap()
        .into_iter()
        .find(|&id| matches!(original.edge(id).unwrap().curve(), EdgeCurve::Circle(_)))
        .unwrap();
    let stored = original.edge(edge).unwrap();
    let (lo, hi) = stored.trim().unwrap();
    let EdgeCurve::Circle(circle) = stored.curve() else {
        unreachable!()
    };
    let curve = remus_geometry::convert::curve_to_nurbs::circle_to_nurbs(circle, lo, hi).unwrap();
    let (lo, hi) = curve.domain();
    let stored = original.edge_mut(edge).unwrap();
    stored.set_curve(EdgeCurve::NurbsCurve(curve));
    stored.set_trim(Some((lo, hi + 1.0)));
    assert!(
        !remus_check::distance::face_bounds::edge_span_bound(&original, edge)
            .unwrap()
            .is_prunable()
    );
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for (blank, tool) in [(a, b), (b, a)] {
        for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
            for route in 0..4 {
                let mut topo = original.clone();
                let counts = live_counts(&topo);
                let slots = topo.allocated_slot_count();
                let ticks = topo.mutation_ticks();
                let journal = topo.journal().snapshot();
                let error = match route {
                    0 => boolean_with_context(&mut topo, op, blank, tool, &context).unwrap_err(),
                    1 => boolean_with_entity_evolution(&mut topo, op, blank, tool).unwrap_err(),
                    2 => boolean_regions(&mut topo, op, blank, tool).unwrap_err(),
                    3 => boolean_journaled_with_operation(&mut topo, op, blank, tool).unwrap_err(),
                    _ => unreachable!(),
                };
                assert!(
                    matches!(
                        error,
                        remus_operations::OperationsError::ExactOnlyUnattainable
                    ),
                    "{op:?}/{route}: {error:?}"
                );
                assert_eq!(live_counts(&topo), counts);
                assert_eq!(topo.allocated_slot_count(), slots);
                assert_eq!(topo.mutation_ticks(), ticks);
                assert_eq!(topo.journal().snapshot(), journal);
            }
        }
    }
}

#[test]
fn carrier_disjoint_certificate_covers_narrow_nurbs_trim_spans() {
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_operations::copy::copy_solid;
    use remus_topology::edge::EdgeCurve;

    let mut topo = Topology::new();
    let a = noncanonical_cylinder(&mut topo, false);
    let b = copy_solid(&mut topo, a).unwrap();
    shift(&mut topo, b, 0.0, 0.0, 4.0);
    let edge = remus_topology::explorer::solid_edges(&topo, a)
        .unwrap()
        .into_iter()
        .find(|&id| {
            let edge = topo.edge(id).unwrap();
            matches!(edge.curve(), EdgeCurve::Line)
                && topo.vertex(edge.start()).unwrap().point().z() == 0.0
                && topo.vertex(edge.end()).unwrap().point().z() == 2.0
        })
        .unwrap();
    let start = topo
        .vertex(topo.edge(edge).unwrap().start())
        .unwrap()
        .point();
    let delta = topo.vertex(topo.edge(edge).unwrap().end()).unwrap().point() - start;
    // Deliberately fold the shared planar/cylinder seam in a narrow knot span.
    // This defensive fixture is not asserted to be a valid model: it proves
    // the new certificate cannot rely on measured trim samples or endpoints.
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.002, 0.003, 0.004, 1.0, 1.0],
        vec![
            start,
            start + delta * 0.002,
            start + delta * 3.0,
            start + delta * 0.004,
            start + delta,
        ],
        vec![1.0; 5],
    )
    .unwrap();
    assert_eq!(curve.evaluate(0.003).z(), 6.0);
    let stored = topo.edge_mut(edge).unwrap();
    stored.set_curve(EdgeCurve::NurbsCurve(curve));
    stored.set_trim(Some((0.0, 1.0)));
    let bound = remus_check::distance::face_bounds::edge_span_bound(&topo, edge).unwrap();
    assert!(bound.is_prunable());
    assert_eq!(bound.aabb().max.z(), 6.0);
    assert_eq!(
        remus_operations::measure::solid_bounding_box(&topo, a)
            .unwrap()
            .max
            .z(),
        2.0
    );
    let counts = live_counts(&topo);
    let slots = topo.allocated_slot_count();
    let journal = topo.journal().snapshot();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
        assert!(matches!(
            boolean_with_context(&mut topo, op, a, b, &context),
            Err(remus_operations::OperationsError::ExactOnlyUnattainable)
        ));
        assert_eq!(live_counts(&topo), counts);
        assert_eq!(topo.allocated_slot_count(), slots);
        assert_eq!(topo.journal().snapshot(), journal);
    }
}

#[test]
fn separated_cropped_carriers_preserve_approximate_only_policy() {
    use remus_operations::copy::copy_solid;
    let mut topo = Topology::new();
    let a = noncanonical_cylinder(&mut topo, false);
    let b = copy_solid(&mut topo, a).unwrap();
    shift(&mut topo, b, 0.0, 0.0, 4.0);
    let context =
        OperationContext::new().with_fallback(FallbackPolicy::ApproximateOnly { budget: 0.1 });
    let counts = live_counts(&topo);
    let slots = topo.allocated_slot_count();
    let journal = topo.journal().snapshot();
    // The existing mesh route rejects this fixture's free boundaries. The
    // disjoint exact certificate must not bypass an ApproximateOnly request.
    let error = boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &context).unwrap_err();
    assert!(
        matches!(error, remus_operations::OperationsError::InvalidInput { reason }
        if reason.contains("non-manifold") && reason.contains("free boundary"))
    );
    assert_eq!(live_counts(&topo), counts);
    // Failed mesh work may reserve retired handles; rollback preserves their
    // high-water marks rather than permitting those IDs to alias later work.
    assert!(topo.allocated_slot_count() >= slots);
    assert_eq!(topo.journal().snapshot(), journal);
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
