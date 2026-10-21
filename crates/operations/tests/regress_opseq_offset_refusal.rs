//! F7: assembly refusals keep their type and restore the complete live model.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_io::arena_io::serialize_solids;
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_offset::{OffsetError, OffsetOptions};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::offset_v2::{
    offset_solid_v2, offset_solid_v2_with_entity_evolution, offset_solid_v2_with_evolution,
    shell_v2,
};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn live_state(t: &Topology) -> String {
    format!(
        "{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{}",
        t.vertices().iter().collect::<Vec<_>>(),
        t.edges().iter().collect::<Vec<_>>(),
        t.wires().iter().collect::<Vec<_>>(),
        t.faces().iter().collect::<Vec<_>>(),
        t.shells().iter().collect::<Vec<_>>(),
        t.solids().iter().collect::<Vec<_>>(),
        t.compounds().iter().collect::<Vec<_>>(),
        t.compsolids().iter().collect::<Vec<_>>(),
        (t.num_loops(), t.num_coedges(), t.num_pcurves()),
        t.journal().snapshot(),
        t.mutation_ticks()
    )
}

#[test]
#[allow(clippy::too_many_lines)]
fn inward_fuse_offset_refusals_are_typed_and_atomic() {
    // Literal unit cube/cylinder F7, two neighboring widths and distances.
    for width in [0.9_f64, 1.0, 1.1] {
        for distance in [-0.2, -0.25, -0.3] {
            for scale in [1.0, 1000.0] {
                for placed in [false, true] {
                    let mut original = Topology::new();
                    let cylinder = make_cylinder(&mut original, scale, scale).unwrap();
                    let cube = make_box(&mut original, width * scale, scale, scale).unwrap();
                    let outcome = boolean_with_context(
                        &mut original,
                        BooleanOp::Fuse,
                        cube,
                        cylinder,
                        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                    )
                    .unwrap();
                    assert_eq!(outcome.quality, BooleanQuality::Exact);
                    let source = outcome.solid;
                    let m = if placed {
                        Mat4::translation(11.0 * scale, -7.0 * scale, 5.0 * scale)
                            * Mat4::rotation_z(0.7)
                            * Mat4::rotation_x(0.3)
                    } else {
                        Mat4::identity()
                    };
                    transform_solid(&mut original, source, &m).unwrap();
                    // Independent Cartesian union: disc plus rectangle minus
                    // integral_0^min(width,1) sqrt(1-x^2) dx, times height.
                    let x = width.min(1.0);
                    let overlap = 0.5 * (x * (1.0 - x * x).sqrt() + x.asin());
                    let expected = (std::f64::consts::PI + width - overlap) * scale.powi(3);
                    let volume =
                        remus_operations::measure::solid_volume(&original, source, 1e-4 * scale)
                            .unwrap();
                    assert!((volume - expected).abs() < 1e-6 * expected);
                    for (point, expected) in [
                        (Point3::new(-0.5, 0.0, 0.5), PointClassification::Inside),
                        (Point3::new(0.8, 0.9, 0.5), PointClassification::Inside),
                        (Point3::new(1.2, 0.0, 0.5), PointClassification::Outside),
                    ] {
                        let p = m.mul_point(Point3::new(
                            point.x() * scale,
                            point.y() * scale,
                            point.z() * scale,
                        ));
                        assert_eq!(
                            classify_point(&original, source, p, &ClassifyOptions::default())
                                .unwrap(),
                            expected
                        );
                    }
                    for route in 0..7 {
                        let mut topology = original.clone();
                        let before = live_state(&topology);
                        let sources = [cylinder, cube, source];
                        let bytes = serialize_solids(&topology, &sources).unwrap();
                        let slots = topology.allocated_slot_count();
                        let distance = distance * scale;
                        let error = match route {
                            0 => remus_offset::offset_solid(
                                &mut topology,
                                source,
                                distance,
                                OffsetOptions::default(),
                            )
                            .unwrap_err(),
                            1 => remus_offset::thick_solid(
                                &mut topology,
                                source,
                                distance,
                                &[],
                                OffsetOptions::default(),
                            )
                            .unwrap_err(),
                            2 => remus_offset::offset_solid_with_face_map(
                                &mut topology,
                                source,
                                distance,
                                OffsetOptions::default(),
                            )
                            .unwrap_err(),
                            _ => {
                                let error = match route {
                                    3 => {
                                        offset_solid_v2(&mut topology, source, distance).map(|_| ())
                                    }
                                    4 => offset_solid_v2_with_evolution(
                                        &mut topology,
                                        source,
                                        distance,
                                    )
                                    .map(|_| ()),
                                    5 => offset_solid_v2_with_entity_evolution(
                                        &mut topology,
                                        source,
                                        distance,
                                    )
                                    .map(|_| ()),
                                    _ => shell_v2(&mut topology, source, distance, &[]).map(|_| ()),
                                }
                                .unwrap_err();
                                let OperationsError::Offset(error) = error else {
                                    panic!(
                                        "route={route} width={width} distance={distance} scale={scale} placed={placed}: mistyped {error:?}"
                                    )
                                };
                                error
                            }
                        };
                        assert!(
                            matches!(error, OffsetError::AssemblyFailed { ref reason } if reason.contains("wire loop walk")),
                            "{error:?}"
                        );
                        assert_eq!(
                            live_state(&topology),
                            before,
                            "route={route} left live temporary entities"
                        );
                        assert_eq!(serialize_solids(&topology, &sources).unwrap(), bytes);
                        assert!(
                            topology.allocated_slot_count() > slots,
                            "must reach construction before refusing"
                        );
                        // Every allocated solid remains retired; no rejected result
                        // may be discoverable through the public arena iterator.
                        assert_eq!(topology.num_solids(), original.num_solids());
                    }
                }
            }
        }
    }
}

#[test]
fn caller_errors_remain_invalid_input_and_atomic() {
    let mut t = Topology::new();
    let source = make_box(&mut t, 1.0, 1.0, 1.0).unwrap();
    let before = live_state(&t);
    for distance in [0.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            offset_solid_v2(&mut t, source, distance),
            Err(OperationsError::InvalidInput { .. })
        ));
        assert_eq!(live_state(&t), before);
    }
}
