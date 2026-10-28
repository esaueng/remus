//! A closed section inside a spherical hemisphere must classify its own cap.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::make_sphere;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn check_boolean(scale: f64, placed: bool, reverse: bool, operation: BooleanOp) {
    let mut topo = Topology::new();
    let a = make_sphere(&mut topo, 4.5 * scale, 16).unwrap();
    let b = make_sphere(&mut topo, 5.5 * scale, 16).unwrap();
    let movement = Mat4::translation(0.0, scale, -4.0 * scale) * Mat4::rotation_x(PI / 4.0);
    transform_solid(&mut topo, a, &movement).unwrap();
    if placed {
        let placement =
            Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale) * Mat4::rotation_z(0.37);
        transform_solid(&mut topo, a, &placement).unwrap();
        transform_solid(&mut topo, b, &placement).unwrap();
    }
    let operands = remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let result = boolean_with_context(
        &mut topo,
        operation,
        if reverse { b } else { a },
        if reverse { a } else { b },
        &context,
    )
    .unwrap();
    assert_eq!(result.quality, BooleanQuality::Exact);
    let report = remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
    assert!(
        report.is_valid(),
        "operation={operation:?} scale={scale} placed={placed} reverse={reverse}: {report:?}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap(),
        operands,
        "boolean must preserve both operands"
    );

    for [x, y, z] in [
        [0.0, 0.0, 4.0],
        [-5.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, -4.0],
        [0.0, 1.0, -8.0],
        [0.0, 0.0, 6.0],
        [0.0, 0.0, -9.0],
    ] {
        let in_a = x * x + (y - 1.0_f64).powi(2) + (z + 4.0_f64).powi(2) < 4.5 * 4.5;
        let in_b = x * x + y * y + z * z < 5.5 * 5.5;
        let point = if placed {
            let (sine, cosine) = 0.37_f64.sin_cos();
            Point3::new(
                (cosine * x - sine * y + 10.0) * scale,
                (sine * x + cosine * y - 7.0) * scale,
                (z + 3.0) * scale,
            )
        } else {
            Point3::new(x * scale, y * scale, z * scale)
        };
        assert_eq!(
            classify_point(&topo, result.solid, point, &ClassifyOptions::default()).unwrap(),
            if match operation {
                BooleanOp::Fuse => in_a || in_b,
                BooleanOp::Cut =>
                    if reverse {
                        in_b && !in_a
                    } else {
                        in_a && !in_b
                    },
                BooleanOp::Intersect => in_a && in_b,
            } {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            },
            "material at {x},{y},{z}"
        );
    }

    // Independent two-sphere lens formula, with center separation sqrt(17).
    let d = 17.0_f64.sqrt();
    let r = 4.5_f64;
    let big = 5.5_f64;
    let lens = PI * (r + big - d).powi(2) * (d * d + 2.0 * d * (r + big) - 3.0 * (r - big).powi(2))
        / (12.0 * d);
    let expected = match operation {
        BooleanOp::Fuse => (4.0 * PI / 3.0) * (r.powi(3) + big.powi(3)) - lens,
        BooleanOp::Cut => (4.0 * PI / 3.0) * if reverse { big.powi(3) } else { r.powi(3) } - lens,
        BooleanOp::Intersect => lens,
    } * scale.powi(3);
    let integrated = mass_properties(&topo, result.solid).unwrap().mass;
    assert!(
        (integrated - expected).abs() < expected * 1e-5,
        "mass_properties operation={operation:?} scale={scale} placed={placed} reverse={reverse}: {integrated} != {expected}"
    );
    let volume = solid_volume(&topo, result.solid, 0.01 * scale).unwrap();
    if matches!(operation, BooleanOp::Fuse) {
        // Preserve the original finding's tight gate on both public queries.
        assert!(
            (volume - expected).abs() < expected * 1e-5,
            "solid_volume operation={operation:?} scale={scale} placed={placed} reverse={reverse}: {volume} != {expected}"
        );
    } else {
        // Hole-free sphere booleans use the deflection-controlled mesh path
        // in solid_volume. Unlike the analytic mass query above, its coarse
        // input does not promise 1e-5 relative volume. Check convergence to
        // the independent lens quantity without changing any kernel policy.
        let refined = solid_volume(&topo, result.solid, 0.0003 * scale).unwrap();
        assert!(volume.is_finite() && volume > 0.0);
        assert!(refined.is_finite() && refined > 0.0);
        let coarse_error = (volume - expected).abs();
        let refined_error = (refined - expected).abs();
        assert!(
            refined_error < coarse_error
                || (coarse_error < expected * 1e-5 && refined_error < expected * 1e-5),
            "solid_volume must converge or already meet the quantity oracle: operation={operation:?} scale={scale} placed={placed} reverse={reverse}: coarse={volume}, refined={refined}, exact={expected}"
        );
    }
    let mut previous_mesh_error = f64::INFINITY;
    for deflection in [0.02, 0.005] {
        let mesh = tessellate_solid(&topo, result.solid, deflection * scale).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0);
        assert_eq!(non_manifold_edge_count(&mesh), 0);
        let mut directed_uses = std::collections::BTreeMap::<_, Vec<bool>>::new();
        for triangle in mesh.indices.chunks_exact(3) {
            for index in 0..3 {
                let a = triangle[index];
                let b = triangle[(index + 1) % 3];
                directed_uses
                    .entry((a.min(b), a.max(b)))
                    .or_default()
                    .push(a < b);
            }
        }
        for directions in directed_uses.values() {
            assert_eq!(directions.len(), 2, "raw mesh incidence");
            assert_ne!(directions[0], directions[1], "raw mesh winding");
        }
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mesh_volume: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|triangle| {
                let [a, b, c] = [triangle[0], triangle[1], triangle[2]]
                    .map(|id| mesh.positions[id as usize] - origin);
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        let error = (mesh_volume - expected).abs() / expected;
        assert!(error < 0.005, "mesh volume {mesh_volume} != {expected}");
        assert!(
            error < previous_mesh_error,
            "finer mesh must converge to the independent lens quantity"
        );
        previous_mesh_error = error;
    }
}

#[test]
fn opseq_closed_sphere_section_classifies_its_actual_cap() {
    check_boolean(1.0, false, false, BooleanOp::Fuse);
}

#[test]
fn closed_sphere_section_scale_placement_and_operand_order() {
    for scale in [0.001, 1.0, 1000.0] {
        for placed in [false, true] {
            for reverse in [false, true] {
                for operation in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                    check_boolean(scale, placed, reverse, operation);
                }
            }
        }
    }
}
