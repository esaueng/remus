//! Small through-tools must retain their material in long bodies.
#![allow(clippy::unwrap_used)]
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
    vec::Point3,
};
use remus_operations::{
    boolean::{BooleanOp, boolean_with_context},
    classify::{PointClassification, classify_point},
    primitives::make_box,
    transform::transform_solid,
};
use remus_topology::Topology;

#[test]
fn anisotropic_through_tool_preserves_local_material() {
    check_anisotropic_material(false);
}

#[test]
#[ignore = "open: world-coordinate volume loses small-feature precision on rotated long bodies"]
fn anisotropic_world_volume_resolves_small_feature_scale() {
    check_anisotropic_material(true);
}

fn check_anisotropic_material(strict_world_volume: bool) {
    let mut failures = Vec::new();
    for length in [1.0, 1e3, 1e6] {
        for width in [0.1, 0.001] {
            for placed in [false, true] {
                for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                    let mut topo = Topology::new();
                    let blank = make_box(&mut topo, length, 1.0, 1.0).unwrap();
                    let tool = make_box(&mut topo, width, 0.4, 2.0).unwrap();
                    transform_solid(&mut topo, tool, &Mat4::translation(width, 0.3, -0.5)).unwrap();
                    let placement = if placed {
                        Mat4::translation(17.0, -23.0, 31.0) * Mat4::rotation_y(0.37)
                    } else {
                        Mat4::identity()
                    };
                    for solid in [blank, tool] {
                        transform_solid(&mut topo, solid, &placement).unwrap();
                    }
                    let label = format!("{op:?} length={length:e} width={width:e} placed={placed}");
                    let result = boolean_with_context(
                        &mut topo,
                        op,
                        blank,
                        tool,
                        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                    );
                    let Ok(result) = result else {
                        failures.push(format!("{label}: {result:?}"));
                        continue;
                    };
                    let overlap = width * 0.4;
                    let expected = match op {
                        BooleanOp::Fuse => length + overlap,
                        BooleanOp::Cut => length - overlap,
                        BooleanOp::Intersect => overlap,
                    };
                    let volume =
                        remus_operations::measure::solid_volume(&topo, result.solid, 0.001)
                            .unwrap();
                    let volume_ok = if strict_world_volume || !placed {
                        (volume - expected).abs() / overlap <= 1e-5
                    } else {
                        (volume - expected).abs() <= (expected * 1e-9).max(overlap * 1e-5)
                    };
                    if !volume_ok {
                        failures.push(format!("{label}: volume={volume} expected={expected}"));
                    }
                    for (z, expected_inside) in [
                        (-0.25, matches!(op, BooleanOp::Fuse)),
                        (0.5, !matches!(op, BooleanOp::Cut)),
                        (1.25, matches!(op, BooleanOp::Fuse)),
                    ] {
                        let actual = classify_point(
                            &topo,
                            result.solid,
                            placement.mul_point(Point3::new(1.5 * width, 0.5, z)),
                            0.001,
                            1e-7,
                        )
                        .unwrap();
                        let expected = if expected_inside {
                            PointClassification::Inside
                        } else {
                            PointClassification::Outside
                        };
                        if actual != expected {
                            failures.push(format!("{label}: z={z}: {actual:?} vs {expected:?}"));
                        }
                    }
                    let crop = make_box(&mut topo, 3.0 * width, 1.0, 2.0).unwrap();
                    transform_solid(
                        &mut topo,
                        crop,
                        &(placement * Mat4::translation(0.0, 0.0, -0.5)),
                    )
                    .unwrap();
                    let local = boolean_with_context(
                        &mut topo,
                        BooleanOp::Intersect,
                        result.solid,
                        crop,
                        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                    );
                    match local {
                        Ok(local) => {
                            let expected_local = match op {
                                BooleanOp::Fuse => 3.0 * width + overlap,
                                BooleanOp::Cut => 3.0 * width - overlap,
                                BooleanOp::Intersect => overlap,
                            };
                            let actual =
                                remus_operations::measure::solid_volume(&topo, local.solid, 0.001)
                                    .unwrap();
                            if (actual - expected_local).abs() / overlap > 1e-5 {
                                failures.push(format!(
                                    "{label}: local volume={actual} expected={expected_local}"
                                ));
                            }
                        }
                        Err(error) => {
                            failures.push(format!("{label}: local material query: {error}"));
                        }
                    }
                    let mesh =
                        remus_operations::tessellate::tessellate_solid(&topo, result.solid, 0.001)
                            .unwrap();
                    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
                    if !quality.is_watertight() {
                        failures.push(format!("{label}: mesh {quality:?}"));
                    }
                    let report =
                        remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
                    if !report.is_valid() {
                        failures.push(format!("{label}: {report:?}"));
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
