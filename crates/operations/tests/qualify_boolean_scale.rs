//! A through-tool must keep the same material across the declared scale range.
#![allow(clippy::unwrap_used)]
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, boolean_with_context},
    primitives::make_box,
    transform::transform_solid,
};
use remus_topology::Topology;

#[test]
fn through_tool_booleans_preserve_material_from_1e_minus_5_to_1e6() {
    let mut failures = Vec::new();
    for exponent in -5..=6 {
        let scale = 10.0_f64.powi(exponent);
        for placed in [false, true] {
            for (op, expected) in [
                (BooleanOp::Fuse, 1.16),
                (BooleanOp::Cut, 0.84),
                (BooleanOp::Intersect, 0.16),
            ] {
                let mut topo = Topology::new();
                let blank = make_box(&mut topo, scale, scale, scale).unwrap();
                let tool = make_box(&mut topo, 0.4 * scale, 0.4 * scale, 2.0 * scale).unwrap();
                transform_solid(
                    &mut topo,
                    tool,
                    &Mat4::translation(0.3 * scale, 0.3 * scale, -0.5 * scale),
                )
                .unwrap();
                if placed {
                    let placement = Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                        * Mat4::rotation_y(0.37);
                    for solid in [blank, tool] {
                        transform_solid(&mut topo, solid, &placement).unwrap();
                    }
                }
                let result = boolean_with_context(
                    &mut topo,
                    op,
                    blank,
                    tool,
                    &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                );
                let label = format!("{op:?} scale={scale:e} placed={placed}");
                let Ok(result) = result else {
                    failures.push(format!("{label}: {result:?}"));
                    continue;
                };
                let report =
                    remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
                if !report.is_valid() {
                    failures.push(format!("{label}: {report:?}"));
                }
                let volume =
                    remus_operations::measure::solid_volume(&topo, result.solid, 0.01 * scale)
                        .unwrap()
                        / scale.powi(3);
                if (volume - expected).abs() / expected > 1e-6 {
                    failures.push(format!("{label}: volume {volume} vs {expected}"));
                }
                let mesh = remus_operations::tessellate::tessellate_solid(
                    &topo,
                    result.solid,
                    0.01 * scale,
                )
                .unwrap();
                let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
                if !quality.is_watertight() {
                    failures.push(format!("{label}: {quality:?}"));
                }
                let origin = mesh.positions[0];
                let mesh_volume: f64 = mesh
                    .indices
                    .chunks_exact(3)
                    .map(|tri| {
                        let a = mesh.positions[tri[0] as usize] - origin;
                        let b = mesh.positions[tri[1] as usize] - origin;
                        let c = mesh.positions[tri[2] as usize] - origin;
                        a.dot(b.cross(c)) / 6.0
                    })
                    .sum::<f64>()
                    / scale.powi(3);
                if (mesh_volume - expected).abs() / expected > 1e-6 {
                    failures.push(format!("{label}: mesh volume {mesh_volume} vs {expected}"));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
