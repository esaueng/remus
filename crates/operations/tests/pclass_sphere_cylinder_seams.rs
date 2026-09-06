//! General quadric seams must preserve both operands under exact-only booleans.
#![allow(clippy::unwrap_used, clippy::panic)]
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, boolean_with_context},
    primitives::{make_cylinder, make_sphere},
    transform::transform_solid,
};
use remus_topology::Topology;

fn intersection_volume() -> f64 {
    // Horizontal sections are two disks separated by 2 mm. Their overlap
    // integrates independently of kernel construction, projection, or meshes.
    let area = |z: f64| {
        let a = (36.0 - z * z).max(0.0).sqrt();
        let b = 3.0;
        let d = 2.0;
        if d >= a + b {
            return 0.0;
        }
        if d <= (a - b).abs() {
            return std::f64::consts::PI * a.min(b).powi(2);
        }
        a * a * ((d * d + a * a - b * b) / (2.0 * d * a)).acos()
            + b * b * ((d * d + b * b - a * a) / (2.0 * d * b)).acos()
            - 0.5 * ((-d + a + b) * (d + a - b) * (d - a + b) * (d + a + b)).sqrt()
    };
    let n = 20_000;
    let h = 12.0 / f64::from(n);
    h / 3.0
        * (area(-6.0)
            + area(6.0)
            + (1..n)
                .map(|i| (if i % 2 == 0 { 2.0 } else { 4.0 }) * area(-6.0 + f64::from(i) * h))
                .sum::<f64>())
}

fn qualify(op: BooleanOp) {
    let overlap = intersection_volume();
    let sphere_volume_expected = 288.0 * std::f64::consts::PI;
    let cylinder_volume_expected = 180.0 * std::f64::consts::PI;
    let expected = match op {
        BooleanOp::Fuse => sphere_volume_expected + cylinder_volume_expected - overlap,
        BooleanOp::Cut => sphere_volume_expected - overlap,
        BooleanOp::Intersect => overlap,
    };
    for scale in [0.1_f64, 1.0, 10.0] {
        for placed in [false, true] {
            let mut topo = Topology::new();
            let sphere_operand = make_sphere(&mut topo, 6.0 * scale, 24).unwrap();
            let cylinder_operand = make_cylinder(&mut topo, 3.0 * scale, 20.0 * scale).unwrap();
            transform_solid(
                &mut topo,
                cylinder_operand,
                &Mat4::translation(2.0 * scale, 0.0, -10.0 * scale),
            )
            .unwrap();
            if placed {
                let transform = Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                    * Mat4::rotation_y(0.37);
                for solid in [sphere_operand, cylinder_operand] {
                    transform_solid(&mut topo, solid, &transform).unwrap();
                }
            }
            let ctx = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
            let result =
                boolean_with_context(&mut topo, op, sphere_operand, cylinder_operand, &ctx)
                    .unwrap_or_else(|e| panic!("{op:?} scale={scale} placed={placed}: {e}"));
            let report = remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
            assert!(
                report.is_valid(),
                "{op:?} scale={scale} placed={placed}: {report:?}"
            );
            for fid in remus_topology::explorer::solid_faces(&topo, result.solid).unwrap() {
                let face = topo.face(fid).unwrap();
                let mut error = 0.0_f64;
                for eid in remus_topology::explorer::face_edges(&topo, fid).unwrap() {
                    let e = topo.edge(eid).unwrap();
                    if !matches!(e.curve(), remus_topology::edge::EdgeCurve::NurbsCurve(_)) {
                        continue;
                    }
                    let (a, b) = e.strict_domain().unwrap();
                    for i in 0..=128 {
                        let p = e.curve().evaluate_with_endpoints(
                            a + (b - a) * f64::from(i) / 128.0,
                            topo.vertex(e.start()).unwrap().point(),
                            topo.vertex(e.end()).unwrap().point(),
                        );
                        if let Some((u, v)) = face.surface().project_point(p) {
                            error =
                                error.max((p - face.surface().evaluate(u, v).unwrap()).length());
                        }
                    }
                }
                assert!(
                    error < 2e-9,
                    "{op:?} scale={scale} placed={placed}: {} seam residual {error}",
                    face.surface().type_tag()
                );
            }
            let target = expected * scale.powi(3);
            let volume =
                remus_operations::measure::solid_volume(&topo, result.solid, 0.01 * scale).unwrap();

            let mesh =
                remus_operations::tessellate::tessellate_solid(&topo, result.solid, 0.005 * scale)
                    .unwrap();
            let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
            assert!(
                quality.is_watertight(),
                "{op:?} scale={scale} placed={placed}: {quality:?}"
            );
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
                .sum();
            assert!(
                (volume - target).abs() / target < 0.001,
                "{op:?} scale={scale} placed={placed}: {volume} vs {target}; mesh {mesh_volume}; signed {:?}",
                remus_topology::explorer::solid_faces(&topo, result.solid)
                    .unwrap()
                    .iter()
                    .map(
                        |&f| remus_check::properties::face_integrator::integrate_face(&topo, f, 8)
                            .unwrap()
                            .volume
                    )
                    .sum::<f64>()
            );
            assert!(
                (mesh_volume - target).abs() / target < 0.01,
                "{op:?} scale={scale} placed={placed}: mesh {mesh_volume} vs {target}"
            );
        }
    }
}
#[test]
fn offset_sphere_cylinder_fuse_retains_both_operands() {
    qualify(BooleanOp::Fuse);
}
#[test]
fn offset_sphere_cylinder_cut_matches_disk_overlap() {
    qualify(BooleanOp::Cut);
}
#[test]
fn offset_sphere_cylinder_intersect_matches_disk_overlap() {
    qualify(BooleanOp::Intersect);
}
