//! General quadric seams must preserve both operands under exact-only booleans.
#![allow(clippy::unwrap_used, clippy::panic)]
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, boolean_with_context},
    primitives::{make_cone, make_sphere},
    transform::transform_solid,
};
use remus_topology::Topology;

fn intersection_volume() -> f64 {
    // Horizontal sections are two disks separated by 2 mm. Their overlap
    // integrates independently of kernel construction, projection, or meshes.
    let area = |z: f64| {
        let a = 6.0 - z / 3.0;
        let b = (16.0 - (z - 6.0).powi(2)).max(0.0).sqrt();
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
    let h = 8.0 / f64::from(n);
    h / 3.0
        * (area(2.0)
            + area(10.0)
            + (1..n)
                .map(|i| (if i % 2 == 0 { 2.0 } else { 4.0 }) * area(2.0 + f64::from(i) * h))
                .sum::<f64>())
}

fn qualify(op: BooleanOp) {
    let overlap = intersection_volume();
    let cone_volume = 208.0 * std::f64::consts::PI;
    let sphere_volume = 256.0 * std::f64::consts::PI / 3.0;
    let expected = match op {
        BooleanOp::Fuse => cone_volume + sphere_volume - overlap,
        BooleanOp::Cut => cone_volume - overlap,
        BooleanOp::Intersect => overlap,
    };
    for scale in [0.1_f64, 1.0, 10.0] {
        for placed in [false, true] {
            let mut topo = Topology::new();
            let cone = make_cone(&mut topo, 6.0 * scale, 2.0 * scale, 12.0 * scale).unwrap();
            let sphere = make_sphere(&mut topo, 4.0 * scale, 24).unwrap();
            transform_solid(
                &mut topo,
                sphere,
                &Mat4::translation(2.0 * scale, 0.0, 6.0 * scale),
            )
            .unwrap();
            if placed {
                let transform = Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                    * Mat4::rotation_y(0.37);
                for solid in [cone, sphere] {
                    transform_solid(&mut topo, solid, &transform).unwrap();
                }
            }
            let ctx = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
            let result = boolean_with_context(&mut topo, op, cone, sphere, &ctx)
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
                "{op:?} scale={scale} placed={placed}: {volume} vs {target}"
            );
            assert!(
                (mesh_volume - target).abs() / target < 0.01,
                "{op:?} scale={scale} placed={placed}: mesh {mesh_volume} vs {target}"
            );
        }
    }
}
#[test]
fn offset_cone_sphere_fuse_retains_protruding_sphere() {
    qualify(BooleanOp::Fuse);
}
#[test]
fn offset_cone_sphere_cut_matches_disk_overlap() {
    qualify(BooleanOp::Cut);
}
#[test]
fn offset_cone_sphere_intersect_matches_disk_overlap() {
    qualify(BooleanOp::Intersect);
}

#[test]
fn remaining_quartic_witnesses_refuse_without_mutating_operands() {
    use remus_operations::primitives::{make_cylinder, make_torus};
    for torus_pair in [false, true] {
        for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
            let mut topo = Topology::new();
            let (a, b) = if torus_pair {
                let a = make_torus(&mut topo, 6.0, 2.0, 32).unwrap();
                let b = make_sphere(&mut topo, 3.0, 24).unwrap();
                transform_solid(&mut topo, b, &Mat4::translation(5.0, 0.0, 1.0)).unwrap();
                (a, b)
            } else {
                let a = make_sphere(&mut topo, 6.0, 24).unwrap();
                let b = make_cylinder(&mut topo, 3.0, 20.0).unwrap();
                transform_solid(&mut topo, b, &Mat4::translation(2.0, 0.0, -10.0)).unwrap();
                (a, b)
            };
            let counts = |t: &Topology| {
                (
                    t.num_vertices(),
                    t.num_edges(),
                    t.num_wires(),
                    t.num_faces(),
                    t.num_shells(),
                    t.num_solids(),
                )
            };
            let before = counts(&topo);
            let ctx = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
            assert!(matches!(
                boolean_with_context(&mut topo, op, a, b, &ctx),
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
            ));
            assert_eq!(counts(&topo), before);
            for operand in [a, b] {
                assert!(
                    remus_operations::validate::validate_solid(&topo, operand)
                        .unwrap()
                        .is_valid()
                );
            }
        }
    }
}
