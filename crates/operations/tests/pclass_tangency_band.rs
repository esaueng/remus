//! Four-wall contacts must not lose cylinder or box material near tangency.
#![allow(clippy::unwrap_used, clippy::panic)]

use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
    vec::Point3,
};
use remus_operations::{
    boolean::{BooleanOp, boolean_with_context},
    classify::{PointClassification, classify_point},
    primitives::{make_box, make_cylinder},
    transform::transform_solid,
};
use remus_topology::Topology;

fn qualify(operation: BooleanOp, epsilon: f64) {
    let _ = env_logger::try_init();
    for scale in [0.1_f64, 1.0, 10.0] {
        for placed in [false, true] {
            let mut topo = Topology::new();
            let half = 4.0 + epsilon;
            let cylinder = make_cylinder(&mut topo, 4.0 * scale, 12.0 * scale).unwrap();
            let tool = make_box(
                &mut topo,
                2.0 * half * scale,
                2.0 * half * scale,
                8.0 * scale,
            )
            .unwrap();
            transform_solid(
                &mut topo,
                tool,
                &Mat4::translation(-half * scale, -half * scale, 6.0 * scale),
            )
            .unwrap();
            let placement = if placed {
                Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                    * Mat4::rotation_y(0.37)
            } else {
                Mat4::identity()
            };
            if placed {
                for solid in [cylinder, tool] {
                    transform_solid(&mut topo, solid, &placement).unwrap();
                }
            }
            let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
            let label = format!("{operation:?} epsilon={epsilon:e} scale={scale} placed={placed}");
            let operand_bytes =
                remus_io::arena_io::serialize_solids(&topo, &[cylinder, tool]).unwrap();
            let before = (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
            );
            let result = match boolean_with_context(&mut topo, operation, cylinder, tool, &context)
            {
                Ok(result) => result,
                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
                    if epsilon != 0.0 && epsilon.abs() <= 1e-7 =>
                {
                    // The tangency exit contract permits typed refusal, but
                    // an unsuccessful operation must not mutate its operands.
                    assert_eq!(
                        before,
                        (
                            topo.num_vertices(),
                            topo.num_edges(),
                            topo.num_wires(),
                            topo.num_faces(),
                            topo.num_shells(),
                            topo.num_solids(),
                        ),
                        "{label}: refusal leaked topology"
                    );
                    assert_eq!(
                        operand_bytes,
                        remus_io::arena_io::serialize_solids(&topo, &[cylinder, tool]).unwrap(),
                        "{label}: refusal changed operand bytes"
                    );
                    for operand in [cylinder, tool] {
                        assert!(
                            remus_operations::validate::validate_solid(&topo, operand)
                                .unwrap()
                                .is_valid(),
                            "{label}: refusal damaged operand"
                        );
                    }
                    log::info!("TANGENCY_REFUSAL {label}");
                    continue;
                }
                Err(error) => panic!("{label}: {error}"),
            };
            let validation =
                remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
            assert!(validation.is_valid(), "{label}: {validation:?}");
            for face in remus_topology::explorer::solid_faces(&topo, result.solid).unwrap() {
                assert!(
                    matches!(
                        topo.face(face).unwrap().surface(),
                        remus_topology::face::FaceSurface::Plane { .. }
                            | remus_topology::face::FaceSurface::Cylinder(_)
                    ),
                    "{label}: non-analytic face"
                );
            }
            for face_id in remus_topology::explorer::solid_faces(&topo, result.solid).unwrap() {
                let face = topo.face(face_id).unwrap();
                if let remus_topology::face::FaceSurface::Plane { normal, d } = face.surface() {
                    for wire_id in
                        std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
                    {
                        for oriented in topo.wire(wire_id).unwrap().edges() {
                            let edge = topo.edge(oriented.edge()).unwrap();
                            if let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve() {
                                let (lo, hi) = edge.trim().unwrap();
                                let point = circle.evaluate(f64::midpoint(lo, hi));
                                let residual =
                                    (normal.dot(point - Point3::new(0.0, 0.0, 0.0)) - d).abs();
                                assert!(
                                    residual <= 1e-7,
                                    "{label}: plane {face_id:?} edge {:?} carrier residual {residual}",
                                    oriented.edge()
                                );
                            }
                        }
                    }
                }
            }
            // Four non-overlapping circular caps are clipped when the square is smaller.
            let cap = if epsilon < 0.0 {
                16.0 * (half / 4.0).acos() - half * (16.0 - half * half).sqrt()
            } else {
                0.0
            };
            let overlap = 6.0 * (16.0 * std::f64::consts::PI - 4.0 * cap);
            let cylinder_volume = 192.0 * std::f64::consts::PI;
            let box_volume = 32.0 * half * half;
            let expected = match operation {
                BooleanOp::Fuse => cylinder_volume + box_volume - overlap,
                BooleanOp::Cut => cylinder_volume - overlap,
                BooleanOp::Intersect => overlap,
            } * scale.powi(3);
            let volume =
                remus_operations::measure::solid_volume(&topo, result.solid, 0.005 * scale)
                    .unwrap();
            let budget = (expected.abs() * 1e-8).max(512.0 * scale * scale * 1e-7);
            let mut probes = vec![
                (
                    Point3::new(0.0, 0.0, 3.0),
                    !matches!(operation, BooleanOp::Intersect),
                ),
                (
                    Point3::new(0.0, 0.0, 9.0),
                    !matches!(operation, BooleanOp::Cut),
                ),
                (
                    Point3::new(0.0, 0.0, 13.0),
                    matches!(operation, BooleanOp::Fuse),
                ),
            ];
            if epsilon * scale < -16.0 * 1e-7 {
                let radius = f64::midpoint(4.0, half);
                for (x, y) in [(radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius)] {
                    probes.push((
                        Point3::new(x, y, 9.0),
                        !matches!(operation, BooleanOp::Intersect),
                    ));
                }
            }
            for (point, inside) in probes {
                let point = placement.mul_point(Point3::new(
                    point.x() * scale,
                    point.y() * scale,
                    point.z() * scale,
                ));
                let actual =
                    classify_point(&topo, result.solid, point, 0.005 * scale, 1e-7).unwrap();
                let expected = if inside {
                    PointClassification::Inside
                } else {
                    PointClassification::Outside
                };
                assert_eq!(actual, expected, "{label}: material at {point:?}");
            }
            for deflection in [0.005, 0.02] {
                let mesh = remus_operations::tessellate::tessellate_solid(
                    &topo,
                    result.solid,
                    deflection * scale,
                )
                .unwrap();
                let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
                assert!(
                    quality.is_watertight(),
                    "{label} deflection={deflection}: {quality:?}"
                );
            }
            assert!(
                (volume - expected).abs() <= budget,
                "{label}: volume {volume} vs {expected}, budget {budget}"
            );
        }
    }
}

#[test]
fn cylinder_box_fuse_n3() {
    qualify(BooleanOp::Fuse, -1e-3);
}

#[test]
fn cylinder_box_fuse_n5() {
    qualify(BooleanOp::Fuse, -1e-5);
}

#[test]
fn cylinder_box_fuse_n7() {
    qualify(BooleanOp::Fuse, -1e-7);
}

#[test]
fn cylinder_box_fuse_n9() {
    qualify(BooleanOp::Fuse, -1e-9);
}

#[test]
fn cylinder_box_fuse_zero() {
    qualify(BooleanOp::Fuse, 0.0);
}

#[test]
fn cylinder_box_fuse_p9() {
    qualify(BooleanOp::Fuse, 1e-9);
}

#[test]
fn cylinder_box_fuse_p7() {
    qualify(BooleanOp::Fuse, 1e-7);
}

#[test]
fn cylinder_box_fuse_p5() {
    qualify(BooleanOp::Fuse, 1e-5);
}

#[test]
fn cylinder_box_fuse_p3() {
    qualify(BooleanOp::Fuse, 1e-3);
}

#[test]
fn cylinder_box_cut_n3() {
    qualify(BooleanOp::Cut, -1e-3);
}

#[test]
fn cylinder_box_cut_n5() {
    qualify(BooleanOp::Cut, -1e-5);
}

#[test]
fn cylinder_box_cut_n7() {
    qualify(BooleanOp::Cut, -1e-7);
}

#[test]
fn cylinder_box_cut_n9() {
    qualify(BooleanOp::Cut, -1e-9);
}

#[test]
fn cylinder_box_cut_zero() {
    qualify(BooleanOp::Cut, 0.0);
}

#[test]
fn cylinder_box_cut_p9() {
    qualify(BooleanOp::Cut, 1e-9);
}

#[test]
fn cylinder_box_cut_p7() {
    qualify(BooleanOp::Cut, 1e-7);
}

#[test]
fn cylinder_box_cut_p5() {
    qualify(BooleanOp::Cut, 1e-5);
}

#[test]
fn cylinder_box_cut_p3() {
    qualify(BooleanOp::Cut, 1e-3);
}

#[test]
fn cylinder_box_intersect_n3() {
    qualify(BooleanOp::Intersect, -1e-3);
}

#[test]
fn cylinder_box_intersect_n5() {
    qualify(BooleanOp::Intersect, -1e-5);
}

#[test]
fn cylinder_box_intersect_n7() {
    qualify(BooleanOp::Intersect, -1e-7);
}

#[test]
fn cylinder_box_intersect_n9() {
    qualify(BooleanOp::Intersect, -1e-9);
}

#[test]
fn cylinder_box_intersect_zero() {
    qualify(BooleanOp::Intersect, 0.0);
}

#[test]
fn cylinder_box_intersect_p9() {
    qualify(BooleanOp::Intersect, 1e-9);
}

#[test]
fn cylinder_box_intersect_p7() {
    qualify(BooleanOp::Intersect, 1e-7);
}

#[test]
fn cylinder_box_intersect_p5() {
    qualify(BooleanOp::Intersect, 1e-5);
}

#[test]
fn cylinder_box_intersect_p3() {
    qualify(BooleanOp::Intersect, 1e-3);
}
