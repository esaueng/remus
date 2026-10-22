//! F17: exact conic sections must partition both complete frustum walls.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::primitives::make_cone;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

/// Integrate the overlap of two horizontal radius-r discs, separated by d.
/// This derives quantity targets without the cone/conic construction under test.
fn quantities() -> (f64, f64) {
    let (bottom, top, height, half_distance) = (4.5_f64, 2.5_f64, 5.5_f64, 3.5_f64);
    let slope = (bottom - top) / height;
    let chord = (bottom * bottom - half_distance * half_distance).sqrt();
    let angle = (half_distance / bottom).acos();
    let lens = 2.0 * (bottom * bottom * angle - half_distance * chord);
    let overlap = 2.0 / (3.0 * slope)
        * (bottom.powi(3) * angle - 2.0 * half_distance * bottom * chord
            + half_distance.powi(3) * (bottom / half_distance).acosh());
    let primitive_volume = PI * height * (bottom * bottom + bottom * top + top * top) / 3.0;
    let primitive_area =
        PI * (bottom + top) * height.hypot(bottom - top) + PI * (bottom * bottom + top * top);
    (
        2.0 * primitive_volume - overlap,
        2.0 * primitive_area - lens * (1.0 + 1.0_f64.hypot(slope) / slope),
    )
}

#[test]
fn disc_overlap_integral_independently_checks_the_closed_form() {
    let n = 10_000;
    let step = 2.75 / f64::from(n);
    let area = |z: f64| {
        let r = 4.5 - 4.0 * z / 11.0;
        2.0 * (r * r * (3.5 / r).acos() - 3.5 * (r * r - 12.25).max(0.0).sqrt())
    };
    let overlap = step / 3.0
        * (area(0.0)
            + area(2.75)
            + (1..n)
                .map(|i| if i % 2 == 0 { 2.0 } else { 4.0 } * area(f64::from(i) * step))
                .sum::<f64>());
    let primitive = PI * 5.5 * (4.5_f64.powi(2) + 4.5 * 2.5 + 2.5_f64.powi(2)) / 3.0;
    assert!((2.0 * primitive - overlap - quantities().0).abs() < 1e-9);
}

#[test]
#[allow(clippy::too_many_lines)]
fn complete_equal_frustum_fuse_accepts_and_refuses_the_declared_matrix_atomically() {
    let (expected_volume, expected_area) = quantities();
    let mut accepted = 0;
    let mut refused = 0;
    for mirrored in [false, true] {
        for scale in [1e-3_f64, 1.0, 1e3] {
            for placed in [false, true] {
                for swapped in [false, true] {
                    let label = format!(
                        "mirrored={mirrored} scale={scale} placed={placed} swapped={swapped}"
                    );
                    let mut topo = Topology::new();
                    let a = make_cone(&mut topo, 4.5 * scale, 2.5 * scale, 5.5 * scale).unwrap();
                    let b = if mirrored {
                        remus_operations::mirror::mirror(
                            &mut topo,
                            a,
                            Point3::new(3.5 * scale, 0.0, 0.0),
                            Vec3::new(1.0, 0.0, 0.0),
                        )
                        .unwrap()
                    } else {
                        let b =
                            make_cone(&mut topo, 4.5 * scale, 2.5 * scale, 5.5 * scale).unwrap();
                        transform_solid(&mut topo, b, &Mat4::translation(7.0 * scale, 0.0, 0.0))
                            .unwrap();
                        b
                    };
                    let pose = if placed {
                        Mat4::translation(13.0 * scale, -7.0 * scale, 9.0 * scale)
                            * Mat4::rotation_x(0.37)
                            * Mat4::rotation_z(0.23)
                    } else {
                        Mat4::identity()
                    };
                    if placed {
                        for solid in [a, b] {
                            transform_solid(&mut topo, solid, &pose).unwrap();
                        }
                    }
                    let original = remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap();
                    let counts = (
                        topo.num_vertices(),
                        topo.num_edges(),
                        topo.num_wires(),
                        topo.num_faces(),
                        topo.num_shells(),
                        topo.num_solids(),
                    );
                    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
                    let (lhs, rhs) = if swapped { (b, a) } else { (a, b) };
                    let result =
                        boolean_with_context(&mut topo, BooleanOp::Fuse, lhs, rhs, &context);
                    // The two tiny translated and all twelve rotated cells are
                    // retained limits, not a promise of general X10 closure.
                    if placed || (!mirrored && scale.to_bits() == 1e-3_f64.to_bits()) {
                        assert!(
                            matches!(
                                result,
                                Err(remus_operations::OperationsError::ExactOnlyUnattainable)
                            ),
                            "{label}: {result:?}"
                        );
                        assert_eq!(
                            counts,
                            (
                                topo.num_vertices(),
                                topo.num_edges(),
                                topo.num_wires(),
                                topo.num_faces(),
                                topo.num_shells(),
                                topo.num_solids()
                            ),
                            "{label}: rollback"
                        );
                        refused += 1;
                    } else {
                        let result = result.unwrap_or_else(|e| panic!("{label}: {e:?}"));
                        assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
                        let report =
                            remus_operations::validate::validate_solid(&topo, result.solid)
                                .unwrap();
                        assert!(report.is_valid(), "{label}: {:?}", report.issues);
                        let mut uses = BTreeMap::new();
                        let mut cones = 0;
                        let faces = solid_faces(&topo, result.solid).unwrap();
                        assert!(faces.len() <= 10, "{label}: faceted output");
                        for fid in faces {
                            let face = topo.face(fid).unwrap();
                            assert!(
                                matches!(
                                    face.surface(),
                                    FaceSurface::Cone(_) | FaceSurface::Plane { .. }
                                ),
                                "{label}"
                            );
                            cones += usize::from(matches!(face.surface(), FaceSurface::Cone(_)));
                            for wid in std::iter::once(face.outer_wire())
                                .chain(face.inner_wires().iter().copied())
                            {
                                for edge in topo.wire(wid).unwrap().edges() {
                                    uses.entry(edge.edge())
                                        .or_insert_with(Vec::new)
                                        .push(edge.is_forward() != face.is_reversed());
                                }
                            }
                        }
                        assert_eq!(cones, 2, "{label}: native carriers");
                        for directions in uses.values() {
                            assert_eq!(directions.len(), 2, "{label}: edge incidence");
                            assert_ne!(directions[0], directions[1], "{label}: edge winding");
                        }
                        for z in [0.5, 1.5, 3.0, 4.5] {
                            let radius = 4.5 - 4.0 * z / 11.0;
                            for x in [-3.0_f64, 0.0, 3.5, 7.0, 10.0] {
                                for y in [-2.0, 0.0, 2.0] {
                                    let in_a = x.hypot(y) < radius;
                                    let in_b = (x - 7.0_f64).hypot(y) < radius;
                                    let p = pose.mul_point(Point3::new(
                                        x * scale,
                                        y * scale,
                                        z * scale,
                                    ));
                                    for (solid, inside) in
                                        [(a, in_a), (b, in_b), (result.solid, in_a || in_b)]
                                    {
                                        assert_eq!(
                                            classify_point(&topo, solid, p, 0.01 * scale, 1e-7)
                                                .unwrap(),
                                            if inside {
                                                PointClassification::Inside
                                            } else {
                                                PointClassification::Outside
                                            },
                                            "{label}: material({x},{y},{z})"
                                        );
                                    }
                                }
                            }
                        }
                        for deflection in [1e-4, 5e-5] {
                            let volume = remus_operations::measure::solid_volume(
                                &topo,
                                result.solid,
                                deflection * scale,
                            )
                            .unwrap()
                                / scale.powi(3);
                            assert!(
                                (volume - expected_volume).abs() / expected_volume < 1e-4,
                                "{label}: volume {volume}"
                            );
                        }
                        let mass = remus_operations::measure::mass_properties(&topo, result.solid)
                            .unwrap()
                            .mass
                            / scale.powi(3);
                        assert!(
                            (mass - expected_volume).abs() / expected_volume < 1e-4,
                            "{label}: mass {mass}"
                        );
                        let area = remus_operations::measure::solid_surface_area(
                            &topo,
                            result.solid,
                            1e-4 * scale,
                        )
                        .unwrap()
                            / scale.powi(2);
                        assert!(
                            (area - expected_area).abs() / expected_area < 1e-4,
                            "{label}: area {area}"
                        );
                        for deflection in [0.1, 0.01] {
                            let mesh =
                                tessellate_solid(&topo, result.solid, deflection * scale).unwrap();
                            assert_eq!(boundary_edge_count(&mesh), 0, "{label}: raw mesh boundary");
                            assert_eq!(
                                non_manifold_edge_count(&mesh),
                                0,
                                "{label}: raw mesh incidence"
                            );
                            let origin = mesh.positions[0];
                            let signed: f64 = mesh
                                .indices
                                .chunks_exact(3)
                                .map(|tri| {
                                    let a = mesh.positions[tri[0] as usize] - origin;
                                    let b = mesh.positions[tri[1] as usize] - origin;
                                    let c = mesh.positions[tri[2] as usize] - origin;
                                    a.dot(b.cross(c)) / 6.0
                                })
                                .sum();
                            assert!(signed > 0.0, "{label}: inward raw mesh");
                            assert!(
                                (signed / scale.powi(3) - expected_volume).abs() / expected_volume
                                    < 0.01,
                                "{label}: raw mesh quantity"
                            );
                        }
                        accepted += 1;
                    }
                    assert_eq!(
                        remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap(),
                        original,
                        "{label}: operands changed"
                    );
                }
            }
        }
    }
    assert_eq!((accepted, refused), (10, 14));
}

#[test]
fn cancelled_equal_frustum_fuse_preserves_the_operand_state() {
    let mut topo = Topology::new();
    let a = make_cone(&mut topo, 4.5, 2.5, 5.5).unwrap();
    let b = make_cone(&mut topo, 4.5, 2.5, 5.5).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(7.0, 0.0, 0.0)).unwrap();
    let before = remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap();
    let solids = topo.num_solids();
    let token = remus_math::context::CancellationToken::new();
    token.cancel();
    let context = OperationContext::new()
        .with_fallback(FallbackPolicy::ExactOnly)
        .with_cancellation(token);
    assert!(matches!(
        boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &context),
        Err(remus_operations::OperationsError::Math(
            remus_math::MathError::Cancelled
        ))
    ));
    assert_eq!(topo.num_solids(), solids);
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap(),
        before
    );
}

#[test]
#[allow(clippy::too_many_lines)]
fn original_equal_frustum_complementary_operators_retain_material_and_quantities() {
    let (union_volume, union_area) = quantities();
    let primitive_volume = PI * 5.5 * (4.5_f64.powi(2) + 4.5 * 2.5 + 2.5_f64.powi(2)) / 3.0;
    let primitive_area = PI * 7.0 * 5.5_f64.hypot(2.0) + PI * (4.5_f64.powi(2) + 2.5_f64.powi(2));
    let overlap = 2.0 * primitive_volume - union_volume;
    let bottom_lens = 2.0 * (4.5_f64.powi(2) * (3.5_f64 / 4.5).acos() - 3.5 * 8.0_f64.sqrt());
    for mirrored in [false, true] {
        for swapped in [false, true] {
            for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                let label = format!("{op:?} mirrored={mirrored} swapped={swapped}");
                let mut topo = Topology::new();
                let a = make_cone(&mut topo, 4.5, 2.5, 5.5).unwrap();
                let b = if mirrored {
                    remus_operations::mirror::mirror(
                        &mut topo,
                        a,
                        Point3::new(3.5, 0.0, 0.0),
                        Vec3::new(1.0, 0.0, 0.0),
                    )
                    .unwrap()
                } else {
                    let b = make_cone(&mut topo, 4.5, 2.5, 5.5).unwrap();
                    transform_solid(&mut topo, b, &Mat4::translation(7.0, 0.0, 0.0)).unwrap();
                    b
                };
                let original = remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap();
                let (lhs, rhs) = if swapped { (b, a) } else { (a, b) };
                let result = boolean_with_context(
                    &mut topo,
                    op,
                    lhs,
                    rhs,
                    &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                )
                .unwrap();
                assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
                let report =
                    remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
                assert!(report.is_valid(), "{label}: {:?}", report.issues);
                let (volume, area) = match op {
                    BooleanOp::Fuse => (union_volume, union_area),
                    BooleanOp::Intersect => (overlap, 2.0 * primitive_area - union_area),
                    BooleanOp::Cut => (primitive_volume - overlap, primitive_area - bottom_lens),
                };
                for d in [1e-4, 5e-5] {
                    let actual =
                        remus_operations::measure::solid_volume(&topo, result.solid, d).unwrap();
                    assert!(
                        (actual - volume).abs() / volume < 1e-4,
                        "{label}: volume {actual} vs {volume}"
                    );
                }
                let actual = remus_operations::measure::mass_properties(&topo, result.solid)
                    .unwrap()
                    .mass;
                assert!(
                    (actual - volume).abs() / volume < 1e-4,
                    "{label}: mass {actual} vs {volume}"
                );
                let actual =
                    remus_operations::measure::solid_surface_area(&topo, result.solid, 1e-4)
                        .unwrap();
                assert!(
                    (actual - area).abs() / area < 1e-4,
                    "{label}: area {actual} vs {area}"
                );
                for z in [0.5, 1.5, 3.0, 4.5] {
                    for x in [-3.0_f64, 0.0, 3.5, 7.0, 10.0] {
                        for y in [-2.0, 0.0, 2.0] {
                            let radius = 4.5 - 4.0 * z / 11.0;
                            let (ia, ib) = (x.hypot(y) < radius, (x - 7.0).hypot(y) < radius);
                            let (il, ir) = if swapped { (ib, ia) } else { (ia, ib) };
                            let inside = match op {
                                BooleanOp::Fuse => il || ir,
                                BooleanOp::Cut => il && !ir,
                                BooleanOp::Intersect => il && ir,
                            };
                            assert_eq!(
                                classify_point(
                                    &topo,
                                    result.solid,
                                    Point3::new(x, y, z),
                                    0.01,
                                    1e-7
                                )
                                .unwrap(),
                                if inside {
                                    PointClassification::Inside
                                } else {
                                    PointClassification::Outside
                                },
                                "{label}: material {x},{y},{z}"
                            );
                        }
                    }
                }
                for d in [0.1, 0.01] {
                    let mesh = tessellate_solid(&topo, result.solid, d).unwrap();
                    assert_eq!(boundary_edge_count(&mesh), 0, "{label}: raw mesh boundary");
                    assert_eq!(
                        non_manifold_edge_count(&mesh),
                        0,
                        "{label}: raw mesh incidence"
                    );
                    let origin = mesh.positions[0];
                    let signed: f64 = mesh
                        .indices
                        .chunks_exact(3)
                        .map(|tri| {
                            let a = mesh.positions[tri[0] as usize] - origin;
                            let b = mesh.positions[tri[1] as usize] - origin;
                            let c = mesh.positions[tri[2] as usize] - origin;
                            a.dot(b.cross(c)) / 6.0
                        })
                        .sum();
                    assert!(signed > 0.0, "{label}: inward raw mesh");
                }
                assert_eq!(
                    remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap(),
                    original,
                    "{label}: operand preservation"
                );
            }
        }
    }
}
