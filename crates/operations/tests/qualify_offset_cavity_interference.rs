//! Separate cavity collision from outer-wall collapse at the public offset entry.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::{mat::Mat4, vec::Point3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::offset_v2::offset_solid_v2;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::tessellate_solid;
use remus_operations::transform::transform_solid;
use remus_topology::{Topology, solid::SolidId};

fn two_cavities(topo: &mut Topology, scale: f64, reverse: bool) -> SolidId {
    let mut solid = make_box(topo, 20.0 * scale, 12.0 * scale, 12.0 * scale).unwrap();
    for x in if reverse { [10.0, 6.0] } else { [6.0, 10.0] } {
        let void = make_box(topo, 2.0 * scale, 2.0 * scale, 2.0 * scale).unwrap();
        transform_solid(
            topo,
            void,
            &Mat4::translation(x * scale, 5.0 * scale, 5.0 * scale),
        )
        .unwrap();
        solid = boolean(topo, BooleanOp::Cut, solid, void).unwrap();
    }
    assert_eq!(topo.solid(solid).unwrap().inner_shells().len(), 2);
    solid
}

#[test]
fn separated_cavities_keep_material_and_collisions_refuse_without_mutation() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        for reverse in [false, true] {
            for shift in [[0.0, 0.0, 0.0], [17.0, -23.0, 31.0]] {
                let mut topo = Topology::new();
                let source = two_cavities(&mut topo, scale, reverse);
                transform_solid(
                    &mut topo,
                    source,
                    &Mat4::translation(shift[0] * scale, shift[1] * scale, shift[2] * scale),
                )
                .unwrap();
                for distance in [0.5_f64, -0.75] {
                    let result = offset_solid_v2(&mut topo, source, distance * scale).unwrap();
                    assert_eq!(topo.solid(result).unwrap().inner_shells().len(), 2);
                    let expected = ((20.0 + 2.0 * distance) * (12.0 + 2.0 * distance).powi(2)
                        - 2.0 * (2.0 - 2.0 * distance).powi(3))
                        * scale.powi(3);
                    let mass = mass_properties(&topo, result).unwrap().mass;
                    assert!(
                        (mass - expected).abs() / expected < 1e-8,
                        "{mass} vs {expected}"
                    );
                    for deflection in [0.01, 0.001] {
                        let volume = solid_volume(&topo, result, deflection * scale).unwrap();
                        assert!((volume - expected).abs() / expected < 1e-8);
                        let mesh = tessellate_solid(&topo, result, deflection * scale).unwrap();
                        assert!(remus_operations::tessellate::is_watertight(&mesh));
                        assert!(
                            remus_operations::tessellate::welded_mesh_quality(&mesh)
                                .is_watertight()
                        );
                    }
                    for (x, expected_class) in [
                        (7.0, PointClassification::Outside),
                        (11.0, PointClassification::Outside),
                        (9.0, PointClassification::Inside),
                    ] {
                        let point = Point3::new(
                            (x + shift[0]) * scale,
                            (6.0 + shift[1]) * scale,
                            (6.0 + shift[2]) * scale,
                        );
                        assert_eq!(
                            classify_point(&topo, result, point, 0.01 * scale, 1e-7 * scale)
                                .unwrap(),
                            expected_class
                        );
                    }
                }
                for distance in [-1.0, -1.1] {
                    let before = topo.allocated_slot_count();
                    let shells = topo.solid(source).unwrap().inner_shells().to_vec();
                    let error = offset_solid_v2(&mut topo, source, distance * scale).unwrap_err();
                    assert!(
                        matches!(error, remus_operations::OperationsError::InvalidInput { ref reason }
                        if reason.contains("cavities to stay disjoint")),
                        "{error}"
                    );
                    assert_eq!(
                        topo.allocated_slot_count(),
                        before,
                        "collision must refuse before construction"
                    );
                    assert_eq!(topo.solid(source).unwrap().inner_shells(), shells);
                    let expected = (20.0 * 12.0 * 12.0 - 16.0) * scale.powi(3);
                    assert!(
                        (solid_volume(&topo, source, 0.01 * scale).unwrap() - expected).abs()
                            / expected
                            < 1e-8
                    );
                }
            }
        }
    }
}
