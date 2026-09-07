//! Boundary-aware resize of the captured quarter-cylinder wall.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::{
    classify::{PointClassification, classify_point},
    measure::solid_volume,
    push_pull::resize_cylindrical_face,
    tessellate::{tessellate_solid, welded_mesh_quality},
    validate::validate_solid,
};
use remus_topology::{Topology, explorer::solid_faces, face::FaceSurface};

const FIXTURE: &str = include_str!("data/jolly_fox_partial_cylinder.step");

fn expected_volume(radius: f64) -> f64 {
    73.0 * 41.0 * 8.0 + 12.0 * 30.5 * 12.0 + std::f64::consts::PI * radius * radius * 12.0 / 4.0
}

fn qualify(radius: f64, use_replacement: bool) {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(FIXTURE, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    let source = solids[0];
    let report = validate_solid(&topo, source).unwrap();
    assert_eq!((report.error_count(), report.warning_count()), (0, 0));
    assert!((solid_volume(&topo, source, 0.01).unwrap() - expected_volume(20.5)).abs() < 1e-6);
    let selected = solid_faces(&topo, source)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
    let result = if use_replacement {
        let FaceSurface::Cylinder(cylinder) = topo.face(selected).unwrap().surface() else {
            unreachable!()
        };
        let replacement = remus_math::surfaces::CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            radius,
            cylinder.x_axis(),
        )
        .unwrap();
        remus_operations::replace_surface::replace_surface(
            &mut topo,
            source,
            selected,
            FaceSurface::Cylinder(replacement),
        )
        .unwrap()
        .solid
    } else {
        resize_cylindrical_face(&mut topo, source, selected, radius).unwrap()
    };
    assert_eq!(
        before,
        remus_io::arena_io::serialize_solid(&topo, source).unwrap()
    );
    let report = validate_solid(&topo, result).unwrap();
    assert_eq!((report.error_count(), report.warning_count()), (0, 0));
    assert!((solid_volume(&topo, result, 0.01).unwrap() - expected_volume(radius)).abs() < 1e-6);
    let cylinders: Vec<_> = solid_faces(&topo, result)
        .unwrap()
        .into_iter()
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder) => Some(cylinder),
            _ => None,
        })
        .collect();
    assert_eq!(cylinders.len(), 1);
    assert!((cylinders[0].axis() - remus_math::vec::Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9);
    let wall = solid_faces(&topo, result)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    assert!(
        (remus_operations::measure::face_area(&topo, wall, 0.005).unwrap()
            - std::f64::consts::PI * radius * 12.0 / 2.0)
            .abs()
            < 1e-6
    );
    let levels: Vec<_> = topo
        .wire(topo.face(wall).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .flat_map(|oriented| {
            let edge = topo.edge(oriented.edge()).unwrap();
            [edge.start(), edge.end()]
        })
        .map(|vertex| topo.vertex(vertex).unwrap().point().z())
        .collect();
    assert!((levels.iter().copied().fold(f64::INFINITY, f64::min) - 8.0).abs() < 1e-9);
    assert!((levels.iter().copied().fold(f64::NEG_INFINITY, f64::max) - 20.0).abs() < 1e-9);
    assert!((cylinders[0].radius() - radius).abs() < 1e-9);
    assert!((cylinders[0].origin() - Point3::new(61.0, 41.0, 0.0)).length() < 1e-9);
    for (point, inside) in [
        (Point3::new(10.0, 10.0, 4.0), true),
        (Point3::new(67.0, 20.0, 14.0), true),
        (Point3::new(10.0, 10.0, 14.0), false),
        (Point3::new(50.0, 45.0, 14.0), false),
        (Point3::new(50.0, 30.0, 22.0), false),
        (
            Point3::new(
                61.0 - (radius + 20.5) / (2.0 * 2.0_f64.sqrt()),
                41.0 - (radius + 20.5) / (2.0 * 2.0_f64.sqrt()),
                14.0,
            ),
            radius > 20.5,
        ),
    ] {
        assert_eq!(
            classify_point(&topo, result, point, 0.005, 1e-7).unwrap(),
            if inside {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            }
        );
    }
    for deflection in [0.005, 0.02] {
        let mesh = tessellate_solid(&topo, result, deflection).unwrap();
        assert!(welded_mesh_quality(&mesh).is_watertight());
    }
    let step = remus_io::step::writer::write_step(&topo, &[result]).unwrap();
    let mut round_topo = Topology::new();
    let round = remus_io::step::reader::read_step(&step, &mut round_topo).unwrap();
    assert_eq!(round.len(), 1);
    let round_cylinders: Vec<_> = solid_faces(&round_topo, round[0])
        .unwrap()
        .into_iter()
        .filter_map(|face| match round_topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder) => Some(cylinder),
            _ => None,
        })
        .collect();
    assert_eq!(round_cylinders.len(), 1);
    assert!((round_cylinders[0].radius() - radius).abs() < 1e-9);
    let report = validate_solid(&round_topo, round[0]).unwrap();
    assert_eq!((report.error_count(), report.warning_count()), (0, 0));
    assert!(
        (solid_volume(&round_topo, round[0], 0.01).unwrap() - expected_volume(radius)).abs() < 1e-6
    );
}

#[test]
fn quarter_cylinder_small_inward_resize() {
    qualify(20.0, false);
}

#[test]
fn quarter_cylinder_small_outward_resize() {
    qualify(21.0, false);
}

#[test]
fn quarter_cylinder_support_replacement_inward() {
    qualify(20.0, true);
}

#[test]
fn quarter_cylinder_support_replacement_outward() {
    qualify(21.0, true);
}

#[test]
fn quarter_cylinder_scaled_and_placed() {
    use remus_math::mat::Mat4;
    for scale in [0.1_f64, 1.0, 10.0] {
        for placed in [false, true] {
            for radius in [20.0, 21.0, 22.0, 28.0] {
                let mut topo = Topology::new();
                let source = remus_io::step::reader::read_step(FIXTURE, &mut topo).unwrap()[0];
                let placement = if placed {
                    Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                        * Mat4::rotation_y(0.37)
                        * Mat4::rotation_z(0.51)
                } else {
                    Mat4::identity()
                };
                let matrix = placement * Mat4::scale(scale, scale, scale);
                remus_operations::transform::transform_solid(&mut topo, source, &matrix).unwrap();
                let selected = solid_faces(&topo, source)
                    .unwrap()
                    .into_iter()
                    .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
                    .unwrap();
                let result = resize_cylindrical_face(&mut topo, source, selected, radius * scale)
                    .unwrap_or_else(|error| {
                        panic!("scale={scale} placed={placed} radius={radius}: {error}")
                    });
                let report = validate_solid(&topo, result).unwrap();
                assert_eq!((report.error_count(), report.warning_count()), (0, 0));
                let expected = expected_volume(radius) * scale.powi(3);
                assert!(
                    (solid_volume(&topo, result, 0.01 * scale).unwrap() - expected).abs()
                        < expected * 1e-9
                );
                for deflection in [0.005, 0.02] {
                    assert!(
                        welded_mesh_quality(
                            &tessellate_solid(&topo, result, deflection * scale).unwrap()
                        )
                        .is_watertight()
                    );
                }
                let step = remus_io::step::writer::write_step(&topo, &[result]).unwrap();
                let mut round_topo = Topology::new();
                let round = remus_io::step::reader::read_step(&step, &mut round_topo).unwrap();
                assert_eq!(round.len(), 1);
                assert!(
                    (solid_volume(&round_topo, round[0], 0.01 * scale).unwrap() - expected).abs()
                        < expected * 1e-9
                );
            }
        }
    }
}

#[test]
fn quarter_cylinder_shoulder_collision_rolls_back() {
    for radius in [30.5, 32.0] {
        let mut topo = Topology::new();
        let source = remus_io::step::reader::read_step(FIXTURE, &mut topo).unwrap()[0];
        let selected = solid_faces(&topo, source)
            .unwrap()
            .into_iter()
            .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
        let slots = topo.allocated_slot_count();
        let error = resize_cylindrical_face(&mut topo, source, selected, radius).unwrap_err();
        assert!(
            matches!(error, remus_operations::OperationsError::Offset(_)),
            "{error}"
        );
        assert!(
            error
                .to_string()
                .contains("quarter-wall sweep may contact a nonadjacent source face"),
            "{error}"
        );
        assert_eq!(slots, topo.allocated_slot_count());
        assert_eq!(
            before,
            remus_io::arena_io::serialize_solid(&topo, source).unwrap()
        );
    }
}

#[test]
fn quarter_cylinder_face_interior_collision_rolls_back() {
    use remus_math::{
        context::{FallbackPolicy, OperationContext},
        mat::Mat4,
    };
    use remus_operations::{
        boolean::{BooleanOp, boolean_with_context},
        primitives::make_box,
        transform::transform_solid,
    };
    let mut topo = Topology::new();
    let source = remus_io::step::reader::read_step(FIXTURE, &mut topo).unwrap()[0];
    let obstacle = make_box(&mut topo, 0.4, 40.0, 30.0).unwrap();
    transform_solid(&mut topo, obstacle, &Mat4::translation(39.8, 10.0, 0.0)).unwrap();
    let source = boolean_with_context(
        &mut topo,
        BooleanOp::Fuse,
        source,
        obstacle,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap()
    .solid;
    let report = validate_solid(&topo, source).unwrap();
    assert_eq!((report.error_count(), report.warning_count()), (0, 0));
    assert!(
        (solid_volume(&topo, source, 0.01).unwrap() - expected_volume(20.5) - 380.8).abs() < 1e-6
    );
    let selected = solid_faces(&topo, source)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
    let slots = topo.allocated_slot_count();
    let error = resize_cylindrical_face(&mut topo, source, selected, 21.0).unwrap_err();
    assert!(
        matches!(error, remus_operations::OperationsError::Offset(_)),
        "{error}"
    );
    assert!(
        error
            .to_string()
            .contains("quarter-wall sweep may contact a nonadjacent source face"),
        "{error}"
    );
    assert_eq!(
        before,
        remus_io::arena_io::serialize_solid(&topo, source).unwrap()
    );
    assert_eq!(slots, topo.allocated_slot_count());
}

#[test]
fn half_cylinder_resize_refuses_without_mutation() {
    use remus_math::vec::Vec3;
    let mut topo = Topology::new();
    let wire = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 12.0),
            Point3::new(0.0, 0.0, 12.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = remus_topology::builder::make_planar_face_from_wire(&mut topo, wire).unwrap();
    let source = remus_operations::revolve::revolve(
        &mut topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::PI,
    )
    .unwrap();
    let report = validate_solid(&topo, source).unwrap();
    assert_eq!((report.error_count(), report.warning_count()), (0, 0));
    let selected = solid_faces(&topo, source)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
    let slots = topo.allocated_slot_count();
    let error = resize_cylindrical_face(&mut topo, source, selected, 4.5).unwrap_err();
    assert!(
        matches!(error, remus_operations::OperationsError::Offset(_)),
        "{error}"
    );
    assert!(
        error
            .to_string()
            .contains("quarter-wall boundaries must be lines and circles"),
        "{error}"
    );
    assert_eq!(
        before,
        remus_io::arena_io::serialize_solid(&topo, source).unwrap()
    );
    assert_eq!(slots, topo.allocated_slot_count());
}
