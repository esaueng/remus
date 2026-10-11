//! Issue #984: a centered floor bore must not turn a cup into a narrow pipe
//! in the GFA analytic classifier, or silently add already-contained material.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_algo::FaceClass;
use remus_algo::classifier::{
    AnalyticClassifier, RayCastGeoms, classify_point, classify_point_cached,
    try_build_analytic_classifier,
};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn placed_box(topo: &mut Topology, position: [f64; 3], size: [f64; 3]) -> SolidId {
    let solid = make_box(topo, size[0], size[1], size[2]).unwrap();
    transform_solid(
        topo,
        solid,
        &Mat4::translation(position[0], position[1], position[2]),
    )
    .unwrap();
    solid
}

fn cup(topo: &mut Topology, hole_x: Option<f64>) -> SolidId {
    let outer = placed_box(topo, [0., 0., 0.], [20., 20., 10.]);
    let cavity = placed_box(topo, [2., 2., 2.], [16., 16., 10.]);
    let cup = boolean(topo, BooleanOp::Cut, outer, cavity).unwrap();
    let Some(x) = hole_x else { return cup };
    let drill = make_cylinder(topo, 3., 4.).unwrap();
    transform_solid(topo, drill, &Mat4::translation(x, 10., -1.)).unwrap();
    boolean(topo, BooleanOp::Cut, cup, drill).unwrap()
}

fn assert_volume(topo: &Topology, solid: SolidId, expected: f64) {
    let actual = solid_volume(topo, solid, 0.001).unwrap();
    assert!(
        (actual - expected).abs() < 1e-7,
        "volume {actual} != {expected}"
    );
    let validation = validate_solid(topo, solid).unwrap();
    assert!(validation.is_valid(), "{:#?}", validation.issues);
}

#[test]
fn gfa_classifies_cup_material_and_voids_with_a_centered_floor_bore() {
    for hole_x in [None, Some(10.), Some(5.)] {
        let mut topo = Topology::new();
        let cup = cup(&mut topo, hole_x);
        if hole_x.is_none() {
            assert!(matches!(
                try_build_analytic_classifier(&topo, cup),
                Some(AnalyticClassifier::Composite { .. })
            ));
        }
        let cache = RayCastGeoms::new(&topo, cup).unwrap();
        for (point, expected) in [
            (Point3::new(0.5, 8., 5.), FaceClass::Inside),
            (Point3::new(19.5, 8., 5.), FaceClass::Inside),
            (Point3::new(8., 0.5, 5.), FaceClass::Inside),
            (Point3::new(8., 19.5, 5.), FaceClass::Inside),
            (Point3::new(3., 3., 1.), FaceClass::Inside),
            (Point3::new(10., 10., 5.), FaceClass::Outside),
            (Point3::new(-1., 8., 5.), FaceClass::Outside),
            (
                Point3::new(hole_x.unwrap_or(10.), 10., 1.),
                if hole_x.is_some() {
                    FaceClass::Outside
                } else {
                    FaceClass::Inside
                },
            ),
        ] {
            assert_eq!(
                classify_point(&topo, cup, point).unwrap(),
                expected,
                "hole_x={hole_x:?}, point={point:?}"
            );
            assert_eq!(
                classify_point_cached(&topo, cup, Some(&cache), point).unwrap(),
                expected,
                "cached hole_x={hole_x:?}, point={point:?}"
            );
        }
    }
}

fn check_wall_booleans(contained: bool) {
    let cup_volume = 1952. - 18. * std::f64::consts::PI;
    for placement in [
        Mat4::identity(),
        Mat4::translation(37., -19., 11.),
        Mat4::translation(37., -19., 11.) * Mat4::rotation_x(0.37) * Mat4::rotation_y(0.28),
    ] {
        for swapped in [false, true] {
            for operation in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                // Difference is directional, so only commutative operations swap operands.
                if swapped && matches!(operation, BooleanOp::Cut) {
                    continue;
                }
                let mut topo = Topology::new();
                let cup = cup(&mut topo, Some(10.));
                assert_volume(&topo, cup, cup_volume);
                let (x, width) = if contained { (0.5, 1.) } else { (-3., 4.) };
                let tool = placed_box(&mut topo, [x, 5., 3.], [width, 6., 4.]);
                transform_solid(&mut topo, cup, &placement).unwrap();
                transform_solid(&mut topo, tool, &placement).unwrap();
                let (a, b) = if swapped { (tool, cup) } else { (cup, tool) };
                let result = boolean(&mut topo, operation, a, b).unwrap();
                let expected = match operation {
                    BooleanOp::Fuse => cup_volume + if contained { 0. } else { 72. },
                    BooleanOp::Cut => cup_volume - 24.,
                    BooleanOp::Intersect => 24.,
                };
                assert_volume(&topo, result, expected);
            }
        }
    }
}

#[test]
fn centered_floor_bore_preserves_exact_overlapping_wall_booleans() {
    check_wall_booleans(false);
}

#[test]
fn centered_floor_bore_does_not_add_contained_tool_volume() {
    check_wall_booleans(true);
}

#[test]
fn centered_floor_boss_does_not_exclude_cup_walls() {
    let mut topo = Topology::new();
    let cup = cup(&mut topo, None);
    let boss = make_cylinder(&mut topo, 3., 4.).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(10., 10., 2.)).unwrap();
    let cup = boolean(&mut topo, BooleanOp::Fuse, cup, boss).unwrap();
    assert_volume(&topo, cup, 1952. + 36. * std::f64::consts::PI);
    for (point, expected) in [
        (Point3::new(0.5, 8., 5.), FaceClass::Inside),
        (Point3::new(10., 10., 3.), FaceClass::Inside),
        (Point3::new(15., 10., 3.), FaceClass::Outside),
    ] {
        assert_eq!(classify_point(&topo, cup, point).unwrap(), expected);
    }
    let tool = placed_box(&mut topo, [0.5, 5., 3.], [1., 6., 4.]);
    let result = boolean(&mut topo, BooleanOp::Fuse, cup, tool).unwrap();
    assert_volume(&topo, result, 1952. + 36. * std::f64::consts::PI);
}
