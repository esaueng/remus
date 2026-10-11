//! Issue #984: a centred floor bore must not turn cup wall material into void.
//!
//! The old analytic composite promoted the inward bore to an outer cylinder
//! because its vertex-average reference lay radially inside the carrier. That
//! constraint limited the whole cup to the floor's axial range and radius.
//! Trimmed rays classified these same points correctly. The oracles below use
//! inclusion/exclusion volumes, topology validation and welded mesh closure.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::{FaceClass, classifier};
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
    vec::Point3,
};
use remus_operations::{
    boolean::{BooleanOp, BooleanQuality, boolean_with_context},
    primitives::{make_box, make_cylinder},
    transform::transform_solid,
};
use remus_topology::{Topology, solid::SolidId};

fn placed_box(topo: &mut Topology, p: [f64; 3], size: [f64; 3]) -> SolidId {
    let s = make_box(topo, size[0], size[1], size[2]).unwrap();
    transform_solid(topo, s, &Mat4::translation(p[0], p[1], p[2])).unwrap();
    s
}

fn exact(topo: &mut Topology, op: BooleanOp, a: SolidId, b: SolidId) -> SolidId {
    let out = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap();
    assert_eq!(out.quality, BooleanQuality::Exact);
    out.solid
}

fn cup(topo: &mut Topology) -> SolidId {
    bored_cup(topo, [10.0, 10.0], 3.0, 4.0, 2.0)
}

fn plain_cup(topo: &mut Topology, floor: f64) -> SolidId {
    let outer = placed_box(topo, [0.0; 3], [20.0, 20.0, 10.0]);
    let cavity = placed_box(topo, [2.0, 2.0, floor], [16.0, 16.0, 10.0]);
    exact(topo, BooleanOp::Cut, outer, cavity)
}

fn bored_cup(
    topo: &mut Topology,
    center: [f64; 2],
    radius: f64,
    height: f64,
    floor: f64,
) -> SolidId {
    let cup = plain_cup(topo, floor);
    let bore = make_cylinder(topo, radius, height).unwrap();
    transform_solid(topo, bore, &Mat4::translation(center[0], center[1], -1.0)).unwrap();
    exact(topo, BooleanOp::Cut, cup, bore)
}

fn assert_volume(topo: &Topology, s: SolidId, expected: f64) {
    let report = remus_operations::validate::validate_solid(topo, s).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let volume = remus_operations::measure::mass_properties(topo, s)
        .unwrap()
        .mass;
    assert!(
        (volume - expected).abs() < expected * 1e-9,
        "volume {volume}, expected {expected}"
    );
    let mesh = remus_operations::tessellate::tessellate_solid(topo, s, 0.02).unwrap();
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "{quality:?}");
}

#[test]
fn gfa_classifies_bored_cup_material_and_void() {
    let mut topo = Topology::new();
    let cup = cup(&mut topo);
    assert!(classifier::try_build_analytic_classifier(&topo, cup).is_none());
    let geoms = classifier::RayCastGeoms::new(&topo, cup).unwrap();
    for (p, expected) in [
        ([0.5, 8.0, 5.0], FaceClass::Inside),
        ([19.5, 8.0, 5.0], FaceClass::Inside),
        ([8.0, 0.5, 5.0], FaceClass::Inside),
        ([8.0, 19.5, 5.0], FaceClass::Inside),
        ([5.0, 5.0, 1.0], FaceClass::Inside),
        ([10.0, 10.0, 1.0], FaceClass::Outside),
        ([10.0, 10.0, 5.0], FaceClass::Outside),
        ([-0.5, 8.0, 5.0], FaceClass::Outside),
    ] {
        let p = Point3::new(p[0], p[1], p[2]);
        assert_eq!(
            classifier::classify_ray_cast(&topo, cup, p).unwrap(),
            expected
        );
        assert_eq!(
            classifier::classify_ray_cast_cached(&geoms, p).unwrap(),
            expected
        );
        assert_eq!(
            classifier::classify_point_cached(&topo, cup, Some(&geoms), p).unwrap(),
            expected
        );
        assert_eq!(classifier::classify_point(&topo, cup, p).unwrap(), expected);
    }
}

#[test]
fn bored_cup_wall_overlap_booleans_are_exact() {
    for (op, delta) in [
        (BooleanOp::Fuse, 72.0),
        (BooleanOp::Cut, -24.0),
        (BooleanOp::Intersect, 24.0),
    ] {
        let mut topo = Topology::new();
        let cup = cup(&mut topo);
        let tool = placed_box(&mut topo, [-3.0, 5.0, 3.0], [4.0, 6.0, 4.0]);
        let result = exact(&mut topo, op, cup, tool);
        let expected = if op == BooleanOp::Intersect {
            delta
        } else {
            1952.0 - 18.0 * std::f64::consts::PI + delta
        };
        assert_volume(&topo, result, expected);
    }
}

#[test]
fn bored_cup_inside_wall_fuse_preserves_volume() {
    let mut topo = Topology::new();
    let cup = cup(&mut topo);
    let tool = placed_box(&mut topo, [0.5, 5.0, 3.0], [1.0, 6.0, 4.0]);
    let result = exact(&mut topo, BooleanOp::Fuse, cup, tool);
    assert_volume(&topo, result, 1952.0 - 18.0 * std::f64::consts::PI);
}

#[test]
fn bored_cup_contained_cut_and_intersect_are_exact() {
    for (op, expected) in [
        (BooleanOp::Cut, 1952.0 - 18.0 * std::f64::consts::PI - 24.0),
        (BooleanOp::Intersect, 24.0),
    ] {
        let mut topo = Topology::new();
        let cup = cup(&mut topo);
        let tool = placed_box(&mut topo, [0.5, 5.0, 3.0], [1.0, 6.0, 4.0]);
        let result = exact(&mut topo, op, cup, tool);
        assert_volume(&topo, result, expected);
    }
}

#[test]
fn bored_cup_booleans_survive_placement_and_operand_swap() {
    for placement in [Mat4::translation(37.0, -11.0, 8.0), Mat4::rotation_x(0.37)] {
        for contained in [false, true] {
            for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
                for swap in [false, true] {
                    if swap && op == BooleanOp::Cut {
                        continue;
                    }
                    let mut topo = Topology::new();
                    let cup = cup(&mut topo);
                    let tool = if contained {
                        placed_box(&mut topo, [0.5, 5.0, 3.0], [1.0, 6.0, 4.0])
                    } else {
                        placed_box(&mut topo, [-3.0, 5.0, 3.0], [4.0, 6.0, 4.0])
                    };
                    transform_solid(&mut topo, cup, &placement).unwrap();
                    transform_solid(&mut topo, tool, &placement).unwrap();
                    let (a, b) = if swap { (tool, cup) } else { (cup, tool) };
                    let result = exact(&mut topo, op, a, b);
                    let base = 1952.0 - 18.0 * std::f64::consts::PI;
                    let expected = match op {
                        BooleanOp::Fuse => base + if contained { 0.0 } else { 72.0 },
                        BooleanOp::Cut => base - 24.0,
                        BooleanOp::Intersect => 24.0,
                    };
                    assert_volume(&topo, result, expected);
                }
            }
        }
    }
}

#[test]
fn cup_bore_trigger_controls_retain_exact_wall_fuse() {
    for variant in 0..7 {
        let mut topo = Topology::new();
        let (cup, expected) = match variant {
            0 => (plain_cup(&mut topo, 2.0), 1952.0),
            1 => (
                bored_cup(&mut topo, [5.0, 5.0], 3.0, 4.0, 2.0),
                1952.0 - 18.0 * std::f64::consts::PI,
            ),
            2 => (
                bored_cup(&mut topo, [10.0, 10.0], 3.0, 2.0, 2.0),
                1952.0 - 9.0 * std::f64::consts::PI,
            ),
            3 => (
                bored_cup(&mut topo, [10.0, 10.0], 3.0, 7.0, 5.0),
                2720.0 - 45.0 * std::f64::consts::PI,
            ),
            4 => (
                bored_cup(&mut topo, [10.4, 10.0], 0.5, 4.0, 2.0),
                1952.0 - 0.5 * std::f64::consts::PI,
            ),
            5 => (
                bored_cup(&mut topo, [10.6, 10.0], 0.5, 4.0, 2.0),
                1952.0 - 0.5 * std::f64::consts::PI,
            ),
            6 => {
                let cup = plain_cup(&mut topo, 2.0);
                let bore = placed_box(&mut topo, [7.0, 7.0, -1.0], [6.0, 6.0, 4.0]);
                (exact(&mut topo, BooleanOp::Cut, cup, bore), 1952.0 - 72.0)
            }
            _ => unreachable!(),
        };
        let tool = placed_box(&mut topo, [-3.0, 5.0, 3.0], [4.0, 6.0, 4.0]);
        let result = exact(&mut topo, BooleanOp::Fuse, cup, tool);
        assert_volume(&topo, result, expected + 72.0);
    }
}

#[test]
fn cup_with_centered_boss_does_not_clip_wall_material_to_the_boss() {
    let mut topo = Topology::new();
    let cup = plain_cup(&mut topo, 2.0);
    let boss = make_cylinder(&mut topo, 3.0, 2.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(10.0, 10.0, 2.0)).unwrap();
    let cup = exact(&mut topo, BooleanOp::Fuse, cup, boss);
    assert!(classifier::try_build_analytic_classifier(&topo, cup).is_none());
    let tool = placed_box(&mut topo, [0.5, 5.0, 3.0], [1.0, 6.0, 4.0]);
    let result = exact(&mut topo, BooleanOp::Fuse, cup, tool);
    assert_volume(&topo, result, 1952.0 + 18.0 * std::f64::consts::PI);
}
