//! A small angular residual can move long or large carriers by real distances.

#![allow(clippy::unwrap_used, clippy::panic, clippy::float_cmp)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use remus_algo::FaceClass;
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::surfaces::ToroidalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{
    BooleanOp, BooleanQuality, boolean, boolean_regions, boolean_with_context,
    boolean_with_entity_evolution, boolean_with_evolution,
};
use remus_operations::journal_ops::boolean_journaled_with_operation;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_cone, make_cylinder, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::face::FaceSurface;
use remus_topology::journal::EntityKey;
use remus_topology::{SolidId, Topology};

fn curved_face(topo: &Topology, solid: SolidId) -> &FaceSurface {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find_map(|face| {
            let surface = topo.face(face).unwrap().surface();
            matches!(
                surface,
                FaceSurface::Cylinder(_) | FaceSurface::Cone(_) | FaceSurface::Torus(_)
            )
            .then_some(surface)
        })
        .unwrap()
}

fn material_class(topo: &Topology, solid: SolidId, point: Point3) -> Option<FaceClass> {
    remus_algo::classifier::try_build_analytic_classifier(topo, solid)
        .unwrap()
        .classify(point, Tolerance::default())
}

fn assert_distinct_material(topo: &Topology, a: SolidId, b: SolidId, witness: Point3) {
    assert_eq!(material_class(topo, a, witness), Some(FaceClass::Outside));
    assert_eq!(material_class(topo, b, witness), Some(FaceClass::Inside));
}

fn assert_valid_inputs(topo: &Topology, a: SolidId, b: SolidId) {
    for solid in [a, b] {
        let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
        assert!(report.is_valid(), "{solid:?}: {report:?}");
    }
}

fn torus_tube_distance(surface: &ToroidalSurface, point: Point3) -> f64 {
    let offset = point - surface.center();
    let axial = offset.dot(surface.z_axis());
    let radial = (offset - surface.z_axis() * axial).length();
    (radial - surface.major_radius()).hypot(axial)
}

fn assert_refusals_preserve_inputs(mut topo: Topology, a: SolidId, b: SolidId) {
    assert_valid_inputs(&topo, a, b);
    // Keep real history present, so a failed entry point must preserve both
    // recorded entries and their operation/ordinal namespace.
    let face = remus_topology::explorer::solid_faces(&topo, a).unwrap()[0];
    let pending = topo.journal_begin("carrier alignment fixture");
    topo.journal_record_barrier(pending, vec![EntityKey::face(face.index())]);
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let before = format!("{topo:?}");
    let journal_before = topo.journal().snapshot();
    let slots_before = topo.allocated_slot_count();

    for op in [BooleanOp::Fuse, BooleanOp::Intersect, BooleanOp::Cut] {
        for entry_point in [
            "plain",
            "context",
            "evolution",
            "entity evolution",
            "regions",
            "journaled",
        ] {
            let result = match entry_point {
                "plain" => boolean(&mut topo, op, a, b).map(|_| ()),
                "context" => boolean_with_context(&mut topo, op, a, b, &context).map(|_| ()),
                "evolution" => boolean_with_evolution(&mut topo, op, a, b).map(|_| ()),
                "entity evolution" => {
                    boolean_with_entity_evolution(&mut topo, op, a, b).map(|_| ())
                }
                "regions" => boolean_regions(&mut topo, op, a, b).map(|_| ()),
                "journaled" => boolean_journaled_with_operation(&mut topo, op, a, b).map(|_| ()),
                _ => unreachable!(),
            };
            assert!(
                matches!(result, Err(OperationsError::ExactOnlyUnattainable)),
                "{entry_point} {op:?}: {result:?}"
            );
            assert_eq!(format!("{topo:?}"), before, "{entry_point} {op:?}");
            assert_eq!(topo.journal().snapshot(), journal_before);
            assert_eq!(topo.allocated_slot_count(), slots_before);
        }
    }
}

#[test]
fn large_torus_tilt_cannot_publish_a_radius_only_boolean() {
    let mut topo = Topology::new();
    let a = make_torus(&mut topo, 1e12, 1.0, 16).unwrap();
    let b = make_torus(&mut topo, 1e12, 1.0, 16).unwrap();
    transform_solid(&mut topo, a, &Mat4::rotation_y(FRAC_PI_4)).unwrap();
    transform_solid(&mut topo, b, &Mat4::rotation_y(FRAC_PI_4 + 1e-14)).unwrap();
    let FaceSurface::Torus(surface) = curved_face(&topo, b) else {
        panic!("expected torus");
    };
    let witness =
        surface.evaluate(0.0, 3.0 * FRAC_PI_2) - surface.normal(0.0, 3.0 * FRAC_PI_2) * 0.001;
    assert_distinct_material(&topo, a, b, witness);
    let FaceSurface::Torus(other) = curved_face(&topo, a) else {
        panic!("expected torus");
    };
    // Independently evaluate distance to each complete major circle. The
    // margins exceed coordinate rounding at this scale and linear tolerance.
    assert!(torus_tube_distance(other, witness) > other.minor_radius() + 0.005);
    assert!(torus_tube_distance(surface, witness) < surface.minor_radius() - 0.0005);
    let residual = (surface.z_axis() - other.z_axis()).length();
    assert!(residual <= 128.0 * f64::EPSILON);
    assert!(surface.major_radius() * residual > 0.005);
    assert_refusals_preserve_inputs(topo, a, b);
}

#[test]
fn long_cylinder_tilt_cannot_publish_a_coaxial_boolean() {
    let mut topo = Topology::new();
    let a = make_cylinder(&mut topo, 1.0, 1e6).unwrap();
    let b = make_cylinder(&mut topo, 1.0, 1e6).unwrap();
    transform_solid(&mut topo, b, &Mat4::rotation_y(1e-8)).unwrap();
    let FaceSurface::Cylinder(surface) = curved_face(&topo, b) else {
        panic!("expected cylinder");
    };
    let witness =
        surface.evaluate(3.0 * FRAC_PI_2, 9e5) - surface.normal(3.0 * FRAC_PI_2, 9e5) * 0.001;
    // The point exceeds the unrotated radius by over 0.005 length units,
    // rather than relying on a tolerance-boundary classifier result.
    assert!(witness.x().hypot(witness.y()) > 1.005);
    assert_distinct_material(&topo, a, b, witness);
    assert_refusals_preserve_inputs(topo, a, b);
}

#[test]
fn long_shared_apex_frustum_tilt_cannot_publish_a_coaxial_boolean() {
    let mut topo = Topology::new();
    let a = make_cone(&mut topo, 1.0, 0.5, 1e6).unwrap();
    let b = make_cone(&mut topo, 1.0, 0.5, 1e6).unwrap();
    let tilt_about_apex = Mat4::translation(0.0, 0.0, 2e6)
        * Mat4::rotation_y(1e-8)
        * Mat4::translation(0.0, 0.0, -2e6);
    transform_solid(&mut topo, b, &tilt_about_apex).unwrap();
    let FaceSurface::Cone(surface) = curved_face(&topo, b) else {
        panic!("expected cone");
    };
    let v = 1.9e6 / surface.half_angle().sin();
    let witness = surface.evaluate(FRAC_PI_2, v) - surface.normal(FRAC_PI_2, v) * 0.001;
    let unrotated_radius = (2e6 - witness.z()) * 0.5e-6;
    assert!(witness.x().hypot(witness.y()) > unrotated_radius + 0.01);
    assert_distinct_material(&topo, a, b, witness);
    assert_refusals_preserve_inputs(topo, a, b);
}

fn assert_result_axis(surface: &FaceSurface, expected: Vec3) {
    let axis = match surface {
        FaceSurface::Cylinder(surface) => surface.axis(),
        FaceSurface::Cone(surface) => surface.axis(),
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => panic!("expected rotation surface"),
    };
    assert!((axis - expected).length().min((axis + expected).length()) < 1e-14);
    assert!(axis.x().abs() > 0.5e-8, "the result must retain its tilt");
}

#[test]
fn same_axis_tilted_cylinder_stack_preserves_the_real_axis() {
    let mut topo = Topology::new();
    let a = make_cylinder(&mut topo, 1.0, 1e6).unwrap();
    let b = make_cylinder(&mut topo, 1.0, 1e6).unwrap();
    let angle = 1e-8_f64;
    let rotation = Mat4::rotation_y(angle);
    transform_solid(&mut topo, b, &Mat4::translation(0.0, 0.0, 0.5e6)).unwrap();
    for solid in [a, b] {
        transform_solid(&mut topo, solid, &rotation).unwrap();
    }
    assert_valid_inputs(&topo, a, b);
    let witness = rotation.mul_point(Point3::new(0.999, 0.0, 1.4e6));
    assert_eq!(material_class(&topo, b, witness), Some(FaceClass::Inside));
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &context).unwrap();
    assert_eq!(outcome.quality, BooleanQuality::Exact);
    assert_result_axis(
        curved_face(&topo, outcome.solid),
        Vec3::new(angle.sin(), 0.0, angle.cos()),
    );
    assert_eq!(
        material_class(&topo, outcome.solid, witness),
        Some(FaceClass::Inside)
    );
    let expected_volume = 1.5e6 * PI;
    assert!(
        (solid_volume(&topo, outcome.solid, 0.001).unwrap() - expected_volume).abs()
            < expected_volume * 1e-8
    );
}

#[test]
fn same_axis_tilted_frustum_stack_preserves_the_real_axis() {
    let mut topo = Topology::new();
    let a = make_cone(&mut topo, 1.0, 0.5, 1e6).unwrap();
    let b = make_cone(&mut topo, 1.5, 1.0, 1e6).unwrap();
    let angle = 1e-8_f64;
    let rotation = Mat4::rotation_y(angle);
    transform_solid(&mut topo, b, &Mat4::translation(0.0, 0.0, -1e6)).unwrap();
    for solid in [a, b] {
        transform_solid(&mut topo, solid, &rotation).unwrap();
    }
    assert_valid_inputs(&topo, a, b);
    let witness = rotation.mul_point(Point3::new(-1.449, 0.0, -0.9e6));
    assert_eq!(material_class(&topo, b, witness), Some(FaceClass::Inside));
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &context).unwrap();
    assert_eq!(outcome.quality, BooleanQuality::Exact);
    assert_result_axis(
        curved_face(&topo, outcome.solid),
        Vec3::new(angle.sin(), 0.0, angle.cos()),
    );
    assert_eq!(
        material_class(&topo, outcome.solid, witness),
        Some(FaceClass::Inside)
    );
    let expected_volume = 2e6 * PI * (1.5_f64.powi(2) + 1.5 * 0.5 + 0.5_f64.powi(2)) / 3.0;
    assert!(
        (solid_volume(&topo, outcome.solid, 0.001).unwrap() - expected_volume).abs()
            < expected_volume * 1e-8
    );
}
