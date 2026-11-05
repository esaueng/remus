//! A single winding section must have identical edge partitions on both faces.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
    vec::Point3,
};
use remus_operations::{
    boolean::{BooleanOp, BooleanQuality, boolean_with_context},
    measure::{mass_properties_with_options, solid_volume},
    primitives::{make_cylinder, make_sphere},
    tessellate::{boundary_edge_count, non_manifold_edge_count, tessellate_solid},
    transform::transform_solid,
};
use remus_topology::{Topology, face::FaceSurface};

#[test]
fn off_axis_sphere_cylinder_union_keeps_shared_closed_section() {
    for swapped in [false, true] {
        check_fixture(1.0, 32, 0.0, false, swapped);
    }
}

#[test]
fn closed_section_survives_scale_placement_and_seam_phase() {
    for (scale, segments, phase) in [(0.1, 8, 0.0), (1.0, 4, 0.37)] {
        check_fixture(scale, segments, phase, true, false);
    }
}

#[test]
fn large_rotated_closed_section_keeps_sphere_orientation() {
    check_fixture(10.0, 32, 1.1, true, false);
}

fn check_fixture(scale: f64, segments: usize, phase: f64, placed: bool, swapped: bool) {
    let _ = env_logger::builder().is_test(true).try_init();
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 73.5 * scale, segments).unwrap();
    let cylinder = make_cylinder(&mut topo, 6.0 * scale, 64.100_000_000_000_01 * scale).unwrap();
    transform_solid(
        &mut topo,
        sphere,
        &Mat4::translation(25.5 * scale, -1.5 * scale, 0.0),
    )
    .unwrap();
    transform_solid(
        &mut topo,
        cylinder,
        &(Mat4::translation(0.0, 0.0, 58.0 * scale) * Mat4::rotation_z(phase)),
    )
    .unwrap();
    let placement = if placed {
        Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    for input in [sphere, cylinder] {
        transform_solid(&mut topo, input, &placement).unwrap();
        let validation = remus_operations::validate::validate_solid(&topo, input).unwrap();
        assert!(
            validation.is_valid(),
            "input scale={scale}, segments={segments}, phase={phase}: {:?}",
            validation.issues
        );
    }
    let original = remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap();
    let (a, b) = if swapped {
        (cylinder, sphere)
    } else {
        (sphere, cylinder)
    };
    let result = boolean_with_context(
        &mut topo,
        BooleanOp::Fuse,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|error| panic!("swapped={swapped}: {error:?}"));
    assert!(matches!(result.quality, BooleanQuality::Exact));
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap(),
        original
    );
    let solid = result.solid;
    let validation = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(
        validation.is_valid(),
        "scale={scale}, segments={segments}, phase={phase}: {:?}",
        validation.issues
    );
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    let mut counts = [0; 3];
    let mut uses = BTreeMap::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            FaceSurface::Sphere(_) => counts[0] += 1,
            FaceSurface::Cylinder(_) => counts[1] += 1,
            FaceSurface::Plane { .. } => counts[2] += 1,
            other => panic!("unexpected {} carrier", other.type_tag()),
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                uses.entry(edge.edge())
                    .or_insert_with(Vec::new)
                    .push(edge.is_forward() != face.is_reversed());
            }
        }
    }
    assert_eq!(counts, [2, 1, 1]);
    for (edge, directions) in uses {
        assert_eq!(directions.len(), 2, "incidence {edge:?}");
        assert_ne!(directions[0], directions[1], "orientation {edge:?}");
    }
    for (point, expected) in [
        (Point3::new(0.0, 0.0, 60.0), PointClassification::Inside),
        (Point3::new(0.0, 0.0, 100.0), PointClassification::Inside),
        (Point3::new(25.5, -1.5, 0.0), PointClassification::Inside),
        (Point3::new(7.0, 0.0, 100.0), PointClassification::Outside),
        (Point3::new(0.0, 0.0, 125.0), PointClassification::Outside),
    ] {
        assert_eq!(
            classify_point(
                &topo,
                solid,
                placement.mul_point(Point3::new(
                    point.x() * scale,
                    point.y() * scale,
                    point.z() * scale
                )),
                &ClassifyOptions::default()
            )
            .unwrap(),
            expected
        );
    }
    // Independent integration of sqrt(R²-(x-cx)²-(y-cy)²)-58 over the
    // radius-6 cylinder disk gives overlap 1219.0344613486054 mm³.
    // Compare the added material separately so the large sphere cannot
    // hide a missing boss in a relative whole-solid quantity assertion.
    let sphere_volume = 4.0 * std::f64::consts::PI * (73.5 * scale).powi(3) / 3.0;
    // Tighten the quadrature estimator in placed/scaled cells: subtracting
    // the large sphere magnifies the default whole-solid estimator error.
    let measured = mass_properties_with_options(
        &topo,
        solid,
        &remus_check::properties::PropertiesOptions {
            gauss_order: 8,
            adaptive_eps: 1e-10,
            max_depth: 16,
        },
    )
    .unwrap()
    .mass;
    let added = measured - sphere_volume;
    // The fixed linear modeling tolerance contributes an area-scaled
    // allowance; quantity integration also samples curved trims at a
    // fixed angular resolution, whose error scales with volume. Keep
    // the recorded scale=1 fixture's 0.05 mm³ bound (8.3 ppm of the boss).
    assert!(
        (added - 6_030.504_746_075_203 * scale.powi(3)).abs()
            < 0.05 * scale.powi(2).max(scale.powi(3)),
        "scale={scale}, segments={segments}, phase={phase}: added volume {added}"
    );
    if !placed {
        let displayed_added = solid_volume(&topo, solid, 0.01).unwrap() - sphere_volume;
        assert!(
            (displayed_added - 6_030.504_746_075_203).abs() < 0.05,
            "displayed added volume {displayed_added}"
        );
    }
    for deflection in [0.1, 0.01] {
        let mesh = tessellate_solid(&topo, solid, deflection * scale).unwrap();
        assert!(!mesh.indices.is_empty());
        assert_eq!(boundary_edge_count(&mesh), 0, "deflection={deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "deflection={deflection}");
    }
}
