//! Regression: an orientation-reversing map kept each wire's traversal.
//!
//! `mirror` (and `transform_solid` / `copy_and_transform_solid` with a
//! negative determinant) moved every vertex, edge and carrier and kept each
//! face normal outward, but left every wire traversed as before. A
//! reflection reverses winding, so every loop then wound against its stored
//! normal: a mirrored box drew six `FaceOrientationConsistency` findings, and
//! a mirrored box intersected with a cylinder came back with "6 shared edges
//! have inconsistent face orientations" (op_seq_81 finding F16).
//!
//! The oracles are independent of the transform: the checker's Newell
//! winding against the stored normal, the shell's opposite-use pairing, and
//! closed-form volumes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::validate::ValidateOptions;
use remus_check::validate::checks::CheckId;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::measure::solid_volume;
use remus_operations::mirror::mirror;
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Faces the checker reports as winding against their stored normal.
fn misoriented(topo: &Topology, solid: SolidId) -> usize {
    remus_check::validate::validate_solid(topo, solid, &ValidateOptions::default())
        .unwrap()
        .issues
        .iter()
        .filter(|issue| issue.check == CheckId::FaceOrientationConsistency)
        .count()
}

/// The image must draw no orientation finding its source did not (a wrapped
/// periodic wall's Newell normal is noise-sized, so the checker flags some
/// unmoved lateral walls too), validate clean, and keep its closed-form volume.
fn assert_sound(
    topo: &Topology,
    solid: SolidId,
    source_misoriented: usize,
    volume: f64,
    rel: f64,
    what: &str,
) {
    let found = misoriented(topo, solid);
    assert!(
        found <= source_misoriented,
        "{what}: {found} face(s) wind against their normal (source: {source_misoriented})"
    );
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.issues.is_empty(),
        "{what}: validation findings {:?}",
        report.issues
    );
    let got = solid_volume(topo, solid, 1e-3).unwrap();
    assert!(
        (got - volume).abs() <= rel * volume,
        "{what}: volume {got} vs closed form {volume}"
    );
}

#[test]
fn mirrored_box_winds_with_its_normals() {
    for (point, normal) in [
        (Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
        (Point3::new(5.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        (Point3::new(0.5, -1.0, 2.0), Vec3::new(1.0, 2.0, -3.0)),
    ] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
        let mirrored = mirror(&mut topo, solid, point, normal).unwrap();
        assert_sound(
            &topo,
            mirrored,
            0,
            6.0,
            1e-9,
            &format!("box mirrored in {normal:?}"),
        );
    }
}

#[test]
fn reflections_keep_every_primitive_consistent() {
    type Make = fn(&mut Topology) -> SolidId;
    let primitives: [(&str, Make, f64); 5] = [
        ("box", |t| make_box(t, 1.0, 2.0, 3.0).unwrap(), 6.0),
        (
            "cylinder",
            |t| make_cylinder(t, 1.0, 2.0).unwrap(),
            2.0 * PI,
        ),
        (
            "frustum",
            |t| make_cone(t, 2.0, 1.0, 3.0).unwrap(),
            PI * 3.0 * (4.0 + 2.0 + 1.0) / 3.0,
        ),
        (
            "sphere",
            |t| make_sphere(t, 1.5, 8).unwrap(),
            4.0 / 3.0 * PI * 1.5_f64.powi(3),
        ),
        (
            "torus",
            |t| make_torus(t, 3.0, 1.0, 16).unwrap(),
            2.0 * PI * PI * 3.0,
        ),
    ];
    for (name, make, volume) in primitives {
        for (sx, sy, sz) in [(-1.0, 1.0, 1.0), (1.0, 1.0, -1.0), (-1.0, -1.0, -1.0)] {
            let matrix = Mat4::scale(sx, sy, sz);
            let what = format!("{name} reflected by ({sx}, {sy}, {sz})");

            let mut topo = Topology::new();
            let solid = make(&mut topo);
            let source = misoriented(&topo, solid);
            transform_solid(&mut topo, solid, &matrix).unwrap();
            assert_sound(
                &topo,
                solid,
                source,
                volume,
                1e-4,
                &format!("{what} in place"),
            );

            let mut topo = Topology::new();
            let solid = make(&mut topo);
            let copy = copy_and_transform_solid(&mut topo, solid, &matrix).unwrap();
            assert_sound(
                &topo,
                copy,
                source,
                volume,
                1e-4,
                &format!("{what} by copy"),
            );
        }
    }
}

#[test]
fn anisotropic_reflection_keeps_box_and_cylinder_consistent() {
    let matrix = Mat4::scale(-2.0, 1.0, 0.5);
    let det = 1.0;
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    transform_solid(&mut topo, solid, &matrix).unwrap();
    assert_sound(
        &topo,
        solid,
        0,
        6.0 * det,
        1e-9,
        "anisotropic reflected box",
    );

    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let source = misoriented(&topo, solid);
    let copy = copy_and_transform_solid(&mut topo, solid, &matrix).unwrap();
    assert_sound(
        &topo,
        copy,
        source,
        2.0 * PI * det,
        1e-4,
        "anisotropic reflected cylinder (NURBS wall) by copy",
    );
}

/// The F16 pair, driven directly: a y-mirrored unit box intersected with a
/// unit cylinder through the exact-only boolean. The intersection is the
/// quarter of the cylinder in `x ≥ 0, y ≤ 0`, height 1: volume π/4.
#[test]
fn mirrored_box_intersect_cylinder_is_consistent() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1.0, 1.0).unwrap();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let mirrored = mirror(
        &mut topo,
        cube,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let result = boolean(&mut topo, BooleanOp::Intersect, mirrored, cylinder).unwrap();
    assert_sound(&topo, result, 0, PI / 4.0, 1e-6, "mirrored box ∩ cylinder");
}
