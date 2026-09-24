//! A shell's inner skin must carry the outer sphere's frame.
//!
//! Fuzz Smoke 2026-09-22 (`modifier_ops`, seed
//! `fuzz/corpus/modifier_ops/shell-box-fused-with-rotated-disjoint-sphere`):
//! a 1×1.5×1.5 box fused with a disjoint r=1.5 sphere rotated 45° about Y and
//! placed at (-2, 2.5, -0.5), then hollowed at thickness 0.6 with no open
//! faces, tessellated open (21 boundary edges at the harness deflection).
//! `shell_op` and the offset engine rebuilt the inner sphere from centre and
//! radius alone — the B41 class: a default-aligned inner sphere under a
//! rotated outer one, so the cavity's hemispheres and equator disagree.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use std::f64::consts::FRAC_PI_4;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::shell_op::shell;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn harness_deflection(topo: &Topology, solid: SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    ((aabb.max - aabb.min).length() * 4e-5).max(1e-7)
}

fn assert_watertight(topo: &Topology, solid: SolidId, what: &str) {
    let harness = harness_deflection(topo, solid) * 4.0;
    for d in [0.1, 0.01, harness] {
        let mesh = tessellate_solid(topo, solid, d).unwrap();
        let (b, n) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
        assert!(
            b == 0 && n == 0,
            "{what}: mesh at deflection {d} has {b} boundary and {n} non-manifold edges"
        );
    }
}

/// Both volume routes printed before either is believed (2026-09-02 lesson).
fn assert_volumes_agree(topo: &Topology, solid: SolidId, expected: f64, what: &str) {
    let mesh = solid_volume(topo, solid, harness_deflection(topo, solid)).unwrap();
    let exact = mass_properties(topo, solid).unwrap().mass;
    eprintln!("{what}: solid_volume={mesh:.9} mass_properties={exact:.9} expected={expected:.9}");
    for (route, v) in [("solid_volume", mesh), ("mass_properties", exact)] {
        assert!(
            ((v - expected) / expected).abs() < 2e-3,
            "{what}: {route} {v} vs expected {expected}"
        );
    }
}

fn rotated_sphere(topo: &mut Topology, r: f64) -> SolidId {
    let sphere = make_sphere(topo, r, 13).unwrap();
    let place = Mat4::translation(-2.0, 2.5, -0.5) * Mat4::rotation_y(FRAC_PI_4);
    transform_solid(topo, sphere, &place).unwrap();
    sphere
}

/// The narrow case: one rotated sphere hollowed at 0.6.
#[test]
fn hollowed_rotated_sphere_meshes_closed() {
    let mut topo = Topology::new();
    let sphere = rotated_sphere(&mut topo, 1.5);
    let hollow = shell(&mut topo, sphere, 0.6, &[]).unwrap();
    validate_solid(&topo, hollow).unwrap();
    assert_watertight(&topo, hollow, "hollow rotated sphere");
    let expected = 4.0 / 3.0 * std::f64::consts::PI * (1.5f64.powi(3) - 0.9f64.powi(3));
    assert_volumes_agree(&topo, hollow, expected, "hollow rotated sphere");
}

/// The fuzz body: box fused with the disjoint rotated sphere. At 0.6 the
/// box lump's inner prism fully collapses (1.0 − 2·0.6 < 0): the standalone
/// box refuses, and the multi-lump body must refuse the same way instead of
/// shipping an inverted cavity through the ordinary gate.
#[test]
fn hollowed_box_with_rotated_sphere_boss_refuses_collapsed_lump() {
    let mut topo = Topology::new();
    let stock = make_box(&mut topo, 1.0, 1.5, 1.5).unwrap();
    let boss = rotated_sphere(&mut topo, 1.5);
    let body = boolean(&mut topo, BooleanOp::Fuse, stock, boss).unwrap();
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let error = shell(&mut topo, body, 0.6, &[]).expect_err("collapsed lump must refuse");
    assert!(
        matches!(error, remus_operations::OperationsError::Unsupported { .. }),
        "typed refusal expected, got {error}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before,
        "refusal must roll back"
    );
}

/// The same two-lump body at a wall the box can carry hollows both lumps.
#[test]
fn hollowed_box_with_rotated_sphere_boss_meshes_closed_at_thin_wall() {
    let mut topo = Topology::new();
    let stock = make_box(&mut topo, 1.0, 1.5, 1.5).unwrap();
    let boss = rotated_sphere(&mut topo, 1.5);
    let body = boolean(&mut topo, BooleanOp::Fuse, stock, boss).unwrap();
    let hollow = shell(&mut topo, body, 0.1, &[]).unwrap();
    assert_watertight(&topo, hollow, "hollow box+sphere 0.1");
    let expected = (1.0 * 1.5 * 1.5 - 0.8 * 1.3 * 1.3)
        + 4.0 / 3.0 * std::f64::consts::PI * (1.5f64.powi(3) - 1.4f64.powi(3));
    assert_volumes_agree(&topo, hollow, expected, "hollow box+sphere 0.1");
}
