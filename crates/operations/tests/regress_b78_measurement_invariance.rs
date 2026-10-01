//! B78 measurement invariance: placed-hollow and bored-quadric volume must not
//! depend on the world origin.
//!
//! Starting revision `4a93e758` (origin/main):
//! - thin hollow (box fused with oblique cylinder, shell 0.1) reads
//!   `solid_volume` 4.807 unplaced vs 7.196 placed (49.7% drift) while its
//!   whole-solid mesh volume is bit-identical (4.936724885 at 0.01);
//! - base fuse (strict-valid, 9 faces, bored-quadric path) drifts 2.25e-6 at
//!   (10,−20,30), 2.25e-4 at (1000,−2000,3000), and 1.11e-4 relative at 1e-3
//!   with the harness offset;
//! - cross-drilled shafts (strict-valid, independent Simpson closed forms)
//!   drift 3.7e-5 (bore 1) and 3.5e-4 (bore 2) at (10,−20,30).
//!
//! Root: every volume fast path summed about the world origin (`integrate_face`,
//! origin-anchored analytic closed forms, and the direct path's origin-split
//! tetrahedra), so `|offset|`-sized terms cancelled to an `L³` answer. The fix
//! extends the B58 `integration_reference` contract to every path: bored-quadric
//! Gauss, shell signed volumes, torus bands, exact analytic closed forms, and
//! the direct per-face sum (plus holed-quadric tessellation instead of
//! rectangle overcount).
//!
//! Valid base and bored bodies remain invariant under rigid placement.
//! The historical hollow is now refused by the shell carrier gate; its
//! translation/rotation regression pins refusal and complete rollback.
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
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::shell_op::shell;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn base_body(topo: &mut Topology) -> remus_topology::solid::SolidId {
    let stock = make_box(topo, 1.0, 1.5, 1.5).unwrap();
    let tool = make_cylinder(topo, 1.5, 3.5).unwrap();
    let place = Mat4::translation(-1.5, 2.5, -0.5) * Mat4::rotation_y(FRAC_PI_4);
    transform_solid(topo, tool, &place).unwrap();
    boolean(topo, BooleanOp::Fuse, stock, tool).unwrap()
}

fn deflection(topo: &Topology, solid: remus_topology::solid::SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    ((aabb.max - aabb.min).length() * 4e-5).max(1e-7)
}

/// The formerly measurable off-carrier hollow must refuse under placement.
#[test]
fn off_carrier_hollow_refuses_with_rollback_under_placement() {
    for placement in [
        Mat4::identity(),
        Mat4::translation(10.0, -20.0, 30.0),
        Mat4::translation(10.0, -20.0, 30.0) * Mat4::rotation_z(std::f64::consts::FRAC_PI_2),
    ] {
        let mut topo = Topology::new();
        let body = base_body(&mut topo);
        transform_solid(&mut topo, body, &placement).unwrap();
        let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
        let slots = topo.allocated_slot_count();
        let journal = topo.journal().entries().len();
        let error = shell(&mut topo, body, 0.1, &[]).unwrap_err();
        assert!(matches!(
            error,
            remus_operations::OperationsError::Unsupported { .. }
        ));
        assert_eq!(
            before,
            remus_io::arena_io::serialize_solid(&topo, body).unwrap()
        );
        assert_eq!(slots, topo.allocated_slot_count());
        assert_eq!(journal, topo.journal().entries().len());
    }
}

/// Valid bored-quadric witness (strict-valid, no shell): the base fuse must be
/// invariant and agree with `mass_properties` (local reference).
#[test]
fn base_fuse_bored_quadric_is_invariant() {
    let mut topo0 = Topology::new();
    let b0 = base_body(&mut topo0);
    let v0 = solid_volume(&topo0, b0, deflection(&topo0, b0)).unwrap();
    let m0 = mass_properties(&topo0, b0).unwrap().mass;

    let mut topo1 = Topology::new();
    let b1 = base_body(&mut topo1);
    transform_solid(&mut topo1, b1, &Mat4::translation(10.0, -20.0, 30.0)).unwrap();
    let v1 = solid_volume(&topo1, b1, deflection(&topo1, b1)).unwrap();
    let m1 = mass_properties(&topo1, b1).unwrap().mass;

    eprintln!("base unplaced solid={v0:.9} mass={m0:.9}");
    eprintln!("base placed   solid={v1:.9} mass={m1:.9}");
    assert!(
        (v1 - v0).abs() / v0 < 1e-9,
        "base fuse drifted: {v0:.9} vs {v1:.9} (was 2.25e-6 apart)"
    );
    assert!(
        (m1 - m0).abs() / m0 < 1e-12,
        "mass must be invariant: {m0:.9} vs {m1:.9}"
    );
    assert!(
        (v0 - m0).abs() / v0 < 1e-9,
        "solid vs mass must agree: {v0:.9} vs {m0:.9}"
    );
}

/// Small-scale translation covariance (B56 class): 1e-3 base moved by the
/// harness offset must hold 1e-9 relative (was 1.11e-4).
#[test]
fn small_base_translation_holds_1e_9() {
    let mut topo = Topology::new();
    let b = base_body(&mut topo);
    transform_solid(&mut topo, b, &Mat4::scale(0.001, 0.001, 0.001)).unwrap();
    let v0 = solid_volume(&topo, b, deflection(&topo, b)).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(13.0, -7.0, 5.0)).unwrap();
    let v1 = solid_volume(&topo, b, deflection(&topo, b)).unwrap();
    eprintln!("1e-3 base in-place={v0:.9e} moved={v1:.9e}");
    assert!(
        (v1 - v0).abs() / v0 < 1e-9,
        "small-body drifted: {v0:.9e} vs {v1:.9e}"
    );
}

/// Rigid rotation invariance for the valid base (volume) and a valid bored box
/// (volume + centroid covariance).
#[test]
fn rotation_preserves_volume_and_moves_centroid() {
    use remus_math::vec::Point3;
    // Valid base volume must remain invariant under rigid placement.
    let mut topo0 = Topology::new();
    let h0 = base_body(&mut topo0);
    let v0 = solid_volume(&topo0, h0, deflection(&topo0, h0)).unwrap();
    let mut topo1 = Topology::new();
    let h1 = base_body(&mut topo1);
    transform_solid(
        &mut topo1,
        h1,
        &(Mat4::translation(10.0, -20.0, 30.0) * Mat4::rotation_z(std::f64::consts::FRAC_PI_2)),
    )
    .unwrap();
    let v1 = solid_volume(&topo1, h1, deflection(&topo1, h1)).unwrap();
    assert!(
        (v1 - v0).abs() / v0 < 1e-9,
        "rotated base drifted: {v0:.9} vs {v1:.9}"
    );

    // Valid bored box: centroid must translate exactly, volume bit-identical.
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let tool = make_cylinder(&mut topo, 2.0, 20.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(5.0, 5.0, -5.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    let c0 = mass_properties(&topo, body).unwrap().center;
    let v0 = solid_volume(&topo, body, deflection(&topo, body)).unwrap();
    let t = Mat4::translation(10.0, -20.0, 30.0);
    transform_solid(&mut topo, body, &t).unwrap();
    let c1 = mass_properties(&topo, body).unwrap().center;
    let v1 = solid_volume(&topo, body, deflection(&topo, body)).unwrap();
    assert!(
        (v1 - v0).abs() / v0 < 1e-12,
        "bored-box volume drifted: {v0:.9} vs {v1:.9}"
    );
    let expect = Point3::new(c0.x() + 10.0, c0.y() - 20.0, c0.z() + 30.0);
    assert!(
        (c1 - expect).length() < 1e-9,
        "centroid did not translate: {c1:?} vs {expect:?}"
    );
}
