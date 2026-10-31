//! A sphere whose cap pokes up into another body through that body's flat
//! base. The fuse keeps the sphere's band below the base: its outer loop is
//! the equator and its hole the circle where the base plane cuts the
//! sphere, both wrapping the sphere's u period, so the hole must wrap the
//! opposite way to the equator.

#![allow(clippy::unwrap_used, clippy::expect_used, missing_docs)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_cone, make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;

/// A sphere r 2 centred 1 below the base, so the base cuts it at r √3 and
/// leaves the cap of height 1 (volume 5π/3) inside the other body.
#[test]
fn a_sphere_band_below_a_flat_base_keeps_its_hole_opposite_its_equator() {
    let below = 4.0 / 3.0 * PI * 8.0 - 5.0 * PI / 3.0;
    for segments in [16, 32] {
        for cone in [false, true] {
            let mut topo = Topology::new();
            let (body, body_volume) = if cone {
                // The op-seq fixture's frustum r 7.5 -> 5 over 7.
                let v = PI * 7.0 / 3.0 * (7.5f64.powi(2) + 7.5 * 5.0 + 25.0);
                (make_cone(&mut topo, 7.5, 5.0, 7.0).unwrap(), v)
            } else {
                (make_cylinder(&mut topo, 5.0, 5.0).unwrap(), PI * 25.0 * 5.0)
            };
            let sphere = make_sphere(&mut topo, 2.0, segments).unwrap();
            transform_solid(&mut topo, sphere, &Mat4::translation(-3.0, 0.0, -1.0)).unwrap();
            let what = format!("segments {segments}, cone {cone}");
            let fused = boolean(&mut topo, BooleanOp::Fuse, sphere, body).unwrap();
            let report = validate_solid(&topo, fused).unwrap();
            assert!(report.is_valid(), "{what}: {:?}", report.issues);
            let expected = body_volume + below;
            for deflection in [1e-2, 1e-4] {
                let volume = solid_volume(&topo, fused, deflection).unwrap();
                assert!(
                    ((volume - expected) / expected).abs() < 1e-9,
                    "{what}: volume {volume} vs {expected} at deflection {deflection:e}"
                );
            }
        }
    }
}
