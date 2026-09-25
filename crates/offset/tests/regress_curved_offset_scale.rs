//! Intersection-joint offsets of curved primitives refuse above a size band
//! (bridge row B55).
//!
//! A cylinder or cone of radius 1e-3, 1 or 10 offsets to the closed-form
//! solid, but the same body at radius 100 or 1000 refuses with "offset face N
//! has no reconstructed wire loops", outward and inward, at the origin and
//! under a rigid placement. Boxes offset at every scale, so the refusal is
//! specific to the curved-face loop reconstruction, not to the offset
//! pipeline as a whole. Found 2026-09-25 while qualifying B18's offset
//! edge/vertex history; the geometry itself was out of that slice's scope.
//!
//! Every figure is a closed form: a cylinder of radius `r` and height `h`
//! offset by `d` with mitred joints is a cylinder of radius `r + d` and
//! height `h + 2d`.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_offset::{OffsetOptions, offset_solid};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_cone, make_cylinder};
use remus_topology::Topology;

fn assert_cylinder_offset(scale: f64, sign: f64) {
    let (r, h, d) = (scale, 2.0 * scale, sign * 0.2 * scale);
    let mut topo = Topology::new();
    let source = make_cylinder(&mut topo, r, h).unwrap();
    let result = offset_solid(&mut topo, source, d, OffsetOptions::default())
        .unwrap_or_else(|error| panic!("cylinder scale {scale} distance {d}: {error}"));
    let expected = std::f64::consts::PI * (r + d).powi(2) * (h + 2.0 * d);
    let volume = solid_volume(&topo, result, 1e-3 * scale).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "cylinder scale {scale} distance {d}: volume {volume}, closed form {expected}"
    );
}

fn assert_cone_offsets(scale: f64, sign: f64) {
    let mut topo = Topology::new();
    let source = make_cone(&mut topo, scale, 0.5 * scale, 2.0 * scale).unwrap();
    let d = sign * 0.2 * scale;
    offset_solid(&mut topo, source, d, OffsetOptions::default())
        .unwrap_or_else(|error| panic!("cone scale {scale} distance {d}: {error}"));
}

/// The qualified band: curved offsets succeed with closed-form volumes.
#[test]
fn curved_offsets_succeed_up_to_ten_units() {
    for scale in [1e-3, 1.0, 10.0] {
        for sign in [1.0, -1.0] {
            assert_cylinder_offset(scale, sign);
            assert_cone_offsets(scale, sign);
        }
    }
}

/// Acceptance target for B55: the same bodies at 100 and 1000 units.
#[test]
#[ignore = "open: B55 — curved intersection-joint offsets refuse at radius >= 100 \
            (no reconstructed wire loops)"]
fn curved_offsets_succeed_at_large_scale() {
    for scale in [100.0, 1000.0] {
        for sign in [1.0, -1.0] {
            assert_cylinder_offset(scale, sign);
            assert_cone_offsets(scale, sign);
        }
    }
}
