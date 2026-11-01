//! Exact volume of a box fused with a corner boss whose top rim is rounded.
//!
//! Fixture: `tests/data/box_corner_rounded_cylinder_boss.step`, a Remus STEP
//! export (no identity metadata): a 30 × 18 × 18 box at the origin fused with
//! a Z-axis cylinder r6 centred on the box corner (0, 0), height 28.5, whose
//! top rim carries an r1.8 round — a quarter torus (major 4.2, minor 1.8,
//! centre z 26.7) from the wall's top circle up to the r4.2 cap at z 28.5.
//!
//! The torus is stored as a latitude band: an outer wire (the r6 ring at
//! z 26.7) and one inner wire (the r4.2 ring at z 28.5). That inner wire sends
//! the body down the per-face route, where the torus used to take its own
//! mesh — and the standalone torus tessellator reads the outer ring alone and
//! skins the whole tube. The face measured as the complete torus (268.6 where
//! its band encloses 385.6), and the body read 12292.87, 117 mm³ light.
//!
//! The expectation is the closed form, never another of the kernel's own
//! measurements:
//!
//! * box `30·18·18 = 9720`;
//! * the three quarters of the cylinder outside the box below z 18:
//!   `¾·π·6²·18`;
//! * the whole cylinder from z 18 to the torus centre at z 26.7: `π·6²·8.7`;
//! * the rounded cap from z 26.7 to 28.5, a solid of revolution with radius
//!   `R + √(a² − t²)` at height `t` (`R = 4.2`, `a = 1.8`):
//!   `π∫₀ᵃ (R + √(a² − t²))² dt = π(R²a + πRa²/2 + 2a³/3)`.

#![allow(
    clippy::disallowed_methods,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic
)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::solid_volume;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// The body's volume in closed form (see the module docs).
fn closed_form_volume() -> f64 {
    let (r, big_r, a) = (6.0_f64, 4.2_f64, 1.8_f64);
    let box_volume = 30.0 * 18.0 * 18.0;
    let wall = PI * r * r * (0.75 * 18.0 + 8.7);
    let rounded_cap = PI * (big_r * big_r * a + PI * big_r * a * a / 2.0 + 2.0 * a.powi(3) / 3.0);
    box_volume + wall + rounded_cap
}

fn load_fixture() -> (Topology, SolidId) {
    let step = include_str!("data/box_corner_rounded_cylinder_boss.step");
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(step, &mut topo).unwrap();
    assert_eq!(solids.len(), 1, "fixture holds one solid");
    (topo, solids[0])
}

fn assert_closed_form(topo: &Topology, solid: SolidId, what: &str) {
    let expected = closed_form_volume();
    // Both requests are finer than the clamp; the route is exact, so the
    // deflection must not show in the answer at all.
    for deflection in [0.005, 0.0005] {
        let volume = solid_volume(topo, solid, deflection).unwrap();
        assert!(
            (volume - expected).abs() <= 1e-9 * expected,
            "{what} at deflection {deflection}: {volume} vs closed form {expected} \
             ({:+.3e} relative)",
            (volume - expected) / expected
        );
    }
}

#[test]
fn fixture_rim_is_a_two_ring_torus_latitude_band() {
    let (topo, solid) = load_fixture();
    let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    let tori: Vec<_> = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .collect();
    assert_eq!(tori.len(), 1, "one rolled rim");
    let face = topo.face(*tori[0]).unwrap();
    assert_eq!(
        face.inner_wires().len(),
        1,
        "the rim is stored outer + inner ring"
    );
    let mut rings = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let edges = topo.wire(wire).unwrap().edges();
        assert_eq!(edges.len(), 1, "each ring is one closed circle");
        let EdgeCurve::Circle(circle) = topo.edge(edges[0].edge()).unwrap().curve() else {
            panic!("each ring is a circle");
        };
        rings.push((circle.radius(), circle.center().z()));
    }
    for (got, want) in rings.iter().zip([(6.0, 26.7), (4.2, 28.5)]) {
        assert!(
            (got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9,
            "ring {got:?} vs {want:?}"
        );
    }
}

#[test]
fn rounded_boss_volume_matches_closed_form() {
    let (topo, solid) = load_fixture();
    assert_closed_form(&topo, solid, "as imported");
}

#[test]
fn rounded_boss_volume_survives_translation_and_mirroring() {
    // The band integral is taken about the body's reference point and picks
    // its band from the rings' orientation; a far translation and mirrors
    // (which reverse every boundary's traversal) must read the same body.
    let (mut topo, solid) = load_fixture();
    let moved = remus_operations::copy::copy_and_transform_solid(
        &mut topo,
        solid,
        &Mat4::translation(137.0, -59.0, 41.0),
    )
    .unwrap();
    assert_closed_form(&topo, moved, "translated");
    for (point, normal) in [
        (Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        (Point3::new(3.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        (Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.3)),
    ] {
        let mirrored = remus_operations::mirror::mirror(&mut topo, solid, point, normal).unwrap();
        assert_closed_form(&topo, mirrored, &format!("mirrored across {normal:?}"));
    }
}
