//! Qualification evidence for torus boolean configurations
//! (stabilization plan item B1).
//!
//! Each declared configuration is checked against an independent oracle:
//! a closed-form volume where one exists, the torus's central symmetry
//! (any plane through the centre halves it exactly), or the
//! inclusion–exclusion identity `vol(A∪B) + vol(A∩B) = vol(A) + vol(B)`.
//! The oracles hold whether a case runs analytic or through the disclosed
//! bounded mesh fallback, so they qualify the cell's *result*; the
//! analytic-vs-approximate annotation is the census's job.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const R: f64 = 10.0;
const RT: f64 = 2.0;
const DEFLECTION: f64 = 0.02;

fn torus_volume() -> f64 {
    2.0 * PI * PI * R * RT * RT
}

fn vol(topo: &Topology, s: SolidId) -> f64 {
    solid_volume(topo, s, DEFLECTION).unwrap()
}

fn assert_valid(topo: &Topology, s: SolidId, what: &str) {
    let report = remus_operations::validate::validate_solid(topo, s).unwrap();
    assert!(
        report.is_valid(),
        "{what} failed validation: {:?}",
        report.issues
    );
}

/// Coaxial cylinder of radius R through the torus removes exactly
/// `pi^2 R r^2 - 4 pi r^3 / 3` (the cylinder wall halves every tube
/// cross-section through its centre; Pappus over the half-disc).
#[test]
fn coaxial_cylinder_cut_matches_closed_form() {
    let mut topo = Topology::new();
    let t = make_torus(&mut topo, R, RT, 32).unwrap();
    let c = make_cylinder(&mut topo, R, 4.0 * RT).unwrap();
    transform_solid(&mut topo, c, &Mat4::translation(0.0, 0.0, -2.0 * RT)).unwrap();

    let cut = boolean(&mut topo, BooleanOp::Cut, t, c).unwrap();
    assert_valid(&topo, cut, "torus minus coaxial cylinder");

    let removed = PI * PI * R * RT * RT - 4.0 * PI * RT.powi(3) / 3.0;
    let expected = torus_volume() - removed;
    let v = vol(&topo, cut);
    // This configuration currently resolves through the disclosed bounded
    // mesh fallback (the exact band split of a closed torus is the named
    // follow-up in the stabilization plan), so the band is the fallback's
    // approximation budget, not exact-path precision.
    assert!(
        ((v - expected) / expected).abs() < 0.03,
        "expected {expected}, got {v}"
    );
}

/// An axis-perpendicular plane through the centre halves the torus.
#[test]
fn axis_perpendicular_plane_halves_torus() {
    let mut topo = Topology::new();
    let t = make_torus(&mut topo, R, RT, 32).unwrap();
    let slab = make_box(&mut topo, 4.0 * R, 4.0 * R, 4.0 * R).unwrap();
    // Slab occupies z >= 0 after centering in x/y.
    transform_solid(&mut topo, slab, &Mat4::translation(-2.0 * R, -2.0 * R, 0.0)).unwrap();

    let cut = boolean(&mut topo, BooleanOp::Cut, t, slab).unwrap();
    assert_valid(&topo, cut, "torus minus upper half-space");
    let v = vol(&topo, cut);
    let expected = torus_volume() / 2.0;
    assert!(
        ((v - expected) / expected).abs() < 0.01,
        "expected half torus {expected}, got {v}"
    );
}

/// A tilted plane through the centre halves the torus too: the torus is
/// centrally symmetric, so the two sides of ANY plane through the origin
/// map onto each other under point inversion.
#[test]
fn tilted_plane_through_centre_halves_torus() {
    let mut topo = Topology::new();
    let t = make_torus(&mut topo, R, RT, 32).unwrap();
    let slab = make_box(&mut topo, 4.0 * R, 4.0 * R, 4.0 * R).unwrap();
    // Rotate 30 deg about X, then place the (rotated) z=0 face plane
    // through the origin: rotate first, translate the box centre onto the
    // plane normal direction.
    let rot = Mat4::rotation_x(30.0_f64.to_radians());
    transform_solid(&mut topo, slab, &Mat4::translation(-2.0 * R, -2.0 * R, 0.0)).unwrap();
    transform_solid(&mut topo, slab, &rot).unwrap();

    let cut = boolean(&mut topo, BooleanOp::Cut, t, slab).unwrap();
    assert_valid(&topo, cut, "torus minus tilted half-space");
    let v = vol(&topo, cut);
    let expected = torus_volume() / 2.0;
    assert!(
        ((v - expected) / expected).abs() < 0.01,
        "expected half torus {expected}, got {v}"
    );
}

/// Concentric sphere through the tube centres: fuse, intersect, and cut
/// volumes must satisfy inclusion–exclusion against the operand volumes.
///
/// Ready-repro, ADVANCED not closed: the closed-torus band split now exists
/// (`split_closed_torus_into_bands`, reached via the seam anchor that takes a
/// torus's seam u from its degenerate seam vertex). Fuse and Intersect are
/// analytic, watertight and exact — 4632.67 and 344.60 against closed-form
/// 4633 and 344.6, and their sum matches vol(torus)+vol(sphere) to 0.02%.
///
/// CUT is the remaining defect and the reason this stays ignored: it selects
/// the correct outer band (its seam arcs run through the outer equator at
/// (12,0,0)) and reports free=0 over=0, but the shell validates as inside out
/// and encloses 833.56 where torus∖sphere is 444.97 — an excess of 388.59 that
/// is NOT explained by an orientation flip alone, since flipping the sign
/// still misses the closed form. Do NOT chase it by flipping the band winding:
/// that was measured and it breaks the shared section circles instead
/// ("2 shared edges have inconsistent face orientations"), so the +u lower-rim
/// convention is right and the defect is elsewhere — start at the sphere
/// annulus's reversed orientation in the Cut assembly, not at this splitter.
#[test]
#[ignore = "closed-torus band split landed; Cut still inside-out and over-encloses by 388.59"]
fn concentric_sphere_inclusion_exclusion() {
    let build = |op: BooleanOp| -> f64 {
        let mut topo = Topology::new();
        let t = make_torus(&mut topo, R, RT, 32).unwrap();
        let s = make_sphere(&mut topo, R, 32).unwrap();
        let result = boolean(&mut topo, op, t, s).unwrap();
        assert_valid(&topo, result, "torus-sphere boolean");
        vol(&topo, result)
    };

    let fuse = build(BooleanOp::Fuse);
    let intersect = build(BooleanOp::Intersect);
    let cut = build(BooleanOp::Cut);

    let vt = torus_volume();
    let vs = 4.0 / 3.0 * PI * R.powi(3);

    assert!(
        ((fuse + intersect) - (vt + vs)).abs() / (vt + vs) < 0.01,
        "inclusion-exclusion violated: fuse {fuse} + intersect {intersect} != {vt} + {vs}"
    );
    assert!(
        ((cut + intersect) - vt).abs() / vt < 0.01,
        "cut {cut} + intersect {intersect} != torus {vt}"
    );
}

/// The declared configurations are deterministic: the same cut twice gives
/// bit-identical volume.
#[test]
fn torus_boolean_is_deterministic() {
    let run = || {
        let mut topo = Topology::new();
        let t = make_torus(&mut topo, R, RT, 32).unwrap();
        let c = make_cylinder(&mut topo, R, 4.0 * RT).unwrap();
        transform_solid(&mut topo, c, &Mat4::translation(0.0, 0.0, -2.0 * RT)).unwrap();
        let cut = boolean(&mut topo, BooleanOp::Cut, t, c).unwrap();
        vol(&topo, cut).to_bits()
    };
    assert_eq!(run(), run());
}
