//! Regression: a solid's bounding box must bound the faces, not the surfaces
//! those faces are cut from.
//!
//! `solid_bounding_box` expanded spherical and toroidal faces to their whole
//! analytic surface. That is right for a primitive sphere or a whole torus —
//! their boundary is a seam that bounds nothing, so the surface *is* the face —
//! but wrong for the trimmed blends an imported part is full of. A 1.2 MB CATIA
//! import (72 toroidal faces) reported a box roughly twice the part's true
//! extent in two axes, both hitting the same value: the untrimmed radius of one
//! big ring. Anything that frames a camera or culls by bounds then works from a
//! box the model rattles around inside — OpenZCAD's Fit View shrank the part
//! into a corner of the viewport.
//!
//! The reproduction is synthetic on purpose (the reporting part is proprietary):
//! a 14° revolve of a small circle 200 mm off the axis. The band it sweeps is a
//! sliver of a torus whose full surface is 410 mm across, so an untrimmed
//! expansion overshoots by more than an order of magnitude in area — the same
//! failure as the customer part, in a shape small enough to state exactly.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::measure;
use brepkit_operations::primitives;
use brepkit_operations::revolve::revolve;
use brepkit_topology::Topology;
use brepkit_topology::builder::{make_circle_edge, make_face_from_wire};
use brepkit_topology::solid::SolidId;
use brepkit_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;
/// The box is analytically exact for these shapes; this only absorbs the
/// arithmetic.
const EPS: f64 = 1e-6;

/// Revolve a circle of radius `minor`, centred `major` from the axis, through
/// `sweep` about Z. A partial sweep yields one trimmed toroidal band closed by
/// two planar disc caps; a full sweep yields one doubly-periodic torus face.
fn revolved_ring(topo: &mut Topology, major: f64, minor: f64, sweep: f64) -> SolidId {
    let profile = make_circle_edge(
        topo,
        Point3::new(major, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        minor,
        TOL,
    )
    .unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(profile, true)], true).unwrap());
    let face = make_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        sweep,
    )
    .unwrap()
}

#[track_caller]
fn assert_box(topo: &Topology, solid: SolidId, expect_min: [f64; 3], expect_max: [f64; 3]) {
    let bb = measure::solid_bounding_box(topo, solid).unwrap();
    let got_min = [bb.min.x(), bb.min.y(), bb.min.z()];
    let got_max = [bb.max.x(), bb.max.y(), bb.max.z()];
    for axis in 0..3 {
        assert!(
            (got_min[axis] - expect_min[axis]).abs() < EPS
                && (got_max[axis] - expect_max[axis]).abs() < EPS,
            "axis {axis}: got [{}, {}], want [{}, {}]",
            got_min[axis],
            got_max[axis],
            expect_min[axis],
            expect_max[axis]
        );
    }
}

/// The band swept by the profile circle, computed by hand.
///
/// Points are `(R + r·cos v)·(cos u, sin u, 0) + (0, 0, r·sin v)`, so the
/// distance from the axis runs over `[R − r, R + r]` and `z` over `[−r, r]`.
/// Over `u ∈ [0, sweep]` with `sweep < π/2`, `x` is largest at `u = 0` on the
/// outer wall and smallest at `u = sweep` on the inner one, while `y` runs from
/// 0 up to the outer wall at `u = sweep`.
fn swept_band_extent(major: f64, minor: f64, sweep: f64) -> ([f64; 3], [f64; 3]) {
    let (r_in, r_out) = (major - minor, major + minor);
    (
        [r_in * sweep.cos(), 0.0, -minor],
        [r_out, r_out * sweep.sin(), minor],
    )
}

#[test]
fn a_trimmed_torus_band_is_bounded_by_the_band_not_the_ring() {
    let (major, minor, sweep) = (200.0, 5.0, 0.25);
    let mut topo = Topology::new();
    let solid = revolved_ring(&mut topo, major, minor, sweep);

    let (want_min, want_max) = swept_band_extent(major, minor, sweep);
    assert_box(&topo, solid, want_min, want_max);

    // Guard the specific regression: the untrimmed ring reaches ±(R + r) in
    // both X and Y, which is what the old expansion reported. The band's own
    // Y extent is under a tenth of that and it never crosses y = 0 at all.
    let bb = measure::solid_bounding_box(&topo, solid).unwrap();
    assert!(
        bb.min.y() > -EPS,
        "band lies at y >= 0; got y_min {} (untrimmed ring would give {})",
        bb.min.y(),
        -(major + minor)
    );
    assert!(
        bb.max.y() < (major + minor) * 0.3,
        "band spans {sweep} rad of the ring; got y_max {}",
        bb.max.y()
    );
}

#[test]
fn a_torus_face_that_wraps_its_surface_still_gets_the_whole_ring() {
    // The complement of the case above: with no trim to find, the expansion
    // must still reach the full analytic extent. A whole torus's boundary is
    // a degenerate seam, so nothing about the face's own edges reveals how far
    // the surface runs.
    let (major, minor) = (200.0, 5.0);
    let rr = major + minor;

    let mut topo = Topology::new();
    let revolved = revolved_ring(&mut topo, major, minor, TAU);
    assert_box(&topo, revolved, [-rr, -rr, -minor], [rr, rr, minor]);

    let mut topo = Topology::new();
    let primitive = primitives::make_torus(&mut topo, 30.0, 7.0, 32).unwrap();
    assert_box(&topo, primitive, [-37.0, -37.0, -7.0], [37.0, 37.0, 7.0]);
}

#[test]
fn a_whole_sphere_still_gets_its_whole_radius() {
    // A sphere's seam bounds nothing either, and a polar cap's only boundary
    // is one latitude circle — the pole beyond it has to come from the
    // surface. Trimming latitude is only safe once longitude is bounded.
    let mut topo = Topology::new();
    let solid = primitives::make_sphere(&mut topo, 11.0, 32).unwrap();
    assert_box(&topo, solid, [-11.0, -11.0, -11.0], [11.0, 11.0, 11.0]);
}

#[test]
fn a_quarter_ring_is_bounded_on_the_axis_it_does_not_reach() {
    // A quarter turn is wide enough to hold the outer wall's X maximum at
    // u = 0 and its Y maximum at u = π/2, but must still report nothing below
    // zero on either — the half of the ring it never sweeps.
    let (major, minor) = (120.0, 8.0);
    let sweep = std::f64::consts::FRAC_PI_2;
    let mut topo = Topology::new();
    let solid = revolved_ring(&mut topo, major, minor, sweep);

    let bb = measure::solid_bounding_box(&topo, solid).unwrap();
    assert!(
        bb.min.x() > -EPS && bb.min.y() > -EPS,
        "quarter ring stays in the +X+Y quadrant; got min ({}, {})",
        bb.min.x(),
        bb.min.y()
    );
    assert!(
        (bb.max.x() - (major + minor)).abs() < EPS && (bb.max.y() - (major + minor)).abs() < EPS,
        "quarter ring reaches the outer wall on both axes; got max ({}, {})",
        bb.max.x(),
        bb.max.y()
    );
}
