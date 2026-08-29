//! A cross-drilled shaft must measure the material that is left.
//!
//! `solid_volume` and `mass_properties` both reach
//! `remus_check::properties::face_integrator::integrate_face` for an
//! all-analytic body. On the quadric path that integrator had two defects, and
//! a bore drilled through the side of a shaft meets both at once:
//!
//! 1. A face's inner wires were never subtracted from a curved face, so the
//!    two rims the bore opens in the shaft wall stayed material.
//! 2. A quadric face bounded by one closed edge — which is what each lobe of
//!    the bore wall is — had no UV bounds to take from its single boundary
//!    vertex, fell back to the cylinder's analytic domain, and that domain is
//!    unbounded in `v`. Every abscissa came out non-finite, all of them were
//!    rejected, and the bore walls contributed exactly zero.
//!
//! With both, an r = 3 bore through an r = 3, h = 30 shaft measured
//! 848.230016 mm³ — π·r²·h to the last digit, the shaft as though it had never
//! been drilled — against a true 704.230016. The 144 mm³ splits exactly in
//! half: 72 mm³ of rim kept as material by (1), and 72 mm³ of bore wall lost
//! by (2). `solid_volume` did not even reach the integrator, because
//! `analytic_faces_solid_volume` deferred any holed cylinder wall to
//! tessellation for precisely this reason.
//!
//! An equal-radius bore is used because that is the cross-drill the boolean
//! keeps analytic: the two cylinders then meet in a pair of PLANE ellipses.
//! Unequal radii meet in a quartic the boolean has no analytic edge for, and
//! the result comes back fully faceted, which this path never sees. The equal
//! case has a closed form — the removed material is the Steinmetz solid,
//! `16r³/3` — so every expected value here is composed from `r` and `h`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const R: f64 = 3.0;
const H: f64 = 30.0;

/// The bore's rims and lobes are trimmed by chord polylines of their true
/// ellipses, so the body is exact only to that chording — 4.7e-5 relative
/// here. It is a bound on the outline, not on the quadrature: the defects it
/// replaces were 2.0e-1.
const REL: f64 = 1e-4;

/// A shaft of radius `R` and height `H` with an equal-radius bore drilled
/// clean through its side at mid-height, perpendicular to its axis.
fn cross_drilled_shaft() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, R, H).unwrap();
    // Long enough to exit both sides, centred on the shaft's axis at H/2.
    let len = H + 4.0 * R;
    let bore = make_cylinder(&mut topo, R, len).unwrap();
    transform_solid(&mut topo, bore, &Mat4::rotation_y(FRAC_PI_2)).unwrap();
    transform_solid(
        &mut topo,
        bore,
        &Mat4::translation(-len / 2.0, 0.0, H / 2.0),
    )
    .unwrap();
    let res = boolean(&mut topo, BooleanOp::Cut, shaft, bore).unwrap();
    (topo, res)
}

/// Material left after the bore: the shaft minus the Steinmetz solid the two
/// equal perpendicular cylinders share.
fn closed_form_volume() -> f64 {
    PI * R * R * H - 16.0 / 3.0 * R * R * R
}

/// The whole point: the bore is really gone.
#[test]
fn cross_drilled_shaft_measures_its_closed_form() {
    let (topo, solid) = cross_drilled_shaft();
    let expected = closed_form_volume();
    let undrilled = PI * R * R * H;

    for (what, v) in [
        (
            "mass_properties",
            mass_properties(&topo, solid).unwrap().mass,
        ),
        ("solid_volume", solid_volume(&topo, solid, 0.01).unwrap()),
    ] {
        assert!(
            (v - expected).abs() <= REL * expected,
            "{what}: expected the closed form {expected:.6}, got {v:.6} \
             ({:+.6}, {:+.4} %)",
            v - expected,
            100.0 * (v - expected) / expected
        );
        // The defect was the undrilled shaft to the last digit; a tolerance
        // alone would not say that the bore is gone rather than merely small.
        assert!(
            (undrilled - v) >= 0.9 * (undrilled - expected),
            "{what}: {v:.6} still keeps most of the bore — the undrilled shaft \
             is {undrilled:.6}"
        );
    }
}

/// Pin the shape the exact Steinmetz seam (2026-08) produces, so a silent
/// representation change is caught. The wall no longer arrives holed — each
/// wall splits into two seam-free bands bounded by the exact seam ellipse
/// arcs, and the bore contributes lens patches bounded by the same arcs.
/// (The pre-exact-seam topology this fixture used to pin — one holed wall +
/// two single-closed-edge lobes — no longer arises from this construction;
/// the integrator's inner-wire and closed-edge paths keep their direct unit
/// coverage in `remus-check`.)
#[test]
fn the_shaft_splits_into_wall_bands_and_bore_lens_patches() {
    let (topo, solid) = cross_drilled_shaft();
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();

    let mut cylinder_faces = 0;
    let mut holed_walls = 0;
    let mut planes = 0;
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            FaceSurface::Cylinder(_) => {
                cylinder_faces += 1;
                if !face.inner_wires().is_empty() {
                    holed_walls += 1;
                }
            }
            FaceSurface::Plane { .. } => planes += 1,
            other => panic!("unexpected non-analytic face {other:?}"),
        }
    }
    assert_eq!(planes, 2, "the two shaft caps");
    assert_eq!(holed_walls, 0, "the exact seam leaves no holed wall");
    assert_eq!(
        cylinder_faces, 5,
        "two shaft wall bands plus the bore's three lens patches"
    );
}

/// A measurement is a property of the body, not of the quadrature order or the
/// preview quality the caller asked for. This is #46's guard, on a body whose
/// faces are trimmed quadrics rather than planes.
#[test]
fn cross_drilled_shaft_measurement_is_not_a_setting() {
    let (topo, solid) = cross_drilled_shaft();
    let reference = mass_properties(&topo, solid).unwrap().mass;

    // Looser than the all-planar bodies' 1e-9: the u-quadrature is split at
    // every vertex of a 128-segment outline, so which abscissae land where
    // shifts with the order. It is four orders of magnitude below the
    // chording, and three below what the defects cost.
    let invariance = 1e-6;

    for order in [4_usize, 5, 6, 8, 10, 12, 16] {
        let options = remus_check::properties::PropertiesOptions {
            gauss_order: order,
            ..Default::default()
        };
        let v = remus_check::properties::solid_volume(&topo, solid, &options).unwrap();
        assert!(
            (v - reference).abs() <= invariance * reference,
            "the shaft's volume depends on Gauss order — {reference} vs {v} at \
             order {order}"
        );
    }

    for deflection in [1.0, 0.5, 0.1, 0.01, 1e-4, 1e-6] {
        let v = solid_volume(&topo, solid, deflection).unwrap();
        assert!(
            (v - reference).abs() <= invariance * reference,
            "mass_properties disagrees with solid_volume at deflection \
             {deflection} — {reference} vs {v}"
        );
    }
}

/// The same lens, kept rather than removed. The two walls of a perpendicular
/// cylinder fuse each carry the lens rims as holes, so defect 1 alone put the
/// body 144 mm³ heavy: 1130.973355 against a true 986.973355.
///
/// `solid_volume` recognises this body and answers from a closed form, so the
/// integrator is exercised through `mass_properties`, which has no such path.
#[test]
fn fused_cylinder_walls_subtract_their_lens_rims() {
    const L: f64 = 20.0;
    let mut topo = Topology::new();
    let a = make_cylinder(&mut topo, R, L).unwrap();
    transform_solid(&mut topo, a, &Mat4::translation(0.0, 0.0, -L / 2.0)).unwrap();
    let b = make_cylinder(&mut topo, R, L).unwrap();
    transform_solid(&mut topo, b, &Mat4::rotation_y(FRAC_PI_2)).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(-L / 2.0, 0.0, 0.0)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();

    // Two full cylinders less the lens they share once over.
    let expected = 2.0 * PI * R * R * L - 16.0 / 3.0 * R * R * R;
    let double_counted = 2.0 * PI * R * R * L;
    let v = mass_properties(&topo, fused).unwrap().mass;

    assert!(
        (v - expected).abs() <= REL * expected,
        "expected the closed form {expected:.6}, got {v:.6} ({:+.6}, {:+.4} %)",
        v - expected,
        100.0 * (v - expected) / expected
    );
    assert!(
        (double_counted - v) >= 0.9 * (double_counted - expected),
        "{v:.6} still double-counts most of the lens — two whole cylinders \
         are {double_counted:.6}"
    );
}
