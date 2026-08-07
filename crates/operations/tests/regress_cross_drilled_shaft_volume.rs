//! Regression: `solid_volume` must keep the exact per-face integrator on a
//! cross-drilled shaft, WITHOUT reopening the over-count it declines a
//! circle-outside cone/box fuse for. Both shapes are pinned here, because the
//! defect was a predicate that could not tell them apart.
//!
//! `analytic_faces_solid_volume` declines a solid carrying a "notched" quadric
//! wall whose outer wire is a marched NURBS rim, and hands the body to
//! tessellation. That is right for the wavy band a circle-outside cone/box
//! fuse leaves: its rim marches the WHOLE way round the lateral, so the
//! per-face integrator has no closed outline to trim on and credits the
//! analytic rectangle, over-counting the removed lobes.
//!
//! A cross-drilled bore's wall answers both of those tests too — no inner
//! wires, a single closed NURBS rim visiting three or more axial levels — and
//! differs only in the property that decides whether the integrator can see
//! it: its rim CLOSES within the period instead of winding it. Declining it
//! sent every cross-drilled shaft to tessellation, which reads the UN-BORED
//! stock:
//!
//! | bore r | tessellated | closed form | error   |
//! |--------|-------------|-------------|---------|
//! |   3    |  848.040240 |  704.230016 | +20.4 % |
//! |   2    |  848.040240 |  777.293907 |  +9.1 % |
//! |   1    |  848.040240 |  829.646029 |  +2.2 % |
//!
//! The same number for three geometrically different holes, converging on
//! `volume(makeCylinder(3, 30))` as the deflection tightens — the signature of
//! a body whose bore was never subtracted at all.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::make_cylinder;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::solid::SolidId;

/// Shaft radius and height.
const R: f64 = 3.0;
const H: f64 = 30.0;

/// Volume of the shaft before it is drilled.
fn stock() -> f64 {
    PI * R * R * H
}

/// A shaft of radius `R`, height `H`, cross-drilled clean through at
/// mid-height by a bore of radius `bore` on the +x axis.
fn cross_drilled_shaft(bore: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, R, H).unwrap();
    // Long enough to exit both sides, centred on the shaft's axis at H/2.
    let len = H + 4.0 * R;
    let tool = make_cylinder(&mut topo, bore, len).unwrap();
    transform_solid(&mut topo, tool, &Mat4::rotation_y(FRAC_PI_2)).unwrap();
    transform_solid(
        &mut topo,
        tool,
        &Mat4::translation(-len / 2.0, 0.0, H / 2.0),
    )
    .unwrap();
    let res = boolean(&mut topo, BooleanOp::Cut, shaft, tool).unwrap();
    (topo, res)
}

/// Volume shared by two orthogonal cylinders of radii `a` and `b <= a` whose
/// axes meet — the material a cross-drill removes:
/// `8 * integral_0^b sqrt(a^2 - y^2) * sqrt(b^2 - y^2) dy`.
///
/// Written as quadrature because the closed form is elliptic for `b < a`. At
/// `b == a` it is the Steinmetz solid `16 a^3 / 3`, which
/// [`steinmetz_matches_its_closed_form_at_equal_radii`] checks, so the rule
/// itself is pinned rather than trusted.
fn shared_volume(a: f64, b: f64) -> f64 {
    let n = 200_000_usize;
    let h = b / n as f64;
    let f = |y: f64| ((a * a - y * y).max(0.0)).sqrt() * ((b * b - y * y).max(0.0)).sqrt();
    let mut s = f(0.0) + f(b);
    for i in 1..n {
        #[allow(clippy::cast_precision_loss)]
        let y = i as f64 * h;
        s += if i % 2 == 1 { 4.0 } else { 2.0 } * f(y);
    }
    8.0 * s * h / 3.0
}

#[test]
fn steinmetz_matches_its_closed_form_at_equal_radii() {
    let q = shared_volume(R, R);
    let closed = 16.0 / 3.0 * R * R * R;
    assert!(
        (q - closed).abs() <= 1e-6 * closed,
        "quadrature {q:.9} vs closed form {closed:.9}"
    );
}

/// THE defect, stated as the symptom that identifies it: a drilled shaft is
/// not the stock it was cut from, and three different bores do not remove the
/// same amount of material.
///
/// This is deliberately loose — it asserts only that the bore was subtracted
/// at all, and that the answer moves with the bore radius. A body measured as
/// un-bored stock fails it at every radius, and fails the second half however
/// the tessellation is tuned, because it returns ONE number for all three.
#[test]
fn a_cross_drilled_shaft_is_not_measured_as_unbored_stock() {
    let mut measured = Vec::new();
    for bore in [3.0_f64, 2.0, 1.0] {
        let (topo, solid) = cross_drilled_shaft(bore);
        let v = solid_volume(&topo, solid, 0.08).unwrap();
        let removed = stock() - v;
        let should_remove = shared_volume(R, bore);
        assert!(
            removed > 0.5 * should_remove,
            "bore r={bore}: measured {v:.6} removes only {removed:.6} of the \
             {should_remove:.6} a bore that size takes out of the {:.6} stock — \
             the hole is missing from the measurement",
            stock()
        );
        measured.push(v);
    }
    for (i, a) in measured.iter().enumerate() {
        for b in &measured[i + 1..] {
            assert!(
                (a - b).abs() > 1.0,
                "three different bore radii measured the same volume ({a:.6}, \
                 {b:.6}); the bore is not being subtracted"
            );
        }
    }
}

/// The exact integrator is what measures the shaft, and at equal radii its
/// answer is the closed form.
///
/// `1e-4` relative is the residual chording of the bore rim's own polyline,
/// not slack: the measured value is 704.263359 against 704.230016.
#[test]
fn a_cross_drilled_shaft_keeps_the_analytic_integrator() {
    let (topo, solid) = cross_drilled_shaft(R);
    let expected = stock() - 16.0 / 3.0 * R * R * R;
    let v = solid_volume(&topo, solid, 0.08).unwrap();
    assert!(
        (v - expected).abs() <= 1e-4 * expected,
        "expected the closed form {expected:.6}, got {v:.6}"
    );

    // Deflection-independence is the proof it is NOT tessellating: the
    // tessellated reading of this body changes with deflection (848.040 at
    // 0.08, 848.219 at 1e-4) and the analytic one does not.
    let fine = solid_volume(&topo, solid, 1e-4).unwrap();
    assert!(
        (v - fine).abs() <= 1e-9 * expected,
        "volume moved with deflection ({v:.9} at 0.08, {fine:.9} at 1e-4), so \
         the body is being tessellated rather than integrated"
    );
}

/// The other side of the same predicate: the shape the decline exists for must
/// still be declined.
///
/// A box smaller than the cone's section circle, fused so its corners poke
/// out, leaves a lateral rim that winds the whole way round — 4 corner
/// ring-arcs alternating with 4 wall arches. The analytic rectangle credits
/// the whole lateral for it, so the body must go to the structured
/// tessellator. Closed form: cone 208π + box 288 − overlap 159.00.
#[test]
fn the_circle_outside_cone_box_fuse_is_still_declined() {
    let mut topo = Topology::new();
    let cone = brepkit_operations::primitives::make_cone(&mut topo, 6.0, 2.0, 12.0).unwrap();
    let b = brepkit_operations::primitives::make_box(&mut topo, 6.0, 6.0, 8.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(-3.0, -3.0, 6.0)).unwrap();
    let result =
        brepkit_algo::gfa::boolean(&mut topo, brepkit_algo::bop::BooleanOp::Fuse, cone, b).unwrap();

    let vol = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (vol - 782.449).abs() < 1.0,
        "volume {vol} should be ~782.449; the historical broken readings were \
         921.7 (whole lateral credited) and 318.4 (cone dropped)"
    );
}

/// The bore radii the exact integrator still does not measure correctly, with
/// the quadrature truths written out so a fix has something to land on.
///
/// This is NOT a tessellation problem and NOT the dispatch defect the rest of
/// this file covers — with the analytic path restored these read 750.651763
/// and 802.579475, and they were the same before the dispatch regression too.
/// The cause is upstream in the B-rep: `algebraic_cylinder_cylinder`
/// (crates/math/src/analytic_intersection.rs) samples the two cylinders'
/// intersection at 128 angular stations, DROPS the stations where the ring
/// misses without recording that a gap was there, then closes one NURBS
/// through the survivors. For `bore < R` the two openings are disjoint, so the
/// fitted curve runs from one lobe to the other through solid material — 1.25
/// mm off a 3 mm shaft at bore r=2, 2.96 mm at r=1. No integrator can recover
/// the right volume from a hole outlined that far from where it is. At
/// `bore == R` no station is dropped, which is why only that radius has ever
/// measured right.
///
/// Splitting the samples into contiguous windows fixes the curves (deviation
/// falls to 1.5e-4) but leaves the two rims disjoint on the BORE cylinder,
/// where the face splitter cannot yet pair two period-wrapping rims into one
/// band and drops the middle of the tube. Both halves are needed.
#[test]
#[ignore = "B-rep defect upstream of measurement: algebraic_cylinder_cylinder \
            splices the two disjoint bore lobes into one curve for bore < R"]
fn a_cross_drilled_shaft_measures_its_closed_form_at_every_bore_radius() {
    for bore in [3.0_f64, 2.0, 1.0] {
        let (topo, solid) = cross_drilled_shaft(bore);
        let expected = stock() - shared_volume(R, bore);
        let v = solid_volume(&topo, solid, 0.08).unwrap();
        assert!(
            (v - expected).abs() <= 1e-4 * expected,
            "bore r={bore}: expected {expected:.6}, got {v:.6} \
             ({:+.4} %)",
            (v - expected) / expected * 100.0
        );
    }
}
