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

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Shaft radius and height.
const R: f64 = 3.0;
const H: f64 = 30.0;

/// Bore radii whose breakout rims put a 256-step winding sample exactly on a
/// cylinder meridian. These must take the same exact band-split route as the
/// neighbouring unequal-radius bores, not the faceted boolean fallback.
const MERIDIAN_SAMPLE_RADII: [f64; 2] = [2.5, 2.95];

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
    let cone = remus_operations::primitives::make_cone(&mut topo, 6.0, 2.0, 12.0).unwrap();
    let b = remus_operations::primitives::make_box(&mut topo, 6.0, 6.0, 8.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(-3.0, -3.0, 6.0)).unwrap();
    let result =
        remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Fuse, cone, b).unwrap();

    let vol = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (vol - 782.449).abs() < 1.0,
        "volume {vol} should be ~782.449; the historical broken readings were \
         921.7 (whole lateral credited) and 318.4 (cone dropped)"
    );
}

/// Both halves of the defect, closed: the drilled shaft measures its closed
/// form at every bore radius, not just at `bore == R`.
///
/// The first half was the CURVE. `algebraic_cylinder_cylinder`
/// (crates/math/src/analytic_intersection.rs) swept cylinder 1's full 2π and
/// concatenated every surviving sample into one loop, chording across the
/// angular windows where the quadratic has no real root, so for `bore < R` the
/// fitted curve ran from one lobe to the other through solid material — 2.96 mm
/// off a 3 mm shaft at bore r=1. Fixed by emitting one loop per window (#112).
///
/// The second half was the FACE SPLITTER, not the measurement. Seen from the
/// BORE cylinder those two rims are closed loops that WIND its `u` period once
/// each, and a period-winding loop bounds no disc — it separates the lateral
/// into bands, with the tube the drill leaves inside the shaft as the middle
/// one. `split_face_2d` read them as contractible holes instead, built a disc
/// off each rim and dropped the tube between them. The census hid it: 5 faces,
/// a wall with 2 inner wires and 2 bore-tube faces LOOKS right. What gave it
/// away was that the mesh carried 20 boundary edges and `classify_point` put
/// the bore's centre INSIDE the solid — the bore was never carved, so no
/// integrator could have recovered its volume.
///
/// `1e-4` relative is the residual chording of the rims' own polylines, not
/// slack: measured 704.263359 / 777.295044 / 829.646153 against 704.230016 /
/// 777.293907 / 829.646029.
#[test]
fn a_cross_drilled_shaft_measures_its_closed_form_at_every_bore_radius() {
    for bore in [3.0_f64, 2.95, 2.5, 2.0, 1.0, 0.5] {
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

/// A meridian-coincident sample must not erase the winding separator. The
/// exact result has the shaft wall, the unified bore wall, and two planar
/// caps; the mesh fallback replaces them with dozens of planar facets.
#[test]
fn meridian_sample_breakouts_preserve_the_exact_brep() {
    for bore in MERIDIAN_SAMPLE_RADII {
        let (topo, solid) = cross_drilled_shaft(bore);
        let mut planes = 0;
        let mut cylinders = 0;
        let mut other = 0;
        for fid in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
            match topo.face(fid).unwrap().surface() {
                FaceSurface::Plane { .. } => planes += 1,
                FaceSurface::Cylinder(_) => cylinders += 1,
                _ => other += 1,
            }
        }
        assert!(
            planes == 2 && cylinders >= 2 && other == 0 && planes + cylinders <= 5,
            "bore r={bore}: expected the compact exact two-cap/cylindrical B-rep, \
             got planes={planes} cylinders={cylinders} other={other}; a planar census \
             signals the mesh fallback"
        );
    }
}

/// The exact split must remain a closed, 2-manifold B-rep: every topological
/// edge has exactly two face uses, with no free or over-shared edge.
#[test]
fn meridian_sample_breakouts_are_closed_and_manifold() {
    for bore in MERIDIAN_SAMPLE_RADII {
        let (topo, solid) = cross_drilled_shaft(bore);
        let mut edge_uses = std::collections::BTreeMap::new();
        for fid in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    *edge_uses.entry(oe.edge().index()).or_insert(0_usize) += 1;
                }
            }
        }
        let irregular: Vec<_> = edge_uses
            .into_iter()
            .filter(|(_, uses)| *uses != 2)
            .collect();
        assert!(
            irregular.is_empty(),
            "bore r={bore}: exact B-rep is not a closed 2-manifold; \
             irregular edge uses={irregular:?}"
        );
    }
}

/// The bore is actually CARVED — the check the volume alone cannot make.
///
/// A body measured as un-bored stock and a body whose bore is missing from the
/// B-rep read the same on any integrator, and the face census reads the same
/// too. Ray-casting at points that encode the intent separates them: the bore's
/// axis is void, the material beside it is not.
#[test]
fn a_cross_drilled_shaft_has_its_bore_carved_out() {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};

    let opts = ClassifyOptions::default();
    for bore in [3.0_f64, 2.95, 2.5, 2.0, 1.0, 0.5] {
        let (topo, solid) = cross_drilled_shaft(bore);
        let mut probes = vec![
            // On the bore's axis at mid-height: removed at every radius.
            (
                remus_math::vec::Point3::new(0.0, 0.0, H / 2.0),
                PointClassification::Outside,
            ),
            // Off-axis but still down the bore: removed too — this is the one
            // that catches a tube whose MIDDLE was dropped.
            (
                remus_math::vec::Point3::new(0.9 * R, 0.0, H / 2.0),
                PointClassification::Outside,
            ),
            // Well clear of the bore along the shaft: kept.
            (
                remus_math::vec::Point3::new(0.0, 0.0, H / 6.0),
                PointClassification::Inside,
            ),
        ];
        if bore < R {
            // Beside the bore, still inside the shaft: kept. Only exists when
            // the bore is narrower than the shaft — at equal radii the bore's
            // section at mid-height is the shaft's whole disc.
            probes.push((
                remus_math::vec::Point3::new(0.0, f64::midpoint(bore, R), H / 2.0),
                PointClassification::Inside,
            ));
        }
        for (p, want) in probes {
            let got = classify_point(&topo, solid, p, &opts).unwrap();
            assert_eq!(
                got,
                want,
                "bore r={bore}: ({:.3},{:.3},{:.3}) classified {got:?}, expected {want:?}",
                p.x(),
                p.y(),
                p.z()
            );
        }
    }
}
