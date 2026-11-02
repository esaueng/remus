#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;

use super::*;
use crate::measure::solid_volume;
use crate::primitives::{make_box, make_cylinder};
use crate::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use crate::transform::transform_solid;
use crate::validate::validate_solid;

fn cylinder_face(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    let matches: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| match topo.face(f).unwrap().surface() {
            FaceSurface::Cylinder(c) => (c.radius() - radius).abs() < 1e-9,
            _ => false,
        })
        .collect();
    assert_eq!(matches.len(), 1, "one cylinder face of radius {radius}");
    matches[0]
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, 1e-4).unwrap()
}

fn assert_valid_and_watertight(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.2).unwrap();
    assert!(is_watertight(&mesh));
}

/// Area enclosed by the exact transverse outline, in closed form: an arc of
/// angle `Δθ` encloses `r²Δθ/2` with the axis, a linear spiral from `r₀` to
/// `r₁` encloses `Δθ(r₀² + r₀r₁ + r₁²)/6`, a radial step nothing. An axial
/// width `w` covers `Δθ = 2πw/lead`.
fn outline_area(form: &ThreadForm, crest: f64, root: f64) -> f64 {
    let lead = form.lead();
    let angle = |w: f64| TAU * w / lead;
    let depth = (crest - root).abs();
    let flank = |a: f64| {
        angle(depth * a.tan()) / 6.0 * root.mul_add(root, crest.mul_add(crest, crest * root))
    };
    let per_pitch = 0.5 * crest * crest * angle(form.crest_width)
        + 0.5 * root * root * angle(form.root_width())
        + flank(form.upper_flank_angle)
        + flank(form.lower_flank_angle);
    f64::from(form.starts) * per_pitch
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

fn boss(topo: &mut Topology, radius: f64, height: f64) -> (SolidId, FaceId) {
    let body = make_cylinder(topo, radius, height).unwrap();
    (body, cylinder_face(topo, body, radius))
}

/// A tube of `outer` radius with a bore of `bore` radius, `height` tall.
fn bored(topo: &mut Topology, outer: f64, bore: f64, height: f64) -> (SolidId, FaceId) {
    let blank = make_cylinder(topo, outer, height).unwrap();
    let hole = make_cylinder(topo, bore, height + 2.0).unwrap();
    transform_solid(topo, hole, &Mat4::translation(0.0, 0.0, -1.0)).unwrap();
    let body = boolean(topo, BooleanOp::Cut, blank, hole).unwrap();
    (body, cylinder_face(topo, body, bore))
}

fn tight(form: &ThreadForm) -> CylinderThreadOptions {
    CylinderThreadOptions {
        tolerance: Some(1e-7 * form.pitch),
        ..CylinderThreadOptions::default()
    }
}

#[test]
fn iso_m6_external_volume_is_the_outline_area_times_length() {
    let mut topo = Topology::new();
    let (r, h) = (3.0, 8.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let outcome = cut_thread(&mut topo, body, face, &form, &tight(&form)).unwrap();
    assert!(outcome.glued);
    let g = &outcome.geometry;
    assert_eq!(g.kind, ThreadKind::External);
    assert!((g.major_diameter - 6.0).abs() < 1e-12);
    assert!((g.minor_diameter - 2.0f64.mul_add(-form.depth, 6.0)).abs() < 1e-12);
    assert!((g.root_width - 0.25).abs() < 1e-12);
    assert!((g.turns - 8.0).abs() < 1e-12);
    assert!(outcome.deviation_bound < 1e-7);
    let expected = outline_area(&form, r, r - form.depth) * h;
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 1e-6, "{got} vs {expected}");
    assert_valid_and_watertight(&topo, outcome.solid);
}

#[test]
fn iso_m6_internal_volume_removes_the_outline_from_the_bore() {
    let mut topo = Topology::new();
    let form = ThreadForm::iso_metric(1.0, ThreadKind::Internal);
    let (outer, h) = (6.0, 6.0);
    let minor = 3.0 - form.depth;
    let (body, face) = bored(&mut topo, outer, minor, h);
    let outcome = cut_thread(&mut topo, body, face, &form, &tight(&form)).unwrap();
    assert!(outcome.glued);
    let g = &outcome.geometry;
    assert_eq!(g.kind, ThreadKind::Internal);
    assert!((g.major_diameter - 6.0).abs() < 1e-9);
    assert!((g.minor_diameter - 2.0 * minor).abs() < 1e-9);
    assert!((g.root_width - 0.125).abs() < 1e-12);
    let expected = PI * outer * outer * h - outline_area(&form, minor, 3.0) * h;
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 1e-6, "{got} vs {expected}");
    assert_valid_and_watertight(&topo, outcome.solid);
}

#[test]
fn turned_down_left_hand_two_start_band_keeps_collars() {
    let mut topo = Topology::new();
    let (r, h) = (5.0, 12.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm {
        starts: 2,
        hand: ThreadHand::Left,
        ..ThreadForm::iso_metric(1.5, ThreadKind::External)
    };
    let (offset, length, major) = (2.0, 7.0, 9.6);
    let options = CylinderThreadOptions {
        major_diameter: Some(major),
        offset,
        length: Some(length),
        ..CylinderThreadOptions::default()
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert!(outcome.glued, "a turned-down band glues onto its collars");
    assert!((outcome.geometry.lead - 3.0).abs() < 1e-12);
    let crest = 0.5 * major;
    let expected =
        PI * r * r * (h - length) + outline_area(&form, crest, crest - form.depth) * length;
    let got = volume(&topo, outcome.solid);
    // The default tolerance: a deviation bound over roughly the thread's
    // swept area.
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
    assert_valid_and_watertight(&topo, outcome.solid);
    // The collars keep the boss diameter on either side of the band.
    for z in [1.0, 10.5] {
        let inside = Point3::new(r - 0.05, 0.0, z);
        assert_eq!(
            classify_point(&topo, outcome.solid, inside, 0.01, 1e-6).unwrap(),
            PointClassification::Inside
        );
    }
}

/// A print-oriented form: a 45° lower flank (no overhang steeper than 45°
/// with the axis up), a square upper flank and wide crest and root flats.
#[test]
fn print_form_with_square_flank_and_sharp_root_builds() {
    let mut topo = Topology::new();
    let (r, h) = (4.0, 6.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm {
        pitch: 2.0,
        depth: 0.8,
        crest_width: 0.6,
        upper_flank_angle: 0.0,
        lower_flank_angle: PI / 4.0,
        starts: 1,
        hand: ThreadHand::Right,
    };
    assert!((form.root_width() - 0.6).abs() < 1e-12);
    let outcome = cut_thread(
        &mut topo,
        body,
        face,
        &form,
        &CylinderThreadOptions::default(),
    )
    .unwrap();
    let expected = outline_area(&form, r, r - form.depth) * h;
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
    assert_valid_and_watertight(&topo, outcome.solid);

    // A sharp crest (no flat) also builds.
    let mut topo = Topology::new();
    let (body, face) = boss(&mut topo, r, h);
    let sharp = ThreadForm {
        crest_width: 0.0,
        ..form
    };
    let outcome = cut_thread(
        &mut topo,
        body,
        face,
        &sharp,
        &CylinderThreadOptions::default(),
    )
    .unwrap();
    let expected = outline_area(&sharp, r, r - sharp.depth) * h;
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
    assert_valid_and_watertight(&topo, outcome.solid);
}

fn refusal(result: Result<ThreadOutcome, ThreadError>) -> ThreadRefusal {
    match result {
        Err(ThreadError::Refused { code, .. }) => code,
        Err(other) => panic!("expected a refusal, got {other}"),
        Ok(_) => panic!("expected a refusal, got a thread"),
    }
}

#[test]
fn refusals_leave_the_topology_unchanged() {
    let mut topo = Topology::new();
    let (body, face) = boss(&mut topo, 3.0, 8.0);
    let before = topo.num_faces();
    let iso = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let defaults = CylinderThreadOptions::default();

    let closed = ThreadForm {
        crest_width: 0.9,
        ..iso.clone()
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &closed, &defaults)),
        ThreadRefusal::RootClosed
    );
    let big = CylinderThreadOptions {
        major_diameter: Some(6.5),
        ..defaults.clone()
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &iso, &big)),
        ThreadRefusal::CylinderTooSmall
    );
    let long = CylinderThreadOptions {
        offset: 2.0,
        length: Some(7.0),
        ..defaults.clone()
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &iso, &long)),
        ThreadRefusal::SpanOutsideFace
    );
    let deep = ThreadForm {
        depth: 3.5,
        upper_flank_angle: 0.0,
        lower_flank_angle: 0.0,
        ..iso.clone()
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &deep, &defaults)),
        ThreadRefusal::TooDeep
    );
    let cap = solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { .. }))
        .unwrap();
    assert_eq!(
        refusal(cut_thread(&mut topo, body, cap, &iso, &defaults)),
        ThreadRefusal::NotACylinder
    );
    let reversed = CylinderThreadOptions {
        length: Some(-1.0),
        ..defaults
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &iso, &reversed)),
        ThreadRefusal::InvalidParameter
    );
    assert_eq!(before, topo.num_faces());
}

#[test]
fn cross_hole_in_the_band_refuses() {
    let mut topo = Topology::new();
    let (body, _) = boss(&mut topo, 3.0, 8.0);
    let slot = make_box(&mut topo, 10.0, 1.0, 1.0).unwrap();
    transform_solid(&mut topo, slot, &Mat4::translation(-5.0, -0.5, 3.5)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, body, slot).unwrap();
    let face = cylinder_face(&topo, body, 3.0);
    let iso = ThreadForm::iso_metric(1.0, ThreadKind::External);
    assert_eq!(
        refusal(cut_thread(
            &mut topo,
            body,
            face,
            &iso,
            &CylinderThreadOptions::default()
        )),
        ThreadRefusal::FeaturesInThreadRegion
    );
}

/// A long thread: twenty-four turns, so forty-eight ring segments.
#[test]
fn long_thread_stacks_segments() {
    let mut topo = Topology::new();
    let (r, h) = (2.5, 19.2);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm::iso_metric(0.8, ThreadKind::External);
    let outcome = cut_thread(
        &mut topo,
        body,
        face,
        &form,
        &CylinderThreadOptions::default(),
    )
    .unwrap();
    assert!((outcome.geometry.turns - 24.0).abs() < 1e-12);
    let expected = outline_area(&form, r, r - form.depth) * h;
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
    assert_valid_and_watertight(&topo, outcome.solid);
}

/// An internal thread over the middle of a bore glues onto the bore's
/// collars on both sides.
#[test]
fn internal_band_between_collars() {
    let mut topo = Topology::new();
    let form = ThreadForm::iso_metric(1.25, ThreadKind::Internal);
    let (outer, h) = (8.0, 12.0);
    let bore = 3.5;
    let (body, face) = bored(&mut topo, outer, bore, h);
    let major = 2.0 * (bore + 0.05 + form.depth);
    let options = CylinderThreadOptions {
        major_diameter: Some(major),
        offset: 3.0,
        length: Some(5.0),
        ..CylinderThreadOptions::default()
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert!(outcome.glued);
    let crest = 0.5 * major - form.depth;
    let expected = PI * (outer * outer - bore * bore) * h
        - (outline_area(&form, crest, 0.5 * major) - PI * bore * bore) * 5.0;
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
    assert_valid_and_watertight(&topo, outcome.solid);
}

/// A crest flush with a collar (a bolt's shank at the major diameter, a
/// tapped hole whose bore continues past the thread) glues on, the step
/// splitting into crescents between the crest arcs.
#[test]
fn flush_collars_split_into_crescents() {
    for internal in [false, true] {
        for phase in [0.0, 1.0, 2.5] {
            let mut topo = Topology::new();
            let kind = if internal {
                ThreadKind::Internal
            } else {
                ThreadKind::External
            };
            let form = ThreadForm::iso_metric(1.0, kind);
            let h = 10.0;
            let (body, face) = if internal {
                bored(&mut topo, 6.0, 3.0 - form.depth, h)
            } else {
                boss(&mut topo, 3.0, h)
            };
            let (offset, length) = (2.0, 5.3);
            let options = CylinderThreadOptions {
                offset,
                length: Some(length),
                phase,
                ..CylinderThreadOptions::default()
            };
            let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
            assert!(outcome.glued, "internal={internal} phase={phase}");
            let expected = if internal {
                let minor = 3.0 - form.depth;
                PI * 36.0 * h
                    - PI * minor * minor * (h - length)
                    - outline_area(&form, minor, 3.0) * length
            } else {
                PI * 9.0 * (h - length) + outline_area(&form, 3.0, 3.0 - form.depth) * length
            };
            let got = volume(&topo, outcome.solid);
            assert!(
                rel(got, expected) < 2e-4,
                "internal={internal} phase={phase}: {got} vs {expected}"
            );
            assert_valid_and_watertight(&topo, outcome.solid);
        }
    }
}
