#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_2;

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

/// The thread surface's radius at `(θ, z)` in the frame of a thread cut
/// with phase 0: the axial profile (crest flat, upper flank, root flat,
/// lower flank) at `z − θ/k`, independent of the kernel's outline code.
fn thread_radius(form: &ThreadForm, crest: f64, root: f64, theta: f64, z: f64) -> f64 {
    let k = match form.hand {
        ThreadHand::Right => TAU / form.lead(),
        ThreadHand::Left => -TAU / form.lead(),
    };
    let u = (z - theta / k).rem_euclid(form.pitch);
    let depth = (crest - root).abs();
    let upper = depth * form.upper_flank_angle.tan();
    let lower = depth * form.lower_flank_angle.tan();
    let a = form.crest_width;
    let b = a + upper;
    let c = b + form.root_width();
    if u < a {
        crest
    } else if u < b {
        crest + (root - crest) * (u - a) / upper
    } else if u < c {
        root
    } else {
        root + (crest - root) * (u - c) / lower
    }
}

/// Volume of a thread's material between radii `inner` and the thread
/// surface (external) or the thread surface and `outer` (internal) over
/// `[0, length]`, clipped by `cone(z)`, by midpoint quadrature in `θ` and
/// `z`.
fn chamfered_volume(
    form: &ThreadForm,
    kind: ThreadKind,
    crest: f64,
    root: f64,
    bound: f64,
    length: f64,
    cone: impl Fn(f64) -> f64,
) -> f64 {
    let (nz, nt) = (4000_usize, 4000_usize);
    let dz = length / nz as f64;
    let dt = TAU / nt as f64;
    let mut total = 0.0;
    for iz in 0..nz {
        let z = (iz as f64 + 0.5) * dz;
        let c = cone(z);
        let mut area = 0.0;
        for it in 0..nt {
            let theta = (it as f64 + 0.5) * dt;
            let r = thread_radius(form, crest, root, theta, z);
            area += match kind {
                ThreadKind::External => 0.5 * (r.min(c).powi(2) - bound * bound),
                ThreadKind::Internal => 0.5 * (bound * bound - r.max(c).powi(2)),
            } * dt;
        }
        total += area * dz;
    }
    total
}

/// As [`chamfered_volume`] for any end finishes: `clip(r, z)` is the radius
/// of the material's boundary where the thread's own is `r` at height `z`.
fn finished_volume(
    form: &ThreadForm,
    kind: ThreadKind,
    crest: f64,
    root: f64,
    bound: f64,
    length: f64,
    clip: impl Fn(f64, f64) -> f64,
) -> f64 {
    let (nz, nt) = (4000_usize, 4000_usize);
    let dz = length / nz as f64;
    let dt = TAU / nt as f64;
    let mut total = 0.0;
    for iz in 0..nz {
        let z = (iz as f64 + 0.5) * dz;
        let mut area = 0.0;
        for it in 0..nt {
            let theta = (it as f64 + 0.5) * dt;
            let r = clip(thread_radius(form, crest, root, theta, z), z);
            area += match kind {
                ThreadKind::External => 0.5 * (r * r - bound * bound),
                ThreadKind::Internal => 0.5 * (bound * bound - r * r),
            } * dt;
        }
        total += area * dz;
    }
    total
}

fn both_chamfers(angle: f64) -> CylinderThreadOptions {
    let chamfer = ThreadEnd::Chamfer { angle };
    CylinderThreadOptions {
        start: chamfer,
        end: chamfer,
        ..CylinderThreadOptions::default()
    }
}

#[test]
fn chamfered_external_ends_taper_into_the_cone() {
    let mut topo = Topology::new();
    let (r, h) = (3.0, 8.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let outcome = cut_thread(&mut topo, body, face, &form, &both_chamfers(PI / 4.0)).unwrap();
    assert!(outcome.glued);
    assert_valid_and_watertight(&topo, outcome.solid);
    let root = r - form.depth;
    let at_end = root - 0.05 * form.depth;
    let expected = chamfered_volume(&form, ThreadKind::External, r, root, 0.0, h, |z| {
        at_end + z.min(h - z)
    });
    // The face integrals measure the body (its shell closes, see
    // `chamfer_cones_integrate_their_own_region`): 6.2e-6 from the
    // quadrature, which itself sits within 4e-8 of its converged value.
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-5, "{got} vs {expected}");
    // The two chamfers take about 1.9 volume units off the plain body.
    let plain = outline_area(&form, r, root) * h;
    assert!(plain - got > 1.5, "{plain} vs {got}");
    // On each end plane the chamfer clears the crest: a point just inside
    // the crest radius there is air, a point inside the cone is material.
    for z in [1e-3, h - 1e-3] {
        let air = Point3::new(r - 0.02, 0.0, z);
        let core = Point3::new(at_end - 0.02, 0.0, z);
        assert_eq!(
            classify_point(&topo, outcome.solid, air, 0.01, 1e-6).unwrap(),
            PointClassification::Outside
        );
        assert_eq!(
            classify_point(&topo, outcome.solid, core, 0.01, 1e-6).unwrap(),
            PointClassification::Inside
        );
    }
}

#[test]
fn chamfered_internal_ends_countersink_the_thread() {
    let mut topo = Topology::new();
    let form = ThreadForm::iso_metric(1.0, ThreadKind::Internal);
    let (outer, h) = (6.0, 6.0);
    let minor = 3.0 - form.depth;
    let (body, face) = bored(&mut topo, outer, minor, h);
    let outcome = cut_thread(&mut topo, body, face, &form, &both_chamfers(PI / 4.0)).unwrap();
    assert!(outcome.glued);
    assert_valid_and_watertight(&topo, outcome.solid);
    let at_end = 3.0 + 0.05 * form.depth;
    let expected = chamfered_volume(&form, ThreadKind::Internal, minor, 3.0, outer, h, |z| {
        at_end - z.min(h - z)
    });
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 1e-5, "{got} vs {expected}");
}

/// The vector area `½∮ x × dx` of `face`'s wires, densely sampled: the
/// face's oriented `∫ n dA`, which depends on its boundary alone.
fn boundary_vector_area(topo: &Topology, face: FaceId) -> remus_math::vec::Vec3 {
    use remus_math::vec::Vec3;
    use remus_topology::edge::EdgeCurve;
    let f = topo.face(face).unwrap();
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    for wire in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
        let mut points = Vec::new();
        for oe in topo.wire(wire).unwrap().edges() {
            let e = topo.edge(oe.edge()).unwrap();
            let start = topo.vertex(e.start()).unwrap().point();
            let end = topo.vertex(e.end()).unwrap().point();
            let (t0, t1) = if matches!(e.curve(), EdgeCurve::Line) {
                (0.0, 1.0)
            } else {
                e.strict_domain().unwrap()
            };
            let (a, b) = if oe.is_forward() { (t0, t1) } else { (t1, t0) };
            let n = 4000;
            points.extend((0..n).map(|k| {
                let t = (b - a).mul_add(k as f64 / f64::from(n), a);
                e.curve().evaluate_with_endpoints(t, start, end)
            }));
        }
        for (i, &p) in points.iter().enumerate() {
            let q = points[(i + 1) % points.len()];
            sum += Vec3::new(p.x(), p.y(), p.z()).cross(Vec3::new(q.x(), q.y(), q.z())) * 0.5;
        }
    }
    if f.is_reversed() { -sum } else { sum }
}

/// A chamfer cone is one full turn of the cone at the thread's foot plus a
/// tab that climbs between the flanks, past the end of that turn in the
/// cone's angle. Its integrated vector area must be its boundary's, and the
/// shell must close: the face integrator wrapped the tab back into one turn,
/// counted part of the cone twice, and read the external thread's first cone
/// 0.69 off its boundary (the shell 2.7e-3 of its area open), so the volume
/// fell back to the whole-solid mesh.
#[test]
fn chamfer_cones_integrate_their_own_region() {
    use remus_math::vec::Vec3;
    use remus_topology::face::FaceSurface;
    let mut topo = Topology::new();
    let (external, face) = boss(&mut topo, 3.0, 8.0);
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let external = cut_thread(&mut topo, external, face, &form, &both_chamfers(PI / 4.0))
        .unwrap()
        .solid;
    let form = ThreadForm::iso_metric(1.0, ThreadKind::Internal);
    let (internal, face) = bored(&mut topo, 6.0, 3.0 - form.depth, 6.0);
    let internal = cut_thread(&mut topo, internal, face, &form, &both_chamfers(PI / 4.0))
        .unwrap()
        .solid;
    for (label, solid) in [("external", external), ("internal", internal)] {
        let reference = Point3::new(0.0, 0.0, 4.0);
        let (mut closure, mut area, mut cones) = (Vec3::new(0.0, 0.0, 0.0), 0.0, 0);
        for face in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
            let term = remus_check::properties::face_cache::integrate_face_volume_about_memoized(
                &topo, face, 5, reference,
            )
            .unwrap();
            let n = term.vector_area.unwrap();
            closure += n;
            area += term.area;
            if matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_)) {
                cones += 1;
                let gap = (n - boundary_vector_area(&topo, face)).length();
                assert!(
                    gap <= 1e-3 * term.area,
                    "{label} cone: {gap:e} of {}",
                    term.area
                );
            }
        }
        assert_eq!(cones, 2, "{label}");
        assert!(
            closure.length() <= 1e-6 * area,
            "{label}: {:e} of {area}",
            closure.length()
        );
    }
}

#[test]
fn print_form_takes_a_start_chamfer_and_refuses_overlapping_ones() {
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
        hand: ThreadHand::Left,
    };
    let options = CylinderThreadOptions {
        start: ThreadEnd::Chamfer { angle: PI / 6.0 },
        ..CylinderThreadOptions::default()
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    let root = r - form.depth;
    let at_end = root - 0.05 * form.depth;
    let slope = (PI / 6.0).tan();
    let expected = chamfered_volume(&form, ThreadKind::External, r, root, 0.0, h, |z| {
        slope.mul_add(z, at_end)
    });
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");

    let ThreadEndGeometry::Chamfer { length: reach } = outcome.geometry.start else {
        panic!("a chamfer: {:?}", outcome.geometry.start);
    };
    // From just inside the root (a twentieth of the depth) to the crest.
    assert!((reach - 1.05 * form.depth / slope).abs() < 1e-12);
    assert_eq!(outcome.geometry.end, ThreadEndGeometry::Plain);

    // Chamfers longer than the thread refuse with their own code.
    let steep = CylinderThreadOptions {
        length: Some(1.5),
        ..both_chamfers(PI / 12.0)
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &form, &steep)),
        ThreadRefusal::EndsOverlap
    );
}

/// Volume of a runout zone's section from the axis, in closed form. At cone
/// radius `c` the section is the outline of the same form with its root
/// raised to `c`, whose area is a cubic in `c`; `c` is linear along the
/// zone, so Simpson's rule integrates it exactly. For an internal thread
/// that section is the air inside the thread.
fn runout_volume(form: &ThreadForm, crest: f64, root: f64, angle: f64) -> f64 {
    let height = (crest - root).abs() / angle.tan();
    let area = |c: f64| {
        let raised = ThreadForm {
            depth: (crest - c).abs(),
            ..form.clone()
        };
        outline_area(&raised, crest, c)
    };
    height / 6.0 * (4.0f64.mul_add(area(0.5 * (root + crest)), area(root)) + area(crest))
}

fn role_faces(outcome: &ThreadOutcome, role: ThreadFaceRole) -> Vec<FaceId> {
    outcome
        .face_roles
        .iter()
        .filter(|(_, r)| *r == role)
        .map(|(face, _)| *face)
        .collect()
}

/// Every role names a distinct face of the solid, lands are on cylinders
/// (cones on a tapered thread), flanks are swept and end cones are cones.
fn assert_roles_are_faces_of(topo: &Topology, outcome: &ThreadOutcome) {
    assert_roles(topo, outcome, false);
}

/// As [`assert_roles_are_faces_of`]. `long_finish` allows swept lands: the
/// zone of an end finish is one segment whatever its height, and a land
/// whose strip passes a full turn there is swept like a flank.
fn assert_roles(topo: &Topology, outcome: &ThreadOutcome, long_finish: bool) {
    let tapered = outcome.geometry.taper_angle > 0.0;
    let faces = solid_faces(topo, outcome.solid).unwrap();
    let mut seen = std::collections::BTreeSet::new();
    for &(face, role) in &outcome.face_roles {
        assert!(faces.contains(&face), "{} is not a face", role.name());
        assert!(seen.insert(face.index()), "{} twice", role.name());
        let surface = topo.face(face).unwrap().surface();
        match role {
            ThreadFaceRole::Crest | ThreadFaceRole::Root => {
                let land = if tapered {
                    matches!(surface, FaceSurface::Cone(_))
                } else {
                    matches!(surface, FaceSurface::Cylinder(_))
                };
                let swept = long_finish && matches!(surface, FaceSurface::Nurbs(_));
                assert!(land || swept, "{}", role.name());
            }
            ThreadFaceRole::UpperFlank | ThreadFaceRole::LowerFlank => {
                assert!(matches!(surface, FaceSurface::Nurbs(_)), "{}", role.name());
            }
            ThreadFaceRole::Chamfer(_) | ThreadFaceRole::Runout(_) => {
                assert!(matches!(surface, FaceSurface::Cone(_)), "{}", role.name());
            }
        }
    }
}

fn inside(topo: &Topology, solid: SolidId, point: Point3) -> bool {
    classify_point(topo, solid, point, 0.01, 1e-6).unwrap() == PointClassification::Inside
}

#[test]
fn plain_thread_reports_a_role_for_every_land_and_flank() {
    let mut topo = Topology::new();
    let (body, face) = boss(&mut topo, 3.0, 4.0);
    let form = ThreadForm {
        starts: 2,
        ..ThreadForm::iso_metric(1.0, ThreadKind::External)
    };
    let outcome = cut_thread(
        &mut topo,
        body,
        face,
        &form,
        &CylinderThreadOptions::default(),
    )
    .unwrap();
    assert!(outcome.glued);
    assert_roles_are_faces_of(&topo, &outcome);
    // Two turns of a two-start thread in half-turn segments: four segments
    // of two lobes each.
    for role in [
        ThreadFaceRole::Crest,
        ThreadFaceRole::Root,
        ThreadFaceRole::UpperFlank,
        ThreadFaceRole::LowerFlank,
    ] {
        assert_eq!(role_faces(&outcome, role).len(), 8, "{}", role.name());
    }
    assert_eq!(outcome.face_roles.len(), 32);
    // The upper flank faces the end of the thread: its outward normal has a
    // positive axial part, the lower flank's a negative one.
    for (role, sign) in [
        (ThreadFaceRole::UpperFlank, 1.0),
        (ThreadFaceRole::LowerFlank, -1.0),
    ] {
        for flank in role_faces(&outcome, role) {
            let data = topo.face(flank).unwrap();
            let FaceSurface::Nurbs(surface) = data.surface() else {
                panic!("flank is not swept");
            };
            let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
            let normal = surface.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).unwrap();
            let outward = if data.is_reversed() { -normal } else { normal };
            assert!(sign * outward.z() > 0.1, "{}: {outward:?}", role.name());
        }
    }
}

/// A bolt: the thread stops short of the head end of the shank, which stays
/// at the major diameter. The chamfer asked for there is a runout.
#[test]
fn runout_fades_an_external_thread_into_a_flush_collar() {
    let mut topo = Topology::new();
    let (r, h, length) = (3.0, 10.0, 6.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let angle = PI / 4.0;
    let options = CylinderThreadOptions {
        length: Some(length),
        end: ThreadEnd::Chamfer { angle },
        ..tight(&form)
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert!(outcome.glued);
    assert_valid_and_watertight(&topo, outcome.solid);
    let root = r - form.depth;
    let reach = form.depth / angle.tan();
    assert_eq!(outcome.geometry.start, ThreadEndGeometry::Plain);
    let ThreadEndGeometry::Runout { length: reported } = outcome.geometry.end else {
        panic!(
            "the stopped-short end is a runout: {:?}",
            outcome.geometry.end
        );
    };
    assert!((reported - reach).abs() < 1e-12);
    let expected = outline_area(&form, r, root) * (length - reach)
        + runout_volume(&form, r, root, angle)
        + PI * r * r * (h - length);
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 1e-6, "{got} vs {expected}");
    // The runout puts back material the plain thread cuts away.
    assert!(got - (outline_area(&form, r, root) * length + PI * r * r * (h - length)) > 0.5);

    assert_roles_are_faces_of(&topo, &outcome);
    let pieces = role_faces(&outcome, ThreadFaceRole::Runout(ThreadEndSide::End));
    // The groove's patch turns 1.1 times about the axis: three pieces of at
    // most half a turn.
    assert!(pieces.len() >= 3, "{} pieces", pieces.len());
    assert!(role_faces(&outcome, ThreadFaceRole::Runout(ThreadEndSide::Start)).is_empty());
    assert!(role_faces(&outcome, ThreadFaceRole::Chamfer(ThreadEndSide::End)).is_empty());

    // No undercut: just under the end plane the groove is gone all the way
    // round, and the cone's line holds at every height of the zone.
    for k in 0..12 {
        let theta = TAU * f64::from(k) / 12.0 + 0.1;
        let at = |radius: f64, z: f64| Point3::new(radius * theta.cos(), radius * theta.sin(), z);
        assert!(inside(&topo, outcome.solid, at(r - 0.01, length - 2e-3)));
        for step in 1..4 {
            let z = length - reach * f64::from(step) / 4.0;
            let cone = r - (length - z);
            assert!(
                inside(&topo, outcome.solid, at(cone - 0.01, z)),
                "{theta} {z}"
            );
        }
    }
    // Below the zone the groove is at full depth somewhere round the axis.
    let z = length - reach - 0.3;
    let grooved = (0..24).any(|k| {
        let theta = TAU * f64::from(k) / 24.0;
        let radius = root + 0.02;
        !inside(
            &topo,
            outcome.solid,
            Point3::new(radius * theta.cos(), radius * theta.sin(), z),
        )
    });
    assert!(grooved);
}

/// A turned-down band between two collars, left hand and two starts, with
/// a runout at each end.
#[test]
fn runouts_at_both_ends_of_a_turned_down_band() {
    let mut topo = Topology::new();
    let (r, h) = (5.0, 12.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm {
        starts: 2,
        hand: ThreadHand::Left,
        ..ThreadForm::iso_metric(1.5, ThreadKind::External)
    };
    let (offset, length, major) = (2.0, 7.0, 9.6);
    let angle = PI / 6.0;
    let chamfer = ThreadEnd::Chamfer { angle };
    let options = CylinderThreadOptions {
        major_diameter: Some(major),
        offset,
        length: Some(length),
        start: chamfer,
        end: chamfer,
        ..tight(&form)
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert!(outcome.glued);
    assert_valid_and_watertight(&topo, outcome.solid);
    let crest = 0.5 * major;
    let root = crest - form.depth;
    let reach = form.depth / angle.tan();
    for end in [outcome.geometry.start, outcome.geometry.end] {
        let ThreadEndGeometry::Runout { length: reported } = end else {
            panic!("a runout: {end:?}");
        };
        assert!((reported - reach).abs() < 1e-12);
    }
    let expected = PI * r * r * (h - length)
        + outline_area(&form, crest, root) * 2.0f64.mul_add(-reach, length)
        + 2.0 * runout_volume(&form, crest, root, angle);
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 1e-6, "{got} vs {expected}");
    assert_roles_are_faces_of(&topo, &outcome);
    for side in [ThreadEndSide::Start, ThreadEndSide::End] {
        // One patch per start.
        assert!(role_faces(&outcome, ThreadFaceRole::Runout(side)).len() >= 2);
    }
    // The groove is gone on both end planes, where the body steps up from
    // the crest to the collar.
    for z in [offset + 2e-3, offset + length - 2e-3] {
        for k in 0..8 {
            let theta = TAU * f64::from(k) / 8.0;
            let radius = crest - 0.01;
            assert!(inside(
                &topo,
                outcome.solid,
                Point3::new(radius * theta.cos(), radius * theta.sin(), z)
            ));
        }
    }
}

/// A tapped band in a bore that continues past it on both sides: once at
/// the thread's minor diameter (flush collars), once narrower.
#[test]
fn runouts_at_both_ends_of_an_internal_band() {
    for flush in [true, false] {
        let mut topo = Topology::new();
        let form = ThreadForm::iso_metric(1.25, ThreadKind::Internal);
        let (outer, h) = (8.0, 12.0);
        let bore = 3.5;
        let (body, face) = bored(&mut topo, outer, bore, h);
        let minor = if flush { bore } else { bore + 0.05 };
        let major = minor + form.depth;
        let (offset, length) = (3.0, 6.0);
        let angle = PI / 4.0;
        let chamfer = ThreadEnd::Chamfer { angle };
        let options = CylinderThreadOptions {
            major_diameter: Some(2.0 * major),
            offset,
            length: Some(length),
            start: chamfer,
            end: chamfer,
            ..tight(&form)
        };
        let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
        assert!(outcome.glued, "flush={flush}");
        assert_valid_and_watertight(&topo, outcome.solid);
        let reach = form.depth / angle.tan();
        // The air inside the band, less the bore that was there before.
        let air = outline_area(&form, minor, major) * 2.0f64.mul_add(-reach, length)
            + 2.0 * runout_volume(&form, minor, major, angle);
        let expected = PI * (outer * outer - bore * bore) * h - (air - PI * bore * bore * length);
        let got = volume(&topo, outcome.solid);
        assert!(
            rel(got, expected) < 1e-6,
            "flush={flush}: {got} vs {expected}"
        );
        assert_roles_are_faces_of(&topo, &outcome);
        for side in [ThreadEndSide::Start, ThreadEndSide::End] {
            assert!(!role_faces(&outcome, ThreadFaceRole::Runout(side)).is_empty());
        }
        // On each end plane the groove is gone: material right outside the
        // minor diameter all the way round.
        for z in [offset + 2e-3, offset + length - 2e-3] {
            for k in 0..8 {
                let theta = TAU * f64::from(k) / 8.0 + 0.2;
                let radius = minor + 0.01;
                assert!(
                    inside(
                        &topo,
                        outcome.solid,
                        Point3::new(radius * theta.cos(), radius * theta.sin(), z)
                    ),
                    "flush={flush} z={z} theta={theta}"
                );
            }
        }
    }
}

/// A stud on a flange: the thread runs out against the shoulder at the
/// start of the face, which only an explicit runout can ask for, and is
/// chamfered at its free end. The whole body against a quadrature of the
/// exact thread under both cones.
#[test]
fn explicit_runout_at_a_shoulder_with_a_chamfered_free_end() {
    let mut topo = Topology::new();
    let (flange, thick) = (6.0, 2.0);
    let (r, h) = (3.0, 7.0);
    let base = make_cylinder(&mut topo, flange, thick).unwrap();
    let stud = make_cylinder(&mut topo, r, thick + h).unwrap();
    let body = boolean(&mut topo, BooleanOp::Fuse, base, stud).unwrap();
    let face = cylinder_face(&topo, body, r);
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let angle = PI / 4.0;
    let options = CylinderThreadOptions {
        start: ThreadEnd::Runout { angle },
        end: ThreadEnd::Chamfer { angle },
        ..CylinderThreadOptions::default()
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    assert!(matches!(
        outcome.geometry.start,
        ThreadEndGeometry::Runout { .. }
    ));
    assert!(matches!(
        outcome.geometry.end,
        ThreadEndGeometry::Chamfer { .. }
    ));
    let root = r - form.depth;
    let at_end = root - 0.05 * form.depth;
    let thread = finished_volume(&form, ThreadKind::External, r, root, 0.0, h, |radius, z| {
        radius.min(at_end + (h - z)).max(r - z)
    });
    let expected = PI * flange * flange * thick + thread;
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
    if outcome.glued {
        assert_roles_are_faces_of(&topo, &outcome);
        assert!(!role_faces(&outcome, ThreadFaceRole::Runout(ThreadEndSide::Start)).is_empty());
        assert_eq!(
            role_faces(&outcome, ThreadFaceRole::Chamfer(ThreadEndSide::End)).len(),
            1
        );
    }
    // No undercut at the shoulder.
    for k in 0..8 {
        let theta = TAU * f64::from(k) / 8.0;
        let radius = r - 0.01;
        assert!(inside(
            &topo,
            outcome.solid,
            Point3::new(radius * theta.cos(), radius * theta.sin(), thick + 2e-3)
        ));
    }
}

/// A sharp crest and a square flank: the patch's arc on the end plane is
/// the whole crest circle, and one of its sides is a helix about the axis.
#[test]
fn runout_of_a_sharp_crested_square_flanked_thread() {
    let mut topo = Topology::new();
    let (r, h, length) = (4.0, 9.0, 6.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm {
        pitch: 2.0,
        depth: 0.8,
        crest_width: 0.0,
        upper_flank_angle: 0.0,
        lower_flank_angle: PI / 4.0,
        starts: 1,
        hand: ThreadHand::Right,
    };
    let angle = PI / 3.0;
    let options = CylinderThreadOptions {
        length: Some(length),
        end: ThreadEnd::Chamfer { angle },
        ..CylinderThreadOptions::default()
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    let root = r - form.depth;
    let reach = form.depth / angle.tan();
    let expected = outline_area(&form, r, root) * (length - reach)
        + runout_volume(&form, r, root, angle)
        + PI * r * r * (h - length);
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
}

#[test]
fn runouts_that_reach_past_each_other_refuse() {
    let mut topo = Topology::new();
    let (body, face) = boss(&mut topo, 3.0, 10.0);
    let before = topo.num_faces();
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    // Each runout reaches depth / tan 15° = 2.02 into a band 3 long.
    let chamfer = ThreadEnd::Chamfer { angle: PI / 12.0 };
    let options = CylinderThreadOptions {
        offset: 2.0,
        length: Some(3.0),
        start: chamfer,
        end: chamfer,
        ..CylinderThreadOptions::default()
    };
    let result = cut_thread(&mut topo, body, face, &form, &options);
    let Err(error) = &result else {
        panic!("expected a refusal");
    };
    assert!(
        error.to_string().starts_with("thread:ends-overlap:"),
        "{error}"
    );
    assert_eq!(refusal(result), ThreadRefusal::EndsOverlap);
    // A chamfer and a runout overlap the same way.
    let mixed = CylinderThreadOptions {
        offset: 0.0,
        ..options.clone()
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &form, &mixed)),
        ThreadRefusal::EndsOverlap
    );
    let flat = CylinderThreadOptions {
        end: ThreadEnd::Runout { angle: 0.5 * PI },
        ..CylinderThreadOptions::default()
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &form, &flat)),
        ThreadRefusal::InvalidParameter
    );
    assert_eq!(before, topo.num_faces());
    // Runouts that meet exactly, with no full-depth thread between, build.
    let reach = form.depth / (PI / 4.0).tan();
    let chamfer = ThreadEnd::Chamfer { angle: PI / 4.0 };
    let touching = CylinderThreadOptions {
        offset: 2.0,
        length: Some(2.0 * reach),
        start: chamfer,
        end: chamfer,
        ..CylinderThreadOptions::default()
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &touching).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    let expected = PI * 9.0 * 2.0f64.mul_add(-reach, 10.0)
        + 2.0 * runout_volume(&form, 3.0, 3.0 - form.depth, PI / 4.0);
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
}

/// The generators that cut a runout patch never split its root arc, keep
/// clear of its corners and leave no piece wider than the limit plus the
/// root arc.
#[test]
fn runout_cut_angles_keep_clear_of_the_patch_corners() {
    let max_width = PI;
    let mut cases = 0;
    for bottom in [0.0, 0.3, 1.5, 3.0, 5.5] {
        for lean in [-9.0, -4.1, -1.0, -0.2, -1e-3, 0.0, 1e-3, 0.2, 1.0, 4.1, 9.0] {
            for widening in [0.0, 1e-3, 0.4, 2.0] {
                for shift in [0.0, 0.31, 0.77] {
                    let t1: f64 = lean - shift * widening;
                    let t2 = t1 + bottom + widening;
                    if t2 - t1 > TAU || t2 <= t1 {
                        continue;
                    }
                    cases += 1;
                    let cuts = runout::cut_angles(bottom, (t1, t2), max_width);
                    let (lo, hi) = (t1.min(0.0), t2.max(bottom));
                    let label = format!("bottom={bottom} top=({t1}, {t2}): {cuts:?}");
                    #[allow(clippy::float_cmp)]
                    for &cut in &cuts {
                        assert!(cut > lo && cut < hi, "{label}");
                        assert!(!(cut > 0.0 && cut < bottom), "{label}");
                        let through = cut == 0.0 || cut == bottom;
                        for corner in [0.0, bottom, t1, t2] {
                            let clear = (cut - corner).abs() > 1e-3 * max_width;
                            assert!(clear || (through && cut == corner), "{label}");
                        }
                    }
                    let mut bounds = vec![lo];
                    bounds.extend(&cuts);
                    bounds.push(hi);
                    for pair in bounds.windows(2) {
                        assert!(pair[1] > pair[0], "{label}");
                        assert!(pair[1] - pair[0] <= bottom + 1.25 * max_width, "{label}");
                    }
                }
            }
        }
    }
    assert!(cases > 300, "{cases}");
}

/// A runout that takes the whole thread, beside a plain end: there is no
/// full-depth segment, so the plain end's cap is the runout zone's own far
/// plane. The ring used to be left open there and the build failed with
/// `ExactOnlyUnattainable`. Both ends, and both spellings of the finish.
#[test]
fn runout_that_takes_the_whole_thread_keeps_the_plain_end_cap() {
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let angle = PI / 4.0;
    let (r, h, offset) = (3.0, 8.0, 2.0);
    let length = form.depth / angle.tan();
    for at_start in [true, false] {
        for finish in [ThreadEnd::Chamfer { angle }, ThreadEnd::Runout { angle }] {
            let mut topo = Topology::new();
            let (body, face) = boss(&mut topo, r, h);
            let (start, end) = if at_start {
                (finish, ThreadEnd::Plain)
            } else {
                (ThreadEnd::Plain, finish)
            };
            let options = CylinderThreadOptions {
                offset,
                length: Some(length),
                start,
                end,
                ..tight(&form)
            };
            let label = format!("at_start={at_start} {finish:?}");
            let outcome = cut_thread(&mut topo, body, face, &form, &options)
                .unwrap_or_else(|error| panic!("{label}: {error}"));
            assert_valid_and_watertight(&topo, outcome.solid);
            let (built, plain) = if at_start {
                (outcome.geometry.start, outcome.geometry.end)
            } else {
                (outcome.geometry.end, outcome.geometry.start)
            };
            assert!(matches!(built, ThreadEndGeometry::Runout { .. }), "{label}");
            assert_eq!(plain, ThreadEndGeometry::Plain, "{label}");
            let expected =
                PI * r * r * (h - length) + runout_volume(&form, r, r - form.depth, angle);
            let got = volume(&topo, outcome.solid);
            assert!(rel(got, expected) < 1e-6, "{label}: {got} vs {expected}");
            // The plain end is a full-depth wedge: just inside the zone's
            // plain plane the groove is open somewhere round the axis.
            let z = if at_start {
                offset + length - 2e-3
            } else {
                offset + 2e-3
            };
            let grooved = (0..24).any(|k| {
                let theta = TAU * f64::from(k) / 24.0;
                let radius = r - form.depth + 0.02;
                !inside(
                    &topo,
                    outcome.solid,
                    Point3::new(radius * theta.cos(), radius * theta.sin(), z),
                )
            });
            assert!(grooved, "{label}");
        }
    }
}

/// The axial widths `(upper flank, lower flank, root)` of a tapered form,
/// from the axial section alone: a flank at angle `α` from the plane across
/// the axis falls `cot α` per unit of height, and the crest and root lines
/// climb `rate` (toward the thread's material for `flank_rate > 0`), so the
/// flank closes the depth at `cot α ± flank_rate`.
fn taper_widths(form: &ThreadForm, flank_rate: f64) -> (f64, f64, f64) {
    let upper = form.depth / (1.0 / form.upper_flank_angle.tan() + flank_rate);
    let lower = form.depth / (1.0 / form.lower_flank_angle.tan() - flank_rate);
    (upper, lower, form.pitch - form.crest_width - upper - lower)
}

/// Area enclosed by a tapered thread's transverse outline with its crest at
/// `crest` and its root `form.depth` from it: as [`outline_area`], with the
/// tapered widths.
fn tapered_outline_area(form: &ThreadForm, flank_rate: f64, crest: f64, root: f64) -> f64 {
    let (upper, lower, root_width) = taper_widths(form, flank_rate);
    let angle = |w: f64| TAU * w / form.lead();
    let flank = |w: f64| angle(w) / 6.0 * root.mul_add(root, crest.mul_add(crest, crest * root));
    let per_pitch = 0.5 * crest * crest * angle(form.crest_width)
        + 0.5 * root * root * angle(root_width)
        + flank(upper)
        + flank(lower);
    f64::from(form.starts) * per_pitch
}

/// Volume from the axis under a tapered thread over `length`, its crest at
/// `crest` on the start plane and moving `rate` per unit of height. The
/// section area is a quadratic in the height, so Simpson's rule is exact.
fn tapered_volume(form: &ThreadForm, kind: ThreadKind, crest: f64, rate: f64, length: f64) -> f64 {
    let (flank_rate, depth) = match kind {
        ThreadKind::External => (rate, -form.depth),
        ThreadKind::Internal => (-rate, form.depth),
    };
    let area = |z: f64| {
        let c = rate.mul_add(z, crest);
        tapered_outline_area(form, flank_rate, c, c + depth)
    };
    length / 6.0 * (4.0f64.mul_add(area(0.5 * length), area(0.0)) + area(length))
}

const INCH: f64 = 25.4;

/// The largest and smallest radius of the body's boundary on the plane at
/// height `z`, over 36 angles, each bisected between `inner` (material) and
/// `outer` (air) to `1e-5`. Ten degrees apart is finer than any land here.
fn radial_extent(topo: &Topology, solid: SolidId, z: f64, inner: f64, outer: f64) -> (f64, f64) {
    let steps = 36;
    let at = |k: usize, radius: f64| {
        let theta = TAU * k as f64 / f64::from(steps) + 0.013;
        Point3::new(radius * theta.cos(), radius * theta.sin(), z)
    };
    let classify = |radii: &[f64]| -> Vec<bool> {
        let points: Vec<Point3> = radii.iter().enumerate().map(|(k, &r)| at(k, r)).collect();
        crate::classify::classify_points(topo, solid, &points, 0.01, 1e-6)
            .unwrap()
            .into_iter()
            .map(|class| class == PointClassification::Inside)
            .collect()
    };
    let mut lo = vec![inner; steps as usize];
    let mut hi = vec![outer; steps as usize];
    assert!(
        classify(&lo).iter().all(|&material| material),
        "inner bound"
    );
    assert!(
        classify(&hi).iter().all(|&material| !material),
        "outer bound"
    );
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let halvings = ((outer - inner) / 1e-5).log2().ceil() as usize;
    for _ in 0..halvings {
        let mid: Vec<f64> = lo.iter().zip(&hi).map(|(a, b)| 0.5 * (a + b)).collect();
        for (k, material) in classify(&mid).into_iter().enumerate() {
            if material {
                lo[k] = mid[k];
            } else {
                hi[k] = mid[k];
            }
        }
    }
    (
        lo.iter().copied().fold(f64::INFINITY, f64::min),
        hi.iter().copied().fold(0.0, f64::max),
    )
}

/// ASME B1.20.1 NPT: threads per inch, pitch diameter at the small end
/// `E0`, hand-tight length `L1` (the gauge plane, where the pitch diameter
/// is `E1`), effective length `L2` (pitch diameter `E2`) and the pipe's
/// outside diameter, in inches.
struct Npt {
    name: &'static str,
    tpi: f64,
    e0: f64,
    l1: f64,
    e1: f64,
    l2: f64,
    e2: f64,
    od: f64,
}

const NPT_SIZES: [Npt; 2] = [
    Npt {
        name: "1/4-18",
        tpi: 18.0,
        e0: 0.47739,
        l1: 0.2278,
        e1: 0.49163,
        l2: 0.4018,
        e2: 0.50250,
        od: 0.540,
    },
    Npt {
        name: "1/2-14",
        tpi: 14.0,
        e0: 0.75843,
        l1: 0.3200,
        e1: 0.77843,
        l2: 0.5337,
        e2: 0.79179,
        od: 0.840,
    },
];

/// A male NPT thread cut on a boss of the pipe's outside diameter, from the
/// pipe end up to the gauge plane, against the standard's table.
#[test]
fn npt_external_threads_match_the_standard() {
    for size in &NPT_SIZES {
        let pitch = INCH / size.tpi;
        let form = ThreadForm::npt(pitch);
        // Thread height 0.8·P; the truncations leave equal flats.
        let h = 0.8 * pitch;
        assert!((form.depth - h).abs() < 1e-12);
        let mut topo = Topology::new();
        let (r, height) = (0.5 * size.od * INCH, size.l2 * INCH + 6.0);
        let (body, face) = boss(&mut topo, r, height);
        let length = size.l1 * INCH;
        let options = CylinderThreadOptions {
            major_diameter: Some(size.e1 * INCH + h),
            length: Some(length),
            taper: Some(ThreadTaper {
                gauge_offset: Some(length),
                ..ThreadTaper::pipe(ThreadEndSide::Start)
            }),
            ..tight(&form)
        };
        let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
        assert!(outcome.glued, "{}", size.name);
        assert_valid_and_watertight(&topo, outcome.solid);
        let g = &outcome.geometry;
        // 1 in 16 on the diameter: a half-angle of 1°47'24".
        let half_angle = (1.0_f64 / 32.0).atan();
        assert!((g.taper_angle - half_angle).abs() < 1e-15);
        assert!((g.taper_angle.to_degrees() - (1.0 + 47.0 / 60.0 + 24.0 / 3600.0)).abs() < 1e-4);
        // The table's diameters are rounded to 1e-5 in.
        let table = 0.6e-5 * INCH;
        let pitch_diameter = |major: f64, minor: f64| 0.5 * (major + minor);
        assert!((g.major_diameter - (size.e1 * INCH + h)).abs() < 1e-12);
        assert!((g.major_diameter - g.minor_diameter - 2.0 * h).abs() < 1e-12);
        let at_start = pitch_diameter(g.start_major_diameter, g.start_minor_diameter);
        let at_end = pitch_diameter(g.end_major_diameter, g.end_minor_diameter);
        assert!(
            (at_start - size.e0 * INCH).abs() < table,
            "{}: E0",
            size.name
        );
        assert!((at_end - size.e1 * INCH).abs() < 1e-9, "{}: E1", size.name);
        // The same cone carried on to the effective length reaches E2.
        let e2 = at_start + size.l2 * INCH / 16.0;
        assert!((e2 - size.e2 * INCH).abs() < table, "{}: E2", size.name);

        // The body itself: on a plane across the axis the crest is the
        // largest radius of the boundary and the root the smallest, and both
        // climb 1 in 32.
        for fraction in [0.25, 0.5, 0.75] {
            let z = fraction * length;
            let major = 0.5 * g.start_major_diameter + z / 32.0;
            let (least, most) = radial_extent(&topo, outcome.solid, z, major - h - 0.2, r + 0.1);
            assert!(
                (most - major).abs() < 2e-5,
                "{}: crest {most} vs {major}",
                size.name
            );
            assert!(
                (least - (major - h)).abs() < 2e-5,
                "{}: root {least} vs {}",
                size.name,
                major - h
            );
        }
        // Crest and root lands are cones of that taper; flanks are swept.
        assert_roles_are_faces_of(&topo, &outcome);
        for role in [ThreadFaceRole::Crest, ThreadFaceRole::Root] {
            let lands = role_faces(&outcome, role);
            assert!(!lands.is_empty());
            for land in lands {
                let FaceSurface::Cone(cone) = topo.face(land).unwrap().surface() else {
                    panic!("{}: a {} that is not a cone", size.name, role.name());
                };
                // The cone's angle is measured from the plane across the axis.
                assert!((cone.half_angle() - (FRAC_PI_2 - half_angle)).abs() < 1e-12);
            }
        }
        let crest = 0.5 * g.start_major_diameter;
        let expected = tapered_volume(&form, ThreadKind::External, crest, 1.0 / 32.0, length)
            + PI * r * r * (height - length);
        let got = volume(&topo, outcome.solid);
        assert!(
            rel(got, expected) < 1e-6,
            "{}: {got} vs {expected}",
            size.name
        );
    }
}

/// ISO 7-1 / BS 21 R (BSPT): threads per inch, the major, pitch and minor
/// diameters on the gauge plane, the gauge length from the small end and
/// the pipe's outside diameter, in millimetres.
struct Bspt {
    name: &'static str,
    tpi: f64,
    major: f64,
    pitch_diameter: f64,
    minor: f64,
    gauge_length: f64,
    od: f64,
}

const BSPT_SIZES: [Bspt; 2] = [
    Bspt {
        name: "R 1/4-19",
        tpi: 19.0,
        major: 13.157,
        pitch_diameter: 12.301,
        minor: 11.445,
        gauge_length: 6.0,
        od: 13.5,
    },
    Bspt {
        name: "R 1/2-14",
        tpi: 14.0,
        major: 20.955,
        pitch_diameter: 19.793,
        minor: 18.631,
        gauge_length: 8.2,
        od: 21.3,
    },
];

/// A male BSPT thread on a boss of the pipe's outside diameter, running
/// two pitches past the gauge plane and out against the pipe.
#[test]
fn bspt_external_threads_match_the_standard_and_run_out() {
    for size in &BSPT_SIZES {
        let pitch = INCH / size.tpi;
        let form = ThreadForm::bspt(pitch);
        // Whitworth: thread height 0.640327·P, 55° included angle. The
        // standard's rounded crests and roots are flats of P/6 here.
        assert!((form.depth - 0.640_327 * pitch).abs() < 1e-12);
        assert!((form.upper_flank_angle.to_degrees() - 27.5).abs() < 1e-12);
        assert!((form.crest_width - pitch / 6.0).abs() < 1e-12);
        let mut topo = Topology::new();
        let (r, height) = (0.5 * size.od, 24.0);
        let (body, face) = boss(&mut topo, r, height);
        let length = 2.0f64.mul_add(pitch, size.gauge_length);
        let angle = PI / 6.0;
        let options = CylinderThreadOptions {
            major_diameter: Some(size.major),
            length: Some(length),
            taper: Some(ThreadTaper {
                gauge_offset: Some(size.gauge_length),
                ..ThreadTaper::pipe(ThreadEndSide::Start)
            }),
            end: ThreadEnd::Chamfer { angle },
            ..tight(&form)
        };
        let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
        assert!(outcome.glued, "{}", size.name);
        assert_valid_and_watertight(&topo, outcome.solid);
        let g = &outcome.geometry;
        // The table's diameters are rounded to 1e-3 mm.
        let table = 0.6e-3;
        assert!((g.major_diameter - size.major).abs() < 1e-12);
        assert!(
            (g.minor_diameter - size.minor).abs() < table,
            "{}: minor",
            size.name
        );
        let pitch_diameter = 0.5 * (g.major_diameter + g.minor_diameter);
        assert!(
            (pitch_diameter - size.pitch_diameter).abs() < table,
            "{}: pitch diameter",
            size.name
        );
        assert!((g.start_major_diameter - (size.major - size.gauge_length / 16.0)).abs() < 1e-12);
        assert!((g.end_major_diameter - (size.major + 2.0 * pitch / 16.0)).abs() < 1e-12);
        assert!((g.taper_angle - (1.0_f64 / 32.0).atan()).abs() < 1e-15);

        // On the gauge plane the body's crest and root are the table's.
        let (least, most) = radial_extent(
            &topo,
            outcome.solid,
            size.gauge_length,
            0.5 * size.minor - 0.2,
            r + 0.1,
        );
        assert!(
            (most - 0.5 * size.major).abs() < 2e-5,
            "{}: {most}",
            size.name
        );
        assert!(
            (least - 0.5 * size.minor).abs() < 0.5 * table + 2e-5,
            "{}: {least}",
            size.name
        );

        // The stopped-short end is a runout: its cone falls from the crest
        // on the end plane to the root cone, which climbs toward it.
        let ThreadEndGeometry::Runout { length: reach } = g.end else {
            panic!("{}: a runout, got {:?}", size.name, g.end);
        };
        let closing = angle.tan() - 1.0 / 32.0;
        assert!((reach - form.depth / closing).abs() < 1e-12);
        // The runout turns 1.2 times about the axis: its crest lands are
        // swept; the full thread's are cones.
        assert_roles(&topo, &outcome, true);
        let cone_lands = role_faces(&outcome, ThreadFaceRole::Crest)
            .into_iter()
            .filter(|&land| matches!(topo.face(land).unwrap().surface(), FaceSurface::Cone(_)))
            .count();
        assert!(cone_lands >= 8, "{}: {cone_lands} cone crests", size.name);
        assert!(!role_faces(&outcome, ThreadFaceRole::Runout(ThreadEndSide::End)).is_empty());
        // The whole body against a quadrature of the exact tapered thread
        // under the runout cone.
        let crest = 0.5 * g.start_major_diameter;
        let crest_end = 0.5 * g.end_major_diameter;
        let thread = tapered_finished_volume(
            &form,
            ThreadKind::External,
            crest,
            1.0 / 32.0,
            length,
            |radius, z| radius.max(crest_end - (length - z) * angle.tan()),
        );
        let expected = thread + PI * r * r * (height - length);
        let got = volume(&topo, outcome.solid);
        assert!(
            rel(got, expected) < 2e-5,
            "{}: {got} vs {expected}",
            size.name
        );
    }
}

/// The radius of a tapered thread's surface at `(θ, z)` for phase 0: the
/// axial profile with the tapered widths at `z − θ/k`, plus the taper.
fn tapered_thread_radius(
    form: &ThreadForm,
    kind: ThreadKind,
    crest: f64,
    rate: f64,
    theta: f64,
    z: f64,
) -> f64 {
    let (flank_rate, depth) = match kind {
        ThreadKind::External => (rate, -form.depth),
        ThreadKind::Internal => (-rate, form.depth),
    };
    let (upper, lower, root_width) = taper_widths(form, flank_rate);
    let k = match form.hand {
        ThreadHand::Right => TAU / form.lead(),
        ThreadHand::Left => -TAU / form.lead(),
    };
    let u = (z - theta / k).rem_euclid(form.pitch);
    let a = form.crest_width;
    let b = a + upper;
    let c = b + root_width;
    let profile = if u < a {
        0.0
    } else if u < b {
        depth * (u - a) / upper
    } else if u < c {
        depth
    } else {
        depth * (1.0 - (u - c) / lower)
    };
    rate.mul_add(z, crest) + profile
}

/// As [`finished_volume`] for a tapered thread, measured from the axis.
fn tapered_finished_volume(
    form: &ThreadForm,
    kind: ThreadKind,
    crest: f64,
    rate: f64,
    length: f64,
    clip: impl Fn(f64, f64) -> f64,
) -> f64 {
    let (nz, nt) = (4000_usize, 4000_usize);
    let dz = length / nz as f64;
    let dt = TAU / nt as f64;
    let mut total = 0.0;
    for iz in 0..nz {
        let z = (iz as f64 + 0.5) * dz;
        let mut area = 0.0;
        for it in 0..nt {
            let theta = (it as f64 + 0.5) * dt;
            let r = clip(tapered_thread_radius(form, kind, crest, rate, theta, z), z);
            area += 0.5 * r * r * dt;
        }
        total += area * dz;
    }
    total
}

/// A female NPT thread tapped into a bore from its mouth, the large end,
/// where the standard's gauge plane is: pitch diameter `E1` on the face.
#[test]
fn npt_internal_thread_opens_toward_the_mouth() {
    let size = &NPT_SIZES[0];
    let pitch = INCH / size.tpi;
    let form = ThreadForm::npt(pitch);
    let h = form.depth;
    let mut topo = Topology::new();
    let (outer, height) = (12.0, 14.0);
    let length = size.l2 * INCH;
    let minor_at_mouth = 0.5 * (size.e1 * INCH - h);
    let minor_small = minor_at_mouth - length / 32.0;
    // A bore a little under the thread's smallest minor diameter.
    let bore = minor_small - 0.1;
    let (body, face) = bored(&mut topo, outer, bore, height);
    let options = CylinderThreadOptions {
        major_diameter: Some(size.e1 * INCH + h),
        length: Some(length),
        taper: Some(ThreadTaper {
            gauge_offset: Some(length),
            ..ThreadTaper::pipe(ThreadEndSide::End)
        }),
        start: ThreadEnd::Chamfer { angle: PI / 4.0 },
        ..tight(&form)
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert!(outcome.glued);
    assert_valid_and_watertight(&topo, outcome.solid);
    let g = &outcome.geometry;
    assert_eq!(g.kind, ThreadKind::Internal);
    assert!((g.start_minor_diameter - 2.0 * minor_at_mouth).abs() < 1e-9);
    assert!((g.end_minor_diameter - 2.0 * minor_small).abs() < 1e-9);
    assert!(
        (0.5 * (g.start_major_diameter + g.start_minor_diameter) - size.e1 * INCH).abs() < 1e-9
    );
    assert!(matches!(g.start, ThreadEndGeometry::Chamfer { .. }));
    assert_roles_are_faces_of(&topo, &outcome);
    for land in role_faces(&outcome, ThreadFaceRole::Crest) {
        assert!(matches!(
            topo.face(land).unwrap().surface(),
            FaceSurface::Cone(_)
        ));
    }
    // The air inside the thread, countersunk at the mouth, against a
    // quadrature of the exact tapered thread over the chamfer cone.
    let root_at_mouth = minor_at_mouth + h;
    let at_end = root_at_mouth + 0.05 * h;
    let air = tapered_finished_volume(
        &form,
        ThreadKind::Internal,
        minor_at_mouth,
        -1.0 / 32.0,
        length,
        |radius, z| radius.max(at_end - z),
    );
    let expected = PI * outer * outer * height - air - PI * bore * bore * (height - length);
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-5, "{got} vs {expected}");
}

/// A taper with a square lower flank, two starts and the left hand, small
/// end up, turned down from the boss: closed-form volume and the flank
/// widths the taper leaves.
#[test]
fn tapered_square_flanked_two_start_thread_small_end_up() {
    let mut topo = Topology::new();
    let (r, h) = (6.0, 10.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm {
        pitch: 2.0,
        depth: 0.8,
        crest_width: 0.5,
        upper_flank_angle: PI / 4.0,
        lower_flank_angle: 0.0,
        starts: 2,
        hand: ThreadHand::Left,
    };
    let ratio = 0.125;
    let (offset, length) = (1.0, 7.0);
    let options = CylinderThreadOptions {
        offset,
        length: Some(length),
        taper: Some(ThreadTaper {
            ratio,
            small_end: ThreadEndSide::End,
            gauge_offset: None,
        }),
        ..tight(&form)
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert!(outcome.glued);
    assert_valid_and_watertight(&topo, outcome.solid);
    let g = &outcome.geometry;
    // Fitted to the boss: the crest is flush with it at the large end.
    assert!((g.start_major_diameter - 2.0 * r).abs() < 1e-12);
    assert!((g.end_major_diameter - ratio.mul_add(-length, 2.0 * r)).abs() < 1e-12);
    // The gauge plane defaults to the small end.
    assert!((g.major_diameter - g.end_major_diameter).abs() < 1e-12);
    // The radius falls 1/16 per unit up the axis: the upper flank, which
    // falls with it, is steeper against the profile and so narrower.
    let (upper, lower, root_width) = taper_widths(&form, -0.5 * ratio);
    assert!((upper - 0.8 / (1.0 - 0.0625)).abs() < 1e-12);
    assert!(lower.abs() < 1e-12);
    assert!((g.root_width - root_width).abs() < 1e-12);
    let expected = PI * r * r * (h - length)
        + tapered_volume(&form, ThreadKind::External, r, -0.5 * ratio, length);
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 1e-6, "{got} vs {expected}");
    assert_roles_are_faces_of(&topo, &outcome);
}

#[test]
fn taper_refusals() {
    let mut topo = Topology::new();
    let size = &NPT_SIZES[0];
    let pitch = INCH / size.tpi;
    let form = ThreadForm::npt(pitch);
    let h = form.depth;
    let r = 0.5 * size.od * INCH;
    let (body, face) = boss(&mut topo, r, 20.0);
    let before = topo.num_faces();
    let pipe = ThreadTaper::pipe(ThreadEndSide::Start);
    let with = |taper: ThreadTaper, major: Option<f64>, length: f64| CylinderThreadOptions {
        major_diameter: major,
        length: Some(length),
        taper: Some(taper),
        ..CylinderThreadOptions::default()
    };
    for ratio in [0.0, -0.0625, 1e-12, 0.999e-3, 0.3, f64::NAN] {
        let taper = ThreadTaper { ratio, ..pipe };
        assert_eq!(
            refusal(cut_thread(
                &mut topo,
                body,
                face,
                &form,
                &with(taper, None, 6.0)
            )),
            ThreadRefusal::TaperOutOfRange,
            "ratio {ratio}"
        );
    }
    // The standard thread carried to its effective length passes the pipe's
    // outside diameter before it gets there: the large end decides.
    let gauged = ThreadTaper {
        gauge_offset: Some(size.l1 * INCH),
        ..pipe
    };
    let major = Some(size.e1 * INCH + h);
    assert_eq!(
        refusal(cut_thread(
            &mut topo,
            body,
            face,
            &form,
            &with(gauged, major, size.l2 * INCH)
        )),
        ThreadRefusal::CylinderTooSmall
    );
    // A long taper on a small major diameter runs its root into the axis at
    // the small end.
    let deep = ThreadTaper {
        ratio: 0.25,
        small_end: ThreadEndSide::Start,
        gauge_offset: Some(19.0),
    };
    assert_eq!(
        refusal(cut_thread(
            &mut topo,
            body,
            face,
            &form,
            &with(deep, Some(6.0), 19.0)
        )),
        ThreadRefusal::TooDeep
    );
    // A chamfer shallower than the taper never clears the crest.
    let shallow = CylinderThreadOptions {
        start: ThreadEnd::Chamfer {
            angle: 1.0_f64.to_radians(),
        },
        ..with(pipe, None, 6.0)
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &form, &shallow)),
        ThreadRefusal::InvalidParameter
    );
    let nowhere = ThreadTaper {
        gauge_offset: Some(f64::INFINITY),
        ..pipe
    };
    assert_eq!(
        refusal(cut_thread(
            &mut topo,
            body,
            face,
            &form,
            &with(nowhere, None, 6.0)
        )),
        ThreadRefusal::InvalidParameter
    );
    assert_eq!(before, topo.num_faces());

    // In a bore, the small end decides: the bore must not be wider than the
    // thread's minor diameter there.
    let (outer, height) = (12.0, 14.0);
    let minor_at_mouth = 0.5 * (size.e1 * INCH - h);
    let (body, face) = bored(&mut topo, outer, minor_at_mouth - 0.05, height);
    let female = ThreadTaper {
        gauge_offset: Some(10.0),
        ..ThreadTaper::pipe(ThreadEndSide::End)
    };
    assert_eq!(
        refusal(cut_thread(
            &mut topo,
            body,
            face,
            &form,
            &with(female, Some(size.e1 * INCH + h), 10.0)
        )),
        ThreadRefusal::BoreTooLarge
    );
    // A cone face is not a cylinder.
    let cone = crate::primitives::make_cone(&mut topo, 4.0, 3.0, 8.0).unwrap();
    let side = solid_faces(&topo, cone)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_)))
        .unwrap();
    assert_eq!(
        refusal(cut_thread(
            &mut topo,
            cone,
            side,
            &form,
            &with(pipe, None, 6.0)
        )),
        ThreadRefusal::NotACylinder
    );
}

/// A deep form under a 45° chamfer: on the NPT form (0.8·P deep) one flank's
/// trim turns 1.3 times about the axis between the cone's root and crest
/// crossings. The cone loop's winding was read off each trim's midpoint,
/// which loses a whole turn there: the cone's end circle came out wound
/// with the face it bounds, and the body failed validation.
#[test]
fn chamfer_trims_that_turn_more_than_once_keep_the_cone_loop_wound() {
    let pitch = 1.5;
    let form = ThreadForm::npt(pitch);
    for internal in [false, true] {
        let mut topo = Topology::new();
        let h = 8.0;
        let (body, face) = if internal {
            bored(&mut topo, 9.0, 4.0, h)
        } else {
            boss(&mut topo, 4.0 + form.depth, h)
        };
        let outcome = cut_thread(&mut topo, body, face, &form, &both_chamfers(PI / 4.0)).unwrap();
        assert!(outcome.glued, "internal={internal}");
        assert_valid_and_watertight(&topo, outcome.solid);
        let (crest, root) = if internal {
            (4.0, 4.0 + form.depth)
        } else {
            (4.0 + form.depth, 4.0)
        };
        let kind = if internal {
            ThreadKind::Internal
        } else {
            ThreadKind::External
        };
        let sign = if internal { 1.0 } else { -1.0 };
        let at_end = root + sign * 0.05 * form.depth;
        let bound = if internal { 9.0 } else { 0.0 };
        let expected = chamfered_volume(&form, kind, crest, root, bound, h, |z| {
            at_end - sign * z.min(h - z)
        });
        let got = volume(&topo, outcome.solid);
        assert!(
            rel(got, expected) < 2e-5,
            "internal={internal}: {got} vs {expected}"
        );
    }
}

/// A flank that covers more than half a turn of the outline (a 50° flank
/// on a deep single-start form): its own angular run was read as the
/// shorter way round, which sent its chamfer trim round the axis backwards.
#[test]
fn chamfer_on_a_flank_wider_than_half_a_turn() {
    let mut topo = Topology::new();
    let (r, h) = (5.0, 7.0);
    let (body, face) = boss(&mut topo, r, h);
    let form = ThreadForm {
        pitch: 2.0,
        depth: 1.0,
        crest_width: 0.05,
        upper_flank_angle: 50.0_f64.to_radians(),
        lower_flank_angle: 0.0,
        starts: 1,
        hand: ThreadHand::Right,
    };
    // The upper flank takes 0.596 of the lead.
    assert!(form.depth * form.upper_flank_angle.tan() > 0.5 * form.lead());
    let options = CylinderThreadOptions {
        start: ThreadEnd::Chamfer { angle: PI / 3.0 },
        end: ThreadEnd::Chamfer { angle: PI / 3.0 },
        ..CylinderThreadOptions::default()
    };
    let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    let root = r - form.depth;
    let at_end = root - 0.05 * form.depth;
    let slope = (PI / 3.0).tan();
    let expected = chamfered_volume(&form, ThreadKind::External, r, root, 0.0, h, |z| {
        slope.mul_add(z.min(h - z), at_end)
    });
    let got = volume(&topo, outcome.solid);
    assert!(rel(got, expected) < 2e-4, "{got} vs {expected}");
}

/// The largest distance from a cone land's boundary (its vertices and the
/// middle of every edge) to the cone it is said to lie on.
fn cone_land_residual(topo: &Topology, outcome: &ThreadOutcome) -> (usize, f64) {
    let (mut lands, mut worst) = (0, 0.0_f64);
    for &(face, role) in &outcome.face_roles {
        if !matches!(role, ThreadFaceRole::Crest | ThreadFaceRole::Root) {
            continue;
        }
        let data = topo.face(face).unwrap();
        let FaceSurface::Cone(cone) = data.surface() else {
            continue;
        };
        lands += 1;
        for oe in topo.wire(data.outer_wire()).unwrap().edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            let (t0, t1) = edge.strict_domain().unwrap();
            let mid = edge
                .curve()
                .evaluate_with_endpoints(0.5 * (t0 + t1), start, end);
            for p in [start, mid, end] {
                let (u, v) = cone.project_point(p);
                worst = worst.max((cone.evaluate(u, v) - p).length());
            }
        }
    }
    (lands, worst)
}

/// A slight taper's lands are slender cones: the apex is `ρ/τ` away and
/// evaluating the cone loses about `ε·ρ/τ`. At a ratio of 1e-12, which was
/// accepted, the lands' own vertices sat 1.27e-3 off their cones under a
/// reported bound of 8.1e-5. The ratio now has a floor, and from the floor
/// up every land's boundary lies on its cone within the reported bound.
#[test]
fn slight_tapers_keep_their_cone_lands_within_the_reported_bound() {
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    for ratio in [MIN_TAPER_RATIO, 1.5e-3, 4e-3, 0.02, 1.0 / 16.0] {
        for small_end in [ThreadEndSide::Start, ThreadEndSide::End] {
            let mut topo = Topology::new();
            let (body, face) = boss(&mut topo, 6.0, 8.0);
            let options = CylinderThreadOptions {
                taper: Some(ThreadTaper {
                    ratio,
                    small_end,
                    gauge_offset: None,
                }),
                ..CylinderThreadOptions::default()
            };
            let outcome = cut_thread(&mut topo, body, face, &form, &options).unwrap();
            let (lands, residual) = cone_land_residual(&topo, &outcome);
            assert!(lands >= 32, "ratio {ratio}: {lands} cone lands");
            assert!(
                residual <= outcome.deviation_bound,
                "ratio {ratio} {small_end:?}: lands {residual:e} off their cones, bound {:e}",
                outcome.deviation_bound
            );
            // Far inside it, in fact: the cones are well conditioned here.
            assert!(residual < 1e-9, "ratio {ratio} {small_end:?}: {residual:e}");
            assert_valid_and_watertight(&topo, outcome.solid);
        }
    }
    // Below the floor the taper refuses, the reviewed ratio among them.
    let mut topo = Topology::new();
    let (body, face) = boss(&mut topo, 6.0, 8.0);
    for ratio in [1e-12, 1e-6, 0.99 * MIN_TAPER_RATIO] {
        let options = CylinderThreadOptions {
            taper: Some(ThreadTaper {
                ratio,
                ..ThreadTaper::pipe(ThreadEndSide::Start)
            }),
            ..CylinderThreadOptions::default()
        };
        assert_eq!(
            refusal(cut_thread(&mut topo, body, face, &form, &options)),
            ThreadRefusal::TaperOutOfRange,
            "ratio {ratio}"
        );
    }
}
