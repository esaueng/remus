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

/// Every role names a distinct face of the solid, lands are on cylinders,
/// flanks are swept and end cones are cones.
fn assert_roles_are_faces_of(topo: &Topology, outcome: &ThreadOutcome) {
    let faces = solid_faces(topo, outcome.solid).unwrap();
    let mut seen = std::collections::BTreeSet::new();
    for &(face, role) in &outcome.face_roles {
        assert!(faces.contains(&face), "{} is not a face", role.name());
        assert!(seen.insert(face.index()), "{} twice", role.name());
        let surface = topo.face(face).unwrap().surface();
        match role {
            ThreadFaceRole::Crest | ThreadFaceRole::Root => {
                assert!(
                    matches!(surface, FaceSurface::Cylinder(_)),
                    "{}",
                    role.name()
                );
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
