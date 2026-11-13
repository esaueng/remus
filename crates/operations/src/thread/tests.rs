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
fn print_form_takes_a_start_chamfer_and_refuses_misplaced_ones() {
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

    // A chamfer on an end that stops short of the face's end refuses, and
    // so do chamfers longer than the thread.
    let banded = CylinderThreadOptions {
        offset: 1.0,
        ..options
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &form, &banded)),
        ThreadRefusal::InvalidParameter
    );
    let steep = CylinderThreadOptions {
        length: Some(1.5),
        ..both_chamfers(PI / 12.0)
    };
    assert_eq!(
        refusal(cut_thread(&mut topo, body, face, &form, &steep)),
        ThreadRefusal::InvalidParameter
    );
}
