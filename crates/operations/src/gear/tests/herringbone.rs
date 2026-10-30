//! Herringbone gear oracles: Cavalieri volumes, both halves' flank points on
//! their own exact involute helicoid, the far cap's profile back in line
//! with the start cap's, unique half-qualified roles, and refusals.

use super::*;

fn herringbone(teeth: u32, helix_deg: f64, face_width: f64, hand: Hand) -> GearParams {
    GearParams {
        helix_angle: helix_deg.to_radians(),
        hand,
        herringbone: true,
        ..GearParams::standard(GearKind::External, 2.0, teeth, face_width)
    }
}

#[test]
fn herringbone_gears_are_valid_and_keep_their_transverse_area() {
    let mut bored = herringbone(19, 25.0, 8.0, Hand::Right);
    bored.bore_diameter = Some(10.0);
    let left = herringbone(23, 30.0, 6.0, Hand::Left);
    let mut internal = GearParams {
        helix_angle: 18.0_f64.to_radians(),
        herringbone: true,
        ..GearParams::standard(GearKind::Internal, 1.5, 36, 5.0)
    };
    internal.rim_diameter = Some(70.0);
    for params in [bored, left, internal] {
        let (topo, outcome) = build_gear(&params, &GearPlacement::default());
        assert!(outcome.spans_along_axis >= 1);
        assert!(outcome.flank_deviation_bound <= flank_tolerance(&params) * 1.01);
        assert_valid_and_watertight(&topo, outcome.solid);
        let expected = exact_profile_area(&params) * params.face_width;
        let volume = solid_volume(&topo, outcome.solid, 1e-4).unwrap();
        assert!(
            ((volume - expected) / expected).abs() < 1e-6,
            "{:?}: volume {volume} vs exact {expected}",
            params.kind
        );
        // Every side face twice, once per half; caps once.
        let (planes, cylinders, nurbs, other) = census(&topo, outcome.solid);
        let z = params.teeth as usize;
        let ring = usize::from(params.bore_diameter.is_some() || params.rim_diameter.is_some());
        assert_eq!((planes, other), (2, 0), "{:?}", params.kind);
        assert_eq!(cylinders, 2 * (2 * z + ring), "{:?}", params.kind);
        assert!(nurbs >= 4 * z, "{:?}", params.kind);
    }
}

/// Polar angle of a point about the `z` axis.
fn polar(p: Point3) -> f64 {
    p.y().atan2(p.x())
}

#[test]
fn each_half_lies_on_its_own_involute_helicoid() {
    for hand in [Hand::Right, Hand::Left] {
        let params = herringbone(21, 30.0, 12.0, hand);
        let placement = GearPlacement {
            phase: 0.15,
            ..GearPlacement::default()
        };
        let (topo, outcome) = build_gear(&params, &placement);
        let c = circular(&params);
        let rate = twist(&params, &outcome.geometry).unwrap().rate;
        let half_width = 0.5 * params.face_width;
        let pitch = TAU / f64::from(params.teeth);
        let mut checked = [0, 0];
        for (face, named) in &outcome.face_roles {
            let GearFaceRole::Flank { tooth, side } = named.role else {
                continue;
            };
            let half = named.half.expect("a herringbone flank has a half");
            let FaceSurface::Nurbs(surface) = topo.face(*face).unwrap().surface() else {
                panic!("a herringbone flank must be a swept surface");
            };
            let center = placement.phase + f64::from(tooth) * pitch;
            let sign = if side == ToothSide::Plus { 1.0 } else { -1.0 };
            let (u0, u1) = surface.domain_u();
            let (v0, v1) = surface.domain_v();
            for i in 0..=8 {
                for j in 0..=8 {
                    let u = u0 + (u1 - u0) * f64::from(i) / 8.0;
                    let v = v0 + (v1 - v0) * f64::from(j) / 8.0;
                    let q = surface.evaluate(u, v);
                    // The turn at this height: up with the start hand to the
                    // mid-plane, back down after it.
                    let turn = rate * q.z().min(params.face_width - q.z());
                    match half {
                        GearEnd::Start => assert!(q.z() <= half_width + 1e-9),
                        GearEnd::End => assert!(q.z() >= half_width - 1e-9),
                    }
                    let (s, cs) = (-turn).sin_cos();
                    let p = Point3::new(cs * q.x() - s * q.y(), s * q.x() + cs * q.y(), 0.0);
                    let rho = p.x().hypot(p.y());
                    let alpha = (c.base_radius / rho).min(1.0).acos();
                    let expected = center + sign * (c.psi0 - (alpha.tan() - alpha));
                    let deviation = (rho * angle_between(polar(p), expected) * alpha.cos()).abs();
                    assert!(
                        deviation <= outcome.flank_deviation_bound + 1e-9,
                        "{hand:?} {}: {deviation:e} > {:e}",
                        named.name(),
                        outcome.flank_deviation_bound
                    );
                    checked[usize::from(half == GearEnd::End)] += 1;
                }
            }
        }
        assert_eq!(checked, [2 * 21 * 81, 2 * 21 * 81]);
    }
}

#[test]
fn the_far_cap_lines_up_with_the_start_cap() {
    let params = herringbone(20, 30.0, 20.0, Hand::Right);
    let (topo, outcome) = build_gear(&params, &GearPlacement::default());
    let r = 0.5 * outcome.geometry.pitch_diameter.unwrap() + 0.5;
    let rate = twist(&params, &outcome.geometry).unwrap().rate;
    let pitch = TAU / 20.0;
    // Tooth 0 sits at angle 0 on both caps and at the full half-width turn
    // on the mid-plane.
    for (z, turn) in [
        (1e-3, 0.0),
        (10.0, rate * 10.0),
        (params.face_width - 1e-3, 0.0),
    ] {
        assert_eq!(
            classify(&topo, outcome.solid, r, turn, z),
            PointClassification::Inside,
            "tooth at z = {z}"
        );
        assert_eq!(
            classify(&topo, outcome.solid, r, turn + 0.5 * pitch, z),
            PointClassification::Outside,
            "space at z = {z}"
        );
    }
}

#[test]
fn every_face_of_a_herringbone_gear_keeps_one_unique_name() {
    let mut params = herringbone(17, 22.0, 7.0, Hand::Left);
    params.bore_diameter = Some(8.0);
    let (topo, outcome) = build_gear(&params, &GearPlacement::default());
    let faces = solid_faces(&topo, outcome.solid).unwrap();
    assert_eq!(outcome.face_roles.len(), faces.len());
    let names: std::collections::BTreeSet<_> =
        outcome.face_roles.iter().map(|(_, r)| r.name()).collect();
    assert_eq!(names.len(), faces.len());
    for name in [
        "cap.start",
        "cap.end",
        "tooth[16].flank.minus.start-half",
        "tooth[16].flank.minus.end-half",
        "bore.start-half",
        "bore.end-half",
    ] {
        assert!(names.contains(name), "{name}");
    }
}

#[test]
fn a_herringbone_without_a_helix_is_refused() {
    let spur = GearParams {
        herringbone: true,
        ..GearParams::standard(GearKind::External, 2.0, 20, 5.0)
    };
    assert_eq!(refusal(&spur).0, GearRefusal::InvalidParameter);
}
