//! Helical gear oracles: ISO 21771 closed forms, Cavalieri volumes (a
//! twist keeps every transverse section), flank points against the exact
//! involute helicoid, hand and twist direction, and refusals.

use super::*;

fn helical(teeth: u32, helix_deg: f64, face_width: f64, hand: Hand) -> GearParams {
    GearParams {
        helix_angle: helix_deg.to_radians(),
        hand,
        ..GearParams::standard(GearKind::External, 2.0, teeth, face_width)
    }
}

/// Volume against the exact transverse area times the face width, measured
/// fine enough that the swept flanks' quadrature does not dominate.
fn assert_cavalieri(topo: &Topology, outcome: &GearOutcome, params: &GearParams) {
    let expected = exact_profile_area(params) * params.face_width;
    let volume = solid_volume(topo, outcome.solid, 1e-4).unwrap();
    assert!(
        ((volume - expected) / expected).abs() < 1e-6,
        "volume {volume} vs exact {expected}"
    );
}

#[test]
fn helical_dimensions_match_iso_21771() {
    let beta = 20.0_f64.to_radians();
    let g = gear_geometry(&helical(20, 20.0, 10.0, Hand::Right)).unwrap();
    let mt = 2.0 / beta.cos();
    let alpha_t = (20.0_f64.to_radians().tan() / beta.cos()).atan();
    let d = 20.0 * mt;
    assert!((g.module - 2.0).abs() < 1e-12);
    assert!((g.transverse_module - mt).abs() < 1e-12);
    assert!((g.helix_angle - beta).abs() < 1e-15);
    assert!((g.pitch_diameter.unwrap() - d).abs() < 1e-12);
    assert!((g.base_diameter.unwrap() - d * alpha_t.cos()).abs() < 1e-12);
    // Addendum and dedendum stay normal-module lengths.
    assert!((g.tip_diameter.unwrap() - (d + 4.0)).abs() < 1e-12);
    assert!((g.root_diameter.unwrap() - (d - 5.0)).abs() < 1e-12);
    assert!((g.circular_pitch - PI * mt).abs() < 1e-12);
    assert!((g.tooth_thickness - 0.5 * PI * mt).abs() < 1e-12);
    assert!((g.lead.unwrap() - PI * d / beta.tan()).abs() < 1e-9);
    let min_teeth = 2.0 * beta.cos() / alpha_t.sin().powi(2);
    assert!((g.min_teeth_without_undercut.unwrap() - min_teeth).abs() < 1e-12);
    let min_shift = 1.0 - 20.0 * alpha_t.sin().powi(2) / (2.0 * beta.cos());
    assert!((g.min_profile_shift_without_undercut.unwrap() - min_shift).abs() < 1e-12);

    // A spur gear reports itself unchanged.
    let spur = gear_geometry(&spur(20)).unwrap();
    assert!((spur.transverse_module - 2.0).abs() < 1e-15);
    assert!(spur.lead.is_none());
}

#[test]
fn helical_gears_are_valid_and_keep_their_transverse_area() {
    let mut bored = helical(19, 25.0, 8.0, Hand::Right);
    bored.bore_diameter = Some(10.0);
    let left = helical(23, 15.0, 6.0, Hand::Left);
    let mut internal = GearParams {
        helix_angle: 18.0_f64.to_radians(),
        ..GearParams::standard(GearKind::Internal, 1.5, 36, 5.0)
    };
    internal.rim_diameter = Some(70.0);
    for params in [bored, left, internal] {
        let (topo, outcome) = build_gear(&params, &GearPlacement::default());
        assert!(outcome.spans_along_axis >= 1);
        assert!(outcome.flank_deviation_bound <= flank_tolerance(&params) * 1.01);
        assert_valid_and_watertight(&topo, outcome.solid);
        assert_cavalieri(&topo, &outcome, &params);
        // Lands concentric with the axis stay exact cylinders; flanks and
        // fillets are swept surfaces; caps are planes.
        let (planes, cylinders, nurbs, other) = census(&topo, outcome.solid);
        let z = params.teeth as usize;
        let ring = usize::from(params.bore_diameter.is_some() || params.rim_diameter.is_some());
        assert_eq!((planes, other), (2, 0), "{:?}", params.kind);
        assert_eq!(cylinders, 2 * z + ring, "{:?}", params.kind);
        assert!(nurbs >= 2 * z, "{:?}", params.kind);
    }
}

/// Polar angle of a point about the `z` axis.
fn polar(p: Point3) -> f64 {
    p.y().atan2(p.x())
}

#[test]
fn flank_points_lie_on_the_exact_involute_helicoid() {
    for hand in [Hand::Right, Hand::Left] {
        let params = helical(21, 30.0, 12.0, hand);
        let placement = GearPlacement {
            phase: 0.15,
            ..GearPlacement::default()
        };
        let (topo, outcome) = build_gear(&params, &placement);
        let geometry = &outcome.geometry;
        let c = circular(&params);
        let rate = twist(&params, geometry).unwrap().rate;
        let pitch = TAU / f64::from(params.teeth);
        let mut checked = 0;
        for (face, role) in &outcome.face_roles {
            let GearFaceRole::Flank { tooth, side } = role else {
                continue;
            };
            let FaceSurface::Nurbs(surface) = topo.face(*face).unwrap().surface() else {
                panic!("a helical flank must be a swept surface");
            };
            let center = placement.phase + f64::from(*tooth) * pitch;
            let sign = if *side == ToothSide::Plus { 1.0 } else { -1.0 };
            let (u0, u1) = surface.domain_u();
            let (v0, v1) = surface.domain_v();
            for i in 0..=12 {
                for j in 0..=12 {
                    let u = u0 + (u1 - u0) * f64::from(i) / 12.0;
                    let v = v0 + (v1 - v0) * f64::from(j) / 12.0;
                    let q = surface.evaluate(u, v);
                    // Undo the twist at this height: the point must then sit
                    // on the transverse involute.
                    let back = -rate * q.z();
                    let (s, cs) = back.sin_cos();
                    let p = Point3::new(cs * q.x() - s * q.y(), s * q.x() + cs * q.y(), 0.0);
                    let rho = p.x().hypot(p.y());
                    let alpha = (c.base_radius / rho).min(1.0).acos();
                    let expected = center + sign * (c.psi0 - (alpha.tan() - alpha));
                    let offset = angle_between(polar(p), expected);
                    // The tangential offset at this radius, projected onto
                    // the involute's normal.
                    let deviation = (rho * offset * alpha.cos()).abs();
                    assert!(
                        deviation <= outcome.flank_deviation_bound + 1e-9,
                        "{hand:?} {}: {deviation:e} > {:e}",
                        role.name(),
                        outcome.flank_deviation_bound
                    );
                    checked += 1;
                }
            }
        }
        assert_eq!(checked, 2 * 21 * 13 * 13);
    }
}

#[test]
fn the_hand_sets_which_way_the_teeth_turn() {
    for hand in [Hand::Right, Hand::Left] {
        let params = helical(20, 30.0, 20.0, hand);
        let (topo, outcome) = build_gear(&params, &GearPlacement::default());
        let r = 0.5 * outcome.geometry.pitch_diameter.unwrap() + 0.5;
        let turn = twist(&params, &outcome.geometry).unwrap().rate * params.face_width;
        assert_eq!(turn > 0.0, hand == Hand::Right);
        let pitch = TAU / 20.0;
        // At the far cap, tooth 0 has turned by `turn`: its center is
        // material and the middle of the next space is not.
        let z = params.face_width - 1e-3;
        assert_eq!(
            classify(&topo, outcome.solid, r, turn, z),
            PointClassification::Inside
        );
        assert_eq!(
            classify(&topo, outcome.solid, r, turn + 0.5 * pitch, z),
            PointClassification::Outside
        );
    }
}

#[test]
fn every_face_of_a_helical_gear_keeps_one_unique_role() {
    let mut params = helical(17, 22.0, 7.0, Hand::Left);
    params.bore_diameter = Some(8.0);
    let (topo, outcome) = build_gear(&params, &GearPlacement::default());
    let faces = solid_faces(&topo, outcome.solid).unwrap();
    assert_eq!(outcome.face_roles.len(), faces.len());
    let names: std::collections::BTreeSet<_> =
        outcome.face_roles.iter().map(|(_, r)| r.name()).collect();
    assert_eq!(names.len(), faces.len());
    assert!(names.contains("tooth[16].flank.minus") && names.contains("bore"));
}

#[test]
fn helical_parameters_out_of_range_are_refused() {
    let steep = helical(20, 46.0, 5.0, Hand::Right);
    assert_eq!(refusal(&steep).0, GearRefusal::InvalidParameter);
    let rack = GearParams {
        helix_angle: 0.3,
        ..GearParams::standard(GearKind::Rack, 1.0, 5, 2.0)
    };
    assert_eq!(refusal(&rack).0, GearRefusal::InvalidParameter);
    let negative = helical(20, -5.0, 5.0, Hand::Right);
    assert_eq!(refusal(&negative).0, GearRefusal::InvalidParameter);
}
