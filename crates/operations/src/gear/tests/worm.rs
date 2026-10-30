//! Worm oracles: ISO/TR 10828 closed forms, Cavalieri volumes, flank points
//! on the exact involute helicoid, the axial thread thickness and pitch read
//! off the body by classification, the hand, the matching wheel's crossed
//! helical form, face roles, and refusals.

use super::*;
use crate::gear::worm::equivalent;

fn worm(starts: u32, diameter_factor: f64, length: f64, hand: Hand) -> WormParams {
    WormParams {
        hand,
        ..WormParams::standard(2.0, starts, diameter_factor, length)
    }
}

fn build_worm(params: &WormParams, placement: &GearPlacement) -> (Topology, WormOutcome) {
    let mut topo = Topology::new();
    let outcome = make_worm(&mut topo, params, placement)
        .unwrap_or_else(|error| panic!("{params:?}: {error}"));
    (topo, outcome)
}

fn worm_refusal(params: &WormParams) -> GearRefusal {
    let mut topo = Topology::new();
    match make_worm(&mut topo, params, &GearPlacement::default()) {
        Err(GearError::Refused { code, .. }) => {
            assert_eq!(topo.num_faces(), 0, "a refusal must roll back");
            assert_eq!(topo.num_edges(), 0, "a refusal must roll back");
            code
        }
        other => panic!("expected a refusal, got {other:?}"),
    }
}

/// The exact transverse area of the worm (Green's theorem on the true
/// involute), less the bore.
fn exact_worm_area(params: &WormParams) -> f64 {
    let gear = equivalent(params);
    let helical = helical_geometry(&gear);
    let local = local_outline(&gear, &helical, 0.0).unwrap();
    let area = outline_area(
        &local.outline.segments,
        0.5 * helical.base_diameter.unwrap(),
    );
    area - local.bore_radius.map_or(0.0, |r| PI * r * r)
}

#[test]
fn worm_dimensions_match_closed_forms() {
    let params = WormParams {
        backlash: 0.1,
        ..worm(2, 10.0, 20.0, Hand::Right)
    };
    let g = worm_geometry(&params).unwrap();
    let alpha = 20.0_f64.to_radians();
    let gamma = 0.2_f64.atan();
    assert!((g.lead_angle - gamma).abs() < 1e-15);
    assert!((g.axial_module - 2.0).abs() < 1e-15);
    assert!((g.normal_module - 2.0 * gamma.cos()).abs() < 1e-15);
    // m_x·q/z₁: the helical equivalent's transverse module.
    assert!((g.transverse_module - 10.0).abs() < 1e-12);
    assert!((g.pitch_diameter - 20.0).abs() < 1e-12);
    assert!((g.tip_diameter - 24.0).abs() < 1e-12);
    assert!((g.root_diameter - 15.0).abs() < 1e-12);
    assert!((g.axial_pitch - 2.0 * PI).abs() < 1e-12);
    assert!((g.lead - 4.0 * PI).abs() < 1e-12);
    assert!((g.axial_thickness - (PI - 0.1)).abs() < 1e-12);
    // The involute helicoid's base cylinder: tan α_t = tan α_n / sin γ.
    let alpha_t = (alpha.tan() / gamma.sin()).atan();
    assert!((g.base_diameter - 20.0 * alpha_t.cos()).abs() < 1e-12);
    // Axial tip thickness: the transverse half-angle at the tip, turned
    // into axial length at 2/(z₁·m_x) radians per unit of advance.
    let inv = |phi: f64| phi.tan() - phi;
    let rb = 10.0 * alpha_t.cos();
    let transverse_backlash = 0.1 / gamma.tan();
    let psi0 = PI / 4.0 - transverse_backlash / 20.0 + inv(alpha_t);
    let tip_half = psi0 - inv((rb / 12.0).acos());
    assert!((g.axial_tip_thickness - tip_half * 2.0 * 2.0).abs() < 1e-12);
    assert!(g.warnings.is_empty(), "{:?}", g.warnings);
}

#[test]
fn worms_are_valid_and_keep_their_transverse_area() {
    let single = worm(1, 10.0, 20.0, Hand::Right);
    let bored = WormParams {
        bore_diameter: Some(8.0),
        ..worm(2, 9.0, 16.0, Hand::Left)
    };
    let sharp = WormParams {
        root_fillet: 0.0,
        ..worm(4, 12.0, 30.0, Hand::Right)
    };
    for params in [single, bored, sharp] {
        let (topo, outcome) = build_worm(&params, &GearPlacement::default());
        let tolerance = params.tolerance.unwrap_or(2e-5);
        assert!(outcome.spans_along_axis >= 1);
        assert!(outcome.flank_deviation_bound <= tolerance * 1.01);
        assert_valid_and_watertight(&topo, outcome.solid);
        let expected = exact_worm_area(&params) * params.length;
        let volume = solid_volume(&topo, outcome.solid, 1e-4).unwrap();
        assert!(
            ((volume - expected) / expected).abs() < 1e-6,
            "{} starts: volume {volume} vs exact {expected}",
            params.starts
        );
        // The thread turns several times along the worm, so its lands are
        // swept like its flanks; only a bore stays a cylinder.
        let (planes, cylinders, nurbs, other) = census(&topo, outcome.solid);
        let bore = usize::from(params.bore_diameter.is_some());
        assert_eq!((planes, cylinders, other), (2, bore, 0));
        assert!(nurbs >= 4 * params.starts as usize);
    }
}

#[test]
fn flank_points_lie_on_the_exact_involute_helicoid() {
    for (starts, hand) in [(1, Hand::Right), (3, Hand::Left)] {
        let params = worm(starts, 11.0, 12.0, hand);
        let placement = GearPlacement {
            phase: 0.4,
            ..GearPlacement::default()
        };
        let (topo, outcome) = build_worm(&params, &placement);
        let gear = equivalent(&params);
        let c = circular(&gear);
        let rate = twist(&gear, &helical_geometry(&gear)).unwrap().rate;
        let pitch = TAU / f64::from(starts);
        let mut checked = 0;
        for (face, role) in &outcome.face_roles {
            let GearFaceRole::Flank { tooth, side } = role.role else {
                continue;
            };
            let FaceSurface::Nurbs(surface) = topo.face(*face).unwrap().surface() else {
                panic!("a worm flank must be a swept surface");
            };
            let center = placement.phase + f64::from(tooth) * pitch;
            let sign = if side == ToothSide::Plus { 1.0 } else { -1.0 };
            let (u0, u1) = surface.domain_u();
            let (v0, v1) = surface.domain_v();
            for i in 0..=24 {
                for j in 0..=8 {
                    let u = u0 + (u1 - u0) * f64::from(i) / 24.0;
                    let v = v0 + (v1 - v0) * f64::from(j) / 8.0;
                    let q = surface.evaluate(u, v);
                    let back = -rate * q.z();
                    let (s, cs) = back.sin_cos();
                    let p = Point3::new(cs * q.x() - s * q.y(), s * q.x() + cs * q.y(), 0.0);
                    let rho = p.x().hypot(p.y());
                    let alpha = (c.base_radius / rho).min(1.0).acos();
                    let expected = center + sign * (c.psi0 - (alpha.tan() - alpha));
                    let offset = angle_between(p.y().atan2(p.x()), expected);
                    let deviation = (rho * offset * alpha.cos()).abs();
                    assert!(
                        deviation <= outcome.flank_deviation_bound + 1e-9,
                        "{starts} starts, {}: a flank point is off the helicoid",
                        role.name()
                    );
                    checked += 1;
                }
            }
        }
        assert_eq!(checked, 2 * starts as usize * 25 * 9);
    }
}

/// Where material starts and stops along the line parallel to the axis at
/// `radius` and polar angle zero, between `from` and `to`: each change of
/// side bisected forty times (below `1e-12` here).
fn material_runs(topo: &Topology, solid: SolidId, radius: f64, from: f64, to: f64) -> Vec<f64> {
    let inside = |z: f64| classify(topo, solid, radius, 0.0, z) == PointClassification::Inside;
    let steps = 400;
    let mut changes = Vec::new();
    let mut previous = inside(from);
    for i in 1..=steps {
        let z = from + (to - from) * f64::from(i) / f64::from(steps);
        let now = inside(z);
        if now != previous {
            let (mut lo, mut hi) = (z - (to - from) / f64::from(steps), z);
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                if inside(mid) == previous {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            changes.push((0.5 * (lo + hi), now));
        }
        previous = now;
    }
    // Keep whole runs only: drop a leading exit and a trailing entry.
    while changes.first().is_some_and(|change| !change.1) {
        changes.remove(0);
    }
    if changes.len() % 2 == 1 {
        changes.pop();
    }
    changes.into_iter().map(|change| change.0).collect()
}

#[test]
fn the_axial_section_shows_the_thread_thickness_and_pitch() {
    let params = WormParams {
        backlash: 0.2,
        ..worm(2, 10.0, 24.0, Hand::Right)
    };
    let (topo, outcome) = build_worm(&params, &GearPlacement::default());
    let g = &outcome.geometry;
    let runs = material_runs(&topo, outcome.solid, 0.5 * g.pitch_diameter, 1.0, 23.0);
    // Three whole threads cross the reference cylinder between 1 and 23.
    assert_eq!(runs.len(), 6, "material runs along the axial line");
    for pair in runs.chunks(2) {
        assert!(
            (pair[1] - pair[0] - g.axial_thickness).abs() < 1e-4,
            "a thread's axial thickness is off"
        );
    }
    // Two starts: neighbouring threads one axial pitch apart.
    for pairs in runs.chunks(2).collect::<Vec<_>>().windows(2) {
        let spacing = 0.5 * (pairs[1][0] + pairs[1][1] - pairs[0][0] - pairs[0][1]);
        assert!(
            (spacing - g.axial_pitch).abs() < 1e-4,
            "neighbouring threads are not one axial pitch apart"
        );
    }
}

#[test]
fn the_hand_sets_which_way_the_thread_turns() {
    for hand in [Hand::Right, Hand::Left] {
        let params = worm(1, 10.0, 10.0, hand);
        let (topo, outcome) = build_worm(&params, &GearPlacement::default());
        let gear = equivalent(&params);
        let rate = twist(&gear, &helical_geometry(&gear)).unwrap().rate;
        // One start of axial module 2 turns 1 radian per unit of advance.
        assert!((rate.abs() - 1.0).abs() < 1e-12);
        assert_eq!(rate > 0.0, hand == Hand::Right);
        let r = 0.5 * outcome.geometry.pitch_diameter + 0.5;
        let z = params.length - 1e-3;
        let turn = rate * z;
        assert_eq!(
            classify(&topo, outcome.solid, r, turn, z),
            PointClassification::Inside
        );
        assert_eq!(
            classify(&topo, outcome.solid, r, turn + PI, z),
            PointClassification::Outside
        );
    }
}

#[test]
fn the_matching_wheel_is_the_crossed_helical_partner() {
    let params = worm(2, 10.0, 20.0, Hand::Left);
    let wheel = worm_wheel(&params, 30, 12.0).unwrap();
    let gamma = 0.2_f64.atan();
    assert_eq!(wheel.params.hand, Hand::Left);
    assert_eq!(wheel.params.kind, GearKind::External);
    assert!((wheel.params.helix_angle - gamma).abs() < 1e-15);
    // Equal normal modules and pressure angles: equal normal base pitches.
    let worm_geometry = worm_geometry(&params).unwrap();
    assert!((wheel.params.module - worm_geometry.normal_module).abs() < 1e-15);
    assert!((wheel.params.pressure_angle - params.pressure_angle).abs() < 1e-15);
    let g = gear_geometry(&wheel.params).unwrap();
    // Transverse module m_x; addendum and dedendum lengths match the worm's.
    assert!((g.transverse_module - 2.0).abs() < 1e-12);
    assert!((g.pitch_diameter.unwrap() - 60.0).abs() < 1e-12);
    assert!((g.tip_diameter.unwrap() - 64.0).abs() < 1e-12);
    assert!((g.root_diameter.unwrap() - 55.0).abs() < 1e-12);
    assert!((wheel.center_distance - 40.0).abs() < 1e-12);
    // Worm tip to wheel root, and wheel tip to worm root: the same clearance.
    let clearance =
        wheel.center_distance - 0.5 * worm_geometry.tip_diameter - 0.5 * g.root_diameter.unwrap();
    let other =
        wheel.center_distance - 0.5 * g.tip_diameter.unwrap() - 0.5 * worm_geometry.root_diameter;
    assert!((clearance - 0.5).abs() < 1e-12 && (other - 0.5).abs() < 1e-12);
    let (topo, outcome) = build_gear(&wheel.params, &GearPlacement::default());
    assert_valid_and_watertight(&topo, outcome.solid);

    assert_eq!(
        worm_wheel(&params, 2, 12.0)
            .unwrap_err()
            .to_string()
            .split(':')
            .nth(1),
        Some(GearRefusal::InvalidParameter.code())
    );
}

#[test]
fn every_face_of_a_worm_has_one_unique_role() {
    let params = WormParams {
        bore_diameter: Some(6.0),
        ..worm(3, 12.0, 10.0, Hand::Right)
    };
    let (topo, outcome) = build_worm(&params, &GearPlacement::default());
    let faces = solid_faces(&topo, outcome.solid).unwrap();
    assert_eq!(outcome.face_roles.len(), faces.len());
    let names: std::collections::BTreeSet<_> =
        outcome.face_roles.iter().map(|(_, r)| r.name()).collect();
    assert_eq!(names.len(), faces.len());
    for name in [
        "tooth[2].flank.minus",
        "tooth[0].tip",
        "root[1]",
        "bore",
        "cap.end",
    ] {
        assert!(names.contains(name), "{name} missing from {names:?}");
    }
}

#[test]
fn worm_parameters_out_of_range_are_refused() {
    let base = worm(1, 10.0, 10.0, Hand::Right);
    let invalid = [
        WormParams { starts: 0, ..base },
        WormParams {
            starts: MAX_WORM_STARTS + 1,
            diameter_factor: 40.0,
            ..base
        },
        // A 45.7° lead angle.
        WormParams {
            starts: 4,
            diameter_factor: 3.9,
            ..base
        },
        // The root cylinder closes: q ≤ 2·h_f.
        WormParams {
            diameter_factor: 2.5,
            ..base
        },
        WormParams {
            backlash: PI,
            ..base
        },
        WormParams {
            pressure_angle: f64::NAN,
            ..base
        },
        WormParams {
            length: 0.0,
            ..base
        },
    ];
    for params in &invalid {
        assert_eq!(
            worm_refusal(params),
            GearRefusal::InvalidParameter,
            "{params:?}"
        );
    }
    let bored = WormParams {
        bore_diameter: Some(15.0),
        ..base
    };
    assert_eq!(worm_refusal(&bored), GearRefusal::BoreTooLarge);
    let tight = WormParams {
        tolerance: Some(1e-12),
        ..base
    };
    assert_eq!(worm_refusal(&tight), GearRefusal::ToleranceTooTight);
}
