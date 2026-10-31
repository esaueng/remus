//! Double-enveloping wheel oracles: Cavalieri volumes over exact
//! straight-sided sections, the exact straight tooth in the mid-plane,
//! clearance from the worm used as a hob, roles and refusals.

use super::*;

fn probe_worm() -> EnvelopingWormParams {
    EnvelopingWormParams::standard(2.0, 1, 30, 10.0, 24.0)
}

fn build(
    worm: &EnvelopingWormParams,
    options: &EnvelopingWheelOptions,
) -> (Topology, EnvelopingWheelOutcome) {
    let mut topo = Topology::new();
    let outcome = make_enveloping_worm_wheel(&mut topo, worm, options, &GearPlacement::default())
        .unwrap_or_else(|error| panic!("{options:?}: {error}"));
    (topo, outcome)
}

/// The straight flank line's polar angle about the wheel center at radius
/// `rho` in the mid-plane, tooth 0 facing +x: `±(π/2 − α + δ − acos(r_b/ρ))`,
/// `δ` half the tooth angle at the pitch circle less the wheel's backlash;
/// the worm's backlash never thickens the wheel.
fn line(worm: &EnvelopingWormParams, plus: bool, rho: f64, backlash: f64) -> f64 {
    let r2 = 0.5 * worm.module * f64::from(worm.wheel_teeth);
    let rb = r2 * worm.pressure_angle.sin();
    let half = (0.5 * PI).mul_add(worm.module, -backlash) / (2.0 * r2);
    let angle = half + 0.5 * PI - worm.pressure_angle - (rb / rho).acos();
    if plus { angle } else { -angle }
}

/// The minus flank's polar angle about the tooth center at radius `rho` and
/// height `u` from the mid-plane, before the wheel's backlash: the line
/// `x·n(ν(u)) = d(u)` with the outcome's laws, `ν(u) + acos(d(u)/ρ)`. The
/// plus flank is `−minus(ρ, −u)`.
fn minus(worm: &EnvelopingWormParams, outcome: &EnvelopingWheelOutcome, rho: f64, u: f64) -> f64 {
    let r2 = 0.5 * worm.module * f64::from(worm.wheel_teeth);
    let rb = r2 * worm.pressure_angle.sin();
    let half = 0.5 * PI * worm.module / (2.0 * r2);
    let side = usize::from(u > 0.0);
    let [a1, a2] = outcome.flank_turn[side];
    let [b1, b2] = outcome.flank_offset[side];
    let nu = worm.pressure_angle - half - 0.5 * PI + a1 * u + a2 * u * u;
    nu + ((rb + b1 * u + b2 * u * u) / rho).acos()
}

/// `∫∫ dA` of one transverse section at height `s`: the root disc less the
/// bore, plus every tooth between the root and the top radius, each
/// `∫ (θ₊ − θ₋) ρ dρ` with the flanks of the outcome's laws.
fn section_area(
    worm: &EnvelopingWormParams,
    options: &EnvelopingWheelOptions,
    outcome: &EnvelopingWheelOutcome,
    s: f64,
) -> f64 {
    let root = 0.5 * outcome.root_diameter;
    let u = s - 0.5 * options.face_width;
    let half = 0.5 * outcome.throat_width;
    let top = if u.abs() < half {
        outcome.center_distance - u.mul_add(-u, outcome.throat_radius.powi(2)).sqrt()
    } else {
        0.5 * outcome.outside_diameter
    };
    let thinning = options.backlash / (0.5 * outcome.pitch_diameter);
    let gauss = remus_math::quadrature::gauss_legendre_points(8);
    let mut tooth = 0.0;
    let pieces = 8;
    for piece in 0..pieces {
        let a = root + (top - root) * f64::from(piece) / f64::from(pieces);
        let b = a + (top - root) / f64::from(pieces);
        for point in gauss {
            let rho = 0.5 * (a + b) + 0.5 * (b - a) * point.x;
            let width = -minus(worm, outcome, rho, -u) - minus(worm, outcome, rho, u) - thinning;
            tooth += 0.5 * (b - a) * point.w * width * rho;
        }
    }
    let bore = options.bore_diameter.map_or(0.0, |d| 0.25 * PI * d * d);
    PI * root * root - bore + f64::from(worm.wheel_teeth) * tooth
}

fn section_volume(
    worm: &EnvelopingWormParams,
    options: &EnvelopingWheelOptions,
    outcome: &EnvelopingWheelOutcome,
) -> f64 {
    let b = options.face_width;
    let half = 0.5 * outcome.throat_width;
    let gauss = remus_math::quadrature::gauss_legendre_points(8);
    let mut volume = 0.0;
    for (lo, hi, pieces) in [
        (0.0, 0.5 * b - half, 8),
        (0.5 * b - half, 0.5 * b + half, 32),
        (0.5 * b + half, b, 8),
    ] {
        for piece in 0..pieces {
            let a = lo + (hi - lo) * f64::from(piece) / f64::from(pieces);
            let c = a + (hi - lo) / f64::from(pieces);
            for point in gauss {
                let s = 0.5 * (a + c) + 0.5 * (c - a) * point.x;
                volume += 0.5 * (c - a) * point.w * section_area(worm, options, outcome, s);
            }
        }
    }
    volume
}

#[test]
fn enveloping_wheels_are_valid_and_match_the_section_integral() {
    let worm = probe_worm();
    for options in [
        EnvelopingWheelOptions::new(8.0),
        EnvelopingWheelOptions {
            backlash: 0.05,
            bore_diameter: Some(12.0),
            outside_diameter: Some(64.4),
            ..EnvelopingWheelOptions::new(7.0)
        },
    ] {
        let (topo, outcome) = build(&worm, &options);
        let tolerance = DEFAULT_TOLERANCE_PER_MODULE * worm.module;
        assert!(outcome.flank_deviation_bound <= 2.0 * tolerance * 1.01);
        assert!(outcome.relief < 0.05 * worm.module, "{outcome:?}");
        assert_valid_and_watertight(&topo, outcome.solid);
        let expected = section_volume(&worm, &options, &outcome);
        let volume = solid_volume(&topo, outcome.solid, 1e-4).unwrap();
        assert!(
            ((volume - expected) / expected).abs() < 1e-5,
            "volume {volume} vs section integral {expected} ({options:?})"
        );
    }
}

#[test]
fn the_mid_plane_tooth_is_the_hindley_straight_tooth() {
    let worm = probe_worm();
    let options = EnvelopingWheelOptions::new(8.0);
    let (topo, outcome) = build(&worm, &options);
    for rho in [28.0, 30.0, 31.5] {
        for plus in [false, true] {
            let edge = line(&worm, plus, rho, 0.0);
            for (offset, expected) in [
                (-0.004, PointClassification::Inside),
                (0.004, PointClassification::Outside),
            ] {
                // Toward the tooth center is inside, away from it outside.
                let angle = if plus { edge + offset } else { edge - offset };
                let p = Point3::new(rho * angle.cos(), rho * angle.sin(), 4.0);
                let got =
                    crate::classify::classify_point(&topo, outcome.solid, p, 0.01, 1e-6).unwrap();
                assert_eq!(got, expected, "rho {rho} plus {plus} offset {offset}");
            }
        }
    }
}

#[test]
fn the_wheel_clears_the_worm_used_as_a_hob() {
    // Points just inside every flank across the face, and on the tips: the
    // hob never reaches them.
    let worm = probe_worm();
    let options = EnvelopingWheelOptions::new(8.0);
    let (_, outcome) = build(&worm, &options);
    let hob = crate::gear::hindley_wheel::HobSweep::new(&worm);
    let mut worst: f64 = f64::MIN;
    for i in 0..=16 {
        let s = -4.0 + 0.5 * f64::from(i);
        for rho in [27.5, 28.5, 29.5, 30.5, 31.5] {
            for plus in [false, true] {
                let theta = if plus {
                    PI - minus(&worm, &outcome, rho, -s) - 0.02 / rho
                } else {
                    PI + minus(&worm, &outcome, rho, s) + 0.02 / rho
                };
                let depth = hob.depth(Point3::new(s, rho * theta.cos(), rho * theta.sin()));
                worst = worst.max(depth);
            }
        }
    }
    assert!(worst <= 0.0, "the hob reaches {worst} into the wheel");
}

#[test]
fn the_worms_backlash_leaves_the_wheel_unchanged() {
    let options = EnvelopingWheelOptions::new(8.0);
    let (_, nominal) = build(&probe_worm(), &options);
    let thinned = EnvelopingWormParams {
        backlash: 0.1,
        ..probe_worm()
    };
    let (_, wheel) = build(&thinned, &options);
    let pairs = wheel.flank_turn.iter().chain(&wheel.flank_offset).flatten();
    let expected = nominal
        .flank_turn
        .iter()
        .chain(&nominal.flank_offset)
        .flatten();
    for (got, want) in pairs.zip(expected) {
        assert!((got - want).abs() < 1e-12, "{got} vs {want}");
    }
    assert!((wheel.throat_diameter - nominal.throat_diameter).abs() < 1e-12);
}

#[test]
fn every_face_of_an_enveloping_wheel_has_one_unique_role() {
    let worm = probe_worm();
    let options = EnvelopingWheelOptions {
        bore_diameter: Some(12.0),
        ..EnvelopingWheelOptions::new(8.0)
    };
    let (topo, outcome) = build(&worm, &options);
    let faces = solid_faces(&topo, outcome.solid).unwrap();
    assert_eq!(outcome.face_roles.len(), faces.len());
    let names: Vec<String> = outcome.face_roles.iter().map(|e| e.1.name()).collect();
    let unique: std::collections::BTreeSet<&String> = names.iter().collect();
    assert_eq!(unique.len(), names.len());
    for name in [
        "cap.start",
        "cap.end",
        "bore",
        "tooth[0].flank.plus.start-half",
        "tooth[29].flank.minus.end-half",
        "tooth[0].throat",
        "tooth[0].tip.start-half",
        "tooth[0].tip.end-half",
        "root[29]",
    ] {
        assert!(names.iter().any(|n| n == name), "{name} missing");
    }
}

#[test]
fn enveloping_wheels_out_of_range_are_refused() {
    let worm = probe_worm();
    let refusal = |options: EnvelopingWheelOptions| {
        let mut topo = Topology::new();
        match make_enveloping_worm_wheel(&mut topo, &worm, &options, &GearPlacement::default()) {
            Err(GearError::Refused { code, .. }) => {
                assert_eq!(topo.num_faces(), 0);
                code
            }
            other => panic!("expected a refusal, got {other:?}"),
        }
    };
    // The throat (7.75 wide) does not fit a 6 wide face.
    assert_eq!(
        refusal(EnvelopingWheelOptions::new(6.0)),
        GearRefusal::InvalidParameter
    );
    // Wider, the worm cuts into the outside tips near the faces.
    assert_eq!(
        refusal(EnvelopingWheelOptions::new(12.0)),
        GearRefusal::InvalidParameter
    );
    assert_eq!(
        refusal(EnvelopingWheelOptions {
            outside_diameter: Some(63.0),
            ..EnvelopingWheelOptions::new(8.0)
        }),
        GearRefusal::InvalidParameter
    );
    assert_eq!(
        refusal(EnvelopingWheelOptions {
            bore_diameter: Some(56.0),
            ..EnvelopingWheelOptions::new(8.0)
        }),
        GearRefusal::BoreTooLarge
    );
}
