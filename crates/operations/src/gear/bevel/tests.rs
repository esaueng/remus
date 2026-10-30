//! Straight bevel gear oracles: closed-form cone geometry and mate pitch
//! angles, the volume of the apex sweep against the exact heel area (Green's
//! theorem on the true spherical involute), flank points on the exact
//! spherical-involute cones, tooth and gap classification, face roles, and
//! refusals.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use super::*;
use crate::classify::{PointClassification, classify_point};
use crate::measure::solid_volume;
use crate::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use crate::validate::validate_solid;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};

fn build(params: &BevelGearParams, placement: &GearPlacement) -> (Topology, BevelGearOutcome) {
    let mut topo = Topology::new();
    let outcome =
        make_bevel_gear(&mut topo, params, placement).unwrap_or_else(|error| panic!("{error}"));
    (topo, outcome)
}

fn refusal(params: &BevelGearParams) -> GearRefusal {
    let mut topo = Topology::new();
    match make_bevel_gear(&mut topo, params, &GearPlacement::default()) {
        Err(GearError::Refused { code, .. }) => code,
        other => panic!("expected a refusal, got {other:?}"),
    }
}

fn assert_valid_and_watertight(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.2).unwrap();
    assert!(is_watertight(&mesh));
}

/// `[planes, cylinders, cones, nurbs]` over the solid's faces.
fn census(topo: &Topology, solid: SolidId) -> [usize; 4] {
    let mut counts = [0; 4];
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => counts[0] += 1,
            FaceSurface::Cylinder(_) => counts[1] += 1,
            FaceSurface::Cone(_) => counts[2] += 1,
            FaceSurface::Nurbs(_) => counts[3] += 1,
            other => panic!("unexpected surface {other:?}"),
        }
    }
    counts
}

/// The exact heel-plane area: per tooth the tip sector, both flanks
/// (`½∫ρ² dη` along the projected spherical involute, by Simpson's rule on
/// its closed form) and the root sector.
fn exact_heel_area(params: &BevelGearParams) -> f64 {
    let c = cones(params).unwrap();
    let h = c.apex_distance;
    let start = c.root.max(c.base);
    let (s0, s1) = (c.involute.roll_at(start), c.involute.roll_at(c.face));
    let half_area_rate = |phi: f64| {
        let (p, d) = c.involute.sample(phi);
        let (x, y) = (h * p.x() / p.z(), h * p.y() / p.z());
        let dx = h * (d.x() * p.z() - p.x() * d.z()) / (p.z() * p.z());
        let dy = h * (d.y() * p.z() - p.y() * d.z()) / (p.z() * p.z());
        0.5 * x.mul_add(dy, -y * dx)
    };
    let n = 20_000;
    let step = (s1 - s0) / f64::from(n);
    let mut flank = half_area_rate(s0) + half_area_rate(s1);
    for i in 1..n {
        let weight = if i % 2 == 1 { 4.0 } else { 2.0 };
        flank += weight * half_area_rate(step.mul_add(f64::from(i), s0));
    }
    flank *= step / 3.0;
    let tip_half = c.psi0 - c.involute.azimuth_at(c.face);
    let root_half = c.psi0 - c.involute.azimuth_at(start);
    let (r_tip, r_root) = (h * c.face.tan(), h * c.root.tan());
    let z = f64::from(params.teeth);
    z * (r_tip * r_tip).mul_add(
        tip_half,
        (r_root * r_root).mul_add(PI / z - root_half, 2.0 * flank),
    )
}

/// Net area of a planar face, by the shoelace sum over its outer and inner
/// wires sampled on the edge curves: independent of the face integrator.
/// The chord error falls as `1/n²`, so Richardson extrapolation over `n` and
/// `2n` samples per edge cancels its leading term.
fn sampled_face_area(topo: &Topology, face: FaceId) -> f64 {
    let shoelace = |n: u32| {
        let data = topo.face(face).unwrap();
        let mut signed = 0.0;
        for wire in std::iter::once(&data.outer_wire()).chain(data.inner_wires()) {
            for oriented in topo.wire(*wire).unwrap().edges() {
                let edge = topo.edge(oriented.edge()).unwrap();
                let p0 = topo.vertex(edge.start()).unwrap().point();
                let p1 = topo.vertex(edge.end()).unwrap().point();
                let (t0, t1) = if matches!(edge.curve(), EdgeCurve::Line) {
                    (0.0, 1.0)
                } else {
                    crate::tessellate::edge_param_span(topo, edge).unwrap()
                };
                let mut previous = p0;
                let mut part = 0.0;
                for i in 1..=n {
                    let t = (t1 - t0).mul_add(f64::from(i) / f64::from(n), t0);
                    let point = edge.curve().evaluate_with_endpoints(t, p0, p1);
                    part += 0.5 * previous.x().mul_add(point.y(), -point.x() * previous.y());
                    previous = point;
                }
                signed += if oriented.is_forward() { part } else { -part };
            }
        }
        signed.abs()
    };
    (4.0 * shoelace(400) - shoelace(200)) / 3.0
}

/// The two caps' sampled areas, heel (larger) first.
fn cap_areas(topo: &Topology, solid: SolidId) -> (f64, f64) {
    let mut caps: Vec<f64> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { normal, .. } => normal.z().abs() > 1.0 - 1e-12,
            _ => false,
        })
        .map(|face| sampled_face_area(topo, face))
        .collect();
    assert_eq!(caps.len(), 2);
    if caps[0] < caps[1] {
        caps.swap(0, 1);
    }
    (caps[0], caps[1])
}

/// Every section is the heel outline scaled by `t ∈ [s, 1]` at axial
/// distance `t·h` from the apex: `V = A·h·(1 − s³)/3`.
fn sweep_volume(area: f64, params: &BevelGearParams) -> f64 {
    let c = cones(params).unwrap();
    area * c.apex_distance * c.scale.mul_add(-c.scale * c.scale, 1.0) / 3.0
}

fn standard() -> BevelGearParams {
    BevelGearParams::standard(2.0, 20, 40, 8.0)
}

#[test]
fn cone_geometry_matches_the_closed_forms() {
    let g = bevel_gear_geometry(&standard()).unwrap();
    let delta = 0.5_f64.atan();
    let a = 20.0 / delta.sin();
    assert!((g.pitch_angle - delta).abs() < 1e-15);
    assert!((g.pitch_angle + g.mate_pitch_angle - FRAC_PI_2).abs() < 1e-15);
    assert!((g.outer_cone_distance - a).abs() < 1e-12);
    assert!((g.inner_cone_distance - (a - 8.0)).abs() < 1e-12);
    assert!((g.face_angle - (delta + (2.0 / a).atan())).abs() < 1e-15);
    assert!((g.root_angle - (delta - (2.5 / a).atan())).abs() < 1e-15);
    let base = (delta.sin() * 20.0_f64.to_radians().cos()).asin();
    assert!((g.base_cone_angle - base).abs() < 1e-15);
    assert!((g.apex_distance - a * delta.cos()).abs() < 1e-12);
    assert!((g.height - 8.0 * delta.cos()).abs() < 1e-12);
    assert!((g.pitch_diameter - 40.0).abs() < 1e-12);
    // The heel plane meets the face and root cones at h·tan.
    assert!((g.tip_diameter - 2.0 * g.apex_distance * g.face_angle.tan()).abs() < 1e-12);
    assert!((g.root_diameter - 2.0 * g.apex_distance * g.root_angle.tan()).abs() < 1e-12);
    assert!((g.virtual_teeth - 20.0 / delta.cos()).abs() < 1e-12);
    assert!((g.tooth_thickness - PI).abs() < 1e-12);

    // Mates share the shaft angle and the outer cone distance.
    for shaft in [FRAC_PI_2, 60.0_f64.to_radians(), 110.0_f64.to_radians()] {
        let pinion = BevelGearParams {
            shaft_angle: shaft,
            ..BevelGearParams::standard(2.0, 18, 27, 6.0)
        };
        let wheel = BevelGearParams {
            shaft_angle: shaft,
            ..BevelGearParams::standard(2.0, 27, 18, 6.0)
        };
        let (p, w) = (
            bevel_gear_geometry(&pinion).unwrap(),
            bevel_gear_geometry(&wheel).unwrap(),
        );
        assert!((p.pitch_angle + w.pitch_angle - shaft).abs() < 1e-14);
        assert!((p.mate_pitch_angle - w.pitch_angle).abs() < 1e-14);
        assert!((p.outer_cone_distance - w.outer_cone_distance).abs() < 1e-12);
    }
}

#[test]
fn bevel_gears_are_valid_and_sweep_their_exact_heel_area() {
    let fine = |params: BevelGearParams| BevelGearParams {
        tolerance: Some(1e-7 * params.module),
        ..params
    };
    let mut bored = fine(BevelGearParams::standard(1.5, 24, 24, 6.0));
    bored.bore_diameter = Some(8.0);
    let cases = [
        fine(standard()),
        fine(BevelGearParams::standard(2.0, 15, 45, 7.0)),
        fine(BevelGearParams {
            shaft_angle: 70.0_f64.to_radians(),
            ..BevelGearParams::standard(2.5, 16, 20, 7.0)
        }),
        bored,
    ];
    for params in cases {
        let (topo, outcome) = build(&params, &GearPlacement::default());
        assert!(outcome.spans_per_flank >= 1);
        assert_valid_and_watertight(&topo, outcome.solid);
        let c = cones(&params).unwrap();
        let bore_area = params
            .bore_diameter
            .map_or(0.0, |bore| PI * 0.25 * bore * bore);
        // The built caps against the exact heel outline and its toe image.
        // The messages name the check, not the measured areas: CodeQL reads
        // the trim certifier (`certify_curve_authority`) feeding them as a
        // credential and flags printing them.
        let exact = exact_heel_area(&params);
        let (heel, toe) = cap_areas(&topo, outcome.solid);
        let toe_exact = exact * c.scale * c.scale - bore_area;
        assert!(
            ((heel - (exact - bore_area)) / exact).abs() < 1e-8,
            "z {}: heel cap off the exact outline",
            params.teeth
        );
        assert!(
            ((toe - toe_exact) / toe_exact).abs() < 1e-8,
            "z {}: toe cap off the exact outline",
            params.teeth
        );
        let expected = bore_area.mul_add(-outcome.geometry.height, sweep_volume(exact, &params));
        // Divergence theorem about the apex: every side face but the bore
        // is a cone through it and adds nothing; the bore wall, at `x·n = −r`
        // over `2πr·height`, adds `−⅔·bore_area·height`.
        let h = c.apex_distance;
        let from_caps = (c.scale * h).mul_add(-toe, h * heel) / 3.0
            - 2.0 / 3.0 * bore_area * outcome.geometry.height;
        assert!(
            ((from_caps - expected) / expected).abs() < 1e-8,
            "z {}: cap divergence off the exact volume",
            params.teeth
        );
        // The volume path reads the rational ruled flanks to about 1e-5
        // relative, independent of the deflection. On the bored gear its
        // boundary check misses the flank projection and it falls back to a
        // fine mesh, so only the caps above measure that case.
        if params.bore_diameter.is_none() {
            let volume = solid_volume(&topo, outcome.solid, 1e-5).unwrap();
            assert!(
                ((volume - expected) / expected).abs() < 5e-5,
                "z {}: volume {volume} vs exact {expected}",
                params.teeth
            );
        }
        let z = params.teeth as usize;
        let radial = if c.root < c.base { 2 * z } else { 0 };
        let bore = usize::from(params.bore_diameter.is_some());
        assert_eq!(
            census(&topo, outcome.solid),
            [2 + radial, bore, 2 * z, 2 * z],
            "z {}",
            params.teeth
        );
    }
}

/// Signed difference of two angles, wrapped into `(−π, π]`.
fn angle_between(a: f64, b: f64) -> f64 {
    (a - b + PI).rem_euclid(TAU) - PI
}

#[test]
fn flanks_lie_on_the_exact_spherical_involute_cones() {
    let params = BevelGearParams::standard(2.0, 21, 33, 9.0);
    let placement = GearPlacement {
        phase: 0.15,
        ..GearPlacement::default()
    };
    let (topo, outcome) = build(&params, &placement);
    let c = cones(&params).unwrap();
    let apex = Point3::new(0.0, 0.0, c.apex_distance);
    let pitch = TAU / f64::from(params.teeth);
    let mut checked = 0;
    for (face, named) in &outcome.face_roles {
        let GearFaceRole::Flank { tooth, side } = named.role else {
            continue;
        };
        let FaceSurface::Nurbs(surface) = topo.face(*face).unwrap().surface() else {
            panic!("a bevel flank is a ruled NURBS");
        };
        let center = placement.phase + f64::from(tooth) * pitch;
        let sign: f64 = if side == ToothSide::Plus { 1.0 } else { -1.0 };
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        for i in 0..=6 {
            for j in 0..=12 {
                let u = u0 + (u1 - u0) * f64::from(i) / 6.0;
                let v = v0 + (v1 - v0) * f64::from(j) / 12.0;
                let d = surface.evaluate(u, v) - apex;
                // On the sphere about the apex, polar angle from the axis
                // toward the heel.
                let distance = d.length();
                let polar = (-d.z() / distance).acos();
                let expected = sign.mul_add(c.psi0 - c.involute.azimuth_at(polar), center);
                let offset = angle_between(d.y().atan2(d.x()), expected);
                // Along the small circle; the distance to the flank is at
                // most this, and at least half of it for any flank angle
                // under 60°.
                let deviation = (distance * polar.sin() * offset).abs();
                assert!(
                    deviation <= 2.0 * outcome.flank_deviation_bound + 1e-9,
                    "{}: off the exact spherical-involute cone",
                    named.name()
                );
                checked += 1;
            }
        }
    }
    assert_eq!(checked, 2 * 21 * 7 * 13);
    assert!(outcome.flank_deviation_bound <= 2e-5 * 1.01);
}

#[test]
fn teeth_and_gaps_classify_on_the_pitch_cone() {
    let params = standard();
    let placement = GearPlacement {
        phase: 0.3,
        ..GearPlacement::default()
    };
    let (topo, outcome) = build(&params, &placement);
    let g = &outcome.geometry;
    let half_pitch = PI / f64::from(params.teeth);
    // Mid-face on the pitch cone, then near the heel and the toe.
    for distance in [
        g.outer_cone_distance - 0.5 * params.face_width,
        g.outer_cone_distance - 0.1,
        g.inner_cone_distance + 0.1,
    ] {
        let at = |azimuth: f64| {
            let (s, c) = azimuth.sin_cos();
            let radius = distance * g.pitch_angle.sin();
            Point3::new(
                radius * c,
                radius * s,
                distance.mul_add(-g.pitch_angle.cos(), g.apex_distance),
            )
        };
        for k in [0.0_f64, 7.0] {
            let tooth = (2.0 * k).mul_add(half_pitch, placement.phase);
            assert_eq!(
                classify_point(&topo, outcome.solid, at(tooth), 0.01, 1e-6).unwrap(),
                PointClassification::Inside,
                "tooth {k} at {distance}"
            );
            assert_eq!(
                classify_point(&topo, outcome.solid, at(tooth + half_pitch), 0.01, 1e-6).unwrap(),
                PointClassification::Outside,
                "gap after tooth {k} at {distance}"
            );
        }
    }
}

#[test]
fn every_face_keeps_one_unique_role() {
    let mut params = BevelGearParams::standard(2.0, 17, 30, 6.0);
    params.bore_diameter = Some(6.0);
    let (topo, outcome) = build(&params, &GearPlacement::default());
    let faces = solid_faces(&topo, outcome.solid).unwrap();
    assert_eq!(outcome.face_roles.len(), faces.len());
    let names: std::collections::BTreeSet<_> =
        outcome.face_roles.iter().map(|(_, r)| r.name()).collect();
    assert_eq!(names.len(), faces.len());
    for name in [
        "cap.start",
        "cap.end",
        "bore",
        "tooth[16].flank.minus",
        "tooth[0].tip",
        "root[16]",
    ] {
        assert!(names.contains(name), "{name}");
    }
}

#[test]
fn out_of_range_bevel_gears_are_refused() {
    let base = standard();
    let invalid = [
        BevelGearParams {
            shaft_angle: PI,
            ..base
        },
        BevelGearParams {
            shaft_angle: 0.0,
            ..base
        },
        // The face width reaches the apex.
        BevelGearParams {
            face_width: 50.0,
            ..base
        },
        // A pitch angle near 90°: toward a crown gear.
        BevelGearParams::standard(2.0, 60, 4, 4.0),
        BevelGearParams { teeth: 3, ..base },
        BevelGearParams {
            pressure_angle: 0.0,
            ..base
        },
    ];
    for params in invalid {
        assert_eq!(
            refusal(&params),
            GearRefusal::InvalidParameter,
            "{params:?}"
        );
    }
    let bore = BevelGearParams {
        bore_diameter: Some(30.0),
        ..base
    };
    assert_eq!(refusal(&bore), GearRefusal::BoreTooLarge);
    let tight = BevelGearParams {
        tolerance: Some(1e-30),
        ..base
    };
    assert_eq!(refusal(&tight), GearRefusal::ToleranceTooTight);
}
