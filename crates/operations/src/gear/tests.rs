#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use crate::classify::{PointClassification, classify_point};
use crate::measure::solid_volume;
use crate::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use crate::validate::validate_solid;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;

fn spur(teeth: u32) -> GearParams {
    GearParams::standard(GearKind::External, 2.0, teeth, 5.0)
}

fn build_gear(params: &GearParams, placement: &GearPlacement) -> (Topology, GearOutcome) {
    let mut topo = Topology::new();
    let outcome = make_gear(&mut topo, params, placement)
        .unwrap_or_else(|error| panic!("{params:?}: {error}"));
    (topo, outcome)
}

fn refusal(params: &GearParams) -> (GearRefusal, Option<Box<GearGeometry>>) {
    let mut topo = Topology::new();
    match make_gear(&mut topo, params, &GearPlacement::default()) {
        Err(GearError::Refused { code, geometry, .. }) => {
            assert_eq!(topo.num_faces(), 0, "a refusal must roll back");
            assert_eq!(topo.num_edges(), 0, "a refusal must roll back");
            (code, geometry)
        }
        other => panic!("expected a refusal, got {other:?}"),
    }
}

/// `(planes, cylinders, nurbs, other)` over every face of the solid.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => counts.0 += 1,
            FaceSurface::Cylinder(_) => counts.1 += 1,
            FaceSurface::Nurbs(_) => counts.2 += 1,
            _ => counts.3 += 1,
        }
    }
    counts
}

/// Signed short angle from `a` to `b` about `center`.
fn arc_sweep(center: Point3, a: Point3, b: Point3) -> f64 {
    let (ax, ay) = (a.x() - center.x(), a.y() - center.y());
    let (bx, by) = (b.x() - center.x(), b.y() - center.y());
    (ax * by - ay * bx).atan2(ax * bx + ay * by)
}

/// Enclosed area of a local outline by Green's theorem, `½∮(x dy − y dx)`.
///
/// Flanks use the exact involute, not their B-spline: along
/// `P(t) = r_b(cos t + t sin t, sin t − t cos t)` the integrand is
/// `x y' − y x' = r_b² t²`, and rotations about the origin keep it, so a
/// flank traversed over `[t0, t1]` contributes `± r_b² (t1³ − t0³)/6`. The
/// sign (mirror image, walking direction) is read from a coarse quadrature
/// of the B-spline itself.
fn outline_area(segments: &[Segment], base_radius: f64) -> f64 {
    let mut twice_area = 0.0;
    for segment in segments {
        match segment {
            Segment::Line { start, end } => {
                twice_area += start.x() * end.y() - end.x() * start.y();
            }
            Segment::Arc { center, start, end } => {
                let r = (*start - *center).length();
                let theta0 = (start.y() - center.y()).atan2(start.x() - center.x());
                let sweep = arc_sweep(*center, *start, *end);
                let theta1 = theta0 + sweep;
                twice_area += r * r * sweep
                    + r * (center.x() * (theta1.sin() - theta0.sin())
                        - center.y() * (theta1.cos() - theta0.cos()));
            }
            Segment::Flank { curve, reversed } => {
                let (t0, t1) = curve.domain();
                let mut coarse = 0.0;
                for i in 0..64 {
                    let a = curve.evaluate(t0 + (t1 - t0) * f64::from(i) / 64.0);
                    let b = curve.evaluate(t0 + (t1 - t0) * f64::from(i + 1) / 64.0);
                    coarse += a.x() * b.y() - b.x() * a.y();
                }
                let exact = base_radius * base_radius * (t1.powi(3) - t0.powi(3)) / 3.0;
                let walked = if *reversed { -coarse } else { coarse };
                twice_area += walked.signum() * exact;
            }
        }
    }
    0.5 * twice_area
}

fn exact_profile_area(params: &GearParams) -> f64 {
    let geometry = gear_geometry(params).unwrap();
    let local = local_outline(params, &geometry, 0.0).unwrap();
    let base_radius = geometry.base_diameter.map_or(1.0, |d| 0.5 * d);
    let area = outline_area(&local.outline.segments, base_radius);
    let bore = local.bore_radius.map_or(0.0, |r| PI * r * r);
    match local.rim_radius {
        Some(rim) => PI * rim * rim - area,
        None => area - bore,
    }
}

fn assert_volume(topo: &Topology, outcome: &GearOutcome, params: &GearParams, rel: f64) {
    let expected = exact_profile_area(params) * params.face_width;
    let volume = solid_volume(topo, outcome.solid, 1e-2).unwrap();
    assert!(
        ((volume - expected) / expected).abs() < rel,
        "volume {volume} vs exact {expected} ({params:?})"
    );
}

fn assert_valid_and_watertight(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.2).unwrap();
    assert!(is_watertight(&mesh));
}

fn classify(
    topo: &Topology,
    solid: SolidId,
    radius: f64,
    angle: f64,
    z: f64,
) -> PointClassification {
    let (sin, cos) = angle.sin_cos();
    classify_point(
        topo,
        solid,
        Point3::new(radius * cos, radius * sin, z),
        0.01,
        1e-6,
    )
    .unwrap()
}

#[test]
fn standard_dimensions_match_closed_forms() {
    let g = gear_geometry(&spur(20)).unwrap();
    let alpha = 20.0_f64.to_radians();
    assert!((g.pitch_diameter.unwrap() - 40.0).abs() < 1e-12);
    assert!((g.base_diameter.unwrap() - 40.0 * alpha.cos()).abs() < 1e-12);
    assert!((g.tip_diameter.unwrap() - 44.0).abs() < 1e-12);
    assert!((g.root_diameter.unwrap() - 35.0).abs() < 1e-12);
    assert!((g.circular_pitch - 2.0 * PI).abs() < 1e-12);
    assert!((g.tooth_thickness - PI).abs() < 1e-12);
    assert!((g.min_teeth_without_undercut.unwrap() - 2.0 / alpha.sin().powi(2)).abs() < 1e-12);
    assert!(g.warnings.is_empty(), "{:?}", g.warnings);

    let mut shifted = spur(20);
    shifted.profile_shift = 0.3;
    shifted.backlash = 0.1;
    let g = gear_geometry(&shifted).unwrap();
    assert!((g.tip_diameter.unwrap() - 45.2).abs() < 1e-12);
    assert!((g.root_diameter.unwrap() - 36.2).abs() < 1e-12);
    let expected = 2.0 * (PI / 2.0 + 0.6 * alpha.tan()) - 0.1;
    assert!((g.tooth_thickness - expected).abs() < 1e-12);
}

#[test]
fn small_gears_warn_undercut_with_the_avoiding_shift() {
    let g = gear_geometry(&spur(10)).unwrap();
    let sin2 = 20.0_f64.to_radians().sin().powi(2);
    let expected = 1.0 - 5.0 * sin2;
    assert!(g.warnings.iter().any(|w| matches!(
        w,
        GearWarning::Undercut { min_profile_shift } if (min_profile_shift - expected).abs() < 1e-12
    )));
    let mut corrected = spur(10);
    corrected.profile_shift = expected;
    assert!(
        !gear_geometry(&corrected)
            .unwrap()
            .warnings
            .iter()
            .any(|w| w.code() == "undercut")
    );
}

#[test]
fn external_gears_are_valid_exact_extrusions_across_tooth_counts() {
    for teeth in [6, 17, 41, 42, 60] {
        let params = spur(teeth);
        let (topo, outcome) = build_gear(&params, &GearPlacement::default());
        assert_valid_and_watertight(&topo, outcome.solid);
        let (planes, cylinders, nurbs, other) = census(&topo, outcome.solid);
        let z = teeth as usize;
        assert_eq!(nurbs, 2 * z, "teeth {teeth}: two involute flanks per tooth");
        // Tip, root land and two root fillets per tooth.
        assert_eq!(cylinders, 4 * z, "teeth {teeth}");
        assert_eq!(other, 0);
        // Caps, plus radial flank pieces below the base circle when the
        // involute starts above the fillet.
        assert!(
            planes == 2 || planes == 2 + 2 * z,
            "teeth {teeth}: {planes} planes"
        );
        assert_volume(&topo, &outcome, &params, 1e-5);
        assert!(outcome.flank_deviation_bound <= params.module * DEFAULT_TOLERANCE_PER_MODULE);
    }
}

#[test]
fn pitch_circle_tooth_thickness_matches_the_standard_formula() {
    // A point just inside the flank at the pitch circle is material, just
    // outside it is air: the involute sits where s = m(π/2 + 2x tan α) − j
    // puts it.
    for (shift, backlash) in [(0.0, 0.0), (0.4, 0.05), (-0.2, 0.0)] {
        let mut params = spur(24);
        params.profile_shift = shift;
        params.backlash = backlash;
        let (topo, outcome) = build_gear(&params, &GearPlacement::default());
        let g = &outcome.geometry;
        let r = 0.5 * g.pitch_diameter.unwrap();
        let half = g.tooth_thickness / (2.0 * r);
        let z = 0.5 * params.face_width;
        for side in [-1.0, 1.0] {
            assert_eq!(
                classify(&topo, outcome.solid, r, side * (half - 1e-4), z),
                PointClassification::Inside
            );
            assert_eq!(
                classify(&topo, outcome.solid, r, side * (half + 1e-4), z),
                PointClassification::Outside
            );
        }
        let ra = 0.5 * g.tip_diameter.unwrap();
        let rf = 0.5 * g.root_diameter.unwrap();
        let space = PI / f64::from(params.teeth);
        assert_eq!(
            classify(&topo, outcome.solid, ra - 1e-4, 0.0, z),
            PointClassification::Inside
        );
        assert_eq!(
            classify(&topo, outcome.solid, ra + 1e-4, 0.0, z),
            PointClassification::Outside
        );
        assert_eq!(
            classify(&topo, outcome.solid, rf - 1e-4, space, z),
            PointClassification::Inside
        );
        assert_eq!(
            classify(&topo, outcome.solid, rf + 1e-4, space, z),
            PointClassification::Outside
        );
    }
}

#[test]
fn flank_fit_meets_its_proven_bound_against_the_true_involute() {
    let params = spur(30);
    let geometry = gear_geometry(&params).unwrap();
    let local = local_outline(&params, &geometry, 0.0).unwrap();
    let flank = local.outline.flank.as_ref().unwrap();
    let c = circular(&params);
    let (s0, c0) = c.psi0.sin_cos();
    let (t0, t1) = flank.curve.domain();
    let mut worst = 0.0_f64;
    for i in 0..=4000 {
        let t = t0 + (t1 - t0) * f64::from(i) / 4000.0;
        let (s, cs) = t.sin_cos();
        let x = c.base_radius * (cs + t * s);
        let y = -c.base_radius * (s - t * cs);
        let truth = Point3::new(c0 * x - s0 * y, s0 * x + c0 * y, 0.0);
        worst = worst.max((flank.curve.evaluate(t) - truth).length());
    }
    assert!(
        worst <= flank.remainder_bound,
        "{worst:e} > {:e}",
        flank.remainder_bound
    );
    assert!(flank.remainder_bound <= DEFAULT_TOLERANCE_PER_MODULE * params.module);

    // Every tooth's flank is the first one rotated, control point by control
    // point.
    let flanks: Vec<_> = local
        .outline
        .segments
        .iter()
        .filter_map(|s| match s {
            Segment::Flank {
                curve,
                reversed: true,
            } => Some(curve),
            _ => None,
        })
        .collect();
    assert_eq!(flanks.len(), 30);
    for (k, curve) in flanks.iter().enumerate() {
        let (sin, cos) = (2.0 * PI * k as f64 / 30.0).sin_cos();
        for (p, q) in flank
            .curve
            .control_points()
            .iter()
            .zip(curve.control_points())
        {
            let rotated = Point3::new(cos * p.x() - sin * p.y(), sin * p.x() + cos * p.y(), 0.0);
            assert!((rotated - *q).length() < 1e-12);
        }
    }
}

#[test]
fn tighter_tolerance_adds_spans_and_shrinks_the_bound() {
    let mut fine = spur(20);
    fine.tolerance = Some(1e-9);
    let (_, coarse_outcome) = build_gear(&spur(20), &GearPlacement::default());
    let (_, fine_outcome) = build_gear(&fine, &GearPlacement::default());
    assert!(fine_outcome.spans_per_flank > coarse_outcome.spans_per_flank);
    assert!(fine_outcome.flank_deviation_bound < coarse_outcome.flank_deviation_bound);
    assert!(fine_outcome.flank_deviation_bound <= 1e-9 + 1e-12);
}

#[test]
fn sharp_root_and_bore_build_and_measure() {
    let mut params = spur(32);
    params.root_fillet = 0.0;
    params.bore_diameter = Some(20.0);
    let (topo, outcome) = build_gear(&params, &GearPlacement::default());
    assert_valid_and_watertight(&topo, outcome.solid);
    let (_, cylinders, nurbs, _) = census(&topo, outcome.solid);
    assert_eq!(nurbs, 64);
    // Tip and root lands plus the bore.
    assert_eq!(cylinders, 2 * 32 + 1);
    assert_volume(&topo, &outcome, &params, 1e-5);
    let z = 0.5 * params.face_width;
    assert_eq!(
        classify(&topo, outcome.solid, 5.0, 0.3, z),
        PointClassification::Outside
    );
    assert_eq!(
        classify(&topo, outcome.solid, 15.0, 0.3, z),
        PointClassification::Inside
    );
}

#[test]
fn internal_gear_is_a_valid_ring_with_teeth_on_the_inside() {
    let mut params = GearParams::standard(GearKind::Internal, 1.5, 48, 6.0);
    params.rim_diameter = Some(90.0);
    let (topo, outcome) = build_gear(&params, &GearPlacement::default());
    assert_valid_and_watertight(&topo, outcome.solid);
    let (planes, cylinders, nurbs, other) = census(&topo, outcome.solid);
    assert_eq!(nurbs, 96);
    // Tip and root lands, two fillets per space, and the rim.
    assert_eq!(cylinders, 4 * 48 + 1);
    assert_eq!((planes, other), (2, 0));
    assert_volume(&topo, &outcome, &params, 1e-5);

    let g = &outcome.geometry;
    let r_tip = 0.5 * g.tip_diameter.unwrap();
    let r_root = 0.5 * g.root_diameter.unwrap();
    assert!((r_tip - (36.0 - 1.5)).abs() < 1e-12);
    assert!((r_root - (36.0 + 1.875)).abs() < 1e-12);
    let space = PI / 48.0;
    let z = 3.0;
    // A tooth sits on the phase direction; a space half a pitch away.
    assert_eq!(
        classify(&topo, outcome.solid, r_tip + 1e-4, 0.0, z),
        PointClassification::Inside
    );
    assert_eq!(
        classify(&topo, outcome.solid, r_tip - 1e-4, 0.0, z),
        PointClassification::Outside
    );
    assert_eq!(
        classify(&topo, outcome.solid, r_root - 1e-4, space, z),
        PointClassification::Outside
    );
    assert_eq!(
        classify(&topo, outcome.solid, r_root + 1e-4, space, z),
        PointClassification::Inside
    );
    // The internal tooth is as thick as the pitch minus an external tooth.
    let r = 0.5 * g.pitch_diameter.unwrap();
    let half = g.tooth_thickness / (2.0 * r);
    assert_eq!(
        classify(&topo, outcome.solid, r, half - 1e-4, z),
        PointClassification::Inside
    );
    assert_eq!(
        classify(&topo, outcome.solid, r, half + 1e-4, z),
        PointClassification::Outside
    );
}

#[test]
fn rack_volume_matches_the_trapezoid_closed_form_exactly() {
    let mut params = GearParams::standard(GearKind::Rack, 2.0, 7, 4.0);
    params.root_fillet = 0.0;
    params.back_depth = Some(3.0);
    let (topo, outcome) = build_gear(&params, &GearPlacement::default());
    assert_valid_and_watertight(&topo, outcome.solid);
    let (planes, cylinders, nurbs, other) = census(&topo, outcome.solid);
    assert_eq!((cylinders, nurbs, other), (0, 0, 0));
    // Caps, back, two ends, flanks and tip per tooth, z − 1 root lands
    // and the two half lands at the ends.
    assert_eq!(planes, 2 + 3 + 3 * 7 + (7 + 1));
    assert!(outcome.flank_deviation_bound.abs() < f64::MIN_POSITIVE);

    // Independent closed form: back strip plus one trapezoid per tooth.
    let (m, tan_a) = (2.0, 20.0_f64.to_radians().tan());
    let pitch = PI * m;
    let s = pitch / 2.0;
    let root_width = s + 2.0 * 1.25 * m * tan_a;
    let tip_width = s - 2.0 * m * tan_a;
    let area = 7.0 * pitch * 3.0 + 7.0 * 0.5 * (root_width + tip_width) * 2.25 * m;
    let volume = solid_volume(&topo, outcome.solid, 1e-2).unwrap();
    assert!(
        ((volume - area * 4.0) / (area * 4.0)).abs() < 1e-9,
        "{volume} vs {}",
        area * 4.0
    );

    let mut filleted = GearParams::standard(GearKind::Rack, 2.0, 7, 4.0);
    filleted.back_depth = Some(3.0);
    let (topo, outcome) = build_gear(&filleted, &GearPlacement::default());
    assert_valid_and_watertight(&topo, outcome.solid);
    assert_eq!(census(&topo, outcome.solid).1, 14);
    assert_volume(&topo, &outcome, &filleted, 1e-9);
}

#[test]
fn rigid_placement_keeps_volume_and_phase_turns_the_teeth() {
    let params = spur(18);
    let (base_topo, base) = build_gear(&params, &GearPlacement::default());
    let base_volume = solid_volume(&base_topo, base.solid, 1e-2).unwrap();

    let placement = GearPlacement {
        origin: Point3::new(12.0, -7.0, 3.5),
        axis: Vec3::new(1.0, 2.0, -0.5),
        x_direction: Vec3::new(0.0, 0.0, 1.0),
        phase: 0.0,
    };
    let (topo, moved) = build_gear(&params, &placement);
    assert_valid_and_watertight(&topo, moved.solid);
    let volume = solid_volume(&topo, moved.solid, 1e-2).unwrap();
    assert!(((volume - base_volume) / base_volume).abs() < 1e-6);

    // Half a pitch of phase puts a tooth space where the first tooth was.
    let ra = 0.5 * base.geometry.tip_diameter.unwrap() - 0.05;
    let turned = GearPlacement {
        phase: PI / 18.0,
        ..GearPlacement::default()
    };
    let (topo, outcome) = build_gear(&params, &turned);
    assert_eq!(
        classify(&topo, outcome.solid, ra, 0.0, 2.5),
        PointClassification::Outside
    );
    assert_eq!(
        classify(&topo, outcome.solid, ra, PI / 18.0, 2.5),
        PointClassification::Inside
    );
}

#[test]
fn unbuildable_profiles_refuse_atomically_with_dimensions() {
    let mut pointed = spur(8);
    pointed.profile_shift = 1.2;
    assert_eq!(refusal(&pointed).0, GearRefusal::PointedTip);

    let mut huge_fillet = spur(12);
    huge_fillet.root_fillet = 1.5;
    let (code, geometry) = refusal(&huge_fillet);
    assert!(
        matches!(
            code,
            GearRefusal::RootLandClosed | GearRefusal::FlankTooShort
        ),
        "{code:?}"
    );
    assert!(
        geometry.is_some(),
        "dimensions travel with a profile refusal"
    );

    let mut below_base = spur(20);
    below_base.profile_shift = -1.7;
    assert_eq!(refusal(&below_base).0, GearRefusal::TipBelowBaseCircle);

    let mut bore = spur(20);
    bore.bore_diameter = Some(35.0);
    assert_eq!(refusal(&bore).0, GearRefusal::BoreTooLarge);

    let mut rim = GearParams::standard(GearKind::Internal, 1.0, 40, 3.0);
    rim.rim_diameter = Some(42.5);
    assert_eq!(refusal(&rim).0, GearRefusal::RimTooSmall);

    let mut tight = spur(20);
    tight.tolerance = Some(1e-15);
    assert_eq!(refusal(&tight).0, GearRefusal::ToleranceTooTight);
}

#[test]
fn invalid_parameters_refuse_without_dimensions() {
    let cases: &[fn(&mut GearParams)] = &[
        |p| p.module = 0.0,
        |p| p.module = f64::NAN,
        |p| p.teeth = 2,
        |p| p.teeth = MAX_TEETH + 1,
        |p| p.pressure_angle = 0.0,
        |p| p.pressure_angle = 50.0_f64.to_radians(),
        |p| p.face_width = -1.0,
        |p| p.backlash = -0.1,
        |p| p.addendum = 0.0,
        |p| p.dedendum = f64::INFINITY,
        |p| p.root_fillet = -0.1,
        |p| p.tolerance = Some(0.0),
        |p| p.rim_diameter = Some(100.0),
        |p| p.back_depth = Some(1.0),
    ];
    for mutate in cases {
        let mut params = spur(20);
        mutate(&mut params);
        let (code, geometry) = refusal(&params);
        assert_eq!(code, GearRefusal::InvalidParameter, "{params:?}");
        assert!(geometry.is_none());
    }
    let mut rack_shift = GearParams::standard(GearKind::Rack, 1.0, 5, 2.0);
    rack_shift.profile_shift = 0.2;
    assert_eq!(refusal(&rack_shift).0, GearRefusal::InvalidParameter);
    let internal = GearParams::standard(GearKind::Internal, 1.0, 40, 3.0);
    assert_eq!(refusal(&internal).0, GearRefusal::InvalidParameter);

    let mut topo = Topology::new();
    let rack = GearParams::standard(GearKind::Rack, 1.0, 5, 2.0);
    let placement = GearPlacement {
        phase: 0.1,
        ..GearPlacement::default()
    };
    assert!(matches!(
        make_gear(&mut topo, &rack, &placement),
        Err(GearError::Refused {
            code: GearRefusal::InvalidParameter,
            ..
        })
    ));
    let parallel = GearPlacement {
        x_direction: Vec3::new(0.0, 0.0, 2.0),
        ..GearPlacement::default()
    };
    assert!(make_gear(&mut topo, &spur(20), &parallel).is_err());
    assert_eq!(topo.num_faces(), 0);
}

#[test]
fn construction_is_deterministic() {
    let params = spur(25);
    let (a, first) = build_gear(&params, &GearPlacement::default());
    let (b, second) = build_gear(&params, &GearPlacement::default());
    assert_eq!(a.num_faces(), b.num_faces());
    assert_eq!(a.num_edges(), b.num_edges());
    assert_eq!(first, second);
    let va = solid_volume(&a, first.solid, 1e-2).unwrap();
    let vb = solid_volume(&b, second.solid, 1e-2).unwrap();
    assert_eq!(va.to_bits(), vb.to_bits());
}

/// Polar angle of the mean of a face's boundary vertex directions.
fn face_angle(topo: &Topology, face: FaceId) -> f64 {
    let data = topo.face(face).unwrap();
    let (mut sx, mut sy) = (0.0, 0.0);
    for oriented in topo.wire(data.outer_wire()).unwrap().edges() {
        let edge = topo.edge(oriented.edge()).unwrap();
        let start = if oriented.is_forward() {
            edge.start()
        } else {
            edge.end()
        };
        let p = topo.vertex(start).unwrap().point();
        let r = p.x().hypot(p.y());
        sx += p.x() / r;
        sy += p.y() / r;
    }
    sy.atan2(sx)
}

/// Signed angle from `b` to `a`, wrapped into (−π, π].
fn angle_between(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(TAU);
    if d > PI { d - TAU } else { d }
}

#[test]
fn every_face_of_a_new_gear_has_one_unique_construction_role() {
    let mut external = spur(17);
    external.bore_diameter = Some(8.0);
    let mut internal = GearParams::standard(GearKind::Internal, 1.0, 30, 3.0);
    internal.rim_diameter = Some(40.0);
    let rack = GearParams::standard(GearKind::Rack, 1.0, 5, 2.0);
    for (params, phase) in [(external, 0.3), (internal, -0.2), (rack, 0.0)] {
        let placement = GearPlacement {
            phase,
            ..GearPlacement::default()
        };
        let (topo, outcome) = build_gear(&params, &placement);
        let faces = solid_faces(&topo, outcome.solid).unwrap();
        assert_eq!(outcome.face_roles.len(), faces.len(), "{:?}", params.kind);
        let roled: std::collections::BTreeSet<_> =
            outcome.face_roles.iter().map(|(f, _)| *f).collect();
        assert!(faces.iter().all(|f| roled.contains(f)));
        let names: std::collections::BTreeSet<_> =
            outcome.face_roles.iter().map(|(_, r)| r.name()).collect();
        assert_eq!(
            names.len(),
            faces.len(),
            "{:?}: role names repeat",
            params.kind
        );
        assert!(names.contains("cap.start") && names.contains("cap.end"));

        if params.kind == GearKind::Rack {
            for k in 0..params.teeth {
                assert!(names.contains(&format!("tooth[{k}].tip")));
            }
            assert!(names.contains("back") && names.contains("root.start"));
            continue;
        }
        let pitch = TAU / f64::from(params.teeth);
        for (face, role) in &outcome.face_roles {
            match role.role {
                GearFaceRole::Tip { tooth } => {
                    let center = phase + f64::from(tooth) * pitch;
                    let offset = angle_between(face_angle(&topo, *face), center);
                    assert!(
                        offset.abs() < 1e-9,
                        "{} sits {offset} from its tooth ({:?})",
                        role.name(),
                        params.kind
                    );
                }
                GearFaceRole::Flank { tooth, side } => {
                    let center = phase + f64::from(tooth) * pitch;
                    let offset = angle_between(face_angle(&topo, *face), center);
                    let expected_sign = if side == ToothSide::Plus { 1.0 } else { -1.0 };
                    assert!(
                        offset * expected_sign > 0.0 && offset.abs() < 0.5 * pitch,
                        "{} sits {offset} from its tooth",
                        role.name()
                    );
                    assert!(matches!(
                        topo.face(*face).unwrap().surface(),
                        FaceSurface::Nurbs(_)
                    ));
                }
                GearFaceRole::Root { gap } => {
                    let center = phase + (f64::from(gap) + 0.5) * pitch;
                    assert!(
                        angle_between(face_angle(&topo, *face), center).abs() < 1e-9,
                        "{}",
                        role.name()
                    );
                }
                _ => {}
            }
        }
    }
}

mod enveloping;
mod helical;
mod herringbone;
mod hindley_wheel;
mod worm;
