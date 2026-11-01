#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeSet;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;

use super::*;
use crate::gear::{GearFaceRole, Hand, make_gear};
use crate::measure::solid_volume;
use crate::primitives::make_cylinder;
use crate::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use crate::transform::transform_solid;
use crate::validate::validate_solid;

fn moved_cylinder(topo: &mut Topology, radius: f64, height: f64, z: f64) -> SolidId {
    let solid = make_cylinder(topo, radius, height).unwrap();
    transform_solid(topo, solid, &Mat4::translation(0.0, 0.0, z)).unwrap();
    solid
}

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

/// Volume at a fine deflection: the coarse measurement path can read a few
/// parts in 1e4 low on boolean results with NURBS faces.
fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, 1e-4).unwrap()
}

fn assert_valid_and_watertight(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.2).unwrap();
    assert!(is_watertight(&mesh));
}

/// Area enclosed by a standalone gear outline with these parameters, from a
/// boolean-free `make_gear` body: its volume over its face width.
fn outline_area(params: &GearParams) -> f64 {
    let mut topo = Topology::new();
    let outcome = make_gear(&mut topo, params, &GearPlacement::default()).unwrap();
    volume(&topo, outcome.solid) / params.face_width
}

fn spur(teeth: u32, face_width: f64) -> GearParams {
    GearParams::standard(GearKind::External, 1.0, teeth, face_width)
}

fn assert_roles_are_unique_and_typed(topo: &Topology, outcome: &GearOutcome, teeth: u32) {
    let names: BTreeSet<_> = outcome.face_roles.iter().map(|(_, r)| r.name()).collect();
    assert_eq!(names.len(), outcome.face_roles.len(), "role names repeat");
    let faces: BTreeSet<_> = outcome.face_roles.iter().map(|(f, _)| *f).collect();
    assert_eq!(
        faces.len(),
        outcome.face_roles.len(),
        "a face has two roles"
    );
    let mut flanks = 0;
    for (face, role) in &outcome.face_roles {
        let surface = topo.face(*face).unwrap().surface();
        match role.role {
            GearFaceRole::Flank { .. } => {
                flanks += 1;
                assert!(matches!(surface, FaceSurface::Nurbs(_)), "{}", role.name());
            }
            GearFaceRole::Tip { .. } | GearFaceRole::Root { .. } | GearFaceRole::Fillet { .. } => {
                assert!(
                    matches!(surface, FaceSurface::Cylinder(_)),
                    "{}",
                    role.name()
                );
            }
            _ => {}
        }
    }
    assert_eq!(flanks, 2 * teeth, "every flank keeps its role");
}

#[test]
fn keep_diameter_teeth_on_a_plain_boss_match_the_standalone_gear() {
    let mut topo = Topology::new();
    let blank = make_cylinder(&mut topo, 11.0, 6.0).unwrap();
    let face = cylinder_face(&topo, blank, 11.0);
    let params = spur(20, 6.0);
    let outcome = cut_gear_teeth(
        &mut topo,
        blank,
        face,
        &params,
        &CylinderGearOptions::default(),
    )
    .unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);

    // Tip diameter is the boss: m = 22 / (20 + 2).
    let g = &outcome.geometry;
    assert!((g.module - 1.0).abs() < 1e-12);
    assert!((g.tip_diameter.unwrap() - 22.0).abs() < 1e-9);
    let mut standalone = spur(20, 6.0);
    standalone.module = g.module;
    let expected = outline_area(&standalone) * 6.0;
    let got = volume(&topo, outcome.solid);
    assert!(
        ((got - expected) / expected).abs() < 1e-6,
        "{got} vs {expected}"
    );
    assert_roles_are_unique_and_typed(&topo, &outcome, 20);
    // The input body is untouched.
    assert!((volume(&topo, blank) - PI * 121.0 * 6.0).abs() < 1e-6);
}

#[test]
fn teeth_on_the_hub_of_a_stepped_bored_shaft_keep_every_other_feature() {
    let mut topo = Topology::new();
    let hub = make_cylinder(&mut topo, 12.0, 10.0).unwrap();
    let shaft = moved_cylinder(&mut topo, 5.0, 30.0, -10.0);
    let stepped = boolean(&mut topo, BooleanOp::Fuse, hub, shaft).unwrap();
    let drill = moved_cylinder(&mut topo, 2.5, 50.0, -20.0);
    let blank = boolean(&mut topo, BooleanOp::Cut, stepped, drill).unwrap();
    let blank_volume = volume(&topo, blank);
    let face = cylinder_face(&topo, blank, 12.0);

    let params = GearParams::standard(GearKind::External, 1.0, 22, 10.0);
    let outcome = cut_gear_teeth(
        &mut topo,
        blank,
        face,
        &params,
        &CylinderGearOptions::default(),
    )
    .unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);

    // Only the hub changes: its disc becomes the gear outline.
    let area = outline_area(&params);
    let expected = blank_volume - (PI * 144.0 - area) * 10.0;
    let got = volume(&topo, outcome.solid);
    assert!(
        ((got - expected) / expected).abs() < 1e-6,
        "{got} vs {expected}"
    );
    // The bore and the shaft on both sides survive; the hub cylinder is gone.
    cylinder_face(&topo, outcome.solid, 2.5);
    let shaft_faces = solid_faces(&topo, outcome.solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            matches!(topo.face(f).unwrap().surface(),
            FaceSurface::Cylinder(c) if (c.radius() - 5.0).abs() < 1e-9)
        })
        .count();
    assert!(shaft_faces >= 2, "{shaft_faces} shaft faces");
    // Every radius-12 face left is a tooth tip: the hub's own wall is gone.
    let tips: BTreeSet<_> = outcome
        .face_roles
        .iter()
        .filter(|(_, face)| matches!(face.role, GearFaceRole::Tip { .. }))
        .map(|(face, _)| *face)
        .collect();
    for face in solid_faces(&topo, outcome.solid).unwrap() {
        if matches!(topo.face(face).unwrap().surface(),
            FaceSurface::Cylinder(c) if (c.radius() - 12.0).abs() < 1e-9)
        {
            assert!(tips.contains(&face));
        }
    }
    assert_eq!(tips.len(), 22);
    assert_roles_are_unique_and_typed(&topo, &outcome, 22);
}

#[test]
fn internal_teeth_in_a_bore_open_the_ring() {
    let mut topo = Topology::new();
    let outer = make_cylinder(&mut topo, 30.0, 5.0).unwrap();
    let drill = moved_cylinder(&mut topo, 19.0, 20.0, -5.0);
    let ring = boolean(&mut topo, BooleanOp::Cut, outer, drill).unwrap();
    let ring_volume = volume(&topo, ring);
    let face = cylinder_face(&topo, ring, 19.0);

    let params = GearParams::standard(GearKind::Internal, 1.0, 40, 5.0);
    let outcome = cut_gear_teeth(
        &mut topo,
        ring,
        face,
        &params,
        &CylinderGearOptions::default(),
    )
    .unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    // Inner tip diameter is the bore: m = 38 / (40 − 2).
    assert!((outcome.geometry.module - 1.0).abs() < 1e-12);
    assert!((outcome.geometry.tip_diameter.unwrap() - 38.0).abs() < 1e-9);

    // The hole grows from the bore circle to the internal outline.
    let mut standalone = params.clone();
    standalone.rim_diameter = Some(56.0);
    let hole_area = PI * 28.0 * 28.0 - outline_area(&standalone);
    let expected = ring_volume - (hole_area - PI * 19.0 * 19.0) * 5.0;
    let got = volume(&topo, outcome.solid);
    assert!(
        ((got - expected) / expected).abs() < 1e-6,
        "{got} vs {expected}"
    );
    cylinder_face(&topo, outcome.solid, 30.0);
    assert_roles_are_unique_and_typed(&topo, &outcome, 40);
}

#[test]
fn a_given_module_turns_the_boss_down_and_phase_turns_the_teeth() {
    let mut topo = Topology::new();
    let blank = make_cylinder(&mut topo, 15.0, 4.0).unwrap();
    let face = cylinder_face(&topo, blank, 15.0);
    let params = GearParams::standard(GearKind::External, 1.25, 18, 4.0);
    let pitch_angle = 2.0 * PI / 18.0;
    let options = CylinderGearOptions {
        fit: GearFit::UseModule,
        x_direction: Some(Vec3::new(1.0, 0.0, 0.0)),
        phase: 0.5 * pitch_angle,
        ..CylinderGearOptions::default()
    };
    let outcome = cut_gear_teeth(&mut topo, blank, face, &params, &options).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    let tip = 0.5 * outcome.geometry.tip_diameter.unwrap();
    assert!((tip - (11.25 + 1.25)).abs() < 1e-12);
    let expected = outline_area(&params) * 4.0;
    let got = volume(&topo, outcome.solid);
    assert!(
        ((got - expected) / expected).abs() < 1e-6,
        "{got} vs {expected}"
    );

    // Half a pitch of phase: a tooth space on +x, a tooth half a pitch on.
    let probe = |angle: f64| {
        let (sin, cos) = angle.sin_cos();
        let point = Point3::new((tip - 0.05) * cos, (tip - 0.05) * sin, 2.0);
        classify_point(&topo, outcome.solid, point, 0.01, 1e-6).unwrap()
    };
    assert_eq!(probe(0.0), PointClassification::Outside);
    assert_eq!(probe(0.5 * pitch_angle), PointClassification::Inside);
}

fn refusal(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    params: &GearParams,
    options: &CylinderGearOptions,
) -> GearRefusal {
    let before = (topo.num_faces(), topo.num_edges(), topo.num_solids());
    match cut_gear_teeth(topo, solid, face, params, options) {
        Err(GearError::Refused { code, .. }) => {
            assert_eq!(
                (topo.num_faces(), topo.num_edges(), topo.num_solids()),
                before,
                "a refusal must roll back"
            );
            code
        }
        other => panic!("expected a refusal, got {other:?}"),
    }
}

#[test]
fn unbuildable_requests_refuse_atomically() {
    let mut topo = Topology::new();
    let blank = make_cylinder(&mut topo, 11.0, 6.0).unwrap();
    let face = cylinder_face(&topo, blank, 11.0);
    let defaults = CylinderGearOptions::default();

    let mut too_big = GearParams::standard(GearKind::External, 1.0, 21, 6.0);
    too_big.module = 1.0;
    let use_module = CylinderGearOptions {
        fit: GearFit::UseModule,
        ..defaults
    };
    assert_eq!(
        refusal(&mut topo, blank, face, &too_big, &use_module),
        GearRefusal::CylinderTooSmall
    );

    // Teeth reaching past either end of the face are refused.
    for (offset, width) in [(0.0, 6.5), (2.0, 5.0), (-0.5, 4.0)] {
        let past = GearParams::standard(GearKind::External, 1.0, 20, width);
        let options = CylinderGearOptions { offset, ..defaults };
        assert_eq!(
            refusal(&mut topo, blank, face, &past, &options),
            GearRefusal::SpanOutsideFace
        );
    }

    let cap = solid_faces(&topo, blank)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { .. }))
        .unwrap();
    assert_eq!(
        refusal(&mut topo, blank, cap, &spur(20, 6.0), &defaults),
        GearRefusal::NotACylinder
    );

    let internal = GearParams::standard(GearKind::Internal, 1.0, 40, 6.0);
    assert_eq!(
        refusal(&mut topo, blank, face, &internal, &defaults),
        GearRefusal::InvalidParameter
    );
    let mut with_bore = spur(20, 6.0);
    with_bore.bore_diameter = Some(4.0);
    assert_eq!(
        refusal(&mut topo, blank, face, &with_bore, &defaults),
        GearRefusal::InvalidParameter
    );
}

#[test]
fn a_cross_hole_through_the_tooth_band_refuses_instead_of_being_filled() {
    let mut topo = Topology::new();
    let boss = make_cylinder(&mut topo, 11.0, 6.0).unwrap();
    // A radial hole of radius 0.8 along x through the boss at mid height.
    let pin = make_cylinder(&mut topo, 0.8, 30.0).unwrap();
    transform_solid(
        &mut topo,
        pin,
        &(Mat4::translation(-15.0, 0.0, 3.0) * Mat4::rotation_y(0.5 * PI)),
    )
    .unwrap();
    let blank = boolean(&mut topo, BooleanOp::Cut, boss, pin).unwrap();
    let face = cylinder_face(&topo, blank, 11.0);
    assert_eq!(
        refusal(
            &mut topo,
            blank,
            face,
            &spur(20, 6.0),
            &CylinderGearOptions::default()
        ),
        GearRefusal::FeaturesInToothRegion
    );
}

#[test]
fn bore_larger_than_the_inner_tip_refuses_with_dimensions() {
    let mut topo = Topology::new();
    let outer = make_cylinder(&mut topo, 30.0, 5.0).unwrap();
    let drill = moved_cylinder(&mut topo, 19.0, 20.0, -5.0);
    let ring = boolean(&mut topo, BooleanOp::Cut, outer, drill).unwrap();
    let face = cylinder_face(&topo, ring, 19.0);
    let params = GearParams::standard(GearKind::Internal, 0.8, 40, 5.0);
    let options = CylinderGearOptions {
        fit: GearFit::UseModule,
        ..CylinderGearOptions::default()
    };
    match cut_gear_teeth(&mut topo, ring, face, &params, &options) {
        Err(GearError::Refused {
            code: GearRefusal::BoreTooLarge,
            geometry: Some(geometry),
            ..
        }) => assert!((geometry.tip_diameter.unwrap() - 30.4).abs() < 1e-9),
        other => panic!("expected bore-too-large, got {other:?}"),
    }
}

#[test]
fn helical_teeth_on_a_boss_keep_its_diameter_and_match_the_standalone_gear() {
    let mut topo = Topology::new();
    let blank = make_cylinder(&mut topo, 11.0, 6.0).unwrap();
    let face = cylinder_face(&topo, blank, 11.0);
    let params = GearParams {
        helix_angle: 15.0_f64.to_radians(),
        hand: Hand::Left,
        ..spur(20, 6.0)
    };
    let outcome = cut_gear_teeth(
        &mut topo,
        blank,
        face,
        &params,
        &CylinderGearOptions::default(),
    )
    .unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);

    // Tip diameter is the boss: m_n = 22 / (20/cos β + 2).
    let g = &outcome.geometry;
    let expected_module = 22.0 / (20.0 / 15.0_f64.to_radians().cos() + 2.0);
    assert!((g.module - expected_module).abs() < 1e-12);
    assert!((g.tip_diameter.unwrap() - 22.0).abs() < 1e-9);
    assert!(outcome.spans_along_axis >= 1);
    let mut standalone = params.clone();
    standalone.module = g.module;
    let expected = outline_area(&standalone) * 6.0;
    let got = crate::measure::solid_volume(&topo, outcome.solid, 1e-4).unwrap();
    assert!(
        ((got - expected) / expected).abs() < 1e-6,
        "{got} vs {expected}"
    );
    let flanks = outcome
        .face_roles
        .iter()
        .filter(|(_, face)| matches!(face.role, GearFaceRole::Flank { .. }))
        .count();
    assert_eq!(flanks, 40, "every flank keeps its role through the fuse");
}

/// Helical and herringbone teeth over part of a boss, collars kept either
/// side: each twisted flank and land meets a collar cap only along its own end
/// edge. The band changes from the r 12 disc to the gear's transverse outline.
#[test]
fn helical_teeth_over_part_of_a_boss_match_the_standalone_gear() {
    let cases = [
        (Hand::Right, false, 2.0, 6.0),
        (Hand::Left, false, 4.0, 6.0),
        (Hand::Right, true, 2.0, 6.0),
    ];
    for (hand, herringbone, offset, width) in cases {
        let mut topo = Topology::new();
        let blank = make_cylinder(&mut topo, 12.0, 10.0).unwrap();
        let face = cylinder_face(&topo, blank, 12.0);
        let params = GearParams {
            helix_angle: 15.0_f64.to_radians(),
            hand,
            herringbone,
            ..spur(22, width)
        };
        let options = CylinderGearOptions {
            offset,
            ..CylinderGearOptions::default()
        };
        let outcome = cut_gear_teeth(&mut topo, blank, face, &params, &options)
            .unwrap_or_else(|error| panic!("{hand:?} herringbone {herringbone}: {error}"));
        assert_valid_and_watertight(&topo, outcome.solid);
        let mut standalone = params.clone();
        standalone.module = outcome.geometry.module;
        let expected = PI * 144.0 * 10.0 - (PI * 144.0 - outline_area(&standalone)) * width;
        let got = volume(&topo, outcome.solid);
        assert!(
            ((got - expected) / expected).abs() < 1e-9,
            "{hand:?} herringbone {herringbone} at {offset}: {got} vs {expected}"
        );
    }
}

#[test]
fn herringbone_teeth_on_a_boss_keep_its_diameter_and_match_the_standalone_gear() {
    let mut topo = Topology::new();
    let blank = make_cylinder(&mut topo, 11.0, 6.0).unwrap();
    let face = cylinder_face(&topo, blank, 11.0);
    let params = GearParams {
        helix_angle: 15.0_f64.to_radians(),
        herringbone: true,
        ..spur(20, 6.0)
    };
    let outcome = cut_gear_teeth(
        &mut topo,
        blank,
        face,
        &params,
        &CylinderGearOptions::default(),
    )
    .unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    let g = &outcome.geometry;
    assert!((g.tip_diameter.unwrap() - 22.0).abs() < 1e-9);
    let mut standalone = params.clone();
    standalone.module = g.module;
    let expected = outline_area(&standalone) * 6.0;
    let got = crate::measure::solid_volume(&topo, outcome.solid, 1e-4).unwrap();
    assert!(
        ((got - expected) / expected).abs() < 1e-6,
        "{got} vs {expected}"
    );
    let flanks = outcome
        .face_roles
        .iter()
        .filter(|(_, face)| matches!(face.role, GearFaceRole::Flank { .. }))
        .count();
    assert_eq!(
        flanks, 80,
        "both halves' flanks keep their roles through the fuse"
    );
}

/// The stepped, bored shaft of the hub test with teeth over the middle six
/// of the hub's ten: collars of the r 12 hub stay above and below.
fn mid_hub_blank(topo: &mut Topology) -> (SolidId, FaceId, f64) {
    let hub = make_cylinder(topo, 12.0, 10.0).unwrap();
    let shaft = moved_cylinder(topo, 5.0, 30.0, -10.0);
    let stepped = boolean(topo, BooleanOp::Fuse, hub, shaft).unwrap();
    let drill = moved_cylinder(topo, 2.5, 50.0, -20.0);
    let blank = boolean(topo, BooleanOp::Cut, stepped, drill).unwrap();
    // Closed form: hub, shaft beyond it, less the bore through all 30 of it.
    let closed = PI * 144.0 * 10.0 + PI * 25.0 * 20.0 - PI * 6.25 * 30.0;
    let face = cylinder_face(topo, blank, 12.0);
    (blank, face, closed)
}

/// Keep-diameter teeth over z 2..8 of the hub: their tips are flush with the
/// collars. Only the band changes, from the r 12 disc to the gear outline.
#[test]
fn keep_diameter_teeth_on_the_middle_of_a_stepped_bored_hub() {
    let mut topo = Topology::new();
    let (blank, face, closed) = mid_hub_blank(&mut topo);
    assert!(((volume(&topo, blank) - closed) / closed).abs() < 1e-9);
    let params = GearParams::standard(GearKind::External, 1.0, 22, 6.0);
    let options = CylinderGearOptions {
        offset: 2.0,
        ..CylinderGearOptions::default()
    };
    let outcome = cut_gear_teeth(&mut topo, blank, face, &params, &options).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    let expected = closed - (PI * 144.0 - outline_area(&params)) * 6.0;
    let got = volume(&topo, outcome.solid);
    assert!(
        ((got - expected) / expected).abs() < 1e-9,
        "{got} vs {expected}"
    );
    // The collars keep the full r 12 disc; the band has tooth spaces.
    let pitch = 2.0 * PI / 22.0;
    let probe = |angle: f64, z: f64| {
        let (sin, cos) = angle.sin_cos();
        let point = Point3::new(11.5 * cos, 11.5 * sin, z);
        classify_point(&topo, outcome.solid, point, 0.01, 1e-6).unwrap()
    };
    let mut spaces = 0;
    for k in 0..44 {
        let angle = f64::from(k) * 0.5 * pitch + 0.1;
        assert_eq!(probe(angle, 1.0), PointClassification::Inside);
        assert_eq!(probe(angle, 9.0), PointClassification::Inside);
        spaces += usize::from(probe(angle, 5.0) == PointClassification::Outside);
    }
    assert!(
        (15..=29).contains(&spaces),
        "{spaces} of 44 probes in tooth spaces"
    );
    assert_roles_are_unique_and_typed(&topo, &outcome, 22);
}

/// A given module turns the band down below the collars: the ring's caps
/// meet the collars' annular caps around the whole closed tooth outline.
#[test]
fn turned_down_teeth_on_the_middle_of_a_stepped_bored_hub() {
    let mut topo = Topology::new();
    let (blank, face, closed) = mid_hub_blank(&mut topo);
    let params = GearParams::standard(GearKind::External, 0.9, 22, 6.0);
    let options = CylinderGearOptions {
        fit: GearFit::UseModule,
        offset: 2.0,
        ..CylinderGearOptions::default()
    };
    let outcome = cut_gear_teeth(&mut topo, blank, face, &params, &options).unwrap();
    assert_valid_and_watertight(&topo, outcome.solid);
    assert!((outcome.geometry.tip_diameter.unwrap() - 21.6).abs() < 1e-9);
    let expected = closed - (PI * 144.0 - outline_area(&params)) * 6.0;
    let got = volume(&topo, outcome.solid);
    assert!(
        ((got - expected) / expected).abs() < 1e-9,
        "{got} vs {expected}"
    );
    let probe = |r: f64, z: f64| {
        classify_point(&topo, outcome.solid, Point3::new(0.0, r, z), 0.01, 1e-6).unwrap()
    };
    assert_eq!(probe(11.5, 0.5), PointClassification::Inside);
    assert_eq!(probe(11.5, 5.0), PointClassification::Outside);
    assert_eq!(probe(11.5, 9.5), PointClassification::Inside);
}

/// Internal teeth over part of a bore, collars kept either side: the hole
/// grows from the bore circle to the internal outline over the span only.
/// The outline's tip corners sit on the bore circle of the collars' caps.
#[test]
fn internal_teeth_over_part_of_a_bore_open_only_the_span() {
    for (teeth, offset, width) in [
        (40, 2.0, 5.0),
        (40, 0.0, 5.0),
        (40, 4.0, 5.0),
        (30, 3.0, 3.0),
    ] {
        let mut topo = Topology::new();
        let outer = make_cylinder(&mut topo, 30.0, 9.0).unwrap();
        let drill = moved_cylinder(&mut topo, 19.0, 20.0, -5.0);
        let ring = boolean(&mut topo, BooleanOp::Cut, outer, drill).unwrap();
        let ring_volume = volume(&topo, ring);
        let face = cylinder_face(&topo, ring, 19.0);
        let params = GearParams::standard(GearKind::Internal, 1.0, teeth, width);
        let options = CylinderGearOptions {
            offset,
            ..CylinderGearOptions::default()
        };
        let outcome = cut_gear_teeth(&mut topo, ring, face, &params, &options)
            .unwrap_or_else(|error| panic!("{teeth} teeth at {offset}: {error}"));
        assert_valid_and_watertight(&topo, outcome.solid);
        // The inner tip diameter is the bore: m = 38 / (z − 2).
        let module = outcome.geometry.module;
        assert!((module - 38.0 / f64::from(teeth - 2)).abs() < 1e-12);

        let mut standalone = GearParams::standard(GearKind::Internal, module, teeth, width);
        standalone.rim_diameter = Some(56.0);
        let hole_area = PI * 28.0 * 28.0 - outline_area(&standalone);
        let expected = ring_volume - (hole_area - PI * 19.0 * 19.0) * width;
        let got = volume(&topo, outcome.solid);
        assert!(
            ((got - expected) / expected).abs() < 1e-9,
            "{teeth} teeth at {offset}..{}: {got} vs {expected}",
            offset + width
        );
    }
}

/// The circles the operation adds start on the phase reference, not on the
/// face's seam: any reference gives the same volume, whole face or partial.
#[test]
fn any_phase_reference_builds_the_same_teeth() {
    let params = GearParams::standard(GearKind::External, 1.0, 22, 6.0);
    for degrees in [0.0_f64, 37.0, 90.0, 200.0] {
        let (sin, cos) = degrees.to_radians().sin_cos();
        let mut topo = Topology::new();
        let (blank, face, closed) = mid_hub_blank(&mut topo);
        let options = CylinderGearOptions {
            offset: 2.0,
            x_direction: Some(Vec3::new(cos, sin, 0.0)),
            ..CylinderGearOptions::default()
        };
        let outcome = cut_gear_teeth(&mut topo, blank, face, &params, &options)
            .unwrap_or_else(|error| panic!("reference {degrees}°: {error}"));
        assert_valid_and_watertight(&topo, outcome.solid);
        let got = volume(&topo, outcome.solid);
        let expected = closed - (PI * 144.0 - outline_area(&params)) * 6.0;
        assert!(
            ((got - expected) / expected).abs() < 1e-9,
            "reference {degrees}°: {got} vs {expected}"
        );
    }
    let whole = GearParams::standard(GearKind::External, 1.0, 20, 6.0);
    for degrees in [37.0_f64, 271.0] {
        let (sin, cos) = degrees.to_radians().sin_cos();
        let mut topo = Topology::new();
        let blank = make_cylinder(&mut topo, 11.0, 6.0).unwrap();
        let face = cylinder_face(&topo, blank, 11.0);
        let options = CylinderGearOptions {
            x_direction: Some(Vec3::new(cos, sin, 0.0)),
            ..CylinderGearOptions::default()
        };
        let outcome = cut_gear_teeth(&mut topo, blank, face, &whole, &options)
            .unwrap_or_else(|error| panic!("whole face, reference {degrees}°: {error}"));
        assert_valid_and_watertight(&topo, outcome.solid);
        let expected = outline_area(&whole) * 6.0;
        let got = volume(&topo, outcome.solid);
        assert!(
            ((got - expected) / expected).abs() < 1e-9,
            "whole face, reference {degrees}°: {got} vs {expected}"
        );
    }
}

/// Flush tips over the middle of a plain boss, for every tooth count from 16
/// to 30: tip lands ride the collars' rims, and wherever a rim's seam or
/// quarter station falls on a tip land the ring's cap still pairs with the
/// collar's.
#[test]
fn keep_diameter_teeth_on_the_middle_of_a_plain_boss() {
    for teeth in 16_u32..=30 {
        let radius = 0.5 * f64::from(teeth + 2);
        let mut topo = Topology::new();
        let blank = make_cylinder(&mut topo, radius, 6.0).unwrap();
        let face = cylinder_face(&topo, blank, radius);
        let params = GearParams::standard(GearKind::External, 1.0, teeth, 3.0);
        let options = CylinderGearOptions {
            offset: 1.5,
            ..CylinderGearOptions::default()
        };
        let outcome = cut_gear_teeth(&mut topo, blank, face, &params, &options)
            .unwrap_or_else(|error| panic!("{teeth} teeth: {error}"));
        assert_valid_and_watertight(&topo, outcome.solid);
        let expected =
            PI * radius * radius * 6.0 - (PI * radius * radius - outline_area(&params)) * 3.0;
        let got = volume(&topo, outcome.solid);
        assert!(
            ((got - expected) / expected).abs() < 1e-9,
            "{teeth} teeth: {got} vs {expected}"
        );
    }
}

/// Internal helical teeth in a bore, whole or over part of it. The tooth tip
/// and root lands are helical strips facing the axis; once their twist
/// outgrew their width, the shell orientation test read the fused ring as
/// inward and assembly refused it.
#[test]
fn internal_helical_teeth_in_a_bore_match_the_standalone_gear() {
    for (hand, offset, width, height) in [
        (Hand::Right, 0.0, 7.0, 7.0),
        (Hand::Left, 0.0, 10.0, 10.0),
        (Hand::Left, 1.0, 7.0, 9.0),
    ] {
        let mut topo = Topology::new();
        let outer = make_cylinder(&mut topo, 30.0, height).unwrap();
        let drill = moved_cylinder(&mut topo, 19.0, height + 10.0, -5.0);
        let ring = boolean(&mut topo, BooleanOp::Cut, outer, drill).unwrap();
        let ring_volume = volume(&topo, ring);
        let face = cylinder_face(&topo, ring, 19.0);
        let params = GearParams {
            helix_angle: 15.0_f64.to_radians(),
            hand,
            ..GearParams::standard(GearKind::Internal, 1.0, 40, width)
        };
        let options = CylinderGearOptions {
            offset,
            ..CylinderGearOptions::default()
        };
        let outcome = cut_gear_teeth(&mut topo, ring, face, &params, &options)
            .unwrap_or_else(|error| panic!("{hand:?} {offset}..{}: {error}", offset + width));
        assert_valid_and_watertight(&topo, outcome.solid);

        let mut standalone = params.clone();
        standalone.module = outcome.geometry.module;
        standalone.rim_diameter = Some(56.0);
        let hole_area = PI * 28.0 * 28.0 - outline_area(&standalone);
        let expected = ring_volume - (hole_area - PI * 19.0 * 19.0) * width;
        let got = volume(&topo, outcome.solid);
        assert!(
            ((got - expected) / expected).abs() < 1e-9,
            "{hand:?} {offset}..{}: {got} vs {expected}",
            offset + width
        );
    }
}

/// Internal herringbone teeth in a bore, whole or over part of it. The
/// ring's rim is two faces, one per half; the groove wall it meets is one,
/// and the fuse kept that wall as a stray face until the halves were merged.
#[test]
fn internal_herringbone_teeth_in_a_bore_match_the_standalone_gear() {
    for (offset, width, height) in [(0.0, 6.0, 6.0), (2.0, 6.0, 9.0)] {
        let mut topo = Topology::new();
        let outer = make_cylinder(&mut topo, 30.0, height).unwrap();
        let drill = moved_cylinder(&mut topo, 19.0, height + 10.0, -5.0);
        let ring = boolean(&mut topo, BooleanOp::Cut, outer, drill).unwrap();
        let ring_volume = volume(&topo, ring);
        let face = cylinder_face(&topo, ring, 19.0);
        let params = GearParams {
            helix_angle: 15.0_f64.to_radians(),
            herringbone: true,
            ..GearParams::standard(GearKind::Internal, 1.0, 40, width)
        };
        let options = CylinderGearOptions {
            offset,
            ..CylinderGearOptions::default()
        };
        let outcome = cut_gear_teeth(&mut topo, ring, face, &params, &options)
            .unwrap_or_else(|error| panic!("{offset}..{}: {error}", offset + width));
        assert_valid_and_watertight(&topo, outcome.solid);

        let mut standalone = params.clone();
        standalone.module = outcome.geometry.module;
        standalone.rim_diameter = Some(56.0);
        let hole_area = PI * 28.0 * 28.0 - outline_area(&standalone);
        let expected = ring_volume - (hole_area - PI * 19.0 * 19.0) * width;
        let got = volume(&topo, outcome.solid);
        assert!(
            ((got - expected) / expected).abs() < 1e-9,
            "{offset}..{}: {got} vs {expected}",
            offset + width
        );
    }
}
