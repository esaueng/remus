//! Qualification evidence for the draft operation's declared planar domain.
//!
//! Axes covered (see `docs/kernel-maturity/stabilization-plan.md`, item A1):
//! face selection (single wall, all walls, holed neighbours), pull/neutral
//! placement (base, mid-height), angle sign and near-zero boundary, scale
//! (1e-3 / 1 / 1e3), body type (plain, holed, cavity-bearing), and typed
//! refusals tested from both sides of each declared boundary.
//!
//! Every positive case is checked against a closed-form volume, not a prior
//! run's output.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::draft::draft;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};

use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const ANGLE: f64 = 5.0_f64.to_radians();

/// Outer-shell faces whose plane normal matches `target` (within 1e-6) and,
/// when `at` is given, whose plane offset `d` matches it too.
fn planar_faces(topo: &Topology, solid: SolidId, target: Vec3, at: Option<f64>) -> Vec<FaceId> {
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    sh.faces()
        .iter()
        .filter(|&&fid| {
            let f = topo.face(fid).unwrap();
            if let FaceSurface::Plane { normal, d } = f.surface() {
                (*normal - target).length() < 1e-6 && at.is_none_or(|v| (d - v).abs() < 1e-6)
            } else {
                false
            }
        })
        .copied()
        .collect()
}

fn face_count(topo: &Topology, solid: SolidId) -> usize {
    let s = topo.solid(solid).unwrap();
    topo.shell(s.outer_shell()).unwrap().faces().len()
}

/// Single +X wall drafted about the base plane at model scales 1e-3, 1, 1e3:
/// the wall leans outward and adds a wedge of exactly `s^3 * tan(a) / 2`.
#[test]
fn single_wall_volume_exact_across_scales() {
    for s in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, s, s, s).unwrap();
        let wall = planar_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0), None);
        assert_eq!(wall.len(), 1);

        let result = draft(
            &mut topo,
            cube,
            &wall,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            ANGLE,
        )
        .unwrap();

        let expected = s * s * s * (1.0 + ANGLE.tan() / 2.0);
        let vol = solid_volume(&topo, result, 0.01 * s).unwrap();
        // Planar-only tessellation is exact; hold the oracle tight, relative
        // to the body's own volume.
        assert!(
            ((vol - expected) / expected).abs() < 1e-9,
            "scale {s}: relative error too large: expected {expected}, got {vol}"
        );
        assert_eq!(face_count(&topo, result), 6);
    }
}

/// All four side walls drafted about the base plane form an exact frustum:
/// V = ((1 + 2t)^3 - 1) / (6t) for a unit cube, t = tan(angle).
#[test]
fn four_wall_frustum_volume_exact() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let mut walls = Vec::new();
    for n in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
    ] {
        let f = planar_faces(&topo, cube, n, None);
        assert_eq!(f.len(), 1);
        walls.extend(f);
    }

    let result = draft(
        &mut topo,
        cube,
        &walls,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        ANGLE,
    )
    .unwrap();

    let t = ANGLE.tan();
    let expected = ((1.0 + 2.0 * t).powi(3) - 1.0) / (6.0 * t);
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        ((vol - expected) / expected).abs() < 1e-9,
        "expected frustum volume {expected}, got {vol}"
    );
    assert_eq!(face_count(&topo, result), 6);
}

/// A neutral plane at mid-height adds above what it removes below: the
/// volume is unchanged to closed-form precision.
#[test]
fn mid_height_neutral_plane_preserves_volume() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let wall = planar_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0), None);

    let result = draft(
        &mut topo,
        cube,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.5),
        ANGLE,
    )
    .unwrap();

    let vol = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (vol - 1.0).abs() < 1e-9,
        "mid-neutral draft must preserve volume, got {vol}"
    );
}

/// A negative angle tapers inward and removes the same wedge the positive
/// angle adds.
#[test]
fn negative_angle_removes_wedge() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let wall = planar_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0), None);

    let result = draft(
        &mut topo,
        cube,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        -ANGLE,
    )
    .unwrap();

    let expected = 1.0 - ANGLE.tan() / 2.0;
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        ((vol - expected) / expected).abs() < 1e-9,
        "expected {expected}, got {vol}"
    );
}

/// Both sides of the zero-angle boundary: exactly zero is refused as
/// invalid input; an angle barely above the angular tolerance succeeds and
/// still matches the closed form.
#[test]
fn zero_angle_boundary_both_sides() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let wall = planar_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0), None);

    let err = draft(
        &mut topo,
        cube,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        0.0,
    )
    .unwrap_err();
    assert!(matches!(err, OperationsError::InvalidInput { .. }));

    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let wall = planar_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0), None);
    let tiny = 1e-6;
    let result = draft(
        &mut topo,
        cube,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        tiny,
    )
    .unwrap();
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (vol - (1.0 + tiny.tan() / 2.0)).abs() < 1e-12,
        "near-zero draft should still match the closed form, got {vol}"
    );
}

/// A hole through an UNdrafted neighbour is carried through verbatim: the
/// draft succeeds and the result volume is the drafted prism minus the
/// untouched hole prism.
#[test]
fn untouched_hole_carried_through() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    // Square hole through Z at the centre: rims live on the top and bottom
    // faces, well away from the +X wall's lean.
    let cutter = make_box(&mut topo, 0.4, 0.4, 2.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(0.3, 0.3, -0.5)).unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, cube, cutter).unwrap();

    let wall = planar_faces(&topo, holed, Vec3::new(1.0, 0.0, 0.0), Some(1.0));
    assert_eq!(wall.len(), 1, "the +X wall should be a single face");

    let result = draft(
        &mut topo,
        holed,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        ANGLE,
    )
    .unwrap();

    let expected = 1.0 + ANGLE.tan() / 2.0 - 0.4 * 0.4;
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        ((vol - expected) / expected).abs() < 1e-9,
        "expected drafted-minus-hole volume {expected}, got {vol}"
    );
}

/// Drafting a face that itself carries a hole rim is refused by name — the
/// rim cannot be rebuilt from outer-wire positions.
#[test]
fn drafting_holed_face_refused_typed() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    // Square hole through X: rims live on the +X and -X walls.
    let cutter = make_box(&mut topo, 2.0, 0.4, 0.4).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(-0.5, 0.3, 0.3)).unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, cube, cutter).unwrap();

    let wall = planar_faces(&topo, holed, Vec3::new(1.0, 0.0, 0.0), Some(1.0));
    assert_eq!(wall.len(), 1);
    assert!(
        !topo.face(wall[0]).unwrap().inner_wires().is_empty(),
        "test setup: the wall must carry the hole rim"
    );

    let err = draft(
        &mut topo,
        holed,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        ANGLE,
    )
    .unwrap_err();
    assert!(
        matches!(&err, OperationsError::Unsupported { operation, reason }
            if *operation == "draft" && reason.contains("inner wire")),
        "expected a typed inner-wire refusal, got {err:?}"
    );
}

/// A cavity-bearing solid is refused by name rather than silently drafted
/// with its cavity left behind.
#[test]
fn cavity_solid_refused_typed() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    // A tool strictly inside the blank cuts a sealed cavity: the result
    // carries the void as an inner shell.
    let void = make_box(&mut topo, 0.4, 0.4, 0.4).unwrap();
    transform_solid(&mut topo, void, &Mat4::translation(0.3, 0.3, 0.3)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, cube, void).unwrap();
    assert!(
        !topo.solid(hollow).unwrap().inner_shells().is_empty(),
        "test setup: the cut must carry the void as a cavity shell"
    );

    let wall = planar_faces(&topo, hollow, Vec3::new(1.0, 0.0, 0.0), Some(1.0));
    assert_eq!(wall.len(), 1);

    let err = draft(
        &mut topo,
        hollow,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        ANGLE,
    )
    .unwrap_err();
    assert!(
        matches!(&err, OperationsError::Unsupported { operation, reason }
            if *operation == "draft" && reason.contains("cavity")),
        "expected a typed cavity refusal, got {err:?}"
    );
}

/// A non-planar target face is invalid input, not a silent approximation.
#[test]
fn non_planar_target_refused_typed() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let s = topo.solid(cyl).unwrap();
    let lateral: Vec<FaceId> = topo
        .shell(s.outer_shell())
        .unwrap()
        .faces()
        .iter()
        .filter(|&&fid| !matches!(topo.face(fid).unwrap().surface(), FaceSurface::Plane { .. }))
        .copied()
        .collect();
    assert!(!lateral.is_empty());

    let err = draft(
        &mut topo,
        cyl,
        &lateral,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        ANGLE,
    )
    .unwrap_err();
    assert!(
        matches!(&err, OperationsError::InvalidInput { reason } if reason.contains("planar")),
        "expected an invalid-input refusal, got {err:?}"
    );
}

/// A face handle from a different solid is invalid input.
#[test]
fn foreign_face_refused_typed() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let b_wall = planar_faces(&topo, b, Vec3::new(1.0, 0.0, 0.0), None);
    assert_eq!(b_wall.len(), 1);

    let err = draft(
        &mut topo,
        a,
        &b_wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        ANGLE,
    )
    .unwrap_err();
    assert!(matches!(err, OperationsError::InvalidInput { .. }));
}

/// The same draft in two fresh topologies yields bit-identical volume and
/// face count — the operation is deterministic.
#[test]
fn draft_is_deterministic() {
    let run = || {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let wall = planar_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0), None);
        let result = draft(
            &mut topo,
            cube,
            &wall,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            ANGLE,
        )
        .unwrap();
        let vol = solid_volume(&topo, result, 0.01).unwrap();
        (vol.to_bits(), face_count(&topo, result))
    };
    assert_eq!(run(), run());
}
