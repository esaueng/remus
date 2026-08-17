//! Regression: `draft` must not fill the holes in the faces it rebuilds.
//!
//! `draft` rebuilt every face of the solid from a list of outer-wire vertex
//! positions and handed the assembler `inner_wires: vec![]` for all of them.
//! A face carrying an inner wire — a through bore's mouth, a pocket opening —
//! therefore came back solid: the bore walls kept their rims but nothing
//! referenced them, and the drafted body silently gained the material the
//! holes had removed. The same rebuild left the neighbours of a drafted face
//! at their original corners while the drafted face's corners moved, so the
//! shell did not even close; nothing checked, because `draft` returned
//! whatever the assembler produced.
//!
//! This is the same defect that made `defeature` lose the second bore of a
//! two-bore plate. Both are the worst kind: a confident, well-formed, wrong
//! solid.
//!
//! What must hold now:
//!   * a face `draft` does not move comes through verbatim, holes included;
//!   * a face `draft` does move keeps its inner wires, or the operation is
//!     refused with a typed error naming the face;
//!   * every returned solid is closed, valid, and encloses positive volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::draft::draft;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::{OperationsError, measure, validate};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const W: f64 = 80.0;
const D: f64 = 60.0;
const T: f64 = 6.0;
const BORE_R: f64 = 4.0;
/// Draft angle used throughout. Large enough that a filled hole cannot hide
/// inside the taper's own volume change.
const ANGLE: f64 = 5.0;
/// Deflection for volume comparisons; fine enough that a bore's faceting
/// stays far inside the margins asserted below.
const DEFLECTION: f64 = 0.01;

fn pull() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

fn neutral() -> Point3 {
    Point3::new(0.0, 0.0, 0.0)
}

/// A `W` x `D` x `T` plate with `holes` drilled straight through in Z.
fn plate(topo: &mut Topology, holes: &[(f64, f64)]) -> SolidId {
    let mut body = make_box(topo, W, D, T).expect("plate blank");
    for &(x, y) in holes {
        let drill = make_cylinder(topo, BORE_R, T + 4.0).expect("drill");
        transform_solid(topo, drill, &Mat4::translation(x, y, -2.0)).expect("place drill");
        body = boolean(topo, BooleanOp::Cut, body, drill).expect("drill bore");
    }
    body
}

/// Total inner-wire count across the solid's outer shell.
fn hole_count(topo: &Topology, solid: SolidId) -> usize {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    shell
        .faces()
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .sum()
}

fn face_count(topo: &Topology, solid: SolidId) -> usize {
    topo.shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .faces()
        .len()
}

/// Panics with the validator's own words if `solid` is not a valid solid.
fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate::validate_solid(topo, solid).expect("validate");
    assert!(
        report.is_valid(),
        "{what} is not a valid solid: {}",
        report
            .issues
            .iter()
            .filter(|i| i.severity == validate::Severity::Error)
            .map(|i| i.description.clone())
            .collect::<Vec<_>>()
            .join("; ")
    );
}

/// Planar faces whose outward normal is `n`.
fn faces_facing(topo: &Topology, solid: SolidId, n: Vec3) -> Vec<FaceId> {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    shell
        .faces()
        .iter()
        .filter(|&&f| {
            let face = topo.face(f).unwrap();
            matches!(face.surface(), FaceSurface::Plane { .. })
                && face
                    .effective_plane_normal()
                    .is_some_and(|e| (e - n).length() < 1e-9)
        })
        .copied()
        .collect()
}

/// The outermost face facing `n` — a pocket's own low-X wall also faces +X,
/// so "the +X wall of the block" is the one furthest along `n`.
fn outer_face_facing(topo: &Topology, solid: SolidId, n: Vec3) -> Vec<FaceId> {
    let mut best: Option<(f64, FaceId)> = None;
    for f in faces_facing(topo, solid, n) {
        let wire = topo.wire(topo.face(f).unwrap().outer_wire()).unwrap();
        let edge = topo.edge(wire.edges()[0].edge()).unwrap();
        let p = topo.vertex(edge.start()).unwrap().point();
        let reach = n.dot(Vec3::new(p.x(), p.y(), p.z()));
        if best.is_none_or(|(r, _)| reach > r) {
            best = Some((reach, f));
        }
    }
    best.into_iter().map(|(_, f)| f).collect()
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    measure::solid_volume(topo, solid, DEFLECTION).expect("volume")
}

/// The defect, stated as material: drafting a side wall of a two-bore plate
/// must not change how much metal the bores removed.
#[test]
fn draft_keeps_both_bores_of_a_two_bore_plate() {
    let mut topo = Topology::new();
    let holed = plate(&mut topo, &[(20.0, 30.0), (60.0, 30.0)]);
    let plain = plate(&mut topo, &[]);

    assert_eq!(hole_count(&topo, holed), 4, "two through bores = 4 mouths");
    assert_valid(&topo, holed, "input plate");

    let wall = faces_facing(&topo, holed, Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(wall.len(), 1, "plate has one +X wall");
    let plain_wall = faces_facing(&topo, plain, Vec3::new(1.0, 0.0, 0.0));

    let holed_before = volume(&topo, holed);
    let plain_before = volume(&topo, plain);

    let drafted_holed = draft(
        &mut topo,
        holed,
        &wall,
        pull(),
        neutral(),
        ANGLE.to_radians(),
    )
    .expect("drafting a plain side wall of a drilled plate must succeed");
    let drafted_plain = draft(
        &mut topo,
        plain,
        &plain_wall,
        pull(),
        neutral(),
        ANGLE.to_radians(),
    )
    .expect("drafting the same wall of an undrilled plate must succeed");

    assert_valid(&topo, drafted_holed, "drafted drilled plate");
    assert_valid(&topo, drafted_plain, "drafted plain plate");

    assert_eq!(
        hole_count(&topo, drafted_holed),
        4,
        "drafting a wall must not fill the plate's bores"
    );
    assert_eq!(
        face_count(&topo, drafted_holed),
        face_count(&topo, holed),
        "drafting one wall must not change the face count"
    );

    // The bores are the only difference between the two bodies, before and
    // after. If draft filled them the drilled plate would gain that material.
    let bore_volume = 2.0 * PI * BORE_R * BORE_R * T;
    let margin = bore_volume * 0.02;
    assert!(
        (holed_before - (plain_before - bore_volume)).abs() < margin,
        "input plate volume {holed_before} does not match plain {plain_before} minus bores {bore_volume}"
    );
    let holed_after = volume(&topo, drafted_holed);
    let plain_after = volume(&topo, drafted_plain);
    assert!(
        (holed_after - (plain_after - bore_volume)).abs() < margin,
        "drafted drilled plate {holed_after} lost its bores: plain drafted is \
         {plain_after}, bores are {bore_volume}"
    );

    // The taper itself must still have happened: the +X wall leans out by
    // h*tan(angle), adding a wedge of D * T^2 * tan(angle) / 2.
    let wedge = D * T * T * ANGLE.to_radians().tan() / 2.0;
    assert!(
        (holed_after - holed_before - wedge).abs() < wedge * 0.02,
        "expected the drafted plate to gain the taper wedge {wedge}, got {}",
        holed_after - holed_before
    );
}

/// A blind pocket's mouth is an inner wire on a face draft does not move.
#[test]
fn draft_keeps_a_blind_pocket_mouth() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let cutter = make_box(&mut topo, 12.0, 12.0, 4.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(14.0, 14.0, 7.0)).unwrap();
    let pocketed = boolean(&mut topo, BooleanOp::Cut, blank, cutter).expect("mill pocket");

    assert_eq!(hole_count(&topo, pocketed), 1, "pocket mouth is one hole");
    assert_valid(&topo, pocketed, "pocketed block");
    let before = volume(&topo, pocketed);

    let wall = outer_face_facing(&topo, pocketed, Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(wall.len(), 1);

    let drafted = draft(
        &mut topo,
        pocketed,
        &wall,
        pull(),
        neutral(),
        ANGLE.to_radians(),
    )
    .expect("drafting a wall clear of the pocket must succeed");

    assert_valid(&topo, drafted, "drafted pocketed block");
    assert_eq!(
        hole_count(&topo, drafted),
        1,
        "drafting a wall must not fill the pocket"
    );
    let after = volume(&topo, drafted);
    let wedge = 40.0 * 10.0 * 10.0 * ANGLE.to_radians().tan() / 2.0;
    assert!(
        (after - before - wedge).abs() < wedge * 0.02,
        "expected the taper wedge {wedge}, got {}",
        after - before
    );
}

/// Drafting a face that itself carries a hole moves the hole's rim off the
/// bore that owns it. That is not implementable by moving outer-wire
/// vertices, so it must be refused by name — never approximated.
#[test]
fn drafting_a_face_that_carries_a_hole_is_refused() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, W, D, T).unwrap();
    // Bore straight through in Y, so the +/-Y walls carry the mouths and are
    // still square to the pull direction (i.e. otherwise draftable).
    let drill = make_cylinder(&mut topo, 1.5, D + 4.0).unwrap();
    transform_solid(
        &mut topo,
        drill,
        &(Mat4::translation(40.0, -2.0, 3.0) * Mat4::rotation_x(-PI / 2.0)),
    )
    .unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, blank, drill).expect("cross bore");
    assert_eq!(hole_count(&topo, body), 2, "cross bore opens two mouths");

    let holed_wall = faces_facing(&topo, body, Vec3::new(0.0, 1.0, 0.0));
    assert_eq!(holed_wall.len(), 1, "one +Y wall");

    let err = draft(
        &mut topo,
        body,
        &holed_wall,
        pull(),
        neutral(),
        ANGLE.to_radians(),
    )
    .expect_err("drafting a holed face must be refused, not approximated");

    match err {
        OperationsError::Unsupported { operation, reason } => {
            assert_eq!(operation, "draft");
            assert!(
                reason.contains("inner wire"),
                "refusal must name the inner wire, got: {reason}"
            );
        }
        other => panic!("expected a typed Unsupported refusal, got {other:?}"),
    }
}

/// The simplest possible draft has to produce a closed solid. It did not.
#[test]
fn drafting_a_cube_wall_closes_the_shell() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let wall = faces_facing(&topo, cube, Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(wall.len(), 1);

    let drafted = draft(
        &mut topo,
        cube,
        &wall,
        pull(),
        neutral(),
        ANGLE.to_radians(),
    )
    .expect("drafting one wall of a box must succeed");

    assert_valid(&topo, drafted, "drafted box");
    assert_eq!(face_count(&topo, drafted), 6);

    let wedge = 10.0 * 10.0 * 10.0 * ANGLE.to_radians().tan() / 2.0;
    let after = volume(&topo, drafted);
    assert!(
        (after - 1000.0 - wedge).abs() < wedge * 0.01,
        "expected 1000 + wedge {wedge}, got {after}"
    );
}

/// Drafting every wall at once moves shared corners under two drafted planes
/// at the same time; the result must still be a closed, valid solid with its
/// holes intact.
#[test]
fn drafting_all_four_walls_of_a_drilled_plate() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(20.0, 30.0), (60.0, 30.0)]);
    let before = volume(&topo, body);

    let mut walls = Vec::new();
    for n in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
    ] {
        walls.extend(faces_facing(&topo, body, n));
    }
    assert_eq!(walls.len(), 4);

    let drafted = draft(
        &mut topo,
        body,
        &walls,
        pull(),
        neutral(),
        ANGLE.to_radians(),
    )
    .expect("drafting all four walls must succeed");

    assert_valid(&topo, drafted, "fully drafted plate");
    assert_eq!(
        hole_count(&topo, drafted),
        4,
        "drafting every wall must not fill the bores"
    );
    assert!(
        volume(&topo, drafted) > before,
        "outward draft must add material"
    );
}
