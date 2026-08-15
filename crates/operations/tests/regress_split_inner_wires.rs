//! Regression: `split` must not fill the holes in the faces it carries over,
//! must not lose the orientation of the faces it does not trim, and must
//! return two halves that add back up to the body it was given.
//!
//! `split` rebuilt every face of the solid from a list of outer-wire vertex
//! positions and handed the assembler `inner_wires: vec![]` for all of them.
//! Any face carrying an inner wire — a through bore's mouth, a pocket opening
//! — came back solid: the bore walls kept their rims but nothing referenced
//! them. The same rebuild routed every planar face through `FaceSpec::Planar`,
//! which has no `reversed` field, so the reversed walls of a milled pocket
//! came back inside out. Splitting an 80x60x6 plate with four 8 mm bores
//! returned **0 bores on both halves**, both halves failing `validate_solid`
//! with 128 boundary edges each, and halves whose volumes summed to 29328
//! against an input of 27593 — 1735 mm3 of metal invented out of nothing.
//!
//! What must hold now:
//!   * a face wholly on one side of the plane comes through verbatim — exact
//!     surface, curved edges, orientation and every inner wire;
//!   * a face the plane trims keeps the inner wires that stay on its side;
//!   * the cut cap carries a hole wherever the plane crosses a bore, and that
//!     hole's rim is the same edge as the bore wall's new rim;
//!   * both halves are valid solids of positive volume, and their volumes sum
//!     to the input's;
//!   * anything that cannot be built exactly is refused by name.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::split::{SplitResult, split};
use remus_operations::transform::transform_solid;
use remus_operations::{OperationsError, measure, validate};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const W: f64 = 80.0;
const D: f64 = 60.0;
const T: f64 = 6.0;
const BORE_R: f64 = 4.0;
/// Deflection for volume comparisons. `solid_volume` integrates analytic
/// surfaces exactly, so this only bounds the NURBS-free fallback paths.
const DEFLECTION: f64 = 0.01;

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

/// The four bores of the audit's plate, all clear of the mid-plane in X.
fn four_bores() -> [(f64, f64); 4] {
    [(20.0, 20.0), (20.0, 40.0), (60.0, 20.0), (60.0, 40.0)]
}

/// Total inner-wire count across the solid's outer shell.
fn hole_count(topo: &Topology, solid: SolidId) -> usize {
    faces(topo, solid)
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .sum()
}

fn faces(topo: &Topology, solid: SolidId) -> Vec<remus_topology::face::FaceId> {
    topo.shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .faces()
        .to_vec()
}

/// Faces carrying a genuine cylindrical surface (not a faceted stand-in).
fn cylinder_count(topo: &Topology, solid: SolidId) -> usize {
    faces(topo, solid)
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count()
}

fn reversed_count(topo: &Topology, solid: SolidId) -> usize {
    faces(topo, solid)
        .iter()
        .filter(|&&f| topo.face(f).unwrap().is_reversed())
        .count()
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    measure::solid_volume(topo, solid, DEFLECTION).expect("volume")
}

fn errors(topo: &Topology, solid: SolidId) -> Vec<String> {
    validate::validate_solid(topo, solid)
        .expect("validate")
        .issues
        .iter()
        .filter(|i| i.severity == validate::Severity::Error)
        .map(|i| i.description.clone())
        .collect()
}

fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let errs = errors(topo, solid);
    assert!(errs.is_empty(), "{what} is not a valid solid: {errs:?}");
}

/// Both halves valid, both positive, and the arithmetic closed: the two
/// halves are a decomposition of the input, so their volumes must sum to it.
/// This is the strongest single check available on a split.
fn assert_halves_add_up(topo: &Topology, result: &SplitResult, input_volume: f64, what: &str) {
    assert_valid(topo, result.positive, &format!("{what}: positive half"));
    assert_valid(topo, result.negative, &format!("{what}: negative half"));

    let pos = volume(topo, result.positive);
    let neg = volume(topo, result.negative);
    assert!(pos > 0.0 && pos.is_finite(), "{what}: positive half {pos}");
    assert!(neg > 0.0 && neg.is_finite(), "{what}: negative half {neg}");
    // Tessellation of a curved face is deterministic and the halves reuse the
    // input's own surfaces, so the sum is exact up to accumulated rounding.
    let slack = input_volume.abs() * 1e-9;
    assert!(
        (pos + neg - input_volume).abs() <= slack,
        "{what}: halves {pos} + {neg} = {} do not sum to the input {input_volume}",
        pos + neg
    );
}

/// The audit's measurement: an 80x60x6 plate with four 8 mm through bores,
/// split by a plane that misses every bore. It returned 0 bores on both
/// halves, both halves invalid.
#[test]
fn split_clear_of_the_bores_keeps_all_four() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &four_bores());
    assert_eq!(hole_count(&topo, body), 8, "four through bores = 8 mouths");
    assert_valid(&topo, body, "input plate");
    let before = volume(&topo, body);

    let result = split(
        &mut topo,
        body,
        Point3::new(40.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("a plane clear of every bore must split the plate");

    assert_halves_add_up(&topo, &result, before, "plate split clear of the bores");

    // Two bores each side, each still opening on both faces of its half.
    assert_eq!(
        hole_count(&topo, result.positive),
        4,
        "the +X half keeps its two bores"
    );
    assert_eq!(
        hole_count(&topo, result.negative),
        4,
        "the -X half keeps its two bores"
    );
    assert_eq!(
        cylinder_count(&topo, result.positive),
        2,
        "the +X half's bore walls must still be cylinders"
    );
    assert_eq!(cylinder_count(&topo, result.negative), 2);

    // Each half is a 40x60x6 plate less two bores.
    let bore = PI * BORE_R * BORE_R * T;
    let expect = 40.0 * D * T - 2.0 * bore;
    for (half, label) in [(result.positive, "+X"), (result.negative, "-X")] {
        let v = volume(&topo, half);
        assert!(
            (v - expect).abs() < expect * 1e-9,
            "{label} half is {v}, expected {expect}"
        );
    }
}

/// The plane crosses every bore, square to their axes: the cut cap must gain
/// four holes of its own, each half's bore walls must survive as cylinders,
/// and each hole's rim must be shared between the cap and the wall.
#[test]
fn split_square_through_the_bores_caps_each_one() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &four_bores());
    let before = volume(&topo, body);

    let result = split(
        &mut topo,
        body,
        Point3::new(0.0, 0.0, T / 2.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .expect("a plane square through the bores must split the plate");

    assert_halves_add_up(&topo, &result, before, "plate split through the bores");

    for (half, label) in [(result.positive, "+Z"), (result.negative, "-Z")] {
        assert_eq!(
            hole_count(&topo, half),
            8,
            "{label} half: four bores opening on two faces each"
        );
        assert_eq!(
            cylinder_count(&topo, half),
            4,
            "{label} half: four bore walls, still cylinders"
        );
        let expect = W * D * (T / 2.0) - 4.0 * PI * BORE_R * BORE_R * (T / 2.0);
        let v = volume(&topo, half);
        assert!(
            (v - expect).abs() < expect * 1e-9,
            "{label} half is {v}, expected {expect}"
        );
    }
}

/// A milled pocket's walls are reversed faces and its mouth is an inner wire.
/// Splitting well below the pocket must move both through untouched: the
/// old code routed every planar face through `FaceSpec::Planar`, which has no
/// `reversed` field, so the pocket came back inside out with its mouth filled.
#[test]
fn split_below_a_pocket_keeps_its_mouth_and_its_orientation() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let cutter = make_box(&mut topo, 12.0, 12.0, 4.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(14.0, 14.0, 7.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, blank, cutter).expect("mill pocket");

    assert_eq!(hole_count(&topo, body), 1, "the pocket mouth is one hole");
    assert_eq!(
        reversed_count(&topo, body),
        5,
        "pocket walls plus its floor"
    );
    let before = volume(&topo, body);
    assert!((before - (16000.0 - 432.0)).abs() < 1e-9);

    let result = split(
        &mut topo,
        body,
        Point3::new(0.0, 0.0, 3.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .expect("a plane below the pocket must split the block");

    assert_halves_add_up(&topo, &result, before, "pocketed block");

    assert_eq!(
        hole_count(&topo, result.positive),
        1,
        "the upper half keeps the pocket mouth"
    );
    assert_eq!(
        reversed_count(&topo, result.positive),
        5,
        "the pocket's five reversed faces must stay reversed"
    );
    let pos = volume(&topo, result.positive);
    assert!(
        (pos - (40.0 * 40.0 * 7.0 - 432.0)).abs() < 1e-9,
        "upper half is {pos}, expected {}",
        40.0 * 40.0 * 7.0 - 432.0
    );
    let neg = volume(&topo, result.negative);
    assert!((neg - 4800.0).abs() < 1e-9, "lower half is {neg}");
}

/// Assert a typed `split` refusal whose reason names `needle`.
fn assert_refused(err: OperationsError, needle: &str) {
    match err {
        OperationsError::Unsupported { operation, reason } => {
            assert_eq!(operation, "split");
            assert!(
                reason.contains(needle),
                "the refusal must name {needle:?}, got: {reason}"
            );
        }
        other => panic!("expected a typed Unsupported refusal, got {other:?}"),
    }
}

/// A plane that runs down a bore has to cut that bore's rim circles into
/// semicircular arcs and graft them onto the plate's own outer boundary.
/// There is no exact construction for that here, so it must be refused by
/// name — never handed back as a chord across the arc.
#[test]
fn split_lengthwise_through_a_bore_is_refused() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(20.0, 30.0)]);

    let err = split(
        &mut topo,
        body,
        Point3::new(20.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect_err("splitting down the axis of a bore must be refused");
    assert_refused(err, "curved edge");
}

/// A rectangular pocket's mouth is bounded by straight edges, so the plane can
/// cut them — but the hole would stop being a hole and become a notch in the
/// face's own boundary. That is the inner-wire case, and it too is refused.
#[test]
fn split_through_a_rectangular_pocket_mouth_is_refused() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let cutter = make_box(&mut topo, 12.0, 12.0, 4.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(14.0, 14.0, 7.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, blank, cutter).expect("mill pocket");

    let err = split(
        &mut topo,
        body,
        Point3::new(20.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect_err("splitting through a pocket mouth must be refused");
    assert_refused(err, "inner wire");
}

/// Splitting a solid cylinder square to its axis is the case where the cut
/// cap's *outer* wire is a minted circle rather than a chain of segments.
#[test]
fn split_a_cylinder_square_to_its_axis() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let before = volume(&topo, cyl);

    let result = split(
        &mut topo,
        cyl,
        Point3::new(0.0, 0.0, 4.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .expect("a cylinder must split square to its axis");

    assert_halves_add_up(&topo, &result, before, "cylinder");
    assert_eq!(cylinder_count(&topo, result.positive), 1);
    assert_eq!(cylinder_count(&topo, result.negative), 1);

    let pos = volume(&topo, result.positive);
    assert!(
        (pos - PI * 25.0 * 6.0).abs() < 1e-9,
        "upper piece is {pos}, expected {}",
        PI * 25.0 * 6.0
    );
}

/// A plane that misses the body is not a split, and must not come back as one
/// well-formed half and one empty one.
#[test]
fn split_by_a_plane_that_misses_the_body_fails() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(20.0, 30.0)]);
    assert!(
        split(
            &mut topo,
            body,
            Point3::new(0.0, 0.0, 50.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .is_err(),
        "a plane clear of the body must not report a split"
    );
}

/// Every face of the plate is either wholly on one side or trimmed; nothing
/// may be dropped. Counted as faces, the two halves account for the input.
#[test]
fn split_accounts_for_every_face_of_the_plate() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &four_bores());
    // 6 box faces + 4 bore walls.
    assert_eq!(faces(&topo, body).len(), 10);

    let result = split(
        &mut topo,
        body,
        Point3::new(40.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("split");

    // Each half: top, bottom, three walls, one cap, two bore walls.
    for half in [result.positive, result.negative] {
        assert_eq!(faces(&topo, half).len(), 8);
    }
}

/// A refusal must leave the body exactly as it was — same faces, same holes,
/// same volume — so a caller can try a different plane.
#[test]
fn a_refused_split_leaves_the_body_intact() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(20.0, 30.0)]);
    let before_faces = faces(&topo, body).len();
    let before_holes = hole_count(&topo, body);
    let before_volume = volume(&topo, body);

    assert!(
        split(
            &mut topo,
            body,
            Point3::new(20.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .is_err()
    );

    assert_valid(&topo, body, "the body after a refused split");
    assert_eq!(faces(&topo, body).len(), before_faces);
    assert_eq!(hole_count(&topo, body), before_holes);
    assert!((volume(&topo, body) - before_volume).abs() < 1e-12);
}
