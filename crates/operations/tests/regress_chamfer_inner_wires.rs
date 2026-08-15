//! Regression: `chamfer` must not fill the holes in the faces it trims, must
//! not lose a face's orientation, and must not return an invalid solid.
//!
//! `chamfer` rebuilt every face of the solid from a list of outer-wire vertex
//! positions and handed the assembler `inner_wires: vec![]` for all of them.
//! Any face carrying an inner wire — a through bore's mouth, a pocket opening
//! — therefore came back solid: the bore walls kept their rims but nothing
//! referenced them, and the chamfered body silently *gained* the material the
//! holes had removed. Chamfering the top perimeter of a four-bore plate
//! returned a plate with no bores and 256 free edges.
//!
//! The same rebuild passed curved faces through as a straight-chord polygon
//! with `reversed: false` hardcoded, so a bore wall — which a boolean cut
//! always leaves reversed — came back facing the wrong way and faceted into a
//! 64-gon, and the reversed planar walls of a milled pocket came back inside
//! out. Nothing checked any of it, because `chamfer` returned whatever the
//! assembler produced: it had no result gate at all.
//!
//! What must hold now:
//!   * a face the bevel does not move comes through verbatim — surface,
//!     curved edges, orientation and holes;
//!   * a face the bevel does move keeps its inner wires while its outer wire
//!     is rebuilt, or the operation is refused with a typed error;
//!   * every returned solid is closed, valid, encloses positive volume, and
//!     encloses *less* of it than the input, because a chamfer removes
//!     material.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::{chamfer, chamfer_asymmetric};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::{OperationsError, measure, validate};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const W: f64 = 80.0;
const D: f64 = 60.0;
const T: f64 = 6.0;
const BORE_R: f64 = 4.0;
/// Deflection for volume comparisons. Fine enough that a bore's faceting is
/// identical between input and output, so a filled hole cannot hide in it.
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

/// A 40 x 40 x 10 block with a 12 x 12 x 3 blind pocket whose low-X wall sits
/// at `pocket_x`.
fn pocketed_block(topo: &mut Topology, pocket_x: f64) -> SolidId {
    let blank = make_box(topo, 40.0, 40.0, 10.0).expect("blank");
    let cutter = make_box(topo, 12.0, 12.0, 4.0).expect("cutter");
    transform_solid(topo, cutter, &Mat4::translation(pocket_x, 14.0, 7.0)).expect("place cutter");
    boolean(topo, BooleanOp::Cut, blank, cutter).expect("mill pocket")
}

fn shell_faces(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    topo.shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .faces()
        .to_vec()
}

/// Total inner-wire count across the solid's outer shell.
fn hole_count(topo: &Topology, solid: SolidId) -> usize {
    shell_faces(topo, solid)
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .sum()
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    measure::solid_volume(topo, solid, DEFLECTION).expect("volume")
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

/// The single planar face whose *outward* normal is `n`, chosen as the one
/// furthest along `n` so a pocket's own wall is never mistaken for the block's.
fn outer_face_facing(topo: &Topology, solid: SolidId, n: Vec3) -> FaceId {
    let mut best: Option<(f64, FaceId)> = None;
    for f in shell_faces(topo, solid) {
        let face = topo.face(f).unwrap();
        if !matches!(face.surface(), FaceSurface::Plane { .. })
            || !face
                .effective_plane_normal()
                .is_some_and(|e| (e - n).length() < 1e-9)
        {
            continue;
        }
        let wire = topo.wire(face.outer_wire()).unwrap();
        let edge = topo.edge(wire.edges()[0].edge()).unwrap();
        let p = topo.vertex(edge.start()).unwrap().point();
        let reach = n.dot(Vec3::new(p.x(), p.y(), p.z()));
        if best.is_none_or(|(r, _)| reach > r) {
            best = Some((reach, f));
        }
    }
    best.expect("no face with that outward normal").1
}

/// The outer-wire edges of the face facing `n`.
fn perimeter_edges(topo: &Topology, solid: SolidId, n: Vec3) -> Vec<EdgeId> {
    let face = topo.face(outer_face_facing(topo, solid, n)).unwrap();
    topo.wire(face.outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::wire::OrientedEdge::edge)
        .collect()
}

/// How much material a `d`-chamfer of the whole top perimeter of a `w` x `dp`
/// box takes off.
///
/// The mitred bevels turn the top `d` of the body into a frustum whose section
/// at height `t` above the shoulder is `(w - 2t) x (dp - 2t)`, so
/// `removed = (w + dp) d^2 - 4 d^3 / 3`.
fn perimeter_chamfer_removal(w: f64, dp: f64, d: f64) -> f64 {
    (w + dp).mul_add(d * d, -(4.0 / 3.0) * d * d * d)
}

/// One row of the "every accepted chamfer" table.
struct Case {
    name: &'static str,
    build: fn(&mut Topology) -> SolidId,
    /// Outward normal of the face whose perimeter supplies the target edges.
    facing: Vec3,
    /// How many of that perimeter's edges to bevel.
    take: usize,
    d: f64,
}

fn expect_refusal(result: Result<SolidId, OperationsError>, needle: &str, what: &str) {
    match result {
        Err(OperationsError::Unsupported { operation, reason }) => {
            assert_eq!(operation, "chamfer", "{what}: wrong operation name");
            assert!(
                reason.contains(needle),
                "{what}: refusal should mention {needle:?}, got: {reason}"
            );
        }
        Err(other) => panic!("{what}: expected a typed Unsupported refusal, got {other:?}"),
        Ok(_) => panic!("{what}: must be refused, not approximated"),
    }
}

/// The defect, stated as material: chamfering the top perimeter of a four-bore
/// plate must not change how much metal the bores removed, and must take the
/// bevel's own material off rather than adding any.
#[test]
fn chamfer_keeps_all_four_bores_of_a_drilled_plate() {
    let mut topo = Topology::new();
    let body = plate(
        &mut topo,
        &[(20.0, 20.0), (60.0, 20.0), (20.0, 40.0), (60.0, 40.0)],
    );

    assert_eq!(hole_count(&topo, body), 8, "four through bores = 8 mouths");
    assert_valid(&topo, body, "input plate");
    let before = volume(&topo, body);

    let edges = perimeter_edges(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(edges.len(), 4, "the top face has a four-sided boundary");

    let d = 2.0;
    let result = chamfer(&mut topo, body, &edges, d).expect("chamfering a clear perimeter");

    assert_valid(&topo, result, "chamfered drilled plate");
    assert_eq!(
        hole_count(&topo, result),
        8,
        "chamfering the perimeter must not fill the plate's bores"
    );

    let after = volume(&topo, result);
    assert!(
        after < before,
        "a chamfer removes material: {after} is not less than {before}"
    );
    let expected = before - perimeter_chamfer_removal(W, D, d);
    assert!(
        (after - expected).abs() < 1e-6,
        "expected {expected} (input {before} less the mitred bevel ring), got {after}"
    );
}

/// The bores themselves must come back as the reversed cylinders they were,
/// with their rims still true circles — not as forward-facing 64-gons.
#[test]
fn chamfer_carries_a_reversed_curved_face_through_verbatim() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(40.0, 30.0)]);

    let bore_before = shell_faces(&topo, body)
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .expect("the plate has a bore wall");
    assert!(
        topo.face(bore_before).unwrap().is_reversed(),
        "a cut bore wall is a reversed cylinder; this test has nothing to prove otherwise"
    );
    let edges_before = topo
        .wire(topo.face(bore_before).unwrap().outer_wire())
        .unwrap()
        .edges()
        .len();

    // A bottom-face edge: nowhere near the bore, so the wall must not move.
    let edges = perimeter_edges(&topo, body, Vec3::new(0.0, 0.0, -1.0));
    let result = chamfer(&mut topo, body, &edges[..1], 1.0).expect("chamfering one bottom edge");

    assert_valid(&topo, result, "chamfered single-bore plate");
    assert_eq!(hole_count(&topo, result), 2, "the bore must survive");

    let walls: Vec<FaceId> = shell_faces(&topo, result)
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .collect();
    assert_eq!(walls.len(), 1, "the bore wall must still be one cylinder");
    let wall = topo.face(walls[0]).unwrap();
    assert!(
        wall.is_reversed(),
        "the bore wall lost its reversed flag, so it now faces into the metal"
    );
    let wire = topo.wire(wall.outer_wire()).unwrap();
    assert_eq!(
        wire.edges().len(),
        edges_before,
        "the bore wall was rebuilt from chords instead of carried through"
    );
    assert!(
        wire.edges()
            .iter()
            .any(|oe| matches!(topo.edge(oe.edge()).unwrap().curve(), EdgeCurve::Circle(_))),
        "the bore's rims must still be circles"
    );
}

/// A milled pocket's walls are reversed planar faces. Chamfering an edge of
/// the block must leave every one of them facing into the cavity.
#[test]
fn chamfer_preserves_reversed_planar_walls() {
    let mut topo = Topology::new();
    let body = pocketed_block(&mut topo, 14.0);

    let before: Vec<(Vec3, bool)> = shell_faces(&topo, body)
        .iter()
        .filter_map(|&f| {
            let face = topo.face(f).unwrap();
            face.effective_plane_normal()
                .map(|n| (n, face.is_reversed()))
        })
        .collect();
    let reversed_before = before.iter().filter(|(_, r)| *r).count();
    assert_eq!(reversed_before, 5, "a box pocket has five reversed walls");

    let edges = perimeter_edges(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    let result = chamfer(&mut topo, body, &edges[..1], 2.0).expect("chamfering one top edge");

    assert_valid(&topo, result, "chamfered pocketed block");
    let reversed_after = shell_faces(&topo, result)
        .iter()
        .filter(|&&f| topo.face(f).unwrap().is_reversed())
        .count();
    assert_eq!(
        reversed_after, reversed_before,
        "the pocket's walls were flattened to forward-facing faces"
    );

    // Every outward normal of the input must still be present in the output —
    // an inverted pocket wall would show up as its own negation.
    for (n, _) in &before {
        assert!(
            shell_faces(&topo, result).iter().any(|&f| topo
                .face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|m| (m - *n).length() < 1e-9)),
            "no face of the result faces {n:?} any more"
        );
    }
}

/// A blind pocket's mouth is an inner wire on the very face the bevel trims.
/// It has to survive the trim, and the trim has to cost exactly the prism it
/// cuts.
#[test]
fn chamfer_keeps_a_blind_pocket_mouth() {
    let mut topo = Topology::new();
    let body = pocketed_block(&mut topo, 14.0);

    assert_eq!(hole_count(&topo, body), 1, "the pocket mouth is one hole");
    assert_valid(&topo, body, "pocketed block");
    let before = volume(&topo, body);

    let edges = perimeter_edges(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    let d = 2.0;
    let result = chamfer(&mut topo, body, &edges[..1], d).expect("chamfering one top edge");

    assert_valid(&topo, result, "chamfered pocketed block");
    assert_eq!(
        hole_count(&topo, result),
        1,
        "chamfering a clear edge must not fill the pocket"
    );

    let after = volume(&topo, result);
    // One bevelled edge takes a right-triangular prism of legs d over the
    // whole 40 mm edge.
    let expected = before - 0.5 * d * d * 40.0;
    assert!(
        (after - expected).abs() < 1e-6,
        "expected {expected}, got {after}"
    );
    assert!(after < before, "a chamfer removes material");
}

/// The asymmetric entry point shares the rebuild, so it must keep holes too —
/// and take off exactly `d1*d2/2` per unit of edge.
#[test]
fn asymmetric_chamfer_keeps_a_blind_pocket_mouth() {
    let mut topo = Topology::new();
    let body = pocketed_block(&mut topo, 14.0);
    let before = volume(&topo, body);

    let edges = perimeter_edges(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    let (d1, d2) = (1.5, 3.0);
    let result =
        chamfer_asymmetric(&mut topo, body, &edges[..1], d1, d2).expect("asymmetric chamfer");

    assert_valid(&topo, result, "asymmetrically chamfered pocketed block");
    assert_eq!(hole_count(&topo, result), 1, "the pocket must survive");

    let after = volume(&topo, result);
    let expected = before - 0.5 * d1 * d2 * 40.0;
    assert!(
        (after - expected).abs() < 1e-6,
        "expected {expected}, got {after}"
    );
}

/// A bevel wide enough to reach a hole cannot keep that hole where it is. The
/// face would come back with a rim lying across its own boundary, so the
/// operation is refused by name instead.
#[test]
fn a_bevel_that_reaches_a_hole_is_refused() {
    // The pocket's low-X wall is at x = 4, so a setback of 4 lands exactly on
    // the rim and a setback of 6 cuts straight through it.
    for d in [4.0, 6.0] {
        let mut topo = Topology::new();
        let body = pocketed_block(&mut topo, 4.0);
        assert_eq!(hole_count(&topo, body), 1);

        // The top face's -X edge is the one the pocket sits behind.
        let top = outer_face_facing(&topo, body, Vec3::new(0.0, 0.0, 1.0));
        let target = topo
            .wire(topo.face(top).unwrap().outer_wire())
            .unwrap()
            .edges()
            .iter()
            .map(remus_topology::wire::OrientedEdge::edge)
            .find(|&e| {
                let edge = topo.edge(e).unwrap();
                let a = topo.vertex(edge.start()).unwrap().point();
                let b = topo.vertex(edge.end()).unwrap().point();
                a.x().abs() < 1e-9 && b.x().abs() < 1e-9
            })
            .expect("the top face has an edge on x = 0");

        expect_refusal(
            chamfer(&mut topo, body, &[target], d),
            "inner wire",
            &format!("a setback of {d} reaching the pocket"),
        );
    }
}

/// Chamfering a hole's rim would have to rebuild an inner wire, which this
/// algorithm never does. It must say so rather than quietly produce something.
#[test]
fn chamfering_a_bore_rim_is_refused() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(40.0, 30.0)]);

    let top = outer_face_facing(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    let rim = topo.face(top).unwrap().inner_wires()[0];
    let target = topo.wire(rim).unwrap().edges()[0].edge();

    expect_refusal(
        chamfer(&mut topo, body, &[target], 0.5),
        "rim",
        "chamfering a bore rim",
    );
}

/// A bevel is cut from the two planes meeting at the edge. Where one of them
/// is a cylinder there is no such plane, and the old code silently passed the
/// cylinder through unchanged while trimming its neighbour away from it.
#[test]
fn chamfering_an_edge_on_a_curved_face_is_refused() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 5.0, 12.0).expect("cylinder");

    let cap = outer_face_facing(&topo, cyl, Vec3::new(0.0, 0.0, 1.0));
    let target = topo
        .wire(topo.face(cap).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();

    expect_refusal(
        chamfer(&mut topo, cyl, &[target], 1.0),
        "curved",
        "chamfering a cylinder's cap rim",
    );
}

/// A corner where the bevel meets a curved face cannot be relocated by moving
/// outer-wire positions: the curved neighbour would be left behind and the
/// shell would open. Refuse it by name.
#[test]
fn a_corner_on_a_curved_face_is_refused() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, W, D, T).unwrap();
    // A drill straddling the -Y wall notches it, putting cylinder geometry on
    // the top face's own boundary.
    let drill = make_cylinder(&mut topo, BORE_R, T + 4.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(40.0, 0.0, -2.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, blank, drill).expect("notch the wall");

    let top = outer_face_facing(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    let target = topo
        .wire(topo.face(top).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::wire::OrientedEdge::edge)
        .find(|&e| {
            let edge = topo.edge(e).unwrap();
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line)
                && topo.vertex(edge.start()).unwrap().point().y().abs() < 1e-6
                && topo.vertex(edge.end()).unwrap().point().y().abs() < 1e-6
        })
        .expect("the notched top face still has a straight run along y = 0");

    // The material claim is "refused, not approximated". Which typed refusal
    // fires depends on how the notch cut resolved: with an exact
    // plane-parallel-to-axis intersection the notch stays a cylinder face and
    // the corner-on-curved-face check refuses with `Unsupported`; while that
    // exact arm is missing, the cut facets the notch into short line chords
    // and the setback range check refuses first with `InvalidInput` (the
    // 1.0 setback cannot fit the adjacent ~0.13 chord). Accept either — both
    // keep the shell closed; approximating or succeeding silently would not.
    match chamfer(&mut topo, body, &[target], 1.0) {
        Err(OperationsError::Unsupported { operation, reason }) => {
            assert_eq!(operation, "chamfer", "wrong operation name");
            assert!(
                reason.contains("curved"),
                "refusal should mention \"curved\", got: {reason}"
            );
        }
        Err(OperationsError::InvalidInput { reason }) => {
            assert!(
                reason.contains("does not fit"),
                "refusal should be the setback range check, got: {reason}"
            );
        }
        Err(other) => {
            panic!("chamfering into the notch: expected a typed refusal, got {other:?}")
        }
        Ok(_) => panic!("chamfering into the notch must be refused, not approximated"),
    }
}

/// Whatever the configuration, an accepted chamfer comes back valid, closed,
/// with every hole it started with, and smaller than it started.
#[test]
fn every_accepted_chamfer_removes_material_and_keeps_its_holes() {
    let cases = [
        Case {
            name: "plain box",
            build: |t| make_box(t, 10.0, 10.0, 10.0).unwrap(),
            facing: Vec3::new(0.0, 0.0, 1.0),
            take: 4,
            d: 1.0,
        },
        Case {
            name: "one bore",
            build: |t| plate(t, &[(40.0, 30.0)]),
            facing: Vec3::new(0.0, 0.0, 1.0),
            take: 4,
            d: 2.0,
        },
        Case {
            name: "four bores",
            build: |t| plate(t, &[(20.0, 20.0), (60.0, 20.0), (20.0, 40.0), (60.0, 40.0)]),
            facing: Vec3::new(0.0, 0.0, -1.0),
            take: 4,
            d: 1.5,
        },
        Case {
            name: "pocket",
            build: |t| pocketed_block(t, 14.0),
            facing: Vec3::new(0.0, 0.0, 1.0),
            take: 2,
            d: 2.0,
        },
    ];

    for case in cases {
        let Case {
            name,
            build,
            facing,
            take,
            d,
        } = case;
        let mut topo = Topology::new();
        let body = build(&mut topo);
        let holes = hole_count(&topo, body);
        let before = volume(&topo, body);

        let edges = perimeter_edges(&topo, body, facing);
        let result = chamfer(&mut topo, body, &edges[..take.min(edges.len())], d)
            .unwrap_or_else(|e| panic!("{name}: chamfer should succeed, got {e}"));

        assert_valid(&topo, result, name);
        assert_eq!(hole_count(&topo, result), holes, "{name}: lost a hole");
        let after = volume(&topo, result);
        assert!(after.is_finite() && after > 0.0, "{name}: encloses {after}");
        assert!(
            after < before,
            "{name}: a chamfer removes material, {after} is not less than {before}"
        );
    }
}

/// The full-perimeter closed form, on a body with nothing else going on, so a
/// regression in the mitre arithmetic cannot hide behind a bore.
#[test]
fn perimeter_chamfer_matches_the_closed_form() {
    for d in [0.5, 1.0, 2.0] {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 10.0, 8.0, 6.0).unwrap();
        let edges = perimeter_edges(&topo, body, Vec3::new(0.0, 0.0, 1.0));
        let result = chamfer(&mut topo, body, &edges, d).expect("perimeter chamfer");

        assert_valid(&topo, result, "chamfered box");
        let expected = 10.0f64.mul_add(8.0 * 6.0, -perimeter_chamfer_removal(10.0, 8.0, d));
        let after = volume(&topo, result);
        assert!(
            (after - expected).abs() < 1e-9,
            "d={d}: expected {expected}, got {after}"
        );
        let _ = Point3::new(0.0, 0.0, 0.0);
    }
}
