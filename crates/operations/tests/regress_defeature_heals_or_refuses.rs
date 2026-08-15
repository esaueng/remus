//! `defeature` must close the gap it opens, or refuse.
//!
//! Before this regression existed, `defeature` collected each *kept* face's
//! outer polygon and handed the set to a plane-set reassembly. That cannot
//! represent a concave body, and it silently discarded every inner wire. On
//! the chamfer, pocket, boss, L-notch and plate-face cases below it returned a
//! solid that `validate_solid` flagged with two errors (broken Euler
//! characteristic plus boundary edges — an open shell), and on a two-hole
//! plate it deleted the second hole as a side effect of filling the first.
//!
//! Every test here asserts one of two things: the heal is exact (validated
//! shell, exact volume), or the refusal is typed. There is no third outcome.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use brepkit_math::mat::Mat4;
use brepkit_math::vec::Point3;
use brepkit_operations::OperationsError;
use brepkit_operations::blend_ops::fillet_v2;
use brepkit_operations::boolean::{BooleanOp, boolean, face_polygon};
use brepkit_operations::defeature::defeature;
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_operations::validate::validate_solid;
use brepkit_topology::Topology;
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;

/// Assert the solid is structurally sound and has the expected volume.
fn assert_healed(topo: &Topology, solid: SolidId, expected_volume: f64) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "healed solid must validate, got {} error(s): {:?}",
        report.error_count(),
        report
            .issues
            .iter()
            .map(|i| i.description.clone())
            .collect::<Vec<_>>()
    );
    let volume = brepkit_operations::measure::solid_volume(topo, solid, 0.02).unwrap();
    let error = (volume - expected_volume).abs() / expected_volume.abs();
    assert!(
        error < 1e-9,
        "healed volume {volume} differs from {expected_volume} (relative {error:.3e})"
    );
}

/// Assert the operation declined with the typed refusal rather than returning
/// a solid or a generic error.
fn assert_refused(result: Result<SolidId, OperationsError>) -> String {
    match result {
        Ok(_) => panic!("defeature returned a solid where it cannot heal the gap"),
        Err(OperationsError::Unsupported { operation, reason }) => {
            assert_eq!(operation, "defeature");
            assert!(!reason.is_empty(), "refusal must name a reason");
            reason
        }
        Err(other) => panic!("expected a typed Unsupported refusal, got {other:?}"),
    }
}

fn face_centroid(topo: &Topology, face: FaceId) -> Point3 {
    let verts = face_polygon(topo, face).unwrap();
    #[allow(clippy::cast_precision_loss)]
    let n = verts.len() as f64;
    let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
    for v in &verts {
        x += v.x();
        y += v.y();
        z += v.z();
    }
    Point3::new(x / n, y / n, z / n)
}

fn faces_of(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    brepkit_topology::explorer::solid_faces(topo, solid).unwrap()
}

/// Faces whose stored plane is neither of the six axis-aligned directions —
/// the bevels a chamfer introduces.
fn oblique_faces(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    faces_of(topo, solid)
        .into_iter()
        .filter(|f| match topo.face(*f).unwrap().surface() {
            FaceSurface::Plane { normal, .. } => ![normal.x(), normal.y(), normal.z()]
                .iter()
                .any(|c| (c.abs() - 1.0).abs() < 1e-6),
            _ => false,
        })
        .collect()
}

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    brepkit_operations::transform::transform_solid(topo, solid, &Mat4::translation(x, y, z))
        .unwrap();
}

// ---------------------------------------------------------------------------
// Extend heal: the gap closes by growing the adjacent faces
// ---------------------------------------------------------------------------

#[test]
fn chamfer_removal_restores_the_sharp_edge() {
    let mut topo = Topology::new();
    let box_solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let top_edge = brepkit_topology::explorer::solid_edges(&topo, box_solid)
        .unwrap()
        .into_iter()
        .find(|e| {
            let edge = topo.edge(*e).unwrap();
            [edge.start(), edge.end()]
                .iter()
                .all(|v| (topo.vertex(*v).unwrap().point().z() - 10.0).abs() < 1e-9)
        })
        .unwrap();
    let chamfered =
        brepkit_operations::chamfer::chamfer(&mut topo, box_solid, &[top_edge], 2.0).unwrap();

    let bevel = oblique_faces(&topo, chamfered);
    assert_eq!(bevel.len(), 1, "one edge chamfered => one bevel face");

    let healed = defeature(&mut topo, chamfered, &bevel).unwrap();
    // Extending the top and front faces until they meet puts the box back.
    assert_healed(&topo, healed, 1000.0);
    assert_eq!(faces_of(&topo, healed).len(), 6);
}

#[test]
fn plane_plane_fillet_removal_restores_the_sharp_edge() {
    let mut topo = Topology::new();
    let box_solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = brepkit_topology::explorer::solid_edges(&topo, box_solid)
        .unwrap()
        .into_iter()
        .find(|edge_id| {
            let edge = topo.edge(*edge_id).unwrap();
            [edge.start(), edge.end()].iter().all(|vertex| {
                let point = topo.vertex(*vertex).unwrap().point();
                (point.x() - 10.0).abs() < 1e-9 && (point.y() - 10.0).abs() < 1e-9
            })
        })
        .unwrap();
    let filleted = fillet_v2(&mut topo, box_solid, &[edge], 2.0).unwrap().solid;
    let bands: Vec<FaceId> = faces_of(&topo, filleted)
        .into_iter()
        .filter(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder) if (cylinder.radius() - 2.0).abs() < 1e-9
            )
        })
        .collect();
    assert_eq!(bands.len(), 1);

    let healed = defeature(&mut topo, filleted, &bands).unwrap();
    assert_healed(&topo, healed, 1000.0);
    assert_eq!(faces_of(&topo, healed).len(), 6);
}

#[test]
fn corner_cut_removal_restores_the_corner() {
    // Chamfer all four top edges, then take all four bevels away at once. The
    // patch is four faces wide, so the corner each vertex moves to is only
    // determined after widening the search across the patch.
    let mut topo = Topology::new();
    let box_solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let top_edges: Vec<_> = brepkit_topology::explorer::solid_edges(&topo, box_solid)
        .unwrap()
        .into_iter()
        .filter(|e| {
            let edge = topo.edge(*e).unwrap();
            [edge.start(), edge.end()]
                .iter()
                .all(|v| (topo.vertex(*v).unwrap().point().z() - 10.0).abs() < 1e-9)
        })
        .collect();
    assert_eq!(top_edges.len(), 4);
    let chamfered =
        brepkit_operations::chamfer::chamfer(&mut topo, box_solid, &top_edges, 2.0).unwrap();

    let bevels = oblique_faces(&topo, chamfered);
    assert_eq!(bevels.len(), 4);

    let healed = defeature(&mut topo, chamfered, &bevels).unwrap();
    assert_healed(&topo, healed, 1000.0);
    assert_eq!(faces_of(&topo, healed).len(), 6);
}

#[test]
fn l_notch_removal_restores_the_full_block() {
    // 20x20x10 block with a corner notch cut out of it. The notch is three
    // faces; healing has to extend the top and the two far sides.
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 12.0, 12.0, 6.0).unwrap();
    translate(&mut topo, tool, 10.0, 10.0, 5.0);
    let notched = boolean(&mut topo, BooleanOp::Cut, base, tool).unwrap();

    // The notch walls face into the material (-X, -Y) and its floor faces
    // down; the block's own faces at x=20 / y=20 face outward.
    let notch: Vec<FaceId> = faces_of(&topo, notched)
        .into_iter()
        .filter(|f| match topo.face(*f).unwrap().surface() {
            FaceSurface::Plane { normal, d } => {
                (normal.x() < -0.5 && (*d + 10.0).abs() < 1e-9)
                    || (normal.y() < -0.5 && (*d + 10.0).abs() < 1e-9)
                    || (normal.z() < -0.5 && (*d + 5.0).abs() < 1e-9)
            }
            _ => false,
        })
        .collect();
    assert_eq!(notch.len(), 3, "two notch walls and a notch floor");

    let healed = defeature(&mut topo, notched, &notch).unwrap();
    assert_healed(&topo, healed, 4000.0);
    assert_eq!(faces_of(&topo, healed).len(), 6);
}

// ---------------------------------------------------------------------------
// Cap heal: the gap closes by deleting the openings the patch left behind
// ---------------------------------------------------------------------------

#[test]
fn blind_pocket_removal_fills_the_pocket() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 6.0, 6.0, 4.0).unwrap();
    translate(&mut topo, tool, 7.0, 7.0, 8.0);
    let pocketed = boolean(&mut topo, BooleanOp::Cut, base, tool).unwrap();

    let pocket: Vec<FaceId> = faces_of(&topo, pocketed)
        .into_iter()
        .filter(|f| {
            let c = face_centroid(&topo, *f);
            (6.5..13.5).contains(&c.x())
                && (6.5..13.5).contains(&c.y())
                && (0.1..9.9).contains(&c.z())
        })
        .collect();
    assert_eq!(pocket.len(), 5, "four pocket walls and a pocket floor");

    let healed = defeature(&mut topo, pocketed, &pocket).unwrap();
    assert_healed(&topo, healed, 4000.0);
    assert_eq!(faces_of(&topo, healed).len(), 6);
}

#[test]
fn boss_removal_flattens_the_base() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let boss = make_box(&mut topo, 6.0, 6.0, 4.0).unwrap();
    translate(&mut topo, boss, 7.0, 7.0, 10.0);
    let bossed = boolean(&mut topo, BooleanOp::Fuse, base, boss).unwrap();

    let boss_faces: Vec<FaceId> = faces_of(&topo, bossed)
        .into_iter()
        .filter(|f| face_centroid(&topo, *f).z() > 10.01)
        .collect();
    assert_eq!(boss_faces.len(), 5, "four boss walls and a boss top");

    let healed = defeature(&mut topo, bossed, &boss_faces).unwrap();
    assert_healed(&topo, healed, 4000.0);
    assert_eq!(faces_of(&topo, healed).len(), 6);
}

#[test]
fn through_hole_wall_removal_fills_the_bore() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 30.0, 30.0, 10.0).unwrap();
    let bore = make_cylinder(&mut topo, 4.0, 30.0).unwrap();
    translate(&mut topo, bore, 15.0, 15.0, -10.0);
    let holed = boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();

    let wall: Vec<FaceId> = faces_of(&topo, holed)
        .into_iter()
        .filter(|f| !topo.face(*f).unwrap().surface().is_planar())
        .collect();
    assert_eq!(wall.len(), 1, "one cylindrical bore wall");

    let healed = defeature(&mut topo, holed, &wall).unwrap();
    assert_healed(&topo, healed, 9000.0);
    assert_eq!(faces_of(&topo, healed).len(), 6);
}

#[test]
fn filling_one_hole_leaves_the_other_hole_alone() {
    // The old implementation kept only each face's OUTER polygon, so filling
    // one bore silently deleted every other hole in the plate as well. It also
    // refused outright as soon as any kept face was non-planar, which the
    // surviving bore wall is.
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 30.0, 30.0, 10.0).unwrap();
    let first = make_cylinder(&mut topo, 3.0, 30.0).unwrap();
    translate(&mut topo, first, 8.0, 15.0, -10.0);
    let one_hole = boolean(&mut topo, BooleanOp::Cut, plate, first).unwrap();
    let second = make_cylinder(&mut topo, 3.0, 30.0).unwrap();
    translate(&mut topo, second, 22.0, 15.0, -10.0);
    let two_holes = boolean(&mut topo, BooleanOp::Cut, one_hole, second).unwrap();

    let before = brepkit_operations::measure::solid_volume(&topo, two_holes, 0.02).unwrap();

    let near_wall: Vec<FaceId> = faces_of(&topo, two_holes)
        .into_iter()
        .filter(|f| {
            !topo.face(*f).unwrap().surface().is_planar() && face_centroid(&topo, *f).x() < 15.0
        })
        .collect();
    assert_eq!(near_wall.len(), 1);

    let healed = defeature(&mut topo, two_holes, &near_wall).unwrap();

    let report = validate_solid(&topo, healed).unwrap();
    assert!(report.is_valid(), "healed solid must validate");

    // The far bore must survive, wall and both rim loops.
    let remaining = faces_of(&topo, healed)
        .into_iter()
        .filter(|f| !topo.face(*f).unwrap().surface().is_planar())
        .count();
    assert_eq!(
        remaining, 1,
        "the untouched bore wall must still be a cylindrical face"
    );
    let rim_loops: usize = faces_of(&topo, healed)
        .iter()
        .map(|f| topo.face(*f).unwrap().inner_wires().len())
        .sum();
    assert_eq!(rim_loops, 2, "the surviving bore keeps a rim on each face");

    // Volume grows by exactly the filled bore and nothing else. The bound is
    // the tessellation error of the *surviving* bore, which is measured the
    // same way on both sides of the operation.
    let after = brepkit_operations::measure::solid_volume(&topo, healed, 0.02).unwrap();
    let filled = std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (after - before - filled).abs() < 0.5,
        "volume grew by {}, expected {filled}",
        after - before
    );
}

// ---------------------------------------------------------------------------
// Refusals: configurations with no exact heal
// ---------------------------------------------------------------------------

#[test]
fn removing_a_whole_plate_face_is_refused() {
    // The four side planes are pairwise parallel; extending them never closes
    // the top. The old code returned an open five-face shell.
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 30.0, 30.0, 10.0).unwrap();
    let top: Vec<FaceId> = faces_of(&topo, plate)
        .into_iter()
        .filter(|f| face_centroid(&topo, *f).z() > 9.9)
        .collect();
    assert_eq!(top.len(), 1);

    let reason = assert_refused(defeature(&mut topo, plate, &top));
    assert!(
        reason.contains("parallel"),
        "refusal should name the parallel neighbours, got {reason:?}"
    );
}

#[test]
fn removing_a_through_groove_is_refused_not_faked() {
    // Filling a groove that runs right through the block needs the two
    // coplanar top fragments to be merged, not extended. There is no corner to
    // extend into, so this must refuse rather than return a plausible shell.
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 4.0, 22.0, 5.0).unwrap();
    translate(&mut topo, tool, 8.0, -1.0, 6.0);
    let grooved = boolean(&mut topo, BooleanOp::Cut, base, tool).unwrap();

    let groove: Vec<FaceId> = faces_of(&topo, grooved)
        .into_iter()
        .filter(|f| match topo.face(*f).unwrap().surface() {
            FaceSurface::Plane { normal, d } => {
                (normal.x() < -0.5 && (*d + 8.0).abs() < 1e-9)
                    || (normal.x() > 0.5 && (*d - 12.0).abs() < 1e-9)
                    || (normal.z() < -0.5 && (*d + 6.0).abs() < 1e-9)
            }
            _ => false,
        })
        .collect();
    assert_eq!(groove.len(), 3, "two groove walls and a groove floor");

    assert_refused(defeature(&mut topo, grooved, &groove));
}

#[test]
fn removing_two_faces_of_a_box_is_refused() {
    let mut topo = Topology::new();
    let box_solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let faces = faces_of(&topo, box_solid);
    assert_refused(defeature(&mut topo, box_solid, &faces[0..2]));
}

#[test]
fn a_face_from_another_solid_is_rejected() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let b = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    let foreign = faces_of(&topo, b)[0];
    assert!(matches!(
        defeature(&mut topo, a, &[foreign]),
        Err(OperationsError::InvalidInput { .. })
    ));
}

#[test]
fn the_input_solid_survives_a_refusal() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 30.0, 30.0, 10.0).unwrap();
    let top: Vec<FaceId> = faces_of(&topo, plate)
        .into_iter()
        .filter(|f| face_centroid(&topo, *f).z() > 9.9)
        .collect();
    assert_refused(defeature(&mut topo, plate, &top));

    let report = validate_solid(&topo, plate).unwrap();
    assert!(report.is_valid(), "a refusal must not damage the input");
    assert_eq!(faces_of(&topo, plate).len(), 6);
}

#[test]
fn the_input_solid_survives_a_successful_heal() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 6.0, 6.0, 4.0).unwrap();
    translate(&mut topo, tool, 7.0, 7.0, 8.0);
    let pocketed = boolean(&mut topo, BooleanOp::Cut, base, tool).unwrap();
    let before = faces_of(&topo, pocketed).len();

    let pocket: Vec<FaceId> = faces_of(&topo, pocketed)
        .into_iter()
        .filter(|f| {
            let c = face_centroid(&topo, *f);
            (6.5..13.5).contains(&c.x())
                && (6.5..13.5).contains(&c.y())
                && (0.1..9.9).contains(&c.z())
        })
        .collect();
    let healed = defeature(&mut topo, pocketed, &pocket).unwrap();
    assert_ne!(healed, pocketed);

    assert_eq!(faces_of(&topo, pocketed).len(), before);
    assert!(validate_solid(&topo, pocketed).unwrap().is_valid());
    assert!(
        // 20*20*10 block less a 6*6*2 pocket (the tool overhangs the top face).
        (brepkit_operations::measure::solid_volume(&topo, pocketed, 0.02).unwrap() - 3928.0).abs()
            < 1e-9,
        "the pocketed input must keep its own volume"
    );
}
