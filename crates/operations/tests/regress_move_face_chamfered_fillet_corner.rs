//! Moving a face of a rounded block whose foot is chamfered.
//!
//! The block is rounded along its top edges (r3), then its vertical edges
//! (r2), then its whole foot is chamfered (0.7), the order a user takes in
//! the app. The chamfer turns a cone round the foot of each vertical round.
//! Moving an end or a side face along its normal translates that end's blend
//! cluster rigidly; before the fix the cluster stopped at the chamfer (not
//! tangent to anything it bevels) and the move refused with "blend face …
//! reaches non-invariant cone support face …". The chamfer band at the moved
//! end now rides with the cluster, so the move is the exact prismatic
//! extension: same topology, the moved extent shifts by the distance, and the
//! volume changes by the distance times the block's rounded cross-section.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Vec3;
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade, planar_chamfer_with_origins};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{face_area, solid_bounding_box, solid_volume};
use remus_operations::primitives::make_box;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

const X: f64 = 30.0;
const Y: f64 = 18.0;
const Z: f64 = 24.0;
const TOP_ROUND: f64 = 3.0;
const SIDE_ROUND: f64 = 2.0;
const FOOT_CHAMFER: f64 = 0.7;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

fn edges_where(
    topo: &Topology,
    solid: SolidId,
    keep: impl Fn([f64; 3], [f64; 3]) -> bool,
) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let edge = topo.edge(edge).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            keep([a.x(), a.y(), a.z()], [b.x(), b.y(), b.z()])
        })
        .collect()
}

/// The block, rounded and chamfered one operation at a time as the app does
/// (the chamfer takes the planar bevel first and the general engine after).
fn chamfered_rounded_block(topo: &mut Topology) -> SolidId {
    let mut solid = make_box(topo, X, Y, Z).unwrap();
    let top = edges_where(topo, solid, |a, b| near(a[2], Z) && near(b[2], Z));
    assert_eq!(top.len(), 4);
    solid = fillet_cascade(topo, solid, &top, TOP_ROUND).unwrap().solid;
    let vertical = edges_where(topo, solid, |a, b| {
        near(a[0], b[0]) && near(a[1], b[1]) && (a[2] - b[2]).abs() > 5.0
    });
    assert_eq!(vertical.len(), 4);
    solid = fillet_cascade(topo, solid, &vertical, SIDE_ROUND)
        .unwrap()
        .solid;
    let foot = edges_where(topo, solid, |a, b| near(a[2], 0.0) && near(b[2], 0.0));
    assert!(!foot.is_empty());
    match planar_chamfer_with_origins(topo, solid, &foot, FOOT_CHAMFER) {
        Ok((chamfered, _)) => chamfered,
        Err(_) => {
            chamfer_v2(topo, solid, &foot, FOOT_CHAMFER, FOOT_CHAMFER)
                .unwrap()
                .solid
        }
    }
}

/// The largest planar face whose outward normal is `normal`: the block's
/// own face, not a wall of a cavity facing the same way.
fn face_facing(topo: &Topology, solid: SolidId, normal: Vec3) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(normal) > 1.0 - 1e-9)
        })
        .max_by(|&a, &b| {
            face_area(topo, a, 0.01)
                .unwrap()
                .total_cmp(&face_area(topo, b, 0.01).unwrap())
        })
        .unwrap_or_else(|| panic!("no face facing {normal:?}"))
}

/// The block's cross-section across its prismatic middle: the rectangle less
/// the two top rounds and the two foot chamfers that run along the move.
fn rounded_section(width: f64) -> f64 {
    let round_corner = (1.0 - PI / 4.0) * TOP_ROUND * TOP_ROUND;
    let chamfer_corner = FOOT_CHAMFER * FOOT_CHAMFER / 2.0;
    width.mul_add(Z, -2.0 * (round_corner + chamfer_corner))
}

fn check_move(normal: Vec3, distance: f64, section: f64) {
    let mut topo = Topology::new();
    let solid = chamfered_rounded_block(&mut topo);
    let face = face_facing(&topo, solid, normal);
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let volume = solid_volume(&topo, solid, 0.01).unwrap();
    let before = solid_bounding_box(&topo, solid).unwrap();

    let moved = move_faces(&mut topo, solid, &[face], distance)
        .unwrap_or_else(|error| panic!("move {normal:?} by {distance}: {error}"));

    assert!(validate_solid(&topo, moved).unwrap().is_valid());
    assert_eq!(solid_entity_counts(&topo, moved).unwrap(), counts);
    let mesh = tessellate_solid_with_tolerance(&topo, moved, 0.01, 0.08).unwrap();
    assert!(is_watertight(&mesh), "the moved block's mesh must close");

    // Only the moved axis changes extent, by exactly the distance.
    let after = solid_bounding_box(&topo, moved).unwrap();
    let grown = (after.max - after.min) - (before.max - before.min);
    let axis = Vec3::new(normal.x().abs(), normal.y().abs(), normal.z().abs());
    assert!(
        (grown - axis * distance).length() < 1e-9,
        "extent changed by {grown:?}, expected {:?}",
        axis * distance
    );

    let change = solid_volume(&topo, moved, 0.01).unwrap() - volume;
    let expected = distance * section;
    assert!(
        (change - expected).abs() <= 1e-3 * expected.abs(),
        "volume changed by {change}, expected {expected}"
    );
}

#[test]
fn end_face_pushes_in_past_its_chamfered_corners() {
    check_move(Vec3::new(1.0, 0.0, 0.0), -9.0, rounded_section(Y));
}

#[test]
fn end_face_pulls_out_past_its_chamfered_corners() {
    check_move(Vec3::new(1.0, 0.0, 0.0), 6.0, rounded_section(Y));
}

#[test]
fn side_face_pulls_out_past_its_chamfered_corners() {
    check_move(Vec3::new(0.0, 1.0, 0.0), 6.0, rounded_section(X));
}

#[test]
fn side_face_pushes_in_past_its_chamfered_corners() {
    check_move(Vec3::new(0.0, -1.0, 0.0), -3.0, rounded_section(X));
}

/// The block with a sealed 0.2 mm cube of a cavity at `corner`, inside the
/// foot just behind the end's chamfer.
fn block_with_cavity(topo: &mut Topology, corner: [f64; 3]) -> SolidId {
    let block = chamfered_rounded_block(topo);
    let cavity = make_box(topo, 0.2, 0.2, 0.2).unwrap();
    transform_solid(
        topo,
        cavity,
        &Mat4::translation(corner[0], corner[1], corner[2]),
    )
    .unwrap();
    boolean(topo, BooleanOp::Cut, block, cavity).unwrap()
}

/// The end face's own sweep passes above a cavity low in the foot, but the
/// chamfer riding with it does not: the move must refuse, not carry the
/// chamfer through the cavity (or past it, leaving its shell outside).
#[test]
fn riding_chamfer_refuses_to_sweep_through_a_cavity() {
    for (corner, distance) in [([24.55, 9.0, 0.3], -5.0), ([25.0, 9.0, 0.3], -9.0)] {
        let mut topo = Topology::new();
        let solid = block_with_cavity(&mut topo, corner);
        let face = face_facing(&topo, solid, Vec3::new(1.0, 0.0, 0.0));
        let counts = solid_entity_counts(&topo, solid).unwrap();
        let error = move_faces(&mut topo, solid, &[face], distance)
            .expect_err("the chamfer would sweep through the cavity");
        assert!(
            error
                .to_string()
                .contains("swept face reaches nonadjacent face"),
            "{corner:?} by {distance}: {error}"
        );
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
    }
}
