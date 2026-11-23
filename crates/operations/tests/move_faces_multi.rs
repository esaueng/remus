//! B287 acceptance: several planar faces moved at once, each along its own
//! outward normal (`remus_operations::multi_move`), checked against the
//! same faces moved one coplanar group (or one face) at a time.

#![allow(
    clippy::disallowed_methods,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation
)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_offset::OffsetError;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::journal_ops::move_faces_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::multi_move::{
    move_faces_multi, move_faces_multi_journaled, move_faces_multi_with_evolution,
};
use remus_operations::prepared_move::PreparedFaceMove;
use remus_operations::primitives::make_box;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::{
    face_vertices, solid_edges, solid_entity_counts, solid_faces, solid_vertices,
};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

fn placed_box(topo: &mut Topology, min: [f64; 3], max: [f64; 3]) -> SolidId {
    let solid = make_box(topo, max[0] - min[0], max[1] - min[1], max[2] - min[2]).expect("box");
    transform_solid(topo, solid, &Mat4::translation(min[0], min[1], min[2])).expect("place box");
    solid
}

/// A 30 × 20 × 10 block with a 10-wide, 4-deep slot through it along y:
/// tops at z = 10 beside x ∈ [10, 20], slot floor at z = 6.
fn slotted_block(topo: &mut Topology) -> SolidId {
    let block = placed_box(topo, [0.0, 0.0, 0.0], [30.0, 20.0, 10.0]);
    let slot = placed_box(topo, [10.0, -1.0, 6.0], [20.0, 21.0, 11.0]);
    boolean(topo, BooleanOp::Cut, block, slot).expect("slot cut")
}

/// A 40 × 30 × 10 block with a 20 × 10 × 6 pocket in its top.
fn pocketed_block(topo: &mut Topology) -> SolidId {
    let block = placed_box(topo, [0.0, 0.0, 0.0], [40.0, 30.0, 10.0]);
    let pocket = placed_box(topo, [10.0, 10.0, 4.0], [30.0, 20.0, 11.0]);
    boolean(topo, BooleanOp::Cut, block, pocket).expect("pocket cut")
}

/// A 40 × 40 × 5 plate with a 10 × 10 × 10 boss on it.
fn bossed_plate(topo: &mut Topology) -> SolidId {
    let plate = placed_box(topo, [0.0, 0.0, 0.0], [40.0, 40.0, 5.0]);
    let boss = placed_box(topo, [15.0, 15.0, 5.0], [25.0, 25.0, 15.0]);
    boolean(topo, BooleanOp::Fuse, plate, boss).expect("boss fuse")
}

fn face_centroid(topo: &Topology, face: FaceId) -> Point3 {
    let vertices = face_vertices(topo, face).expect("face vertices");
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    for &vertex in &vertices {
        sum += topo.vertex(vertex).expect("vertex").point() - Point3::new(0.0, 0.0, 0.0);
    }
    Point3::new(0.0, 0.0, 0.0) + sum * (1.0 / vertices.len() as f64)
}

/// The planar faces of `solid` whose outward normal is `normal` and that
/// lie on the plane through `on`, in face order.
fn faces_on(topo: &Topology, solid: SolidId, normal: Vec3, on: Point3) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .expect("faces")
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .expect("face")
                .effective_plane_normal()
                .is_some_and(|actual| actual.dot(normal) > 1.0 - 1e-12)
                && (face_centroid(topo, face) - on).dot(normal).abs() < 1e-9
        })
        .collect()
}

/// The planar face with outward `normal` whose centroid is nearest `near`.
fn face_near(topo: &Topology, solid: SolidId, normal: Vec3, near: Point3) -> FaceId {
    solid_faces(topo, solid)
        .expect("faces")
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .expect("face")
                .effective_plane_normal()
                .is_some_and(|actual| actual.dot(normal) > 1.0 - 1e-12)
        })
        .min_by(|&a, &b| {
            (face_centroid(topo, a) - near)
                .length()
                .total_cmp(&(face_centroid(topo, b) - near).length())
        })
        .expect("a face with that normal")
}

fn x() -> Vec3 {
    Vec3::new(1.0, 0.0, 0.0)
}
fn y() -> Vec3 {
    Vec3::new(0.0, 1.0, 0.0)
}
fn z() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

/// Rounded geometric fingerprint: volume, F/E/V, sorted vertex positions
/// and sorted edge lengths.
#[derive(Debug, PartialEq)]
struct Fingerprint {
    counts: (usize, usize, usize),
    vertices: Vec<[i64; 3]>,
    edge_lengths: Vec<i64>,
}

fn round(value: f64) -> i64 {
    (value * 1e7).round() as i64
}

fn fingerprint(topo: &Topology, solid: SolidId) -> Fingerprint {
    let mut vertices: Vec<_> = solid_vertices(topo, solid)
        .expect("vertices")
        .into_iter()
        .map(|vertex| {
            let point = topo.vertex(vertex).expect("vertex").point();
            [round(point.x()), round(point.y()), round(point.z())]
        })
        .collect();
    vertices.sort_unstable();
    let mut edge_lengths: Vec<_> = solid_edges(topo, solid)
        .expect("edges")
        .into_iter()
        .map(|edge| {
            let edge = topo.edge(edge).expect("edge");
            let start = topo.vertex(edge.start()).expect("start").point();
            let end = topo.vertex(edge.end()).expect("end").point();
            round((end - start).length())
        })
        .collect();
    edge_lengths.sort_unstable();
    Fingerprint {
        counts: solid_entity_counts(topo, solid).expect("counts"),
        vertices,
        edge_lengths,
    }
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, 0.01).expect("volume")
}

fn assert_verified(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).expect("strict validation");
    assert!(report.is_valid(), "strict validation: {:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).expect("tessellation");
    assert!(is_watertight(&mesh), "mesh is not watertight");
}

/// Moves `faces` together, checks the result against `chain` (the same
/// faces moved a group at a time, each step re-finding its faces on the
/// intermediate solid), the closed-form `expected` volume and the journal.
fn assert_matches_chain(
    build: fn(&mut Topology) -> SolidId,
    select: impl Fn(&Topology, SolidId) -> Vec<FaceId>,
    distance: f64,
    chain: impl Fn(&mut Topology, SolidId) -> SolidId,
    expected: f64,
) {
    let mut topo = Topology::new();
    let source = build(&mut topo);
    let faces = select(&topo, source);
    let source_counts = solid_entity_counts(&topo, source).expect("counts");
    let multi = move_faces_multi(&mut topo, source, &faces, &[distance])
        .unwrap_or_else(|error| panic!("multi move by {distance}: {error}"));
    assert_verified(&topo, multi);
    let actual = volume(&topo, multi);
    assert!(
        (actual - expected).abs() <= 1e-6 * expected.abs().max(1.0),
        "volume {actual} != closed form {expected} at {distance}"
    );
    assert_eq!(
        solid_entity_counts(&topo, multi).expect("counts"),
        source_counts
    );

    let mut chained_topo = Topology::new();
    let chained_source = build(&mut chained_topo);
    let chained = chain(&mut chained_topo, chained_source);
    assert!((volume(&chained_topo, chained) - actual).abs() <= 1e-6 * actual.abs().max(1.0));
    assert_eq!(
        fingerprint(&topo, multi),
        fingerprint(&chained_topo, chained),
        "multi move by {distance} differs from the chained moves"
    );

    // Journaled: one entry, exact and total face map, same content.
    let mut journal_topo = Topology::new();
    let journal_source = build(&mut journal_topo);
    let journal_faces = select(&journal_topo, journal_source);
    let journaled = move_faces_multi_journaled(
        &mut journal_topo,
        journal_source,
        &journal_faces,
        &[distance],
    )
    .expect("journaled multi move");
    let source_faces = solid_faces(&journal_topo, journal_source).expect("source faces");
    let result_faces = solid_faces(&journal_topo, journaled.solid).expect("result faces");
    let map = &journaled.map;
    assert!(map.unresolved.is_empty() && map.generated.is_empty() && map.deleted.is_empty());
    assert_eq!(map.modified.len(), source_faces.len());
    let mut targets: Vec<_> = map.modified.values().flatten().copied().collect();
    targets.sort_unstable();
    let mut expected_targets: Vec<_> = result_faces.iter().map(|face| face.index()).collect();
    expected_targets.sort_unstable();
    assert_eq!(
        targets, expected_targets,
        "face map is not total and one-to-one"
    );
    assert!(map.modified.values().all(|outputs| outputs.len() == 1));
    assert_eq!(
        fingerprint(&journal_topo, journaled.solid),
        fingerprint(&topo, multi)
    );

    // Prepared: the same content as the unprepared call, across restores.
    let mut prepared_topo = Topology::new();
    let prepared_source = build(&mut prepared_topo);
    let prepared_faces = select(&prepared_topo, prepared_source);
    let checkpoint = prepared_topo.clone();
    let mut prepared =
        PreparedFaceMove::prepare_multi(&prepared_topo, prepared_source, &prepared_faces, &[])
            .expect("prepare");
    for _ in 0..2 {
        prepared_topo.restore_preserving_handle_slots(&checkpoint);
        assert!(prepared.is_current(&prepared_topo));
        let previewed = prepared
            .move_faces_journaled(&mut prepared_topo, distance)
            .expect("prepared journaled multi move");
        assert_eq!(
            fingerprint(&prepared_topo, previewed.solid),
            fingerprint(&topo, multi)
        );
    }
}

fn slotted_tops_and_floor(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    let mut faces = faces_on(topo, solid, z(), Point3::new(0.0, 0.0, 10.0));
    assert_eq!(faces.len(), 2, "two tops beside the slot");
    faces.extend(faces_on(topo, solid, z(), Point3::new(0.0, 0.0, 6.0)));
    assert_eq!(faces.len(), 3, "and the slot floor");
    faces
}

#[test]
fn stepped_tops_and_slot_floor_move_together() {
    // Positive, negative, and past the 4-unit step either way.
    for distance in [1.5, -1.5, 5.0, -5.0] {
        let chain = |topo: &mut Topology, solid: SolidId| {
            let tops = faces_on(topo, solid, z(), Point3::new(0.0, 0.0, 10.0));
            let floor = faces_on(topo, solid, z(), Point3::new(0.0, 0.0, 6.0));
            // The leading plane in the direction of travel moves first, as
            // a chain must: the other order folds the slot walls.
            if distance > 0.0 {
                let solid = move_faces(topo, solid, &tops, distance).expect("tops");
                let floor = faces_on(topo, solid, z(), Point3::new(0.0, 0.0, 6.0));
                move_faces(topo, solid, &floor, distance).expect("floor")
            } else {
                let solid = move_faces(topo, solid, &floor, distance).expect("floor");
                let tops = faces_on(topo, solid, z(), Point3::new(0.0, 0.0, 10.0));
                move_faces(topo, solid, &tops, distance).expect("tops")
            }
        };
        // The 30 × 20 footprint rises by `distance` everywhere.
        let expected = 20.0f64.mul_add(-10.0 * 4.0, 30.0 * 20.0 * (10.0 + distance));
        assert_matches_chain(
            slotted_block,
            slotted_tops_and_floor,
            distance,
            chain,
            expected,
        );
    }
}

fn pocket_walls(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    vec![
        face_near(topo, solid, x(), Point3::new(10.0, 15.0, 7.0)),
        face_near(topo, solid, -x(), Point3::new(30.0, 15.0, 7.0)),
        face_near(topo, solid, y(), Point3::new(20.0, 10.0, 7.0)),
        face_near(topo, solid, -y(), Point3::new(20.0, 20.0, 7.0)),
    ]
}

fn chain_pocket_walls(topo: &mut Topology, solid: SolidId, distance: f64) -> SolidId {
    let mut solid = solid;
    let inset = distance;
    for (normal, near) in [
        (x(), Point3::new(10.0, 15.0, 7.0)),
        (-x(), Point3::new(30.0, 15.0, 7.0)),
        (y(), Point3::new(20.0, 10.0, 7.0)),
        (-y(), Point3::new(20.0, 20.0, 7.0)),
    ] {
        let face = face_near(topo, solid, normal, near + normal * inset);
        solid = move_faces(topo, solid, &[face], distance).expect("single wall");
    }
    solid
}

#[test]
fn pocket_walls_move_in_together() {
    for distance in [1.0, 2.5, 4.0] {
        // The 20 × 10 × 6 pocket shrinks by 2·distance each way.
        let pocket = (20.0 - 2.0 * distance) * (10.0 - 2.0 * distance) * 6.0;
        assert_matches_chain(
            pocketed_block,
            pocket_walls,
            distance,
            |topo, solid| chain_pocket_walls(topo, solid, distance),
            40.0f64.mul_add(30.0 * 10.0, -pocket),
        );
    }
}

#[test]
fn pocket_walls_move_out_together() {
    for distance in [-1.0, -3.0, -6.0] {
        let pocket = (20.0 - 2.0 * distance) * (10.0 - 2.0 * distance) * 6.0;
        assert_matches_chain(
            pocketed_block,
            pocket_walls,
            distance,
            |topo, solid| chain_pocket_walls(topo, solid, distance),
            40.0f64.mul_add(30.0 * 10.0, -pocket),
        );
    }
}

fn boss_walls(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    vec![
        face_near(topo, solid, -x(), Point3::new(15.0, 20.0, 10.0)),
        face_near(topo, solid, x(), Point3::new(25.0, 20.0, 10.0)),
        face_near(topo, solid, -y(), Point3::new(20.0, 15.0, 10.0)),
        face_near(topo, solid, y(), Point3::new(20.0, 25.0, 10.0)),
    ]
}

#[test]
fn boss_walls_thicken_and_thin_together() {
    for distance in [2.0, 5.0, -2.0, -4.0] {
        let chain = |topo: &mut Topology, solid: SolidId| {
            let mut solid = solid;
            for (normal, near) in [
                (-x(), Point3::new(15.0, 20.0, 10.0)),
                (x(), Point3::new(25.0, 20.0, 10.0)),
                (-y(), Point3::new(20.0, 15.0, 10.0)),
                (y(), Point3::new(20.0, 25.0, 10.0)),
            ] {
                let face = face_near(topo, solid, normal, near + normal * distance);
                solid = move_faces(topo, solid, &[face], distance).expect("single boss wall");
            }
            solid
        };
        let side = 2.0f64.mul_add(distance, 10.0);
        assert_matches_chain(
            bossed_plate,
            boss_walls,
            distance,
            chain,
            (side * side).mul_add(10.0, 40.0 * 40.0 * 5.0),
        );
    }
}

#[test]
fn per_face_distances_move_each_face_its_own_way() {
    let mut topo = Topology::new();
    let source = pocketed_block(&mut topo);
    let walls = pocket_walls(&topo, source);
    let distances = [1.0, -2.0, 0.5, -1.5];
    let moved = move_faces_multi_with_evolution(&mut topo, source, &walls, &distances)
        .expect("per-face multi move");
    assert_verified(&topo, moved.solid);
    // x walls at 11 and 32, y walls at 10.5 and 21.5.
    let pocket = (32.0 - 11.0) * (21.5 - 10.5) * 6.0;
    let actual = volume(&topo, moved.solid);
    let expected = 40.0f64.mul_add(30.0 * 10.0, -pocket);
    assert!(
        (actual - expected).abs() <= 1e-6 * expected,
        "{actual} != {expected}"
    );
}

#[test]
fn a_pocket_wall_moved_past_its_opposite_refuses_naming_the_face() {
    for distance in [5.0, 6.0, 12.0] {
        let mut topo = Topology::new();
        let source = pocketed_block(&mut topo);
        let walls = pocket_walls(&topo, source);
        let before = fingerprint(&topo, source);
        let error = move_faces_multi(&mut topo, source, &walls, &[distance])
            .expect_err("the pocket's y walls meet at 5");
        let OperationsError::Offset(OffsetError::TopologyChange {
            face: Some(face), ..
        }) = &error
        else {
            panic!("expected a TopologyChange naming a face at {distance}, got {error}");
        };
        // The x walls are the ones squeezed shut between the y walls.
        assert!(
            walls.contains(face),
            "named face {face:?} is not a pocket wall ({error})"
        );
        assert_eq!(
            fingerprint(&topo, source),
            before,
            "refusal edited the source"
        );
    }

    // One wall driven past the opposite one while the others move a little.
    let mut topo = Topology::new();
    let source = pocketed_block(&mut topo);
    let walls = pocket_walls(&topo, source);
    let error = move_faces_multi(&mut topo, source, &walls, &[21.0, 0.5, 0.5, 0.5])
        .expect_err("the x = 10 wall passes the x = 30 wall");
    assert!(
        matches!(
            &error,
            OperationsError::Offset(OffsetError::TopologyChange { face: Some(face), .. })
                if walls.contains(face)
        ),
        "{error}"
    );
}

#[test]
fn a_coplanar_group_is_the_ordinary_move_bit_for_bit() {
    let mut multi_topo = Topology::new();
    let source = slotted_block(&mut multi_topo);
    let tops = faces_on(&multi_topo, source, z(), Point3::new(0.0, 0.0, 10.0));
    let mut single_topo = multi_topo.clone();
    let multi = move_faces_multi_journaled(&mut multi_topo, source, &tops, &[2.0, 2.0])
        .expect("coplanar multi move");
    let single = move_faces_journaled(&mut single_topo, source, &tops, 2.0).expect("single move");
    assert_eq!(multi.solid, single.solid);
    assert_eq!(multi.op, single.op);
    assert_eq!(format!("{:?}", multi_topo), format!("{:?}", single_topo));
}

#[test]
fn invalid_multi_selections_refuse_without_edits() {
    let mut topo = Topology::new();
    let source = pocketed_block(&mut topo);
    let walls = pocket_walls(&topo, source);
    let before = fingerprint(&topo, source);
    let cases: Vec<(Vec<FaceId>, Vec<f64>)> = vec![
        (Vec::new(), vec![1.0]),
        (walls.clone(), vec![1.0, 2.0]),
        (walls.clone(), vec![0.0]),
        (walls.clone(), vec![f64::NAN]),
        (vec![walls[0], walls[1], walls[0]], vec![1.0]),
    ];
    for (faces, distances) in cases {
        let error =
            move_faces_multi(&mut topo, source, &faces, &distances).expect_err("invalid selection");
        assert!(
            matches!(
                error,
                OperationsError::Offset(OffsetError::InvalidInput { .. })
            ),
            "{faces:?} {distances:?}: {error}"
        );
        assert_eq!(fingerprint(&topo, source), before);
    }
}

#[test]
fn a_prepared_multi_move_refuses_a_stale_source() {
    let mut topo = Topology::new();
    let source = pocketed_block(&mut topo);
    let walls = pocket_walls(&topo, source);
    let mut prepared =
        PreparedFaceMove::prepare_multi(&topo, source, &walls, &[1.0, 1.0, 2.0, 2.0])
            .expect("prepare");
    let unprepared = {
        let mut copy = topo.clone();
        let moved =
            move_faces_multi(&mut copy, source, &walls, &[1.0, 1.0, 2.0, 2.0]).expect("unprepared");
        fingerprint(&copy, moved)
    };
    let checkpoint = topo.clone();
    let moved = prepared.move_faces(&mut topo, 1.0).expect("prepared");
    assert_eq!(fingerprint(&topo, moved.solid), unprepared);

    // Translating the source in place makes the preparation stale; a
    // refusal edits nothing, and restoring the prepared content revives it.
    topo.restore_preserving_handle_slots(&checkpoint);
    transform_solid(&mut topo, source, &Mat4::translation(0.0, 0.0, 1.0)).expect("translate");
    assert!(!prepared.is_current(&topo));
    let after_edit = fingerprint(&topo, source);
    let stale = prepared
        .move_faces_journaled(&mut topo, 1.0)
        .expect_err("stale source");
    assert!(stale.to_string().contains("stale"), "{stale}");
    assert_eq!(fingerprint(&topo, source), after_edit);
    topo.restore_preserving_handle_slots(&checkpoint);
    assert!(prepared.is_current(&topo));
    let moved = prepared.move_faces(&mut topo, 1.0).expect("current again");
    assert_eq!(fingerprint(&topo, moved.solid), unprepared);
}

/// A 20 mm cube with two enclosed cavities, `a` and `b`.
fn cube_with_cavities(topo: &mut Topology, a: [[f64; 3]; 2], b: [[f64; 3]; 2]) -> SolidId {
    let cube = placed_box(topo, [0.0, 0.0, 0.0], [20.0, 20.0, 20.0]);
    let first = placed_box(topo, a[0], a[1]);
    let cut = boolean(topo, BooleanOp::Cut, cube, first).expect("first cavity");
    let second = placed_box(topo, b[0], b[1]);
    boolean(topo, BooleanOp::Cut, cut, second).expect("second cavity")
}

/// Cavities `[2,6]×[2,6]×[2,7]` and `[8,12]×[8,12]×[2,7]`; the first
/// cavity's `x = 6` and `y = 6` walls (outward normals −x and −y).
fn diagonal_cavities(topo: &mut Topology) -> (SolidId, Vec<FaceId>) {
    let solid = cube_with_cavities(
        topo,
        [[2.0, 2.0, 2.0], [6.0, 6.0, 7.0]],
        [[8.0, 8.0, 2.0], [12.0, 12.0, 7.0]],
    );
    let walls = vec![
        face_near(topo, solid, -x(), Point3::new(6.0, 4.0, 4.5)),
        face_near(topo, solid, -y(), Point3::new(4.0, 6.0, 4.5)),
    ];
    (solid, walls)
}

/// Cavities `[2,6]×[2,6]×[2,7]` and `[8,12]×[2,6]×[2,7]` a 2 mm gap apart;
/// the first cavity's `x = 6` wall (normal −x) and the second's `x = 8`
/// wall (normal +x), facing each other across the gap.
fn facing_cavities(topo: &mut Topology) -> (SolidId, Vec<FaceId>) {
    let solid = cube_with_cavities(
        topo,
        [[2.0, 2.0, 2.0], [6.0, 6.0, 7.0]],
        [[8.0, 2.0, 2.0], [12.0, 6.0, 7.0]],
    );
    let walls = vec![
        face_near(topo, solid, -x(), Point3::new(6.0, 4.0, 4.5)),
        face_near(topo, solid, x(), Point3::new(8.0, 4.0, 4.5)),
    ];
    (solid, walls)
}

fn assert_region_refusal(
    topo: &mut Topology,
    source: SolidId,
    faces: &[FaceId],
    distances: &[f64],
    source_volume: f64,
) {
    let before = fingerprint(topo, source);
    let error = move_faces_multi(topo, source, faces, distances)
        .expect_err("the moved region reaches the other cavity");
    let OperationsError::Offset(OffsetError::TopologyChange {
        face: Some(face), ..
    }) = &error
    else {
        panic!("expected a TopologyChange naming a face, got {error}");
    };
    assert!(faces.contains(face), "{error}");
    assert_eq!(
        fingerprint(topo, source),
        before,
        "refusal edited the source"
    );
    assert!((volume(topo, source) - source_volume).abs() <= 1e-6 * source_volume);
}

/// Review reproducer: the corner the two moved walls build reaches the
/// second cavity (1 × 1 × 5 = 5 mm³ of overlap) although neither wall's
/// own sweep does. It used to succeed with two overlapping cavity shells
/// and 7,675 mm³; it must refuse. A move that stays clear builds.
#[test]
fn a_corner_two_moved_walls_build_into_another_cavity_refuses() {
    let source_volume = 8000.0 - 80.0 - 80.0;
    let mut topo = Topology::new();
    let (source, walls) = diagonal_cavities(&mut topo);
    assert!((volume(&topo, source) - source_volume).abs() <= 1e-6 * source_volume);
    assert_region_refusal(&mut topo, source, &walls, &[-3.0], source_volume);

    // Prepared drags refuse the same move.
    let mut prepared =
        PreparedFaceMove::prepare_multi(&topo, source, &walls, &[]).expect("prepare");
    assert!(prepared.move_faces(&mut topo, -3.0).is_err());

    // Clear by 1 mm: the first cavity grows to [2,7]² × 5.
    let moved = move_faces_multi(&mut topo, source, &walls, &[-1.0]).expect("clear corner move");
    assert_verified(&topo, moved);
    let expected = 8000.0 - 5.0 * 5.0 * 5.0 - 80.0;
    let actual = volume(&topo, moved);
    assert!(
        (actual - expected).abs() <= 1e-6 * expected,
        "{actual} != {expected}"
    );
}

/// Review reproducer: two selected walls of separate cavities each advance
/// 1.5 mm into the 2 mm gap between them, so the cavities overlap by
/// 1 × 4 × 5 mm³ although neither wall reaches the other's source. It must
/// refuse; 0.5 mm each builds (both cavities 4.5 × 4 × 5).
#[test]
fn walls_advancing_into_one_gap_from_both_sides_refuse() {
    let source_volume = 8000.0 - 80.0 - 80.0;
    let mut topo = Topology::new();
    let (source, walls) = facing_cavities(&mut topo);
    assert_region_refusal(&mut topo, source, &walls, &[-1.5], source_volume);
    assert_region_refusal(&mut topo, source, &walls, &[-1.5, -1.0], source_volume);

    let moved = move_faces_multi(&mut topo, source, &walls, &[-0.5]).expect("clear gap move");
    assert_verified(&topo, moved);
    let expected = 2.0f64.mul_add(-90.0, 8000.0);
    let actual = volume(&topo, moved);
    assert!(
        (actual - expected).abs() <= 1e-6 * expected,
        "{actual} != {expected}"
    );
}
