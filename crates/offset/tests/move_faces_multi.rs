//! Engine-level coverage of `remus_offset::move_faces_multi_with_entity_map`
//! (B287): each selected planar face moved along its own outward normal.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_offset::{
    OffsetError, coplanar_group_distance, move_faces_multi_with_entity_map,
    move_faces_with_entity_map,
};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{
    face_vertices, solid_edges, solid_entity_counts, solid_faces, solid_vertices,
};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

fn placed_box(topo: &mut Topology, min: [f64; 3], max: [f64; 3]) -> SolidId {
    let solid = make_box(topo, max[0] - min[0], max[1] - min[1], max[2] - min[2]).unwrap();
    transform_solid(topo, solid, &Mat4::translation(min[0], min[1], min[2])).unwrap();
    solid
}

fn pocketed_block(topo: &mut Topology) -> SolidId {
    let block = placed_box(topo, [0.0, 0.0, 0.0], [40.0, 30.0, 10.0]);
    let pocket = placed_box(topo, [10.0, 10.0, 4.0], [30.0, 20.0, 11.0]);
    boolean(topo, BooleanOp::Cut, block, pocket).unwrap()
}

/// Faces with outward `normal` through the plane at `on`.
fn faces_on(topo: &Topology, solid: SolidId, normal: Vec3, on: Point3) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            let point = topo
                .vertex(face_vertices(topo, face).unwrap()[0])
                .unwrap()
                .point();
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|actual| actual.dot(normal) > 1.0 - 1e-12)
                && (point - on).dot(normal).abs() < 1e-9
        })
        .collect()
}

fn pocket_walls(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    let mut walls = Vec::new();
    for (normal, on) in [
        (Vec3::new(1.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)),
        (Vec3::new(-1.0, 0.0, 0.0), Point3::new(30.0, 0.0, 0.0)),
        (Vec3::new(0.0, 1.0, 0.0), Point3::new(0.0, 10.0, 0.0)),
        (Vec3::new(0.0, -1.0, 0.0), Point3::new(0.0, 20.0, 0.0)),
    ] {
        walls.extend(faces_on(topo, solid, normal, on));
    }
    assert_eq!(walls.len(), 4);
    walls
}

/// The source solid's vertex positions and edge ends, in handle order.
fn source_content(topo: &Topology, solid: SolidId) -> String {
    let vertices: Vec<_> = solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .map(|vertex| (vertex, topo.vertex(vertex).unwrap().point()))
        .collect();
    let edges: Vec<_> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .map(|edge| {
            let data = topo.edge(edge).unwrap();
            (
                edge,
                data.start(),
                data.end(),
                format!("{:?}", data.curve()),
            )
        })
        .collect();
    format!(
        "{vertices:?} {edges:?} {:?}",
        solid_entity_counts(topo, solid).unwrap()
    )
}

#[test]
fn pocket_walls_move_with_a_total_entity_map() {
    for distance in [2.0, -3.0] {
        let mut topo = Topology::new();
        let source = pocketed_block(&mut topo);
        let walls = pocket_walls(&topo, source);
        let counts = solid_entity_counts(&topo, source).unwrap();
        let moved = move_faces_multi_with_entity_map(&mut topo, source, &walls, &[distance])
            .expect("multi move");
        assert_eq!(solid_entity_counts(&topo, moved.solid).unwrap(), counts);
        assert_eq!(moved.face_map.len(), counts.0);
        assert_eq!(moved.edge_map.len(), counts.1);
        assert_eq!(moved.vertex_map.len(), counts.2);
        let pocket = 2.0f64.mul_add(-distance, 20.0) * 2.0f64.mul_add(-distance, 10.0) * 6.0;
        let expected = 40.0f64.mul_add(300.0, -pocket);
        let actual = solid_volume(&topo, moved.solid, 0.01).unwrap();
        assert!(
            (actual - expected).abs() <= 1e-6 * expected,
            "{actual} != {expected}"
        );
    }
}

#[test]
fn walls_meeting_refuse_naming_a_moved_face_and_restore() {
    let mut topo = Topology::new();
    let source = pocketed_block(&mut topo);
    let walls = pocket_walls(&topo, source);
    let before = source_content(&topo, source);
    for distance in [5.0, 7.0] {
        let error = move_faces_multi_with_entity_map(&mut topo, source, &walls, &[distance])
            .expect_err("the y walls meet at 5");
        let OffsetError::TopologyChange {
            face: Some(face),
            edge: Some(_),
            ..
        } = &error
        else {
            panic!("{error}");
        };
        assert!(walls.contains(face), "{error}");
        assert_eq!(
            source_content(&topo, source),
            before,
            "refusal edited the source"
        );
    }
}

#[test]
fn a_coplanar_group_is_the_coplanar_move_bit_for_bit() {
    let mut multi = Topology::new();
    let source = pocketed_block(&mut multi);
    let tops = faces_on(
        &multi,
        source,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 10.0),
    );
    assert_eq!(
        coplanar_group_distance(&multi, &tops, &[1.5]).unwrap(),
        Some(1.5)
    );
    let walls = pocket_walls(&multi, source);
    assert_eq!(
        coplanar_group_distance(&multi, &walls, &[1.5]).unwrap(),
        None
    );
    let mut single = multi.clone();
    let a = move_faces_multi_with_entity_map(&mut multi, source, &tops, &[1.5]).unwrap();
    let b = move_faces_with_entity_map(&mut single, source, &tops, 1.5).unwrap();
    assert_eq!(a.solid, b.solid);
    assert_eq!(format!("{multi:?}"), format!("{single:?}"));
}

#[test]
fn opposite_normal_walls_of_a_rib_both_thicken() {
    // A 4-wide rib on a plate: its two long walls face -x and +x.
    let mut topo = Topology::new();
    let plate = placed_box(&mut topo, [0.0, 0.0, 0.0], [30.0, 20.0, 4.0]);
    let rib = placed_box(&mut topo, [13.0, 2.0, 4.0], [17.0, 18.0, 12.0]);
    let body = boolean(&mut topo, BooleanOp::Fuse, plate, rib).unwrap();
    let mut walls = faces_on(
        &topo,
        body,
        Vec3::new(-1.0, 0.0, 0.0),
        Point3::new(13.0, 0.0, 0.0),
    );
    walls.extend(faces_on(
        &topo,
        body,
        Vec3::new(1.0, 0.0, 0.0),
        Point3::new(17.0, 0.0, 0.0),
    ));
    assert_eq!(walls.len(), 2);
    for (distance, width) in [(1.0, 6.0), (-1.5, 1.0)] {
        let mut copy = topo.clone();
        let moved = move_faces_multi_with_entity_map(&mut copy, body, &walls, &[distance])
            .expect("rib walls");
        let actual = solid_volume(&copy, moved.solid, 0.01).unwrap();
        let expected = (width * 16.0f64).mul_add(8.0, 30.0 * 20.0 * 4.0);
        assert!(
            (actual - expected).abs() <= 1e-6 * expected,
            "{actual} != {expected}"
        );
    }
    let error = move_faces_multi_with_entity_map(&mut topo, body, &walls, &[-2.0])
        .expect_err("the rib's walls meet at -2");
    assert!(
        matches!(error, OffsetError::TopologyChange { face: Some(_), .. }),
        "{error}"
    );
}
