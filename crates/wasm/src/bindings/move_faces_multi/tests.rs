#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{face_vertices, solid_entity_counts, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;

fn placed_box(topo: &mut Topology, min: [f64; 3], max: [f64; 3]) -> SolidId {
    let solid = make_box(topo, max[0] - min[0], max[1] - min[1], max[2] - min[2]).unwrap();
    transform_solid(topo, solid, &Mat4::translation(min[0], min[1], min[2])).unwrap();
    solid
}

fn face_with(topo: &Topology, solid: SolidId, normal: Vec3, on: Point3) -> Vec<FaceId> {
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

/// A pocketed block and its four pocket walls (x = 10, x = 30, y = 10,
/// y = 20); the same handles in every kernel built this way.
fn pocket(kernel: &mut RemusKernel) -> (u32, Vec<u32>) {
    let topo = kernel.topo_mut();
    let block = placed_box(topo, [0.0, 0.0, 0.0], [40.0, 30.0, 10.0]);
    let tool = placed_box(topo, [10.0, 10.0, 4.0], [30.0, 20.0, 11.0]);
    let body = boolean(topo, BooleanOp::Cut, block, tool).unwrap();
    let mut walls = Vec::new();
    for (normal, on) in [
        (Vec3::new(1.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)),
        (Vec3::new(-1.0, 0.0, 0.0), Point3::new(30.0, 0.0, 0.0)),
        (Vec3::new(0.0, 1.0, 0.0), Point3::new(0.0, 10.0, 0.0)),
        (Vec3::new(0.0, -1.0, 0.0), Point3::new(0.0, 20.0, 0.0)),
    ] {
        walls.extend(face_with(kernel.topo(), body, normal, on));
    }
    assert_eq!(walls.len(), 4);
    (
        solid_id_to_u32(body),
        walls.into_iter().map(face_id_to_u32).collect(),
    )
}

fn batch(kernel: &mut RemusKernel, ops: &serde_json::Value) -> Vec<serde_json::Value> {
    serde_json::from_str(&kernel.execute_batch(&ops.to_string())).unwrap()
}

#[test]
fn direct_batch_and_detailed_multi_moves_agree() {
    let mut direct = RemusKernel::new();
    let (source, walls) = pocket(&mut direct);
    let direct_json = direct
        .move_faces_multi_journaled_json(source, &walls, &[2.0])
        .unwrap();
    let direct_solid = u32::try_from(direct_json["solid"].as_u64().unwrap()).unwrap();
    let id = direct.resolve_solid(direct_solid).unwrap();
    let source_id = direct.resolve_solid(source).unwrap();
    assert_eq!(
        solid_entity_counts(direct.topo(), id).unwrap(),
        solid_entity_counts(direct.topo(), source_id).unwrap()
    );

    let mut batched = RemusKernel::new();
    assert_eq!(pocket(&mut batched), (source, walls.clone()));
    let results = batch(
        &mut batched,
        &serde_json::json!([{
            "op": "moveFacesMultiJournaled",
            "args": {"solid": source, "faces": walls, "distances": [2.0]},
        }]),
    );
    assert_eq!(results[0]["ok"], direct_json, "{results:?}");
    assert_eq!(
        batched.serialize_solids(&[direct_solid]).unwrap(),
        direct.serialize_solids(&[direct_solid]).unwrap()
    );

    let mut detailed = RemusKernel::new();
    assert_eq!(pocket(&mut detailed), (source, walls.clone()));
    let result = detailed.move_faces_multi_detailed_impl(source, &walls, &[2.0]);
    let value = serde_json::to_value(&result).unwrap();
    assert_eq!(value["status"], "ok", "{value}");
    assert_eq!(value["details"]["quality"], "exact");
    let evolution = &value["details"]["faceEvolution"];
    assert_eq!(evolution["origin"], "construction", "{evolution}");
    assert_eq!(evolution["unresolved"], serde_json::json!({}));
    let solid = u32::try_from(value["value"].as_u64().unwrap()).unwrap();
    let mut plain = RemusKernel::new();
    pocket(&mut plain);
    let moved = plain.move_faces_multi_impl(source, &walls, &[2.0]).unwrap();
    assert_eq!(
        detailed.serialize_solids(&[solid]).unwrap(),
        plain.serialize_solids(&[moved]).unwrap()
    );
}

#[test]
fn a_coplanar_selection_returns_what_move_faces_journaled_returns() {
    let mut multi = RemusKernel::new();
    let (source, _) = pocket(&mut multi);
    let id = multi.resolve_solid(source).unwrap();
    let tops: Vec<u32> = face_with(
        multi.topo(),
        id,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 10.0),
    )
    .into_iter()
    .map(face_id_to_u32)
    .collect();
    let mut single = RemusKernel::new();
    pocket(&mut single);
    let a = multi
        .move_faces_multi_journaled_json(source, &tops, &[1.5])
        .unwrap();
    let b = single
        .move_faces_journaled_json(source, &tops, 1.5)
        .unwrap();
    assert_eq!(a, b);
    let solid = u32::try_from(a["solid"].as_u64().unwrap()).unwrap();
    assert_eq!(
        multi.serialize_solids(&[solid]).unwrap(),
        single.serialize_solids(&[solid]).unwrap()
    );
}

#[test]
fn walls_moved_past_each_other_refuse_naming_a_wall_without_edits() {
    let mut kernel = RemusKernel::new();
    let (source, walls) = pocket(&mut kernel);
    let before = kernel.serialize_solids(&[source]).unwrap();
    let journal = kernel.topo().journal().len();
    let error = kernel
        .move_faces_multi_journaled_json(source, &walls, &[6.0])
        .unwrap_err();
    let message = error.message().to_owned();
    assert!(message.contains("would change topology"), "{message}");
    let named = walls
        .iter()
        .any(|&wall| message.contains(&format!("face Some(Id({wall}))")));
    assert!(named, "{message}");
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);
    assert_eq!(kernel.topo().journal().len(), journal);

    let value = serde_json::to_value(kernel.move_faces_multi_detailed_impl(source, &walls, &[6.0]))
        .unwrap();
    assert_eq!(value["status"], "error", "{value}");
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);

    let results = batch(
        &mut kernel,
        &serde_json::json!([
            {"op": "moveFacesMulti", "args": {"solid": source, "faces": walls, "distances": [1.0, 2.0]}},
            {"op": "moveFacesMulti", "args": {"solid": source, "faces": walls, "distances": [6.0]}},
        ]),
    );
    assert!(
        results.iter().all(|result| result.get("error").is_some()),
        "{results:?}"
    );
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);
}

/// A multi-face drag: the prepared move against `moveFacesMultiJournaled`
/// in a second kernel, both restoring one checkpoint every frame; then a
/// stale source, and the release.
#[test]
fn a_prepared_multi_drag_matches_the_unprepared_call() {
    let mut plain = RemusKernel::new();
    let (source, walls) = pocket(&mut plain);
    let plain_checkpoint = plain.checkpoint_impl().unwrap();
    let mut kernel = RemusKernel::new();
    assert_eq!(pocket(&mut kernel), (source, walls.clone()));
    let checkpoint = kernel.checkpoint_impl().unwrap();
    let weights = [1.0, 1.0, 0.5, 0.5];
    let prepared = kernel
        .prepare_move_faces_multi_impl(source, &walls, &weights)
        .unwrap();

    for distance in [2.0, -3.0, 0.5, 12.0, 0.0, -1.25] {
        plain.restore_checkpoint_impl(plain_checkpoint).unwrap();
        kernel.restore_checkpoint_impl(checkpoint).unwrap();
        let distances: Vec<f64> = weights.iter().map(|weight| weight * distance).collect();
        let expected = plain
            .move_faces_multi_journaled_json(source, &walls, &distances)
            .map_err(|error| error.message().to_owned());
        let actual = kernel
            .move_faces_prepared_journaled_json(prepared, distance)
            .map_err(|error| error.message().to_owned());
        assert_eq!(actual, expected, "distance {distance}");
        if let Ok(value) = &expected {
            let solid = u32::try_from(value["solid"].as_u64().unwrap()).unwrap();
            assert_eq!(
                kernel.serialize_solids(&[solid]).unwrap(),
                plain.serialize_solids(&[solid]).unwrap()
            );
        }
    }

    // The unjournaled prepared call returns `moveFacesMulti`'s document.
    plain.restore_checkpoint_impl(plain_checkpoint).unwrap();
    kernel.restore_checkpoint_impl(checkpoint).unwrap();
    let moved = kernel.move_faces_prepared_impl(prepared, 1.0).unwrap();
    let expected = plain
        .move_faces_multi_impl(source, &walls, &[1.0, 1.0, 0.5, 0.5])
        .unwrap();
    assert_eq!(
        kernel.serialize_solids(&[moved]).unwrap(),
        plain.serialize_solids(&[expected]).unwrap()
    );

    // An edit of the source makes it stale; the checkpoint revives it.
    kernel.restore_checkpoint_impl(checkpoint).unwrap();
    let source_id = kernel.resolve_solid(source).unwrap();
    transform_solid(
        kernel.topo_mut(),
        source_id,
        &Mat4::translation(0.0, 0.0, 1.0),
    )
    .unwrap();
    let stale = kernel
        .move_faces_prepared_journaled_json(prepared, 1.0)
        .unwrap_err();
    assert!(stale.message().contains("stale"), "{}", stale.message());
    kernel.restore_checkpoint_impl(checkpoint).unwrap();
    assert!(kernel.move_faces_prepared_impl(prepared, 1.0).is_ok());

    assert!(kernel.release_prepared_move(prepared));
    assert!(!kernel.release_prepared_move(prepared));
    assert!(
        kernel
            .prepare_move_faces_multi_impl(source, &walls, &[1.0, 2.0])
            .is_err(),
        "weights must be one per face"
    );
}

/// The `topology_change` refusal reaches JS typed: all four walls of the
/// 20 × 10 pocket moved inward 6 mm fold the y walls past each other. The
/// detailed twin, the v2 batch (plain and detailed op) and the journaled
/// call all carry `kernelCode`/code `topology_change`, category
/// `unsupported`, and the named face and edge as handles of the input.
#[test]
fn a_topology_change_refusal_carries_its_face_and_edge() {
    let mut kernel = RemusKernel::new();
    let (source, walls) = pocket(&mut kernel);
    let source_id = kernel.resolve_solid(source).unwrap();
    let edges: Vec<u32> = remus_topology::explorer::solid_edges(kernel.topo(), source_id)
        .unwrap()
        .into_iter()
        .map(crate::handles::edge_id_to_u32)
        .collect();
    let before = kernel.serialize_solids(&[source]).unwrap();
    let check = |details: &serde_json::Value, context: &str| {
        let face = u32::try_from(details["face"].as_u64().expect(context)).unwrap();
        assert!(walls.contains(&face), "{context}: {details}");
        let edge = u32::try_from(details["edge"].as_u64().expect(context)).unwrap();
        assert!(edges.contains(&edge), "{context}: {details}");
    };

    let value = serde_json::to_value(kernel.move_faces_multi_detailed_impl(source, &walls, &[6.0]))
        .unwrap();
    assert_eq!(value["status"], "error", "{value}");
    assert_eq!(value["code"], "topology_change", "{value}");
    assert_eq!(value["category"], "unsupported", "{value}");
    check(&value["details"], "detailed");
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);

    let journaled = kernel
        .move_faces_multi_journaled_json(source, &walls, &[6.0])
        .unwrap_err();
    assert_eq!(journaled.kernel_code(), Some("topology_change"));
    let (_, _, details) = journaled.into_direct_parts();
    check(&serde_json::Value::Object(details), "journaled");

    let results: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch_v2(
            &serde_json::json!([
                {"op": "moveFacesMulti", "args": {"solid": source, "faces": walls, "distances": [6.0]}},
            ])
            .to_string(),
        ),
    )
    .unwrap();
    let error = &results[0]["error"];
    assert_eq!(
        error["details"]["kernelCode"], "topology_change",
        "{results:?}"
    );
    assert_eq!(error["category"], "unsupported", "{results:?}");
    check(&error["details"], "batch v2");

    let results: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch_v2(
            &serde_json::json!([
                {"op": "moveFacesMultiDetailed", "args": {"solid": source, "faces": walls, "distances": [6.0]}},
            ])
            .to_string(),
        ),
    )
    .unwrap();
    let detailed = &results[0]["ok"];
    assert_eq!(detailed["code"], "topology_change", "{results:?}");
    check(&detailed["details"], "batch v2 detailed");
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);
}

/// The single-group `moveFacesDetailed` projects the same refusal: one
/// pocket wall pushed past the opposite wall names that wall.
#[test]
fn move_faces_detailed_refuses_a_topology_change_typed() {
    let mut kernel = RemusKernel::new();
    let (source, walls) = pocket(&mut kernel);
    let before = kernel.serialize_solids(&[source]).unwrap();
    let value =
        serde_json::to_value(kernel.move_faces_detailed_impl(source, &walls[..1], 25.0)).unwrap();
    assert_eq!(value["status"], "error", "{value}");
    assert_eq!(value["code"], "topology_change", "{value}");
    assert_eq!(value["category"], "unsupported", "{value}");
    assert_eq!(value["details"]["face"], walls[0], "{value}");
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);
}
