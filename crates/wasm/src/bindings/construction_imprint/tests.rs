#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
use super::*;
use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

fn fixture() -> (BrepKernel, u32, u32, PersistentRef) {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(10., 8., 4.).unwrap();
    let face = kernel
        .get_solid_faces(solid)
        .unwrap()
        .into_iter()
        .find(|face| kernel.get_face_normal(*face).unwrap()[2] > 0.99)
        .unwrap();
    let source = kernel.resolve_solid(solid).unwrap();
    let pending = kernel.topo_mut().journal_begin("imprint_binding_seed");
    let mut draft = EvolutionDraft::construction();
    for key in remus_operations::journal_ops::solid_entity_keys(kernel.topo(), source).unwrap() {
        draft.push(key, EventDraft::Generated { sources: vec![] });
    }
    let op = kernel
        .topo_mut()
        .journal_record_evolution(pending, draft)
        .unwrap();
    let reference=(0..6).map(|i|PersistentRef::operation_output(op,remus_topology::journal::EntityKind::Face,i)).find(|r| matches!(resolve(kernel.topo(),r),Resolution::Bound {entity,..} if entity==EntityKey::face(face as usize))).unwrap();
    (kernel, solid, face, reference)
}
fn points() -> Vec<f64> {
    vec![3., 2., 4., 7., 2., 4., 7., 6., 4., 3., 6., 4.]
}
fn invoke(kernel: &mut BrepKernel, route: usize, solid: u32, face: u32, points: &[f64]) -> Value {
    if route == 0 {
        return serde_json::from_str(
            &kernel
                .imprint_planar_polygon_js(solid, face, points)
                .unwrap(),
        )
        .unwrap();
    }
    let input=serde_json::json!([{"op":"imprintPlanarPolygon","args":{"solid":solid,"face":face,"points":points}}]).to_string();
    let text = if route == 1 {
        kernel.execute_batch(&input)
    } else {
        kernel.execute_batch_v2(&input)
    };
    let rows: Value = serde_json::from_str(&text).unwrap();
    assert!(rows[0].get("error").is_none(), "{rows}");
    rows[0]["ok"].clone()
}
#[test]
fn public_routes_preserve_history_names_and_checkpoint_undo_without_handle_reuse() {
    for route in 0..3 {
        let (mut kernel, solid, face, reference) = fixture();
        kernel.set_face_name(face, Some("support".into())).unwrap();
        let before = kernel.checkpoint().unwrap();
        let journal = kernel.journal_summary();
        let result = invoke(&mut kernel, route, solid, face, &points());
        let result_solid = u32::try_from(result["solid"].as_u64().unwrap()).unwrap();
        let inner = u32::try_from(result["innerFace"].as_u64().unwrap()).unwrap();
        let surround = u32::try_from(result["surroundFace"].as_u64().unwrap()).unwrap();
        for patch in [inner, surround] {
            assert_eq!(
                kernel.get_face_name(patch).unwrap().as_deref(),
                Some("support")
            );
        }
        assert_eq!(kernel.get_solid_faces(result_solid).unwrap().len(), 7);
        assert!(
            matches!(resolve(kernel.topo(),&reference),Resolution::BoundMany {provenance:Provenance::Construction,entities} if entities.len()==2)
        );
        assert_eq!(kernel.validate_solid(result_solid).unwrap(), 0);
        let value = remus_operations::measure::solid_volume(
            kernel.topo(),
            kernel.resolve_solid(result_solid).unwrap(),
            1e-4,
        )
        .unwrap();
        assert!((value - 320.).abs() < 1e-8);
        let after = kernel.checkpoint().unwrap();
        let failed = kernel
            .imprint_planar_polygon_impl(result_solid, surround, &points())
            .unwrap_err();
        assert!(failed.message().contains("target face holes"));
        kernel.restore(after).unwrap();
        assert!(matches!(
            resolve(kernel.topo(), &reference),
            Resolution::BoundMany { .. }
        ));
        kernel.restore(before).unwrap();
        assert_eq!(kernel.journal_summary(), journal);
        assert!(kernel.resolve_solid(result_solid).is_err());
        assert!(kernel.resolve_face(inner).is_err());
        assert!(matches!(
            resolve(kernel.topo(), &reference),
            Resolution::Bound {
                provenance: Provenance::Construction,
                ..
            }
        ));
        let replay = invoke(&mut kernel, route, solid, face, &points());
        assert_ne!(replay["solid"], result["solid"]);
        assert_ne!(replay["innerFace"], result["innerFace"]);
        assert_ne!(replay["op"], result["op"]);
    }
}
#[test]
fn parser_and_native_refusals_leave_live_session_and_history_unchanged() {
    let (mut kernel, solid, face, reference) = fixture();
    let before = kernel.journal_summary();
    let counts = (
        kernel.topo().num_vertices(),
        kernel.topo().num_edges(),
        kernel.topo().num_faces(),
        kernel.topo().num_solids(),
        kernel.topo().num_loops(),
        kernel.topo().num_coedges(),
    );
    let mut nan = points();
    nan[4] = f64::NAN;
    assert!(
        kernel
            .imprint_planar_polygon_impl(solid, face, &nan)
            .is_err()
    );
    let mut boundary = points();
    boundary[0] = 0.;
    for route in [1, 2] {
        for args in [
            serde_json::json!({"solid":solid,"face":face,"points":boundary}),
            serde_json::json!({"solid":solid,"face":face,"points":vec![0.;51]}),
            serde_json::json!({"solid":solid,"face":face,"points":[1,2]}),
            serde_json::json!({"solid":solid,"face":face,"points":"bad"}),
            serde_json::json!({"solid":solid,"face":u32::MAX,"points":points()}),
        ] {
            let input = serde_json::json!([{"op":"imprintPlanarPolygon","args":args}]).to_string();
            let output = if route == 1 {
                kernel.execute_batch(&input)
            } else {
                kernel.execute_batch_v2(&input)
            };
            let rows: Value = serde_json::from_str(&output).unwrap();
            assert!(rows[0].get("error").is_some(), "{rows}");
            if route == 2 && args["points"] == serde_json::json!(boundary) {
                assert_eq!(rows[0]["error"]["category"], "unsupported");
                assert_eq!(
                    rows[0]["error"]["details"]["kernelCode"],
                    "unsupported_imprint"
                );
            }
            assert_eq!(kernel.journal_summary(), before);
            assert_eq!(
                (
                    kernel.topo().num_vertices(),
                    kernel.topo().num_edges(),
                    kernel.topo().num_faces(),
                    kernel.topo().num_solids(),
                    kernel.topo().num_loops(),
                    kernel.topo().num_coedges()
                ),
                counts
            );
            assert!(matches!(
                resolve(kernel.topo(), &reference),
                Resolution::Bound {
                    provenance: Provenance::Construction,
                    ..
                }
            ));
        }
    }
}

#[test]
fn checkpoint_sharing_refusals_are_exact_and_unshared_late_slots_stay_retired() {
    for route in 0..3 {
        for shared in [false, true] {
            for late in [false, true] {
                let (mut kernel, solid, face, _) = fixture();
                if late {
                    let other = kernel
                        .get_solid_faces(solid)
                        .unwrap()
                        .into_iter()
                        .find(|f| *f != face)
                        .unwrap();
                    let id = kernel.resolve_face(other).unwrap();
                    kernel.topo_mut().face_mut(id).unwrap().set_reversed(true);
                }
                let _checkpoint = shared.then(|| kernel.checkpoint().unwrap());
                let owner = std::rc::Rc::as_ptr(&kernel.topo);
                let stats = kernel.checkpoint_memory_stats().unwrap();
                let journal = kernel.journal_summary();
                let slots = kernel.topo().allocated_slot_count();
                let mut coordinates = points();
                if !late {
                    coordinates[0] = 0.;
                }
                if route == 0 {
                    let error = kernel
                        .imprint_planar_polygon_impl(solid, face, &coordinates)
                        .unwrap_err();
                    assert!(!error.message().is_empty());
                } else {
                    let input=serde_json::json!([{"op":"imprintPlanarPolygon","args":{"solid":solid,"face":face,"points":coordinates}}]).to_string();
                    let output = if route == 1 {
                        kernel.execute_batch(&input)
                    } else {
                        kernel.execute_batch_v2(&input)
                    };
                    let rows: Value = serde_json::from_str(&output).unwrap();
                    assert!(rows[0].get("error").is_some(), "{rows}");
                    // Refusal payloads contain no newly allocated result handles.
                    assert!(rows[0].get("ok").is_none());
                    if late && route == 2 {
                        let details = rows[0]["error"]["details"].as_object().unwrap();
                        let keys: std::collections::BTreeSet<_> =
                            details.keys().map(String::as_str).collect();
                        assert_eq!(
                            keys,
                            [
                                "bodyClass",
                                "errorCount",
                                "kernelCode",
                                "operation",
                                "operationIndex"
                            ]
                            .into_iter()
                            .collect()
                        );
                        assert_eq!(details["kernelCode"], "body_validation_failed");
                    }
                }
                assert_eq!(kernel.journal_summary(), journal);
                assert_eq!(std::rc::Rc::as_ptr(&kernel.topo), owner);
                if shared {
                    assert_eq!(kernel.checkpoint_memory_stats().unwrap(), stats);
                }
                if !shared && late {
                    assert!(kernel.topo().allocated_slot_count() > slots);
                    assert!(kernel.topo().vertex_id_from_index(8).is_none());
                    kernel.make_box_solid(1., 1., 1.).unwrap();
                    assert!(
                        kernel.topo().vertex_id_from_index(8).is_none(),
                        "private failed handle must stay retired on the unshared path"
                    );
                }
            }
        }
    }
}
