//! Imported Gecko contracts through the consumer's batch/journal boundary.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;
use remus_math::compact_sort::CompactSort;
use remus_topology::naming::{Provenance, Resolution};
use remus_topology::{explorer::solid_faces, face::FaceSurface};
use serde_json::{Value, json};

// A successful journaled edit adds session history even when serializing its
// unchanged source. Compare the exact geometry/attributes separately from it.
fn geometry(k: &RemusKernel, solid: u32) -> Value {
    let mut doc: Value = serde_json::from_slice(&k.serialize_solids(&[solid]).unwrap()).unwrap();
    doc.as_object_mut().unwrap().remove("journal");
    doc
}

fn fixture(k: &mut RemusKernel) -> u32 {
    let solid = remus_io::step::reader::read_step(
        include_str!("../../../operations/tests/data/gentle_gecko_filleted.step"),
        k.topo_mut(),
    )
    .unwrap()[0];
    solid_id_to_u32(solid)
}

fn cylinder(k: &RemusKernel, solid: u32, along_x: bool) -> u32 {
    face_id_to_u32(solid_faces(k.topo(), k.resolve_solid(solid).unwrap()).unwrap()
        .into_iter().find(|&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Cylinder(c)
            if if along_x { c.axis().x().abs() > 0.99 } else {
                c.axis().z().abs() > 0.99 && c.origin().x().abs() < 1e-7 && c.origin().y().abs() < 1e-7
            })).unwrap())
}

fn batch(k: &mut RemusKernel, op: &str, args: Value) -> Value {
    let start = std::time::Instant::now();
    let out: Value =
        serde_json::from_str(&k.execute_batch(&json!([{"op":op,"args":args}]).to_string()))
            .unwrap();
    if let Some(path) = std::env::var_os("REMUS_W12_TIMINGS") {
        use std::io::Write;
        let row = json!({"op":op,"args":args,"ms":start.elapsed().as_secs_f64()*1000.0});
        writeln!(
            std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .unwrap(),
            "{row}"
        )
        .unwrap();
    }
    assert!(out[0].get("ok").is_some(), "{op} {args}: {out}");
    out[0]["ok"].clone()
}

#[test]
fn gecko_prepared_move_preview_cancel() {
    let mut k = RemusKernel::new();
    let solid = fixture(&mut k);
    let before = geometry(&k, solid);
    let face = solid_faces(k.topo(), k.resolve_solid(solid).unwrap())
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Plane { normal, d }
            if normal.z().abs() > 0.99 && (d.abs()-36.0).abs() < 1e-7)
        })
        .unwrap();
    let prepared = k
        .prepare_move_faces_impl(solid, &[face_id_to_u32(face)])
        .unwrap();
    for _ in 0..3 {
        let cp = k.checkpoint_impl().unwrap();
        batch(
            &mut k,
            "moveFacesPreparedJournaled",
            json!({"prepared":prepared,"distance":2.0}),
        );
        k.restore_checkpoint_impl(cp).unwrap();
        k.discard_checkpoint_impl(cp).unwrap();
        assert_eq!(geometry(&k, solid), before);
    }
    assert!(k.release_prepared_move(prepared));
}

fn qualify(k: &RemusKernel, solid: u32) {
    use remus_operations::tessellate::{
        boundary_edge_count, non_manifold_edge_count, tessellate_solid,
    };
    let id = k.resolve_solid(solid).unwrap();
    assert_eq!(k.validate_solid(solid).unwrap(), 0);
    let faces = solid_faces(k.topo(), id).unwrap();
    assert_eq!(faces.len(), 15);
    assert_eq!(
        faces
            .iter()
            .filter(|&&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
            .count(),
        2
    );
    let mut volumes = Vec::new();
    for d in [0.001, 0.0005] {
        let mesh = tessellate_solid(k.topo(), id, d).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0);
        assert_eq!(non_manifold_edge_count(&mesh), 0);
        volumes.push(k.volume(solid, d).unwrap());
    }
    assert!((volumes[0] - volumes[1]).abs() / volumes[0] < 1e-4);
}

#[test]
fn gecko_a2_journaled_convex_binding() {
    let mut k = RemusKernel::new();
    let source = fixture(&mut k);
    let keys = remus_operations::journal_ops::solid_entity_keys(
        k.topo(),
        k.resolve_solid(source).unwrap(),
    )
    .unwrap();
    let pending = k.topo_mut().journal_begin("import_step");
    let mut draft = remus_topology::journal::EvolutionDraft::construction();
    for &key in &keys {
        draft.push(
            key,
            remus_topology::journal::EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let anchor = k
        .topo_mut()
        .journal_record_evolution(pending, draft)
        .unwrap();
    let before = geometry(&k, source);
    let face = cylinder(&k, source, true);
    let out = batch(
        &mut k,
        "resizeBlendJournaled",
        json!({"solid":source,"face":face,"expectedRadius":4.0,"newRadius":1.5}),
    );
    assert!(out["solid"].is_number());
    assert_eq!(geometry(&k, source), before);
    let solid = u32::try_from(out["solid"].as_u64().unwrap()).unwrap();
    let first_op = anchor;
    let result = solid;
    let live = remus_operations::journal_ops::solid_entity_keys(
        k.topo(),
        k.resolve_solid(result).unwrap(),
    )
    .unwrap();
    let mut face_images = std::collections::BTreeMap::new();
    for kind in [
        remus_topology::journal::EntityKind::Face,
        remus_topology::journal::EntityKind::Edge,
    ] {
        let sources = keys.iter().filter(|key| key.kind == kind);
        for (index, source_key) in sources.enumerate() {
            let reference =
                remus_topology::naming::PersistentRef::operation_output(first_op, kind, index);
            let resolution = remus_topology::naming::resolve(k.topo(), &reference);
            match resolution {
                Resolution::Bound {
                    entity,
                    provenance: Provenance::Inferred,
                } => {
                    assert!(live.contains(&entity));
                    if kind == remus_topology::journal::EntityKind::Face {
                        face_images.insert(source_key.index, entity.index);
                    }
                }
                Resolution::BoundMany {
                    entities,
                    provenance: Provenance::Inferred,
                } if kind == remus_topology::journal::EntityKind::Edge => {
                    assert!(entities.iter().all(|e| live.contains(e)));
                }
                Resolution::Dangling { .. }
                    if kind == remus_topology::journal::EntityKind::Edge =>
                {
                    let old = remus_topology::explorer::edge_to_face_map(
                        k.topo(),
                        k.resolve_solid(source).unwrap(),
                    )
                    .unwrap();
                    let mut pair: Vec<_> = old[&source_key.index]
                        .iter()
                        .map(|f| face_images[&f.index()])
                        .collect();
                    pair.compact_sort_unstable();
                    let new = remus_topology::explorer::edge_to_face_map(
                        k.topo(),
                        k.resolve_solid(result).unwrap(),
                    )
                    .unwrap();
                    assert!(
                        !new.values().any(|faces| {
                            let mut indices: Vec<_> = faces.iter().map(|f| f.index()).collect();
                            indices.compact_sort_unstable();
                            indices == pair
                        }),
                        "deleted an edge whose face incidence survives"
                    );
                }
                Resolution::UnresolvedAcrossOperation { .. }
                    if kind == remus_topology::journal::EntityKind::Edge =>
                {
                    let old = remus_topology::explorer::edge_to_face_map(
                        k.topo(),
                        k.resolve_solid(source).unwrap(),
                    )
                    .unwrap();
                    let mut pair = old[&source_key.index].clone();
                    pair.compact_sort_unstable();
                    assert!(
                        old.values()
                            .filter(|faces| {
                                let mut f = (*faces).clone();
                                f.compact_sort_unstable();
                                f == pair
                            })
                            .count()
                            > 1,
                        "unique edge identity was lost"
                    );
                }
                other => panic!("{kind:?} slot {index}: {other:?}"),
            }
        }
    }
    qualify(&k, solid);
}

// This extra probe exceeds W12a's initial convex edit. The underlying group
// removal cannot yet recover the unequal-radius result for a second edit.
#[test]
#[ignore = "open: group removal of an unequal-radius rebuilt corner is not supported"]
fn gecko_convex_reedit_after_unequal_group_rebuild() {
    let mut k = RemusKernel::new();
    let source = fixture(&mut k);
    let face = cylinder(&k, source, true);
    let first = batch(
        &mut k,
        "resizeBlendJournaled",
        json!({"solid":source,"face":face,"expectedRadius":4.0,"newRadius":1.5}),
    );
    let solid = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
    let face = cylinder(&k, solid, true);
    batch(
        &mut k,
        "resizeBlendJournaled",
        json!({"solid":solid,"face":face,"expectedRadius":1.5,"newRadius":1.9}),
    );
}

#[test]
fn gecko_a7_journaled_sequence() {
    let mut k = RemusKernel::new();
    let mut solid = fixture(&mut k);
    for (step, diameter) in [14.6, 17.0, 21.1, 14.7].into_iter().enumerate() {
        let before = geometry(&k, solid);
        let face = cylinder(&k, solid, false);
        let cp = k.checkpoint_impl().unwrap();
        let out = batch(
            &mut k,
            "resizeCylindricalFaceJournaled",
            json!({"solid":solid,"face":face,"radius":diameter/2.0}),
        );
        assert_eq!(geometry(&k, solid), before);
        let result = u32::try_from(out["solid"].as_u64().unwrap()).unwrap();
        qualify(&k, result);
        if step == 2 {
            k.restore_checkpoint_impl(cp).unwrap();
            assert_eq!(geometry(&k, solid), before);
            assert!(k.resolve_solid(result).is_err());
        } else {
            solid = result;
        }
        k.discard_checkpoint_impl(cp).unwrap();
    }
    let face = cylinder(&k, solid, false);
    let before = geometry(&k, solid);
    let journal = k.journal_summary();
    let out: Value = serde_json::from_str(&k.execute_batch_v2(&json!([{"op":"resizeCylindricalFaceJournaled","args":{"solid":solid,"face":face,"radius":30.0}}]).to_string())).unwrap();
    assert!(
        out.to_string().contains("cylinder-diameter-unbuildable"),
        "{out}"
    );
    assert_eq!(geometry(&k, solid), before);
    assert_eq!(k.journal_summary(), journal);
}

#[test]
fn gecko_a6_explicit_chain_bindings() {
    for op in ["chamferV2", "chamferJournaled"] {
        let mut k = RemusKernel::new();
        let source = fixture(&mut k);
        let bottom = solid_faces(k.topo(), k.resolve_solid(source).unwrap()).unwrap().into_iter()
            .find(|&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Plane { normal, d } if normal.z().abs() > 0.99 && d.abs() < 1e-8)).unwrap();
        let edges: Vec<_> = k
            .topo()
            .wire(k.topo().face(bottom).unwrap().outer_wire())
            .unwrap()
            .edges()
            .iter()
            .filter(|e| {
                let edge = k.topo().edge(e.edge()).unwrap();
                let a = k.topo().vertex(edge.start()).unwrap().point();
                let b = k.topo().vertex(edge.end()).unwrap().point();
                !((a.x() - 30.0).abs() < 1e-7 && (b.x() - 30.0).abs() < 1e-7
                    || (a.y() - 18.0).abs() < 1e-7 && (b.y() - 18.0).abs() < 1e-7)
            })
            .map(|e| crate::handles::edge_id_to_u32(e.edge()))
            .collect();
        assert_eq!(edges.len(), 7);
        let before = geometry(&k, source);
        let out = batch(
            &mut k,
            op,
            json!({"solid":source,"edges":edges,"d1":0.8,"d2":0.8}),
        );
        let result =
            u32::try_from(out["solid"].as_u64().or_else(|| out.as_u64()).unwrap()).unwrap();
        assert_eq!(k.validate_solid(result).unwrap(), 0);
        assert_eq!(geometry(&k, source), before);
    }
}
