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

/// Record the imported `source` as one construction in the journal, so its
/// faces and edges have persistent references; returns its entity keys and
/// the operation.
fn anchor_import(
    k: &mut RemusKernel,
    source: u32,
) -> (
    Vec<remus_topology::journal::EntityKey>,
    remus_topology::journal::OpId,
) {
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
    (keys, anchor)
}

/// Edge history carried along a chain of journaled edits of the imported
/// body: where each imported edge slot stands after the latest edit.
struct EdgeChain {
    /// Slot -> its edges (arena indices) on the latest result.
    images: std::collections::BTreeMap<usize, Vec<usize>>,
    /// Slots an edit deleted or recorded as unresolved, with that edit.
    ended: std::collections::BTreeMap<usize, remus_topology::journal::OpId>,
}

impl EdgeChain {
    /// Every edge slot of the import at its own edge.
    fn new(keys: &[remus_topology::journal::EntityKey]) -> Self {
        Self {
            images: keys
                .iter()
                .filter(|key| key.kind == remus_topology::journal::EntityKind::Edge)
                .enumerate()
                .map(|(slot, key)| (slot, vec![key.index]))
                .collect(),
            ended: std::collections::BTreeMap::new(),
        }
    }
}

/// Face pairs (sorted face indices) shared by more than one edge of `solid`.
fn repeated_pairs(k: &RemusKernel, solid: u32) -> std::collections::BTreeSet<Vec<usize>> {
    let mut counts = std::collections::BTreeMap::<Vec<usize>, usize>::new();
    let map = remus_topology::explorer::edge_to_face_map(k.topo(), k.resolve_solid(solid).unwrap())
        .unwrap();
    for faces in map.values() {
        let mut pair: Vec<_> = faces.iter().map(|f| f.index()).collect();
        pair.compact_sort_unstable();
        *counts.entry(pair).or_default() += 1;
    }
    counts
        .into_iter()
        .filter(|&(_, count)| count > 1)
        .map(|(pair, _)| pair)
        .collect()
}

/// Every face and edge of the imported `source` (`keys`, recorded by
/// `anchor`) resolves through the journaled edits to `result`, the output of
/// edit `op` on `input`: faces to one live face, edges to live edges, or
/// deleted only where the edit consumed the edge's face incidence, or
/// unresolved only where the edit's input held the edge's face incidence on
/// more than one edge (the boundary induction never guesses between them).
#[allow(clippy::too_many_arguments)]
fn assert_journal_history(
    k: &RemusKernel,
    keys: &[remus_topology::journal::EntityKey],
    anchor: remus_topology::journal::OpId,
    source: u32,
    (input, op): (u32, remus_topology::journal::OpId),
    result: u32,
    chain: &mut EdgeChain,
    label: &str,
) {
    let live = remus_operations::journal_ops::solid_entity_keys(
        k.topo(),
        k.resolve_solid(result).unwrap(),
    )
    .unwrap();
    let input_edges =
        remus_topology::explorer::edge_to_face_map(k.topo(), k.resolve_solid(input).unwrap())
            .unwrap();
    let ambiguous = repeated_pairs(k, input);
    let imported_edges =
        remus_topology::explorer::edge_to_face_map(k.topo(), k.resolve_solid(source).unwrap())
            .unwrap();
    let result_edges =
        remus_topology::explorer::edge_to_face_map(k.topo(), k.resolve_solid(result).unwrap())
            .unwrap();
    // A bound edge must lie between the images of the faces its imported
    // edge lay between: liveness alone would accept a wrong binding.
    let assert_incidence = |face_images: &std::collections::BTreeMap<usize, usize>,
                            imported: usize,
                            bound: &[remus_topology::journal::EntityKey],
                            index: usize| {
        let mut expected: Vec<usize> = imported_edges[&imported]
            .iter()
            .map(|f| face_images[&f.index()])
            .collect();
        expected.compact_sort_unstable();
        for edge in bound {
            let mut faces: Vec<usize> = result_edges[&edge.index]
                .iter()
                .map(|f| f.index())
                .collect();
            faces.compact_sort_unstable();
            assert_eq!(
                faces, expected,
                "{label}: edge slot {index} bound to edge {} between other faces",
                edge.index
            );
        }
    };
    let mut face_images = std::collections::BTreeMap::new();
    for kind in [
        remus_topology::journal::EntityKind::Face,
        remus_topology::journal::EntityKind::Edge,
    ] {
        let sources = keys.iter().filter(|key| key.kind == kind);
        for (index, source_key) in sources.enumerate() {
            let reference =
                remus_topology::naming::PersistentRef::operation_output(anchor, kind, index);
            let resolution = remus_topology::naming::resolve(k.topo(), &reference);
            let edge = kind == remus_topology::journal::EntityKind::Edge;
            // The slot's pieces on this edit's input share their face pair
            // with another input edge: the boundary induction cannot tell
            // them apart, so the edit may leave the slot unresolved, or
            // bind it to every result edge of that pair, never to one.
            let shared = edge
                && chain.images.get(&index).is_some_and(|pieces| {
                    pieces.iter().any(|piece| {
                        let mut pair: Vec<_> =
                            input_edges[piece].iter().map(|f| f.index()).collect();
                        pair.compact_sort_unstable();
                        ambiguous.contains(&pair)
                    })
                });
            match &resolution {
                Resolution::Bound {
                    entity,
                    provenance: Provenance::Inferred,
                } => {
                    assert!(live.contains(entity), "{label}: {kind:?} slot {index}");
                    if edge {
                        assert!(
                            !shared,
                            "{label}: edge slot {index}, ambiguous on the input, bound to one \
                             edge {}",
                            entity.index
                        );
                        assert_incidence(&face_images, source_key.index, &[*entity], index);
                        chain.images.insert(index, vec![entity.index]);
                    } else {
                        face_images.insert(source_key.index, entity.index);
                    }
                }
                Resolution::BoundMany {
                    entities,
                    provenance: Provenance::Inferred,
                } if edge => {
                    assert!(
                        entities.iter().all(|e| live.contains(e)),
                        "{label}: edge slot {index}"
                    );
                    assert_incidence(&face_images, source_key.index, entities, index);
                    if shared {
                        // Every result edge between the same two faces.
                        let pair = &result_edges[&entities[0].index];
                        let mut pair: Vec<_> = pair.iter().map(|f| f.index()).collect();
                        pair.compact_sort_unstable();
                        let mut all: Vec<usize> = result_edges
                            .iter()
                            .filter(|(_, faces)| {
                                let mut f: Vec<_> = faces.iter().map(|f| f.index()).collect();
                                f.compact_sort_unstable();
                                f == pair
                            })
                            .map(|(&edge, _)| edge)
                            .collect();
                        all.compact_sort_unstable();
                        let mut bound: Vec<usize> = entities.iter().map(|e| e.index).collect();
                        bound.compact_sort_unstable();
                        assert_eq!(
                            bound, all,
                            "{label}: edge slot {index}, ambiguous on the input, bound to part \
                             of its face pair"
                        );
                    }
                    chain
                        .images
                        .insert(index, entities.iter().map(|e| e.index).collect());
                }
                Resolution::Dangling { deleted_at } if edge && *deleted_at != op => {
                    // Deleted by an earlier edit, whose result no longer
                    // held its face incidence (a later edit may restore the
                    // incidence on a new edge).
                    assert_eq!(
                        chain.ended.get(&index),
                        Some(deleted_at),
                        "{label}: edge slot {index}: {resolution:?}"
                    );
                }
                Resolution::Dangling { .. } if edge => {
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
                        "{label}: deleted edge slot {index} whose face incidence {pair:?} survives"
                    );
                    chain.images.remove(&index);
                    chain.ended.insert(index, op);
                }
                Resolution::UnresolvedAcrossOperation { op: at, .. } if edge => {
                    if *at == op {
                        // Severed by this edit: every piece of the slot on
                        // the edit's input shares its face pair with another
                        // edge there.
                        let pieces = chain.images.remove(&index).unwrap_or_default();
                        assert!(!pieces.is_empty(), "{label}: edge slot {index}");
                        for piece in pieces {
                            let mut pair: Vec<_> =
                                input_edges[&piece].iter().map(|f| f.index()).collect();
                            pair.compact_sort_unstable();
                            assert!(
                                ambiguous.contains(&pair),
                                "{label}: unique identity of edge slot {index} (input edge \
                                 {piece}, faces {pair:?}) was lost: {resolution:?}"
                            );
                        }
                        chain.ended.insert(index, op);
                    } else {
                        assert_eq!(
                            chain.ended.get(&index),
                            Some(at),
                            "{label}: edge slot {index}: {resolution:?}"
                        );
                    }
                }
                other => panic!("{label}: {kind:?} slot {index}: {other:?}"),
            }
        }
    }
}

fn op_of(out: &Value) -> remus_topology::journal::OpId {
    remus_topology::journal::OpId::from_value(out["op"].as_u64().unwrap())
}

#[test]
fn gecko_a2_journaled_convex_binding() {
    let mut k = RemusKernel::new();
    let source = fixture(&mut k);
    let (keys, anchor) = anchor_import(&mut k, source);
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
    let mut chain = EdgeChain::new(&keys);
    assert_journal_history(
        &k,
        &keys,
        anchor,
        source,
        (source, op_of(&out)),
        solid,
        &mut chain,
        "R4 -> R1.5",
    );
    qualify(&k, solid);
}

/// B274: the band the first edit rebuilt at R1.5 (its walked transition
/// into the seam round is an unequal-radius corner, and the boss wall's seam
/// stands on the shoulder's sharp end) is resized again, R1.5 -> 1.9, and
/// back to R4. Each edit leaves its source unchanged and meets the bar, and
/// the imported faces and edges resolve through every journaled edit.
///
/// The R1.5 and R1.9 bodies hold the boss-shoulder contact as two edges
/// split at the boss seam, with one face pair: an edit from such a body
/// records them as unresolved (ambiguous incidence) rather than guessing
/// which is which, and the chain check holds each edit to that rule.
#[test]
fn gecko_convex_reedit_after_unequal_group_rebuild() {
    let mut k = RemusKernel::new();
    let source = fixture(&mut k);
    let (keys, anchor) = anchor_import(&mut k, source);
    let imported = geometry(&k, source);
    let mut chain = EdgeChain::new(&keys);
    let contact = shoulder_contact_slot(&k, source, &keys);
    let contact_ref = remus_topology::naming::PersistentRef::operation_output(
        anchor,
        remus_topology::journal::EntityKind::Edge,
        contact,
    );
    let mut severed_at = None;
    let mut solid = source;
    for (step, (expected, radius)) in [(4.0, 1.5), (1.5, 1.9), (1.9, 4.0)].into_iter().enumerate() {
        let label = format!("R{expected} -> R{radius}");
        let before = geometry(&k, solid);
        let face = cylinder(&k, solid, true);
        let out = batch(
            &mut k,
            "resizeBlendJournaled",
            json!({"solid":solid,"face":face,"expectedRadius":expected,"newRadius":radius}),
        );
        assert_eq!(geometry(&k, solid), before, "{label}");
        let result = u32::try_from(out["solid"].as_u64().unwrap()).unwrap();
        qualify(&k, result);
        assert_journal_history(
            &k,
            &keys,
            anchor,
            source,
            (solid, op_of(&out)),
            result,
            &mut chain,
            &label,
        );
        // The boss-shoulder contact: the first edit splits it at the boss
        // seam into two arcs of its closed-form circle; the second edit,
        // from a body holding those two arcs with one face pair, records it
        // as unresolved rather than guess, and it stays so.
        let resolution = remus_topology::naming::resolve(k.topo(), &contact_ref);
        match (step, &resolution) {
            (
                0,
                Resolution::BoundMany {
                    entities,
                    provenance: Provenance::Inferred,
                },
            ) => {
                assert_eq!(entities.len(), 2, "{label}: {resolution:?}");
                for piece in entities {
                    assert_on_shoulder_contact(&k, piece.index, &label);
                }
            }
            (1, Resolution::UnresolvedAcrossOperation { op, .. }) => {
                assert_eq!(*op, op_of(&out), "{label}");
                severed_at = Some(*op);
            }
            (2, Resolution::UnresolvedAcrossOperation { op, .. }) => {
                assert_eq!(Some(*op), severed_at, "{label}");
            }
            _ => panic!("{label}: boss-shoulder contact {resolution:?}"),
        }
        solid = result;
    }
    assert_eq!(geometry(&k, source), imported);
}

/// The imported edge slot between the boss wall and the shoulder torus.
fn shoulder_contact_slot(
    k: &RemusKernel,
    source: u32,
    keys: &[remus_topology::journal::EntityKey],
) -> usize {
    let id = k.resolve_solid(source).unwrap();
    let edges = remus_topology::explorer::edge_to_face_map(k.topo(), id).unwrap();
    let boss = cylinder(k, source, false);
    let slots: Vec<usize> = keys
        .iter()
        .filter(|key| key.kind == remus_topology::journal::EntityKind::Edge)
        .enumerate()
        .filter(|(_, key)| {
            let faces = &edges[&key.index];
            faces.iter().any(|&f| face_id_to_u32(f) == boss)
                && faces
                    .iter()
                    .any(|&f| matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Torus(_)))
        })
        .map(|(slot, _)| slot)
        .collect();
    assert_eq!(slots.len(), 1, "one imported boss-shoulder contact");
    slots[0]
}

/// Edge `index` lies on the boss-shoulder contact's closed form: the R4
/// shoulder ball rolls at radius 10 and height 28, so it meets the R6 boss
/// on the circle of radius 6 about the Z axis at z = 28.
fn assert_on_shoulder_contact(k: &RemusKernel, index: usize, label: &str) {
    let edge = k.topo().edge_id_from_index(index).unwrap();
    let data = k.topo().edge(edge).unwrap();
    let remus_topology::edge::EdgeCurve::Circle(circle) = data.curve() else {
        panic!("{label}: contact piece {index} is not a circle");
    };
    assert!((circle.radius() - 6.0).abs() < 1e-9, "{label}: {circle:?}");
    assert!(
        circle.center().x().abs() < 1e-9
            && circle.center().y().abs() < 1e-9
            && (circle.center().z() - 28.0).abs() < 1e-9,
        "{label}: {circle:?}"
    );
    assert!(
        circle.normal().z().abs() > 1.0 - 1e-12,
        "{label}: {circle:?}"
    );
    for vertex in [data.start(), data.end()] {
        let p = k.topo().vertex(vertex).unwrap().point();
        assert!(
            (p.x().hypot(p.y()) - 6.0).abs() < 1e-7 && (p.z() - 28.0).abs() < 1e-7,
            "{label}: contact piece {index} ends at {p:?}"
        );
    }
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

/// Issue #1746: `resizeCylindricalFaceDetailed` names which step refused a
/// diameter, so a client can tell a size limit from blends that do not
/// re-fillet. Ø40 meets the block's back face; Ø34 resizes sharp but leaves
/// an edge no R4 round fits.
#[test]
fn gecko_diameter_refusal_names_its_cause() {
    let mut k = RemusKernel::new();
    let solid = fixture(&mut k);
    let face = cylinder(&k, solid, false);
    let before = geometry(&k, solid);
    for (radius, reason, advice) in [
        (20.0, "wall_meets_face", true),
        (17.0, "group_reblend_failed", false),
    ] {
        let crate::types::SolidOperationDetailedResult::Error { code, details, .. } =
            k.resize_cylindrical_face_reblend_detailed_impl(solid, face, radius, true)
        else {
            panic!("R{radius} resized");
        };
        assert_eq!(code, "cylinder-diameter-unbuildable", "R{radius}");
        assert_eq!(
            details["kernelCode"], "cylinder-diameter-unbuildable",
            "R{radius}"
        );
        assert_eq!(details["reason"], reason, "R{radius}");
        assert_eq!(details["requestedDiameter"], radius * 2.0, "R{radius}");
        let message = details["message"].as_str().unwrap_or_default();
        assert_eq!(
            message.contains("try a smaller diameter"),
            advice,
            "R{radius}: {message}"
        );
        assert_eq!(geometry(&k, solid), before, "R{radius}");
    }
}
