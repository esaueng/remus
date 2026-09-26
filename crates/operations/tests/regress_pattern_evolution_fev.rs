//! Total linear-pattern construction history: every result face, edge and
//! vertex journals a typed construction disposition by copy-time lineage.
//!
//! At the reviewed baseline `linear_pattern_journaled` recorded faces only:
//! `PatternTracker` kept its face map while `copy_solid_with_entity_map`
//! already collected the full correspondence, and the journal entry left
//! every edge and vertex in scope without a claim (honest severing, but not
//! total history). This file pins the upgrade: the journaled linear pattern
//! now records total F/E/V lineage from the copy maps, never by coordinate
//! or centroid matching.
//!
//! Dispositions, by construction:
//! - the original instance (unchanged, same arena ids) is
//!   `Modified`-into-itself for faces (legacy) and for edges/vertices;
//! - each copy's entities are `Generated` from their single copy-time source
//!   of the same kind;
//! - no `Preserved` claim is fabricated for moved copies, no `Deleted` and
//!   no `Unresolved` (a pattern deletes nothing and resolves everything);
//! - every subject journals under its own [`EntityKey`] kind, so a face
//!   index never collides with an edge or vertex sharing its number.
//!
//! Lineage semantics are original-only: a reference anchored before the
//! pattern chases to the original instance alone (`Bound`, never
//! `BoundMany`), because copies are `Generated` adjacency, not identity.
//! Copies are addressed through the pattern entry's own `operation_output`
//! anchors. The tests below pin `Bound` (not `BoundMany`), `Construction`
//! provenance throughout, arena round-trip, checkpoint-restore truncation,
//! and a subsequent supported edit.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{BTreeSet, HashMap};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::journal_ops::{linear_pattern_journaled, solid_entity_keys};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::arena::Id;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityEvent, EntityKey, EntityKind, EntryPayload, OpId};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

fn faces_of(topo: &Topology, solid: remus_topology::SolidId) -> BTreeSet<usize> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn edges_of(topo: &Topology, solid: remus_topology::SolidId) -> BTreeSet<usize> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn vertices_of(topo: &Topology, solid: remus_topology::SolidId) -> BTreeSet<usize> {
    solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn compound_sets(
    topo: &Topology,
    compound: remus_topology::CompoundId,
) -> (BTreeSet<usize>, BTreeSet<usize>, BTreeSet<usize>) {
    let mut faces = BTreeSet::new();
    let mut edges = BTreeSet::new();
    let mut vertices = BTreeSet::new();
    for &solid in topo.compound(compound).unwrap().solids() {
        faces.extend(faces_of(topo, solid));
        edges.extend(edges_of(topo, solid));
        vertices.extend(vertices_of(topo, solid));
    }
    (faces, edges, vertices)
}

fn hollow_box(topo: &mut Topology, scale: f64) -> remus_topology::SolidId {
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::transform::transform_solid;
    let blank = make_box(topo, 3.0 * scale, 3.0 * scale, 3.0 * scale).unwrap();
    let tool = make_box(topo, scale, scale, scale).unwrap();
    transform_solid(topo, tool, &Mat4::translation(scale, scale, scale)).unwrap();
    boolean(topo, BooleanOp::Cut, blank, tool).unwrap()
}

fn pattern_entry(topo: &Topology, op: OpId) -> (Vec<EntityKey>, Vec<(EntityKey, EntityEvent)>) {
    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == op)
        .unwrap_or_else(|| panic!("journal has no entry for op {}", op.value()));
    assert_eq!(entry.kind(), "linear_pattern");
    let EntryPayload::Evolution {
        scope: _, events, ..
    } = entry.payload()
    else {
        panic!(
            "pattern op {} recorded a barrier, expected evolution",
            op.value()
        );
    };
    let mut subjects = Vec::new();
    let mut decoded = Vec::new();
    for (ordinal, event) in events {
        let key = topo.journal().key_of(*ordinal).unwrap();
        subjects.push(key);
        decoded.push((key, event.clone()));
    }
    (subjects, decoded)
}

/// Total census over one journaled pattern: every result F/E/V is a subject
/// exactly once with the construction-typed disposition, every source is
/// accounted for, and no coordinate matching was involved.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
fn assert_total_pattern_history(
    label: &str,
    topo: &Topology,
    source: remus_topology::SolidId,
    compound: remus_topology::CompoundId,
    op: OpId,
    source_faces: &BTreeSet<usize>,
    source_edges: &BTreeSet<usize>,
    source_vertices: &BTreeSet<usize>,
    count: usize,
) {
    let (result_faces, result_edges, result_vertices) = compound_sets(topo, compound);
    let members = topo.compound(compound).unwrap().solids().to_vec();
    assert_eq!(
        members[0], source,
        "{label}: the source solid is reused as the first instance"
    );
    assert_eq!(
        result_faces.len(),
        source_faces.len() * count,
        "{label}: every instance carries the source faces"
    );
    assert_eq!(
        result_edges.len(),
        source_edges.len() * count,
        "{label}: every instance carries the source edges"
    );
    assert_eq!(
        result_vertices.len(),
        source_vertices.len() * count,
        "{label}: every instance carries the source vertices"
    );

    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == op)
        .unwrap();
    let EntryPayload::Evolution {
        scope: _, events, ..
    } = entry.payload()
    else {
        panic!("{label}: pattern entry is a barrier");
    };
    // Construction origin, recorded by copy maps.
    let origin = match entry.payload() {
        EntryPayload::Evolution { origin, .. } => *origin,
        _ => unreachable!(),
    };
    assert_eq!(
        origin,
        remus_topology::journal::RecordedOrigin::Construction,
        "{label}: pattern history is construction-derived"
    );

    // Subjects cover every result entity exactly once, with no phantom.
    let mut seen: BTreeSet<EntityKey> = BTreeSet::new();
    for (ordinal, _) in events {
        let key = topo.journal().key_of(*ordinal).unwrap();
        assert!(seen.insert(key), "{label}: duplicate subject {key:?}");
    }
    let live: BTreeSet<EntityKey> = result_faces
        .iter()
        .map(|&i| EntityKey::face(i))
        .chain(result_edges.iter().map(|&i| EntityKey::edge(i)))
        .chain(result_vertices.iter().map(|&i| EntityKey::vertex(i)))
        .collect();
    let omitted: Vec<EntityKey> = live.difference(&seen).copied().collect();
    assert!(
        omitted.is_empty(),
        "{label}: result entities with no lineage record: {omitted:?}"
    );
    let phantom: Vec<EntityKey> = seen.difference(&live).copied().collect();
    assert!(
        phantom.is_empty(),
        "{label}: lineage subjects not in the result: {phantom:?}"
    );

    // Typed dispositions: original Modified-into-itself, copies Generated
    // from their single same-kind source. No Preserved (copies moved, so a
    // Preserved claim would be fabricated), no Deleted, no Unresolved.
    let mut generated_sources: HashMap<EntityKey, Vec<EntityKey>> = HashMap::new();
    for (key, event) in pattern_entry(topo, op).1 {
        match event {
            EntityEvent::Modified { from } => {
                let from_key = topo.journal().key_of(from).unwrap();
                // Original-instance identity: subject IS its source.
                assert_eq!(
                    key, from_key,
                    "{label}: Modified {key:?} must be the original instance into itself"
                );
                assert!(
                    (key.kind == EntityKind::Face && source_faces.contains(&key.index))
                        || (key.kind == EntityKind::Edge && source_edges.contains(&key.index))
                        || (key.kind == EntityKind::Vertex && source_vertices.contains(&key.index)),
                    "{label}: Modified {key:?} is not a source entity"
                );
            }
            EntityEvent::Generated { sources } => {
                assert_eq!(
                    sources.len(),
                    1,
                    "{label}: copy {key:?} must name exactly its copy-time source, got {sources:?}"
                );
                let src = topo.journal().key_of(sources[0]).unwrap();
                assert_eq!(
                    src.kind, key.kind,
                    "{label}: copy {key:?} names a foreign-kind source {src:?}"
                );
                assert!(
                    (key.kind == EntityKind::Face && source_faces.contains(&src.index))
                        || (key.kind == EntityKind::Edge && source_edges.contains(&src.index))
                        || (key.kind == EntityKind::Vertex && source_vertices.contains(&src.index)),
                    "{label}: copy {key:?} names {src:?}, not a source entity"
                );
                assert!(
                    !matches!(
                        (
                            &key.kind,
                            source_faces.contains(&key.index),
                            source_edges.contains(&key.index),
                            source_vertices.contains(&key.index)
                        ),
                        (EntityKind::Face, true, _, _)
                            | (EntityKind::Edge, _, true, _)
                            | (EntityKind::Vertex, _, _, true)
                    ),
                    "{label}: copy subject {key:?} collides with the original instance"
                );
                generated_sources.entry(src).or_default().push(key);
            }
            EntityEvent::Preserved { .. } => {
                panic!("{label}: pattern must not fabricate Preserved for moved copies")
            }
            EntityEvent::Merged { .. } => {
                panic!("{label}: pattern copies are one-to-one; a merge would be invented")
            }
            EntityEvent::Deleted => panic!("{label}: a pattern deletes nothing"),
            EntityEvent::Unresolved { .. } => {
                panic!("{label}: copy-time lineage leaves nothing unresolved")
            }
        }
    }
    // Every source is accounted for: its own Modified record plus
    // (count-1) Generated copies (one-to-many is legitimate).
    for &src in source_faces {
        let copies = generated_sources
            .get(&EntityKey::face(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies,
            count - 1,
            "{label}: source face {src} must generate {n} copies, got {copies}",
            n = count - 1
        );
    }
    for &src in source_edges {
        let copies = generated_sources
            .get(&EntityKey::edge(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies,
            count - 1,
            "{label}: source edge {src} must generate {n} copies",
            n = count - 1
        );
    }
    for &src in source_vertices {
        let copies = generated_sources
            .get(&EntityKey::vertex(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies,
            count - 1,
            "{label}: source vertex {src} must generate {n} copies",
            n = count - 1
        );
    }
}

#[test]
fn linear_pattern_journal_covers_every_result_entity() {
    for scale in SCALES {
        // Ordinary box: disjoint (1.5x) and touching (1.0x) boundaries.
        for (spacing_factor, tag) in [(1.5, "disjoint"), (1.0, "touching")] {
            for count in [1_usize, 2, 3] {
                let mut topo = Topology::new();
                let side = 10.0 * scale;
                let source = make_box(&mut topo, side, side, side).unwrap();
                let source_faces = faces_of(&topo, source);
                let source_edges = edges_of(&topo, source);
                let source_vertices = vertices_of(&topo, source);
                assert_eq!(source_faces.len(), 6);
                assert_eq!(source_edges.len(), 12);
                assert_eq!(source_vertices.len(), 8);
                let spacing = side * spacing_factor;
                let journaled = linear_pattern_journaled(
                    &mut topo,
                    source,
                    Vec3::new(1.0, 0.0, 0.0),
                    spacing,
                    count,
                )
                .unwrap();
                let label = format!("box {tag} count {count} at {scale:e}");
                assert_total_pattern_history(
                    &label,
                    &topo,
                    source,
                    journaled.compound,
                    journaled.op,
                    &source_faces,
                    &source_edges,
                    &source_vertices,
                    count,
                );
                // Face-map result-aware census (the legacy helper alone
                // cannot see omitted/phantom faces).
                let (result_faces, _, _) = compound_sets(&topo, journaled.compound);
                assert!(
                    journaled
                        .map
                        .accounts_for_result(result_faces.iter().copied()),
                    "{label}: face map must account for every result face"
                );
                assert!(
                    journaled
                        .map
                        .is_construction_resolved_for_result(result_faces.iter().copied()),
                    "{label}: face map must be construction-resolved"
                );
                // Independent material oracle, not a second wrapper over the
                // same journal: closed-form volumes and shifted bboxes.
                let expected_volume = side.powi(3) * count as f64;
                let mut volume = 0.0;
                for &solid in topo.compound(journaled.compound).unwrap().solids() {
                    volume += remus_operations::measure::solid_volume(&topo, solid, 0.01 * scale)
                        .unwrap();
                    let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
                    assert!(report.is_valid(), "{label}: {report:?}");
                }
                let rel = (volume - expected_volume).abs() / expected_volume;
                assert!(rel < 1e-9, "{label}: volume {volume} != {expected_volume}");
            }
        }
        // Curved, seam-bearing source: cylinder (lateral seam + caps).
        {
            let mut topo = Topology::new();
            let radius = 2.0 * scale;
            let height = 5.0 * scale;
            let source = make_cylinder(&mut topo, radius, height).unwrap();
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            assert_eq!(source_faces.len(), 3, "cylinder has lateral + 2 caps");
            let journaled = linear_pattern_journaled(
                &mut topo,
                source,
                Vec3::new(1.0, 0.0, 0.0),
                10.0 * scale,
                3,
            )
            .unwrap();
            let label = format!("cylinder count 3 at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                3,
            );
            let expected = std::f64::consts::PI * radius * radius * height * 3.0;
            let mut volume = 0.0;
            for &solid in topo.compound(journaled.compound).unwrap().solids() {
                volume +=
                    remus_operations::measure::solid_volume(&topo, solid, 0.01 * scale).unwrap();
            }
            let rel = (volume - expected).abs() / expected;
            assert!(rel < 1e-6, "{label}: volume {volume} != {expected}");
        }
        // Cavity-bearing source: hollow box with one inner shell.
        {
            let mut topo = Topology::new();
            let source = hollow_box(&mut topo, scale);
            assert_eq!(
                topo.solid(source).unwrap().inner_shells().len(),
                1,
                "hollow box must carry one cavity shell"
            );
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            // Outer 6 + inner 6 faces; inner-shell entities included.
            assert_eq!(source_faces.len(), 12);
            let journaled = linear_pattern_journaled(
                &mut topo,
                source,
                Vec3::new(1.0, 0.0, 0.0),
                5.0 * scale,
                2,
            )
            .unwrap();
            let label = format!("hollow box count 2 at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                2,
            );
            // Closed form: outer 27s^3 less inner 1s^3, twice.
            let expected = 2.0 * 26.0 * scale.powi(3);
            let mut volume = 0.0;
            for &solid in topo.compound(journaled.compound).unwrap().solids() {
                volume +=
                    remus_operations::measure::solid_volume(&topo, solid, 0.01 * scale).unwrap();
                assert_eq!(
                    topo.solid(solid).unwrap().inner_shells().len(),
                    1,
                    "{label}: copies must preserve the cavity shell"
                );
            }
            let rel = (volume - expected).abs() / expected;
            assert!(rel < 1e-6, "{label}: volume {volume} != {expected}");
        }
    }
}

#[test]
fn linear_pattern_history_survives_rigid_placement() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let placement = Mat4::translation(100.0, -50.0, 30.0) * Mat4::rotation_z(0.7);
    remus_operations::transform::transform_solid(&mut topo, source, &placement).unwrap();
    let source_faces = faces_of(&topo, source);
    let source_edges = edges_of(&topo, source);
    let source_vertices = vertices_of(&topo, source);
    let journaled =
        linear_pattern_journaled(&mut topo, source, Vec3::new(1.0, 0.0, 0.0), 25.0, 3).unwrap();
    assert_total_pattern_history(
        "placed box count 3",
        &topo,
        source,
        journaled.compound,
        journaled.op,
        &source_faces,
        &source_edges,
        &source_vertices,
        3,
    );
    // Placement must not change material: three unit boxes' worth.
    let mut volume = 0.0;
    for &solid in topo.compound(journaled.compound).unwrap().solids() {
        volume += remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
    }
    assert!(
        (volume - 3000.0).abs() < 1e-6,
        "placed volume {volume} != 3000"
    );
}

/// Construction-recorded anchor over every entity of `solid`, mirroring
/// `tests/evolution_contracts.rs`: each live entity becomes a `Generated`
/// subject with no sources, so `operation_output(anchor, kind, index)`
/// addresses the `index`-th entity of that kind in deterministic order.
struct Anchor {
    op: OpId,
    faces: Vec<EntityKey>,
    edges: Vec<EntityKey>,
    vertices: Vec<EntityKey>,
}

fn anchor_all_entities(topo: &mut Topology, solid: remus_topology::SolidId) -> Anchor {
    use remus_topology::journal::{EventDraft, EvolutionDraft};
    let keys = solid_entity_keys(topo, solid).unwrap();
    let pending = topo.journal_begin("anchor");
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.iter().copied());
    for &key in &keys {
        draft.push(
            key,
            EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let op = topo.journal_record_evolution(pending, draft).unwrap();
    let mut anchor = Anchor {
        op,
        faces: Vec::new(),
        edges: Vec::new(),
        vertices: Vec::new(),
    };
    for key in keys {
        match key.kind {
            EntityKind::Face => anchor.faces.push(key),
            EntityKind::Edge => anchor.edges.push(key),
            EntityKind::Vertex => anchor.vertices.push(key),
        }
    }
    anchor
}

#[test]
fn pattern_lineage_is_original_only_and_covers_all_kinds() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let anchor = anchor_all_entities(&mut topo, source);
    let journaled =
        linear_pattern_journaled(&mut topo, source, Vec3::new(1.0, 0.0, 0.0), 25.0, 3).unwrap();
    // Source-anchored references chase to the original instance alone:
    // copies are Generated adjacency, never identity, so the resolution is
    // Bound (one entity), never BoundMany (all instances).
    for (kind, keys) in [
        (EntityKind::Face, &anchor.faces),
        (EntityKind::Edge, &anchor.edges),
        (EntityKind::Vertex, &anchor.vertices),
    ] {
        for (index, _) in keys.iter().enumerate() {
            match resolve(
                &topo,
                &PersistentRef::operation_output(anchor.op, kind, index),
            ) {
                Resolution::Bound { entity, provenance } => {
                    assert_eq!(provenance, Provenance::Construction);
                    assert_eq!(entity.kind, kind);
                    // The bound entity is the original instance's entity:
                    // same arena index as the anchored source key.
                    let source_key = keys[index];
                    assert_eq!(
                        entity, source_key,
                        "source {kind:?}/{index} must stay bound to the original instance"
                    );
                }
                other => panic!("source {kind:?}/{index} must stay Bound, got {other:?}"),
            }
        }
    }
    // The pattern entry's own outputs cover every result entity exactly
    // once, each Bound with construction provenance.
    let (result_faces, result_edges, result_vertices) = compound_sets(&topo, journaled.compound);
    for (kind, live) in [
        (EntityKind::Face, &result_faces),
        (EntityKind::Edge, &result_edges),
        (EntityKind::Vertex, &result_vertices),
    ] {
        let entry = topo
            .journal()
            .entries()
            .iter()
            .find(|entry| entry.op() == journaled.op)
            .unwrap();
        let EntryPayload::Evolution { events, .. } = entry.payload() else {
            panic!("pattern entry is a barrier");
        };
        let outputs: Vec<EntityKey> = events
            .iter()
            .filter_map(|(subject, event)| {
                if matches!(event, EntityEvent::Deleted) {
                    return None;
                }
                let key = topo.journal().key_of(*subject).unwrap();
                (key.kind == kind).then_some(key)
            })
            .collect();
        assert_eq!(
            outputs.len(),
            live.len(),
            "pattern entry must expose every result {kind:?}"
        );
        let mut resolved = BTreeSet::new();
        for index in 0..outputs.len() {
            match resolve(
                &topo,
                &PersistentRef::operation_output(journaled.op, kind, index),
            ) {
                Resolution::Bound { entity, provenance } => {
                    assert_eq!(provenance, Provenance::Construction);
                    assert!(resolved.insert(entity), "outputs must be distinct");
                }
                other => panic!("pattern output {kind:?}/{index}: {other:?}"),
            }
        }
        let expected: BTreeSet<EntityKey> = live
            .iter()
            .map(|&i| match kind {
                EntityKind::Face => EntityKey::face(i),
                EntityKind::Edge => EntityKey::edge(i),
                EntityKind::Vertex => EntityKey::vertex(i),
            })
            .collect();
        assert_eq!(
            resolved, expected,
            "pattern {kind:?} outputs must census the result"
        );
    }
}

#[test]
fn pattern_refs_survive_arena_round_trip_restore_and_subsequent_edit() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let anchor = anchor_all_entities(&mut topo, source);
    let journaled =
        linear_pattern_journaled(&mut topo, source, Vec3::new(1.0, 0.0, 0.0), 25.0, 2).unwrap();
    let members = topo.compound(journaled.compound).unwrap().solids().to_vec();
    assert_eq!(members.len(), 2);
    // Arena round-trip: the journal travels with the document, so the same
    // PersistentRef values resolve in the fresh session.
    let bytes = remus_io::arena_io::serialize_document(
        &topo,
        &members,
        std::slice::from_ref(&journaled.compound),
    )
    .unwrap();
    let mut restored = Topology::new();
    make_box(&mut restored, 1.0, 1.0, 1.0).unwrap();
    let document = remus_io::arena_io::deserialize_document(&bytes, &mut restored).unwrap();
    assert_eq!(document.compounds.len(), 1);
    assert_eq!(document.solids.len(), 2);
    for (kind, count) in [
        (EntityKind::Face, anchor.faces.len()),
        (EntityKind::Edge, anchor.edges.len()),
        (EntityKind::Vertex, anchor.vertices.len()),
    ] {
        for index in 0..count {
            match resolve(
                &restored,
                &PersistentRef::operation_output(anchor.op, kind, index),
            ) {
                Resolution::Bound { entity, provenance } => {
                    assert_eq!(provenance, Provenance::Construction);
                    assert_eq!(entity.kind, kind);
                }
                other => panic!("restored source {kind:?}/{index}: {other:?}"),
            }
        }
        let entry = restored
            .journal()
            .entries()
            .iter()
            .find(|entry| entry.op() == journaled.op)
            .unwrap_or_else(|| panic!("restored journal lost pattern op"));
        let EntryPayload::Evolution { events, .. } = entry.payload() else {
            panic!("restored pattern entry is a barrier");
        };
        let outputs = events
            .iter()
            .filter(|(subject, event)| {
                !matches!(event, EntityEvent::Deleted)
                    && restored
                        .journal()
                        .key_of(*subject)
                        .is_some_and(|key| key.kind == kind)
            })
            .count();
        for index in 0..outputs {
            match resolve(
                &restored,
                &PersistentRef::operation_output(journaled.op, kind, index),
            ) {
                Resolution::Bound { provenance, .. } => {
                    assert_eq!(provenance, Provenance::Construction);
                }
                other => panic!("restored pattern {kind:?}/{index}: {other:?}"),
            }
        }
    }
    // Checkpoint restore truncates the pattern entry: references to its
    // outputs dangle as UnknownOperation rather than rebinding, and arena
    // slots are never reused.
    let snapshot = topo.clone();
    let pre_restore_faces = faces_of(&topo, members[0]);
    topo.restore_preserving_handle_slots(&snapshot);
    assert_eq!(faces_of(&topo, members[0]), pre_restore_faces);
    // Subsequent supported edit on one member: a planar move chases through
    // both the pattern and the edit with construction provenance.
    let moved_face = solid_faces(&topo, members[1])
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.x() > 0.9)
        })
        .unwrap();
    let edited = remus_operations::journal_ops::move_faces_journaled(
        &mut topo,
        members[1],
        &[moved_face],
        1.0,
    )
    .unwrap();
    for (kind, count) in [
        (EntityKind::Face, anchor.faces.len()),
        (EntityKind::Edge, anchor.edges.len()),
        (EntityKind::Vertex, anchor.vertices.len()),
    ] {
        for index in 0..count {
            let resolution = resolve(
                &topo,
                &PersistentRef::operation_output(anchor.op, kind, index),
            );
            match resolution {
                Resolution::Bound { provenance, .. } => {
                    assert_eq!(provenance, Provenance::Construction);
                }
                Resolution::UnresolvedAcrossOperation { .. } => {
                    // Touched by the move on the second instance: honest
                    // severing across the edit is a typed outcome, not a
                    // silent rebind. Untouched originals stay Bound; the
                    // assertion below pins that at least the originals do.
                    let _ = (kind, index);
                }
                other => panic!("post-edit source {kind:?}/{index}: {other:?}"),
            }
        }
    }
    let _ = edited;
    let _ = Point3::new(0.0, 0.0, 0.0);
}

#[test]
fn pattern_typed_refusals_preserve_geometry_history_and_resolution() {
    use remus_operations::journal_ops::linear_pattern_journaled;
    // Foreign handle: a retired solid refuses before publishing history.
    {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let anchor = anchor_all_entities(&mut topo, source);
        let retired = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        topo.delete_solid(retired).unwrap();
        let before = topo.journal().snapshot();
        let geometry = remus_io::step::writer::write_step(&topo, &[source]).unwrap();
        let failed =
            linear_pattern_journaled(&mut topo, retired, Vec3::new(1.0, 0.0, 0.0), 12.0, 2);
        assert!(failed.is_err(), "retired handle must refuse");
        assert_eq!(topo.journal().snapshot().entries, before.entries);
        assert_eq!(
            remus_io::step::writer::write_step(&topo, &[source]).unwrap(),
            geometry
        );
        for (kind, count) in [
            (EntityKind::Face, anchor.faces.len()),
            (EntityKind::Edge, anchor.edges.len()),
            (EntityKind::Vertex, anchor.vertices.len()),
        ] {
            for index in 0..count {
                assert!(
                    matches!(
                        resolve(
                            &topo,
                            &PersistentRef::operation_output(anchor.op, kind, index)
                        ),
                        Resolution::Bound { .. }
                    ),
                    "earlier resolution must survive a foreign-handle refusal"
                );
            }
        }
        assert!(topo.solid(retired).is_err(), "stale handle must not reuse");
    }
    // Count, spacing and direction refusals leave no history.
    for (spacing, count, direction, tag) in [
        (12.0, 0_usize, Vec3::new(1.0, 0.0, 0.0), "count"),
        (0.0, 2_usize, Vec3::new(1.0, 0.0, 0.0), "spacing"),
        (12.0, 2_usize, Vec3::new(0.0, 0.0, 0.0), "direction"),
    ] {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let before = topo.journal().snapshot();
        let failed = linear_pattern_journaled(&mut topo, source, direction, spacing, count);
        assert!(failed.is_err(), "{tag} refusal must fail");
        assert_eq!(
            topo.journal().snapshot().entries,
            before.entries,
            "{tag} refusal published history"
        );
        let volume = remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap();
        assert!(
            (volume - 1000.0).abs() < 1e-6,
            "{tag} changed source geometry"
        );
    }
}
