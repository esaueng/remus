//! Round-trip tests for naming serialization (RFC 0003, Stage 5): the
//! journal, attributes, and persistent references survive the native
//! arena format, so a naming regression is a replayable fixture.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::bop::BooleanOp;
use remus_io::arena_io::{deserialize_document, serialize_document};
use remus_io::naming_io::{deserialize_persistent_ref, serialize_persistent_ref};
use remus_operations::journal_ops::boolean_journaled;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::attributes::EntityAttributes;
use remus_topology::journal::{EntityKind, OpId};
use remus_topology::naming::{
    Discriminator, EntitySignature, PersistentRef, Provenance, Resolution, resolve,
    resolve_face_attributes,
};

/// A journaled fuse of two named-face boxes, with attributes propagated.
fn journaled_named_fuse(topo: &mut Topology) -> (remus_topology::SolidId, OpId) {
    let a = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let shift = remus_math::mat::Mat4::translation(5.0, 5.0, 5.0);
    remus_operations::transform::transform_solid(topo, b, &shift).unwrap();
    for (i, face) in remus_topology::explorer::solid_faces(topo, a)
        .unwrap()
        .into_iter()
        .enumerate()
    {
        topo.set_face_attributes(
            face,
            EntityAttributes {
                name: Some(format!("a-face-{i}")),
                ..Default::default()
            },
        )
        .unwrap();
    }
    let fused = boolean_journaled(topo, BooleanOp::Fuse, a, b).unwrap();
    topo.propagate_attributes_for_op(fused.op, false).unwrap();
    (fused.solid, fused.op)
}

#[test]
fn references_resolve_identically_after_a_document_round_trip() {
    let mut topo = Topology::new();
    let (solid, op) = journaled_named_fuse(&mut topo);

    // Resolve every face output pre-save, remembering the outcome by
    // content (names / outcome class) — arena indices will differ after
    // the document's dense re-indexing, so indices cannot be compared.
    let describe = |topo: &Topology, reference: &PersistentRef| -> String {
        match resolve(topo, reference) {
            Resolution::Bound { entity, provenance } => {
                let name = topo
                    .face_id_from_index(entity.index)
                    .and_then(|id| topo.attributes().face(id))
                    .and_then(|a| a.name.clone())
                    .unwrap_or_default();
                format!("bound:{name}:{provenance:?}")
            }
            Resolution::BoundMany { entities, .. } => format!("many:{}", entities.len()),
            Resolution::UnresolvedAcrossOperation { kind, .. } => format!("severed:{kind}"),
            other => format!("{other:?}"),
        }
    };
    let refs: Vec<PersistentRef> = (0..12)
        .map(|index| PersistentRef::operation_output(op, EntityKind::Face, index))
        .collect();
    let before: Vec<String> = refs.iter().map(|r| describe(&topo, r)).collect();
    assert!(
        before
            .iter()
            .any(|outcome| outcome.starts_with("bound:a-face-")),
        "the fixture must carry names through the fuse: {before:?}"
    );

    // Write the document, encode the references, and reload EVERYTHING
    // into a fresh session.
    let document = serialize_document(&topo, &[solid], &[]).unwrap();
    let encoded_refs: Vec<Vec<u8>> = refs
        .iter()
        .map(|r| serialize_persistent_ref(r).unwrap())
        .collect();

    let mut restored = Topology::new();
    deserialize_document(&document, &mut restored).unwrap();
    let after: Vec<String> = encoded_refs
        .iter()
        .map(|bytes| describe(&restored, &deserialize_persistent_ref(bytes).unwrap()))
        .collect();

    assert_eq!(
        before, after,
        "a reference must resolve identically across save/load — this is \
         what makes a naming regression a replayable fixture"
    );

    // The reference-keyed attribute read works in the restored session.
    let named_ref = refs
        .iter()
        .zip(&before)
        .find(|(_, outcome)| outcome.starts_with("bound:a-face-"))
        .map(|(reference, _)| reference.clone())
        .unwrap();
    let bound = resolve_face_attributes(&restored, &named_ref).unwrap();
    assert!(bound[0].1.is_some_and(|a| a.name.is_some()));
}

#[test]
fn journal_survives_further_editing_after_a_reload() {
    let mut topo = Topology::new();
    let (solid, op) = journaled_named_fuse(&mut topo);
    let document = serialize_document(&topo, &[solid], &[]).unwrap();

    let mut restored = Topology::new();
    let roots = deserialize_document(&document, &mut restored).unwrap();
    let reference = PersistentRef::operation_output(op, EntityKind::Face, 0);
    assert!(matches!(
        resolve(&restored, &reference),
        Resolution::Bound { .. }
    ));

    // The restored journal and counter are consistent: an unjournaled
    // mutation after the load severs exactly as it would have before it.
    let shift = remus_math::mat::Mat4::translation(1.0, 0.0, 0.0);
    remus_operations::transform::transform_solid(&mut restored, roots.solids[0], &shift).unwrap();
    let pending = restored.journal_begin("post_load_probe");
    restored.journal_record_barrier(pending, Vec::new());
    let r = resolve(&restored, &reference);
    assert!(
        matches!(
            r,
            Resolution::UnresolvedAcrossOperation { ref kind, .. }
                if kind == remus_topology::journal::UNJOURNALED_MUTATIONS
        ),
        "gap detection must keep working across a reload: {r:?}"
    );
}

#[test]
fn entities_outside_the_document_report_not_present_after_reload() {
    let mut topo = Topology::new();
    let (fused, op) = journaled_named_fuse(&mut topo);
    // Export one half of a split: the fuse's history rides along, but the
    // other half's entities are not in the document.
    let split = remus_operations::journal_ops::split_journaled(
        &mut topo,
        fused,
        remus_math::vec::Point3::new(7.5, 0.0, 0.0),
        remus_math::vec::Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let document = serialize_document(&topo, &[split.result.positive], &[]).unwrap();
    let mut restored = Topology::new();
    deserialize_document(&document, &mut restored).unwrap();
    let outside = (0..64)
        .map(|index| PersistentRef::operation_output(split.op, EntityKind::Face, index))
        .find(|reference| {
            let Resolution::Bound { entity, .. } = resolve(&topo, reference) else {
                return false;
            };
            let face = topo.face_id_from_index(entity.index).unwrap();
            remus_topology::explorer::solid_faces(&topo, split.result.negative)
                .unwrap()
                .contains(&face)
        })
        .expect("the split has an output on the half left out");
    let r = resolve(&restored, &outside);
    assert!(
        matches!(r, Resolution::NoMatch { .. }),
        "a reference to an entity the document does not contain must \
         report no match, never bind a stale index: {r:?}"
    );

    // A document holding none of the fuse's lineage carries none of it.
    let bystander = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let document = serialize_document(&topo, &[bystander], &[]).unwrap();
    let mut restored = Topology::new();
    deserialize_document(&document, &mut restored).unwrap();
    let r = resolve(
        &restored,
        &PersistentRef::operation_output(op, EntityKind::Face, 0),
    );
    assert!(
        matches!(r, Resolution::UnknownOperation { op: unknown } if unknown == op),
        "{r:?}"
    );
}

#[test]
fn signature_references_recover_across_sessions() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 20.0, 30.0).unwrap();
    let face = remus_topology::explorer::solid_faces(&topo, solid).unwrap()[2];
    let signature = EntitySignature::capture_face(&topo, face, 1e-7).unwrap();
    let reference = PersistentRef::signature(signature);
    let encoded = serialize_persistent_ref(&reference).unwrap();

    // A different session rebuilds the same model shape — no journal at
    // all. The signature reference recovers the face, marked inferred.
    let mut other = Topology::new();
    make_box(&mut other, 10.0, 20.0, 30.0).unwrap();
    let decoded = deserialize_persistent_ref(&encoded).unwrap();
    assert_eq!(decoded, reference, "references are value objects");
    let r = resolve(&other, &decoded);
    assert!(
        matches!(
            r,
            Resolution::Bound {
                provenance: Provenance::Inferred,
                ..
            }
        ),
        "{r:?}"
    );
}

#[test]
fn nested_and_discriminated_references_round_trip_exactly() {
    let base = PersistentRef::operation_output(OpId::from_value(7), EntityKind::Edge, 3)
        .with_discriminator(Discriminator::CurveType("circle".into()));
    let wrapped = PersistentRef::lineage_of(base)
        .with_discriminator(Discriminator::CurveType("circle".into()));
    let bytes = serialize_persistent_ref(&wrapped).unwrap();
    assert_eq!(deserialize_persistent_ref(&bytes).unwrap(), wrapped);
}

#[test]
fn journal_less_documents_stay_byte_identical() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let document = serialize_document(&topo, &[solid], &[]).unwrap();
    let json = String::from_utf8(document).unwrap();
    assert!(
        !json.contains("\"journal\"") && !json.contains("\"attributes\""),
        "the additive fields must be absent when empty, so historical \
         output stays byte-identical"
    );
}

/// A save/load cannot turn an unvouched source lineage into a binding. A
/// transform changes topology after the latest journal entry, including the
/// boundary and pcurve rewrites performed for a reflection.
#[test]
fn unjournaled_source_tail_stays_unresolved_after_arena_roundtrip() {
    use remus_math::mat::Mat4;
    use remus_topology::journal::UNJOURNALED_MUTATIONS;
    for matrix in [
        Mat4::translation(3.0, -2.0, 1.0),
        Mat4::scale(-1.0, 1.0, 1.0),
    ] {
        let mut source = Topology::new();
        let (solid, op) = journaled_named_fuse(&mut source);
        let reference = PersistentRef::operation_output(op, EntityKind::Face, 0);
        assert!(matches!(
            resolve(&source, &reference),
            Resolution::Bound { .. }
        ));
        remus_operations::transform::transform_solid(&mut source, solid, &matrix).unwrap();
        assert!(matches!(
            resolve(&source, &reference),
            Resolution::Stale { .. }
        ));
        let journal = source.journal().snapshot();
        let ticks = source.mutation_ticks();
        let document = serialize_document(&source, &[solid], &[]).unwrap();
        assert_eq!(
            document,
            serialize_document(&source, &[solid], &[]).unwrap()
        );
        assert_eq!(source.journal().snapshot(), journal, "saving is read-only");
        assert_eq!(source.mutation_ticks(), ticks);
        let mut restored = Topology::new();
        let roots = deserialize_document(&document, &mut restored).unwrap();
        assert_eq!(
            document,
            serialize_document(&restored, &roots.solids, &[]).unwrap(),
            "a replayed gap must not allocate another barrier"
        );
        let resolution = resolve(&restored, &reference);
        assert!(
            matches!(resolution,
                Resolution::UnresolvedAcrossOperation { ref kind, .. } if kind == UNJOURNALED_MUTATIONS
            ),
            "a source reference with an unjournaled tail must remain unresolved after loading: {resolution:?}"
        );
    }
}
