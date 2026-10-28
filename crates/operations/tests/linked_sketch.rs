//! Bounded 6.6 inputs use real construction history and independent coordinates.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::frame::Frame3;
use remus_math::vec::{Point3, Vec3};
use remus_operations::journal_ops::{begin_scoped, move_faces_journaled, solid_entity_keys};
use remus_operations::linked_sketch::{
    ExternalLineReference, LinkedSketchError, PlanarSketchAttachment,
};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft, OpId};
use remus_topology::naming::{PersistentRef, Resolution, resolve};

fn reference(topo: &Topology, op: OpId, key: EntityKey) -> PersistentRef {
    (0..100)
        .map(|index| PersistentRef::operation_output(op, key.kind, index))
        .find(|r| matches!(resolve(topo, r), Resolution::Bound { entity, .. } if entity == key))
        .unwrap()
}

fn fixture(
    scale: f64,
) -> (
    Topology,
    remus_topology::SolidId,
    PlanarSketchAttachment,
    ExternalLineReference,
) {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0 * scale, 8.0 * scale, 4.0 * scale).unwrap();
    let top = solid_faces(&topo, solid).unwrap().into_iter().find(|&f| {
        matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { normal, d } if normal.z() > 0.99 && (*d - 4.0 * scale).abs() < 1e-8)
    }).unwrap();
    let edge = topo
        .wire(topo.face(top).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();
    let keys = solid_entity_keys(&topo, solid).unwrap();
    let pending = begin_scoped(&mut topo, "fixture_primitive", &[solid]).unwrap();
    let mut draft = EvolutionDraft::construction();
    for key in keys {
        draft.push(key, EventDraft::Generated { sources: vec![] });
    }
    let op = topo.journal_record_evolution(pending, draft).unwrap();
    let attachment = PlanarSketchAttachment::new(
        &topo,
        reference(&topo, op, EntityKey::face(top.index())),
        Frame3 {
            origin: Point3::new(0.0, 0.0, 4.0 * scale),
            x: Vec3::new(1.0, 0.0, 0.0),
            y: Vec3::new(0.0, 1.0, 0.0),
            z: Vec3::new(0.0, 0.0, 1.0),
        },
    )
    .unwrap();
    let source = ExternalLineReference {
        edge: reference(&topo, op, EntityKey::edge(edge.index())),
        start: reference(
            &topo,
            op,
            EntityKey::vertex(topo.edge(edge).unwrap().start().index()),
        ),
        end: reference(
            &topo,
            op,
            EntityKey::vertex(topo.edge(edge).unwrap().end().index()),
        ),
    };
    (topo, solid, attachment, source)
}

#[test]
fn linked_line_and_plane_survive_real_face_edits_across_scales() {
    for scale in [0.001, 1.0, 1000.0] {
        let (mut topo, solid, attachment, source) = fixture(scale);
        let original = attachment.resolve_line(&topo, &source).unwrap();
        assert!(((original.end - original.start).length() - 10.0 * scale).abs() < 1e-8 * scale);
        let right = solid_faces(&topo, solid).unwrap().into_iter().find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.x() > 0.99)).unwrap();
        let moved = move_faces_journaled(&mut topo, solid, &[right], 2.0 * scale).unwrap();
        let image = attachment.resolve_line(&topo, &source).unwrap();
        assert!(((image.end - image.start).length() - 12.0 * scale).abs() < 1e-8 * scale);
        let top = solid_faces(&topo, moved.solid).unwrap().into_iter().find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.99)).unwrap();
        let moved = move_faces_journaled(&mut topo, moved.solid, &[top], 3.0 * scale).unwrap();
        let ticks = topo.mutation_ticks();
        let frame = attachment.resolve_frame(&topo).unwrap();
        assert!((frame.origin.z() - 7.0 * scale).abs() < 1e-8 * scale);
        let after = attachment.resolve_line(&topo, &source).unwrap();
        assert!((after.start - image.start).length() < 1e-8 * scale);
        assert!((after.end - image.end).length() < 1e-8 * scale);
        assert_eq!(topo.mutation_ticks(), ticks);
        assert!(
            remus_check::validate::validate_solid(
                &topo,
                moved.solid,
                &remus_check::validate::ValidateOptions::default()
            )
            .unwrap()
            .is_valid()
        );
    }
}

#[test]
fn portable_references_resolve_after_exact_arena_round_trip() {
    let (topo, solid, attachment, source) = fixture(1.0);
    let frame = attachment.resolve_frame(&topo).unwrap();
    let bytes = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let mut restored = Topology::new();
    remus_io::arena_io::deserialize_solids(&bytes, &mut restored).unwrap();
    let codec = |r: &PersistentRef| {
        remus_io::naming_io::deserialize_persistent_ref(
            &remus_io::naming_io::serialize_persistent_ref(r).unwrap(),
        )
        .unwrap()
    };
    let restored_attachment =
        PlanarSketchAttachment::new(&restored, codec(attachment.reference()), frame).unwrap();
    let old = attachment.resolve_line(&topo, &source).unwrap();
    let new = restored_attachment
        .resolve_line(
            &restored,
            &ExternalLineReference {
                edge: codec(&source.edge),
                start: codec(&source.start),
                end: codec(&source.end),
            },
        )
        .unwrap();
    assert_eq!(old.start, new.start);
    assert_eq!(old.end, new.end);
}

#[test]
fn splits_deletion_ambiguity_and_unjournaled_edits_refuse() {
    for split in [true, false] {
        let (mut topo, _, attachment, source) = fixture(1.0);
        let Resolution::Bound { entity, .. } = resolve(&topo, &source.edge) else {
            panic!("fixture source")
        };
        let id = topo.edge_id_from_index(entity.index).unwrap();
        let pending = topo.journal_begin("fixture_source_edit");
        let mut draft = EvolutionDraft::construction();
        if split {
            let other = topo.add_edge(topo.edge(id).unwrap().clone());
            draft.push(entity, EventDraft::Modified { from: entity });
            draft.push(
                EntityKey::edge(other.index()),
                EventDraft::Modified { from: entity },
            );
        } else {
            draft.push(entity, EventDraft::Deleted);
        }
        topo.journal_record_evolution(pending, draft).unwrap();
        let error = attachment.resolve_line(&topo, &source).unwrap_err();
        if split {
            assert!(matches!(
                error,
                LinkedSketchError::Unbound {
                    resolution: Resolution::BoundMany { .. },
                    ..
                }
            ));
        } else {
            assert!(matches!(
                error,
                LinkedSketchError::Unbound {
                    resolution: Resolution::Dangling { .. },
                    ..
                }
            ));
        }
    }
    let (mut topo, _, attachment, source) = fixture(1.0);
    let Resolution::Bound { entity, .. } = resolve(&topo, &source.edge) else {
        panic!()
    };
    let edge = topo.edge_id_from_index(entity.index).unwrap();
    let signature =
        remus_topology::naming::EntitySignature::capture_edge(&topo, edge, 1e-7).unwrap();
    let inferred = ExternalLineReference {
        edge: PersistentRef::signature(signature),
        ..source.clone()
    };
    assert!(matches!(
        attachment.resolve_line(&topo, &inferred),
        Err(LinkedSketchError::InferredReference)
    ));
    let pending = topo.journal_begin("fixture_duplicate_edge");
    let duplicate = make_box(&mut topo, 10.0, 8.0, 4.0).unwrap();
    assert!(matches!(
        attachment.resolve_line(&topo, &source),
        Err(LinkedSketchError::Unbound {
            resolution: Resolution::Stale { .. },
            ..
        })
    ));
    let mut draft = EvolutionDraft::construction();
    for key in solid_entity_keys(&topo, duplicate).unwrap() {
        draft.push(key, EventDraft::Generated { sources: vec![] });
    }
    topo.journal_record_evolution(pending, draft).unwrap();
    let signature =
        remus_topology::naming::EntitySignature::capture_edge(&topo, edge, 1e-7).unwrap();
    let inferred = ExternalLineReference {
        edge: PersistentRef::signature(signature),
        ..source
    };
    assert!(matches!(
        attachment.resolve_line(&topo, &inferred),
        Err(LinkedSketchError::Unbound {
            resolution: Resolution::Ambiguous { .. },
            ..
        })
    ));
}

#[test]
fn malformed_frames_wrong_kinds_and_rotation_refuse() {
    let (mut topo, _, attachment, source) = fixture(1.0);
    let mut frame = attachment.resolve_frame(&topo).unwrap();
    frame.x = Vec3::new(f64::NAN, 0.0, 0.0);
    assert!(matches!(
        PlanarSketchAttachment::new(&topo, attachment.reference().clone(), frame),
        Err(LinkedSketchError::InvalidFrame)
    ));
    assert!(matches!(
        attachment.resolve_line(
            &topo,
            &ExternalLineReference {
                edge: attachment.reference().clone(),
                ..source.clone()
            }
        ),
        Err(LinkedSketchError::InvalidReferenceKind("edge"))
    ));
    let Resolution::Bound { entity, .. } = resolve(&topo, attachment.reference()) else {
        panic!()
    };
    let face = topo.face_id_from_index(entity.index).unwrap();
    let pending = topo.journal_begin("fixture_rotation");
    topo.face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 4.0,
        });
    let mut draft = EvolutionDraft::construction();
    draft.push(entity, EventDraft::Preserved { from: entity });
    topo.journal_record_evolution(pending, draft).unwrap();
    assert!(matches!(
        attachment.resolve_line(&topo, &source),
        Err(LinkedSketchError::UnsupportedAttachment)
    ));
}

#[test]
fn endpoint_identity_survives_reversed_edge_storage_and_mismatch_refuses() {
    let (mut topo, _, attachment, source) = fixture(1.0);
    let before = attachment.resolve_line(&topo, &source).unwrap();
    let Resolution::Bound { entity, .. } = resolve(&topo, &source.edge) else {
        panic!()
    };
    let id = topo.edge_id_from_index(entity.index).unwrap();
    let edge = topo.edge(id).unwrap().clone();
    let pending = topo.journal_begin("fixture_reverse_storage");
    *topo.edge_mut(id).unwrap() = remus_topology::edge::Edge::new(
        edge.end(),
        edge.start(),
        remus_topology::edge::EdgeCurve::Line,
    );
    let mut draft = EvolutionDraft::construction();
    draft.push(entity, EventDraft::Modified { from: entity });
    topo.journal_record_evolution(pending, draft).unwrap();
    let after = attachment.resolve_line(&topo, &source).unwrap();
    assert_eq!(before.start, after.start);
    assert_eq!(before.end, after.end);
    let bad = ExternalLineReference {
        end: source.start.clone(),
        ..source
    };
    assert!(matches!(
        attachment.resolve_line(&topo, &bad),
        Err(LinkedSketchError::EndpointMismatch)
    ));
}

#[test]
fn accepted_frame_roundoff_is_aligned_to_the_face_at_large_scale() {
    let (topo, _, attachment, source) = fixture(1000.0);
    let theta = 5e-10_f64;
    let frame = Frame3 {
        origin: Point3::new(0.0, 0.0, 4000.0),
        x: Vec3::new(theta.cos(), 0.0, -theta.sin()),
        y: Vec3::new(0.0, 1.0, 0.0),
        z: Vec3::new(theta.sin(), 0.0, theta.cos()),
    };
    let attached =
        PlanarSketchAttachment::new(&topo, attachment.reference().clone(), frame).unwrap();
    let resolved = attached.resolve_frame(&topo).unwrap();
    let image = attached.resolve_line(&topo, &source).unwrap();
    for point in [image.start, image.end] {
        let lifted = resolved.origin + resolved.x * point.x() + resolved.y * point.y();
        assert!((lifted.z() - 4000.0).abs() <= remus_math::tolerance::Tolerance::default().linear);
    }
    assert!((image.end.x() - 10000.0).abs() <= 1e-7);
}
