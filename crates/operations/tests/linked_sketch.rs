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

mod ellipse_inputs {
    use super::*;
    use remus_math::curves::Ellipse3D;
    use remus_operations::linked_sketch::ExternalEllipseReference;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;

    fn add_source(
        topo: &mut Topology,
        ellipse: Ellipse3D,
        start: f64,
        end: f64,
    ) -> (remus_topology::EdgeId, ExternalEllipseReference) {
        add_source_with_vertex_tolerance(topo, ellipse, start, end, 1e-7)
    }

    fn add_source_with_vertex_tolerance(
        topo: &mut Topology,
        ellipse: Ellipse3D,
        start: f64,
        end: f64,
        vertex_tolerance: f64,
    ) -> (remus_topology::EdgeId, ExternalEllipseReference) {
        let pending = topo.journal_begin("ellipse_fixture");
        let vertex = topo.add_vertex(Vertex::new(ellipse.evaluate(start), vertex_tolerance));
        let mut edge =
            Edge::with_tolerance(vertex, vertex, EdgeCurve::Ellipse(ellipse), Some(1e-7));
        edge.set_trim(Some((start, end)));
        let edge = topo.add_edge(edge);
        let mut draft = EvolutionDraft::construction();
        for key in [
            EntityKey::vertex(vertex.index()),
            EntityKey::edge(edge.index()),
        ] {
            draft.push(key, EventDraft::Generated { sources: vec![] });
        }
        let op = topo.journal_record_evolution(pending, draft).unwrap();
        (
            edge,
            ExternalEllipseReference {
                edge: reference(topo, op, EntityKey::edge(edge.index())),
            },
        )
    }

    #[test]
    fn complete_ellipse_images_preserve_native_axes_sense_and_signed_seams() {
        for scale in [0.001, 1.0, 1000.0] {
            for normal_sign in [-1.0, 1.0] {
                for attachment_sign in [-1.0, 1.0] {
                    for axis_sign in [-1.0, 1.0] {
                        for (start, span) in [
                            (0.0, std::f64::consts::TAU),
                            (0.37, std::f64::consts::TAU),
                            (7.0, -std::f64::consts::TAU),
                        ] {
                            let (mut topo, solid, base, _) = fixture(scale);
                            let mut frame = base.resolve_frame(&topo).unwrap();
                            frame.y = frame.y * attachment_sign;
                            frame.z = frame.z * attachment_sign;
                            let attachment =
                                PlanarSketchAttachment::new(&topo, base.reference().clone(), frame)
                                    .unwrap();
                            let angle = 0.37_f64;
                            let normal = Vec3::new(0.0, 0.0, normal_sign);
                            let u = Vec3::new(angle.cos(), angle.sin(), 0.0) * axis_sign;
                            let ellipse = Ellipse3D::with_axes(
                                Point3::new(5.0 * scale, 4.0 * scale, -scale),
                                normal,
                                3.0 * scale,
                                2.0 * scale,
                                u,
                                normal.cross(u),
                            )
                            .unwrap();
                            let (_, source) =
                                add_source(&mut topo, ellipse.clone(), start, start + span);
                            let ticks = topo.mutation_ticks();
                            let image = attachment.resolve_ellipse(&topo, &source).unwrap();
                            assert_eq!(topo.mutation_ticks(), ticks);
                            assert!((image.center.x() / scale - 5.0).abs() < 1e-9);
                            assert!(
                                (image.center.y() / scale - 4.0 * attachment_sign).abs() < 1e-9
                            );
                            assert_eq!(image.a.to_bits(), (3.0 * scale).to_bits());
                            assert_eq!(image.b.to_bits(), (2.0 * scale).to_bits());
                            assert!((image.angle - angle * attachment_sign).sin().abs() < 1e-12);
                            // Independent implicit ellipse equation on projected native points.
                            for t in [0.0_f64, 0.41, 1.7, 3.9, 5.8] {
                                let point = ellipse.evaluate(t) - frame.origin;
                                let x = point.dot(frame.x) - image.center.x();
                                let y = point.dot(frame.y) - image.center.y();
                                let u = x * image.angle.cos() + y * image.angle.sin();
                                let v = -x * image.angle.sin() + y * image.angle.cos();
                                assert!(
                                    ((u / image.a).powi(2) + (v / image.b).powi(2) - 1.0).abs()
                                        < 1e-12
                                );
                            }
                            let top = solid_faces(&topo, solid).unwrap().into_iter().find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.99)).unwrap();
                            move_faces_journaled(&mut topo, solid, &[top], scale).unwrap();
                            let after = attachment.resolve_ellipse(&topo, &source).unwrap();
                            assert_eq!(image.center, after.center);
                            assert_eq!(image.a.to_bits(), after.a.to_bits());
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn ellipse_noncanonical_axes_unsound_frames_and_incomplete_ranges_refuse() {
        for (a, b, normal, u, v, start, end) in [
            (
                2.0,
                3.0,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                std::f64::consts::TAU,
            ),
            (
                3.0,
                3.0,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                std::f64::consts::TAU,
            ),
            (
                3.0,
                3.0 - 5e-8,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                std::f64::consts::TAU,
            ),
            (
                3.0,
                2.0,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                std::f64::consts::PI,
            ),
            (
                3.0,
                2.0,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                2.0 * std::f64::consts::TAU,
            ),
            (
                3.0,
                2.0,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                0.0,
                std::f64::consts::TAU,
            ),
            (
                3.0,
                2.0,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.01, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                std::f64::consts::TAU,
            ),
            (
                3.0,
                2.0,
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, -1.0),
                0.0,
                std::f64::consts::TAU,
            ),
            (
                f64::INFINITY,
                2.0,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                std::f64::consts::TAU,
            ),
            // Angular roundoff cannot consume more than the linear model budget.
            (
                1e7,
                2e6,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0 + 1e-13, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                0.0,
                std::f64::consts::TAU,
            ),
        ] {
            let (mut topo, _, attachment, _) = fixture(1.0);
            let ellipse =
                Ellipse3D::with_axes(Point3::new(0.0, 0.0, 0.0), normal, a, b, u, v).unwrap();
            let (_, source) = add_source(&mut topo, ellipse, start, end);
            let ticks = topo.mutation_ticks();
            assert!(matches!(
                attachment.resolve_ellipse(&topo, &source),
                Err(LinkedSketchError::UnsupportedCurve)
            ));
            assert_eq!(topo.mutation_ticks(), ticks);
        }
    }

    #[test]
    fn ellipse_seam_endpoints_and_construction_identity_are_authoritative() {
        let (mut topo, _, attachment, line) = fixture(1.0);
        let ellipse = Ellipse3D::new(
            Point3::new(1.0, 2.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            2.0,
        )
        .unwrap();
        // The start endpoint matches exactly; the opposite endpoint is wrong.
        let (id, source) =
            add_source_with_vertex_tolerance(&mut topo, ellipse, 0.0, std::f64::consts::TAU, 0.0);
        assert!(topo.edge(id).unwrap().strict_domain().is_ok());
        assert!(matches!(
            attachment.resolve_ellipse(&topo, &source),
            Err(LinkedSketchError::UnsupportedCurve)
        ));
        let pending = topo.journal_begin("fixture_missing_ellipse_trim");
        topo.edge_mut(id).unwrap().set_trim(None);
        let key = EntityKey::edge(id.index());
        let mut draft = EvolutionDraft::construction();
        draft.push(key, EventDraft::Modified { from: key });
        topo.journal_record_evolution(pending, draft).unwrap();
        assert!(matches!(
            attachment.resolve_ellipse(&topo, &source),
            Err(LinkedSketchError::UnsupportedCurve)
        ));
        assert!(matches!(
            attachment.resolve_ellipse(&topo, &ExternalEllipseReference { edge: line.edge }),
            Err(LinkedSketchError::UnsupportedCurve)
        ));
        assert!(matches!(
            attachment.resolve_ellipse(
                &topo,
                &ExternalEllipseReference {
                    edge: attachment.reference().clone()
                }
            ),
            Err(LinkedSketchError::InvalidReferenceKind("edge"))
        ));
    }
}

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

mod circles {
    use super::*;
    use remus_math::curves::Circle3D;
    use remus_math::mat::Mat4;
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::journal_ops::resize_cylindrical_face_journaled;
    use remus_operations::linked_sketch::ExternalCircleReference;
    use remus_topology::edge::EdgeCurve;

    fn cylinder(topo: &Topology, solid: remus_topology::SolidId) -> remus_topology::FaceId {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap()
    }

    fn setup(
        scale: f64,
        placement: usize,
    ) -> (
        Topology,
        remus_topology::SolidId,
        PlanarSketchAttachment,
        ExternalCircleReference,
    ) {
        let mut topo = Topology::new();
        let box_id = make_box(&mut topo, 10.0 * scale, 8.0 * scale, 4.0 * scale).unwrap();
        let tool = remus_operations::primitives::make_cylinder(&mut topo, 2.0 * scale, 6.0 * scale)
            .unwrap();
        remus_operations::transform::transform_solid(
            &mut topo,
            tool,
            &Mat4([
                [1.0, 0.0, 0.0, 5.0 * scale],
                [0.0, 1.0, 0.0, 4.0 * scale],
                [0.0, 0.0, 1.0, -scale],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        )
        .unwrap();
        let solid = boolean(&mut topo, BooleanOp::Cut, box_id, tool).unwrap();
        let (origin, x, y, z) = if placement == 2 {
            remus_operations::transform::transform_solid(
                &mut topo,
                solid,
                &Mat4([
                    [1.0, 0.0, 0.0, 3.0 * scale],
                    [0.0, 0.0, -1.0, -5.0 * scale],
                    [0.0, 1.0, 0.0, 7.0 * scale],
                    [0.0, 0.0, 0.0, 1.0],
                ]),
            )
            .unwrap();
            (
                Point3::new(3.0 * scale, -9.0 * scale, 7.0 * scale),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, -1.0, 0.0),
            )
        } else {
            let shift = if placement == 1 {
                Vec3::new(3.0 * scale, -5.0 * scale, 7.0 * scale)
            } else {
                Vec3::new(0.0, 0.0, 0.0)
            };
            if placement == 1 {
                remus_operations::transform::transform_solid(
                    &mut topo,
                    solid,
                    &Mat4([
                        [1.0, 0.0, 0.0, shift.x()],
                        [0.0, 1.0, 0.0, shift.y()],
                        [0.0, 0.0, 1.0, shift.z()],
                        [0.0, 0.0, 0.0, 1.0],
                    ]),
                )
                .unwrap();
            }
            (
                Point3::new(0.0, 0.0, 4.0 * scale) + shift,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
        };
        let bore = cylinder(&topo, solid);
        let seeded =
            resize_cylindrical_face_journaled(&mut topo, solid, bore, 2.5 * scale).unwrap();
        let top = solid_faces(&topo, seeded.solid).unwrap().into_iter().find(|&f| {
            matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { normal, d } if normal.dot(z)>0.99 && (*d-normal.dot(origin-Point3::new(0.0,0.0,0.0))).abs()<1e-8*scale)
        }).unwrap();
        let edge = topo
            .face(top)
            .unwrap()
            .inner_wires()
            .iter()
            .flat_map(|w| topo.wire(*w).unwrap().edges())
            .find(|oe| matches!(topo.edge(oe.edge()).unwrap().curve(), EdgeCurve::Circle(_)))
            .unwrap()
            .edge();
        let attachment = PlanarSketchAttachment::new(
            &topo,
            reference(&topo, seeded.op, EntityKey::face(top.index())),
            Frame3 { origin, x, y, z },
        )
        .unwrap();
        let source = ExternalCircleReference {
            edge: reference(&topo, seeded.op, EntityKey::edge(edge.index())),
        };
        (topo, seeded.solid, attachment, source)
    }

    fn edge(topo: &Topology, source: &ExternalCircleReference) -> remus_topology::EdgeId {
        let Resolution::Bound { entity, .. } = resolve(topo, &source.edge) else {
            panic!("fixture circle")
        };
        topo.edge_id_from_index(entity.index).unwrap()
    }

    #[test]
    fn complete_circle_follows_radius_growth_and_parallel_cap_move() {
        for scale in [0.001, 1.0, 1000.0] {
            for placement in 0..3 {
                let (mut topo, solid, attachment, source) = setup(scale, placement);
                let before = attachment.resolve_circle(&topo, &source).unwrap();
                let original_frame = attachment.resolve_frame(&topo).unwrap();
                assert!((before.center.x() - 5.0 * scale).abs() < 1e-8 * scale);
                assert!((before.center.y() - 4.0 * scale).abs() < 1e-8 * scale);
                assert!((before.radius - 2.5 * scale).abs() < 1e-8 * scale);
                let bore = cylinder(&topo, solid);
                let grown =
                    resize_cylindrical_face_journaled(&mut topo, solid, bore, 3.0 * scale).unwrap();
                let after = attachment.resolve_circle(&topo, &source).unwrap();
                assert!((after.radius - 3.0 * scale).abs() < 1e-8 * scale);
                assert!((after.center - before.center).length() < 1e-8 * scale);
                let Resolution::Bound { entity, .. } = resolve(&topo, attachment.reference())
                else {
                    panic!("attachment")
                };
                let top = topo.face_id_from_index(entity.index).unwrap();
                let moved = move_faces_journaled(&mut topo, grown.solid, &[top], scale).unwrap();
                let frame = attachment.resolve_frame(&topo).unwrap();
                let image = attachment.resolve_circle(&topo, &source).unwrap();
                assert!((image.center - before.center).length() < 1e-8 * scale);
                assert!((image.radius - 3.0 * scale).abs() < 1e-8 * scale);
                assert!(
                    (frame.origin - (original_frame.origin + original_frame.z * scale)).length()
                        < 1e-8 * scale
                );
                assert!(
                    remus_operations::validate::validate_solid(&topo, moved.solid)
                        .unwrap()
                        .is_valid()
                );
                assert!(
                    (remus_operations::measure::solid_volume(&topo, moved.solid, 0.01 * scale)
                        .unwrap()
                        / scale.powi(3)
                        - (80.0 - 9.0 * std::f64::consts::PI) * 5.0)
                        .abs()
                        < 1e-7
                );
            }
        }
    }

    #[test]
    fn complete_circle_seam_phase_and_reversed_carrier_keep_the_same_image() {
        let (mut topo, _, attachment, source) = setup(1.0, 0);
        let expected = attachment.resolve_circle(&topo, &source).unwrap();
        let id = edge(&topo, &source);
        let old = topo.edge(id).unwrap().clone();
        let EdgeCurve::Circle(circle) = old.curve() else {
            panic!("native circle")
        };
        let (a, b) = old.strict_domain().unwrap();
        let phase = 0.375_f64;
        let rotated = Circle3D::with_axes(
            circle.center(),
            circle.normal(),
            circle.radius(),
            circle.u_axis() * phase.cos() + circle.v_axis() * phase.sin(),
            circle.v_axis() * phase.cos() - circle.u_axis() * phase.sin(),
        )
        .unwrap();
        let pending = topo.journal_begin("fixture_circle_phase");
        let changed = topo.edge_mut(id).unwrap();
        changed.set_curve(EdgeCurve::Circle(rotated.reversed()));
        changed.set_trim(Some((phase - a, phase - b)));
        let mut draft = EvolutionDraft::construction();
        let key = EntityKey::edge(id.index());
        draft.push(key, EventDraft::Modified { from: key });
        topo.journal_record_evolution(pending, draft).unwrap();
        let result = attachment.resolve_circle(&topo, &source).unwrap();
        assert_eq!(result.center, expected.center);
        assert_eq!(result.radius.to_bits(), expected.radius.to_bits());
    }

    #[test]
    fn incomplete_unsound_and_oblique_circle_inputs_refuse_atomically() {
        for mutation in 0..15 {
            let (mut topo, _, attachment, source) = setup(1.0, 0);
            let id = edge(&topo, &source);
            let original = topo.edge(id).unwrap().clone();
            let EdgeCurve::Circle(circle) = original.curve().clone() else {
                panic!("circle")
            };
            let pending = topo.journal_begin("fixture_circle_poison");
            match mutation {
                0 => topo.edge_mut(id).unwrap().set_trim(None),
                1 => topo
                    .edge_mut(id)
                    .unwrap()
                    .set_trim(Some((0.0, std::f64::consts::PI))),
                2 => topo
                    .edge_mut(id)
                    .unwrap()
                    .set_trim(Some((0.0, 2.0 * std::f64::consts::TAU))),
                3 => {
                    let twin = topo.add_vertex(topo.vertex(original.start()).unwrap().clone());
                    topo.edge_mut(id).unwrap().set_end(twin);
                }
                4..=6 => {
                    let axis = if mutation == 4 {
                        Vec3::new(2.0, 0.0, 0.0)
                    } else if mutation == 5 {
                        Vec3::new(f64::NAN, 0.0, 0.0)
                    } else {
                        Vec3::new(-1.0, 0.0, 0.0)
                    };
                    let poisoned = Circle3D::with_axes(
                        circle.center(),
                        circle.normal(),
                        circle.radius(),
                        axis,
                        circle.v_axis(),
                    )
                    .unwrap();
                    let e = topo.edge_mut(id).unwrap();
                    e.set_curve(EdgeCurve::Circle(poisoned));
                    e.set_trim(original.trim());
                }
                7 => {
                    let e = topo.edge_mut(id).unwrap();
                    e.set_curve(EdgeCurve::Circle(
                        Circle3D::new(circle.center(), Vec3::new(0.0, 1.0, 1.0), circle.radius())
                            .unwrap(),
                    ));
                    e.set_trim(original.trim());
                }
                8 => {
                    let point = Point3::new(99.0, 99.0, 99.0);
                    *topo.vertex_mut(original.start()).unwrap() =
                        remus_topology::vertex::Vertex::new(point, 1e-7);
                }
                9 => {
                    *topo.vertex_mut(original.start()).unwrap() =
                        remus_topology::vertex::Vertex::new(Point3::new(f64::NAN, 0.0, 0.0), 1e-7);
                }
                10 => topo.edge_mut(id).unwrap().set_curve(EdgeCurve::Line),
                11..=13 => {
                    let radius = if mutation == 11 {
                        f64::NAN
                    } else if mutation == 12 {
                        f64::INFINITY
                    } else {
                        circle.radius()
                    };
                    let poisoned = Circle3D::with_axes(
                        if mutation == 13 {
                            Point3::new(f64::NAN, 0.0, 0.0)
                        } else {
                            circle.center()
                        },
                        circle.normal(),
                        radius,
                        circle.u_axis(),
                        circle.v_axis(),
                    )
                    .unwrap();
                    let changed = topo.edge_mut(id).unwrap();
                    changed.set_curve(EdgeCurve::Circle(poisoned));
                    changed.set_trim(original.trim());
                }
                _ => {
                    let poisoned = Circle3D::new_with_ref(
                        circle.center(),
                        Vec3::new(0.0, 5e-13, 1.0),
                        circle.radius(),
                        circle.u_axis(),
                    )
                    .unwrap();
                    let changed = topo.edge_mut(id).unwrap();
                    changed.set_curve(EdgeCurve::Circle(poisoned));
                    changed.set_trim(original.trim());
                }
            }
            let mut draft = EvolutionDraft::construction();
            let key = EntityKey::edge(id.index());
            draft.push(key, EventDraft::Modified { from: key });
            topo.journal_record_evolution(pending, draft).unwrap();
            let before = format!("{topo:?}");
            assert!(
                matches!(
                    attachment.resolve_circle(&topo, &source),
                    Err(LinkedSketchError::UnsupportedCurve)
                ),
                "mutation={mutation}"
            );
            assert_eq!(format!("{topo:?}"), before);
        }
    }
    #[test]
    fn complete_circle_construction_references_round_trip_with_exact_arena_history() {
        let (mut topo, solid, attachment, source) = setup(1.0, 1);
        let face = cylinder(&topo, solid);
        let edited = resize_cylindrical_face_journaled(&mut topo, solid, face, 3.0).unwrap();
        let frame = attachment.resolve_frame(&topo).unwrap();
        let expected = attachment.resolve_circle(&topo, &source).unwrap();
        let bytes = remus_io::arena_io::serialize_solid(&topo, edited.solid).unwrap();
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
        let image = restored_attachment
            .resolve_circle(
                &restored,
                &ExternalCircleReference {
                    edge: codec(&source.edge),
                },
            )
            .unwrap();
        assert_eq!(image.center, expected.center);
        assert_eq!(image.radius.to_bits(), expected.radius.to_bits());
    }
    #[test]
    fn complete_circle_radius_has_no_model_space_cutoff() {
        for radius in [5e-8, f64::MIN_POSITIVE] {
            let (mut topo, _, attachment, _) = super::fixture(1.0);
            let pending = topo.journal_begin("tiny_complete_circle");
            let edge = remus_topology::builder::make_circle_edge(
                &mut topo,
                Point3::new(1.0, 2.0, 4.0),
                Vec3::new(0.0, 0.0, 1.0),
                radius,
                1e-7,
            )
            .unwrap();
            let mut draft = EvolutionDraft::construction();
            draft.push(
                EntityKey::edge(edge.index()),
                EventDraft::Generated { sources: vec![] },
            );
            let op = topo.journal_record_evolution(pending, draft).unwrap();
            let source = ExternalCircleReference {
                edge: reference(&topo, op, EntityKey::edge(edge.index())),
            };
            let image = attachment.resolve_circle(&topo, &source).unwrap();
            assert_eq!(image.radius.to_bits(), radius.to_bits());
            assert_eq!(image.center, remus_math::vec::Point2::new(1.0, 2.0));
        }
    }
}
