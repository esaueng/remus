//! Integration tests for the persistent-reference resolver (RFC 0003,
//! Stage 2): references anchored in the journal resolve against real
//! modeling history.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::bop::BooleanOp;
use remus_operations::blend_ops::fillet_with_evolution;
use remus_operations::journal_ops::{begin_scoped, boolean_journaled, record_face_evolution};
use remus_operations::primitives::make_box;
use remus_topology::journal::{EntityKind, OpId};
use remus_topology::naming::{Discriminator, PersistentRef, Provenance, Resolution, resolve};
use remus_topology::{SolidId, Topology};

fn fused_boxes(topo: &mut Topology) -> (SolidId, OpId) {
    let a = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let shift = remus_math::mat::Mat4::translation(5.0, 5.0, 5.0);
    remus_operations::transform::transform_solid(topo, b, &shift).unwrap();
    let fused = boolean_journaled(topo, BooleanOp::Fuse, a, b).unwrap();
    (fused.solid, fused.op)
}

#[test]
fn operation_output_refs_bind_live_entities_of_a_real_boolean() {
    let mut topo = Topology::new();
    let (solid, op) = fused_boxes(&mut topo);

    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();

    // The 0th face output of the fuse binds a live face of the result,
    // with construction provenance.
    let r = resolve(
        &topo,
        &PersistentRef::operation_output(op, EntityKind::Face, 0),
    );
    let Resolution::Bound { entity, provenance } = r else {
        panic!("expected a bound face: {r:?}");
    };
    assert_eq!(provenance, Provenance::Construction);
    let id = topo.face_id_from_index(entity.index).unwrap();
    assert!(topo.face(id).is_ok(), "the bound face is live");
    assert!(
        faces.iter().any(|f| f.index() == entity.index),
        "the bound face belongs to the result solid"
    );

    // Every face output of the entry resolves to a distinct live face.
    let mut seen = std::collections::BTreeSet::new();
    for index in 0..faces.len() {
        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, index),
        );
        let Resolution::Bound { entity, .. } = r else {
            panic!("face output {index}: {r:?}");
        };
        assert!(seen.insert(entity.index), "outputs are distinct");
    }
}

#[test]
fn refs_chase_through_a_second_boolean() {
    let mut topo = Topology::new();
    // Every solid is created before journaled history starts — creating
    // one mid-history is an unjournaled mutation and would (correctly)
    // sever everything with a global barrier.
    let cutter = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let (solid, op) = fused_boxes(&mut topo);

    // A second journaled boolean on the same solid: every face reference
    // anchored at the fuse resolves to a typed outcome — bound to live
    // pieces (possibly several, a split) or honestly severed — and never
    // to a retired entity.
    let cut = boolean_journaled(&mut topo, BooleanOp::Cut, solid, cutter).unwrap();

    let result_faces: std::collections::BTreeSet<usize> =
        remus_topology::explorer::solid_faces(&topo, cut.solid)
            .unwrap()
            .iter()
            .map(|id| id.index())
            .collect();

    let mut bound = 0;
    let mut split = 0;
    let mut severed = 0;
    for index in 0..12 {
        let r = resolve(
            &topo,
            &PersistentRef::operation_output(op, EntityKind::Face, index),
        );
        match r {
            Resolution::Bound { entity, .. } => {
                assert!(
                    result_faces.contains(&entity.index),
                    "a bound face must be a live face of the current result"
                );
                bound += 1;
            }
            Resolution::BoundMany { entities, .. } => {
                for entity in entities {
                    assert!(result_faces.contains(&entity.index));
                }
                split += 1;
            }
            Resolution::UnresolvedAcrossOperation { .. } | Resolution::Dangling { .. } => {
                severed += 1;
            }
            other => panic!("face ref {index}: unexpected {other:?}"),
        }
    }
    assert!(bound > 0, "faces away from the cutter stay bound");
    assert_eq!(
        bound + split + severed,
        12,
        "every reference resolves to a typed outcome"
    );
}

#[test]
fn unrelated_solids_do_not_sever_each_others_references() {
    let mut topo = Topology::new();
    // All solids exist before journaled history starts.
    let c = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    let d = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    let shift = remus_math::mat::Mat4::translation(1.5, 1.5, 1.5);
    remus_operations::transform::transform_solid(&mut topo, d, &shift).unwrap();
    let (_, op) = fused_boxes(&mut topo);

    let reference = PersistentRef::operation_output(op, EntityKind::Face, 0);
    let before = resolve(&topo, &reference);
    assert!(matches!(before, Resolution::Bound { .. }));

    // A journaled boolean on the two completely unrelated solids: its
    // scope does not contain the fused solid's entities, so the reference
    // carries through unchanged.
    boolean_journaled(&mut topo, BooleanOp::Fuse, c, d).unwrap();
    assert_eq!(
        resolve(&topo, &reference),
        before,
        "an operation on other solids must not sever this reference"
    );
}

#[test]
fn a_faces_only_blend_entry_severs_edge_references_honestly() {
    let mut topo = Topology::new();
    let (solid, op) = fused_boxes(&mut topo);

    // Anchor an edge reference at the fuse.
    let edge_ref = PersistentRef::operation_output(op, EntityKind::Edge, 0);
    assert!(matches!(
        resolve(&topo, &edge_ref),
        Resolution::Bound { .. }
    ));

    // A journaled fillet records face evolution only; its scope covers
    // the solid's edges, so the edge reference fails closed naming the
    // fillet rather than resolving through records that do not exist.
    let edges = remus_topology::explorer::solid_edges(&topo, solid).unwrap();
    let pending = begin_scoped(&mut topo, "fillet", &[solid]).unwrap();
    let (result, map) = fillet_with_evolution(&mut topo, solid, &[edges[0]], 1.0).unwrap();
    let fillet_op = record_face_evolution(&mut topo, pending, &map, &[result.solid]).unwrap();

    let r = resolve(&topo, &edge_ref);
    assert_eq!(
        r,
        Resolution::UnresolvedAcrossOperation {
            op: fillet_op,
            kind: "fillet".to_owned()
        },
        "absent edge claims are gaps, not implicit preservation"
    );
}

#[test]
fn discriminators_filter_on_real_geometry() {
    let mut topo = Topology::new();
    let (_, op) = fused_boxes(&mut topo);

    // Every face of a box fuse is planar: the plane discriminator keeps
    // the binding, the cylinder discriminator reports itself.
    let plane_ref = PersistentRef::operation_output(op, EntityKind::Face, 0)
        .with_discriminator(Discriminator::SurfaceType("plane".into()));
    assert!(matches!(
        resolve(&topo, &plane_ref),
        Resolution::Bound { .. }
    ));

    let cylinder_ref = PersistentRef::operation_output(op, EntityKind::Face, 0)
        .with_discriminator(Discriminator::SurfaceType("cylinder".into()));
    let r = resolve(&topo, &cylinder_ref);
    assert!(
        matches!(r, Resolution::NoMatch { ref reason } if reason.contains("surface_type:cylinder")),
        "{r:?}"
    );

    let line_ref = PersistentRef::operation_output(op, EntityKind::Edge, 0)
        .with_discriminator(Discriminator::CurveType("line".into()));
    assert!(matches!(
        resolve(&topo, &line_ref),
        Resolution::Bound { .. }
    ));
}

#[test]
fn resolution_is_deterministic_across_identical_histories() {
    let run = || {
        let mut topo = Topology::new();
        let (solid, op) = fused_boxes(&mut topo);
        let cutter = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        boolean_journaled(&mut topo, BooleanOp::Cut, solid, cutter).unwrap();
        (0..12)
            .map(|index| {
                resolve(
                    &topo,
                    &PersistentRef::operation_output(op, EntityKind::Face, index),
                )
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(
        run(),
        run(),
        "identical history plus identical reference must resolve identically"
    );
}

// ── Signature tier (RFC 0003, Stage 3) ──────────────────────────────────

const QUANTUM: f64 = 1e-7;

#[test]
fn every_box_face_signature_recovers_its_own_face() {
    use remus_topology::naming::EntitySignature;

    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 20.0, 30.0).unwrap();

    // An unequal box: all six planes are distinct, so every face's
    // signature recovers exactly that face, marked inferred.
    for face in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
        let signature = EntitySignature::capture_face(&topo, face, QUANTUM).unwrap();
        let r = resolve(&topo, &PersistentRef::signature(signature));
        let Resolution::Bound { entity, provenance } = r else {
            panic!("face {face:?}: {r:?}");
        };
        assert_eq!(entity.index, face.index());
        assert_eq!(provenance, Provenance::Inferred);
    }
}

#[test]
fn coplanar_twin_faces_are_ambiguous_and_discriminators_cannot_rescue_them() {
    use remus_topology::naming::EntitySignature;

    let mut topo = Topology::new();
    let a = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let shift = remus_math::mat::Mat4::translation(20.0, 0.0, 0.0);
    remus_operations::transform::transform_solid(&mut topo, b, &shift).unwrap();

    // The two top faces lie on the same plane (z = 10) with identical
    // adjacency: geometrically indistinguishable, so the signature must
    // report both — never first-match one of them.
    let top_of = |topo: &Topology, solid| {
        remus_topology::explorer::solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&f| {
                matches!(
                    topo.face(f).unwrap().surface(),
                    remus_topology::face::FaceSurface::Plane { normal, d }
                        if normal.z() > 0.9 && (d - 10.0).abs() < 1e-9
                )
            })
            .unwrap()
    };
    let top_a = top_of(&topo, a);
    let top_b = top_of(&topo, b);

    let signature = EntitySignature::capture_face(&topo, top_a, QUANTUM).unwrap();
    let r = resolve(&topo, &PersistentRef::signature(signature.clone()));
    let Resolution::Ambiguous { candidates, .. } = r else {
        panic!("coplanar twins must be ambiguous: {r:?}");
    };
    assert_eq!(candidates.len(), 2);
    assert!(candidates.contains(&remus_topology::journal::EntityKey::face(top_a.index())));
    assert!(candidates.contains(&remus_topology::journal::EntityKey::face(top_b.index())));

    // A type discriminator keeps both candidates (both are planes): the
    // ambiguity survives, fail-closed, rather than being "resolved" by
    // an arbitrary pick.
    let discriminated = PersistentRef::signature(signature)
        .with_discriminator(Discriminator::SurfaceType("plane".into()));
    assert!(matches!(
        resolve(&topo, &discriminated),
        Resolution::Ambiguous { .. }
    ));
}

#[test]
fn cylinder_signature_carries_quantized_radius() {
    use remus_topology::naming::EntitySignature;

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_cylinder(&mut topo, 5.0, 12.0).unwrap();

    let lateral = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .unwrap();

    let signature = EntitySignature::capture_face(&topo, lateral, QUANTUM).unwrap();
    assert_eq!(signature.type_tag, "cylinder");
    // The radius is the last parameter: 5.0 in units of the 1e-7 quantum.
    assert_eq!(signature.params.last().copied(), Some(50_000_000));

    let r = resolve(&topo, &PersistentRef::signature(signature));
    assert_eq!(
        r,
        Resolution::Bound {
            entity: remus_topology::journal::EntityKey::face(lateral.index()),
            provenance: Provenance::Inferred
        }
    );
}

#[test]
fn signatures_recover_after_journal_severing() {
    use remus_topology::naming::EntitySignature;

    // The recovery story: an edge reference severed by a faces-only
    // fillet entry (tested above) can be re-anchored by signature —
    // knowingly, with inferred provenance — against the current model.
    let mut topo = Topology::new();
    let (solid, _) = fused_boxes(&mut topo);
    let edges = remus_topology::explorer::solid_edges(&topo, solid).unwrap();
    let pending = begin_scoped(&mut topo, "fillet", &[solid]).unwrap();
    let (result, map) = fillet_with_evolution(&mut topo, solid, &[edges[0]], 1.0).unwrap();
    record_face_evolution(&mut topo, pending, &map, &[result.solid]).unwrap();

    // Capture a surviving edge's signature from the current model and
    // resolve it: the signature tier answers where the journal cannot,
    // and says it inferred the answer.
    let surviving = remus_topology::explorer::solid_edges(&topo, result.solid).unwrap();
    let signature = EntitySignature::capture_edge(&topo, surviving[3], QUANTUM).unwrap();
    let r = resolve(&topo, &PersistentRef::signature(signature));
    match r {
        Resolution::Bound { provenance, .. } => assert_eq!(provenance, Provenance::Inferred),
        Resolution::Ambiguous { .. } => {} // several congruent edges: honest
        other => panic!("recovery must answer or refuse loudly: {other:?}"),
    }
}

// ── Journal-driven attribute integration (RFC 0003, Stage 4) ────────────

#[test]
fn names_ride_construction_lineage_through_a_real_boolean() {
    use remus_topology::attributes::EntityAttributes;
    use remus_topology::naming::resolve_face_attributes;

    let mut topo = Topology::new();
    let a = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let shift = remus_math::mat::Mat4::translation(5.0, 5.0, 5.0);
    remus_operations::transform::transform_solid(&mut topo, b, &shift).unwrap();

    // Name every face of operand A before the operation.
    for (i, face) in remus_topology::explorer::solid_faces(&topo, a)
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

    let fused = boolean_journaled(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let report = topo.propagate_attributes_for_op(fused.op, false).unwrap();
    assert!(
        !report.refused_inferred,
        "GFA history is construction-derived"
    );
    assert!(
        report.carried > 0,
        "operand A's names must ride onto the result's carried faces"
    );

    // Every attributed result face carries an unmodified operand-A name;
    // section-generated faces stay bare.
    let mut named = 0;
    for face in remus_topology::explorer::solid_faces(&topo, fused.solid).unwrap() {
        if let Some(attributes) = topo.attributes().face(face) {
            named += 1;
            assert!(
                attributes
                    .name
                    .as_deref()
                    .unwrap_or("")
                    .starts_with("a-face-"),
                "carried names are never synthesized or suffixed"
            );
        }
    }
    assert_eq!(named, report.carried);

    // The reference-keyed read: an operation-output reference whose bound
    // face is attributed yields the name through resolution.
    let mut read_through_ref = 0;
    for index in 0..12 {
        let reference = PersistentRef::operation_output(fused.op, EntityKind::Face, index);
        if let Ok(bound) = resolve_face_attributes(&topo, &reference) {
            for (_, attributes) in bound {
                if attributes.is_some() {
                    read_through_ref += 1;
                }
            }
        }
    }
    assert!(
        read_through_ref > 0,
        "attributes must be readable through persistent references"
    );
}
