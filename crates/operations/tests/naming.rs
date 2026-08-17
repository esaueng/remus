//! Integration tests for the persistent-reference resolver (RFC 0003,
//! Stage 2): references anchored in the journal resolve against real
//! modeling history.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use brepkit_algo::bop::BooleanOp;
use brepkit_operations::blend_ops::fillet_with_evolution;
use brepkit_operations::journal_ops::{begin_scoped, boolean_journaled, record_face_evolution};
use brepkit_operations::primitives::make_box;
use brepkit_topology::journal::{EntityKind, OpId};
use brepkit_topology::naming::{Discriminator, PersistentRef, Provenance, Resolution, resolve};
use brepkit_topology::{SolidId, Topology};

fn fused_boxes(topo: &mut Topology) -> (SolidId, OpId) {
    let a = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let shift = brepkit_math::mat::Mat4::translation(5.0, 5.0, 5.0);
    brepkit_operations::transform::transform_solid(topo, b, &shift).unwrap();
    let fused = boolean_journaled(topo, BooleanOp::Fuse, a, b).unwrap();
    (fused.solid, fused.op)
}

#[test]
fn operation_output_refs_bind_live_entities_of_a_real_boolean() {
    let mut topo = Topology::new();
    let (solid, op) = fused_boxes(&mut topo);

    let faces = brepkit_topology::explorer::solid_faces(&topo, solid).unwrap();

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
        brepkit_topology::explorer::solid_faces(&topo, cut.solid)
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
    let shift = brepkit_math::mat::Mat4::translation(1.5, 1.5, 1.5);
    brepkit_operations::transform::transform_solid(&mut topo, d, &shift).unwrap();
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
    let edges = brepkit_topology::explorer::solid_edges(&topo, solid).unwrap();
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
