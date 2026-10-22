//! Copy compaction preserves the exact document while discarding session debris.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::arena_io::{
    CompactionRoots, compact_document, compact_document_with_limit,
    serialize_body_document_with_wires,
};
use remus_math::vec::Point3;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::attributes::EntityAttributes;
use remus_topology::compound::Compound;
use remus_topology::explorer::solid_faces;
use remus_topology::vertex::Vertex;

#[test]
fn repeated_restore_debris_is_not_copied_and_source_snapshots_survive() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let snapshot = source.clone();
    let mut retired = Vec::new();
    for _ in 0..200 {
        let vertex = source.add_vertex(Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));
        retired.push(vertex);
        source.restore_preserving_handle_slots(&snapshot);
    }
    let source_slots = source.allocated_slot_count();
    assert!(source_slots >= snapshot.allocated_slot_count() + 200);
    assert!(retired.iter().all(|&id| source.vertex(id).is_err()));
    let before = serialize_body_document_with_wires(&source, &[solid], &[], &[], &[]).unwrap();

    let mut compact = compact_document(
        &source,
        CompactionRoots {
            solids: &[solid],
            ..CompactionRoots::default()
        },
    )
    .unwrap();

    assert_eq!(
        compact.topology.allocated_slot_count(),
        snapshot.allocated_slot_count()
    );
    assert_eq!(compact.roots.solids.len(), 1);
    assert_eq!(
        serialize_body_document_with_wires(&compact.topology, &compact.roots.solids, &[], &[], &[])
            .unwrap(),
        before,
    );
    let volume =
        remus_operations::measure::solid_volume(&compact.topology, compact.roots.solids[0], 0.01)
            .unwrap();
    assert!((volume - 24.0).abs() < 1e-10);
    assert_eq!(source.allocated_slot_count(), source_slots);
    assert_eq!(
        serialize_body_document_with_wires(&source, &[solid], &[], &[], &[]).unwrap(),
        before
    );
    assert!(retired.iter().all(|&id| source.vertex(id).is_err()));

    // Both owners remain independent; a new allocation in either never
    // reissues a retired source handle, and the old snapshot still restores.
    let compact_slot_count = compact.topology.allocated_slot_count();
    let next = source.add_vertex(Vertex::new(Point3::new(9.0, 8.0, 7.0), 1e-7));
    assert!(retired.iter().all(|&old| old != next));
    source.restore_preserving_handle_slots(&snapshot);
    assert!(source.vertex(next).is_err());
    assert_eq!(compact.topology.allocated_slot_count(), compact_slot_count);
    compact
        .topology
        .delete_solid(compact.roots.solids[0])
        .unwrap();
    assert!(source.solid(solid).is_ok());
    assert!(snapshot.solid(solid).is_ok());
}

#[test]
fn mixed_roots_keep_shared_subtrees_aliases_attributes_and_exact_authority() {
    let mut source = Topology::new();
    // Unselected but live topology must also be omitted.
    let unrelated = make_box(&mut source, 99.0, 98.0, 97.0).unwrap();
    let solid = make_cylinder(&mut source, 2.0, 5.0).unwrap();
    let face = solid_faces(&source, solid).unwrap()[0];
    source
        .set_solid_attributes(
            solid,
            EntityAttributes {
                name: Some("selected".into()),
                ..EntityAttributes::default()
            },
        )
        .unwrap();
    source
        .set_face_attributes(
            face,
            EntityAttributes {
                name: Some("shared face".into()),
                ..EntityAttributes::default()
            },
        )
        .unwrap();
    let sheet = remus_operations::sew::make_sheet_body(&mut source, &[face]).unwrap();
    let wire = source.face(face).unwrap().outer_wire();
    let compound = source.add_compound(Compound::new(vec![solid, solid]));
    let before = serialize_body_document_with_wires(
        &source,
        &[solid, solid],
        &[sheet, sheet],
        &[wire, wire],
        &[compound, compound],
    )
    .unwrap();

    let compact = compact_document(
        &source,
        CompactionRoots {
            solids: &[solid, solid],
            sheets: &[sheet, sheet],
            wires: &[wire, wire],
            compounds: &[compound, compound],
        },
    )
    .unwrap();
    assert_eq!(compact.roots.solids[0], compact.roots.solids[1]);
    assert_eq!(compact.roots.sheets[0], compact.roots.sheets[1]);
    assert_eq!(compact.roots.wires[0], compact.roots.wires[1]);
    assert_eq!(compact.roots.compounds[0], compact.roots.compounds[1]);
    assert_eq!(compact.topology.num_solids(), 1);
    let members = compact
        .topology
        .compound(compact.roots.compounds[0])
        .unwrap()
        .solids();
    assert_eq!(members, &[compact.roots.solids[0], compact.roots.solids[0]]);
    let copied_faces = solid_faces(&compact.topology, compact.roots.solids[0]).unwrap();
    assert_eq!(
        compact
            .topology
            .shell(compact.roots.sheets[0])
            .unwrap()
            .faces(),
        &[copied_faces[0]]
    );
    assert_eq!(
        compact.topology.face(copied_faces[0]).unwrap().outer_wire(),
        compact.roots.wires[0]
    );
    assert_eq!(
        serialize_body_document_with_wires(
            &compact.topology,
            &compact.roots.solids,
            &compact.roots.sheets,
            &compact.roots.wires,
            &compact.roots.compounds
        )
        .unwrap(),
        before
    );
    assert!(compact.topology.allocated_slot_count() < source.allocated_slot_count());
    assert!(source.solid(unrelated).is_ok());
}

#[test]
fn empty_selection_is_empty_and_bad_roots_or_budgets_leave_source_unchanged() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 1.0, 2.0, 3.0).unwrap();
    let dead = make_box(&mut source, 4.0, 5.0, 6.0).unwrap();
    source.delete_solid(dead).unwrap();
    let before = serialize_body_document_with_wires(&source, &[solid], &[], &[], &[]).unwrap();
    let slots = source.allocated_slot_count();
    let empty = compact_document_with_limit(&source, CompactionRoots::default(), 0).unwrap();
    assert_eq!(empty.topology.allocated_slot_count(), 0);
    assert!(empty.roots.solids.is_empty());
    let roots = CompactionRoots {
        solids: &[solid],
        ..CompactionRoots::default()
    };
    assert!(compact_document_with_limit(&source, roots, 1).is_err());
    assert!(
        compact_document(
            &source,
            CompactionRoots {
                solids: &[dead],
                ..CompactionRoots::default()
            }
        )
        .is_err()
    );
    let shell = source.solid(solid).unwrap().outer_shell();
    assert!(
        compact_document(
            &source,
            CompactionRoots {
                sheets: &[shell],
                ..CompactionRoots::default()
            }
        )
        .is_err()
    );
    assert_eq!(source.allocated_slot_count(), slots);
    assert_eq!(
        serialize_body_document_with_wires(&source, &[solid], &[], &[], &[]).unwrap(),
        before
    );
}

#[test]
fn imported_nurbs_and_cavity_documents_preserve_all_reachable_data() {
    for fixture in ["shapr3d_hammer_holder.step", "multicavity_bin_box.step"] {
        let mut source = Topology::new();
        let text = std::fs::read_to_string(format!(
            "{}/tests/data/{fixture}",
            env!("CARGO_MANIFEST_DIR")
        ))
        .unwrap();
        let solids = remus_io::step::read_step(&text, &mut source).unwrap();
        let before = serialize_body_document_with_wires(&source, &solids, &[], &[], &[]).unwrap();
        let compact = compact_document(
            &source,
            CompactionRoots {
                solids: &solids,
                ..CompactionRoots::default()
            },
        )
        .unwrap();
        assert_eq!(
            serialize_body_document_with_wires(
                &compact.topology,
                &compact.roots.solids,
                &[],
                &[],
                &[]
            )
            .unwrap(),
            before,
            "{fixture}"
        );
    }
}

#[test]
fn selected_journal_lineage_and_persistent_references_survive() {
    use remus_algo::bop::BooleanOp;
    use remus_topology::journal::EntityKind;
    use remus_topology::naming::{PersistentRef, Resolution, resolve};

    let mut source = Topology::new();
    let outer = make_box(&mut source, 7.0, 3.0, 4.0).unwrap();
    let inner = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
    let result = remus_operations::journal_ops::boolean_journaled(
        &mut source,
        BooleanOp::Fuse,
        outer,
        inner,
    )
    .unwrap();
    let reference = PersistentRef::operation_output(result.op, EntityKind::Face, 0);
    assert!(matches!(
        resolve(&source, &reference),
        Resolution::Bound { .. }
    ));
    let before =
        serialize_body_document_with_wires(&source, &[result.solid], &[], &[], &[]).unwrap();
    let compact = compact_document(
        &source,
        CompactionRoots {
            solids: &[result.solid],
            ..CompactionRoots::default()
        },
    )
    .unwrap();
    assert_eq!(
        serialize_body_document_with_wires(&compact.topology, &compact.roots.solids, &[], &[], &[])
            .unwrap(),
        before
    );
    let Resolution::Bound { entity, .. } = resolve(&compact.topology, &reference) else {
        panic!("selected lineage must resolve in the new document");
    };
    assert!(
        solid_faces(&compact.topology, compact.roots.solids[0])
            .unwrap()
            .iter()
            .any(|face| face.index() == entity.index)
    );
    assert!(matches!(
        resolve(&source, &reference),
        Resolution::Bound { .. }
    ));
}

#[test]
fn invalid_reachable_geometry_refuses_without_changing_source() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let vertex = remus_topology::explorer::solid_vertices(&source, solid).unwrap()[0];
    source
        .vertex_mut(vertex)
        .unwrap()
        .set_point(Point3::new(f64::NAN, 0.0, 0.0));
    let identity = source.cache_identity();
    let slots = source.allocated_slot_count();
    assert!(
        compact_document(
            &source,
            CompactionRoots {
                solids: &[solid],
                ..CompactionRoots::default()
            }
        )
        .is_err()
    );
    assert_eq!(source.cache_identity(), identity);
    assert_eq!(source.allocated_slot_count(), slots);
    assert!(source.vertex(vertex).unwrap().point().x().is_nan());
}

#[test]
fn repeated_references_are_budgeted_before_copying_lists() {
    use remus_math::vec::Vec3;
    use remus_topology::builder::make_circle_edge;
    use remus_topology::wire::{OrientedEdge, Wire};

    let mut source = Topology::new();
    let edge = make_circle_edge(
        &mut source,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        1e-7,
    )
    .unwrap();
    let wire = source.add_wire(Wire::new(vec![OrientedEdge::new(edge, true); 1000], true).unwrap());
    let slots = source.allocated_slot_count();
    let failure = compact_document_with_limit(
        &source,
        CompactionRoots {
            wires: &[wire],
            ..CompactionRoots::default()
        },
        10,
    );
    assert!(matches!(
        failure,
        Err(remus_io::IoError::LimitExceeded {
            resource: "arena wire edge references",
            actual: 1000,
            limit: 10
        })
    ));
    assert_eq!(source.allocated_slot_count(), slots);
    let bytes = remus_io::arena_io::serialize_wire(&source, wire).unwrap();
    let mut destination = Topology::new();
    let sentinel = destination.add_empty_solid();
    let destination_slots = destination.allocated_slot_count();
    let failure = remus_io::arena_io::deserialize_wires_with_limits(
        &bytes,
        &mut destination,
        remus_io::ImportLimits {
            max_model_entities: 10,
            ..remus_io::ImportLimits::default()
        },
    );
    assert!(matches!(
        failure,
        Err(remus_io::IoError::LimitExceeded {
            resource: "arena wire edge references",
            actual: 1000,
            limit: 10
        })
    ));
    assert_eq!(destination.allocated_slot_count(), destination_slots);
    assert!(destination.is_empty_solid(sentinel));

    let solid = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
    let compound = source.add_compound(Compound::new(vec![solid; 1000]));
    let failure = compact_document_with_limit(
        &source,
        CompactionRoots {
            compounds: &[compound],
            ..CompactionRoots::default()
        },
        100,
    );
    assert!(matches!(
        failure,
        Err(remus_io::IoError::LimitExceeded {
            resource: "arena compound solid references",
            actual: 1000,
            limit: 100
        })
    ));
}
