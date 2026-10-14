//! The exact-only entity-evolution and journaled booleans gate their result.
//!
//! `boolean_with_entity_evolution` and `boolean_journaled` used to hand the
//! raw GFA output back as success with no acceptance gate. A sphere fused with
//! a copy of itself came back as a single-face open shell (16 boundary edges)
//! and was journaled, while `boolean` answers the same pair with a valid solid.
//! Both entry points now refuse a result that fails the exact-result gate and
//! roll back; ordinary exact results are unchanged.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean, boolean_with_entity_evolution};
use remus_operations::classify::{PointClassification, classify_points};
use remus_operations::copy::copy_solid;
use remus_operations::journal_ops::boolean_journaled_with_operation;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::transform::transform_solid;
use remus_operations::validate;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityKind, EntryPayload, RecordedOrigin};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
use remus_topology::solid::SolidId;

type Entry = fn(&mut Topology, BooleanOp, SolidId, SolidId) -> Result<SolidId, OperationsError>;

fn via_evolution(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, OperationsError> {
    boolean_with_entity_evolution(topo, op, a, b).map(|(solid, _)| solid)
}

fn via_journal(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, OperationsError> {
    boolean_journaled_with_operation(topo, op, a, b).map(|r| r.solid)
}

const ENTRIES: [(&str, Entry); 2] = [
    ("boolean_with_entity_evolution", via_evolution),
    ("boolean_journaled", via_journal),
];

fn arena_counts(topo: &Topology) -> [usize; 11] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_compounds(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
        topo.journal().len(),
    ]
}

#[test]
fn identical_sphere_fuse_is_refused_not_returned_open() {
    for (name, entry) in ENTRIES {
        let mut topo = Topology::new();
        let a = make_sphere(&mut topo, 3.0, 16).unwrap();
        let b = copy_solid(&mut topo, a).unwrap();
        let before = arena_counts(&topo);
        match entry(&mut topo, BooleanOp::Fuse, a, b) {
            Ok(solid) => {
                let report = validate::validate_solid(&topo, solid).unwrap();
                assert!(
                    report.is_valid(),
                    "{name}: returned an invalid solid: {:#?}",
                    report.issues
                );
            }
            Err(error) => {
                assert!(
                    matches!(error, OperationsError::ExactOnlyUnattainable),
                    "{name}: unexpected refusal {error}"
                );
                assert_eq!(arena_counts(&topo), before, "{name}: refusal leaked state");
            }
        }
    }
    // Control: the gated boolean answers the same pair with a valid solid.
    let mut topo = Topology::new();
    let a = make_sphere(&mut topo, 3.0, 16).unwrap();
    let b = copy_solid(&mut topo, a).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    assert!(validate::validate_solid(&topo, fused).unwrap().is_valid());
}

#[test]
fn ordinary_exact_booleans_still_pass_the_gate() {
    // (op, expected volume) for a 10³ box against a 10×4×4 bar through it.
    let cases = [
        (BooleanOp::Fuse, 1000.0 + 2.0 * 16.0 * 5.0),
        (BooleanOp::Cut, 1000.0 - 16.0 * 10.0),
        (BooleanOp::Intersect, 16.0 * 10.0),
    ];
    for (name, entry) in ENTRIES {
        for (op, expected) in cases {
            let mut topo = Topology::new();
            let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
            let bar = make_box(&mut topo, 20.0, 4.0, 4.0).unwrap();
            transform_solid(&mut topo, bar, &Mat4::translation(-5.0, 3.0, 3.0)).unwrap();
            let solid = entry(&mut topo, op, blank, bar)
                .unwrap_or_else(|error| panic!("{name} {op:?}: {error}"));
            let report = validate::validate_solid(&topo, solid).unwrap();
            assert!(report.is_valid(), "{name} {op:?}: {:#?}", report.issues);
            let volume = solid_volume(&topo, solid, 1e-4).unwrap();
            assert!(
                (volume - expected).abs() < 1e-6,
                "{name} {op:?}: volume {volume}, expected {expected}"
            );
        }
    }
}

#[test]
fn centred_internal_cavity_cut_passes_the_gate() {
    // A 10³ box minus a centred 2³ box: the result keeps the outer shell and
    // gains a cavity. The outer shell's box centre sits inside the tool, which
    // the tool-interior probe must not read as a retained tool piece.
    let expected = 1000.0 - 8.0;
    for (name, entry) in ENTRIES {
        let mut topo = Topology::new();
        let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let tool = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        transform_solid(&mut topo, tool, &Mat4::translation(4.0, 4.0, 4.0)).unwrap();
        let solid = entry(&mut topo, BooleanOp::Cut, blank, tool)
            .unwrap_or_else(|error| panic!("{name}: {error}"));
        let report = validate::validate_solid(&topo, solid).unwrap();
        assert!(report.is_valid(), "{name}: {:#?}", report.issues);
        assert_eq!(topo.solid(solid).unwrap().inner_shells().len(), 1, "{name}");
        let volume = solid_volume(&topo, solid, 1e-4).unwrap();
        assert!(
            (volume - expected).abs() < 1e-6,
            "{name}: volume {volume}, expected {expected}"
        );
    }
}

fn two_piece_cavity_fixture() -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(20.0, 0.0, 0.0)).unwrap();
    let blank = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let tool = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(4.0, 4.0, 4.0)).unwrap();
    (topo, blank, tool)
}

fn assert_two_piece_cavity(topo: &Topology, solid: SolidId) {
    assert!(validate::validate_solid(topo, solid).unwrap().is_valid());
    assert_eq!(topo.solid(solid).unwrap().inner_shells().len(), 1);
    assert!((solid_volume(topo, solid, 1e-4).unwrap() - 1992.0).abs() < 1e-6);
    let points = [
        Point3::new(5.0, 5.0, 5.0),  // removed cavity
        Point3::new(1.0, 1.0, 1.0),  // first piece's material
        Point3::new(25.0, 5.0, 5.0), // second piece's material
        Point3::new(15.0, 5.0, 5.0), // gap between pieces
    ];
    assert_eq!(
        classify_points(topo, solid, &points, 0.01, 1e-7).unwrap(),
        [
            PointClassification::Outside,
            PointClassification::Inside,
            PointClassification::Inside,
            PointClassification::Outside,
        ]
    );
}

#[test]
fn multi_piece_internal_cavity_cut_preserves_material_and_construction_history() {
    let (mut topo, blank, tool) = two_piece_cavity_fixture();
    let solid = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    assert_two_piece_cavity(&topo, solid);

    let (mut topo, blank, tool) = two_piece_cavity_fixture();
    let (solid, evolution) =
        boolean_with_entity_evolution(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    assert_two_piece_cavity(&topo, solid);
    for (actual, expected) in [
        (
            evolution
                .faces
                .iter()
                .map(|(index, _)| *index)
                .collect::<Vec<_>>(),
            solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .map(remus_topology::arena::Id::index)
                .collect(),
        ),
        (
            evolution
                .edges
                .iter()
                .map(|(index, _)| *index)
                .collect::<Vec<_>>(),
            solid_edges(&topo, solid)
                .unwrap()
                .into_iter()
                .map(remus_topology::arena::Id::index)
                .collect(),
        ),
        (
            evolution
                .vertices
                .iter()
                .map(|(index, _)| *index)
                .collect::<Vec<_>>(),
            solid_vertices(&topo, solid)
                .unwrap()
                .into_iter()
                .map(remus_topology::arena::Id::index)
                .collect(),
        ),
    ] {
        let unique = |indices: Vec<usize>| {
            indices
                .into_iter()
                .collect::<std::collections::BTreeSet<_>>()
        };
        assert_eq!(
            unique(actual),
            unique(expected),
            "construction claims must cover cavity entities too"
        );
    }

    let (mut topo, blank, tool) = two_piece_cavity_fixture();
    let result = boolean_journaled_with_operation(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    assert_two_piece_cavity(&topo, result.solid);
    let entry = topo.journal().entries().last().unwrap();
    assert!(matches!(
        entry.payload(),
        EntryPayload::Evolution {
            origin: RecordedOrigin::Construction,
            ..
        }
    ));
    for (kind, count) in [
        (
            EntityKind::Face,
            solid_faces(&topo, result.solid).unwrap().len(),
        ),
        (
            EntityKind::Edge,
            solid_edges(&topo, result.solid).unwrap().len(),
        ),
        (
            EntityKind::Vertex,
            solid_vertices(&topo, result.solid).unwrap().len(),
        ),
    ] {
        for index in 0..count {
            assert!(
                matches!(
                    resolve(
                        &topo,
                        &PersistentRef::operation_output(result.op, kind, index)
                    ),
                    Resolution::Bound {
                        provenance: Provenance::Construction,
                        ..
                    }
                ),
                "{kind:?} output {index} must remain construction-derived"
            );
        }
    }
}

#[test]
fn exact_multi_piece_intersect_and_severing_cut_remain_available() {
    for (name, entry) in ENTRIES {
        let (mut topo, blank, _) = two_piece_cavity_fixture();
        let clip = make_box(&mut topo, 20.0, 4.0, 4.0).unwrap();
        transform_solid(&mut topo, clip, &Mat4::translation(5.0, 3.0, 3.0)).unwrap();
        let common = entry(&mut topo, BooleanOp::Intersect, blank, clip)
            .unwrap_or_else(|error| panic!("{name} multi-piece intersect: {error}"));
        assert!(validate::validate_solid(&topo, common).unwrap().is_valid());
        assert!((solid_volume(&topo, common, 1e-4).unwrap() - 160.0).abs() < 1e-6);

        // Containing both pieces is exact too. The intersection audit's
        // A-(A-B) identity must treat the canonical empty Cut as volume zero.
        let enclosing = make_box(&mut topo, 40.0, 20.0, 20.0).unwrap();
        transform_solid(&mut topo, enclosing, &Mat4::translation(-5.0, -5.0, -5.0)).unwrap();
        let common = entry(&mut topo, BooleanOp::Intersect, blank, enclosing)
            .unwrap_or_else(|error| panic!("{name} contained multi-piece intersect: {error}"));
        assert!(validate::validate_solid(&topo, common).unwrap().is_valid());
        assert!((solid_volume(&topo, common, 1e-4).unwrap() - 2000.0).abs() < 1e-6);

        let mut topo = Topology::new();
        let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let tool = make_box(&mut topo, 2.0, 20.0, 20.0).unwrap();
        transform_solid(&mut topo, tool, &Mat4::translation(4.0, -5.0, -5.0)).unwrap();
        let cut = entry(&mut topo, BooleanOp::Cut, blank, tool)
            .unwrap_or_else(|error| panic!("{name} severing cut: {error}"));
        assert!(validate::validate_solid(&topo, cut).unwrap().is_valid());
        assert!((solid_volume(&topo, cut, 1e-4).unwrap() - 800.0).abs() < 1e-6);
    }
}

#[test]
fn gate_refusal_restores_existing_journal_and_pending_mutation_gap() {
    let (mut topo, blank, tool) = two_piece_cavity_fixture();
    boolean_journaled_with_operation(&mut topo, BooleanOp::Fuse, blank, tool).unwrap();
    let a = make_sphere(&mut topo, 3.0, 16).unwrap();
    let b = copy_solid(&mut topo, a).unwrap();
    // Arena documents include journal ID high-water marks, which legitimately
    // advance after a refusal. Compare exact geometry separately from the
    // full published-history checks below.
    let geometry_document = |topo: &Topology| {
        let bytes = remus_io::arena_io::serialize_solids(topo, &[blank, tool, a, b]).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document.as_object_mut().unwrap().remove("journal");
        document
    };
    let geometry = geometry_document(&topo);
    let before = arena_counts(&topo);
    let history = topo.journal().snapshot();
    let slots = topo.allocated_slot_count();
    let error = boolean_journaled_with_operation(&mut topo, BooleanOp::Fuse, a, b).unwrap_err();
    assert!(matches!(error, OperationsError::ExactOnlyUnattainable));
    assert_eq!(arena_counts(&topo), before);
    assert_eq!(geometry_document(&topo), geometry);
    let after = topo.journal().snapshot();
    assert_eq!(after.entries, history.entries);
    assert_eq!(after.index, history.index);
    assert_eq!(after.next_ordinal, history.next_ordinal);
    assert!(after.next_op >= history.next_op);
    assert!(
        topo.allocated_slot_count() > slots,
        "exercise a gate refusal after GFA export"
    );
    // The gap preceding the refused operation must still be detected next time.
    boolean_journaled_with_operation(&mut topo, BooleanOp::Fuse, blank, tool).unwrap();
    let entries = topo.journal().entries();
    assert_eq!(entries.len(), history.entries.len() + 2);
    assert_eq!(
        entries[entries.len() - 2].kind(),
        remus_topology::journal::UNJOURNALED_MUTATIONS
    );
}
