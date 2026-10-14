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
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean, boolean_with_entity_evolution};
use remus_operations::copy::copy_solid;
use remus_operations::journal_ops::boolean_journaled_with_operation;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::transform::transform_solid;
use remus_operations::validate;
use remus_topology::Topology;
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

fn arena_counts(topo: &Topology) -> (usize, usize, usize, usize, usize) {
    (
        topo.num_faces(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
        topo.journal().len(),
    )
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
