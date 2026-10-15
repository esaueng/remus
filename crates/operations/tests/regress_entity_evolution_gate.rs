//! The entity-evolution boolean routes apply the same result gate as
//! [`boolean`].
//!
//! `boolean_with_entity_evolution` and `boolean_journaled` used to return the
//! raw GFA result unchecked. A pocket cut through the embossed letters of the
//! public hammer-holder fixture makes the GFA build an open shell, which
//! `boolean` refuses; both entity routes handed it back as a solid.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::boolean::{
    BooleanOp, boolean, boolean_with_entity_evolution, boolean_with_evolution,
};
use remus_operations::journal_ops::boolean_journaled_with_operation;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::{SolidId, Topology};

const HAMMER_HOLDER: &str = include_str!("../../io/tests/data/shapr3d_hammer_holder.step");

fn live_counts(t: &Topology) -> [usize; 6] {
    [
        t.num_vertices(),
        t.num_edges(),
        t.num_wires(),
        t.num_faces(),
        t.num_shells(),
        t.num_solids(),
    ]
}

fn solid_snapshot(topo: &Topology, solid: SolidId) -> String {
    let root = topo.solid(solid).unwrap();
    let mut entities = vec![format!("{root:?}")];
    for face in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face).unwrap();
        entities.push(format!("{face:?}"));
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            entities.push(format!("{:?}", topo.wire(wire).unwrap()));
        }
    }
    for edge in solid_edges(topo, solid).unwrap() {
        entities.push(format!("{:?}", topo.edge(edge).unwrap()));
    }
    for vertex in solid_vertices(topo, solid).unwrap() {
        entities.push(format!("{:?}", topo.vertex(vertex).unwrap()));
    }
    entities.join("\n")
}

/// Edges used by exactly one wire occurrence across the solid's faces.
fn free_edge_count(topo: &Topology, solid: SolidId) -> usize {
    let mut uses = std::collections::BTreeMap::<usize, usize>::new();
    for face in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face).unwrap();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wire).unwrap().edges() {
                *uses.entry(oe.edge().index()).or_default() += 1;
            }
        }
    }
    uses.values().filter(|&&n| n == 1).count()
}

/// The hammer holder plus a box pocket of `size` at `origin`, in model units.
fn holder_and_pocket(origin: [f64; 3], size: [f64; 3]) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(HAMMER_HOLDER, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    let pocket = make_box(&mut topo, size[0], size[1], size[2]).unwrap();
    transform_solid(
        &mut topo,
        pocket,
        &Mat4::translation(origin[0], origin[1], origin[2]),
    )
    .unwrap();
    (topo, solids[0], pocket)
}

#[test]
fn entity_evolution_routes_refuse_open_letter_pocket_like_boolean() {
    // The letters stand 0.4 on the x = -12 wall (y 16..21, z 12..47); this
    // pocket ends inside the lettering, sinking 1 below the wall.
    let (original, holder, pocket) = holder_and_pocket([-13.0, 15.0, 20.0], [3.0, 7.0, 10.0]);

    // Precondition: the raw GFA still builds an open shell here. If it stops
    // doing so, this fixture no longer exercises the gate; pick a new one.
    let mut raw = original.clone();
    let (open, _) = remus_algo::gfa::boolean_with_entity_evolution(
        &mut raw,
        remus_algo::bop::BooleanOp::Cut,
        holder,
        pocket,
    )
    .unwrap();
    assert!(free_edge_count(&raw, open) > 0);

    let mut topo = original.clone();
    let error = boolean(&mut topo, BooleanOp::Cut, holder, pocket).unwrap_err();
    assert!(
        matches!(error, OperationsError::ExactOnlyUnattainable),
        "{error:?}"
    );
    let mut topo = original.clone();
    let error = boolean_with_evolution(&mut topo, BooleanOp::Cut, holder, pocket).unwrap_err();
    assert!(
        matches!(error, OperationsError::ExactOnlyUnattainable),
        "{error:?}"
    );

    let operands = [
        solid_snapshot(&original, holder),
        solid_snapshot(&original, pocket),
    ];
    for journaled in [false, true] {
        let mut topo = original.clone();
        let counts = live_counts(&topo);
        let journal = topo.journal().snapshot();
        let error = if journaled {
            boolean_journaled_with_operation(&mut topo, BooleanOp::Cut, holder, pocket).map(|_| ())
        } else {
            boolean_with_entity_evolution(&mut topo, BooleanOp::Cut, holder, pocket).map(|_| ())
        }
        .unwrap_err();
        assert!(
            matches!(error, OperationsError::ExactOnlyUnattainable),
            "journaled={journaled}: {error:?}"
        );
        assert_eq!(live_counts(&topo), counts, "journaled={journaled}");
        assert_eq!(topo.journal().snapshot(), journal, "journaled={journaled}");
        assert_eq!(
            [solid_snapshot(&topo, holder), solid_snapshot(&topo, pocket)],
            operands,
            "journaled={journaled}"
        );
    }
}

#[test]
fn entity_evolution_routes_accept_closed_letter_pocket_like_boolean() {
    // A pocket spanning the whole lettering closes up; the gate must not
    // refuse what `boolean` accepts.
    let (original, holder, pocket) = holder_and_pocket([-13.0, 15.0, 11.0], [3.0, 7.0, 37.0]);

    let mut topo = original.clone();
    let expected = boolean(&mut topo, BooleanOp::Cut, holder, pocket).unwrap();
    let expected_faces = solid_faces(&topo, expected).unwrap().len();

    let mut topo = original.clone();
    let (solid, evolution) =
        boolean_with_entity_evolution(&mut topo, BooleanOp::Cut, holder, pocket).unwrap();
    assert_eq!(free_edge_count(&topo, solid), 0);
    assert_eq!(solid_faces(&topo, solid).unwrap().len(), expected_faces);
    assert_eq!(evolution.faces.len(), expected_faces);

    let mut topo = original;
    let journaled =
        boolean_journaled_with_operation(&mut topo, BooleanOp::Cut, holder, pocket).unwrap();
    assert_eq!(free_edge_count(&topo, journaled.solid), 0);
    assert_eq!(
        solid_faces(&topo, journaled.solid).unwrap().len(),
        expected_faces
    );
}
