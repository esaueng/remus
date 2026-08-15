//! Explicit edge trims: persistence round trip (RFC 0002, Stage 3).
//!
//! The GFA pave filler and builder record exact sub-span trims on split
//! edges inside the boolean's working store (unit-tested at their creation
//! sites); the result-assembly rebuild paths and the op-level analytic fast
//! paths do not yet carry them into result topologies — that migration is
//! queued in RFC 0002. What must already hold everywhere: a stored trim
//! survives copy and the arena format, and the domain accessor prefers it.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use brepkit_operations::copy::copy_solid;
use brepkit_operations::primitives::make_cylinder;
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeCurve;

/// A cylinder whose rim circle edge gets an explicit (partial) trim
/// stamped on it, standing in for a boolean split arc.
fn cylinder_with_trimmed_rim() -> (Topology, brepkit_topology::SolidId) {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 3.0, 4.0).unwrap();
    let rim = topo
        .edges()
        .iter()
        .find(|(_, e)| matches!(e.curve(), EdgeCurve::Circle(_)))
        .map(|(id, _)| id)
        .unwrap();
    // Not geometrically meaningful for the closed rim; the point is purely
    // that the stored interval survives every copy path bit-for-bit.
    let mut edge = topo.edge(rim).unwrap().clone();
    edge.set_trim(Some((0.5, 2.5)));
    *topo.edge_mut(rim).unwrap() = edge;
    (topo, solid)
}

fn trimmed_edges(topo: &Topology) -> usize {
    topo.edges()
        .iter()
        .filter(|(_, e)| e.trim().is_some())
        .count()
}

#[test]
fn trims_survive_solid_copy() {
    let (mut topo, solid) = cylinder_with_trimmed_rim();
    assert_eq!(trimmed_edges(&topo), 1);
    let copied = copy_solid(&mut topo, solid).unwrap();
    assert_ne!(copied, solid);
    assert_eq!(
        trimmed_edges(&topo),
        2,
        "the copy must carry the stored trim"
    );
}

#[test]
fn trims_survive_arena_round_trip() {
    let (topo, solid) = cylinder_with_trimmed_rim();
    let bytes = brepkit_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let mut restored = Topology::new();
    let _ = brepkit_io::arena_io::deserialize_solid(&bytes, &mut restored).unwrap();
    let trims: Vec<_> = restored
        .edges()
        .iter()
        .filter_map(|(_, e)| e.trim())
        .collect();
    assert_eq!(trims, vec![(0.5, 2.5)]);
}
