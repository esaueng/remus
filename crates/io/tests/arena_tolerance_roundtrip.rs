//! CHARACTERIZATION (RFC 0004 Stage 1): round-trip byte stability for
//! tolerance-bearing legacy documents. Both tolerance fields are additive
//! arena fields (`SerVertex.tolerance` required, `SerEdge.tolerance`
//! optional) — this stage adds no format change, and serialization of
//! tolerance-bearing documents must stay byte-identical across a
//! serialize -> deserialize -> serialize cycle, with values restored
//! bit-for-bit.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_topology::Topology;

#[test]
fn tolerance_bearing_document_round_trips_byte_identically() {
    let mut source = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut source, 10.0, 20.0, 30.0).unwrap();

    // Stamp tolerance-bearing state on the model: one raised vertex ball
    // and two declared edge tolerances (one the sub-floor sewing-style
    // value, one a plain raise).
    let vertices = remus_topology::explorer::solid_vertices(&source, solid).unwrap();
    let edges = remus_topology::explorer::solid_edges(&source, solid).unwrap();
    source
        .vertex_mut(vertices[0])
        .unwrap()
        .set_tolerance(2.5e-5)
        .unwrap();
    source
        .edge_mut(edges[0])
        .unwrap()
        .set_tolerance(Some(3.5e-8))
        .unwrap();
    source
        .edge_mut(edges[1])
        .unwrap()
        .set_tolerance(Some(5.0e-5))
        .unwrap();

    let bytes = serialize_solids(&source, &[solid]).unwrap();

    let mut destination = Topology::new();
    let restored = deserialize_solids(&bytes, &mut destination).unwrap();
    assert_eq!(restored.len(), 1);

    // Every tolerance value survives bit-for-bit (same multiset; document
    // order is dense-local, not source arena order).
    let mut vertex_balls: Vec<u64> = source
        .vertices()
        .iter()
        .map(|(_id, v)| v.tolerance().to_bits())
        .collect();
    vertex_balls.sort_unstable();
    let mut restored_balls: Vec<u64> = destination
        .vertices()
        .iter()
        .map(|(_id, v)| v.tolerance().to_bits())
        .collect();
    restored_balls.sort_unstable();
    assert_eq!(vertex_balls, restored_balls);

    let mut edge_tols: Vec<Option<u64>> = source
        .edges()
        .iter()
        .map(|(_id, e)| e.tolerance().map(f64::to_bits))
        .collect();
    edge_tols.sort_unstable();
    let mut restored_edge_tols: Vec<Option<u64>> = destination
        .edges()
        .iter()
        .map(|(_id, e)| e.tolerance().map(f64::to_bits))
        .collect();
    restored_edge_tols.sort_unstable();
    assert_eq!(edge_tols, restored_edge_tols);

    // Re-serializing the restored model reproduces the exact document.
    let round_tripped = serialize_solids(&destination, &[restored[0]]).unwrap();
    assert_eq!(
        std::str::from_utf8(&bytes).unwrap(),
        std::str::from_utf8(&round_tripped).unwrap(),
        "legacy documents are byte-stable across arena round trips"
    );
}
