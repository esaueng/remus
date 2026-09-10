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

#[test]
fn translated_certified_endpoint_survives_strict_arena_transfer() {
    use remus_math::{mat::Mat4, nurbs::curve::NurbsCurve, vec::Point3};
    use remus_topology::edge::EdgeCurve;
    let mut source = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut source, 10.0, 20.0, 30.0).unwrap();
    let edge_id = remus_topology::explorer::solid_edges(&source, solid)
        .unwrap()
        .into_iter()
        .find(|id| {
            source
                .vertex(source.edge(*id).unwrap().start())
                .unwrap()
                .point()
                .x()
                == 0.0
        })
        .unwrap();
    let edge = source.edge(edge_id).unwrap();
    let p = source.vertex(edge.start()).unwrap().point();
    let q = source.vertex(edge.end()).unwrap().point();
    let gap = 0.00004;
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(p.x() + gap, p.y(), p.z()), q],
        vec![1.0, 1.0],
    )
    .unwrap();
    let edge = source.edge_mut(edge_id).unwrap();
    edge.set_curve(EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some((0.0, 1.0)));
    edge.set_tolerance(Some(gap)).unwrap();
    let before = serialize_solids(&source, &[solid]).unwrap();
    deserialize_solids(&before, &mut Topology::new()).unwrap();
    let copied = remus_operations::copy::copy_and_transform_solid(
        &mut source,
        solid,
        &Mat4::translation(4.5, 0.0, 0.0),
    )
    .unwrap();
    let copy_bytes = serialize_solids(&source, &[copied]).unwrap();
    deserialize_solids(&copy_bytes, &mut Topology::new()).unwrap();
    assert_eq!(serialize_solids(&source, &[solid]).unwrap(), before);
    remus_operations::transform::transform_solid(
        &mut source,
        solid,
        &Mat4::translation(4.5, 0.0, 0.0),
    )
    .unwrap();
    let after = serialize_solids(&source, &[solid]).unwrap();
    let mut restored = Topology::new();
    let solids = deserialize_solids(&after, &mut restored).unwrap();
    assert_eq!(serialize_solids(&restored, &solids).unwrap(), after);
    assert!(source.edge(edge_id).unwrap().tolerance().unwrap() - gap < 1e-15);
}
