//! A vertex merge must not rebuild the edges it touches.
//!
//! Four sites overwrote an existing edge with `Edge::new(start, end, curve)`
//! to change nothing but its endpoints. `Edge::new` resets the explicit trim
//! (RFC 0002, Stage 3) and the edge-specific tolerance, and neither is
//! recoverable from the endpoints — the trim exists precisely so the domain
//! never has to be reconstructed by endpoint projection.
//!
//! The correct pattern already existed three doors away in
//! `heal::fix::split_vertex` and `heal::fix::wireframe`, which use
//! `set_start`/`set_end`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_heal::reshape::ReShape;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};

/// A trim that differs from the `[0, 1]` a `Line` would reconstruct, so a
/// silent reset is visible rather than coincidentally correct.
const TRIM: (f64, f64) = (0.0, 1.0);
const EDGE_TOL: f64 = 3.5e-8;

fn unit_box(topo: &mut Topology) -> (SolidId, Vec<EdgeId>) {
    let solid = remus_operations::primitives::make_box(topo, 1.0, 1.0, 1.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(topo, solid).unwrap();
    for &e in &edges {
        let em = topo.edge_mut(e).unwrap();
        em.set_trim(Some(TRIM));
        em.set_tolerance(Some(EDGE_TOL)).unwrap();
    }
    (solid, edges)
}

/// Split one corner off into a coincident duplicate, so a merge has work to
/// do. Returns (original, duplicate, the edge that was repointed).
fn duplicate_a_corner(topo: &mut Topology, edges: &[EdgeId], offset: f64) -> (VertexId, VertexId) {
    let first = topo.edge(edges[0]).unwrap().start();
    let p = topo.vertex(first).unwrap().point();
    let dup = topo.add_vertex(Vertex::new(Point3::new(p.x() + offset, p.y(), p.z()), 1e-7));
    for &e in edges {
        if topo.edge(e).unwrap().start() == first {
            topo.edge_mut(e).unwrap().set_start(dup);
            break;
        }
    }
    (first, dup)
}

fn assert_all_preserved(topo: &Topology, edges: &[EdgeId], case: &str) {
    for &e in edges {
        let ed = topo.edge(e).unwrap();
        assert_eq!(
            ed.trim(),
            Some(TRIM),
            "{case}: edge {e:?} lost its explicit trim"
        );
        assert_eq!(
            ed.tolerance(),
            Some(EDGE_TOL),
            "{case}: edge {e:?} lost its edge-specific tolerance"
        );
    }
}

/// Assert the two vertices have collapsed onto one another: exactly one of
/// them survives, carrying every use. Which one survives is the merge's
/// choice, not this test's business.
fn assert_collapsed(topo: &Topology, edges: &[EdgeId], a: VertexId, b: VertexId, case: &str) {
    let (ra, rb) = (references(topo, edges, a), references(topo, edges, b));
    assert!(
        ra == 0 || rb == 0,
        "{case}: {a:?} and {b:?} both still referenced ({ra} and {rb}) - nothing merged"
    );
    assert_eq!(
        ra + rb,
        3,
        "{case}: the merged corner should carry all three incident edge uses"
    );
}

fn references(topo: &Topology, edges: &[EdgeId], v: VertexId) -> usize {
    edges
        .iter()
        .filter(|&&e| {
            let ed = topo.edge(e).unwrap();
            ed.start() == v || ed.end() == v
        })
        .count()
}

#[test]
fn reshape_vertex_replacement_preserves_trim() {
    let mut topo = Topology::new();
    let (solid, edges) = unit_box(&mut topo);

    let corner = topo.edge(edges[0]).unwrap().start();
    let p = topo.vertex(corner).unwrap().point();
    let fresh = topo.add_vertex(Vertex::new(p, 1e-7));

    let mut reshape = ReShape::new();
    reshape.replace_vertex(corner, fresh);
    reshape.apply(&mut topo, solid).unwrap();

    // The replacement must actually have happened, or preservation is vacuous.
    assert_eq!(
        references(&topo, &edges, corner),
        0,
        "the replaced vertex should no longer be referenced"
    );
    assert!(
        references(&topo, &edges, fresh) > 0,
        "replacement took effect"
    );

    assert_all_preserved(&topo, &edges, "ReShape::apply");
}

#[test]
fn operations_merge_coincident_vertices_preserves_trim() {
    let mut topo = Topology::new();
    let (solid, edges) = unit_box(&mut topo);
    let (orig, dup) = duplicate_a_corner(&mut topo, &edges, 0.0);

    let merged = remus_operations::heal::merge_coincident_vertices(&mut topo, solid, 1e-6).unwrap();

    assert!(
        merged > 0,
        "the fixture must give the merge something to do"
    );
    assert_collapsed(
        &topo,
        &edges,
        orig,
        dup,
        "operations::heal::merge_coincident_vertices",
    );
    assert_all_preserved(&topo, &edges, "operations::heal::merge_coincident_vertices");
}

#[test]
fn operations_close_wire_gaps_preserves_trim() {
    let mut topo = Topology::new();
    let (solid, edges) = unit_box(&mut topo);
    // A gap well inside the closing tolerance.
    let (orig, near) = duplicate_a_corner(&mut topo, &edges, 1e-9);

    let closed = remus_operations::heal::close_wire_gaps(&mut topo, solid, 1e-6).unwrap();

    assert!(
        closed > 0,
        "the fixture must give gap closing something to do"
    );
    assert_collapsed(
        &topo,
        &edges,
        orig,
        near,
        "operations::heal::close_wire_gaps",
    );
    assert_all_preserved(&topo, &edges, "operations::heal::close_wire_gaps");
}

#[test]
fn heal_fix_shape_merge_preserves_trim() {
    let mut topo = Topology::new();
    let (solid, edges) = unit_box(&mut topo);
    let (orig, dup) = duplicate_a_corner(&mut topo, &edges, 0.0);

    let config = remus_heal::fix::config::FixConfig::default();
    remus_heal::fix::fix_shape(&mut topo, solid, &config).expect("fix_shape on a box");

    assert_collapsed(&topo, &edges, orig, dup, "heal::fix::fix_shape");
    assert_all_preserved(&topo, &edges, "heal::fix::fix_shape");
}
