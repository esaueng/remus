//! Regression gate for offset face-pair determinism.
//!
//! `offset::inter3d::intersect_faces_3d` iterates `edge_to_face_map` to pair up
//! adjacent offset faces. While that map was a std `HashMap` its order was
//! seed-dependent, which fixed the order of `OffsetData::intersections` and
//! `boundary_edges` differently in every process — and decided which pair the
//! `?` blames when one cannot be intersected. The `approx_census` NURBS-loft row
//! named a different pair on nearly every run (`Id(3)`/`Id(4)`, `Id(5)`/`Id(8)`,
//! `Id(3)`/`Id(7)`, …), which is exactly the noise that stops a census diff from
//! proving a change moved nothing from analytic to fallback.
//!
//! `edge_to_face_map` returns a `BTreeMap` now, so nothing here sorts: this test
//! guards that property from the consumer side, and fails if the map ever goes
//! back to an unordered one.
//!
//! It is deliberately an in-process loop. `RandomState` re-seeds per `HashMap`
//! instance rather than once per process, so rebuilding the solid each iteration
//! is enough to expose seed-dependence without spawning anything: against the
//! unordered map this body reported SEVEN distinct face pairs in 24 iterations
//! of a single process. That is what makes it a CI-enforceable gate, where the
//! `determinism_sweep` example — which covers this same scenario across
//! processes, and more thoroughly — has to be run by hand.
//!
//! The assertion is on the outcome being STABLE, not on which pair it names or
//! on the operation failing at all: teaching the offset engine to intersect
//! NURBS surfaces should turn this into a success, and the message check below
//! is what will notice.

#![allow(clippy::unwrap_used)]

use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::loft::loft_smooth;
use brepkit_operations::offset_v2::offset_solid_v2;
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve};
use brepkit_topology::face::{Face, FaceId, FaceSurface};
use brepkit_topology::vertex::Vertex;
use brepkit_topology::wire::{OrientedEdge, Wire};

/// A closed square profile in the z = `z` plane, as `approx_census` builds it.
fn square_at(topo: &mut Topology, size: f64, z: f64) -> FaceId {
    let hs = size / 2.0;
    let tol = 1e-7;
    let v: Vec<_> = [(-hs, -hs), (hs, -hs), (hs, hs), (-hs, hs)]
        .into_iter()
        .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, z), tol)))
        .collect();
    let e: Vec<_> = (0..4)
        .map(|i| topo.add_edge(Edge::new(v[i], v[(i + 1) % 4], EdgeCurve::Line)))
        .collect();
    let wire = Wire::new(
        (0..4).map(|i| OrientedEdge::new(e[i], true)).collect(),
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

#[test]
fn nurbs_loft_offset_reports_a_stable_face_pair() {
    const ITERATIONS: usize = 24;

    let mut outcomes: Vec<String> = Vec::with_capacity(ITERATIONS);
    for _ in 0..ITERATIONS {
        let mut topo = Topology::new();
        let p0 = square_at(&mut topo, 6.0, 0.0);
        let p1 = square_at(&mut topo, 3.0, 5.0);
        let p2 = square_at(&mut topo, 6.0, 10.0);
        let solid = loft_smooth(&mut topo, &[p0, p1, p2]).unwrap();

        outcomes.push(match offset_solid_v2(&mut topo, solid, 0.5) {
            Ok(_) => "ok".to_string(),
            Err(e) => format!("err: {e}"),
        });
    }

    let first = &outcomes[0];
    let distinct: std::collections::BTreeSet<&str> = outcomes.iter().map(String::as_str).collect();
    assert_eq!(
        distinct.len(),
        1,
        "offset outcome is not reproducible across {ITERATIONS} runs — the \
         face-pair visit order in intersect_faces_3d is seed-dependent again. \
         Saw: {distinct:?}"
    );

    // Pin what the census currently records, so this test also notices if the
    // engine gains NURBS-NURBS intersection (then update the row above).
    assert!(
        first.contains("NURBS surface intersection not yet implemented"),
        "expected the unimplemented-NURBS-intersection failure, got: {first}"
    );
}
