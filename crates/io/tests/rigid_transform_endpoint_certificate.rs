//! Regression: a rigid transform must not strand an edge whose measured
//! endpoint residual is exactly its stamped tolerance.
//!
//! Production case (Tiny-Fox export failure): a STEP import stamps a circle
//! edge's tolerance as the measured endpoint residual
//! (`4.459513151817844e-05`), then a 90°-about-X Move re-rounds the mapped
//! circle frame through `normalize()`. The recomputed residual exceeds the
//! carried tolerance by `8.13e-16` — pure f64 rounding of a mathematically
//! invariant quantity — and the strict arena I/O gate refuses the document
//! on export. The transform layer now carries the certificate forward within
//! a coordinate-scale budget, exactly as it already did for translations.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::copy::copy_and_transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_edges;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// The production Move: 90° about X plus translation.
/// `transformMatrix` in `packages/kernel-adapter/src/exact-math.ts` builds
/// row1=[0,ca,-sa], row2=[0,sa,ca] with sa=sin(90°), ca=cos(90°).
fn production_move() -> Mat4 {
    let rx = 90.0_f64.to_radians();
    let (sa, ca) = (rx.sin(), rx.cos());
    Mat4([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, ca, -sa, 66.5],
        [0.0, sa, ca, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

/// A single-face solid (quad) whose edge 0 is the production circle: the
/// import-time state where the end residual is exactly the stamped
/// tolerance, mirroring the STEP reader's measure-then-stamp contract.
fn quad_solid_with_import_circle() -> (Topology, remus_topology::solid::SolidId) {
    let circle = Circle3D::with_axes(
        Point3::new(-15.0, 34.5, 9.5),
        Vec3::new(0.0, 1.0, -6.123_233_995_736_766e-17),
        3.0,
        Vec3::new(0.0, -6.123_233_995_736_766e-17, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let start = Point3::new(-12.0, 34.5, 9.5);
    let end = Point3::new(-12.940_962_466, 34.499_957_672, 11.681_806_638);
    let t0 = circle.project(start);
    let projected_end = circle.project(end);
    let range1 = t0 + (projected_end - t0).rem_euclid(std::f64::consts::TAU);
    let tolerance = (circle.evaluate(t0) - start)
        .length()
        .max((circle.evaluate(range1) - end).length());

    let mut topo = Topology::new();
    let v_start = topo.add_vertex(Vertex::new(start, 1e-7));
    let v_end = topo.add_vertex(Vertex::new(end, 1e-7));
    // Close the quad with line edges; only edge 0 carries the circle.
    let v2 = topo.add_vertex(Vertex::new(Point3::new(-12.0, 30.0, 9.5), 1e-7));
    let v3 = topo.add_vertex(Vertex::new(Point3::new(-12.0, 30.0, 12.0), 1e-7));
    let mut arc = Edge::with_tolerance(v_start, v_end, EdgeCurve::Circle(circle), Some(tolerance));
    arc.set_trim(Some((t0, range1)));
    let e0 = topo.add_edge(arc);
    let e1 = topo.add_edge(Edge::new(v_end, v2, EdgeCurve::Line));
    let e2 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
    let e3 = topo.add_edge(Edge::new(v3, v_start, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, true),
                OrientedEdge::new(e2, true),
                OrientedEdge::new(e3, true),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(1.0, 0.0, 0.0),
            d: -12.0,
        },
    ));
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    (topo, solid)
}

#[test]
fn rigid_transform_preserves_exact_endpoint_certificate() {
    let (mut topo, solid) = quad_solid_with_import_circle();

    // Import-time state fits its own stamp (the reader measures the stored
    // range, so this holds by construction — the assertion pins the fixture).
    let pre_bytes = serialize_solids(&topo, &[solid]).unwrap();
    deserialize_solids(&pre_bytes, &mut Topology::new()).unwrap();

    // The production rotation re-rounds the circle frame; without certificate
    // carry the recomputed residual exceeds the stamp by 8.13e-16 and the
    // strict gate refuses the document.
    let moved = copy_and_transform_solid(&mut topo, solid, &production_move()).unwrap();
    let moved_edge = solid_edges(&topo, moved).unwrap()[0];
    let edge = topo.edge(moved_edge).unwrap();
    let (a, b) = edge.strict_domain().unwrap();
    let p = topo.vertex(edge.start()).unwrap().point();
    let q = topo.vertex(edge.end()).unwrap().point();
    let residual = (edge.curve().evaluate_with_endpoints(a, p, q) - p)
        .length()
        .max((edge.curve().evaluate_with_endpoints(b, p, q) - q).length());
    let carried = edge.effective_tolerance(
        topo.vertex(edge.start())
            .unwrap()
            .tolerance()
            .max(topo.vertex(edge.end()).unwrap().tolerance()),
    );
    assert!(
        residual <= carried,
        "rigid transform must carry the certificate"
    );

    // And the moved document passes the strict arena gate — the exact failure
    // the production export hit.
    let bytes = serialize_solids(&topo, &[moved]).unwrap();
    let mut restored = Topology::new();
    let solids = deserialize_solids(&bytes, &mut restored).unwrap();
    assert_eq!(solids.len(), 1);
}

#[test]
fn genuinely_invalid_edge_still_fails_after_rigid_transform() {
    let (mut topo, solid) = quad_solid_with_import_circle();
    // Corrupt the edge far beyond any roundoff budget: shift the end vertex
    // 10× the edge tolerance off the carrier.
    let moved = copy_and_transform_solid(&mut topo, solid, &production_move()).unwrap();
    let moved_edge = solid_edges(&topo, moved).unwrap()[0];
    let edge_data = topo.edge(moved_edge).unwrap();
    let (start_id, end_id) = (edge_data.start(), edge_data.end());
    let edge_tol = edge_data.effective_tolerance(
        topo.vertex(start_id)
            .unwrap()
            .tolerance()
            .max(topo.vertex(end_id).unwrap().tolerance()),
    );
    let bad = topo.vertex(end_id).unwrap().point() + Vec3::new(10.0 * edge_tol, 0.0, 0.0);
    topo.vertex_mut(end_id).unwrap().set_point(bad);
    let bytes = serialize_solids(&topo, &[moved]).unwrap();
    let err = deserialize_solids(&bytes, &mut Topology::new()).unwrap_err();
    assert!(
        format!("{err:?}").contains("exceeds effective tolerance"),
        "a real gap must still fail: {err:?}"
    );
}
