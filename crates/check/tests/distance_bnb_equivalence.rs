//! PERF-Q04 Milestone 4: branch-and-bound verified independently of itself.
//!
//! Two independent rungs:
//!
//! 1. **Exhaustive equivalence.** [`point_to_solid_exhaustive`] uses the
//!    same narrow phase ([`point_to_face`]) in the same deterministic order
//!    but never prunes. Every query below asserts the accelerated path
//!    agrees with it on distance — and on the closest point whenever the
//!    minimum is unique. This rung proves *safe pruning*; it says nothing
//!    about global NURBS optimality.
//! 2. **Closed-form / boundary / independently-constructed validation.**
//!    Representative answers are checked against analytic values (box,
//!    sphere), boundary minima (edge, vertex), and hand-computed closest
//!    points — oracles that do not share code with either traversal.
//!
//! Explicitly out of scope (documented, not fixed): the narrow phase treats
//! hole interiors as material (outer-wire polygon only) and walks curved
//! wires as vertex chords. Both behaviors are identical in the two modes,
//! so they cannot be introduced by branch-and-bound; the hole test below
//! pins the current behavior with the true rim answer beside it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// Bitwise float equality is intentional below: both traversal modes share the
// narrow phase and candidate order, so agreement must be exact, not approximate.
#![allow(clippy::float_cmp)]

use remus_math::curves::Circle3D;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use remus_check::distance::{point_to_solid, point_to_solid_exhaustive, point_to_solid_with_stats};

const TOL: f64 = 1e-7;

/// Build a closed box solid spanning `min..=max` (faces not shared; distance
/// queries do not need manifold edge sharing).
fn make_box(topo: &mut Topology, min: Point3, max: Point3) -> SolidId {
    let v = |topo: &mut Topology, x: f64, y: f64, z: f64| {
        topo.add_vertex(Vertex::new(Point3::new(x, y, z), TOL))
    };
    let v000 = v(topo, min.x(), min.y(), min.z());
    let v100 = v(topo, max.x(), min.y(), min.z());
    let v110 = v(topo, max.x(), max.y(), min.z());
    let v010 = v(topo, min.x(), max.y(), min.z());
    let v001 = v(topo, min.x(), min.y(), max.z());
    let v101 = v(topo, max.x(), min.y(), max.z());
    let v111 = v(topo, max.x(), max.y(), max.z());
    let v011 = v(topo, min.x(), max.y(), max.z());

    let e = |topo: &mut Topology, a: VertexId, b: VertexId| {
        topo.add_edge(Edge::new(a, b, EdgeCurve::Line))
    };
    let e0 = e(topo, v000, v100);
    let e1 = e(topo, v100, v110);
    let e2 = e(topo, v110, v010);
    let e3 = e(topo, v010, v000);
    let e4 = e(topo, v001, v101);
    let e5 = e(topo, v101, v111);
    let e6 = e(topo, v111, v011);
    let e7 = e(topo, v011, v001);
    let e8 = e(topo, v000, v001);
    let e9 = e(topo, v100, v101);
    let e10 = e(topo, v110, v111);
    let e11 = e(topo, v010, v011);

    let faces = vec![
        quad_face(
            topo,
            [(e0, true), (e1, true), (e2, true), (e3, true)],
            Vec3::new(0.0, 0.0, -1.0),
            -min.z(),
        ),
        quad_face(
            topo,
            [(e4, true), (e5, true), (e6, true), (e7, true)],
            Vec3::new(0.0, 0.0, 1.0),
            max.z(),
        ),
        quad_face(
            topo,
            [(e0, true), (e9, true), (e4, false), (e8, false)],
            Vec3::new(0.0, -1.0, 0.0),
            -min.y(),
        ),
        quad_face(
            topo,
            [(e2, true), (e11, true), (e6, false), (e10, false)],
            Vec3::new(0.0, 1.0, 0.0),
            max.y(),
        ),
        quad_face(
            topo,
            [(e3, true), (e8, true), (e7, false), (e11, false)],
            Vec3::new(-1.0, 0.0, 0.0),
            -min.x(),
        ),
        quad_face(
            topo,
            [(e1, true), (e10, true), (e5, false), (e9, false)],
            Vec3::new(1.0, 0.0, 0.0),
            max.x(),
        ),
    ];
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// One quad face from four oriented edges.
fn quad_face(
    topo: &mut Topology,
    edges: [(remus_topology::edge::EdgeId, bool); 4],
    normal: Vec3,
    d: f64,
) -> FaceId {
    let wire = Wire::new(
        edges
            .iter()
            .map(|&(id, fwd)| OrientedEdge::new(id, fwd))
            .collect(),
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
}

/// Assert both modes agree exactly, and return the accelerated result.
fn assert_modes_agree(
    topo: &Topology,
    query: Point3,
    solid: SolidId,
) -> remus_check::distance::DistanceResult {
    let (fast, fast_stats) = point_to_solid_with_stats(topo, query, solid).unwrap();
    let (slow, slow_stats) = point_to_solid_exhaustive(topo, query, solid).unwrap();
    assert_eq!(
        fast.distance, slow.distance,
        "branch-and-bound must match exhaustive distance for {query:?}"
    );
    assert_eq!(
        fast.point_b, slow.point_b,
        "unique minima must have identical closest points for {query:?}"
    );
    // Stats self-consistency.
    assert_eq!(
        fast_stats.faces_total,
        fast_stats.faces_prunable + fast_stats.faces_mandatory
    );
    assert_eq!(slow_stats.faces_skipped_by_bound, 0);
    assert_eq!(slow_stats.faces_evaluated, slow_stats.faces_total);
    assert_eq!(
        fast_stats.faces_evaluated + fast_stats.faces_skipped_by_bound,
        fast_stats.faces_total,
        "evaluated + skipped must account for every face"
    );
    fast
}

#[test]
fn box_closed_forms_and_edge_vertex_minima() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    // Above the top face: interior minimum.
    let r = assert_modes_agree(&topo, Point3::new(0.5, 0.5, 3.0), solid);
    assert!((r.distance - 2.0).abs() < 1e-9);
    assert!((r.point_b.z() - 1.0).abs() < 1e-9);
    // Near an edge (x = 1 wall edge at y = 0.5): minimum on the edge.
    let r = assert_modes_agree(&topo, Point3::new(3.0, 0.5, 0.5), solid);
    assert!((r.distance - 2.0).abs() < 1e-9);
    assert!((r.point_b.x() - 1.0).abs() < 1e-9);
    // Near a vertex (1, 1, 1): minimum exactly the vertex (closed form √3).
    let r = assert_modes_agree(&topo, Point3::new(2.0, 2.0, 2.0), solid);
    assert!(
        (r.distance - 3.0f64.sqrt()).abs() < 1e-9,
        "got {}",
        r.distance
    );
    assert!((r.point_b.x() - 1.0).abs() < 1e-9);
    assert!((r.point_b.y() - 1.0).abs() < 1e-9);
    assert!((r.point_b.z() - 1.0).abs() < 1e-9);
    // Inside: distance to the nearest wall.
    let r = assert_modes_agree(&topo, Point3::new(0.5, 0.5, 0.5), solid);
    assert!((r.distance - 0.5).abs() < 1e-9);
}

#[test]
fn cavity_inner_shell_is_traversed() {
    let mut topo = Topology::new();
    let outer = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 10.0),
    );
    let inner = make_box(
        &mut topo,
        Point3::new(3.0, 3.0, 3.0),
        Point3::new(7.0, 7.0, 7.0),
    );
    let outer_shell = topo.solid(outer).unwrap().outer_shell();
    let inner_shell = topo.solid(inner).unwrap().outer_shell();
    let hollow = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));

    // Cavity centre: the nearest boundary is the cavity wall at 2.0, not the
    // outer wall at 5.0. An outer-shell-only walk answers 5.0.
    let r = assert_modes_agree(&topo, Point3::new(5.0, 5.0, 5.0), hollow);
    assert!((r.distance - 2.0).abs() < 1e-6, "got {}", r.distance);
    // Outside, far away: the outer wall wins in both modes.
    let r = assert_modes_agree(&topo, Point3::new(5.0, 5.0, 20.0), hollow);
    assert!((r.distance - 10.0).abs() < 1e-9, "got {}", r.distance);
}

#[test]
fn hole_interior_documents_narrow_phase_boundary() {
    // A single holed plate (open solid): outer 4x4, square hole 1x1..2x2.
    let mut topo = Topology::new();
    let v = |topo: &mut Topology, x: f64, y: f64| {
        topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), TOL))
    };
    let (o0, o1, o2, o3) = (
        v(&mut topo, 0.0, 0.0),
        v(&mut topo, 4.0, 0.0),
        v(&mut topo, 4.0, 4.0),
        v(&mut topo, 0.0, 4.0),
    );
    let (h0, h1, h2, h3) = (
        v(&mut topo, 1.0, 1.0),
        v(&mut topo, 2.0, 1.0),
        v(&mut topo, 2.0, 2.0),
        v(&mut topo, 1.0, 2.0),
    );
    let line = |a: VertexId, b: VertexId| Edge::new(a, b, EdgeCurve::Line);
    let o0e = topo.add_edge(line(o0, o1));
    let o1e = topo.add_edge(line(o1, o2));
    let o2e = topo.add_edge(line(o2, o3));
    let o3e = topo.add_edge(line(o3, o0));
    let h0e = topo.add_edge(line(h0, h1));
    let h1e = topo.add_edge(line(h1, h2));
    let h2e = topo.add_edge(line(h2, h3));
    let h3e = topo.add_edge(line(h3, h0));
    let outer = Wire::new(
        [o0e, o1e, o2e, o3e]
            .iter()
            .map(|&id| OrientedEdge::new(id, true))
            .collect(),
        true,
    )
    .unwrap();
    let hole = Wire::new(
        [h0e, h1e, h2e, h3e]
            .iter()
            .map(|&id| OrientedEdge::new(id, true))
            .collect(),
        true,
    )
    .unwrap();
    let outer_id = topo.add_wire(outer);
    let hole_id = topo.add_wire(hole);
    let fid = topo.add_face(Face::new(
        outer_id,
        vec![hole_id],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let shell = topo.add_shell(Shell::new(vec![fid]).unwrap());
    let plate = topo.add_solid(Solid::new(shell, vec![]));

    // Above the hole centre: the independently constructed truth is the hole
    // rim at √(0.5² + 5²) ≈ 5.0249, but the narrow phase tests only the
    // outer polygon, so both modes answer the plate projection at 5.0.
    // This pins the current narrow-phase behavior; fixing hole handling is
    // a separate change, and branch-and-bound must not paper over it.
    let query = Point3::new(1.5, 1.5, 5.0);
    let (fast, _) = point_to_solid_with_stats(&topo, query, plate).unwrap();
    let (slow, _) = point_to_solid_exhaustive(&topo, query, plate).unwrap();
    assert_eq!(fast.distance, slow.distance);
    assert!((fast.distance - 5.0).abs() < 1e-9, "got {}", fast.distance);
    let rim_truth = (0.5f64.mul_add(0.5, 25.0)).sqrt();
    assert!(
        (fast.distance - rim_truth).abs() > 1e-6,
        "this test documents hole-as-material behavior; update it if hole handling changes"
    );
    // Beside the hole, over material: both modes agree with the projection.
    let r = assert_modes_agree(&topo, Point3::new(3.0, 3.0, 5.0), plate);
    assert!((r.distance - 5.0).abs() < 1e-9);
}

/// Two hemispherical faces sharing one closed rim: sphere closed forms.
#[test]
fn sphere_hemispheres_closed_forms() {
    use remus_math::surfaces::SphericalSurface;
    let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
    let rim = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();

    let mut topo = Topology::new();
    let seam = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), TOL));
    let t0 = rim.project(Point3::new(1.0, 0.0, 0.0));
    let mut rim_edge = Edge::new(seam, seam, EdgeCurve::Circle(rim));
    rim_edge.set_trim(Some((t0, t0 + std::f64::consts::TAU)));
    let rim_id = topo.add_edge(rim_edge);
    let mut faces: Vec<FaceId> = Vec::new();
    for _ in 0..2 {
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim_id, true)], true).unwrap());
        faces.push(topo.add_face(Face::new(wire, vec![], FaceSurface::Sphere(sphere.clone()))));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let ball = topo.add_solid(Solid::new(shell, vec![]));

    // North pole query: closest is the pole, distance 4.
    let r = assert_modes_agree(&topo, Point3::new(0.0, 0.0, 5.0), ball);
    assert!((r.distance - 4.0).abs() < 1e-6, "got {}", r.distance);
    // Equatorial query: distance 4.
    let r = assert_modes_agree(&topo, Point3::new(5.0, 0.0, 0.0), ball);
    assert!((r.distance - 4.0).abs() < 1e-6, "got {}", r.distance);
    // Centre: every surface point is distance 1.
    let r = assert_modes_agree(&topo, Point3::new(0.0, 0.0, 0.0), ball);
    assert!((r.distance - 1.0).abs() < 1e-6, "got {}", r.distance);
}

/// Flat NURBS patch: Newton is exact on a bilinear patch, so the closed
/// form validates end to end.
#[test]
fn flat_nurbs_patch_closed_form() {
    let surface = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap();
    let mut topo = Topology::new();
    let v = |topo: &mut Topology, x: f64, y: f64| {
        topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), TOL))
    };
    let (v0, v1, v2, v3) = (
        v(&mut topo, 0.0, 0.0),
        v(&mut topo, 1.0, 0.0),
        v(&mut topo, 1.0, 1.0),
        v(&mut topo, 0.0, 1.0),
    );
    let line = |a: VertexId, b: VertexId| Edge::new(a, b, EdgeCurve::Line);
    let f0 = topo.add_edge(line(v0, v1));
    let f1 = topo.add_edge(line(v1, v2));
    let f2 = topo.add_edge(line(v2, v3));
    let f3 = topo.add_edge(line(v3, v0));
    let wire = topo.add_wire(
        Wire::new(
            [f0, f1, f2, f3]
                .iter()
                .map(|&id| OrientedEdge::new(id, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    let fid = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface)));
    let shell = topo.add_shell(Shell::new(vec![fid]).unwrap());
    let patch = topo.add_solid(Solid::new(shell, vec![]));

    let r = assert_modes_agree(&topo, Point3::new(0.5, 0.5, 2.0), patch);
    assert!((r.distance - 2.0).abs() < 1e-6, "got {}", r.distance);
}

/// A wavy NURBS patch: both modes share the local Newton narrow phase, so
/// they must agree exactly — while the test documents that neither mode
/// certifies the global minimum (safe pruning is not global optimization).
#[test]
fn wavy_nurbs_locality_is_shared_not_cured() {
    let mut topo = Topology::new();
    let (patch, surface) = wavy_patch(&mut topo, 0.08, 2.0, 0.0);

    for query in [
        Point3::new(0.5, 0.5, 2.0),
        Point3::new(0.2, 0.8, -1.0),
        Point3::new(1.5, 0.5, 0.3),
    ] {
        let (fast, fast_stats) = point_to_solid_with_stats(&topo, query, patch).unwrap();
        let (slow, _) = point_to_solid_exhaustive(&topo, query, patch).unwrap();
        // Non-vacuous: the narrow phase must actually succeed here (no
        // shared-failure trivial agreement).
        assert!(
            fast_stats.narrow_phase_failures == 0 && fast.distance.is_finite(),
            "query {query:?} must converge for this test to mean anything"
        );
        // Bitwise agreement: same narrow phase, same order, same winner.
        assert_eq!(fast.distance, slow.distance, "query {query:?}");
        assert_eq!(fast.point_b, slow.point_b, "query {query:?}");
        // Sanity against a dense-sample upper bound on the true minimum:
        // neither mode may report *less* than the sampled minimum minus a
        // loose Newton tolerance (they can report more — a local well).
        let mut sample_min = f64::INFINITY;
        for i in 0..=20 {
            for j in 0..=20 {
                let p = surface.evaluate(f64::from(i) / 20.0, f64::from(j) / 20.0);
                sample_min = sample_min.min((p - query).length());
            }
        }
        assert!(
            fast.distance >= sample_min - 0.05,
            "query {query:?}: {} vs sampled {sample_min}",
            fast.distance
        );
    }
}

/// Build a single-face wavy NURBS solid: 4x4 net of amplitude `amp` and
/// frequency `freq`, translated by `ox`. Returns the solid and the surface
/// (for independent dense-sample oracles).
fn wavy_patch(topo: &mut Topology, amp: f64, freq: f64, ox: f64) -> (SolidId, NurbsSurface) {
    let mut cps = Vec::new();
    let mut ws = Vec::new();
    for i in 0..4 {
        let mut row = Vec::new();
        let mut wrow = Vec::new();
        for j in 0..4 {
            let x = ox + f64::from(i) / 3.0;
            let y = f64::from(j) / 3.0;
            let z = amp * (freq * x).sin() * (freq * y).cos();
            row.push(Point3::new(x, y, z));
            wrow.push(1.0);
        }
        cps.push(row);
        ws.push(wrow);
    }
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let surface = NurbsSurface::new(3, 3, knots.clone(), knots, cps, ws).unwrap();
    let v = |topo: &mut Topology, x: f64, y: f64, z: f64| {
        topo.add_vertex(Vertex::new(Point3::new(x, y, z), TOL))
    };
    let corners = [
        v(topo, ox, 0.0, surface.evaluate(0.0, 0.0).z()),
        v(topo, ox + 1.0, 0.0, surface.evaluate(1.0, 0.0).z()),
        v(topo, ox + 1.0, 1.0, surface.evaluate(1.0, 1.0).z()),
        v(topo, ox, 1.0, surface.evaluate(0.0, 1.0).z()),
    ];
    // Boundary curves as fitted NURBS through sampled surface edges so the
    // wires track the wavy boundary (with recorded trims for authority).
    let edge_curve = |u0: f64, v0: f64, u1: f64, v1: f64| {
        let pts: Vec<Point3> = (0..=8)
            .map(|k| {
                let a = f64::from(k) / 8.0;
                surface.evaluate(u0 + (u1 - u0) * a, v0 + (v1 - v0) * a)
            })
            .collect();
        remus_math::nurbs::fitting::interpolate(&pts, 3).unwrap()
    };
    let curves = [
        edge_curve(0.0, 0.0, 1.0, 0.0),
        edge_curve(1.0, 0.0, 1.0, 1.0),
        edge_curve(1.0, 1.0, 0.0, 1.0),
        edge_curve(0.0, 1.0, 0.0, 0.0),
    ];
    let mut edge_ids = Vec::new();
    for (k, curve) in curves.into_iter().enumerate() {
        let (d0, d1) = curve.domain();
        let mut edge = Edge::new(
            corners[k],
            corners[(k + 1) % 4],
            EdgeCurve::NurbsCurve(curve),
        );
        edge.set_trim(Some((d0, d1)));
        edge_ids.push(topo.add_edge(edge));
    }
    let wire = topo.add_wire(
        Wire::new(
            edge_ids
                .iter()
                .map(|&id| OrientedEdge::new(id, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    let fid = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())));
    let shell = topo.add_shell(Shell::new(vec![fid]).unwrap());
    (topo.add_solid(Solid::new(shell, vec![])), surface)
}

/// Newton divergence is shared, not hidden: on a steep wave with a query
/// the local projection cannot handle, the narrow phase fails and both
/// modes report the failure identically (counted, infinite distance) rather
/// than branch-and-bound silently dropping the face.
#[test]
fn newton_divergence_is_shared_not_hidden() {
    let mut topo = Topology::new();
    // Steep wave (verified to defeat the Newton projection from above).
    let (patch, _) = wavy_patch(&mut topo, 0.25, 3.0, 0.0);
    let query = Point3::new(0.5, 0.5, 2.0);
    let (fast, fast_stats) = point_to_solid_with_stats(&topo, query, patch).unwrap();
    let (slow, slow_stats) = point_to_solid_exhaustive(&topo, query, patch).unwrap();
    assert_eq!(fast_stats.narrow_phase_failures, 1);
    assert_eq!(slow_stats.narrow_phase_failures, 1);
    assert!(fast.distance.is_infinite() && slow.distance.is_infinite());
    assert_eq!(fast.distance, slow.distance);
    assert_eq!(fast.point_b, slow.point_b);
}

/// Six-way tie at a box centre: both modes pick the same winner, and
/// repeated runs are deterministic.
#[test]
fn centre_tie_is_deterministic() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let query = Point3::new(0.5, 0.5, 0.5);
    let first = point_to_solid(&topo, query, solid).unwrap();
    assert!((first.distance - 0.5).abs() < 1e-9);
    for _ in 0..5 {
        let again = point_to_solid(&topo, query, solid).unwrap();
        assert_eq!(first.distance, again.distance);
        assert_eq!(first.point_b, again.point_b);
    }
    let (slow, _) = point_to_solid_exhaustive(&topo, query, solid).unwrap();
    assert_eq!(first.distance, slow.distance);
    assert_eq!(first.point_b, slow.point_b);
}

/// Multi-face sparse model: pruning must actually skip faces while matching
/// exhaustive exactly.
#[test]
fn sparse_row_prunes_but_matches() {
    let mut topo = Topology::new();
    let mut faces: Vec<FaceId> = Vec::new();
    for k in 0..20 {
        let solid = make_box(
            &mut topo,
            Point3::new(k as f64 * 3.0, 0.0, 0.0),
            Point3::new(k as f64 * 3.0 + 1.0, 1.0, 1.0),
        );
        let shell = topo.solid(solid).unwrap().outer_shell();
        faces.extend(topo.shell(shell).unwrap().faces().iter().copied());
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let row = topo.add_solid(Solid::new(shell, vec![]));

    let query = Point3::new(0.5, 0.5, 3.0);
    let (fast, stats) = point_to_solid_with_stats(&topo, query, row).unwrap();
    let (slow, slow_stats) = point_to_solid_exhaustive(&topo, query, row).unwrap();
    assert_eq!(fast.distance, slow.distance);
    assert_eq!(fast.point_b, slow.point_b);
    assert!((fast.distance - 2.0).abs() < 1e-9);
    assert_eq!(stats.faces_total, 120);
    assert_eq!(stats.faces_mandatory, 0);
    assert!(
        stats.faces_skipped_by_bound > 0,
        "a query beside the first box must prune distant boxes"
    );
    assert_eq!(
        stats.faces_evaluated + stats.faces_skipped_by_bound,
        stats.faces_total
    );
    assert_eq!(slow_stats.faces_evaluated, 120);
}

/// Randomized equivalence: random query points around unit boxes must give
/// bitwise-identical answers in both modes (same narrow phase, same order,
/// same strict-improvement rule).
#[cfg(test)]
mod randomized {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use proptest::prelude::*;

    use super::*;

    fn unit_cube(topo: &mut Topology) -> SolidId {
        make_box(topo, Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0))
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        #[test]
        fn random_queries_match_exhaustive(
            x in -2.0f64..3.0,
            y in -2.0f64..3.0,
            z in -2.0f64..3.0,
        ) {
            let mut topo = Topology::new();
            let solid = unit_cube(&mut topo);
            let query = Point3::new(x, y, z);
            let (fast, fast_stats) =
                point_to_solid_with_stats(&topo, query, solid).unwrap();
            let (slow, slow_stats) =
                point_to_solid_exhaustive(&topo, query, solid).unwrap();
            prop_assert_eq!(fast.distance, slow.distance);
            prop_assert_eq!(fast.point_b, slow.point_b);
            prop_assert_eq!(
                fast_stats.faces_evaluated + fast_stats.faces_skipped_by_bound,
                fast_stats.faces_total
            );
            prop_assert_eq!(slow_stats.faces_evaluated, slow_stats.faces_total);
        }

        #[test]
        fn random_queries_match_on_two_box_row(
            x in -2.0f64..8.0,
            y in -2.0f64..3.0,
            z in -2.0f64..3.0,
        ) {
            let mut topo = Topology::new();
            let a = make_box(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 1.0),
            );
            let b = make_box(
                &mut topo,
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(6.0, 1.0, 1.0),
            );
            let mut faces: Vec<FaceId> = Vec::new();
            for solid in [a, b] {
                let shell = topo.solid(solid).unwrap().outer_shell();
                faces.extend(topo.shell(shell).unwrap().faces().iter().copied());
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let row = topo.add_solid(Solid::new(shell, vec![]));
            let query = Point3::new(x, y, z);
            let (fast, _) = point_to_solid_with_stats(&topo, query, row).unwrap();
            let (slow, _) = point_to_solid_exhaustive(&topo, query, row).unwrap();
            prop_assert_eq!(fast.distance, slow.distance);
            prop_assert_eq!(fast.point_b, slow.point_b);
        }
    }
}
