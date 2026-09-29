//! Q06 prepared distance: one-shot / prepared / batched / exhaustive equivalence.
//!
//! Proves the amortized path reproduces the one-shot contract bit for bit while
//! covering analytic fixtures, cavities, holes, unknown bounds, degenerate
//! inputs, large placements, scales, boundary minima, repeated calls after
//! narrow-phase failure, identical numeric IDs across documents, and
//! mutation-requires-rebuild.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![allow(clippy::float_cmp)]

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use remus_check::distance::{
    DistanceOptions, DistanceScratch, PreparedDistanceSolid, point_to_solid, point_to_solid_batch,
    point_to_solid_exhaustive, point_to_solid_with_stats,
};

const TOL: f64 = 1e-7;

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
    let quad = |topo: &mut Topology,
                edges: [(remus_topology::edge::EdgeId, bool); 4],
                normal: Vec3,
                d: f64| {
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
    };
    let faces = vec![
        quad(
            topo,
            [(e0, true), (e1, true), (e2, true), (e3, true)],
            Vec3::new(0.0, 0.0, -1.0),
            -min.z(),
        ),
        quad(
            topo,
            [(e4, true), (e5, true), (e6, true), (e7, true)],
            Vec3::new(0.0, 0.0, 1.0),
            max.z(),
        ),
        quad(
            topo,
            [(e0, true), (e9, true), (e4, false), (e8, false)],
            Vec3::new(0.0, -1.0, 0.0),
            -min.y(),
        ),
        quad(
            topo,
            [(e2, true), (e11, true), (e6, false), (e10, false)],
            Vec3::new(0.0, 1.0, 0.0),
            max.y(),
        ),
        quad(
            topo,
            [(e3, true), (e8, true), (e7, false), (e11, false)],
            Vec3::new(-1.0, 0.0, 0.0),
            -min.x(),
        ),
        quad(
            topo,
            [(e1, true), (e10, true), (e5, false), (e9, false)],
            Vec3::new(1.0, 0.0, 0.0),
            max.x(),
        ),
    ];
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// Assert all four execution modes agree exactly on one query.
fn assert_all_modes_agree(topo: &Topology, query: Point3, solid: SolidId) {
    let one_shot = point_to_solid(topo, query, solid).unwrap();
    let one_shot_stats = point_to_solid_with_stats(topo, query, solid).unwrap();
    let prepared = PreparedDistanceSolid::prepare(topo, solid).unwrap();
    let mut scratch = DistanceScratch::new();
    let via_query = prepared.query(query, &mut scratch).unwrap();
    let via_stats = prepared.query_with_stats(query, &mut scratch).unwrap();
    let exhaustive = prepared
        .query_exhaustive_with_stats(query, &mut scratch)
        .unwrap();
    let forced = point_to_solid_exhaustive(topo, query, solid).unwrap();
    let batch = point_to_solid_batch(topo, &[query], solid).unwrap();

    assert_eq!(one_shot.distance, via_query.distance, "query {query:?}");
    assert_eq!(one_shot.point_b, via_query.point_b, "query {query:?}");
    assert_eq!(one_shot.distance, via_stats.0.distance, "query {query:?}");
    assert_eq!(one_shot.distance, exhaustive.0.distance, "query {query:?}");
    assert_eq!(one_shot.distance, forced.0.distance, "query {query:?}");
    assert_eq!(one_shot.point_b, forced.0.point_b, "query {query:?}");
    assert_eq!(one_shot.distance, batch[0].distance, "query {query:?}");
    assert_eq!(one_shot.point_b, batch[0].point_b, "query {query:?}");
    assert_eq!(via_stats.1.faces_total, one_shot_stats.1.faces_total);
    assert_eq!(via_stats.1.faces_prunable, one_shot_stats.1.faces_prunable);
    assert_eq!(
        via_stats.1.faces_mandatory,
        one_shot_stats.1.faces_mandatory
    );
    assert_eq!(
        via_stats.1.faces_evaluated,
        one_shot_stats.1.faces_evaluated
    );
    assert_eq!(
        via_stats.1.faces_skipped_by_bound,
        one_shot_stats.1.faces_skipped_by_bound
    );
    assert_eq!(
        via_stats.1.narrow_phase_failures,
        one_shot_stats.1.narrow_phase_failures
    );
}

#[test]
fn box_analytic_fixtures_agree_across_modes() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let r = point_to_solid(&topo, Point3::new(0.5, 0.5, 3.0), solid).unwrap();
    assert!((r.distance - 2.0).abs() < 1e-9);
    assert!((r.point_b.z() - 1.0).abs() < 1e-9);
    for q in [
        Point3::new(0.5, 0.5, 3.0),
        Point3::new(3.0, 0.5, 0.5),
        Point3::new(2.0, 2.0, 2.0),
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(0.5, 0.5, 0.0),
        Point3::new(0.0, 0.0, 0.0),
    ] {
        assert_all_modes_agree(&topo, q, solid);
    }
    let centre = Point3::new(0.5, 0.5, 0.5);
    let first = point_to_solid(&topo, centre, solid).unwrap();
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    let mut scratch = DistanceScratch::new();
    for _ in 0..5 {
        let again = prepared.query(centre, &mut scratch).unwrap();
        assert_eq!(first.distance, again.distance);
        assert_eq!(first.point_b, again.point_b);
    }
}

#[test]
fn cavity_both_shells_agree() {
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
    let os = topo.solid(outer).unwrap().outer_shell();
    let is = topo.solid(inner).unwrap().outer_shell();
    let hollow = topo.add_solid(Solid::new(os, vec![is]));
    let r = point_to_solid(&topo, Point3::new(5.0, 5.0, 5.0), hollow).unwrap();
    assert!((r.distance - 2.0).abs() < 1e-6);
    for q in [
        Point3::new(5.0, 5.0, 5.0),
        Point3::new(5.0, 5.0, 20.0),
        Point3::new(0.0, 0.0, 0.0),
    ] {
        assert_all_modes_agree(&topo, q, hollow);
    }
    let prepared = PreparedDistanceSolid::prepare(&topo, hollow).unwrap();
    assert_eq!(prepared.face_count(), 12);
    assert_eq!(
        prepared.face_count(),
        prepared.prunable_count() + prepared.mandatory_count()
    );
}

#[test]
fn unknown_bound_heavy_stays_mandatory_and_matches() {
    let mut topo = Topology::new();
    let mut faces: Vec<FaceId> = Vec::new();
    for k in 0..8 {
        #[allow(clippy::cast_precision_loss)]
        let ox = k as f64 * 3.0;
        let rim = Circle3D::new(Point3::new(ox, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let seam = topo.add_vertex(Vertex::new(Point3::new(ox + 1.0, 0.0, 0.0), TOL));
        let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(rim));
        edge.set_trim(Some((0.0, std::f64::consts::TAU + 0.5)));
        let rim_id = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim_id, true)], true).unwrap());
        faces.push(topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        )));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    assert_eq!(prepared.mandatory_count(), 8);
    assert_eq!(prepared.prunable_count(), 0);
    for q in [Point3::new(0.0, 0.0, 3.0), Point3::new(5.0, 0.0, 1.0)] {
        assert_all_modes_agree(&topo, q, solid);
    }
    let pts = vec![
        Point3::new(0.0, 0.0, 3.0),
        Point3::new(3.0, 0.0, 3.0),
        Point3::new(6.0, 0.0, 3.0),
    ];
    let mut scratch = DistanceScratch::new();
    let batched = prepared.batch(&pts, &mut scratch).unwrap();
    for (i, p) in pts.iter().enumerate() {
        let single = point_to_solid(&topo, *p, solid).unwrap();
        assert_eq!(single.distance, batched[i].distance);
        assert_eq!(single.point_b, batched[i].point_b);
    }
}

#[test]
fn batch_preserves_input_order_and_matches_one_shot() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let pts = vec![
        Point3::new(0.5, 0.5, 3.0),
        Point3::new(3.0, 0.5, 0.5),
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(2.0, 2.0, 2.0),
        Point3::new(0.5, 0.5, 3.0),
    ];
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    let mut scratch = DistanceScratch::new();
    let batched = prepared.batch(&pts, &mut scratch).unwrap();
    assert_eq!(batched.len(), pts.len());
    for (i, p) in pts.iter().enumerate() {
        let single = point_to_solid(&topo, *p, solid).unwrap();
        assert_eq!(single.distance, batched[i].distance, "index {i}");
        assert_eq!(single.point_b, batched[i].point_b, "index {i}");
    }
    assert_eq!(batched[0].distance, batched[4].distance);
    let free = point_to_solid_batch(&topo, &pts, solid).unwrap();
    assert_eq!(free.len(), pts.len());
    for i in 0..pts.len() {
        assert_eq!(free[i].distance, batched[i].distance);
    }
}

#[test]
fn empty_batch_clears_scratch_and_returns_empty() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    let mut scratch = DistanceScratch::with_capacity(16);
    let _ = prepared
        .query(Point3::new(0.5, 0.5, 3.0), &mut scratch)
        .unwrap();
    assert!(!scratch.is_empty());
    let out = prepared.batch(&[], &mut scratch).unwrap();
    assert!(out.is_empty());
    assert!(scratch.is_empty());
    let again = prepared
        .query(Point3::new(0.5, 0.5, 3.0), &mut scratch)
        .unwrap();
    assert!((again.distance - 2.0).abs() < 1e-9);
    let free = point_to_solid_batch(&topo, &[], solid).unwrap();
    assert!(free.is_empty());
}

#[test]
fn scratch_reuse_needs_no_reallocation_after_warmup() {
    let mut topo = Topology::new();
    let mut faces: Vec<FaceId> = Vec::new();
    for k in 0..10 {
        let s = make_box(
            &mut topo,
            Point3::new(k as f64 * 3.0, 0.0, 0.0),
            Point3::new(k as f64 * 3.0 + 1.0, 1.0, 1.0),
        );
        let sh = topo.solid(s).unwrap().outer_shell();
        faces.extend(topo.shell(sh).unwrap().faces().iter().copied());
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let row = topo.add_solid(Solid::new(shell, vec![]));
    let prepared = PreparedDistanceSolid::prepare(&topo, row).unwrap();
    let mut scratch = DistanceScratch::new();
    let q = Point3::new(0.5, 0.5, 3.0);
    let _ = prepared.query(q, &mut scratch).unwrap();
    let cap = scratch.capacity();
    assert!(cap >= prepared.prunable_count());
    for i in 0..20 {
        let p = Point3::new(0.5 + i as f64 * 0.1, 0.5, 3.0);
        let _ = prepared.query(p, &mut scratch).unwrap();
        assert_eq!(scratch.capacity(), cap);
    }
}

#[test]
fn identical_numeric_ids_across_documents_stay_distinct() {
    let mut a = Topology::new();
    let sa = make_box(
        &mut a,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let mut b = Topology::new();
    let sb = make_box(
        &mut b,
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(11.0, 1.0, 1.0),
    );
    assert_eq!(sa.index(), sb.index());
    let pa = PreparedDistanceSolid::prepare(&a, sa).unwrap();
    let pb = PreparedDistanceSolid::prepare(&b, sb).unwrap();
    let mut scratch = DistanceScratch::new();
    let q = Point3::new(0.5, 0.5, 3.0);
    let ra = pa.query(q, &mut scratch).unwrap();
    let rb = pb.query(q, &mut scratch).unwrap();
    assert!((ra.distance - 2.0).abs() < 1e-9);
    let expected_b = (9.5f64.mul_add(9.5, 4.0)).sqrt();
    assert!((rb.distance - expected_b).abs() < 1e-6);
    assert_ne!(ra.distance, rb.distance);
}

#[test]
fn large_translation_and_scale_match_analytic() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(1e6, 0.0, 0.0),
        Point3::new(1e6 + 1.0, 1.0, 1.0),
    );
    assert_all_modes_agree(&topo, Point3::new(1e6 + 0.5, 0.5, 3.0), solid);
    let mut small = Topology::new();
    let ss = make_box(
        &mut small,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1e-3, 1e-3, 1e-3),
    );
    let r = point_to_solid(&small, Point3::new(5e-4, 5e-4, 5e-4), ss).unwrap();
    assert!((r.distance - 5e-4).abs() < 1e-12);
    assert_all_modes_agree(&small, Point3::new(5e-4, 5e-4, 5e-4), ss);
    let mut big = Topology::new();
    let sb = make_box(
        &mut big,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1e3, 1e3, 1e3),
    );
    assert_all_modes_agree(&big, Point3::new(500.0, 500.0, 1500.0), sb);
}

#[test]
fn degenerate_points_match_one_shot() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let pts = vec![
        Point3::new(0.5, 0.5, 3.0),
        Point3::new(0.5, 0.5, 3.0),
        Point3::new(0.5, 0.5, 1.0),
        Point3::new(f64::NAN, 0.0, 0.0),
        Point3::new(f64::INFINITY, 0.0, 0.0),
    ];
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    let mut scratch = DistanceScratch::new();
    for p in &pts {
        let one = point_to_solid(&topo, *p, solid).unwrap();
        let via = prepared.query(*p, &mut scratch).unwrap();
        assert_eq!(
            one.distance.to_bits(),
            via.distance.to_bits(),
            "point {p:?}"
        );
        assert_eq!(one.point_b.x().to_bits(), via.point_b.x().to_bits());
    }
}

#[test]
fn repeated_calls_after_narrow_phase_failure_agree() {
    let mut topo = Topology::new();
    let (patch, _) = wavy_patch(&mut topo, 0.25, 3.0, 0.0);
    let query = Point3::new(0.5, 0.5, 2.0);
    let (fast, fast_stats) = point_to_solid_with_stats(&topo, query, patch).unwrap();
    assert_eq!(fast_stats.narrow_phase_failures, 1);
    assert!(fast.distance.is_infinite());
    let prepared = PreparedDistanceSolid::prepare(&topo, patch).unwrap();
    let mut scratch = DistanceScratch::new();
    for _ in 0..3 {
        let (via, stats) = prepared.query_with_stats(query, &mut scratch).unwrap();
        assert_eq!(via.distance, fast.distance);
        assert_eq!(via.point_b, fast.point_b);
        assert_eq!(stats.narrow_phase_failures, 1);
    }
    let (exhaustive, estats) = prepared
        .query_exhaustive_with_stats(query, &mut scratch)
        .unwrap();
    assert_eq!(exhaustive.distance, fast.distance);
    assert_eq!(estats.narrow_phase_failures, 1);
}

#[test]
fn mutation_requires_dropping_and_rebuilding() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let before = {
        let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
        let mut scratch = DistanceScratch::new();
        prepared
            .query(Point3::new(0.5, 0.5, 3.0), &mut scratch)
            .unwrap()
    };
    assert!((before.distance - 2.0).abs() < 1e-9);
    let other = make_box(
        &mut topo,
        Point3::new(5.0, 0.0, 0.0),
        Point3::new(6.0, 1.0, 1.0),
    );
    let mut faces: Vec<FaceId> = Vec::new();
    for s in [solid, other] {
        let sh = topo.solid(s).unwrap().outer_shell();
        faces.extend(topo.shell(sh).unwrap().faces().iter().copied());
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let wider = topo.add_solid(Solid::new(shell, vec![]));
    let prepared = PreparedDistanceSolid::prepare(&topo, wider).unwrap();
    assert_eq!(prepared.face_count(), 12);
    let mut scratch = DistanceScratch::new();
    let after = prepared
        .query(Point3::new(5.5, 0.5, 3.0), &mut scratch)
        .unwrap();
    assert!((after.distance - 2.0).abs() < 1e-9);
}

#[test]
fn invalid_options_and_solid_report_typed_errors() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    for bad in [f64::NAN, f64::INFINITY, 0.0, -1e-7] {
        let err = PreparedDistanceSolid::prepare_with_options(
            &topo,
            solid,
            DistanceOptions {
                projection_tolerance: bad,
            },
        )
        .unwrap_err();
        assert!(
            format!("{err:?}").contains("projection tolerance"),
            "unexpected error for {bad}: {err:?}"
        );
    }
    let empty = Topology::new();
    let prepared_err = PreparedDistanceSolid::prepare(&empty, solid).unwrap_err();
    let oneshot_err = point_to_solid(&empty, Point3::new(0.5, 0.5, 3.0), solid).unwrap_err();
    assert_eq!(format!("{prepared_err:?}"), format!("{oneshot_err:?}"));
    assert!(point_to_solid_batch(&empty, &[], solid).is_err());
}

#[test]
fn accessors_report_the_frozen_preparation() {
    let mut topo = Topology::new();
    let solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    assert_eq!(prepared.solid(), solid);
    assert_eq!(prepared.face_count(), 6);
    assert_eq!(prepared.prunable_count(), 6);
    assert_eq!(prepared.mandatory_count(), 0);
    assert_eq!(prepared.faces().len(), 6);
    assert_eq!(
        prepared.options(),
        DistanceOptions {
            projection_tolerance: 1e-7
        }
    );
}

fn wavy_patch(
    topo: &mut Topology,
    amp: f64,
    freq: f64,
    ox: f64,
) -> (SolidId, remus_math::nurbs::surface::NurbsSurface) {
    use remus_math::nurbs::surface::NurbsSurface;
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

#[test]
fn query_error_clears_populated_scratch() {
    let mut topo = Topology::new();
    let box_solid = make_box(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let box_shell = topo.solid(box_solid).unwrap().outer_shell();
    let mut faces = topo.shell(box_shell).unwrap().faces().to_vec();

    let seam = topo.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), TOL));
    let corner_b = topo.add_vertex(Vertex::new(Point3::new(3.0, 0.0, 0.0), TOL));
    let corner_c = topo.add_vertex(Vertex::new(Point3::new(2.0, 1.0, 0.0), TOL));
    let rim = Circle3D::new(Point3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let mut unknown = Edge::new(seam, corner_b, EdgeCurve::Circle(rim));
    unknown.set_trim(Some((0.0, std::f64::consts::TAU + 0.5)));
    let unknown_id = topo.add_edge(unknown);

    let mut other_topo = Topology::new();
    let missing_vertex = (0..20)
        .map(|_| other_topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL)))
        .last()
        .unwrap();
    let broken_id = topo.add_edge(Edge::new(corner_b, missing_vertex, EdgeCurve::Line));
    let closing_id = topo.add_edge(Edge::new(corner_c, seam, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(unknown_id, true),
                OrientedEdge::new(broken_id, true),
                OrientedEdge::new(closing_id, true),
            ],
            true,
        )
        .unwrap(),
    );
    faces.push(topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    )));
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    assert!(prepared.prunable_count() > 0);
    assert_eq!(prepared.mandatory_count(), 1);

    let mut scratch = DistanceScratch::new();
    let point = Point3::new(20.0, 20.0, 3.0);
    assert!(prepared.query(point, &mut scratch).is_err());
    assert!(scratch.is_empty());
    assert!(
        prepared
            .query_exhaustive_with_stats(point, &mut scratch)
            .is_err()
    );
    assert!(scratch.is_empty());
    assert!(prepared.batch(&[point], &mut scratch).is_err());
    assert!(scratch.is_empty());
    assert!(point_to_solid_batch(&topo, &[], solid).is_err());
}
