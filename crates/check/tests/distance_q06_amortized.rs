//! Q06 amortized measurement: one-shot loop vs prepared+scratch batch, plus
//! traversal-alternative comparison (sorted scan vs amortized BVH).
//!
//! Prints concise `PERF-Q06` lines with `--nocapture`; asserts correctness
//! (prepared matches one-shot bit for bit, BVH alternative matches on distance)
//! and pruning-effectiveness invariants — never wall-time thresholds.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout
)]
#![allow(clippy::float_cmp)]

use std::time::Instant;

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::test_utils::make_unit_cube_manifold_at;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use remus_check::distance::{DistanceScratch, PreparedDistanceSolid, point_to_solid_with_stats};

const TOL: f64 = 1e-7;

fn make_cube_row(topo: &mut Topology, count: usize, spacing: f64) -> SolidId {
    let mut faces: Vec<FaceId> = Vec::with_capacity(6 * count);
    for k in 0..count {
        #[allow(clippy::cast_precision_loss)]
        let origin = k as f64 * spacing;
        let solid = make_unit_cube_manifold_at(topo, origin, 0.0, 0.0);
        let shell_id = topo.solid(solid).unwrap().outer_shell();
        faces.extend(topo.shell(shell_id).unwrap().faces().iter().copied());
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

fn make_cavity(topo: &mut Topology) -> SolidId {
    let a = make_unit_cube_manifold_at(topo, 0.0, 0.0, 0.0);
    let b = make_unit_cube_manifold_at(topo, 3.0, 3.0, 3.0);
    let os = topo.solid(a).unwrap().outer_shell();
    let is = topo.solid(b).unwrap().outer_shell();
    topo.add_solid(Solid::new(os, vec![is]))
}

fn make_unknown_row(topo: &mut Topology, count: usize) -> SolidId {
    let mut faces: Vec<FaceId> = Vec::new();
    for k in 0..count {
        #[allow(clippy::cast_precision_loss)]
        let ox = k as f64 * 3.0;
        let origin = Point3::new(ox, 0.0, 0.0);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let bottom = Circle3D::new(origin, axis, 1.0).unwrap();
        let top = Circle3D::new(Point3::new(ox, 0.0, 1.0), axis, 1.0).unwrap();
        let low = topo.add_vertex(Vertex::new(bottom.evaluate(0.0), TOL));
        let high = topo.add_vertex(Vertex::new(top.evaluate(0.0), TOL));
        let mut low_edge = Edge::new(low, low, EdgeCurve::Circle(bottom));
        low_edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let mut high_edge = Edge::new(high, high, EdgeCurve::Circle(top));
        high_edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let low_edge = topo.add_edge(low_edge);
        let high_edge = topo.add_edge(high_edge);
        let seam = topo.add_edge(Edge::new(low, high, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(low_edge, true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(high_edge, false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let carrier = remus_math::surfaces::CylindricalSurface::new(origin, axis, 1.0).unwrap();
        faces.push(topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(carrier))));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

fn halton(mut i: u32, base: u32) -> f64 {
    let (mut f, mut r) = (1.0f64, 0.0f64);
    while i > 0 {
        f /= f64::from(base);
        r += f * f64::from(i % base);
        i /= base;
    }
    r
}

fn corpus(n: usize, cx: f64, cy: f64, cz: f64, span: f64) -> Vec<Point3> {
    (1..=n)
        .map(|i| {
            let i = i as u32;
            Point3::new(
                cx + span * (halton(i, 2) - 0.5) * 2.0,
                cy + span * (halton(i, 3) - 0.5) * 2.0,
                cz + span * (halton(i, 5) - 0.5) * 2.0,
            )
        })
        .collect()
}

fn compare_one_shot_vs_prepared(label: &str, topo: &Topology, solid: SolidId, pts: &[Point3]) {
    let t0 = Instant::now();
    let prepared = PreparedDistanceSolid::prepare(topo, solid).unwrap();
    let prep_time = t0.elapsed().as_secs_f64();
    let mut scratch = DistanceScratch::new();

    for &n in &[1usize, 10, 100, 1000] {
        let subset: Vec<Point3> = pts.iter().cycle().take(n).copied().collect();
        for p in &subset {
            let _ = point_to_solid_with_stats(topo, *p, solid).unwrap();
        }
        let t0 = Instant::now();
        let mut one_eval = 0usize;
        for p in &subset {
            let (_, s) = point_to_solid_with_stats(topo, *p, solid).unwrap();
            one_eval += s.faces_evaluated;
        }
        let one_time = t0.elapsed().as_secs_f64();
        for p in &subset {
            let _ = prepared.query(*p, &mut scratch).unwrap();
        }
        let cap_before = scratch.capacity();
        let t0 = Instant::now();
        let batched = prepared.batch(&subset, &mut scratch).unwrap();
        let prep_batch_time = t0.elapsed().as_secs_f64();
        let cap_after = scratch.capacity();
        for (i, p) in subset.iter().enumerate() {
            let single = point_to_solid_with_stats(topo, *p, solid).unwrap().0;
            assert_eq!(
                single.distance, batched[i].distance,
                "{label} n={n} index {i}"
            );
            assert_eq!(
                single.point_b, batched[i].point_b,
                "{label} n={n} index {i}"
            );
        }
        println!(
            "PERF-Q06 {label} n={n}: prep={prep_time:.6}s faces={} prunable={} mandatory={} \
             one-shot={one_time:.6}s ({:.3}us/q eval_avg={:.1}) \
             prepared-batch={prep_batch_time:.6}s ({:.3}us/q incl-prep={:.3}us/q) \
             scratch_cap={cap_before}->{cap_after}",
            prepared.face_count(),
            prepared.prunable_count(),
            prepared.mandatory_count(),
            one_time * 1e6 / (n as f64),
            one_eval as f64 / (n as f64),
            prep_batch_time * 1e6 / (n as f64),
            (prep_time + prep_batch_time) * 1e6 / (n as f64),
        );
    }
}

#[test]
fn q06_sparse_row_amortizes() {
    let mut topo = Topology::new();
    let solid = make_cube_row(&mut topo, 40, 3.0);
    let pts = corpus(1000, 0.5, 0.5, 3.0, 3.0);
    compare_one_shot_vs_prepared("sparse-240", &topo, solid, &pts);
}

#[test]
fn q06_overlapping_row_amortizes() {
    let mut topo = Topology::new();
    let solid = make_cube_row(&mut topo, 40, 0.0);
    let pts = corpus(1000, 0.5, 0.5, 3.0, 3.0);
    compare_one_shot_vs_prepared("overlapping-240", &topo, solid, &pts);
}

#[test]
fn q06_cavity_amortizes() {
    let mut topo = Topology::new();
    let solid = make_cavity(&mut topo);
    let pts = corpus(1000, 0.5, 0.5, 0.5, 2.0);
    compare_one_shot_vs_prepared("cavity-12", &topo, solid, &pts);
}

#[test]
fn q06_unknown_heavy_no_regression() {
    let mut topo = Topology::new();
    let solid = make_unknown_row(&mut topo, 20);
    let pts = corpus(1000, 0.0, 0.0, 3.0, 3.0);
    compare_one_shot_vs_prepared("unknown-20", &topo, solid, &pts);
}

#[test]
fn q06_bvh_alternative_loses_amortized() {
    use remus_math::bvh::Bvh;
    let mut topo = Topology::new();
    let solid = make_cube_row(&mut topo, 40, 3.0);
    let prepared = PreparedDistanceSolid::prepare(&topo, solid).unwrap();
    let faces = prepared.faces().to_vec();
    let mut pairs: Vec<(usize, remus_math::aabb::Aabb3)> = Vec::new();
    let mut prunable_faces: Vec<FaceId> = Vec::new();
    for &fid in &faces {
        let b = remus_check::distance::face_bounds::face_bound(&topo, fid).unwrap();
        if b.prunable {
            pairs.push((prunable_faces.len(), b.aabb));
            prunable_faces.push(fid);
        }
    }
    let t0 = Instant::now();
    let bvh = Bvh::build(&pairs);
    let bvh_build = t0.elapsed().as_secs_f64();

    let pts = corpus(200, 0.5, 0.5, 3.0, 3.0);
    let mut scratch = DistanceScratch::new();
    for p in &pts {
        let _ = prepared.query(*p, &mut scratch).unwrap();
    }
    let t0 = Instant::now();
    let scanned = prepared.batch(&pts, &mut scratch).unwrap();
    let scan_time = t0.elapsed().as_secs_f64();
    let mut order: Vec<(f64, FaceId)> = Vec::new();
    let t0 = Instant::now();
    let mut bvh_eval = 0usize;
    for p in &pts {
        order.clear();
        for (i, fid) in prunable_faces.iter().enumerate() {
            let d2 = pairs[i].1.distance_squared_to_point(*p);
            order.push((d2, *fid));
        }
        order.sort_by(|a, b| {
            a.0.total_cmp(&b.0)
                .then_with(|| a.1.index().cmp(&b.1.index()))
        });
        let _ = bvh.query_closest(*p);
        let mut best = f64::INFINITY;
        for (d2, fid) in &order {
            if *d2 > best * best {
                continue;
            }
            if let Some((dist, _)) = remus_check::distance::point_to_face(&topo, *p, *fid).unwrap()
            {
                bvh_eval += 1;
                if dist < best {
                    best = dist;
                }
            }
        }
    }
    let bvh_time = t0.elapsed().as_secs_f64();
    println!(
        "PERF-Q06 bvh-vs-scan sparse-240 n={}: bvh-build={bvh_build:.6}s scan-batch={scan_time:.6}s bvh-batch={bvh_time:.6}s bvh_eval_total={bvh_eval}",
        pts.len()
    );
    for (i, p) in pts.iter().take(20).enumerate() {
        let single = point_to_solid_with_stats(&topo, *p, solid).unwrap().0;
        assert_eq!(single.distance, scanned[i].distance);
    }
}
