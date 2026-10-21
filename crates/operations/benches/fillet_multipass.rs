//! Multi-pass fillet workloads: many-edge and sequential fillets, and a
//! boolean through a filleted body.
//!
//! Two exact bodies, each filleted pass by pass through the cascade the
//! public `fillet` binding runs (`filter_filletable_edges` then
//! `blend_ops::fillet_cascade`):
//!
//! - **bracket**: the profile (0,0),(22,0),(22,8),(52,8),(52,0),(74,0),
//!   (74,60),(0,60) extruded by 4, fused with tabs x∈[0,18] and [56,74],
//!   y∈[0,10], z∈[0,40]; then the front vertical corners (r6), the slot
//!   corners (x∈{22,52}, y=8, r2), and a through hole with a countersink
//!   cut through the filleted plate.
//! - **plate**: a 40×30×4 box; the 4 vertical corners (r5), then the 8-edge
//!   tangent top loop (r1).
//!
//! Every pass is timed on its own, from a clone of the previous pass's
//! result (the clone is setup, not measured), and each body is also timed
//! end to end. Only passes that succeed on `main` are benched; every pass
//! panics on failure and each final body is checked valid, so a regression
//! fails loudly instead of timing an error path.
//!
//! Not benched yet (refused on `main` with `UnsupportedVertexBlend`, two
//! stripes): the bracket's tab top loops (the 10 edges at z=40, r2) after
//! the front corners. Add the pass between the slot corners and the hole
//! once it succeeds.
//!
//! Run locally: `cargo bench -p remus-operations --bench fillet_multipass`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::primitives::{make_box, make_cone, make_cylinder};
use remus_operations::query::filter_filletable_edges;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_planar_face;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_edges;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-7;
/// Coordinate match for edge selection; far below every feature size here.
const PICK: f64 = 1e-6;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < PICK
}

/// Edges of `solid` whose two endpoints both satisfy `at`.
fn edges_where(topo: &Topology, solid: SolidId, at: impl Fn(Point3) -> bool) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            at(a) && at(b)
        })
        .collect()
}

/// Vertical (z-parallel) edges of `solid` standing on one of the `xy` points.
fn vertical_edges_at(topo: &Topology, solid: SolidId, xy: &[(f64, f64)]) -> Vec<EdgeId> {
    let on = |p: Point3| xy.iter().any(|&(x, y)| near(p.x(), x) && near(p.y(), y));
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            on(a) && near(a.x(), b.x()) && near(a.y(), b.y()) && !near(a.z(), b.z())
        })
        .collect()
}

/// The `fillet` binding's path: drop tangent/degenerate edges, then cascade.
fn fillet(topo: &mut Topology, solid: SolidId, edges: &[EdgeId], radius: f64) -> SolidId {
    assert!(!edges.is_empty(), "selection must not be empty");
    let edges = filter_filletable_edges(topo, solid, edges).unwrap();
    fillet_cascade(topo, solid, &edges, radius)
        .expect("fillet pass succeeds on main")
        .solid
}

fn translated(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) -> SolidId {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
    solid
}

// ── bracket ──────────────────────────────────────────────────────────

fn bracket_body(topo: &mut Topology) -> SolidId {
    let profile: Vec<Point3> = [
        (0.0, 0.0),
        (22.0, 0.0),
        (22.0, 8.0),
        (52.0, 8.0),
        (52.0, 0.0),
        (74.0, 0.0),
        (74.0, 60.0),
        (0.0, 60.0),
    ]
    .iter()
    .map(|&(x, y)| Point3::new(x, y, 0.0))
    .collect();
    let face = make_planar_face(topo, &profile, TOL).unwrap();
    let mut body = extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
    for x in [0.0, 56.0] {
        let tab = make_box(topo, 18.0, 10.0, 40.0).unwrap();
        let tab = translated(topo, tab, x, 0.0, 0.0);
        body = boolean(topo, BooleanOp::Fuse, body, tab).expect("tab fuse");
    }
    body
}

fn bracket_front_corners(topo: &mut Topology, body: SolidId) -> SolidId {
    let edges = vertical_edges_at(topo, body, &[(0.0, 0.0), (74.0, 0.0)]);
    fillet(topo, body, &edges, 6.0)
}

fn bracket_slot_corners(topo: &mut Topology, body: SolidId) -> SolidId {
    let edges = vertical_edges_at(topo, body, &[(22.0, 8.0), (52.0, 8.0)]);
    assert_eq!(edges.len(), 2, "two slot corners");
    fillet(topo, body, &edges, 2.0)
}

/// A Ø6 through hole under a 90° countersink (Ø12 at the top face), cut
/// through the plate between the tabs. The cone runs past the top face so
/// no tool face is coplanar with the body.
fn bracket_hole_and_countersink(topo: &mut Topology, body: SolidId) -> SolidId {
    let (cx, cy) = (37.0, 34.0);
    let bore = make_cylinder(topo, 3.0, 8.0).unwrap();
    let bore = translated(topo, bore, cx, cy, -2.0);
    let body = boolean(topo, BooleanOp::Cut, body, bore).expect("through hole");
    let sink = make_cone(topo, 3.0, 7.0, 4.0).unwrap();
    let sink = translated(topo, sink, cx, cy, 1.0);
    boolean(topo, BooleanOp::Cut, body, sink).expect("countersink")
}

// ── plate ────────────────────────────────────────────────────────────

fn plate_corners(topo: &mut Topology, body: SolidId) -> SolidId {
    let corners = [(0.0, 0.0), (40.0, 0.0), (40.0, 30.0), (0.0, 30.0)];
    let edges = vertical_edges_at(topo, body, &corners);
    assert_eq!(edges.len(), 4, "four vertical corners");
    fillet(topo, body, &edges, 5.0)
}

fn plate_top_loop(topo: &mut Topology, body: SolidId) -> SolidId {
    let edges = edges_where(topo, body, |p| near(p.z(), 4.0));
    assert_eq!(edges.len(), 8, "tangent top loop: 4 lines + 4 arcs");
    fillet(topo, body, &edges, 1.0)
}

// ── benches ──────────────────────────────────────────────────────────

type Pass = fn(&mut Topology, SolidId) -> SolidId;

/// Times `pass` alone, each iteration on a fresh clone of `(topo, input)`.
fn bench_pass(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &str,
    topo: &Topology,
    input: SolidId,
    pass: Pass,
) {
    group.bench_function(name, |b| {
        b.iter_batched(
            || topo.clone(),
            |mut t| black_box(pass(&mut t, input)),
            BatchSize::LargeInput,
        );
    });
}

fn bench_bracket(c: &mut Criterion) {
    let mut group = c.benchmark_group("fillet_multipass/bracket");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(3));

    let passes: [(&str, Pass); 3] = [
        ("front_corners_r6", bracket_front_corners),
        ("slot_corners_r2", bracket_slot_corners),
        ("hole_countersink_cut", bracket_hole_and_countersink),
    ];
    let mut topo = Topology::new();
    let mut body = bracket_body(&mut topo);
    let start = (topo.clone(), body);
    for (name, pass) in passes {
        bench_pass(&mut group, name, &topo, body, pass);
        body = pass(&mut topo, body);
    }
    assert!(validate_solid(&topo, body).unwrap().is_valid());
    group.bench_function("full_sequence", |b| {
        b.iter_batched(
            || start.0.clone(),
            |mut t| {
                let mut s = start.1;
                for (_, pass) in passes {
                    s = pass(&mut t, s);
                }
                black_box(s)
            },
            BatchSize::LargeInput,
        );
    });
    group.finish();
}

fn bench_plate(c: &mut Criterion) {
    let mut group = c.benchmark_group("fillet_multipass/plate");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(3));

    let passes: [(&str, Pass); 2] = [
        ("corners_r5", plate_corners),
        ("top_loop_r1", plate_top_loop),
    ];
    let mut topo = Topology::new();
    let mut body = make_box(&mut topo, 40.0, 30.0, 4.0).unwrap();
    let start = (topo.clone(), body);
    for (name, pass) in passes {
        bench_pass(&mut group, name, &topo, body, pass);
        body = pass(&mut topo, body);
    }
    assert!(validate_solid(&topo, body).unwrap().is_valid());
    group.bench_function("full_sequence", |b| {
        b.iter_batched(
            || start.0.clone(),
            |mut t| {
                let mut s = start.1;
                for (_, pass) in passes {
                    s = pass(&mut t, s);
                }
                black_box(s)
            },
            BatchSize::LargeInput,
        );
    });
    group.finish();
}

criterion_group!(benches, bench_bracket, bench_plate);
criterion_main!(benches);
