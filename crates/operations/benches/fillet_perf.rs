//! Fillet cascade cost: one successful edit and one typed refusal.
//!
//! * `fillet/plate_top_loop_r1`: a 40×30×4 plate whose four vertical corners
//!   were filleted at r5, then its eight-edge tangent top loop at r1. The
//!   rolling-ball result carries NURBS bands, so the timed call pays the
//!   volume guard's whole-solid meshes and the B71 convex-material sampler.
//! * `fillet/bracket_refusal_r{0.5,1,2}`: a whole-body fillet of a holed
//!   L-bracket, refused with `unsupported-vertex-blend`. The walking builder
//!   refuses before any stripe is built; what remains is the analysis that
//!   decides whether the selection splits into independent features.
//!
//! Run locally: `cargo bench -p remus-operations --bench fillet_perf`

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::blend_ops::{blend_failure_code, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_edges;
use remus_topology::solid::SolidId;

fn edges_where(
    topo: &Topology,
    solid: SolidId,
    keep: impl Fn(Point3, Point3) -> bool,
) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            keep(
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            )
        })
        .collect()
}

/// The plate with its vertical corners rounded at r5, and its top loop.
fn rounded_plate() -> (Topology, SolidId, Vec<EdgeId>) {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 40.0, 30.0, 4.0).unwrap();
    let verticals = edges_where(&topo, plate, |a, b| {
        (a.x() - b.x()).abs() < 1e-9 && (a.y() - b.y()).abs() < 1e-9
    });
    let rounded = fillet_cascade(&mut topo, plate, &verticals, 5.0)
        .unwrap()
        .solid;
    let top = edges_where(&topo, rounded, |a, b| {
        (a.z() - 4.0).abs() < 1e-9 && (b.z() - 4.0).abs() < 1e-9
    });
    assert_eq!(top.len(), 8);
    (topo, rounded, top)
}

/// L-bracket (40×30×4 base, 40×4×26 upright) with two bolt holes in the
/// base and one through the upright.
fn holed_bracket() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 40.0, 30.0, 4.0).unwrap();
    let upright = make_box(&mut topo, 40.0, 4.0, 26.0).unwrap();
    transform_solid(&mut topo, upright, &Mat4::translation(0.0, 0.0, 4.0)).unwrap();
    let mut body = boolean(&mut topo, BooleanOp::Fuse, base, upright).unwrap();
    for (x, y, z, along_y) in [
        (10.0, 18.0, -1.0, false),
        (30.0, 18.0, -1.0, false),
        (20.0, -1.0, 18.0, true),
    ] {
        let hole = make_cylinder(&mut topo, 2.5, 6.0).unwrap();
        if along_y {
            transform_solid(
                &mut topo,
                hole,
                &Mat4::rotation_x(-std::f64::consts::FRAC_PI_2),
            )
            .unwrap();
        }
        transform_solid(&mut topo, hole, &Mat4::translation(x, y, z)).unwrap();
        body = boolean(&mut topo, BooleanOp::Cut, body, hole).unwrap();
    }
    (topo, body)
}

fn bench_fillet(c: &mut Criterion) {
    let mut group = c.benchmark_group("fillet");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(4));

    let (topo, rounded, top) = rounded_plate();
    group.bench_function("plate_top_loop_r1", |bencher| {
        bencher.iter_batched(
            || topo.clone(),
            |mut t| black_box(fillet_cascade(&mut t, rounded, &top, 1.0).unwrap().solid),
            criterion::BatchSize::LargeInput,
        );
    });

    let (topo, bracket) = holed_bracket();
    let edges = solid_edges(&topo, bracket).unwrap();
    for radius in [0.5, 1.0, 2.0] {
        group.bench_function(format!("bracket_refusal_r{radius}"), |bencher| {
            bencher.iter_batched(
                || topo.clone(),
                |mut t| {
                    let Err(error) = fillet_cascade(&mut t, bracket, &edges, radius) else {
                        panic!("the whole-body bracket fillet must refuse");
                    };
                    assert_eq!(blend_failure_code(&error), "unsupported-vertex-blend");
                },
                criterion::BatchSize::LargeInput,
            );
        });
    }
    group.finish();
}

criterion_group!(benches, bench_fillet);
criterion_main!(benches);
