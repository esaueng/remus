#![allow(clippy::unwrap_used, clippy::print_stdout, missing_docs)]
use remus_math::{
    aabb::Aabb3,
    bvh::Bvh,
    nurbs::{
        fitting::interpolate,
        intersection::{IntersectionPoint, chain_intersection_points},
        surface::NurbsSurface,
    },
    vec::Point3,
};
use remus_operations::primitives::make_box;
use remus_topology::{Topology, transaction::run_transacted};
use std::{hint::black_box, time::Instant};

fn measure(mut f: impl FnMut(), count: usize) -> (f64, f64, f64) {
    f();
    let mut times = Vec::new();
    for _ in 0..9 {
        let start = Instant::now();
        for _ in 0..count {
            f();
        }
        times.push(start.elapsed().as_secs_f64() * 1e6 / count as f64);
    }
    times.sort_by(f64::total_cmp);
    (times[0], times[4], times[8])
}

fn main() {
    println!(
        "Units: microseconds per operation; min/median/max of 9 batches; one warmup; setup excluded."
    );
    for n in [50, 200, 800, 3200] {
        let mut topo = Topology::new();
        for _ in 0..n {
            make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        }
        let slots = topo.allocated_slot_count();
        let time = measure(
            || {
                black_box(topo.clone());
            },
            10,
        );
        println!("topology_clone boxes={n} slots={slots} us={time:?}");
        let time = measure(
            || {
                run_transacted(&mut topo, |t| {
                    black_box(t.num_vertices());
                    Ok::<(), ()>(())
                })
                .unwrap();
            },
            10,
        );
        assert_eq!(topo.allocated_slot_count(), slots);
        println!("noop_transaction boxes={n} slots={slots} us={time:?}");
    }
    for n in [128, 512, 2048] {
        let points: Vec<_> = (0..n)
            .map(|i| IntersectionPoint {
                point: Point3::new(i as f64, 0.0, 0.0),
                param1: (i as f64, 0.0),
                param2: (i as f64, 0.0),
            })
            .collect();
        let chains = chain_intersection_points(&points, 1.1);
        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0].len(), n);
        let time = measure(
            || {
                black_box(chain_intersection_points(black_box(&points), 1.1));
            },
            3,
        );
        println!("chain_points n={n} us={time:?}");
        let fit_points: Vec<_> = points
            .iter()
            .map(|p| Point3::new(p.point.x(), (p.point.x() * 0.01).sin(), 0.0))
            .collect();
        let curve = interpolate(&fit_points, 3).unwrap();
        let (a, b) = curve.domain();
        assert!((curve.evaluate(a) - fit_points[0]).length() < 1e-8);
        assert!((curve.evaluate(b) - fit_points[n - 1]).length() < 1e-8);
        let time = measure(
            || {
                black_box(interpolate(black_box(&fit_points), 3).unwrap());
            },
            3,
        );
        println!(
            "interpolate_cubic n={n} dense_matrix_bytes={} us={time:?}",
            n * n * 8
        );
    }
    for n in [100, 1000, 10000] {
        let boxes: Vec<_> = (0..n)
            .map(|i| {
                let x = (i % 100) as f64 * 2.0;
                let y = (i / 100) as f64 * 2.0;
                (
                    i,
                    Aabb3::try_from_points([
                        Point3::new(x, y, 0.0),
                        Point3::new(x + 1.0, y + 1.0, 1.0),
                    ])
                    .unwrap(),
                )
            })
            .collect();
        let time = measure(
            || {
                black_box(Bvh::build(black_box(&boxes)));
            },
            3,
        );
        println!("bvh_build n={n} us={time:?}");
        let bvh = Bvh::build(&boxes);
        let query =
            Aabb3::try_from_points([Point3::new(0.1, 0.1, 0.1), Point3::new(0.9, 0.9, 0.9)])
                .unwrap();
        assert_eq!(bvh.query_overlap(&query), vec![0]);
        let time = measure(
            || {
                black_box(bvh.query_overlap(black_box(&query)));
            },
            1000,
        );
        println!("bvh_reused_query n={n} us={time:?}");
    }
    for degree in [3, 9] {
        let mut knots = vec![0.0; degree + 1];
        knots.extend(vec![1.0; degree + 1]);
        let cps: Vec<Vec<_>> = (0..=degree)
            .map(|i| {
                (0..=degree)
                    .map(|j| {
                        Point3::new(
                            i as f64,
                            j as f64,
                            (i as f64 * 0.3).sin() * (j as f64 * 0.3).cos(),
                        )
                    })
                    .collect()
            })
            .collect();
        let weights = (0..=degree)
            .map(|i| {
                (0..=degree)
                    .map(|j| 1.0 + 0.05 * ((i + j) % 3) as f64)
                    .collect()
            })
            .collect();
        let surface =
            NurbsSurface::new(degree, degree, knots.clone(), knots, cps, weights).unwrap();
        let mut evaluator = surface.evaluator();
        for i in 0..100 {
            let u = i as f64 / 100.0;
            assert!((surface.evaluate(u, 0.37) - evaluator.point(u, 0.37)).length() < 1e-7);
        }
        let time = measure(
            || {
                for i in 0..100 {
                    black_box(surface.evaluate(black_box(i as f64 / 100.0), black_box(0.37)));
                }
            },
            50,
        );
        println!("surface_direct_100 degree={degree} us={time:?}");
        let time = measure(
            || {
                for i in 0..100 {
                    black_box(evaluator.point(black_box(i as f64 / 100.0), black_box(0.37)));
                }
            },
            50,
        );
        println!("surface_cached_100 degree={degree} us={time:?}");
    }
}
