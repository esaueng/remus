//! Native disjoint-pattern scaling probe. Arguments are instance count,
//! unrelated box count, retained samples, warmups, and optional new arena path.
//!
//! Build with `cargo build -p remus-operations --example perf_pattern_copy
//! --profile profiling`, then run the executable directly on a reserved core.
//! Fixture setup, output qualification, serialization and teardown are untimed.
//! Compare distinct baseline/candidate processes with balanced launch ordering.

#![allow(clippy::expect_used, clippy::print_stdout)]
// Unit boxes and integer translations have exact bounds and unit volume.
#![allow(clippy::float_cmp)]

use std::fs::OpenOptions;
use std::io::Write;
use std::time::Instant;

use remus_math::vec::Vec3;
use remus_operations::{measure, pattern, primitives};
use remus_topology::Topology;

fn main() {
    let arguments = std::env::args().skip(1).collect::<Vec<_>>();
    assert!(arguments.len() <= 5, "expected at most five arguments");
    let parse = |index: usize, default: usize| {
        arguments.get(index).map_or(default, |value| {
            value.parse::<usize>().expect("argument must be an integer")
        })
    };
    let instances = parse(0, 100);
    let unrelated = parse(1, 1000);
    let samples = parse(2, 11);
    let warmups = parse(3, 2);
    assert!((1..=10_000).contains(&instances));
    assert!(unrelated <= 10_000 && (1..=1000).contains(&samples) && warmups <= 100);

    let mut expected_arena = None;
    for sample in 0..(warmups + samples) {
        let mut topo = Topology::new();
        for _ in 0..unrelated {
            primitives::make_box(&mut topo, 2.0, 2.0, 2.0).expect("unrelated box");
        }
        let source = primitives::make_box(&mut topo, 1.0, 1.0, 1.0).expect("source box");

        let started = Instant::now();
        let compound =
            pattern::linear_pattern(&mut topo, source, Vec3::new(1.0, 0.0, 0.0), 2.0, instances)
                .expect("disjoint pattern");
        let elapsed_ns = u64::try_from(started.elapsed().as_nanos()).expect("elapsed duration");

        let solids = topo.compound(compound).expect("pattern compound").solids();
        assert_eq!(solids.len(), instances);
        assert_eq!(solids[0], source);
        assert_eq!(topo.num_solids(), unrelated + instances);
        assert_eq!(topo.num_faces(), 6 * (unrelated + instances));
        for (index, &solid) in solids.iter().enumerate() {
            let bounds = measure::solid_bounding_box(&topo, solid).expect("instance bounds");
            let offset = 2.0 * f64::from(u32::try_from(index).expect("instance index"));
            assert_eq!(bounds.min.x(), offset);
            assert_eq!(bounds.max.x(), offset + 1.0);
            assert_eq!((bounds.min.y(), bounds.max.y()), (0.0, 1.0));
            assert_eq!((bounds.min.z(), bounds.max.z()), (0.0, 1.0));
            assert_eq!(
                measure::solid_volume(&topo, solid, 0.1).expect("instance volume"),
                1.0
            );
        }
        let arena = remus_io::arena_io::serialize_solids(&topo, solids).expect("exact arena");
        if let Some(expected) = &expected_arena {
            assert_eq!(&arena, expected, "exact output changed across samples");
        } else {
            if let Some(path) = arguments.get(4) {
                OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(path)
                    .expect("arena output path must be new")
                    .write_all(&arena)
                    .expect("arena output write");
            }
            expected_arena = Some(arena.clone());
        }
        println!(
            "{}",
            serde_json::json!({
                "schema_version": 1,
                "workload": "linear_pattern_disjoint_boxes",
                "instances": instances,
                "unrelated_boxes": unrelated,
                "sample": sample,
                "warmup": sample < warmups,
                "elapsed_ns": elapsed_ns,
                "arena_bytes": arena.len(),
                "output_solids": solids.len(),
                "total_solids": topo.num_solids(),
                "total_faces": topo.num_faces(),
            })
        );
    }
}
