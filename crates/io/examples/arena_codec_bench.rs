//! Times the exact arena codec on representative documents.
//!
//! ```text
//! cargo run --release -p remus-io --example arena_codec_bench -- prep <dir>
//! cargo run --release -p remus-io --example arena_codec_bench -- de <document> <n>
//! cargo run --release -p remus-io --example arena_codec_bench -- ser <document> <n>
//! ```
//!
//! `prep` writes `hammer.arena` (the imported hammer-holder STEP fixture),
//! `boxes500.arena` (500 box roots) and `box.arena` (one box). `de`
//! deserializes a document `n` times into fresh topologies; `ser` loads it
//! once and serializes its roots `n` times. Both print the median and
//! minimum time per call; run them under `/usr/bin/time -l` for retired
//! instruction counts when comparing builds.

#![allow(clippy::unwrap_used, clippy::print_stdout)]

use std::time::Instant;

use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_operations::primitives::make_box;
use remus_topology::Topology;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("prep") => {
            let out = std::path::Path::new(&args[2]);
            let data = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
            let step = std::fs::read_to_string(data.join("shapr3d_hammer_holder.step")).unwrap();
            let mut topo = Topology::new();
            let solids = remus_io::step::reader::read_step(&step, &mut topo).unwrap();
            let hammer = serialize_solids(&topo, &solids).unwrap();
            std::fs::write(out.join("hammer.arena"), hammer).unwrap();
            let mut topo = Topology::new();
            let boxes: Vec<_> = (0..500)
                .map(|i| make_box(&mut topo, 1.0 + f64::from(i) * 0.01, 2.0, 3.0).unwrap())
                .collect();
            let many = serialize_solids(&topo, &boxes).unwrap();
            std::fs::write(out.join("boxes500.arena"), many).unwrap();
            let one = serialize_solids(&topo, &boxes[..1]).unwrap();
            std::fs::write(out.join("box.arena"), one).unwrap();
        }
        Some(mode @ ("de" | "ser")) => {
            let bytes = std::fs::read(&args[2]).unwrap();
            let n: usize = args[3].parse().unwrap();
            let mut times = Vec::with_capacity(n);
            let mut checksum = 0_usize;
            if mode == "de" {
                for _ in 0..n {
                    let mut topo = Topology::new();
                    let start = Instant::now();
                    let roots = deserialize_solids(&bytes, &mut topo).unwrap();
                    times.push(start.elapsed().as_nanos());
                    checksum += roots.len() + topo.num_faces();
                }
            } else {
                let mut topo = Topology::new();
                let roots = deserialize_solids(&bytes, &mut topo).unwrap();
                for _ in 0..n {
                    let start = Instant::now();
                    let out = serialize_solids(&topo, &roots).unwrap();
                    times.push(start.elapsed().as_nanos());
                    checksum += out.len();
                }
            }
            times.sort_unstable();
            println!(
                "{mode} {} median_us {:.1} min_us {:.1} checksum {checksum}",
                args[2],
                times[times.len() / 2] as f64 / 1000.0,
                times[0] as f64 / 1000.0
            );
        }
        _ => {
            println!(
                "usage: arena_codec_bench prep <dir> | de <document> <n> | ser <document> <n>"
            );
        }
    }
}
