//! Time each cut of a plate drilled one hole at a time.
//!
//! The general GFA re-processes every face of the blank on every boolean, so
//! a plate's per-cut cost grows with the holes already drilled. This probe
//! drills an `n × n` grid of through-holes in sequence and prints the cost of
//! selected cuts against the blank's face count, which exposes that slope
//! (see `docs/performance/perf-local-boolean.md`).
//!
//! Run with
//! `cargo run --profile profiling -p remus-operations --example sequential_cut_profile -- [n] [blind]`.
//! `blind` drills blind holes instead, which stay on the general GFA path.

#![allow(clippy::unwrap_used, clippy::print_stdout, missing_docs)]

use std::time::Instant;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn main() {
    let side: u32 = std::env::args().nth(1).map_or(16, |s| s.parse().unwrap());
    let blind = std::env::args().nth(2).is_some_and(|mode| mode == "blind");
    let mut topo = Topology::new();
    let extent = f64::from(side) * 12.0 + 4.0;
    let mut plate = primitives::make_box(&mut topo, extent, extent, 10.0).unwrap();
    let (base, height) = if blind { (4.0, 20.0) } else { (-5.0, 20.0) };
    let mut cut = 0_u32;
    let mut total = 0.0;
    for row in 0..side {
        for col in 0..side {
            let tool = primitives::make_cylinder(&mut topo, 2.0, height).unwrap();
            let x = 6.0 + f64::from(col) * 12.0;
            let y = 6.0 + f64::from(row) * 12.0;
            transform_solid(&mut topo, tool, &Mat4::translation(x, y, base)).unwrap();
            let start = Instant::now();
            plate = boolean(&mut topo, BooleanOp::Cut, plate, tool).unwrap();
            let elapsed = start.elapsed().as_secs_f64() * 1e3;
            total += elapsed;
            cut += 1;
            if cut == 1 || cut.is_multiple_of(side * 2) {
                let faces = remus_topology::explorer::solid_faces(&topo, plate)
                    .unwrap()
                    .len();
                println!("cut {cut:4}: {elapsed:8.3} ms  faces {faces}");
            }
        }
    }
    println!("total {total:.1} ms for {cut} cuts");
}
