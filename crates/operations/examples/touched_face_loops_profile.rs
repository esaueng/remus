//! Time a general (non-cell) cut against the loops its touched faces carry.
//!
//! A plate is drilled with an `n × n` grid of holes, then cut by rectangular
//! slots that pass between the holes. The slots meet only the plate's top and
//! bottom faces, which already carry `n²` hole loops each, so their cost shows
//! how much of the GFA scales with existing loops on touched faces (see
//! `docs/performance/perf-local-boolean.md`, stage 3).
//!
//! Run with
//! `cargo run --profile profiling -p remus-operations --example touched_face_loops_profile -- [n...]`
//! (default `0 6 12 18`); `repeat <n> <k>` cuts `k` slots on one `n` plate,
//! for a profiler. A leading `memo` enables the GFA's content-keyed memos,
//! as the WASM kernel does.

#![allow(clippy::unwrap_used, clippy::print_stdout, missing_docs)]

use std::time::Instant;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// A plate holding an `n × n` grid of r = 2 through-holes on a 12 pitch.
fn drilled_plate(topo: &mut Topology, n: u32) -> SolidId {
    let extent = f64::from(n.max(4)) * 12.0 + 4.0;
    let mut plate = primitives::make_box(topo, extent, extent, 10.0).unwrap();
    for row in 0..n {
        for col in 0..n {
            let tool = primitives::make_cylinder(topo, 2.0, 20.0).unwrap();
            let at = Mat4::translation(
                6.0 + f64::from(col) * 12.0,
                6.0 + f64::from(row) * 12.0,
                -5.0,
            );
            transform_solid(topo, tool, &at).unwrap();
            plate = boolean(topo, BooleanOp::Cut, plate, tool).unwrap();
        }
    }
    plate
}

/// Cut `k` 2 × 6 slots between hole columns; returns milliseconds per cut.
fn cut_slots(topo: &mut Topology, mut plate: SolidId, k: u32) -> (SolidId, f64) {
    let mut total = 0.0;
    for slot in 0..k {
        let tool = primitives::make_box(topo, 2.0, 6.0, 20.0).unwrap();
        let at = Mat4::translation(11.0 + f64::from(slot % 4) * 12.0, 9.0, -5.0);
        transform_solid(topo, tool, &at).unwrap();
        let start = Instant::now();
        plate = boolean(topo, BooleanOp::Cut, plate, tool).unwrap();
        total += start.elapsed().as_secs_f64() * 1e3;
    }
    (plate, total / f64::from(k))
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // `memo` first: enable the GFA's content-keyed memos (planar arc
    // pcurves, revolved wire areas), as the WASM kernel does (the library
    // leaves them off).
    if args.first().map(String::as_str) == Some("memo") {
        args.remove(0);
        remus_algo::pcurve_memo::enable_thread_pcurve_memo();
        remus_algo::wire_area_memo::enable_thread_wire_area_memo();
    }
    if args.first().map(String::as_str) == Some("repeat") {
        let n: u32 = args.get(1).map_or(18, |s| s.parse().unwrap());
        let k: u32 = args.get(2).map_or(200, |s| s.parse().unwrap());
        let mut topo = Topology::new();
        let plate = drilled_plate(&mut topo, n);
        let mut per = 0.0;
        for _ in 0..k {
            let mut copy = topo.clone();
            per += cut_slots(&mut copy, plate, 1).1;
        }
        println!("n {n}: {:.3} ms per slot over {k} runs", per / f64::from(k));
        return;
    }
    let sizes: Vec<u32> = if args.is_empty() {
        vec![0, 6, 12, 18]
    } else {
        args.iter().map(|s| s.parse().unwrap()).collect()
    };
    for n in sizes {
        let mut topo = Topology::new();
        let plate = drilled_plate(&mut topo, n);
        let faces = remus_topology::explorer::solid_faces(&topo, plate)
            .unwrap()
            .len();
        let (_, per) = cut_slots(&mut topo, plate, 4);
        println!("holes {:4} faces {faces:4}: {per:8.3} ms per slot", n * n);
    }
}
