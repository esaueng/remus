//! Probe: census cluster 3 — tangency ERR cases (box∪box edge-touch,
//! box∪cyl axis-tangent-to-corner-edge).
//!
//! `CASE=<name> cargo run --release --example debug_edge_touch -p remus-operations`
#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::panic,
    missing_docs
)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::transform::transform_solid;
use remus_operations::{measure, primitives};
use remus_topology::Topology;

fn main() {
    env_logger::init();
    let case = std::env::var("CASE").unwrap_or_else(|_| "edge-touch".into());
    let raw = std::env::var("RAW").is_ok();

    let mut topo = Topology::new();
    let a = primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
    let (b, mv) = match case.as_str() {
        "edge-touch" => (
            primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap(),
            (30.0, 18.0, 0.0),
        ),
        "corner-touch" => (
            primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap(),
            (30.0, 18.0, 24.0),
        ),
        "cyl-top-flush" => (
            primitives::make_cylinder(&mut topo, 6.0, 30.0).unwrap(),
            (-6.0, 0.0, -6.0),
        ),
        "cyl-bottom-flush" => (
            primitives::make_cylinder(&mut topo, 6.0, 30.0).unwrap(),
            (-6.0, 0.0, 0.0),
        ),
        "cyl-no-flush" => (
            primitives::make_cylinder(&mut topo, 6.0, 30.0).unwrap(),
            (-6.0, 0.0, -3.0),
        ),
        "cyl-24" => (
            primitives::make_cylinder(&mut topo, 6.0, 30.0).unwrap(),
            (24.0, 0.0, -6.0),
        ),
        other => panic!("unknown case {other}"),
    };
    let mat = Mat4::translation(mv.0, mv.1, mv.2);
    transform_solid(&mut topo, b, &mat).unwrap();

    let result = if raw {
        remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Fuse, a, b)
            .map_err(|e| format!("{e}"))
    } else {
        boolean(&mut topo, BooleanOp::Fuse, a, b).map_err(|e| format!("{e}"))
    };
    match result {
        Ok(r) => {
            let faces = remus_topology::explorer::solid_faces(&topo, r).unwrap();
            let vol = measure::solid_volume(&topo, r, 0.05).unwrap_or(-1.0);
            println!("{case}: ok F={} vol={vol:.3}", faces.len());
        }
        Err(e) => println!("{case}: ERR {e}"),
    }
}
