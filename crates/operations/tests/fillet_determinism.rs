//! Cross-process determinism gate for the rolling-ball fillet.
//!
//! The fillet's adjacency, setback and contact tables were std
//! `HashMap`/`HashSet`, whose per-process random seed reached the iteration
//! order that builds the result, so the same fillet could come back with its
//! faces in a different order — and different arena bytes — in each process.
//! Inside one process every map shares a seed counter, so a single run cannot
//! see that; this test re-runs itself as worker processes and compares the
//! serialized arena of each.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, deprecated)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::fillet::fillet_rolling_ball;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_edges;
use remus_topology::solid::SolidId;

const WORKER_ENV: &str = "REMUS_FILLET_DETERMINISM_WORKER";
const WORKERS: usize = 4;

/// All 12 edges of a 20×10×40 box at r = 2.
fn box_all_edges(topo: &mut Topology) -> SolidId {
    let solid = make_box(topo, 20.0, 10.0, 40.0).unwrap();
    let edges = solid_edges(topo, solid).unwrap();
    assert_eq!(edges.len(), 12);
    fillet_rolling_ball(topo, solid, &edges, 2.0).unwrap()
}

/// A fused L-bracket: the outer convex edges of the wall top at r = 1.5,
/// mixing edges on the original and the boolean-built faces.
fn l_bracket(topo: &mut Topology) -> SolidId {
    let base = make_box(topo, 60.0, 40.0, 8.0).unwrap();
    let wall = make_box(topo, 60.0, 8.0, 32.0).unwrap();
    transform_solid(topo, wall, &Mat4::translation(0.0, 32.0, 7.5)).unwrap();
    let blank = boolean(topo, BooleanOp::Fuse, base, wall).unwrap();
    let top = 39.5;
    let edges: Vec<_> = solid_edges(topo, blank)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            (a.z() - top).abs() < 1e-6 && (b.z() - top).abs() < 1e-6
        })
        .collect();
    assert_eq!(edges.len(), 4, "wall top rim");
    fillet_rolling_ball(topo, blank, &edges, 1.5).unwrap()
}

/// The walking-blend (v2) engine on all 12 edges of the same box.
fn box_v2(topo: &mut Topology) -> SolidId {
    let solid = make_box(topo, 20.0, 10.0, 40.0).unwrap();
    let edges = solid_edges(topo, solid).unwrap();
    remus_operations::blend_ops::fillet_v2(topo, solid, &edges, 2.0)
        .unwrap()
        .solid
}

fn case_bytes(case: &str) -> Vec<u8> {
    let mut topo = Topology::new();
    let solid = match case {
        "box_all_edges" => box_all_edges(&mut topo),
        "l_bracket" => l_bracket(&mut topo),
        "box_v2" => box_v2(&mut topo),
        other => panic!("unknown case {other}"),
    };
    remus_io::arena_io::serialize_solid(&topo, solid).unwrap()
}

/// Worker entry: serializes the named case to the given file. A no-op unless
/// the parent test spawned this process.
#[test]
fn fillet_determinism_worker_entry() {
    let Ok(spec) = std::env::var(WORKER_ENV) else {
        return;
    };
    let (case, out) = spec.split_once('=').expect("case=path");
    std::fs::write(out, case_bytes(case)).unwrap();
}

fn run_workers(case: &str) -> Vec<Vec<u8>> {
    let exe = std::env::current_exe().unwrap();
    let dir = std::env::temp_dir();
    (0..WORKERS)
        .map(|i| {
            let out = dir.join(format!(
                "remus_fillet_det_{}_{case}_{i}.bin",
                std::process::id()
            ));
            let status = std::process::Command::new(&exe)
                .arg("fillet_determinism_worker_entry")
                .arg("--exact")
                .arg("--test-threads=1")
                .env(WORKER_ENV, format!("{case}={}", out.display()))
                .stdout(std::process::Stdio::null())
                .status()
                .unwrap();
            assert!(status.success(), "worker {i} for {case} failed");
            let bytes = std::fs::read(&out).unwrap();
            let _ = std::fs::remove_file(&out);
            bytes
        })
        .collect()
}

fn assert_cross_process_identical(case: &str) {
    let runs = run_workers(case);
    let local = case_bytes(case);
    for (i, bytes) in runs.iter().enumerate() {
        assert!(
            *bytes == local,
            "{case}: worker {i} serialized {} bytes that differ from this process's {} \
             — fillet output depends on per-process hash seeds",
            bytes.len(),
            local.len()
        );
    }
}

#[test]
fn box_all_edges_fillet_is_byte_identical_across_processes() {
    assert_cross_process_identical("box_all_edges");
}

#[test]
fn l_bracket_fillet_is_byte_identical_across_processes() {
    assert_cross_process_identical("l_bracket");
}

#[test]
fn box_v2_fillet_is_byte_identical_across_processes() {
    assert_cross_process_identical("box_v2");
}
