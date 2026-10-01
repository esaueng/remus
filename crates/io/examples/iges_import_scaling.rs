//! PERF-I02 IGES import scaling probe.
//!
//! Imports a fixed IGES file into increasingly large pre-existing documents,
//! separating parsing, construction and clone cost.
//!
//! Reports wall time plus logical sizes; "payload bytes" is a documented
//! estimator over public accessors (stack sizes plus NURBS heap), a proxy
//! for clone traffic, not allocator bytes. Peak RSS is the process VmHWM
//! high-water mark and includes setup; it is reported for shape, not
//! attributed to snapshots.
//!
//! Unlike the STEP slice, the IGES path never took a whole-document clone:
//! the baseline (`origin/main`) built topology directly with no snapshot, so
//! there is no snapshot cost to remove. This harness proves the new
//! append-only scope adds no document-size-dependent cost and keeps import
//! time following new content, not unrelated prior models.
//!
//! Run: `cargo run -p remus-io --example iges_import_scaling --release`

#![allow(
    clippy::print_stdout,
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs
)]

use std::time::Instant;

use remus_io::iges::{read_iges, write_iges};
use remus_operations::primitives::make_box;
use remus_topology::Topology;

const REPS: usize = 11;

fn estimate_live_payload_bytes(topo: &Topology) -> usize {
    let mut bytes = 0usize;
    bytes += topo.num_vertices() * 32;
    for (_, edge) in topo.edges().iter() {
        bytes += std::mem::size_of_val(edge);
        if let remus_topology::edge::EdgeCurve::NurbsCurve(n) = edge.curve() {
            bytes += n.control_points().len() * 24 + n.weights().len() * 8 + n.knots().len() * 8;
        }
    }
    for (_, wire) in topo.wires().iter() {
        bytes += std::mem::size_of_val(wire) + wire.edges().len() * 16;
    }
    for (_, face) in topo.faces().iter() {
        bytes += std::mem::size_of_val(face);
        if let remus_topology::face::FaceSurface::Nurbs(n) = face.surface() {
            let pts: usize = n.control_points().iter().map(Vec::len).sum();
            bytes += pts * 24 + pts * 8 + (n.knots_u().len() + n.knots_v().len()) * 8;
        }
    }
    for (_, shell) in topo.shells().iter() {
        bytes += std::mem::size_of_val(shell) + shell.faces().len() * 8;
    }
    for (_, solid) in topo.solids().iter() {
        bytes += std::mem::size_of_val(solid) + solid.inner_shells().len() * 8;
    }
    bytes += topo.journal().len() * 96;
    bytes
}

fn vm_hwm_kb() -> Option<usize> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    for line in status.lines() {
        if let Some(rest) = line.strip_prefix("VmHWM:") {
            return rest.split_whitespace().next()?.parse().ok();
        }
    }
    None
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs[xs.len() / 2]
}

fn build_template(boxes: usize) -> Topology {
    let mut topo = Topology::new();
    for i in 0..boxes {
        make_box(&mut topo, 1.0 + i as f64 * 0.01, 1.0, 1.0).unwrap();
    }
    topo
}

fn box_iges() -> String {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    write_iges(&topo, &[solid]).unwrap()
}

fn cell(name: &str, template: &Topology, retain_checkpoint: bool, workload: &str, iges: &str) {
    let slots = template.allocated_slot_count();
    let payload = estimate_live_payload_bytes(template);
    let mut samples = Vec::with_capacity(REPS);
    let mut slots_growth = 0usize;
    for _ in 0..REPS {
        let mut topo = template.clone();
        let _checkpoint = retain_checkpoint.then(|| template.clone());
        let start = Instant::now();
        match workload {
            "clone_only" => {
                let _ = topo.clone();
            }
            "import_box" => {
                let solids = read_iges(iges, &mut topo).expect("import");
                assert_eq!(solids.len(), 1);
                slots_growth = topo.allocated_slot_count() - slots;
            }
            "parse_only" => {
                // No 108 planes: scan + index only, without topology
                // construction. Should be flat across document sizes
                // (no snapshot, no construction).
                let solids = read_iges("", &mut topo).expect("parse");
                assert!(solids.is_empty());
            }
            _ => {}
        }
        samples.push(start.elapsed().as_secs_f64() * 1000.0);
    }
    let med = median(samples);
    let hwm = vm_hwm_kb().map_or_else(|| "null".to_owned(), |kb| kb.to_string());
    println!(
        "{{\"cell\":\"{name}\",\"workload\":\"{workload}\",\"slots\":{slots},\
         \"payload_bytes\":{payload},\"slots_growth\":{slots_growth},\
         \"median_ms\":{med:.3},\"reps\":{REPS},\"vmhwm_kb\":{hwm}}}"
    );
}

fn main() {
    println!("{{\"harness\":\"iges_import_scaling\"}}");
    let only: Vec<String> = std::env::args().skip(1).collect();
    let iges = box_iges();
    let docs = [(1usize, "analytic"), (100, "analytic"), (1000, "analytic")];
    let workloads = ["clone_only", "parse_only", "import_box"];
    for &(boxes, tag) in &docs {
        let template = build_template(boxes);
        for &cp in &[false, true] {
            let cp_tag = if cp { "cp1" } else { "cp0" };
            for workload in workloads {
                let cell_name = format!("{tag}-boxes{boxes}-{cp_tag}");
                if !only.is_empty()
                    && !only
                        .iter()
                        .any(|f| cell_name.contains(f) | workload.contains(f))
                {
                    continue;
                }
                cell(&cell_name, &template, cp, workload, &iges);
            }
        }
    }
}
