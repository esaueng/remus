//! PERF-I02 STEP import scaling probe.
//!
//! Imports a fixed STEP file into increasingly large pre-existing documents,
//! separating parsing, construction, validation and snapshot cost.
//!
//! Reports wall time plus logical sizes; "payload bytes" is a documented
//! estimator over public accessors (stack sizes plus NURBS heap), a proxy
//! for clone traffic, not allocator bytes. Peak RSS is the process VmHWM
//! high-water mark and includes setup; it is reported for shape, not
//! attributed to snapshots.
//!
//! Run: `cargo run -p remus-io --example step_import_scaling --release`

#![allow(
    clippy::print_stdout,
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs
)]

use std::time::Instant;

use remus_io::ImportLimits;
use remus_io::step::reader::{StepValidationOptions, read_step, read_step_with_validation};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_operations::primitives::make_box;
use remus_topology::Topology;

const TOL: f64 = 1e-7;
const REPS: usize = 11;

fn flat_nurbs_patch(n: usize, z: f64) -> NurbsSurface {
    let mut control = Vec::with_capacity(n);
    let mut weights = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(n);
        let mut wrow = Vec::with_capacity(n);
        for j in 0..n {
            row.push(Point3::new(i as f64, j as f64, z));
            wrow.push(1.0);
        }
        control.push(row);
        weights.push(wrow);
    }
    let knots = |m: usize| {
        let mut k = vec![0.0; 4];
        for i in 1..m - 3 {
            k.push(i as f64);
        }
        k.extend(vec![(m - 3) as f64; 4]);
        k
    };
    NurbsSurface::new(3, 3, knots(n), knots(n), control, weights).unwrap()
}

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

fn build_template(boxes: usize, nurbs_faces: usize, nurbs_n: usize) -> Topology {
    let mut topo = Topology::new();
    for i in 0..boxes {
        make_box(&mut topo, 1.0 + i as f64 * 0.01, 1.0, 1.0).unwrap();
    }
    for i in 0..nurbs_faces {
        remus_topology::builder::make_nurbs_face(
            &mut topo,
            flat_nurbs_patch(nurbs_n, 100.0 + i as f64),
            TOL,
        )
        .unwrap();
    }
    topo
}

fn box_step() -> String {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    remus_io::step::write_step(&topo, &[solid]).unwrap()
}

fn cell(name: &str, template: &Topology, retain_checkpoint: bool, workload: &str, step: &str) {
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
                let solids = read_step(step, &mut topo).expect("import");
                assert_eq!(solids.len(), 1);
                slots_growth = topo.allocated_slot_count() - slots;
            }
            "import_box_validated" => {
                let result = read_step_with_validation(
                    step,
                    &mut topo,
                    ImportLimits::default(),
                    StepValidationOptions::default(),
                )
                .expect("validated import");
                assert_eq!(result.solids().len(), 1);
                slots_growth = topo.allocated_slot_count() - slots;
            }
            "parse_only" => {
                // No MANIFOLD_SOLID_BREP roots: scan + index + unit resolution
                // only, without topology construction. Should be flat across
                // document sizes (no snapshot, no construction).
                let repeated = concat!(
                    "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('R'),'1');\n",
                    "FILE_NAME('R','',(),(), '', '', '');\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n",
                    "#1=GLOBAL_UNIT_ASSIGNED_CONTEXT((#2,#3));\n",
                    "#2=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n",
                    "#3=(PLANE_ANGLE_UNIT()NAMED_UNIT(*)SI_UNIT($,.RADIAN.));\n",
                    "ENDSEC;\nEND-ISO-10303-21;"
                );
                let solids = read_step(repeated, &mut topo).expect("parse");
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
    println!("{{\"harness\":\"step_import_scaling\"}}");
    let only: Vec<String> = std::env::args().skip(1).collect();
    let step = box_step();
    let docs = [
        (1usize, 0usize, 0usize, "analytic"),
        (100, 0, 0, "analytic"),
        (1000, 0, 0, "analytic"),
    ];
    let workloads = [
        "clone_only",
        "parse_only",
        "import_box",
        "import_box_validated",
    ];
    for &(boxes, nurbs_faces, nurbs_n, tag) in &docs {
        let template = build_template(boxes, nurbs_faces, nurbs_n);
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
                cell(&cell_name, &template, cp, workload, &step);
            }
        }
    }
    let template = build_template(100, 24, 48);
    for &cp in &[false, true] {
        let cp_tag = if cp { "cp1" } else { "cp0" };
        for workload in ["clone_only", "import_box", "import_box_validated"] {
            let cell_name = format!("nurbs-boxes100+n24x48-{cp_tag}");
            if !only.is_empty()
                && !only
                    .iter()
                    .any(|f| cell_name.contains(f) | workload.contains(f))
            {
                continue;
            }
            cell(&cell_name, &template, cp, workload, &step);
        }
    }
}
