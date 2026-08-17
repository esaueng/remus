//! Probe: fuse of two parallel overlapping cylinders (census cluster 4a).
//!
//! `cargo run --release --example debug_parallel_cyl -p remus-operations`
#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs
)]

use remus_check::classify::{ClassifyOptions, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::transform::transform_solid;
use remus_operations::{measure, primitives};
use remus_topology::Topology;

fn census(topo: &Topology, solid: remus_topology::solid::SolidId, tag: &str) {
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let mut counts: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    for &f in &faces {
        *counts
            .entry(topo.face(f).unwrap().surface().type_tag())
            .or_default() += 1;
    }
    // edge usage
    let mut usage: std::collections::HashMap<remus_topology::edge::EdgeId, usize> =
        std::collections::HashMap::new();
    for &f in &faces {
        let face = topo.face(f).unwrap();
        let wires: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();
        for w in wires {
            for oe in topo.wire(w).unwrap().edges() {
                *usage.entry(oe.edge()).or_default() += 1;
            }
        }
    }
    let free = usage.values().filter(|&&c| c == 1).count();
    let over = usage.values().filter(|&&c| c > 2).count();
    if std::env::var("FREE").is_ok() {
        let all = std::env::var("FREE").is_ok_and(|v| v == "all");
        for (&eid, &c) in &usage {
            if c == 2 && !all {
                continue;
            }
            if let Ok(e) = topo.edge(eid) {
                let s = topo.vertex(e.start()).unwrap().point();
                let t = topo.vertex(e.end()).unwrap().point();
                println!(
                    "  edge {eid:?} use={c} {} ({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3})",
                    e.curve().type_tag(),
                    s.x(),
                    s.y(),
                    s.z(),
                    t.x(),
                    t.y(),
                    t.z()
                );
            }
        }
    }
    let vol = measure::solid_volume(topo, solid, 0.05).unwrap_or(-1.0);
    println!(
        "{tag}: F={} mix={counts:?} free={free} over={over} vol={vol:.3}",
        faces.len()
    );
    if std::env::var("FACES").is_ok() {
        for &f in &faces {
            let face = topo.face(f).unwrap();
            let area = measure::face_area(topo, f, 0.05).unwrap_or(-1.0);
            println!(
                "  face {:?} {} reversed={} inner={} area={area:.2}",
                f,
                face.surface().type_tag(),
                face.is_reversed(),
                face.inner_wires().len()
            );
            let wires: Vec<_> = std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .collect();
            for (wi, w) in wires.iter().enumerate() {
                let ids: Vec<String> = topo
                    .wire(*w)
                    .unwrap()
                    .edges()
                    .iter()
                    .map(|oe| format!("{:?}{}", oe.edge(), if oe.is_forward() { "+" } else { "-" }))
                    .collect();
                println!("    wire[{wi}] {}", ids.join(" "));
            }
        }
    }
}

fn main() {
    env_logger::init();
    let offset = std::env::var("OFF")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8.0);
    let h2 = std::env::var("H2")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(24.0);
    let z2 = std::env::var("Z2")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    let raw = std::env::var("RAW").is_ok();

    let r1 = std::env::var("R1")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(6.0);
    let r2 = std::env::var("R2")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(6.0);
    let h1 = std::env::var("H1")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(24.0);
    let mut topo = Topology::new();
    let a = primitives::make_cylinder(&mut topo, r1, h1).unwrap();
    let b = primitives::make_cylinder(&mut topo, r2, h2).unwrap();
    let mat = Mat4::translation(offset, 0.0, z2);
    transform_solid(&mut topo, b, &mat).unwrap();
    census(&topo, a, "operand A");
    census(&topo, b, "operand B");

    let result = if raw {
        match remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Fuse, a, b) {
            Ok(r) => {
                println!("raw GFA ok");
                r
            }
            Err(e) => {
                println!("raw GFA ERR: {e}");
                return;
            }
        }
    } else {
        match boolean(&mut topo, BooleanOp::Fuse, a, b) {
            Ok(r) => r,
            Err(e) => {
                println!("ops ERR: {e}");
                return;
            }
        }
    };
    census(&topo, result, "result");

    // probe points: inside each operand
    let opts = ClassifyOptions::default();
    for (tag, p) in [
        ("A axis mid", Point3::new(0.0, 0.0, 12.0)),
        ("A left", Point3::new(-r1 * 0.6, 0.0, 2.4)),
        ("B axis mid", Point3::new(offset, 0.0, z2 + h2 * 0.5)),
        (
            "B right",
            Point3::new(r2.mul_add(0.6, offset), 0.0, z2 + h2 * 0.9),
        ),
        ("lens mid", Point3::new(offset * 0.5, 0.0, 12.0)),
    ] {
        let c = classify_point(&topo, result, p, &opts);
        println!("probe {tag} {p:?} -> {c:?}");
    }
}
