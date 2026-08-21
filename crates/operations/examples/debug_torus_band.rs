#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs
)]
//! Probe for the closed-torus band split (roadmap B1, ready-repro
//! `qualify_torus_boolean.rs::concentric_sphere_inclusion_exclusion`).
//!
//! Dumps both operands and then raw GFA *below* the operations acceptance
//! gate, so the structural failure is visible instead of the ops-level
//! "mesh boolean work limit exceeded" that the gate substitutes.
//!
//! What it shows, and why the split has no arm. The torus operand is ONE
//! face whose whole boundary is 2 distinct zero-length seam edges, each
//! traversed twice and all anchored at a single vertex at (12,0,0) — the
//! UV rectangle with both seams collapsed to a point. Two gaps follow, and
//! they are one contract:
//!
//! 1. `fill_images_faces::compute_seam_anchors` skips any surface that is
//!    not Cylinder/Cone, and `seam_anchor_on_circle` looks for a
//!    NON-degenerate seam Line — a torus has neither, so its section
//!    circles are never re-anchored to the face's seam.
//! 2. `face_splitter::special_cases::split_periodic_face_into_bands`
//!    rejects a torus on its first line, and its boundary precondition
//!    (exactly 2 closed circle edges plus seam Lines) is unsatisfiable for
//!    a doubly-periodic face that has no boundary circles at all.
//!
//! The anchor point IS the shared vertex between a band's seam segment and
//! the section edge, which is why the 2026-08-21 attempt at (2) alone
//! assembled free-edged. For torus R=10 rho=2 against a concentric sphere
//! R=10 the sections are exact circles at s=9.8, z=+/-1.98997 (tube angle
//! v ~ +/-95.74 deg) and the seam anchors are (9.8, 0, +/-1.98997).
//!
//! The partner side is compatible: on a hemisphere those circles genuinely
//! bound spherical caps, so the sphere keeps each as a 1-edge inner wire.
//! Opening a closed circle gives it a start/end vertex without splitting
//! the edge, so one edge can serve as both the sphere's hole wire and the
//! torus band's separator.

use remus_operations::primitives::{make_sphere, make_torus};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use std::collections::BTreeMap;

fn ecurve(topo: &Topology, e: remus_topology::edge::EdgeId) -> String {
    let ed = topo.edge(e).unwrap();
    let v0 = topo.vertex(ed.start()).unwrap().point();
    let v1 = topo.vertex(ed.end()).unwrap().point();
    let k = ed.curve().type_tag();
    format!(
        "{k} v{:?}->{:?} ({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3})",
        ed.start(),
        ed.end(),
        v0.x(),
        v0.y(),
        v0.z(),
        v1.x(),
        v1.y(),
        v1.z()
    )
}

/// Edge-use census by edge id: free (1 use) and over-shared (>2).
fn edge_census(topo: &Topology, s: SolidId) {
    let faces = solid_faces(topo, s).unwrap();
    let mut uses: BTreeMap<usize, usize> = BTreeMap::new();
    for &f in &faces {
        let fa = topo.face(f).unwrap();
        let mut wires = vec![fa.outer_wire()];
        wires.extend(fa.inner_wires().iter().copied());
        for w in wires {
            for oe in topo.wire(w).unwrap().edges() {
                *uses.entry(oe.edge().index()).or_default() += 1;
            }
        }
    }
    let free = uses.values().filter(|&&c| c == 1).count();
    let over = uses.values().filter(|&&c| c > 2).count();
    println!("    edge-use: total={} free={free} over={over}", uses.len());
}

fn kind(s: &FaceSurface) -> &'static str {
    s.type_tag()
}

fn dump(topo: &Topology, s: SolidId, label: &str) {
    let faces = solid_faces(topo, s).unwrap();
    let mut mix: std::collections::BTreeMap<&str, usize> = BTreeMap::new();
    for &f in &faces {
        *mix.entry(kind(topo.face(f).unwrap().surface()))
            .or_default() += 1;
    }
    let vol = remus_operations::measure::solid_volume(topo, s, 0.02).unwrap_or(f64::NAN);
    println!("--- {label}: F={} vol={vol:.2} mix={mix:?}", faces.len());
    edge_census(topo, s);
    for &f in &faces {
        let fa = topo.face(f).unwrap();
        let ow = topo.wire(fa.outer_wire()).unwrap();
        println!(
            "    {f:?} {} rev={} outer_edges={} inner_wires={}",
            kind(fa.surface()),
            fa.is_reversed(),
            ow.edges().len(),
            fa.inner_wires().len()
        );
        for oe in ow.edges() {
            println!("            outer: {}", ecurve(topo, oe.edge()));
        }
        for (i, &iw) in fa.inner_wires().iter().enumerate() {
            println!(
                "        inner[{i}] edges={}",
                topo.wire(iw).unwrap().edges().len()
            );
            for oe in topo.wire(iw).unwrap().edges() {
                println!("            inner: {}", ecurve(topo, oe.edge()));
            }
        }
    }
}

fn main() {
    let mut topo = Topology::new();
    let t = make_torus(&mut topo, 10.0, 2.0, 32).unwrap();
    let s = make_sphere(&mut topo, 10.0, 32).unwrap();

    dump(&topo, t, "TORUS operand");
    dump(&topo, s, "SPHERE operand");

    for op in [
        remus_algo::bop::BooleanOp::Fuse,
        remus_algo::bop::BooleanOp::Intersect,
        remus_algo::bop::BooleanOp::Cut,
    ] {
        let mut work = Topology::new();
        let t2 = make_torus(&mut work, 10.0, 2.0, 32).unwrap();
        let s2 = make_sphere(&mut work, 10.0, 32).unwrap();
        println!("\n=== RAW GFA {op:?} ===");
        match remus_algo::gfa::boolean(&mut work, op, t2, s2) {
            Ok(r) => dump(&work, r, &format!("raw {op:?}")),
            Err(e) => println!("    Err: {e}"),
        }
        let mut ow = Topology::new();
        let t3 = make_torus(&mut ow, 10.0, 2.0, 32).unwrap();
        let s3 = make_sphere(&mut ow, 10.0, 32).unwrap();
        let ops_op = match op {
            remus_algo::bop::BooleanOp::Fuse => remus_operations::boolean::BooleanOp::Fuse,
            remus_algo::bop::BooleanOp::Intersect => {
                remus_operations::boolean::BooleanOp::Intersect
            }
            remus_algo::bop::BooleanOp::Cut => remus_operations::boolean::BooleanOp::Cut,
        };
        match remus_operations::boolean::boolean(&mut ow, ops_op, t3, s3) {
            Ok(r) => dump(&ow, r, &format!("OPS {op:?}")),
            Err(e) => println!("    OPS Err: {e}"),
        }
    }
}
