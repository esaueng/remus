//! Probe: chamfer the REAL flange rim, not a bare cylinder.
//!
//! The bare-cylinder repro has a disc cap. The flange's caps are ANNULI with
//! bolt holes, which is a different case for the rim assembler.
//!
//! Run: `cargo run --release --example debug_flange_chamfer -p brepkit-operations`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    clippy::print_stderr
)]

use brepkit_math::mat::Mat4;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::heal::unify_faces;
use brepkit_operations::primitives;
use brepkit_operations::revolve::revolve;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use brepkit_topology::edge::EdgeId;
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::solid::SolidId;

const TOL: f64 = 1e-7;

fn revolved_annulus(t: &mut Topology, ri: f64, ro: f64, z0: f64, z1: f64) -> SolidId {
    let pts = [
        Point3::new(ri, 0.0, z0),
        Point3::new(ro, 0.0, z0),
        Point3::new(ro, 0.0, z1),
        Point3::new(ri, 0.0, z1),
    ];
    let w = make_polygon_wire(t, &pts, TOL).unwrap();
    let f = make_planar_face_from_wire(t, w).unwrap();
    revolve(
        t,
        f,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
    )
    .unwrap()
}

fn drilled_flange(t: &mut Topology) -> SolidId {
    let rim = revolved_annulus(t, 24.0, 45.0, 0.0, 10.0);
    let hub = revolved_annulus(t, 12.0, 24.0, 0.0, 26.0);
    let blank = boolean(t, BooleanOp::Fuse, rim, hub).expect("fuse");
    unify_faces(t, blank).unwrap();

    let mut pattern = None;
    for i in 0..6 {
        let a = std::f64::consts::TAU * f64::from(i) / 6.0;
        let c = primitives::make_cylinder(t, 3.0, 16.0).unwrap();
        transform_solid(
            t,
            c,
            &Mat4::translation(34.0 * a.cos(), 34.0 * a.sin(), -3.0),
        )
        .unwrap();
        pattern = Some(match pattern {
            None => c,
            Some(p) => boolean(t, BooleanOp::Fuse, p, c).expect("pattern fuse"),
        });
    }
    boolean(t, BooleanOp::Cut, blank, pattern.unwrap()).expect("drill")
}

/// Describe the cap face on the other side of a closed rim edge.
fn describe_neighbours(t: &Topology, s: SolidId, e: EdgeId) -> String {
    let mut out = Vec::new();
    for fid in solid_faces(t, s).unwrap() {
        let f = t.face(fid).unwrap();
        let uses = std::iter::once(f.outer_wire())
            .chain(f.inner_wires().iter().copied())
            .any(|w| t.wire(w).unwrap().edges().iter().any(|oe| oe.edge() == e));
        if uses {
            let outer_len = t.wire(f.outer_wire()).unwrap().edges().len();
            out.push(format!(
                "{}(outer_edges={outer_len}, inner_wires={})",
                f.surface().type_tag(),
                f.inner_wires().len()
            ));
        }
    }
    out.join(" + ")
}

fn main() {
    env_logger::init();

    let mut t = Topology::new();
    let body = drilled_flange(&mut t);

    // The demo picks edges at radius 45, plus the r24 hub lip at z >= 25.5,
    // constrained to edges flat in Z (OpenZCAD #34).
    let mut picked: Vec<EdgeId> = Vec::new();
    let mut seen: Vec<EdgeId> = Vec::new();
    for fid in solid_faces(&t, body).unwrap() {
        let f = t.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in t.wire(wid).unwrap().edges() {
                if seen.contains(&oe.edge()) {
                    continue;
                }
                seen.push(oe.edge());
                let ed = t.edge(oe.edge()).unwrap();
                let a = t.vertex(ed.start()).unwrap().point();
                let r = a.x().hypot(a.y());
                let closed = ed.start() == ed.end();
                if closed && ((r - 45.0).abs() < 1e-6 || ((r - 24.0).abs() < 1e-6 && a.z() >= 25.5))
                {
                    picked.push(oe.edge());
                }
            }
        }
    }

    println!("picked {} closed rim edges", picked.len());
    for &e in &picked {
        let ed = t.edge(e).unwrap();
        let a = t.vertex(ed.start()).unwrap().point();
        println!(
            "  r={:.1} z={:.1}  neighbours: {}",
            a.x().hypot(a.y()),
            a.z(),
            describe_neighbours(&t, body, e)
        );
    }

    // All three at once — what the demo actually does.
    {
        let mut t2 = Topology::new();
        let b2 = drilled_flange(&mut t2);
        let mut picks: Vec<EdgeId> = Vec::new();
        let mut seen2: Vec<EdgeId> = Vec::new();
        for fid in solid_faces(&t2, b2).unwrap() {
            let f = t2.face(fid).unwrap();
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in t2.wire(wid).unwrap().edges() {
                    if seen2.contains(&oe.edge()) {
                        continue;
                    }
                    seen2.push(oe.edge());
                    let e2 = t2.edge(oe.edge()).unwrap();
                    let p = t2.vertex(e2.start()).unwrap().point();
                    let r = p.x().hypot(p.y());
                    if e2.start() == e2.end()
                        && ((r - 45.0).abs() < 1e-6 || ((r - 24.0).abs() < 1e-6 && p.z() >= 25.5))
                    {
                        picks.push(oe.edge());
                    }
                }
            }
        }
        let before = brepkit_operations::measure::solid_volume(&t2, b2, 0.05).unwrap();
        match brepkit_operations::blend_ops::chamfer_v2(&mut t2, b2, &picks, 1.5, 1.5) {
            Ok(r) => {
                let after = brepkit_operations::measure::solid_volume(&t2, r.solid, 0.05).unwrap();
                let mut census = std::collections::BTreeMap::new();
                for fid in solid_faces(&t2, r.solid).unwrap() {
                    *census
                        .entry(t2.face(fid).unwrap().surface().type_tag())
                        .or_insert(0) += 1;
                }
                // Pappus per rim: triangle area d^2/2 revolved at centroid radius.
                let d = 1.5_f64;
                let wedge = |rr: f64, sign: f64| {
                    0.5 * d * d * std::f64::consts::TAU * (rr + sign * d / 3.0)
                };
                // r45 rims cut inward (centroid 45 - d/3), the r24 hub lip too.
                let expect = before - 2.0 * wedge(45.0, -1.0) - wedge(24.0, -1.0);
                let mut usage: std::collections::BTreeMap<usize, usize> =
                    std::collections::BTreeMap::new();
                for fid in solid_faces(&t2, r.solid).unwrap() {
                    let f = t2.face(fid).unwrap();
                    for wid in
                        std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied())
                    {
                        for oe in t2.wire(wid).unwrap().edges() {
                            *usage.entry(oe.edge().index()).or_insert(0) += 1;
                        }
                    }
                }
                let free = usage.values().filter(|&&c| c == 1).count();
                let nm = usage.values().filter(|&&c| c >= 3).count();
                println!(
                    "ALL THREE: OK failed={} vol {after:.2} vs {expect:.2} (err {:.2e}) brep free={free} nm={nm} {census:?}",
                    r.failed.len(),
                    ((after - expect) / expect).abs()
                );
            }
            Err(err) => println!("ALL THREE: ERR {err}"),
        }
    }

    // One at a time, so a single failure does not hide the others.
    for &e in &picked {
        let ed = t.edge(e).unwrap();
        let a = t.vertex(ed.start()).unwrap().point();
        let label = format!("r={:.1} z={:.1}", a.x().hypot(a.y()), a.z());
        let mut t2 = Topology::new();
        let b2 = drilled_flange(&mut t2);
        // Re-find the same edge in the fresh topology by geometry.
        let mut target = None;
        let mut seen2: Vec<EdgeId> = Vec::new();
        for fid in solid_faces(&t2, b2).unwrap() {
            let f = t2.face(fid).unwrap();
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in t2.wire(wid).unwrap().edges() {
                    if seen2.contains(&oe.edge()) {
                        continue;
                    }
                    seen2.push(oe.edge());
                    let e2 = t2.edge(oe.edge()).unwrap();
                    let p = t2.vertex(e2.start()).unwrap().point();
                    if e2.start() == e2.end()
                        && (p.x().hypot(p.y()) - a.x().hypot(a.y())).abs() < 1e-6
                        && (p.z() - a.z()).abs() < 1e-6
                    {
                        target = Some(oe.edge());
                    }
                }
            }
        }
        let Some(tg) = target else {
            println!("  {label}: could not re-find edge");
            continue;
        };
        match brepkit_operations::blend_ops::chamfer_v2(&mut t2, b2, &[tg], 1.5, 1.5) {
            Ok(r) => println!("  {label}: chamfer OK failed={}", r.failed.len()),
            Err(err) => println!("  {label}: chamfer ERR {err}"),
        }
    }
}
