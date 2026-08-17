//! Probe: chamfer the REAL flange rim, not a bare cylinder.
//!
//! The bare-cylinder repro has a disc cap. The flange's caps are ANNULI with
//! bolt holes, which is a different case for the rim assembler.
//!
//! Run: `cargo run --release --example debug_flange_chamfer -p remus-operations`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    clippy::print_stderr
)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::primitives;
use remus_operations::revolve::revolve;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

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
        let before = remus_operations::measure::solid_volume(&t2, b2, 0.05).unwrap();
        match remus_operations::blend_ops::chamfer_v2(&mut t2, b2, &picks, 1.5, 1.5) {
            Ok(r) => {
                let after = remus_operations::measure::solid_volume(&t2, r.solid, 0.05).unwrap();
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

    // Simplest holed cap: a washer (annulus, ONE inner wire, no bolt holes).
    {
        let mut tw = Topology::new();
        let washer = revolved_annulus(&mut tw, 12.0, 24.0, 0.0, 26.0);
        let mut tgt = None;
        let mut seen: Vec<EdgeId> = Vec::new();
        for fid in solid_faces(&tw, washer).unwrap() {
            let f = tw.face(fid).unwrap();
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in tw.wire(wid).unwrap().edges() {
                    if seen.contains(&oe.edge()) {
                        continue;
                    }
                    seen.push(oe.edge());
                    let e = tw.edge(oe.edge()).unwrap();
                    if e.start() != e.end() {
                        continue;
                    }
                    let p = tw.vertex(e.start()).unwrap().point();
                    if (p.x().hypot(p.y()) - 24.0).abs() < 1e-6 && (p.z() - 26.0).abs() < 1e-6 {
                        tgt = Some(oe.edge());
                    }
                }
            }
        }
        if let Some(g) = tgt {
            let r = 1.5_f64;
            let before = remus_operations::measure::solid_volume(&tw, washer, 0.002).unwrap();
            match remus_operations::blend_ops::fillet_v2(&mut tw, washer, &[g], r) {
                Ok(res) => {
                    let after =
                        remus_operations::measure::solid_volume(&tw, res.solid, 0.002).unwrap();
                    let area = r * r * (1.0 - std::f64::consts::PI / 4.0);
                    let num =
                        (24.0 - r / 2.0) - (std::f64::consts::PI / 4.0) * (24.0 - r) - r / 3.0;
                    let cen = num / (1.0 - std::f64::consts::PI / 4.0);
                    let expect = area * std::f64::consts::TAU * cen;
                    println!(
                        "  WASHER rim: removed {:.4} vs {expect:.4} (err {:.2e})",
                        before - after,
                        ((before - after) - expect).abs() / expect
                    );
                }
                Err(e) => println!("  WASHER rim: ERR {e}"),
            }
            // Dump the resulting faces so the geometry can be read directly.
            let mut tw2 = Topology::new();
            let w2 = revolved_annulus(&mut tw2, 12.0, 24.0, 0.0, 26.0);
            let mut g2 = None;
            let mut sn: Vec<EdgeId> = Vec::new();
            for fid in solid_faces(&tw2, w2).unwrap() {
                let f = tw2.face(fid).unwrap();
                for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                    for oe in tw2.wire(wid).unwrap().edges() {
                        if sn.contains(&oe.edge()) {
                            continue;
                        }
                        sn.push(oe.edge());
                        let e = tw2.edge(oe.edge()).unwrap();
                        if e.start() != e.end() {
                            continue;
                        }
                        let p = tw2.vertex(e.start()).unwrap().point();
                        if (p.x().hypot(p.y()) - 24.0).abs() < 1e-6 && (p.z() - 26.0).abs() < 1e-6 {
                            g2 = Some(oe.edge());
                        }
                    }
                }
            }
            if let Ok(res) =
                remus_operations::blend_ops::fillet_v2(&mut tw2, w2, &[g2.unwrap()], 1.5)
            {
                println!("  --- washer after fillet ---");
                for fid in solid_faces(&tw2, res.solid).unwrap() {
                    let f = tw2.face(fid).unwrap();
                    let extra = match f.surface() {
                        remus_topology::face::FaceSurface::Torus(t) => format!(
                            " major={:.3} minor={:.3} c=({:.2},{:.2},{:.2})",
                            t.major_radius(),
                            t.minor_radius(),
                            t.center().x(),
                            t.center().y(),
                            t.center().z()
                        ),
                        remus_topology::face::FaceSurface::Cylinder(c) => {
                            format!(" r={:.3}", c.radius())
                        }
                        _ => String::new(),
                    };
                    let mut ring = Vec::new();
                    for wid in
                        std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied())
                    {
                        for oe in tw2.wire(wid).unwrap().edges() {
                            let e = tw2.edge(oe.edge()).unwrap();
                            let p = tw2.vertex(e.start()).unwrap().point();
                            ring.push(format!("({:.2}@z{:.2})", p.x().hypot(p.y()), p.z()));
                        }
                    }
                    println!(
                        "    {}{} rev={} inner={} [{}]",
                        f.surface().type_tag(),
                        extra,
                        f.is_reversed(),
                        f.inner_wires().len(),
                        ring.join(" ")
                    );
                }
            }
        }
    }

    // Per-rim fillet volume vs closed form — isolate which rim is off.
    for (want_r, want_z) in [(45.0_f64, 10.0_f64), (45.0, 0.0), (24.0, 26.0)] {
        let mut t2 = Topology::new();
        let b2 = drilled_flange(&mut t2);
        let mut tgt = None;
        let mut seen: Vec<EdgeId> = Vec::new();
        for fid in solid_faces(&t2, b2).unwrap() {
            let f = t2.face(fid).unwrap();
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in t2.wire(wid).unwrap().edges() {
                    if seen.contains(&oe.edge()) {
                        continue;
                    }
                    seen.push(oe.edge());
                    let e = t2.edge(oe.edge()).unwrap();
                    if e.start() != e.end() {
                        continue;
                    }
                    let p = t2.vertex(e.start()).unwrap().point();
                    if (p.x().hypot(p.y()) - want_r).abs() < 1e-6 && (p.z() - want_z).abs() < 1e-6 {
                        tgt = Some(oe.edge());
                    }
                }
            }
        }
        let Some(g) = tgt else { continue };
        let r = 1.5_f64;
        let before = remus_operations::measure::solid_volume(&t2, b2, 0.002).unwrap();
        if let Ok(res) = remus_operations::blend_ops::fillet_v2(&mut t2, b2, &[g], r) {
            let after = remus_operations::measure::solid_volume(&t2, res.solid, 0.002).unwrap();
            let area = r * r * (1.0 - std::f64::consts::PI / 4.0);
            let num = (want_r - r / 2.0) - (std::f64::consts::PI / 4.0) * (want_r - r) - r / 3.0;
            let cen = num / (1.0 - std::f64::consts::PI / 4.0);
            let expect = area * std::f64::consts::TAU * cen;
            println!(
                "  rim r={want_r} z={want_z}: removed {:.3} vs {expect:.3} (err {:.2e})",
                before - after,
                ((before - after) - expect).abs() / expect
            );
        }
    }

    // Fillet convergence + the reaching-a-hole case.
    {
        let mut tf = Topology::new();
        let bf = drilled_flange(&mut tf);
        let rims: Vec<EdgeId> = {
            let mut v = Vec::new();
            let mut seen: Vec<EdgeId> = Vec::new();
            for fid in solid_faces(&tf, bf).unwrap() {
                let f = tf.face(fid).unwrap();
                for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                    for oe in tf.wire(wid).unwrap().edges() {
                        if seen.contains(&oe.edge()) {
                            continue;
                        }
                        seen.push(oe.edge());
                        let e = tf.edge(oe.edge()).unwrap();
                        if e.start() != e.end() {
                            continue;
                        }
                        let p = tf.vertex(e.start()).unwrap().point();
                        let r = p.x().hypot(p.y());
                        if (r - 45.0).abs() < 1e-6 || ((r - 24.0).abs() < 1e-6 && p.z() >= 25.5) {
                            v.push(oe.edge());
                        }
                    }
                }
            }
            v
        };
        for defl in [0.05, 0.01, 0.002] {
            let mut t2 = Topology::new();
            let b2 = drilled_flange(&mut t2);
            let rr: Vec<EdgeId> = {
                let mut v = Vec::new();
                let mut seen: Vec<EdgeId> = Vec::new();
                for fid in solid_faces(&t2, b2).unwrap() {
                    let f = t2.face(fid).unwrap();
                    for wid in
                        std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied())
                    {
                        for oe in t2.wire(wid).unwrap().edges() {
                            if seen.contains(&oe.edge()) {
                                continue;
                            }
                            seen.push(oe.edge());
                            let e = t2.edge(oe.edge()).unwrap();
                            if e.start() != e.end() {
                                continue;
                            }
                            let p = t2.vertex(e.start()).unwrap().point();
                            let r = p.x().hypot(p.y());
                            if (r - 45.0).abs() < 1e-6 || ((r - 24.0).abs() < 1e-6 && p.z() >= 25.5)
                            {
                                v.push(oe.edge());
                            }
                        }
                    }
                }
                v
            };
            let before = remus_operations::measure::solid_volume(&t2, b2, defl).unwrap();
            if let Ok(res) = remus_operations::blend_ops::fillet_v2(&mut t2, b2, &rr, 1.5) {
                let after = remus_operations::measure::solid_volume(&t2, res.solid, defl).unwrap();
                println!(
                    "  fillet defl={defl}: removed {:.3} (analytic 342.81)",
                    before - after
                );
            }
        }
        // Radius that reaches the bolt circle.
        let top = rims
            .iter()
            .copied()
            .find(|&e| {
                let p = tf.vertex(tf.edge(e).unwrap().start()).unwrap().point();
                (p.x().hypot(p.y()) - 45.0).abs() < 1e-6 && (p.z() - 10.0).abs() < 1e-6
            })
            .unwrap();
        let mut t3 = Topology::new();
        let b3 = drilled_flange(&mut t3);
        let rr: Vec<EdgeId> = solid_faces(&t3, b3)
            .unwrap()
            .into_iter()
            .flat_map(|fid| {
                let f = t3.face(fid).unwrap();
                let mut v = Vec::new();
                for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                    for oe in t3.wire(wid).unwrap().edges() {
                        let e = t3.edge(oe.edge()).unwrap();
                        if e.start() != e.end() {
                            continue;
                        }
                        let p = t3.vertex(e.start()).unwrap().point();
                        if (p.x().hypot(p.y()) - 45.0).abs() < 1e-6 && (p.z() - 10.0).abs() < 1e-6 {
                            v.push(oe.edge());
                        }
                    }
                }
                v
            })
            .collect();
        let _ = top;
        let vb = remus_operations::measure::solid_volume(&t3, b3, 0.05).unwrap();
        match remus_operations::blend_ops::fillet_v2(&mut t3, b3, &rr[..1], 10.0) {
            Ok(res) => {
                let va = remus_operations::measure::solid_volume(&t3, res.solid, 0.05).unwrap();
                let mut cen = std::collections::BTreeMap::new();
                for fid in solid_faces(&t3, res.solid).unwrap() {
                    *cen.entry(t3.face(fid).unwrap().surface().type_tag())
                        .or_insert(0) += 1;
                }
                println!("  fillet r=10 (reaches bolts): OK vol {vb:.1} -> {va:.1} {cen:?}");
            }
            Err(e) => println!("  fillet r=10 (reaches bolts): ERR {e}"),
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
        match remus_operations::blend_ops::chamfer_v2(&mut t2, b2, &[tg], 1.5, 1.5) {
            Ok(r) => println!("  {label}: chamfer OK failed={}", r.failed.len()),
            Err(err) => println!("  {label}: chamfer ERR {err}"),
        }
        // Same rim, FILLET — does it hit the bare-disc gate too?
        let mut t3 = Topology::new();
        let b3 = drilled_flange(&mut t3);
        let mut tgt3 = None;
        let mut seen3: Vec<EdgeId> = Vec::new();
        for fid in solid_faces(&t3, b3).unwrap() {
            let f = t3.face(fid).unwrap();
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in t3.wire(wid).unwrap().edges() {
                    if seen3.contains(&oe.edge()) {
                        continue;
                    }
                    seen3.push(oe.edge());
                    let e3 = t3.edge(oe.edge()).unwrap();
                    let p = t3.vertex(e3.start()).unwrap().point();
                    if e3.start() == e3.end()
                        && (p.x().hypot(p.y()) - a.x().hypot(a.y())).abs() < 1e-6
                        && (p.z() - a.z()).abs() < 1e-6
                    {
                        tgt3 = Some(oe.edge());
                    }
                }
            }
        }
        if let Some(g3) = tgt3 {
            match remus_operations::blend_ops::fillet_v2(&mut t3, b3, &[g3], 1.5) {
                Ok(r) => println!("  {label}: fillet  OK failed={}", r.failed.len()),
                Err(err) => println!("  {label}: fillet  ERR {err}"),
            }
        }
    }
}
