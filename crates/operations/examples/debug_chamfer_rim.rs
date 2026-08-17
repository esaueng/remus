//! Repro: chamfering a CLOSED circular edge (a cylinder rim).
//!
//! Run: `cargo run --release --example debug_chamfer_rim -p remus-operations`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    clippy::print_stderr
)]

use remus_operations::primitives;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

fn surface_census(topo: &Topology, s: SolidId) -> std::collections::BTreeMap<&'static str, usize> {
    let mut m = std::collections::BTreeMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        *m.entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    m
}

fn mesh_edges(topo: &Topology, s: SolidId) -> (usize, usize) {
    let Ok(mesh) =
        remus_operations::tessellate::tessellate_solid_with_tolerance(topo, s, 0.01, 0.1)
    else {
        return (usize::MAX, usize::MAX);
    };
    let q = 1e6;
    let mut canon: std::collections::BTreeMap<(i64, i64, i64), u32> =
        std::collections::BTreeMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    let mut edges: std::collections::BTreeMap<(u32, u32), u32> = std::collections::BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c >= 3).count(),
    )
}

fn counts(topo: &Topology, s: SolidId) -> (usize, usize, usize) {
    remus_topology::explorer::solid_entity_counts(topo, s).unwrap()
}

fn solid_edges(topo: &Topology, s: SolidId) -> Vec<remus_topology::edge::EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                if !seen.contains(&oe.edge()) {
                    seen.push(oe.edge());
                }
            }
        }
    }
    seen
}

fn main() {
    env_logger::init();

    // BARE FILLET CHECK: cylinder rim fillet against the exact closed form.
    for r in [0.5_f64, 1.5, 3.0] {
        let mut t = Topology::new();
        let c = primitives::make_cylinder(&mut t, 45.0, 10.0).unwrap();
        let es = solid_edges(&t, c);
        let rim = es
            .iter()
            .copied()
            .find(|&e| {
                let ed = t.edge(e).unwrap();
                ed.start() == ed.end()
            })
            .unwrap();
        let before = remus_operations::measure::solid_volume(&t, c, 0.002).unwrap();
        if let Ok(res) = remus_operations::blend_ops::fillet_v2(&mut t, c, &[rim], r) {
            let after = remus_operations::measure::solid_volume(&t, res.solid, 0.002).unwrap();
            let big = 45.0_f64;
            let area = r * r * (1.0 - std::f64::consts::PI / 4.0);
            let num = (big - r / 2.0) - (std::f64::consts::PI / 4.0) * (big - r) - r / 3.0;
            let cen = num / (1.0 - std::f64::consts::PI / 4.0);
            let want = area * std::f64::consts::TAU * cen;
            println!(
                "BARE FILLET r={r}: removed {:.4} vs closed form {want:.4} (err {:.2e})",
                before - after,
                ((before - after) - want).abs() / want
            );
        }
    }

    for dist in [1.5_f64, 0.5] {
        let mut topo = Topology::new();
        let cyl = primitives::make_cylinder(&mut topo, 45.0, 10.0).unwrap();
        let edges = solid_edges(&topo, cyl);
        println!("\n=== cylinder r45 h10, distance {dist} ===");
        println!("  {} edges, counts {:?}", edges.len(), counts(&topo, cyl));
        for (i, &e) in edges.iter().enumerate() {
            let ed = topo.edge(e).unwrap();
            let a = topo.vertex(ed.start()).unwrap().point();
            let b = topo.vertex(ed.end()).unwrap().point();
            let closed = (a - b).length() < 1e-9;
            println!(
                "  edge[{i}] {:>7} closed={closed} ({:.1},{:.1},{:.1})",
                ed.curve().type_tag(),
                a.x(),
                a.y(),
                a.z()
            );
        }

        for (i, &e) in edges.iter().enumerate() {
            // v1 flat-bevel chamfer (the `chamfer` binding)
            let mut t = Topology::new();
            let c = primitives::make_cylinder(&mut t, 45.0, 10.0).unwrap();
            let es = solid_edges(&t, c);
            let before = counts(&t, c);
            match remus_operations::chamfer::chamfer(&mut t, c, &[es[i]], dist) {
                Ok(r) => {
                    let after = counts(&t, r);
                    println!(
                        "  v1 chamfer edge[{i}]: OK {before:?} -> {after:?}{}",
                        if before == after { "  <-- NO-OP" } else { "" }
                    );
                }
                Err(err) => println!("  v1 chamfer edge[{i}]: ERR {err}"),
            }

            // v2 walking chamfer (the `chamferV2` binding)
            let mut t = Topology::new();
            let c = primitives::make_cylinder(&mut t, 45.0, 10.0).unwrap();
            let es = solid_edges(&t, c);
            let before = counts(&t, c);
            match remus_operations::blend_ops::chamfer_v2(&mut t, c, &[es[i]], dist, dist) {
                Ok(r) => {
                    let after = counts(&t, r.solid);
                    let vol = remus_operations::measure::solid_volume(&t, r.solid, 0.02)
                        .unwrap_or(f64::NAN);
                    // Pappus: revolving the right triangle (legs d,d) at
                    // centroid radius 45 - d/3 removes this much material.
                    let full = std::f64::consts::PI * 45.0 * 45.0 * 10.0;
                    let removed = 0.5 * dist * dist * std::f64::consts::TAU * (45.0 - dist / 3.0);
                    let expect = full - removed;
                    let census = surface_census(&t, r.solid);
                    let (mb, mnm) = mesh_edges(&t, r.solid);
                    println!(
                        "  v2 chamfer edge[{i}]: OK {before:?} -> {after:?} failed={} \n\
                         \t vol {vol:.2} vs {expect:.2} (err {:.2e}) mesh b={mb} nm={mnm} {census:?}",
                        r.failed.len(),
                        ((vol - expect) / expect).abs()
                    );
                }
                Err(err) => println!("  v2 chamfer edge[{i}]: ERR {err}"),
            }
            // fillet_v2 for comparison — its builder HAS a closed-rim path.
            let mut t = Topology::new();
            let c = primitives::make_cylinder(&mut t, 45.0, 10.0).unwrap();
            let es = solid_edges(&t, c);
            let before = counts(&t, c);
            match remus_operations::blend_ops::fillet_v2(&mut t, c, &[es[i]], dist) {
                Ok(r) => {
                    let after = counts(&t, r.solid);
                    println!(
                        "  v2 fillet  edge[{i}]: OK {before:?} -> {after:?} failed={}{}",
                        r.failed.len(),
                        if before == after { "  <-- NO-OP" } else { "" }
                    );
                }
                Err(err) => println!("  v2 fillet  edge[{i}]: ERR {err}"),
            }
            let _ = e;
        }
    }
}
