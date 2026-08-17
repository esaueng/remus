//! Repro/probe: Cut with a multi-component subtrahend on the flange blank.
//!
//! Sweeps N = 1..6 bolt cylinders fused into ONE subtrahend body and cuts that
//! from the unified rim+hub blank in a single boolean. Reports the surface
//! census, B-Rep edge usage AND mesh-level watertightness for each N — the
//! B-Rep can be closed while the tessellation is not, so both are printed.
//!
//! Also isolates the blank itself, which carries a separate, pre-existing
//! defect: the r12 bore seam is split at z=10 by a spurious vertex (the rim
//! cap's unbounded plane paves the bore seam even though the cap annulus spans
//! r24..45 and never reaches r=12). The B-Rep stays closed, but the cylinder
//! band mesher mis-stitches the split seam into 8 unmatched mesh edges.
//!
//! Run: `cargo run --release --example debug_multicut -p remus-operations`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    clippy::print_stderr
)]

use std::collections::BTreeMap;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::primitives;
use remus_operations::revolve::revolve;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-7;

fn census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut m = BTreeMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        *m.entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    m
}

/// B-Rep edge usage: 1 = free boundary, 3+ = non-manifold.
fn free_and_nonmanifold(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mut usage: BTreeMap<usize, usize> = BTreeMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    (
        usage.values().filter(|&&c| c == 1).count(),
        usage.values().filter(|&&c| c >= 3).count(),
    )
}

/// Weld the mesh by position, then return `(boundary, non_manifold, bad_edges)`
/// with a representative 3D point per unmatched edge so it can be located.
#[allow(clippy::type_complexity)]
fn mesh_edge_report(topo: &Topology, solid: SolidId) -> (usize, usize, Vec<(Point3, Point3, u32)>) {
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).unwrap();
    let q = 1e6;
    let mut canon: BTreeMap<(i64, i64, i64), u32> = BTreeMap::new();
    let mut rep: Vec<Point3> = Vec::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        let next = canon.len() as u32;
        let id = *canon.entry(key).or_insert(next);
        if id as usize == rep.len() {
            rep.push(*p);
        }
        remap[i] = id;
    }
    let mut edges: BTreeMap<(u32, u32), u32> = BTreeMap::new();
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
    let bad: Vec<_> = edges
        .iter()
        .filter(|&(_, &c)| c != 2)
        .map(|(&(a, b), &c)| (rep[a as usize], rep[b as usize], c))
        .collect();
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c >= 3).count(),
        bad,
    )
}

fn revolved_annulus(
    topo: &mut Topology,
    r_inner: f64,
    r_outer: f64,
    z_lo: f64,
    z_hi: f64,
) -> SolidId {
    let pts = [
        Point3::new(r_inner, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_hi),
        Point3::new(r_inner, 0.0, z_hi),
    ];
    let wire = make_polygon_wire(topo, &pts, TOL).unwrap();
    let face = make_planar_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
    )
    .unwrap()
}

fn blank(topo: &mut Topology) -> SolidId {
    let rim = revolved_annulus(topo, 24.0, 45.0, 0.0, 10.0);
    let hub = revolved_annulus(topo, 12.0, 24.0, 0.0, 26.0);
    let b = boolean(topo, BooleanOp::Fuse, rim, hub).expect("blank fuse");
    unify_faces(topo, b).unwrap();
    b
}

fn bolt(topo: &mut Topology, i: usize, n: usize) -> SolidId {
    #[allow(clippy::cast_precision_loss)]
    let angle = std::f64::consts::TAU * (i as f64) / (n as f64);
    let c = primitives::make_cylinder(topo, 3.0, 16.0).unwrap();
    transform_solid(
        topo,
        c,
        &Mat4::translation(34.0 * angle.cos(), 34.0 * angle.sin(), -3.0),
    )
    .unwrap();
    c
}

/// Print the r12 bore face's wire — a split seam shows up as two Line edges
/// meeting at an interior vertex instead of one full-height seam.
fn dump_bore_wire(topo: &Topology, solid: SolidId) {
    println!("  r12 bore face wire:");
    for fid in solid_faces(topo, solid).unwrap() {
        let f = topo.face(fid).unwrap();
        if f.surface().type_tag() != "cylinder" {
            continue;
        }
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                let a = topo.vertex(e.start()).unwrap().point();
                let b = topo.vertex(e.end()).unwrap().point();
                if (a.x().hypot(a.y()) - 12.0).abs() < 1e-6 {
                    println!(
                        "    {:>8}: ({:.2},{:.2},{:.2}) -> ({:.2},{:.2},{:.2})",
                        e.curve().type_tag(),
                        a.x(),
                        a.y(),
                        a.z(),
                        b.x(),
                        b.y(),
                        b.z()
                    );
                }
            }
        }
    }
}

fn main() {
    env_logger::init();

    // --- The blank on its own, and the pieces it is built from. ---
    let mut t = Topology::new();
    let b0 = blank(&mut t);
    let (mb, mnm, bad) = mesh_edge_report(&t, b0);
    println!(
        "blank:             {:?} brep free/nm={:?} | mesh boundary={mb} nm={mnm}",
        census(&t, b0),
        free_and_nonmanifold(&t, b0)
    );
    for (a, b, c) in &bad {
        println!(
            "    use={c} ({:.3},{:.3},{:.3})[r={:.3}] -> ({:.3},{:.3},{:.3})[r={:.3}]",
            a.x(),
            a.y(),
            a.z(),
            a.x().hypot(a.y()),
            b.x(),
            b.y(),
            b.z(),
            b.x().hypot(b.y())
        );
    }
    dump_bore_wire(&t, b0);

    let mut t2 = Topology::new();
    let rim = revolved_annulus(&mut t2, 24.0, 45.0, 0.0, 10.0);
    let (mb, mnm, _) = mesh_edge_report(&t2, rim);
    println!("rim annulus alone: mesh boundary={mb} nm={mnm}");

    let mut t3 = Topology::new();
    let cyl = primitives::make_cylinder(&mut t3, 10.0, 5.0).unwrap();
    let (mb, mnm, _) = mesh_edge_report(&t3, cyl);
    println!("plain cylinder:    mesh boundary={mb} nm={mnm}");

    // --- The sweep: N bolts fused into ONE subtrahend, cut in one boolean. ---
    for n in 1..=6usize {
        let mut topo = Topology::new();
        let body = blank(&mut topo);

        let mut pattern = bolt(&mut topo, 0, n);
        for i in 1..n {
            let next = bolt(&mut topo, i, n);
            pattern = boolean(&mut topo, BooleanOp::Fuse, pattern, next)
                .unwrap_or_else(|e| panic!("N={n}: pattern fuse {i} failed: {e:?}"));
        }

        match boolean(&mut topo, BooleanOp::Cut, body, pattern) {
            Ok(r) => {
                let (f, nm) = free_and_nonmanifold(&topo, r);
                let (mb, mnm, _) = mesh_edge_report(&topo, r);
                println!(
                    "N={n} OK  {:?} brep free={f} nm={nm} | mesh boundary={mb} nm={mnm}",
                    census(&topo, r)
                );
            }
            Err(e) => println!("N={n} FAILED {e:?}"),
        }
    }
}
