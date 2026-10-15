//! Deterministic instruction-count (cachegrind `Ir`) workloads with an
//! output digest.
//!
//! Each workload runs once and prints a digest of its result: per face in
//! solid order, the surface type and exact `Debug` geometry, orientation, and
//! every wire's ordered traversal with vertex position bits, curve geometry,
//! each use's coedge authority (its edge, stored pcurve, periodic winding)
//! and the solid's connectivity: which edge and vertices each use names
//! (first-seen ordinals) and how many uses share each. Tessellation
//! workloads digest position bits and indices. Two builds whose digests
//! match produced bit-identical output. Solids also print a `canonical`
//! digest of the same content that ignores face order and closed-wire
//! start, for workloads whose order varies run to run.
//!
//! glibc selects libm variants by CPU feature, and valgrind's virtual CPU
//! reports fewer features, so digests compare native with native and
//! valgrind with valgrind.
//!
//! Valgrind cannot read lld-linked symbols, so link with bfd:
//!
//! ```text
//! cargo rustc --profile profiling -p remus-operations --example ir_probe \
//!     -- -C link-arg=-fuse-ld=bfd
//! RAYON_NUM_THREADS=1 valgrind --tool=cachegrind --cache-sim=no \
//!     target/profiling/examples/ir_probe <workload>
//! ```
//!
//! Run with no argument to list the workloads.

#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    missing_docs
)]

use std::fmt::Write as _;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::compound_ops::fuse_all;
#[allow(deprecated)]
use remus_operations::fillet::fillet_rolling_ball;
use remus_operations::primitives;
use remus_operations::tessellate::{self, TriangleMesh};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::compound::Compound;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

struct Fnv(u64);

impl Fnv {
    const fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
    fn bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
    }
    fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    fn str(&mut self, s: &str) {
        self.u64(s.len() as u64);
        self.bytes(s.as_bytes());
    }
}

/// Hash of one oriented edge use: direction, end vertex bits and curve.
fn oriented_edge_hash(
    topo: &Topology,
    oe: &remus_topology::wire::OrientedEdge,
    coedge: Option<remus_topology::coedge::CoedgeId>,
    sharing: &Sharing,
    scratch: &mut String,
) -> u64 {
    let mut h = Fnv::new();
    let edge = topo.edge(oe.edge()).unwrap();
    h.u64(u64::from(oe.is_forward()));
    // How many uses share this edge and each end vertex: connectivity that
    // survives reordering, so a duplicated or aliased entity changes it.
    h.u64(sharing.edge_uses[&oe.edge()]);
    for vid in [oe.oriented_start(edge), oe.oriented_end(edge)] {
        h.u64(sharing.vertex_uses[&vid]);
    }
    for vid in [oe.oriented_start(edge), oe.oriented_end(edge)] {
        let v = topo.vertex(vid).unwrap();
        let p = v.point();
        for c in [p.x(), p.y(), p.z(), v.tolerance()] {
            h.u64(c.to_bits());
        }
    }
    scratch.clear();
    write!(
        scratch,
        "{:?}|{:?}|{:?}",
        edge.curve(),
        edge.trim(),
        edge.tolerance()
    )
    .unwrap();
    h.str(scratch);
    // The use's own authority: its stored pcurve and periodic winding.
    scratch.clear();
    match coedge.map(|id| topo.coedge(id).unwrap()) {
        Some(c) => {
            write!(
                scratch,
                "{}|{:?}|{:?}",
                c.is_forward(),
                c.pcurve(),
                c.periodic_winding()
            )
            .unwrap();
            // The coedge names its own edge. Agreement leaves the hash as
            // is; any other edge is hashed by its sharing and geometry.
            if c.edge() != oe.edge() {
                let other = topo.edge(c.edge()).unwrap();
                write!(
                    scratch,
                    "|other-edge|{}|{:?}|{:?}|{:?}",
                    sharing.edge_uses.get(&c.edge()).copied().unwrap_or(0),
                    other.curve(),
                    other.trim(),
                    other.tolerance()
                )
                .unwrap();
            }
        }
        None => scratch.push_str("no-coedge"),
    }
    h.str(scratch);
    h.0
}

/// Use counts of every edge and vertex over a solid's wire uses, and each
/// entity's first-seen ordinal in solid order.
struct Sharing {
    edge_uses: std::collections::HashMap<remus_topology::edge::EdgeId, u64>,
    vertex_uses: std::collections::HashMap<remus_topology::vertex::VertexId, u64>,
    edge_ordinal: std::collections::HashMap<remus_topology::edge::EdgeId, u64>,
    vertex_ordinal: std::collections::HashMap<remus_topology::vertex::VertexId, u64>,
}

impl Sharing {
    fn of(topo: &Topology, faces: &[remus_topology::face::FaceId]) -> Self {
        let mut sharing = Self {
            edge_uses: std::collections::HashMap::new(),
            vertex_uses: std::collections::HashMap::new(),
            edge_ordinal: std::collections::HashMap::new(),
            vertex_ordinal: std::collections::HashMap::new(),
        };
        for &fid in faces {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    let edge = topo.edge(oe.edge()).unwrap();
                    *sharing.edge_uses.entry(oe.edge()).or_default() += 1;
                    let next = sharing.edge_ordinal.len() as u64;
                    sharing.edge_ordinal.entry(oe.edge()).or_insert(next);
                    for vid in [oe.oriented_start(edge), oe.oriented_end(edge)] {
                        *sharing.vertex_uses.entry(vid).or_default() += 1;
                        let next = sharing.vertex_ordinal.len() as u64;
                        sharing.vertex_ordinal.entry(vid).or_insert(next);
                    }
                }
            }
        }
        sharing
    }
}

/// Digests of a solid: `strict` hashes faces in solid order and every wire
/// from its stored first edge; `canonical` is the same content with faces
/// sorted and each closed wire rotated to its smallest edge hash, so it is
/// blind only to face order and wire start.
fn solid_digest(topo: &Topology, solid: SolidId) -> String {
    let mut strict = Fnv::new();
    let mut face_hashes = Vec::new();
    let faces = solid_faces(topo, solid).unwrap();
    let sharing = Sharing::of(topo, &faces);
    let mut edges = 0usize;
    let mut kinds = std::collections::BTreeMap::<&str, usize>::new();
    let mut scratch = String::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        *kinds.entry(face.surface().type_tag()).or_default() += 1;
        let mut head = Fnv::new();
        scratch.clear();
        write!(scratch, "{:?}", face.surface()).unwrap();
        head.str(&scratch);
        head.u64(u64::from(face.is_reversed()));
        head.u64(face.inner_wires().len() as u64);
        strict.u64(head.0);
        let mut canon_wires = Vec::new();
        // Loop `j` holds the coedges of wire `j`, position by position.
        let loops = topo.loops_of_face(fid).unwrap_or(&[]);
        let wires = std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied());
        for (j, wid) in wires.enumerate() {
            let wire = topo.wire(wid).unwrap();
            let coedges = loops
                .get(j)
                .map(|&l| topo.face_loop(l).unwrap().coedges().to_vec())
                .unwrap_or_default();
            let uses: Vec<u64> = wire
                .edges()
                .iter()
                .enumerate()
                .map(|(k, oe)| {
                    oriented_edge_hash(topo, oe, coedges.get(k).copied(), &sharing, &mut scratch)
                })
                .collect();
            edges += uses.len();
            strict.u64(u64::from(wire.is_closed()));
            strict.u64(uses.len() as u64);
            for (&u, oe) in uses.iter().zip(wire.edges()) {
                strict.u64(u);
                // Which entity each use names, by first-seen ordinal.
                let edge = topo.edge(oe.edge()).unwrap();
                strict.u64(sharing.edge_ordinal[&oe.edge()]);
                strict.u64(sharing.vertex_ordinal[&oe.oriented_start(edge)]);
                strict.u64(sharing.vertex_ordinal[&oe.oriented_end(edge)]);
            }
            let start = if wire.is_closed() {
                (0..uses.len()).min_by_key(|&i| uses[i]).unwrap_or(0)
            } else {
                0
            };
            let mut w = Fnv::new();
            w.u64(u64::from(wire.is_closed()));
            for i in 0..uses.len() {
                w.u64(uses[(start + i) % uses.len()]);
            }
            canon_wires.push(w.0);
        }
        canon_wires[1..].sort_unstable();
        for w in canon_wires {
            head.u64(w);
        }
        face_hashes.push(head.0);
    }
    face_hashes.sort_unstable();
    let mut canonical = Fnv::new();
    canonical.u64(sharing.edge_uses.len() as u64);
    canonical.u64(sharing.vertex_uses.len() as u64);
    for f in face_hashes {
        canonical.u64(f);
    }
    format!(
        "faces={} edges={} vertices={} oriented_edges={} kinds={kinds:?} digest={:016x} \
         canonical={:016x}",
        faces.len(),
        sharing.edge_uses.len(),
        sharing.vertex_uses.len(),
        edges,
        strict.0,
        canonical.0
    )
}

fn mesh_digest(mesh: &TriangleMesh) -> String {
    let mut h = Fnv::new();
    for p in &mesh.positions {
        for c in [p.x(), p.y(), p.z()] {
            h.u64(c.to_bits());
        }
    }
    for n in &mesh.normals {
        for c in [n.x(), n.y(), n.z()] {
            h.u64(c.to_bits());
        }
    }
    for &i in &mesh.indices {
        h.u64(u64::from(i));
    }
    format!(
        "positions={} triangles={} digest={:016x}",
        mesh.positions.len(),
        mesh.indices.len() / 3,
        h.0
    )
}

fn collect_edges(topo: &Topology, solid: SolidId) -> Vec<remus_topology::edge::EdgeId> {
    let mut edges = Vec::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for oe in topo.wire(face.outer_wire()).unwrap().edges() {
            if !edges.contains(&oe.edge()) {
                edges.push(oe.edge());
            }
        }
    }
    edges
}

fn box_grid(n: usize, size: f64) -> (Topology, Vec<SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::with_capacity(n * n);
    for row in 0..n {
        for col in 0..n {
            let bx = primitives::make_box(&mut topo, size, size, 1.0).unwrap();
            let mat = Mat4::translation(col as f64, row as f64, 0.0);
            transform_solid(&mut topo, bx, &mat).unwrap();
            solids.push(bx);
        }
    }
    (topo, solids)
}

/// `box+fillet` in `cad_operations`: box(20) with every edge filleted at r=1.
fn box_fillet() -> String {
    let mut topo = Topology::new();
    let solid = primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let edges = collect_edges(&topo, solid);
    #[allow(deprecated)]
    let out = fillet_rolling_ball(&mut topo, solid, &edges, 1.0).unwrap();
    solid_digest(&topo, out)
}

/// box(20) with every edge filleted at r=1 by the v2 walking engine
/// (`blend_ops::fillet_v2`).
fn box_fillet_v2() -> String {
    let mut topo = Topology::new();
    let solid = primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let edges = collect_edges(&topo, solid);
    let out = remus_operations::blend_ops::fillet_v2(&mut topo, solid, &edges, 1.0).unwrap();
    solid_digest(&topo, out.solid)
}

/// `fuse_balanced/balanced_N=144` and `fuse_touching/balanced_N=144`.
fn fuse_grid(size: f64) -> String {
    let (mut topo, solids) = box_grid(12, size);
    let cid = topo.add_compound(Compound::new(solids));
    let out = fuse_all(&mut topo, cid).unwrap();
    solid_digest(&topo, out)
}

/// `fuse_balanced/sequential_N=16`.
fn sequential_fuse_16() -> String {
    let (mut topo, solids) = box_grid(4, 1.01);
    let mut result = solids[0];
    for &s in &solids[1..] {
        result = boolean(&mut topo, BooleanOp::Fuse, result, s).unwrap();
    }
    solid_digest(&topo, result)
}

/// `cut(box,cyl) x10` in `cad_operations`.
fn cut_box_cyl_x10() -> String {
    let mut all = Fnv::new();
    let mut last = String::new();
    for _ in 0..10 {
        let mut topo = Topology::new();
        let a = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let cyl = primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
        let r = boolean(&mut topo, BooleanOp::Cut, a, cyl).unwrap();
        last = solid_digest(&topo, r);
        all.str(&last);
    }
    format!("{last} all={:016x}", all.0)
}

/// `mesh sphere (tol=0.01)` in `cad_operations`.
fn mesh_sphere() -> String {
    let mut topo = Topology::new();
    let solid = primitives::make_sphere(&mut topo, 10.0, 16).unwrap();
    let mut combined = TriangleMesh::default();
    for fid in solid_faces(&topo, solid).unwrap() {
        if let Ok(mesh) = tessellate::tessellate(&topo, fid, 0.01) {
            let offset = combined.positions.len() as u32;
            combined.positions.extend_from_slice(&mesh.positions);
            combined.normals.extend_from_slice(&mesh.normals);
            combined
                .indices
                .extend(mesh.indices.iter().map(|i| i + offset));
        }
    }
    mesh_digest(&combined)
}

/// `sequential_cylinder_cuts/N=64` in `boolean_perf`.
fn sequential_cuts_64() -> String {
    let n = 64usize;
    let mut topo = Topology::new();
    let mut result = primitives::make_box(&mut topo, 100.0, 100.0, 10.0).unwrap();
    let cols = (n as f64).sqrt().ceil() as usize;
    let rows = n.div_ceil(cols);
    let x_spacing = 100.0 / (cols + 1) as f64;
    let y_spacing = 100.0 / (rows + 1) as f64;
    for i in 0..n {
        let x = x_spacing * (i % cols + 1) as f64;
        let y = y_spacing * (i / cols + 1) as f64;
        let cyl = primitives::make_cylinder(&mut topo, 2.0, 20.0).unwrap();
        transform_solid(&mut topo, cyl, &Mat4::translation(x, y, -5.0)).unwrap();
        result = boolean(&mut topo, BooleanOp::Cut, result, cyl).unwrap();
    }
    solid_digest(&topo, result)
}

type Workload = fn() -> String;

/// Whole-solid tessellation of a 64-hole plate: many band faces sharing one
/// vertex pool.
fn mesh_plate_64() -> String {
    let n = 64usize;
    let mut topo = Topology::new();
    let mut plate = primitives::make_box(&mut topo, 100.0, 100.0, 10.0).unwrap();
    for i in 0..n {
        let x = 100.0 / 9.0 * (i % 8 + 1) as f64;
        let y = 100.0 / 9.0 * (i / 8 + 1) as f64;
        let cyl = primitives::make_cylinder(&mut topo, 2.0, 20.0).unwrap();
        transform_solid(&mut topo, cyl, &Mat4::translation(x, y, -5.0)).unwrap();
        plate = boolean(&mut topo, BooleanOp::Cut, plate, cyl).unwrap();
    }
    let mesh = tessellate::tessellate_solid(&topo, plate, 0.01).unwrap();
    mesh_digest(&mesh)
}

const WORKLOADS: &[(&str, Workload)] = &[
    ("mesh_plate_64", mesh_plate_64),
    ("box_fillet", box_fillet),
    ("box_fillet_v2", box_fillet_v2),
    ("fuse_overlap_12", || fuse_grid(1.01)),
    ("fuse_touch_12", || fuse_grid(1.0)),
    ("seq_fuse_16", sequential_fuse_16),
    ("cut_box_cyl_x10", cut_box_cyl_x10),
    ("mesh_sphere", mesh_sphere),
    ("seq_cuts_64", sequential_cuts_64),
];

fn main() {
    let Some(name) = std::env::args().nth(1) else {
        for (name, _) in WORKLOADS {
            eprintln!("{name}");
        }
        return;
    };
    let Some((_, run)) = WORKLOADS.iter().find(|(n, _)| *n == name) else {
        eprintln!("unknown workload {name}");
        std::process::exit(2);
    };
    println!("{name} {}", run());
}
