//! PERF-D07: the whole-mesh passes as they were before their rewrite, kept
//! verbatim as test oracles, and comparisons of the pipeline with them.
//!
//! The rewritten passes (`mesh_ops`) and the indexed circle contact
//! refinement (`pool_index`) must produce byte-identical meshes. Under
//! `cfg(test)` a thread-local switch (`use_reference_passes`) routes the
//! pipeline through these copies and through the full-pool circle scan, so
//! every comparison below meshes the same body both ways.

#![allow(clippy::expect_used, clippy::panic)]

use std::cell::Cell;

use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::super::TriangleMesh;
use super::super::mesh_ops::COINCIDENT_DEDUPE_GRID;
use super::super::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_grouped_with_tolerance,
};

thread_local! {
    static REFERENCE: Cell<bool> = const { Cell::new(false) };
}

/// Whether this thread's pipeline runs the reference passes.
pub(in crate::tessellate) fn use_reference_passes() -> bool {
    REFERENCE.with(Cell::get)
}

/// Run `f` with the reference passes switched on for this thread.
fn with_reference<T>(f: impl FnOnce() -> T) -> T {
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            REFERENCE.with(|r| r.set(false));
        }
    }
    REFERENCE.with(|r| r.set(true));
    let _reset = Reset;
    f()
}

/// Reference implementations (verbatim before PERF-D07).
pub(in crate::tessellate) mod reference {
    use super::{COINCIDENT_DEDUPE_GRID, DetHashMap, DetHashSet, Point3, TriangleMesh, Vec3};

    pub(in crate::tessellate) fn dedupe_coincident_triangles(
        mesh: &mut TriangleMesh,
        tri_faces: Option<&mut Vec<u32>>,
    ) {
        const POS_GRID: f64 = COINCIDENT_DEDUPE_GRID;

        type TriKey = [(i64, i64, i64); 3];
        type TriRefs = Vec<(usize, bool)>;

        let tri_count = mesh.indices.len() / 3;
        if tri_count < 2 {
            return;
        }

        #[allow(clippy::cast_possible_truncation)]
        let quant = |p: Point3| -> (i64, i64, i64) {
            let s = 1.0 / POS_GRID;
            (
                (p.x() * s).round() as i64,
                (p.y() * s).round() as i64,
                (p.z() * s).round() as i64,
            )
        };

        let mut by_key: DetHashMap<TriKey, TriRefs> = DetHashMap::default();
        for t in 0..tri_count {
            let (a, b, c) = (
                mesh.indices[t * 3] as usize,
                mesh.indices[t * 3 + 1] as usize,
                mesh.indices[t * 3 + 2] as usize,
            );
            let mut tri_pts = [
                quant(mesh.positions[a]),
                quant(mesh.positions[b]),
                quant(mesh.positions[c]),
            ];
            // Sort tri_pts ascending; track parity of the sort permutation.
            let mut parity_even = true;
            if tri_pts[0] > tri_pts[1] {
                tri_pts.swap(0, 1);
                parity_even = !parity_even;
            }
            if tri_pts[1] > tri_pts[2] {
                tri_pts.swap(1, 2);
                parity_even = !parity_even;
            }
            if tri_pts[0] > tri_pts[1] {
                tri_pts.swap(0, 1);
                parity_even = !parity_even;
            }
            // Skip degenerate triangles (collapsed to <3 distinct positions).
            if tri_pts[0] == tri_pts[1] || tri_pts[1] == tri_pts[2] {
                continue;
            }
            by_key.entry(tri_pts).or_default().push((t, parity_even));
        }

        let mut keep = vec![true; tri_count];
        for tris in by_key.values() {
            if tris.len() < 2 {
                continue;
            }
            let (even, odd): (Vec<_>, Vec<_>) = tris.iter().partition(|&&(_, p)| p);
            let cancel_pairs = even.len().min(odd.len());
            for &(t, _) in even.iter().take(cancel_pairs) {
                keep[t] = false;
            }
            for &(t, _) in odd.iter().take(cancel_pairs) {
                keep[t] = false;
            }
            // Of the surviving same-winding triangles, keep only one.
            let leftover_even: Vec<_> = even.iter().skip(cancel_pairs).copied().collect();
            let leftover_odd: Vec<_> = odd.iter().skip(cancel_pairs).copied().collect();
            for &(t, _) in leftover_even.iter().skip(1) {
                keep[t] = false;
            }
            for &(t, _) in leftover_odd.iter().skip(1) {
                keep[t] = false;
            }
        }

        if keep.iter().all(|&k| k) {
            return;
        }

        let mut new_indices = Vec::with_capacity(mesh.indices.len());
        let mut new_tri_faces = Vec::with_capacity(tri_faces.as_ref().map_or(0, |tf| tf.len()));
        for (t, &k) in keep.iter().enumerate().take(tri_count) {
            if k {
                new_indices.extend_from_slice(&mesh.indices[t * 3..t * 3 + 3]);
                if let Some(&f) = tri_faces.as_ref().and_then(|tf| tf.get(t)) {
                    new_tri_faces.push(f);
                }
            }
        }
        if let Some(tf) = tri_faces {
            *tf = new_tri_faces;
        }

        // Compact the position/normal buffers: drop any vertex no longer
        // referenced by a surviving triangle. Downstream consumers that iterate
        // `mesh.positions` directly (e.g. bbox passes, exporters that walk
        // vertices rather than triangle indices) would otherwise see phantom
        // vertices from removed triangles.
        let n_verts = mesh.positions.len();
        let mut remap: Vec<u32> = vec![u32::MAX; n_verts];
        let mut new_positions: Vec<Point3> = Vec::new();
        let mut new_normals: Vec<Vec3> = Vec::new();
        for idx in &mut new_indices {
            let old = *idx as usize;
            if remap[old] == u32::MAX {
                #[allow(clippy::cast_possible_truncation)]
                let new_id = new_positions.len() as u32;
                remap[old] = new_id;
                new_positions.push(mesh.positions[old]);
                if old < mesh.normals.len() {
                    new_normals.push(mesh.normals[old]);
                }
            }
            *idx = remap[old];
        }
        mesh.indices = new_indices;
        mesh.positions = new_positions;
        mesh.normals = new_normals;
    }

    pub(in crate::tessellate) fn weld_boundary_vertices(
        mesh: &mut TriangleMesh,
        deflection: f64,
        tri_faces: Option<&mut Vec<u32>>,
    ) {
        let n_verts = mesh.positions.len();
        if n_verts == 0 || mesh.indices.is_empty() {
            return;
        }

        let mut half_edges: DetHashMap<(u32, u32), usize> = DetHashMap::default();
        for tri in mesh.indices.chunks_exact(3) {
            let (i0, i1, i2) = (tri[0], tri[1], tri[2]);
            *half_edges.entry((i0, i1)).or_default() += 1;
            *half_edges.entry((i1, i2)).or_default() += 1;
            *half_edges.entry((i2, i0)).or_default() += 1;
        }

        // Boundary vertices: incident on half-edges without a matching reverse.
        let mut boundary_set: DetHashSet<u32> = DetHashSet::default();
        for &(a, b) in half_edges.keys() {
            if !half_edges.contains_key(&(b, a)) {
                boundary_set.insert(a);
                boundary_set.insert(b);
            }
        }

        if boundary_set.is_empty() {
            return;
        }

        // Sorted iteration keeps grid-cell contents and union order independent
        // of DetHashSet iteration order, so welded meshes are reproducible.
        let mut boundary_verts: Vec<u32> = boundary_set.into_iter().collect();
        boundary_verts.sort_unstable();

        #[allow(clippy::items_after_statements)]
        fn uf_find(parent: &mut [u32], mut x: u32) -> u32 {
            while parent[x as usize] != x {
                parent[x as usize] = parent[parent[x as usize] as usize];
                x = parent[x as usize];
            }
            x
        }
        // Rooting at the smallest index makes the cluster representative a pure
        // function of the weld partition, independent of union call order.
        #[allow(clippy::items_after_statements)]
        fn uf_union(parent: &mut [u32], a: u32, b: u32) {
            let ra = uf_find(parent, a);
            let rb = uf_find(parent, b);
            if ra != rb {
                let (root, child) = (ra.min(rb), ra.max(rb));
                parent[child as usize] = root;
            }
        }

        let mut parent: Vec<u32> = (0..n_verts as u32).collect();

        let weld_tol = deflection.max(1e-6) * 2.0;
        let inv_cell = 1.0 / weld_tol;

        #[allow(clippy::cast_possible_truncation)]
        let cell_key = |p: Point3| -> (i64, i64, i64) {
            (
                (p.x() * inv_cell).floor() as i64,
                (p.y() * inv_cell).floor() as i64,
                (p.z() * inv_cell).floor() as i64,
            )
        };

        let mut grid: DetHashMap<(i64, i64, i64), Vec<u32>> = DetHashMap::default();
        for &vid in &boundary_verts {
            let p = mesh.positions[vid as usize];
            grid.entry(cell_key(p)).or_default().push(vid);
        }

        for &vid in &boundary_verts {
            let p = mesh.positions[vid as usize];
            let (cx, cy, cz) = cell_key(p);

            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(cell) = grid.get(&(cx + dx, cy + dy, cz + dz)) {
                            for &other in cell {
                                if other <= vid {
                                    continue;
                                }
                                let q = mesh.positions[other as usize];
                                if (p - q).length() < weld_tol {
                                    uf_union(&mut parent, vid, other);
                                }
                            }
                        }
                    }
                }
            }
        }

        let mut changed = false;
        for idx in &mut mesh.indices {
            let root = uf_find(&mut parent, *idx);
            if root != *idx {
                *idx = root;
                changed = true;
            }
        }

        if changed {
            let mut new_indices = Vec::with_capacity(mesh.indices.len());
            let mut new_tri_faces = Vec::with_capacity(tri_faces.as_ref().map_or(0, |tf| tf.len()));
            for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
                let (i0, i1, i2) = (tri[0], tri[1], tri[2]);
                if i0 != i1 && i1 != i2 && i2 != i0 {
                    new_indices.push(i0);
                    new_indices.push(i1);
                    new_indices.push(i2);
                    if let Some(&f) = tri_faces.as_ref().and_then(|tf| tf.get(t)) {
                        new_tri_faces.push(f);
                    }
                }
            }
            mesh.indices = new_indices;
            if let Some(tf) = tri_faces {
                *tf = new_tri_faces;
            }
        }
    }

    pub(in crate::tessellate) fn fill_sub_deflection_triangular_gaps(
        mesh: &mut TriangleMesh,
        deflection: f64,
        mut tri_faces: Option<&mut Vec<u32>>,
    ) {
        if !deflection.is_finite() || deflection <= 0.0 || mesh.indices.len() < 9 {
            return;
        }

        let mut directed = DetHashMap::<(u32, u32), usize>::default();
        for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
            for edge in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                directed.entry(edge).or_insert(triangle);
            }
        }
        let mut boundary: Vec<_> = directed
            .iter()
            .filter_map(|(&(a, b), &triangle)| {
                (!directed.contains_key(&(b, a))).then_some((a, b, triangle))
            })
            .collect();
        boundary.sort_unstable();
        let boundary_map: DetHashMap<(u32, u32), usize> = boundary
            .iter()
            .map(|&(a, b, triangle)| ((a, b), triangle))
            .collect();

        let face_by_triangle = tri_faces.as_deref();
        let mut fillers = Vec::<([u32; 3], u32)>::new();
        let mut seen = DetHashSet::<[u32; 3]>::default();
        for &(a, b, ab_triangle) in &boundary {
            for (&(from, c), &bc_triangle) in &boundary_map {
                if from != b || c == a {
                    continue;
                }
                let Some(&ca_triangle) = boundary_map.get(&(c, a)) else {
                    continue;
                };
                let mut key = [a, b, c];
                key.sort_unstable();
                if !seen.insert(key) {
                    continue;
                }
                let mut sources = [ab_triangle, bc_triangle, ca_triangle];
                sources.sort_unstable();
                if sources[0] == sources[2] {
                    continue;
                }
                let (pa, pb, pc) = (
                    mesh.positions[a as usize],
                    mesh.positions[b as usize],
                    mesh.positions[c as usize],
                );
                let edge_lengths = [(pb - pa).length(), (pc - pb).length(), (pa - pc).length()];
                let longest = edge_lengths.into_iter().fold(0.0_f64, f64::max);
                if longest > 8.0 * deflection || longest <= f64::EPSILON {
                    continue;
                }
                let twice_area = (pb - pa).cross(pc - pa).length();
                let shortest_altitude = edge_lengths
                    .into_iter()
                    .filter(|&length| length > f64::EPSILON)
                    .map(|length| twice_area / length)
                    .fold(f64::INFINITY, f64::min);
                if shortest_altitude > deflection {
                    continue;
                }

                let face = face_by_triangle.map_or(0, |faces| {
                    [ab_triangle, bc_triangle, ca_triangle]
                        .into_iter()
                        .filter_map(|triangle| faces.get(triangle).copied())
                        .min()
                        .unwrap_or(0)
                });
                fillers.push(([b, a, c], face));
            }
        }

        for (triangle, face) in fillers {
            mesh.indices.extend_from_slice(&triangle);
            if let Some(faces) = tri_faces.as_deref_mut() {
                faces.push(face);
            }
        }
    }
}

fn grouped(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angular: f64,
) -> (TriangleMesh, Vec<u32>) {
    tessellate_solid_grouped_with_tolerance(topo, solid, deflection, angular).expect("tessellate")
}

fn assert_bit_identical(a: &(TriangleMesh, Vec<u32>), b: &(TriangleMesh, Vec<u32>), context: &str) {
    assert_eq!(a.1, b.1, "{context}: face offsets differ");
    assert_eq!(a.0.indices, b.0.indices, "{context}: indices differ");
    let bits = |m: &TriangleMesh| -> (Vec<[u64; 3]>, Vec<[u64; 3]>) {
        (
            m.positions
                .iter()
                .map(|p| [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()])
                .collect(),
            m.normals
                .iter()
                .map(|n| [n.x().to_bits(), n.y().to_bits(), n.z().to_bits()])
                .collect(),
        )
    };
    let (pa, na) = bits(&a.0);
    let (pb, nb) = bits(&b.0);
    assert!(pa == pb, "{context}: positions differ");
    assert!(na == nb, "{context}: normals differ");
}

/// Bodies whose display meshes exercise every pass: circle contact
/// insertions (fillets, drilled holes, boolean rims), coincident triangles
/// left by a coplanar fuse, cavities and the analytic primitives.
fn bodies(topo: &mut Topology) -> Vec<(&'static str, SolidId)> {
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
    use crate::transform::transform_solid;

    let mut out = Vec::new();
    let block = make_box(topo, 20.0, 20.0, 10.0).unwrap();
    let tool = make_cylinder(topo, 3.0, 20.0).unwrap();
    transform_solid(topo, tool, &Mat4::translation(10.0, 10.0, -5.0)).unwrap();
    out.push((
        "drilled",
        boolean(topo, BooleanOp::Cut, block, tool).unwrap(),
    ));

    let a = make_cylinder(topo, 4.0, 10.0).unwrap();
    let b = make_cylinder(topo, 4.0, 10.0).unwrap();
    transform_solid(topo, b, &Mat4::translation(5.0, 0.0, 3.0)).unwrap();
    out.push(("cylinders", boolean(topo, BooleanOp::Fuse, a, b).unwrap()));

    let base = make_box(topo, 10.0, 10.0, 4.0).unwrap();
    let boss = make_cylinder(topo, 2.5, 6.0).unwrap();
    transform_solid(topo, boss, &Mat4::translation(5.0, 5.0, 2.0)).unwrap();
    out.push(("boss", boolean(topo, BooleanOp::Fuse, base, boss).unwrap()));

    let left = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let right = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    transform_solid(topo, right, &Mat4::translation(10.0, 0.0, 0.0)).unwrap();
    out.push((
        "coplanar",
        boolean(topo, BooleanOp::Fuse, left, right).unwrap(),
    ));

    let sharp = make_box(topo, 20.0, 10.0, 10.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(topo, sharp).unwrap();
    if let Ok(filleted) = crate::blend_ops::fillet_v2(topo, sharp, &edges, 1.5) {
        out.push(("filleted", filleted.solid));
    }

    let solid = make_box(topo, 6.0, 6.0, 6.0).unwrap();
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let open = faces
        .into_iter()
        .find(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.99)
        })
        .unwrap();
    out.push((
        "hollow",
        crate::shell_op::shell(topo, solid, 1.0, &[open]).unwrap(),
    ));

    // A shallow extruded ridge filleted at r = 0.02: its blend rims touch
    // the laterals tangentially, so the circle contact refinement inserts
    // pool vertices into circle chains (and the line contact refinement
    // splits straight edges).
    let profile = remus_topology::builder::make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.05, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, -3.0, 0.0),
            Point3::new(0.0, -3.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(remus_topology::face::Face::new(
        profile,
        vec![],
        remus_topology::face::FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let prism = crate::extrude::extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 8.0).unwrap();
    let ridge = remus_topology::explorer::solid_edges(topo, prism)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let e = topo.edge(e).unwrap();
            let s = topo.vertex(e.start()).unwrap().point();
            let t = topo.vertex(e.end()).unwrap().point();
            (s.x() - 5.0).abs() < 1e-9 && (t.x() - 5.0).abs() < 1e-9 && (s.z() - t.z()).abs() > 1.0
        })
        .unwrap();
    out.push((
        "ridge",
        crate::blend_ops::fillet_v2(topo, prism, &[ridge], 0.02)
            .unwrap()
            .solid,
    ));

    out.push(("sphere", make_sphere(topo, 4.0, 24).unwrap()));
    out.push(("torus", make_torus(topo, 5.0, 1.5, 48).unwrap()));
    out.push(("cone", make_cone(topo, 3.0, 1.0, 4.0).unwrap()));
    out
}

#[test]
fn pipeline_matches_reference_passes_bit_for_bit() {
    let mut topo = Topology::new();
    for (label, body) in bodies(&mut topo) {
        for (deflection, angular) in [(0.2, 0.5), (0.05, 0.3), (0.01, 0.1)] {
            let context = format!("{label} at {deflection}");
            let fast = grouped(&topo, body, deflection, angular);
            let reference = with_reference(|| grouped(&topo, body, deflection, angular));
            assert_bit_identical(&fast, &reference, &context);
            assert_eq!(boundary_edge_count(&fast.0), 0, "{context}: open");
            assert_eq!(non_manifold_edge_count(&fast.0), 0, "{context}: branching");
            // The ungrouped entry point skips face attribution.
            let plain = super::super::tessellate_solid(&topo, body, deflection).unwrap();
            let plain_reference =
                with_reference(|| super::super::tessellate_solid(&topo, body, deflection).unwrap());
            assert_bit_identical(
                &(plain, Vec::new()),
                &(plain_reference, Vec::new()),
                &format!("{context} ungrouped"),
            );
        }
    }
}

/// Deterministic xorshift for the perturbation tests.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, n: usize) -> usize {
        #[allow(clippy::cast_possible_truncation)]
        let r = (self.next() % n as u64) as usize;
        r
    }
}

/// A real display mesh damaged the ways the passes repair: duplicated
/// triangles (same and opposite winding, on fresh coincident vertices),
/// degenerate triangles, removed triangles (open boundaries, three-edge
/// gaps) and vertices nudged within the weld radius.
fn damaged(mesh: &TriangleMesh, faces: &[u32], seed: u64) -> (TriangleMesh, Vec<u32>) {
    let mut rng = Rng(seed);
    let mut m = mesh.clone();
    let mut f = faces.to_vec();
    let tris = m.indices.len() / 3;
    for _ in 0..tris / 20 {
        let t = rng.below(tris);
        let mut tri = [m.indices[t * 3], m.indices[t * 3 + 1], m.indices[t * 3 + 2]];
        match rng.below(4) {
            0 => tri.swap(1, 2),
            1 => {
                // Same positions on new vertex ids.
                for corner in &mut tri {
                    let p = m.positions[*corner as usize];
                    m.positions.push(p);
                    m.normals.push(Vec3::new(0.0, 0.0, 1.0));
                    #[allow(clippy::cast_possible_truncation)]
                    {
                        *corner = (m.positions.len() - 1) as u32;
                    }
                }
                if rng.below(2) == 0 {
                    tri.swap(0, 2);
                }
            }
            2 => tri[2] = tri[1],
            _ => tri.rotate_left(1),
        }
        m.indices.extend_from_slice(&tri);
        f.push(f[t]);
    }
    // Remove some triangles (boundaries and gaps).
    let mut keep = vec![true; m.indices.len() / 3];
    for _ in 0..tris / 50 {
        keep[rng.below(tris)] = false;
    }
    let mut indices = Vec::new();
    let mut kept_faces = Vec::new();
    for (t, &k) in keep.iter().enumerate() {
        if k {
            indices.extend_from_slice(&m.indices[t * 3..t * 3 + 3]);
            kept_faces.push(f[t]);
        }
    }
    m.indices = indices;
    // Nudge some vertices by less than the dedupe grid and the weld radius.
    for _ in 0..m.positions.len() / 30 {
        let v = rng.below(m.positions.len());
        #[allow(clippy::cast_precision_loss)]
        let d = (rng.below(1000) as f64 - 500.0) * 1e-10;
        let p = m.positions[v];
        m.positions[v] = Point3::new(p.x() + d, p.y() - d, p.z() + 0.5 * d);
    }
    (m, kept_faces)
}

fn tri_faces_from_offsets(offsets: &[u32]) -> Vec<u32> {
    let mut faces = Vec::new();
    for (face, w) in offsets.windows(2).enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        faces.extend(std::iter::repeat_n(
            face as u32,
            ((w[1] - w[0]) / 3) as usize,
        ));
    }
    faces
}

#[test]
fn passes_match_reference_on_damaged_meshes() {
    let mut topo = Topology::new();
    let mut exercised = [0_usize; 3];
    for (label, body) in bodies(&mut topo) {
        let (mesh, offsets) = grouped(&topo, body, 0.05, 0.3);
        let faces = tri_faces_from_offsets(&offsets);
        for seed in 1..=6_u64 {
            let (input, input_faces) = damaged(&mesh, &faces, seed * 7919 + 1);
            for deflection in [0.05, 1e-9] {
                let context = format!("{label} seed {seed} deflection {deflection}");
                for pass in 0..3 {
                    let run = |reference: bool| {
                        let mut m = input.clone();
                        let mut f = input_faces.clone();
                        let call = |m: &mut TriangleMesh, f: Option<&mut Vec<u32>>| match pass {
                            0 => super::super::mesh_ops::weld_boundary_vertices(m, deflection, f),
                            1 => super::super::mesh_ops::dedupe_coincident_triangles(m, f),
                            _ => {
                                super::super::mesh_ops::fill_sub_deflection_triangular_gaps(
                                    m, deflection, f,
                                );
                                true
                            }
                        };
                        let mut plain = input.clone();
                        let reported = if reference {
                            with_reference(|| {
                                call(&mut plain, None);
                                call(&mut m, Some(&mut f))
                            })
                        } else {
                            call(&mut plain, None);
                            call(&mut m, Some(&mut f))
                        };
                        (m, f, plain, reported)
                    };
                    let (fast, fast_faces, fast_plain, reported) = run(false);
                    let (slow, slow_faces, slow_plain, _) = run(true);
                    let context = format!("{context} pass {pass}");
                    // The fast passes' reports, which let the pipeline skip
                    // gap fill: weld says `false` only for a mesh without
                    // boundary half-edges (left untouched), dedupe says
                    // whether it removed anything.
                    match pass {
                        0 if !reported => {
                            assert_eq!(boundary_edge_count(&input), 0, "{context}");
                            assert_eq!(fast.indices, input.indices, "{context}");
                        }
                        1 => assert_eq!(
                            reported,
                            fast.indices.len() != input.indices.len(),
                            "{context}"
                        ),
                        _ => {}
                    }
                    assert_bit_identical(
                        &(fast, fast_faces.clone()),
                        &(slow, slow_faces),
                        &context,
                    );
                    assert_bit_identical(
                        &(fast_plain, Vec::new()),
                        &(slow_plain, Vec::new()),
                        &format!("{context} untracked"),
                    );
                    if fast_faces != input_faces || fast_faces.len() * 3 != input.indices.len() {
                        exercised[pass] += 1;
                    }
                }
            }
        }
    }
    // Every pass changed some of the damaged inputs (the comparison is not
    // vacuous).
    assert!(exercised.iter().all(|&n| n > 0), "{exercised:?}");
}

#[test]
fn passes_match_reference_on_degenerate_inputs() {
    let tri = |indices: Vec<u32>, n: usize| TriangleMesh {
        #[allow(clippy::cast_precision_loss)]
        positions: (0..n)
            .map(|i| Point3::new((i % 3) as f64, (i / 3) as f64, 0.0))
            .collect(),
        normals: vec![Vec3::new(0.0, 0.0, 1.0); n],
        indices,
    };
    let cases = [
        tri(vec![], 0),
        tri(vec![0, 1, 2], 3),
        tri(vec![0, 1, 2, 0, 2, 1], 3),
        tri(vec![0, 1, 2, 0, 1, 2, 0, 1, 2, 2, 1, 0, 2, 1, 0], 3),
        tri(vec![0, 0, 0, 1, 1, 1, 0, 1, 2], 3),
        tri(vec![0, 1, 3, 1, 4, 3, 1, 2, 4, 3, 4, 6, 4, 7, 6], 9),
    ];
    for (k, mesh) in cases.iter().enumerate() {
        for pass in 0..3 {
            let run = |reference: bool| {
                let mut m = mesh.clone();
                let mut f: Vec<u32> = (0..m.indices.len() as u32 / 3).collect();
                let go = |m: &mut TriangleMesh, f: &mut Vec<u32>| match pass {
                    0 => {
                        super::super::mesh_ops::weld_boundary_vertices(m, 0.5, Some(f));
                    }
                    1 => {
                        super::super::mesh_ops::dedupe_coincident_triangles(m, Some(f));
                    }
                    _ => {
                        super::super::mesh_ops::fill_sub_deflection_triangular_gaps(
                            m,
                            0.5,
                            Some(f),
                        );
                    }
                };
                if reference {
                    with_reference(|| go(&mut m, &mut f));
                } else {
                    go(&mut m, &mut f);
                }
                (m, f)
            };
            assert_bit_identical(&run(false), &run(true), &format!("case {k} pass {pass}"));
        }
    }
}
