//! Mesh validation and boundary operations.

use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use super::TriangleMesh;
use super::edge_sampling::sample_edge;

/// 1µm position-quantization grid for coincident-triangle dedupe: tight
/// enough that legitimately distinct CAD features (down to ~10µm geometry
/// like thin plates) keep separate keys, while still merging post-merge
/// floating-point noise in coincident vertices that boundary-vertex welding
/// didn't catch. Shared with the mesh-boolean output self-check so both
/// measure manifoldness on the same weld grid.
pub const COINCIDENT_DEDUPE_GRID: f64 = 1e-6;

/// Check if a mesh is a closed 2-manifold.
///
/// Returns `true` iff the mesh contains at least one triangle and every edge
/// is shared by exactly 2 triangles: no gaps (boundary edges), no branching
/// (non-manifold edges). Useful for
/// validating that `tessellate_solid` produces watertight meshes suitable
/// for slicers and downstream geometric operations.
#[must_use]
pub fn is_watertight(mesh: &TriangleMesh) -> bool {
    !mesh.indices.is_empty()
        && mesh.indices.len().is_multiple_of(3)
        && boundary_edge_count(mesh) == 0
        && non_manifold_edge_count(mesh) == 0
}

/// Count boundary (one-sided) edges in a mesh.
///
/// A boundary edge is one where the half-edge `(a, b)` exists but `(b, a)`
/// does not. Returns the number of such edges. A watertight mesh has 0.
#[must_use]
pub fn boundary_edge_count(mesh: &TriangleMesh) -> usize {
    let mut half_edges: DetHashSet<(u32, u32)> = DetHashSet::default();
    let tri_count = mesh.indices.len() / 3;

    for t in 0..tri_count {
        let i0 = mesh.indices[t * 3];
        let i1 = mesh.indices[t * 3 + 1];
        let i2 = mesh.indices[t * 3 + 2];
        half_edges.insert((i0, i1));
        half_edges.insert((i1, i2));
        half_edges.insert((i2, i0));
    }

    half_edges
        .iter()
        .filter(|&&(a, b)| !half_edges.contains(&(b, a)))
        .count()
}

/// Count non-manifold (branching) edges in a mesh.
///
/// An undirected edge `{a, b}` is non-manifold when 3 or more triangles
/// reference it. A 2-manifold mesh has 0 such edges. Distinct from
/// [`boundary_edge_count`], which counts 1-sided edges. Use both together
/// to validate that a tessellated solid is a closed 2-manifold.
#[must_use]
pub fn non_manifold_edge_count(mesh: &TriangleMesh) -> usize {
    let mut edge_count: DetHashMap<(u32, u32), u32> = DetHashMap::default();
    for tri in mesh.indices.chunks_exact(3) {
        let (a, b, c) = (tri[0], tri[1], tri[2]);
        for (p, q) in [(a, b), (b, c), (c, a)] {
            let key = if p < q { (p, q) } else { (q, p) };
            *edge_count.entry(key).or_default() += 1;
        }
    }

    edge_count.values().filter(|&&c| c > 2).count()
}

/// Position-welded mesh quality metrics.
///
/// Unlike the index-based [`boundary_edge_count`] / [`non_manifold_edge_count`],
/// these are computed after quantizing vertex positions to
/// the 1 µm coincident-dedupe grid, so position-duplicate vertices (distinct indices
/// at coincident coordinates) cannot mask a leak or fake a boundary.
#[derive(Debug, Clone, Copy)]
pub struct WeldedMeshQuality {
    /// Non-degenerate triangles retained after position welding.
    pub triangle_count: usize,
    /// Edges used by exactly one triangle (0 for a watertight mesh).
    pub boundary_edges: usize,
    /// Edges used by three or more triangles.
    pub non_manifold_edges: usize,
    /// Euler characteristic `V - E + F` of the welded mesh (2 for a single
    /// closed genus-0 shell).
    pub euler_characteristic: i64,
}

impl WeldedMeshQuality {
    /// True when the welded mesh is non-empty and has no boundary or
    /// non-manifold edges.
    #[must_use]
    pub const fn is_watertight(&self) -> bool {
        self.triangle_count > 0 && self.boundary_edges == 0 && self.non_manifold_edges == 0
    }
}

/// Compute position-welded boundary/non-manifold edge counts and the Euler
/// characteristic of a mesh.
///
/// Vertices are quantized to the coincident-dedupe grid (1 µm) so that
/// coincident-but-distinct indices weld together; degenerate (collapsed)
/// triangles are skipped.
#[must_use]
pub fn welded_mesh_quality(mesh: &TriangleMesh) -> WeldedMeshQuality {
    type Q = (i64, i64, i64);
    let s = 1.0 / COINCIDENT_DEDUPE_GRID;
    #[allow(clippy::cast_possible_truncation)]
    let q = |p: Point3| -> Q {
        (
            (p.x() * s).round() as i64,
            (p.y() * s).round() as i64,
            (p.z() * s).round() as i64,
        )
    };

    let mut verts: DetHashSet<Q> = DetHashSet::default();
    let mut edges: DetHashMap<(Q, Q), u32> = DetHashMap::default();
    let mut face_count: i64 = 0;
    for tri in mesh.indices.chunks_exact(3) {
        let Some((&pa, &pb, &pc)) = tri
            .first()
            .and_then(|&a| mesh.positions.get(a as usize))
            .zip(tri.get(1).and_then(|&b| mesh.positions.get(b as usize)))
            .zip(tri.get(2).and_then(|&c| mesh.positions.get(c as usize)))
            .map(|((a, b), c)| (a, b, c))
        else {
            return WeldedMeshQuality {
                triangle_count: 0,
                boundary_edges: usize::MAX,
                non_manifold_edges: usize::MAX,
                euler_characteristic: 0,
            };
        };
        let a = q(pa);
        let b = q(pb);
        let c = q(pc);
        if a == b || b == c || a == c {
            continue;
        }
        face_count += 1;
        for v in [a, b, c] {
            verts.insert(v);
        }
        for (p, r) in [(a, b), (b, c), (c, a)] {
            let key = if p <= r { (p, r) } else { (r, p) };
            *edges.entry(key).or_default() += 1;
        }
    }

    #[allow(clippy::cast_possible_wrap)]
    WeldedMeshQuality {
        triangle_count: face_count as usize,
        boundary_edges: edges.values().filter(|&&c| c == 1).count(),
        non_manifold_edges: edges.values().filter(|&&c| c > 2).count(),
        euler_characteristic: verts.len() as i64 - edges.len() as i64 + face_count,
    }
}

/// Remove duplicate triangles, cancelling opposing pairs and dedup same-winding pairs.
///
/// Workaround for issue #696: when a boolean leaves overlapping coplanar faces
/// in its output (the GFA path can do this without breaking Euler), tessellating
/// each face independently produces multiple triangles on the same 3D positions.
/// Slicers see this as branching (an edge shared by 3+ triangles), then "repair"
/// it by dropping pieces — turning hollow baseplates into solid blocks.
///
/// Triangles are keyed by their **quantized vertex positions** (sorted), not
/// global vertex IDs — boundary-vertex welding only runs on edges that already
/// look like boundaries, so two coplanar interior overlaps can survive with
/// distinct IDs at coincident positions. Pairs with matching winding
/// (sort-permutation parity equal) deduplicate to one triangle; pairs with
/// opposite winding cancel (both removed) — that's the signature of two faces
/// tessellated from opposite sides of the same plane.
/// `tri_faces` is a parallel tri -> face attribution array (one entry per
/// triangle); entries for removed triangles are filtered alongside so group
/// offsets recomputed from it stay aligned.
pub(super) fn dedupe_coincident_triangles(
    mesh: &mut TriangleMesh,
    tri_faces: Option<&mut Vec<u32>>,
) {
    #[cfg(test)]
    if super::tests::mesh_passes::use_reference_passes() {
        return super::tests::mesh_passes::reference::dedupe_coincident_triangles(mesh, tri_faces);
    }
    let tri_count = mesh.indices.len() / 3;
    if tri_count < 2 {
        return;
    }

    // PERF-D07: quantize each vertex once and give every distinct quantized
    // position a compact id, then group triangles by their sorted id
    // triples with a counting sort on the smallest id. This replaces one
    // 72-byte hash key (and one heap list) per triangle. The grouping is the
    // same: two triangles share a key exactly when their quantized corner
    // positions are equal as sets. Sorting ids instead of quantized tuples
    // may relabel which winding class of a group counts as "even", but the
    // removal rule below is symmetric in the two classes, and triangles
    // within a group stay in index order, so the kept set is unchanged.
    let Some(ids) = quantized_vertex_ids(mesh) else {
        return;
    };

    let mut keys: Vec<[u32; 3]> = Vec::with_capacity(tri_count);
    let mut parity: Vec<bool> = Vec::with_capacity(tri_count);
    let mut by_min = vec![0_u32; ids.distinct + 1];
    for tri in mesh.indices.chunks_exact(3) {
        let mut k = [
            ids.of[tri[0] as usize],
            ids.of[tri[1] as usize],
            ids.of[tri[2] as usize],
        ];
        // Sort ascending; track parity of the sort permutation.
        let mut parity_even = true;
        if k[0] > k[1] {
            k.swap(0, 1);
            parity_even = !parity_even;
        }
        if k[1] > k[2] {
            k.swap(1, 2);
            parity_even = !parity_even;
        }
        if k[0] > k[1] {
            k.swap(0, 1);
            parity_even = !parity_even;
        }
        // Degenerate triangles (collapsed to <3 distinct positions) never
        // take part.
        if k[0] == k[1] || k[1] == k[2] {
            k = [u32::MAX; 3];
        } else {
            by_min[k[0] as usize] += 1;
        }
        keys.push(k);
        parity.push(parity_even);
    }

    // Counting sort of the non-degenerate triangles by their smallest id;
    // each bucket keeps triangle index order.
    let mut start = Vec::with_capacity(by_min.len() + 1);
    let mut total = 0_u32;
    start.push(0);
    for count in &by_min {
        total += count;
        start.push(total);
    }
    let mut cursor = start.clone();
    let mut sorted = vec![0_u32; total as usize];
    for (t, k) in keys.iter().enumerate() {
        if k[0] != u32::MAX {
            let slot = &mut cursor[k[0] as usize];
            #[allow(clippy::cast_possible_truncation)]
            {
                sorted[*slot as usize] = t as u32;
            }
            *slot += 1;
        }
    }

    let mut keep = vec![true; tri_count];
    let mut done = vec![false; sorted.len()];
    let mut group: Vec<(usize, bool)> = Vec::new();
    for bucket in start.windows(2) {
        let (lo, hi) = (bucket[0] as usize, bucket[1] as usize);
        if hi - lo < 2 {
            continue;
        }
        for i in lo..hi {
            if done[i] {
                continue;
            }
            let ti = sorted[i] as usize;
            group.clear();
            group.push((ti, parity[ti]));
            for j in i + 1..hi {
                let tj = sorted[j] as usize;
                if !done[j] && keys[tj] == keys[ti] {
                    done[j] = true;
                    group.push((tj, parity[tj]));
                }
            }
            if group.len() >= 2 {
                resolve_coincident_group(&group, &mut keep);
            }
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

/// Removal rule for one group of position-coincident triangles (in index
/// order, with the parity of their corner sort): opposite windings cancel
/// pairwise (both removed), and of the survivors of each winding only the
/// first is kept. Symmetric in the two winding classes.
fn resolve_coincident_group(tris: &[(usize, bool)], keep: &mut [bool]) {
    let (even, odd): (Vec<_>, Vec<_>) = tris.iter().partition(|&&(_, p)| p);
    let cancel_pairs = even.len().min(odd.len());
    for &(t, _) in even.iter().take(cancel_pairs) {
        keep[t] = false;
    }
    for &(t, _) in odd.iter().take(cancel_pairs) {
        keep[t] = false;
    }
    for &(t, _) in even.iter().skip(cancel_pairs).skip(1) {
        keep[t] = false;
    }
    for &(t, _) in odd.iter().skip(cancel_pairs).skip(1) {
        keep[t] = false;
    }
}

/// Compact ids of the distinct quantized vertex positions (1 µm grid).
struct QuantizedIds {
    /// Id of every vertex's quantized position.
    of: Vec<u32>,
    /// Number of distinct ids.
    distinct: usize,
}

/// Quantize every vertex to the coincident-dedupe grid and number the
/// distinct cells; `None` when the vertex count does not fit the id type.
fn quantized_vertex_ids(mesh: &TriangleMesh) -> Option<QuantizedIds> {
    let s = 1.0 / COINCIDENT_DEDUPE_GRID;
    let mut cells: DetHashMap<(i64, i64, i64), u32> =
        DetHashMap::with_capacity_and_hasher(mesh.positions.len(), remus_math::det_hash::DetState);
    let mut of = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (p.x() * s).round() as i64,
            (p.y() * s).round() as i64,
            (p.z() * s).round() as i64,
        );
        // `u32::MAX` marks a degenerate triangle in the caller.
        let next = u32::try_from(cells.len()).ok().filter(|&n| n != u32::MAX)?;
        of.push(*cells.entry(key).or_insert(next));
    }
    Some(QuantizedIds {
        distinct: cells.len(),
        of,
    })
}

/// Edge polyline data for wireframe visualization.
///
/// Contains flattened position data for all edges in a solid, plus offsets
/// to identify where each edge's polyline starts.
#[derive(Debug, Clone, Default)]
pub struct EdgeLines {
    /// Vertex positions for all edge polylines (concatenated).
    pub positions: Vec<Point3>,
    /// Start index (in vertex count, not float count) of each edge polyline.
    /// The i-th edge's points are `positions[offsets[i]..offsets[i+1]]`
    /// (or `..positions.len()` for the last edge).
    pub offsets: Vec<usize>,
}

/// Check whether two face surfaces represent the same geometric surface.
fn surfaces_equivalent(a: &FaceSurface, b: &FaceSurface) -> bool {
    let tol = remus_math::tolerance::Tolerance::new();
    let lin = tol.linear;
    let ang = tol.angular;

    match (a, b) {
        (FaceSurface::Plane { normal: na, d: da }, FaceSurface::Plane { normal: nb, d: db }) => {
            let dot = na.dot(*nb);
            (dot.abs() - 1.0).abs() < ang && (da - db * dot.signum()).abs() < lin
        }
        (FaceSurface::Cylinder(ca), FaceSurface::Cylinder(cb)) => {
            (ca.radius() - cb.radius()).abs() < lin
                && ca.axis().dot(cb.axis()).abs() > 1.0 - ang
                && {
                    let d = cb.origin() - ca.origin();
                    let cross = d.cross(ca.axis());
                    cross.dot(cross) < lin * lin
                }
        }
        (FaceSurface::Cone(ca), FaceSurface::Cone(cb)) => {
            (ca.half_angle() - cb.half_angle()).abs() < ang
                && ca.axis().dot(cb.axis()).abs() > 1.0 - ang
                && {
                    let d = cb.apex() - ca.apex();
                    d.dot(d) < lin * lin
                }
        }
        (FaceSurface::Sphere(sa), FaceSurface::Sphere(sb)) => {
            (sa.radius() - sb.radius()).abs() < lin && {
                let d = sb.center() - sa.center();
                d.dot(d) < lin * lin
            }
        }
        (FaceSurface::Torus(ta), FaceSurface::Torus(tb)) => {
            (ta.major_radius() - tb.major_radius()).abs() < lin
                && (ta.minor_radius() - tb.minor_radius()).abs() < lin
                && ta.z_axis().dot(tb.z_axis()).abs() > 1.0 - ang
                && {
                    let d = tb.center() - ta.center();
                    d.dot(d) < lin * lin
                }
        }
        (FaceSurface::Nurbs(_), FaceSurface::Nurbs(_)) => false,
        _ => false,
    }
}

/// Sample all edges of a solid into polylines for wireframe rendering.
///
/// Each edge is sampled according to the given `deflection` tolerance.
/// Returns [`EdgeLines`] containing the polyline data for all unique edges.
///
/// # Errors
///
/// Returns an error if topology traversal or edge sampling fails.
pub fn sample_solid_edges(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<EdgeLines, crate::OperationsError> {
    sample_solid_edges_filtered(
        topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
        true,
    )
}

/// Sample edges of a solid, optionally filtering out smooth (co-surface) edges.
///
/// When `filter_smooth` is `true`, edges shared by two faces on the same
/// underlying geometric surface are omitted. These edges arise from boolean
/// face-splitting and add wireframe clutter without representing visible creases.
///
/// `angular_tol` caps the per-segment turn angle when discretizing curved
/// edges (circles, ellipses, NURBS); a smaller value yields smoother polylines.
/// Pass [`remus_math::chord::DEFAULT_ANGULAR_TOL`] for the historical default.
///
/// # Errors
///
/// Returns an error if topology traversal or edge sampling fails.
pub fn sample_solid_edges_filtered(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angular_tol: f64,
    filter_smooth: bool,
) -> Result<EdgeLines, crate::OperationsError> {
    let edges = remus_topology::explorer::solid_edges(topo, solid)?;

    let edge_face_map = if filter_smooth {
        Some(remus_topology::explorer::edge_to_face_map(topo, solid)?)
    } else {
        None
    };

    let mut result = EdgeLines {
        positions: Vec::new(),
        offsets: Vec::with_capacity(edges.len()),
    };

    for edge_id in &edges {
        if let Some(ref efm) = edge_face_map
            && let Some(faces) = efm.get(&edge_id.index())
            && faces.len() == 2
        {
            let fa = topo.face(faces[0])?;
            let fb = topo.face(faces[1])?;
            if surfaces_equivalent(fa.surface(), fb.surface()) {
                continue;
            }
        }

        result.offsets.push(result.positions.len());
        let edge = topo.edge(*edge_id)?;
        let points = sample_edge(topo, edge, deflection, angular_tol, false)?;
        result.positions.extend(points);
    }

    Ok(result)
}

/// Weld remaining boundary vertices by merging coincident positions.
///
/// Uses union-find over a spatial hash grid to merge boundary vertices that
/// are within `weld_tol` of each other. Rewrites triangle indices and removes
/// degenerate triangles (where merged indices create duplicate vertices).
/// `tri_faces` is the parallel tri -> face attribution array; entries for
/// removed degenerate triangles are filtered alongside.
pub(super) fn weld_boundary_vertices(
    mesh: &mut TriangleMesh,
    deflection: f64,
    tri_faces: Option<&mut Vec<u32>>,
) {
    #[cfg(test)]
    if super::tests::mesh_passes::use_reference_passes() {
        return super::tests::mesh_passes::reference::weld_boundary_vertices(
            mesh, deflection, tri_faces,
        );
    }
    let n_verts = mesh.positions.len();
    if n_verts == 0 || mesh.indices.is_empty() {
        return;
    }

    // Boundary vertices: incident on half-edges without a matching reverse.
    // PERF-D07: found through a vertex-indexed half-edge table instead of a
    // hash map of every half-edge; the set is the same.
    let Some(table) = HalfEdgeTable::new(&mesh.indices, n_verts) else {
        return;
    };
    let mut is_boundary = vec![false; n_verts];
    let mut any_boundary = false;
    for (a, b, _) in table.edges() {
        if !table.contains(b, a) {
            is_boundary[a as usize] = true;
            is_boundary[b as usize] = true;
            any_boundary = true;
        }
    }

    if !any_boundary {
        return;
    }

    // Sorted iteration keeps grid-cell contents and union order independent
    // of DetHashSet iteration order, so welded meshes are reproducible.
    #[allow(clippy::cast_possible_truncation)]
    let boundary_verts: Vec<u32> = (0..n_verts)
        .filter(|&v| is_boundary[v])
        .map(|v| v as u32)
        .collect();

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

/// Close a three-edge tessellation gap that is smaller than the requested
/// display resolution.
///
/// Tangential analytic intersections can leave three independently sampled
/// face triangles around a narrow cusp. When the whole gap is bounded to a
/// few deflection lengths and its shortest altitude is sub-deflection, adding
/// the missing oppositely oriented triangle restores the closed projection
/// without changing the exact B-rep.
pub(super) fn fill_sub_deflection_triangular_gaps(
    mesh: &mut TriangleMesh,
    deflection: f64,
    mut tri_faces: Option<&mut Vec<u32>>,
) {
    #[cfg(test)]
    if super::tests::mesh_passes::use_reference_passes() {
        return super::tests::mesh_passes::reference::fill_sub_deflection_triangular_gaps(
            mesh, deflection, tri_faces,
        );
    }
    if !deflection.is_finite() || deflection <= 0.0 || mesh.indices.len() < 9 {
        return;
    }

    // PERF-D07: boundary half-edges (with the first triangle using each) via
    // a vertex-indexed table instead of a hash map of every half-edge.
    let Some(table) = HalfEdgeTable::new(&mesh.indices, mesh.positions.len()) else {
        return;
    };
    let mut boundary: Vec<(u32, u32, usize)> = Vec::new();
    for (a, b, triangle) in table.edges() {
        if !table.contains(b, a) && table.first_use(a, b) == Some(triangle) {
            boundary.push((a, b, triangle));
        }
    }
    if boundary.is_empty() {
        return;
    }
    boundary.sort_unstable();
    let boundary_map: DetHashMap<(u32, u32), usize> = boundary
        .iter()
        .map(|&(a, b, triangle)| ((a, b), triangle))
        .collect();
    // The candidate scan below visits boundary half-edges in `boundary_map`
    // order; index them by their start vertex once, keeping that order, so
    // each lookup visits only the half-edges leaving `b`.
    let mut leaving: DetHashMap<u32, Vec<(u32, usize)>> = DetHashMap::default();
    for (&(from, to), &triangle) in &boundary_map {
        leaving.entry(from).or_default().push((to, triangle));
    }

    let face_by_triangle = tri_faces.as_deref();
    let mut fillers = Vec::<([u32; 3], u32)>::new();
    let mut seen = DetHashSet::<[u32; 3]>::default();
    for &(a, b, ab_triangle) in &boundary {
        for &(c, bc_triangle) in leaving.get(&b).into_iter().flatten() {
            if c == a {
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

/// Directed half-edges of a triangle list, indexed by start vertex
/// (compressed rows), each with the triangle that uses it. Rows keep
/// triangle order. Replaces hash maps keyed by every half-edge in the
/// whole-mesh passes (PERF-D07).
struct HalfEdgeTable {
    /// Row start per vertex (`n + 1` entries).
    start: Vec<u32>,
    /// `(end vertex, triangle)` per half-edge, grouped by start vertex.
    entries: Vec<(u32, u32)>,
}

impl HalfEdgeTable {
    /// `None` when an index is out of range or the counts overflow `u32`.
    fn new(indices: &[u32], n_verts: usize) -> Option<Self> {
        let tri_count = indices.len() / 3;
        let total = u32::try_from(tri_count.checked_mul(3)?).ok()?;
        let mut start = vec![0_u32; n_verts + 1];
        for &v in &indices[..tri_count * 3] {
            *start.get_mut(v as usize + 1)? += 1;
        }
        for i in 0..n_verts {
            start[i + 1] += start[i];
        }
        debug_assert_eq!(start[n_verts], total);
        let mut cursor = start.clone();
        let mut entries = vec![(0_u32, 0_u32); total as usize];
        for (t, tri) in indices.chunks_exact(3).enumerate() {
            let t = u32::try_from(t).ok()?;
            for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let slot = &mut cursor[a as usize];
                entries[*slot as usize] = (b, t);
                *slot += 1;
            }
        }
        Some(Self { start, entries })
    }

    fn row(&self, a: u32) -> &[(u32, u32)] {
        let a = a as usize;
        &self.entries[self.start[a] as usize..self.start[a + 1] as usize]
    }

    /// Whether some triangle uses the half-edge `a -> b`.
    fn contains(&self, a: u32, b: u32) -> bool {
        self.row(a).iter().any(|&(to, _)| to == b)
    }

    /// The first triangle (lowest index) using `a -> b`.
    fn first_use(&self, a: u32, b: u32) -> Option<usize> {
        self.row(a)
            .iter()
            .find(|&&(to, _)| to == b)
            .map(|&(_, t)| t as usize)
    }

    /// Every half-edge as `(start, end, triangle)`, by start vertex.
    fn edges(&self) -> impl Iterator<Item = (u32, u32, usize)> + '_ {
        self.start.windows(2).enumerate().flat_map(move |(a, w)| {
            #[allow(clippy::cast_possible_truncation)]
            let a = a as u32;
            self.entries[w[0] as usize..w[1] as usize]
                .iter()
                .map(move |&(b, t)| (a, b, t as usize))
        })
    }
}
