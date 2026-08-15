//! Mesh validation and boundary operations.

use brepkit_math::det_hash::{DetHashMap, DetHashSet};
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::face::FaceSurface;
use brepkit_topology::solid::SolidId;

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
/// Returns `true` iff every edge is shared by exactly 2 triangles: no gaps
/// (boundary edges), no branching (non-manifold edges). Useful for
/// validating that `tessellate_solid` produces watertight meshes suitable
/// for slicers and downstream geometric operations.
#[must_use]
pub fn is_watertight(mesh: &TriangleMesh) -> bool {
    boundary_edge_count(mesh) == 0 && non_manifold_edge_count(mesh) == 0
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
    /// Edges used by exactly one triangle (0 for a watertight mesh).
    pub boundary_edges: usize,
    /// Edges used by three or more triangles.
    pub non_manifold_edges: usize,
    /// Euler characteristic `V - E + F` of the welded mesh (2 for a single
    /// closed genus-0 shell).
    pub euler_characteristic: i64,
}

impl WeldedMeshQuality {
    /// True when the welded mesh has no boundary and no non-manifold edges.
    #[must_use]
    pub const fn is_watertight(&self) -> bool {
        self.boundary_edges == 0 && self.non_manifold_edges == 0
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
    let tol = brepkit_math::tolerance::Tolerance::new();
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
        brepkit_math::chord::DEFAULT_ANGULAR_TOL,
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
/// Pass [`brepkit_math::chord::DEFAULT_ANGULAR_TOL`] for the historical default.
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
    let edges = brepkit_topology::explorer::solid_edges(topo, solid)?;

    let edge_face_map = if filter_smooth {
        Some(brepkit_topology::explorer::edge_to_face_map(topo, solid)?)
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
