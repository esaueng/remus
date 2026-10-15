//! STL import: convert triangle meshes into B-Rep topology.
//!
//! Takes a [`TriangleMesh`] (from [`read_stl`](super::reader::read_stl))
//! and builds topology entities: one planar face per triangle, assembled
//! into a shell and solid.

use std::collections::HashMap;

use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::IoError;

/// Import a [`TriangleMesh`] into topology as a single solid.
///
/// Each triangle becomes a planar face. Vertices at the same position
/// (within `tolerance`) are merged. The resulting faces are assembled
/// into a closed shell and solid. A closed, coherently wound mesh is
/// oriented outward by the sign of its enclosed volume, whatever its stored
/// normals say.
///
/// # Errors
///
/// Returns [`IoError`] if:
/// - The mesh has no triangles
/// - Three or more triangles share an edge after vertex welding
/// - Wire or shell construction fails
pub fn import_mesh(
    topo: &mut Topology,
    mesh: &TriangleMesh,
    tolerance: f64,
) -> Result<SolidId, IoError> {
    validate_mesh(mesh, tolerance)?;

    let vertex_ids = build_vertex_map(topo, &mesh.positions, tolerance);

    // Stored per-vertex normals (e.g. STL facet normals) may correct the
    // winding of individual triangles. They cannot decide the orientation of
    // the whole body: every reader except STL synthesises its normals from
    // the winding, and an STL facet normal may be zero, so an inward-wound
    // mesh would always agree with its own normals.
    let has_normals = mesh.normals.len() >= mesh.positions.len();
    let mut triangles: Vec<[usize; 3]> = Vec::with_capacity(mesh.indices.len() / 3);
    for tri in mesh.indices.chunks_exact(3) {
        let (i0, mut i1, mut i2) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);

        // Skip degenerate triangles (two or more coincident vertices).
        let (v0, v1, v2) = (vertex_ids[i0], vertex_ids[i1], vertex_ids[i2]);
        if v0 == v1 || v1 == v2 || v0 == v2 {
            continue;
        }

        if has_normals {
            let p0 = mesh.positions[i0];
            let geo_normal = (mesh.positions[i1] - p0).cross(mesh.positions[i2] - p0);
            if geo_normal.dot(mesh.normals[i0]) < 0.0 {
                std::mem::swap(&mut i1, &mut i2);
            }
        }
        triangles.push([i0, i1, i2]);
    }
    let closed = validate_triangle_edge_incidence(&vertex_ids, &triangles)?;

    // The orientation of the body comes from geometry. A closed, coherently
    // wound component encloses positive signed volume exactly when it faces
    // outward, so a closed mesh whose every component is inward is flipped.
    // Mixed signs may be a correctly wound cavity or a partly inverted
    // multi-body file, which one flip cannot tell apart; those, and open
    // meshes, keep the previous rule: no global flip with stored normals, the
    // origin-relative signed volume without.
    let inward = closed.then(|| {
        closed_components_inward(
            &welded_positions(&mesh.positions, &vertex_ids),
            &vertex_ids,
            &triangles,
        )
    });
    let flip_all = match inward.flatten() {
        Some(inward) => inward,
        None if has_normals => false,
        None => signed_six_volume(&mesh.positions, &triangles, Point3::new(0.0, 0.0, 0.0)) < 0.0,
    };

    let mut face_ids = Vec::new();
    // Edges are shared between the two triangles that meet along them. Giving
    // each triangle its own three edges would leave every face an island: the
    // shell would have no adjacency at all and every edge would read as free.
    let mut edge_map: HashMap<(usize, usize), EdgeId> = HashMap::new();
    for &[i0, i1, i2] in &triangles {
        let (i1, i2) = if flip_all { (i2, i1) } else { (i1, i2) };
        let face_id = build_triangle_face(
            topo,
            &mut edge_map,
            vertex_ids[i0],
            vertex_ids[i1],
            vertex_ids[i2],
        )?;
        face_ids.push(face_id);
    }

    if face_ids.is_empty() {
        return Err(IoError::InvalidTopology {
            reason: "no valid triangles in mesh".to_string(),
        });
    }

    let shell = Shell::new(face_ids).map_err(|e| IoError::ParseError {
        reason: format!("failed to build shell from mesh: {e}"),
    })?;
    let shell_id = topo.add_shell(shell);
    let solid_id = topo.add_solid(Solid::new(shell_id, Vec::new()));

    Ok(solid_id)
}

/// Six times the signed volume the triangles enclose, summed as tetrahedra
/// about `reference`.
fn signed_six_volume(positions: &[Point3], triangles: &[[usize; 3]], reference: Point3) -> f64 {
    triangles
        .iter()
        .map(|&[a, b, c]| {
            let (a, b, c) = (
                positions[a] - reference,
                positions[b] - reference,
                positions[c] - reference,
            );
            a.dot(b.cross(c))
        })
        .sum()
}

/// Each position replaced by the position of the vertex it was welded to,
/// the first position that created that vertex. The orientation test then
/// measures the geometry the faces are built from: sub-tolerance cracks
/// between repeated copies (as in STL) do not enter the volume.
fn welded_positions(positions: &[Point3], vertex_ids: &[VertexId]) -> Vec<Point3> {
    let mut first: HashMap<usize, Point3> = HashMap::new();
    positions
        .iter()
        .zip(vertex_ids)
        .map(|(&position, id)| *first.entry(id.index()).or_insert(position))
        .collect()
}

/// `Some(true)` when every connected component of a closed, coherently wound
/// mesh encloses negative signed volume, `Some(false)` when every one encloses
/// positive volume, and `None` when the signs are mixed or one is zero.
///
/// Components are joined across shared edges, not shared vertices: two shells
/// that only touch at a corner are separate bodies with their own orientation.
/// Each component is closed, since both triangles on each of its edges belong
/// to it, so its signed volume decides its orientation.
///
/// Each component's tetrahedra are summed about one of its own vertices:
/// about a far point, such as the world origin or the centre of a box that
/// includes unused or distant vertices, the rounding error grows with the
/// cube of the distance and can swamp the component's volume (see B56).
fn closed_components_inward(
    positions: &[Point3],
    vertex_ids: &[VertexId],
    triangles: &[[usize; 3]],
) -> Option<bool> {
    // Union-find over triangles, joining the two triangles on each edge.
    let mut parent: Vec<usize> = (0..triangles.len()).collect();
    let mut first_on_edge: HashMap<(usize, usize), usize> = HashMap::new();
    for (index, tri) in triangles.iter().enumerate() {
        let [a, b, c] = tri.map(|i| vertex_ids[i]);
        for (start, end) in [(a, b), (b, c), (c, a)] {
            match first_on_edge.entry(edge_key(start, end)) {
                std::collections::hash_map::Entry::Occupied(other) => {
                    let (root, other) = (
                        find_root(&mut parent, index),
                        find_root(&mut parent, *other.get()),
                    );
                    parent[other] = root;
                }
                std::collections::hash_map::Entry::Vacant(slot) => {
                    slot.insert(index);
                }
            }
        }
    }

    // Per component: its reference vertex and six times its signed volume.
    let mut components: HashMap<usize, (Point3, f64)> = HashMap::new();
    for (index, tri) in triangles.iter().enumerate() {
        let (reference, volume) = components
            .entry(find_root(&mut parent, index))
            .or_insert((positions[tri[0]], 0.0));
        *volume += signed_six_volume(positions, std::slice::from_ref(tri), *reference);
    }
    let mut volumes = components.values().map(|&(_, volume)| volume);
    let first = volumes.next()?;
    let inward = first < 0.0;
    (first != 0.0 && volumes.all(|volume| volume != 0.0 && (volume < 0.0) == inward))
        .then_some(inward)
}

/// Union-find root of `node`, halving the path on the way.
fn find_root(parent: &mut [usize], mut node: usize) -> usize {
    while parent[node] != node {
        parent[node] = parent[parent[node]];
        node = parent[node];
    }
    node
}

/// Reject triangle edges incident to more than two non-degenerate faces, and
/// report whether the mesh is closed and coherently wound: every welded edge
/// used by exactly two triangles, once in each direction. Only then is the
/// signed volume independent of the reference point, so only then does its
/// sign say which side is inside.
///
/// Incidence is counted after tolerance welding because file formats such as
/// STL repeat each triangle's positions instead of sharing vertex indices.
/// This runs before allocating any edges, faces, shells, or solids, so an
/// over-shared mesh cannot become invalid B-Rep topology. `triangles` holds
/// only the non-degenerate triangles `import_mesh` will build, so skipped
/// ones contribute no phantom incidences.
fn validate_triangle_edge_incidence(
    vertex_ids: &[VertexId],
    triangles: &[[usize; 3]],
) -> Result<bool, IoError> {
    // Per edge: incident faces, and uses along minus against its key order.
    let mut edge_incidence: HashMap<(usize, usize), (usize, isize)> = HashMap::new();

    for tri in triangles {
        let [v0, v1, v2] = tri.map(|i| vertex_ids[i]);
        for (a, b) in [(v0, v1), (v1, v2), (v2, v0)] {
            let (incident_faces, net) = edge_incidence.entry(edge_key(a, b)).or_default();
            *incident_faces += 1;
            *net += if a.index() < b.index() { 1 } else { -1 };
        }
    }

    if let Some(((start, end), (incident_faces, _))) = edge_incidence
        .iter()
        .filter(|(_, (incident_faces, _))| *incident_faces > 2)
        .min_by_key(|(edge, _)| **edge)
    {
        return Err(IoError::InvalidTopology {
            reason: format!(
                "non-manifold mesh edge between welded vertices {start} and {end} has \
                 {incident_faces} incident faces (maximum is 2)"
            ),
        });
    }

    Ok(!edge_incidence.is_empty()
        && edge_incidence
            .values()
            .all(|&(incident_faces, net)| incident_faces == 2 && net == 0))
}

/// Validate mesh data before allocating any topology entities.
fn validate_mesh(mesh: &TriangleMesh, tolerance: f64) -> Result<(), IoError> {
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return Err(IoError::InvalidTopology {
            reason: "mesh import tolerance must be finite and positive".to_string(),
        });
    }

    if mesh.indices.len() < 3 {
        return Err(IoError::InvalidTopology {
            reason: "mesh has no triangles".to_string(),
        });
    }
    if !mesh.indices.len().is_multiple_of(3) {
        return Err(IoError::InvalidTopology {
            reason: "mesh index count is not divisible by three".to_string(),
        });
    }

    for (index, position) in mesh.positions.iter().enumerate() {
        if !position.x().is_finite() || !position.y().is_finite() || !position.z().is_finite() {
            return Err(IoError::InvalidTopology {
                reason: format!("mesh position {index} is not finite"),
            });
        }
    }
    for (index, normal) in mesh.normals.iter().enumerate() {
        if !normal.x().is_finite() || !normal.y().is_finite() || !normal.z().is_finite() {
            return Err(IoError::InvalidTopology {
                reason: format!("mesh normal {index} is not finite"),
            });
        }
    }
    for (index, &vertex_index) in mesh.indices.iter().enumerate() {
        if (vertex_index as usize) >= mesh.positions.len() {
            return Err(IoError::InvalidTopology {
                reason: format!(
                    "mesh index {index} references vertex {vertex_index}, but the mesh has {} vertices",
                    mesh.positions.len()
                ),
            });
        }
    }

    Ok(())
}

/// Build vertex IDs, merging coincident positions.
fn build_vertex_map(topo: &mut Topology, positions: &[Point3], tolerance: f64) -> Vec<VertexId> {
    weld_positions(positions, tolerance, |pos| {
        topo.add_vertex(Vertex::new(pos, tolerance))
    })
    .0
}

/// Weld `positions` within `tolerance`, calling `add_vertex` for each new
/// vertex. Returns the vertex of every position and the number of candidate
/// distance checks made, which the regression tests bound.
///
/// Candidates are found through a uniform spatial hash, so a scan is bounded
/// by the 27 cells around a point rather than by every vertex seen so far. A
/// mesh scan arrives with almost every position distinct, which is the
/// linear search's worst case: it made import quadratic in vertex count.
///
/// Cell membership alone never decides a merge — two points in one cell can
/// still be up to a diagonal apart, and two points a hair either side of a
/// cell boundary are neighbours. Every candidate is distance-checked, and the
/// 27-cell probe is what stops a boundary-straddling pair from being missed.
fn weld_positions(
    positions: &[Point3],
    tolerance: f64,
    mut add_vertex: impl FnMut(Point3) -> VertexId,
) -> (Vec<VertexId>, usize) {
    let tol_sq = tolerance * tolerance;
    let grid = WeldGrid::new(tolerance);
    let mut buckets: HashMap<[u64; 3], Vec<(Point3, VertexId)>> = HashMap::new();
    let mut map = Vec::with_capacity(positions.len());
    let mut checks = 0_usize;

    for &pos in positions {
        let cells = [grid.axis(pos.x()), grid.axis(pos.y()), grid.axis(pos.z())];

        // Among every candidate within tolerance, take the earliest-created
        // vertex. Insertion order decided the winner in the linear scan this
        // replaces, and vertex ids are allocated in order, so picking the
        // smallest id keeps welding deterministic and order-independent.
        let mut best: Option<VertexId> = None;
        for &dx in cells[0].offsets() {
            for &dy in cells[1].offsets() {
                for &dz in cells[2].offsets() {
                    let key = [cells[0].key(dx), cells[1].key(dy), cells[2].key(dz)];
                    let Some(bucket) = buckets.get(&key) else {
                        continue;
                    };
                    checks += bucket.len();
                    for &(p, vid) in bucket {
                        let ex = p.x() - pos.x();
                        let ey = p.y() - pos.y();
                        let ez = p.z() - pos.z();
                        if ex.mul_add(ex, ey.mul_add(ey, ez * ez)) < tol_sq
                            && best.is_none_or(|b| vid < b)
                        {
                            best = Some(vid);
                        }
                    }
                }
            }
        }

        if let Some(vid) = best {
            map.push(vid);
        } else {
            let vid = add_vertex(pos);
            let key = [cells[0].key(0.0), cells[1].key(0.0), cells[2].key(0.0)];
            buckets.entry(key).or_default().push((pos, vid));
            map.push(vid);
        }
    }

    (map, checks)
}

/// The weld hash's cell geometry.
///
/// Keying by `(coordinate / tolerance).floor() as i64` saturated once the
/// ratio passed 2^63 (x = 1e12 at the 1e-7 default), so every vertex of a
/// far-placed part shared one bucket and the weld went quadratic. Cells are
/// keyed by the bits of the floored `f64` index instead, which never
/// saturates, and the cell edge is the smallest power of two at least twice
/// the tolerance, so the index is the coordinate scaled exactly: a pair
/// within tolerance is under half a cell apart and its indices differ by at
/// most one, with no rounding to widen that. Keys depend only on the point
/// itself, so an outlier vertex cannot coarsen the cells of the rest.
struct WeldGrid {
    /// `1 / cell` as two exact power-of-two factors, so neither overflows
    /// when the tolerance is subnormal.
    scale: [f64; 2],
}

/// One axis of a point's cell.
#[derive(Clone, Copy)]
enum AxisCell {
    /// The floored, exactly scaled index, below 2^53 in magnitude: integer
    /// neighbours are exact, and a partner within tolerance is at most one
    /// cell away.
    Index(f64),
    /// The index reached 2^53 (or overflowed): the coordinate's float
    /// spacing is already more than twice the tolerance, so only an
    /// identical coordinate is within tolerance and the cell needs no
    /// neighbours. The key is the coordinate's own bits.
    Exact(u64),
}

impl WeldGrid {
    fn new(tolerance: f64) -> Self {
        // Smallest k with 2^k >= 2 * tol, clamped to the f64 exponent range.
        let target = 2.0 * tolerance;
        #[allow(clippy::cast_possible_truncation)]
        let mut k = target.log2().ceil().clamp(-1074.0, 1023.0) as i32;
        while k < 1023 && pow2(k) < target {
            k += 1;
        }
        while k > -1074 && pow2(k - 1) >= target {
            k -= 1;
        }
        let first = -k / 2;
        Self {
            scale: [pow2(first), pow2(-k - first)],
        }
    }

    fn axis(&self, coordinate: f64) -> AxisCell {
        const EXACT_LIMIT: f64 = 9_007_199_254_740_992.0; // 2^53
        let scaled = coordinate * self.scale[0] * self.scale[1];
        if scaled.is_finite() && scaled.abs() < EXACT_LIMIT {
            AxisCell::Index(scaled.floor())
        } else {
            // `+ 0.0` folds -0.0 onto 0.0 so equal coordinates share a key.
            AxisCell::Exact((coordinate + 0.0).to_bits())
        }
    }
}

/// Exactly 2^k for k in [-1074, 1023], subnormals included.
fn pow2(k: i32) -> f64 {
    if k >= -1022 {
        f64::from_bits(u64::from(k.wrapping_add(1023).unsigned_abs()) << 52)
    } else {
        f64::from_bits(1_u64 << k.wrapping_add(1074).unsigned_abs())
    }
}

impl AxisCell {
    fn offsets(self) -> &'static [f64] {
        match self {
            Self::Index(_) => &[-1.0, 0.0, 1.0],
            Self::Exact(_) => &[0.0],
        }
    }

    fn key(self, offset: f64) -> u64 {
        match self {
            Self::Index(index) => (index + offset + 0.0).to_bits(),
            Self::Exact(bits) => bits,
        }
    }
}

/// Return the shared edge between `a` and `b`, creating it on first use.
///
/// The key is the unordered vertex pair, so the two triangles that meet along
/// an edge resolve to one [`EdgeId`]; the returned flag says whether this use
/// traverses it in its natural direction.
fn shared_edge(
    topo: &mut Topology,
    edge_map: &mut HashMap<(usize, usize), EdgeId>,
    a: VertexId,
    b: VertexId,
) -> (EdgeId, bool) {
    let key = edge_key(a, b);
    if let Some(&edge_id) = edge_map.get(&key) {
        // Natural direction is whichever pair created it.
        let forward = topo.edge(edge_id).is_ok_and(|e| e.start() == a);
        return (edge_id, forward);
    }
    let edge_id = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
    edge_map.insert(key, edge_id);
    (edge_id, true)
}

fn edge_key(a: VertexId, b: VertexId) -> (usize, usize) {
    if a.index() <= b.index() {
        (a.index(), b.index())
    } else {
        (b.index(), a.index())
    }
}

/// Build a single triangular planar face from three vertex IDs, reusing the
/// edges already created for neighbouring triangles.
fn build_triangle_face(
    topo: &mut Topology,
    edge_map: &mut HashMap<(usize, usize), EdgeId>,
    v0: VertexId,
    v1: VertexId,
    v2: VertexId,
) -> Result<remus_topology::face::FaceId, IoError> {
    let (e01, f01) = shared_edge(topo, edge_map, v0, v1);
    let (e12, f12) = shared_edge(topo, edge_map, v1, v2);
    let (e20, f20) = shared_edge(topo, edge_map, v2, v0);

    let oriented = vec![
        OrientedEdge::new(e01, f01),
        OrientedEdge::new(e12, f12),
        OrientedEdge::new(e20, f20),
    ];
    let wire = Wire::new(oriented, true).map_err(|e| IoError::ParseError {
        reason: format!("failed to build triangle wire: {e}"),
    })?;
    let wire_id = topo.add_wire(wire);

    let p0 = topo.vertex(v0).map_err(topo_err)?.point();
    let p1 = topo.vertex(v1).map_err(topo_err)?.point();
    let p2 = topo.vertex(v2).map_err(topo_err)?.point();

    let edge1 = p1 - p0;
    let edge2 = p2 - p0;
    let normal = edge1
        .cross(edge2)
        .normalize()
        .unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    let d = normal.dot(Vec3::new(p0.x(), p0.y(), p0.z()));

    let surface = FaceSurface::Plane { normal, d };
    let face_id = topo.add_face(Face::new(wire_id, Vec::new(), surface));

    Ok(face_id)
}

/// Convert a [`TopologyError`] into an [`IoError`].
fn topo_err(e: remus_topology::TopologyError) -> IoError {
    IoError::Operations(remus_operations::OperationsError::from(e))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stderr)]

    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_cube_non_manifold;

    use super::*;
    use crate::stl::reader::read_stl;

    /// The orientation test measures welded positions, not the raw copies:
    /// here the raw copies of one corner sit far enough apart to turn an
    /// outward tetrahedron's volume negative, but all weld to one vertex.
    #[test]
    fn orientation_uses_welded_positions() {
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        // Outward tetrahedron; the slanted face uses a second copy of the
        // apex, pushed below the base, that still welds to the apex.
        let triangles = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 4]];
        let mut positions = corners.to_vec();
        positions.push(Point3::new(0.0, 0.0, -10.0));
        let mut topo = Topology::new();
        let ids: Vec<VertexId> = corners
            .iter()
            .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let vertex_ids = [ids[0], ids[1], ids[2], ids[3], ids[3]];
        assert!(
            closed_components_inward(&positions, &vertex_ids, &triangles) != Some(false),
            "raw copies alone must not read as outward for this test to matter"
        );
        assert_eq!(
            closed_components_inward(
                &welded_positions(&positions, &vertex_ids),
                &vertex_ids,
                &triangles
            ),
            Some(false)
        );
    }
    /// The weld's definition, quadratic: each position takes the
    /// earliest-created vertex strictly within `tolerance`, else creates one.
    fn reference_weld(positions: &[Point3], tolerance: f64) -> Vec<usize> {
        let tol_sq = tolerance * tolerance;
        let mut created: Vec<Point3> = Vec::new();
        positions
            .iter()
            .map(|&pos| {
                let near = created.iter().position(|p| {
                    let (ex, ey, ez) = (p.x() - pos.x(), p.y() - pos.y(), p.z() - pos.z());
                    ex.mul_add(ex, ey.mul_add(ey, ez * ez)) < tol_sq
                });
                near.unwrap_or_else(|| {
                    created.push(pos);
                    created.len() - 1
                })
            })
            .collect()
    }

    fn grid_weld(positions: &[Point3], tolerance: f64) -> Vec<usize> {
        let mut topo = Topology::new();
        let ids = build_vertex_map(&mut topo, positions, tolerance);
        let mut order: HashMap<VertexId, usize> = HashMap::new();
        ids.into_iter()
            .map(|id| {
                let next = order.len();
                *order.entry(id).or_insert(next)
            })
            .collect()
    }

    /// Coordinates clustered within a few tolerances (or a few ulps) of
    /// `base`, which ranges over the origin, both sides of the near-field
    /// limit, and far coordinates of either sign.
    fn clustered(base: f64, tolerance: f64, steps: &[(i8, i8, i8)], ulps: bool) -> Vec<Point3> {
        let along = |b: f64, k: i8| {
            if ulps {
                let mut v = b;
                for _ in 0..k.unsigned_abs() {
                    v = if k > 0 { v.next_up() } else { v.next_down() };
                }
                v
            } else {
                b + f64::from(k) * 0.35 * tolerance
            }
        };
        steps
            .iter()
            .map(|&(i, j, k)| Point3::new(along(base, i), along(-base, j), along(base, k)))
            .collect()
    }

    proptest::proptest! {
        #[test]
        fn grid_weld_matches_the_quadratic_definition_at_every_magnitude(
            base_index in 0_usize..8,
            tolerance_exp in -12_i32..2,
            ulps in proptest::bool::ANY,
            steps in proptest::collection::vec((-6_i8..6, -6_i8..6, -6_i8..6), 1..60),
        ) {
            let tolerance = 10_f64.powi(tolerance_exp);
            let limit = tolerance * 1_125_899_906_842_624.0;
            let base = [
                0.0,
                1.0,
                limit,
                limit.next_down(),
                limit - 0.6 * tolerance,
                -limit,
                1e12,
                -3e300,
            ][base_index];
            let positions = clustered(base, tolerance, &steps, ulps);
            proptest::prop_assert_eq!(
                grid_weld(&positions, tolerance),
                reference_weld(&positions, tolerance)
            );
        }
    }

    use crate::stl::writer::{self, StlFormat};

    #[test]
    fn vol_from_faces_8_vertex_box() {
        use remus_math::vec::Vec3;
        use remus_operations::tessellate::TriangleMesh;

        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(10.0, 10.0, 10.0),
            Point3::new(0.0, 10.0, 10.0),
        ];
        let normals = vec![Vec3::new(0.0, 0.0, 1.0); 8];
        let indices = vec![
            0u32, 2, 1, 0, 3, 2, // bottom -Z
            4, 5, 6, 4, 6, 7, // top +Z
            0, 1, 5, 0, 5, 4, // front -Y
            2, 3, 7, 2, 7, 6, // back +Y
            0, 4, 7, 0, 7, 3, // left -X
            1, 2, 6, 1, 6, 5, // right +X
        ];

        let mesh = TriangleMesh {
            positions,
            normals,
            indices,
        };
        let mut topo = Topology::new();
        let solid = import_mesh(&mut topo, &mesh, 1e-7).unwrap();

        let vol = remus_operations::measure::solid_volume_from_faces(&topo, solid, 0.01).unwrap();
        assert!(
            (vol - 1000.0).abs() < 10.0,
            "expected ~1000 from vol_from_faces, got {vol}"
        );
    }

    #[test]
    fn vol_from_faces_stl_roundtrip() {
        // This simulates the actual path: tessellate box → flat mesh → import_mesh
        let mut write_topo = Topology::new();
        let solid =
            remus_operations::primitives::make_box(&mut write_topo, 10.0, 10.0, 10.0).unwrap();

        let stl_bytes = writer::write_stl(&write_topo, &[solid], 0.1, StlFormat::Binary).unwrap();
        let mesh = read_stl(&stl_bytes).unwrap();

        let mut topo = Topology::new();
        let imported = import_mesh(&mut topo, &mesh, 1e-4).unwrap();

        let vol =
            remus_operations::measure::solid_volume_from_faces(&topo, imported, 0.01).unwrap();
        assert!(
            (vol - 1000.0).abs() < 10.0,
            "expected ~1000 from vol_from_faces, got {vol}"
        );
    }

    #[test]
    fn vol_from_faces_per_face_tessellation() {
        // Simulates the JS 3MF path: per-face tessellate → flat mesh → import_mesh
        // This is the path that produces 333.33 instead of 1000.
        use remus_operations::tessellate;

        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

        // Per-face tessellation (same as JS meshSolid/tessellateFace)
        let solid_data = topo.solid(solid).unwrap();
        let shell = topo.shell(solid_data.outer_shell()).unwrap();
        let face_ids: Vec<_> = shell.faces().to_vec();

        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();
        let mut vert_offset = 0u32;

        for &fid in &face_ids {
            let mesh = tessellate::tessellate(&topo, fid, 0.1).unwrap();
            let n_verts = mesh.positions.len();
            positions.extend_from_slice(&mesh.positions);
            normals.extend_from_slice(&mesh.normals);
            for &idx in &mesh.indices {
                indices.push(idx + vert_offset);
            }
            vert_offset += n_verts as u32;
        }

        let mesh = remus_operations::tessellate::TriangleMesh {
            positions,
            normals,
            indices,
        };

        let mut import_topo = Topology::new();
        let imported = import_mesh(&mut import_topo, &mesh, 1e-4).unwrap();

        let vol_from_faces =
            remus_operations::measure::solid_volume_from_faces(&import_topo, imported, 0.01)
                .unwrap();
        let vol_standard =
            remus_operations::measure::solid_volume(&import_topo, imported, 0.01).unwrap();

        eprintln!("vol_from_faces = {vol_from_faces}");
        eprintln!("vol_standard   = {vol_standard}");

        assert!(
            (vol_from_faces - 1000.0).abs() < 10.0,
            "expected ~1000 from vol_from_faces, got {vol_from_faces}"
        );
        assert!(
            (vol_standard - 1000.0).abs() < 10.0,
            "expected ~1000 from vol_standard, got {vol_standard}"
        );
    }

    #[test]
    fn import_single_triangle() {
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
            indices: vec![0, 1, 2],
        };

        let mut topo = Topology::new();
        let solid_id = import_mesh(&mut topo, &mesh, 1e-7).unwrap();

        let solid = topo.solid(solid_id).unwrap();
        let shell = topo.shell(solid.outer_shell()).unwrap();
        assert_eq!(shell.faces().len(), 1);
    }

    #[test]
    fn import_two_triangles() {
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![Vec3::new(0.0, 0.0, 1.0); 6],
            indices: vec![0, 1, 2, 3, 4, 5],
        };

        let mut topo = Topology::new();
        let solid_id = import_mesh(&mut topo, &mesh, 1e-7).unwrap();

        let solid = topo.solid(solid_id).unwrap();
        let shell = topo.shell(solid.outer_shell()).unwrap();
        assert_eq!(shell.faces().len(), 2);
    }

    #[test]
    fn import_stl_roundtrip_unit_cube() {
        let mut write_topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut write_topo);

        let stl_bytes = writer::write_stl(&write_topo, &[solid], 0.1, StlFormat::Binary).unwrap();
        let mesh = read_stl(&stl_bytes).unwrap();

        let mut read_topo = Topology::new();
        let imported = import_mesh(&mut read_topo, &mesh, 1e-4).unwrap();

        let read_solid = read_topo.solid(imported).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();
        // Unit cube: 12 triangles.
        assert_eq!(shell.faces().len(), 12);
    }

    #[test]
    fn vertex_merging() {
        // Two triangles sharing an edge — should merge 2 vertices.
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.5, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0), // Same as [1]
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(0.5, 1.0, 0.0), // Same as [2]
            ],
            normals: vec![Vec3::new(0.0, 0.0, 1.0); 6],
            indices: vec![0, 1, 2, 3, 4, 5],
        };

        let mut topo = Topology::new();
        let _solid = import_mesh(&mut topo, &mesh, 1e-6).unwrap();

        // Should have 4 unique vertices, not 6.
        assert_eq!(topo.vertices().len(), 4);
    }

    /// A strip of `count` distinct positions spaced one unit apart along x
    /// from `x0`, each listed twice the way STL repeats shared corners.
    fn doubled_strip(x0: f64, count: usize) -> Vec<Point3> {
        (0..count)
            .flat_map(|i| {
                let p = Point3::new(x0 + i as f64, 0.25 * (i % 2) as f64, 0.0);
                [p, p]
            })
            .collect()
    }

    fn weld(positions: &[Point3], tolerance: f64) -> (Vec<VertexId>, usize) {
        let mut topo = Topology::new();
        weld_positions(positions, tolerance, |p| {
            topo.add_vertex(Vertex::new(p, tolerance))
        })
    }

    /// IO-2 (kernel review 2026-10-14): at x = 1e12 the old absolute key
    /// `(x / 1e-7).floor() as i64` saturated to `i64::MAX` for every vertex,
    /// so all of them shared one bucket and each insertion distance-checked
    /// every earlier vertex: 4,000,000 checks for this strip instead of a
    /// handful per vertex. Far-from-origin parts must weld as cheaply as
    /// parts at the origin.
    #[test]
    fn far_from_origin_weld_stays_linear() {
        let count = 2000;
        for x0 in [0.0, 1e12, -3e13, -3e15] {
            let positions = doubled_strip(x0, count);
            let (map, checks) = weld(&positions, 1e-7);
            let mut unique = map.clone();
            unique.dedup();
            assert_eq!(unique.len(), count, "x0 = {x0}: every pair must weld");
            assert!(
                checks <= 4 * positions.len(),
                "x0 = {x0}: {checks} distance checks for {} positions",
                positions.len()
            );
        }
    }

    /// One far vertex must not coarsen the cells of every other vertex, as a
    /// grid sized from the bounding box would (a strip at the origin plus an
    /// unused vertex at 1e308).
    #[test]
    fn far_outlier_vertex_does_not_coarsen_the_other_cells() {
        let count = 2000;
        let mut positions = doubled_strip(0.0, count);
        positions.push(Point3::new(1e308, -1e308, 1e308));
        let (map, checks) = weld(&positions, 1e-7);
        let mut unique = map;
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), count + 1);
        assert!(checks <= 4 * positions.len(), "{checks} distance checks");
    }

    #[test]
    fn overflowing_cell_indices_weld_only_identical_coordinates() {
        // At 1e-150 the scaled index of 1e160 overflows and that of 1.0 is
        // past 2^53, so both use exact-coordinate keys.
        let tolerance = 1e-150;
        let positions = [
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(1e160, -0.0, 7.0),
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(1e160, 0.0, 7.0),
            Point3::new(1e160 * (1.0 + f64::EPSILON), 0.0, 7.0),
        ];
        let (map, _) = weld(&positions, tolerance);
        assert_eq!(map[0], map[2]);
        assert_eq!(map[1], map[3], "-0.0 and 0.0 are the same coordinate");
        assert_ne!(map[3], map[4]);
        assert_ne!(map[0], map[1]);
    }

    #[test]
    fn power_of_two_cells_bracket_twice_the_tolerance() {
        for tolerance in [1e-320, 5e-324, 1e-300, 1e-7, 0.5, 1.0, 3.0, 1e300, f64::MAX] {
            let grid = WeldGrid::new(tolerance);
            let cell = (1.0 / grid.scale[0]) * (1.0 / grid.scale[1]);
            assert!(grid.scale.iter().all(|s| s.is_finite() && *s > 0.0));
            if (2.0 * tolerance).is_finite() && cell.is_finite() {
                assert!(cell >= 2.0 * tolerance, "tol {tolerance}: cell {cell}");
                assert!(cell < 4.0 * tolerance, "tol {tolerance}: cell {cell}");
            }
        }
    }

    #[test]
    fn weld_over_the_full_f64_range_keys_finite_cells() {
        let positions = [
            Point3::new(-1e308, 0.0, 0.0),
            Point3::new(1e308, 0.0, 0.0),
            Point3::new(1e308, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5e-8, 0.0, 0.0),
        ];
        let (map, _) = weld(&positions, 1e-7);
        assert_eq!(map[1], map[2]);
        assert_eq!(map[3], map[4]);
        assert_ne!(map[0], map[1]);
        assert_ne!(map[0], map[3]);
    }

    #[test]
    fn pairs_just_inside_tolerance_weld_across_cell_boundaries() {
        // Straddle cell boundaries at several offsets: a pair 0.99 tol apart
        // must weld wherever the boundary falls.
        let tolerance = 1e-3;
        for step in 0..40 {
            let x = 1.0 + f64::from(step) * 0.37 * tolerance;
            let positions = [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(x, x, -x),
                Point3::new(x + 0.99 * tolerance, x, -x),
            ];
            let (map, _) = weld(&positions, tolerance);
            assert_eq!(map[1], map[2], "x = {x}");
        }
    }

    #[test]
    fn empty_mesh_error() {
        let mesh = TriangleMesh::default();
        let mut topo = Topology::new();
        let result = import_mesh(&mut topo, &mesh, 1e-7);
        assert!(result.is_err());
    }

    #[test]
    fn invalid_indices_return_error_without_mutating_topology() {
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: Vec::new(),
            indices: vec![0, 1, 3],
        };
        let mut topo = Topology::new();

        let result = import_mesh(&mut topo, &mesh, 1e-7);

        assert!(result.is_err());
        assert!(topo.vertices().is_empty());
        assert!(topo.edges().is_empty());
        assert!(topo.faces().is_empty());
    }

    #[test]
    fn non_finite_mesh_data_returns_error_without_mutating_topology() {
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(f64::NAN, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: Vec::new(),
            indices: vec![0, 1, 2],
        };
        let mut topo = Topology::new();

        let result = import_mesh(&mut topo, &mesh, 1e-7);

        assert!(result.is_err());
        assert!(topo.vertices().is_empty());
    }

    #[test]
    fn degenerate_triangles_skipped() {
        // Triangle with two coincident vertices should be skipped.
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0), // Same as [0]
                Point3::new(1.0, 1.0, 0.0),
                // Valid triangle
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![Vec3::new(0.0, 0.0, 1.0); 6],
            indices: vec![0, 1, 2, 3, 4, 5],
        };

        let mut topo = Topology::new();
        let solid = import_mesh(&mut topo, &mesh, 1e-6).unwrap();

        let s = topo.solid(solid).unwrap();
        let shell = topo.shell(s.outer_shell()).unwrap();
        // Only the valid triangle should remain.
        assert_eq!(shell.faces().len(), 1);
    }
}
