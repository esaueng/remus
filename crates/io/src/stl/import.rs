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
/// into a closed shell and solid.
///
/// # Errors
///
/// Returns [`IoError`] if:
/// - The mesh has no triangles
/// - Wire or shell construction fails
pub fn import_mesh(
    topo: &mut Topology,
    mesh: &TriangleMesh,
    tolerance: f64,
) -> Result<SolidId, IoError> {
    validate_mesh(mesh, tolerance)?;

    let vertex_ids = build_vertex_map(topo, &mesh.positions, tolerance);

    // Determine whether the mesh winding needs to be flipped.
    // For a closed mesh, outward-facing triangles produce positive signed
    // volume via the divergence theorem. If the raw signed volume is negative,
    // the winding is predominantly inward — we flip all triangles.
    //
    // When per-vertex normals are available (e.g. STL), we use them to orient
    // individual triangles. When normals are absent (e.g. 3MF), we rely on
    // the signed-volume heuristic to flip the entire mesh if needed.
    let has_normals = mesh.normals.len() >= mesh.positions.len();
    let flip_all = if has_normals {
        false // per-triangle correction below handles it
    } else {
        let mut total = 0.0;
        for tri in mesh.indices.chunks_exact(3) {
            let p0 = mesh.positions[tri[0] as usize];
            let p1 = mesh.positions[tri[1] as usize];
            let p2 = mesh.positions[tri[2] as usize];
            let a = Vec3::new(p0.x(), p0.y(), p0.z());
            let b = Vec3::new(p1.x(), p1.y(), p1.z());
            let c = Vec3::new(p2.x(), p2.y(), p2.z());
            total += a.dot(b.cross(c));
        }
        total < 0.0
    };

    let mut face_ids = Vec::new();
    // Edges are shared between the two triangles that meet along them. Giving
    // each triangle its own three edges would leave every face an island: the
    // shell would have no adjacency at all and every edge would read as free.
    let mut edge_map: HashMap<(usize, usize), EdgeId> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;

        let v0 = vertex_ids[i0];
        let mut v1 = vertex_ids[i1];
        let mut v2 = vertex_ids[i2];

        // Skip degenerate triangles (two or more coincident vertices).
        if v0 == v1 || v1 == v2 || v0 == v2 {
            continue;
        }

        // Orient triangle: use per-vertex normals when available,
        // otherwise apply the global flip from signed-volume check.
        if has_normals {
            let p0 = mesh.positions[i0];
            let p1 = mesh.positions[i1];
            let p2 = mesh.positions[i2];
            let geo_normal = (p1 - p0).cross(p2 - p0);
            let mesh_normal = mesh.normals[i0];
            if geo_normal.dot(mesh_normal) < 0.0 {
                std::mem::swap(&mut v1, &mut v2);
            }
        } else if flip_all {
            std::mem::swap(&mut v1, &mut v2);
        }

        let face_id = build_triangle_face(topo, &mut edge_map, v0, v1, v2)?;
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
///
/// Candidates are found through a uniform spatial hash whose cell edge is the
/// weld tolerance, so a scan is bounded by the 27 cells that can hold a point
/// within tolerance rather than by every vertex seen so far. A mesh scan
/// arrives with almost every position distinct, which is the linear search's
/// worst case: it made import quadratic in vertex count.
///
/// Cell membership alone never decides a merge — two points in one cell can
/// still be up to a diagonal apart, and two points a hair either side of a
/// cell boundary are neighbours. Every candidate is distance-checked, and the
/// 27-cell probe is what stops a boundary-straddling pair from being missed.
fn build_vertex_map(topo: &mut Topology, positions: &[Point3], tolerance: f64) -> Vec<VertexId> {
    let tol_sq = tolerance * tolerance;
    let mut buckets: HashMap<(i64, i64, i64), Vec<(Point3, VertexId)>> = HashMap::new();
    let mut map = Vec::with_capacity(positions.len());

    // `as i64` saturates rather than wrapping, so an extreme
    // coordinate-to-tolerance ratio degrades to larger buckets, never to a
    // wrong cell.
    let cell = |v: f64| -> i64 { (v / tolerance).floor() as i64 };

    for &pos in positions {
        let (cx, cy, cz) = (cell(pos.x()), cell(pos.y()), cell(pos.z()));

        // Among every candidate within tolerance, take the earliest-created
        // vertex. Insertion order decided the winner in the linear scan this
        // replaces, and vertex ids are allocated in order, so picking the
        // smallest id keeps welding deterministic and order-independent.
        let mut best: Option<VertexId> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = (
                        cx.saturating_add(dx),
                        cy.saturating_add(dy),
                        cz.saturating_add(dz),
                    );
                    let Some(bucket) = buckets.get(&key) else {
                        continue;
                    };
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
            let vid = topo.add_vertex(Vertex::new(pos, tolerance));
            buckets.entry((cx, cy, cz)).or_default().push((pos, vid));
            map.push(vid);
        }
    }

    map
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
    let key = if a.index() <= b.index() {
        (a.index(), b.index())
    } else {
        (b.index(), a.index())
    };
    if let Some(&edge_id) = edge_map.get(&key) {
        // Natural direction is whichever pair created it.
        let forward = topo.edge(edge_id).is_ok_and(|e| e.start() == a);
        return (edge_id, forward);
    }
    let edge_id = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
    edge_map.insert(key, edge_id);
    (edge_id, true)
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
