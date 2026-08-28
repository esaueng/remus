//! Mesh import must produce connected B-Rep topology, not a pile of islands.
//!
//! `import_mesh` gave every triangle its own three edges, so no two faces
//! ever shared one: a closed cube imported as 36 distinct edges, every one of
//! them free, and the shell failed every adjacency-based operation. Vertex
//! welding was a linear scan per position, which is quadratic on the
//! all-distinct input a mesh scan actually produces.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_io::IoError;
use remus_io::stl::{import_mesh, read_stl_solid};
use remus_io::threemf::read_threemf_solid;
use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const NON_MANIFOLD_ASCII_STL: &[u8] = br"solid non_manifold
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex 1 0 0
      vertex 0 0 0
      vertex 0 -1 0
    endloop
  endfacet
  facet normal 0 -1 0
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 0 1
    endloop
  endfacet
endsolid non_manifold
";

fn non_manifold_threemf() -> Vec<u8> {
    let model = br#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" type="model">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/>
          <vertex x="1" y="0" z="0"/>
          <vertex x="0" y="1" z="0"/>
          <vertex x="0" y="-1" z="0"/>
          <vertex x="0" y="0" z="1"/>
        </vertices>
        <triangles>
          <triangle v1="0" v2="1" v3="2"/>
          <triangle v1="1" v2="0" v3="3"/>
          <triangle v1="0" v2="1" v3="4"/>
        </triangles>
      </mesh>
    </object>
  </resources>
  <build><item objectid="1"/></build>
</model>"#;

    let cursor = std::io::Cursor::new(Vec::new());
    let mut archive = zip::ZipWriter::new(cursor);
    archive
        .start_file("3D/3dmodel.model", zip::write::SimpleFileOptions::default())
        .unwrap();
    std::io::Write::write_all(&mut archive, model).unwrap();
    archive.finish().unwrap().into_inner()
}

fn assert_rejected_before_brep_construction(result: Result<SolidId, IoError>, topo: &Topology) {
    let err = result.expect_err("an edge shared by three triangles must be rejected");
    let IoError::InvalidTopology { reason } = err else {
        panic!("expected invalid topology error, got {err}");
    };
    assert!(
        reason.contains("non-manifold mesh edge") && reason.contains("3 incident faces"),
        "unexpected rejection reason: {reason}"
    );
    assert_eq!(topo.num_edges(), 0, "the manifold gate must precede edges");
    assert_eq!(topo.num_faces(), 0, "the manifold gate must precede faces");
    assert_eq!(
        topo.num_shells(),
        0,
        "the manifold gate must precede shells"
    );
    assert_eq!(
        topo.num_solids(),
        0,
        "the manifold gate must precede solids"
    );
}

/// Unit cube: 12 triangles over 8 shared vertices, wound outward.
fn cube_mesh() -> TriangleMesh {
    let p = Point3::new;
    let positions = vec![
        p(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(1.0, 1.0, 0.0),
        p(0.0, 1.0, 0.0),
        p(0.0, 0.0, 1.0),
        p(1.0, 0.0, 1.0),
        p(1.0, 1.0, 1.0),
        p(0.0, 1.0, 1.0),
    ];
    let indices = vec![
        0, 2, 1, 0, 3, 2, // bottom
        4, 5, 6, 4, 6, 7, // top
        0, 1, 5, 0, 5, 4, // front
        1, 2, 6, 1, 6, 5, // right
        2, 3, 7, 2, 7, 6, // back
        3, 0, 4, 3, 4, 7, // left
    ];
    TriangleMesh {
        normals: vec![Vec3::new(0.0, 0.0, 0.0); positions.len()],
        positions,
        indices,
    }
}

/// (distinct edges, free edges) across every wire of the solid.
fn edge_usage(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    (usage.len(), usage.values().filter(|&&c| c == 1).count())
}

#[test]
fn adjacent_triangles_share_one_edge() {
    let mut topo = Topology::new();
    let solid = import_mesh(&mut topo, &cube_mesh(), 1e-7).unwrap();

    let (distinct, free) = edge_usage(&topo, solid);
    // Euler for a closed triangulated cube: V=8, F=12, so E=18.
    assert_eq!(
        distinct, 18,
        "12 triangles over 8 vertices must resolve to 18 shared edges, got {distinct}"
    );
    assert_eq!(free, 0, "a closed mesh must import with no free edges");
}

#[test]
fn stl_solid_rejects_edge_incident_to_three_faces() {
    let mut topo = Topology::new();
    let result = read_stl_solid(&mut topo, NON_MANIFOLD_ASCII_STL, 1e-7);

    assert_rejected_before_brep_construction(result, &topo);
}

#[test]
fn threemf_solid_rejects_edge_incident_to_three_faces() {
    let mut topo = Topology::new();
    let result = read_threemf_solid(&mut topo, &non_manifold_threemf(), 1e-7);

    assert_rejected_before_brep_construction(result, &topo);
}

#[test]
fn imported_closed_mesh_is_a_closed_manifold_shell() {
    let mut topo = Topology::new();
    let solid = import_mesh(&mut topo, &cube_mesh(), 1e-7).unwrap();
    let shell_id = topo.solid(solid).unwrap().outer_shell();
    remus_topology::validation::validate_shell_closed(topo.shell(shell_id).unwrap(), &topo)
        .expect("an imported closed mesh must be a closed 2-manifold");
}

#[test]
fn coincident_vertices_straddling_a_hash_cell_boundary_still_weld() {
    // The hazard the spatial hash introduces: welding by cell membership
    // alone would miss a pair that sits a hair either side of a boundary.
    // With cell edge == tolerance, these two land in cells 0 and 1 but are
    // 2e-10 apart, far inside the 1e-3 weld tolerance.
    let tol = 1e-3;
    let below = 1e-3 - 1e-10;
    let above = 1e-3 + 1e-10;

    let p = Point3::new;
    let positions = vec![
        p(below, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(0.5, 1.0, 0.0),
        p(above, 0.0, 0.0), // must weld onto vertex 0
        p(0.5, 1.0, 1.0),
    ];
    let mesh = TriangleMesh {
        normals: vec![Vec3::new(0.0, 0.0, 1.0); positions.len()],
        positions,
        indices: vec![0, 1, 2, 3, 4, 1],
    };

    let mut topo = Topology::new();
    let before = topo.num_vertices();
    import_mesh(&mut topo, &mesh, tol).unwrap();
    let created = topo.num_vertices() - before;

    assert_eq!(
        created, 4,
        "the boundary-straddling pair must weld to one vertex (5 positions -> 4), got {created}"
    );
}

#[test]
fn vertices_further_apart_than_the_tolerance_stay_distinct() {
    // The control for the weld: a hash that merged whole cells would collapse
    // these two, which sit in one cell but 3e-4 apart at a 1e-4 tolerance.
    let p = Point3::new;
    let positions = vec![
        p(0.0, 0.0, 0.0),
        p(3e-4, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(0.5, 1.0, 0.0),
    ];
    let mesh = TriangleMesh {
        normals: vec![Vec3::new(0.0, 0.0, 1.0); positions.len()],
        positions,
        indices: vec![0, 2, 3, 1, 2, 3],
    };

    let mut topo = Topology::new();
    import_mesh(&mut topo, &mesh, 1e-4).unwrap();
    assert_eq!(
        topo.num_vertices(),
        4,
        "positions beyond the weld tolerance must stay distinct"
    );
}

#[test]
fn extreme_coordinate_to_tolerance_ratio_does_not_overflow_hash_probe() {
    // Converting a finite coordinate/tolerance ratio beyond i64's range
    // saturates the cell coordinate. Probing its neighbouring cells must
    // saturate too rather than overflowing at i64::MIN or i64::MAX.
    let p = Point3::new;
    let positions = vec![p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, 0.0, 0.0)];
    let mesh = TriangleMesh {
        normals: vec![Vec3::new(0.0, 0.0, 1.0); positions.len()],
        positions,
        indices: vec![0, 1, 2],
    };

    let mut topo = Topology::new();
    import_mesh(&mut topo, &mesh, 1e-300)
        .expect("a finite coordinate/tolerance ratio must not overflow the hash probe");
}

#[test]
fn welding_does_not_depend_on_input_order() {
    // Insertion order picked the winner in the linear scan this replaces;
    // the hash keeps that by taking the earliest-created candidate. Reversing
    // the mesh must still collapse to the same vertex count.
    let mut mesh = cube_mesh();
    let mut topo_a = Topology::new();
    import_mesh(&mut topo_a, &mesh, 1e-7).unwrap();

    mesh.positions.reverse();
    mesh.normals.reverse();
    let last = u32::try_from(mesh.positions.len() - 1).unwrap();
    for i in &mut mesh.indices {
        *i = last - *i;
    }
    let mut topo_b = Topology::new();
    import_mesh(&mut topo_b, &mesh, 1e-7).unwrap();

    assert_eq!(
        topo_a.num_vertices(),
        topo_b.num_vertices(),
        "vertex welding must not depend on the order positions arrive in"
    );
}

#[test]
fn a_multi_body_mesh_still_imports_as_a_valid_solid() {
    // Two disjoint cubes in one mesh land in one shell. Documented, not
    // asserted as ideal: a shell is conventionally connected, and splitting
    // components into separate solids would change the mesh-reader return
    // convention. What matters here is that it is not *broken* — before edge
    // sharing this case was a pile of islands.
    let a = cube_mesh();
    let offset = u32::try_from(a.positions.len()).unwrap();
    let mut mesh = a.clone();
    mesh.positions.extend(
        a.positions
            .iter()
            .map(|q| Point3::new(q.x() + 10.0, q.y(), q.z())),
    );
    mesh.normals.extend(a.normals.iter().copied());
    mesh.indices.extend(a.indices.iter().map(|i| i + offset));

    let mut topo = Topology::new();
    let solid = import_mesh(&mut topo, &mesh, 1e-7).unwrap();

    let (_, free) = edge_usage(&topo, solid);
    assert_eq!(free, 0, "each body must be closed in itself");

    let volume = remus_operations::measure::solid_volume_from_faces(&topo, solid, 0.01).unwrap();
    assert!(
        (volume - 2.0).abs() < 1e-6,
        "two unit cubes should measure 2.0, got {volume}"
    );
    let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(report.is_valid(), "multi-body import must still validate");
}

/// The correctness tests above all pass against a QUADRATIC weld — a linear
/// scan gives the same answers, just slowly. Nothing here stopped the weld
/// regressing to one, and the cost is invisible at test sizes: it was 153ms at
/// 26k vertices, with per-vertex cost climbing 0.0004 -> 0.0059 ms across the
/// range, extrapolating to roughly a quarter of an hour at the 2,000,000-vertex
/// `ImportLimits` ceiling the importers already enforce.
///
/// So this asserts the SHAPE of the curve rather than a wall-clock number: cost
/// per vertex must not grow with the mesh. A quadratic weld fails it by a wide
/// margin — measured 11x growth in per-vertex cost over this range — while the
/// 8x bound keeps it from being a flaky timing test on a shared runner.
#[test]
fn vertex_weld_cost_does_not_grow_with_mesh_size() {
    use std::time::Instant;

    // A k x k grid of quads, all vertices distinct: nothing to merge, so this
    // measures the candidate search itself rather than the merge.
    fn grid(k: usize) -> TriangleMesh {
        let mut positions = Vec::new();
        for j in 0..=k {
            for i in 0..=k {
                positions.push(Point3::new(i as f64, j as f64, 0.0));
            }
        }
        let idx = |i: usize, j: usize| (j * (k + 1) + i) as u32;
        let mut indices = Vec::new();
        for j in 0..k {
            for i in 0..k {
                indices.extend_from_slice(&[idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)]);
                indices.extend_from_slice(&[idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)]);
            }
        }
        let n = positions.len();
        TriangleMesh {
            positions,
            normals: vec![Vec3::new(0.0, 0.0, 1.0); n],
            indices,
        }
    }

    let per_vertex = |k: usize| -> f64 {
        let mesh = grid(k);
        let n = mesh.positions.len() as f64;
        let mut topo = Topology::new();
        let started = Instant::now();
        let _ = import_mesh(&mut topo, &mesh, 1e-9);
        started.elapsed().as_secs_f64() / n
    };

    let small = per_vertex(40);
    let large = per_vertex(160);
    assert!(
        large < small * 8.0,
        "per-vertex weld cost grew from {small:.3e}s to {large:.3e}s over a 16x larger mesh — \
         the weld is superlinear again"
    );
}
