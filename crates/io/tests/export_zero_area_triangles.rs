//! Regression: the STL and 3MF exporters must never serialize an exact
//! zero-area facet.
//!
//! A boolean or an import can leave a face whose tessellation contains
//! collapsed triangles (two vertices at the same position, or three exactly
//! collinear vertices). A serialized record with zero area reads back as an
//! open mesh: the facet contributes two identical directed edges that cancel
//! each other, so a consumer's closure count sees a hole where there is
//! none. PR #520 added [`remus_io::retain_nondegenerate_triangles`] at the
//! writer seam; these tests pin the public contract from the outside:
//!
//! 1. The provider filter drops only triangles whose exact cross product is
//!    zero or non-finite — a `1e-15`-height sliver survives, as does any
//!    finite nonzero-area triangle.
//! 2. The mesh-level STL writers (binary and ASCII) apply the filter: a
//!    controlled mesh carrying collapsed facets exports without them, and
//!    the emitted file is closed, consistently oriented, and volume-correct.
//! 3. The mesh-level 3MF writer applies the same filter.
//! 4. The B-rep writers (`write_stl`, `write_threemf`) emit meshes whose
//!    every facet has finite nonzero area and which are watertight.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::stl::writer::StlFormat;
use remus_math::vec::{Point3, Vec3};
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{TriangleMesh, tessellate_solid, welded_mesh_quality};
use remus_topology::Topology;

/// A closed outward-wound tetrahedron: 4 triangles, volume `1000/6`.
fn tetrahedron() -> (Vec<Point3>, Vec<u32>) {
    (
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
        ],
        vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2],
    )
}

/// The tetrahedron with three collapsed triangles appended: two with a
/// coincident vertex (the production failure class — survives `f32`
/// quantization exactly) and one with non-finite coordinates.
fn degenerate_fixture() -> TriangleMesh {
    let (positions, mut indices) = tetrahedron();
    // a == c: exactly zero cross product.
    indices.extend_from_slice(&[0, 1, 0]);
    // b == c: exactly zero cross product.
    indices.extend_from_slice(&[1, 2, 1]);
    // non-finite coordinate: zero-area and must be refused, not serialized.
    let mut positions = positions;
    positions.push(Point3::new(f64::NAN, 0.0, 0.0));
    indices.extend_from_slice(&[0, 1, 4]);
    TriangleMesh {
        positions,
        normals: Vec::new(),
        indices,
    }
}

/// Signed volume of a closed triangulated mesh via the divergence theorem.
fn signed_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let a = mesh.positions[t[0] as usize];
            let b = mesh.positions[t[1] as usize];
            let c = mesh.positions[t[2] as usize];
            let av = Vec3::new(a.x(), a.y(), a.z());
            let bv = Vec3::new(b.x(), b.y(), b.z());
            let cv = Vec3::new(c.x(), c.y(), c.z());
            av.dot(bv.cross(cv)) / 6.0
        })
        .sum()
}

/// Assert every facet of the mesh has three pairwise-distinct vertex
/// positions (exact f64 equality) and finite coordinates.
fn assert_no_degenerate_facets(mesh: &TriangleMesh, label: &str) {
    assert_eq!(mesh.indices.len() % 3, 0, "{label}: index count");
    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let a = mesh.positions[tri[0] as usize];
        let b = mesh.positions[tri[1] as usize];
        let c = mesh.positions[tri[2] as usize];
        assert!(
            [a, b, c]
                .iter()
                .all(|p| p.x().is_finite() && p.y().is_finite() && p.z().is_finite()),
            "{label}: triangle {t} has a non-finite coordinate"
        );
        assert!(
            a != b && b != c && a != c,
            "{label}: triangle {t} has coincident vertices"
        );
        let cross = (b - a).cross(c - a);
        assert!(
            cross.length_squared() > 0.0 && cross.length_squared().is_finite(),
            "{label}: triangle {t} has zero area"
        );
    }
}

#[test]
fn provider_filter_removes_only_exact_zero_area_and_non_finite_triangles() {
    let mut mesh = TriangleMesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 1.0e-15, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        normals: Vec::new(),
        indices: vec![0, 1, 2, 0, 0, 2, 0, 1, 4, 0, 1, 3],
    };

    remus_io::retain_nondegenerate_triangles(&mut mesh);

    // The coincident-vertex triangle (0,0,2) is dropped, the exactly
    // collinear-free valid ones survive, and the 1e-15-height sliver (0,1,3)
    // is kept: its area is tiny but nonzero and finite.
    assert_eq!(mesh.indices, vec![0, 1, 2, 0, 1, 3]);
    assert_eq!(mesh.positions.len(), 5, "positions must be untouched");
}

#[test]
fn mesh_stl_writers_omit_collapsed_facets() {
    let fixture = degenerate_fixture();
    for (label, format) in [("binary", StlFormat::Binary), ("ascii", StlFormat::Ascii)] {
        let bytes = remus_io::stl::write_mesh_stl(&fixture, format).unwrap();
        let mesh = remus_io::stl::read_stl(&bytes).unwrap();
        assert_eq!(
            mesh.indices.len(),
            12,
            "{label}: collapsed facets must not be written"
        );
        assert_no_degenerate_facets(&mesh, label);
        let quality = welded_mesh_quality(&mesh);
        assert_eq!(
            (quality.boundary_edges, quality.non_manifold_edges),
            (0, 0),
            "{label}: emitted STL must be closed and manifold"
        );
        let volume = signed_volume(&mesh).abs();
        assert!(
            (volume - 1000.0 / 6.0).abs() < 1e-6,
            "{label}: volume {volume} vs 1000/6"
        );
    }
}

#[test]
fn mesh_threemf_writer_omits_collapsed_facets() {
    let fixture = degenerate_fixture();
    let bytes = remus_io::threemf::write_mesh_threemf(&[fixture]).unwrap();
    let meshes = remus_io::threemf::read_threemf(&bytes).unwrap();
    assert_eq!(meshes.len(), 1, "one mesh in the model part");
    let mesh = &meshes[0];
    assert_eq!(
        mesh.indices.len(),
        12,
        "collapsed facets must not be written"
    );
    assert_no_degenerate_facets(mesh, "3mf");
    let quality = welded_mesh_quality(mesh);
    assert_eq!(
        (quality.boundary_edges, quality.non_manifold_edges),
        (0, 0),
        "emitted 3MF must be closed and manifold"
    );
    let volume = signed_volume(mesh).abs();
    assert!((volume - 1000.0 / 6.0).abs() < 1e-9, "3mf volume {volume}");
}

#[test]
fn brep_exporters_emit_only_finite_nonzero_area_facets() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let brep = tessellate_solid(&topo, solid, 0.05).unwrap();
    let mut filtered = brep.clone();
    remus_io::retain_nondegenerate_triangles(&mut filtered);
    assert_eq!(
        filtered.indices, brep.indices,
        "a box tessellation has no degenerates to drop"
    );

    let stl = remus_io::stl::write_stl(&topo, &[solid], 0.05, StlFormat::Binary).unwrap();
    let mesh = remus_io::stl::read_stl(&stl).unwrap();
    assert_eq!(
        mesh.indices.len() / 3,
        brep.indices.len() / 3,
        "STL facet count"
    );
    assert_no_degenerate_facets(&mesh, "STL");
    let quality = welded_mesh_quality(&mesh);
    assert_eq!(
        (quality.boundary_edges, quality.non_manifold_edges),
        (0, 0),
        "STL export of a box must be closed and manifold"
    );
    let volume = signed_volume(&mesh).abs();
    assert!((volume - 24.0).abs() < 1e-6, "STL volume {volume}");

    let threemf = remus_io::threemf::write_threemf(&topo, &[solid], 0.05).unwrap();
    let meshes = remus_io::threemf::read_threemf(&threemf).unwrap();
    assert_eq!(meshes.len(), 1, "3MF mesh count");
    let mesh = &meshes[0];
    assert_eq!(
        mesh.indices.len() / 3,
        brep.indices.len() / 3,
        "3MF facet count"
    );
    assert_no_degenerate_facets(mesh, "3MF");
    let quality = welded_mesh_quality(mesh);
    assert_eq!(
        (quality.boundary_edges, quality.non_manifold_edges),
        (0, 0),
        "3MF export of a box must be closed and manifold"
    );
    let volume = signed_volume(mesh).abs();
    assert!((volume - 24.0).abs() < 1e-9, "3MF volume {volume}");
}
