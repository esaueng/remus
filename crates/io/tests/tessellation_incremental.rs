//! Incremental tessellation must be the fresh result, including shared edges,
//! normals and face attribution. Source geometry is immutable through reuse.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_io::arena_io::serialize_solids;
use remus_io::step::reader::read_step;
use remus_operations::measure::face_area;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{
    TriangleMesh, disable_face_mesh_cache, disable_mesh_reuse, enable_mesh_reuse, mesh_reuse_stats,
    tessellate_solid_grouped_with_tolerance, welded_mesh_quality,
};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const FIXTURE: &str = include_str!("data/shapr3d_hammer_holder.step");
const DEFLECTION: f64 = 0.0148;
const ANGULAR: f64 = 0.06;

type Mesh = (TriangleMesh, Vec<u32>);

struct Guard;
impl Guard {
    fn new() -> Self {
        disable_face_mesh_cache();
        disable_mesh_reuse();
        Self
    }
}
impl Drop for Guard {
    fn drop(&mut self) {
        disable_mesh_reuse();
    }
}

fn edited_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid).unwrap().into_iter().find(|&face| {
        matches!(topo.face(face).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.x() > 0.99)
            && (face_area(topo, face, 0.05).unwrap() - 1045.93).abs() < 1.0
    }).expect("outer arm face")
}
fn mesh(topo: &Topology, solid: SolidId, linear: f64, angular: f64) -> Mesh {
    tessellate_solid_grouped_with_tolerance(topo, solid, linear, angular).unwrap()
}
fn same(actual: &Mesh, expected: &Mesh) {
    assert_eq!(actual.1, expected.1, "face attribution");
    assert_eq!(
        actual.0.indices, expected.0.indices,
        "triangle connectivity"
    );
    assert_eq!(actual.0.positions.len(), expected.0.positions.len());
    assert_eq!(actual.0.normals.len(), expected.0.normals.len());
    for (a, b) in actual.0.positions.iter().zip(&expected.0.positions) {
        assert_eq!(
            a.0.map(f64::to_bits),
            b.0.map(f64::to_bits),
            "position bits"
        );
    }
    for (a, b) in actual.0.normals.iter().zip(&expected.0.normals) {
        assert_eq!(a.0.map(f64::to_bits), b.0.map(f64::to_bits), "normal bits");
    }
    let quality = welded_mesh_quality(&actual.0);
    assert_eq!(quality.boundary_edges, 0);
    assert_eq!(quality.non_manifold_edges, 0);
    let mut winding = std::collections::BTreeMap::<(u32, u32), i32>::new();
    for tri in actual.0.indices.chunks_exact(3) {
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *winding.entry((a.min(b), a.max(b))).or_default() += if a < b { 1 } else { -1 };
        }
    }
    assert!(winding.values().all(|&sum| sum == 0), "consistent winding");
}

#[test]
fn filleted_face_drag_reuses_boundaries_and_cdt_without_changing_a_bit() {
    let _guard = Guard::new();
    let mut topo = Topology::new();
    let source = read_step(FIXTURE, &mut topo).unwrap()[0];
    let face = edited_face(&topo, source);
    let source_bytes = serialize_solids(&topo, &[source]).unwrap();
    // Keep several new solids in one arena: none of the changed faces has
    // the original face/edge handles. Reverse and repeated values exercise
    // exact-plan replay after incremental misses.
    let distances = [0.5, 1.0, 5.0, -5.0, -1.0, 0.5];
    let solids: Vec<_> = distances
        .into_iter()
        .map(|distance| move_faces(&mut topo, source, &[face], distance).unwrap())
        .collect();
    let expected: Vec<_> = std::iter::once(source)
        .chain(solids.iter().copied())
        .map(|solid| mesh(&topo, solid, DEFLECTION, ANGULAR))
        .collect();
    enable_mesh_reuse();
    same(&mesh(&topo, source, DEFLECTION, ANGULAR), &expected[0]);
    let before = mesh_reuse_stats().unwrap();
    for (&solid, expected) in solids.iter().zip(&expected[1..]) {
        same(&mesh(&topo, solid, DEFLECTION, ANGULAR), expected);
        assert_eq!(solid_faces(&topo, solid).unwrap().len(), 160);
    }
    same(
        &mesh(&topo, *solids.last().unwrap(), DEFLECTION, ANGULAR),
        expected.last().unwrap(),
    );
    let stats = mesh_reuse_stats().unwrap();
    assert!(
        stats.edge_hits > before.edge_hits + 1000,
        "unchanged shared edges are reused: {stats:?}"
    );
    assert!(
        stats.planar_hits > before.planar_hits,
        "unchanged holed planes are reused: {stats:?}"
    );
    assert!(
        stats.boundary_hits > before.boundary_hits,
        "repeated edit reuses the complete plan: {stats:?}"
    );
    assert!(
        stats.weld_hits > before.weld_hits,
        "mesh connectivity is reused: {stats:?}"
    );
    assert!(
        stats.weld_updated_triangles > 0,
        "changed triangles update the retained edge counts: {stats:?}"
    );
    assert!(
        stats.cleanup_hits > before.cleanup_hits,
        "coincident-triangle membership is reused: {stats:?}"
    );
    assert!(
        stats.cleanup_updated_triangles > 0,
        "changed triangles are qualified again: {stats:?}"
    );
    assert_eq!(
        stats.retained_bytes,
        stats.stage_bytes + stats.connectivity_bytes
    );
    assert!(stats.retained_bytes <= stats.max_bytes);
    assert!(stats.entries <= stats.max_entries);
    assert_eq!(serialize_solids(&topo, &[source]).unwrap(), source_bytes);
}

#[test]
fn rebuilt_arena_and_quality_changes_are_qualified_against_fresh_meshes() {
    let _guard = Guard::new();
    let mut first = Topology::new();
    let source = read_step(FIXTURE, &mut first).unwrap()[0];
    // Import a sacrificial body first, so identical geometry has different
    // handles and different gaps in the arena's index space.
    let mut second = Topology::new();
    let _padding = remus_operations::primitives::make_box(&mut second, 1.0, 2.0, 3.0).unwrap();
    let rebuilt = read_step(FIXTURE, &mut second).unwrap()[0];
    let qualities = [(DEFLECTION, ANGULAR), (0.05, 0.1), (DEFLECTION, 0.12)];
    let expected: Vec<_> = qualities
        .into_iter()
        .map(|(linear, angular)| mesh(&second, rebuilt, linear, angular))
        .collect();
    enable_mesh_reuse();
    mesh(&first, source, DEFLECTION, ANGULAR);
    let before = mesh_reuse_stats().unwrap();
    for ((linear, angular), expected) in qualities.into_iter().zip(&expected) {
        same(&mesh(&second, rebuilt, linear, angular), expected);
    }
    let stats = mesh_reuse_stats().unwrap();
    assert_eq!(
        stats.boundary_hits - before.boundary_hits,
        1,
        "only the matching quality may reuse a boundary plan"
    );
}
