//! PERF-D01 on the Hammer Holder: the display mesh of a direct edit reuses
//! the source body's per-face meshes and stays identical to a fresh mesh.
//!
//! The consumer (OpenZCAD) re-meshes the whole body after every direct edit
//! at its display tolerances. After a planar face move most faces are
//! untouched and the moved region is a rigid translation, so the per-face
//! mesh cache serves them; this pins that the result is the fresh mesh
//! (indices, face offsets and normals identical, positions within roundoff)
//! and watertight.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::reader::read_step;
use remus_operations::measure::face_area;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{
    FaceMeshCacheStats, TriangleMesh, boundary_edge_count, disable_face_mesh_cache,
    enable_face_mesh_cache, face_mesh_cache_stats, non_manifold_edge_count,
    tessellate_solid_grouped_with_tolerance,
};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const HAMMER_HOLDER: &str = include_str!("data/shapr3d_hammer_holder.step");
/// OpenZCAD's display tolerances for this body (2e-4 x its largest extent,
/// 0.06 rad).
const DEFLECTION: f64 = 0.0148;
const ANGULAR: f64 = 0.06;

fn mesh(topo: &Topology, solid: SolidId) -> (TriangleMesh, Vec<u32>) {
    tessellate_solid_grouped_with_tolerance(topo, solid, DEFLECTION, ANGULAR).unwrap()
}

fn stats() -> FaceMeshCacheStats {
    face_mesh_cache_stats().expect("cache enabled")
}

/// The +X planar face OpenZCAD's offset harness edits (area 1045.93).
fn edited_face(topo: &Topology, solid: SolidId) -> FaceId {
    let mut best = None;
    for face in solid_faces(topo, solid).unwrap() {
        if let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface()
            && normal.x() > 0.99
        {
            let error = (face_area(topo, face, 0.05).unwrap() - 1045.93).abs();
            if best.is_none_or(|(_, e)| error < e) {
                best = Some((face, error));
            }
        }
    }
    let (face, error) = best.expect("a +X planar face");
    assert!(error < 1.0, "edited face area is off by {error}");
    face
}

#[test]
fn offset_face_edit_reuses_source_meshes_exactly() {
    let mut topo = Topology::new();
    let solid = read_step(HAMMER_HOLDER, &mut topo).unwrap()[0];
    let face = edited_face(&topo, solid);
    let moved = move_faces(&mut topo, solid, &[face], -6.0).unwrap();

    disable_face_mesh_cache();
    let source_fresh = mesh(&topo, solid);
    let moved_fresh = mesh(&topo, moved);

    enable_face_mesh_cache();
    let cold = mesh(&topo, solid);
    assert_eq!(
        cold.0.indices, source_fresh.0.indices,
        "cold capture changed the mesh"
    );
    let s0 = stats();
    let reused = mesh(&topo, moved);
    let s1 = stats();
    disable_face_mesh_cache();

    assert_eq!(reused.1, moved_fresh.1, "face offsets differ");
    assert_eq!(reused.0.indices, moved_fresh.0.indices, "triangles differ");
    assert_eq!(reused.0.positions.len(), moved_fresh.0.positions.len());
    let scale = moved_fresh
        .0
        .positions
        .iter()
        .flat_map(|p| [p.x().abs(), p.y().abs(), p.z().abs()])
        .fold(1.0, f64::max);
    let tol = 64.0 * f64::EPSILON * scale;
    for (a, b) in reused.0.positions.iter().zip(&moved_fresh.0.positions) {
        assert!((*a - *b).length() <= tol, "vertex moved: {a:?} vs {b:?}");
    }
    for (a, b) in reused.0.normals.iter().zip(&moved_fresh.0.normals) {
        assert_eq!(
            (a.x().to_bits(), a.y().to_bits(), a.z().to_bits()),
            (b.x().to_bits(), b.y().to_bits(), b.z().to_bits()),
            "normal differs"
        );
    }
    assert_eq!(boundary_edge_count(&reused.0), 0);
    assert_eq!(non_manifold_edge_count(&reused.0), 0);

    // 160 faces; the 6 holed planar faces are CDT jobs outside the cache.
    let lookups = s1.lookups - s0.lookups;
    let exact = s1.exact_hits - s0.exact_hits;
    let translated = s1.translated_hits - s0.translated_hits;
    let misses = s1.misses - s0.misses;
    let refused = s1.translation_refused - s0.translation_refused;
    assert_eq!(lookups, 154);
    assert_eq!(exact + translated + misses, lookups);
    // Every face the edit did not touch is served exactly, and part of the
    // moved region is served by translation.
    assert!(exact >= 120, "only {exact} exact hits");
    assert!(translated >= 1, "no translated hit");
    assert!(refused <= misses);
    assert_eq!(s1.replay_conflicts - s0.replay_conflicts, 0);
    assert_eq!(s1.uncacheable - s0.uncacheable, 0);
}
