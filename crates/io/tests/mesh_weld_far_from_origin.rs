//! IO-2 (kernel review 2026-10-14): the mesh weld bucketed vertices on a
//! `coordinate / tolerance` grid cast with `as i64`. Past `2^63·tolerance`
//! the cast saturates, so every coordinate lands in the same cell and the
//! weld degrades to a linear scan per vertex. A strip at x = 1e12 took 11.7×
//! the time for 4× the vertices, against 3.1× near the origin.
//!
//! This asserts the shape of the cost curve rather than a wall-clock number,
//! like `vertex_weld_cost_does_not_grow_with_mesh_size` near the origin: cost
//! per vertex must not grow with the mesh, wherever the mesh sits.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use std::time::Instant;

use remus_io::stl::import::import_mesh;
use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;

/// A k × k grid of quads offset to `origin`; all vertices distinct.
fn grid(k: usize, origin: f64) -> TriangleMesh {
    let mut positions = Vec::new();
    for j in 0..=k {
        for i in 0..=k {
            positions.push(Point3::new(origin + i as f64, origin + j as f64, origin));
        }
    }
    let idx = |i: usize, j: usize| u32::try_from(j * (k + 1) + i).unwrap();
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

fn per_vertex(k: usize, origin: f64) -> f64 {
    let mesh = grid(k, origin);
    let n = mesh.positions.len() as f64;
    // Best of three, so one scheduler hiccup cannot fail the bound.
    (0..3)
        .map(|_| {
            let mut topo = Topology::new();
            let started = Instant::now();
            let solid = import_mesh(&mut topo, &mesh, 1e-9);
            let elapsed = started.elapsed().as_secs_f64() / n;
            assert!(solid.is_err() || topo.num_vertices() == mesh.positions.len());
            elapsed
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn vertex_weld_cost_does_not_grow_far_from_the_origin() {
    // 1e12 / 1e-9 = 1e21 > 2^63: every axis saturated the old cell index.
    let small = per_vertex(40, 1e12);
    let large = per_vertex(160, 1e12);
    assert!(
        large < small * 8.0,
        "per-vertex weld cost at 1e12 grew from {small:.3e}s to {large:.3e}s over a 16x larger \
         mesh — the weld grid saturates far from the origin"
    );
}

/// Welding is unchanged far out: coincident copies merge, and distinct
/// floats one ulp apart (1.2e-4 at 1e12, far more than the tolerance) do not.
#[test]
fn far_vertices_weld_exactly_like_near_ones() {
    let mut mesh = grid(3, 1e12);
    let copies = mesh.positions.clone();
    let base = u32::try_from(copies.len()).unwrap();
    mesh.indices
        .extend(mesh.indices.clone().iter().map(|i| i + base));
    mesh.positions.extend(copies);
    mesh.normals.extend(mesh.normals.clone());
    let mut topo = Topology::new();
    let _ = import_mesh(&mut topo, &mesh, 1e-9);
    assert_eq!(
        topo.num_vertices(),
        16,
        "duplicated far grid must weld to 16 vertices"
    );
}
