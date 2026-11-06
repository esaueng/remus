//! WASM contract test for the r 2 fillet on the seam of an r 6 boss fused
//! into an r 73.5 sphere (esaueng/remus#1561, OpenZCAD K02/W01), built the
//! way the consumer builds it: batch primitives, a fuse, a `fillet` on the
//! four seam edges, and `tessellateSolidGroupedBinary` at the consumer's
//! display settings.
//!
//! Before, the closed walk's contacts were chord polylines that sat up to
//! 0.11 mm off the boss while recording 1e-7, and the display mesh had 286
//! open edges on the sphere's equator. The native suite
//! (`crates/operations/tests/regress_sphere_boss_seam_fillet.rs`) carries the
//! independent volume estimate; this pins the binding's output.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::face::FaceSurface;

use crate::kernel::RemusKernel;

fn run(kernel: &mut RemusKernel, ops: &serde_json::Value) -> Vec<serde_json::Value> {
    serde_json::from_str(&kernel.execute_batch(&ops.to_string())).unwrap()
}

fn ok_handle(out: &serde_json::Value) -> u32 {
    let handle = out["ok"]
        .as_u64()
        .unwrap_or_else(|| panic!("expected an ok handle, got {out}"));
    u32::try_from(handle).unwrap()
}

/// The fused sphere and boss (tilted 20° about X and 10° about Y when
/// `tilted`), as a batch handle. A fresh kernel hands out the sphere as
/// handle 0 and the cylinder as 1; `transform` moves a solid in place.
fn sphere_boss(kernel: &mut RemusKernel, tilted: bool) -> u32 {
    let boss = if tilted {
        let (a, b) = (20f64.to_radians(), 10f64.to_radians());
        // translation(0, 0, 58) · rotation_y(b) · rotation_x(a), row-major.
        serde_json::json!([
            b.cos(),
            b.sin() * a.sin(),
            b.sin() * a.cos(),
            0.0,
            0.0,
            a.cos(),
            -a.sin(),
            0.0,
            -b.sin(),
            b.cos() * a.sin(),
            b.cos() * a.cos(),
            58.0,
            0.0,
            0.0,
            0.0,
            1.0
        ])
    } else {
        serde_json::json!([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 58, 0, 0, 0, 1])
    };
    let out = run(
        kernel,
        &serde_json::json!([
            {"op": "makeSphere", "args": {"radius": 73.5, "segments": 32}},
            {"op": "transform", "args": {"solid": 0, "matrix": [
                1, 0, 0, 25.5,
                0, 1, 0, -1.5,
                0, 0, 1, 0,
                0, 0, 0, 1
            ]}},
            {"op": "makeCylinder", "args": {"radius": 6, "height": 64.1}},
            {"op": "transform", "args": {"solid": 1, "matrix": boss}},
            {"op": "fuse", "args": {"solidA": 0, "solidB": 1}},
        ]),
    );
    ok_handle(&out[4])
}

/// Batch handles of the edges between the boss wall and the sphere.
fn seam_edges(kernel: &RemusKernel, solid: u32) -> Vec<u32> {
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let adjacency = AdjacencyIndex::build(kernel.topo(), solid_id).unwrap();
    let handles = kernel.get_solid_edges(solid).unwrap();
    let seams: Vec<u32> = handles
        .into_iter()
        .filter(|&handle| {
            let edge = kernel.resolve_edge(handle).unwrap();
            let faces = adjacency.faces_for_edge(edge);
            let kind = |surface: &str| {
                faces
                    .iter()
                    .any(|&face| kernel.topo().face(face).unwrap().surface().type_tag() == surface)
            };
            faces.len() == 2 && kind("sphere") && kind("cylinder")
        })
        .collect();
    assert_eq!(
        seams.len(),
        4,
        "the fuse leaves four boss/sphere seam edges"
    );
    seams
}

/// `(open, non-manifold, inconsistently wound)` edges of an indexed mesh.
fn closure(indices: &[u32]) -> (usize, usize, usize) {
    let mut uses: BTreeMap<(u32, u32), (usize, i32)> = BTreeMap::new();
    for triangle in indices.chunks_exact(3) {
        for k in 0..3 {
            let (a, b) = (triangle[k], triangle[(k + 1) % 3]);
            let entry = uses.entry((a.min(b), a.max(b))).or_default();
            entry.0 += 1;
            entry.1 += if a < b { 1 } else { -1 };
        }
    }
    let mut counts = (0, 0, 0);
    for &(count, balance) in uses.values() {
        match count {
            1 => counts.0 += 1,
            2 if balance != 0 => counts.2 += 1,
            2 => {}
            _ => counts.1 += 1,
        }
    }
    counts
}

#[test]
fn sphere_boss_seam_fillet_display_mesh_is_closed() {
    for tilted in [false, true] {
        let mut kernel = RemusKernel::new();
        let fused = sphere_boss(&mut kernel, tilted);
        let edges = seam_edges(&kernel, fused);
        let out = run(
            &mut kernel,
            &serde_json::json!([
                {"op": "fillet", "args": {"solid": fused, "radius": 2.0, "edges": edges}}
            ]),
        );
        let filleted = ok_handle(&out[0]);
        assert_ne!(
            filleted, fused,
            "tilted={tilted}: the fillet returns a new handle"
        );
        assert_eq!(
            kernel.validate_solid(filleted).unwrap(),
            0,
            "tilted={tilted}: no validation errors"
        );

        // OpenZCAD's display settings, then the issue's other rows.
        for (deflection, angular) in [(0.0294, 0.06), (0.1, 0.06), (0.01, 0.06), (0.0294, 0.5)] {
            let mesh = kernel
                .tessellate_solid_grouped_binary(filleted, deflection, Some(angular))
                .unwrap();
            // Before: 286 open edges at the display settings.
            assert_eq!(
                closure(&mesh.indices()),
                (0, 0, 0),
                "tilted={tilted} at {deflection} / {angular}: (open, non-manifold, flipped) edges"
            );
        }

        // Every walked contact lies on its support within the tolerance its
        // edge records (before: 9.2e-2 off the boss, recording 1e-7).
        let solid_id = kernel.resolve_solid(filleted).unwrap();
        let topo = kernel.topo();
        let adjacency = AdjacencyIndex::build(topo, solid_id).unwrap();
        let mut checked = 0;
        for (edge, faces) in adjacency.edge_faces_iter() {
            let data = topo.edge(edge).unwrap();
            let surfaces: Vec<&FaceSurface> = faces
                .iter()
                .map(|&face| topo.face(face).unwrap().surface())
                .collect();
            if !surfaces.iter().any(|s| matches!(s, FaceSurface::Nurbs(_))) {
                continue;
            }
            let start = topo.vertex(data.start()).unwrap().point();
            let end = topo.vertex(data.end()).unwrap().point();
            let (t0, t1) = data.strict_domain().unwrap();
            for support in surfaces
                .iter()
                .filter(|s| matches!(s, FaceSurface::Sphere(_) | FaceSurface::Cylinder(_)))
            {
                let deviation = (0..=2000)
                    .map(|i| {
                        let t = t0 + (t1 - t0) * f64::from(i) / 2000.0;
                        let point = data.curve().evaluate_with_endpoints(t, start, end);
                        let (u, v) = support.project_point(point).unwrap();
                        (support.evaluate(u, v).unwrap() - point).length()
                    })
                    .fold(0.0_f64, f64::max);
                let tolerance = data.tolerance().unwrap_or(1e-7);
                assert!(
                    deviation <= tolerance,
                    "tilted={tilted}: contact {edge:?} lies {deviation:e} off its {} support, \
                     recording {tolerance:e}",
                    support.type_tag()
                );
                checked += 1;
            }
        }
        assert!(checked >= 2, "tilted={tilted}: both contacts were measured");
    }
}
