//! The sorted and vertex-bucketed edge censuses behind the whole-solid mesh
//! route return what the hash-map censuses they replaced returned.
//!
//! Both references are verbatim from before the change. They are compared on
//! random index soups (repeated, reversed and degenerate edges, small vertex
//! pools so edges recur three or more times) and on the meshes the volume
//! route actually reads.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use remus_math::det_hash::DetHashMap;
use remus_math::mat::Mat4;

/// `mesh_boundary_edge_count` before the sort, verbatim.
fn reference_boundary_edge_count(mesh: &tessellate::TriangleMesh) -> usize {
    let mut counts: DetHashMap<(u32, u32), usize> = DetHashMap::default();
    for tri in mesh.indices.chunks_exact(3) {
        for &(i, j) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if i < j { (i, j) } else { (j, i) };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    counts.values().filter(|&&c| c != 2).count()
}

/// `directed_edges_cancel` before the sort, verbatim.
fn reference_edges_cancel<K: Copy + Ord + std::hash::Hash>(
    triangles: impl IntoIterator<Item = [K; 3]>,
) -> bool {
    let mut net: DetHashMap<(K, K), i64> = DetHashMap::default();
    for [a, b, c] in triangles {
        for (p, q) in [(a, b), (b, c), (c, a)] {
            match p.cmp(&q) {
                std::cmp::Ordering::Less => *net.entry((p, q)).or_insert(0) += 1,
                std::cmp::Ordering::Greater => *net.entry((q, p)).or_insert(0) -= 1,
                std::cmp::Ordering::Equal => {}
            }
        }
    }
    net.values().all(|&n| n == 0)
}

fn triangles(mesh: &tessellate::TriangleMesh) -> Vec<[u32; 3]> {
    mesh.indices
        .chunks_exact(3)
        .map(|t| [t[0], t[1], t[2]])
        .collect()
}

fn assert_census_matches(mesh: &tessellate::TriangleMesh, what: &str) {
    assert_eq!(
        mesh_boundary_edge_count(mesh),
        reference_boundary_edge_count(mesh),
        "{what}: boundary edge count"
    );
    assert_eq!(
        directed_edges_cancel(triangles(mesh)),
        reference_edges_cancel(triangles(mesh)),
        "{what}: edge cancellation"
    );
    assert_eq!(
        mesh_directed_edges_cancel(mesh),
        reference_edges_cancel(triangles(mesh)),
        "{what}: bucketed edge cancellation"
    );
}

#[test]
fn edge_censuses_match_the_hash_map_references_on_index_soups() {
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = |n: u32| {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        #[allow(clippy::cast_possible_truncation)]
        let r = (state % u64::from(n)) as u32;
        r
    };
    let mut cancelled = 0;
    let mut open = 0;
    for case in 0..2000 {
        let pool = 3 + next(12);
        let mut mesh = tessellate::TriangleMesh::default();
        for _ in 0..next(24) {
            let t = [next(pool), next(pool), next(pool)];
            mesh.indices.extend_from_slice(&t);
            // Often add the reversed twin, so closed and cancelling soups
            // occur, not just open ones.
            if next(3) != 0 {
                mesh.indices.extend_from_slice(&[t[0], t[2], t[1]]);
            }
        }
        if reference_edges_cancel(triangles(&mesh)) {
            cancelled += 1;
        }
        if reference_boundary_edge_count(&mesh) > 0 {
            open += 1;
        }
        assert_census_matches(&mesh, &format!("soup {case}"));
    }
    assert!(cancelled > 100, "cancelling soups {cancelled}");
    assert!(open > 100, "open soups {open}");
}

#[test]
fn edge_censuses_match_the_hash_map_references_on_solid_meshes() {
    let mut topo = Topology::new();
    let block = crate::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(&topo, block).unwrap();
    let chamfered = crate::chamfer::chamfer(&mut topo, block, &edges, 1.0).unwrap();
    let mut plate = crate::primitives::make_box(&mut topo, 40.0, 40.0, 5.0).unwrap();
    for (x, y) in [(10.0, 10.0), (30.0, 10.0), (20.0, 30.0)] {
        let pin = crate::primitives::make_cylinder(&mut topo, 3.0, 10.0).unwrap();
        crate::transform::transform_solid(&mut topo, pin, &Mat4::translation(x, y, -2.0)).unwrap();
        plate =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, plate, pin).unwrap();
    }
    let ball = crate::primitives::make_sphere(&mut topo, 7.0, 16).unwrap();
    for (what, solid) in [
        ("chamfered box", chamfered),
        ("drilled plate", plate),
        ("sphere", ball),
    ] {
        for deflection in [0.5, 0.05, 2e-3] {
            let mesh =
                tessellate::tessellate_solid_for_measurement(&topo, solid, deflection).unwrap();
            assert_eq!(mesh_boundary_edge_count(&mesh), 0, "{what} closes");
            assert_census_matches(&mesh, what);
            // Ten triangles removed open the mesh.
            let mut open = mesh.clone();
            open.indices.truncate(open.indices.len() - 30);
            assert!(mesh_boundary_edge_count(&open) > 0, "{what} opens");
            assert_census_matches(&open, what);
        }
    }
}
