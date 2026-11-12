//! [`edge_census`] returns what the hash-set boundary count and the hash-map
//! non-manifold count it replaced returned.
//!
//! Both references are verbatim from before the change. They are compared on
//! random index soups (repeated, reversed and degenerate sides, small vertex
//! pools so edges recur three or more times, indices past the vertex list)
//! and on whole-solid meshes.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;

/// `boundary_edge_count` before the census, verbatim.
fn reference_boundary_edge_count(mesh: &TriangleMesh) -> usize {
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

/// `non_manifold_edge_count` before the census, verbatim.
fn reference_non_manifold_edge_count(mesh: &TriangleMesh) -> usize {
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

fn assert_matches(mesh: &TriangleMesh, what: &str) {
    let census = edge_census(mesh);
    assert_eq!(
        census.boundary_edges,
        reference_boundary_edge_count(mesh),
        "{what}: boundary edges"
    );
    assert_eq!(
        census.non_manifold_edges,
        reference_non_manifold_edge_count(mesh),
        "{what}: non-manifold edges"
    );
    assert_eq!(boundary_edge_count(mesh), census.boundary_edges);
    assert_eq!(non_manifold_edge_count(mesh), census.non_manifold_edges);
    assert_eq!(
        is_watertight(mesh),
        !mesh.indices.is_empty()
            && mesh.indices.len().is_multiple_of(3)
            && census.boundary_edges == 0
            && census.non_manifold_edges == 0,
        "{what}: watertight"
    );
}

fn soup(seed: u64, triangles: usize, pool: u32, positions: usize, tail: usize) -> TriangleMesh {
    let mut state = seed;
    let mut next = || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        state >> 33
    };
    let mut indices = Vec::new();
    for _ in 0..triangles {
        // Mix in degenerate sides and repeats of earlier triangles, both
        // ways round.
        match next() % 8 {
            0 if indices.len() >= 3 => {
                let t = (next() as usize % (indices.len() / 3)) * 3;
                let tri: [u32; 3] = [indices[t], indices[t + 1], indices[t + 2]];
                indices.extend([tri[0], tri[2], tri[1]]);
            }
            1 if indices.len() >= 3 => {
                let t = (next() as usize % (indices.len() / 3)) * 3;
                let tri: [u32; 3] = [indices[t], indices[t + 1], indices[t + 2]];
                indices.extend(tri);
            }
            2 => {
                let a = (next() % u64::from(pool)) as u32;
                let b = (next() % u64::from(pool)) as u32;
                indices.extend([a, a, b]);
            }
            _ => {
                for _ in 0..3 {
                    indices.push((next() % u64::from(pool)) as u32);
                }
            }
        }
    }
    for _ in 0..tail {
        indices.push((next() % u64::from(pool)) as u32);
    }
    TriangleMesh {
        positions: vec![Point3::new(0.0, 0.0, 0.0); positions],
        normals: Vec::new(),
        indices,
    }
}

#[test]
fn census_matches_the_hash_counts_on_index_soups() {
    for seed in 0..400_u64 {
        let pool = 3 + (seed % 40) as u32;
        let triangles = (seed as usize * 7) % 300;
        let tail = (seed % 3) as usize;
        // Vertex lists that hold every index, and ones that do not (the
        // whole-list sort).
        for positions in [pool as usize, (pool / 2) as usize, 0] {
            let mesh = soup(seed, triangles, pool, positions, tail);
            assert_matches(
                &mesh,
                &format!("seed {seed}, pool {pool}, {positions} positions"),
            );
        }
    }
}

#[test]
fn census_matches_on_extreme_indices_and_empty_meshes() {
    let mut mesh = TriangleMesh {
        positions: Vec::new(),
        normals: Vec::new(),
        indices: Vec::new(),
    };
    assert_matches(&mesh, "empty");
    mesh.indices = vec![u32::MAX, 0, u32::MAX - 1, 0, u32::MAX, 7, 1];
    assert_matches(&mesh, "extreme indices");
    mesh.indices = vec![5, 5, 5];
    mesh.positions = vec![Point3::new(0.0, 0.0, 0.0); 6];
    assert_matches(&mesh, "one degenerate triangle");
}

#[test]
fn census_matches_on_solid_meshes() {
    use crate::primitives::{make_box, make_cylinder, make_sphere, make_torus};
    let mut topo = Topology::new();
    let solids = [
        make_box(&mut topo, 2.0, 3.0, 4.0).unwrap(),
        make_cylinder(&mut topo, 1.5, 4.0).unwrap(),
        make_sphere(&mut topo, 2.0, 24).unwrap(),
        make_torus(&mut topo, 3.0, 1.0, 24).unwrap(),
    ];
    for solid in solids {
        for deflection in [0.2, 0.01] {
            let mesh = crate::tessellate::tessellate_solid(&topo, solid, deflection).unwrap();
            assert_matches(&mesh, &format!("solid {} at {deflection}", solid.index()));
            // Opened and doubled copies.
            let mut open = mesh.clone();
            open.indices.truncate(open.indices.len() - 6);
            assert_matches(&open, "opened");
            let mut doubled = mesh.clone();
            doubled.indices.extend_from_slice(&mesh.indices[..9]);
            assert_matches(&doubled, "doubled");
        }
    }
}
