//! Independent fresh-pass and mutation oracles for retained mesh stages.
#![allow(clippy::unwrap_used, clippy::expect_used)]
use super::*;
use crate::tessellate::{TriangleMesh, mesh_ops};
use remus_math::vec::{Point3, Vec3};

fn session() -> Session {
    Session {
        cache: Some(Cache {
            max_entries: 64,
            max_bytes: 4 * 1024 * 1024,
            ..Cache::default()
        }),
        retention: RetentionClass::Display,
    }
}
fn same(actual: &TriangleMesh, expected: &TriangleMesh) {
    assert_eq!(actual.indices, expected.indices);
    assert_eq!(actual.positions.len(), expected.positions.len());
    assert_eq!(actual.normals.len(), expected.normals.len());
    for (a, b) in actual.positions.iter().zip(&expected.positions) {
        assert_eq!(a.0.map(f64::to_bits), b.0.map(f64::to_bits));
    }
    for (a, b) in actual.normals.iter().zip(&expected.normals) {
        assert_eq!(a.0.map(f64::to_bits), b.0.map(f64::to_bits));
    }
}
fn two_triangles(z: f64) -> TriangleMesh {
    TriangleMesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, z),
            Point3::new(1.0, 0.0, z),
            Point3::new(0.0, 1.0, z),
        ],
        normals: vec![Vec3::new(0.0, 0.0, 1.0); 6],
        indices: vec![0, 1, 2, 3, 5, 4],
    }
}

#[test]
fn new_coincident_opposite_faces_cannot_use_an_old_clean_certificate() {
    let mut session = session();
    let mut separated = two_triangles(1.0);
    assert!(!mesh_ops::dedupe_coincident_triangles_cached(
        &mut separated,
        None,
        Some(&mut session)
    ));
    let mut repeated = two_triangles(1.0);
    assert!(!mesh_ops::dedupe_coincident_triangles_cached(
        &mut repeated,
        None,
        Some(&mut session)
    ));
    assert_eq!(session.cache.as_ref().unwrap().counters.cleanup_hits, 1);
    let mut actual = two_triangles(0.0);
    let mut expected = actual.clone();
    let mut faces = vec![10, 11];
    let mut expected_faces = faces.clone();
    assert!(mesh_ops::dedupe_coincident_triangles(
        &mut expected,
        Some(&mut expected_faces)
    ));
    assert!(mesh_ops::dedupe_coincident_triangles_cached(
        &mut actual,
        Some(&mut faces),
        Some(&mut session)
    ));
    same(&actual, &expected);
    assert!(actual.indices.is_empty(), "opposite faces cancel");
    assert!(faces.is_empty());
    assert_eq!(faces, expected_faces);
    assert_eq!(session.cache.as_ref().unwrap().counters.cleanup_hits, 1);
}

#[test]
fn cached_boundary_connectivity_still_rechecks_distances_and_deflection() {
    let mut session = session();
    for (z, deflection) in [(1.0, 0.01), (0.001, 0.01), (0.001, 0.0001), (1.0, 0.01)] {
        let mut actual = two_triangles(z);
        let mut expected = actual.clone();
        let mut faces = vec![4, 9];
        let mut expected_faces = faces.clone();
        let expected_boundary =
            mesh_ops::weld_boundary_vertices(&mut expected, deflection, Some(&mut expected_faces));
        let actual_boundary = mesh_ops::weld_boundary_vertices_cached(
            &mut actual,
            deflection,
            Some(&mut faces),
            Some(&mut session),
        );
        assert_eq!(actual_boundary, expected_boundary);
        same(&actual, &expected);
        assert_eq!(faces, expected_faces);
        if z < 0.01 && deflection > 0.001 {
            assert_eq!(actual.indices, vec![0, 1, 2, 0, 2, 1]);
        } else {
            assert_eq!(actual.indices, vec![0, 1, 2, 3, 5, 4]);
        }
    }
    assert_eq!(session.cache.as_ref().unwrap().counters.weld_hits, 3);
}

#[test]
fn connectivity_changes_update_and_preserve_face_attribution() {
    let mut session = session();
    for indices in [
        vec![0, 1, 2, 3, 5, 4],
        vec![0, 1, 2, 3, 4, 5],
        vec![0, 1, 2],
    ] {
        let mut actual = two_triangles(0.001);
        actual.indices = indices;
        let mut expected = actual.clone();
        let expected_boundary = mesh_ops::weld_boundary_vertices(&mut expected, 0.01, None);
        let actual_boundary =
            mesh_ops::weld_boundary_vertices_cached(&mut actual, 0.01, None, Some(&mut session));
        assert_eq!(actual_boundary, expected_boundary);
        same(&actual, &expected);
    }
    assert_eq!(session.cache.as_ref().unwrap().counters.weld_hits, 1);
    assert_eq!(
        session
            .cache
            .as_ref()
            .unwrap()
            .counters
            .weld_updated_triangles,
        1
    );
}

#[test]
fn unfit_quantity_admission_preserves_display_and_other_quantity_work() {
    let mut cache = Cache {
        max_entries: 2,
        max_bytes: 4096,
        ..Cache::default()
    };
    cache.insert(
        vec![1],
        Value::Planar((Vec::new(), Vec::new())),
        RetentionClass::Display,
    );
    cache.insert(
        vec![2],
        Value::Planar((Vec::new(), Vec::new())),
        RetentionClass::Quantity,
    );
    cache.insert(
        vec![3],
        Value::Edge(vec![Point3::new(0.0, 0.0, 0.0); 1024], Vec::new()),
        RetentionClass::Quantity,
    );
    assert!(cache.get(&[1], RetentionClass::Quantity).is_some());
    assert!(cache.get(&[2], RetentionClass::Quantity).is_some());
    assert_eq!(cache.counters.evictions, 0);
    cache.max_entries = 1;
    assert!(!cache.make_room(100, 1, RetentionClass::Quantity));
    assert_eq!(
        cache.entries.len(),
        2,
        "failed admission does not evict anything"
    );
    assert!(cache.make_room(0, 0, RetentionClass::Display));
    assert!(cache.get(&[1], RetentionClass::Display).is_some());
    assert!(cache.get(&[2], RetentionClass::Quantity).is_none());
}

#[test]
fn zero_bounds_and_oversize_captures_retain_nothing() {
    let mut cache = Cache::default();
    cache.insert(
        vec![1],
        Value::Planar((Vec::new(), Vec::new())),
        RetentionClass::Display,
    );
    assert!(cache.entries.is_empty());
    cache.max_entries = 1;
    cache.max_bytes = 32;
    cache.insert(
        vec![1],
        Value::Planar((Vec::new(), Vec::new())),
        RetentionClass::Display,
    );
    assert!(cache.entries.is_empty());
    assert_eq!(cache.bytes, 0);
}

#[test]
fn curve_mutation_and_restoration_never_serve_old_samples() {
    use crate::primitives::make_cylinder;
    use crate::tessellate::edge_sampling::sample_edge_with_params;
    use remus_topology::edge::EdgeCurve;
    use remus_topology::explorer::solid_edges;
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 5.0).unwrap();
    let edge_id = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Circle(_)))
        .unwrap();
    let edge = topo.edge(edge_id).unwrap().clone();
    let mut session = session();
    let key = edge_key(&topo, edge_id, 0.01, 0.1, false).unwrap();
    let (points, params) = sample_edge_with_params(&topo, &edge, 0.01, 0.1, false).unwrap();
    session.store_edge(key.clone(), &points, &params);
    assert!(session.edge(&key).is_some());
    // An excessive authoritative trim must still reach the existing sample
    // budget refusal even though the original edge has a retained chain.
    topo.edge_mut(edge_id).unwrap().set_trim(Some((0.0, 1e12)));
    let changed = edge_key(&topo, edge_id, 0.01, 0.1, false).unwrap();
    assert!(session.edge(&changed).is_none());
    assert!(sample_edge_with_params(&topo, topo.edge(edge_id).unwrap(), 0.01, 0.1, false).is_err());
    *topo.edge_mut(edge_id).unwrap() = edge;
    assert_eq!(edge_key(&topo, edge_id, 0.01, 0.1, false).unwrap(), key);
    let restored = session.edge(&key).unwrap();
    assert_eq!(restored.0.len(), points.len());
    for (actual, expected) in restored.0.iter().zip(points) {
        assert_eq!(actual.0.map(f64::to_bits), expected.0.map(f64::to_bits));
    }
}

#[test]
fn changed_triangles_recheck_duplicate_membership_and_opposing_winding() {
    for replacement in [[0, 1, 3], [0, 1, 2], [0, 2, 1]] {
        let mut session = session();
        let mut seed = two_triangles(1.0);
        assert!(!mesh_ops::dedupe_coincident_triangles_cached(
            &mut seed,
            None,
            Some(&mut session)
        ));
        let mut actual = two_triangles(1.0);
        actual.indices[3..].copy_from_slice(&replacement);
        let mut expected = actual.clone();
        let mut faces = vec![10, 11];
        let mut expected_faces = faces.clone();
        let fresh = mesh_ops::dedupe_coincident_triangles(&mut expected, Some(&mut expected_faces));
        let reused = mesh_ops::dedupe_coincident_triangles_cached(
            &mut actual,
            Some(&mut faces),
            Some(&mut session),
        );
        assert_eq!(reused, fresh);
        same(&actual, &expected);
        assert_eq!(faces, expected_faces);
        match replacement {
            [0, 1, 3] => {
                assert_eq!(actual.indices.len(), 6);
                assert_eq!(
                    session
                        .cache
                        .as_ref()
                        .unwrap()
                        .counters
                        .cleanup_updated_triangles,
                    1
                );
            }
            [0, 1, 2] => assert_eq!(actual.indices.len(), 3),
            _ => assert!(actual.indices.is_empty()),
        }
    }
}

#[test]
fn streamed_triangle_edits_match_fresh_cleanup_with_repeated_directed_edges() {
    let mut session = session();
    for step in 0_u32..60 {
        let mut actual = TriangleMesh {
            positions: (0..25)
                .map(|i| Point3::new(f64::from(i % 5), f64::from(i / 5), 0.0))
                .collect(),
            normals: vec![Vec3::new(0.0, 0.0, 1.0); 25],
            indices: Vec::new(),
        };
        for i in 0_u32..16 {
            let a = (i / 4) * 5 + i % 4;
            let (b, c) = if (i + step).is_multiple_of(3) {
                (a + 6, a + 1)
            } else {
                (a + 1, a + 6)
            };
            actual.indices.extend([a, b, c, a, a + 6, a + 5]);
        }
        // Mix non-manifold duplicates, reverse edges and one changed corner.
        if step % 4 == 1 {
            actual.indices[5] = actual.indices[2];
        }
        if step % 4 == 2 {
            actual.indices[9..12].copy_from_slice(&[0, 1, 6]);
        }
        let mut expected = actual.clone();
        let mut faces: (Vec<u32>, Vec<u32>) = ((0..32).collect(), (0..32).collect());
        assert_eq!(
            mesh_ops::weld_boundary_vertices_cached(
                &mut actual,
                0.001,
                Some(&mut faces.0),
                Some(&mut session)
            ),
            mesh_ops::weld_boundary_vertices(&mut expected, 0.001, Some(&mut faces.1))
        );
        assert_eq!(
            mesh_ops::dedupe_coincident_triangles_cached(
                &mut actual,
                Some(&mut faces.0),
                Some(&mut session)
            ),
            mesh_ops::dedupe_coincident_triangles(&mut expected, Some(&mut faces.1))
        );
        same(&actual, &expected);
        assert_eq!(faces.0, faces.1);
    }
    let stats = &session.cache.as_ref().unwrap().counters;
    assert!(stats.weld_hits > 50);
    assert!(stats.weld_updated_triangles > 0);
}
