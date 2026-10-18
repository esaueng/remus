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
fn connectivity_changes_miss_and_preserve_face_attribution() {
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
    assert_eq!(session.cache.as_ref().unwrap().counters.weld_hits, 0);
}

#[test]
fn unfit_quantity_admission_preserves_display_and_other_quantity_work() {
    let mut cache = Cache {
        max_entries: 2,
        max_bytes: 4096,
        ..Cache::default()
    };
    cache.insert(vec![1], Value::Clean, RetentionClass::Display);
    cache.insert(vec![2], Value::Clean, RetentionClass::Quantity);
    cache.insert(
        vec![3],
        Value::Weld(vec![0; 1024]),
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
    cache.insert(vec![1], Value::Clean, RetentionClass::Display);
    assert!(cache.entries.is_empty());
    cache.max_entries = 1;
    cache.max_bytes = 32;
    cache.insert(vec![1], Value::Clean, RetentionClass::Display);
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
