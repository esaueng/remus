//! PERF-Q02: arena serialization round trips never alias persistent
//! classification preparation.
//!
//! Runtime cache identity stays out of the document bytes; deserializing
//! into a fresh or existing topology yields a different lineage or a bumped
//! generation, so the same numeric solid index never reuses stale bounds,
//! BVH, or trims.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_check::classify::{ClassificationCache, ClassifyOptions, PointClassification};
use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_math::vec::Point3;
use remus_topology::Topology;

#[test]
fn arena_round_trip_into_fresh_topology_never_reuses_preparation() {
    let mut source = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut source, 2.0, 2.0, 2.0).unwrap();
    let options = ClassifyOptions::default();
    let inside = Point3::new(1.0, 1.0, 1.0);

    let mut cache = ClassificationCache::new();
    assert_eq!(
        cache
            .classify_point(&source, solid, inside, &options)
            .unwrap(),
        PointClassification::Inside
    );
    assert_eq!(cache.stats().rebuilds, 1);

    let bytes = serialize_solids(&source, &[solid]).unwrap();
    // Runtime identity is not part of the document: the same bytes load
    // into a fresh lineage.
    let mut destination = Topology::new();
    assert_ne!(destination.cache_lineage(), source.cache_lineage());
    let restored = deserialize_solids(&bytes, &mut destination).unwrap();
    assert_eq!(restored.len(), 1);

    // Same numeric index, different lineage: miss, correct verdict, and the
    // source entry is still live for its own document.
    assert_eq!(
        cache
            .classify_point(&destination, restored[0], inside, &options)
            .unwrap(),
        PointClassification::Inside
    );
    assert_eq!(cache.stats().rebuilds, 2);
    assert_eq!(cache.stats().len, 2);
    assert_eq!(
        cache
            .classify_point(&source, solid, inside, &options)
            .unwrap(),
        PointClassification::Inside
    );
    assert_eq!(cache.stats().hits, 1);
}

#[test]
fn arena_append_into_existing_topology_bumps_generation() {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let options = ClassifyOptions::default();
    let point = Point3::new(1.0, 1.0, 1.0);

    let mut cache = ClassificationCache::new();
    cache.classify_point(&topo, solid, point, &options).unwrap();
    assert_eq!(cache.stats().rebuilds, 1);

    let bytes = serialize_solids(&topo, &[solid]).unwrap();
    let generation_before = topo.cache_generation();
    let restored = deserialize_solids(&bytes, &mut topo).unwrap();
    assert_eq!(restored.len(), 1);
    assert!(topo.cache_generation() > generation_before);

    // Prior preparation is stale (whole-topology invalidation): miss, but
    // both the original and the appended copy still classify correctly.
    assert_eq!(
        cache.classify_point(&topo, solid, point, &options).unwrap(),
        PointClassification::Inside
    );
    assert_eq!(
        cache
            .classify_point(&topo, restored[0], point, &options)
            .unwrap(),
        PointClassification::Inside
    );
    assert!(cache.stats().rebuilds >= 3);
}
