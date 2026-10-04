//! B28 strict-validation probes under the opt-in face-integral cache.
//!
//! The orientation probe may be answered from
//! `remus_check::properties::face_cache` once an application enables it (the
//! WASM kernel does). The cache keys faces by content, so the probes must
//! agree — verdict, shell, Gauss order and face count exactly — whether the
//! cache is off, cold, warm, or serving a deserialized copy of the body in a
//! fresh topology, the way a consumer's short-lived probe kernel validates
//! it; the face areas exactly too.
//!
//! The signed volume is the one reading allowed to move, and only in its
//! last bits: a face whose content is a rigid translation of one integrated
//! before (a patterned hole, a repeated blend) reads that face's volume term
//! re-expressed by its vector area instead of integrating again, which
//! regroups the same sum. So the cached probes are compared with the
//! uncached one at relative 1e-12 — the tolerance the recorded values were
//! already pinned at, far below the 1e-9 × diag³ floor the verdict reads —
//! and with each other bit for bit (warm and copy replay the cold pass).
//! The signed volumes are also pinned against the values recorded before
//! the cache and the first-order NURBS solve landed (relative 1e-12: platform
//! `libm` may move the last bits of the analytic faces).

#![allow(clippy::expect_used, clippy::unwrap_used)]

use remus_check::properties::face_cache::{
    enable_thread_face_cache, set_thread_face_cache_limits, thread_face_cache_stats,
};
use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_io::step::reader::read_step;
use remus_operations::measure::face_area;
use remus_operations::validate::{ValidationOptions, validate_solid_with_budget_probes};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Fixture, then (faces, signed volume) of its one measured outer shell.
const FIXTURES: [(&str, &str, usize, f64); 4] = [
    (
        "hammer holder",
        include_str!("data/shapr3d_hammer_holder.step"),
        160,
        50_245.521_289_691_05,
    ),
    (
        "bored plate",
        include_str!("data/openzcad_a_export_bored_plate.step"),
        7,
        8_814.601_836_602_664,
    ),
    (
        "analytic fillet plate",
        include_str!("data/openzcad_e_analytic_fillet_plate.step"),
        10,
        9_522.743_338_823_033,
    ),
    (
        "NURBS fillet plate",
        include_str!("data/openzcad_e_nurbs_fillet_plate.step"),
        10,
        9_499.999_999_999_985,
    ),
];

struct CacheOff;

impl Drop for CacheOff {
    fn drop(&mut self) {
        set_thread_face_cache_limits(0, 0);
    }
}

type Reading = (String, Vec<(usize, usize, usize, u64)>, Vec<u64>);

fn read(topo: &Topology, solid: SolidId) -> Reading {
    let (report, probes) =
        validate_solid_with_budget_probes(topo, solid, &ValidationOptions::default())
            .expect("strict validation runs");
    let areas = solid_faces(topo, solid)
        .expect("faces")
        .into_iter()
        .map(|face| face_area(topo, face, 0.08).expect("face area").to_bits())
        .collect();
    (
        format!("{report:?}"),
        probes
            .shells
            .iter()
            .map(|p| (p.shell, p.order, p.faces, p.signed_volume.to_bits()))
            .collect(),
        areas,
    )
}

/// Report, probe shells/orders/face counts and face areas exactly; the
/// signed volume to relative 1e-12 (see the module docs).
fn assert_agrees(cached: &Reading, uncached: &Reading, label: &str) {
    assert_eq!(cached.0, uncached.0, "{label}: report");
    assert_eq!(cached.2, uncached.2, "{label}: face areas");
    assert_eq!(cached.1.len(), uncached.1.len(), "{label}: probes");
    for (c, u) in cached.1.iter().zip(&uncached.1) {
        assert_eq!((c.0, c.1, c.2), (u.0, u.1, u.2), "{label}: probe");
        let (c, u) = (f64::from_bits(c.3), f64::from_bits(u.3));
        assert!(
            (c - u).abs() <= u.abs() * 1e-12,
            "{label}: signed volume {c} vs uncached {u}"
        );
    }
}

#[test]
fn b28_probes_are_identical_with_the_face_cache() {
    set_thread_face_cache_limits(0, 0);
    let _off = CacheOff;
    let order = remus_check::properties::PropertiesOptions::default().gauss_order;
    for (label, step, faces, signed) in FIXTURES {
        let mut topo = Topology::new();
        let solids = read_step(step, &mut topo).expect("import");
        assert_eq!(solids.len(), 1, "{label}");
        let solid = solids[0];

        set_thread_face_cache_limits(0, 0);
        let uncached = read(&topo, solid);
        assert!(!uncached.0.contains("Error"), "{label}: strictly valid");
        assert_eq!(uncached.1.len(), 1, "{label}: one measured shell");
        let (shell, probe_order, probe_faces, bits) = uncached.1[0];
        assert_eq!(
            (shell, probe_order, probe_faces),
            (0, order, faces),
            "{label}"
        );
        let measured = f64::from_bits(bits);
        assert!(
            (measured - signed).abs() <= signed.abs() * 1e-12,
            "{label}: signed volume {measured} moved from the recorded {signed}"
        );

        enable_thread_face_cache();
        let cold = read(&topo, solid);
        let before_warm = thread_face_cache_stats();
        let warm = read(&topo, solid);
        let after_warm = thread_face_cache_stats();
        assert_agrees(&cold, &uncached, &format!("{label}: cold cache"));
        assert_eq!(warm, cold, "{label}: warm cache replays the cold pass");
        assert_eq!(
            after_warm.misses, before_warm.misses,
            "{label}: warm must hit"
        );

        // A deserialized copy in a fresh topology is the same content.
        let bytes = serialize_solids(&topo, &[solid]).expect("serialize");
        let mut copy = Topology::new();
        let copied = deserialize_solids(&bytes, &mut copy).expect("deserialize")[0];
        let before_copy = thread_face_cache_stats();
        assert_eq!(
            read(&copy, copied),
            cold,
            "{label}: deserialized copy replays the cold pass"
        );
        assert_eq!(
            thread_face_cache_stats().misses,
            before_copy.misses,
            "{label}: the copy must hit"
        );
    }
}
