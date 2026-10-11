//! Qualification of exact scalar memo reuse, independently of its enablement
//! in a consumer. The oracle reruns the pre-memo standalone tessellation and
//! triangle accumulation, including its iteration order, for area-bit identity.

#![allow(clippy::unwrap_used, clippy::panic, clippy::cast_precision_loss)]

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::builder::{make_nurbs_face, make_polygon_wire};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::transaction::RollbackSnapshot;

use super::*;
use crate::measure::face_area;

const DEFLECTION: f64 = 0.08;

struct ResetOnDrop;

impl Drop for ResetOnDrop {
    fn drop(&mut self) {
        disable_thread_nurbs_area_memo();
    }
}

fn fresh_memo() -> ResetOnDrop {
    disable_thread_nurbs_area_memo();
    enable_thread_nurbs_area_memo();
    ResetOnDrop
}

fn patch() -> NurbsSurface {
    let points: Vec<Vec<Point3>> = (0..4)
        .map(|i| {
            (0..3)
                .map(|j| {
                    let z = if (i == 1 || i == 2) && j == 1 {
                        0.75
                    } else {
                        0.0
                    };
                    Point3::new(i as f64, j as f64, z)
                })
                .collect()
        })
        .collect();
    let mut weights = vec![vec![1.0; 3]; 4];
    weights[1][1] = 1.125;
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points,
        weights,
    )
    .unwrap()
}

fn with_change(surface: &NurbsSurface, change: &str) -> NurbsSurface {
    let mut points = surface.control_points().to_vec();
    let mut knots_u = surface.knots_u().to_vec();
    let mut weights = surface.weights().to_vec();
    match change {
        "point" => {
            let p = points[1][1];
            points[1][1] = Point3::new(p.x(), p.y(), p.z() + 0.3);
        }
        "knot" => knots_u[3] = 0.6,
        "weight" => weights[1][1] = 1.375,
        "signed-zero" => {
            let p = points[0][0];
            points[0][0] = Point3::new(-0.0, p.y(), p.z());
        }
        _ => panic!("unknown change"),
    }
    NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        knots_u,
        surface.knots_v().to_vec(),
        points,
        weights,
    )
    .unwrap()
}

fn face() -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let face = make_nurbs_face(&mut topo, patch(), 1e-7).unwrap();
    (topo, face)
}

fn standalone_oracle(topo: &Topology, face: FaceId, deflection: f64) -> f64 {
    let mesh = crate::tessellate::tessellate(topo, face, deflection).unwrap();
    let mut area = 0.0;
    for triangle in mesh.indices.chunks_exact(3) {
        let p0 = mesh.positions[triangle[0] as usize];
        let p1 = mesh.positions[triangle[1] as usize];
        let p2 = mesh.positions[triangle[2] as usize];
        area += 0.5 * (p1 - p0).cross(p2 - p0).length();
    }
    area
}

fn assert_area_bits(topo: &Topology, face: FaceId, deflection: f64) -> f64 {
    let expected = standalone_oracle(topo, face, deflection);
    let measured = face_area(topo, face, deflection).unwrap();
    assert_eq!(measured.to_bits(), expected.to_bits());
    measured
}

#[test]
fn source_clone_and_surface_edit_replay_area_bits() {
    let _reset = fresh_memo();
    let (source, face) = face();
    let baseline = assert_area_bits(&source, face, DEFLECTION);
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().misses, 1);

    let mut edited = source.clone();
    assert_ne!(source.cache_identity(), edited.cache_identity());
    assert_eq!(
        assert_area_bits(&edited, face, DEFLECTION).to_bits(),
        baseline.to_bits()
    );
    let after_clone = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((after_clone.hits, after_clone.misses), (1, 1));

    let before_edit_generation = edited.cache_generation();
    let rollback = RollbackSnapshot::capture(&mut edited);
    edited
        .face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(with_change(&patch(), "point")));
    let changed = assert_area_bits(&edited, face, DEFLECTION);
    assert_ne!(
        changed.to_bits(),
        baseline.to_bits(),
        "nontrivial geometry edit is exercised"
    );
    let after_edit = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!(
        (after_edit.hits, after_edit.misses, after_edit.len),
        (1, 2, 2)
    );

    let same_edited = edited.clone();
    assert_eq!(
        assert_area_bits(&same_edited, face, DEFLECTION).to_bits(),
        changed.to_bits()
    );
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().hits, 2);

    // A restore of the old geometry must recover its old exact-content entry,
    // even while the topology generation continues moving forward.
    rollback.restore(&mut edited);
    assert!(edited.cache_generation() > before_edit_generation);
    assert_eq!(
        assert_area_bits(&edited, face, DEFLECTION).to_bits(),
        baseline.to_bits()
    );
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().hits, 3);
}

#[test]
fn geometry_and_request_changes_miss_and_match_fresh_area_bits() {
    let _reset = fresh_memo();
    let (source, face) = face();
    assert_area_bits(&source, face, DEFLECTION);
    for (index, change) in ["point", "knot", "weight", "signed-zero"]
        .into_iter()
        .enumerate()
    {
        let mut changed = source.clone();
        changed
            .face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(with_change(&patch(), change)));
        assert_area_bits(&changed, face, DEFLECTION);
        let stats = thread_nurbs_area_memo_stats().unwrap();
        assert_eq!(stats.misses, index as u64 + 2, "{change} must miss");
        assert_eq!(stats.hits, 0);
    }

    assert_area_bits(&source, face, DEFLECTION / 4.0);
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().misses, 6);
    let mut reversed = source.clone();
    reversed.face_mut(face).unwrap().set_reversed(true);
    assert_area_bits(&reversed, face, DEFLECTION);
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().misses, 7);

    // Repeating either policy returns its independently checked original bits.
    assert_area_bits(&source, face, DEFLECTION / 4.0);
    assert_area_bits(&reversed, face, DEFLECTION);
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().hits, 2);
}

#[test]
fn a_warm_support_cannot_bypass_the_holed_face_refusal() {
    let _reset = fresh_memo();
    let (mut topo, face) = face();
    let before = assert_area_bits(&topo, face, DEFLECTION);
    let stats = thread_nurbs_area_memo_stats().unwrap();
    let hole = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(0.75, 0.5, 0.0),
            Point3::new(0.75, 0.75, 0.0),
            Point3::new(0.5, 0.75, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let outer = topo.face(face).unwrap().outer_wire();
    topo.set_face_boundary_wires(face, outer, vec![hole])
        .unwrap();
    let expected = crate::tessellate::tessellate(&topo, face, DEFLECTION)
        .unwrap_err()
        .to_string();
    assert!(expected.contains("holed NURBS face"));
    for _ in 0..2 {
        let error = face_area(&topo, face, DEFLECTION).unwrap_err();
        assert_eq!(error.to_string(), expected);
    }
    let after = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!(
        (after.hits, after.misses, after.len),
        (stats.hits, stats.misses, stats.len)
    );
    topo.set_face_boundary_wires(face, outer, Vec::new())
        .unwrap();
    assert_eq!(
        assert_area_bits(&topo, face, DEFLECTION).to_bits(),
        before.to_bits()
    );
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().hits, stats.hits + 1);
}

#[test]
fn errors_are_not_retained_and_a_later_success_is_measured() {
    let _reset = fresh_memo();
    let surface = patch();
    for _ in 0..2 {
        let error = memoized(&surface, DEFLECTION, false, || {
            Err(crate::OperationsError::InvalidInput {
                reason: "transient test error".to_owned(),
            })
        })
        .unwrap_err();
        assert!(error.to_string().contains("transient test error"));
    }
    let failed = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((failed.hits, failed.misses, failed.len), (0, 2, 0));
    let (topo, face) = face();
    assert_area_bits(&topo, face, DEFLECTION);
    let succeeded = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((succeeded.hits, succeeded.misses, succeeded.len), (0, 3, 1));
    assert_area_bits(&topo, face, DEFLECTION);
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().hits, 1);
}

#[test]
fn fifo_eviction_honors_entry_and_byte_bounds_and_oversize_bypasses() {
    let _reset = fresh_memo();
    set_thread_nurbs_area_memo_limits(2, DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET);
    let (source, face) = face();
    let baseline = assert_area_bits(&source, face, DEFLECTION);
    for change in ["point", "knot"] {
        let mut edited = source.clone();
        edited
            .face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(with_change(&patch(), change)));
        assert_area_bits(&edited, face, DEFLECTION);
    }
    let full = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((full.len, full.evictions, full.misses), (2, 1, 3));
    assert!(full.retained_bytes <= DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET);
    assert_eq!(
        assert_area_bits(&source, face, DEFLECTION).to_bits(),
        baseline.to_bits()
    );
    assert_eq!(
        thread_nurbs_area_memo_stats().unwrap().misses,
        4,
        "evicted source must be recalculated"
    );

    disable_thread_nurbs_area_memo();
    enable_thread_nurbs_area_memo();
    let bytes = content_key(&patch(), DEFLECTION, false, usize::MAX)
        .unwrap()
        .len()
        * std::mem::size_of::<u64>()
        + std::mem::size_of::<Entry>();
    let byte_limit = bytes * 2 - 1;
    set_thread_nurbs_area_memo_limits(DEFAULT_NURBS_AREA_MEMO_CAPACITY, byte_limit);
    assert_area_bits(&source, face, DEFLECTION);
    let mut edited = source.clone();
    edited
        .face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(with_change(&patch(), "point")));
    assert_area_bits(&edited, face, DEFLECTION);
    let bounded = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((bounded.len, bounded.evictions), (1, 1));
    assert!(bounded.retained_bytes <= byte_limit);

    disable_thread_nurbs_area_memo();
    enable_thread_nurbs_area_memo();
    set_thread_nurbs_area_memo_limits(DEFAULT_NURBS_AREA_MEMO_CAPACITY, bytes - 1);
    assert_area_bits(&source, face, DEFLECTION);
    let oversize = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!(
        (oversize.len, oversize.retained_bytes, oversize.misses),
        (0, 0, 1)
    );
}

#[test]
fn hash_collision_still_compares_the_complete_surface_key() {
    let _reset = fresh_memo();
    let (source, face) = face();
    let baseline = assert_area_bits(&source, face, DEFLECTION);
    let changed = with_change(&patch(), "point");
    let changed_key = content_key(
        &changed,
        DEFLECTION,
        false,
        DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET,
    )
    .unwrap();
    let changed_hash = hash_key(&changed_key);
    MEMO.with(|cell| {
        // Force an unequal source key into the changed key's candidate group.
        // A hash-only lookup would return the source's incorrect area.
        cell.borrow_mut()
            .as_mut()
            .unwrap()
            .entries
            .front_mut()
            .unwrap()
            .hash = changed_hash;
    });
    let mut edited = source;
    edited
        .face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(changed));
    let area = assert_area_bits(&edited, face, DEFLECTION);
    assert_ne!(area.to_bits(), baseline.to_bits());
    let stats = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((stats.hits, stats.misses, stats.len), (0, 2, 2));
}

#[test]
fn changing_limits_and_clearing_preserve_counters_and_release_readings() {
    let _reset = fresh_memo();
    let (source, face) = face();
    let baseline = assert_area_bits(&source, face, DEFLECTION);
    let mut edited = source.clone();
    edited
        .face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(with_change(&patch(), "point")));
    assert_area_bits(&edited, face, DEFLECTION);

    set_thread_nurbs_area_memo_limits(1, DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET);
    let reduced = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!(
        (
            reduced.len,
            reduced.evictions,
            reduced.misses,
            reduced.capacity
        ),
        (1, 1, 2, 1)
    );
    clear_thread_nurbs_area_memo();
    let cleared = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!(
        (
            cleared.len,
            cleared.retained_bytes,
            cleared.misses,
            cleared.evictions
        ),
        (0, 0, 2, 1)
    );

    set_thread_nurbs_area_memo_limits(0, DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET);
    assert_eq!(
        assert_area_bits(&source, face, DEFLECTION).to_bits(),
        baseline.to_bits()
    );
    let disabled = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!(
        (disabled.len, disabled.misses, disabled.capacity),
        (0, 2, 0)
    );
    set_thread_nurbs_area_memo_limits(
        DEFAULT_NURBS_AREA_MEMO_CAPACITY,
        DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET,
    );
    assert_eq!(
        assert_area_bits(&source, face, DEFLECTION).to_bits(),
        baseline.to_bits()
    );
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().misses, 3);
    disable_thread_nurbs_area_memo();
    assert!(thread_nurbs_area_memo_stats().is_none());
}

#[test]
fn repeated_enable_preserves_custom_and_disabled_bounds() {
    let _reset = fresh_memo();
    let (source, face) = face();
    let baseline = assert_area_bits(&source, face, DEFLECTION);
    set_thread_nurbs_area_memo_limits(3, 4096);
    let configured = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((configured.capacity, configured.byte_budget), (3, 4096));
    enable_thread_nurbs_area_memo();
    assert_eq!(thread_nurbs_area_memo_stats().unwrap(), configured);
    assert_eq!(
        assert_area_bits(&source, face, DEFLECTION).to_bits(),
        baseline.to_bits()
    );
    assert_eq!(
        thread_nurbs_area_memo_stats().unwrap().hits,
        configured.hits + 1
    );

    for (capacity, byte_budget) in [(0, 4096), (3, 0), (0, 0)] {
        set_thread_nurbs_area_memo_limits(capacity, byte_budget);
        let disabled = thread_nurbs_area_memo_stats().unwrap();
        assert_eq!(
            (disabled.capacity, disabled.byte_budget),
            (capacity, byte_budget)
        );
        assert_eq!((disabled.len, disabled.retained_bytes), (0, 0));
        enable_thread_nurbs_area_memo();
        assert_eq!(thread_nurbs_area_memo_stats().unwrap(), disabled);
        assert_eq!(
            assert_area_bits(&source, face, DEFLECTION).to_bits(),
            baseline.to_bits()
        );
        assert_eq!(thread_nurbs_area_memo_stats().unwrap(), disabled);
    }
}

#[test]
fn unavailable_cache_borrow_falls_back_to_the_same_measured_area() {
    let _reset = fresh_memo();
    let (source, face) = face();
    let baseline = assert_area_bits(&source, face, DEFLECTION);
    let before = thread_nurbs_area_memo_stats().unwrap();
    MEMO.with(|cell| {
        let _held = cell.borrow_mut();
        let area = face_area(&source, face, DEFLECTION).unwrap();
        assert_eq!(area.to_bits(), baseline.to_bits());
    });
    assert_eq!(thread_nurbs_area_memo_stats().unwrap(), before);
    assert_eq!(
        assert_area_bits(&source, face, DEFLECTION).to_bits(),
        baseline.to_bits()
    );
    assert_eq!(
        thread_nurbs_area_memo_stats().unwrap().hits,
        before.hits + 1
    );
}
