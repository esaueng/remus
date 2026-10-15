//! Qualification of exact scalar memo reuse, independently of its enablement
//! in a consumer. The oracle reruns the trimmed measurement tessellation and
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

fn measurement_oracle(topo: &Topology, face: FaceId, deflection: f64) -> f64 {
    let mesh = crate::tessellate::tessellate_face_for_measurement(topo, face, deflection).unwrap();
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
    let expected = measurement_oracle(topo, face, deflection);
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

/// A flat 2 × 2 bilinear patch, so polygon trims lie on the carrier and
/// their areas are exact.
fn flat_patch() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.0)],
            vec![Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)],
        ],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap()
}

fn square(topo: &mut Topology, lo: f64, hi: f64) -> remus_topology::wire::WireId {
    make_polygon_wire(
        topo,
        &[
            Point3::new(lo, lo, 0.0),
            Point3::new(hi, lo, 0.0),
            Point3::new(hi, hi, 0.0),
            Point3::new(lo, hi, 0.0),
        ],
        1e-7,
    )
    .unwrap()
}

/// OPS-4: the reading depends on the trim, so a warm entry for one trim of
/// a carrier must never answer another trim of the same carrier. A
/// surface-only key returned the untrimmed 4.0 for both faces below.
#[test]
fn a_warm_support_misses_after_its_trim_changes() {
    let _reset = fresh_memo();
    let mut topo = Topology::new();
    let full = make_nurbs_face(&mut topo, flat_patch(), 1e-7).unwrap();
    let full_area = assert_area_bits(&topo, full, DEFLECTION);
    assert!((full_area - 4.0).abs() < 1e-12, "{full_area}");

    let outer = square(&mut topo, 0.5, 1.5);
    let trimmed = topo.add_face(remus_topology::face::Face::new(
        outer,
        Vec::new(),
        FaceSurface::Nurbs(flat_patch()),
    ));
    let trimmed_area = assert_area_bits(&topo, trimmed, DEFLECTION);
    assert!((trimmed_area - 1.0).abs() < 1e-12, "{trimmed_area}");
    let stats = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((stats.hits, stats.misses, stats.len), (0, 2, 2));

    // A hole on the warm face: a new inner wire is new content.
    let full_outer = topo.face(full).unwrap().outer_wire();
    let hole = square(&mut topo, 0.25, 0.75);
    topo.set_face_boundary_wires(full, full_outer, vec![hole])
        .unwrap();
    let holed_area = assert_area_bits(&topo, full, DEFLECTION);
    assert!((holed_area - 3.75).abs() < 1e-12, "{holed_area}");
    let stats = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((stats.hits, stats.misses, stats.len), (0, 3, 3));

    // Removing it restores the original content and its original bits.
    topo.set_face_boundary_wires(full, full_outer, Vec::new())
        .unwrap();
    assert_eq!(
        assert_area_bits(&topo, full, DEFLECTION).to_bits(),
        full_area.to_bits()
    );
    assert_eq!(thread_nurbs_area_memo_stats().unwrap().hits, 1);
}

/// Moving one boundary vertex of an unchanged carrier changes the key.
#[test]
fn boundary_vertex_change_misses() {
    let _reset = fresh_memo();
    let mut topo = Topology::new();
    let outer = square(&mut topo, 0.5, 1.5);
    let face = topo.add_face(remus_topology::face::Face::new(
        outer,
        Vec::new(),
        FaceSurface::Nurbs(flat_patch()),
    ));
    let baseline = assert_area_bits(&topo, face, DEFLECTION);

    let mut moved = topo.clone();
    let edge = moved.wire(outer).unwrap().edges()[0].edge();
    let vertex = moved.edge(edge).unwrap().start();
    moved
        .vertex_mut(vertex)
        .unwrap()
        .set_point(Point3::new(0.25, 0.5, 0.0));
    let moved_area = assert_area_bits(&moved, face, DEFLECTION);
    assert!(
        (moved_area - baseline - 0.125).abs() < 1e-12,
        "{moved_area}"
    );
    let stats = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((stats.hits, stats.misses), (0, 2));
}

/// The same square with its four edges allocated in a different arena order.
fn square_allocated_in_order(
    topo: &mut Topology,
    order: [usize; 4],
) -> remus_topology::wire::WireId {
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    let corners = [(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)];
    let vertices: Vec<_> = corners
        .iter()
        .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)))
        .collect();
    let mut edges = [None; 4];
    for side in order {
        let edge = Edge::new(vertices[side], vertices[(side + 1) % 4], EdgeCurve::Line);
        edges[side] = Some(topo.add_edge(edge));
    }
    let oriented = edges
        .iter()
        .map(|edge| OrientedEdge::new(edge.unwrap(), true))
        .collect();
    topo.add_wire(Wire::new(oriented, true).unwrap())
}

/// The boundary pool reads edges in arena-index order, so the same face
/// content allocated in another edge order is a different key.
#[test]
fn boundary_edge_arena_order_misses() {
    let _reset = fresh_memo();
    let mut measured = Vec::new();
    for order in [[0, 1, 2, 3], [2, 0, 3, 1], [0, 1, 2, 3]] {
        let mut topo = Topology::new();
        let outer = square_allocated_in_order(&mut topo, order);
        let face = topo.add_face(remus_topology::face::Face::new(
            outer,
            Vec::new(),
            FaceSurface::Nurbs(flat_patch()),
        ));
        measured.push(assert_area_bits(&topo, face, DEFLECTION));
    }
    assert!(
        measured.iter().all(|area| (area - 1.0).abs() < 1e-12),
        "{measured:?}"
    );
    let stats = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((stats.hits, stats.misses), (1, 2));
}

/// An edge's own tolerance feeds `Edge::strict_domain` in the face mesher, so
/// setting or changing it changes the key.
#[test]
fn boundary_edge_tolerance_change_misses() {
    let _reset = fresh_memo();
    let mut topo = Topology::new();
    let outer = square(&mut topo, 0.5, 1.5);
    let face = topo.add_face(remus_topology::face::Face::new(
        outer,
        Vec::new(),
        FaceSurface::Nurbs(flat_patch()),
    ));
    assert_area_bits(&topo, face, DEFLECTION);

    let edge = topo.wire(outer).unwrap().edges()[0].edge();
    for (index, tolerance) in [Some(1e-3), Some(0.0), None].into_iter().enumerate() {
        topo.edge_mut(edge)
            .unwrap()
            .set_tolerance(tolerance)
            .unwrap();
        assert_area_bits(&topo, face, DEFLECTION);
        let stats = thread_nurbs_area_memo_stats().unwrap();
        let expected = if tolerance.is_none() {
            (1, 3)
        } else {
            (0, index as u64 + 2)
        };
        assert_eq!((stats.hits, stats.misses), expected, "{tolerance:?}");
    }
}

#[test]
fn errors_are_not_retained_and_a_later_success_is_measured() {
    let _reset = fresh_memo();
    let surface = patch();
    let (topo, face) = face();
    for _ in 0..2 {
        let error = memoized(&topo, face, &surface, DEFLECTION, || {
            Err(crate::OperationsError::InvalidInput {
                reason: "transient test error".to_owned(),
            })
        })
        .unwrap_err();
        assert!(error.to_string().contains("transient test error"));
    }
    let failed = thread_nurbs_area_memo_stats().unwrap();
    assert_eq!((failed.hits, failed.misses, failed.len), (0, 2, 0));
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
    let bytes = content_key(&source, face, &patch(), DEFLECTION, usize::MAX)
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
fn hash_collision_still_compares_the_complete_key() {
    let _reset = fresh_memo();
    let (source, face) = face();
    let baseline = assert_area_bits(&source, face, DEFLECTION);
    let changed = with_change(&patch(), "point");
    let changed_key = content_key(
        &source,
        face,
        &changed,
        DEFLECTION,
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
