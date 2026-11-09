//! OpenZCAD issue 1246: exact Union of three overlapping boxes depended on
//! the grouping. `(A ∪ B) ∪ C` gave the 26748 mm³ solid while `A ∪ (B ∪ C)`
//! refused in both orders.
//!
//! B ∪ C carries an L-shaped wall at x = 24 (C's side less B). A's top plane
//! cuts it along one section that ends exactly at the L's reflex corner. The
//! face splitter's greedy walker took the boundary's right turn there instead
//! of the section, ran the section out and back as a zero-area spur, kept the
//! wall unsplit, and the part of the wall above A went missing: six free
//! edges, and an exact-only refusal.
//!
//! Every grouping and operand order must give the same exact solid: volume,
//! a closed oriented manifold mesh, one lump, the material of each box and
//! none outside them, with the operands left as they were.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::tessellate_solid;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::solid::SolidId;

type Bounds = ([f64; 3], [f64; 3]);
const A: Bounds = ([0.0, 0.0, 0.0], [30.0, 18.0, 24.0]);
const B: Bounds = ([15.0, 6.0, 0.0], [45.0, 24.0, 24.0]);
const C: Bounds = ([-6.0, 4.0, 3.0], [24.0, 22.0, 27.0]);
/// |A| + |B| + |C| − |A∩B| − |A∩C| − |B∩C| + |A∩B∩C|
/// = 3·12960 − 4320 − 7056 − 3024 + 2268.
const UNION_VOLUME: f64 = 26_748.0;

fn place(topo: &mut Topology, (lo, hi): Bounds) -> SolidId {
    let solid = make_box(topo, hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]).unwrap();
    transform_solid(topo, solid, &Mat4::translation(lo[0], lo[1], lo[2])).unwrap();
    solid
}

fn inside((lo, hi): Bounds, p: [f64; 3]) -> bool {
    (0..3).all(|k| lo[k] < p[k] && p[k] < hi[k])
}

fn clear_of_faces(boxes: &[Bounds], p: [f64; 3]) -> bool {
    boxes
        .iter()
        .all(|(lo, hi)| (0..3).all(|k| (p[k] - lo[k]).abs() > 0.2 && (p[k] - hi[k]).abs() > 0.2))
}

/// Closed, consistently oriented, outward triangle mesh with the exact
/// volume; a closed 2-manifold B-rep with one lump that validates.
fn assert_sound_union(topo: &Topology, solid: SolidId, what: &str) {
    let report =
        validate_solid(topo, solid).unwrap_or_else(|e| panic!("{what}: invalid result: {e:?}"));
    assert!(report.is_valid(), "{what}: {:?}", report.issues);
    let adj = AdjacencyIndex::build(topo, solid).unwrap();
    assert!(adj.boundary_edges().is_empty(), "{what}: free edges");
    assert!(
        adj.non_manifold_edges().is_empty(),
        "{what}: non-manifold edges"
    );
    let volume = solid_volume(topo, solid, 0.01).unwrap();
    assert!(
        (volume - UNION_VOLUME).abs() < 1e-6,
        "{what}: volume {volume}, expected {UNION_VOLUME}"
    );

    let mesh = tessellate_solid(topo, solid, 0.01).unwrap();
    let mut ids = HashMap::<(i64, i64, i64), u32>::new();
    #[allow(clippy::cast_possible_truncation)]
    let remap: Vec<u32> = mesh
        .positions
        .iter()
        .map(|p| {
            let key = (
                (p.x() * 1e6).round() as i64,
                (p.y() * 1e6).round() as i64,
                (p.z() * 1e6).round() as i64,
            );
            let next = ids.len() as u32;
            *ids.entry(key).or_insert(next)
        })
        .collect();
    let mut directed = HashMap::<(u32, u32), usize>::new();
    let mut six_volume = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let [p, q, r] = [0, 1, 2].map(|i| mesh.positions[tri[i] as usize]);
        six_volume += (p - Point3::new(0.0, 0.0, 0.0)).dot((q - p).cross(r - p));
        let v = [0, 1, 2].map(|i| remap[tri[i] as usize]);
        for (a, b) in [(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            if a != b {
                *directed.entry((a, b)).or_default() += 1;
            }
        }
    }
    for (&(a, b), &n) in &directed {
        assert_eq!(n, 1, "{what}: directed mesh edge {a}->{b} used {n} times");
        assert_eq!(
            directed.get(&(b, a)),
            Some(&1),
            "{what}: mesh edge {a}->{b} has no opposite half (open or misoriented)"
        );
    }
    assert!(
        (six_volume / 6.0 - UNION_VOLUME).abs() < 1e-3,
        "{what}: mesh volume {}",
        six_volume / 6.0
    );

    // Material: a grid of points clear of every box face is inside the union
    // exactly when it is inside one of the boxes.
    let boxes = [A, B, C];
    let mut checked = 0;
    for i in 0..=26 {
        for j in 0..=13 {
            for k in 0..=14 {
                #[allow(clippy::cast_lossless)]
                let p = [
                    -8.0 + 2.0 * f64::from(i) + 0.31,
                    -2.0 + 2.0 * f64::from(j) + 0.37,
                    -2.0 + 2.0 * f64::from(k) + 0.43,
                ];
                if !clear_of_faces(&boxes, p) {
                    continue;
                }
                let expected = if boxes.iter().any(|&b| inside(b, p)) {
                    PointClassification::Inside
                } else {
                    PointClassification::Outside
                };
                let got =
                    classify_point(topo, solid, Point3::new(p[0], p[1], p[2]), 0.01, 1e-7).unwrap();
                assert_eq!(got, expected, "{what}: material at {p:?}");
                checked += 1;
            }
        }
    }
    assert!(checked > 1000, "{what}: only {checked} material probes");
}

fn assert_operand_unchanged(topo: &Topology, solid: SolidId, volume: f64, what: &str) {
    let v = solid_volume(topo, solid, 0.01).unwrap();
    assert!(
        (v - volume).abs() < 1e-6,
        "{what}: operand volume changed to {v}"
    );
    let report =
        validate_solid(topo, solid).unwrap_or_else(|e| panic!("{what}: operand invalid: {e:?}"));
    assert!(report.is_valid(), "{what}: operand {:?}", report.issues);
}

#[test]
fn every_grouping_and_order_gives_the_same_exact_union() {
    // (first pair, last box): the pair is fused first in both of its orders,
    // then the union is taken with the last box in both orders.
    let groupings: [(&str, [Bounds; 2], Bounds); 3] = [
        ("A ∪ (B ∪ C)", [B, C], A),
        ("(A ∪ B) ∪ C", [A, B], C),
        ("(A ∪ C) ∪ B", [A, C], B),
    ];
    for (name, pair, last) in groupings {
        for swap_pair in [false, true] {
            for swap_outer in [false, true] {
                let what = format!("{name} swap_pair={swap_pair} swap_outer={swap_outer}");
                let mut topo = Topology::new();
                let (p, q) = if swap_pair {
                    (pair[1], pair[0])
                } else {
                    (pair[0], pair[1])
                };
                let p = place(&mut topo, p);
                let q = place(&mut topo, q);
                let last = place(&mut topo, last);
                let first = boolean(&mut topo, BooleanOp::Fuse, p, q)
                    .unwrap_or_else(|e| panic!("{what}: first union refused: {e:?}"));
                let first_volume = solid_volume(&topo, first, 0.01).unwrap();
                let (x, y) = if swap_outer {
                    (last, first)
                } else {
                    (first, last)
                };
                let union = boolean(&mut topo, BooleanOp::Fuse, x, y)
                    .unwrap_or_else(|e| panic!("{what}: union refused: {e:?}"));
                assert_sound_union(&topo, union, &what);
                assert_operand_unchanged(&topo, first, first_volume, &what);
                assert_operand_unchanged(&topo, p, 12_960.0, &what);
                assert_operand_unchanged(&topo, q, 12_960.0, &what);
                assert_operand_unchanged(&topo, last, 12_960.0, &what);
            }
        }
    }
}

/// The production reproduction imported A and a precomputed B ∪ C as STEP.
#[test]
fn step_round_tripped_operands_give_the_same_exact_union() {
    let mut topo = Topology::new();
    let a = place(&mut topo, A);
    let b = place(&mut topo, B);
    let c = place(&mut topo, C);
    let bc = boolean(&mut topo, BooleanOp::Fuse, b, c).unwrap();
    let text = remus_io::step::writer::write_step(&topo, &[a, bc]).unwrap();
    let mut reloaded = Topology::new();
    let solids = remus_io::step::reader::read_step(&text, &mut reloaded).unwrap();
    assert_eq!(solids.len(), 2);
    for (x, y) in [(solids[0], solids[1]), (solids[1], solids[0])] {
        let union = boolean(&mut reloaded, BooleanOp::Fuse, x, y).unwrap();
        assert_sound_union(&reloaded, union, "STEP A ∪ (B ∪ C)");
        assert_operand_unchanged(&reloaded, solids[0], 12_960.0, "STEP A");
        assert_operand_unchanged(&reloaded, solids[1], 22_896.0, "STEP B ∪ C");
    }
}
