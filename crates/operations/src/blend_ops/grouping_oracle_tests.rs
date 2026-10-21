//! Differential oracle for [`super::independent_blend_groups`]: the pairwise
//! grouping before its connected-pair skip, box prefilter and early-exit
//! distance scan, kept verbatim, against the current one on boxes at and
//! around their split radius, a holed L-bracket's whole-body selection, a
//! drilled plate's perimeter plus bore rim, and deterministic edge subsets.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::solid::SolidId;

use super::{
    OperationsError, edge_polyline, independent_blend_groups, planar_ridge_setback,
    point_segment_distance, union_find_root,
};
use crate::boolean::{BooleanOp, boolean};
use crate::primitives::{make_box, make_cylinder};
use crate::transform::transform_solid;

thread_local! {
    /// Seed pairs whose polylines were scanned point by point (past the
    /// connected-pair skip and the box prefilter).
    pub(super) static SCANNED_PAIRS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Shortest distance between two sampled polylines (verbatim).
fn polyline_distance(a: &[Point3], b: &[Point3]) -> f64 {
    let mut best = f64::INFINITY;
    for &p in a {
        for w in b.windows(2) {
            best = best.min(point_segment_distance(p, w[0], w[1]));
        }
    }
    for &p in b {
        for w in a.windows(2) {
            best = best.min(point_segment_distance(p, w[0], w[1]));
        }
    }
    best
}

/// `independent_blend_groups` before the shortcuts (verbatim).
fn legacy_groups(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
    size: f64,
) -> Result<Vec<Vec<EdgeId>>, OperationsError> {
    if edges.len() < 2 {
        return Ok(vec![edges.to_vec()]);
    }
    let tol = remus_math::tolerance::Tolerance::new();
    let index = remus_blend::g1_chain::G1ChainIndex::build(topo, solid)?;
    let adjacency = topo.build_adjacency(solid)?;
    let mut chains: Vec<Vec<EdgeId>> = Vec::with_capacity(edges.len());
    let mut outlines: Vec<Vec<Point3>> = Vec::with_capacity(edges.len());
    let mut bands: Vec<f64> = Vec::with_capacity(edges.len());
    for &edge in edges {
        let chain = remus_blend::g1_chain::expand_g1_chain_with(&index, &[edge], tol)?;
        let chain = if chain.is_empty() { vec![edge] } else { chain };
        let mut outline = Vec::new();
        let mut setback = 1.0f64;
        for &member in &chain {
            outline.extend(edge_polyline(topo, member, size * 0.25)?);
            if let Some(member_setback) = planar_ridge_setback(topo, &adjacency, member)? {
                setback = setback.max(member_setback);
            }
        }
        chains.push(chain);
        outlines.push(outline);
        bands.push(size * setback);
    }
    let mut parent: Vec<usize> = (0..edges.len()).collect();
    let slack = size * 0.25;
    for i in 0..edges.len() {
        for j in (i + 1)..edges.len() {
            let shares_ridgeline = chains[i].iter().any(|e| chains[j].contains(e));
            let reach = bands[i] + bands[j] + slack;
            let touches = shares_ridgeline
                || !reach.is_finite()
                || polyline_distance(&outlines[i], &outlines[j]) <= reach;
            if touches {
                let (ri, rj) = (
                    union_find_root(&mut parent, i),
                    union_find_root(&mut parent, j),
                );
                if ri != rj {
                    parent[ri] = rj;
                }
            }
        }
    }
    let mut order: Vec<usize> = Vec::new();
    let mut groups: Vec<Vec<EdgeId>> = Vec::new();
    for i in 0..edges.len() {
        let root = union_find_root(&mut parent, i);
        let slot = if let Some(pos) = order.iter().position(|&r| r == root) {
            pos
        } else {
            order.push(root);
            groups.push(Vec::new());
            groups.len() - 1
        };
        groups[slot].push(edges[i]);
    }
    Ok(groups)
}

fn assert_same(topo: &Topology, solid: SolidId, edges: &[EdgeId], size: f64) -> usize {
    let expected = legacy_groups(topo, solid, edges, size).unwrap();
    let actual = independent_blend_groups(topo, solid, edges, size).unwrap();
    assert_eq!(actual, expected, "size {size}, edges {edges:?}");
    actual.len()
}

/// Deterministic subsets of `edges`: every edge with probability ~1/2 under
/// a fixed LCG, so the oracle sees disconnected selections too.
fn subsets(edges: &[EdgeId], count: usize) -> Vec<Vec<EdgeId>> {
    let mut state = 0x5eed_u64;
    (0..count)
        .map(|_| {
            edges
                .iter()
                .copied()
                .filter(|_| {
                    state = state
                        .wrapping_mul(6_364_136_223_846_793_005)
                        .wrapping_add(1_442_695_040_888_963_407);
                    state >> 63 == 1
                })
                .collect()
        })
        .collect()
}

fn holed_bracket(topo: &mut Topology) -> SolidId {
    let base = make_box(topo, 40.0, 30.0, 4.0).unwrap();
    let upright = make_box(topo, 40.0, 4.0, 26.0).unwrap();
    transform_solid(topo, upright, &Mat4::translation(0.0, 0.0, 4.0)).unwrap();
    let mut body = boolean(topo, BooleanOp::Fuse, base, upright).unwrap();
    for (x, y, z, along_y) in [
        (10.0, 18.0, -1.0, false),
        (30.0, 18.0, -1.0, false),
        (20.0, -1.0, 18.0, true),
    ] {
        let hole = make_cylinder(topo, 2.5, 6.0).unwrap();
        if along_y {
            transform_solid(topo, hole, &Mat4::rotation_x(-std::f64::consts::FRAC_PI_2)).unwrap();
        }
        transform_solid(topo, hole, &Mat4::translation(x, y, z)).unwrap();
        body = boolean(topo, BooleanOp::Cut, body, hole).unwrap();
    }
    body
}

/// Box verticals 10 apart join at `2.25·size >= 10`: the split radius
/// `10 / 2.25` itself is a tie the box prefilter must not decide.
#[test]
fn box_grouping_matches_the_legacy_partition_around_the_split() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(&topo, cube).unwrap();
    let tie = 10.0_f64 / 2.25;
    for size in [
        0.5,
        2.0,
        4.4,
        tie.next_down(),
        tie,
        tie.next_up(),
        4.5,
        20.0,
    ] {
        assert_same(&topo, cube, &edges, size);
        for subset in subsets(&edges, 6) {
            assert_same(&topo, cube, &subset, size);
        }
    }
}

/// The holed L-bracket whole-body selection and its subsets, at the radii
/// the refusal bench times.
#[test]
fn bracket_grouping_matches_the_legacy_partition() {
    let mut topo = Topology::new();
    let body = holed_bracket(&mut topo);
    let edges = remus_topology::explorer::solid_edges(&topo, body).unwrap();
    for size in [0.5, 1.0, 2.0] {
        assert_same(&topo, body, &edges, size);
        for subset in subsets(&edges, 4) {
            assert_same(&topo, body, &subset, size);
        }
    }
}

/// A plate's top perimeter and a bore rim split into two groups below the
/// rim clearance and join above it.
#[test]
fn drilled_plate_grouping_matches_the_legacy_partition() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 40.0, 30.0, 4.0).unwrap();
    let bore = make_cylinder(&mut topo, 3.0, 6.0).unwrap();
    transform_solid(&mut topo, bore, &Mat4::translation(20.0, 15.0, -1.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();
    let top: Vec<EdgeId> = remus_topology::explorer::solid_edges(&topo, body)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            (a.z() - 4.0).abs() < 1e-9 && (b.z() - 4.0).abs() < 1e-9
        })
        .collect();
    let mut split = false;
    for size in [0.5, 1.0, 3.0, 5.0, 6.0, 8.0] {
        split |= assert_same(&topo, body, &top, size) > 1;
    }
    assert!(split, "the sweep must cover a split partition");
}

/// Complexity guard: a whole-body selection scans only the pairs that
/// decide its partition, not all `n(n-1)/2`. Deterministic counts, no
/// timing.
#[test]
fn whole_body_grouping_scans_few_pairs() {
    let mut topo = Topology::new();
    let body = holed_bracket(&mut topo);
    let edges = remus_topology::explorer::solid_edges(&topo, body).unwrap();
    let pairs = edges.len() * (edges.len() - 1) / 2;
    for size in [0.5, 1.0, 2.0] {
        SCANNED_PAIRS.with(|n| n.set(0));
        independent_blend_groups(&topo, body, &edges, size).unwrap();
        let scanned = SCANNED_PAIRS.with(std::cell::Cell::get);
        assert!(
            4 * scanned < pairs,
            "size {size}: scanned {scanned} of {pairs} pairs"
        );
    }
}
