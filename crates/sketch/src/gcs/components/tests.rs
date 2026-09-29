//! Tests for the PERF-S02 component decomposition.
//!
//! These pin the structural contract: which constraints join components,
//! how fixed parameters and degenerate groups behave, determinism, and the
//! join/split roundtrip. Numerical agreement with the dense path lives in
//! `crates/sketch/tests/gcs_components_agreement.rs`.

use std::collections::HashMap;

use super::super::constraint::Constraint;
use super::super::entity::ParamRef::{CircleRadius as R, PointX as X, PointY as Y};
use super::super::entity::{ParamRef, PointData};
use super::super::system::GcsSystem;
use super::{constraint_param_indices, decompose};

fn free_pt(sys: &mut GcsSystem, x: f64, y: f64) -> super::super::entity::PointId {
    sys.add_point(PointData { x, y, fixed: false }).unwrap()
}

fn fixed_pt(sys: &mut GcsSystem, x: f64, y: f64) -> super::super::entity::PointId {
    sys.add_point(PointData { x, y, fixed: true }).unwrap()
}

/// One anchor/free pair with a Distance target, the S01 independent shape.
fn anchor_pair(sys: &mut GcsSystem, ax: f64) {
    let anchor = fixed_pt(sys, ax, 0.0);
    let free = free_pt(sys, ax + 1.0, 1.0);
    sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
        .unwrap();
}

fn param_index_of(sys: &mut GcsSystem) -> HashMap<ParamRef, usize> {
    sys.rebuild_if_dirty();
    sys.param_index_map().clone()
}

#[test]
fn disjoint_pairs_split_into_one_component_each() {
    let mut sys = GcsSystem::new();
    for i in 0..3 {
        anchor_pair(&mut sys, 10.0 * i as f64);
    }
    let d = decompose(&mut sys);
    assert_eq!(d.num_params, 6);
    assert_eq!(d.num_equations, 3);
    assert_eq!(d.components.len(), 3);
    // param_map order: anchor(fixed), free(2 params) × 3 → [0,1],[2,3],[4,5].
    for (i, comp) in d.components.iter().enumerate() {
        assert_eq!(comp.params, vec![2 * i, 2 * i + 1]);
        assert_eq!(comp.constraint_ids.len(), 1);
        assert!(!comp.is_free() && !comp.is_pinned());
    }
}

#[test]
fn shared_free_point_joins() {
    let mut sys = GcsSystem::new();
    let a = fixed_pt(&mut sys, 0.0, 0.0);
    let b = free_pt(&mut sys, 1.0, 0.0);
    let c = free_pt(&mut sys, 2.0, 0.0);
    sys.add_constraint(Constraint::Distance(a, b, 1.0)).unwrap();
    sys.add_constraint(Constraint::Distance(b, c, 1.0)).unwrap();
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 1);
    assert_eq!(d.components[0].params.len(), 4);
    assert_eq!(d.components[0].constraint_ids.len(), 2);
}

#[test]
fn shared_fixed_point_joins_nothing() {
    let mut sys = GcsSystem::new();
    let anchor = fixed_pt(&mut sys, 0.0, 0.0);
    let b = free_pt(&mut sys, 1.0, 0.0);
    let c = free_pt(&mut sys, 0.0, 1.0);
    sys.add_constraint(Constraint::Distance(anchor, b, 1.0))
        .unwrap();
    sys.add_constraint(Constraint::Distance(anchor, c, 1.0))
        .unwrap();
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 2);
    for comp in &d.components {
        assert_eq!(comp.params.len(), 2);
        assert_eq!(comp.constraint_ids.len(), 1);
    }
}

#[test]
fn mutable_fixed_flag_keeps_partition_aligned_with_cached_parameter_map() {
    let mut sys = GcsSystem::new();
    let formerly_free = free_pt(&mut sys, 1.0, 0.0);
    let formerly_fixed = fixed_pt(&mut sys, 2.0, 0.0);
    let other_free = free_pt(&mut sys, 3.0, 0.0);
    let first = sys
        .add_constraint(Constraint::FixX(formerly_free, 1.0))
        .unwrap();
    let pinned = sys
        .add_constraint(Constraint::FixX(formerly_fixed, 2.0))
        .unwrap();
    sys.add_constraint(Constraint::FixX(other_free, 3.0))
        .unwrap();
    let cached = param_index_of(&mut sys);
    sys.point_mut(formerly_free).unwrap().fixed = true;
    sys.point_mut(formerly_fixed).unwrap().fixed = false;

    let d = decompose(&mut sys);
    let first_component = d
        .components
        .iter()
        .find(|component| component.constraint_ids.contains(&first))
        .unwrap();
    assert!(first_component.params.contains(&cached[&X(formerly_free)]));
    let pinned_component = d
        .components
        .iter()
        .find(|component| component.constraint_ids.contains(&pinned))
        .unwrap();
    assert!(pinned_component.is_pinned());
    assert!(!cached.contains_key(&X(formerly_fixed)));
}

#[test]
fn point_mut_fixed_change_after_solve_preserves_multiblock_result() {
    let mut dense = GcsSystem::new();
    let dense_point = free_pt(&mut dense, 0.0, 0.0);
    dense
        .add_constraint(Constraint::FixX(dense_point, 1.0))
        .unwrap();
    assert!(dense.solve_detailed(50, 1e-9).unwrap().converged);
    let point = dense.point_mut(dense_point).unwrap();
    point.fixed = true;
    point.x = 0.0;
    let dense_result = dense.solve_detailed(50, 1e-9).unwrap();

    let mut split = GcsSystem::new();
    let changed = free_pt(&mut split, 0.0, 0.0);
    let other = free_pt(&mut split, 0.0, 0.0);
    split
        .add_constraint(Constraint::FixX(changed, 1.0))
        .unwrap();
    split.add_constraint(Constraint::FixX(other, 2.0)).unwrap();
    assert!(split.solve_detailed(50, 1e-9).unwrap().converged);
    let point = split.point_mut(changed).unwrap();
    point.fixed = true;
    point.x = 0.0;
    let split_result = split.solve_detailed(50, 1e-9).unwrap();

    assert_eq!(split_result.converged, dense_result.converged);
    assert_eq!(split_result.rolled_back, dense_result.rolled_back);
    assert_eq!(split_result.classification, dense_result.classification);
    assert!((split.point(changed).unwrap().x - dense.point(dense_point).unwrap().x).abs() < 1e-9);
    assert!((split.point(other).unwrap().x - 2.0).abs() < 1e-9);
}

#[test]
fn line_endpoints_couple_through_the_line() {
    let mut sys = GcsSystem::new();
    let p1 = free_pt(&mut sys, 0.0, 0.5);
    let p2 = free_pt(&mut sys, 1.0, 0.0);
    let line = sys.add_line(p1, p2).unwrap();
    sys.add_constraint(Constraint::Horizontal(line)).unwrap();
    // FixY shares the Y coordinates Horizontal reads; FixX would not —
    // coordinate precision keeps disjoint coordinates in separate components.
    sys.add_constraint(Constraint::FixY(p1, 0.5)).unwrap();
    // Unrelated pair stays separate.
    anchor_pair(&mut sys, 10.0);
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 3);
    // Param order: p1 → X=0,Y=1; p2 → X=2,Y=3. The line block owns the two Y
    // params; the untouched X params form the free group; the pair is last.
    assert_eq!(d.components[0].params, vec![1, 3]);
    assert_eq!(d.components[0].constraint_ids.len(), 2);
    let free = d.components.iter().find(|c| c.is_free()).unwrap();
    assert_eq!(free.params, vec![0, 2]);
}

#[test]
fn circle_center_and_radius_couple() {
    let mut sys = GcsSystem::new();
    let center = free_pt(&mut sys, 0.0, 0.0);
    let rim = free_pt(&mut sys, 2.0, 0.0);
    let circ = sys.add_circle(center, 2.0).unwrap();
    sys.add_constraint(Constraint::PointOnCircle(rim, circ))
        .unwrap();
    sys.add_constraint(Constraint::CircleRadius(circ, 2.0))
        .unwrap();
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 1);
    // rim x/y, center x/y, radius.
    assert_eq!(d.components[0].params.len(), 5);
}

#[test]
fn shared_center_couples_two_circles() {
    let mut sys = GcsSystem::new();
    let center = free_pt(&mut sys, 0.0, 0.0);
    let r1 = free_pt(&mut sys, 1.0, 0.0);
    let r2 = free_pt(&mut sys, 0.0, 2.0);
    let c1 = sys.add_circle(center, 1.0).unwrap();
    let c2 = sys.add_circle(center, 2.0).unwrap();
    sys.add_constraint(Constraint::PointOnCircle(r1, c1))
        .unwrap();
    sys.add_constraint(Constraint::PointOnCircle(r2, c2))
        .unwrap();
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 1);
    // rim points (2+2), shared center (2), and both circle radii (1+1).
    assert_eq!(d.components[0].params.len(), 8);
}

#[test]
fn arc_internal_tie_couples_the_arc() {
    let mut sys = GcsSystem::new();
    let center = free_pt(&mut sys, 0.0, 0.0);
    let start = free_pt(&mut sys, 1.0, 0.0);
    let end = free_pt(&mut sys, 0.0, 1.0);
    let arc = sys.add_arc(center, start, end).unwrap();
    // A user constraint touching only the center still joins the arc's
    // internal PointOnArc tie, which names center/start/end.
    sys.add_constraint(Constraint::FixX(center, 0.0)).unwrap();
    let _ = arc;
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 1);
    assert_eq!(d.components[0].params.len(), 6);
    assert_eq!(d.components[0].constraint_ids.len(), 2);
}

#[test]
fn degenerate_zero_gradient_keeps_the_structural_edge() {
    // Distance from a point to itself has an all-zero analytic Jacobian row at
    // every iterate (dx = dy = 0 identically), yet it structurally names the
    // point's coordinates. A numerically-observed-zero rule would split it
    // from FixX on the same point; the structural edge keeps them joined.
    let mut sys = GcsSystem::new();
    let p = free_pt(&mut sys, 1.0, 1.0);
    sys.add_constraint(Constraint::Distance(p, p, 1.0)).unwrap();
    sys.add_constraint(Constraint::FixX(p, 1.0)).unwrap();
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 1);
    assert!(d.is_single_connected());
}

#[test]
fn fixed_only_constraints_form_the_pinned_group() {
    let mut sys = GcsSystem::new();
    anchor_pair(&mut sys, 0.0);
    let pinned = fixed_pt(&mut sys, 7.0, 3.0);
    sys.add_constraint(Constraint::FixX(pinned, 7.0)).unwrap();
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 2);
    assert!(!d.is_single_connected());
    let pinned_comp = d.components.last().unwrap();
    assert!(pinned_comp.is_pinned());
    assert_eq!(pinned_comp.constraint_ids.len(), 1);
    // Pinned rows still count as equations.
    assert_eq!(d.num_equations, 2);
}

#[test]
fn isolated_free_parameters_form_the_free_group() {
    let mut sys = GcsSystem::new();
    anchor_pair(&mut sys, 0.0);
    let _lonely = free_pt(&mut sys, 100.0, 100.0);
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 2);
    let free = d
        .components
        .iter()
        .find(|c| c.is_free())
        .expect("free group present");
    assert_eq!(free.params.len(), 2);
    // The lonely point keeps its two degrees of freedom.
    assert_eq!(d.num_params, 4);
}

#[test]
fn empty_and_trivial_shapes() {
    let mut sys = GcsSystem::new();
    let d = decompose(&mut sys);
    assert!(d.components.is_empty());
    assert_eq!((d.num_params, d.num_equations), (0, 0));

    free_pt(&mut sys, 1.0, 2.0);
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 1);
    assert!(d.components[0].is_free());

    let mut sys = GcsSystem::new();
    let p = fixed_pt(&mut sys, 1.0, 2.0);
    sys.add_constraint(Constraint::FixY(p, 2.0)).unwrap();
    let d = decompose(&mut sys);
    assert_eq!(d.components.len(), 1);
    assert!(d.components[0].is_pinned());
}

#[test]
fn connecting_constraint_joins_and_removal_splits() {
    let mut sys = GcsSystem::new();
    anchor_pair(&mut sys, 0.0);
    anchor_pair(&mut sys, 10.0);
    let before = decompose(&mut sys);
    assert_eq!(before.components.len(), 2);

    // Join the two free points with a coincident constraint.
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    // Insertion order: anchor, free, anchor, free.
    let cid = sys
        .add_constraint(Constraint::Coincident(ids[1], ids[3]))
        .unwrap();
    let joined = decompose(&mut sys);
    assert_eq!(joined.components.len(), 1);
    assert!(joined.is_single_connected());

    sys.remove_constraint(cid).unwrap();
    let split = decompose(&mut sys);
    assert_eq!(split.components.len(), 2);
    for (a, b) in before.components.iter().zip(split.components.iter()) {
        assert_eq!(a.params, b.params);
        assert_eq!(a.constraint_ids, b.constraint_ids);
    }
}

#[test]
fn decomposition_is_deterministic() {
    let mut sys = GcsSystem::new();
    for i in 0..5 {
        anchor_pair(&mut sys, 10.0 * i as f64);
    }
    let lonely = free_pt(&mut sys, 999.0, 999.0);
    let _ = lonely;
    let pinned = fixed_pt(&mut sys, 3.0, 4.0);
    sys.add_constraint(Constraint::FixY(pinned, 4.0)).unwrap();
    let a = decompose(&mut sys);
    let b = decompose(&mut sys);
    assert_eq!(a.components.len(), b.components.len());
    for (ca, cb) in a.components.iter().zip(b.components.iter()) {
        assert_eq!(ca.params, cb.params);
        assert_eq!(ca.constraint_ids, cb.constraint_ids);
    }
}

/// Every constraint variant must declare structural references.
///
/// Builds one constraint per variant over shared entities and asserts the
/// exact free-parameter set. Variants touching only fixed geometry map to the
/// empty set (the pinned group), everything else must name its parameters —
/// a variant silently missing here would solve in the wrong component.
#[test]
fn every_variant_declares_its_free_parameters() {
    let mut sys = GcsSystem::new();
    let p0 = fixed_pt(&mut sys, 0.0, 0.0);
    let p1 = free_pt(&mut sys, 1.0, 0.0);
    let p2 = free_pt(&mut sys, 0.0, 1.0);
    let p3 = free_pt(&mut sys, 3.0, 3.0);
    let p4 = free_pt(&mut sys, 5.0, 5.0);
    let pc = free_pt(&mut sys, 10.0, 10.0);
    let l01 = sys.add_line(p0, p1).unwrap();
    let l23 = sys.add_line(p2, p3).unwrap();
    let c1 = sys.add_circle(pc, 2.0).unwrap();
    let c2 = sys.add_circle(p4, 1.0).unwrap();
    let a1 = sys.add_arc(pc, p1, p2).unwrap();
    let a2 = sys.add_arc(p4, p1, p3).unwrap();
    let index = param_index_of(&mut sys);
    let resolve = |refs: &[ParamRef]| -> Vec<usize> {
        let mut v: Vec<usize> = refs.iter().map(|r| index[r]).collect();
        v.sort_unstable();
        v
    };
    let check = |c: Constraint, expected: &[ParamRef]| {
        let got = constraint_param_indices(&c, &sys, &index);
        assert_eq!(got, resolve(expected), "variant {c:?}");
    };

    check(
        Constraint::Coincident(p1, p2),
        &[X(p1), Y(p1), X(p2), Y(p2)],
    );
    check(
        Constraint::Distance(p1, p2, 1.0),
        &[X(p1), Y(p1), X(p2), Y(p2)],
    );
    check(
        Constraint::PointLineDistance(p3, l01, 0.0),
        &[X(p3), Y(p3), X(p1), Y(p1)],
    );
    check(Constraint::FixX(p1, 1.0), &[X(p1)]);
    check(Constraint::FixY(p1, 0.0), &[Y(p1)]);
    check(Constraint::FixX(p0, 0.0), &[]);
    check(Constraint::Horizontal(l01), &[Y(p1)]);
    check(Constraint::Vertical(l01), &[X(p1)]);
    check(
        Constraint::Angle(l01, l23, 0.5),
        &[X(p1), Y(p1), X(p2), Y(p2), X(p3), Y(p3)],
    );
    check(
        Constraint::Perpendicular(l01, l23),
        &[X(p1), Y(p1), X(p2), Y(p2), X(p3), Y(p3)],
    );
    check(
        Constraint::Parallel(l01, l23),
        &[X(p1), Y(p1), X(p2), Y(p2), X(p3), Y(p3)],
    );
    check(
        Constraint::PointOnCircle(p1, c1),
        &[X(p1), Y(p1), X(pc), Y(pc), R(c1)],
    );
    check(
        Constraint::PointOnArc(p3, a1),
        &[X(p3), Y(p3), X(pc), Y(pc), X(p1), Y(p1), X(p2), Y(p2)],
    );
    check(
        Constraint::TangentLineArc(l01, a1, p1),
        &[X(p1), Y(p1), X(pc), Y(pc), X(p2), Y(p2)],
    );
    check(
        Constraint::TangentArcArc(a1, a2, p1),
        &[
            X(pc),
            Y(pc),
            X(p1),
            Y(p1),
            X(p2),
            Y(p2),
            X(p4),
            Y(p4),
            X(p3),
            Y(p3),
        ],
    );
    check(
        Constraint::EqualRadiusArcArc(a1, a2),
        &[
            X(pc),
            Y(pc),
            X(p1),
            Y(p1),
            X(p2),
            Y(p2),
            X(p4),
            Y(p4),
            X(p3),
            Y(p3),
        ],
    );
    check(
        Constraint::EqualRadiusArcCircle(a1, c1),
        &[X(pc), Y(pc), X(p1), Y(p1), X(p2), Y(p2), R(c1)],
    );
    check(
        Constraint::ArcLength(a1, 1.0),
        &[X(pc), Y(pc), X(p1), Y(p1), X(p2), Y(p2)],
    );
    check(
        Constraint::ConcentricArcArc(a1, a2),
        &[
            X(pc),
            Y(pc),
            X(p1),
            Y(p1),
            X(p2),
            Y(p2),
            X(p4),
            Y(p4),
            X(p3),
            Y(p3),
        ],
    );
    check(
        Constraint::ConcentricArcCircle(a1, c1),
        &[X(pc), Y(pc), X(p1), Y(p1), X(p2), Y(p2)],
    );
    check(Constraint::CircleRadius(c1, 2.0), &[R(c1)]);
    check(Constraint::EqualRadiusCircleCircle(c1, c2), &[R(c1), R(c2)]);
    check(
        Constraint::EqualLength(l01, l23),
        &[X(p1), Y(p1), X(p2), Y(p2), X(p3), Y(p3)],
    );
    check(Constraint::Midpoint(p4, l01), &[X(p4), Y(p4), X(p1), Y(p1)]);
    check(
        Constraint::Symmetric(p1, p2, l23),
        &[X(p1), Y(p1), X(p2), Y(p2), X(p3), Y(p3)],
    );
    check(
        Constraint::TangentLineCircle(l01, c1),
        &[X(p1), Y(p1), X(pc), Y(pc), R(c1)],
    );
    check(
        Constraint::SymmetricAboutPoint(p1, p2, pc),
        &[X(p1), Y(p1), X(p2), Y(p2), X(pc), Y(pc)],
    );
}

/// Block dimensions and deterministic byte sizes at 1000 parameters.
///
/// No solve runs here (debug CI stays fast): the partition alone pins what
/// every later factorization allocates. Dense bytes are `m*n*8`; block bytes
/// are the sum over blocks. The 10000-parameter rows the S01 runner refuses
/// (800 MB dense) decompose into the same 2x2 blocks, which is why the
/// component path solves them.
#[test]
fn large_fixtures_pin_block_dimensions_and_byte_sizes() {
    // independent_solved_1000: 500 disjoint 2-param pairs.
    let mut sys = GcsSystem::new();
    for i in 0..500 {
        let ax = 10.0 * i as f64;
        let anchor = fixed_pt(&mut sys, ax, 0.0);
        let free = free_pt(&mut sys, ax + 1.0, 1.0);
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
        sys.add_constraint(Constraint::FixY(free, 4.0)).unwrap();
    }
    let d = decompose(&mut sys);
    assert_eq!((d.num_params, d.num_equations), (1000, 1000));
    assert_eq!(d.components.len(), 500);
    let mut block_bytes = 0_usize;
    let mut max_elems = 0_usize;
    for comp in &d.components {
        assert_eq!(comp.params.len(), 2);
        assert_eq!(comp.num_equations(), 2);
        block_bytes += comp.params.len() * comp.num_equations() * 8;
        max_elems = max_elems.max(comp.params.len() * comp.num_equations());
    }
    assert_eq!(block_bytes, 500 * 2 * 2 * 8);
    assert_eq!(max_elems, 4);
    // Dense equivalent for the record: 8,000,000 bytes per Jacobian.
    assert_eq!(d.num_params * d.num_equations * 8, 8_000_000);

    // coupled_chain_1000: one connected 1000x1000 block (dense path kept).
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(501);
    pts.push(fixed_pt(&mut sys, 0.0, 0.0));
    for i in 1..501 {
        pts.push(free_pt(&mut sys, i as f64, 0.5 * f64::from((i % 2) as u8)));
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
    }
    let d = decompose(&mut sys);
    assert!(d.is_single_connected());
    assert_eq!((d.num_params, d.num_equations), (1000, 1000));

    // mixed_1000: one 500-param chain block beside 250 tiny pair blocks.
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(251);
    pts.push(fixed_pt(&mut sys, 0.0, 0.0));
    for i in 1..251 {
        pts.push(free_pt(&mut sys, i as f64, 0.5 * f64::from((i % 2) as u8)));
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
    }
    for i in 0..250 {
        let ax = 1000.0 + 10.0 * i as f64;
        let anchor = fixed_pt(&mut sys, ax, 0.0);
        let free = free_pt(&mut sys, ax + 1.0, 1.0);
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
        sys.add_constraint(Constraint::FixY(free, 4.0)).unwrap();
    }
    let d = decompose(&mut sys);
    assert_eq!((d.num_params, d.num_equations), (1000, 1000));
    assert_eq!(d.components.len(), 251);
    let mut chain_blocks = 0;
    let mut pair_blocks = 0;
    let mut other_blocks = 0;
    let mut block_bytes = 0_usize;
    for comp in &d.components {
        block_bytes += comp.params.len() * comp.num_equations() * 8;
        if comp.params.len() == 500 {
            chain_blocks += 1;
            assert_eq!(comp.num_equations(), 500);
        } else if comp.params.len() == 2 {
            pair_blocks += 1;
        } else {
            other_blocks += 1;
        }
    }
    assert_eq!(other_blocks, 0, "every block is chain- or pair-sized");
    assert_eq!((chain_blocks, pair_blocks), (1, 250));
    assert_eq!(block_bytes, 500 * 500 * 8 + 250 * 2 * 2 * 8);
}
