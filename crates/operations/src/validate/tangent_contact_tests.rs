//! esaueng/remus#1739: a hole cut exactly tangent to a wall leaves the solid
//! zero thickness along a line, and must not pass as valid.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use crate::boolean::{BooleanOp, boolean};
use crate::validate::validate_solid;
use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const CONTACT: &str = "touch along a line inside both faces";

fn placed_box(topo: &mut Topology, size: [f64; 3], at: [f64; 3]) -> SolidId {
    let solid = crate::primitives::make_box(topo, size[0], size[1], size[2]).unwrap();
    crate::transform::transform_solid(topo, solid, &Mat4::translation(at[0], at[1], at[2]))
        .unwrap();
    solid
}

/// 40 × 20 × 10 block stepped down 5 mm over half its length: tall for
/// x < 20 (`tall_left`) or for x > 20.
fn stepped(topo: &mut Topology, tall_left: bool) -> SolidId {
    let block = placed_box(topo, [40.0, 20.0, 10.0], [0.0, 0.0, 0.0]);
    let step = placed_box(
        topo,
        [20.0, 20.0, 10.0],
        [if tall_left { 20.0 } else { 0.0 }, 0.0, 5.0],
    );
    boolean(topo, BooleanOp::Cut, block, step).unwrap()
}

/// A vertical Ø6 through hole at (x, 10).
fn hole(topo: &mut Topology, x: f64) -> SolidId {
    let tool = crate::primitives::make_cylinder(topo, 3.0, 30.0).unwrap();
    crate::transform::transform_solid(topo, tool, &Mat4::translation(x, 10.0, -10.0)).unwrap();
    tool
}

/// The cut's verdict: `None` when the boolean refused, else whether the
/// result validates and whether it was flagged for tangent line contact.
fn drilled(target: impl Fn(&mut Topology) -> SolidId, x: f64) -> Option<(bool, bool)> {
    let mut topo = Topology::new();
    let target = target(&mut topo);
    let tool = hole(&mut topo, x);
    let result = boolean(&mut topo, BooleanOp::Cut, target, tool).ok()?;
    let report = validate_solid(&topo, result).unwrap();
    let flagged = report
        .issues
        .iter()
        .any(|issue| issue.description.contains(CONTACT));
    Some((report.is_valid(), flagged))
}

/// Never a valid result: refused, or reported invalid for the contact.
fn assert_not_valid(verdict: Option<(bool, bool)>, case: &str) {
    if let Some((valid, flagged)) = verdict {
        assert!(
            !valid && flagged,
            "{case}: valid {valid}, flagged {flagged}"
        );
    }
}

fn assert_valid(verdict: Option<(bool, bool)>, case: &str) {
    assert_eq!(verdict, Some((true, false)), "{case}");
}

#[test]
fn a_hole_tangent_to_a_step_wall_inside_the_tall_side_is_not_valid() {
    // The hole's wall touches the step wall along x = 20, z ∈ [5, 10]: the
    // material between them is zero thick there. The tall-right cut was
    // returned as a valid exact solid; the boolean already refused the
    // tall-left mirror.
    assert_not_valid(
        drilled(|t| stepped(t, false), 23.0),
        "tall right, hole at 23",
    );
    assert_not_valid(drilled(|t| stepped(t, true), 17.0), "tall left, hole at 17");
}

#[test]
fn a_hole_clear_of_the_step_wall_or_touching_it_at_a_point_stays_valid() {
    // 0.1 mm clear of the wall, inside the tall side.
    assert_valid(
        drilled(|t| stepped(t, false), 23.1),
        "tall right, hole at 23.1",
    );
    assert_valid(
        drilled(|t| stepped(t, true), 16.9),
        "tall left, hole at 16.9",
    );
    // In the low side the hole's rim meets the wall's foot at one point.
    // (The mirror image of this case is refused by the boolean today; that
    // refusal is not this check's concern.)
    assert_valid(drilled(|t| stepped(t, true), 23.0), "tall left, hole at 23");
}

#[test]
fn a_fillet_tangent_to_its_supports_along_its_edges_stays_valid() {
    let mut topo = Topology::new();
    let block = placed_box(&mut topo, [40.0, 20.0, 10.0], [0.0, 0.0, 0.0]);
    let edges = remus_topology::explorer::solid_edges(&topo, block).unwrap();
    let rounded = crate::blend_ops::fillet_v2(&mut topo, block, &edges, 2.0)
        .unwrap()
        .solid;
    let report = validate_solid(&topo, rounded).unwrap();
    assert!(report.is_valid(), "{report:?}");
    assert!(
        report
            .issues
            .iter()
            .all(|issue| !issue.description.contains(CONTACT)),
        "{report:?}"
    );
}

/// 40 × 20 × 10 block with an R4 fillet on its top edge at x = 40.
fn rounded_end(topo: &mut Topology) -> SolidId {
    let block = placed_box(topo, [40.0, 20.0, 10.0], [0.0, 0.0, 0.0]);
    let edge = remus_topology::explorer::solid_edges(topo, block)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let edge = topo.edge(edge).unwrap();
            [edge.start(), edge.end()].iter().all(|&v| {
                let p = topo.vertex(v).unwrap().point();
                (p.x() - 40.0).abs() < 1e-9 && (p.z() - 10.0).abs() < 1e-9
            })
        })
        .unwrap();
    crate::blend_ops::fillet_v2(topo, block, &[edge], 4.0)
        .unwrap()
        .solid
}

#[test]
fn a_hole_tangent_to_the_wall_below_a_fillet_band_is_not_valid() {
    // At x = 37 the hole's wall touches the end wall x = 40 along z ∈ [0, 6]
    // (the boolean refuses it today; this pins that it never validates).
    assert_not_valid(drilled(rounded_end, 37.0), "R4 band, hole at 37");
    // Clear of the end wall: through the flat top, and through the band.
    assert_valid(drilled(rounded_end, 30.0), "R4 band, hole at 30");
    if let Some(verdict) = drilled(rounded_end, 36.0) {
        assert_valid(Some(verdict), "R4 band, hole at 36");
    }
}

#[test]
fn the_exact_gate_refuses_the_tangent_cut_it_used_to_return() {
    let mut topo = Topology::new();
    let target = stepped(&mut topo, false);
    let tool = hole(&mut topo, 23.0);
    let refused = crate::boolean::boolean_with_options(
        &mut topo,
        BooleanOp::Cut,
        target,
        tool,
        crate::boolean::BooleanOptions::default(),
    );
    assert!(refused.is_err(), "{refused:?}");
}
