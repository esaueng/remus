//! A hollow body's Euler characteristic is 2 PER SHELL, not 2 in total.
//!
//! `check_euler` counts V, E and F over the whole solid — every shell, not
//! just the outer one — and used to compare that total against a flat 2. A
//! body hollowed by a fully enclosed cavity carries the cavity as a second
//! shell, and each closed genus-0 surface contributes 2 independently, so a
//! correct hollow body scores 4 and was reported as anomalous.
//!
//! The check is a `Warning`, so this never made a body invalid — it made every
//! hollowed body carry a permanent complaint, which is the way a validation
//! report stops being read.
//!
//! These live in `operations` rather than beside the check because building
//! the cavity needs a boolean, and `check` sits below `operations`.
#![allow(clippy::unwrap_used, clippy::cast_possible_wrap)]

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer;

fn euler_issues(topo: &Topology, solid: remus_topology::solid::SolidId) -> Vec<String> {
    validate_solid(topo, solid, &ValidateOptions::default())
        .unwrap()
        .issues
        .into_iter()
        .filter(|issue| issue.description.contains("Euler"))
        .map(|issue| issue.description)
        .collect()
}

#[test]
fn a_solid_block_scores_two_over_one_shell() {
    // The control. Without it, a check that never fires would pass the test
    // below just as happily as one that fires correctly.
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let (f, e, v) = explorer::solid_entity_counts(&topo, block).unwrap();
    assert_eq!((v, e, f), (8, 12, 6));
    assert_eq!(v as i64 - e as i64 + f as i64, 2);
    assert!(topo.solid(block).unwrap().inner_shells().is_empty());
    assert_eq!(euler_issues(&topo, block), Vec::<String>::new());
}

#[test]
fn a_hollow_block_scores_four_over_two_shells_and_is_not_flagged() {
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let void = make_box(&mut topo, 6.0, 6.0, 6.0).unwrap();
    // Off-centre on purpose: a concentric void is symmetric enough to hide
    // several unrelated defects, and this file should not depend on where it
    // sits.
    remus_operations::transform::transform_solid(
        &mut topo,
        void,
        &Mat4::translation(7.0, 7.0, 7.0),
    )
    .unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, void).unwrap();

    // The cavity really is a second shell, not a second lump. If this stops
    // holding, the assertion below is measuring something else.
    assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 1);

    let (f, e, v) = explorer::solid_entity_counts(&topo, hollow).unwrap();
    assert_eq!((v, e, f), (16, 24, 12));
    // Two cubes' worth of everything, so two lots of 2.
    assert_eq!(v as i64 - e as i64 + f as i64, 4);

    assert_eq!(
        euler_issues(&topo, hollow),
        Vec::<String>::new(),
        "a correct hollow body must not be reported as an Euler anomaly"
    );
}

#[test]
fn a_wrong_total_is_still_reported() {
    // The check must still fire on something. Two disjoint blocks in one solid
    // are two shells' worth of entities under ONE outer shell, so the total is
    // 4 where 2 is expected — the shape of anomaly this check exists to catch,
    // and proof the fix widened the expectation rather than disabling it.
    let mut topo = Topology::new();
    let left = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let right = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        right,
        &Mat4::translation(40.0, 0.0, 0.0),
    )
    .unwrap();
    let pair = boolean(&mut topo, BooleanOp::Fuse, left, right).unwrap();
    let (f, e, v) = explorer::solid_entity_counts(&topo, pair).unwrap();
    let chi = v as i64 - e as i64 + f as i64;
    let shells = 1 + topo.solid(pair).unwrap().inner_shells().len();
    if chi != 2 * shells as i64 {
        assert_eq!(
            euler_issues(&topo, pair).len(),
            1,
            "chi {chi} over {shells} shell(s) should have been reported"
        );
    }
}
