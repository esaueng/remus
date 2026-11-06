//! The sphere-boss seam fillet (OpenZCAD K02/W01): its validation cost and
//! its cooperative cancellation / work-budget contract.
//!
//! An r 73.5 sphere fused with an r 6 boss, filleted at r 2 along the four
//! boss/sphere seam edges, took 1.6–1.7 s natively (minutes in a loaded
//! browser). 90% of it was the volume postcondition's convexity probes: they
//! tessellated the whole input at an absolute 0.01 deflection (271 k
//! triangles), once per engine attempt, because that mesh overflows the
//! scoped mesh memo. The probes now ask the exact classifier, so the fillet
//! builds no whole-solid robust mesh at all; that count, and the walker's
//! step count, are the deterministic work counters these tests pin.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::MathError;
use remus_math::context::{CancellationToken, OperationContext, WorkBudgets};
use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use super::{fillet_cascade, fillet_cascade_with_context, fillet_v2_with_context};
use crate::OperationsError;
use crate::boolean::{BooleanOp, boolean};
use crate::primitives::{make_cylinder, make_sphere};
use crate::transform::transform_solid;

/// The fused body and its boss/sphere seam edges, axis-aligned or with the
/// boss tilted 20° about X and 10° about Y before placement.
fn sphere_boss(topo: &mut Topology, tilted: bool) -> (SolidId, Vec<EdgeId>) {
    let sphere = make_sphere(topo, 73.5, 32).unwrap();
    transform_solid(topo, sphere, &Mat4::translation(25.5, -1.5, 0.0)).unwrap();
    let boss = make_cylinder(topo, 6.0, 64.1).unwrap();
    let place = if tilted {
        Mat4::translation(0.0, 0.0, 58.0)
            * Mat4::rotation_y(10f64.to_radians())
            * Mat4::rotation_x(20f64.to_radians())
    } else {
        Mat4::translation(0.0, 0.0, 58.0)
    };
    transform_solid(topo, boss, &place).unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, sphere, boss).unwrap();
    let adjacency = AdjacencyIndex::build(topo, fused).unwrap();
    let mut seams: Vec<EdgeId> = adjacency
        .edge_faces_iter()
        .filter(|(_, faces)| {
            let has = |sphere: bool| {
                faces.iter().any(|&face| {
                    matches!(
                        (sphere, topo.face(face).unwrap().surface()),
                        (true, FaceSurface::Sphere(_)) | (false, FaceSurface::Cylinder(_))
                    )
                })
            };
            faces.len() == 2 && has(true) && has(false)
        })
        .map(|(edge, _)| edge)
        .collect();
    seams.sort_by_key(|edge| edge.index());
    assert_eq!(
        seams.len(),
        4,
        "the fuse leaves four boss/sphere seam edges"
    );
    (fused, seams)
}

fn robust_mesh_builds() -> usize {
    crate::classify::ROBUST_MESH_BUILDS.with(std::cell::Cell::get)
}

fn state(topo: &Topology) -> String {
    format!("{topo:?}")
}

fn live_state(topo: &Topology) -> String {
    // Live arena content: a rolled-back attempt keeps the slots it allocated
    // retired (so no stale handle aliases a later entity), which a whole-arena
    // `Debug` would count as a difference.
    let loops: Vec<_> = topo
        .faces()
        .iter()
        .flat_map(|(_, face)| face.boundary_loops())
        .map(|id| topo.face_loop(*id).unwrap())
        .collect();
    let uses: Vec<_> = loops
        .iter()
        .flat_map(|loop_| loop_.coedges())
        .map(|id| topo.coedge(*id).unwrap())
        .collect();
    format!(
        "{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}",
        topo.vertices().iter().collect::<Vec<_>>(),
        topo.edges().iter().collect::<Vec<_>>(),
        topo.wires().iter().collect::<Vec<_>>(),
        topo.faces().iter().collect::<Vec<_>>(),
        topo.shells().iter().collect::<Vec<_>>(),
        topo.solids().iter().collect::<Vec<_>>(),
        loops,
        uses,
        topo.attributes(),
        topo.journal().entries()
    )
}

#[test]
fn sphere_boss_seam_fillet_builds_no_whole_solid_robust_mesh() {
    for tilted in [false, true] {
        let mut topo = Topology::new();
        let (solid, seams) = sphere_boss(&mut topo, tilted);
        let before = robust_mesh_builds();
        let result = fillet_cascade(&mut topo, solid, &seams, 2.0).unwrap();
        assert_eq!(
            robust_mesh_builds() - before,
            0,
            "tilted={tilted}: the volume sign rule classified the seams without \
             tessellating the whole input (it built two 271 k-triangle meshes before)"
        );
        assert!(!result.is_partial);
        let volume = crate::measure::solid_volume(&topo, result.solid, 0.1).unwrap();
        let input = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
        assert!(
            volume > input,
            "tilted={tilted}: a concave seam fillet adds material ({input} -> {volume})"
        );
    }
}

#[test]
fn default_context_matches_the_ungoverned_cascade_and_reports_its_walk_steps() {
    let mut plain = Topology::new();
    let (solid, seams) = sphere_boss(&mut plain, false);
    let ungoverned = fillet_cascade(&mut plain, solid, &seams, 2.0).unwrap();

    let mut topo = Topology::new();
    let (solid, seams) = sphere_boss(&mut topo, false);
    let governed =
        fillet_cascade_with_context(&mut topo, solid, &seams, 2.0, &OperationContext::new())
            .unwrap();
    assert_eq!(
        state(&topo),
        state(&plain),
        "the default context changes nothing"
    );
    assert_eq!(governed.result.solid, ungoverned.solid);

    // Deterministic work: the same run makes the same number of walker
    // steps. Today it is 40 (one 20-step closed walk, built twice by the
    // lazy material guard); the envelope leaves room for walker tuning but
    // catches a cascade that starts walking every attempt.
    let mut again = Topology::new();
    let (solid, seams) = sphere_boss(&mut again, false);
    let repeat =
        fillet_cascade_with_context(&mut again, solid, &seams, 2.0, &OperationContext::new())
            .unwrap();
    assert_eq!(repeat.walk_steps, governed.walk_steps);
    assert!(
        (1..=80).contains(&governed.walk_steps),
        "walk steps {} outside the pinned envelope",
        governed.walk_steps
    );
}

#[test]
fn a_spent_step_budget_refuses_typed_and_restores_the_topology() {
    let mut probe = Topology::new();
    let (solid, seams) = sphere_boss(&mut probe, true);
    let needed =
        fillet_cascade_with_context(&mut probe, solid, &seams, 2.0, &OperationContext::new())
            .unwrap()
            .walk_steps;

    for limit in [0, 1, needed - 1] {
        let mut topo = Topology::new();
        let (solid, seams) = sphere_boss(&mut topo, true);
        let before = live_state(&topo);
        let context =
            OperationContext::new().with_budgets(WorkBudgets::new().with_blend_walk_steps(limit));
        let error = fillet_cascade_with_context(&mut topo, solid, &seams, 2.0, &context)
            .err()
            .expect("an under-budget fillet must refuse");
        assert!(
            matches!(
                error,
                OperationsError::Math(MathError::ResourceLimitExceeded {
                    resource: remus_blend::governor::STEP_RESOURCE,
                    limit: reported,
                    actual,
                }) if reported == limit && actual == limit + 1
            ),
            "limit {limit}: {error:?}"
        );
        assert_eq!(super::blend_failure_code(&error), "resource-limit");
        assert_eq!(
            live_state(&topo),
            before,
            "limit {limit}: the refusal rolls back"
        );
    }

    // Exactly the steps the run needs is enough.
    let mut topo = Topology::new();
    let (solid, seams) = sphere_boss(&mut topo, true);
    let context =
        OperationContext::new().with_budgets(WorkBudgets::new().with_blend_walk_steps(needed));
    assert!(fillet_cascade_with_context(&mut topo, solid, &seams, 2.0, &context).is_ok());
}

#[test]
fn a_cancelled_token_refuses_typed_and_restores_the_topology() {
    for v2 in [false, true] {
        let mut topo = Topology::new();
        let (solid, seams) = sphere_boss(&mut topo, false);
        let before = live_state(&topo);
        let token = CancellationToken::new();
        token.cancel();
        let context = OperationContext::new().with_cancellation(token);
        let result = if v2 {
            fillet_v2_with_context(&mut topo, solid, &seams, 2.0, &context)
        } else {
            fillet_cascade_with_context(&mut topo, solid, &seams, 2.0, &context)
        };
        let error = result.err().expect("a cancelled fillet must refuse");
        assert!(
            matches!(error, OperationsError::Math(MathError::Cancelled)),
            "v2={v2}: {error:?}"
        );
        assert_eq!(super::blend_failure_code(&error), "cancelled");
        assert_eq!(live_state(&topo), before, "v2={v2}: the refusal rolls back");
    }
}

#[test]
fn v2_honours_the_step_budget_too() {
    let mut topo = Topology::new();
    let (solid, seams) = sphere_boss(&mut topo, false);
    let before = live_state(&topo);
    let context = OperationContext::new().with_budgets(WorkBudgets::new().with_blend_walk_steps(3));
    let error = fillet_v2_with_context(&mut topo, solid, &seams, 2.0, &context)
        .err()
        .expect("a three-step budget cannot walk the seam");
    assert!(matches!(
        error,
        OperationsError::Math(MathError::ResourceLimitExceeded {
            limit: 3,
            actual: 4,
            ..
        })
    ));
    assert_eq!(live_state(&topo), before);
}
