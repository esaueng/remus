//! Structured fuzzing of the boolean engine.
//!
//! Builds a bounded expression tree of primitives, rigid placements and
//! booleans, then asserts the properties a boolean must have. The point is not
//! to reach a panic in the engine — it is to catch a *plausible* answer that
//! is wrong: material invented, a bore filled, a shell left open, two volume
//! integrators disagreeing, or the same input producing two different results.
//!
//! Structural invariants run at every internal node; the expensive
//! measurement, determinism and idempotence batteries run once at the root.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod invariants;
mod shapegen;

use invariants as inv;
use remus_topology::Topology;
use shapegen::{Node, Refusal};

/// Cap on result complexity for the expensive root battery. A mesh fallback
/// can produce thousands of planar faces; measuring those four ways turns a
/// fuzz iteration into a timeout report, which teaches nothing.
const HEAVY_FACE_LIMIT: usize = 120;

fuzz_target!(|node: Node| {
    let mut topo = Topology::new();

    // Per-node structural checks, plus operand-relative volume bounds.
    // B26 oracle 5 (position-quantized closedness) runs per node alongside
    // the edge-id census gate.
    let mut violations_checked = 0usize;
    let root = shapegen::eval(&mut topo, &node, &mut |t, c| {
        violations_checked += 1;
        let what = format!("{} (node {violations_checked})", c.kind.name());

        // I1/I2 — the result is actually a solid.
        if let Ok(census) = inv::census(t, c.result) {
            inv::assert_closed_manifold(&what, &census);

            // B26/5 per node: the by-edge-ID gate above is blind to
            // position-duplicate free edges (distinct ids, coincident
            // geometry), so every node also runs the position-quantized
            // supplement. A typed refusal is a pass per the file's rule;
            // this check only reads topology, so it cannot refuse — any
            // position-level gap panics here, at the node that produced it.
            inv::assert_position_closed(&what, t, c.result);

            // I4 — the result may not exceed its operands. Both operands are
            // still live in `topo`, so this is checked where the engine made
            // the claim rather than only at the root.
            if census.faces <= HEAVY_FACE_LIMIT
                && let (Some(a), Some(b), Some(r)) = (
                    inv::measure(t, c.lhs),
                    inv::measure(t, c.rhs),
                    inv::measure(t, c.result),
                )
            {
                inv::assert_volume_bounds(&what, c.kind.name(), &a, &b, &r);

                // I5 (primary) — when the operands cannot overlap, the answer
                // is a number known before the engine ran, so the inequality
                // above becomes an equality. This is where a *dropped operand*
                // shows: a `fuse` that quietly returns its target alone, or a
                // `cut` that is a no-op, is well-formed, watertight, within
                // every bound — and wrong by exactly the operand it discarded.
                if c.disjoint
                    && let (Some(va), Some(vb)) = (c.lhs_exact, c.rhs_exact)
                {
                    inv::assert_disjoint_boolean_exact(&what, c.kind.name(), va, vb, r.volume);
                }
            }
        }
    });

    // Every `Err` here is the engine refusing: an empty algebraic result, an
    // unsupported configuration, a degenerate primitive. Refusing is a correct
    // outcome and is not a finding.
    let root = match root {
        Ok(root) => root,
        Err(Refusal::Engine(_) | Refusal::Degenerate) => return,
    };

    let Ok(census) = inv::census(&topo, root.solid) else {
        return;
    };
    inv::assert_closed_manifold("root", &census);

    if census.faces > HEAVY_FACE_LIMIT {
        return;
    }

    // B26/5 at the root: the position-quantized supplement to the id census.
    inv::assert_position_closed("root", &topo, root.solid);

    // I1 (mesh rung) — a closed B-Rep that tessellates leaky is still broken.
    // Necessary but not sufficient: #52 was watertight with the bore filled and
    // the bore walls missing, two errors that cancelled.
    if let Ok(aabb) = remus_operations::measure::solid_bounding_box(&topo, root.solid) {
        let diag = (aabb.max - aabb.min).length();
        inv::assert_watertight_mesh(
            "root",
            &topo,
            root.solid,
            inv::volume_deflection(diag) * 4.0,
        );
    }

    if let Some(m) = inv::measure(&topo, root.solid) {
        // I5 (primary) — a volume derived outside the kernel. This is the only
        // measurement oracle here that does not consult the code under test.
        if let Some(expected) = root.exact {
            inv::assert_exact_volume("root", expected, m.volume);
        }
        // I5 (secondary, weak) — the two internal routes share their face
        // integrator and agree even when both are wrong (#53). Kept only
        // because it catches a defect confined to one route (#46).
        inv::assert_measurements_agree("root", &topo, root.solid, m.volume);
        inv::assert_deflection_stable("root", &topo, root.solid, m.volume);
        // B26/3 at the root: volume must not follow the body through space.
        // A translation-variant reading is the doubled-boundary signature.
        inv::assert_translation_invariant("root", &topo, root.solid, m.volume, 13.25, -7.5, 3.0);
    }

    // B26/1+2+4 at the root: replay the root tree's top-level boolean twice
    // more per identity. The tree evaluator consumes operand handles, so this
    // replays the root node only when the root is a single Combine over two
    // freshly evaluable subtrees; deeper trees are covered per-node by I4/I5
    // above and by the nudge pair below.
    check_root_boolean_identities(&node);
    // B26/4 at the root: the exact outcome kind (refusal vs success, full
    // census, volume) must survive a rigid 1e-13 nudge of one operand.
    check_root_nudge_stable(&node);

    // I5b — the same shape at another size must give the same relative answers.
    // Aimed squarely at tolerances written as absolute distances.
    //
    // 0.001 is the metres-to-millimetres case. It used to be out of reach:
    // `transform_solid` rejected any uniform scale whose determinant fell under
    // `Tolerance.linear`, which for s^3 <= 1e-7 meant every s <= 0.00464, so
    // the sweep had to stop at 0.01. That guard is now dimensionless. See
    // `assert_scale_invariant` for the floor that remains.
    for s in [1000.0, 0.001] {
        inv::assert_scale_invariant("root", &topo, root.solid, s);
    }

    // I6 — determinism. A second evaluation of the identical tree, in a fresh
    // arena, must produce the identical fingerprint.
    let Some(fp1) = inv::fingerprint(&topo, root.solid) else {
        return;
    };
    let mut topo2 = Topology::new();
    if let Ok(root2) = shapegen::eval_quiet(&mut topo2, &node)
        && let Some(fp2) = inv::fingerprint(&topo2, root2.solid)
    {
        inv::assert_deterministic("root", &fp1, &fp2);
    }

    // I7 — idempotence. `fuse(r, r)` must be `r`. Built on a copy of the root
    // so the self-fuse operates on two distinct handles.
    check_self_fuse(&topo, root.solid, &census);
});

/// B26/1+2: inclusion–exclusion and cut complement at the root combine.
///
/// Replays the root `Combine(a, b)` with fresh operand evaluations so each
/// identity boolean gets unconsumed inputs: union+intersect for
/// inclusion–exclusion, cut+intersect for the complement. Every boolean here
/// runs `ExactOnly` with disclosed quality via [`inv::exact_boolean_outcome`]
/// on cloned arenas; any typed refusal on any leg is a pass. Non-`Combine`
/// roots (a lone primitive) have no identity to check.
fn check_root_boolean_identities(node: &shapegen::Node) {
    use remus_operations::boolean::BooleanOp;

    let shapegen::Node::Combine(kind, left, right) = node else {
        return;
    };
    // Measure the operands on a fresh evaluation of each subtree.
    let mut ta = Topology::new();
    let mut tb = Topology::new();
    let (Ok(va_node), Ok(vb_node)) = (
        shapegen::eval_quiet(&mut ta, left),
        shapegen::eval_quiet(&mut tb, right),
    ) else {
        return; // operand construction refused: nothing to check
    };
    let (Some(ma), Some(mb)) = (
        inv::measure(&ta, va_node.solid),
        inv::measure(&tb, vb_node.solid),
    ) else {
        return;
    };

    // Each identity leg runs on its own scratch arena so operand handles
    // stay valid across legs.
    let run_exact = |op: BooleanOp| -> Option<f64> {
        // Rebuild both operands in the scratch arena from the same subtrees.
        let mut s = Topology::new();
        let (Ok(x), Ok(y)) = (
            shapegen::eval_quiet(&mut s, left),
            shapegen::eval_quiet(&mut s, right),
        ) else {
            return None;
        };
        match inv::exact_boolean_outcome(&mut s, op, x.solid, y.solid) {
            inv::NudgeOutcome::Success { volume, .. } => Some(volume),
            inv::NudgeOutcome::Refused { .. } | inv::NudgeOutcome::Unmeasurable { .. } => None,
        }
    };

    match kind.op() {
        BooleanOp::Fuse | BooleanOp::Cut | BooleanOp::Intersect => {
            // Inclusion–exclusion needs union + intersect regardless of the
            // root op; cut complement needs cut + intersect.
            let (Some(v_union), Some(v_inter)) =
                (run_exact(BooleanOp::Fuse), run_exact(BooleanOp::Intersect))
            else {
                return;
            };
            inv::assert_inclusion_exclusion("root", ma.volume, mb.volume, v_union, v_inter);
            if let Some(v_cut) = run_exact(BooleanOp::Cut) {
                inv::assert_cut_complement("root", ma.volume, v_cut, v_inter);
            }
        }
    }
}

/// B26/4: the root combine's exact outcome must survive a rigid 1e-13 nudge.
///
/// Rebuilds both root operands fresh, runs the exact boolean on `(a, b)` and
/// on `(a, b + 1e-13 x via transform_solid)`, and compares refusal-vs-success
/// with [`inv::assert_nudge_stable`]. Either leg refusing outright is handled
/// inside the comparison (refused-vs-refused passes).
fn check_root_nudge_stable(node: &shapegen::Node) {
    let shapegen::Node::Combine(kind, left, right) = node else {
        return;
    };
    let mut s = Topology::new();
    let (Ok(x), Ok(y)) = (
        shapegen::eval_quiet(&mut s, left),
        shapegen::eval_quiet(&mut s, right),
    ) else {
        return;
    };
    let Some((base, nudged)) = inv::exact_boolean_nudged_pair(&s, kind.op(), x.solid, y.solid)
    else {
        return;
    };
    inv::assert_nudge_stable("root", &base, &nudged);
}

/// `fuse(a, a)` must be `a`: same hole count, same volume.
fn check_self_fuse(topo: &Topology, root: remus_topology::solid::SolidId, before: &inv::Census) {
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::copy::copy_solid;

    let mut t = topo.clone();
    let Ok(twin) = copy_solid(&mut t, root) else {
        return;
    };
    let Ok(fused) = boolean(&mut t, BooleanOp::Fuse, root, twin) else {
        return; // a refusal is a pass
    };
    let Ok(after) = inv::census(&t, fused) else {
        return;
    };
    inv::assert_closed_manifold("fuse(a, a)", &after);
    inv::assert_holes_preserved("fuse(a, a)", before, &after);

    let (Some(v0), Some(v1)) = (inv::measure(topo, root), inv::measure(&t, fused)) else {
        return;
    };
    inv::assert_idempotent("fuse(a, a)", before, &after, v0.volume, v1.volume);
}
