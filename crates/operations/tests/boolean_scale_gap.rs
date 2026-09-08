//! Small-model-scale booleans: the result must stay inside what its operands allow.
//!
//! At model scale 1e-3 (a 1 µm-featured body under the kernel's fixed 1e-7 mm
//! linear tolerance) a plain box ∖ box through-cut used to return a solid whose
//! hole walls kept the TOOL's full extent — they protruded beyond the blank on
//! both sides, and the measured volume EXCEEDED the blank's own (1.2e-9 against
//! a correct 0.84e-9). At scales 1 and 1e3 the same configuration is exact.
//!
//! What made it dangerous was not the wrong volume but that nothing said so.
//! The result carried the right face count (10), tessellated watertight and
//! manifold, and `validate_solid` returned `is_valid = true` — its eight
//! `VertexOnSurface` issues stay Warning because that check's tolerance is an
//! absolute 1e-4. Every structural gate in the boolean's acceptance path is
//! topological, so a well-formed solid in the wrong place passes all of them.
//!
//! Closed by `result_within_operand_bounds` (`boolean/mod.rs`), the upper half
//! of the contract whose lower half `operands_are_represented` already checked:
//! removing material cannot extend a shape, so a difference sits inside its
//! blank. The witness is a result VERTEX outside the allowed box — not the
//! result's own bounding box, which over-approximates trimmed curved faces (see
//! `curved_cut_result_is_not_rejected` below, which pins exactly that).
//!
//! The public gate first stopped silent wrong answers. Local junction and
//! face-arrangement bands now preserve the exact through-cut down to 1e-5;
//! the smaller 1e-6 witness still refuses under `ExactOnly`. The complete
//! operator/placement matrix lives in `qualify_boolean_scale.rs`.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean, boolean_with_context};
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_faces, solid_vertices};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Blank `s`³ with a `0.4s` square tool passed all the way through it, offset so
/// the hole is off-centre. The tool protrudes `0.5s` beyond the blank at both
/// ends, which is what the broken result used to keep.
fn through_cut(topo: &mut Topology, s: f64) -> (SolidId, SolidId) {
    let blank = make_box(topo, s, s, s).unwrap();
    let tool = make_box(topo, 0.4 * s, 0.4 * s, 2.0 * s).unwrap();
    transform_solid(topo, tool, &Mat4::translation(0.3 * s, 0.3 * s, -0.5 * s)).unwrap();
    (blank, tool)
}

#[test]
fn micron_scale_through_cut_volume_is_correct() {
    let s = 1e-3;
    let mut topo = Topology::new();
    let (blank, tool) = through_cut(&mut topo, s);
    let holed = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    let vol = solid_volume(&topo, holed, 0.01 * s).unwrap();
    let expected = 0.84 * s * s * s;
    assert!(
        ((vol - expected) / expected).abs() < 1e-6,
        "expected {expected:.6e}, got {vol:.6e}"
    );
}

/// The same configuration is exact at unit scale — the boundary's good side.
#[test]
fn unit_scale_through_cut_volume_is_correct() {
    let mut topo = Topology::new();
    let (blank, tool) = through_cut(&mut topo, 1.0);
    let holed = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    let vol = solid_volume(&topo, holed, 0.01).unwrap();
    assert!(
        ((vol - 0.84) / 0.84).abs() < 1e-9,
        "expected 0.84, got {vol}"
    );
}

/// The invariant itself, stated directly: a difference cannot reach outside the
/// blank it was cut from. This is the assertion that fails on the old defect
/// even where a volume comparison might not.
#[test]
fn cut_result_never_escapes_the_blank() {
    for exponent in [-5i32, -4, -3, -2, -1, 0, 3] {
        let s = f64::from(10i32).powi(exponent);
        let mut topo = Topology::new();
        let (blank, tool) = through_cut(&mut topo, s);
        let blank_box = solid_bounding_box(&topo, blank).unwrap();
        let Ok(holed) = boolean(&mut topo, BooleanOp::Cut, blank, tool) else {
            continue; // refusing is fine; answering wrongly is not
        };
        // Margin matches the acceptance gate's own: relative to the blank.
        let margin = (blank_box.max - blank_box.min).length() * 1e-6;
        let allowed = blank_box.expanded(margin);
        for vid in solid_vertices(&topo, holed).unwrap() {
            let p = topo.vertex(vid).unwrap().point();
            assert!(
                allowed.contains_point(p),
                "scale 1e{exponent}: cut result vertex ({:.4e}, {:.4e}, {:.4e}) lies outside \
                 the blank it was cut from",
                p.x(),
                p.y(),
                p.z()
            );
        }
    }
}

/// Fail closed, not open. Where the exact pipeline cannot hold at small scale,
/// `ExactOnly` must refuse — the one thing it must never do is report success
/// on the wrong solid, which is what this configuration did at 1e-3.
///
/// The boundary has moved twice, and both times the assertion for the freed
/// scale was *strengthened* rather than dropped: 1e-3 flipped exact when the
/// FF junction band's absolute 1e-3 floor was capped to the face pair's
/// extent, and 1e-4 flipped when the `tol.linear * 1000.0` term — which had
/// escaped that cap — was brought under it. Refusing is no longer the correct
/// answer at either scale, so `small_scale_cut_is_exact_under_exact_only`
/// demands the exact one. The local arrangement-band audit subsequently
/// freed 1e-5; this test pins the smaller scale where refusal still is.
#[test]
fn small_scale_cut_still_refuses_below_the_exact_boundary() {
    let s = 1e-6;
    let mut topo = Topology::new();
    let (blank, tool) = through_cut(&mut topo, s);
    let ctx = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(&mut topo, BooleanOp::Cut, blank, tool, &ctx);
    assert!(
        outcome.is_err(),
        "scale 1e-6: exact-only returned a result where the exact pipeline does not \
         hold; it must refuse instead"
    );
}

/// The other side of that boundary: down to 1e-4 the exact pipeline now
/// holds through 1e-5, so `ExactOnly` must return the exact answer — not refuse, and not
/// fall back.
#[test]
fn small_scale_cut_is_exact_under_exact_only() {
    for s in [1e-3, 2e-4, 1e-4, 1e-5] {
        let mut topo = Topology::new();
        let (blank, tool) = through_cut(&mut topo, s);
        let ctx = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
        let result = boolean_with_context(&mut topo, BooleanOp::Cut, blank, tool, &ctx);
        assert!(
            result.is_ok(),
            "exact-only must produce a result at {s:e}: {result:?}"
        );
        let outcome = result.unwrap();
        assert!(
            matches!(outcome.quality, BooleanQuality::Exact),
            "scale {s:e}: expected an exact result, got {:?}",
            outcome.quality
        );
        let vol = solid_volume(&topo, outcome.solid, 0.01 * s).unwrap();
        let expected = 0.84 * s * s * s;
        assert!(
            ((vol - expected) / expected).abs() < 1e-6,
            "scale {s:e}: expected {expected:.6e}, got {vol:.6e}"
        );
    }
}

/// Guards the reason the check tests VERTICES rather than the result's bounding
/// box. `solid_bounding_box` bounds a trimmed spherical patch by its untrimmed
/// surface, so this result — which genuinely lies inside the box it was cut
/// from — reports a box of [−1, 11]³ against a blank of [0, 10]³. A containment
/// test written against that box rejects this and three other correct census
/// rows, degrading them to a mesh fallback. Vertex positions carry no such
/// slack. If this ever comes back as a many-faced all-planar result, the
/// acceptance gate has started rejecting exact analytic geometry.
#[test]
fn curved_cut_result_is_not_rejected() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let sphere = make_sphere(&mut topo, 6.0, 24).unwrap();
    transform_solid(&mut topo, sphere, &Mat4::translation(5.0, 5.0, 5.0)).unwrap();

    let carved = boolean(&mut topo, BooleanOp::Cut, blank, sphere).unwrap();
    let faces = solid_faces(&topo, carved).unwrap();
    assert_eq!(
        faces.len(),
        8,
        "box − sphere should stay exact analytic at 8 faces, got {} — a mesh \
         fallback here means the containment check is rejecting correct geometry",
        faces.len()
    );
    let spherical = faces
        .iter()
        .filter(|&&fid| {
            matches!(
                topo.face(fid).map(|f| f.surface().clone()),
                Ok(FaceSurface::Sphere { .. })
            )
        })
        .count();
    assert!(
        spherical > 0,
        "the carved cavity must keep its spherical faces, not be re-meshed into planes"
    );
}
