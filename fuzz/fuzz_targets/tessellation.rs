//! Structured fuzzing of the tessellation engine.
//!
//! Builds a bounded primitive leaf (optionally rigidly placed, per
//! `shapegen`) and tessellates it at two deflections. The oracle is
//! independent of the mesher under test: the hand-derived closed-form volume
//! that `shapegen::build_prim_measured` returns alongside each primitive —
//! the same oracle the boolean targets use, computed from the construction
//! magnitudes without consulting any kernel measurement path.
//!
//! Properties checked per case:
//!
//! * The mesh is non-empty and every triangle index resolves to a vertex
//!   (structural soundness — a malformed mesh is a finding, not a skip).
//!   B-Rep closedness is gated by the census before meshing; mesh-level
//!   seam cracks over a closed B-Rep are a tessellation remark, not a
//!   finding here (see `mesh_volume`).
//! * The signed mesh volume at both deflections agrees with the closed form
//!   within [`VOL_SLACK`] (the shared gross-disagreement band), and the
//!   finer deflection is not farther from the closed form than the coarse
//!   one by more than the same band (no refinement divergence).
//!
//! Inputs are primitive leaves only \u2014 never placements, never boolean
//! trees: a rigid placement can rotate a sphere's seam/pole frame out of
//! the tessellator's weld alignment (a PI/6-rotated sphere tessellates with
//! 24 seam boundary edges where the unplaced one closes exactly), and a
//! misclassified boolean hands the mesher a wrong-but-closed solid \u2014 both
//! failure classes are owned elsewhere (seam handling, `boolean_tree`).
//! Restricting the generator keeps every tessellation finding attributable
//! to the mesher on unplaced primitives (pure tessellation fidelity).
//!
//! **A typed refusal is a pass.** Tessellation or construction errors stop
//! the case silently. Panics only fire on malformed-but-successful output.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod invariants;
mod shapegen;

use invariants::VOL_SLACK;
use remus_operations::measure::solid_bounding_box;
use remus_operations::tessellate::tessellate_solid;
use remus_topology::Topology;

/// Cap on faces for the mesh battery: beyond this a fuzz iteration is a
/// timeout report, not a correctness report.
const FACE_LIMIT: usize = 120;

/// Absolute floor, so near-zero volumes do not divide the relative test.
const VOL_FLOOR: f64 = 1e-6;

fuzz_target!(|prim: shapegen::Prim| {
    // Leaves only (see the module docs): placements rotate seam/pole
    // frames out of weld alignment and booleans entangle misclassification.
    let mut topo = Topology::new();
    let root = match shapegen::build_prim_measured(&mut topo, prim) {
        Ok((solid, exact)) => shapegen::Valued {
            solid,
            exact: Some(exact),
        },
        Err(shapegen::Refusal::Engine(_) | shapegen::Refusal::Degenerate) => return,
    };
    let Ok(census) = invariants::census(&topo, root.solid) else {
        return;
    };
    if census.faces > FACE_LIMIT {
        return;
    }
    // Closed-form volume is known only for constructions the generator can
    // price by hand (primitives under rigid placements). Without it there
    // is no independent oracle here.
    let Some(expected) = root.exact else {
        return;
    };
    if !expected.is_finite() {
        return;
    }

    let Ok(aabb) = solid_bounding_box(&topo, root.solid) else {
        return;
    };
    let diag = (aabb.max - aabb.min).length();
    if !(diag.is_finite() && diag > 0.0) {
        return;
    }
    // Two deflections a factor of 4 apart, both finer than the volume
    // clamp (`bbox_diag * 5e-5`) so they genuinely differ.
    let coarse = (diag * 4e-5).max(1e-7) * 4.0;
    let fine = (diag * 4e-5).max(1e-7);

    let (Some(v_coarse), Some(v_fine)) = (
        mesh_volume(&topo, root.solid, coarse),
        mesh_volume(&topo, root.solid, fine),
    ) else {
        return; // tessellation refusal is a pass
    };

    let check = |label: &str, v: f64| {
        assert!(
            v.is_finite(),
            "{label}: mesh volume is {v} — successful output must be finite",
        );
        let scale = expected.abs().max(v.abs()).max(VOL_FLOOR);
        let rel = (expected - v).abs() / scale;
        assert!(
            rel <= VOL_SLACK,
            "{label}: mesh volume {v:.9} disagrees with closed form {expected:.9} \
             (relative error {rel:.3e})",
        );
    };
    check("coarse", v_coarse);
    check("fine", v_fine);

    // Refinement must not diverge from the closed form: the fine reading
    // must be at least as close as the coarse one, within the same band.
    let err = |v: f64| (expected - v).abs() / expected.abs().max(VOL_FLOOR);
    assert!(
        err(v_fine) <= err(v_coarse) + VOL_SLACK,
        "refinement diverged: coarse error {:.3e}, fine error {:.3e} (closed form {expected:.9})",
        err(v_coarse),
        err(v_fine),
    );
});

/// Tessellate and return the signed mesh volume, after asserting structural
/// soundness (non-empty, valid indices). Returns `None` on tessellation
/// refusal.
///
/// Watertightness is deliberately NOT asserted here: a closed B-Rep can
/// tessellate with per-face seam boundary edges (the tessellator's own
/// documented behavior — see the `solid-verification` skill, rung 4), so a
/// leaky mesh over a census-closed solid is a tessellation remark, not a
/// target finding. B-Rep closedness is already gated by the census before
/// this function is reached; mesh-level closure of boolean results is owned
/// by the `boolean_tree` target's `assert_watertight_mesh` battery.
fn mesh_volume(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
    deflection: f64,
) -> Option<f64> {
    let mesh = tessellate_solid(topo, solid, deflection).ok()?;
    assert!(
        !mesh.indices.is_empty() && mesh.indices.len().is_multiple_of(3),
        "tessellation returned a malformed index buffer (len {})",
        mesh.indices.len(),
    );
    let n_verts = mesh.positions.len();
    assert!(
        mesh.indices.iter().all(|&i| (i as usize) < n_verts),
        "tessellation returned out-of-range triangle indices",
    );
    // Signed volume via divergence (tetrahedra from the origin).
    let mut vol = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let p = |i: u32| mesh.positions[i as usize];
        let (a, b, c) = (p(tri[0]), p(tri[1]), p(tri[2]));
        let (ax, ay, az) = (a.x(), a.y(), a.z());
        let (bx, by, bz) = (b.x(), b.y(), b.z());
        let (cx, cy, cz) = (c.x(), c.y(), c.z());
        vol += ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
    }
    Some(vol / 6.0)
}
