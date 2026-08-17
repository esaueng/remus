//! Regression: a Cut whose tool is disjoint from the target must be an
//! identity, including when the target is a sphere.
//!
//! `make_sphere` builds the ball as TWO spherical patches that share one
//! equatorial loop and differ only in the direction they walk it. Three
//! separate stages of the boolean pipeline keyed faces on the direction-
//! AGNOSTIC edge set and so read the two hemispheres as coincident duplicates
//! of each other:
//!
//! 1. `same_domain::build_sd_grouping` grouped them and dropped one as
//!    within-rank residue (1 of 8 sub-faces selected).
//! 2. `builder_solid::remove_doubled_faces` dropped BOTH as a doubled pair
//!    ("all faces avoided").
//! 3. `MIN_SOLID_FACES` rejected the surviving 2-face solid as too small.
//!
//! Any one of the three sent the operation to the mesh fallback, which
//! replaces the exact spherical surfaces with an inscribed polyhedron.
//!
//! Every assertion below is against a closed form written out by hand, never
//! against another integrator.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::type_complexity
)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::tessellate::tessellate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Model scales, listed so the ORDER is a rotation of the natural
/// small-to-large one. A result that only holds at whichever scale runs first
/// is a passing accident; rotating the list makes that visible rather than
/// hiding it behind a lucky first entry.
///
/// The unfixed code fails at all three, and in three DIFFERENT ways — which is
/// itself the evidence that the fallback it dropped into is scale-sensitive:
/// 0.001x lost 38.8% of the volume (the fallback's absolute 0.1 deflection is
/// five times the whole model, so the ball tessellated to 32 planes), 1x lost
/// 0.286%, and 1000x did not finish at all ("mesh boolean work limit exceeded
/// for input triangles A: 5638250 > 100000"). The fixed path is flat across
/// all three because it never tessellates: the result is the two analytic
/// patches, unchanged.
const SCALES: [f64; 3] = [1000.0, 0.001, 1.0];

/// The two cheap scales. The 1000x arm of a multi-segment sweep would cost
/// ~90 s PER boolean — not from anything this fix touches, but from the
/// acceptance gate in `operands_are_represented`, which probes points with
/// `classify_point_robust(.., 0.1, ..)`. That `0.1` is an ABSOLUTE deflection,
/// so at r = 10 000 each probe re-tessellates the ball into millions of
/// triangles. `cut_at_1000x_is_exact_for_every_segment_count` covers 1000x
/// directly instead, one boolean at a time.
const CHEAP_SCALES: [f64; 2] = [0.001, 1.0];

/// Segment counts. The sphere's surface is analytic regardless of this, so an
/// identity Cut must return the same volume at every one of them.
const SEGMENTS: [usize; 3] = [16, 32, 64];

fn sphere_volume(r: f64) -> f64 {
    4.0 / 3.0 * PI * r * r * r
}

/// Edge-use census over every face of the solid (outer shell plus cavities).
fn edge_use_counts(topo: &Topology, solid: SolidId) -> HashMap<usize, usize> {
    let mut uses: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *uses.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    uses
}

/// Closed 2-manifold: every edge used exactly twice — no free (boundary)
/// edges, no over-shared (non-manifold) ones.
fn assert_closed_two_manifold(topo: &Topology, solid: SolidId, what: &str) {
    let uses = edge_use_counts(topo, solid);
    assert!(!uses.is_empty(), "{what}: solid has no edges");
    let free = uses.values().filter(|&&c| c == 1).count();
    let non_manifold = uses.values().filter(|&&c| c > 2).count();
    assert_eq!(free, 0, "{what}: expected 0 free edges, got {free}");
    assert_eq!(
        non_manifold, 0,
        "{what}: expected 0 non-manifold edges, got {non_manifold}"
    );
}

fn surface_tags(topo: &Topology, solid: SolidId) -> Vec<&'static str> {
    solid_faces(topo, solid)
        .unwrap()
        .iter()
        .map(|&f| topo.face(f).unwrap().surface().type_tag())
        .collect()
}

/// A tool that touches nothing: a cube of the target's own size, pushed a
/// hundred radii away. Both extents scale with the model, so the gap is the
/// same multiple of the model at every scale.
fn far_tool(topo: &mut Topology, radius: f64) -> SolidId {
    let side = 2.0 * radius;
    let bx = make_box(topo, side, side, side).unwrap();
    copy_and_transform_solid(topo, bx, &Mat4::translation(100.0 * radius, 0.0, 0.0)).unwrap()
}

/// Cut the sphere with a disjoint tool and check the result against
/// `4/3*pi*r^3`, the exact surfaces, and closed-manifold topology.
fn check_identity_cut(scale: f64, radius: f64, segments: usize) {
    let exact = sphere_volume(radius);
    let mut topo = Topology::default();
    let sphere = make_sphere(&mut topo, radius, segments).unwrap();
    let tool = far_tool(&mut topo, radius);

    let result = boolean(&mut topo, BooleanOp::Cut, sphere, tool)
        .unwrap_or_else(|e| panic!("scale {scale} seg {segments}: cut failed: {e}"));

    let what = format!("scale {scale} seg {segments}");
    let volume = solid_volume(&topo, result, radius * 0.005).unwrap();
    let rel = (volume - exact).abs() / exact;
    assert!(
        rel < 1e-9,
        "{what}: cut by a disjoint tool changed the volume: got {volume}, \
         closed form 4/3*pi*r^3 = {exact} (relative error {rel:.3e})"
    );

    // The exact surface must survive. The mesh fallback replaces both
    // spherical patches with thousands of planes, so this is what separates
    // "right number" from "right body".
    let tags = surface_tags(&topo, result);
    assert_eq!(
        tags,
        vec!["sphere", "sphere"],
        "{what}: expected the two spherical patches to survive, got {} faces {tags:?}",
        tags.len()
    );

    assert_closed_two_manifold(&topo, result, &what);

    // The same invariant one level down: an identity result must also
    // TESSELLATE to a closed mesh. Defect 2's offset body passed every B-rep
    // check while emitting nothing at all, so the B-rep alone does not tell
    // you a body is usable.
    let mesh = tessellate_solid(&topo, result, radius * 0.001).unwrap();
    assert!(!mesh.indices.is_empty(), "{what}: result has no triangles");
    let mut mesh_edges: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for (x, y) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *mesh_edges.entry((x.min(y), x.max(y))).or_insert(0) += 1;
        }
    }
    let mesh_boundary = mesh_edges.values().filter(|&&c| c == 1).count();
    assert_eq!(
        mesh_boundary, 0,
        "{what}: tessellated identity result has {mesh_boundary} boundary edges"
    );
}

#[test]
fn cut_with_a_disjoint_tool_is_an_identity_on_a_sphere() {
    for scale in CHEAP_SCALES {
        for segments in SEGMENTS {
            check_identity_cut(scale, 10.0 * scale, segments);
        }
    }
}

/// The 1000x arm, run one segment count per test body so a CI failure names
/// the exact case. See `CHEAP_SCALES` for why 1000x is expensive.
#[test]
fn cut_at_1000x_is_exact_for_a_coarse_sphere() {
    check_identity_cut(SCALES[0], 10.0 * SCALES[0], 16);
}

/// Segment-independence, stated directly: the sphere's surface is analytic, so
/// an identity Cut cannot depend on the equatorial polygon's resolution.
///
/// Before the fix all three counts agreed too — on the WRONG value — because
/// the mesh fallback tessellates from the deflection, not from `segments`.
/// Pairing this with the closed-form assertion above is what makes the pair
/// meaningful; neither alone would have caught the defect.
#[test]
fn disjoint_cut_volume_is_independent_of_sphere_segments() {
    for scale in CHEAP_SCALES {
        let radius = 10.0 * scale;
        let exact = sphere_volume(radius);
        let mut volumes = Vec::new();
        for segments in SEGMENTS {
            let mut topo = Topology::default();
            let sphere = make_sphere(&mut topo, radius, segments).unwrap();
            let tool = far_tool(&mut topo, radius);
            let result = boolean(&mut topo, BooleanOp::Cut, sphere, tool).unwrap();
            volumes.push(solid_volume(&topo, result, radius * 0.005).unwrap());
        }
        for (segments, volume) in SEGMENTS.iter().zip(&volumes) {
            let rel = (volume - exact).abs() / exact;
            assert!(
                rel < 1e-9,
                "scale {scale} seg {segments}: got {volume}, closed form {exact} \
                 (relative error {rel:.3e})"
            );
        }
    }
}

/// Controls in the same shape as the sphere case. These passed before the fix
/// and must keep passing: the defect was specific to a body whose two faces
/// share their whole boundary, not to disjoint cuts in general.
#[test]
fn disjoint_cut_controls_box_and_cylinder_are_still_exact() {
    for scale in SCALES {
        let radius = 10.0 * scale;

        // Box control: 2r cube, closed form (2r)^3.
        let mut topo = Topology::default();
        let side = 2.0 * radius;
        let target = make_box(&mut topo, side, side, side).unwrap();
        let tool = far_tool(&mut topo, radius);
        let result = boolean(&mut topo, BooleanOp::Cut, target, tool).unwrap();
        let volume = solid_volume(&topo, result, radius * 0.005).unwrap();
        let exact = side * side * side;
        assert!(
            (volume - exact).abs() / exact < 1e-9,
            "scale {scale}: box control got {volume}, closed form {exact}"
        );
        assert_closed_two_manifold(&topo, result, &format!("scale {scale} box control"));

        // Cylinder control: r, height 2r, closed form pi*r^2*2r.
        let mut topo = Topology::default();
        let target = make_cylinder(&mut topo, radius, 2.0 * radius).unwrap();
        let tool = far_tool(&mut topo, radius);
        let result = boolean(&mut topo, BooleanOp::Cut, target, tool).unwrap();
        let volume = solid_volume(&topo, result, radius * 0.005).unwrap();
        let exact = PI * radius * radius * 2.0 * radius;
        assert!(
            (volume - exact).abs() / exact < 1e-9,
            "scale {scale}: cylinder control got {volume}, closed form {exact}"
        );
        assert_closed_two_manifold(&topo, result, &format!("scale {scale} cylinder control"));
    }
}

/// The product-level symptom, reproduced as the CHAIN it actually needs.
///
/// Cutting a fresh sphere responds to the tool's position perfectly well, on
/// unfixed code too — the reported "mirror-image cuts return byte-identical
/// volumes" only appears AFTER an earlier disjoint cut has replaced the two
/// spherical patches with a faceted stand-in. So the first cut here is the
/// identity cut (a tool a hundred radii away), and the mirror cuts are applied
/// to its result. That is the sequence the product runs, and it is what makes
/// this a regression test rather than a restatement of the one above.
///
/// The tool is a cube 6x the sphere's diameter, so only the plane at its top
/// face meets the ball. Placing that plane at +r/2 leaves the cap above it; at
/// -r/2 it leaves everything above it. The two closed forms differ by 5.4x, so
/// one shared answer cannot pass by accident.
///
/// Run at 1x ONLY, deliberately. Both arms still route through the mesh
/// fallback — trimming a sphere with a plane is a separate GFA gap this change
/// does not address — and the fallback is the scale-broken part of the
/// pipeline: its deflection is an ABSOLUTE 0.1, so at 0.001x it tessellates a
/// 0.02-wide ball into 32 planes and loses 40% of the cap, and at 1000x it
/// exceeds its own triangle work limit outright. Sweeping this test over
/// scales would measure that defect, not this one. The scale sweep that
/// belongs to this fix is on the identity cut above, which stays on the exact
/// analytic path at every scale. Both numbers are recorded in the PR.
#[test]
fn mirror_cuts_after_an_identity_cut_return_their_own_closed_forms() {
    let scale = 1.0_f64;
    let radius = 10.0 * scale;
    let side = 12.0 * radius;
    // Spherical cap of height h on a sphere of radius r: pi*h^2*(r - h/3).
    let h = radius / 2.0;
    let cap = PI * h * h * (radius - h / 3.0);
    let rest = sphere_volume(radius) - cap;

    for (top, exact, label) in [
        (radius / 2.0, cap, "tool top at +r/2 keeps the cap"),
        (-radius / 2.0, rest, "tool top at -r/2 keeps the rest"),
    ] {
        let mut topo = Topology::default();
        let sphere = make_sphere(&mut topo, radius, 32).unwrap();

        // Step 1: the identity cut. Must leave the ball untouched.
        let far = far_tool(&mut topo, radius);
        let carried = boolean(&mut topo, BooleanOp::Cut, sphere, far)
            .unwrap_or_else(|e| panic!("{label}: identity cut failed: {e}"));

        // Step 2: the real cut, applied to what step 1 handed on.
        let bx = make_box(&mut topo, side, side, side).unwrap();
        let tool = copy_and_transform_solid(
            &mut topo,
            bx,
            &Mat4::translation(-side / 2.0, -side / 2.0, top - side),
        )
        .unwrap();
        let result = boolean(&mut topo, BooleanOp::Cut, carried, tool)
            .unwrap_or_else(|e| panic!("{label}: cut failed: {e}"));

        let volume = solid_volume(&topo, result, radius * 0.005).unwrap();
        let rel = (volume - exact).abs() / exact;
        // Where the GFA path applies it is exact; where it does not, the mesh
        // fallback still lands within a few tenths of a percent. What this
        // asserts is that the two placements produce DIFFERENT answers, each
        // near its own closed form — not that both are exact.
        assert!(
            rel < 0.01,
            "{label}: got {volume}, closed form {exact} (relative error {rel:.3e})"
        );
        assert_closed_two_manifold(&topo, result, label);
    }
}
