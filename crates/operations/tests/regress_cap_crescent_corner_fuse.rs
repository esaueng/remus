//! Fusing a cylinder onto a box it protrudes from must keep the protruding
//! cap.
//!
//! The reported case is the OpenZCAD default layout — a box and a cylinder both
//! based at z=0 with equal height, so both cylinder caps are coplanar with the
//! box's top and bottom — where the cylinder wall pokes out past a vertical
//! corner of the box. Every such placement fell back to a co-refined mesh:
//! ~40-60 all-planar faces where a dozen analytic ones were due.
//!
//! Two independent defects, both about the cap disc the corner cuts in two:
//!
//! 1. The disc is cut into an in-body WEDGE and a protruding CRESCENT, and the
//!    two are bounded by the same chord walked opposite ways round the circle —
//!    so they carry the same vertex-pair edge set. The box floor's own wedge is
//!    coextensive with the cap's wedge and hashes with both, pulling all three
//!    into one same-domain group; the crescent was then dropped as a within-rank
//!    duplicate of the wedge it tiles the disc WITH. Its rim edges were left
//!    used once, the shell came back open, the gate rejected it.
//!
//! 2. The split parameter for a CLOSED rim was measured from the circle's own
//!    angular origin instead of the rim edge's seam vertex. Where those differ
//!    the wedge came back twice and the crescent not at all. The anchoring fix
//!    already existed for cylinder and cone rims and was scoped away from
//!    planes; a cap coplanar with a box face is exactly the plane case.
//!
//! 3. The remaining `cx=5, cy=4` placement crosses both side planes without
//!    swallowing the corner. Four rim crossings form two disjoint protruding
//!    segments. Although each split arc is minor, their chords can step across
//!    a circle extremum before the next station; the planar arrangement then
//!    classifies the wrong cap cell and its triangulation loses one segment.
//!
//! A closed manifold shell is NOT enough to pass here: a wrongly-traced arc
//! leaves the topology intact and the volume wrong, so every case is pinned
//! against the closed-form union volume as well.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point_robust};
use remus_operations::measure::mass_properties;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;

const BOX: (f64, f64, f64) = (30.0, 18.0, 24.0);
const RADIUS: f64 = 6.0;

struct Fused {
    topo: Topology,
    solid: remus_topology::solid::SolidId,
}

fn fuse(cx: f64, cy: f64, cz: f64, h: f64) -> Fused {
    let mut topo = Topology::new();
    let bx = make_box(&mut topo, BOX.0, BOX.1, BOX.2).unwrap();
    let cyl = make_cylinder(&mut topo, RADIUS, h).unwrap();
    transform_solid(&mut topo, cyl, &Mat4::translation(cx, cy, cz)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Fuse, bx, cyl).unwrap();
    Fused { topo, solid }
}

impl Fused {
    /// Curved faces present means the analytic result survived. A mesh
    /// fallback replaces every surface with a planar facet, so this is the
    /// only reliable tell — the fallback is watertight and passes validation.
    fn curved_faces(&self) -> usize {
        solid_faces(&self.topo, self.solid)
            .unwrap()
            .iter()
            .filter(|f| self.topo.face(**f).unwrap().surface().type_tag() != "plane")
            .count()
    }

    fn face_count(&self) -> usize {
        solid_faces(&self.topo, self.solid).unwrap().len()
    }

    /// `(free, non_manifold)` edge counts: an edge used once bounds a hole in
    /// the shell, one used three or more times is a branching junction.
    fn open_edges(&self) -> (usize, usize) {
        let mut usage: std::collections::HashMap<EdgeId, usize> = std::collections::HashMap::new();
        for fid in solid_faces(&self.topo, self.solid).unwrap() {
            let f = self.topo.face(fid).unwrap();
            for w in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in self.topo.wire(w).unwrap().edges() {
                    *usage.entry(oe.edge()).or_default() += 1;
                }
            }
        }
        (
            usage.values().filter(|n| **n == 1).count(),
            usage.values().filter(|n| **n >= 3).count(),
        )
    }

    fn volume(&self) -> f64 {
        remus_operations::measure::solid_volume(&self.topo, self.solid, 0.02).unwrap()
    }
}

/// Area of the radius-`RADIUS` disc centred at (`cx`, `cy`) lying inside the
/// box footprint, by direct integration over x. The integrand has vertical
/// tangents at the disc's extremes, so this uses many samples rather than a
/// high-order rule; it converges to ~1e-6 relative, far below the 1e-3 gate.
fn overlap_area(cx: f64, cy: f64) -> f64 {
    const N: usize = 2_000_000;
    let (x0, x1) = ((cx - RADIUS).max(0.0), (cx + RADIUS).min(BOX.0));
    if x1 <= x0 {
        return 0.0;
    }
    let dx = (x1 - x0) / N as f64;
    let mut acc = 0.0;
    for k in 0..N {
        let x = (k as f64 + 0.5).mul_add(dx, x0);
        let half = RADIUS
            .mul_add(RADIUS, -((x - cx) * (x - cx)))
            .max(0.0)
            .sqrt();
        let lo = (cy - half).max(0.0);
        let hi = (cy + half).min(BOX.1);
        acc += (hi - lo).max(0.0);
    }
    acc * dx
}

/// Closed-form volume of the union: box + cylinder - the prism they share.
fn expected_volume(cx: f64, cy: f64, cz: f64, h: f64) -> f64 {
    let z_lo = cz.max(0.0);
    let z_hi = (cz + h).min(BOX.2);
    let shared = overlap_area(cx, cy) * (z_hi - z_lo).max(0.0);
    (BOX.0 * BOX.1).mul_add(BOX.2, std::f64::consts::PI * RADIUS * RADIUS * h - shared)
}

/// Area of the minor circular segment beyond a chord whose perpendicular
/// distance from the circle centre is `distance`.
fn circular_segment_area(distance: f64) -> f64 {
    RADIUS * RADIUS * (distance / RADIUS).acos()
        - distance * RADIUS.mul_add(RADIUS, -(distance * distance)).sqrt()
}

fn assert_sound(cx: f64, cy: f64, cz: f64, h: f64) {
    let f = fuse(cx, cy, cz, h);
    let at = format!("cx={cx} cy={cy} cz={cz} h={h}");
    assert!(
        f.curved_faces() >= 1,
        "{at}: mesh fallback — {} faces, all planar",
        f.face_count()
    );
    let (free, nonman) = f.open_edges();
    assert_eq!(
        (free, nonman),
        (0, 0),
        "{at}: shell is not a closed manifold ({free} free, {nonman} non-manifold edges)"
    );
    let (got, want) = (f.volume(), expected_volume(cx, cy, cz, h));
    let rel = (got - want).abs() / want;
    assert!(
        rel < 1e-3,
        "{at}: volume {got:.4} but the closed form is {want:.4} (relative {rel:.2e})"
    );
}

#[test]
fn the_reported_corner_placement_stays_analytic() {
    // The minimal repro from the report.
    let f = fuse(-4.0, 4.0, 0.0, 24.0);
    assert!(
        f.curved_faces() >= 1,
        "reported placement fell back to a {}-face mesh",
        f.face_count()
    );
    assert_sound(-4.0, 4.0, 0.0, 24.0);
}

#[test]
fn the_protruding_cap_crescent_is_present_at_both_ends() {
    // The crescent is the part of the cap outside the box. Its area is the
    // disc minus the quarter the corner cuts off, and it must appear ONCE at
    // each end of the cylinder — the defect dropped it at one end, the other,
    // or both, depending on where the seam fell.
    let f = fuse(-4.0, 4.0, 0.0, 24.0);
    let crescent = std::f64::consts::PI.mul_add(RADIUS * RADIUS, -overlap_area(-4.0, 4.0));
    let mut found = 0;
    for fid in solid_faces(&f.topo, f.solid).unwrap() {
        let face = f.topo.face(fid).unwrap();
        if face.surface().type_tag() != "plane" {
            continue;
        }
        let area = remus_operations::measure::face_area(&f.topo, fid, 0.02).unwrap_or(0.0);
        if (area - crescent).abs() < 0.05 {
            found += 1;
        }
    }
    assert_eq!(
        found, 2,
        "expected the {crescent:.4}-area crescent at both z=0 and z=24, found {found}"
    );
}

/// Whether the box's vertical corner at the origin falls inside the cylinder.
///
/// This is the discriminator for the whole reported family: a cylinder that
/// crosses only ONE side face was always fine, and every failing placement
/// swallowed the corner. Placements that cross two side faces WITHOUT
/// swallowing the corner (`cx=5, cy=4` here) protrude as two disjoint lobes,
/// so they do not belong to this corner-swallowing sweep. Their separate
/// regression below checks both lobes directly.
fn swallows_corner(cx: f64, cy: f64) -> bool {
    cx.hypot(cy) < RADIUS
}

#[test]
fn two_side_lobes_without_the_corner_are_both_present() {
    // The circle crosses x=0 and y=0 but does not contain their corner. Its
    // two exterior circular segments are therefore disjoint, giving a closed
    // form for the union without numerical overlap integration.
    let f = fuse(5.0, 4.0, 0.0, BOX.2);
    assert!(
        f.curved_faces() >= 1,
        "two-lobe placement fell back to a {}-face mesh",
        f.face_count()
    );
    let (free, nonmanifold) = f.open_edges();
    assert_eq!(
        (free, nonmanifold),
        (0, 0),
        "two-lobe result is not a closed manifold ({free} free, {nonmanifold} non-manifold edges)"
    );

    for (label, probe) in [
        ("x-side lobe", Point3::new(-0.5, 4.0, BOX.2 / 2.0)),
        ("y-side lobe", Point3::new(5.0, -0.5, BOX.2 / 2.0)),
    ] {
        assert_eq!(
            classify_point_robust(&f.topo, f.solid, probe, 0.02, 1e-7).unwrap(),
            PointClassification::Inside,
            "{label} probe {probe:?} must remain inside the fused solid"
        );
    }

    let expected =
        BOX.0 * BOX.1 * BOX.2 + BOX.2 * (circular_segment_area(5.0) + circular_segment_area(4.0));
    let exact = mass_properties(&f.topo, f.solid).unwrap().mass;
    let exact_relative = (exact - expected).abs() / expected;
    assert!(
        exact_relative < 1e-8,
        "two-lobe exact volume {exact:.10} against closed form {expected:.10} ({exact_relative:.3e} relative)"
    );

    // `solid_volume` takes the result through its bounded tessellation path.
    // Before the fix that triangulation swallowed the y-side circular segment
    // and read 0.475% low even though the exact integral and both interior
    // probes above proved that the analytic B-rep still contained the lobe.
    let measured = f.volume();
    let measured_relative = (measured - expected).abs() / expected;
    assert!(
        measured_relative < 1e-5,
        "two-lobe measured volume {measured:.10} against closed form {expected:.10} ({measured_relative:.3e} relative)"
    );
}

#[test]
fn every_corner_placement_with_coplanar_caps_is_exact() {
    // The whole reported region: the cylinder based at z=0 with the box's
    // height, swept across the corner it swallows.
    let mut checked = 0;
    for cxi in -5..=5 {
        for cy in [-2.0, 0.0, 4.0] {
            let cx = f64::from(cxi);
            if !swallows_corner(cx, cy) {
                continue;
            }
            assert_sound(cx, cy, 0.0, 24.0);
            checked += 1;
        }
    }
    assert_eq!(checked, 31, "the corner-swallowing sweep changed shape");
}

#[test]
fn the_same_placements_survive_a_taller_cylinder() {
    // Cap coplanarity is not the trigger — the corner is — so a cylinder that
    // overhangs the box must behave identically.
    for cxi in -5..=5 {
        for cy in [-2.0, 0.0, 4.0] {
            let cx = f64::from(cxi);
            if !swallows_corner(cx, cy) {
                continue;
            }
            assert_sound(cx, cy, 0.0, 30.0);
        }
    }
}

/// Whether the cylinder axis lies on (or within a hair of) one of the box's
/// side-face planes.
///
/// A plane through the axis meets the cylinder in two straight generators
/// rather than an ellipse, and `exact_plane_cylinder` has no arm for a plane
/// parallel to the axis — it drops to a sampled point chain
/// (`math/src/analytic_intersection.rs`). Inside a band of roughly r/600 the
/// sampled chain degrades and the fuse loses the whole protrusion below the
/// box; the acceptance gate catches it every time (the result's bounding box
/// no longer contains the cylinder's) and the mesh fallback returns the right
/// volume, so these placements are correct but faceted. That is a separate,
/// pre-existing defect in a different crate — it reproduces with NO coplanar
/// cap at all — so it is excluded here rather than asserted.
fn axis_on_a_side_plane(cx: f64, cy: f64) -> bool {
    cx.abs() < 0.01 || cy.abs() < 0.01
}

#[test]
fn the_reported_document_with_only_its_top_cap_flush_is_exact() {
    // The user's actual document: the cylinder is taller than the box, its top
    // cap flush with the box top, hanging below. Only ONE cap is coplanar —
    // enough to trigger the defect — and this placement returned 57 all-planar
    // faces before the fix.
    let (cx, cy, cz, h) = (-4.0, 4.0, -6.0, 30.0);
    let f = fuse(cx, cy, cz, h);
    assert!(
        f.curved_faces() >= 1,
        "the reported document fell back to a {}-face mesh",
        f.face_count()
    );
    assert_sound(cx, cy, cz, h);
}

#[test]
fn one_flush_cap_is_enough_at_either_end() {
    // Top-flush (hangs below) and bottom-flush (sticks above) are the two
    // one-cap variants. Both faceted before the fix wherever the wall crossed
    // the corner, so both are swept here.
    for (cz, h) in [(-6.0, 30.0), (0.0, 30.0)] {
        let mut checked = 0;
        for cxi in -5..=5 {
            for cy in [-2.0, 4.0] {
                let cx = f64::from(cxi);
                if !swallows_corner(cx, cy) || axis_on_a_side_plane(cx, cy) {
                    continue;
                }
                assert_sound(cx, cy, cz, h);
                checked += 1;
            }
        }
        assert!(checked >= 15, "the cz={cz} sweep changed shape ({checked})");
    }
}

#[test]
fn the_result_does_not_depend_on_where_the_cylinder_seam_falls() {
    // Rotating the cylinder about its own axis moves the seam vertex without
    // changing the geometry at all, so every rotation must give the same
    // volume. The closed-rim anchoring defect made the answer depend on
    // whether the seam landed on the wedge's rim or the crescent's.
    let want = expected_volume(-4.0, 4.0, 0.0, 24.0);
    for deg in [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0] {
        let mut topo = Topology::new();
        let bx = make_box(&mut topo, BOX.0, BOX.1, BOX.2).unwrap();
        let cyl = make_cylinder(&mut topo, RADIUS, 24.0).unwrap();
        transform_solid(&mut topo, cyl, &Mat4::rotation_z(f64::to_radians(deg))).unwrap();
        transform_solid(&mut topo, cyl, &Mat4::translation(-4.0, 4.0, 0.0)).unwrap();
        let solid = boolean(&mut topo, BooleanOp::Fuse, bx, cyl).unwrap();
        let curved = solid_faces(&topo, solid)
            .unwrap()
            .iter()
            .filter(|f| topo.face(**f).unwrap().surface().type_tag() != "plane")
            .count();
        assert!(curved >= 1, "seam at {deg} deg fell back to a mesh");
        let got = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();
        let rel = (got - want).abs() / want;
        assert!(
            rel < 1e-3,
            "seam at {deg} deg: volume {got:.4} but the closed form is {want:.4}"
        );
    }
}
