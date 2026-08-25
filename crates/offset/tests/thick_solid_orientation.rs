//! A thick solid's outer skin must face outward.
//!
//! `thick_solid` keeps both skins — the offset one and a clone of the
//! original — and exactly one of them is the cavity. Two rules decided that
//! and both assumed the offset skin was the inner one:
//!
//! * the loop builder wound each offset face's wire against the face's
//!   EFFECTIVE normal rather than its stored surface normal, leaving the
//!   stored winding disagreeing with the stored surface;
//! * assembly then flipped the offset skin whenever any face was excluded,
//!   regardless of which way the offset actually ran.
//!
//! Shell orientation reads the wire traversal, so the first rule fed it a
//! contradiction and it propagated a shell that was edge-coherent but
//! geometrically inconsistent: the outer skin pointed inward while the cavity
//! stayed correct. Such a body measures as the SUM of its two skins instead of
//! their difference — a 10 mm box shelled 1 mm outward reported 2584 mm^3 for
//! a part containing 584 — and exports inside out.
//!
//! Every case here is an all-planar box, whose exact material volume is a
//! closed form, so the assertions are exact rather than approximate.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_offset::{OffsetOptions, thick_solid};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

fn opts() -> OffsetOptions {
    OffsetOptions {
        remove_self_intersections: false,
        ..Default::default()
    }
}

/// Material volume of a cube of side `s` shelled by `thickness` with one face
/// left open: the outer block minus the void it wraps.
///
/// Outward (`thickness > 0`) grows the block and leaves the original cube as
/// the void; inward keeps the cube as the block and shrinks the void. The open
/// face is not offset, so only two of the three extents pick up the full
/// double thickness.
fn expected_volume(s: f64, thickness: f64) -> f64 {
    if thickness > 0.0 {
        let t = thickness;
        t.mul_add(2.0, s).powi(2) * (s + t) - s.powi(3)
    } else {
        let h = -thickness;
        h.mul_add(-2.0, s).powi(2).mul_add(-(s - h), s.powi(3))
    }
}

/// Shell a cube with its first face left open.
fn shelled_cube(topo: &mut Topology, side: f64, thickness: f64) -> SolidId {
    let solid = make_box(topo, side, side, side).unwrap();
    let exclude = vec![solid_faces(topo, solid).unwrap()[0]];
    thick_solid(topo, solid, thickness, &exclude, opts())
        .unwrap_or_else(|e| panic!("shelling a {side} cube by {thickness} must succeed: {e}"))
}

#[test]
fn a_shelled_box_measures_its_wall_not_its_two_skins_added_up() {
    // 1.0 on a 10 mm cube is the approx_census `shell box (1 face open)` row,
    // which is what caught the inversion.
    for (side, thickness) in [
        (10.0, 1.0),
        (10.0, -1.0),
        (10.0, 0.25),
        (10.0, -4.0),
        (2.0, -0.2),
        (0.02, 0.002),
    ] {
        let mut topo = Topology::new();
        let result = shelled_cube(&mut topo, side, thickness);
        let got = solid_volume(&topo, result, 0.01).unwrap();
        let want = expected_volume(side, thickness);
        assert!(
            (got - want).abs() / want < 1e-9,
            "cube {side} shelled by {thickness}: want {want}, got {got}"
        );
    }
}

#[test]
fn a_shelled_box_is_not_inside_out() {
    for (side, thickness) in [(10.0, 1.0), (10.0, -1.0), (2.0, -0.2)] {
        let mut topo = Topology::new();
        let result = shelled_cube(&mut topo, side, thickness);
        assert!(
            !remus_operations::measure::solid_is_inverted(&topo, result).unwrap(),
            "cube {side} shelled by {thickness} came back inside out"
        );
        let report = remus_operations::validate::validate_solid(&topo, result).unwrap();
        assert!(
            report.is_valid(),
            "cube {side} shelled by {thickness} is invalid: {:?}",
            report
                .issues
                .iter()
                .map(|i| i.description.as_str())
                .collect::<Vec<_>>()
        );
    }
}

#[test]
fn every_face_of_a_shelled_box_winds_with_its_own_surface() {
    // The invariant the loop builder broke: a face's STORED wire winding
    // follows its STORED surface normal, whatever its reversal flag says.
    // Shell orientation and the planar tessellator both read the wire, the
    // property integrator reads the surface — they agree only while this
    // holds.
    for (side, thickness) in [(10.0, 1.0), (10.0, -1.0), (2.0, -0.2)] {
        let mut topo = Topology::new();
        let result = shelled_cube(&mut topo, side, thickness);
        let report = remus_check::validate::validate_solid(
            &topo,
            result,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        let inconsistent: Vec<_> = report
            .issues
            .iter()
            .filter(|i| i.check == remus_check::validate::CheckId::FaceOrientationConsistency)
            .map(|i| i.description.as_str())
            .collect();
        assert!(
            inconsistent.is_empty(),
            "cube {side} shelled by {thickness}: {inconsistent:?}"
        );
    }
}
