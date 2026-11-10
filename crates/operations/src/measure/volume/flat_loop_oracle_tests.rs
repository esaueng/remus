//! Differential oracle for [`planar_face_signed_volume`]: on planar faces
//! whose inner loops do not nest it returns every field bit for bit what the
//! subtract-every-hole version below (kept verbatim, only renamed) returned;
//! on a nested stack it adds the islands back, matching the closed form.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use crate::measure::loop_fixtures::{Placement, SCALES, flat_faces, nested_face};

/// `planar_face_signed_volume` as it was before inner-wire nesting.
fn main_planar_face_signed_volume(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Option<PlanarFaceExact>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Plane { normal, d } = face.surface() else {
        return Ok(None);
    };
    let normal = *normal;
    // `d` is the offset along the STORED normal (`P·n = d`); the divergence
    // term needs it along the UNIT normal, so scale by 1/|n| for the callers
    // that hand in an unnormalised plane.
    let n_len = normal.length();
    if n_len <= 0.0 || !n_len.is_finite() {
        return Ok(None);
    }
    let d = *d / n_len;

    // Right-handed in-plane frame: ex × ey = normal, so a boundary wound CCW as
    // seen from +normal yields a positive signed area.
    let frame = match remus_math::frame::Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), normal) {
        Ok(f) => f,
        Err(_) => return Ok(None),
    };
    let (ex, ey) = (frame.x, frame.y);

    // Hole areas subtract by MAGNITUDE, not by trusting the stored hole-wire
    // winding: boolean results can emit an inner rim wound the SAME way as the
    // outer (the #1045 hole-winding class), which would ADD the hole's disc.
    // A hole is inside the outer by definition, so |outer| − Σ|hole| is the
    // geometric area either way.
    let Some(outer) = planar_wire_signed_area2(topo, face.outer_wire(), ex, ey)? else {
        return Ok(None);
    };
    let mut arc_edges = outer.arc_edges;
    let mut arc_length = outer.arc_length;
    let mut area_mag2 = outer.area2.abs();
    let mut exact_boundary = outer.exact_boundary;
    for &iw in face.inner_wires() {
        let Some(hole) = planar_wire_signed_area2(topo, iw, ex, ey)? else {
            return Ok(None);
        };
        arc_edges += hole.arc_edges;
        arc_length += hole.arc_length;
        exact_boundary &= hole.exact_boundary;
        area_mag2 -= hole.area2.abs();
    }
    if area_mag2 < 0.0 {
        return Ok(None); // holes exceed the outer boundary — not a sane cap
    }

    // `area_mag2/2` is the geometric area. The divergence-theorem contribution
    // is (1/3)·(p·n̂_out)·|A|, where the outward normal is `+normal` for a
    // forward face and `−normal` for a reversed one, so `p·n̂_out = ±d` (the
    // plane offset along that outward normal). The sign therefore comes only
    // from the outward offset, not from the wire winding.
    let area = area_mag2 / 2.0;
    let d_out = if face.is_reversed() { -d } else { d };
    Ok(Some(PlanarFaceExact {
        volume: d_out * area / 3.0,
        area,
        arc_edges,
        arc_length,
        exact_boundary,
    }))
}

fn fields(exact: &PlanarFaceExact) -> (u64, u64, usize, u64, bool) {
    (
        exact.volume.to_bits(),
        exact.area.to_bits(),
        exact.arc_edges,
        exact.arc_length.to_bits(),
        exact.exact_boundary,
    )
}

#[test]
fn planar_face_signed_volume_is_unchanged_unless_loops_nest() {
    for scale in SCALES {
        for pl in [Placement::flat(scale), Placement::tilted(scale)] {
            let mut topo = Topology::new();
            for (label, face) in flat_faces(&mut topo, &pl) {
                let main = main_planar_face_signed_volume(&topo, face).unwrap();
                let now = planar_face_signed_volume(&topo, face).unwrap();
                assert_eq!(
                    main.as_ref().map(fields),
                    now.as_ref().map(fields),
                    "{label}, scale {scale}"
                );
            }
            let (face, area) = nested_face(&mut topo, &pl);
            let now = planar_face_signed_volume(&topo, face).unwrap().unwrap();
            assert!(
                (now.area - area).abs() <= 1e-9 * area,
                "nested, scale {scale}: {} != {area}",
                now.area
            );
        }
    }
}
