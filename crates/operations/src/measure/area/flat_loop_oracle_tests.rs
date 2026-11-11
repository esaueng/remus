//! Differential oracle for the sampled planar area: on planar faces whose
//! inner loops do not nest it returns bit for bit what the
//! subtract-every-hole version below (kept verbatim, only renamed) returned;
//! on a nested stack it adds the islands back.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use crate::measure::loop_fixtures::{Placement, SCALES, flat_faces, nested_face};

/// `planar_face_area` as it was before inner-wire nesting.
fn main_planar_face_area(topo: &Topology, face_id: FaceId) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let outer_wire = topo.wire(face.outer_wire())?;
    let outer_positions = collect_wire_positions(topo, outer_wire)?;

    let outer_area = newell_area(&outer_positions);

    // Subtract hole areas.
    let mut hole_area = 0.0;
    for &inner_wid in face.inner_wires() {
        let inner_wire = topo.wire(inner_wid)?;
        let inner_positions = collect_wire_positions(topo, inner_wire)?;
        hole_area += newell_area(&inner_positions);
    }

    Ok((outer_area - hole_area).abs())
}

#[test]
fn sampled_planar_area_is_unchanged_unless_loops_nest() {
    for scale in SCALES {
        for pl in Placement::all(scale) {
            let mut topo = Topology::new();
            for (label, face) in flat_faces(&mut topo, &pl) {
                let main = main_planar_face_area(&topo, face).unwrap();
                let now = planar_face_area(&topo, face, 1e-3 * scale).unwrap();
                assert_eq!(main.to_bits(), now.to_bits(), "{label}, scale {scale}");
            }
            // The samples chord the circles, so the nested area is only as
            // close as the sampling; the islands' signs are what is checked.
            // A billion times the scale from the origin, the sampled Newell
            // sum of absolute coordinates loses those digits (as on `main`).
            if pl.far() {
                continue;
            }
            let (face, area) = nested_face(&mut topo, &pl);
            let now = planar_face_area(&topo, face, 1e-3 * scale).unwrap();
            let main = main_planar_face_area(&topo, face).unwrap();
            assert!((now - area).abs() <= 1e-3 * area, "nested: {now} != {area}");
            assert!((main - area).abs() > 1.0 * scale * scale, "subtract-all");
        }
    }
}
