//! The boolean wrapper's cheap disjointness refutation must reach the verdict
//! of the full per-component witness it short-circuits.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::{BooleanOp, aabbs_clear_gap, assembly, boolean, solids_provably_disjoint};
use crate::primitives::{make_box, make_cylinder, make_sphere};
use crate::transform::transform_solid;

/// The witness before the vertex-box refutation, verbatim.
fn sampled_witness(topo: &Topology, a: SolidId, b: SolidId, margin: f64) -> bool {
    let comps_a = assembly::face_components(topo, a);
    let comps_b = assembly::face_components(topo, b);
    if comps_a.is_empty() || comps_b.is_empty() {
        return false;
    }
    let boxes = |comps: &[Vec<FaceId>]| -> Option<Vec<remus_math::aabb::Aabb3>> {
        comps
            .iter()
            .map(|faces| {
                let has_nurbs = faces.iter().try_fold(false, |has_nurbs, &face_id| {
                    topo.face(face_id)
                        .map(|face| has_nurbs || matches!(face.surface(), FaceSurface::Nurbs(_)))
                        .ok()
                })?;
                if has_nurbs {
                    return None;
                }
                crate::measure::face_set_bounding_box(topo, faces).ok()
            })
            .collect()
    };
    let (Some(boxes_a), Some(boxes_b)) = (boxes(&comps_a), boxes(&comps_b)) else {
        return false;
    };
    boxes_a
        .iter()
        .all(|ba| boxes_b.iter().all(|bb| aabbs_clear_gap(ba, bb, margin)))
}

fn moved(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) -> SolidId {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
    solid
}

#[test]
fn vertex_box_refutation_matches_the_sampled_witness() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let far = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let far = moved(&mut topo, far, 20.0, 0.0, 0.0);
    let touching = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let touching = moved(&mut topo, touching, 10.0, 0.0, 0.0);
    let overlapping = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let overlapping = moved(&mut topo, overlapping, 9.0, 4.0, 4.0);
    let inside = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let inside = moved(&mut topo, inside, 4.0, 4.0, 4.0);
    // A cylinder's vertices sit on its seam, so its vertex box misses most of
    // the circle: these probes have clear vertex boxes but overlapping (or
    // only nearly clear) sampled boxes, exercising the full witness.
    let drum = make_cylinder(&mut topo, 3.0, 2.0).unwrap();
    let drum = moved(&mut topo, drum, 20.0, 20.0, 0.0);
    let behind_seam = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    // Inside the drum, opposite its seam: x ∈ [17.5, 18.5] against a vertex
    // box at x = 23.
    let behind_seam = moved(&mut topo, behind_seam, 17.5, 19.5, 0.5);
    let clear_of_drum = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let clear_of_drum = moved(&mut topo, clear_of_drum, 15.0, 30.0, 0.5);
    let ball = make_sphere(&mut topo, 2.0, 16).unwrap();
    let ball = moved(&mut topo, ball, 14.0, 5.0, 5.0);
    // A two-lump accumulator whose overall box spans a probe it never meets.
    let left = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let left = moved(&mut topo, left, 0.0, 40.0, 0.0);
    let right = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let right = moved(&mut topo, right, 10.0, 40.0, 0.0);
    let lumps = boolean(&mut topo, BooleanOp::Fuse, left, right).unwrap();
    let between = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let between = moved(&mut topo, between, 5.0, 40.0, 0.0);

    let solids = [
        base,
        far,
        touching,
        overlapping,
        inside,
        drum,
        behind_seam,
        clear_of_drum,
        ball,
        lumps,
        between,
    ];
    let margin = remus_math::tolerance::Tolerance::new().linear;
    let mut disjoint = 0;
    let mut compared = 0;
    for &a in &solids {
        for &b in &solids {
            if a == b {
                continue;
            }
            let expected = sampled_witness(&topo, a, b, margin);
            assert_eq!(
                solids_provably_disjoint(&topo, a, b, margin),
                expected,
                "solids {} and {}",
                a.index(),
                b.index()
            );
            disjoint += usize::from(expected);
            compared += 1;
        }
    }
    assert!(
        disjoint > 10,
        "only {disjoint} of {compared} pairs were disjoint"
    );
    // base × {touching, overlapping, inside} and drum × behind_seam, both
    // orders.
    assert_eq!(compared - disjoint, 8, "refuted pairs of {compared}");
    assert!(!sampled_witness(&topo, drum, behind_seam, margin));
    assert!(sampled_witness(&topo, lumps, between, margin));
}
