//! Whole-edge fillet of the rib-on-plate fixture (convex-singleton mirror).
//!
//! Fixture: backplate box 60(X) x 50(Y) x 6(Z) at the origin plus one
//! reinforcing rib box 6 x 20 x 18 standing on the plate at
//! x in [12,18], y in [10,30], z in [6,24] (fused). The four rib-base
//! vertices each join two concave base edges with one convex vertical rib
//! edge — the convex-singleton mirror of the L-bracket notch — and close
//! with exact torus patches (ring center `V-R(wa+wb)+R.cap`, ring `2R`,
//! tube `R`, verified against the isolated independent reference at
//! V=(12,10,6): C2'=(13,11,7)).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::query::{edge_is_g1, filter_filletable_edges};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Plate 60x50x6 + one rib 6x20x18 at x=[12,18], y=[10,30], z=[6,24].
fn rib_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 60.0, 50.0, 6.0).unwrap();
    let rib = make_box(topo, 6.0, 20.0, 18.0).unwrap();
    transform_solid(topo, rib, &Mat4::translation(12.0, 10.0, 6.0)).unwrap();
    boolean(topo, BooleanOp::Fuse, plate, rib).unwrap()
}

fn fillet_all(topo: &mut Topology, solid: SolidId, radius: f64) -> SolidId {
    let all = solid_edges(topo, solid).unwrap();
    let physical = filter_filletable_edges(topo, solid, &all).unwrap();
    let result = fillet_cascade(topo, solid, &physical, radius)
        .unwrap_or_else(|e| panic!("r={radius}: whole-edge fillet must build: {e}"));
    assert!(!result.is_partial, "r={radius}: no partial blends");
    result.solid
}

fn torus_count(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .iter()
        .filter(|fid| matches!(topo.face(**fid).unwrap().surface(), FaceSurface::Torus(_)))
        .count()
}

/// Whole-edge r=1: builds, closes the four rib bases with mirror tori,
/// stays valid, removes material.
#[test]
fn rib_base_mirrors_close_whole_edge() {
    let mut topo = Topology::new();
    let solid = rib_plate(&mut topo);
    let before = solid_volume(&topo, solid, 0.01).unwrap();
    // 60*50*6 + 6*20*18.
    assert!(
        (before - (18000.0 + 2160.0)).abs() < 0.5,
        "rib-plate fixture must be {before:.4}"
    );
    let out = fillet_all(&mut topo, solid, 1.0);
    assert_eq!(
        torus_count(&topo, out),
        4,
        "all four rib-base vertices must close with mirror tori"
    );
    let report = validate_solid(&topo, out, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "filleted rib plate must validate");
    let after = solid_volume(&topo, out, 0.01).unwrap();
    assert!(
        after < before,
        "a convex fillet must remove material: {before:.4} -> {after:.4}"
    );
    eprintln!("rib-plate r=1: {before:.4} -> {after:.4}");
}

/// G1 continuity across every mirror seam (station circles + tangency arcs):
/// 4 seam arcs per patch x 4 rib-base vertices = 16.
#[test]
fn rib_mirror_seams_are_g1() {
    use remus_math::vec::Point3;
    let mut topo = Topology::new();
    let solid = rib_plate(&mut topo);
    let out = fillet_all(&mut topo, solid, 1.0);
    let mut seam_count = 0;
    for eid in solid_edges(&topo, out).unwrap() {
        let edge = topo.edge(eid).unwrap();
        if !matches!(edge.curve(), remus_topology::edge::EdgeCurve::Circle(_)) {
            continue;
        }
        let a = topo.vertex(edge.start()).unwrap().point();
        let b = topo.vertex(edge.end()).unwrap().point();
        let near_base =
            |p: Point3| p.x() > 10.5 && p.x() < 19.5 && p.y() > 8.5 && p.y() < 31.5 && p.z() < 8.5;
        if near_base(a) && near_base(b) {
            let mut incident = vec![];
            for fid in solid_faces(&topo, out).unwrap() {
                let face = topo.face(fid).unwrap();
                let mut wires = vec![face.outer_wire()];
                wires.extend(face.inner_wires().iter().copied());
                for wid in wires {
                    for oe in topo.wire(wid).unwrap().edges() {
                        if oe.edge() == eid {
                            incident.push(fid);
                        }
                    }
                }
            }
            incident.sort_by_key(|f| f.index());
            incident.dedup();
            assert_eq!(incident.len(), 2, "seam edge {eid:?} must join two faces");
            assert!(
                edge_is_g1(&topo, eid, incident[0], incident[1]).unwrap(),
                "rib mirror seam {eid:?} ({a:?}->{b:?}) must be G1"
            );
            seam_count += 1;
        }
    }
    assert_eq!(
        seam_count, 16,
        "all rib mirror seams must be present and G1"
    );
}
