//! A convex fillet chained around a vertical corner fillet digs into the part.
//!
//! A 74x60x4 plate gets r = 6 fillets on its two back vertical corners, then
//! an r = 1 fillet on its back top edge. The selection expands along the G1
//! chain: back edge, the two quarter arcs on top of the r = 6 corners, and the
//! two side edges down to the front. The rolling ball builds the arc
//! stretches as NURBS walls against the corner cylinders, and those walls dip
//! through material: points inside the plate, well below the fillet band
//! (z < 3 here, under the cylinder's contact circle at z = 3), come out
//! outside. The result is valid and watertight, so nothing refuses it.
//!
//! The band only removes points within one radius of the top edges, so no
//! point below z = 3 may change sides.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::classify::{PointClassification, classify_points};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_edges;
use remus_topology::solid::SolidId;

fn edge_where(topo: &Topology, solid: SolidId, test: impl Fn(Point3, Point3) -> bool) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            test(a, b)
        })
        .expect("edge")
}

#[test]
#[ignore = "open: the G1 chain's NURBS walls around the r6 corner arcs remove material below the fillet band"]
fn g1_chain_around_corner_fillets_stays_in_its_band() {
    let near = |a: f64, b: f64| (a - b).abs() < 1e-6;
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 74.0, 60.0, 4.0).unwrap();
    let corners: Vec<EdgeId> = [0.0, 74.0]
        .into_iter()
        .map(|x| {
            edge_where(&topo, plate, |a, b| {
                near(a.x(), x) && near(b.x(), x) && near(a.y(), 60.0) && near(b.y(), 60.0)
            })
        })
        .collect();
    let rounded = fillet_cascade(&mut topo, plate, &corners, 6.0)
        .unwrap()
        .solid;
    let back = edge_where(&topo, rounded, |a, b| {
        near(a.y(), 60.0) && near(b.y(), 60.0) && near(a.z(), 4.0) && near(b.z(), 4.0)
    });
    let banded = fillet_cascade(&mut topo, rounded, &[back], 1.0)
        .unwrap()
        .solid;

    let mut points = Vec::new();
    for i in 0..=296 {
        for j in 0..=240 {
            for k in 0..12 {
                points.push(Point3::new(
                    f64::from(i) * 0.25 + 0.01,
                    f64::from(j) * 0.25 + 0.013,
                    f64::from(k) * 0.25 + 0.11,
                ));
            }
        }
    }
    let before = classify_points(&topo, rounded, &points, 0.01, 1e-7).unwrap();
    let after = classify_points(&topo, banded, &points, 0.01, 1e-7).unwrap();
    let dug: Vec<Point3> = points
        .iter()
        .zip(before.iter().zip(&after))
        .filter(|(_, (b, a))| {
            matches!(b, PointClassification::Inside) && matches!(a, PointClassification::Outside)
        })
        .map(|(p, _)| *p)
        .collect();
    assert!(
        dug.is_empty(),
        "{} points below the band were removed, e.g. {:?}",
        dug.len(),
        dug.first()
    );
}
