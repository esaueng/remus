//! Isolated two-face fillet material sides (mid-span, away from vertices).
//!
//! Establishes through the production engine (`fillet_cascade` +
//! `classify_point`) the retained material sides that any corner model must
//! respect. The uniform rule: rolling-ball centers (spines) are on the
//! retained side; only the corner-side exterior is modified.
//!
//! - Convex edge (40x30x20 box, bottom-X edge, r=1, spine `(y,z)=(1,1)`):
//!   spine Inside, edge-side sliver Outside, both contacts OnBoundary,
//!   removal exactly `A(r)*L` with `A(r) = r^2*(1-pi/4)`.
//! - Concave edge (extruded L notch, vertical edge at `(8,8)`, r=1, spine
//!   `(9,9)`): spine Outside (void beyond the arc), corner-side lens
//!   Inside, both contacts OnBoundary, addition exactly `A(r)*L`.
//!
//! These are the analytic section/volume oracles for the mixed-notch corner
//! work: support tangencies are exact, sections are true rolling-ball arcs,
//! and volumes match closed form to mesh tolerance.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::query::filter_filletable_edges;
use remus_topology::Topology;
use remus_topology::explorer::solid_edges;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;

fn classify(topo: &Topology, solid: SolidId, p: [f64; 3]) -> PointClassification {
    classify_point(topo, solid, Point3::new(p[0], p[1], p[2]), 0.01, 1e-7).unwrap()
}

fn is_inside(topo: &Topology, solid: SolidId, p: [f64; 3]) -> bool {
    matches!(classify(topo, solid, p), PointClassification::Inside)
}

fn is_outside(topo: &Topology, solid: SolidId, p: [f64; 3]) -> bool {
    matches!(classify(topo, solid, p), PointClassification::Outside)
}

fn is_on(topo: &Topology, solid: SolidId, p: [f64; 3]) -> bool {
    matches!(classify(topo, solid, p), PointClassification::OnBoundary)
}

fn bottom_x_edge(topo: &Topology, solid: SolidId) -> remus_topology::edge::EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&eid| {
            let e = topo.edge(eid).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            let near = |q: Point3, x: f64, y: f64, z: f64| {
                (q.x() - x).abs() < 1e-9 && (q.y() - y).abs() < 1e-9 && (q.z() - z).abs() < 1e-9
            };
            (near(a, 0.0, 0.0, 0.0) && near(b, 40.0, 0.0, 0.0))
                || (near(b, 0.0, 0.0, 0.0) && near(a, 40.0, 0.0, 0.0))
        })
        .expect("bottom-X edge")
}

fn extruded_l(topo: &mut Topology) -> SolidId {
    use remus_operations::extrude::extrude;
    use remus_topology::builder::make_polygon_wire;
    let profile = make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(40.0, 0.0, 0.0),
            Point3::new(40.0, 8.0, 0.0),
            Point3::new(8.0, 8.0, 0.0),
            Point3::new(8.0, 50.0, 0.0),
            Point3::new(0.0, 50.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 20.0).unwrap()
}

/// Convex: spine retained, sliver removed, contacts exact, `-A(r)*L` exact.
#[test]
fn convex_edge_keeps_spine_and_cuts_sliver() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 40.0, 30.0, 20.0).unwrap();
    let target = bottom_x_edge(&topo, solid);
    let before = solid_volume(&topo, solid, 0.05).unwrap();
    let res = fillet_cascade(&mut topo, solid, &[target], 1.0).unwrap();
    let after = solid_volume(&topo, res.solid, 0.05).unwrap();
    let expected = before - (1.0 - std::f64::consts::FRAC_PI_4) * 40.0;
    assert!(
        (after - expected).abs() < 0.05,
        "convex removal must be A(1)*40: {before:.4} -> {after:.4}, expected {expected:.4}"
    );
    // Mid-span (x=20), away from vertices.
    assert!(
        is_inside(&topo, res.solid, [20.0, 1.0, 1.0]),
        "spine retained"
    );
    assert!(
        is_outside(&topo, res.solid, [20.0, 0.1, 0.1]),
        "sliver removed"
    );
    assert!(is_on(&topo, res.solid, [20.0, 1.0, 0.0]), "B contact exact");
    assert!(is_on(&topo, res.solid, [20.0, 0.0, 1.0]), "S contact exact");
    assert!(
        is_inside(&topo, res.solid, [20.0, 5.0, 5.0]),
        "deep material kept"
    );
    assert!(
        is_outside(&topo, res.solid, [20.0, 0.0, 0.0]),
        "edge line removed"
    );
}

/// Concave: spine stays void, lens added, contacts exact, `+A(r)*L` exact.
#[test]
fn concave_edge_leaves_spine_void_and_adds_lens() {
    let mut topo = Topology::new();
    let solid = extruded_l(&mut topo);
    let all = solid_edges(&topo, solid).unwrap();
    let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
    let target = physical
        .into_iter()
        .find(|&eid| {
            let e = topo.edge(eid).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            let at = |q: Point3| (q.x() - 8.0).abs() < 1e-9 && (q.y() - 8.0).abs() < 1e-9;
            at(a) && at(b)
        })
        .expect("concave notch edge");
    let before = solid_volume(&topo, solid, 0.05).unwrap();
    let res = fillet_cascade(&mut topo, solid, &[target], 1.0).unwrap();
    let after = solid_volume(&topo, res.solid, 0.05).unwrap();
    let expected = before + (1.0 - std::f64::consts::FRAC_PI_4) * 20.0;
    assert!(
        (after - expected).abs() < 0.05,
        "concave addition must be A(1)*20: {before:.4} -> {after:.4}, expected {expected:.4}"
    );
    // Mid-span (z=10), away from vertices.
    assert!(
        is_outside(&topo, res.solid, [9.0, 9.0, 10.0]),
        "spine stays void"
    );
    assert!(is_inside(&topo, res.solid, [8.1, 8.1, 10.0]), "lens added");
    assert!(
        is_on(&topo, res.solid, [9.0, 8.0, 10.0]),
        "S1 contact exact"
    );
    assert!(
        is_on(&topo, res.solid, [8.0, 9.0, 10.0]),
        "S2 contact exact"
    );
    assert!(
        is_outside(&topo, res.solid, [12.0, 12.0, 10.0]),
        "deep void kept void"
    );
}
