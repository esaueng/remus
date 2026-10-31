//! A full-turn cylinder band whose rims arrive in many arcs integrates over
//! its exact `(u, v)` rectangle, past the trim-sampling budget.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{PI, TAU};

use super::*;
use remus_math::curves::Circle3D;
use remus_math::surfaces::CylindricalSurface;
use remus_topology::edge::Edge;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const RADIUS: f64 = 2.0;
const HEIGHT: f64 = 3.0;

/// The band `r = 2`, `z ∈ [0, 3]` with its bottom rim split into `arcs`
/// arcs and its top rim one closed circle, joined by a seam line, wound like
/// `make_cylinder`'s lateral face.
fn band(arcs: u32) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let bottom = Circle3D::new(Point3::new(0.0, 0.0, 0.0), z, RADIUS).unwrap();
    let top = Circle3D::new(Point3::new(0.0, 0.0, HEIGHT), z, RADIUS).unwrap();
    let step = TAU / f64::from(arcs);
    let rim: Vec<_> = (0..arcs)
        .map(|k| topo.add_vertex(Vertex::new(bottom.evaluate(step * f64::from(k)), 1e-7)))
        .collect();
    let mut uses = Vec::new();
    for k in 0..arcs {
        let next = rim[usize::try_from((k + 1) % arcs).unwrap()];
        let mut arc = Edge::new(
            rim[usize::try_from(k).unwrap()],
            next,
            EdgeCurve::Circle(bottom.clone()),
        );
        arc.set_trim(Some((step * f64::from(k), step * f64::from(k + 1))));
        uses.push(OrientedEdge::new(topo.add_edge(arc), true));
    }
    let apex = topo.add_vertex(Vertex::new(top.evaluate(0.0), 1e-7));
    let mut circle = Edge::new(apex, apex, EdgeCurve::Circle(top));
    circle.set_trim(Some((0.0, TAU)));
    let circle = topo.add_edge(circle);
    let seam = topo.add_edge(Edge::new(rim[0], apex, EdgeCurve::Line));
    uses.extend([
        OrientedEdge::new(seam, true),
        OrientedEdge::new(circle, false),
        OrientedEdge::new(seam, false),
    ]);
    let wire = topo.add_wire(Wire::new(uses, true).unwrap());
    let surface = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), z, RADIUS).unwrap();
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(surface)));
    (topo, face)
}

/// 40 arcs at 128 samples each overrun the 4096-point trim budget; the band
/// still reads its closed-form area, and the volume term the one-arc band
/// reads.
#[test]
fn fragmented_rim_band_integrates_its_whole_rectangle() {
    let (whole_topo, whole) = band(1);
    let (split_topo, split) = band(40);
    let reference = integrate_face(&whole_topo, whole, 5).unwrap();
    let fragmented = integrate_face(&split_topo, split, 5).unwrap();
    let area = TAU * RADIUS * HEIGHT;
    assert!(
        (reference.area - area).abs() <= 1e-12 * area,
        "{}",
        reference.area
    );
    assert!(
        (fragmented.area - area).abs() <= 1e-12 * area,
        "{}",
        fragmented.area
    );
    // About the origin `P · N = r` on the wall, so its volume term is
    // r/3 · area = 2/3 · π r² h (the caps carry the other third).
    let volume = 2.0 / 3.0 * PI * RADIUS * RADIUS * HEIGHT;
    assert!(
        (fragmented.volume - volume).abs() <= 1e-12 * volume,
        "{}",
        fragmented.volume
    );
    assert!((fragmented.volume - reference.volume).abs() <= 1e-12 * volume);
}

/// A band that does not close one turn on each rim is no rectangle: the
/// recognizer declines it and the budget refusal stands.
#[test]
fn rims_short_of_a_turn_keep_the_budget_refusal() {
    let (topo, face) = band(40);
    let face_data = topo.face(face).unwrap().clone();
    let FaceSurface::Cylinder(cylinder) = face_data.surface() else {
        unreachable!()
    };
    assert_eq!(
        cylinder_band_domain(&topo, &face_data, cylinder).unwrap(),
        Some((0.0, HEIGHT))
    );
    // Drop one bottom arc: the bottom rim sweeps 39/40 of a turn.
    let mut topo = topo;
    let wire = face_data.outer_wire();
    let mut uses = topo.wire(wire).unwrap().edges().to_vec();
    uses.remove(0);
    let short = topo.add_wire(Wire::new(uses, true).unwrap());
    let short_face = Face::new(short, vec![], face_data.surface().clone());
    assert_eq!(
        cylinder_band_domain(&topo, &short_face, cylinder).unwrap(),
        None
    );
}
