//! Long open spline arcs must retain their real winding, not their chord's.

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use super::*;
use remus_math::curves::Circle3D;
use remus_math::nurbs::NurbsCurve;
use remus_math::surfaces::CylindricalSurface;
use remus_topology::edge::Edge;
use remus_topology::face::Face;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn vertex(topo: &mut Topology, p: Point3) -> remus_topology::vertex::VertexId {
    topo.add_vertex(Vertex::new(p, 1e-8))
}

/// A 270-degree cylindrical wall, z=0 below and z=10+0.2x above.
/// The top is one exact rational spline; the bottom is three quarter arcs.
fn sector_wall(topo: &mut Topology) -> FaceId {
    let circle = Circle3D::new_with_ref(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        6.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let bottom: Vec<_> = (0..=3)
        .map(|i| vertex(topo, circle.evaluate(f64::from(i) * FRAC_PI_2)))
        .collect();
    let points = [
        (6., 0.),
        (6., 6.),
        (0., 6.),
        (-6., 6.),
        (-6., 0.),
        (-6., -6.),
        (0., -6.),
    ]
    .map(|(x, y)| Point3::new(x, y, 10.0 + 0.2 * x));
    let upper = NurbsCurve::new(
        2,
        vec![0., 0., 0., 1., 1., 2., 2., 3., 3., 3.],
        points.to_vec(),
        vec![
            1.,
            0.5_f64.sqrt(),
            1.,
            0.5_f64.sqrt(),
            1.,
            0.5_f64.sqrt(),
            1.,
        ],
    )
    .unwrap();
    let first = vertex(topo, points[0]);
    let last = vertex(topo, points[6]);
    let mut edges = Vec::new();
    for i in 0..3 {
        let mut edge = Edge::new(bottom[i], bottom[i + 1], EdgeCurve::Circle(circle.clone()));
        edge.set_trim(Some((i as f64 * FRAC_PI_2, (i + 1) as f64 * FRAC_PI_2)));
        edges.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let end = topo.add_edge(Edge::new(bottom[3], last, EdgeCurve::Line));
    let mut upper = Edge::new(first, last, EdgeCurve::NurbsCurve(upper));
    upper.set_trim(Some((0., 3.)));
    let upper = topo.add_edge(upper);
    let start = topo.add_edge(Edge::new(bottom[0], first, EdgeCurve::Line));
    edges.extend([
        OrientedEdge::new(end, true),
        OrientedEdge::new(upper, false),
        OrientedEdge::new(start, false),
    ]);
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Cylinder(
            CylindricalSurface::new(Point3::new(0., 0., 0.), Vec3::new(0., 0., 1.), 6.).unwrap(),
        ),
    ))
}

#[test]
fn a_long_open_spline_closes_the_wall_instead_of_winding_its_period() {
    let mut topo = Topology::new();
    let face = sector_wall(&mut topo);
    assert!(quadric_wall_is_notched_band(&topo, face));
    assert!(!quadric_wall_boundary_winds_period(&topo, face));
    assert_eq!(gauss_unqualified_face(&topo, face).unwrap(), None);
    let term = remus_check::properties::face_integrator::integrate_face_about(
        &topo,
        face,
        &remus_check::properties::PropertiesOptions::default(),
        Point3::new(0., 0., 0.),
    )
    .unwrap();
    // Divergence contribution: r²/3 ∫₀³π/₂ (10+0.2r cos θ) dθ.
    let expected = 180.0 * PI - 14.4;
    assert!(
        (term.volume - expected).abs() < 2e-5 * expected,
        "{}",
        term.volume
    );
}

#[test]
fn a_real_winding_notched_rim_still_declines() {
    let mut topo = Topology::new();
    let n = 64;
    let points: Vec<_> = (0..=n)
        .map(|i| {
            let t = TAU * f64::from(i) / f64::from(n);
            Point3::new(6. * t.cos(), 6. * t.sin(), 10. + t.sin())
        })
        .collect();
    let mut knots = vec![0.];
    knots.extend((0..=n).map(f64::from));
    knots.push(f64::from(n));
    let curve = NurbsCurve::new(1, knots, points.clone(), vec![1.; points.len()]).unwrap();
    let v = vertex(&mut topo, points[0]);
    let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some((0., f64::from(n))));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Cylinder(
            CylindricalSurface::new(Point3::new(0., 0., 0.), Vec3::new(0., 0., 1.), 6.).unwrap(),
        ),
    ));
    // Its coarse polygon samples the closed spline, so the genuine winding
    // and the existing conservative rejection remain unchanged.
    assert!(quadric_wall_boundary_winds_period(&topo, face));
}
