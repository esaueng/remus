//! Independent planar oracles for a trim crossing many spline knot spans.

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use super::*;
use remus_topology::edge::Edge;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn trimmed_plane(transpose: bool, reverse: bool, hole: bool) -> (Topology, FaceId) {
    let spans = 1024;
    let mut knots = vec![0.];
    knots.extend((0..=spans).map(|k| k as f64 / spans as f64));
    knots.push(1.);
    let points: Vec<Vec<_>> = [0., 1.]
        .into_iter()
        .map(|u| {
            (0..=spans)
                .map(|k| Point3::new(u, k as f64 / spans as f64, 2.))
                .collect()
        })
        .collect();
    let weights = vec![vec![1.; spans + 1]; 2];
    let surface = if transpose {
        NurbsSurface::new(
            1,
            1,
            knots,
            vec![0., 0., 1., 1.],
            (0..=spans)
                .map(|k| points.iter().map(|r| r[k]).collect())
                .collect(),
            vec![vec![1.; 2]; spans + 1],
        )
    } else {
        NurbsSurface::new(1, 1, vec![0., 0., 1., 1.], knots, points, weights)
    }
    .unwrap();
    let mut topo = Topology::new();
    let mut wire = |points: &[(f64, f64)]| {
        let vertices: Vec<_> = points
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 2.), 1e-8)))
            .collect();
        let edges = (0..vertices.len())
            .map(|k| {
                let edge = topo.add_edge(Edge::new(
                    vertices[k],
                    vertices[(k + 1) % vertices.len()],
                    EdgeCurve::Line,
                ));
                OrientedEdge::new(edge, true)
            })
            .collect();
        topo.add_wire(Wire::new(edges, true).unwrap())
    };
    let outer = wire(&[(0., 0.), (1., 0.), (1., 0.75), (0., 0.25)]);
    let inner = if hole {
        vec![wire(&[(0.4, 0.1), (0.4, 0.2), (0.6, 0.2), (0.6, 0.1)])]
    } else {
        vec![]
    };
    let mut face = Face::new(outer, inner, FaceSurface::Nurbs(surface));
    face.set_reversed(reverse);
    let face = topo.add_face(face);
    (topo, face)
}

#[test]
fn a_many_span_trapezoid_measures_in_either_parameter_direction() {
    for transpose in [false, true] {
        for reverse in [false, true] {
            for hole in [false, true] {
                let (topo, face) = trimmed_plane(transpose, reverse, hole);
                let expected = if hole { 0.48 } else { 0.5 };
                let sign = if transpose ^ reverse { -1. } else { 1. };
                for order in [3, 5] {
                    let read = integrate_face(&topo, face, order).unwrap();
                    assert!((read.area - expected).abs() < 1e-10, "{}", read.area);
                    assert!(
                        (read.volume - sign * 2. / 3. * expected).abs() < 1e-10,
                        "{}",
                        read.volume
                    );
                    let shifted =
                        integrate_face_fixed_about(&topo, face, order, Point3::new(0., 0., 1.))
                            .unwrap();
                    assert!((shifted.volume - sign / 3. * expected).abs() < 1e-10);
                }
            }
        }
    }
}
