//! Rotated circular caps can retain an ellipse carrier without losing exact moments.
#![allow(clippy::unwrap_used)]
use remus_check::properties::face_integrator::integrate_face;
use remus_math::{
    curves::Ellipse3D,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::{Edge, EdgeCurve},
    face::{Face, FaceSurface},
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

#[test]
fn planar_ellipse_area_and_volume_do_not_use_a_chord_polygon() {
    for scale in [0.1, 1.0, 10.0] {
        for ratio in [1.0, 0.4] {
            let mut topo = Topology::new();
            let normal = Vec3::new(0.37_f64.sin(), 0.0, 0.37_f64.cos());
            let center = Point3::new(17.0 * scale, -23.0 * scale, 31.0 * scale);
            let a = 6.0 * scale;
            let b = a * ratio;
            let ellipse = Ellipse3D::new(center, normal, a, b).unwrap();
            let seam = topo.add_vertex(Vertex::new(ellipse.evaluate(0.0), 1e-7));
            let mut edge = Edge::new(seam, seam, EdgeCurve::Ellipse(ellipse));
            edge.set_trim(Some((0.0, std::f64::consts::TAU)));
            let edge = topo.add_edge(edge);
            let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
            let d = normal.dot(Vec3::new(center.x(), center.y(), center.z()));
            let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }));
            let measured = integrate_face(&topo, face, 8).unwrap();
            let area = std::f64::consts::PI * a * b;
            assert!((measured.area - area).abs() <= area * 1e-12);
            assert!((measured.volume - area * d / 3.0).abs() <= (area * d).abs() * 1e-12);
        }
    }
}
