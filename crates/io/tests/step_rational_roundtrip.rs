//! Rational NURBS STEP round trips.
//!
//! A STEP export must retain projective weights. Dropping them keeps the same
//! control polygon but changes conics and surfaces of revolution into different
//! geometry, so these tests compare both the weights and sampled geometry after
//! a write/read cycle.

#![allow(clippy::unwrap_used, clippy::panic)]

use remus_io::step::{read_step, write_step};
use remus_math::mat::Mat4;
use remus_math::nurbs::{NurbsCurve, NurbsSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn one_edge_solid(topo: &mut Topology, curve: NurbsCurve) -> SolidId {
    let (start, end) = curve.domain();
    let v0 = topo.add_vertex(Vertex::new(curve.evaluate(start), 1e-7));
    let v1 = topo.add_vertex(Vertex::new(curve.evaluate(end), 1e-7));
    let mut edge = Edge::new(v0, v1, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some((start, end)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

fn one_face_solid(topo: &mut Topology, surface: NurbsSurface) -> SolidId {
    let face = remus_topology::builder::make_nurbs_face(topo, surface, 1e-7).unwrap();
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

fn nurbs_curves(topo: &Topology, solid: SolidId) -> Vec<NurbsCurve> {
    let mut curves = Vec::new();
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id).unwrap().edges() {
                if let EdgeCurve::NurbsCurve(curve) = topo.edge(oriented.edge()).unwrap().curve() {
                    curves.push(curve.clone());
                }
            }
        }
    }
    curves
}

fn nurbs_surfaces(topo: &Topology, solid: SolidId) -> Vec<NurbsSurface> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|face_id| match topo.face(face_id).unwrap().surface() {
            FaceSurface::Nurbs(surface) => Some(surface.clone()),
            _ => None,
        })
        .collect()
}

fn assert_weights_close(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len());
    for (&actual, &expected) in actual.iter().zip(expected) {
        assert!(
            (actual - expected).abs() < 1e-14,
            "weight {actual} differs from {expected}"
        );
    }
}

fn assert_weights_exact(actual: &[Vec<f64>], expected: &[Vec<f64>]) {
    assert_eq!(actual.len(), expected.len());
    for (actual_row, expected_row) in actual.iter().zip(expected) {
        assert_eq!(actual_row.len(), expected_row.len());
        for (&actual, &expected) in actual_row.iter().zip(expected_row) {
            assert_eq!(
                actual.to_bits(),
                expected.to_bits(),
                "weight {actual} does not exactly match {expected}"
            );
        }
    }
}

fn assert_point_exact(actual: Point3, expected: Point3) {
    assert_eq!(
        [
            actual.x().to_bits(),
            actual.y().to_bits(),
            actual.z().to_bits(),
        ],
        [
            expected.x().to_bits(),
            expected.y().to_bits(),
            expected.z().to_bits(),
        ],
        "point {actual:?} does not exactly match {expected:?}"
    );
}

#[test]
fn rational_curve_edge_weights_survive_step_round_trip() {
    let source = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
    )
    .unwrap();
    let mut topo = Topology::new();
    let solid = one_edge_solid(&mut topo, source.clone());

    let step = write_step(&topo, &[solid]).unwrap();
    assert!(step.contains("RATIONAL_B_SPLINE_CURVE"));

    let mut back = Topology::new();
    let solids = read_step(&step, &mut back).unwrap();
    let curves = nurbs_curves(&back, solids[0]);
    assert_eq!(curves.len(), 1);
    let round_tripped = &curves[0];
    assert_weights_close(round_tripped.weights(), source.weights());

    for i in 0..=16 {
        let t = f64::from(i) / 16.0;
        assert!(
            (round_tripped.evaluate(t) - source.evaluate(t)).length() < 1e-12,
            "curve changed at parameter {t}"
        );
    }
}

#[test]
fn near_unity_surface_weight_survives_step_round_trip_exactly() {
    let source = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.5)],
            vec![Point3::new(3.0, 0.0, 1.0), Point3::new(3.0, 2.0, 2.0)],
        ],
        vec![vec![1.0, 1.000_000_01], vec![1.0, 1.0]],
    )
    .unwrap();
    let mut topo = Topology::new();
    let solid = one_face_solid(&mut topo, source.clone());

    let step = write_step(&topo, &[solid]).unwrap();
    assert!(step.contains("RATIONAL_B_SPLINE_SURFACE"));
    assert!(step.contains("1.00000000999999994E0"));

    let mut back = Topology::new();
    let solids = read_step(&step, &mut back).unwrap();
    let surfaces = nurbs_surfaces(&back, solids[0]);
    assert_eq!(surfaces.len(), 1);
    let round_tripped = &surfaces[0];

    assert_weights_exact(round_tripped.weights(), source.weights());
    for (u, v) in [(0.25, 0.75), (0.5, 0.5), (0.875, 0.125)] {
        assert_point_exact(round_tripped.evaluate(u, v), source.evaluate(u, v));
    }
}

#[test]
fn rational_torus_surface_weights_survive_step_round_trip() {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_torus(&mut topo, 5.0, 1.5, 24).unwrap();
    remus_operations::transform::transform_solid(&mut topo, solid, &Mat4::scale(1.25, 1.0, 1.0))
        .unwrap();

    let source_surfaces = nurbs_surfaces(&topo, solid);
    assert!(!source_surfaces.is_empty());
    assert!(source_surfaces.iter().all(NurbsSurface::is_rational));

    let step = write_step(&topo, &[solid]).unwrap();
    assert!(step.contains("RATIONAL_B_SPLINE_SURFACE"));

    let mut back = Topology::new();
    let solids = read_step(&step, &mut back).unwrap();
    let surfaces = nurbs_surfaces(&back, solids[0]);
    assert_eq!(surfaces.len(), source_surfaces.len());
    for (round_tripped, source) in surfaces.iter().zip(&source_surfaces) {
        assert_eq!(round_tripped.degree_u(), source.degree_u());
        assert_eq!(round_tripped.degree_v(), source.degree_v());
        assert_eq!(round_tripped.weights().len(), source.weights().len());
        for (actual, expected) in round_tripped.weights().iter().zip(source.weights()) {
            assert_weights_close(actual, expected);
        }

        let (u0, u1) = source.domain_u();
        let (v0, v1) = source.domain_v();
        for ui in 0..=8 {
            for vi in 0..=8 {
                let u = (u1 - u0).mul_add(f64::from(ui) / 8.0, u0);
                let v = (v1 - v0).mul_add(f64::from(vi) / 8.0, v0);
                assert!(
                    (round_tripped.evaluate(u, v) - source.evaluate(u, v)).length() < 1e-11,
                    "surface changed at ({u}, {v})"
                );
            }
        }
    }
}
