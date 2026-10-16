//! Regression coverage for transformed pcurves: periodic apex lifts and small valid scales.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::{
    curves2d::{Curve2D, Line2D},
    frame::Frame3,
    mat::Mat4,
    vec::{Point2, Point3, Vec2},
};
use remus_operations::{primitives::make_box, transform::transform_solid};
use remus_topology::{
    Topology,
    coedge::{CoedgeId, PeriodicWinding},
    explorer::solid_faces,
    face::FaceSurface,
    pcurve::PCurve,
};

#[test]
fn rotated_pointed_cone_retains_step_export() {
    use std::f64::consts::TAU;
    // A pointed cone's seam runs from the base to the apex, where the
    // projection always reports `u = 0`. Turning the chart by 0.3 moves
    // the seam pcurves' apex ends to `u = 0.3`, which the STEP writer
    // cannot certify as a lift.
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_cone(&mut topo, 2.0, 0.0, 3.0).unwrap();
    // `make_cone` puts its seam at `u = 3π/2`, which no apex end can
    // certify; turn it to `u = 0` (the axis points down -z) first, so the
    // seam pcurves export before the transform under test.
    remus_operations::transform::transform_solid(
        &mut topo,
        solid,
        &Mat4::rotation_z(1.5 * std::f64::consts::PI),
    )
    .unwrap();
    let cone_face = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_)))
        .unwrap();
    let surface = topo.face(cone_face).unwrap().surface().clone();
    let mut uses = Vec::new();
    for &loop_id in topo.face(cone_face).unwrap().boundary_loops() {
        uses.extend_from_slice(topo.face_loop(loop_id).unwrap().coedges());
    }
    let ends = |topo: &Topology, coedge: CoedgeId| {
        let data = topo.coedge(coedge).unwrap();
        let edge = topo.edge(data.edge()).unwrap();
        let (a, b) = if data.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        (
            topo.vertex(a).unwrap().point(),
            topo.vertex(b).unwrap().point(),
        )
    };
    let mut seams = Vec::new();
    for coedge in uses {
        let edge = topo.coedge(coedge).unwrap().edge();
        if !matches!(
            topo.edge(edge).unwrap().curve(),
            remus_topology::edge::EdgeCurve::Line
        ) {
            continue;
        }
        let (start, end) = ends(&topo, coedge);
        let (uv_start, uv_end) = (
            surface.project_point(start).unwrap(),
            surface.project_point(end).unwrap(),
        );
        let (v_start, v_end) = (uv_start.1, uv_end.1);
        // The base end's `u`; the other side of the seam one turn on.
        let base_u = if v_start > v_end {
            uv_start.0
        } else {
            uv_end.0
        };
        let u = if seams.is_empty() {
            base_u
        } else {
            base_u + TAU
        };
        let length = (v_end - v_start).abs();
        let line = Line2D::new(
            Point2::new(u, v_start),
            Vec2::new(0.0, (v_end - v_start).signum()),
        )
        .unwrap();
        let pcurve = PCurve::new(Curve2D::Line(line), 0.0, length);
        topo.set_coedge_pcurve(coedge, pcurve).unwrap();
        topo.set_coedge_periodic_winding(
            coedge,
            PeriodicWinding::new(if seams.is_empty() { 0 } else { 1 }, 0),
        )
        .unwrap();
        seams.push(coedge);
    }
    assert_eq!(seams.len(), 2, "pointed cone seam uses");

    remus_io::step::write_step(&topo, &[solid]).expect("STEP export before rotation");
    remus_operations::transform::transform_solid(&mut topo, solid, &Mat4::rotation_z(0.3)).unwrap();
    remus_topology::validation::validate_boundary_authority(&topo).unwrap();
    for coedge in seams {
        let data = topo.coedge(coedge).unwrap();
        assert!(
            data.pcurve().is_none(),
            "uncertifiable cone-apex hints must be removed"
        );
        assert_eq!(data.periodic_winding(), PeriodicWinding::ZERO);
    }
    let text = remus_io::step::write_step(&topo, &[solid]).expect("STEP export after rotation");
    let mut restored = Topology::new();
    let bodies = remus_io::step::read_step(&text, &mut restored).unwrap();
    assert_eq!(bodies.len(), 1);
    assert!(
        remus_operations::validate::validate_solid(&restored, bodies[0])
            .unwrap()
            .is_valid()
    );
    let volume = remus_operations::measure::solid_volume(&restored, bodies[0], 0.01).unwrap();
    assert!((volume - 4.0 * std::f64::consts::PI).abs() < 1e-6);
}

#[test]
fn large_box_normalizes_to_unit_box_with_or_without_pcurves() {
    for pcurves in [false, true] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 1e16, 1e16, 1e16).unwrap();
        if pcurves {
            let face = solid_faces(&topo, solid).unwrap()[0];
            let FaceSurface::Plane { normal, d } = *topo.face(face).unwrap().surface() else {
                panic!("plane")
            };
            let foot = normal * (d / normal.dot(normal));
            let frame =
                Frame3::from_normal(Point3::new(foot.x(), foot.y(), foot.z()), normal).unwrap();
            let wire = topo.face(face).unwrap().outer_wire();
            let oe = topo.wire(wire).unwrap().edges()[0];
            let e = topo.edge(oe.edge()).unwrap();
            let uv = |p: Point3| {
                let p = p - frame.origin;
                Point2::new(p.dot(frame.x), p.dot(frame.y))
            };
            let a = uv(topo.vertex(oe.oriented_start(e)).unwrap().point());
            let b = uv(topo.vertex(oe.oriented_end(e)).unwrap().point());
            let pc = PCurve::new(
                Curve2D::Line(Line2D::new(a, b - a).unwrap()),
                0.0,
                (b - a).length(),
            );
            topo.set_pcurve_oriented(oe.edge(), face, oe.is_forward(), pc)
                .unwrap();
        }
        remus_io::step::write_step(&topo, &[solid]).expect("valid source pcurve before scaling");
        transform_solid(&mut topo, solid, &Mat4::scale(1e-16, 1e-16, 1e-16))
            .unwrap_or_else(|e| panic!("pcurves={pcurves}: {e}"));
        remus_topology::validation::validate_boundary_authority(&topo).unwrap();
        assert!(
            remus_operations::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
        let bytes = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let mut restored = Topology::new();
        let bodies = remus_io::arena_io::deserialize_solids(&bytes, &mut restored).unwrap();
        let volume = remus_operations::measure::solid_volume(&restored, bodies[0], 0.01).unwrap();
        assert!((volume - 1.0).abs() < 1e-9);
        let text = remus_io::step::write_step(&restored, &bodies).unwrap();
        let mut round_trip = Topology::new();
        let bodies = remus_io::step::read_step(&text, &mut round_trip).unwrap();
        assert!(
            remus_operations::validate::validate_solid(&round_trip, bodies[0])
                .unwrap()
                .is_valid()
        );
    }
}
