//! Independent spherical-zone/cap area, flux and volume oracles.
#![allow(clippy::unwrap_used, clippy::panic)]
use super::*;
use remus_math::mat::Mat4;
use remus_topology::edge::Edge;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn direction(m: Mat4, v: Vec3) -> Vec3 {
    Vec3::new(
        m.0[0][0] * v.x() + m.0[0][1] * v.y() + m.0[0][2] * v.z(),
        m.0[1][0] * v.x() + m.0[1][1] * v.y() + m.0[1][2] * v.z(),
        m.0[2][0] * v.x() + m.0[2][1] * v.y() + m.0[2][2] * v.z(),
    )
}
fn latitude_ring(
    topo: &mut Topology,
    radius: f64,
    height: f64,
    count: usize,
    positive: bool,
    m: Mat4,
) -> WireId {
    let circle = Circle3D::with_axes(
        m.mul_point(Point3::new(0.0, 0.0, height)),
        direction(m, Vec3::new(0.0, 0.0, 1.0)),
        (radius * radius - height * height).sqrt(),
        direction(m, Vec3::new(1.0, 0.0, 0.0)),
        direction(m, Vec3::new(0.0, 1.0, 0.0)),
    )
    .unwrap();
    let vertices: Vec<_> = (0..count)
        .map(|i| {
            topo.add_vertex(Vertex::new(
                circle.evaluate(TAU * i as f64 / count as f64),
                1e-7,
            ))
        })
        .collect();
    let mut edges: Vec<_> = (0..count)
        .map(|i| {
            let mut e = Edge::new(
                vertices[i],
                vertices[(i + 1) % count],
                EdgeCurve::Circle(circle.clone()),
            );
            e.set_trim(Some((
                TAU * i as f64 / count as f64,
                TAU * (i + 1) as f64 / count as f64,
            )));
            OrientedEdge::new(topo.add_edge(e), positive)
        })
        .collect();
    if !positive {
        edges.reverse();
    }
    topo.add_wire(Wire::new(edges, true).unwrap())
}
fn fixture(
    radius: f64,
    height: f64,
    hole: Option<f64>,
    positive: bool,
    count: usize,
    placed: bool,
) -> (Topology, FaceId, Point3, Vec3) {
    let m = if placed {
        Mat4::translation(10.0 * radius, -7.0 * radius, 3.0 * radius)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    let mut topo = Topology::new();
    let outer = latitude_ring(&mut topo, radius, height, count, positive, m);
    let holes = hole
        .map(|h| latitude_ring(&mut topo, radius, h, 1, !positive, m))
        .into_iter()
        .collect();
    let center = m.mul_point(Point3::new(0.0, 0.0, 0.0));
    let axis = direction(m, Vec3::new(0.0, 0.0, 1.0));
    let face = topo.add_face(Face::new(
        outer,
        holes,
        FaceSurface::Sphere(SphericalSurface::with_axis(center, radius, axis).unwrap()),
    ));
    (topo, face, center, axis)
}
fn resolved(topo: &Topology, face: FaceId) -> Option<(f64, f64)> {
    let f = topo.face(face).unwrap();
    let FaceSurface::Sphere(s) = f.surface() else {
        panic!("fixture sphere")
    };
    domain(topo, f, s).unwrap()
}

// Independent three-support description of the northern hemisphere with the
// positive x/y octant removed: two quarter meridians and a 3/4 equator arc.
fn three_support_fixture() -> (Topology, FaceId) {
    let (mut topo, face, _, _) = fixture(1.0, 0.0, None, true, 64, false);
    let old = topo.face(face).unwrap().outer_wire();
    let equator = topo.wire(old).unwrap().edges().to_vec();
    let x = topo.edge(equator[0].edge()).unwrap().start();
    let y = topo.edge(equator[16].edge()).unwrap().start();
    let north = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 1.0), 1e-7));
    let origin = Point3::new(0.0, 0.0, 0.0);
    let mut uses = Vec::new();
    for (start, end, u, v, normal) in [
        (
            x,
            north,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, -1.0, 0.0),
        ),
        (
            north,
            y,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
        ),
    ] {
        let circle = Circle3D::with_axes(origin, normal, 1.0, u, v).unwrap();
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, FRAC_PI_2)));
        uses.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    uses.extend_from_slice(&equator[16..]);
    let wire = topo.add_wire(Wire::new(uses, true).unwrap());
    topo.set_face_boundary_wires(face, wire, vec![]).unwrap();
    (topo, face)
}

fn prepared(topo: &Topology, face: FaceId) -> Option<PartitionSamples> {
    let face = topo.face(face).unwrap();
    let FaceSurface::Sphere(sphere) = face.surface() else {
        panic!("fixture sphere")
    };
    partition_samples(topo, face, sphere).unwrap()
}

#[test]
fn coalesced_major_equator_keeps_three_support_domain_and_closed_form_integrals() {
    let (mut topo, face) = three_support_fixture();
    assert!(super::super::check_trim_point_budget(&topo, topo.face(face).unwrap()).is_err());
    assert!(resolved(&topo, face).is_none());
    let samples = prepared(&topo, face).unwrap();
    assert_eq!(samples.outer.len(), 3 * super::super::TRIM_SAMPLES);
    assert!(samples.holes.is_empty());
    for point in [Point3::new(-1.0, 0.0, 0.0), Point3::new(0.0, -1.0, 0.0)] {
        assert!(samples.outer.iter().any(|p| (*p - point).length() < 0.02));
    }
    let result =
        super::super::integrate_face_fixed_about(&topo, face, 5, Point3::new(0.0, 0.0, 0.0))
            .unwrap();
    assert!(
        (result.area - 1.5 * PI).abs() < 0.005 * 1.5 * PI,
        "area={}",
        result.area
    );
    assert!(
        (result.volume - 0.5 * PI).abs() < 0.005 * 0.5 * PI,
        "volume={}",
        result.volume
    );

    // A descending native domain paired with a reversed coedge is the same
    // physical traversal; rotating the wire start also retains the cyclic run.
    let old = topo.face(face).unwrap().outer_wire();
    let mut uses = topo.wire(old).unwrap().edges().to_vec();
    for use_ in &mut uses {
        let e = topo.edge(use_.edge()).unwrap();
        let (start, end, (a, b)) = (e.start(), e.end(), e.trim().unwrap());
        let e = topo.edge_mut(use_.edge()).unwrap();
        e.set_start(end);
        e.set_end(start);
        e.set_trim(Some((b, a)));
        *use_ = OrientedEdge::new(use_.edge(), false);
    }
    uses.rotate_left(13);
    let replacement = topo.add_wire(Wire::new(uses, true).unwrap());
    topo.set_face_boundary_wires(face, replacement, vec![])
        .unwrap();
    let reversed = prepared(&topo, face).unwrap();
    assert_eq!(reversed.outer, samples.outer);
}

#[test]
fn gapped_retraced_multiturn_foreign_and_unknown_partitions_keep_budget_refusal() {
    for mutation in 0..6 {
        let (mut topo, face) = three_support_fixture();
        let wire = topo.face(face).unwrap().outer_wire();
        let id = topo.wire(wire).unwrap().edges()[10].edge();
        let e = topo.edge(id).unwrap();
        let (start, (a, b)) = (e.start(), e.trim().unwrap());
        let EdgeCurve::Circle(circle) = e.curve() else {
            panic!("fixture circle")
        };
        let circle = circle.clone();
        match mutation {
            0 => topo.edge_mut(id).unwrap().set_trim(Some((a + 1e-4, b))),
            1 => topo.edge_mut(id).unwrap().set_trim(Some((b, a))),
            2 => topo.edge_mut(id).unwrap().set_trim(Some((a, b + TAU))),
            3 => {
                let foreign = Circle3D::with_axes(
                    circle.center() + Vec3::new(0.0, 0.0, 1e-4),
                    circle.normal(),
                    circle.radius(),
                    circle.u_axis(),
                    circle.v_axis(),
                )
                .unwrap();
                topo.edge_mut(id)
                    .unwrap()
                    .set_curve(EdgeCurve::Circle(foreign));
            }
            4 => {
                let duplicate =
                    topo.add_vertex(Vertex::new(topo.vertex(start).unwrap().point(), 1e-7));
                topo.edge_mut(id).unwrap().set_start(duplicate);
            }
            5 => topo.edge_mut(id).unwrap().set_curve(EdgeCurve::Line),
            _ => unreachable!(),
        }
        assert!(prepared(&topo, face).is_none(), "mutation={mutation}");
        assert!(
            super::super::integrate_face_fixed_about(&topo, face, 5, Point3::new(0.0, 0.0, 0.0))
                .is_err(),
            "mutation={mutation}"
        );
    }
}

#[test]
fn native_circle_caps_and_overbudget_band_match_closed_form_properties() {
    for radius in [0.001, 1.0, 10.0, 1000.0] {
        for placed in [false, true] {
            for positive in [false, true] {
                for with_hole in [false, true] {
                    let sign = if positive { 1.0 } else { -1.0 };
                    let hole = with_hole.then_some(sign * radius / 2.0);
                    let (topo, face, center, axis) =
                        fixture(radius, 0.0, hole, positive, 32, placed);
                    assert!(resolved(&topo, face).is_some());
                    if with_hole {
                        assert!(
                            super::super::check_trim_point_budget(&topo, topo.face(face).unwrap())
                                .is_err()
                        );
                    }
                    let (p, flux) =
                        super::super::integrate_face_fixed_flux_about(&topo, face, 8, center)
                            .unwrap();
                    let area = if with_hole {
                        PI * radius * radius
                    } else {
                        2.0 * PI * radius * radius
                    };
                    let volume = radius * area / 3.0;
                    let flux = flux.unwrap();
                    let expected_flux =
                        axis * (sign * PI * radius * radius * if with_hole { 0.25 } else { 1.0 });
                    assert!(
                        (p.area - area).abs() <= area * 2e-12,
                        "area {} vs {area}",
                        p.area
                    );
                    assert!(
                        (p.volume - volume).abs() <= volume * 2e-12,
                        "V {} vs {volume}",
                        p.volume
                    );
                    assert!((flux - expected_flux).length() <= area * 2e-12);
                }
            }
        }
    }
}

#[test]
fn native_descending_trim_with_reversed_coedge_preserves_cap_domain() {
    let (mut topo, face, center, _) = fixture(10.0, 0.0, Some(5.0), true, 32, true);
    let expected = resolved(&topo, face).unwrap();
    let f = topo.face(face).unwrap();
    let wires: Vec<_> = std::iter::once(f.outer_wire())
        .chain(f.inner_wires().iter().copied())
        .collect();
    let mut replacement_wires = Vec::new();
    for wire in wires {
        let edges = topo.wire(wire).unwrap().edges().to_vec();
        let mut reversed = Vec::new();
        for use_ in edges {
            let e = topo.edge(use_.edge()).unwrap();
            let (a, b, trim) = (e.start(), e.end(), e.trim().unwrap());
            let e = topo.edge_mut(use_.edge()).unwrap();
            e.set_start(b);
            e.set_end(a);
            e.set_trim(Some((trim.1, trim.0)));
            reversed.push(OrientedEdge::new(use_.edge(), !use_.is_forward()));
        }
        replacement_wires.push(topo.add_wire(Wire::new(reversed, true).unwrap()));
    }
    topo.set_face_boundary_wires(face, replacement_wires[0], replacement_wires[1..].to_vec())
        .unwrap();
    assert_eq!(resolved(&topo, face).unwrap(), expected);
    let a = super::super::integrate_face_fixed_about(&topo, face, 8, center).unwrap();
    topo.face_mut(face).unwrap().set_reversed(true);
    let b = super::super::integrate_face_fixed_about(&topo, face, 8, center).unwrap();
    assert_eq!(a.area.to_bits(), b.area.to_bits());
    assert_eq!(a.volume.to_bits(), (-b.volume).to_bits());
}

#[test]
fn broken_native_trims_carriers_and_vertex_chains_decline() {
    for mutation in 0..5 {
        let (mut topo, face, _, _) = fixture(10.0, 0.0, Some(5.0), true, 32, false);
        let wire = topo.face(face).unwrap().outer_wire();
        let id = topo.wire(wire).unwrap().edges()[0].edge();
        let e = topo.edge(id).unwrap();
        let (start, end, trim) = (e.start(), e.end(), e.trim().unwrap());
        let EdgeCurve::Circle(c) = e.curve() else {
            panic!("fixture circle")
        };
        let c = c.clone();
        match mutation {
            0 => topo.edge_mut(id).unwrap().set_trim(None),
            1 => topo
                .edge_mut(id)
                .unwrap()
                .set_trim(Some((trim.0, trim.1 + TAU))),
            2 => {
                let duplicate =
                    topo.add_vertex(Vertex::new(topo.vertex(start).unwrap().point(), 1e-7));
                topo.edge_mut(id).unwrap().set_start(duplicate);
            }
            3 => topo.edge_mut(id).unwrap().set_curve(EdgeCurve::Line),
            4 => {
                let carrier = Circle3D::with_axes(
                    c.center() + Vec3::new(1e-4, 0.0, 0.0),
                    c.normal(),
                    c.radius(),
                    c.u_axis(),
                    c.v_axis(),
                )
                .unwrap();
                topo.edge_mut(id)
                    .unwrap()
                    .set_curve(EdgeCurve::Circle(carrier));
            }
            _ => unreachable!(),
        }
        let _ = end;
        assert!(resolved(&topo, face).is_none(), "mutation={mutation}");
    }
}

#[test]
fn wrong_hole_orientation_or_side_and_degenerate_tiny_frames_decline() {
    let (topo, face, _, _) = fixture(10.0, 0.0, Some(-5.0), true, 32, false);
    assert!(resolved(&topo, face).is_none());
    let (mut topo, face, _, _) = fixture(10.0, 0.0, Some(5.0), true, 32, false);
    let hole = topo.face(face).unwrap().inner_wires()[0];
    let opposite = topo
        .wire(hole)
        .unwrap()
        .edges()
        .iter()
        .map(|use_| OrientedEdge::new(use_.edge(), !use_.is_forward()))
        .collect();
    let opposite = topo.add_wire(Wire::new(opposite, true).unwrap());
    let outer = topo.face(face).unwrap().outer_wire();
    topo.set_face_boundary_wires(face, outer, vec![opposite])
        .unwrap();
    assert!(resolved(&topo, face).is_none());
    let (mut topo, face, _, _) = fixture(1e-12, 0.0, None, true, 1, false);
    let edge = topo
        .wire(topo.face(face).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();
    let carrier = Circle3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1e-12,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
    )
    .unwrap();
    topo.edge_mut(edge)
        .unwrap()
        .set_curve(EdgeCurve::Circle(carrier));
    assert!(resolved(&topo, face).is_none());
}

#[test]
fn exact_ring_path_keeps_the_existing_aggregate_work_ceiling() {
    let (topo, face, _, _) = fixture(
        1.0,
        0.0,
        Some(0.5),
        true,
        super::super::MAX_TRIM_POINTS,
        false,
    );
    assert!(resolved(&topo, face).is_none());
}

#[test]
fn tiny_spheres_still_require_one_native_turn_and_parameter_continuity() {
    for to in [TAU / 4.0, 3.0 * TAU] {
        let (mut topo, face, _, _) = fixture(1e-15, 0.0, None, true, 1, false);
        let edge = topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()[0]
            .edge();
        topo.edge_mut(edge).unwrap().set_trim(Some((0.0, to)));
        assert!(resolved(&topo, face).is_none());
    }
    let (mut topo, face, _, _) = fixture(1e-15, 0.0, None, true, 4, false);
    let edge = topo
        .wire(topo.face(face).unwrap().outer_wire())
        .unwrap()
        .edges()[1]
        .edge();
    let (a, b) = topo.edge(edge).unwrap().trim().unwrap();
    topo.edge_mut(edge)
        .unwrap()
        .set_trim(Some((a + PI / 4.0, b + PI / 4.0)));
    assert!(resolved(&topo, face).is_none());
}

#[test]
fn nonzero_latitude_selects_the_directed_small_or_large_cap() {
    for height in [-5.0, 5.0] {
        for positive in [false, true] {
            for placed in [false, true] {
                let (topo, face, center, axis) = fixture(10.0, height, None, positive, 1, placed);
                assert!(resolved(&topo, face).is_some());
                let (p, flux) =
                    super::super::integrate_face_fixed_flux_about(&topo, face, 8, center).unwrap();
                let sign = if positive { 1.0 } else { -1.0 };
                let area = 20.0 * PI * (10.0 - sign * height);
                let expected_flux = axis * (sign * PI * (100.0 - height * height));
                assert!((p.area - area).abs() < 1e-8);
                assert!((p.volume - 10.0 * area / 3.0).abs() < 1e-8);
                assert!((flux.unwrap() - expected_flux).length() < 1e-8);
            }
        }
    }
}
