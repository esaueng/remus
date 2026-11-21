//! The outline certificate never accepts a face the mesh probe would decline,
//! declines every face it cannot prove, and spares the probe its meshes.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::planar_outline::outline_area_is_certified;
use super::*;
use remus_math::mat::Mat4;
use remus_topology::builder::{make_face_from_wire, make_polygon_wire};

/// What the probe and the certificate say about one face at one deflection.
struct Verdicts {
    certified: bool,
    meshed: bool,
}

/// Both verdicts for `face`, exactly as [`planar_face_area_is_consistent`]
/// forms its probe and budget; `None` for a face the exact path declines
/// before probing.
fn verdicts(topo: &Topology, face: FaceId, deflection: f64) -> Option<Verdicts> {
    let exact = planar_face_signed_volume(topo, face).ok()??;
    if !exact.exact_boundary {
        return None;
    }
    let probe = if exact.arc_length > 0.0 {
        deflection.max(exact.arc_length * 1e-3)
    } else {
        deflection
    };
    let budget =
        16.0 * (2.0 / 3.0) * exact.arc_length * probe.max(0.0) + 1e-9 * exact.area.abs().max(1.0);
    Some(Verdicts {
        certified: outline_area_is_certified(topo, face, exact.area, budget),
        meshed: face_mesh_area_agrees(topo, face, exact.area, probe, budget),
    })
}

fn top_face(topo: &Topology, solid: SolidId) -> FaceId {
    *topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .faces()
        .last()
        .unwrap()
}

fn prism(topo: &mut Topology, points: &[Point3], height: f64) -> SolidId {
    let wire = make_polygon_wire(topo, points, 1e-7).unwrap();
    let face = make_face_from_wire(topo, wire).unwrap();
    crate::extrude::extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), height).unwrap()
}

fn planar_face(topo: &mut Topology, points: &[Point3]) -> FaceId {
    let wire = make_polygon_wire(topo, points, 1e-7).unwrap();
    make_face_from_wire(topo, wire).unwrap()
}

/// Solids whose planar faces span the certificate's domain and its edges:
/// boxes at several scales and placements, shelled and chamfered bodies,
/// boolean results with collinear-vertex outlines, and polygon prisms.
fn corpus(topo: &mut Topology) -> Vec<SolidId> {
    let mut solids = Vec::new();
    for (size, offset) in [
        (10.0, 0.0),
        (1e-3, 0.0),
        (2e-3, 13.0),
        (1e3, 0.0),
        (7.3, 1e4),
        (0.4, 1e5),
    ] {
        let b = crate::primitives::make_box(topo, size, size * 0.7, size * 1.3).unwrap();
        crate::transform::transform_solid(topo, b, &Mat4::translation(offset, -offset, offset))
            .unwrap();
        solids.push(b);
    }
    let bin = crate::primitives::make_box(topo, 42.0, 42.0, 21.0).unwrap();
    let open = top_face(topo, bin);
    let shelled = crate::shell_op::shell(topo, bin, 1.6, &[open]).unwrap();
    solids.push(shelled);
    let edges = remus_topology::explorer::solid_edges(topo, shelled).unwrap();
    solids.push(crate::chamfer::chamfer(topo, shelled, &edges[..4], 0.8).unwrap());
    let block = crate::primitives::make_box(topo, 20.0, 20.0, 20.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(topo, block).unwrap();
    solids.push(crate::chamfer::chamfer(topo, block, &edges, 1.0).unwrap());
    let a = crate::primitives::make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let b = crate::primitives::make_box(topo, 4.0, 4.0, 20.0).unwrap();
    crate::transform::transform_solid(topo, b, &Mat4::translation(3.0, 3.0, -5.0)).unwrap();
    solids.push(crate::boolean::boolean(topo, crate::boolean::BooleanOp::Cut, a, b).unwrap());
    let c = crate::primitives::make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let d = crate::primitives::make_box(topo, 10.0, 6.0, 4.0).unwrap();
    crate::transform::transform_solid(topo, d, &Mat4::translation(5.0, 2.0, 3.0)).unwrap();
    solids.push(crate::boolean::boolean(topo, crate::boolean::BooleanOp::Fuse, c, d).unwrap());
    let l_outline = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(30.5, 0.0, 0.0),
        Point3::new(30.5, 12.25, 0.0),
        Point3::new(10.1, 12.25, 0.0),
        Point3::new(10.1, 40.0, 0.0),
        Point3::new(0.0, 40.0, 0.0),
    ];
    solids.push(prism(topo, &l_outline, 3.3));
    #[allow(clippy::cast_precision_loss)]
    for (n, radius) in [(5, 9.0), (12, 4.5), (16, 30.0), (24, 2.0)] {
        let star: Vec<Point3> = (0..n)
            .map(|i| {
                let t = std::f64::consts::TAU * i as f64 / n as f64;
                let r = if i % 2 == 0 { radius } else { radius * 0.45 };
                Point3::new(r * t.cos() + 1.7, r * t.sin() - 0.6, 0.0)
            })
            .collect();
        solids.push(prism(topo, &star, 2.5));
    }
    solids
}

#[test]
fn certified_outlines_pass_the_mesh_probe() {
    let mut topo = Topology::new();
    let solids = corpus(&mut topo);
    let mut certified = 0_usize;
    let mut declined = 0_usize;
    for solid in solids {
        for face in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
            for deflection in [0.1, 1e-3, 1e-6] {
                let Some(v) = verdicts(&topo, face, deflection) else {
                    continue;
                };
                if v.certified {
                    certified += 1;
                    assert!(v.meshed, "certified face {} fails the probe", face.index());
                } else {
                    declined += 1;
                }
            }
        }
    }
    // The corpus exercises both outcomes: the axis-aligned outlines certify,
    // tilted faces, holed rims, far-placed bodies and the 24-gon do not.
    assert!(certified > 200, "certified {certified}");
    assert!(declined > 200, "declined {declined}");
}

#[test]
fn outlines_outside_the_proof_are_left_to_the_probe() {
    let mut topo = Topology::new();
    let p = Point3::new;
    let certified =
        |topo: &Topology, face: FaceId| verdicts(topo, face, 1e-3).is_some_and(|v| v.certified);
    // Each refusal below other than `tilted` must come from the condition it
    // names, not from the plane fit leaving the normal off an axis.
    let on_axis = |topo: &Topology, face: FaceId| {
        let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface() else {
            return false;
        };
        normal.x() == 0.0 && normal.y() == 0.0 && normal.z() != 0.0
    };

    let square = planar_face(
        &mut topo,
        &[
            p(0.0, 0.0, 2.0),
            p(3.0, 0.0, 2.0),
            p(3.0, 3.0, 2.0),
            p(0.0, 3.0, 2.0),
        ],
    );
    assert!(
        on_axis(&topo, square) && certified(&topo, square),
        "the reference square certifies"
    );

    // Self-crossing (bow tie).
    let bow_tie = planar_face(
        &mut topo,
        &[
            p(0.0, 0.0, 2.0),
            p(3.0, 3.0, 2.0),
            p(3.0, 0.0, 2.0),
            p(0.0, 3.0, 2.0),
        ],
    );
    assert!(on_axis(&topo, bow_tie) && !certified(&topo, bow_tie));
    // A vertex almost on a non-adjacent edge (a pinch the CDT would snap).
    let pinch = planar_face(
        &mut topo,
        &[
            p(0.0, 0.0, 2.0),
            p(4.0, 0.0, 2.0),
            p(4.0, 2.0, 2.0),
            p(2.0, 1e-7, 2.0),
            p(0.0, 2.0, 2.0),
        ],
    );
    assert!(on_axis(&topo, pinch) && !certified(&topo, pinch));
    // An edge shorter than the separation.
    let sliver_edge = planar_face(
        &mut topo,
        &[
            p(0.0, 0.0, 2.0),
            p(3.0, 0.0, 2.0),
            p(3.0, 3.0, 2.0),
            p(3.0 - 4e-7, 3.0, 2.0),
            p(0.0, 3.0, 2.0),
        ],
    );
    assert!(on_axis(&topo, sliver_edge) && !certified(&topo, sliver_edge));
    // Not exactly in a coordinate plane.
    let tilted = planar_face(
        &mut topo,
        &[
            p(0.0, 0.0, 0.0),
            p(3.0, 0.0, 0.3),
            p(3.0, 3.0, 0.3),
            p(0.0, 3.0, 0.0),
        ],
    );
    assert!(!on_axis(&topo, tilted) && !certified(&topo, tilted));
    // Small and far from the origin: Green's round-off outgrows the budget.
    let far = planar_face(
        &mut topo,
        &[
            p(2e5, 2e5, 1.0),
            p(2e5 + 0.5, 2e5, 1.0),
            p(2e5 + 0.5, 2e5 + 0.5, 1.0),
            p(2e5, 2e5 + 0.5, 1.0),
        ],
    );
    assert!(on_axis(&topo, far) && !certified(&topo, far));
    // More vertices than the certificate checks.
    #[allow(clippy::cast_precision_loss)]
    let many: Vec<Point3> = (0..17)
        .map(|i| {
            let t = std::f64::consts::TAU * f64::from(i) / 17.0;
            p(5.0 * t.cos(), 5.0 * t.sin(), 0.0)
        })
        .collect();
    let many = planar_face(&mut topo, &many);
    assert!(on_axis(&topo, many) && !certified(&topo, many));
    // Holed (the shelled bin's rim) and arc-bounded (a cylinder's cap) faces.
    let bin = crate::primitives::make_box(&mut topo, 42.0, 42.0, 21.0).unwrap();
    let open = top_face(&topo, bin);
    let shelled = crate::shell_op::shell(&mut topo, bin, 1.6, &[open]).unwrap();
    let rim = remus_topology::explorer::solid_faces(&topo, shelled)
        .unwrap()
        .into_iter()
        .find(|&f| !topo.face(f).unwrap().inner_wires().is_empty())
        .unwrap();
    assert!(!certified(&topo, rim));
    let cylinder = crate::primitives::make_cylinder(&mut topo, 2.0, 5.0).unwrap();
    for face in remus_topology::explorer::solid_faces(&topo, cylinder).unwrap() {
        if matches!(
            topo.face(face).unwrap().surface(),
            FaceSurface::Plane { .. }
        ) {
            assert!(!certified(&topo, face), "arc-bounded cap");
        }
    }
}

#[test]
fn shelled_bin_volume_meshes_only_its_holed_rim() {
    let mut topo = Topology::new();
    let bin = crate::primitives::make_box(&mut topo, 42.0, 42.0, 21.0).unwrap();
    let open = top_face(&topo, bin);
    let shelled = crate::shell_op::shell(&mut topo, bin, 1.6, &[open]).unwrap();
    let faces = remus_topology::explorer::solid_faces(&topo, shelled).unwrap();
    assert_eq!(faces.len(), 11);

    PROBE_MESHES.with(|count| count.set(0));
    let volume = solid_volume(&topo, shelled, 0.1).unwrap();
    // Ten axis-aligned rectangles certify; the rim, holed, is still meshed.
    assert_eq!(PROBE_MESHES.with(std::cell::Cell::get), 1);
    // The open side wall leaves a 40.4 × 38.8 × 17.8 cavity.
    let expected = 42.0 * 42.0 * 21.0 - 40.4 * 38.8 * 17.8;
    assert!(
        (volume - expected).abs() <= 1e-9 * expected,
        "{volume} vs {expected}"
    );
}

fn plate_cap(topo: &mut Topology, locations: &[(f64, f64, f64)], offset: f64) -> FaceId {
    use remus_math::curves::Circle3D;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    let face = planar_face(
        topo,
        &[(0., 0.), (40., 0.), (40., 30.), (0., 30.)]
            .map(|(x, y)| Point3::new(x + offset, y + offset, 2.)),
    );
    let mut holes = Vec::new();
    for (i, &(x, y, r)) in locations.iter().enumerate() {
        let circle = Circle3D::new(
            Point3::new(x + offset, y + offset, 2.),
            Vec3::new(0., 0., if i % 2 == 0 { 1. } else { -1. }),
            r,
        )
        .unwrap();
        let start = [0.0, 0.125, -0.25, 1.5][i % 4];
        let seam = circle.evaluate(start);
        let v = topo.add_vertex(Vertex::new(seam, 1e-7));
        let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        edge.set_trim(Some((start, start + std::f64::consts::TAU)));
        let edge = topo.add_edge(edge);
        let wire =
            topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, i % 3 != 0)], true).unwrap());
        holes.push(wire);
    }
    topo.set_face_boundary_wires(face, topo.face(face).unwrap().outer_wire(), holes)
        .unwrap();
    face
}

#[test]
fn perforated_cap_compares_the_chorded_region_to_the_mesh() {
    let mut topo = Topology::new();
    for offset in [0., -1000., 9100.] {
        let face = plate_cap(
            &mut topo,
            &[(5., 6., 2.), (20., 15., 3.), (12., 24., 2.5)],
            offset,
        );
        let exact = planar_face_signed_volume(&topo, face).unwrap().unwrap();
        for deflection in [0.001_f64, 0.1, 1.0] {
            let probe = deflection.max(exact.arc_length * 1e-3);
            let budget = 16. * (2. / 3.) * exact.arc_length * probe + 1e-9 * exact.area.max(1.);
            assert!(perforated_plate::sampled_area_agrees(
                &topo, face, exact.area, probe, budget
            ));
            assert!(face_mesh_area_agrees(
                &topo, face, exact.area, probe, budget
            ));
            assert!(!perforated_plate::sampled_area_agrees(
                &topo,
                face,
                exact.area + 2. * budget,
                probe,
                budget
            ));
        }
    }
    for holes in [
        [(5., 6., 2.), (9., 6., 2.)],  // tangent
        [(5., 6., 2.), (5., 6., 1.)],  // nested
        [(2., 6., 2.), (15., 6., 2.)], // touches the rim
    ] {
        let face = plate_cap(&mut topo, &holes, 0.);
        assert!(!perforated_plate::sampled_area_agrees(
            &topo, face, 1000., 0.1, 1e5
        ));
    }
}

#[test]
fn many_separate_circle_rims_match_the_plate_mesh() {
    let mut topo = Topology::new();
    for count in [8, 32, 96] {
        let holes = (0..count)
            .map(|i| {
                let x = 2.0 + f64::from(i % 12) * 3.1;
                let y = 2.0 + f64::from(i / 12) * 3.2;
                let r = 0.45 + 0.15 * f64::from(i % 5);
                (x, y, r)
            })
            .collect::<Vec<_>>();
        let face = plate_cap(&mut topo, &holes, -75.5);
        let exact = planar_face_signed_volume(&topo, face).unwrap().unwrap();
        for probe in [0.08_f64.max(exact.arc_length * 1e-3), 0.7] {
            let budget = 16. * (2. / 3.) * exact.arc_length * probe + 1e-9 * exact.area;
            assert!(perforated_plate::sampled_area_agrees(
                &topo, face, exact.area, probe, budget
            ));
            assert!(face_mesh_area_agrees(
                &topo, face, exact.area, probe, budget
            ));
        }
    }
}

#[test]
fn volume_of_a_drilled_plate_checks_its_caps_without_meshing_them_again() {
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_box, make_cylinder};
    use crate::transform::transform_solid;

    let mut topo = Topology::new();
    let mut plate = make_box(&mut topo, 40., 30., 4.).unwrap();
    for (i, (x, y)) in [(5., 6.), (9., 10.), (23., 14.), (35., 24.)]
        .into_iter()
        .enumerate()
    {
        let tool = make_cylinder(&mut topo, 2.5, 8.).unwrap();
        transform_solid(&mut topo, tool, &Mat4::translation(x, y, -2.)).unwrap();
        plate = boolean(&mut topo, BooleanOp::Cut, plate, tool).unwrap();
        let count = i + 1;
        let caps = remus_topology::explorer::solid_faces(&topo, plate)
            .unwrap()
            .into_iter()
            .filter(|&f| {
                let face = topo.face(f).unwrap();
                matches!(face.surface(), FaceSurface::Plane { .. })
                    && face.inner_wires().len() == count
            })
            .count();
        assert_eq!(caps, 2);
        PROBE_MESHES.with(|n| n.set(0));
        let v = solid_volume(&topo, plate, 0.08).unwrap();
        let meshes = PROBE_MESHES.with(std::cell::Cell::get);
        if count == 1 {
            assert!(meshes >= 2, "one hole uses the original cap meshes");
        } else {
            assert_eq!(meshes, 0, "{count} disjoint bores");
        }
        let expected = 40. * 30. * 4.
            - f64::from(u32::try_from(count).unwrap()) * std::f64::consts::PI * 2.5 * 2.5 * 4.;
        assert!(
            (v - expected).abs() < 1e-8,
            "after {count} cuts: {v} vs {expected}"
        );
    }
}

#[test]
fn an_unsupported_wire_or_trim_keeps_the_mesh_probe() {
    let mut topo = Topology::new();
    let cap = plate_cap(&mut topo, &[(5., 6., 2.), (20., 15., 3.)], 0.);
    let check = |t: &Topology| perforated_plate::sampled_area_agrees(t, cap, 1160., 0.1, 1000.);
    assert!(check(&topo));
    let outer = topo.face(cap).unwrap().outer_wire();
    let holes = topo.face(cap).unwrap().inner_wires().to_vec();
    let triangle = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0., 0., 2.),
            Point3::new(40., 0., 2.),
            Point3::new(0., 30., 2.),
        ],
        1e-7,
    )
    .unwrap();
    topo.set_face_boundary_wires(cap, triangle, holes.clone())
        .unwrap();
    assert!(!check(&topo));
    topo.set_face_boundary_wires(cap, outer, holes.clone())
        .unwrap();
    let edge = topo.wire(holes[0]).unwrap().edges()[0].edge();
    for trim in [Some((0., std::f64::consts::PI)), None] {
        topo.edge_mut(edge).unwrap().set_trim(trim);
        assert!(!check(&topo));
    }
}
