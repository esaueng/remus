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
