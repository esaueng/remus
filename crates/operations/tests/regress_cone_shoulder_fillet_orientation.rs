//! OpenZCAD #1258: a 1 mm fillet on the cone-to-cone shoulder that a 6.1 mm
//! top-rim chamfer leaves on a drafted frustum.
//!
//! The frustum is the −33° drafted extrusion of an r √29 circle, 23 mm tall
//! (top radius √29 + 23·tan 33°). Its shoulder is the circle at
//! z = 17.884109535532910. Remus 2026.1.122 built the shoulder fillet as a
//! closed walking band whose B-rep normal pointed radially inward: strict
//! validation passed, but the display mesh carried 1 674 inconsistently
//! wound edges and front-face culling hid the band. #1567 made the closed
//! walking band compare its normal with the ball-centre side at the same
//! point, and read concavity from the support instead of assuming it.
//!
//! The issue's chain is reproduced through the same calls the WASM `chamfer`
//! and `fillet` bindings make (planar bevel first, then `chamfer_v2`; the
//! filletable-edge filter, then `fillet_cascade`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Vec3;
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade, planar_chamfer_with_origins};
use remus_operations::primitives::make_cone;
use remus_operations::query::filter_filletable_edges;
use remus_operations::tessellate::{
    boundary_edge_count, is_watertight, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const SHOULDER_Z: f64 = 17.884_109_535_532_91;

fn circle_at(topo: &Topology, solid: SolidId, z: f64) -> EdgeId {
    let matches: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            matches!(edge.curve(), EdgeCurve::Circle(_))
                && (topo.vertex(edge.start()).unwrap().point().z() - z).abs() < 1e-5
        })
        .collect();
    assert_eq!(matches.len(), 1, "expected one circular edge at z {z}");
    matches[0]
}

/// The WASM `chamfer` binding's order: the planar bevel when it closes the
/// shell, otherwise the rolled-back topology goes to `chamfer_v2`.
fn app_chamfer(topo: &mut Topology, solid: SolidId, edge: EdgeId, distance: f64) -> SolidId {
    let snapshot = topo.clone();
    if let Ok((result, _)) = planar_chamfer_with_origins(topo, solid, &[edge], distance)
        && remus_operations::validate::validate_solid(topo, result)
            .is_ok_and(|report| report.is_valid())
    {
        return result;
    }
    topo.restore_preserving_handle_slots(&snapshot);
    chamfer_v2(topo, solid, &[edge], distance, distance)
        .unwrap()
        .solid
}

fn shoulder_fillet() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let bottom = 29.0f64.sqrt();
    let top = 23.0f64.mul_add(33.0f64.to_radians().tan(), bottom);
    let frustum = make_cone(&mut topo, bottom, top, 23.0).unwrap();
    let rim = circle_at(&topo, frustum, 23.0);
    let chamfered = app_chamfer(&mut topo, frustum, rim, 6.1);
    let shoulder = circle_at(&topo, chamfered, SHOULDER_Z);
    let edges = filter_filletable_edges(&topo, chamfered, &[shoulder]).unwrap();
    assert_eq!(edges, vec![shoulder]);
    let blended = fillet_cascade(&mut topo, chamfered, &edges, 1.0)
        .unwrap()
        .solid;
    (topo, blended)
}

#[test]
fn cone_shoulder_fillet_band_points_outward_and_pairs_its_boundaries() {
    let (topo, solid) = shoulder_fillet();
    let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(report.is_valid(), "{report:?}");

    let faces = solid_faces(&topo, solid).unwrap();
    let bands: Vec<_> = faces
        .iter()
        .copied()
        .filter(|&f| {
            let surface = topo.face(f).unwrap().surface();
            !matches!(
                surface,
                FaceSurface::Plane { .. } | FaceSurface::Cone(_) | FaceSurface::Cylinder(_)
            )
        })
        .collect();
    assert!(!bands.is_empty(), "the shoulder fillet adds a blend band");

    // The oriented B-rep normal of every blend face points out of the
    // convex shoulder, i.e. radially outward (the issue measured −0.98 …
    // −0.89 on 2026.1.122), and the band sits on the shoulder.
    for &band in &bands {
        let face = topo.face(band).unwrap();
        let FaceSurface::Nurbs(surface) = face.surface() else {
            panic!("the walked shoulder band is a NURBS face");
        };
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        for i in 1..8 {
            for j in 1..4 {
                let u = u0 + (u1 - u0) * f64::from(i) / 8.0;
                let v = v0 + (v1 - v0) * f64::from(j) / 4.0;
                let point = surface.evaluate(u, v);
                assert!(
                    (point.z() - SHOULDER_Z).abs() < 1.0,
                    "band point {point:?} is not on the shoulder"
                );
                let mut normal = surface.normal(u, v).unwrap();
                if face.is_reversed() {
                    normal = normal * -1.0;
                }
                let radial = Vec3::new(point.x(), point.y(), 0.0).normalize().unwrap();
                assert!(
                    normal.dot(radial) > 0.5,
                    "blend normal must point radially outward, got dot {}",
                    normal.dot(radial)
                );
            }
        }
    }

    // Each band boundary edge is used once in each direction by the shell:
    // the band and its neighbour traverse the shared contact oppositely.
    for &band in &bands {
        let wire = topo.wire(topo.face(band).unwrap().outer_wire()).unwrap();
        for use_ in wire.edges() {
            let mut senses = Vec::new();
            for &f in &faces {
                let face = topo.face(f).unwrap();
                for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().to_vec()) {
                    for other in topo.wire(wid).unwrap().edges() {
                        if other.edge() == use_.edge() {
                            senses.push(other.is_forward() != face.is_reversed());
                        }
                    }
                }
            }
            let forward = senses.iter().filter(|&&sense| sense).count();
            assert_eq!(
                (forward, senses.len()),
                (1, 2),
                "band boundary edge {:?} must be used once each way",
                use_.edge()
            );
        }
    }

    // Closed, consistently wound and outward at both of the issue's
    // deflections: an inward band shows as one-sided half-edges here.
    for deflection in [0.005, 0.05] {
        let mesh = tessellate_solid(&topo, solid, deflection).unwrap();
        assert!(is_watertight(&mesh), "open mesh at {deflection}");
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "inconsistent winding at {deflection}"
        );
        assert_eq!(non_manifold_edge_count(&mesh), 0);
        let signed: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|tri| {
                let [a, b, c] = [0, 1, 2].map(|k| {
                    let p = mesh.positions[tri[k] as usize];
                    Vec3::new(p.x(), p.y(), p.z())
                });
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        assert!(
            signed > 0.0,
            "mesh must enclose the solid outward at {deflection}"
        );
    }
}
