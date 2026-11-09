//! OpenZCAD: a fillet on the top rim of a cylinder that stands on a box
//! corner.
//!
//! Fusing an R6 × 31.25 cylinder with a 30 × 18 × 24 box whose corner sits on
//! the cylinder's axis leaves the cylinder's top rim (z = 31.25) as four
//! co-circular quarter arcs, tangent where they meet, between one planar cap
//! and one full-turn cylindrical wall. Moved off the corner, the same rim
//! stays one closed circle and rounds to an exact torus at any radius.
//!
//! The arc chain is built by the rolling-ball engine, one stripe per arc.
//! Each arc between the cap and its coaxial wall sweeps an exact torus about
//! the post's axis with ring radius `R − r`, but the engine took it only as a
//! ring torus (`R − r > r`). From half the rim radius up it lofted the arcs
//! instead, between straight edges: the cap came back as a square inscribed
//! in the contact circle, four straight seams cut ~1.1 mm inside the true
//! torus, and the solid lost 1.1–1.8 mm³ too much — while it still
//! validated clean. The spindle torus (`R − r ≤ r`) is the same exact
//! surface: the stripe is the quarter of the tube beyond the ball centres,
//! where it does not cross itself.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_4, PI, TAU};

use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::compound_ops::fuse_solids;
use remus_operations::heal::unify_faces_checked;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::query::filter_filletable_edges;
use remus_operations::tessellate::{
    boundary_edge_count, is_watertight, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const R: f64 = 6.0;
const TOP: f64 = 31.25;
const BOX: [f64; 3] = [30.0, 18.0, 24.0];

/// The fused body and the edges of its top rim.
fn fused_on_corner() -> (Topology, SolidId, Vec<EdgeId>) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, BOX[0], BOX[1], BOX[2]).unwrap();
    let post = make_cylinder(&mut topo, R, TOP).unwrap();
    let fused = fuse_solids(&mut topo, &[block, post]).unwrap();
    let report = unify_faces_checked(&mut topo, fused).unwrap();
    assert!(!report.reverted && report.result_errors == 0, "{report:?}");
    let rim: Vec<EdgeId> = solid_edges(&topo, fused)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            [edge.start(), edge.end()]
                .iter()
                .all(|&v| (topo.vertex(v).unwrap().point().z() - TOP).abs() < 1e-9)
        })
        .collect();
    (topo, fused, rim)
}

/// `π R² H` of the post plus the box, less the quarter of the post inside it.
fn union_volume() -> f64 {
    let post = PI * R * R;
    BOX[0].mul_add(BOX[1] * BOX[2], post * TOP) - post / 4.0 * BOX[2]
}

/// Material a convex rolling-ball fillet of radius `r` removes from the rim of
/// a radius-`R` cylinder, by Pappus: the square-minus-quarter-disc section
/// `r²(1 − π/4)` swept about the axis at its centroid radius.
fn torus_fillet_removal(r: f64) -> f64 {
    TAU * r * r * R.mul_add(1.0 - FRAC_PI_4, -r * (5.0 / 6.0 - FRAC_PI_4))
}

/// Largest distance from the sampled points of `edge` to the surface of
/// `face`.
fn edge_face_deviation(topo: &Topology, edge: EdgeId, face: FaceId) -> f64 {
    let data = topo.edge(edge).unwrap();
    let start = topo.vertex(data.start()).unwrap().point();
    let end = topo.vertex(data.end()).unwrap().point();
    let points: Vec<Point3> = match data.curve() {
        EdgeCurve::Line => (1..8)
            .map(|i| start + (end - start) * (f64::from(i) / 8.0))
            .collect(),
        EdgeCurve::Circle(circle) => {
            let (t0, t1) = data.trim().unwrap_or((0.0, TAU));
            (0..=16)
                .map(|i| circle.evaluate((t1 - t0).mul_add(f64::from(i) / 16.0, t0)))
                .collect()
        }
        _ => panic!("unexpected curve type on the rounded rim"),
    };
    let surface = topo.face(face).unwrap().surface();
    points
        .into_iter()
        .map(|point| {
            if let FaceSurface::Plane { normal, d } = surface {
                (normal.dot(point - Point3::new(0.0, 0.0, 0.0)) - d).abs()
            } else {
                let (u, v) = surface.project_point(point).unwrap();
                (surface.evaluate(u, v).unwrap() - point).length()
            }
        })
        .fold(0.0, f64::max)
}

#[test]
fn the_fused_rim_is_four_co_circular_quarter_arcs() {
    let (topo, _, rim) = fused_on_corner();
    assert_eq!(rim.len(), 4, "the boolean splits the top rim into quarters");
    for &e in &rim {
        let EdgeCurve::Circle(circle) = topo.edge(e).unwrap().curve() else {
            panic!("rim edge is not a circle");
        };
        assert!((circle.radius() - R).abs() < 1e-9);
        assert!((circle.center() - Point3::new(0.0, 0.0, TOP)).length() < 1e-9);
    }
}

#[test]
fn quarter_arc_rim_rounds_to_the_exact_torus_past_half_the_rim_radius() {
    for r in [2.9, 3.1, 3.8, 5.0] {
        let (mut topo, fused, rim) = fused_on_corner();
        let edges = filter_filletable_edges(&topo, fused, &rim).unwrap();
        assert_eq!(edges.len(), 4, "r {r}: every quarter arc is filletable");
        let solid = fillet_cascade(&mut topo, fused, &edges, r).unwrap().solid;
        let report = validate_solid(&topo, solid).unwrap();
        assert!(report.is_valid(), "r {r}: {report:?}");

        // Toroidal band faces of the requested section about the post's
        // axis, and no approximate face anywhere.
        let faces = solid_faces(&topo, solid).unwrap();
        let tori: Vec<_> = faces
            .iter()
            .filter_map(|&f| match topo.face(f).unwrap().surface() {
                FaceSurface::Torus(torus) => Some(torus.clone()),
                _ => None,
            })
            .collect();
        assert!(!tori.is_empty(), "r {r}: an exact torus band");
        for torus in &tori {
            assert!((torus.minor_radius() - r).abs() < 1e-9, "r {r}");
            assert!((torus.major_radius() - (R - r)).abs() < 1e-9, "r {r}");
            assert!(torus.z_axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12);
            assert!((torus.center() - Point3::new(0.0, 0.0, TOP - r)).length() < 1e-9);
        }
        assert!(
            faces
                .iter()
                .all(|&f| !matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_))),
            "r {r}: no approximate face"
        );

        // The cap is the disc of radius R − r: every edge bounding it is an
        // arc of that circle.
        let cap = faces
            .iter()
            .copied()
            .find(|&f| {
                matches!(
                    topo.face(f).unwrap().surface(),
                    FaceSurface::Plane { normal, d } if normal.z() > 0.5 && (d - TOP).abs() < 1e-9
                )
            })
            .expect("top cap");
        let cap_face = topo.face(cap).unwrap();
        assert!(
            cap_face.inner_wires().is_empty(),
            "r {r}: the cap is a disc"
        );
        for use_ in topo.wire(cap_face.outer_wire()).unwrap().edges() {
            let EdgeCurve::Circle(contact) = topo.edge(use_.edge()).unwrap().curve() else {
                panic!("r {r}: a cap boundary edge is not an arc");
            };
            assert!((contact.radius() - (R - r)).abs() < 1e-9, "r {r}");
            assert!((contact.center() - Point3::new(0.0, 0.0, TOP)).length() < 1e-9);
        }

        // Every edge of the result lies on each face it bounds.
        for &f in &faces {
            let face = topo.face(f).unwrap();
            for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().to_vec()) {
                for use_ in topo.wire(wire).unwrap().edges() {
                    let deviation = edge_face_deviation(&topo, use_.edge(), f);
                    assert!(
                        deviation < 1e-7,
                        "r {r}: edge {:?} leaves face {f:?} by {deviation:.3e}",
                        use_.edge()
                    );
                }
            }
        }

        // The removed material is exactly the torus fillet's.
        let volume = solid_volume(&topo, solid, 1e-3).unwrap();
        let expected = union_volume() - torus_fillet_removal(r);
        assert!(
            (volume - expected).abs() < 1e-3,
            "r {r}: volume {volume} expected {expected}"
        );

        let mesh = tessellate_solid(&topo, solid, 0.01).unwrap();
        assert!(is_watertight(&mesh), "r {r}");
        assert_eq!(boundary_edge_count(&mesh), 0, "r {r}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "r {r}");
    }
}
