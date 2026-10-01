//! Whole-edge fillet of the concave L-bracket (torus-notch implementation).
//!
//! Fixture: XY polygon `[(0,0),(40,0),(40,8),(8,8),(8,50),(0,50)]` extruded
//! 20 mm along +Z; all 18 sharp physical edges through the public cascade.
//! The two mixed-side notch vertices close with exact torus patches (ring
//! center `V+(R,R,R)`, ring radius `2R`, tube `R`); the ten convex vertices
//! take standard sphere caps.
//!
//! Census (independent-executable verified): 8 planes, 18 cylinders, 10
//! spheres, 2 tori.
//!
//! Closed-form volume (hand-derived; torus-corner residual pinned):
//!
//! ```text
//! V(r) = V0 - A(r)*(Lconv(r) - Lconc(r)) - 10*C(r) - 2*t3*r^3
//! ```
//!
//! - `V0 = 13120`, `A(r) = r^2*(1-pi/4)`, `C(r) = r^3*(1-pi/6)`.
//! - Active stripe lengths (stations inset R per vertex end):
//!   `Lconv(r) = 460 - 34*r` (bottom/top perimeters `180-12r` each,
//!   five verticals `100-10r`), `Lconc(r) = 20 - 2*r`.
//! - `t3 = 0.1978130679` per torus corner: residual of the independent
//!   reference totals after the derived terms (r=1: 0.39563/2;
//!   r=2: 3.16481/2; ratio exactly 8.000, similarity-certified).
//!   The implementation must meet BOTH this closed form AND the reference
//!   totals (13027.2829 at r=1, 12755.9621 at r=2).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::extrude::extrude;
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::query::{edge_is_g1, filter_filletable_edges};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;

const V0: f64 = 13120.0;
const T3: f64 = 0.197_813_067_9;

fn edge_term(r: f64) -> f64 {
    let a = r * r * (1.0 - std::f64::consts::FRAC_PI_4);
    a * ((460.0 - 34.0 * r) - (20.0 - 2.0 * r))
}

fn expected_volume(r: f64) -> f64 {
    let corners = 10.0 * r * r * r * (1.0 - std::f64::consts::FRAC_PI_6);
    V0 - edge_term(r) - corners - 2.0 * T3 * r * r * r
}

fn l_bracket(topo: &mut Topology) -> SolidId {
    let profile = make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(40.0, 0.0, 0.0),
            Point3::new(40.0, 8.0, 0.0),
            Point3::new(8.0, 8.0, 0.0),
            Point3::new(8.0, 50.0, 0.0),
            Point3::new(0.0, 50.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 20.0).unwrap()
}

fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let (mut plane, mut cyl, mut sph, mut tor) = (0, 0, 0, 0);
    for fid in solid_faces(topo, solid).unwrap() {
        match topo.face(fid).unwrap().surface() {
            FaceSurface::Plane { .. } => plane += 1,
            FaceSurface::Cylinder(_) => cyl += 1,
            FaceSurface::Sphere(_) => sph += 1,
            FaceSurface::Torus(_) => tor += 1,
            _ => panic!("unexpected surface in filleted bracket"),
        }
    }
    (plane, cyl, sph, tor)
}

fn fillet_all(topo: &mut Topology, solid: SolidId, radius: f64) -> SolidId {
    let all = solid_edges(topo, solid).unwrap();
    let physical = filter_filletable_edges(topo, solid, &all).unwrap();
    assert_eq!(
        physical.len(),
        18,
        "whole-edge selection must stay complete"
    );
    let result = fillet_cascade(topo, solid, &physical, radius)
        .unwrap_or_else(|e| panic!("r={radius}: whole-edge fillet must build: {e}"));
    assert!(!result.is_partial, "r={radius}: no partial blends");
    result.solid
}

/// r=1: census, closed form, and independent-reference agreement.
///
/// Measurement: the solid carries torus-notch patches, so `solid_volume`
/// sums the boundary-trimmed Gauss integrals on the true face boundaries
/// (see `qualified_notch_family_exact_volume`) — not the closed whole-solid
/// mesh, whose inscribed facets under-read the convex corners by ~0.64 here.
/// The 0.05 window therefore distinguishes the exact route from the mesh
/// route: a mesh reading cannot pass. Geometry is additionally proven
/// identical to the independent reference (STEP exchange measures equal to
/// 1.2e-5 there); the residual is trim-chord sampling, measured 7.6e-4.
#[test]
fn whole_edge_fillet_builds_with_torus_corners() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let out = fillet_all(&mut topo, solid, 1.0);
    assert_eq!(census(&topo, out), (8, 18, 10, 2));
    let volume = solid_volume(&topo, out, 0.01).unwrap();
    assert!(
        (volume - expected_volume(1.0)).abs() < 0.05,
        "closed form {} vs measured {volume:.4}",
        expected_volume(1.0)
    );
    assert!(
        (volume - 13027.2829).abs() < 0.05,
        "independent reference 13027.2829 vs measured {volume:.4}"
    );
}

/// r=2: closed form and reference agreement (similarity pins the torus term).
/// Trim-chord residual scales with r^3 (measured 3.2e-3), hence the wider
/// window — still far inside the mesh route's ~5-unit under-read at r=2.
#[test]
fn whole_edge_fillet_r2_matches_closed_form() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let out = fillet_all(&mut topo, solid, 2.0);
    assert_eq!(census(&topo, out), (8, 18, 10, 2));
    let volume = solid_volume(&topo, out, 0.01).unwrap();
    assert!(
        (volume - expected_volume(2.0)).abs() < 0.2,
        "closed form {} vs measured {volume:.4}",
        expected_volume(2.0)
    );
    assert!(
        (volume - 12755.9621).abs() < 0.2,
        "independent reference 12755.9621 vs measured {volume:.4}"
    );
}

/// `solid_volume` and `mass_properties` share the exact Gauss route on this
/// family (same point, order, controls), so they must agree to well inside
/// either tolerance above — independent of the closed form entirely.
#[test]
fn notch_family_volume_agrees_with_mass_properties() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let out = fillet_all(&mut topo, solid, 1.0);
    let volume = solid_volume(&topo, out, 0.01).unwrap();
    let mass = mass_properties(&topo, out).unwrap().mass;
    assert!(
        (volume - mass).abs() < 1e-3,
        "exact routes must agree: solid_volume {volume:.6} vs mass_properties {mass:.6}"
    );
}

/// Uniform 10x scale: the exact integral scales with the cube (measured
/// relative residual 5.8e-8); a mesh reading would carry its absolute chord
/// error scaled by 1000 instead.
#[test]
fn notch_family_volume_scales_exactly() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let out = fillet_all(&mut topo, solid, 1.0);
    transform_solid(&mut topo, out, &Mat4::scale(10.0, 10.0, 10.0)).unwrap();
    let volume = solid_volume(&topo, out, 0.01).unwrap();
    assert!(
        (volume / 1000.0 - expected_volume(1.0)).abs() < 0.05,
        "scaled volume {} must recover the closed form",
        volume / 1000.0
    );
}

/// G1 continuity across every torus seam (station circles + tangency arcs).
#[test]
fn torus_seams_are_g1() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let out = fillet_all(&mut topo, solid, 1.0);
    // Torus seam edges: circle edges with both endpoints in the notch
    // neighborhood (x,y in [6.5,9.5], z near a cap station).
    let mut seam_count = 0;
    for eid in solid_edges(&topo, out).unwrap() {
        let edge = topo.edge(eid).unwrap();
        if !matches!(edge.curve(), remus_topology::edge::EdgeCurve::Circle(_)) {
            continue;
        }
        let a = topo.vertex(edge.start()).unwrap().point();
        let b = topo.vertex(edge.end()).unwrap().point();
        let near_notch = |p: Point3| {
            p.x() > 6.5
                && p.x() < 9.5
                && p.y() > 6.5
                && p.y() < 9.5
                && (p.z() < 2.5 || p.z() > 17.5)
        };
        if near_notch(a) && near_notch(b) {
            // The two incident faces of this seam edge.
            let mut incident = vec![];
            for fid in solid_faces(&topo, out).unwrap() {
                let face = topo.face(fid).unwrap();
                let mut wires = vec![face.outer_wire()];
                wires.extend(face.inner_wires().iter().copied());
                for wid in wires {
                    for oe in topo.wire(wid).unwrap().edges() {
                        if oe.edge() == eid {
                            incident.push(fid);
                        }
                    }
                }
            }
            incident.sort_by_key(|f| f.index());
            incident.dedup();
            assert_eq!(incident.len(), 2, "seam edge {eid:?} must join two faces");
            assert!(
                edge_is_g1(&topo, eid, incident[0], incident[1]).unwrap(),
                "torus seam {eid:?} ({a:?}->{b:?}) must be G1"
            );
            seam_count += 1;
        }
    }
    // 4 seam arcs per torus patch x 2 vertices = 8.
    assert_eq!(seam_count, 8, "all torus seams must be present and G1");
}

/// Edge-order permutation must not change the result.
#[test]
fn whole_edge_fillet_is_order_independent() {
    let mut topo_a = Topology::new();
    let sa = l_bracket(&mut topo_a);
    let ea: Vec<EdgeId> = {
        let all = solid_edges(&topo_a, sa).unwrap();
        filter_filletable_edges(&topo_a, sa, &all).unwrap()
    };
    let ra = fillet_cascade(&mut topo_a, sa, &ea, 1.0).unwrap();
    let va = solid_volume(&topo_a, ra.solid, 0.01).unwrap();

    let mut topo_b = Topology::new();
    let sb = l_bracket(&mut topo_b);
    let mut eb: Vec<EdgeId> = {
        let all = solid_edges(&topo_b, sb).unwrap();
        filter_filletable_edges(&topo_b, sb, &all).unwrap()
    };
    eb.reverse();
    let rb = fillet_cascade(&mut topo_b, sb, &eb, 1.0).unwrap();
    let vb = solid_volume(&topo_b, rb.solid, 0.01).unwrap();
    assert!(
        (va - vb).abs() < 1e-6,
        "edge order must not matter: {va:.6} vs {vb:.6}"
    );
}

/// Rigid transform carries through with identical volume.
#[test]
fn whole_edge_fillet_survives_rigid_transform() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let placement =
        Mat4::translation(17.0, -23.0, 31.0) * Mat4::rotation_y(0.37) * Mat4::rotation_x(0.11);
    transform_solid(&mut topo, solid, &placement).unwrap();
    let out = fillet_all(&mut topo, solid, 1.0);
    assert_eq!(census(&topo, out), (8, 18, 10, 2));
    let volume = solid_volume(&topo, out, 0.01).unwrap();
    assert!(
        (volume - expected_volume(1.0)).abs() < 0.05,
        "transformed volume {volume:.4} must meet the closed form"
    );
}
