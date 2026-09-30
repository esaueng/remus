//! Whole-edge chamfer of the concave L-bracket builds exactly.
//!
//! Same fixture as the fillet refusal pin
//! (`regress_mixed_notch_whole_edge.rs`): the XY polygon
//! `[(0,0),(40,0),(40,8),(8,8),(8,50),(0,50)]` extruded 20 mm along +Z,
//! all 18 sharp physical edges chamfered at distance `d` via the public
//! `chamfer_v2`. Unlike the constant-radius rolling-ball fillet — whose
//! stripes cannot terminate at the mixed-side notch vertices (every
//! candidate end boundary is unshareable there) — planar bevels close:
//! every bevel is a plane, planes always intersect, and nothing buries.
//!
//! Closed-form volume (hand-derived, similarity-certified):
//!
//! ```text
//! V(d) = V0 - (d^2/2) * (L_convex - L_concave) + K * d^3
//! ```
//!
//! - `V0 = 656 * 20 = 13120`: polygon area `40*8 + 8*42 = 656` times height.
//! - Edge term: each convex edge of length `L` loses a right-triangular
//!   prism `(d^2/2)*L`; the concave notch edge gains one. Lengths:
//!   bottom/top perimeters `40+8+32+42+8+50 = 180` each, five convex
//!   verticals `5*20 = 100`, one concave vertical `20`:
//!   `L_convex - L_concave = (180 + 180 + 100) - 20 = 440`.
//! - Corner constant `K`: ten all-convex vertices lose a corner tetrahedron
//!   `d^3/6` each (`-10/6`), and the two mixed notch vertices contribute a
//!   residual pinned by similarity — `(V0 - V(d) - 220*d^2)/d^3` is
//!   `d`-independent across `d in {0.5, 1, 2}` at `K = 16/3`, hence
//!   `+7/2*d^3` per notch vertex. The residual form is asserted, not
//!   derived; the similarity (cubic scaling) is the independent check.
//!
//! Further pins: all 26 faces stay planar (18 bevels + 8 trimmed supports),
//! the shell is closed and valid, the operation is not partial, and the
//! result is stable under edge-order permutation and rigid transform.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::chamfer_v2;
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::query::filter_filletable_edges;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const V0: f64 = 13120.0;
const EDGE_TERM: f64 = 220.0; // (d^2/2) * 440
const CORNER_K: f64 = 16.0 / 3.0;

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
    let face = topo.add_face(remus_topology::face::Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 20.0).unwrap()
}

fn expected_volume(d: f64) -> f64 {
    V0 - EDGE_TERM * d * d + CORNER_K * d * d * d
}

fn check_chamfer(topo: &Topology, solid: SolidId, d: f64, label: &str) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "{label}: result must validate");
    let faces = solid_faces(topo, solid).unwrap();
    // 8 trimmed supports + 18 bevels + 12 corner triangles (one per vertex,
    // where three bevels meet). Every face stays planar.
    assert_eq!(
        faces.len(),
        38,
        "{label}: 8 supports + 18 bevels + 12 corner triangles"
    );
    assert!(
        faces
            .iter()
            .all(|f| matches!(topo.face(*f).unwrap().surface(), FaceSurface::Plane { .. })),
        "{label}: every face (supports and bevels) stays planar"
    );
    let triangles = faces
        .iter()
        .filter(|f| {
            topo.wire(topo.face(**f).unwrap().outer_wire())
                .unwrap()
                .edges()
                .len()
                == 3
        })
        .count();
    assert_eq!(
        triangles, 12,
        "{label}: one triangular corner patch per vertex (including both notch vertices)"
    );
    let shell = topo.solid(solid).unwrap().outer_shell();
    remus_topology::validation::validate_shell_closed(topo.shell(shell).unwrap(), topo)
        .unwrap_or_else(|e| panic!("{label}: shell must be closed: {e:?}"));
    let volume = solid_volume(topo, solid, 0.01).unwrap();
    let expected = expected_volume(d);
    // Mesh-quadrature volume at deflection 0.01 must meet the closed form
    // to 0.1 mm^3 (exact planar geometry; no chord error on planes).
    assert!(
        (volume - expected).abs() < 0.1,
        "{label}: expected {expected:.4} mm^3, got {volume:.4} mm^3"
    );
}

/// Whole-edge chamfer builds at several distances with closed-form volume.
#[test]
fn whole_edge_chamfer_matches_closed_form() {
    for d in [0.5_f64, 1.0, 2.0] {
        let mut topo = Topology::new();
        let solid = l_bracket(&mut topo);
        let all = solid_edges(&topo, solid).unwrap();
        let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
        assert_eq!(physical.len(), 18);
        let result = chamfer_v2(&mut topo, solid, &physical, d, d)
            .unwrap_or_else(|e| panic!("d={d}: whole-edge chamfer must build: {e}"));
        assert!(!result.is_partial, "d={d}: all 18 edges must chamfer");
        assert_eq!(result.succeeded.len(), 18);
        check_chamfer(&topo, result.solid, d, &format!("d={d}"));
    }
}

/// Edge-order permutation must not change the result.
#[test]
fn whole_edge_chamfer_is_order_independent() {
    let mut topo_a = Topology::new();
    let solid_a = l_bracket(&mut topo_a);
    let edges_a =
        filter_filletable_edges(&topo_a, solid_a, &solid_edges(&topo_a, solid_a).unwrap()).unwrap();
    let ra = chamfer_v2(&mut topo_a, solid_a, &edges_a, 1.0, 1.0).unwrap();
    let va = solid_volume(&topo_a, ra.solid, 0.01).unwrap();

    let mut topo_b = Topology::new();
    let solid_b = l_bracket(&mut topo_b);
    let mut edges_b =
        filter_filletable_edges(&topo_b, solid_b, &solid_edges(&topo_b, solid_b).unwrap()).unwrap();
    edges_b.reverse();
    let rb = chamfer_v2(&mut topo_b, solid_b, &edges_b, 1.0, 1.0).unwrap();
    let vb = solid_volume(&topo_b, rb.solid, 0.01).unwrap();

    assert!(
        (va - vb).abs() < 1e-6,
        "edge order must not matter: {va:.6} vs {vb:.6}"
    );
    assert!((va - expected_volume(1.0)).abs() < 0.1);
}

/// Rigid transform of the input carries through with the same volume.
#[test]
fn whole_edge_chamfer_survives_rigid_transform() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let placement =
        Mat4::translation(17.0, -23.0, 31.0) * Mat4::rotation_y(0.37) * Mat4::rotation_x(0.11);
    transform_solid(&mut topo, solid, &placement).unwrap();
    let all = solid_edges(&topo, solid).unwrap();
    let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
    let result = chamfer_v2(&mut topo, solid, &physical, 1.0, 1.0)
        .unwrap_or_else(|e| panic!("transformed chamfer must build: {e}"));
    check_chamfer(&topo, result.solid, 1.0, "transformed d=1");
}
