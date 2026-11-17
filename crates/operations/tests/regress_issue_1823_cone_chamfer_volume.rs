//! Regression (#1823): `solid_volume` read a chamfer over rounded corners
//! low at the default deflection.
//!
//! A box with its four upright edges rounded R5 measures exactly: every face
//! is a plane or a line/circle-trimmed cylinder, which `solid_volume`
//! integrates in closed form. Chamfering its top loop adds four exact cone
//! bands, and the cones sent the whole body to the inscribed whole-solid
//! mesh instead, which chords every round. The removal then read 0.19 mm³
//! heavy at a deflection of 1e-3 (0.097 at 5e-4, 0.039 at 2e-4), against a
//! closed form of 74.66.
//!
//! The cones now integrate exactly too, by Green's theorem over their chart
//! when they are bounded by rulings and coaxial rims. The volumes below are
//! computed without the kernel (Pappus for the chamfer, the elementary
//! formulas for the rest).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::blend_ops;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure;
use remus_operations::primitives::{make_box, make_cone, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// The deflection volume tests in this crate measure at.
const DEFLECTION: f64 = 1e-3;

fn face_edges(topo: &Topology, f: FaceId) -> Vec<EdgeId> {
    let face = topo.face(f).unwrap();
    std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .flat_map(|w| topo.wire(w).unwrap().edges().to_vec())
        .map(|oe| oe.edge())
        .collect()
}

fn solid_edges(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for f in solid_faces(topo, s).unwrap() {
        for e in face_edges(topo, f) {
            if !seen.contains(&e) {
                seen.push(e);
            }
        }
    }
    seen
}

fn ends(topo: &Topology, e: EdgeId) -> (Point3, Point3) {
    let edge = topo.edge(e).unwrap();
    (
        topo.vertex(edge.start()).unwrap().point(),
        topo.vertex(edge.end()).unwrap().point(),
    )
}

fn cones(topo: &Topology, s: SolidId) -> usize {
    solid_faces(topo, s)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_)))
        .count()
}

fn assert_close(actual: f64, expected: f64, budget: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= budget,
        "{what}: volume {actual:.9}, expected {expected:.9} (off {:e}, budget {budget:e})",
        actual - expected
    );
}

/// A 40 x 40 x 20 box with its four upright edges rounded `radius`.
fn rounded_box(topo: &mut Topology, radius: f64) -> SolidId {
    let body = make_box(topo, 40.0, 40.0, 20.0).unwrap();
    let uprights: Vec<_> = solid_edges(topo, body)
        .into_iter()
        .filter(|&e| {
            let (a, b) = ends(topo, e);
            (a.z() - b.z()).abs() > 2.5
        })
        .collect();
    assert_eq!(uprights.len(), 4);
    let result = blend_ops::fillet_cascade(topo, body, &uprights, radius).unwrap();
    assert!(!result.is_partial);
    result.solid
}

/// The box's rounded loop at height `z` chamfered by `d`.
fn chamfer_loop(topo: &mut Topology, s: SolidId, z: f64, d: f64) -> SolidId {
    let edges: Vec<_> = solid_edges(topo, s)
        .into_iter()
        .filter(|&e| {
            let (a, b) = ends(topo, e);
            (a.z() - z).abs() < 1e-6 && (b.z() - z).abs() < 1e-6
        })
        .collect();
    assert_eq!(edges.len(), 8, "four lines and four arcs");
    let result = blend_ops::chamfer_v2(topo, s, &edges, d, d).unwrap();
    assert!(!result.is_partial);
    result.solid
}

/// The material a chamfer `d` removes from a 40-square loop rounded `radius`:
/// four straight prisms of the half `d` square, and the four quarter turns
/// of it about the rounds' axes (Pappus: its centroid sits `d/3` in from the
/// rounds' radius).
fn loop_chamfer_removal(radius: f64, d: f64) -> f64 {
    let section = 0.5 * d * d;
    4.0 * (40.0 - 2.0 * radius) * section + 2.0 * PI * (radius - d / 3.0) * section
}

/// The rounded box: the 40 x 40 x 20 block less the four corner spandrels.
fn rounded_box_volume(radius: f64) -> f64 {
    40.0 * 40.0 * 20.0 - 4.0 * (1.0 - PI / 4.0) * radius * radius * 20.0
}

/// The issue's case, its bottom-loop mirror, and a larger chamfer: exact at
/// the default deflection and at every other, as the unchamfered body is.
#[test]
fn rounded_loop_chamfer_over_cone_bands_reads_its_closed_form() {
    for (z, d) in [(20.0, 1.0), (0.0, 1.0), (20.0, 2.5)] {
        let what = format!("box, R5 rounds, loop at z {z} chamfered {d}");
        let mut topo = Topology::new();
        let round = rounded_box(&mut topo, 5.0);
        let chamfered = chamfer_loop(&mut topo, round, z, d);
        assert_eq!(
            cones(&topo, chamfered),
            4,
            "{what}: one cone over each round"
        );
        let before = rounded_box_volume(5.0);
        let after = before - loop_chamfer_removal(5.0, d);
        for deflection in [0.1, DEFLECTION, 2e-4] {
            let budget = 1e-9 * before;
            let what = format!("{what}, deflection {deflection}");
            assert_close(
                measure::solid_volume(&topo, round, deflection).unwrap(),
                before,
                budget,
                &format!("{what}: rounded"),
            );
            assert_close(
                measure::solid_volume(&topo, chamfered, deflection).unwrap(),
                after,
                budget,
                &format!("{what}: chamfered"),
            );
        }
    }
}

/// Planes and cones alone, through the same exact route: a cone and a
/// frustum standing on a box's top face. Each cone wall is bounded by its
/// base circle and either the apex or its top circle.
#[test]
fn coaxial_cones_on_a_box_read_their_closed_forms() {
    let cases = [
        ("cone", 5.0, 0.0, 8.0),
        ("frustum", 5.0, 2.0, 6.0),
        ("widening frustum", 2.0, 5.0, 6.0),
    ];
    for (what, bottom, top, height) in cases {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
        let cone = make_cone(&mut topo, bottom, top, height).unwrap();
        transform_solid(&mut topo, cone, &Mat4::translation(10.0, 10.0, 10.0)).unwrap();
        let fused = boolean(&mut topo, BooleanOp::Fuse, block, cone).unwrap();
        assert_eq!(cones(&topo, fused), 1, "{what}: the cone wall stays a cone");
        let frustum = PI * height / 3.0 * (bottom * bottom + bottom * top + top * top);
        let expected = 20.0 * 20.0 * 10.0 + frustum;
        assert_close(
            measure::solid_volume(&topo, fused, DEFLECTION).unwrap(),
            expected,
            1e-9 * expected,
            what,
        );
    }
}

/// The primitives keep their closed forms.
#[test]
fn primitive_volumes_are_unchanged() {
    let mut topo = Topology::new();
    let cases = [
        ("box", make_box(&mut topo, 3.0, 4.0, 5.0).unwrap(), 60.0),
        (
            "cylinder",
            make_cylinder(&mut topo, 2.0, 7.0).unwrap(),
            PI * 4.0 * 7.0,
        ),
        (
            "cone",
            make_cone(&mut topo, 3.0, 0.0, 6.0).unwrap(),
            PI * 9.0 * 6.0 / 3.0,
        ),
        (
            "frustum",
            make_cone(&mut topo, 3.0, 1.0, 6.0).unwrap(),
            PI * 6.0 / 3.0 * (9.0 + 3.0 + 1.0),
        ),
    ];
    for (what, solid, expected) in cases {
        assert_close(
            measure::solid_volume(&topo, solid, DEFLECTION).unwrap(),
            expected,
            1e-9 * expected,
            what,
        );
    }
}
