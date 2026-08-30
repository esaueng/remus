//! A face's measured properties must not depend on the orientation flags its
//! wires happen to store.
//!
//! `integrate_face` measures a line-and-arc-bounded planar face in closed form
//! (Green's theorem on the boundary), falling back to a chord polygon only when
//! the boundary carries an edge type the closed form cannot handle. Deciding
//! between the two needs the boundary's own plane, and `wire_newell_normal`
//! used to sample that plane by pushing `edge.start()` for every line edge,
//! ignoring the direction the WIRE traverses it.
//!
//! Booleans routinely emit a loop whose stored flags alternate — a rectangle
//! as `(A→B fwd), (C→B rev), (C→D fwd), (A→D rev)` — and for such a loop that
//! sampling yields `A, C, C, A`: a collapsed sequence whose Newell normal is
//! zero. The closed form was then rejected for the WHOLE face, holes included,
//! and a full-circle hole fell back to a 32-gon that is inscribed in it. The
//! face kept the sagitta ring as material.
//!
//! On the OpenZCAD demo bracket that face is the y = 32 wall, a 80 × 31.5
//! rectangle around the r = 10 boss bore: 2.0147 mm² of the bore counted as
//! material, and `mass_properties` read 21.49 mm³ (0.045 %) heavy. This test
//! pins the same face standalone, with no boolean in the loop.
//!
//! Every expected value below is a closed form in the dimension constants, not
//! a recorded measurement. `y` is constant on the face, so each divergence
//! integrand collapses to a power of `y` times a planar moment of the region.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::properties::face_integrator::{FaceContribution, integrate_face};
use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// The bracket's wall face: `x ∈ [0, W]`, `z ∈ [Z0, Z1]`, on the plane `y = Y`,
/// with a full-circle bore of radius `R` centred at `(CX, Y, CZ)`.
const W: f64 = 80.0;
const Z0: f64 = 8.0;
const Z1: f64 = 39.5;
const Y: f64 = 32.0;
const R: f64 = 10.0;
const CX: f64 = 40.0;
const CZ: f64 = 24.0;
const TOL: f64 = 1e-7;

/// How the four rectangle edges are wired up.
#[derive(Clone, Copy, Debug)]
enum Flags {
    /// Every edge stored start→end in loop order, every flag forward.
    AllForward,
    /// Alternating: edges 1 and 3 stored against the loop, flagged reversed.
    /// This is the shape a boolean leaves behind, and the one that used to
    /// collapse the plane-normal sample.
    Alternating,
}

/// Build the wall face on its own and return its integrated contribution.
fn wall_face(flags: Flags) -> FaceContribution {
    let mut topo = Topology::new();

    // Loop order: A(0,Z0) → B(W,Z0) → C(W,Z1) → D(0,Z1) → A, walked so the
    // region is on the left when viewed down −y.
    let a = topo_add(&mut topo, 0.0, Z0);
    let b = topo_add(&mut topo, W, Z0);
    let c = topo_add(&mut topo, W, Z1);
    let d = topo_add(&mut topo, 0.0, Z1);

    let oriented = match flags {
        Flags::AllForward => {
            let e0 = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
            let e1 = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
            let e2 = topo.add_edge(Edge::new(c, d, EdgeCurve::Line));
            let e3 = topo.add_edge(Edge::new(d, a, EdgeCurve::Line));
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, true),
                OrientedEdge::new(e2, true),
                OrientedEdge::new(e3, true),
            ]
        }
        Flags::Alternating => {
            let e0 = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
            let e1 = topo.add_edge(Edge::new(c, b, EdgeCurve::Line));
            let e2 = topo.add_edge(Edge::new(c, d, EdgeCurve::Line));
            let e3 = topo.add_edge(Edge::new(a, d, EdgeCurve::Line));
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, false),
                OrientedEdge::new(e2, true),
                OrientedEdge::new(e3, false),
            ]
        }
    };
    let outer = topo.add_wire(Wire::new(oriented, true).unwrap());

    // The bore rim: one closed circle edge on a single seam vertex.
    let circle = Circle3D::new(Point3::new(CX, Y, CZ), Vec3::new(0.0, -1.0, 0.0), R).unwrap();
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
    let mut rim_edge = Edge::new(seam, seam, EdgeCurve::Circle(circle));
    rim_edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let rim = topo.add_edge(rim_edge);
    let inner = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());

    let face = topo.add_face(Face::new(
        outer,
        vec![inner],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, -1.0, 0.0),
            d: -Y,
        },
    ));

    integrate_face(&topo, face, 8).unwrap()
}

fn topo_add(topo: &mut Topology, x: f64, z: f64) -> remus_topology::vertex::VertexId {
    topo.add_vertex(Vertex::new(Point3::new(x, Y, z), TOL))
}

fn assert_close(actual: f64, expected: f64, what: &str) {
    let scale = expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= 1e-9 * scale,
        "{what}: expected the closed form {expected:.9}, got {actual:.9} ({:+.9})",
        actual - expected
    );
}

#[test]
fn holed_planar_face_measures_the_same_under_either_wire_flagging() {
    let forward = wall_face(Flags::AllForward);
    let alternating = wall_face(Flags::Alternating);

    for (name, got, want) in [
        ("area", alternating.area, forward.area),
        ("volume", alternating.volume, forward.volume),
        (
            "volume_moment_y",
            alternating.volume_moment_y,
            forward.volume_moment_y,
        ),
        (
            "volume_second_y",
            alternating.volume_second_y,
            forward.volume_second_y,
        ),
        (
            "volume_product_yz",
            alternating.volume_product_yz,
            forward.volume_product_yz,
        ),
        ("centroid_x", alternating.centroid_x, forward.centroid_x),
        ("centroid_z", alternating.centroid_z, forward.centroid_z),
    ] {
        assert!(
            (got - want).abs() <= 1e-9 * want.abs().max(1.0),
            "{name} depends on how the wire flagged its edges: {want} vs {got}"
        );
    }
}

#[test]
fn holed_planar_face_contribution_matches_closed_form() {
    // Region: a rectangle less a disc that lies wholly inside it.
    let area = W * (Z1 - Z0) - PI * R * R;
    // First moments of the region about the global axes. The disc is concentric
    // in x with the rectangle here, but keep the general form.
    let rect_area = W * (Z1 - Z0);
    let disc_area = PI * R * R;
    let int_x = rect_area * (W / 2.0) - disc_area * CX;
    let int_z = rect_area * f64::midpoint(Z0, Z1) - disc_area * CZ;

    // Effective outward normal is the stored one (the face is not reversed).
    let ny = -1.0;

    for flags in [Flags::AllForward, Flags::Alternating] {
        let c = wall_face(flags);
        assert_close(c.area, area, &format!("{flags:?} area"));

        // volume = (1/3) ∮ P·n dA; only the y term survives, and y ≡ Y.
        assert_close(c.volume, ny * Y * area / 3.0, &format!("{flags:?} volume"));

        // volume_moment_* = (1/2) ∫ x_i² n_i dA — zero on the axes whose
        // normal component vanishes.
        assert_close(c.volume_moment_x, 0.0, &format!("{flags:?} moment_x"));
        assert_close(
            c.volume_moment_y,
            0.5 * ny * Y * Y * area,
            &format!("{flags:?} moment_y"),
        );
        assert_close(c.volume_moment_z, 0.0, &format!("{flags:?} moment_z"));

        // volume_second_y = (1/3) ∫ y³ n_y dA, volume_product_yz = (1/2) ∫ y² z n_y dA.
        assert_close(
            c.volume_second_y,
            ny * Y * Y * Y * area / 3.0,
            &format!("{flags:?} second_y"),
        );
        assert_close(c.volume_second_x, 0.0, &format!("{flags:?} second_x"));
        assert_close(c.volume_second_z, 0.0, &format!("{flags:?} second_z"));
        assert_close(
            c.volume_product_yz,
            0.5 * ny * Y * Y * int_z,
            &format!("{flags:?} product_yz"),
        );
        assert_close(c.volume_product_xy, 0.0, &format!("{flags:?} product_xy"));
        assert_close(c.volume_product_xz, 0.0, &format!("{flags:?} product_xz"));

        // Area-weighted centroid components are the region's first moments.
        assert_close(c.centroid_x, int_x, &format!("{flags:?} centroid_x"));
        assert_close(c.centroid_y, Y * area, &format!("{flags:?} centroid_y"));
        assert_close(c.centroid_z, int_z, &format!("{flags:?} centroid_z"));
    }
}

#[test]
fn a_full_circle_hole_is_not_charged_as_an_inscribed_polygon() {
    // The failure mode was specific: the hole was subtracted as the 32-gon
    // `crate::util::wire_polygon` samples, which is INSCRIBED in the rim, so
    // the face kept `πR² − (n/2)R² sin(2π/n)` of the bore as material. Assert
    // the residual is nowhere near that, in either flagging.
    let inscribed_32gon = 16.0 * R * R * (PI / 16.0).sin();
    let chord_deficit = PI * R * R - inscribed_32gon;
    assert!(
        chord_deficit > 2.0,
        "the 32-gon deficit this test guards against is {chord_deficit}, \
         too small to distinguish"
    );

    let exact = W * (Z1 - Z0) - PI * R * R;
    for flags in [Flags::AllForward, Flags::Alternating] {
        let area = wall_face(flags).area;
        assert!(
            (area - exact).abs() < 1e-9,
            "{flags:?}: face area {area} is not the closed form {exact}; \
             a chorded hole would read {}",
            W * (Z1 - Z0) - inscribed_32gon
        );
    }
}
