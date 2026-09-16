//! B20 exit-criterion scale matrix: relative error ≤ 1e-6 against closed
//! forms on filleted and cavity primitives at scales 1e-3, 1, 1e3,
//! independent of caller deflection.
//!
//! Bodies (all closed forms composed from the same dimension constants the
//! model is built from — nothing is a recorded measurement):
//!
//! * `filleted_box_*` — box with one rolling-ball edge fillet (planar +
//!   cylinder faces; exercises the exact planar-boundary and curved-face
//!   area paths, plus deflection-independent volume/centroid/inertia).
//! * `hollow_box_*` — contained box cut (inner-shell cavity; the void
//!   subtracts through face reversal on every path).
//! * `bored_box_*` — box with a through cylindrical bore (circular-hole
//!   planar caps plus a cylinder wall).
//! * `tube_*` — coaxial cylinder minus cylinder (annulus caps, two walls).
//! * `hyperbola_segment_*` / `nurbs_circle_disc_*` — planar faces with a
//!   hyperbola boundary and a recognized-NURBS circular boundary.
//!
//! Every body is measured at three model scales and (where the API takes
//! one) three caller deflections spanning coarse-preview to fine. Volume,
//! centroid, and inertia go through both `mass_properties` (exact, no
//! deflection knob) and the deflection-taking `solid_volume` /
//! `solid_center_of_mass` / `face_area` / `solid_surface_area` entry points,
//! which must agree with each other and with the closed form.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{
    face_area, mass_properties, solid_center_of_mass, solid_surface_area, solid_volume,
};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::make_planar_face_from_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// B20 exit tolerance: relative error ≤ 1e-6.
const REL: f64 = 1e-6;

fn assert_rel(actual: f64, expected: f64, what: &str) {
    let scale = expected.abs().max(1e-300);
    assert!(
        (actual - expected).abs() <= REL * scale,
        "{what}: expected closed form {expected:.9e}, got {actual:.9e} \
         (rel {:+.3e} > {REL:.0e})",
        (actual - expected) / scale,
    );
}

/// Centroid assertion scaled by the body's own extent so the tolerance means
/// the same thing on every axis and at every model scale.
fn assert_com(actual: Point3, expected: Point3, extent: f64, what: &str) {
    for (axis, a, e) in [
        ("x", actual.x(), expected.x()),
        ("y", actual.y(), expected.y()),
        ("z", actual.z(), expected.z()),
    ] {
        assert!(
            (a - e).abs() <= REL * extent,
            "{what} CoM {axis}: expected {e:.9e}, got {a:.9e}",
        );
    }
}

/// Inertia assertion scaled by the largest diagonal moment (products of
/// inertia are legitimately zero, so they cannot scale by themselves).
fn assert_inertia(actual: [f64; 6], expected: [f64; 6], what: &str) {
    let iscale = expected[0]
        .abs()
        .max(expected[1].abs())
        .max(expected[2].abs())
        .max(1e-300);
    for (k, name) in ["Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]
        .iter()
        .enumerate()
    {
        assert!(
            (actual[k] - expected[k]).abs() <= REL * iscale,
            "{what} {name}: expected {:+.9e}, got {:+.9e}",
            expected[k],
            actual[k],
        );
    }
}

/// Inertia of a box `(dx, dy, dz)` centred at `c`, minus a coaxial z-cylinder
/// `(r, h)` centred at `bc` — the bored-box closed form about the global
/// origin, shifted to the centroid by the caller via Steiner where needed.
///
/// Returns `(volume, com, inertia_about_com)`.
fn bored_box_closed_form(
    dx: f64,
    dy: f64,
    dz: f64,
    box_min: Point3,
    r: f64,
    bore_center_xy: (f64, f64),
) -> (f64, Point3, [f64; 6]) {
    let bc = Point3::new(
        box_min.x() + dx / 2.0,
        box_min.y() + dy / 2.0,
        box_min.z() + dz / 2.0,
    );
    let vb = dx * dy * dz;
    let vc = PI * r * r * dz;
    let v = vb - vc;
    // Both centroids lie on the bore axis, so the composite centroid does too.
    let c = Point3::new(bore_center_xy.0, bore_center_xy.1, bc.z());
    // Box about its own centroid.
    let ib = [
        vb / 12.0 * (dy * dy + dz * dz),
        vb / 12.0 * (dx * dx + dz * dz),
        vb / 12.0 * (dx * dx + dy * dy),
    ];
    // Cylinder about its own centroid (transverse r²/4 + h²/12, axial r²/2).
    let ic = [
        vc * (3.0 * r * r + dz * dz) / 12.0,
        vc * (3.0 * r * r + dz * dz) / 12.0,
        vc * r * r / 2.0,
    ];
    // Parallel-axis shifts to the composite centroid.
    let shift = |m: f64, d: Point3| {
        let dx = d.x() - c.x();
        let dy = d.y() - c.y();
        let dz = d.z() - c.z();
        (
            m * (dy * dy + dz * dz),
            m * (dx * dx + dz * dz),
            m * (dx * dx + dy * dy),
            m * dx * dy,
            m * dx * dz,
            m * dy * dz,
        )
    };
    let sb = shift(vb, bc);
    let sc = shift(vc, Point3::new(bore_center_xy.0, bore_center_xy.1, bc.z()));
    let inertia = [
        ib[0] + sb.0 - (ic[0] + sc.0),
        ib[1] + sb.1 - (ic[1] + sc.1),
        ib[2] + sb.2 - (ic[2] + sc.2),
        sb.3 - sc.3,
        sb.4 - sc.4,
        sb.5 - sc.5,
    ];
    (v, c, inertia)
}

fn deflections(scale: f64) -> [f64; 3] {
    [scale * 1e3, scale * 0.1, scale * 1e-4]
}

// ---------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------

/// Box `(20, 20, 20)·s` with one rolling-ball edge fillet `r = 2·s`.
///
/// Closed-form volume `V = 8000·s³ − (1 − π/4)·(2s)²·20s`; the area closes
/// per the measured face census (see the area comment in the test).
fn filleted_box(topo: &mut Topology, s: f64) -> SolidId {
    let solid = make_box(topo, 20.0 * s, 20.0 * s, 20.0 * s).unwrap();
    let edges = remus_topology::explorer::solid_edges(topo, solid).unwrap();
    #[allow(deprecated)]
    remus_operations::fillet::fillet_rolling_ball(topo, solid, &[edges[0]], 2.0 * s).unwrap()
}

/// Box `(3, 3, 3)·s` with a contained `(1, 1, 1)·s` void at `(s, s, s)`.
/// Closed-form volume `26·s³`, centroid at the body centre.
fn hollow_box(topo: &mut Topology, s: f64) -> SolidId {
    let blank = make_box(topo, 3.0 * s, 3.0 * s, 3.0 * s).unwrap();
    let tool = make_box(topo, s, s, s).unwrap();
    transform_solid(topo, tool, &Mat4::translation(s, s, s)).unwrap();
    boolean(topo, BooleanOp::Cut, blank, tool).unwrap()
}

/// Box `(20, 20, 20)·s` with a through cylindrical bore `r = 2·s` along z at
/// the centre. Closed form via [`bored_box_closed_form`].
fn bored_box(topo: &mut Topology, s: f64) -> SolidId {
    let plate = make_box(topo, 20.0 * s, 20.0 * s, 20.0 * s).unwrap();
    let bore = make_cylinder(topo, 2.0 * s, 24.0 * s).unwrap();
    transform_solid(topo, bore, &Mat4::translation(10.0 * s, 10.0 * s, -2.0 * s)).unwrap();
    boolean(topo, BooleanOp::Cut, plate, bore).unwrap()
}

/// Coaxial tube: outer cylinder `r = 3·s, h = 10·s` less bore `r = s`.
/// Closed-form volume `80·π·s³`, centroid `(0, 0, 5·s)`.
fn tube(topo: &mut Topology, s: f64) -> SolidId {
    let outer = make_cylinder(topo, 3.0 * s, 10.0 * s).unwrap();
    let bore = make_cylinder(topo, s, 10.0 * s).unwrap();
    boolean(topo, BooleanOp::Cut, outer, bore).unwrap()
}

// ---------------------------------------------------------------------------
// Matrix
// ---------------------------------------------------------------------------

#[test]
fn b20_filleted_box_scale_matrix() {
    for s in [1e-3_f64, 1.0, 1e3] {
        let mut topo = Topology::new();
        let solid = filleted_box(&mut topo, s);
        let expected_v = (8000.0 - (1.0 - PI / 4.0) * 4.0 * 20.0) * s.powi(3);
        // Per-face census (probe b20_probe): 2 faces at 400·s², 2 at
        // 360·s² (each loses a 2s·20s rectangle), 2 at 399.14159265·s²
        // (= 400 − 4 + π: the tangency-trimmed pair), plus the
        // quarter-cylinder wall π·2s·20s/2 = 20π·s².
        let expected_a =
            (2.0 * 400.0 + 2.0 * 360.0 + 2.0 * 399.141_592_653_589_8 + 20.0 * PI) * s * s;
        let what = format!("filleted box s={s:e}");

        // mass_properties (exact, no deflection knob).
        let props = mass_properties(&topo, solid).unwrap();
        assert_rel(
            props.mass,
            expected_v,
            &format!("{what} mass_properties volume"),
        );

        // Deflection-taking entries agree with the closed form AND each
        // other at every caller deflection.
        for d in deflections(s) {
            assert_rel(
                solid_volume(&topo, solid, d).unwrap(),
                expected_v,
                &format!("{what} solid_volume d={d:e}"),
            );
            assert_rel(
                solid_surface_area(&topo, solid, d).unwrap(),
                expected_a,
                &format!("{what} solid_surface_area d={d:e}"),
            );
            // Centroid lies on the body's symmetry diagonal plane x = 10·s;
            // the fillet shifts y and z equally off 10·s.
            let com = solid_center_of_mass(&topo, solid, d).unwrap();
            assert_rel(com.x(), 10.0 * s, &format!("{what} CoM x d={d:e}"));
            assert!(
                (com.y() - com.z()).abs() <= REL * 20.0 * s,
                "{what} CoM y/z symmetry at d={d:e}: {} vs {}",
                com.y(),
                com.z(),
            );
        }
        // mass_properties centroid agrees with the deflection-taking one.
        let com = solid_center_of_mass(&topo, solid, s * 0.1).unwrap();
        assert!(
            (props.center.x() - com.x()).abs() <= REL * 20.0 * s,
            "{what} CoM path agreement x"
        );
        assert!(
            (props.center.y() - com.y()).abs() <= REL * 20.0 * s,
            "{what} CoM path agreement y"
        );
        assert!(
            (props.center.z() - com.z()).abs() <= REL * 20.0 * s,
            "{what} CoM path agreement z"
        );
    }
}

#[test]
fn b20_cavity_box_scale_matrix() {
    for s in [1e-3_f64, 1.0, 1e3] {
        let mut topo = Topology::new();
        let solid = hollow_box(&mut topo, s);
        // Exactly one inner (cavity) shell.
        assert_eq!(
            topo.solid(solid).unwrap().inner_shells().len(),
            1,
            "hollow box s={s:e} cavity"
        );
        let expected_v = 26.0 * s.powi(3);
        let expected_c = Point3::new(1.5 * s, 1.5 * s, 1.5 * s);
        let what = format!("hollow box s={s:e}");

        let props = mass_properties(&topo, solid).unwrap();
        assert_rel(
            props.mass,
            expected_v,
            &format!("{what} mass_properties volume"),
        );
        assert_com(
            props.center,
            expected_c,
            3.0 * s,
            &format!("{what} mass_properties"),
        );

        for d in deflections(s) {
            assert_rel(
                solid_volume(&topo, solid, d).unwrap(),
                expected_v,
                &format!("{what} solid_volume d={d:e}"),
            );
            assert_com(
                solid_center_of_mass(&topo, solid, d).unwrap(),
                expected_c,
                3.0 * s,
                &format!("{what} center_of_mass d={d:e}"),
            );
        }
    }
}

#[test]
fn b20_bored_box_scale_matrix() {
    for s in [1e-3_f64, 1.0, 1e3] {
        let mut topo = Topology::new();
        let solid = bored_box(&mut topo, s);
        let (expected_v, expected_c, expected_i) = bored_box_closed_form(
            20.0 * s,
            20.0 * s,
            20.0 * s,
            Point3::new(0.0, 0.0, 0.0),
            2.0 * s,
            (10.0 * s, 10.0 * s),
        );
        let what = format!("bored box s={s:e}");

        let props = mass_properties(&topo, solid).unwrap();
        assert_rel(
            props.mass,
            expected_v,
            &format!("{what} mass_properties volume"),
        );
        assert_com(
            props.center,
            expected_c,
            20.0 * s,
            &format!("{what} mass_properties"),
        );
        assert_inertia(
            props.inertia,
            expected_i,
            &format!("{what} mass_properties"),
        );

        for d in deflections(s) {
            assert_rel(
                solid_volume(&topo, solid, d).unwrap(),
                expected_v,
                &format!("{what} solid_volume d={d:e}"),
            );
            assert_com(
                solid_center_of_mass(&topo, solid, d).unwrap(),
                expected_c,
                20.0 * s,
                &format!("{what} center_of_mass d={d:e}"),
            );
        }
    }
}

#[test]
fn b20_tube_scale_matrix() {
    for s in [1e-3_f64, 1.0, 1e3] {
        let mut topo = Topology::new();
        let solid = tube(&mut topo, s);
        let expected_v = 80.0 * PI * s.powi(3);
        let expected_c = Point3::new(0.0, 0.0, 5.0 * s);
        // Closed-form inertia about the centroid: outer minus bore.
        let mo = PI * 9.0 * s * s * 10.0 * s;
        let mi = PI * s * s * 10.0 * s;
        let transverse = |m: f64, r: f64, h: f64| m * (3.0 * r * r + h * h) / 12.0;
        let expected_i = [
            transverse(mo, 3.0 * s, 10.0 * s) - transverse(mi, s, 10.0 * s),
            transverse(mo, 3.0 * s, 10.0 * s) - transverse(mi, s, 10.0 * s),
            mo * 9.0 * s * s / 2.0 - mi * s * s / 2.0,
            0.0,
            0.0,
            0.0,
        ];
        let what = format!("tube s={s:e}");

        let props = mass_properties(&topo, solid).unwrap();
        assert_rel(
            props.mass,
            expected_v,
            &format!("{what} mass_properties volume"),
        );
        assert_com(
            props.center,
            expected_c,
            10.0 * s,
            &format!("{what} mass_properties"),
        );
        assert_inertia(
            props.inertia,
            expected_i,
            &format!("{what} mass_properties"),
        );

        for d in deflections(s) {
            assert_rel(
                solid_volume(&topo, solid, d).unwrap(),
                expected_v,
                &format!("{what} solid_volume d={d:e}"),
            );
            assert_com(
                solid_center_of_mass(&topo, solid, d).unwrap(),
                expected_c,
                10.0 * s,
                &format!("{what} center_of_mass d={d:e}"),
            );
        }
    }
}

#[test]
fn b20_conic_planar_boundary_scale_matrix() {
    // Hyperbola segment (native edge): region between the arc
    // `a = 2·s, b = s, t ∈ [−0.8, 0.8]` and its chord. Closed form
    // `A = 2·b·(xc·sinh(t1) − a·(t1/2 + sinh(2·t1)/4))`, verified against
    // Simpson quadrature (0.775567953200 at unit scale).
    //
    // NURBS circle disc (recognized boundary): full-turn NURBS conversion
    // of a circle `r = 2·s`; closed form `4·π·s²`.
    for s in [1e-3_f64, 1.0, 1e3] {
        // --- Hyperbola segment face ---
        let h = remus_math::curves::Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            2.0 * s,
            s,
        )
        .unwrap();
        let (t0, t1) = (-0.8_f64, 0.8_f64);
        let (p0, p1) = (h.evaluate(t0), h.evaluate(t1));
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(p0, 1e-9));
        let v1 = topo.add_vertex(Vertex::new(p1, 1e-9));
        let mut arc = Edge::new(v0, v1, EdgeCurve::Hyperbola(h));
        arc.set_trim(Some((t0, t1)));
        let arc = topo.add_edge(arc);
        let chord = topo.add_edge(Edge::new(v1, v0, EdgeCurve::Line));
        let wire = Wire::new(
            vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
            true,
        )
        .unwrap();
        let wire = topo.add_wire(wire);
        let face = make_planar_face_from_wire(&mut topo, wire).unwrap();
        let xc = 2.0 * s * t1.cosh();
        let expected_h =
            2.0 * s * (xc * t1.sinh() - 2.0 * s * (t1 / 2.0 + (2.0 * t1).sinh() / 4.0));
        for d in [s * 1e3, s * 1e-6] {
            assert_rel(
                face_area(&topo, face, d).unwrap(),
                expected_h,
                &format!("hyperbola segment s={s:e} d={d:e}"),
            );
        }

        // --- NURBS circle disc face ---
        let c = Circle3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0 * s,
        )
        .unwrap();
        let nc = remus_geometry::convert::curve_to_nurbs::circle_to_nurbs(
            &c,
            0.0,
            std::f64::consts::TAU,
        )
        .unwrap();
        let dom = nc.domain();
        let p0 = nc.evaluate(dom.0);
        let mut topo = Topology::new();
        let v = topo.add_vertex(Vertex::new(p0, 1e-9));
        let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(nc));
        edge.set_trim(Some(dom));
        let edge = topo.add_edge(edge);
        let wire = Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap();
        let wire = topo.add_wire(wire);
        let face = make_planar_face_from_wire(&mut topo, wire).unwrap();
        let expected_n = 4.0 * PI * s * s;
        for d in [s * 1e3, s * 1e-6] {
            assert_rel(
                face_area(&topo, face, d).unwrap(),
                expected_n,
                &format!("NURBS circle disc s={s:e} d={d:e}"),
            );
        }
    }
}
