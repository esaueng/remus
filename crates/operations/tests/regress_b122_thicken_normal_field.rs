//! B122: `thicken` must grow a face along its own outward normal field.
//!
//! `thicken` extruded every face along the carrier normal at parameter
//! `(0, 0)` and ignored `is_reversed`. A curved face was swept along that one
//! direction instead of offset (a half-cylinder wall slid sideways whichever
//! half it was), and a reversed face — a bored hole's wall, a planar face
//! carried against its plane normal — grew into the material it bounds.
//!
//! Oracles, all independent of the kernel's measurement routing:
//!
//! - closed-form volumes of the thickened regions: annular sectors and tubes
//!   `θ/2·(r₁² − r₀²)·h`, a hemispherical shell `2π/3·(r₁³ − r₀³)`, and a
//!   frustum wall by Pappus (a rectangle `L × t` in the meridian half-plane,
//!   revolved about its centroid);
//! - `classify_point` probes on both sides of the source face, so a solid of
//!   the right size on the wrong side fails;
//! - `validate_solid` on every result.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::thicken::thicken;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use PointClassification::{Inside, Outside};

/// An oblique placement off the origin, so a wall facing the wrong way
/// cannot hide behind a plane through the origin.
fn placement() -> Mat4 {
    Mat4::translation(7.0, -4.0, 3.0) * Mat4::rotation_x(0.6) * Mat4::rotation_z(0.3)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// The thickened solid is valid, measures `expected` on both volume routes,
/// and classifies every probe as given.
fn check(topo: &Topology, solid: SolidId, expected: f64, probes: &[(Point3, PointClassification)]) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "invalid thickened solid: {:?}",
        report.issues
    );
    let exact = mass_properties(topo, solid).unwrap().mass;
    assert!(
        rel(exact, expected) <= 1e-6,
        "mass_properties {exact}, closed form {expected}"
    );
    let routed = solid_volume(topo, solid, 0.01).unwrap();
    assert!(
        rel(routed, expected) <= 1e-3,
        "solid_volume {routed}, closed form {expected}"
    );
    for &(p, want) in probes {
        let got = classify_point(topo, solid, p, 0.01, 1e-7).unwrap();
        assert_eq!(got, want, "probe {p:?}");
    }
}

/// The half-cylinder wall `r = R`, `θ ∈ [π, 2π]` (the `y < 0` half),
/// `z ∈ [0, h]`, as a cylinder face bounded by two rulings and two
/// half-circle arcs. `reversed` carries it against the carrier normal; the
/// kernel then reads the same stored wire the other way round.
fn half_cylinder_wall(topo: &mut Topology, radius: f64, height: f64, reversed: bool) -> FaceId {
    let tol = 1e-7;
    let z = Vec3::new(0.0, 0.0, 1.0);
    let arc = |level: f64| {
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, level),
            z,
            radius,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    };
    let (bottom, top) = (arc(0.0), arc(height));
    let p0 = topo.add_vertex(Vertex::new(bottom.evaluate(PI), tol));
    let p1 = topo.add_vertex(Vertex::new(bottom.evaluate(TAU), tol));
    let p2 = topo.add_vertex(Vertex::new(top.evaluate(TAU), tol));
    let p3 = topo.add_vertex(Vertex::new(top.evaluate(PI), tol));
    let arc_edge = |topo: &mut Topology, a, b, circle: Circle3D| {
        let mut edge = Edge::new(a, b, EdgeCurve::Circle(circle));
        edge.set_trim(Some((PI, TAU)));
        topo.add_edge(edge)
    };
    let e_bottom = arc_edge(topo, p0, p1, bottom);
    let e_top = arc_edge(topo, p3, p2, top);
    let e_right = topo.add_edge(Edge::new(p1, p2, EdgeCurve::Line));
    let e_left = topo.add_edge(Edge::new(p3, p0, EdgeCurve::Line));
    // Counter-clockwise about the carrier (−y) normal: along the bottom arc
    // through (0, −R, 0), up the x = R ruling, back along the top arc, down.
    let uses = vec![
        OrientedEdge::new(e_bottom, true),
        OrientedEdge::new(e_right, true),
        OrientedEdge::new(e_top, false),
        OrientedEdge::new(e_left, true),
    ];
    let wire = topo.add_wire(Wire::new(uses, true).unwrap());
    let surface = FaceSurface::Cylinder(
        CylindricalSurface::with_ref_dir(
            Point3::new(0.0, 0.0, 0.0),
            z,
            radius,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    if reversed {
        topo.add_face(Face::new_reversed(wire, vec![], surface))
    } else {
        topo.add_face(Face::new(wire, vec![], surface))
    }
}

/// The review's case: the `y < 0` half-cylinder wall grows outward (to
/// larger radius, still at `y < 0`) and shrinks inward, never sideways.
#[test]
fn b122_half_cylinder_wall_thickens_along_its_normals() {
    let (r, h) = (3.0, 4.0);
    for (t, inner, outer) in [(1.0, 3.0, 4.0), (-1.0, 2.0, 3.0)] {
        let mut topo = Topology::new();
        let face = half_cylinder_wall(&mut topo, r, h, false);
        let solid = thicken(&mut topo, face, t).unwrap();
        let mid = 0.5 * (inner + outer);
        check(
            &topo,
            solid,
            FRAC_PI_2 * (outer * outer - inner * inner) * h,
            &[
                (Point3::new(0.0, -mid, 2.0), Inside),
                (Point3::new(mid * 0.6, -mid * 0.8, 1.0), Inside),
                (Point3::new(0.0, mid, 2.0), Outside),
                (Point3::new(0.0, -(inner - 0.5), 2.0), Outside),
                (Point3::new(0.0, -(outer + 0.5), 2.0), Outside),
                (Point3::new(mid, 0.5, 2.0), Outside),
            ],
        );
    }
}

/// The same wall carried reversed faces the axis: positive thickness grows
/// towards it.
#[test]
fn b122_reversed_half_cylinder_wall_grows_towards_the_axis() {
    let mut topo = Topology::new();
    let face = half_cylinder_wall(&mut topo, 3.0, 4.0, true);
    let solid = thicken(&mut topo, face, 1.0).unwrap();
    check(
        &topo,
        solid,
        FRAC_PI_2 * (9.0 - 4.0) * 4.0,
        &[
            (Point3::new(0.0, -2.5, 2.0), Inside),
            (Point3::new(0.0, -3.5, 2.0), Outside),
        ],
    );
}

/// The wall of a hole bored through a box is reversed: thickening it by `t`
/// lines the hole (a tube inside it), by `−t` grows into the box.
#[test]
fn b122_bored_hole_wall_thickens_into_the_hole() {
    let (r, h) = (2.0, 10.0);
    for (t, inner, outer) in [(1.0, 1.0, 2.0), (-1.0, 2.0, 3.0)] {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, h).unwrap();
        let tool = make_cylinder(&mut topo, r, 3.0 * h).unwrap();
        remus_operations::transform::transform_solid(
            &mut topo,
            tool,
            &remus_math::mat::Mat4::translation(5.0, 5.0, -h),
        )
        .unwrap();
        let bored = boolean(&mut topo, BooleanOp::Cut, block, tool).unwrap();
        let walls: Vec<FaceId> = remus_topology::explorer::solid_faces(&topo, bored)
            .unwrap()
            .into_iter()
            .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
            .collect();
        assert_eq!(walls.len(), 1, "one bore wall");
        assert!(
            topo.face(walls[0]).unwrap().is_reversed(),
            "a bore wall faces its axis"
        );

        let solid = thicken(&mut topo, walls[0], t).unwrap();
        let mid = 0.5 * (inner + outer);
        check(
            &topo,
            solid,
            PI * (outer * outer - inner * inner) * h,
            &[
                (Point3::new(5.0 + mid, 5.0, 5.0), Inside),
                (Point3::new(5.0, 5.0 - mid, 1.0), Inside),
                (Point3::new(5.0, 5.0, 5.0), Outside),
                (Point3::new(5.0 + inner - 0.5, 5.0, 5.0), Outside),
                (Point3::new(5.0 + outer + 0.5, 5.0, 5.0), Outside),
            ],
        );
    }
}

/// A planar face carried against its plane normal thickens along its own
/// outward side.
#[test]
fn b122_reversed_planar_face_thickens_along_its_outward_normal() {
    let tol = 1e-7;
    let mut topo = Topology::new();
    let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), tol)));
    // Counter-clockwise about the +z plane normal; the reversed face reads it
    // the other way round, about its −z outward normal.
    let uses: Vec<OrientedEdge> = (0..4)
        .map(|i| {
            let e = topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line));
            OrientedEdge::new(e, true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(uses, true).unwrap());
    let face = topo.add_face(Face::new_reversed(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let solid = thicken(&mut topo, face, 2.0).unwrap();
    check(
        &topo,
        solid,
        2.0,
        &[
            (Point3::new(0.5, 0.5, -1.0), Inside),
            (Point3::new(0.5, 0.5, 1.0), Outside),
        ],
    );
}

/// A sphere face offsets to the concentric sphere; its equator wall is a
/// planar annulus.
#[test]
fn b122_hemisphere_thickens_to_a_spherical_shell() {
    for t in [1.5, -1.5] {
        let r = 5.0;
        let mut topo = Topology::new();
        let ball = make_sphere(&mut topo, r, 16).unwrap();
        let m = placement();
        remus_operations::transform::transform_solid(&mut topo, ball, &m).unwrap();
        let upper = remus_topology::explorer::solid_faces(&topo, ball)
            .unwrap()
            .into_iter()
            .find(|&f| {
                // The hemisphere holding the north pole inside its trim.
                let pole = m.mul_point(Point3::new(0.0, 0.0, r));
                remus_check::classify::surface_point_in_face(&topo, f, pole).unwrap()
            })
            .expect("upper hemisphere");
        let solid = thicken(&mut topo, upper, t).unwrap();
        let (inner, outer) = if t > 0.0 { (r, r + t) } else { (r + t, r) };
        let mid = 0.5 * (inner + outer);
        check(
            &topo,
            solid,
            TAU / 3.0 * (outer.powi(3) - inner.powi(3)),
            &[
                (m.mul_point(Point3::new(0.0, 0.0, mid)), Inside),
                (m.mul_point(Point3::new(mid * 0.6, 0.0, mid * 0.8)), Inside),
                (m.mul_point(Point3::new(0.0, 0.0, -mid)), Outside),
                (m.mul_point(Point3::new(0.0, 0.0, inner - 0.5)), Outside),
                (m.mul_point(Point3::new(0.0, 0.0, outer + 0.5)), Outside),
            ],
        );
    }
}

/// A frustum's lateral face: its offset is a cone with the same half-angle,
/// its rim walls are cones whose apexes sit on the axis.
#[test]
fn b122_frustum_wall_thickens_by_pappus() {
    frustum_wall_thickens_by_pappus(4.0, 2.0, 3.0);
}

/// A nearly flat frustum (radii 4 and 0.1 over a height of 2e-7, half-angle
/// 5e-8): its top rim sits 0.1·h/3.9 ≈ 5e-9 above the apex, within the
/// linear tolerance, though 0.1 from it along the generator. It is not at
/// its apex, and thickens like any other cone wall.
#[test]
fn b122_shallow_frustum_wall_is_not_at_its_apex() {
    frustum_wall_thickens_by_pappus(4.0, 0.1, 2e-7);
}

fn frustum_wall_thickens_by_pappus(r0: f64, r1: f64, h: f64) {
    for t in [0.5, -0.5] {
        let mut topo = Topology::new();
        let frustum = make_cone(&mut topo, r0, r1, h).unwrap();
        let m = placement();
        remus_operations::transform::transform_solid(&mut topo, frustum, &m).unwrap();
        let wall = remus_topology::explorer::solid_faces(&topo, frustum)
            .unwrap()
            .into_iter()
            .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_)))
            .expect("frustum wall");
        let solid = thicken(&mut topo, wall, t).unwrap();
        // Meridian half-plane: the generator from (r0, 0) to (r1, h), its
        // outward unit normal n, and the rectangle it sweeps to its offset,
        // revolved about the axis through its centroid.
        let (dx, dz) = (r1 - r0, h);
        let length = dx.hypot(dz);
        let (nx, nz) = (dz / length, -dx / length);
        let (rho, z) = (0.5 * (r0 + r1) + 0.5 * t * nx, 0.5 * h + 0.5 * t * nz);
        let expected = TAU * rho * length * t.abs();
        // Across the wall, a quarter thickness beyond either face.
        let across = |k: f64| {
            m.mul_point(Point3::new(
                rho + k * t.abs() * nx,
                0.0,
                z + k * t.abs() * nz,
            ))
        };
        check(
            &topo,
            solid,
            expected,
            &[
                (m.mul_point(Point3::new(rho, 0.0, z)), Inside),
                (m.mul_point(Point3::new(0.0, -rho, z)), Inside),
                (across(0.75), Outside),
                (across(-0.75), Outside),
                (m.mul_point(Point3::new(0.0, 0.0, 0.5 * h)), Outside),
            ],
        );
    }
}

/// A whole torus (one face, closed line seams, no boundary) thickens to a
/// hollow tube: the offset torus and the original bound it as outer shell
/// and cavity.
#[test]
fn b122_whole_torus_thickens_to_a_hollow_tube() {
    let (major, minor) = (3.0, 1.0);
    for t in [0.5, -0.5] {
        let mut topo = Topology::new();
        let ring = make_torus(&mut topo, major, minor, 16).unwrap();
        let m = placement();
        remus_operations::transform::transform_solid(&mut topo, ring, &m).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, ring).unwrap();
        assert_eq!(faces.len(), 1, "make_torus is one face");
        let solid = thicken(&mut topo, faces[0], t).unwrap();
        let (inner, outer) = if t > 0.0 {
            (minor, minor + t)
        } else {
            (minor + t, minor)
        };
        let mid = 0.5 * (inner + outer);
        check(
            &topo,
            solid,
            2.0 * PI * PI * major * (outer * outer - inner * inner),
            &[
                (m.mul_point(Point3::new(major + mid, 0.0, 0.0)), Inside),
                (m.mul_point(Point3::new(0.0, -major, mid)), Inside),
                (m.mul_point(Point3::new(major, 0.0, 0.0)), Outside),
                (
                    m.mul_point(Point3::new(major + outer + 0.25, 0.0, 0.0)),
                    Outside,
                ),
                (m.mul_point(Point3::new(0.0, 0.0, 0.0)), Outside),
            ],
        );
    }
}

/// Thickness must be finite and non-zero, and an offset that collapses the
/// carrier is a typed refusal rather than a folded solid.
#[test]
fn b122_refusals_are_typed() {
    let mut topo = Topology::new();
    let face = half_cylinder_wall(&mut topo, 3.0, 4.0, false);
    for bad in [f64::NAN, f64::INFINITY, 0.0] {
        assert!(
            matches!(
                thicken(&mut topo, face, bad),
                Err(OperationsError::InvalidInput { .. })
            ),
            "thickness {bad} must be refused"
        );
    }
    // Thickening inward past the axis collapses the cylinder.
    assert!(matches!(
        thicken(&mut topo, face, -3.0),
        Err(OperationsError::Unsupported { .. })
    ));
}
