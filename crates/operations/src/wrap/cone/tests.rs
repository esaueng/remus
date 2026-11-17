#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_polygon_wire};
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::*;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Apex at the origin, axis +Z, remus half-angle 60° (generators steep),
/// angle zero along +X; the profile's `z = 0` is the circle at height 10.
/// A profile on the plane x = 20 has arc length `y` and developed height
/// `z − 10`; the developed apex is at `(0, −ρ₀)`.
fn cone(beta_deg: f64) -> WrapCone {
    WrapCone {
        apex: Point3::new(0.0, 0.0, 0.0),
        axis: Z,
        reference: X,
        half_angle: beta_deg.to_radians(),
        origin: Point3::new(0.0, 0.0, 10.0),
    }
}

fn slant(beta_deg: f64) -> f64 {
    10.0 / beta_deg.to_radians().sin()
}

fn plane_face(topo: &mut Topology, wire: WireId, holes: Vec<WireId>) -> FaceId {
    topo.add_face(Face::new(
        wire,
        holes,
        FaceSurface::Plane { normal: X, d: 20.0 },
    ))
}

/// A point of the plane x = 20 at developed polar `(ρ, ψ)` about the
/// developed apex.
fn developed(rho: f64, psi: f64, rho0: f64) -> Point3 {
    Point3::new(20.0, rho * psi.sin(), 10.0 - rho0 + rho * psi.cos())
}

/// The annular sector `ρ ∈ [r1, r2]`, `ψ ∈ [p1, p2]` about the developed
/// apex, counter-clockwise about +X.
fn sector(topo: &mut Topology, r1: f64, r2: f64, p1: f64, p2: f64, rho0: f64) -> FaceId {
    let centre = Point3::new(20.0, 0.0, 10.0 - rho0);
    let v = |topo: &mut Topology, rho: f64, psi: f64| {
        topo.add_vertex(Vertex::new(developed(rho, psi, rho0), 1e-7))
    };
    let (a1, a2, b1, b2) = (
        v(topo, r1, p1),
        v(topo, r1, p2),
        v(topo, r2, p1),
        v(topo, r2, p2),
    );
    let arc = |topo: &mut Topology, r: f64, s, e| {
        let circle = Circle3D::with_axes(centre, -X, r, Z, Y).unwrap();
        let mut edge = Edge::new(s, e, EdgeCurve::Circle(circle));
        edge.set_trim(Some((p1, p2)));
        topo.add_edge(edge)
    };
    let inner_arc = arc(topo, r1, a1, a2);
    let outer_arc = arc(topo, r2, b1, b2);
    let down = topo.add_edge(Edge::new(b1, a1, EdgeCurve::Line));
    let up = topo.add_edge(Edge::new(a2, b2, EdgeCurve::Line));
    // ψ grows clockwise about +X, so the outer arc runs backwards.
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(outer_arc, false),
                OrientedEdge::new(down, true),
                OrientedEdge::new(inner_arc, true),
                OrientedEdge::new(up, true),
            ],
            true,
        )
        .unwrap(),
    );
    plane_face(topo, wire, vec![])
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::mass_properties(topo, solid).unwrap().mass
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

/// `dV = (1 + d·tan β / ρ) dA dd`: a sector between offsets `d0` and `d1`
/// in closed form.
fn sector_volume(beta: f64, r1: f64, r2: f64, dpsi: f64, d0: f64, d1: f64) -> f64 {
    let (sin, cos) = beta.sin_cos();
    dpsi / cos
        * (cos * (r2 * r2 - r1 * r1) / 2.0 * (d1 - d0)
            + sin * (r2 - r1) * (d1 * d1 - d0 * d0) / 2.0)
}

#[test]
fn a_developed_sector_wraps_to_two_cones_at_the_closed_form_volume() {
    let rho0 = slant(60.0);
    let mut topo = Topology::new();
    let profile = sector(&mut topo, rho0 - 3.0, rho0 + 5.0, -0.2, 0.3, rho0);
    let solid = wrap_profile_on_cone(&mut topo, profile, &cone(60.0), -0.5, 1.5).unwrap();
    assert_valid(&topo, solid, "sector");
    let mut cones = 0;
    for face in solid_faces(&topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Cone(_) => cones += 1,
            FaceSurface::Nurbs(_) => {}
            other => panic!("unexpected {}", other.type_tag()),
        }
    }
    assert_eq!(cones, 2);
    let exact = sector_volume(60f64.to_radians(), rho0 - 3.0, rho0 + 5.0, 0.5, -0.5, 1.5);
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-6, "{measured} vs {exact}");
}

#[test]
fn a_disc_wraps_at_its_developed_volume() {
    let beta = 50f64;
    let rho0 = slant(beta);
    let mut topo = Topology::new();
    let edge = make_circle_edge(&mut topo, Point3::new(20.0, 1.5, 12.0), X, 2.0, 1e-7).unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let profile = plane_face(&mut topo, wire, vec![]);
    let (d0, d1) = (0.0, 1.0);
    let solid = wrap_profile_on_cone(&mut topo, profile, &cone(beta), d0, d1).unwrap();
    assert_valid(&topo, solid, "disc");
    // V = Δd·A + tan β·(d1² − d0²)/2 · ∬ dA/ρ, the integral by polar
    // quadrature about the disc's centre (developed (1.5, 2 + ρ₀)).
    let (n_r, n_t) = (1200, 1200);
    let mut integral = 0.0;
    for i in 0..n_r {
        let r = 2.0 * (f64::from(i) + 0.5) / f64::from(n_r);
        for j in 0..n_t {
            let t = std::f64::consts::TAU * f64::from(j) / f64::from(n_t);
            let (s, y) = (1.5 + r * t.cos(), 2.0 + rho0 + r * t.sin());
            integral += r / s.hypot(y);
        }
    }
    integral *= (2.0 / f64::from(n_r)) * (std::f64::consts::TAU / f64::from(n_t));
    let area = std::f64::consts::PI * 4.0;
    let exact = (d1 - d0) * area + beta.to_radians().tan() * (d1 * d1 - d0 * d0) / 2.0 * integral;
    // The fitted edges lie on their offset cones: the certified part.
    let (sb, cb) = cone(beta).half_angle.sin_cos();
    for face in solid_faces(&topo, solid).unwrap() {
        let f = topo.face(face).unwrap();
        for oe in topo.wire(f.outer_wire()).unwrap().edges() {
            if let EdgeCurve::NurbsCurve(nc) = topo.edge(oe.edge()).unwrap().curve() {
                let (u0, u1) = nc.domain();
                let offsets: Vec<f64> = (0..=2000)
                    .map(|k| {
                        let p = nc.evaluate(u0 + (u1 - u0) * f64::from(k) / 2000.0);
                        p.x().hypot(p.y()) * sb - p.z() * cb
                    })
                    .collect();
                let near = |d: f64| offsets.iter().all(|&o| (o - d).abs() < 1e-6);
                assert!(near(d0) || near(d1), "an edge leaves its cone");
            }
        }
    }
    // Remus's face integrator reads a cone face trimmed by a curved (u, v)
    // boundary to the chording of that boundary (see `integrate_face`), here
    // 1.3e-4; a fine tessellation converges on the closed form instead.
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 2e-4, "{measured} vs {exact}");
    let mesh = crate::tessellate::tessellate_solid(&topo, solid, 2e-4).unwrap();
    let six: f64 = mesh
        .indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize]);
            Vec3::new(a.x(), a.y(), a.z()).dot(Vec3::new(b.x(), b.y(), b.z()).cross(Vec3::new(
                c.x(),
                c.y(),
                c.z(),
            )))
        })
        .sum();
    assert!(rel(six / 6.0, exact) < 5e-5, "{} vs {exact}", six / 6.0);
}

#[test]
fn wrapped_cone_solids_tessellate_watertight() {
    let rho0 = slant(60.0);
    let mut topo = Topology::new();
    let profile = sector(&mut topo, rho0 - 3.0, rho0 + 5.0, -0.2, 0.3, rho0);
    let solid = wrap_profile_on_cone(&mut topo, profile, &cone(60.0), 0.0, 1.0).unwrap();
    for deflection in [0.05, 0.01] {
        let mesh = crate::tessellate::tessellate_solid(&topo, solid, deflection).unwrap();
        let mut counts = std::collections::HashMap::new();
        for tri in mesh.indices.chunks_exact(3) {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                *counts.entry((a.min(b), a.max(b))).or_insert(0usize) += 1;
            }
        }
        let open = counts.values().filter(|&&n| n == 1).count();
        assert_eq!(open, 0, "open edges at deflection {deflection}");
    }
}

#[test]
fn cone_refusals_are_typed_and_leave_the_topology_unchanged() {
    type Case = (
        &'static str,
        fn(&mut Topology) -> Result<SolidId, OperationsError>,
        &'static str,
    );
    let cases: [Case; 5] = [
        (
            "a profile reaching below the apex",
            |topo| {
                let rho0 = slant(60.0);
                let corners: Vec<Point3> = [
                    (-1.0, -rho0 - 1.0),
                    (1.0, -rho0 - 1.0),
                    (1.0, 0.0),
                    (-1.0, 0.0),
                ]
                .iter()
                .map(|&(y, z)| Point3::new(20.0, y, 10.0 + z))
                .collect();
                let wire = make_polygon_wire(topo, &corners, 1e-7).unwrap();
                let profile = plane_face(topo, wire, vec![]);
                wrap_profile_on_cone(topo, profile, &cone(60.0), 0.0, 1.0)
            },
            "apex",
        ),
        (
            "a profile more than a turn round a flat cone",
            |topo| {
                let rho0 = slant(80.0);
                let corners: Vec<Point3> = [(-8.0, 0.0), (8.0, 0.0), (8.0, 1.0), (-8.0, 1.0)]
                    .iter()
                    .map(|&(y, z)| Point3::new(20.0, y * rho0 / 10.0, 10.0 + z))
                    .collect();
                let wire = make_polygon_wire(topo, &corners, 1e-7).unwrap();
                let profile = plane_face(topo, wire, vec![]);
                wrap_profile_on_cone(topo, profile, &cone(80.0), 0.0, 1.0)
            },
            "one turn",
        ),
        (
            "an inner offset past the apex",
            |topo| {
                let corners: Vec<Point3> = [(-1.0, 0.0), (1.0, 0.0), (1.0, 1.0), (-1.0, 1.0)]
                    .iter()
                    .map(|&(y, z)| Point3::new(20.0, y, 10.0 + z))
                    .collect();
                let wire = make_polygon_wire(topo, &corners, 1e-7).unwrap();
                let profile = plane_face(topo, wire, vec![]);
                wrap_profile_on_cone(topo, profile, &cone(60.0), -100.0, 1.0)
            },
            "passes the apex",
        ),
        (
            "a flat half-angle",
            |topo| {
                let corners: Vec<Point3> = [(-1.0, 0.0), (1.0, 0.0), (1.0, 1.0), (-1.0, 1.0)]
                    .iter()
                    .map(|&(y, z)| Point3::new(20.0, y, 10.0 + z))
                    .collect();
                let wire = make_polygon_wire(topo, &corners, 1e-7).unwrap();
                let profile = plane_face(topo, wire, vec![]);
                wrap_profile_on_cone(topo, profile, &cone(90.0), 0.0, 1.0)
            },
            "half-angle",
        ),
        (
            "a profile plane across the axis",
            |topo| {
                let corners: Vec<Point3> = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
                    .iter()
                    .map(|&(x, y)| Point3::new(x + 20.0, y, 12.0))
                    .collect();
                let wire = make_polygon_wire(topo, &corners, 1e-7).unwrap();
                let profile = topo.add_face(Face::new(
                    wire,
                    vec![],
                    FaceSurface::Plane { normal: Z, d: 12.0 },
                ));
                wrap_profile_on_cone(topo, profile, &cone(60.0), 0.0, 1.0)
            },
            "parallel to the cone's axis",
        ),
    ];
    for (label, build, needle) in cases {
        let mut topo = Topology::new();
        let reason = match build(&mut topo) {
            Err(OperationsError::InvalidInput { reason }) => reason,
            other => panic!("{label}: expected a typed refusal, got {other:?}"),
        };
        assert!(reason.contains(needle), "{label}: {reason}");
        assert_eq!(
            topo.num_faces(),
            1,
            "{label}: only the profile face is left"
        );
    }
}
