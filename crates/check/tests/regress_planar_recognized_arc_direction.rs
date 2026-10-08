//! A planar face whose boundary carries a NURBS arc recognized as a circle or
//! an ellipse integrates the arc the edge traces, not its complement.
//!
//! The fuzz `modifier_ops` finding (2026-10-04): a box fused with a tilted
//! torus read 1457.5 by `mass_properties` against an exact 804.35. The torus
//! bit into two box faces, so their section arcs ran clockwise in the face
//! frame (one with a reversed `(1, 0)` trim): the circle arm took the
//! counter-clockwise span and integrated each meridian arc's ~2pi complement,
//! and the ellipse arm measured its span as an angle about the face's first
//! vertex rather than in the ellipse's own parameter.
//!
//! Each face here is an `L x L` square whose bottom side is replaced by a
//! conic arc, either biting into the square (clockwise) or bulging out of it
//! (counter-clockwise, minor and major arcs). Every arc is built four ways:
//! forward or reversed NURBS parameterization, and edge used forward or
//! reversed in the wire. The areas are closed forms; the chord-polygon
//! fallback misses them by ~1e-3, so the tolerance also pins the exact path.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::PI;

use remus_check::properties::face_integrator::integrate_face;
use remus_geometry::convert::curve_to_nurbs::{circle_to_nurbs, ellipse_to_nurbs};
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// Side of the square, before scaling.
const L: f64 = 2.0;

/// An in-plane placement: `origin + x * ex + y * ey`, scaled.
struct Placement {
    origin: Point3,
    ex: Vec3,
    ey: Vec3,
    scale: f64,
}

impl Placement {
    fn at(&self, x: f64, y: f64) -> Point3 {
        self.origin + self.ex * (x * self.scale) + self.ey * (y * self.scale)
    }

    fn normal(&self) -> Vec3 {
        self.ex.cross(self.ey)
    }
}

/// The bottom side's conic, in local coordinates, with the parameters of
/// `A = (0, 0)` and `B = (L, 0)` on it and the closed-form region area.
struct Arc {
    /// Builds the NURBS arc from parameter `t0` to `t1`.
    nurbs: Box<dyn Fn(f64, f64) -> NurbsCurve>,
    t_a: f64,
    t_b: f64,
    /// Area of the square with the arc replacing its bottom side.
    area: f64,
}

/// How the bottom arc's edge is stored and used.
#[derive(Clone, Copy, Debug)]
enum Variant {
    /// NURBS runs A to B; edge A to B with trim `(d0, d1)`; used forward.
    Forward,
    /// NURBS runs B to A; edge A to B with the reversed trim `(d1, d0)`.
    ReversedTrim,
    /// NURBS runs A to B; edge B to A with trim `(d1, d0)`; used reversed.
    ReversedUse,
    /// NURBS runs B to A; edge B to A with trim `(d0, d1)`; used reversed.
    ReversedBoth,
}

const VARIANTS: [Variant; 4] = [
    Variant::Forward,
    Variant::ReversedTrim,
    Variant::ReversedUse,
    Variant::ReversedBoth,
];

fn circle_arc(place: &Placement, cy: f64, major: bool) -> Arc {
    let rho = (0.25 * L * L + cy * cy).sqrt();
    let center = place.at(0.5 * L, cy);
    let circle = Circle3D::new(center, place.normal(), rho * place.scale).unwrap();
    let t_a = circle.project(place.at(0.0, 0.0));
    // Counter-clockwise from A to B, then flipped if that walks the wrong side.
    let mut t_b = t_a + (circle.project(place.at(L, 0.0)) - t_a).rem_euclid(2.0 * PI);
    // Half-angle the chord subtends at the centre, from the construction.
    let half = (0.5 * L).atan2(cy.abs());
    let minor_segment = 0.5 * rho * rho * (2.0 * half - (2.0 * half).sin());
    // Walk the arc that stays below the chord (outward) or above it (a bite).
    let below = |t: f64| (circle.evaluate(t) - place.origin).dot(place.ey) < 0.0;
    let ccw_is_below = below(0.5 * (t_a + t_b));
    let bite = cy < 0.0 && !major;
    if ccw_is_below == bite {
        t_b -= 2.0 * PI;
    }
    let area = if bite {
        L * L - minor_segment
    } else if major {
        L * L + PI * rho * rho - minor_segment
    } else {
        L * L + minor_segment
    };
    Arc {
        nurbs: Box::new(move |t0, t1| circle_to_nurbs(&circle, t0, t1).unwrap()),
        t_a,
        t_b,
        area: area * place.scale * place.scale,
    }
}

/// An ellipse biting into the square: centre `(1, -0.9)`, semi-axis 1.5
/// along local `y` and 1.25 along local `x`, through `A` and `B`.
fn ellipse_bite(place: &Placement) -> Arc {
    let (a, b) = (1.5, 1.25);
    let center = place.at(0.5 * L, -0.9);
    let ellipse = Ellipse3D::with_axes(
        center,
        place.normal(),
        a * place.scale,
        b * place.scale,
        place.ey,
        place.normal().cross(place.ey),
    )
    .unwrap();
    let t_a = ellipse.project(place.at(0.0, 0.0));
    let mut t_b = ellipse.project(place.at(L, 0.0));
    // In the unit-circle image A and B sit at +-atan2(0.8, 0.6) about `u`.
    let delta = 2.0 * 0.8_f64.atan2(0.6);
    if (t_b - t_a).abs() > PI {
        t_b += if t_b < t_a { 2.0 * PI } else { -2.0 * PI };
    }
    assert!(((t_b - t_a).abs() - delta).abs() < 1e-12);
    let segment = 0.5 * a * b * (delta - delta.sin());
    Arc {
        nurbs: Box::new(move |t0, t1| ellipse_to_nurbs(&ellipse, t0, t1).unwrap()),
        t_a,
        t_b,
        area: (L * L - segment) * place.scale * place.scale,
    }
}

/// Integrate the square-with-arc face and return `(area, volume)`.
fn integrate(place: &Placement, arc: &Arc, variant: Variant) -> (f64, f64) {
    let mut topo = Topology::new();
    let v =
        |topo: &mut Topology, x: f64, y: f64| topo.add_vertex(Vertex::new(place.at(x, y), 1e-7));
    let (va, vb, vc, vd) = (
        v(&mut topo, 0.0, 0.0),
        v(&mut topo, L, 0.0),
        v(&mut topo, L, L),
        v(&mut topo, 0.0, L),
    );
    let a_to_b = matches!(variant, Variant::Forward | Variant::ReversedUse);
    let nurbs = if a_to_b {
        (arc.nurbs)(arc.t_a, arc.t_b)
    } else {
        (arc.nurbs)(arc.t_b, arc.t_a)
    };
    let (d0, d1) = nurbs.domain();
    let (start, end, trim, forward) = match variant {
        Variant::Forward => (va, vb, (d0, d1), true),
        Variant::ReversedTrim => (va, vb, (d1, d0), true),
        Variant::ReversedUse => (vb, va, (d1, d0), false),
        Variant::ReversedBoth => (vb, va, (d0, d1), false),
    };
    let mut bottom = Edge::new(start, end, EdgeCurve::NurbsCurve(nurbs));
    bottom.set_trim(Some(trim));
    let bottom = topo.add_edge(bottom);
    let line = |topo: &mut Topology, s, e| topo.add_edge(Edge::new(s, e, EdgeCurve::Line));
    let right = line(&mut topo, vb, vc);
    let top = line(&mut topo, vc, vd);
    let left = line(&mut topo, vd, va);
    let wire = Wire::new(
        vec![
            OrientedEdge::new(bottom, forward),
            OrientedEdge::new(right, true),
            OrientedEdge::new(top, true),
            OrientedEdge::new(left, true),
        ],
        true,
    )
    .unwrap();
    let wire = topo.add_wire(wire);
    let normal = place.normal();
    let d = normal.dot(place.origin - Point3::new(0.0, 0.0, 0.0));
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }));
    let c = integrate_face(&topo, face, 8).unwrap();
    (c.area, c.volume)
}

fn placements() -> Vec<Placement> {
    let tilt = Vec3::new(0.3, -0.5, 0.81).normalize().unwrap();
    let ex = Vec3::new(1.0, 0.0, 0.0);
    let ex = (ex - tilt * ex.dot(tilt)).normalize().unwrap();
    let ey = tilt.cross(ex);
    let mut out = Vec::new();
    for scale in [1e-3, 1.0, 1e3] {
        out.push(Placement {
            origin: Point3::new(0.0, 0.0, 0.0),
            ex: Vec3::new(1.0, 0.0, 0.0),
            ey: Vec3::new(0.0, 1.0, 0.0),
            scale,
        });
        out.push(Placement {
            origin: Point3::new(37.0 * scale, -91.0 * scale, 13.0 * scale),
            ex,
            ey,
            scale,
        });
    }
    out
}

fn assert_face(label: &str, place: &Placement, arc: &Arc) {
    let d = place
        .normal()
        .dot(place.origin - Point3::new(0.0, 0.0, 0.0));
    for variant in VARIANTS {
        let (area, volume) = integrate(place, arc, variant);
        assert!(
            (area - arc.area).abs() <= arc.area * 1e-9,
            "{label} {variant:?} scale {}: area {area} vs closed form {}",
            place.scale,
            arc.area,
        );
        let expected = arc.area * d / 3.0;
        let vol_scale = (arc.area * place.scale).max(expected.abs());
        assert!(
            (volume - expected).abs() <= vol_scale * 1e-9,
            "{label} {variant:?} scale {}: volume {volume} vs closed form {expected}",
            place.scale,
        );
    }
}

#[test]
fn circle_bite_traced_clockwise_integrates_the_bitten_segment() {
    for place in placements() {
        assert_face("circle bite", &place, &circle_arc(&place, -1.5, false));
    }
}

#[test]
fn circle_bulge_minor_and_major_arcs_integrate_their_own_segment() {
    for place in placements() {
        assert_face(
            "circle minor bulge",
            &place,
            &circle_arc(&place, 1.5, false),
        );
        assert_face(
            "circle major bulge",
            &place,
            &circle_arc(&place, -0.4, true),
        );
    }
}

#[test]
fn ellipse_bite_integrates_the_bitten_segment() {
    for place in placements() {
        assert_face("ellipse bite", &place, &ellipse_bite(&place));
    }
}

/// A full circle whose NURBS parameterization crowds three quadrants into the
/// first fifth of its domain: the knot values move, the Bezier segments (and
/// so the geometry) do not.
fn skewed_full_circle(
    circle: &Circle3D,
    reverse: bool,
    stationary: bool,
    crowding: f64,
) -> NurbsCurve {
    let turn = if reverse { -2.0 * PI } else { 2.0 * PI };
    let uniform = circle_to_nurbs(circle, 0.0, turn).unwrap();
    let (d0, d1) = uniform.domain();
    let remap = |u: f64| {
        let f = (u - d0) / (d1 - d0);
        let g = if f <= 0.75 {
            f * (crowding / 0.75)
        } else {
            crowding + (f - 0.75) * ((1.0 - crowding) / 0.25)
        };
        d0 + g * (d1 - d0)
    };
    if !stationary {
        return NurbsCurve::new(
            uniform.degree(),
            uniform.knots().iter().map(|&u| remap(u)).collect(),
            uniform.control_points().to_vec(),
            uniform.weights().to_vec(),
        )
        .unwrap();
    }
    // Compose every quadratic Bezier span with t -> t^2 in homogeneous
    // coordinates. Geometry is unchanged, and the start derivative is zero.
    // The degree-four Bernstein controls are H0, H0,
    // (2 H0 + H1)/3, H1, H2.
    let mut cps = Vec::new();
    let mut weights = Vec::new();
    for seg in 0..4 {
        let i = 2 * seg;
        let a = uniform.control_points()[i];
        let b = uniform.control_points()[i + 1];
        let c = uniform.control_points()[i + 2];
        let wa = uniform.weights()[i];
        let wb = uniform.weights()[i + 1];
        let wc = uniform.weights()[i + 2];
        let wm = (2.0 * wa + wb) / 3.0;
        let m = Point3::new(
            (2.0 * wa * a.x() + wb * b.x()) / (3.0 * wm),
            (2.0 * wa * a.y() + wb * b.y()) / (3.0 * wm),
            (2.0 * wa * a.z() + wb * b.z()) / (3.0 * wm),
        );
        if seg == 0 {
            cps.push(a);
            weights.push(wa);
        }
        cps.extend([a, m, b, c]);
        weights.extend([wa, wm, wb, wc]);
    }
    let mut knots = vec![d0; 5];
    for seg in 1..4 {
        knots.extend([remap(d0 + (d1 - d0) * seg as f64 / 4.0); 4]);
    }
    knots.extend([d1; 5]);
    let composed = NurbsCurve::new(4, knots, cps, weights).unwrap();
    assert_eq!(composed.derivatives(d0, 1)[1], Vec3::new(0., 0., 0.));
    for k in 0..=256 {
        let u = d0 + (d1 - d0) * k as f64 / 256.0;
        let r = (composed.evaluate(u) - circle.center()).length();
        assert!(
            (r - circle.radius()).abs() < circle.radius() * 1e-12,
            "composition changed the exact circle: {r}"
        );
    }
    composed
}

/// A square with a slit from its right side to a circular hole: one wire that
/// walks in along the slit, round the hole clockwise on a single closed NURBS
/// edge, and back out. The hole's direction comes from that edge alone (the
/// wire's other edges fix the winding), so a full turn taken the wrong way
/// round adds the disc instead of removing it.
#[test]
fn closed_circle_in_a_keyhole_wire_turns_the_traced_way() {
    const SIDE: f64 = 4.0;
    for place in placements() {
        for (stationary, crowding) in [(false, 0.2), (true, 0.2), (true, 1e-12)] {
            for reverse in [false, true] {
                let mut topo = Topology::new();
                let circle =
                    Circle3D::new(place.at(2.0, 2.0), place.normal(), place.scale).unwrap();
                let nurbs = skewed_full_circle(&circle, reverse, stationary, crowding);
                // The discriminating premise: a quarter of the domain is already
                // past halfway round, in the direction the NURBS runs.
                let (d0, d1) = nurbs.domain();
                let quarter = circle.project(nurbs.evaluate(0.75f64.mul_add(d0, 0.25 * d1)));
                let quarter = if reverse { -quarter } else { quarter }.rem_euclid(2.0 * PI);
                assert!(quarter > PI, "skew too weak: {quarter}");

                let v = |topo: &mut Topology, x: f64, y: f64| {
                    topo.add_vertex(Vertex::new(place.at(x, y), 1e-7))
                };
                let (a, b, s, c, d) = (
                    v(&mut topo, 0.0, 0.0),
                    v(&mut topo, SIDE, 0.0),
                    v(&mut topo, SIDE, 2.0),
                    v(&mut topo, SIDE, SIDE),
                    v(&mut topo, 0.0, SIDE),
                );
                let seam = v(&mut topo, 3.0, 2.0);
                let line =
                    |topo: &mut Topology, s, e| topo.add_edge(Edge::new(s, e, EdgeCurve::Line));
                let ab = line(&mut topo, a, b);
                let bs = line(&mut topo, b, s);
                let slit = line(&mut topo, s, seam);
                let sc = line(&mut topo, s, c);
                let cd = line(&mut topo, c, d);
                let da = line(&mut topo, d, a);
                let mut hole = Edge::new(seam, seam, EdgeCurve::NurbsCurve(nurbs));
                hole.set_trim(Some((d0, d1)));
                let hole = topo.add_edge(hole);
                // The circle runs counter-clockwise unless `reverse`; the hole
                // must be walked clockwise.
                let wire = Wire::new(
                    vec![
                        OrientedEdge::new(ab, true),
                        OrientedEdge::new(bs, true),
                        OrientedEdge::new(slit, true),
                        OrientedEdge::new(hole, reverse),
                        OrientedEdge::new(slit, false),
                        OrientedEdge::new(sc, true),
                        OrientedEdge::new(cd, true),
                        OrientedEdge::new(da, true),
                    ],
                    true,
                )
                .unwrap();
                let wire = topo.add_wire(wire);
                let normal = place.normal();
                let dist = normal.dot(place.origin - Point3::new(0.0, 0.0, 0.0));
                let face = topo.add_face(Face::new(
                    wire,
                    vec![],
                    FaceSurface::Plane { normal, d: dist },
                ));
                let area = integrate_face(&topo, face, 8).unwrap().area;
                let expected = (SIDE * SIDE - PI) * place.scale * place.scale;
                assert!(
                    (area - expected).abs() <= expected * 1e-9,
                    "reverse {reverse} scale {}: area {area} vs closed form {expected}",
                    place.scale,
                );
            }
        }
    }
}
