//! The check classifier's torus crossings over every kind of line–torus
//! contact.
//!
//! `remus_math::ray::ray_torus` certifies each contact by the quartic's
//! sign. A line built to be tangent is tangent only to the rounding of its
//! construction: it misses by a hair (no hit), cuts a shallow chord (two
//! certified roots within `1e-6` rad of tangency, each unclassified) or
//! stays unresolved (one tangential or, for a triple contact, unclassified
//! window). Whatever comes back, a ray either counts with the parity of its
//! origin's side of the tube or abstains; it never miscounts. Checked on a
//! whole-torus face (two point seams, as `make_torus` builds it).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use super::count_face_ray_crossings;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// A whole-torus face bounded by its two point seams.
fn whole_torus(topo: &mut Topology, torus: &ToroidalSurface) -> FaceId {
    let v = topo.add_vertex(Vertex::new(torus.evaluate(0.0, 0.0), 1e-7));
    let a = topo.add_edge(Edge::new(v, v, EdgeCurve::Line));
    let b = topo.add_edge(Edge::new(v, v, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(a, true),
            OrientedEdge::new(b, true),
            OrientedEdge::new(a, false),
            OrientedEdge::new(b, false),
        ],
        true,
    )
    .unwrap();
    let wire = topo.add_wire(wire);
    topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(torus.clone())))
}

fn distance(t: &ToroidalSurface, p: Point3) -> f64 {
    let q = p - t.center();
    let (a, b, c) = (t.x_axis().dot(q), t.y_axis().dot(q), t.z_axis().dot(q));
    (a.hypot(b) - t.major_radius()).hypot(c) - t.minor_radius()
}

/// Exterior touches (0), interior touches (1), triple contacts (2) and
/// quadruple contacts (3), at three scales, from origins along each line on
/// both sides of the tube.
#[test]
fn torus_contacts_resolve_or_abstain_with_the_right_parity() {
    let mut resolved = [0_usize; 4];
    let mut abstained = [0_usize; 4];
    for scale in [1e-3, 1.0, 1e6] {
        let torus = ToroidalSurface::with_axis(
            Point3::new(2.0 * scale, -3.0 * scale, 4.0 * scale),
            1.5 * scale,
            0.5 * scale,
            Vec3::new(1.0, 2.0, 3.0),
        )
        .unwrap();
        let mut topo = Topology::default();
        let face = whole_torus(&mut topo, &torus);
        let (big_r, small_r) = (torus.major_radius(), torus.minor_radius());
        let size = big_r + small_r;
        let frame = |u: f64, v: f64| {
            let radial = torus.x_axis() * u.cos() + torus.y_axis() * u.sin();
            let e_u = torus.y_axis() * u.cos() - torus.x_axis() * u.sin();
            let e_v = torus.z_axis() * v.cos() - radial * v.sin();
            (torus.evaluate(u, v), e_u, e_v)
        };
        let inner = |u: f64, v: f64, ratio: f64, sign: f64| {
            let (_, e_u, e_v) = frame(u, v);
            let slope = ratio * (-small_r * v.cos() / small_r.mul_add(v.cos(), big_r)).sqrt();
            (e_u + e_v * (sign * slope)).normalize().unwrap()
        };
        // The core circle is inside: a sanity check of the face's count.
        let core = torus.center() + torus.x_axis() * big_r;
        assert_eq!(
            count_face_ray_crossings(&topo, face, core, torus.z_axis()).unwrap() % 2,
            1
        );
        let mut lines: Vec<(usize, Point3, Vec3)> = Vec::new();
        for iu in 0..12 {
            let u = f64::from(iu).mul_add(TAU / 12.0, 0.3);
            for (v, theta) in [(0.4, 0.0_f64), (-0.7, 1.2), (0.0, 0.5)] {
                let (p, e_u, e_v) = frame(u, v);
                lines.push((0, p, e_u * theta.cos() + e_v * theta.sin()));
            }
            for (v, ratio) in [(2.5, 0.0), (3.6, 0.5)] {
                lines.push((1, frame(u, v).0, inner(u, v, ratio, 1.0)));
            }
            for v in [2.2, 4.0] {
                for sign in [-1.0, 1.0] {
                    lines.push((2, frame(u, v).0, inner(u, v, 1.0, sign)));
                }
            }
            for sign in [-1.0, 1.0] {
                lines.push((3, frame(u, PI).0, inner(u, PI, 1.0, sign)));
            }
        }
        for (kind, point, dir) in lines {
            for back in [0.05, 0.3, 0.9, 1.7, 3.0, 40.0] {
                let origin = point - dir * (back * size);
                let side = distance(&torus, origin);
                if side.abs() <= 1e-6 * small_r {
                    continue;
                }
                let context = format!(
                    "kind {kind} scale {scale:e} back {back} along {dir:?}: {:?}",
                    remus_math::ray::ray_torus(origin, dir, &torus)
                );
                match count_face_ray_crossings(&topo, face, origin, dir) {
                    Ok(count) => {
                        assert_eq!(count % 2 == 1, side < 0.0, "{context}");
                        resolved[kind] += 1;
                    }
                    Err(error) => {
                        // A shallow chord's two grazing roots, or an
                        // unresolved contact, inside the face: the ray
                        // abstains and the vote loop draws another.
                        assert!(crate::classify::is_nurbs_ray_graze(&error), "{context}");
                        abstained[kind] += 1;
                    }
                }
            }
        }
    }
    // Every kind is exercised. The touches resolve on the lines the
    // rounding moved off the tube; a triple contact always holds a crossing
    // within `1e-6` rad of tangency, ahead of every origin here, so its rays
    // abstain.
    for kind in 0..4 {
        assert!(
            resolved[kind] + abstained[kind] > 100 && (kind == 2 || resolved[kind] > 20),
            "{resolved:?} {abstained:?}"
        );
    }
}

/// A contact whose window reaches the ray origin is never a clean miss:
/// from a point of exact tangency the ray abstains, since the origin lies on
/// the face; from just past the contact (the window wholly behind) it counts
/// nothing, and from just before it the touch adds no parity.
#[test]
fn a_contact_at_the_ray_origin_abstains() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap();
    let mut topo = Topology::default();
    let face = whole_torus(&mut topo, &torus);
    // Tangent lines with exactly representable contact points: the outer
    // equator along its circle, and the tube top across its circle.
    for (contact, dir) in [
        (Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
        (Point3::new(1.5, 0.0, 0.5), Vec3::new(1.0, 0.0, 0.0)),
    ] {
        let hits = remus_math::ray::ray_torus(contact, dir, &torus);
        assert!(
            hits.iter()
                .any(|h| h.t - h.spread <= 0.0 && h.t + h.spread >= 0.0),
            "{hits:?}"
        );
        let at_contact = count_face_ray_crossings(&topo, face, contact, dir);
        assert!(
            at_contact
                .as_ref()
                .is_err_and(crate::classify::is_nurbs_ray_graze),
            "{at_contact:?}"
        );
        let past = count_face_ray_crossings(&topo, face, contact + dir * 1e-3, dir).unwrap();
        let before = count_face_ray_crossings(&topo, face, contact - dir * 1e-3, dir).unwrap();
        // The tube-top line touches again at x = -1.5, behind both origins.
        assert_eq!((past % 2, before % 2), (0, 0), "{dir:?}");
    }
}

/// Review round 4 of the line–torus solve, through the face count.
///
/// Four crossings eight spacings of `t` apart on a unit torus `1e15` along
/// the ray are four crossings. Two crossings in the `f64` cells either side
/// of `t = 1500` (a tube `1.5e-7` across, a hair inside its equator) are
/// one even contact: wholly ahead and inside the face at both ends of its
/// window, it adds no crossing. They had come back as two crossings, each
/// said to be alone in a window that held both, and the ray abstained.
#[test]
fn crossings_count_by_what_the_ray_parameter_can_separate() {
    let x = Vec3::new(1.0, 0.0, 0.0);
    let start = Point3::new(0.0, 0.0, 0.0);
    let far = ToroidalSurface::new(Point3::new(1e15, 0.0, 0.0), 1.5, 0.5).unwrap();
    let mut topo = Topology::default();
    let face = whole_torus(&mut topo, &far);
    assert_eq!(count_face_ray_crossings(&topo, face, start, x).unwrap(), 4);

    // A quarter the size: each pair two spacings apart, one `f64` between
    // its crossings. Two even contacts from outside; one crossing from
    // inside the tube.
    let quarter = ToroidalSurface::new(Point3::new(1e15, 0.0, 0.0), 0.375, 0.125).unwrap();
    let mut topo = Topology::default();
    let face = whole_torus(&mut topo, &quarter);
    assert_eq!(count_face_ray_crossings(&topo, face, start, x).unwrap(), 0);
    let inside = Point3::new(1e15 + 0.375, 0.0, 0.0);
    assert_eq!(count_face_ray_crossings(&topo, face, inside, x).unwrap(), 1);

    let thin = ToroidalSurface::new(start, 0.0015, 0.000_000_15).unwrap();
    let mut topo = Topology::default();
    let face = whole_torus(&mut topo, &thin);
    let below = Point3::new(0.001_500_15, 0.0, -1500.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    let hits = remus_math::ray::ray_torus(below, z, &thin);
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(
        hits[0].kind,
        remus_math::intersect::ContactKind::Tangential,
        "{hits:?}"
    );
    assert_eq!(count_face_ray_crossings(&topo, face, below, z).unwrap(), 0);
}
