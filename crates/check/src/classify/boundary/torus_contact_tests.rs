//! The check classifier's torus crossings over every kind of line–torus
//! contact.
//!
//! `remus_math::ray::ray_torus` reports each contact once, certified by the
//! quartic's sign: a tangential contact (exterior or interior touch,
//! quadruple contact) must add no crossing and resolve, and an odd contact
//! (a triple, inflectional crossing) must either count its one crossing or
//! make the ray abstain, never miscount. Checked on a whole-torus face (two
//! point seams, as `make_torus` builds it) against the origin's side of the
//! tube.

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
                        // Only an odd contact's uncertain position may make
                        // the ray abstain; a touch always resolves.
                        assert!(crate::classify::is_nurbs_ray_graze(&error), "{context}");
                        assert_eq!(kind, 2, "{context}");
                        abstained[kind] += 1;
                    }
                }
            }
        }
    }
    assert!(
        resolved[0] > 100 && resolved[1] > 50 && resolved[3] > 50,
        "{resolved:?}"
    );
    // The triple contact is never miscounted: it abstains (or counts once).
    assert!(
        resolved[2] + abstained[2] > 100,
        "{resolved:?} {abstained:?}"
    );
}
