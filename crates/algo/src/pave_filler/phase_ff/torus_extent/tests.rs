//! Oracles for the trimmed-torus interior extent: the closed-form critical
//! points against dense carrier sampling, on random ring tori in random
//! frames and random (wrapping, full-turn and partial) rectangles.
#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use super::*;
use remus_math::vec::{Point3, Vec3};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.next(), lo)
    }

    fn unit(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            if let Ok(n) = v.normalize()
                && v.length() > 0.1
            {
                return n;
            }
        }
    }
}

fn grow(bbox: &mut Option<Aabb3>, p: Point3) {
    let point = Aabb3 { min: p, max: p };
    *bbox = Some(bbox.map_or(point, |b| b.union(point)));
}

/// Dense samples of the rectangle `[u0, u0 + su] x [v0, v0 + sv]`: the whole
/// patch, and its four sides alone (the boundary the caller samples).
fn sampled(torus: &ToroidalSurface, u: (f64, f64), v: (f64, f64), n: usize) -> (Aabb3, Aabb3) {
    let (mut patch, mut boundary) = (None, None);
    for i in 0..=n {
        for j in 0..=n {
            let p = torus.evaluate(
                u.1.mul_add(i as f64 / n as f64, u.0),
                v.1.mul_add(j as f64 / n as f64, v.0),
            );
            grow(&mut patch, p);
            if i == 0 || j == 0 || i == n || j == n {
                grow(&mut boundary, p);
            }
        }
    }
    (patch.unwrap(), boundary.unwrap())
}

fn within(inner: Aabb3, outer: Aabb3, slack: f64) -> bool {
    outer.expanded(slack).contains_point(inner.min)
        && outer.expanded(slack).contains_point(inner.max)
}

#[test]
fn rectangle_extent_is_sound_and_tight_against_dense_sampling() {
    let tol = Tolerance::default();
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let tau = std::f64::consts::TAU;
    for case in 0..300 {
        let major = rng.range(1.0, 50.0);
        let minor = major * rng.range(0.05, 0.95);
        let center = Point3::new(
            rng.range(-20.0, 20.0),
            rng.range(-20.0, 20.0),
            rng.range(-20.0, 20.0),
        );
        let torus = ToroidalSurface::with_axis(center, major, minor, rng.unit()).unwrap();
        let u = (rng.range(-tau, tau), rng.range(0.05, tau));
        // A third of the cases span the whole minor circle: a swept tube.
        let v = if case % 3 == 0 {
            (rng.range(-tau, tau), tau)
        } else {
            (rng.range(-tau, tau), rng.range(0.05, tau))
        };
        let n = 400;
        let (patch, boundary) = sampled(&torus, u, v, n);
        let extent = rectangle_extent(&torus, u, v, tol);
        let bound = extent.map_or(boundary, |e| boundary.union(e));
        // Sound: no sample of the patch lies outside boundary + extent.
        assert!(
            within(patch, bound, 1e-9 * (major + minor + 40.0)),
            "case {case}: patch {patch:?} escapes {bound:?}"
        );
        // Tight: the bound reaches no further than the patch itself, up to
        // the dense grid's chordal sampling error.
        // A coordinate's extreme lies within half a grid step of a sample in
        // each parameter; bound the miss by a whole step's chordal sag in
        // each, over the largest radius either parameter sweeps.
        let step = tau / n as f64;
        let sag = 2.0 * (major + minor) * (1.0 - step.cos()) + 1e-9;
        assert!(
            within(bound, patch, sag),
            "case {case}: bound {bound:?} looser than patch {patch:?}"
        );
    }
}

#[test]
fn swept_tube_crown_is_recovered() {
    // The #1242 tube: R 6.8955, r 1.64, axis +y, ends at u = 0 and 215.9 deg,
    // seam on the inner equator (v = pi). Its sides alone stop at
    // z = -1.4755 + 5.2555 * sin(...) well below the crown at z = 7.0600.
    let tol = Tolerance::default();
    let torus = ToroidalSurface::with_axis_and_ref_dir(
        Point3::new(6.0, 1.0, -1.475_454_545_455),
        6.895_454_545_455,
        1.64,
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(-0.951_351_351_351, 0.0, -0.308_108_108_108),
    )
    .unwrap();
    let span = 215.890_506_459_585_6_f64.to_radians();
    let tau = std::f64::consts::TAU;
    let (_, boundary) = sampled(&torus, (0.0, span), (std::f64::consts::PI, tau), 2000);
    assert!(
        boundary.max.z() < 6.0,
        "the sides alone reach {}",
        boundary.max.z()
    );
    let extent = rectangle_extent(&torus, (0.0, span), (0.0, tau), tol).unwrap();
    let crown = -1.475_454_545_455 + 6.895_454_545_455 + 1.64;
    assert!((extent.max.z() - crown).abs() < 1e-9, "{extent:?}");
    // The complementary band must not be admitted: the bottom of the full
    // torus (z = -10.0109) lies on the omitted angular band.
    assert!(extent.min.z() > -1.475_454_545_455 - 6.895_454_545_455 - 1.64 + 1.0);
}

#[test]
fn periodic_interval_membership() {
    let pi = std::f64::consts::PI;
    assert!(in_periodic_interval(0.1, (-0.2, 0.5), 0.0));
    assert!(in_periodic_interval(2.0 * pi + 0.1, (-0.2, 0.5), 0.0));
    assert!(!in_periodic_interval(0.4, (-0.2, 0.5), 0.0));
    assert!(in_periodic_interval(0.4, (-0.2, 0.5), 0.11));
    // A window wrapping through pi.
    assert!(in_periodic_interval(-pi + 0.05, (pi - 0.1, 0.3), 0.0));
    assert!(!in_periodic_interval(0.0, (pi - 0.1, 0.3), 0.0));
    // A full turn holds everything.
    assert!(in_periodic_interval(1.234, (5.0, 2.0 * pi), 0.0));
}

#[test]
fn full_turn_rectangle_is_the_whole_carrier() {
    // A rectangle spanning both periods holds every critical point, whose box
    // is the whole carrier's in any frame.
    let tol = Tolerance::default();
    let mut rng = Rng(7);
    for _ in 0..50 {
        let major = rng.range(1.0, 10.0);
        let torus = ToroidalSurface::with_axis(
            Point3::new(rng.range(-5.0, 5.0), 1.0, 2.0),
            major,
            major * rng.range(0.1, 0.9),
            rng.unit(),
        )
        .unwrap();
        let tau = std::f64::consts::TAU;
        let (full, _) = sampled(&torus, (0.0, tau), (0.0, tau), 400);
        let all = rectangle_extent(&torus, (0.0, tau), (0.0, tau), tol).unwrap();
        assert!(within(full, all, 1e-9 * (major + 10.0)));
        assert!(within(all, full, major * 1e-4));
    }
}

#[test]
fn nurbs_sided_patch_box_reaches_its_ridge_through_the_broad_phase() {
    // R 10, r 2 about +z; the patch u in [0.7, 1.2], v in [1.3, 1.9] holds
    // the top latitude ridge v = pi/2, z = 2, between its sides. Its four
    // sides are exact rational quadratic arcs stored as NURBS, so the
    // rectangle recognizer (analytic circles only) declines, and nine
    // samples per side stop at z = 1.99914720608301: a plane at z = 1.9997
    // crossing the patch was rejected by the box. The box the broad phase
    // actually builds must reach the ridge.
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    let tol = Tolerance::default();
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 2.0).unwrap();
    let (u0, u1, v0, v1) = (0.7, 1.2, 1.3, 1.9);
    // Exact rational quadratic iso-arc of the carrier between two corners.
    let arc = |(ua, va): (f64, f64), (ub, vb): (f64, f64)| {
        let (p0, pm, p2) = (
            torus.evaluate(ua, va),
            torus.evaluate(0.5 * (ua + ub), 0.5 * (va + vb)),
            torus.evaluate(ub, vb),
        );
        #[allow(clippy::float_cmp)]
        let (centre, half) = if ua == ub {
            (
                Point3::new(10.0 * ua.cos(), 10.0 * ua.sin(), 0.0),
                0.5 * (vb - va).abs(),
            )
        } else {
            (Point3::new(0.0, 0.0, 2.0 * va.sin()), 0.5 * (ub - ua).abs())
        };
        let p1 = centre + (pm - centre) * (1.0 / half.cos());
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![p0, p1, p2],
            vec![1.0, half.cos(), 1.0],
        )
        .unwrap()
    };
    let corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)];
    let mut topo = Topology::new();
    let vertices: Vec<_> = corners
        .iter()
        .map(|&(u, v)| topo.add_vertex(Vertex::new(torus.evaluate(u, v), 1e-7)))
        .collect();
    let mut oriented = Vec::new();
    for k in 0..4 {
        let curve = arc(corners[k], corners[(k + 1) % 4]);
        // Each side lies on the carrier: an exact arc, not a fit.
        for i in 0..=16 {
            let p = curve.evaluate(f64::from(i) / 16.0);
            let (u, v) = torus.project_point(p);
            assert!((torus.evaluate(u, v) - p).length() < 1e-12);
        }
        let mut edge = Edge::new(
            vertices[k],
            vertices[(k + 1) % 4],
            EdgeCurve::NurbsCurve(curve),
        );
        edge.set_trim(Some((0.0, 1.0)));
        oriented.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let wire = topo.add_wire(Wire::new(oriented, true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Torus(torus.clone()),
    ));
    assert!(
        crate::classifier::rectangular_torus_domain(&topo, face, tol).is_none(),
        "the recognizer must decline NURBS sides for this to test the fallback"
    );

    // The trap: the sides' own nine samples stay below the plane.
    let mut sides_top = f64::MIN;
    for k in 0..4 {
        let curve = arc(corners[k], corners[(k + 1) % 4]);
        for i in 0..=8 {
            sides_top = sides_top.max(curve.evaluate(f64::from(i) / 8.0).z());
        }
    }
    assert!(sides_top < 1.9997, "sides reach {sides_top}");

    let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
    assert!(
        bbox.max.z() >= 2.0 - 1e-12,
        "box stops at z = {}",
        bbox.max.z()
    );
    let plane = Aabb3 {
        min: Point3::new(-20.0, -20.0, 1.9997),
        max: Point3::new(20.0, 20.0, 1.9997),
    };
    assert!(bbox.expanded(tol.linear).intersects(plane));
}
