//! Rays tangent to a torus must not flip the GFA ray-cast classifier's
//! crossing parity.
//!
//! The line–torus solver works from the line's closest point to the torus
//! centre, and there it merges a tangent line's double root into one root
//! where it once returned a close pair. The GFA classifier counted bare
//! roots and saw a tangency only as a close pair, so a ray touching the tube
//! counted one clean crossing: the point `(-1.2, -0.5, 0.5)`, just outside a
//! torus `R = 1.5, r = 0.5`, classified Inside because both horizontal
//! cardinal rays run in the tube's top tangent plane. The classifier now
//! reads qualified hits (`remus_math::ray::ray_torus`): a tangential contact
//! leaves parity unchanged and flags the ray as the close pair did.
//!
//! Points are swept along lines tangent to a whole torus and to a fillet's
//! torus band (top and bottom circles, outer and inner equator, the tube at
//! large, and points both horizontal rays touch from), nudged off the
//! tangent lines by small offsets, and checked
//! against the closed-form solid by both the GFA and the `remus-check`
//! classifiers. Points near the surface are skipped.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_algo::FaceClass;
use remus_algo::classifier::{RayCastGeoms, classify_ray_cast, ray_cast_rays_cached};
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::surfaces::ToroidalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::primitives::{make_cylinder, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// A whole torus about `z` through `centre`: one face whose boundary is a
/// single closed circle on the outer equator, the untrimmed torus.
///
/// The GFA classifier reads this face as the whole torus and the `remus-check`
/// classifier does not (it reads interior points Outside); `make_torus`'s
/// face, bounded by two degenerate line seams, is the other way round. So
/// the GFA classifier is checked on this one and `remus-check` on
/// [`made_torus`].
fn whole_torus(topo: &mut Topology, centre: Point3, big: f64, small: f64) -> SolidId {
    let surface = ToroidalSurface::new(centre, big, small).unwrap();
    let v0 = topo.add_vertex(Vertex::new(surface.evaluate(0.0, 0.0), 1e-7));
    let circle = Circle3D::new(centre, Vec3::new(0.0, 0.0, 1.0), big + small).unwrap();
    let mut edge = Edge::new(v0, v0, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, TAU)));
    let e = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(surface)));
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// `make_torus`'s whole torus about `z`, moved to `centre`.
fn made_torus(topo: &mut Topology, centre: Point3, big: f64, small: f64) -> SolidId {
    let solid = make_torus(topo, big, small, 32).unwrap();
    transform_solid(
        topo,
        solid,
        &Mat4::translation(centre.x(), centre.y(), centre.z()),
    )
    .unwrap();
    solid
}

/// Deterministic xorshift64* in `[0, 1)`.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        let bits = self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11;
        #[allow(clippy::cast_precision_loss)]
        let unit = bits as f64 / (1_u64 << 53) as f64;
        unit
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.next(), lo)
    }

    fn pick<T: Copy>(&mut self, items: &[T]) -> T {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let i = ((self.next() * items.len() as f64) as usize).min(items.len() - 1);
        items[i]
    }
}

/// The torus point at `(u, v)` (axis `z`) and its outward unit normal.
fn torus_point(centre: Point3, big: f64, small: f64, u: f64, v: f64) -> (Point3, Vec3) {
    let radial = Vec3::new(u.cos(), u.sin(), 0.0);
    let normal = radial * v.cos() + Vec3::new(0.0, 0.0, v.sin());
    let p = centre + radial * small.mul_add(v.cos(), big) + Vec3::new(0.0, 0.0, small * v.sin());
    (p, normal)
}

/// A line tangent to the torus `(big, small)` about `z` through `centre`
/// along a cardinal axis: the contact point, its outward normal and the
/// direction, drawn from one of the families whose contact a cardinal ray
/// can make.
fn tangent_line(rng: &mut Rng, centre: Point3, big: f64, small: f64) -> (Point3, Vec3, Vec3) {
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = Vec3::new(0.0, 1.0, 0.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    let (u, v, dir) = match rng.pick(&[0, 1, 2, 3, 4, 5]) {
        // Top or bottom circle: every horizontal direction is tangent.
        0 => (
            rng.range(0.0, TAU),
            rng.pick(&[FRAC_PI_2, -FRAC_PI_2]),
            rng.pick(&[x, y]),
        ),
        // Outer equator, vertical ray.
        1 => (rng.range(0.0, TAU), 0.0, z),
        // Inner equator, vertical ray.
        2 => (rng.range(0.0, TAU), PI, z),
        // Outer or inner equator, along the circumference.
        3 => {
            let v = rng.pick(&[0.0, PI]);
            if rng.next() < 0.5 {
                (rng.pick(&[FRAC_PI_2, -FRAC_PI_2]), v, x)
            } else {
                (rng.pick(&[0.0, PI]), v, y)
            }
        }
        // The tube at large, along the circumference: x at u = ±π/2, y at
        // u = 0 or π, any tube angle.
        _ => {
            let v = rng.range(0.0, TAU);
            if rng.next() < 0.5 {
                (rng.pick(&[FRAC_PI_2, -FRAC_PI_2]), v, x)
            } else {
                (rng.pick(&[0.0, PI]), v, y)
            }
        }
    };
    let (p, n) = torus_point(centre, big, small, u, v);
    (p, n, dir)
}

/// A query point on or next to the tangent line through `p` along `dir`:
/// `p + δ n − s dir`, so a ray from it along `dir` makes (or nearly makes)
/// the contact at `t = s` (behind it for `s < 0`).
fn near_tangent_point(rng: &mut Rng, p: Point3, n: Vec3, dir: Vec3, scale: f64) -> Point3 {
    let delta = rng.pick(&[0.0, 0.0, 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-7, -1e-7]) * scale;
    let s = if rng.next() < 0.8 {
        rng.range(0.05, 3.0) * scale
    } else {
        -rng.range(0.05, 3.0) * scale
    };
    p + n * delta - dir * s
}

/// A point both horizontal cardinal rays (`+x`, `+y`) touch the torus from,
/// as the reviewed point does: the section of the torus at height
/// `small sin v` holds the circle of radius `ρ = big + small cos v`, and from
/// `(−ρ, −ρ)` beside it the `+x` ray touches that circle at `(0, −ρ)` and the
/// `+y` ray at `(−ρ, 0)`, each along the circumference, so tangent to the
/// torus. Each coordinate is nudged off its tangent line by a small offset.
fn double_tangent_point(rng: &mut Rng, centre: Point3, big: f64, small: f64, v: f64) -> Point3 {
    let offsets = [0.0, 0.0, 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-7, -1e-7];
    let rho = small.mul_add(v.cos(), big);
    let (dx, dy) = (rng.pick(&offsets) * small, rng.pick(&offsets) * small);
    centre + Vec3::new(-rho + dx, -rho + dy, small * v.sin())
}

/// Whole-torus truth: the tube's signed distance.
fn torus_signed_distance(q: Point3, centre: Point3, big: f64, small: f64) -> f64 {
    let d = q - centre;
    let rho = d.x().hypot(d.y());
    (rho - big).hypot(d.z()) - small
}

/// The two solids a point is classified against: the GFA classifier reads
/// `gfa`, the `remus-check` classifier `check`. They are the same solid
/// except for the whole torus, whose two classifiers each take only one of
/// its constructions (see [`whole_torus`]).
struct Pair<'a> {
    topo: &'a Topology,
    gfa: SolidId,
    geoms: RayCastGeoms,
    check: SolidId,
}

impl<'a> Pair<'a> {
    fn new(topo: &'a Topology, gfa: SolidId, check: SolidId) -> Self {
        let geoms = RayCastGeoms::new(topo, gfa).unwrap();
        Self {
            topo,
            gfa,
            geoms,
            check,
        }
    }

    /// Classify `q` with both classifiers and compare with `inside`: the
    /// failures (empty when both agree) and whether a cardinal GFA ray
    /// reported a graze.
    fn check_point(&self, q: Point3, inside: bool) -> (Vec<String>, bool) {
        let mut failures = Vec::new();
        let gfa = classify_ray_cast(self.topo, self.gfa, q).unwrap();
        let rays = ray_cast_rays_cached(&self.geoms, q, Tolerance::default(), true);
        if (gfa == FaceClass::Inside) != inside {
            failures.push(format!(
                "gfa {q:?}: {gfa:?}, expected inside={inside}, cardinal rays {:?}",
                rays[0]
            ));
        }
        let check = classify_point(self.topo, self.check, q, &ClassifyOptions::default()).unwrap();
        let expected = if inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        if check != expected {
            failures.push(format!("check {q:?}: {check:?}, expected {expected:?}"));
        }
        (failures, rays[0].iter().any(|r| r.1))
    }
}

/// The reviewed point: both horizontal cardinal rays lie in the tube's top
/// tangent plane `z = r` and touch the top circle ahead of the point.
#[test]
fn point_beside_the_top_circle_is_outside() {
    let mut topo = Topology::default();
    let centre = Point3::new(0.0, 0.0, 0.0);
    let solid = whole_torus(&mut topo, centre, 1.5, 0.5);
    let made = made_torus(&mut topo, centre, 1.5, 0.5);
    let q = Point3::new(-1.2, -0.5, 0.5);
    let sd = torus_signed_distance(q, centre, 1.5, 0.5);
    assert!(sd > 0.038 && sd < 0.039, "{sd}");

    assert_eq!(
        classify_ray_cast(&topo, solid, q).unwrap(),
        FaceClass::Outside
    );
    let geoms = RayCastGeoms::new(&topo, solid).unwrap();
    let [cardinal, _] = ray_cast_rays_cached(&geoms, q, Tolerance::default(), true);
    // z misses the tube; x and y touch it once each, an even contact the
    // ray flags rather than counts.
    assert_eq!(cardinal, [(false, false), (false, true), (false, true)]);
    assert_eq!(
        classify_point(&topo, made, q, &ClassifyOptions::default()).unwrap(),
        PointClassification::Outside
    );
    assert_eq!(
        remus_operations::classify::classify_point(&topo, made, q, 0.01, 1e-7).unwrap(),
        remus_operations::classify::PointClassification::Outside
    );
}

/// Points along cardinal lines tangent to a whole torus, at three scales
/// and off-origin centres, against the tube's signed distance.
#[test]
fn whole_torus_tangent_ray_sweep() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let mut failures = Vec::new();
    let (mut checked, mut grazed) = (0_usize, 0_usize);
    for (scale, centre) in [
        (1.0, Point3::new(0.0, 0.0, 0.0)),
        (0.1, Point3::new(0.3, -0.2, 0.05)),
        (10.0, Point3::new(-7.0, 4.0, 2.5)),
    ] {
        let (big, small) = (1.5 * scale, 0.5 * scale);
        let mut topo = Topology::default();
        let gfa = whole_torus(&mut topo, centre, big, small);
        let check = made_torus(&mut topo, centre, big, small);
        let pair = Pair::new(&topo, gfa, check);
        for _ in 0..400 {
            let (p, n, dir) = tangent_line(&mut rng, centre, big, small);
            let q = near_tangent_point(&mut rng, p, n, dir, scale);
            let sd = torus_signed_distance(q, centre, big, small);
            if sd.abs() <= 1e-5 * scale {
                continue;
            }
            let (f, g) = pair.check_point(q, sd < 0.0);
            failures.extend(f);
            checked += 1;
            grazed += usize::from(g);
        }
        for _ in 0..100 {
            let v = rng.range(0.0, TAU);
            let q = double_tangent_point(&mut rng, centre, big, small, v);
            let sd = torus_signed_distance(q, centre, big, small);
            if sd.abs() <= 1e-5 * scale {
                continue;
            }
            let (f, g) = pair.check_point(q, sd < 0.0);
            failures.extend(f);
            checked += 1;
            grazed += usize::from(g);
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {checked} points misclassified:\n{}",
        failures.len(),
        failures[..failures.len().min(20)].join("\n")
    );
    assert!(checked > 1200, "{checked}");
    assert!(grazed * 4 > checked, "only {grazed} of {checked} grazed");
}

/// A fillet's quarter-tube band on a cylinder's top rim: the torus face is
/// bounded in both angles, so the classifier reads its bands. Lines tangent
/// to the carrier inside the band and outside it, against the closed-form
/// rounded cylinder.
#[test]
fn fillet_band_tangent_ray_sweep() {
    let (radius, height, round) = (2.0, 3.0, 0.5);
    let mut topo = Topology::default();
    let cylinder = make_cylinder(&mut topo, radius, height).unwrap();
    let rim: Vec<_> = remus_topology::explorer::solid_edges(&topo, cylinder)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            matches!(edge.curve(), EdgeCurve::Circle(c) if (c.center().z() - height).abs() < 1e-9)
        })
        .collect();
    assert_eq!(rim.len(), 1);
    let solid = fillet_cascade(&mut topo, cylinder, &rim, round)
        .unwrap()
        .solid;
    let tori = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .count();
    assert_eq!(tori, 1);

    let centre = Point3::new(0.0, 0.0, height - round);
    let big = radius - round;
    // Inside the rounded cylinder; `None` within `band` of its boundary.
    let band = 1e-5;
    let truth = |q: Point3| -> Option<bool> {
        let rho = q.x().hypot(q.y());
        let z = q.z();
        let corner = rho > big && z > height - round;
        let tube = (rho - big).hypot(z - (height - round)) - round;
        let near = z.abs() <= band
            || (z - height).abs() <= band
            || (rho - radius).abs() <= band
            || (corner && tube.abs() <= band);
        if near {
            return None;
        }
        Some(z > 0.0 && z < height && rho < radius && (!corner || tube < 0.0))
    };

    let mut rng = Rng(0xd1b5_4a32_d192_ed03);
    let pair = Pair::new(&topo, solid, solid);
    let mut failures = Vec::new();
    let (mut checked, mut grazed) = (0_usize, 0_usize);
    for _ in 0..1200 {
        let (p, n, dir) = tangent_line(&mut rng, centre, big, round);
        let q = near_tangent_point(&mut rng, p, n, dir, 1.0);
        let Some(inside) = truth(q) else {
            continue;
        };
        let (f, g) = pair.check_point(q, inside);
        failures.extend(f);
        checked += 1;
        grazed += usize::from(g);
    }
    // Both horizontal rays touching the band inside its quarter tube, and
    // touching the carrier off the face.
    for _ in 0..400 {
        let v = if rng.next() < 0.75 {
            rng.range(0.01, FRAC_PI_2 - 0.01)
        } else {
            rng.range(FRAC_PI_2, TAU)
        };
        let q = double_tangent_point(&mut rng, centre, big, round, v);
        let Some(inside) = truth(q) else {
            continue;
        };
        let (f, g) = pair.check_point(q, inside);
        failures.extend(f);
        checked += 1;
        grazed += usize::from(g);
    }
    assert!(
        failures.is_empty(),
        "{} of {checked} points misclassified:\n{}",
        failures.len(),
        failures[..failures.len().min(20)].join("\n")
    );
    assert!(checked > 1200, "{checked}");
    assert!(grazed * 4 > checked, "only {grazed} of {checked} grazed");
}
