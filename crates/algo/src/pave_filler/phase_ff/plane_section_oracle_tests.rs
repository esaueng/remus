//! A seeded differential oracle for the certificate of a planar face's
//! section against its holes.
//!
//! Random holes — simple polygons, slivers narrower than the tolerance band,
//! polygons with a vertex within the band of the section, corners chamfered
//! by short edges, holes smaller than the separation distance, and circles —
//! against circles and ellipses of eccentricity up to 1000, at model scales
//! 1e-3, 1 and 1e6. The reference shares no code with the certificate: the
//! crossings come from the section's implicit quadratic along each edge in
//! double-double arithmetic (about 31 significant digits), and the side of
//! the hole a point of the section lies on from the polygon's winding number
//! and its distance to the edges.
//!
//! What is asserted, for every case:
//!
//! - the collector either refuses by name or returns an even number of
//!   crossings;
//! - every crossing it returns lies within the tolerance band of an exact
//!   crossing, or of a hole vertex the section passes within the band of;
//! - between consecutive returned crossings the section never lies deeper
//!   than the band on both sides of the hole, and it changes sides at every
//!   one of them: nothing the reference sees deep inside the hole is left on
//!   a span with something deep outside it;
//! - the trim keeps no point that is deeper than the band off the face and
//!   drops none that is deeper than the band on it;
//! - a case with nothing near the tolerance (crossings and vertices far
//!   apart, every excursion deep) is never refused.
//!
//! `cargo test -p remus-algo --lib plane_section_oracle -- --nocapture`
//! prints the accepted and refused counts per family and scale. With
//! `REMUS_SECTION_ORACLE_DUMP=<file>` every case's outcome is written there,
//! one line each, for comparing two revisions.

// The tallies are the point of running this with `--nocapture`.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout
)]

use std::f64::consts::{PI, TAU};
use std::fmt::Write as _;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::CylindricalSurface;
use remus_topology::face::Face;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::plane_hole_tests::{circle_wire, plane_at, polygon_wire, vertex};
use super::{
    Aabb3, AlgoError, Edge, EdgeCurve, FaceExtent, FaceId, FaceSurface, Point3, RawCurve,
    Tolerance, Topology, Vec3, plane_hole_crossings, trim_ellipse_to_boundary_crossings,
};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
const SCALES: [f64; 3] = [1e-3, 1.0, 1e6];
const CASES_PER_CELL: usize = 60;

// ── Double-double arithmetic ────────────────────────────────────────────

/// An unevaluated sum `hi + lo` with `|lo| ≤ ulp(hi) / 2`.
#[derive(Clone, Copy, Debug)]
struct Dd {
    hi: f64,
    lo: f64,
}

fn two_sum(a: f64, b: f64) -> Dd {
    let s = a + b;
    let bb = s - a;
    Dd {
        hi: s,
        lo: (a - (s - bb)) + (b - bb),
    }
}

fn two_prod(a: f64, b: f64) -> Dd {
    let p = a * b;
    Dd {
        hi: p,
        lo: a.mul_add(b, -p),
    }
}

impl Dd {
    const fn of(x: f64) -> Self {
        Self { hi: x, lo: 0.0 }
    }

    fn renorm(hi: f64, lo: f64) -> Self {
        let s = hi + lo;
        Self {
            hi: s,
            lo: lo - (s - hi),
        }
    }

    fn add(self, o: Self) -> Self {
        let s = two_sum(self.hi, o.hi);
        Self::renorm(s.hi, s.lo + self.lo + o.lo)
    }

    fn neg(self) -> Self {
        Self {
            hi: -self.hi,
            lo: -self.lo,
        }
    }

    fn sub(self, o: Self) -> Self {
        self.add(o.neg())
    }

    fn mul(self, o: Self) -> Self {
        let p = two_prod(self.hi, o.hi);
        Self::renorm(p.hi, p.lo + self.hi.mul_add(o.lo, self.lo * o.hi))
    }

    fn div(self, o: Self) -> Self {
        let q1 = self.hi / o.hi;
        let r = self.sub(o.mul(Self::of(q1)));
        let q2 = r.hi / o.hi;
        let r = r.sub(o.mul(Self::of(q2)));
        let q3 = r.hi / o.hi;
        Self::renorm(q1, q2).add(Self::of(q3))
    }

    fn sqrt(self) -> Self {
        if self.hi <= 0.0 {
            return Self::of(0.0);
        }
        let x = Self::of(self.hi.sqrt());
        // One Newton step in double-double: x + (self − x²) / (2x).
        x.add(self.sub(x.mul(x)).div(x.add(x)))
    }

    fn value(self) -> f64 {
        self.hi + self.lo
    }

    fn sign(self) -> f64 {
        if self.hi != 0.0 {
            self.hi.signum()
        } else if self.lo != 0.0 {
            self.lo.signum()
        } else {
            0.0
        }
    }
}

/// `a · b` for 2-vectors with double-double components.
fn dot_dd(a: (Dd, Dd), b: (Dd, Dd)) -> Dd {
    a.0.mul(b.0).add(a.1.mul(b.1))
}

// ── Deterministic random numbers ────────────────────────────────────────

/// `splitmix64`.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    #[allow(clippy::cast_precision_loss)]
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.unit(), lo)
    }

    /// Log-uniform between `10^lo` and `10^hi`.
    fn decade(&mut self, lo: f64, hi: f64) -> f64 {
        10.0_f64.powf(self.range(lo, hi))
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn below(&mut self, n: usize) -> usize {
        (self.unit() * n as f64) as usize % n
    }

    fn coin(&mut self) -> bool {
        self.next() & 1 == 1
    }
}

// ── Cases ───────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Family {
    Polygon,
    Sliver,
    VertexNear,
    Chamfer,
    Tiny,
    Circle,
}

const FAMILIES: [Family; 6] = [
    Family::Polygon,
    Family::Sliver,
    Family::VertexNear,
    Family::Chamfer,
    Family::Tiny,
    Family::Circle,
];

#[derive(Clone, Debug)]
enum Hole {
    Polygon(Vec<(f64, f64)>),
    Circle { centre: (f64, f64), radius: f64 },
}

/// A closed section in the plane `z = 0`: centre, semi-axes and the major
/// axis' direction, as the kernel's curve stores them.
#[derive(Clone, Debug)]
struct Section {
    curve: EdgeCurve,
    partner: FaceSurface,
    centre: (f64, f64),
    u: (f64, f64),
    v: (f64, f64),
    a: f64,
    b: f64,
}

impl Section {
    fn new(centre: (f64, f64), a: f64, b: f64, turn: f64) -> Self {
        let c = Point3::new(centre.0, centre.1, 0.0);
        let reference = Vec3::new(turn.cos(), turn.sin(), 0.0);
        let height = 4.0 * a;
        #[allow(clippy::float_cmp)]
        let (curve, partner, u, v) = if a == b {
            let circle = Circle3D::new_with_ref(c, Z, a, reference).unwrap();
            let (u, v) = (circle.u_axis(), circle.v_axis());
            let wall = CylindricalSurface::new(c + Z * -height, Z, a).unwrap();
            (EdgeCurve::Circle(circle), wall, u, v)
        } else {
            let ellipse = Ellipse3D::new_with_ref(c, Z, a, b, reference).unwrap();
            let (u, v) = (ellipse.u_axis(), ellipse.v_axis());
            // A cylinder of radius `b` tilted in the plane of `u` and Z so
            // that the plane cuts it in the ellipse (`cos θ = b / a`).
            let cos = b / a;
            let axis = u * -(1.0 - cos * cos).sqrt() + Z * cos;
            let wall = CylindricalSurface::new(c + axis * -height, axis, b).unwrap();
            (EdgeCurve::Ellipse(ellipse), wall, u, v)
        };
        Self {
            curve,
            partner: FaceSurface::Cylinder(partner),
            centre,
            u: (u.x(), u.y()),
            v: (v.x(), v.y()),
            a,
            b,
        }
    }

    fn at(&self, t: f64) -> (f64, f64) {
        let (sin, cos) = t.sin_cos();
        (
            (self.b * sin).mul_add(self.v.0, (self.a * cos).mul_add(self.u.0, self.centre.0)),
            (self.b * sin).mul_add(self.v.1, (self.a * cos).mul_add(self.u.1, self.centre.1)),
        )
    }

    /// The unit outward normal at `t`.
    fn normal(&self, t: f64) -> (f64, f64) {
        let (sin, cos) = t.sin_cos();
        let (nu, nv) = (cos / self.a, sin / self.b);
        let length = nu.hypot(nv);
        (
            nu.mul_add(self.u.0, nv * self.v.0) / length,
            nu.mul_add(self.u.1, nv * self.v.1) / length,
        )
    }

    fn tangent(&self, t: f64) -> (f64, f64) {
        let n = self.normal(t);
        (-n.1, n.0)
    }

    /// The chart angle of `p`.
    fn angle(&self, p: (f64, f64)) -> f64 {
        let w = (p.0 - self.centre.0, p.1 - self.centre.1);
        let x = w.0.mul_add(self.u.0, w.1 * self.u.1) / self.a;
        let y = w.0.mul_add(self.v.0, w.1 * self.v.1) / self.b;
        y.atan2(x).rem_euclid(TAU)
    }

    /// The distance from `p` to the section, by bisection on the chart
    /// angle around the radial foot (independent of the certificate's
    /// first-order estimate).
    fn distance(&self, p: (f64, f64)) -> f64 {
        let t0 = self.angle(p);
        let gap = |t: f64| {
            let q = self.at(t);
            (q.0 - p.0).hypot(q.1 - p.1)
        };
        let mut best = gap(t0);
        let mut centre = t0;
        let mut width = 0.5;
        for _ in 0..60 {
            for t in [centre - width, centre + width] {
                let d = gap(t);
                if d < best {
                    best = d;
                    centre = t;
                }
            }
            width *= 0.5;
        }
        best
    }

    /// The largest coordinate magnitude a crossing is computed from.
    fn coord(&self) -> f64 {
        self.centre.0.hypot(self.centre.1) + self.a
    }
}

#[derive(Clone, Debug)]
struct Case {
    family: Family,
    scale: f64,
    index: usize,
    section: Section,
    hole: Hole,
}

fn add(p: (f64, f64), d: (f64, f64), k: f64) -> (f64, f64) {
    (k.mul_add(d.0, p.0), k.mul_add(d.1, p.1))
}

fn turned(angle: f64) -> (f64, f64) {
    (angle.cos(), angle.sin())
}

fn make_case(family: Family, scale: f64, index: usize, rng: &mut Rng) -> Case {
    let s = scale;
    let band = 1e-7_f64;
    // The section: a circle, or an ellipse of ratio up to 1000.
    let ratio = if family == Family::Circle {
        1.0
    } else {
        [1.0, 1.0, 1.5, 10.0, 100.0, 1000.0][rng.below(6)]
    };
    let b = rng.range(0.5, 2.0) * s;
    let a = b * ratio;
    let centre = (rng.range(-20.0, 20.0) * s, rng.range(-20.0, 20.0) * s);
    let section = Section::new(centre, a, b, rng.range(0.0, TAU));
    // A point of the section the hole is placed at, away from nothing in
    // particular: any parameter, so the eccentric ones are met at their
    // flat sides and round their ends alike.
    let t0 = rng.range(0.0, TAU);
    let (on, normal, tangent) = (section.at(t0), section.normal(t0), section.tangent(t0));
    let hole = match family {
        Family::Polygon => {
            let centre = add(on, normal, rng.range(-0.5, 0.5) * b);
            let count = 3 + rng.below(5);
            let phase = rng.range(0.0, TAU);
            #[allow(clippy::cast_precision_loss)]
            let corners = (0..count)
                .map(|i| {
                    let angle = phase + TAU * (i as f64 + rng.range(0.1, 0.9)) / count as f64;
                    add(centre, turned(angle), rng.range(0.2, 0.8) * b)
                })
                .collect();
            Hole::Polygon(corners)
        }
        Family::Sliver => {
            // A rectangle across the section, narrower than the band up to
            // a hundred bands wide.
            let along = turned(tangent.1.atan2(tangent.0) + rng.range(0.3, PI - 0.3));
            let across = (-along.1, along.0);
            let half_length = rng.range(0.05, 0.5) * b;
            let half_width = 0.5 * rng.decade(-9.0, -5.0);
            let shift = rng.range(-0.8, 0.8) * half_length;
            let mid = add(on, along, shift);
            let corner =
                |sl: f64, sw: f64| add(add(mid, along, sl * half_length), across, sw * half_width);
            Hole::Polygon(vec![
                corner(-1.0, -1.0),
                corner(1.0, -1.0),
                corner(1.0, 1.0),
                corner(-1.0, 1.0),
            ])
        }
        Family::VertexNear => {
            // A wedge whose tip sits within a few bands of the section.
            let offset = if rng.below(6) == 0 {
                0.0
            } else {
                rng.range(-5.0, 5.0) * band
            };
            let tip = add(on, normal, offset);
            let heading = rng.range(0.0, TAU);
            let opening = rng.range(0.35, 2.8);
            let length = rng.range(0.2, 0.8) * b;
            Hole::Polygon(vec![
                tip,
                add(tip, turned(heading), length),
                add(tip, turned(heading + opening), length),
            ])
        }
        Family::Chamfer => {
            // A square with one corner cut by a short edge the section
            // passes at.
            let side = rng.range(0.3, 1.0) * b;
            let cut = rng.decade(-7.5, -4.0);
            let heading = rng.range(0.0, TAU);
            let (e1, e2) = (turned(heading), turned(heading + 0.5 * PI));
            // The chamfer's midpoint on the section, give or take its own
            // length.
            let mid = add(on, normal, rng.range(-1.0, 1.0) * cut);
            let corner = add(add(mid, e1, -0.5 * cut), e2, -0.5 * cut);
            Hole::Polygon(vec![
                add(corner, e1, cut),
                add(corner, e1, side),
                add(add(corner, e1, side), e2, side),
                add(corner, e2, side),
                add(corner, e2, cut),
            ])
        }
        Family::Tiny => {
            // A hole smaller than the distance two crossings must keep.
            let radius = rng.decade(-8.0, -5.0);
            let centre = add(on, normal, rng.range(-1.5, 1.5) * radius);
            let count = 3 + rng.below(3);
            let phase = rng.range(0.0, TAU);
            #[allow(clippy::cast_precision_loss)]
            let corners = (0..count)
                .map(|i| {
                    add(
                        centre,
                        turned(phase + TAU * i as f64 / count as f64),
                        radius,
                    )
                })
                .collect();
            Hole::Polygon(corners)
        }
        Family::Circle => {
            let radius = rng.range(0.1, 2.0) * b;
            let direction = turned(rng.range(0.0, TAU));
            // A third of the holes within a few bands of tangency, inside
            // or outside.
            let distance = match rng.below(6) {
                0 => rng.range(-5.0, 5.0).mul_add(band, a + radius),
                1 => rng.range(-5.0, 5.0).mul_add(band, (a - radius).abs()),
                _ => rng.range(0.5 * (a - radius).abs(), 1.2 * (a + radius)),
            };
            Hole::Circle {
                centre: add(centre, direction, distance.max(1e-3 * b)),
                radius,
            }
        }
    };
    let hole = match hole {
        Hole::Polygon(mut corners) => {
            if rng.coin() {
                corners.reverse();
            }
            Hole::Polygon(corners)
        }
        circle @ Hole::Circle { .. } => circle,
    };
    Case {
        family,
        scale,
        index,
        section,
        hole,
    }
}

// ── The reference ───────────────────────────────────────────────────────

/// The exact crossings of the section with the hole: the chart angle and
/// the point of each, from the implicit quadratic along every edge in
/// double-double arithmetic (a root exactly at an edge's end belongs to the
/// edge it starts).
fn reference_crossings(section: &Section, hole: &Hole) -> Vec<(f64, (f64, f64))> {
    let mut found = Vec::new();
    let (a, b) = (Dd::of(section.a), Dd::of(section.b));
    let u = (Dd::of(section.u.0), Dd::of(section.u.1));
    let v = (Dd::of(section.v.0), Dd::of(section.v.1));
    let c = (Dd::of(section.centre.0), Dd::of(section.centre.1));
    match hole {
        Hole::Polygon(corners) => {
            for (i, &from) in corners.iter().enumerate() {
                let to = corners[(i + 1) % corners.len()];
                let w = (Dd::of(from.0).sub(c.0), Dd::of(from.1).sub(c.1));
                let d = (
                    Dd::of(to.0).sub(Dd::of(from.0)),
                    Dd::of(to.1).sub(Dd::of(from.1)),
                );
                // In the chart that maps the section to the unit circle.
                let (wx, wy) = (dot_dd(w, u).div(a), dot_dd(w, v).div(b));
                let (dx, dy) = (dot_dd(d, u).div(a), dot_dd(d, v).div(b));
                let qa = dx.mul(dx).add(dy.mul(dy));
                let half_b = wx.mul(dx).add(wy.mul(dy));
                let qc = wx.mul(wx).add(wy.mul(wy)).sub(Dd::of(1.0));
                let disc = half_b.mul(half_b).sub(qa.mul(qc));
                if qa.value() <= 0.0 || disc.sign() < 0.0 {
                    continue;
                }
                let root = disc.sqrt();
                for sign in [-1.0, 1.0] {
                    let s = half_b.neg().add(root.mul(Dd::of(sign))).div(qa);
                    if s.sign() < 0.0 || s.sub(Dd::of(1.0)).sign() >= 0.0 {
                        continue;
                    }
                    let s = s.value();
                    let p = (
                        s.mul_add(to.0 - from.0, from.0),
                        s.mul_add(to.1 - from.1, from.1),
                    );
                    found.push((section.angle(p), p));
                    if disc.sign() == 0.0 {
                        break;
                    }
                }
            }
        }
        Hole::Circle { centre, radius } => {
            // Two circles: the chord's foot `along` the centre line and its
            // half-length `h`.
            let (r, rho) = (a, Dd::of(*radius));
            let e = (Dd::of(centre.0).sub(c.0), Dd::of(centre.1).sub(c.1));
            let d2 = e.0.mul(e.0).add(e.1.mul(e.1));
            let d = d2.sqrt();
            if d.value() > 0.0 {
                let along = d2.add(r.mul(r)).sub(rho.mul(rho)).div(d.add(d));
                let h2 = r.mul(r).sub(along.mul(along));
                if h2.sign() >= 0.0 {
                    let h = h2.sqrt();
                    let (ex, ey) = (e.0.div(d), e.1.div(d));
                    for sign in [-1.0, 1.0] {
                        let hs = h.mul(Dd::of(sign));
                        let p = (
                            c.0.add(along.mul(ex)).sub(hs.mul(ey)).value(),
                            c.1.add(along.mul(ey)).add(hs.mul(ex)).value(),
                        );
                        found.push((section.angle(p), p));
                        if h2.sign() == 0.0 {
                            break;
                        }
                    }
                }
            }
        }
    }
    found.sort_by(|x, y| x.0.total_cmp(&y.0));
    found
}

fn segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (ab, ap) = ((b.0 - a.0, b.1 - a.1), (p.0 - a.0, p.1 - a.1));
    let length_squared = ab.0.mul_add(ab.0, ab.1 * ab.1);
    let along = if length_squared > 0.0 {
        (ap.0.mul_add(ab.0, ap.1 * ab.1) / length_squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (ap.0 - along * ab.0).hypot(ap.1 - along * ab.1)
}

/// How deep `p` lies inside the hole: its distance from the hole's
/// boundary, negative outside.
fn depth_in_hole(p: (f64, f64), hole: &Hole) -> f64 {
    match hole {
        Hole::Polygon(corners) => {
            let mut distance = f64::INFINITY;
            let mut winding = 0.0;
            for (i, &a) in corners.iter().enumerate() {
                let b = corners[(i + 1) % corners.len()];
                distance = distance.min(segment_distance(p, a, b));
                let (da, db) = ((a.0 - p.0, a.1 - p.1), (b.0 - p.0, b.1 - p.1));
                winding +=
                    da.0.mul_add(db.1, -(da.1 * db.0))
                        .atan2(da.0.mul_add(db.0, da.1 * db.1));
            }
            if (winding / TAU).round() == 0.0 {
                -distance
            } else {
                distance
            }
        }
        Hole::Circle { centre, radius } => radius - (p.0 - centre.0).hypot(p.1 - centre.1),
    }
}

fn hole_vertices(hole: &Hole) -> Vec<(f64, f64)> {
    match hole {
        Hole::Polygon(corners) => corners.clone(),
        Hole::Circle { .. } => Vec::new(),
    }
}

/// The band the certificate works with at this section's coordinates.
fn band_of(section: &Section) -> f64 {
    1e-7_f64.max(32.0 * f64::EPSILON * section.coord())
}

/// Where the reference sees the section deeper than 1.5 bands on one side
/// of the hole: one parameter per stretch between exact crossings, with the
/// side (`true` inside).
fn deep_stretches(section: &Section, hole: &Hole, exact: &[(f64, (f64, f64))]) -> Vec<(f64, bool)> {
    let band = band_of(section);
    let mut cuts: Vec<f64> = exact.iter().map(|x| x.0).collect();
    if cuts.is_empty() {
        cuts.push(0.0);
    }
    let mut deep = Vec::new();
    for (i, &lo) in cuts.iter().enumerate() {
        let next = cuts[(i + 1) % cuts.len()];
        let hi = if next > lo { next } else { next + TAU };
        let mut best: Option<(f64, f64)> = None;
        for k in 1..=31 {
            let t = (hi - lo).mul_add(f64::from(k) / 32.0, lo);
            let depth = depth_in_hole(section.at(t), hole);
            if best.is_none_or(|(_, d)| depth.abs() > d.abs()) {
                best = Some((t, depth));
            }
        }
        if let Some((t, depth)) = best
            && depth.abs() > 1.5 * band
        {
            deep.push((t.rem_euclid(TAU), depth > 0.0));
        }
    }
    deep
}

/// Whether nothing in the case is near a tolerance: its exact crossings
/// and the hole's vertices are farther apart than four separation
/// distances, the section is ten bands deep between crossings, and no
/// straight edge runs within four bands of the cut surface where it is
/// deepest inside the section. Such a case has no reason to be refused.
fn is_clean(case: &Case, exact: &[(f64, (f64, f64))]) -> bool {
    let section = &case.section;
    let band = band_of(section);
    let far = 400.0 * band;
    let vertices = hole_vertices(&case.hole);
    let apart = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).hypot(p.1 - q.1);
    if exact.is_empty() || !exact.len().is_multiple_of(2) {
        return false;
    }
    for (i, x) in exact.iter().enumerate() {
        if exact.iter().skip(i + 1).any(|y| apart(x.1, y.1) <= far)
            || vertices.iter().any(|v| apart(x.1, *v) <= far)
        {
            return false;
        }
    }
    if vertices.iter().any(|v| section.distance(*v) <= 10.0 * band) {
        return false;
    }
    let stretches = deep_stretches(section, &case.hole, exact);
    if stretches.len() != exact.len() {
        return false;
    }
    for (t, _) in &stretches {
        if depth_in_hole(section.at(*t), &case.hole).abs() <= 10.0 * band {
            return false;
        }
    }
    // A tangent contact nowhere near a crossing is a touch, not a reason to
    // refuse, but one within reach of the grazing test is left out.
    if let (Hole::Polygon(corners), FaceSurface::Cylinder(cylinder)) =
        (&case.hole, &section.partner)
    {
        for (i, &from) in corners.iter().enumerate() {
            let to = corners[(i + 1) % corners.len()];
            for k in 0..=64 {
                let f = f64::from(k) / 64.0;
                let p = (
                    f.mul_add(to.0 - from.0, from.0),
                    f.mul_add(to.1 - from.1, from.1),
                );
                let w = Point3::new(p.0, p.1, 0.0) - cylinder.origin();
                let radial = (w - cylinder.axis() * cylinder.axis().dot(w)).length();
                let inside_section =
                    section.distance(p) > 10.0 * band && depth_of_section(section, p) > 0.0;
                if inside_section && (radial - cylinder.radius()).abs() <= 4.0 * band {
                    return false;
                }
            }
        }
    }
    true
}

/// Positive when `p` lies inside the section's curve.
fn depth_of_section(section: &Section, p: (f64, f64)) -> f64 {
    let w = (p.0 - section.centre.0, p.1 - section.centre.1);
    let x = w.0.mul_add(section.u.0, w.1 * section.u.1) / section.a;
    let y = w.0.mul_add(section.v.0, w.1 * section.v.1) / section.b;
    1.0 - x.hypot(y)
}

// ── The collector under test ────────────────────────────────────────────

fn hole_wire(topo: &mut Topology, hole: &Hole) -> WireId {
    match hole {
        Hole::Polygon(corners) => polygon_wire(topo, corners, 0.0),
        Hole::Circle { centre, radius } => {
            let circle = Circle3D::new(Point3::new(centre.0, centre.1, 0.0), Z, *radius).unwrap();
            circle_wire(topo, &circle, true)
        }
    }
}

/// The crossings the certificate reports for the case's hole.
fn collect(case: &Case) -> Result<Option<Vec<Point3>>, AlgoError> {
    let mut topo = Topology::new();
    let hole = hole_wire(&mut topo, &case.hole);
    plane_hole_crossings(
        &topo,
        hole,
        &case.section.curve,
        &case.section.partner,
        Tolerance::new(),
    )
    .map(|found| found.map(|found| found.crossings))
}

/// What became of one case at one level.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Outcome {
    /// Accepted, with this many crossings (or arcs).
    Accepted(usize),
    /// Refused by name.
    Refused(String),
    /// The hole was not read, or the trim left the pair to the generic
    /// path.
    Declined,
}

/// The name of a refusal: its message with the coordinates and ids cut
/// out, so that refusals of one kind count together.
fn refusal_kind(error: &AlgoError) -> String {
    let text = error.to_string();
    for (needle, kind) in [
        (
            "within tolerance of the cut surface",
            "edge grazes the cut surface",
        ),
        (
            "crossing(s) of the edge bound it",
            "edge chord not resolved",
        ),
        ("is a vertex of the face", "crossing beside a vertex"),
        ("from a vertex of the face", "crossing beside a vertex"),
        (
            "across the hole between them",
            "crossings closer than the separation",
        ),
        (
            "cross the hole more than once",
            "crossings closer than the separation",
        ),
        ("runs along the hole beside it", "side undecided"),
        (
            "another point it is split at",
            "crossings closer than the separation",
        ),
        ("changes sides of loop", "crossing cannot be placed"),
        ("does not alternate", "sides do not alternate"),
        ("an odd number", "odd crossing count"),
        ("could not be decided", "side undecided"),
        ("stays within the tolerance of loop", "span within the band"),
        (
            "no point farther than the tolerance",
            "span within the band",
        ),
        (
            "inside a span it would keep whole",
            "crossing inside a kept span",
        ),
        ("narrows to a point", "pinched face"),
        ("probe of the section disagrees", "probe disagrees"),
    ] {
        if text.contains(needle) {
            return kind.to_owned();
        }
    }
    panic!("a refusal without a name: {text}");
}

/// Check an accepted set of crossings against the reference; `Err` says
/// what is wrong with it.
fn check_crossings(
    case: &Case,
    found: &[Point3],
    exact: &[(f64, (f64, f64))],
) -> Result<f64, String> {
    let section = &case.section;
    let band = band_of(section);
    if !found.len().is_multiple_of(2) {
        return Err(format!("an odd number of crossings ({})", found.len()));
    }
    // Each returned crossing is an exact one, or a hole vertex the section
    // passes within the band of. Where the section crosses the hole at a
    // shallow angle the exact crossing is itself ill-conditioned (one unit
    // in the last place of a centre moves it along the two curves by far
    // more than the band), and the returned point is then held to what is
    // well-posed: it lies on both curves within the band, and the section
    // from it to an exact crossing stays within 1.5 bands of the hole's
    // boundary — the same contact, placed within its own tolerance.
    let near_vertices: Vec<(f64, f64)> = hole_vertices(&case.hole)
        .into_iter()
        .filter(|v| section.distance(*v) <= 1.01 * band)
        .collect();
    let mut worst = 0.0_f64;
    for p in found {
        let p = (p.x(), p.y());
        let apart = |q: &(f64, f64)| (p.0 - q.0).hypot(p.1 - q.1);
        let to_exact = exact
            .iter()
            .map(|x| apart(&x.1))
            .fold(f64::INFINITY, f64::min);
        let to_vertex = near_vertices
            .iter()
            .map(apart)
            .fold(f64::INFINITY, f64::min);
        if to_exact.min(to_vertex) <= band {
            if to_exact <= to_vertex {
                worst = worst.max(to_exact);
            }
            continue;
        }
        let on_both = depth_in_hole(p, &case.hole).abs() <= band && section.distance(p) <= band;
        let t = section.angle(p);
        let same_contact = exact.iter().any(|x| {
            let turn = (x.0 - t + PI).rem_euclid(TAU) - PI;
            (1..16).all(|k| {
                let q = section.at(turn.mul_add(f64::from(k) / 16.0, t));
                depth_in_hole(q, &case.hole).abs() <= 1.5 * band
            })
        });
        if !(on_both && same_contact) {
            return Err(format!(
                "crossing {p:?} is {to_exact:e} from the nearest exact crossing and \
                 {to_vertex:e} from the nearest vertex on the section, and is not a point of \
                 both curves in an exact crossing's contact"
            ));
        }
    }
    // Between consecutive returned crossings the section is deep on one
    // side of the hole at most, and the side changes at every crossing.
    let mut cuts: Vec<f64> = found
        .iter()
        .map(|p| section.angle((p.x(), p.y())))
        .collect();
    cuts.sort_by(f64::total_cmp);
    let span_of = |t: f64| {
        if cuts.is_empty() {
            0
        } else {
            (cuts.partition_point(|c| *c <= t) + cuts.len() - 1) % cuts.len()
        }
    };
    let mut sides: Vec<Option<bool>> = vec![None; cuts.len().max(1)];
    for (t, inside) in deep_stretches(section, &case.hole, exact) {
        let span = span_of(t);
        match sides[span] {
            Some(known) if known != inside => {
                return Err(format!(
                    "the span after crossing {span} is deep inside the hole and deep outside \
                     it (at parameter {t})"
                ));
            }
            _ => sides[span] = Some(inside),
        }
    }
    for (i, side) in sides.iter().enumerate() {
        if let (Some(here), Some(next)) = (side, sides[(i + 1) % sides.len()])
            && sides.len() > 1
            && *here == next
        {
            return Err(format!(
                "crossing {} does not change the side of the hole",
                (i + 1) % sides.len()
            ));
        }
    }
    Ok(worst)
}

// ── The trim under test ─────────────────────────────────────────────────

/// The rim of the wall of `surface` at `at`, on its axis.
fn wall_rim(surface: &CylindricalSurface, at: Point3) -> Circle3D {
    let axis = surface.axis();
    let reference = if axis.z().abs() < 0.9 {
        Z
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    Circle3D::new_with_ref(at, axis, surface.radius(), reference).unwrap()
}

/// Where the wall's seam meets the plane `z = 0`: a split point of the
/// section that is not the hole's.
fn seam_point(section: &Section) -> (f64, f64) {
    let FaceSurface::Cylinder(cylinder) = &section.partner else {
        panic!("a cylinder")
    };
    let foot = wall_rim(cylinder, cylinder.origin()).evaluate(0.0);
    let p = foot + cylinder.axis() * (-foot.z() / cylinder.axis().z());
    (p.x(), p.y())
}

/// A full cylinder wall about `axis` through `origin`, `height` long: two
/// rim circles joined by one seam.
fn wall(topo: &mut Topology, surface: &CylindricalSurface, height: f64) -> FaceId {
    let (origin, axis) = (surface.origin(), surface.axis());
    let rim = |at: Point3| wall_rim(surface, at);
    let (bottom, top) = (rim(origin), rim(origin + axis * height));
    let (vb, vt) = (
        vertex(topo, bottom.evaluate(0.0)),
        vertex(topo, top.evaluate(0.0)),
    );
    let mut closed = |circle: &Circle3D, v| {
        let mut edge = Edge::with_tolerance(v, v, EdgeCurve::Circle(circle.clone()), Some(1e-7));
        edge.set_trim(Some((0.0, TAU)));
        topo.add_edge(edge)
    };
    let (eb, et) = (closed(&bottom, vb), closed(&top, vt));
    let seam = topo.add_edge(Edge::new(vb, vt, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(eb, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(et, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Cylinder(surface.clone()),
    ))
}

/// The trim of the case's section on a square face that holds the whole
/// section and the hole: the parameter ranges it keeps.
fn trim(case: &Case) -> Result<Option<Vec<(f64, f64)>>, AlgoError> {
    let section = &case.section;
    let tol = Tolerance::new();
    let mut topo = Topology::new();
    let half = 4.0 * section.a + 3.0 * section.b;
    let c = section.centre;
    let outline = [
        (c.0 - half, c.1 - half),
        (c.0 + half, c.1 - half),
        (c.0 + half, c.1 + half),
        (c.0 - half, c.1 + half),
    ];
    let outer = polygon_wire(&mut topo, &outline, 0.0);
    let hole = hole_wire(&mut topo, &case.hole);
    let plane = topo.add_face(Face::new(outer, vec![hole], plane_at(0.0)));
    let FaceSurface::Cylinder(cylinder) = &section.partner else {
        panic!("a cylinder")
    };
    let height = 8.0 * section.a;
    let wall_face = wall(&mut topo, cylinder, height);
    let start = match &section.curve {
        EdgeCurve::Circle(circle) => circle.evaluate(0.0),
        EdgeCurve::Ellipse(ellipse) => ellipse.evaluate(0.0),
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => panic!("a closed section"),
    };
    let raw = RawCurve {
        bbox: Aabb3::from_points([
            Point3::new(c.0 - section.a, c.1 - section.a, 0.0),
            Point3::new(c.0 + section.a, c.1 + section.a, 0.0),
        ]),
        curve: section.curve.clone(),
        t_range: (0.0, TAU),
        p_start: start,
        p_end: start,
    };
    let plane_surface = plane_at(0.0);
    let plane_extent = FaceExtent::new(&topo, plane, &plane_surface, None, tol)
        .unwrap()
        .unwrap();
    let wall_extent = FaceExtent::new(&topo, wall_face, &section.partner, Some((0.0, height)), tol)
        .unwrap()
        .unwrap();
    trim_ellipse_to_boundary_crossings(
        &topo,
        plane,
        wall_face,
        &plane_surface,
        &section.partner,
        &raw,
        &plane_extent,
        &wall_extent,
        tol,
    )
    .map(|arcs| arcs.map(|arcs| arcs.iter().map(|arc| arc.t_range).collect()))
}

/// Check the spans a trim kept: no kept point deeper than the band inside
/// the hole, no dropped point deeper than the band on the face.
fn check_spans(
    case: &Case,
    kept: &[(f64, f64)],
    exact: &[(f64, (f64, f64))],
) -> Result<(), String> {
    let section = &case.section;
    let band = band_of(section);
    let is_kept = |t: f64| {
        kept.iter().any(|&(lo, hi)| {
            let offset = (t - lo).rem_euclid(TAU);
            offset <= hi - lo
        })
    };
    // Samples: an even spread, and one in every stretch between exact
    // crossings (so that no excursion, however short, goes unsampled).
    let mut samples: Vec<f64> = (0..720).map(|k| TAU * f64::from(k) / 720.0).collect();
    samples.extend(
        deep_stretches(section, &case.hole, exact)
            .into_iter()
            .map(|x| x.0),
    );
    for t in samples {
        let depth = depth_in_hole(section.at(t), &case.hole);
        if depth.abs() <= 1.5 * band {
            continue;
        }
        // Clear of the span ends too: an end is a crossing, where the side
        // is not the sample's to decide.
        let near_an_end = kept.iter().any(|&(lo, hi)| {
            [lo, hi].iter().any(|end| {
                let q = section.at(*end);
                let p = section.at(t);
                (p.0 - q.0).hypot(p.1 - q.1) <= 2.0 * band
            })
        });
        if near_an_end {
            continue;
        }
        if depth > 0.0 && is_kept(t) {
            return Err(format!(
                "a kept span runs {depth:e} inside the hole at parameter {t}"
            ));
        }
        if depth < 0.0 && !is_kept(t) {
            return Err(format!(
                "a point {:e} clear of the hole at parameter {t} was dropped",
                -depth
            ));
        }
    }
    Ok(())
}

// ── The run ─────────────────────────────────────────────────────────────

#[derive(Default)]
struct Tally {
    accepted: usize,
    refused: usize,
    declined: usize,
    clean: usize,
}

/// Every case, in a fixed order.
fn all_cases() -> Vec<Case> {
    let mut cases = Vec::new();
    for (f, &family) in FAMILIES.iter().enumerate() {
        for (s, &scale) in SCALES.iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let mut rng = Rng(0x5EC7_10A1_u64 ^ ((f as u64) << 32) ^ ((s as u64) << 16));
            for index in 0..CASES_PER_CELL {
                cases.push(make_case(family, scale, index, &mut rng));
            }
        }
    }
    cases
}

fn outcome_of<T>(result: &Result<Option<Vec<T>>, AlgoError>) -> Outcome {
    match result {
        Ok(Some(items)) => Outcome::Accepted(items.len()),
        Ok(None) => Outcome::Declined,
        Err(error) => Outcome::Refused(refusal_kind(error)),
    }
}

fn dump(level: &str, lines: &str) {
    if let Ok(path) = std::env::var("REMUS_SECTION_ORACLE_DUMP") {
        std::fs::write(format!("{path}.{level}"), lines).unwrap();
    }
}

fn report(title: &str, tallies: &[((Family, f64), Tally)], kinds: &[(String, usize)]) {
    let mut text = format!("\n{title}\n");
    writeln!(
        text,
        "{:<12} {:>7} {:>9} {:>8} {:>9} {:>6}",
        "family", "scale", "accepted", "refused", "declined", "clean"
    )
    .unwrap();
    for ((family, scale), tally) in tallies {
        writeln!(
            text,
            "{:<12} {:>7e} {:>9} {:>8} {:>9} {:>6}",
            format!("{family:?}"),
            scale,
            tally.accepted,
            tally.refused,
            tally.declined,
            tally.clean
        )
        .unwrap();
    }
    for (kind, count) in kinds {
        writeln!(text, "  refused, {kind}: {count}").unwrap();
    }
    println!("{text}");
}

fn bump(kinds: &mut Vec<(String, usize)>, kind: &str) {
    match kinds.iter_mut().find(|(k, _)| k == kind) {
        Some((_, count)) => *count += 1,
        None => kinds.push((kind.to_owned(), 1)),
    }
}

/// One case through the collector: what became of it, and whether that is
/// right by the reference (`Err` says what is wrong).
fn judge_collector(case: &Case) -> (Outcome, Result<f64, String>) {
    let exact = reference_crossings(&case.section, &case.hole);
    let clean = is_clean(case, &exact);
    let result = collect(case);
    let verdict = match &result {
        Ok(Some(found)) => check_crossings(case, found, &exact).and_then(|worst| {
            if clean && found.len() != exact.len() {
                Err(format!(
                    "a clean case with {} exact crossings was given {}",
                    exact.len(),
                    found.len()
                ))
            } else {
                Ok(worst)
            }
        }),
        // Only a hole the section runs along all the way round is declined,
        // and none is generated.
        Ok(None) => Err("declined".to_owned()),
        Err(error) if clean => Err(format!("a clean case was refused: {error}")),
        Err(_) => Ok(0.0),
    };
    let verdict = verdict.map_err(|why| format!("{why}\n  result {result:?}\n  exact {exact:?}"));
    (outcome_of(&result), verdict)
}

/// One case through the trim: what became of it, and whether that is right
/// by the reference.
fn judge_trim(case: &Case) -> (Outcome, Result<f64, String>) {
    let exact = reference_crossings(&case.section, &case.hole);
    // The wall's seam splits the section too: a hole crossing beside it is
    // a reason to refuse.
    let seam = seam_point(&case.section);
    let far = 400.0 * band_of(&case.section);
    let clean = is_clean(case, &exact)
        && exact
            .iter()
            .all(|x| (x.1.0 - seam.0).hypot(x.1.1 - seam.1) > far);
    let result = trim(case);
    let verdict = match &result {
        Ok(Some(kept)) => check_spans(case, kept, &exact).map(|()| 0.0),
        Ok(None) => {
            // Fewer than two split points: the section is not trimmed here
            // at all. Nothing deep inside the hole may be left.
            let inside = deep_stretches(&case.section, &case.hole, &exact)
                .iter()
                .any(|(_, inside)| *inside);
            if clean && inside {
                Err("a clean crossing was left untrimmed".to_owned())
            } else {
                Ok(0.0)
            }
        }
        Err(error) if clean => Err(format!("a clean case was refused: {error}")),
        Err(_) => Ok(0.0),
    };
    let verdict = verdict.map_err(|why| format!("{why}\n  result {result:?}\n  exact {exact:?}"));
    (outcome_of(&result), verdict)
}

/// Run every case through `judge`, tally it, and (unless the outcomes are
/// being dumped for a comparison) stop at the first wrong answer.
fn run(level: &str, title: &str, judge: fn(&Case) -> (Outcome, Result<f64, String>)) -> f64 {
    let dumping = std::env::var("REMUS_SECTION_ORACLE_DUMP").is_ok();
    let mut tallies: Vec<((Family, f64), Tally)> = Vec::new();
    let mut kinds: Vec<(String, usize)> = Vec::new();
    let mut lines = String::new();
    let mut worst = 0.0_f64;
    let mut wrong = 0_usize;
    for case in all_cases() {
        let exact = reference_crossings(&case.section, &case.hole);
        let (outcome, verdict) = judge(&case);
        let key = (case.family, case.scale);
        if tallies.last().is_none_or(|(k, _)| *k != key) {
            tallies.push((key, Tally::default()));
        }
        let tally = &mut tallies.last_mut().unwrap().1;
        tally.clean += usize::from(is_clean(&case, &exact));
        match &outcome {
            Outcome::Accepted(_) => tally.accepted += 1,
            Outcome::Declined => tally.declined += 1,
            Outcome::Refused(kind) => {
                tally.refused += 1;
                bump(&mut kinds, kind);
            }
        }
        writeln!(
            lines,
            "{level} {:?} {:e} {} {outcome:?} {}",
            case.family,
            case.scale,
            case.index,
            if verdict.is_ok() { "right" } else { "WRONG" }
        )
        .unwrap();
        match verdict {
            Ok(error) => worst = worst.max(error / band_of(&case.section)),
            Err(why) => {
                wrong += 1;
                assert!(
                    dumping,
                    "{:?} scale {:e} case {}: {:?} against {:?}\n  {why}",
                    case.family, case.scale, case.index, case.hole, case.section
                );
            }
        }
    }
    report(title, &tallies, &kinds);
    println!("wrong answers: {wrong}");
    dump(level, &lines);
    worst
}

/// The certificate's crossings of every random hole against the reference.
#[test]
fn the_collector_matches_the_reference_or_refuses() {
    let worst = run(
        "collector",
        "collector: crossings of one hole",
        judge_collector,
    );
    println!("worst accepted crossing: {worst:.3} bands from its exact one");
    // An accepted crossing that is placed by an exact one sits within the
    // band of it (a shallow one is held to its contact instead).
    assert!(worst <= 1.0, "{worst}");
}

/// The trim's kept spans of every random case against the reference.
#[test]
fn the_trim_keeps_material_and_drops_holes_or_refuses() {
    run("trim", "trim: spans kept of the section", judge_trim);
}

/// The reference itself: double-double arithmetic to its precision, and
/// closed-form crossings.
#[test]
fn the_reference_is_exact_where_a_closed_form_exists() {
    // (1 + 2^-60)² − 1 = 2^-59 + 2^-120, which a double loses entirely.
    let tiny = 2.0_f64.powi(-60);
    let x = Dd::of(1.0).add(Dd::of(tiny));
    let y = x.mul(x).sub(Dd::of(1.0));
    assert!((y.value() - 2.0 * tiny).abs() <= 4.0 * tiny * tiny);
    let root = Dd::of(2.0).sqrt();
    assert!(root.mul(root).sub(Dd::of(2.0)).value().abs() < 1e-30);
    let third = Dd::of(1.0).div(Dd::of(3.0));
    assert!(third.mul(Dd::of(3.0)).sub(Dd::of(1.0)).value().abs() < 1e-30);

    // The unit circle against the square [0.6, 2] × [−0.5, 0.5]: it enters
    // through x = 0.6 at y = ±0.8?  No: |y| ≤ 0.5 there, so it crosses
    // y = ±0.5 at x = √0.75.
    let section = Section::new((0.0, 0.0), 1.0, 1.0, 0.0);
    let hole = Hole::Polygon(vec![(0.6, -0.5), (2.0, -0.5), (2.0, 0.5), (0.6, 0.5)]);
    let exact = reference_crossings(&section, &hole);
    assert_eq!(exact.len(), 2, "{exact:?}");
    for (_, p) in &exact {
        assert!((p.0 - 0.75_f64.sqrt()).abs() < 1e-15 && (p.1.abs() - 0.5).abs() < 1e-15);
    }
    assert!((depth_in_hole((1.0, 0.0), &hole) - 0.4).abs() < 1e-15);
    assert!((depth_in_hole((0.0, 0.0), &hole) + 0.6).abs() < 1e-15);
    // Two unit-distance circles of radius 1 meet at x = 1/2, y = ±√3/2.
    let hole = Hole::Circle {
        centre: (1.0, 0.0),
        radius: 1.0,
    };
    let exact = reference_crossings(&section, &hole);
    assert_eq!(exact.len(), 2);
    for (_, p) in &exact {
        assert!((p.0 - 0.5).abs() < 1e-15 && (p.1.abs() - 0.75_f64.sqrt()).abs() < 1e-15);
    }
}
