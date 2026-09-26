//! Independent-oracle tests for the phase-FF exact-vs-refuse helpers.
//!
//! Each helper here decides whether a section survives, is dropped, or is
//! trimmed to an exact arc. The expected answers come from closed forms that
//! do not share code with the helper under test: point-to-segment distance for
//! the disc-cap clip, circle–circle intersection for the torus oval trim, and
//! hand-placed points for the graze and single-rim notch tests.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_1_SQRT_2, TAU};

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_topology::face::Face;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{
    Aabb3, Edge, EdgeCurve, FaceClip, FaceExtent, FaceId, FaceSurface, Point3, RawCurve, Tolerance,
    Topology, Vec3, Vertex, clip_line_to_face, section_notches_one_rim, single_hit_circle_is_graze,
    trim_torus_oval_to_box_face,
};

// ── Shared fixture builders ─────────────────────────────────────────────

/// An orthonormal in-plane basis built without the kernel's `PlaneFrame`.
fn plane_basis(normal: Vec3) -> (Vec3, Vec3) {
    let n = normal.normalize().unwrap();
    let seed = if n.z().abs() < 0.9 {
        Vec3::new(0.0, 0.0, 1.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let u = seed.cross(n).normalize().unwrap();
    (u, n.cross(u))
}

fn plane_surface(normal: Vec3, through: Point3) -> FaceSurface {
    let n = normal.normalize().unwrap();
    FaceSurface::Plane {
        normal: n,
        d: n.dot(through - Point3::new(0.0, 0.0, 0.0)),
    }
}

fn vertex(topo: &mut Topology, p: Point3) -> remus_topology::vertex::VertexId {
    topo.add_vertex(Vertex::new(p, 1e-7))
}

/// A circle edge over `[t0, t1]` between existing vertices.
fn arc_edge(
    topo: &mut Topology,
    circle: &Circle3D,
    (t0, t1): (f64, f64),
    start: remus_topology::vertex::VertexId,
    end: remus_topology::vertex::VertexId,
) -> remus_topology::edge::EdgeId {
    let mut edge = Edge::with_tolerance(start, end, EdgeCurve::Circle(circle.clone()), Some(1e-7));
    edge.set_trim(Some((t0, t1)));
    topo.add_edge(edge)
}

/// A full circle as one closed edge whose seam sits at parameter `seam`.
fn closed_circle_wire(
    topo: &mut Topology,
    circle: &Circle3D,
    seam: f64,
) -> remus_topology::wire::WireId {
    let v = vertex(topo, circle.evaluate(seam));
    let e = arc_edge(topo, circle, (seam, seam + TAU), v, v);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap())
}

fn line_edge(
    topo: &mut Topology,
    start: remus_topology::vertex::VertexId,
    end: remus_topology::vertex::VertexId,
) -> remus_topology::edge::EdgeId {
    topo.add_edge(Edge::new(start, end, EdgeCurve::Line))
}

/// A planar polygon face over the given corners (counter-clockwise about
/// `normal`).
fn polygon_face(topo: &mut Topology, corners: &[Point3], normal: Vec3) -> FaceId {
    let verts: Vec<_> = corners.iter().map(|&p| vertex(topo, p)).collect();
    let edges: Vec<_> = (0..verts.len())
        .map(|i| {
            let e = line_edge(topo, verts[i], verts[(i + 1) % verts.len()]);
            OrientedEdge::new(e, true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(wire, vec![], plane_surface(normal, corners[0])))
}

fn raw_line(p: Point3, q: Point3) -> RawCurve {
    RawCurve {
        curve: EdgeCurve::Line,
        bbox: Aabb3::from_points([p, q]),
        t_range: (0.0, 1.0),
        p_start: p,
        p_end: q,
    }
}

/// Distance from `c` to the closed segment `[p, q]` (clamped projection).
fn segment_distance(c: Point3, p: Point3, q: Point3) -> f64 {
    let d = q - p;
    let len2 = d.dot(d);
    let t = if len2 > 0.0 {
        ((c - p).dot(d) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p + d * t - c).length()
}

/// Deterministic uniform draws in `[lo, hi)` (64-bit LCG, top 53 bits).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self, lo: f64, hi: f64) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        #[allow(clippy::cast_precision_loss)]
        let unit = (self.0 >> 11) as f64 / (1_u64 << 53) as f64;
        lo + (hi - lo) * unit
    }
}

// ── clip_line_to_face: B32 disc-cap miss test ───────────────────────────

struct Disc {
    topo: Topology,
    face: FaceId,
    center: Point3,
    radius: f64,
    u: Vec3,
    v: Vec3,
}

impl Disc {
    fn new(center: Point3, normal: Vec3, radius: f64, seam: f64) -> Self {
        let mut topo = Topology::new();
        let circle = Circle3D::new(center, normal, radius).unwrap();
        let wire = closed_circle_wire(&mut topo, &circle, seam);
        let face = topo.add_face(Face::new(wire, vec![], plane_surface(normal, center)));
        let (u, v) = plane_basis(normal);
        Self {
            topo,
            face,
            center,
            radius,
            u,
            v,
        }
    }

    fn at(&self, x: f64, y: f64) -> Point3 {
        self.center + self.u * x + self.v * y
    }

    /// `true` = the clip keeps the line (`Indeterminate`), `false` = it
    /// drops it (`Empty`). A disc cap never yields a trimmed `Range`.
    fn keeps(&self, p: Point3, q: Point3) -> bool {
        match clip_line_to_face(&self.topo, self.face, &raw_line(p, q), Tolerance::new()) {
            FaceClip::Indeterminate => true,
            FaceClip::Empty => false,
            FaceClip::Range(range) => panic!("disc cap returned a trimmed range {range:?}"),
        }
    }

    /// The exact oracle: a segment touches the disc iff its distance to the
    /// center is within the radius plus the linear tolerance.
    fn expected(&self, p: Point3, q: Point3) -> bool {
        segment_distance(self.center, p, q) <= self.radius + Tolerance::new().linear
    }
}

fn tilted_disc() -> Disc {
    Disc::new(
        Point3::new(0.3, -0.2, 0.5),
        Vec3::new(1.0, 2.0, 3.0),
        1.7,
        0.4,
    )
}

#[test]
fn disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch() {
    let disc = tilted_disc();
    let r = disc.radius;
    let tol = Tolerance::new().linear;
    // (from, to, touches the disc)
    let cases = [
        // A line passing wide of the disc.
        ((-3.0, 2.2), (3.0, 2.2), false),
        // A chord straight through.
        ((-3.0, 0.4), (3.0, 0.4), true),
        // On a line through the disc, but the segment stops short of it.
        ((-6.0, 0.4), (-2.5, 0.4), false),
        ((-2.5, 0.4), (-6.0, 0.4), false),
        // ... or starts past it.
        ((2.5, 0.4), (6.0, 0.4), false),
        ((6.0, -0.4), (2.5, -0.4), false),
        // Starting inside, ending outside; and wholly inside.
        ((0.2, 0.1), (5.0, 1.0), true),
        ((5.0, 1.0), (0.2, 0.1), true),
        ((-0.5, 0.3), (0.6, -0.2), true),
        // Tangent grazes: half a tolerance outside stays, three drop.
        ((-3.0, r + 0.5 * tol), (3.0, r + 0.5 * tol), true),
        ((-3.0, r + 3.0 * tol), (3.0, r + 3.0 * tol), false),
        ((-3.0, -r - 0.5 * tol), (3.0, -r - 0.5 * tol), true),
        ((-3.0, -r - 3.0 * tol), (3.0, -r - 3.0 * tol), false),
        // A segment ending exactly on the tolerance band, radially.
        ((0.0, 4.0), (0.0, r + 0.5 * tol), true),
        ((0.0, 4.0), (0.0, r + 3.0 * tol), false),
    ];
    for ((x0, y0), (x1, y1), touches) in cases {
        let (p, q) = (disc.at(x0, y0), disc.at(x1, y1));
        assert_eq!(
            disc.expected(p, q),
            touches,
            "oracle self-check at {x0},{y0}"
        );
        assert_eq!(
            disc.keeps(p, q),
            touches,
            "segment ({x0}, {y0}) -> ({x1}, {y1}) against r = {r}"
        );
    }
}

#[test]
fn disc_clip_decides_point_segments_by_distance_to_the_center() {
    let disc = tilted_disc();
    let r = disc.radius;
    let tol = Tolerance::new().linear;
    for k in 0..12 {
        let angle = f64::from(k) * TAU / 12.0 + 0.1;
        let (c, s) = (angle.cos(), angle.sin());
        for (radius, touches) in [
            (0.0, true),
            (0.66 * r, true),
            (r + 0.5 * tol, true),
            (r + 3.0 * tol, false),
            (1.3 * r, false),
        ] {
            let p = disc.at(radius * c, radius * s);
            assert_eq!(
                disc.keeps(p, p),
                touches,
                "point at radius {radius} angle {angle} (disc r = {r})"
            );
        }
    }
}

#[test]
fn disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point() {
    // A disc about the origin in z = 0. Its plane frame is then axis-aligned
    // and exact, so the touch points below sit on the tolerance circle
    // (radius r + linear tolerance) with no rounding at all.
    let r = 1.7;
    let r_tol = r + Tolerance::new().linear;
    let disc = Disc::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), r, 0.4);
    let touch = Point3::new(-r_tol, 0.0, 0.0);
    // (from, to): each segment meets the tolerance circle only at `touch`.
    let cases = [
        // Tangent there.
        (touch, Point3::new(-r_tol, 2.0, 0.0)),
        (Point3::new(-r_tol, -2.0, 0.0), touch),
        // Leaving radially outward from it.
        (touch, Point3::new(-r_tol - 1.0, 0.0, 0.0)),
        (
            Point3::new(0.0, r_tol, 0.0),
            Point3::new(0.0, r_tol + 3.0, 0.0),
        ),
    ];
    for (p, q) in cases {
        assert!(
            (segment_distance(disc.center, p, q) - r_tol).abs() <= 0.0,
            "fixture: {p:?} -> {q:?} touches the tolerance circle exactly"
        );
        assert!(
            disc.keeps(p, q),
            "a touch at exactly r + tolerance is kept: {p:?} -> {q:?}"
        );
    }
}

#[test]
fn disc_clip_matches_the_distance_oracle_on_generated_segments() {
    for (disc, span, seed) in [
        (tilted_disc(), 4.0, 0x5eed_0001_u64),
        // Far from the origin, large, and tilted the other way: every
        // quantity in the quadratic changes scale.
        (
            Disc::new(
                Point3::new(1.0e3, -500.0, 40.0),
                Vec3::new(-0.3, 0.1, 1.0),
                250.0,
                -2.0,
            ),
            600.0,
            0x5eed_0002,
        ),
        // Small and off-origin.
        (
            Disc::new(
                Point3::new(-0.02, 0.013, 0.007),
                Vec3::new(0.0, 1.0, 0.2),
                3.0e-3,
                1.0,
            ),
            8.0e-3,
            0x5eed_0003,
        ),
    ] {
        let mut rng = Lcg(seed);
        let (mut kept, mut dropped) = (0, 0);
        for _ in 0..400 {
            let p = disc.at(rng.next(-span, span), rng.next(-span, span));
            let q = disc.at(rng.next(-span, span), rng.next(-span, span));
            let d = segment_distance(disc.center, p, q);
            if (d - disc.radius).abs() < 1e-6 * disc.radius {
                continue; // too close to call against the tolerance band
            }
            let expected = disc.expected(p, q);
            assert_eq!(
                disc.keeps(p, q),
                expected,
                "segment {p:?} -> {q:?}: distance {d} vs radius {}",
                disc.radius
            );
            if expected {
                kept += 1;
            } else {
                dropped += 1;
            }
        }
        assert!(
            kept > 50 && dropped > 50,
            "sweep too one-sided: {kept} kept, {dropped} dropped"
        );
    }
}

// ── single_hit_circle_is_graze: B33 one-hit veto ────────────────────────

/// A cylinder lateral face with no `v_range` has no extent, so it abstains;
/// pairing it with the plane isolates the plane's decision.
fn abstaining_face(topo: &mut Topology) -> (FaceId, FaceSurface) {
    let surface = FaceSurface::Cylinder(
        CylindricalSurface::new(Point3::new(9.0, 9.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap(),
    );
    let circle = Circle3D::new(Point3::new(9.0, 9.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
    let wire = closed_circle_wire(topo, &circle, 0.0);
    (
        topo.add_face(Face::new(wire, vec![], surface.clone())),
        surface,
    )
}

fn rectangle(topo: &mut Topology, x: (f64, f64), y: (f64, f64)) -> (FaceId, FaceSurface) {
    let corners = [
        Point3::new(x.0, y.0, 0.0),
        Point3::new(x.1, y.0, 0.0),
        Point3::new(x.1, y.1, 0.0),
        Point3::new(x.0, y.1, 0.0),
    ];
    let face = polygon_face(topo, &corners, Vec3::new(0.0, 0.0, 1.0));
    let surface = topo.face(face).unwrap().surface().clone();
    (face, surface)
}

/// Signed distance of `p` outside the axis-aligned rectangle (negative inside).
fn outside_distance(p: Point3, x: (f64, f64), y: (f64, f64)) -> f64 {
    let dx = (x.0 - p.x()).max(p.x() - x.1);
    let dy = (y.0 - p.y()).max(p.y() - y.1);
    if dx <= 0.0 && dy <= 0.0 {
        dx.max(dy)
    } else {
        dx.max(0.0).hypot(dy.max(0.0))
    }
}

fn graze(
    topo: &Topology,
    plane: (FaceId, &FaceSurface),
    other: (FaceId, &FaceSurface),
    circle: &Circle3D,
    t_hit: f64,
) -> bool {
    single_hit_circle_is_graze(
        topo,
        [(plane.0, plane.1, None), (other.0, other.1, None)],
        circle,
        t_hit,
        Tolerance::new(),
    )
    .unwrap()
}

#[test]
fn one_hit_circle_is_a_graze_exactly_when_its_antipode_is_clearly_outside() {
    let mut topo = Topology::new();
    let (xr, yr) = ((0.0, 2.0), (0.0, 2.0));
    let (plane, plane_surface) = rectangle(&mut topo, xr, yr);
    let (other, other_surface) = abstaining_face(&mut topo);
    // Straddling the y = 0 edge, so antipodes land on both sides of it.
    let center = Point3::new(1.0, 0.1, 0.0);
    let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 0.6).unwrap();
    let (mut grazes, mut keeps) = (0, 0);
    for k in 0..24 {
        let t_hit = -3.0 + f64::from(k) * 0.29;
        let hit = circle.evaluate(t_hit);
        let antipode = center + (center - hit);
        let outside = outside_distance(antipode, xr, yr);
        // Decisive only far from the boundary band (the extent's 1% margin
        // is an implementation allowance; the oracle stays clear of it).
        if outside > -0.05 && outside < 0.05 {
            continue;
        }
        let expected = outside > 0.0;
        assert_eq!(
            graze(
                &topo,
                (plane, &plane_surface),
                (other, &other_surface),
                &circle,
                t_hit
            ),
            expected,
            "t_hit = {t_hit}: antipode {antipode:?} is {outside} outside"
        );
        if expected {
            grazes += 1;
        } else {
            keeps += 1;
        }
    }
    assert!(grazes >= 4 && keeps >= 4, "{grazes} grazes, {keeps} keeps");
    // The same answers with the pair in the other order.
    let t_hit = circle.project(Point3::new(1.0, 0.7, 0.0));
    assert!(graze(
        &topo,
        (other, &other_surface),
        (plane, &plane_surface),
        &circle,
        t_hit
    ));
}

#[test]
fn one_hit_circle_just_outside_within_the_boundary_margin_is_kept() {
    let mut topo = Topology::new();
    let (plane, plane_surface) = rectangle(&mut topo, (0.0, 2.0), (0.0, 2.0));
    let (other, other_surface) = abstaining_face(&mut topo);
    // Antipode 0.005 below the y = 0 edge: outside the polygon, but inside
    // the face's documented boundary allowance, so not decisive.
    let center = Point3::new(1.0, 0.495, 0.0);
    let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
    let t_hit = circle.project(Point3::new(1.0, 0.995, 0.0));
    assert!(!graze(
        &topo,
        (plane, &plane_surface),
        (other, &other_surface),
        &circle,
        t_hit
    ));
}

#[test]
fn one_hit_circle_on_a_sliver_face_honors_the_weld_band_not_just_the_margin() {
    // A 1e-4-wide sliver: its proportional margin (1e-6) is tighter than the
    // weld band (100 × linear tolerance = 1e-5), so the band decides.
    let mut topo = Topology::new();
    let (plane, plane_surface) = rectangle(&mut topo, (0.0, 1.0e-4), (0.0, 1.0));
    let (other, other_surface) = abstaining_face(&mut topo);
    let normal = Vec3::new(0.0, 0.0, 1.0);
    for (outside, expected_graze) in [(5.0e-6, false), (5.0e-5, true)] {
        // Antipode at x = -outside, y = 0.5; the hit is on the far side.
        let radius = 0.3;
        let center = Point3::new(radius - outside, 0.5, 0.0);
        let circle = Circle3D::new(center, normal, radius).unwrap();
        let t_hit = circle.project(Point3::new(2.0 * radius - outside, 0.5, 0.0));
        assert_eq!(
            graze(
                &topo,
                (plane, &plane_surface),
                (other, &other_surface),
                &circle,
                t_hit
            ),
            expected_graze,
            "antipode {outside} outside a sliver face"
        );
    }
}

// ── section_notches_one_rim: B39 single-rim lens ────────────────────────

#[test]
fn a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound() {
    let axis_origin = Point3::new(0.4, -1.1, 0.25);
    let wall = FaceSurface::Cylinder(
        CylindricalSurface::new(axis_origin, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap(),
    );
    let torus =
        FaceSurface::Torus(ToroidalSurface::new(Point3::new(3.0, 0.0, 1.0), 2.0, 0.5).unwrap());
    let plane = plane_surface(Vec3::new(0.0, 0.0, 1.0), axis_origin);
    // The wall's axial band, in the cylinder's own v (height above origin).
    let band = (0.75, 2.5);
    let on_wall =
        |angle: f64, v: f64| axis_origin + Vec3::new(2.0 * angle.cos(), 2.0 * angle.sin(), v);
    let weld = Tolerance::new().linear * 100.0;
    // (start v, end v, notches one rim)
    let cases = [
        (band.0, band.0, true),
        (band.1, band.1, true),
        (band.0, band.1, false),
        (band.1, band.0, false),
        (1.4, 1.9, false),
        (band.0, 1.4, false),
        (1.4, band.1, false),
        (band.0 + 0.5 * weld, band.0 - 0.5 * weld, true),
        (band.1 - 0.5 * weld, band.1, true),
        (band.0 + 3.0 * weld, band.0, false),
        (band.1, band.1 - 3.0 * weld, false),
    ];
    for (vs, ve, expected) in cases {
        let raw = raw_line(on_wall(0.3, vs), on_wall(2.1, ve));
        let tol = Tolerance::new();
        assert_eq!(
            section_notches_one_rim(&wall, &torus, Some(band), None, &raw, tol),
            expected,
            "wall first: v {vs} -> {ve}"
        );
        assert_eq!(
            section_notches_one_rim(&torus, &wall, None, Some(band), &raw, tol),
            expected,
            "wall second: v {vs} -> {ve}"
        );
        // Only a torus against a frustum/cylinder wall is this lens.
        assert!(!section_notches_one_rim(
            &wall,
            &plane,
            Some(band),
            None,
            &raw,
            tol
        ));
        assert!(!section_notches_one_rim(
            &plane,
            &torus,
            Some(band),
            None,
            &raw,
            tol
        ));
        // No band on the wall side: nothing to notch.
        assert!(!section_notches_one_rim(
            &wall,
            &torus,
            None,
            Some(band),
            &raw,
            tol
        ));
    }
}

// ── trim_torus_oval_to_box_face: B39 disk cap and straight box ─────────

/// A torus off the origin, cut by a plane half a unit above its center: the
/// section is two exact circles about the torus axis, radii
/// `R ± sqrt(r² − h²)`.
struct TorusCut {
    torus: ToroidalSurface,
    plane_point: Point3,
    axis_foot: Point3,
    outer: f64,
    inner: f64,
    /// Start of the oval's knot domain (its length is always 4).
    domain_start: f64,
}

impl TorusCut {
    fn new() -> Self {
        let center = Point3::new(0.7, -0.3, 0.2);
        let (major, minor, h): (f64, f64, f64) = (3.0, 1.25, 0.5);
        let half = minor.mul_add(minor, -h * h).sqrt();
        let axis_foot = center + Vec3::new(0.0, 0.0, h);
        Self {
            torus: ToroidalSurface::new(center, major, minor).unwrap(),
            plane_point: axis_foot,
            axis_foot,
            outer: major + half,
            inner: major - half,
            domain_start: 1.0,
        }
    }

    /// The same cut with the oval's parameter domain starting at `t0`.
    const fn with_domain_start(mut self, t0: f64) -> Self {
        self.domain_start = t0;
        self
    }

    fn in_plane(&self, x: f64, y: f64) -> Point3 {
        self.axis_foot + Vec3::new(x, y, 0.0)
    }

    /// The outer section circle as an exact rational NURBS over
    /// `[domain_start, domain_start + 4]`,
    /// shaped like the marcher's closed oval (a non-`Line`, non-`Circle`
    /// closed curve with a domain that does not start at zero).
    fn outer_oval(&self) -> RawCurve {
        let w = FRAC_1_SQRT_2;
        let dirs = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (-1.0, 1.0),
            (-1.0, 0.0),
            (-1.0, -1.0),
            (0.0, -1.0),
            (1.0, -1.0),
            (1.0, 0.0),
        ];
        let points: Vec<Point3> = dirs
            .iter()
            .map(|&(x, y)| self.in_plane(self.outer * x, self.outer * y))
            .collect();
        let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let t0 = self.domain_start;
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0]
            .map(|k| k + t0)
            .to_vec();
        let curve = NurbsCurve::new(2, knots, points.clone(), weights).unwrap();
        RawCurve {
            curve: EdgeCurve::NurbsCurve(curve),
            bbox: Aabb3::from_points(points.iter().copied()),
            t_range: (t0, t0 + 4.0),
            p_start: points[0],
            p_end: points[0],
        }
    }

    fn plane(&self) -> FaceSurface {
        plane_surface(Vec3::new(0.0, 0.0, 1.0), self.plane_point)
    }

    /// Run the trim with the plane face as `fa` and a stand-in torus face.
    fn trim(&self, topo: &mut Topology, plane_face: FaceId) -> Option<Vec<RawCurve>> {
        let torus_surface = FaceSurface::Torus(self.torus.clone());
        let circle = Circle3D::new(self.axis_foot, Vec3::new(0.0, 0.0, 1.0), 0.1).unwrap();
        let torus_wire = closed_circle_wire(topo, &circle, 0.0);
        let torus_face = topo.add_face(Face::new(torus_wire, vec![], torus_surface.clone()));
        let plane = self.plane();
        let tol = Tolerance::new();
        let ext = FaceExtent::new(topo, plane_face, &plane, None, tol)
            .unwrap()
            .expect("a planar face has an extent");
        trim_torus_oval_to_box_face(
            topo,
            plane_face,
            torus_face,
            &plane,
            &torus_surface,
            &self.outer_oval(),
            &ext,
            &ext,
            tol,
        )
    }
}

/// Circle(center `a`, radius `ra`) ∩ circle(center `b`, radius `rb`) in the
/// z = const plane of `a`, by the closed form along and across `a→b`.
fn circle_circle(a: Point3, ra: f64, b: Point3, rb: f64) -> [Point3; 2] {
    let ab = b - a;
    let d = ab.length();
    let along = (d * d + ra * ra - rb * rb) / (2.0 * d);
    let across = ra.mul_add(ra, -along * along).sqrt();
    let e = ab * (1.0 / d);
    let n = Vec3::new(-e.y(), e.x(), 0.0);
    let base = a + e * along;
    [base + n * across, base - n * across]
}

/// The trimmed pieces must retrace one arc of the outer section circle
/// between `ends`, stay inside `inside`, and cover `expected_angle` radians.
fn assert_kept_arc(
    cut: &TorusCut,
    pieces: &[RawCurve],
    ends: [Point3; 2],
    expected_angle: f64,
    inside: &dyn Fn(Point3) -> bool,
    label: &str,
) {
    assert!(!pieces.is_empty(), "{label}: no pieces");
    // Chain the pieces end to end.
    let first = pieces[0].p_start;
    let last = pieces[pieces.len() - 1].p_end;
    for pair in pieces.windows(2) {
        assert!(
            (pair[0].p_end - pair[1].p_start).length() < 1e-9,
            "{label}: pieces do not chain"
        );
    }
    let matches = |a: Point3, b: Point3| (a - b).length() < 1e-8;
    assert!(
        (matches(first, ends[0]) && matches(last, ends[1]))
            || (matches(first, ends[1]) && matches(last, ends[0])),
        "{label}: arc ends {first:?} .. {last:?}, expected the exact crossings {ends:?}"
    );
    let mut angle = 0.0;
    for piece in pieces {
        let mut prev: Option<Point3> = None;
        for i in 0..=64 {
            let t = piece.t_range.0 + (piece.t_range.1 - piece.t_range.0) * f64::from(i) / 64.0;
            let p = piece
                .curve
                .evaluate_with_endpoints(t, piece.p_start, piece.p_end);
            let radial = Vec3::new(p.x() - cut.axis_foot.x(), p.y() - cut.axis_foot.y(), 0.0);
            assert!(
                (radial.length() - cut.outer).abs() < 1e-6,
                "{label}: sample off the outer section circle by {}",
                radial.length() - cut.outer
            );
            assert!(
                (p.z() - cut.axis_foot.z()).abs() < 1e-6,
                "{label}: sample off the plane"
            );
            assert!(
                inside(p),
                "{label}: kept sample {p:?} lies outside the face"
            );
            if let Some(q) = prev {
                let a = Vec3::new(q.x() - cut.axis_foot.x(), q.y() - cut.axis_foot.y(), 0.0);
                angle += a.cross(radial).z().atan2(a.dot(radial)).abs();
            }
            prev = Some(p);
        }
    }
    assert!(
        (angle - expected_angle).abs() < 1e-6,
        "{label}: kept arc spans {angle} rad, the in-face arc spans {expected_angle}"
    );
}

/// A cap disc on the cut plane centered 3 units out along +x, radius 1.5:
/// it crosses both the outer (kept) and inner (other-branch) section circles.
fn cap_center_and_radius(cut: &TorusCut) -> (Point3, f64) {
    (cut.in_plane(3.0, 0.0), 1.5)
}

#[test]
fn torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam() {
    // Two caps: one crossing the other section branch far outside the kept
    // arc's angular span, one crossing it well inside that span (so an
    // off-branch crossing would split the kept arc). Two oval domains: one
    // starting above zero, one below.
    for (offset, rim_radius, t0) in [(3.0, 1.5, 1.0), (4.2, 2.5, -3.0), (3.0, 1.5, -3.0)] {
        disk_cap_seam_sweep(&TorusCut::new().with_domain_start(t0), offset, rim_radius);
    }
}

fn disk_cap_seam_sweep(cut: &TorusCut, offset_from_axis: f64, rim_radius: f64) {
    let rim_center = cut.in_plane(offset_from_axis, 0.0);
    // The cap reaches across the other (inner) section circle too.
    assert!(
        offset_from_axis - rim_radius < cut.inner,
        "fixture: the cap spans both branches"
    );
    let ends = circle_circle(cut.axis_foot, cut.outer, rim_center, rim_radius);
    let half_angle = {
        let e = ends[0] - cut.axis_foot;
        e.y().atan2(e.x()).abs()
    };
    let inside = |p: Point3| (p - rim_center).length() <= rim_radius + 1e-9;
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let rim = Circle3D::new(rim_center, normal, rim_radius).unwrap();
    // The cap's seam vertex is an artifact of construction; the kept arc
    // must not depend on where it sits, nor on the rim being split in two.
    // Seams spread round the rim, plus seams placed just inside and just
    // outside the tube within the on-oval band (0.05 from the outer section
    // circle), where the rim's starting side must still be read correctly.
    let near_oval_seams = [-0.05, 0.05, -0.02, 0.02].map(|offset: f64| {
        // In-plane rim angle φ (from the rim center, away from the torus
        // axis) where the rim is `outer + offset` from the axis.
        let target = cut.outer + offset;
        let d = offset_from_axis;
        let cos_phi = (target * target - d * d - rim_radius * rim_radius) / (2.0 * d * rim_radius);
        let phi = cos_phi.acos() * offset.signum();
        let seam_point =
            rim_center + Vec3::new(rim_radius * phi.cos(), rim_radius * phi.sin(), 0.0);
        assert!(
            ((seam_point - cut.axis_foot).length() - target).abs() < 1e-9,
            "fixture: seam sits {offset} from the outer section circle"
        );
        rim.project(seam_point)
    });
    let seams = (0..8)
        .map(|k| -3.0 + f64::from(k) * 0.8)
        .chain(near_oval_seams);
    for seam in seams {
        let mut topo = Topology::new();
        let wire = closed_circle_wire(&mut topo, &rim, seam);
        let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
        let pieces = cut
            .trim(&mut topo, face)
            .unwrap_or_else(|| panic!("seam {seam}: the in-cap arc was refused"));
        assert_kept_arc(
            cut,
            &pieces,
            ends,
            2.0 * half_angle,
            &inside,
            &format!("seam {seam}"),
        );

        let mut topo = Topology::new();
        let a = vertex(&mut topo, rim.evaluate(seam));
        let b = vertex(&mut topo, rim.evaluate(seam + 2.5));
        let e0 = arc_edge(&mut topo, &rim, (seam, seam + 2.5), a, b);
        let e1 = arc_edge(&mut topo, &rim, (seam + 2.5, seam + TAU), b, a);
        let wire = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
        let pieces = cut
            .trim(&mut topo, face)
            .unwrap_or_else(|| panic!("split rim at seam {seam}: refused"));
        assert_kept_arc(
            cut,
            &pieces,
            ends,
            2.0 * half_angle,
            &inside,
            &format!("split {seam}"),
        );
    }
}

#[test]
fn torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs() {
    let cut = TorusCut::new();
    let (rim_center, rim_radius) = cap_center_and_radius(&cut);
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let rim = Circle3D::new(rim_center, normal, rim_radius).unwrap();
    // A splitter-rebuilt rim: the second arc's circle is off by 1e-6 in
    // center and radius, well inside the 100 × tolerance weld band.
    let noisy = Circle3D::new(
        rim_center + Vec3::new(6.0e-7, -8.0e-7, 0.0),
        normal,
        rim_radius + 1.0e-6,
    )
    .unwrap();
    let mut topo = Topology::new();
    let a = vertex(&mut topo, rim.evaluate(0.3));
    let b = vertex(&mut topo, rim.evaluate(3.0));
    let e0 = arc_edge(&mut topo, &rim, (0.3, 3.0), a, b);
    let e1 = arc_edge(&mut topo, &noisy, (3.0, 0.3 + TAU), b, a);
    let wire = topo.add_wire(
        Wire::new(
            vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
    let pieces = cut
        .trim(&mut topo, face)
        .expect("one rim circle up to weld noise");
    let ends = circle_circle(cut.axis_foot, cut.outer, rim_center, rim_radius);
    let half_angle = {
        let e = ends[0] - cut.axis_foot;
        e.y().atan2(e.x()).abs()
    };
    // The crossings land on the noisy arc; allow for its 1e-6 offset.
    let first = pieces[0].p_start;
    let last = pieces[pieces.len() - 1].p_end;
    let near = |a: Point3, b: Point3| (a - b).length() < 1e-5;
    assert!(
        (near(first, ends[0]) && near(last, ends[1]))
            || (near(first, ends[1]) && near(last, ends[0])),
        "noisy rim arc ends {first:?} .. {last:?}"
    );
    let inside = |p: Point3| (p - rim_center).length() <= rim_radius + 1e-5;
    let exact_ends = [first, last];
    let e0v = first - cut.axis_foot;
    let e1v = last - cut.axis_foot;
    let angle = e0v.y().atan2(e0v.x()).abs() + e1v.y().atan2(e1v.x()).abs();
    assert!((angle - 2.0 * half_angle).abs() < 1e-5);
    assert_kept_arc(&cut, &pieces, exact_ends, angle, &inside, "noisy rim");
}

#[test]
fn torus_oval_defers_on_outlines_that_are_not_one_rim_circle_or_all_straight() {
    let cut = TorusCut::new();
    let (rim_center, rim_radius) = cap_center_and_radius(&cut);
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let rim = Circle3D::new(rim_center, normal, rim_radius).unwrap();

    // A "D": rim arc plus a straight chord (mixed outline).
    let mut topo = Topology::new();
    let (t0, t1) = (-1.2, 1.9);
    let a = vertex(&mut topo, rim.evaluate(t0));
    let b = vertex(&mut topo, rim.evaluate(t1));
    let arc = arc_edge(&mut topo, &rim, (t0, t1), a, b);
    let chord = line_edge(&mut topo, b, a);
    let wire = topo.add_wire(
        Wire::new(
            vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
    assert!(
        cut.trim(&mut topo, face).is_none(),
        "mixed arc + line outline must defer"
    );

    // The straight box below with its far side bulged into an outward arc:
    // three straight sides plus an arc is still a mixed outline.
    let mut topo = Topology::new();
    let (x0, x1, y0, y1) = (3.0, 5.2, -1.0, 1.3);
    let [c00, c10, c11, c01] = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        .map(|(x, y)| vertex(&mut topo, cut.in_plane(x, y)));
    let bulge_center = cut.in_plane(4.6, 0.15);
    let bulge = Circle3D::new(
        bulge_center,
        normal,
        (cut.in_plane(x1, y0) - bulge_center).length(),
    )
    .unwrap();
    let (b0, b1) = (
        bulge.project(cut.in_plane(x1, y0)),
        bulge.project(cut.in_plane(x1, y1)),
    );
    let bulge_edge = arc_edge(&mut topo, &bulge, (b0.min(b1), b0.max(b1)), c10, c11);
    let sides = [
        line_edge(&mut topo, c00, c10),
        line_edge(&mut topo, c11, c01),
        line_edge(&mut topo, c01, c00),
    ];
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(sides[0], true),
                OrientedEdge::new(bulge_edge, true),
                OrientedEdge::new(sides[1], true),
                OrientedEdge::new(sides[2], true),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
    assert!(
        cut.trim(&mut topo, face).is_none(),
        "three sides plus an arc must defer"
    );

    // An annular cap: the rim with a hole.
    let mut topo = Topology::new();
    let outer = closed_circle_wire(&mut topo, &rim, 0.5);
    let hole_circle = Circle3D::new(rim_center, normal, 0.3).unwrap();
    let hole = closed_circle_wire(&mut topo, &hole_circle, 0.5);
    let face = topo.add_face(Face::new(outer, vec![hole], cut.plane()));
    assert!(
        cut.trim(&mut topo, face).is_none(),
        "a holed cap is not a disk cap"
    );

    // A lens bounded by arcs of two different equal-radius circles; the
    // outer section circle enters it across one arc and leaves across the
    // other, so only the one-circle rule keeps it from being trimmed.
    let mut topo = Topology::new();
    let other_center = rim_center + Vec3::new(0.0, 1.2, 0.0);
    let other = Circle3D::new(other_center, normal, rim_radius).unwrap();
    let [p, q] = circle_circle(rim_center, rim_radius, other_center, rim_radius);
    let (p, q) = if p.x() > q.x() { (p, q) } else { (q, p) };
    // Counter-clockwise (about +z) angle from `a` to `b` around `c`.
    let ccw = |c: Point3, a: Point3, b: Point3| {
        let (da, db) = (a - c, b - c);
        (db.y().atan2(db.x()) - da.y().atan2(da.x())).rem_euclid(TAU)
    };
    let (vp, vq) = (vertex(&mut topo, p), vertex(&mut topo, q));
    // Rim arc p -> q over the top, other arc q -> p along the bottom.
    let (tp, span_rim) = (rim.project(p), ccw(rim_center, p, q));
    let (tq, span_other) = (other.project(q), ccw(other_center, q, p));
    assert!(
        (rim.evaluate(tp + span_rim) - q).length() < 1e-9,
        "fixture: rim arc ends at q"
    );
    assert!(
        (other.evaluate(tq + span_other) - p).length() < 1e-9,
        "fixture: arc ends at p"
    );
    let e0 = arc_edge(&mut topo, &rim, (tp, tp + span_rim), vp, vq);
    let e1 = arc_edge(&mut topo, &other, (tq, tq + span_other), vq, vp);
    let wire = topo.add_wire(
        Wire::new(
            vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
    assert!(
        cut.trim(&mut topo, face).is_none(),
        "arcs of two circles are not one rim"
    );

    // Concentric arcs of different radii (a radius-only mismatch).
    let mut topo = Topology::new();
    let smaller = Circle3D::new(rim_center, normal, 1.2).unwrap();
    let a = vertex(&mut topo, rim.evaluate(0.0));
    let b = vertex(&mut topo, rim.evaluate(3.0));
    let e0 = arc_edge(&mut topo, &rim, (0.0, 3.0), a, b);
    let e1 = arc_edge(&mut topo, &smaller, (3.0, TAU), b, a);
    let wire = topo.add_wire(
        Wire::new(
            vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
    assert!(
        cut.trim(&mut topo, face).is_none(),
        "rim arcs of different radii are not one rim"
    );
}

#[test]
fn torus_oval_wholly_inside_a_cap_is_kept_whole_and_wholly_outside_defers() {
    let cut = TorusCut::new();
    let normal = Vec3::new(0.0, 0.0, 1.0);
    // A cap around the whole outer section circle, clear of the tube.
    let big = Circle3D::new(cut.axis_foot, normal, cut.outer + 1.0).unwrap();
    let mut topo = Topology::new();
    let wire = closed_circle_wire(&mut topo, &big, 1.0);
    let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
    let kept = cut.trim(&mut topo, face).expect("an enclosed oval is kept");
    assert_eq!(kept.len(), 1);
    assert!((kept[0].p_start - cut.outer_oval().p_start).length() < 1e-12);

    // A small cap inside the torus hole, clear of both section circles.
    let hole = Circle3D::new(cut.axis_foot, normal, 0.5 * cut.inner).unwrap();
    let mut topo = Topology::new();
    let wire = closed_circle_wire(&mut topo, &hole, 1.0);
    let face = topo.add_face(Face::new(wire, vec![], cut.plane()));
    assert!(
        cut.trim(&mut topo, face).is_none(),
        "an oval outside the cap is not kept"
    );
}

#[test]
fn torus_oval_on_a_straight_box_face_keeps_the_exact_in_box_arc() {
    let cut = TorusCut::new();
    let (x0, x1, y0, y1) = (3.0, 5.2, -1.0, 1.3);
    let corners = [
        cut.in_plane(x0, y0),
        cut.in_plane(x1, y0),
        cut.in_plane(x1, y1),
        cut.in_plane(x0, y1),
    ];
    let mut topo = Topology::new();
    let face = polygon_face(&mut topo, &corners, Vec3::new(0.0, 0.0, 1.0));
    let pieces = cut.trim(&mut topo, face).expect("the in-box arc");
    // The outer circle leaves the box through its y = y0 and y = y1 sides.
    let x_at = |y: f64| cut.outer.mul_add(cut.outer, -y * y).sqrt();
    let ends = [cut.in_plane(x_at(y0), y0), cut.in_plane(x_at(y1), y1)];
    let expected_angle = y1.atan2(x_at(y1)) - y0.atan2(x_at(y0));
    let foot = cut.axis_foot;
    let inside = |p: Point3| {
        let (x, y) = (p.x() - foot.x(), p.y() - foot.y());
        x >= x0 - 1e-9 && x <= x1 + 1e-9 && y >= y0 - 1e-9 && y <= y1 + 1e-9
    };
    assert_kept_arc(&cut, &pieces, ends, expected_angle, &inside, "straight box");
}

// ── perform_with_context: B39 single-rim notch split ─────────────────────

/// The make_torus topology: one toroidal face bounded by two degenerate
/// seam loops, R = 3, r = 0.5, about +z at the origin.
fn torus_solid(topo: &mut Topology) -> remus_topology::solid::SolidId {
    let surface = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 0.5).unwrap();
    let v0 = vertex(topo, Point3::new(3.5, 0.0, 0.0));
    let ea = topo.add_edge(Edge::new(v0, v0, EdgeCurve::Line));
    let eb = topo.add_edge(Edge::new(v0, v0, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(ea, true),
                OrientedEdge::new(eb, true),
                OrientedEdge::new(ea, false),
                OrientedEdge::new(eb, false),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(surface)));
    let shell = topo.add_shell(remus_topology::shell::Shell::new(vec![face]).unwrap());
    topo.add_solid(remus_topology::solid::Solid::new(shell, vec![]))
}

/// The B39 oblique cell's frustum (base r 1.5, top r 1, height 1), built in
/// place: base center (2.5, 0, 0), axis Rx(π/4)·z. Returns the solid, the
/// wall face, and the base/top rim centers along the axis.
fn oblique_frustum(topo: &mut Topology) -> (remus_topology::solid::SolidId, FaceId, Vec3, Point3) {
    use remus_math::surfaces::ConicalSurface;
    let s = FRAC_1_SQRT_2;
    let origin = Point3::new(2.5, 0.0, 0.0);
    let ex = Vec3::new(1.0, 0.0, 0.0);
    let ez = Vec3::new(0.0, -s, s);
    let (r_bot, r_top, h) = (1.5, 1.0, 1.0);
    let bot_c = origin;
    let top_c = origin + ez * h;
    let v_bot = vertex(topo, bot_c + ex * r_bot);
    let v_top = vertex(topo, top_c + ex * r_top);
    let bot = Circle3D::new(bot_c, ez, r_bot).unwrap();
    let top = Circle3D::new(top_c, ez, r_top).unwrap();
    let tb = bot.project(bot_c + ex * r_bot);
    let tt = top.project(top_c + ex * r_top);
    let e_bot = arc_edge(topo, &bot, (tb, tb + TAU), v_bot, v_bot);
    let e_top = arc_edge(topo, &top, (tt, tt + TAU), v_top, v_top);
    let e_seam = line_edge(topo, v_bot, v_top);
    // Apex beyond the small end; the axis runs apex → base.
    let to_apex = r_top * h / (r_bot - r_top);
    let apex = origin + ez * (h + to_apex);
    let cone = ConicalSurface::new(apex, -ez, (to_apex + h).atan2(r_bot)).unwrap();
    let lateral_wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(e_bot, true),
                OrientedEdge::new(e_seam, true),
                OrientedEdge::new(e_top, false),
                OrientedEdge::new(e_seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    let wall = topo.add_face(Face::new(lateral_wire, vec![], FaceSurface::Cone(cone)));
    let bot_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e_bot, false)], true).unwrap());
    let bot_cap = topo.add_face(Face::new(bot_wire, vec![], plane_surface(-ez, bot_c)));
    let top_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e_top, true)], true).unwrap());
    let top_cap = topo.add_face(Face::new(top_wire, vec![], plane_surface(ez, top_c)));
    let shell =
        topo.add_shell(remus_topology::shell::Shell::new(vec![wall, bot_cap, top_cap]).unwrap());
    (
        topo.add_solid(remus_topology::solid::Solid::new(shell, vec![])),
        wall,
        ez,
        origin,
    )
}

/// Distance from `p` to the torus surface (R = 3, r = 0.5 about +z).
fn torus_distance(p: Point3) -> f64 {
    let radial = p.x().hypot(p.y()) - 3.0;
    (radial.hypot(p.z()) - 0.5).abs()
}

#[test]
fn a_torus_section_notching_one_frustum_rim_is_split_at_an_interior_vertex() {
    // B39 oblique cell: the tube leaves the frustum wall through its top rim
    // and re-enters through the same rim. That section and the rim span
    // between its ends would share both endpoints, so it must reach FF
    // consumers as two pieces joined at an interior vertex.
    let mut topo = Topology::new();
    let torus = torus_solid(&mut topo);
    let (frustum, wall, ez, origin) = oblique_frustum(&mut topo);
    let mut arena = crate::ds::GfaArena::new();
    crate::pave_filler::PaveFiller::with_tolerance(&mut topo, torus, frustum, Tolerance::default())
        .perform(&mut arena)
        .unwrap();
    let top_center = origin + ez;
    let o = Point3::new(0.0, 0.0, 0.0);
    let mut ends = Vec::new();
    for c in arena
        .curves
        .iter()
        .filter(|c| c.face_a == wall || c.face_b == wall)
    {
        let at = |t: f64| c.curve.evaluate_with_endpoints(t, o, o);
        let (t0, t1) = c.t_range;
        for i in 0..=16 {
            let p = at(t0 + (t1 - t0) * f64::from(i) / 16.0);
            assert!(
                torus_distance(p) < 1e-5,
                "section sample {p:?} is off the torus"
            );
        }
        let (s, e) = (at(t0), at(t1));
        let axial = |p: Point3| (p - origin).dot(ez);
        for bound in [0.0, 1.0] {
            assert!(
                !((axial(s) - bound).abs() < 1e-6 && (axial(e) - bound).abs() < 1e-6),
                "a torus x wall section runs rim-to-same-rim at axial {bound}: {s:?} -> {e:?}"
            );
        }
        ends.push((s, e));
    }
    // The pieces chain rim -> interior vertex -> rim.
    assert_eq!(
        ends.len(),
        2,
        "the notch reaches FF as two pieces, got {ends:?}"
    );
    let on_top_rim = |p: Point3| {
        let d = p - top_center;
        d.dot(ez).abs() < 1e-7 && (d.length() - 1.0).abs() < 1e-7 && torus_distance(p) < 1e-7
    };
    let shared = [
        (ends[0].0, ends[1].0),
        (ends[0].0, ends[1].1),
        (ends[0].1, ends[1].0),
        (ends[0].1, ends[1].1),
    ]
    .into_iter()
    .position(|(a, b)| (a - b).length() < 1e-9)
    .expect("the two pieces share their interior vertex");
    let (far0, far1, joint) = match shared {
        0 => (ends[0].1, ends[1].1, ends[0].0),
        1 => (ends[0].1, ends[1].0, ends[0].0),
        2 => (ends[0].0, ends[1].1, ends[0].1),
        _ => (ends[0].0, ends[1].0, ends[0].1),
    };
    assert!(
        on_top_rim(far0) && on_top_rim(far1),
        "both far ends are the exact top-rim x torus crossings: {far0:?}, {far1:?}"
    );
    assert!(
        (far0 - far1).length() > 0.1,
        "the notch spans a real rim arc"
    );
    let joint_axial = (joint - origin).dot(ez);
    assert!(
        joint_axial > 1e-3 && joint_axial < 1.0 - 1e-3,
        "the joint vertex {joint:?} lies inside the wall band, axial {joint_axial}"
    );
}

#[test]
fn torus_oval_on_a_notched_box_face_drops_the_short_excursion_through_the_notch() {
    // The straight box with a slot cut in from its far side: the oval leaves
    // the face through the slot's floor and re-enters through its roof, 0.1
    // later. Only the two in-face arcs survive; the excursion is outside.
    let cut = TorusCut::new();
    let (x0, x1, y0, y1) = (3.0, 5.2, -1.0, 1.3);
    let (slot_x, slot_y0, slot_y1) = (4.0, 0.2, 0.3);
    let outline = [
        (x0, y0),
        (x1, y0),
        (x1, slot_y0),
        (slot_x, slot_y0),
        (slot_x, slot_y1),
        (x1, slot_y1),
        (x1, y1),
        (x0, y1),
    ];
    let corners: Vec<Point3> = outline.iter().map(|&(x, y)| cut.in_plane(x, y)).collect();
    let mut topo = Topology::new();
    let face = polygon_face(&mut topo, &corners, Vec3::new(0.0, 0.0, 1.0));
    let pieces = cut.trim(&mut topo, face).expect("the two in-face arcs");

    let x_at = |y: f64| cut.outer.mul_add(cut.outer, -y * y).sqrt();
    let angle_at = |y: f64| y.atan2(x_at(y));
    let crossings = [y0, slot_y0, slot_y1, y1].map(|y| cut.in_plane(x_at(y), y));
    let expected_angle = (angle_at(slot_y0) - angle_at(y0)) + (angle_at(y1) - angle_at(slot_y1));
    let foot = cut.axis_foot;
    let in_face = |p: Point3| {
        let (x, y) = (p.x() - foot.x(), p.y() - foot.y());
        let in_box = x >= x0 - 1e-9 && x <= x1 + 1e-9 && y >= y0 - 1e-9 && y <= y1 + 1e-9;
        let in_slot = x > slot_x + 1e-9 && y > slot_y0 + 1e-9 && y < slot_y1 - 1e-9;
        in_box && !in_slot
    };
    let mut angle = 0.0;
    for piece in &pieces {
        let mut prev: Option<Vec3> = None;
        for i in 0..=64 {
            let t = piece.t_range.0 + (piece.t_range.1 - piece.t_range.0) * f64::from(i) / 64.0;
            let p = piece
                .curve
                .evaluate_with_endpoints(t, piece.p_start, piece.p_end);
            let radial = Vec3::new(p.x() - foot.x(), p.y() - foot.y(), 0.0);
            assert!(
                (radial.length() - cut.outer).abs() < 1e-6,
                "sample off the section circle"
            );
            assert!(
                in_face(p),
                "kept sample {p:?} lies in the slot or outside the face"
            );
            if let Some(a) = prev {
                angle += a.cross(radial).z().atan2(a.dot(radial)).abs();
            }
            prev = Some(radial);
        }
    }
    assert!(
        (angle - expected_angle).abs() < 1e-6,
        "kept arcs span {angle} rad, the in-face arcs span {expected_angle}"
    );
    for crossing in crossings {
        assert!(
            pieces.iter().any(|piece| {
                (piece.p_start - crossing).length() < 1e-8
                    || (piece.p_end - crossing).length() < 1e-8
            }),
            "no kept arc ends at the exact crossing {crossing:?}"
        );
    }
}

#[test]
fn curved_nurbs_trim_authority_stops_at_the_extent_handoff() {
    use crate::pave_filler::curved_section_clip::{clip_section, tests::fixture};
    use remus_math::context::OperationContext;

    let (topo, traces, section) = fixture(
        &[[0.125, 0.875, -1.0, 1.0], [0.375, 0.625, -0.5, 0.5]],
        &[[0.0, 1.0, -1.5, 1.5]],
        false,
        1.0,
        false,
        true,
        [(0.0, 1.0); 2],
    );
    let surfaces = traces.map(|trace| topo.face(trace.face).unwrap().surface());
    for j in 0..2 {
        assert!(
            FaceExtent::new(
                &topo,
                traces[j].face,
                surfaces[j],
                None,
                Tolerance::default()
            )
            .unwrap()
            .is_none()
        );
    }
    let raw = RawCurve {
        bbox: Aabb3::from_points(section.control_points().iter().copied()),
        t_range: section.domain(),
        p_start: section.evaluate(-8.0),
        p_end: section.evaluate(24.0),
        curve: EdgeCurve::NurbsCurve(section.clone()),
    };
    let current = super::restrict_curves_to_faces(
        &topo,
        traces[0].face,
        traces[1].face,
        surfaces[0],
        surfaces[1],
        None,
        None,
        vec![raw],
        Tolerance::default(),
        &mut super::JunctionRegistry::default(),
    )
    .unwrap();
    assert_eq!(current.len(), 1);
    assert!((current[0].t_range.0 + 8.0).abs() < 1e-12);
    assert!((current[0].t_range.1 - 24.0).abs() < 1e-12);
    let clipped = clip_section(&topo, traces, &section, &OperationContext::new()).unwrap();
    assert_eq!(clipped.intervals.len(), 2);
    for (interval, expected) in clipped.intervals.iter().zip([[-4.0, 4.0], [12.0, 20.0]]) {
        assert!((interval.source_range[0] - expected[0]).abs() < 1e-12);
        assert!((interval.source_range[1] - expected[1]).abs() < 1e-12);
    }
}
