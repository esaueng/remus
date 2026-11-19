//! The chord-polygon fan's area bound (`fan_bound`) against closed forms.
//!
//! A planar face falls to the fan when the exact reader declines one of its
//! edges. Here that edge is a loop `δ` wide at a corner (three control
//! points, the middle one `δ` off): too wide to be a point chord, its own
//! moments too small for the adaptive rule to certify. Every other edge is
//! read by the fan's chords, so the fan loses their slivers, and the bound
//! must cover that loss (the loop itself encloses about `δ²`).

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};

use super::*;
use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_topology::edge::Edge;
use remus_topology::face::Face;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// How the quarter disc's arc is stored.
#[derive(Clone, Copy, Debug)]
enum Arc {
    /// A native circle edge.
    Circle,
    /// The rational quadratic quarter arc.
    Rational,
}

/// The rational quadratic arc of radius `r` about `c` over `[a, a + π/2]`.
fn rational_quarter(c: Point3, r: f64, a: f64) -> NurbsCurve {
    let at = |t: f64, s: f64| Point3::new(c.x() + s * r * t.cos(), c.y() + s * r * t.sin(), c.z());
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            at(a, 1.0),
            at(a + FRAC_PI_2 / 2.0, 2.0_f64.sqrt()),
            at(a + FRAC_PI_2, 1.0),
        ],
        vec![1.0, FRAC_1_SQRT_2, 1.0],
    )
    .unwrap()
}

/// The full rational circle of radius `r` about `c`, four quarter spans.
fn rational_circle(c: Point3, r: f64) -> NurbsCurve {
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for k in 0..4 {
        #[allow(clippy::cast_precision_loss)]
        let a = k as f64 * FRAC_PI_2;
        let q = rational_quarter(c, r, a);
        points.extend_from_slice(&q.control_points()[..2]);
        weights.extend_from_slice(&q.weights()[..2]);
    }
    points.push(points[0]);
    weights.push(1.0);
    NurbsCurve::new(
        2,
        vec![
            0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
        ],
        points,
        weights,
    )
    .unwrap()
}

/// The quarter disc of radius `r` at `c`, its arc stored as `arc`, its
/// corner at `c` closed by a loop `delta` wide, and optionally a circular
/// hole of radius `hole` about `c + (r/2, r/2)/√2·…`, stored as a closed
/// native circle or rational circle.
fn quarter_disc(
    topo: &mut Topology,
    c: Point3,
    r: f64,
    arc: Arc,
    delta: Option<f64>,
    hole: Option<(f64, Arc)>,
) -> FaceId {
    let v = |topo: &mut Topology, p: Point3| topo.add_vertex(Vertex::new(p, 1e-9));
    let a = v(topo, c);
    let b = v(topo, Point3::new(c.x() + r, c.y(), c.z()));
    let p = v(topo, Point3::new(c.x(), c.y() + r, c.z()));
    let curve = match arc {
        Arc::Circle => {
            let circle = Circle3D::new(c, Z, r).unwrap();
            let t0 = circle.project(Point3::new(c.x() + r, c.y(), c.z()));
            let mut t1 = circle.project(Point3::new(c.x(), c.y() + r, c.z()));
            if t1 < t0 {
                t1 += 2.0 * PI;
            }
            (EdgeCurve::Circle(circle), (t0, t1))
        }
        Arc::Rational => (
            EdgeCurve::NurbsCurve(rational_quarter(c, r, 0.0)),
            (0.0, 1.0),
        ),
    };
    let mut arc_edge = Edge::new(b, p, curve.0);
    arc_edge.set_trim(Some(curve.1));
    let mut edges = vec![Edge::new(a, b, EdgeCurve::Line), arc_edge];
    match delta {
        Some(delta) => {
            let a2 = v(topo, c);
            edges.push(Edge::new(p, a2, EdgeCurve::Line));
            let off = Point3::new(c.x() + delta, c.y() + delta, c.z());
            let lump = NurbsCurve::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![c, off, c],
                vec![1.0, FRAC_1_SQRT_2, 1.0],
            )
            .unwrap();
            let mut lump_edge = Edge::new(a2, a, EdgeCurve::NurbsCurve(lump));
            lump_edge.set_trim(Some((0.0, 1.0)));
            edges.push(lump_edge);
        }
        None => edges.push(Edge::new(p, a, EdgeCurve::Line)),
    }
    let edges = edges
        .into_iter()
        .map(|e| OrientedEdge::new(topo.add_edge(e), true))
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let inner = hole.map(|(radius, kind)| {
        let centre = Point3::new(c.x() + 0.45 * r, c.y() + 0.45 * r, c.z());
        // A seam vertex off the curve's parameter origin, so the closed
        // rational circle's chords cross the end of its domain.
        let seam = Point3::new(
            centre.x() + radius * 1.0_f64.cos(),
            centre.y() + radius * 1.0_f64.sin(),
            c.z(),
        );
        let s = v(topo, seam);
        let curve = match kind {
            Arc::Circle => EdgeCurve::Circle(Circle3D::new(centre, Z, radius).unwrap()),
            Arc::Rational => EdgeCurve::NurbsCurve(rational_circle(centre, radius)),
        };
        let mut edge = Edge::new(s, s, curve);
        if matches!(kind, Arc::Circle) {
            edge.set_trim(Some((0.0, 2.0 * PI)));
        }
        let e = topo.add_edge(edge);
        topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap())
    });
    topo.add_face(Face::new(
        wire,
        inner.into_iter().collect(),
        FaceSurface::Plane {
            normal: Z,
            d: c.z(),
        },
    ))
}

/// The fan's samples of the quarter arc: 32 chords over its parameter.
fn arc_samples(c: Point3, r: f64, arc: Arc) -> Vec<Point3> {
    let nurbs = rational_quarter(c, r, 0.0);
    (0..=32)
        .map(|i| {
            let f = f64::from(i) / 32.0;
            match arc {
                Arc::Circle => Point3::new(
                    c.x() + r * (f * FRAC_PI_2).cos(),
                    c.y() + r * (f * FRAC_PI_2).sin(),
                    c.z(),
                ),
                Arc::Rational => nurbs.evaluate(f),
            }
        })
        .collect()
}

/// The fan's samples of the closed hole: 32 chords around from its seam.
fn hole_samples(centre: Point3, radius: f64, kind: Arc) -> Vec<Point3> {
    let nurbs = rational_circle(centre, radius);
    let seam_point = Point3::new(
        centre.x() + radius * 1.0_f64.cos(),
        centre.y() + radius * 1.0_f64.sin(),
        centre.z(),
    );
    let seam = crate::util::nurbs_seam_parameter(&nurbs, seam_point, 0.0, 1.0);
    (0..=32)
        .map(|i| {
            let f = f64::from(i) / 32.0;
            match kind {
                Arc::Circle => Point3::new(
                    centre.x() + radius * (f * 2.0 * PI).cos(),
                    centre.y() + radius * (f * 2.0 * PI).sin(),
                    centre.z(),
                ),
                Arc::Rational => nurbs.evaluate((seam + f).rem_euclid(1.0)),
            }
        })
        .collect()
}

/// `Σ R²(θ − sin θ)/2` over consecutive `samples` on the circle of radius
/// `radius` about `centre`.
fn sliver_loss(centre: Point3, radius: f64, samples: &[Point3]) -> f64 {
    samples
        .windows(2)
        .map(|pair| {
            let (a, b) = (pair[0] - centre, pair[1] - centre);
            let theta = a.cross(b).length().atan2(a.dot(b));
            radius * radius * (theta - theta.sin()) / 2.0
        })
        .sum()
}

fn fixed_reading(topo: &Topology, face: FaceId) -> FluxReading {
    integrate_face_fixed_flux_with_boundary_about(topo, face, 5, Point3::new(0.0, 0.0, 0.0), true)
        .unwrap()
}

#[test]
fn the_fan_bound_covers_the_chord_slivers_and_is_tight() {
    // Off the origin, where the loop's moments are rounding noise (at the
    // origin they are exactly zero and the adaptive rule converges).
    for c in [
        Point3::new(0.05, 0.03, 0.01),
        Point3::new(5.0, 3.0, 1.0),
        Point3::new(-500.0, 300.0, 100.0),
    ] {
        for r in [1e-2, 2.0, 1e2] {
            for arc in [Arc::Circle, Arc::Rational] {
                for hole in [
                    None,
                    Some((0.2 * r, Arc::Circle)),
                    Some((0.2 * r, Arc::Rational)),
                ] {
                    let label = format!("c={c:?} r={r} arc={arc:?} hole={hole:?}");
                    let mut topo = Topology::new();
                    let face = quarter_disc(&mut topo, c, r, arc, Some(1e-8 * r), hole);
                    assert!(
                        integrate_planar_face_exact(&topo, face, Z, c)
                            .unwrap()
                            .is_none(),
                        "{label}: the loop forces the fan"
                    );
                    let reading = fixed_reading(&topo, face);
                    assert!(reading.fitted_boundary.is_none(), "{label}");
                    let bound = reading
                        .planar_fan_area_bound
                        .unwrap_or_else(|| panic!("{label}: a fan reading carries its bound"));
                    let hole_area = hole.map_or(0.0, |(radius, _)| PI * radius * radius);
                    let exact = PI * r * r / 4.0 - hole_area;
                    // The slivers the fan's chords cut off, each a circular
                    // segment `R²(θ − sin θ)/2` of its chord's central angle.
                    let outer = sliver_loss(c, r, &arc_samples(c, r, arc));
                    let inner = hole.map_or(0.0, |(radius, kind)| {
                        let centre = Point3::new(c.x() + 0.45 * r, c.y() + 0.45 * r, c.z());
                        sliver_loss(centre, radius, &hole_samples(centre, radius, kind))
                    });
                    let expected = exact - outer + inner;
                    // The δ loop encloses at most δ²; rounding at the position.
                    let slack = 1e-15 * r * r + 64.0 * f64::EPSILON * r * c.x().abs().max(1.0);
                    assert!(
                        (reading.contribution.area - expected).abs() <= slack,
                        "{label}: the fan reads {}, its chord polygon {expected}",
                        reading.contribution.area
                    );
                    let error = (reading.contribution.area - exact).abs();
                    assert!(
                        error <= bound && outer + inner <= bound,
                        "{label}: the fan is {error:.3e} off (slivers {:.3e}), bound {bound:.3e}",
                        outer + inner
                    );
                    // Within a small factor of the slivers it bounds: a circle's
                    // `g · ℓ` is `(Δ²r/8)(Δr)` against a segment of `Δ³r²/12`,
                    // 1.5; a rational piece's halved hull stands off its chord
                    // the sagitta, against a segment of two-thirds chord times
                    // sagitta, also 1.5.
                    assert!(
                        bound <= 1.55 * (outer + inner),
                        "{label}: bound {bound:.3e} against slivers of {:.3e}",
                        outer + inner
                    );
                }
            }
        }
    }
}

#[test]
fn exact_planar_readings_carry_no_fan_bound() {
    for arc in [Arc::Circle, Arc::Rational] {
        for hole in [None, Some((0.4, Arc::Circle)), Some((0.4, Arc::Rational))] {
            let mut topo = Topology::new();
            let face = quarter_disc(&mut topo, Point3::new(5.0, 3.0, 1.0), 2.0, arc, None, hole);
            let reading = fixed_reading(&topo, face);
            assert_eq!(reading.planar_fan_area_bound, None, "{arc:?} {hole:?}");
            let exact = PI - hole.map_or(0.0, |(radius, _)| PI * radius * radius);
            assert!((reading.contribution.area - exact).abs() <= 1e-13 * exact);
        }
    }
}

#[test]
fn a_straight_edged_fan_face_has_a_zero_bound() {
    // A triangle closed by the δ loop: every chord but the loop's is exact.
    let mut topo = Topology::new();
    let c = Point3::new(5.0, 3.0, 1.0);
    let a = topo.add_vertex(Vertex::new(c, 1e-9));
    let a2 = topo.add_vertex(Vertex::new(c, 1e-9));
    let b = topo.add_vertex(Vertex::new(Point3::new(7.0, 3.0, 1.0), 1e-9));
    let p = topo.add_vertex(Vertex::new(Point3::new(5.0, 5.0, 1.0), 1e-9));
    let lump = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![c, Point3::new(5.0 + 1e-8, 3.0 + 1e-8, 1.0), c],
        vec![1.0, FRAC_1_SQRT_2, 1.0],
    )
    .unwrap();
    let mut lump_edge = Edge::new(a2, a, EdgeCurve::NurbsCurve(lump));
    lump_edge.set_trim(Some((0.0, 1.0)));
    let edges = [
        Edge::new(a, b, EdgeCurve::Line),
        Edge::new(b, p, EdgeCurve::Line),
        Edge::new(p, a2, EdgeCurve::Line),
        lump_edge,
    ]
    .into_iter()
    .map(|e| OrientedEdge::new(topo.add_edge(e), true))
    .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: Z, d: 1.0 },
    ));
    assert!(
        integrate_planar_face_exact(&topo, face, Z, c)
            .unwrap()
            .is_none()
    );
    let reading = fixed_reading(&topo, face);
    let bound = reading.planar_fan_area_bound.unwrap();
    // Only the loop's own hull: δ wide, about 2δ long.
    assert!(bound <= 1e-15, "{bound:.3e}");
    assert!((reading.contribution.area - 2.0).abs() <= 1e-14);
}

/// A cached fan reading answers a request about another reference, or for a
/// translated copy of its face, with its bound: the bound depends on neither.
#[test]
fn cached_volume_terms_keep_the_fan_bound() {
    use crate::properties::face_cache::FaceIntegralCache;

    let mut topo = Topology::new();
    let at = Point3::new(5.0, 3.0, 1.0);
    let source = quarter_disc(&mut topo, at, 2.0, Arc::Circle, Some(2e-8), None);
    let copy = quarter_disc(
        &mut topo,
        at + Vec3::new(2.5, -1.0, 4.0),
        2.0,
        Arc::Circle,
        Some(2e-8),
        None,
    );
    let cold = |face, reference| {
        FaceIntegralCache::disabled()
            .integrate_face_volume_about(&topo, face, 5, reference)
            .unwrap()
            .planar_fan_area_bound
            .expect("a fan reading")
    };
    let (near, far) = (Point3::new(1.0, 2.0, 3.0), Point3::new(-4.0, 0.5, 9.0));
    let mut cache = FaceIntegralCache::new();
    let first = cache
        .integrate_face_volume_about(&topo, source, 5, near)
        .unwrap();
    assert_eq!(first.planar_fan_area_bound, Some(cold(source, near)));
    let rereferenced = cache
        .integrate_face_volume_about(&topo, source, 5, far)
        .unwrap();
    let translated = cache
        .integrate_face_volume_about(&topo, copy, 5, far)
        .unwrap();
    let stats = cache.stats();
    assert_eq!((stats.rereferenced, stats.translated), (1, 1));
    assert_eq!(
        rereferenced.planar_fan_area_bound,
        first.planar_fan_area_bound
    );
    let bound = translated.planar_fan_area_bound.unwrap();
    let fresh = cold(copy, far);
    assert!((bound - fresh).abs() <= 1e-12 * fresh, "{bound} vs {fresh}");
}
