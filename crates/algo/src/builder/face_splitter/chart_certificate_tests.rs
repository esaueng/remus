//! Soundness and adversarial tests of the chart certificate.
//!
//! The enclosure tests sample spans densely and require every sample inside
//! the span's enclosure; the adversarial tests build exact curves whose
//! contacts fall between any fixed sampling (a near-tangent double crossing,
//! a seam excursion between samples, a rim touch) and require a decline,
//! next to near misses that must still certify.
#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::many_single_char_names
)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_6};

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::{NurbsCurve, interpolate};
use remus_math::surfaces::ToroidalSurface;

use super::super::chart_arrangement::{Authority, Piece};
use super::*;

const R: f64 = 10.0;
const MINOR: f64 = 2.0;

#[test]
fn long_knot_span_trimming_preserves_the_original_curve_endpoints() {
    let half = 0.1_f64;
    let end = 0.2_f64;
    let mu = 1e-12;
    let curve = NurbsCurve::new(
        2,
        vec![-1e12, -1e12, -1e12, 1.0, 1.0, 1.0],
        vec![
            Point3::new(12.0, 0.0, 0.0),
            Point3::new(12.0, 12.0 * half.tan(), 0.0),
            Point3::new(12.0 * end.cos(), 12.0 * end.sin(), 0.0),
        ],
        vec![1.0, half.cos() * mu, mu * mu],
    )
    .unwrap();
    let midpoint = Point3::new(12.0 * half.cos(), 12.0 * half.sin(), 0.0);
    assert!((curve.evaluate(0.0) - midpoint).length() < 1e-9);
    let map = plane_map();
    for (lo, hi) in [(0.0, 1.0), (0.0, 0.5), (0.5, 0.0)] {
        let mut nodes = Vec::new();
        let arc = edge(
            &mut nodes,
            &map,
            EdgeCurve::NurbsCurve(curve.clone()),
            lo,
            hi,
        );
        let pieces = piece_geoms(&map, &mut BezierCache::default(), &arc).unwrap();
        for (got, parameter) in [(pieces[0].start(), lo), (pieces.last().unwrap().end(), hi)] {
            let expected = map.local(curve.evaluate(parameter));
            let distance = got
                .into_iter()
                .zip(expected)
                .fold(0.0_f64, |d, (a, b)| d.hypot(a - b));
            assert!(
                distance < 1e-10,
                "trim ({lo},{hi}) endpoint moved by {distance:e}"
            );
        }
    }
}

#[test]
fn coincident_meridians_decline_within_the_pair_check_budget() {
    let map = torus_map();
    let mut nodes = Vec::new();
    let (curve, t0, t1) = meridian(0.1, 1.2);
    let (_, s0, s1) = meridian(0.3, 1.0);
    let a = edge(&mut nodes, &map, curve.clone(), t0, t1);
    let b = edge(&mut nodes, &map, curve, s0, s1);
    let mut pairs_left = 128;
    assert!(certify_with_pair_budget(&map, &[a, b], &nodes, &cfg(), &mut pairs_left).is_none());
    assert_eq!(
        pairs_left, 0,
        "the conflict scan must stop at its work limit"
    );
}

#[test]
fn homogeneous_trimming_preserves_a_low_weight_endpoint() {
    let weight = 1e8;
    let cps = vec![
        [100.0, 0.0, 0.0, 1.0],
        [100.0 * weight, 100.0 * weight, 0.0, weight],
        [0.0, 100.0, 0.0, 1.0],
    ];
    let t = 1.0 - 1e-9;
    // Independent positive Bernstein sum, rather than recursive de Casteljau.
    let basis = [(1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t];
    let expected_hom: [f64; 4] =
        std::array::from_fn(|k| basis.iter().zip(&cps).map(|(b, p)| b * p[k]).sum());
    let expected = dehom(expected_hom);
    let (left, right) = bez_split(&cps, t);
    for got in [
        dehom(*left.last().unwrap()),
        dehom(right[0]),
        bez_eval(&cps, t),
    ] {
        let distance = got
            .into_iter()
            .zip(expected)
            .fold(0.0_f64, |d, (a, b)| d.hypot(a - b));
        assert!(distance < 1e-10, "trim endpoint moved by {distance:e}");
    }
}

#[test]
fn compressed_knot_decomposition_cannot_certify_an_unchecked_curve_tail() {
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1e-16, 6e-16, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(5.0, 8.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
        ],
        vec![1.0; 6],
    )
    .unwrap();
    // Locate the real curve's transversal crossing independently of the
    // decomposition used by certification.
    let (mut lo, mut hi) = (0.4, 0.5);
    assert!(curve.evaluate(lo).x() < 4.5 && curve.evaluate(hi).x() > 4.5);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if curve.evaluate(mid).x() < 4.5 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    assert!((1.0..3.0).contains(&curve.evaluate(0.5 * (lo + hi)).y()));
    let map = plane_map();
    let mut nodes = Vec::new();
    let arc = edge(&mut nodes, &map, EdgeCurve::NurbsCurve(curve), 0.0, 1.0);
    let crossing = edge_between(
        &mut nodes,
        &map,
        EdgeCurve::Line,
        0.0,
        1.0,
        Point3::new(4.5, 1.0, 0.0),
        Point3::new(4.5, 3.0, 0.0),
    );
    assert!(certify(&map, &[arc, crossing], &nodes, &cfg()).is_none());
}

#[test]
fn a_tiny_parameter_tail_with_a_large_excursion_cannot_be_skipped() {
    // Positive rational weights compress a 2.9-unit excursion into the
    // final 1e-15 of the first span. Its start is within 3e-16 of the node;
    // endpoint agreement cannot replace an enclosure of the whole trim.
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0],
        vec![
            Point3::new(0.0, -10.0, 0.0),
            Point3::new(-9.999_999_999_999_99, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-1.0, 3.0, 0.0),
            Point3::new(-1.0, 6.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ],
        vec![
            1.0,
            1.001_600_474_254_403_4e30,
            1_000_799_917_193_443.5,
            1.0,
            1.0,
            1.0,
            1.0,
        ],
    )
    .unwrap();
    let map = plane_map();
    let mut nodes = Vec::new();
    let arc = edge_between(
        &mut nodes,
        &map,
        EdgeCurve::NurbsCurve(curve),
        1.0 - 1e-15,
        2.0,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    );
    let crossing = edge_between(
        &mut nodes,
        &map,
        EdgeCurve::Line,
        0.0,
        1.0,
        Point3::new(1.0, -1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    );
    assert!(certify(&map, &[arc, crossing], &nodes, &cfg()).is_none());
}

fn torus() -> ToroidalSurface {
    ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), R, MINOR).unwrap()
}

fn torus_map() -> ChartMap {
    ChartMap::of_carrier(&ChartCarrier::Torus(torus()))
}

fn plane_map() -> ChartMap {
    ChartMap::Plane {
        origin: Point3::new(0.0, 0.0, 0.0),
        axes: [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ],
    }
}

fn cfg() -> CertConfig {
    CertConfig {
        mismatch_cap: 1e-9,
        ball_cap: 1e-7,
    }
}

fn uv(map: &ChartMap, p: Point3) -> Point2 {
    let (u, v) = map.chart_raw(map.local(p)).unwrap();
    Point2::new(u, v)
}

/// A graph edge for `curve` over `[ta, tb]`, with a node at each end.
fn edge(nodes: &mut Vec<Node>, map: &ChartMap, curve: EdgeCurve, ta: f64, tb: f64) -> GraphEdge {
    let o = Point3::new(0.0, 0.0, 0.0);
    let pa = curve.evaluate_with_endpoints(ta, o, o);
    let pb = curve.evaluate_with_endpoints(tb, o, o);
    edge_between(nodes, map, curve, ta, tb, pa, pb)
}

fn edge_between(
    nodes: &mut Vec<Node>,
    map: &ChartMap,
    curve: EdgeCurve,
    ta: f64,
    tb: f64,
    pa: Point3,
    pb: Point3,
) -> GraphEdge {
    let mut add = |p: Point3| {
        nodes.push(Node {
            uv: uv(map, p),
            p3: p,
            authority: Authority::SectionEnd,
        });
        nodes.len() - 1
    };
    let (a, b) = (add(pa), add(pb));
    GraphEdge {
        piece: Piece {
            curve,
            t_a: ta,
            t_b: tb,
            p3_a: pa,
            p3_b: pb,
            auth_a: Authority::SectionEnd,
            auth_b: Authority::SectionEnd,
            poly: vec![nodes[a].uv, nodes[b].uv],
            pave_block_id: None,
            source_topo_edge: None,
            boundary: false,
        },
        node_a: a,
        node_b: b,
    }
}

/// The parallel circle of the torus at minor angle `v`.
fn parallel(v: f64) -> EdgeCurve {
    EdgeCurve::Circle(
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, MINOR * v.sin()),
            Vec3::new(0.0, 0.0, 1.0),
            R + MINOR * v.cos(),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    )
}

/// The meridian circle of the torus at major angle 0, and its parameters at
/// minor angles `v0`, `v1`.
fn meridian(v0: f64, v1: f64) -> (EdgeCurve, f64, f64) {
    let c = Circle3D::new_with_ref(
        Point3::new(R, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        MINOR,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let t0 = c.project(torus().evaluate(0.0, v0));
    let mut t1 = c.project(torus().evaluate(0.0, v1));
    // The short way round.
    t1 += TAU * ((t0 - t1) / TAU).round();
    (EdgeCurve::Circle(c), t0, t1)
}

/// A cubic NURBS through torus points `(u(s), v(s))` at the given `s`.
fn torus_section(f: impl Fn(f64) -> (f64, f64), ss: &[f64]) -> EdgeCurve {
    let t = torus();
    let pts: Vec<Point3> = ss
        .iter()
        .map(|&s| {
            let (u, v) = f(s);
            t.evaluate(u, v)
        })
        .collect();
    EdgeCurve::NurbsCurve(interpolate(&pts, 3).unwrap())
}

fn uniform(n: usize) -> Vec<f64> {
    (0..=n).map(|k| k as f64 / n as f64).collect()
}

// ---------------------------------------------------------------------------
// Enclosure soundness
// ---------------------------------------------------------------------------

/// Every dense sample of every certified span lies in its enclosure.
fn assert_enclosures_hold(map: &ChartMap, e: &GraphEdge) {
    let mut cache = BezierCache::default();
    let mut geoms = Vec::new();
    for g in piece_geoms(map, &mut cache, e).unwrap() {
        usable(map, g, 0, &mut geoms).unwrap();
    }
    // Refine further so narrow enclosures are exercised too.
    for _ in 0..3 {
        let mut next = Vec::new();
        for (g, d, _) in geoms {
            let (l, r) = g.split();
            usable(map, l, d + 1, &mut next).unwrap();
            usable(map, r, d + 1, &mut next).unwrap();
        }
        geoms = next;
    }
    let mut cur = map.unwrap_near(
        map.chart_raw(geoms[0].0.start()).unwrap(),
        Point2::new(0.0, 0.0),
    );
    for (g, _, cone) in &geoms {
        let end = map.unwrap_near(map.chart_raw(g.end()).unwrap(), cur);
        let poly = convex_hull(quad(cur, end, *cone).unwrap());
        for j in 0..=200 {
            let p = map.unwrap_near(map.chart_raw(g.at(f64::from(j) / 200.0)).unwrap(), cur);
            let d = polygon_distance(p, &poly);
            assert!(
                d <= 1e-12,
                "sample {j} of a span lies {d:e} outside its enclosure"
            );
        }
        cur = end;
    }
}

#[test]
fn enclosures_contain_conic_and_rational_spans_in_every_chart() {
    let cyl = remus_math::surfaces::CylindricalSurface::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        3.0,
    )
    .unwrap();
    let maps = [
        torus_map(),
        ChartMap::of_carrier(&ChartCarrier::Cylinder(cyl)),
        plane_map(),
    ];
    let tilted = Ellipse3D::new_with_ref(
        Point3::new(8.0, 1.0, 2.5),
        Vec3::new(0.3, 0.2, 1.0),
        2.0,
        1.2,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let circle = Circle3D::new_with_ref(
        Point3::new(10.0, 0.0, 2.0),
        Vec3::new(0.1, 1.0, 0.2),
        0.8,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    // A rational quadratic with strongly uneven positive weights.
    let rational = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![
            Point3::new(11.0, -1.0, 2.7),
            Point3::new(12.0, 0.0, 4.0),
            Point3::new(11.0, 1.5, 2.9),
            Point3::new(10.0, 2.5, 2.0),
        ],
        vec![1.0, 40.0, 0.05, 1.0],
    )
    .unwrap();
    for map in &maps {
        let mut nodes = Vec::new();
        let curves = [
            (EdgeCurve::Ellipse(tilted.clone()), 0.3, 2.9),
            (EdgeCurve::Circle(circle.clone()), 2.0, -1.0),
            (EdgeCurve::NurbsCurve(rational.clone()), 0.05, 0.97),
            (EdgeCurve::NurbsCurve(rational.clone()), 0.9, 0.1),
        ];
        for (c, ta, tb) in curves {
            let e = edge(&mut nodes, map, c, ta, tb);
            assert_enclosures_hold(map, &e);
        }
    }
}

// ---------------------------------------------------------------------------
// Adversarial arrangements
// ---------------------------------------------------------------------------

/// The plane `z = 1 + (x − x*)` cuts the torus near the parallel
/// `v = π/6` (`z = 1`): for `x*` just under the parallel's radius it crosses
/// that parallel twice, `±0.004` rad apart around `u = 0`, in a lens
/// `7·10⁻⁵` mm deep. A 96-segment polyline of the section misses both
/// crossings in about a third of its sample phases; the certificate must
/// decline whatever the sampling.
fn near_tangent_pair(offset: f64) -> (ChartMap, Vec<Node>, Vec<GraphEdge>) {
    let map = torus_map();
    let rho = R + MINOR * FRAC_PI_6.cos();
    let x_star = rho + offset;
    let v_of = |u: f64| {
        let mut v = FRAC_PI_6;
        for _ in 0..80 {
            let g = MINOR * v.sin() - 1.0 - ((R + MINOR * v.cos()) * u.cos() - x_star);
            let dg = MINOR * v.cos() + MINOR * v.sin() * u.cos();
            v -= g / dg;
        }
        v
    };
    let mut nodes = Vec::new();
    let a = edge(&mut nodes, &map, parallel(FRAC_PI_6), -0.6, 0.6);
    let b = edge(
        &mut nodes,
        &map,
        torus_section(|s| (-0.6 + 1.2 * s, v_of(-0.6 + 1.2 * s)), &uniform(240)),
        0.0,
        1.0,
    );
    (map, nodes, vec![a, b])
}

#[test]
fn near_tangent_double_crossing_declines() {
    let (map, nodes, edges) = near_tangent_pair(-1e-4);
    // The exact section really crosses the parallel twice.
    let EdgeCurve::NurbsCurve(n) = &edges[1].piece.curve else {
        unreachable!()
    };
    let mut signs = Vec::new();
    for k in 0..=20_000 {
        let v = uv(&map, n.evaluate(f64::from(k) / 20_000.0)).y();
        let s = v > FRAC_PI_6;
        if signs.last() != Some(&s) {
            signs.push(s);
        }
    }
    assert_eq!(
        signs.len(),
        3,
        "the section dips across the parallel and back"
    );
    assert!(certify(&map, &edges, &nodes, &cfg()).is_none());
}

#[test]
fn near_tangent_clear_miss_certifies() {
    let (map, nodes, edges) = near_tangent_pair(1e-3);
    assert!(certify(&map, &edges, &nodes, &cfg()).is_some());
}

/// A seam meridian `u = 0` from `v = 0` to `v = π/2`, and a section at
/// `u ≈ 0.05` with a narrow bump towards the seam reaching `u = depth`.
/// The bump sits between the samples of a 512-sample sweep of the section.
fn seam_bump(depth: f64) -> (ChartMap, Vec<Node>, Vec<GraphEdge>) {
    let map = torus_map();
    let mut nodes = Vec::new();
    let (seam, t0, t1) = meridian(0.0, FRAC_PI_2);
    let seam = edge(&mut nodes, &map, seam, t0, t1);
    let centre = 255.5 / 512.0;
    let width = 2.5e-4;
    let u = move |s: f64| 0.05 - (0.05 - depth) * (-((s - centre) / width).powi(2)).exp();
    let mut ss = uniform(400);
    ss.extend((1..400).map(|k| centre - 8.0 * width + 16.0 * width * f64::from(k) / 400.0));
    ss.sort_by(f64::total_cmp);
    ss.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let section = edge(
        &mut nodes,
        &map,
        torus_section(|s| (u(s), 0.2 + 1.0 * s), &ss),
        0.0,
        1.0,
    );
    (map, nodes, vec![seam, section])
}

#[test]
fn seam_excursion_between_samples_declines() {
    let (map, nodes, edges) = seam_bump(-2e-5);
    let EdgeCurve::NurbsCurve(n) = &edges[1].piece.curve else {
        unreachable!()
    };
    let u_at = |t: f64| uv(&map, n.evaluate(t)).x();
    // A 512-sample sweep sees no excursion; a dense one does.
    assert!((0..=512).all(|k| u_at(f64::from(k) / 512.0) > 0.0));
    assert!((0..=200_000).any(|k| u_at(f64::from(k) / 200_000.0) < 0.0));
    assert!(certify(&map, &edges, &nodes, &cfg()).is_none());
}

#[test]
fn seam_graze_declines_and_a_near_miss_certifies() {
    // Crossing the seam by 1e-8 rad: two crossings a few 1e-6 apart.
    let (map, nodes, edges) = seam_bump(-1e-8);
    let EdgeCurve::NurbsCurve(n) = &edges[1].piece.curve else {
        unreachable!()
    };
    assert!((0..=400_000).any(|k| uv(&map, n.evaluate(f64::from(k) / 400_000.0)).x() < 0.0));
    assert!(certify(&map, &edges, &nodes, &cfg()).is_none());
    let (map, nodes, edges) = seam_bump(2e-3);
    assert!(certify(&map, &edges, &nodes, &cfg()).is_some());
}

#[test]
fn rim_graze_without_a_node_declines() {
    // A rim parallel at v = 1.0 and a section rising to graze it: it pokes
    // 1e-6 rad over the rim for 3e-3 rad of u, between any coarse samples.
    let map = torus_map();
    let mut nodes = Vec::new();
    let rim = edge(&mut nodes, &map, parallel(1.0), -0.5, 0.5);
    let section = |gap: f64, nodes: &mut Vec<Node>| {
        edge(
            nodes,
            &map,
            torus_section(
                |s| {
                    let u = -0.4 + 0.8 * s;
                    (u, 1.0 - gap - 0.5 * u * u)
                },
                &uniform(200),
            ),
            0.0,
            1.0,
        )
    };
    let grazing = section(-1e-6, &mut nodes);
    let EdgeCurve::NurbsCurve(n) = &grazing.piece.curve else {
        unreachable!()
    };
    assert!((0..=100_000).any(|k| uv(&map, n.evaluate(f64::from(k) / 100_000.0)).y() > 1.0));
    assert!(certify(&map, &[rim.clone(), grazing], &nodes, &cfg()).is_none());
    let clear = section(1e-3, &mut nodes);
    assert!(certify(&map, &[rim, clear], &nodes, &cfg()).is_some());
}

/// A rational quadratic with a heavy middle weight: almost all of its
/// parameter range is spent near its ends, and its apex region is crossed in
/// a sliver of parameter. The certificate does not depend on the
/// parameterisation.
fn compressed(weight: f64) -> EdgeCurve {
    EdgeCurve::NurbsCurve(
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            vec![1.0, weight, 1.0],
        )
        .unwrap(),
    )
}

#[test]
fn compressed_rational_spans_are_certified_exactly() {
    // Apex height w / (1 + w) = 0.98039 for w = 50.
    let map = plane_map();
    let bar = |y: f64, nodes: &mut Vec<Node>| {
        edge_between(
            nodes,
            &map,
            EdgeCurve::Line,
            0.0,
            1.0,
            Point3::new(0.9, y, 0.0),
            Point3::new(1.1, y, 0.0),
        )
    };
    let mut nodes = Vec::new();
    let arc = edge(&mut nodes, &map, compressed(50.0), 0.0, 1.0);
    let crossing = bar(0.975, &mut nodes);
    assert!(certify(&map, &[arc.clone(), crossing], &nodes, &cfg()).is_none());
    let above = bar(0.985, &mut nodes);
    assert!(certify(&map, &[arc, above], &nodes, &cfg()).is_some());
}

#[test]
fn non_positive_weights_are_outside_the_family() {
    let map = plane_map();
    for w in [0.0, -0.5] {
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            vec![1.0, w, 1.0],
        );
        let Ok(curve) = curve else {
            continue;
        };
        let mut nodes = Vec::new();
        let e = edge_between(
            &mut nodes,
            &map,
            EdgeCurve::NurbsCurve(curve),
            0.0,
            1.0,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        );
        assert!(certify(&map, &[e], &nodes, &cfg()).is_none());
    }
}

#[test]
fn pieces_leaving_a_node_tangentially_decline() {
    // Two arcs leaving both their nodes in the same directions: no sector
    // separates them, so their contact is not confined to the nodes.
    let map = plane_map();
    let mut nodes = Vec::new();
    let a = edge(&mut nodes, &map, compressed(1.0), 0.0, 1.0);
    let b = edge(&mut nodes, &map, compressed(2.0), 0.0, 1.0);
    let mut edges = vec![a, b];
    edges[1].node_a = edges[0].node_a;
    edges[1].node_b = edges[0].node_b;
    assert!(certify(&map, &edges, &nodes, &cfg()).is_none());
    // Different tangents at both ends: a certified lens.
    let mut nodes = Vec::new();
    let a = edge(&mut nodes, &map, compressed(1.0), 0.0, 1.0);
    let flat = edge_between(
        &mut nodes,
        &map,
        EdgeCurve::Line,
        0.0,
        1.0,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    );
    let mut edges = vec![a, flat];
    edges[1].node_a = edges[0].node_a;
    edges[1].node_b = edges[0].node_b;
    let cert = certify(&map, &edges, &nodes, &cfg()).unwrap();
    // The arc leaves its first node at 45°, the line at 0°.
    assert!((cert.edges[0].leave_a - std::f64::consts::FRAC_PI_4).abs() < 0.2);
    assert!(cert.edges[1].leave_a.abs() < 1e-6);
}
