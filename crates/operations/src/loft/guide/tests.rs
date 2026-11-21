//! Guided loft: closed-form checks.
//!
//! Every case keeps each section horizontal and the guide's height a
//! monotone function of its parameter, so the loft's cross-section at height
//! `z` is the interpolated section translated sideways: Cavalieri gives the
//! volume in closed form, independent of how far the guide bends.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::nurbs::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::loft_with_guide;
use crate::OperationsError;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// An axis-aligned square of side `side` at height `z`, its corner (−1, −1)
/// first, counter-clockwise.
fn square(topo: &mut Topology, side: f64, z: f64) -> FaceId {
    let s = side - 1.0;
    polygon(
        topo,
        &[p(-1.0, -1.0, z), p(s, -1.0, z), p(s, s, z), p(-1.0, s, z)],
    )
}

fn polygon(topo: &mut Topology, points: &[Point3]) -> FaceId {
    let wire = make_polygon_wire(topo, points, 1e-7).unwrap();
    let z = points[0].z();
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

fn bezier(control_points: Vec<Point3>, weights: Vec<f64>) -> NurbsCurve {
    let p = control_points.len() - 1;
    let mut knots = vec![0.0; p + 1];
    knots.extend(std::iter::repeat_n(1.0, p + 1));
    NurbsCurve::new(p, knots, control_points, weights).unwrap()
}

fn assert_valid(topo: &Topology, solid: SolidId) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::solid_volume(topo, solid, 0.01).unwrap()
}

fn walls(topo: &Topology, solid: SolidId) -> Vec<remus_math::nurbs::surface::NurbsSurface> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|f| match topo.face(f).unwrap().surface() {
            FaceSurface::Nurbs(s) => Some(s.clone()),
            _ => None,
        })
        .collect()
}

/// The guide is a wall boundary: exactly two walls (one each side of the
/// guide corner) carry `guide(u)` along their `v = 0` or `v = 1` edge, at
/// every sampled `u`, and one solid edge is the guide.
fn assert_guide_on_surface(topo: &Topology, solid: SolidId, guide: &NurbsCurve) {
    let (a, b) = guide.domain();
    let on_guide = |at: &dyn Fn(f64) -> Point3| {
        (0..=40).all(|j| {
            let u = f64::from(j) / 40.0;
            (at(u) - guide.evaluate(a + u * (b - a))).length() < 1e-9
        })
    };
    let carriers = walls(topo, solid)
        .iter()
        .filter(|s| on_guide(&|u| s.evaluate(u, 0.0)) || on_guide(&|u| s.evaluate(u, 1.0)))
        .count();
    assert_eq!(carriers, 2, "walls bounded by the guide");
    let guide_edges = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| match topo.edge(e).unwrap().curve() {
            remus_topology::edge::EdgeCurve::NurbsCurve(c) => on_guide(&|u| c.evaluate(u)),
            _ => false,
        })
        .count();
    assert_eq!(guide_edges, 1, "one solid edge is the guide");
}

/// At `u = t` every wall runs along the boundary of `section`: its points
/// stay at the section's height and on one of the section's edges.
fn assert_section_interpolated(topo: &Topology, solid: SolidId, t: f64, section: &[Point3]) {
    let n = section.len();
    let on_boundary = |q: Point3| {
        (0..n).any(|i| {
            let (s, e) = (section[i], section[(i + 1) % n]);
            let d = e - s;
            let r = (q - s).dot(d) / d.dot(d);
            (-1e-12..=1.0 + 1e-12).contains(&r) && (q - (s + d * r)).length() < 1e-9
        })
    };
    for wall in walls(topo, solid) {
        for j in 0..=10 {
            let q = wall.evaluate(t, f64::from(j) / 10.0);
            assert!(on_boundary(q), "{q:?} at u = {t} is off the section");
        }
    }
}

/// Two equal squares 10 apart, the corner (−1, −1) guided along the parabola
/// `x = −1 + 6u(1 − u)`, `z = 10u`. Every horizontal slice is the square
/// translated by `6u(1 − u)` in x, so the volume stays exactly 2·2·10 = 40
/// and the centroid moves by the mean shift, `∫6u(1 − u) du = 1`.
#[test]
fn a_parabolic_guide_carries_the_square_along_it() {
    let mut topo = Topology::new();
    let (a, b) = (square(&mut topo, 2.0, 0.0), square(&mut topo, 2.0, 10.0));
    let guide = bezier(
        vec![p(-1.0, -1.0, 0.0), p(2.0, -1.0, 5.0), p(-1.0, -1.0, 10.0)],
        vec![1.0; 3],
    );
    let solid = loft_with_guide(&mut topo, &[a, b], &guide, false).unwrap();
    assert_valid(&topo, solid);
    assert_guide_on_surface(&topo, solid, &guide);
    let measured = volume(&topo, solid);
    assert!((measured - 40.0).abs() < 1e-6, "{measured}");
    let centroid = crate::measure::solid_center_of_mass(&topo, solid, 0.01).unwrap();
    assert!(
        (centroid.x() - 1.0).abs() < 1e-6 && centroid.y().abs() < 1e-6,
        "{centroid:?}"
    );
    // Every face is exact: two plane caps and four NURBS walls.
    assert_eq!(walls(&topo, solid).len(), 4);
    assert_eq!(solid_faces(&topo, solid).unwrap().len(), 6);
}

/// The same loft from a guide drawn top to bottom.
#[test]
fn a_reversed_guide_builds_the_same_loft() {
    let mut topo = Topology::new();
    let (a, b) = (square(&mut topo, 2.0, 0.0), square(&mut topo, 2.0, 10.0));
    let guide = bezier(
        vec![p(-1.0, -1.0, 10.0), p(2.0, -1.0, 5.0), p(-1.0, -1.0, 0.0)],
        vec![1.0; 3],
    );
    let solid = loft_with_guide(&mut topo, &[a, b], &guide, true).unwrap();
    assert_valid(&topo, solid);
    assert_guide_on_surface(&topo, solid, &guide.reversed());
    assert!((volume(&topo, solid) - 40.0).abs() < 1e-6);
}

/// Three squares of sides 2, 3, 2, their corners (−1, −1) on a cubic guide
/// that sways ±0.29 in x and rises linearly. The smooth loft's side is
/// `a(u) = 2 + 4u(1 − u)` at height 10u, so its volume is
/// `10·∫(2 + 4u − 4u²)² du = 10·(4 + 8 − 8 + 16/5) = 72`.
#[test]
fn a_smooth_loft_follows_a_cubic_guide_through_three_sections() {
    let mut topo = Topology::new();
    let sections = [
        square(&mut topo, 2.0, 0.0),
        square(&mut topo, 3.0, 5.0),
        square(&mut topo, 2.0, 10.0),
    ];
    // x(u) = −1 + 3·u(1 − u)(2u − 1): Bernstein coefficients 0, −1/3, 1/3, 0.
    let guide = bezier(
        vec![
            p(-1.0, -1.0, 0.0),
            p(-2.0, -1.0, 10.0 / 3.0),
            p(0.0, -1.0, 20.0 / 3.0),
            p(-1.0, -1.0, 10.0),
        ],
        vec![1.0; 4],
    );
    let solid = loft_with_guide(&mut topo, &sections, &guide, true).unwrap();
    assert_valid(&topo, solid);
    assert_guide_on_surface(&topo, solid, &guide);
    let middle = [
        p(-1.0, -1.0, 5.0),
        p(2.0, -1.0, 5.0),
        p(2.0, 2.0, 5.0),
        p(-1.0, 2.0, 5.0),
    ];
    assert_section_interpolated(&topo, solid, 0.5, &middle);
    let measured = volume(&topo, solid);
    assert!((measured - 72.0).abs() < 1e-6, "{measured}");
}

/// A 90° circular arc (rational quadratic, middle weight √2/2) through the
/// corners of three equal squares. The arc's height rises monotonically,
/// so the volume is still exactly 40, and each wall stays a straight blend
/// between rails that share the arc's weights.
#[test]
fn an_arc_guide_is_exact_in_homogeneous_form() {
    let mut topo = Topology::new();
    let r = 50.0f64.sqrt();
    let apex = p(-6.0 + r, -1.0, 5.0);
    let sections = [
        square(&mut topo, 2.0, 0.0),
        polygon(
            &mut topo,
            &[
                apex,
                apex + Vec3::new(2.0, 0.0, 0.0),
                apex + Vec3::new(2.0, 2.0, 0.0),
                apex + Vec3::new(0.0, 2.0, 0.0),
            ],
        ),
        square(&mut topo, 2.0, 10.0),
    ];
    let guide = bezier(
        vec![p(-1.0, -1.0, 0.0), p(4.0, -1.0, 5.0), p(-1.0, -1.0, 10.0)],
        vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
    );
    for smooth in [false, true] {
        let solid = loft_with_guide(&mut topo, &sections, &guide, smooth).unwrap();
        assert_valid(&topo, solid);
        assert_guide_on_surface(&topo, solid, &guide);
        let measured = volume(&topo, solid);
        assert!(
            (measured - 40.0).abs() < 1e-6,
            "smooth {smooth}: {measured}"
        );
        // Every point of the guide is on the arc of radius r about (−6, −1, 5).
        for wall in walls(&topo, solid) {
            assert!(wall.weights().iter().flatten().any(|&w| w < 1.0 - 1e-6));
        }
    }
}

/// The arc again, through squares of sides 2, 3, 2: the ruled offsets grow
/// the side linearly in the arc's rational parameter, `a(u) = 3 − |2u − 1|`
/// (2 → 3 → 2), while the height follows the arc. Each slice is still a
/// horizontal square, so the volume is `∫ a(u)² z′(u) du`, integrated here
/// independently by Simpson's rule on the closed-form arc.
#[test]
fn an_arc_guide_with_growing_sections_matches_its_integral() {
    let mut topo = Topology::new();
    let r = 50.0f64.sqrt();
    let apex = p(-6.0 + r, -1.0, 5.0);
    let middle = [
        apex,
        apex + Vec3::new(3.0, 0.0, 0.0),
        apex + Vec3::new(3.0, 3.0, 0.0),
        apex + Vec3::new(0.0, 3.0, 0.0),
    ];
    let sections = [
        square(&mut topo, 2.0, 0.0),
        polygon(&mut topo, &middle),
        square(&mut topo, 2.0, 10.0),
    ];
    let guide = bezier(
        vec![p(-1.0, -1.0, 0.0), p(4.0, -1.0, 5.0), p(-1.0, -1.0, 10.0)],
        vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
    );
    let solid = loft_with_guide(&mut topo, &sections, &guide, false).unwrap();
    assert_valid(&topo, solid);
    assert_guide_on_surface(&topo, solid, &guide);
    assert_section_interpolated(&topo, solid, 0.5, &middle);

    let side = |u: f64| 3.0 - (2.0 * u - 1.0).abs();
    let height_rate = |u: f64| guide.derivatives(u, 1)[1].z();
    let steps = 4000;
    let h = 1.0 / f64::from(steps);
    let exact: f64 = (0..=steps)
        .map(|j| {
            let u = f64::from(j) * h;
            let weight = if j == 0 || j == steps {
                1.0
            } else if j % 2 == 1 {
                4.0
            } else {
                2.0
            };
            weight * side(u).powi(2) * height_rate(u)
        })
        .sum::<f64>()
        * h
        / 3.0;
    let measured = volume(&topo, solid);
    assert!((measured - exact).abs() < 1e-6, "{measured} vs {exact}");
}

/// A two-span polyline guide whose knot meets the middle section: the
/// section parameter snaps onto the guide's knot, so the walls carry no
/// sliver span. Every slice is a translated square: volume 40.
#[test]
fn a_polyline_guide_shares_its_knot_with_the_middle_section() {
    let mut topo = Topology::new();
    let kink = p(0.5, -1.0, 5.0);
    let sections = [
        square(&mut topo, 2.0, 0.0),
        polygon(
            &mut topo,
            &[
                kink,
                kink + Vec3::new(2.0, 0.0, 0.0),
                kink + Vec3::new(2.0, 2.0, 0.0),
                kink + Vec3::new(0.0, 2.0, 0.0),
            ],
        ),
        square(&mut topo, 2.0, 10.0),
    ];
    let guide = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 2.5, 5.0, 5.0],
        vec![p(-1.0, -1.0, 0.0), kink, p(-1.0, -1.0, 10.0)],
        vec![1.0; 3],
    )
    .unwrap();
    let solid = loft_with_guide(&mut topo, &sections, &guide, false).unwrap();
    assert_valid(&topo, solid);
    assert_guide_on_surface(&topo, solid, &guide);
    for wall in walls(&topo, solid) {
        assert_eq!(wall.knots_u(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
    }
    assert!((volume(&topo, solid) - 40.0).abs() < 1e-6);
}

/// A loft without a guide is untouched by the shared rail builder: the
/// smooth loft of the same three squares still measures its closed form.
#[test]
fn unguided_smooth_loft_is_unchanged() {
    let mut topo = Topology::new();
    let sections = [
        square(&mut topo, 2.0, 0.0),
        square(&mut topo, 2.0, 5.0),
        square(&mut topo, 2.0, 10.0),
    ];
    let solid = crate::loft::loft_smooth(&mut topo, &sections).unwrap();
    assert_valid(&topo, solid);
    assert!((volume(&topo, solid) - 40.0).abs() < 1e-6);
}

fn refusal(sections: impl Fn(&mut Topology) -> Vec<FaceId>, guide: &NurbsCurve) -> OperationsError {
    let mut topo = Topology::new();
    let faces = sections(&mut topo);
    let before = topo.num_faces();
    let error = loft_with_guide(&mut topo, &faces, guide, true).unwrap_err();
    assert_eq!(topo.num_faces(), before, "{error}");
    error
}

#[test]
fn guided_loft_refusals_are_typed_and_leave_the_topology_unchanged() {
    let straight = bezier(vec![p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)], vec![1.0; 2]);
    let two_squares = |topo: &mut Topology| vec![square(topo, 2.0, 0.0), square(topo, 2.0, 10.0)];

    // No exact construction: a curved section, unequal corner counts.
    let curved = |topo: &mut Topology| {
        use remus_topology::builder::make_circle_edge;
        use remus_topology::wire::{OrientedEdge, Wire};
        let edge =
            make_circle_edge(topo, p(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0), 1.0, 1e-7).unwrap();
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        let disc = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 10.0,
            },
        ));
        vec![square(topo, 2.0, 0.0), disc]
    };
    let triangle = |topo: &mut Topology| {
        vec![
            square(topo, 2.0, 0.0),
            polygon(
                topo,
                &[p(-1.0, -1.0, 10.0), p(1.0, -1.0, 10.0), p(-1.0, 1.0, 10.0)],
            ),
        ]
    };
    for (label, error, phrase) in [
        ("curved", refusal(curved, &straight), "curved edge"),
        ("corners", refusal(triangle, &straight), "one to one"),
    ] {
        let OperationsError::Unsupported { operation, reason } = &error else {
            panic!("{label}: untyped {error:?}");
        };
        assert_eq!(*operation, "loft_with_guide");
        assert!(reason.contains(phrase), "{label}: {reason}");
    }

    // Wrong arguments.
    let misses = bezier(vec![p(-1.0, -1.0, 0.0), p(-1.0, 0.0, 10.0)], vec![1.0; 2]);
    let overshoots = bezier(vec![p(-1.0, -1.0, -1.0), p(-1.0, -1.0, 11.0)], vec![1.0; 2]);
    let unclamped = NurbsCurve::new(
        1,
        vec![0.0, 0.5, 1.0, 1.5, 2.0],
        vec![p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0), p(-1.0, -1.0, 20.0)],
        vec![1.0; 3],
    )
    .unwrap();
    let one_section = |topo: &mut Topology| vec![square(topo, 2.0, 0.0)];
    for (label, error, phrase) in [
        (
            "misses",
            refusal(two_squares, &misses),
            "does not pass through a corner of section 1",
        ),
        (
            "overshoots",
            refusal(two_squares, &overshoots),
            "must start on the first section",
        ),
        ("unclamped", refusal(two_squares, &unclamped), "clamped"),
        (
            "one section",
            refusal(one_section, &straight),
            "at least 2 profiles",
        ),
    ] {
        let OperationsError::InvalidInput { reason } = &error else {
            panic!("{label}: untyped {error:?}");
        };
        assert!(reason.contains(phrase), "{label}: {reason}");
    }
}

/// Review regression: a guide that climbs to z = 7, drops to z = 3 and
/// climbs again would cover the z = 3..7 strip three times with every wall.
/// It is refused by name, never built.
#[test]
fn a_guide_that_doubles_back_is_refused() {
    let guide = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0],
        vec![
            p(-1.0, -1.0, 0.0),
            p(-1.0, -1.0, 7.0),
            p(-1.0, -1.0, 3.0),
            p(-1.0, -1.0, 10.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    for smooth in [false, true] {
        let mut topo = Topology::new();
        let faces = [square(&mut topo, 2.0, 0.0), square(&mut topo, 2.0, 10.0)];
        let before = topo.num_faces();
        let error = loft_with_guide(&mut topo, &faces, &guide, smooth).unwrap_err();
        let OperationsError::Unsupported { operation, reason } = &error else {
            panic!("untyped {error:?}");
        };
        assert_eq!(*operation, "loft_with_guide");
        assert!(reason.contains("doubles back"), "{reason}");
        assert_eq!(topo.num_faces(), before);
    }
}

/// Review regression: the upper square drawn clockwise (with its matching
/// downward normal) is rewound on its own, keeping the guide corner first,
/// so the loft stays the 2 × 2 × 10 prism instead of collapsing at z = 5.
#[test]
fn opposite_section_windings_still_pair_corner_for_corner() {
    let mut topo = Topology::new();
    let lower = square(&mut topo, 2.0, 0.0);
    let wire = make_polygon_wire(
        &mut topo,
        &[
            p(-1.0, -1.0, 10.0),
            p(-1.0, 1.0, 10.0),
            p(1.0, 1.0, 10.0),
            p(1.0, -1.0, 10.0),
        ],
        1e-7,
    )
    .unwrap();
    let upper = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, -1.0),
            d: -10.0,
        },
    ));
    let guide = bezier(vec![p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)], vec![1.0; 2]);
    for smooth in [false, true] {
        let solid = loft_with_guide(&mut topo, &[lower, upper], &guide, smooth).unwrap();
        assert_valid(&topo, solid);
        let middle = [
            p(-1.0, -1.0, 5.0),
            p(1.0, -1.0, 5.0),
            p(1.0, 1.0, 5.0),
            p(-1.0, 1.0, 5.0),
        ];
        assert_section_interpolated(&topo, solid, 0.5, &middle);
        assert!((volume(&topo, solid) - 40.0).abs() < 1e-6);
    }
}

/// Review regression: weights 1, 1 + 8e-13, 1 on a guide four million units
/// long are a rational curve, not a polynomial one: making them equal moved
/// the guide by about 4e-7. The loft's guide edge stays on the given curve.
#[test]
fn nearly_equal_weights_keep_the_guide_rational() {
    let mut topo = Topology::new();
    let side = |topo: &mut Topology, z: f64| {
        polygon(
            topo,
            &[
                p(0.0, 0.0, z),
                p(2000.0, 0.0, z),
                p(2000.0, 2000.0, z),
                p(0.0, 2000.0, z),
            ],
        )
    };
    let faces = [side(&mut topo, 0.0), side(&mut topo, 4e6)];
    let guide = bezier(
        vec![p(0.0, 0.0, 0.0), p(2e6, 0.0, 2e6), p(0.0, 0.0, 4e6)],
        vec![1.0, 1.0 + 8e-13, 1.0],
    );
    let solid = loft_with_guide(&mut topo, &faces, &guide, false).unwrap();
    let deviation = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|e| match topo.edge(e).unwrap().curve() {
            remus_topology::edge::EdgeCurve::NurbsCurve(c) => Some(c.clone()),
            _ => None,
        })
        .map(|c| {
            (0..=200)
                .map(|j| {
                    let u = f64::from(j) / 200.0;
                    (c.evaluate(u) - guide.evaluate(u)).length()
                })
                .fold(0.0f64, f64::max)
        })
        .fold(f64::INFINITY, f64::min);
    assert!(deviation < 1e-8, "guide edge is {deviation} off the guide");
}

/// Review regression: the middle section sits at guide parameter
/// 0.5000000005, half a nanounit past the polyline guide's knot at 0.5.
/// Snapping the offset's bend onto that knot would make the width at u = 0.5
/// 20000 instead of the interpolated 10000 + 10000·0.5/0.5000000005, 1e-5
/// away; the knot is kept instead.
#[test]
fn snapping_never_moves_an_offset_beyond_tolerance() {
    let mut topo = Topology::new();
    let guide = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 5.0), p(0.0, 0.0, 10.0)],
        vec![1.0; 3],
    )
    .unwrap();
    let t1 = 0.500_000_000_5;
    let sections: Vec<FaceId> = [(0.0, 10_000.0), (t1, 20_000.0), (1.0, 10_000.0)]
        .into_iter()
        .map(|(t, s)| {
            let c = guide.evaluate(t);
            polygon(
                &mut topo,
                &[
                    c,
                    c + Vec3::new(s, 0.0, 0.0),
                    c + Vec3::new(s, s, 0.0),
                    c + Vec3::new(0.0, s, 0.0),
                ],
            )
        })
        .collect();
    let solid = loft_with_guide(&mut topo, &sections, &guide, false).unwrap();
    let expected = 10_000.0 + 10_000.0 * 0.5 / t1;
    let widths: Vec<f64> = walls(&topo, solid)
        .iter()
        .map(|w| (w.evaluate(0.5, 1.0) - w.evaluate(0.5, 0.0)).length())
        .collect();
    assert!(
        widths.iter().any(|w| (w - expected).abs() < 1e-7),
        "{widths:?} vs {expected}"
    );
}

fn expect_doubling_back(faces: impl Fn(&mut Topology) -> Vec<FaceId>, guide: &NurbsCurve) {
    let mut topo = Topology::new();
    let faces = faces(&mut topo);
    let error = loft_with_guide(&mut topo, &faces, guide, false).unwrap_err();
    let OperationsError::Unsupported { reason, .. } = &error else {
        panic!("untyped {error:?}");
    };
    assert!(reason.contains("doubles back"), "{reason}");
}

/// Re-check regression: the upper square lies on the tilted plane
/// z = 10 + x + y and the guide reaches its corner from the far side of that
/// plane (final tangent (5, 0, 4) against the normal (−1, −1, 1)). Flipping
/// that ring alone swapped the corners beside the guide corner and the rails
/// crossed at u = 0.5; the inconsistent correspondence is refused.
#[test]
fn a_guide_crossing_a_section_backwards_is_refused() {
    let mut topo = Topology::new();
    let lower = square(&mut topo, 2.0, 0.0);
    let corners = [
        p(-1.0, -1.0, 8.0),
        p(1.0, -1.0, 10.0),
        p(1.0, 1.0, 12.0),
        p(-1.0, 1.0, 10.0),
    ];
    let wire = make_polygon_wire(&mut topo, &corners, 1e-7).unwrap();
    let normal = Vec3::new(-1.0, -1.0, 1.0).normalize().unwrap();
    let upper = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: normal.dot(Vec3::new(-1.0, -1.0, 8.0)),
        },
    ));
    let guide = bezier(
        vec![p(-1.0, -1.0, 0.0), p(-6.0, -1.0, 4.0), p(-1.0, -1.0, 8.0)],
        vec![1.0; 3],
    );
    for smooth in [false, true] {
        let before = topo.num_faces();
        let error = loft_with_guide(&mut topo, &[lower, upper], &guide, smooth).unwrap_err();
        let OperationsError::Unsupported { reason, .. } = &error else {
            panic!("untyped {error:?}");
        };
        assert!(reason.contains("against the direction"), "{reason}");
        assert_eq!(topo.num_faces(), before);
    }
}

/// Re-check regression: four back-steps of 7.5e-8 each pass a per-step
/// tolerance but retrace 3e-7 in total; progress is measured against the
/// best reached so far.
#[test]
fn small_back_steps_that_add_up_are_refused() {
    let mut points = vec![p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 5.0)];
    for j in 1..=4 {
        points.push(p(-1.0, -1.0, 7.5e-8f64.mul_add(-f64::from(j), 5.0)));
    }
    points.push(p(-1.0, -1.0, 10.0));
    let n = points.len();
    let mut knots = vec![0.0, 0.0];
    #[allow(clippy::cast_precision_loss)]
    knots.extend((1..n - 1).map(|j| j as f64 / (n - 1) as f64));
    knots.extend([1.0, 1.0]);
    let guide = NurbsCurve::new(1, knots, points, vec![1.0; n]).unwrap();
    expect_doubling_back(
        |topo| vec![square(topo, 2.0, 0.0), square(topo, 2.0, 10.0)],
        &guide,
    );
}

/// Re-check regression: a straight frustum from a side-20 square about the
/// origin to a side-2 square about (5, 0, 1) leans away from the centroid
/// axis on one rail, yet advances steadily between the two planes. Its
/// volume is h/3·(A₁ + A₂ + √(A₁A₂)) = (400 + 4 + 40)/3 = 148.
#[test]
fn an_oblique_frustum_advances_between_its_section_planes() {
    let mut topo = Topology::new();
    let base = polygon(
        &mut topo,
        &[
            p(-10.0, -10.0, 0.0),
            p(10.0, -10.0, 0.0),
            p(10.0, 10.0, 0.0),
            p(-10.0, 10.0, 0.0),
        ],
    );
    let top = polygon(
        &mut topo,
        &[
            p(4.0, -1.0, 1.0),
            p(6.0, -1.0, 1.0),
            p(6.0, 1.0, 1.0),
            p(4.0, 1.0, 1.0),
        ],
    );
    let guide = bezier(vec![p(-10.0, -10.0, 0.0), p(4.0, -1.0, 1.0)], vec![1.0; 2]);
    let solid = loft_with_guide(&mut topo, &[base, top], &guide, false).unwrap();
    assert_valid(&topo, solid);
    assert_guide_on_surface(&topo, solid, &guide);
    let measured = volume(&topo, solid);
    assert!((measured - 148.0).abs() < 1e-6, "{measured}");
    // The doubling-back guide is still refused under the per-interval rule.
    let back = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0],
        vec![
            p(-1.0, -1.0, 0.0),
            p(-1.0, -1.0, 7.0),
            p(-1.0, -1.0, 3.0),
            p(-1.0, -1.0, 10.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    expect_doubling_back(
        |topo| vec![square(topo, 2.0, 0.0), square(topo, 2.0, 10.0)],
        &back,
    );
}

/// Re-check regression: two interior sections 7.5e-10 either side of the
/// guide's knot at 0.5 both snapped onto it, doubling the knot and breaking
/// the ruled loft. At most one snaps; this prism measures 2·2·10 = 40.
#[test]
fn two_offsets_never_snap_onto_one_guide_knot() {
    let mut topo = Topology::new();
    let guide = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        vec![p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 5.0), p(-1.0, -1.0, 10.0)],
        vec![1.0; 3],
    )
    .unwrap();
    let sections: Vec<FaceId> = [0.0, 0.5 - 7.5e-10, 0.5 + 7.5e-10, 1.0]
        .into_iter()
        .map(|t| square(&mut topo, 2.0, guide.evaluate(t).z()))
        .collect();
    let solid = loft_with_guide(&mut topo, &sections, &guide, false).unwrap();
    assert_valid(&topo, solid);
    assert!((volume(&topo, solid) - 40.0).abs() < 1e-6);
}

/// Re-check regression: controls (−1, −1, 0) twice then (−1, −1, 10) make a
/// straight guide whose derivative vanishes at its start. Its geometric
/// direction there is still +z, so the prism builds: volume 40.
#[test]
fn a_stationary_guide_start_still_has_a_direction() {
    let guide = bezier(
        vec![p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)],
        vec![1.0; 3],
    );
    for smooth in [false, true] {
        let mut topo = Topology::new();
        let faces = [square(&mut topo, 2.0, 0.0), square(&mut topo, 2.0, 10.0)];
        let solid = loft_with_guide(&mut topo, &faces, &guide, smooth).unwrap();
        assert_valid(&topo, solid);
        assert!((volume(&topo, solid) - 40.0).abs() < 1e-6);
    }
}
