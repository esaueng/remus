//! Native equator partition and whole-interval circular rim coverage oracles.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::curves::Circle3D;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

use super::{OrientedPCurveEdge, is_sphere_equator_partition};

fn fixture(segments: usize, forward: bool, scale: f64) -> (FaceSurface, Vec<OrientedPCurveEdge>) {
    let center = Point3::new(7.0 * scale, -3.0 * scale, 2.0 * scale);
    let axis = Vec3::new(1.0, 2.0, 3.0).normalize().unwrap();
    let circle = Circle3D::new(center, axis, scale).unwrap();
    let surface = FaceSurface::Sphere(SphericalSurface::with_axis(center, scale, axis).unwrap());
    let edges = (0..segments)
        .map(|index| {
            let part = if forward { index } else { segments - 1 - index };
            let a = TAU * part as f64 / segments as f64;
            let b = TAU * (part + 1) as f64 / segments as f64;
            let (start, end) = if forward { (a, b) } else { (b, a) };
            OrientedPCurveEdge {
                curve_3d: EdgeCurve::Circle(circle.clone()),
                trim: Some((a, b)),
                pcurve: Curve2D::Line(
                    Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
                ),
                start_uv: Point2::new(start, 0.0),
                end_uv: Point2::new(end, 0.0),
                start_3d: circle.evaluate(start),
                end_3d: circle.evaluate(end),
                forward,
                source_edge_idx: None,
                pave_block_id: None,
                source_topo_edge: None,
            }
        })
        .collect();
    (surface, edges)
}

#[test]
fn complete_equators_admit_both_directions_scales_and_wire_starts() {
    for scale in [0.001, 1.0, 1000.0] {
        for segments in [4, 8, 16, 32] {
            for forward in [false, true] {
                let (surface, mut edges) = fixture(segments, forward, scale);
                assert!(is_sphere_equator_partition(&surface, &edges, 1e-7));
                edges.rotate_left(segments / 2);
                assert!(is_sphere_equator_partition(&surface, &edges, 1e-7));
                for edge in &mut edges {
                    let (a, b) = edge.trim.unwrap();
                    edge.trim = Some((b, a));
                    edge.forward = !edge.forward;
                }
                assert!(is_sphere_equator_partition(&surface, &edges, 1e-7));
            }
        }
    }
}

#[test]
fn incomplete_retraced_and_malformed_partitions_decline() {
    let (surface, original) = fixture(16, true, 1.0);
    let mut edges = original.clone();
    edges.pop();
    assert!(!is_sphere_equator_partition(&surface, &edges, 1e-7));
    let mut edges = original.clone();
    edges.extend(original.clone());
    assert!(!is_sphere_equator_partition(&surface, &edges, 1e-7));
    let mut edges = original.clone();
    edges[3].trim = None;
    assert!(!is_sphere_equator_partition(&surface, &edges, 1e-7));
    let mut edges = original.clone();
    edges[3].trim = Some((f64::NAN, 1.0));
    assert!(!is_sphere_equator_partition(&surface, &edges, 1e-7));
    let mut edges = original.clone();
    edges[3].forward = false;
    assert!(!is_sphere_equator_partition(&surface, &edges, 1e-7));
    let mut edges = original.clone();
    edges[3].start_3d = Point3::new(f64::NAN, 0.0, 0.0);
    assert!(!is_sphere_equator_partition(&surface, &edges, 1e-7));
    let mut edges = original.clone();
    edges[3].curve_3d = EdgeCurve::Line;
    assert!(!is_sphere_equator_partition(&surface, &edges, 1e-7));
    assert!(!is_sphere_equator_partition(&surface, &original, f64::NAN));
}

#[test]
fn a_common_circle_that_is_not_the_sphere_equator_declines() {
    let (surface, edges) = fixture(16, true, 1.0);
    let FaceSurface::Sphere(sphere) = surface else {
        unreachable!()
    };
    let shifted = FaceSurface::Sphere(
        SphericalSurface::with_axis(
            sphere.center() + sphere.z_axis() * 0.25,
            1.0,
            sphere.z_axis(),
        )
        .unwrap(),
    );
    assert!(!is_sphere_equator_partition(&shifted, &edges, 1e-7));
}

fn arc_use(circle: Circle3D, trim: (f64, f64), forward: bool) -> OrientedPCurveEdge {
    let (from, to) = if forward { trim } else { (trim.1, trim.0) };
    OrientedPCurveEdge {
        start_3d: circle.evaluate(from),
        end_3d: circle.evaluate(to),
        curve_3d: EdgeCurve::Circle(circle),
        trim: Some(trim),
        forward,
        pcurve: Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
        start_uv: Point2::new(from, 0.0),
        end_uv: Point2::new(to, 0.0),
        source_edge_idx: None,
        pave_block_id: None,
        source_topo_edge: None,
    }
}

fn nurbs_use(points: Vec<Point3>, weights: Vec<f64>) -> OrientedPCurveEdge {
    let curve = remus_math::nurbs::curve::NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points,
        weights,
    )
    .unwrap();
    let mut edge = arc_use(
        Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap(),
        (0.0, 1.0),
        true,
    );
    edge.start_3d = curve.evaluate(0.0);
    edge.end_3d = curve.evaluate(1.0);
    edge.curve_3d = EdgeCurve::NurbsCurve(curve);
    edge
}

#[test]
fn positive_weight_nurbs_hulls_separated_from_rim_plane_cannot_cover_it() {
    for radius in [0.001, 1.0, 1000.0] {
        for placed in [false, true] {
            let center = if placed {
                Point3::new(10.0 * radius, -7.0 * radius, 3.0 * radius)
            } else {
                Point3::new(0.0, 0.0, 0.0)
            };
            let axis = if placed {
                Vec3::new(1.0, 2.0, 3.0)
            } else {
                Vec3::new(0.0, 0.0, 1.0)
            };
            let circle = Circle3D::new(center, axis, radius).unwrap();
            let rim = arc_use(circle.clone(), (0.0, std::f64::consts::FRAC_PI_2), true);
            for sign in [-1.0, 1.0] {
                let points =
                    [(1.0, 0.0, 0.5), (0.75, 0.75, 0.9), (0.0, 1.0, 0.5)].map(|(x, y, z)| {
                        center
                            + (circle.u_axis() * x
                                + circle.v_axis() * y
                                + circle.normal() * (sign * z))
                                * radius
                    });
                let mut arc = nurbs_use(points.to_vec(), vec![0.5, 2.0, 1.0]);
                assert_eq!(
                    super::circular_rim_coverage::covers(&arc, &rim, 1e-7),
                    Some(false),
                    "radius={radius} placed={placed} sign={sign}"
                );
                arc.trim = Some((0.9, 0.1));
                assert_eq!(
                    super::circular_rim_coverage::covers(&arc, &rim, 1e-7),
                    Some(false)
                );
                arc.trim = Some((-0.1, 0.9));
                assert_eq!(super::circular_rim_coverage::covers(&arc, &rim, 1e-7), None);
                arc.trim = Some((0.0, 1.0));
                arc.end_3d = center;
                assert_eq!(super::circular_rim_coverage::covers(&arc, &rim, 1e-7), None);
            }
        }
    }
}

#[test]
fn touching_crossing_and_circular_nurbs_hulls_keep_overlap_unresolved() {
    let center = Point3::new(0.0, 0.0, 0.0);
    let circle = Circle3D::new_with_ref(
        center,
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let rim = arc_use(circle, (0.0, std::f64::consts::FRAC_PI_2), true);
    for heights in [[1.0, -3.0, 1.0], [1.0, 0.0, 1.0], [5e-8; 3]] {
        let arc = nurbs_use(
            vec![
                Point3::new(1.0, 0.0, heights[0]),
                Point3::new(1.0, 1.0, heights[1]),
                Point3::new(0.0, 1.0, heights[2]),
            ],
            vec![1.0; 3],
        );
        assert_eq!(super::circular_rim_coverage::covers(&arc, &rim, 1e-7), None);
    }
    // This positive-weight rational quadratic is the rim's actual quarter
    // circle, so a blanket "NURBS cannot overlap" decision would be wrong.
    let coincident = nurbs_use(
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
    );
    assert_eq!(
        super::circular_rim_coverage::covers(&coincident, &rim, 1e-7),
        None
    );
}

#[test]
fn nonpositive_nurbs_weights_cannot_claim_the_control_hull_exclusion() {
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let rim = arc_use(circle, (0.0, std::f64::consts::FRAC_PI_2), true);
    let mut arc = nurbs_use(
        vec![
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 2.0),
            Point3::new(0.0, 1.0, 1.0),
        ],
        vec![1.0; 3],
    );
    let EdgeCurve::NurbsCurve(curve) = &arc.curve_3d else {
        panic!("NURBS fixture");
    };
    let mut value = serde_json::to_value(curve).unwrap();
    value["weights"] = serde_json::json!([1.0, -0.9, 1.0]);
    let curve: remus_math::nurbs::curve::NurbsCurve = serde_json::from_value(value).unwrap();
    // Every control point is above the rim plane, while the negative weight
    // sends the actual midpoint below it. The constructor rejects this case;
    // unchecked deserialization must not acquire a false whole-hull proof.
    assert!(curve.evaluate(0.5).z() < 0.0);
    arc.curve_3d = EdgeCurve::NurbsCurve(curve);
    assert_eq!(super::circular_rim_coverage::covers(&arc, &rim, 1e-7), None);
}

#[test]
fn native_circle_span_plane_exclusion_preserves_the_literal_tilted_equator_case() {
    let finish = 3.481_429_563_043_915;
    for scale in [0.001, 1.0, 1000.0] {
        for placed in [false, true] {
            let axis = Vec3::new(1.0, 2.0, 3.0).normalize().unwrap();
            let (sine, cosine) = 0.37_f64.sin_cos();
            let rotate = |v: Vec3| {
                if placed {
                    v * cosine + axis.cross(v) * sine + axis * (axis.dot(v) * (1.0 - cosine))
                } else {
                    v
                }
            };
            let center = if placed {
                Point3::new(10.0 * scale, -7.0 * scale, 3.0 * scale)
            } else {
                Point3::new(0.0, 0.0, -2.5 * scale)
            };
            let radius = 3.0_f64.next_down() * scale;
            let section = Circle3D::with_axes(
                center,
                rotate(Vec3::new(-1.0, 0.0, 0.0)),
                radius,
                rotate(Vec3::new(0.0, 0.0, -1.0)),
                rotate(Vec3::new(0.0, -1.0, 0.0)),
            )
            .unwrap();
            let rim = Circle3D::with_axes(
                center,
                rotate(Vec3::new(
                    0.500_000_000_000_000_1,
                    0.0,
                    0.866_025_403_784_438_8,
                )),
                radius,
                rotate(Vec3::new(
                    0.866_025_403_784_438_8,
                    0.0,
                    -0.500_000_000_000_000_1,
                )),
                rotate(Vec3::new(0.0, 1.0, 0.0)),
            )
            .unwrap();
            // In the source frame h(t)=-r*cos(30°)*cos(t). On this native
            // span h>=sqrt(8)*cos(30°)>2.44 at unit scale, although the two
            // complete Circle carriers intersect at the opposite Y directions.
            for forward in [false, true] {
                let arc = arc_use(section.clone(), (std::f64::consts::PI, finish), forward);
                let boundary = arc_use(rim.clone(), (0.0, std::f64::consts::FRAC_PI_8), forward);
                assert_eq!(
                    super::circular_rim_coverage::covers(&arc, &boundary, 1e-7),
                    Some(false),
                    "scale={scale} placed={placed} forward={forward}"
                );
            }
        }
    }
}

#[test]
fn native_circle_span_plane_exclusion_declines_major_crossings_and_contact() {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_8, PI};
    let center = Point3::new(0.0, 0.0, -2.5);
    let section = Circle3D::with_axes(
        center,
        Vec3::new(-1.0, 0.0, 0.0),
        3.0,
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, -1.0, 0.0),
    )
    .unwrap();
    let rim = Circle3D::with_axes(
        center,
        Vec3::new(0.5, 0.0, 3.0_f64.sqrt() / 2.0),
        3.0,
        Vec3::new(3.0_f64.sqrt() / 2.0, 0.0, -0.5),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let boundary = arc_use(rim, (0.0, FRAC_PI_8), true);
    for span in [
        (3.481_429_563_043_915, PI + TAU),
        (FRAC_PI_2, PI),
        (FRAC_PI_2 + 1e-9, PI),
    ] {
        let arc = arc_use(section.clone(), span, true);
        assert_eq!(
            super::circular_rim_coverage::covers(&arc, &boundary, 1e-7),
            None,
            "span={span:?}"
        );
    }
    let mut arc = arc_use(section, (PI, 3.481_429_563_043_915), true);
    arc.end_3d = center;
    assert_eq!(
        super::circular_rim_coverage::covers(&arc, &boundary, 1e-7),
        None
    );
}

#[test]
fn circle_rim_coverage_maps_complete_native_spans_across_frames_and_directions() {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_8};
    for radius in [0.001, 1.0, 1000.0] {
        for placed in [false, true] {
            let center = if placed {
                Point3::new(10.0 * radius, -7.0 * radius, 3.0 * radius)
            } else {
                Point3::new(0.0, 0.0, 0.0)
            };
            let normal = if placed {
                Vec3::new(1.0, 2.0, 3.0)
            } else {
                Vec3::new(0.0, 0.0, 1.0)
            };
            let a = Circle3D::new(center, normal, radius).unwrap();
            let phase = 0.37_f64;
            let reference = a.u_axis() * phase.cos() + a.v_axis() * phase.sin();
            for reverse_frame in [false, true] {
                let b = Circle3D::new_with_ref(
                    center,
                    a.normal() * if reverse_frame { -1.0 } else { 1.0 },
                    radius,
                    reference,
                )
                .unwrap();
                let sign = if reverse_frame { -1.0 } else { 1.0 };
                let native = ((FRAC_PI_8 - phase) * sign, (FRAC_PI_4 - phase) * sign);
                for forward in [false, true] {
                    let cap = arc_use(a.clone(), (0.0, FRAC_PI_2), forward);
                    let rim = arc_use(b.clone(), native, forward);
                    assert_eq!(
                        super::circular_rim_coverage::covers(&cap, &rim, 1e-7),
                        Some(true),
                        "radius={radius} placed={placed} reverse={reverse_frame} forward={forward}"
                    );
                }
            }
        }
    }
}

#[test]
fn circle_rim_major_gap_cannot_hide_between_covered_endpoints() {
    use std::f64::consts::{FRAC_PI_2, PI};
    let circle = Circle3D::new_with_ref(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let quarter = arc_use(circle.clone(), (0.0, FRAC_PI_2), true);
    let long_rim = arc_use(circle.clone(), (PI / 3.0, TAU + PI / 6.0), true);
    assert_eq!(
        super::circular_rim_coverage::covers(&quarter, &long_rim, 1e-7),
        Some(false)
    );
    let major_cap = arc_use(circle.clone(), (0.0, 1.5 * PI), true);
    let crossing_gap = arc_use(circle.clone(), (PI, TAU + PI / 4.0), true);
    assert_eq!(
        super::circular_rim_coverage::covers(&major_cap, &crossing_gap, 1e-7),
        Some(false)
    );
    let within = arc_use(circle, (PI, 1.25 * PI), true);
    assert_eq!(
        super::circular_rim_coverage::covers(&major_cap, &within, 1e-7),
        Some(true)
    );
}

#[test]
fn literal_small_equator_partition_is_covered_by_an_opposite_frame_cap() {
    use std::f64::consts::FRAC_PI_2;
    let center = Point3::new(0.0, 0.0, 0.0);
    let rim = Circle3D::new_with_ref(
        center,
        Vec3::new(0.0, 0.0, 1.0),
        0.001,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let cap = Circle3D::new(center, Vec3::new(0.0, 0.0, -1.0), 0.001).unwrap();
    let a = cap.project(rim.evaluate(0.0));
    let b = cap.project(rim.evaluate(FRAC_PI_2));
    let quarter = arc_use(cap, (b, a), true);
    for index in 0..16 {
        let edge = arc_use(
            rim.clone(),
            (
                TAU * f64::from(index) / 16.0,
                TAU * f64::from(index + 1) / 16.0,
            ),
            true,
        );
        assert_eq!(
            super::circular_rim_coverage::covers(&quarter, &edge, 1e-7),
            Some(index < 4),
            "rim piece {index}"
        );
    }
}

#[test]
fn circle_rim_coverage_checks_authority_and_certifies_separated_carriers() {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let cap = arc_use(circle.clone(), (0.0, FRAC_PI_2), true);
    let mut edge = arc_use(circle, (0.0, FRAC_PI_4), true);
    edge.start_3d = edge.start_3d + Vec3::new(0.0, 0.0, 1e-5);
    assert!(super::circular_rim_coverage::covers(&cap, &edge, 1e-7).is_none());
    edge.trim = None;
    assert!(super::circular_rim_coverage::covers(&cap, &edge, 1e-7).is_none());
    let shifted =
        Circle3D::new(Point3::new(0.0, 0.0, 1e-6), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let shifted = arc_use(shifted, (0.0, FRAC_PI_4), true);
    assert_eq!(
        super::circular_rim_coverage::covers(&cap, &shifted, 1e-7),
        Some(false)
    );
    let nearby = Circle3D::new(Point3::new(0.0, 0.0, 5e-8), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let nearby = arc_use(nearby, (0.0, FRAC_PI_4), true);
    assert!(super::circular_rim_coverage::covers(&cap, &nearby, 1e-7).is_none());
}

#[test]
fn full_circle_coverage_requires_bounded_native_turn_and_closure() {
    use std::f64::consts::{FRAC_PI_4, PI};
    for radius in [0.001, 1.0, 1000.0] {
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            radius,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let full = arc_use(circle.clone(), (0.37, 0.37 + TAU), true);
        let segment = arc_use(circle.clone(), (PI, 1.5 * PI), true);
        assert_eq!(
            super::circular_rim_coverage::covers(&full, &segment, 1e-7),
            Some(true)
        );
        let gap = 1e-4;
        let almost_full = arc_use(circle.clone(), (0.0, TAU - gap), true);
        let across_gap = arc_use(circle, (TAU - 2.0 * gap, TAU + FRAC_PI_4), true);
        assert_eq!(
            super::circular_rim_coverage::covers(&almost_full, &across_gap, 1e-7),
            Some(false)
        );
    }
}
