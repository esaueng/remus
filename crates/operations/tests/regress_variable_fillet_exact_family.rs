//! The variable-radius fillet's exact family, and typed refusals outside it.
//!
//! The engine builds a section exactly only on a straight spine between two
//! planar supports: there each section is a rational circular arc in a fixed
//! frame and each row of the band is the radius law itself in cubic Hermite
//! form. This file pins three things.
//!
//! * Station laws (interior control radii under linear or S-curve pieces)
//!   build inside that family as multi-span exact bands. Every sampled band
//!   point lies on the circle of radius `r(x)` centred `r(x)` off both
//!   supports — a certificate that does not depend on volume quadrature —
//!   and the solid measures the closed form `V − (1 − π/4)∫r²`.
//! * An overflow at a station is caught there even when no fixed sampling
//!   station is near it.
//! * Everything else refuses by type, before anything is built: a curved
//!   spine or a curved support with `UnsupportedVariableSupport`, and two or
//!   more stripes meeting at a corner without qualified setbacks with
//!   `UnsupportedVertexBlend` (both used to fail late as
//!   `postcondition-failed` with a free-edge count).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_4;

use remus_blend::BlendError;
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::blend_failure_code;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::fillet::{
    FilletEdgeSetback, FilletRadiusLaw, FilletStationBlend, fillet_variable_with_setbacks,
};
use remus_operations::measure::mass_properties;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const LENGTH: f64 = 60.0;
const DEPTH: f64 = 40.0;
const HEIGHT: f64 = 20.0;

fn ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let edge = topo.edge(edge).unwrap();
    (
        topo.vertex(edge.start()).unwrap().point(),
        topo.vertex(edge.end()).unwrap().point(),
    )
}

/// The straight box edge whose two ends satisfy `on`.
fn edge_where(topo: &Topology, solid: SolidId, on: impl Fn(Point3) -> bool) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| {
            let (a, b) = ends(topo, id);
            matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Line) && on(a) && on(b)
        })
        .expect("edge")
}

fn spec(edge: EdgeId, law: FilletRadiusLaw) -> FilletEdgeSetback {
    FilletEdgeSetback {
        edge,
        law,
        start_setback: 0.0,
        end_setback: 0.0,
    }
}

fn stations(blend: FilletStationBlend) -> FilletRadiusLaw {
    FilletRadiusLaw::Stations {
        blend,
        stations: vec![(0.0, 1.0), (0.25, 3.0), (0.6, 1.5), (1.0, 2.0)],
    }
}

/// `∫₀¹ r(t)² dt`, exact: four-point Gauss–Legendre on every cubic piece
/// integrates its sextic square without error.
fn mean_square_radius(law: &FilletRadiusLaw) -> f64 {
    const NODES: [(f64, f64); 4] = [
        (-0.861_136_311_594_052_6, 0.347_854_845_137_453_9),
        (-0.339_981_043_584_856_3, 0.652_145_154_862_546_1),
        (0.339_981_043_584_856_3, 0.652_145_154_862_546_1),
        (0.861_136_311_594_052_6, 0.347_854_845_137_453_9),
    ];
    law.segments()
        .iter()
        .map(|piece| {
            let half = 0.5 * (piece.t1 - piece.t0);
            let mid = 0.5 * (piece.t1 + piece.t0);
            NODES
                .iter()
                .map(|(x, w)| {
                    let r = law.evaluate(half.mul_add(*x, mid));
                    w * r * r * half
                })
                .sum::<f64>()
        })
        .sum()
}

/// Build a station-law round on the bottom-front edge and certify it.
fn certify(law: &FilletRadiusLaw) {
    let label = format!("{law:?}");
    let mut topo = Topology::new();
    let input = make_box(&mut topo, LENGTH, DEPTH, HEIGHT).unwrap();
    let edge = edge_where(&topo, input, |p| p.y().abs() < 1e-9 && p.z().abs() < 1e-9);
    let (start, end) = ends(&topo, edge);
    let result = fillet_variable_with_setbacks(&mut topo, input, &[spec(edge, law.clone())])
        .unwrap_or_else(|e| panic!("{label}: {e}"));

    let issues = validate_solid(&topo, result).unwrap();
    assert!(issues.is_valid(), "{label}: {issues:?}");
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, &topo).unwrap_or_else(|e| panic!("{label}: {e}"));

    let bands: Vec<_> = solid_faces(&topo, result)
        .unwrap()
        .into_iter()
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Nurbs(surface) => Some(surface.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(bands.len(), 1, "{label}: one band face");
    let band = &bands[0];

    // One Bézier piece per law segment, joined at triple interior knots.
    let mut interior: Vec<f64> = band
        .knots_v()
        .iter()
        .copied()
        .filter(|k| *k > 0.0 && *k < 1.0)
        .collect();
    interior.dedup();
    let breakpoints = law.breakpoints();
    assert_eq!(
        interior.len(),
        breakpoints.len(),
        "{label}: {:?}",
        band.knots_v()
    );
    for (knot, station) in interior.iter().zip(&breakpoints) {
        assert!(
            (knot - station).abs() < 1e-15,
            "{label}: knot {knot} at {station}"
        );
    }

    // Exact-section certificate: every band point lies on the circle of
    // radius r(x) about (x, r(x), r(x)), between the two contacts.
    let direction = (end - start) * (1.0 / LENGTH);
    let mut worst = 0.0_f64;
    for i in 0..=40 {
        for j in 0..=200 {
            let p = band.evaluate(f64::from(i) / 40.0, f64::from(j) / 200.0);
            let t = (p - start).dot(direction) / LENGTH;
            assert!((-1e-12..=1.0 + 1e-12).contains(&t), "{label}: t {t}");
            let r = law.evaluate(t);
            let off = (p.y() - r).hypot(p.z() - r) - r;
            worst = worst.max(off.abs());
            assert!(
                p.y() <= r + 1e-9 && p.z() <= r + 1e-9,
                "{label}: {p:?} past a contact"
            );
        }
    }
    assert!(worst < 1e-9, "{label}: worst off-circle distance {worst:e}");

    let expected = LENGTH.mul_add(
        -(1.0 - FRAC_PI_4) * mean_square_radius(law),
        LENGTH * DEPTH * HEIGHT,
    );
    let measured = mass_properties(&topo, result).unwrap().mass;
    assert!(
        (measured - expected).abs() < 1e-6 * expected,
        "{label}: {measured:.6} against the closed form {expected:.6}"
    );
}

#[test]
fn linear_station_law_builds_an_exact_multi_span_band() {
    certify(&stations(FilletStationBlend::Linear));
}

#[test]
fn s_curve_station_law_builds_an_exact_multi_span_band() {
    certify(&stations(FilletStationBlend::SCurve));
}

#[test]
fn the_three_standard_laws_still_build_one_exact_piece() {
    for law in [
        FilletRadiusLaw::Constant(2.0),
        FilletRadiusLaw::Linear {
            start: 1.0,
            end: 2.0,
        },
        FilletRadiusLaw::SCurve {
            start: 2.0,
            end: 1.0,
        },
    ] {
        certify(&law);
    }
}

/// The refusal's typed error and stable code, with the input left intact.
fn refuse(
    topo: &mut Topology,
    input: SolidId,
    specs: &[FilletEdgeSetback],
) -> (OperationsError, &'static str) {
    let faces = solid_faces(topo, input).unwrap().len();
    let edges = solid_edges(topo, input).unwrap().len();
    let error = fillet_variable_with_setbacks(topo, input, specs).unwrap_err();
    let code = blend_failure_code(&error);
    assert_eq!(solid_faces(topo, input).unwrap().len(), faces);
    assert_eq!(solid_edges(topo, input).unwrap().len(), edges);
    assert!(validate_solid(topo, input).unwrap().is_valid());
    (error, code)
}

#[test]
fn an_overflow_at_a_station_is_caught_between_sampling_stations() {
    // A spike to 21 on the 20-wide front face, narrow enough that every fixed
    // sampling station sees radius 1.
    let mut topo = Topology::new();
    let input = make_box(&mut topo, LENGTH, DEPTH, HEIGHT).unwrap();
    let edge = edge_where(&topo, input, |p| p.y().abs() < 1e-9 && p.z().abs() < 1e-9);
    let law = FilletRadiusLaw::Stations {
        blend: FilletStationBlend::Linear,
        stations: vec![
            (0.0, 1.0),
            (0.5005, 1.0),
            (0.501, 21.0),
            (0.5015, 1.0),
            (1.0, 1.0),
        ],
    };
    let (error, code) = refuse(&mut topo, input, &[spec(edge, law)]);
    assert_eq!(code, "cliff-encountered");
    let OperationsError::Blend(BlendError::CliffEncountered {
        requested_radius,
        available_radius,
        ..
    }) = error
    else {
        panic!("{error:?}");
    };
    assert!(
        (requested_radius - 21.0).abs() < 1e-12,
        "{requested_radius}"
    );
    assert!(
        (available_radius - HEIGHT).abs() < 1e-9,
        "{available_radius}"
    );
}

#[test]
fn malformed_stations_refuse_as_invalid_blend_input() {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, LENGTH, DEPTH, HEIGHT).unwrap();
    let edge = edge_where(&topo, input, |p| p.y().abs() < 1e-9 && p.z().abs() < 1e-9);
    for stations in [
        vec![(0.0, 1.0)],
        vec![(0.0, 1.0), (0.6, 2.0), (0.4, 2.0), (1.0, 1.0)],
        vec![(0.0, 1.0), (0.5, 2.0), (0.5, 2.0), (1.0, 1.0)],
        vec![(0.1, 1.0), (1.0, 1.0)],
        vec![(0.0, 1.0), (0.5, -1.0), (1.0, 1.0)],
        vec![(0.0, 1.0), (0.5, f64::NAN), (1.0, 1.0)],
    ] {
        let law = FilletRadiusLaw::Stations {
            blend: FilletStationBlend::SCurve,
            stations: stations.clone(),
        };
        let (error, _) = refuse(&mut topo, input, &[spec(edge, law)]);
        assert!(
            matches!(
                error,
                OperationsError::Blend(BlendError::InvalidInput { .. })
            ),
            "{stations:?}: {error:?}"
        );
    }
}

fn assert_inexact(error: &OperationsError, code: &str, needle: &str) {
    assert_eq!(code, "unsupported-variable-support", "{error}");
    let OperationsError::Blend(BlendError::UnsupportedVariableSupport { reason, .. }) = error
    else {
        panic!("{error:?}");
    };
    assert!(reason.contains(needle), "{reason}");
}

#[test]
fn a_curved_spine_refuses_by_type_before_building() {
    let mut topo = Topology::new();
    let input = make_cylinder(&mut topo, 10.0, 30.0).unwrap();
    let rim = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .find(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_)))
        .unwrap();
    for law in [
        FilletRadiusLaw::Constant(1.0),
        FilletRadiusLaw::Linear {
            start: 1.0,
            end: 2.0,
        },
    ] {
        let (error, code) = refuse(&mut topo, input, &[spec(rim, law)]);
        assert_inexact(&error, code, "circle");
    }
}

#[test]
fn a_straight_edge_on_a_curved_support_refuses_by_type_before_building() {
    // Half a cylinder: its two straight edges run between the flat cut face
    // and the cylinder wall.
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 10.0, 30.0).unwrap();
    let cutter = make_box(&mut topo, 30.0, 30.0, 40.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(-15.0, 0.0, -5.0)).unwrap();
    let input = boolean(&mut topo, BooleanOp::Cut, cylinder, cutter).unwrap();
    let line = edge_where(&topo, input, |p| {
        (p.x() - 10.0).abs() < 1e-9 && p.y().abs() < 1e-9
    });
    for law in [
        FilletRadiusLaw::Constant(1.5),
        FilletRadiusLaw::Linear {
            start: 1.0,
            end: 2.0,
        },
    ] {
        let (error, code) = refuse(&mut topo, input, &[spec(line, law)]);
        assert_inexact(&error, code, "a cylinder");
    }
}

#[test]
fn stripes_meeting_at_an_unset_corner_refuse_by_type_before_building() {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, LENGTH, DEPTH, HEIGHT).unwrap();
    let at_origin_along = |axis: usize| {
        edge_where(&topo, input, |p| {
            (0..3).all(|other| other == axis || [p.x(), p.y(), p.z()][other].abs() < 1e-9)
        })
    };
    let (x, y, z) = (at_origin_along(0), at_origin_along(1), at_origin_along(2));
    let linear = FilletRadiusLaw::Linear {
        start: 2.0,
        end: 2.0,
    };
    for (edges, stripes) in [(vec![x, y], 2), (vec![x, y, z], 3)] {
        let specs: Vec<_> = edges.iter().map(|&e| spec(e, linear.clone())).collect();
        let (error, code) = refuse(&mut topo, input, &specs);
        assert_eq!(code, "unsupported-vertex-blend", "{error}");
        let OperationsError::Blend(BlendError::UnsupportedVertexBlend { stripes: met, .. }) = error
        else {
            panic!("{error:?}");
        };
        assert_eq!(met, stripes);
    }

    // Disjoint parallel edges share no vertex and still build.
    let far = edge_where(&topo, input, |p| {
        (p.y() - DEPTH).abs() < 1e-9 && (p.z() - HEIGHT).abs() < 1e-9
    });
    let built = fillet_variable_with_setbacks(
        &mut topo,
        input,
        &[
            spec(x, linear),
            spec(
                far,
                FilletRadiusLaw::Linear {
                    start: 1.0,
                    end: 3.0,
                },
            ),
        ],
    )
    .unwrap();
    assert!(validate_solid(&topo, built).unwrap().is_valid());
}

/// A round whose stripe ends on a cap carrying a circular rim: the vertical
/// corner edge of a 60 × 60 × 20 box with a boss fused on its top, or a
/// blind bore cut into it.
///
/// The cap is trimmed but keeps its hole, so it is copied with a rebuilt
/// outer wire. That wire used to mint a straight chord across the stripe's
/// end section before the band published its arc, and the band then shared
/// the chord: the solid validated, but its top boundary left the band
/// surface and the body measured 19.60 mm³ removed against the closed
/// form's 10.01. The rebuilt wire now takes the band's exact arc.
#[test]
fn a_stripe_ending_on_a_holed_cap_keeps_its_end_arc() {
    for op in [BooleanOp::Fuse, BooleanOp::Cut] {
        for law in [
            FilletRadiusLaw::Linear {
                start: 1.0,
                end: 2.0,
            },
            FilletRadiusLaw::SCurve {
                start: 2.0,
                end: 1.0,
            },
            stations(FilletStationBlend::Linear),
        ] {
            let label = format!("{op:?} {law:?}");
            let mut topo = Topology::new();
            let slab = make_box(&mut topo, 60.0, 60.0, HEIGHT).unwrap();
            let cylinder = make_cylinder(&mut topo, 8.0, 20.0).unwrap();
            transform_solid(&mut topo, cylinder, &Mat4::translation(30.0, 30.0, 10.0)).unwrap();
            let input = boolean(&mut topo, op, slab, cylinder).unwrap();
            let before = mass_properties(&topo, input).unwrap().mass;
            let edge = edge_where(&topo, input, |p| p.x().abs() < 1e-9 && p.y().abs() < 1e-9);
            let result =
                fillet_variable_with_setbacks(&mut topo, input, &[spec(edge, law.clone())])
                    .unwrap_or_else(|e| panic!("{label}: {e}"));
            assert!(validate_solid(&topo, result).unwrap().is_valid(), "{label}");

            let band = solid_faces(&topo, result)
                .unwrap()
                .into_iter()
                .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
                .unwrap();
            let face = topo.face(band).unwrap();
            for oriented in topo.wire(face.outer_wire()).unwrap().edges() {
                assert!(
                    !matches!(topo.edge(oriented.edge()).unwrap().curve(), EdgeCurve::Line),
                    "{label}: a band boundary is a chord"
                );
            }

            let removed = (1.0 - FRAC_PI_4) * HEIGHT * mean_square_radius(&law);
            let measured = mass_properties(&topo, result).unwrap().mass;
            assert!(
                (before - measured - removed).abs() < 1e-6 * before,
                "{label}: removed {} against the closed form {removed}",
                before - measured
            );
        }
    }
}

/// A 60-long prism whose trapezoidal section `(0,0) (40,0) (40−t,20) (t,20)`
/// in yz puts 60° between its floor and slanted walls and 120° between its
/// top and the same walls.
fn trapezoid_prism(topo: &mut Topology) -> SolidId {
    let t = HEIGHT / 3.0_f64.sqrt();
    let wire = remus_topology::builder::make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 40.0, 0.0),
            Point3::new(0.0, 40.0 - t, HEIGHT),
            Point3::new(0.0, t, HEIGHT),
        ],
        1e-7,
    )
    .unwrap();
    let face = remus_topology::builder::make_face_from_wire(topo, wire).unwrap();
    remus_operations::extrude::extrude(
        topo,
        face,
        remus_math::vec::Vec3::new(1.0, 0.0, 0.0),
        LENGTH,
    )
    .unwrap()
}

/// A round on a convex straight edge between planes at an interior angle
/// other than 90° is the exact rolling ball: every band point lies `r(x)`
/// from the centre `r(x)/sin(α/2)` along the bisector, and the body loses
/// `(cot(α/2) − (π − α)/2)·∫r²` along the edge. On the 2026.1.157 pin the
/// band put its contacts `r` along each face with apex weight `cos(α/2)` —
/// right only at 90° — and a constant r = 1 round on the 120° edge removed
/// 13.697 mm³ against the closed form's 3.225, reported as a success.
#[test]
fn an_oblique_dihedral_builds_the_exact_rolling_ball_section() {
    use std::f64::consts::PI;
    let t = HEIGHT / 3.0_f64.sqrt();
    let wall = remus_math::vec::Vec3::new(0.0, -HEIGHT, t)
        .normalize()
        .unwrap();
    // (edge on, its two outward support normals, interior angle)
    let cases = [
        (
            Point3::new(0.0, 0.0, 0.0),
            remus_math::vec::Vec3::new(0.0, 0.0, -1.0),
            wall,
            PI / 3.0,
        ),
        (
            Point3::new(0.0, t, HEIGHT),
            remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
            wall,
            2.0 * PI / 3.0,
        ),
    ];
    for (at, n1, n2, interior) in cases {
        for law in [
            FilletRadiusLaw::Constant(1.0),
            FilletRadiusLaw::Linear {
                start: 1.0,
                end: 2.0,
            },
            stations(FilletStationBlend::SCurve),
        ] {
            let label = format!("{:.0}° {law:?}", interior.to_degrees());
            let mut topo = Topology::new();
            let input = trapezoid_prism(&mut topo);
            let before = mass_properties(&topo, input).unwrap().mass;
            let edge = edge_where(&topo, input, |p| {
                (p.y() - at.y()).abs() < 1e-9 && (p.z() - at.z()).abs() < 1e-9
            });
            let (start, end) = ends(&topo, edge);
            let result =
                fillet_variable_with_setbacks(&mut topo, input, &[spec(edge, law.clone())])
                    .unwrap_or_else(|e| panic!("{label}: {e}"));
            assert!(validate_solid(&topo, result).unwrap().is_valid(), "{label}");

            let band = solid_faces(&topo, result)
                .unwrap()
                .into_iter()
                .find_map(|face| match topo.face(face).unwrap().surface() {
                    FaceSurface::Nurbs(surface) => Some(surface.clone()),
                    _ => None,
                })
                .unwrap();
            let direction = (end - start) * (1.0 / LENGTH);
            let mut worst = 0.0_f64;
            for i in 0..=40 {
                for j in 0..=120 {
                    let p = band.evaluate(f64::from(i) / 40.0, f64::from(j) / 120.0);
                    let s = (p - start).dot(direction) / LENGTH;
                    let r = law.evaluate(s);
                    let on_edge = start + direction * (LENGTH * s);
                    let center = on_edge - (n1 + n2) * (r / (1.0 + n1.dot(n2)));
                    worst = worst.max(((p - center).length() - r).abs());
                }
            }
            assert!(worst < 1e-9, "{label}: worst off-ball distance {worst:e}");

            let coefficient = 1.0 / (interior / 2.0).tan() - (PI - interior) / 2.0;
            let removed = coefficient * LENGTH * mean_square_radius(&law);
            let measured = before - mass_properties(&topo, result).unwrap().mass;
            assert!(
                (measured - removed).abs() < 1e-6 * before,
                "{label}: removed {measured} against the closed form {removed}"
            );
        }
    }
}

#[test]
fn a_concave_edge_refuses_by_type_before_building() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, LENGTH, DEPTH, 10.0).unwrap();
    let upright = make_box(&mut topo, LENGTH, 10.0, 30.0).unwrap();
    let input = boolean(&mut topo, BooleanOp::Fuse, base, upright).unwrap();
    let edge = edge_where(&topo, input, |p| {
        (p.y() - 10.0).abs() < 1e-9 && (p.z() - 10.0).abs() < 1e-9
    });
    let (error, code) = refuse(
        &mut topo,
        input,
        &[spec(edge, FilletRadiusLaw::Constant(2.0))],
    );
    assert_inexact(&error, code, "concave");
}

/// An S-curve can overrun a support whose width shrinks along the edge while
/// every sampled station fits: the floor of `(0,0) (60,0) (60,1.05)
/// (0,101.05)` narrows as `101.05 − 100τ`, and stations `(0, 2) (0.99, 2)
/// (1, 1)` fit at every fixed station and breakpoint, yet at τ = 0.9925 the
/// radius is 1.84375 against a width of 1.8. The overflow check is exact per
/// boundary segment now, and refuses.
#[test]
fn an_s_curve_overrun_between_stations_is_refused() {
    let mut topo = Topology::new();
    let wire = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(LENGTH, 0.0, 0.0),
            Point3::new(LENGTH, 1.05, 0.0),
            Point3::new(0.0, 101.05, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = remus_topology::builder::make_face_from_wire(&mut topo, wire).unwrap();
    let input = remus_operations::extrude::extrude(
        &mut topo,
        face,
        remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
        HEIGHT,
    )
    .unwrap();
    let edge = edge_where(&topo, input, |p| p.y().abs() < 1e-9 && p.z().abs() < 1e-9);
    let (start, _) = ends(&topo, edge);
    // Stations in the edge's own direction.
    let forward = [(0.0, 2.0), (0.99, 2.0), (1.0, 1.0)];
    let stations: Vec<(f64, f64)> = if start.x().abs() < 1e-9 {
        forward.to_vec()
    } else {
        forward.iter().rev().map(|(t, r)| (1.0 - t, *r)).collect()
    };
    let law = FilletRadiusLaw::Stations {
        blend: FilletStationBlend::SCurve,
        stations,
    };
    let (error, code) = refuse(&mut topo, input, &[spec(edge, law)]);
    assert_eq!(code, "cliff-encountered", "{error}");
    let OperationsError::Blend(BlendError::CliffEncountered {
        requested_radius,
        available_radius,
        ..
    }) = error
    else {
        panic!("{error:?}");
    };
    assert!(
        requested_radius > available_radius && requested_radius < 2.0 && available_radius > 1.05,
        "{requested_radius} against {available_radius}"
    );
}

/// A narrow S-curve spike beside a bore's nearest point must meet the bore.
/// A 60 × 60 × 20 block with a radius-10 through bore at (20, 20): the bore's
/// leftmost point stands 20 off the front-bottom edge at x = 10. A spike to
/// 21 between x = 10.0002 and 10.0006 runs into the bore wall, which is only
/// ~19.91 off the edge there. The curved boundary is checked through a
/// closed polygon circumscribed about the whole circle, so no ray passes
/// between its pieces (the earlier per-chord lens left gaps there).
#[test]
fn an_overrun_into_a_bore_between_its_chords_is_refused() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 60.0, 60.0, HEIGHT).unwrap();
    let bore = make_cylinder(&mut topo, 10.0, 40.0).unwrap();
    transform_solid(&mut topo, bore, &Mat4::translation(20.0, 20.0, -10.0)).unwrap();
    let input = boolean(&mut topo, BooleanOp::Cut, block, bore).unwrap();
    let edge = edge_where(&topo, input, |p| p.y().abs() < 1e-9 && p.z().abs() < 1e-9);
    let (start, _) = ends(&topo, edge);
    let at = |x: f64| x / 60.0;
    let forward = [
        (0.0, 1.0),
        (at(10.0002), 1.0),
        (at(10.0004), 21.0),
        (at(10.0006), 1.0),
        (1.0, 1.0),
    ];
    let stations: Vec<(f64, f64)> = if start.x().abs() < 1e-9 {
        forward.to_vec()
    } else {
        forward.iter().rev().map(|(t, r)| (1.0 - t, *r)).collect()
    };
    let law = FilletRadiusLaw::Stations {
        blend: FilletStationBlend::SCurve,
        stations,
    };
    let (error, code) = refuse(&mut topo, input, &[spec(edge, law)]);
    assert_eq!(code, "cliff-encountered", "{error}");
    let OperationsError::Blend(BlendError::CliffEncountered {
        requested_radius,
        available_radius,
        ..
    }) = error
    else {
        panic!("{error:?}");
    };
    assert!(requested_radius > 19.9, "{requested_radius}");
    assert!(available_radius < requested_radius, "{available_radius}");
}

/// A reentrant edge between two lips thinner than the concavity probe: the
/// region [−0.05, 60]² less its negative-negative quadrant, extruded 60. A
/// probe that steps over the 0.05 lips can call the edge convex. The side is
/// read from the faces' loops, and the probe only cross-checks it, so the
/// edge is refused, never built with its contacts behind its supports.
#[test]
fn a_concave_edge_between_thin_lips_is_not_taken_for_convex() {
    let mut topo = Topology::new();
    let wire = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, -0.05, 0.0),
            Point3::new(60.0, -0.05, 0.0),
            Point3::new(60.0, 60.0, 0.0),
            Point3::new(-0.05, 60.0, 0.0),
            Point3::new(-0.05, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = remus_topology::builder::make_face_from_wire(&mut topo, wire).unwrap();
    let input = remus_operations::extrude::extrude(
        &mut topo,
        face,
        remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
        60.0,
    )
    .unwrap();
    let edge = edge_where(&topo, input, |p| p.x().abs() < 1e-9 && p.y().abs() < 1e-9);
    let (error, code) = refuse(
        &mut topo,
        input,
        &[spec(edge, FilletRadiusLaw::Constant(1.0))],
    );
    assert_eq!(code, "unsupported-variable-support", "{error}");
/// The three edges meeting at the box's origin corner, each with its law
/// given as (radius at the corner, radius at the far end).
fn corner_specs(
    topo: &Topology,
    input: SolidId,
    laws: [(FilletStationBlend, f64, f64); 3],
) -> Vec<FilletEdgeSetback> {
    (0..3)
        .map(|axis| {
            let edge = edge_where(topo, input, |p| {
                (0..3).all(|other| other == axis || [p.x(), p.y(), p.z()][other].abs() < 1e-9)
            });
            let (start, _) = ends(topo, edge);
            let (blend, at_corner, far) = laws[axis];
            let (a, b) = if start.x().abs() + start.y().abs() + start.z().abs() < 1e-9 {
                (at_corner, far)
            } else {
                (far, at_corner)
            };
            let law = match blend {
                FilletStationBlend::SCurve => FilletRadiusLaw::SCurve { start: a, end: b },
                FilletStationBlend::Linear => FilletRadiusLaw::Linear { start: a, end: b },
            };
            spec(edge, law)
        })
        .collect()
}

/// M03 setback corners: where three rounded box edges meet, the engine sets
/// each stripe back to the one ball tangent to the three faces and closes
/// the corner with that ball's spherical cap, exactly. The stripes' radii
/// must agree at the corner with zero slope there (an S-curve or constant
/// law), and the body loses the stripes' sections over their cropped spans
/// plus the corner cube less the ball's octant: `r³(1 − π/6)`.
#[test]
fn tangent_ball_corners_close_three_variable_stripes_exactly() {
    use remus_operations::fillet::{VariableCorners, fillet_variable_with_corners};
    use std::f64::consts::PI;
    let s = FilletStationBlend::SCurve;
    for laws in [
        [(s, 2.0, 2.0), (s, 2.0, 2.0), (s, 2.0, 2.0)],
        [(s, 2.0, 1.0), (s, 2.0, 3.0), (s, 2.0, 1.5)],
    ] {
        let label = format!("{laws:?}");
        let mut topo = Topology::new();
        let input = make_box(&mut topo, LENGTH, DEPTH, HEIGHT).unwrap();
        let specs = corner_specs(&topo, input, laws);
        let result =
            fillet_variable_with_corners(&mut topo, input, &specs, VariableCorners::TangentBall)
                .unwrap_or_else(|e| panic!("{label}: {e}"));
        assert!(validate_solid(&topo, result).unwrap().is_valid(), "{label}");
        let faces = solid_faces(&topo, result).unwrap();
        assert_eq!(faces.len(), 6 + 3 + 1, "{label}");
        let caps: Vec<_> = faces
            .iter()
            .filter_map(|&f| match topo.face(f).unwrap().surface() {
                FaceSurface::Sphere(sphere) => Some((sphere.center(), sphere.radius())),
                _ => None,
            })
            .collect();
        assert_eq!(caps.len(), 1, "{label}");
        let (center, radius) = caps[0];
        assert!((radius - 2.0).abs() < 1e-12, "{label}: {radius}");
        assert!(
            (center - Point3::new(2.0, 2.0, 2.0)).length() < 1e-12,
            "{label}"
        );

        // Each stripe runs over its edge less the 2 mm setback, with its law
        // stretched over that span.
        let lengths = [LENGTH, DEPTH, HEIGHT];
        let stripes: f64 = laws
            .iter()
            .zip(lengths)
            .map(|(&(_, corner, far), length)| {
                let law = FilletRadiusLaw::SCurve {
                    start: corner,
                    end: far,
                };
                (1.0 - FRAC_PI_4) * (length - 2.0) * mean_square_radius(&law)
            })
            .sum();
        let removed = stripes + 8.0 * (1.0 - PI / 6.0);
        let measured = mass_properties(&topo, result).unwrap().mass;
        let expected = LENGTH * DEPTH * HEIGHT - removed;
        assert!(
            (measured - expected).abs() < 1e-6 * expected,
            "{label}: {measured} against the closed form {expected}"
        );
    }
}

#[test]
fn tangent_ball_corners_refuse_what_no_single_ball_closes() {
    use remus_operations::fillet::{VariableCorners, fillet_variable_with_corners};
    let s = FilletStationBlend::SCurve;
    let l = FilletStationBlend::Linear;
    let mut topo = Topology::new();
    let input = make_box(&mut topo, LENGTH, DEPTH, HEIGHT).unwrap();
    let run = |topo: &mut Topology, specs: &[FilletEdgeSetback], corners| {
        let faces = solid_faces(topo, input).unwrap().len();
        let error = fillet_variable_with_corners(topo, input, specs, corners).unwrap_err();
        assert_eq!(solid_faces(topo, input).unwrap().len(), faces);
        error
    };

    // Unequal radii at the corner, or a slope there: typed corner refusals.
    for laws in [
        [(s, 2.0, 2.0), (s, 2.5, 2.0), (s, 2.0, 2.0)],
        [(l, 2.0, 1.0), (s, 2.0, 2.0), (s, 2.0, 2.0)],
    ] {
        let specs = corner_specs(&topo, input, laws);
        let error = run(&mut topo, &specs, VariableCorners::TangentBall);
        assert_eq!(
            blend_failure_code(&error),
            "unsupported-setback-corner",
            "{error}"
        );
    }
    // Without tangent-ball corners the undeclared corner stays refused.
    let specs = corner_specs(&topo, input, [(s, 2.0, 2.0); 3]);
    let error = run(&mut topo, &specs, VariableCorners::Declared);
    assert_eq!(
        blend_failure_code(&error),
        "unsupported-vertex-blend",
        "{error}"
    );
    // A declared setback beside tangent-ball corners is a contradiction.
    let mut declared = specs;
    declared[0].start_setback = 2.0;
    let error = run(&mut topo, &declared, VariableCorners::TangentBall);
    assert!(
        matches!(error, OperationsError::InvalidInput { .. }),
        "{error:?}"
    );
}
