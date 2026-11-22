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
            spec(x, linear.clone()),
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
