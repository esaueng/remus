#![allow(clippy::unwrap_used, clippy::panic, clippy::float_cmp)]

use super::*;
use crate::{RuledSectionError, discover_ruled_face_section};
use remus_math::context::{CancellationToken, WorkBudgets};
use remus_math::vec::Vec3;

use super::super::tests::{face, face_in_chart, fixture, place, section_and_surfaces};

const FULL: [f64; 4] = [0.0, 1.0, -1.5, 1.5];

fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 2e-10 * (1.0 + b.abs()), "{a} != {b}");
}

fn locus(f: f64, rational: bool) -> Point3 {
    if rational {
        let middle = std::f64::consts::SQRT_2 * f * (1.0 - f);
        let denominator = (1.0 - f).powi(2) + middle + f.powi(2);
        Point3::new(
            ((1.0 - f).powi(2) + middle) / denominator,
            0.0,
            (middle + f.powi(2)) / denominator,
        )
    } else {
        Point3::new(f, 0.0, f * f)
    }
}

#[test]
fn public_discovery_proves_curved_sections_and_preserves_native_trim_events() {
    let chart_pairs = [
        [(0.0, 1.0); 2],
        [(-8.0, 24.0), (16.0, 32.0)],
        [(0.0, 2_f64.powi(-20)), (1024.0, 2048.0)],
    ];
    for rational in [false, true] {
        for scale in [1e-3, 1.0, 1e3] {
            for moved in [false, true] {
                for reversed in [false, true] {
                    for domains in chart_pairs {
                        for swapped in [false, true] {
                            let (topo, traces, _) = fixture(
                                &[[0.125, 0.875, -1.0, 1.0], [0.375, 0.625, -0.5, 0.5]],
                                &[FULL],
                                rational,
                                scale,
                                moved,
                                reversed,
                                domains,
                            );
                            let (traces, domains) = if swapped {
                                ([traces[1], traces[0]], [domains[1], domains[0]])
                            } else {
                                (traces, domains)
                            };
                            let before = format!("{topo:?}");
                            let result = discover_ruled_face_section(
                                &topo,
                                traces.map(|trace| trace.face),
                                &OperationContext::new(),
                            )
                            .unwrap_or_else(|error| {
                                panic!("rational={rational} scale={scale} moved={moved} reversed={reversed} swapped={swapped} domains={domains:?}: {error}")
                            });
                            assert_eq!(result.curve.domain(), domains[0]);
                            assert_eq!(result.clipping.intervals.len(), 2);
                            for (interval, fractions) in result
                                .clipping
                                .intervals
                                .iter()
                                .zip([[0.125, 0.375], [0.625, 0.875]])
                            {
                                for (j, fraction) in fractions.into_iter().enumerate() {
                                    near(
                                        interval.source_range[j],
                                        domains[0].0 + (domains[0].1 - domains[0].0) * fraction,
                                    );
                                    assert!(!interval.endpoints[j].is_empty());
                                }
                            }
                            assert_eq!(result.clipping.events.len(), 6);
                            for event in &result.clipping.events {
                                let coedge = topo.coedge(event.coedge).unwrap();
                                assert_eq!(coedge.edge(), event.edge);
                                assert_eq!(coedge.is_forward(), event.forward);
                                assert!(
                                    topo.face_loop(event.boundary_loop)
                                        .unwrap()
                                        .coedges()
                                        .contains(&event.coedge)
                                );
                                assert!(
                                    event.section_parameter_bound[0] <= event.section_parameter
                                );
                                assert!(
                                    event.section_parameter_bound[1] >= event.section_parameter
                                );
                                assert!(event.boundary_residual < 1e-7);
                                assert!(event.surface_residuals.into_iter().all(|r| r < 1e-7));
                            }
                            for bound in result.clipping.surface_residual_bounds {
                                assert!(bound.is_finite() && bound < 1e-7);
                            }
                            assert_native_trace_bounds(&topo, &result, 31);
                            for trace in result.traces {
                                let j = usize::from(trace.face != traces[0].face);
                                assert_eq!(trace.source_range, [domains[0].0, domains[0].1]);
                                assert_eq!(trace.u_range, [domains[j].0, domains[j].1]);
                                near(trace.v, 0.0);
                            }
                            // Closed forms are independent of the stored surface
                            // evaluator and of the discovered control points.
                            for k in 0..=32 {
                                let fraction = f64::from(k) / 32.0;
                                let parameter =
                                    domains[0].0 + (domains[0].1 - domains[0].0) * fraction;
                                let expected = place(locus(fraction, rational), scale, moved);
                                assert!(
                                    (result.curve.evaluate(parameter) - expected).length() < 1e-7
                                );
                            }
                            assert_eq!(format!("{topo:?}"), before);
                        }
                    }
                }
            }
        }
    }
}

fn assert_native_trace_bounds(topo: &Topology, result: &RuledFaceSection, samples: u32) {
    for index in 0..2 {
        let trace = result.traces[index];
        let FaceSurface::Nurbs(surface) = topo.face(trace.face).unwrap().surface() else {
            panic!("discovered traces retain original NURBS carriers");
        };
        for k in 0..=samples {
            let fraction = f64::from(k) / f64::from(samples);
            let parameter =
                trace.source_range[0] + (trace.source_range[1] - trace.source_range[0]) * fraction;
            // Evaluate the actual returned maps as an ordinary caller does,
            // including native subtraction/division and target addition.
            let fraction = (parameter - trace.source_range[0])
                / (trace.source_range[1] - trace.source_range[0]);
            let u = trace.u_range[0] + (trace.u_range[1] - trace.u_range[0]) * fraction;
            let observed =
                (result.curve.evaluate(parameter) - surface.evaluate(u, trace.v)).length();
            assert!(
                observed <= result.clipping.surface_residual_bounds[index],
                "trace={trace:?} parameter={parameter} observed={observed} bound={}",
                result.clipping.surface_residual_bounds[index]
            );
        }
    }
}

#[test]
fn poorly_conditioned_native_u_maps_refuse_in_both_operand_orders() {
    for rational in [false, true] {
        for domains in [
            [(0.0, 1.0), (1e16, 1e16 + 16.0)],
            [(1e16, 1e16 + 16.0), (0.0, 1.0)],
        ] {
            let (topo, traces, _) = fixture(&[FULL], &[FULL], rational, 1.0, false, false, domains);
            let before = format!("{topo:?}");
            assert!(matches!(
                discover_ruled_face_section(
                    &topo,
                    traces.map(|trace| trace.face),
                    &OperationContext::new(),
                ),
                Err(RuledSectionError::Residual { bound }) if bound > 1e-7
            ));
            assert_eq!(format!("{topo:?}"), before);
        }
    }
}

#[test]
fn a_nonfinite_native_roundoff_allowance_cannot_be_published() {
    let (topo, traces, curve) = fixture(
        &[FULL],
        &[FULL],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0), (0.0, f64::MAX)],
    );
    let before = format!("{topo:?}");
    // Exercise the roundoff qualifier directly: an earlier physical-boundary
    // interval check is also allowed to refuse this extreme chart. The
    // qualifier must independently refuse a nonfinite allowance even when
    // the formal dual-surface residual is zero.
    let mut clipping = ClippedSection {
        intervals: Vec::new(),
        events: Vec::new(),
        surface_residual_bounds: [0.0; 2],
    };
    assert!(matches!(
        qualify_native_traces(&topo, traces, &curve, [0.0; 2], &mut clipping, &OperationContext::new()),
        Err(RuledSectionError::Residual { bound }) if !bound.is_finite()
    ));
    assert!(
        discover_ruled_face_section(
            &topo,
            traces.map(|trace| trace.face),
            &OperationContext::new(),
        )
        .is_err()
    );
    assert_eq!(format!("{topo:?}"), before);
}

#[test]
fn distinct_discarded_crossings_cannot_share_a_rounded_source_parameter() {
    let (topo, traces, _) = fixture(
        &[FULL, [0.25, 0.75, -0.5, 0.5]],
        &[[0.375, 0.382_812_5, -1.0, 1.0]],
        false,
        1e-6,
        false,
        false,
        [(1e16, 1e16 + 64.0), (0.0, 1.0)],
    );
    let before = format!("{topo:?}");
    // The two second-face crossings are geometrically distinct but both
    // reconstruct to source 1e16+24. Their material is covered by the first
    // face's hole, so checking only retained-interval endpoints misses the
    // collapsed pair. Model-space tolerance must not merge their identity.
    assert_eq!(1e16 + 64.0 * 0.375, 1e16 + 64.0 * 0.382_812_5);
    let result = discover_ruled_face_section(
        &topo,
        traces.map(|trace| trace.face),
        &OperationContext::new(),
    );
    assert!(
        matches!(result, Err(RuledSectionError::AmbiguousBoundary)),
        "{result:?}"
    );
    assert_eq!(format!("{topo:?}"), before);
}

#[test]
fn non_dyadic_source_endpoints_keep_the_original_native_domain() {
    for rational in [false, true] {
        // Recomposition overshoots 0.1 and undershoots 0.2 respectively;
        // neither result is the authoritative stored source endpoint.
        for domain in [(-1.0, 0.1), (-0.5, 0.2)] {
            assert_ne!(domain.0 + (domain.1 - domain.0), domain.1);
            let (_, surfaces) = section_and_surfaces(rational, 1.0, false, [domain, (0.0, 1.0)]);
            let mut topo = Topology::new();
            let first = face_in_chart(
                &mut topo,
                surfaces[0].clone(),
                &[[0.0, domain.1, -1.0, 1.0]],
                false,
                true,
            );
            let second = face(&mut topo, surfaces[1].clone(), &[FULL], false);
            let before = format!("{topo:?}");
            let result =
                discover_ruled_face_section(&topo, [first, second], &OperationContext::new())
                    .unwrap();
            assert_eq!(result.curve.domain(), domain);
            assert_eq!(result.clipping.intervals.len(), 1);
            let interval = &result.clipping.intervals[0];
            assert_eq!(interval.source_range, [0.0, domain.1]);
            assert!(!interval.endpoints[1].is_empty());
            for event in &interval.endpoints[1] {
                assert_eq!(event.section_parameter, domain.1);
                assert!(event.section_parameter_bound[0] <= domain.1);
                assert!(event.section_parameter_bound[1] >= domain.1);
            }
            assert_native_trace_bounds(&topo, &result, 997);
            assert_eq!(format!("{topo:?}"), before);
        }
    }
}

#[test]
fn admitted_offset_charts_report_bounds_covering_actual_native_evaluations() {
    for rational in [false, true] {
        for domains in [
            [(-8.0, 24.0), (1e6, 1e6 + 16.0)],
            [(1e6, 1e6 + 16.0), (-8.0, 24.0)],
        ] {
            let (topo, traces, _) = fixture(&[FULL], &[FULL], rational, 1.0, false, false, domains);
            let result = discover_ruled_face_section(
                &topo,
                traces.map(|trace| trace.face),
                &OperationContext::new(),
            )
            .unwrap();
            assert_native_trace_bounds(&topo, &result, 997);
            assert!(
                result
                    .clipping
                    .surface_residual_bounds
                    .into_iter()
                    .all(|b| b < 1e-7)
            );
        }
    }
}

fn translated_weighted_faces(anchor: f64, weights: [f64; 3]) -> (Topology, [FaceId; 2]) {
    let (_, surfaces) = section_and_surfaces(false, 1.0, false, [(0.0, 1.0), (0.0, 16.0)]);
    let mut topo = Topology::new();
    let faces = surfaces.map(|surface| {
        let points = surface
            .control_points()
            .iter()
            .map(|row| {
                row.iter()
                    .map(|point| Point3::new(anchor + 16.0 * point.x(), point.y(), point.z()))
                    .collect()
            })
            .collect();
        let original = NurbsSurface::new(
            2,
            1,
            surface.knots_u().to_vec(),
            surface.knots_v().to_vec(),
            points,
            weights.map(|w| vec![w, w]).to_vec(),
        )
        .unwrap();
        // Construct independent boundary curves with projectively normalized
        // weights so the fixture itself does not overflow while multiplying
        // coordinates by raw-MAX weights. Retain the original carrier below.
        let maximum = weights.into_iter().fold(0.0_f64, f64::max);
        let boundary_surface = NurbsSurface::new(
            2,
            1,
            original.knots_u().to_vec(),
            original.knots_v().to_vec(),
            original.control_points().to_vec(),
            weights.map(|w| vec![w / maximum; 2]).to_vec(),
        )
        .unwrap();
        let id = face(&mut topo, boundary_surface, &[FULL], false);
        topo.face_mut(id)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(original));
        id
    });
    (topo, faces)
}

#[test]
fn absolute_world_evaluation_roundoff_is_admitted_or_refused_truthfully() {
    for anchor in [1e8, 1e16] {
        let (topo, faces) = translated_weighted_faces(anchor, [1.0; 3]);
        let before = format!("{topo:?}");
        assert!(matches!(
            discover_ruled_face_section(&topo, faces, &OperationContext::new()),
            Err(RuledSectionError::Residual { bound }) if bound > 1e-7
        ));
        assert_eq!(format!("{topo:?}"), before);
    }
    let (topo, faces) = translated_weighted_faces(1e6, [1.0; 3]);
    let before = format!("{topo:?}");
    let result = discover_ruled_face_section(&topo, faces, &OperationContext::new()).unwrap();
    assert_native_trace_bounds(&topo, &result, 997);
    assert!(
        result
            .clipping
            .surface_residual_bounds
            .into_iter()
            .all(|b| b < 1e-7)
    );
    assert_eq!(format!("{topo:?}"), before);
}

#[test]
fn evaluator_weight_scaling_is_qualified_without_admitting_unsafe_regimes() {
    for weight in [1e-300, 1.0, 1e300] {
        let (topo, faces) = translated_weighted_faces(0.0, [weight; 3]);
        let before = format!("{topo:?}");
        let result = discover_ruled_face_section(&topo, faces, &OperationContext::new())
            .unwrap_or_else(|error| panic!("weight={weight}: {error}"));
        assert_native_trace_bounds(&topo, &result, 997);
        assert_eq!(format!("{topo:?}"), before);
    }
    for weights in [
        [f64::from_bits(1); 3],
        [f64::MIN_POSITIVE; 3],
        [f64::MAX; 3],
        [1.0, 1e8, 1.0],
    ] {
        let (topo, faces) = translated_weighted_faces(0.0, weights);
        let before = format!("{topo:?}");
        let FaceSurface::Nurbs(surface) = topo.face(faces[0]).unwrap().surface() else {
            panic!("original carrier");
        };
        assert!(
            !evaluator_roundoff(
                surface.control_points().iter().flatten().copied(),
                surface.weights().iter().flatten().copied(),
                &[surface.domain_u(), surface.domain_v()],
            )
            .is_finite()
        );
        assert!(discover_ruled_face_section(&topo, faces, &OperationContext::new()).is_err());
        assert_eq!(format!("{topo:?}"), before);
    }
}

#[test]
fn native_v_proposal_rounding_must_still_match_both_original_carriers() {
    let (_, mut surfaces) = section_and_surfaces(false, 1.0, false, [(0.0, 1.0); 2]);
    let second = &surfaces[1];
    let points = second
        .control_points()
        .iter()
        .map(|row| {
            // The exact common section is at fraction 3/16, whose native
            // coordinate 1e16+3 rounds to 1e16+4. The stored trace then lies
            // 1/16 of a ruling away from the original first carrier.
            let shift = (row[1] - row[0]) * (0.5 - 3.0 / 16.0);
            vec![row[0] + shift, row[1] + shift]
        })
        .collect();
    surfaces[1] = NurbsSurface::new(
        2,
        1,
        second.knots_u().to_vec(),
        vec![1e16, 1e16, 1e16 + 16.0, 1e16 + 16.0],
        points,
        second.weights().to_vec(),
    )
    .unwrap();
    let mut topo = Topology::new();
    let first = face(&mut topo, surfaces[0].clone(), &[FULL], false);
    let second = face(
        &mut topo,
        surfaces[1].clone(),
        &[[0.0, 1.0, 1e16, 1e16 + 16.0]],
        false,
    );
    let before = format!("{topo:?}");
    assert!(matches!(
        discover_ruled_face_section(&topo, [first, second], &OperationContext::new()),
        Err(RuledSectionError::Residual { bound }) if bound > 0.01
    ));
    assert_eq!(format!("{topo:?}"), before);
}

#[test]
fn whole_section_derivative_bounds_cover_independent_polynomial_and_rational_speeds() {
    let line = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 4.0, 0.0)],
        vec![1.0; 2],
    )
    .unwrap();
    let bound = bernstein::derivative_bound(&curve_h(&line, Point3::new(0.0, 0.0, 0.0)).unwrap());
    assert!((5.0..5.0 + 1e-12).contains(&bound));
    for rational in [false, true] {
        let (curve, _) = section_and_surfaces(rational, 1.0, false, [(0.0, 1.0); 2]);
        let bound =
            bernstein::derivative_bound(&curve_h(&curve, Point3::new(0.0, 0.0, 0.0)).unwrap());
        for k in 0..=997 {
            let fraction = f64::from(k) / 997.0;
            let speed = if rational {
                std::f64::consts::SQRT_2
                    / ((1.0 - fraction).powi(2)
                        + std::f64::consts::SQRT_2 * fraction * (1.0 - fraction)
                        + fraction.powi(2))
            } else {
                1.0_f64.hypot(2.0 * fraction)
            };
            assert!(
                bound.is_finite() && bound >= speed,
                "bound={bound} speed={speed}"
            );
        }
    }
}

#[test]
fn v_coordinates_are_discovered_instead_of_assumed_to_be_midpoints() {
    let (_, surfaces) = section_and_surfaces(false, 1.0, false, [(0.0, 1.0); 2]);
    let mut topo = Topology::new();
    let faces = std::array::from_fn(|j| {
        let surface = &surfaces[j];
        let shift = if j == 0 { 0.25 } else { -0.25 };
        let points = surface
            .control_points()
            .iter()
            .map(|row| {
                let delta = (row[1] - row[0]) * shift;
                vec![row[0] + delta, row[1] + delta]
            })
            .collect();
        face(
            &mut topo,
            NurbsSurface::new(
                2,
                1,
                surface.knots_u().to_vec(),
                vec![10.0, 10.0, 26.0, 26.0],
                points,
                surface.weights().to_vec(),
            )
            .unwrap(),
            &[[0.0, 1.0, 11.0, 25.0]],
            false,
        )
    });
    let result = discover_ruled_face_section(&topo, faces, &OperationContext::new()).unwrap();
    near(result.traces[0].v, 14.0);
    near(result.traces[1].v, 22.0);

    let mut topo = Topology::new();
    let faces = std::array::from_fn(|j| {
        let surface = &surfaces[j];
        let shift = if j == 0 { 0.25 } else { -0.25 };
        let points = surface
            .control_points()
            .iter()
            .map(|row| {
                let delta = (row[1] - row[0]) * shift;
                vec![row[0] + delta, row[1] + delta]
            })
            .collect();
        face(
            &mut topo,
            NurbsSurface::new(
                2,
                1,
                surface.knots_u().to_vec(),
                surface.knots_v().to_vec(),
                points,
                surface.weights().to_vec(),
            )
            .unwrap(),
            &[FULL],
            true,
        )
    });
    let result = discover_ruled_face_section(&topo, faces, &OperationContext::new()).unwrap();
    near(result.traces[0].v, -1.0);
    near(result.traces[1].v, 1.0);
    for k in 0..=16 {
        let f = f64::from(k) / 16.0;
        assert!((result.curve.evaluate(f) - locus(f, false)).length() < 1e-12);
    }
}

#[test]
fn false_shared_curve_agreeing_at_endpoints_and_midpoint_refuses_whole_span() {
    let knots_u = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0 / 3.0, 0.0, 0.0),
        Point3::new(2.0 / 3.0, 0.0, 1.0 / 3.0),
        Point3::new(1.0, 0.0, 1.0),
    ];
    let surfaces: [NurbsSurface; 2] = std::array::from_fn(|j| {
        let direction = if j == 0 {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 1.0)
        };
        NurbsSurface::new(
            3,
            1,
            knots_u.clone(),
            vec![-2.0, -2.0, 2.0, 2.0],
            points
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    // The difference is a multiple of t(1-t)(1-2t),
                    // zero at both ends and the midpoint, but not inside.
                    let bump = if j == 1 {
                        [0.0, 0.1, -0.1, 0.0][i]
                    } else {
                        0.0
                    };
                    let p = *p + Vec3::new(bump, 0.0, 0.0);
                    vec![p - direction * 0.5, p + direction * 0.5]
                })
                .collect(),
            vec![vec![1.0, 1.0]; 4],
        )
        .unwrap()
    });
    for u in [0.0, 0.5, 1.0] {
        assert!((surfaces[0].evaluate(u, 0.0) - surfaces[1].evaluate(u, 0.0)).length() < 1e-14);
    }
    assert!((surfaces[0].evaluate(0.25, 0.0) - surfaces[1].evaluate(0.25, 0.0)).length() > 0.02);
    // Any boundary would be premature: rejection must occur on the whole
    // source span, before the clipping component reads boundary pcurves.
    let mut topo = Topology::new();
    let faces = surfaces.map(|surface| face(&mut topo, surface, &[FULL], false));
    let before = format!("{topo:?}");
    assert!(matches!(
        discover_ruled_face_section(&topo, faces, &OperationContext::new()),
        Err(RuledSectionError::Residual { bound }) if bound > 0.02
    ));
    assert_eq!(format!("{topo:?}"), before);
}

#[test]
fn missing_authority_parallel_rulings_budget_and_cancel_publish_nothing() {
    let (mut topo, traces, _) =
        fixture(&[FULL], &[FULL], false, 1.0, false, false, [(0.0, 1.0); 2]);
    let faces = traces.map(|trace| trace.face);
    let context = OperationContext::new().with_budgets(WorkBudgets::new().with_segments(0));
    let before = format!("{topo:?}");
    assert!(matches!(
        discover_ruled_face_section(&topo, faces, &context),
        Err(RuledSectionError::WorkBudgetExceeded)
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut context = OperationContext::new();
    context.cancellation = Some(token);
    assert!(matches!(
        discover_ruled_face_section(&topo, faces, &context),
        Err(RuledSectionError::Cancelled)
    ));
    assert!(matches!(
        discover_ruled_face_section(&topo, [faces[0]; 2], &OperationContext::new()),
        Err(RuledSectionError::UnresolvedContact)
    ));
    assert_eq!(format!("{topo:?}"), before);
    let cid = topo
        .face_loop(topo.loops_of_face(faces[0]).unwrap()[0])
        .unwrap()
        .coedges()[0];
    topo.remove_coedge_pcurve(cid).unwrap();
    let before = format!("{topo:?}");
    assert!(matches!(
        discover_ruled_face_section(&topo, faces, &OperationContext::new()),
        Err(RuledSectionError::MissingAuthority)
    ));
    assert_eq!(format!("{topo:?}"), before);
}

#[test]
fn trimmed_away_section_is_explicitly_not_face_disjointness_proof() {
    let (topo, traces, _) = fixture(
        &[[0.0, 1.0, 0.25, 1.5]],
        &[FULL],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0); 2],
    );
    let result = discover_ruled_face_section(
        &topo,
        traces.map(|trace| trace.face),
        &OperationContext::new(),
    )
    .unwrap();
    assert!(result.clipping.intervals.is_empty());
    assert!((result.curve.evaluate(0.25) - locus(0.25, false)).length() < 1e-12);
}

#[test]
fn discovery_retains_the_existing_exact_boundary_work_admission() {
    let (topo, traces, _) = fixture(&[FULL], &[FULL], false, 1.0, false, false, [(0.0, 1.0); 2]);
    let faces = traces.map(|trace| trace.face);
    let at =
        |segments| OperationContext::new().with_budgets(WorkBudgets::new().with_segments(segments));
    assert!(matches!(
        discover_ruled_face_section(&topo, faces, &at(7)),
        Err(RuledSectionError::WorkBudgetExceeded)
    ));
    let result = discover_ruled_face_section(&topo, faces, &at(8)).unwrap();
    assert_eq!(result.clipping.intervals.len(), 1);
    assert_eq!(result.clipping.events.len(), 4);
    for linear in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut context = OperationContext::new();
        context.tolerance.linear = linear;
        assert!(matches!(
            discover_ruled_face_section(&topo, faces, &context),
            Err(RuledSectionError::InvalidInput)
        ));
    }
}

#[test]
fn degree_one_and_three_sources_match_independent_polynomial_loci() {
    for degree in [1, 3] {
        for scale in [1e-3, 1.0, 1e3] {
            for reverse in [false, true] {
                let mut topo = Topology::new();
                let faces = std::array::from_fn(|j| {
                    let direction = if j == 0 {
                        Vec3::new(0.0, scale, 0.0)
                    } else {
                        Vec3::new(0.0, scale, scale)
                    };
                    let domain = if j == 0 { (-4.0, 4.0) } else { (8.0, 24.0) };
                    let points = (0..=degree)
                        .map(|i| {
                            let x = f64::from(i) / f64::from(degree);
                            let z = if i == degree { 1.0 } else { 0.0 };
                            let p = Point3::new(scale * x, 0.0, scale * z);
                            vec![p - direction * 0.5, p + direction * 0.5]
                        })
                        .collect();
                    let count = usize::try_from(degree + 1).unwrap();
                    let surface = NurbsSurface::new(
                        usize::try_from(degree).unwrap(),
                        1,
                        [vec![domain.0; count], vec![domain.1; count]].concat(),
                        vec![-2.0, -2.0, 2.0, 2.0],
                        points,
                        vec![vec![1.0, 1.0]; count],
                    )
                    .unwrap();
                    let trim = if j == 0 {
                        [0.125, 0.875, -1.0, 1.0]
                    } else {
                        FULL
                    };
                    face(&mut topo, surface, &[trim], reverse)
                });
                let result =
                    discover_ruled_face_section(&topo, faces, &OperationContext::new()).unwrap();
                assert_eq!(result.curve.degree(), usize::try_from(degree).unwrap());
                assert_eq!(result.clipping.intervals.len(), 1);
                near(result.clipping.intervals[0].source_range[0], -3.0);
                near(result.clipping.intervals[0].source_range[1], 3.0);
                for k in 0..=32 {
                    let f = f64::from(k) / 32.0;
                    let expected = Point3::new(scale * f, 0.0, scale * f.powi(degree));
                    assert!((result.curve.evaluate(-4.0 + 8.0 * f) - expected).length() < 1e-7);
                }
            }
        }
    }
}
