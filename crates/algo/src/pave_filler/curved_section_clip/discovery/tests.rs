#![allow(clippy::unwrap_used, clippy::panic, clippy::float_cmp)]

use super::*;
use crate::{RuledSectionError, discover_ruled_face_section};
use remus_math::context::{CancellationToken, WorkBudgets};
use remus_math::vec::Vec3;

use super::super::tests::{face, fixture, place, section_and_surfaces};

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
    let mut context = OperationContext::new();
    context.tolerance.linear = f64::NAN;
    assert!(matches!(
        discover_ruled_face_section(&topo, faces, &context),
        Err(RuledSectionError::InvalidInput)
    ));
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
